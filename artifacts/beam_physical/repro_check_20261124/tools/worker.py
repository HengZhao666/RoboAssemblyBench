"""Round-54 optimised worker (from the 2026-11-18 performance A/B opt_worker).

Starts from lean_worker.py (same guards, CAD gates, timing, audit, progress,
streaming capture) and adds runtime patches selected by environment flags.
The production tree is not modified by these patches; each one was bit-exact
with the compat configuration over 18,000 steps (perf_ab_20261118).

Round-54 addition (diagnostics only): a call goes to the ORIGINAL contact
trace writer, which writes a full record, when it is a failure snapshot
(force_write) or when task step >= BEAM_OPT_FULL_TRACE_FROM_STEP and
step % BEAM_OPT_FULL_TRACE_STRIDE == 0. The original writer creates the same
probes as the probe-only path (it is what the compat configuration ran).

BEAM_OPT_PROBE_TRACE=1  _write_physical_contact_trace and the round-24 grasp
    sweep only create the same contact probes, in the same order and at the
    same task steps, without building or writing records.
BEAM_OPT_FASTCOPY=1     copy.deepcopy in the task / support modules becomes a
    pickle round-trip (exact values and shared references); anything that
    cannot be pickled falls back to copy.deepcopy.
BEAM_OPT_FRAME_CACHE=1  _gripper_contact_metrics is memoised per physics
    frame and per (object, robot, attach_spec). Results with an invalid force
    probe are not stored; every task state write clears the cache.

Round-56 addition (2026-11-19 profile: 22% of wall time, ~24 calls/step):
BEAM_OPT_VELOCITY_CACHE=1  the pure read _object_velocity_metrics is memoised
    per physics frame and per (object, keyword arguments, pose-history
    fingerprint: identity, length, first and last samples with their exact
    values). It writes no state; the result depends only on the frame's body
    velocities, that history and the arguments. Every task state write clears
    the cache. Hits return an independent copy.
"""
import copy as _copy
import json
import os
import pickle
import runpy
import sys
from pathlib import Path

AB = Path(__file__).resolve().parent
OUT = Path(os.environ['BEAM_AB_OUT'])
STATS = {'fastcopy_calls': 0, 'fastcopy_fallbacks': 0, 'cache_hits': 0, 'cache_misses': 0,
         'cache_uncacheable': 0, 'cache_invalidations': 0, 'probe_trace_calls': 0, 'probe_sweep_calls': 0,
         'full_trace_calls': 0, 'failure_snapshot_calls': 0,
         'velocity_cache_hits': 0, 'velocity_cache_misses': 0, 'velocity_cache_uncacheable': 0,
         'velocity_cache_invalidations': 0}
FLAGS = {k: os.environ.get(k) == '1' for k in ('BEAM_OPT_PROBE_TRACE', 'BEAM_OPT_FASTCOPY', 'BEAM_OPT_FRAME_CACHE',
                                                'BEAM_OPT_VELOCITY_CACHE')}
FULL_TRACE_FROM = int(os.environ.get('BEAM_OPT_FULL_TRACE_FROM_STEP', '-1'))
FULL_TRACE_STRIDE = int(os.environ.get('BEAM_OPT_FULL_TRACE_STRIDE', '8'))
assert FULL_TRACE_STRIDE > 0


def fast_deepcopy(value, memo=None):
    STATS['fastcopy_calls'] += 1
    if memo is None:
        try:
            return pickle.loads(pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL))
        except Exception:
            STATS['fastcopy_fallbacks'] += 1
    return _copy.deepcopy(value, memo)


class _CopyShim:
    """Module stand-in: only deepcopy is replaced."""
    deepcopy = staticmethod(fast_deepcopy)

    def __getattr__(self, name):
        return getattr(_copy, name)


GRIPPER_LINKS = ('base_link', 'left_outer_knuckle', 'left_outer_finger', 'left_inner_finger',
                 'left_inner_knuckle', 'right_outer_knuckle', 'right_outer_finger',
                 'right_inner_finger', 'right_inner_knuckle')
SUPPORTS = ('fabrica_fixture', 'fixture_support', 'factory_tabletop_visual', 'optical_board', 'assembly_support')


def install_patches():
    from internutopia_extension.tasks import factory_dual_franka_assembly_task as task_module
    Task = task_module.FactoryDualFrankaAssemblyTask

    if FLAGS['BEAM_OPT_FASTCOPY']:
        shim = _CopyShim()
        task_module.copy = shim
        from toolkits.factory_dual_franka_assembly import beam_support_state, beam_contact_relief
        beam_support_state.copy = shim
        beam_contact_relief.copy = shim

    if FLAGS['BEAM_OPT_PROBE_TRACE']:
        original_trace = Task._write_physical_contact_trace

        def probe_only_trace(self, object_name, metrics, attach_spec, strict_contact,
                             *, force_write=False, validation_stage='contact_check'):
            # Same gate and de-duplication as the production writer.
            if not os.environ.get('BEAM_CONTACT_TRACE_PATH'):
                return
            step = int(self.step_counter)
            late = FULL_TRACE_FROM >= 0 and step >= FULL_TRACE_FROM and step % FULL_TRACE_STRIDE == 0
            key = (step, self.phase, object_name, metrics.get('robot'))
            if force_write or late:
                # The original writer applies its own de-duplication and creates
                # the same probes; it writes the full record.
                STATS['failure_snapshot_calls' if force_write else 'full_trace_calls'] += 1
                original_trace(self, object_name, metrics, attach_spec, strict_contact,
                               force_write=force_write, validation_stage=validation_stage)
            elif not force_write and getattr(self, '_last_physical_trace_key', None) == key:
                pass
            else:
                self._last_physical_trace_key = key
                STATS['probe_trace_calls'] += 1
                if os.environ.get('BEAM_SUPPORT_CONTACT_TRACE') == '1':
                    robot_name = metrics['robot']
                    target_path = self._resolve_object(object_name).unwrap().prim_path
                    body_paths = [target_path] + [metrics[s].get('prim_path') for s in ('left_finger', 'right_finger')]
                    expanded = bool(force_write or int(self.step_counter) % 8 == 0)
                    if expanded:
                        for link in GRIPPER_LINKS:
                            body = self._robot_rigid_body_by_suffix(robot_name, 'Robotiq_2F_85/' + link)
                            if body is None:
                                continue
                            body_paths.append(str(body.unwrap().prim_path))
                    for support_name in SUPPORTS:
                        if support_name not in self._object_prims:
                            continue
                        support_path = str(self._object_prims[support_name].GetPath())
                        for body_path in dict.fromkeys(body_paths):
                            if body_path:
                                self._get_contact_probe(body_path, support_path)
                    if validation_stage == 'pregrasp' and expanded:
                        for name, other in self._object_prims.items():
                            if not name.startswith('fabrica_beam_') or name == object_name:
                                continue
                            for body_path in dict.fromkeys(body_paths):
                                if body_path:
                                    self._get_contact_probe(body_path, str(other.GetPath()))
            # Round-24 grasp sweep: same phase/object/stride gate, probe creation only.
            if not self.phase.startswith(('assemble_', 'hold_', 'handover_')) or not object_name.startswith('fabrica_beam_'):
                return
            if int(self.step_counter) % 8 or getattr(self, '_short_sweep_last_step', None) == int(self.step_counter):
                return
            self._short_sweep_last_step = int(self.step_counter)
            STATS['probe_sweep_calls'] += 1
            robot_name = metrics['robot']
            for link in GRIPPER_LINKS:
                body = self._robot_rigid_body_by_suffix(robot_name, 'Robotiq_2F_85/' + link)
                if body is None:
                    continue
                path = str(body.unwrap().prim_path)
                for _, prim in self._object_prims.items():
                    self._get_contact_probe(path, str(prim.GetPath()))

        Task._write_physical_contact_trace = probe_only_trace

    if FLAGS['BEAM_OPT_FRAME_CACHE']:
        from isaacsim.core.api import SimulationContext
        original_metrics = Task._gripper_contact_metrics
        original_write = Task._trace_beam_state_write

        def frame_stamp():
            context = SimulationContext.instance()
            if context is None:
                return None
            index = context.current_time_step_index
            if index is None:
                return None
            return id(context), int(index), float(context.current_time)

        def cached_metrics(self, object_name, robot_name, attach_spec=None):
            stamp = frame_stamp()
            try:
                spec_key = pickle.dumps(attach_spec or {}, protocol=pickle.HIGHEST_PROTOCOL)
            except Exception:
                spec_key = None
            if stamp is None or spec_key is None:
                STATS['cache_uncacheable'] += 1
                return original_metrics(self, object_name, robot_name, attach_spec=attach_spec)
            if getattr(self, '_opt_metrics_stamp', None) != stamp:
                self._opt_metrics_stamp = stamp
                self._opt_metrics_cache = {}
            key = (object_name, robot_name, spec_key)
            cached = self._opt_metrics_cache.get(key)
            if cached is not None:
                STATS['cache_hits'] += 1
                return pickle.loads(cached)
            STATS['cache_misses'] += 1
            result = original_metrics(self, object_name, robot_name, attach_spec=attach_spec)
            fingers = [result.get(s) or {} for s in ('left_finger', 'right_finger')]
            valid = all(f.get('force_source') == 'disabled' or f.get('force_probe_valid') for f in fingers)
            if valid:
                try:
                    self._opt_metrics_cache[key] = pickle.dumps(result, protocol=pickle.HIGHEST_PROTOCOL)
                except Exception:
                    STATS['cache_uncacheable'] += 1
            return result

        def write_and_invalidate(self, *args, **kwargs):
            if getattr(self, '_opt_metrics_cache', None):
                STATS['cache_invalidations'] += 1
            self._opt_metrics_cache = {}
            self._opt_metrics_stamp = None
            return original_write(self, *args, **kwargs)

        Task._gripper_contact_metrics = cached_metrics
        Task._trace_beam_state_write = write_and_invalidate

    if FLAGS['BEAM_OPT_VELOCITY_CACHE']:
        from isaacsim.core.api import SimulationContext
        original_velocity = Task._object_velocity_metrics
        previous_write = Task._trace_beam_state_write

        def velocity_stamp():
            context = SimulationContext.instance()
            if context is None:
                return None
            index = context.current_time_step_index
            if index is None:
                return None
            return id(context), int(index), float(context.current_time)

        def sample_key(sample):
            step, position, orientation = sample
            return (int(step), tuple(float(x) for x in position), tuple(float(x) for x in orientation))

        def cached_velocity(self, object_name, **kwargs):
            stamp = velocity_stamp()
            try:
                history = self._object_pose_history.get(object_name)
                history_key = (None if history is None else
                               (id(history), len(history)) + ((sample_key(history[0]), sample_key(history[-1]))
                                                              if len(history) else ()))
                key = (object_name, tuple(sorted((k, float(v)) for k, v in kwargs.items())), history_key)
            except Exception:
                key = None
            if stamp is None or key is None:
                STATS['velocity_cache_uncacheable'] += 1
                return original_velocity(self, object_name, **kwargs)
            if getattr(self, '_opt_velocity_stamp', None) != stamp:
                self._opt_velocity_stamp = stamp
                self._opt_velocity_cache = {}
            cached = self._opt_velocity_cache.get(key)
            if cached is not None:
                STATS['velocity_cache_hits'] += 1
                return pickle.loads(cached)
            STATS['velocity_cache_misses'] += 1
            result = original_velocity(self, object_name, **kwargs)
            if result.get('valid'):
                try:
                    self._opt_velocity_cache[key] = pickle.dumps(result, protocol=pickle.HIGHEST_PROTOCOL)
                except Exception:
                    STATS['velocity_cache_uncacheable'] += 1
            return result

        def write_and_invalidate_velocity(self, *args, **kwargs):
            if getattr(self, '_opt_velocity_cache', None):
                STATS['velocity_cache_invalidations'] += 1
            self._opt_velocity_cache = {}
            self._opt_velocity_stamp = None
            return previous_write(self, *args, **kwargs)

        Task._object_velocity_metrics = cached_velocity
        Task._trace_beam_state_write = write_and_invalidate_velocity


def dump_stats():
    (OUT / 'opt_stats.json').write_text(json.dumps(dict(flags=FLAGS, **STATS), indent=2) + '\n')


from toolkits.factory_dual_franka_assembly import generate_demos as demo

previous_build = demo._build_env


def build(*args, **kwargs):
    env = previous_build(*args, **kwargs)
    install_patches()
    original_close = env.close

    def close(*a, **k):
        dump_stats()
        return original_close(*a, **k)

    env.close = close
    return env


demo._build_env = build
try:
    runpy.run_path(str(AB / 'lean_worker.py'), run_name='__main__')
finally:
    dump_stats()
