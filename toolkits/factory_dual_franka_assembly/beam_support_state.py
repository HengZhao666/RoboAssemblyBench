"""Physical Beam support bookkeeping and measured table-load gates.

This module never writes a body pose, velocity, collision filter, or joint.
"""
from __future__ import annotations
import copy
import itertools
import json
import math
import os
import numpy as np
from toolkits.factory_dual_franka_assembly.planner_primitives import quat_rotate, pose_error


def physics_sample_token(sample):
    """Return a validated physical sample identity, never a task-counter fallback.

    Epoch changes invalidate histories after a context/view reset. Time and dt
    are mandatory so an apparently consecutive integer cannot mask bad timing.
    ``step`` remains a task label for human-readable traces only.
    """
    epoch, index = sample.get('physics_epoch'), sample.get('physics_step_index')
    try:
        valid = (sample.get('physics_stamp_valid') is True
                 and type(epoch) is int and epoch >= 0
                 and type(index) is int and index >= 0
                 and math.isfinite(sample['physics_time']) and sample['physics_time'] >= 0
                 and math.isfinite(sample['physics_dt']) and sample['physics_dt'] > 0)
    except (KeyError, TypeError, ValueError):
        valid = False
    return (epoch, index) if valid else None


def physics_samples_consecutive(previous, sample):
    """Require consecutive indices and elapsed time from the same physical clock."""
    old, new = physics_sample_token(previous or {}), physics_sample_token(sample)
    if old is None or new is None or new != (old[0], old[1] + 1):
        return False
    dt = sample['physics_dt']
    return (math.isclose(previous['physics_dt'], dt, rel_tol=1e-9, abs_tol=1e-12)
            and math.isclose(sample['physics_time'] - previous['physics_time'], dt,
                             rel_tol=1e-6, abs_tol=max(1e-10, dt * 1e-6)))


def table_support_ready(sample, minimum_force):
    return bool(sample.get('valid') and sample.get('motion_valid')
                and minimum_force <= sample['vertical_force'] <= 40.0
                and -.0005 <= sample['bottom_gap'] <= .001
                and sample['linear_speed'] <= .002 and sample['angular_speed'] <= .03)


def combine_support_surfaces(surfaces, world_corners):
    """Sum real, filtered object/surface forces in the object's footprint.

    The thick assembly collider is above the visual tabletop. Reading only the
    visual prim misses that load. Overlapping physical colliders exert separate
    forces; each configured body is sampled once, without synthesizing contact.
    """
    corners = np.asarray(world_corners)
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    under = [s for s in surfaces if np.all(np.asarray(s['lower'])[:2] <= hi[:2])
             and np.all(np.asarray(s['upper'])[:2] >= lo[:2])]
    if not under or any(not s['contact'].get('valid') or 'force_world' not in s['contact'] for s in under):
        raise ValueError('Physical support pair force unavailable')
    top = max(s['upper'][2] for s in under)
    signed_force = sum(float(s['contact']['force_world'][2]) for s in under)
    return {'valid': signed_force >= 0., 'table_top': float(top), 'bottom_gap': float(lo[2]-top),
            'vertical_force': signed_force,
            'support_surfaces': under}


def cube_world_bounds(size, usd_row_transform):
    """Compute analytic Cube bounds; Isaac authors a pre-scaled extent attribute.

    BBoxCache transforms that extent again. Transform the unscaled primitive
    corners instead, preserving USD's row-vector matrix convention.
    """
    half = float(size) / 2
    corners = np.array([(*p, 1.) for p in itertools.product([-half, half], repeat=3)])
    points = (corners @ np.asarray(usd_row_transform))[:, :3]
    return points.min(axis=0).tolist(), points.max(axis=0).tolist()


def live_body_snapshot(view):
    """Read one live body; never accept authored USD state as physical motion.

    Contact views are already initialized by the pair-force reader. Using that
    same view avoids initializing a second wrapper (and its state side effects).
    Copy all buffers before another getter can reuse them. Coordinates are world
    coordinates and angular velocity is rad/s, as in the tensor view API.
    """
    if not view.is_physics_handle_valid():
        raise ValueError('Beam motion view has no live physics handle')
    positions, orientations = view.get_world_poses(clone=True)
    position = np.array(positions, dtype=float, copy=True)
    orientation = np.array(orientations, dtype=float, copy=True)
    velocities = np.array(view.get_velocities(clone=True), dtype=float, copy=True)
    if (position.shape != (1, 3) or orientation.shape != (1, 4)
            or velocities.shape != (1, 6)
            or not all(np.all(np.isfinite(v)) for v in (position, orientation, velocities))
            or np.linalg.norm(orientation[0]) < 1e-9):
        raise ValueError('Invalid single-body Beam physics snapshot')
    return {'position': position[0].tolist(),
            'orientation': (orientation[0] / np.linalg.norm(orientation[0])).tolist(),
            'linear_velocity': velocities[0, :3].tolist(),
            'angular_velocity': velocities[0, 3:].tolist(),
            'linear_speed': float(np.linalg.norm(velocities[0, :3])),
            'angular_speed': float(np.linalg.norm(velocities[0, 3:])),
            'motion_valid': True, 'motion_source': 'live_pair_rigid_view',
            'physics_handle_valid': True, 'pose_frame': 'world'}


class BeamSupportState:
    def _attachment_for(self, object_name, robot_name=None):
        primary = self._attachments.get(object_name)
        if primary is not None and (robot_name is None or primary.get('robot_name') == robot_name):
            return primary
        return getattr(self, '_beam_shared_grasps', {}).get((object_name, robot_name))

    def _all_physical_grasps(self):
        return [*self._attachments.items(),
                *((name, state) for (name, _), state in getattr(self, '_beam_shared_grasps', {}).items())]

    def _store_physical_grasp(self, object_name, state):
        previous = self._attachments.get(object_name)
        if previous and previous['robot_name'] != state['robot_name']:
            if not (state['attach_spec'].get('allow_shared_physical_grasp')
                    and previous['attach_spec'].get('allow_shared_physical_grasp')
                    and previous['mode'] == state['mode'] == 'pure_physical_grasp'):
                raise RuntimeError('A second physical grasp requires explicit shared support')
            self._beam_shared_grasps[(object_name, state['robot_name'])] = state
        else:
            self._attachments[object_name] = state

    def _remove_physical_grasp(self, object_name, robot_name):
        """Remove only this hand; promote the other anchor without recapture."""
        shared = getattr(self, '_beam_shared_grasps', {})
        primary = self._attachments.get(object_name)
        if primary is not None and primary.get('robot_name') == robot_name:
            self._attachments.pop(object_name)
            for key, other in list(shared.items()):
                if key[0] == object_name:
                    self._attachments[object_name] = shared.pop(key)
                    break
        else:
            shared.pop((object_name, robot_name), None)

    def _beam_physics_stamp(self):
        """Read the existing clock, with an epoch for rollback or backend reset.

        This never initializes a simulation/view and never mutates physics. A
        backend-view replacement also starts an epoch even if its new clock
        happens to equal the previously sampled index and time.
        """
        epoch = int(getattr(self, '_beam_physics_epoch', 0))
        try:
            from isaacsim.core.api import SimulationContext
            context = SimulationContext.instance()
            if context is None:
                raise ValueError('No existing SimulationContext')
            if callable(getattr(context, 'is_stopped', None)) and context.is_stopped():
                raise ValueError('SimulationContext is stopped')
            from isaacsim.core.simulation_manager import SimulationManager
            getter = getattr(SimulationManager, 'get_physics_sim_view', None)
            if not callable(getter):
                raise ValueError('No existing physics-view identity reader')
            view = getter()
            if view is None:
                raise ValueError('No existing physics simulation view')
            identity_source = 'simulation_context_and_physics_view'
            raw_index = context.current_time_step_index
            index = int(raw_index)
            time = float(context.current_time)
            dt = float(context.get_physics_dt())
            if (isinstance(raw_index, (bool, np.bool_)) or raw_index != index or index < 0
                    or not math.isfinite(time) or time < 0 or not math.isfinite(dt) or dt <= 0
                    or context.current_time_step_index != raw_index):
                raise ValueError('Invalid or changing physics clock')
            previous = getattr(self, '_beam_last_physics_stamp', None)
            identity = getattr(self, '_beam_physics_identity', None)
            replaced = identity is not None and (identity[0] is not context or identity[1] is not view)
            rolled_back = previous is not None and (index < previous['index'] or time < previous['time'])
            recovered = getattr(self, '_beam_physics_clock_invalid', False)
            dt_changed = previous is not None and not math.isclose(dt, previous['dt'], rel_tol=1e-9, abs_tol=1e-12)
            if previous is not None and not (replaced or rolled_back or dt_changed):
                if index == previous['index'] and time != previous['time']:
                    raise ValueError('Physics time changed without advancing its index')
                if index > previous['index'] and time <= previous['time']:
                    raise ValueError('Physics index advanced without advancing its time')
            if replaced or rolled_back or recovered or dt_changed:
                epoch += 1
            self._beam_physics_epoch = epoch
            # Keep references so object-id reuse cannot hide a context replacement.
            self._beam_physics_identity = (context, view)
            self._beam_physics_clock_invalid = False
            stamp = {'valid': True, 'epoch': epoch, 'index': index, 'time': time, 'dt': dt,
                     'identity_source': identity_source}
            self._beam_last_physics_stamp = dict(stamp)
            return stamp
        except Exception as exc:
            self._beam_physics_clock_invalid = True
            return {'valid': False, 'epoch': epoch, 'reason': f'{type(exc).__name__}: {exc}'}

    def _beam_read_table_support_sample(self, names):
        """Collect contact and body motion between two validated clock reads."""
        from pxr import Usd, UsdGeom
        base = self._resolve_object('fabrica_beam_6')
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render', 'proxy'])
        surfaces = []
        motion_view = None
        for name in names:
            self._resolve_object(name)
            table = self._object_prims[name]
            if table.IsA(UsdGeom.Cube):
                lower, upper = cube_world_bounds(UsdGeom.Cube(table).GetSizeAttr().Get(),
                    UsdGeom.Xformable(table).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            else:
                bounds = cache.ComputeWorldBound(table).ComputeAlignedRange()
                lower, upper = list(bounds.GetMin()), list(bounds.GetMax())
            observation = self._pair_contact_observation(str(base.unwrap().prim_path), str(table.GetPath()))
            surfaces.append({'object': name, 'lower': lower, 'upper': upper, 'contact': observation})
            if motion_view is None and observation.get('valid'):
                motion_view = self._get_contact_probe(str(base.unwrap().prim_path), str(table.GetPath()))
        snapshot = live_body_snapshot(motion_view)
        if getattr(self, '_beam_exploration_enabled', False):
            # Read COM from this already initialized motion view, between the
            # same two clock reads as actor pose/contact. Never substitute an
            # actor origin or an asynchronous diagnostic record for missing COM.
            from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import _body_com_snapshot
            snapshot['native_com'] = _body_com_snapshot(motion_view,
                np.asarray([snapshot['position']]), np.asarray([snapshot['orientation']]))
            snapshot['motion_view_id'] = id(motion_view)
            snapshot['motion_body_id'] = str(base.unwrap().prim_path)
            # The contact cache normally owns it; retain a reference explicitly
            # so a later wrapper allocation cannot silently reuse this identity.
            if not hasattr(self, '_beam_exploration_views'):
                self._beam_exploration_views = {}
            self._beam_exploration_views.setdefault(id(motion_view), motion_view)
        corners = itertools.product([-.0762, .0762], [-.00889, .00889], [0., .0127])
        world_corners = [np.asarray(snapshot['position']) + quat_rotate(snapshot['orientation'], p) for p in corners]
        support = combine_support_surfaces(surfaces, world_corners)
        legacy_view = getattr(base.unwrap(), '_rigid_prim_view', None)
        return {'valid': True, **support, **snapshot,
                'legacy_velocity': self._object_velocity_metrics('fabrica_beam_6'),
                'legacy_physics_handle_valid': bool(legacy_view is not None and legacy_view.is_physics_handle_valid()),
                'body_solver_state': [
                    {'path': str(body.GetPath()),
                     'velocity_iterations': body.GetAttribute('physxRigidBody:solverVelocityIterationCount').Get(),
                     'position_iterations': body.GetAttribute('physxRigidBody:solverPositionIterationCount').Get(),
                     'stabilization_threshold': body.GetAttribute('physxRigidBody:stabilizationThreshold').Get()}
                    for body in self._iter_rigid_body_prims(self._object_prims['fabrica_beam_6'])]}

    def _beam_trace_clock(self, event):
        trace = os.environ.get('BEAM_MOTION_TRACE_PATH')
        if trace:
            with open(os.path.join(os.path.dirname(trace), 'table_physics_clock_trace.jsonl'), 'a') as stream:
                stream.write(json.dumps(event, allow_nan=False) + '\n')

    def _beam_table_observation(self, sample_context='controller_or_observation'):
        step = int(self.step_counter)
        before = self._beam_physics_stamp()
        names = None
        try:
            names = self.get_current_phase_spec().get('beam_table_support_objects', ['factory_tabletop_visual'])
            if (not isinstance(names, (list, tuple)) or not names
                    or not all(isinstance(name, str) and name for name in names)):
                raise ValueError('Invalid Beam support object collection')
            # Preserve configured probe/read order and the original first live
            # motion view. Only the cache signature is order independent.
            names = tuple(dict.fromkeys(names))
            key = ((before['epoch'], before['index'], before['time'], before['dt'], tuple(sorted(names)))
                   if before['valid'] else None)
            cached = getattr(self, '_beam_table_sample', None)
            if key is not None and key == getattr(self, '_beam_table_sample_key', None) and cached is not None:
                result = copy.deepcopy(cached)
                result.update(step=step, phase=self.phase, read_context=sample_context, cache_hit=True)
                trace_key = (key, sample_context, step)
                if getattr(self, '_beam_table_last_cache_trace', None) != trace_key:
                    self._beam_table_last_cache_trace = trace_key
                    self._beam_trace_clock({'event': 'cache_read', 'task_step_label': step,
                        'phase': self.phase, 'consumer': sample_context, 'physics_stamp': before,
                        'sample_task_step': result['sample_task_step'], 'sample_context': result['sample_context'],
                        'sample_valid': result['valid'], 'physics_stamp_valid': result['physics_stamp_valid']})
                return result
        except Exception as exc:
            key = None
            before = {**before, 'valid': False, 'reason': f'{type(exc).__name__}: {exc}'}
        sample = {'step': step, 'phase': self.phase, 'sample_task_step': step, 'sample_phase': self.phase,
                  'sample_context': sample_context, 'read_context': sample_context, 'cache_hit': False,
                  'valid': False, 'physics_stamp_valid': False, 'physics_stamp': dict(before),
                  'physics_epoch': before['epoch'], 'support_object_names': list(names or [])}
        if before['valid']:
            sample.update(physics_step_index=before['index'], physics_time=before['time'], physics_dt=before['dt'])
            try:
                sample.update(self._beam_read_table_support_sample(names))
            except Exception as exc:
                sample['error'] = f'{type(exc).__name__}: {exc}'
            after = self._beam_physics_stamp()
            sample['physics_stamp_after'] = dict(after)
            if after != before:
                sample.update(valid=False, motion_valid=False, error='Physics clock changed during Beam sample')
                key = None
            else:
                sample['physics_stamp_valid'] = True
        else:
            sample['error'] = before.get('reason', 'Invalid Beam physical clock')
        # Never expose the cached object to controllers, gates or trace callers.
        self._beam_table_sample = copy.deepcopy(sample)
        self._beam_table_sample_key = key
        self._beam_trace_clock({'event': 'sample', 'task_step_label': step, 'phase': self.phase,
            'sample_context': sample_context, 'consumer': sample_context, 'physics_stamp': before,
            'physics_stamp_after': sample.get('physics_stamp_after'),
            'sample_valid': sample['valid'], 'physics_stamp_valid': sample['physics_stamp_valid'],
            'support_object_names': sample['support_object_names'], 'error': sample.get('error')})
        trace = os.environ.get('BEAM_MOTION_TRACE_PATH')
        if trace and before.get('valid') and before['index'] % 8 == 0:
            with open(os.path.join(os.path.dirname(trace), 'table_support_trace.jsonl'), 'a') as stream:
                stream.write(json.dumps(sample, allow_nan=False) + '\n')
        return copy.deepcopy(sample)

    def _beam_table_supported(self, advance):
        sample = self._beam_table_observation(sample_context='post_physics_gate')
        key = (self.phase_index, self.phase_entry_step, tuple(sorted(sample.get('support_object_names', []))))
        state = getattr(self, '_beam_support_stability', None)
        if state is None or state['key'] != key:
            state = self._beam_support_stability = {'key': key, 'last_token': None, 'sample': None, 'count': 0}
        token = physics_sample_token(sample)
        if token is None or not sample.get('valid'):
            state.update(last_token=None, sample=None, count=0)
            return self._beam_exploration_seat(sample, False) if (
                getattr(self, '_beam_exploration_enabled', False) and self.phase == 'base_6_set_down') else False
        if token != state['last_token']:
            consecutive = physics_samples_consecutive(state['sample'], sample)
            ready = table_support_ready(sample, float(advance.get('minimum_vertical_force', 1.962)))
            state['count'] = (state['count'] if consecutive else 0) + 1 if ready else 0
            state['last_token'] = token
            state['sample'] = copy.deepcopy(sample)
            self._beam_trace_clock({'event': 'support_gate', 'task_step_label': int(self.step_counter),
                'consumer': 'post_physics_gate',
                'phase': self.phase, 'physics_epoch': token[0], 'physics_step_index': token[1],
                'physics_time': sample['physics_time'], 'physics_dt': sample['physics_dt'],
                'sample_task_step': sample.get('sample_task_step'), 'sample_context': sample.get('sample_context'),
                'cache_hit': sample.get('cache_hit'), 'ready': ready,
                'consecutive': consecutive, 'stable_count': state['count']})
        elif state['sample']['physics_time'] != sample['physics_time']:
            state.update(last_token=None, sample=None, count=0)
            return self._beam_exploration_seat(sample, False) if (
                getattr(self, '_beam_exploration_enabled', False) and self.phase == 'base_6_set_down') else False
        original_ready = state['count'] >= max(int(advance.get('stable_steps', 96)), 1)
        if getattr(self, '_beam_exploration_enabled', False) and self.phase == 'base_6_set_down':
            return self._beam_exploration_seat(sample, original_ready)
        if self._beam_continuing_final():
            pose = self._beam_phase_pose_result('fabrica_beam_6', sample)
            ready = bool(pose.get('ready') and sample.get('valid')
                and float(advance.get('minimum_vertical_force', 1.962)) <= sample['vertical_force'] <= 40.
                and -.0005 <= sample['bottom_gap'] <= .001)
            self._beam_exploration_record('final_support_gate', sample,
                {'mode': 'qualifying', 'ready': ready, 'original_ready': original_ready,
                 'formal_success': False}, pose)
            return ready
        return original_ready

    def _beam_continuing_final(self):
        return bool(getattr(self, '_beam_exploration_enabled', False)
            and getattr(self, '_beam_exploration_state', {}).get('continue_after_first_insertion')
            and self.phase == 'beam_final_unloaded_verification')

    def _beam_continuing_stability(self):
        return bool(getattr(self, '_beam_exploration_enabled', False)
            and getattr(self, '_beam_exploration_state', {}).get('continue_after_first_insertion')
            and getattr(self, '_beam_exploration_state', {}).get('anchor')
            and (self.phase.endswith('_release')
                 or self.phase == 'beam_final_unloaded_verification'))

    def _beam_phase_pose_result(self, name, sample, *, linear=.002, angular=.02):
        """Reuse the existing actor/COM windows on one stationary phase/body."""
        from toolkits.factory_dual_franka_assembly.beam_pose_stability import PoseStabilityTracker
        key = (self.phase_index, self.phase_entry_step, name, linear, angular)
        trackers = getattr(self, '_beam_phase_pose_trackers', None)
        if trackers is None:
            trackers = self._beam_phase_pose_trackers = {}
        if key not in trackers:
            trackers[key] = PoseStabilityTracker(linear_speed_limit=linear, angular_speed_limit=angular)
        tracker = trackers[key]
        com = sample.get('native_com', {})
        positions = com.get('world_positions', []) if com.get('valid') else []
        return tracker.observe(sample, com_position=positions[0] if positions else None,
            body_id=sample.get('motion_body_id'), view_id=sample.get('motion_view_id'),
            anchor_eligible=bool(sample.get('valid') and sample.get('motion_valid')))

    def _beam_object_pose_sample(self, name):
        """Copy only an already initialized contact view; never create one here."""
        if name == 'fabrica_beam_6':
            return self._beam_table_observation(sample_context='exploration_final_static')
        from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import _body_com_snapshot
        before = self._beam_physics_stamp()
        result = {'valid': False, 'motion_valid': False, 'physics_stamp_valid': False}
        try:
            if before.get('valid') is not True:
                raise ValueError('Final pose clock invalid')
            path = str(self._object_prims[name].GetPath())
            cache = getattr(self, '_beam_final_motion_views', None)
            if cache is None:
                cache = self._beam_final_motion_views = {}
            view = cache.get(path)
            if view is None:
                views = [v for (source, _), v in self._contact_probes.items()
                    if source == path and v.is_physics_handle_valid()]
                if not views:
                    raise ValueError('No existing live final-part contact view: ' + name)
                view = cache[path] = views[0]
            result.update(live_body_snapshot(view))
            result['native_com'] = _body_com_snapshot(view,
                np.asarray([result['position']]), np.asarray([result['orientation']]))
            result.update(valid=True, motion_body_id=path, motion_view_id=id(view),
                physics_stamp_valid=True, physics_epoch=before['epoch'],
                physics_step_index=before['index'], physics_time=before['time'], physics_dt=before['dt'])
            if self._beam_physics_stamp() != before:
                raise ValueError('Final pose sample changed clock while reading')
        except Exception as exc:
            result.update(valid=False, motion_valid=False, physics_stamp_valid=False,
                error=f'{type(exc).__name__}: {exc}')
        return result

    def _beam_pose_objects_static(self, object_specs, *, linear_threshold, angular_threshold):
        ready, original_ready, results = True, True, {}
        for spec in self._as_list(object_specs):
            name = self._extract_object_name(spec)
            settings = spec if isinstance(spec, dict) else {}
            linear = float(settings.get('linear_velocity_threshold', linear_threshold))
            angular = float(settings.get('angular_velocity_threshold', angular_threshold))
            raw = self._object_velocity_metrics(name, linear_threshold=linear, angular_threshold=angular)
            original_ready = bool(original_ready and raw.get('valid') and raw.get('is_static'))
            sample = self._beam_object_pose_sample(name)
            pose = self._beam_phase_pose_result(name, sample, linear=linear, angular=angular)
            ready = bool(ready and pose.get('ready'))
            results[name] = {'sample': sample, 'pose': pose, 'original': raw}
        self._beam_exploration_record('phase_objects_static', None,
            {'mode': 'qualifying', 'ready': ready, 'original_ready': original_ready,
             'formal_success': False, 'objects': results})
        return ready

    def _beam_exploration_record(self, event, sample, decision, pose_result=None):
        self._beam_exploration_last_decision = copy.deepcopy(decision)
        path = os.environ.get('BEAM_EXPLORATION_TRACE_PATH')
        if not path and os.environ.get('BEAM_MOTION_TRACE_PATH'):
            path = os.path.join(os.path.dirname(os.environ['BEAM_MOTION_TRACE_PATH']), 'exploration_trace.jsonl')
        if path:
            record = {'event': event, 'task_step_label': int(self.step_counter),
                      'phase': self.phase, 'sample': sample, 'decision': decision,
                      'pose_result': pose_result, 'formal_success': False}
            with open(path, 'a') as stream:
                stream.write(json.dumps(record, allow_nan=False) + '\n')

    def _beam_exploration_seat(self, sample, original_ready):
        from toolkits.factory_dual_franka_assembly.beam_pose_stability import PoseStabilityTracker
        from toolkits.factory_dual_franka_assembly.beam_exploration import exploration_seat_decision
        tracker = getattr(self, '_beam_exploration_pose_tracker', None)
        if tracker is None:
            tracker = self._beam_exploration_pose_tracker = PoseStabilityTracker()
        try:
            signed_force = sum(float(s['contact']['force_world'][2]) for s in sample['support_surfaces'])
            eligible = bool(sample.get('valid') and sample.get('motion_valid')
                            and math.isfinite(signed_force) and 1.962 <= signed_force <= 10.
                            and -.0005 <= sample['bottom_gap'] <= .001)
        except (KeyError, TypeError, ValueError, OverflowError):
            eligible = False
        com = sample.get('native_com', {})
        com_position = com.get('world_positions', [None])[0] if com.get('valid') else None
        pose = tracker.observe(sample, com_position=com_position,
            body_id=sample.get('motion_body_id'), view_id=sample.get('motion_view_id'),
            anchor_eligible=eligible)
        decision = exploration_seat_decision(self._beam_exploration_state, sample, self.phase,
            copy.deepcopy(getattr(self, '_continuous_grasp_loss_generation', {})), True,
            pose_result=pose, original_ready=original_ready)
        self._beam_exploration_record('seat_gate', sample, decision, pose)
        if decision['ready'] and 'native_anchor' not in self._beam_exploration_state:
            self._beam_exploration_state['native_anchor'] = {
                'com_position': list(pose['anchor']['com_position']), 'body_id': sample['motion_body_id'],
                'view_id': sample['motion_view_id']}
        if decision['mode'] == 'failure':
            self._set_terminal_state('failed', reason='beam-exploration-' + decision['reason'],
                status='failed', transition_type='failure', detail=decision)
        return bool(decision['ready'])

    def _beam_exploration_guard(self, phase, *, phase_complete=False):
        """Fail or stop before the next interaction/action; never command physics."""
        from toolkits.factory_dual_franka_assembly.beam_exploration import exploration_guard_decision
        state = self._beam_exploration_state
        if state.get('anchor') and phase.get('name') != 'base_6_set_down':
            try:
                # Entry may introduce an installed interface before its first
                # watchdog tick. Validate it before entry interactions too.
                self._beam_assembly_integrity(phase)
            except Exception as exc:
                self._set_terminal_state('failed', reason='beam-exploration-integrity-unavailable',
                    status='failed', transition_type='failure', detail={'error': f'{type(exc).__name__}: {exc}'})
                return True
            if self.failed:
                return True
        # Before seating the helper validates phase scope without reading views.
        sample = self._beam_table_observation(sample_context='exploration_guard') if state.get('anchor') else None
        decision = exploration_guard_decision(state, sample, phase.get('name'),
            copy.deepcopy(getattr(self, '_continuous_grasp_loss_generation', {})), True,
            phase_complete=phase_complete, integrity_valid=not self.failed)
        if sample is not None and decision['mode'] not in ('failure', 'seating', 'waiting'):
            try:
                native = state['native_anchor']
                com = sample['native_com']
                current = np.asarray(com['world_positions'], dtype=float)
                if (com.get('valid') is not True or current.shape != (1, 3)
                        or not np.all(np.isfinite(current))
                        or sample['motion_body_id'] != native['body_id']
                        or sample['motion_view_id'] != native['view_id']):
                    raise ValueError('Native COM or motion-view identity changed')
                if float(np.linalg.norm(current[0] - np.asarray(native['com_position']))) > .001:
                    raise ValueError('Native COM displaced beyond seated-base budget')
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                state['failure'] = 'native_pose_guard_invalid: ' + str(exc)
                decision = {'mode': 'failure', 'ready': False, 'reason': state['failure'], 'formal_success': False}
        if sample is not None or decision['mode'] in ('failure', 'complete'):
            self._beam_exploration_record('guard', sample, decision)
        if decision['mode'] == 'failure':
            self._set_terminal_state('failed', reason='beam-exploration-' + decision['reason'],
                status='failed', transition_type='failure', detail=decision)
            return True
        if decision['mode'] == 'complete':
            self._beam_exploration_complete = True
            self._set_terminal_state('exploration_stopped', reason='beam-first-insertion-exploration-complete',
                status='exploration_stopped', detail=decision)
            return True
        return False

    def _beam_joint_conditions_stable(self, advance, ready):
        """Qualify the complete final predicate on fresh, consecutive physics samples."""
        sample = self._beam_table_observation(sample_context='post_physics_joint_gate')
        key = (self.phase_index, self.phase_entry_step, json.dumps(advance, sort_keys=True))
        state = getattr(self, '_beam_joint_stability', None)
        if state is None or state['key'] != key:
            state = self._beam_joint_stability = {'key': key, 'sample': None, 'count': 0}
        token = physics_sample_token(sample)
        if token is None or not sample.get('valid'):
            state.update(sample=None, count=0)
            return False
        previous = state['sample']
        old_token = physics_sample_token(previous or {})
        if not ready:
            state.update(sample=copy.deepcopy(sample), count=0)
            return False
        if token == old_token:
            if not (sample['physics_time'] == previous['physics_time']
                    and sample['physics_dt'] == previous['physics_dt']):
                state.update(sample=None, count=0)
                return False
        else:
            state['count'] = (state['count'] if physics_samples_consecutive(previous, sample) else 0) + 1
            state['sample'] = copy.deepcopy(sample)
        return state['count'] >= max(int(advance['physics_stable_steps']), 1)

    def _beam_support_watchdog(self, phase):
        if not phase.get('beam_require_table_support'):
            return
        self._beam_assembly_integrity(phase)
        sample = self._beam_table_observation(sample_context='post_physics_watchdog')
        token = physics_sample_token(sample)
        previous = getattr(self, '_beam_support_watchdog_sample', None)
        if token is None:
            self._beam_support_missing_since = None
            self._beam_support_watchdog_sample = None
            self._set_terminal_state('failed', reason='beam-table-support-clock-invalid', status='failed',
                                     transition_type='failure', detail=sample)
            return
        if previous is not None and physics_sample_token(previous) == token:
            return
        if not physics_samples_consecutive(previous, sample):
            self._beam_support_missing_since = None
        self._beam_support_watchdog_sample = copy.deepcopy(sample)
        # Motion during insertion is not itself a support-loss event. Require
        # measured table load and bounded separation, independent of hand hold.
        valid = (sample.get('valid') and sample['vertical_force'] >= .5
                 and -.001 <= sample['bottom_gap'] <= .0015)
        if valid:
            self._beam_support_missing_since = None
        else:
            start = getattr(self, '_beam_support_missing_since', None)
            if start is None:
                self._beam_support_missing_since = token[1]
            elif token[1] - start >= 48:
                self._set_terminal_state('failed', reason='beam-table-support-lost', status='failed',
                                         transition_type='failure', detail=sample)

    def _beam_assembly_integrity(self, phase):
        """Detect a held post pulling away from the seated base during insertion.

        Pair forces are recorded as evidence; zero normal force alone is not
        declared separation for an unloaded, fitted interface.
        """
        base = self._resolve_object('fabrica_beam_6')
        base_pose = base.get_pose()
        records = []
        for pid in phase.get('beam_installed_parts', []):
            if pid == '6':
                continue
            part = self._resolve_object('fabrica_beam_' + pid)
            position, orientation = part.get_pose()
            pe, ae = pose_error(current_position=position, current_orientation=orientation,
                                target_position=base_pose[0], target_orientation=base_pose[1])
            if getattr(self, '_beam_exploration_enabled', False) and not (
                    math.isfinite(pe) and math.isfinite(ae)):
                raise ValueError('Non-finite installed-interface pose error')
            parent = base if pid in ['2','3'] else self._resolve_object('fabrica_beam_' + ('2' if pid=='0' else '3'))
            pair = self._pair_contact_observation(str(part.unwrap().prim_path), str(parent.unwrap().prim_path))
            records.append({'part':pid,'relative_position_error':float(pe),
                            'relative_orientation_error':float(ae),'interface_contact':pair})
            if pe > .003 or ae > .05:
                self._set_terminal_state('failed', reason='beam-installed-interface-displaced', status='failed',
                                         transition_type='failure', detail=records[-1])
                return
        step = int(self.step_counter)
        trace = os.environ.get('BEAM_MOTION_TRACE_PATH')
        if trace and step % 8 == 0 and getattr(self, '_beam_last_integrity_trace', None) != step:
            self._beam_last_integrity_trace = step
            with open(os.path.join(os.path.dirname(trace), 'assembly_integrity_trace.jsonl'), 'a') as f:
                f.write(json.dumps({'step':step,'phase':self.phase,'interfaces':records})+'\n')
