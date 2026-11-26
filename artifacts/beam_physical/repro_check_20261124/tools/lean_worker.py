"""Lean Beam worker for the performance A/B (2026-11-18).

Same as round-53 tools/worker.py + round24 full_assembly_worker.py except:
no lossless trace store, no grasp-sweep contact probes, no gripper-shell USD
trace. Kept: task config snapshot, collision-observer getter contract,
joint/close CAD guards (they terminate the task), phase timing, state-write
audit, progress, resource stop request, streaming capture.
"""
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path('/home/user/beam_integration_20261010_upstream6f57dae')
SOURCE = ROOT / 'candidate_repository'
OUT = Path(os.environ['BEAM_AB_OUT'])
sys.path.insert(0, str(SOURCE))
from toolkits.factory_dual_franka_assembly import generate_demos as demo

callbacks = []


def run_callbacks():
    errors = []
    while callbacks:
        name, callback = callbacks.pop(0)
        try:
            callback()
        except Exception as exc:
            errors.append(dict(observer=name, error=repr(exc)))
    (OUT / 'passive_observer_close.json').write_text(json.dumps(dict(passed=not errors, errors=errors)) + '\n')


original_build = demo._build_env


def build(*args, **kwargs):
    configs = kwargs.get('task_configs', args[0] if args else [])
    demo._write_json(OUT / 'task_config_snapshot.json', {'task_configs': [c.model_dump() for c in configs]})
    env = original_build(*args, **kwargs)
    original_close = env.close

    def close(*a, **k):
        run_callbacks()
        return original_close(*a, **k)

    env.close = close
    from internutopia_extension.tasks.factory_dual_franka_assembly_task import FactoryDualFrankaAssemblyTask as Task
    from toolkits.factory_dual_franka_assembly.demo_policy import DualFrankaAssemblyDemoPolicy
    from joint_cad_guard import JointCADGuard
    from close_cad_guard import CloseCADGuard
    from phase_timing import PhaseTiming
    from state_write_audit import StateWriteAudit

    # Collision-observer getter contract (round24), unchanged logic, no sweep/shell traces.
    original_getter = Task.get_tracked_object_states
    guard_path = OUT / 'collision_observer_getter_contract.json'
    contract = {'enabled': True, 'all_getter_calls': 0, 'observer_getter_calls': 0, 'held_phases_observed': [],
                'scope': 'Runtime native contract; reject collision integration qualification getters before invocation'}

    def save_contract():
        guard_path.write_text(json.dumps(contract, indent=2) + '\n')

    save_contract()

    def qualified_getter(self):
        contract['all_getter_calls'] += 1
        frame = sys._getframe(1)
        while frame is not None:
            filename = frame.f_code.co_filename
            if '/toolkits/constraint_checking/integration/' in filename:
                contract['observer_getter_calls'] += 1
                contract['last_violation'] = {'step': int(self.step_counter), 'phase': self.phase, 'caller': filename}
                save_contract()
                raise RuntimeError('Collision observer attempted grasp qualification state getter')
            frame = frame.f_back
        if self._attachments and self.phase not in contract['held_phases_observed']:
            contract['held_phases_observed'].append(self.phase)
            save_contract()
        return original_getter(self)

    Task.get_tracked_object_states = qualified_getter

    geometry_root = ROOT / 'grasp_repairs_20261012_20rounds/batch34_38_20261111'
    os.environ['BEAM_ROUTE_SOURCE'] = str(SOURCE)
    joint = JointCADGuard(geometry_root / 'joint_geometry/scene_geometry.json', geometry_root / 'geometry_tools', OUT)
    joint.install(DualFrankaAssemblyDemoPolicy)
    close_guard = CloseCADGuard(geometry_root / 'joint_geometry/scene_geometry.json', geometry_root / 'geometry_tools', OUT)
    close_guard.install(DualFrankaAssemblyDemoPolicy)
    timing = PhaseTiming(OUT)
    timing.install(env, DualFrankaAssemblyDemoPolicy)
    audit = StateWriteAudit(OUT)
    audit.install(Task)
    callbacks.extend([('joint_CAD', lambda: joint.flush(final=True)),
                      ('close_CAD', lambda: close_guard.flush(final=True)),
                      ('phase_timing', lambda: timing.flush(final=True)),
                      ('state_write_counts', audit.close)])
    original_update = Task._update_task_state
    last = [None, -1]
    started = time.monotonic()

    def update(task):
        audit.begin_execution(task)
        result = original_update(task)
        if (OUT / 'resource_stop_request.json').exists() and not task.failed and not task.success:
            task._set_terminal_state('failed', reason='beam-resource-stop-requested', status='failed',
                                     transition_type='failure',
                                     detail={'request': str(OUT / 'resource_stop_request.json')})
        stamp = task._beam_physics_stamp()
        native_index = int(stamp.get('index') or 0)
        if task.phase != last[0] or native_index - last[1] >= 240 or task.failed or task.success:
            support = getattr(task, '_beam_support_stability', None) or {}
            jstab = getattr(task, '_beam_joint_stability', None) or {}
            row = dict(task_step=int(task.step_counter), phase=task.phase,
                       phase_index=int(task.phase_index), phase_step=int(task.phase_step_counter),
                       physics_stamp=stamp, failed=bool(task.failed), success=bool(task.success),
                       elapsed_wall_seconds=time.monotonic() - started,
                       supported_count=support.get('count'), legacy_supported_max=support.get('legacy_max_count'),
                       terminal_count=jstab.get('count'), legacy_terminal_max=jstab.get('legacy_max_count'),
                       staged_motion=getattr(task, '_beam_staged_motion_latest', None))
            target = OUT / 'progress.json'
            tmp = target.with_suffix('.json.tmp')
            tmp.write_text(json.dumps(row, indent=2) + '\n')
            tmp.replace(target)
            with (OUT / 'progress_history.jsonl').open('a') as history:
                history.write(json.dumps(row) + '\n')
            last[:] = [task.phase, native_index]
        return result

    Task._update_task_state = update
    return env


demo._build_env = build
if os.environ.get('BEAM_STREAMING_CAPTURE') == '1':
    from physical_stream_capture import install as install_capture
    install_capture(demo, OUT)
try:
    demo.main()
finally:
    run_callbacks()
