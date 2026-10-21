"""Read-only Beam motion snapshots at runner boundaries.

Enable with BEAM_PHYSICS_DIAGNOSTICS=1 and set
BEAM_PHYSICS_DIAGNOSTICS_PATH to a JSONL file. Without an explicit file, the
directory of BEAM_MOTION_TRACE_PATH is used. Defaults sample every physical
step of base_6_set_down. PHASES, START_STEP, END_STEP, STRIDE and OBJECTS, with
the same BEAM_PHYSICS_DIAGNOSTICS_ prefix, can restrict the capture further.

No wrapper, contact probe or physics handle is created here. Missing live
handles are errors in the evidence, never replaced with authored USD poses.
The module does not call task observations, task completion or any setter.
"""
from __future__ import annotations

import json
import os
import traceback
import weakref

import numpy as np


_PREFIX = 'BEAM_PHYSICS_DIAGNOSTICS_'
_BOUNDARIES = frozenset(('before_action', 'after_action', 'after_physics', 'after_observation'))
_PATH_TASKS = {}


def _error(exc):
    return f'{type(exc).__name__}: {exc}'


def _array(value, finite=True):
    """Detach a getter's entire buffer before invoking the next getter."""
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    result = np.array(value, dtype=float, copy=True)
    if finite and not np.all(np.isfinite(result)):
        raise ValueError('Non-finite physical buffer')
    return result


def _unwrap(wrapper):
    unwrap = getattr(wrapper, 'unwrap', None)
    return unwrap() if callable(unwrap) else wrapper


def _view(wrapper, kind='_rigid_prim_view'):
    """Only inspect an existing wrapper; never initialize a new view."""
    if wrapper is None:
        return None
    candidate = getattr(wrapper, kind, None)
    if candidate is None:
        unwrapped = _unwrap(wrapper)
        candidate = getattr(unwrapped, kind, None)
    # A cached contact probe is already a view, rather than a wrapper.
    if candidate is None and callable(getattr(wrapper, 'is_physics_handle_valid', None)):
        candidate = wrapper
    return candidate


def _require_live(view):
    if view is None:
        raise ValueError('No cached physical view')
    valid = getattr(view, 'is_physics_handle_valid', None)
    if not callable(valid) or not valid():
        raise ValueError('Cached view has no valid live physics handle')


def _body_snapshot(view):
    _require_live(view)
    positions, orientations = view.get_world_poses(clone=True)
    positions = _array(positions)
    orientations = _array(orientations)
    velocities = _array(view.get_velocities(clone=True))
    if (positions.ndim != 2 or positions.shape[1] != 3 or positions.shape[0] < 1
            or orientations.shape != (len(positions), 4)
            or velocities.shape != (len(positions), 6)
            or np.any(np.abs(np.linalg.norm(orientations, axis=1) - 1.) > .001)):
        raise ValueError('Invalid physical pose/velocity shape or quaternion')
    result = {'valid': True, 'physics_handle_valid': True,
            'view_id': id(view), 'view_type': type(view).__name__,
            'pose_frame': 'world', 'orientation_order': 'wxyz',
            'velocity_frame': 'world', 'velocity_reference': 'center_of_mass',
            'positions': positions.tolist(), 'orientations': orientations.tolist(),
            'linear_velocities': velocities[:, :3].tolist(),
            'angular_velocities': velocities[:, 3:].tolist()}
    result['center_of_mass'] = _body_com_snapshot(view, positions, orientations)
    return result


def _body_com_snapshot(view, positions, orientations):
    """Read COM offsets from the same view for pose/velocity comparisons.

    Actor origins and COM positions differ for these assets. The installed
    RigidPrim getter converts its COM quaternions to wxyz and may retain a
    per-body singleton dimension. A missing COM never substitutes the actor
    origin or invalidates otherwise available raw motion evidence.
    """
    result = {'source': 'same_live_physical_view.get_coms',
              'position_frame': 'world', 'orientation_order': 'wxyz'}
    try:
        local_positions, local_orientations = view.get_coms(clone=True)
        local_positions = _array(local_positions)
        local_orientations = _array(local_orientations)
        count = len(positions)
        if local_positions.shape == (count, 1, 3):
            local_positions = local_positions[:, 0, :]
        if local_orientations.shape == (count, 1, 4):
            local_orientations = local_orientations[:, 0, :]
        if (local_positions.shape != positions.shape
                or local_orientations.shape != orientations.shape
                or np.any(np.abs(np.linalg.norm(local_orientations, axis=1) - 1.) > .001)):
            raise ValueError('Invalid live COM shape or quaternion')
        actor = orientations / np.linalg.norm(orientations, axis=1, keepdims=True)
        local = local_orientations / np.linalg.norm(local_orientations, axis=1, keepdims=True)
        rotated = (local_positions + 2. * np.cross(actor[:, 1:],
                   np.cross(actor[:, 1:], local_positions) + actor[:, :1] * local_positions))
        world_orientations = np.concatenate((
            actor[:, :1] * local[:, :1] - np.sum(actor[:, 1:] * local[:, 1:], axis=1, keepdims=True),
            actor[:, :1] * local[:, 1:] + local[:, :1] * actor[:, 1:]
            + np.cross(actor[:, 1:], local[:, 1:])), axis=1)
        result.update(valid=True, local_positions=local_positions.tolist(),
                      local_orientations=local_orientations.tolist(),
                      world_positions=(positions + rotated).tolist(),
                      world_orientations=world_orientations.tolist())
    except Exception as exc:
        result.update(valid=False, error=_error(exc))
    return result


def _native_scene_gravity():
    """Read the existing native scene, never the authored context sentinel."""
    result = {'source': 'existing_SimulationManager.physics_sim_view.get_gravity',
              'frame': 'world', 'units': 'm/s^2'}
    try:
        from isaacsim.core.simulation_manager import SimulationManager
        view = SimulationManager.get_physics_sim_view()
        if view is None:
            raise ValueError('No existing native physics simulation view')
        gravity = _array(view.get_gravity())
        if gravity.shape != (3,):
            raise ValueError(f'Invalid native gravity shape {gravity.shape}')
        result.update(valid=True, value=gravity.tolist())
    except Exception as exc:
        result.update(valid=False, error=_error(exc))
    return result


def _pair_load_snapshot(probe, body, other_body, selected_body, dt):
    """Keep signed normal and friction loads separately, without double counting.

    Only existing cached views are read; an unfiltered net tensor is not added
    to filtered loads. Force direction is on the sensor body; reverse views are
    explicitly negated for the selected Beam body. A zero normal tensor still
    requires a friction read, so the diagnostic does not discard that evidence.
    """
    result = {'body': body, 'other_body': other_body, 'selected_body': selected_body,
              'source': 'existing_cached_pair_probe', 'force_frame': 'world',
              'force_direction': 'on_sensor_body', 'physics_dt': dt, 'errors': {}}
    sign = 1. if body == selected_body else -1.
    try:
        _require_live(probe)
        normal = _array(probe.get_contact_force_matrix(clone=True, dt=dt))
        if normal.ndim != 3 or normal.shape[-1] != 3 or normal.size == 0:
            raise ValueError('Invalid pair normal tensor shape')
        normal_sum = normal.reshape(-1, 3).sum(axis=0)
        result.update(normal_valid=True, normal_force_world=normal_sum.tolist(),
                      normal_force_on_selected_world=(sign * normal_sum).tolist(),
                      normal_tensor_shape=list(normal.shape))
    except Exception as exc:
        result['normal_valid'] = False
        result['errors']['normal'] = _error(exc)
    try:
        _require_live(probe)
        forces, points, counts, starts = probe.get_friction_data(clone=True, dt=dt)
        # Unused padding need not be finite. Validate only active buffer slices.
        forces = _array(forces, finite=False)
        points = _array(points, finite=False)
        counts, starts = _array(counts), _array(starts)
        if (forces.ndim != 2 or forces.shape[1] != 3 or points.shape != forces.shape
                or counts.ndim != 2 or starts.shape != counts.shape
                or np.any(counts != np.floor(counts)) or np.any(starts != np.floor(starts))):
            raise ValueError('Invalid friction buffer shape or integer ranges')
        active = []
        for start, count in zip(starts.reshape(-1), counts.reshape(-1)):
            start, count = int(start), int(count)
            if count == 0:
                # The SDK defines starts only for a nonempty range. An unused
                # uint32 sentinel is not an active out-of-bounds contact.
                continue
            if start < 0 or count < 0 or start + count > len(forces):
                raise ValueError('Invalid friction buffer range')
            active.extend(range(start, start + count))
        if len(set(active)) != len(active):
            raise ValueError('Overlapping friction buffer ranges')
        if not np.all(np.isfinite(forces[active])) or not np.all(np.isfinite(points[active])):
            raise ValueError('Non-finite active friction data')
        friction_sum = forces[active].sum(axis=0) if active else np.zeros(3)
        result.update(friction_valid=True, friction_force_world=friction_sum.tolist(),
                      friction_force_on_selected_world=(sign * friction_sum).tolist(),
                      friction_anchor_count=len(active), friction_buffer_capacity=len(forces),
                      friction_at_capacity=len(active) >= len(forces))
    except Exception as exc:
        result['friction_valid'] = False
        result['errors']['friction'] = _error(exc)
    if result.get('normal_valid') and result.get('friction_valid'):
        result['total_force_on_selected_world'] = (
            np.asarray(result['normal_force_on_selected_world'])
            + np.asarray(result['friction_force_on_selected_world'])).tolist()
    result['valid'] = not result['errors']
    return result



def _continuing_unfiltered_scope(task):
    state = getattr(task, '_beam_exploration_state', {})
    return bool(state.get('continue_after_first_insertion') and state.get('anchor'))


def _unfiltered_phase_selected(task, phase):
    if os.environ.get('BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED') != '1':
        return False
    return (phase == 'assemble_00_part_3_assembly_clearance'
            or (_continuing_unfiltered_scope(task)
                and (phase.startswith(('assemble_', 'hold_', 'handover_'))
                     or phase == 'beam_final_unloaded_verification')))


def _unfiltered_contact_snapshot(probe, sensor_body, dt):
    """Read the existing sensor API separately from the pair ledger.

    Never negate this all-contact value onto another body or add it to filtered
    loads. Its unavailable fields do not invalidate independently valid pairs.
    """
    result = {'sensor_body': sensor_body, 'force_frame': 'world', 'physics_dt': dt,
              'source': 'existing_cached_probe.get_net_contact_forces',
              'scope': 'unfiltered_sensor_api; component coverage requires calibration'}
    try:
        _require_live(probe)
        forces = _array(probe.get_net_contact_forces(clone=True, dt=dt))
        if forces.ndim != 2 or forces.shape[1] != 3 or forces.size == 0:
            raise ValueError('Invalid unfiltered contact force tensor shape')
        result.update(valid=True, tensor_shape=list(forces.shape),
                      force_on_sensor_world=forces.sum(axis=0).tolist())
    except Exception as exc:
        result.update(valid=False, error=_error(exc))
    return result


def _body_metadata(view):
    result = {'source': 'same_live_physical_view', 'errors': {}}
    for field, getter in (('masses', 'get_masses'), ('inertias', 'get_inertias')):
        try:
            result[field] = _array(getattr(view, getter)(clone=True)).tolist()
        except Exception as exc:
            result['errors'][field] = _error(exc)
    try:
        positions, orientations = view.get_coms(clone=True)
        result['com_local_positions'] = _array(positions).tolist()
        result['com_local_orientations'] = _array(orientations).tolist()
        result['com_orientation_order'] = 'wxyz'
    except Exception as exc:
        result['errors']['center_of_mass'] = _error(exc)
    try:
        # Installed rigid/articulation APIs do not accept clone here.
        result['sleep_thresholds'] = _sleep_thresholds(view)
    except Exception as exc:
        result['errors']['sleep_thresholds'] = _error(exc)
    result['valid'] = not result['errors']
    return result


def _sleep_thresholds(view):
    values = _array(view.get_sleep_thresholds())
    if values.size == 0 or np.any(values < 0):
        raise ValueError('Invalid live sleep thresholds')
    return values.tolist()


def _joint_metadata(articulation):
    result = {'source': 'same_live_articulation_view', 'errors': {}}
    try:
        view = _view(articulation, '_articulation_view')
        _require_live(view)
        result['sleep_thresholds'] = _sleep_thresholds(view)
    except Exception as exc:
        result['errors']['sleep_thresholds'] = _error(exc)
    result['valid'] = not result['errors']
    return result


def _joints_snapshot(articulation):
    view = _view(articulation, '_articulation_view')
    _require_live(view)
    positions = _array(view.get_joint_positions(clone=True))
    velocities = _array(view.get_joint_velocities(clone=True))
    if (positions.ndim != 2 or positions.size == 0 or velocities.shape != positions.shape):
        raise ValueError('Invalid full articulation q/qdot buffers')
    result = {'valid': True, 'physics_handle_valid': True, 'view_id': id(view),
            'joint_positions': positions.tolist(), 'joint_velocities': velocities.tolist(),
            'native_command_targets': _joint_command_targets(view, positions.shape)}
    try:
        names = list(articulation.dof_names)
        if len(names) != positions.shape[1] or len(set(names)) != len(names):
            raise ValueError('Articulation joint names do not uniquely match full joint buffers')
        result['joint_names'] = [str(name) for name in names]
    except Exception as exc:
        result['joint_names_error'] = _error(exc)
    return result


def _joint_command_targets(view, shape):
    """Copy targets from the existing native backend, never from last_action.

    A completed SDK submission does not prove physical tracking. These buffers
    allow comparison of the published target with the backend command state.
    Missing getters are unavailable evidence; no view or target is created.
    """
    result = {'source': 'existing_articulation_native_tensor_view', 'errors': {}}
    native = getattr(view, '_physics_view', None)
    for field, getter in (('position_targets', 'get_dof_position_targets'),
                          ('velocity_targets', 'get_dof_velocity_targets'),
                          ('actuation_forces', 'get_dof_actuation_forces')):
        try:
            values = _array(getattr(native, getter)())
            if values.shape != shape:
                raise ValueError(f'Native target shape {values.shape} differs from joints {shape}')
            result[field] = values.tolist()
        except Exception as exc:
            result['errors'][field] = _error(exc)
    result['valid'] = not result['errors']
    return result


def _clock(context):
    if context is None:
        # Import only for an explicitly enabled capture; this is not initialization.
        from isaacsim.core.api import SimulationContext
        context = SimulationContext.instance()
    if context is None:
        raise ValueError('No existing SimulationContext')
    timing = {'physics_step_index': int(context.current_time_step_index),
              'physics_time': float(context.current_time),
              'physics_dt': float(context.get_physics_dt())}
    if (timing['physics_step_index'] < 0 or timing['physics_time'] < 0
            or timing['physics_dt'] <= 0
            or not np.isfinite(timing['physics_time']) or not np.isfinite(timing['physics_dt'])):
        raise ValueError('Invalid physics clock values')
    return context, timing


def _stabilization(context):
    """Read the global setting without calling any enable/disable method."""
    try:
        physics = context.get_physics_context()
        for method in ('is_stabilization_enabled', 'is_stablization_enabled',
                       'get_stabilization_enabled', 'get_enable_stabilization',
                       'get_stablization_enabled', 'get_enable_stablization'):
            read = getattr(physics, method, None)
            if callable(read):
                value = read()
                if value is None:
                    raise ValueError('Stabilization getter returned no value')
                return {'valid': True, 'enabled': bool(value), 'source': method}
        scene = getattr(physics, '_physics_scene', None)
        if scene is None:
            raise ValueError('Global stabilization has no existing readback source')
        prim = scene.GetPrim() if callable(getattr(scene, 'GetPrim', None)) else scene
        value = prim.GetAttribute('physxScene:enableStabilization').Get()
        if value is None:
            raise ValueError('Global stabilization attribute has no readable value')
        return {'valid': True, 'enabled': bool(value),
                'source': 'existing_scene_attribute:physxScene:enableStabilization'}
    except Exception as exc:
        return {'valid': False, 'error': _error(exc)}


def _global_physics_settings(context):
    result = {}
    for field, getter in (('solver_type', 'get_solver_type'),
                          ('gpu_dynamics_enabled', 'is_gpu_dynamics_enabled')):
        try:
            value = getattr(context.get_physics_context(), getter)()
            if value is None:
                raise ValueError(f'{getter} returned no value')
            result[field] = {'valid': True, 'source': getter,
                             'value': bool(value) if field == 'gpu_dynamics_enabled' else str(value)}
        except Exception as exc:
            result[field] = {'valid': False, 'source': getter, 'error': _error(exc)}
    result['external_forces_every_iteration'] = _external_forces_readback(context)
    return result


def _external_forces_readback(context):
    """Read an existing scene attribute, not a native active-solver flag.

    No schema Apply/Create, simulation view, or state setter is used. The USD
    scene value must be compared with startup configuration before inferring
    that the running solver consumed it.
    """
    name = 'physxScene:enableExternalForcesEveryIteration'
    result = {'attribute': name, 'scope': 'existing_scene_attribute'}
    try:
        physics = context.get_physics_context()
        api = getattr(physics, '_physx_scene_api', None)
        read = getattr(api, 'GetEnableExternalForcesEveryIterationAttr', None)
        if callable(read):
            result['source'] = 'existing_physx_scene_api.GetEnableExternalForcesEveryIterationAttr'
            attribute = read()
        else:
            result['source'] = 'existing_scene_attribute:' + name
            scene = getattr(physics, '_physics_scene', None)
            if scene is None:
                raise ValueError('No existing physics scene for external-forces readback')
            prim = scene.GetPrim() if callable(getattr(scene, 'GetPrim', None)) else scene
            attribute = prim.GetAttribute(name)
        value = attribute.Get()
        if not isinstance(value, (bool, np.bool_)):
            raise ValueError('External-forces scene attribute has no valid boolean value')
        return {**result, 'valid': True, 'value': bool(value)}
    except Exception as exc:
        return {**result, 'valid': False, 'error': _error(exc)}


def _is_beam(task):
    for field in ('_resolved_objects', '_object_metadata_map', 'target_poses'):
        if any(str(name).startswith('fabrica_beam_') for name in getattr(task, field, {})):
            return True
    return any(spec.get('beam_coordinated') for spec in getattr(task, 'phase_specs', []))


def _safe_arguments(value):
    if isinstance(value, dict):
        return {str(key): _safe_arguments(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe_arguments(item) for item in value]
    if isinstance(value, np.ndarray):
        return _safe_arguments(value.tolist())
    if isinstance(value, np.generic):
        return _safe_arguments(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return {'invalid_numeric': str(value)}
    if value is None or isinstance(value, (str, bool, int, float)):
        return value
    return {'unserialized_type': type(value).__name__}


class BeamPhysicsDiagnostics:
    def __init__(self):
        self.path = os.environ.get(_PREFIX + 'PATH')
        if not self.path:
            motion_path = os.environ.get('BEAM_MOTION_TRACE_PATH')
            if not motion_path:
                raise ValueError('Enabled Beam physics diagnostics require an output path')
            self.path = os.path.join(os.path.dirname(motion_path), 'physics_boundary_trace.jsonl')
        self.phases = set(os.environ.get(_PREFIX + 'PHASES', 'base_6_set_down').split(','))
        self.objects = tuple(os.environ.get(_PREFIX + 'OBJECTS', 'fabrica_beam_6').split(','))
        self.start = int(os.environ.get(_PREFIX + 'START_STEP', '0'))
        self.end = int(os.environ.get(_PREFIX + 'END_STEP', '-1'))
        self.stride = int(os.environ.get(_PREFIX + 'STRIDE', '1'))
        if self.stride < 1 or self.start < 0 or (self.end >= 0 and self.end < self.start):
            raise ValueError('Invalid Beam diagnostics sampling window')
        self.metadata_seen = set()
        self.joint_metadata_seen = set()
        self.global_metadata_seen = False
        self.cycle_step = None
        self.cycle_phase = None
        self.cycle_handoff = False
        self.sequence = 0

    @staticmethod
    def _handoff_capture_cycle(task, step):
        if getattr(task, '_beam_assisted_enabled', False) is not True:
            return False
        phases = getattr(task, 'phase_specs', ())
        index = getattr(task, 'phase_index', None)
        entry = getattr(task, 'phase_entry_step', None)
        if (type(index) is not int or not 0 <= index < len(phases)
                or type(entry) is not int):
            return False
        phase = phases[index]
        skill = phase.get('local_skill') or {}
        if skill.get('beam_assisted_transition_settle'):
            return bool(phase.get('name') == str(getattr(task, 'phase', '')) and step >= entry)
        ticks = (24 if skill.get('beam_assisted_phase_handoff_ticks') is not None
                 else 96 if skill.get('beam_assisted_alignment_hold')
                 and phase.get('name', '').endswith('_insert_00') else -1)
        return bool(phase.get('name') == str(getattr(task, 'phase', ''))
                    and 0 <= step-entry <= ticks)

    def selected(self, task, step, phase=None, *, handoff_cycle=False):
        phase = str(getattr(task, 'phase', '')) if phase is None else phase
        return (('*' in self.phases or phase in self.phases)
                and step >= self.start and (self.end < 0 or step <= self.end)
                and ((step - self.start) % self.stride == 0 or handoff_cycle))

    def write(self, record):
        record['sequence'] = self.sequence
        self.sequence += 1
        # Requested output failures must remain visible, rather than silently skip evidence.
        with open(self.path, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')
        return record

    def body(self, label, wrapper, path=None, task=None):
        result = {'name': label, 'path': path}
        try:
            view = _view(wrapper)
            source = 'cached_rigid_wrapper'
            # Use an existing pair view when available, without constructing a probe.
            if task is not None and path:
                for key, probe in getattr(task, '_contact_probes', {}).items():
                    if isinstance(key, tuple) and key[0] == path:
                        candidate = _view(probe)
                        if candidate is not None and candidate.is_physics_handle_valid():
                            view, source = candidate, 'cached_pair_probe'
                            break
            result.update(_body_snapshot(view), source=source)
            metadata_key = (label, id(view))
            if metadata_key not in self.metadata_seen:
                result['metadata'] = _body_metadata(view)
                self.metadata_seen.add(metadata_key)
        except Exception as exc:
            result.update(valid=False, error=_error(exc))
        return result

    def sample(self, task, boundary, context, cycle_step):
        if boundary not in _BOUNDARIES:
            raise ValueError(f'Unsupported physics boundary: {boundary}')
        task_step = int(getattr(task, 'step_counter', 0))
        cycle_step = task_step if cycle_step is None else int(cycle_step)
        if boundary == 'before_action' or self.cycle_step != cycle_step:
            self.cycle_phase = str(getattr(task, 'phase', ''))
            self.cycle_handoff = self._handoff_capture_cycle(task, cycle_step)
        self.cycle_step = cycle_step
        if not self.selected(task, cycle_step, self.cycle_phase,
                             handoff_cycle=self.cycle_handoff):
            return None
        record = {'kind': 'boundary', 'boundary': boundary, 'task_step': task_step,
                  'cycle_step': cycle_step, 'cycle_phase': self.cycle_phase,
                  'phase': str(getattr(task, 'phase', '')),
                  'task_name': str(getattr(task, 'name', '')), 'bodies': [], 'robots': {}}
        try:
            context, timing = _clock(context)
            record.update(timing, clock_valid=True)
        except Exception as exc:
            record.update(clock_valid=False, clock_error=_error(exc))
        record['native_scene_gravity'] = _native_scene_gravity()
        if not self.global_metadata_seen:
            record['global_stabilization'] = _stabilization(context)
            record['global_physics_settings'] = _global_physics_settings(context)
            self.global_metadata_seen = True
        record['task_constraints'] = {
            'locked_targets': _safe_arguments(getattr(task, '_locked_targets', {})),
            'lock_pin_pose': _safe_arguments(getattr(task, '_lock_pin_pose', {})),
            'attachment_joint_paths': _safe_arguments(getattr(task, '_attachment_joints', {})),
            'attachments': {str(name): {'mode': state.get('mode'),
                'robot_name': state.get('robot_name'), 'attach_step': state.get('attach_step')}
                for name, state in getattr(task, '_attachments', {}).items()},
            'shared_grasps': [{'object': str(key[0]), 'robot': str(key[1]),
                               'mode': state.get('mode')}
                              for key, state in getattr(task, '_beam_shared_grasps', {}).items()]}
        selected_paths = []
        for name in self.objects:
            wrapper = getattr(task, '_resolved_objects', {}).get(name)
            try:
                path = str(getattr(_unwrap(wrapper), 'prim_path', '')) if wrapper is not None else None
            except Exception:
                path = None
            record['bodies'].append(self.body(name, wrapper, path, task))
            if path:
                selected_paths.append(path)
        for robot_name, robot in getattr(task, 'robots', {}).items():
            try:
                articulation = getattr(robot, 'articulation', None)
                joints = _joints_snapshot(articulation)
                metadata_key = (robot_name, joints['view_id'])
                if metadata_key not in self.joint_metadata_seen:
                    joints['metadata'] = _joint_metadata(articulation)
                    self.joint_metadata_seen.add(metadata_key)
            except Exception as exc:
                joints = {'valid': False, 'error': _error(exc)}
            record['robots'][robot_name] = joints
            joints['sdk_submission'] = {
                'scope': 'completed_apply_action_calls_not_physical_tracking',
                'batch_sequence': getattr(robot, '_action_batch_sequence', None),
                'controllers': _safe_arguments(getattr(robot, 'last_action_controller_names', [])),
                'actions': _safe_arguments(getattr(robot, 'last_action', [])),
                'skipped': _safe_arguments(getattr(robot, 'last_action_skipped', [])),
            }
            rigid_map = getattr(robot, '_rigid_body_map', {})
            config = getattr(robot, 'config', None)
            links = ('Robotiq_2F_85/base_link',
                     str(getattr(config, 'left_finger_link_name', None) or 'left_inner_finger'),
                     str(getattr(config, 'right_finger_link_name', None) or 'right_inner_finger'))
            if getattr(task, '_beam_assisted_enabled', False):
                links += ('wrist_3_link',)
            for link in links:
                matches = [(path, body) for path, body in rigid_map.items()
                           if path.endswith('/' + link.strip('/'))]
                if not matches:
                    root = str(getattr(getattr(robot, 'articulation', None), '_root_prim_path', '') or '')
                    readers = getattr(task, '_assisted_native_body_readers', {})
                    matches = [(path, body) for path, body in readers.items()
                               if root and path.startswith(root.rstrip('/') + '/')
                               and path.endswith('/' + link.strip('/'))]
                if not matches and link == 'Robotiq_2F_85/base_link':
                    # The idle arm may never have needed contact probes. Its
                    # existing end-effector wrapper is already initialized.
                    # Reuse it only when it is actually the requested base.
                    existing = getattr(getattr(robot, 'articulation', None), 'end_effector', None)
                    try:
                        path = str(getattr(_unwrap(existing), 'prim_path', '')) if existing is not None else ''
                    except Exception:
                        path = ''
                    if path.endswith('/' + link):
                        matches = [(path, existing)]
                if len(matches) != 1:
                    record['bodies'].append({'name': f'{robot_name}/{link}', 'valid': False,
                                             'error': f'Expected one cached body; found {len(matches)}'})
                else:
                    path, wrapper = matches[0]
                    record['bodies'].append(self.body(f'{robot_name}/{link}', wrapper, path, task))
        ledger = {'scope': 'existing_cached_pairs_for_selected_bodies_not_complete_force_balance',
                  'pairs': [], 'aggregate': 'none; opposite-direction cached views may duplicate a physical pair'}
        unfiltered_seen = set()
        if record['clock_valid']:
            for key, probe in getattr(task, '_contact_probes', {}).items():
                if not isinstance(key, tuple) or len(key) != 2:
                    continue
                for selected in selected_paths:
                    if selected in key:
                        ledger['pairs'].append(_pair_load_snapshot(probe, key[0], key[1], selected,
                                                                  record['physics_dt']))
                        if (_unfiltered_phase_selected(task, self.cycle_phase)
                                and ((key[0] == selected and (_continuing_unfiltered_scope(task)
                                     or selected.endswith(('/fabrica_beam_3', '/fabrica_beam_6'))))
                                     or '/Robotiq_2F_85/' in key[0])
                                and key[0] not in unfiltered_seen):
                            unfiltered_seen.add(key[0])
                            ledger.setdefault('unfiltered_sensors', []).append(
                                _unfiltered_contact_snapshot(probe, key[0], record['physics_dt']))
        else:
            ledger['error'] = 'No valid physical dt for contact impulse conversion'
        ledger['valid'] = bool(ledger['pairs'] and all(pair['valid'] for pair in ledger['pairs']))
        record['contact_loads'] = ledger
        record['valid'] = bool(record['clock_valid'] and all(body['valid'] for body in record['bodies'])
                               and all(robot['valid'] for robot in record['robots'].values()))
        return self.write(record)


def _diagnostics(task):
    if os.environ.get('BEAM_PHYSICS_DIAGNOSTICS') != '1' or not _is_beam(task):
        return None
    instance = getattr(task, '_beam_physics_diagnostics', None)
    if instance is None:
        instance = BeamPhysicsDiagnostics()
        task._beam_physics_diagnostics = instance
    _register_paths(task)
    return instance


def _register_paths(task):
    """Associate existing body/robot paths with their owner, without resolving USD."""
    try:
        task_reference = weakref.ref(task)
    except TypeError:
        task_reference = lambda: task  # Lightweight test doubles may not allow weak refs.
    paths = []
    for body in getattr(task, '_resolved_objects', {}).values():
        try:
            paths.append(getattr(_unwrap(body), 'prim_path', None))
        except Exception:
            continue
    for robot in getattr(task, 'robots', {}).values():
        paths.extend(getattr(robot, '_rigid_body_map', {}).keys())
        paths.append(getattr(getattr(robot, 'config', None), 'prim_path', None))
        try:
            raw = _unwrap(getattr(robot, 'articulation', None))
            for field in ('prim_path', '_prim_path', '_root_prim_path'):
                paths.append(getattr(raw, field, None))
        except Exception:
            continue
    for path in paths:
        if path:
            _PATH_TASKS[str(path).rstrip('/')] = task_reference


def sample_boundary(task, boundary, context=None, *, cycle_step=None):
    """Runner hook; pass one fixed cycle_step to all four samples of a cycle."""
    diagnostics = _diagnostics(task)
    return None if diagnostics is None else diagnostics.sample(task, boundary, context, cycle_step)


def record_state_write(task, event, object_name=None, body_path=None, arguments=None):
    """Log a setter call site; the caller must label attempted/completed calls.

    This function observes neither task state nor the effect of a setter. It
    must be called by the existing writer and does not perform the write itself.
    """
    diagnostics = _diagnostics(task)
    if diagnostics is None:
        return None
    step = int(getattr(task, 'step_counter', 0))
    cycle_step = diagnostics.cycle_step if diagnostics.cycle_step is not None else step
    if not diagnostics.selected(task, cycle_step, diagnostics.cycle_phase):
        return None
    stack = traceback.extract_stack(limit=6)[:-1]
    return diagnostics.write({'kind': 'state_write_call_site', 'event': str(event),
                              'task_step': step, 'cycle_step': cycle_step,
                              'phase': str(getattr(task, 'phase', '')), 'object': object_name,
                              'body_path': body_path, 'arguments': _safe_arguments(arguments),
                              'callers': [{'file': frame.filename, 'line': frame.lineno,
                                           'function': frame.name} for frame in stack]})


def record_path_state_write(body_path, event, arguments=None):
    """Low-level setter hook, matched only to paths of previously sampled Beam tasks."""
    if os.environ.get('BEAM_PHYSICS_DIAGNOSTICS') != '1' or not body_path:
        return None
    path = str(body_path).rstrip('/')
    matching = sorted((registered for registered in _PATH_TASKS
                       if path == registered or path.startswith(registered + '/')),
                      key=len, reverse=True)
    for registered in matching:
        task = _PATH_TASKS[registered]()
        if task is not None:
            return record_state_write(task, event, body_path=path, arguments=arguments)
        _PATH_TASKS.pop(registered, None)
    return None
