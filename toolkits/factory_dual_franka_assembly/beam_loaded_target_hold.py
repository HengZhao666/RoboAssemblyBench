"""Opt-in, bounded loaded-target diagnostic; never a seating success gate.

The caller must retain grasp, slip, joint-tracking and hard-overload checks.
This module only qualifies a capture interval and reads existing SDK targets;
it never writes a body state, initializes a view, or declares seating success.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from toolkits.factory_dual_franka_assembly.beam_support_state import (
    physics_sample_token, physics_samples_consecutive,
)
from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import (
    _array, _joint_command_targets, _require_live, _view,
)


def loaded_target_hold_decision(state, sample, duration_seconds):
    """Capture once after 24 real load/gap samples, then hold for at most 2 s.

    Qualification deliberately excludes the stalled speed predicate. After
    capture, a clock discontinuity fails closed instead of recapturing a new
    target. Ordinary force/gap changes are observed during the hold; the hard
    10 N and -1 mm limits remain failures. The original caller's success gate
    and all physical grasp protections remain authoritative.
    """
    def result(mode, reason, elapsed=0.0):
        return {'mode': mode, 'reason': reason,
                'qualified_steps': int(state.get('qualified_steps', 0)),
                'elapsed': float(elapsed), 'duration_seconds': duration_seconds}

    def failure(reason):
        state['failure'] = reason
        return result('failure', reason)

    if state.get('failure'):
        return result('failure', state['failure'])
    try:
        duration_seconds = float(duration_seconds)
        if not math.isfinite(duration_seconds) or not 0 <= duration_seconds <= 2:
            return failure('invalid_hold_duration')
    except (ValueError, TypeError):
        return failure('invalid_hold_duration')
    if 'duration_seconds' in state and duration_seconds != state['duration_seconds']:
        return failure('hold_duration_changed')
    if duration_seconds == 0:
        return result('disabled', 'not_enabled')
    state.setdefault('duration_seconds', duration_seconds)
    if state.get('complete'):
        return result('complete', 'interval_complete', state['elapsed'])

    token = physics_sample_token(sample)
    try:
        valid = (token is not None and sample.get('valid') is True
                 and sample.get('motion_valid') is True
                 and sample.get('physics_handle_valid') is True
                 and math.isfinite(sample['vertical_force'])
                 and math.isfinite(sample['bottom_gap']))
    except (KeyError, TypeError, ValueError):
        valid = False
    if not valid:
        return failure('invalid_hold_sample')
    if sample['vertical_force'] > 10.0 or sample['bottom_gap'] < -.001:
        return failure('hold_overload_or_penetration')

    previous = state.get('last_sample')
    duplicate = token == physics_sample_token(previous or {})
    if duplicate:
        if (sample['physics_time'] != previous['physics_time']
                or sample['physics_dt'] != previous['physics_dt']):
            return failure('inconsistent_hold_clock')
        mode = 'holding' if state.get('captured') else 'qualifying'
        return result(mode, 'same_physics_sample', state.get('elapsed', 0.0))
    consecutive = physics_samples_consecutive(previous, sample)
    if state.get('captured') and not consecutive:
        return failure('hold_clock_discontinuity')
    # Store only the read-only observations needed for the clock transition.
    state['last_sample'] = copy.deepcopy(sample)
    if state.get('captured'):
        elapsed = float(sample['physics_time'] - state['capture_time'])
        state['elapsed'] = elapsed
        if elapsed >= duration_seconds - 1e-10:
            state['complete'] = True
            return result('complete', 'interval_complete', elapsed)
        return result('holding', 'fixed_native_targets', elapsed)

    qualified = (1.962 <= sample['vertical_force'] <= 10.0
                 and -.0005 <= sample['bottom_gap'] <= .001)
    if qualified:
        state['qualified_steps'] = (state.get('qualified_steps', 0) if consecutive else 0) + 1
    else:
        state['qualified_steps'] = 0
    if state['qualified_steps'] < 24:
        return result('qualifying', 'qualifying_load_and_gap')
    state.update(captured=True, capture_time=float(sample['physics_time']),
                 capture_token=token, elapsed=0.0)
    return result('capture', 'load_and_gap_qualified')


def capture_loaded_native_targets(robot, saved, open_q, closed_q, min_openness):
    """Read complete named arm/finger targets from the live existing backend.

    A generated cache is accepted only when it matches the published target.
    Nonzero velocity/effort targets are rejected because a position-only hold
    cannot preserve the intended static drive contract in that case.
    """
    try:
        articulation = robot.articulation
        view = _view(articulation, '_articulation_view')
        _require_live(view)
        measured = _array(view.get_joint_positions(clone=True))
        if measured.ndim != 2 or measured.shape[0] != 1 or measured.shape[1] == 0:
            raise ValueError('Expected one complete live articulation')
        targets = _joint_command_targets(view, measured.shape)
        if not targets['valid']:
            raise ValueError(f"Native target readback unavailable: {targets['errors']}")
        controller = robot.controllers['arm_joint_controller']
        subset = controller.get_joint_subset()
        names = list(controller.joint_names)
        if not names or len(set(names)) != len(names):
            raise ValueError('Missing or duplicate named arm DOFs')
        named_indices = [articulation.get_dof_index(name) for name in names]
        if any(not isinstance(index, (int, np.integer))
               or isinstance(index, (bool, np.bool_)) for index in named_indices):
            raise ValueError('Invalid named arm DOF index')
        indices = np.asarray(named_indices, dtype=int)
        raw_subset = np.asarray(subset.joint_indices, dtype=float).reshape(-1)
        if (raw_subset.shape != indices.shape or not np.all(np.isfinite(raw_subset))
                or not np.array_equal(raw_subset, indices)
                or len(set(indices.tolist())) != len(indices)
                or np.any(indices < 0) or np.any(indices >= measured.shape[1])):
            raise ValueError('Named arm indices disagree with live controller subset')
        gripper_index_raw = articulation.get_dof_index(robot.config.gripper_dof_name)
        if (not isinstance(gripper_index_raw, (int, np.integer))
                or isinstance(gripper_index_raw, (bool, np.bool_))):
            raise ValueError('Invalid named gripper DOF index')
        gripper_index = int(gripper_index_raw)
        if (gripper_index < 0 or gripper_index >= measured.shape[1]
                or gripper_index in indices):
            raise ValueError('Invalid or overlapping gripper DOF index')
        position_targets = np.asarray(targets['position_targets'], dtype=float)[0]
        arm_q = position_targets[indices].copy()
        cached = np.asarray(saved['arm_q'], dtype=float)
        if (cached.shape != arm_q.shape or not np.all(np.isfinite(cached))
                or not np.allclose(cached, arm_q, rtol=0., atol=1e-6)):
            raise ValueError('Grasp cache differs from published arm targets')
        selected = np.r_[indices, gripper_index]
        for field in ('velocity_targets', 'actuation_forces'):
            if np.any(np.abs(np.asarray(targets[field], dtype=float)[0, selected]) > 1e-9):
                raise ValueError(f'Nonzero selected native {field}')
        opened, closed, minimum = float(open_q), float(closed_q), float(min_openness)
        if (not all(math.isfinite(v) for v in (opened, closed, minimum))
                or abs(opened - closed) <= 1e-8 or not 0 <= minimum <= 1):
            raise ValueError('Invalid gripper command range')
        gripper_q = float(position_targets[gripper_index])
        openness = (gripper_q - closed) / (opened - closed)
        minimum_target = closed + minimum * (opened - closed)
        lower, upper = sorted((opened, minimum_target))
        # Float32 native targets can straddle a configured endpoint by one
        # representational bit. Accept that readback tolerance without clipping
        # the captured value or allowing a physically different target.
        if not lower - 1e-7 <= gripper_q <= upper + 1e-7:
            raise ValueError('Published finger target outside configured openness range')
        cached_openness = float(saved['gripper_openness'])
        if (not math.isfinite(cached_openness)
                or abs(closed + cached_openness * (opened - closed) - gripper_q) > 1e-6):
            raise ValueError('Grasp cache differs from published finger target')
        return {'valid': True, 'reason': 'native_targets_captured',
                'arm_q': arm_q.tolist(), 'arm_indices': indices.tolist(),
                'gripper_q': gripper_q, 'gripper_index': gripper_index,
                'gripper_openness': float(openness), 'full_native_targets': copy.deepcopy(targets)}
    except Exception as exc:
        return {'valid': False, 'reason': f'{type(exc).__name__}: {exc}'}
