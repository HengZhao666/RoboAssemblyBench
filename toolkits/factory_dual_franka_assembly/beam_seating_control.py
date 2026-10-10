"""Bounded vertical command increments for a physically held Beam base.

This controller only proposes robot target changes. Contact, grasp, IK and
table stability are independently checked by the caller/task. Timing comes
from sampled physics state; task labels never create a new control interval.
"""
import math
import numpy as np
from toolkits.factory_dual_franka_assembly.beam_support_state import physics_sample_token


def _physics_clock(sample, dt):
    """Require the live sample's clock and the caller's physical control period."""
    token = physics_sample_token(sample)
    if token is None:
        return None
    physical_dt = float(sample['physics_dt'])
    if not math.isclose(float(dt), physical_dt, rel_tol=1e-6, abs_tol=1e-9):
        return None
    return token, float(sample['physics_time']), physical_dt


def _clock_transition(previous, clock):
    if previous is None:
        return 'first'
    token, time, dt = clock
    old_token, old_time, old_dt = previous
    if token == old_token:
        return 'duplicate' if math.isclose(time, old_time, rel_tol=0., abs_tol=1e-10) else 'invalid'
    if token[0] != old_token[0] or token[1] <= old_token[1] or time <= old_time:
        return 'reset'
    if token[1] != old_token[1] + 1:
        return 'gap'
    if (not math.isclose(old_dt, dt, rel_tol=1e-9, abs_tol=1e-12)
            or not math.isclose(time - old_time, dt, rel_tol=1e-6,
                                abs_tol=max(1e-10, dt * 1e-6))):
        return 'gap'
    return 'next'


def _clear_motion_window(state):
    state['motion_history'] = []
    state.pop('motion_physics_clock', None)


def pose_motion_window(state, sample, dt, window):
    """Measure drift and excursion over contiguous, real physics time.

    Duplicate calls do not add points. A missing/reset clock or an unsampled
    interval invalidates the previous window, rather than assuming that unseen
    motion was stationary. All pose arrays are copied out of caller buffers.
    """
    clock = _physics_clock(sample, dt)
    if clock is None or not math.isfinite(window) or window <= 0:
        _clear_motion_window(state)
        return {'valid': False, 'ready': False, 'reason': 'invalid_physics_clock'}
    try:
        position = np.array(sample['position'], dtype=float, copy=True)
        orientation = np.array(sample['orientation'], dtype=float, copy=True)
        if (position.shape != (3,) or orientation.shape != (4,)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation))
                or np.linalg.norm(orientation) < 1e-9):
            raise ValueError('Invalid measured pose')
        orientation = orientation / np.linalg.norm(orientation)
    except (KeyError, ValueError, TypeError):
        _clear_motion_window(state)
        return {'valid': False, 'ready': False, 'reason': 'invalid_measured_pose'}
    transition = _clock_transition(state.get('motion_physics_clock'), clock)
    if transition == 'invalid':
        _clear_motion_window(state)
        return {'valid': False, 'ready': False, 'reason': 'inconsistent_physics_clock'}
    if transition in ('reset', 'gap'):
        _clear_motion_window(state)
    history = state.setdefault('motion_history', [])
    _, time, _ = clock
    if transition != 'duplicate':
        history.append((time, position, orientation))
        state['motion_physics_clock'] = clock
    else:
        # Re-reading an identity cannot change the already sampled motion.
        _, position, orientation = history[-1]
    cutoff = time - window
    while len(history) > 1 and history[1][0] <= cutoff + 1e-10:
        history.pop(0)
    duration = time - history[0][0]
    if duration < window - 1e-10:
        return {'valid': True, 'ready': False, 'duration': duration,
                'clock_transition': transition}

    def angle(q):
        return 2 * math.acos(float(np.clip(abs(np.dot(orientation, q)), 0., 1.)))

    return {'valid': True, 'ready': True, 'duration': duration,
            'clock_transition': transition,
            'linear_speed': float(np.linalg.norm(position-history[0][1])) / duration,
            'angular_speed': angle(history[0][2]) / duration,
            'position_excursion': max(float(np.linalg.norm(position-p)) for _, p, _ in history),
            'orientation_excursion': max(angle(q) for _, _, q in history)}


def seating_decision(state, sample, command_z, dt, spec):
    decision = {'mode': 'hold', 'delta_z': 0.0, 'failure': None}

    def fail(reason):
        return {**decision, 'mode': 'failed', 'failure': reason}

    if not math.isfinite(dt) or dt <= 0:
        return fail('seat_invalid_control_dt')
    clock = _physics_clock(sample, dt)
    if clock is None:
        _clear_motion_window(state)
        state['clock_sample_missing'] = True
        return fail('seat_invalid_physics_stamp')
    if (not sample.get('valid') or not sample.get('motion_valid')
            or not all(math.isfinite(sample.get(k, float('nan'))) for k in
                       ('vertical_force', 'bottom_gap', 'linear_speed', 'angular_speed'))):
        _clear_motion_window(state)
        state['clock_sample_missing'] = True
        return fail('seat_contact_probe_invalid')
    force = sample['vertical_force']
    if force > 10.0 or sample['bottom_gap'] < -.001:
        return fail('seat_overload_or_penetration')
    if not math.isfinite(command_z):
        return fail('seat_invalid_command')
    transition = _clock_transition(state.get('last_physics_clock'), clock)
    if transition == 'invalid':
        _clear_motion_window(state)
        state['clock_sample_missing'] = True
        return fail('seat_invalid_physics_stamp')
    if transition == 'duplicate':
        return {**decision, 'mode': 'already_sampled'}
    if state.pop('clock_sample_missing', False):
        transition = 'gap'
    state['last_physics_clock'] = clock
    token, time, physical_dt = clock
    state.setdefault('start_z', command_z)
    discontinuity = transition in ('reset', 'gap')
    if discontinuity:
        _clear_motion_window(state)
        # Keep both displacement budgets and the contact latch. A clock reset
        # must not grant another free descent or erase prior unload travel.
        if 'contact_step' in state:
            state['contact_time'] = time
            state['contact_epoch'] = token[0]
            state['contact_step'] = token[1]
    motion = pose_motion_window(state, sample, physical_dt, float(spec['seat_motion_window']))
    decision['pose_motion'] = motion
    if not motion['valid']:
        state['clock_sample_missing'] = True
        return fail('seat_pose_probe_invalid')
    weight = float(spec['seat_payload_mass']) * 9.81
    if force >= float(spec['seat_contact_force']) and 'contact_step' not in state:
        state['contact_step'] = token[1]
        state['contact_epoch'] = token[0]
        state['contact_time'] = time
        state['contact_z'] = command_z
    contact = 'contact_step' in state
    if discontinuity:
        return {**decision, 'mode': 'clock_rewarm', 'target_force': weight,
                'contact_step': state.get('contact_step'),
                'contact_physics_time': state.get('contact_time')}
    motion_settled = (motion['ready']
        and motion['linear_speed'] <= float(spec['seat_max_linear_speed'])
        and motion['angular_speed'] <= float(spec['seat_max_angular_speed'])
        and motion['position_excursion'] <= float(spec['seat_max_position_excursion'])
        and motion['orientation_excursion'] <= float(spec['seat_max_orientation_excursion'])
        and sample['linear_speed'] <= float(spec['seat_raw_linear_speed_limit'])
        and sample['angular_speed'] <= float(spec['seat_raw_angular_speed_limit']))
    deadband = float(spec['seat_force_deadband'])
    # Unload even during the initial contact wait if force is already excessive.
    # The raw 10 N stop above always takes precedence over any filtered response.
    if contact and force > weight + deadband:
        velocity = min(float(spec['seat_up_speed']),
                       float(spec['seat_admittance']) * (force - weight))
        decision.update(mode='unload', delta_z=velocity * physical_dt)
    elif contact and time - state['contact_time'] < float(spec['seat_contact_wait']) - 1e-10:
        decision['mode'] = 'first_contact_wait'
    elif not motion_settled:
        decision['mode'] = 'motion_wait'
    elif contact:
        if force < weight - deadband:
            velocity = min(float(spec['seat_load_speed']),
                           float(spec['seat_admittance']) * (weight - force))
            decision.update(mode='load', delta_z=-velocity * physical_dt)
        else:
            decision['mode'] = 'force_band_hold'
    else:
        near = sample['bottom_gap'] <= float(spec['seat_slow_gap'])
        velocity = float(spec['seat_near_speed'] if near else spec['seat_approach_speed'])
        decision.update(mode='approach_near' if near else 'approach', delta_z=-velocity * physical_dt)
    proposed_z = command_z + decision['delta_z']
    if state['start_z'] - proposed_z > float(spec['seat_max_travel']):
        return fail('seat_travel_exhausted')
    if contact and proposed_z - state['contact_z'] > float(spec['seat_max_unload_travel']):
        return fail('seat_unload_travel_exhausted')
    return {**decision, 'target_force': weight, 'contact_step': state.get('contact_step'),
            'contact_physics_time': state.get('contact_time')}
