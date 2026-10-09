"""Bounded vertical command increments for a physically held Beam base.

This controller only proposes robot target changes. Contact, grasp, IK and
table stability are independently checked by the caller/task.
"""
import math
import numpy as np


def pose_motion_window(state, sample, dt, window):
    """Measure drift and excursion, so returning oscillations cannot cancel out."""
    try:
        position = np.asarray(sample['position'], dtype=float)
        orientation = np.asarray(sample['orientation'], dtype=float)
        if (position.shape != (3,) or orientation.shape != (4,)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation))
                or np.linalg.norm(orientation) < 1e-9):
            raise ValueError('Invalid measured pose')
        orientation = orientation / np.linalg.norm(orientation)
    except (KeyError, ValueError, TypeError):
        return {'valid': False, 'ready': False}
    history = state.setdefault('motion_history', [])
    step = sample['step']
    history.append((step, position, orientation))
    cutoff = step - math.ceil(window / dt)
    while len(history) > 1 and history[1][0] <= cutoff:
        history.pop(0)
    duration = (step - history[0][0]) * dt
    if duration < window - 1e-10:
        return {'valid': True, 'ready': False, 'duration': duration}
    def angle(q):
        return 2 * math.acos(float(np.clip(abs(np.dot(orientation, q)), 0., 1.)))
    return {'valid': True, 'ready': True, 'duration': duration,
            'linear_speed': float(np.linalg.norm(position-history[0][1])) / duration,
            'angular_speed': angle(history[0][2]) / duration,
            'position_excursion': max(float(np.linalg.norm(position-p)) for _,p,_ in history),
            'orientation_excursion': max(angle(q) for _,_,q in history)}


def seating_decision(state, sample, command_z, dt, spec):
    decision = {'mode': 'hold', 'delta_z': 0.0, 'failure': None}

    def fail(reason):
        return {**decision, 'mode': 'failed', 'failure': reason}

    if not math.isfinite(dt) or dt <= 0:
        return fail('seat_invalid_control_dt')
    if (not sample.get('valid') or not sample.get('motion_valid')
            or not all(math.isfinite(sample.get(k, float('nan'))) for k in
                       ('vertical_force', 'bottom_gap', 'linear_speed', 'angular_speed'))):
        return fail('seat_contact_probe_invalid')
    force = sample['vertical_force']
    if force > 10.0 or sample['bottom_gap'] < -.001:
        return fail('seat_overload_or_penetration')
    if not math.isfinite(command_z):
        return fail('seat_invalid_command')
    step = sample['step']
    if state.get('last_step') == step:
        return {**decision, 'mode': 'already_sampled'}
    state['last_step'] = step
    state.setdefault('start_z', command_z)
    motion = pose_motion_window(state, sample, dt, float(spec['seat_motion_window']))
    decision['pose_motion'] = motion
    if not motion['valid']:
        return fail('seat_pose_probe_invalid')
    weight = float(spec['seat_payload_mass']) * 9.81
    if force >= float(spec['seat_contact_force']) and 'contact_step' not in state:
        state['contact_step'] = step
        state['contact_z'] = command_z
    contact = 'contact_step' in state
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
        decision.update(mode='unload', delta_z=velocity * dt)
    elif contact and (step - state['contact_step']) * dt < float(spec['seat_contact_wait']):
        decision['mode'] = 'first_contact_wait'
    elif not motion_settled:
        decision['mode'] = 'motion_wait'
    elif contact:
        if force < weight - deadband:
            velocity = min(float(spec['seat_load_speed']),
                           float(spec['seat_admittance']) * (weight - force))
            decision.update(mode='load', delta_z=-velocity * dt)
        else:
            decision['mode'] = 'force_band_hold'
    else:
        near = sample['bottom_gap'] <= float(spec['seat_slow_gap'])
        velocity = float(spec['seat_near_speed'] if near else spec['seat_approach_speed'])
        decision.update(mode='approach_near' if near else 'approach', delta_z=-velocity * dt)
    proposed_z = command_z + decision['delta_z']
    if state['start_z'] - proposed_z > float(spec['seat_max_travel']):
        return fail('seat_travel_exhausted')
    if contact and proposed_z - state['contact_z'] > float(spec['seat_max_unload_travel']):
        return fail('seat_unload_travel_exhausted')
    return {**decision, 'target_force': weight, 'contact_step': state.get('contact_step')}
