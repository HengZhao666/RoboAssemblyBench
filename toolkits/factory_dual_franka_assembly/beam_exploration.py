"""Pure, default-off decisions for pose-qualified Beam exploration.

The caller must continue evaluating the original support gate and final gate.
No body, robot, command, or simulation is read or written by this module.
"""
from __future__ import annotations

import copy
import math


ENV_NAME = 'BEAM_EXPLORE_POST_SEAT'
CONTINUE_ENV_NAME = 'BEAM_EXPLORE_CONTINUE'
SEAT_PHASE = 'base_6_set_down'
STOP_PHASE = 'assemble_00_part_3_physical_hold_check'
FORBIDDEN_RELEASE_PHASE = 'hold_6_to_3_switch_release'
_PRE_SEAT = {'fixture_parts_settle', *(f'base_6_{suffix}' for suffix in (
    'move_above', 'preshape', 'descend', 'close_and_attach', 'grip_settle',
    'peel_lift', 'grip_contact_prime', 'peel_physical_hold_check', 'lift',
    'post_lift_restore_gravity', 'assembly_clearance', 'transport_hover',
    'place', 'physical_hold_check', 'restore_gravity', 'clear_support_filters'))}
_POST_SEAT = {*(f'assemble_00_part_3_{suffix}' for suffix in (
    'transit_clearance', 'move_above', 'preshape', 'descend', 'close_and_attach',
    'grip_settle', 'peel_lift', 'grip_contact_prime', 'peel_physical_hold_check',
    'lift', 'post_lift_restore_gravity', 'pickup_clearance', 'assembly_clearance',
    'transport_hover', 'physical_hold_check')),
    *(f'assemble_00_part_3_insert_{index:02d}' for index in range(10))}



def continuation_step_budget(phases, current_budget, default_phase_timeout):
    """Cover the complete existing phase graph without changing any phase gate.

    One entry tick per phase plus one final tick keep the administrative episode
    limit beyond all original phase timeouts. Each phase still fails at its own
    unchanged timeout. Only explicit continuous exploration uses this budget.
    """
    if type(current_budget) is not int or current_budget <= 0:
        raise ValueError('Positive integer episode budget required')
    if type(default_phase_timeout) is not int or default_phase_timeout <= 0:
        raise ValueError('Positive integer default phase timeout required')
    if not phases:
        raise ValueError('Continuous exploration requires a phase graph')
    timeouts = [p.get('timeout_steps', default_phase_timeout) for p in phases]
    if any(type(value) is not int or value <= 0 for value in timeouts):
        raise ValueError('Continuous exploration requires positive phase timeouts')
    return max(current_budget, sum(timeouts) + len(phases) + 1)


def parse_exploration_environment(environ):
    """Accept only the exact opt-in string '1'; unset/'0' is disabled."""
    value = environ.get(ENV_NAME)
    if value is None or value == '0':
        return False
    if value == '1':
        return True
    raise ValueError(f'{ENV_NAME} must be unset, "0", or "1"')


def parse_exploration_continuation(environ, enabled):
    value = environ.get(CONTINUE_ENV_NAME, '0')
    if value not in ('0', '1'):
        raise ValueError(f'{CONTINUE_ENV_NAME} must be unset, "0", or "1"')
    if value == '1' and not enabled:
        raise ValueError(f'{CONTINUE_ENV_NAME} requires {ENV_NAME}=1')
    return value == '1'


def _vector(value, size):
    if len(value) != size:
        raise ValueError('Invalid vector shape')
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Non-finite vector')
    return result


def _distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _angle(a, b):
    return 2. * math.acos(min(1., max(0., abs(sum(x * y for x, y in zip(a, b))))))


def _sample(sample):
    if not isinstance(sample, dict):
        raise ValueError('Support sample missing')
    if (sample.get('valid') is not True or sample.get('motion_valid') is not True
            or sample.get('physics_stamp_valid') is not True):
        raise ValueError('Invalid support/pose/stamp')
    epoch, index = sample['physics_epoch'], sample['physics_step_index']
    time, dt = float(sample['physics_time']), float(sample['physics_dt'])
    if (type(epoch) is not int or epoch < 0 or type(index) is not int or index < 0
            or not math.isfinite(time) or time < 0 or not math.isfinite(dt) or dt <= 0):
        raise ValueError('Invalid physical clock')
    clock = (epoch, index, time, dt)
    for key in ('physics_stamp', 'physics_stamp_after'):
        if key in sample:
            stamp = sample[key]
            if (not isinstance(stamp, dict) or stamp.get('valid') is not True
                    or (stamp['epoch'], stamp['index'], stamp['time'], stamp['dt']) != clock):
                raise ValueError('Support sample clock changed during reading')
    position, orientation = _vector(sample['position'], 3), _vector(sample['orientation'], 4)
    norm = math.sqrt(sum(v * v for v in orientation))
    if abs(norm - 1.) > .001:
        raise ValueError('Invalid pose quaternion')
    orientation = tuple(v / norm for v in orientation)
    force, gap = float(sample['vertical_force']), float(sample['bottom_gap'])
    linear, angular = float(sample['linear_speed']), float(sample['angular_speed'])
    if (not all(math.isfinite(v) for v in (force, gap, linear, angular))
            or min(force, linear, angular) < 0):
        raise ValueError('Invalid support/motion values')
    surfaces = sample['support_surfaces']
    if not surfaces:
        raise ValueError('Support pairs missing')
    signed, seen = 0., set()
    for surface in surfaces:
        if not isinstance(surface, dict):
            raise ValueError('Support pair malformed')
        name, contact = surface['object'], surface['contact']
        if (not isinstance(name, str) or not name or name in seen
                or not isinstance(contact, dict) or contact.get('valid') is not True):
            raise ValueError('Invalid or duplicate support pair')
        seen.add(name)
        signed += _vector(contact['force_world'], 3)[2]
    if not math.isfinite(signed) or abs(abs(signed) - force) > 1e-5:
        raise ValueError('Signed support load disagrees with aggregate')
    return {'clock': clock, 'position': position, 'orientation': orientation,
            'force': signed, 'gap': gap, 'linear_speed': linear, 'angular_speed': angular}


def _generations(values):
    if not isinstance(values, dict):
        raise ValueError('Grasp-loss generation map missing')
    if any(not isinstance(k, str) or not k or type(v) is not int or v < 0
           for k, v in values.items()):
        raise ValueError('Invalid grasp-loss generation')
    return dict(values)


def _result(state, mode, reason, **extra):
    result = {'mode': mode, 'reason': reason, 'ready': mode == 'ready',
              'qualified_steps': state.get('seat_count', 0), **extra}
    if mode in ('ready', 'complete'):
        result['marker'] = {
            'kind': 'beam_post_seat_exploration', 'formal_success': False,
            'exploration_used': bool(state.get('exploration_used')),
            'original_seat_ready': bool(state.get('original_seat_ready')),
            'first_insertion_complete': bool(state.get('first_insertion_complete') or mode == 'complete'),
            'stop_before_phase': None if state.get('continue_after_first_insertion') else FORBIDDEN_RELEASE_PHASE,
            'capture_clock': state.get('capture_clock'), 'reason': reason}
    return result


def _fail(state, reason):
    state['failure'] = reason
    return _result(state, 'failure', reason)


def _clock_transition(previous, current):
    if previous is None:
        return 'first'
    if previous == current:
        return 'duplicate'
    pe, pi, pt, pd = previous
    e, i, t, dt = current
    if (e != pe or i != pi + 1 or not math.isclose(pd, dt, rel_tol=1e-9, abs_tol=1e-12)
            or not math.isclose(t - pt, dt, rel_tol=1e-6, abs_tol=max(1e-10, dt * 1e-6))):
        return 'invalid'
    return 'consecutive'


def _safety_reason(data, maximum_force, minimum_force=None, gap_bounds=None):
    if data['force'] < 0:
        return 'support_force_wrong_direction'
    if data['force'] > maximum_force:
        return 'support_overload'
    if data['gap'] < -.001:
        return 'support_penetration'
    if minimum_force is not None and data['force'] < minimum_force:
        return 'support_lost'
    if gap_bounds is not None and not gap_bounds[0] <= data['gap'] <= gap_bounds[1]:
        return 'support_gap_out_of_bounds'
    return None


def _pose_result(result, clock):
    """Require the tracker's same-frame contract without calculating its windows."""
    if not isinstance(result, dict) or result.get('valid') is not True:
        raise ValueError('Pose tracker invalid')
    if result.get('latched_failure'):
        raise ValueError('Pose tracker failure latched')
    stamp = result['physics_stamp']
    if not isinstance(stamp, dict):
        raise ValueError('Pose tracker clock missing')
    if (stamp['physics_epoch'], stamp['physics_step_index'],
            stamp['physics_time'], stamp['physics_dt']) != clock:
        raise ValueError('Pose tracker stamp is not this support sample')
    if type(result.get('ready')) is not bool:
        raise ValueError('Pose tracker ready flag invalid')
    if result['ready']:
        anchor = result['anchor']
        if (not isinstance(anchor, dict) or result.get('motion_windows_ready') is not True
                or anchor.get('captured') is not True or anchor.get('within_budget') is not True):
            raise ValueError('Pose tracker ready without its windows and anchor')
        position = _vector(anchor['actor_position'], 3)
        orientation = _vector(anchor['orientation'], 4)
        norm = math.sqrt(sum(v * v for v in orientation))
        if abs(norm - 1.) > .001:
            raise ValueError('Invalid tracker anchor quaternion')
        anchor_clock = (anchor['physics_epoch'], anchor['physics_step_index'],
                        anchor['physics_time'], clock[3])
        if (type(anchor_clock[0]) is not int or type(anchor_clock[1]) is not int
                or anchor_clock[0] != clock[0] or not 0 <= anchor_clock[1] <= clock[1]
                or not math.isfinite(anchor_clock[2]) or not 0 <= anchor_clock[2] <= clock[2]):
            raise ValueError('Invalid tracker anchor clock')
        return True, {'position': position,
                      'orientation': tuple(v / norm for v in orientation)}, anchor_clock
    return False, None, None


def exploration_seat_decision(state, sample, phase_name, loss_generations, enabled,
                              pose_result, original_ready=False):
    """Parallel pose qualification for only base6; never replace the original gate.

    Ninety-six fresh samples must jointly pass load, gap and the pose tracker.
    Its ready flag represents both 0.4/2-second windows and a captured anchor.
    An original gate pass is recorded but never bypasses the exploration gate.
    The caller chooses whether to advance and records both independent results.
    """
    if not enabled:
        return {'mode': 'disabled', 'reason': 'option_disabled', 'ready': False, 'qualified_steps': 0}
    if state.get('failure'):
        return _fail(state, state['failure'])
    if phase_name != SEAT_PHASE:
        return _fail(state, 'seat_option_outside_phase')
    if type(original_ready) is not bool:
        return _fail(state, 'invalid_original_gate_result')
    try:
        data, generations = _sample(sample), _generations(loss_generations)
        pose_ready, pose_anchor, anchor_clock = _pose_result(pose_result, data['clock'])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return _fail(state, 'invalid_sample_or_generation: ' + str(exc))
    reason = _safety_reason(data, 10.)
    if reason:
        return _fail(state, reason)
    previous = state.get('seat_last')
    transition = _clock_transition(previous['clock'] if previous else None, data['clock'])
    if transition == 'invalid':
        return _fail(state, 'seat_clock_discontinuity')
    if transition == 'duplicate' and previous != data:
        return _fail(state, 'duplicate_sample_changed')
    if state.get('anchor'):
        if generations != state['loss_generations']:
            return _fail(state, 'grasp_generation_changed_after_capture')
        if not pose_ready or data['force'] < 1.962 or not -.0005 <= data['gap'] <= .001:
            return _fail(state, 'seat_qualification_lost_before_advance')
        state['seat_last'] = data
        state['guard_last'] = data
        return _result(state, 'ready', 'already_qualified',
                       signed_vertical_force=data['force'], pose_motion=copy.deepcopy(pose_result))
    state['original_seat_ever_ready'] = bool(state.get('original_seat_ever_ready') or original_ready)
    if transition != 'duplicate':
        state['seat_last'] = data
        if pose_ready and data['force'] >= 1.962 and -.0005 <= data['gap'] <= .001:
            state['seat_count'] = min(state.get('seat_count', 0) + 1, 96)
        else:
            state['seat_count'] = 0
    if state.get('seat_count', 0) >= 96 and pose_ready:
        state.update(anchor=pose_anchor, capture_clock=anchor_clock, gate_clock=data['clock'], loss_generations=generations,
                     guard_last=data, exploration_used=not original_ready,
                     original_seat_ready=original_ready)
        return _result(state, 'ready', 'pose_and_load_gate_ready',
                       signed_vertical_force=data['force'], pose_motion=copy.deepcopy(pose_result))
    reason = 'pose_window_unqualified' if not pose_ready else 'load_gap_window_unqualified'
    return _result(state, 'qualifying', reason, signed_vertical_force=data['force'],
                   original_ready=original_ready, pose_motion=copy.deepcopy(pose_result))


def exploration_guard_decision(state, sample, phase_name, loss_generations, enabled,
                               phase_complete=False, integrity_valid=True):
    """Stop before base6 release; observe only the seated base, not moving part3.

    Call after state synchronization and before any next action or interaction.
    At STOP_PHASE pass its original completion result, before phase advancement.
    A complete result is an exploratory stop request, never benchmark success.
    """
    if not enabled:
        return {'mode': 'disabled', 'reason': 'option_disabled', 'ready': False, 'qualified_steps': 0}
    if state.get('failure'):
        return _fail(state, state['failure'])
    if phase_name in _PRE_SEAT:
        if state.get('anchor'):
            return _fail(state, 'exploration_stage_regressed')
        return _result(state, 'waiting', 'before_seating')
    if phase_name == SEAT_PHASE and not state.get('anchor'):
        return _result(state, 'seating', 'seat_not_yet_qualified')
    continuing = bool(state.get('continue_after_first_insertion'))
    if phase_name != SEAT_PHASE and phase_name not in _POST_SEAT and not continuing:
        return _fail(state, 'exploration_stop_boundary_exceeded')
    if not state.get('anchor'):
        return _fail(state, 'exploration_seat_not_qualified')
    if type(phase_complete) is not bool or integrity_valid is not True:
        return _fail(state, 'integrity_or_completion_invalid')
    try:
        data, generations = _sample(sample), _generations(loss_generations)
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return _fail(state, 'invalid_sample_or_generation: ' + str(exc))
    baseline = state['loss_generations']
    if any(generations.get(k, 0) != v for k, v in baseline.items()):
        return _fail(state, 'unexpected_grasp_loss_or_generation_reset')
    if any(v > baseline.get(k, 0) for k, v in generations.items()):
        return _fail(state, 'unexpected_grasp_loss_or_generation_reset')
    previous = state['guard_last']
    transition = _clock_transition(previous['clock'], data['clock'])
    if transition == 'invalid':
        return _fail(state, 'guard_clock_discontinuity')
    if transition == 'duplicate' and previous != data:
        return _fail(state, 'duplicate_sample_changed')
    reason = _safety_reason(data, 10. if phase_name == SEAT_PHASE else 40., .5, (-.001, .0015))
    if reason:
        return _fail(state, reason)
    position_drift = _distance(data['position'], state['anchor']['position'])
    orientation_drift = _angle(data['orientation'], state['anchor']['orientation'])
    if position_drift > .001 or orientation_drift > .01:
        return _fail(state, 'seated_base_displaced')
    state['guard_last'] = data
    if phase_name == STOP_PHASE and phase_complete:
        state['first_insertion_complete'] = True
        if not continuing:
            state['complete'] = True
    if state.get('complete'):
        return _result(state, 'complete', 'first_insertion_verified_stop_before_release',
                       position_drift=position_drift, orientation_drift=orientation_drift)
    return _result(state, 'guarding', 'base_support_retained',
                   signed_vertical_force=data['force'], position_drift=position_drift,
                   orientation_drift=orientation_drift)
