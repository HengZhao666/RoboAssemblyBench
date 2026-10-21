"""Select a bounded grip-force band after measured Beam tabletop contact.

This selects robot force-control parameters only. It does not declare seating,
release a grasp, recapture its anchor, or write any object state.
"""
import math
import copy
from toolkits.factory_dual_franka_assembly.beam_support_state import (
    physics_sample_token, physics_samples_consecutive,
)


def contact_relief_spec(state, sample, carry_spec, config):
    """Qualify distinct consecutive physics samples, then retain real contact.

    Partial table contact permits a 3--4 N peak-finger target band. At balanced
    loads this leaves nominal carrying margin for the 0.25 kg base at the trial
    coefficient 0.8; it is not proof of equal finger forces or a stable hold.
    Independent bilateral contact, slip, overload and seating gates remain.
    """
    step = sample.get('physics_step_index')
    previous = state.get('last_physics_sample')
    token = physics_sample_token(sample)
    old_token = physics_sample_token(previous or {})
    duplicate = token is not None and token == old_token
    same_clock = (duplicate and sample['physics_time'] == previous['physics_time']
                  and sample['physics_dt'] == previous['physics_dt'])
    try:
        contact_valid = (token is not None and not (duplicate and not same_clock)
                 and sample.get('valid') and sample.get('motion_valid')
                 and sample.get('physics_handle_valid') and type(step) is int
                 and all(math.isfinite(sample.get(k, float('nan'))) for k in
                         ('vertical_force', 'bottom_gap'))
                 and -.0005 <= sample['bottom_gap'] <= .001
                 and 0. <= sample['vertical_force'])
        valid = contact_valid and sample['vertical_force'] <= 10.
    except (TypeError, ValueError):
        contact_valid = valid = False
    force = sample.get('vertical_force', 0.)
    pressure_limited = bool(contact_valid and force > 10. and state.get('active')
                            and (same_clock or physics_samples_consecutive(previous, sample)))
    if pressure_limited:
        # A large supported load is a reason to keep reducing grip pressure,
        # not to restore the stronger/faster carry controller. It earns no new
        # qualification: only previously qualified, continuous contact can
        # retain this control mode. All grasp and seating gates stay separate.
        state.update(last_physics_step=step, last_physics_sample=copy.deepcopy(sample))
        reason = 'high_table_load_relief'
    elif not valid or force < float(config['retain_force']):
        state.update(active=False, count=0, last_physics_step=step,
                     last_physics_sample=copy.deepcopy(sample) if valid else None)
        reason = 'invalid_or_lost_table_contact'
    elif same_clock:
        reason = 'same_physics_sample'
    else:
        state['last_physics_step'] = step
        if not physics_samples_consecutive(previous, sample):
            state.update(active=False, count=0)
        state['last_physics_sample'] = copy.deepcopy(sample)
        if not state.get('active'):
            count = state.get('count', 0)
            state['count'] = count + 1 if force >= float(config['qualification_force']) else 0
            state['active'] = state['count'] >= int(config['stable_steps'])
        reason = 'contact_relief' if state.get('active') else 'qualifying_table_contact'
    selected = dict(carry_spec)
    if state.get('active'):
        selected.update(close_force_low=float(config['force_low']),
                        close_force_high=float(config['force_high']),
                        close_force_joint_rate=float(config['joint_rate']))
    return selected, {'active': bool(state.get('active')), 'reason': reason,
                      'qualification_valid': bool(valid), 'pressure_limited': pressure_limited,
                      'qualified_steps': state.get('count', 0), 'physics_step_index': step,
                      'table_force': force, 'bottom_gap': sample.get('bottom_gap'),
                      'force_low': selected.get('close_force_low', 4.),
                      'force_high': selected.get('close_force_high', 12.),
                      'joint_rate': selected.get('close_force_joint_rate', .025)}
