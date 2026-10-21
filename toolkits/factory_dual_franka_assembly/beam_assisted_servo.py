"""Coherent object-space commands and descent admission for fixed Beam grasps.

These functions only calculate targets. They never change bodies or physics.
"""
from __future__ import annotations

import math
import numpy as np

from .planner_primitives import normalize_quat, pose_error, quat_conjugate, quat_multiply, quat_rotate


def _position(value):
    result = np.asarray(value, float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError('assisted servo requires finite 3D positions')
    return result


def _orientation(value):
    value = np.asarray(value, float)
    if value.shape != (4,) or not np.all(np.isfinite(value)) or np.linalg.norm(value) < 1e-12:
        raise ValueError('assisted servo requires finite nonzero quaternions')
    return normalize_quat(value)


def _orientation_step(current, target, limit):
    current, target = _orientation(current), _orientation(target)
    dot = float(np.dot(current, target))
    if dot < 0:
        target, dot = -target, -dot
    angle = 2 * math.acos(float(np.clip(dot, -1, 1)))
    if angle <= limit or angle < 1e-12:
        return target
    ratio = limit / angle
    if dot > .9995:
        return normalize_quat((1-ratio)*current + ratio*target)
    half = angle/2
    return normalize_quat((math.sin((1-ratio)*half)*current + math.sin(ratio*half)*target)/math.sin(half))


def object_pose_from_tcp(tcp_pose, object_in_tcp_position, object_in_tcp_orientation):
    q = _orientation(tcp_pose['orientation'])
    return {'position': _position(tcp_pose['position']) + quat_rotate(q, _position(object_in_tcp_position)),
            'orientation': normalize_quat(quat_multiply(q, _orientation(object_in_tcp_orientation)))}


def anchored_object_prediction(native_object_pose, measured_fk_object_pose, command_fk_object_pose):
    """Apply the model's joint-motion increment to the current native actor.

    FK calibration/grasp constraint residual is measured separately. In
    particular a measured-joint hold predicts exactly zero actor motion.
    """
    rotation_increment = quat_multiply(_orientation(command_fk_object_pose['orientation']),
                                        quat_conjugate(_orientation(measured_fk_object_pose['orientation'])))
    return {'position': _position(native_object_pose['position'])
            +_position(command_fk_object_pose['position'])-_position(measured_fk_object_pose['position']),
            'orientation': normalize_quat(quat_multiply(rotation_increment,
                                          _orientation(native_object_pose['orientation'])))}


def model_command_for_native_pose(native_object_pose, measured_fk_object_pose, desired_native_pose):
    """Inverse of anchored prediction; compensate measured model/grasp bias."""
    return {'position': _position(measured_fk_object_pose['position'])
            +_position(desired_native_pose['position'])-_position(native_object_pose['position']),
            'orientation': normalize_quat(quat_multiply(
                quat_multiply(_orientation(desired_native_pose['orientation']),
                              quat_conjugate(_orientation(native_object_pose['orientation']))),
                _orientation(measured_fk_object_pose['orientation'])))}


def _consecutive_native_com_rate(state, stamp, motion):
    """End-of-tick COM pose change; raw solver velocity remains separate."""
    if (motion.get('center_of_mass_valid') is not True
            or motion.get('center_of_mass_position_frame') != 'world'
            or motion.get('center_of_mass_orientation_order') != 'wxyz'):
        raise ValueError('assisted COM pose rate lacks a valid world-frame native COM')
    position = _position(motion.get('center_of_mass_position'))
    orientation = _orientation(motion.get('center_of_mass_orientation'))
    previous = state.get('com_pose_rate')
    if previous is not None and stamp['index'] == previous['stamp']['index']:
        same_orientation = (np.allclose(orientation, previous['orientation'], rtol=0., atol=1e-12)
                            or np.allclose(orientation, -previous['orientation'], rtol=0., atol=1e-12))
        if (stamp != previous['stamp'] or not np.array_equal(position, previous['position'])
                or not same_orientation):
            raise ValueError('assisted COM pose changed under a duplicate native clock')
        return previous, False
    valid = False
    reset = previous is not None
    linear = angular = None
    interval = None
    count = 1
    if previous is not None:
        old = previous['stamp']
        if stamp['index'] < old['index'] or stamp['time'] <= old['time']:
            raise ValueError('assisted COM pose native clock reversed')
        interval = stamp['time'] - old['time']
        valid = bool(stamp['index'] == old['index'] + 1
                     and abs(stamp['dt'] - old['dt']) < 1e-12
                     and abs(interval - stamp['dt']) < 1e-8)
        if valid:
            distance = float(np.linalg.norm(position - previous['position']))
            delta = quat_multiply(orientation, quat_conjugate(previous['orientation']))
            angle = 2. * math.atan2(float(np.linalg.norm(delta[1:])), abs(float(delta[0])))
            linear, angular = float(distance / interval), float(angle / interval)
            count = previous['consecutive_samples'] + 1
            reset = False
    result = dict(stamp=dict(stamp), position=position.copy(), orientation=orientation.copy(),
                  valid=valid, linear_speed=linear, angular_speed=angular,
                  interval_s=interval, consecutive_samples=count)
    state['com_pose_rate'] = result
    return result, reset


def settled_transition_ready(state, stamp, motion, pose_ready, settings):
    """Qualify a stopped loaded handoff using independent native samples.

    This only gates phase completion. The retained drive target, captured
    grasp, native velocity and existing model/safety budgets are unchanged.
    """
    from .beam_assisted_state import consecutive_native_gate
    ticks = settings['stable_native_ticks']
    minimum = settings['minimum_hold_native_ticks']
    maximum = settings['maximum_hold_native_ticks']
    linear = float(settings['maximum_linear_speed_m_s'])
    angular = float(settings['maximum_angular_speed_rad_s'])
    if (any(type(v) is not int for v in (ticks, minimum, maximum))
            or not 2 <= ticks <= minimum < maximum
            or not all(math.isfinite(v) and v > 0 for v in (linear, angular))):
        raise ValueError('invalid assisted settled transition contract')
    valid = bool(stamp.get('valid') is True and type(stamp.get('epoch')) is int
                 and type(stamp.get('index')) is int and stamp['epoch'] >= 0 and stamp['index'] >= 0
                 and math.isfinite(float(stamp.get('time', math.nan)))
                 and math.isfinite(float(stamp.get('dt', math.nan)))
                 and stamp['time'] >= 0 and stamp['dt'] > 0
                 and motion.get('motion_valid') is True
                 and motion.get('physics_handle_valid') is True
                 and motion.get('body_path') and motion.get('motion_view_id') is not None)
    if not valid:
        raise ValueError('assisted settled transition lacks a live native sample')
    speeds = [float(motion.get(k, math.inf)) for k in ('linear_speed', 'angular_speed')]
    if not all(math.isfinite(v) and v >= 0 for v in speeds):
        raise ValueError('assisted settled transition has invalid native velocity')
    metric = settings.get('motion_metric', 'native_velocity')
    if metric not in {'native_velocity', 'native_com_pose_difference'}:
        raise ValueError('invalid assisted transition motion metric')
    identity = (stamp['epoch'], motion['body_path'], motion['motion_view_id'])
    if state.get('identity') != identity:
        state.clear()
        state['identity'] = identity
    measured = speeds
    rate_valid = True
    metric_detail = dict(motion_metric=metric)
    if metric == 'native_com_pose_difference':
        rate, reset = _consecutive_native_com_rate(state, stamp, motion)
        measured = [rate['linear_speed'], rate['angular_speed']]
        rate_valid = rate['valid']
        if reset:
            state.pop('hold_start', None)
            state.pop('native_gate', None)
        metric_detail.update(pose_rate_valid=rate_valid,
            pose_rate_interval_s=rate['interval_s'], pose_rate_native_samples=rate['consecutive_samples'],
            native_center_of_mass_position=rate['position'].tolist(),
            native_center_of_mass_orientation=rate['orientation'].tolist())
    if not pose_ready:
        state.pop('hold_start', None)
    elif 'hold_start' not in state:
        state['hold_start'] = dict(stamp)
    start = state.get('hold_start')
    age = stamp['index'] - start['index'] if start is not None else 0
    if age < 0:
        raise ValueError('assisted settled transition native clock reversed')
    quiet = bool(pose_ready and rate_valid and measured[0] <= linear and measured[1] <= angular)
    poses = {'object': dict(position=_position(motion['position']),
                           orientation=_orientation(motion['orientation']),
                           body_path=motion['body_path'], motion_view_id=motion['motion_view_id'])}
    qualified = consecutive_native_gate(state.setdefault('native_gate', {}), stamp, quiet, ticks, poses)
    ready = bool(pose_ready and age >= minimum and qualified)
    if pose_ready and age >= maximum and not ready:
        raise ValueError('assisted settled transition did not settle within its native hold budget')
    return ready, dict(ready=ready, pose_ready=bool(pose_ready), quiet=quiet,
                       hold_native_ticks=age, stable_native_ticks=state['native_gate'].get('count', 0),
                       linear_speed_m_s=speeds[0], angular_speed_rad_s=speeds[1],
                       qualified_linear_speed_m_s=measured[0], qualified_angular_speed_rad_s=measured[1],
                       **metric_detail,
                       captured_grasp_reference_changed=False)


def validate_model_native_residual(native_object_pose, measured_fk_object_pose,
                                   position_limit=.0001, orientation_limit=.0015):
    for limit in (position_limit, orientation_limit):
        if not math.isfinite(float(limit)) or float(limit) <= 0:
            raise ValueError('invalid assisted FK/native residual budget')
    position, angle = pose_error(_position(native_object_pose['position']), _orientation(native_object_pose['orientation']),
                                 _position(measured_fk_object_pose['position']), _orientation(measured_fk_object_pose['orientation']))
    if position > position_limit or angle > orientation_limit:
        raise ValueError('assisted FK/native residual exceeds its model budget: '
                         + repr((position, angle)))
    return position, angle


def staged_clearance_target(state, stamp, current, target, settings):
    """Separate loaded clearance into lift, turn, transfer and lower.

    Stage changes use measured poses. The entry pose and world height stay
    fixed for this captured-grasp/phase context; publication caches survive.
    """
    if not isinstance(settings, dict):
        raise ValueError('staged clearance requires its reviewed settings')
    padding = float(settings['height_padding_m'])
    tolerance = float(settings['position_tolerance_m'])
    angle_tolerance = float(settings['orientation_tolerance_rad'])
    if not all(math.isfinite(v) and v > 0 for v in (padding, tolerance, angle_tolerance)):
        raise ValueError('invalid staged clearance limits')
    if stamp.get('valid') is not True or type(stamp.get('epoch')) is not int or type(stamp.get('index')) is not int:
        raise ValueError('staged clearance lacks native time')
    path = state.setdefault('clearance_path', {})
    clock = (stamp['epoch'], stamp['index'])
    p, q = _position(current['position']), _orientation(current['orientation'])
    goal, goal_q = _position(target['position']), _orientation(target['orientation'])
    if not path:
        path.update(entry_position=p.copy(), entry_orientation=q.copy(),
                    goal_position=goal.copy(), goal_orientation=goal_q.copy(),
                    height=max(float(p[2]), float(goal[2]))+padding, stage='lift')
    elif (np.max(abs(goal-path['goal_position'])) > 1e-10
          or abs(float(np.dot(goal_q, path['goal_orientation']))) < 1-1e-10):
        raise ValueError('staged clearance goal changed within its phase')
    old = path.get('clock')
    if old is not None and (clock[0] != old[0] or clock[1] < old[1]):
        raise ValueError('staged clearance clock changed')
    entry = path['entry_position']
    high_start = np.array([entry[0], entry[1], path['height']])
    high_end = np.array([goal[0], goal[1], path['height']])
    stages = [('lift', high_start, path['entry_orientation']),
              ('turn', high_start, goal_q), ('transfer', high_end, goal_q),
              ('lower', goal, goal_q)]
    index = next(i for i, s in enumerate(stages) if s[0] == path['stage'])
    _, position, orientation = stages[index]
    dp, da = pose_error(p, q, position, orientation)
    # At most one transition per independent native tick, even if policy and
    # completion code ask twice. Neither repeated calls nor elapsed wall time
    # can turn a low actual pose into a qualified high clearance.
    if old != clock and index < 3 and dp <= tolerance and da <= angle_tolerance:
        index += 1
    stage, position, orientation = stages[index]
    path.update(clock=clock, stage=stage)
    return {'position': position.copy(), 'orientation': orientation.copy()}


def absolute_object_servo(current_object_pose, target_object_pose,
                          object_in_tcp_position, object_in_tcp_orientation,
                          position_step, orientation_step, previous_object_command=None,
                          position_lookahead=.003, orientation_lookahead=.02,
                          convergence_axis=None, allow_axial_motion=True,
                          measured_fk_object_pose=None):
    """Accumulate object progress, then invert one fixed SE(3) grasp transform.

    TCP translation is never integrated. In particular, rotating the lever arm
    cannot add another lateral increment to an old TCP command. An object-space
    lead can overcome native drive tracking error, but cannot cross its goal.
    """
    for value in (position_step, orientation_step, position_lookahead, orientation_lookahead):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('assisted servo step and lookahead must be positive')
    current, goal = _position(current_object_pose['position']), _position(target_object_pose['position'])
    current_q, goal_q = _orientation(current_object_pose['orientation']), _orientation(target_object_pose['orientation'])
    previous = previous_object_command or current_object_pose
    anchor = _position(previous['position'])
    # Stale/out-of-phase commands have no authority beyond the measured lead.
    if np.linalg.norm(anchor-current) > position_lookahead + 1e-12:
        anchor = current.copy()
    delta = goal-current
    if convergence_axis is not None:
        axis = _position(convergence_axis)
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            raise ValueError('assisted servo needs a nonzero convergence axis')
        axis /= norm
        if not allow_axial_motion:
            delta -= np.dot(delta, axis)*axis
            anchor -= np.dot(anchor-current, axis)*axis
    distance = float(np.linalg.norm(delta))
    increment = delta * min(1., position_step/max(distance, 1e-15))
    candidate = anchor + increment
    # Component-wise bounds prevent an old lateral lead persisting after the
    # live object crosses the goal, including during a phase change.
    candidate = current + np.clip(candidate-current, np.minimum(delta, 0), np.maximum(delta, 0))
    if convergence_axis is not None and not allow_axial_motion:
        candidate -= np.dot(candidate-current, axis)*axis
    lead = candidate-current
    if np.linalg.norm(lead) > position_lookahead:
        candidate = current + lead*(position_lookahead/np.linalg.norm(lead))
    command_q = _orientation_step(previous['orientation'], goal_q, orientation_step)
    command_q = _orientation_step(current_q, command_q, orientation_lookahead)
    native_command = {'position': candidate, 'orientation': command_q}
    model_command = model_command_for_native_pose(current_object_pose,
        measured_fk_object_pose or current_object_pose, native_command)
    tcp_q = normalize_quat(quat_multiply(model_command['orientation'],
                          quat_conjugate(_orientation(object_in_tcp_orientation))))
    tcp_p = model_command['position']-quat_rotate(tcp_q, _position(object_in_tcp_position))
    return ({'position': tcp_p, 'orientation': tcp_q},
            native_command)


def interpolate_object_pose(previous, target, ratio):
    if not math.isfinite(ratio) or not 0 <= ratio <= 1:
        raise ValueError('invalid object increment fraction')
    angle = pose_error(previous['position'], previous['orientation'],
                       previous['position'], target['orientation'])[1]
    return dict(position=_position(previous['position'])+ratio*(
                    _position(target['position'])-_position(previous['position'])),
                orientation=(_orientation_step(previous['orientation'], target['orientation'],
                    max(1e-15, angle*ratio)) if ratio else _orientation(previous['orientation']).copy()))


def incremental_clearance_command(current, measured_model, previous_tcp, relative,
                                  target, stage, position_step, orientation_step,
                                  position_lookahead=.003, orientation_lookahead=.02,
                                  hold=False, drive_reference=None):
    """Add only the stage's new motion to the last FINAL drive pose.

    The existing loaded offset is never clipped to measured goal coordinates.
    Lead limits reduce the new increment or reject an invalid incoming target.
    This is target generation, not a rigid-body state or grasp-anchor update.
    """
    if stage not in ('lift', 'turn', 'transfer', 'lower'):
        raise ValueError('unknown incremental clearance stage')
    if any(not math.isfinite(float(x)) or x <= 0 for x in
           (position_step, orientation_step, position_lookahead, orientation_lookahead)):
        raise ValueError('invalid incremental clearance envelope')
    previous = object_pose_from_tcp(previous_tcp, *relative)
    if drive_reference is None:
        drive_reference=dict(position=previous['position']-_position(current['position']),
            orientation=normalize_quat(quat_multiply(previous['orientation'],quat_conjugate(_orientation(current['orientation'])))))
    drive_goal=dict(position=_position(target['position'])+_position(drive_reference['position']),
        orientation=normalize_quat(quat_multiply(_orientation(drive_reference['orientation']),_orientation(target['orientation']))))
    # Cap the commanded trajectory at the goal plus the retained ENTRY load
    # bias. Transient tracking lookahead is not a new equilibrium offset.
    delta = drive_goal['position']-previous['position']
    mask = np.array([1., 1., 0.]) if stage == 'transfer' else np.array([0., 0., 1.])
    if hold:
        delta = np.zeros(3)
    elif stage == 'turn':
        # A moving entry target is not a permanent load offset. Retract only
        # surplus upward drive if measured height overshoots high_start.
        # Never add turn translation or drop the full accepted SE(3) pose.
        # The model/native offset supplies a floor, not a new grasp anchor.
        floor = model_command_for_native_pose(current, measured_model, target)
        excess = max(0., float(_position(current['position'])[2]-_position(target['position'])[2]))
        surplus = max(0., float(previous['position'][2]-floor['position'][2]))
        delta = np.array([0., 0., -min(excess, surplus, position_step)])
    else:
        delta = delta*mask
        delta *= min(1., position_step/max(float(np.linalg.norm(delta)), 1e-15))
    rotation = previous['orientation'].copy()
    if stage == 'turn' and not hold:
        rotation = _orientation_step(previous['orientation'],drive_goal['orientation'],orientation_step)
    for ratio in (1., .5, .25, .125, .0625, 0.):
        model = interpolate_object_pose(previous,
            dict(position=previous['position']+delta, orientation=rotation), ratio)
        native = anchored_object_prediction(current, measured_model, model)
        distance, angle = pose_error(current['position'], current['orientation'],
                                     native['position'], native['orientation'])
        if distance <= position_lookahead+1e-12 and angle <= orientation_lookahead+1e-12:
            q = normalize_quat(quat_multiply(model['orientation'], quat_conjugate(_orientation(relative[1]))))
            tcp = dict(position=model['position']-quat_rotate(q, _position(relative[0])), orientation=q)
            return tcp, model, dict(stage=stage, added_position_m=ratio*delta,
                added_orientation_rad=pose_error(previous['position'], previous['orientation'],
                    previous['position'], model['orientation'])[1], lead_ratio=ratio,
                predicted_native_pose=native, previous_model_pose=previous, drive_goal_model_pose=drive_goal, retained_entry_drive_bias=drive_reference)
    raise ValueError('incoming final publication exceeds original clearance lead envelope')


def descent_admission(state, stamp, current_object_pose, target_object_pose,
                       convergence_axis, contract, motion_detail):
    """Qualify measured alignment on consecutive physical ticks, fail closed."""
    from .beam_assisted_state import consecutive_native_gate
    required = ('lateral_tolerance_m', 'orientation_tolerance_rad', 'stable_native_ticks',
                'max_linear_speed_m_s', 'max_angular_speed_rad_s', 'geometry_certificate')
    if not isinstance(contract, dict) or any(k not in contract for k in required):
        raise ValueError('assisted descent lacks its complete geometry admission contract')
    for key in required[:2] + required[3:5]:
        if not math.isfinite(float(contract[key])) or float(contract[key]) <= 0:
            raise ValueError('assisted descent tolerances and speed bounds must be positive')
    if (type(contract['stable_native_ticks']) is not int or contract['stable_native_ticks'] < 2
            or not isinstance(contract['geometry_certificate'], str) or not contract['geometry_certificate']):
        raise ValueError('assisted descent needs multiple native ticks and a geometry certificate')
    axis = _position(convergence_axis)
    norm = float(np.linalg.norm(axis))
    if norm < 1e-12:
        raise ValueError('assisted descent needs a nonzero convergence axis')
    axis /= norm
    delta = _position(target_object_pose['position'])-_position(current_object_pose['position'])
    lateral = float(np.linalg.norm(delta-np.dot(delta, axis)*axis))
    _, angle = pose_error(current_object_pose['position'], current_object_pose['orientation'],
                           target_object_pose['position'], target_object_pose['orientation'])
    motion = motion_detail or {}
    valid = bool(motion.get('motion_valid') is True and motion.get('physics_handle_valid') is True
                 and motion.get('body_path') and motion.get('motion_view_id') is not None)
    try:
        linear, angular = float(motion['linear_speed']), float(motion['angular_speed'])
        valid = bool(valid and math.isfinite(linear) and math.isfinite(angular))
    except (KeyError, TypeError, ValueError):
        linear, angular, valid = math.inf, math.inf, False
    metric = contract.get('motion_metric', 'native_velocity')
    if metric not in {'native_velocity', 'native_com_pose_difference'}:
        raise ValueError('invalid assisted descent motion metric')
    qualified_linear, qualified_angular = linear, angular
    qualified_velocity = motion.get('linear_velocity')
    rate_valid, metric_detail = True, {}
    if metric == 'native_com_pose_difference':
        stamp_valid = bool(stamp.get('valid') is True and type(stamp.get('epoch')) is int
            and type(stamp.get('index')) is int and stamp['epoch'] >= 0 and stamp['index'] >= 0
            and math.isfinite(float(stamp.get('time', math.nan)))
            and math.isfinite(float(stamp.get('dt', math.nan)))
            and stamp['time'] >= 0 and stamp['dt'] > 0)
        if not (valid and stamp_valid and linear >= 0 and angular >= 0):
            raise ValueError('assisted descent COM rate lacks a live native sample')
        identity = (stamp['epoch'], motion['body_path'], motion['motion_view_id'])
        if state.get('motion_metric_identity') != identity:
            state.clear()
            state['motion_metric_identity'] = identity
        samples = state.setdefault('motion_metric_samples', {})
        previous = samples.get('com_pose_rate')
        rate, _ = _consecutive_native_com_rate(samples, stamp, motion)
        rate_valid = rate['valid']
        if rate_valid and previous is not None and previous['stamp'] != stamp:
            rate['linear_velocity'] = ((rate['position']-previous['position'])
                                       /rate['interval_s'])
        qualified_linear = rate['linear_speed'] if rate_valid else math.inf
        qualified_angular = rate['angular_speed'] if rate_valid else math.inf
        qualified_velocity = rate.get('linear_velocity') if rate_valid else None
        metric_detail = dict(motion_metric=metric, pose_rate_valid=rate_valid,
            pose_rate_interval_s=rate['interval_s'], pose_rate_native_samples=rate['consecutive_samples'],
            qualified_linear_speed_m_s=rate['linear_speed'], qualified_angular_speed_rad_s=rate['angular_speed'],
            qualified_linear_velocity_world=(qualified_velocity.tolist() if qualified_velocity is not None else None),
            native_center_of_mass_position=rate['position'].tolist(),
            native_center_of_mass_orientation=rate['orientation'].tolist())
    admission_warm = state.get('count', 0) >= contract['stable_native_ticks']
    continuing_linear_ready = qualified_linear <= contract['max_linear_speed_m_s']
    if admission_warm and qualified_velocity is not None:
        velocity = _position(qualified_velocity)
        axial_speed = abs(float(np.dot(velocity, axis)))
        lateral_speed = float(np.linalg.norm(velocity-np.dot(velocity, axis)*axis))
        continuing_linear_ready = bool(lateral_speed <= contract['max_linear_speed_m_s']
            and axial_speed <= float(contract.get('max_axial_speed_m_s', .05)))
    # Warm-up requires settling. Once admitted, axial insertion is the intended
    # motion and must not erase the alignment qualification on every step.
    linear_ready = continuing_linear_ready if admission_warm else qualified_linear <= contract['max_linear_speed_m_s']
    aligned = bool(valid and rate_valid and lateral <= contract['lateral_tolerance_m']
                   and angle <= contract['orientation_tolerance_rad']
                   and linear_ready
                   and qualified_angular <= contract['max_angular_speed_rad_s'])
    # The pose signature also rejects changed views and changed observations at
    # the same physical tick; no number of policy calls can warm this gate.
    raw_signature = tuple(_position(current_object_pose['position'])) + tuple(_orientation(current_object_pose['orientation']))
    previous_stamp, previous_signature = state.get('raw_stamp'), state.get('raw_signature')
    if (previous_stamp == stamp and previous_signature is not None and previous_signature != raw_signature):
        aligned = False
    state.update(raw_stamp=dict(stamp), raw_signature=raw_signature)
    projected_pose = {**current_object_pose,
        'position': _position(current_object_pose['position'])
                    -np.dot(_position(current_object_pose['position']), axis)*axis}
    poses = {'object': {**projected_pose, 'body_path': motion.get('body_path'),
                        'motion_view_id': motion.get('motion_view_id')}} if valid else None
    ready = consecutive_native_gate(state, stamp, aligned, contract['stable_native_ticks'], poses)
    return ready, {'ready': ready, 'aligned': aligned, 'lateral_error_m': lateral,
                   'orientation_error_rad': angle, 'linear_speed_m_s': linear,
                   'angular_speed_rad_s': angular, 'native_count': state.get('count', 0),
                   'geometry_certificate': contract['geometry_certificate'],
                   'alignment_was_warm': bool(admission_warm), **metric_detail}


def published_object_command_safe(current_object_pose, predicted_object_pose, target_object_pose,
                                  convergence_axis, contract, descent_ready):
    """Check FK *after* all joint clamps; numerical slack is part of the contract."""
    axis = _position(convergence_axis)
    axis /= np.linalg.norm(axis)
    current = _position(current_object_pose['position'])
    predicted = _position(predicted_object_pose['position'])
    goal = _position(target_object_pose['position'])
    slack = float(contract.get('published_axial_slack_m', .00002))
    lateral_slack = float(contract.get('published_lateral_slack_m', .00002))
    if not math.isfinite(slack) or slack < 0 or not math.isfinite(lateral_slack) or lateral_slack < 0:
        raise ValueError('invalid assisted published-command slack')
    def lateral(p):
        d = goal-p
        return float(np.linalg.norm(d-np.dot(d, axis)*axis))
    axial_motion = float(np.dot(predicted-current, axis))
    current_error, predicted_error = lateral(current), lateral(predicted)
    _, predicted_angle = pose_error(predicted, predicted_object_pose['orientation'], goal,
                                     target_object_pose['orientation'])
    _, current_angle = pose_error(current, current_object_pose['orientation'], goal,
                                  target_object_pose['orientation'])
    # Re-alignment may start outside the corridor; it may not make that error
    # worse. Once descent is admitted, the predicted object must remain inside.
    lateral_ok = predicted_error <= max(current_error, contract['lateral_tolerance_m']) + lateral_slack
    corridor_ok = (not descent_ready or (predicted_error <= contract['lateral_tolerance_m'] + lateral_slack
                    and predicted_angle <= contract['orientation_tolerance_rad'] + .0001))
    axial_ok = descent_ready or axial_motion <= slack
    orientation_ok = predicted_angle <= max(current_angle, contract['orientation_tolerance_rad']) + .0001
    return bool(lateral_ok and corridor_ok and axial_ok and orientation_ok), {
        'predicted_axial_motion_m': axial_motion, 'current_lateral_error_m': current_error,
        'predicted_lateral_error_m': predicted_error, 'predicted_orientation_error_rad': predicted_angle,
        'descent_ready': bool(descent_ready), 'lateral_ok': bool(lateral_ok),
        'corridor_ok': bool(corridor_ok), 'axial_ok': bool(axial_ok),
        'orientation_ok': bool(orientation_ok)}
