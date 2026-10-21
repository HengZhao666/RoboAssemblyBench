"""Motion-only qualification of copied native Beam actor and COM poses.

There are no SDK calls, environment reads, or physical writes in this module.
The caller must provide COM and actor poses from the same existing live view
and physical sample, and must independently qualify support, grasp and safety.
One tracker belongs to one phase and one body. Create a new tracker explicitly
when entering a new phase; clock gaps and view replacement never rebase an
already captured cumulative-motion anchor.
"""
from __future__ import annotations

from collections import deque
from collections.abc import Mapping
import copy
import math
from numbers import Real


def _number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _vector(values, size):
    if isinstance(values, (str, bytes, Mapping)):
        return None
    try:
        values = list(values)
    except TypeError:
        return None
    if len(values) != size or not all(_number(value) for value in values):
        return None
    return tuple(float(value) for value in values)


def _distance(left, right):
    return _norm(a-b for a, b in zip(left, right))


def _norm(values):
    result = 0.
    for value in values:
        result = math.hypot(result, value)
    return result


def _quaternion(values):
    values = _vector(values, 4)
    length = _norm(values) if values else 0.
    return tuple(value/length for value in values) if math.isfinite(length) and length > 1.e-12 else None


def quaternion_angle(current, previous):
    """Shortest SO(3) angle in radians, robust for q/-q and small rotations."""
    a, b = _quaternion(current), _quaternion(previous)
    if a is None or b is None:
        raise ValueError('Invalid wxyz quaternion')
    return _unit_angle(a, b)


def _unit_angle(a, b):
    """Inputs have already been normalized and copied at sample ingestion."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b[0], -b[1], -b[2], -b[3]
    relative = (aw*bw-ax*bx-ay*by-az*bz,
                aw*bx+ax*bw+ay*bz-az*by,
                aw*by-ax*bz+ay*bw+az*bx,
                aw*bz+ax*by-ay*bx+az*bw)
    return 2.*math.atan2(_norm(relative[1:]), abs(relative[0]))


def _clock(sample):
    epoch, index = sample.get('physics_epoch'), sample.get('physics_step_index')
    time, dt = sample.get('physics_time'), sample.get('physics_dt')
    if (sample.get('physics_stamp_valid') is not True
            or type(epoch) is not int or epoch < 0 or type(index) is not int or index < 0
            or not _number(time) or time < 0 or not _number(dt) or dt <= 0):
        return None
    return epoch, index, float(time), float(dt)


def _clock_transition(previous, current):
    if previous is None:
        return 'first'
    epoch, index, time, dt = current
    old_epoch, old_index, old_time, old_dt = previous
    if epoch != old_epoch:
        return 'epoch_change'
    if not math.isclose(dt, old_dt, rel_tol=1.e-9, abs_tol=1.e-12):
        return 'dt_change'
    if index == old_index:
        return 'duplicate' if time == old_time else 'invalid'
    if index < old_index or time <= old_time:
        return 'rollback'
    if (index != old_index+1 or not math.isclose(time-old_time, dt, rel_tol=1.e-6,
                                               abs_tol=max(1.e-10, dt*1.e-6))):
        return 'gap'
    return 'next'


class PoseStabilityTracker:
    """Two physical-time windows plus a one-time cumulative-motion anchor.

    ``observe`` accepts the existing table sample and explicit same-frame
    world COM/identity arguments. ``anchor_eligible`` is a caller-owned boolean;
    this class never reads force/gap and never proves support. The anchor is
    captured only after BOTH motion windows qualify and eligibility is true,
    allowing normal initial seating descent without a phase-entry pose budget.
    Once captured it is preserved across all invalidations. Exceeding its
    budget or changing body identity latches rejection until a new tracker is
    explicitly created. View/clock changes clear windows and require rewarm.
    ``ready`` requires both motion windows and a captured, valid anchor. It
    cannot be used alone as an assembly or current table-support predicate.
    """

    def __init__(self, *, short_window=.4, long_window=2., position_budget=.0003,
                 orientation_budget=.005, linear_speed_limit=.002,
                 angular_speed_limit=.03, anchor_position_budget=.0003,
                 anchor_orientation_budget=.005):
        values = locals().copy()
        values.pop('self')
        if any(not _number(value) or value <= 0 for value in values.values()) or long_window < short_window:
            raise ValueError('Finite positive budgets and long_window >= short_window are required')
        self._config = {key: float(value) for key, value in values.items()}
        self._history = deque()
        self._last = None
        self._body_id = None
        self._anchor = None
        self._latched_reason = None

    def _invalidate(self):
        self._history.clear()
        self._last = None

    def _window(self, seconds):
        if not self._history:
            return {'valid': False, 'full': False, 'ready': False, 'samples': 0, 'duration': 0.}
        items = list(self._history)
        cutoff = items[-1]['clock'][2]-seconds
        start = 0
        while start+1 < len(items) and items[start+1]['clock'][2] <= cutoff+1.e-10:
            start += 1
        items = items[start:]
        first, last = items[0], items[-1]
        duration = last['clock'][2]-first['clock'][2]
        result = {'valid': True, 'full': len(items) > 1 and duration > 0 and duration >= seconds-1.e-10, 'ready': False,
                  'samples': len(items), 'duration': duration,
                  'diameter_method': 'conservative_twice_max_distance_from_first'}
        if not result['full']:
            return result
        edges = items[1:]
        metrics = {}
        for field in ('actor', 'com'):
            metrics[field+'_diameter_upper_bound'] = 2.*max(_distance(item[field], first[field]) for item in items)
            metrics[field+'_path_speed'] = sum(item['edge'][field] for item in edges)/duration
            metrics[field+'_max_step_speed'] = max(item['edge'][field]/item['edge']['dt'] for item in edges)
            metrics[field+'_endpoint_drift_speed'] = _distance(last[field], first[field])/duration
        metrics['orientation_diameter_upper_bound'] = 2.*max(_unit_angle(item['q'], first['q']) for item in items)
        metrics['angular_path_speed'] = sum(item['edge']['angle'] for item in edges)/duration
        metrics['angular_max_step_speed'] = max(item['edge']['angle']/item['edge']['dt'] for item in edges)
        metrics['angular_endpoint_drift_speed'] = _unit_angle(last['q'], first['q'])/duration
        limits = {field+'_diameter_upper_bound': self._config['position_budget'] for field in ('actor', 'com')}
        limits['orientation_diameter_upper_bound'] = self._config['orientation_budget']
        for field in ('actor', 'com'):
            for metric in ('path_speed', 'max_step_speed', 'endpoint_drift_speed'):
                limits[field+'_'+metric] = self._config['linear_speed_limit']
        for metric in ('path_speed', 'max_step_speed', 'endpoint_drift_speed'):
            limits['angular_'+metric] = self._config['angular_speed_limit']
        metrics = {key: value if math.isfinite(value) else None for key, value in metrics.items()}
        result.update(metrics=metrics, conditions={key: value is not None and value <= limits[key] for key, value in metrics.items()})
        result['ready'] = all(result['conditions'].values())
        return result

    def _anchor_detail(self, current=None, captured=False):
        if self._anchor is None:
            return {'captured': False, 'captured_now': False, 'within_budget': None}
        result = {'captured': True, 'captured_now': captured,
                  'physics_epoch': self._anchor['clock'][0], 'physics_step_index': self._anchor['clock'][1],
                  'physics_time': self._anchor['clock'][2], 'body_id': self._body_id,
                  'actor_position': list(self._anchor['actor']), 'com_position': list(self._anchor['com']),
                  'orientation': list(self._anchor['q']),
                  'position_budget': self._config['anchor_position_budget'],
                  'orientation_budget': self._config['anchor_orientation_budget'], 'within_budget': None}
        if current is not None:
            result.update(actor_displacement=_distance(current['actor'], self._anchor['actor']),
                          com_displacement=_distance(current['com'], self._anchor['com']),
                          orientation_displacement=_unit_angle(current['q'], self._anchor['q']))
            position_valid = all(math.isfinite(result[key]) for key in ('actor_displacement', 'com_displacement'))
            result['within_budget'] = (position_valid and max(result['actor_displacement'], result['com_displacement']) <= self._config['anchor_position_budget']
                                       and result['orientation_displacement'] <= self._config['anchor_orientation_budget'])
            for key in ('actor_displacement', 'com_displacement'):
                if not math.isfinite(result[key]):
                    result[key] = None
        return result

    def _result(self, *, valid, reason, transition, fresh=False, current=None,
                anchor_eligible=False, captured=False, windows=None):
        short, long = windows if windows is not None else (self._window(self._config[key]) for key in ('short_window', 'long_window'))
        anchor = self._anchor_detail(current, captured)
        motion_ready = bool(valid and short['ready'] and long['ready'])
        ready = bool(motion_ready and self._latched_reason is None
                     and anchor['captured'] and anchor['within_budget'] is True)
        if motion_ready and not anchor['captured'] and reason is None:
            reason = 'awaiting_anchor_eligibility'
        stamp = dict(zip(('physics_epoch', 'physics_step_index', 'physics_time', 'physics_dt'), current['clock'])) if current else None
        result = {'valid': valid, 'ready': ready, 'motion_windows_ready': motion_ready,
                  'scope': 'motion_only', 'support_verified': False, 'fresh_sample': fresh,
                  'clock_transition': transition, 'reason': self._latched_reason or reason,
                  'short_window': short, 'long_window': long, 'anchor': anchor,
                  'physics_stamp': stamp, 'anchor_captured': anchor['captured'],
                  'anchor_eligible': anchor_eligible, 'latched_failure': self._latched_reason is not None,
                  'thresholds': dict(self._config)}
        return result

    def observe(self, sample, *, com_position, body_id, view_id, anchor_eligible=False):
        """Observe one copied world-pose sample; duplicates never add duration.

        Required sample fields are the table sample's valid/motion_valid,
        physics_stamp_valid/epoch/index/time/dt, position and wxyz orientation.
        ``com_position`` has no actor-origin fallback. ``body_id`` is the prim
        path and ``view_id`` identifies the same existing motion-view instance.
        Eligibility can be supplied later on a duplicate without adding samples;
        such a duplicate cannot recapture or move an existing anchor.
        """
        if not isinstance(sample, Mapping):
            self._invalidate()
            return self._result(valid=False, reason='invalid_sample', transition='invalid')
        clock = _clock(sample)
        actor, com, q = _vector(sample.get('position'), 3), _vector(com_position, 3), _quaternion(sample.get('orientation'))
        identity_valid = (isinstance(body_id, str) and bool(body_id)
                          and isinstance(view_id, (str, int)) and not isinstance(view_id, bool)
                          and (not isinstance(view_id, str) or bool(view_id)))
        if (clock is None or sample.get('valid') is not True or sample.get('motion_valid') is not True
                or actor is None or com is None or q is None or not identity_valid
                or type(anchor_eligible) is not bool):
            self._invalidate()
            return self._result(valid=False, reason='invalid_clock_or_native_pose', transition='invalid')
        current = {'actor': actor, 'com': com, 'q': q, 'clock': clock, 'view_id': view_id}
        if self._body_id is None:
            self._body_id = body_id
        elif self._body_id != body_id:
            self._latched_reason = 'body_identity_changed_requires_new_tracker'
            self._invalidate()
            return self._result(valid=False, reason=self._latched_reason, transition='body_change')
        transition = _clock_transition(self._last['clock'] if self._last else None, clock)
        if self._last is not None and self._last['view_id'] != view_id:
            transition = 'view_change'
        if transition == 'invalid':
            self._invalidate()
            return self._result(valid=False, reason='inconsistent_physics_clock', transition=transition)
        if transition == 'duplicate':
            if (actor != self._last['actor'] or com != self._last['com']
                    or _unit_angle(q, self._last['q']) > 1.e-12):
                self._invalidate()
                return self._result(valid=False, reason='duplicate_pose_changed', transition='invalid')
            current = self._last
        else:
            if transition != 'next':
                self._invalidate()
            previous = self._last
            current['edge'] = {'actor': _distance(actor, previous['actor']) if previous else 0.,
                               'com': _distance(com, previous['com']) if previous else 0.,
                               'angle': _unit_angle(q, previous['q']) if previous else 0.,
                               'dt': clock[2]-previous['clock'][2] if previous else 0.}
            self._history.append(current)
            self._last = current
            cutoff = clock[2]-self._config['long_window']
            while len(self._history) > 1 and self._history[1]['clock'][2] <= cutoff+1.e-10:
                self._history.popleft()
        short, long = (self._window(self._config[key]) for key in ('short_window', 'long_window'))
        captured = False
        if (self._anchor is None and self._latched_reason is None and anchor_eligible
                and short['ready'] and long['ready']):
            self._anchor = copy.deepcopy(current)
            captured = True
        if self._anchor is not None and self._anchor_detail(current)['within_budget'] is False:
            self._latched_reason = 'cumulative_anchor_budget_exceeded'
        reason = None if short['ready'] and long['ready'] else 'warming_or_motion_exceeds_budget'
        return self._result(valid=True, reason=reason, transition=transition,
                            fresh=transition != 'duplicate', current=current,
                            anchor_eligible=anchor_eligible, captured=captured, windows=(short, long))
