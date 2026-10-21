"""Clock metadata for retained Beam images; never advance physics or rendering."""
import math
import numbers


def render_reference_time(value):
    """Preserve Replicator's rational render time; missing data stays unknown."""
    if not isinstance(value, dict):
        return {'valid': False, 'reason': 'reference_time_unavailable'}
    if value.get('error'):
        return {'valid': False, 'reason': str(value['error'])}
    numerator = value.get('referenceTimeNumerator')
    denominator = value.get('referenceTimeDenominator')
    if (isinstance(numerator, bool) or isinstance(denominator, bool)
            or not isinstance(numerator, numbers.Integral)
            or not isinstance(denominator, numbers.Integral)
            or numerator < 0 or denominator <= 0):
        return {'valid': False, 'reason': 'invalid_reference_time_rational'}
    seconds = int(numerator) / int(denominator)
    if not math.isfinite(seconds):
        return {'valid': False, 'reason': 'nonfinite_reference_time'}
    return {'valid': True, 'numerator': int(numerator),
            'denominator': int(denominator), 'seconds': seconds,
            'source': 'same_render_product.ReferenceTime'}


def video_frame_clock(task, *, step, camera, frame_index, sensor_observation):
    """Record both clocks without treating the task label as physical time."""
    try:
        stamp = dict(task._beam_physics_stamp())
    except Exception as exc:
        stamp = {'valid': False, 'reason': type(exc).__name__ + ': ' + str(exc)}
    reference = render_reference_time((sensor_observation or {}).get('render_reference_time'))
    result = {'task_step_label': int(step), 'camera': camera,
              'frame_index': int(frame_index), 'physics_stamp': stamp,
              'render_reference_time': reference,
              'clock_relation': 'unverified; distinct clocks retained'}
    if stamp.get('valid') and reference['valid']:
        result['physics_time_minus_render_time_s'] = float(stamp['time']) - reference['seconds']
    return result
