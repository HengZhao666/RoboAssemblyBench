"""Summarize reached visual-support evidence without importing simulation code.

Usage: python tools/summarize_C_support.py RUN_NAME
Only recorded samples are qualified; this never asserts force support or assembly
success. A missing/empty trace cannot pass. Run after the CAD monitor has ended.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math


FINGERS = ('left_inner_finger', 'right_inner_finger')
ACQUISITION = 'base_6_visual_support'
FILES = ('task_config_snapshot.json', 'collect_results.json',
         'visual_support_trace.jsonl', 'visible_mesh_runtime_report.json',
         'entity_geometry_trace.jsonl', 'physics_boundary_trace.jsonl', 'finished_at')


def digest(path):
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def load_json(path, errors):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError) as exc:
        errors.append({'file': path.name, 'error': type(exc).__name__ + ': ' + str(exc)})
        return None


def json_lines(path, errors):
    if not path.is_file():
        errors.append({'file': path.name, 'error': 'missing_file'})
        return
    with path.open() as stream:
        for index, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError('JSONL row is not an object')
                yield row
            except ValueError as exc:
                errors.append({'file': path.name, 'line': index, 'error': str(exc)})


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite_original_number': repr(value)}
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def pose_matrix(body):
    if not body.get('valid') or not body.get('physics_handle_valid'):
        raise ValueError('invalid native body/handle')
    if body.get('pose_frame') != 'world' or body.get('orientation_order') != 'wxyz':
        raise ValueError('unsupported native pose convention')
    p, q = body['positions'][0], body['orientations'][0]
    if len(p) != 3 or len(q) != 4 or not all(finite(x) for x in [*p, *q]):
        raise ValueError('invalid native pose values')
    norm = math.sqrt(sum(x*x for x in q))
    if norm == 0:
        raise ValueError('zero native quaternion')
    w, x, y, z = (v / norm for v in q)
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), p[0]],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), p[1]],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), p[2]],
            [0., 0., 0., 1.]]


def original_report_scope(report):
    """Preserve reported errors/skips and coverage, including explicit absences."""
    if not isinstance(report, dict):
        return {'reported': False}
    fields = ('enabled', 'mode', 'scope', 'geometry_mode', 'check_stride',
              'observed_steps', 'checks', 'samples', 'meshes', 'pair_checks',
              'full_entity_sweep_verified', 'frame_coverage', 'entity_geometry_coverage',
              'gripper_prediction', 'self_checked', 'self_collision_checked',
              'continuous_ccd_checked', 'physx_cooked_decomposition_or_sdf_reconstructed',
              'errors', 'monitor_error', 'geometry_errors', 'entity_geometry_errors',
              'entity_geometry_initialize_errors', 'skip_reasons', 'skipped', 'missing',
              'missing_prims', 'missing_object_geometry', 'missing_static_geometry',
              'unknown_contact_offset_colliders', 'gaps', 'events_retention',
              'events_dropped', 'events_dropped_from_summary', 'violations_by_kind',
              'violation_total', 'intersections', 'failures_above_precision',
              'visual_support_samples', 'visual_support_failures', 'finished_marker_seen',
              'stop_requested', 'last_step', 'source_sha256',
              'failure_tail_has_separate_native_trace_not_this_entity_stream',
              'native_errors', 'native_samples', 'native_valid_samples', 'capture_complete', 'native_complete')
    out = {'reported': True, 'reported_fields': {k: report[k] for k in fields if k in report},
           'not_reported_fields': [k for k in fields if k not in report]}
    if isinstance(report.get('entity_geometry'), dict):
        out['entity_geometry'] = original_report_scope(report['entity_geometry'])
    return out


def summarize(run_dir):
    errors = []
    before = {name: digest(run_dir / name) for name in FILES}
    config = load_json(run_dir / FILES[0], errors)
    collected = load_json(run_dir / FILES[1], errors)
    raw = load_json(run_dir / 'visible_mesh_runtime_report.json', errors)
    try:
        phases = config['task_configs'][0]['phase_specs']
        contracts = {p['name']: p['beam_assisted_visual_support'] for p in phases
                     if p.get('beam_assisted_visual_support')}
    except (KeyError, TypeError, IndexError) as exc:
        errors.append({'file': FILES[0], 'error': 'invalid phase configuration: ' + str(exc)})
        contracts = {}
    if not isinstance(collected, list) or len(collected) != 1 or not isinstance(collected[0], dict):
        errors.append({'file': FILES[1], 'error': 'expected exactly one task result'})
        result = {}
    else:
        result = collected[0]
    reached = set(result.get('phase_history', []))
    for event in result.get('phase_transition_history', []):
        reached.update(p for p in (event.get('phase'), event.get('from_phase'), event.get('to_phase')) if p)
    reached.update(p for p in (result.get('phase'),) if p)
    stats = {}

    def phase_stats(phase):
        return stats.setdefault(phase, {'phase': phase, 'acquisition_phase': phase == ACQUISITION,
            'trace_samples': 0, 'ready_samples': 0, 'failed_samples': 0,
            'same_native_pose_samples': 0, 'expected_entity_samples': 0,
            'left_inner_finger_gap_m': {'min': None, 'max': None},
            'right_inner_finger_gap_m': {'min': None, 'max': None}})

    for phase in contracts:
        if phase in reached:
            phase_stats(phase)
    trace = {}
    failures = []
    for row in json_lines(run_dir / 'visual_support_trace.jsonl', errors):
        phase, step = row.get('phase'), row.get('step')
        if phase not in contracts or not isinstance(step, int) or isinstance(step, bool):
            errors.append({'file': 'visual_support_trace.jsonl', 'error': 'unknown phase or invalid step',
                           'phase': phase, 'step': step})
            continue
        if phase not in reached:
            errors.append({'phase': phase, 'step': step, 'error': 'trace phase absent from collected history'})
        st = phase_stats(phase)
        st['trace_samples'] += 1
        key = (step, phase)
        if key in trace:
            errors.append({'phase': phase, 'step': step, 'error': 'duplicate visual support sample'})
        trace[key] = row
        contract = contracts[phase]
        rows = row.get('fingers', [])
        if not isinstance(rows, list):
            rows = []
        by_finger = {f.get('finger'): f for f in rows if isinstance(f, dict)}
        valid = (row.get('ready') is True and row.get('force_support_claimed') is False
                 and row.get('robot') == contract.get('robot') and row.get('object') == contract.get('object')
                 and len(rows) == 2 and set(by_finger) == set(FINGERS))
        for finger in FINGERS:
            f = by_finger.get(finger, {})
            gap = f.get('signed_gap_m')
            if finite(gap):
                limits = st[finger + '_gap_m']
                limits['min'] = gap if limits['min'] is None else min(limits['min'], gap)
                limits['max'] = gap if limits['max'] is None else max(limits['max'], gap)
            valid = bool(valid and finite(gap) and f.get('within_visual_support_gap') is True
                         and isinstance(f.get('mesh_pairs'), int) and f['mesh_pairs'] > 0
                         and contract['minimum_surface_gap_m'] <= gap <= contract['maximum_surface_gap_m'])
        st['ready_samples' if valid else 'failed_samples'] += 1
        if not valid:
            failures.append(row)

    # The support watcher has no native stamp. Bind it to its saved entity poses,
    # then compare all three support bodies against one valid before_action row.
    entities = {}
    for row in json_lines(run_dir / 'entity_geometry_trace.jsonl', errors):
        phase, step = row.get('phase'), row.get('step')
        if phase not in contracts or phase == ACQUISITION:
            continue
        st = phase_stats(phase)
        st['expected_entity_samples'] += 1
        key = (step, phase)
        if key in entities:
            errors.append({'phase': phase, 'step': step, 'error': 'duplicate entity geometry sample'})
        obj = contracts[phase]['object']
        entities[key] = {p: t for p, t in row.get('body_poses', {}).items()
                         if p.endswith('/' + obj) or any(p.endswith('/' + f) for f in FINGERS)}
    matched = set()
    native_matches = []
    for row in json_lines(run_dir / 'physics_boundary_trace.jsonl', errors):
        key = (row.get('cycle_step'), row.get('cycle_phase'))
        if row.get('boundary') != 'before_action' or key not in trace:
            continue
        step, phase = key
        try:
            if (not row.get('valid') or not row.get('clock_valid') or row.get('task_step') != step
                    or row.get('phase') != phase or not isinstance(row.get('physics_step_index'), int)
                    or not finite(row.get('physics_time'))):
                raise ValueError('invalid or mismatched native clock/phase')
            if key in matched:
                raise ValueError('duplicate native before_action sample')
            contract = contracts[phase]
            names = (contract['object'], *(contract['robot'] + '/' + f for f in FINGERS))
            bodies = {b.get('name'): b for b in row.get('bodies', []) if b.get('name') in names}
            if set(bodies) != set(names) or key not in entities:
                raise ValueError('missing support native body or saved entity sample')
            max_error = 0.
            for body in bodies.values():
                actual = pose_matrix(body)
                entity = entities[key][body['path']]
                if len(entity) != 4 or any(len(line) != 4 for line in entity):
                    raise ValueError('invalid entity pose matrix')
                if not all(finite(v) for line in entity for v in line):
                    raise ValueError('nonfinite entity pose matrix')
                max_error = max(max_error, max(abs(actual[i][j]-entity[i][j])
                                               for i in range(4) for j in range(4)))
            if max_error > 1e-10:
                raise ValueError('saved support poses differ from same-clock native bodies: ' + str(max_error))
            matched.add(key)
            stats[phase]['same_native_pose_samples'] += 1
            native_matches.append({'step': step, 'phase': phase, 'physics_step_index': row['physics_step_index'],
                                   'physics_time': row['physics_time'], 'maximum_pose_matrix_error': max_error})
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            errors.append({'phase': phase, 'step': step, 'error': 'native binding: ' + str(exc)})
    missing_trace = sorted(set(entities) - set(trace))
    missing_entity = sorted(set(trace) - set(entities))
    missing_native = sorted(set(trace) - matched)
    for phase, st in stats.items():
        st['contract'] = contracts[phase]
        if phase == ACQUISITION:
            st['status'] = 'acquisition_skill_phase; maintenance CAD sampling intentionally begins afterward'
        elif not st['trace_samples']:
            st['status'] = 'reached_but_no_support_samples'
        elif st['failed_samples']:
            st['status'] = 'observed_gap_failure'
        elif st['same_native_pose_samples'] != st['trace_samples']:
            st['status'] = 'native_binding_missing'
        else:
            st['status'] = 'recorded_samples_passed'
    raw_consistent = isinstance(raw, dict) and raw.get('visual_support_samples') == sum(
        s['trace_samples'] for s in stats.values()) and raw.get('visual_support_failures') == failures
    if not raw_consistent:
        errors.append({'file': 'visible_mesh_runtime_report.json', 'error': 'support totals/failures missing or disagree with trace'})
    after = {name: digest(run_dir / name) for name in FILES}
    unchanged = before == after
    if not unchanged:
        errors.append({'error': 'input files changed during summary; rerun after monitor exits'})
    samples = sum(s['trace_samples'] for s in stats.values())
    sampled_passed = bool(samples and not failures and len(matched) == samples)
    complete = bool(sampled_passed and not errors and not missing_trace and not missing_entity and not missing_native
                    and before['finished_at'] is not None and raw.get('finished_marker_seen') is True)
    status = ('recorded_support_evidence_complete' if complete else 'observed_support_failure' if failures
              else 'support_not_reached' if not stats else 'support_evidence_incomplete')
    summary = {'run': run_dir.name, 'source_run': str(run_dir), 'status': status,
        'force_support_claimed': False, 'full_assembly_claimed': False,
        'scope': 'Only reached phases and recorded CAD samples; three support-body poses bound to valid same-clock native before_action samples. Between-sample motion, force support, camera visibility and assembly success are separate evidence.',
        'observed_samples_passed': sampled_passed, 'recorded_support_evidence_complete': complete,
        'trace_samples': samples, 'same_native_pose_samples': len(matched),
        'native_pose_matrix_arithmetic_tolerance': 1e-10,
        'reached_support_phases': list(stats.values()), 'support_failures': failures,
        'missing_trace_samples': [{'step': s, 'phase': p} for s, p in missing_trace],
        'missing_entity_samples': [{'step': s, 'phase': p} for s, p in missing_entity],
        'missing_native_samples': [{'step': s, 'phase': p} for s, p in missing_native],
        'unsampled_reached_maintenance_phases': [p for p, s in stats.items()
            if p != ACQUISITION and not s['trace_samples']],
        'native_matches': native_matches, 'errors': errors,
        'actual_result': {k: result.get(k) for k in ('execution_route', 'success', 'assisted_success', 'failed',
                                                  'steps', 'phase_status', 'terminal_reason', 'assisted_boundary_stop')},
        'original_reports': {'visible_mesh_runtime_report': original_report_scope(raw), **{
            k: original_report_scope(result.get(k)) for k in
            ('runtime_constraint_monitor', 'stage_trajectory_precheck', 'assembly_sequence_precheck',
             'post_failure_recording')}},
        'source_sha256_before': before, 'source_sha256_after': after, 'source_unchanged': unchanged}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_name')
    args = parser.parse_args()
    if Path(args.run_name).name != args.run_name or args.run_name in ('', '.', '..'):
        parser.error('run_name must be a single directory name')
    p = Path(__file__).resolve().parents[1]
    d = p / 'runs' / args.run_name
    if not d.is_dir():
        parser.error('run directory does not exist: ' + str(d))
    report = summarize(d)
    out = d / 'C_support_summary.json'
    out.write_text(json.dumps(json_safe(report), indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({k: report[k] for k in ('run', 'status', 'trace_samples', 'same_native_pose_samples',
                                            'observed_samples_passed', 'recorded_support_evidence_complete')},
                     ensure_ascii=False))
    print(str(out))
    return 0 if report['recorded_support_evidence_complete'] else 3 if report['support_failures'] else 4


if __name__ == '__main__':
    raise SystemExit(main())
