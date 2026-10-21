"""Shared timing, bounded attempt claims, and source-bound delta checks."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import math
import os
import subprocess
import time

CHECKS = {'actual_configuration_and_targets', 'full_phase_release_completion_interfaces',
          'full_entities_and_camera_visibility', 'production_controller_object_space_replay',
          'admitted_error_and_final_command_geometry', 'bimanual_support_lifecycle'}


def sha(path):
    with Path(path).open('rb') as stream:
        h = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp.' + str(os.getpid()))
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(temporary, path)


def record(directory, phase, category, action, **detail):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    row = dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
               wall_unix_s=time.time(), monotonic_s=time.monotonic(), pid=os.getpid(),
               phase=phase, category=category, action=action, **detail)
    with (directory / 'timing_events.jsonl').open('a') as stream:
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
    return row


def timed(directory, phase, category, command):
    start = record(directory, phase, category, 'start', command=command,
                   scope='Inclusive elapsed wall time; nested phase intervals must not be added.')
    log = Path(directory) / (phase + '.log')
    with log.open('ab') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT)
    row = record(directory, phase, category, 'end',
                 elapsed_s=time.monotonic() - start['monotonic_s'],
                 returncode=result.returncode, log=str(log))
    print(json.dumps(row))
    return result.returncode


def validate_attempt_window(batch, run_name, cycle):
    """Preserve historical caps and require the explicit bounded continuation record."""
    cap, started = batch['maximum_native_attempts'], batch['native_attempts_started']
    if not (type(cap) is int and 0 < cap <= 31 and type(started) is int
            and 0 <= started < cap and batch['completed_cycles'] == started):
        raise ValueError('Attempt limit, count, or previous-cycle closure invalid')
    if batch.get('resumed_single_round_authorization') is not None:
        authorization = batch['resumed_single_round_authorization']
        baseline = authorization.get('baseline_closed') if isinstance(authorization, dict) else None
        if not (isinstance(authorization, dict)
                and authorization.get('authorized_by_human') is True
                and type(baseline) is int and baseline >= 0
                and type(authorization.get('baseline_started')) is int
                and authorization['baseline_started'] == baseline == started
                and type(authorization.get('additional_native_attempts')) is int
                and authorization['additional_native_attempts'] == 1
                and type(authorization.get('maximum_native_attempts')) is int
                and authorization['maximum_native_attempts'] == cap == baseline + 1
                and cycle == authorization.get('cycle') == f'D{started+1:02d}'
                and run_name == authorization.get('run')
                and isinstance(run_name, str) and run_name.startswith(f'A{started+17}_')
                and isinstance(authorization.get('human_instruction'), str)
                and authorization['human_instruction'].strip()
                and batch.get('latest_human_instruction') == authorization['human_instruction']
                and batch.get('pause_after_cycle') == cycle
                and batch.get('human_continuation_remains_active') is True):
            raise ValueError('Resumed single round requires current explicit human grant, closed baseline and exact one-run identity')
    elif batch.get('resumed_five_round_authorization') is not None:
        authorization = batch['resumed_five_round_authorization']
        required = dict(authorized_by_human=True, baseline_started=22, baseline_closed=22,
                        additional_native_attempts=5, maximum_native_attempts=27)
        if not (isinstance(authorization, dict) and cap == 27
                and all(type(authorization.get(k)) is type(v) and authorization[k] == v
                        for k, v in required.items())
                and 22 <= started < 27 and type(batch['completed_cycles']) is int
                and cycle == f'D{started+1:02d}'
                and isinstance(run_name, str) and run_name.startswith(f'A{started+17}_')
                and authorization.get('human_instruction') == '优化一下流程和经验转化。然后开始新的五轮修复'
                and batch.get('latest_human_instruction') == authorization['human_instruction']
                and batch.get('human_continuation_remains_active') is True):
            raise ValueError('Fresh five-round window requires baseline22, cap27, current human instruction and ordered identity')
    elif cap > 19:
        authorization = batch.get('resumed_twenty_round_authorization')
        required = dict(authorized_by_human=True, baseline_started=11, baseline_closed=11,
                        total_window_attempts=20, maximum_native_attempts=31,
                        prior_window_attempts_consumed=8, remaining_at_grant=12)
        if not (isinstance(authorization, dict) and cap == 31
                and all(type(authorization.get(k)) is type(v) and authorization[k] == v
                        for k, v in required.items())
                and 19 <= started < 31 and type(batch['completed_cycles']) is int
                and cycle == f'D{started+1:02d}'
                and isinstance(run_name, str) and run_name.startswith(f'A{started+17}_')
                and authorization.get('human_instruction') == '上限改为20轮'
                and batch.get('human_continuation_remains_active') is True):
            raise ValueError('Twenty-round extension requires explicit human record, spent-eight preservation and ordered identity')
    elif cap > 11:
        authorization = batch.get('resumed_eight_round_authorization')
        required = dict(authorized_by_human=True, baseline_started=11, baseline_closed=11,
                        additional_native_attempts=8, maximum_native_attempts=19)
        if not (isinstance(authorization, dict) and cap == 19
                and all(type(authorization.get(k)) is type(v) and authorization[k] == v
                        for k, v in required.items())
                and 11 <= started < 19 and type(batch['completed_cycles']) is int
                and cycle == f'D{started+1:02d}'
                and isinstance(run_name, str) and run_name.startswith(f'A{started+17}_')
                and isinstance(authorization.get('human_instruction'), str)
                and authorization['human_instruction'].strip()
                and batch.get('human_continuation_remains_active') is True):
            raise ValueError('Resumed window requires explicit eight-round human authorization and ordered identity')
    if cap == 11:
        authorization = batch.get('extra_single_run_authorization')
        expected = dict(authorized_by_human=True, previous_closed=10,
                        cycle='D11', run='A27_beam_full',
                        maximum_native_attempts=11, additional_native_attempts=1)
        if not (isinstance(authorization, dict)
                and all(type(authorization.get(k)) is type(v) and authorization[k] == v
                        for k, v in expected.items())
                and type(batch['completed_cycles']) is int
                and started == batch['completed_cycles'] == 10
                and cycle == authorization['cycle']
                and run_name == authorization['run']
                and isinstance(authorization.get('human_instruction'), str)
                and authorization['human_instruction'].strip()):
            raise ValueError('D11 requires the recorded explicit one-run human authorization')


def claim(path, run_name, cycle):
    path = Path(path)
    batch = json.loads(path.read_text())
    validate_attempt_window(batch, run_name, cycle)
    cap, started = batch['maximum_native_attempts'], batch['native_attempts_started']
    if batch.get('next_native_run_authorized') is not True:
        raise ValueError('Native work is not authorized')
    row = batch['cycles'][started]
    if row['id'] != cycle or row['planned_run'] != run_name or row.get('native_started'):
        raise ValueError('Attempt order/identity mismatch or duplicate claim')
    row.update(native_started=True, state='native_starting', actual_run=run_name)
    batch.update(native_attempts_started=started + 1, current_cycle=cycle, status='native_running')
    save(path, batch)
    record(path.parent / 'batch_D_20261016', cycle, 'native', 'claimed', run=run_name,
           native_attempts_started=started + 1, maximum_native_attempts=cap)
    return dict(claimed_cycle=cycle, run=run_name, native_attempts_started=started + 1,
                maximum_native_attempts=cap)


def pointer(value, expression):
    for part in expression.strip('/').split('/') if expression.strip('/') else []:
        part = part.replace('~1', '/').replace('~0', '~')
        value = value[int(part)] if isinstance(value, list) else value[part]
    return value


def validate_delta(gate, source_manifest, tools_manifest):
    binding = gate['delta_manifest']
    path = Path(binding['path'])
    if sha(path) != binding['sha256']:
        raise ValueError('Delta manifest changed')
    delta = json.loads(path.read_text())
    expected = dict(run_name=gate['run_name'], cycle=gate['cycle'],
                    source_manifest_sha256=sha(source_manifest),
                    tools_manifest_sha256=sha(tools_manifest))
    if any(delta.get(k) != v for k, v in expected.items()):
        raise ValueError('Delta/source/tool binding is stale')
    if set(delta['checks']) != CHECKS or not delta.get('changed_factors'):
        raise ValueError('Six scoped checks and declared changes are required')
    count = 0
    for name, check in delta['checks'].items():
        if not check.get('scope') or not check.get('reports'):
            raise ValueError('Missing scoped evidence: ' + name)
        for evidence in check['reports']:
            report_path = Path(evidence['path'])
            if sha(report_path) != evidence['sha256']:
                raise ValueError('Evidence changed: ' + str(report_path))
            report = json.loads(report_path.read_text())
            assertions = evidence.get('assertions', [])
            if len(assertions) < 2:
                raise ValueError('A passed flag alone cannot qualify evidence')
            for assertion in assertions:
                value = pointer(report, assertion['pointer'])
                if isinstance(value, float) and not math.isfinite(value):
                    raise ValueError('Nonfinite evidence value')
                operators = set(assertion) - {'pointer'}
                if not operators or operators - {'equals', 'minimum', 'maximum', 'length_min'}:
                    raise ValueError('Unsupported/empty assertion')
                if 'equals' in assertion and (type(value) is not type(assertion['equals']) or value != assertion['equals']):
                    raise ValueError('Evidence equality failed: ' + assertion['pointer'])
                if 'minimum' in assertion and (type(value) not in (int, float) or value < assertion['minimum']):
                    raise ValueError('Evidence minimum failed: ' + assertion['pointer'])
                if 'maximum' in assertion and (type(value) not in (int, float) or value > assertion['maximum']):
                    raise ValueError('Evidence maximum failed: ' + assertion['pointer'])
                if 'length_min' in assertion and len(value) < assertion['length_min']:
                    raise ValueError('Evidence coverage incomplete: ' + assertion['pointer'])
                count += 1
    return dict(passed=True, checked_assertions=count, delta_sha256=sha(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['run', 'event'])
    parser.add_argument('directory', type=Path)
    parser.add_argument('phase')
    parser.add_argument('category', choices=['analysis', 'edit', 'compute', 'network', 'wait', 'native', 'postprocess'])
    parser.add_argument('arguments', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.operation == 'event':
        print(json.dumps(record(args.directory, args.phase, args.category,
                                args.arguments[0] if args.arguments else 'observed')))
        return 0
    command = args.arguments[1:] if args.arguments[:1] == ['--'] else args.arguments
    if not command:
        parser.error('run requires a command')
    return timed(args.directory, args.phase, args.category, command)


if __name__ == '__main__':
    raise SystemExit(main())
