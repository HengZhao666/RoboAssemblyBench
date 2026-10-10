from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any


TASKS = (
    'beam',
    'car',
    'cooling_manifold',
    'duct',
    'gamepad',
    'plumbers_block',
    'stool_circular',
)
PROFILES = ('position', 'object_distractors', 'texture', 'lighting', 'table_color', 'scene')
CORRUPT_OUTPUT_MARKERS = (
    'Parquet magic bytes not found',
    'Incomplete LeRobot metadata',
    'Conversion manifest is not a prefix',
)


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _is_complete(output_dir: Path, target: int) -> bool:
    marker = output_dir / '.roboassemblybench_export_complete'
    manifest = output_dir / 'roboassemblybench_conversion_manifest.json'
    if not marker.is_file() or not manifest.is_file():
        return False
    payload = _load(manifest)
    return int(payload.get('total_episodes', len(payload.get('episodes') or []))) == target


def _quarantine_output(output_dir: Path, *, reason: str) -> Path | None:
    """Atomically preserve an unusable dataset without recursively deleting it."""
    if not output_dir.exists():
        return None
    label = re.sub(r'[^a-z0-9_-]+', '-', reason.lower()).strip('-') or 'incomplete'
    timestamp = time.strftime('%Y%m%dT%H%M%S')
    for sequence in range(1000):
        suffix = f'.quarantine-{label}-{timestamp}'
        if sequence:
            suffix += f'-{sequence}'
        destination = output_dir.with_name(output_dir.name + suffix)
        if destination.exists():
            continue
        output_dir.rename(destination)
        return destination
    raise RuntimeError(f'Unable to allocate a quarantine path for {output_dir}.')


def _log_contains_corrupt_output(log_path: Path, *, start_offset: int) -> bool:
    if not log_path.is_file():
        return False
    with log_path.open('r', encoding='utf-8', errors='replace') as handle:
        handle.seek(start_offset)
        output = handle.read()
    return any(marker in output for marker in CORRUPT_OUTPUT_MARKERS)


def _run_subset(
    *,
    python: str,
    exporter: Path,
    source_dir: Path,
    output_dir: Path,
    task: str,
    profile: str,
    target: int,
    repo_id: str,
    log_path: Path,
    gpu: int,
    attempts: int,
    encoder_threads: int,
    pythonpath: str,
) -> dict[str, Any]:
    output_dir.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if _is_complete(output_dir, target):
        return {'task': task, 'profile': profile, 'status': 'already_complete', 'episodes': target}

    quarantined = None
    info_path = output_dir / 'meta' / 'info.json'
    if output_dir.is_dir() and not info_path.is_file():
        quarantined = _quarantine_output(output_dir, reason='missing-info')
        with log_path.open('a', encoding='utf-8') as log:
            log.write(f'quarantined_incomplete_output={quarantined}\n')

    for attempt in range(1, attempts + 1):
        info_path = output_dir / 'meta' / 'info.json'
        resume = (output_dir / 'meta' / 'info.json').is_file()
        command = [
            python,
            str(exporter),
            '--input-dir', str(source_dir),
            '--output-dir', str(output_dir),
            '--repo-id', repo_id,
            '--max-episodes', str(target),
            '--encoder-threads', str(encoder_threads),
        ]
        if resume:
            command.append('--resume')
        env = os.environ.copy()
        env.update(
            {
                'CUDA_VISIBLE_DEVICES': str(gpu),
                'PYTHONPATH': pythonpath,
                'PYTHONNOUSERSITE': '1',
                'PYTHONUNBUFFERED': '1',
                'TOKENIZERS_PARALLELISM': 'false',
            }
        )
        log_offset = log_path.stat().st_size if log_path.is_file() else 0
        with log_path.open('a', encoding='utf-8') as log:
            log.write(
                f'\n[{time.strftime("%Y-%m-%dT%H:%M:%S%z")}] '
                f'task={task}/{profile} attempt={attempt}/{attempts} gpu={gpu} resume={resume}\n'
            )
            log.flush()
            completed = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        if completed.returncode == 0 and _is_complete(output_dir, target):
            return {'task': task, 'profile': profile, 'status': 'complete', 'episodes': target, 'attempt': attempt}
        with log_path.open('a', encoding='utf-8') as log:
            log.write(
                f'[{time.strftime("%Y-%m-%dT%H:%M:%S%z")}] '
                f'conversion_failed returncode={completed.returncode}\n'
            )
        if _log_contains_corrupt_output(log_path, start_offset=log_offset):
            quarantined = _quarantine_output(output_dir, reason='corrupt-parquet')
            with log_path.open('a', encoding='utf-8') as log:
                log.write(f'quarantined_corrupt_output={quarantined}\n')
        time.sleep(min(30 * attempt, 120))
    result = {'task': task, 'profile': profile, 'status': 'failed', 'episodes': 0}
    if quarantined is not None:
        result['quarantined_output'] = str(quarantined)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description='Convert the frozen Fabrica 100k set to LeRobot v3.')
    parser.add_argument('--data-root', type=Path, default=Path('/data/a17/baiyongjie/data/ur5e'))
    parser.add_argument('--output-root', type=Path, default=Path('/data/a17/baiyongjie/data/lerobot/ur5e'))
    parser.add_argument('--code-root', type=Path, default=Path('/data/a17/baiyongjie/InternUtopia'))
    parser.add_argument('--python', default=sys.executable)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--encoder-threads', type=int, default=2)
    parser.add_argument(
        '--local-site-packages',
        type=Path,
        default=Path('/home/byj/RoboAssemblyBench/.venv/lib/python3.11/site-packages'),
    )
    parser.add_argument(
        '--lerobot-site-packages',
        type=Path,
        default=Path('/data/a17/baiyongjie/ur5e/lerobot_env/lib/python3.11/site-packages'),
    )
    parser.add_argument('--prepare', action='store_true')
    args = parser.parse_args()

    args.output_root.mkdir(parents=True, exist_ok=True)
    freeze_manifest = args.data_root / 'task7rand5' / 'dataset_manifest.json'
    if args.prepare or not (args.output_root / 'source_selection_manifest.json').is_file():
        from prepare_fabrica_lerobot_sources import prepare_sources

        summary = prepare_sources(
            data_root=args.data_root,
            output_root=args.output_root,
            manifest_path=freeze_manifest,
        )
        print(json.dumps({'prepared_total_episodes': summary['prepared_total_episodes']}), flush=True)

    selection = _load(args.output_root / 'source_selection_manifest.json')
    source_root = args.output_root / '_conversion_sources'
    exporter = args.code_root / 'roboassemblybench/scripts/export_fabrica_lerobot_v3.py'
    if not exporter.is_file():
        raise FileNotFoundError(exporter)
    pythonpath = ':'.join(
        str(path) for path in (args.code_root, args.local_site_packages, args.lerobot_site_packages)
    )

    jobs = []
    for task in TASKS:
        for profile in PROFILES:
            key = f'{task}/{profile}'
            jobs.append(
                {
                    'task': task,
                    'profile': profile,
                    'key': key,
                    'target': int(selection['subsets'][key]),
                    'source_dir': source_root / task / profile,
                    'output_dir': args.output_root / task / profile,
                    'log_path': args.output_root / 'conversion_logs' / f'{task}__{profile}.log',
                    'repo_id': f'baiyu858/roboassemblybench_fabrica_ur5e_{task}_{profile}_10w',
                }
            )

    results: dict[str, Any] = {}
    workers = max(1, min(int(args.workers), len(jobs)))
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {
            pool.submit(
                _run_subset,
                python=args.python,
                exporter=exporter,
                source_dir=job['source_dir'],
                output_dir=job['output_dir'],
                task=job['task'],
                profile=job['profile'],
                target=job['target'],
                repo_id=job['repo_id'],
                log_path=job['log_path'],
                gpu=index % workers,
                attempts=max(1, int(args.attempts)),
                encoder_threads=max(1, int(args.encoder_threads)),
                pythonpath=pythonpath,
            ): job
            for index, job in enumerate(jobs)
        }
        for future in as_completed(futures):
            job = futures[future]
            result = future.result()
            results[job['key']] = result
            print(json.dumps(result, ensure_ascii=False), flush=True)

    failed = [key for key, result in results.items() if result['status'] == 'failed']
    converted = sum(int(result.get('episodes', 0)) for result in results.values())
    summary = {
        'schema_version': 'roboassemblybench_lerobot_v3_conversion_run_v1',
        'output_root': str(args.output_root.resolve()),
        'workers': workers,
        'converted_episodes': converted,
        'target_episodes': int(selection['prepared_total_episodes']),
        'failed_subsets': failed,
        'subsets': results,
    }
    (args.output_root / 'conversion_run_summary.json').write_text(
        json.dumps(summary, indent=2), encoding='utf-8'
    )
    if failed or converted != summary['target_episodes']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
