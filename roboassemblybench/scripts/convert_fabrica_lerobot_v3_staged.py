from __future__ import annotations

import argparse
import errno
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
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
CAMERA_KEYS = (
    'observation.images.front',
    'observation.images.left_wrist',
    'observation.images.right_wrist',
)


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _write_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    temporary.replace(path)


def _episode_count(output_dir: Path) -> int:
    manifest_path = output_dir / 'roboassemblybench_conversion_manifest.json'
    if not manifest_path.is_file():
        return 0
    try:
        return len(_load(manifest_path).get('episodes') or [])
    except (OSError, json.JSONDecodeError):
        return 0


def _is_complete(output_dir: Path, target: int) -> bool:
    marker = output_dir / '.roboassemblybench_export_complete'
    return marker.is_file() and _episode_count(output_dir) == target


def _parquet_files_are_valid(output_dir: Path) -> bool:
    info_path = output_dir / 'meta' / 'info.json'
    manifest_path = output_dir / 'roboassemblybench_conversion_manifest.json'
    if not info_path.is_file() or not manifest_path.is_file():
        return False
    try:
        import pyarrow.parquet as pq

        files = list((output_dir / 'meta').rglob('*.parquet')) + list(
            (output_dir / 'data').rglob('*.parquet')
        )
        if not files:
            return False
        for path in files:
            pq.read_metadata(path)
    except Exception:
        return False
    return True


def _run(command: list[str], *, env: dict[str, str] | None = None) -> None:
    subprocess.run(command, check=True, env=env)


class StagedConverter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.output_root = args.output_root.resolve()
        self.source_root = self.output_root / '_conversion_sources'
        self.selection = _load(self.output_root / 'source_selection_manifest.json')
        self.targets = {key: int(value) for key, value in self.selection['subsets'].items()}
        self.exporter = args.code_root / 'roboassemblybench/scripts/export_fabrica_lerobot_v3.py'
        self.transfer_lock = threading.Lock()
        self.queue_lock = threading.Lock()
        self.log_lock = threading.Lock()
        self.failures: dict[str, str] = {}
        jobs = []
        for task in TASKS:
            for profile in PROFILES:
                key = f'{task}/{profile}'
                final_dir = self.output_root / key
                target = self.targets[key]
                if not _is_complete(final_dir, target):
                    jobs.append(
                        {
                            'key': key,
                            'task': task,
                            'profile': profile,
                            'target': target,
                            'current': _episode_count(final_dir),
                        }
                    )
        jobs.sort(key=lambda job: (job['target'] - job['current'], job['profile'] == 'scene'))
        self.jobs = deque(jobs)

    def log(self, message: str, **fields: Any) -> None:
        event = {'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'message': message, **fields}
        with self.log_lock:
            print(json.dumps(event, ensure_ascii=False, default=str), flush=True)

    def _claim_job(self, stage_root: Path) -> dict[str, Any] | None:
        state_path = stage_root / 'staged_job.json'
        with self.queue_lock:
            if state_path.is_file():
                state = _load(state_path)
                key = str(state.get('key', ''))
                for job in list(self.jobs):
                    if job['key'] == key:
                        self.jobs.remove(job)
                        return job
            if not self.jobs:
                return None
            # The shared-memory slot avoids the largest 3,297-episode subset.
            if str(stage_root).startswith('/dev/shm'):
                for job in self.jobs:
                    if job['target'] <= 2700:
                        self.jobs.remove(job)
                        return job
                return None
            return self.jobs.popleft()

    def _copy_partial(self, source_dir: Path, local_dir: Path) -> bool:
        if not _parquet_files_are_valid(source_dir):
            return False
        local_dir.mkdir(parents=True, exist_ok=True)
        with self.transfer_lock:
            self.log('copy_partial_start', source=str(source_dir), destination=str(local_dir))
            _run(
                [
                    'rsync',
                    '-a',
                    '--delete',
                    '--exclude=/sensors/depth/***',
                    '--exclude=*.tmp',
                    f'{source_dir}/',
                    f'{local_dir}/',
                ]
            )
            self.log('copy_partial_complete', episodes=_episode_count(local_dir))
        return True

    def _legacy_stage_dataset(self, key: str) -> Path | None:
        """Find a resumable dataset left in an older stage root.

        Older runs used NFS-backed stage roots.  Keeping these roots as read-only
        resume sources avoids throwing away thousands of already encoded episodes
        when moving future work to local storage.
        """
        for stage_root in self.args.legacy_stage_root:
            state_path = stage_root / 'staged_job.json'
            if not state_path.is_file():
                continue
            try:
                state = _load(state_path)
            except (OSError, json.JSONDecodeError):
                continue
            if state.get('key') != key:
                continue
            dataset = stage_root / 'work' / 'dataset'
            if dataset.is_dir() and _parquet_files_are_valid(dataset):
                return dataset
        return None

    def _conversion_signature(self, local_dir: Path, log_path: Path) -> tuple[int, int, int]:
        count = _episode_count(local_dir)
        try:
            log_size = log_path.stat().st_size
        except OSError:
            log_size = 0
        try:
            info_mtime = (local_dir / 'meta' / 'info.json').stat().st_mtime_ns
        except OSError:
            info_mtime = 0
        return count, log_size, info_mtime

    def _terminate(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=20)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            pass

    def _convert(self, job: dict[str, Any], stage_root: Path, local_dir: Path, gpu: int) -> None:
        log_path = stage_root / 'export.log'
        source_dir = self.source_root / job['key']
        repo_id = f"baiyu858/roboassemblybench_fabrica_ur5e_{job['task']}_{job['profile']}_10w"
        for attempt in range(1, self.args.attempts + 1):
            resume = (local_dir / 'meta' / 'info.json').is_file()
            if resume and not _parquet_files_are_valid(local_dir):
                self.log('discard_corrupt_stage', key=job['key'], attempt=attempt)
                shutil.rmtree(local_dir)
                self._copy_partial(self.output_root / job['key'], local_dir)
                resume = (local_dir / 'meta' / 'info.json').is_file()
            command = [
                str(self.args.python),
                str(self.exporter),
                '--input-dir',
                str(source_dir),
                '--output-dir',
                str(local_dir),
                '--repo-id',
                repo_id,
                '--max-episodes',
                str(job['target']),
                '--encoder-threads',
                str(self.args.encoder_threads),
                '--vcodec',
                self.args.vcodec,
            ]
            if resume:
                command.append('--resume')
            env = os.environ.copy()
            env.update(
                {
                    'PYTHONPATH': self.args.pythonpath,
                    'PYTHONNOUSERSITE': '1',
                    'PYTHONDONTWRITEBYTECODE': '1',
                    'PYTHONUNBUFFERED': '1',
                    'TOKENIZERS_PARALLELISM': 'false',
                    'RAB_DEFER_DEPTH_SIDECARS': '1',
                    'RAB_H264_PRESET': self.args.h264_preset,
                    'CUDA_VISIBLE_DEVICES': str(gpu),
                    'HF_HOME': str(stage_root / 'hf_cache'),
                    'HF_DATASETS_CACHE': str(stage_root / 'hf_cache' / 'datasets'),
                    'TMPDIR': str(stage_root / 'tmp'),
                }
            )
            (stage_root / 'tmp').mkdir(parents=True, exist_ok=True)
            self.log(
                'conversion_start',
                key=job['key'],
                attempt=attempt,
                resume=resume,
                current=_episode_count(local_dir),
                target=job['target'],
                vcodec=self.args.vcodec,
                gpu=gpu,
            )
            with log_path.open('a', encoding='utf-8') as log:
                process = subprocess.Popen(
                    command,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            signature = self._conversion_signature(local_dir, log_path)
            last_change = time.monotonic()
            while process.poll() is None:
                time.sleep(self.args.poll_seconds)
                current_signature = self._conversion_signature(local_dir, log_path)
                if current_signature != signature:
                    if current_signature[0] != signature[0]:
                        self.log(
                            'conversion_progress',
                            key=job['key'],
                            episodes=current_signature[0],
                            target=job['target'],
                        )
                    signature = current_signature
                    last_change = time.monotonic()
                free_bytes = shutil.disk_usage(stage_root).free
                if free_bytes < self.args.minimum_free_gib * 1024**3:
                    self._terminate(process)
                    raise RuntimeError(
                        f'{stage_root} has only {free_bytes / 1024**3:.1f} GiB free.'
                    )
                if time.monotonic() - last_change > self.args.stall_timeout:
                    self.log('conversion_stalled', key=job['key'], attempt=attempt)
                    self._terminate(process)
                    break
            returncode = process.poll()
            if returncode == 0 and _is_complete(local_dir, job['target']):
                self.log('conversion_complete', key=job['key'], episodes=job['target'])
                return
            self.log(
                'conversion_retry',
                key=job['key'],
                attempt=attempt,
                returncode=returncode,
                episodes=_episode_count(local_dir),
            )
            time.sleep(min(30 * attempt, 120))
        raise RuntimeError(f"{job['key']} did not complete after {self.args.attempts} attempts.")

    def _resolved_metadata(self, source_metadata: str) -> dict[str, Any]:
        from roboassemblybench.scripts.export_fabrica_plumbers_block_lerobot_v3 import (
            _resolve_episode_assets,
        )

        metadata_path = Path(source_metadata)
        metadata = _load(metadata_path)
        metadata['metadata_path'] = str(metadata_path.resolve())
        return _resolve_episode_assets(metadata)

    def _materialize_depth(self, publish_dir: Path, target: int) -> None:
        manifest = _load(publish_dir / 'roboassemblybench_conversion_manifest.json')
        entries = manifest.get('episodes') or []
        if len(entries) != target:
            raise RuntimeError(f'Publish manifest contains {len(entries)} episodes; expected {target}.')
        for episode_index, entry in enumerate(entries):
            metadata = self._resolved_metadata(str(entry['source_metadata']))
            index = {
                'schema_version': 'roboassemblybench_metric_depth_index_v1',
                'episode_index': episode_index,
                'frame_count': int(metadata['frame_count']),
                'fps': int(metadata['fps']),
                'streams': {},
            }
            chunk = episode_index // 1000
            for camera_key in CAMERA_KEYS:
                stream = dict((metadata.get('depth') or {})[camera_key])
                source = Path(stream.pop('path'))
                relative = (
                    Path('sensors')
                    / 'depth'
                    / camera_key
                    / f'chunk-{chunk:03d}'
                    / f'episode_{episode_index:06d}.u16.bshuf.zst'
                )
                destination = publish_dir / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                if destination.exists():
                    destination.unlink()
                try:
                    os.link(source, destination)
                except OSError as exc:
                    if exc.errno not in (errno.EXDEV, errno.EPERM, errno.EACCES):
                        raise
                    if self.args.depth_fallback == 'copy':
                        shutil.copy2(source, destination)
                    else:
                        destination.symlink_to(os.path.relpath(source, destination.parent))
                index['streams'][camera_key] = {**stream, 'path': relative.as_posix()}
            _write_atomic(
                publish_dir
                / 'sensors'
                / 'depth'
                / 'index'
                / f'episode_{episode_index:06d}.json',
                index,
            )

    def _verify_publish(self, publish_dir: Path, target: int) -> None:
        if not _is_complete(publish_dir, target):
            raise RuntimeError(f'{publish_dir} is missing its complete marker.')
        info = _load(publish_dir / 'meta' / 'info.json')
        if int(info.get('total_episodes', -1)) != target:
            raise RuntimeError(f'{publish_dir} info.json has the wrong episode count.')
        if not _parquet_files_are_valid(publish_dir):
            raise RuntimeError(f'{publish_dir} contains invalid parquet files.')
        indexes = list((publish_dir / 'sensors' / 'depth' / 'index').glob('episode_*.json'))
        if len(indexes) != target:
            raise RuntimeError(f'{publish_dir} contains {len(indexes)} depth indexes; expected {target}.')
        for index_path in indexes:
            index = _load(index_path)
            for stream in (index.get('streams') or {}).values():
                if not (publish_dir / stream['path']).is_file():
                    raise RuntimeError(f'{index_path} references a missing depth payload.')

    def _publish(self, job: dict[str, Any], local_dir: Path) -> None:
        final_dir = self.output_root / job['key']
        publish_dir = final_dir.with_name(
            f'.{final_dir.name}.publish-{time.strftime("%Y%m%dT%H%M%S")}-{uuid.uuid4().hex[:8]}'
        )
        with self.transfer_lock:
            self.log('publish_start', key=job['key'], destination=str(publish_dir))
            publish_dir.mkdir(parents=True, exist_ok=False)
            try:
                if local_dir.stat().st_dev == publish_dir.parent.stat().st_dev:
                    publish_dir.rmdir()
                    local_dir.rename(publish_dir)
                else:
                    _run(['rsync', '-a', '--delete', f'{local_dir}/', f'{publish_dir}/'])
                manifest_path = publish_dir / 'roboassemblybench_conversion_manifest.json'
                manifest = _load(manifest_path)
                manifest['output_dir'] = str(final_dir)
                _write_atomic(manifest_path, manifest)
                summary_path = publish_dir / 'roboassemblybench_export_summary.json'
                if summary_path.is_file():
                    summary = _load(summary_path)
                    summary['output_dir'] = str(final_dir)
                    _write_atomic(summary_path, summary)
                self._materialize_depth(publish_dir, job['target'])
                self._verify_publish(publish_dir, job['target'])

                backup = (
                    self.output_root
                    / '_recovery_backups'
                    / job['task']
                    / f'{job["profile"]}.pre-staged-{time.strftime("%Y%m%dT%H%M%S")}'
                )
                backup.parent.mkdir(parents=True, exist_ok=True)
                if final_dir.exists():
                    final_dir.rename(backup)
                try:
                    publish_dir.rename(final_dir)
                except Exception:
                    if backup.exists() and not final_dir.exists():
                        backup.rename(final_dir)
                    raise
            except Exception:
                self.log('publish_failed', key=job['key'], staging=str(publish_dir))
                raise
            self.log('publish_complete', key=job['key'], episodes=job['target'])

    def _run_job(self, job: dict[str, Any], stage_root: Path, gpu: int) -> None:
        stage_root.mkdir(parents=True, exist_ok=True)
        state_path = stage_root / 'staged_job.json'
        work_root = stage_root / 'work'
        local_dir = work_root / 'dataset'
        state = _load(state_path) if state_path.is_file() else {}
        if state.get('key') != job['key'] or not bool(state.get('copy_complete', False)):
            if work_root.exists():
                shutil.rmtree(work_root)
            work_root.mkdir(parents=True)
            _write_atomic(state_path, {'key': job['key'], 'target': job['target']})
            legacy_dataset = self._legacy_stage_dataset(job['key'])
            if legacy_dataset is not None:
                self.log(
                    'copy_legacy_stage_start',
                    key=job['key'],
                    source=str(legacy_dataset),
                    destination=str(local_dir),
                )
                copied = self._copy_partial(legacy_dataset, local_dir)
                if copied:
                    self.log('copy_legacy_stage_complete', key=job['key'], episodes=_episode_count(local_dir))
                else:
                    self.log('copy_legacy_stage_invalid', key=job['key'], source=str(legacy_dataset))
            if not local_dir.exists() or not _parquet_files_are_valid(local_dir):
                self._copy_partial(self.output_root / job['key'], local_dir)
            _write_atomic(
                state_path,
                {'key': job['key'], 'target': job['target'], 'copy_complete': True},
            )
        self._convert(job, stage_root, local_dir, gpu)
        self._publish(job, local_dir)
        shutil.rmtree(work_root, ignore_errors=True)
        state_path.unlink(missing_ok=True)

    def worker(self, stage_root: Path, gpu: int) -> None:
        while True:
            job = self._claim_job(stage_root)
            if job is None:
                return
            try:
                self._run_job(job, stage_root, gpu)
            except Exception as exc:
                self.failures[job['key']] = f'{type(exc).__name__}: {exc}'
                self.log('job_failed', key=job['key'], error=self.failures[job['key']])

    def run(self) -> int:
        self.log('staged_conversion_start', jobs=len(self.jobs), stage_roots=self.args.stage_root)
        gpus = self.args.gpu or list(range(len(self.args.stage_root)))
        if len(gpus) != len(self.args.stage_root):
            raise ValueError('Provide exactly one --gpu value for each --stage-root.')
        threads = [
            threading.Thread(target=self.worker, args=(path.resolve(), gpu), daemon=False)
            for path, gpu in zip(self.args.stage_root, gpus, strict=True)
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        remaining = [
            key for key, target in self.targets.items() if not _is_complete(self.output_root / key, target)
        ]
        summary = {
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'remaining_subsets': remaining,
            'failures': self.failures,
            'complete': not remaining,
        }
        _write_atomic(self.output_root / 'staged_conversion_summary.json', summary)
        self.log('staged_conversion_end', **summary)
        return 0 if not remaining else 1


def main() -> None:
    parser = argparse.ArgumentParser(description='Convert Fabrica LeRobot v3 subsets through local staging.')
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--code-root', required=True, type=Path)
    parser.add_argument('--python', required=True, type=Path)
    parser.add_argument('--pythonpath', required=True)
    parser.add_argument('--stage-root', action='append', required=True, type=Path)
    parser.add_argument(
        '--legacy-stage-root',
        action='append',
        default=[],
        type=Path,
        help='Read-only stage roots from an older run to resume partial datasets.',
    )
    parser.add_argument('--encoder-threads', type=int, default=4)
    parser.add_argument('--vcodec', default='h264')
    parser.add_argument('--gpu', action='append', type=int)
    parser.add_argument('--h264-preset', default='veryfast')
    parser.add_argument('--depth-fallback', choices=('symlink', 'copy'), default='symlink')
    parser.add_argument('--attempts', type=int, default=3)
    parser.add_argument('--poll-seconds', type=int, default=30)
    parser.add_argument('--stall-timeout', type=int, default=1200)
    parser.add_argument('--minimum-free-gib', type=int, default=4)
    args = parser.parse_args()
    raise SystemExit(StagedConverter(args).run())


if __name__ == '__main__':
    main()
