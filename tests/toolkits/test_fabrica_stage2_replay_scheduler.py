import json
import os
import socket
import time
from pathlib import Path

from roboassemblybench.scripts.pipeline_fabrica_stage2_replay import (
    REPLAY_QUALITY_POLICY_VERSION,
    ReplayJob,
    _job_has_external_lock,
    _parse_profiles,
    _replay_complete,
    _select_job,
)


def _write_manifest(path: Path, *, policy: str | None) -> None:
    path.write_text(
        json.dumps(
            {
                'complete': True,
                'quality_policy_version': policy,
                'num_successful': 2,
                'successful_episodes': {'1': {}, '2': {}},
            }
        ),
        encoding='utf-8',
    )


def test_old_quality_manifest_is_not_reusable(tmp_path: Path):
    manifest = tmp_path / 'replay_manifest.json'
    _write_manifest(manifest, policy=None)
    assert not _replay_complete(manifest, 2)

    _write_manifest(manifest, policy=REPLAY_QUALITY_POLICY_VERSION)
    assert _replay_complete(manifest, 2)


def test_profile_selector_rejects_duplicates_and_unknown_values():
    assert _parse_profiles('scene,lighting') == ('scene', 'lighting')

    try:
        _parse_profiles('scene,scene')
    except ValueError as error:
        assert 'duplicates' in str(error)
    else:
        raise AssertionError('duplicate profiles should be rejected')

    try:
        _parse_profiles('scene,unknown')
    except ValueError as error:
        assert 'unknown' in str(error)
    else:
        raise AssertionError('unknown profiles should be rejected')


def test_active_external_replay_lock_is_not_started_again(tmp_path: Path):
    output_dir = tmp_path / 'rendered' / 'task' / 'scene' / 'shards' / 'shard_000'
    lock_dir = output_dir / '.collection.lock.d'
    lock_dir.mkdir(parents=True)
    (lock_dir / 'owner.json').write_text(
        json.dumps(
            {
                'pid': os.getpid(),
                'hostname': socket.gethostname(),
                'created_at_unix': time.time(),
            }
        ),
        encoding='utf-8',
    )
    job = ReplayJob(
        task='task',
        profile='scene',
        shard_name='shard_000',
        source_dir=tmp_path / 'source',
        output_dir=output_dir,
        recipe='recipe',
        scene_profile='scene',
        target_episodes=1,
        ready_at=0.0,
    )

    assert _job_has_external_lock(job)
    assert _select_job(
        [job],
        {'jobs': {}},
        time.time(),
        active_job_keys=set(),
        externally_locked_job_keys={job.key},
    ) is None
