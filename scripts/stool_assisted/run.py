#!/usr/bin/env python3
"""Run the byte-preserved, accepted Stool controller in an isolated source tree.

This adapter relocates paths and records a new run identity. It never edits the
historical ledger, publishes historical results as a new run, or tunes physics.
"""
from __future__ import annotations

import argparse
import copy
import datetime
import gzip
import hashlib
import html
import importlib
import importlib.metadata
import json
import os
import platform
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ROOT / 'artifacts/stool_assisted'
RELEASE = ARTIFACTS / 'release.json'
RUNTIME = ROOT / 'outputs/stool_assisted_reference'
SOURCE = RUNTIME / 'stool_repository'
TOOLS = RUNTIME / 'tools'
RECIPE = 'fabrica_stool_circular_ur5e_staged'
PROFILE = 'taoyuan_grscenes_tabletop'
OLD_WORKSPACE = '/home/user/beam_integration_20261010_upstream6f57dae'
OLD_SOURCE = OLD_WORKSPACE + '/stool_assisted_20261021/stool_repository'
OLD_ISAAC = '/home/user/isaacsim-5.1.0'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    path = Path(path)
    encoded = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    temporary = path.with_name(path.name + '.tmp.' + str(os.getpid()))
    temporary.write_text(encoded)
    os.replace(temporary, path)


def read_release():
    return json.loads(RELEASE.read_text())


def verify_files(base, manifest):
    for relative, expected in manifest.items():
        name = Path(relative)
        if name.is_absolute() or '..' in name.parts:
            raise ValueError('Unsafe manifest path: ' + relative)
        path = base / name
        if not path.is_file() or digest(path) != expected:
            raise ValueError('Reference file missing or changed: ' + str(path))


def extract_exact(archive, destination, manifest):
    with tarfile.open(archive, 'r:gz') as stream:
        members = stream.getmembers()
        if len(members) != len(manifest) or {m.name for m in members} != set(manifest):
            raise ValueError('Archive members differ from original frozen manifest')
        for member in members:
            name = Path(member.name)
            if not member.isfile() or name.is_absolute() or '..' in name.parts:
                raise ValueError('Only relative ordinary source files may be extracted')
            target = destination / name
            target.parent.mkdir(parents=True, exist_ok=True)
            with stream.extractfile(member) as src, target.open('xb') as dst:
                shutil.copyfileobj(src, dst)
            target.chmod(member.mode & 0o777)
    verify_files(destination, manifest)


def link_missing_assets(origin, destination):
    """Keep frozen metadata; share new checkout assets through local links."""
    if not origin.is_dir():
        raise ValueError('Missing checkout asset directory: ' + str(origin))
    destination.mkdir(parents=True, exist_ok=True)
    for path in origin.iterdir():
        target = destination / path.name
        if not target.exists() and not target.is_symlink():
            target.symlink_to(path.resolve(), target_is_directory=path.is_dir())
        elif path.is_dir() and target.is_dir():
            link_missing_assets(path, target)
        elif path.is_file() and target.is_file() and digest(path) != digest(target):
            raise ValueError('Frozen asset metadata differs from checkout: ' + str(target))


def prepare():
    release = read_release()
    verify_files(ROOT, release['packages'])
    verify_files(ROOT, release['adapter_files'])
    verify_files(ROOT, release['assets'])
    binding = json.loads((ARTIFACTS / 'source_binding.json').read_text())
    if not RUNTIME.exists():
        RUNTIME.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.stool-reference-', dir=RUNTIME.parent) as temp:
            stage = Path(temp) / 'runtime'
            stage.mkdir()
            extract_exact(
                ARTIFACTS / 'S24_source_snapshot.tar.gz', stage / 'stool_repository', binding['source_manifest']
            )
            extract_exact(ARTIFACTS / 'S24_tools_snapshot.tar.gz', stage / 'tools', binding['tools_manifest'])
            os.replace(stage, RUNTIME)
    verify_files(SOURCE, binding['source_manifest'])
    verify_files(TOOLS, binding['tools_manifest'])
    link_missing_assets(ROOT / 'roboassemblybench/assets', SOURCE / 'roboassemblybench/assets')
    # This asset domain is optional for this task, but the base package exposes it.
    if (ROOT / 'internutopia/assets').is_dir():
        link_missing_assets(ROOT / 'internutopia/assets', SOURCE / 'internutopia/assets')
    return dict(
        passed=True,
        native_started=False,
        source_root=str(SOURCE),
        tools_root=str(TOOLS),
        source_files=len(binding['source_manifest']),
        tool_files=len(binding['tools_manifest']),
        assets_checked=len(release['assets']),
        historical_ledger_modified=False,
    )


def normalized(value, roots, top=True):
    if isinstance(value, dict):
        return {k: normalized(v, roots, False) for k, v in value.items() if not (top and k == 'recipe_fingerprint')}
    if isinstance(value, (list, tuple)):
        return [normalized(v, roots, False) for v in value]
    if isinstance(value, str):
        for root, token in sorted(roots, key=lambda pair: len(pair[0]), reverse=True):
            if value == root or value.startswith(root + '/'):
                length = len(root)
                return token + value[length:]
    return value


def differences(left, right, prefix=''):
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            path = prefix + '/' + key
            result.extend([path] if key not in left or key not in right else differences(left[key], right[key], path))
        return result
    if isinstance(left, list) and isinstance(right, list):
        result = [prefix + '/length'] if len(left) != len(right) else []
        for i, (a, b) in enumerate(zip(left, right)):
            result.extend(differences(a, b, prefix + '/' + str(i)))
        return result
    return [] if left == right else [prefix]


def configure(out=None):
    for key in list(os.environ):
        if key.startswith(('BEAM_', 'STOOL_')) and key not in ('STOOL_PYTHON', 'STOOL_EXTRA_PYTHONPATH'):
            del os.environ[key]
    release = read_release()
    os.environ.update(release['reference_environment'])
    os.environ.update(
        INTERNUTOPIA_EXTENSION_PROFILE='ur5e_assembly',
        OMNI_KIT_ACCEPT_EULA='YES',
        PYTHONNOUSERSITE='1',
        PYTHONUNBUFFERED='1',
        OMP_NUM_THREADS='4',
        MKL_NUM_THREADS='4',
        OPENBLAS_NUM_THREADS='4',
    )
    paths = [str(SOURCE), str(TOOLS)]
    paths += [p for p in os.environ.get('STOOL_EXTRA_PYTHONPATH', '').split(os.pathsep) if p]
    paths += [
        p
        for p in os.environ.get('PYTHONPATH', '').split(os.pathsep)
        if p and p not in (str(ROOT), str(SOURCE), str(TOOLS))
    ]
    os.environ['PYTHONPATH'] = os.pathsep.join(dict.fromkeys(paths))
    sys.path[:0] = list(dict.fromkeys(paths))
    if out is not None:
        os.environ.update(STOOL_RUN_DIR=str(out), STOOL_STARTUP_ONLY='0')
        for key, filename in release['trace_files'].items():
            os.environ[key] = str(out / filename)


def preflight():
    if platform.system() != 'Linux' or sys.version_info[:2] != (3, 11):
        raise ValueError('Use Linux, Python 3.11, NVIDIA GPU and Isaac Sim 5.1.0.')
    isaac = Path(os.environ.get('ISAAC_SIM_ROOT', '')).expanduser().resolve()
    if not (isaac / 'setup_conda_env.sh').is_file():
        raise ValueError('ISAAC_SIM_ROOT must point to Isaac Sim 5.1.0')
    prepared = prepare()
    configure()
    for binary in ('ffmpeg', 'ffprobe', 'nvidia-smi'):
        if not shutil.which(binary):
            raise ValueError('Required command missing from PATH: ' + binary)
    gpu = subprocess.run(
        ['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    ).stdout.strip()
    if not gpu:
        raise ValueError('NVIDIA GPU is unavailable')
    import fcl

    if importlib.metadata.version('python-fcl') != '0.7.0.8':
        raise ValueError('Install requirements/stool_assisted.txt into the Isaac environment')
    from internutopia.core.task_config_manager.base import setup_offset_for_assets
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import _plain
    from toolkits.factory_dual_franka_assembly.fabrica_assisted_preflight import (
        assert_fabrica_assisted_configuration,
    )
    from toolkits.factory_dual_franka_assembly.generate_demos import (
        _build_collect_worker_task_configs,
    )

    configs = _build_collect_worker_task_configs(
        recipes=[RECIPE],
        seeds=[0],
        layout_seeds=[4906],
        scene_profile=PROFILE,
        attach_runtime_cameras=True,
        domain_randomization_enabled=True,
        control_fps=240,
    )
    if len(configs) != 1:
        raise ValueError('Exactly one reference episode is required')
    config = configs[0]
    compiled = assert_fabrica_assisted_configuration(config)
    with gzip.open(ARTIFACTS / 'reference_worker_config.json.gz', 'rt') as stream:
        reference = json.load(stream)
    old_roots = [
        (OLD_SOURCE + '/roboassemblybench/assets', '<ASSETS>'),
        (OLD_WORKSPACE + '/candidate_repository/roboassemblybench/assets', '<ASSETS>'),
        (OLD_SOURCE, '<SOURCE>'),
        (OLD_ISAAC, '<ISAAC>'),
    ]
    roots = [
        (str(SOURCE / 'roboassemblybench/assets'), '<ASSETS>'),
        (str(ROOT / 'roboassemblybench/assets'), '<ASSETS>'),
        (str(SOURCE), '<SOURCE>'),
        (str(isaac), '<ISAAC>'),
    ]
    delta = differences(normalized(_plain(config), roots), normalized(reference, old_roots))
    if delta:
        raise ValueError('Physical configuration differs from accepted reference: ' + ', '.join(delta[:20]))
    mapped = []
    for index in (0, 1):
        candidate = copy.deepcopy(config)
        setup_offset_for_assets(candidate, index, [0.0, 0.0, 0.0])
        mapped.append(dict(env_id=index, report=assert_fabrica_assisted_configuration(candidate)))
    origins = {}
    for name in (
        'toolkits.factory_dual_franka_assembly.generate_demos',
        'toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills',
        'roboassemblybench.core.fabrica_assisted_compiler',
    ):
        path = Path(importlib.import_module(name).__file__).resolve()
        path.relative_to(SOURCE.resolve())
        origins[name] = str(path)
    for row in read_release()['isaac_files']:
        path = isaac / row['path']
        if not path.is_file() or digest(path) != row['sha256']:
            raise ValueError('Isaac robot/motion reference differs: ' + str(path))
    return dict(
        prepared,
        passed=True,
        scope='Actual collect worker -> env0/env1 mapping -> production preflight; no simulator',
        recipe=RECIPE,
        seed=0,
        layout_seed=4906,
        phase_count=compiled['phase_count'],
        configuration_signature=compiled['configuration_signature'],
        control_source_signature=read_release()['control_source_signature'],
        control_tools_signature=read_release()['control_tools_signature'],
        reference_physical_configuration_equal=True,
        relocation_metadata=['recipe_fingerprint'],
        framework_mapping=mapped,
        import_origins=origins,
        fcl_import=str(Path(fcl.__file__).resolve()),
        gpu=gpu,
    )


def worker_arguments(out):
    return [
        '--worker-mode',
        'collect',
        '--worker-recipe',
        RECIPE,
        '--worker-scene-profile',
        PROFILE,
        '--worker-results-path',
        str(out / 'collect_results.json'),
        '--worker-seeds',
        '0',
        '--worker-layout-seeds',
        '4906',
        '--max-trials',
        '1',
        '--start-seed',
        '0',
        '--record-live-video',
        '--live-video-frame-stride',
        '8',
        '--live-video-fps',
        '30',
        '--keep-video-frames',
        '--output-dir',
        str(out),
        '--domain-randomization',
        '--skip-episode-steps',
        '--runtime-constraint-monitor',
        '--constraint-check-stride',
        '8',
        '--assembly-sequence-precheck',
        '--stage-trajectory-precheck',
        '--stage-precheck-stride',
        '64',
        '--stage-precheck-waypoints',
        '4',
        '--headless',
    ]


def approve(out):
    ready = json.loads((out / 'startup_images_ready.json').read_text())
    action = json.loads((out / 'startup_first_production_action.json').read_text())
    if ready.get('physics_stamp') != action.get('physics_stamp') or action.get('passed') is not True:
        raise ValueError('Startup images and the real first action are not bound to the same physical state')
    rows = ready['images']
    if len(rows) != 3 or len({r['camera'] for r in rows}) != 3:
        raise ValueError('Three distinct startup camera images are required')
    hashes = {}
    for row in rows:
        path = Path(row['path']).resolve()
        path.relative_to(out.resolve())
        if digest(path) != row['sha256']:
            raise ValueError('Startup image changed: ' + str(path))
        hashes[row['camera']] = row['sha256']
    target = out / 'startup_image_review.json'
    # Exclusive creation: an old approval is never silently replaced or reused.
    with target.open('x') as stream:
        json.dump(
            dict(
                accepted=True,
                physics_stamp=ready['physics_stamp'],
                image_sha256=hashes,
                reviewer='Explicit portable entrypoint invocation after image review',
                scope='Startup image approval only; no task completion claim',
            ),
            stream,
            indent=2,
        )


def startup_page(out):
    ready = json.loads((out / 'startup_images_ready.json').read_text())
    figures = ''.join(
        '<figure><img style="max-width:100%" src="'
        + html.escape(Path(row['path']).name)
        + '"><figcaption>'
        + html.escape(row['camera'])
        + '</figcaption></figure>'
        for row in ready['images']
    )
    (out / 'startup_review.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>Stool 启动图</title>'
        '<h1>确认正面、左腕和右腕三张启动图</h1><p>查看后在运行终端输入 yes，或使用 --approve-startup。</p>' + figures
    )


def finish(out, code, checked):
    original = json.loads((out / 'collect_results.json').read_text()) if (out / 'collect_results.json').exists() else []
    row = original[0] if original else {}
    unchanged, identity_error = True, None
    try:
        verify_files(ROOT, read_release()['packages'])
        verify_files(ROOT, read_release()['adapter_files'])
        verify_files(ROOT, read_release()['assets'])
        frozen = json.loads((out / 'source_binding.json').read_text())
        verify_files(SOURCE, frozen['source_manifest'])
        verify_files(TOOLS, frozen['tools_manifest'])
    except (ValueError, OSError) as exc:
        unchanged, identity_error = False, str(exc)
    final = row.get('assisted_final_qualification', {}).get('current_native_recheck', {})
    success = bool(
        code == 0
        and row.get('success') is True
        and row.get('assisted_success') is True
        and final.get('ready') is True
        and unchanged
    )
    result = dict(
        functional_success=success,
        original_success=row.get('success', False),
        assisted_success=row.get('assisted_success', False),
        terminal_reason=row.get('terminal_reason'),
        steps=row.get('steps'),
        worker_exit_code=code,
        source_assets_unchanged=unchanged,
        identity_error=identity_error,
        strict_independent_terminal_verified=False,
        strict_scope=(
            'Historical S24 independent result is archived separately; '
            'this new run has not undergone that full post-audit.'
        ),
        retained_world_support=True,
        force_support_claimed=False,
        preflight=checked,
        finished_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        videos=[str(p.relative_to(out)) for p in sorted(out.glob('episode_*_live_videos/*.mp4'))],
    )
    save(out / 'result_summary.json', result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if success else 1


def run(out, no_prompt=False):
    out = out.expanduser().resolve()
    if out.exists():
        raise ValueError('Use a new output directory; existing runs are never overwritten: ' + str(out))
    if out == RUNTIME or RUNTIME in out.parents:
        raise ValueError('Run output must be separate from the frozen runtime directory')
    checked = preflight()
    out.parent.mkdir(parents=True, exist_ok=True)
    if shutil.disk_usage(out.parent).free < 30 * 10**9:
        raise ValueError('Reserve at least 30 GB for this full reference run and its original evidence')
    out.mkdir()
    save(out / 'preflight.json', checked)
    binding = json.loads((ARTIFACTS / 'source_binding.json').read_text())
    # Bind to the new location and new preflight, retaining the exact original
    # source/tools inventories. Historical launch gates and grants are not used.
    for key in ('startup_preflight', 'actual_worker_preflight_sha256', 'original_worker_baseline_sha256'):
        binding.pop(key, None)
    binding.update(
        run=out.name,
        grant=None,
        mode='portable_reference',
        run_directory=str(out),
        source_root=str(SOURCE),
        historical_reference='S24',
        configuration_signature=checked['configuration_signature'],
        actual_worker_preflight_sha256=digest(out / 'preflight.json'),
        portability_scope=(
            'Unmodified accepted controller/tools; new paths and fresh startup approval; ' 'not the historical run'
        ),
    )
    save(out / 'source_binding.json', binding)
    save(out / 'launch_gate.json', dict(checked, run=out.name))
    configure(out)
    command = [sys.executable, '-u', str(TOOLS / 'stool_worker.py')] + worker_arguments(out)
    with (out / 'worker.log').open('w') as log:
        process = subprocess.Popen(command, cwd=SOURCE, env=dict(os.environ), stdout=log, stderr=subprocess.STDOUT)
        save(
            out / 'launch_receipt.json',
            dict(
                pid=process.pid,
                command=command,
                started_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                native_completed=False,
            ),
        )
        print('Stool output: ' + str(out), flush=True)
        shown = False
        try:
            while process.poll() is None:
                if not shown and (out / 'startup_images_ready.json').exists():
                    startup_page(out)
                    shown = True
                    print('View ' + str(out / 'startup_review.html') + ' before approving.', flush=True)
                    if not no_prompt and sys.stdin.isatty():
                        if input('Reviewed all three images? Type yes to start assembly: ').strip() == 'yes':
                            approve(out)
                        else:
                            save(out / 'stop_request.json', dict(reason='Startup approval declined'))
                if (out / 'stop_request.json').exists() and not (out / 'startup_image_review.json').exists():
                    process.send_signal(signal.SIGINT)
                time.sleep(1.0)
        except KeyboardInterrupt:
            save(out / 'stop_request.json', dict(reason='User interrupted portable run'))
            if not (out / 'startup_image_review.json').exists():
                process.send_signal(signal.SIGINT)
            process.wait()
    code = process.wait()
    (out / 'exit_code').write_text(str(code) + '\n')
    (out / 'finished_at').write_text(datetime.datetime.now(datetime.timezone.utc).isoformat() + '\n')
    return finish(out, code, checked)


def main():
    parser = argparse.ArgumentParser(
        description='Accepted nine-part Stool assisted reference; see docs/stool_assisted/RUN.md'
    )
    parser.add_argument(
        '--prepare', action='store_true', help='Verify/extract the frozen source and link checkout assets; no simulator'
    )
    parser.add_argument('--check', action='store_true', help='Real worker construction/mapping/preflight; no simulator')
    parser.add_argument('--output-dir', type=Path, help='Start a full simulation in a new directory')
    parser.add_argument(
        '--approve-startup', type=Path, help='Approve three reviewed images and the bound first real action'
    )
    parser.add_argument('--stop', type=Path, help='Request a graceful stop')
    parser.add_argument('--no-prompt', action='store_true', help='Use a second terminal to approve startup')
    args = parser.parse_args()
    modes = [
        args.prepare,
        args.check,
        args.output_dir is not None,
        args.approve_startup is not None,
        args.stop is not None,
    ]
    if sum(modes) != 1:
        parser.error('Choose exactly one of --prepare, --check, --output-dir, --approve-startup or --stop')
    if args.prepare:
        print(json.dumps(prepare(), indent=2))
        return 0
    if args.approve_startup:
        approve(args.approve_startup.expanduser().resolve())
        return 0
    if args.stop:
        out = args.stop.expanduser().resolve()
        if not (out / 'launch_receipt.json').is_file() or (out / 'finished_at').exists():
            raise ValueError('Not an active portable Stool run')
        save(out / 'stop_request.json', dict(reason='Explicit portable --stop request'))
        return 0
    if args.check:
        print(json.dumps(preflight(), indent=2))
        return 0
    return run(args.output_dir, args.no_prompt)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, ImportError, subprocess.SubprocessError) as exc:
        print('Stool entrypoint: ' + str(exc), file=sys.stderr)
        sys.exit(2)
