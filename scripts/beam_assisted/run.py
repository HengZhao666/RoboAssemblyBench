#!/usr/bin/env python3
"""Portable, source-bound execution of the validated Beam assisted reference."""
from __future__ import annotations

import argparse
import datetime
import hashlib
import html
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import select
import shutil
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'outputs/beam_assisted_20261015/tools'
HELPERS = Path(__file__).resolve().parent / 'runtime'
RELEASE = ROOT / 'artifacts/beam_assisted/portable_release.json'
REFERENCE = ROOT / 'artifacts/beam_assisted/portable_reference_config.json'
ASSET_URL = 'https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1'
RECIPE = 'fabrica_beam_ur5e_assisted'
PROFILE = 'taoyuan_grscenes_tabletop'


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp.' + str(os.getpid()))
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    os.replace(temporary, path)


def checkout_path(relative):
    path = ROOT / relative
    if Path(relative).is_absolute() or '..' in Path(relative).parts:
        raise ValueError('Unsafe release path: ' + relative)
    path.resolve().relative_to(ROOT.resolve())
    return path


def runtime_identity(release):
    fields = ('runtime_files', 'assets', 'isaac_files', 'reference_environment',
              'trace_files', 'primary_scene', 'physics_source_version')
    value = {k: release[k] for k in fields}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def verify_release(release=None):
    release = release if release is not None else json.loads(RELEASE.read_text())
    failures = []
    for group in ('runtime_files', 'assets'):
        for name, expected in release[group].items():
            try:
                file = checkout_path(name)
                if not file.is_file() or digest(file) != expected:
                    failures.append(name)
            except (ValueError, OSError):
                failures.append(name)
    if failures:
        raise ValueError('Missing/changed reference files or assets (re-download this release): '
                         + ', '.join(failures[:12]))
    return release


def normalized(value, roots, top=True):
    if isinstance(value, dict):
        return {k: normalized(v, roots, False) for k, v in value.items()
                if not (top and k == 'recipe_fingerprint')}
    if isinstance(value, (list, tuple)):
        return [normalized(v, roots, False) for v in value]
    if isinstance(value, str):
        for root, token in sorted(roots, key=lambda pair: len(pair[0]), reverse=True):
            if value == root or value.startswith(root + '/'):
                return token + value[len(root):]
    return value


def differences(left, right, prefix=''):
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(set(left) | set(right)):
            path = prefix + '/' + key
            result.extend([path] if key not in left or key not in right
                          else differences(left[key], right[key], path))
        return result
    if isinstance(left, list) and isinstance(right, list):
        result = [prefix + '/length'] if len(left) != len(right) else []
        for i, (a, b) in enumerate(zip(left, right)):
            result.extend(differences(a, b, prefix + '/' + str(i)))
        return result
    return [] if left == right else [prefix]


def configure_environment(output=None):
    # These are the unchanged reference launcher settings, not a tuning surface.
    release = json.loads(RELEASE.read_text())
    for key in list(os.environ):
        if key.startswith('BEAM_') and key not in ('BEAM_PYTHON', 'BEAM_EXTRA_PYTHONPATH'):
            del os.environ[key]
    os.environ.update(release['reference_environment'])
    os.environ.update(BEAM_ROUTE_SOURCE=str(ROOT), BEAM_EXECUTION_ROUTE='assisted',
                      BEAM_ASSISTED_TOOLS=str(TOOLS), ISAAC_ASSETS_ROOT=ASSET_URL,
                      INTERNUTOPIA_EXTENSION_PROFILE='ur5e_assembly',
                      OMNI_KIT_ACCEPT_EULA='YES', PYTHONNOUSERSITE='1', PYTHONUNBUFFERED='1',
                      OMP_NUM_THREADS='4', MKL_NUM_THREADS='4', OPENBLAS_NUM_THREADS='4')
    paths = [str(ROOT), str(HELPERS), str(TOOLS)]
    paths += [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]
    os.environ['PYTHONPATH'] = os.pathsep.join(dict.fromkeys(paths))
    sys.path[:0] = paths[:3]
    if output is not None:
        out = Path(output)
        os.environ['BEAM_ASSISTED_OUTPUT_DIR'] = str(out)
        for key, filename in release['trace_files'].items():
            os.environ[key] = str(out / filename)


def preflight():
    if platform.system() != 'Linux' or sys.version_info[:2] != (3, 11):
        raise ValueError('Supported runtime: Linux + NVIDIA GPU + Python 3.11 + Isaac Sim 5.1.0.')
    isaac = Path(os.environ.get('ISAAC_SIM_ROOT', '')).expanduser().resolve()
    if not (isaac / 'setup_conda_env.sh').is_file():
        raise ValueError('ISAAC_SIM_ROOT must point to your installed Isaac Sim 5.1.0.')
    for binary in ('ffmpeg', 'ffprobe', 'nvidia-smi'):
        if not shutil.which(binary):
            raise ValueError('Required command is missing from PATH: ' + binary)
    gpu = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader'],
                         capture_output=True, text=True, timeout=15)
    if gpu.returncode or not gpu.stdout.strip():
        raise ValueError('NVIDIA driver/GPU is unavailable: ' + gpu.stderr[-500:])
    release = verify_release()
    versions = {}
    for name in ('numpy', 'scipy', 'pydantic', 'Pillow', 'PyYAML', 'opencv-python', 'python-fcl'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    if versions['python-fcl'] != '0.7.0.8':
        raise ValueError('Install the assisted override in this environment: '
                         'python -m pip install -r requirements/beam_assisted.txt')
    if not (versions.get('pydantic') or '').startswith('2.'):
        raise ValueError('The project base environment requires Pydantic 2; see docs/beam_assisted/RUN.md.')
    try:
        import fcl
        import cv2
        import scipy
        import yaml
        import numpy
    except ImportError as exc:
        raise ValueError('Incomplete project base environment; follow docs/beam_assisted/RUN.md. ' + str(exc)) from exc
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import (
        assert_assisted_configuration, _plain)
    from internutopia.core.task_config_manager.base import setup_offset_for_assets
    import copy
    config = build_dual_franka_assembly_episode(recipe=RECIPE, scene_profile=PROFILE,
        seed=0, layout_seed=505, domain_randomization_enabled=True)
    compiled = assert_assisted_configuration(config)
    actual = _plain(config)
    expected = json.loads(REFERENCE.read_text())
    delta = differences(normalized(actual, [(str(ROOT), '<REPO>'), (str(isaac), '<ISAAC>')]),
                        expected['configuration'])
    if delta:
        raise ValueError('Configuration differs from the validated physical reference: '
                         + ', '.join(delta[:12]))
    mapped = copy.deepcopy(config)
    setup_offset_for_assets(mapped, 0, [0., 0., 0.])
    runtime = assert_assisted_configuration(mapped)
    for entry in release.get('isaac_files', []):
        file = isaac / entry['path']
        if not file.is_file() or digest(file) != entry['sha256']:
            raise ValueError('Isaac reference file is missing/changed: ' + str(file))
    return dict(passed=True, scope='Real task builder -> env0 mapping -> production preflight; no simulator',
        source_root=str(ROOT), isaac_sim_root=str(isaac), recipe=RECIPE, seed=0, layout_seed=505,
        scene_profile=PROFILE, configuration_signature=compiled['configuration_signature'],
        phase_target_signature=compiled['phase_target_signature'], mapped_preflight_passed=runtime['passed'],
        reference_physical_configuration_equal=True, relocation_only_metadata=['recipe_fingerprint'],
        runtime_files_checked=len(release['runtime_files']), assets_checked=len(release['assets']),
        versions=versions, fcl_import=str(Path(fcl.__file__).resolve()), gpu=gpu.stdout.strip(),
        portable_release_sha256=digest(RELEASE), runtime_binding_sha256=runtime_identity(release), native_started=False)


def control(directory, kind):
    sys.path.insert(0, str(TOOLS))
    from publish_runtime_control import publish, publish_stop_request
    directory = Path(directory).expanduser().resolve()
    if kind == 'stop':
        return publish_stop_request(directory, {'reason': 'User requested stop through portable entrypoint'})
    ready = json.loads((directory / 'startup_images_ready.json').read_text())
    return publish(directory, directory.name, 'ack', dict(run_name=directory.name, accepted=True,
        reviewer='Invoker of the portable startup approval',
        scope='Explicit --approve-startup invocation or terminal yes after viewing startup_review.html; no success claim',
        physics_stamp=ready['physics_stamp'], image_sha256={r['camera']: r['sha256'] for r in ready['images']}))


def worker_arguments(out, keep_frames=False):
    args = ['--worker-mode', 'collect', '--worker-recipe', RECIPE,
        '--worker-scene-profile', PROFILE, '--worker-results-path', str(out / 'collect_results.json'),
        '--worker-seeds', '0', '--worker-layout-seeds', '505', '--max-trials', '1', '--start-seed', '0',
        '--record-live-video', '--live-video-frame-stride', '8', '--live-video-fps', '30',
        '--output-dir', str(out), '--domain-randomization', '--skip-episode-steps',
        '--runtime-constraint-monitor', '--constraint-check-stride', '8',
        '--assembly-sequence-precheck', '--stage-trajectory-precheck',
        '--stage-precheck-stride', '64', '--stage-precheck-waypoints', '4', '--headless']
    if keep_frames:
        args.append('--keep-video-frames')
    return args


def startup_page(out):
    ready = json.loads((out / 'startup_images_ready.json').read_text())
    images = ''.join('<figure><img style="max-width:100%" src="' + html.escape(Path(r['path']).name)
                     + '"><figcaption>' + html.escape(r['camera']) + '</figcaption></figure>' for r in ready['images'])
    page = '<!doctype html><meta charset="utf-8"><title>Beam startup review</title><h1>检查三路启动图</h1>'
    page += '<p>确认两台机械臂、零件和三个相机正常，左右腕夹爪在画面下方，再在终端输入 yes。</p>' + images
    (out / 'startup_review.html').write_text(page)


def finish(out, worker_code, watcher_code, checked):
    results = json.loads((out / 'collect_results.json').read_text()) if (out / 'collect_results.json').exists() else []
    row = results[0] if results else {}
    current = row.get('assisted_final_qualification', {}).get('current_native_recheck', {})
    guard = json.loads((out / 'visible_mesh_runtime_report.json').read_text()) if (out / 'visible_mesh_runtime_report.json').exists() else {}
    unchanged = True
    try:
        frozen = json.loads((out / 'runtime_release.json').read_text())
        if runtime_identity(frozen) != checked['runtime_binding_sha256']:
            raise ValueError('Frozen runtime binding changed')
        verify_release(frozen)
    except (ValueError, OSError):
        unchanged = False
    monitor_passed = bool(watcher_code == 0 and guard.get('finished_marker_seen') is True
        and guard.get('samples', 0) > 0 and guard.get('failures_above_precision') == []
        and guard.get('stop_requested') is False)
    success = bool(worker_code == 0 and monitor_passed and row.get('success') is True
                   and row.get('assisted_success') is True and current.get('ready') is True and unchanged)
    report = dict(functional_success=success, assisted_success=row.get('assisted_success', False),
        recipe=RECIPE, seed=0, layout_seed=505, steps=row.get('steps'), terminal_reason=row.get('terminal_reason'),
        worker_exit_code=worker_code, geometry_monitor_exit_code=watcher_code,
        geometry_monitor_passed=monitor_passed, geometry_failures=len(guard.get('failures_above_precision', [])),
        geometry_samples=guard.get('samples'), geometry_gaps=guard.get('gaps'), source_assets_unchanged=unchanged,
        preflight=checked, strict_independent_terminal_verified=False,
        limits=['Auxiliary fixed joints and installation support remain active.',
                'This summary verifies the original functional contract, not full independent terminal raw coverage.'],
        output_dir=str(out), finished_at=datetime.datetime.now(datetime.timezone.utc).isoformat())
    save(out / 'result_summary.json', report)
    return success


def launch(args, checked):
    out = (args.output_dir or ROOT / 'outputs/beam_assisted' /
           datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')).expanduser().resolve()
    if out.exists():
        raise ValueError('Output already exists; choose a new --output-dir: ' + str(out))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.mkdir()
    configure_environment(out)
    save(out / 'preflight.json', checked)
    save(out / 'runtime_release.json', json.loads(RELEASE.read_text()))
    sys.path.insert(0, str(TOOLS))
    from bounded_scene_http_probe import fetch
    network = fetch()
    expected = json.loads(RELEASE.read_text())['primary_scene']
    if network['sha256'] != expected['sha256']:
        raise ValueError('Official primary scene differs from the recorded reference; do not run')
    bounded = dict(url=network['url'], sha256=network['sha256'], helper_sha256=digest(TOOLS / 'bounded_scene_http_probe.py'))
    save(out / 'launch_gate.json', {'gate': dict(route='assisted', run_name=out.name,
        actual_configuration_signature=checked['configuration_signature'],
        native_initialization_check_required=True, bounded_primary_scene_probe=bounded,
        portable_preflight='preflight.json', old_experiment_ledger_required=False)})
    save(out / 'source_before.json', dict(route='assisted', source=str(ROOT),
        portable_release_sha256=digest(RELEASE), files_checked=checked['runtime_files_checked'], differences=[]))
    (out / 'started_at').write_text(datetime.datetime.now(datetime.timezone.utc).isoformat() + '\n')
    command = [sys.executable, str(HELPERS / 'worker.py')] + worker_arguments(out, args.keep_video_frames)
    save(out / 'run_configuration.json', dict(command=command, source_root=str(ROOT), output_dir=str(out),
        reference_environment=json.loads(RELEASE.read_text())['reference_environment']))
    env = os.environ.copy()
    print('运行目录：' + str(out), flush=True)
    worker = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True)
    with (out / 'geometry_monitor.log').open('w') as guard_log:
        watcher = subprocess.Popen([sys.executable, str(HELPERS / 'watch_visible_meshes.py')],
            cwd=ROOT, env=env, stdout=guard_log, stderr=subprocess.STDOUT, start_new_session=True)
        def copy_log():
            last_phase = None
            last_print = 0.
            with (out / 'worker.log').open('w') as log:
                for line in worker.stdout:
                    log.write(line)
                    log.flush()
                    match = re.search(r'\bphase=(\S+)', line)
                    phase = match.group(1) if match else None
                    if ('[rollout-progress]' in line and
                        (phase != last_phase or time.monotonic() - last_print >= 15)) or 'Traceback' in line:
                        print(line.rstrip(), flush=True)
                        last_phase, last_print = phase, time.monotonic()
        reader = threading.Thread(target=copy_log, daemon=True)
        reader.start()
        shown = False
        try:
            while worker.poll() is None:
                if watcher.poll() is not None and not (out / 'stop_request.json').exists():
                    sys.path.insert(0, str(TOOLS))
                    from publish_runtime_control import publish_stop_request
                    publish_stop_request(out, {'reason': 'Portable independent geometry monitor exited before the worker',
                                              'monitor_returncode': watcher.returncode})
                if not shown and (out / 'startup_images_ready.json').exists():
                    startup_page(out)
                    print('请查看三张启动图：' + str(out / 'startup_review.html'), flush=True)
                    print('查看后输入 yes；非交互终端请另行运行：', flush=True)
                    print('bash roboassemblybench/scripts/run_beam_assisted.sh --approve-startup "' + str(out) + '"', flush=True)
                    shown = True
                if shown and not args.no_prompt and sys.stdin.isatty() and not (out / 'startup_image_review.json').exists():
                    if select.select([sys.stdin], [], [], .1)[0]:
                        answer = sys.stdin.readline().strip().lower()
                        if answer == 'yes':
                            control(out, 'ack')
                        elif answer:
                            control(out, 'stop')
                time.sleep(.3)
        except KeyboardInterrupt:
            print('已请求停止，保留原有物理尾录和日志……', flush=True)
            control(out, 'stop')
            worker.wait()
        finally:
            if worker.poll() is None:
                worker.terminate()
                worker.wait()
            reader.join(timeout=15)
            (out / 'exit_code').write_text(str(worker.returncode) + '\n')
            (out / 'finished_at').write_text(datetime.datetime.now(datetime.timezone.utc).isoformat() + '\n')
            try:
                watcher.wait(timeout=120)
            except subprocess.TimeoutExpired:
                watcher.terminate()
                watcher.wait()
    success = finish(out, worker.returncode, watcher.returncode, checked)
    print(('五件辅助装配和双臂回位已完成。' if success else '运行未通过，请查看 result_summary.json 和 worker.log。')
          + '\n结果：' + str(out / 'result_summary.json'), flush=True)
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-sim-root', type=Path, help='Your Isaac Sim 5.1.0 installation')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--check', action='store_true', help='Check dependencies/assets and build configuration, without simulation')
    parser.add_argument('--report', type=Path, help='Optional preflight JSON path')
    parser.add_argument('--approve-startup', type=Path, metavar='RUN_DIR', help='After viewing that run\'s three startup images')
    parser.add_argument('--stop', type=Path, metavar='RUN_DIR')
    parser.add_argument('--no-prompt', action='store_true', help='Use --approve-startup from another terminal')
    parser.add_argument('--keep-video-frames', action='store_true', help='Keep the optional PNG frame cache (~8 GB)')
    args = parser.parse_args()
    if args.approve_startup or args.stop:
        print(json.dumps(control(args.approve_startup or args.stop, 'ack' if args.approve_startup else 'stop')))
        return 0
    if args.isaac_sim_root:
        os.environ['ISAAC_SIM_ROOT'] = str(args.isaac_sim_root.expanduser().resolve())
    configure_environment()
    checked = preflight()
    if args.report:
        save(args.report, checked)
    if args.check:
        print(json.dumps(checked, ensure_ascii=False, indent=2))
        return 0
    return launch(args, checked)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, ImportError, subprocess.SubprocessError) as error:
        print('Beam assisted: ' + str(error), file=sys.stderr)
        raise SystemExit(2)
