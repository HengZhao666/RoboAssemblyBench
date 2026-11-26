"""One formal from-start Stool physical round per invocation (max 5, stop on success).

Sole authority: outputs/stool_physical/ledger.json. Freezes the source delta and
SHA list before spawning; counts the round once the validator is spawned.
Lightweight output: no debug traces, 10 fps / stride 24 streamed video.
"""
import datetime, fcntl, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

ROOT = Path('/home/user/work/zhaoheng/stool_physical_20261120')
OUT = ROOT / 'outputs/stool_physical'
LEDGER = OUT / 'ledger.json'
PY = '/Embodied_AI/miniconda3/envs/internutopia311/bin/python'
GIT = ['/Embodied_AI/miniconda3/bin/git', '-C', str(ROOT)]
GPU = 'GPU-e064bda0-b445-2d7f-50d5-c1180deadea2'
RECIPE = 'fabrica_stool_circular_ur5e_physical'


def now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def save(ledger):
    tmp = LEDGER.with_suffix('.tmp'); tmp.write_text(json.dumps(ledger, indent=2, ensure_ascii=False) + '\n'); tmp.replace(LEDGER)


def main():
    package = sys.argv[1]
    assert os.environ.get('PLUMBERS_REFERENCE_RUNTIME') == 'locked', 'source integration/runtime_env.sh first'
    lock = (OUT / 'launch.lock').open('a+'); fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    ledger = json.loads(LEDGER.read_text())
    auth = ledger['authorization']
    rounds = ledger['rounds']
    assert ledger['active_round'] is None, 'a round is active'
    assert not any(r.get('verified_success') for r in rounds), 'stop after verified success'
    assert all(r.get('review_complete') for r in rounds), 'review previous round first'
    assert len(rounds) < auth['maximum_formal_rounds'], 'round budget exhausted'
    assert not ledger.get('human_pause_requested'), 'paused by human'
    number = len(rounds) + 1
    free = shutil.disk_usage(ROOT).free / 1024**3
    assert free >= 20, f'free disk {free:.1f} GiB < 20'
    mem = next(x for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:'))
    assert int(mem.split()[1]) > 12 * 1024**2, 'memory'
    gpu = subprocess.check_output(['nvidia-smi', '--query-gpu=uuid,memory.free', '--format=csv,noheader,nounits']).decode()
    assert any(l.split(',')[0].strip() == GPU and int(l.split(',')[1]) > 12 * 1024 for l in gpu.splitlines()), 'gpu'
    assert not subprocess.run(['pgrep', '-f', 'plumbers_validate.py'], stdout=subprocess.PIPE).stdout.strip(), 'validator running'
    # Freeze the exact source delta of this package.
    pkg = OUT / 'packages' / package; pkg.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, GIT_NO_LAZY_FETCH='1')
    subprocess.run(GIT + ['add', '--sparse', '-A', 'roboassemblybench/core', 'roboassemblybench/tasks/' + RECIPE,
                          'toolkits/factory_dual_franka_assembly', 'internutopia_extension/tasks', 'integration/phase_journal.py'],
                   check=True, env=env, timeout=120)
    delta = subprocess.check_output(GIT + ['diff', '--cached', '--binary'], env=env, timeout=120)
    names = subprocess.check_output(GIT + ['diff', '--cached', '--name-only'], env=env, timeout=120).decode().split()
    shas = {n: hashlib.sha256((ROOT / n).read_bytes()).hexdigest() for n in names}
    (pkg / 'delta_from_6f1de1d.patch').write_bytes(delta)
    (pkg / 'source_sha256.json').write_text(json.dumps(shas, indent=1) + '\n')
    tz = datetime.timezone(datetime.timedelta(hours=8))
    run = OUT / 'rounds' / f'R{number:02d}_{datetime.datetime.now(tz).strftime("%Y%m%d_%H%M%S")}_{package}'
    run.mkdir(parents=True)
    command = [PY, '-B', str(ROOT / 'integration/plumbers_validate.py'), '--tasks', RECIPE,
               '--seed', '4906', '--layout-seed', '4906', '--output-dir', str(run / 'native'),
               '--worker-timeout-seconds', '14400', '--record-live-video', '--live-video-fps', '10',
               '--live-video-frame-stride', '24', '--live-video-encoder', 'stream',
               # Close-phase traces only (file writes, ~70 MB/run) for failure diagnosis.
               '--full-debug-traces']
    record = dict(round=number, package=package, run_directory=str(run), started_at=now(),
                  delta_sha256=hashlib.sha256(delta).hexdigest(), source_sha256=shas,
                  resource_start_free_gib=round(free, 2), command=command, state='running', review_complete=False)
    run_env = {k: v for k, v in os.environ.items() if not k.startswith('BEAM_')}
    run_env.pop('PLUMBERS_HANDOFF_REPAIR', None)
    run_env['BEAM_CLOSE_CONTROL_TRACE_PATH'] = str(run / 'close_control_trace.jsonl')
    run_env['BEAM_CLOSE_GATE_TRACE_PATH'] = str(run / 'close_gate_trace.jsonl')
    with (run / 'launcher.log').open('w') as log:
        process = subprocess.Popen(command, cwd=ROOT, env=run_env, stdout=log, stderr=subprocess.STDOUT)
        record['pid'] = process.pid
        rounds.append(record); ledger.update(active_round=number, state='running'); save(ledger)
        (run / 'launch.json').write_text(json.dumps(record, indent=2) + '\n')
        code = process.wait()
    ledger = json.loads(LEDGER.read_text())
    summary_path = run / 'native/validation_summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    task = (summary or {}).get('tasks', [{}])[0] if summary else {}
    rec = ledger['rounds'][number - 1]
    rec.update(state='finished', finished_at=now(), exit_code=code,
               raw_success=bool(summary and summary.get('num_successful') == 1),
               steps=task.get('steps'), terminal_reason=task.get('terminal_reason'),
               terminal_phase=task.get('terminal_phase'))
    ledger.update(active_round=None, state='round_finished_review_pending')
    save(ledger)
    print(json.dumps({k: rec[k] for k in ('round', 'raw_success', 'steps', 'terminal_reason', 'terminal_phase')}))


if __name__ == '__main__':
    main()
