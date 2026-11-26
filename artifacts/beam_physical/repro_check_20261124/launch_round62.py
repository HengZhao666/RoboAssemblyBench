"""One authorized round-62 launch (batch 62-66, 2026-11-24). Never replays an existing launch."""
import datetime, hashlib, json, os, shutil, subprocess, sys
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae'); C = R/'candidate_repository'
B = R/'grasp_repairs_20261012_20rounds'; Q = B/'round62_v239_unload'; OUT = C/'outputs/fabrica_beam_v239_unload'
ROUND = 62
p = B/'round_ledger.json'; raw = p.read_bytes(); l = json.loads(raw); batch = l['current_batch']
assert l['active_round'] is None and l['next_round'] == ROUND and l['native_continuation_authorized'] is True
assert not any(l.get(k) for k in ['paused', 'pause_pending', 'pause_after_round', 'no_new_native_after_current', 'authorization_exhausted'])
assert not batch['paused'] and batch['round_numbers'] == [62, 66] and batch['remaining_rounds'] == 5
assert batch['currently_resumed_formal_rounds'] == [62, 63, 64, 65, 66] and batch['unstarted_rounds_on_hold'] == []
assert l['round51_final_acceptance_approval']['human_answer'] == '允许改为位姿稳定验收，原始速度单独报告'
# Round 59 is the first full-assembly success; round 62 runs after it by the human instruction
# of 2026-11-24 (batch 62-66, "授权五轮，或修好停止"), recorded in the batch authorization file.
assert [r['round'] for r in l['rounds'] if r.get('full_assembly_success') is True] == [59]
assert json.loads(Path(batch['authorization_record']).read_text())['human_answer'].startswith('请查清方案')
row = next(r for r in l['rounds'] if r['round'] == ROUND)
assert row['status'] == 'ready_for_native' and not row['native_started'] and not row.get('launch_attempted')
assert not OUT.exists() and not (Q/'supervisor_launch.json').exists(), 'Existing launch intent must be reconciled; never replay'
sha = lambda f: hashlib.sha256(Path(f).read_bytes()).hexdigest()
review = json.loads((Q/'candidate_review.json').read_text()); assert review['passed'] and review['human_pose_acceptance_approved']
assert sha(Q/'source_manifest.json') == row['source_manifest_sha256'] == review['source_manifest_sha256']
assert sha(Q/'production_worker_config.json') == row['worker_config_sha256'] == review['worker_config_sha256']
for name in ['source_manifest', 'tools_manifest', 'launch_identity_manifest']:
    for f in json.loads((Q/(name+'.json')).read_text())['files']:
        assert sha((C if name == 'source_manifest' else Q)/f['path']) == f['sha256'], f['path']
for f in json.loads((Q/'close_geometry_asset_manifest.json').read_text())['files']: assert sha(f['path']) == f['sha256']
for f in json.loads((Q/'asset_manifest.json').read_text())['files']: assert sha(f['path']) == f['sha256']
smoke = json.loads((Q/'smoke_review.json').read_text()); assert smoke['passed']
sys.path.insert(0, str(Q/'tools'))
from resource_budget import startup_memory_required
minimum = startup_memory_required(16, l.get('shared_resource_permission', {}).get('allowed') is True, batch=batch, round_number=ROUND)
memory = next(int(v.split()[1])*1024 for v in Path('/proc/meminfo').read_text().splitlines() if v.startswith('MemAvailable:'))
disk = shutil.disk_usage(R).free
gpu = subprocess.run(['nvidia-smi', '--id=GPU-e064bda0-b445-2d7f-50d5-c1180deadea2', '--query-gpu=memory.free', '--format=csv,noheader,nounits'], capture_output=True, text=True, check=True)
snapshot = {'memory_available_GiB': memory/2**30, 'disk_free_GiB': disk/2**30, 'GPU_free_MiB': int(gpu.stdout.strip())}
if memory < minimum or disk < 19*2**30 or snapshot['GPU_free_MiB'] < 12288:
    print(json.dumps({'status': 'blocked_by_current_resource_admission', 'native_started': False, 'snapshot': snapshot})); raise SystemExit(0)
assert set(range(8)).issubset(os.sched_getaffinity(0)) and p.read_bytes() == raw
intent = {'round': ROUND, 'state': 'launch_intent', 'at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'resource_snapshot': snapshot,
          'source_manifest_sha256': row['source_manifest_sha256'], 'worker_config_sha256': row['worker_config_sha256']}
(Q/'supervisor_launch.json').write_text(json.dumps(intent, indent=2)+'\n')
l.update(active_round=ROUND, status='round62_launching'); t = p.with_name(p.name+'.launch62.tmp'); t.write_text(json.dumps(l, ensure_ascii=False, indent=2)+'\n'); t.replace(p)
env = dict(os.environ, BEAM_REPAIR_ROUND=str(ROUND), BEAM_OUTPUT_NAME='fabrica_beam_v239_unload', BEAM_PHYSICAL_ACCEPTANCE_PROFILE='plumbers_staged_v2',
           BEAM_EMPTY_RETREAT_LOCAL_IK='1', BEAM_COLUMN2_SIDE_CLEARANCE_ROUTE='1', BEAM_COMMON_POSE_WINDOW='1', BEAM_FINAL_POSE_ACCEPTANCE='1',
           BEAM_STREAMING_CAPTURE='1', BEAM_FULL_OUTPUT_BUDGET_GIB='14', BEAM_START_MEMORY_MIN_GIB='16', BEAM_RESOURCE_CHECKPOINT_PHASE_INDEX='46',
           BEAM_PHYSICS_SOLVER_TYPE='TGS', BEAM_EXTERNAL_FORCES_EVERY_ITERATION='0')
with (Q/'supervisor.log').open('ab', buffering=0) as log:
    child = subprocess.Popen(['/usr/bin/taskset', '-c', '0-7', sys.executable, '-B', str(Q/'tools/supervise.py')], cwd=C, env=env,
                             stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
intent.update(state='supervisor_spawned', supervisor_pid=child.pid); (Q/'supervisor_launch.json').write_text(json.dumps(intent, indent=2)+'\n')
print(json.dumps({'status': 'supervisor_spawned', 'round': ROUND, 'supervisor_pid': child.pid, 'snapshot': snapshot}))
