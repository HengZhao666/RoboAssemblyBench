"""Synchronize closure metadata and verify pause; never runs/stops a simulator."""
from pathlib import Path
import datetime, hashlib, json, os, sys, tarfile, io, fcntl
R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
P = R / 'beam_assisted_20261015'
D = P / 'runs/A12_beam_full'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p): return json.loads(p.read_text())
payload = sys.stdin.buffer.read()
synced = []
with tarfile.open(fileobj=io.BytesIO(payload), mode='r:gz') as t:
    for m in t.getmembers():
        n = Path(m.name)
        assert m.isfile() and not n.is_absolute() and '..' not in n.parts
        data = t.extractfile(m).read()
        dest = P / n
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        h = hashlib.sha256(data).hexdigest()
        assert sha(dest) == h
        synced.append({'path': str(n), 'sha256': h, 'bytes': len(data)})
expected = read(P / 'documentation_expected_sha256.json')
docs = []
for root in [R, R/'candidate_repository', Path('/home/user/RoboAssemblyBench'), P/'assisted_repository']:
    for n, h in expected.items():
        assert sha(root/n) == h, str(root/n)
        docs.append({'path': str(root/n), 'sha256': h})
source_after = read(D/'source_after.json')
assert not source_after['differences'] and not source_after['tool_differences']
counts = {}
for label, root, manifest in [
    ('physical', R/'candidate_repository', P/'tools/physical_manifest.json'),
    ('assisted', P/'assisted_repository', P/'tools/assisted_manifest.json'),
    ('critical_tools', P/'tools', P/'tools/assisted_tools_manifest.json')]:
    values = read(manifest)
    for n, h in values.items(): assert sha(root/n) == h, f'{label}: {n}'
    counts[label] = {'files': len(values), 'differences': [], 'manifest_sha256': sha(manifest)}
assert counts['physical']['files'] == 876 and counts['assisted']['files'] == 887 and counts['critical_tools']['files'] == 9
for v in read(D/'video_manifest.json'):
    p = D/v['relative_path']
    assert p.stat().st_size == v['size'] and sha(p) == v['sha256']
    assert v['decode_exit'] == 0 and not v['decode_errors'].strip()
for v in read(P/'A12_key_nodes/manifest.json')['clips']:
    p = Path(v['path'])
    assert p.stat().st_size == v['size'] and sha(p) == v['sha256'] and v['full_decode_exit'] == 0
archives = []
for n, h in [
    ('preserved_v201_source876.tar.gz', 'b29ec75309254201f7ceefbe9fa6d02ef1cf085917b864455e7990e134e052c3'),
    ('versions/assisted_v16b_before_A12_final.tar.gz', '64d2b28f26264f188723f0a7bc56804028b287f00f5db3db2dac4a4c19c1fc42'),
    ('versions/assisted_v16b_tools_before_A12_final.tar.gz', '8d807eb94bc979a3806e701ad0f49102664ad0f71226fdcfc75d3d600b7a0599'),
    ('A12_beam_full_evidence.tar.gz', '39409905fdef6eb74834f636d0407d02aa4186a485b4b79cdd5129c4f00c043c'),
    ('A12_beam_full_review_bundle.tar.gz', '653e10458e87656957ab27cfcfeb5500ef501ecfc029d54b497171b1c64ae469'),
    ('A12_key_nodes.tar.gz', 'cebad60b683475341a932d7885d9b7039d8c73ab2d1afba13900ac85b805563e')]:
    p = P/n
    assert sha(p) == h, n
    archives.append({'path': str(p), 'sha256': h, 'bytes': p.stat().st_size})
assert (D/'finished_at').exists() and read(D/'post_failure_recording.json')['completed']
active = []
targets = {str(P/'tools/assisted_worker.py'), str(P/'tools/run_beam_route.sh')}
for proc in Path('/proc').glob('[0-9]*'):
    try: args = (proc/'cmdline').read_bytes().split(b'\0')
    except (FileNotFoundError, ProcessLookupError, PermissionError): continue
    if any(a.decode(errors='replace') in targets for a in args): active.append({'pid': int(proc.name), 'args': [a.decode(errors='replace') for a in args]})
assert not active, active
with (P/'beam_run.lock').open('a') as lock:
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
batch = read(P/'review_batch_20261015.json')
contract = read(P/'experiment_contract.json')
assert batch['completed_cycles'] == 2 and batch['status'] == 'paused_by_user_after_B02'
assert not batch['next_native_run_authorized'] and not contract['next_native_run_authorized']
report = {'verified_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'status': 'B02_closed_user_paused', 'run': 'A12_beam_full',
    'finished_at': (D/'finished_at').read_text().strip(), 'active_route_processes': active,
    'lock_free': True, 'next_native_run_authorized': False,
    'completed_cycles': 2, 'B03_B05_deferred': True, 'source_verification': counts,
    'documentation_files_verified': len(docs), 'documentation': docs,
    'metadata_files_verified': len(synced), 'metadata': synced, 'archives': archives,
    'original_videos': 3, 'key_clips': 5, 'video_size_sha_verified_current_server': True,
    'local_downloads': 'Verified local size/SHA certificates synchronized in metadata',
    'success': False, 'assisted_success': False, 'formal_success': False,
    'unrelated_processes_modified': False, 'new_simulation_started': False}
(P/'B02_final_pause_certificate.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({k: report[k] for k in ['status','lock_free','documentation_files_verified','metadata_files_verified','source_verification','original_videos','key_clips','next_native_run_authorized']}))
