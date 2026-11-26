"""Stage round 62 (v239_unload): round dir, tools, compiler deployment, source manifest.

Run once on the server after offline_v239_unload/config_check.json passed. Only
the compiler changes against round 61 (lift completion tolerance max(lift/2, 0.4 mm)); the support-state module and the worker are
the round-56/57 files, whose gate and velocity-cache checks are reused (copied
with their SHAs). tools/preflight.py gets the new flag in its environment.
Verifies the repository still holds the round-61 compiler bytes before
replacing it. Starts no simulation and does not touch the ledger.
"""
import hashlib
import json
import shutil
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
Q61 = B / 'round61_v238_lift04'
P61 = B / 'offline_v238_lift04'
Q = B / 'round62_v239_unload'
P = B / 'offline_v239_unload'
COMPILER = 'roboassemblybench/core/beam_physical_compiler.py'
SUPPORT = 'toolkits/factory_dual_franka_assembly/beam_support_state.py'
STATIC = ['asset_manifest.json', 'capture_output_contract.json', 'close_geometry_asset_manifest.json',
          'geometry_reference_manifest.json', 'human_final_acceptance_approval.json', 'review_workflow.py',
          'scene_asset_signatures.json', 'human_instruction_round56.json']
PREFLIGHT_OLD = "BEAM_HANDOVER_UNLOAD_LIFT_M='0.0004',"
PREFLIGHT_NEW = "BEAM_HANDOVER_UNLOAD_LIFT_M='0.0002',"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    assert json.loads((P / 'config_check.json').read_text())['passed']
    assert not Q.exists()
    before = sha(P61 / 'source' / COMPILER)
    assert sha(P / 'compiler_before62.py') == before
    for name in ('gate_check.json', 'velocity_cache_ab.json'):
        data = json.loads((P61 / name).read_text())
        assert data['passed']
        shutil.copy2(P61 / name, P / name)
    assert sha(C / SUPPORT) == sha(P61 / 'source' / SUPPORT) == sha(P / 'source' / SUPPORT)
    (Q / 'tools').mkdir(parents=True)
    for name in STATIC:
        shutil.copy2(Q61 / name, Q / name)
    for tool in sorted((Q61 / 'tools').glob('*.py')):
        shutil.copy2(tool, Q / 'tools' / tool.name)
    pre = Q / 'tools/preflight.py'
    text = pre.read_text()
    assert text.count(PREFLIGHT_OLD) == 1
    pre.write_text(text.replace(PREFLIGHT_OLD, PREFLIGHT_NEW))
    target, candidate = C / COMPILER, P / 'source' / COMPILER
    after = sha(candidate)
    if sha(target) == before:
        shutil.copy2(candidate, target)
    assert sha(target) == after
    manifest = json.loads((Q61 / 'source_manifest.json').read_text())
    manifest['profile'] = 'native_v239_unload_candidate'
    updated = 0
    for item in manifest['files']:
        if item['path'] == COMPILER:
            assert item['sha256'] == before
            item['sha256'] = after
            updated += 1
    assert updated == 1
    for item in manifest['files']:
        assert sha(C / item['path']) == item['sha256'], item['path']
    (Q / 'source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(dict(staged=str(Q), files=len(manifest['files']),
                          source_manifest_sha256=sha(Q / 'source_manifest.json'),
                          compiler_before=before, compiler_sha256=after, support_state_sha256=sha(C / SUPPORT),
                          worker_sha256=sha(Q / 'tools/worker.py'), preflight_sha256=sha(pre))))


if __name__ == '__main__':
    main()
