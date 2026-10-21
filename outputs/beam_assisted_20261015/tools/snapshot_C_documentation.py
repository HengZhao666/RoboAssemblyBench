"""Recheck all coordinated server copies and record their current hashes."""
from pathlib import Path
import hashlib
import json
import sys

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
P = R / 'beam_assisted_20261015'
cycle = sys.argv[1]
assert cycle in {'C01', 'C02', 'C03', 'C04', 'C05'}
names = ['AGENTS.md', 'BEAM_REPAIR_HANDOFF.md', 'BEAM_REVIEW_CHECKLIST.md',
         'BEAM_FINDINGS_INDEX.md', 'BEAM_ASSISTED_HANDOFF.md', 'BEAM_WORKFLOW_LOG.md']
roots = [R, R / 'candidate_repository', Path('/home/user/RoboAssemblyBench'),
         P / 'assisted_repository']
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
rows = []
for name in names:
    expected = sha(R / name)
    for root in roots:
        actual = sha(root / name)
        assert actual == expected, str(root / name)
        rows.append({'path': str(root / name), 'sha256': actual})
batch = json.loads((P / 'repair_batch_C_20261015.json').read_text())
report = {'passed': True, 'cycle': cycle, 'workflow_version': batch['workflow_version'],
          'verified_files': len(rows), 'rows': rows,
          'native_attempts_started': batch['native_attempts_started'],
          'maximum_native_attempts': batch['maximum_native_attempts']}
(P / (cycle + '_workflow_documentation_sync.json')).write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps(report, ensure_ascii=False))
