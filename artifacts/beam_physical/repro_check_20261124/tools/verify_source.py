from pathlib import Path
import hashlib
import json
import sys

ROOT = Path('/home/user/beam_integration_20261010_upstream6f57dae')
SOURCE = ROOT/'candidate_repository'
MANIFEST = Path(__file__).resolve().parent.parent/'source_manifest.json'
data = json.loads(MANIFEST.read_text())
bad = []
for item in data['files']:
    path = SOURCE/item['path']
    actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    if actual != item['sha256']:
        bad.append(dict(path=item['path'],expected=item['sha256'],actual=actual))
result = dict(passed=not bad,source=str(SOURCE),manifest=str(MANIFEST),
              manifest_sha256=hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
              files=len(data['files']),differences=bad)
Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
sys.exit(1 if bad else 0)
