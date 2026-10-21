"""One transfer for six coordinated documents and selected workflow metadata."""
from pathlib import Path
import hashlib
import io
import json
import sys
import tarfile

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
P = R / 'beam_assisted_20261015'
DOCS = {'AGENTS.md', 'BEAM_REPAIR_HANDOFF.md', 'BEAM_REVIEW_CHECKLIST.md',
        'BEAM_FINDINGS_INDEX.md', 'BEAM_ASSISTED_HANDOFF.md', 'BEAM_WORKFLOW_LOG.md'}
sha = lambda value: hashlib.sha256(value).hexdigest()


def main():
    rows = []
    found = set()
    with tarfile.open(fileobj=io.BytesIO(sys.stdin.buffer.read()), mode='r:gz') as archive:
        for member in archive.getmembers():
            name = Path(member.name)
            if not member.isfile() or name.is_absolute() or '..' in name.parts:
                raise ValueError('Unsafe member')
            data = archive.extractfile(member).read()
            if len(name.parts) == 1 and str(name) in DOCS:
                found.add(str(name))
                targets = [root / name for root in (R, R / 'candidate_repository',
                           Path('/home/user/RoboAssemblyBench'), P / 'assisted_repository')]
            else:
                if not (len(name.parts) == 1 or name.parts[0] in
                        {'tools', 'batch_D_20261016', 'coordination_history', 'A16_live_keyframes'}):
                    raise ValueError('Unsupported workflow destination: ' + str(name))
                targets = [P / name]
            for target in targets:
                target.parent.mkdir(parents=True, exist_ok=True)
                if not target.parent.resolve().is_relative_to(R.resolve()) and not str(target).startswith('/home/user/RoboAssemblyBench/'):
                    raise ValueError('Destination escapes approved roots')
                if target.is_symlink():
                    target.unlink()
                target.write_bytes(data)
                if sha(target.read_bytes()) != sha(data):
                    raise ValueError('Synchronization hash mismatch')
                rows.append(dict(path=str(target), sha256=sha(data), bytes=len(data)))
    if found != DOCS:
        raise ValueError('All six coordinated documents are required')
    report = dict(passed=True, coordinated_document_copies=24, files=rows,
                  native_started=False, scope='File synchronization only; no source deployment or native execution')
    (P / 'latest_workflow_sync.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
