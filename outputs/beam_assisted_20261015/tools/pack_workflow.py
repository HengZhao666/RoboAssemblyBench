"""Pack one authoritative documentation revision and selected small metadata."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='*')
    args = parser.parse_args()
    p = Path(__file__).resolve().parents[1]
    root = p.parents[1]
    names = ['AGENTS.md', 'BEAM_REPAIR_HANDOFF.md', 'BEAM_REVIEW_CHECKLIST.md',
             'BEAM_FINDINGS_INDEX.md', 'BEAM_ASSISTED_HANDOFF.md', 'BEAM_WORKFLOW_LOG.md']
    expected = {}
    with tarfile.open('/tmp/beam_workflow_sync.tar.gz', 'w:gz') as archive:
        for name in names:
            source = root / name
            expected[name] = hashlib.sha256(source.read_bytes()).hexdigest()
            for target in [root / 'submission/beam_upstream_integration_2026-10-10' / name,
                           p / 'assisted_repository' / name]:
                if target.is_symlink():
                    target.unlink()
                shutil.copy2(source, target)
                assert hashlib.sha256(target.read_bytes()).hexdigest() == expected[name]
            archive.add(source, arcname=name, recursive=False)
        for value in dict.fromkeys(args.files):
            name = Path(value)
            assert not name.is_absolute() and '..' not in name.parts
            archive.add(p / name, arcname=str(name), recursive=False)
    (p / 'documentation_expected_sha256.json').write_text(json.dumps(expected, indent=2) + '\n')
    print(json.dumps(dict(archive='/tmp/beam_workflow_sync.tar.gz', local_copies_verified=12,
                          metadata_files=len(set(args.files)))))


if __name__ == '__main__':
    main()
