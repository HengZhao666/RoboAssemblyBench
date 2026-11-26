#!/usr/bin/env python3
"""Rebuild the exact source trees of the two verified pure-physical assembly runs.

  beam   Beam v239: the 900-file source tree of the verified run, plus the Beam
         Fabrica assets it loaded.
  stool  Stool circular P09: Plumbers integration commit 6f1de1d + delta P09, all
         tracked files of the verified run.

Files that are byte-identical on this repository's main branch are copied from the
repository checkout; the rest come from artifacts/<route>_physical/source_overlay_*.tar.gz.
A few such files (old notes) were later removed from main; they are read from the pinned
commit SNAPSHOT_COMMIT instead, so a git clone (not a bare archive download) is needed.
Every file is checked against the recorded SHA-256 before the tree is reported ready.

Usage (run from anywhere inside the checkout, Python >= 3.8, stdlib only):
  python3 roboassemblybench/scripts/prepare_physical_reference.py beam
  python3 roboassemblybench/scripts/prepare_physical_reference.py stool --dest /path/to/tree
  python3 roboassemblybench/scripts/prepare_physical_reference.py beam --verify-only

The default destination is outputs/<route>_physical_reference/ (git-ignored outputs).
Nothing here starts a simulation.
"""
import argparse
import subprocess
import hashlib
import json
import os
import shutil
import sys
import tarfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ART = ROOT / 'artifacts'
# Commit at which every file the manifests take from main was present.
SNAPSHOT_COMMIT = '34d4333abf4760a3c1842378a973bec3145106cf'

ROUTES = {
    'beam': {
        'overlay': ART / 'beam_physical/source_overlay_v239.tar.gz',
        'manifest': ART / 'beam_physical/launch_round62/source_manifest.json',
        'asset_manifest': ART / 'beam_physical/launch_round62/asset_manifest.json',
        'recorded_root': '/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository/',
        'asset_dir': 'roboassemblybench/assets',
    },
    'stool': {
        'overlay': ART / 'stool_physical/source_overlay_P09.tar.gz',
        'manifest': ART / 'stool_physical/source_manifest.json',
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def load_expected(route):
    """Return ({relative path: sha256}, {relative path: symlink target})."""
    cfg = ROUTES[route]
    data = json.loads(cfg['manifest'].read_text())
    links = {}
    if route == 'beam':
        files = {item['path']: item['sha256'] for item in data['files']}
        prefix = cfg['recorded_root']
        for section in ('files', 'negative_control_assets'):
            for item in json.loads(cfg['asset_manifest'].read_text()).get(section, []):
                assert item['path'].startswith(prefix), item['path']
                files[item['path'][len(prefix):]] = item['sha256']
    else:
        files = {path: item['sha256'] for path, item in data['files'].items()}
        links = data['symlinks']
    return files, links


def safe_member(member, dest):
    target = (dest / member.name).resolve()
    if not str(target).startswith(str(dest.resolve()) + os.sep):
        raise SystemExit(f'unsafe path in overlay: {member.name}')


def place(src, dst):
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        dst.unlink()
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def from_history(rel, out):
    try:
        data = subprocess.run(['git', '-C', str(ROOT), 'cat-file', 'blob', f'{SNAPSHOT_COMMIT}:{rel}'],
                              capture_output=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        raise SystemExit(f'{rel}: not in this checkout and not readable from commit {SNAPSHOT_COMMIT}; '
                         'use a full git clone of the repository')
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists() or out.is_symlink():
        out.unlink()
    out.write_bytes(data)


def fill_with_links(src_dir, dst_dir):
    dst_dir.mkdir(parents=True, exist_ok=True)
    for entry in sorted(src_dir.iterdir()):
        out = dst_dir / entry.name
        if not out.exists() and not out.is_symlink():
            os.symlink(entry, out)
        elif entry.is_dir() and out.is_dir() and not out.is_symlink():
            fill_with_links(entry, out)


def prepare(route, dest):
    cfg = ROUTES[route]
    files, links = load_expected(route)
    dest.mkdir(parents=True, exist_ok=True)
    from_overlay = set()
    with tarfile.open(cfg['overlay']) as archive:
        for member in archive.getmembers():
            safe_member(member, dest)
            if member.issym():
                continue  # recreated from the manifest below
            if not member.isfile():
                continue
            out = dest / member.name
            out.parent.mkdir(parents=True, exist_ok=True)
            if out.exists() or out.is_symlink():
                out.unlink()
            with archive.extractfile(member) as src, open(out, 'wb') as handle:
                shutil.copyfileobj(src, handle)
            os.chmod(out, member.mode & 0o777)
            from_overlay.add(member.name)
    for rel in files:
        if rel in from_overlay:
            continue
        src = ROOT / rel
        if src.is_file():
            place(src, dest / rel)
        else:
            from_history(rel, dest / rel)
    if route == 'beam':
        # The run loads the Beam bundle (and its scene/fixture files) from
        # roboassemblybench/assets; link every asset entry of this checkout that the
        # tree does not already hold, so it sees the same asset layout as the verified run.
        fill_with_links(ROOT / cfg['asset_dir'], dest / cfg['asset_dir'])
    for rel, target in links.items():
        out = dest / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        if out.is_symlink() or out.exists():
            if out.is_dir() and not out.is_symlink():
                continue
            out.unlink()
        os.symlink(target, out)
    return verify(route, dest)


def verify(route, dest):
    files, links = load_expected(route)
    bad = []
    for rel, expected in sorted(files.items()):
        path = dest / rel
        if not path.is_file():
            bad.append((rel, 'missing'))
        elif sha256(path) != expected:
            bad.append((rel, 'sha256 differs'))
    for rel, target in links.items():
        path = dest / rel
        if not path.is_symlink() or os.readlink(path) != target:
            bad.append((rel, 'symlink differs'))
    report = {'route': route, 'dest': str(dest), 'files_checked': len(files),
              'symlinks_checked': len(links), 'problems': len(bad), 'first_problems': bad[:20]}
    print(json.dumps(report, indent=1))
    return not bad


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('route', choices=sorted(ROUTES))
    parser.add_argument('--dest', type=Path)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    dest = (args.dest or ROOT / 'outputs' / f'{args.route}_physical_reference').resolve()
    ok = verify(args.route, dest) if args.verify_only else prepare(args.route, dest)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
