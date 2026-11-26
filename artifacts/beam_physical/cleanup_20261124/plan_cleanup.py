"""Plan (and with --execute, perform) the 2026-11-24 Beam pure-physics output cleanup.

Human instruction 2026-11-24: delete archived and unarchived Beam pure-physics outputs; keep the
latest result (round 62, v239_unload) and everything needed to reproduce it. The Beam assisted route
is explicitly excluded ("beam辅助路线的结果先别删"). Rule agreed with the human: inside the scope roots,
delete regular files larger than 2 MiB; keep all smaller files (scripts, configs, reviews, ledger,
authorisations) and any file referenced from the round-62 keep areas or the production code.
Writes a receipt (path, size, mtime) before deleting anything.
"""
import json
import os
import re
import sys
import time
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
OUT = B / 'cleanup_20261124'
LIMIT = 2 * 1024 * 1024

KEEP_DIRS = [B / 'round62_v239_unload', B / 'offline_v239_unload', B / 'batch62_66_20261124',
             C / 'outputs' / 'fabrica_beam_v239_unload', OUT]
SCAN_TEXT = [B / 'round62_v239_unload', B / 'offline_v239_unload', B / 'batch62_66_20261124',
             C / 'roboassemblybench', C / 'internutopia', C / 'internutopia_extension', C / 'toolkits',
             C / 'tests', C / 'fabrica_tasks', C / 'skills', C / 'beam_routes', R / 'runtime_tools']
TEXT_SUFFIX = {'.py', '.sh', '.json', '.md', '.txt', '.yaml', '.yml', '.toml', '.cfg'}


def scope_roots():
    roots = [p for p in sorted((C / 'outputs').glob('fabrica_beam_*'))
             if p.is_dir() and p.name != 'fabrica_beam_v239_unload']
    roots += [p for p in sorted(B.iterdir()) if p.is_dir() and p not in KEEP_DIRS]
    roots += [R / 'runtime_repairs_20261011', R / 'runtime_repairs_20261012']
    return roots


def inside(path, dirs):
    return any(path == d or d in path.parents for d in dirs)


def referenced_paths():
    """Absolute and R-relative paths named in keep-area / production text files."""
    pat_abs = re.compile(r'/home/user/beam_integration_20261010_upstream6f57dae/[^\s"\'`<>),;]+')
    pat_rel = re.compile(r'(?:grasp_repairs_20261012_20rounds|candidate_repository/outputs|runtime_repairs_2026101[12])'
                         r'/[^\s"\'`<>),;]+')
    refs = set()
    for root in SCAN_TEXT:
        if not root.exists():
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            if Path(dirpath) == C / 'roboassemblybench' / 'assets':
                dirnames[:] = []
                continue
            for name in filenames:
                p = Path(dirpath) / name
                if p.suffix not in TEXT_SUFFIX or p.name == 'native_finished.json':
                    continue
                try:
                    if p.stat().st_size > LIMIT:
                        continue
                    text = p.read_text(errors='ignore')
                except OSError:
                    continue
                for m in pat_abs.findall(text):
                    refs.add(Path(m.rstrip('.\'"')))
                for m in pat_rel.findall(text):
                    refs.add(R / m.rstrip('.\'"'))
    return refs


def main():
    execute = '--execute' in sys.argv
    OUT.mkdir(exist_ok=True)
    refs = referenced_paths()
    # Bare roots (R, C, B, C/outputs) would protect everything; they name locations, not dependencies.
    refs -= {R, C, B, C / 'outputs', R / 'grasp_repairs_20261012_20rounds'}
    dir_refs = sorted(str(r) for r in refs if r.suffix == '' and any(r == s or s in r.parents for s in scope_roots()))
    print('directory-like refs inside scope:', len(dir_refs))
    protected, delete = [], []
    for root in scope_roots():
        if not root.exists():
            continue
        for dirpath, _, filenames in os.walk(root):
            for name in filenames:
                p = Path(dirpath) / name
                if p.is_symlink() or not p.is_file():
                    continue
                st = p.stat()
                if st.st_size <= LIMIT or inside(p, KEEP_DIRS):
                    continue
                hit = p in refs  # exact file references only; directory mentions name history, not dependencies
                row = dict(path=str(p), bytes=st.st_size, mtime=int(st.st_mtime), nlink=st.st_nlink)
                (protected if hit else delete).append(row)
    by_root = {}
    for row in delete:
        rel = Path(row['path']).relative_to(R).parts
        key = '/'.join(rel[:3] if rel[0] == 'candidate_repository' else rel[:2])
        by_root[key] = by_root.get(key, 0) + row['bytes']
    summary = dict(instruction='2026-11-24 human: delete archived and unarchived Beam pure-physics outputs, '
                               'keep latest result (round 62) reproducible; assisted route excluded',
                   rule='regular files > 2 MiB inside scope roots, minus keep dirs and exactly referenced files',
                   keep_dirs=[str(d) for d in KEEP_DIRS], scope_roots=[str(r) for r in scope_roots()],
                   reference_count=len(refs), delete_files=len(delete),
                   delete_bytes=sum(r['bytes'] for r in delete), protected=protected,
                   multi_link_files=[r for r in delete if r['nlink'] > 1][:50],
                   by_root=dict(sorted(by_root.items(), key=lambda kv: -kv[1])),
                   planned_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'))
    (OUT / 'plan_summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False) + '\n')
    (OUT / 'delete_receipt.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in delete))
    print(json.dumps({k: v for k, v in summary.items() if k not in ('by_root', 'scope_roots', 'multi_link_files')},
                     indent=1, ensure_ascii=False)[:3000])
    print('multi-link:', len(summary['multi_link_files']))
    for k, v in list(summary['by_root'].items())[:25]:
        print(f'{v / 2**30:7.2f} GiB  {k}')
    if not execute:
        return
    freed, failed = 0, []
    for row in delete:
        try:
            os.remove(row['path'])
            freed += row['bytes']
        except OSError as exc:
            failed.append(dict(path=row['path'], error=str(exc)))
    done = dict(executed_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'), removed_files=len(delete) - len(failed),
                removed_bytes=freed, failed=failed)
    (OUT / 'execution.json').write_text(json.dumps(done, indent=1) + '\n')
    print(json.dumps(done, indent=1)[:2000])


if __name__ == '__main__':
    main()
