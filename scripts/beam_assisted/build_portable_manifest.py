#!/usr/bin/env python3
"""Maintainer-only release binding; never changes original v35 archives/manifests."""
from pathlib import Path
import hashlib
import json
import re
import shlex

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'outputs/beam_assisted_20261015/tools'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def main():
    base = json.loads((TOOLS / 'assisted_manifest.json').read_text())
    runtime = {}
    for name, expected in base.items():
        if name.startswith(('tests/', 'outputs/')) or Path(name).name.startswith('._'):
            continue
        if Path(name).suffix not in ('.py', '.yaml', '.yml', '.json', '.urdf', '.srdf'):
            continue
        file = ROOT / name
        if sha(file) != expected:
            raise ValueError('Original v35 runtime source changed: ' + name)
        runtime[name] = expected
    helpers = ['assisted_worker.py', 'export_geometry.py', 'visual_mesh_check.py',
               'visual_support_check.py', 'camera_visibility_preflight.py',
               'publish_runtime_control.py', 'bounded_scene_http_probe.py']
    for name in helpers:
        file = TOOLS / name
        runtime[file.relative_to(ROOT).as_posix()] = sha(file)
    for file in [ROOT / 'scripts/beam_assisted/run.py'] + list((ROOT / 'scripts/beam_assisted/runtime').glob('*.py')):
        runtime[file.relative_to(ROOT).as_posix()] = sha(file)
    for name in ['roboassemblybench/scripts/run_beam_assisted.sh', 'requirements/beam_assisted.txt']:
        runtime[name] = sha(ROOT / name)
    assets = {}
    extra = json.loads((ROOT / 'artifacts/beam_assisted/additional_asset_copy_manifest.json').read_text())
    for row in extra['files']:
        name = row['repository_target']
        if sha(ROOT / name) != row['sha256']:
            raise ValueError('Published Beam asset changed: ' + name)
        assets[name] = row['sha256']
    robot = ROOT / 'roboassemblybench/assets/Fabrica/fabrica_ur5e_cooling_optical_board_black_fullbundle_sdf001'
    for file in robot.rglob('*'):
        if file.is_file():
            assets[file.relative_to(ROOT).as_posix()] = sha(file)
    reference = json.loads((ROOT / 'outputs/beam_assisted_20261015/D28_work/A44_media/task_config_snapshot.json').read_text())['task_configs'][0]
    old_repo = '/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015/assisted_repository'
    old_shared = '/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository'
    old_isaac = '/home/user/isaacsim-5.1.0'
    roots = [(old_repo, '<REPO>'), (old_shared, '<REPO>'), (old_isaac, '<ISAAC>')]
    def normalize(x, top=True):
        if isinstance(x, dict):
            return {k: normalize(v, False) for k, v in x.items()
                    if not (top and k == 'recipe_fingerprint')}
        if isinstance(x, list):
            return [normalize(v, False) for v in x]
        if isinstance(x, str):
            for prefix, token in roots:
                if x == prefix or x.startswith(prefix + '/'):
                    return token + x[len(prefix):]
        return x
    contract = dict(schema='beam-assisted-reference-physical-config-v1', reference_run='A44_beam_full',
        source_version='v35', configuration=normalize(reference),
        excluded_derived_metadata=['recipe_fingerprint'],
        scope='Every other task field is compared after declared repository/Isaac path relocation.')
    target = ROOT / 'artifacts/beam_assisted/portable_reference_config.json'
    target.write_text(json.dumps(contract, ensure_ascii=False, indent=2) + '\n')
    runtime[target.relative_to(ROOT).as_posix()] = sha(target)
    environment = {}
    traces = {}
    launcher = (TOOLS / 'run_beam_route.sh').read_text().split('cd "$C"', 1)[1]
    for line in launcher.splitlines():
        line = line.strip()
        if not line.startswith('export '):
            continue
        for entry in shlex.split(line[7:]):
            if '=' not in entry:
                continue
            key, value = entry.split('=', 1)
            if value.startswith('$OUT/'):
                traces[key] = value[5:]
            elif key.startswith('BEAM_') and '$' not in value:
                environment[key] = value
    original_gate = json.loads((ROOT / 'outputs/beam_assisted_20261015/D28_work/A44_media/launch_gate.json').read_text())['gate']
    deps = json.loads((ROOT / 'artifacts/beam_assisted/runtime_dependencies.json').read_text())
    isaac_files = []
    # Only files actually named by the reference task are required, not an entire SDK installation.
    def paths(x):
        if isinstance(x, dict):
            for v in x.values():
                yield from paths(v)
        elif isinstance(x, list):
            for v in x:
                yield from paths(v)
        elif isinstance(x, str) and x.startswith(old_isaac + '/'):
            yield x
    referenced = set(paths(reference))
    rows = deps['isaac_and_fallback_files']
    rows = rows.values() if isinstance(rows, dict) else rows
    for row in rows:
        if isinstance(row, dict) and row.get('path') in referenced and row.get('sha256'):
            isaac_files.append(dict(path=row['path'][len(old_isaac)+1:], sha256=row['sha256']))
    manifest = dict(schema='beam-assisted-portable-release-v1', version='1.0.0',
        physics_source_version='v35', reference_run='A44_beam_full', supported_runtime='Linux NVIDIA / Python3.11 / Isaac Sim5.1.0',
        runtime_files=runtime, assets=assets, isaac_files=isaac_files,
        reference_environment=environment, trace_files=traces,
        primary_scene={k: original_gate['bounded_primary_scene_probe'][k] for k in ['url', 'sha256']},
        prior_snapshot_manifest_sha256=sha(ROOT / 'artifacts/beam_assisted/publication_manifest.json'),
        scope='Portable bootstrap adds no grasp/control/physics changes; original archives and seals remain immutable.')
    published = json.loads((ROOT / 'artifacts/beam_assisted/publication_manifest.json').read_text())
    by_path = {r['path']: r for r in published['files']}
    allowed = ['.gitignore', 'README.md', 'BEAM_ASSISTED_PR_HANDOFF.md',
               'docs/beam_assisted/DEPLOYMENT.md', 'scripts/beam_assisted/verify_publication.py']
    manifest['publication_overrides'] = {}
    for name in allowed:
        file = ROOT / name
        if name in by_path and sha(file) != by_path[name]['sha256']:
            manifest['publication_overrides'][name] = dict(original_sha256=by_path[name]['sha256'],
                sha256=sha(file), bytes=file.stat().st_size, reason='Portable release navigation/integrity tool; original bytes remain in v35 snapshot')
    manifest['documentation_files'] = {name: sha(ROOT / name) for name in
        ['README.md', 'BEAM_REPAIR_README.md', 'BEAM_ASSISTED_PR_HANDOFF.md',
         'docs/beam_assisted/RUN.md', 'docs/beam_assisted/DEPLOYMENT.md'] if (ROOT / name).is_file()}
    file = ROOT / 'artifacts/beam_assisted/portable_release.json'
    file.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(runtime_files=len(runtime), assets=len(assets), isaac_files=isaac_files,
                         traces=len(traces), manifest=str(file))))


if __name__ == '__main__':
    main()
