"""Deployment guards for the isolated historical Stool reference; no simulator."""

import importlib.util
import json
import tarfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('portable_stool_test', ROOT / 'scripts/stool_assisted/run.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def test_full_archives_match_original_manifest(tmp_path):
    binding = json.loads((entry.ARTIFACTS / 'source_binding.json').read_text())
    for kind, manifest in [('source', 'source_manifest'), ('tools', 'tools_manifest')]:
        entry.extract_exact(entry.ARTIFACTS / ('S24_' + kind + '_snapshot.tar.gz'), tmp_path / kind, binding[manifest])
        assert len(binding[manifest]) == (914 if kind == 'source' else 41)


def test_extraction_rejects_path_escape(tmp_path):
    archive = tmp_path / 'bad.tar.gz'
    with tarfile.open(archive, 'w:gz') as stream:
        member = tarfile.TarInfo('../escape')
        stream.addfile(member)
    with pytest.raises(ValueError, match='relative ordinary'):
        entry.extract_exact(archive, tmp_path / 'target', {'../escape': ''})
    assert not (tmp_path / 'escape').exists()


def test_frozen_metadata_cannot_be_overwritten(tmp_path):
    origin, target = tmp_path / 'assets', tmp_path / 'reference'
    origin.mkdir()
    target.mkdir()
    (origin / 'metadata.json').write_text('new')
    (target / 'metadata.json').write_text('frozen')
    with pytest.raises(ValueError, match='metadata differs'):
        entry.link_missing_assets(origin, target)
    assert (target / 'metadata.json').read_text() == 'frozen'


def test_prepare_does_not_reuse_other_source_tree(tmp_path, monkeypatch):
    monkeypatch.setattr(entry, 'RUNTIME', tmp_path / 'runtime with spaces')
    monkeypatch.setattr(entry, 'SOURCE', entry.RUNTIME / 'stool_repository')
    monkeypatch.setattr(entry, 'TOOLS', entry.RUNTIME / 'tools')
    assert entry.prepare()['passed']
    target = entry.SOURCE / 'roboassemblybench/core/fabrica_assisted_compiler.py'
    target.write_text(target.read_text() + '\n# modified\n')
    with pytest.raises(ValueError, match='changed'):
        entry.prepare()


def test_original_arguments_preserve_physics_and_recording_flags(tmp_path):
    args = entry.worker_arguments(tmp_path / 'output with spaces')
    assert args[args.index('--worker-layout-seeds') + 1] == '4906'
    assert args[args.index('--worker-seeds') + 1] == '0'
    assert args[args.index('--live-video-frame-stride') + 1] == '8'
    for flag in (
        '--keep-video-frames',
        '--runtime-constraint-monitor',
        '--assembly-sequence-precheck',
        '--stage-trajectory-precheck',
    ):
        assert flag in args


def test_normalization_only_relocates_and_ignores_top_fingerprint():
    a = {'recipe_fingerprint': 'old', 'p': '/old/assets/file.usd', 'nested': {'recipe_fingerprint': 'kept'}, 'gain': 12}
    b = entry.normalized(a, [('/old', '<SOURCE>')])
    assert b == {'p': '<SOURCE>/assets/file.usd', 'nested': {'recipe_fingerprint': 'kept'}, 'gain': 12}
    assert entry.differences(b, dict(b, gain=13)) == ['/gain']


def startup_fixture(tmp_path):
    stamp = {'epoch': 0, 'index': 3}
    rows = []
    for name in ('front', 'left_wrist', 'right_wrist'):
        path = tmp_path / (name + '.png')
        path.write_bytes(name.encode())
        rows.append({'camera': name, 'path': str(path), 'sha256': entry.digest(path)})
    entry.save(tmp_path / 'startup_images_ready.json', {'physics_stamp': stamp, 'images': rows})
    entry.save(tmp_path / 'startup_first_production_action.json', {'passed': True, 'physics_stamp': stamp})
    return rows


def test_approval_binds_three_images_and_cannot_be_overwritten(tmp_path):
    startup_fixture(tmp_path)
    entry.approve(tmp_path)
    assert json.loads((tmp_path / 'startup_image_review.json').read_text())['accepted']
    with pytest.raises(FileExistsError):
        entry.approve(tmp_path)


def test_approval_rejects_changed_image_or_clock(tmp_path):
    rows = startup_fixture(tmp_path)
    Path(rows[0]['path']).write_bytes(b'changed')
    with pytest.raises(ValueError, match='image changed'):
        entry.approve(tmp_path)
    startup_fixture(tmp_path)
    entry.save(tmp_path / 'startup_first_production_action.json', {'passed': True, 'physics_stamp': {'index': 4}})
    with pytest.raises(ValueError, match='same physical state'):
        entry.approve(tmp_path)
