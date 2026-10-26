"""Deployment regressions: no Isaac, physics, or GPU is initialized here."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('portable_beam', ROOT / 'scripts/beam_assisted/run.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class DeploymentTests(unittest.TestCase):
    def test_only_derived_top_level_fingerprint_is_ignored(self):
        roots = [('/new checkout', '<REPO>')]
        x = {'recipe_fingerprint': 'new', 'asset': '/new checkout/assets/a', 'nested': {'recipe_fingerprint': 'keep'}}
        self.assertEqual(runner.normalized(x, roots),
            {'asset': '<REPO>/assets/a', 'nested': {'recipe_fingerprint': 'keep'}})

    def test_physical_changes_remain_visible(self):
        original = {'phases': [{'position_tolerance': .00015}], 'recipe_fingerprint': 'a'}
        changed = {'phases': [{'position_tolerance': .00016}], 'recipe_fingerprint': 'b'}
        self.assertEqual(runner.differences(runner.normalized(original, []), runner.normalized(changed, [])),
                         ['/phases/0/position_tolerance'])

    def test_legacy_asset_path_does_not_get_silently_rebased(self):
        self.assertNotEqual(runner.normalized('/old/server/assets/a', [('/new/checkout', '<REPO>')]),
                            '<REPO>/assets/a')

    def test_documentation_does_not_invalidate_runtime_binding(self):
        release = {k: {} for k in ('runtime_files', 'assets', 'isaac_files', 'reference_environment',
                                  'trace_files', 'primary_scene', 'physics_source_version')}
        before = runner.runtime_identity(release)
        release['documentation_files'] = {'README.md': 'changed'}
        self.assertEqual(before, runner.runtime_identity(release))
        release['runtime_files']['controller.py'] = 'changed'
        self.assertNotEqual(before, runner.runtime_identity(release))

    def test_existing_output_is_refused_before_network_or_simulation(self):
        with tempfile.TemporaryDirectory() as tmp:
            from types import SimpleNamespace
            with self.assertRaisesRegex(ValueError, 'Output already exists'):
                runner.launch(SimpleNamespace(output_dir=Path(tmp)), {})

    def test_exit_zero_does_not_replace_the_required_geometry_report(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)
            release = {k: {} for k in ('runtime_files', 'assets', 'isaac_files', 'reference_environment',
                                      'trace_files', 'primary_scene', 'physics_source_version')}
            (out / 'runtime_release.json').write_text(json.dumps(release))
            (out / 'collect_results.json').write_text(json.dumps([dict(success=True, assisted_success=True,
                assisted_final_qualification={'current_native_recheck': {'ready': True}})]))
            checked = {'runtime_binding_sha256': runner.runtime_identity(release)}
            self.assertFalse(runner.finish(out, 0, 0, checked))
            (out / 'visible_mesh_runtime_report.json').write_text(json.dumps(dict(
                samples=1, finished_marker_seen=True, failures_above_precision=[], stop_requested=False)))
            self.assertTrue(runner.finish(out, 0, 0, checked))

    def test_real_worker_arguments_keep_reference_route_and_guards(self):
        args = runner.worker_arguments(Path('/tmp/output with spaces'))
        self.assertEqual(args[args.index('--worker-recipe') + 1], 'fabrica_beam_ur5e_assisted')
        self.assertEqual(args[args.index('--worker-layout-seeds') + 1], '505')
        for flag in ['--runtime-constraint-monitor', '--assembly-sequence-precheck',
                     '--stage-trajectory-precheck', '--record-live-video']:
            self.assertIn(flag, args)
        self.assertNotIn('--keep-video-frames', args)
        self.assertIn('--keep-video-frames', runner.worker_arguments(Path('/tmp/out'), True))

    def test_helper_geometry_class_is_unchanged(self):
        old = ast.parse((ROOT / 'outputs/beam_assisted_20261015/tools/sweep_model.py').read_text())
        new = ast.parse((ROOT / 'scripts/beam_assisted/runtime/sweep_model.py').read_text())
        classes = lambda tree: [ast.dump(n, include_attributes=False) for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
        self.assertEqual(classes(old), classes(new))

    def test_independent_monitor_native_loop_is_unchanged(self):
        old = ast.parse((ROOT / 'outputs/beam_assisted_20261015/tools/watch_visible_meshes.py').read_text())
        new = ast.parse((ROOT / 'scripts/beam_assisted/runtime/watch_visible_meshes.py').read_text())
        loops = lambda tree: [ast.dump(n, include_attributes=False) for n in tree.body if isinstance(n, ast.With)]
        self.assertEqual(loops(old), loops(new))

    def test_release_rejects_missing_or_changed_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp);file = root / 'control.py';file.write_text('original')
            manifest = root / 'release.json'
            manifest.write_text(json.dumps({'runtime_files': {'control.py': runner.digest(file)}, 'assets': {}}))
            with patch.object(runner, 'ROOT', root), patch.object(runner, 'RELEASE', manifest):
                runner.verify_release()
                file.write_text('modified')
                with self.assertRaisesRegex(ValueError, 'Missing/changed'):
                    runner.verify_release()

    def test_release_rejects_assets_symlinked_outside_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / 'checkout';root.mkdir();outside = Path(tmp) / 'shared';outside.write_text('asset')
            (root / 'asset').symlink_to(outside)
            manifest = root / 'release.json';manifest.write_text(json.dumps({'runtime_files': {}, 'assets': {'asset': runner.digest(outside)}}))
            with patch.object(runner, 'ROOT', root), patch.object(runner, 'RELEASE', manifest):
                with self.assertRaisesRegex(ValueError, 'Missing/changed'):
                    runner.verify_release()

    def test_help_works_outside_checkout_without_isaac(self):
        env = dict(os.environ, BEAM_PYTHON=sys.executable)
        env.pop('ISAAC_SIM_ROOT', None)
        with tempfile.TemporaryDirectory(prefix='beam elsewhere ') as tmp:
            result = subprocess.run(['bash', str(ROOT / 'roboassemblybench/scripts/run_beam_assisted.sh'), '--help'],
                                    cwd=tmp, env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--approve-startup', result.stdout)

    def test_shell_reports_missing_isaac_before_running(self):
        env = dict(os.environ, BEAM_PYTHON=sys.executable, ISAAC_SIM_ROOT='/missing/isaac', ISAAC_PATH='')
        result = subprocess.run(['bash', str(ROOT / 'roboassemblybench/scripts/run_beam_assisted.sh'), '--check'],
                                env=env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn('ISAAC_SIM_ROOT', result.stderr)


if __name__ == '__main__':
    unittest.main()
