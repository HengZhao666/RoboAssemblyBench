"""Reusable offline, source-bound affected-module check. Never creates physics."""
from pathlib import Path
import argparse, hashlib, inspect, json, os, sys, time


def digest(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('work', type=Path)
    ap.add_argument('--broad', action='store_true')
    ap.add_argument('--modules', help='Comma-separated existing toolkit test modules for the changed dependency domain')
    a = ap.parse_args(); w = a.work.resolve(); c = w/'candidate'
    target = w/'affected_module_result.json'
    if target.exists():
        raise ValueError('Keep previous results; use a distinct qualification work directory')
    baseline = json.loads((w/'baseline_manifest.json').read_text())
    files = {n:digest(c/n) for n in baseline}
    changed = [n for n in baseline if baseline[n] != files[n]]
    sys.path.insert(0, str(c)); os.chdir(c)
    # Recorded module-test environment; actual production layouts are checked
    # separately with their production route settings, never inferred here.
    os.environ['BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE'] = '0'
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    assert Path(inspect.getfile(UR5eAssemblyAtomicSkillAdapter)).resolve().is_relative_to(c)
    import pytest
    class Counts:
        def pytest_sessionfinish(self, session, exitstatus):
            self.collected = session.testscollected
            stats = session.config.pluginmanager.get_plugin('terminalreporter').stats
            self.counts = {k:len(v) for k,v in stats.items() if k in ('passed','failed','error','skipped','deselected')}
    names=['test_beam_assisted_object_servo.py']
    if a.broad:
        names=['test_beam_assisted_route.py','test_beam_assisted_object_servo.py','test_beam_assisted_preflight.py','test_beam_assisted_release_state.py','test_beam_assisted_camera_contract.py','test_beam_coordinated.py','test_beam_canonical_integration.py','test_beam_idle_hold.py','test_beam_release_joint_branch.py','test_beam_side_clearance.py','test_beam_physics_diagnostics.py','test_beam_physics_contract.py','test_beam_runtime_integrity.py','constraint_checking/test_entity_geometry.py','test_beam_failure_video.py']
        for n in ['BEAM_EXPLORE_POST_SEAT','BEAM_EXPLORE_CONTINUE','BEAM_COLUMN3_PICKUP_DEPTH_M','BEAM_COLUMN3_TRANSPORT_RAISE_M','BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N','BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S','BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N']:os.environ.pop(n,None)
    if a.modules:
        if a.broad:
            raise ValueError('Choose the broad domain or explicit affected modules')
        names = a.modules.split(',')
        if (not names or len(names) != len(set(names))
                or any(Path(n).is_absolute() or '..' in Path(n).parts
                       or not n.endswith('.py') or not (c/'tests/toolkits'/n).is_file() for n in names)):
            raise ValueError('Affected tests must be unique existing toolkit modules')
    counts = Counts(); started = time.monotonic()
    code = pytest.main(['--confcutdir=tests/toolkits', *['tests/toolkits/'+n for n in names],
                       '-q', '-p', 'no:cacheprovider', '--tb=short'], plugins=[counts])
    assert all(digest(c/n) == h for n,h in files.items())
    report = dict(passed=code == 0, returncode=int(code), collected=counts.collected,
        counts=counts.counts, elapsed_s=time.monotonic()-started, native_physics=False,
        candidate=str(c), test_files=names, broad_numerical_change_review=bool(a.broad), baseline_manifest_sha256=digest(w/'baseline_manifest.json'),
        source_files=len(files), changed_files=changed, candidate_files=files,
        test_environment={'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'0'},
        tool_sha256=digest(__file__),
        scope='Affected module, existing protection tests and newly added continuity counterexamples. No dynamic qualification.')
    with target.open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps({k:report[k] for k in ['passed','counts','collected','elapsed_s','changed_files']}))
    return code


if __name__ == '__main__':
    raise SystemExit(main())
