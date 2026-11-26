"""Engine-free check of the v239 compiler candidate (v238 with completion tolerance max(lift/2, 0.4 mm)).

  1. BEAM_HANDOVER_UNLOAD_LIFT_M unset or 0.001 -> byte-identical to the round-60 production config;
  2. =0.0004 -> round-61 config except the lift tolerance 0.0002 -> 0.0004;
  3. =0.0002 (round 62) -> round-61 config except the lift offset 0.0004 -> 0.0002 and tolerance -> 0.0004;
  4. out-of-range or non-numeric values rejected; phase-contract review clean; final verification unchanged.
"""
import copy
import hashlib
import importlib.util
import json
import os
import sys
from pathlib import Path

R = Path('/home/user/beam_integration_20261010_upstream6f57dae')
C = R / 'candidate_repository'
B = R / 'grasp_repairs_20261012_20rounds'
Q = B / 'round60_v237_column2'
Q61 = B / 'round61_v238_lift04'
HERE = Path(__file__).resolve().parent
CANDIDATE = HERE / 'source/roboassemblybench/core/beam_physical_compiler.py'
sys.path[:0] = [str(C)]

os.environ.update(BEAM_PHYSICAL_ACCEPTANCE_PROFILE='plumbers_staged_v2',
    BEAM_EXPLORE_POST_SEAT='0', BEAM_EXPLORE_CONTINUE='0', BEAM_IDLE_HOLD_ANCHOR='1',
    BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE='1', BEAM_COLUMN3_TRANSPORT_RAISE_M='.16',
    BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N='16,24', BEAM_COLUMN3_PICKUP_DEPTH_M='-.004',
    BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S='.20', BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N='16,24',
    BEAM_EMPTY_RETREAT_LOCAL_IK='1', BEAM_COLUMN2_SIDE_CLEARANCE_ROUTE='1',
    BEAM_COMMON_POSE_WINDOW='1', BEAM_FINAL_POSE_ACCEPTANCE='1', BEAM_CAP0_PICKUP_DEPTH_M='-0.0015',
    BEAM_PART0_WINDOW_SPEEDUP='1', BEAM_FAST_MOTION='1', BEAM_FINAL_ASSEMBLY_LOAD='1',
    BEAM_PART0_RELEASE_AFTER_CLEAR='1', BEAM_RELEASE_WITHOUT_STACK_WINDOW='1', BEAM_HANDOVER_UNLOAD_LIFT='1')

import roboassemblybench.core as core_pkg  # noqa: E402

spec = importlib.util.spec_from_file_location('roboassemblybench.core.beam_physical_compiler', CANDIDATE)
module = importlib.util.module_from_spec(spec)
sys.modules['roboassemblybench.core.beam_physical_compiler'] = module
spec.loader.exec_module(module)
core_pkg.beam_physical_compiler = module

from toolkits.factory_dual_franka_assembly import generate_demos as demo  # noqa: E402

FLAG = 'BEAM_HANDOVER_UNLOAD_LIFT_M'
LIFT = 'handover_left_2_unload_lift'


def build(value):
    if value is None:
        os.environ.pop(FLAG, None)
    else:
        os.environ[FLAG] = value
    configs = demo._build_collect_worker_task_configs(recipes=['fabrica_beam_ur5e_staged'], seeds=[0],
        layout_seeds=[505], scene_profile='taoyuan_grscenes_tabletop', attach_runtime_cameras=True,
        domain_randomization_enabled=True, control_fps=240)
    assert len(configs) == 1
    return configs[0].model_dump()


def raw(cfg):
    return (json.dumps(cfg, indent=2) + '\n').encode()


def main():
    report = {}
    reference = (Q / 'production_worker_config.json').read_bytes()
    r61 = json.loads((Q61 / 'production_worker_config.json').read_text())
    base = build(None)
    report['unset_matches_round60_config'] = raw(base) == reference
    report['default_value_matches_round60_config'] = raw(build('0.001')) == reference
    names = [p['name'] for p in base['phase_specs']]
    i = names.index(LIFT)
    c04 = build('0.0004')
    exp04 = copy.deepcopy(r61)
    exp04['phase_specs'][i]['local_skill']['position_tolerance'] = 0.0004
    exp04['recipe_fingerprint'] = c04.get('recipe_fingerprint')
    report['lift04_is_round61_with_tolerance_0p4'] = json.dumps(exp04, sort_keys=True) == json.dumps(c04, sort_keys=True)
    cand = build('0.0002')
    cand_raw = raw(cand)
    report['candidate_config_sha256'] = hashlib.sha256(cand_raw).hexdigest()
    report['candidate_recipe_fingerprint'] = cand.get('recipe_fingerprint')
    (HERE / 'production_worker_config.json').write_bytes(cand_raw)
    expected = copy.deepcopy(r61)
    expected['phase_specs'][i]['local_skill'].update(offset=[0.0, 0.0, 0.0002], position_tolerance=0.0004)
    expected['recipe_fingerprint'] = cand.get('recipe_fingerprint')
    report['only_intended_change_vs_round61'] = json.dumps(expected, sort_keys=True) == json.dumps(cand, sort_keys=True)
    report['lift_skill_after'] = {k: cand['phase_specs'][i]['local_skill'][k]
                                  for k in ('offset', 'position_tolerance', 'cartesian_position_step')}
    report['max_steps'] = [r61['max_steps'], cand['max_steps']]
    report['final_verification_unchanged'] = cand['phase_specs'][-1] == base['phase_specs'][-1]
    rejected = {}
    for value in ('0.002', '0.0001', '0', 'x', 'nan'):
        try:
            build(value)
            rejected[value] = False
        except ValueError:
            rejected[value] = True
    report['invalid_value_rejected'] = rejected
    spec_audit = importlib.util.spec_from_file_location('beam_workflow_audit', Q / 'review_workflow.py')
    audit = importlib.util.module_from_spec(spec_audit)
    spec_audit.loader.exec_module(audit)
    review = audit.review(cand)
    report['phase_contract_issues'] = review['issues']
    report['passed'] = bool(report['unset_matches_round60_config'] and report['default_value_matches_round60_config']
                            and report['lift04_is_round61_with_tolerance_0p4']
                            and report['only_intended_change_vs_round61'] and report['final_verification_unchanged']
                            and all(rejected.values()) and not review['issues'])
    (HERE / 'config_check.json').write_text(json.dumps(report, indent=2, default=str) + '\n')
    print(json.dumps(report, indent=1, default=str))


if __name__ == '__main__':
    main()
