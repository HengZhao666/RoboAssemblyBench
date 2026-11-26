"""Build the actual worker configs and inspect the selected physical contract."""
from pathlib import Path
import hashlib
import json
import os

ROOT = Path('/home/user/beam_integration_20261010_upstream6f57dae')
OUT = Path(__file__).resolve().parent.parent
profile=os.environ.get('BEAM_PHYSICAL_ACCEPTANCE_PROFILE','plumbers_staged_v2')
assert profile in {'plumbers_staged_v1','plumbers_staged_v2'},profile
os.environ.update(BEAM_PHYSICAL_ACCEPTANCE_PROFILE=profile,
    BEAM_EXPLORE_POST_SEAT='0', BEAM_EXPLORE_CONTINUE='0', BEAM_IDLE_HOLD_ANCHOR='1',
    BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE='1', BEAM_COLUMN3_TRANSPORT_RAISE_M='.16',
    BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N='16,24', BEAM_COLUMN3_PICKUP_DEPTH_M='-.004',
    BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S='.20', BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N='16,24',
    BEAM_CAP0_PICKUP_DEPTH_M='-0.0015', BEAM_PART0_WINDOW_SPEEDUP='1', BEAM_FAST_MOTION='1', BEAM_FINAL_ASSEMBLY_LOAD='1', BEAM_PART0_RELEASE_AFTER_CLEAR='1', BEAM_RELEASE_WITHOUT_STACK_WINDOW='1', BEAM_HANDOVER_UNLOAD_LIFT='1', BEAM_HANDOVER_UNLOAD_LIFT_M='0.0002', BEAM_COMMON_POSE_WINDOW='1', BEAM_FINAL_POSE_ACCEPTANCE='1',
    BEAM_EMPTY_RETREAT_LOCAL_IK='1', BEAM_COLUMN2_SIDE_CLEARANCE_ROUTE='1')
from toolkits.factory_dual_franka_assembly import generate_demos as demo
from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe

recipe=load_task_recipe('fabrica_beam_ur5e_staged',scene_profile='taoyuan_grscenes_tabletop')
configs=demo._build_collect_worker_task_configs(recipes=['fabrica_beam_ur5e_staged'],seeds=[0],
    layout_seeds=[505],scene_profile='taoyuan_grscenes_tabletop',attach_runtime_cameras=True,
    domain_randomization_enabled=True,control_fps=240)
assert len(configs)==1
cfg=configs[0].model_dump()
(OUT/'production_worker_config.json').write_text(json.dumps(cfg,indent=2)+'\n')
issues=[]
for phase in recipe['phases']:
    for key in ('lock','fixture_lock','kinematic_carry'):
        if phase.get(key):issues.append([phase['name'],key])
    for key in ('object_collisions','object_gravity'):
        for entry in phase.get(key,[]):
            if not entry.get('enabled',True):issues.append([phase['name'],key,entry])
    for entry in phase.get('collision_filters',[]):
        if entry.get('enabled',True):issues.append([phase['name'],'collision_filter',entry])
    for entry in phase.get('attach',[]):
        if entry.get('attachment_mode')!='pure_physical_grasp' or entry.get('disable_collision_on_attach'):
            issues.append([phase['name'],'attachment',entry])
assert not issues,issues
final=recipe['phases'][-1]
assert final['name']=='beam_final_unloaded_verification'
assert final['advance']['physics_stable_steps']==240
assert len([p for p in recipe['phases'] if p['name'].startswith('beam_terminal_home_')])==2
assert all(s['position_tolerance']==.003 and s['orientation_tolerance']==.04
           and s['linear_velocity_threshold']==.01 and s['angular_velocity_threshold']==.1
           and s['require_released'] and s['require_static'] for s in recipe['success'])
contract=recipe['beam_physical_contract']
assert contract['exploration_bypass'] is False
assert contract['profile']==profile,contract
close_phases=[]
for phase in cfg['phase_specs']:
    for skill in [phase.get('local_skill') or {}, *phase.get('local_skills', {}).values()]:
        if skill.get('name')=='ur5e_close_gripper' and skill.get('close_force_regulation'):
            assert skill['close_force_weak_side_preload'] is True
            assert skill['close_force_require_clamp_band'] is True
            if profile=='plumbers_staged_v2':
                assert skill['close_force_recenter_single_contact_full_step'] is True
                assert skill['close_force_recenter_consistent_target'] is True
                assert skill['close_force_recenter_feedback'] is True
            if profile=='plumbers_staged_v2' and phase['name']!='base_6_close_and_attach':
                assert skill['close_force_balance_min']==8.0
                assert skill['close_force_min_stable']==skill['close_force_low']==4.0
            close_phases.append(dict(phase=phase['name'],robot=skill['robot'],
                                     force_low=skill['close_force_low'],force_high=skill['close_force_high']))
assert close_phases
if profile=='plumbers_staged_v2':
    attachments=[entry for phase in cfg['phase_specs'] for entry in phase.get('attach',[])]
    assert len(attachments)==8,len(attachments)
    assert all(entry['hold_position_slip_reference']=='grasp_point' for entry in attachments)
home_phase=[p for p in cfg['phase_specs'] if p['name'].startswith('beam_terminal_home_')][-1]
for kind in ['local_skill_complete','gripper_open']:
    assert {x['robot'] for x in home_phase['advance']['conditions'] if x['type']==kind}=={'franka_left','franka_right'}
# Necessary cross-stage invariants; no engine or nominal ownership certification.
if profile == 'plumbers_staged_v2':
    import importlib.util
    _module_spec = importlib.util.spec_from_file_location('beam_workflow_audit', OUT/'review_workflow.py')
    _audit = importlib.util.module_from_spec(_module_spec)
    _module_spec.loader.exec_module(_audit)
    _result = _audit.review(cfg)
    (OUT/'phase_contract_preflight.json').write_text(json.dumps(_result, indent=2)+'\n')
    assert not _result['issues'], _result['issues']

report=dict(passed=True,native_started=False,profile=contract,phase_count=len(recipe['phases']),
            worker_config_max_steps=cfg['max_steps'],worker_robots=[r['name'] for r in cfg['robots']],
            runtime_cameras=len(cfg.get('camera_metadata',[])),forbidden_mechanisms=issues,
            weak_side_closing_phases=close_phases,
            both_home_conditions=home_phase['advance'],
            worker_config_sha256=hashlib.sha256((OUT/'production_worker_config.json').read_bytes()).hexdigest(),
            baseline_worker_sha256=hashlib.sha256((ROOT/'grasp_repairs_20261012_20rounds/round24_native_release/full_assembly_worker.py').read_bytes()).hexdigest(),
            final=final)
(OUT/'production_preflight.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='final'},ensure_ascii=False))
