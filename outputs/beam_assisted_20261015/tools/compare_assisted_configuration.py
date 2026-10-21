"""Fresh production-environment baseline and whole four-layout comparison."""
from pathlib import Path
import copy,inspect,json,runpy,sys,time
import argparse
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);mode=ap.add_mutually_exclusive_group();mode.add_argument('--baseline',action='store_true');mode.add_argument('--bind-deployed',action='store_true');ap.add_argument('--object-position-iterations',type=int);ap.add_argument('--baseline-run',default='A27_beam_full');ap.add_argument('--descent-com-metric',action='store_true');ap.add_argument('--joint-branch-prealign',action='store_true');ap.add_argument('--recipe-only',action='store_true');ap.add_argument('--reuse-emitted',action='store_true');ap.add_argument('--staged-object',choices=['fabrica_beam_0']);ap.add_argument('--final-home',action='store_true');opt=ap.parse_args();assert opt.object_position_iterations in (None,64)
W=opt.work.resolve();P=W.parent;C=W/'candidate';A=P/'assisted_repository'
baseline=opt.baseline;root=A if baseline or opt.bind_deployed else C
sys.path.insert(0,str(root));sys.path.insert(1,str(P/'tools'))
from roboassemblybench.core import beam_assisted_compiler
from roboassemblybench.core.task_registry import load_task_recipe,task_recipe_fingerprint
from toolkits.factory_dual_franka_assembly import scene_builder
from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import configuration_signature
from beam_workflow import sha,save
assert Path(inspect.getfile(beam_assisted_compiler)).is_relative_to(root)
if opt.recipe_only:
    recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    name='base_recipe_input.json' if baseline else 'actual_recipe_input.json';assert not (W/name).exists()
    save(W/name,recipe);print(json.dumps(dict(recipe_input=str(W/name),fingerprint=task_recipe_fingerprint(recipe))));raise SystemExit(0)
prefix='authority_configuration' if opt.bind_deployed else ('base_configuration' if baseline else 'actual_configuration')
tool=P/'tools/emit_configuration_preflight.py';begin=time.monotonic();sys.argv=[str(tool),str(W/(prefix+'.json'))]
build=scene_builder.build_dual_franka_assembly_episode;captured=[]
def capture(**kwargs):
    config=build(**kwargs);layout=kwargs['layout_seed'];captured.append(layout)
    save(W/f'{prefix}_layout{layout}_config.json',config.model_dump(mode='python',serialize_as_any=True))
    return config
if opt.reuse_emitted:
    assert not baseline and not opt.bind_deployed
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration
    saved=json.loads((W/(prefix+'.json')).read_text());assert saved['passed'] and saved['simulator_created'] is False
    for layout in (0,1,505,777):
        cfg=json.loads((W/f'{prefix}_layout{layout}_config.json').read_text());report=assert_assisted_configuration(cfg)
        assert report['passed'] and report['configuration_signature']==next(v['report']['configuration_signature'] for v in saved['rows'] if v['layout_seed']==layout)
        captured.append(layout)
else:
    scene_builder.build_dual_franka_assembly_episode=capture
    try:runpy.run_path(str(tool),run_name='__main__')
    finally:scene_builder.build_dual_franka_assembly_episode=build
assert set(captured)=={0,1,505,777} and len(captured)==4
if opt.bind_deployed:
    source=json.loads((P/'tools/assisted_manifest.json').read_text());delta=json.loads((W/'configuration_delta.json').read_text())
    assert len(source)==889 and all(sha(A/n)==h and sha(C/n)==h for n,h in source.items())
    def authority_path(value):
        if isinstance(value,dict):return {k:authority_path(v) for k,v in value.items()}
        if isinstance(value,list):return [authority_path(v) for v in value]
        if isinstance(value,str) and value.startswith(str(C)+'/'):
            suffix=value[len(str(C)):];target=str(A)+suffix
            if '/roboassemblybench/assets/' in value and Path(value).is_file():
                assert sha(value)==sha(target)
                return target
            return target
        return value
    recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    fp=task_recipe_fingerprint(recipe);assert fp==delta['expected_authority_recipe_fingerprint']
    checks=[];inputs={}
    for layout in (0,1,505,777):
        cp=W/f'actual_configuration_layout{layout}_config.json';ap=W/f'authority_configuration_layout{layout}_config.json'
        candidate=json.loads(cp.read_text());actual=json.loads(ap.read_text());expected=authority_path(candidate);expected['recipe_fingerprint']=fp
        assert expected==actual,'Deployed model differs from qualified candidate at layout'+str(layout)
        sig=configuration_signature(actual);assert sig==next(v['expected_authority_signature'] for v in delta['layout_checks'] if v['layout_seed']==layout)
        checks.append(dict(layout_seed=layout,full_task_model_exact=True,signature=sig));inputs.update({str(cp):sha(cp),str(ap):sha(ap)})
    target=W/'authority_configuration_binding.json';assert not target.exists()
    save(target,dict(passed=True,current_source_manifest_sha256=sha(P/'tools/assisted_manifest.json'),candidate_and_authority_sources_exact=True,
        full_task_model_exact=True,all_four_authority_layouts_passed=True,recipe_fingerprint_independently_rederived=True,
        authority_configuration_signature=checks[2]['signature'],layout_checks=checks,input_sha256=inputs,tool_sha256=sha(__file__),native_started=False))
    print(json.dumps(dict(passed=True,layouts=4,authority_signature=checks[2]['signature'])));raise SystemExit(0)
if baseline:
    real=json.loads((P/'runs'/opt.baseline_run/'task_config_snapshot.json').read_text())['task_configs'][0]
    assert json.loads((W/'base_configuration_layout505_config.json').read_text())==real,'Baseline is not the actual normal launcher configuration'
    save(W/'base_configuration_identity.json',dict(passed=True,actual_recorded_full_build_config_equal=True,baseline_run=opt.baseline_run,actual_A27_full_build_config_equal=opt.baseline_run=='A27_beam_full',source_manifest_sha256=sha(P/'tools/assisted_manifest.json'),elapsed_s=time.monotonic()-begin))
    raise SystemExit(0)
# Resolve assets against the actual baseline references. The authority uses
# preserved asset symlinks; candidate regular-file paths differ but bytes must match.
asset_map={}
def assets(v):
    if isinstance(v,dict):
        for x in v.values():assets(x)
    elif isinstance(v,list):
        for x in v:assets(x)
    elif isinstance(v,str) and '/roboassemblybench/assets/' in v:
        suffix=v.split('/roboassemblybench/assets/',1)[1]
        if Path(v).is_file() and str(C/'roboassemblybench/assets'/suffix) not in asset_map:
            candidate=C/'roboassemblybench/assets'/suffix
            assert candidate.is_file() and sha(candidate)==sha(v)
            asset_map[str(candidate)]=v
assets(json.loads((W/'base_configuration_layout505_config.json').read_text()))
def authority(value):
    if isinstance(value,dict):return {k:authority(v) for k,v in value.items()}
    if isinstance(value,list):return [authority(v) for v in value]
    if isinstance(value,str):
        if value in asset_map:return asset_map[value]
        if value.startswith(str(C)+'/'):return str(A)+value[len(str(C)):]
    return value
recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
projected_recipe=authority(copy.deepcopy(recipe));removed=0
if opt.final_home:
    old_recipe=json.loads((W/'base_recipe_input.json').read_text())
    return_names={'beam_assisted_return_home_franka_left',
                  'beam_assisted_return_clearance_franka_right',
                  'beam_assisted_return_home_franka_right'}
    def project_home(value, collection, prior):
        actual=value[collection];removed=[p for p in actual if p['name'] in return_names]
        assert len(removed)==3 and {p['name'] for p in removed}==return_names
        assert all(p['beam_assisted_empty_return_guard'] and not p.get('attach')
                   and all(v=='open' for v in p['gripper_commands'].values()) for p in removed)
        value[collection]=[p for p in actual if p['name'] not in return_names]
        final=value[collection][-1]
        assert final['name']=='beam_assisted_final_verification'
        assert final.pop('beam_assisted_empty_return_guard') is True
        targets=final.pop('beam_assisted_home_targets')
        assert set(targets)=={'franka_left','franka_right'}
        assert value['max_steps']-prior['max_steps']==sum(p['timeout_steps'] for p in removed)
        value['max_steps']=prior['max_steps']
        for metadata_key in ['metadata' if collection=='phases' else 'benchmark_metadata','task_metadata']:
            metadata=value[metadata_key];prior_metadata=prior[metadata_key]
            interface=metadata['local_skills'].pop('move_arm_to_joint_positions')
            assert interface==prior_metadata['local_skills']['ur5e_retreat_vertical']
        if collection=='phase_specs':
            annotations=value['phase_annotations']
            assert [v['name'] for v in annotations]==[p['name'] for p in actual]
            value['phase_annotations']=[v for v in annotations if v['name'] not in return_names]
        return removed
    extra=project_home(projected_recipe,'phases',old_recipe)
    baseline_fp=task_recipe_fingerprint(projected_recipe)
    assert baseline_fp==old_recipe['recipe_fingerprint'], 'Projected recipe fingerprint differs from actual baseline'
    projected_recipe['recipe_fingerprint']=baseline_fp
    assert projected_recipe==old_recipe, 'Recipe changed outside declared final empty-hand return'
    actual_fp=task_recipe_fingerprint(authority(copy.deepcopy(recipe)))
    checks=[]
    for layout in (0,1,505,777):
        new=json.loads((W/f'actual_configuration_layout{layout}_config.json').read_text())
        old=json.loads((W/f'base_configuration_layout{layout}_config.json').read_text())
        assert task_recipe_fingerprint(recipe)==new['recipe_fingerprint']
        projected=authority(copy.deepcopy(new));project_home(projected,'phase_specs',old)
        assert baseline_fp==old['recipe_fingerprint'];projected['recipe_fingerprint']=baseline_fp
        assert projected==old, 'Undeclared home configuration delta at layout'+str(layout)
        canonical=authority(copy.deepcopy(new));canonical['recipe_fingerprint']=actual_fp
        checks.append(dict(layout_seed=layout,whole_config_exact_after_declared_projection=True,
            baseline_signature=configuration_signature(old),
            expected_authority_signature=configuration_signature(canonical),
            success_criteria_exact=new['success_criteria']==old['success_criteria']))
    save(W/'configuration_delta.json',dict(passed=True,actual_layouts=4,final_home=True,
        unchanged_prefix_and_original_final_gates=True,added_empty_return_phases=[p['name'] for p in extra],
        wrist_camera_configuration_exact=True,native_camera_roll_source_checked_separately=True,
        expected_authority_recipe_fingerprint=actual_fp,layout_checks=checks,
        recipe_path_fingerprint_recomputed=True,source_manifest_sha256=sha(P/'tools/assisted_manifest.json'),
        tool_sha256=sha(__file__),simulator_created=False,elapsed_s=time.monotonic()-begin))
    print(json.dumps(dict(passed=True,layouts=4,added_return_phases=3,expected505=checks[2]['expected_authority_signature'])))
    raise SystemExit(0)
def restore_branch(phase,old):
    assert phase.pop('beam_assisted_native_joint_limits') is True
    count=1
    if phase['name'] in ('assemble_02_part_1_move_above','assemble_03_part_0_move_above'):
        settings=phase['local_skill'];prior=old['local_skill']
        expected=dict(prealign_until_converged=True,prealign_joint_position_tolerance=.05,
                      prealign_timeout_steps=phase['timeout_steps'],unwrap_revolute_joints=False)
        for k,v in expected.items():
            assert settings[k]==v
            if k in prior:settings[k]=copy.deepcopy(prior[k])
            else:settings.pop(k)
        count+=1
    return count
def restore_staged(phase, old):
    if phase['name'].endswith('_assembly_clearance') and phase.get('local_skill',{}).get('object')==opt.staged_object:
        expected={'beam_assisted_staged_clearance':dict(height_padding_m=.04,position_tolerance_m=.001,orientation_tolerance_rad=.003),
                  'beam_assisted_lift_command_rotation_hold':True,'beam_assisted_incremental_clearance':True}
        settings=phase['local_skill'];prior=old['local_skill']
        for k,v in expected.items():
            assert settings[k]==v
            if k in prior:settings[k]=copy.deepcopy(prior[k])
            else:settings.pop(k)
        return 3
    return 0
old_recipe=json.loads((W/'base_recipe_input.json').read_text()) if opt.joint_branch_prealign or opt.staged_object else None
for phase in projected_recipe['phases']:
    settings=phase.get('local_skill') or {}
    if opt.staged_object:
        removed+=restore_staged(phase,next(p for p in old_recipe['phases'] if p['name']==phase['name']))
    elif opt.joint_branch_prealign:
        removed+=restore_branch(phase,next(p for p in old_recipe['phases'] if p['name']==phase['name']))
    elif opt.descent_com_metric and settings.get('beam_assisted_descent_admission'):
        assert settings['beam_assisted_descent_admission'].pop('motion_metric')=='native_com_pose_difference'
        removed+=1
    elif not opt.descent_com_metric and phase['name']=='assemble_02_part_1_assembly_clearance':
        assert settings.pop('beam_assisted_incremental_clearance') is True
        removed+=1
assert removed==(3 if opt.staged_object else 100 if opt.joint_branch_prealign else 40 if opt.descent_com_metric else 1)
if opt.object_position_iterations is not None:
    selected=[o for o in projected_recipe['objects'] if o['name']=='fabrica_beam_1'];assert len(selected)==1 and selected[0]['solver_position_iteration_count']==opt.object_position_iterations
    selected[0]['solver_position_iteration_count']=16
baseline_fp=task_recipe_fingerprint(projected_recipe);actual_fp=task_recipe_fingerprint(authority(recipe));checks=[]
for layout in (0,1,505,777):
    new=json.loads((W/f'actual_configuration_layout{layout}_config.json').read_text());old=json.loads((W/f'base_configuration_layout{layout}_config.json').read_text())
    assert task_recipe_fingerprint(recipe)==new['recipe_fingerprint']
    projected=authority(copy.deepcopy(new));removed=0
    for phase in projected['phase_specs']:
        settings=phase.get('local_skill') or {}
        if opt.staged_object:
            removed+=restore_staged(phase,next(p for p in old['phase_specs'] if p['name']==phase['name']))
        elif opt.joint_branch_prealign:
            removed+=restore_branch(phase,next(p for p in old['phase_specs'] if p['name']==phase['name']))
        elif opt.descent_com_metric and settings.get('beam_assisted_descent_admission'):
            assert settings['beam_assisted_descent_admission'].pop('motion_metric')=='native_com_pose_difference'
            removed+=1
        elif not opt.descent_com_metric and phase['name']=='assemble_02_part_1_assembly_clearance':
            assert settings.pop('beam_assisted_incremental_clearance') is True
            removed+=1
    if opt.object_position_iterations is not None:
        for collection in ['objects','object_metadata']:
            selected=[o for o in projected[collection] if o['name']=='fabrica_beam_1'];assert len(selected)==1 and selected[0]['solver_position_iteration_count']==opt.object_position_iterations
            selected[0]['solver_position_iteration_count']=16
    assert removed==(3 if opt.staged_object else 100 if opt.joint_branch_prealign else 40 if opt.descent_com_metric else 1) and baseline_fp==old['recipe_fingerprint']
    projected['recipe_fingerprint']=baseline_fp;assert projected==old,'Undeclared configuration delta at layout'+str(layout)
    canonical=authority(new);canonical['recipe_fingerprint']=actual_fp
    checks.append(dict(layout_seed=layout,whole_config_exact_after_declared_projection=True,whole_config_exact_after_only_incremental_reference_projection=not opt.joint_branch_prealign and not opt.descent_com_metric and opt.object_position_iterations is None,
        baseline_signature=configuration_signature(old),expected_authority_signature=configuration_signature(canonical),
        success_criteria_exact=new['success_criteria']==old['success_criteria']))
save(W/'configuration_delta.json',dict(passed=True,actual_layouts=4,staged_object=opt.staged_object,only_fourth_incremental_reference_changed=not opt.joint_branch_prealign and not opt.descent_com_metric and opt.object_position_iterations is None,declared_object_position_iterations=opt.object_position_iterations,only_declared_config_delta=True,
    native_joint_limit_guard_phase_count=98 if opt.joint_branch_prealign else 0,raw_branch_prealign_phase_count=2 if opt.joint_branch_prealign else 0,
    descent_COM_metric_phase_count=40 if opt.descent_com_metric else 0,baseline_run=opt.baseline_run,actual_recorded_baseline_matched=True,actual_A27_baseline_matched=opt.baseline_run=='A27_beam_full',production_launcher_environment_used=True,recipe_path_fingerprint_recomputed=True,
    layout_checks=checks,expected_authority_recipe_fingerprint=actual_fp,elapsed_s=time.monotonic()-begin,
    source_manifest_sha256=sha(P/'tools/assisted_manifest.json'),tool_sha256=sha(__file__),simulator_created=False))
print(json.dumps(dict(passed=True,layouts=4,expected505=checks[2]['expected_authority_signature'])))
