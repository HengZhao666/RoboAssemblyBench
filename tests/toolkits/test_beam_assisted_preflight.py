import ast
import copy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import numpy as np

from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import (
    AssistedConfigurationError, BEAM_OBJECTS, SUPPORTED_ADVANCE_TYPES,
    assert_assisted_configuration,
    configuration_signature,
)


@pytest.fixture(scope='module')
def framework_model():
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    return build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',
        scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=505,
        domain_randomization_enabled=True)


@pytest.fixture(scope='module')
def resolved(framework_model):
    return framework_model.model_dump(serialize_as_any=True)


def _framework_mapped_config(framework_model, env_id):
    from internutopia.core.task_config_manager.base import setup_offset_for_assets
    config = framework_model.model_copy(deep=True)
    setup_offset_for_assets(config, env_id, [0., 0., 0.])
    assert [r.name for r in config.robots] == [name + '_' + str(env_id) for name in config.robot_names]
    return config


def _recorded_framework_model_copy(template, recorded):
    """Keep every actual config subclass while applying the full saved data.

    Revalidation through RobotCfg/ControllerCfg/SensorCfg base annotations can
    discard subclasses. Copy the builder's actual models at every layer;
    the final worker-equivalent JSON equality still checks every saved field.
    """
    from pydantic import BaseModel
    if isinstance(template, BaseModel):
        assert isinstance(recorded, dict)
        field_names = set(type(template).model_fields)
        field_names.update(template.__pydantic_extra__ or {})
        assert set(recorded) == field_names
        updates = {name: _recorded_framework_model_copy(getattr(template, name), value)
                   for name, value in recorded.items()}
        result = template.model_copy(deep=True, update=updates)
        assert type(result) is type(template)
        return result
    if isinstance(template, dict) and isinstance(recorded, dict):
        return {name: _recorded_framework_model_copy(template[name], value)
                if name in template else copy.deepcopy(value)
                for name, value in recorded.items()}
    if isinstance(template, (list, tuple)) and isinstance(recorded, (list, tuple)):
        if any(isinstance(value, BaseModel) for value in template):
            assert all(isinstance(value, BaseModel) for value in template)
            by_name = {value.name: value for value in template}
            assert len(by_name) == len(template) == len(recorded)
            assert {value['name'] for value in recorded} == set(by_name)
            values = [_recorded_framework_model_copy(by_name[value['name']], value)
                      for value in recorded]
        else:
            values = [_recorded_framework_model_copy(template[index], value)
                      if index < len(template) else copy.deepcopy(value)
                      for index, value in enumerate(recorded)]
        return tuple(values) if isinstance(template, tuple) else values
    return copy.deepcopy(recorded)


@pytest.mark.parametrize('env_id', [0, 1])
def test_final_home_accepts_real_framework_mapping_without_changing_targets(framework_model, env_id):
    config = _framework_mapped_config(framework_model, env_id)
    before = framework_model.model_dump(serialize_as_any=True)
    assert config.robot_names == before['robot_names']
    assert config.phase_specs[-1]['beam_assisted_home_targets'] == before['phase_specs'][-1]['beam_assisted_home_targets']
    assert [r.initial_joint_positions for r in config.robots] == [r['initial_joint_positions'] for r in before['robots']]
    assert assert_assisted_configuration(config)['passed']
    assert assert_assisted_configuration(_runtime_task(config.model_dump(serialize_as_any=True)))['passed']


def test_final_home_unmapped_config_still_accepts_original_initial_joints(framework_model):
    assert assert_assisted_configuration(framework_model)['passed']


@pytest.mark.parametrize('env_id', [0, 1])
@pytest.mark.parametrize('logical_name', ['franka_left', 'franka_right'])
def test_final_home_real_framework_mapping_still_rejects_wrong_initial_joints(framework_model, env_id, logical_name):
    config = _framework_mapped_config(framework_model, env_id)
    robot = next(r for r in config.robots if r.name == logical_name + '_' + str(env_id))
    robot.initial_joint_positions['wrist_3_joint'] += .02
    with pytest.raises(AssistedConfigurationError, match=logical_name + ': final home target differs'):
        assert_assisted_configuration(config)


@pytest.mark.parametrize('defect', ['duplicate_alias', 'missing_arm', 'mixed_environment',
                                  'mixed_scope', 'unknown_alias', 'repeated_scope',
                                  'noncanonical_scope', 'duplicate_logical_name'])
def test_final_home_real_framework_mapping_rejects_ambiguous_aliases(framework_model, defect):
    config = _framework_mapped_config(framework_model, 0)
    if defect == 'duplicate_alias':
        duplicate = config.robots[0].model_copy(deep=True)
        duplicate.name = config.robot_names[0]
        config.robots.append(duplicate)
    elif defect == 'missing_arm':
        config.robots.pop()
    elif defect == 'mixed_environment':
        config.robots[1].name = config.robot_names[1] + '_1'
    elif defect == 'mixed_scope':
        config.robots[0].name = config.robot_names[0]
    elif defect == 'unknown_alias':
        config.robots[0].name = 'unrelated_robot_0'
    elif defect == 'repeated_scope':
        config.robots[0].name += '_0'
    elif defect == 'noncanonical_scope':
        config.robots[0].name = config.robot_names[0] + '_00'
    else:
        config.robot_names = (*config.robot_names, config.robot_names[0])
    with pytest.raises(AssistedConfigurationError, match='robot configuration aliases are invalid'):
        assert_assisted_configuration(config)


def test_final_home_real_framework_mapping_does_not_depend_on_robot_list_order(framework_model):
    config = _framework_mapped_config(framework_model, 1)
    config.robots.reverse()
    assert assert_assisted_configuration(config)['passed']


@pytest.mark.parametrize('env_id', [0, 1])
def test_recorded_a43_config_distinguishes_old_alias_failure_from_fix(framework_model, env_id):
    import importlib.util
    import json
    import os
    from toolkits.factory_dual_franka_assembly.generate_demos import _to_jsonable
    recorded = os.environ.get('BEAM_ASSISTED_PREFLIGHT_SNAPSHOT')
    baseline = os.environ.get('BEAM_ASSISTED_PREFLIGHT_BASELINE')
    if not recorded and not baseline:
        pytest.skip('Bind the original A43 snapshot and original v34 preflight for recorded-input validation')
    assert recorded and baseline, 'Recorded-input validation requires both original evidence paths'
    raw = json.loads(Path(recorded).read_text())['task_configs'][0]
    config = _recorded_framework_model_copy(framework_model, raw)
    # Rehydrate the complete recorded model without replacing any joint target,
    # phase, tolerance, path or metadata with newly generated values.
    assert json.loads(json.dumps(_to_jsonable(
        config.model_dump(mode='python', serialize_as_any=True)))) == raw
    spec = importlib.util.spec_from_file_location('toolkits.factory_dual_franka_assembly._original_a43_preflight', baseline)
    original = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(original)
    assert original.assert_assisted_configuration(config)['passed']
    mapped = _framework_mapped_config(config, env_id)
    with pytest.raises(original.AssistedConfigurationError, match='final home target differs'):
        original.assert_assisted_configuration(mapped)
    assert assert_assisted_configuration(mapped)['passed']


def test_reviewed_configuration_signature_changes_with_actual_camera_or_target(resolved):
    original = configuration_signature(resolved)
    assert configuration_signature(copy.deepcopy(resolved)) == original
    changed = copy.deepcopy(resolved)
    changed['target_poses']['part_6_assembled']['position'][0] += .001
    assert configuration_signature(changed) != original
    changed = copy.deepcopy(resolved)
    wrist = next(c for c in changed['camera_metadata']
                 if c.get('target_source') == 'native_fingertip_midpoint')
    wrist['native_mount_offset'][2] += .01
    assert configuration_signature(changed) != original


@pytest.mark.parametrize('field', ['initial_joints', 'robot_base', 'robot_drive', 'robot_usd',
                                  'controller', 'object_pose', 'object_scale', 'object_usd',
                                  'object_physics', 'scene', 'workspace', 'layout'])
def test_signature_covers_physical_geometry_and_initialization_inputs(resolved, field):
    changed=copy.deepcopy(resolved)
    robot=changed['robots'][0]
    part=next(o for o in changed['objects'] if o['name']=='fabrica_beam_6')
    if field=='initial_joints':robot['initial_joint_positions']['wrist_3_joint']+=.01
    elif field=='robot_base':robot['position']=list(robot['position']);robot['position'][0]+=.01
    elif field=='robot_drive':robot['arm_drive_kp']+=1.
    elif field=='robot_usd':robot['usd_path']+='-different'
    elif field=='controller':robot['controllers'][0]['name']+='-different'
    elif field=='object_pose':part['position']=list(part['position']);part['position'][0]+=.01
    elif field=='object_scale':part['scale']=list(part['scale']);part['scale'][0]*=1.01
    elif field=='object_usd':part['usd_path']+='-different'
    elif field=='object_physics':part['collider']=not part['collider']
    elif field=='scene':changed['scene_asset_path']+='-different'
    elif field=='workspace':changed['workspace_offset'][0]+=.01
    else:changed['layout_seed']+=1
    assert configuration_signature(changed)!=configuration_signature(resolved)


def test_model_signature_keeps_subclass_fields_omitted_by_default_pydantic_dump():
    from pydantic import BaseModel
    class Robot(BaseModel):
        name:str
    class UR5e(Robot):
        initial_joint_positions:dict[str,float]
    class Task(BaseModel):
        robots:list[Robot]
    model=Task(robots=[UR5e(name='arm',initial_joint_positions={'wrist_3_joint':0.})])
    assert 'initial_joint_positions' not in model.model_dump()['robots'][0]
    assert configuration_signature(model)==configuration_signature(model.model_dump(serialize_as_any=True))
    original=configuration_signature(model)
    model.robots[0].initial_joint_positions['wrist_3_joint']=.01
    assert configuration_signature(model)!=original


def _runtime_task(config):
    config=copy.deepcopy(config)
    return NS(cfg=config,phase_specs=copy.deepcopy(config['phase_specs']),
        target_poses=copy.deepcopy(config['target_poses']),max_steps=config['max_steps'],
        _beam_assisted_enabled=True,_object_prims={name:object() for name in BEAM_OBJECTS},
        robots={name:NS(controllers={'arm_joint_controller':object(),'gripper_controller':object()})
                for name in config['robot_names']},
        _assisted_idle_action=lambda *a:None,_assisted_guard_actions=lambda *a:None)


@pytest.mark.parametrize('field', ['phase_specs', 'target_poses', 'max_steps'])
def test_runtime_configuration_divergence_cannot_reuse_reviewed_config_identity(resolved,field):
    task=_runtime_task(resolved)
    if field=='target_poses':
        task.target_poses['part_6_assembled']['position'][0]+=.001
    elif field=='phase_specs':
        task.phase_specs[0]['local_skill']['cartesian_position_step']*=.9
    else:task.max_steps+=1
    with pytest.raises(AssistedConfigurationError,match='runtime .* differ'):
        assert_assisted_configuration(task)


def test_supported_condition_contract_matches_real_evaluator():
    source=Path(__file__).resolve().parents[2]/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
    tree=ast.parse(source.read_text())
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)
                and n.name=='_evaluate_advance_condition')
    supported=set()
    for node in ast.walk(method):
        if isinstance(node,ast.Compare) and isinstance(node.left,ast.Name) and node.left.id=='advance_type':
            for comparator in node.comparators:
                if isinstance(comparator,ast.Constant) and isinstance(comparator.value,str):
                    supported.add(comparator.value)
                elif isinstance(comparator,(ast.Set,ast.Tuple,ast.List)):
                    supported.update(x.value for x in comparator.elts if isinstance(x,ast.Constant))
    assert supported==SUPPORTED_ADVANCE_TYPES


def test_real_randomized_config_and_runtime_interfaces_are_admitted_without_claiming_physics(resolved):
    import json
    report=assert_assisted_configuration(resolved)
    json.dumps(report)
    assert report['passed'] and report['phase_count']==101
    assert report['runtime_interfaces_checked'] is False
    assert report['formal_physical_success'] is False and len(report['not_certified'])>=5
    assert len(report['capture_targets'])==5
    # Same data can be checked after task construction; this fixture checks
    # interface admission only. Native body/command evidence belongs to the run.
    task=_runtime_task(resolved)
    runtime_report=assert_assisted_configuration(task)
    assert runtime_report['runtime_interfaces_checked'] is True
    assert runtime_report['runtime_phase_target_values_match_config'] is True
    assert runtime_report['phase_target_signature']==report['phase_target_signature']
    del task.robots['franka_right'].controllers['arm_joint_controller']
    with pytest.raises(AssistedConfigurationError,match='controllers are not registered'):
        assert_assisted_configuration(task)


@pytest.mark.parametrize('defect,expected',[
    ('capture_alias','static assisted_capture aliases'),
    ('offset','world-Z 3 mm'),
    ('missing_object','unknown object'),
    ('unsupported_final','unsupported advance type object_at_target'),
    ('short_budget','below all-phase budget'),
    ('collision_disable','keep collisions and gravity enabled'),
    ('nonfinite_future_target','every target requires a finite position'),
    ('zero_future_quaternion','every target requires a finite position'),
    ('unknown_skill','registered task skill interface'),
    ('completion_owner','completion skill/robot does not match'),
    ('capture_tolerance','strict 1 mm/3 mrad'),
    ('capture_split_axis','strict 1 mm/3 mrad'),
    ('capture_overdrive','strict 1 mm/3 mrad'),
    ('insertion_boundary','below the 3 mm capture boundary'),
])
def test_known_engineering_failures_are_rejected_before_action(resolved,defect,expected):
    config=copy.deepcopy(resolved)
    capture=next(p['local_skill'] for p in config['phase_specs']
                 if p.get('local_skill',{}).get('target_object_target','').endswith('_assembled'))
    if defect=='capture_alias':
        alias=capture['target_object_target']+'_assisted_capture'
        config['target_poses'][alias]=copy.deepcopy(config['target_poses'][capture['target_object_target']])
        capture['target_object_target']=alias
    elif defect=='offset':
        capture['target_object_offset']=[.084,.099,.003]
    elif defect=='missing_object':
        capture['object']='missing_beam'
    elif defect=='unsupported_final':
        config['phase_specs'][-1]['advance']['conditions'][0]={'type':'object_at_target',
            'object':'fabrica_beam_6','target':'part_6_assembled'}
    elif defect=='short_budget':
        config['max_steps']=30000
    elif defect=='collision_disable':
        config['phase_specs'][0]['object_collisions']=[{'object':'fabrica_beam_6','enabled':False}]
    elif defect=='nonfinite_future_target':
        config['target_poses']['part_0_pickup_clearance']['position'][2]=float('nan')
    elif defect=='zero_future_quaternion':
        config['target_poses']['part_0_insert_08']['orientation']=[0.,0.,0.,0.]
    elif defect=='unknown_skill':
        config['phase_specs'][0]['local_skill']['name']='unknown_skill'
    elif defect=='completion_owner':
        advance=config['phase_specs'][0]['advance']
        advance['robot']='franka_left' if advance['robot']=='franka_right' else 'franka_right'
    elif defect=='capture_tolerance':capture['target_object_position_tolerance']=.008
    elif defect=='capture_split_axis':capture['target_object_lateral_position_tolerance']=.015
    elif defect=='capture_overdrive':capture['target_object_servo_position_command_gate_overdrive']=True
    else:
        config['target_poses']['part_3_insert_08']['position'][2]=config['target_poses']['part_3_assembled']['position'][2]+.00082
    with pytest.raises(AssistedConfigurationError,match=expected) as caught:
        assert_assisted_configuration(config)
    assert caught.value.report['passed'] is False


def test_short_budget_requires_explicit_bounded_mode(resolved):
    config=copy.deepcopy(resolved)
    config['max_steps']=30000
    with pytest.raises(AssistedConfigurationError,match='below all-phase budget'):
        assert_assisted_configuration(config)
    report=assert_assisted_configuration(config,full_mode=False)
    assert report['passed'] and report['full_mode'] is False
    assert report['full_run_budget_checked'] is False
    assert report['full_run_max_steps']<report['all_phase_budget_required']


def test_right_visual_support_has_no_second_attachment_and_stays_until_left_work_ends(resolved):
    phases = resolved['phase_specs']
    names = [p['name'] for p in phases]
    release = names.index('base_6_release_and_lock')
    support = names.index('base_6_visual_support')
    opening = names.index('base_6_visual_support_open')
    assert support == release + 1
    assert all(i < opening for i, name in enumerate(names) if name.startswith('assemble_'))
    assert names[opening + 1:] == ['base_6_retreat',
        'beam_assisted_return_home_franka_left',
        'beam_assisted_return_clearance_franka_right',
        'beam_assisted_return_home_franka_right', 'beam_assisted_final_verification']
    assert 'base_6_park' not in names
    assert names[-1] == 'beam_assisted_final_verification'
    close = phases[support]
    assert close['advance'] == {'type': 'local_skill_complete', 'robot': 'franka_right',
                                'skill': 'ur5e_close_gripper', 'min_steps': 1}
    assert close['local_skill']['beam_assisted_visual_pinch'] is True
    assert close['local_skill']['requires_held_object'] is False
    assert close['local_skill']['require_close_pose_gate'] is False
    for phase in phases[support:opening]:
        assert not any(a.get('object') == 'fabrica_beam_6' for a in phase.get('attach', []))
        assert not any(a.get('object') == 'fabrica_beam_6' for a in phase.get('lock', []))
        marker = phase['beam_assisted_visual_support']
        assert marker['mode'] == 'non_attached_visible_pinch'
        assert marker['minimum_surface_gap_m'] == .000005
        assert marker['maximum_surface_gap_m'] == .00025
        assert marker['force_support_claimed'] is False
    for phase in phases[support + 1:opening]:
        assert phase.get('local_skill', {}).get('robot') != 'franka_right'
        assert 'franka_right' not in phase.get('local_skills', {})
        assert 'franka_right' not in phase.get('gripper_commands', {})
    open_spec = phases[opening]['local_skill']
    assert open_spec['name'] == 'ur5e_preshape_gripper'
    assert open_spec['gripper_openness'] == 1.
    assert open_spec['gripper_position_tolerance'] == .005
    assert open_spec['stable_steps'] == 12
    assert not phases[opening].get('detach') and not phases[opening].get('lock')


@pytest.mark.parametrize('defect', ['initial_branch', 'gripper_closed', 'wrap_enabled',
                                  'missing_guard', 'wrong_order', 'unsafe_lift', 'no_native_ticks'])
def test_final_home_contract_rejects_unsafe_return(resolved, defect):
    config = copy.deepcopy(resolved)
    phases = config['phase_specs']
    final = phases[-1]
    left = next(p for p in phases if p['name'] == 'beam_assisted_return_home_franka_left')
    clearance = next(p for p in phases if p['name'] == 'beam_assisted_return_clearance_franka_right')
    if defect == 'initial_branch':
        final['beam_assisted_home_targets']['franka_left']['joint_positions'][-1] += 2*np.pi
    elif defect == 'gripper_closed':
        left['gripper_commands']['franka_right'] = 'close'
    elif defect == 'wrap_enabled':
        left['local_skill']['unwrap_revolute_joints'] = True
    elif defect == 'missing_guard':
        final['beam_assisted_empty_return_guard'] = False
    elif defect == 'wrong_order':
        i, j = phases.index(left), phases.index(clearance)
        phases[i], phases[j] = phases[j], phases[i]
    elif defect == 'unsafe_lift':
        clearance['local_skill']['beam_assisted_joint_target_delta'][0] = .24
    else:
        left['local_skill']['beam_assisted_native_joint_target'] = False
    with pytest.raises(AssistedConfigurationError):
        assert_assisted_configuration(config)


@pytest.mark.parametrize('outlet', ['primary', 'companion', 'gripper'])
def test_support_compiler_refuses_conflicting_right_commands(outlet):
    from roboassemblybench.core.beam_assisted_compiler import _add_visual_base_support
    phase = {'name': 'assemble_00_part_3_place'}
    if outlet == 'primary':phase['local_skill'] = {'robot': 'franka_right'}
    elif outlet == 'companion':phase['local_skills'] = {'franka_right': {}}
    else:phase['gripper_commands'] = {'franka_right': 'open'}
    result = {'phases': [{'name': 'base_6_release_and_lock'}, {'name': 'base_6_retreat'},
                         {'name': 'base_6_park'}, phase], 'success': []}
    with pytest.raises(ValueError, match='support conflicts with a right-arm command'):
        _add_visual_base_support(result)


def test_assisted_object_targets_use_absolute_servo_and_descent_admission(resolved):
    checked = set()
    for phase in resolved['phase_specs']:
        specs = [phase.get('local_skill', {}), *phase.get('local_skills', {}).values()]
        for spec in specs:
            if not spec.get('requires_held_object'):
                continue
            name = spec.get('target_object_target', '')
            if not name:
                assert not spec.get('beam_assisted_absolute_object_servo')
                continue
            assert spec['beam_assisted_absolute_object_servo'] is True
            assert spec['beam_assisted_fk_native_position_limit_m'] == .0001
            assert spec['beam_assisted_fk_native_orientation_limit_rad'] == .0015
            if spec['object'] == 'fabrica_beam_6' or not (
                    name.endswith(('_assembly_hover', '_assembled')) or '_insert_' in name):
                continue
            checked.add(spec['object'])
            admission = spec['beam_assisted_descent_admission']
            assert admission['lateral_tolerance_m'] == .00015
            assert admission['orientation_tolerance_rad'] == .001
            assert admission['stable_native_ticks'] == 12
            assert admission['geometry_certificate'] == 'C01_full_insert_corridor'
            assert spec['target_object_convergence_axis'] == [0., 0., -1.]
    assert checked == {'fabrica_beam_3', 'fabrica_beam_2', 'fabrica_beam_1', 'fabrica_beam_0'}


def test_second_column_rises_before_turning_over_the_support_arm(resolved):
    pickup = resolved['target_poses']['part_2_pickup_clearance']
    carry = resolved['target_poses']['part_2_assembly_clearance']
    fixture = resolved['target_poses']['part_2_fixture_pickup']
    assert pickup['position'][2] >= fixture['position'][2] + .149 - 1e-12
    assert carry['position'][2] >= pickup['position'][2]
    assert resolved['target_poses']['part_2_insert_00']['position'][2] < carry['position'][2]


def test_unknown_descent_motion_metric_rejected_before_native_start(resolved):
    changed=copy.deepcopy(resolved)
    spec=next(p['local_skill'] for p in changed['phase_specs'] if p.get('local_skill',{}).get('beam_assisted_descent_admission'))
    spec['beam_assisted_descent_admission']['motion_metric']='unknown'
    with pytest.raises(AssistedConfigurationError,match='unsupported assisted descent motion metric'):
        assert_assisted_configuration(changed)
