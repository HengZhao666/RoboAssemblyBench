import copy
import numpy as np
import pytest
from roboassemblybench.core.task_registry import load_task_recipe
from toolkits.factory_dual_franka_assembly.beam_assisted_state import bounded_native_target, BeamAssistedState

def test_native_wrist_branch_does_not_jump_one_turn():
    q=np.array([0.,0.,0.,0.,0.,-3.4416294098])
    candidate=q.copy();candidate[-1]+=2*np.pi
    np.testing.assert_allclose(bounded_native_target(candidate,q),q,atol=1e-12)

def test_stale_target_cannot_pull_arm_back_by_one_radian():
    actual=np.zeros(6);old=np.ones(6)
    result=bounded_native_target(old,actual,old)
    assert np.max(np.abs(result-actual)) <= .02+1e-12

def test_step_and_tracking_are_bounded_against_actual_and_published():
    actual=np.zeros(6);prior=np.full(6,.11)
    result=bounded_native_target(np.ones(6),actual,prior)
    assert np.max(np.abs(result-actual)) <= .12+1e-12
    assert np.max(np.abs(result-prior)) <= .02+1e-12

@pytest.mark.parametrize('target,actual', [([np.nan]*6,[0.]*6),([0.]*5,[0.]*6)])
def test_invalid_command_is_rejected(target,actual):
    with pytest.raises(ValueError):bounded_native_target(target,actual)

def test_assistance_rejects_empty_grasp_before_any_joint_or_pose_write():
    class Task(BeamAssistedState):
        def _gripper_contact_metrics(self,*args,**kwargs):return {}
        def _strict_physical_grasp_contact(self,*args,**kwargs):return {'physical_contact_ready':False}
    with pytest.raises(RuntimeError,match='not touching'):
        Task()._assisted_attach('part','robot',{}, {})

def test_assisted_recipe_keeps_colliders_and_removes_teleport_carry():
    r=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    assert r['metadata']['beam_execution_route']=='assisted'
    assert r['metadata']['formal_physical_success'] is False
    assert len(r['phases'])>50
    for p in r['phases']:
        assert p['beam_assisted'] is True
        assert not p.get('kinematic_carry') and not p.get('collision_filters')
        for a in p.get('attach',[]):
            assert a['attachment_mode']=='fixed_joint'
            assert not a['disable_collision_on_attach']
            assert not a['filter_gripper_collisions_on_attach']
            assert not a['allow_noncontact_joint_attachment']
        for a in p.get('lock',[]):
            assert a['freeze_current_pose']
            assert not a['pin_pose'] and not a['disable_collision_on_lock']
    assert all(not p['name'].startswith(('hold_2_', 'hold_3_', 'handover_')) for p in r['phases'])

def test_physical_recipe_stays_separate():
    r=load_task_recipe('fabrica_beam_ur5e_staged',scene_profile='taoyuan_grscenes_tabletop')
    assert len(r['phases'])>=140
    assert not any(p.get('beam_assisted') for p in r['phases'])
    for p in r['phases']:
        for a in p.get('attach',[]):assert a['attachment_mode']=='pure_physical_grasp'


def test_assisted_scene_and_randomization_refer_to_same_objects():
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    config=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',
        scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=505,
        domain_randomization_enabled=True)
    assert config.recipe=='fabrica_beam_ur5e_assisted'
    assert all(p.get('beam_assisted') for p in config.phase_specs)


def test_assisted_idle_arm_holds_native_branch_and_opens_gripper():
    from types import SimpleNamespace as NS
    native=np.array([0.,0.,0.,0.,0.,-3.44])
    subset=NS(get_joint_positions=lambda:native.copy())
    controller=NS(get_joint_subset=lambda:subset)
    t=BeamAssistedState();t.robots={'right':NS(controllers={'arm_joint_controller':controller})}
    a=t._assisted_idle_action('right',{'gripper_commands':{'right':'open'}})
    assert 'arm_ik_controller' not in a
    np.testing.assert_allclose(a['arm_joint_controller'][0],native)
    assert a['gripper_controller']==[1.]

def test_final_guard_removes_overridden_ik_and_preserves_native_command():
    from types import SimpleNamespace as NS
    native=np.array([0.,0.,0.,0.,0.,-3.44])
    subset=NS(get_joint_positions=lambda:native.copy())
    controller=NS(get_joint_subset=lambda:subset)
    t=BeamAssistedState();t.step_counter=1
    t.robots={'right':NS(controllers={'arm_joint_controller':controller})}
    requested=native.copy();requested[-1]+=2*np.pi
    a=t._assisted_guard_actions({'right':{'arm_joint_controller':[requested.tolist()],
        'arm_ik_controller':[[0.,0.,0.],[1.,0.,0.,0.]]}})
    assert 'arm_ik_controller' not in a['right']
    np.testing.assert_allclose(a['right']['arm_joint_controller'][0],native,atol=1e-12)


def test_idle_anchor_does_not_follow_gravity_and_rearms_after_active_motion():
    from types import SimpleNamespace as NS
    actual=np.array([0.,-1.57,1.57,-1.57,-1.57,0.])
    subset=NS(get_joint_positions=lambda:actual.copy())
    t=BeamAssistedState();t.robots={'left':NS(controllers={'arm_joint_controller':NS(get_joint_subset=lambda:subset)})}
    first=t._assisted_idle_action('left',{})['arm_joint_controller'][0]
    actual[1]+=.06
    np.testing.assert_allclose(t._assisted_idle_action('left',{})['arm_joint_controller'][0],first)
    t._assisted_active_arm('left')
    np.testing.assert_allclose(t._assisted_idle_action('left',{})['arm_joint_controller'][0],actual)

def test_assisted_motion_uses_actual_state_without_friction_preload_or_joint_relaxation():
    r=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    for p in r['phases']:
        for s in [p.get('local_skill',{}),*p.get('local_skills',{}).values()]:
            if s.get('requires_held_object'):
                assert s['ik_reference_mode']=='hybrid'
                assert not s.get('beam_coordinated')
                assert not s.get('inherit_grasp_drive_target')
                assert not s.get('grasp_drive_smooth_transport')
                assert not s.get('track_published_grasp_command')
                assert not any(k.startswith('relax_fixed_attachment_') for k in s)


def test_assisted_runtime_rejects_hidden_pose_velocity_or_collision_bypasses():
    t=BeamAssistedState();t._beam_assisted_enabled=True
    for operation in ['pose write','velocity reset','collision disable','kinematic toggle']:
        with pytest.raises(RuntimeError,match='forbids'):
            t._assisted_forbid_body_rewrite('fabrica_beam_6',operation)
    t._beam_assisted_enabled=False
    t._assisted_forbid_body_rewrite('fabrica_beam_6','pose write')


def test_assisted_release_waits_for_all_six_gripper_joints():
    from types import SimpleNamespace as NS
    q=np.zeros(12);q[9]=.2
    t=BeamAssistedState();t.robots={'right':NS(articulation=NS(get_joint_positions=lambda:q.copy()))}
    phase={'lock':[{'object':'fabrica_beam_6'}],'gripper_commands':{'right':'open'}}
    assert not t._assisted_release_open(phase)
    q[9]=.04
    assert t._assisted_release_open(phase)


def test_loaded_servo_accumulates_bounded_progress_when_actual_state_sags():
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    a=UR5eAssemblyAtomicSkillAdapter({})
    actual={'position':np.array([0.,0.,1.]),'orientation':np.array([1.,0.,0.,0.])}
    goal={'position':np.array([0.,0.,1.14]),'orientation':np.array([1.,0.,0.,0.])}
    commands=[]
    for i in range(40):
        # Observation stays still under load; accepted commands must build the
        # finite lead needed for gravity compensation, without old phase caches.
        c=a._cartesian_command_servo_target_pose(phase_key=('lift',), current_pose=actual,
            target_pose=goal,max_position_step=.0002,max_orientation_step=.0005,
            spec={'beam_assisted':True,'requires_held_object':True})
        a._remember_cartesian_command_position(phase_key=('lift',),command_target_pose=c)
        commands.append(c['position'][2])
    assert commands[5]>commands[0]+.0009
    assert max(commands)<=1.003+1e-12
    other=a._cartesian_command_servo_target_pose(phase_key=('next',),current_pose=actual,
        target_pose=goal,max_position_step=.0002,max_orientation_step=.0005,
        spec={'beam_assisted':True,'requires_held_object':True})
    assert other['position'][2]<1.00021


def test_install_alignment_waits_until_complete_native_gripper_is_open():
    from types import SimpleNamespace as NS
    q=np.zeros(12);q[9]=.2
    t=BeamAssistedState();t.failed=False
    t.robots={'right':NS(articulation=NS(get_joint_positions=lambda:q.copy()))}
    anchor=np.array([.1,.2,.3])
    t._assisted_install_alignment={'part':{'complete':False,'release_robot':'right','position':anchor.copy()}}
    t._assisted_update_install_alignment()
    np.testing.assert_array_equal(t._assisted_install_alignment['part']['position'],anchor)


def test_assisted_object_transport_can_compute_orientation_before_position():
    from types import SimpleNamespace as NS
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    a=UR5eAssemblyAtomicSkillAdapter({})
    current={'position':np.array([0.,0.,1.2]),'orientation':np.array([1.,0.,0.,0.])}
    spec={'beam_assisted':True,'requires_held_object':True,'object':'part',
          'servo_target_object_pose':True,'target_object_position':[0.,0.,1.1],
          'target_object_orientation':[1.,0.,0.,0.],
          'cartesian_position_step':.001,'cartesian_orientation_step':.004,
          'target_object_servo_position_command_warm_start':True,
          'target_object_servo_position_command_lookahead':.003}
    command=a._target_object_servo_pose(phase_key=('pickup_clearance',),task=NS(),
        robot_name='right',spec=spec,tracked_robots={},
        tracked_objects={'part':{'position':[0.,0.,1.],
            'orientation':[1.,0.,0.,0.],'attachment':{'mode':'fixed_joint'}}},
        current_pose=current,target_pose={'position':[0.,0.,1.3],'orientation':[1.,0.,0.,0.]})
    assert np.all(np.isfinite(command['position']))
    assert 0 < command['position'][2]-current['position'][2] <= .003


@pytest.mark.parametrize('layout_seed', [0, 1, 505, 777])
def test_assisted_loaded_insert_path_stops_above_capture_without_reverse_descent(layout_seed):
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    from types import SimpleNamespace as NS
    config=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',
        scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=layout_seed,
        domain_randomization_enabled=True)
    adapter=UR5eAssemblyAtomicSkillAdapter({})
    task=NS(target_poses=config.target_poses)
    for pid in ('3','2','1','0'):
        heights=[]
        for phase in config.phase_specs:
            spec=phase.get('local_skill',{});name=spec.get('target_object_target','')
            if name.startswith('part_'+pid+'_insert_'):
                goal=adapter._target_object_pose(task=task,spec=spec)
                heights.append(float(goal['position'][2]))
                assert spec['target_object_servo_position_command_gate_overdrive'] is False
                gate=np.asarray(goal['position'],float)
                current=gate+np.array([0.,0.,.0002])
                key=('regression-cached-target',phase['name'])
                adapter._cartesian_command_positions[key]=gate.copy()
                command=adapter._accumulated_cartesian_command_position(
                    phase_key=key,current_position=current,
                    one_step_position=current-np.array([0.,0.,.0002]),
                    gate_position=gate,enabled=True,lookahead=.003,
                    allow_gate_overdrive=spec['target_object_servo_position_command_gate_overdrive'],
                    accumulation_step=.0002)
                # Old True crossed below the valid target's capture height.
                np.testing.assert_allclose(command,gate,atol=1e-12)
        capture=float(config.target_poses['part_'+pid+'_assembled']['position'][2])+.003
        assert heights
        assert all(a+1e-12>=b for a,b in zip(heights,heights[1:]))
        assert all(h+1e-12>=capture for h in heights)
        assert abs(heights[-1]-capture)<1e-12


@pytest.mark.parametrize('layout_seed', [0, 1, 505, 777])
def test_assisted_capture_and_tcp_use_runtime_randomized_assembly_target(layout_seed):
    from types import SimpleNamespace as NS
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    config=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',
        scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=layout_seed,
        domain_randomization_enabled=True)
    adapter=UR5eAssemblyAtomicSkillAdapter({})
    task=NS(target_poses=copy.deepcopy(config.target_poses))
    assert not any(name.endswith('_assisted_capture') for name in task.target_poses)
    captures=[]
    for phase in config.phase_specs:
        spec=phase.get('local_skill',{});name=spec.get('target_object_target','')
        if not name.endswith('_assembled'):
            continue
        assert spec['target_object_offset_frame']=='world'
        assert spec['target_object_position_tolerance']==.001
        assert spec['target_object_servo_position_command_gate_overdrive'] is False
        final=task.target_poses[name]
        expected=np.asarray(final['position'])+np.array([0.,0.,.003])
        # Exercise the production runtime resolver, after the episode builder
        # has applied the actual layout translation and workspace offset.
        goal=adapter._target_object_pose(task=task,spec=spec)
        np.testing.assert_allclose(goal['position'],expected,atol=1e-12)
        np.testing.assert_allclose(goal['orientation'],final['orientation'],atol=1e-12)
        relative=np.array([0.,0.,-.09])
        tracked={spec['object']:{'position':final['position'],
            'orientation':final['orientation'],'attachment':{
                'robot_name':spec['robot'],'mode':'fixed_joint',
                'position':relative.tolist(),'orientation':[1.,0.,0.,0.],
                'attach_step':1,'joint_path':'/test/grasp'}}}
        tcp=adapter._target_pose(task=task,robot_name=spec['robot'],spec=spec,
            tracked_robots={},tracked_objects=tracked)
        from toolkits.factory_dual_franka_assembly.planner_primitives import quat_rotate
        np.testing.assert_allclose(tcp['position'],
            expected-quat_rotate(tcp['orientation'],relative),atol=1e-12)
        # Resolving twice cannot accumulate an offset on the named target.
        np.testing.assert_allclose(adapter._target_object_pose(task=task,spec=spec)['position'],expected,atol=1e-12)
        np.testing.assert_array_equal(task.target_poses[name]['position'],config.target_poses[name]['position'])
        captures.append(spec['object'])
    assert sorted(captures)==sorted(c['object'] for c in config.success_criteria)


def _production_final_evaluator(config):
    """Execute the task's actual condition methods without starting Isaac."""
    import ast
    from pathlib import Path
    from types import SimpleNamespace as NS
    from toolkits.factory_dual_franka_assembly.planner_primitives import pose_within_tolerance
    source=Path(__file__).resolve().parents[2]/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
    names={'_evaluate_advance_condition','_object_targets_reached','_objects_static','_target_tolerances'}
    tree=ast.parse(source.read_text())
    nodes=[n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name in names]
    assert {n.name for n in nodes}==names
    scope={'pose_within_tolerance':pose_within_tolerance}
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes,type_ignores=[])),str(source),'exec'),scope)
    task=type('FinalEvaluator',(BeamAssistedState,),{name:scope[name] for name in names})()
    task.phase_step_counter=0
    task._beam_assisted_enabled=True
    task._attachments={}
    task._beam_continuing_stability=lambda:False
    # This test checks the real compiled condition interface. Native consecutive
    # clock qualification and release readback are tested by the state tests.
    task._beam_joint_conditions_stable=lambda advance,ready:ready
    task._assisted_conditions_stable=lambda advance,ready:ready
    task._beam_physics_stamp=lambda:{'valid':True,'epoch':0,'index':240,
                                    'time':1.,'dt':1/240.}
    task._as_list=lambda value:value if isinstance(value,list) else [value]
    task._extract_object_name=lambda value:value.get('object') if isinstance(value,dict) else value
    task.poses={c['object']:copy.deepcopy(config.target_poses[c['target']]) for c in config.success_criteria}
    # The production static-condition branch consumes a same-clock native
    # installation sample. Use the real BeamAssistedState membership/freshness
    # helper; detailed SDK readback and 240-tick drift tests live in state tests.
    task._assisted_install_gate_sample={'ready':True,'physics_stamp':task._beam_physics_stamp(),
        'objects':{name:{**copy.deepcopy(pose),'ready':True}
                   for name,pose in task.poses.items()}}
    task._resolve_object=lambda name:NS(get_pose=lambda:(task.poses[name]['position'],task.poses[name]['orientation']))
    task._resolve_target_pose_spec=lambda name:(name,config.target_poses[name]['position'],config.target_poses[name]['orientation'],{})
    task._object_velocity_metrics=lambda name,**kwargs:{'valid':True,'is_static':True}
    return task


@pytest.mark.parametrize('failure',['position','orientation','native_pose_missing','attached'])
def test_assisted_final_compiled_predicate_uses_supported_evaluator_and_checks_every_part(failure):
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    config=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',
        scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=505,
        domain_randomization_enabled=True)
    final=config.phase_specs[-1]
    assert final['name']=='beam_assisted_final_verification'
    assert final['advance']['physics_stable_steps']==240
    objects=[c['object'] for c in config.success_criteria]
    assert sorted(final['beam_assisted_installed_parts'])==sorted(objects)
    for name in objects:
        task=_production_final_evaluator(config)
        assert task._evaluate_advance_condition(phase_spec=final,advance=final['advance'])
        if failure=='position':
            task.poses[name]['position']=np.asarray(task.poses[name]['position'])+[.00301,0.,0.]
        elif failure=='orientation':
            task.poses[name]['orientation']=[np.cos(.0401/2),0.,0.,np.sin(.0401/2)]
        elif failure=='native_pose_missing':
            task._assisted_install_gate_sample['objects'].pop(name)
        else:
            task._attachments[name]={'robot_name':'franka_left'}
        assert not task._evaluate_advance_condition(phase_spec=final,advance=final['advance']), (name,failure)


def test_assisted_full_recipe_budget_includes_all_phases_and_final_verification():
    recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    assert recipe['max_steps']>=sum(p['timeout_steps'] for p in recipe['phases'])+1000
    assert recipe['max_steps']>30000


@pytest.mark.parametrize('eye,target,up',[
    ([.4,.3,1.2],[.45,.2,1.3],[0.,0.,1.]),
    ([0.,0.,0.],[0.,0.,1.],[0.,0.,1.])])
def test_native_camera_world_transform_projects_target_to_image_center(eye,target,up):
    from toolkits.factory_dual_franka_assembly.beam_assisted_camera import usd_camera_world_matrix
    W=usd_camera_world_matrix(eye,target,up);local=np.linalg.inv(W)@np.r_[target,1.]
    np.testing.assert_allclose(local[:2],0.,atol=1e-12)
    assert local[2]<0
    np.testing.assert_allclose(W[:3,:3].T@W[:3,:3],np.eye(3),atol=1e-12)
