"""Counterexamples from A12/A13 plus native-clock/final-publication checks."""
import ast
from functools import lru_cache
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from toolkits.factory_dual_franka_assembly.beam_assisted_servo import (
    absolute_object_servo, anchored_object_prediction, descent_admission,
    object_pose_from_tcp, published_object_command_safe, validate_model_native_residual,
)
from toolkits.factory_dual_franka_assembly.beam_assisted_state import BeamAssistedState
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
from toolkits.factory_dual_franka_assembly.planner_primitives import (
    compose_pose, normalize_quat, quat_conjugate, quat_multiply, relative_pose,
)


def test_empty_return_native_clamp_retains_raw_home_branch_and_default_wrap_behavior():
    from toolkits.factory_dual_franka_assembly.beam_assisted_state import bounded_native_target
    actual = np.zeros(6); actual[-1] = -2*np.pi
    target = np.zeros(6)
    legacy = bounded_native_target(target, actual)
    assert np.allclose(legacy, actual, atol=1e-12)
    assert np.array_equal(legacy, bounded_native_target(target, actual, unwrap=True))
    raw = bounded_native_target(target, actual, unwrap=False)
    assert raw[-1] == pytest.approx(actual[-1]+.02)
    assert np.max(abs(raw-actual)) <= .02+1e-12


def _native_joint_target_fixture():
    adapter = Adapter({}); q = np.zeros(6); tick = [1]
    task = NS(phase_step_counter=0, step_counter=1,
              _beam_physics_stamp=lambda: stamp(tick[0]))
    adapter._current_arm_q = lambda *a: q.copy()
    adapter._command_reference_q = lambda **kw: q.copy()
    adapter._last_command_q = lambda **kw: q.copy()
    adapter._remember_arm_command = lambda *a: None
    adapter._gripper_command_value = lambda **kw: 1.
    adapter._debug_grasp_enabled = lambda: False
    completed = []
    adapter._mark_complete = lambda **kw: completed.append(kw['detail'])
    spec = dict(joint_positions=[0.]*6, unwrap_revolute_joints=False,
                joint_position_tolerance=.01, joint_target_stable_steps=8,
                beam_assisted_native_joint_target=True)
    def act():
        return adapter._move_arm_to_joint_positions_action(phase_key=('native_home',),
            task=task, robot_name='left', skill_name='move_arm_to_joint_positions', spec=spec)
    return adapter, task, q, tick, spec, completed, act


def test_home_joint_skill_counts_eight_real_ticks_and_rejects_missing_clock():
    adapter, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    for _ in range(100):
        act()
    assert not completed
    assert adapter._completion_state[('native_home',)]['joint_target_stable_steps'] == 1
    for i in range(2, 9):
        tick[0] = i; act()
    assert completed[-1]['stable_steps'] == 8
    _, task, _, _, _, completed, act = _native_joint_target_fixture()
    del task._beam_physics_stamp
    for _ in range(20):
        act()
    assert not completed


def test_home_joint_skill_resets_after_outside_tolerance_or_clock_gap():
    adapter, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    for i in range(1, 8):
        tick[0] = i; act()
    q[0] = .02; tick[0] = 8; act()
    assert adapter._completion_state[('native_home',)]['joint_target_stable_steps'] == 0
    q[0] = 0.; tick[0] = 9; act()
    tick[0] = 11; act()
    assert adapter._completion_state[('native_home',)]['joint_target_stable_steps'] == 1
    assert not completed
    for i in range(12, 19):
        tick[0] = i; act()
    assert completed[-1]['stable_steps'] == 8


def test_native_delta_joint_target_captures_entry_once_and_moves_only_selected_axis():
    adapter, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    q[:] = [.2,-1.2,1.,-.7,-.8,.3]
    entry=q.copy();delta=np.zeros(6);delta[1]=-.24
    spec.update(beam_assisted_joint_target_delta=delta.tolist(),
                joint_positions=[999.]*6, max_joint_step=.01, max_command_joint_step=.01,
                limit_command_to_measured_state=True, max_command_tracking_error=.03,
                max_wrist_command_tracking_error=.03)
    first=np.asarray(act()['arm_joint_controller'][0])
    assert first[1]==pytest.approx(entry[1]-.01)
    assert np.array_equal(first[[0,2,3,4,5]],entry[[0,2,3,4,5]])
    cached=adapter._completion_state[('native_home',)]['native_delta_target']
    assert np.array_equal(cached['entry_joint_positions'],entry)
    assert np.allclose(cached['target_joint_positions'],entry+delta)
    q[1]-=.10
    for _ in range(20):
        act()
    assert cached is adapter._completion_state[('native_home',)]['native_delta_target']
    assert np.array_equal(cached['entry_joint_positions'],entry)
    assert np.allclose(cached['target_joint_positions'],entry+delta)
    q[:]=entry+delta
    for i in range(2,10):
        tick[0]=i;act()
    assert completed[-1]['native_delta_target']['entry_joint_positions']==entry.tolist()
    assert completed[-1]['target_joint_positions']==pytest.approx((entry+delta).tolist())
    # A different real phase entry gets its own measured reference.
    new_entry=q.copy()
    adapter._move_arm_to_joint_positions_action(phase_key=('second_phase',),task=task,
        robot_name='left',skill_name='move_arm_to_joint_positions',spec=spec)
    second=adapter._completion_state[('second_phase',)]['native_delta_target']
    assert second is not cached
    assert second['entry_joint_positions']==new_entry.tolist()
    assert second['target_joint_positions']==pytest.approx((new_entry+delta).tolist())


@pytest.mark.parametrize('bad_delta',[[0.]*5,[0.,float('nan'),0.,0.,0.,0.],
    [0.,float('inf'),0.,0.,0.,0.],None,'invalid'])
def test_native_delta_target_rejects_invalid_or_changed_input(bad_delta):
    adapter, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    spec['beam_assisted_joint_target_delta']=bad_delta
    action=act()
    assert action['__local_skill_failure__'] is True
    assert 'arm_joint_controller' not in action
    assert not completed


def test_native_delta_target_rejects_mutation_within_phase_and_requires_native_flag():
    adapter, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    spec['beam_assisted_joint_target_delta']=[0.,-.24,0.,0.,0.,0.]
    act()
    spec['beam_assisted_joint_target_delta'][1]=-.3
    assert act()['__local_skill_failure__'] is True
    _, task, q, tick, spec, completed, act = _native_joint_target_fixture()
    spec['beam_assisted_joint_target_delta']=[0.]*6
    spec['beam_assisted_native_joint_target']=False
    assert act()['__local_skill_failure__'] is True


def _empty_final_batch_fixture(allowed=True):
    task = BeamAssistedState(); tick = [10]; events = []; checks = []
    task.step_counter = 10; task.phase_entry_step = 0
    q = {name: np.zeros(12) for name in ('left','right')}
    task.robots = {name: NS(config=NS(gripper_open_position=0.,
        initial_joint_positions={'finger_joint':0.}),
        articulation=NS(get_joint_positions=lambda name=name:q[name].copy()),
        controllers={'arm_joint_controller':NS(get_joint_subset=lambda name=name:
            NS(get_joint_positions=lambda:q[name][:6].copy()))}) for name in q}
    phase = dict(beam_assisted_empty_return_guard=True, beam_assisted_native_joint_limits=True,
        beam_assisted_installed_parts=['fabrica_beam_'+n for n in ('6','3','2','1','0')])
    task.get_current_phase_spec = lambda: phase
    task._attachments = {}
    task._beam_physics_stamp = lambda: stamp(tick[0])
    task._assisted_arm_joint_limits = lambda name: np.tile([-2*np.pi,2*np.pi],(6,1))
    task._assisted_event = lambda event, **kw: events.append((event,kw))
    def guard(**kw):
        checks.append((kw,{n:v.copy() for n,v in task._assisted_batch_arm_candidates.items()}))
        return allowed, dict(passed=allowed,mode='empty_return',physics_stamp=kw['physics_stamp'])
    task._assisted_production_cad_guard = guard
    actions = {name:dict(arm_joint_controller=[[.1]*6],gripper_controller=[1.]) for name in q}
    return task,q,tick,phase,checks,events,actions


def test_empty_return_guard_receives_both_actual_final_clamped_targets_once():
    task,q,tick,phase,checks,events,actions = _empty_final_batch_fixture()
    received = task._assisted_guard_actions(actions)
    assert len(checks)==1 and checks[0][0]['empty'] is True
    assert checks[0][0]['physics_stamp']==stamp(tick[0])
    assert set(checks[0][1])==set(task.robots)
    for name in task.robots:
        assert np.allclose(checks[0][1][name],.02)
        assert np.array_equal(checks[0][1][name],received[name]['arm_joint_controller'][0])


@pytest.mark.parametrize('invalid',['missing_arm','missing_target','attachment','closed_native',
    'close_command','clock','missing_guard','collision','clock_changed','nonzero_open'])
def test_empty_return_unsafe_batch_does_not_reach_sdk_receiver(invalid):
    task,q,tick,phase,checks,events,actions = _empty_final_batch_fixture(allowed=invalid!='collision')
    if invalid=='missing_arm':del actions['right']
    elif invalid=='missing_target':del actions['right']['arm_joint_controller']
    elif invalid=='attachment':task._attachments={'part':dict(robot_name='left')}
    elif invalid=='closed_native':q['right'][9]=.1
    elif invalid=='close_command':actions['left']['gripper_controller']=[0.]
    elif invalid=='clock':task._beam_physics_stamp=lambda:dict(valid=False)
    elif invalid=='missing_guard':task._assisted_production_cad_guard=None
    elif invalid=='nonzero_open':task.robots['right'].config.gripper_open_position=.1
    elif invalid=='clock_changed':
        def moving_guard(**kw):
            tick[0]+=1
            return True,dict(passed=True)
        task._assisted_production_cad_guard=moving_guard
    sdk_received=[]
    with pytest.raises(ValueError):
        sdk_received.append(task._assisted_guard_actions(actions))
    assert not sdk_received


@pytest.mark.parametrize('wrist', [-2.34, -2*np.pi+.01, 2*np.pi-.01])
def test_absolute_prealign_does_not_accept_timed_out_or_modulo_equivalent_branch(wrist):
    adapter=Adapter({});q=np.array([-1.9,-np.pi/2,np.pi/2,-np.pi/2,-np.pi/2,wrist])
    task=NS(phase='assemble_02_part_1_move_above',phase_step_counter=240,step_counter=31000)
    adapter._current_arm_q=lambda *a:q.copy()
    adapter._command_reference_q=lambda **kw:q.copy()
    adapter._current_arm_dynamics=lambda **kw:dict(joint_velocity=np.zeros(6))
    adapter._gripper_command_value=lambda **kw:1.
    adapter._debug_grasp_enabled=lambda:False
    goal=q.copy();goal[-1]=0.
    spec=dict(prealign_steps=240,prealign_joint_positions=goal.tolist(),
        prealign_max_joint_step=.035,prealign_until_converged=True,
        prealign_timeout_steps=3600,prealign_joint_position_tolerance=.05,
        unwrap_revolute_joints=False,max_command_tracking_error=.12)
    args=dict(phase_key=(1,),task=task,robot_name='left',target_pose=pose(),spec=spec)
    action=adapter._prealign_action(**args)
    assert action is not None and '__local_skill_failure__' not in action
    command=np.asarray(action['arm_joint_controller'][0])
    assert abs(command[-1])<abs(q[-1]) and np.max(abs(command-q))<=.035+1e-12
    assert not adapter._prealign_state[(1,)].get('converged')
    # Arrival is measured on the requested raw native branch, not modulo2pi.
    q[-1]=0.;task.phase_step_counter=241
    assert adapter._prealign_action(**args) is None
    assert adapter._prealign_state[(1,)]['converged']


def _native_limit_fixture(wrist=-6.28):
    task=BeamAssistedState();task.step_counter=10;task.phase_entry_step=0;q=np.zeros(6);q[-1]=wrist
    names=['finger_joint','wrist_3_joint','elbow_joint','shoulder_pan_joint',
           'wrist_1_joint','shoulder_lift_joint','wrist_2_joint']
    limits=np.tile(np.array([-2*np.pi,2*np.pi],dtype=np.float32),(len(names),1))
    limits[names.index('elbow_joint')]=[-np.pi,np.pi]
    view=NS(is_physics_handle_valid=lambda:True,get_dof_limits=lambda:limits[None].copy())
    articulation=NS(dof_names=names,_articulation_view=view)
    task.robots={'left':NS(articulation=articulation,controllers={'arm_joint_controller':
        NS(get_joint_subset=lambda:NS(get_joint_positions=lambda:q.copy()))})}
    task.get_current_phase_spec=lambda:dict(beam_assisted_native_joint_limits=True)
    task._beam_physics_stamp=lambda:stamp(10)
    task._assisted_published_q={'left':q.copy()}
    task._assisted_event=lambda *a,**kw:None
    return task,q,view,limits


def test_native_limit_guard_rejects_A36_unreachable_final_target_before_publication():
    task,q,view,limits=_native_limit_fixture();before=task._assisted_published_q['left'].copy()
    requested=q.copy();requested[-1]=-6.302875299436907
    with pytest.raises(ValueError,match='actual native joint limits'):
        task._assisted_guard_actions({'left':{'arm_joint_controller':[requested.tolist()]}})
    np.testing.assert_array_equal(task._assisted_published_q['left'],before)


def test_native_limit_guard_accepts_exact_SDK_float32_boundary_without_modulo():
    task,q,view,limits=_native_limit_fixture(float(np.float32(-2*np.pi)))
    out=task._assisted_guard_actions({'left':{'arm_joint_controller':[q.tolist()]}})
    np.testing.assert_array_equal(out['left']['arm_joint_controller'][0],q)
    assert task._assisted_arm_joint_limits('left')[2,0]==np.float32(-np.pi)


@pytest.mark.parametrize('invalid',['dead','shape','nonfinite','inverted','missing_name'])
def test_native_limit_guard_fails_closed_on_unreadable_or_ambiguous_limits(invalid):
    task,q,view,limits=_native_limit_fixture()
    if invalid=='dead':view.is_physics_handle_valid=lambda:False
    if invalid=='shape':view.get_dof_limits=lambda:limits
    if invalid=='nonfinite':limits[1,0]=np.nan
    if invalid=='inverted':limits[1]=[1,0]
    if invalid=='missing_name':task.robots['left'].articulation.dof_names[1]='unknown_joint'
    with pytest.raises(ValueError,match='native.*limit|joint limit'):
        task._assisted_guard_actions({'left':{'arm_joint_controller':[q.tolist()]}})


def test_native_limit_guard_refreshes_changed_view_and_rejects_new_range():
    task,q,view,limits=_native_limit_fixture(-1.)
    task._assisted_arm_joint_limits('left')
    changed=limits.copy();changed[1]=[-.5,.5]
    task.robots['left'].articulation._articulation_view=NS(
        is_physics_handle_valid=lambda:True,get_dof_limits=lambda:changed[None])
    with pytest.raises(ValueError,match='actual native joint limits'):
        task._assisted_guard_actions({'left':{'arm_joint_controller':[q.tolist()]}})


def test_assisted_remaining_pickups_require_raw_branch_arrival_with_original_phase_budget():
    from roboassemblybench.core.task_registry import load_task_recipe
    r=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    assert all(p['beam_assisted_native_joint_limits'] is True for p in r['phases'])
    for name in ['assemble_02_part_1_move_above','assemble_03_part_0_move_above']:
        p=next(p for p in r['phases'] if p['name']==name);s=p['local_skill']
        assert s['prealign_until_converged'] and s['unwrap_revolute_joints'] is False
        assert s['prealign_timeout_steps']==p['timeout_steps']==3600
        assert s['prealign_joint_positions'][-1]==0.
        assert s['prealign_max_joint_step']==.035 and s['max_wrist_command_tracking_error']==.10


def _handoff_chain(enabled=True, gated=False, alignment=False, goal_position=None):
    """Real adapter and final task guard; explicit linear-FK test double."""
    adapter, task = Adapter({}), BeamAssistedState()
    tick, measured = [100], np.zeros(6)
    spec = dict(name='ur5e_move_to_pose', robot='left', object='part',
                requires_held_object=True, beam_assisted_absolute_object_servo=True,
                cartesian_position_step=.001, cartesian_orientation_step=.004)
    if enabled:spec['beam_assisted_phase_handoff_ticks'] = 24
    if gated:spec['beam_assisted_descent_admission'] = contract()
    if alignment:
        spec['beam_assisted_alignment_hold'] = dict(maximum_lateral_compensation_m=.00012,
            maximum_integral_step_m=.000002, integral_gain_per_native_tick=.02,
            cold_actual_axial_limit_m=.0005)
    scale=np.array([1.,1.,.5 if gated else 1.])
    state = {}
    task.phase_index, task.phase_entry_step, task.step_counter = 1, 100, 100
    task._beam_physics_stamp = lambda: stamp(tick[0])
    task._assisted_published_q = {'left': np.array([0., 0., .003, 0., 0., 0.])}
    if gated:task._assisted_published_q['left'][2]=-.006
    task._assisted_published_q_stamps = {'left': dict(epoch=0, index=99)}
    task._attachments = {'part': dict(robot_name='left', attach_spec={'beam_assisted':True},
                                     assisted_hold_openness=.3)}
    task.get_current_phase_spec = lambda: {'local_skill':spec}
    task.robots = {'left':NS(controllers={'arm_joint_controller':NS(get_joint_subset=lambda:
        NS(get_joint_positions=lambda:measured.copy()))})}
    events = []
    task._assisted_event = lambda event, **fields:events.append((event,fields))
    current = lambda:pose(measured[:3]*scale+[0,0,1])
    adapter._assisted_object_servo_context = lambda **kw:('part',stamp(tick[0]),
        native(current()),current(),(np.zeros(3),[1,0,0,0]),state)
    adapter._current_arm_q = lambda *a:measured.copy()
    adapter._grasp_command_fk_pose = lambda **kw:pose(np.asarray(kw['command_q'])[:3]*scale+[0,0,1])
    goal = pose(goal_position) if goal_position is not None else pose((.002,0,.9)) if gated else pose((0,0,1.3))
    adapter._target_object_pose = lambda **kw:goal
    context = dict(adapter=adapter,spec=spec,phase_key=(1,),phase_index=1,
                   phase_entry_step=100,tracked_robots={},tracked_objects={})
    task._beam_assisted_command_contexts = {'left':context}
    def act():
        task.step_counter=tick[0]
        command=adapter._assisted_absolute_object_servo_pose(phase_key=(1,),task=task,
            robot_name='left',spec=spec,tracked_robots={},tracked_objects={},
            object_pose=current(),target_object_pose=goal)
        requested=np.r_[(command['position']-[0,0,1])/scale,np.zeros(3)]
        result=task._assisted_guard_actions({'left':{'arm_joint_controller':[requested.tolist()]}})
        return np.asarray(result['left']['arm_joint_controller'][0])
    return adapter,task,spec,state,tick,measured,events,act


@pytest.mark.parametrize('response',[.02,.08,.2,1.])
def test_cold_hold_with_persistent_biased_tracking_advances_only_after_actual_alignment(response):
    outcomes=[]
    for enabled in (False,True):
        _,task,_,state,tick,measured,_,act=_handoff_chain(False,True,enabled,(0.,0.,.9))
        bias=np.array([.00008,-.00018,.00064,0.,0.,0.])
        measured[:2]=-bias[:2]
        ready=False;minimum_height=1.
        for n in range(320):
            result=act()
            if state['descent_ready']:
                ready=True
                break
            measured[:]+=response*(result-bias-measured)
            minimum_height=min(minimum_height,1.+.5*measured[2])
            tick[0]+=1
        outcomes.append((ready,minimum_height,n))
        if enabled:
            assert ready and state['admission']['count']>=12
            assert np.linalg.norm(measured[:2])<=.00015
            assert minimum_height>=1.-.0005-1e-12
            assert np.linalg.norm(state['alignment_reference']['lateral_bias'])<=.00012+1e-12
    assert outcomes[0][0] is False and outcomes[0][1]<1.-.001
    assert outcomes[1][0] is True


def test_cold_actual_height_guard_rejects_unmodeled_motion_before_next_publication():
    _,_,_,_,tick,measured,events,act=_handoff_chain(False,True,True,(0.,0.,.9))
    measured[:2]=[.00008,.00018]
    act();count=len(events)
    tick[0]+=1;measured[2]-=.001002
    with pytest.raises(ValueError,match='fixed world-height envelope'):act()
    assert events[-1][0]=='cold_actual_height_violation'
    assert not [e for e in events[count:] if e[0]=='published_command']


def test_alignment_compensation_counts_distinct_native_ticks_and_rejects_changed_sameclock_pose():
    _,_,_,state,tick,measured,_,act=_handoff_chain(False,True,True,(0.,0.,.9))
    measured[:2]=[.00008,.00018]
    act();tick[0]+=1;act()
    bias=state['alignment_reference']['lateral_bias'].copy()
    for _ in range(6):act()
    np.testing.assert_array_equal(state['alignment_reference']['lateral_bias'],bias)
    measured[0]+=.00001
    with pytest.raises(ValueError,match='same native tick'):act()


def test_alignment_native_height_reference_restarts_after_admission_loss_without_changing_grasp():
    adapter,task,spec,state,tick,measured,_,act=_handoff_chain(False,True,True,(0.,0.,.9))
    settings=dict(task=task,spec=spec,state=state,stamp=stamp(tick[0]),
                  current_object=pose(),model_object=pose(),target_object=pose((0,0,.9)))
    adapter._assisted_alignment_reference(**settings,descent_ready=False)
    tick[0]+=1;settings['stamp']=stamp(tick[0]);adapter._assisted_alignment_reference(**settings,descent_ready=True)
    assert state['alignment_reference']['cold_axis_anchor'] is None
    tick[0]+=1;settings.update(stamp=stamp(tick[0]),current_object=pose((0,0,.95)),model_object=pose((0,0,.95)))
    adapter._assisted_alignment_reference(**settings,descent_ready=False)
    assert state['alignment_reference']['cold_axis_anchor']==pytest.approx(-.95)


def test_all_forty_strict_insertions_enable_hold_without_relaxing_original_limits():
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    selected=[p['local_skill'] for p in recipe['phases']
              if p.get('local_skill',{}).get('beam_assisted_alignment_hold')]
    assert len(selected)==40
    for s in selected:
        assert s['beam_assisted_descent_admission']['lateral_tolerance_m']==.00015
        assert s['beam_assisted_descent_admission']['orientation_tolerance_rad']==.001
        assert s['beam_assisted_descent_admission']['motion_metric']=='native_com_pose_difference'
        assert s['beam_assisted_descent_admission']['stable_native_ticks']==12
        assert s['beam_assisted_descent_admission']['max_linear_speed_m_s']==.02
        assert s['beam_assisted_descent_admission']['max_angular_speed_rad_s']==.2
        assert s['beam_assisted_fk_native_orientation_limit_rad']==.0015
        assert not s.get('beam_assisted_phase_handoff_ticks')


def test_first_strict_entry_final_publication_is_recorded_after_outer_guard():
    _,task,spec,_,tick,measured,events,act=_handoff_chain(False,True,True,(0.,0.,.9))
    measured[:2]=[.00008,.00018]
    task.get_current_phase_spec=lambda:dict(name='assemble_00_part_3_insert_00',local_skill=spec)
    for age in (1,95,96,97):
        tick[0]=100+age;before=len(events);result=act()
        published=[r for event,r in events[before:] if event=='published_command']
        assert bool(published)==(age<=96)
        if published:np.testing.assert_array_equal(published[0]['published_q'],result)


def test_loaded_phase_first_final_publication_preserves_preload_instead_of_reanchoring():
    _,old,_,_,_,_,old_events,old_act=_handoff_chain(False)
    _,new,_,state,_,_,events,new_act=_handoff_chain(True)
    old_previous=old._assisted_published_q['left'].copy()
    new_previous=new._assisted_published_q['left'].copy()
    assert np.max(abs(old_act()-old_previous))>.001
    np.testing.assert_allclose(new_act(),new_previous,atol=1e-12,rtol=0)
    assert state['phase_handoff_age']==0
    assert events[0][0]=='phase_handoff_seed'
    assert events[0][1]['captured_grasp_reference_changed'] is False
    assert not [e for e in old_events if e[0]=='published_command']
    published=[e[1] for e in events if e[0]=='published_command']
    assert len(published)==1
    np.testing.assert_allclose(published[0]['published_q'],new_previous,atol=1e-12,rtol=0)


def test_cold_descent_gate_unwinds_inherited_downward_lead_before_handoff():
    _,task,_,state,_,_,_,act=_handoff_chain(gated=True)
    previous=task._assisted_published_q['left'].copy()
    # 6mrad at .5m/rad is the real allowed3mm lookahead, still within tracking.
    result=act()
    assert state['phase_handoff_joint_step']==pytest.approx(.02)
    assert state['descent_ready'] is False
    assert previous[2]<0 and result[2]>=-1e-12
    assert np.max(abs(result-previous))<=.02+1e-12
    assert state['object_command']['position'][2]>=1.-1e-12


def test_loaded_handoff_warms_on_native_time_and_eventually_advances():
    _,task,_,state,tick,measured,events,act=_handoff_chain()
    previous=act()
    for _ in range(5):
        np.testing.assert_allclose(act(),previous,atol=1e-12,rtol=0)
        assert state['phase_handoff_age']==0
    for age in range(1,30):
        tick[0]=100+age
        measured[2]+=.0001
        result=act()
        assert np.max(abs(result-previous))<=state['phase_handoff_joint_step']+1e-12
        assert 0<result[2]-measured[2]<=.003+1e-12
        previous=result
    assert state['phase_handoff_joint_step']==pytest.approx(.02)
    assert len([x for x in events if x[0]=='phase_handoff_seed'])==1


@pytest.mark.parametrize('bad',['missing','stale','epoch','future','nonfinite','tracking'])
def test_loaded_handoff_rejects_invalid_final_publication(bad):
    _,task,_,_,_,_,_,act=_handoff_chain()
    if bad=='missing':task._assisted_published_q_stamps={}
    if bad=='stale':task._assisted_published_q_stamps['left']['index']=97
    if bad=='epoch':task._assisted_published_q_stamps['left']['epoch']=1
    if bad=='future':task._assisted_published_q_stamps['left']['index']=101
    if bad=='nonfinite':task._assisted_published_q['left'][0]=np.nan
    if bad=='tracking':task._assisted_published_q['left'][0]=.121
    with pytest.raises(ValueError,match='fresh final'):act()


@pytest.mark.parametrize('axis',range(6))
@pytest.mark.parametrize('sign',[-1,1])
def test_handoff_does_not_hide_six_dof_capture_relation_errors(axis,sign):
    adapter,_,_,_,_,_,events,act=_handoff_chain()
    ordinary=adapter._assisted_object_servo_context
    def biased(**kwargs):
        values=list(ordinary(**kwargs));p=pose()
        if axis<3:p['position'][axis]+=sign*.00011
        else:
            q=np.zeros(4);q[0]=np.cos(.002/2);q[axis-2]=sign*np.sin(.002/2)
            p['orientation']=q
        values[3]=p
        return tuple(values)
    adapter._assisted_object_servo_context=biased
    with pytest.raises(ValueError,match='model budget'):act()
    assert not [x for x in events if x[0]=='phase_handoff_seed']


def test_only_twelve_column_free_move_phases_enable_loaded_handoff():
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe=load_task_recipe('fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop')
    names=[p['name'] for p in recipe['phases'] if p.get('local_skill',{}).get('beam_assisted_phase_handoff_ticks')]
    assert names==[f'assemble_{i:02d}_part_{part}_{suffix}'
                  for i,part in enumerate(('3','2','1','0'))
                  for suffix in ('pickup_clearance','assembly_clearance','transport_hover')]
    assert all(p['local_skill'].get('beam_assisted_production_cad_guard') is True
               for p in recipe['phases'] if p['name'] in names)


def pose(position=(0., 0., 1.), orientation=(1., 0., 0., 0.)):
    return {'position': np.asarray(position, float), 'orientation': np.asarray(orientation, float)}


def contract():
    # Candidate geometry limits, not a substitute for the launch CAD report.
    return dict(lateral_tolerance_m=.00015, orientation_tolerance_rad=.001,
                stable_native_ticks=12, max_linear_speed_m_s=.02,
                max_angular_speed_rad_s=.2, geometry_certificate='test_fixture',
                published_axial_slack_m=.00002, published_lateral_slack_m=.00002)


def stamp(tick, epoch=0):
    return dict(valid=True, index=tick, epoch=epoch, time=tick/240., dt=1/240.)


def native(p):
    return dict(**p, motion_valid=True, physics_handle_valid=True,
                body_path='/World/part', motion_view_id=19, linear_speed=0., angular_speed=0.)


def transition_contract():
    return dict(stable_native_ticks=24, minimum_hold_native_ticks=48,
                maximum_hold_native_ticks=480, maximum_linear_speed_m_s=.002,
                maximum_angular_speed_rad_s=.02)


def test_loaded_transition_waits_for_native_quiet_window_without_counting_policy_retries():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    state = {}
    sample = native(pose())
    sample['angular_speed'] = .2
    for tick in range(60):
        ready, _ = settled_transition_ready(state, stamp(tick), sample, True, transition_contract())
        assert not ready
    sample['angular_speed'] = .01
    for _ in range(40):
        ready, detail = settled_transition_ready(state, stamp(60), sample, True, transition_contract())
        assert not ready and detail['stable_native_ticks'] == 1
    for tick in range(61, 84):
        ready, _ = settled_transition_ready(state, stamp(tick), sample, True, transition_contract())
        assert ready == (tick == 83)


def test_loaded_transition_rejects_missing_ticks_changed_sameclock_pose_and_non_native_samples():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    state = {}; sample = native(pose())
    for tick in range(100):
        ready, _ = settled_transition_ready(state, stamp(2*tick), sample, True, transition_contract())
        assert not ready
    sample['position'][0] += .00001
    ready, detail = settled_transition_ready(state, stamp(198), sample, True, transition_contract())
    assert not ready and detail['stable_native_ticks'] == 0
    sample['physics_handle_valid'] = False
    with pytest.raises(ValueError, match='live native sample'):
        settled_transition_ready(state, stamp(199), sample, True, transition_contract())


def test_loaded_transition_lost_original_pose_gate_and_changed_identity_reset_qualification():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    state = {}; sample = native(pose())
    for tick in range(49):
        ready, _ = settled_transition_ready(state, stamp(tick), sample, True, transition_contract())
    assert ready
    ready, _ = settled_transition_ready(state, stamp(49), sample, False, transition_contract())
    assert not ready
    sample['motion_view_id'] += 1
    ready, detail = settled_transition_ready(state, stamp(50), sample, True, transition_contract())
    assert not ready and detail['hold_native_ticks'] == 0 and detail['stable_native_ticks'] == 1


def test_loaded_transition_cannot_hold_forever_or_bypass_original_model_budget():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    state = {}; sample = native(pose()); sample['linear_speed'] = .01
    settled_transition_ready(state, stamp(0), sample, True, transition_contract())
    with pytest.raises(ValueError, match='native hold budget'):
        settled_transition_ready(state, stamp(480), sample, True, transition_contract())
    with pytest.raises(ValueError, match='model budget'):
        validate_model_native_residual(pose(), pose((.000101, 0, 0)))
    angle = .001501
    with pytest.raises(ValueError, match='model budget'):
        validate_model_native_residual(pose(), pose(orientation=(np.cos(angle/2), np.sin(angle/2), 0, 0)))


@pytest.mark.parametrize('bad', ['missing', 'stale', 'epoch', 'future', 'tracking'])
def test_settled_hold_uses_fresh_outer_final_target_and_rejects_bad_publications(bad):
    _, task, _, _, tick, measured, _, act = _handoff_chain()
    final = act()
    # Model a pre-clamp adapter cache which differs from the accepted target.
    task._ur5e_plumbers_last_arm_command_q = {'left': final + .002}
    np.testing.assert_array_equal(Adapter._assisted_transition_hold_target(task, 'left', measured), final)
    if bad == 'missing': task._assisted_published_q_stamps = {}
    if bad == 'stale': task._assisted_published_q_stamps['left']['index'] = tick[0]-2
    if bad == 'epoch': task._assisted_published_q_stamps['left']['epoch'] += 1
    if bad == 'future': task._assisted_published_q_stamps['left']['index'] = tick[0]+1
    if bad == 'tracking': task._assisted_published_q['left'][0] = .121
    with pytest.raises(ValueError, match='fresh final published target'):
        Adapter._assisted_transition_hold_target(task, 'left', measured)


def test_only_part1_pickup_adds_settled_transition_and_keeps_original_handoff_and_budgets():
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe = load_task_recipe('fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop')
    selected = [p for p in recipe['phases'] if p.get('local_skill', {}).get('beam_assisted_transition_settle')]
    assert len(selected) == 1 and selected[0]['name'] == 'assemble_02_part_1_pickup_clearance'
    s = selected[0]['local_skill']
    assert s['beam_assisted_transition_settle'] == dict(transition_contract(), motion_metric='native_com_pose_difference')
    # The new native gate supersedes neither old pose tolerances nor any
    # unrelated phase's existing completion settings.
    assert not s['hold_for_target_object_settle'] and not s['require_target_object_static']
    assert s['beam_assisted_phase_handoff_ticks'] == 24
    assert s['beam_assisted_fk_native_position_limit_m'] == .0001
    assert s['beam_assisted_fk_native_orientation_limit_rad'] == .0015


def com_transition_sample(position=(0., 0., 0.), orientation=(1., 0., 0., 0.)):
    sample = native(pose())
    sample.update(linear_speed=.020, angular_speed=.262, center_of_mass_valid=True,
        center_of_mass_position_frame='world', center_of_mass_orientation_order='wxyz',
        center_of_mass_position=np.asarray(position, float),
        center_of_mass_orientation=np.asarray(orientation, float))
    return sample


def test_com_quiet_rate_advances_actual_pose_without_overwriting_solver_velocity():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    contract = dict(transition_contract(), motion_metric='native_com_pose_difference')
    state = {}; sample = com_transition_sample()
    for tick in range(49):
        ready, detail = settled_transition_ready(state, stamp(tick), sample, True, contract)
        assert ready == (tick == 48)
        assert detail['linear_speed_m_s'] == .020 and detail['angular_speed_rad_s'] == .262
        assert sample['linear_speed'] == .020 and sample['angular_speed'] == .262
        if tick:
            assert detail['qualified_linear_speed_m_s'] == detail['qualified_angular_speed_rad_s'] == 0.
        for _ in range(3):
            repeated, duplicate = settled_transition_ready(state, stamp(tick), sample, True, contract)
            assert repeated == ready and duplicate['stable_native_ticks'] == detail['stable_native_ticks']


@pytest.mark.parametrize('mode', ['linear', 'angular'])
def test_com_rate_rejects_moving_pose_with_falsely_zero_reported_velocity(mode):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    contract = dict(transition_contract(), motion_metric='native_com_pose_difference')
    state = {}
    for tick in range(480):
        angle = .05*tick/240. if mode == 'angular' else 0.
        position = (.005*tick/240., 0., 0.) if mode == 'linear' else (0., 0., 0.)
        sample = com_transition_sample(position, (np.cos(angle/2.), 0., 0., np.sin(angle/2.)))
        sample.update(linear_speed=0., angular_speed=0.)
        ready, detail = settled_transition_ready(state, stamp(tick), sample, True, contract)
        assert not ready and not detail['quiet']
    with pytest.raises(ValueError, match='native hold budget'):
        settled_transition_ready(state, stamp(480), sample, True, contract)


def test_com_rate_clock_gap_restarts_hold_and_sameclock_alias_is_rejected():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    contract = dict(transition_contract(), motion_metric='native_com_pose_difference')
    state = {}; sample = com_transition_sample()
    for tick in list(range(26)) + list(range(38, 87)):
        ready, detail = settled_transition_ready(state, stamp(tick), sample, True, contract)
        assert ready == (tick == 86)
    sample['center_of_mass_orientation'] *= -1.
    ready, _ = settled_transition_ready(state, stamp(86), sample, True, contract)
    assert ready
    sample['center_of_mass_position'][0] += 1e-7
    with pytest.raises(ValueError, match='duplicate native clock'):
        settled_transition_ready(state, stamp(86), sample, True, contract)
    with pytest.raises(ValueError, match='clock reversed'):
        settled_transition_ready(state, stamp(85), sample, True, contract)


@pytest.mark.parametrize('bad', ['missing_com', 'coordinates', 'frame', 'quaternion', 'nan_position', 'nan_velocity', 'metric'])
def test_com_rate_invalid_native_measurements_never_qualify(bad):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    contract = dict(transition_contract(), motion_metric='native_com_pose_difference')
    sample = com_transition_sample()
    if bad == 'missing_com': sample['center_of_mass_valid'] = False
    if bad == 'coordinates': sample.pop('center_of_mass_position')
    if bad == 'frame': sample['center_of_mass_position_frame'] = 'local'
    if bad == 'quaternion': sample['center_of_mass_orientation'] *= 0.
    if bad == 'nan_position': sample['center_of_mass_position'][0] = np.nan
    if bad == 'nan_velocity': sample['angular_speed'] = np.nan
    if bad == 'metric': contract['motion_metric'] = 'unknown'
    with pytest.raises(ValueError):
        settled_transition_ready({}, stamp(0), sample, True, contract)


@pytest.mark.parametrize('changed', ['epoch', 'view'])
def test_com_rate_identity_change_requires_a_new_complete_quiet_window(changed):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import settled_transition_ready
    contract = dict(transition_contract(), motion_metric='native_com_pose_difference')
    state = {}; sample = com_transition_sample()
    for tick in range(49):
        ready, _ = settled_transition_ready(state, stamp(tick), sample, True, contract)
    assert ready
    if changed == 'view': sample['motion_view_id'] += 1
    for tick in range(49, 98):
        ready, detail = settled_transition_ready(state, stamp(tick, epoch=int(changed == 'epoch')), sample, True, contract)
        assert ready == (tick == 97)


def test_optional_native_com_reads_same_existing_view_and_keeps_actor_origin_separate():
    class View:
        calls = 0
        def is_physics_handle_valid(self): return True
        def get_world_poses(self, clone):
            return np.array([[1., 2., 3.]]), np.array([[np.sqrt(.5), 0., 0., np.sqrt(.5)]])
        def get_velocities(self, clone): return np.array([[.02, 0., 0., 0., 0., .262]])
        def get_coms(self, clone):
            self.calls += 1
            return np.array([[.1, 0., 0.]]), np.array([[1., 0., 0., 0.]])
    view = View(); path = '/World/part'; reader = NS(prim_path=path, _rigid_prim_view=view)
    task = BeamAssistedState(); task._assisted_body_path = lambda name: (reader, path)
    original = task._assisted_native_body_motion('part', 0)
    assert view.calls == 0 and 'center_of_mass_position' not in original
    measured = task._assisted_native_body_motion('part', 0, include_center_of_mass=True)
    assert view.calls == 1 and measured['motion_view_id'] == id(view)
    np.testing.assert_allclose(measured['center_of_mass_position'], [1., 2.1, 3.], atol=1e-12)
    np.testing.assert_array_equal(measured['position'], original['position'])
    assert measured['linear_speed'] == original['linear_speed'] == .02
    assert measured['angular_speed'] == original['angular_speed'] == .262


def test_a12_tcp_y_sign_is_not_object_motion_direction_and_misaligned_descent_is_blocked():
    # A12 native entity step18264 and the same captured transform used by its
    # target TCP. The old command rotates a long lever arm: positive TCP Y is
    # necessary compensation, not evidence of reverse object Y motion.
    current = pose((.7840536832809448, -.11967593431472778, 1.015513300895691),
                   (.9999998735333776, .00035233848584325565, .00012579265490083905, .0003361056799722178))
    goal = pose((.7844409166438082, -.12056654334103567, 1.0058400003077836))
    relative_p = np.array([-.044290602618869065, -.000010001041940936423, .22370729041864004])
    relative_q = np.array([.0001633644480051413, -.999999961685151, .00008278977309102179, .00020757554604661834])
    old = pose((.8284748411246184, -.11971194472727269, 1.238207633015457),
               (.0001633644480051413, .999999961685151, -.00008278977309102179, -.00020757554604661834))
    old_object = object_pose_from_tcp(old, relative_p, relative_q)
    old_delta = old_object['position']-current['position']
    assert old_delta[1] < -.00009
    assert old_delta[2] < -.00099  # The actual preflight miss: descent outside the narrow corridor.
    ready, detail = descent_admission({}, stamp(1), current, goal, [0, 0, -1], contract(), native(current))
    assert not ready and detail['lateral_error_m'] > .00097
    command, object_command = absolute_object_servo(current, goal, relative_p, relative_q,
        .001, .004, convergence_axis=[0, 0, -1], allow_axial_motion=ready)
    reconstructed = object_pose_from_tcp(command, relative_p, relative_q)
    np.testing.assert_allclose(reconstructed['position'], object_command['position'], atol=1e-13)
    assert object_command['position'][2] == current['position'][2]
    assert np.linalg.norm((object_command['position']-goal['position'])[:2]) < 1e-12


@pytest.mark.parametrize('angle', [0., .0008, -.0008, .01])
def test_rotating_long_fixed_grasp_keeps_one_exact_object_tcp_transform(angle):
    current = pose((.002, -.001, 1.1), (np.cos(angle/2), np.sin(angle/2), 0., 0.))
    goal = pose((0., 0., 1.))
    relative_p, relative_q = [.04, 0., .23], [0., 1., 0., 0.]
    tcp, obj = absolute_object_servo(current, goal, relative_p, relative_q, .001, .004)
    predicted = object_pose_from_tcp(tcp, relative_p, relative_q)
    np.testing.assert_allclose(predicted['position'], obj['position'], atol=1e-14)
    assert abs(np.dot(predicted['orientation'], obj['orientation'])) == pytest.approx(1., abs=1e-14)


def test_object_lead_is_bounded_and_repeated_rotation_never_integrates_tcp_translation():
    current = pose((0., .001, 1.1), (.99999992, .0004, 0., 0.))
    goal = pose((0., 0., 1.))
    previous = None
    for _ in range(100):
        tcp, previous = absolute_object_servo(current, goal, [.04, 0., .23], [0., 1., 0., 0.],
            .001, .004, previous_object_command=previous, position_lookahead=.003,
            convergence_axis=[0, 0, -1], allow_axial_motion=False)
        assert previous['position'][2] == current['position'][2]
        assert 0. <= previous['position'][1] <= .001
        np.testing.assert_allclose(object_pose_from_tcp(tcp, [.04, 0., .23], [0., 1., 0., 0.])['position'],
                                   previous['position'], atol=1e-14)
    assert previous['position'][1] == 0.


def test_measured_sag_can_build_finite_object_lead_without_crossing_target():
    current, goal = pose(), pose((0, 0, 1.0015))
    previous = None
    for _ in range(50):
        _, previous = absolute_object_servo(current, goal, [0, 0, .23], [1, 0, 0, 0],
            .0002, .0005, previous_object_command=previous)
    assert previous['position'][2] == goal['position'][2]


def test_new_phase_has_no_old_goal_or_old_lateral_lead():
    current = pose((0., .0001, 1.1))
    old = pose((0., .003, 1.097))
    goal = pose((0., 0., 1.08))
    _, command = absolute_object_servo(current, goal, [.04, 0., .23], [0., 1., 0., 0.],
        .001, .004, previous_object_command=old, convergence_axis=[0, 0, -1], allow_axial_motion=False)
    assert command['position'][2] == current['position'][2]
    assert 0 <= command['position'][1] <= current['position'][1]


def test_policy_calls_cannot_warm_native_gate_and_gap_or_bad_alignment_resets_it():
    current, goal = pose(), pose((0, 0, .99))
    state = {}
    for _ in range(50):
        assert not descent_admission(state, stamp(1), current, goal, [0, 0, -1], contract(), native(current))[0]
    for tick in range(2, 13):
        ready, _ = descent_admission(state, stamp(tick), current, goal, [0, 0, -1], contract(), native(current))
    assert ready
    assert not descent_admission(state, stamp(14), current, goal, [0, 0, -1], contract(), native(current))[0]
    shifted = pose((.00016, 0, 1))
    assert not descent_admission(state, stamp(15), shifted, goal, [0, 0, -1], contract(), native(shifted))[0]
    assert state['count'] == 0


def test_admitted_continuous_axial_descent_keeps_lateral_gate_warm():
    current, goal, state = pose(), pose((0, 0, .9)), {}
    for tick in range(12):
        ready, _ = descent_admission(state, stamp(tick), current, goal, [0, 0, -1], contract(), native(current))
    assert ready
    for tick in range(12, 45):
        current = pose((0, 0, 1.-(tick-11)*.00015))
        motion = native(current)
        motion.update(linear_speed=.036, linear_velocity=[0, 0, -.036])
        ready, detail = descent_admission(state, stamp(tick), current, goal,
                                          [0, 0, -1], contract(), motion)
        assert ready and detail['alignment_was_warm']
    assert state['count'] == 45
    shifted = pose((.000151, 0, current['position'][2]))
    assert not descent_admission(state, stamp(45), shifted, goal,
                                  [0, 0, -1], contract(), native(shifted))[0]
    assert state['count'] == 0


def test_same_tick_changed_raw_axial_pose_cannot_hide_behind_projected_gate():
    current, goal, state = pose(), pose((0, 0, .9)), {}
    for tick in range(12):
        ready, _ = descent_admission(state, stamp(tick), current, goal, [0, 0, -1], contract(), native(current))
    assert ready
    changed = pose((0, 0, .99999))
    assert not descent_admission(state, stamp(11), changed, goal,
                                  [0, 0, -1], contract(), native(changed))[0]
    assert state['count'] == 0


@pytest.mark.parametrize('invalid', ['missing_certificate', 'bad_view', 'fast_native', 'orientation'])
def test_missing_geometry_or_native_qualification_cannot_admit_descent(invalid):
    current, goal = pose(), pose((0, 0, .99))
    c, motion = contract(), native(current)
    if invalid == 'missing_certificate':
        del c['geometry_certificate']
        with pytest.raises(ValueError): descent_admission({}, stamp(1), current, goal, [0, 0, -1], c, motion)
        return
    if invalid == 'bad_view': motion['physics_handle_valid'] = False
    if invalid == 'fast_native': motion['angular_speed'] = .21
    if invalid == 'orientation': current = pose(orientation=(np.cos(.002/2), np.sin(.002/2), 0, 0))
    state = {}
    for tick in range(15):
        assert not descent_admission(state, stamp(tick), current, goal, [0, 0, -1], c, motion)[0]


def test_joint_clamp_coupling_cannot_publish_unadmitted_descent_or_increasing_error():
    current, goal = pose((.001, 0, 1.)), pose((0, 0, .99))
    descending = pose((.0009, 0, .9999))
    safe, detail = published_object_command_safe(current, descending, goal, [0, 0, -1], contract(), False)
    assert not safe and not detail['axial_ok']
    away = pose((.00103, 0, 1.))
    assert not published_object_command_safe(current, away, goal, [0, 0, -1], contract(), False)[0]
    assert published_object_command_safe(current, pose((.0009, 0, 1.)), goal,
                                          [0, 0, -1], contract(), False)[0]


def test_admitted_command_must_stay_within_predicted_corridor():
    current, goal = pose((.0001, 0, 1.)), pose((0, 0, .99))
    assert not published_object_command_safe(current, pose((.00018, 0, .999)), goal,
                                              [0, 0, -1], contract(), True)[0]


def test_final_guard_is_after_native_clamp_and_only_this_robot_current_skill():
    t = BeamAssistedState()
    actual = np.zeros(6)
    t.step_counter = 8
    t.robots = {'left': NS(controllers={'arm_joint_controller': NS(get_joint_subset=lambda:
                NS(get_joint_positions=lambda: actual.copy()))}),
                'right': NS(controllers={'arm_joint_controller': NS(get_joint_subset=lambda:
                NS(get_joint_positions=lambda: actual.copy()))})}
    t.get_current_phase_spec = lambda: {'local_skill': {'name': 'test', 'robot': 'left',
        'requires_held_object': True, 'beam_assisted_absolute_object_servo': True}}
    observed = []
    def verify(**kwargs):
        observed.append(np.asarray(kwargs['command_q']))
        return np.zeros(6), dict(accepted_ratio=0.)
    t._beam_assisted_command_contexts = {'left': {'adapter': NS(_assisted_verify_published_held_command=verify)}}
    t._assisted_event = lambda *a, **kw: None
    result = t._assisted_guard_actions({r: {'arm_joint_controller': [np.ones(6).tolist()]} for r in t.robots})
    np.testing.assert_allclose(observed, [np.full(6, .02)])
    np.testing.assert_allclose(result['left']['arm_joint_controller'][0], actual)
    np.testing.assert_allclose(result['right']['arm_joint_controller'][0], .02)


def test_missing_current_context_is_rejected_instead_of_publishing_old_target():
    t = BeamAssistedState()
    t.step_counter = 1
    t.robots = {'left': NS(controllers={'arm_joint_controller': NS(get_joint_subset=lambda:
                NS(get_joint_positions=lambda: np.zeros(6)))})}
    t.get_current_phase_spec = lambda: {'local_skill': {'name': 'test', 'robot': 'left',
        'requires_held_object': True, 'beam_assisted_absolute_object_servo': True}}
    with pytest.raises(ValueError, match='no current'):
        t._assisted_guard_actions({'left': {'arm_joint_controller': [np.ones(6).tolist()]}})


def test_final_adapter_guard_uses_clamped_fk_and_does_not_accept_rejected_ik_cache():
    adapter = Adapter({})
    current, goal = pose((.001, 0, 1)), pose((0, 0, .99))
    state = {'descent_ready': False}
    adapter._assisted_object_servo_context = lambda **kw: ('part', stamp(1), native(current),
        current, (np.zeros(3), [1, 0, 0, 0]), state)
    adapter._target_object_pose = lambda **kw: goal
    # A synthetic coupled FK: requested positive joint lowers and aligns the
    # part. The no-descent gate must backtrack to a checked measured hold.
    adapter._grasp_command_fk_pose = lambda **kw: pose((.001-.001*kw['command_q'][0], 0,
                                                       1-.002*kw['command_q'][0]))
    context = dict(spec={'beam_assisted_descent_admission': contract()}, phase_key=(1,),
                   phase_index=1, phase_entry_step=1, tracked_robots={}, tracked_objects={})
    q, detail = adapter._assisted_verify_published_held_command(task=NS(phase_index=1, phase_entry_step=1),
        robot_name='left', context=context, command_q=np.ones(6), current_q=np.zeros(6))
    assert detail['accepted_ratio'] == 0.
    np.testing.assert_allclose(q, 0.)
    np.testing.assert_allclose(state['object_command']['position'], current['position'])
    context['phase_entry_step'] = 0
    with pytest.raises(ValueError, match='stale'):
        adapter._assisted_verify_published_held_command(task=NS(phase_index=1, phase_entry_step=1),
            robot_name='left', context=context, command_q=np.ones(6), current_q=np.zeros(6))


def test_native_anchored_fk_increment_keeps_measured_hold_exact_without_hiding_baseline():
    measured = pose((.001, 0, 1))
    model = pose((.001, 0, .99992))
    anchored = anchored_object_prediction(measured, model, model)
    np.testing.assert_allclose(anchored['position'], measured['position'], atol=1e-15)
    moved = pose((.0009, 0, .99992))
    prediction = anchored_object_prediction(measured, model, moved)
    np.testing.assert_allclose(prediction['position'], [.0009, 0, 1.], atol=1e-15)


def test_native_measured_bias_compensation_can_reach_one_milliradian_gate():
    # A12 has up to 1.174 mrad native/model orientation residual. Sending the
    # model to identity alone cannot guarantee a native 1 mrad goal. Compensate
    # that measured residual in commands, keeping the actual goal unchanged.
    model = pose()
    native_actual = pose((.00008, 0., 1.), (np.cos(.0012/2), np.sin(.0012/2), 0, 0))
    goal = pose()
    relative = ([.04, 0., .23], [0., 1., 0., 0.])
    dp, dq = validate_model_native_residual(native_actual, model)
    assert dp == pytest.approx(.00008)
    assert dq > .001 and dq < .0015
    tcp, wanted_native = absolute_object_servo(native_actual, goal, *relative, .001, .004,
        convergence_axis=[0, 0, -1], allow_axial_motion=False, measured_fk_object_pose=model)
    requested_model = object_pose_from_tcp(tcp, *relative)
    predicted_native = anchored_object_prediction(native_actual, model, requested_model)
    np.testing.assert_allclose(predicted_native['position'], goal['position'], atol=1e-14)
    assert abs(np.dot(predicted_native['orientation'], goal['orientation'])) == pytest.approx(1., abs=1e-14)
    state = {}
    for tick in range(12):
        ready, _ = descent_admission(state, stamp(tick), predicted_native, goal,
                                     [0, 0, -1], contract(), native(predicted_native))
    assert ready


@pytest.mark.parametrize('invalid', ['position', 'orientation', 'nonfinite', 'budget'])
def test_bias_compensation_rejects_residual_outside_its_independent_model_budget(invalid):
    model, native_actual = pose(), pose()
    limits = {}
    if invalid == 'position': native_actual = pose((.000101, 0, 1))
    if invalid == 'orientation': native_actual = pose(orientation=(np.cos(.0016/2), np.sin(.0016/2), 0, 0))
    if invalid == 'nonfinite': native_actual = pose((np.nan, 0, 1))
    if invalid == 'budget': limits['orientation_limit'] = float('nan')
    with pytest.raises(ValueError):
        validate_model_native_residual(native_actual, model, **limits)


def test_full_recipe_final_without_split_lateral_field_cannot_skip_cold_or_lost_native_gate():
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe = load_task_recipe('fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop')
    phase = next(p for p in recipe['phases']
                 if p.get('local_skill', {}).get('target_object_target') == 'part_3_assembled')
    spec = dict(phase['local_skill'])
    assert 'target_object_lateral_position_tolerance' not in spec
    assert spec['beam_assisted_absolute_object_servo']
    assert spec['beam_assisted_descent_admission']
    adapter = Adapter({})
    current = pose()
    state = {}
    task = NS(phase_index=7, phase_entry_step=0, phase_step_counter=0,
              target_poses={'part_3_assembled': pose((0, 0, .997))})
    tick = [0]
    adapter._assisted_object_servo_context = lambda **kw: ('fabrica_beam_3', stamp(tick[0]),
        _com_admission_sample(current), current, (np.zeros(3), [1, 0, 0, 0]), state)
    adapter._object_motion_detail = lambda **kw: dict(valid=True, is_static=True,
        linear_speed=0., angular_speed=0., pose_stable_override=False)
    completed = []
    adapter._mark_complete = lambda **kw: completed.append(tick[0])
    tracked = {'fabrica_beam_3': {**current, 'attachment': {'mode': 'fixed_joint'}}}
    for t in range(13):
        tick[0] = t
        adapter._maybe_mark_complete(phase_key=(7,), task=task, robot_name=spec['robot'],
            skill_name=spec['name'], spec=spec, target_pose=pose(), ik_target_pose=pose(),
            current_pose=pose(), tracked_objects=tracked, current_q=np.zeros(6), target_q=np.zeros(6))
        if t < 12:
            assert not completed
    assert completed == [12]
    # A shift still within the old final 1 mm bound loses the stricter current
    # corridor immediately; force-complete and old static flags cannot help.
    completed.clear()
    tick[0] = 13
    current['position'][0] = .0002
    spec['force_complete_after_steps'] = 0
    adapter._maybe_mark_complete(phase_key=(7,), task=task, robot_name=spec['robot'],
        skill_name=spec['name'], spec=spec, target_pose=pose(), ik_target_pose=pose(),
        current_pose=pose(), tracked_objects=tracked, current_q=np.zeros(6), target_q=np.zeros(6))
    assert not completed


@lru_cache(maxsize=1)
def _production_relative_frame_conversion():
    # Importing the simulator task class requires Isaac. Execute its unchanged
    # pure production conversion method instead; never replace adapter context.
    source = Path(__file__).resolve().parents[2]/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
    methods = [n for n in ast.walk(ast.parse(source.read_text()))
               if isinstance(n, ast.FunctionDef) and n.name == '_physical_relative_pose_to_control_frame']
    assert len(methods) == 1
    namespace = dict(compose_pose=compose_pose, relative_pose=relative_pose)
    exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])),
                 str(source), 'exec'), namespace)
    return namespace['_physical_relative_pose_to_control_frame']


class _LiveContextFixture:
    """Production context/servo/verifier, with explicit native/FK test inputs.

    The two control-frame observations are A13 motion steps6504/6505 from
    C01_identity_native_inputs.json (SHA4641cf05...86894). Native actor poses and
    the reversible six-coordinate FK below are test doubles, not robot dynamics
    or a geometric path certificate. The defect is in context lifetime.
    """
    frames = (
        pose((.4500080179225504, -.16675178409700164, 1.222321115214682),
             (.4184304019951088, .24027118729139282, .8739269081965567, -.05863031957787255)),
        pose((.4500478475007438, -.16674984135929893, 1.2222936090516885),
             (.4185109248616466, .24011623089981812, .8739310053026033, -.05862933907776778)),
    )
    relative = (np.array([.006526582375386364, -5.3891034882408756e-6, .17308614118824744]),
                np.array([.5174320395627138, 4.678963196813577e-5, -.8557242999118494, -6.917424455033362e-5]))

    def __init__(self, admission=False):
        self.adapter = Adapter({})
        self.task = NS(step_counter=1, phase_index=1, phase_entry_step=0)
        self.tick, self.epoch, self.body, self.view = 1, 0, '/World/part', 'existing_native_view'
        self.clock_valid, self.motion_valid, self.handle_valid = True, True, True
        self.phase_key = (1, 'held')
        self.spec = dict(object='part', target_object_target='goal',
            beam_assisted_absolute_object_servo=True, cartesian_position_step=.001,
            cartesian_orientation_step=.004, target_object_servo_position_command_lookahead=.003,
            cartesian_orientation_command_lookahead=.02)
        if admission:
            self.spec['beam_assisted_descent_admission'] = contract()
        self.attachment = dict(mode='fixed_joint', robot_name='left', joint_path='/World/part/grasp',
            attach_step=9, position=self.relative[0].copy(), orientation=self.relative[1].copy())
        self.tracked = {'part': {'attachment': self.attachment}}
        self.task._get_robot_task_pose = lambda name: (self.frame['position'], self.frame['orientation'])
        self.task._get_robot_control_pose = self.task._get_robot_task_pose
        convert = _production_relative_frame_conversion()
        self.task._physical_relative_pose_to_control_frame = lambda *args: convert(self.task, *args)
        self.task._beam_physics_stamp = lambda: {**stamp(self.tick, self.epoch), 'valid': self.clock_valid}
        self.task._assisted_native_body_motion = lambda name, epoch: dict(**self.object_pose,
            motion_valid=self.motion_valid, physics_handle_valid=self.handle_valid,
            body_path=self.body, motion_view_id=self.view, linear_speed=0., angular_speed=0.)
        self.task._assisted_event = lambda *args, **kwargs: None
        self.adapter._current_arm_q = lambda *args: np.zeros(6)
        self.adapter._grasp_command_fk_pose = lambda **kw: self.fk(kw['command_q'])
        self.advance(1)
        self.task.target_poses = {'goal': pose(self.object_pose['position']+[.01, 0, 0],
            quat_multiply([np.cos(.05), 0, 0, np.sin(.05)], self.object_pose['orientation']))}

    def advance(self, tick):
        self.tick = self.task.step_counter = tick
        # Alternate actual native-run observations. Even a stationary physical
        # grasp yields different derived floating tuples in these two frames.
        self.frame = self.frames[(tick-1) % 2]
        self.object_pose = object_pose_from_tcp(self.frame, *self.relative)

    def fk(self, q):
        q = np.asarray(q, float)
        angle = float(np.linalg.norm(q[3:]))
        rotation = np.r_[np.cos(angle/2), q[3:]*(np.sin(angle/2)/angle if angle else .5)]
        return pose(self.frame['position']+q[:3],
            quat_multiply(rotation, self.frame['orientation']))

    def inverse_fk(self, target):
        rotation = normalize_quat(quat_multiply(target['orientation'], quat_conjugate(self.frame['orientation'])))
        if rotation[0] < 0:
            rotation = -rotation
        sine = float(np.linalg.norm(rotation[1:]))
        rotvec = rotation[1:]*(2*np.arctan2(sine, rotation[0])/sine if sine else 2.)
        return np.r_[target['position']-self.frame['position'], rotvec]

    def context(self):
        # This must remain the real production method: overriding it was the
        # C01 offline preflight's missing lifecycle coverage.
        return self.adapter._assisted_object_servo_context(phase_key=self.phase_key,
            task=self.task, robot_name='left', spec=self.spec,
            tracked_robots={}, tracked_objects=self.tracked)

    def publish(self, command_q, previous_q=None):
        return self.adapter._assisted_verify_published_held_command(task=self.task, robot_name='left',
            context=dict(spec=self.spec, phase_key=self.phase_key, phase_index=self.task.phase_index,
                phase_entry_step=self.task.phase_entry_step, tracked_robots={}, tracked_objects=self.tracked),
            command_q=command_q, current_q=np.zeros(6), previous_q=previous_q)

    def admission(self):
        _, s, motion, actual, _, state = self.context()
        return descent_admission(state.setdefault('admission', {}), s, actual,
            self.task.target_poses['goal'], [0, 0, -1], contract(), motion)


def test_live_context_keeps_cache_across_actual_a13_frame_roundoff():
    f = _LiveContextFixture()
    *_, relative_a, state_a = f.context()
    sentinel = pose((.01, .02, 1.))
    state_a.update(admission={'count': 11}, object_command=sentinel)
    f.advance(2)
    *_, relative_b, state_b = f.context()
    assert not (np.array_equal(relative_a[0], relative_b[0]) and np.array_equal(relative_a[1], relative_b[1]))
    assert np.max(abs(relative_a[0]-relative_b[0])) < 1e-14
    assert state_b is state_a
    assert state_b['admission']['count'] == 11
    assert state_b['object_command'] is sentinel


@pytest.mark.parametrize('change', ['joint_path', 'attach_step', 'epoch', 'body_path', 'motion_view_id'])
def test_live_context_authoritative_change_resets_state(change):
    f = _LiveContextFixture()
    state = f.context()[-1]
    state.update(admission={'count': 12}, object_command=pose(),alignment_reference={'sentinel':True})
    if change == 'joint_path': f.attachment['joint_path'] += '_reattached'
    if change == 'attach_step': f.attachment['attach_step'] += 1
    if change == 'epoch': f.epoch += 1
    if change == 'body_path': f.body += '_replacement'
    if change == 'motion_view_id': f.view += '_replacement'
    assert set(f.context()[-1]) == {'identity'}


def test_live_context_new_phase_cannot_inherit_old_admission_or_command():
    f = _LiveContextFixture()
    old = f.context()[-1]
    old.update(admission={'count': 12}, object_command=pose(),alignment_reference={'sentinel':True})
    f.phase_key = (2, 'next_insert')
    f.task.phase_index, f.task.phase_entry_step = 2, 2
    assert f.context()[-1] is not old
    assert set(f.context()[-1]) == {'identity'}
    assert old['admission']['count'] == 12


def test_live_context_reentrant_policy_calls_cannot_warm_one_native_tick():
    f = _LiveContextFixture(admission=True)
    f.object_pose, f.task.target_poses['goal'] = pose(), pose((0, 0, .99))
    for _ in range(30):
        ready, detail = f.admission()
        assert not ready and detail['native_count'] == 1
        # Guard/completion context reentry cannot erase or increment the gate.
        assert f.context()[-1]['admission']['count'] == 1


@pytest.mark.parametrize('loss', ['lateral', 'orientation'])
def test_live_context_warms_twelve_ticks_then_loses_and_reacquires_alignment(loss):
    f = _LiveContextFixture(admission=True)
    f.task.target_poses['goal'] = pose((0, 0, .99))
    for tick in range(1, 13):
        f.advance(tick)
        f.object_pose = pose()
        ready, detail = f.admission()
        assert ready == (tick == 12)
        assert detail['native_count'] == tick
    f.advance(13)
    f.object_pose = (pose((.000151, 0, 1)) if loss == 'lateral'
                     else pose(orientation=(np.cos(.00101/2), np.sin(.00101/2), 0, 0)))
    ready, detail = f.admission()
    assert not ready and detail['native_count'] == 0
    for tick in range(14, 26):
        f.advance(tick)
        f.object_pose = pose()
        ready, detail = f.admission()
        assert ready == (tick == 25)
        assert detail['native_count'] == tick-13


@pytest.mark.parametrize('invalid', ['clock', 'motion', 'physics_handle', 'wrong_robot'])
def test_live_context_invalid_native_or_wrong_grasp_fails_closed(invalid):
    f = _LiveContextFixture()
    if invalid == 'clock': f.clock_valid = False
    if invalid == 'motion': f.motion_valid = False
    if invalid == 'physics_handle': f.handle_valid = False
    if invalid == 'wrong_robot': f.attachment['robot_name'] = 'right'
    with pytest.raises(ValueError):
        f.context()


def test_live_context_published_cache_accumulates_bounded_object_lead():
    f = _LiveContextFixture()
    leads, previous = [], None
    for tick in range(1, 10):
        f.advance(tick)
        tcp = f.adapter._assisted_absolute_object_servo_pose(phase_key=f.phase_key, task=f.task,
            robot_name='left', spec=f.spec, tracked_robots={}, tracked_objects=f.tracked,
            object_pose=f.object_pose, target_object_pose=f.task.target_poses['goal'])
        requested = f.inverse_fk(tcp)
        published, detail = f.publish(requested, previous)
        state = f.context()[-1]
        relative = f.context()[-2]
        predicted_model = object_pose_from_tcp(f.fk(published), *relative)
        model_actual = object_pose_from_tcp(f.fk(np.zeros(6)), *relative)
        expected = anchored_object_prediction(f.object_pose, model_actual, predicted_model)
        np.testing.assert_allclose(state['object_command']['position'], expected['position'], atol=1e-14)
        assert abs(np.dot(state['object_command']['orientation'], expected['orientation'])) == pytest.approx(1., abs=1e-14)
        lead = 2*np.arccos(min(1., abs(np.dot(f.object_pose['orientation'], state['object_command']['orientation']))))
        leads.append(lead)
        # Existing short-angle nlerp can exceed its ideal arc by tens of nrad.
        # This 0.1 urad test tolerance does not change any production budget.
        assert lead <= .0200001
        assert np.linalg.norm(state['object_command']['position']-f.object_pose['position']) <= .0030000001
        assert detail['accepted_ratio'] == 1.
        previous = published
    assert leads[0] == pytest.approx(.004, abs=1e-10)
    assert leads[1] > .007
    assert max(leads[4:]) > .019


def test_live_context_unadmitted_guard_caches_final_hold_and_keeps_it_next_tick():
    f = _LiveContextFixture(admission=True)
    f.task.target_poses['goal'] = pose(f.object_pose['position']+[.001, 0, -.01], f.object_pose['orientation'])
    state = f.context()[-1]
    state.update(descent_ready=False, stamp=stamp(f.tick))
    rejected = np.array([.0001, 0, -.001, 0, 0, 0])
    published, detail = f.publish(rejected, np.zeros(6))
    assert detail['accepted_ratio'] == 0.
    np.testing.assert_allclose(published, 0., atol=1e-14)
    np.testing.assert_allclose(state['object_command']['position'], f.object_pose['position'], atol=1e-14)
    cached = state['object_command']
    f.advance(2)
    assert f.context()[-1]['object_command'] is cached
def test_staged_clearance_requires_measured_lift_before_turn_or_transfer():
    import numpy as np
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import staged_clearance_target
    settings = dict(height_padding_m=.04, position_tolerance_m=.001,
                    orientation_tolerance_rad=.003)
    entry = dict(position=[.23, -.16, 1.17], orientation=[.70710678, 0, 0, .70710678])
    goal = dict(position=[.78, -.12, 1.192], orientation=[1, 0, 0, 0])
    state = {}; stamp = dict(valid=True, epoch=0, index=10)
    target = staged_clearance_target(state, stamp, entry, goal, settings)
    assert state['clearance_path']['stage'] == 'lift'
    np.testing.assert_allclose(target['position'], [.23, -.16, 1.232])
    np.testing.assert_allclose(target['orientation'], np.array(entry['orientation']) / np.linalg.norm(entry['orientation']))
    # A far-future call with an unchanged low observation still cannot turn.
    stamp['index'] = 500
    staged_clearance_target(state, stamp, entry, goal, settings)
    assert state['clearance_path']['stage'] == 'lift'
    high = dict(position=target['position'], orientation=entry['orientation'])
    stamp['index'] += 1
    target = staged_clearance_target(state, stamp, high, goal, settings)
    assert state['clearance_path']['stage'] == 'turn'
    np.testing.assert_allclose(target['position'][:2], entry['position'][:2])
    turned = dict(position=high['position'], orientation=goal['orientation'])
    staged_clearance_target(state, stamp, turned, goal, settings)
    assert state['clearance_path']['stage'] == 'turn'
    stamp['index'] += 1
    target = staged_clearance_target(state, stamp, turned, goal, settings)
    assert state['clearance_path']['stage'] == 'transfer'
    np.testing.assert_allclose(target['position'], [.78, -.12, 1.232])
    stamp['index'] += 1
    staged_clearance_target(state, stamp, target, goal, settings)
    assert state['clearance_path']['stage'] == 'lower'


def test_staged_clearance_rejects_clock_and_goal_changes():
    import pytest
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import staged_clearance_target
    settings = dict(height_padding_m=.04, position_tolerance_m=.001,
                    orientation_tolerance_rad=.003)
    current = dict(position=[0, 0, 1], orientation=[1, 0, 0, 0])
    target = dict(position=[1, 0, 1], orientation=[1, 0, 0, 0])
    state = {}
    staged_clearance_target(state, dict(valid=True, epoch=0, index=2), current, target, settings)
    with pytest.raises(ValueError, match='clock'):
        staged_clearance_target(state, dict(valid=True, epoch=1, index=3), current, target, settings)
    with pytest.raises(ValueError, match='goal'):
        staged_clearance_target(state, dict(valid=True, epoch=0, index=3), current,
                                {**target, 'position': [1.01, 0, 1]}, settings)


def test_free_motion_final_guard_rejects_missing_cad_and_never_caches_rejected_ik():
    adapter = Adapter({}); current = pose(); state = {}
    adapter._assisted_object_servo_context = lambda **kw: (
        'part', stamp(1), native(current), current, (np.zeros(3), [1, 0, 0, 0]), state)
    adapter._target_object_pose = lambda **kw: current
    adapter._grasp_command_fk_pose = lambda **kw: current
    context = dict(spec={'beam_assisted_production_cad_guard': True}, phase_key=(1,),
                   phase_index=1, phase_entry_step=1, tracked_robots={}, tracked_objects={})
    task = NS(phase_index=1, phase_entry_step=1)
    args = dict(task=task, robot_name='left', context=context,
                command_q=np.ones(6), current_q=np.zeros(6))
    with pytest.raises(ValueError, match='CAD guard'):
        adapter._assisted_verify_published_held_command(**args)
    calls = []
    def check(**kw):
        calls.append(kw['command_q'].copy())
        return bool(np.max(abs(kw['command_q'])) < .01), {'fixture': True}
    task._assisted_production_cad_guard = check
    final, detail = adapter._assisted_verify_published_held_command(**args)
    assert len(calls) == 6 and detail['accepted_ratio'] == 0
    np.testing.assert_allclose(final, 0)
    np.testing.assert_allclose(state['published_q'], final)
    task._assisted_production_cad_guard = lambda **kw: (False, {'fixture': True})
    with pytest.raises(ValueError, match='no safe'):
        adapter._assisted_verify_published_held_command(**args)



def test_fourth_clearance_adds_only_existing_staged_motion_contract():
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe = load_task_recipe('fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop')
    phases = [p for p in recipe['phases'] if p.get('local_skill', {}).get('beam_assisted_staged_clearance')]
    assert [p['name'] for p in phases] == ['assemble_01_part_2_assembly_clearance', 'assemble_02_part_1_assembly_clearance', 'assemble_03_part_0_assembly_clearance']
    phase = phases[1]; spec = phase['local_skill']
    assert spec['beam_assisted_staged_clearance'] == dict(height_padding_m=.04, position_tolerance_m=.001, orientation_tolerance_rad=.003)
    assert spec['cartesian_position_step'] == .001 and spec['cartesian_orientation_step'] == .004
    assert spec['beam_assisted_phase_handoff_ticks'] == 24 and spec['max_command_joint_step'] == .02
    assert spec['beam_assisted_fk_native_position_limit_m'] == .0001 and spec['beam_assisted_fk_native_orientation_limit_rad'] == .0015
    assert phase['timeout_steps'] == 12000
    assert spec['beam_assisted_lift_command_rotation_hold'] is True
    assert [p['name'] for p in recipe['phases'] if p.get('local_skill', {}).get(
        'beam_assisted_lift_command_rotation_hold')] == ['assemble_02_part_1_assembly_clearance', 'assemble_03_part_0_assembly_clearance']


class _LiftCommandFixture:
    """Real servo/context/outer publication chain; explicit coupled FK double.

    Translation couples into wrist rotation, so axiswise clipping and safety
    backtracking cannot accidentally satisfy the orientation invariant. This
    verifies control bookkeeping, not a native dynamics or CAD certificate.
    """
    def __init__(self):
        self.adapter, self.task = Adapter({}), BeamAssistedState()
        self.tick, self.actual = 100, np.zeros(6)
        self.relative = (np.array([.04, 0., .18]), np.array([1., 0., 0., 0.]))
        self.attachment = dict(mode='fixed_joint', robot_name='left', joint_path='/World/grasp',
            attach_step=9, attach_spec={'beam_assisted': True}, assisted_hold_openness=.3)
        self.tracked = {'part': {'attachment': self.attachment}}
        self.spec = dict(name='ur5e_move_to_pose', robot='left', object='part',
            requires_held_object=True, beam_assisted_absolute_object_servo=True,
            beam_assisted_phase_handoff_ticks=24, beam_assisted_production_cad_guard=True,
            beam_assisted_lift_command_rotation_hold=True,
            beam_assisted_staged_clearance=dict(height_padding_m=.04,
                position_tolerance_m=.001, orientation_tolerance_rad=.003),
            cartesian_position_step=.001, cartesian_orientation_step=.004,
            ik_position_tolerance=.00005, ik_orientation_tolerance=.0005)
        self.phase_key = (id(self.task), 2, 100, 'left', 'ur5e_move_to_pose')
        self.task.phase_index, self.task.phase_entry_step, self.task.step_counter = 2, 100, 100
        self.task._beam_physics_stamp = lambda: stamp(self.tick)
        self.task._assisted_published_q = {'left': np.array([.0002, -.0001, .0003, .0002, 0., 0.])}
        self.task._assisted_published_q_stamps = {'left': dict(epoch=0, index=99)}
        self.task._attachments = {'part': self.attachment}
        self.task.get_current_phase_spec = lambda: {'local_skill': self.spec}
        self.task.robots = {'left': NS(controllers={'arm_joint_controller': NS(get_joint_subset=lambda:
            NS(get_joint_positions=lambda: self.actual.copy()))})}
        self.events, self.refinements, self.cad_calls = [], [], []
        self.task._assisted_event = lambda event, **fields: self.events.append((event, fields))
        self.native_pose = object_pose_from_tcp(self.fk(self.actual), *self.relative)
        self.task._assisted_native_body_motion = lambda name, epoch: dict(**self.native_pose,
            motion_valid=True, physics_handle_valid=True, body_path='/World/part', motion_view_id='existing')
        self.adapter._object_tcp_relative_pose = lambda **kw: self.relative
        self.adapter._current_arm_q = lambda *args: self.actual.copy()
        self.adapter._grasp_command_fk_pose = lambda **kw: self.fk(kw['command_q'])
        self.goal = pose(self.native_pose['position']+[.02, 0., .01],
                         [np.cos(.1/2), 0., 0., np.sin(.1/2)])
        self.adapter._target_object_pose = lambda **kw: self.goal
        self.context = dict(adapter=self.adapter, spec=self.spec, phase_key=self.phase_key,
            phase_index=2, phase_entry_step=100, tracked_robots={}, tracked_objects=self.tracked)
        self.task._beam_assisted_command_contexts = {'left': self.context}
        self.task._assisted_production_cad_guard = self.cad
        self.adapter._local_grasp_ik = self.refine

    @staticmethod
    def fk(q):
        q = np.asarray(q, float)
        v = q[3:].copy(); v[0] += 5.*q[0]
        angle = float(np.linalg.norm(v))
        return pose(q[:3]+[.3, .2, 1.], np.r_[np.cos(angle/2),
            v*(np.sin(angle/2)/angle if angle else .5)])

    @staticmethod
    def inverse(target):
        rotation = normalize_quat(target['orientation'])
        if rotation[0] < 0:
            rotation = -rotation
        sine = float(np.linalg.norm(rotation[1:]))
        v = rotation[1:]*(2*np.arctan2(sine, rotation[0])/sine if sine else 2.)
        p = np.asarray(target['position'])-[.3, .2, 1.]
        v[0] -= 5.*p[0]
        return np.r_[p, v]

    def refine(self, **kw):
        self.refinements.append(kw)
        q = self.inverse(kw['target_pose'])
        return q if np.max(abs(q-kw['reference_q'])) <= kw['joint_radius']+1e-12 else None

    def cad(self, **kw):
        self.cad_calls.append(kw)
        return True, {'explicit_test_double': True}

    def act(self, requested=None):
        self.task.step_counter = self.tick
        self.command = self.adapter._assisted_absolute_object_servo_pose(
            phase_key=self.phase_key, task=self.task, robot_name='left', spec=self.spec,
            tracked_robots={}, tracked_objects=self.tracked, object_pose=self.native_pose,
            target_object_pose=self.goal)
        q = self.inverse(self.command) if requested is None else np.asarray(requested, float)
        result = self.task._assisted_guard_actions({'left': {'arm_joint_controller': [q.tolist()]}})
        return np.asarray(result['left']['arm_joint_controller'][0])

    def state(self):
        return self.task._beam_assisted_object_servo_states[('left', self.phase_key)]


@pytest.mark.parametrize('step', [.001, .0001166666667, .000001, 1e-9])
def test_incremental_lift_small_positive_motion_never_unloads_lateral_drive_bias(step):
    f = _LiftCommandFixture()
    f.spec.update(beam_assisted_incremental_clearance=True, cartesian_position_step=step)
    previous = f.task._assisted_published_q['left'].copy()
    before = f.fk(previous)
    np.testing.assert_array_equal(f.act(), previous)
    f.tick += 1
    final = f.act()
    after = f.fk(final)
    np.testing.assert_allclose(after['position'][:2], before['position'][:2], atol=1e-12, rtol=0)
    assert after['position'][2] >= before['position'][2]
    assert after['position'][2]-before['position'][2] <= step+1e-12
    np.testing.assert_allclose(after['orientation'], before['orientation'], atol=1e-12, rtol=0)


def test_incremental_clearance_repeated_native_tick_retains_accepted_final_q():
    f = _LiftCommandFixture(); f.spec['beam_assisted_incremental_clearance'] = True
    f.act(); f.tick += 1
    first = f.act(); second = f.act(); third = f.act()
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(first, third)
    f.tick += 1
    assert not np.array_equal(first, f.act())


@pytest.mark.parametrize('stage', ['turn', 'transfer', 'lower'])
def test_incremental_other_clearance_stages_keep_unchanged_object_axes(stage):
    f = _LiftCommandFixture(); f.spec['beam_assisted_incremental_clearance'] = True
    f.act(); f.tick += 1
    state = f.state(); state['clearance_path']['stage'] = stage
    old = object_pose_from_tcp(f.fk(f.task._assisted_published_q['left']), *f.relative)
    result = f.act(); new = object_pose_from_tcp(f.fk(result), *f.relative)
    if stage == 'turn':
        np.testing.assert_allclose(new['position'][:2], old['position'][:2], atol=1e-12, rtol=0)
        assert new['position'][2] <= old['position'][2]+1e-12
        assert abs(new['position'][2]-old['position'][2]) <= f.spec.get('cartesian_position_step', .001)+1e-9
        assert not np.allclose(new['orientation'], old['orientation'], atol=1e-7, rtol=0)
    else:
        np.testing.assert_allclose(new['orientation'], old['orientation'], atol=1e-12, rtol=0)
        if stage == 'transfer':
            assert new['position'][2] == pytest.approx(old['position'][2], abs=1e-12)
        else:
            np.testing.assert_allclose(new['position'][:2], old['position'][:2], atol=1e-12, rtol=0)


def test_incremental_final_guard_repairs_outer_joint_clip_as_complete_pose():
    f = _LiftCommandFixture(); f.spec['beam_assisted_incremental_clearance'] = True
    f.act(); f.tick += 1
    previous = f.task._assisted_published_q['left'].copy()
    final = f.act(requested=previous+np.array([.03, -.03, .03, .03, -.03, .03]))
    assert f.refinements
    before, after = f.fk(previous), f.fk(final)
    np.testing.assert_allclose(before['position'][:2], after['position'][:2], atol=1e-12, rtol=0)
    np.testing.assert_allclose(before['orientation'], after['orientation'], atol=1e-12, rtol=0)
    assert np.max(abs(final-previous)) <= .02


def test_incremental_cad_fallback_preserves_last_loaded_publication():
    f = _LiftCommandFixture(); f.spec['beam_assisted_incremental_clearance'] = True
    f.act(); f.tick += 1
    previous = f.task._assisted_published_q['left'].copy()
    f.task._assisted_production_cad_guard = lambda **k: (np.array_equal(k['command_q'], previous), {})
    np.testing.assert_array_equal(f.act(), previous)
    event = [d for e, d in f.events if e == 'incremental_clearance_command'][-1]
    assert event['hold_fallback'] and event['accepted_ratio'] == 0.


def test_incremental_clearance_never_erases_incoming_out_of_envelope_lead():
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import incremental_clearance_command
    f = _LiftCommandFixture(); previous = f.fk(np.array([.01, 0., 0., 0., 0., 0.]))
    with pytest.raises(ValueError, match='incoming final publication'):
        incremental_clearance_command(f.native_pose, f.native_pose, previous, f.relative,
            f.goal, 'lift', .001, .004, hold=True)


def test_loaded_lift_preserves_final_drive_rotation_and_frozen_lever_after_ik():
    from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error
    f = _LiftCommandFixture(); previous = f.task._assisted_published_q['left'].copy()
    np.testing.assert_array_equal(f.act(), previous)
    original_relative = tuple(x.copy() for x in f.relative)
    f.tick += 1
    final = f.act()
    assert np.linalg.norm(f.fk(final)['position']-f.fk(previous)['position']) > 1e-5
    _, angle = pose_error(f.fk(final)['position'], f.fk(final)['orientation'],
                         f.fk(final)['position'], f.fk(previous)['orientation'])
    assert angle <= 1e-6
    for actual, expected in zip(f.relative, original_relative):
        np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(f.state()['published_q'], final)
    assert f.cad_calls[-1]['current_q'].tolist() == f.actual.tolist()
    assert not f.state()['lift_command_zero_motion']


def test_loaded_lift_zero_motion_preserves_previous_q_exactly_after_outer_clamp(monkeypatch):
    import toolkits.factory_dual_franka_assembly.beam_assisted_servo as servo
    def zero_target(state, stamp, current, target, settings):
        state['clearance_path'] = {'stage': 'lift'}
        return current
    monkeypatch.setattr(servo, 'staged_clearance_target', zero_target)
    f = _LiftCommandFixture(); previous = f.task._assisted_published_q['left'].copy()
    f.act(); f.tick += 1
    final = f.act(requested=[.05, .05, .05, .05, .05, .05])
    np.testing.assert_array_equal(final, previous)
    assert f.state()['lift_command_zero_motion']
    assert f.cad_calls


def test_loaded_lift_refines_axiswise_clamp_and_cad_backtrack_inside_original_joint_box():
    from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error
    f = _LiftCommandFixture(); previous = f.act(); f.tick += 1
    requested = previous.copy(); requested[0] += .002; requested[3] += .03
    # Axiswise outer clamp destroys orientation; the CAD double then requires
    # a still smaller candidate. Production guard must re-solve each candidate.
    def cad(**kw):
        f.cad_calls.append(kw)
        return bool(np.max(abs(kw['command_q']-previous)) < .002), {'explicit_test_double': True}
    f.task._assisted_production_cad_guard = cad
    final = f.act(requested)
    assert f.refinements and np.linalg.norm(final-previous) > 1e-6
    assert any(np.max(abs(x['command_q']-previous)) >= .002 for x in f.cad_calls[1:])
    assert all(x['joint_radius'] == f.state()['phase_handoff_joint_step'] for x in f.refinements)
    assert all(x['spec']['ik_orientation_tolerance'] == 1e-6 for x in f.refinements)
    _, angle = pose_error(f.fk(final)['position'], f.fk(final)['orientation'],
                         f.fk(final)['position'], f.fk(previous)['orientation'])
    assert angle <= 1e-6
    report = [d for event, d in f.events if event == 'lift_command_rotation_hold'][-1]
    assert report['refined'] and report['accepted_ratio'] < 1.
    assert not report['hold_fallback']


def test_loaded_lift_cad_rejection_caches_exact_previous_and_reports_hold_fallback():
    f = _LiftCommandFixture(); previous = f.act(); f.tick += 1
    f.task._assisted_production_cad_guard = lambda **kw: (
        np.array_equal(kw['command_q'], previous), {'explicit_test_double': True})
    final = f.act()
    np.testing.assert_array_equal(final, previous)
    np.testing.assert_array_equal(f.state()['published_q'], previous)
    report = [d for event, d in f.events if event == 'lift_command_rotation_hold'][-1]
    assert report['hold_fallback'] and report['accepted_ratio'] == 0.


@pytest.mark.parametrize('bad', ['stale', 'epoch', 'nonfinite', 'tracking'])
def test_loaded_lift_rejects_invalid_publication_on_later_ticks(bad):
    f = _LiftCommandFixture(); f.act(); f.tick += 1
    if bad == 'stale': f.task._assisted_published_q_stamps['left']['index'] = f.tick-2
    if bad == 'epoch': f.task._assisted_published_q_stamps['left']['epoch'] = 1
    if bad == 'nonfinite': f.task._assisted_published_q['left'][0] = np.nan
    if bad == 'tracking': f.task._assisted_published_q['left'][0] = .121
    with pytest.raises(ValueError, match='fresh final published target'):
        f.act()


def test_loaded_lift_rejects_changed_attachment_instead_of_reanchoring():
    f = _LiftCommandFixture(); f.act(); f.tick += 1
    f.attachment['joint_path'] = '/World/replaced_grasp'
    with pytest.raises(ValueError, match='grasp identity changed'):
        f.act()


def test_loaded_lift_turn_stage_restores_normal_rotation_from_final_cache():
    from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error
    f = _LiftCommandFixture(); previous = f.act(); f.tick += 1
    state = f.state(); path = state['clearance_path']
    f.actual[2] += path['height']-f.native_pose['position'][2]
    f.native_pose = object_pose_from_tcp(f.fk(f.actual), *f.relative)
    # The test double advances a genuine matching drive/measurement vertically;
    # its previous-final timestamp stays fresh and its frozen relation is fixed.
    f.task._assisted_published_q['left'] = previous+f.actual
    final = f.act()
    assert state['clearance_path']['stage'] == 'turn'
    assert not state['lift_command_hold_active']
    _, angle = pose_error(f.command['position'], f.command['orientation'],
                         f.command['position'], state['lift_command_orientation'])
    assert angle > 1e-5
    assert np.all(np.isfinite(final))


def test_loaded_lift_original_model_failure_keeps_exact_exception_input():
    f = _LiftCommandFixture()
    f.native_pose['position'] = f.native_pose['position']+[.00011, 0., 0.]
    with pytest.raises(ValueError, match='model'):
        f.act()
    report = [d for event, d in f.events if event == 'object_model_residual_rejected'][-1]
    np.testing.assert_array_equal(report['native_object_pose']['position'], f.native_pose['position'])
    np.testing.assert_array_equal(report['actual_q'], f.actual)
    np.testing.assert_array_equal(report['captured_object_in_tcp_position'], f.relative[0])
    assert report['physics_stamp'] == stamp(f.tick)
    assert report['position_limit_m'] == .0001 and report['orientation_limit_rad'] == .0015


@pytest.mark.parametrize('response',[.05,.1,.2])
def test_incremental_terminal_height_retains_entry_load_bias_without_trapping_turn(response):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import incremental_clearance_command
    relative=(np.array([.03,-.02,.234]),np.array([1.,0.,0.,0.]))
    reference=dict(position=np.array([.00012,-.00016,.00045]),orientation=np.array([1.,0.,0.,0.]))
    current=pose((0.,0.,0.));goal=pose((0.,0.,.05))
    previous_model=dict(position=current['position']+reference['position'],orientation=reference['orientation'].copy())
    previous=dict(position=previous_model['position']-relative[0],orientation=previous_model['orientation'].copy())
    stage='lift';entered=None;maximum=-1.
    for tick in range(1400):
        if stage=='lift' and abs(current['position'][2]-.05)<.001:stage='turn';entered=tick
        tcp,model,detail=incremental_clearance_command(current,current,previous,relative,goal,stage,.001,.004,drive_reference=reference)
        maximum=max(maximum,float(model['position'][2]))
        current['position']+=response*(model['position']-reference['position']-current['position'])
        previous=tcp
        if entered is not None and tick>entered+350:break
    assert entered is not None and maximum<=.05045+1e-12
    assert abs(current['position'][2]-.05)<1e-8
    np.testing.assert_allclose(object_pose_from_tcp(previous,*relative)['position'],model['position'],atol=1e-12)
    np.testing.assert_array_equal(relative[0],[.03,-.02,.234])


@pytest.mark.parametrize('response',[.05,.1,.2])
def test_incremental_terminal_rotation_settles_inside_original_gate_after_transfer(response):
    from scipy.spatial.transform import Rotation
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import incremental_clearance_command
    bias=Rotation.from_rotvec([.001,0.,0.]);goal_rot=Rotation.from_rotvec([0.,0.,1.])
    q=lambda r:r.as_quat()[[3,0,1,2]]
    reference=dict(position=np.array([.0001,-.0001,.0004]),orientation=q(bias))
    relative=(np.array([.03,.04,.234]),np.array([1.,0.,0.,0.]))
    current=pose((0.,0.,1.));current['orientation']=q(Rotation.identity());goal=pose((0.,0.,1.));goal['orientation']=q(goal_rot)
    previous=dict(position=current['position']+reference['position']-bias.apply(relative[0]),orientation=q(bias))
    stage='turn';entered=None;actual=Rotation.identity()
    for tick in range(2400):
        if stage=='turn' and (actual*goal_rot.inv()).magnitude()<.003:stage='transfer';entered=tick
        tcp,model,detail=incremental_clearance_command(current,current,previous,relative,goal,stage,.001,.004,drive_reference=reference)
        desired=bias.inv()*Rotation.from_quat(model['orientation'][[1,2,3,0]])
        actual=Rotation.from_rotvec(response*(desired*actual.inv()).as_rotvec())*actual
        current['orientation']=q(actual);previous=tcp
        if entered is not None and tick>entered+350:break
    assert entered is not None
    assert (actual*goal_rot.inv()).magnitude()<1e-8
    np.testing.assert_allclose(model['orientation'],q(bias*goal_rot),atol=1e-10)


# Reuse the COM estimator already exercised by the loaded pickup transition.
def _com_admission_sample(current, *, raw_linear=.0198, raw_angular=.305):
    sample=com_transition_sample(current['position'].copy(),current['orientation'].copy())
    sample.update(linear_speed=raw_linear,angular_speed=raw_angular)
    return sample


def _com_admission_contract():
    return dict(contract(),motion_metric='native_com_pose_difference')


def test_com_admission_quiet_D18_shape_advances_without_changing_raw_solver_velocity():
    current=pose((.000058,0,1.),(np.cos(.000668/2),np.sin(.000668/2),0,0))
    goal=pose((0,0,.99));state={};old={};sample=_com_admission_sample(current)
    for tick in range(13):
        assert not descent_admission(old,stamp(tick),current,goal,[0,0,-1],contract(),sample)[0]
        ready,detail=descent_admission(state,stamp(tick),current,goal,[0,0,-1],_com_admission_contract(),sample)
        assert ready==(tick==12)
        assert detail['angular_speed_rad_s']==sample['angular_speed']==.305
        assert detail['linear_speed_m_s']==sample['linear_speed']==.0198
        if tick:assert detail['qualified_angular_speed_rad_s']==detail['qualified_linear_speed_m_s']==0.
    assert detail['native_count']==12


@pytest.mark.parametrize('mode',['axial','angular'])
def test_com_admission_rejects_actual_motion_despite_zero_reported_velocity(mode):
    state={};goal=pose((0,0,.9))
    for tick in range(30):
        angle=.21*tick/240 if mode=='angular' else 0.
        current=pose((0,0,1.-(.025*tick/240 if mode=='axial' else 0.)),
                     (np.cos(angle/2),np.sin(angle/2),0,0))
        sample=_com_admission_sample(current,raw_linear=0.,raw_angular=0.)
        ready,detail=descent_admission(state,stamp(tick),current,goal,[0,0,-1],_com_admission_contract(),sample)
        assert not ready
        if tick:assert detail['qualified_linear_speed_m_s']>.02 if mode=='axial' else detail['qualified_angular_speed_rad_s']>.2


@pytest.mark.parametrize('mode',['axial_allowed','axial_fast','lateral_fast'])
def test_com_admission_warm_axis_uses_measured_COM_vector_with_original_limits(mode):
    state={};current=pose();goal=pose((0,0,.9))
    for tick in range(13):
        ready,_=descent_admission(state,stamp(tick),current,goal,[0,0,-1],_com_admission_contract(),_com_admission_sample(current))
    assert ready
    speed=.04 if mode=='axial_allowed' else .051 if mode=='axial_fast' else .021
    velocity=np.array([speed,0,0]) if mode=='lateral_fast' else np.array([0,0,-speed])
    for tick in range(13,18):
        current=pose(np.array([0,0,1.])+velocity*(tick-12)/240)
        sample=_com_admission_sample(current,raw_linear=1.)
        sample['linear_velocity']=[1.,0,0]  # Raw solver vector is diagnostic too.
        ready,detail=descent_admission(state,stamp(tick),current,goal,[0,0,-1],_com_admission_contract(),sample)
        assert ready==(mode=='axial_allowed')
        np.testing.assert_allclose(detail['qualified_linear_velocity_world'],velocity,atol=1e-10)


@pytest.mark.parametrize('bad',['missing_com','coordinates','frame','quaternion','nan_position','nan_velocity','negative_velocity','metric','invalid_clock'])
def test_com_admission_invalid_data_never_qualifies(bad):
    current=pose();sample=_com_admission_sample(current);settings=_com_admission_contract();clock=stamp(0)
    if bad=='missing_com':sample['center_of_mass_valid']=False
    if bad=='coordinates':sample.pop('center_of_mass_position')
    if bad=='frame':sample['center_of_mass_position_frame']='local'
    if bad=='quaternion':sample['center_of_mass_orientation']*=0.
    if bad=='nan_position':sample['center_of_mass_position'][0]=np.nan
    if bad=='nan_velocity':sample['angular_speed']=np.nan
    if bad=='negative_velocity':sample['linear_speed']=-1.
    if bad=='metric':settings['motion_metric']='unknown'
    if bad=='invalid_clock':clock['valid']=False
    with pytest.raises(ValueError):descent_admission({},clock,current,pose((0,0,.9)),[0,0,-1],settings,sample)


@pytest.mark.parametrize('changed',['epoch','view','body'])
def test_com_admission_identity_change_requires_new_native_window(changed):
    state={};current=pose();sample=_com_admission_sample(current)
    for tick in range(13):ready,_=descent_admission(state,stamp(tick),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)
    assert ready
    if changed=='view':sample['motion_view_id']+=1
    if changed=='body':sample['body_path']+='changed'
    for tick in range(13,26):
        ready,_=descent_admission(state,stamp(tick,epoch=int(changed=='epoch')),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)
        assert ready==(tick==25)


def test_com_admission_duplicate_clock_gap_and_changed_observation_fail_closed():
    state={};current=pose();sample=_com_admission_sample(current)
    for _ in range(30):assert not descent_admission(state,stamp(0),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)[0]
    for tick in range(1,13):ready,_=descent_admission(state,stamp(tick),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)
    assert ready
    for tick in range(20,33):
        ready,_=descent_admission(state,stamp(tick),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)
        assert ready==(tick==32)
    sample['center_of_mass_orientation']*=-1.
    assert descent_admission(state,stamp(32),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)[0]
    sample['center_of_mass_position'][0]+=1e-7
    with pytest.raises(ValueError,match='duplicate native clock'):descent_admission(state,stamp(32),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)
    with pytest.raises(ValueError,match='clock reversed'):descent_admission(state,stamp(31),current,pose((0,0,.9)),[0,0,-1],_com_admission_contract(),sample)


@pytest.mark.parametrize('metric_owner',['descent','transition','neither'])
def test_COM_context_requests_same_existing_native_COM_reader(metric_owner):
    adapter=Adapter({});task=NS();calls=[]
    task._beam_physics_stamp=lambda:stamp(0)
    def read(name,epoch,**options):calls.append(options);return _com_admission_sample(pose())
    task._assisted_native_body_motion=read
    adapter._object_tcp_relative_pose=lambda **kw:(np.zeros(3),np.array([1.,0,0,0]))
    spec={'object':'part'}
    if metric_owner=='descent':spec['beam_assisted_descent_admission']=_com_admission_contract()
    if metric_owner=='transition':spec['beam_assisted_transition_settle']={'motion_metric':'native_com_pose_difference'}
    tracked={'part':{'attachment':{'mode':'fixed_joint','robot_name':'left','joint_path':'joint','attach_step':0}}}
    adapter._assisted_object_servo_context(phase_key=(1,),task=task,robot_name='left',spec=spec,tracked_robots={},tracked_objects=tracked)
    assert calls==([{}] if metric_owner=='neither' else [{'include_center_of_mass':True}])


@pytest.mark.parametrize('object_name', ['fabrica_beam_2', 'fabrica_beam_1', 'fabrica_beam_0'])
def test_shared_loaded_clearance_reaches_transfer_only_after_its_own_measured_lift(object_name):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import staged_clearance_target
    from roboassemblybench.core.task_registry import load_task_recipe
    recipe = load_task_recipe('fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop')
    phase = next(p for p in recipe['phases'] if p['name'].endswith('_assembly_clearance')
                 and p.get('local_skill', {}).get('object') == object_name)
    settings = phase['local_skill']['beam_assisted_staged_clearance']
    state = {}; entry = dict(position=np.array([.5, -.2, 1.1]), orientation=np.array([1.,0.,0.,0.]))
    goal = dict(position=np.array([.75, -.1, 1.13]), orientation=np.array([1.,0.,0.,0.]))
    for tick in (1, 2, 3):
        target = staged_clearance_target(state, stamp(tick), entry, goal, settings)
        assert state['clearance_path']['stage'] == 'lift'
        np.testing.assert_allclose(target['position'][:2], entry['position'][:2])
    high = dict(position=target['position'], orientation=target['orientation'])
    staged_clearance_target(state, stamp(4), high, goal, settings)
    staged_clearance_target(state, stamp(5), high, goal, settings)
    assert state['clearance_path']['stage'] == 'transfer'
    assert phase['local_skill']['beam_assisted_production_cad_guard'] is True
    assert phase['local_skill']['beam_assisted_fk_native_orientation_limit_rad'] == .0015


@pytest.mark.parametrize('height_error', [.0016513208389281608, -.0014])
def test_incremental_turn_corrects_measured_height_with_transient_entry_lead(height_error):
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import incremental_clearance_command
    relative=(np.array([.03,-.02,.234]),np.array([1.,0.,0.,0.]))
    current=pose((.37076956,-.26158321,1.1446+height_error))
    target=pose((.37074405,-.26163831,1.1446))
    reference=dict(position=np.array([-.0002209232,-.000286816,.0027707892]),orientation=np.array([1.,0.,0.,0.]))
    previous_model=pose(tuple(current['position']+np.array([-.0001,-.00012,.000354])))
    previous=pose(tuple(previous_model['position']-relative[0]))
    tcp,model,detail=incremental_clearance_command(current,current,previous,relative,target,'turn',.001,.004,drive_reference=reference)
    change=model['position']-previous_model['position']
    if height_error > 0:
        assert change[2] < -.0009
    else:
        np.testing.assert_allclose(change, np.zeros(3), atol=1e-12)
    assert np.linalg.norm(change)<=.001+1e-12
    np.testing.assert_allclose(change[:2], np.zeros(2), atol=1e-12)
    # Incoming command bias survives as a full pose; no native pose/anchor reset.
    np.testing.assert_allclose(object_pose_from_tcp(tcp,*relative)['position'],model['position'],atol=1e-12)
    held,held_model,_=incremental_clearance_command(current,current,previous,relative,target,'turn',.001,.004,drive_reference=reference,hold=True)
    np.testing.assert_allclose(held_model['position'],previous_model['position'],atol=1e-12)
