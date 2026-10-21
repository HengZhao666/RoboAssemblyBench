"""Reusable real-record final-publication/CAD check; never advances physics."""
from pathlib import Path
import hashlib
from types import SimpleNamespace as NS
import copy,inspect,json,sys,time
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import argparse
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('--baseline',action='store_true');ap.add_argument('--recorded-run',default='A27_beam_full');ap.add_argument('--step',type=int,default=33344);ap.add_argument('--phase',default='assemble_02_part_1_assembly_clearance');ap.add_argument('--adjacent-only',action='store_true');ap.add_argument('--recorded-native-motion',action='store_true');ap.add_argument('--audit-final-cad',action='store_true');ap.add_argument('--pregrasp-joint-branch',action='store_true');ap.add_argument('--output-name');ap.add_argument('--carry-path',action='store_true');ap.add_argument('--carry-max-ticks',type=int,default=5000);ap.add_argument('--geometric-waypoints',action='store_true');ap.add_argument('--recorded-clearance-state',action='store_true');ap.add_argument('--final-home',action='store_true');opt=ap.parse_args()
W=opt.work.resolve();P=W.parent;C=P/'assisted_repository' if opt.baseline else W/'candidate';D=P/'runs'/opt.recorded_run
sys.path[:0]=[str(C),str(P/'tools')]
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
from toolkits.factory_dual_franka_assembly.beam_assisted_state import BeamAssistedState
from toolkits.factory_dual_franka_assembly.beam_assisted_servo import object_pose_from_tcp,validate_model_native_residual
from beam_workflow import sha,save
from index_run import select
from sweep_model import SceneModel,pose_matrix
from visual_mesh_check import ProductionCarryChecker
assert Path(inspect.getfile(Adapter)).is_relative_to(C)
begin=time.monotonic();tool_sha=sha(__file__);read=lambda p:json.loads(p.read_text())
if opt.final_home:
    assert not opt.baseline and not any((opt.pregrasp_joint_branch,opt.carry_path,
        opt.geometric_waypoints,opt.recorded_clearance_state,opt.adjacent_only))
    seed_path=W/'A42_return_seed.json';seed=read(seed_path)
    assert seed['run']=='A42_beam_full' and seed['task_step']==57408
    D=P/'runs'/seed['run'];raw=seed['native'];entity=seed['entity'];clocks=seed['camera_clock']
    assert raw['boundary']=='before_action' and raw['task_step']==seed['task_step']
    assert entity['step']==seed['task_step'] and len(entity['body_poses'])==44
    assert len(clocks)==3 and {r['camera'] for r in clocks}=={
        'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'}
    initial_stamp=clocks[0]['physics_stamp']
    assert all(r['physics_stamp']==initial_stamp and r['task_step_label']==seed['task_step'] for r in clocks)
    assert initial_stamp['valid'] and initial_stamp['index']==raw['physics_step_index']
    assert abs(initial_stamp['time']-raw['physics_time'])<1e-9
    cfg_path=W/'actual_configuration_layout505_config.json';cfg=read(cfg_path)
    phase_names=['beam_assisted_return_home_franka_left',
        'beam_assisted_return_clearance_franka_right','beam_assisted_return_home_franka_right']
    phases=[next(p for p in cfg['phase_specs'] if p['name']==name) for name in phase_names]
    assert [p['name'] for p in cfg['phase_specs'][-4:-1]]==phase_names
    assert cfg['phase_specs'][-1]['name']=='beam_assisted_final_verification'
    for phase in phases:
        spec=phase['local_skill']
        assert phase['beam_assisted_empty_return_guard'] and phase['beam_assisted_native_joint_limits']
        assert spec['name']=='move_arm_to_joint_positions' and spec['beam_assisted_native_joint_target']
        assert spec['unwrap_revolute_joints'] is False and spec['joint_target_stable_steps']==8
        assert spec['joint_position_tolerance']==.01 and spec['max_joint_step']==.01
        assert spec['max_command_tracking_error']==.03
    assert phases[1]['local_skill']['beam_assisted_joint_target_delta']==[0.,-.24,0.,0.,0.,0.]
    robot_configs={r['name']:r for r in cfg['robots']}
    initial_q={n:np.asarray(r['joint_positions'][0],float) for n,r in raw['robots'].items()}
    initial_pub={n:np.asarray(next(a['joint_positions'] for a in r['sdk_submission']['actions']
        if a.get('joint_indices')==list(range(6))),float) for n,r in raw['robots'].items()}
    nonideal={n:float(np.max(abs(initial_pub[n]-initial_q[n][:6]))) for n in initial_q}
    assert set(initial_q)==set(robot_configs)=={'franka_left','franka_right'}
    assert all(r['valid'] and r['physics_handle_valid'] for r in raw['robots'].values())
    assert all(v>1e-6 for v in nonideal.values()),'Do not replace the recorded tracking-biased seed with an ideal state'
    model=SceneModel(D/'scene_geometry.json')
    initial_fk=model.poses(initial_q);native0={b:np.asarray(v,float) for b,v in entity['body_poses'].items()}
    robot_bodies={m['body'] for m in model.data['visible_meshes'] if m.get('robot')}
    corrections={b:native0[b]@np.linalg.inv(initial_fk[b]) for b in robot_bodies}
    assert len(corrections)==32 and not model.gaps and len(model.data['visible_meshes'])==54
    limits_rows=[e for e in select(D,'assisted_trace.jsonl',0,256)
        if e.get('event')=='native_arm_joint_limits']
    native_limits={n:next(e for e in reversed(limits_rows) if e.get('robot')==n) for n in initial_q}
    source_before={n:sha(C/n) for n in read(W/'baseline_manifest.json')}
    assert len(source_before)==889
    checker_sha=sha(P/'tools/visual_mesh_check.py');response=.08

    def run_return_case(*, unsafe=False):
        q={n:v.copy() for n,v in initial_q.items()};pub={n:v.copy() for n,v in initial_pub.items()}
        selected=phases
        if unsafe:
            # Exact geometric sensitivity counterexample: already parked left
            # arm, but the measured right elbow is +.03rad off its A42 seed.
            home=cfg['phase_specs'][-1]['beam_assisted_home_targets']['franka_left']['joint_positions']
            q['franka_left'][:6]=home;pub['franka_left']=np.asarray(home,float)
            q['franka_right'][2]+=.03
            selected=[phases[1]]
        checker=ProductionCarryChecker(model);task=BeamAssistedState();adapter=Adapter({})
        task._attachments={};task._assisted_published_q={n:v.copy() for n,v in pub.items()}
        task._ur5e_plumbers_last_arm_command_q={n:v.copy() for n,v in pub.items()}
        task._beam_physics_stamp=lambda:dict(fresh)
        task.phase_specs=cfg['phase_specs'];task.robots={};events=[];completed=[];checks=[];received=[]
        task._assisted_event=lambda event,**fields:events.append(dict(event=event,**fields))
        task.mark_local_skill_complete=lambda **fields:completed.append(copy.deepcopy(fields))
        task.get_current_phase_spec=lambda:phase
        fresh=dict(initial_stamp)
        for name,r in raw['robots'].items():
            names=list(r['joint_names']);limit=native_limits[name]
            bounds=np.full((len(names),2),np.nan,dtype=np.float32)
            for joint,row in zip(limit['joint_names'],limit['limits_rad']):bounds[names.index(joint)]=row
            view=NS(is_physics_handle_valid=lambda:True,
                get_dof_limits=lambda b=bounds:b[None].copy(),
                get_joint_positions=lambda clone,name=name:q[name][None].copy())
            articulation=NS(dof_names=names,_articulation_view=view,
                get_joint_positions=lambda name=name:q[name].copy())
            controller=NS(joint_names=names[:6],get_joint_subset=lambda name=name:
                NS(get_joint_positions=lambda:q[name][:6].copy()))
            task.robots[name]=NS(config=NS(**robot_configs[name]),articulation=articulation,
                controllers={'arm_joint_controller':controller})
        def current_native():
            fk=model.poses(q);native=dict(native0)
            for body,correction in corrections.items():native[body]=correction@fk[body]
            return native
        def production_guard(**command):
            assert command.pop('empty') is True and command['physics_stamp']==fresh
            allowed,detail=checker.check_empty(native_poses=current_native(),actual_q=q,
                other_targets=task._assisted_batch_arm_candidates,phase=phase['name'],**command)
            assert command['physics_stamp']==fresh
            checks.append(dict(phase=phase['name'],allowed=allowed,detail=detail))
            return allowed,detail
        task._assisted_production_cad_guard=production_guard
        summaries=[];trace=[];rejection=None;duplicate_checked=[];delta_frozen=True
        for phase in selected:
            task.phase=phase['name'];task.phase_index=cfg['phase_specs'].index(phase)
            task.phase_entry_step=int(fresh['index']);spec=phase['local_skill'];robot=spec['robot']
            entry=q[robot][:6].copy();task._assisted_active_arm(robot)
            before_completed=len(completed);duplicate=False
            phase_key=(id(task),task.phase_index,task.phase_entry_step,robot,spec['name'])
            expected_delta_target=entry+np.asarray(spec.get('beam_assisted_joint_target_delta',[0.]*6))
            for age in range(int(phase['timeout_steps'])):
                task.step_counter=fresh['index'];task.phase_step_counter=age
                action=adapter.act(task=task,robot_name=robot,phase_spec=phase,skill_spec=spec,
                    tracked_robots={},tracked_objects={})
                assert not action.get('__local_skill_failure__'),action
                state=adapter._completion_state[phase_key]
                if 'beam_assisted_joint_target_delta' in spec:
                    delta=state['native_delta_target']
                    delta_frozen=bool(delta_frozen and np.array_equal(delta['entry_joint_positions'],entry)
                        and np.array_equal(delta['target_joint_positions'],expected_delta_target))
                    assert delta_frozen,'A measured delta target was re-anchored within the phase'
                gate=state['joint_target_native_gate']
                if gate.get('count')==1 and not duplicate:
                    # Repeat reads at the same synthetic native tick. Keep
                    # unpublished adapter command caches out of the trajectory.
                    old_task_cache=copy.deepcopy(task._ur5e_plumbers_last_arm_command_q)
                    old_adapter_cache=copy.deepcopy(adapter._last_arm_command_q)
                    for _ in range(16):
                        repeat=adapter.act(task=task,robot_name=robot,phase_spec=phase,skill_spec=spec,
                            tracked_robots={},tracked_objects={})
                        assert not repeat.get('__local_skill_failure__') and gate['count']==1
                        assert len(completed)==before_completed
                    task._ur5e_plumbers_last_arm_command_q=old_task_cache
                    adapter._last_arm_command_q=old_adapter_cache
                    duplicate=True;duplicate_checked.append(phase['name'])
                actions={name:(action if name==robot else task._assisted_idle_action(name,phase))
                    for name in task.robots}
                try:guarded=task._assisted_guard_actions(actions)
                except ValueError as exc:
                    rejection=str(exc);break
                # This is the explicit simulated SDK receiver boundary; a
                # rejected batch never reaches it or the proxy response.
                received.append(dict(phase=phase['name'],physics_stamp=dict(fresh)))
                final={n:np.asarray(a['arm_joint_controller'][0],dtype=np.float32).astype(float)
                    for n,a in guarded.items()}
                assert all(np.max(abs(final[n]-q[n][:6]))<=.030001 for n in q)
                for name in q:
                    q[name][:6]+=response*(final[name]-q[name][:6])
                    q[name][6:]+=response*(0.-q[name][6:])
                if age%48==0:trace.append(dict(phase=phase['name'],age=age,
                    actual_q={n:q[n][:6].tolist() for n in q},published_q={n:v.tolist() for n,v in final.items()}))
                if len(completed)>before_completed:
                    detail=completed[-1]['detail']
                    assert detail['stable_steps']>=8 and detail['required_stable_steps']==8
                    summaries.append(dict(phase=phase['name'],ticks=age+1,native_gate=copy.deepcopy(gate),
                        completion=detail,entry_joint_positions=entry.tolist(),
                        final_joint_positions=q[robot][:6].tolist(),duplicate_tick_checked=duplicate))
                    break
                fresh.update(index=fresh['index']+1,time=fresh['time']+fresh['dt'])
            if rejection:break
            if len(completed)==before_completed:
                rejection='declared proxy did not finish within original phase timeout';break
            fresh.update(index=fresh['index']+1,time=fresh['time']+fresh['dt'])
        home_errors={n:float(np.max(abs(q[n][:6]-np.asarray(
            cfg['phase_specs'][-1]['beam_assisted_home_targets'][n]['joint_positions'],float)))) for n in q}
        return dict(completed_phases=summaries,rejection=rejection,guard_calls=checker.calls,
            mesh_samples=checker.samples,checks=checks,published_batches=len(received),
            native_completion_gate_checked=len(duplicate_checked)==len(selected) and len(summaries)==len(selected),
            phase_entry_delta_frozen=delta_frozen,home_errors_rad=home_errors,path=trace,
            duplicate_native_ticks_checked=duplicate_checked,
            unsafe_initial_right_elbow_delta_rad=.03 if unsafe else 0.)

    positive=run_return_case();negative=run_return_case(unsafe=True)
    all_complete=[r['phase'] for r in positive['completed_phases']]==phase_names
    positive_failures=[r for r in positive['checks'] if not r['allowed']]
    negative_rejected=bool(negative['rejection'] and negative['published_batches']==0
        and negative['checks'] and not negative['checks'][0]['allowed'])
    chain=bool(positive['published_batches']>0 and positive['guard_calls']==positive['published_batches'])
    report=dict(passed=bool(all_complete and not positive_failures and chain
        and positive['native_completion_gate_checked'] and positive['phase_entry_delta_frozen']
        and all(v<=.01 for v in positive['home_errors_rad'].values()) and negative_rejected),
        qualification_kind='final_home_production_chain',native_physics=False,simulator_created=False,
        checkpoint_used=False,production_entry_chain_verified=chain,full54=True,self_checked=True,
        guard_calls=positive['guard_calls']+negative['guard_calls'],
        mesh_samples=positive['mesh_samples']+negative['mesh_samples'],geometry_failures=positive_failures,
        actual_recorded_run=seed['run'],actual_input_step=seed['task_step'],
        source_manifest=source_before,all_three_return_phases_completed=all_complete,
        native_completion_gate_checked=positive['native_completion_gate_checked'],
        phase_entry_delta_frozen=positive['phase_entry_delta_frozen'],
        unsafe_counterexample_rejected_before_publication=negative_rejected,
        nonideal_seed_tracking_delta_rad=nonideal,positive=positive,unsafe_counterexample=negative,
        seed_sha256=sha(seed_path),configuration_file_sha256=sha(cfg_path),
        scene_geometry_sha256=sha(D/'scene_geometry.json'),raw_index_sha256=sha(D/'raw_offset_index.json'),
        original_three_camera_clock=initial_stamp,native_arm_limits= native_limits,
        tool_sha256=tool_sha,empty_checker_sha256=checker_sha,elapsed_s=time.monotonic()-begin,
        kinematic_response=response,
        scope='Production adapter.act joint target skill, actual final task guard and full54/self check_empty using A42 measured q/publication bias and all44 same-clock bodies. Subsequent states follow an explicit .08 bounded first-order kinematic response, including open grippers. Native clocks/articulation views after the seed are read-only test doubles. No physical dynamics/contact/solver equivalence, no final240 or complete-assembly proof; live return remains to be tested.')
    assert all(sha(C/n)==h for n,h in source_before.items())
    assert sha(__file__)==tool_sha and sha(P/'tools/visual_mesh_check.py')==checker_sha
    name=opt.output_name or 'final_home_control_replay.json'
    assert Path(name).name==name and name.endswith('.json')
    target=W/name;assert not target.exists();save(target,report)
    print(json.dumps({k:report[k] for k in ['passed','all_three_return_phases_completed',
        'native_completion_gate_checked','unsafe_counterexample_rejected_before_publication',
        'guard_calls','mesh_samples','elapsed_s']}));raise SystemExit(0 if report['passed'] else 1)

if opt.pregrasp_joint_branch:
    cfg=read(W/('base_configuration_layout505_config.json' if opt.baseline else 'actual_configuration_layout505_config.json'))
    phase=next(p for p in cfg['phase_specs'] if p['name']=='assemble_02_part_1_move_above');spec=phase['local_skill']
    assert not spec.get('prealign_target_ik') and not spec.get('requires_held_object')
    raw=select(D,'physics_boundary_trace.jsonl',opt.step,opt.step,'before_action')[0]
    entity=select(D,'entity_geometry_trace.jsonl',opt.step,opt.step)[0]
    clocks=select(D,'video_frame_clock.jsonl',opt.step,opt.step)
    assert len(clocks)==3 and len(entity['body_poses'])==44
    assert all(r['physics_stamp']['index']==raw['physics_step_index'] for r in clocks)
    q={n:np.asarray(r['joint_positions'][0],float) for n,r in raw['robots'].items()}
    published={n:np.asarray(next(a['joint_positions'] for a in r['sdk_submission']['actions'] if a.get('joint_indices')==list(range(6))),float) for n,r in raw['robots'].items()}
    limits_data=read(P/'D20_work/asset_joint_limits.json')
    by_name={r['name']:np.radians([r['lower_degrees'],r['upper_degrees']]).astype(np.float32) for r in limits_data['joint_limits']}
    model=SceneModel(D/'scene_geometry.json');checker=ProductionCarryChecker(model)
    original=model.poses(q);native={b:np.asarray(v,float) for b,v in entity['body_poses'].items()}
    corrections={b:native[b]@np.linalg.inv(original[b]) for b in native if '/robots/' in b}
    task=BeamAssistedState();task.phase=phase['name'];task.phase_index=0;task.phase_entry_step=0;task.step_counter=0
    task.get_current_phase_spec=lambda:phase;task._assisted_published_q={n:v.copy() for n,v in published.items()}
    task._ur5e_plumbers_last_arm_command_q={n:v.copy() for n,v in published.items()}
    task._assisted_event=lambda *a,**kw:None;tick=[raw['physics_step_index']]
    task._beam_physics_stamp=lambda:dict(valid=True,epoch=0,index=tick[0],time=tick[0]/240,dt=1/240)
    task.robots={}
    for n,r in raw['robots'].items():
        names=r['joint_names'];bounds=np.array([by_name.get(name,[-10,10]) for name in names],dtype=np.float32)
        view=NS(is_physics_handle_valid=lambda:True,get_dof_limits=lambda b=bounds:b[None].copy())
        task.robots[n]=NS(articulation=NS(dof_names=names,_articulation_view=view),controllers={'arm_joint_controller':NS(get_joint_subset=lambda n=n:NS(get_joint_positions=lambda:q[n][:6].copy()))})
    adapter=Adapter({});adapter._current_arm_q=lambda task,robot_name:q[robot_name][:6].copy()
    adapter._current_arm_dynamics=lambda **kw:dict(joint_velocity=np.zeros(6))
    adapter._gripper_command_value=lambda **kw:1.;adapter._debug_grasp_enabled=lambda:False
    path=[];collisions=[];numerical_contacts=0;complete=False;max_step=0.;goal=np.asarray(spec['prealign_joint_positions'])
    # Measured .08 response is taken from A36's original prealign delta/drive
    # gap. It is an explicit kinematic counterfactual, never changed dynamics.
    for age in range(3601):
        task.step_counter=age;task.phase_step_counter=age;tick[0]+=1
        action=adapter._prealign_action(phase_key=(0,),task=task,robot_name='franka_left',target_pose=dict(position=np.zeros(3),orientation=[1,0,0,0]),spec=spec)
        if action is None:complete=True;break
        assert '__local_skill_failure__' not in action,action
        previous=task._assisted_published_q['franka_left'].copy()
        bounded=task._assisted_guard_actions({'franka_left':action,'franka_right':{'arm_joint_controller':[published['franka_right'].tolist()]}})
        final={n:np.asarray(a['arm_joint_controller'][0]) for n,a in bounded.items()}
        max_step=max(max_step,float(np.max(abs(final['franka_left']-previous))))
        for fraction in (0.,.25,.5,.75,1.):
            trial={n:v.copy() for n,v in q.items()}
            for n in trial:trial[n][:6]+=fraction*(final[n]-trial[n][:6])
            fk=model.poses(trial);poses={**model.rest,**native}
            for b,correction in corrections.items():poses[b]=correction@fk[b]
            checker._scene_check(poses,phase['name'],age,(None,None))
            failures=[r for r in checker.checker.rows if r['depth']>1e-6]
            numerical_contacts+=len(checker.checker.rows)-len(failures)
            if failures:collisions.extend(failures);break
        if collisions:break
        q['franka_left'][:6]+=.08*(final['franka_left']-q['franka_left'][:6])
        if age%24==0:path.append(dict(age=age,actual_q=q['franka_left'][:6].tolist(),published_q=final['franka_left'].tolist()))
    error=float(np.max(abs(goal-q['franka_left'][:6])))
    expected_complete=bool(complete and (opt.baseline or error<=.05))
    report=dict(passed=expected_complete and not collisions,qualification_kind='joint_branch_prealign',native_physics=False,simulator_created=False,source_manifest={n:sha(C/n) for n in read(W/'baseline_manifest.json')},actual_recorded_run=D.name,actual_input_step=opt.step,exact_entity_count=44,original_three_camera_clock=clocks[0]['physics_stamp'],prealign_policy_returned_complete=complete,raw_requested_branch_reached=error<=.05,raw_joint_error_rad=error,physical_branch_counterexample=bool(opt.baseline and error>.05),completed_after_ticks=age,maximum_final_command_step_rad=max_step,full54=True,self_checked=True,mesh_samples=checker.checker.samples,geometry_failures=collisions,numerical_surface_contact_samples=numerical_contacts,kinematic_response=.08,path=path,configuration_sha256=sha(W/('base_configuration_layout505_config.json' if opt.baseline else 'actual_configuration_layout505_config.json')),raw_index_sha256=sha(D/'raw_offset_index.json'),scene_geometry_sha256=sha(D/'scene_geometry.json'),asset_joint_limits_sha256=sha(P/'D20_work/asset_joint_limits.json'),tool_sha256=sha(__file__),elapsed_s=time.monotonic()-begin,scope='Original production prealign and final task guard with recorded q/publication bias and all44 native bodies, five-fraction full54-self scene sweep. Subsequent state follows explicitly declared .08 kinematic response. Not physical dynamics, no grasp equivalence or checkpoint claim; live native limit API must be confirmed in short screen.')
    name=opt.output_name or ('baseline_control_replay.json' if opt.baseline else 'candidate_control_replay.json');target=W/name;assert not target.exists();save(target,report)
    print(json.dumps({k:report[k] for k in ['passed','raw_requested_branch_reached','raw_joint_error_rad','completed_after_ticks','mesh_samples','geometry_failures','elapsed_s']}));raise SystemExit(0 if report['passed'] else 1)
def pose(t):return dict(position=t[:3,3].copy(),orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]])
def mat(p):return pose_matrix(p['position'],p['orientation'])
cfg=read(W/('base_configuration_layout505_config.json' if opt.baseline else 'actual_configuration_layout505_config.json'));phase=next(p for p in cfg['phase_specs'] if p['name']==opt.phase)
spec=phase['local_skill'];robot=spec['robot'];part=spec['object'];step=opt.step
raw=select(D,'physics_boundary_trace.jsonl',step,step,'before_action')[0]
entity=select(D,'entity_geometry_trace.jsonl',step,step)[0]
clocks=[json.loads(line) for line in (D/'video_frame_clock.jsonl').open() if json.loads(line)['task_step_label']==step]
assert len(clocks)==3 and {r['camera'] for r in clocks}=={'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'}
assert all(r['physics_stamp']['valid'] and r['physics_stamp']['index']==raw['physics_step_index'] and abs(r['physics_stamp']['time']-raw['physics_time'])<1e-9 for r in clocks)
q0={n:np.asarray(r['joint_positions'][0],float) for n,r in raw['robots'].items()}
pub={n:np.asarray(next(a['joint_positions'] for a in r['sdk_submission']['actions'] if a.get('joint_indices')==list(range(6))),float) for n,r in raw['robots'].items()}
bias={n:pub[n]-q0[n][:6] for n in q0};assert np.max(abs(bias[robot]))>1e-4
model=SceneModel(D/'scene_geometry.json');checker=ProductionCarryChecker(model)
origin=model.poses(q0);tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
actual0={b:np.asarray(t,float) for b,t in entity['body_poses'].items()};assert len(actual0)==44
body=next(b for b in actual0 if b.endswith('/'+part))
events=select(D,'assisted_trace.jsonl',0,step)
recorded_events=events
assert any(e['step']==step and e.get('physics_stamp')==clocks[0]['physics_stamp'] for e in events)
capture=next(e for e in events if e.get('event')=='attach_measured_pose' and e.get('object')==part)
qc={n:x.copy() for n,x in q0.items()};qc[robot]=np.asarray(capture['full_joint_positions'],float)
relative=pose(np.linalg.inv(model.poses(qc)[tcp])@pose_matrix(capture['position'],capture['orientation']))
native_relation=np.linalg.inv(origin[tcp])@actual0[body]
initial_residual=validate_model_native_residual(pose(actual0[body]),object_pose_from_tcp(pose(origin[tcp]),relative['position'],relative['orientation']))
goal={k:np.asarray(v,float) for k,v in cfg['target_poses'][spec['target_object_target']].items()}
target=copy.deepcopy(goal) # Derived TCP goal is supplied independently below.
driver=next(a['joint_positions'][0] for a in raw['robots'][robot]['sdk_submission']['actions'] if a.get('joint_indices')==[6])
rc=next(r for r in cfg['robots'] if r['name']==robot);opened=rc['gripper_open_position'];closed=rc['gripper_closed_position'];openness=1-(driver-opened)/(closed-opened)
assert 0<=openness<=1 and abs(opened+(1-openness)*(closed-opened)-driver)<1e-12
attachment=dict(mode='fixed_joint',robot_name=robot,joint_path=capture['joint_path'],attach_step=capture['step'],attach_spec={'beam_assisted':True},assisted_hold_openness=openness)
tracked={part:dict(attachment=attachment)}
q={n:x.copy() for n,x in q0.items()};fresh=dict(valid=True,epoch=0,index=raw['physics_step_index'],time=raw['physics_time'],dt=raw['physics_dt'])
observed=pose(actual0[body]);actual=dict(actual0);J=None;age=0;completed=[];events=[];geometry=[];ik_calls=0
class Fixture(Adapter):
    def _current_robot_pose(self,**kw):return pose(model.poses(q)[tcp])
    def _object_pose(self,**kw):return copy.deepcopy(observed)
    def _target_object_pose(self,**kw):return copy.deepcopy(goal)
    def _target_pose(self,**kw):return pose(mat(goal)@np.linalg.inv(mat(relative)))
    def _object_tcp_relative_pose(self,**kw):return relative['position'].copy(),relative['orientation'].copy()
    def _current_arm_q(self,*a):return q[robot][:6].copy()
    def _grasp_command_fk_pose(self,**kw):
        trial={n:x.copy() for n,x in q.items()};trial[robot][:6]=kw['command_q'];return pose(model.poses(trial)[tcp])
    def _local_grasp_ik(self, *, task, robot_name, target_pose, reference_q, spec, joint_radius):
        # Same production bounded refinement; replace only the FK oracle with
        # the already declared calibrated nonlinear joint graph.
        def forward(x):
            trial={n:v.copy() for n,v in q.items()};trial[robot][:6]=x
            t=model.poses(trial)[tcp];return t[:3,3],t[:3,:3]
        return self._bounded_local_pose_ik(forward=forward,reference_q=reference_q,
            target_position=target_pose['position'],target_orientation=target_pose['orientation'],
            position_tolerance=float(spec['ik_position_tolerance']),
            orientation_tolerance=float(spec['ik_orientation_tolerance']),joint_radius=joint_radius)
    def _object_motion_detail(self,**kw):return dict(valid=True,linear_speed=0.,angular_speed=0.)
    def _gripper_command_value(self,**kw):return openness
    def _solve_ik(self,**kw):
        global J,ik_calls
        ik_calls+=1;t=mat(kw['target_pose']);seed=q[robot][:6].copy() if kw.get('warm_start') is None else np.asarray(kw['warm_start'],float)
        assert seed.shape==(6,) and np.all(np.isfinite(seed))
        def residual(x):
            trial={n:v.copy() for n,v in q.items()};trial[robot][:6]=x;f=model.poses(trial)[tcp]
            return np.r_[(f[:3,3]-t[:3,3])*5,Rotation.from_matrix(t[:3,:3].T@f[:3,:3]).as_rotvec()]
        solution=least_squares(residual,seed,bounds=(seed-.25,seed+.25),max_nfev=30,
            ftol=1e-11,xtol=1e-11,gtol=1e-11)
        result=solution.x
        if not solution.success or not np.all(np.isfinite(result)) or np.linalg.norm(residual(result))>2e-5:
            raise ValueError('Declared bounded nonlinear IK proxy failed convergence: '+repr(dict(age=age,status=solution.status,residual=float(np.linalg.norm(residual(result))))))
        return result
adapter=Fixture({});task=BeamAssistedState();index=cfg['phase_specs'].index(phase)
task.phase=phase['name'];task.phase_index=index;task.phase_entry_step=step;task.step_counter=step;task.phase_step_counter=0
task._attachments={part:attachment};task._assisted_published_q={n:x.copy() for n,x in pub.items()};task._assisted_published_q_stamps={n:dict(epoch=0,index=fresh['index']-1) for n in q}
task._beam_physics_stamp=lambda:dict(fresh)
def motion(*a,**kw):
    result=dict(**observed,motion_valid=True,physics_handle_valid=True,body_path=body,motion_view_id=19,linear_speed=0.,angular_speed=0.)
    if opt.recorded_native_motion:
        b=next(b for b in raw['bodies'] if b['path']==body);com=b['center_of_mass'];assert b['valid'] and com['valid']
        result.update(motion_view_id=b['view_id'],linear_speed=float(np.linalg.norm(b['linear_velocities'][0])),
            angular_speed=float(np.linalg.norm(b['angular_velocities'][0])),linear_velocity=b['linear_velocities'][0],
            center_of_mass_valid=True,center_of_mass_position_frame=com['position_frame'],center_of_mass_orientation_order=com['orientation_order'],
            center_of_mass_position=np.asarray(com['world_positions'][0]),center_of_mass_orientation=np.asarray(com['world_orientations'][0]))
    return result
task._assisted_native_body_motion=motion;task._physical_hold_valid=lambda *a,**kw:True
task._assisted_event=lambda event,**kw:events.append(dict(event=event,step=task.step_counter,**kw))
task.get_current_phase_spec=lambda:phase
bounds_rows=[e for e in select(D,'assisted_trace.jsonl',0,step) if e.get('event')=='native_arm_joint_limits']
task.robots={n:NS(controllers={'arm_joint_controller':NS(get_joint_subset=lambda n=n:NS(get_joint_positions=lambda:q[n][:6].copy()))}) for n in q}
if spec.get('beam_assisted_native_joint_limits') or phase.get('beam_assisted_native_joint_limits'):
    limits_data=read(P/'D20_work/asset_joint_limits.json')
    by_name={r['name']:np.radians([r['lower_degrees'],r['upper_degrees']]).astype(np.float32) for r in limits_data['joint_limits']}
    for n,rr in raw['robots'].items():
        bounds=np.array([by_name.get(name,[-10,10]) for name in rr['joint_names']],dtype=np.float32)
        view=NS(is_physics_handle_valid=lambda:True,get_dof_limits=lambda b=bounds:b[None].copy())
        task.robots[n].articulation=NS(dof_names=rr['joint_names'],_articulation_view=view)

production_cad_calls=audit_cad_calls=0
minimum_right_payload_gap=float('inf');geometric_path_completed=False;geometric_steps=[]
def cad(**kw):
    global production_cad_calls,audit_cad_calls,minimum_right_payload_gap
    targets=kw.pop('audit_targets',None);external=targets is not None
    result=checker.check(native_poses=actual,actual_q=q,other_targets=targets if external else task._assisted_batch_arm_candidates,support=phase['beam_assisted_visual_support'],phase=phase['name'],**kw)
    if opt.carry_path:
        import fcl
        c=checker.checker
        for i,m in enumerate(c.meshes):
            if m.get('object')!=part:continue
            for j,n in enumerate(c.meshes):
                if n.get('robot')!='franka_right':continue
                r=fcl.CollisionResult();hit=fcl.collide(c.shapes[i],c.shapes[j],fcl.CollisionRequest(enable_contact=True,num_max_contacts=8),r)
                distance=-max((float(x.penetration_depth) for x in r.contacts),default=0) if hit else float(fcl.distance(c.shapes[i],c.shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
                minimum_right_payload_gap=min(minimum_right_payload_gap,distance)
    if external:audit_cad_calls+=1
    else:production_cad_calls+=1
    geometry.append(dict(age=age,passed=result[0],detail=result[1],independent_after_final_publication=external));return result
task._assisted_production_cad_guard=cad
adapter._mark_complete=lambda **kw:completed.append(dict(age=age,detail=kw['detail']))
args=dict(task=task,robot_name=robot,phase_spec=phase,skill_spec=spec,tracked_robots={},tracked_objects=tracked)

# All repeated observations remain the exact original measured state. Only the
# declared control clock advances. This checks commands, never dynamic progress.
trace=[];baseline_pub=pub[robot].copy();baseline_tcp=adapter._grasp_command_fk_pose(task=task,robot_name=robot,command_q=baseline_pub)
recorded_clearance_seed=None
if opt.recorded_clearance_state:
    assert not opt.baseline and not opt.carry_path and not opt.geometric_waypoints
    selected=[e for e in recorded_events if e.get('phase')==phase['name']]
    entry=next(e for e in selected if e.get('event')=='incremental_drive_reference')
    latest=[e for e in selected if e.get('event')=='absolute_object_servo'][-1]
    assert latest['clearance_stage']=='turn'
    phase_key=(id(task),task.phase_index,task.phase_entry_step,robot,spec['name'])
    *_,state=adapter._assisted_object_servo_context(phase_key=phase_key,task=task,robot_name=robot,spec=spec,tracked_robots={},tracked_objects=tracked)
    ep={k:np.asarray(v,float) for k,v in entry['native_entry_pose'].items()}
    state['clearance_path']=dict(entry_position=ep['position'],entry_orientation=ep['orientation'],goal_position=goal['position'].copy(),goal_orientation=goal['orientation'].copy(),height=max(float(ep['position'][2]),float(goal['position'][2]))+spec['beam_assisted_staged_clearance']['height_padding_m'],stage='turn',clock=(fresh['epoch'],fresh['index']-1))
    state['incremental_drive_reference']={k:np.asarray(v,float) for k,v in entry['retained_entry_bias'].items()}
    state['phase_handoff_start']=dict(fresh,index=fresh['index']-spec['beam_assisted_phase_handoff_ticks'])
    state['phase_handoff_seed_q']=pub[robot].copy()
    recorded_clearance_seed=dict(entry_event=entry,latest_event=latest,scope='Reconstructed recorded path/drive references only; static measured input, no restored physics/contact/solver state.')
for age in range(opt.carry_max_ticks if opt.carry_path else 27):
    task.step_counter=step+age;task.phase_step_counter=age
    fresh.update(index=raw['physics_step_index']+age,time=raw['physics_time']+age*raw['physics_dt'])
    action=adapter.act(**args);assert not action.get('__local_skill_failure__'),action
    guarded=task._assisted_guard_actions({robot:action,'franka_right':{'arm_joint_controller':[pub['franka_right'].tolist()]}})
    final=np.asarray(guarded[robot]['arm_joint_controller'][0])
    if opt.audit_final_cad and not spec.get('beam_assisted_production_cad_guard'):
        targets={n:np.asarray(v['arm_joint_controller'][0]) for n,v in guarded.items()}
        allowed,detail=cad(robot_name=robot,object_name=part,command_q=final,current_q=q[robot][:6],
            relative=(relative['position'],relative['orientation']),physics_stamp=dict(fresh),audit_targets=targets)
        assert allowed,detail
    final_tcp=adapter._grasp_command_fk_pose(task=task,robot_name=robot,command_q=final)
    delta=final_tcp['position']-baseline_tcp['position']
    trace.append(dict(age=age,delta_tcp_m=delta.tolist(),q=final.tolist()))
    if age==0 and spec.get('beam_assisted_phase_handoff_ticks') and not opt.recorded_clearance_state:assert np.max(abs(final-baseline_pub))<1e-12
    pub[robot]=final
    if opt.carry_path and age%128==0:
        states=list(getattr(task,'_beam_assisted_object_servo_states',{}).values())
        print(json.dumps(dict(age=age,observed={k:np.asarray(v).tolist() for k,v in observed.items()},clearance_stages=[x.get('clearance_path',{}).get('stage') for x in states],minimum_right_payload_gap_m=minimum_right_payload_gap)),flush=True)
    if opt.carry_path:
        # Explicit first-order kinematic response; native load/contact/solver
        # transients are deliberately outside this offline qualification.
        q[robot][:6]+=.12*(final-q[robot][:6]-bias[robot])
        fk=model.poses(q)
        for body_name in actual:
            if '/robots/' in body_name:actual[body_name]=actual0[body_name]@np.linalg.inv(origin[body_name])@fk[body_name]
        actual[body]=fk[tcp]@native_relation
        observed=pose(actual[body])
        if completed:break
if opt.geometric_waypoints:
    import fcl
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import staged_clearance_target
    from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error
    entry=pose(actual0[body]);high=max(entry['position'][2],goal['position'][2])+spec['beam_assisted_staged_clearance']['height_padding_m']
    starts=entry
    waypoints=[('lift',dict(position=np.array([*entry['position'][:2],high]),orientation=entry['orientation'])),
               ('turn',dict(position=np.array([*entry['position'][:2],high]),orientation=goal['orientation'])),
               ('transfer',dict(position=np.array([*goal['position'][:2],high]),orientation=goal['orientation'])),
               ('lower',goal)]
    corrections={b:actual0[b]@np.linalg.inv(origin[b]) for b in actual0 if '/robots/' in b}
    for label,end_pose in waypoints:
        distance,angle=pose_error(starts['position'],starts['orientation'],end_pose['position'],end_pose['orientation'])
        count=max(1,int(np.ceil(max(distance/.002,angle/.01))))
        from toolkits.factory_dual_franka_assembly.beam_assisted_servo import interpolate_object_pose
        for sub in range(1,count+1):
            planned=interpolate_object_pose(starts,end_pose,sub/count)
            target_tcp=mat(planned)@np.linalg.inv(native_relation)
            solved=adapter._solve_ik(task=task,robot_name=robot,target_pose=pose(target_tcp),warm_start=q[robot][:6].copy(),spec=spec)
            arm_bounds=np.array([by_name[n] for n in raw['robots'][robot]['joint_names'][:6]],dtype=np.float32)
            assert np.all(solved>=arm_bounds[:,0]) and np.all(solved<=arm_bounds[:,1]), 'Planned raw branch violates actual asset joint range'
            previous=q[robot][:6].copy();delta=solved-previous
            split=max(1,int(np.ceil(np.max(abs(delta))/.01)))
            for piece in range(1,split+1):
                q[robot][:6]=previous+piece/split*delta
                for fraction in (0.,.25,.5,.75,1.):
                    trial={n:v.copy() for n,v in q.items()};trial[robot][:6]=previous+(piece-1+fraction)/split*delta
                    fk=model.poses(trial);poses={**model.rest,**actual0}
                    for bn,correction in corrections.items():poses[bn]=correction@fk[bn]
                    poses[body]=fk[tcp]@native_relation
                    checker._scene_check(poses,phase['name']+':planned_'+label,len(geometric_steps),(robot,part))
                    c=checker.checker;bad=[r for r in c.rows if r['depth']>1e-6]
                    if bad:raise ValueError('Planned geometry collision: '+repr(bad[0]))
                    if any(v['signed_separation'] < -1e-6 for v in c.grasp_contacts):raise ValueError('Planned intended grasp overlap')
                    for i,m in enumerate(c.meshes):
                        if m.get('object')!=part:continue
                        for j,n in enumerate(c.meshes):
                            if n.get('robot')!='franka_right':continue
                            rr=fcl.CollisionResult();hit=fcl.collide(c.shapes[i],c.shapes[j],fcl.CollisionRequest(enable_contact=True,num_max_contacts=8),rr)
                            d=-max((float(x.penetration_depth) for x in rr.contacts),default=0) if hit else float(fcl.distance(c.shapes[i],c.shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
                            minimum_right_payload_gap=min(minimum_right_payload_gap,d)
            geometric_steps.append(dict(stage=label,position=planned['position'].tolist(),q=q[robot][:6].tolist()))
        starts=end_pose
    geometric_path_completed=True
# The exact A27 final over-limit guard input is still rejected. No neighbouring
# full-entity frame is claimed for this negative.
rejected=None
if not opt.adjacent_only and not opt.carry_path and not opt.geometric_waypoints:
    failure=next(e for e in select(D,'assisted_trace.jsonl',33351,33351) if e.get('event')=='object_model_residual_rejected')
    q[robot][:6]=np.asarray(failure['actual_q'],float)
    observed={k:np.asarray(v,float) for k,v in failure['native_object_pose'].items()}
    fresh.update(failure['physics_stamp'])
    task.step_counter=failure['step'];task.phase_step_counter=task.step_counter-step
    oldcalls=checker.calls;oldik=ik_calls;rejected=False
    try:
        result=adapter.act(**args)
        rejected=bool(result.get('__local_skill_failure__') and 'model budget' in repr(result))
    except ValueError as exc:rejected='model budget' in str(exc)
    assert rejected and checker.calls==oldcalls and ik_calls==oldik
xy=float(np.linalg.norm(np.asarray(trace[-1]['delta_tcp_m'])[:2]))
if not opt.adjacent_only and not opt.carry_path and not opt.geometric_waypoints:
    if opt.baseline:assert xy>1e-4
    else:assert xy<2e-6 and trace[-1]['delta_tcp_m'][2]>0
report=dict(passed=True,native_physics=False,checkpoint_used=False,simulator_created=False,
    source_root=str(C),source_manifest={n:sha(C/n) for n in read(W/'baseline_manifest.json')},
    configuration_file_sha256=sha(W/('base_configuration_layout505_config.json' if opt.baseline else 'actual_configuration_layout505_config.json')),
    scene_geometry_sha256=sha(D/'scene_geometry.json'),raw_index_sha256=sha(D/'raw_offset_index.json'),
    selected_observation_sha256=hashlib.sha256(json.dumps(dict(native=raw,entity=entity,cameras=clocks),sort_keys=True).encode()).hexdigest(),
    actual_recorded_run=D.name,actual_input_step=step,requested_control_phase=phase['name'],adjacent_counterfactual=opt.adjacent_only,exact_entity_count=44,original_clock=clocks[0]['physics_stamp'],
    scope=('Production entry27command counterfactual from exact recorded native input; then independent small-step nonlinear IK geometric waypoint path with five-fraction full54-self sweeps. Geometric plan is not simulated execution, tracking-envelope certification or loaded completion. Native progress remains untested.' if opt.geometric_waypoints else 'Recorded native initial q, publication bias, full44 frames and frozen grasp with production controller/final full54-self guard; subsequent state follows declared .12 first-order nonlinear kinematic response with the initial measured published-minus-actual joint bias retained. Not dynamics, cooked-contact, checkpoint equivalence or full assembly proof.' if opt.carry_path else 'Actual recorded inputs, original frozen relation and nonlinear calibrated FK/IK proxy; repeated state is static counterfactual. An adjacent phase is explicitly counterfactual and does not supply missing terminal native/entity observations. No dynamic improvement or original episode equivalence claim.'),
    original_over_limit_rejected_before_IK_CAD=rejected,full54=True,self_checked=True,qualification_kind='staged_clearance_object' if opt.carry_path or opt.geometric_waypoints else 'static_transition',minimum_right_payload_gap_m=minimum_right_payload_gap if opt.carry_path or opt.geometric_waypoints else None,geometric_path_completed=geometric_path_completed,production_entry_chain_verified=len(trace)==27,geometric_steps=geometric_steps,carry_path_completed=bool(completed),kinematic_response=.12 if opt.carry_path else None,retained_measured_joint_tracking_bias=bool(opt.carry_path),
    production_CAD_calls=production_cad_calls,independent_final_CAD_calls=audit_cad_calls,
    recorded_native_motion=opt.recorded_native_motion,recorded_clearance_seed=recorded_clearance_seed,
    descent_admission_observed=any(bool((e.get('descent_admission') or {}).get('ready')) for e in events),
    final_publication_axial_delta_m=float(trace[-1]['delta_tcp_m'][2]),
    guard_calls=checker.calls,mesh_samples=checker.samples,CAD_wall_s=checker.wall_seconds,
    final_publication_lateral_delta_m=xy,trace=trace,
    geometry_failures=[g for g in geometry if not g['passed']],final_observed={k:np.asarray(v).tolist() for k,v in observed.items()},goal_object_pose={k:np.asarray(v).tolist() for k,v in goal.items()},clearance_stages=[x.get('clearance_path',{}).get('stage') for x in getattr(task,'_beam_assisted_object_servo_states',{}).values()],tool_sha256=sha(__file__),elapsed_s=time.monotonic()-begin)
report['passed']=not report['geometry_failures'] and (not opt.carry_path or bool(report['carry_path_completed'] and report['minimum_right_payload_gap_m']>=.005)) and (not opt.geometric_waypoints or bool(geometric_path_completed and minimum_right_payload_gap>=.005))
name=opt.output_name or ('adjacent_control_replay.json' if opt.adjacent_only else ('baseline_control_replay.json' if opt.baseline else 'candidate_control_replay.json'))
assert Path(name).name==name and name.endswith('.json')
path=W/name
assert not path.exists();save(path,report)
print(json.dumps({k:report[k] for k in ['passed','carry_path_completed','minimum_right_payload_gap_m','guard_calls','mesh_samples','clearance_stages','elapsed_s']}))

raise SystemExit(0 if report['passed'] else 1)
