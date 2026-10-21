"""Isolated fourth-part fixture and passive host-wall profiler.

The fixture starts a new physical episode from explicit geometry and zero initial
velocities. It is not a checkpoint or an equivalent continuation. The full normal
worker, final publication guard, geometry and camera checks are reused unchanged.
"""
from pathlib import Path
import argparse,copy,hashlib,json,os,runpy,sys,time


def apply_fixture(config, fixture, baseline):
    for robot in config.robots:
        row=fixture['robots'][robot.name]
        robot.initial_joint_positions=dict(zip(row['joint_names'],row['joint_positions'][0]))
    for obj in config.objects:
        if obj.name in fixture['objects']:
            pose=fixture['objects'][obj.name]
            obj.position=tuple(pose['position']);obj.orientation=tuple(pose['orientation'])
        if obj.name=='fabrica_beam_1' and fixture.get('solver_position_iterations') is not None:
            obj.solver_position_iteration_count=int(fixture['solver_position_iterations'])
    phase=next(p for p in config.phase_specs if p['name']==fixture.get('stop_phase','assemble_02_part_1_assembly_clearance'))
    if baseline:phase['local_skill'].pop('beam_assisted_incremental_clearance',None)
    return config


def prepare(work, branch_screen=False, probe_object='fabrica_beam_1'):
    import numpy as np
    from scipy.spatial.transform import Rotation
    from index_run import select
    from beam_workflow import save,sha
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration
    fifth=probe_object=='fabrica_beam_0'
    assert not (fifth and branch_screen)
    prefix='assemble_03_part_0' if fifth else 'assemble_02_part_1'
    p=work.parent;run=p/'runs'/('A38_beam_full' if fifth else 'A36_beam_full' if branch_screen else 'A27_beam_full')
    step=30992 if branch_screen else 31984
    if fifth:
        result=json.loads((run/'collect_results.json').read_text())[0]
        entry=next(e['step_counter'] for e in result['phase_transition_history'] if e.get('to_phase')==prefix+'_move_above')
        entities=select(run,'entity_geometry_trace.jsonl',entry,entry+64)
        natives={r['task_step']:r for r in select(run,'physics_boundary_trace.jsonl',entry,entry+64,'before_action')}
        for entity in entities:
            candidate_step=entity['step'];row=natives.get(candidate_step)
            clocks=select(run,'video_frame_clock.jsonl',candidate_step,candidate_step)
            if row and row['phase']==prefix+'_move_above' and len(clocks)==3 and all(c['physics_stamp']['index']==row['physics_step_index'] for c in clocks):
                step=candidate_step;break
        else:raise ValueError('No complete pregrasp native/entity/camera clock for selected fifth fixture')
    else:
        row=select(run,'physics_boundary_trace.jsonl',step,step,'before_action')[0]
        entity=select(run,'entity_geometry_trace.jsonl',step,step)[0]
        clocks=select(run,'video_frame_clock.jsonl',step,step)
    assert len(clocks)==3 and len(entity['body_poses'])==44
    assert all(c['physics_stamp']['index']==row['physics_step_index'] and abs(c['physics_stamp']['time']-row['physics_time'])<1e-9 for c in clocks)
    objects={}
    for path,t in entity['body_poses'].items():
        name=path.rsplit('/',1)[-1]
        if name.startswith('fabrica_beam_'):
            t=np.asarray(t,float);objects[name]=dict(position=t[:3,3].tolist(),orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]].tolist())
    assert len(objects)==5
    fixture=dict(schema='assisted_isolated_selected_part_fixture_v2',object=probe_object,recorded_run=run.name,recorded_step=step,
        recorded_clock=clocks[0]['physics_stamp'],robots=row['robots'],objects=objects,
        start_phase=prefix+'_move_above' if branch_screen or fifth else prefix+'_close_and_attach',stop_phase=prefix+'_assembly_clearance',
        stop_clearance_stage='__whole_clearance__' if branch_screen or fifth else 'turn',maximum_diagnostic_steps=15000 if branch_screen or fifth else 3600,
        installed=['fabrica_beam_6','fabrica_beam_3','fabrica_beam_2']+(['fabrica_beam_1'] if fifth else []),
        original_entity_count=44,three_view_clock_join=True,initial_velocities='new-episode zero defaults; recorded velocities are not restored',
        equivalence_claim=False,checkpoint_restore=False,full_assembly_acceptance=False,
        scope='New local diagnostic scene. Full original phase configuration/targets/criteria retained; earlier history is unvisited. Warm closing/capture/lift and phase handoff execute original production code.',
        source_inputs={n:sha(run/n) for n in ['task_config_snapshot.json','scene_geometry.json','raw_offset_index.json']})
    if branch_screen or fifth:
        from bounded_scene_http_probe import fetch
        fixture.update(support_warmup_phase='base_6_visual_support',
            stop_after_phase=fixture['stop_phase'],solver_position_iterations=None if fifth else 64,
            bounded_primary_scene_probe=fetch(),
            scope=f'New empty-pregrasp physical scene for {probe_object}, initialized from original {run.name} full-body observation. Actual support requalification, raw-branch prealignment, grasp and full clearance run production code. Recorded velocities/hidden solver/contact state are not restored; no checkpoint equivalence or full assembly claim.')
    target=work/'isolated_fixture.json';assert not target.exists();save(target,fixture)
    reports={}
    for baseline in ((False,) if branch_screen or fifth else (True,False)):
        cfg=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=505,domain_randomization_enabled=True,attach_runtime_cameras=True)
        cfg=apply_fixture(cfg,fixture,baseline)
        check=assert_assisted_configuration(cfg);assert check['passed']
        key='baseline' if baseline else 'candidate'
        save(work/f'probe_{key}_configuration.json',cfg.model_dump(mode='python',serialize_as_any=True))
        reports[key]=check
    save(work/'probe_preparation.json',dict(passed=True,fixture_sha256=sha(target),configurations=reports,simulator_created=False,tool_sha256=sha(__file__),
        qualification_kind='staged_clearance_object' if fifth else 'joint_branch_prealign' if branch_screen else 'legacy_lift',
        whole_free_clearance_screen=branch_screen or fifth,http_probe_tool_sha256=sha(Path(__file__).parent/'bounded_scene_http_probe.py')))
    print(json.dumps(dict(passed=True,scene=probe_object+' isolated',recorded_step=step,configs=list(reports))))


def execute():
    source=Path(os.environ['BEAM_ROUTE_SOURCE']);sys.path.insert(0,str(source))
    p=Path(os.environ['BEAM_ASSISTED_ROOT']);sys.path.insert(1,str(p/'tools'))
    out=Path(os.environ['BEAM_CONTACT_TRACE_PATH']).parent
    work=Path(os.environ['BEAM_PROBE_WORK']);fixture=json.loads(Path(os.environ.get('BEAM_PROBE_FIXTURE',str(work/'isolated_fixture.json'))).read_text())
    baseline=os.environ['BEAM_PROBE_BASELINE']=='1'
    from toolkits.factory_dual_franka_assembly import generate_demos as demo
    original_collect=demo._build_collect_worker_task_configs
    def collect(*args,**kwargs):
        cfgs=original_collect(*args,**kwargs);assert len(cfgs)==1
        return [apply_fixture(cfgs[0],fixture,baseline)]
    demo._build_collect_worker_task_configs=collect
    # Nested timers are reported with both inclusive and exclusive host wall.
    stats={};stack=[]
    def persist_timing(boundary):
        from beam_workflow import save,sha
        save(out/'short_segment_timing.json',dict(schema='assisted_nested_host_wall_v1',elapsed_s=time.perf_counter()-start,
            timings=copy.deepcopy(stats),saved_at=boundary,
            scope='Passive nested host-wall timers. Exclusive values exclude timed children; asynchronous GPU execution is not independently isolated. Initialization/ACK/tail/media included in total elapsed, not a prediction of full-run speedup.',
            tool_sha256=sha(__file__),diagnostic_scene=True))
    def wrap(owner,name,label):
        original=getattr(owner,name)
        def measured(*args,**kwargs):
            state=[time.perf_counter(),0.];stack.append(state)
            try:return original(*args,**kwargs)
            finally:
                elapsed=time.perf_counter()-state[0];stack.pop()
                r=stats.setdefault(label,dict(calls=0,inclusive_s=0.,exclusive_s=0.))
                r['calls']+=1;r['inclusive_s']+=elapsed;r['exclusive_s']+=elapsed-state[1]
                if stack:stack[-1][1]+=elapsed
        setattr(owner,name,measured)
    original_build=demo._build_env
    def build(*args,**kwargs):
        env=original_build(*args,**kwargs);original_reset=env.reset
        if fixture.get('bounded_primary_scene_probe'):
            from bounded_scene_http_probe import install
            install(fixture,out)
        def reset(*a,**kw):
            result=original_reset(*a,**kw)
            if not env.runner.current_tasks:return result
            task=next(iter(env.runner.current_tasks.values()))
            from beam_workflow import save
            for name in fixture['installed']:
                target='part_'+name.rsplit('_',1)[-1]+'_assembled'
                task._assisted_lock(name,target,{})
            task._set_phase([s['name'] for s in task.phase_specs].index(fixture.get('support_warmup_phase',fixture['start_phase'])),reason='isolated-diagnostic-initialization')
            task.phase_history=[task.phase]
            task._assisted_published_q={n:copy.deepcopy(next(x['joint_positions'] for x in r['sdk_submission']['actions'] if x.get('joint_indices')==list(range(6)))) for n,r in fixture['robots'].items()}
            stamp=task._beam_physics_stamp();task._assisted_published_q_stamps={n:dict(epoch=stamp['epoch'],index=stamp['index']-1) for n in fixture['robots']}
            actual={n:dict(joint_names=list(r.articulation.dof_names),joint_positions=r.articulation.get_joint_positions().tolist()) for n,r in task.robots.items()}
            save(out/'probe_initialization.json',dict(fixture=fixture,actual_robot_initialization=actual,physics_stamp=stamp,phase=task.phase,
                scope='New episode diagnostic initialization; no old solver state/contact history/velocity/hidden caches restored. No state setters after initialization.'))
            wrap(task,'get_observations','observation_readback_and_task_update')
            return result
        env.reset=reset
        original_close=env.close
        def close(*a,**kw):
            persist_timing('immediately before original Env.close / SimulationApp.close')
            return original_close(*a,**kw)
        env.close=close
        wrap(env.runner,'step','environment_step');wrap(env.runner,'get_obs','runner_observation')
        wrap(env.runner._world,'step','physics_and_render_step');wrap(env.runner._world,'render','render')
        wrap(env.runner._world._physics_context,'_step','explicit_nonrender_physics_step')
        return env
    demo._build_env=build
    original_policy=demo.DualFrankaAssemblyDemoPolicy.act
    def policy(self,task,*args,**kwargs):
        if not getattr(task,'_probe_solver_readback_done',False):
            from beam_workflow import save
            from isaacsim.core.utils.stage import get_current_stage
            body,path=task._assisted_body_path(fixture.get('object','fabrica_beam_1'))
            stage=get_current_stage();prim=stage.GetPrimAtPath(path)
            readback={'part_path':path,'USD_position_iterations':prim.GetAttribute('physxRigidBody:solverPositionIterationCount').Get(),
                'USD_velocity_iterations':prim.GetAttribute('physxRigidBody:solverVelocityIterationCount').Get(),
                'native_API':{},'errors':{},'effective_island_iterations':'not directly observed'}
            for key,owner in [('part',body.unwrap()),('left_articulation',task.robots['franka_left'].articulation)]:
                for method in ['get_solver_position_iteration_count','get_solver_velocity_iteration_count']:
                    try:readback['native_API'][key+'.'+method]=int(getattr(owner,method)())
                    except Exception as exc:readback['errors'][key+'.'+method]=str(exc)
            save(out/'solver_runtime_readback.json',readback)
            if fixture.get('solver_position_iterations') is not None:
                assert readback['USD_position_iterations']==fixture['solver_position_iterations']
            task._probe_solver_readback_done=True
        warmup=fixture.get('support_warmup_phase')
        if warmup and not getattr(task,'_probe_support_requalified',False):
            names=[p['name'] for p in task.phase_specs]
            if task.phase_index>names.index(warmup) and not task.failed:
                demo._write_json(out/'support_warmup_qualification.json',dict(
                    completed_original_phase=warmup,transition=copy.deepcopy(task.phase_transition_history[-1]),
                    physics_stamp=task._beam_physics_stamp(),step=int(task.step_counter),
                    retained_final_arm_targets=copy.deepcopy(task._assisted_published_q),
                    scope='Original right visual support close controller and original completion predicate actually ran in the new numerical scene. Its accepted commands/caches are retained; only diagnostic phase routing jumps to the selected pregrasp.'))
                task._set_phase(names.index(fixture['start_phase']),reason='isolated-support-requalification-complete')
                task._probe_support_requalified=True
        if fixture.get('stop_after_phase') and not task.failed:
            names=[p['name'] for p in task.phase_specs]
            if task.phase_index>names.index(fixture['stop_after_phase']):
                task._beam_assisted_boundary_stop=dict(completed_phase=fixture['stop_after_phase'],completed_substage=fixture.get('stop_label','all_clearance'),
                    step=int(task.step_counter),physics_stamp=task._beam_physics_stamp(),reason='isolated-'+fixture.get('stop_label','full-clearance')+'-screen-complete',success=False)
                demo._write_json(out/'bounded_stop.json',task._beam_assisted_boundary_stop);return {}
        if task.phase==fixture['stop_phase']:
            states=getattr(task,'_beam_assisted_object_servo_states',{}).values()
            if any(s.get('clearance_path',{}).get('stage')==fixture['stop_clearance_stage'] for s in states):
                task._beam_assisted_boundary_stop=dict(completed_phase=task.phase,completed_substage='lift',step=int(task.step_counter),
                    physics_stamp=task._beam_physics_stamp(),reason='isolated-lift-screen-complete',success=False)
                demo._write_json(out/'bounded_stop.json',task._beam_assisted_boundary_stop);return {}
        if task.step_counter>=fixture['maximum_diagnostic_steps'] and not task.failed:
            task._set_terminal_state('failed',reason='isolated-diagnostic-step-limit',status='failed')
            return {}
        return original_policy(self,task,*args,**kwargs)
    demo.DualFrankaAssemblyDemoPolicy.act=policy
    wrap(demo.DualFrankaAssemblyDemoPolicy,'act','production_policy')
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
    for name in ('_solve_ik','_local_grasp_ik','_grasp_command_fk_pose'):
        wrap(UR5eAssemblyAtomicSkillAdapter,name,'control'+name)
    from visual_mesh_check import ProductionCarryChecker
    wrap(ProductionCarryChecker,'check','full_final_publication_CAD')
    from toolkits.factory_dual_franka_assembly import beam_physics_diagnostics
    wrap(beam_physics_diagnostics,'sample_boundary','native_diagnostics_readback_and_serialization')
    import cv2
    wrap(cv2,'imwrite','PNG_encoding_and_write');wrap(json,'dumps','JSON_serialization')
    wrap(demo.LiveRolloutVideoRecorder,'record','video_record')
    start=time.perf_counter()
    try:runpy.run_path(str(p/'tools/assisted_worker.py'),run_name='__main__')
    finally:
        if not (out/'short_segment_timing.json').exists():persist_timing('worker finally')


def prepare_solver_profile(work, directory, count, support_warmup=False, bounded_scene=False, full_clearance=False):
    from beam_workflow import save,sha
    from toolkits.factory_dual_franka_assembly.scene_builder import build_dual_franka_assembly_episode
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration
    assert count==64 and not directory.exists();directory.mkdir()
    base=json.loads((work/'isolated_fixture.json').read_text());fixture=copy.deepcopy(base)
    fixture['solver_position_iterations']=count
    if support_warmup:fixture['support_warmup_phase']='base_6_visual_support'
    if full_clearance:
        fixture['stop_clearance_stage']='__whole_clearance__';fixture['stop_after_phase']=fixture['stop_phase'];fixture['maximum_diagnostic_steps']=15000
    if bounded_scene:
        from bounded_scene_http_probe import fetch
        fixture['bounded_primary_scene_probe']=fetch()
    save(directory/'isolated_fixture.json',fixture)
    cfg=build_dual_franka_assembly_episode(recipe='fabrica_beam_ur5e_assisted',scene_profile='taoyuan_grscenes_tabletop',seed=0,layout_seed=505,domain_randomization_enabled=True,attach_runtime_cameras=True)
    apply_fixture(cfg,fixture,False);report=assert_assisted_configuration(cfg);assert report['passed']
    actual=json.loads(json.dumps(cfg.model_dump(mode='python',serialize_as_any=True)))
    previous=json.loads((work/'probe_candidate_configuration.json').read_text());projected=copy.deepcopy(actual)
    compiled_count=next(o['solver_position_iteration_count'] for o in previous['objects'] if o['name']=='fabrica_beam_1')
    assert compiled_count in (16,64)
    changed=[]
    for obj in projected['objects']:
        if obj['name']=='fabrica_beam_1':
            assert obj['solver_position_iteration_count']==64
            obj['solver_position_iteration_count']=compiled_count;changed.append(obj['name'])
    assert projected==previous and changed==['fabrica_beam_1']
    save(directory/'probe_candidate_configuration.json',actual)
    save(directory/'probe_preparation.json',dict(passed=True,fixture_sha256=sha(directory/'isolated_fixture.json'),
        configurations={'candidate':report},tool_sha256=sha(__file__),simulator_created=False,
        base_fixture_sha256=sha(work/'isolated_fixture.json'),only_fourth_part_position_iterations_16_to_64=True,
        full_config_exact_after_single_property_projection=True,controls_and_guards_unchanged=True,support_warmup_original_controller=bool(support_warmup),
        bounded_same_primary_scene_transport=bool(bounded_scene),whole_free_clearance_screen=bool(full_clearance),
        http_probe_tool_sha256=sha(Path(__file__).parent/'bounded_scene_http_probe.py'),
        baseline_articulation_position_iterations=32,planned_part_position_iterations=count,compiled_position_iterations=compiled_count,additional_physics_override=compiled_count!=count,
        hypothesis='Independent native wrist/joint-anchor error matches model failure. Raising only part1 position-iteration lower bound above the 32-iteration articulation should improve constraint convergence.',
        falsification='Native request not applied, joint residual does not materially reduce, or lift still fails: reject this candidate and do not launch a full episode.',
        background_native_anchor_audit_sha256=sha(work/'A29_joint_anchor_audit.json')))
    print(json.dumps(dict(passed=True,position_iterations=count,configuration_signature=report['configuration_signature'])))

def prepare_release_variant(work, directory):
    from beam_workflow import save,sha
    assert not directory.exists();directory.mkdir()
    fixture=json.loads((work/'isolated_fixture.json').read_text());base=copy.deepcopy(fixture)
    assert fixture.get('object')=='fabrica_beam_0'
    fixture.update(stop_after_phase='assemble_03_part_0_park',stop_label='release_and_park',maximum_diagnostic_steps=30000)
    allowed={'stop_after_phase','stop_label','maximum_diagnostic_steps'}
    assert all(fixture[k]==v for k,v in base.items() if k not in allowed)
    config=json.loads((work/'probe_candidate_configuration.json').read_text())
    names=[p['name'] for p in config['phase_specs']]
    assert names.index(fixture['stop_after_phase'])>names.index(fixture['stop_phase'])
    prep=json.loads((work/'probe_preparation.json').read_text());assert prep['passed']
    save(directory/'isolated_fixture.json',fixture)
    save(directory/'probe_candidate_configuration.json',config)
    save(directory/'probe_preparation.json',dict(passed=True,fixture_sha256=sha(directory/'isolated_fixture.json'),
        configurations=prep['configurations'],qualification_kind='staged_clearance_object',whole_free_clearance_screen=True,
        same_initialization_and_full_configuration=True,base_fixture_sha256=sha(work/'isolated_fixture.json'),
        tool_sha256=sha(__file__),http_probe_tool_sha256=sha(Path(__file__).parent/'bounded_scene_http_probe.py'),
        scope='Same independently initialized four-installed fixture/source/configuration. Only diagnostic stop extends through fifth insertion/release/park, before right withdrawal/final240ticks; not a restored continuation or full success.'))
    print(json.dumps(dict(passed=True,stop_after_phase=fixture['stop_after_phase'],configuration_unchanged=True)))

if __name__=='__main__':
    if os.environ.get('BEAM_PROBE_EXECUTE')=='1':execute()
    else:
        parser=argparse.ArgumentParser();parser.add_argument('work',type=Path);parser.add_argument('--solver-profile',type=Path);parser.add_argument('--warmup-support',action='store_true');parser.add_argument('--bounded-scene',action='store_true');parser.add_argument('--full-clearance',action='store_true');parser.add_argument('--branch-screen',action='store_true');parser.add_argument('--release-variant',type=Path);parser.add_argument('--probe-object',choices=['fabrica_beam_1','fabrica_beam_0'],default='fabrica_beam_1');args=parser.parse_args()
        sys.path[:0]=[str(args.work.resolve()/'candidate'),str(args.work.resolve().parent/'tools')]
        if args.release_variant:prepare_release_variant(args.work.resolve(),args.release_variant.resolve())
        elif args.solver_profile:prepare_solver_profile(args.work.resolve(),args.solver_profile.resolve(),64,args.warmup_support,args.bounded_scene,args.full_clearance)
        else:prepare(args.work.resolve(),args.branch_screen,args.probe_object)
