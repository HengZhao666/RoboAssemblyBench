"""C02 finite coherent-cache probes; no simulator and no critical-tool edits.

The preserved C01 cases supply measured/native error fixtures. This supplement
changes only their previous object command / previous published q pair. Cache
seeds are bounded stress fixtures, not claimed native histories.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pose(t):
    q=Rotation.from_matrix(t[:3,:3]).as_quat()
    return {'position':t[:3,3].copy(),'orientation':q[[3,0,1,2]]}
def serial(v):
    if isinstance(v,np.ndarray):return v.tolist()
    if isinstance(v,np.generic):return v.item()
    raise TypeError(type(v).__name__)
def angle(a,b):
    return float(Rotation.from_quat(np.asarray(a)[[1,2,3,0]]).inv().__mul__(Rotation.from_quat(np.asarray(b)[[1,2,3,0]])).magnitude())

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in ('repository','tools','geometry','config','path','support','reuse-certificate','base-run'):
        ap.add_argument('--'+n,type=Path,required=True)
    ap.add_argument('--scope-only',action='store_true')
    ap.add_argument('--output',type=Path)
    a=ap.parse_args();p=a.path.parent
    sys.path[:0]=[str(a.repository),str(a.tools),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    from sweep_model import SceneModel,pose_matrix
    from visual_mesh_check import VisualMeshChecker
    from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import absolute_object_servo,anchored_object_prediction,descent_admission,published_object_command_safe,object_pose_from_tcp
    import fcl
    cert=json.loads(a.reuse_certificate.read_text())
    if cert.get('passed') is not True:raise ValueError('exact C02 reuse certificate has not passed')
    config=json.loads(a.config.read_text());config=config.get('task_configs',[config])[0]
    phases={v['name']:v['local_skill'] for v in config['phase_specs'] if 'local_skill' in v}
    targets=config['target_poses'];model=SceneModel(a.geometry);checker=VisualMeshChecker(model,include_self=True)
    rows=[json.loads(l) for l in a.path.open()];support=json.loads(a.support.read_text())
    body={m['object']:m['body'] for m in model.data['visible_meshes'] if str(m['object']).startswith('fabrica_beam_')}
    old=json.loads((p/'C01_full_published_corridor.json').read_text())
    hover=json.loads((p/'C01_full_published_corridor_final.json').read_text())
    by_index={}
    for s in old['final_published_full_mesh']['scenarios']+hover['final_published_full_mesh']['scenarios']:
        by_index.setdefault(s['path_index'],[]).append(s)
    selected=[];selection=[]
    for part in (3,2,1,0):
        for suffix in ('insert_01','insert_05','insert_07','insert_08','insert_09'):
            ix=[i for i,r in enumerate(rows) if f'_part_{part}_' in r['phase'] and r['phase'].endswith(suffix)]
            i=ix[-1] if suffix=='insert_09' else ix[0]
            selected.append(i);selection.append({'path_index':i,'kind':'insertion','phase':rows[i]['phase']})
    # All left-arm/held-part to right-arm triangle distances, not just TCPs.
    proximity=[]
    def right_distance(i):
        r=rows[i];poses=model.poses(r['q']);poses.update({body[k]:np.asarray(v) for k,v in r['part_poses'].items()})
        name=r['intended_grasp'][1]
        for m,obj in zip(checker.meshes,checker.shapes):
            t=poses[m['body']];obj.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
        left=[j for j,m in enumerate(checker.meshes) if m['robot']=='franka_left' or m['object']==name]
        right=[j for j,m in enumerate(checker.meshes) if m['robot']=='franka_right']
        best=None
        for j in left:
            for k in right:
                d=float(fcl.distance(checker.shapes[j],checker.shapes[k],fcl.DistanceRequest(),fcl.DistanceResult()))
                if best is None or d<best['distance_m']:best={'distance_m':d,'pair':[checker.meshes[j]['prim'],checker.meshes[k]['prim']]}
        return best
    for part in (3,2,1,0):
        ix=sorted(i for i in by_index if f'_part_{part}_' in rows[i]['phase'] and rows[i]['phase'].endswith('_transport_hover'))
        distances=[(i,right_distance(i)) for i in ix]
        near=min(distances,key=lambda x:x[1]['distance_m'])
        picks=sorted(set([ix[0],ix[-1],near[0]]))
        proximity.append({'part':part,'nominal_anchors_examined':len(ix),'nearest_path_index':near[0],**near[1]})
        for i in picks:
            selected.append(i);selection.append({'path_index':i,'kind':'free_hover','phase':rows[i]['phase']})
    fixtures=[]
    for i in selected:
        r=rows[i];ss=by_index[i]
        if len(ss)!=21 or {s['case'] for s in ss}!=set(range(21)):raise ValueError('selected C01 anchor lacks its complete exact21 cases')
        robot,name=r['intended_grasp'];q={k:np.asarray(v,float) for k,v in r['q'].items()};q['franka_right']=np.asarray(support['support_q']['franka_right'])
        tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        relation=np.linalg.inv(model.poses(q)[tcp])@np.asarray(r['part_poses'][name])
        for s in sorted(ss,key=lambda x:x['case']):
            qs={k:v.copy() for k,v in q.items()};qs[robot][:6]=s['actual_q']
            fixtures.append(dict(kind='free_hover' if r['phase'].endswith('_transport_hover') else 'insertion',
                phase=r['phase'],path_index=i,case=s['case'],robot=robot,name=name,q=qs,
                part_poses=r['part_poses'],relation=relation,current=s['actual_native_object_pose'],
                ready=s['admitted'],mode='combined',source_case=s))
    # Three native base-turn states with exact current native body and native q.
    base_config=json.loads((a.base_run/'task_config_snapshot.json').read_text());base_config=base_config.get('task_configs',[base_config])[0]
    base_phase='base_6_assembly_clearance';bs=next(v['local_skill'] for v in base_config['phase_specs'] if v['name']==base_phase)
    base_goal=base_config['target_poses'][bs['target_object_target']]
    motions={}
    for line in (a.base_run/'motion_trace.jsonl').open():
        r=json.loads(line)
        if r.get('phase')==base_phase and r.get('robot')=='franka_right':motions[r['step']]=r
    entities={}
    for line in (a.base_run/'entity_geometry_trace.jsonl').open():
        r=json.loads(line)
        if r.get('phase')==base_phase and r['step'] in motions:entities[r['step']]=r
    native=[]
    for line in (a.base_run/'physics_boundary_trace.jsonl').open():
        r=json.loads(line);step=r.get('cycle_step')
        if r.get('boundary')!='before_action' or step not in entities:continue
        b=next(v for v in r['bodies'] if v['name']=='fabrica_beam_6')
        if not r.get('valid') or not b.get('valid'):raise ValueError('base native input is invalid')
        current={'position':b['positions'][0],'orientation':b['orientations'][0]}
        native.append((step,r,current,angle(current['orientation'],base_goal['orientation'])))
    if len(native)<3:raise ValueError('base turn lacks three real native observations')
    half=(native[0][3]+native[-1][3])/2
    base_picks=[native[0],min(native,key=lambda r:abs(r[3]-half)),native[-1]]
    if len({v[0] for v in base_picks})!=3:raise ValueError('base start/middle/end samples are not distinct')
    for step,n,current,err in base_picks:
        motion=motions[step];target_tcp=pose_matrix(motion['target_pose']['position'],motion['target_pose']['orientation'])
        target_part=pose_matrix(base_goal['position'],base_goal['orientation'])
        if bs.get('target_object_offset'):target_part[:3,3]+=bs['target_object_offset']
        relation=np.linalg.inv(target_tcp)@target_part
        q={k:np.asarray(v['joint_positions'][0],float) for k,v in n['robots'].items()}
        parts={k:entities[step]['body_poses'][body[k]] for k in body}
        for mode in ('empty','position','orientation','combined'):
            fixtures.append(dict(kind='native_base_turn',phase=base_phase,path_index=None,native_step=step,
                native_angle_to_target_rad=err,case=mode,robot='franka_right',name='fabrica_beam_6',q=q,
                part_poses=parts,relation=relation,current=current,ready=True,mode=mode))
    source_paths=[a.repository/'toolkits/factory_dual_franka_assembly'/v for v in ('plumbers_block_ur5e_skills.py','beam_assisted_servo.py','beam_assisted_state.py')]+[a.tools/'sweep_model.py',a.tools/'visual_mesh_check.py']
    binding={str(v):sha(v) for v in source_paths}
    inputs={str(v):sha(v) for v in [a.geometry,a.config,a.path,a.support,a.reuse_certificate,p/'C01_full_published_corridor.json',p/'C01_full_published_corridor_final.json']}
    inputs.update({str(a.base_run/v):sha(a.base_run/v) for v in ('motion_trace.jsonl','entity_geometry_trace.jsonl','physics_boundary_trace.jsonl','task_config_snapshot.json')})
    def goal_for(f):
        spec=phases[f['phase']];g={k:np.asarray(v,float) for k,v in targets[spec['target_object_target']].items()}
        if spec.get('target_object_offset'):
            if spec.get('target_object_offset_frame')!='world':raise ValueError('unmodeled target offset frame')
            g['position']=g['position']+spec['target_object_offset']
        return spec,g
    seed_probes=[]
    for f in fixtures:
        spec,g=goal_for(f);current=f['current'];rp=pose(f['relation']);q=f['q'];robot=f['robot']
        tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link';fk=pose(model.poses(q)[tcp]@f['relation'])
        sg={k:np.asarray(v,float).copy() for k,v in g.items()}
        if f['mode'] in ('empty','orientation'):sg['position']=np.asarray(current['position']).copy()
        if f['mode'] in ('empty','position'):sg['orientation']=np.asarray(current['orientation']).copy()
        # Stay 0.1um / 0.1urad inside the hard cache radius so tiny IK
        # roundoff cannot exercise the stale-cache reset instead of the lead.
        st,sp=absolute_object_servo(current,sg,rp['position'],rp['orientation'],.003-1e-7,.02-1e-7,
            convergence_axis=[0,0,-1] if spec.get('beam_assisted_descent_admission') else None,
            allow_axial_motion=f['ready'],measured_fk_object_pose=fk)
        dp=float(np.linalg.norm(np.asarray(sp['position'])-current['position']));dr=angle(current['orientation'],sp['orientation'])
        if dp>.003000001 or dr>.020000001:raise ValueError('synthetic seed exceeds production lookahead')
        f.update(seed_tcp=st,seed_pose=sp,model_current=fk)
        seed_probes.append({'phase':f['phase'],'path_index':f['path_index'],'native_step':f.get('native_step'),'case':f['case'],'seed_position_lead_m':dp,'seed_orientation_lead_rad':dr})
    scope=dict(insertion_anchors=20,free_hover_anchors=len(selected)-20,native_base_anchors=3,
        positive_cases=len(fixtures),maximum_authorized_positive_cases=684,dangerous_counterexamples=8,
        expected_case_count=len(fixtures),actual_case_count=0,
        expected_counterexamples=8,actual_counterexamples=0,
        selected_anchors=selection,free_hover_right_nearest=proximity,
        native_base_steps=[v[0] for v in base_picks],
        cache_seed_scope='bounded coherent object cache / published-q stress fixtures; not native histories',
        seed_numerical_inset_position_m=1e-7,seed_numerical_inset_orientation_rad=1e-7,
        seed_lookahead_precheck_passed=True,mesh_count=54,self_and_complete_grippers_required=True,
        full_visible_meshes=54,self_checked=True,full_q_states=0)
    if len(fixtures)>684:raise ValueError('finite approved case budget exceeded')
    if a.scope_only:
        out=dict(schema='beam_C02_cached_lead_scope_v1',passed=True,errors=[],tool_sha256=sha(__file__),config_sha256=sha(a.config),source_sha256=binding,input_sha256=inputs,scope=scope,seed_probes=seed_probes,
            pending=['robot IK and previous-q tracking feasibility','production final-q guard','all54/self swept mesh'])
    else:
        errors=[];results=[];counterexamples=[]
        for number,f in enumerate(fixtures):
            spec,g=goal_for(f);robot,name=f['robot'],f['name'];q={k:v.copy() for k,v in f['q'].items()};qa=q[robot][:6].copy()
            tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link';relation=f['relation'];rp=pose(relation);current=f['current'];fk=f['model_current']
            def ik(t,ref):
                qs={k:v.copy() for k,v in q.items()}
                def residual(x):
                    qs[robot][:6]=x;v=model.poses(qs)[tcp]
                    return np.r_[(v[:3,3]-t[:3,3])*5,Rotation.from_matrix(t[:3,:3].T@v[:3,:3]).as_rotvec()]
                sol=least_squares(residual,ref,max_nfev=70,xtol=1e-10,ftol=1e-10,gtol=1e-10);e=residual(sol.x)
                if np.linalg.norm(e[:3])/5>.00001 or np.linalg.norm(e[3:])>.0001:raise ValueError('cache/command IK exceeds10um/.1mrad')
                return ref+(sol.x-ref+np.pi)%(2*np.pi)-np.pi
            record={k:f.get(k) for k in ('kind','phase','path_index','native_step','case','mode')}
            try:
                qprev=ik(pose_matrix(f['seed_tcp']['position'],f['seed_tcp']['orientation']),qa)
                limited=Adapter._limit_command_to_measured_state(current_q=qa,command_q=qprev,spec=spec)
                if not np.allclose(qprev,limited,atol=1e-10,rtol=0):raise ValueError('coherent cache seed is outside actual production measured tracking budget')
                qpstate={k:v.copy() for k,v in q.items()};qpstate[robot][:6]=qprev
                cache=anchored_object_prediction(current,fk,pose(model.poses(qpstate)[tcp]@relation))
                command,_=absolute_object_servo(current,g,rp['position'],rp['orientation'],float(spec.get('cartesian_position_step',.001)),float(spec.get('cartesian_orientation_step',.004)),
                    previous_object_command=cache,position_lookahead=float(spec.get('target_object_servo_position_command_lookahead',.003)),orientation_lookahead=float(spec.get('cartesian_orientation_command_lookahead',.02)),
                    convergence_axis=[0,0,-1] if spec.get('beam_assisted_descent_admission') else None,allow_axial_motion=f['ready'],measured_fk_object_pose=fk)
                plain,_=absolute_object_servo(current,g,rp['position'],rp['orientation'],float(spec.get('cartesian_position_step',.001)),float(spec.get('cartesian_orientation_step',.004)),
                    convergence_axis=[0,0,-1] if spec.get('beam_assisted_descent_admission') else None,allow_axial_motion=f['ready'],measured_fk_object_pose=fk)
                cache_effect_position=float(np.linalg.norm(np.asarray(command['position'])-plain['position']))
                cache_effect_angle=angle(command['orientation'],plain['orientation'])
                qr=ik(pose_matrix(command['position'],command['orientation']),qa)
                publication_state={'descent_ready':f['ready'],'stamp':{'valid':True,'epoch':0,'index':12,'time':.05,'dt':1/240}}
                class Offline(Adapter):
                    def _assisted_object_servo_context(self,**kw):return name,publication_state['stamp'],{},current,(rp['position'],rp['orientation']),publication_state
                    def _target_object_pose(self,**kw):return g
                    def _grasp_command_fk_pose(self,**kw):
                        qq={k:v.copy() for k,v in q.items()};qq[robot][:6]=kw['command_q'];return pose(model.poses(qq)[tcp])
                adapter=Offline.__new__(Offline);adapter._last_arm_command_q={robot:qprev.copy()}
                task=SimpleNamespace(phase_index=0,phase_entry_step=0,_ur5e_plumbers_last_arm_command_q={robot:qprev.copy()})
                limits=Adapter._command_joint_step_limits(spec=spec,joint_count=6)
                qr=Adapter._limited_joint_target(current_q=qa,target_q=qr,max_joint_step=.02 if limits is None else limits)
                qr=adapter._continuous_command_q(task=task,robot_name=robot,command_q=qr,spec=spec)
                qr=Adapter._limit_command_to_measured_state(current_q=qa,command_q=qr,spec=spec)
                qp,detail=adapter._assisted_verify_published_held_command(task=task,robot_name=robot,
                    context={'spec':spec,'phase_key':(0,),'phase_index':0,'phase_entry_step':0,'tracked_robots':{},'tracked_objects':{}},command_q=qr,current_q=qa,previous_q=qprev)
                count=0
                # Check both coherent previous publication and new final q.
                for endpoint in (qprev,qp):
                    ns=max(2,int(np.ceil(np.max(abs(endpoint-qa))/.004))+1)
                    for t in np.linspace(0,1,ns):
                        qq={k:v.copy() for k,v in q.items()};qq[robot][:6]=qa+t*(endpoint-qa)
                        poses=model.poses(qq);poses.update({body[k]:np.asarray(v) for k,v in f['part_poses'].items()})
                        native_obj=anchored_object_prediction(current,fk,pose(poses[tcp]@relation))
                        poses[body[name]]=pose_matrix(native_obj['position'],native_obj['orientation'])
                        checker.check(poses,f['phase'],number,(robot,name));count+=1
                record.update(passed=True,actual_q=qa,previous_published_q=qprev,cached_native_object_pose=cache,final_q=qp,publication=detail,
                    seed_position_lead_m=float(np.linalg.norm(np.asarray(cache['position'])-current['position'])),seed_orientation_lead_rad=angle(current['orientation'],cache['orientation']),mesh_sweep_samples=count)
                record.update(cache_effect_on_tcp_position_m=cache_effect_position,cache_effect_on_tcp_orientation_rad=cache_effect_angle)
            except Exception as e:
                record.update(passed=False,error=str(e));errors.append(record)
            results.append(record)
            if number%50==0:print('C02 cache cases '+str(number+1)+'/'+str(len(fixtures)),file=sys.stderr,flush=True)
        # Dangerous cached descent is removed in object space before IK. These
        # are rejection controls, never required to fit the admitted CAD tube.
        for part in (3,2,1,0):
            f=next(v for v in fixtures if v['kind']=='insertion' and f'_part_{part}_' in v['phase'])
            spec,g=goal_for(f);rp=pose(f['relation']);contract=spec['beam_assisted_descent_admission']
            for kind in ('lateral_1p2mm','orientation_20mrad'):
                bad={k:np.asarray(v,float).copy() for k,v in f['current'].items()}
                if kind.startswith('lateral'):bad['position'][0]+=.0012
                else:bad['orientation']=(Rotation.from_rotvec([.02,0,0])*Rotation.from_quat(bad['orientation'][[1,2,3,0]])).as_quat()[[3,0,1,2]]
                cache={k:v.copy() for k,v in bad.items()};cache['position'][2]-=.003
                if kind.startswith('orientation'):
                    cache['orientation']=(Rotation.from_rotvec([.02,0,0])*Rotation.from_quat(bad['orientation'][[1,2,3,0]])).as_quat()[[3,0,1,2]]
                st={};motion=dict(motion_valid=True,physics_handle_valid=True,body_path=body[f['name']],motion_view_id='C02-negative',linear_speed=0.,angular_speed=0.)
                ready=False
                for tick in range(1,13):ready,detail=descent_admission(st,dict(valid=True,epoch=0,index=tick,time=tick/240,dt=1/240),bad,g,[0,0,-1],contract,motion)
                _,cmd=absolute_object_servo(bad,g,rp['position'],rp['orientation'],.001,.004,previous_object_command=cache,convergence_axis=[0,0,-1],allow_axial_motion=ready)
                safe,guard=published_object_command_safe(bad,cmd,g,[0,0,-1],contract,ready)
                passed=not ready and float(cmd['position'][2]-bad['position'][2])>=-1e-12 and (not safe if kind.startswith('orientation') else safe)
                counterexamples.append(dict(part=part,kind=kind,passed=passed,ready=ready,object_z_delta_m=float(cmd['position'][2]-bad['position'][2]),admission=detail,
                    direct_command_guard_accepts=safe,guard_detail=guard,rejection_expected=kind.startswith('orientation')))
                if not passed:errors.append({'counterexample':counterexamples[-1]})
        report=checker.report();badmesh=[v for v in report['intersections'] if v['depth']>.000001]
        if badmesh:errors.append({'non_target_intersections_above_1um':len(badmesh)})
        if len(results)!=len(fixtures) or len(counterexamples)!=8:errors.append('incomplete approved case matrix')
        if max((v.get('cache_effect_on_tcp_position_m',0.) for v in results),default=0.)<.00001 or max((v.get('cache_effect_on_tcp_orientation_rad',0.) for v in results),default=0.)<.001:
            errors.append('cached lead did not materially change both position and orientation commands')
        scope.update(actual_case_count=len(results),actual_counterexamples=len(counterexamples),full_q_states=report['samples'])
        out=dict(schema='beam_C02_cached_lead_full54_v1',passed=not errors,errors=errors,tool_sha256=sha(__file__),config_sha256=sha(a.config),source_sha256=binding,input_sha256=inputs,scope=scope,
            cases=results,counterexamples=counterexamples,mesh_report=report,
            gaps=['Bounded coherent cache stress fixtures are not native tracking histories.','Only the declared finite critical nodes and error directions are checked.','No between-sample CCD, unexported warehouse, or PhysX cooked/SDF certificate.'],formal_physical_success=False)
    text=json.dumps(out,indent=2,default=serial)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')
    return 0 if out['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
