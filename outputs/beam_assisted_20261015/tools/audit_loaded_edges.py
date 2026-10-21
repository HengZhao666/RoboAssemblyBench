"""Finite nonzero-preload replay on all12 actual free-move phase edges."""
from loaded_handoff_preflight import *
import hashlib
EXPECTED_LOOP_SOURCE_SHA='83515ce39189a56a176c047afb5fb1906938ec3a2440fc7f9c48f6918ddc7864'

def main():
    source=Path(__file__).with_name('loaded_handoff_preflight.py').read_text()
    parsed=ast.parse(source);func=next(n for n in parsed.body if isinstance(n,ast.FunctionDef) and n.name=='replay')
    original_loop=next(n for n in func.body if isinstance(n,ast.For) and ast.unparse(n.target)=='(i, f)')
    require(hashlib.sha256(ast.get_source_segment(source,original_loop).encode()).hexdigest()==EXPECTED_LOOP_SOURCE_SHA,'Shared production replay loop changed; review the extraction before reuse')
    cfg=json.loads(CONFIG.read_text());specs={v['name']:v for v in cfg['phase_specs']}
    model=SceneModel(P/'runs/A16_beam_full/scene_geometry.json');checker=VisualMeshChecker(model,include_self=True)
    rows=[json.loads(line) for line in (P/'C01_bimanual_repaired_v2_path.jsonl').open()]
    body={m['object']:m['body'] for m in model.data['visible_meshes'] if str(m['object']).startswith('fabrica_beam_')}
    fixtures=[];seeds=[]
    for phase,entry in specs.items():
        spec=entry.get('local_skill',{})
        if spec.get('beam_assisted_phase_handoff_ticks') is None:continue
        require(spec.get('beam_assisted_descent_admission') is None,'Loaded free-move seed cannot stand in for a cold insertion')
        row=next(r for r in rows if r['phase']==phase);robot,name=row['intended_grasp']
        q={k:np.asarray(v,float) for k,v in row['q'].items()};tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        native_tcp=model.poses(q)[tcp];relation=np.linalg.inv(native_tcp)@np.asarray(row['part_poses'][name])
        target=native_tcp.copy();target[2,3]+=.002;qa=q[robot][:6].copy()
        def residual(x):
            qq={k:v.copy() for k,v in q.items()};qq[robot][:6]=x;t=model.poses(qq)[tcp]
            return np.r_[(t[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@t[:3,:3]).as_rotvec()]
        sol=least_squares(residual,qa,max_nfev=70,ftol=1e-10,gtol=1e-10,xtol=1e-10);r=residual(sol.x)
        require(np.linalg.norm(r[:3])/5<1e-5 and np.linalg.norm(r[3:])<1e-4,'Coherent2mm preload IK not solved')
        lead=float(np.max(abs(sol.x-qa)));require(1e-4<lead<=.12,'Nonzero final-publication tracking preload absent/outside budget')
        fixtures.append(dict(phase=phase,q=q,parts={body[k]:np.asarray(v,float) for k,v in row['part_poses'].items()},relation=relation,current=pose(np.asarray(row['part_poses'][name])),previous=sol.x.copy(),kind='declared_coherent_2mm_incoming_publication'))
        seeds.append(dict(phase=phase,TCP_position_residual_m=float(np.linalg.norm(r[:3])/5),TCP_orientation_residual_rad=float(np.linalg.norm(r[3:])),previous_minus_actual_max_rad=lead,requested_vertical_lead_m=.002))
    require(len(fixtures)==12,'All12 actual free-move edge entries must be covered')
    results=[];support=[]

    for i,f in enumerate(fixtures):
        robot='franka_left';name=specs[f['phase']]['local_skill']['object'];spec=specs[f['phase']]['local_skill'];qq={k:v.copy() for k,v in f['q'].items()};tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        qa=qq[robot][:6].copy();qprev=f['previous'].copy();rp=pose(f['relation']);current=f['current'];state={};stamp={'valid':True,'epoch':0,'index':100,'time':100/240,'dt':1/240}
        g={k:np.asarray(v,float) for k,v in cfg['target_poses'][spec['target_object_target']].items()}
        if spec.get('target_object_offset'):g['position']+=spec['target_object_offset']
        class Offline(Adapter):
            def _assisted_object_servo_context(self,**kw):return name,stamp,{},current,(rp['position'],rp['orientation']),state
            def _target_object_pose(self,**kw):return g
            def _current_arm_q(self,*args):return qa
            def _grasp_command_fk_pose(self,**kw):
                x={k:v.copy() for k,v in qq.items()};x[robot][:6]=kw['command_q'];return pose(model.poses(x)[tcp])
        adapter=Offline.__new__(Offline);adapter._insertion_lateral_alignment_active={}
        if f.get('synthesize_downward_lead'):
            original=pose(model.poses(qq)[tcp]);target=pose_matrix(original['position'],original['orientation']);target[2,3]-=.003
            def seed_residual(x):
                ss={k:v.copy() for k,v in qq.items()};ss[robot][:6]=x;tt=model.poses(ss)[tcp]
                return np.r_[(tt[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@tt[:3,:3]).as_rotvec()]
            sol=least_squares(seed_residual,qa,max_nfev=70,ftol=1e-10,gtol=1e-10,xtol=1e-10);qprev=sol.x
            require(np.max(abs(qprev-qa))<=.12,'synthetic lead exceeds tracking budget')
        task=SimpleNamespace(phase_index=0,phase_entry_step=0,step_counter=0,_assisted_published_q={robot:qprev.copy()},_assisted_published_q_stamps={robot:{'epoch':0,'index':99}},_assisted_event=lambda *args,**kw:None)
        first=None;counts=0;caps=[]
        for age in range(2):
            stamp.update(index=100+age,time=(100+age)/240)
            command=adapter._assisted_absolute_object_servo_pose(phase_key=(0,),task=task,robot_name=robot,spec=spec,tracked_robots={},tracked_objects={},object_pose=current,target_object_pose=g)
            target=pose_matrix(command['position'],command['orientation'])
            def residual(x):
                ss={k:v.copy() for k,v in qq.items()};ss[robot][:6]=x;tt=model.poses(ss)[tcp]
                return np.r_[(tt[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@tt[:3,:3]).as_rotvec()]
            sol=least_squares(residual,qprev,max_nfev=60,ftol=1e-9,gtol=1e-9,xtol=1e-9)
            request=Adapter._limited_joint_target(current_q=qprev,target_q=sol.x,max_joint_step=spec['max_command_joint_step'])
            request=Adapter._limit_command_to_measured_state(current_q=qa,command_q=request,spec=spec)
            final,detail=adapter._assisted_verify_published_held_command(task=task,robot_name=robot,context=dict(spec=spec,phase_key=(0,),phase_index=0,phase_entry_step=0,tracked_robots={},tracked_objects={}),command_q=request,current_q=qa,previous_q=qprev)
            cap=state.get('phase_handoff_joint_step',.02);require(np.max(abs(final-qprev))<=cap+1e-10,'final cap exceeded');caps.append(cap)
            if first is None:
                first=float(np.max(abs(final-qprev)))
                if f.get('synthesize_downward_lead'):
                    predicted=anchored_object_prediction(current,object_pose_from_tcp(pose(model.poses(qq)[tcp]),rp['position'],rp['orientation']),object_pose_from_tcp(adapter._grasp_command_fk_pose(command_q=final),rp['position'],rp['orientation']))
                    require(not state['descent_ready'] and predicted['position'][2]>=current['position'][2]-2.0001e-5,'cold gate allowed inherited descent')
                else:require(first<1e-12,'admissible entry publication preload changed')
            actual_model=object_pose_from_tcp(pose(model.poses(qq)[tcp]),rp['position'],rp['orientation'])
            for fraction in np.linspace(0.,1.,max(3,int(np.ceil(np.max(abs(final-qa))/.004))+1)):
                qs={k:v.copy() for k,v in qq.items()};qs[robot][:6]=qa+fraction*(final-qa);poses=model.poses(qs)
                # Anchor every dynamic frame to the actual/nominal fixture; robot frames follow FK deltas.
                origin=model.poses(qq)
                for b,mat in f['parts'].items():
                    poses[b]=mat@np.linalg.inv(origin[b])@poses[b] if b in origin and '/robots/' in b else mat
                pred=anchored_object_prediction(current,actual_model,object_pose_from_tcp(pose(model.poses(qs)[tcp]),rp['position'],rp['orientation']))
                poses[body[name]]=pose_matrix(pred['position'],pred['orientation'])
                require(not missing_dynamic_poses(model,poses),'missing dynamic entity')
                checker.check(poses,f['phase'],i*100+age,(robot,name));counts+=1
                support.append(check_visual_support(checker,specs[f['phase']]['beam_assisted_visual_support'],f['phase'],i*100+age)['ready'])
            qprev=final
        results.append(dict(kind=f['kind'],phase=f['phase'],ticks=2,mesh_states=counts,entry_publication_delta_rad=first,preload_rad=float(np.max(abs(f['previous']-qa))),final_joint_cap_rad=caps[-1]))

    mesh=checker.report()
    require(mesh['meshes']==54 and mesh['self_checked'] and all(support),'Complete meshes/support required')
    require(not any(v['depth']>1e-6 for v in mesh['intersections']),'Non-target triangle overlap')
    require(mesh['minimum_intended_grasp_signed_separation']>=-1e-6,'Intended finger penetration')
    report=dict(passed=True,edge_count=12,native_ticks_per_edge=2,seed_checks=seeds,results=results,full_mesh=mesh,support_checks=len(support),support_all_ready=all(support),tool_sha256=sha(__file__),shared_loop_source_sha256=EXPECTED_LOOP_SOURCE_SHA,shared_tool_sha256=sha(Path(__file__).with_name('loaded_handoff_preflight.py')),source_sha256={k:sha(A/k) for k in (SK,ST,CO,TE,DI,TD)},configuration_sha256=sha(CONFIG),native_physics_executed=0,limits=['Every incoming2mm lead is a declared synthetic FK/IK-coherent stress state, not a native history from an unreached part.','First two native-clock control steps only; all25 ramp steps are independently tested at the original29 fixtures.','Full54+self swept samples subdivide actual-to-final q increments at most4mrad per joint; no between-sample CCD proof.'])
    save(P/(CYCLE+'_loaded_edge_entries.json'),report)
    print(json.dumps(dict(passed=True,edges=12,mesh_states=mesh['samples'],minimum_grasp_gap_m=mesh['minimum_intended_grasp_signed_separation'])))
if __name__=='__main__':main()
