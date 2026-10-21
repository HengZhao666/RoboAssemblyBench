"""Fresh finite nonlinear-FK biased tracking plant, final production q guard and54CAD.

No native physics. Actual q evolves every distinct tick. The IK proxy solves the
exported full joint graph; the production two final joint/object guards run.
"""
import copy,json,sys,tarfile
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parents[1];A=P/'assisted_repository'
sys.path[:0]=[str(A),str(P/'tools'),str(P.parent/'runtime_tools/geometry_deps')]
from beam_workflow import save,sha
from sweep_model import SceneModel,pose_matrix
from visual_mesh_check import VisualMeshChecker
from visual_support_check import check_visual_support,missing_dynamic_poses
from index_run import select
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
from toolkits.factory_dual_franka_assembly.beam_assisted_state import BeamAssistedState
from toolkits.factory_dual_franka_assembly.beam_assisted_servo import object_pose_from_tcp,anchored_object_prediction

def pose(t):return dict(position=t[:3,3].copy(),orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]])
def mat(p):return pose_matrix(p['position'],p['orientation'])
def require(v,m):
 if not v:raise ValueError(m)

def main():
 cycle=sys.argv[1];cfg=json.loads((P/(cycle+'_actual_configuration_layout505_config.json')).read_text());specs={s['name']:s for s in cfg['phase_specs']}
 model=SceneModel(P/'runs/A18_beam_full/scene_geometry.json');checker=VisualMeshChecker(model,include_self=True)
 path=[json.loads(l) for l in (P/'C01_bimanual_repaired_v2_path.jsonl').open()]
 body={m['object']:m['body'] for m in model.data['visible_meshes'] if str(m['object']).startswith('fabrica_beam_')};robot='franka_left';tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
 nn=select(P/'runs/A18_beam_full','physics_boundary_trace.jsonl',14216,14216,'before_action')[0];en=select(P/'runs/A18_beam_full','entity_geometry_trace.jsonl',14216,14216)[0]
 tail=json.loads((P/'runs/A18_beam_full/post_failure_physics_trace.jsonl').read_text().splitlines()[-1]);tr=tail['robots'][robot]
 tailq=np.asarray(tr['joint_positions'][0])[:6];tailtarget=np.asarray(next(a['joint_positions'] for a in tr['sdk_submission']['actions'] if a.get('joint_indices')==list(range(6))))
 bias=tailtarget-tailq
 require(0<np.max(abs(bias))<.002,'observed settled tracking bias absent/outside scope')
 fixtures=[]
 actualq={k:np.asarray(v['joint_positions'][0],float) for k,v in nn['robots'].items()};b=next(b for b in nn['bodies'] if b['name']=='fabrica_beam_3')
 fixtures.append(dict(kind='A18_actual14216',phase=next(n for n in specs if '_part_3_' in n and n.endswith('_insert_00')),q=actualq,
  parts={k:np.asarray(v,float) for k,v in en['body_poses'].items()},relation=np.asarray(json.loads((P/'D02_handoff_analysis.json').read_text())['captured_controller_relation']),current=dict(position=np.asarray(b['positions'][0]),orientation=np.asarray(b['orientations'][0]))))
 # All4 first insertion entries, using the measured joint tracking bias as a declared stress.
 for part in (3,2,1,0):
  phase=next(n for n in specs if f'_part_{part}_' in n and n.endswith('_insert_00'))
  r=next(r for r in path if r['phase']==phase);q={k:np.asarray(v,float) for k,v in r['q'].items()};name=specs[phase]['local_skill']['object']
  fixtures.append(dict(kind='nominal_entry_biased_counterfactual',phase=phase,q=q,parts={body[k]:np.asarray(v,float) for k,v in r['part_poses'].items()},relation=np.linalg.inv(model.poses(q)[tcp])@np.asarray(r['part_poses'][name]),current=pose(np.asarray(r['part_poses'][name]))))
 results=[];support=[];all_states=[]
 for fi,f in enumerate(fixtures):
  for alpha in ((.02,.08,.2) if fi==0 else (.08,)):
   for enabled in (False,True):
    spec=copy.deepcopy(specs[f['phase']]['local_skill']);name=spec['object'];rp=pose(f['relation']);baseq={k:v.copy() for k,v in f['q'].items()};origin=model.poses(baseq)
    goal={k:np.asarray(v,float).copy() for k,v in cfg['target_poses'][spec['target_object_target']].items()}
    if spec.get('target_object_offset'):goal['position']+=spec['target_object_offset']
    if not enabled:spec.pop('beam_assisted_alignment_hold')
    q={k:v.copy() for k,v in baseq.items()};qa=q[robot][:6];qprev=qa+bias;state={};stamp=dict(valid=True,epoch=0,index=100,time=100/240,dt=1/240);current=copy.deepcopy(f['current'])
    baseline_model=object_pose_from_tcp(pose(origin[tcp]),rp['position'],rp['orientation'])
    calibration=mat(current)@np.linalg.inv(mat(baseline_model))
    motion=dict(motion_valid=True,physics_handle_valid=True,body_path=body[name],motion_view_id=19,linear_speed=0.,angular_speed=0.,linear_velocity=np.zeros(3))
    class Offline(Adapter):
     def _assisted_object_servo_context(self,**kw):return name,stamp,motion,current,(rp['position'],rp['orientation']),state
     def _current_arm_q(self,*a):return qa.copy()
     def _target_object_pose(self,**kw):return goal
     def _grasp_command_fk_pose(self,**kw):
      qs={k:v.copy() for k,v in q.items()};qs[robot][:6]=kw['command_q'];return pose(model.poses(qs)[tcp])
    adapter=Offline({});task=BeamAssistedState();task.phase_index=0;task.phase_entry_step=100;task.step_counter=100
    task._beam_physics_stamp=lambda:stamp;task._assisted_published_q={robot:qprev.copy()};task._assisted_published_q_stamps={robot:dict(epoch=0,index=99)}
    # Openness is an explicit fixture constant, not inferred native gripper force;
    # complete CAD fingers retain the recorded actual full articulation q.
    task._attachments={name:dict(robot_name=robot,attach_spec={'beam_assisted':True},assisted_hold_openness=.3)}
    task.robots={robot:NS(controllers={'arm_joint_controller':NS(get_joint_subset=lambda:NS(get_joint_positions=lambda:qa.copy()))})}
    task.get_current_phase_spec=lambda:{'name':f['phase'],'local_skill':spec};events=[];task._assisted_event=lambda event,**kw:events.append((event,kw))
    context=dict(adapter=adapter,spec=spec,phase_key=(0,),phase_index=0,phase_entry_step=100,tracked_robots={},tracked_objects={});task._beam_assisted_command_contexts={robot:context}
    heights=[];lateral=[];warm=False;violation=None;minimum=float(current['position'][2]);initial=float(current['position'][2]);ticks=0;states=0;J=None
    for age in range(420):
     stamp.update(index=100+age,time=(100+age)/240);task.step_counter=100+age
     err=np.asarray(goal['position'])-current['position'];axis=np.asarray(spec['target_object_convergence_axis'],float);axis/=np.linalg.norm(axis)
     lateral.append(float(np.linalg.norm(err-np.dot(err,axis)*axis)));heights.append(float(current['position'][2]));minimum=min(minimum,heights[-1])
     try:command=adapter._assisted_absolute_object_servo_pose(phase_key=(0,),task=task,robot_name=robot,spec=spec,tracked_robots={},tracked_objects={},object_pose=current,target_object_pose=goal)
     except ValueError as e:violation=str(e);break
     if state['descent_ready']:
      warm=True;require(state['admission']['count']>=12 and lateral[-1]<=.00015+1e-12,'warm without actual12tick admission');break
     target=mat(command)
     def fk(x):
      qs={k:v.copy() for k,v in q.items()};qs[robot][:6]=x;return model.poses(qs)[tcp]
     def residual(x):
      t=fk(x);return np.r_[(t[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@t[:3,:3]).as_rotvec()]
     # Reused local Jacobian only affects this offline IK proxy; final production
     # object/limit guard checks the exact nonlinear FK target every tick.
     if J is None or age%25==0:
      z=residual(qprev);J=np.column_stack([(residual(qprev+np.eye(6)[j]*1e-6)-z)/1e-6 for j in range(6)])
     candidate=qprev.copy()
     for _ in range(2):candidate-=np.linalg.lstsq(J,residual(candidate),rcond=None)[0]
     require(np.linalg.norm(residual(candidate))<2e-5,'offline IK proxy convergence')
     request=Adapter._limited_joint_target(current_q=qprev,target_q=candidate,max_joint_step=spec['max_command_joint_step']);request=Adapter._limit_command_to_measured_state(current_q=qa,command_q=request,spec=spec)
     action=task._assisted_guard_actions({robot:{'arm_joint_controller':[request.tolist()]}});final=np.asarray(action[robot]['arm_joint_controller'][0]);ticks+=1
     require(np.max(abs(final-qa))<=.12+1e-12 and np.max(abs(final-qprev))<=.02+1e-12,'final SDK-prelimit budget')
     if enabled and (age%16==0 or age==0):
      actual_model=object_pose_from_tcp(pose(model.poses(q)[tcp]),rp['position'],rp['orientation'])
      for fraction in (0.,.5,1.):
       qs={k:v.copy() for k,v in q.items()};qs[robot][:6]=qa+fraction*(final-qa);poses=model.poses(qs)
       for bn,mt in f['parts'].items():poses[bn]=mt@np.linalg.inv(origin[bn])@poses[bn] if bn in origin and '/robots/' in bn else mt
       pred=anchored_object_prediction(current,actual_model,object_pose_from_tcp(pose(model.poses(qs)[tcp]),rp['position'],rp['orientation']));poses[body[name]]=mat(pred)
       require(not missing_dynamic_poses(model,poses),'missing full dynamic mesh');checker.check(poses,f['phase'],fi*10000+age,(robot,name));states+=1
       support.append(check_visual_support(checker,specs[f['phase']]['beam_assisted_visual_support'],f['phase'],age)['ready'])
     qa[:]+=alpha*(final-bias-qa);qprev=final
     actual=object_pose_from_tcp(pose(model.poses(q)[tcp]),rp['position'],rp['orientation']);new_current=pose(calibration@mat(actual))
     velocity=(new_current['position']-current['position'])*240
     angular=Rotation.from_matrix(mat(current)[:3,:3].T@mat(new_current)[:3,:3]).magnitude()*240
     motion.update(linear_velocity=velocity,linear_speed=float(np.linalg.norm(velocity)),angular_speed=float(angular));current=new_current
    row=dict(kind=f['kind'],phase=f['phase'],response=alpha,enabled=enabled,ticks=ticks,ready=warm,violation=violation,initial_lateral_m=lateral[0],final_lateral_m=lateral[-1],cold_descent_m=initial-minimum,mesh_states=states,actual_q_changed_rad=float(np.max(abs(qa-baseq[robot][:6]))),bias_rad=bias.tolist())
    results.append(row);save(P/(cycle+'_alignment_plant_progress.json'),dict(results=results));print(json.dumps(row),flush=True)
    if enabled:require(warm and violation is None and initial-minimum<=.0005+1e-12,'new nonlinear plant failed actual admission/height')
    elif fi==0:require(not warm and initial-minimum>.001,'actual-biased old controller negative control did not reproduce creeping cold hold')
 # All40 strict phase nominal endpoints plus four compensation directions and
 # actual cold-height boundary: newly affected geometry, independent of plant.
 anchors=0
 for phase,ps in specs.items():
  spec=ps.get('local_skill',{})
  if not spec.get('beam_assisted_alignment_hold'):continue
  rs=[r for r in path if r['phase']==phase];require(rs,'missing40phase path')
  for r in (rs[0],rs[-1]):
   qs={k:np.asarray(v,float) for k,v in r['q'].items()};original=model.poses(qs);name=spec['object'];target0=np.asarray(r['part_poses'][name]);rel=np.linalg.inv(original[tcp])@target0
   for dx,dy,dz in ((0,0,0),(.00012,0,-.0005),(-.00012,0,-.0005),(0,.00012,-.0005),(0,-.00012,-.0005)):
    goal=target0.copy();goal[:3,3]+=[dx,dy,dz];tcpgoal=goal@np.linalg.inv(rel)
    def res(x):
     qq={k:v.copy() for k,v in qs.items()};qq[robot][:6]=x;t=model.poses(qq)[tcp];return np.r_[(t[:3,3]-tcpgoal[:3,3])*5,Rotation.from_matrix(tcpgoal[:3,:3].T@t[:3,:3]).as_rotvec()]
    sol=least_squares(res,qs[robot][:6],max_nfev=20);require(np.linalg.norm(res(sol.x))<1e-6,'corridor IK failed');qend={k:v.copy() for k,v in qs.items()};qend[robot][:6]=sol.x
    poses=model.poses(qend)
    for bn,t in r['part_poses'].items():poses[body[bn]]=np.asarray(t)
    poses[body[name]]=goal;checker.check(poses,phase,anchors,(robot,name));support.append(check_visual_support(checker,ps['beam_assisted_visual_support'],phase,anchors)['ready']);anchors+=1
 mesh=checker.report();require(mesh['meshes']==54 and mesh['self_checked'] and all(support),'mesh/support coverage');require(not any(v['depth']>1e-6 for v in mesh['intersections']),'non-target CAD penetration')
 report=dict(passed=True,results=results,persistent_actual_plant=True,actual_plant_cases=sum(r['enabled'] for r in results),old_negative_controls=sum(not r['enabled'] for r in results),actual_first_entry_step=14216,source_manifest_sha256=sha(P/'tools/assisted_manifest.json'),configuration_sha256=sha(P/(cycle+'_actual_configuration_layout505_config.json')),full_mesh=mesh,corridor_phase_count=40,corridor_boundary_states=anchors,support_all_ready=all(support),tool_sha256=sha(__file__),native_physics_executed=0,
 limits=['Finite declared first-order tracking plants with measured A18 settled joint bias, not PhysX outcome. Recorded grasp reference frozen; actual q and body evolve every tick. Offline IK proxy; exact production final guards and full nonlinear FK checked. CAD sampled every16plant ticks at3sweep fractions and40phase endpoints with5offsets; no between-sample CCD. Openness fixture is not contact-force evidence. Runtime native/geometry/video gates remain mandatory.'])
 save(P/(cycle+'_alignment_closed_loop.json'),report);print(json.dumps(dict(passed=True,cases=len(results),mesh_states=mesh['samples'],corridor_states=anchors)))
if __name__=='__main__':main()
