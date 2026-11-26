"""Read-only CAD checks of final published joint skills, before SDK submission.

Full physical task mesh set, both actual arm targets, independent open-gripper
sweep. Visual CAD candidates are conservative stops, not measured collisions.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import itertools,json,time,sys,numpy as np
class JointCADGuard:
 def __init__(self,geometry,geometry_tools,out):
  sys.path.insert(0,str(geometry_tools))
  from sweep_model import SceneModel
  from visual_mesh_check import VisualMeshChecker,ProductionCarryChecker
  self.model=SceneModel(geometry);self.model_module=SceneModel.__module__;self.out=Path(out);self.reader=None;self.calls=0;self.samples=0;self.seconds=0.;self.last_dump=0.;self.last=None;self.by_phase={};self.first_inputs={};self.release_limits={}
  c=VisualMeshChecker(self.model,include_self=True);assert len(c.meshes)==52 and not self.model.gaps
  pairs=[]
  for i,j in itertools.combinations(range(len(c.meshes)),2):
   a,b=c.meshes[i],c.meshes[j]
   if a['body']==b['body']:continue
   if a['robot']and a['robot']==b['robot']:
    if frozenset([a['body'],b['body']])in c.joint_interfaces:continue
    housing=(a['body'].endswith('/Gripper/Robotiq_2F_85/base_link')and b['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle')))or(b['body'].endswith('/Gripper/Robotiq_2F_85/base_link')and a['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle')))
    if housing:continue
   mount=(a['object']=='factory_tabletop_visual'and b['robot']and b['body']==self.model.data['robots'][b['robot']]['root_path']+'/base_link')or(b['object']=='factory_tabletop_visual'and a['robot']and a['body']==self.model.data['robots'][a['robot']]['root_path']+'/base_link')
   if not mount:pairs.append((i,j))
  self.query=NS(checker=c,pairs=pairs,pair_indices=np.array(pairs,int),corners=np.array(c.corners))
  self.scene_check=lambda poses,phase,index:ProductionCarryChecker._scene_check(self.query,poses,phase,index,('',''))
 def specs(self,task):
  from toolkits.factory_dual_franka_assembly.local_skills import normalize_local_skill_spec
  p=task.get_current_phase_spec();return {r:s for r in self.model.data['robots']if(s:=normalize_local_skill_spec(p,r))and (s.get('name')in ['move_arm_to_joint_positions','ur5e_move_arm_to_joint_positions'] or s.get('beam_free_transit'))}
 def flush(self,final=False):
  data=dict(final=final,calls=self.calls,samples=self.samples,wall_seconds=self.seconds,by_phase=self.by_phase,first_inputs=self.first_inputs,last=self.last,meshes=52,scope='Final joint skills and explicitly unloaded Cartesian transit targets; both full arms/actual geometry, held-payload geometric prediction and independent empty-gripper opening. Unchanged contact/carry controllers retain original monitors. CAD guard time is inside policy_act, not additive. Finite midpoint sampling; no CCD/cooked PhysX proof.')
  p=self.out/'joint_cad_guard.json';t=p.with_suffix('.tmp');t.write_text(json.dumps(data,indent=2)+'\n');t.replace(p);self.last_dump=time.monotonic()
 def check(self,task,actions):
  specs=self.specs(task)
  if not specs:return True,None
  from toolkits.constraint_checking.integration.entity_geometry import TaskEntityGeometryMonitor,pose_matrix
  from toolkits.constraint_checking.integration.contact_policy import AssemblyContactPolicy
  if self.reader is None:self.reader=TaskEntityGeometryMonitor(task)
  start=time.perf_counter();self.calls+=1;phase=task.phase;self.by_phase[phase]=self.by_phase.get(phase,0)+1
  try:
   stamp=task._beam_physics_stamp();assert stamp.get('valid')is True and isinstance(stamp.get('index'),int)
   native={**self.model.rest,**self.reader.current_poses()}
   q={r:np.asarray(task.robots[r].articulation.get_joint_positions(),float).reshape(-1)for r in self.model.data['robots']};end={r:v.copy()for r,v in q.items()}
   assert all(v.shape==(12,)and np.all(np.isfinite(v))for v in q.values())
   holders=list(task._all_physical_grasps());moving=set(specs)
   for r in moving:
    assert not any(a.get('robot_name')==r for _,a in holders),'joint_skill_requires_empty_arm'
    if not specs[r].get('beam_free_transit'):assert np.max(np.abs(q[r][6:]))<=.05,'joint_skill_gripper_not_yet_open'
   for r in q:
    arm=(actions.get(r)or{}).get('arm_joint_controller')
    if arm is not None:
     target=np.asarray(arm[0],float);assert target.shape==(6,)and np.all(np.isfinite(target));end[r][:6]=target
    elif r in moving:raise ValueError('joint skill final target missing')
   def input_context():
    return dict(physics_stamp=stamp,actual_q={r:v.tolist()for r,v in q.items()},native_body_poses={b:v.tolist()for b,v in native.items()},final_arm_targets={r:v[:6].tolist()for r,v in end.items()},holders=[dict(object=n,robot=a.get('robot_name'),mode=a.get('mode'))for n,a in holders])
   if phase not in self.first_inputs:self.first_inputs[phase]=input_context()
   origin=self.model.poses(q);body_robot={m['body']:m['robot']for m in self.model.data['visible_meshes']if m['robot']}
   corrections={b:native[b]@np.linalg.inv(origin[b])for b in body_robot}
   policy=AssemblyContactPolicy();snapshot=policy.snapshot_from_task(task)
   # A released pad may start in its explicitly named former target. Allow
   # only that existing target/pad overlap to separate, never to deepen.
   self.scene_check(native,phase,stamp['index'])
   release_overlaps={}
   for row in self.query.checker.rows:
    pair=frozenset([row['body_a'],row['body_b']])
    for r,s in specs.items():
     prefix='/robots/'+('ur5e_left'if r=='franka_left'else'ur5e_right')+'/Gripper/'
     obj=s.get('object')
     if (s.get('beam_free_transit') and s.get('name')=='ur5e_retreat_vertical' and obj
       and any(prefix in b for b in pair)
       and any(b.endswith('/objects/'+obj) for b in pair)):
      release_overlaps[pair]=row['depth']
   initial=self.release_limits.setdefault(phase,release_overlaps.copy())
   for pair in initial:initial[pair]=min(initial[pair],release_overlaps.get(pair,0.))
   release_overlaps=initial
   sample_count=0
   for arm_fraction in [0.,.5,1.]:
    for grip_fraction in [0.,1.]:
     sample={r:q[r]+arm_fraction*(end[r]-q[r])for r in q}
     for r in moving:sample[r][6:]=(1-grip_fraction)*q[r][6:]
     fk=self.model.poses(sample);poses=dict(native)
     for b,c in corrections.items():poses[b]=c@fk[b]
     # Predict payload geometry from this instantaneous measured relation only;
     # no task state, anchor, body pose or force qualifier is written.
     for obj,a in holders:
      r=a.get('robot_name');paths={m['body']for m in self.model.data['visible_meshes']if m.get('object')==obj}
      if r in q and len(paths)==1:
       b=next(iter(paths));tcp=self.model.data['robots'][r]['root_path']+'/Gripper/Robotiq_2F_85/base_link';poses[b]=poses[tcp]@np.linalg.inv(native[tcp])@native[b]
     self.scene_check(poses,phase,stamp['index']);sample_count+=1;self.samples+=1
     for row in self.query.checker.rows:
      # Full other-arm geometry is predicted above. Only the newly activated
      # joint-controlled arm can create a changed-path stop; unchanged holding
      # controllers keep their existing force/slip/geometry validation.
      if not any('/robots/'+('ur5e_left'if r=='franka_left'else'ur5e_right')+'/'in row['body_a']or'/robots/'+('ur5e_left'if r=='franka_left'else'ur5e_right')+'/'in row['body_b']for r in moving):continue
      allowed=release_overlaps.get(frozenset([row['body_a'],row['body_b']]),0.)
      if row['depth']>max(1e-6,allowed+1e-6):
       self.last=dict(passed=False,phase=phase,physics_stamp=stamp,arm_fraction=arm_fraction,gripper_fraction=grip_fraction,collision_candidate=row,input=input_context());return False,self.last
   self.last=dict(passed=True,phase=phase,physics_stamp=stamp,checked_samples=sample_count,final_targets={r:end[r][:6].tolist()for r in q});return True,self.last
  except Exception as exc:
   self.last=dict(passed=False,phase=phase,reason='joint_CAD_probe_or_contract_error',error=repr(exc));
   if 'input_context'in locals():self.last['input']=input_context()
   return False,self.last
  finally:
   self.seconds+=time.perf_counter()-start
   if not self.last or not self.last.get('passed')or time.monotonic()-self.last_dump>60:self.flush()
 def install(self,policy_class):
  original=policy_class.act
  def act(policy,task,*args,**kwargs):
   actions=original(policy,task,*args,**kwargs)
   if getattr(task,'failed',False)or getattr(task,'success',False):return actions
   passed,detail=self.check(task,actions)
   if passed:return actions
   task._set_terminal_state('failed',reason='joint-final-command-CAD-screen',status='failed',detail=detail)
   return {r:{}for r in task.config.robot_names}
  policy_class.act=act
