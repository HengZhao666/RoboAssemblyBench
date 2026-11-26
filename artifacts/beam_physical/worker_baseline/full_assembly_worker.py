"""Full assembly worker with read-only gripper sweep diagnostics; no pose/velocity restore."""
from pathlib import Path
import json, os, sys
C=Path('/home/user/beam_integration_20261010_upstream6f57dae/candidate_repository')
sys.path.insert(0,str(C))
from toolkits.factory_dual_franka_assembly import generate_demos as demo

original_build=demo._build_env
def build(*args,**kwargs):
 configs=kwargs.get('task_configs',args[0] if args else [])
 out=Path(os.environ['BEAM_CONTACT_TRACE_PATH']).parent
 demo._write_json(out/'task_config_snapshot.json',{'task_configs':[c.model_dump() for c in configs]})
 env=original_build(*args,**kwargs)
 from internutopia_extension.tasks.factory_dual_franka_assembly_task import FactoryDualFrankaAssemblyTask as T
 original_getter=T.get_tracked_object_states
 guard_path=out/'collision_observer_getter_contract.json'
 guard={'enabled':True,'all_getter_calls':0,'observer_getter_calls':0,'held_phases_observed':[],
        'scope':'Runtime native contract; reject collision integration qualification getters before invocation'}
 last_guard_step=[None]
 def save_guard():
  guard_path.write_text(json.dumps(guard,indent=2)+'\n')
 save_guard()
 def qualified_getter(self):
  guard['all_getter_calls']+=1
  frame=sys._getframe(1)
  while frame is not None:
   filename=frame.f_code.co_filename
   if '/toolkits/constraint_checking/integration/' in filename:
    guard['observer_getter_calls']+=1
    guard['last_violation']={'step':int(self.step_counter),'phase':self.phase,'caller':filename}
    save_guard()
    raise RuntimeError('Collision observer attempted grasp qualification state getter')
   frame=frame.f_back
  if self._attachments and self.phase not in guard['held_phases_observed']:
   guard['held_phases_observed'].append(self.phase);save_guard()
  step=int(self.step_counter)
  if step%240==0 and last_guard_step[0]!=step:
   guard['last_sample_step']=step;last_guard_step[0]=step;save_guard()
  return original_getter(self)
 T.get_tracked_object_states=qualified_getter
 original_trace=T._write_physical_contact_trace
 def trace(self,object_name,metrics,attach_spec,strict_contact,**kw):
  original_trace(self,object_name,metrics,attach_spec,strict_contact,**kw)
  if not self.phase.startswith(('assemble_', 'hold_', 'handover_')) or not object_name.startswith('fabrica_beam_'): return
  if int(self.step_counter)%8 or getattr(self,'_short_sweep_last_step',None)==int(self.step_counter): return
  self._short_sweep_last_step=int(self.step_counter)
  robot_name=metrics['robot']; links=[]; pairs=[]
  for link in ('base_link','left_outer_knuckle','left_outer_finger','left_inner_finger','left_inner_knuckle',
               'right_outer_knuckle','right_outer_finger','right_inner_finger','right_inner_knuckle'):
   body=self._robot_rigid_body_by_suffix(robot_name,'Robotiq_2F_85/'+link)
   if body is None:
    links.append({'link':link,'valid':False});continue
   path=str(body.unwrap().prim_path); p,q=body.get_pose()
   links.append({'link':link,'valid':True,'path':path,'position':p.tolist(),'orientation':q.tolist()})
   for name,prim in self._object_prims.items():
    other=str(prim.GetPath())
    observation=self._pair_contact_observation(path,other)
    pairs.append({'link':link,'other_object':name,**observation})
  path=Path(os.environ['BEAM_GRASP_SWEEP_TRACE_PATH'])
  with path.open('a') as f:
   f.write(json.dumps({'step':int(self.step_counter),'phase':self.phase,'robot':robot_name,
                      'links':links,'pairs':pairs,'stride':8})+'\n')
 T._write_physical_contact_trace=trace
 # Observe the shell without changing transforms, visibility or physics.
 from pxr import Usd, UsdGeom, UsdShade, UsdPhysics
 original_update=T._update_task_state
 def update(self):
  result=original_update(self)
  step=int(self.step_counter)
  if step % (8 if step <= 2600 else 240): return result
  stamp=self._beam_physics_stamp()
  key=(stamp.get('epoch'),stamp.get('index'))
  if getattr(self,'_shell_last_clock',None)==key:return result
  self._shell_last_clock=key
  from isaacsim.core.utils.stage import get_current_stage
  stage=get_current_stage();cache=UsdGeom.XformCache();items=[]
  for robot_name, runtime_robot in self.robots.items():
   root=str(runtime_robot.articulation._root_prim_path)
   # Resolve the unique full suffix; never map both base_link names together.
   prims=[p for p in Usd.PrimRange(stage.GetPrimAtPath(root),Usd.TraverseInstanceProxies())
          if str(p.GetPath()).endswith('/Robotiq_2F_85/base_link')]
   for base in prims:
    visual=stage.GetPrimAtPath(base.GetPath().AppendPath('visuals'))
    record={'robot':robot_name,'base':str(base.GetPath()),'base_world_matrix':str(cache.GetLocalToWorldTransform(base)),
            'rigid_enabled':base.GetAttribute('physics:rigidBodyEnabled').Get(),'meshes':[],'colliders':[]}
    for p in list(Usd.PrimRange(visual,Usd.TraverseInstanceProxies())) + list(Usd.PrimRange(stage.GetPrimAtPath(base.GetPath().AppendPath('beam_housing_visual')),Usd.TraverseInstanceProxies())):
     if not p.IsA(UsdGeom.Gprim):continue
     img=UsdGeom.Imageable(p);g=UsdGeom.Gprim(p);mat,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
     record['meshes'].append({'path':str(p.GetPath()),'instance_proxy':p.IsInstanceProxy(),
        'visibility':str(img.ComputeVisibility()),'purpose':str(img.ComputePurpose()),
        'opacity':list(g.GetDisplayOpacityAttr().Get() or []),'material':str(mat.GetPath()) if mat else None,
        'collision_api':p.HasAPI(UsdPhysics.CollisionAPI),'world_matrix':str(cache.GetLocalToWorldTransform(p))})
    for p in Usd.PrimRange(base,Usd.TraverseInstanceProxies()):
     if p.HasAPI(UsdPhysics.CollisionAPI):record['colliders'].append({'path':str(p.GetPath()),'enabled':UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()})
    items.append(record)
  with (out/'shell_visual_trace.jsonl').open('a') as f:
   f.write(json.dumps({'step':step,'phase':self.phase,'physics_stamp':stamp,'shells':items})+'\n')
  return result
 T._update_task_state=update
 return env
demo._build_env=build
demo.main()
