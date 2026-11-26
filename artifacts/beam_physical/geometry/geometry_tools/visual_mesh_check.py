"""Read-only complete visible triangle checks, including shells and table legs."""
from pathlib import Path
import json, sys, itertools, os, time
import numpy as np
from sweep_model import SceneModel, pose_matrix
import fcl

class VisualMeshChecker:
 def __init__(self, model, include_self=False):
  self.model=model;self.meshes=model.data['visible_meshes'];self.shapes=[];self.corners=[]
  self.include_self=include_self
  self.joint_interfaces={frozenset([j['parent'][0],j['child'][0]]) for j in model.data['joints'] if j['robot'] and j['enabled'] and len(j['parent'])==len(j['child'])==1}
  self.intrinsic_interfaces=[]
  for mesh in self.meshes:
   points=np.asarray(mesh['points'],float);faces=np.asarray(mesh['triangles'],np.int32)
   shape=fcl.BVHModel();shape.beginModel(len(faces),len(points));shape.addSubModel(points,faces);shape.endModel()
   self.shapes.append(fcl.CollisionObject(shape))
   lo=points.min(axis=0);hi=points.max(axis=0)
   self.corners.append(np.array(list(itertools.product(*zip(lo,hi)))))
  self.rows=[];self.minimum=None;self.samples=0;self.pair_checks=0;self.grasp_contacts=[]

 def check(self, poses, phase, step, intended_grasp=None):
  bounds=[]
  for mesh,obj,corners in zip(self.meshes,self.shapes,self.corners):
   T=poses[mesh['body']];obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
   world=corners@T[:3,:3].T+T[:3,3];bounds.append((world.min(axis=0),world.max(axis=0)))
  self.samples+=1
  for i,j in itertools.combinations(range(len(self.meshes)),2):
   a,b=self.meshes[i],self.meshes[j]
   if a['body']==b['body']:continue
   if a['robot'] and a['robot']==b['robot']:
    if not self.include_self:continue
    if frozenset([a['body'],b['body']]) in self.joint_interfaces:continue
    # The two inner-knuckle bearings enter the authored gripper housing in
    # every pose, including untouched native initial rest. Keep these exact
    # manufactured interfaces visible in the report, never skip an arm body.
    housing=(a['body'].endswith('/Gripper/Robotiq_2F_85/base_link') and b['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle'))) or (b['body'].endswith('/Gripper/Robotiq_2F_85/base_link') and a['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle')))
    if housing:
     pair=[a['prim'],b['prim']]
     if pair not in self.intrinsic_interfaces:self.intrinsic_interfaces.append(pair)
     continue
   if not (a['robot'] or b['robot'] or str(a['object']).startswith('fabrica_beam_') or str(b['object']).startswith('fabrica_beam_')):continue
   mount=(a['object']=='factory_tabletop_visual' and b['robot'] and b['body']==self.model.data['robots'][b['robot']]['root_path']+'/base_link') or (b['object']=='factory_tabletop_visual' and a['robot'] and a['body']==self.model.data['robots'][a['robot']]['root_path']+'/base_link')
   if mount:continue
   grip=False
   if intended_grasp:
    robot,part=intended_grasp
    grip=(a['object']==part and b['robot']==robot and b['body'].endswith(('/left_inner_finger','/right_inner_finger'))) or (b['object']==part and a['robot']==robot and a['body'].endswith(('/left_inner_finger','/right_inner_finger')))
   self.pair_checks+=1
   la,ha=bounds[i];lb,hb=bounds[j];lower=float(np.linalg.norm(np.maximum(np.maximum(la-hb,lb-ha),0)))
   if lower>.003 and (self.minimum is None or lower>self.minimum['distance']):continue
   result=fcl.CollisionResult();hit=fcl.collide(self.shapes[i],self.shapes[j],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
   row={'phase':phase,'step':step,'mesh_a':a['prim'],'mesh_b':b['prim'],'body_a':a['body'],'body_b':b['body']}
   if hit:
    depth=max((float(c.penetration_depth) for c in result.contacts),default=0.)
    item={**row,'depth':depth,'contacts':[{'position':c.pos.tolist(),'depth':float(c.penetration_depth)} for c in result.contacts]}
    if grip:self.grasp_contacts.append({**row,'signed_separation':-depth})
    else:self.rows.append(item)
   else:
    distance=float(fcl.distance(self.shapes[i],self.shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
    if grip:self.grasp_contacts.append({**row,'signed_separation':distance})
    elif self.minimum is None or distance<self.minimum['distance']:self.minimum={**row,'distance':distance}

 def report(self):
  by_pair={}
  for r in self.rows:
   key=r['mesh_a']+' | '+r['mesh_b']
   if key not in by_pair:by_pair[key]={'count':0,'first':r,'max_depth':0.}
   by_pair[key]['count']+=1;by_pair[key]['max_depth']=max(by_pair[key]['max_depth'],r['depth'])
  return {'scope':'all54 exported visible authored triangle meshes; cross-arm, payload-arm, payload-payload, robot/payload-environment; exact current intended inner-finger grasp measured separately, fixed robot/table mounts excluded',
   'samples':self.samples,'meshes':len(self.meshes),'pair_checks':self.pair_checks,'intersections':self.rows,'pairs':list(by_pair.values()),'closest_separated_pair':self.minimum,
   'intended_grasp_mesh_separations':self.grasp_contacts,
   'minimum_intended_grasp_signed_separation':min((r['signed_separation'] for r in self.grasp_contacts),default=None),
   'self_checked':self.include_self,'intrinsic_housing_bearing_interfaces':self.intrinsic_interfaces,
   'gaps':([] if self.include_self else ['same-robot self topology'])+['unexported warehouse geometry','between-sample CCD','PhysX cooked collision/SDF differs from visible CAD']}

class ProductionCarryChecker:
 """Same-clock native scene and final-joint sweep; no reused nominal corridor.

 All exported meshes participate. Native bodies calibrate the FK increments
 at each tick; the captured object relation remains fixed outside this tool.
 """
 def __init__(self, model):
  self.model=model;self.checker=VisualMeshChecker(model,include_self=True)
  self.calls=0;self.samples=0;self.pairs=[];self.wall_seconds=0.
  c=self.checker
  if len(c.meshes)!=54 or model.gaps:
   raise ValueError('production CAD guard requires reviewed complete54 joint/body graph')
  for i,j in itertools.combinations(range(len(c.meshes)),2):
   a,b=c.meshes[i],c.meshes[j]
   if a['body']==b['body']:continue
   if a['robot'] and a['robot']==b['robot']:
    if frozenset([a['body'],b['body']]) in c.joint_interfaces:continue
    housing=(a['body'].endswith('/Gripper/Robotiq_2F_85/base_link') and b['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle'))) or (b['body'].endswith('/Gripper/Robotiq_2F_85/base_link') and a['body'].endswith(('/left_inner_knuckle','/right_inner_knuckle')))
    if housing:continue
   if not (a['robot'] or b['robot'] or str(a['object']).startswith('fabrica_beam_') or str(b['object']).startswith('fabrica_beam_')):continue
   mount=(a['object']=='factory_tabletop_visual' and b['robot'] and b['body']==model.data['robots'][b['robot']]['root_path']+'/base_link') or (b['object']=='factory_tabletop_visual' and a['robot'] and a['body']==model.data['robots'][a['robot']]['root_path']+'/base_link')
   if not mount:self.pairs.append((i,j))
  self.pair_indices=np.asarray(self.pairs,int);self.corners=np.asarray(c.corners)

 def _scene_check(self,poses,phase,index,intended):
  """Vectorized AABB broad phase; every overlapping pair reaches exact FCL.

  Far separated meshes have a proven AABB lower bound >3mm. Near pairs use
  the same exact triangles and exemptions as the independent legacy checker.
  """
  c=self.checker;c.rows.clear();c.grasp_contacts.clear();c.minimum=None
  transforms=np.asarray([poses[m['body']] for m in c.meshes])
  world=np.einsum('nkj,nij->nki',self.corners,transforms[:,:3,:3])+transforms[:,:3,3][:,None,:]
  lo=world.min(axis=1);hi=world.max(axis=1);ii,jj=self.pair_indices.T
  gaps=np.maximum(np.maximum(lo[ii]-hi[jj],lo[jj]-hi[ii]),0.)
  selected=np.flatnonzero(np.sum(gaps*gaps,axis=1)<=.003**2)
  for mesh,obj,t in zip(c.meshes,c.shapes,transforms):obj.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
  c.samples+=1;c.pair_checks+=len(self.pairs)
  for pair_index in selected:
   i,j=self.pairs[pair_index];a,b=c.meshes[i],c.meshes[j]
   robot,part=intended
   grip=(a['object']==part and b['robot']==robot and b['body'].endswith(('/left_inner_finger','/right_inner_finger'))) or (b['object']==part and a['robot']==robot and a['body'].endswith(('/left_inner_finger','/right_inner_finger')))
   row=dict(phase=phase,step=index,mesh_a=a['prim'],mesh_b=b['prim'],body_a=a['body'],body_b=b['body'])
   result=fcl.CollisionResult();hit=fcl.collide(c.shapes[i],c.shapes[j],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
   if hit:
    depth=max((float(v.penetration_depth) for v in result.contacts),default=0.)
    if grip:c.grasp_contacts.append(dict(**row,signed_separation=-depth))
    else:c.rows.append(dict(**row,depth=depth,contacts=[dict(position=v.pos.tolist(),depth=float(v.penetration_depth)) for v in result.contacts]))

 def check(self, **kwargs):
  start=time.perf_counter()
  try:return self._check(**kwargs)
  finally:self.wall_seconds+=time.perf_counter()-start

 def check_empty(self, *, native_poses, actual_q, other_targets, physics_stamp,
                 installed_objects, phase='empty_return'):
  """Sweep both final arm targets with open measured grippers and no payload.

  Installed parts retain their native world poses. No finger/part pair is
  treated as an intended grasp, and the existing full54/self pair set remains
  unchanged. The caller must reject attachments before this read-only check.
  """
  start=time.perf_counter()
  try:
   from visual_support_check import missing_dynamic_poses
   if (physics_stamp.get('valid') is not True
       or type(physics_stamp.get('epoch')) is not int
       or type(physics_stamp.get('index')) is not int
       or physics_stamp['epoch'] < 0 or physics_stamp['index'] < 0
       or not all(np.isfinite(physics_stamp.get(k, np.nan)) for k in ('time','dt'))
       or physics_stamp['time'] < 0 or physics_stamp['dt'] <= 0):
    raise ValueError('empty return CAD sweep lacks a valid native clock')
   expected={'fabrica_beam_'+n for n in ('6','3','2','1','0')}
   if len(installed_objects)!=5 or set(installed_objects)!=expected:
    raise ValueError('empty return CAD sweep must retain all five installed parts')
   missing=missing_dynamic_poses(self.model,native_poses)
   if missing:raise ValueError('empty return CAD sweep missing native bodies: '+repr(missing))
   robots=set(self.model.data['robots'])
   if len(robots)!=2 or set(actual_q)!=robots or set(other_targets)!=robots:
    raise ValueError('empty return CAD sweep lacks both measured arms/final targets')
   q={k:np.asarray(v,float).reshape(-1).copy() for k,v in actual_q.items()}
   end={k:v.copy() for k,v in q.items()}
   for name in robots:
    goal=np.asarray(other_targets[name],float)
    if (q[name].shape!=(12,) or not np.all(np.isfinite(q[name]))
        or np.max(abs(q[name][6:])) > .05):
     raise ValueError('empty return CAD sweep lacks a fully open native gripper')
    if (goal.shape!=(6,) or not np.all(np.isfinite(goal))
        or np.max(abs(goal-q[name][:6])) > .12+1e-12):
     raise ValueError('empty return CAD final arm target outside tracking envelope')
    end[name][:6]=goal
    # SDK openness1 drives the UR5/Robotiq gripper to its authored zero-open
    # branch. Sweep all six mimic/pad joints from the same native readback.
    end[name][6:]=0.
   actual={k:np.asarray(v,float) for k,v in native_poses.items()}
   if any(v.shape!=(4,4) or not np.all(np.isfinite(v)) for v in actual.values()):
    raise ValueError('empty return CAD sweep has an invalid native body pose')
   for name in expected:
    bodies={m['body'] for m in self.model.data['visible_meshes'] if m.get('object')==name}
    if len(bodies)!=1 or not bodies.issubset(actual):
     raise ValueError('empty return CAD lacks an exact native installed body: '+name)
   origin=self.model.poses(q)
   robot_bodies={m['body'] for m in self.model.data['visible_meshes'] if m.get('robot')}
   corrections={}
   for body in robot_bodies:
    if body not in origin or body not in actual:
     raise ValueError('empty return CAD lacks a full robot/gripper body: '+body)
    rest=origin[body];inverse=np.eye(4);inverse[:3,:3]=rest[:3,:3].T
    inverse[:3,3]=-inverse[:3,:3]@rest[:3,3]
    corrections[body]=actual[body]@inverse
   native={**self.model.rest,**actual};self.calls+=1;checked=0
   for fraction in (0.,.25,.5,.75,1.):
    qs={k:q[k]+fraction*(end[k]-q[k]) for k in q}
    fk=self.model.poses(qs);poses=dict(native)
    for body,correction in corrections.items():poses[body]=correction@fk[body]
    # Empty identifiers cannot match any configured robot/Beam object pair.
    self._scene_check(poses,phase,physics_stamp['index'],('',''))
    checked+=1;self.samples+=1
    failures=[r for r in self.checker.rows if r['depth']>1e-6]
    if self.checker.grasp_contacts:
     raise ValueError('empty return CAD unexpectedly exempted a grasp pair')
    if failures:
     return False,dict(passed=False,mode='empty_return',fraction=fraction,
      checked_samples=checked,first_collision=failures[0],
      complete_visible_meshes=len(self.checker.meshes),self_checked=True,
      intended_grasp_exemptions=False,physics_stamp=dict(physics_stamp))
   return True,dict(passed=True,mode='empty_return',checked_samples=checked,
    complete_visible_meshes=len(self.checker.meshes),self_checked=True,
    installed_objects=sorted(expected),intended_grasp_exemptions=False,
    physics_stamp=dict(physics_stamp),
    gaps=['finite five-fraction joint sweep; no between-sample CCD',
          'unexported warehouse and PhysX cooked geometry',
          'open-zero gripper endpoint is a kinematic prediction; native fully open readback still required'])
  finally:self.wall_seconds+=time.perf_counter()-start

 def _check(self, *, native_poses, actual_q, other_targets, robot_name,
           object_name, command_q, current_q, relative, physics_stamp,
           support=None, phase='loaded_free_move'):
  from scipy.spatial.transform import Rotation
  from visual_support_check import missing_dynamic_poses,check_visual_support
  from toolkits.factory_dual_franka_assembly.beam_assisted_servo import anchored_object_prediction,object_pose_from_tcp
  if (physics_stamp.get('valid') is not True
      or type(physics_stamp.get('epoch')) is not int
      or type(physics_stamp.get('index')) is not int):
   raise ValueError('production CAD sweep lacks a native clock')
  missing=missing_dynamic_poses(self.model,native_poses)
  if missing:raise ValueError('production CAD sweep missing native bodies: '+repr(missing))
  q={k:np.asarray(v,float).reshape(-1).copy() for k,v in actual_q.items()}
  if (set(q)!=set(self.model.data['robots'])
      or any(not np.all(np.isfinite(v)) for v in q.values())
      or np.max(abs(q[robot_name][:6]-current_q))>1e-9):
   raise ValueError('production CAD joint/frame identity mismatch')
  if set(other_targets)!=set(q):
   raise ValueError('production CAD sweep lacks both final arm targets')
  origin=self.model.poses(q);end={k:v.copy() for k,v in q.items()}
  for name,goal in other_targets.items():
   if name!=robot_name:
    goal=np.asarray(goal,float)
    if goal.shape!=(6,) or not np.all(np.isfinite(goal)) or np.max(abs(goal-q[name][:6]))>.12+1e-12:
     raise ValueError('production CAD other-arm target outside tracking envelope')
    end[name][:6]=goal
  end[robot_name][:6]=command_q
  actual={k:np.asarray(v,float) for k,v in native_poses.items()}
  if any(v.shape!=(4,4) or not np.all(np.isfinite(v)) for v in actual.values()):
   raise ValueError('production CAD sweep has an invalid native body pose')
  native={**self.model.rest,**actual}
  def rigid_inverse(t):
   inverse=np.eye(4);inverse[:3,:3]=t[:3,:3].T
   inverse[:3,3]=-inverse[:3,:3]@t[:3,3]
   return inverse
  corrections={b:actual[b]@rigid_inverse(origin[b]) for b in actual if b in origin and '/robots/' in b}
  body=next(m['body'] for m in self.model.data['visible_meshes'] if m.get('object')==object_name)
  tcp=self.model.data['robots'][robot_name]['root_path']+'/wrist_3_link'
  def pose(t):return dict(position=t[:3,3],orientation=Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]])
  current=pose(actual[body]);measured=object_pose_from_tcp(pose(origin[tcp]),*relative)
  self.calls+=1;checked=0
  for fraction in (0.,.25,.5,.75,1.):
   qs={k:q[k]+fraction*(end[k]-q[k]) for k in q};fk=self.model.poses(qs);poses=dict(native)
   for b,c in corrections.items():poses[b]=c@fk[b]
   pred=anchored_object_prediction(current,measured,object_pose_from_tcp(pose(fk[tcp]),*relative))
   poses[body]=pose_matrix(pred['position'],pred['orientation'])
   c=self.checker
   self._scene_check(poses,phase,physics_stamp['index'],(robot_name,object_name));checked+=1;self.samples+=1
   failures=[r for r in c.rows if r['depth']>1e-6]
   grips=[r for r in c.grasp_contacts if r['signed_separation'] < -1e-6]
   support_detail=(check_visual_support(c,support,phase,physics_stamp['index'])
                   if support and fraction in (0.,1.) else None)
   # The 5--250um qualification is a measured native-pose contract. A final
   # idle-arm drive target has a real steady tracking bias and is not a new
   # native support observation. All predicted non-target overlaps still fail.
   if failures or grips or (fraction==0. and support_detail and not support_detail['ready']):
    return False,dict(passed=False,fraction=fraction,checked_samples=checked,
     first_collision=failures[0] if failures else None,
     first_grasp_overlap=grips[0] if grips else None,visual_support=support_detail,
     complete_visible_meshes=len(c.meshes),physics_stamp=dict(physics_stamp))
  return True,dict(passed=True,checked_samples=checked,complete_visible_meshes=len(self.checker.meshes),
   self_checked=True,physics_stamp=dict(physics_stamp),
   predicted_final_visual_support=support_detail,
   predicted_support_is_not_native_qualification=True,
   gaps=['finite five-fraction joint sweep; no between-sample CCD',
         'unexported warehouse and PhysX cooked geometry',
         'gripper joints held at measured aperture during loaded free motion'])


if __name__=='__main__':
 P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1]
 model=SceneModel(D/'scene_geometry.json');checker=VisualMeshChecker(model)
 if '--nominal-base' in sys.argv:
  label=os.environ.get('BEAM_BASE_PREFLIGHT_LABEL','base_nominal')
  payload_body=next(c.body for c in model.checker.colliders if c.entity=='fabrica_beam_6')
  for step,line in enumerate((P/(label+'_path.jsonl')).open()):
   r=json.loads(line);poses=model.poses(r['q']);poses[payload_body]=np.asarray(r['payload_pose'])
   intended=None if r['segment'] in ['base_6_retreat','base_6_park','base_6_world_settle'] else ('franka_right','fabrica_beam_6')
   checker.check(poses,r['segment'],step,intended)
  report=checker.report();(P/(label+'_visible_mesh_report.json')).write_text(json.dumps(report,indent=2)+'\n')
  print(json.dumps({k:v for k,v in report.items() if k not in ['intersections','pairs','intended_grasp_mesh_separations']}));print(json.dumps({'pairs':report['pairs']})[:8000]);sys.exit(0)
 config=json.loads((D/'task_config_snapshot.json').read_text())['task_configs'][0];phases={p['name']:p for p in config['phase_specs']}
 for line in (D/'entity_geometry_trace.jsonl').open():
  r=json.loads(line);p=phases.get(r['phase'],{});s=p.get('local_skill',{});intended=None
  if s.get('requires_held_object') or r['phase'].endswith('_close_and_attach'):intended=(s['robot'],s['object'])
  elif r['phase'].endswith('_release_and_lock'):
   parts=p.get('lock',[]);commands=p.get('gripper_commands',{})
   if len(parts)==1 and len(commands)==1:intended=(next(iter(commands)),parts[0]['object'])
  poses={**model.rest,**{k:np.asarray(v,float) for k,v in r['body_poses'].items()}}
  checker.check(poses,r['phase'],r['step'],intended)
 report=checker.report();(D/'visual_mesh_audit.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps({k:v for k,v in report.items() if k not in ['intersections','pairs','intended_grasp_mesh_separations']}));print(json.dumps({'pairs':report['pairs']})[:8000])
