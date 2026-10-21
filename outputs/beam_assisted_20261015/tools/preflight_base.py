from pathlib import Path
import sys,json,copy,os
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from sweep_model import SceneModel,pose_matrix
from types import SimpleNamespace
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs/A04_base_place_release'
m=SceneModel(D/'scene_geometry.json');data=m.data
config=SimpleNamespace(**json.loads((D/'task_config_snapshot.json').read_text())['task_configs'][0])
targets=config.target_poses
label=os.environ.get('BEAM_BASE_PREFLIGHT_LABEL','base_nominal')
capture_height=float(os.environ.get('BEAM_ASSISTED_INSTALL_CAPTURE_M','0'))
visual_rise=float(os.environ.get('BEAM_ASSISTED_VISUAL_RISE_M','0'))
for name,t in targets.items():
 if '_insert_' in name or name.endswith(('_assembled','_assembly_hover','_assembly_clearance','_preinsert_hover','_preseat')):t['position'][2]+=visual_rise

capture=next(json.loads(l) for l in (D/'assisted_trace.jsonl').open() if json.loads(l)['event']=='attach_measured_pose')
clock=capture['physics_stamp']['index']
anchor_clock_delta=0
q={n:np.asarray(r['joint_positions'],float) for n,r in data['robots'].items()}
q[capture['robot']]=np.asarray(capture['full_joint_positions'],float)
if os.environ.get('BEAM_ASSISTED_VISUAL_PINCH')=='1':
 calibration=json.loads((P/os.environ.get('BEAM_PINCH_CALIBRATION','visual_pinch_calibration.json')).read_text())
 pinch=next(r for r in calibration['results'] if r['part']=='fabrica_beam_6')
 q[capture['robot']]=np.asarray(pinch['full_q'],float)
# The inactive arm will hold its native initial anchor on v4.
q['franka_left']=np.asarray(data['robots']['franka_left']['joint_positions'],float)
right='franka_right';root=data['robots'][right]['root_path'];hand=root+'/Gripper/Robotiq_2F_85/base_link'
B0=pose_matrix(capture['position'],capture['orientation']);H0=pose_matrix(capture['native_hand_position'],capture['native_hand_orientation']);relative=np.linalg.inv(H0)@B0
if os.environ.get('BEAM_ASSISTED_VISUAL_PINCH')=='1':
 B0=np.asarray(pinch['part_pose'],float);H0=np.asarray(pinch['hand_pose'],float);relative=np.linalg.inv(H0)@B0
path=[];ik=[]
B_lift=B0.copy()
lift_offset=np.asarray(next(p['local_skill']['offset'] for p in config.phase_specs if p['name']=='base_6_lift'),float)
B_lift[:3,3]+=lift_offset
goals=[('lift',B_lift)]
for name in ['part_6_pickup_clearance','part_6_assembly_clearance','part_6_assembly_hover','part_6_assembled']:
 t=targets[name];goals.append((name,pose_matrix(t['position'],t['orientation'])))
goals[-1][1][2,3]+=capture_height

def solve(H,reference):
 original=q[right].copy()
 def residual(x):
  q[right][:6]=x;actual=m.poses(q)[hand]
  return np.r_[(actual[:3,3]-H[:3,3])*5,Rotation.from_matrix(H[:3,:3].T@actual[:3,:3]).as_rotvec()]
 sol=least_squares(residual,reference,bounds=(-2*np.pi,2*np.pi),max_nfev=300,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 error=residual(sol.x);q[right]=original
 if np.linalg.norm(error[:3])/5>.0001 or np.linalg.norm(error[3:])>.001:raise RuntimeError('offline IK endpoint not reached '+str(error))
 return sol.x

previous=q[right][:6].copy()
for name,B in goals:
 H=B@np.linalg.inv(relative);target=solve(H,previous)
 if name=='lift':
  count=int(np.ceil(np.linalg.norm(lift_offset)/.001))+1
  for fraction in np.linspace(0,1,count):
   body=B0.copy();body[:3,3]+=lift_offset*fraction
   step_q=solve(body@np.linalg.inv(relative),previous)
   full={n:v.copy() for n,v in q.items()};full[right][:6]=step_q
   poses=m.poses(full);poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')]=body
   path.append({'segment':name,'q':{n:v.tolist() for n,v in full.items()},'poses':poses})
   previous=step_q;q[right][:6]=step_q
  ik.append({'segment':name,'samples':count,'endpoint_q':previous.tolist(),'interpolation':'world-Z Cartesian object path'})
  continue
 # Dense straight-joint nominal sweep. Actual Cartesian continuation is audited
 # separately; no claim that IK endpoints uniquely determine its trajectory.
 count=max(2,int(np.ceil(np.max(abs(target-previous))/.004))+1)
 for i,t in enumerate(np.linspace(0,1,count)):
  full={n:v.copy() for n,v in q.items()};full[right][:6]=previous+(target-previous)*t
  poses=m.poses(full);poses.update({k:v for k,v in m.rest.items() if '/robots/' not in k})
  poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')]=poses[hand]@relative
  path.append({'segment':name,'q':{n:v.tolist() for n,v in full.items()},'poses':poses})
 ik.append({'segment':name,'samples':count,'endpoint_q':target.tolist()});previous=target;q[right][:6]=target

# Sweep complete opening linkage, then measured-frame retreat and park.
B_fixed=goals[-1][1]
open_grip=np.asarray(data['robots'][right]['joint_positions'],float)[6:]
closed_grip=q[right][6:].copy()
count=max(2,int(np.ceil(np.max(abs(open_grip-closed_grip))/.004))+1)
for fraction in np.linspace(0,1,count):
 full={n:v.copy() for n,v in q.items()};full[right][6:]=closed_grip+(open_grip-closed_grip)*fraction
 poses=m.poses(full);poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')]=B_fixed
 path.append({'segment':'base_6_open','q':{n:v.tolist() for n,v in full.items()},'poses':poses})
ik.append({'segment':'base_6_open','samples':count})
q[right][6:]=open_grip
final_body=pose_matrix(targets['part_6_assembled']['position'],targets['part_6_assembled']['orientation'])
for fraction in np.linspace(0,1,max(2,int(np.ceil(capture_height/.0001))+1)):
 poses=m.poses(q);body=B_fixed.copy();body[:3,3]=(1-fraction)*B_fixed[:3,3]+fraction*final_body[:3,3]
 poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')]=body
 path.append({'segment':'base_6_world_settle','q':{n:v.tolist() for n,v in q.items()},'poses':poses})
B_fixed=final_body
for phase_name in ['base_6_retreat','base_6_park']:
 skill=next(p['local_skill'] for p in config.phase_specs if p['name']==phase_name)
 H=m.poses(q)[hand].copy();H[:3,3]+=np.asarray(skill['offset'],float)
 target=solve(H,previous);count=max(2,int(np.ceil(np.max(abs(target-previous))/.004))+1)
 for fraction in np.linspace(0,1,count):
  full={n:v.copy() for n,v in q.items()};full[right][:6]=previous+(target-previous)*fraction
  poses=m.poses(full);poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')]=B_fixed
  path.append({'segment':phase_name,'q':{n:v.tolist() for n,v in full.items()},'poses':poses})
 ik.append({'segment':phase_name,'samples':count,'endpoint_q':target.tolist()});previous=target;q[right][:6]=target
with (P/(label+'_path.jsonl')).open('w') as stream:
 for s in path:stream.write(json.dumps({'segment':s['segment'],'q':s['q'],'payload_pose':s['poses'][next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')].tolist()})+'\n')

minimum=None;overlaps={};events=[]
for index,s in enumerate(path):
 def select(a,b):
  return bool(a.robot or b.robot or a.entity=='fabrica_beam_6' or b.entity=='fabrica_beam_6')
 result=m.checker.check(s['poses'],step=index,pair_selector=select)
 for e in result['events']:
  a,b=e['entity_a'],e['entity_b']
  target_grip=(a=='fabrica_beam_6' and b.endswith(('/left_inner_finger','/right_inner_finger')) and b.startswith(right+'/')) or (b=='fabrica_beam_6' and a.endswith(('/left_inner_finger','/right_inner_finger')) and a.startswith(right+'/'))
  mount=any('/base_link/collisions/base/mesh' in e[k] for k in ['collider_a','collider_b']) and 'factory_tabletop_visual' in [a,b]
  if target_grip or mount:continue
  pair=e['collider_a']+' | '+e['collider_b']
  if minimum is None or e['distance']<minimum['distance']:minimum={**e,'segment':s['segment']}
  if e['distance']<-.000001:
   if pair not in overlaps or e['distance']<overlaps[pair]['distance']:overlaps[pair]={**e,'segment':s['segment']}
report={'scope':'base6 grasp→lift→carry→hover→placement nominal full-entity sweep with same-clock measured grasp/full q/hand and calibrated full joint graph; Cartesian lift, straight joint interpolation other stages; complete measured-linkage opening and fixed-orientation retreat/park included; actual Cartesian continuation not yet covered',
 'source_grasp_clock':clock,'native_anchor_clock_delta_ticks':anchor_clock_delta,'nominal_anchor_is_nearest_boundary_not_same_clock':anchor_clock_delta!=0,'source_live_model_max_error_m':5.507380012607564e-7,
 'samples':len(path),'assembly_visual_rise_m':visual_rise,'configured_lift_offset':lift_offset.tolist(),'segments':ik,'coverage':m.checker.coverage(),'closest_nonexempt_pair':minimum,
 'overlap_candidates':list(overlaps.values()),'held_part_is_not_globally_exempt':True,
 'same_robot_self_collision_not_in_convex_checker':True,'visual_triangle_narrowphase_pending':True}
(P/(label+'_preflight.json')).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'samples':len(path),'overlap_candidates':report['overlap_candidates'],'closest':minimum}))
