from pathlib import Path
import json,sys,os
import numpy as np
from sweep_model import SceneModel
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');m=SceneModel(P/'runs/A08b_beam_full/scene_geometry.json')
meshes=[]
for mesh in m.data['visible_meshes']:
 v=np.asarray(mesh['points'],float);f=np.asarray(mesh['triangles'],int)
 meshes.append((mesh,v.min(0),v.max(0),v[f]))
eye_by_robot={'franka_left':np.array([0.,-.16,.08]),'franka_right':np.array([0.,.16,.08])};target_local=np.array([0.,0.,.17]);failures=[];count=0;possible_inside=[]
if os.environ.get('BEAM_CAMERA_EYE_MODE')=='broadside':eye_by_robot={k:np.array([.16,0.,.08]) for k in eye_by_robot}
if os.environ.get('BEAM_CAMERA_EYE_MODE')=='raised_side':eye_by_robot={k:np.array([-.12,0.,.12]) for k in eye_by_robot}
if os.environ.get('BEAM_CAMERA_EYE_MODE')=='oblique_side':eye_by_robot={'franka_left':np.array([-.12,-.12,.12]),'franka_right':np.array([-.12,.12,.12])}
pad_centers={}
for robot,info in m.data['robots'].items():
 pad_centers[robot]=[]
 for finger in ['left_inner_finger','right_inner_finger']:
  body=info['root_path']+'/Gripper/Robotiq_2F_85/'+finger
  mesh=next(x for x in m.data['visible_meshes'] if x['body']==body and 'fingertipsstep' in x['prim'])
  pad_centers[robot].append((body,np.asarray(mesh['points'],float).mean(0)))
def audit(poses,label,index):
 global count
 count+=1
 for robot,info in m.data['robots'].items():
  T=poses[info['root_path']+'/wrist_3_link'];eye=T[:3,:3]@eye_by_robot[robot]+T[:3,3]
  target=np.mean([poses[b][:3,:3]@p+poses[b][:3,3] for b,p in pad_centers[robot]],axis=0)
  for mesh,lo,hi,tri in meshes:
   B=poses.get(mesh['body'],m.rest[mesh['body']]);p=(eye-B[:3,3])@B[:3,:3];end=(target-B[:3,3])@B[:3,:3]
   if np.all(p>=lo) and np.all(p<=hi):possible_inside.append(dict(label=label,index=index,robot=robot,mesh=mesh['prim']))
   if mesh['robot']!=robot or '/Gripper/' in mesh['body']:continue
   if np.any(np.maximum(p,end)<lo) or np.any(np.minimum(p,end)>hi):continue
   direction=end-p;a,b,c=tri[:,0],tri[:,1],tri[:,2];e1=b-a;e2=c-a;h=np.cross(direction,e2);den=np.einsum('ij,ij->i',e1,h);mask=abs(den)>1e-12
   inv=np.zeros_like(den);inv[mask]=1/den[mask];s=p-a;u=np.einsum('ij,ij->i',s,h)*inv;q=np.cross(s,e1);v=q@direction*inv;t=np.einsum('ij,ij->i',e2,q)*inv
   hits=mask&(u>=0)&(v>=0)&(u+v<=1)&(t>1e-6)&(t<1-1e-6)
   if np.any(hits):failures.append(dict(label=label,index=index,robot=robot,mesh=mesh['prim'],first_fraction=float(t[hits].min())))
for source in sys.argv[1:]:
 path=P/source
 for i,line in enumerate(path.open()):
  r=json.loads(line)
  if 'q' in r:
   poses=m.poses(r['q'])
   for n,B in r.get('part_poses',{}).items():poses[next(k for k in poses if k.endswith('/'+n))]=np.asarray(B)
   if 'payload_pose' in r:poses[next(k for k in poses if k.endswith('/fabrica_beam_6'))]=np.asarray(r['payload_pose'])
  else:poses={**m.rest,**{k:np.asarray(B) for k,B in r['body_poses'].items()}}
  audit(poses,source,i)
report=dict(scope='native and dense nominal full joint graph; both optical eyes against all54 visible mesh AABBs; ray to actual coupled visible pad midpoint against own non-gripper arm triangles; AABB inside is a conservative candidate, actual image still required',samples=count,eye_local={k:v.tolist() for k,v in eye_by_robot.items()},target_source='same full-path native visible pad centroid as production camera updater',inside_AABB_candidates=possible_inside,arm_occlusions=failures)
(P/(os.environ.get('BEAM_CAMERA_AUDIT_LABEL','camera_v11_preflight')+'.json')).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(samples=count,inside_AABB_count=len(possible_inside),inside_first=possible_inside[:3],arm_occlusions=len(failures),occlusion_first=failures[:3])))
