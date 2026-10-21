"""Nonadjacent visible robot-body sweeps; mechanical joint pairs listed explicitly."""
from pathlib import Path
import json,itertools,os
import numpy as np
import fcl
from sweep_model import SceneModel
from visual_mesh_check import VisualMeshChecker
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs/A04_base_place_release'
m=SceneModel(D/'scene_geometry.json');v=VisualMeshChecker(m)
adjacent={frozenset([j['parent'][0],j['child'][0]]):j['path'] for j in m.data['joints'] if j['robot'] and j['enabled'] and len(j['parent'])==len(j['child'])==1}
pairs=[]
for i,j in itertools.combinations(range(len(v.meshes)),2):
 a,b=v.meshes[i],v.meshes[j]
 if not a['robot'] or a['robot']!=b['robot'] or a['body']==b['body']:continue
 if frozenset([a['body'],b['body']]) in adjacent:continue
 pairs.append((i,j))
hits={};minimum=None;count=0
label=os.environ.get('BEAM_PREFLIGHT_LABEL','all_v7_with_preshape')
for line in (P/f'nominal_remaining_{label}_path.jsonl').open():
 r=json.loads(line);poses=m.poses(r['q']);bounds=[];count+=1
 for mesh,obj,corners in zip(v.meshes,v.shapes,v.corners):
  T=poses[mesh['body']];obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]));world=corners@T[:3,:3].T+T[:3,3];bounds.append((world.min(0),world.max(0)))
 for i,j in pairs:
  a,b=v.meshes[i],v.meshes[j];la,ha=bounds[i];lb,hb=bounds[j]
  lower=float(np.linalg.norm(np.maximum(np.maximum(la-hb,lb-ha),0)))
  if lower>.003:continue
  result=fcl.CollisionResult();hit=fcl.collide(v.shapes[i],v.shapes[j],fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
  if hit:
   key=a['prim']+' | '+b['prim'];depth=max((float(c.penetration_depth) for c in result.contacts),default=0.)
   if key not in hits:hits[key]={'mesh_a':a['prim'],'mesh_b':b['prim'],'body_a':a['body'],'body_b':b['body'],'count':0,'first_phase':r['phase'],'first_step':count-1,'max_contact_depth':0.,'first_contacts':[{'position':c.pos.tolist(),'depth':float(c.penetration_depth)} for c in result.contacts]}
   hits[key]['count']+=1;hits[key]['max_contact_depth']=max(hits[key]['max_contact_depth'],depth)
  else:
   distance=float(fcl.distance(v.shapes[i],v.shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
   if minimum is None or distance<minimum['distance']:minimum={'distance':distance,'phase':r['phase'],'mesh_a':a['prim'],'mesh_b':b['prim']}
report={'scope':'all exported visible meshes of each robot, nonadjacent rigid bodies; direct authored mechanical-joint body pairs explicitly listed as manufactured connection interfaces, no other same-robot exemption',
 'samples':count,'self_mesh_pairs':len(pairs),'mechanical_joint_interfaces':[{'bodies':list(k),'joint':p} for k,p in adjacent.items()],
 'intersection_pairs':list(hits.values()),'closest_separated_pair':minimum,'native_following_still_requires_actual_recheck':True}
(P/f'self_nominal_mesh_audit_{label}.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'samples':count,'pairs':[{'a':p['body_a'],'b':p['body_b'],'count':p['count'],'phase':p['first_phase']} for p in hits.values()]}))
