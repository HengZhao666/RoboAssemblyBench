from pathlib import Path
import sys,json,itertools
import numpy as np
import fcl
from sweep_model import SceneModel
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1];m=SceneModel(D/'scene_geometry.json')
meshes=[x for x in m.data['visible_meshes'] if x['object']=='fabrica_beam_6' or (x['robot']=='franka_right' and x['body'].endswith(('/left_inner_finger','/right_inner_finger')))]
shapes=[]
for mesh in meshes:
 points=np.asarray(mesh['points'],float);triangles=np.asarray(mesh['triangles'],np.int32)
 s=fcl.BVHModel();s.beginModel(len(triangles),len(points));s.addSubModel(points,triangles);s.endModel();shapes.append(fcl.CollisionObject(s))
rows=[]
for line in (D/'entity_geometry_trace.jsonl').open():
 r=json.loads(line)
 if r['step']<1800 or not r['phase'].startswith('base_6_'):continue
 poses={**m.rest,**{k:np.asarray(v) for k,v in r['body_poses'].items()}}
 for mesh,obj in zip(meshes,shapes):
  T=poses[mesh['body']];obj.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
 for i,j in itertools.combinations(range(len(meshes)),2):
  a,b=meshes[i],meshes[j]
  if a['object']!='fabrica_beam_6' and b['object']!='fabrica_beam_6':continue
  result=fcl.CollisionResult();hit=fcl.collide(shapes[i],shapes[j],fcl.CollisionRequest(num_max_contacts=16,enable_contact=True),result)
  gap=-max((float(c.penetration_depth) for c in result.contacts),default=0.) if hit else float(fcl.distance(shapes[i],shapes[j],fcl.DistanceRequest(),fcl.DistanceResult()))
  rows.append({'step':r['step'],'phase':r['phase'],'mesh_a':a['prim'],'mesh_b':b['prim'],'hit':bool(hit),'signed_separation':gap,'contacts':[{'position':c.pos.tolist(),'depth':float(c.penetration_depth)} for c in result.contacts]})
report={'scope':'actual visible base6 and both complete inner-finger visual meshes; no grasp contact exemption; same entity sample native body poses','rows':rows,'samples':len(rows),'intersections':sum(r['hit'] for r in rows),'minimum_signed_separation':min((r['signed_separation'] for r in rows),default=None),'worst':min(rows,key=lambda r:r['signed_separation']) if rows else None}
(D/'grasp_visible_mesh_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ['rows','worst']}));print(json.dumps(report['worst']))
