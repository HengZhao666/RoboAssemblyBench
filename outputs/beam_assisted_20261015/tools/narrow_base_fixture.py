from pathlib import Path
import json,sys
import numpy as np
from sweep_model import SceneModel
import fcl
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
run_name=sys.argv[sys.argv.index('--run')+1] if '--run' in sys.argv else 'A03_base_grasp'
D=P/'runs'/run_name;m=SceneModel(D/'scene_geometry.json')
shapes={}
for object_name in ['fabrica_fixture','fabrica_beam_6']:
 items=[]
 for mesh in m.data['visible_meshes']:
  if mesh['object']!=object_name:continue
  points=np.asarray(mesh['points'],float);triangles=np.asarray(mesh['triangles'],np.int32)
  shape=fcl.BVHModel();shape.beginModel(len(triangles),len(points));shape.addSubModel(points,triangles);shape.endModel()
  items.append((mesh,fcl.CollisionObject(shape)))
 shapes[object_name]=items
assert all(shapes.values())
actual='--actual' in sys.argv
stream=(D/'entity_geometry_trace.jsonl') if actual else (P/'base_nominal_path.jsonl')
hits=[];checked=0;minimum=None
for line in stream.open():
 r=json.loads(line)
 if actual:
  poses={**m.rest,**{k:np.asarray(v) for k,v in r['body_poses'].items()}}
  payload=poses[next(c.body for c in m.checker.colliders if c.entity=='fabrica_beam_6')];r['segment']=r['phase']
 else:
  poses=m.poses(r['q']);payload=np.asarray(r['payload_pose'])
 checked+=1
 for a,oa in shapes['fabrica_fixture']:
  A=poses[a['body']];oa.setTransform(fcl.Transform(A[:3,:3],A[:3,3]))
  for b,ob in shapes['fabrica_beam_6']:
   ob.setTransform(fcl.Transform(payload[:3,:3],payload[:3,3]));result=fcl.CollisionResult()
   count=fcl.collide(oa,ob,fcl.CollisionRequest(num_max_contacts=8,enable_contact=True),result)
   if count:
    hits.append({'sample':checked-1,'segment':r['segment'],'fixture':a['prim'],'payload':b['prim'],'contacts':[{'position':c.pos.tolist(),'depth':float(c.penetration_depth)} for c in result.contacts]})
   else:
    d=float(fcl.distance(oa,ob,fcl.DistanceRequest(),fcl.DistanceResult()))
    if minimum is None or d<minimum['distance']:minimum={'sample':checked-1,'segment':r['segment'],'distance':d}
report={'geometry':'complete visible authored triangle meshes; no convex fill of fixture holes','samples':checked,'surface_intersection_candidates':hits,'closest_separated_pair':minimum,'fixture_meshes':len(shapes['fabrica_fixture']),'part_meshes':len(shapes['fabrica_beam_6']),'scope':'fixture↔base6 only; all other entity pairs remain in full convex sweep; same-robot topology and unexported warehouse remain gaps'}
(D/'base_fixture_actual_triangle_audit.json' if actual else P/'base_fixture_triangle_narrowphase.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'samples':checked,'intersection_samples':len(hits),'first_intersection':hits[0] if hits else None,'closest_separated_pair':minimum}))
