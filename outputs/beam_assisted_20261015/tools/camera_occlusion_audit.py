from pathlib import Path
import json,sys
import numpy as np
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1]
g=json.loads((D/'scene_geometry.json').read_text());spec=next(x for x in json.loads((D/'task_config_snapshot.json').read_text())['task_configs'][0]['camera_metadata'] if x['name']=='right_wrist')
body=g['robots']['franka_right']['root_path']+'/wrist_3_link';frames={400,440,460,479,520,646};clocks={}
for line in (D/'video_frame_clock.jsonl').open():
 r=json.loads(line)
 if r['frame_index'] in frames and 'right' in r['camera']:clocks[r['task_step_label']]=r
steps=set(clocks);rows=[]
for line in (D/'entity_geometry_trace.jsonl').open():
 r=json.loads(line)
 if r['step'] not in steps:continue
 poses={k:np.asarray(v,float) for k,v in r['body_poses'].items()};T=poses[body];eye=T[:3,:3]@np.asarray(spec['translation'])+T[:3,3];candidates=[]
 for m in g['visible_meshes']:
  B=poses.get(m['body'],np.asarray(m['body_rest']));p=(eye-B[:3,3])@B[:3,:3];v=np.asarray(m['points'],float)
  if np.any(p<v.min(0)) or np.any(p>v.max(0)):continue
  t=v[np.asarray(m['triangles'],int)]-p;a,b,c=t[:,0],t[:,1],t[:,2];na=np.linalg.norm(a,axis=1);nb=np.linalg.norm(b,axis=1);nc=np.linalg.norm(c,axis=1)
  angle=2*np.arctan2(np.einsum('ij,ij->i',a,np.cross(b,c)),na*nb*nc+np.einsum('ij,ij->i',a,b)*nc+np.einsum('ij,ij->i',b,c)*na+np.einsum('ij,ij->i',c,a)*nb)
  winding=float(angle.sum()/(4*np.pi));candidates.append(dict(mesh=m['prim'],body=m['body'],signed_winding=winding))
 rows.append(dict(step=r['step'],phase=r['phase'],image_clock=clocks[r['step']],eye=eye.tolist(),AABB_candidates=candidates))
(P/(sys.argv[1]+'_camera_eye_audit_corrected_clock.json')).write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows))
