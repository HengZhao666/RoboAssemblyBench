from pathlib import Path
import json,sys,collections
import numpy as np
from sweep_model import SceneModel,pose_matrix
D=Path(sys.argv[1]);model=SceneModel(D/'scene_geometry.json')
by_cycle=collections.defaultdict(list)
for line in (D/'physics_boundary_trace.jsonl').open():
 r=json.loads(line);bodies={b['path']:pose_matrix(b['positions'][0],b['orientations'][0]) for b in r['bodies'] if b.get('valid') and b.get('path')}
 if not all(x.get('valid') for x in r['robots'].values()):continue
 by_cycle[int(r['cycle_step'])].append({'clock':r['physics_step_index'],'boundary':r['boundary'],
   'bodies':bodies,'q':{n:x['joint_positions'][0] for n,x in r['robots'].items()}})
rows=[];unmatched=[]
for i,line in enumerate((D/'entity_geometry_trace.jsonl').open()):
 r=json.loads(line)
 if i%10 and int(r['step'])<2940:continue
 actual={k:np.asarray(v,float) for k,v in r['body_poses'].items()};options=[]
 for offset in [0,-1,1,-8,8]:
  for n in by_cycle.get(int(r['step'])+offset,[]):
   keys=set(actual)&set(n['bodies']);robot_keys=[k for k in keys if '/robots/' in k]
   if not robot_keys:continue
   error=max(float(np.max(abs(actual[k]-n['bodies'][k]))) for k in robot_keys)
   options.append((error,n))
 if not options:unmatched.append({'step':r['step'],'reason':'no_native_candidates'});continue
 error,n=min(options,key=lambda x:x[0])
 if error>1e-5:unmatched.append({'step':r['step'],'native_body_match_error':error});continue
 predicted=model.poses(n['q']);robot_keys=[k for k in actual if '/robots/' in k]
 matrix=max(float(np.max(abs(predicted[k]-actual[k]))) for k in robot_keys)
 position=max(float(np.linalg.norm(predicted[k][:3,3]-actual[k][:3,3])) for k in robot_keys)
 rows.append({'step':r['step'],'native_clock':n['clock'],'native_boundary':n['boundary'],
   'native_body_match_error':error,'full_robot_matrix_error':matrix,'full_robot_position_error_m':position})
report={'scope':'full32 robot rigid poses from live entity stream matched by native full-path body poses to the same physics boundary; calibrated full USD joint graph includes all gripper joints',
 'initial_model':model.initial_validation(),'rows':rows,'unmatched':unmatched,
 'max_full_robot_position_error_m':max((r['full_robot_position_error_m'] for r in rows),default=None),
 'max_full_robot_matrix_error':max((r['full_robot_matrix_error'] for r in rows),default=None)}
(D/'live_model_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ['max_full_robot_position_error_m','max_full_robot_matrix_error','unmatched']}))
