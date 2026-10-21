"""Audit named saved paths without replacing older geometry evidence."""
from pathlib import Path
import json,sys,numpy as np
from sweep_model import SceneModel
from visual_mesh_check import VisualMeshChecker
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
output,*names=sys.argv[1:]
m=SceneModel(P/'runs/A08b_beam_full/scene_geometry.json');v=VisualMeshChecker(m,include_self=True)
counts={}
for name in names:
 count=0
 for i,line in enumerate((P/name).open()):
  r=json.loads(line);poses=m.poses(r['q']);phase=r.get('phase',r.get('segment'));intended=r.get('intended_grasp')
  for obj,B in r.get('part_poses',{}).items():poses[next(k for k in poses if k.endswith('/'+obj))]=np.asarray(B)
  if 'payload_pose' in r:
   poses[next(k for k in poses if k.endswith('/fabrica_beam_6'))]=np.asarray(r['payload_pose'])
   if phase not in ['base_6_open','base_6_world_settle','base_6_retreat','base_6_park']:intended=['franka_right','fabrica_beam_6']
  v.check(poses,phase,i,intended);count+=1
 counts[name]=count
report=v.report();bad=[p for p in report['pairs'] if p['max_depth']>1e-6]
badgrips=[p for p in report['intended_grasp_mesh_separations'] if p['signed_separation'] < -1e-6]
report.update(passed=not bad and not badgrips,input_sample_counts=counts,failures_above_precision=bad,
 grasp_failures_above_precision=badgrips,epsilon_m=1e-6,physics_claim=False,
 gaps=['nominal interpolation only; actual dynamic controller path requires live check',
       'unexported warehouse geometry','between-sample CCD','cooked/SDF versus visible CAD'])
(P/output).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(passed=report['passed'],samples=report['samples'],input_counts=counts,
 non_target_failure_pairs=len(bad),grasp_failures=len(badgrips),self_checked=True)))
raise SystemExit(0 if report['passed'] else 3)
