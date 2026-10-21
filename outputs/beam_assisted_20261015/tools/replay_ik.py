from pathlib import Path
import sys,json,time
import numpy as np
from scipy.spatial.transform import Rotation
from sweep_model import SceneModel,pose_matrix
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
sys.path.insert(0,str(P/'drafts/v6'))
sys.path.insert(0,'/home/user/isaacsim-5.1.0/exts/isaacsim.robot_motion.lula/pip_prebundle')
import lula
from beam_assisted_ik import bounded_local_ik
D=P/'runs/A05_base_place_release';m=SceneModel(D/'scene_geometry.json')
root=m.data['robots']['franka_right']['root_path'];base=m.rest[root+'/base_link'];hand=root+'/Gripper/Robotiq_2F_85/base_link'
cfgroot=Path('/home/user/isaacsim-5.1.0/exts/isaacsim.robot_motion.motion_generation/motion_policy_configs/universal_robots/ur5e')
desc=lula.load_robot(str(cfgroot/'rmpflow/ur5e_robot_description.yaml'),str(cfgroot/'ur5e.urdf'));kin=desc.kinematics()
def fk(q):
 T=kin.pose(np.asarray(q)[:,None],'tool0')
 return base[:3,:3]@np.asarray(T.translation)+base[:3,3],base[:3,:3]@T.rotation.matrix()
rows=[]
source=[json.loads(l) for l in (D/'motion_trace.jsonl').open() if json.loads(l)['phase']=='base_6_lift']
for source_index,r in enumerate(source):
 if source_index%max(1,len(source)//40):continue
 q=np.asarray(r['current_q']);p,R=fk(q)
 frame_error=float(np.linalg.norm(p-np.asarray(r['current_pose']['position'])))
 target=p+np.array([0,0,.0002]);target_R=R
 quat=Rotation.from_matrix(target_R).as_quat();wxyz=quat[[3,0,1,2]]
 config=lula.CyclicCoordDescentIkConfig();config.ccd_max_iterations=150;config.position_tolerance=1e-5
 config.orientation_tolerance=float(np.linalg.norm([1-np.cos(.0001),np.sin(.0001)]));config.cspace_seeds=[q]
 local_p=base[:3,:3].T@(target-base[:3,3]);local_R=base[:3,:3].T@target_R
 old=lula.compute_ik_ccd(kin,lula.Pose3(lula.Rotation3(local_R),local_p),'tool0',config)
 old_q=np.asarray(old.cspace_position).reshape(-1);op,oR=fk(old_q)
 start=time.perf_counter();new,detail=bounded_local_ik(fk,q,target,wxyz,position_tolerance=1e-5,orientation_tolerance=1e-4);seconds=time.perf_counter()-start
 if new is None:np_error=nr_error=None
 else:
  np_,nR=fk(new);np_error=float(np.linalg.norm(np_-target));nr_error=float(Rotation.from_matrix(target_R@nR.T).magnitude())
 rows.append({'step':r['step'],'native_FK_frame_position_error':frame_error,
  'old_success':bool(old.success),'old_q_delta':float(np.max(abs(old_q-q))),
  'old_position_error':float(np.linalg.norm(op-target)),'old_orientation_error':float(Rotation.from_matrix(target_R@oR.T).magnitude()),
  'new_converged':new is not None,'new_position_error':np_error,'new_orientation_error':nr_error,
  'new_q_delta':None if new is None else float(np.max(abs(new-q))),'new_seconds':seconds,'new_detail':detail})
assert max(r['native_FK_frame_position_error'] for r in rows)<2e-6
assert all(r['new_converged'] and r['new_q_delta']<.004 for r in rows)
report={'source':'A05 actual measured q/current pose; same raw Lula URDF/base frame; local FK evaluations, no physics scene writes','rows':rows,
 'samples':len(rows),'old_successes':sum(r['old_success'] for r in rows),'new_successes':sum(r['new_converged'] for r in rows),
 'max_frame_error':max(r['native_FK_frame_position_error'] for r in rows),'max_new_position_error':max(r['new_position_error'] for r in rows),
 'max_new_orientation_error':max(r['new_orientation_error'] for r in rows),'max_new_q_delta':max(r['new_q_delta'] for r in rows),'mean_new_seconds':sum(r['new_seconds'] for r in rows)/len(rows)}
(P/'A05_inverse_replay.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
