from pathlib import Path
import sys,json,importlib.util,os
import numpy as np
from scipy.spatial.transform import Rotation
from sweep_model import pose_matrix
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs/A04_base_place_release'
module_name='toolkits.factory_dual_franka_assembly.beam_assisted_visual_grasp'
spec=importlib.util.spec_from_file_location(module_name,P/'assisted_repository/toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py');module=importlib.util.module_from_spec(spec);sys.modules[module_name]=module;spec.loader.exec_module(module)
from toolkits.factory_dual_franka_assembly.beam_assisted_ik import bounded_local_ik
model=module.VisiblePinchGeometry(json.loads((D/'scene_geometry.json').read_text()))
captures={}
base_run=os.environ.get('BEAM_CALIBRATION_BASE_RUN',D.name)
e=next(json.loads(l) for l in (P/'runs'/base_run/'assisted_trace.jsonl').open() if json.loads(l)['event']=='attach_measured_pose')
captures['6']={'robot':'franka_right','q':e['full_joint_positions'],'part_pose':pose_matrix(e['position'],e['orientation']).tolist()}
for line in (P/'nominal_remaining_3_2_1_0_path.jsonl').open():
 r=json.loads(line)
 if r['phase'].endswith('_close_and_attach'):
  pid=r['phase'].split('_part_')[1].split('_')[0]
  captures[pid]={'robot':'franka_left','q':r['q']['franka_left'],'part_pose':r['part_poses']['fabrica_beam_'+pid]}
results=[]
for pid,capture in captures.items():
 robot=capture['robot'];part='fabrica_beam_'+pid;q=np.asarray(capture['q'],float);B=np.asarray(capture['part_pose'],float)
 root=model.data['robots'][robot]['root_path'];hand=root+'/Gripper/Robotiq_2F_85/base_link'
 start=model.poses(robot,q)[hand].copy();shifts=[]
 for attempt in range(13):
  goal,gaps=model.goal(robot,part,q,B)
  if max(x['gap'] for x in gaps)<=.00025:break
  if attempt==12:raise RuntimeError('calibration centering not converged '+pid+' '+str(gaps))
  poses=model.poses(robot,q);shift=model.center_shift(robot,part,q,B,goal);H=poses[hand]
  def forward(x):
   full=q.copy();full[:6]=x;T=model.poses(robot,full)[hand];return T[:3,3],T[:3,:3]
  quat=Rotation.from_matrix(H[:3,:3]).as_quat()[[3,0,1,2]];new,detail=bounded_local_ik(forward,q[:6],H[:3,3]+shift,quat)
  if new is None:raise RuntimeError('calibration inverse failed '+pid)
  q[:6]=new;shifts.append(shift.tolist())
 driver=model.data['robots'][robot]['joint_names'].index('finger_joint');change=goal-q[driver]
 for name,sign in {'finger_joint':1,'right_outer_knuckle_joint':1,'left_inner_finger_joint':-1,'right_inner_finger_joint':1,'left_inner_finger_knuckle_joint':-1,'right_inner_finger_knuckle_joint':-1}.items():q[model.data['robots'][robot]['joint_names'].index(name)]+=sign*change
 poses=model.poses(robot,q);part_body=next(k for k in model.shapes if k.endswith('/'+part));poses[part_body]=B;gaps=model.gaps(robot,part,poses)
 assert all(not x['intersect'] and .000005<=x['gap']<=.00025 for x in gaps)
 results.append({'part':part,'robot':robot,'driver_q':goal,'full_q':q.tolist(),'part_pose':B.tolist(),'hand_pose':poses[hand].tolist(),'gaps':gaps,'centering_steps':shifts,'total_TCP_shift':(poses[hand][:3,3]-start[:3,3]).tolist()})
(P/'visual_pinch_calibration_v11.json').write_text(json.dumps({'scope':'base6 actual measured grasp pose; other4 nominal audited grasps; visible mesh surfaces, no physics writes; future native pinch still must qualify','results':results},indent=2)+'\n')
print(json.dumps([{k:v for k,v in r.items() if k not in ['full_q','part_pose','hand_pose']} for r in results]))
