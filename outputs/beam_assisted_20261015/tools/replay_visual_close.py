from pathlib import Path
import sys,json,importlib.util,types
import numpy as np
from scipy.spatial.transform import Rotation
from sweep_model import pose_matrix
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs/A06_base_place_release'
def load(label,path):
 name='toolkits.factory_dual_franka_assembly.visual_replay_'+label
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
old=load('old',P/'assisted_repository/toolkits/factory_dual_franka_assembly/beam_assisted_visual_grasp.py')
new=load('new',P/'drafts/v8/beam_assisted_visual_grasp.py')
observed=[json.loads(l) for l in (D/'assisted_trace.jsonl').open() if json.loads(l)['event']=='visual_pinch_observation']
rates=[]
for a,b in zip(observed,observed[1:]):
 if 1800<=a['step']<2800:
  rates.append((b['actual_driver_q']-a['actual_driver_q'])/max(1,b['step']-a['step'])/.001)
alpha=float(np.median(rates));assert .12<alpha<.2
data=json.loads((D/'scene_geometry.json').read_text());cal=json.loads((P/'visual_pinch_calibration.json').read_text())['results'][0]
qclosed=np.asarray(cal['full_q'],float);B=np.asarray(cal['part_pose']);Rquat=Rotation.from_matrix(B[:3,:3]).as_quat()[[3,0,1,2]]
root=data['robots']['franka_right']['root_path'];part='fabrica_beam_6';robot='franka_right';signs=np.array([1,1,-1,1,-1,-1])
def simulate(module):
 q=qclosed.copy();q[6:]+=signs*(-q[6]);model=module.VisiblePinchGeometry(data)
 task=types.SimpleNamespace(_assisted_geometry_snapshot=data,phase_index=3,phase_entry_step=1206,step_counter=0)
 task._assisted_event=lambda *a,**kw:None
 task._resolve_object=lambda name:types.SimpleNamespace(get_pose=lambda:(B[:3,3],Rquat))
 task.robots={robot:types.SimpleNamespace(articulation=types.SimpleNamespace(get_joint_positions=lambda:q.copy()))}
 task._beam_physics_stamp=lambda:{'valid':True,'epoch':0,'index':task.step_counter}
 task._assisted_native_body_readers={}
 for finger in ['left_inner_finger','right_inner_finger']:
  path=root+'/Gripper/Robotiq_2F_85/'+finger
  def get_pose(path=path):
   T=model.poses(robot,q)[path];Q=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]];return T[:3,3],Q
  task._assisted_native_body_readers[path]=types.SimpleNamespace(unwrap=lambda path=path:types.SimpleNamespace(prim_path=path),get_pose=get_pose)
 ready_at=None;minimum=np.inf
 for tick in range(2000):
  task.step_counter=tick
  action,ready,detail=module.close_action(task,None,robot,part,{'arm_joint_controller':[q[:6].tolist()]})
  minimum=min(minimum,min(x['gap'] for x in detail['gaps']))
  if ready:ready_at=tick;break
  command=.8*(1-action['gripper_controller'][0]);change=alpha*(command-q[6]);q[6:]+=signs*change
 return {'ready_at':ready_at,'driver_q':float(q[6]),'minimum_CAD_gap':minimum,'final_gaps':detail['gaps']}
r={'scope':'actual A06 no-contact actuator response fitted from trace; true visible CAD and full native joint graph replay, no contact dynamics or native completion claim','measured_response_fraction_per_tick':alpha,'old':simulate(old),'new':simulate(new)}
assert r['old']['ready_at'] is None
assert r['new']['ready_at'] is not None and r['new']['ready_at']<1200 and r['new']['minimum_CAD_gap']>0
(P/'A06_visual_close_replay.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
