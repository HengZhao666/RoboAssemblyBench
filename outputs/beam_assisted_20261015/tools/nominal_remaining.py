"""Dense nominal assisted motions; not a prediction of contact refinement."""
from pathlib import Path
import json,sys,os
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from sweep_model import SceneModel,pose_matrix
from visual_mesh_check import VisualMeshChecker
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs/A04_base_place_release'
m=SceneModel(D/'scene_geometry.json');checker=VisualMeshChecker(m)
config=json.loads((D/'task_config_snapshot.json').read_text())['task_configs'][0]
actual_config_path=os.environ.get('BEAM_PREFLIGHT_CONFIG')
if actual_config_path:
 config=json.loads(Path(actual_config_path).read_text())
 if 'task_configs' in config:config=config['task_configs'][0]
targets=config['target_poses'];phases=config['phase_specs']
park_scale=float(os.environ.get('BEAM_ASSISTED_LEFT_PARK_SCALE','1'))
for phase in phases:
 if not actual_config_path and phase['name'].endswith('_park') and phase.get('local_skill',{}).get('robot')=='franka_left':
  phase['local_skill']['offset']=[float(v)*park_scale for v in phase['local_skill']['offset']]
visual_rise=float(os.environ.get('BEAM_ASSISTED_VISUAL_RISE_M','0'))
for name,t in targets.items():
 if '_insert_' in name or name.endswith(('_assembled','_assembly_hover','_assembly_clearance','_preinsert_hover','_preseat')):t['position'][2]+=visual_rise

capture_height=float(os.environ.get('BEAM_ASSISTED_INSTALL_CAPTURE_M','0'))
for phase in phases:
 s=phase.get('local_skill',{});n=s.get('target_object_target','')
 if n.endswith('_assembled') and capture_height and not actual_config_path:
  t=json.loads(json.dumps(targets[n]));t['position'][2]+=capture_height;targets[n+'_assisted_capture']=t;s['target_object_target']=n+'_assisted_capture'
base_path=[json.loads(l) for l in (P/(os.environ.get('BEAM_BASE_PREFLIGHT_LABEL','base_nominal')+'_path.jsonl')).open()]
q={n:np.asarray(v,float) for n,v in base_path[-1]['q'].items()}
parts={name:pose_matrix(t['position'],t['orientation']) for name,t in targets.items() if name.endswith('_assembled')}
placed={'fabrica_beam_6':parts['part_6_assembled']}
object_body={c.entity:c.body for c in m.checker.colliders if c.entity.startswith('fabrica_beam_')}
robot='franka_left';hand=m.data['robots'][robot]['root_path']+'/Gripper/Robotiq_2F_85/base_link'
segments=[];rows=[];held=None;relative=None
def poses():
 p=m.poses(q)
 for name,B in placed.items():p[object_body[name]]=B
 if held:p[object_body[held]]=p[hand]@relative
 return p
def record(phase):
 p=poses();checker.check(p,phase,len(rows),(robot,held) if held else None)
 rows.append({'phase':phase,'q':{n:v.tolist() for n,v in q.items()},'part_poses':{n:p[b].tolist() for n,b in object_body.items()},'intended_grasp':[robot,held] if held else None})
def solve(H,reference):
 original=q[robot][:6].copy()
 def residual(x):
  q[robot][:6]=x;T=m.poses(q)[hand]
  return np.r_[(T[:3,3]-H[:3,3])*5,Rotation.from_matrix(H[:3,:3].T@T[:3,:3]).as_rotvec()]
 s=least_squares(residual,reference,max_nfev=100,xtol=1e-10,ftol=1e-10,gtol=1e-10)
 error=residual(s.x);q[robot][:6]=original
 if np.linalg.norm(error[:3])/5>.0001 or np.linalg.norm(error[3:])>.001:raise RuntimeError('IK endpoint error '+str(error))
 result=reference+(s.x-reference+np.pi)%(2*np.pi)-np.pi
 if np.max(abs(result-reference))>.12:raise RuntimeError('nominal IK branch step >.12rad')
 return result
def move(H,phase):
 start=m.poses(q)[hand].copy();dist=np.linalg.norm(H[:3,3]-start[:3,3]);angle=Rotation.from_matrix(start[:3,:3].T@H[:3,:3]).magnitude()
 count=max(2,int(np.ceil(max(dist/.002,angle/.01)))+1);slerp=Slerp([0,1],Rotation.from_matrix([start[:3,:3],H[:3,:3]]))
 first=len(rows)
 for f in np.linspace(0,1,count):
  T=np.eye(4);T[:3,3]=(1-f)*start[:3,3]+f*H[:3,3];T[:3,:3]=slerp(f).as_matrix()
  q[robot][:6]=solve(T,q[robot][:6]);record(phase)
 segments.append({'phase':phase,'samples':len(rows)-first,'max_cartesian_sampling_m':.002,'max_orientation_sampling_rad':.01})
def fingers(value,phase):
 start=q[robot][6:].copy();end=np.array([value,value,-value,value,-value,-value]);count=max(2,int(np.ceil(np.max(abs(end-start))/.008))+1)
 for f in np.linspace(0,1,count):q[robot][6:]=(1-f)*start+f*end;record(phase)

calibration={r['part']:r for r in json.loads((P/os.environ.get('BEAM_PINCH_CALIBRATION','visual_pinch_calibration.json')).read_text())['results']} if os.environ.get('BEAM_ASSISTED_VISUAL_PINCH')=='1' else {}
failure=None
try:
 for pid in (sys.argv[1:] or ['3']):
  obj='fabrica_beam_'+pid;B=poses()[object_body[obj]]
  for phase in [p for p in phases if '_part_'+pid+'_' in p['name']]:
   name=phase['name'];s=phase.get('local_skill',{})
   if name.endswith('_move_above') and s.get('prealign_joint_positions'):
    start=q[robot][:6].copy();goal=np.asarray(s['prealign_joint_positions']);count=max(2,int(np.ceil(np.max(abs(goal-start))/.01))+1)
    for f in np.linspace(0,1,count):q[robot][:6]=(1-f)*start+f*goal;record(name+':prealign')
   if name.endswith('_preshape'):
    fingers(.8*(1-float(s.get('gripper_openness',1.))),name);continue
   if name.endswith('_close_and_attach'):
    # Audited nominal linkage only; a future actual dual-contact grasp may
    # differ. All opening/closing linkage states are swept here.
    held=obj;relative=np.linalg.inv(m.poses(q)[hand])@B
    fingers(calibration[obj]['driver_q'] if obj in calibration else .74 if pid in ['3','2'] else .71,name);continue
   if name.endswith('_release_and_lock'):
    start=poses()[object_body[obj]].copy();placed[obj]=start;held=None;relative=None;fingers(0.,name)
    final=parts[f'part_{pid}_assembled']
    for f in np.linspace(0,1,max(2,int(np.ceil(capture_height/.0001))+1)):
     t=start.copy();t[:3,3]=(1-f)*start[:3,3]+f*final[:3,3];placed[obj]=t;record(name+':world_settle')
    placed[obj]=final;continue
   H=m.poses(q)[hand].copy()
   if s.get('relative_to_current_tcp'):
    offset=np.asarray(s.get('offset',[0,0,0]),float)
    if s.get('offset_frame') in ['tcp','eef','target','gripper']:offset=H[:3,:3]@offset
    H[:3,3]+=offset
   elif s.get('grasp_relative_position') is not None:
    B=poses()[object_body[obj]]
    H=B@np.linalg.inv(pose_matrix(s['grasp_relative_position'],s['grasp_relative_orientation']))
    H[:3,3]+=B[:3,:3]@np.asarray(s.get('grasp_object_frame_offset',[0,0,0]))
    offset=np.asarray(s.get('offset',[0,0,0]),float)
    if s.get('offset_frame') in ['object','local','part']:offset=B[:3,:3]@offset
    elif s.get('offset_frame') in ['target','eef','tcp','gripper']:offset=H[:3,:3]@offset
    H[:3,3]+=offset
   elif s.get('target_object_target'):
    t=targets[s['target_object_target']];goal=pose_matrix(t['position'],t['orientation'])
    if actual_config_path and s.get('target_object_offset'):
     if s.get('target_object_offset_frame')!='world':raise RuntimeError('unmodeled runtime target offset frame')
     goal[:3,3]+=np.asarray(s['target_object_offset'],float)
    H=goal@np.linalg.inv(relative)
   else:raise RuntimeError('unmodeled nominal skill '+name)
   try:move(H,name)
   except RuntimeError as e:
    actual=m.poses(q)[hand];dp=float(np.linalg.norm(actual[:3,3]-H[:3,3]));dr=float(Rotation.from_matrix(H[:3,:3]@actual[:3,:3].T).magnitude())
    if name.endswith('_park') and dp<float(s.get('position_tolerance',.018)) and dr<float(s.get('orientation_tolerance',.12)):
     segments.append({'phase':name,'exact_endpoint_error':str(e),'actual_position_error_m':dp,'actual_orientation_error_rad':dr,'existing_runtime_stage_gate_reached':True})
    else:raise
except Exception as e:failure={'phase':name,'error':str(e),'sample':len(rows)}
report=checker.report();report.update(assembly_visual_rise_m=visual_rise,failure=failure,segments=segments,parts=sys.argv[1:] or ['3'],scope_note='nominal Cartesian interpolation at exact audited grasps; initial right park is nominal base path, actual end pose/grasp refinement still requires native recheck; full gripper linkage covered')
label=os.environ.get('BEAM_PREFLIGHT_LABEL','_'.join(sys.argv[1:] or ['3']));(P/f'nominal_remaining_{label}.json').write_text(json.dumps(report,indent=2)+'\n')
with (P/f'nominal_remaining_{label}_path.jsonl').open('w') as f:
 for r in rows:f.write(json.dumps(r)+'\n')
print(json.dumps({'samples':len(rows),'failure':failure,'pairs':report['pairs'],'closest':report['closest_separated_pair']}))
