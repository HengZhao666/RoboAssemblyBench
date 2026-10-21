"""Reclassify an actual native admission stream and compare the legacy default."""
from pathlib import Path
import argparse,ast,copy,json,math,sys
import numpy as np

ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('run');ap.add_argument('phase');args=ap.parse_args()
w=args.work.resolve();p=w.parent;c=w/'candidate';a=p/'assisted_repository';d=p/'runs'/args.run
sys.path[:0]=[str(c),str(p/'tools')]
from beam_workflow import sha,save
from toolkits.factory_dual_franka_assembly import beam_assisted_servo as servo
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
read=lambda f:json.loads(Path(f).read_text())
source={n:sha(c/n) for n in read(w/'baseline_manifest.json')}
key='toolkits/factory_dual_franka_assembly/beam_assisted_servo.py'
assert sha(a/key)==read(w/'baseline_manifest.json')[key]
oldtree=ast.parse((a/key).read_text());newtree=ast.parse((c/key).read_text())
fn=lambda tree,name:next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
protected={name:ast.dump(fn(oldtree,name),include_attributes=False)==ast.dump(fn(newtree,name),include_attributes=False)
 for name in ['_consecutive_native_com_rate','settled_transition_ready','validate_model_native_residual','absolute_object_servo','published_object_command_safe','anchored_object_prediction']}
assert all(protected.values())
namespace=dict(vars(servo));exec(compile(ast.fix_missing_locations(ast.Module(body=[fn(oldtree,'descent_admission')],type_ignores=[])),str(a/key),'exec'),namespace);old=namespace['descent_admission']
cfg=read(d/'task_config_snapshot.json')['task_configs'][0];phase=next(v for v in cfg['phase_specs'] if v['name']==args.phase);spec=phase['local_skill'];contract=spec['beam_assisted_descent_admission'];new_contract=dict(contract,motion_metric='native_com_pose_difference')
goal=Adapter({})._target_object_pose(task=type('Task',(),{'target_poses':cfg['target_poses']})(),spec=spec)
epoch=read(d/'startup_images_ready.json')['physics_stamp']['epoch'];axis=spec.get('target_object_convergence_axis',[0,0,-1]);samples={};missing=[];last_index=-1
with (d/'physics_boundary_trace.jsonl').open() as stream:
 for line in stream:
  r=json.loads(line)
  if r.get('cycle_phase',r.get('phase'))!=args.phase:continue
  assert r['clock_valid'] and r['physics_step_index']>=last_index;last_index=r['physics_step_index']
  b=next((v for v in r['bodies'] if v['path'].endswith('/'+spec['object'])),None);com=(b or {}).get('center_of_mass',{})
  if not(b and b['valid'] and b['physics_handle_valid'] and com.get('valid')):missing.append([r['cycle_step'],r['boundary']]);continue
  stamp=dict(valid=True,epoch=epoch,index=r['physics_step_index'],time=r['physics_time'],dt=r['physics_dt'])
  motion=dict(motion_valid=True,physics_handle_valid=True,body_path=b['path'],motion_view_id=b['view_id'],
      position=b['positions'][0],orientation=b['orientations'][0],linear_velocity=b['linear_velocities'][0],
      linear_speed=float(np.linalg.norm(b['linear_velocities'][0])),angular_speed=float(np.linalg.norm(b['angular_velocities'][0])),
      center_of_mass_valid=True,center_of_mass_position_frame=com['position_frame'],center_of_mass_orientation_order=com['orientation_order'],
      center_of_mass_position=np.asarray(com['world_positions'][0]),center_of_mass_orientation=np.asarray(com['world_orientations'][0]))
  native_key=(stamp['index'],stamp['time'])
  if native_key in samples:
   previous=samples[native_key]['motion']
   for name in ['position','orientation','center_of_mass_position']:
    assert np.array_equal(previous[name],motion[name]),(native_key,name)
   assert (np.allclose(previous['center_of_mass_orientation'],motion['center_of_mass_orientation'],rtol=0,atol=1e-12)
           or np.allclose(previous['center_of_mass_orientation'],-motion['center_of_mass_orientation'],rtol=0,atol=1e-12))
   assert previous['linear_speed']==motion['linear_speed'] and previous['angular_speed']==motion['angular_speed']
  else:samples[native_key]=dict(stamp=stamp,motion=motion,step=r['cycle_step'],boundary=r['boundary'])
assert samples and not missing
oldstate={};newstate={};decisions=[]
for native_key,item in sorted(samples.items()):
 motion=item['motion'];current={k:np.asarray(motion[k]) for k in ['position','orientation']}
 rejected,old_detail=old(oldstate,item['stamp'],current,goal,axis,contract,motion)
 ready,detail=servo.descent_admission(newstate,item['stamp'],current,goal,axis,new_contract,motion)
 decisions.append(dict(step=item['step'],stamp=item['stamp'],old_ready=rejected,new_ready=ready,
     old_detail=old_detail,new_detail=detail))
positive=[r for r in decisions if r['new_ready'] and not r['old_ready']]
assert positive,'Actual dense native stream did not qualify new gate; do not invent missing ticks'

def plain(value):
 if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
 if isinstance(value,(list,tuple)):return [plain(v) for v in value]
 if isinstance(value,np.ndarray):return value.tolist()
 if isinstance(value,np.generic):return value.item()
 return value

default_cases=0
for mode in ['quiet','warm_axial','fast','lost_pose','same_clock','clock_gap','invalid_native']:
 before={};after={}
 for tick in range(64):
  index=tick+(3 if mode=='clock_gap' and tick>31 else 0)
  if mode=='same_clock':index=tick//2
  stamp=dict(valid=mode!='invalid_native',epoch=0,index=index,time=index/240,dt=1/240)
  z=1.-max(0,tick-12)*.00015 if mode=='warm_axial' else 1.
  current=dict(position=np.array([.0002 if mode=='lost_pose' and tick>31 else 0.,0.,z]),orientation=np.array([1.,0,0,0]))
  motion=dict(motion_valid=True,physics_handle_valid=True,body_path='default_fixture',motion_view_id=1,
              linear_speed=.036 if mode=='warm_axial' and tick>12 else 0.,angular_speed=.21 if mode=='fast' else 0.,linear_velocity=[0,0,-.036])
  target=dict(position=[0.,0.,.9],orientation=[1.,0.,0.,0.])
  x=old(before,stamp,current,target,axis,contract,motion);y=servo.descent_admission(after,stamp,current,target,axis,contract,motion)
  assert plain(x)==plain(y) and plain(before)==plain(after),(mode,tick)
  default_cases+=1
target=w/'native_admission_replay.json';assert not target.exists()
save(target,dict(passed=True,native_started=False,source_manifest=source,actual_run=args.run,actual_phase=args.phase,
    native_samples=len(samples),missing_native_samples=missing,first_new_qualified_old_rejected=positive[0],
    new_qualified_old_rejected_samples=len(positive),legacy_default_exact_cases=default_cases,
    protected_function_AST_exact=protected,original_contract=contract,new_contract=new_contract,
    physics_epoch_source='Same single episode startup stamp; original boundary stream has no epoch field. Monotonic native index/time verified; no restoration.',
    decisions=decisions,tool_sha256=sha(__file__),raw_inputs={n:sha(d/n) for n in ['physics_boundary_trace.jsonl','task_config_snapshot.json','startup_images_ready.json','source_after.json']},
    limits=['Classification of actual old-control observations, including real dense consecutive ticks. It does not simulate the changed trajectory after admission.',
            'Missing native ticks are never interpolated or counted. Full from-head verification remains required.']))
print(json.dumps(dict(passed=True,native_samples=len(samples),first_new_qualified=positive[0]['stamp'],default_exact_cases=default_cases,full_dynamic_success=False)))
