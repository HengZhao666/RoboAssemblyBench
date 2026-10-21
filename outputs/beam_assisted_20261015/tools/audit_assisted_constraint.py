"""Independent native wrist/part joint-anchor error; no controller-FK inference."""
from pathlib import Path
import argparse,json,sys
import numpy as np
from scipy.spatial.transform import Rotation
from beam_workflow import sha,save

p=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('run');ap.add_argument('output',type=Path);ap.add_argument('--object',default='fabrica_beam_1',choices=['fabrica_beam_0','fabrica_beam_1','fabrica_beam_2','fabrica_beam_3']);ap.add_argument('--all-held-phases',action='store_true');args=ap.parse_args();d=p/'runs'/args.run
trace=[json.loads(l) for l in (d/'assisted_trace.jsonl').open()]
capture=next(e for e in trace if e.get('event')=='attach_measured_pose' and e.get('object')==args.object)
anchor=next(e for e in trace if e.get('event')=='joint_anchor_readback' and e['joint_path']==capture['joint_path'])
assert anchor['local_position0']==[0.,0.,0.]
release_step=min([e['step'] for e in trace if e.get('event')=='support_measured_pose' and e.get('object')==args.object and e['step']>capture['step']]+[2**31-1])
hand=anchor['body1'][0];part=anchor['body0'][0]
def rot(q):return Rotation.from_quat(np.asarray(q)[[1,2,3,0]])
lp=np.array(anchor['local_position1']);lr=rot(anchor['local_orientation1']);rows=[];missing=[]
with (d/'physics_boundary_trace.jsonl').open() as stream:
 for line in stream:
  r=json.loads(line);phase=r.get('cycle_phase',r.get('phase',''))
  if args.all_held_phases:
   if not capture['step']<=r['cycle_step']<release_step:continue
  elif not any(s in phase for s in ['part_'+args.object.rsplit('_',1)[1]+'_pickup_clearance','part_'+args.object.rsplit('_',1)[1]+'_assembly_clearance']):continue
  bodies={b['path']:b for b in r['bodies']}
  if hand not in bodies or part not in bodies:
   missing.append(dict(task_step=r['cycle_step'],boundary=r['boundary'],index=r['physics_step_index']));continue
  h=bodies[hand];b=bodies[part]
  assert h['valid'] and b['valid']
  hr=rot(h['orientations'][0]);br=rot(b['orientations'][0]);expected=np.array(h['positions'][0])+hr.apply(lp)
  delta=np.array(b['positions'][0])-expected
  angle=float((br*(hr*lr).inv()).magnitude())
  rows.append(dict(phase=phase,task_step=r['cycle_step'],boundary=r['boundary'],physics_index=r['physics_step_index'],physics_time=r['physics_time'],
      position_error_m=float(np.linalg.norm(delta)),position_error_world_m=delta.tolist(),angle_error_rad=angle))
assert rows
summary={}
for phase in sorted({r['phase'] for r in rows}):
 rr=[r for r in rows if r['phase']==phase]
 summary[phase]=dict(samples=len(rr),first=rr[0],last=rr[-1],max_position=max(rr,key=lambda r:r['position_error_m']),max_angle=max(rr,key=lambda r:r['angle_error_rad']))
checks=[]
for e in trace:
 if e.get('event')!='object_model_residual_rejected':continue
 rr=[r for r in rows if r['physics_index']==e['physics_stamp']['index'] and abs(r['physics_time']-e['physics_stamp']['time'])<1e-9]
 checks.append(dict(guard_task_step=e['step'],guard_position_m=e['position_residual_m'],guard_angle_rad=e['orientation_residual_rad'],same_native_clock_rows=rr,
     scope='Joined by exact native index/time; boundary and its original cycle label retained, never renamed as the missing failure-cycle hook.'))
model_checks=[]
for e in trace:
 if e.get('event')!='absolute_object_servo' or e.get('object')!=args.object or 'fk_native_orientation_residual_rad' not in e:continue
 stamp=e.get('physics_stamp') or {}
 rr=[r for r in rows if r['physics_index']==stamp.get('index') and abs(r['physics_time']-stamp.get('time',float('inf')))<1e-9]
 if rr:model_checks.append(dict(event_step=e['step'],physics_stamp=stamp,model_position_m=e['fk_native_position_residual_m'],model_angle_rad=e['fk_native_orientation_residual_rad'],same_native_clock_rows=rr))
report=dict(run=args.run,object=args.object,analysis_complete=True,native_physics_started=False,solver_changed=False,model_relation_changed=False,
 scope='Original float32-authored fixed joint local anchor composed with independently recorded native wrist actor pose, compared to native part actor pose. Rot0 identity is authored by the verified production constructor and was not independently read back in the original event. No FK model or positional-write simulation used.',
 all_held_phases=args.all_held_phases,release_world_support_step_excluded=release_step,anchor=anchor,capture_step=capture['step'],samples=len(rows),missing_native_hand_samples=missing,phase_summary=summary,guard_clock_comparisons=checks,model_same_clock_comparisons=model_checks,
 source_sha256={n:sha(d/n) for n in ['assisted_trace.jsonl','physics_boundary_trace.jsonl','route_source_manifest.json']},tool_sha256=sha(__file__))
assert not args.output.exists();save(args.output,report)
print(json.dumps(dict(run=args.run,samples=len(rows),missing=len(missing),phases={k:dict(position_um=v['max_position']['position_error_m']*1e6,angle_mrad=v['max_angle']['angle_error_rad']*1e3) for k,v in summary.items()},guard_matches=[len(x['same_native_clock_rows']) for x in checks])))
