"""Compare recorded native COM pose rates with raw solver velocity; no physics."""
from pathlib import Path
import argparse,json,math,sys
import numpy as np
from scipy.spatial.transform import Rotation
from beam_workflow import sha,save

p=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('run');ap.add_argument('phase');ap.add_argument('output',type=Path);ap.add_argument('--object',default='fabrica_beam_1');ap.add_argument('--last-native-ticks',type=int,default=256);ap.add_argument('--native-joint-limits-from');ap.add_argument('--through-step',type=int);ap.add_argument('--asset-identity-report',type=Path);args=ap.parse_args()
d=p/'runs'/args.run;assert not args.output.exists()
if args.native_joint_limits_from:
 other=p/'runs'/args.native_joint_limits_from
 assert args.through_step is not None and args.through_step>=0
 for run in (d,other):
  assert (run/'finished_at').exists()
  after=json.loads((run/'source_after.json').read_text());assert not after['differences'] and not after.get('tool_differences')
 configs=[json.loads((run/'task_config_snapshot.json').read_text())['task_configs'][0] for run in (d,other)]
 assert args.asset_identity_report and args.asset_identity_report.is_file()
 asset_identity=json.loads(args.asset_identity_report.read_text())
 geometry=[json.loads((run/'scene_geometry.json').read_text()) for run in (d,other)]
 arms={};assets=[]
 for old in configs[0]['robots']:
  new=next(r for r in configs[1]['robots'] if r['name']==old['name'])
  paths=[r['usd_path'] for r in (old,new)];relative=['roboassemblybench/'+s.split('/roboassemblybench/',1)[1] for s in paths]
  assert relative[0]==relative[1] and relative[0]=='roboassemblybench/'+asset_identity['asset'].split('/roboassemblybench/',1)[1]
  assert all(sha(path)==asset_identity['asset_sha256'] for path in paths)
  original_joints=[r for r in geometry[0]['joints'] if r.get('robot')==old['name']]
  current_joints=[r for r in geometry[1]['joints'] if r.get('robot')==old['name']]
  assert original_joints and original_joints==current_joints
  names=[next(c['joint_names'] for c in r['controllers'] if c['name']=='arm_joint_controller') for r in (old,new)]
  assert names[0]==names[1] and len(names[0])==6
  arms[old['name']]=names[0];assets.append(dict(robot=old['name'],relative_path=relative[0],main_USD_sha256=asset_identity['asset_sha256'],exported_robot_joints_exact=True))
 limits={};limit_records=[]
 for line in (other/'assisted_trace.jsonl').open():
  r=json.loads(line)
  if r.get('event')!='native_arm_joint_limits':continue
  name=r['robot'];assert r['source']=='same_existing_live_articulation_view.get_dof_limits' and r['joint_names']==arms[name]
  value=np.asarray(r['limits_rad'],dtype=np.float32);assert value.shape==(6,2) and np.all(np.isfinite(value)) and np.all(value[:,0]<value[:,1])
  if name in limits:assert np.array_equal(limits[name],value)
  limits[name]=value;limit_records.append(r)
 assert set(limits)==set(arms)
 counts={name:0 for name in arms};violations=[];maxima={name:np.full(6,-np.inf) for name in arms};minima={name:np.full(6,np.inf) for name in arms}
 for line in (d/'assisted_trace.jsonl').open():
  r=json.loads(line)
  if r.get('event')!='published_command' or r['step']>args.through_step:continue
  name=r['robot'];q=np.asarray(r['published_q'],dtype=np.float32);assert q.shape==(6,) and np.all(np.isfinite(q))
  counts[name]+=1;minima[name]=np.minimum(minima[name],q);maxima[name]=np.maximum(maxima[name],q)
  if np.any(q<limits[name][:,0]) or np.any(q>limits[name][:,1]):violations.append(dict(step=r['step'],phase=r['phase'],robot=name,published_q_float32=q.tolist()))
 assert all(counts.values())
 report=dict(passed=not violations,native_started=False,actual_published_prefix=True,run=args.run,through_step=args.through_step,native_limits_recorded_run=other.name,
  commands_checked=counts,violations=violations,live_native_limit_records=limit_records,unchanged_robot_assets=assets,
  command_ranges={n:dict(minimum=minima[n].tolist(),maximum=maxima[n].tolist()) for n in arms},
  input_sha256={**{str(run/name):sha(run/name) for run in (d,other) for name in ['assisted_trace.jsonl','source_after.json','task_config_snapshot.json','scene_geometry.json']},str(args.asset_identity_report):sha(args.asset_identity_report)},tool_sha256=sha(__file__),
  scope='Historical actual final publications cast to native float32 and checked against a later completed short run\'s actual read-only native limits. Main USD roots match the preserved asset record, actual exported robot joints and semantic joint mappings are equal. This does not assert a transitive USD asset seal or historical live limit readback. This checks the new limit guard\'s unchanged-prefix applicability; it does not replay physics or certify future trajectories.')
 save(args.output,report);print(json.dumps({k:report[k] for k in ['passed','commands_checked','through_step','violations','native_started']}));raise SystemExit(0 if report['passed'] else 1)
rows={};missing=[];duplicates=[];original_labels=set()
with (d/'physics_boundary_trace.jsonl').open() as stream:
 for line in stream:
  r=json.loads(line);phase=r.get('cycle_phase',r.get('phase',''))
  if phase!=args.phase:continue
  b=next((b for b in r['bodies'] if b['path'].endswith('/'+args.object)),None)
  com=(b or {}).get('center_of_mass') or {}
  if not(b and b['valid'] and com.get('valid') and com.get('position_frame')=='world' and com.get('orientation_order')=='wxyz'):
   missing.append(dict(step=r['cycle_step'],boundary=r['boundary'],index=r['physics_step_index']));continue
  epoch=r.get('physics_epoch',0);key=(epoch,r['physics_step_index'],r['physics_time'])
  item=dict(epoch=epoch,index=key[1],time=key[2],step=r['cycle_step'],boundary=r['boundary'],phase=phase,
      position=com['world_positions'][0],orientation=com['world_orientations'][0],
      raw_linear=float(np.linalg.norm(b['linear_velocities'][0])),raw_angular=float(np.linalg.norm(b['angular_velocities'][0])))
  if key in rows:
   old=rows[key];delta=float(np.linalg.norm(np.asarray(old['position'])-item['position']))
   if delta>1e-9:duplicates.append(dict(index=key[1],COM_position_difference_m=delta,boundaries=[old['boundary'],item['boundary']]))
  else:rows[key]=item
  original_labels.add((phase,r['boundary']))
assert rows
ordered=[rows[k] for k in sorted(rows)];cut=ordered[-1]['index']-args.last_native_ticks
selected=[r for r in ordered if r['index']>=cut];rates=[];chain=best=1
for before,after in zip(ordered,ordered[1:]):
 dt=after['time']-before['time'];ticks=after['index']-before['index']
 assert dt>0 and ticks>0 and before['epoch']==after['epoch']
 velocity=(np.asarray(after['position'])-before['position'])/dt
 angle=float((Rotation.from_quat(np.asarray(after['orientation'])[[1,2,3,0]])*Rotation.from_quat(np.asarray(before['orientation'])[[1,2,3,0]]).inv()).magnitude())
 if ticks==1:chain+=1
 else:chain=1
 best=max(best,chain)
 if after['index']>=cut:rates.append(dict(index=after['index'],native_tick_interval=ticks,interval_s=dt,linear_speed_m_s=float(np.linalg.norm(velocity)),angular_speed_rad_s=angle/dt))
adjacent=[r for r in rates if r['native_tick_interval']==1];assert adjacent
report=dict(run=args.run,phase=args.phase,analysis_complete=True,native_started=False,
    unique_native_samples=len(ordered),longest_consecutive_native_chain=best,late_window_samples=len(selected),late_one_tick_pairs=len(adjacent),
    missing_native_COM=missing,duplicate_clock_pose_differences=duplicates,original_phase_boundary_labels=[list(v) for v in sorted(original_labels)],
    late_raw_linear_range_m_s=[min(r['raw_linear'] for r in selected),max(r['raw_linear'] for r in selected)],
    late_raw_angular_range_rad_s=[min(r['raw_angular'] for r in selected),max(r['raw_angular'] for r in selected)],
    late_one_tick_COM_linear_max_m_s=max(r['linear_speed_m_s'] for r in adjacent),
    late_one_tick_COM_angular_max_rad_s=max(r['angular_speed_rad_s'] for r in adjacent),
    late_COM_displacement_extent_m=float(np.linalg.norm(np.ptp(np.asarray([r['position'] for r in selected]),axis=0))),
    observed_rates=rates,
    source_sha256={n:sha(d/n) for n in ['physics_boundary_trace.jsonl','assisted_trace.jsonl','source_after.json','task_config_snapshot.json']},tool_sha256=sha(__file__),
    limits=['Only actual recorded native samples. Rates spanning gaps are marked by tick interval and never counted as consecutive-tick qualification.',
            'Finite pose-rate observations do not replace or erase original raw solver velocities, model guards or missing ticks.'])
save(args.output,report)
print(json.dumps({k:v for k,v in report.items() if k not in ['source_sha256','observed_rates','original_phase_boundary_labels','limits']}))
