"""Read-only exact CAD follow-up for original approximation warnings."""
from pathlib import Path
import json,sys,collections,argparse
import numpy as np
from scipy.spatial.transform import Rotation


def review_final_home(directory, result, trace, entities):
 """Audit every stored empty-return batch; never infer unrecorded ticks."""
 from sweep_model import SceneModel
 from visual_mesh_check import ProductionCarryChecker
 from review_cache import dynamic_mesh_bodies, immutable_inputs, phase_entries
 from review_execution_evidence import completed_assisted_sequence
 robots={'franka_left','franka_right'}
 parts={'fabrica_beam_'+n for n in ('6','3','2','1','0')}
 phase_names=['beam_assisted_return_home_franka_left',
              'beam_assisted_return_clearance_franka_right',
              'beam_assisted_return_home_franka_right',
              'beam_assisted_final_verification']
 arm_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
            'wrist_1_joint','wrist_2_joint','wrist_3_joint']
 errors=[]; samples=[]; terminal={}; window={}
 def require(condition, message):
  if not condition:raise ValueError(message)
 def stamp_key(stamp):
  require(stamp.get('valid') is True and type(stamp.get('epoch')) is int
   and type(stamp.get('index')) is int and stamp['epoch']>=0 and stamp['index']>=0
   and all(np.isfinite(stamp.get(k,np.nan)) for k in ('time','dt'))
   and stamp['time']>=0 and stamp['dt']>0,'invalid native physics stamp')
  return stamp['epoch'],stamp['index'],stamp['time'],stamp['dt']
 def native_matches(row, stamp):
  return (row.get('valid') is True and row.get('clock_valid') is True
   and row.get('physics_step_index')==stamp['index']
   and row.get('physics_time')==stamp['time'] and row.get('physics_dt')==stamp['dt'])
 def native_robots(row):
  require(set(row.get('robots',{}))==robots,'native readback lacks two distinct robots')
  q={}
  for name,raw in row['robots'].items():
   names=raw.get('joint_names',[]);value=np.asarray(raw.get('joint_positions'),float)
   require(raw.get('valid') is True and raw.get('physics_handle_valid') is True
    and len(names)==12 and len(set(names))==12 and names[:6]==arm_names
    and value.shape==(1,12) and np.all(np.isfinite(value)),
    'invalid full native articulation readback: '+name)
   require(np.max(abs(value[0,6:]))<=.05,'native gripper is not fully open: '+name)
   q[name]=value[0]
  require(len({row['robots'][n].get('view_id') for n in robots})==2,
          'aliased native robot views')
  constraints=row.get('task_constraints',{})
  require(constraints.get('attachments')=={} and constraints.get('attachment_joint_paths')=={},
          'empty-return native sample has attachments or lacks their readback')
  return q
 def add_error(scope, exc, **identity):
  errors.append(dict(scope=scope,error=type(exc).__name__+': '+str(exc),**identity))
 config=json.loads((directory/'task_config_snapshot.json').read_text())['task_configs'][0]
 phases={p['name']:p for p in config['phase_specs']}
 entries=phase_entries(result['phase_transition_history'])
 coverage={p:dict(reached=p in entries,guard_events=0,checked_samples=0) for p in phase_names}
 model=SceneModel(directory/'scene_geometry.json');checker=ProductionCarryChecker(model)
 required_bodies=dynamic_mesh_bodies(model.data)
 goals={r['name']:np.asarray([r['initial_joint_positions'][j] for j in arm_names],float)
        for r in config['robots']}
 try:
  require(set(goals)==robots,'initial home configuration lacks both robots')
  criteria=config.get('success_criteria',[])
  require(len(criteria)==5 and {c.get('object') for c in criteria}==parts
   and all(c.get('position_tolerance')==.003 and c.get('orientation_tolerance')==.04 for c in criteria),
   'original five-part target criteria are missing or changed')
  for phase in phase_names:
   require(phases.get(phase,{}).get('beam_assisted_empty_return_guard') is True,
           'empty-return phase/configuration missing: '+phase)
   require(coverage[phase]['reached'],'required return/final phase was not reached: '+phase)
  final=phases[phase_names[-1]]
  require(set(final.get('beam_assisted_home_targets',{}))==robots,'final home targets incomplete')
  for name,target in final['beam_assisted_home_targets'].items():
   require(target.get('joint_names')==arm_names and target.get('tolerance_rad')==.01
    and np.array_equal(np.asarray(target.get('joint_positions'),float),goals[name]),
    'final home goal differs from configured raw initial branch: '+name)
 except Exception as exc:add_error('configuration_or_reached_phases',exc)
 guards=collections.defaultdict(list);publications=collections.defaultdict(list)
 for row in trace:
  if row.get('event')=='empty_return_published_guard':
   guards[row['step']].append(row)
   if row.get('phase') in coverage:coverage[row['phase']]['guard_events']+=1
   else:add_error('guard_phase',ValueError('unexpected empty-return guard phase'),step=row['step'])
  if row.get('event')=='published_command' and row.get('phase') in coverage:
   publications[(row['step'],row['robot'])].append(row)
  if row.get('event')=='empty_return_cad_rejected':
   add_error('native_guard_rejection',ValueError('original empty CAD guard rejected'),step=row['step'],detail=row)
 observed=set(guards)|{step for step,row in entities.items() if row.get('phase') in coverage}
 observed.update(step for step,_ in publications)
 first=min(observed,default=max(0,result.get('steps',0)-8))
 raw=select(directory,'physics_boundary_trace.jsonl',first,2147483647)
 before=collections.defaultdict(list);after=collections.defaultdict(list)
 for row in raw:
  if row.get('boundary')=='before_action':before[row['cycle_step']].append(row)
  if row.get('boundary')=='after_action':after[row['cycle_step']].append(row)
 camera_rows=select(directory,'video_frame_clock.jsonl',first,2147483647)
 clocks=collections.defaultdict(list)
 for row in camera_rows:clocks[row['task_step_label']].append(row)
 cameras={'observation.images.'+name for name in ('front','left_wrist','right_wrist')}
 for step in sorted(observed):
  try:
   require(len(guards[step])==1,'missing/duplicate stored empty-return guard')
   guard=guards[step][0];phase=guard.get('phase');stamp=guard.get('physics_stamp',{});stamp_key(stamp)
   require(phase in coverage and guard.get('passed') is True and guard.get('mode')=='empty_return'
    and guard.get('checked_samples')==5 and guard.get('complete_visible_meshes')==54
    and guard.get('self_checked') is True and guard.get('intended_grasp_exemptions') is False
    and len(guard.get('installed_objects',[]))==5
    and set(guard.get('installed_objects',[]))==parts,'incomplete original full54/self empty-return guard')
   require(len(before[step])==len(after[step])==1,'missing/duplicate native before/after action')
   a,b=before[step][0],after[step][0]
   require(native_matches(a,stamp) and native_matches(b,stamp),'native action clock differs from guard')
   q=native_robots(a);after_q=native_robots(b)
   entity_row=entities.get(step,{})
   require(required_bodies<=set(entity_row.get('body_poses',{}))
    and len(entity_row['body_poses'])>=44,'same-clock complete dynamic entity poses missing')
   views=clocks[step]
   require(len(views)==3 and {v['camera'] for v in views}==cameras
    and all(stamp_key(v['physics_stamp'])==stamp_key(stamp) for v in views),
    'three original cameras do not bind the native/entity sample')
   targets={};sdk_summary={}
   for name in sorted(robots):
    rows=publications[(step,name)];require(len(rows)==1,'missing/duplicate final published target: '+name)
    publication=rows[0]
    require(publication.get('phase')==phase and stamp_key(publication['physics_stamp'])==stamp_key(stamp),
            'publication phase/clock differs from empty guard')
    actual=np.asarray(publication['actual_q'],float);target=np.asarray(publication['published_q'],float)
    require(actual.shape==target.shape==(6,) and np.all(np.isfinite(target))
     and np.array_equal(actual,q[name][:6]) and np.array_equal(after_q[name],q[name]),
     'published actual joint state differs from native readback: '+name)
    tracking=float(np.max(abs(target-actual)))
    require(tracking<=.03+1e-12,'final published return target exceeds configured .03rad tracking cap: '+name)
    sdk=b['robots'][name].get('sdk_submission',{});actions=sdk.get('actions',[])
    arms=[v for v in actions if v.get('joint_indices')==list(range(6))]
    fingers=[v for v in actions if v.get('joint_indices')==[6]]
    require(len(arms)==len(fingers)==1 and sdk.get('skipped')==[]
     and np.array_equal(np.asarray(arms[0]['joint_positions'],float),target)
     and fingers[0].get('joint_positions')==[0.], 'SDK final dual-arm/open batch differs: '+name)
    batch=sdk.get('batch_sequence');previous=a['robots'][name].get('sdk_submission',{}).get('batch_sequence')
    require(type(batch) is int and type(previous) is int and batch==previous+1,
            'SDK batch was not newly applied once: '+name)
    native_targets=b['robots'][name].get('native_command_targets',{})
    native_positions=np.asarray(native_targets.get('position_targets'),float)
    expected=np.asarray(target,dtype=np.float32).astype(float)
    require(native_targets.get('valid') is True and not native_targets.get('errors')
     and native_positions.shape==(1,12) and np.array_equal(native_positions[0,:6],expected)
     and native_positions[0,6]==0.,'native backend target differs from exact SDK float32: '+name)
    targets[name]=target
    sdk_summary[name]=dict(batch_sequence=batch,actual_q=actual.tolist(),published_q=target.tolist(),
      tracking_error_rad=tracking,configured_tracking_limit_rad=.03,SDK_delta_rad=0.,native_float32_delta_rad=0.)
   allowed,detail=checker.check_empty(native_poses=entity_row['body_poses'],actual_q=q,
    other_targets=targets,physics_stamp=stamp,installed_objects=sorted(parts),phase=phase)
   require(allowed and detail.get('passed') is True,'independent stored final-command full54/self sweep rejected: '+repr(detail))
   coverage[phase]['checked_samples']+=1
   samples.append(dict(step=step,phase=phase,physics_stamp=stamp,SDK=sdk_summary,
                       independent_full54_sweep=detail))
  except Exception as exc:add_error('stored_empty_return_batch',exc,step=step)
 for phase,row in coverage.items():
  if not row['guard_events']:add_error('guard_coverage',ValueError('no stored guard in required phase'),phase=phase)
 try:
  qualification=result.get('assisted_final_qualification') or {};certificate=qualification.get('certificate') or {}
  current=qualification.get('current_native_sample') or {};stamp=certificate.get('physics_stamp',{});stamp_key(stamp)
  original_ready=completed_assisted_sequence(result)
  certified=certificate.get('native_home',{});fresh=current.get('home',{})
  terminal=dict(passed=False,original_five_part_common240_qualified=original_ready,
   original_native_home_certificate_ready=certified.get('ready') is True,
   original_current_native_home_ready=fresh.get('ready') is True,physics_stamp=stamp,
   original_native_home_certificate=certified)
  window=dict(original_certificate_stable_physics_steps=certificate.get('stable_physics_steps'),
   required_stable_physics_steps=certificate.get('required_stable_physics_steps'),
   independently_observed_native_ticks=None,
   scope='Original production certificate only until independently joined to raw samples.')
  require(original_ready,'original five-part/common240 success qualification is incomplete')
  required=certificate['required_stable_physics_steps']
  require(required==phases[phase_names[-1]]['advance']['physics_stable_steps'] and required>=240,
          'native common window differs from final configuration')
  require(certified.get('ready') is True and fresh.get('ready') is True
   and stamp_key(certified['physics_stamp'])==stamp_key(stamp)==stamp_key(fresh['physics_stamp'])
   and set(certified.get('robots',{}))==robots==set(fresh.get('robots',{})),
   'final native_home certificate/current readback missing or stale')
  matching=[r for r in raw if native_matches(r,stamp)]
  terminal['independent_terminal_raw_native_available']=bool(matching)
  require(matching,'no complete independent raw native sample at the terminal certificate clock')
  terminal_q=native_robots(matching[-1]);home={}
  for name in sorted(robots):
   old=certified['robots'][name];new=fresh['robots'][name]
   for field in ('articulation_id','motion_view_id','native_dof_names','joint_names',
                 'target_joint_positions','full_native_joint_positions'):
    require(old.get(field)==new.get(field),'same-clock native_home identity/state changed: '+name+'/'+field)
   require(old.get('ready') is True and old.get('joint_names')==arm_names
    and old.get('tolerance_rad')==.01 and old.get('native_dof_names')==matching[-1]['robots'][name]['joint_names']
    and old.get('motion_view_id')==matching[-1]['robots'][name]['view_id']
    and np.array_equal(np.asarray(old.get('target_joint_positions'),float),goals[name])
    and np.array_equal(np.asarray(old.get('full_native_joint_positions'),float),terminal_q[name])
    and np.array_equal(np.asarray(old.get('joint_positions'),float),terminal_q[name][:6]),
    'native_home differs from configured goal or terminal raw native state: '+name)
   error=float(np.max(abs(terminal_q[name][:6]-goals[name])))
   require(error<=.01 and old.get('maximum_error_rad')==error,'raw initial-branch home error exceeds .01rad: '+name)
   home[name]=dict(maximum_error_rad=error,target=goals[name].tolist(),actual=terminal_q[name][:6].tolist())
  require(len({certified['robots'][n]['articulation_id'] for n in robots})==2
   and len({certified['robots'][n]['motion_view_id'] for n in robots})==2,'native_home aliases robot identities')
  terminal_steps=[step for step,views in clocks.items() if len(views)==3
   and {v['camera'] for v in views}==cameras
   and all(stamp_key(v['physics_stamp'])==stamp_key(stamp) for v in views)
   and required_bodies<=set(entities.get(step,{}).get('body_poses',{}))]
  require(terminal_steps,'terminal native_home clock lacks complete same-clock entity/three-view record')
  terminal_entity=entities[max(terminal_steps)]['body_poses']
  for name,item in certificate['native_poses'].items():
   measured=np.asarray(terminal_entity[item['body_path']],float)
   recorded=pose_matrix(item['position'],item['orientation'])
   require(np.linalg.norm(measured[:3,3]-recorded[:3,3])<=1e-9
    and Rotation.from_matrix(measured[:3,:3]@recorded[:3,:3].T).magnitude()<=1e-8,
    'terminal installed body differs from original certificate: '+name)
  first_index=stamp['index']-required+1
  window_rows=[r for r in raw if first_index<=r.get('physics_step_index',-1)<=stamp['index']]
  indices=set()
  for row in window_rows:
   require(row.get('valid') is True and row.get('clock_valid') is True,'invalid stored common-window native sample')
   joints=native_robots(row)
   require(all(np.max(abs(joints[n][:6]-goals[n]))<=.01 for n in robots),
           'stored common-window actual arm left home tolerance')
   bodies=collections.defaultdict(list)
   for body in row.get('bodies',[]):bodies[body.get('name')].append(body)
   for criterion in config['success_criteria']:
    name=criterion['object'];matches=bodies[name]
    require(len(matches)==1 and matches[0].get('valid') is True,
            'stored common-window native installed body is missing: '+name)
    body=matches[0];position=np.asarray(body.get('positions'),float);orientation=np.asarray(body.get('orientations'),float)
    require(position.shape==(1,3) and orientation.shape==(1,4)
     and np.all(np.isfinite(position)) and np.all(np.isfinite(orientation)),
     'stored common-window native installed pose is invalid: '+name)
    target=config['target_poses'][criterion['target']]
    measured=pose_matrix(position[0],orientation[0]);goal=pose_matrix(target['position'],target['orientation'])
    require(np.linalg.norm(measured[:3,3]-goal[:3,3])<=criterion['position_tolerance']
     and Rotation.from_matrix(measured[:3,:3]@goal[:3,:3].T).magnitude()<=criterion['orientation_tolerance'],
     'stored common-window installed body left original target tolerance: '+name)
   indices.add(row['physics_step_index'])
  window=dict(original_certificate_stable_physics_steps=certificate['stable_physics_steps'],
   required_stable_physics_steps=required,first_physics_index=first_index,last_physics_index=stamp['index'],
   independently_observed_native_ticks=len(indices),unrecorded_ticks=required-len(indices),
   scope='Original common240 certificate plus all stored native samples; missing ticks are not reconstructed.')
  terminal.update(passed=True,home=home,entity_task_step=max(terminal_steps),
   raw_boundaries=[dict(cycle_step=r['cycle_step'],boundary=r['boundary']) for r in matching])
 except Exception as exc:
  add_error('terminal_native_home_and_common_window',exc);terminal['passed']=False
 progress=directory/'production_cad_progress.json'
 return dict(passed=not errors,phase_coverage=coverage,stored_guard_events=sum(map(len,guards.values())),
  independently_replayed_stored_batches=len(samples),independently_replayed_CAD_samples=checker.samples,
  sample_reviews=samples,terminal=terminal,common_window=window,errors=errors,
  original_native_guard_progress=json.loads(progress.read_text()) if progress.exists() else None,
  inputs=immutable_inputs(directory),checker_sha256=sha(Path(__file__).with_name('visual_mesh_check.py')),
  limits=['Stored samples only. The live guard runs every SDK batch; its full unrecorded call history is not reconstructed here.',
          'Five fractions per stored final-command batch are finite checks, not continuous CCD.',
          'The common240 certificate is original production evidence; independent stored native samples do not fill missing ticks.',
          'Visible CAD differs from PhysX cooked geometry and excludes unexported warehouse geometry.',
          'Original execution-evidence errors and missing terminal raw native data remain failures; no tolerance is widened.'])


P=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--payload-right-clearance',choices=['fabrica_beam_0','fabrica_beam_1']);ap.add_argument('--final-home',action='store_true');args=ap.parse_args();D=P/'runs'/args.run
sys.path[:0]=[str(P/'tools'),str(P.parent/'runtime_tools/geometry_deps')]
import fcl
from beam_workflow import sha,save
from index_run import select
from sweep_model import pose_matrix
g=json.loads((D/'scene_geometry.json').read_text());result=json.loads((D/'collect_results.json').read_text())[0]
post=json.loads((D/'postreview_summary.json').read_text())
entity={r['step']:r for r in select(D,'entity_geometry_trace.jsonl',0,2147483647)}
objects=[]
for m in g['visible_meshes']:
 s=fcl.BVHModel();v=np.asarray(m['points'],float);f=np.asarray(m['triangles'],np.int32);s.beginModel(len(f),len(v));s.addSubModel(v,f);s.endModel();objects.append((m,fcl.CollisionObject(s)))
def body_for_entity(name):
 if '/' in name:
  who,link=name.split('/',1);root=g['robots'][who]['root_path'];return root+'/'+link
 return next(m['body'] for m in g['visible_meshes'] if m.get('object')==name)
def pair(a,b,poses):
 pairs=[[x for x in objects if x[0]['body']==body] for body in [a,b]]
 assert all(pairs),(a,b)
 minimum=float('inf')
 for group in pairs:
  for m,o in group:
   T=np.asarray(poses[m['body']],float);o.setTransform(fcl.Transform(T[:3,:3],T[:3,3]))
 for _,x in pairs[0]:
  for _,y in pairs[1]:
   r=fcl.CollisionResult();hit=fcl.collide(x,y,fcl.CollisionRequest(enable_contact=True,num_max_contacts=8),r)
   value=-max((float(c.penetration_depth) for c in r.contacts),default=0.) if hit else float(fcl.distance(x,y,fcl.DistanceRequest(),fcl.DistanceResult()))
   minimum=min(minimum,value)
 return minimum
checks=[]
for r in result['runtime_constraint_monitor']['events']:
 who,links=r['entity_a'].split('/',1);links=links.split('->');root=g['robots'][who]['root_path'];b=body_for_entity(r['entity_b'])
 poses=entity[r['step']]['body_poses'];values=[pair(root+'/'+link,b,poses) for link in links]
 checks.append(dict(original_kind='runtime_capsule',original=r,visible_CAD_minimum_gap_m=min(values),scope='Exact meshes on both named capsule-end bodies; not the artificial capsule segment.'))
for r in post['entity_geometry']['nonexempt_overlap_candidates']+post['entity_geometry']['intended_grasp_convex_candidates']:
 a,b=map(body_for_entity,[r['entity_a'],r['entity_b']]);value=pair(a,b,entity[r['step']]['body_poses'])
 checks.append(dict(original_kind='convex_enclosure',original=r,visible_CAD_minimum_gap_m=value))
trace=select(D,'assisted_trace.jsonl',0,2147483647)
captures=[r for r in trace if r.get('event')=='attach_measured_pose'];drifts=[]
for c in captures:
 release=min([r['step'] for r in trace if r.get('event')=='support_measured_pose' and r['step']>c['step'] and r.get('object')==c['object']] + [result['steps']])
 root=g['robots'][c['robot']]['root_path'];hand=root+'/wrist_3_link';body=body_for_entity(c['object'])
 relation=np.linalg.inv(pose_matrix(c['native_hand_position'],c['native_hand_orientation']))@pose_matrix(c['position'],c['orientation'])
 samples=[]
 for step,r in entity.items():
  if not c['step']<=step<release:continue
  actual=np.asarray(r['body_poses'][body]);pred=np.asarray(r['body_poses'][hand])@relation
  samples.append(dict(step=step,position_m=float(np.linalg.norm(actual[:3,3]-pred[:3,3])),orientation_rad=float(Rotation.from_matrix(actual[:3,:3]@pred[:3,:3].T).magnitude())))
 drifts.append(dict(object=c['object'],captured_relation=relation.tolist(),capture_step=c['step'],release_world_support_step_excluded=release,visible_pinch_at_capture=c['visible_pinch'],samples=len(samples),maximum_position_drift_m=max((r['position_m'] for r in samples),default=None),maximum_orientation_drift_rad=max((r['orientation_rad'] for r in samples),default=None)))
residual=[r for r in trace if r.get('event') in ['absolute_object_servo','published_object_guard']]
groups={}
for r in residual:
 if 'fk_native_position_residual_m' not in r:continue
 key=r.get('object','unknown');s=groups.setdefault(key,dict(samples=0,max_position_m=0.,max_orientation_rad=0.))
 s['samples']+=1;s['max_position_m']=max(s['max_position_m'],r['fk_native_position_residual_m']);s['max_orientation_rad']=max(s['max_orientation_rad'],r['fk_native_orientation_residual_rad'])
right_clearance=None
if args.payload_right_clearance:
 selected=args.payload_right_clearance;payload=body_for_entity(selected)
 right=sorted({m['body'] for m,_ in objects if m.get('robot')=='franka_right'})
 assert right and any(b.endswith('/wrist_2_link') for b in right)
 capture=next(c for c in captures if c['object']==selected)
 clock_rows=collections.defaultdict(list)
 for row in select(D,'video_frame_clock.jsonl',0,2147483647):clock_rows[row['task_step_label']].append(row)
 samples=[]
 for step,row in entity.items():
  if step<capture['step'] or not row.get('phase','').endswith('_part_'+selected.rsplit('_',1)[-1]+'_assembly_clearance'):continue
  values=[(pair(payload,b,row['body_poses']),b) for b in right]
  value,other=min(values)
  trio=clock_rows[step];assert len(trio)==3 and all(v['physics_stamp']==trio[0]['physics_stamp'] for v in trio)
  samples.append(dict(step=step,physics_stamp=trio[0]['physics_stamp'],minimum_gap_m=value,right_body=other))
 assert samples
 right_clearance=dict(object=selected,right_bodies=right,samples=len(samples),minimum=min(samples,key=lambda r:r['minimum_gap_m']),first_step=samples[0]['step'],last_step=samples[-1]['step'],sample_records=samples,scope='Recorded full visible right arm/gripper versus selected payload at actual assembly_clearance entity stride; all original candidate warnings retained. Not cooked-contact force or between-sample CCD.')
report=dict(run=D.name,recorded_right_clearance=right_clearance,passed_finite_recorded_checks=all(r['visible_CAD_minimum_gap_m']>=-1e-6 for r in checks),source_after_sha256=sha(D/'source_after.json'),tool_sha256=sha(__file__),
 original_runtime_capsule_candidates=len(result['runtime_constraint_monitor']['events']),original_stage_capsule_candidates=len(result['stage_trajectory_precheck']['events']),checks=checks,
 original_sequence_warnings=result['assembly_sequence_precheck']['warnings'],
 original_missing_geometry=result['runtime_constraint_monitor']['missing_object_geometry'],original_stage_frame_coverage=result['stage_trajectory_precheck']['frame_coverage'],
 captured_native_relations=drifts,logged_FK_native_residuals=groups,
 gaps=['Stage capsule candidates are predicted approximation states, not actual mesh intersections; gripper model frames absent in that original report. No complete original sweep certification claimed.','Original sequence checker does not model assisted attachment lifecycle and emits10 release_without_attach warnings; actual captures/releases are retained separately; unreached part1/0 warnings are not executed releases.','Exact visible mesh evidence does not settle cooked/SDF impulse forces or between-sample motion.','Frozen native relation drift is measured at recorded entity stride8 samples only; no unobserved transient maximum asserted.'])
if args.final_home:report['final_home_review']=review_final_home(D,result,trace,entity)
save(P/(args.cycle+'_original_candidates_review.json'),report)
print(json.dumps(dict(passed=report['passed_finite_recorded_checks'],exact_candidates=len(checks),minimum_gap_m=min((r['visible_CAD_minimum_gap_m'] for r in checks),default=None),captured_objects=len(drifts),residuals=groups)))
