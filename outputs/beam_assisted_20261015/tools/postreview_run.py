from pathlib import Path
import json, sys, collections, math
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1]
data=json.loads((D/'collect_results.json').read_text());r=data[0]
out={k:r.get(k) for k in ['execution_route','assisted_success','success','failed','steps','phase_status','terminal_reason','phase_transition_history','assisted_boundary_stop','post_failure_recording']}
out['reports']={}
for name in ['runtime_constraint_monitor','stage_trajectory_precheck','assembly_sequence_precheck']:
 report=r.get(name,{})
 if isinstance(report,dict):
  out['reports'][name]={k:v if not isinstance(v,list) else {'count':len(v),'first':v[:1]} for k,v in report.items() if k not in ['events','checks','entity_geometry_checks','trajectory_checks','snapshots']}
events=[]
for line in (D/'assisted_trace.jsonl').open():
 e=json.loads(line)
 if e.get('event')=='published_command':continue
 if e.get('event')=='attach_measured_pose':
  cm=e['contact_metrics'];sc=e['strict_contact']
  e={k:v for k,v in e.items() if k not in ['contact_metrics','strict_contact']}
  e['force_N']={f:cm.get(f,{}).get('force') for f in ['left_finger','right_finger']}
  e['strict_contact']={k:sc.get(k) for k in ['physical_contact_ready','strict_surface_gap_limit','strict_sample_pair']}
 events.append(e)
out['assistance_events']=events
overlaps={};grasp_candidates={};count=0
payloads={'fabrica_beam_'+i for i in ['6','3','2','1','0']}
configs=json.loads((D/'task_config_snapshot.json').read_text())['task_configs']
phase_context={p['name']:p for p in configs[0]['phase_specs']}
for line in (D/'entity_geometry_trace.jsonl').open():
 e=json.loads(line);count+=1
 phase=phase_context.get(e.get('phase'),{})
 intended=[]
 for a in phase.get('attach',[]):intended.append((a.get('object'),a.get('robot')))
 for s in [phase.get('local_skill',{}),*phase.get('local_skills',{}).values()]:
  if s.get('requires_held_object'):intended.append((s.get('object'),s.get('robot')))
 for c in e.get('events',[]):
  if c['distance']>=-1e-6:continue
  a,b=c['entity_a'],c['entity_b'];robot=any('/ur5e_' in c[k] for k in ['collider_a','collider_b'])
  if not robot and not payloads.intersection((a,b)):continue
  mount='factory_tabletop_visual' in (a,b) and any('/base_link/collisions/base/mesh' in c[k] for k in ['collider_a','collider_b'])
  grip=any(obj in (a,b) and who in ['franka_left','franka_right'] and any('ur5e_'+who.removeprefix('franka_')+'/Gripper/Robotiq_2F_85/'+f+'/' in c[k] for k in ['collider_a','collider_b'] for f in ['left_inner_finger','right_inner_finger']) for obj,who in intended)
  if mount:continue
  key=c['collider_a']+' | '+c['collider_b']
  target=grasp_candidates if grip else overlaps
  if key not in target or c['distance']<target[key]['distance']:target[key]={**c,'phase':e['phase']}
out['entity_geometry']={'samples':count,'nonexempt_overlap_candidates':list(overlaps.values()),'intended_grasp_convex_candidates':list(grasp_candidates.values()),'scope':'both robots and all five Beam parts; per-phase explicit intended inner-finger grasp candidates separated but retained; fixed base mounts excluded; convex candidates are not visible-CAD or cooked-physics certification'}
n=valid=0;max_error=0.;errors=collections.Counter()
for line in (D/'physics_boundary_trace.jsonl').open():
 e=json.loads(line);n+=1;valid+=bool(e.get('valid'))
 for name,robot in e.get('robots',{}).items():
  if not robot.get('valid'):errors['robot:'+name]+=1;continue
  q=robot.get('joint_positions',[[]])[0][:6]
  target=robot.get('native_command_targets',{}).get('position_targets',[[]])[0][:6]
  if len(q)==len(target)==6:max_error=max(max_error,max(abs(a-b) for a,b in zip(q,target)))
 for b in e.get('bodies',[]):
  if not b.get('valid'):errors[b.get('path',b.get('name','unknown_body'))]+=1
out['native']={'samples':n,'valid_samples':valid,'max_arm_actual_to_native_target_rad':max_error,'invalid_bodies_or_robots':dict(errors)}
for s in ['before','after']:
 p=D/f'source_{s}.json'
 if p.exists():
  e=json.loads(p.read_text());out[f'source_{s}']={'files':len(e['files']),'differences':e['differences']}
(D/(sys.argv[2] if len(sys.argv)>2 else 'postreview_summary.json')).write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:v for k,v in out.items() if k not in ['reports','assistance_events','entity_geometry']},ensure_ascii=False))
