"""Ephemeral read-only CAD monitor tied to one native assisted run."""
from pathlib import Path
import sys,json,time,hashlib,shutil
import numpy as np
from sweep_model import SceneModel
from visual_mesh_check import VisualMeshChecker
from visual_support_check import check_visual_support, missing_dynamic_poses
from publish_runtime_control import publish_stop_request
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015');D=P/'runs'/sys.argv[1]
while not (D/'scene_geometry.json').exists():
 if (D/'finished_at').exists():raise SystemExit('run ended before visible geometry was initialized')
 time.sleep(.2)
while True:
 try:model=SceneModel(D/'scene_geometry.json');break
 except json.JSONDecodeError:
  if (D/'finished_at').exists():raise SystemExit('run ended with incomplete visible geometry')
  time.sleep(.2)
checker=VisualMeshChecker(model,include_self=True);epsilon=1e-6
config=json.loads((D/'task_config_snapshot.json').read_text())['task_configs'][0];phases={p['name']:p for p in config['phase_specs']}
source_hashes={}
for name in ['watch_visible_meshes.py','visual_mesh_check.py','sweep_model.py','visual_support_check.py']:
 p=P/'tools'/name;source_hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest();shutil.copy2(p,D/('cad_guard_'+name))
while not (D/'entity_geometry_trace.jsonl').exists():
 if (D/'finished_at').exists():raise SystemExit('run ended before entity trajectory initialization')
 time.sleep(.1)
failures=[];numerical_contacts=0;last_step=None;stopped=False;support_samples=0;support_failures=[]
with (D/'entity_geometry_trace.jsonl').open() as stream:
 while True:
  position=stream.tell();line=stream.readline()
  if not line or not line.endswith('\n'):
   stream.seek(position)
   if (D/'finished_at').exists():break
   time.sleep(.05);continue
  r=json.loads(line);phase=phases.get(r['phase'],{});skill=phase.get('local_skill',{});intended=None
  if skill.get('requires_held_object') or r['phase'].endswith('_close_and_attach'):intended=(skill['robot'],skill['object'])
  elif r['phase'].endswith('_release_and_lock'):
   locks=phase.get('lock',[]);commands=phase.get('gripper_commands',{})
   if len(locks)==len(commands)==1:intended=(next(iter(commands)),locks[0]['object'])
  poses={**model.rest,**{k:np.asarray(v,float) for k,v in r['body_poses'].items()}}
  missing=missing_dynamic_poses(model,r['body_poses'])
  if missing:
   failure={'step':r['step'],'phase':r['phase'],'missing_dynamic_body_poses':missing}
   failures.append(failure)
   if not stopped and not (D/'stop_request.json').exists():
    publish_stop_request(D,{'reason':'CAD monitor lacks current dynamic entity poses; rest fallback rejected','first_failure':failure})
   stopped=True;last_step=r['step'];continue
  before=len(checker.rows);grips_before=len(checker.grasp_contacts)
  checker.check(poses,r['phase'],r['step'],intended);last_step=r['step']
  contacts=checker.rows[before:];bad=[e for e in contacts if e['depth']>epsilon]
  numerical_contacts+=sum(e['depth']<=epsilon for e in contacts)
  bad_grips=[e for e in checker.grasp_contacts[grips_before:] if e['signed_separation'] < -epsilon]
  support=phase.get('beam_assisted_visual_support');support_bad=None
  if support and r['phase']!='base_6_visual_support':
   try:observation=check_visual_support(checker,support,r['phase'],r['step'])
   except Exception as exc:observation={'step':r['step'],'phase':r['phase'],'ready':False,'error':type(exc).__name__+': '+str(exc)}
   support_samples+=1
   with (D/'visual_support_trace.jsonl').open('a') as out:out.write(json.dumps(observation)+'\n')
   if not observation['ready']:support_bad=observation;support_failures.append(observation)
  if bad or bad_grips or support_bad:
   failures.append({'step':r['step'],'phase':r['phase'],'non_target':bad,'grasp':bad_grips,'visual_support':support_bad})
   if not stopped:
    request={'reason':'actual visible support no longer satisfies both pad gaps' if support_bad and not (bad or bad_grips) else 'actual visible triangle intersection exceeds1um numerical precision','source':'same-clock full-path native entity body poses; no body state writes','first_failure':failures[-1],'epsilon_m':epsilon}
    if not (D/'stop_request.json').exists():publish_stop_request(D,request)
    stopped=True
  if checker.samples%100==0:
   (D/'visible_mesh_guard_progress.json').write_text(json.dumps({'samples':checker.samples,'last_step':last_step,'phase':r['phase'],'failures':len(failures),'numerical_surface_contacts':numerical_contacts,'stop_requested':stopped})+'\n')
report=checker.report();report.update(epsilon_m=epsilon,failures_above_precision=failures,numerical_surface_contacts=numerical_contacts,last_step=last_step,stop_requested=stopped,finished_marker_seen=True,source_sha256=source_hashes,visual_support_samples=support_samples,visual_support_failures=support_failures,
 same_robot_self_is_not_in_this_monitor=False,separate_self_preflight=str(P/'self_nominal_mesh_audit_all_v11_capture.json'),failure_tail_has_separate_native_trace_not_this_entity_stream=True)
(D/'visible_mesh_runtime_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'samples':checker.samples,'last_step':last_step,'failures':len(failures),'numerical_surface_contacts':numerical_contacts,'stop_requested':stopped,'gaps':report['gaps']}))
raise SystemExit(3 if failures else (4 if checker.samples==0 else 0))
