"""Read-only original same-clock admission/reference analysis after native closure."""
import json,sys
from pathlib import Path
from beam_workflow import save,sha
from index_run import select
P=Path(__file__).resolve().parents[1];run,cycle=sys.argv[1:3];d=P/'runs'/run
require=lambda v,m:None if v else (_ for _ in ()).throw(ValueError(m))
result=json.loads((d/'collect_results.json').read_text())[0];trace=select(d,'assisted_trace.jsonl',0,2147483647)
phases={r['phase'] for r in trace if r.get('event')=='absolute_object_servo' and '_insert_' in r.get('phase','')};reports=[]
for phase in sorted(phases):
 samples=[r for r in trace if r.get('event')=='absolute_object_servo' and r.get('phase')==phase]
 ref=[r for r in trace if r.get('event')=='alignment_reference' and r.get('phase')==phase]
 warm=[r for r in samples if r['descent_admission']['ready']];first=warm[0]['step'] if warm else None
 cold=[r for r in samples if first is None or r['step']<first]
 heights=[r['current_object_pose']['position'][2] for r in cold]
 reports.append(dict(phase=phase,samples=len(samples),ready_samples=len(warm),first_ready_step=first,max_native_count=max(r['descent_admission']['native_count'] for r in samples),
  minimum_lateral_m=min(r['descent_admission']['lateral_error_m'] for r in samples),maximum_lateral_m=max(r['descent_admission']['lateral_error_m'] for r in samples),
  first_lateral_m=samples[0]['descent_admission']['lateral_error_m'],last_lateral_m=samples[-1]['descent_admission']['lateral_error_m'],
  maximum_orientation_error_rad=max(r['descent_admission']['orientation_error_rad'] for r in samples),
  first_warm_detail=warm[0]['descent_admission'] if warm else None,pre_first_ready_actual_descent_m=heights[0]-min(heights) if heights else None,
  maximum_recorded_cold_reference_displacement_m=max((r['actual_cold_axial_displacement_m'] for r in ref),default=None),
  maximum_lateral_bias_m=max((sum(v*v for v in r['lateral_bias_m'])**.5 for r in ref),default=None),last_sample=samples[-1]))
violations=[r for r in trace if r.get('event')=='cold_actual_height_violation']
# Every stored first96 publication must match the original after_action SDK;
# failed prepublication ticks intentionally have no SDK acceptance.
publications={(r['step'],r['robot']):r for r in trace if r.get('event')=='published_command' and '_insert_' in r.get('phase','')}
native=select(d,'physics_boundary_trace.jsonl',0,2147483647,'after_action');matched=[];maximum=0.
for n in native:
 for who,s in n['robots'].items():
  key=(n['cycle_step'],who)
  if key not in publications:continue
  command=next(a['joint_positions'] for a in s['sdk_submission']['actions'] if a.get('joint_indices')==list(range(6)))
  delta=max(abs(a-b) for a,b in zip(command,publications[key]['published_q']));require(delta<1e-12,'SDK mismatch');maximum=max(maximum,delta);matched.append(dict(step=key[0],robot=who,physics_index=n['physics_step_index']))
# Measured file timestamps separate init/ACK/action, and encode durations come
# from the production recorder itself, without estimating old uninstrumented time.
startup={}
for n in ['started_at','startup_images_ready.json','startup_image_review.json','collect_results.json','finished_at']:
 f=d/n
 if f.exists():startup[n]=f.stat().st_mtime
if 'startup_images_ready.json' in startup and 'startup_image_review.json' in startup:startup['ready_to_ACK_s']=startup['startup_image_review.json']-startup['startup_images_ready.json']
video=json.loads((d/'episode_0000.json').read_text());enc=[]
def walk(x,path=''):
 if isinstance(x,dict):
  if 'encode_elapsed_s' in x:enc.append(dict(path=path,elapsed_s=x['encode_elapsed_s']))
  for k,v in x.items():walk(v,path+'/'+k)
 elif isinstance(x,list):
  for i,v in enumerate(x):walk(v,path+'/'+str(i))
walk(video.get('recorded_video_summaries',{}),'/recorded_video_summaries')
require(len(enc)==3,'three unique production encoder durations required')
for r in reports:
 r['pre_first_stored_ready_sample_actual_descent_m']=r.pop('pre_first_ready_actual_descent_m')
 r['first_ready_step_scope']='First stored stride8 ready sample, not exact first ready control tick. Missing ready sample is not proof no readiness occurred.'
report=dict(run=run,cycle=cycle,passed_analysis=True,formal_success=False,terminal_transition=result['phase_transition_history'][-1],result_fields={k:result.get(k) for k in ['success','assisted_success','steps','phase','failure_reason','phase_status']},insertion_phases=reports,cold_height_violations=violations,stored_insertion_SDK_matches=len(matched),maximum_insertion_SDK_delta_rad=maximum,file_wall_times=startup,encode_intervals=enc,encode_total_s=sum(r['elapsed_s'] for r in enc),source_after_sha256=sha(d/'source_after.json'),tool_sha256=sha(__file__),limits=['Original stored admission samples at stride8; first96 initial-insert native/SDK window is denser but not every control event is recorded. No unobserved peaks inferred. Distance before first stored ready sample may include unrecorded admitted ticks; actual cold displacement uses the logged per-episode world reference. File timestamps measure receipt boundaries; encoder intervals are production monotonic durations.'])
save(P/(cycle+'_alignment_native_analysis_reviewed.json'),report);print(json.dumps({k:v for k,v in report.items() if k not in ['insertion_phases','cold_height_violations']}))
