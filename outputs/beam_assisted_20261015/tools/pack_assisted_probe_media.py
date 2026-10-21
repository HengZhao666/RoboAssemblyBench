"""One small transfer: three complete probe videos, immutable reports, clocked trios."""
from pathlib import Path
import argparse,json,tarfile
from beam_workflow import sha,save
from index_run import select

p=Path(__file__).resolve().parents[1]
ap=argparse.ArgumentParser();ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('work',type=Path);ap.add_argument('--tag',default='media');ap.add_argument('--compress',action='store_true');ap.add_argument('--review-kind',choices=['probe','full'],default='probe');ap.add_argument('--extra',action='append',default=[]);ap.add_argument('--anchor',action='append',default=[],help='Additional label:native-index selected from original records');ap.add_argument('--object',choices=['fabrica_beam_0','fabrica_beam_1']);args=ap.parse_args();assert args.tag.replace('_','').isalnum()
d=p/'runs'/args.run;w=args.work.resolve();target=w/(args.run+'_'+args.tag+('.tar.gz' if args.compress else '.tar'));assert not target.exists()
review=p/(args.cycle+('_probe_review.json' if args.review_kind=='probe' else '_postprocess_pipeline.json'));r=json.loads(review.read_text());assert r['postprocessing_completed' if args.review_kind=='probe' else 'all_required_stages_passed']
clock=[json.loads(line) for line in (d/'video_frame_clock.jsonl').open()];groups={}
for row in clock:
    s=row['physics_stamp'];groups.setdefault((s['epoch'],s['index'],s['time']),[]).append(row)
expected={'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'}
assert all({v['camera'] for v in rows}==expected and len(rows)==3 for rows in groups.values())
trace=select(d,'assisted_trace.jsonl',0,2**31-1)
selected=args.object or r.get('object','fabrica_beam_1')
selected_phase_suffix='_part_'+selected.rsplit('_',1)[-1]+'_assembly_clearance'
attach=[e for e in trace if e.get('event')=='attach_measured_pose' and e.get('object')==selected]
fail=[e for e in trace if e.get('event')=='object_model_residual_rejected'];seed=[e for e in trace if e.get('event')=='phase_handoff_seed' and e.get('phase','').endswith(selected_phase_suffix)]
first=min(k[1] for k in groups);last=max(k[1] for k in groups);targets=[]
if attach:targets.append(('capture',attach[-1]['physics_stamp']['index']))
if seed:targets.append(('clearance_entry',seed[-1]['physics_stamp']['index']))
if fail:targets.append(('failure',fail[-1]['physics_stamp']['index']))
if fail:targets.append(('tail_middle',(fail[-1]['physics_stamp']['index']+last)//2))
for stage in ['turn','transfer','lower']:
    entries=[e for e in trace if e.get('event')=='incremental_clearance_command' and e.get('stage')==stage and e.get('phase','').endswith(selected_phase_suffix)]
    if entries:targets.append((stage+'_entry',entries[0]['physics_stamp']['index']))
if args.review_kind in ('full','probe'):
    if not fail and (d/'post_failure_recording.json').exists():
        failure=json.loads((d/'post_failure_recording.json').read_text())
        index=failure['failure_physics_stamp']['index'];targets.extend([('failure',index),('tail_middle',(index+failure['end_physics_stamp']['index'])//2)])
if not fail and (d/'stop_request.json').exists():
    stop=json.loads((d/'stop_request.json').read_text());taskstep=stop.get('first_failure',{}).get('step')
    if type(taskstep) is int:
        actual=min(clock,key=lambda x:abs(x['task_step_label']-taskstep))['physics_stamp']['index']
        targets.append(('external_stop',actual));targets.append(('tail_middle',(actual+last)//2))
targets.append(('last',last))
for value in args.anchor:
    label,number=value.rsplit(':',1);index=int(number)
    assert label and all(c.isalnum() or c=='_' for c in label) and first<=index<=last
    assert label not in {name for name,_ in targets}
    targets.append((label,index))
trios=[];files={}
for name in ['video_manifest.json','collect_results.json','postreview_summary.json','startup_camera_render_diagnostics.json','short_segment_timing.json','visible_mesh_runtime_report.json','post_failure_recording.json','probe_process_receipt.json','solver_runtime_readback.json','support_warmup_qualification.json','bounded_primary_scene_runtime.json','stop_request.json','source_before.json','source_after.json','route_source_manifest.json','launch_gate.json','task_config_snapshot.json']:
    f=d/name
    if f.exists():files[name]=f
files['probe_review.json' if args.review_kind=='probe' else 'pipeline_review.json']=review;files['execution_evidence.json']=p/(args.cycle+'_execution_evidence.json')
for name in args.extra:
    path=Path(name);assert not path.is_absolute() and '..' not in path.parts and (p/path).resolve().is_relative_to(p.resolve()) and (p/path).is_file()
    files['extra/'+name]=p/path
for video in json.loads((d/'video_manifest.json').read_text()):files[video['relative_path']]=d/video['relative_path']
for label,index in targets:
    key=min(groups,key=lambda k:abs(k[1]-index));entries=[]
    for row in groups[key]:
        path=d/row['image_path'];name='review_images/'+label+'_'+row['camera'].replace('.','_')+'.png'
        assert path.is_file();files[name]=path;entries.append(dict(camera=row['camera'],path=name,source_path=str(path),sha256=sha(path),frame_index=row['frame_index']))
    trios.append(dict(label=label,requested_physics_index=index,physics_stamp=groups[key][0]['physics_stamp'],offset_ticks=key[1]-index,images=entries))
manifest=dict(schema='assisted_probe_media_transfer_v1',run=args.run,cycle=args.cycle,complete_original_videos=3,
    server_complete_decode_passed=True,full_assembly_claimed=False,trios=trios,
    files={n:dict(sha256=sha(f),size=f.stat().st_size) for n,f in files.items()},tool_sha256=sha(__file__))
mp=w/(args.run+'_'+args.tag+'_manifest.json');assert not mp.exists();save(mp,manifest)
with tarfile.open(target,'w:gz' if args.compress else 'w') as archive:
    for name,f in files.items():archive.add(f,arcname=name,recursive=False)
    archive.add(mp,arcname='transfer_manifest.json',recursive=False)
save(w/(args.run+'_'+args.tag+'_receipt.json'),dict(run=args.run,path=str(target),sha256=sha(target),bytes=target.stat().st_size,manifest_sha256=sha(mp)))
print(json.dumps(dict(run=args.run,bytes=target.stat().st_size,sha256=sha(target),clocked_trios=len(trios),original_videos=3)))
