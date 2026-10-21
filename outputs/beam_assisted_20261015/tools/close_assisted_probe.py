"""Close a diagnostic probe with original reports, clock/CAD checks and full media.
Unvisited assembly phases are reported as unvisited, not dynamically accepted.
"""
from pathlib import Path
import argparse,fcntl,json,subprocess,sys,time
from beam_workflow import save,sha

p=Path(__file__).resolve().parents[1];t=p/'tools'
ap=argparse.ArgumentParser();ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--resume',type=Path);args=ap.parse_args()
d=p/'runs'/args.run;assert (d/'finished_at').exists() and (d/'source_after.json').exists()
after=json.loads((d/'source_after.json').read_text());assert not after['differences'] and not after.get('tool_differences')
target=p/(args.cycle+'_probe_review.json');assert not target.exists()
begin=time.monotonic();commands=[]
resume=None
if args.resume:
    from review_cache import immutable_inputs
    resume=json.loads(args.resume.read_text())
    assert resume['run']==args.run and resume['source_after_sha256']==sha(d/'source_after.json')
    assert resume['immutable_inputs']==immutable_inputs(d)

for tool,arguments,output in [
 ('index_run.py',[str(d)],d/'raw_offset_index.json'),
 ('postreview_run.py',[args.run],d/'postreview_summary.json'),
 ('review_execution_evidence.py',[args.run,args.cycle],p/(args.cycle+'_execution_evidence.json')),
 ('archive_run.py',[args.run],p/(args.run+'_archive_manifest.json'))]:
    if output.exists():
        previous=(resume or {}).get('completed_stages',{}).get(tool)
        assert previous and previous['tool_sha256']==sha(t/tool) and previous['output_sha256']==sha(output), 'Preserve existing unbound derived output: '+str(output)
        commands.append(dict(tool=tool,returncode=0,reused=True,output_sha256=sha(output),tool_sha256=sha(t/tool),binding_receipt_sha256=sha(args.resume),elapsed_s=None))
        continue
    now=time.monotonic()
    with (d/('post_'+tool+'.log')).open('w') as stream:
        proc=subprocess.run([sys.executable,str(t/tool),*arguments],stdout=stream,stderr=subprocess.STDOUT)
    commands.append(dict(tool=tool,returncode=proc.returncode,elapsed_s=time.monotonic()-now,tool_sha256=sha(t/tool)))
    assert proc.returncode==0 and output.exists(),commands[-1]
from index_run import select
result=json.loads((d/'collect_results.json').read_text())[0]
trace=select(d,'assisted_trace.jsonl',0,2**31-1)
fixture=json.loads((d/'probe_initialization.json').read_text())['fixture']
selected_object=fixture.get('object','fabrica_beam_1')
attach=[e for e in trace if e.get('event')=='attach_measured_pose' and e.get('object')==selected_object]
rejections=[e for e in trace if e.get('event')=='object_model_residual_rejected']
commands_trace=[e for e in trace if e.get('event')=='incremental_clearance_command']
watch=json.loads((d/'visible_mesh_runtime_report.json').read_text()) if (d/'visible_mesh_runtime_report.json').exists() else None
media=json.loads((d/'video_manifest.json').read_text());assert len(media)==3 and all(v['decode_exit']==0 and not v['decode_errors'] for v in media)
evidence=json.loads((p/(args.cycle+'_execution_evidence.json')).read_text())
timing=(json.loads((d/'short_segment_timing.json').read_text()) if (d/'short_segment_timing.json').exists() else dict(available=False,reason='Original first-probe profiler finally block was not reached before SimulationApp process exit; no category timing inferred.',wrapper_receipt=json.loads((d/'probe_process_receipt.json').read_text())))
report=dict(schema='assisted_isolated_probe_review_v1',run=args.run,cycle=args.cycle,postprocessing_completed=True,
    full_assembly_accepted=False,checkpoint_restore=False,new_episode_fixture=True,object=selected_object,source_files_verified=len(after['files']),
    source_after_sha256=sha(d/'source_after.json'),original_result={k:result.get(k) for k in ['success','assisted_success','failed','steps','terminal_reason','phase_history','assisted_boundary_stop','post_failure_recording']},
    native_grasp_requalified=bool(attach),attach=attach,model_guard_rejections=rejections,
    incremental_publications=len(commands_trace),screened_lift_completed=bool(result.get('assisted_boundary_stop',{})),
    visible_full_CAD_original=watch,execution_evidence_passed=evidence['evidence_checks_passed'],
    execution_evidence_sha256=sha(p/(args.cycle+'_execution_evidence.json')),
    timing=timing,original_video_count=3,complete_video_decode_passed=True,video_manifest_sha256=sha(d/'video_manifest.json'),
    postprocess_stages=commands,elapsed_s=time.monotonic()-begin,tool_sha256=sha(__file__),
    limits=['Diagnostic initialization does not reproduce original hidden/contact/velocity state. Compare measured grasp residual/lead/phase progression before transferring the result.',
        'Original precheck/capsule/sequence warnings and missing clocks remain in original result and independent evidence; absent phases are unvisited.',
        'A passed short lift screen is not complete clearance/assembly acceptance. Full from-head verification and original success/release gates remain required.'])
save(target,report)
with (p/'beam_run.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ledger=p/'repair_batch_D_20261016.json';batch=json.loads(ledger.read_text())
    assert batch['native_attempts_started']==batch['completed_cycles']+1
    row=batch['cycles'][-1];assert row['id']==args.cycle and row['actual_run']==args.run
    row.update(state='closed',closed=True,outcome='isolated_lift_screen_passed' if report['screened_lift_completed'] else 'isolated_probe_stopped',
        review_path=str(target),review_sha256=sha(target),formal_assembly_success=False)
    batch.update(completed_cycles=batch['native_attempts_started'],last_closed_cycle=args.cycle,status='repair_preparing',next_native_run_authorized=False,
        pending_next_step='Close complete-video download and same-clock review, then select the next evidence-backed candidate within the current sole ledger grant.')
    save(ledger,batch)
print(json.dumps(dict(postprocessing_completed=True,run=args.run,screened_lift_completed=report['screened_lift_completed'],native_grasp_requalified=bool(attach),
    steps=result['steps'],reason=result.get('terminal_reason'),model_rejections=len(rejections),full_assembly_accepted=False)))
