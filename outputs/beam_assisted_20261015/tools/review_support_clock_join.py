"""Independent same-clock support review; retain the original phase-key report."""
import argparse,copy,json,sys
from pathlib import Path
from beam_workflow import save,sha
from index_run import select
from summarize_C_support import pose_matrix,finite,FINGERS

def bind(native,entity,frames,contract):
    step=entity['step']
    if (native.get('boundary')!='before_action' or native.get('cycle_step')!=step
        or native.get('task_step')!=step or native.get('valid') is not True
        or native.get('clock_valid') is not True):raise ValueError('invalid native cycle')
    if len(frames)!=3 or {v['camera'] for v in frames}!={'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'}:raise ValueError('three unique camera clocks required')
    stamps=[v['physics_stamp'] for v in frames]
    if not all(v.get('valid') is True and type(v.get('epoch')) is int and v['index']==native['physics_step_index'] and abs(v['time']-native['physics_time'])<1e-9 for v in stamps):raise ValueError('camera/native clock disagreement')
    if len({(v['epoch'],v['index'],v['time']) for v in stamps})!=1:raise ValueError('camera clocks disagree')
    names={contract['object'],*(contract['robot']+'/'+f for f in FINGERS)}
    bodies={v['name']:v for v in native['bodies'] if v['name'] in names}
    if set(bodies)!=names:raise ValueError('support native body coverage missing')
    error=0.
    for body in bodies.values():
        actual=pose_matrix(body);saved=entity['body_poses'][body['path']]
        if len(saved)!=4 or any(len(v)!=4 for v in saved) or not all(finite(v) for line in saved for v in line):raise ValueError('invalid saved entity pose')
        error=max(error,max(abs(actual[i][j]-saved[i][j]) for i in range(4) for j in range(4)))
    if error>1e-10:raise ValueError('native/entity support-body pose mismatch')
    return dict(step=step,epoch=stamps[0]['epoch'],physics_index=native['physics_step_index'],physics_time=native['physics_time'],maximum_pose_matrix_error=error,native_phase=native.get('phase'),cycle_phase=native.get('cycle_phase'),entity_phase=entity['phase'],phase_labels_differ=native.get('phase')!=entity['phase'])

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');a=ap.parse_args()
    p=Path(__file__).resolve().parents[1];d=p/'runs'/a.run
    original=d/'C_support_summary.json';original_sha=sha(original)
    cfg=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0]
    contracts={v['name']:v['beam_assisted_visual_support'] for v in cfg['phase_specs'] if v.get('beam_assisted_visual_support')}
    trace=select(d,'visual_support_trace.jsonl',0,2147483647) if (d/'visual_support_trace.jsonl').exists() else []
    if not trace:
        save(d/'support_clock_join_review.json',dict(passed=False,status='not_reached_or_no_support_samples',trace_samples=0,original_report_sha256=original_sha,tool_sha256=sha(__file__)))
        return 4
    low=min(v['step'] for v in trace);high=max(v['step'] for v in trace)
    def unique(rows,key):
        out={}
        for row in rows:
            k=row[key]
            if k in out:raise ValueError('duplicate clock/cycle row')
            out[k]=row
        return out
    entities=unique(select(d,'entity_geometry_trace.jsonl',low,high),'step')
    natives=unique(select(d,'physics_boundary_trace.jsonl',low,high,'before_action'),'cycle_step')
    clocks={}
    for v in select(d,'video_frame_clock.jsonl',low,high):clocks.setdefault(v['task_step_label'],[]).append(v)
    matches=[];errors=[];seen=set();failed=[]
    for row in trace:
        step=row['step'];phase=row['phase']
        try:
            if step in seen:raise ValueError('duplicate support step')
            seen.add(step);contract=contracts[phase]
            if entities[step]['phase']!=phase:raise ValueError('support/entity phase mismatch')
            fingers=row['fingers']
            if not(row['ready'] is True and row['force_support_claimed'] is False and len(fingers)==2 and {v['finger'] for v in fingers}==set(FINGERS) and all(v['within_visual_support_gap'] is True and finite(v['signed_gap_m']) and contract['minimum_surface_gap_m']<=v['signed_gap_m']<=contract['maximum_surface_gap_m'] for v in fingers)):failed.append(step)
            matches.append(bind(natives[step],entities[step],clocks[step],contract))
        except (KeyError,TypeError,ValueError,IndexError) as exc:errors.append(dict(step=step,phase=phase,error=str(exc)))
    first=trace[0];step=first['step'];contract=contracts[first['phase']];controls={}
    for name,kind in [('clock_shift_rejected','clock'),('wrong_body_pose_rejected','pose')]:
        frames=copy.deepcopy(clocks[step]);entity=copy.deepcopy(entities[step])
        if kind=='clock':frames[0]['physics_stamp']['index']+=1
        else:entity['body_poses'][next(v['path'] for v in natives[step]['bodies'] if v['name']==contract['object'])][0][3]+=.0001
        try:bind(natives[step],entity,frames,contract)
        except ValueError:controls[name]=True
        else:controls[name]=False
    if sha(original)!=original_sha:raise ValueError('Original support report changed')
    report=dict(passed=not errors and not failed and len(matches)==len(trace) and all(controls.values()),run=a.run,trace_samples=len(trace),same_native_pose_samples=len(matches),failed_support_steps=failed,errors=errors,negative_controls=controls,original_report_sha256=original_sha,tool_sha256=sha(__file__),raw_index_sha256=sha(d/'raw_offset_index.json'),phase_relabels=[v for v in matches if v['phase_labels_differ']],native_matches=matches,limits=['Same-cycle before_action bound by native index/epoch/three camera clocks and exact three native support poses; phase names are diagnostic labels.','Only recorded reached support samples, no force support or between-sample CCD/assembly claim.'])
    save(d/'support_clock_join_review.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ['native_matches']}))
    return 0 if report['passed'] else 4
if __name__=='__main__':raise SystemExit(main())
