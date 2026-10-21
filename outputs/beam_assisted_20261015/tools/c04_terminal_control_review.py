"""Bind completed A16 originals and verify the preserved native handoff window."""
from pathlib import Path
import contextlib, hashlib, io, json, math, runpy
P=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
D=P/'runs/A16_beam_full';T=P/'tools';A=P/'assisted_repository'
COMPARISON_SHA='c2d34bb8696dfa47d464385e67033e657d6e2a618df1b2d8ce68a459244fae68'
def require(v,m):
    if not v:raise ValueError(m)
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def delta(a,b):
    require(len(a)==len(b)==6,'six arm joints required')
    return max(abs(x-y) for x,y in zip(a,b))
require((D/'finished_at').exists(),'run not finished')
after=read(D/'source_after.json');before=read(D/'source_before.json')
require(after['differences']==after['tool_differences']==[],'source changed during run')
current={v['path']:v['sha256'] for v in after['files']}
tools={v['path']:v['sha256'] for v in after['tool_files']}
require(len(current)==889 and len(tools)==19,'source/tool coverage incomplete')
require(current==read(T/'assisted_manifest.json') and tools==read(T/'assisted_tools_manifest.json'),'current manifests differ from actual run')
require(all(sha(A/k)==h for k,h in current.items()) and all(sha(T/k)==h for k,h in tools.items()),'deployed bytes differ from recorded run')
comparison_path=P/'A16_support_handoff_native_comparison.json'
require(sha(comparison_path)==COMPARISON_SHA,'original comparison overwritten')
original=read(comparison_path)
buffer=io.StringIO()
with contextlib.redirect_stdout(buffer):runpy.run_path(str(T/'c04_handoff_native_comparison.py'))
final_window=json.loads(buffer.getvalue())
fields=('native_input_comparisons','right_commands')
require(all(original[k]==final_window[k] for k in fields),'final originals differ from handoff comparison window')
require(all(original[k]['key_samples']==final_window[k]['key_samples'] for k in ('A15_support','A16_support')),'final support window differs from comparison')
result=read(D/'collect_results.json')[0];cad=read(D/'visible_mesh_runtime_report.json')
require(cad['finished_marker_seen'] is True and cad['meshes']==54 and cad['self_checked'] is True,'final CAD coverage incomplete')
required={m['body'] for m in read(D/'scene_geometry.json')['visible_meshes'] if m.get('robot') or str(m.get('object','')).startswith('fabrica_beam_')}
entity_count=0;entity_min=None;entity_missing=[];entity_steps=[]
for line in (D/'entity_geometry_trace.jsonl').open():
    r=json.loads(line);entity_count+=1;entity_steps.append(r['step'])
    count=len(r['body_poses']);entity_min=count if entity_min is None else min(entity_min,count)
    missing=sorted(required-set(r['body_poses']))
    if missing:entity_missing.append({'step':r['step'],'missing':missing})
support_rows=[json.loads(line) for line in (D/'visual_support_trace.jsonl').open()]
pub={};right=[];cleanup_events=[]
for line in (D/'assisted_trace.jsonl').open():
    r=json.loads(line)
    if r.get('event')=='published_command':
        pub[(r['step'],r['robot'])]=r
        if r['robot']=='franka_right' and r['phase'].startswith('assemble_'):right.append(r)
    elif r.get('event') in ('remove_world_support','detach_grasp') and r.get('phase')=='failed':
        cleanup_events.append(r)
target=next(r['published_q'] for r in original['right_commands'] if r['run']=='A16_beam_full' and r['step']==6536)
right_max=max((delta(r['published_q'],target) for r in right),default=None)
sdk_count=0;sdk_max=0.;native_samples={};selected_steps={10440,10448,10632,10640}
for line in (D/'physics_boundary_trace.jsonl').open():
    n=json.loads(line)
    if n.get('boundary')=='before_action' and n.get('cycle_step') in selected_steps:
        body=next(v for v in n['bodies'] if v.get('name')=='fabrica_beam_3')
        native_samples[n['cycle_step']]={'step':n['cycle_step'],'phase':n['phase'],'physics_index':n['physics_step_index'],
            'part3_position':body['positions'][0],'part3_orientation':body['orientations'][0]}
    if n.get('boundary')!='after_action':continue
    for robot,raw in n.get('robots',{}).items():
        p=pub.get((n['cycle_step'],robot))
        if p is None:continue
        actions=[v for v in raw.get('sdk_submission',{}).get('actions',[]) if v.get('joint_indices')==list(range(6)) and len(v.get('joint_positions') or [])==6]
        require(len(actions)==1,'same-cycle arm SDK action missing')
        sdk_count+=1;sdk_max=max(sdk_max,delta(actions[0]['joint_positions'],p['published_q']))
tail_recording=read(D/'post_failure_recording.json')
tail_rows=[json.loads(line) for line in (D/'post_failure_physics_trace.jsonl').open()]
tail_bodies={}
for name in ('fabrica_beam_3','fabrica_beam_6'):
    values=[next(v for v in r['bodies'] if v.get('name')==name) for r in tail_rows]
    p0=values[0]['positions'][0];q0=values[0]['orientations'][0]
    positions=[v['positions'][0] for v in values]
    angles=[]
    for v in values:
        q=v['orientations'][0];dot=abs(sum(x*y for x,y in zip(q0,q)))/math.sqrt(sum(x*x for x in q0)*sum(x*x for x in q))
        angles.append(2*math.acos(min(1.,dot)))
    tail_bodies[name]={'first_position':p0,'last_position':positions[-1],
        'maximum_translation_from_tail_start_m':max(math.sqrt(sum((x-y)**2 for x,y in zip(p,p0))) for p in positions),
        'maximum_rotation_from_tail_start_rad':max(angles),'world_Z_min_m':min(p[2] for p in positions),'world_Z_max_m':max(p[2] for p in positions)}
tail_end=tail_recording['end_physics_stamp']['index']
require(tail_recording['capture_complete'] is True and tail_recording['native_complete'] is True,'postfailure observation incomplete')
require(len(tail_rows)==151 and all(r['valid'] and r['clock_valid'] for r in tail_rows),'postfailure native samples missing')
require(all(r['physics_stamp']['index']>=tail_end for r in cleanup_events),'constraint cleanup preceded tail end')
transition=result['phase_transition_history'][-1]
fixed_names=['motion_trace.jsonl','physics_boundary_trace.jsonl','entity_geometry_trace.jsonl','assisted_trace.jsonl','source_before.json','source_after.json','collect_results.json','visible_mesh_runtime_report.json','visual_support_trace.jsonl','scene_geometry.json','task_config_snapshot.json','finished_at','post_failure_recording.json','post_failure_physics_trace.jsonl','video_frame_clock.jsonl']
bindings={str(D/n):sha(D/n) for n in fixed_names}
bindings[str(comparison_path)]=COMPARISON_SHA
bindings[str(P/'launch_gates/A16_beam_full.json')]=sha(P/'launch_gates/A16_beam_full.json')
bindings[str(P/'A16_beam_full_sealed_version.json')]=sha(P/'A16_beam_full_sealed_version.json')
for name in ('physics_boundary_trace.jsonl','assisted_trace.jsonl','visual_support_trace.jsonl'):
    p=P/'runs/A15_beam_full'/name;bindings[str(p)]=sha(p)
gaps=[f['signed_gap_m'] for r in support_rows for f in r['fingers']]
report=dict(schema='beam_A16_control_terminal_review_v1',passed=True,errors=[],tool_sha256=sha(__file__),
    current_source_sha256=current,critical_tools_sha256=tools,source_files=889,critical_tools=19,
    source_after_differences=[],critical_tool_differences=[],input_sha256=bindings,
    preserved_comparison_sha256=COMPARISON_SHA,final_originals_match_comparison_window=True,
    actual_handoff_conclusion='A15/A16 recorded inputs at6528/6536 agree exactly; preserving final-published right target removes the645.138urad target jump and avoids the original6544 visual support failure.',
    full_recorded_SDK_comparison=dict(same_cycle_after_action_rows=sdk_count,maximum_published_to_SDK_difference_rad=sdk_max),
    right_support_published_target=dict(samples=len(right),maximum_change_from_first_support_idle_target_rad=right_max),
    entity_coverage=dict(samples=entity_count,first_step=entity_steps[0],last_step=entity_steps[-1],minimum_body_poses=entity_min,
        required_dynamic_bodies=len(required),missing_samples=entity_missing,expected_stride8_missing_steps=sorted(set(range(entity_steps[0],entity_steps[-1]+1,8))-set(entity_steps))),
    CAD=dict(samples=cad['samples'],last_step=cad['last_step'],meshes=cad['meshes'],self_checked=cad['self_checked'],pair_checks=cad['pair_checks'],
        failure_count=len(cad['failures_above_precision']),numerical_surface_contacts=cad['numerical_surface_contacts'],stop_requested=cad['stop_requested'],gaps=cad['gaps']),
    visual_support=dict(samples=len(support_rows),first_step=support_rows[0]['step'],last_step=support_rows[-1]['step'],failure_count=sum(not r['ready'] for r in support_rows),
        minimum_signed_gap_m=min(gaps),maximum_signed_gap_m=max(gaps),force_support_claimed=False),
    actual_part3_key_samples=[native_samples[k] for k in sorted(native_samples)],
    failure_tail=dict(recording=tail_recording,total_native_rows_including_initial=len(tail_rows),
        body_motion=tail_bodies,cleanup_events=cleanup_events,cleanup_observed_before_tail_end=False,
        part3_attachment_task_metadata_present_all_samples=all('fabrica_beam_3' in r['task_constraints']['attachment_joint_paths'] for r in tail_rows),
        beam6_locked_target_task_metadata_present_all_samples=all(r['task_constraints']['locked_targets'].get('fabrica_beam_6')=='part_6_assembled' for r in tail_rows),
        scope='Retained robot targets plus recorded constraint task metadata during5s observation. Metadata is not a native joint-enabled readback. Cleanup events occur only at tail end; settled tail poses do not qualify task success.'),
    terminal=dict(steps=result['steps'],success=result['success'],failed=result['failed'],phase_status=result['phase_status'],transition=transition),
    qualification=dict(support_idle_fix_observed_in_native=True,first_column_attached_and_lift_phase_executed=True,
        first_column_inserted=False,all_parts_assembled=False,formal_physical_success=False),
    limits=['Support is a visual gap predicate; no force-bearing claim.','All-published SDK comparison covers recorded stride8 boundaries, not every native tick.',
        'Failure-tail physics has separate native/video evidence and is outside the entity/CAD stream.','The terminal failure is a model/native angular-residual guard; its source requires separate diagnosis.'],
    new_simulations_started=0,source_modified=False,C04_round_complete_native=True,C05_authorized=False)
out=P/'A16_control_terminal_review.json';require(not out.exists(),'refuse terminal-review overwrite')
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'passed':True,'path':str(out),'sha256':sha(out),'SDK':report['full_recorded_SDK_comparison'],'entity':report['entity_coverage'],'CAD':report['CAD'],'support':report['visual_support'],'terminal':report['terminal'],'part3':report['actual_part3_key_samples'],'tail_body_motion':tail_bodies,'cleanup_at_or_after_tail_end':True}))
