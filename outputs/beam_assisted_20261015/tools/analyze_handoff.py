"""Compare recorded control transitions and frozen-grasp residuals; no physics."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from beam_workflow import save,sha
from index_run import select
from c04_held_residual_failure_review import err,serial

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--part',type=int);a=ap.parse_args()
    p=Path(__file__).resolve().parents[1];d=p/'runs'/a.run;repository=p/'assisted_repository'
    sys.path[:0]=[str(repository),str(p.parent/'runtime_tools/geometry_deps')]
    from sweep_model import SceneModel,pose_matrix
    cfg=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0];phases={v['name']:v for v in cfg['phase_specs']}
    trace=select(d,'assisted_trace.jsonl',0,2147483647)
    result=json.loads((d/'collect_results.json').read_text())[0]
    terminal=result['phase_transition_history'][-1];failed_spec=phases.get(terminal.get('from_phase'),{}).get('local_skill',{})
    captures=[v for v in trace if v.get('event')=='attach_measured_pose' and v.get('object')!='fabrica_beam_6']
    name='fabrica_beam_'+str(a.part) if a.part is not None else failed_spec.get('object')
    if not name and captures:name=captures[-1]['object']
    capture=next((v for v in reversed(captures) if v.get('object')==name),None)
    if capture is None:
        save(p/(a.cycle+'_handoff_analysis.json'),dict(run=a.run,status='selected_part_capture_not_reached',selected_object=name,formal_success=False));return
    start=capture['step'];end=result['steps'];robot=capture['robot'];name=capture['object']
    motion=[v for v in select(d,'motion_trace.jsonl',start,end) if v.get('robot')==robot]
    model=SceneModel(d/'scene_geometry.json');tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
    relations=[]
    for m in motion:
        s=phases.get(m['phase'],{}).get('local_skill',{})
        if s.get('object')!=name or not s.get('requires_held_object') or not s.get('target_object_target'):continue
        g=cfg['target_poses'][s['target_object_target']];goal=pose_matrix(g['position'],g['orientation'])
        if s.get('target_object_offset'):goal[:3,3]+=s['target_object_offset']
        relations.append(np.linalg.inv(pose_matrix(m['target_pose']['position'],m['target_pose']['orientation']))@goal)
    if not relations:raise ValueError('No recorded target relation')
    relation=relations[0];spread=max(float(np.max(abs(v-relation))) for v in relations)
    if spread>1e-8:raise ValueError('Captured relation changed')
    native_relation=np.linalg.inv(pose_matrix(capture['native_hand_position'],capture['native_hand_orientation']))@pose_matrix(capture['position'],capture['orientation'])
    native=select(d,'physics_boundary_trace.jsonl',start,end);entities={v['step']:v for v in select(d,'entity_geometry_trace.jsonl',start,end)}
    rows=[];groups={}
    for n in native:
        attachment=n.get('task_constraints',{}).get('attachments',{}).get(name)
        if not attachment or attachment.get('robot_name')!=robot:continue
        b=next((v for v in n['bodies'] if v['name']==name),None)
        if not b or not b.get('valid') or not n.get('valid'):continue
        q={k:np.asarray(v['joint_positions'][0],float) for k,v in n['robots'].items()};fk=model.poses(q)[tcp]
        actual=pose_matrix(b['positions'][0],b['orientations'][0]);residual=err(actual,fk@relation)
        r=dict(step=n['cycle_step'],boundary=n['boundary'],phase=n['cycle_phase'],physics_index=n['physics_step_index'],model_native_residual=residual)
        if n['boundary']=='before_action' and n['cycle_step'] in entities:
            e=entities[n['cycle_step']];hand=np.asarray(e['body_poses'][tcp]);r.update(wrist_FK_native=err(fk,hand),captured_native_relation_drift=err(actual,hand@native_relation))
        rows.append(r);key=r['phase']+'|'+r['boundary'];g=groups.setdefault(key,dict(samples=0,max_position_m=0.,max_orientation_rad=0.))
        g['samples']+=1
        for k in ('position_m','orientation_rad'):g['max_'+k]=max(g['max_'+k],residual[k])
    increments=[];by_step={v['step']:v for v in motion}
    part_id=name.rsplit('_',1)[-1]
    for phase in (v for v in phases if '_part_'+part_id+'_' in v and v.endswith(('_pickup_clearance','_assembly_clearance','_transport_hover'))):
        selected=[v for v in motion if v['phase']==phase]
        if not selected:continue
        first=selected[0];prev=by_step.get(first['step']-1)
        row=dict(phase=phase,first_command_step=first['step'],last_command_step=selected[-1]['step'],raw_adapter_scope='Before outer final clamp; not a per-tick SDK measurement')
        if prev:
            row.update(raw_TCP_command_increment=err(pose_matrix(prev['command_target_pose']['position'],prev['command_target_pose']['orientation']),pose_matrix(first['command_target_pose']['position'],first['command_target_pose']['orientation'])),raw_joint_command_increment_rad=float(np.max(abs(np.asarray(first['command_q'])-prev['command_q']))))
        increments.append(row)
    anchor=next((v for v in trace if v.get('event')=='joint_anchor_readback' and v.get('joint_path')==capture['joint_path']),None)
    anchor_error=None
    if anchor:
        local0=np.eye(4);local0[:3,3]=anchor['local_position0']
        local1=pose_matrix(anchor['local_position1'],anchor['local_orientation1'])
        anchor_error=err(pose_matrix(capture['position'],capture['orientation'])@local0,pose_matrix(capture['native_hand_position'],capture['native_hand_orientation'])@local1)
    # Prefix equality uses exact original row extraction, before either new handoff.
    prefix=[];old=p/'runs/A16_beam_full'
    for step in (10448,10632):
        left=select(old,'physics_boundary_trace.jsonl',step,step,'before_action');right=select(d,'physics_boundary_trace.jsonl',step,step,'before_action')
        if len(left)==len(right)==1:
            prefix.append(dict(step=step,maximum_all_joint_difference_rad=max(float(np.max(abs(np.asarray(left[0]['robots'][k]['joint_positions'])-right[0]['robots'][k]['joint_positions']))) for k in left[0]['robots'])))
    publications={(v['step'],v['robot']):v for v in trace if v.get('event')=='published_command'}
    sdk_count=0;sdk_max=0.;sdk_errors=[]
    for n in select(d,'physics_boundary_trace.jsonl',0,end,'after_action'):
        for who,raw in n['robots'].items():
            key=(n['cycle_step'],who)
            if key not in publications:continue
            actions=[v for v in raw['sdk_submission']['actions'] if v.get('joint_indices')==list(range(6))]
            if len(actions)!=1:sdk_errors.append(dict(step=key[0],robot=who,error='SDK arm action absent/duplicate'));continue
            sdk_count+=1;sdk_max=max(sdk_max,float(np.max(abs(np.asarray(actions[0]['joint_positions'])-publications[key]['published_q']))))
    sdk=dict(completed_submissions=sdk_count,maximum_published_to_SDK_delta_rad=sdk_max,errors=sdk_errors,passed=sdk_count>0 and sdk_max<1e-12 and not sdk_errors)
    report=dict(run=a.run,cycle=a.cycle,formal_success=False,captured_controller_relation=relation.tolist(),captured_native_relation=native_relation.tolist(),captured_relation_spread=spread,authored_joint_anchor_at_capture_error=anchor_error,groups=groups,rows=rows,SDK_receipt=sdk,raw_adapter_transition_increments=increments,prefix_against_A16=prefix,handoff_seed_events=[v for v in trace if v.get('event')=='phase_handoff_seed'],terminal_transition=result['phase_transition_history'][-1],tool_sha256=sha(__file__),raw_index_sha256=sha(d/'raw_offset_index.json'),limits=['Only recorded native boundary samples; no between-stride peak claim.','Raw adapter commands precede the outer publication guard; SDK receipt is separately verified. Off-stride first transition final-q/SDK was not recorded individually in v20.','USD anchor readback proves authored frames at capture, not native solver compliance.','A16 and this run prefix comparisons show measured equality only at stated samples.'])
    save(p/(a.cycle+'_handoff_analysis.json'),json.loads(json.dumps(report,default=serial)))
    print(json.dumps({k:v for k,v in report.items() if k in ['run','cycle','groups','raw_adapter_transition_increments','prefix_against_A16','authored_joint_anchor_at_capture_error','terminal_transition']}))
if __name__=='__main__':main()
