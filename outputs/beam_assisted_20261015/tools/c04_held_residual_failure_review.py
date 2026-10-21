"""Read-only A12/A16 part3 capture/lift/pickup residual decomposition; no physics."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def err(a, b):
    return dict(position_m=float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
                orientation_rad=float(Rotation.from_matrix(a[:3, :3]@b[:3, :3].T).magnitude()))
def serial(v):
    if isinstance(v, np.ndarray): return v.tolist()
    if isinstance(v, np.generic): return v.item()
    raise TypeError(type(v).__name__)


def run_review(d, repository, tools):
    from sweep_model import SceneModel, pose_matrix
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import validate_model_native_residual
    cfg=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0]
    phases={p['name']:p for p in cfg['phase_specs']}
    name='fabrica_beam_3';robot='franka_left'
    capture=None;terminal=[]
    for line in (d/'assisted_trace.jsonl').open():
        v=json.loads(line)
        if v.get('event')=='attach_measured_pose' and v.get('object')==name:capture=v
        if v.get('event')=='terminal_transition':terminal.append(v)
    if capture is None:raise ValueError('missing measured attachment')
    model=SceneModel(d/'scene_geometry.json');tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
    motion={};relations=[];selected_names=set()
    for line in (d/'motion_trace.jsonl').open():
        v=json.loads(line);s=phases.get(v.get('phase'),{}).get('local_skill',{})
        if s.get('object')!=name or not s.get('requires_held_object') or v.get('robot')!=robot:continue
        if v['phase'].endswith(('_lift','_pickup_clearance')):
            motion[v['step']]=v;selected_names.add(v['phase'])
        if s.get('target_object_target'):
            goal=cfg['target_poses'][s['target_object_target']]
            target=pose_matrix(goal['position'],goal['orientation'])
            if s.get('target_object_offset'):
                if s.get('target_object_offset_frame')!='world':raise ValueError('unsupported target offset')
                target[:3,3]+=s['target_object_offset']
            relations.append((v['step'],np.linalg.inv(pose_matrix(v['target_pose']['position'],v['target_pose']['orientation']))@target))
    if not motion or not relations:raise ValueError('no held motion/target relation')
    relation=relations[-1][1]
    relation_spread=max(float(np.max(abs(r-relation))) for _,r in relations)
    if relation_spread>1e-8:raise ValueError('full target relation not invariant')
    start=capture['step'];end=max(motion)+1
    native_capture_relation=np.linalg.inv(pose_matrix(capture['native_hand_position'],capture['native_hand_orientation']))@pose_matrix(capture['position'],capture['orientation'])
    qcapture={n:np.asarray(v['joint_positions'],float) for n,v in model.data['robots'].items()}
    qcapture[robot]=np.asarray(capture['full_joint_positions'])
    model_capture=model.poses(qcapture)[tcp]
    captured_relation_from_model=np.linalg.inv(model_capture)@pose_matrix(capture['position'],capture['orientation'])
    entities={}
    for line in (d/'entity_geometry_trace.jsonl').open():
        v=json.loads(line)
        if start<=v['step']<=end:entities[v['step']]=v
    lula_matches=[]
    for step,m in motion.items():
        q={n:np.asarray(v['joint_positions'],float) for n,v in model.data['robots'].items()}
        q[robot][:6]=m['current_q'];lula=pose_matrix(m['current_pose']['position'],m['current_pose']['orientation'])
        lula_matches.append(dict(step=step,**err(model.poses(q)[tcp],lula)))
    rows=[]
    for line in (d/'physics_boundary_trace.jsonl').open():
        v=json.loads(line);step=v.get('cycle_step',-1)
        if not start<=step<=end:continue
        b=next((x for x in v['bodies'] if x['name']==name),None)
        if b is None or not b.get('valid') or not v.get('valid'):continue
        q={n:np.asarray(x['joint_positions'][0]) for n,x in v['robots'].items()}
        model_poses=model.poses(q);fk=model_poses[tcp];actual=pose_matrix(b['positions'][0],b['orientations'][0]);pred=fk@relation
        residual=err(actual,pred)
        quat=lambda t: Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]]
        try:
            validate_model_native_residual({'position':actual[:3,3],'orientation':quat(actual)},
                {'position':pred[:3,3],'orientation':quat(pred)},.0001,.0015)
            accepted=True
        except ValueError:accepted=False
        phase=v.get('cycle_phase',v.get('phase'))
        if phase=='failed' and step==end:phase=motion[max(motion)]['phase']
        row=dict(step=step,boundary=v['boundary'],physics_index=v.get('physics_step_index'),physics_time=v.get('physics_time'),
                 phase=phase,actual_q=q[robot][:6],sdk=v['robots'][robot].get('sdk_submission'),actual_object_pose=actual,
                 model_tcp_pose=fk,model_object_pose=pred,model_native_residual=residual,current_production_budget_passed=accepted)
        # Same-cycle entity rows were captured at before_action. Do not join
        # them to after_physics/after_observation under the same step label.
        if v['boundary']=='before_action' and step in entities:
            e=entities[step];native_tcp=np.asarray(e['body_poses'][tcp]);entity_obj=np.asarray(e['body_poses'][next(k for k in e['body_poses'] if k.endswith('/'+name))])
            row.update(entity_join='exact cycle before_action; body/native object agreement checked',
                entity_native_body_error=err(entity_obj,actual),
                model_tcp_vs_native_tcp=err(fk,native_tcp),
                native_tcp_capture_vs_native_object=err(native_tcp@native_capture_relation,actual),
                captured_relation_model_vs_native_at_attach=err(captured_relation_from_model,native_capture_relation),
                native_tcp_pose=native_tcp)
            row['per_link_model_native_error']={k:err(model_poses[k],np.asarray(t)) for k,t in e['body_poses'].items()
                if k in model_poses and k.startswith(model.data['robots'][robot]['root_path']+'/')}
            lags=[]
            for offset in (-2,-1,0,1,2):
                m=motion.get(step+offset)
                if m is None:continue
                shifted={n:v.copy() for n,v in q.items()};shifted[robot][:6]=m['current_q']
                shifted_tcp=model.poses(shifted)[tcp]
                lags.append(dict(joint_sample_step=step+offset,offset_ticks=offset,
                    joint_delta_rad=float(np.max(abs(shifted[robot][:6]-q[robot][:6]))),**err(shifted_tcp,native_tcp)))
            row['native_tcp_vs_nearby_logged_joint_FK']=lags
            if step in motion:row['same_step_motion_native_joint_delta_rad']=float(np.max(abs(np.asarray(motion[step]['current_q'])-q[robot][:6])))
        if step in motion:
            m=motion[step];row['motion_command_q']=m.get('command_q');row['motion_current_q']=m.get('current_q')
        rows.append(row)
    groups={}
    for row in rows:
        key=row['phase']+'|'+row['boundary'];g=groups.setdefault(key,dict(samples=0,max_orientation_rad=0.,max_position_m=0.,exceeds_budget=0))
        r=row['model_native_residual'];g['samples']+=1;g['exceeds_budget']+=not row['current_production_budget_passed']
        if r['orientation_rad']>g['max_orientation_rad']:g.update(max_orientation_rad=r['orientation_rad'],worst_step=row['step'])
        g['max_position_m']=max(g['max_position_m'],r['position_m'])
    return dict(run=d.name,capture=capture,captured_controller_relation=relation,captured_native_hand_relation=native_capture_relation,
        relation_invariance_max_matrix_error=relation_spread,relation_capture_model_agreement=err(relation,captured_relation_from_model),
        selected_window=[start,end],phase_contracts={n:phases[n]['local_skill'] for n in selected_names},
        scene_FK_vs_logged_Lula=dict(samples=len(lula_matches),max_position_m=max(x['position_m'] for x in lula_matches),
             max_orientation_rad=max(x['orientation_rad'] for x in lula_matches)),groups=groups,
        current_budget_failed_samples=sum(not x['current_production_budget_passed'] for x in rows),rows=rows,terminal_events=terminal,
        input_sha256={str(d/n):sha(d/n) for n in ('scene_geometry.json','task_config_snapshot.json','assisted_trace.jsonl','motion_trace.jsonl','entity_geometry_trace.jsonl','physics_boundary_trace.jsonl')})


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();repository=a.root/'assisted_repository';tools=a.root/'tools'
    sys.path[:0]=[str(repository),str(tools),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    result=dict(schema='beam_C04_A16_held_residual_failure_review_v1',native_physics_executed=0,source_modified=False,
        gate_modified=False,C05_started=False,formal_success=False,tool_sha256=sha(__file__),
        source_sha256={str(repository/n):sha(repository/n) for n in ('toolkits/factory_dual_franka_assembly/beam_assisted_servo.py',
             'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py','internutopia_extension/tasks/factory_dual_franka_assembly_task.py')},
        runs=[run_review(a.root/'runs'/r,repository,tools) for r in ('A12_beam_full','A16_beam_full')],
        limits=['All four stored native boundary types are checked at their own q/body clock. Only before_action may join same-step entity.',
                'SceneModel FK is checked against recorded Lula current_pose; remaining model discrepancy is reported, not assumed zero.',
                'Native hand capture is wrist_3_link from actual robot hand_link_name; current native TCP comes only from observed entity matrix.',
                'Finite stored traces cannot prove between-stride maxima or future dynamics; no threshold or source is changed.'])
    a.output.write_text(json.dumps(result,indent=2,default=serial)+'\n')
    print(json.dumps(dict(report=str(a.output),sha256=sha(a.output),runs=[dict(run=r['run'],window=r['selected_window'],groups=r['groups'],scene_FK_vs_logged_Lula=r['scene_FK_vs_logged_Lula']) for r in result['runs']])))
if __name__=='__main__':main()
