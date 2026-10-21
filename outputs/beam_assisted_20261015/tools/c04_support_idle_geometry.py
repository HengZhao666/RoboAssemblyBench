"""Nine declared counterfactual CAD states around A15's support handoff.

Only two left-arm endpoints are observed native states. Right bodies are held
at their observed6536 transforms; intermediate left joints are interpolated.
This is a bounded geometry counterfactual, never a dynamics prediction.
"""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def require(v,m):
    if not v:raise ValueError(m)
def interp(a,b,t):
    out=np.eye(4);out[:3,3]=(1-t)*a[:3,3]+t*b[:3,3]
    out[:3,:3]=a[:3,:3]@Rotation.from_rotvec(t*Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).as_rotvec()).as_matrix()
    return out
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in ('run-dir','tools','native-input'):
        ap.add_argument('--'+n,type=Path,required=True)
    ap.add_argument('--output',type=Path);a=ap.parse_args();d=a.run_dir
    sys.path[:0]=[str(a.tools),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    from sweep_model import SceneModel,pose_matrix
    from visual_mesh_check import VisualMeshChecker
    from visual_support_check import check_visual_support,missing_dynamic_poses
    import fcl
    saved=json.loads(a.native_input.read_text());require(saved['run']==d.name=='A15_beam_full','wrong preserved run')
    for n,h in saved['inputs_sha256'].items():require(sha(d/n)==h,'preserved native input changed: '+n)
    model=SceneModel(d/'scene_geometry.json');actual={};native={}
    for line in (d/'entity_geometry_trace.jsonl').open():
        r=json.loads(line)
        if r['step'] in (6536,6544):actual[r['step']]=r
    for line in (d/'physics_boundary_trace.jsonl').open():
        r=json.loads(line)
        if r.get('boundary')=='before_action' and r.get('cycle_step') in (6536,6544):native[r['cycle_step']]=r
    require(set(actual)==set(native)=={6536,6544},'both exact native endpoints required')
    stamps={v['step']:v['physics_stamp'] for v in saved['published_command_events']}
    require(stamps[6536]['valid'] is True and stamps[6536]['index']==native[6536]['physics_step_index'],'first endpoint native index does not match published epoch/stamp')
    clocks={6536:[],6544:[]}
    for line in (d/'video_frame_clock.jsonl').open():
        v=json.loads(line)
        if v.get('task_step_label') in clocks:clocks[v['task_step_label']].append(v)
    for step,values in clocks.items():
        require(len(values)==3 and {v['camera'] for v in values}=={'observation.images.front','observation.images.left_wrist','observation.images.right_wrist'},'three unique endpoint clock receipts required')
        require(all(v['physics_stamp']['valid'] is True and v['physics_stamp']['epoch']==stamps[6536]['epoch'] and v['physics_stamp']['index']==native[step]['physics_step_index'] and abs(v['physics_stamp']['time']-native[step]['physics_time'])<1e-9 for v in values),'endpoint epoch/index/time mismatch')
    require(all(v['valid'] and v['clock_valid'] for v in native.values()),'invalid native endpoint')
    require(native[6544]['physics_step_index']-native[6536]['physics_step_index']==8,'native clock reset/gap')
    require(abs(native[6544]['physics_time']-native[6536]['physics_time']-8*native[6536]['physics_dt'])<1e-6,'native time discontinuity')
    cfg=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0]
    phase='assemble_00_part_3_move_above'
    support=next(v['beam_assisted_visual_support'] for v in cfg['phase_specs'] if v['name']==phase)
    poses={s:{**model.rest,**{k:np.asarray(v,float) for k,v in actual[s]['body_poses'].items()}} for s in actual}
    qs={s:{k:np.asarray(v['joint_positions'][0],float) for k,v in native[s]['robots'].items()} for s in native}
    left={m['body'] for m in model.data['visible_meshes'] if m['robot']=='franka_left'}
    right={m['body'] for m in model.data['visible_meshes'] if m['robot']=='franka_right'}
    require(left and right and not left&right,'robot fullpath mapping is not unique')
    endpoint_info=[]
    for s in (6536,6544):
        missing=missing_dynamic_poses(model,actual[s]['body_poses']);require(not missing,'actual dynamic bodies missing')
        body=next(v for v in native[s]['bodies'] if v['name']=='fabrica_beam_6')
        path=next(m['body'] for m in model.data['visible_meshes'] if m['object']=='fabrica_beam_6')
        t=pose_matrix(body['positions'][0],body['orientations'][0])
        dp=float(np.linalg.norm(t[:3,3]-poses[s][path][:3,3]));da=float(Rotation.from_matrix(t[:3,:3]@poses[s][path][:3,:3].T).magnitude())
        require(dp<1e-6 and da<1e-6,'native body/entity pose disagreement')
        endpoint_info.append(dict(cycle_step=s,physics_index=native[s]['physics_step_index'],native_phase=native[s]['phase'],entity_phase=actual[s]['phase'],native_body_entity_position_error_m=dp,native_body_entity_angle_error_rad=da,body_count=len(actual[s]['body_poses'])))
    def update(checker,p):
        for m,o in zip(checker.meshes,checker.shapes):
            t=p[m['body']];o.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
    actual_checker=VisualMeshChecker(model,include_self=True);actual_support=[]
    for s in (6536,6544):
        actual_checker.check(poses[s],actual[s]['phase'],s,None)
        actual_support.append(check_visual_support(actual_checker,support,actual[s]['phase'],s))
    require(actual_support[0]['ready'] is True and actual_support[1]['ready'] is False,'actual support failure was not reproduced')
    original=json.loads((d/'stop_request.json').read_text())['first_failure']['visual_support']
    require(all(abs(x['signed_gap_m']-y['signed_gap_m'])<1e-10 for x,y in zip(actual_support[1]['fingers'],original['fingers'])),'same-clock failure gap differs from original monitor')
    checker=VisualMeshChecker(model,include_self=True);observations=[]
    model0=model.poses(qs[6536]);model1=model.poses(qs[6544]);calibration={b:poses[6536][b]@np.linalg.inv(model0[b]) for b in left}
    endpoint_residual=max(float(np.linalg.norm((calibration[b]@model1[b])[:3,3]-poses[6544][b][:3,3])) for b in left)
    for i in range(9):
        fraction=i/8
        p={b:interp(poses[6536][b],poses[6544][b],fraction) for b in poses[6536]}
        if i in (0,8):
            for b in left:p[b]=poses[6536 if i==0 else 6544][b]
        else:
            q={k:v.copy() for k,v in qs[6536].items()};q['franka_left']=(1-fraction)*qs[6536]['franka_left']+fraction*qs[6544]['franka_left']
            fk=model.poses(q)
            for b in left:p[b]=calibration[b]@fk[b]
        for b in right:p[b]=poses[6536][b]
        require(not missing_dynamic_poses(model,p),'counterfactual dynamic bodies missing')
        checker.check(p,phase,6536+i,None)
        observation=check_visual_support(checker,support,phase,6536+i)
        observations.append(dict(counterfactual_step_label=6536+i,left_endpoint_observed=i in (0,8),right_replaced_with_6536=True,**observation))
    mr=checker.report();ar=actual_checker.report();errors=[]
    if any(v['depth']>1e-6 for v in mr['intersections']):errors.append('counterfactual non-target CAD overlap above1um')
    if not all(v['ready'] for v in observations):errors.append('counterfactual support gaps fail')
    if mr['samples']!=9 or mr['meshes']!=54 or mr['self_checked'] is not True:errors.append('counterfactual incomplete54/self coverage')
    inputs={str(a.native_input):sha(a.native_input)}
    inputs.update({str(d/n):sha(d/n) for n in ('scene_geometry.json','task_config_snapshot.json','entity_geometry_trace.jsonl','physics_boundary_trace.jsonl','stop_request.json','source_after.json','video_frame_clock.jsonl','support_native_phase_boundary_review.json')})
    result=dict(schema='beam_C04_support_idle_geometry_v1',passed=not errors,errors=errors,tool_sha256=sha(__file__),input_sha256=inputs,
        source_sha256={str(a.tools/n):sha(a.tools/n) for n in ('sweep_model.py','visual_mesh_check.py','visual_support_check.py')},
        actual_endpoints=endpoint_info,clock_join=dict(published_first_stamp=stamps[6536],entity_has_own_native_stamp=False,direct_epoch_index_receipts=[{'camera':v['camera'],'frame_index':v['frame_index'],'task_step_label':v['task_step_label'],'physics_stamp':v['physics_stamp']} for values in clocks.values() for v in values],phase_labels_are_not_join_keys=True),
        actual_failed_reproduction=dict(observations=actual_support,original_failure_step=6544,full54_self_mesh=ar),
        counterfactual=dict(total_states=9,actual_left_endpoints=2,interpolated_left_states=7,actual_complete_native_states_claimed=0,right_fixed_native_step=6536,
            native_left_endpoint_steps=[6536,6544],left_FK_endpoint_position_residual_m=endpoint_residual,full54_self_mesh=mr,support_observations=observations),
        limits=['All nine rows are declared counterfactual; two left endpoints are measured, seven are joint interpolants.',
          'Right bodies are frozen at observed6536, not predicted from new commands or actuator dynamics.',
          'Entity rows carry cycle_step only; both native epochs/indices are directly bound to three endpoint frame-clock receipts and the same native cycle.',
          'No between-sample CCD, unexported warehouse or PhysX cooked/SDF proof.'],formal_physical_success=False,native_physics_executed=0)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
