"""Compare calibrated full-body FK with existing native held-part observations."""
from pathlib import Path
import argparse, hashlib, json, sys
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-dir',type=Path,required=True)
    ap.add_argument('--repository',type=Path,required=True)
    ap.add_argument('--tools',type=Path,required=True)
    ap.add_argument('--part',default='3')
    ap.add_argument('--position-budget',type=float,default=.0001)
    ap.add_argument('--orientation-budget',type=float,default=.0015)
    ap.add_argument('--output',type=Path)
    a=ap.parse_args()
    sys.path[:0]=[str(a.repository),str(a.tools),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    from sweep_model import SceneModel,pose_matrix
    d=a.run_dir; config=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0]
    phases={p['name']:p for p in config['phase_specs']}; model=SceneModel(d/'scene_geometry.json')
    name='fabrica_beam_'+a.part; motions={}; relations=[]
    for l in (d/'motion_trace.jsonl').open():
        r=json.loads(l); s=phases.get(r.get('phase'),{}).get('local_skill',{})
        if (s.get('object')!=name or not s.get('requires_held_object')
                or not s.get('target_object_target') or r.get('robot')!=s.get('robot')):
            continue
        goal=config['target_poses'][s['target_object_target']]
        t=pose_matrix(goal['position'],goal['orientation'])
        if s.get('target_object_offset'):
            if s.get('target_object_offset_frame')!='world':raise ValueError('unsupported goal offset')
            t[:3,3]+=np.asarray(s['target_object_offset'])
        c=pose_matrix(r['target_pose']['position'],r['target_pose']['orientation'])
        relation=np.linalg.inv(c)@t
        relations.append((r['step'],relation))
        motions[r['step']]=r
    if not relations:raise ValueError('no captured-target relation samples')
    # Last insert's full target is a direct reconstruction of the original
    # fixed grasp. Earlier phase samples independently check its invariance.
    relation=relations[-1][1]
    relation_error=max(float(np.max(abs(t-relation))) for _,t in relations)
    entities={}
    for l in (d/'entity_geometry_trace.jsonl').open():
        r=json.loads(l)
        spec=phases.get(r.get('phase'),{}).get('local_skill',{})
        if spec.get('requires_held_object') and spec.get('object')==name:
            entities[r['step']]=r
    by_phase={}; errors=[]; maxpos=0.; maxangle=0.; fk_native_max=0.; samples=0
    for l in (d/'physics_boundary_trace.jsonl').open():
        r=json.loads(l); step=r.get('cycle_step')
        if r.get('boundary')!='before_action' or step not in entities:continue
        e=entities[step]; robot=phases[e['phase']]['local_skill']['robot']
        body=next((b for b in r['bodies'] if b['name']==name),None)
        if not r.get('valid') or body is None or not body.get('valid'):
            errors.append({'step':step,'reason':'invalid native observation'});continue
        q={n:np.asarray(v['joint_positions'][0]) for n,v in r['robots'].items()}
        fk=model.poses(q); tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        actual=pose_matrix(body['positions'][0],body['orientations'][0]); predicted=fk[tcp]@relation
        dp=float(np.linalg.norm(predicted[:3,3]-actual[:3,3])); dr=float(Rotation.from_matrix(predicted[:3,:3]@actual[:3,:3].T).magnitude())
        entity_tcp=np.asarray(e['body_poses'][tcp]); fk_native=float(np.linalg.norm(fk[tcp][:3,3]-entity_tcp[:3,3]))
        phase=e['phase']; row=by_phase.setdefault(phase,{'samples':0,'first_step':step,'last_step':step,'max_position_error_m':0.,'max_orientation_error_rad':0.,'max_nonzero_joint_FK_vs_native_TCP_position_error_m':0.})
        row['samples']+=1;row['last_step']=step
        if dp>row['max_position_error_m']:row.update(max_position_error_m=dp,worst_position_step=step)
        if dr>row['max_orientation_error_rad']:row.update(max_orientation_error_rad=dr,worst_orientation_step=step)
        row['max_nonzero_joint_FK_vs_native_TCP_position_error_m']=max(row['max_nonzero_joint_FK_vs_native_TCP_position_error_m'],fk_native)
        maxpos=max(maxpos,dp);maxangle=max(maxangle,dr);fk_native_max=max(fk_native_max,fk_native);samples+=1
    report={'kind':'existing_native_held_FK_baseline','run':str(d),'part':name,'samples':samples,
            'captured_relation_matrix':relation.tolist(),'relation_invariance_max_matrix_error':relation_error,
            'max_native_vs_captured_model_FK_object_position_error_m':maxpos,
            'max_native_vs_captured_model_FK_object_orientation_error_rad':maxangle,
            'max_nonzero_joint_FK_vs_native_TCP_position_error_m':fk_native_max,
            'phase_coverage':by_phase,'errors':errors,
            'candidate_baseline_contract':{'position_m':a.position_budget,'orientation_rad':a.orientation_budget},
            'passed':bool(samples and not errors and relation_error<1e-8 and maxpos<=a.position_budget and maxangle<=a.orientation_budget),
            'source_sha256':{n:hashlib.sha256((d/n).read_bytes()).hexdigest() for n in ['scene_geometry.json','task_config_snapshot.json','entity_geometry_trace.jsonl','motion_trace.jsonl','physics_boundary_trace.jsonl']},
            'scope':'all matching stride8 before_action requires_held_object phase samples; fixed capture relation independently reconstructed from full target phases and then applied to lift/free-space phases too; no new physics, no force-balance or future-route proof',
            'gaps':['Phases without an explicit requires_held_object declaration are excluded.','Between-stride and later unexecuted parts are unmeasured.','Native cooked/SDF vs visible geometry is outside this baseline check.']}
    text=json.dumps(report,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')
    return int(not report['passed'])


if __name__=='__main__':raise SystemExit(main())
