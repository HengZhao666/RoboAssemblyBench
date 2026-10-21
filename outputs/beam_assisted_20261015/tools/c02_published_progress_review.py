"""Derive post-guard progress and cache coherence from C02's preserved full run."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pose(t):
    q=Rotation.from_matrix(t[:3,:3]).as_quat()
    return {'position':t[:3,3].copy(),'orientation':q[[3,0,1,2]]}
def angle(a,b):
    return float((Rotation.from_quat(np.asarray(a)[[1,2,3,0]]).inv()*Rotation.from_quat(np.asarray(b)[[1,2,3,0]])).magnitude())
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    for n in ('full-report','repository','tools','geometry','config','path','base-run'):
        ap.add_argument('--'+n,type=Path,required=True)
    ap.add_argument('--output',type=Path);a=ap.parse_args();p=a.path.parent
    sys.path[:0]=[str(a.repository),str(a.tools),'/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    from sweep_model import SceneModel,pose_matrix
    from toolkits.factory_dual_franka_assembly.beam_assisted_servo import anchored_object_prediction
    full=json.loads(a.full_report.read_text());c=json.loads(a.config.read_text());c=c.get('task_configs',[c])[0]
    specs={v['name']:v['local_skill'] for v in c['phase_specs'] if 'local_skill' in v}
    model=SceneModel(a.geometry);rows=[json.loads(l) for l in a.path.open()]
    old=json.loads((p/'C01_full_published_corridor.json').read_text());hov=json.loads((p/'C01_full_published_corridor_final.json').read_text())
    src={(v['path_index'],v['case']):v for v in old['final_published_full_mesh']['scenarios']+hov['final_published_full_mesh']['scenarios']}
    base_steps={v['native_step'] for v in full['cases'] if v['kind']=='native_base_turn'}
    native={};motions={}
    for line in (a.base_run/'physics_boundary_trace.jsonl').open():
        v=json.loads(line)
        if v.get('boundary')=='before_action' and v.get('cycle_step') in base_steps:native[v['cycle_step']]=v
    for line in (a.base_run/'motion_trace.jsonl').open():
        v=json.loads(line)
        if v['step'] in base_steps and v.get('robot')=='franka_right':motions[v['step']]=v
    errors=[]
    for path,digest in full['source_sha256'].items():
        if sha(path)!=digest:errors.append('source changed: '+path)
    for path,digest in full['input_sha256'].items():
        if sha(path)!=digest:errors.append('input changed: '+path)
    if full['config_sha256']!=sha(a.config):errors.append('actual configuration changed')
    rels={}
    def fixture(f):
        spec=specs[f['phase']];goal=dict(c['target_poses'][spec['target_object_target']])
        if spec.get('target_object_offset'):goal['position']=np.asarray(goal['position'])+spec['target_object_offset']
        if f['kind']=='native_base_turn':
            n=native[f['native_step']];b=next(v for v in n['bodies'] if v['name']=='fabrica_beam_6')
            current={'position':b['positions'][0],'orientation':b['orientations'][0]}
            q={k:np.asarray(v['joint_positions'][0],float) for k,v in n['robots'].items()};robot='franka_right'
            mt=motions[f['native_step']]['target_pose'];rel=np.linalg.inv(pose_matrix(mt['position'],mt['orientation']))@pose_matrix(goal['position'],goal['orientation'])
        else:
            r=rows[f['path_index']];robot,name=r['intended_grasp'];q={k:np.asarray(v,float) for k,v in r['q'].items()}
            current=src[(f['path_index'],f['case'])]['actual_native_object_pose']
            if f['path_index'] not in rels:
                tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
                rels[f['path_index']]=np.linalg.inv(model.poses(q)[tcp])@np.asarray(r['part_poses'][name])
            rel=rels[f['path_index']]
        q[robot][:6]=f['actual_q'];tcp=model.data['robots'][robot]['root_path']+'/wrist_3_link'
        measured=pose(model.poses(q)[tcp]@rel)
        def predict(joints):
            qq={k:v.copy() for k,v in q.items()};qq[robot][:6]=joints
            return anchored_object_prediction(current,measured,pose(model.poses(qq)[tcp]@rel))
        return current,goal,predict
    inserts=[];bases=[];cache_errors=[];maxp=0.;maxr=0.;checked=0
    for f in full['cases']:
        if f.get('passed') is not True:
            errors.append({'failed_full_case':{k:f.get(k) for k in ('phase','case','native_step','error')}});continue
        current,goal,predict=fixture(f);cache=predict(f['previous_published_q'])
        dp=float(np.linalg.norm(np.asarray(cache['position'])-current['position']));dr=angle(current['orientation'],cache['orientation'])
        cp=float(np.linalg.norm(np.asarray(cache['position'])-f['cached_native_object_pose']['position']));cr=angle(cache['orientation'],f['cached_native_object_pose']['orientation'])
        maxp=max(maxp,dp);maxr=max(maxr,dr);checked+=1
        if dp>.003+1e-12 or dr>.02+1e-12 or cp>1e-10 or cr>1e-9:
            cache_errors.append({'phase':f['phase'],'case':f['case'],'path_index':f['path_index'],'native_step':f.get('native_step'),'position_radius_m':dp,'orientation_radius_rad':dr,'cache_FK_position_mismatch_m':cp,'cache_FK_angle_mismatch_rad':cr})
        is_insert=f['kind']=='insertion' and f['phase'].endswith('_insert_01') and f['case']==0
        is_base=f['kind']=='native_base_turn' and f['mode'] in ('orientation','combined')
        if not (is_insert or is_base):continue
        final=predict(f['final_q']);ratio=f['publication']['accepted_ratio'];jointdelta=float(np.max(abs(np.asarray(f['final_q'])-f['actual_q'])))
        row={k:f.get(k) for k in ('phase','path_index','case','native_step','mode')}
        row.update(accepted_ratio=ratio,published_joint_delta_rad=jointdelta)
        if is_insert:
            progress=float(current['position'][2]-final['position'][2])
            row.update(part=f['phase'].split('_part_')[1].split('_')[0],actual_height_m=current['position'][2],target_height_m=goal['position'][2],final_native_axial_progress_m=progress,
                passed=bool(ratio>0 and jointdelta>1e-10 and progress>1e-6));inserts.append(row)
        else:
            initial_error=angle(current['orientation'],goal['orientation']);final_error=angle(final['orientation'],goal['orientation'])
            progress=initial_error-final_error
            row.update(actual_orientation_error_rad=initial_error,final_orientation_error_rad=final_error,orientation_progress_rad=progress,
                passed=bool(ratio>0 and jointdelta>1e-10 and progress>1e-6));bases.append(row)
    if cache_errors:errors.append({'invalid_cache_radii_or_incoherent_previous_q':len(cache_errors)})
    if len(inserts)!=4 or {v['part'] for v in inserts}!={'0','1','2','3'} or not all(v['passed'] for v in inserts):errors.append('four warm insert center cases lack post-guard forward progress')
    if len(bases)!=6 or not all(v['passed'] for v in bases):errors.append('six native base orientation/combined cases lack post-guard angular progress')
    if checked!=600 or full['passed'] is not True:errors.append('complete original600-case full report is not passed')
    result=dict(schema='beam_C02_published_progress_v1',passed=not errors,errors=errors,tool_sha256=sha(__file__),
        full_report_path=str(a.full_report),full_report_sha256=sha(a.full_report),config_sha256=sha(a.config),source_sha256=full['source_sha256'],input_sha256=full['input_sha256'],
        cache_coherence=dict(checked_cases=checked,invalid_cases=cache_errors,max_position_lead_m=maxp,max_orientation_lead_rad=maxr),
        insert_checks=inserts,base_checks=bases,
        scope='Source-bound post-final-guard native-anchored FK progress, not requested TCP progress; no new IK/FCL/native execution.',formal_physical_success=False)
    text=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(text)
    else:print(text,end='')
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
