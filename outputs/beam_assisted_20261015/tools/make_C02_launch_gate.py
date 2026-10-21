"""Create C02 gate from an explicit identity-only reuse proof and new stateful checks."""
from pathlib import Path
import copy,hashlib,json,math,re,sys
P=Path(__file__).resolve().parents[1];T=P/'tools';A=P/'assisted_repository';R=P.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(n):return json.loads((P/n).read_text())
def require(v,m):
 if not v:raise ValueError(m)
def bound(n):
 p=Path(n);p=p if p.is_absolute() else P/p
 require(p.is_file(),'missing '+str(p));return {'path':str(p),'sha256':sha(p)}

def finite(v):return type(v) in (int,float) and math.isfinite(v)
def good(report,schema):
 require(isinstance(report,dict) and report['schema']==schema and report['passed'] is True and report['errors']==[],schema+' failed/incomplete')
def vector(v):return isinstance(v,list) and len(v)==6 and all(finite(x) for x in v)
def cache_key(v):return (v['kind'],v['phase'],v.get('path_index'),v.get('native_step'),v['case'])
def cache_scope(s,full):
 require(s['insertion_anchors']==20 and s['free_hover_anchors']==8 and s['native_base_anchors']==3,'wrong approved anchor matrix')
 require(s['positive_cases']==s['expected_case_count']==600 and s['maximum_authorized_positive_cases']==684,'wrong600-case budget')
 require(s['dangerous_counterexamples']==s['expected_counterexamples']==8,'wrong negative matrix')
 require(s['seed_lookahead_precheck_passed'] is True and s['seed_numerical_inset_position_m']==s['seed_numerical_inset_orientation_rad']==1e-7,'seed limit/precheck missing')
 require(s['mesh_count']==s['full_visible_meshes']==54 and s['self_checked'] is True and s['self_and_complete_grippers_required'] is True,'full54/self scope missing')
 selected=s['selected_anchors'];require(isinstance(selected,list) and len(selected)==28,'selected anchor coverage')
 expected=set();insert=set();hover={i:[] for i in range(4)}
 for v in selected:
  require(type(v['path_index']) is int and 0<=v['path_index']<6306,'invalid path anchor')
  m=re.search(r'_part_([0-3])_(insert_0[15789]|transport_hover)$',v['phase']);require(m is not None,'unexpected held phase')
  part=int(m.group(1))
  if v['kind']=='insertion':
   require(m.group(2)!='transport_hover','insertion phase mismatch');insert.add((part,m.group(2)))
  else:
   require(v['kind']=='free_hover' and m.group(2)=='transport_hover','hover phase mismatch');hover[part].append(v['path_index'])
  for case in range(21):expected.add((v['kind'],v['phase'],v['path_index'],None,case))
 require(insert=={(p,'insert_'+s) for p in range(4) for s in ('01','05','07','08','09')} and all(len(set(v))==2 for v in hover.values()),'part/height coverage incomplete')
 near=s['free_hover_right_nearest'];require(len(near)==4 and {v['part'] for v in near}==set(range(4)),'nearest-right hover coverage')
 for v in near:require(v['nearest_path_index'] in hover[v['part']] and finite(v['distance_m']),'nearest-right node omitted')
 steps=s['native_base_steps'];require(isinstance(steps,list) and len(set(steps))==3 and all(type(x) is int and x>=0 for x in steps),'native base anchors missing')
 for step in steps:
  for mode in ('empty','position','orientation','combined'):expected.add(('native_base_turn','base_6_assembly_clearance',None,step,mode))
 require(len(expected)==600,'duplicate selected cases')
 if full:require(s['actual_case_count']==600 and s['actual_counterexamples']==8 and type(s['full_q_states']) is int and s['full_q_states']>0,'full execution incomplete')
 else:require(s['actual_case_count']==s['actual_counterexamples']==s['full_q_states']==0,'scope report masquerades as full execution')
 return expected

def cached_bindings(report,sources,inputs,tool,cfg):
 require(report['source_sha256']==sources and len(sources)==5,'cached source exact-five binding')
 require(report['input_sha256']==inputs and len(inputs)==11,'cached input exact-eleven binding')
 require(report['tool_sha256']==tool and report['config_sha256']==cfg,'cached tool/config changed')

def validate_cached(cached,scope,sources,inputs,tool,cfg):
 good(cached,'beam_C02_cached_lead_full54_v1');good(scope,'beam_C02_cached_lead_scope_v1')
 for r in (cached,scope):cached_bindings(r,sources,inputs,tool,cfg)
 expected=cache_scope(cached['scope'],True);require(cache_scope(scope['scope'],False)==expected,'scope/full matrix differs')
 require({k:v for k,v in cached['scope'].items() if k not in ('actual_case_count','actual_counterexamples','full_q_states')}=={k:v for k,v in scope['scope'].items() if k not in ('actual_case_count','actual_counterexamples','full_q_states')},'scope/full declaration changed')
 require(scope['pending']==['robot IK and previous-q tracking feasibility','production final-q guard','all54/self swept mesh'],'scope pending boundaries missing')
 seeds=scope['seed_probes'];require(isinstance(seeds,list) and len(seeds)==600,'seed matrix incomplete')
 require({(v['phase'],v.get('path_index'),v.get('native_step'),v['case']) for v in seeds}=={(v[1],v[2],v[3],v[4]) for v in expected},'seed case keys incomplete')
 for v in seeds:
  require(finite(v['seed_position_lead_m']) and 0<=v['seed_position_lead_m']<=.003000001 and finite(v['seed_orientation_lead_rad']) and 0<=v['seed_orientation_lead_rad']<=.020000001,'seed beyond production lookahead')
 cases=cached['cases'];require(isinstance(cases,list) and len(cases)==600 and {cache_key(v) for v in cases}==expected,'full case keys/count incomplete')
 for v in cases:
  require(v['passed'] is True and all(vector(v[k]) for k in ('actual_q','previous_published_q','final_q')),'failed/incomplete published-q case')
  require(max(abs(a-b) for a,b in zip(v['actual_q'],v['previous_published_q']))<=.1200000001,'cache previous publication exceeds tracking budget')
  ratio=v['publication']['accepted_ratio'];require(finite(ratio) and 0<=ratio<=1,'invalid final guard ratio')
  require(type(v['mesh_sweep_samples']) is int and v['mesh_sweep_samples']>=4,'case lacks both endpoint sweeps')
  for key in ('cache_effect_on_tcp_position_m','cache_effect_on_tcp_orientation_rad'):require(finite(v[key]) and v[key]>=0,'nonfinite cached effect')
 require(max(v['cache_effect_on_tcp_position_m'] for v in cases)>1e-5 and max(v['cache_effect_on_tcp_orientation_rad'] for v in cases)>.001,'cached command never exercised')
 negatives=cached['counterexamples'];require(len(negatives)==8 and {(v['part'],v['kind']) for v in negatives}=={(p,k) for p in range(4) for k in ('lateral_1p2mm','orientation_20mrad')},'negative case keys incomplete')
 for v in negatives:
  orientation=v['kind']=='orientation_20mrad'
  require(v['passed'] is True and v['ready'] is False and finite(v['object_z_delta_m']) and v['object_z_delta_m']>=-1e-12,'unsafe cached descent accepted')
  require(v['rejection_expected'] is orientation and v['direct_command_guard_accepts'] is (not orientation),'negative guard classification mismatch')
 mesh=cached['mesh_report'];require(mesh['meshes']==54 and mesh['self_checked'] is True and mesh['samples']==cached['scope']['full_q_states']==sum(v['mesh_sweep_samples'] for v in cases),'mesh coverage/count mismatch')
 require(type(mesh['pair_checks']) is int and mesh['pair_checks']>0 and isinstance(mesh['intersections'],list),'missing mesh pairs/intersections')
 require(all(finite(v['depth']) and 0<=v['depth']<=1e-6 for v in mesh['intersections']),'non-target intersection exceeds1um')

def validate_lifecycle(report,current,tests,old):
 require(report['schema']=='beam_C02_identity_lifecycle_regression_v1' and report['passed'] is True,'lifecycle report schema/status')
 require(report['current_source_sha256']==current and report['testfile_sha256']==tests and report['old_source_sha256']==old,'lifecycle stale source/tests')
 require(report['old_source_counterexample_failed'] is True and report['actual_frame_cache_preserved'] is True and report['warm12_ready'] is True,'lifecycle proof incomplete')
 require(report['fixed_tests_passed']==report['fixed_tests_total']==16 and finite(report['maximum_orientation_lead_rad']) and .019<report['maximum_orientation_lead_rad']<=.0200001 and report['test_angle_numerical_tolerance_rad']==1e-7,'lifecycle counts/lead limit')
 before,after=report['results']['original'],report['results']['fixed']
 for r,h in ((before,old),(after,current)):
  require(r['source_sha256']==h and r['total']==len(r['cases'])==16 and r['passed']==sum(v['passed'] is True for v in r['cases']) and r['failed']==sum(v['passed'] is False for v in r['cases']),'lifecycle detailed counts/source')
 require(before['failed']==5 and after['failed']==0 and after['actual_frame_cache_preserved'] is True,'old/new counterexample mismatch')
 require(after['warm12']==[{'tick':i,'ready':i==12,'count':i} for i in range(1,13)],'native twelve-tick warm window missing')
 leads=after['lead_trace'];require(len(leads)==9 and max(v['orientation_lead_rad'] for v in leads)==report['maximum_orientation_lead_rad'] and leads[1]['orientation_lead_rad']>.007,'final published cache did not accumulate')

def validate_reuse(report,current,tools,old,oldtools,prior_evidence,prior_sha,inputs,after_sha,archive_shas,reuse_tool_sha):
 good(report,'beam_C02_exact_identity_reuse_v1');change=report['source_change']
 require(change['source_count']==len(current)==len(old)==889 and change['frozen_tool_count']==len(tools)==len(oldtools)==18,'reuse source/tool counts')
 require(change['identity_ast_only'] is True and change['current_source_sha256']==current and change['old_source_sha256']==old and change['frozen_tools_sha256']==tools==oldtools,'reuse current/old bindings')
 require(set(change['changed_paths'])=={'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py','tests/toolkits/test_beam_assisted_object_servo.py'},'reuse changed paths')
 require(set(current)==set(old) and len(change['changed_paths'])==2 and {k for k in current if current[k]!=old[k]}==set(change['changed_paths']),'reuse declared/actual change mismatch')
 require(change['old_source_after_sha256']==after_sha and change['old_source_archive_sha256']==archive_shas[0] and change['old_tools_archive_sha256']==archive_shas[1] and report['tool_sha256']==reuse_tool_sha,'reuse after/archive/tool binding')
 require(report['original_C01_gate_sha256']==prior_sha and set(report['original_C01_evidence'])==set(prior_evidence),'reuse original evidence keys')
 for name,h in prior_evidence.items():require(report['original_C01_evidence'][name]['sha256']==report['original_C01_evidence'][name]['expected_sha256']==h,'reuse old evidence hash')
 require(set(report['unchanged_inputs'])==set(inputs)=={'actual_configuration','geometry','bimanual_path','support','support_path'},'reuse required input keys')
 for name,(path,h) in inputs.items():
  v=report['unchanged_inputs'][name];require(v['unchanged'] is True and str(Path(v['path']))==path and v['sha256']==v['expected_sha256']==h,'reuse input mismatch')
 controls={'exact_identity_suffix_deletion','unchanged_old_source_rejected','comment_only_candidate_accepted','helper_call_change_rejected','guard_change_rejected','phase_key_change_rejected','authoritative_field_change_rejected'}
 require(set(report['contract_controls'])==controls and all(v is True for v in report['contract_controls'].values()),'reuse negative controls missing')
 require(report['reused_scope']==dict(insert_anchors=210,insert_cases=4410,hover_anchors=73,hover_cases=1533,right_acquisition_full54_self_samples=108,previous_q_equals_actual_q=True,previous_object_command_is_none=True,geometry_pair_grid_and_world_anchor_unchanged=True,persistent_cache_coverage_inferred=False,native_state_lifetime_coverage_inferred=False),'reuse scope widened/incomplete')

def validate_progress(progress,cached,sources,inputs,tool,cfg,full_sha):
 good(progress,'beam_C02_published_progress_v1')
 cached_bindings(progress,sources,inputs,tool,cfg)
 require(progress['full_report_sha256']==full_sha,'progress not bound to completed full report')
 coherence=progress['cache_coherence']
 require(coherence['checked_cases']==600 and coherence['invalid_cases']==[] and finite(coherence['max_position_lead_m']) and 0<=coherence['max_position_lead_m']<=.003+1e-12 and finite(coherence['max_orientation_lead_rad']) and 0<=coherence['max_orientation_lead_rad']<=.02+1e-12,'post-IK cache radius/coherence failed')
 cases={cache_key(v):v for v in cached['cases']}
 inserts,bases=progress['insert_checks'],progress['base_checks']
 require(len(inserts)==4 and {v['part'] for v in inserts}=={'0','1','2','3'},'four post-guard insert progress checks missing')
 require(len(bases)==6 and {(v['native_step'],v['mode']) for v in bases}=={(s,m) for s in cached['scope']['native_base_steps'] for m in ('orientation','combined')},'six post-guard base progress checks missing')
 for rows,kind in ((inserts,'insertion'),(bases,'native_base_turn')):
  for v in rows:
   key=(kind,v['phase'],v['path_index'],v['native_step'],v['case']);require(key in cases,'progress row not a completed case')
   case=cases[key];delta=max(abs(a-b) for a,b in zip(case['actual_q'],case['final_q']))
   require(v['passed'] is True and finite(v['accepted_ratio']) and v['accepted_ratio']==case['publication']['accepted_ratio']>0 and finite(v['published_joint_delta_rad']) and abs(v['published_joint_delta_rad']-delta)<=1e-12 and delta>1e-10,'final guard published only a hold')
   if kind=='insertion':
    require(v['phase'].endswith('_insert_01') and '_part_'+v['part']+'_' in v['phase'] and v['case']==0,'wrong insertion center')
    require(all(finite(v[k]) for k in ('actual_height_m','target_height_m','final_native_axial_progress_m')) and v['actual_height_m']>v['target_height_m'] and 1e-6<v['final_native_axial_progress_m']<=v['actual_height_m']-v['target_height_m']+.00002,'no legal final axial progress')
   else:
    require(v['case']==v['mode'] and all(finite(v[k]) and v[k]>=0 for k in ('actual_orientation_error_rad','final_orientation_error_rad','orientation_progress_rad')) and v['orientation_progress_rad']>1e-6 and abs(v['actual_orientation_error_rad']-v['final_orientation_error_rad']-v['orientation_progress_rad'])<=1e-12,'no final angular progress')

def contract_tests():
 """Pure report-contract controls; these never substitute for IK/FCL evidence."""
 sources={f'source{i}':'a'*64 for i in range(5)};inputs={f'input{i}':'b'*64 for i in range(11)};tool='c'*64;cfg='d'*64;progress_tool='e'*64;full_sha='f'*64
 selection=[];i=0
 for part in range(4):
  for suffix in ('01','05','07','08','09'):
   selection.append(dict(path_index=i,kind='insertion',phase=f'assemble_0_part_{part}_insert_{suffix}'));i+=1
  for _ in range(2):selection.append(dict(path_index=i,kind='free_hover',phase=f'assemble_0_part_{part}_transport_hover'));i+=1
 s=dict(insertion_anchors=20,free_hover_anchors=8,native_base_anchors=3,positive_cases=600,maximum_authorized_positive_cases=684,dangerous_counterexamples=8,expected_case_count=600,actual_case_count=600,expected_counterexamples=8,actual_counterexamples=8,selected_anchors=selection,free_hover_right_nearest=[dict(part=p,nearest_path_index=next(v['path_index'] for v in selection if v['kind']=='free_hover' and f'_part_{p}_' in v['phase']),distance_m=.01) for p in range(4)],native_base_steps=[100,200,300],cache_seed_scope='test',seed_numerical_inset_position_m=1e-7,seed_numerical_inset_orientation_rad=1e-7,seed_lookahead_precheck_passed=True,mesh_count=54,self_and_complete_grippers_required=True,full_visible_meshes=54,self_checked=True,full_q_states=2400)
 keys=cache_scope(s,True);cases=[]
 for kind,phase,index,step,case in sorted(keys,key=str):cases.append(dict(kind=kind,phase=phase,path_index=index,native_step=step,case=case,mode='combined' if kind!='native_base_turn' else case,passed=True,actual_q=[0.]*6,previous_published_q=[0.]*6,final_q=[.001,0,0,0,0,0],publication={'accepted_ratio':1.},mesh_sweep_samples=4,cache_effect_on_tcp_position_m=.00002,cache_effect_on_tcp_orientation_rad=.002))
 negatives=[dict(part=p,kind=k,passed=True,ready=False,object_z_delta_m=0.,rejection_expected=k.startswith('orientation'),direct_command_guard_accepts=not k.startswith('orientation')) for p in range(4) for k in ('lateral_1p2mm','orientation_20mrad')]
 full=dict(schema='beam_C02_cached_lead_full54_v1',passed=True,errors=[],tool_sha256=tool,config_sha256=cfg,source_sha256=sources,input_sha256=inputs,scope=s,cases=cases,counterexamples=negatives,mesh_report=dict(meshes=54,self_checked=True,samples=2400,pair_checks=100,intersections=[]))
 scope=copy.deepcopy(full);scope.update(schema='beam_C02_cached_lead_scope_v1',pending=['robot IK and previous-q tracking feasibility','production final-q guard','all54/self swept mesh'],seed_probes=[dict(phase=v[1],path_index=v[2],native_step=v[3],case=v[4],seed_position_lead_m=.001,seed_orientation_lead_rad=.001) for v in keys]);scope['scope'].update(actual_case_count=0,actual_counterexamples=0,full_q_states=0)
 inserts=[];bases=[]
 for v in cases:
  row={k:v[k] for k in ('phase','path_index','native_step','case','mode')};row.update(passed=True,accepted_ratio=1.,published_joint_delta_rad=.001)
  if v['kind']=='insertion' and v['phase'].endswith('_insert_01') and v['case']==0:row.update(part=v['phase'].split('_part_')[1].split('_')[0],actual_height_m=1.,target_height_m=.98,final_native_axial_progress_m=.0005);inserts.append(row)
  if v['kind']=='native_base_turn' and v['mode'] in ('orientation','combined'):row.update(actual_orientation_error_rad=.1,final_orientation_error_rad=.099,orientation_progress_rad=.001);bases.append(row)
 progress=dict(schema='beam_C02_published_progress_v1',passed=True,errors=[],source_sha256=sources,input_sha256=inputs,tool_sha256=progress_tool,config_sha256=cfg,full_report_sha256=full_sha,cache_coherence=dict(checked_cases=600,invalid_cases=[],max_position_lead_m=.003,max_orientation_lead_rad=.02),insert_checks=inserts,base_checks=bases)
 def check(f,q,r):validate_cached(f,q,sources,inputs,tool,cfg);validate_progress(r,f,sources,inputs,progress_tool,cfg,full_sha)
 check(full,scope,progress);results=[{'name':'complete_synthetic_contract_allowed','passed':True}]
 changes=[('empty_passed',lambda f,q,r:f.clear() or f.update(passed=True)),('missing_case',lambda f,q,r:f['cases'].pop()),('missing_source',lambda f,q,r:f['source_sha256'].pop('source0')),('wrong_input_hash',lambda f,q,r:f['input_sha256'].update(input0='0'*64)),('wrong_tool_sha',lambda f,q,r:f.update(tool_sha256='0'*64)),('wrong_mesh_count',lambda f,q,r:f['mesh_report'].update(meshes=53)),('self_check_disabled',lambda f,q,r:f['mesh_report'].update(self_checked=False)),('intersection',lambda f,q,r:f['mesh_report']['intersections'].append({'depth':.000002})),('missing_counterexample',lambda f,q,r:f['counterexamples'].pop()),('scope_used_as_full',lambda f,q,r:f.update(schema='beam_C02_cached_lead_scope_v1')),('scope_source_missing',lambda f,q,r:q['source_sha256'].clear()),('seed_outside_lookahead',lambda f,q,r:q['seed_probes'][0].update(seed_position_lead_m=.0031)),('all_hold_progress',lambda f,q,r:r['insert_checks'][0].update(published_joint_delta_rad=0.)),('no_axial_progress',lambda f,q,r:r['insert_checks'][0].update(final_native_axial_progress_m=0.)),('stale_progress_report',lambda f,q,r:r.update(full_report_sha256='0'*64)),('out_of_radius_cache',lambda f,q,r:r['cache_coherence'].update(max_position_lead_m=.0031))]
 for name,change in changes:
  f,q,r=copy.deepcopy((full,scope,progress));change(f,q,r)
  try:check(f,q,r)
  except (ValueError,KeyError,TypeError):results.append({'name':name+'_rejected','passed':True})
  else:results.append({'name':name+'_rejected','passed':False})
 skills='toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py';test='tests/toolkits/test_beam_assisted_object_servo.py'
 old={**{f'file{i}':'a'*64 for i in range(887)},skills:'a'*64,test:'a'*64};current={**old,skills:'b'*64,test:'b'*64};tools={f'tool{i}':'c'*64 for i in range(18)};refs={'original.json':'d'*64};same={k:('/fixture/'+k,'e'*64) for k in ('actual_configuration','geometry','bimanual_path','support','support_path')}
 cert=dict(schema='beam_C02_exact_identity_reuse_v1',passed=True,errors=[],tool_sha256='f'*64,source_change=dict(source_count=889,frozen_tool_count=18,identity_ast_only=True,current_source_sha256=current,old_source_sha256=old,frozen_tools_sha256=tools,changed_paths=[skills,test],old_source_after_sha256='1'*64,old_source_archive_sha256='2'*64,old_tools_archive_sha256='3'*64),original_C01_gate_sha256='4'*64,original_C01_evidence={'original.json':dict(sha256='d'*64,expected_sha256='d'*64)},unchanged_inputs={k:dict(path=v[0],sha256=v[1],expected_sha256=v[1],unchanged=True) for k,v in same.items()},contract_controls={k:True for k in ('exact_identity_suffix_deletion','unchanged_old_source_rejected','comment_only_candidate_accepted','helper_call_change_rejected','guard_change_rejected','phase_key_change_rejected','authoritative_field_change_rejected')},reused_scope=dict(insert_anchors=210,insert_cases=4410,hover_anchors=73,hover_cases=1533,right_acquisition_full54_self_samples=108,previous_q_equals_actual_q=True,previous_object_command_is_none=True,geometry_pair_grid_and_world_anchor_unchanged=True,persistent_cache_coverage_inferred=False,native_state_lifetime_coverage_inferred=False))
 def reuse_check(v):validate_reuse(v,current,tools,old,tools,refs,'4'*64,same,'1'*64,['2'*64,'3'*64],'f'*64)
 reuse_check(cert);results.append(dict(name='complete_reuse_contract_allowed',passed=True))
 for name,change in [('empty_reuse_inputs',lambda v:v['unchanged_inputs'].clear()),('empty_old_evidence',lambda v:v['original_C01_evidence'].clear()),('missing_old_source',lambda v:v['source_change']['old_source_sha256'].pop(skills)),('missing_reuse_negative_control',lambda v:v['contract_controls'].pop('guard_change_rejected')),('changed_old_archive',lambda v:v['source_change'].update(old_source_archive_sha256='0'*64)),('scope_widening',lambda v:v['reused_scope'].update(persistent_cache_coverage_inferred=True))]:
  v=copy.deepcopy(cert);change(v)
  try:reuse_check(v)
  except (ValueError,KeyError,TypeError):results.append(dict(name=name+'_rejected',passed=True))
  else:results.append(dict(name=name+'_rejected',passed=False))
 life=read('C02_identity_lifecycle_validation.json');life_args=[life[k] for k in ('current_source_sha256','testfile_sha256','old_source_sha256')];validate_lifecycle(life,*life_args);results.append(dict(name='complete_lifecycle_contract_allowed',passed=True))
 for name,change in [('step_only_lead',lambda v:v.update(maximum_orientation_lead_rad=.004)),('warm_count_stuck_one',lambda v:v['results']['fixed']['warm12'][-1].update(count=1)),('missing_lifecycle_case',lambda v:v['results']['fixed']['cases'].pop())]:
  v=copy.deepcopy(life);change(v)
  try:validate_lifecycle(v,*life_args)
  except (ValueError,KeyError,TypeError):results.append(dict(name=name+'_rejected',passed=True))
  else:results.append(dict(name=name+'_rejected',passed=False))
 output=dict(schema='beam_C02_gate_evidence_contract_tests_v1',passed=all(v['passed'] for v in results),pure_memory=True,native_started=False,generator_sha256=sha(__file__),checks=results)
 (P/'C02_gate_evidence_contract_tests.json').write_text(json.dumps(output,indent=2)+'\n');require(output['passed'],'gate evidence negative control failed');return output
def main():
 run='A14_beam_full';cycle='C02';version='v18';workflow='W03.8'
 target=P/'launch_gates'/f'{run}.json';require(not target.exists(),'gate already exists')
 contract_tests()
 manifests={}
 for label,root,path,count in [('source',A,T/'assisted_manifest.json',889),('tools',T,T/'assisted_tools_manifest.json',18),('physical',R/'candidate_repository',T/'physical_manifest.json',876)]:
  manifest=json.loads(path.read_text());require(len(manifest)==count,label+' count')
  for n,h in manifest.items():require(sha(root/n)==h,'changed '+str(root/n))
  manifests[label]={'path':str(path),'sha256':sha(path),'count':count}
 seal=read(run+'_sealed_version.json');require((seal['version'],seal['run'],seal['source_files'],seal['critical_tools'])==(version,run,889,18),'wrong seal')
 require(len(seal['archives'])==2,'rollback archives missing')
 for v in seal['archives']:
  p=P/'versions'/Path(v['path']).name;require(p.stat().st_size==v['bytes'] and sha(p)==v['sha256'],'bad archive')
 prior_path=P/'launch_gates/A13_beam_full.json';prior=json.loads(prior_path.read_text())
 require(sha(prior_path)=='c0cc2a79bbdbe02a26c5bacb270690763a4cf89a5ffa07bfb909abc001f3376e','prior gate changed')
 for check in prior['checks']:
  require(check['passed'] is True,'prior check failed')
  for item in check['evidence']:require(sha(item['path'])==item['sha256'],'prior evidence changed '+item['path'])
 after_path=P/'runs/A13_beam_full/source_after.json';after=read(str(after_path));require(after['differences']==[] and after['tool_differences']==[],'A13 source changed')
 reuse=read('C02_preflight_reuse_certificate.json');cached=read('C02_cached_lead_preflight.json');scope=read('C02_cached_lead_scope_precheck.json');lifecycle=read('C02_identity_lifecycle_validation.json')
 cfgpath=P/'C02_actual_configuration_preflight_v18_layout505_config.json';cfg=json.loads(cfgpath.read_text())
 require(sha(cfgpath)==sha(P/'C01_actual_configuration_preflight_v17d_layout505_config.json'),'actual config changed')
 current_manifest=json.loads((T/'assisted_manifest.json').read_text());tool_manifest=json.loads((T/'assisted_tools_manifest.json').read_text())
 old_sources={v['path']:v['sha256'] for v in after['files']};old_tools={v['path']:v['sha256'] for v in after['tool_files']}
 prior_evidence={Path(v['path']).name:v['sha256'] for c in prior['checks'] for v in c['evidence']}
 old_seal=read('A13_beam_full_sealed_version.json');archive_hashes=[next(v['sha256'] for v in old_seal['archives'] if ('_tools_' in Path(v['path']).name)==tools) for tools in (False,True)]
 original_full=read('C01_full_published_corridor_final.json')
 geometry=Path(reuse['unchanged_inputs']['geometry']['path'])
 expected_inputs={'actual_configuration':(str(cfgpath),sha(cfgpath)),'geometry':(str(geometry),original_full['geometry_sha256']),'bimanual_path':(str(P/'C01_bimanual_repaired_v2_path.jsonl'),'d2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7'),'support':(str(P/'C01_right_support_candidate.json'),prior_evidence['C01_right_support_candidate.json']),'support_path':(str(P/'C01_right_support_path.jsonl'),'d15d23f448ab6fdb66fd371268d755b9ce540b22aad60e5e0b8eef1537022461')}
 for path,h in expected_inputs.values():require(sha(path)==h,'reuse actual input changed '+path)
 validate_reuse(reuse,current_manifest,tool_manifest,old_sources,old_tools,prior_evidence,sha(prior_path),expected_inputs,sha(after_path),archive_hashes,sha(T/'c02_reuse_certificate.py'))
 skills='toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py';testfile='tests/toolkits/test_beam_assisted_object_servo.py'
 validate_lifecycle(lifecycle,sha(A/skills),sha(A/testfile),old_sources[skills])
 sources={str(v):sha(v) for v in [A/skills,A/'toolkits/factory_dual_franka_assembly/beam_assisted_servo.py',A/'toolkits/factory_dual_franka_assembly/beam_assisted_state.py',T/'sweep_model.py',T/'visual_mesh_check.py']}
 input_paths=[geometry,cfgpath,P/'C01_bimanual_repaired_v2_path.jsonl',P/'C01_right_support_candidate.json',P/'C02_preflight_reuse_certificate.json',P/'C01_full_published_corridor.json',P/'C01_full_published_corridor_final.json']+[P/'runs/A13_beam_full'/v for v in ('motion_trace.jsonl','entity_geometry_trace.jsonl','physics_boundary_trace.jsonl','task_config_snapshot.json')]
 inputs={str(v):sha(v) for v in input_paths}
 validate_cached(cached,scope,sources,inputs,sha(T/'c02_cached_lead_preflight.py'),sha(cfgpath))
 progress=read('C02_published_progress_validation.json');require(Path(progress['full_report_path'])==P/'C02_cached_lead_preflight.json','progress full path mismatch')
 validate_progress(progress,cached,sources,inputs,sha(T/'c02_published_progress_review.py'),sha(cfgpath),sha(P/'C02_cached_lead_preflight.json'))
 cfgreport=read('C02_actual_configuration_preflight_v18.json');require(cfgreport['passed'] is True and cfgreport['simulator_created'] is False,'config failed')
 require(len(cfgreport['rows'])==4 and {v['layout_seed'] for v in cfgreport['rows']}=={0,1,505,777},'four layouts required')
 for row in cfgreport['rows']:
  rr=row['report'];require(rr['passed'] is True and rr['errors']==[] and row['phase_count']==98 and rr['all_phase_budget_required']<=row['max_steps'],'config row failed')
 signature=next(v['report']['configuration_signature'] for v in cfgreport['rows'] if v['layout_seed']==505)
 sys.path[:0]=[str(A),str(T)]
 from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import configuration_signature
 from check_launch_gate import validate
 require(signature==configuration_signature(cfg)==prior['actual_configuration_signature'],'actual full-model signature changed')
 log=(P/'C02_checks_v18.log').read_text();match=re.search(r'(\d+) passed, 2 warnings in ',log)
 require(match and int(match[1])==379 and not re.search(r'\b[1-9]\d* (?:failed|errors?)\b',log),'source suite incomplete (expected379)')
 batch=read('repair_batch_C_20261015.json');require(batch['native_attempts_started']==1 and batch['maximum_native_attempts']==5 and batch['next_native_run_authorized'] is True,'attempt count/authorization')
 require(batch['workflow_version']==workflow,'workflow not recorded')
 summary={'passed':True,'source_checks_passed':int(match[1]),'source_manifest':manifests['source'],'tools_manifest':manifests['tools'],'source_log':bound('C02_checks_v18.log'),'native_success_claimed':False}
 (P/'C02_checks_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
 definitions=[
 ('actual_configuration_and_targets','four actual layouts unchanged98-phase full config; stable-identity-only source proof',['C02_actual_configuration_preflight_v18.json',str(cfgpath),'C02_preflight_reuse_certificate.json']),
 ('full_phase_release_completion_interfaces','new stateful lifetime regressions and current complete suite; reused unchanged startup/release/final/cap contracts',['C02_checks_summary.json','C02_identity_lifecycle_validation.json','C02_preflight_reuse_certificate.json']),
 ('full_entities_and_camera_visibility','explicit identity-only reuse of C01 full nominal geometry and complete cameras; new cached final-command full54mesh checks',['C02_preflight_reuse_certificate.json','C02_cached_lead_preflight.json']),
 ('production_controller_object_space_replay','actual A13 consecutive observations through production context plus old unsafe A12 descent counterexample reuse',['C01_identity_native_replay.json','C02_identity_lifecycle_validation.json','C02_preflight_reuse_certificate.json']),
 ('admitted_error_and_final_command_geometry','reused5943 instantaneous cases plus new feasible cache-lead final-q/54mesh, unsafe-descent probes and actual post-guard progress',['C02_cached_lead_scope_precheck.json','C02_cached_lead_preflight.json','C02_published_progress_validation.json','C02_preflight_reuse_certificate.json']),
 ('bimanual_support_lifecycle','unchanged84-phase continuous right visual pinch, acquisition/open/retreat/missing-native checks; cached full-mesh interference supplement',['C02_preflight_reuse_certificate.json','C02_cached_lead_preflight.json'])]
 common=[bound(run+'_sealed_version.json'),bound(prior_path),bound('C02_gate_evidence_contract_tests.json')]+[bound(T/v) for v in ('c02_reuse_certificate.py','c02_cached_lead_preflight.py','c02_published_progress_review.py')]
 gate={'route':'assisted','run_name':run,'cycle':cycle,'workflow_version':workflow,'source_manifest_sha256':manifests['source']['sha256'],'tools_manifest_sha256':manifests['tools']['sha256'],'actual_configuration_signature':signature,'native_initialization_check_required':True,'mandatory_unresolved':[],'right_arm_visual_support_required':True,'control_safety_metric':'object_space_after_final_joint_limits','formal_physical_success':False,'checks':[{'name':n,'scope':s,'passed':True,'evidence':[bound(v) for v in files]+common} for n,s,files in definitions]}
 validation=validate(gate,run_name=run,source_manifest=T/'assisted_manifest.json',tools_manifest=T/'assisted_tools_manifest.json')
 target.write_text(json.dumps(gate,indent=2)+'\n')
 result={'passed':True,'gate':bound(target),'validation':validation,'source_checks':int(match[1]),'generator':bound(Path(__file__)),'native_started':False}
 (P/'C02_launch_gate_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':
 try:
  if '--contract-tests' in sys.argv:print(json.dumps(contract_tests()))
  else:main()
 except (ValueError,KeyError,TypeError,OSError,StopIteration) as e:
  print(json.dumps({'passed':False,'native_started':False,'blocker':str(e)}));raise SystemExit(2)
