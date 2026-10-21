"""C03 resumes byte-identical v18 motion code with atomic ACK and guard-stop publication."""
from pathlib import Path
import hashlib,json,copy,tarfile
P=Path(__file__).resolve().parents[1];T=P/'tools';A=P/'assisted_repository';R=P.parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def require(x,m):
 if not x:raise ValueError(m)
def read(n):return json.loads((P/n).read_text())
def bound(n):
 p=Path(n);p=p if p.is_absolute() else P/p
 return dict(path=str(p),sha256=sha(p))
PROTOCOL_CHECKS={
 'legacy_worker_empty_read_reproduced_5_of_5','new_path_invisible_until_complete_bytes',
 'temporary_fsync_replace_directory_fsync_order','wrong_run_rejected_before_visible_path',
 'wrong_image_map_rejected_before_visible_path','wrong_image_hash_rejected_before_visible_path',
 'changed_image_bytes_rejected_before_visible_path','missing_accepted_rejected_before_visible_path',
 'rejected_accepted_rejected_before_visible_path','missing_reviewer_rejected_before_visible_path',
 'missing_scope_rejected_before_visible_path','missing_stamp_rejected_before_visible_path',
 'wrong_stamp_rejected_before_visible_path','nonfinite_optional_value_rejected_before_visible_path',
 'existing_control_never_overwritten','eight_concurrent_publishers_exactly_one_message',
 'atomic_stop_complete_json','ack_after_stop_rejected',
 'legacy_worker_stop_empty_read_reproduced_5_of_5','worker_stop_reader_observes_complete_json',
 'eight_concurrent_stop_publishers_exactly_one_message','guard_duplicate_preserves_first_stop',
 'wrong_stop_run_rejected','missing_stop_reason_rejected'}
def validate_tool_transition(oldtools,newtools):
 require(isinstance(oldtools,dict) and isinstance(newtools,dict) and len(oldtools)==18 and len(newtools)==19,'exact18 to19 critical transition required')
 require(set(newtools)==set(oldtools)|{'publish_runtime_control.py'},'only publisher may be added to critical tools')
 require(sorted(n for n in oldtools if oldtools[n]!=newtools[n])==['watch_visible_meshes.py'],'only watcher STOP writer may change')
def validate_protocol(protocol,*,publisher_sha,watcher_sha,old_watcher_sha,worker_sha,archive_sha):
 require(isinstance(protocol,dict) and protocol.get('schema')=='beam_C03_control_publish_validation_v1' and protocol.get('passed') is True,'atomic publication protocol report missing/failed')
 require(protocol.get('tool_sha256')==publisher_sha,'publisher source unbound')
 require(protocol.get('watcher_sha256')==watcher_sha and protocol.get('watcher_original_sha256')==old_watcher_sha and protocol.get('watcher_atomic_stop_only_ast') is True and protocol.get('watcher_original_archive_sha256')==archive_sha,'watcher STOP-only AST/source/archive proof missing')
 reader=protocol.get('worker_ack_reader',{})
 require(reader.get('source_sha256')==worker_sha and reader.get('ast_sha256')=='daeb7eb21715a59c7e3e7e5a3780cda3bdd5bca36873bc969edb24ac81673c86','actual unchanged worker ACK reader unbound')
 stop_reader=protocol.get('worker_stop_reader',{})
 require(stop_reader.get('source_sha256')==worker_sha and stop_reader.get('ast_sha256')=='71161d4f611a1eb2f3f90fb809223143ae15e3615c7eebe465dbfdcf7f8f4fe1','actual unchanged worker STOP reader unbound')
 checks=protocol.get('checks',[])
 require(isinstance(checks,list) and all(isinstance(v,dict) and v.get('passed') is True and isinstance(v.get('name'),str) for v in checks),'protocol check failed/malformed')
 keyed={v['name']:v for v in checks}
 require(len(keyed)==len(checks) and PROTOCOL_CHECKS<=set(keyed),'named publication positive/negative coverage incomplete')
 require(keyed['temporary_fsync_replace_directory_fsync_order'].get('observed')==['temporary_fsync','replace','directory_fsync'],'durable publication order not observed')
 require((keyed['eight_concurrent_publishers_exactly_one_message'].get('published'),keyed['eight_concurrent_publishers_exactly_one_message'].get('rejected'))==(1,7),'concurrent publisher ownership not proven')
 require((keyed['eight_concurrent_stop_publishers_exactly_one_message'].get('published'),keyed['eight_concurrent_stop_publishers_exactly_one_message'].get('rejected'))==(1,7),'concurrent STOP ownership not proven')
 require(protocol.get('previous_critical_tool_count')==18 and protocol.get('current_critical_tool_count')==19 and protocol.get('watcher_stop_writer_count')==2,'exact two STOP writer /19-tool scope missing')
 require(protocol.get('native_physics_executed')==0 and protocol.get('worker_source_modified') is False,'protocol tests must preserve unchanged worker and use no native physics')
 require(all(protocol.get(k) is True for k in ('old_direct_write_empty_read_reproduced','atomic_reader_observed_only_complete_json','startup_image_binding_checked','duplicate_publish_rejected')),'publication race/binding summary incomplete')
 require(protocol.get('native_started') is False and protocol.get('production_source_modified') is False,'protocol tests must preserve motion source and use no native process')
def main():
 run='A15_beam_full';out=P/'launch_gates'/f'{run}.json';require(not out.exists(),'refuse gate overwrite')
 priorpath=P/'launch_gates/A14_beam_full.json';require(sha(priorpath)=='f68f0218f5d6b1042bd405ed90bb2099fe098a015182ff23418da0faf62d8e71','C02 gate changed')
 prior=read('launch_gates/A14_beam_full.json')
 for row in prior['checks']:
  require(row['passed'] is True and row['evidence'],'prior gate incomplete')
  for v in row['evidence']:require(sha(v['path'])==v['sha256'],'changed prior evidence '+v['path'])
 for root,mfile,count in [(A,T/'assisted_manifest.json',889),(T,T/'assisted_tools_manifest.json',19),(R/'candidate_repository',T/'physical_manifest.json',876)]:
  m=json.loads(mfile.read_text());require(len(m)==count,'manifest count')
  for n,h in m.items():require(sha(root/n)==h,'current byte changed '+str(root/n))
 require(sha(T/'assisted_manifest.json')==prior['source_manifest_sha256'],'C03 motion source must be byte-identical C02')
 after=read('runs/A14_beam_full/source_after.json');require(after['differences']==after['tool_differences']==[] and len(after['files'])==889 and len(after['tool_files'])==18,'A14 source after not clean')
 require({v['path']:v['sha256'] for v in after['files']}==json.loads((T/'assisted_manifest.json').read_text()),'source-after map differs')
 oldtools={v['path']:v['sha256'] for v in after['tool_files']};newtools=json.loads((T/'assisted_tools_manifest.json').read_text())
 oldseal=read('A14_beam_full_sealed_version.json');oldtoolseal=oldseal['archives'][1]
 oldarchive=P/'versions'/Path(oldtoolseal['path']).name
 require(oldarchive.stat().st_size==oldtoolseal['bytes'] and sha(oldarchive)==oldtoolseal['sha256'],'original18-tool archive changed')
 with tarfile.open(oldarchive) as archived:
  require(json.loads(archived.extractfile('assisted_tools_manifest.json').read())==oldtools,'A14 after-tool map differs from sealed original18')
 validate_tool_transition(oldtools,newtools)
 protocol=read('C03_control_publish_validation.json')
 validate_protocol(protocol,publisher_sha=sha(T/'publish_runtime_control.py'),watcher_sha=sha(T/'watch_visible_meshes.py'),old_watcher_sha=oldtools['watch_visible_meshes.py'],worker_sha=newtools['assisted_worker.py'],archive_sha=sha(P/'versions/assisted_v18_tools_before_A14_beam_full.tar.gz'))
 tests=read('C03_launch_gate_contract_tests.json');require(tests['passed'] is True and tests['generator_sha256']==sha(__file__) and len(tests['checks'])>=6 and all(v['passed'] is True for v in tests['checks']),'new gate contract missing/stale')
 b=read('repair_batch_C_20261015.json');require(b['native_attempts_started']==2 and b['completed_cycles']==2 and b['maximum_native_attempts']==5 and b['next_native_run_authorized'] is True,'C03 bounded authorization')
 require(b['workflow_version']=='W03.9','C01/C02 retrospective not recorded')
 seal=read('A15_beam_full_sealed_version.json');require((seal['run'],seal['version'],seal['source_files'],seal['critical_tools'])==(run,'v18a',889,19),'version reuse metadata mismatch')
 require(seal['source_reused_from_run']=='A14_beam_full' and seal['source_manifest_sha256']==sha(T/'assisted_manifest.json') and seal['tools_manifest_sha256']==sha(T/'assisted_tools_manifest.json'),'version reuse not exact')
 require(len(seal['archives'])==2 and seal['archives'][0]==read('A14_beam_full_sealed_version.json')['archives'][0],'reuse exact v18 source archive; new19-tool archive required')
 for v in seal['archives']:
  q=P/'versions'/Path(v['path']).name;require(q.stat().st_size==v['bytes'] and sha(q)==v['sha256'],'original v18 archive changed')
 summary=dict(passed=True,source_unchanged=True,critical_monitor_math_unchanged=True,source_count=889,critical_tool_count=19,tool_delta='watcher STOP publication only plus shared publisher',reference_gate=bound(priorpath),reference_source_after=bound('runs/A14_beam_full/source_after.json'),source_checks_reused=379,cached_cases_reused=600,original_C01_cases_reused=5943,external_control_publisher=bound(T/'publish_runtime_control.py'),native_motion_fix_still_unvalidated=True)
 (P/'C03_exact_motion_evidence_reuse.json').write_text(json.dumps(summary,indent=2)+'\n')
 gate=copy.deepcopy(prior);gate.update(run_name=run,cycle='C03',workflow_version='W03.9',mandatory_unresolved=[])
 gate_tools_manifest=sha(T/'assisted_tools_manifest.json');gate['tools_manifest_sha256']=gate_tools_manifest
 common=[bound('C03_exact_motion_evidence_reuse.json'),bound('C03_control_publish_validation.json'),bound('C03_launch_gate_contract_tests.json'),bound('A15_beam_full_sealed_version.json'),bound(T/'publish_runtime_control.py'),bound(T/'watch_visible_meshes.py')]
 for row in gate['checks']:
  row['evidence']+=common;row['scope']+='; exact889 v18 motion source reuse,17 unchanged old critical tools plus STOP-only watcher and shared publisher (19 total), independently tested atomic startup/stop message publication'
 import sys
 sys.path.insert(0,str(T));from check_launch_gate import validate
 validation=validate(gate,run_name=run,source_manifest=T/'assisted_manifest.json',tools_manifest=T/'assisted_tools_manifest.json')
 out.write_text(json.dumps(gate,indent=2)+'\n');result=dict(passed=True,gate=bound(out),validation=validation,generator=bound(Path(__file__)),native_started=False)
 (P/'C03_launch_gate_validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':
 try:main()
 except (ValueError,KeyError,TypeError,OSError) as e:print(json.dumps(dict(passed=False,blocker=str(e),native_started=False)));raise SystemExit(2)
