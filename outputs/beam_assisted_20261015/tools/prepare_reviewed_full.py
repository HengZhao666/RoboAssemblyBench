"""Bind a positively screened candidate to one ordinary from-head attempt."""
from pathlib import Path
import argparse,copy,fcntl,json,subprocess,sys,tarfile
from beam_workflow import sha,save,validate_attempt_window

def prepare_joint_branch(args,p,t,w,read):
    """Reuse unchanged contracts; bind the new branch and limit observations."""
    q=read(w/'source_qualification.json');deploy=read(w/'deployment.json');binding=read(w/'authority_configuration_binding.json')
    probe_path=p/(args.probe_cycle+'_probe_review.json');probe=read(probe_path)
    boundary=probe['original_result']['assisted_boundary_stop']
    assert q['passed'] and q['qualification_kind']=='joint_branch_prealign' and deploy['deployed'] and deploy['version']=='v32' and binding['passed']
    assert probe['screened_lift_completed'] and probe['native_grasp_requalified'] and not probe['model_guard_rejections']
    assert probe['execution_evidence_passed'] and probe['complete_video_decode_passed'] and not probe['full_assembly_accepted']
    assert boundary['reason']=='isolated-full-clearance-screen-complete' and boundary['completed_substage']=='all_clearance'
    source_sha=sha(t/'assisted_manifest.json');tools_sha=sha(t/'assisted_tools_manifest.json');source=read(t/'assisted_manifest.json')
    assert source_sha==deploy['source_manifest_sha256']==binding['current_source_manifest_sha256'] and tools_sha==deploy['tools_manifest_sha256']
    assert source==q['candidate_files'] and len(source)==889 and all(sha(p/'assisted_repository'/n)==h for n,h in source.items())
    assert all(sha(t/n)==h for n,h in read(t/'assisted_tools_manifest.json').items())
    assert sha(probe_path)==deploy['positive_probe_sha256']
    after=read(p/'runs'/probe['run']/'source_after.json');assert not after['differences'] and not after.get('tool_differences')
    assert {r['path']:r['sha256'] for r in after['files']}==source
    for e in q['evidence'].values():assert sha(e['path'])==e['sha256']
    for n,h in binding['input_sha256'].items():assert sha(n)==h
    prefix=read(w/'unchanged_prefix_joint_limits.json');assert prefix['passed'] and prefix['actual_published_prefix'] and prefix['through_step']==30987 and not prefix['native_started']
    assert prefix['native_limits_recorded_run']==probe['run'] and not prefix['violations'] and all(prefix['commands_checked'].values())
    for n,h in prefix['input_sha256'].items():assert sha(n)==h
    anchor=read(w/'A37_joint_anchor_audit.json');assert anchor['samples'] and not anchor['missing_native_hand_samples']
    assert all(r['max_position']['position_error_m']<=.0001 and r['max_angle']['angle_error_rad']<=.0015 for r in anchor['phase_summary'].values())
    media=read(p/(args.probe_cycle+'_local_media_review.json'));assert media['passed'] and media['three_complete_originals_downloaded']
    originals_path=p/(args.probe_cycle+'_original_candidates_review.json');assert read(originals_path)['passed_finite_recorded_checks']
    historical=read(p/'D20_work/formal_gate_spec.json');checks=copy.deepcopy(historical['checks']);relocated=[]
    for check in checks.values():
        for e in check['reports']+check.get('files',[]):
            if sha(e['path'])!=e['sha256']:
                f=Path(e['path']);old=w/'before_tools'/f.name
                assert f.parent==t and old.is_file() and sha(old)==e['sha256'],str(f)
                relocated.append(dict(original=str(f),preserved=str(old),sha256=e['sha256']));e['path']=str(old)
    scope=('Explicit human total20-attempt window, preserving all spent attempts. v32 enables existing raw-branch until-arrival prealign only for remaining fourth/fifth pickups and rejects final float32 commands outside actual read-only native limits. Original whole-phase budgets, rates, tracking, lead, model, attachment, solver,54-self sweeps, insertion and final five-part release/240tick gates remain. Existing unchanged-domain SHA-bound evidence is reused. New actual-input production prealign replay and full54-self sweeps,528 related tests,4 exact projected configurations, original-record prefix limit audit and positively screened fourth capture/all-clearance are bound. The short scene was newly initialized, not a proven restoration, and does not certify from-head capture, insertion, fifth-part branch or full assembly. Fresh ordinary from-head runtime checks, three complete videos and original sequence/support reports are required.')
    certificate=w/'formal_reuse_certificate.json';assert not certificate.exists()
    save(certificate,dict(passed=True,source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualified_candidate_exact=True,four_layouts_exact=True,original_success_criteria_exact=True,positive_isolated_all_clearance=True,unchanged_prefix_native_limit_audit=True,checkpoint_used=False,full_assembly_accepted=False,historical_tool_relocations=relocated,scope=scope,tool_sha256=sha(__file__)))
    def report(path,flags):return dict(path=str(path),sha256=sha(path),assertions=[dict(pointer='/'+k,equals=v) for k,v in flags.items()])
    common=[report(w/'source_qualification.json',dict(passed=True,source_files=889,original_model_guard_exact=True,original_success_criteria_exact=True,new_affected_tests=528)),
        report(w/'authority_configuration_binding.json',dict(passed=True,candidate_and_authority_sources_exact=True,full_task_model_exact=True,all_four_authority_layouts_passed=True)),
        report(certificate,dict(passed=True,qualified_candidate_exact=True,four_layouts_exact=True,positive_isolated_all_clearance=True,checkpoint_used=False)),
        report(probe_path,dict(postprocessing_completed=True,native_grasp_requalified=True,execution_evidence_passed=True,complete_video_decode_passed=True,full_assembly_accepted=False)),
        report(w/'unchanged_prefix_joint_limits.json',dict(passed=True,actual_published_prefix=True,native_started=False)),
        report(p/(args.probe_cycle+'_local_media_review.json'),dict(passed=True,three_complete_originals_downloaded=True))]
    extra=[w/'deployment.json',p/(args.run+'_sealed_version.json'),w/'qualification_tools.tar.gz',w/'A37_joint_anchor_audit.json',
        p/(args.probe_cycle+'_retrospective.json'),originals_path,t/'prepare_reviewed_full.py',t/'qualify_assisted_candidate.py',t/'compare_assisted_configuration.py',t/'replay_assisted_transition.py',t/'audit_assisted_motion.py']
    extra.extend(Path(e['path']) for e in q['evidence'].values())
    for name,check in checks.items():
        check['scope']=name+': '+scope;check['reports'].extend(common);check.setdefault('files',[]).extend(dict(path=str(f),sha256=sha(f)) for f in extra)
    bounded=copy.deepcopy(historical['bounded_primary_scene_probe']);assert bounded['helper_sha256']==sha(t/'bounded_scene_http_probe.py')
    spec=w/'formal_gate_spec.json';assert not spec.exists()
    save(spec,dict(run_name=args.run,cycle=args.cycle,workflow_version='W05.5',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,actual_configuration_signature=binding['authority_configuration_signature'],changed_factors=['Raw-branch existing prealign completion for remaining fourth/fifth pickups','Read-only actual native limits checked after final publication refinement'],bounded_primary_scene_probe=bounded,checks=checks))
    subprocess.run([sys.executable,str(t/'build_delta_gate.py'),str(spec)],check=True)
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);ledger=p/'repair_batch_D_20261016.json';batch=read(ledger);validate_attempt_window(batch,args.run,args.cycle)
        started=batch['native_attempts_started'];assert len(batch['cycles'])==started and args.run==deploy['run']
        save(w/'before_formal_ready_ledger.json',batch)
        batch['cycles'].append(dict(id=args.cycle,planned_run=args.run,state='qualified',native_started=False,source_version='v32',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualification_path=str(w/'source_qualification.json'),gate_sha256=sha(p/'launch_gates'/(args.run+'.json')),checkpoint_used=False,hypothesis='Raw native branch prealign prevents the measured negative wrist hard-stop while actual final-target native-limit guard rejects illegal winding before publication. Verify original full sequence.'))
        batch.update(current_cycle=args.cycle,status=args.cycle+'_ready',current_source_version='v32',next_native_run_authorized=True,pending_next_step='Sync explicit documents and bind receipt, then ordinary from-head verification of the positively screened candidate.',final_document_sync_status='pending',final_document_sync_expected='document_sync/beam_assisted_'+args.cycle+'_ready_20261020/server_receipt.json')
        batch.pop('documents_sync_receipt',None);save(ledger,batch)
    print(json.dumps(dict(passed=True,ready=args.run,closed_before=started,total_cap=batch['maximum_native_attempts'],native_started=False)))

def prepare_reused_candidate(args,p,t,w,read):
    """Rebind unchanged qualified product source after an authorized restart."""
    prior=args.reuse_qualified_work.resolve();q=read(prior/'source_qualification.json');binding=read(prior/'authority_configuration_binding.json')
    assert q['passed'] and binding['passed'] and q['original_model_guard_exact'] and q['original_success_criteria_exact']
    source=read(t/'assisted_manifest.json');source_sha=sha(t/'assisted_manifest.json');tools_sha=sha(t/'assisted_tools_manifest.json')
    assert source==q['candidate_files'] and source_sha==binding['current_source_manifest_sha256']
    assert len(source)==889 and all(sha(p/'assisted_repository'/n)==h for n,h in source.items())
    assert all(sha(t/n)==h for n,h in read(t/'assisted_tools_manifest.json').items())
    assert all(sha(e['path'])==e['sha256'] for e in q['evidence'].values())
    assert all(sha(n)==h for n,h in binding['input_sha256'].items())
    operations=read(w/'operations_qualification.json');assert operations['passed'] and not operations['native_started']
    assert all(sha(t/n)==h for n,h in operations['tool_sha256'].items())
    sealed=read(p/(args.run+'_sealed_version.json'));assert sealed['source_manifest_sha256']==source_sha and sealed['tools_manifest_sha256']==tools_sha
    historical=read(prior/'formal_gate_spec.json');checks=copy.deepcopy(historical['checks']);relocated=[]
    for check in checks.values():
        for e in check['reports']+check.get('files',[]):
            if sha(e['path'])!=e['sha256']:
                f=Path(e['path']);old=w/'before_tools'/f.name
                assert f.parent==t and old.is_file() and sha(old)==e['sha256'],str(f)
                relocated.append(dict(original=str(f),preserved=str(old),sha256=e['sha256']));e['path']=str(old)
    certificate=w/'formal_reuse_certificate.json';assert not certificate.exists()
    scope='Explicit human total20-attempt window including8 already spent, no count reset. Product source889 and four-layout configurations exactly equal v31 qualification. Only workflow authorization and bounded exact-URL network preflight/retries changed; original controller/physics/geometry/assets/success criteria unchanged. Old evidence is reused for its unchanged domain. v31 insertion dynamics, later parts and final assisted240tick remain unverified and require this ordinary from-head run; original runtime guards, same-clock three cameras, full54-self sweeps, failure5s tail and full originals remain.'
    save(certificate,dict(passed=True,qualified_source_exact=True,original_success_criteria_exact=True,four_layouts_exact=True,source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,operations_qualification_passed=True,native_started=False,full_assembly_accepted=False,historical_tool_relocations=relocated,scope=scope))
    def report(path,flags):return dict(path=str(path),sha256=sha(path),assertions=[dict(pointer='/'+k,equals=v) for k,v in flags.items()])
    reports=[report(certificate,dict(passed=True,qualified_source_exact=True,four_layouts_exact=True,original_success_criteria_exact=True,native_started=False)),report(w/'operations_qualification.json',dict(passed=True,native_started=False,source_or_physics_changed=False))]
    extra=[p/(args.run+'_sealed_version.json'),t/'beam_workflow.py',t/'launch_reviewed_run.py',t/'bounded_scene_http_probe.py',t/'prepare_reviewed_full.py',w/'auxiliary_tools.tar.gz']
    for check in checks.values():
        check['scope']=scope;check['reports'].extend(reports);check.setdefault('files',[]).extend(dict(path=str(f),sha256=sha(f)) for f in extra)
    bounded=copy.deepcopy(historical['bounded_primary_scene_probe']);bounded['helper_sha256']=sha(t/'bounded_scene_http_probe.py')
    spec=w/'formal_gate_spec.json';assert not spec.exists()
    save(spec,dict(run_name=args.run,cycle=args.cycle,workflow_version='W05.5',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,actual_configuration_signature=binding['authority_configuration_signature'],changed_factors=['Human cap20, preserving spent8','Finite exact primary URL network preflight before native claim and runtime transport retries','Product source/configuration unchanged v31'],bounded_primary_scene_probe=bounded,checks=checks))
    subprocess.run([sys.executable,str(t/'build_delta_gate.py'),str(spec)],check=True)
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ledger=p/'repair_batch_D_20261016.json';batch=read(ledger);validate_attempt_window(batch,args.run,args.cycle)
        started=batch['native_attempts_started'];assert len(batch['cycles'])==started
        save(w/'before_ready_ledger.json',batch)
        batch['cycles'].append(dict(id=args.cycle,planned_run=args.run,state='qualified',native_started=False,source_version='v31',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualification_path=str(prior/'source_qualification.json'),gate_sha256=sha(p/'launch_gates'/(args.run+'.json')),checkpoint_used=False,hypothesis='Native consecutive COM motion admission removes solver-velocity false hold with original thresholds; verify from-head under unchanged v31 product source.'))
        batch.update(current_cycle=args.cycle,status=args.cycle+'_ready',next_native_run_authorized=True,pending_next_step='Sync current explicit documents, bind receipt, network preflight before native claim, then ordinary from-head validation.',final_document_sync_status='pending',final_document_sync_expected='document_sync/beam_assisted_'+args.cycle+'_ready_20261020/server_receipt.json')
        batch.pop('documents_sync_receipt',None);save(ledger,batch)
    print(json.dumps(dict(passed=True,ready=args.run,closed_before=started,total_cap=batch['maximum_native_attempts'],native_started=False)))

def prepare_com_metric(args,p,t,w,read):
    assert args.run=='A35_beam_full' and args.cycle=='D19'
    q=read(w/'source_qualification.json');deploy=read(w/'deployment.json');binding=read(w/'authority_configuration_binding.json')
    assert q['passed'] and q['qualification_kind']=='descent_com_metric' and deploy['deployed'] and deploy['version']=='v31' and binding['passed']
    source_sha=sha(t/'assisted_manifest.json');tools_sha=sha(t/'assisted_tools_manifest.json')
    assert source_sha==deploy['source_manifest_sha256']==binding['current_source_manifest_sha256'] and tools_sha==deploy['tools_manifest_sha256']
    source=read(t/'assisted_manifest.json');assert source==q['candidate_files'] and len(source)==889
    assert all(sha(p/'assisted_repository'/n)==h for n,h in source.items()) and all(sha(t/n)==h for n,h in read(t/'assisted_tools_manifest.json').items())
    for e in q['evidence'].values():assert sha(e['path'])==e['sha256']
    for n,h in binding['input_sha256'].items():assert sha(n)==h
    prior=read(p/'D18_conclusions.json');pipeline=read(p/'D18_postprocess_pipeline.json');media=read(p/'D18_local_media_review.json')
    assert prior['full_from_head'] and prior['complete_originals_downloaded']==3 and media['passed']
    assert pipeline['all_required_stages_passed'] and pipeline['evidence_checks_passed'] and pipeline['support_original_report_passed']
    historical=read(p/'D17_work/formal_gate_spec.json');checks=copy.deepcopy(historical['checks'])
    # All previous evidence stays immutable. This explicitly narrows its scope:
    # the changed admission semantics are qualified by new actual-input checks.
    for check in checks.values():
        for e in check['reports']+check.get('files',[]):assert sha(e['path'])==e['sha256'],e['path']
    scope=('Last of eight explicitly authorized new native attempts. v31 changes only the insertion motion signal to the existing consecutive native COM estimator; original speed, alignment,12tick, model, full54/self, release and assisted five-part240tick gates remain. v30 actual from-head carry passed, and unchanged carry/physics/attachment/geometry/success interfaces reuse prior SHA-bound evidence. New40 insertion contracts were checked in four actual layouts; actual old-control observations qualify12ticks and the production adapter/final publication progresses in the same measured aligned input. Static nonlinear FK/IK replay is not dynamics; no new-trajectory insertion or later-part success is claimed. Fresh ordinary from-head execution, guards and original reports/three complete videos are mandatory. Installation world supports remain part of the assisted contract.')
    reuse_path=w/'formal_reuse_certificate.json';assert not reuse_path.exists()
    save(reuse_path,dict(passed=True,current_source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualified_candidate_exact=True,authority_four_layouts_exact=True,old_bound_evidence_SHA_verified=True,original_success_criteria_exact=True,new_affected_tests=152,reused_other_tests=364,unique_related_tests=516,actual_native_stream_12_tick_positive=True,actual_input_final_command_replay_full54_self=True,prior_full_clearance_completed=True,checkpoint_used=False,full_assembly_accepted=False,scope=scope,tool_sha256=sha(__file__)))
    def report(path,flags):return dict(path=str(path),sha256=sha(path),assertions=[dict(pointer='/'+k,equals=v) for k,v in flags.items()])
    common=[report(w/'source_qualification.json',dict(passed=True,source_files=889,original_model_guard_exact=True,original_success_criteria_exact=True,new_affected_tests=152,actual_native_stream_12_tick_positive=True)),
        report(w/'authority_configuration_binding.json',dict(passed=True,candidate_and_authority_sources_exact=True,full_task_model_exact=True,all_four_authority_layouts_passed=True)),
        report(reuse_path,dict(passed=True,current_source_manifest_sha256=source_sha,qualified_candidate_exact=True,checkpoint_used=False)),
        report(w/'candidate_admission_commands.json',dict(passed=True,descent_admission_observed=True,full54=True,self_checked=True,native_physics=False)),
        report(w/'native_admission_replay.json',dict(passed=True,native_samples=973,legacy_default_exact_cases=448,native_started=False)),
        report(p/'D18_postprocess_pipeline.json',dict(all_required_stages_passed=True,evidence_checks_passed=True,support_original_report_passed=True)),
        report(p/'D18_local_media_review.json',dict(passed=True,three_complete_originals_downloaded=True))]
    extra=[w/'deployment.json',w/'operation_deployment.json',p/(args.run+'_sealed_version.json'),w/'v31_auxiliary_tools.tar.gz',t/'bounded_scene_http_probe.py',p/'D18_conclusions.json',p/'D18_retrospective.json',p/'D18_original_candidates_review.json',p/'D17_work/A34_joint_anchor_audit.json',t/'qualify_assisted_candidate.py',t/'compare_assisted_configuration.py',t/'prepare_reviewed_full.py',t/'replay_assisted_transition.py',t/'replay_assisted_motion_gate.py']
    extra.extend(Path(e['path']) for e in q['evidence'].values())
    for name,check in checks.items():
        check['scope']=name+': '+scope;check['reports'].extend(common)
        check.setdefault('files',[]).extend(dict(path=str(f),sha256=sha(f)) for f in extra)
    bounded=copy.deepcopy(historical['bounded_primary_scene_probe']);assert bounded['helper_sha256']==sha(t/'bounded_scene_http_probe.py')
    spec=w/'formal_gate_spec.json';assert not spec.exists()
    save(spec,dict(run_name=args.run,cycle=args.cycle,workflow_version='W05.5',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,actual_configuration_signature=binding['authority_configuration_signature'],changed_factors=['Insertion motion measurement uses existing consecutive native COM estimator; original limits unchanged'],bounded_primary_scene_probe=bounded,checks=checks))
    subprocess.run([sys.executable,str(t/'build_delta_gate.py'),str(spec)],check=True)
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        ledger_path=p/'repair_batch_D_20261016.json';batch=read(ledger_path)
        assert batch['native_attempts_started']==batch['completed_cycles']==18 and batch['maximum_native_attempts']==19 and len(batch['cycles'])==18
        save(w/'before_formal_ready_ledger.json',batch)
        with tarfile.open(w/'before_formal_ready_documents.tar.gz','x:gz') as tf:
            for n in read(p.parent/'beam_routes/docs_manifest.json')['files']:tf.add(p.parent/n,arcname=n,recursive=False)
        batch['cycles'].append(dict(id=args.cycle,planned_run=args.run,state='qualified',native_started=False,source_version='v31',source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualification_sha256=sha(w/'source_qualification.json'),gate_sha256=sha(p/'launch_gates'/(args.run+'.json')),hypothesis='Use measured consecutive native COM rates to remove false insertion hold caused by solver angular velocity, retaining all original limits.',checkpoint_used=False,acceptance='Original five-part assisted assembly, gripper release, final240native ticks and all sequence/safety gates.'))
        batch.update(current_cycle=args.cycle,status='D19_ready',next_native_run_authorized=True,current_source_version='v31',pending_next_step='Sync explicit27 ready documents, bind receipt, then last authorized ordinary from-head validation.',final_document_sync_status='pending',final_document_sync_expected='document_sync/beam_assisted_D19_ready_20261020/server_receipt.json')
        batch.pop('documents_sync_receipt',None);validate_attempt_window(batch,args.run,args.cycle);save(ledger_path,batch)
    print(json.dumps(dict(passed=True,ready=args.run,counts=[18,18],cap=19,native_started=False)))

ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('probe_cycle');ap.add_argument('--descent-com-metric',action='store_true');ap.add_argument('--reuse-qualified-work',type=Path);ap.add_argument('--joint-branch-prealign',action='store_true');args=ap.parse_args()
p=Path(__file__).resolve().parents[1];t=p/'tools';w=args.work.resolve();read=lambda f:json.loads(Path(f).read_text())
if args.joint_branch_prealign:
    prepare_joint_branch(args,p,t,w,read);raise SystemExit(0)
if args.reuse_qualified_work:
    prepare_reused_candidate(args,p,t,w,read);raise SystemExit(0)
if args.descent_com_metric:
    prepare_com_metric(args,p,t,w,read);raise SystemExit(0)
q=read(w/'source_qualification.json');deploy=read(w/'deployment.json');binding=read(w/'authority_configuration_binding.json')
probe_path=p/(args.probe_cycle+'_probe_review.json');probe=read(probe_path)
boundary=probe['original_result']['assisted_boundary_stop']
assert q['passed'] and deploy['deployed'] and binding['passed']
assert probe['screened_lift_completed'] and probe['native_grasp_requalified'] and not probe['model_guard_rejections']
assert boundary['reason']=='isolated-full-clearance-screen-complete' and boundary['completed_substage']=='all_clearance'
assert probe['execution_evidence_passed'] and probe['complete_video_decode_passed']
source_sha=sha(t/'assisted_manifest.json');tools_sha=sha(t/'assisted_tools_manifest.json')
assert source_sha==deploy['source_manifest_sha256']==binding['current_source_manifest_sha256']
assert tools_sha==deploy['tools_manifest_sha256']
source=read(t/'assisted_manifest.json');assert source==q['candidate_files'] and len(source)==889
assert all(sha(p/'assisted_repository'/n)==h for n,h in source.items())
assert all(sha(t/n)==h for n,h in read(t/'assisted_tools_manifest.json').items())
for e in q['evidence'].values():assert sha(e['path'])==e['sha256']
for n,h in binding['input_sha256'].items():assert sha(n)==h
assert sha(probe_path)==deploy['positive_probe_sha256']
assert (p/(args.probe_cycle+'_retrospective.json')).exists()
originals_path=p/(args.probe_cycle+'_original_candidates_review.json');assert read(originals_path)['passed_finite_recorded_checks']
adjacent=read(w/'adjacent_control_replay.json')
assert adjacent['passed'] and adjacent['adjacent_counterfactual'] and adjacent['full54'] and adjacent['self_checked'] and adjacent['guard_calls']>=27
assert adjacent['source_manifest']==source and adjacent['actual_recorded_run']==probe['run']

scope=('Explicit eight-attempt human grant; short probes count. v30 preserves the accepted SE3 drive target, '
       'freezes phase-entry load bias separately from transient lookahead, caps each staged endpoint, and raises '
       'only fourth-part position-solver iterations16to64. Original grasp/model/full54-self/strict-insertion/release/'
       'five-part240-tick success gates remain. Original physics/attachment implementation, assets, camera, '
       'right-support and unflagged control source bytes or AST are reused only for their unchanged contracts. '
       'New whole-clearance isolated physics screen passed; it is a newly initialized episode and does not '
       'certify restored equivalence, original capture state, later insertion, fifth part or full assembly. '
       'Fresh from-head execution and all original runtime guards/reports/three complete videos remain required. '
       'The same primary scene URL uses bounded HTTP existence checking; original USD loader/configuration remains.')
historical=read(p/'D11_work/gate_spec.json');checks=copy.deepcopy(historical['checks']);relocations=[]
for check in checks.values():
    for e in check['reports']+check.get('files',[]):
        if sha(e['path'])!=e['sha256']:
            path=Path(e['path']);backup=w/'before_authority'/path.name
            assert path.parent==t and backup.is_file() and sha(backup)==e['sha256']
            relocations.append(dict(original_path=str(path),preserved_path=str(backup),sha256=e['sha256']));e['path']=str(backup)
        assert sha(e['path'])==e['sha256']
reuse_path=w/'formal_reuse_certificate.json';assert not reuse_path.exists()
save(reuse_path,dict(passed=True,current_source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,
    qualified_candidate_exact=True,authority_four_layouts_exact=True,old_bound_evidence_SHA_verified=True,
    original_success_criteria_exact=True,old_487_tests_reused_only_unchanged_domain=True,new_affected_tests=130,
    actual_input_final_command_replay_full54_self=True,original_over_limit_rejected_before_IK_CAD=True,
    positive_isolated_all_clearance=True,checkpoint_used=False,full_assembly_accepted=False,
    historical_evidence_relocations=relocations,scope=scope,tool_sha256=sha(__file__)))

def report(path,flags):
    return dict(path=str(path),sha256=sha(path),assertions=[dict(pointer='/'+k,equals=v) for k,v in flags.items()])
common=[report(w/'source_qualification.json',dict(passed=True,source_files=889,original_model_guard_exact=True,original_success_criteria_exact=True,new_affected_tests=130)),
    report(w/'authority_configuration_binding.json',dict(passed=True,candidate_and_authority_sources_exact=True,full_task_model_exact=True,all_four_authority_layouts_passed=True)),
    report(reuse_path,dict(passed=True,current_source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,qualified_candidate_exact=True,positive_isolated_all_clearance=True,checkpoint_used=False)),
    report(probe_path,dict(postprocessing_completed=True,native_grasp_requalified=True,execution_evidence_passed=True,complete_video_decode_passed=True,full_assembly_accepted=False)),
    report(w/'adjacent_control_replay.json',dict(passed=True,adjacent_counterfactual=True,full54=True,self_checked=True,native_physics=False))]
specific={
    'actual_configuration_and_targets':'Four actual production layout models and all original phase targets/success criteria newly compared after deployment.',
    'full_phase_release_completion_interfaces':'All98 phase contracts and unchanged interpreter/release/final predicates retain prior source/config evidence; native later phases remain unvisited by the short screen.',
    'full_entities_and_camera_visibility':'Original complete both-arm/gripper/payload54 geometry, camera frame mapping and asset identity retained. Fresh native initialization/three actual startup images and all runtime sweeps required.',
    'production_controller_object_space_replay':'Actual A27 input passes production Adapter/final clamp/both-arm54-self sweep; dangerous original residual is rejected. Native short scene additionally completed four clearance substages.',
    'admitted_error_and_final_command_geometry':'Frozen capture and original100um/1.5mrad, lead and joint limits retained; independent native-anchor audit and original runtime full-sweep evidence bound. New capture/path domains must pass fresh runtime guards.',
    'bimanual_support_lifecycle':'Unflagged right-support controller, final-target retention and original completion predicates unchanged. Original right support was actually requalified under solver64 in the short screen; full from-head support/release/retract still required.'}
extra=[w/'deployment.json',w/'operation_deployment.json',w/'ops_candidate/ops_qualification.json',p/(args.run+'_sealed_version.json'),
    w/'v30_auxiliary_tools.tar.gz',t/'bounded_scene_http_probe.py',w/'terminal_cap_reuse_certificate.json',p/(args.probe_cycle+'_retrospective.json'),
    originals_path,w/'A33_joint_anchor_audit.json',w/'before_formal_preparation_tools/qualify_assisted_candidate.py',w/'before_formal_preparation_tools/compare_assisted_configuration.py']
for e in q['evidence'].values():extra.append(Path(e['path']))
for name,check in checks.items():
    check['scope']=specific[name]+' '+scope
    check['reports'].extend(common)
    check.setdefault('files',[]).extend(dict(path=str(f),sha256=sha(f)) for f in extra)
profile=read(w/'probe_terminal_cap_full/isolated_fixture.json')['bounded_primary_scene_probe']
bounded={k:profile[k] for k in ['url','sha256']};bounded['helper_sha256']=sha(t/'bounded_scene_http_probe.py')
spec=w/'formal_gate_spec.json';assert not spec.exists()
save(spec,dict(run_name=args.run,cycle=args.cycle,workflow_version='W05.5',source_manifest_sha256=source_sha,
    tools_manifest_sha256=tools_sha,actual_configuration_signature=binding['authority_configuration_signature'],
    changed_factors=['Fourth-part incremental phase-entry load-bias and bounded terminal drive','Only part1 position solver iterations64','Bounded existence transport for identical primary scene URL'],
    bounded_primary_scene_probe=bounded,checks=checks))
subprocess.run([sys.executable,str(t/'build_delta_gate.py'),str(spec)],check=True)
with (p/'beam_run.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    ledger_path=p/'repair_batch_D_20261016.json';batch=read(ledger_path)
    assert batch['native_attempts_started']==batch['completed_cycles']==17 and batch['maximum_native_attempts']==19
    assert args.cycle=='D18' and args.run=='A34_beam_full' and len(batch['cycles'])==17
    save(w/'before_formal_ready_ledger.json',batch)
    with tarfile.open(w/'before_formal_ready_documents.tar.gz','x:gz') as tf:
        for n in read(p.parent/'beam_routes/docs_manifest.json')['files']:tf.add(p.parent/n,arcname=n,recursive=False)
    batch['cycles'].append(dict(id=args.cycle,planned_run=args.run,state='qualified',native_started=False,
        source_version=deploy['version'],source_manifest_sha256=source_sha,tools_manifest_sha256=tools_sha,
        qualification_sha256=sha(w/'source_qualification.json'),gate_sha256=sha(p/'launch_gates'/(args.run+'.json')),
        hypothesis='Native fixed-joint convergence plus continuous bounded final-drive endpoints will pass fourth-part clearance in original from-head sequence.',
        checkpoint_used=False,acceptance='All five original assisted assembly/release/final240tick and sequence/safety gates.'))
    batch.update(current_cycle=args.cycle,status=args.cycle+'_ready',next_native_run_authorized=True,current_source_version=deploy['version'],
        pending_next_step='Synchronize explicit27 ready documents, bind receipt, then ordinary from-head full validation.',
        final_document_sync_status='pending',final_document_sync_expected='document_sync/beam_assisted_D18_ready_20261020/server_receipt.json')
    batch.pop('documents_sync_receipt',None);validate_attempt_window(batch,args.run,args.cycle);save(ledger_path,batch)
print(json.dumps(dict(passed=True,ready=args.run,counts=[17,17],cap=19,configuration_signature=binding['authority_configuration_signature'],native_started=False)))
