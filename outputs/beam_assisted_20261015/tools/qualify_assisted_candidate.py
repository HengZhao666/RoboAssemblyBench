"""Reusable affected-source qualification and atomic, source-bound deployment."""
from pathlib import Path
import argparse,ast,fcntl,json,os,shutil,subprocess,sys,tarfile
from beam_workflow import sha,save

def qualify_final_home_wrist_roll(args,p,t,a,w,c,read,baseline,source):
    """One human-authorized full attempt; qualify new empty-return/camera domain."""
    module=read(w/'affected_module_result.json');cfg=read(w/'configuration_delta.json')
    replay=read(w/'final_home_control_replay.json');geometry=read(w/'return_geometry_delta024.json')
    authorization=read(w/'single_round_authorization_checks.json')
    review=read(w/'prefix_and_tool_review.json')
    changed=[n for n in source if source[n]!=baseline[n]]
    expected={'roboassemblybench/core/beam_assisted_compiler.py',
        'roboassemblybench/tasks/fabrica_beam_ur5e_assisted/recipe.yaml',
        'toolkits/factory_dual_franka_assembly/beam_assisted_camera.py',
        'toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py',
        'toolkits/factory_dual_franka_assembly/beam_assisted_state.py',
        'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py',
        'tests/toolkits/test_beam_assisted_camera_contract.py',
        'tests/toolkits/test_beam_assisted_object_servo.py',
        'tests/toolkits/test_beam_assisted_preflight.py',
        'tests/toolkits/test_beam_assisted_release_state.py'}
    assert len(source)==889 and set(changed)==expected
    assert review['passed'] and review['candidate_files']==source and len(review['checks'])>=48
    assert all(review['checks'].values()) and all(v['passed'] for v in review['numeric_assertions'])
    assert all(v['passed'] for v in review['scope_results'].values())
    assert module['passed'] and module['candidate_files']==source
    assert module['counts']=={'passed':module['collected']}
    unique_tests=module['collected'];reused_tests=0
    if module['collected']<300:
        prior_dir=w/'validation_errors/stale_phase_count'
        prior=read(prior_dir/'affected_module_result.json')
        key='tests/toolkits/test_beam_assisted_preflight.py'
        assert prior['counts']=={'passed':415,'failed':1} and prior['collected']==416
        assert [n for n in source if source[n]!=prior['candidate_files'][n]]==[key]
        old_test=(prior_dir/'test_beam_assisted_preflight.py').read_text()
        assert old_test.count("phase_count']==98")==1
        assert old_test.replace("phase_count']==98","phase_count']==101")== (c/key).read_text()
        assert module['test_files']==['test_beam_assisted_preflight.py']
        unique_tests=416;reused_tests=416-module['collected']
        reuse=dict(passed=True,unique_related_tests=unique_tests,newly_rerun_tests=module['collected'],
            unchanged_other_module_tests_reused=reused_tests,original_415_pass_1_fail_preserved=True,
            only_private_phase_count_expectation_changed=True,production_sources_exact=True,
            prior_result_sha256=sha(prior_dir/'affected_module_result.json'),
            old_test_sha256=sha(prior_dir/'test_beam_assisted_preflight.py'),
            current_result_sha256=sha(w/'affected_module_result.json'))
        path=w/'related_suite_reuse_certificate.json'
        if path.exists():assert read(path)==reuse
        else:save(path,reuse)
    assert cfg['passed'] and cfg['actual_layouts']==4 and cfg['final_home']
    assert cfg['unchanged_prefix_and_original_final_gates'] and cfg['wrist_camera_configuration_exact']
    assert all(x['whole_config_exact_after_declared_projection'] and x['success_criteria_exact'] for x in cfg['layout_checks'])
    assert replay['passed'] and replay['source_manifest']==source and not replay['native_physics']
    assert replay['actual_recorded_run']=='A42_beam_full' and replay['actual_input_step']==57408
    assert all(replay[k] for k in ['production_entry_chain_verified','full54','self_checked',
        'all_three_return_phases_completed','native_completion_gate_checked',
        'phase_entry_delta_frozen','unsafe_counterexample_rejected_before_publication'])
    assert replay['guard_calls']>0 and replay['positive']['mesh_samples']==5*replay['positive']['guard_calls']
    assert replay['unsafe_counterexample']['guard_calls']>=1 and replay['unsafe_counterexample']['published_batches']==0
    assert not replay['geometry_failures'] and all(v>0 for v in replay['nonideal_seed_tracking_delta_rad'].values())
    assert geometry['delta_shoulder_lift_rad']==-.24 and geometry['complete_visible_meshes']==54
    assert geometry['native_body_poses']==44 and geometry['self_checked'] and geometry['intended_grasp'] is None
    assert len(geometry['segments'])==3 and sum(x['samples'] for x in geometry['segments'])==512
    assert all(x['passed'] and not x['first_collision'] for x in geometry['segments'])
    assert authorization['passed'] and len(authorization['cases'])==14 and all(x['passed'] for x in authorization['cases'])
    def functions(path):
        tree=ast.parse(path.read_text())
        return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(tree)
                if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))}
    protected={}
    # Only the declared opt-in functions may change. The original attachment,
    # installation supports, held-part guard and non-return adapter stay covered
    # by the related domain tests and exact old configuration projection.
    domains={
      'toolkits/factory_dual_franka_assembly/beam_assisted_state.py':{
        'bounded_native_target','_assisted_installation_sample','_assisted_conditions_stable',
        '_assisted_final_success_ready','_assisted_guard_actions'},
      'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py':{'_move_arm_to_joint_positions_action'},
      'toolkits/factory_dual_franka_assembly/beam_assisted_camera.py':{'update'}}
    for name,allowed in domains.items():
        old=functions(a/name);new=functions(c/name)
        unexpected=[k for k,v in old.items() if k not in allowed and new.get(k)!=v]
        protected[name]=not unexpected
        assert not unexpected,(name,unexpected)
    assert source['toolkits/factory_dual_franka_assembly/beam_assisted_servo.py']==baseline['toolkits/factory_dual_franka_assembly/beam_assisted_servo.py']
    assert source['internutopia/core/runner.py']==baseline['internutopia/core/runner.py']
    old_tools=read(w/'before_tools/assisted_tools_manifest.json')
    current_tools={n:sha(t/n) for n in old_tools}
    tool_changes=[n for n in current_tools if current_tools[n]!=old_tools[n]]
    assert set(tool_changes)=={'beam_workflow.py','assisted_worker.py','visual_mesh_check.py'}
    for name,allowed in [('visual_mesh_check.py',set()),('assisted_worker.py',{'act','production_cad_guard'})]:
        old=functions(w/'before_tools'/name);new=functions(t/name)
        unexpected=[k for k,v in old.items() if k not in allowed and new.get(k)!=v]
        # Worker top-level exception dispatch is reviewed separately; function
        # comparison preserves all existing named helper definitions.
        protected[name]=not unexpected
        assert not unexpected,(name,unexpected)
    names=['affected_module_result.json','configuration_delta.json','base_configuration_identity.json',
        'final_home_control_replay.json','return_geometry_delta024.json','return_geometry_final.json',
        'single_round_authorization_checks.json','prefix_and_tool_review.json']
    if reused_tests:names+=['related_suite_reuse_certificate.json','validation_errors/stale_phase_count/affected_module_result.json','validation_errors/stale_phase_count/test_beam_assisted_preflight.py']
    report=dict(passed=True,qualification_kind='final_home_wrist_roll',candidate_files=source,
        source_files=889,changed_files=changed,unchanged_source_files=879,
        critical_tool_changes=tool_changes,critical_tools=current_tools,
        new_affected_tests=module['collected'],unique_related_tests=unique_tests,
        reused_other_module_tests=reused_tests,protected_function_AST_exact=protected,
        actual_four_layouts_checked=True,original_success_criteria_exact=True,
        original_model_and_grasp_servo_exact=True,physical_attachment_functions_exact=True,
        production_final_commands_full54_self_checked=True,
        native_started=False,full_assembly_accepted=False,
        source_change_scope='180 degree optical-axis roll in actual wrist Camera update; three final empty-arm phases followed by original five-part240-tick window plus same-clock actual initial-joint qualification. Existing assembly sequence/configuration and success criteria retained. Each new final bounded batch checks both full arms/grippers and all five stationary parts with no grasp exemption.',
        limitations=['Finite visible-CAD proxy only; nominal return and rejected perturbation are retained separately.',
            'Full tracking-error box and continuous/cooked collision are unproved.',
            'Actual view orientation, dynamics, return completion and simultaneous240-tick native home/parts require the single full run.'],
        evidence={n:dict(path=str(w/n),sha256=sha(w/n)) for n in names},
        auxiliary_tools={n:sha(t/n) for n in ['check_assisted_candidate.py','compare_assisted_configuration.py',
            'replay_assisted_transition.py','test_beam_workflow.py']},tool_sha256=sha(__file__))
    if not args.deploy_version:
        assert not (w/'source_qualification.json').exists()
        save(w/'candidate_source_manifest.json',source);save(w/'source_qualification.json',report)
        print(json.dumps(dict(passed=True,changed_files=changed,tests=module['collected'])));return
    assert args.deploy_version=='v34' and args.for_run=='A43_beam_full'
    qualification=read(w/'source_qualification.json')
    assert qualification['candidate_files']==source and qualification['critical_tools']==current_tools
    assert all(sha(v['path'])==v['sha256'] for v in qualification['evidence'].values())
    assert not (w/'deployment.json').exists()
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        batch=read(p/'repair_batch_D_20261016.json');grant=batch['resumed_single_round_authorization']
        assert batch['native_attempts_started']==batch['completed_cycles']==26
        assert batch['maximum_native_attempts']==27 and batch['human_continuation_remains_active']
        assert grant['cycle']=='D27' and grant['run']=='A43_beam_full' and grant['additional_native_attempts']==1
        assert all(sha(a/n)==h for n,h in baseline.items())
        olddir=w/'before_authority';olddir.mkdir()
        for n in ['assisted_manifest.json','assisted_tools_manifest.json']:shutil.copy2(w/'before_tools'/n if (w/'before_tools'/n).exists() else t/n,olddir/n)
        save(olddir/'ledger.json',batch)
        for n in changed:
            backup=olddir/n;backup.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a/n,backup)
            temp=(a/n).with_name(Path(n).name+'.qualified-new');shutil.copy2(c/n,temp);os.replace(temp,a/n)
        assert all(sha(a/n)==h for n,h in source.items())
        subprocess.run([sys.executable,str(t/'seal_version.py'),args.deploy_version,args.for_run],check=True)
        assert read(t/'assisted_manifest.json')==source
        assert read(t/'assisted_tools_manifest.json')==current_tools
        save(w/'deployment.json',dict(deployed=True,version=args.deploy_version,run=args.for_run,
            source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),
            source_qualification_sha256=sha(w/'source_qualification.json'),native_started=False))
    print(json.dumps(read(w/'deployment.json')))

def qualify_staged_object(args,p,t,a,w,c,read,baseline,source):
    module=read(w/'affected_module_result.json');cfg=read(w/'configuration_delta.json')
    old=read(w/'baseline_control_replay.json');new=read(w/'candidate_control_replay.json')
    changed=[n for n in source if source[n]!=baseline[n]]
    allowed={'roboassemblybench/core/beam_assisted_compiler.py','tests/toolkits/test_beam_assisted_object_servo.py'}
    correction='toolkits/factory_dual_franka_assembly/beam_assisted_servo.py' in changed
    if correction:
        allowed.add('toolkits/factory_dual_franka_assembly/beam_assisted_servo.py')
        old_tree=ast.parse((a/'toolkits/factory_dual_franka_assembly/beam_assisted_servo.py').read_text())
        new_tree=ast.parse((c/'toolkits/factory_dual_franka_assembly/beam_assisted_servo.py').read_text())
        def without_turn(tree):
            for node in ast.walk(tree):
                if isinstance(node,ast.FunctionDef) and node.name=='incremental_clearance_command':
                    node.body=[]
            return ast.dump(tree,include_attributes=False)
        assert without_turn(old_tree)==without_turn(new_tree)
        turn=read(w/'recorded_turn_control_replay.json');stall=read(w/'recorded_turn_stall.json')
        assert turn['passed'] and turn['recorded_clearance_seed'] and turn['actual_recorded_run']=='A39_part0_staged_screen'
        assert turn['full54'] and turn['self_checked'] and not turn['geometry_failures']
        assert turn['final_publication_axial_delta_m']<-.0001
        assert stall['excess_over_height_m']>stall['original_position_tolerance_m'] and stall['native_rows']>=100
    assert set(changed)==allowed
    assert module['passed'] and module['candidate_files']==source and module['counts']['passed']==module['collected']>=167
    assert cfg['passed'] and cfg['actual_layouts']==4 and cfg['staged_object']==args.staged_object
    assert all(x['whole_config_exact_after_declared_projection'] and x['success_criteria_exact'] for x in cfg['layout_checks'])
    assert old['spatial_counterexample'] and old['minimum_right_payload_gap_m']<.005
    assert old['actual_recorded_run']=='A38_beam_full' and old['source_manifest']==baseline
    assert new['passed'] and new['geometric_path_completed'] and new['production_entry_chain_verified'] and not new['geometry_failures']
    assert new['full54'] and new['self_checked'] and new['minimum_right_payload_gap_m']>=.005
    assert new['source_manifest']==source and new['actual_recorded_run']=='A38_beam_full'
    names=['affected_module_result.json','configuration_delta.json','base_configuration_identity.json','baseline_control_replay.json','candidate_control_replay.json']
    if correction:names += ['recorded_turn_control_replay.json','recorded_turn_stall.json']
    coverage=[]
    actual=read(w/'actual_configuration_layout505_config.json');prior=read(w/'base_configuration_layout505_config.json')
    for phase in actual['phase_specs']:
        settings=phase.get('local_skill',{});old_phase=next(v for v in prior['phase_specs'] if v['name']==phase['name'])
        if settings.get('beam_assisted_staged_clearance'):
            coverage.append(dict(phase=phase['name'],robot=settings['robot'],object=settings['object'],common_implementation='beam_assisted_servo.staged_clearance_target',configuration_same_as_baseline=phase==old_phase,validation=('common turn correction affected; static replay/tests, native pending' if correction and settings.get('beam_assisted_incremental_clearance') else 'new affected replay; native pending' if settings['object']==args.staged_object else 'unchanged configuration and source; original A38 native result retained, not a new run')))
    report=dict(passed=True,qualification_kind='staged_clearance_object',object=args.staged_object,candidate_files=source,source_files=889,changed_files=changed,unchanged_source_files=889-len(changed),turn_surplus_correction=correction,new_affected_tests=module['collected'],original_model_guard_exact=True,original_success_criteria_exact=True,physical_attachment_code_exact=True,actual_four_layouts_checked=True,actual_input_final_command_replay_full54_self=True,full_assembly_accepted=False,native_started=False,common_mechanism_coverage=coverage,source_change_scope=('Common incremental turn only retracts surplus upward command against measured height; original full SE3, terminal cap, lead/rate/model/attachment/limits/success retained. ' if correction else '')+'Shared compiler branch enables existing staged path and retained final-drive reference only for fifth carry. Original solver/rates/limits/attachment/qualification/success unchanged; complete configuration equals baseline after three declared fifth-phase keys are projected. Five-mm visible gap is candidate screening, not a changed original success criterion or cooked-contact proof.',evidence={n:dict(path=str(w/n),sha256=sha(w/n)) for n in names},tool_sha256=sha(__file__))
    if not args.deploy_version:
        assert not (w/'source_qualification.json').exists();save(w/'candidate_source_manifest.json',source);save(w/'source_qualification.json',report);print(json.dumps(dict(passed=True,object=args.staged_object,tests=module['collected'],coverage=coverage)));return
    assert args.for_run and args.deploy_version.startswith('v')
    assert read(w/'source_qualification.json')['candidate_files']==source
    probe=read(p/(args.probe_cycle+'_probe_review.json'));assert probe['object']==args.staged_object and probe['screened_lift_completed'] and probe['native_grasp_requalified'] and not probe['model_guard_rejections'] and probe['execution_evidence_passed'] and probe['complete_video_decode_passed']
    assert probe['original_result']['assisted_boundary_stop']['completed_substage']=='all_clearance'
    after=read(p/'runs'/probe['run']/'source_after.json');assert not after['differences'] and not after.get('tool_differences') and {r['path']:r['sha256'] for r in after['files']}==source
    assert not (w/'deployment.json').exists()
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);batch=read(p/'repair_batch_D_20261016.json')
        assert batch['native_attempts_started']==batch['completed_cycles'] and batch['human_continuation_remains_active']
        assert all(sha(a/n)==h for n,h in baseline.items())
        olddir=w/'before_authority';olddir.mkdir();shutil.copy2(t/'assisted_manifest.json',olddir/'assisted_manifest.json');shutil.copy2(t/'assisted_tools_manifest.json',olddir/'assisted_tools_manifest.json')
        for n in changed:
            backup=olddir/n;backup.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a/n,backup)
            temp=(a/n).with_name(Path(n).name+'.qualified-new');shutil.copy2(c/n,temp);os.replace(temp,a/n)
        assert all(sha(a/n)==h for n,h in source.items())
        subprocess.run([sys.executable,str(t/'seal_version.py'),args.deploy_version,args.for_run],check=True)
        assert sha(t/'assisted_manifest.json')==sha(w/'candidate_source_manifest.json')
        save(w/'deployment.json',dict(deployed=True,version=args.deploy_version,run=args.for_run,source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),source_qualification_sha256=sha(w/'source_qualification.json'),positive_probe_sha256=sha(p/(args.probe_cycle+'_probe_review.json')),native_started=False))
    print(json.dumps(read(w/'deployment.json')))

def qualify_joint_branch(args,p,t,a,w,c,read,baseline,source):
    module=read(w/'affected_module_result.json');cfg=read(w/'configuration_delta.json')
    old=read(w/'baseline_control_replay.json');new=read(w/'candidate_control_replay.json')
    assert module['passed'] and module['counts']=={'passed':528} and module['collected']==528 and module['candidate_files']==source
    assert cfg['passed'] and cfg['actual_layouts']==4 and cfg['native_joint_limit_guard_phase_count']==98 and cfg['raw_branch_prealign_phase_count']==2
    assert all(x['whole_config_exact_after_declared_projection'] and x['success_criteria_exact'] for x in cfg['layout_checks'])
    assert old['passed'] and old['physical_branch_counterexample'] and old['raw_joint_error_rad']>2.
    assert new['passed'] and new['raw_requested_branch_reached'] and new['raw_joint_error_rad']<=.05 and new['full54'] and new['self_checked'] and not new['geometry_failures']
    assert new['source_manifest']==source and new['actual_input_step']==old['actual_input_step']==30992 and new['actual_recorded_run']==old['actual_recorded_run']=='A36_beam_full'
    changed=[n for n in source if source[n]!=baseline[n]]
    assert set(changed)=={'roboassemblybench/core/beam_assisted_compiler.py','toolkits/factory_dual_franka_assembly/beam_assisted_state.py','tests/toolkits/test_beam_assisted_object_servo.py'}
    dump=lambda node:ast.dump(node,include_attributes=False)
    k='toolkits/factory_dual_franka_assembly/beam_assisted_state.py';project=ast.parse((c/k).read_text());removed=0
    for node in ast.walk(project):
        if hasattr(node,'body') and isinstance(node.body,list):
            body=[]
            for child in node.body:
                if isinstance(child,ast.FunctionDef) and child.name=='_assisted_arm_joint_limits':removed+=1;continue
                if isinstance(child,ast.If) and any(isinstance(x,ast.Constant) and x.value=='beam_assisted_native_joint_limits' for x in ast.walk(child.test)):removed+=1;continue
                body.append(child)
            node.body=body
    assert removed==2 and dump(project)==dump(ast.parse((a/k).read_text()))
    k='roboassemblybench/core/beam_assisted_compiler.py';project=ast.parse((c/k).read_text());removed=0
    for node in ast.walk(project):
        if hasattr(node,'body') and isinstance(node.body,list):
            body=[]
            for child in node.body:
                if isinstance(child,ast.For) and any(isinstance(x,ast.Constant) and x.value=='beam_assisted_native_joint_limits' for x in ast.walk(child)):removed+=1;continue
                body.append(child)
            node.body=body
    assert removed==1 and dump(project)==dump(ast.parse((a/k).read_text()))
    for k in ['toolkits/factory_dual_franka_assembly/beam_assisted_servo.py','toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py','toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py']:
        assert source[k]==baseline[k]
    names=['affected_module_result.json','configuration_delta.json','base_configuration_identity.json','baseline_control_replay.json','candidate_control_replay.json']
    report=dict(passed=True,qualification_kind='joint_branch_prealign',candidate_files=source,source_files=889,changed_files=changed,unchanged_source_files=886,
        new_affected_tests=528,unique_related_tests=528,original_model_guard_exact=True,original_success_criteria_exact=True,physical_attachment_code_exact=True,
        existing_prealign_controller_exact=True,old_state_AST_exact_after_only_new_guard_projection=True,compiler_only_declared_delta=True,
        actual_four_layouts_checked=True,actual_input_final_command_replay_full54_self=True,original_joint_range_not_changed=True,native_started=False,full_assembly_accepted=False,
        source_change_scope='Existing prealign completion/raw-branch options enabled for remaining fourth/fifth pickups, original whole-phase budgets/rates/tracking unchanged. Final explicit commands checked against same-live-view actual native limits after final clamp. No limit setters, q writes, solver/attachment/model/lead/success changes.',
        evidence={n:dict(path=str(w/n),sha256=sha(w/n)) for n in names},tool_sha256=sha(__file__))
    if not args.deploy_version:
        assert not (w/'source_qualification.json').exists();save(w/'candidate_source_manifest.json',source);save(w/'source_qualification.json',report);print(json.dumps(dict(passed=True,source_files=889,changed_files=changed,tests=528)));return
    assert args.deploy_version=='v32' and args.for_run=='A38_beam_full'
    assert read(w/'source_qualification.json')['candidate_files']==source
    probe=read(p/'D21_probe_review.json');assert probe['screened_lift_completed'] and probe['native_grasp_requalified'] and not probe['model_guard_rejections'] and probe['execution_evidence_passed'] and probe['complete_video_decode_passed']
    assert probe['original_result']['assisted_boundary_stop']['completed_substage']=='all_clearance'
    after=read(p/'runs'/probe['run']/'source_after.json');assert not after['differences'] and not after.get('tool_differences') and {r['path']:r['sha256'] for r in after['files']}==source
    assert not (w/'deployment.json').exists()
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB);batch=read(p/'repair_batch_D_20261016.json')
        assert batch['native_attempts_started']==batch['completed_cycles']==21 and batch['maximum_native_attempts']==31
        assert all(sha(a/n)==h for n,h in baseline.items())
        olddir=w/'before_authority';olddir.mkdir()
        for n in ['assisted_manifest.json','assisted_tools_manifest.json']:shutil.copy2(t/n,olddir/n)
        for n in changed:
            backup=olddir/n;backup.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a/n,backup)
            temp=(a/n).with_name(Path(n).name+'.qualified-new');shutil.copy2(c/n,temp);os.replace(temp,a/n)
        assert all(sha(a/n)==h for n,h in source.items())
        subprocess.run([sys.executable,str(t/'seal_version.py'),'v32',args.for_run],check=True)
        assert sha(t/'assisted_manifest.json')==sha(w/'candidate_source_manifest.json')
        save(w/'deployment.json',dict(deployed=True,version='v32',run=args.for_run,source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),source_qualification_sha256=sha(w/'source_qualification.json'),positive_probe_sha256=sha(p/'D21_probe_review.json'),native_started=False))
    print(json.dumps(read(w/'deployment.json')))

def qualify_com_metric(args,p,t,a,w,c,read,baseline,source):
    """Qualify only the shared insertion measurement change against v30."""
    module=read(w/'affected_module_result.json');broad=read(w/'validation_errors/broad_fixture_missing_COM/broad_suite_result.json')
    cfg=read(w/'configuration_delta.json');motion=read(w/'native_admission_replay.json')
    old=read(w/'baseline_admission_commands.json');new=read(w/'candidate_admission_commands.json')
    assert module['passed'] and module['candidate_files']==source and module['counts']=={'passed':152} and module['collected']==152
    assert broad['counts']=={'passed':515,'failed':1} and broad['collected']==516 and not broad['passed']
    key='tests/toolkits/test_beam_assisted_object_servo.py'
    assert [n for n in source if source[n]!=broad['candidate_files'][n]]==[key]
    before=ast.parse((w/'validation_errors/broad_fixture_missing_COM/test_beam_assisted_object_servo.py').read_text())
    after=ast.parse((c/key).read_text());fixture='test_full_recipe_final_without_split_lateral_field_cannot_skip_cold_or_lost_native_gate'
    filtered=lambda tree:ast.dump(ast.Module(body=[n for n in tree.body if not isinstance(n,ast.FunctionDef) or n.name!=fixture],type_ignores=[]),include_attributes=False)
    assert filtered(before)==filtered(after)
    assert cfg['passed'] and cfg['actual_recorded_baseline_matched'] and cfg['baseline_run']=='A34_beam_full'
    assert cfg['descent_COM_metric_phase_count']==40 and all(v['whole_config_exact_after_declared_projection'] and v['success_criteria_exact'] for v in cfg['layout_checks'])
    assert motion['passed'] and motion['source_manifest']==source and motion['native_samples']==973 and not motion['missing_native_samples']
    assert motion['legacy_default_exact_cases']==448 and all(motion['protected_function_AST_exact'].values())
    assert motion['first_new_qualified_old_rejected']['new_ready'] and not motion['first_new_qualified_old_rejected']['old_ready']
    assert motion['first_new_qualified_old_rejected']['new_detail']['native_count']==12
    for r,root in [(old,a),(new,c)]:
        assert r['passed'] and r['source_manifest']=={n:sha(root/n) for n in source}
        assert r['actual_input_step']==38200 and r['actual_recorded_run']=='A34_beam_full' and r['recorded_native_motion']
        assert r['full54'] and r['self_checked'] and r['exact_entity_count']==44 and r['guard_calls']==27 and r['mesh_samples']==135
        assert not r['native_physics'] and not r['geometry_failures']
    assert old['selected_observation_sha256']==new['selected_observation_sha256']
    assert not old['descent_admission_observed'] and new['descent_admission_observed']
    assert -.0031 < new['final_publication_axial_delta_m']-old['final_publication_axial_delta_m'] < -.0029
    changed=[n for n in source if source[n]!=baseline[n]]
    expected={'roboassemblybench/core/beam_assisted_compiler.py','toolkits/factory_dual_franka_assembly/beam_assisted_servo.py','toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py','toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py',key,'tests/toolkits/test_beam_assisted_preflight.py'}
    assert set(changed)==expected and len(source)==889
    # Compare every AST node outside explicitly changed control functions.
    def tree(root,key):return ast.parse((root/key).read_text())
    def dump(x):return ast.dump(x,include_attributes=False)
    def without(tree,name):
        tree=__import__('copy').deepcopy(tree)
        for node in ast.walk(tree):
            if hasattr(node,'body') and isinstance(node.body,list):node.body=[n for n in node.body if not isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef)) or n.name!=name]
        return dump(tree)
    protected={}
    for filename,allowed in [('beam_assisted_servo.py','descent_admission'),('plumbers_block_ur5e_skills.py','_assisted_object_servo_context')]:
        k='toolkits/factory_dual_franka_assembly/'+filename
        protected[filename+'_outside_changed_function']=without(tree(a,k),allowed)==without(tree(c,k),allowed)
    k='roboassemblybench/core/beam_assisted_compiler.py';project=tree(c,k);removed=0
    for node in ast.walk(project):
        if isinstance(node,ast.Dict) and any(isinstance(k,ast.Constant) and k.value=='lateral_tolerance_m' for k in node.keys):
            pairs=[(x,y) for x,y in zip(node.keys,node.values) if not (isinstance(x,ast.Constant) and x.value=='motion_metric' and isinstance(y,ast.Constant) and y.value=='native_com_pose_difference')]
            removed+=len(node.keys)-len(pairs);node.keys=[x for x,y in pairs];node.values=[y for x,y in pairs]
    protected['compiler_only_motion_metric_added']=removed==1 and dump(project)==dump(tree(a,k))
    k='toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py';project=tree(c,k);removed=0
    for node in ast.walk(project):
        if hasattr(node,'body') and isinstance(node.body,list):
            result=[]
            for child in node.body:
                if isinstance(child,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='admission' for x in child.targets):removed+=1;continue
                if isinstance(child,ast.If) and any(isinstance(x,ast.Constant) and x.value=='native_com_pose_difference' for x in ast.walk(child)):removed+=1;continue
                result.append(child)
            node.body=result
    protected['preflight_only_optional_metric_validation']=removed==2 and dump(project)==dump(tree(a,k))
    assert all(protected.values()),protected
    for k in ['toolkits/factory_dual_franka_assembly/beam_assisted_state.py','toolkits/factory_dual_franka_assembly/beam_assisted_camera.py','internutopia/core/runner.py']:
        assert source[k]==baseline[k]
    evidence_names=['affected_module_result.json','configuration_delta.json','base_configuration_identity.json','native_admission_replay.json','baseline_admission_commands.json','candidate_admission_commands.json','asset_bundle_identity_binding.json','validation_errors/broad_fixture_missing_COM/broad_suite_result.json','validation_errors/broad_fixture_missing_COM/test_beam_assisted_object_servo.py']
    reuse_path=w/'related_suite_reuse_certificate.json'
    if not reuse_path.exists():save(reuse_path,dict(passed=True,newly_rerun_affected_tests=152,unchanged_other_module_tests_reused=364,unique_qualified_tests=516,prior_failed_suite_preserved=True,only_private_test_fixture_changed_since_broad=True,all_other_test_AST_exact=True,production_source_exact_since_broad=True,scope='152 passed now plus364 unchanged outside-module tests from prior515-pass/1-failed suite; not516 newly rerun.',evidence={n:sha(w/n) for n in evidence_names}))
    reuse=read(reuse_path);assert reuse['passed'] and all(sha(w/n)==h for n,h in reuse['evidence'].items())
    report=dict(passed=True,qualification_kind='descent_com_metric',candidate_files=source,source_files=889,changed_files=changed,unchanged_source_files=883,protected_function_AST_exact=protected,original_model_guard_exact=True,physical_attachment_code_exact=True,original_success_criteria_exact=True,actual_four_layouts_checked=True,new_affected_tests=152,unique_related_tests=516,reused_other_module_tests=364,actual_native_stream_12_tick_positive=True,actual_input_final_command_replay_full54_self=True,full_assembly_accepted=False,native_started=False,source_change_scope='Only insertion motion measurement uses unchanged native COM pose-rate estimator. All40 insertion contracts retain original speed/geometry/count limits. Raw velocities retained; absent/bad COM and clock/identity gaps fail closed. Physical solver64 and accepted v30 carry fix unchanged.',evidence={n:dict(path=str(w/n),sha256=sha(w/n)) for n in evidence_names+['related_suite_reuse_certificate.json']},tool_sha256=sha(__file__))
    if not args.deploy_version:
        assert not (w/'source_qualification.json').exists();save(w/'candidate_source_manifest.json',source);save(w/'source_qualification.json',report)
        print(json.dumps(dict(passed=True,changed_files=changed,unique_tests=516,newly_rerun=152)));return
    assert args.deploy_version=='v31' and args.for_run=='A35_beam_full'
    assert read(w/'source_qualification.json')['candidate_files']==source and not (w/'deployment.json').exists()
    prior=read(p/'D18_conclusions.json');media=read(p/'D18_local_media_review.json')
    assert prior['full_from_head'] and prior['complete_originals_downloaded']==3 and media['passed']
    with (p/'beam_run.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        batch=read(p/'repair_batch_D_20261016.json');assert batch['native_attempts_started']==batch['completed_cycles']==18 and batch['maximum_native_attempts']==19
        assert all(sha(a/n)==h for n,h in baseline.items()) and all(sha(t/n)==h for n,h in read(t/'assisted_tools_manifest.json').items())
        olddir=w/'before_authority';olddir.mkdir()
        for name in ['assisted_manifest.json','assisted_tools_manifest.json']:shutil.copy2(t/name,olddir/name)
        save(olddir/'ledger.json',batch)
        auxiliary=w/'v31_auxiliary_tools.tar.gz'
        with tarfile.open(auxiliary,'x:gz') as bundle:bundle.add(t/'bounded_scene_http_probe.py',arcname='bounded_scene_http_probe.py',recursive=False)
        save(w/'operation_deployment.json',dict(passed=True,changes={},critical20_unchanged=True,bounded_scene_helper_sha256=sha(t/'bounded_scene_http_probe.py'),auxiliary_archive_sha256=sha(auxiliary),native_started=False))
        for name in changed:
            f=olddir/name;f.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a/name,f)
            temp=(a/name).with_name(Path(name).name+'.qualified-new');shutil.copy2(c/name,temp);os.replace(temp,a/name)
        assert all(sha(a/n)==h for n,h in source.items())
        subprocess.run([sys.executable,str(t/'seal_version.py'),args.deploy_version,args.for_run],check=True)
        assert sha(t/'assisted_manifest.json')==sha(w/'candidate_source_manifest.json')
        assert sha(t/'assisted_tools_manifest.json')==sha(olddir/'assisted_tools_manifest.json')
        save(w/'deployment.json',dict(deployed=True,version=args.deploy_version,run=args.for_run,source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),source_qualification_sha256=sha(w/'source_qualification.json'),prior_full_conclusions_sha256=sha(p/'D18_conclusions.json'),native_started=False))
    print(json.dumps(read(w/'deployment.json')))

p=Path(__file__).resolve().parents[1];t=p/'tools';a=p/'assisted_repository'
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('--deploy-version');ap.add_argument('--for-run');ap.add_argument('--probe-cycle',default='D17');ap.add_argument('--descent-com-metric',action='store_true');ap.add_argument('--joint-branch-prealign',action='store_true');ap.add_argument('--staged-object',choices=['fabrica_beam_0']);ap.add_argument('--final-home-wrist-roll',action='store_true');args=ap.parse_args();w=args.work.resolve();c=w/'candidate'
read=lambda f:json.loads(Path(f).read_text())
baseline=read(w/'baseline_manifest.json');source={n:sha(c/n) for n in sorted(baseline)}
if args.final_home_wrist_roll:
    qualify_final_home_wrist_roll(args,p,t,a,w,c,read,baseline,source);raise SystemExit(0)
if args.staged_object:
    qualify_staged_object(args,p,t,a,w,c,read,baseline,source);raise SystemExit(0)
if args.joint_branch_prealign:
    qualify_joint_branch(args,p,t,a,w,c,read,baseline,source);raise SystemExit(0)
if args.descent_com_metric:
    qualify_com_metric(args,p,t,a,w,c,read,baseline,source);raise SystemExit(0)
module=read(w/'affected_module_result.json');cfg=read(w/'configuration_delta.json');replay=read(w/'candidate_control_replay.json')
assert module['passed'] and module['candidate_files']==source and module['counts']['passed']==module['collected']>=130
assert cfg['passed'] and cfg['actual_layouts']==4 and all(r['success_criteria_exact'] for r in cfg['layout_checks'])
assert replay['passed'] and replay['source_manifest']==source and replay['full54'] and replay['self_checked'] and replay['guard_calls']>=27
assert replay['original_over_limit_rejected_before_IK_CAD'] and replay['final_publication_lateral_delta_m']<2e-6
reuse=read(w/'terminal_cap_reuse_certificate.json');broad=read(w/'before_terminal_cap/broad_suite_result.json')
assert reuse['passed'] and all(reuse[n] for n in ['unflagged_adapter_AST_exact','servo_original_functions_AST_exact','old_tests_all_retained','configuration_compiler_unchanged_since_broad'])
assert broad['passed'] and broad['counts']['passed']==487 and reuse['new_affected_tests']==module['collected']
assert all(sha(n)==h for n,h in reuse['evidence'].items())
changed=[n for n in source if source[n]!=baseline[n]];assert len(changed)==4
assert set(changed)=={'roboassemblybench/core/beam_assisted_compiler.py','toolkits/factory_dual_franka_assembly/beam_assisted_servo.py','toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py','tests/toolkits/test_beam_assisted_object_servo.py'}
# Directly audit unchanged protected functions against the authoritative v28 bytes.
def function(path,name):
    tree=ast.parse(path.read_text());return ast.dump(next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name),include_attributes=False)
checks={}
for name in ['validate_model_native_residual','object_pose_from_tcp','anchored_object_prediction','absolute_object_servo','descent_admission']:
    key='toolkits/factory_dual_franka_assembly/beam_assisted_servo.py';checks[name]=function(a/key,name)==function(c/key,name)
assert all(checks.values())
for key in ['toolkits/factory_dual_franka_assembly/beam_assisted_state.py','toolkits/factory_dual_franka_assembly/beam_assisted_preflight.py','toolkits/factory_dual_franka_assembly/beam_assisted_camera.py','internutopia/core/runner.py']:
    assert sha(c/key)==baseline[key]
report=dict(passed=True,candidate_files=source,source_files=889,changed_files=changed,unchanged_source_files=885,
    protected_function_AST_exact=checks,original_model_guard_exact=True,physical_attachment_code_exact=True,
    numerical_configuration_change='Only fabrica_beam_1 solver_position_iteration_count 16 to 64, including matching object metadata. Velocity iterations unchanged; original success and model limits unchanged.',
    source_change_scope='Fourth-part explicitly flagged incremental final-drive reference and frozen phase-entry drive bias with bounded terminal target; part1 position-solver iterations64. Original speed/lead/model/CAD/attachment/success limits unchanged.',
    affected_module_tests=module['collected'],all_old_module_tests_retained=True,actual_four_layouts_checked=True,
    original_success_criteria_exact=True,full54_self_actual_input_replay=True,original_over_limit_rejected=True,
    baseline_reused_scope=reuse['scope'],broad_cached_tests=487,new_affected_tests=module['collected'],
    evidence={n:dict(path=str(w/n),sha256=sha(w/n)) for n in ['affected_module_result.json','configuration_delta.json','candidate_control_replay.json','baseline_control_replay.json','base_configuration_identity.json','terminal_cap_reuse_certificate.json','before_terminal_cap/broad_suite_result.json']},
    tool_sha256=sha(__file__),native_started=False)
if not args.deploy_version:
    assert not (w/'source_qualification.json').exists();save(w/'candidate_source_manifest.json',source);save(w/'source_qualification.json',report)
    print(json.dumps(dict(passed=True,changed_files=changed,tests=module['collected'])));raise SystemExit(0)
assert args.for_run and args.deploy_version=='v30'
qualification=read(w/'source_qualification.json');assert qualification['candidate_files']==source
probe=read(p/(args.probe_cycle+'_probe_review.json'));assert probe['screened_lift_completed'] and probe['native_grasp_requalified']
boundary=probe['original_result']['assisted_boundary_stop']
assert boundary['reason']=='isolated-full-clearance-screen-complete' and boundary['completed_substage']=='all_clearance'
assert not probe['model_guard_rejections'] and probe['execution_evidence_passed']
probe_root=p/'runs'/probe['run'];probe_source=read(probe_root/'source_after.json')
assert {r['path']:r['sha256'] for r in probe_source['files']}==source and not probe_source['differences'] and not probe_source.get('tool_differences')
assert not (w/'deployment.json').exists()
with (p/'beam_run.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    batch=read(p/'repair_batch_D_20261016.json');assert batch['native_attempts_started']==batch['completed_cycles']
    assert all(sha(a/n)==h for n,h in baseline.items())
    old=w/'before_authority';old.mkdir()
    shutil.copy2(t/'assisted_manifest.json',old/'assisted_manifest.json');shutil.copy2(t/'assisted_tools_manifest.json',old/'assisted_tools_manifest.json')
    operations=read(w/'ops_candidate/ops_qualification.json')
    assert operations['passed'] and operations['worker_unflagged_AST_exact'] and operations['only_bounded_transport_call_added'] and not operations['physics_or_controller_code_changed']
    operation_changes={}
    for name,prefix in [('assisted_worker.py','worker'),('build_delta_gate.py','gate')]:
        candidate=w/'ops_candidate'/name;current=t/name
        assert sha(current)==operations[prefix+'_baseline_sha256'] and sha(candidate)==operations[prefix+'_candidate_sha256']
        shutil.copy2(current,old/name);temp=current.with_name(name+'.qualified-new');shutil.copy2(candidate,temp);os.replace(temp,current)
        operation_changes[name]=dict(before_sha256=sha(old/name),after_sha256=sha(current),backup=str(old/name))
    auxiliary=w/'v30_auxiliary_tools.tar.gz';assert not auxiliary.exists()
    with tarfile.open(auxiliary,'w:gz') as bundle:
        bundle.add(t/'bounded_scene_http_probe.py',arcname='bounded_scene_http_probe.py',recursive=False)
    save(w/'operation_deployment.json',dict(passed=True,changes=operation_changes,bounded_scene_helper_sha256=sha(t/'bounded_scene_http_probe.py'),auxiliary_archive_sha256=sha(auxiliary),native_started=False))
    for name in changed:
        f=old/name;f.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(a/name,f)
        temp=(a/name).with_name(Path(name).name+'.qualified-new');shutil.copy2(c/name,temp);os.replace(temp,a/name)
    assert all(sha(a/n)==h for n,h in source.items())
    subprocess.run([sys.executable,str(t/'seal_version.py'),args.deploy_version,args.for_run],check=True)
    assert sha(t/'assisted_manifest.json')==sha(w/'candidate_source_manifest.json')
    save(w/'deployment.json',dict(deployed=True,version=args.deploy_version,run=args.for_run,source_manifest_sha256=sha(t/'assisted_manifest.json'),tools_manifest_sha256=sha(t/'assisted_tools_manifest.json'),source_qualification_sha256=sha(w/'source_qualification.json'),positive_probe_sha256=sha(p/(args.probe_cycle+'_probe_review.json')),native_started=False))
print(json.dumps(read(w/'deployment.json')))
