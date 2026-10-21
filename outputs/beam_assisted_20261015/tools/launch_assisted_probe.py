"""Qualified bounded diagnostic launch. Every native invocation claims batch budget."""
from pathlib import Path
import argparse,fcntl,json,os,subprocess,sys,time
from beam_workflow import claim,save,sha,validate_attempt_window

p=Path(__file__).resolve().parents[1];t=p/'tools'
ap=argparse.ArgumentParser();ap.add_argument('work',type=Path);ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--baseline',action='store_true');ap.add_argument('--validate-only',action='store_true');ap.add_argument('--preparation-dir',type=Path);ap.add_argument('--branch-screen',action='store_true');ap.add_argument('--staged-object',choices=['fabrica_beam_0']);args=ap.parse_args()
w=args.work.resolve();c=w/'candidate';run=args.run;cycle=args.cycle;prepdir=args.preparation_dir.resolve() if args.preparation_dir else w
assert run.replace('_','').isalnum() and cycle.startswith('D')
with (p/'beam_run.lock').open('w') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    batch=json.loads((p/'repair_batch_D_20261016.json').read_text())
    validate_attempt_window(batch,run,cycle)
    if (p/'runs'/run).exists():raise RuntimeError('Output already exists')
    reports={n:json.loads((w/n).read_text()) for n in ['affected_module_result.json','configuration_delta.json','baseline_control_replay.json','candidate_control_replay.json']}
    assert all(r['passed'] is True for r in reports.values())
    assert reports['affected_module_result.json']['counts']['passed']==reports['affected_module_result.json']['collected']>=124
    assert reports['configuration_delta.json']['actual_layouts']==4
    if args.staged_object:
        assert not args.baseline
        candidate=reports['candidate_control_replay.json'];baseline=reports['baseline_control_replay.json']
        assert candidate['qualification_kind']=='staged_clearance_object' and candidate['geometric_path_completed'] and candidate['production_entry_chain_verified']
        assert candidate['full54'] and candidate['self_checked'] and not candidate['geometry_failures']
        assert candidate['minimum_right_payload_gap_m']>=.005 and baseline['spatial_counterexample']
        assert reports['configuration_delta.json']['staged_object']==args.staged_object
        qualification=json.loads((w/'source_qualification.json').read_text());assert qualification['passed'] and qualification['qualification_kind']=='staged_clearance_object'
        assert sha(t/'qualify_assisted_candidate.py')==qualification['tool_sha256']
        reports['source_qualification.json']=qualification
    elif args.branch_screen:
        assert not args.baseline and prepdir==w
        candidate=reports['candidate_control_replay.json'];baseline=reports['baseline_control_replay.json']
        assert candidate['qualification_kind']=='joint_branch_prealign' and candidate['raw_requested_branch_reached'] and not candidate['geometry_failures']
        assert candidate['full54'] and candidate['self_checked'] and candidate['raw_joint_error_rad']<=.05
        assert baseline['physical_branch_counterexample'] and not baseline['raw_requested_branch_reached']
        assert reports['configuration_delta.json']['native_joint_limit_guard_phase_count']==98
        qualification=json.loads((w/'source_qualification.json').read_text());assert qualification['passed'] and qualification['qualification_kind']=='joint_branch_prealign'
        assert sha(t/'qualify_assisted_candidate.py')==qualification['tool_sha256']
        reports['source_qualification.json']=qualification
    else:
        assert reports['candidate_control_replay.json']['final_publication_lateral_delta_m']<2e-6
        assert reports['baseline_control_replay.json']['final_publication_lateral_delta_m']>1e-4
    manifest=reports['candidate_control_replay.json']['source_manifest']
    assert len(manifest)==889 and all(sha(c/n)==h for n,h in manifest.items())
    for n in ['affected_module_result.json']:
        assert reports[n]['candidate_files']==manifest
    prep=json.loads((prepdir/'probe_preparation.json').read_text());assert prep['passed']
    reports[str(prepdir/'probe_preparation.json')]=prep
    assert sha(prepdir/'isolated_fixture.json')==prep['fixture_sha256']
    if args.branch_screen or args.staged_object:
        assert prep['qualification_kind']==('staged_clearance_object' if args.staged_object else 'joint_branch_prealign') and prep['whole_free_clearance_screen']
        assert sha(t/'bounded_scene_http_probe.py')==prep['http_probe_tool_sha256']
        # Same URL/content check before native output creation and claim.
        from bounded_scene_http_probe import fetch
        network=fetch();fixture=json.loads((prepdir/'isolated_fixture.json').read_text())
        assert network['url']==fixture['bounded_primary_scene_probe']['url'] and network['sha256']==fixture['bounded_primary_scene_probe']['sha256']
        save(w/'probe_network_preflight.json',dict(passed=True,result=network,native_started=False))
    if prepdir!=w and args.staged_object:
        assert prep['same_initialization_and_full_configuration']
        assert prep['base_fixture_sha256']==sha(w/'isolated_fixture.json')
        fixture=json.loads((prepdir/'isolated_fixture.json').read_text());base=json.loads((w/'isolated_fixture.json').read_text())
        assert fixture['stop_after_phase']=='assemble_03_part_0_park' and fixture['stop_label']=='release_and_park'
        assert all(fixture[k]==v for k,v in base.items() if k not in {'stop_after_phase','stop_label','maximum_diagnostic_steps'})
        assert json.loads((prepdir/'probe_candidate_configuration.json').read_text())==json.loads((w/'probe_candidate_configuration.json').read_text())
    elif prepdir!=w:
        assert prep['only_fourth_part_position_iterations_16_to_64'] and prep['full_config_exact_after_single_property_projection']
        if prep.get('bounded_same_primary_scene_transport'):
            assert sha(t/'bounded_scene_http_probe.py')==prep['http_probe_tool_sha256']
        assert prep['controls_and_guards_unchanged'] and sha(w/'isolated_fixture.json')==prep['base_fixture_sha256']
        assert sha(w/'A29_joint_anchor_audit.json')==prep['background_native_anchor_audit_sha256']
    if sha(t/'assisted_probe.py')!=prep['tool_sha256']:
        flush=json.loads((w/'profiler_flush_qualification.json').read_text())
        assert flush['passed'] and flush['fixture_and_preparation_AST_exact'] and flush['only_passive_timing_flush_added']
        assert prep['tool_sha256']==flush['original_sha256'] and sha(t/'assisted_probe.py')==flush['updated_sha256']
        reports['profiler_flush_qualification.json']=flush
    if (w/'terminal_cap_reuse_certificate.json').exists():
        terminal=json.loads((w/'terminal_cap_reuse_certificate.json').read_text());assert terminal['passed'] and terminal['new_affected_tests']==130
        reports['terminal_cap_reuse_certificate.json']=terminal
    tools=json.loads((t/'assisted_tools_manifest.json').read_text());assert all(sha(t/n)==h for n,h in tools.items())
    if args.validate_only:
        print(json.dumps(dict(passed=True,source_files=len(manifest),critical_tools=len(tools),cycle=cycle,run=run,native_started=False)));raise SystemExit(0)
    out=p/'runs'/run;out.mkdir(parents=True)
    save(out/'route_source_manifest.json',manifest)
    save(out/'probe_tools_manifest.json',{str(t/n):sha(t/n) for n in set(tools)|{'assisted_probe.py','launch_assisted_probe.py','probe_runtime.sh','bounded_scene_http_probe.py'}})
    key='baseline' if args.baseline else 'candidate'
    gate=dict(schema='isolated_diagnostic_gate_v1',run_name=run,cycle=cycle,run_class='isolated_selected_part_probe' if args.staged_object else 'isolated_fourth_part_probe',
        actual_configuration_signature=prep['configurations'][key]['configuration_signature'],
        source_manifest_sha256=sha(out/'route_source_manifest.json'),tools_manifest_sha256=sha(out/'probe_tools_manifest.json'),
        reports={n:sha(w/n) for n in reports},original_assembly_acceptance=False,
        scope='New short physical scene, preserved production command/CAD/success guards. Never a checkpoint or an end-to-end success claim.')
    save(out/'launch_gate.json',dict(gate=gate))
    index=batch['native_attempts_started'];assert len(batch['cycles'])==index
    batch['cycles'].append(dict(id=cycle,planned_run=run,native_started=False,state='ready',run_class=gate['run_class'],candidate='unsealed_'+sha(out/'route_source_manifest.json')[:12],source_root=str(c),source_manifest_sha256=sha(out/'route_source_manifest.json')))
    batch.update(next_native_run_authorized=True,status='ready',current_cycle=cycle)
    save(p/'repair_batch_D_20261016.json',batch)
    receipt=claim(p/'repair_batch_D_20261016.json',run,cycle);save(out/'native_claim.json',receipt)
    env=os.environ.copy();env.update(BEAM_PROBE_WORK=str(w),BEAM_PROBE_BASELINE='1' if args.baseline else '0',BEAM_ROUTE_SOURCE=str(c),BEAM_PROBE_OUT=str(out),BEAM_PROBE_FIXTURE=str(prepdir/'isolated_fixture.json'))
    begin=time.monotonic()
    result=subprocess.run(['bash',str(t/'probe_runtime.sh')],env=env,pass_fds=(lock.fileno(),))
    save(out/'probe_process_receipt.json',dict(returncode=result.returncode,elapsed_s=time.monotonic()-begin,claimed=True,claim=receipt))
    raise SystemExit(result.returncode)
