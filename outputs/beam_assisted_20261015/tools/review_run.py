"""One timed postprocessing entry; preserve every original review and video."""
import argparse,json,sys,time
from pathlib import Path
from beam_workflow import save,sha,timed
from review_cache import immutable_inputs, fingerprint, reusable, file_bindings

def review_startup_failure(p,d,t,args,source,target):
    """Preserve a pre-observation crash without manufacturing task results/media."""
    import tarfile
    assert not target.exists()
    absent=['collect_results.json','startup_images_ready.json','startup_image_review_ack.json',
            'assisted_trace.jsonl','physics_boundary_trace.jsonl','entity_geometry_trace.jsonl','video_frame_clock.jsonl']
    assert all(not (d/n).exists() for n in absent)
    assert not list(d.rglob('*.mp4')) and not list(d.rglob('*.png'))
    native_log=p/'batch_D_20261016'/(args.cycle+'_native.log')
    log=native_log.read_text(errors='replace')
    assert 'demo.main()' in log and 'env = _build_env(' in log
    diagnosis=None;seal=None;context=[];configuration_check=None
    home_error=('AssistedConfigurationError: assisted configuration preflight rejected:' in log
        and 'runtime_report=assert_assisted_configuration(expected_config)' in log
        and all(name+': '+message in log for name in ('franka_left','franka_right')
                for message in ('final home target differs from configured initial native branch',
                                'return action does not target initial joints')))
    if home_error:
        # The saved snapshot precedes the framework's name transformation.
        # Check its numbers directly; bind the separate source/stack diagnosis.
        cfg=json.loads((d/'task_config_snapshot.json').read_text())['task_configs'][0]
        robots={r['name']:r for r in cfg['robots']};home=cfg['phase_specs'][-1]['beam_assisted_home_targets']
        assert set(robots)==set(home)=={'franka_left','franka_right'}
        configuration_check={}
        for name,home_target in home.items():
            initial=robots[name]['initial_joint_positions']
            values=[initial[j] for j in home_target['joint_names']]
            assert len(values)==6 and values==home_target['joint_positions']
            configuration_check[name]=dict(saved_config_name=robots[name]['name'],
                home_target_logical_name=name,initial_joint_positions=values,
                declared_home_joint_positions=home_target['joint_positions'],all_six_values_exact=True)
        diagnostic_path=p/(args.cycle+'_work')/'runtime_name_mapping_counterexample.json'
        diagnosis=json.loads(diagnostic_path.read_text())
        assert (diagnosis.get('run')==args.run and diagnosis.get('passed_source_diagnosis') is True
                and diagnosis.get('all_declared_home_joint_values_exact') is True
                and diagnosis.get('first_action_executed') is False
                and isinstance(diagnosis.get('evidence_class'),str) and diagnosis['evidence_class'])
        for name in robots:
            row=diagnosis['robots'][name]
            assert (row['actual_build_config_name']==name and row['all_six_values_exact'] is True
                    and row['source_derived_framework_env0_config_name']==name+'_0'
                    and row['new_preflight_exact_lookup_would_find_config'] is False)
        for path,expected in diagnosis['inputs'].items():assert sha(path)==expected
        sealed_path=p/(args.run+'_sealed_version.json');seal=json.loads(sealed_path.read_text())
        receipt=json.loads((p/(args.cycle+'_launch_receipt.json')).read_text())
        assert (seal['run']==receipt['run']==args.run and receipt['cycle']==args.cycle
                and seal['version']=='v34' and seal['critical_tools']==20
                and seal['source_manifest_sha256']==receipt['source_manifest_sha256']
                and seal['tools_manifest_sha256']==receipt['tools_manifest_sha256']
                and sha(d/'route_source_manifest.json')==seal['source_manifest_sha256'])
        context=[diagnostic_path,sealed_path]
        failure_kind='assisted_final_home_logical_name_configuration'
        reason='New final-home runtime preflight rejected the framework logical-name mapping before environment construction; both saved home6q targets exactly match the saved initial configuration.'
        candidate_limit='v34 wrist-view roll, empty-arm return and native-home qualification have no native dynamic result; the earlier v33/A42 assisted success remains separate.'
    else:
        # Preserve the previously accepted TLS failure class exactly.
        assert 'bounded_scene_http_probe.py' in log and 'The handshake operation timed out' in log
        failure_kind='primary_scene_TLS_handshake_timeout'
        reason='TLS handshake timeout during bounded check of exact primary scene URL, before environment construction completed.'
        candidate_limit='v31 COM admission fix retains only offline qualification; A34 v30 actual carry progress remains separate.'
    timing_path=p/'batch_D_20261016/timing_events.jsonl'
    timing=[json.loads(line) for line in timing_path.read_text().splitlines()]
    ends=[r for r in timing if r.get('phase')==args.cycle+'_native' and r.get('action')=='end'];assert len(ends)==1
    assert ends[0]['returncode']==1
    run_files=[f for f in sorted(d.rglob('*')) if f.is_file()]
    external=[native_log,p/(args.run.split('_')[0]+'_launch.log'),p/(args.cycle+'_launch_receipt.json'),timing_path]+context
    original_files=file_bindings(run_files+external)
    history=p/(args.cycle+'_work')/'before_startup_failure_postreview'
    auxiliary_versions=file_bindings([Path(__file__)]+sorted(history.glob('review_run.py.*')))
    report=dict(schema='W05.3-startup-failure',run=args.run,cycle=args.cycle,
        startup_failure_review_completed=True,all_required_stages_passed=False,evidence_checks_passed=False,
        source_after_sha256=sha(d/'source_after.json'),source_files_verified=len(source['files']),source_differences=source['differences'],tool_differences=source.get('tool_differences',[]),
        failure_kind=failure_kind,reason=reason,source_version=seal['version'] if seal else None,
        sealed_version=file_bindings(context[-1:]) if seal else {},
        configuration_home_numeric_check=configuration_check,
        logical_name_diagnosis_evidence_class=diagnosis.get('evidence_class') if diagnosis else None,
        original_run_file_count=len(run_files),external_evidence_file_count=len(external),
        original_native_wrapper_timing_event=ends[0],environment_constructed=False,
        first_action_executed=False,first_action_receipt_present=False,observed_native_motion_samples=0,
        task_result_available=False,full_assembly_accepted=False,candidate_dynamic_tested=False,
        startup_images=0,complete_original_videos=0,post_failure_tail_available=False,
        worker_exit_file=int((d/'exit_code').read_text().strip()),native_wrapper_returncode=ends[0]['returncode'],native_wrapper_elapsed_s=ends[0]['elapsed_s'],
        missing_data=absent,original_files=original_files,postreview_auxiliary_versions=auxiliary_versions,
        review_tool_in_native_critical_manifest=any(r['path']=='review_run.py' for r in source.get('tool_files',[])),
        stages=[dict(name=n,status='unavailable_before_environment_start',passed=False) for n in ['native_initialization','actual_three_camera_review','first_action_and_SDK','native_model_and_motion','full54_self_sweep','original_task_reports','five_second_physical_tail','three_complete_videos']],
        limits=['No original task result or video was generated; absent success/phase/physics fields are unknown, not a task-level measured failure.',
                'Startup attempt counts against explicit native cap. Never automatically retry it.',
                'Worker exit file0 does not certify success; wrapper returned1 and traceback is retained.',
                candidate_limit,
                'Postreview auxiliary versions are preserved separately and are not represented as part of the native critical20 seal.']
               +(['Saved configuration names are logical names; framework suffixed names are source/failed-stack inference until the separately recorded offline reproduction.'] if home_error else []),tool_sha256=sha(__file__))
    save(target,report)
    archive=p/(args.run+'_startup_failure_evidence.tar.gz');assert not archive.exists()
    with tarfile.open(archive,'x:gz') as bundle:
        for name in original_files:
            f=Path(name);bundle.add(f,arcname=str(f.relative_to(p)),recursive=False)
        for name,binding in auxiliary_versions.items():
            bundle.add(name,arcname='postreview_tools/review_run.'+binding['sha256']+'.py',recursive=False)
        bundle.add(target,arcname=target.name,recursive=False)
    manifest=p/(args.run+'_startup_failure_manifest.json')
    save(manifest,dict(passed_original_integrity=True,archive=str(archive),bytes=archive.stat().st_size,sha256=sha(archive),original_files=original_files,postreview_auxiliary_versions=auxiliary_versions,original_run_file_count=len(run_files),external_evidence_file_count=len(external),review_sha256=sha(target),videos_generated=0,media_review_applicable=False))
    print(json.dumps(dict(startup_failure_review_completed=True,full_assembly_accepted=False,archive=str(archive),archive_sha256=sha(archive),videos_generated=0,native_wrapper_returncode=ends[0]['returncode'])))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('run');ap.add_argument('cycle');ap.add_argument('--payload-right-clearance',choices=['fabrica_beam_0','fabrica_beam_1']);ap.add_argument('--final-home',action='store_true');ap.add_argument('--plan',action='store_true');ap.add_argument('--focus',help='Comma-separated reached clip names; first failure/tail is always retained');a=ap.parse_args()
    p=Path(__file__).resolve().parents[1];d=p/'runs'/a.run;t=p/'tools';events=p/'batch_D_20261016'
    if not (d/'finished_at').exists() or not (d/'source_after.json').exists():raise ValueError('Native process not fully closed')
    source=json.loads((d/'source_after.json').read_text())
    if source['differences'] or source.get('tool_differences'):raise ValueError('Runtime source/tool drift requires review first')
    gate=json.loads((d/'launch_gate.json').read_text())['gate']
    for check in gate['checks']:
        for evidence in check['evidence']:
            if sha(evidence['path'])!=evidence['sha256']:raise ValueError('Gate-bound evidence/tool changed during run: '+evidence['path'])
    report=dict(schema='W05.3',run=a.run,cycle=a.cycle,source_after_sha256=sha(d/'source_after.json'),
                stages=[],all_required_stages_passed=False)
    target=p/(a.cycle+'_postprocess_pipeline.json')
    if not a.plan and not (d/'collect_results.json').exists() and not (d/'startup_images_ready.json').exists():
        review_startup_failure(p,d,t,a,source,target);return
    previous=json.loads(target.read_text()) if target.exists() else {}
    if previous and previous.get('schema')!='W05.3' and not a.plan:
        raise ValueError('Preserve legacy pipeline/evidence; use --plan for a read-only check')
    prior={s['name']:s for s in previous.get('stages',[])}
    inputs=immutable_inputs(d)
    label=a.run.split('_')[0]
    stages=[('raw_index','index_run.py',[str(d)],True),
            ('original_postreview','postreview_run.py',[a.run],True),
            ('original_support_review','summarize_C_support.py',[a.run],False),
            ('support_clock_join','review_support_clock_join.py',[a.run],False),
            ('production_evidence','review_execution_evidence.py',[a.run,a.cycle],True),
            ('original_candidates','review_original_candidates.py',[a.run,a.cycle]+(['--payload-right-clearance',a.payload_right_clearance] if a.payload_right_clearance else [])+(['--final-home'] if a.final_home else []),True),
            ('strict_insertion_evidence','analyze_alignment_run.py',[a.run,a.cycle],True),
            ('full_archive_and_video_decode','archive_run.py',[a.run],True),
            ('three_original_video_bundle','pack_review_bundle.py',[a.run],True),
            ('key_node_clips','build_key_nodes.py',[a.run]+(['--focus',a.focus] if a.focus else []),True)]
    outputs={
        'raw_index':[d/'raw_offset_index.json'],
        'original_postreview':[d/'postreview_summary.json'],
        'original_support_review':[d/'C_support_summary.json'],
        'support_clock_join':[d/'support_clock_join_review.json'],
        'production_evidence':[p/(a.cycle+'_execution_evidence.json')],
        'original_candidates':[p/(a.cycle+'_original_candidates_review.json')],
        'strict_insertion_evidence':[p/(a.cycle+'_alignment_native_analysis_reviewed.json')],
        'full_archive_and_video_decode':[p/(a.run+'_evidence.tar.gz'),p/(a.run+'_archive_manifest.json'),d/'video_manifest.json'],
        'three_original_video_bundle':[p/(a.run+'_review_bundle.tar.gz'),p/(a.run+'_review_bundle_manifest.json')],
        'key_node_clips':[p/(label+'_key_nodes.tar.gz'),p/(label+'_key_nodes/manifest.json')]}
    for name,tool,args,required in stages:
        binding=fingerprint(dict(run=a.run,cycle=a.cycle,inputs=inputs,arguments=args,tool=sha(t/tool),
                                 helper=sha(t/'review_cache.py'),
                                 dependencies=[s.get('outputs',{}) for s in report['stages']]))
        if a.focus and name=='key_node_clips':binding=fingerprint(dict(default_binding=binding,focus_arguments=args))
        old=prior.get(name);cached=reusable(old,binding)
        if a.plan:
            print(json.dumps(dict(stage=name,reusable=cached,
                                  existing_unbound_outputs=[str(f) for f in outputs[name] if f.exists() and not cached])))
            report['stages'].append(old if cached else dict(name=name,outputs={}))
            continue
        if cached:
            report['stages'].append(dict(old,reused=True));save(target,report);continue
        existing=[f for f in outputs[name] if f.exists()]
        if existing and old is None:
            raise ValueError('Unbound existing evidence must be preserved: '+str(existing))
        # A failed/changed derived stage may retry; archive its prior outputs.
        for f in existing:
            backup=p/'processing_backups'/a.run/(f.name+'.'+sha(f)[:16])
            backup.parent.mkdir(parents=True,exist_ok=True)
            if not backup.exists(): f.replace(backup)
            else: f.unlink()
        start=time.monotonic()
        command=([ 'bash',str(t/'run_offline_checks.sh'),str(t/tool),*args]
                 if name=='original_candidates' else [sys.executable,str(t/tool),*args])
        code=timed(events,a.cycle+'_'+name,'postprocess',command)
        if code==0 and not all(f.is_file() for f in outputs[name]):
            raise ValueError('Stage returned0 without all required outputs: '+name)
        recorded_outputs=list(outputs[name])
        if code==0 and name=='key_node_clips':
            recorded_outputs += [Path(r['path']) for r in json.loads((p/(label+'_key_nodes/manifest.json')).read_text())['clips']]
        report['stages'].append(dict(name=name,returncode=code,elapsed_s=time.monotonic()-start,
                                    required=required,tool_sha256=sha(t/tool),binding=binding,
                                    outputs=file_bindings([f for f in recorded_outputs if f.is_file()])))
        save(target,report)
        if code and required:raise RuntimeError('Required postprocessing failed: '+name)
    if a.plan:return
    report['all_required_stages_passed']=True
    report['support_original_report_passed']=all(s['returncode']==0 for s in report['stages'] if s['name']=='original_support_review')
    report['evidence_checks_passed']=json.loads((p/(a.cycle+'_execution_evidence.json')).read_text())['evidence_checks_passed']
    if a.final_home:
        report['original_execution_evidence_checks_passed']=report['evidence_checks_passed']
        report['final_home_evidence_passed']=json.loads((p/(a.cycle+'_original_candidates_review.json')).read_text()).get('final_home_review',{}).get('passed') is True
        report['evidence_checks_passed']=report['evidence_checks_passed'] and report['final_home_evidence_passed']
    report['limits']=['Process completion and cached media do not certify assembly. Original nonzero reports remain visible and require interpretation.']
    save(target,report)
    print(json.dumps(report))
if __name__=='__main__':main()
