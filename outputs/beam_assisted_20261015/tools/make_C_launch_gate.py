"""Create a source-bound C launch gate only after all six real checks pass.

This is a coordination helper, outside the frozen critical-tool manifest. It
never starts a simulator. Failed/incomplete reports are reported as blockers;
an older gate for the same run is not silently reused.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import sys
from pathlib import Path


class Blocked(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise Blocked(message)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise Blocked('missing/incomplete JSON: ' + str(path) + ': ' + str(exc)) from exc


def config_value(value):
    return value['task_configs'][0] if 'task_configs' in value else value


def line_count(path):
    with Path(path).open() as stream:
        return sum(1 for line in stream if line.strip())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015'))
    parser.add_argument('--run-name', default='A13_beam_full')
    parser.add_argument('--cycle', default='C01')
    parser.add_argument('--version', default='v17b')
    parser.add_argument('--config-label', default='C01_actual_configuration_preflight_v17b')
    parser.add_argument('--checks-log', default='C01_checks_v17b.log')
    parser.add_argument('--published-corridor-report', default='C01_full_published_corridor_final.json')
    parser.add_argument('--expected-checks', type=int, default=363)
    parser.add_argument('--workflow-version', default='W03.4')
    parser.add_argument('--geometry-reuse-certificate', type=Path)
    parser.add_argument('--camera-base-prefix-report', type=Path)
    parser.add_argument('--camera-base-prefix-execution', type=Path)
    parser.add_argument('--dry-run', action='store_true', help='validate without writing a gate or summaries')
    args = parser.parse_args()
    p, t, a, r = args.root, args.root/'tools', args.root/'assisted_repository', args.root.parent
    cfg_path = p/(args.config_label+'_layout505_config.json')
    cfg = config_value(read(cfg_path))
    cfg_sha = sha(cfg_path)
    evidence_paths = set()

    def evidence(name):
        path = Path(name)
        if not path.is_absolute():
            path = p/path
        require(path.is_file(), 'missing evidence: '+str(path))
        evidence_paths.add(str(path))
        return {'path': str(path), 'sha256': sha(path)}

    def check_hashes(mapping, base=p):
        require(isinstance(mapping, dict) and mapping, 'empty source/input binding')
        for name, expected in mapping.items():
            path = Path(name)
            if not path.is_absolute():
                path = base/path
            require(path.is_file() and sha(path) == expected, 'stale bound input: '+str(path))
            evidence_paths.add(str(path))

    reuse_certificate = None
    reuse_certificate_path = args.geometry_reuse_certificate
    if reuse_certificate_path is not None:
        if not reuse_certificate_path.is_absolute():
            reuse_certificate_path = p/reuse_certificate_path
        reuse_certificate = read(reuse_certificate_path)
        require(reuse_certificate.get('passed') is True
                and reuse_certificate.get('kind') == 'exact_camera_only_configuration_reuse'
                and reuse_certificate.get('excluded_top_level_fields') in
                    (['camera_metadata'],['camera_metadata','recipe_fingerprint']),
                'invalid explicit camera-only reuse certificate')
        old_entry, new_entry = reuse_certificate['old_configuration'], reuse_certificate['new_configuration']
        old_path, new_path = Path(old_entry['path']), Path(new_entry['path'])
        require(old_path.is_file() and new_path == cfg_path
                and sha(old_path) == old_entry['sha256'] and sha(new_path) == new_entry['sha256'] == cfg_sha,
                'camera-only reuse certificate configuration bytes changed')
        old_value = config_value(read(old_path))
        excluded = reuse_certificate['excluded_top_level_fields']
        if 'recipe_fingerprint' in excluded:
            derived = reuse_certificate['derived_recipe_fingerprint']
            require(derived.get('method') == 'actual_compiled_payload_and_right_camera_counterfactual',
                    'recipe fingerprint was excluded without an explicit derivation proof')
            payload_entry = derived['payload_evidence']
            payload_path = Path(payload_entry['path'])
            require(payload_path.is_file() and sha(payload_path) == payload_entry['sha256'], 'compiled payload evidence changed')
            proof = read(payload_path)
            require(proof['kind'] == 'actual_compiled_recipe_payload_for_fingerprint_replay'
                    and proof['simulator_created'] is False
                    and proof['recipe'] == derived['recipe'] == 'fabrica_beam_ur5e_assisted'
                    and proof['scene_profile'] == derived['scene_profile'] == 'taoyuan_grscenes_tabletop',
                    'payload proof is for another recipe/profile')
            expected_env = {
                'INTERNUTOPIA_EXTENSION_PROFILE':'ur5e_assembly','PYTHONNOUSERSITE':'1',
                'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'1','BEAM_COLUMN3_PICKUP_DEPTH_M':'-0.004',
                'BEAM_COLUMN3_TRANSPORT_RAISE_M':'0.16','BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N':'16,24',
                'BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S':'0.20','BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N':'16,24',
                'BEAM_LOADED_TARGET_HOLD_SECONDS':'0','BEAM_GRIPPER_SHELL_VISUALS':'1'}
            require({k:proof['compiler_environment'].get(k) for k in expected_env} == expected_env,
                    'payload does not use the actual configuration launcher environment')
            require(proof['source_sha256'] == derived['source_sha256'], 'payload fingerprint source coverage changed')
            check_hashes(proof['source_sha256'])
            original_env = {k:os.environ.get(k) for k in proof['compiler_environment']}
            sys.path.insert(0,str(a))
            try:
                for key,value in proof['compiler_environment'].items():
                    if value is None: os.environ.pop(key,None)
                    else: os.environ[key] = value
                from roboassemblybench.core.task_registry import load_task_recipe, task_recipe_fingerprint
                payload = load_task_recipe(proof['recipe'],scene_profile=proof['scene_profile'])
                require(payload == proof['payload'], 'saved payload differs from actual compiled source')
                require(task_recipe_fingerprint(payload) == proof['computed_fingerprint']
                        == derived['new_fingerprint'] == cfg['recipe_fingerprint'], 'actual new recipe fingerprint was not recomputed')
                shadow = copy.deepcopy(payload)
                camera_key = 'camera_specs' if 'camera_specs' in shadow else 'cameras'
                right = [(i,v) for i,v in enumerate(shadow[camera_key]) if v.get('view_type') == 'wrist'
                         and v.get('robot',v.get('owner')) == 'franka_right']
                require(len(right) == 1 and camera_key == derived['camera_payload_key']
                        and right[0][0] == derived['right_camera_payload_index'], 'right recipe camera mapping changed')
                old_camera = next(v for v in old_value['camera_metadata'] if v.get('view_type') == 'wrist'
                                  and v.get('robot',v.get('owner')) == 'franka_right')
                new_camera = next(v for v in cfg['camera_metadata'] if v.get('view_type') == 'wrist'
                                  and v.get('robot',v.get('owner')) == 'franka_right')
                metadata_shadow = copy.deepcopy(old_value['camera_metadata'])
                metadata_right = next(v for v in metadata_shadow if v.get('view_type') == 'wrist'
                                      and v.get('robot',v.get('owner')) == 'franka_right')
                for field in ('mount_offset','native_mount_offset'):
                    require(right[0][1][field] == new_camera[field] == derived['right_camera_new_offsets'][field]
                            and old_camera[field] == derived['right_camera_old_offsets'][field], 'payload/config right mount differs')
                    right[0][1][field] = copy.deepcopy(old_camera[field])
                    metadata_right[field] = copy.deepcopy(new_camera[field])
                require(metadata_shadow == cfg['camera_metadata'], 'metadata changed beyond the right wrist mounting offsets')
                require(task_recipe_fingerprint(shadow) == derived['old_counterfactual_fingerprint']
                        == old_value['recipe_fingerprint'], 'restoring only the right mount did not recover the old fingerprint')
            finally:
                for key,value in original_env.items():
                    if value is None: os.environ.pop(key,None)
                    else: os.environ[key] = value
            evidence(payload_path)
        require({k:v for k,v in old_value.items() if k not in excluded}
                == {k:v for k,v in cfg.items() if k not in excluded},
                'camera-only reuse attempted to hide a noncamera configuration change')
        def changed_paths(x, y, path=''):
            if isinstance(x, dict) and isinstance(y, dict):
                paths = []
                for k in sorted(set(x)|set(y)):
                    paths += [path+'/'+str(k)] if k not in x or k not in y else changed_paths(x[k],y[k],path+'/'+str(k))
                return paths
            if isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
                return [v for i,(u,w) in enumerate(zip(x,y)) for v in changed_paths(u,w,path+'/'+str(i))]
            return [] if x == y else [path]
        paths = changed_paths(old_value, cfg)
        require(paths and paths == reuse_certificate['changed_paths']
                and all(v.startswith('/camera_metadata/') or
                        (v == '/recipe_fingerprint' and 'recipe_fingerprint' in excluded) for v in paths),
                'camera-only exact change-path list differs')
        if 'recipe_fingerprint' in excluded:
            require(changed_paths(payload,shadow) == derived['changed_payload_paths'], 'counterfactual payload change paths differ')
        evidence(reuse_certificate_path)

    def report_config_bound(name, value):
        if value == cfg_sha:
            return True
        require(reuse_certificate is not None
                and value == reuse_certificate['old_configuration']['sha256']
                and reuse_certificate['reused_reports'].get(name) == sha(p/name),
                name+' is not bound to the actual config or an explicit exact camera-only reuse certificate')
        return True

    manifests = {}
    for label, root, manifest, count in (
        ('physical', r/'candidate_repository', t/'physical_manifest.json', 876),
        ('assisted', a, t/'assisted_manifest.json', 889),
        ('tools', t, t/'assisted_tools_manifest.json', 18),
    ):
        value = read(manifest)
        require(isinstance(value, dict) and len(value) == count, label+' manifest count mismatch')
        check_hashes(value, root)
        manifests[label] = {'count': count, 'sha256': sha(manifest)}

    sealed = read(p/(args.run_name+'_sealed_version.json'))
    require((sealed.get('version'), sealed.get('run'), sealed.get('source_files'), sealed.get('critical_tools'))
            == (args.version, args.run_name, 889, 18), 'sealed version/count mismatch')
    require(len(sealed.get('archives', [])) == 2, 'both source/tool rollback archives required')
    for archive in sealed['archives']:
        path = p/'versions'/Path(archive['path']).name
        require(path.is_file() and path.stat().st_size == archive['bytes'] and sha(path) == archive['sha256'],
                'sealed rollback archive missing/changed: '+str(path))

    # Recompute the semantic config hash with the production implementation;
    # file bytes and the entire-model signature are deliberately different.
    sys.path[:0] = [str(a), str(t)]
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import configuration_signature
    from check_launch_gate import validate
    cfg_report = read(p/(args.config_label+'.json'))
    require(cfg_report.get('passed') is True and cfg_report.get('simulator_created') is False,
            'actual configuration preflight is not complete/passed')
    rows = cfg_report.get('rows', [])
    require(len(rows) == 4 and {v.get('layout_seed') for v in rows} == {0, 1, 505, 777}, 'four actual layouts required')
    for row in rows:
        rr = row['report']
        require(rr.get('passed') is True and rr.get('errors') == [] and row['seed'] == 0,
                'failed actual layout configuration')
        require(row['phase_count'] == rr['phase_count'] == 98 and rr['full_mode'] is True,
                'actual configuration must cover all98 phases')
        require(rr['configuration_signature_schema'] == 'entire_task_model/v2'
                and rr['configuration_signature_excluded_fields'] == [] and rr['full_run_budget_checked'] is True,
                'full-model/budget configuration checks missing')
        require(row['max_steps'] >= rr['all_phase_budget_required'], 'phase budget not covered')
    chosen = next(v for v in rows if v['layout_seed'] == 505)
    signature = chosen['report']['configuration_signature']
    require(configuration_signature(cfg) == signature and len(cfg['phase_specs']) == 98,
            'reviewed505 config bytes do not match the full-model signature')

    log = p/args.checks_log
    require(log.is_file(), 'missing source test log')
    log_text = log.read_text()
    require(re.search(r'\b'+str(args.expected_checks)+r' passed, 2 warnings in ', log_text)
            and not re.search(r'\b[1-9]\d* (?:failed|errors?)\b', log_text), 'final source checks incomplete/failed')
    startup = read(p/'C01_startup_contract_tests.json')
    require(startup.get('passed') is True and startup.get('tests_run') == 6
            and startup.get('failures') == startup.get('errors') == 0, 'startup six contracts failed')
    require(startup['worker_sha256'] == sha(t/'assisted_worker.py')
            and startup['test_sha256'] == sha(t/'test_startup_render_contract.py'), 'startup contracts stale')
    for filename, tool in [('C01_launch_gate_contract_tests.json', 'check_launch_gate.py'),
                           ('C01_geometry_reuse_contract_tests.json', 'robust_insert_corridor.py'),
                           ('C01_split_control_scope_contract_tests.json', 'robust_insert_corridor.py')]:
        contract_test = read(p/filename)
        require(contract_test.get('passed') is True and contract_test['tool_sha256'] == sha(t/tool),
                filename+' is failed/stale')
    require(read(p/'C01_native_cap_contract_tests.json').get('passed') is True, 'five-attempt cap contract failed')

    replay = read(p/'C01_controller_replay_final.json')
    require(replay.get('passed') is True and replay.get('failures') == []
            and report_config_bound('C01_controller_replay_final.json',replay.get('actual_config_sha256')),
            'production replay failed/stale actual98 cfg')
    require(set(replay['source_sha256']) == {
        'toolkits/factory_dual_franka_assembly/'+name for name in
        ('beam_assisted_servo.py','beam_assisted_state.py','planner_primitives.py','plumbers_block_ur5e_skills.py')},
        'production replay source coverage incomplete')
    check_hashes(replay['source_sha256'], a)
    require(replay['tool_sha256'] == sha(t/'controller_descent_replay.py')
            and replay['input_sha256'] == sha(p/'C01_A12_replay_inputs.json'), 'production replay tool/input changed')
    require(replay.get('actual_native_observations') == len(replay.get('samples', [])) == 11
            and replay.get('native_physics_steps_executed') == 0
            and replay.get('old_unaligned_descent_steps'), 'A12 unaligned-descent counterexample not exercised')

    # The original nominal audit deliberately remains failed. Its failed rows
    # must be replaced by the independently audited346-state route; do not
    # turn the original report's false into a synthetic true.
    reuse = read(p/'C01_bimanual_reuse_coverage_validation.json')
    require(reuse.get('passed') is True
            and (reuse['rows'], reuse['exact_reused_geometry_rows'], reuse['new_audited_rows']) == (6306, 5960, 346)
            and reuse['original_failure_rows_removed'] == 138
            and reuse['final_geometry_exactly_inherits_verified_retreat'] is True,
            'complete nominal row reuse/replacement evidence missing')
    check_hashes(reuse['input_sha256'])
    route = p/'C01_bimanual_repaired_v2_path.jsonl'
    require(sha(route) == 'd2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7'
            and line_count(route) == 6306, 'reviewed complete route changed')
    support_path = p/'C01_right_support_path.jsonl'
    require(line_count(support_path) == 108, 'complete support acquisition route missing')
    probe = read(p/'C01_part2_clearance_probe.json')
    accepted = [v for v in probe.get('cases', []) if v.get('accepted') is True]
    require(probe.get('passed') is True and len(accepted) == 1, 'part2 replacement audit did not pass')
    probe = accepted[0]
    require(probe['samples'] == probe['audit']['samples'] == 346 and probe['audit']['meshes'] == 54
            and probe['audit']['self_checked'] is True and probe['failure'] is None
            and probe['non_target_failure_pairs'] == [] and probe['grasp_failure_count'] == 0
            and probe['right_support_failure_count'] == 0
            and probe['endpoint_max_q_difference_from_reused_suffix_rad'] <= 1e-9,
            'replacement346 full-entity audit/endpoint continuity failed')

    robust = read(p/args.published_corridor_report)
    require(robust.get('launch_qualified') is True and robust.get('geometry_passed') is True
            and robust.get('failures') == []
            and report_config_bound(args.published_corridor_report,robust['config_sha256']),
            'final published-command corridor is failed/incomplete/stale')
    require(set(robust['source_sha256']) == {str(t/name) for name in ('sweep_model.py','visual_mesh_check.py')}
            | {str(a/'toolkits/factory_dual_franka_assembly'/name) for name in
               ('beam_assisted_servo.py','plumbers_block_ur5e_skills.py','beam_assisted_state.py')},
            'published corridor production source coverage incomplete')
    check_hashes(robust['source_sha256'])
    require(robust['geometry_sha256'] == sha(p/'runs/A12_beam_full/scene_geometry.json'), 'corridor geometry changed')
    require(robust['lateral_radius_m'] == .00015 and robust['orientation_radius_rad'] == .001
            and robust['minimum_required_interface_margin_m'] >= .00005, 'corridor admission/margin changed')
    require(robust['geometry_reused_from_sha256'] == sha(p/'C01_geometric_corridor_final.json')
            and robust['geometry_reuse_original_config_supplied_sha256'] == sha(p/'C01_geometric_corridor_source99phase_config.json')
            and robust['geometry_reused_original_config_sha256'] == sha(p/'C01_geometric_corridor_source99phase_config.json'),
            'geometry-only reuse lacks its original input proof')
    full = robust['final_published_full_mesh']
    require(full.get('passed') is True and full['solve_failures'] == full['publication_failures']
            == full['intersections_above_precision'] == [] and full['covered_parts'] == ['0', '1', '2', '3'],
            'final-limit full-mesh four-part check failed')
    require(robust.get('control_coverage_mode') == 'split_hover_completion_of_source_bound_insertion_report'
            and robust.get('combined_anchor_count') == 283 and robust.get('combined_case_count') == 5943,
            'actual insertion/free-hover control scopes were not separately qualified')
    require(full['selected_anchors'] == 73 and len(full['scenarios']) == 1533
            and full['support_acquisition_full_self_samples'] == 108
            and full.get('support_acquisition_full_self_reused') is True,
            'new free-hover/acquisition coverage changed/incomplete')
    require(full['full_visible_mesh_report']['meshes'] == 54
            and full['full_visible_mesh_report']['self_checked'] is True,
            'free-hover final commands lack complete54-mesh/self coverage')
    require(all(v.get('control_scope') == 'free_hover_no_admission_contract' for v in full['scenarios']),
            'free-hover was qualified using a fabricated assembly-admission contract')
    prior_path = p/'C01_full_published_corridor.json'
    prior = read(prior_path)
    prior_full = prior['final_published_full_mesh']
    insert_reuse = robust['successful_insertion_reuse']
    require(sha(prior_path) == insert_reuse['original_report_sha256']
            == '4341049b06bd3056635a372b2775bf628f2e24c0b68b54eefa25839025310564'
            and prior.get('launch_qualified') is False
            and insert_reuse['original_report_launch_qualified'] is False
            and prior_full['passed'] is False,
            'original failed full report was changed/relabelled for reuse')
    old_failures = prior_full['solve_failures']
    require(len(old_failures) == 74 and insert_reuse['original_failures_preserved'] == old_failures
            and sum(v.get('reason') == 'compiled_descent_contract_missing' for v in old_failures) == 73
            and sum(v.get('reason') == 'missing_complete_four_part_anchor_case_coverage' for v in old_failures) == 1,
            'prior failures include something beyond the recognized missing hover scope')
    require(prior_full['publication_failures'] == prior_full['intersections_above_precision'] == []
            and len(prior_full['scenarios']) == insert_reuse['reused_insertion_cases'] == 4410
            and insert_reuse['reused_insertion_anchors'] == 210
            and insert_reuse['reused_acquisition_full_self_samples'] == 108
            and insert_reuse['no_publication_or_mesh_failure_in_reused_branch'] is True,
            'insertion partial success is incomplete or contains a publication/mesh failure')
    require(insert_reuse['source_sha256'] == prior['source_sha256'] == robust['source_sha256']
            and insert_reuse['config_sha256'] == prior['config_sha256'] == robust['config_sha256']
            and insert_reuse['path_sha256'] == prior_full['path_sha256'] == sha(route)
            and insert_reuse['geometry_sha256'] == prior['geometry_sha256'] == robust['geometry_sha256'],
            'reused insertion production source/config/path/geometry changed')
    insertion_cases = {}
    for sample in prior_full['scenarios']:
        require('_insert_' in sample['phase'], 'non-insertion sample included in insertion-only reuse')
        insertion_cases.setdefault((sample['phase'],sample['path_index']),set()).add(sample['case'])
    require(len(insertion_cases) == 210 and all(v == set(range(21)) for v in insertion_cases.values()),
            'insertion reuse omitted or duplicated an anchor error case')
    hover_cases = {}
    for sample in full['scenarios']:
        require(sample['phase'].endswith('_transport_hover'), 'non-hover sample hidden in new hover scope')
        hover_cases.setdefault((sample['phase'],sample['path_index']),set()).add(sample['case'])
    require(len(hover_cases) == 73 and all(v == set(range(21)) for v in hover_cases.values()),
            'free-hover omitted or duplicated an anchor error case')
    scopes = robust['phase_control_scopes']
    hover_specs = {v['name']:v['local_skill'] for v in cfg['phase_specs']
                   if v['name'].startswith('assemble_') and v['name'].endswith('_transport_hover')}
    require(set(scopes) == set(hover_specs) and len(scopes) == 4,
            'free-hover phase coverage differs from the actual four production phases')
    expected_hover_anchors = [41,14,7,11]
    for (name,spec),expected_count in zip(hover_specs.items(),expected_hover_anchors):
        scope = scopes[name]
        require(spec.get('beam_assisted_descent_admission') is None
                and spec.get('beam_assisted_absolute_object_servo') is True
                and scope['kind'] == 'free_hover' and scope['contract_is_not_injected'] is True
                and scope['selected_anchors'] == expected_count,
                'reported free-hover control does not match the actual branch')
        require(scope['object_position_probe_radius_m'] == spec['target_object_position_tolerance'] == .008
                and scope['object_orientation_probe_radius_rad'] == spec['target_object_orientation_tolerance'] == .15,
                'free-hover did not probe the actual8mm/.15rad stage completion bounds')
        require(scope['minimum_observed_height_above_final_m'] >= .1019
                and scope['maximum_observed_height_above_final_m'] >= scope['minimum_observed_height_above_final_m'],
                'free-hover height/phase classification differs from the reviewed route')
    require(prior_full['dangerous_probe_results'] and all(v['ready'] is False and v['aligned'] is False
            for v in prior_full['dangerous_probe_results']), 'unsafe insertion descent probes were not rejected')
    require(full['path_sha256'] == sha(route) and full['support_sha256'] == sha(p/'C01_right_support_candidate.json')
            and full['support_acquisition_path_sha256'] == sha(support_path), 'corridor route/support input stale')

    monitor = read(p/'C01_visual_support_monitor_validation.json')
    require(monitor.get('passed') is True and monitor['mesh_count'] == 54, 'support monitor validation failed')
    for key, name in {'helper':'visual_support_check.py', 'watcher':'watch_visible_meshes.py',
                      'checker':'visual_mesh_check.py', 'model':'sweep_model.py'}.items():
        require(monitor['source_sha256'][key] == sha(t/name), 'support monitor source changed: '+name)
    old_cfg_path = p/'C01_actual_configuration_preflight_layout505_config.json'
    require(monitor['source_sha256']['config'] == sha(old_cfg_path), 'original monitor config input changed')
    old_cfg = config_value(read(old_cfg_path))
    def monitor_projection(value):
        phases = value['phase_specs']
        return {'scene': {k:value[k] for k in ('scene_asset_path','scene_position','scene_orientation','scene_scale','robots','objects')},
                'support_metadata': {v['name']:v['beam_assisted_visual_support'] for v in phases if v.get('beam_assisted_visual_support')},
                'initial_support_phase':next(v for v in phases if v['name']=='base_6_visual_support'),
                'base_assembled_target':value['target_poses']['part_6_assembled']}
    require(monitor_projection(old_cfg) == monitor_projection(cfg), 'monitor old/new configuration projection differs')
    active = [v['name'] for v in cfg['phase_specs'] if v.get('beam_assisted_visual_support')
              and v['name'] != 'base_6_visual_support']
    require(len(active) == 84 and monitor['phase_contract']['runtime_checked_phase_names'] == active,
            'monitor does not cover actual84 assembly support phases')
    require(monitor['reuse']['second_FCL_models_created_by_support_helper'] == 0
            and monitor['dynamic_pose_membership']['required_dynamic_bodies'] == 37
            and monitor['dynamic_pose_membership']['complete_actual_sample_accepted'] is True
            and all(v['accepted'] is False for v in monitor['dynamic_pose_membership']['missing_cases']),
            'monitor reuse/missing-native-pose checks failed')
    require(len(monitor['cases']) == 6 and all(v['result']['ready'] is v['expected_ready'] for v in monitor['cases']),
            'support gap positive/negative cases failed')
    require(monitor['source_sha256']['candidate'] == sha(p/'C01_right_support_candidate.json')
            and monitor['source_sha256']['geometry'] == sha(p/'runs/A12_beam_full/scene_geometry.json'),
            'monitor candidate/native geometry changed')
    monitor_reuse = {'passed': True, 'scope': 'only visible-support monitor contract and original acquisition geometry; not controller/whole-route reuse',
        'old_configuration': evidence(old_cfg_path), 'actual_configuration': evidence(cfg_path),
        'monitor_validation': evidence('C01_visual_support_monitor_validation.json'),
        'matching_monitored_phase_count':84, 'scene_robots_objects_initial_support_and_base_target_identical':True,
        'separate_nominal_replacement_and_final_controller_checks_required':True}

    camera = read(p/'C01_camera_full_path.json')
    execution = read(p/'C01_camera_execution.json')
    require(execution.get('returncode') == 0
            and execution.get('source_unchanged') is True
            and execution['source_sha256_before'] == execution['source_sha256_after'], 'camera execution incomplete/failed/source changed')
    check_hashes(execution['source_sha256_before'])
    require({str(t/'camera_visibility_preflight.py'),camera['geometry'],str(route),str(support_path)}
            <= set(execution['source_sha256_before']), 'camera execution tool/path source coverage incomplete')
    require(execution['report_sha256'] == sha(p/'C01_camera_full_path.json')
            and execution.get('old_base_camera_report_sha256',execution.get('reused_base_prefix_report_sha256'))
            == sha(p/'B02_camera_full_path.json'), 'camera report/base reuse changed')
    require(execution['new_path_counts'] == {'C01_bimanual_repaired_v2_path.jsonl':6306,'C01_right_support_path.jsonl':108}
            and execution['new_path_rows'] == 6414 and execution['camera_checks'] == 12828,
            'camera source row coverage incomplete')
    require(camera['visible_meshes'] == 54 and camera['stride'] == 1 and camera['max_samples'] == 0
            and camera['pad_samples'] == 9 and camera['min_visible'] == 2 and camera['min_eye_clearance'] == .01
            and camera['closure_sweep'] is False and camera['start_index'] == 0 and camera['end_index'] is None,
            'camera frustum/pad/eye/sample coverage changed')
    offsets = {c['robot']:c['native_mount_offset'] for c in cfg['camera_metadata'] if c.get('view_type') == 'wrist'}
    require(set(offsets) == {'franka_left','franka_right'}
            and all(c['native_mount_offset'] == c['mount_offset'] for c in cfg['camera_metadata'] if c.get('view_type') == 'wrist'),
            'actual wrist mount metadata is missing/inconsistent')
    require(camera['trajectories'] == [str(route),str(support_path)], 'camera did not review actual support/route')
    result = camera['results']['reviewed_side']
    require(result['samples'] == 12828, 'initial two-camera source coverage incomplete')
    base_camera = read(p/'B02_camera_full_path.json')
    old_camera_result = base_camera['results']['reviewed_side']
    require(old_camera_result['pinch_visibility_failures'] == old_camera_result['eye_failures'] == 0
            and old_camera_result['min_visible_samples'] >= 2, 'retained base-prefix source camera check failed')
    prefix_evidence = []
    composite_camera = None
    if (result['pinch_visibility_failures'] == result['eye_failures'] == 0
            and result['first_failures'] == [] and result['min_visible_samples'] >= 2):
        require(execution.get('passed') is True and camera['offsets'] == {'reviewed_side':offsets},
                'passing full camera report uses different actual offsets')
    else:
        # Preserve the failed aggregate. Its complete per-robot failure counts
        # can qualify the unchanged left camera, never the failed right camera.
        original_offsets = camera['offsets']['reviewed_side']
        require(set(original_offsets) == {'franka_left','franka_right'}
                and original_offsets['franka_left'] == offsets['franka_left']
                == base_camera['offsets']['reviewed_side']['franka_left'],
                'unchanged-left-camera reuse requires identical left mounts')
        counts = result['failure_phase_counts']
        require(counts and sum(counts.values()) == result['pinch_visibility_failures']
                and all(key.endswith(':franka_right') and value > 0 for key,value in counts.items())
                and result['eye_failures'] <= result['pinch_visibility_failures'],
                'initial camera counts do not prove zero left visibility/eye failures')
        camera_source = (t/'camera_visibility_preflight.py').read_text()
        require('pinch_visibility_pass=visible_ok and separation>=12 and not eye_bad' in camera_source,
                'per-robot visibility failures no longer include every eye failure')
        right_path, right_exec_path = p/'C01_camera_right_final_v2_path.json', p/'C01_camera_right_final_v2_execution.json'
        right, right_exec = read(right_path), read(right_exec_path)
        require(right_exec.get('passed') is True and right_exec.get('returncode') == 0
                and right_exec.get('source_unchanged') is True
                and right_exec['source_sha256_before'] == right_exec['source_sha256_after']
                and right_exec['report_sha256'] == sha(right_path), 'refreshed right-camera execution failed/stale')
        check_hashes(right_exec['source_sha256_before'])
        right_trajectories = [str(route),str(support_path),str(p/'base_v11_capture_path.jsonl'),
                              str(p/'runs/A10b_beam_full/entity_geometry_trace.jsonl')]
        require({str(t/'camera_visibility_preflight.py'),right['geometry'],*right_trajectories}
                <= set(right_exec['source_sha256_before']), 'right camera tool/path source coverage incomplete')
        require(right_exec['source_counts'] == {
            'C01_bimanual_repaired_v2_path.jsonl':6306,'C01_right_support_path.jsonl':108,
            'base_v11_capture_path.jsonl':1289,'entity_geometry_trace.jsonl':1063}
                and right_exec['camera_checks'] == 8766, 'right camera complete base/support/assembly coverage missing')
        require([line_count(v) for v in right_trajectories] == [6306,108,1289,1063],
                'right camera actual path row counts differ from its execution summary')
        require(all(right[key] == camera[key] for key in (
            'geometry','visible_meshes','stride','max_samples','start_index','end_index',
            'closure_sweep','pad_samples','min_visible','min_eye_clearance')),
                'refreshed right camera altered geometry/sample/visibility settings')
        require(right['trajectories'] == right_trajectories
                and right['offsets'] == {'right_raised_side':{'franka_right':offsets['franka_right']}},
                'refreshed right camera does not use actual mount or complete path')
        rr = right['results']['right_raised_side']
        require(rr['samples'] == 8766 and rr['pinch_visibility_failures'] == rr['eye_failures'] == 0
                and rr['first_failures'] == [] and rr['min_visible_samples'] >= 2, 'refreshed right camera failed')
        prefix_reuse = read(p/'C01_camera_base_prefix_reuse_v2.json')
        require(prefix_reuse.get('passed') is True and prefix_reuse['mesh_count'] == 54
                and prefix_reuse['mesh_identity_and_triangle_topology_equal'] is True
                and prefix_reuse['all_fk_mapping_inputs_exact'] is True
                and prefix_reuse['max_vertex_coordinate_difference_m'] <= 1e-14
                and prefix_reuse['body_rest_matrix_max_difference'] == prefix_reuse['joint_graph_matrix_max_difference']
                == prefix_reuse['native_initial_joint_max_difference_rad'] == 0.
                and prefix_reuse['camera_controls_match'] is True
                and prefix_reuse['prior_visibility_failures'] == prefix_reuse['prior_eye_failures'] == 0
                and prefix_reuse['prior_min_visible_samples'] >= 2 and prefix_reuse['old_report_stride'] == 1,
                'unchanged-left base-prefix geometry/FK/camera reuse was not proven')
        for path_key, sha_key in [('old_report','old_report_sha256'),('old_geometry','old_geometry_sha256'),
                                  ('new_geometry','new_geometry_sha256'),('base_prefix','base_prefix_sha256'),
                                  ('A10b_native_prefix','A10b_native_prefix_sha256')]:
            require(sha(Path(prefix_reuse[path_key])) == prefix_reuse[sha_key], 'base-prefix reuse input changed: '+path_key)
        require(prefix_reuse['old_report'] == str(p/'B02_camera_full_path.json')
                and prefix_reuse['old_geometry'] == base_camera['geometry']
                and prefix_reuse['new_geometry'] == camera['geometry'] == right['geometry'],
                'base-prefix reuse geometry/report mapping differs')
        composite_camera = {'passed':True,
            'scope':'unchanged left6414 nominal/support states plus retained left base prefix; refreshed right8766 complete states',
            'unchanged_left_assembly_support_checks':6414,'unchanged_left_base_prefix_checks':2352,
            'refreshed_right_complete_checks':8766,
            'initial_aggregate_failures_retained':result['pinch_visibility_failures'],
            'initial_complete_failure_counts_prove_left_failures_zero':True,
            'camera_failure_predicate_includes_every_eye_failure':True,
            'actual_offsets':offsets,
            'initial_failed_aggregate':evidence('C01_camera_full_path.json'),
            'initial_execution':evidence('C01_camera_execution.json'),
            'new_right_report':evidence(right_path),'new_right_execution':evidence(right_exec_path),
            'unchanged_left_base_prefix_geometry_reuse':evidence('C01_camera_base_prefix_reuse_v2.json'),
            'unchanged_left_base_prefix_source':evidence('B02_camera_full_path.json'),
            'native_images_still_required':True}
        prefix_evidence += [str(right_path),str(right_exec_path),'B02_camera_full_path.json',
                            'C01_camera_base_prefix_reuse_v2.json','C01_camera_combined_launch_review.json']
    if base_camera['offsets'] != camera['offsets'] and composite_camera is None:
        # A new wrist offset invalidates the old offset's base-prefix image
        # geometry. Motion geometry can be reused; camera geometry cannot.
        require(args.camera_base_prefix_report is not None and args.camera_base_prefix_execution is not None,
                'changed camera offset needs a new base-prefix camera report and execution binding')
        prefix_report_path, prefix_execution_path = args.camera_base_prefix_report, args.camera_base_prefix_execution
        if not prefix_report_path.is_absolute(): prefix_report_path = p/prefix_report_path
        if not prefix_execution_path.is_absolute(): prefix_execution_path = p/prefix_execution_path
        prefix, prefix_exec = read(prefix_report_path), read(prefix_execution_path)
        require(prefix_exec.get('passed') is True and prefix_exec.get('returncode') == 0
                and prefix_exec.get('source_unchanged') is True
                and prefix_exec['source_sha256_before'] == prefix_exec['source_sha256_after']
                and prefix_exec['report_sha256'] == sha(prefix_report_path), 'new base-prefix camera execution failed/stale')
        check_hashes(prefix_exec['source_sha256_before'])
        prefix_paths = base_camera['trajectories'][:-1]
        require(prefix['trajectories'] == prefix_paths and prefix['offsets'] == camera['offsets']
                and prefix['stride'] == 1 and prefix['max_samples'] == 0
                and prefix['start_index'] == 0 and prefix['end_index'] is None
                and prefix['pad_samples'] == 9 and prefix['min_visible'] == 2
                and prefix['min_eye_clearance'] == .01 and prefix['visible_meshes'] == 54,
                'new base-prefix camera coverage/offset/settings changed')
        pr = prefix['results']['reviewed_side']
        require(pr['samples'] == 2*sum(line_count(v) for v in prefix_paths)
                and pr['pinch_visibility_failures'] == pr['eye_failures'] == 0
                and pr['first_failures'] == [] and pr['min_visible_samples'] >= 2,
                'new base-prefix camera visibility/eye check failed')
        prefix_evidence = [str(prefix_report_path),str(prefix_execution_path)]

    summaries = {
        'C01_deployment_verification.json': {'passed':True,'verified_manifests':manifests,'differences':[],
            'sealed_version':evidence(args.run_name+'_sealed_version.json')},
        'C01_checks_summary.json': {'passed':True,'source_checks_passed':args.expected_checks,'startup_contract_checks_passed':6,
            'source_manifest_sha256':manifests['assisted']['sha256'],'tools_manifest_sha256':manifests['tools']['sha256'],
            'source_log':evidence(log),'startup_report':evidence('C01_startup_contract_tests.json'),'dynamic_success':False},
        'C01_monitor_configuration_reuse.json':monitor_reuse,
    }
    if composite_camera is not None:
        summaries['C01_camera_combined_launch_review.json'] = composite_camera
    if args.dry_run:
        print(json.dumps({'passed':True,'dry_run':True,'native_run_started':False,'all_six_checks_ready':True}))
        return
    for name, value in summaries.items():
        (p/name).write_text(json.dumps(value,indent=2)+'\n')
    definitions = [
        ('actual_configuration_and_targets','four98-phase actual layouts; complete-model signature and original randomized targets',
         [args.config_label+'.json',str(cfg_path)]),
        ('full_phase_release_completion_interfaces','source regressions, native startup contracts and explicit release/final/cap interfaces; no dynamics claim',
         ['C01_checks_summary.json','C01_startup_contract_tests.json','C01_launch_gate_contract_tests.json','C01_geometry_reuse_contract_tests.json','C01_split_control_scope_contract_tests.json','C01_native_cap_contract_tests.json']),
        ('full_entities_and_camera_visibility','complete nominal route by exact5960-row reuse and independently audited346-row replacement; both wrists and right support acquisition; native images still required',
         ['C01_bimanual_reuse_coverage_validation.json','C01_camera_full_path.json','C01_camera_execution.json','C01_part2_clearance_probe.json']),
        ('production_controller_object_space_replay','A12 observed unaligned descent counterexample evaluated in object space with native-anchored compensation; no native steps',
         ['C01_controller_replay_final.json']),
        ('admitted_error_and_final_command_geometry','source-bound210-anchor/4410 insertion-case reuse plus73-anchor/1533 actual free-hover cases; final-limit verifier and complete54-mesh/self checks; sampled envelope',
         [args.published_corridor_report,'C01_full_published_corridor.json','C01_geometric_corridor_final.json']),
        ('bimanual_support_lifecycle','unattached right visual pinch during all84 assembly phases; complete acquisition/clearance/open/retreat and missing dynamic-pose rejection',
         ['C01_monitor_configuration_reuse.json','C01_visual_support_monitor_validation.json','C01_right_support_candidate.json','C01_bimanual_reuse_coverage_validation.json']),
    ]
    if reuse_certificate_path is not None:
        for index in (3,4): definitions[index][2].append(str(reuse_certificate_path))
    definitions[2][2].extend(prefix_evidence)
    gate = {'route':'assisted','run_name':args.run_name,'cycle':args.cycle,'workflow_version':args.workflow_version,
        'source_manifest_sha256':manifests['assisted']['sha256'],'tools_manifest_sha256':manifests['tools']['sha256'],
        'actual_configuration_signature':signature,'native_initialization_check_required':True,'mandatory_unresolved':[],
        'right_arm_visual_support_required':True,'control_safety_metric':'object_space_after_final_joint_limits',
        'formal_physical_success':False,'checks':[{'name':name,'passed':True,'scope':scope,
            'evidence':[evidence(v) for v in files]} for name,scope,files in definitions]}
    validation = validate(gate,run_name=args.run_name,source_manifest=t/'assisted_manifest.json',tools_manifest=t/'assisted_tools_manifest.json')
    gate_path = p/'launch_gates'/(args.run_name+'.json')
    require(not gate_path.exists(), 'refusing to overwrite existing launch gate: '+str(gate_path))
    gate_path.parent.mkdir(exist_ok=True)
    gate_path.write_text(json.dumps(gate,indent=2)+'\n')
    (p/'C01_launch_gate_validation.json').write_text(json.dumps({'passed':True,'gate':evidence(gate_path),
        'validation':validation,'source_bound_evidence_files':len(evidence_paths),'native_run_started':False},indent=2)+'\n')
    print(json.dumps({'passed':True,'gate':str(gate_path),'gate_sha256':sha(gate_path),'validation':validation,'native_run_started':False}))


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, TypeError, OSError, StopIteration, IndexError) as exc:
        print(json.dumps({'passed':False,'native_run_started':False,'blocker':str(exc)}))
        raise SystemExit(2)
