"""Build C04's six source-bound checks; this tool never starts native physics.

Old motion/camera evidence is reusable only through the exact support-idle AST
whitelist. The nine new geometry rows are declared counterfactuals, not a new
native trajectory. A fresh complete source suite is required.
"""
import argparse
import ast
import copy
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import tarfile

P = Path(__file__).resolve().parents[1]
T = P / 'tools'
A = P / 'assisted_repository'
R = P.parent
STATE = 'toolkits/factory_dual_franka_assembly/beam_assisted_state.py'
TEST = 'tests/toolkits/test_beam_idle_hold.py'
PRIOR_GATE_SHA = 'db9008c96bff772b04256348ca4ec493f65c1374bee8b4ecf7580a18b1a63702'
OLD_ARCHIVE_SHA = 'b504dd794329b6d022f24610b77b45ae78c4e72376655f04de2178bdde5b6ad4'
OLD_STATE_SHA = '6faeca159b7545120798c5fa78506f8f93d64f5d9d44c4eb9d591f9244b13baa'
NATIVE_INPUT_SHA = '8e7bc1b534e80ea126b899f274379e8db4911e18abe7d49476133675bf6fc6ad'
CONFIG_SHA = 'b535c703e75985b5c9dee845b97cf4a9c008d3949f894620b5d44ba0b74cd38e'
IDLE_AST_SHA = '588385f369d28278fd120a98b4dfb6fdef6a9c973829a006ccbd4cb5c2cb82f3'
META_AST_SHA = '614462b57d2d1944e9ecf9ae63ee24504c39ce898b1e1139a208b3e01c055644'
AST_CONTROLS = {
    'exact_two_reviewed_insertions_accepted', 'unchanged_old_source_rejected', 'comment_only_change_accepted',
    'shared_guard_math_change_rejected', 'ordinary_idle_change_rejected', 'freshness_relaxation_rejected',
    'active_branch_change_rejected', 'unrelated_module_change_rejected',
}
CHECK_NAMES = {
    'actual_configuration_and_targets', 'full_phase_release_completion_interfaces',
    'full_entities_and_camera_visibility', 'production_controller_object_space_replay',
    'admitted_error_and_final_command_geometry', 'bimanual_support_lifecycle',
}
CONTRACT_NAMES = set('''current_control_positive declared_counterfactual_geometry_positive exact_state_and_test_only_positive whole_module_ast_restored_positive
empty_passed_control_rejected control_source_missing_rejected control_target_jump_rejected control_intermediate_native_claim_rejected control_missing_test_rejected
empty_passed_geometry_rejected geometry_source_missing_rejected geometry_input_missing_rejected geometry_wrong_tool_rejected geometry_mesh53_rejected geometry_self_disabled_rejected
geometry_collision_rejected geometry_false_native_claim_rejected geometry_interpolant_count_rejected geometry_missing_state_rejected geometry_changed_gap_limit_rejected geometry_old_failure_erased_rejected geometry_missing_clock_receipt_rejected geometry_actual_step_changed_rejected
critical_tool_change_rejected shared_guard_math_change_rejected freshness_relaxation_rejected closed3_pause_after_C04_positive unclosed_A15_rejected C05_authorization_rejected
fresh385_configuration_positive old379_suite_rejected missing_layout_rejected reuse_positive empty_passed_reuse_rejected reuse_missing_original_evidence_rejected reuse_missing_input_rejected
reuse_old379_rejected reuse_AST_control_missing_rejected reuse_scope_widening_rejected reuse_stale_source_rejected'''.split())


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    path = Path(path)
    return json.loads((path if path.is_absolute() else P / path).read_text())


def bound(path):
    path = Path(path)
    path = path if path.is_absolute() else P / path
    return {'path': str(path), 'sha256': sha(path)}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value):
    return isinstance(value, list) and len(value) == 6 and all(finite(v) for v in value)


def delta(a, b):
    require(vector(a) and vector(b), 'missing/nonfinite complete six-joint vector')
    return max(abs(x - y) for x, y in zip(a, b))


def exact_mapping(actual, expected, label):
    require(isinstance(actual, dict) and actual == expected and bool(expected), label + ' exact binding set/hash mismatch')


def validate_transition(current, tools, old, oldtools):
    require(isinstance(current, dict) and isinstance(old, dict) and len(current) == len(old) == 889,
            'complete889 source maps required')
    require(set(current) == set(old), 'source path set changed')
    require(sorted(n for n in old if old[n] != current[n]) == sorted([STATE, TEST]),
            'C04 may change only support state and its existing idle tests')
    require(isinstance(tools, dict) and len(tools) == 19 and tools == oldtools,
            'all19 critical tools must remain byte-identical A15')


def restored_state_ast(old_text, new_text):
    old, new = ast.parse(old_text), ast.parse(new_text)
    idle = [n for n in ast.walk(new) if isinstance(n, ast.FunctionDef) and n.name == '_assisted_idle_action']
    guard = [n for n in ast.walk(new) if isinstance(n, ast.FunctionDef) and n.name == '_assisted_guard_actions']
    require(len(idle) == len(guard) == 1, 'unique real idle/guard methods required')
    idle = idle[0]
    assignment = [n for n in idle.body if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'support' for t in n.targets)]
    condition = ast.dump(ast.parse("support.get('robot') == robot_name", mode='eval').body, include_attributes=False)
    branch = [n for n in idle.body if isinstance(n, ast.If) and condition == ast.dump(n.test, include_attributes=False)]
    require(len(assignment) == len(branch) == 1 and len(branch[0].orelse) == 2,
            'support-only idle structure differs from reviewed whitelist')
    require(ast.dump(assignment[0], include_attributes=False) == ast.dump(ast.parse("support = phase_spec.get('beam_assisted_visual_support') or {}").body[0], include_attributes=False)
            and hashlib.sha256(ast.dump(branch[0], include_attributes=False).encode()).hexdigest() == IDLE_AST_SHA,
            'support-only assignment/branch differs from reviewed exact AST')
    restored = []
    for n in idle.body:
        if n is assignment[0]:
            continue
        restored.extend(branch[0].orelse if n is branch[0] else [n])
    idle.body = restored
    matches = []
    for parent in ast.walk(guard[0]):
        for _, value in ast.iter_fields(parent):
            if isinstance(value, list):
                for node in value:
                    if isinstance(node, ast.If) and any(isinstance(c, ast.Constant) and c.value == 'beam_assisted_visual_support' for c in ast.walk(node.test)):
                        matches.append((value, node))
    require(len(matches) == 1, 'exactly one support publication metadata block required')
    require(hashlib.sha256(ast.dump(matches[0][1], include_attributes=False).encode()).hexdigest() == META_AST_SHA,
            'support publication metadata differs from reviewed exact AST')
    matches[0][0].remove(matches[0][1])
    require(ast.dump(new, include_attributes=False) == ast.dump(old, include_attributes=False),
            'unapproved source AST change outside support-idle branch/guard metadata')
    return True


def validate_control(report, current, native_sha, archive_sha):
    require(report.get('schema') == 'beam_C04_support_handoff_control_v1' and report.get('passed') is True,
            'support handoff control report missing/failed')
    exact_mapping(report.get('source_sha256'), {STATE: current[STATE], TEST: current[TEST]}, 'control source')
    inputs = report.get('input_sha256')
    require(isinstance(inputs, dict) and len(inputs) == 1 and next(iter(inputs.values())) == native_sha
            and Path(next(iter(inputs))).name == 'A15_support_handoff_native_inputs.json',
            'control must bind preserved A15 native input (local/server path relocation is explicit)')
    require(report.get('old_source_archive_sha256') == archive_sha == OLD_ARCHIVE_SHA
            and report.get('old_state_sha256') == OLD_STATE_SHA, 'control original source/archive unbound')
    for key in ('state_support_idle_and_metadata_only_ast', 'guard_numerical_ast_unchanged',
                'driver_logic_unchanged', 'ordinary_idle_ast_unchanged', 'old_native_counterexample_reproduced'):
        require(report.get(key) is True, 'missing control contract ' + key)
    require(report.get('native_physics_executed') == 0 and report.get('force_support_claimed') is False,
            'offline handoff replay cannot claim native dynamics/load support')
    require(report.get('controlled_replay_tick_count') == 9 and report.get('fixed_lifecycle_tests_passed') == 6
            and report.get('scoped_file_tests_run') == report.get('scoped_file_tests_passed') == 32,
            'support lifecycle coverage missing')
    original, fixed = report['results']['original'], report['results']['fixed']
    require((original['tests_run'], original['tests_passed'], fixed['tests_run'], fixed['tests_passed']) == (6, 2, 6, 6),
            'old counterexample/new six-test result missing')
    require(original['source_sha256'] == OLD_STATE_SHA and fixed['source_sha256'] == current[STATE],
            'old/new production replay source mismatch')
    require(len(original['failed_test_methods']) == 4 and original['test_failures']
            and fixed['failed_test_methods'] == fixed['test_failures'] == fixed['test_errors'] == [],
            'control replay missing real old failures or contains new failures')
    target = fixed['last_active_final_published_q']
    oldjump = delta(original['first_idle_policy_action_sdk_input_q'], target)
    require(abs(oldjump - .0006451382548284812) < 1e-12
            and abs(original['first_idle_target_jump_rad'] - oldjump) < 1e-12
            and abs(report['observed_old_target_jump_rad'] - oldjump) < 1e-12, 'A15 old measured-anchor target jump not reproduced')
    require(delta(fixed['first_idle_policy_action_sdk_input_q'], target) <= 1e-12
            and fixed['first_idle_target_jump_rad'] <= 1e-12 and report['new_first_idle_target_delta_rad'] <= 1e-12,
            'new first SDK-input target is discontinuous')
    trace = fixed['controlled_tick_trace']
    require(isinstance(trace, list) and [v['step'] for v in trace] == list(range(6536, 6545)),
            'nine declared replay ticks missing')
    for item in trace:
        require(vector(item['actual_q']) and delta(item['policy_action_sdk_input_q'], target) <= 1e-12
                and finite(item['max_delta_from_last_active_published_rad']) and item['max_delta_from_last_active_published_rad'] <= 1e-12
                and item['gripper_controller'] is None, 'replay target/driver changed')
        expected = 'actual A15 native q' if item['step'] in (6536, 6544) else 'controlled repeat of step6536 q; not native measurement'
        require(item['input_kind'] == expected, 'controlled intermediate replay mislabeled native')


def validate_mesh(mesh, count):
    require(isinstance(mesh, dict) and (mesh.get('samples'), mesh.get('meshes'), mesh.get('self_checked')) == (count, 54, True),
            'complete54 mesh/self/sample coverage missing')
    require(type(mesh.get('pair_checks')) is int and mesh['pair_checks'] > 0
            and isinstance(mesh.get('intersections'), list), 'mesh pairs/intersections missing')
    for v in mesh['intersections']:
        require(finite(v.get('depth')) and v['depth'] <= 1e-6, 'non-target overlap exceeds1um')
    require(not mesh.get('missing_dynamic_poses'), 'missing dynamic poses')


def validate_support(observation, ready):
    require(observation.get('ready') is ready, 'unexpected support observation ready state')
    fingers = observation.get('fingers')
    require(isinstance(fingers, list) and len(fingers) == 2, 'both complete support fingers required')
    require({v.get('finger') for v in fingers} == {'left_inner_finger', 'right_inner_finger'}
            and observation.get('robot') == 'franka_right' and observation.get('object') == 'fabrica_beam_6'
            and observation.get('force_support_claimed') is False, 'support robot/object/finger mapping changed')
    require(all(finite(v.get('signed_gap_m')) for v in fingers), 'support signed gaps missing/nonfinite')
    if ready:
        require(all(5e-6 <= v['signed_gap_m'] <= 250e-6 for v in fingers), 'support gap outside frozen5–250um')


def validate_geometry(report, sources, inputs, tool_sha, original_failure):
    require(report.get('schema') == 'beam_C04_support_idle_geometry_v1' and report.get('passed') is True
            and report.get('errors') == [], 'support geometry missing/failed')
    exact_mapping(report.get('source_sha256'), sources, 'geometry source')
    exact_mapping(report.get('input_sha256'), inputs, 'geometry input')
    require(len(sources) == 3 and len(inputs) == 9 and report.get('tool_sha256') == tool_sha, 'geometry source/input/tool coverage')
    require(report.get('native_physics_executed') == 0 and report.get('formal_physical_success') is False,
            'counterfactual geometry must not claim native success')
    endpoints = report.get('actual_endpoints')
    require(isinstance(endpoints, list) and [(v['cycle_step'], v['physics_index']) for v in endpoints] == [(6536, 6539), (6544, 6547)],
            'both exact native endpoint indices required')
    for v in endpoints:
        require(finite(v['native_body_entity_position_error_m']) and v['native_body_entity_position_error_m'] < 1e-6
                and finite(v['native_body_entity_angle_error_rad']) and v['native_body_entity_angle_error_rad'] < 1e-6
                and type(v['body_count']) is int and v['body_count'] >= 37, 'native/entity endpoint mapping incomplete')
    clock = report.get('clock_join', {})
    require(clock.get('entity_has_own_native_stamp') is False and clock.get('phase_labels_are_not_join_keys') is True,
            'native/entity clock limitations missing')
    receipts = clock.get('direct_epoch_index_receipts', [])
    cameras = {'observation.images.front', 'observation.images.left_wrist', 'observation.images.right_wrist'}
    require(len(receipts) == 6 and {(v['task_step_label'], v['camera']) for v in receipts} == {(s, c) for s in (6536, 6544) for c in cameras},
            'three same-clock camera receipts per endpoint required')
    for v in receipts:
        stamp = v['physics_stamp']
        require(stamp['valid'] is True and stamp['epoch'] == 0 and stamp['index'] == v['task_step_label'] + 3
                and finite(stamp['time']), 'endpoint native epoch/index/time invalid')
    require(clock.get('published_first_stamp') == next(v['physics_stamp'] for v in receipts if v['task_step_label'] == 6536),
            'first published stamp does not match direct native clock receipt')
    actual = report['actual_failed_reproduction']
    require(actual.get('original_failure_step') == 6544 and len(actual.get('observations', [])) == 2,
            'original actual support failure must remain visible')
    require([v.get('step') for v in actual['observations']] == [6536, 6544], 'actual observations must use both exact preserved endpoint steps')
    validate_mesh(actual['full54_self_mesh'], 2)
    validate_support(actual['observations'][0], True)
    validate_support(actual['observations'][1], False)
    for got, expected in zip(actual['observations'][1]['fingers'], original_failure['fingers']):
        require(got['finger'] == expected['finger'] and abs(got['signed_gap_m'] - expected['signed_gap_m']) < 1e-10, 'actual failure differs from original monitor gap')
    require(any(v['signed_gap_m'] > 250e-6 for v in actual['observations'][1]['fingers']), 'original gap failure erased')
    cf = report['counterfactual']
    require((cf.get('total_states'), cf.get('actual_left_endpoints'), cf.get('interpolated_left_states'),
             cf.get('actual_complete_native_states_claimed'), cf.get('right_fixed_native_step')) == (9, 2, 7, 0, 6536)
            and cf.get('native_left_endpoint_steps') == [6536, 6544], 'counterfactual scope changed or mislabeled native')
    validate_mesh(cf['full54_self_mesh'], 9)
    obs = cf['support_observations']
    require(len(obs) == 9 and [v['counterfactual_step_label'] for v in obs] == list(range(6536, 6545)),
            'all nine counterfactual labels required')
    for v in obs:
        require(v.get('right_replaced_with_6536') is True and v.get('left_endpoint_observed') is (v['counterfactual_step_label'] in (6536, 6544)),
                'right fixed/left interpolation provenance missing')
        validate_support(v, True)


def validate_configuration(report, log):
    require(report.get('passed') is True and report.get('simulator_created') is False,
            'fresh actual configuration failed')
    rows = report.get('rows', [])
    require(len(rows) == 4 and {v['layout_seed'] for v in rows} == {0, 1, 505, 777}, 'four actual layouts required')
    for v in rows:
        q = v['report']
        require(q['passed'] is True and q['errors'] == [] and v['phase_count'] == q['phase_count'] == 98
                and q['configuration_signature_schema'] == 'entire_task_model/v2'
                and q['configuration_signature_excluded_fields'] == [] and q['full_run_budget_checked'] is True
                and q['all_phase_budget_required'] <= v['max_steps'], 'fresh actual layout incomplete')
    matches = re.findall(r'(\d+) passed, (\d+) warnings? in ', log)
    require(matches == [('385', '2')] and not re.search(r'\b[1-9]\d* (?:failed|errors?)\b', log),
            'fresh full source suite incomplete (requires385;379 is historical)')
    return next(v['report']['configuration_signature'] for v in rows if v['layout_seed'] == 505)


def validate_batch(batch):
    require((batch['native_attempts_started'], batch['completed_cycles'], batch['maximum_native_attempts'], batch['workflow_version'])
            == (3, 3, 5, 'W03.10') and batch['next_native_run_authorized'] is True
            and batch.get('pause_after_cycle') == 'C04' and batch.get('pause_after_evidence_sync') is True
            and batch.get('C05_authorized') is False, 'C04 bounded authorization/count/pause boundary mismatch')


def validate_reuse(report, current, tools, refs, evidence, unchanged, tool_sha):
    require(report.get('schema') == 'beam_C04_exact_motion_reuse_v1' and report.get('passed') is True
            and report.get('errors') == [], 'C04 AST-whitelist reuse missing/failed')
    require(report.get('source_files') == 889 and report.get('critical_tools') == 19, 'reuse source/tool counts')
    exact_mapping(report.get('current_source_sha256'), current, 'reuse current source')
    exact_mapping(report.get('critical_tools_sha256'), tools, 'reuse critical tools')
    require(report.get('tool_sha256') == tool_sha, 'reuse tool source unbound')
    change = report['source_change']
    require(sorted(change.get('changed_paths', [])) == sorted([STATE, TEST])
            and change.get('allowed_methods') == ['_assisted_idle_action', '_assisted_guard_actions'], 'reuse source whitelist widened')
    require(all(change.get(v) is True for v in ('whole_state_ast_restored', 'guard_numerical_ast_unchanged', 'ordinary_idle_ast_unchanged')),
            'reuse full-state AST/numerical preservation proof missing')
    require(change.get('old_state_sha256') == OLD_STATE_SHA and change.get('new_state_sha256') == current[STATE]
            and change.get('reviewed_idle_branch_ast_sha256') == IDLE_AST_SHA
            and change.get('reviewed_metadata_if_ast_sha256') == META_AST_SHA, 'reuse AST/source identity unbound')
    require(set(report.get('ast_contract_controls', {})) == AST_CONTROLS
            and all(v is True for v in report['ast_contract_controls'].values())
            and report.get('native_physics_executed') == 0, 'eight actual AST whitelist controls required')
    for name, expected in refs.items():
        exact_mapping(report.get(name), expected, 'reuse reference ' + name)
    exact_mapping(report.get('original_gate_evidence'), evidence, 'original gate evidence')
    exact_mapping(report.get('unchanged_inputs'), unchanged, 'unchanged inputs')
    require(report.get('reused_scope') == dict(cached_cases=600, cached_full54_self_states=3775,
            original_C01_cases=5943, nominal_path_states=6306, source_tests_reused=0)
            and report.get('fresh_source_tests_required') is True, 'reuse scope widened or old379 source tests reused')


def archive_members(seal, count, *, tools=False):
    records = [v for v in seal['archives'] if ('_tools_' in Path(v['path']).name) is tools]
    require(len(records) == 1, 'unique source/tools rollback archive required')
    record = records[0]
    path = P / 'versions' / Path(record['path']).name
    require(path.stat().st_size == record['bytes'] and sha(path) == record['sha256'], 'sealed archive bytes/hash changed')
    with tarfile.open(path) as bundle:
        if tools:
            entries = [m for m in bundle.getmembers() if m.name == 'assisted_tools_manifest.json']
            require(len(entries) == 1, 'archive tool manifest absent/ambiguous')
            manifest = json.loads(bundle.extractfile(entries[0]).read())
        else:
            manifest = {m.name: hashlib.sha256(bundle.extractfile(m).read()).hexdigest()
                        for m in bundle.getmembers() if m.isfile()}
        require(len(manifest) == count, 'archive manifest count mismatch')
    return path, manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract-tests', action='store_true')
    parser.add_argument('--config-label', default='C04_actual_configuration_preflight_v19')
    parser.add_argument('--checks-log', default='C04_checks_v19.log')
    args = parser.parse_args()
    if args.contract_tests:
        return contract_tests()
    run = 'A16_beam_full'
    out = P / 'launch_gates' / (run + '.json')
    require(not out.exists(), 'refuse gate overwrite')
    prior_path = P / 'launch_gates/A15_beam_full.json'
    require(sha(prior_path) == PRIOR_GATE_SHA, 'C03 final gate changed')
    prior = read(prior_path)
    require({v['name'] for v in prior['checks']} == CHECK_NAMES and len(prior['checks']) == 6,
            'C03 six independent checks incomplete')
    prior_evidence = {}
    for row in prior['checks']:
        require(row['passed'] is True and row['evidence'], 'prior gate incomplete')
        for v in row['evidence']:
            require(sha(v['path']) == v['sha256'], 'prior evidence changed ' + v['path'])
            prior_evidence[v['path']] = v['sha256']
    current, tools = read(T / 'assisted_manifest.json'), read(T / 'assisted_tools_manifest.json')
    for root, manifest, count in ((A, current, 889), (T, tools, 19), (R / 'candidate_repository', read(T / 'physical_manifest.json'), 876)):
        require(len(manifest) == count, 'current authoritative manifest count mismatch')
        for name, value in manifest.items():
            require(sha(root / name) == value, 'current bytes changed ' + str(root / name))
    after_path = P / 'runs/A15_beam_full/source_after.json'
    after = read(after_path)
    require(after['differences'] == after['tool_differences'] == []
            and len(after['files']) == 889 and len(after['tool_files']) == 19, 'A15 source/tool after not clean')
    old = {v['path']: v['sha256'] for v in after['files']}
    oldtools = {v['path']: v['sha256'] for v in after['tool_files']}
    validate_transition(current, tools, old, oldtools)
    oldseal = read('A15_beam_full_sealed_version.json')
    oldarchive, archived_old = archive_members(oldseal, 889)
    oldtoolsarchive, archived_tools = archive_members(oldseal, 19, tools=True)
    require(archived_old == old and archived_tools == oldtools and sha(oldarchive) == OLD_ARCHIVE_SHA,
            'A15 sourceafter differs from sealed original source/tools')
    require(prior['source_manifest_sha256'] == oldseal.get('source_manifest_sha256'), 'prior source authority mismatch')
    require(prior['tools_manifest_sha256'] == sha(T / 'assisted_tools_manifest.json'), 'prior critical19 authority mismatch')
    with tarfile.open(oldarchive) as bundle:
        members = [m for m in bundle.getmembers() if m.name == STATE or m.name.endswith('/' + STATE)]
        require(len(members) == 1, 'original state archive member ambiguous')
        oldtext = bundle.extractfile(members[0]).read().decode()
    require(hashlib.sha256(oldtext.encode()).hexdigest() == OLD_STATE_SHA, 'original state source changed')
    restored_state_ast(oldtext, (A / STATE).read_text())
    seal = read(run + '_sealed_version.json')
    require((seal['run'], seal['version'], seal['source_files'], seal['critical_tools']) == (run, 'v19', 889, 19), 'wrong C04 seal')
    _, archived_current = archive_members(seal, 889)
    _, archived_current_tools = archive_members(seal, 19, tools=True)
    require(archived_current == current and archived_current_tools == tools, 'current seal manifest differs')
    native_input = P / 'A15_support_handoff_native_inputs.json'
    require(sha(native_input) == NATIVE_INPUT_SHA, 'preserved native input changed')
    control_path = P / 'C04_support_handoff_control_validation.json'
    control = read(control_path)
    validate_control(control, current, sha(native_input), sha(oldarchive))
    geometry_path = P / 'C04_support_idle_geometry.json'
    geometry = read(geometry_path)
    run_dir = P / 'runs/A15_beam_full'
    input_paths = [native_input] + [run_dir / v for v in ('scene_geometry.json', 'task_config_snapshot.json',
        'entity_geometry_trace.jsonl', 'physics_boundary_trace.jsonl', 'stop_request.json', 'source_after.json',
        'video_frame_clock.jsonl', 'support_native_phase_boundary_review.json')]
    geom_inputs = {str(v): sha(v) for v in input_paths}
    geom_sources = {str(T / n): sha(T / n) for n in ('sweep_model.py', 'visual_mesh_check.py', 'visual_support_check.py')}
    original_failure = read(run_dir / 'stop_request.json')['first_failure']['visual_support']
    validate_geometry(geometry, geom_sources, geom_inputs, sha(T / 'c04_support_idle_geometry.py'), original_failure)
    reuse_path = P / 'C04_exact_motion_reuse.json'
    reuse = read(reuse_path)
    refs = {k: bound(v) for k, v in dict(reference_gate=prior_path, source_after=after_path, source_archive=oldarchive, tools_archive=oldtoolsarchive,
            control_validation=control_path, geometry_validation=geometry_path).items()}
    # Exact original input objects are independently constructed from the current
    # certificate's fixed names below, then rehashed, never inferred by count.
    unchanged = expected_unchanged_inputs(P / (args.config_label + '_layout505_config.json'))
    evidence_map = {k: dict(path=k, sha256=v, expected_sha256=v, unchanged=True) for k, v in prior_evidence.items()}
    validate_reuse(reuse, current, tools, refs, evidence_map, unchanged, sha(T / 'c04_exact_motion_reuse.py'))
    cfgpath = P / (args.config_label + '_layout505_config.json')
    require(sha(cfgpath) == CONFIG_SHA, 'C04 actual config changed relative to reused geometric/camera evidence')
    cfgreport = read(args.config_label + '.json')
    log_path = P / args.checks_log
    signature = validate_configuration(cfgreport, log_path.read_text())
    sys.path[:0] = [str(A), str(T)]
    from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import configuration_signature
    from check_launch_gate import validate
    require(signature == configuration_signature(read(cfgpath)) == prior['actual_configuration_signature'], 'actual full-model signature changed')
    tests_path = P / 'C04_launch_gate_contract_tests.json'
    tests = read(tests_path)
    require(tests.get('schema') == 'beam_C04_gate_evidence_contract_tests_v1' and tests.get('passed') is True
            and tests.get('generator_sha256') == sha(__file__) and tests.get('pure_memory') is True
            and tests.get('native_started') is False and len(tests.get('checks', [])) == len(CONTRACT_NAMES)
            and {v.get('name') for v in tests['checks']} == CONTRACT_NAMES
            and all(v.get('passed') is True for v in tests['checks']), 'C04 gate contract tests missing/stale')
    batch = read('repair_batch_C_20261015.json')
    validate_batch(batch)
    summary = dict(passed=True, source_checks_passed=385, source_checks_reused=0,
                   source_manifest=bound(T / 'assisted_manifest.json'), tools_manifest=bound(T / 'assisted_tools_manifest.json'),
                   source_log=bound(log_path), source_count=889, critical_tool_count=19, native_started=False)
    summary_path = P / 'C04_checks_summary.json'
    summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    scopes = {
        'actual_configuration_and_targets': 'fresh four actual98-phase configurations and unchanged full-model signature; exact motion/camera input reuse',
        'full_phase_release_completion_interfaces': 'fresh385 complete source tests including six support inactive/active/open/freshness lifecycle regressions; zero source tests reused',
        'full_entities_and_camera_visibility': 'AST-whitelist-bound reuse of6306 nominal states/cameras; new two actual and nine declared counterfactual54mesh+self states',
        'production_controller_object_space_replay': 'preserved actual A15 target-jump counterexample and real Policy→idle→finalguard replay; nine controlled inputs, two native endpoints',
        'admitted_error_and_final_command_geometry': 'exact support-only AST reuse of600 cached cases/3775 full54+self states and5943 original cases; unchanged shared numeric guard',
        'bimanual_support_lifecycle': 'same-run fresh final-published inactive right-arm target, original gap failure reproduced; nine frozen-right counterfactual states; native sustained support remains unvalidated',
    }
    common_paths = [prior_path, reuse_path, geometry_path, control_path, tests_path, P / (run + '_sealed_version.json'),
                    summary_path, P / (args.config_label + '.json'), cfgpath, Path(__file__), T / 'c04_exact_motion_reuse.py', T / 'c04_support_idle_geometry.py']
    gate = copy.deepcopy(prior)
    gate.update(run_name=run, cycle='C04', workflow_version='W03.10', source_manifest_sha256=sha(T / 'assisted_manifest.json'),
                tools_manifest_sha256=sha(T / 'assisted_tools_manifest.json'), actual_configuration_signature=signature,
                mandatory_unresolved=[], formal_physical_success=False, pause_after_cycle='C04', pause_after_evidence_sync=True, C05_authorized=False)
    for row in gate['checks']:
        row['scope'] = scopes[row['name']]
        row['evidence'] += [bound(v) for v in common_paths]
    validation = validate(gate, run_name=run, source_manifest=T / 'assisted_manifest.json', tools_manifest=T / 'assisted_tools_manifest.json')
    out.write_text(json.dumps(gate, indent=2) + '\n')
    result = dict(passed=True, gate=bound(out), validation=validation, generator=bound(__file__),
                  source_checks=385, source_checks_reused=0, native_started=False)
    (P / 'C04_launch_gate_validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    return result


def expected_unchanged_inputs(config_path):
    expected = {
        'actual_configuration': (config_path, CONFIG_SHA),
        'geometric_corridor_source_geometry': (P / 'runs/A12_beam_full/scene_geometry.json', 'aac97863c2b892136d9a238bbea92d2c7740684b360c3cfc5b5eddf7a98c38fb'),
        'nominal_bimanual_path': (P / 'C01_bimanual_repaired_v2_path.jsonl', 'd2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7'),
        'right_support_candidate': (P / 'C01_right_support_candidate.json', '8c248424e4f6c4db970d6df4a8d87171bebbd3b68d382e04dafad635ce50109e'),
        'right_support_acquisition_path': (P / 'C01_right_support_path.jsonl', 'd15d23f448ab6fdb66fd371268d755b9ce540b22aad60e5e0b8eef1537022461'),
        'camera_full_report': (P / 'C01_camera_full_path.json', '0f3482bb761327244bee764830fbab894c649fc8afca0dc1cc16dd2b243862df'),
        'camera_right_report': (P / 'C01_camera_right_final_v2_path.json', '5423aaa9b5554fba34b83a28e435489b517c2009f497940ad46ebd186c8227d2'),
    }
    out = {}
    for name, (path, expected_sha) in expected.items():
        require(sha(path) == expected_sha, 'reused input changed ' + name)
        out[name] = dict(path=str(path), sha256=expected_sha, expected_sha256=expected_sha, unchanged=True)
    return out


def contract_tests():
    # Reports are loaded as fixtures, then all positive/negative checks below run
    # in memory. No simulation, IK, mesh construction, or production mutation.
    results = []
    def check(name, call, reject=False):
        try:
            call()
            passed = not reject
        except (ValueError, KeyError, TypeError, StopIteration, SyntaxError):
            passed = reject
        results.append(dict(name=name, passed=passed))
    current = read(T / 'assisted_manifest.json')
    tools = read(T / 'assisted_tools_manifest.json')
    oldarchive = P / 'versions/assisted_v18_before_A14_beam_full.tar.gz'
    with tarfile.open(oldarchive) as bundle:
        old = {v.name: hashlib.sha256(bundle.extractfile(v).read()).hexdigest() for v in bundle.getmembers() if v.isfile()}
        oldtext = bundle.extractfile(STATE).read().decode()
    newtext = (A / STATE).read_text()
    control = read('C04_support_handoff_control_validation.json')
    geometry = read('C04_support_idle_geometry.json')
    geom_sources, geom_inputs = copy.deepcopy(geometry['source_sha256']), copy.deepcopy(geometry['input_sha256'])
    original_failure = copy.deepcopy(geometry['actual_failed_reproduction']['observations'][1])
    geom_tool = sha(T / 'c04_support_idle_geometry.py')
    def ctl(v): return validate_control(v, current, NATIVE_INPUT_SHA, OLD_ARCHIVE_SHA)
    def geo(v): return validate_geometry(v, geom_sources, geom_inputs, geom_tool, original_failure)
    def mutate(value, operation, validator):
        clone = copy.deepcopy(value)
        operation(clone)
        validator(clone)
    check('current_control_positive', lambda: ctl(control))
    check('declared_counterfactual_geometry_positive', lambda: geo(geometry))
    check('exact_state_and_test_only_positive', lambda: validate_transition(current, tools, old, tools))
    check('whole_module_ast_restored_positive', lambda: restored_state_ast(oldtext, newtext))
    cases = [
        ('empty_passed_control_rejected', control, lambda x: (x.clear(), x.update(passed=True)), ctl),
        ('control_source_missing_rejected', control, lambda x: x['source_sha256'].pop(TEST), ctl),
        ('control_target_jump_rejected', control, lambda x: x['results']['fixed']['first_idle_policy_action_sdk_input_q'].__setitem__(1, 0.), ctl),
        ('control_intermediate_native_claim_rejected', control, lambda x: x['results']['fixed']['controlled_tick_trace'][1].update(input_kind='actual A15 native q'), ctl),
        ('control_missing_test_rejected', control, lambda x: x.update(fixed_lifecycle_tests_passed=5), ctl),
        ('empty_passed_geometry_rejected', geometry, lambda x: (x.clear(), x.update(passed=True)), geo),
        ('geometry_source_missing_rejected', geometry, lambda x: x['source_sha256'].pop(next(iter(x['source_sha256']))), geo),
        ('geometry_input_missing_rejected', geometry, lambda x: x['input_sha256'].pop(next(iter(x['input_sha256']))), geo),
        ('geometry_wrong_tool_rejected', geometry, lambda x: x.update(tool_sha256='0'*64), geo),
        ('geometry_mesh53_rejected', geometry, lambda x: x['counterfactual']['full54_self_mesh'].update(meshes=53), geo),
        ('geometry_self_disabled_rejected', geometry, lambda x: x['counterfactual']['full54_self_mesh'].update(self_checked=False), geo),
        ('geometry_collision_rejected', geometry, lambda x: x['counterfactual']['full54_self_mesh']['intersections'].append(dict(depth=2e-6)), geo),
        ('geometry_false_native_claim_rejected', geometry, lambda x: x['counterfactual'].update(actual_complete_native_states_claimed=9), geo),
        ('geometry_interpolant_count_rejected', geometry, lambda x: x['counterfactual'].update(interpolated_left_states=6), geo),
        ('geometry_missing_state_rejected', geometry, lambda x: x['counterfactual']['support_observations'].pop(), geo),
        ('geometry_changed_gap_limit_rejected', geometry, lambda x: x['counterfactual']['support_observations'][0]['fingers'][0].update(signed_gap_m=.000251), geo),
        ('geometry_old_failure_erased_rejected', geometry, lambda x: x['actual_failed_reproduction']['observations'][1].update(ready=True), geo),
        ('geometry_missing_clock_receipt_rejected', geometry, lambda x: x['clock_join']['direct_epoch_index_receipts'].pop(), geo),
        ('geometry_actual_step_changed_rejected', geometry, lambda x: x['actual_failed_reproduction']['observations'][0].update(step=6535), geo),
    ]
    for name, value, change, validator in cases:
        check(name, lambda v=value, c=change, f=validator: mutate(v, c, f), True)
    check('critical_tool_change_rejected', lambda: validate_transition(current, {**tools, 'assisted_worker.py': '0'*64}, old, tools), True)
    check('shared_guard_math_change_rejected', lambda: restored_state_ast(oldtext, newtext.replace('bounded_native_target(target[0], actual, previous.get(name))', 'bounded_native_target(target[0], actual+.001, previous.get(name))', 1)), True)
    check('freshness_relaxation_rejected', lambda: restored_state_ast(oldtext, newtext.replace('not in (0, 1)', 'not in (0, 1, 2)', 1)), True)
    batch = dict(native_attempts_started=3, completed_cycles=3, maximum_native_attempts=5, workflow_version='W03.10',
                 next_native_run_authorized=True, pause_after_cycle='C04', pause_after_evidence_sync=True, C05_authorized=False)
    check('closed3_pause_after_C04_positive', lambda: validate_batch(batch))
    check('unclosed_A15_rejected', lambda: validate_batch({**batch, 'completed_cycles': 2}), True)
    check('C05_authorization_rejected', lambda: validate_batch({**batch, 'C05_authorized': True}), True)
    cfg = read('C02_actual_configuration_preflight_v18.json')
    check('fresh385_configuration_positive', lambda: validate_configuration(cfg, '385 passed, 2 warnings in 175.97s'))
    check('old379_suite_rejected', lambda: validate_configuration(cfg, '379 passed, 2 warnings in 1s'), True)
    check('missing_layout_rejected', lambda: mutate(cfg, lambda x: x['rows'].pop(), lambda x: validate_configuration(x, '385 passed, 2 warnings in 1s')), True)
    reuse = read('C04_exact_motion_reuse.json')
    reuse_refs = {k: copy.deepcopy(reuse[k]) for k in ('reference_gate', 'source_after', 'source_archive', 'tools_archive', 'control_validation', 'geometry_validation')}
    reuse_evidence = copy.deepcopy(reuse['original_gate_evidence'])
    reuse_inputs = copy.deepcopy(reuse['unchanged_inputs'])
    def reuse_check(v):
        return validate_reuse(v, current, tools, reuse_refs, reuse_evidence, reuse_inputs, sha(T / 'c04_exact_motion_reuse.py'))
    check('reuse_positive', lambda: reuse_check(reuse))
    for name, operation in [
        ('empty_passed_reuse_rejected', lambda x: (x.clear(), x.update(passed=True))),
        ('reuse_missing_original_evidence_rejected', lambda x: x['original_gate_evidence'].pop(next(iter(x['original_gate_evidence'])))),
        ('reuse_missing_input_rejected', lambda x: x['unchanged_inputs'].pop('actual_configuration')),
        ('reuse_old379_rejected', lambda x: x['reused_scope'].update(source_tests_reused=379)),
        ('reuse_AST_control_missing_rejected', lambda x: x['ast_contract_controls'].pop('freshness_relaxation_rejected')),
        ('reuse_scope_widening_rejected', lambda x: x['reused_scope'].update(new_native_support_certified=True)),
        ('reuse_stale_source_rejected', lambda x: x['current_source_sha256'].update({STATE: OLD_STATE_SHA})),
    ]:
        check(name, lambda op=operation: mutate(reuse, op, reuse_check), True)
    require(len(results) == len(CONTRACT_NAMES) and {v['name'] for v in results} == CONTRACT_NAMES, 'contract check inventory mismatch')
    result = dict(schema='beam_C04_gate_evidence_contract_tests_v1', passed=all(v['passed'] for v in results),
                  pure_memory=True, generator_sha256=sha(__file__), checks=results, native_started=False)
    (P / 'C04_launch_gate_contract_tests.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
    require(result['passed'], 'C04 pure contract negative control failed')
    return result


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, TypeError, OSError, StopIteration, SyntaxError) as exc:
        print(json.dumps(dict(passed=False, blocker=str(exc), native_started=False)))
        raise SystemExit(2)
