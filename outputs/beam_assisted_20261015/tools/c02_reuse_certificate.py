"""C02-only, read-only proof for the narrow grasp-identity change.

This certificate preserves C01's finite cache-empty checks. Persistent command
lead and native state lifetime are separate C02 checks, never inferred here.
"""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
import tarfile

SKILLS = 'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
OLD_SKILLS = '717154fb4b288e17b474ddbaf59bb9256350405c2b24f5e4fec7f38097bef000'
OLD_FULL = 'cc8c1d6cc0c8406eb633d72cebf9be9f9efb390ede0043cecec8d2b8af684bad'
OLD_FAILED = '4341049b06bd3056635a372b2775bf628f2e24c0b68b54eefa25839025310564'

def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()

def sha(p):
    return sha_bytes(Path(p).read_bytes())

def identity_only(old, new):
    tree = ast.parse(old)
    expected = copy.deepcopy(tree)
    methods = [n for n in ast.walk(expected) if isinstance(n, ast.FunctionDef)
               and n.name == '_assisted_object_servo_context']
    if len(methods) != 1:
        return False
    assignments = [n for n in ast.walk(methods[0]) if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'identity' for t in n.targets)]
    if len(assignments) != 1 or not isinstance(assignments[0].value, ast.Tuple):
        return False
    val = assignments[0].value
    exact = ast.parse("identity = (attachment.get('joint_path'), attachment.get('attach_step'), stamp['epoch'], native.get('body_path'), native.get('motion_view_id'), tuple(np.asarray(relative[0], float)), tuple(np.asarray(relative[1], float)))").body[0].value
    if ast.dump(val) != ast.dump(exact):
        return False
    val.elts = val.elts[:5]
    return ast.dump(expected) == ast.dump(ast.parse(new))

def contract_tests(old, new):
    controls = {'exact_identity_suffix_deletion': identity_only(old, new),
                'unchanged_old_source_rejected': not identity_only(old, old),
                'comment_only_candidate_accepted': identity_only(old, new+'\n# no semantic change\n')}
    for name, before, after in (
        ('helper_call_change_rejected', 'position_lookahead=float(', 'position_lookahead=1+float('),
        ('guard_change_rejected', 'for ratio in (1., .5, .25, .125, .0625, 0.):', 'for ratio in (1., 0.):'),
        ('phase_key_change_rejected', 'states.setdefault((robot_name, phase_key), {})', 'states.setdefault((robot_name, 0), {})'),
        ('authoritative_field_change_rejected', "attachment.get('attach_step')", "attachment.get('wrong_step')")):
        changed = new.replace(before, after, 1)
        controls[name] = changed != new and not identity_only(old, changed)
    return controls

def main():
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('repository', 'tools', 'source-archive', 'tools-archive', 'after-manifest',
                 'old-gate', 'config', 'geometry', 'path', 'support', 'support-path'):
        ap.add_argument('--'+name, type=Path, required=True)
    ap.add_argument('--changed-test', required=True)
    ap.add_argument('--output', type=Path)
    a = ap.parse_args()
    p = a.old_gate.parent.parent
    errors = []
    old_manifest = json.loads(a.after_manifest.read_text())
    old_source = {v['path']: v['sha256'] for v in old_manifest['files']}
    old_tools = {v['path']: v['sha256'] for v in old_manifest['tool_files']}
    if (len(old_source), len(old_tools)) != (889, 18):
        errors.append('old manifest does not contain exact 889 source / 18 tools')
    if old_manifest['differences'] or old_manifest['tool_differences']:
        errors.append('A13 source-after was not unchanged')
    with tarfile.open(a.source_archive) as archive:
        archived = {v.name: archive.extractfile(v).read() for v in archive.getmembers() if v.isfile()}
    with tarfile.open(a.tools_archive) as archive:
        archived_tools = {v.name: archive.extractfile(v).read() for v in archive.getmembers() if v.isfile()}
    if {k: sha_bytes(v) for k,v in archived.items()} != old_source:
        errors.append('v17d source archive does not match A13 source-after')
    for name, digest in old_tools.items():
        if name not in archived_tools or sha_bytes(archived_tools[name]) != digest:
            errors.append('v17d tool archive mismatch: '+name)
    current_source = {name: sha(a.repository/name) for name in old_source}
    changed = sorted(k for k in old_source if current_source[k] != old_source[k])
    if changed != sorted([SKILLS, a.changed_test]):
        errors.append('source changes are not exactly skills plus the declared existing test')
    current_tools = {name: sha(a.tools/name) for name in old_tools}
    if current_tools != old_tools:
        errors.append('a frozen critical tool changed')
    old = archived[SKILLS].decode()
    new = (a.repository/SKILLS).read_text()
    ast_only = sha_bytes(archived[SKILLS]) == OLD_SKILLS and identity_only(old, new)
    if not ast_only:
        errors.append('whole skills AST differs beyond identity tuple 7 to 5')
    tests = contract_tests(old, new)
    if not all(tests.values()):
        errors.append('reuse certificate negative controls failed')
    gate = json.loads(a.old_gate.read_text())
    if len(gate['checks']) != 6 or not all(v['passed'] is True for v in gate['checks']):
        errors.append('original C01 six gates are not all passed')
    evidence = {}
    for check in gate['checks']:
        for e in check['evidence']:
            local = p/Path(e['path']).name
            actual = sha(local)
            evidence[local.name] = {'sha256': actual, 'expected_sha256': e['sha256'], 'original_gate': check['name']}
            if actual != e['sha256']:
                errors.append('C01 original evidence changed: '+local.name)
    full = json.loads((p/'C01_full_published_corridor_final.json').read_text())
    failed = json.loads((p/'C01_full_published_corridor.json').read_text())
    if (sha(p/'C01_full_published_corridor_final.json') != OLD_FULL
            or sha(p/'C01_full_published_corridor.json') != OLD_FAILED
            or full['launch_qualified'] is not True or failed['launch_qualified'] is not False
            or full['combined_case_count'] != 5943 or full['combined_anchor_count'] != 283):
        errors.append('original C01 split 4410 + 1533 evidence is not exact')
    same_inputs = {}
    for key, path, expected in (
        ('actual_configuration', a.config, evidence['C01_actual_configuration_preflight_v17d_layout505_config.json']['sha256']),
        ('geometry', a.geometry, full['geometry_sha256']),
        ('bimanual_path', a.path, 'd2ee32134586fc591a18817a2d9788a8ca8dd21cca0da6f8a53d9e169a890cb7'),
        ('support', a.support, evidence['C01_right_support_candidate.json']['sha256']),
        ('support_path', a.support_path, 'd15d23f448ab6fdb66fd371268d755b9ce540b22aad60e5e0b8eef1537022461')):
        digest = sha(path)
        same_inputs[key] = {'path': str(path), 'sha256': digest, 'expected_sha256': expected, 'unchanged': digest == expected}
        if digest != expected:
            errors.append('input changed: '+key)
    result = dict(schema='beam_C02_exact_identity_reuse_v1', passed=not errors, errors=errors,
        tool_sha256=sha(__file__), source_change=dict(old_source_archive_sha256=sha(a.source_archive),
        old_source_after_sha256=sha(a.after_manifest), old_tools_archive_sha256=sha(a.tools_archive),
        source_count=len(old_source), frozen_tool_count=len(old_tools), changed_paths=changed,
        identity_ast_only=ast_only, old_source_sha256=old_source, current_source_sha256=current_source,
        frozen_tools_sha256=current_tools), unchanged_inputs=same_inputs,
        original_C01_gate_sha256=sha(a.old_gate), original_C01_evidence=evidence,
        contract_controls=tests,
        reused_scope={'insert_anchors':210,'insert_cases':4410,'hover_anchors':73,'hover_cases':1533,
          'right_acquisition_full54_self_samples':108,'previous_q_equals_actual_q':True,
          'previous_object_command_is_none':True,'geometry_pair_grid_and_world_anchor_unchanged':True,
          'persistent_cache_coverage_inferred':False,'native_state_lifetime_coverage_inferred':False},
        required_new_checks=['real production context lifecycle replay','cached lead final-q full54/self supplement','current source tests/config/seal'],
        limits=['Finite C01 sample grid only; no dynamics or continuous-collision guarantee.',
                'No existing failed or passed report is rewritten.'])
    out = json.dumps(result, indent=2)+'\n'
    if a.output:a.output.write_text(out)
    else:print(out,end='')
    return 0 if result['passed'] else 1

if __name__ == '__main__':
    raise SystemExit(main())
