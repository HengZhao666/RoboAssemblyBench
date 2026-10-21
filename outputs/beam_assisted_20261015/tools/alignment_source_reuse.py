"""Strict v21->v22 source/config projection; no simulator or blanket path reuse."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import sys
import tarfile
from beam_workflow import save, sha

p = Path(__file__).resolve().parents[1]
a = p / 'assisted_repository'
sys.path.insert(0, str(a))
SK = 'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
ST = 'toolkits/factory_dual_franka_assembly/beam_assisted_state.py'
DI = 'toolkits/factory_dual_franka_assembly/beam_physics_diagnostics.py'
CO = 'roboassemblybench/core/beam_assisted_compiler.py'
EX = 'toolkits/factory_dual_franka_assembly/export_lerobot.py'
GE = 'toolkits/factory_dual_franka_assembly/generate_demos.py'
TESTS = ['tests/toolkits/test_beam_assisted_object_servo.py', 'tests/toolkits/test_beam_physics_diagnostics.py']


def method(tree, name):
    return next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)


def dumped(tree):
    return ast.dump(tree, include_attributes=False)


def projected(text, name, original):
    tree = ast.parse(text)
    if name == SK:
        class Remove(ast.NodeTransformer):
            def visit_FunctionDef(self, node):
                return None if node.name == '_assisted_alignment_reference' else self.generic_visit(node)
        Remove().visit(tree)
        f = method(tree, '_assisted_absolute_object_servo_pose')
        f.body = [n for n in f.body if not (isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Tuple)
                    and any(isinstance(v, ast.Name) and v.id == 'servo_object' for v in n.targets[0].elts))]
        class Inputs(ast.NodeTransformer):
            def visit_Name(self, node):
                names = dict(servo_object='object_pose', servo_target='target_object_pose', servo_model='model_object')
                if node.id in names: node.id = names[node.id]
                return node
        Inputs().visit(f)
    elif name == CO:
        class Remove(ast.NodeTransformer):
            def visit_Assign(self, node):
                if (isinstance(node.targets[0], ast.Subscript)
                        and isinstance(node.targets[0].slice, ast.Constant)
                        and node.targets[0].slice.value == 'beam_assisted_alignment_hold'):
                    return None
                return self.generic_visit(node)
        Remove().visit(tree)
    elif name == ST:
        f = method(tree, '_assisted_guard_actions')
        old = method(ast.parse(original), '_assisted_guard_actions')
        assignment = next(n for n in ast.walk(old) if isinstance(n, ast.Assign)
                          and any(isinstance(v, ast.Name) and v.id == 'handoff_trace' for v in n.targets))
        class Trace(ast.NodeTransformer):
            def visit_Assign(self, node):
                names = {v.id for v in node.targets if isinstance(v, ast.Name)}
                if names & {'trace_spec', 'trace_ticks'}: return None
                if names == {'handoff_trace'}: return copy.deepcopy(assignment)
                return self.generic_visit(node)
        Trace().visit(f)
    elif name == DI:
        f = method(tree, '_handoff_capture_cycle')
        assert not any(isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store) for n in ast.walk(f))
        for n in ast.walk(f):
            if isinstance(n, ast.Call):
                assert ((isinstance(n.func, ast.Name) and n.func.id in {'getattr', 'type', 'bool', 'str', 'len'})
                        or (isinstance(n.func, ast.Attribute) and n.func.attr in {'get', 'endswith'})), 'capture selector has an unreviewed call'
        old = method(ast.parse(original), '_handoff_capture_cycle')
        f.body = copy.deepcopy(old.body)
    elif name == EX:
        tree.body = [n for n in tree.body if not isinstance(n, ast.FunctionDef) or n.name != '_encode_mp4_from_pngs']
    elif name == GE:
        f = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LiveRolloutVideoRecorder')
        f.body = [n for n in f.body if not isinstance(n, ast.FunctionDef) or n.name != 'finalize']
    return tree


def main():
    cycle = sys.argv[1]
    with tarfile.open(p / 'versions/assisted_v21_before_A18_beam_full.tar.gz') as stream:
        old = {m.name: stream.extractfile(m).read() for m in stream.getmembers() if m.isfile()}
    after = json.loads((p / 'runs/A18_beam_full/source_after.json').read_text())
    assert not after['differences'] and not after['tool_differences']
    assert {n: hashlib.sha256(v).hexdigest() for n, v in old.items()} == {r['path']: r['sha256'] for r in after['files']}
    current = json.loads((p / 'tools/assisted_manifest.json').read_text())
    assert len(current) == 889 and all(sha(a / n) == value for n, value in current.items())
    changed = sorted(n for n, value in current.items() if value != hashlib.sha256(old[n]).hexdigest())
    assert changed == sorted([SK, ST, DI, CO, EX, GE] + TESTS), changed
    for name in (SK, ST, DI, CO, EX, GE):
        baseline = old[name].decode()
        expected = projected(baseline, name, baseline) if name in (EX, GE) else ast.parse(baseline)
        assert dumped(projected((a / name).read_text(), name, baseline)) == dumped(expected), name
    for name in (SK, CO):
        text = (a / name).read_text()
        altered = text.replace('.0015', '.0016', 1)
        assert altered != text and dumped(projected(altered, name, old[name].decode())) != dumped(ast.parse(old[name])), 'projection negative control'
    cfg = json.loads((p / (cycle + '_actual_configuration_layout505_config.json')).read_text())
    oldcfg = json.loads((p / 'D02_actual_configuration_layout505_config.json').read_text())
    def strip(value):
        count = 0
        if isinstance(value, dict):
            if 'beam_assisted_alignment_hold' in value:
                expected = dict(maximum_lateral_compensation_m=.00012, maximum_integral_step_m=.000002,
                                integral_gain_per_native_tick=.02, cold_actual_axial_limit_m=.0005)
                assert value.pop('beam_assisted_alignment_hold') == expected
                assert value.get('beam_assisted_descent_admission') and not value.get('beam_assisted_phase_handoff_ticks')
                count += 1
            count += sum(strip(v) for v in value.values())
        elif isinstance(value, list): count += sum(strip(v) for v in value)
        return count
    stripped = copy.deepcopy(cfg)
    assert strip(stripped) == 40
    from roboassemblybench.core.task_registry import load_task_recipe, task_recipe_fingerprint
    payload = load_task_recipe('fabrica_beam_ur5e_assisted', scene_profile='taoyuan_grscenes_tabletop')
    assert task_recipe_fingerprint(payload) == cfg['recipe_fingerprint']
    assert strip(payload) == 40 and task_recipe_fingerprint(payload) == oldcfg['recipe_fingerprint']
    stripped['recipe_fingerprint'] = oldcfg['recipe_fingerprint']
    assert stripped == oldcfg, 'configuration differs beyond40 explicitly reviewed hold flags'
    geometry = json.loads((p / 'D02_handoff_geometry.json').read_text())
    free = [r for r in geometry['results'] if r['kind'] != 'cold_admission_inherited_downward_lead']
    assert len(free) == 25 and sum(r['mesh_states'] for r in free) == 1875
    evidence = ['D02_handoff_geometry.json', 'D02_loaded_edge_entries.json', 'D02_warm_hover_equivalence.json',
                'D02_pickup_reuse.json', 'D01_handoff_geometry.json', 'D02_postprocess_pipeline.json']
    report = dict(passed=True, source_files=889, changed_paths=changed, whole_AST_projection=True,
                  unchanged_numeric_negative_controls=2, configuration_flag_count=40,
                  source_manifest_sha256=sha(p / 'tools/assisted_manifest.json'),
                  tool_sha256=sha(__file__), old_evidence_sha256={n: sha(p / n) for n in evidence},
                  reused_free_fixtures=25, reused_free_mesh_states=1875,
                  excluded_old_cold_fixtures=4, native_physics_executed=0,
                  scope='Unchanged free-move paths and original final-q guards only. Four old frozen-native cold fixtures are excluded. New closed-loop biased-plant/hold/corridor behavior needs separate fresh evidence; media functions independently byte-checked.')
    save(p / (cycle + '_source_reuse.json'), report)
    print(json.dumps({k: report[k] for k in ('passed', 'source_files', 'configuration_flag_count', 'reused_free_mesh_states')}))


if __name__ == '__main__':
    main()
