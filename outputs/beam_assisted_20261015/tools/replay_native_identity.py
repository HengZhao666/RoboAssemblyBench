#!/usr/bin/env python3
"""Read-only production-method replay of A13's derived-float identity defect.

This noncritical evidence tool never imports or launches Isaac, writes runtime
state, or edits production. Its proposed patch is emitted only as review text.
"""
from __future__ import annotations

import argparse
import ast
import copy
import difflib
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np


ADAPTER = 'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py'
TASK = 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
PLANNER = 'toolkits/factory_dual_franka_assembly/planner_primitives.py'
ROBUST = 'tools/robust_insert_corridor.py'
METHODS = ('_assisted_object_servo_context', '_object_tcp_relative_pose',
           '_physical_relative_pose_to_control_frame')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def function(path, name):
    found = [n for n in ast.walk(ast.parse(path.read_text()))
             if isinstance(n, ast.FunctionDef) and n.name == name]
    if len(found) != 1:
        raise ValueError(f'expected exactly one production method {name}')
    return found[0]


def compile_class(name, functions, namespace):
    node = ast.ClassDef(name=name, bases=[], keywords=[], body=copy.deepcopy(functions), decorator_list=[])
    code = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
    exec(compile(code, f'{name}:unchanged_production_method_AST', 'exec'), namespace)
    return namespace[name]


def quat_angle(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    dot = abs(float(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b))))
    return 2*math.acos(min(1., dot))


def main():
    p = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source-root', type=Path, default=p/'assisted_repository')
    ap.add_argument('--inputs', type=Path, default=p/'C01_identity_native_inputs.json')
    ap.add_argument('--out', type=Path, default=p/'C01_identity_native_replay.json')
    ap.add_argument('--patch-out', type=Path, default=p/'drafts/C02_stable_grasp_identity.patch')
    ap.add_argument('--regression-design-out', type=Path, default=p/'C02_identity_regression_design.json')
    args = ap.parse_args()
    inputs_bytes = args.inputs.read_bytes()
    data = json.loads(inputs_bytes)
    root = args.source_root.resolve()
    bindings = {rel: sha((root/rel).read_bytes()) for rel in data['source_sha256']}
    if bindings != data['source_sha256']:
        raise ValueError('replay source differs from the frozen A13 input source bindings')
    tool_path = p/ROBUST
    if sha(tool_path.read_bytes()) != data['critical_tool_sha256'][ROBUST]:
        raise ValueError('old preflight source differs from its frozen input binding')
    spec = importlib.util.spec_from_file_location('identity_replay_planner', root/PLANNER)
    pp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pp)
    namespace = dict(np=np, normalize_quat=pp.normalize_quat, quat_rotate=pp.quat_rotate,
                     quat_multiply=pp.quat_multiply, compose_pose=pp.compose_pose,
                     relative_pose=pp.relative_pose)
    nodes = {name: function(root/(TASK if name == METHODS[2] else ADAPTER), name) for name in METHODS}
    Adapter = compile_class('ReplayAdapter', [nodes[METHODS[0]], nodes[METHODS[1]],
        function(root/ADAPTER, '_tcp_offset')], namespace)
    Task = compile_class('ReplayTask', [nodes[METHODS[2]]], namespace)
    adapter, task = Adapter(), Task()
    frames = data['motion_observations']
    assert [x['step'] for x in frames] == [6504, 6505]
    cfg = data['phase_spec']['local_skill']
    assert adapter._tcp_offset(cfg) is None
    assert cfg['cartesian_orientation_step'] == .004
    assert cfg['cartesian_orientation_command_lookahead'] == .02
    # The run did not log its private attachment-relative tuple. Recover the
    # relation actually represented by the full TCP/object target pair. Tiny
    # extra roundoff in that recovery is explicit and does not claim a memory dump.
    rel_p, rel_q = pp.relative_pose(frames[0]['target_pose']['position'],
        frames[0]['target_pose']['orientation'], data['object_goal']['position'],
        data['object_goal']['orientation'])
    attachment = dict(mode='fixed_joint', robot_name='franka_right',
        joint_path='/World/env_0/episode_000000/objects/fabrica_beam_6/assembly_attachment_joint',
        attach_step=2053, position=rel_p.copy(), orientation=rel_q.copy())
    tracked = {'fabrica_beam_6': {'attachment': attachment}}
    # These getters share the same observed control pose, as the actual
    # assisted _preserve_beam_v169_task_pose branch specifies. The native body
    # stub is deliberately constant; fixed-joint/no-offset conversion does not
    # consume object_pose. This test isolates context lifetime, not physics.
    task._get_robot_task_pose = lambda robot: (task.frame['position'], task.frame['orientation'])
    task._get_robot_control_pose = task._get_robot_task_pose
    task._beam_physics_stamp = lambda: dict(valid=True, epoch=0, index=task.step+3,
        time=(task.step+3)/240, dt=1/240)
    native = copy.deepcopy(data['absolute_command_observations'][0]['current_object_pose'])
    task._assisted_native_body_motion = lambda name, epoch: dict(native,
        motion_valid=True, physics_handle_valid=True, body_path='fixed_held_body',
        motion_view_id='fixed_existing_native_view')
    adapter._object_name_from_spec = lambda spec: spec['object']
    actual_spec = dict(cfg)
    phase_key = ('base_6_assembly_clearance', 0)
    identities, relatives, results = [], [], []
    for frame in frames:
        task.frame, task.step = frame['current_pose'], frame['step']
        _, stamp, _, _, relative, state = adapter._assisted_object_servo_context(
            phase_key=phase_key, task=task, robot_name='franka_right', spec=actual_spec,
            tracked_robots={}, tracked_objects=tracked)
        identities.append(state['identity'])
        relatives.append(relative)
        results.append(dict(step=task.step, replay_stamp=stamp,
            keys_after_context=sorted(state), admission_count_after_context=state.get('admission', {}).get('count'),
            previous_object_command_survived='object_command' in state,
            converted_position=relative[0].tolist(), converted_orientation=relative[1].tolist()))
        # A warm-up count of 11 is injected AFTER the first exact production
        # context call; it is not a claim that A13 reached insertion count 11.
        state['admission'] = {'count': 11}
        state['object_command'] = copy.deepcopy(native)
    leads = [dict(step=x['step'], lead_rad=quat_angle(x['current_object_pose']['orientation'],
        x['command_object_pose']['orientation'])) for x in data['absolute_command_observations']]
    assert len(leads) == 513
    defect = identities[0] != identities[1] and results[1]['keys_after_context'] == ['identity']
    assert defect and all(abs(x['lead_rad']-.004) < 1e-9 for x in leads)
    robust_lines = tool_path.read_text().splitlines()
    override_line = next(i+1 for i, s in enumerate(robust_lines)
        if 'def _assisted_object_servo_context(self, **kw)' in s)
    phase_state_line = next(i+1 for i, s in enumerate(robust_lines) if "state = {}; stamp =" in s)
    report = dict(schema='beam_identity_native_replay_v1', defect_reproduced=defect,
        evidence_kind='unchanged_production_method_AST_with_existing_native_run_observations',
        input_path=str(args.inputs.resolve()), input_sha256=sha(inputs_bytes),
        replay_script_sha256=sha(Path(__file__).read_bytes()), source_sha256=bindings,
        critical_tool_sha256=data['critical_tool_sha256'], executed_methods={name: dict(
            source=TASK if name == METHODS[2] else ADAPTER, line=nodes[name].lineno,
            ast_sha256=sha(ast.dump(nodes[name], include_attributes=False).encode())) for name in METHODS},
        native_memory_state_was_read=False, runtime_insertion_admission_was_reached=False,
        controls=dict(same_phase=True, same_robot=True, same_object=True, same_attachment=True,
            same_epoch=True, same_body_path=True, same_motion_view=True,
            only_observed_control_frame_changed=True, raw_stamp_index_advances=True),
        recovered_grasp_relation=dict(position=rel_p.tolist(), orientation=rel_q.tolist(),
            provenance=data['captured_relation_provenance']),
        exact_identity_equal=identities[0] == identities[1],
        maximum_derived_relative_position_change_m=float(np.max(abs(relatives[1][0]-relatives[0][0]))),
        maximum_derived_relative_quaternion_change=float(np.max(abs(relatives[1][1]-relatives[0][1]))),
        injected_state_between_frames=dict(admission={'count': 11}, object_command='known nonempty sentinel'),
        state_results=results, command_angle_lead=dict(samples=len(leads),
            configured_step_rad=cfg['cartesian_orientation_step'],
            configured_lookahead_rad=cfg['cartesian_orientation_command_lookahead'],
            minimum_rad=min(x['lead_rad'] for x in leads), maximum_rad=max(x['lead_rad'] for x in leads),
            all_equal_step_within_1e_9=True, observations=leads),
        missed_preflight_cause=dict(source=ROBUST,
            overridden_context_line=override_line, independent_case_state_line=phase_state_line,
            context_override_excerpt=robust_lines[override_line-2:override_line+2],
            facts=['full 5943 scenarios exercised final command/FK/mesh scopes, not this native context lifetime',
                   'OfflineAdapter replaces production _assisted_object_servo_context with a fixed relative pose and supplied publication state',
                   'each anchor/error case creates fresh state and warms descent_admission directly on synthetic constant poses',
                   'therefore no real cross-tick frame conversion can change its identity or erase its warm-up/cache'],
            conclusion='the offline pass did not cover production state identity persistence across ticks'),
        limitations=['No runtime task-memory state was read. The actual 513 step-sized leads corroborate but alone do not prove reset.',
                     'The captured relation is recovered from same-run full goals, not an exact attachment-memory dump.',
                     'The two control TCP observations come from actual measured-joint motion records; replay getters and native motion detail are narrow test doubles.',
                     'Replay stamps use step+3 and dt=1/240 with a fixed epoch; their times are test inputs, not exact recorded native times. Identity uses epoch, not time.',
                     'The count=11 sentinel is a controlled prospective insertion counterexample; A13 never reached insertion.',
                     'No new native simulation, production edit, stop action, or safety-tolerance change was performed.'])
    save(args.out, report)
    old = (root/ADAPTER).read_text()
    before = """        identity = (attachment.get('joint_path'), attachment.get('attach_step'),
                    stamp['epoch'], native.get('body_path'), native.get('motion_view_id'),
                    tuple(np.asarray(relative[0], float)), tuple(np.asarray(relative[1], float)))"""
    after = """        # Grasp identity is authoritative attachment/view lifecycle data.
        # Derived control-frame floats vary at roundoff level across ticks.
        identity = (attachment.get('joint_path'), attachment.get('attach_step'),
                    stamp['epoch'], native.get('body_path'), native.get('motion_view_id'))"""
    assert old.count(before) == 1
    patch = ''.join(difflib.unified_diff(old.splitlines(True), old.replace(before, after).splitlines(True),
        fromfile='a/'+ADAPTER, tofile='b/'+ADAPTER))
    args.patch_out.parent.mkdir(parents=True, exist_ok=True)
    args.patch_out.write_text(patch)
    design = dict(schema='beam_C02_identity_regression_design_v1', status='proposed_not_applied',
        candidate_patch=str(args.patch_out.resolve()), candidate_patch_sha256=sha(patch.encode()),
        source_before_sha256=bindings[ADAPTER], production_source_was_modified=False,
        intended_change='remove only derived relative float tuples from stable attachment/epoch/body/view identity',
        preserve=['live fixed_joint/robot validation', 'phase-key isolation', 'attach_step/joint_path reset',
                  'native epoch/body/view reset', 'strict actual .15mm/1mrad admission',
                  'final explicit-q FK guard and measured model/native budgets'],
        regressions=[
            dict(name='actual_frame_roundoff_preserves_state', input='actual6504/6505 + exact production three methods',
                 before='count11 and final-q object_command are erased', after='both persist despite 1e-16 relative roundoff'),
            dict(name='real_context_warms_gate_over_12_ticks', input='production context + descent_admission + changing measured control frame; explicitly synthetic aligned native body',
                 after='count1..12, ready only at tick12; no context override'),
            dict(name='same_tick_reentry_does_not_warm', input='servo/context/guard/completion reentries on identical native stamp',
                 after='one native count only; cached final-q object command survives all context calls'),
            dict(name='authoritative_identity_changes_reset', input='change attach_step, joint_path, epoch, body_path, motion_view_id one at a time',
                 after='clear previous command and admission immediately'),
            dict(name='new_phase_is_cold', input='same native body/grasp, new phase key',
                 after='no prior-phase admission or command inherited'),
            dict(name='native_invalid_and_wrong_robot_fail', input='invalid clock/view/body flags or attachment assigned to other robot',
                 after='fail closed before publishing'),
            dict(name='lateral_loss_and_recovery', input='warm12 then actual lateral>.15mm or orientation>1mrad',
                 after='ready false immediately, no unauthorized descent; reacquire twelve real ticks'),
            dict(name='published_command_cache_lifecycle', input='real final published verifier across two changing frames',
                 after='only final-q anchored prediction survives and creates bounded lead, never rejected IK or raw TCP accumulation')],
        acceptance='Run lifecycle regressions and existing controller suite after root authorizes patch; then rerun affected production/preflight checks before C02 native. Do not infer full-path native success from these tests.')
    save(args.regression_design_out, design)
    print(json.dumps(dict(defect_reproduced=defect, report=str(args.out.resolve()),
        report_sha256=sha(args.out.read_bytes()), inputs_sha256=sha(inputs_bytes),
        patch_sha256=sha(args.patch_out.read_bytes()), production_modified=False)))


if __name__ == '__main__':
    main()
