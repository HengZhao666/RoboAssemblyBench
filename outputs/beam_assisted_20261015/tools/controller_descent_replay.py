"""Replay recorded observations through the production pure servo helpers.

This is calculation over existing A12 data, not a dynamics prediction.  The
separate mesh-corridor tool checks FK after production joint command limits.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation(q):
    q = np.asarray(q, float)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def quaternion(r):
    # Eigenvector form avoids a near-pi branch singularity at the Beam wrist.
    r = np.asarray(r, float)
    k = np.array([[r[0, 0]-r[1, 1]-r[2, 2], r[1, 0]+r[0, 1], r[2, 0]+r[0, 2], r[2, 1]-r[1, 2]],
                  [r[1, 0]+r[0, 1], r[1, 1]-r[0, 0]-r[2, 2], r[2, 1]+r[1, 2], r[0, 2]-r[2, 0]],
                  [r[2, 0]+r[0, 2], r[2, 1]+r[1, 2], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
                  [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]])/3
    _, v = np.linalg.eigh(k)
    q = v[:, -1][[3, 0, 1, 2]]
    return q if q[0] >= 0 else -q


def matrix(pose):
    t = np.eye(4)
    t[:3, :3] = rotation(pose['orientation'])
    t[:3, 3] = pose['position']
    return t


def pose(t):
    return {'position': np.asarray(t)[:3, 3], 'orientation': quaternion(np.asarray(t)[:3, :3])}


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--inputs', type=Path, required=True)
    ap.add_argument('--repository', type=Path, required=True)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--config', type=Path)
    ap.add_argument('--lateral', type=float, default=.00015)
    ap.add_argument('--orientation', type=float, default=.001)
    args = ap.parse_args()
    sys.path.insert(0, str(args.repository))
    module = importlib.import_module('toolkits.factory_dual_franka_assembly.beam_assisted_servo')
    data = json.loads(args.inputs.read_text())
    phase_specs = {p['name']: p['local_skill'] for p in data['phase_specs'] if 'local_skill' in p}
    def row_target(m):
        return data['targets'][phase_specs[m['phase']]['target_object_target']]
    # Recover the exact relation used by the old controller to produce its
    # *full* TCP target.  Do not guess a wrist/base pi-rotation from a field name.
    # The following whole-window invariant checks that this is one relation.
    grasp = np.linalg.inv(matrix(data['motions'][0]['target_pose'])) @ matrix(row_target(data['motions'][0]))
    grasp_pose = pose(grasp)
    grasp_target_invariant = max(float(np.max(np.abs(
        matrix(m['target_pose']) @ grasp-matrix(row_target(m))))) for m in data['motions'])
    if grasp_target_invariant > 1e-9:
        raise ValueError('A12 full targets do not identify one fixed grasp transform')
    contract = dict(lateral_tolerance_m=args.lateral, orientation_tolerance_rad=args.orientation,
                    stable_native_ticks=12, max_linear_speed_m_s=.02,
                    max_angular_speed_rad_s=.2, geometry_certificate='pending-source-bound-CAD-corridor',
                    published_axial_slack_m=.00002, published_lateral_slack_m=.00002)
    compiled = json.loads(args.config.read_text()) if args.config else None
    if compiled and 'task_configs' in compiled:
        compiled = compiled['task_configs'][0]
    compiled_phases = {p['name']: p['local_skill'] for p in compiled['phase_specs'] if 'local_skill' in p} if compiled else {}
    rows, failures = [], []
    body_suffix = '/objects/fabrica_beam_3'
    for m in data['motions']:
        step = m['step']
        spec, target = phase_specs[m['phase']], row_target(m)
        if compiled:
            spec = compiled_phases[m['phase']]
            target = dict(compiled['target_poses'][spec['target_object_target']])
            if spec.get('target_object_offset'):
                if spec.get('target_object_offset_frame') != 'world':
                    raise ValueError('unmodeled compiled target offset')
                target['position'] = np.asarray(target['position'])+spec['target_object_offset']
            contract = spec['beam_assisted_descent_admission']
        e, n = data['entities'][str(step)], data['native'][str(step)]
        body_path = next(k for k in e['body_poses'] if k.endswith(body_suffix))
        current = pose(np.asarray(e['body_poses'][body_path]))
        body = next(b for b in n['bodies'] if b['name'] == 'fabrica_beam_3')
        linear = float(np.linalg.norm(body['linear_velocities'][0]))
        angular = float(np.linalg.norm(body['angular_velocities'][0]))
        motion = dict(motion_valid=body['valid'], physics_handle_valid=body['physics_handle_valid'],
                      body_path=body_path, motion_view_id=body['view_id'],
                      linear_speed=linear, angular_speed=angular)
        # Real samples are stride8; they never count as twelve consecutive ticks.
        stamp = data.get('native_stamps', {}).get(str(step))
        if stamp is None:
            stamp = {'valid': False}
        ready, admission = module.descent_admission({}, stamp, current, target, [0, 0, -1], contract, motion)
        measured_fk_object = module.object_pose_from_tcp(m['current_pose'], grasp_pose['position'], grasp_pose['orientation'])
        baseline_position, baseline_orientation = module.validate_model_native_residual(
            current, measured_fk_object, .0001, .0015)
        new_tcp, new_object = module.absolute_object_servo(
            current, target, grasp_pose['position'], grasp_pose['orientation'],
            float(spec.get('cartesian_position_step', .001)),
            float(spec.get('cartesian_orientation_step', .01)),
            position_lookahead=float(spec.get('target_object_servo_position_command_lookahead', .003)),
            convergence_axis=[0, 0, -1], allow_axial_motion=ready,
            measured_fk_object_pose=measured_fk_object)
        old_model_object = module.object_pose_from_tcp(m['command_target_pose'], grasp_pose['position'], grasp_pose['orientation'])
        old_object = module.anchored_object_prediction(current, measured_fk_object, old_model_object)
        new_model_object = module.object_pose_from_tcp(new_tcp, grasp_pose['position'], grasp_pose['orientation'])
        new_predicted_native = module.anchored_object_prediction(current, measured_fk_object, new_model_object)
        if np.max(abs(matrix(new_predicted_native)-matrix(new_object))) > 1e-9:
            raise ValueError('native-command/model inverse mapping is not coherent')
        old_safe, old_detail = module.published_object_command_safe(current, old_object, target, [0, 0, -1], contract, ready)
        new_safe, new_detail = module.published_object_command_safe(current, new_object, target, [0, 0, -1], contract, ready)
        current_tcp_grasp = np.linalg.inv(matrix(m['current_pose'])) @ matrix(current)
        row = dict(step=step, phase=m['phase'], current_object_pose=current, target_object_pose=target,
                   old_recorded_command_object_pose=old_object, new_calculated_object_pose=new_object,
                   measured_fk_object_pose=measured_fk_object,
                   native_vs_measured_fk_position_error_m=baseline_position,
                   native_vs_measured_fk_orientation_error_rad=baseline_orientation,
                   new_fk_native_inverse_mapping_max_matrix_error=float(np.max(abs(matrix(new_predicted_native)-matrix(new_object)))),
                   old_recorded_command_safe=old_safe, new_calculated_command_safe=new_safe,
                   new_descent_admitted=ready, admission=admission,
                   old_published_pose_check=old_detail, new_pose_check=new_detail,
                   captured_vs_measured_grasp_position_error_m=float(np.linalg.norm(current_tcp_grasp[:3, 3]-grasp[:3, 3])))
        rows.append(row)
        if ready or not new_safe or new_detail['predicted_axial_motion_m'] > 1e-10:
            failures.append({'step': step, 'reason': 'new_helper_admits_unaligned_descent_or_invalid_command'})
    old_reverse = [r['step'] for r in rows if r['old_published_pose_check']['predicted_lateral_error_m'] >
                   r['old_published_pose_check']['current_lateral_error_m']+.000001]
    old_unaligned_descent = [r['step'] for r in rows if r['admission']['lateral_error_m'] > args.lateral
                             and r['old_published_pose_check']['predicted_axial_motion_m'] > .00002]
    if not old_unaligned_descent:
        failures.append({'reason': 'missing_recorded_old_unaligned_descent_counterexample'})
    if not any(not r['old_recorded_command_safe'] for r in rows):
        failures.append({'reason': 'old_unsafe_descent_counterexample_not_rejected'})
    source_files = ['toolkits/factory_dual_franka_assembly/beam_assisted_servo.py',
                    'toolkits/factory_dual_franka_assembly/beam_assisted_state.py',
                    'toolkits/factory_dual_franka_assembly/planner_primitives.py',
                    'toolkits/factory_dual_franka_assembly/plumbers_block_ur5e_skills.py']
    report = dict(kind='production_helper_A12_replay', passed=not failures, failures=failures,
                  tool_sha256=sha(__file__), input_sha256=sha(args.inputs),
                  actual_config_sha256=sha(args.config) if args.config else None,
                  contract_origin='actual compiled phase' if compiled else 'explicit standalone candidate',
                  source_sha256={p: sha(args.repository/p) for p in source_files},
                  captured_grasp_in_production_tcp=grasp_pose, contract=contract,
                  captured_grasp_recovery='inverse(recorded full target TCP) @ original object target; invariant across all eleven commands',
                  captured_grasp_full_target_max_matrix_error=grasp_target_invariant,
                  original_run_sha256=data['source_sha256'], old_reverse_lateral_steps=old_reverse,
                  old_unaligned_descent_steps=old_unaligned_descent,
                  samples=rows, actual_native_observations=len(rows), native_physics_steps_executed=0,
                  coverage=['production absolute-object helper', 'production measured descent admission',
                            'production published-object pose gate', 'A12 recorded final command TCP'],
                  gaps=['This calculation has no native dynamics or new trajectory.',
                        'A previous interpretation used TCP Y sign as object motion sign. This replay instead evaluates the held object through the recovered fixed transform; a TCP lever-arm correction is not a reverse-object-motion proof.',
                        'New TCP helper output is not a published joint command. Separate production joint-limit/FK mesh replay is required.',
                        'Stride8 observations cannot establish a consecutive twelve-native-tick gate.',
                        'The candidate contract is not a geometric certificate until the corridor report passes.'])
    text = json.dumps(report, indent=2, default=serial)+'\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end='')
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
