"""Read-only four-part path with a near-pinched, unattached right support hand.

This replaces no historical evidence. It models nominal Cartesian waypoints,
including complete gripper linkage and the final right opening/retreat/park.
Dynamic tracking and the robust insertion corridor are separate launch gates.
"""
from pathlib import Path
import json
import hashlib
import os
import sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from sweep_model import SceneModel, pose_matrix
from visual_mesh_check import VisualMeshChecker
from toolkits.factory_dual_franka_assembly.beam_assisted_visual_grasp import VisiblePinchGeometry

P = Path('/home/user/beam_integration_20261010_upstream6f57dae/beam_assisted_20261015')
label = os.environ.get('BEAM_PREFLIGHT_LABEL', 'C01')
config_path = P / (label + '_actual_configuration_preflight_layout505_config.json')
candidate_path = P / (label + '_right_support_candidate.json')
geometry_path = P / 'runs/A12_beam_full/scene_geometry.json'
config = json.loads(config_path.read_text())
if 'task_configs' in config:config = config['task_configs'][0]
candidate = json.loads(candidate_path.read_text())
m = SceneModel(geometry_path)
v = VisualMeshChecker(m, include_self=True)
pinch = VisiblePinchGeometry(m.data)
q = {name: np.asarray(value, float) for name, value in candidate['support_q'].items()}
targets = config['target_poses']
object_body = {c.entity: c.body for c in m.checker.colliders
               if c.entity.startswith('fabrica_beam_')}
initial_parts = {name: np.asarray(candidate['part_poses'][body], float)
                 for name, body in object_body.items()}
placed = {'fabrica_beam_6': np.asarray(candidate['support_object_pose'], float)}
final_parts = {'fabrica_beam_' + pid: pose_matrix(targets['part_' + pid + '_assembled']['position'],
               targets['part_' + pid + '_assembled']['orientation']) for pid in ('6','3','2','1','0')}
robot = 'franka_left'
held = None
relative = None
support_active = True
rows = []
segments = []
grasp_plans = []
support_failures = []
support_min = float('inf')
support_max = 0.
support_body = object_body['fabrica_beam_6']
right_hand = m.data['robots']['franka_right']['root_path'] + '/Gripper/Robotiq_2F_85/base_link'
failure = None

def hand_path(name):
    return m.data['robots'][name]['root_path'] + '/Gripper/Robotiq_2F_85/base_link'

def poses():
    result = m.poses(q)
    for name, body in object_body.items():result[body] = placed.get(name, initial_parts[name])
    if held:result[object_body[held]] = result[hand_path(robot)] @ relative
    return result

def record(phase):
    global support_min, support_max
    p = poses()
    intended = (robot, held) if held else None
    v.check(p, phase, len(rows), intended)
    support_gaps = pinch.gaps('franka_right', 'fabrica_beam_6', p)
    if support_active:
        support_min = min(support_min, *(g['gap'] for g in support_gaps))
        support_max = max(support_max, *(g['gap'] for g in support_gaps))
        if any(g['intersect'] or not .000005 <= g['gap'] <= .00025 for g in support_gaps):
            support_failures.append({'phase': phase, 'step': len(rows), 'gaps': support_gaps})
    rows.append({'phase': phase, 'q': {n: x.tolist() for n, x in q.items()},
                 'part_poses': {name: p[body].tolist() for name, body in object_body.items()},
                 'intended_grasp': list(intended) if intended else None,
                 'right_visual_support_active': support_active,
                 'right_visual_support_gaps': support_gaps})

def solve(H, reference):
    original = q[robot][:6].copy()
    def residual(value):
        q[robot][:6] = value
        T = m.poses(q)[hand_path(robot)]
        return np.r_[(T[:3,3] - H[:3,3]) * 5,
                     Rotation.from_matrix(H[:3,:3].T @ T[:3,:3]).as_rotvec()]
    fitted = least_squares(residual, reference, max_nfev=100,
                           xtol=1e-10, ftol=1e-10, gtol=1e-10)
    error = residual(fitted.x)
    q[robot][:6] = original
    if np.linalg.norm(error[:3]) / 5 > .0001 or np.linalg.norm(error[3:]) > .001:
        raise RuntimeError('nominal IK endpoint error ' + str(error))
    result = reference + (fitted.x - reference + np.pi) % (2*np.pi) - np.pi
    if np.max(abs(result-reference)) > .12:
        raise RuntimeError('nominal IK branch step > .12 rad')
    return result

def move(H, phase):
    start = m.poses(q)[hand_path(robot)].copy()
    distance = np.linalg.norm(H[:3,3]-start[:3,3])
    angle = Rotation.from_matrix(start[:3,:3].T @ H[:3,:3]).magnitude()
    count = max(2, int(np.ceil(max(distance/.002, angle/.01))) + 1)
    slerp = Slerp([0,1], Rotation.from_matrix([start[:3,:3], H[:3,:3]]))
    first = len(rows)
    for f in np.linspace(0,1,count):
        T = np.eye(4)
        T[:3,3] = (1-f)*start[:3,3]+f*H[:3,3]
        T[:3,:3] = slerp(f).as_matrix()
        q[robot][:6] = solve(T, q[robot][:6])
        record(phase)
    segments.append({'phase': phase, 'samples': len(rows)-first,
                     'max_cartesian_sampling_m': .002, 'max_orientation_sampling_rad': .01})

def fingers(driver, phase):
    names = m.data['robots'][robot]['joint_names']
    signs = {'finger_joint':1, 'right_outer_knuckle_joint':1,
             'left_inner_finger_joint':-1, 'right_inner_finger_joint':1,
             'left_inner_finger_knuckle_joint':-1, 'right_inner_finger_knuckle_joint':-1}
    start = q[robot].copy()
    initial_driver = start[names.index('finger_joint')]
    count = max(2, int(np.ceil(abs(driver-initial_driver)/.008))+1)
    for f in np.linspace(0,1,count):
        for name, sign in signs.items():
            idx = names.index(name)
            q[robot][idx] = start[idx] + sign*f*(driver-initial_driver)
        record(phase)

def close_grasp(obj, phase):
    global held, relative
    B = poses()[object_body[obj]].copy()
    shifts = []
    for attempt in range(13):
        goal, gaps = pinch.goal(robot, obj, q[robot], B)
        if max(g['gap'] for g in gaps) <= .00025:break
        if attempt == 12:raise RuntimeError('nominal visible pinch centering exhausted: ' + obj)
        shift = pinch.center_shift(robot, obj, q[robot], B, goal)
        H = m.poses(q)[hand_path(robot)].copy()
        H[:3,3] += shift
        move(H, phase + ':center')
        shifts.append(shift.tolist())
    fingers(goal, phase)
    actual = pinch.gaps(robot, obj, poses())
    if not all(not g['intersect'] and .000005 <= g['gap'] <= .00025 for g in actual):
        raise RuntimeError('nominal pinch is not qualified: ' + repr(actual))
    held = obj
    relative = np.linalg.inv(m.poses(q)[hand_path(robot)]) @ B
    record(phase + ':capture')
    grasp_plans.append({'object': obj, 'driver_q': goal, 'gaps': actual,
                        'centering_shifts': shifts, 'relative_pose': relative.tolist()})

def waypoint(phase):
    s = phase.get('local_skill', {})
    H = m.poses(q)[hand_path(robot)].copy()
    if s.get('relative_to_current_tcp'):
        offset = np.asarray(s.get('offset', [0,0,0]), float)
        if s.get('offset_frame') in ('tcp','eef','target','gripper'):offset = H[:3,:3] @ offset
        H[:3,3] += offset
    elif s.get('grasp_relative_position') is not None:
        B = poses()[object_body[s['object']]]
        H = B @ np.linalg.inv(pose_matrix(s['grasp_relative_position'], s['grasp_relative_orientation']))
        H[:3,3] += B[:3,:3] @ np.asarray(s.get('grasp_object_frame_offset', [0,0,0]))
        offset = np.asarray(s.get('offset', [0,0,0]), float)
        if s.get('offset_frame') in ('object','local','part'):offset = B[:3,:3] @ offset
        elif s.get('offset_frame') in ('target','eef','tcp','gripper'):offset = H[:3,:3] @ offset
        H[:3,3] += offset
    elif s.get('target_object_target'):
        t = targets[s['target_object_target']]
        goal = pose_matrix(t['position'], t['orientation'])
        if s.get('target_object_offset'):
            if s.get('target_object_offset_frame') != 'world':raise RuntimeError('unsupported target offset frame')
            goal[:3,3] += np.asarray(s['target_object_offset'], float)
        if relative is None:raise RuntimeError('held object target without captured relative pose')
        H = goal @ np.linalg.inv(relative)
    else:raise RuntimeError('unmodeled nominal skill ' + phase['name'])
    try:move(H, phase['name'])
    except RuntimeError as exc:
        actual = m.poses(q)[hand_path(robot)]
        dp = float(np.linalg.norm(actual[:3,3]-H[:3,3]))
        dr = float(Rotation.from_matrix(H[:3,:3] @ actual[:3,:3].T).magnitude())
        if phase['name'].endswith('_park') and dp < float(s.get('position_tolerance', .018)) and dr < float(s.get('orientation_tolerance', .12)):
            segments.append({'phase': phase['name'], 'exact_endpoint_error': str(exc),
                'actual_position_error_m': dp, 'actual_orientation_error_rad': dr,
                'existing_runtime_stage_gate_reached': True})
        else:raise

try:
    record('base_6_visual_support:captured_candidate')
    for pid in ('3','2','1','0'):
        obj = 'fabrica_beam_' + pid
        for phase in [p for p in config['phase_specs'] if '_part_' + pid + '_' in p['name']]:
            name, s = phase['name'], phase.get('local_skill', {})
            if name.endswith('_move_above') and s.get('prealign_joint_positions'):
                start, goal = q[robot][:6].copy(), np.asarray(s['prealign_joint_positions'])
                count = max(2, int(np.ceil(np.max(abs(goal-start))/.01))+1)
                for f in np.linspace(0,1,count):
                    q[robot][:6] = (1-f)*start+f*goal
                    record(name + ':prealign')
            if name.endswith('_preshape'):
                fingers(.8*(1-float(s.get('gripper_openness',1.))), name)
                continue
            if name.endswith('_close_and_attach'):
                close_grasp(obj, name)
                continue
            if name.endswith('_release_and_lock'):
                start = poses()[object_body[obj]].copy()
                placed[obj] = start
                held, relative = None, None
                fingers(0., name)
                final = final_parts[obj]
                distance = np.linalg.norm(final[:3,3]-start[:3,3])
                angle = Rotation.from_matrix(start[:3,:3].T @ final[:3,:3]).magnitude()
                settle_orientation = Slerp([0,1],Rotation.from_matrix([start[:3,:3],final[:3,:3]]))
                for f in np.linspace(0,1,max(2,int(np.ceil(max(distance/.0001,angle/.0002)))+1)):
                    B = start.copy()
                    B[:3,3] = (1-f)*start[:3,3]+f*final[:3,3]
                    B[:3,:3] = settle_orientation(f).as_matrix()
                    placed[obj] = B
                    record(name + ':world_settle')
                placed[obj] = final
                continue
            waypoint(phase)
    robot = 'franka_right'
    for name in ('base_6_visual_support_open','base_6_retreat','base_6_park'):
        phase = next(p for p in config['phase_specs'] if p['name'] == name)
        if name.endswith('_open'):
            support_active = False
            fingers(0., name)
        else:waypoint(phase)
    record('beam_assisted_final_verification')
except Exception as exc:
    failure = {'phase': locals().get('name'), 'error': type(exc).__name__ + ': ' + str(exc), 'sample': len(rows)}

right_rows = []
for r in candidate['path']:
    right_rows.append({**r, 'part_poses': {key.rsplit('/',1)[-1]: value for key,value in r['part_poses'].items()}})
right_path = P / (label + '_right_support_path.jsonl')
with right_path.open('w') as stream:
    for row in right_rows:stream.write(json.dumps(row)+'\n')
path = P / (label + '_nominal_bimanual_path.jsonl')
with path.open('w') as stream:
    for row in rows:stream.write(json.dumps(row)+'\n')
report = v.report()
bad = [pair for pair in report['pairs'] if pair['max_depth'] > 1e-6]
bad_grips = [r for r in report['intended_grasp_mesh_separations'] if r['signed_separation'] < -1e-6]
report.update(passed=failure is None and not bad and not bad_grips and not support_failures,
    failure=failure, segments=segments, grasp_plans=grasp_plans,
    failures_above_precision=bad, grasp_failures_above_precision=bad_grips,
    right_support_failures=support_failures, right_support_min_gap_m=support_min,
    right_support_max_gap_m=support_max, right_support_samples=len(right_rows),
    source_config=str(config_path), source_geometry=str(geometry_path),
    source_candidate=str(candidate_path), path=str(path), right_support_path=str(right_path),
    input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (config_path, candidate_path, geometry_path, Path(__file__))},
    completed_left_parts=list(placed), final_right_stages=[s['phase'] for s in segments if s['phase'] in ('base_6_retreat','base_6_park')],
    limitations=['nominal target interpolation, not actual controller dynamics or robust insertion certificate',
                'candidate right support is from prior native release, not a new native execution',
                'unexported warehouse geometry', 'between-sample CCD', 'PhysX cooked/SDF differs from visible CAD'],
    formal_physical_success=False)
out = P / (label + '_nominal_bimanual.json')
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ('passed','samples','failure','right_support_failures','right_support_min_gap_m','right_support_max_gap_m','final_right_stages')}),flush=True)
print(json.dumps({'failures':bad,'grasp_failures':len(bad_grips),'output':str(out),'path':str(path)}),flush=True)
raise SystemExit(0 if report['passed'] else 3)
