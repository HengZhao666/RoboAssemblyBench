"""Sample an admitted object-error corridor and final limited-joint commands.

No simulator is started. Geometry-only mode is useful before deploying a new
controller; --path adds the production helper, production joint limits, final
publication verifier and full robot/gripper/payload visible-mesh sweep.
"""
from __future__ import annotations
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def as_pose(t):
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return {'position': t[:3, 3].copy(), 'orientation': q[[3, 0, 1, 2]]}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--geometry', type=Path, required=True)
    ap.add_argument('--config', type=Path, required=True)
    ap.add_argument('--repository', type=Path, required=True)
    ap.add_argument('--tools', type=Path, required=True)
    ap.add_argument('--path', type=Path)
    ap.add_argument('--support', type=Path)
    ap.add_argument('--support-path', type=Path)
    ap.add_argument('--reuse-geometric-report', type=Path)
    ap.add_argument('--reuse-config', type=Path)
    ap.add_argument('--free-hover-only', action='store_true')
    ap.add_argument('--reuse-successful-insertion-report', type=Path)
    ap.add_argument('--scope-only', action='store_true')
    ap.add_argument('--lateral', type=float, default=.00015)
    ap.add_argument('--orientation', type=float, default=.001)
    ap.add_argument('--minimum-interface-margin', type=float, default=.00005)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    sys.path[:0] = [str(args.repository), str(args.tools),
                   '/home/user/beam_integration_20261010_upstream6f57dae/runtime_tools/geometry_deps']
    from sweep_model import SceneModel, pose_matrix
    import fcl
    config = json.loads(args.config.read_text())
    if 'task_configs' in config:
        config = config['task_configs'][0]
    targets = config['target_poses']
    phases = {p['name']: p for p in config['phase_specs']}
    def geometry_projection(c):
        return {'assembled_targets': {f'part_{i}_assembled': c['target_poses'][f'part_{i}_assembled'] for i in (6, 3, 2, 1, 0)},
                'robots': c['robots'], 'objects': c['objects'],
                'scene_and_roots': {k: c.get(k) for k in ('scene_root_path', 'robots_root_path', 'objects_root_path',
                    'scene_asset_path', 'scene_asset_fallback_path', 'scene_scale', 'scene_position',
                    'scene_orientation', 'workspace_offset')}}
    def signature(v):
        return hashlib.sha256(json.dumps(v, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
    projection = geometry_projection(config)
    projection_sha = signature(projection)
    path_rows = [json.loads(line) for line in args.path.open()] if args.path else []
    selected = []; last = {}; phase_scopes = {}
    for index, r in enumerate(path_rows):
        intended = r.get('intended_grasp')
        if not intended or ('_insert_' not in r['phase'] and not r['phase'].endswith('_hover')):
            continue
        if args.free_hover_only and not r['phase'].endswith('_transport_hover'):
            continue
        name = intended[1]; z = float(np.asarray(r['part_poses'][name])[2, 3]); key = r['phase']
        spec = phases[key]['local_skill']; contract = spec.get('beam_assisted_descent_admission')
        free = contract is None and key.endswith('_transport_hover') and bool(spec.get('beam_assisted_absolute_object_servo'))
        if contract is None and not free:
            raise ValueError('no declared production control scope for '+key)
        scope = phase_scopes.setdefault(key, dict(kind='free_hover' if free else 'admitted_insertion',
            selected_anchors=0, object_position_probe_radius_m=float(spec.get('target_object_position_tolerance', .008)) if free else args.lateral,
            object_orientation_probe_radius_rad=float(spec.get('target_object_orientation_tolerance', .15)) if free else args.orientation,
            contract_is_not_injected=free, minimum_observed_height_above_final_m=float('inf'), maximum_observed_height_above_final_m=-float('inf')))
        final_z = config['target_poses'][f"part_{name.rsplit('_',1)[-1]}_assembled"]['position'][2]
        h = z-final_z
        scope['minimum_observed_height_above_final_m'] = min(h, scope['minimum_observed_height_above_final_m'])
        scope['maximum_observed_height_above_final_m'] = max(h, scope['maximum_observed_height_above_final_m'])
        is_last = index+1 == len(path_rows) or path_rows[index+1]['phase'] != key
        if key not in last or abs(last[key]-z) >= .002 or is_last:
            selected.append((index, r)); last[key] = z; scope['selected_anchors'] += 1
    source_paths = [args.tools/'sweep_model.py', args.tools/'visual_mesh_check.py']+[args.repository/'toolkits/factory_dual_franka_assembly'/p for p in ['beam_assisted_servo.py','plumbers_block_ur5e_skills.py','beam_assisted_state.py']]
    live_source_binding = {str(p): sha(p) for p in source_paths}
    insertion_reuse = None
    if args.reuse_successful_insertion_report:
        if not args.free_hover_only:
            raise ValueError('successful insertion reuse is only legal for an explicit complementary hover-only run')
        old = json.loads(args.reuse_successful_insertion_report.read_text()); oldf = old['final_published_full_mesh']
        failures = oldf['solve_failures']
        missing_hover = [f for f in failures if f.get('reason')=='compiled_descent_contract_missing' and f.get('phase','').endswith('_transport_hover')]
        missing_total = [f for f in failures if f.get('reason')=='missing_complete_four_part_anchor_case_coverage']
        keyed = {}
        for s in oldf['scenarios']:
            if '_insert_' not in s['phase']: raise ValueError('old successful branch includes non-insertion scenarios')
            keyed.setdefault((s['path_index'], s['phase']), set()).add(s['case'])
        if (old['source_sha256'] != live_source_binding or old['geometry_sha256'] != sha(args.geometry)
                or old['config_sha256'] != sha(args.config) or oldf['path_sha256'] != sha(args.path)
                or oldf['support_sha256'] != sha(args.support) or oldf['support_acquisition_path_sha256'] != sha(args.support_path)
                or oldf['publication_failures'] or oldf['intersections_above_precision']
                or len(oldf['scenarios']) != 4410 or len(keyed) != 210
                or any(v != set(range(21)) for v in keyed.values())
                or oldf['covered_parts'] != ['0','1','2','3']
                or len(missing_hover) != 73 or len(missing_total) != 1 or len(failures) != 74
                or oldf['support_acquisition_full_self_samples'] != 108
                or oldf['full_visible_mesh_report']['self_checked'] is not True
                or oldf['full_visible_mesh_report']['meshes'] != 54):
            raise ValueError('successful insertion reuse lacks exact original source/input/4410-case/108-acquisition proof')
        insertion_reuse = dict(original_report_sha256=sha(args.reuse_successful_insertion_report),
            original_report_launch_qualified=old['launch_qualified'], original_failures_preserved=failures,
            reused_insertion_anchors=210, reused_insertion_cases=4410, reused_acquisition_full_self_samples=108,
            source_sha256=old['source_sha256'], config_sha256=old['config_sha256'],
            path_sha256=oldf['path_sha256'], geometry_sha256=old['geometry_sha256'],
            no_publication_or_mesh_failure_in_reused_branch=True)
    if args.scope_only:
        result = dict(kind='preflight_control_scope_contract_check', passed=bool(selected),
            free_hover_only=args.free_hover_only, selected_anchors=len(selected), expected_cases=21*len(selected),
            phase_scopes=phase_scopes, successful_insertion_reuse=insertion_reuse,
            source_sha256=live_source_binding, config_sha256=sha(args.config), path_sha256=sha(args.path),
            no_geometry_or_FK_checks_run=True, no_simulator_steps=True)
        text=json.dumps(result,indent=2,default=serial)+'\n'
        if args.output: args.output.write_text(text)
        else: print(text,end='')
        return int(not result['passed'])
    model = SceneModel(args.geometry)
    part_meshes = {m['object']: m for m in model.data['visible_meshes']
                   if str(m['object']).startswith('fabrica_beam_')}
    objects = {}
    for name, m in part_meshes.items():
        points, faces = np.asarray(m['points'], float), np.asarray(m['triangles'], np.int32)
        bvh = fcl.BVHModel()
        bvh.beginModel(len(faces), len(points)); bvh.addSubModel(points, faces); bvh.endModel()
        objects[name] = fcl.CollisionObject(bvh)

    def pair(a, ta, b, tb):
        oa, ob = objects[a], objects[b]
        oa.setTransform(fcl.Transform(ta[:3, :3], ta[:3, 3]))
        ob.setTransform(fcl.Transform(tb[:3, :3], tb[:3, 3]))
        result = fcl.CollisionResult()
        hit = fcl.collide(oa, ob, fcl.CollisionRequest(num_max_contacts=8, enable_contact=True), result)
        if hit:
            return {'intersect': True, 'max_triangle_contact_parameter_m': max(
                (float(c.penetration_depth) for c in result.contacts), default=0.)}
        return {'intersect': False, 'distance_m': float(fcl.distance(
            oa, ob, fcl.DistanceRequest(), fcl.DistanceResult()))}

    final = {f'fabrica_beam_{i}': pose_matrix(targets[f'part_{i}_assembled']['position'],
                                            targets[f'part_{i}_assembled']['orientation'])
             for i in (6, 3, 2, 1, 0)}
    directions = [(0., 0.)]+[(float(np.cos(a)), float(np.sin(a)))
                            for a in np.arange(8)*np.pi/4]
    tilts = [np.zeros(3)]+[np.eye(3)[i]*s*args.orientation for i in range(3) for s in (-1, 1)]
    heights = [.003, .004, .006, .008, .012, .016, .02, .03, .04, .06]
    # All combinations at the declared radial boundary, plus center/axis tilts.
    # This is a finite sample grid, not a mathematical continuous-volume proof.
    admitted_poses = [(np.asarray(d)*args.lateral, w) for d in directions for w in tilts]
    corridor_rows, anchor_rows, unsafe, closest = [], [], [], None
    reused = None
    if args.reuse_geometric_report:
        reused = json.loads(args.reuse_geometric_report.read_text())
        state_path = args.repository/'toolkits/factory_dual_franka_assembly/beam_assisted_state.py'
        state_key = str(state_path)
        prior_projection = reused.get('geometric_input_projection')
        if prior_projection is None:
            # Legacy reports can only acquire a projection from their exact
            # preserved original configuration, never from a new similar cfg.
            if not args.reuse_config or sha(args.reuse_config) != reused.get('config_sha256'):
                raise ValueError('legacy geometry reuse requires exact original --reuse-config SHA')
            previous_config = json.loads(args.reuse_config.read_text())
            if 'task_configs' in previous_config: previous_config = previous_config['task_configs'][0]
            prior_projection = geometry_projection(previous_config)
        if (not reused.get('geometry_passed') or reused.get('geometry_sha256') != sha(args.geometry)
                or signature(prior_projection) != projection_sha
                or (reused.get('geometric_input_projection_sha256') is not None
                    and reused['geometric_input_projection_sha256'] != signature(prior_projection))
                or reused.get('lateral_radius_m') != args.lateral
                or reused.get('orientation_radius_rad') != args.orientation
                or reused.get('minimum_required_interface_margin_m') != args.minimum_interface_margin
                or reused.get('source_sha256', {}).get(state_key) != sha(state_path)):
            raise ValueError('geometric reuse binding differs in geometry/physical-config-projection/corridor/anchor source')
        corridor_rows = reused['object_pair_corridor']
        anchor_rows = reused['installation_anchor_schedule']
        closest = reused['closest_interface']
    installed = ['fabrica_beam_6']
    for pid in (() if reused else (3, 2, 1, 0)):
        name = f'fabrica_beam_{pid}'
        minimum = None; intersections = []; checks = 0
        for h, (xy, w) in itertools.product(heights, admitted_poses):
            t = final[name].copy(); t[:2, 3] += xy; t[2, 3] += h
            t[:3, :3] = Rotation.from_rotvec(w).as_matrix() @ t[:3, :3]
            for other in installed:
                result = pair(name, t, other, final[other]); checks += 1
                row = dict(part=name, other=other, height_above_final_m=h,
                           lateral_offset_m=xy, orientation_error_rotvec_rad=w, **result)
                if result['intersect'] and result['max_triangle_contact_parameter_m'] > .000001:
                    intersections.append(row)
                elif not result['intersect'] and (minimum is None or result['distance_m'] < minimum['distance_m']):
                    minimum = row
        good_margin = minimum is not None and minimum['distance_m'] >= args.minimum_interface_margin
        corridor_rows.append(dict(part=name, pair_checks=checks, closest_separated=minimum,
                                  intersections_above_precision=intersections,
                                  minimum_margin_passed=good_margin))
        if intersections or not good_margin:
            unsafe.append({'part': name, 'reason': 'sampled_admitted_corridor_lacks_declared_interface_margin'})
        if minimum and (closest is None or minimum['distance_m'] < closest['distance_m']):
            closest = minimum
        # _assisted_update_install_alignment moves a world-support anchor only
        # after complete opening. Position takes <=0.1mm steps and quaternion
        # nlerp takes <=0.2mrad per native tick. Reproduce that numeric schedule
        # from every admitted capture boundary; never hold XY error constant
        # while artificially lowering to a zero-clearance final interface.
        anchor_checks = 0; anchor_hits = []; anchor_minimum = None
        for xy, w in admitted_poses:
            t = final[name].copy(); t[:2, 3] += xy; t[2, 3] += .003
            q0 = Rotation.from_matrix(Rotation.from_rotvec(w).as_matrix() @ final[name][:3, :3]).as_quat()
            qt = Rotation.from_matrix(final[name][:3, :3]).as_quat()
            if np.dot(q0, qt) < 0: qt = -qt
            for tick in range(40):
                delta = final[name][:3, 3]-t[:3, 3]; length = float(np.linalg.norm(delta))
                t[:3, 3] += delta*min(1., .0001/max(length, 1e-15))
                angle = 2*np.arccos(np.clip(np.dot(q0, qt), -1., 1.))
                ratio = min(1., .0002/max(float(angle), 1e-15))
                q0 = (1-ratio)*q0+ratio*qt; q0 /= np.linalg.norm(q0)
                t[:3, :3] = Rotation.from_quat(q0).as_matrix()
                for other in installed:
                    result = pair(name, t, other, final[other]); anchor_checks += 1
                    item = dict(part=name, other=other, tick=tick, start_lateral_offset_m=xy,
                                start_orientation_rotvec_rad=w, **result)
                    if result['intersect'] and result['max_triangle_contact_parameter_m'] > .000001:
                        anchor_hits.append(item)
                    elif not result['intersect'] and (anchor_minimum is None or result['distance_m'] < anchor_minimum['distance_m']):
                        anchor_minimum = item
                if length <= .0001 and angle <= .0002:
                    break
        anchor_rows.append(dict(part=name, pair_checks=anchor_checks,
                                intersections_above_precision=anchor_hits, closest_separated=anchor_minimum,
                                expected_mating_surface_contacts_do_not_require_free_space_50um_margin=True))
        if anchor_hits:
            unsafe.append({'part': name, 'reason': 'installation_anchor_schedule_intersects_visible_CAD'})
        installed.append(name)
    # Preserve the exact A12 visible-CAD counterexample using its actual matrices.
    a12_actual = reused.get('exact_A12_counterexample') if reused else None
    trace = args.geometry.parent/'entity_geometry_trace.jsonl'
    if trace.exists() and not reused:
        for line in trace.open():
            r = json.loads(line)
            if r.get('step') == 18272:
                pa = part_meshes['fabrica_beam_3']['body']; pb = part_meshes['fabrica_beam_6']['body']
                a12_actual = pair('fabrica_beam_3', np.asarray(r['body_poses'][pa]),
                                  'fabrica_beam_6', np.asarray(r['body_poses'][pb]))
                break
    full = None
    if args.path:
        from visual_mesh_check import VisualMeshChecker
        from toolkits.factory_dual_franka_assembly.beam_assisted_servo import absolute_object_servo
        from toolkits.factory_dual_franka_assembly.beam_assisted_servo import anchored_object_prediction
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
        from toolkits.factory_dual_franka_assembly.beam_assisted_servo import descent_admission
        support = json.loads(args.support.read_text()) if args.support else None
        checker = VisualMeshChecker(model, include_self=True)
        scenarios = []; rejected = []; solve_failures = []; publish_failures = []
        support_samples = 0
        if args.support:
            if not args.support_path:
                raise ValueError('right support requires its complete acquisition path for full self-topology checks')
            part_bodies = {name: m['body'] for name, m in part_meshes.items()}
            support_phase = phases.get('base_6_visual_support', {}).get('local_skill', {})
            if support_phase.get('robot') != 'franka_right' or support_phase.get('object') != 'fabrica_beam_6':
                raise ValueError('support candidate is not bound to the compiled right/base6 support phase')
            last_support_row = None
            for line in args.support_path.open():
                sr = json.loads(line)
                if support_samples == 0 and not np.allclose(sr['q']['franka_right'], support['start_q']['franka_right'], atol=1e-9, rtol=0):
                    raise ValueError('right support acquisition start differs from the preserved native candidate')
                sp = {**model.poses(sr['q']), **{part_bodies[k]: np.asarray(v) for k, v in sr['part_poses'].items()}}
                if insertion_reuse is None:
                    checker.check(sp, sr['phase'], -1-support_samples, ('franka_right', 'fabrica_beam_6'))
                support_samples += 1
                last_support_row = sr
            if (last_support_row is None or not np.allclose(last_support_row['q']['franka_right'],
                    support['support_q']['franka_right'], atol=1e-9, rtol=0)):
                raise ValueError('right support acquisition endpoint differs from held support q')
            for body, t in support['part_poses'].items():
                short = body.rsplit('/', 1)[-1]
                if not np.allclose(last_support_row['part_poses'][short], t, atol=1e-9, rtol=0):
                    raise ValueError('right support acquisition endpoint part poses differ from candidate')
        part_bodies = {name: m['body'] for name, m in part_meshes.items()}
        for index, r in selected:
            robot, name = r['intended_grasp']; q0 = {k: np.asarray(v, float) for k, v in r['q'].items()}
            if support:
                q0['franka_right'] = np.asarray(support['support_q']['franka_right'], float)
            p0 = {**model.poses(q0), **{part_bodies[k]: np.asarray(v) for k, v in r['part_poses'].items()}}
            spec = dict(phases[r['phase']]['local_skill'])
            contract = spec.get('beam_assisted_descent_admission')
            free_hover = phase_scopes[r['phase']]['kind']=='free_hover'
            tcp_body = model.data['robots'][robot]['root_path']+'/wrist_3_link'
            b0 = p0[part_bodies[name]]; relation = np.linalg.inv(p0[tcp_body]) @ b0
            goal = dict(targets[spec['target_object_target']])
            if spec.get('target_object_offset'):
                if spec.get('target_object_offset_frame') != 'world':
                    raise ValueError('unmodeled target offset frame')
                goal = {**goal, 'position': np.asarray(goal['position'])+spec['target_object_offset']}
            # Start at ideal XY/orientation at each sampled route height. The
            # case grid is the independently measured/allowed error envelope.
            bcenter = pose_matrix(goal['position'], goal['orientation']); bcenter[2, 3] = b0[2, 3]
            relative_pose = as_pose(relation)

            def ik(t, qref):
                qs = {k: v.copy() for k, v in q0.items()}
                def residual(x):
                    qs[robot][:6] = x
                    actual = model.poses(qs)[tcp_body]
                    return np.r_[(actual[:3, 3]-t[:3, 3])*5,
                                 Rotation.from_matrix(t[:3, :3].T@actual[:3, :3]).as_rotvec()]
                s = least_squares(residual, qref, max_nfev=70, xtol=1e-10, ftol=1e-10, gtol=1e-10)
                error = residual(s.x)
                if np.linalg.norm(error[:3])/5 > .00001 or np.linalg.norm(error[3:]) > .0001:
                    raise ValueError('offline IK lacks 10um/0.1mrad residual')
                return qref+(s.x-qref+np.pi)%(2*np.pi)-np.pi

            # Fifteen object-error boundary cases plus six explicit model/native
            # residual probes. The latter keep native actor alignment intact
            # while the FK differs by the declared1.5mrad and measured11.306um.
            xy_radius = phase_scopes[r['phase']]['object_position_probe_radius_m']
            angular_radius = phase_scopes[r['phase']]['object_orientation_probe_radius_rad']
            cases = [(xy, np.zeros(3), np.zeros(3), np.zeros(3))
                     for xy in (np.asarray(d)*xy_radius for d in directions)]
            cases += [(np.zeros(2), np.eye(3)[i]*sign*angular_radius, np.zeros(3), np.zeros(3))
                      for i in range(3) for sign in (-1,1)]
            # 1urad below the hard1.5mrad model budget prevents the numerical
            # IK residual from masquerading as a physical boundary violation.
            cases += [(np.zeros(2), np.zeros(3), axis*.001499, axis*.000011306)
                      for axis in [np.eye(3)[i]*sign for i in range(3) for sign in (-1, 1)]]
            for case_i, (xy, w, model_bias_w, model_bias_p) in enumerate(cases):
                b = bcenter.copy(); b[:2, 3] += xy
                b[:3, :3] = Rotation.from_rotvec(w).as_matrix() @ b[:3, :3]
                model_b = b.copy(); model_b[:3, 3] += model_bias_p
                model_b[:3, :3] = Rotation.from_rotvec(model_bias_w).as_matrix() @ model_b[:3, :3]
                try:
                    qa = ik(model_b@np.linalg.inv(relation), q0[robot][:6])
                except Exception as exc:
                    solve_failures.append({'phase': r['phase'], 'index': index, 'case': case_i, 'reason': str(exc)}); continue
                current = as_pose(b)
                q_measured = {k: v.copy() for k, v in q0.items()}; q_measured[robot][:6] = qa
                actual_fk_object = as_pose(model.poses(q_measured)[tcp_body] @ relation)
                state = {}; stamp = {'valid': True, 'epoch': 0, 'index': 12, 'time': .05, 'dt': 1/240}
                synthetic_motion = dict(motion_valid=True, physics_handle_valid=True,
                    body_path=part_bodies[name], motion_view_id='offline-fixed-geometry', linear_speed=0., angular_speed=0.)
                ready = True if free_hover else False
                if not free_hover:
                    for tick in range(1, 13):
                        s = {**stamp, 'index': tick, 'time': tick/240}
                        ready, _ = descent_admission(state, s, current, goal, [0, 0, -1], contract, synthetic_motion)
                tcp_goal, _ = absolute_object_servo(current, goal, relative_pose['position'],
                    relative_pose['orientation'], float(spec.get('cartesian_position_step', .001)),
                    float(spec.get('cartesian_orientation_step', .004)), convergence_axis=None if free_hover else [0, 0, -1], allow_axial_motion=ready,
                    measured_fk_object_pose=actual_fk_object)
                try:
                    qr = ik(pose_matrix(tcp_goal['position'], tcp_goal['orientation']), qa)
                except Exception as exc:
                    solve_failures.append({'phase': r['phase'], 'index': index, 'case': case_i, 'reason': str(exc)}); continue
                qs = {k: v.copy() for k, v in q0.items()}; qs[robot][:6] = qa

                class OfflineAdapter(Adapter):
                    def _assisted_object_servo_context(self, **kw):
                        return name, stamp, synthetic_motion, current, (relative_pose['position'], relative_pose['orientation']), publication_state
                    def _target_object_pose(self, **kw):
                        return goal
                    def _grasp_command_fk_pose(self, **kw):
                        test_q = {k: v.copy() for k, v in qs.items()}; test_q[robot][:6] = kw['command_q']
                        return as_pose(model.poses(test_q)[tcp_body])
                adapter = OfflineAdapter.__new__(OfflineAdapter); adapter._last_arm_command_q = {robot: qa.copy()}
                limits = Adapter._command_joint_step_limits(spec=spec, joint_count=6)
                qr = Adapter._limited_joint_target(current_q=qa, target_q=qr, max_joint_step=.02 if limits is None else limits)
                task = SimpleNamespace(phase_index=0, phase_entry_step=0, _ur5e_plumbers_last_arm_command_q={robot: qa.copy()})
                qr = adapter._continuous_command_q(task=task, robot_name=robot, command_q=qr, spec=spec)
                qr = Adapter._limit_command_to_measured_state(current_q=qa, command_q=qr, spec=spec)
                publication_state = {'descent_ready': ready, 'stamp': stamp}
                try:
                    qp, detail = adapter._assisted_verify_published_held_command(task=task, robot_name=robot,
                        context={'spec': spec, 'phase_key': (0,), 'phase_index': 0, 'phase_entry_step': 0,
                                 'tracked_robots': {}, 'tracked_objects': {}}, command_q=qr, current_q=qa, previous_q=qa)
                except Exception as exc:
                    publish_failures.append({'phase': r['phase'], 'index': index, 'case': case_i, 'reason': str(exc)}); continue
                samples = max(2, int(np.ceil(np.max(abs(qp-qa))/.004))+1)
                for f in np.linspace(0, 1, samples):
                    sweep_q = {k: v.copy() for k, v in qs.items()}; sweep_q[robot][:6] = qa+f*(qp-qa)
                    poses = {**model.poses(sweep_q), **{part_bodies[k]: np.asarray(v) for k, v in r['part_poses'].items()}}
                    model_object = as_pose(poses[tcp_body] @ relation)
                    native_object = anchored_object_prediction(current, actual_fk_object, model_object)
                    poses[part_bodies[name]] = pose_matrix(native_object['position'], native_object['orientation'])
                    checker.check(poses, r['phase'], len(scenarios), (robot, name))
                scenarios.append(dict(phase=r['phase'], path_index=index, case=case_i, admitted=ready,
                                      control_scope='free_hover_no_admission_contract' if free_hover else 'admitted_insertion',
                                      lateral_offset_m=xy, orientation_error_rotvec_rad=w,
                                      explicit_model_native_orientation_bias_rad=model_bias_w,
                                      explicit_model_native_position_bias_m=model_bias_p,
                                      actual_native_object_pose=current, measured_fk_object_pose=actual_fk_object,
                                      actual_q=qa, requested_ik_q=qr, final_published_q=qp,
                                      final_publication_check=detail, sweep_samples=samples))
                goal_delta = np.asarray(goal['position'])-b[:3,3]
                published_q = {k:v.copy() for k,v in qs.items()}; published_q[robot][:6]=qp
                published_model = as_pose(model.poses(published_q)[tcp_body] @ relation)
                published_native = anchored_object_prediction(current, actual_fk_object, published_model)
                remaining = float(np.linalg.norm(goal_delta))
                progress = float(np.dot(np.asarray(published_native['position'])-b[:3,3],goal_delta/max(remaining,1e-15)))
                scenarios[-1].update(final_published_native_object_pose=published_native,progress_toward_target_m=progress)
                if case_i == 0 and remaining > .00005 and (not ready or progress <= .000001):
                    publish_failures.append({'phase': r['phase'], 'index': index,
                        'reason': 'aligned center case has no admitted forward progress',
                        'remaining_position_m': remaining, 'publication': detail})
            # The dangerous A12/legacy envelope must fail admission. Deliberate
            # synthetic probes exercise classification, never count as native.
            for radius in (.0012, .002):
                if free_hover: continue
                bad = as_pose(bcenter); bad['position'] = bad['position']+np.array([radius, 0., 0.])
                ready, detail = descent_admission({}, {'valid': True, 'epoch': 0, 'index': 1, 'time': 1/240, 'dt': 1/240},
                    bad, goal, [0, 0, -1], contract, synthetic_motion)
                rejected.append({'phase': r['phase'], 'radius_m': radius, 'ready': ready, 'aligned': detail['aligned']})
        report = checker.report()
        collision_rows = [r for r in report['intersections'] if r['depth'] > .000001]
        covered_parts = sorted({r['phase'].split('_part_')[1].split('_')[0] for r in scenarios if '_part_' in r['phase']})
        if covered_parts != ['0', '1', '2', '3'] or not selected or len(scenarios) != len(selected)*21:
            solve_failures.append({'reason': 'missing_complete_four_part_anchor_case_coverage',
                'covered_parts': covered_parts, 'selected_anchors': len(selected),
                'completed_cases': len(scenarios), 'expected_cases': len(selected)*21})
        full = dict(path_sha256=sha(args.path), support_sha256=sha(args.support) if args.support else None,
                    support_acquisition_path_sha256=sha(args.support_path) if args.support_path else None,
                    support_acquisition_full_self_samples=support_samples,
                    support_acquisition_full_self_reused=insertion_reuse is not None,
                    selected_anchors=len(selected), scenarios=scenarios, dangerous_probe_results=rejected,
                    covered_parts=covered_parts,
                    solve_failures=solve_failures, publication_failures=publish_failures,
                    full_visible_mesh_report=report, intersections_above_precision=collision_rows,
                    passed=not (solve_failures or publish_failures or collision_rows or any(r['ready'] or r['aligned'] for r in rejected)),
                    synthetic_pose_stability_is_not_native_gate_evidence=True)
        if not full['passed']:
            unsafe.append({'reason': 'full_final_published_command_sweep_failed'})
    source_files = ['sweep_model.py', 'visual_mesh_check.py']
    source_binding = {str(args.tools/p): sha(args.tools/p) for p in source_files}
    for p in ['beam_assisted_servo.py', 'plumbers_block_ur5e_skills.py', 'beam_assisted_state.py']:
        path = args.repository/'toolkits/factory_dual_franka_assembly'/p
        if path.exists():
            source_binding[str(path)] = sha(path)
    result = dict(kind='sampled_error_corridor_and_limited_joint_mesh_preflight',
                  geometry_passed=not unsafe, launch_qualified=bool(full and full['passed'] and not unsafe
                    and (not args.free_hover_only or insertion_reuse is not None)),
                  failures=unsafe, source_sha256=source_binding, geometry_sha256=sha(args.geometry), config_sha256=sha(args.config),
                  geometric_input_projection=projection, geometric_input_projection_sha256=projection_sha,
                  lateral_radius_m=args.lateral, orientation_radius_rad=args.orientation,
                  minimum_required_interface_margin_m=args.minimum_interface_margin,
                  object_pair_corridor=corridor_rows, closest_interface=closest,
                  installation_anchor_schedule=anchor_rows,
                  exact_A12_counterexample=a12_actual, final_published_full_mesh=full,
                  geometry_reused_from_sha256=sha(args.reuse_geometric_report) if reused else None,
                  geometry_reused_original_config_sha256=reused.get('config_sha256') if reused else None,
                  geometry_reuse_original_config_supplied_sha256=sha(args.reuse_config) if args.reuse_config else None,
                  control_coverage_mode='split_hover_completion_of_source_bound_insertion_report' if args.free_hover_only else 'complete_mixed_control_scope',
                  phase_control_scopes=phase_scopes, successful_insertion_reuse=insertion_reuse,
                  combined_anchor_count=(len(selected)+(insertion_reuse['reused_insertion_anchors'] if insertion_reuse else 0)),
                  combined_case_count=(len(full['scenarios'])+(insertion_reuse['reused_insertion_cases'] if insertion_reuse else 0)) if full else 0,
                  scope={'parts': [3, 2, 1, 0], 'assembly_heights_m': heights,
                         'object_error_cases_per_height': len(admitted_poses),
                         'geometry_meshes_available': len(model.data['visible_meshes']),
                         'no_simulation_steps': True},
                  gaps=['Finite sampled envelope; no continuous clearance or between-sample CCD proof.',
                        'Warehouse geometry absent from exported mesh set; native cooked/SDF differs from visible CAD.',
                        'Full mesh mode uses exact nominal grasp relations; observed grasp drift must be budgeted in the clearance margin.',
                        'Offline numerical IK is not native Lula/drive dynamics; production clamp and final-verifier functions are executed directly.',
                        'Only sampled insertion/hover anchors are perturbed. Whole-route nominal and right-support-path checks remain separate gates.',
                        'Each anchor uses previous published q equal to current q. Cached-target lead is covered by separate production-controller regression tests, not this mesh grid.',
                        'Free-hover8mm/.15rad probes are separate stage-completion-boundary examples, not a proven full trajectory tracking-error envelope. No descent admission is invented for those production branches.',
                        'Passing pair geometry alone never qualifies launch. --path with final-publication full-mesh mode is required.'])
    text = json.dumps(result, indent=2, default=serial)+'\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text, end='')
    return int(bool(unsafe))


if __name__ == '__main__':
    raise SystemExit(main())
