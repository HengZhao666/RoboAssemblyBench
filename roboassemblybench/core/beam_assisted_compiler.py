"""Separate Beam demonstration route with measured-pose constraint assistance.

This route is never a continuous-contact physics benchmark result. Assets and
targets come from the retained physical recipe; assistance cannot teleport a
part, disable its collider, or attach it before the fingers reach it.
"""
from __future__ import annotations

import copy


def _add_visual_base_support(result):
    """Keep the released base visibly supported while the left arm assembles.

    The world support remains the only joint on the base. The existing CAD
    close controller centers the empty right gripper and stops short of both
    visible faces; this phase deliberately has no attachment operation.
    """
    phases = result['phases']
    retreat = next(p for p in phases if p['name'] == 'base_6_retreat')
    park = next(p for p in phases if p['name'] == 'base_6_park')
    phases[:] = [p for p in phases if p is not retreat and p is not park]
    release_index = next(i for i, p in enumerate(phases)
                         if p['name'] == 'base_6_release_and_lock')
    support = {'robot': 'franka_right', 'object': 'fabrica_beam_6',
               'mode': 'non_attached_visible_pinch',
               'minimum_surface_gap_m': .000005,
               'maximum_surface_gap_m': .00025,
               'stable_native_ticks': 12,
               'force_support_claimed': False}
    phases.insert(release_index + 1, {
        'name': 'base_6_visual_support', 'beam_assisted': True,
        'beam_assisted_installed_parts': ['fabrica_beam_6'],
        'beam_assisted_visual_support': copy.deepcopy(support),
        'timeout_steps': 2400,
        'local_skill': {
            'name': 'ur5e_close_gripper', 'robot': 'franka_right',
            'object': 'fabrica_beam_6', 'beam_assisted': True,
            'beam_assisted_visual_pinch': True,
            'requires_held_object': False, 'require_close_pose_gate': False,
            'use_arm_ik_controller': False,
            'limit_command_to_measured_state': True,
            'guard_ik_branch_jump': True,
            'max_command_tracking_error': .12,
            'max_wrist_command_tracking_error': .10,
        },
        'advance': {'type': 'local_skill_complete', 'robot': 'franka_right',
                    'skill': 'ur5e_close_gripper', 'min_steps': 1},
    })
    # Inactive assisted arms retain their measured arm anchor and the last
    # published gripper aperture. No right-arm companion can reopen/reposition
    # the support hand during the four left-arm assembly sequences.
    for phase in phases[release_index + 2:]:
        phase['beam_assisted_visual_support'] = copy.deepcopy(support)
        primary = phase.get('local_skill', {})
        companions = phase.get('local_skills', {})
        if (primary.get('robot') == 'franka_right'
                or 'franka_right' in companions
                or 'franka_right' in phase.get('gripper_commands', {})):
            raise ValueError('Assisted visual base support conflicts with a right-arm command: '
                             + phase['name'])
    installed = [c['object'] for c in result['success']]
    phases.append({
        'name': 'base_6_visual_support_open', 'beam_assisted': True,
        'beam_assisted_installed_parts': list(installed),
        'timeout_steps': 720,
        'local_skill': {
            'name': 'ur5e_preshape_gripper', 'robot': 'franka_right',
            'object': 'fabrica_beam_6', 'gripper_openness': 1.,
            'gripper_position_tolerance': .005, 'stable_steps': 12,
            'require_object_static': False, 'preshape_timeout_steps': 600,
            'beam_assisted': True, 'use_arm_ik_controller': False,
            'limit_command_to_measured_state': True,
            'guard_ik_branch_jump': True,
            'max_command_tracking_error': .12,
            'max_wrist_command_tracking_error': .10,
        },
        'advance': {'type': 'local_skill_complete', 'robot': 'franka_right',
                    'skill': 'ur5e_preshape_gripper', 'min_steps': 1},
    })
    # The upward retreat is clear with all five installed parts. The former
    # long return crosses the newly installed top beam, so finish at this
    # already-swept open-hand clearance rather than invent another return.
    for phase in (retreat,):
        phase['beam_assisted_installed_parts'] = list(installed)
        phases.append(phase)
    result.setdefault('metadata', {})['beam_assisted_visual_support'] = copy.deepcopy(support)


def _append_final_home_return(result):
    """Return the two released arms separately after the audited outward retreat."""
    from .fabrica_canonical import (
        _initial_arm_joint_positions, _robot_platform, _ARM_JOINT_NAMES_BY_PLATFORM,
    )
    robots = {robot['name']: robot for robot in result['robots']}
    if set(robots) != {'franka_left', 'franka_right'}:
        raise ValueError('assisted home return requires the two declared UR5e arms')
    targets = {}
    installed = [criterion['object'] for criterion in result['success']]
    for name in ('franka_left', 'franka_right'):
        robot = robots[name]
        names = list(_ARM_JOINT_NAMES_BY_PLATFORM[_robot_platform(robot)])
        goal = _initial_arm_joint_positions(robot)
        targets[name] = dict(joint_names=names, joint_positions=goal, tolerance_rad=.01)
        phase = {
            'name': 'beam_assisted_return_home_' + name,
            'beam_assisted': True,
            'beam_assisted_empty_return_guard': True,
            'beam_assisted_installed_parts': list(installed),
            'timeout_steps': 3600,
            'local_skill': {
                'name': 'move_arm_to_joint_positions', 'robot': name,
                'beam_assisted': True,
                'joint_positions': list(goal), 'joint_position_tolerance': .01,
                'joint_target_stable_steps': 8,
                'beam_assisted_native_joint_target': True,
                'requires_held_object': False, 'gripper_command': 'open',
                'unwrap_revolute_joints': False,
                'use_arm_ik_controller': False,
                'max_joint_step': .01, 'max_command_tracking_error': .03,
                'max_wrist_command_tracking_error': .03,
                'limit_command_to_measured_state': True,
                'guard_ik_branch_jump': True,
            },
            'gripper_commands': {arm: 'open' for arm in robots},
            'advance': {'type': 'local_skill_complete', 'robot': name,
                        'skill': 'move_arm_to_joint_positions', 'min_steps': 1},
        }
        if name == 'franka_right':
            clearance = copy.deepcopy(phase)
            clearance['name'] = 'beam_assisted_return_clearance_franka_right'
            delta = [0.] * len(names)
            delta[names.index('shoulder_lift_joint')] = -.24
            clearance['local_skill']['beam_assisted_joint_target_delta'] = delta
            result['phases'].append(clearance)
        result['phases'].append(phase)
    return targets


def compile_assisted_beam_recipe(payload):
    from .fabrica_canonical import compile_fabrica_canonical_recipe
    from .beam_physical_compiler import compile_continuous_beam_recipe

    source = copy.deepcopy(payload)
    spec = source['fabrica_canonical']
    if spec.get('assembly') != 'beam':
        raise ValueError('assisted route is supported only for Beam')
    spec.pop('beam_execution_route', None)
    physical = compile_continuous_beam_recipe(copy.deepcopy(source))
    spec.update(continuous_physics=False, coordinated_support_plan=False,
                official_bimanual_hold=False, beam_assisted_audited_grasps=True,
                stabilize_fixture_parts=True,
                disable_collision_during_fixed_transport=False,
                filter_gripper_collisions_on_attach=False,
                physical_attach_surface_gap=.001,
                insertion_relaxed_position_tolerance=.008)
    result = compile_fabrica_canonical_recipe(source)
    # Keep the audited SDF parts, nest, support, full robots and visible housing.
    result['objects'] = copy.deepcopy(physical['objects'])
    # Native wrist/part anchor audits locate the fourth-part error in the
    # dynamic fixed constraint. Increase numerical convergence without changing
    # masses, frozen grasp, motion rates or the original100um/1.5mrad guard.
    for obj in result['objects']:
        if obj['name'] == 'fabrica_beam_1':
            obj['solver_position_iteration_count'] = 64
    result['domain_randomization'] = copy.deepcopy(physical['domain_randomization'])
    target_map = {t['name']: copy.deepcopy(t) for t in result['targets']}
    target_map.update({t['name']: copy.deepcopy(t) for t in physical['targets']})
    # The authored base bottom extends 20.8 um below the old board target.
    # Raise the whole assisted assembly frame 0.1 mm, preserving every mating
    # transform and avoiding a visible-CAD intersection with the board.
    for name, target in target_map.items():
        if ('_insert_' in name or name.endswith(('_assembled', '_assembly_hover',
                '_assembly_clearance', '_preinsert_hover', '_preseat'))):
            target['position'][2] += .0001
    # Loaded insertion ends at the same capture height as the final skill.
    # The original contact route's last waypoint is only 0.82 mm above the
    # final pose; retaining it would descend below capture then rise again.
    # Keep every XY/orientation waypoint and only truncate that last descent.
    for name, target in target_map.items():
        if '_insert_' in name:
            assembled = target_map[name.split('_insert_', 1)[0] + '_assembled']
            target['position'][2] = max(target['position'][2],
                                        assembled['position'][2] + .003)
    # Initial fixture assistance holds the measured 4 mm spawn clearance.
    # Keep clearance waypoints above the subsequent lift, without depending
    # on the friction controller's saved TCP command to raise them at runtime.
    for pid in ('6', '3', '2', '1', '0'):
        fixture = target_map.get(f'part_{pid}_fixture_pickup')
        clearance = target_map.get(f'part_{pid}_pickup_clearance')
        if fixture and clearance:
            clearance['position'][2] = max(clearance['position'][2],
                                            fixture['position'][2] + .089)
            assembly_clearance = target_map.get(f'part_{pid}_assembly_clearance')
            if assembly_clearance:
                assembly_clearance['position'][2] = max(assembly_clearance['position'][2],
                                                        clearance['position'][2])
            if pid == '2':
                # The right arm now stays at the base instead of parking.
                # Rise before the column's cross-table turn; a three-segment
                # full-mesh sweep qualifies 60 mm extra with 8.576 mm minimum
                # column/right-arm clearance, then rejoins the original hover.
                clearance['position'][2] += .06
                if assembly_clearance:
                    assembly_clearance['position'][2] += .06
    result['targets'] = list(target_map.values())
    # See both pads from the gripper thickness side. Keep the left eye low
    # for its fixture pickups; raise the supporting right eye above the left
    # gripper's insertion sweep. Camera clearance is checked independently.
    for camera in result.get('camera_specs', result.get('cameras', [])):
        if camera.get('view_type') == 'wrist':
            offset = [-.08, 0., .12] if camera.get('robot', camera.get('owner')) == 'franka_right' else [-.12, 0., .08]
            camera.update(prim_path='/World/env_0/cameras/assisted_'+camera['name'],
                          translation=[0.,0.,0.], orientation=[1.,0.,0.,0.],
                          native_mount_offset=offset, mount_offset=offset,
                          target_source='native_fingertip_midpoint')
            for key in ['orientation_euler','look_offset','position','look_at']:camera.pop(key,None)
    physical_phases = {p['name']: p for p in physical['phases']}
    installed = []
    for phase in result['phases']:
        old = physical_phases.get(phase['name'])
        if old and phase['name'].endswith(('_move_above', '_preshape', '_descend',
                '_close_and_attach', '_lift', '_pickup_clearance',
                '_assembly_clearance', '_transport_hover')):
            phase['local_skill'] = copy.deepcopy(old['local_skill'])
            phase['timeout_steps'] = old['timeout_steps']
            if old.get('attach'):
                phase['attach'] = copy.deepcopy(old['attach'])
        phase['beam_assisted'] = True
        # No carry teleport, collision suppression, gravity changes or planner
        # target rebasing. Installed assistance holds the measured pose.
        for key in ('kinematic_carry', 'gripper_sibling_filters',
                    'collision_filters', 'rebase_from_object'):
            phase.pop(key, None)
        for entry in phase.get('object_collisions', []):
            entry['enabled'] = True
        for entry in phase.get('object_gravity', []):
            entry['enabled'] = True
        skills = [phase.get('local_skill', {}),
                  *phase.get('local_skills', {}).values()]
        for skill in skills:
            skill['beam_assisted'] = True
            skill['use_arm_ik_controller'] = False
            skill['limit_command_to_measured_state'] = True
            skill['guard_ik_branch_jump'] = True
            skill['max_command_tracking_error'] = .12
            skill['max_wrist_command_tracking_error'] = .10
            skill['preferred_joint_abs_limit'] = None
            skill['hard_preferred_joint_abs_limit'] = False
            if skill.get('name') in ('ur5e_close_gripper', 'close_gripper'):
                skill['beam_assisted_visual_pinch'] = True
            if skill.get('relative_to_current_tcp') and not skill.get('requires_held_object'):
                skill['lock_target_orientation'] = True
                skill['use_command_warm_start'] = True
                skill['require_warm_start_ik'] = True
                if phase['name'].endswith('_park') and skill.get('robot') == 'franka_left':
                    # The old full 35 cm return folds an open fingertip into
                    # the upper arm. Keep the same retreat direction but stop
                    # 7 cm earlier; the next approach is swept from this pose.
                    skill['offset'] = [float(v) * .8 for v in skill['offset']]
            if skill.get('requires_held_object'):
                skill.pop('beam_coordinated', None)
                # The weld holds the measured grasp. Friction preload caches
                # must not become the reference for this separate controller.
                for key in list(skill):
                    if (key.startswith(('grasp_drive_', 'relax_fixed_attachment_',
                                        'beam_carry_', 'beam_turn_'))
                            or key in {'inherit_grasp_drive_target',
                                       'hold_previous_grasp_command',
                                       'track_published_grasp_command',
                                       'no_descent_during_clearance'}):
                        skill.pop(key)
                skill.update(ik_reference_mode='hybrid',
                             ik_reference_command_max_tracking_error=.12,
                             cartesian_orientation_command_warm_start=True,
                             cartesian_orientation_command_lookahead=.02,
                             target_object_servo_position_command_warm_start=True,
                             target_object_servo_position_command_lookahead=.003,
                             target_object_servo_position_command_accumulation_step=.0002,
                             target_object_servo_position_command_gate_overdrive=True,
                             target_object_use_measured_orientation_for_position_servo=False,
                             preferred_joint_abs_limit=None,
                             hard_preferred_joint_abs_limit=False,
                             cartesian_position_step=.001,
                             cartesian_orientation_step=.004,
                             max_command_joint_step=.02,
                             max_joint_step=.02,
                             ik_position_tolerance=.00005,
                             ik_orientation_tolerance=.0005)
                if skill.get('servo_target_object_pose') and skill.get('target_object_target'):
                    skill['beam_assisted_absolute_object_servo'] = True
                    skill['beam_assisted_fk_native_position_limit_m'] = .0001
                    skill['beam_assisted_fk_native_orientation_limit_rad'] = .0015
                    if (phase['name'].startswith('assemble_')
                            and phase['name'].endswith(('_pickup_clearance',
                                '_assembly_clearance', '_transport_hover'))):
                        # Preserve the loaded lift's final target, then ramp
                        # each free-move controller's first 0.1 s of physics.
                        skill['beam_assisted_phase_handoff_ticks'] = 24
                        skill['beam_assisted_production_cad_guard'] = True
                    if phase['name'] == 'assemble_02_part_1_pickup_clearance':
                        # Complete this loaded lift only after a real drive
                        # hold has settled, before the next orientation move.
                        skill['beam_assisted_transition_settle'] = {
                            'motion_metric': 'native_com_pose_difference',
                            'stable_native_ticks': 24,
                            'minimum_hold_native_ticks': 48,
                            'maximum_hold_native_ticks': 480,
                            'maximum_linear_speed_m_s': .002,
                            'maximum_angular_speed_rad_s': .02,
                        }
                    if (phase['name'].endswith('_assembly_clearance')
                            and skill.get('object') in (
                                'fabrica_beam_2', 'fabrica_beam_1', 'fabrica_beam_0')):
                        # Shared loaded-carry template. Each object still uses
                        # its own measured entry, grasp and configured target.
                        skill['beam_assisted_staged_clearance'] = {
                            'height_padding_m': .04,
                            'position_tolerance_m': .001,
                            'orientation_tolerance_rad': .003,
                        }
                        if skill['object'] in ('fabrica_beam_1', 'fabrica_beam_0'):
                            skill['beam_assisted_lift_command_rotation_hold'] = True
                            skill['beam_assisted_incremental_clearance'] = True
                    target_name = skill['target_object_target']
                    if skill.get('object') != 'fabrica_beam_6' and (
                            target_name.endswith(('_assembly_hover', '_assembled'))
                            or '_insert_' in target_name):
                        # The source-bound CAD corridor certificate is supplied
                        # by the launch gate, not embedded here (which would
                        # make the configuration signature self-referential).
                        skill['target_object_convergence_axis'] = [0., 0., -1.]
                        skill['beam_assisted_descent_admission'] = {
                            'motion_metric': 'native_com_pose_difference',
                            'lateral_tolerance_m': .00015,
                            'orientation_tolerance_rad': .001,
                            'stable_native_ticks': 12,
                            'max_linear_speed_m_s': .02,
                            'max_angular_speed_rad_s': .2,
                            'geometry_certificate': 'C01_full_insert_corridor',
                            'published_axial_slack_m': .00002,
                            'published_lateral_slack_m': .00002,
                        }
                        skill['beam_assisted_alignment_hold'] = {
                            'maximum_lateral_compensation_m': .00012,
                            'maximum_integral_step_m': .000002,
                            'integral_gain_per_native_tick': .02,
                            'cold_actual_axial_limit_m': .0005,
                        }
                if phase['name'].endswith('_lift'):
                    skill.update(cartesian_position_step=.0002,
                                 cartesian_orientation_step=.0005,
                                 max_command_joint_step=.004, max_joint_step=.004,
                                 ik_position_tolerance=.00001,
                                 ik_orientation_tolerance=.0001)
            if '_insert_' in skill.get('target_object_target', ''):
                # Do not let remembered servo commands cross a loaded waypoint.
                skill['target_object_servo_position_command_gate_overdrive'] = False
            if skill.get('target_object_target', '').endswith('_assembled'):
                # End loaded motion above contact. After full opening the
                # measured world-support joint settles the final 3 mm with
                # the existing bounded anchor controller, without body writes.
                # Resolve the original named target after layout randomization,
                # then add the world-Z offset in the actual skill adapter. A
                # copied target name is absent from the randomization group.
                skill['target_object_offset'] = [0., 0., .003]
                skill['target_object_offset_frame'] = 'world'
                skill['target_object_servo_position_command_gate_overdrive'] = False
                # Final convergence uses the full object-position error. The
                # generic split-axis bounds otherwise replace that strict gate.
                skill.pop('target_object_axial_position_tolerance', None)
                skill.pop('target_object_lateral_position_tolerance', None)
                skill.update(position_tolerance=.001,
                             relaxed_position_tolerance=.001,
                             target_object_position_tolerance=.001,
                             relaxed_target_object_position_tolerance=.001,
                             orientation_tolerance=.003,
                             target_object_orientation_tolerance=.003)
                skill['cartesian_position_step'] = .00025
                skill['cartesian_orientation_step'] = .001
        for attach in phase.get('attach', []):
            attach.update(beam_assisted=True, attachment_mode='fixed_joint',
                          beam_assisted_visual_pinch=True,
                          continuous_physics=False,
                          allow_shared_physical_grasp=False,
                          attachment_relative_pose_source='current',
                          disable_collision_on_attach=False,
                          filter_gripper_collisions_on_attach=False,
                          allow_noncontact_joint_attachment=False,
                          force_enable_collision_on_attach=True,
                          physical_attach_surface_gap=.001)
        for key in ('fixture_lock', 'lock'):
            entries = phase.get(key, [])
            if isinstance(entries, dict):
                entries = [entries]
            for lock in entries:
                lock.update(beam_assisted=True, disable_collision_on_lock=False,
                            freeze_current_pose=True, pin_pose=False,
                            teleport_to_target_once=False,
                            snap_free_object=False)
                lock.pop('rebase_targets', None)
                if str(lock.get('target', '')).endswith('_assembled'):
                    lock.update(position_tolerance=.0045,
                                orientation_tolerance=.025,
                                use_lock_spec_tolerances=True)
                    if lock['object'] not in installed:
                        installed.append(lock['object'])
        phase['beam_assisted_installed_parts'] = list(installed)
    _add_visual_base_support(result)
    home_targets = _append_final_home_return(result)
    # The final record explicitly distinguishes assisted stability from friction
    # grasp qualification. The ordinary phase runner must still finish all work.
    result.setdefault('metadata', {})['beam_execution_route'] = 'assisted'
    result['metadata']['formal_physical_success'] = False
    result['metadata']['assistance'] = 'measured_grasp_and_install_fixed_joints'
    for criterion in result['success']:
        criterion.update(position_tolerance=.003, orientation_tolerance=.04)
    result['phases'].append({
        'name': 'beam_assisted_final_verification', 'beam_assisted': True,
        'beam_assisted_empty_return_guard': True,
        'beam_assisted_home_targets': home_targets,
        'beam_assisted_installed_parts': [c['object'] for c in result['success']],
        'timeout_steps': 2400,
        'advance': {'type': 'all_of', 'physics_stable_steps': 240,
                    'conditions': [
                        {'type': 'object_targets_reached',
                         'objects': [
                             {'object': c['object'], 'target': c['target'],
                              'position_tolerance': .003,
                              'orientation_tolerance': .04}
                             for c in result['success']]},
                        {'type': 'objects_static', 'objects': [
                             {'object': c['object'],
                              'linear_velocity_threshold': c.get('linear_velocity_threshold', .05),
                              'angular_velocity_threshold': c.get('angular_velocity_threshold', 2.)}
                             for c in result['success']]},
                        {'type': 'objects_detached',
                         'objects': [c['object'] for c in result['success']]}]},
        'gripper_commands': {r['name']: 'open' for r in result['robots']},
    })
    result['max_steps'] = max(int(result.get('max_steps', 0)),
                              sum(int(p.get('timeout_steps', 1200))
                                  for p in result['phases']) + 1000)
    for phase in result['phases']:
        phase['beam_assisted_native_joint_limits'] = True
        if phase['name'] in ('assemble_02_part_1_move_above', 'assemble_03_part_0_move_above'):
            # A timer and modulo-angle proximity do not certify the native
            # winding needed for the following loaded turn. Move the empty
            # arm to the already configured branch, under the original rate,
            # tracking and whole-phase timeout; never rewrite native q.
            skill = phase['local_skill']
            skill.update(prealign_until_converged=True,
                         prealign_joint_position_tolerance=.05,
                         prealign_timeout_steps=phase['timeout_steps'],
                         unwrap_revolute_joints=False)
    return result
