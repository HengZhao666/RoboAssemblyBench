"""Opt-in Beam choreography using the 2026-09-30 native-mesh audit.

All poses are tool0 in the common part/assembly frame. These are geometric
candidates, not certificates of physical grasp or path feasibility. After the v152 triangle-surface audit, cap 1 is raised 4 mm and cap 0
8 mm; both caps preshape overhead. All changes apply at pickup and assembly.
"""
from __future__ import annotations
import copy
import json
from pathlib import Path
import numpy as np
from toolkits.factory_dual_franka_assembly.fabrica_online_planner_adapter import _matrix_to_quat_wxyz


def configure_beam_velocity_solver(objects, iterations):
    """Opt-in numerical convergence trial; preserve all physical materials.

    Apply only to Beam rigid parts, never the fixture, table, robots or another
    assembly. No velocity is assigned and no constraint is added or removed.
    """
    if type(iterations) is not int or not 4 <= iterations <= 32:
        raise ValueError('Beam velocity solver iterations must be an integer in [4, 32]')
    for entry in objects:
        if str(entry.get('name', '')).startswith('fabrica_beam_') and entry.get('rigid_body'):
            entry['solver_velocity_iteration_count'] = iterations


def audited_grasp(original, key):
    matrices = json.loads(Path(__file__).with_name('beam_coordinated_poses.json').read_text())
    pose = np.asarray(matrices[key], dtype=float)
    inverse = np.linalg.inv(pose)
    result = copy.deepcopy(original)
    result.update(tcp_in_assembly_position=pose[:3, 3].tolist(),
                  tcp_in_assembly_orientation=_matrix_to_quat_wxyz(pose[:3, :3]).tolist(),
                  object_in_tcp_position=inverse[:3, 3].tolist(),
                  object_in_tcp_orientation=_matrix_to_quat_wxyz(inverse[:3, :3]).tolist(),
                  preserve_audited_tcp=True, audited_pose=key,
                  robotiq_open_ratio=1.0 - (.74 if key.startswith('upper') else .72 if key.startswith('side') else .71) / .8)
    return result


def select_grasps(task, spec):
    if spec.get('assembly') != 'beam' or not spec.get('continuous_physics'):
        raise ValueError('coordinated_support_plan requires continuous physical Beam')
    if [str(s['move_part']) for s in task['assembly_steps']] != ['3', '2', '1', '0']:
        raise ValueError('Coordinated Beam requires order 3,2,1,0')
    if not np.allclose(spec['assembly_orientation'], [1, 0, 0, 0]):
        raise ValueError('Audited Beam route requires assembly yaw zero')
    for step, hold in zip(task['assembly_steps'], ['6', '3', '3', '2']):
        pid = str(step['move_part'])
        step['move_grasp'] = audited_grasp(step['move_grasp'],
                                         ('upper_' if pid in ['2', '3'] else 'shallow_') + pid)
        step['optimizer_hold_part'] = int(hold)
        if hold != '6':
            step['hold_grasp'] = audited_grasp(step['hold_grasp'], 'side_' + hold)


def add_preseat_target(place, targets, assembly_target_names):
    """End position-only descent before contact; keep assembly targets intact."""
    target = copy.deepcopy(next(t for t in targets if t['name'] == 'part_6_assembled'))
    target['name'] = 'part_6_preseat'
    target['position'][2] += .004
    targets.append(target)
    assembly_target_names.append(target['name'])
    place['local_skill'].update(target_object_target=target['name'],
        position_tolerance=.002, relaxed_position_tolerance=.002,
        target_object_position_tolerance=.002, relaxed_target_object_position_tolerance=.002)


def coordinate_phases(phases, task, spec, targets, assembly_target_names):
    from roboassemblybench.core.beam_physical_compiler import _skill_phase
    left, right = spec.get('assembly_robot', 'franka_left'), spec.get('base_robot', 'franka_right')
    obj = lambda pid: 'fabrica_beam_' + str(pid)
    by_name = {p['name']: p for p in phases}
    prefix = {str(s['move_part']): f"assemble_{i:02d}_part_{s['move_part']}"
              for i, s in enumerate(task['assembly_steps'])}

    def hold(phase, robot, pid):
        phase.setdefault('local_skills', {})[robot] = {
            'name': 'ur5e_hold_part_end', 'robot': robot, 'object': obj(pid),
            'requires_held_object': True, 'hold_previous_grasp_command': True,
            'use_arm_ik_controller': False, 'gripper_command': 'contact_hold',
            'max_command_tracking_error': .18, 'max_wrist_command_tracking_error': .12}
        phase.setdefault('gripper_commands', {})[robot] = 'contact_hold'

    def motion(name, robot, offset, frame='world', reference_object=None):
        return _skill_phase(skill='retreat_vertical', robot=robot, phase_name=name,
            timeout_steps=3000, parameters={
                'relative_to_current_tcp': True, 'offset': offset, 'offset_frame': frame,
                'offset_reference_object': reference_object,
                'gripper_command': 'open', 'position_tolerance': .002,
                'orientation_tolerance': .025, 'cartesian_position_step': .001,
                'cartesian_orientation_step': .004, 'guard_ik_branch_jump': True,
                'ik_branch_jump_limit': .15, 'max_command_joint_step': .006,
                'max_joint_step': .006, 'use_arm_ik_controller': False,
                'use_command_warm_start': True, 'require_warm_start_ik': True})

    # Selective releases are essential when both robots contact the same part.
    for phase in phases:
        phase['beam_coordinated'] = True
        for entry in phase.get('detach', []):
            if isinstance(entry, dict):
                entry['robot'] = next(iter(phase['gripper_commands']))
        skills = [phase.get('local_skill', {}), *phase.get('local_skills', {}).values()]
        for skill in skills:
            skill['beam_coordinated'] = True
        main = phase.get('local_skill', {})
        name = phase['name']
        axis = 'x' if name.startswith(('hold_2_', 'hold_3_', prefix['0'], prefix['1'])) else 'y'
        for entry in phase.get('attach', []):
            entry.update(allow_shared_physical_grasp=True, physical_contact_axes=axis)
        if name.endswith('_clear_support_filters') and not main:
            condition = next(c for c in phase['advance']['conditions'] if c.get('type') == 'object_attached')
            hold(phase, condition['robot'], condition['object'].rsplit('_', 1)[-1])
            phase['local_skill'] = phase['local_skills'].pop(condition['robot'])
            main = phase['local_skill']
        if name.startswith((prefix['0'], prefix['1'])):
            # Keep the wide opening sweep above the nest walls. At q=.55
            # the jaws still clear the cap, then close under force regulation.
            if name.endswith('_preshape'):
                main['gripper_openness'] = 1.0 - .55 / .8
            elif name.endswith('_descend'):
                main['gripper_command'] = 1.0 - .55 / .8
        if main.get('name') == 'ur5e_close_gripper':
            main.update(physical_contact_axes=axis, allow_shared_physical_grasp=True)
        if name.startswith(('hold_2_', 'hold_3_')) and main.get('grasp_relative_position') is not None:
            # Enter from the outside of the assembly, not down through its caps.
            main.update(align_grasp_axis_to_world_down=False, guard_ik_branch_jump=True,
                        use_arm_ik_controller=False, use_command_warm_start=True,
                        require_warm_start_ik=True, position_tolerance=.001,
                        relaxed_position_tolerance=.001, orientation_tolerance=.02,
                        cartesian_position_step=.001, max_command_joint_step=.004,
                        max_joint_step=.004, trace_pregrasp=True)
            if name.endswith('_move_above'):
                main.update(offset=[0, -.10, 0], offset_frame='object',
                            orientation_first_before_translation=False)
            else:
                main.pop('offset', None)
                main.pop('offset_frame', None)

    # 6 -> 3: left keeps its insertion grasp until right side contact is valid.
    first_release = [by_name[prefix['3'] + '_' + s] for s in ('release', 'retreat', 'park')]
    transfer3 = [p for p in phases if (p['name'].startswith('hold_6_to_3_') or
                p['name'] in ['hold_3_' + suffix for suffix in
                ('approach_clearance','move_above','preshape','descend','close_and_attach')])]
    for p in transfer3:
        hold(p, left, '3')
    old = by_name['hold_3_approach_clearance']
    replacement = motion(old['name'], right, [0, 0, -.12], 'tcp')
    old.clear(); old.update(replacement); hold(old, left, '3')
    for p in first_release:
        hold(p, right, '3')
    phases[:] = [p for p in phases if p not in transfer3]
    index = phases.index(first_release[0])
    phases[index:index] = transfer3

    # 3 -> 2: establish the left upper grasp before releasing the right side.
    temp = []
    for suffix in ('move_above', 'preshape', 'descend', 'close_and_attach'):
        p = copy.deepcopy(by_name[prefix['2'] + '_' + suffix])
        p['name'] = 'handover_left_2_' + suffix
        p.pop('local_skills', None)
        hold(p, right, '3')
        temp.append(p)
    transfer2 = [p for p in phases if p['name'].startswith(('hold_3_to_2_', 'hold_2_'))
                 and not p['name'].startswith('hold_2_final')]
    for p in transfer2:
        hold(p, left, '2')
    old = by_name['hold_2_approach_clearance']
    replacement = motion(old['name'], right, [0, -.10, 0], reference_object=obj('3'))
    old.clear(); old.update(replacement); hold(old, left, '2')
    # move_above moves sideways at Y=-.26; descend approaches along +Y.
    index = phases.index(transfer2[0]); phases[index:index] = temp
    release2 = []
    for suffix in ('release', 'retreat', 'park'):
        p = copy.deepcopy(by_name[prefix['2'] + '_' + suffix])
        p['name'] = 'handover_left_2_' + suffix
        hold(p, right, '2'); release2.append(p)
    index = phases.index(transfer2[-1]) + 1; phases[index:index] = release2

    # Seating must move the loaded command and prove table support, not merely
    # observe a stationary suspended part. The runtime gate counts physics steps.
    add_preseat_target(by_name['base_6_place'], targets, assembly_target_names)
    seat = by_name['base_6_set_down']
    seat.pop('local_skills', None)
    seat['local_skill'] = {
        'name': 'ur5e_hold_part_end', 'robot': right, 'object': obj('6'),
        'beam_table_seat': True, 'gripper_command': 'contact_hold',
        'requires_held_object': True, 'use_arm_ik_controller': False,
        'grasp_drive_position_lookahead': .001, 'grasp_drive_joint_lookahead': .04,
        'cartesian_position_step': .00002, 'cartesian_orientation_step': .001,
        'max_command_joint_step': .004, 'max_joint_step': .004,
        'max_command_tracking_error': .18, 'max_wrist_command_tracking_error': .12,
        'guard_ik_branch_jump': True, 'ik_branch_jump_limit': .15,
        'ik_position_tolerance': 1e-7, 'ik_orientation_tolerance': 1e-5,
        'use_command_warm_start': True, 'require_warm_start_ik': True,
        'seat_max_travel': .008, 'seat_max_unload_travel': .001,
        'seat_payload_mass': .25, 'seat_contact_force': .1,
        'seat_contact_wait': .2, 'seat_force_deadband': .25,
        'seat_approach_speed': .0015, 'seat_near_speed': .00015,
        'seat_slow_gap': .00075, 'seat_load_speed': .0001,
        'seat_up_speed': .0005, 'seat_admittance': .00015,
        'seat_max_linear_speed': .002, 'seat_max_angular_speed': .03,
        'seat_motion_window': .1, 'seat_max_position_excursion': .0003,
        'seat_max_orientation_excursion': .005,
        'seat_raw_linear_speed_limit': .05, 'seat_raw_angular_speed_limit': .5}
    seat['timeout_steps'] = 4800
    seat['advance'] = {'type': 'beam_table_supported', 'stable_steps': 96,
                       'minimum_vertical_force': 1.962}
    # Eliminate the pickup clearance target that lowered the base after lifting.
    phases.remove(by_name['base_6_pickup_clearance'])
    by_name['base_6_assembly_clearance']['local_skill']['no_descent_during_clearance'] = True
    # Side support withdraws outward after the caps are installed.
    final = by_name['hold_2_final_retreat']
    replacement = motion(final['name'], right, [0, -.12, 0], reference_object=obj('2'))
    final.clear(); final.update(replacement)
    phases.append({'name': 'beam_final_unloaded_verification', 'beam_coordinated': True,
        'timeout_steps': 1200, 'robot_targets': {},
        'gripper_commands': {left: 'open', right: 'open'},
        'advance': {'type': 'all_of', 'conditions': [
            {'type': 'beam_table_supported', 'stable_steps': 240, 'minimum_vertical_force': 9.81},
            {'type': 'objects_detached', 'objects': [obj(p) for p in ['6','3','2','1','0']]},
            {'type': 'object_targets_reached', 'tolerance': .003, 'orientation_tolerance': .04,
             'objects': [{'object': obj(p), 'target': f'part_{p}_assembled'} for p in ['6','3','2','1','0']]},
            {'type': 'objects_static', 'objects': [obj(p) for p in ['6','3','2','1','0']],
             'linear_velocity_threshold': .002, 'angular_velocity_threshold': .02}]}})
    seated = False
    installed = ['6']
    for phase in phases:
        for pid, name in prefix.items():
            if phase['name'] == name + '_physical_hold_check' and pid not in installed:
                installed.append(pid)
        phase['beam_installed_parts'] = list(installed)
        phase['beam_coordinated'] = True
        phase['beam_table_support_objects'] = ['optical_board', 'assembly_support', 'fixture_support', 'factory_tabletop_visual']
        phase['beam_require_table_support'] = seated
        if seated or phase['name'] == 'base_6_set_down':
            # Trial relief after persistent real contact; not a seating gate.
            # Peak-finger target 3--4 N, no detachment or grasp-anchor recapture.
            phase['beam_table_contact_force_relief'] = {
                'qualification_force': .25, 'retain_force': .1, 'stable_steps': 24,
                'force_low': 3., 'force_high': 4., 'joint_rate': .0024}
        if phase['name'] == 'base_6_set_down': seated = True
