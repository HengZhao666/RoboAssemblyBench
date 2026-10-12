"""Exercise the actual pure gates without importing or starting Isaac Sim."""
from __future__ import annotations

import ast
import copy
import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter


def extracted_definitions(relative_path, names):
    source = ast.parse((ROOT / relative_path).read_text())
    selected = [node for node in ast.walk(source) if isinstance(node, ast.FunctionDef) and node.name in names]
    assert {node.name for node in selected} == set(names)
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), *selected], type_ignores=[])
    scope = {'np': np, 'copy': copy, 'json': json, 'os': os}
    exec(compile(ast.fix_missing_locations(module), relative_path, 'exec'), scope)
    return {name: scope[name] for name in names}


Task = type('Task', (), extracted_definitions(
    'internutopia_extension/tasks/factory_dual_franka_assembly_task.py',
    ['_strict_physical_grasp_contact', '_strict_dual_finger_contact', '_apply_phase_actions',
     '_pair_contact_observation', '_tensor_to_numpy', '_write_physical_contact_trace'],
))
apply_policy = extracted_definitions('roboassemblybench/core/fabrica_canonical.py', ['_apply_continuous_beam_physics'])['_apply_continuous_beam_physics']


class BeamPhysicsContract(unittest.TestCase):
    def setUp(self):
        self.task = Task()
        self.task._is_slender_attach_object = lambda *a, **kw: True
        self.task._attachment_mode = lambda spec: 'pure_physical_grasp'
        self.task._write_physical_contact_trace = lambda *a: None
        self.spec = {'require_force_contact': True, 'require_dual_force_contact': True,
                     'require_opposing_force_contact': True, 'physical_contact_axes': 'y',
                     'physical_attach_surface_gap': 0.0035, 'physical_contact_interior_scale': 1.0,
                     'physical_contact_interior_margin': 0.0}

    def metrics(self, *, force=True, right_side=-1, x=0.02, axis='y'):
        def finger(side):
            point = [x, side * 0.00889, 0.0]
            local = {'local_point': point, 'axes': {axis: {'contact': True, 'surface_gap': 0.001, 'signed_coordinate': side * 0.00889}}}
            return {'force_contact': force, 'force_probe_valid': True, 'geometric_contact': True,
                    'surface_gap': 0.001, 'local_contact': local, 'sample_contacts': [{'local_contact': local}]}
        return {'left_finger': finger(1), 'right_finger': finger(right_side),
                'pinch_axis': axis, 'contact_box_scale': [0.1524, 0.01778, 0.0127]}

    def ready(self, metrics):
        return self.task._strict_physical_grasp_contact('beam', metrics, self.spec)['physical_contact_ready']

    def test_valid_zero_force_cannot_pass_geometric_grasp(self):
        self.assertFalse(self.ready(self.metrics(force=False)))

    def test_dual_force_requires_opposing_interior_allowed_faces(self):
        self.assertTrue(self.ready(self.metrics()))
        self.assertFalse(self.ready(self.metrics(right_side=1)))
        self.assertFalse(self.ready(self.metrics(x=0.09)))
        self.assertFalse(self.ready(self.metrics(axis='x')))

    def test_invalid_probe_rejects_even_positive_force(self):
        metrics = self.metrics()
        metrics['right_finger']['force_probe_valid'] = False
        self.assertFalse(self.ready(metrics))

    def test_dynamic_noop_preserves_grasp_anchor_and_grace(self):
        state = {'position': [1, 2, 3], 'orientation': [1, 0, 0, 0],
                 'attach_spec': {'continuous_physics': True, 'hold_grace_allow_missing_contact': False}}
        self.task._attachments = {'beam': state}
        self.task._ensure_static_environment_locked = lambda: None
        self.task._as_list = lambda value: [] if value is None else value if isinstance(value, list) else [value]
        self.task._extract_object_name = lambda entry: entry['object']
        before = copy.deepcopy(state)
        self.task._apply_phase_actions({'kinematic_carry': [{'object': 'beam', 'enabled': False}]})
        self.assertEqual(state, before)
        with self.assertRaises(ValueError):
            self.task._apply_phase_actions({'kinematic_carry': [{'object': 'beam', 'enabled': True}]})

    def test_pair_force_does_not_read_aggregate_sensor_force(self):
        self.task._contact_physics_dt = lambda: 1 / 240
        self.task._get_contact_sensor = lambda *a: self.fail('must not read total finger force')
        self.task._get_contact_probe = lambda *a: SimpleNamespace(
            is_physics_handle_valid=lambda: True,
            get_contact_force_matrix=lambda **kw: np.zeros((1, 1, 3)),
            get_contact_force_data=lambda **kw: (np.zeros((4, 1)), np.zeros((4, 3)), np.zeros((4, 3)),
                                                 np.zeros((4, 1)), np.zeros((1, 1), dtype=int), np.zeros((1, 1), dtype=int)),
        )
        observation = self.task._pair_contact_observation('/finger', '/beam')
        self.assertTrue(observation['valid'])
        self.assertEqual(observation['force'], 0)
        self.assertEqual(observation['contacts'], [])

    def test_pair_details_use_count_then_start(self):
        self.task._contact_physics_dt = lambda: 1 / 240
        self.task._get_contact_probe = lambda *a: SimpleNamespace(
            is_physics_handle_valid=lambda: True,
            get_contact_force_matrix=lambda **kw: np.array([[[0., 3., 0.]]]),
            get_contact_force_data=lambda **kw: (np.array([[99.], [3.], [77.]]), np.arange(9).reshape(3, 3),
                                                 np.zeros((3, 3)), np.zeros((3, 1)), np.array([[1]]), np.array([[1]])),
        )
        observation = self.task._pair_contact_observation('/finger', '/beam')
        self.assertEqual(observation['contacts'][0]['normal_force'], 3)
        self.assertEqual(observation['contacts'][0]['point_world'], [3, 4, 5])

    def test_policy_disables_all_width_and_motion_shortcuts(self):
        phase = {'name': 'base_grip_handoff_dynamic', 'kinematic_carry': [{'enabled': False}],
                 'object_gravity': [{'enabled': False}], 'gripper_sibling_filters': [{'enabled': True}],
                 'object_collisions': [{'enabled': False}], 'collision_filters': [{'enabled': True}],
                 'local_skill': {'name': 'ur5e_close_gripper', 'allow_jaw_width_completion': True},
                 'attach': [{'attachment_mode': 'pure_physical_grasp', 'allow_jaw_width_contact_for_physical_grasp': True}]}
        apply_policy([phase])
        self.assertNotIn('kinematic_carry', phase)
        self.assertNotIn('gripper_sibling_filters', phase)
        self.assertTrue(phase['object_gravity'][0]['enabled'])
        self.assertFalse(phase['collision_filters'][0]['enabled'])
        self.assertTrue(phase['object_collisions'][0]['enabled'])
        self.assertFalse(phase['local_skill']['allow_jaw_width_completion'])
        self.assertFalse(phase['local_skill']['use_joint_stall_for_close_until_contact'])
        self.assertFalse(phase['attach'][0]['allow_jaw_width_contact_for_physical_grasp'])
        self.assertTrue(phase['attach'][0]['require_opposing_force_contact'])


class ContactControlRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        self.spec = {'close_force_regulation': True, 'contact_force_threshold': 0.1,
                     'close_gate_recenter_single_finger_contact': True,
                     'close_gate_recenter_stable_steps': 4, 'close_force_validation_steps': 96,
                     'close_contact_stable_steps': 36}

    @staticmethod
    def metrics(left, right, valid=True):
        def finger(force, side):
            return {'force': force, 'force_probe_valid': valid,
                    'local_contact': {'best_surface_gap': 0.0005},
                    'force_observation': {'physics_dt': 1 / 240,
                        'contacts': [{'normal_force': force, 'point_world': [0., side * 0.009, 0.]}]}}
        return {'left_finger': finger(left, 1), 'right_finger': finger(right, -1),
                'contact_box_center': [0., 0., 0.], 'contact_box_orientation': [1., 0., 0., 0.]}

    def command(self, state, metrics, q=0.66):
        return self.adapter._contact_close_control(state=state, spec=self.spec, metrics=metrics,
            gripper_q=q, open_q=0., closed_q=0.8, requested_openness=0.0615, min_openness=0.08)

    def test_first_contact_captures_measured_q_without_old_squeeze(self):
        state = {'contact_command_closure': 0.73}
        control = self.command(state, self.metrics(6., 8.))
        self.assertTrue(control['first_contact'])
        self.assertAlmostEqual(control['command_q'], 0.66)
        self.assertGreater(control['command_openness'], 0.08)

    def test_overload_unloads_and_lost_contact_never_resumes_fast_ramp(self):
        state = {}
        self.command(state, self.metrics(5., 5.))
        control = self.command(state, self.metrics(80., 120.))
        self.assertLess(control['command_q'], 0.66)
        self.assertFalse(control['force_within_limit'])
        control = self.command(state, self.metrics(0., 0.))
        self.assertLess(control['command_q'], 0.661)
        self.assertEqual(control['mode'], 'gentle_seek')

    def test_invalid_probe_never_advances_closure(self):
        control = self.command({}, self.metrics(0., 0., valid=False))
        self.assertLessEqual(control['command_q'], 0.66)
        self.assertFalse(control['force_within_limit'])

    def test_force_recenters_even_when_both_geometry_gaps_are_small(self):
        state = {}
        for _ in range(4):
            result = self.adapter._update_close_recenter_offset(state=state, spec=self.spec,
                close_detail={'contact_detail': {'contact_metrics': self.metrics(0., 5.)}}, close_ready=False)
        self.assertTrue(result['updated'])
        np.testing.assert_allclose(state['recenter_offset_world'], [0., -0.0002, 0.])

    def test_recenter_waits_for_motion_and_stops_if_imbalance_worsens(self):
        state = {}
        for _ in range(4):
            self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 5.))
        result = self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 5.))
        self.assertEqual(result['reason'], 'servo_in_progress')
        state['recenter_target_ready'] = True
        for _ in range(8):
            result = self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 10.))
        self.assertEqual(result['reason'], 'force_imbalance_increased')

    def test_recenter_total_travel_is_bounded(self):
        state = {}
        for _ in range(300):
            state['recenter_target_ready'] = True
            self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 5.))
        self.assertLessEqual(state['force_recenter_travel'], 0.002 + 1e-12)

    def test_recenter_acknowledges_measured_step_with_static_servo_bias(self):
        spec = {**self.spec, 'close_force_recenter_measure_progress': True}
        state = {'close_tcp_position_world': np.array([0., 0.0003, 0.])}
        for _ in range(4):
            self.adapter._update_force_recenter(state=state, spec=spec, metrics=self.metrics(0., 5.))
        pending = state['force_recenter_pending']
        np.testing.assert_allclose(pending['motion_target_position_world'], [0., 0.0001, 0.])
        ready, _ = self.adapter._recenter_motion_ready(pending=pending, current_position=[0., 0.0003, 0.],
            overall_pose_ready=True, tolerance=0.0001)
        self.assertFalse(ready)
        ready, _ = self.adapter._recenter_motion_ready(pending=pending, current_position=[0., 0.0001, 0.],
            overall_pose_ready=True, tolerance=0.0001)
        self.assertTrue(ready)
        ready, _ = self.adapter._recenter_motion_ready(pending=pending, current_position=[0., 0.0001, 0.],
            overall_pose_ready=False, tolerance=0.0001)
        self.assertFalse(ready)

    def test_recenter_settling_requires_consecutive_ready_observations(self):
        state = {'force_recenter_pending': {'imbalance': 5., 'settle_steps': 0}, 'recenter_target_ready': True}
        for _ in range(7):
            self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 5.))
        self.assertEqual(state['force_recenter_pending']['settle_steps'], 7)
        state['recenter_target_ready'] = False
        self.adapter._update_force_recenter(state=state, spec=self.spec, metrics=self.metrics(0., 5.))
        self.assertEqual(state['force_recenter_pending']['settle_steps'], 0)

    def test_recenter_wait_is_bounded_without_claiming_motion_completed(self):
        state = {'force_recenter_pending': {'imbalance': 5., 'settle_steps': 0}, 'recenter_target_ready': False}
        spec = {**self.spec, 'close_force_recenter_timeout_steps': 20}
        for _ in range(20):
            result = self.adapter._update_force_recenter(state=state, spec=spec, metrics=self.metrics(0., 5.))
        self.assertEqual(result['reason'], 'recenter_motion_timeout')
        self.assertFalse(result['updated'])
        self.assertFalse(state['recenter_target_ready'])

    def test_transient_contact_does_not_complete_but_stable_window_preserves_command(self):
        state = {}
        task = SimpleNamespace(robots={'right': SimpleNamespace(config=SimpleNamespace(gripper_close_openness=0.08))})
        self.adapter._current_gripper_q = lambda **kw: 0.66
        self.adapter._gripper_open_closed_q = lambda **kw: (0., 0.8)
        def step(left, right):
            self.adapter._grasp_contact_ready = lambda **kw: (left > 0 and right > 0,
                {'contact_metrics': self.metrics(left, right)})
            return self.adapter._close_until_contact_ready(state=state, task=task, robot_name='right',
                spec=self.spec, tracked_objects={}, close_elapsed_steps=200, gripper_openness=0.0615)
        for _ in range(10):
            ready, detail = step(5., 8.)
            self.assertFalse(ready)
        ready, detail = step(0., 8.)
        self.assertFalse(ready)
        self.assertEqual(detail['stable_steps'], 0)
        for i in range(96):
            ready, detail = step(5., 8.)
            self.assertEqual(ready, i == 95)
        self.assertAlmostEqual(task._ur5e_plumbers_gripper_hold_openness['right'], 0.175)

    def test_carry_policy_keeps_contact_command_and_small_probe_lift(self):
        phase = {'name': 'base_6_peel_lift', 'local_skill': {'requires_held_object': True, 'robot': 'right',
                 'gripper_command': 0.0}, 'gripper_commands': {'right': 0.0},
                 'advance': {'conditions': [{'type': 'object_lifted', 'min_lift': 0.008}]}}
        apply_policy([phase])
        self.assertEqual(phase['local_skill']['gripper_command'], 'contact_hold')
        self.assertEqual(phase['gripper_commands']['right'], 'contact_hold')
        self.assertEqual(phase['local_skill']['offset'][2], 0.005)

    def test_drive_overrides_reach_robot_configuration(self):
        build = extracted_definitions('toolkits/factory_dual_franka_assembly/scene_builder.py', ['_build_robot_cfgs'])['_build_robot_cfgs']
        build.__globals__.update({
            '_resolve_orientation': lambda spec: np.array([1., 0., 0., 0.]),
            'UR5eRobotCfg': lambda **kw: SimpleNamespace(**kw),
            'ur5e_arm_ik_cfg': SimpleNamespace(update=lambda: None),
            'ur5e_arm_joint_cfg': SimpleNamespace(update=lambda: None),
            'ur5e_gripper_cfg': SimpleNamespace(update=lambda: None),
        })
        configs, _ = build({'robots': [{'name': 'right', 'type': 'UR5eRobot', 'prim_path': '/robot',
            'position': [0, 0, 0], 'gripper_close_openness': 0.08,
            'preserve_continuous_arm_joints': True,
            'gripper_drive_kp': 7500., 'gripper_drive_kd': 173., 'gripper_drive_max_effort': 26.,
            'gripper_pad_static_friction':1.,'gripper_pad_dynamic_friction':.8}]})
        self.assertEqual(configs[0].gripper_drive_kp, 7500.)
        self.assertEqual(configs[0].gripper_drive_kd, 173.)
        self.assertEqual(configs[0].gripper_drive_max_effort, 26.)
        self.assertEqual(configs[0].gripper_close_openness, 0.08)
        self.assertTrue(configs[0].preserve_continuous_arm_joints)
        self.assertEqual(configs[0].gripper_pad_static_friction,1.)
        self.assertEqual(configs[0].gripper_pad_dynamic_friction,.8)

    def test_material_trial_is_opt_in_and_rejects_invalid_coefficients(self):
        read=extracted_definitions('internutopia_extension/robots/ur5e.py', ['_gripper_contact_friction'])['_gripper_contact_friction']
        robot=SimpleNamespace(config=SimpleNamespace())
        self.assertEqual(read(robot),(80.,70.))
        robot.config=SimpleNamespace(gripper_pad_static_friction=1.,gripper_pad_dynamic_friction=.8)
        self.assertEqual(read(robot),(1.,.8))
        for static,dynamic in [(1.,None),(None,.8),(-1.,.8),(1.,2.),(float('nan'),.8),(1.,float('inf'))]:
            robot.config=SimpleNamespace(gripper_pad_static_friction=static,gripper_pad_dynamic_friction=dynamic)
            with self.assertRaises(ValueError):read(robot)


class SettledCloseRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        phase = {'name': 'close', 'local_skill': {'name': 'ur5e_close_gripper'}}
        apply_policy([phase])
        self.spec = phase['local_skill']
        self.pose = {'position': np.zeros(3), 'orientation': np.array([1., 0., 0., 0.])}

    def test_near_target_but_moving_never_starts_close(self):
        state = {}
        for i in range(80):
            self.pose['position'][0] = (i % 20) * 0.00005
            ready, _ = self.adapter._close_gate_stability(state=state, spec=self.spec, ready=True, current_pose=self.pose)
            self.assertFalse(ready)

    def test_stationary_window_resets_when_position_gate_is_lost(self):
        state = {}
        for _ in range(23):
            ready, _ = self.adapter._close_gate_stability(state=state, spec=self.spec, ready=True, current_pose=self.pose)
            self.assertFalse(ready)
        self.adapter._close_gate_stability(state=state, spec=self.spec, ready=False, current_pose=self.pose)
        for i in range(24):
            ready, _ = self.adapter._close_gate_stability(state=state, spec=self.spec, ready=True, current_pose=self.pose)
            self.assertEqual(ready, i == 23)

    def test_frozen_close_preserves_clock_on_pose_gate_loss(self):
        state = {'close_arm_frozen': True, 'close_started_step': 42, 'hold_q': np.zeros(6)}
        ready, _ = self.adapter._close_gate_stability(state=state, spec=self.spec, ready=False, current_pose=self.pose)
        self.assertFalse(ready)
        self.assertEqual(state['close_started_step'], 42)
        self.assertIn('hold_q', state)

    def test_arm_freezes_before_contact_and_does_not_chase_object(self):
        adapter = self.adapter
        target = copy.deepcopy(self.pose)
        adapter._target_pose = lambda **kw: copy.deepcopy(target)
        adapter._current_robot_pose = lambda **kw: copy.deepcopy(self.pose)
        adapter._current_tcp_pose = lambda **kw: kw['current_pose']
        adapter._current_arm_q = lambda *args: np.zeros(6)
        adapter._command_reference_q = lambda **kw: kw['current_q']
        adapter._ik_target_pose = lambda **kw: kw['target_pose']
        adapter._solve_ik = lambda **kw: np.zeros(6)
        adapter._ik_branch_jump_detected = lambda **kw: False
        adapter._continuous_command_q = lambda **kw: kw['command_q']
        adapter._limit_command_to_measured_state = lambda **kw: kw['command_q']
        adapter._remember_arm_command = lambda *args: None
        task = SimpleNamespace(step_counter=0, phase_step_counter=0, phase='close')
        for i in range(24):
            task.phase_step_counter = i
            ready, _, _ = adapter._close_pose_gate_action(phase_key='test', task=task, robot_name='right',
                spec=self.spec, tracked_robots={}, tracked_objects={})
        self.assertTrue(ready)
        self.assertTrue(adapter._close_gate_state['test']['close_arm_frozen'])
        self.assertFalse(adapter._close_gate_state['test'].get('contact_control_active', False))
        target['position'][2] = -0.006
        ready, _, detail = adapter._close_pose_gate_action(phase_key='test', task=task, robot_name='right',
            spec=self.spec, tracked_robots={}, tracked_objects={})
        self.assertTrue(ready)
        np.testing.assert_allclose(detail['target_position'], [0, 0, 0])

    def test_hold_failure_records_specific_reason_without_relaxing_gate(self):
        from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error
        methods = extracted_definitions('internutopia_extension/tasks/factory_dual_franka_assembly_task.py',
                                        ['_physical_hold_valid', '_record_physical_hold_result'])
        methods['_physical_hold_valid'].__globals__['pose_error'] = pose_error
        task = type('HoldTask', (), methods)()
        task.step_counter = 12
        task.phase = 'lift'
        task._PHYSICAL_HOLD_POSITION_SLIP = 0.003
        task._PHYSICAL_HOLD_ORIENTATION_SLIP = 0.087
        task._gripper_contact_metrics = lambda *a, **kw: {}
        task._strict_physical_grasp_contact = lambda *a, **kw: {'physical_contact_ready': True}
        task._physical_hold_within_grace = lambda *a: False
        task._get_robot_gripper_opening = lambda *a: 0.1
        task._gripper_opening_limit = lambda *a, **kw: 0.2
        task._jaw_width_contact = lambda *a: False
        snapshots = []
        task._write_physical_contact_trace = lambda *a, **kw: snapshots.append(kw)
        relative = [0.0029, 0, 0]
        task._current_relative_pose = lambda *a: (relative, [1., 0, 0, 0])
        state = {'robot_name': 'right', 'position': [0, 0, 0], 'orientation': [1., 0, 0, 0],
                 'attach_spec': {'continuous_physics': True}}
        self.assertTrue(task._physical_hold_valid('beam', state))
        relative[0] = 0.0031
        self.assertFalse(task._physical_hold_valid('beam', state, validation_stage='state_sync'))
        self.assertEqual(state['last_hold_validation']['reasons'], ['position_slip'])
        self.assertEqual(state['last_hold_validation']['validation_stage'], 'state_sync')
        self.assertEqual(state['last_hold_validation']['sample_index'], 2)
        self.assertEqual(snapshots[-1], {'force_write': True, 'validation_stage': 'state_sync'})
        task._strict_physical_grasp_contact = lambda *a, **kw: {'physical_contact_ready': False}
        self.assertFalse(task._physical_hold_valid('beam', state))
        self.assertEqual(state['last_hold_validation']['reasons'], ['position_slip', 'contact_missing'])
        task._get_robot_gripper_opening = lambda *a: 0.3
        self.assertFalse(task._physical_hold_valid('beam', state))
        self.assertEqual(state['last_hold_validation']['reasons'], ['gripper_opening'])


class ContactStopRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        phase = {'name': 'close', 'local_skill': {'name': 'ur5e_close_gripper'}}
        apply_policy([phase])
        self.spec = {**phase['local_skill'], 'close_gate_recenter_single_finger_contact': True}
        self.state = {
            'contact_arm_anchor': {'position': np.zeros(3), 'orientation': np.array([1., 0., 0., 0.])},
            'recenter_offset_world': np.array([0.0002, 0., 0.]), 'force_recenter_travel': 0.0002,
            'force_recenter_pending': {'offset_before': np.zeros(3),
                'motion_start_position_world': np.array([0.0003, 0., 0.]),
                'motion_delta_world': np.array([0.0002, 0., 0.]),
                'motion_target_position_world': np.array([0.0005, 0., 0.]),
                'imbalance': 5., 'settle_steps': 0, 'adaptive_progress': True},
        }

    def test_bilateral_cancel_retains_anchor_and_travel_budget(self):
        result = self.adapter._stop_recenter_on_bilateral(state=self.state, spec=self.spec,
            metrics=ContactControlRegression.metrics(3., 5.), current_position=[0.00035, 0.00004, 0.])
        self.assertAlmostEqual(result['progress_fraction'], 0.25)
        np.testing.assert_allclose(self.state['recenter_offset_world'], [0.00005, 0., 0.])
        np.testing.assert_allclose(self.state['contact_arm_anchor']['position'], [0., 0., 0.])
        self.assertEqual(self.state['force_recenter_travel'], 0.0002)
        self.assertNotIn('force_recenter_pending', self.state)
        self.assertNotIn('close_contact_stable_steps', self.state)

    def test_invalid_probe_or_pose_cannot_cancel(self):
        for metrics, position in [(ContactControlRegression.metrics(3., 5., valid=False), [0.00035, 0., 0.]),
                                  (ContactControlRegression.metrics(3., 0.), [0.00035, 0., 0.]),
                                  (ContactControlRegression.metrics(3., 5.), [float('nan'), 0., 0.])]:
            self.assertIsNone(self.adapter._stop_recenter_on_bilateral(
                state=self.state, spec=self.spec, metrics=metrics, current_position=position))
            self.assertIn('force_recenter_pending', self.state)

    def test_small_step_requires_actual_progress_and_full_pose_gate(self):
        pending = {'motion_delta_world': np.array([0.000025, 0., 0.]),
                   'motion_target_position_world': np.array([0.000025, 0., 0.]), 'adaptive_progress': True}
        ready, detail = self.adapter._recenter_motion_ready(pending=pending, current_position=[0., 0., 0.],
            overall_pose_ready=True, tolerance=0.0001)
        self.assertFalse(ready)
        self.assertAlmostEqual(detail['recenter_motion_axial_tolerance'], 0.00000625)
        ready, _ = self.adapter._recenter_motion_ready(pending=pending, current_position=[0.000025, 0.00004, 0.],
            overall_pose_ready=True, tolerance=0.0001)
        self.assertTrue(ready)
        ready, _ = self.adapter._recenter_motion_ready(pending=pending, current_position=[0.000025, 0., 0.],
            overall_pose_ready=False, tolerance=0.0001)
        self.assertFalse(ready)

    def test_reversal_halves_step_once_after_fresh_confirmation(self):
        state = {'force_recenter_last_side': 1, 'close_tcp_position_world': np.zeros(3)}
        for _ in range(3):
            result = self.adapter._update_force_recenter(state=state, spec=self.spec,
                metrics=ContactControlRegression.metrics(5., 0.))
            self.assertFalse(result['updated'])
        result = self.adapter._update_force_recenter(state=state, spec=self.spec,
            metrics=ContactControlRegression.metrics(5., 0.))
        self.assertTrue(result['updated'])
        self.assertAlmostEqual(np.linalg.norm(result['motion_delta_world']), 0.0001)
        for _ in range(3):
            self.adapter._update_force_recenter(state=state, spec=self.spec,
                metrics=ContactControlRegression.metrics(5., 0.))
        self.assertEqual(state['force_recenter_step'], 0.0001)

    def test_actual_action_cancels_same_tick_and_caches_contact_only_for_that_step(self):
        adapter = self.adapter
        spec = {**self.spec, 'require_close_pose_gate': True, 'close_until_contact': True}
        pose = {'position': np.array([0.00035, 0., 0.]), 'orientation': np.array([1., 0., 0., 0.])}
        task = SimpleNamespace(step_counter=100, phase_step_counter=100, phase_index=4, phase_entry_step=0,
            phase='close', robots={'right': SimpleNamespace(config=SimpleNamespace(gripper_close_openness=0.08))})
        key = (id(task), 4, 0, 'right', 'ur5e_close_gripper')
        state = self.state
        state.update(close_arm_frozen=True, close_started_step=0, ready_steps=24,
                     contact_control_active=True, contact_capture_closure=0.66)
        adapter._close_gate_state[key] = state
        adapter._target_pose = lambda **kw: copy.deepcopy(pose)
        adapter._current_robot_pose = lambda **kw: copy.deepcopy(pose)
        adapter._current_tcp_pose = lambda **kw: kw['current_pose']
        adapter._current_arm_q = lambda *a: np.zeros(6)
        adapter._command_reference_q = lambda **kw: kw['current_q']
        adapter._ik_target_pose = lambda **kw: kw['target_pose']
        adapter._solve_ik = lambda **kw: np.array([kw['target_pose']['position'][0], 0., 0., 0., 0., 0.])
        adapter._ik_branch_jump_detected = lambda **kw: False
        adapter._current_gripper_q = lambda **kw: 0.66
        adapter._gripper_open_closed_q = lambda **kw: (0., 0.8)
        remembered = []
        adapter._remember_arm_command = lambda task, robot, q: remembered.append(np.asarray(q).copy())
        adapter._continuous_command_q = lambda **kw: kw['command_q']
        adapter._limit_command_to_measured_state = lambda **kw: kw['command_q']
        observations = []
        def observe(**kw):
            observations.append(task.step_counter)
            return True, {'contact_metrics': ContactControlRegression.metrics(3., 5.)}
        adapter._grasp_contact_ready = observe
        for step in (100, 101):
            task.step_counter = step
            action = adapter.act(task=task, robot_name='right', phase_spec={}, skill_spec=spec,
                tracked_robots={}, tracked_objects={})
            self.assertAlmostEqual(action['arm_joint_controller'][0][0], 0.00005)
            self.assertAlmostEqual(state['hold_q'][0], 0.00005)
            self.assertAlmostEqual(remembered[-1][0], 0.00005)
        self.assertEqual(observations, [100, 101])
        self.assertEqual(state['close_contact_stable_steps'], 2)
        self.assertEqual(state['force_recenter_travel'], 0.0002)


class ForceMarginRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        phase = {'name': 'close', 'local_skill': {'name': 'ur5e_close_gripper'}}
        apply_policy([phase])
        self.spec = {**phase['local_skill'], 'close_gate_recenter_single_finger_contact': True}
        self.state = {'close_tcp_position_world': np.zeros(3)}

    def update(self, left=1.1, right=8., valid=True):
        return self.adapter._update_force_recenter(state=self.state, spec=self.spec,
            metrics=ContactControlRegression.metrics(left, right, valid))

    def begin_balance(self):
        for _ in range(3):
            self.assertFalse(self.update()['updated'])
        result = self.update()
        self.assertTrue(result['updated'])
        self.assertEqual(result['motion_mode'], 'force_margin')
        np.testing.assert_allclose(result['motion_delta_world'], [0., -0.000025, 0.])
        return result

    def stop(self, left, right, valid=True):
        return self.adapter._stop_recenter_on_bilateral(state=self.state, spec=self.spec,
            metrics=ContactControlRegression.metrics(left, right, valid),
            current_position=[0., -0.00001, 0.])

    def test_margin_motion_requires_confirmation_and_real_margin_to_stop(self):
        self.begin_balance()
        self.assertIsNone(self.stop(1.1, 8.))
        self.assertIn('force_recenter_pending', self.state)
        event = self.stop(2.1, 5.)
        self.assertEqual(event['stop_cause'], 'force_margin_reached')
        self.assertAlmostEqual(self.state['force_recenter_travel'], 0.000025)
        np.testing.assert_allclose(self.state['recenter_offset_world'], [0., -0.00001, 0.])
        self.assertNotIn('close_contact_stable_steps', self.state)
        self.assertEqual(self.spec['close_force_validation_steps'], 96)
        self.assertEqual(self.spec['close_force_min_stable'], 1.0)

    def test_load_side_reversal_stops_even_if_former_strong_side_loses_contact(self):
        self.begin_balance()
        self.assertIsNone(self.stop(5., 0., valid=False))
        event = self.stop(5., 0.)
        self.assertEqual(event['stop_cause'], 'force_balance_side_reversed')
        self.assertNotIn('force_recenter_pending', self.state)

    def test_confirmation_restarts_after_margin_recovers_or_mode_changes(self):
        for _ in range(3):
            self.update()
        self.assertEqual(self.update(2.5, 5.)['reason'], 'bilateral')
        self.assertFalse(self.update()['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)
        self.update()
        self.update()
        self.assertFalse(self.update(0., 8.)['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)
        self.assertEqual(self.state['force_recenter_confirmation_mode'], 'single_contact')
        self.assertFalse(self.update(8., 0.)['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)

    def test_overload_invalid_probe_and_absent_config_never_start_balance(self):
        for _ in range(3):
            self.update()
        self.assertEqual(self.update(1.1, 13.)['reason'], 'unload_before_recenter')
        self.assertFalse(self.update()['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)
        self.assertEqual(self.update(valid=False)['reason'], 'invalid_probe')
        self.assertEqual(self.state['force_single_steps'], 0)
        self.spec.pop('close_force_balance_min')
        for _ in range(10):
            self.assertEqual(self.update()['reason'], 'bilateral')
        self.assertNotIn('force_recenter_pending', self.state)

    def test_single_contact_overload_preserves_previous_approach_timing(self):
        for _ in range(3):
            self.assertFalse(self.update(0., 8.)['updated'])
        self.assertEqual(self.update(0., 13.)['reason'], 'unload_before_recenter')
        self.assertNotIn('force_recenter_pending', self.state)
        self.assertEqual(self.state['force_single_steps'], 3)
        result = self.update(0., 8.)
        self.assertTrue(result['updated'])
        self.assertEqual(result['motion_mode'], 'single_contact')


class PhysicalRegistrationRegression(unittest.TestCase):
    def setUp(self):
        from toolkits.factory_dual_franka_assembly import planner_primitives as poses
        methods = extracted_definitions('internutopia_extension/tasks/factory_dual_franka_assembly_task.py',
                                        ['_attach_object', '_attachment_relative_pose_from_source'])
        for name in ('relative_pose', 'compose_pose', 'normalize_quat', 'quat_rotate'):
            methods['_attach_object'].__globals__[name] = getattr(poses, name)
        self.task = type('RegistrationTask', (), methods)()
        t = self.task
        t._resolve_object = lambda name: SimpleNamespace(get_pose=lambda: ([.4, .2, 1.], [1., 0., 0., 0.]))
        t._get_robot_task_pose = lambda name: ([.1, .2, 1.2], [1., 0., 0., 0.])
        t._current_relative_pose = lambda *a: (np.array([.3, 0., -.2]), np.array([1., 0., 0., 0.]))
        t._attachment_mode = lambda spec: 'pure_physical_grasp'
        t._JOINT_ATTACHMENT_MODES = set()
        t._gripper_contact_metrics = lambda *a, **kw: {'contact_ready': True}
        t._strict_physical_grasp_contact = lambda *a, **kw: {'physical_contact_ready': True}
        t._attachments = {}; t._locked_collision_states = {}; t._locked_targets = {}; t._frozen_lock_poses = {}
        t.step_counter = 96
        self.writes = []
        t._set_object_pose = lambda *a: self.writes.append(('pose', a))
        t._set_object_collision = lambda *a: self.writes.append(('collision', a))
        t._filter_robot_gripper_from_other_parts = lambda *a, **kw: self.writes.append(('filter', a))
        self.spec = {'continuous_physics': True, 'attachment_relative_pose_source': 'current',
                     'force_enable_collision_on_attach': True, 'require_force_contact': True}

    def test_continuous_registration_records_current_pose_without_physics_writes(self):
        self.task._attach_object('beam', 'right', phase_spec={'name': 'close'}, attach_spec=self.spec)
        self.assertEqual(self.writes, [])
        state = self.task._attachments['beam']
        np.testing.assert_allclose(state['position'], [.3, 0., -.2])
        np.testing.assert_allclose(state['attach_world_position'], [.4, .2, 1.])
        self.assertEqual(state['attach_step'], 96)
        self.assertIsNone(state['joint_path'])

    def test_continuous_registration_cannot_snap_to_an_authored_pose(self):
        spec = {**self.spec, 'snap_object_world_offset_on_attach': [0., 0., .1],
                'attachment_local_position': [0., 0., 0.]}
        self.task._attach_object('beam', 'right', attach_spec=spec)
        self.assertEqual(self.writes, [])
        np.testing.assert_allclose(self.task._attachments['beam']['attach_world_position'], [.4, .2, 1.])

    def test_existing_noncontinuous_mode_retains_pose_and_collision_behavior(self):
        self.task._attach_object('beam', 'right', attach_spec={**self.spec, 'continuous_physics': False})
        self.assertEqual([w[0] for w in self.writes], ['pose', 'collision', 'filter'])


class GraspBoundaryRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        self.q = np.array([1.84, -1.13, 1.69, -2.13, -1.571, -1.29])
        self.pose = {'position': np.array([.3, -.2, 1.2]), 'orientation': np.array([1., 0., 0., 0.])}
        self.task = SimpleNamespace(phase_index=5, phase_entry_step=1957, step_counter=1958,
            phase='base_6_grip_settle', _attachments={'beam': {'robot_name': 'right'}},
            _ur5e_contact_hold_commands={'right': {'object': 'beam', 'arm_q': self.q.copy(),
                'pose': copy.deepcopy(self.pose), 'gripper_openness': .169}})
        self.task._physical_hold_valid = lambda *a, **kw: True
        self.spec = {'name': 'ur5e_retreat_vertical', 'object': 'beam', 'requires_held_object': True,
            'hold_previous_grasp_command': True, 'limit_command_to_measured_state': True,
            'max_command_tracking_error': .18, 'max_wrist_command_tracking_error': .12}
        self.adapter._current_arm_q = lambda *a: self.q + [0, 0, 0, 0, -.0051, 0]
        self.adapter._current_robot_pose = lambda **kw: {**self.pose, 'position': self.pose['position'] + [.00049, .00006, -.00032]}
        self.adapter._solve_ik = lambda **kw: self.fail('zero-offset hold must not solve IK')
        self.adapter._failure_or_hold = lambda *a, **kw: {'failure': a[3]}

    def test_hold_keeps_published_command_despite_nonzero_tracking_error(self):
        action = self.adapter.act(task=self.task, robot_name='right', phase_spec={}, skill_spec=self.spec,
                                  tracked_objects={}, tracked_robots={})
        np.testing.assert_array_equal(action['arm_joint_controller'][0], self.q)
        self.assertEqual(action['gripper_controller'], [.169])
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['pose']['position'], self.pose['position'])

    def test_hold_rejects_missing_contact_or_excessive_tracking_error(self):
        self.task._physical_hold_valid = lambda *a, **kw: False
        action = self.adapter._hold_previous_grasp_command(task=self.task, robot_name='right', spec=self.spec, tracked_robots={})
        self.assertEqual(action['failure'], 'held_object_not_grasped')
        self.task._physical_hold_valid = lambda *a, **kw: True
        self.adapter._current_arm_q = lambda *a: self.q + .3
        action = self.adapter._hold_previous_grasp_command(task=self.task, robot_name='right', spec=self.spec, tracked_robots={})
        self.assertEqual(action['failure'], 'grasp_command_tracking_limit')

    def test_loss_invalidates_only_affected_completion_and_restarts_sample_window(self):
        methods = extracted_definitions('internutopia_extension/tasks/factory_dual_franka_assembly_task.py',
                                        ['_invalidate_continuous_grasp'])
        task = self.task
        task._local_skill_completions = {(5,1957,'right','ur5e_close_gripper'): {},
            (5,1957,'left','ur5e_close_gripper'): {}, (4,800,'right','ur5e_close_gripper'): {}}
        methods['_invalidate_continuous_grasp'](task, 'beam', {'robot_name': 'right', 'attach_spec': {'continuous_physics': True}})
        self.assertNotIn((5,1957,'right','ur5e_close_gripper'), task._local_skill_completions)
        self.assertEqual(len(task._local_skill_completions), 2)
        self.assertNotIn('right', task._ur5e_contact_hold_commands)
        adapter = self.adapter
        adapter._current_gripper_q = lambda **kw: .66
        adapter._gripper_open_closed_q = lambda **kw: (0., .8)
        metrics = ContactControlRegression.metrics(3., 5.)
        adapter._grasp_contact_ready = lambda **kw: (True, {'contact_metrics': metrics})
        task.robots = {'right': SimpleNamespace(config=SimpleNamespace(gripper_close_openness=.08))}
        state = {'close_contact_stable_steps': 97, 'contact_stability_pose': (np.ones(3), np.array([1.,0,0,0]))}
        ready, detail = adapter._close_until_contact_ready(state=state, task=task, robot_name='right',
            spec={'object':'beam', 'continuous_physics':True, 'close_force_regulation':True, 'close_force_validation_steps':96},
            tracked_objects={}, close_elapsed_steps=1000, gripper_openness=.169)
        self.assertFalse(ready)
        self.assertEqual(detail['stable_steps'], 1)


class GraspDriveMotionRegression(unittest.TestCase):
    def setUp(self):
        GraspBoundaryRegression.setUp(self)
        self.spec.pop('hold_previous_grasp_command')
        self.spec.update(inherit_grasp_drive_target=True, relative_to_current_tcp=True,
            lock_target_position=True, offset=[0., 0., .005],
            cartesian_position_step=.0002, grasp_drive_position_lookahead=.001)
        self.ik_calls = []
        def solve(**kw):
            self.ik_calls.append(copy.deepcopy(kw['target_pose']))
            return np.asarray(kw['warm_start']) + [0, .0001, 0, 0, 0, 0]
        self.adapter._solve_ik = solve
        self.adapter._maybe_mark_complete = lambda **kw: None

    def act(self):
        return self.adapter.act(task=self.task, robot_name='right', phase_spec={},
            skill_spec=self.spec, tracked_objects={}, tracked_robots={})

    def test_first_and_second_lift_commands_preserve_loaded_lateral_pose_and_orientation(self):
        self.act()
        self.act()
        for i, pose in enumerate(self.ik_calls):
            np.testing.assert_allclose(pose['position'], self.pose['position'] + [0, 0, .0002*(i+1)], atol=1e-12)
            np.testing.assert_array_equal(pose['orientation'], self.pose['orientation'])
        locked = next(iter(self.adapter._phase_locks.values()))
        np.testing.assert_allclose(locked['position'], self.pose['position'] + [0, 0, .005])

    def test_excessive_cartesian_lag_pauses_without_rebasing_or_advancing_cache(self):
        saved = copy.deepcopy(self.task._ur5e_contact_hold_commands['right'])
        self.adapter._current_robot_pose = lambda **kw: {**self.pose, 'position': self.pose['position'] - [0,0,.002]}
        action = self.act()
        self.assertEqual(self.ik_calls, [])
        np.testing.assert_array_equal(action['arm_joint_controller'][0], saved['arm_q'])
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['pose']['position'], saved['pose']['position'])

    def test_failed_joint_step_does_not_claim_unsent_cartesian_progress(self):
        self.spec['max_command_joint_step'] = .00001
        self.adapter._local_grasp_ik = lambda **kw: None
        action = self.act()
        self.assertEqual(action['failure'], 'grasp_local_ik_failed')
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['pose']['position'], self.pose['position'])

    def test_branch_recovery_backtracks_from_commanded_pose_without_measured_rebase(self):
        self.adapter._solve_ik = lambda **kw: np.asarray(kw['warm_start']) + [1.,0,0,0,0,0]
        candidates = []
        def local(**kw):
            candidates.append(copy.deepcopy(kw))
            return None if len(candidates)==1 else kw['reference_q'] + [0,.0001,0,0,0,0]
        self.adapter._local_grasp_ik = local
        action = self.act()
        self.assertNotIn('failure', action)
        self.assertEqual(len(candidates), 2)
        np.testing.assert_allclose(candidates[1]['target_pose']['position'], self.pose['position'] + [0,0,.0001])
        np.testing.assert_array_equal(candidates[1]['reference_q'], self.q)
        np.testing.assert_allclose(self.task._ur5e_contact_hold_commands['right']['pose']['position'], self.pose['position'] + [0,0,.0001])


class TransportContinuityRegression(unittest.TestCase):
    def setUp(self):
        GraspDriveMotionRegression.setUp(self)
        self.spec.update(grasp_drive_orientation_lookahead=.02, grasp_drive_joint_lookahead=.04,
                         max_command_tracking_error=.24, max_wrist_command_tracking_error=.24)

    def test_recorded_transport_reset_keeps_previous_command_instead_of_reversing(self):
        # v143 step 3740: 0.137 rad measured lag used to reset the reference,
        # producing a +0.06 rad reversal of the wrist command.
        previous=np.array([1.819508929080763,-1.172265653221506,1.8394005956572228,
                           -2.9913534560759993,-2.1165615611332402,2.709287770036968])
        measured=np.array([1.8286073207855225,-1.1780364513397217,1.826805830001831,
                           -2.9161903858184814,-2.139557123184204,2.8465797901153564])
        self.task._ur5e_contact_hold_commands['right']['arm_q']=previous.copy()
        self.adapter._current_arm_q=lambda *a: measured.copy()
        self.adapter._solve_ik=lambda **kw: self.fail('lagging transport must wait before solving')
        action=GraspDriveMotionRegression.act(self)
        np.testing.assert_array_equal(action['arm_joint_controller'][0],previous)
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['arm_q'],previous)

    def test_orientation_lookahead_pauses_without_rebasing(self):
        self.adapter._current_robot_pose=lambda **kw: {**self.pose,
            'orientation':np.array([np.cos(.03/2),0.,0.,np.sin(.03/2)])}
        action=GraspDriveMotionRegression.act(self)
        self.assertEqual(self.ik_calls,[])
        np.testing.assert_array_equal(action['arm_joint_controller'][0],self.q)

    def test_absolute_transport_updates_reference_used_by_next_relative_hold(self):
        self.spec.update(relative_to_current_tcp=False, name='ur5e_move_part_to_staging',
                         target_object_position=[.31,-.2,1.1],derive_tcp_orientation_from_target_object=True)
        self.adapter._target_object_pose=lambda **kw:{'position':np.array([.31,-.2,1.1]),
                                                      'orientation':np.array([1.,0,0,0])}
        self.adapter._object_pose=lambda **kw:{'position':np.array([.3,-.2,1.1]),
                                               'orientation':np.array([1.,0,0,0])}
        self.adapter._object_tcp_relative_pose=lambda **kw:(np.array([0,0,-.1]),np.array([1.,0,0,0]))
        moving=GraspDriveMotionRegression.act(self)
        np.testing.assert_allclose(self.ik_calls[-1]['position'],[.3002,-.2,1.2])
        self.task.phase_index+=1
        self.task.phase_entry_step+=1
        self.spec.update(relative_to_current_tcp=True,hold_previous_grasp_command=True,offset=[0,0,0])
        holding=GraspDriveMotionRegression.act(self)
        np.testing.assert_array_equal(holding['arm_joint_controller'][0],moving['arm_joint_controller'][0])

    def test_candidate_joint_lead_cannot_advance_the_cache(self):
        self.spec['max_command_joint_step']=.008
        self.adapter._current_arm_q=lambda *a:self.q+[0,0,0,0,.038,0]
        self.adapter._solve_ik=lambda **kw:self.q+[0,0,0,0,-.005,0]
        action=GraspDriveMotionRegression.act(self)
        np.testing.assert_array_equal(action['arm_joint_controller'][0],self.q)
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['pose']['position'],self.pose['position'])

    def test_legacy_handoff_caches_published_joint_fk_and_tool_offset(self):
        published=self.q+np.array([.001,0,0,0,0,0])
        observed=[]
        def fk(frame,q):
            observed.append(q.copy())
            return np.array([.15,-.1,.6]),np.eye(3)
        raw=SimpleNamespace(compute_forward_kinematics=fk)
        wrapper=SimpleNamespace(get_kinematics_solver=lambda:raw,get_end_effector_frame=lambda:'tool0')
        self.task.robots={'right':SimpleNamespace(controllers={'arm_ik_controller':
            SimpleNamespace(_kinematics_solver=wrapper,_robot_scale=2.)})}
        spec={**self.spec,'grasp_tcp_offset':[0.,0.,.01]}
        self.assertTrue(self.adapter._record_published_grasp_command(task=self.task,robot_name='right',
            spec=spec,command_q=published,gripper_openness=.17))
        saved=self.task._ur5e_contact_hold_commands['right']
        np.testing.assert_array_equal(observed[0],published)
        np.testing.assert_allclose(saved['pose']['position'],[.3,-.2,1.21])
        self.adapter._current_arm_q=lambda *a:published.copy()
        action=self.adapter._hold_previous_grasp_command(task=self.task,robot_name='right',spec=spec,tracked_robots={})
        np.testing.assert_array_equal(action['arm_joint_controller'][0],published)
        self.assertEqual(action['gripper_controller'],[.17])

    def test_policy_preserves_insertion_alignment_and_completion_requirements(self):
        phases=[{'name':name,'local_skill':{'name':'ur5e_move_part_to_staging','robot':'right',
            'requires_held_object':True,'require_target_object_pose_convergence':True,
            'require_target_object_static':True,'hold_for_target_object_settle':True},'timeout_steps':4800}
            for name in ['base_6_assembly_clearance','base_6_place','assemble_00_part_3_insert_00']]
        apply_policy(phases)
        for p in phases[:2]:self.assertTrue(p['local_skill']['inherit_grasp_drive_target'])
        self.assertNotIn('inherit_grasp_drive_target',phases[2]['local_skill'])
        for p in phases:
            self.assertTrue(p['local_skill']['require_target_object_pose_convergence'])
            self.assertTrue(p['local_skill']['require_target_object_static'])
            self.assertTrue(p['local_skill']['track_published_grasp_command'])


class SmoothTransportRegression(unittest.TestCase):
    def setUp(self):
        GraspDriveMotionRegression.setUp(self)
        self.spec.update(grasp_drive_smooth_transport=True, grasp_drive_position_lookahead=.002,
            grasp_drive_orientation_lookahead=.02, grasp_drive_joint_lookahead=.04,
            grasp_drive_slowdown_start=.55, grasp_drive_slowdown_stop=.95,
            grasp_drive_linear_speed=.04, grasp_drive_angular_speed=.2,
            grasp_drive_joint_speed=.5, grasp_drive_joint_acceleration=4.,
            max_command_joint_step=.008, max_joint_step=.008)
        self.task._contact_physics_dt=lambda:1/240
        self.adapter._current_arm_q=lambda *a:self.q.copy()
        self.adapter._current_robot_pose=lambda **kw:copy.deepcopy(self.pose)
        self.adapter._grasp_command_fk_pose=lambda **kw:{'position':self.pose['position'] +
            .1*(np.asarray(kw['command_q'])-self.q)[:3], 'orientation':self.pose['orientation'].copy()}
        self.adapter._solve_ik=lambda **kw: self.q + np.r_[(kw['target_pose']['position']-self.pose['position'])/.1,[0,0,0]]

    def test_binary_transport_requests_cannot_publish_velocity_jumps(self):
        q=np.zeros(6);v=np.full(6,.3);dt=1/240
        for step in range(120):
            previous=v.copy()
            target=q+(.001315 if step%4 else 0.)
            command,v=self.adapter._rate_limited_grasp_command(reference_q=q,target_q=target,
                previous_velocity=v,dt=dt,speed=.5,acceleration=4.)
            self.assertLessEqual(float(np.max(abs(v-previous)))/dt,4.+1e-10)
            self.assertLessEqual(float(np.max(abs(v))),.5)
            if step==0:self.assertGreater(float(v[0]),.28)
            np.testing.assert_allclose((command-q)/dt,v)
            q=command

    def test_tracking_governor_continuously_reduces_speed_before_boundary(self):
        scales=[self.adapter._grasp_tracking_scale(position_lead=p,orientation_lead=0.,
            joint_lead=0.,spec=self.spec)[0] for p in [.0008,.0013,.0017,.0019,.0021]]
        self.assertEqual(scales[0],1.)
        self.assertTrue(scales[0]>scales[1]>scales[2]>scales[3])
        self.assertEqual(scales[-1],0.)

    def test_actual_published_fk_is_cached_after_acceleration_limiting(self):
        action=GraspDriveMotionRegression.act(self)
        q=np.array(action['arm_joint_controller'][0])
        self.assertAlmostEqual(float(np.max(abs(q-self.q))),4./240**2)
        saved=self.task._ur5e_contact_hold_commands['right']
        np.testing.assert_allclose(saved['pose']['position'],self.pose['position']+.1*(q-self.q)[:3])
        self.assertEqual(saved['pose_source'],'published_joint_fk')
        self.assertEqual(action['gripper_controller'],[.169])

    def test_adjacent_phase_keeps_velocity_and_decelerates_on_reversal(self):
        saved=self.task._ur5e_contact_hold_commands['right']
        saved.update(smooth_step=self.task.step_counter-1,velocity_q=np.array([0,0,.1,0,0,0]))
        self.spec['offset']=[0,0,-.005]
        action=GraspDriveMotionRegression.act(self)
        self.assertGreater(action['arm_joint_controller'][0][2],self.q[2])
        self.assertAlmostEqual(saved['velocity_q'][2],.1-4/240)

    def test_tracking_boundary_rejects_without_advancing_cache(self):
        self.adapter._grasp_command_fk_pose=lambda **kw:{**self.pose,'position':self.pose['position']+[.003,0,0]}
        before=copy.deepcopy(self.task._ur5e_contact_hold_commands['right'])
        action=GraspDriveMotionRegression.act(self)
        self.assertEqual(action['failure'],'grasp_smooth_tracking_boundary')
        np.testing.assert_array_equal(self.task._ur5e_contact_hold_commands['right']['arm_q'],before['arm_q'])

    def test_completion_waits_until_command_velocity_is_small(self):
        self.spec['offset']=[0,0,0]
        saved=self.task._ur5e_contact_hold_commands['right']
        saved.update(smooth_step=self.task.step_counter-1,velocity_q=np.array([0,0,.1,0,0,0]))
        self.adapter._maybe_mark_complete=lambda **kw:self.fail('moving handoff must not complete')
        action=GraspDriveMotionRegression.act(self)
        self.assertNotIn('failure',action)

    def test_stopped_deadband_defers_to_original_physical_completion_gates(self):
        self.spec.update(offset=[0,0,.000095],require_target_object_pose_convergence=True)
        self.adapter._solve_ik=lambda **kw:self.q.copy()
        checks=[]
        self.adapter._maybe_mark_complete=lambda **kw:checks.append(kw)
        action=GraspDriveMotionRegression.act(self)
        self.assertNotIn('failure',action)
        self.assertEqual(len(checks),1)
        self.assertTrue(checks[0]['spec']['require_target_object_pose_convergence'])
        np.testing.assert_allclose(checks[0]['target_pose']['position'],self.pose['position']+[0,0,.000095])

    def test_policy_enables_smoothing_only_for_selected_transport(self):
        phases=[{'name':n,'local_skill':{'requires_held_object':True,'relative_to_current_tcp':rel}}
                for n,rel in [('base_6_lift',True),('base_6_assembly_clearance',False),('assemble_00_insert_00',False)]]
        apply_policy(phases)
        self.assertNotIn('grasp_drive_smooth_transport',phases[0]['local_skill'])
        self.assertTrue(phases[1]['local_skill']['grasp_drive_smooth_transport'])
        # v145's stopped request was 4.95 um and 0.124 mrad; those increments
        # must not be accepted as zero by the configured native IK tolerances.
        self.assertLess(phases[1]['local_skill']['ik_position_tolerance'],4.95e-6/10)
        self.assertLess(phases[1]['local_skill']['ik_orientation_tolerance'],.000124/10)
        self.assertNotIn('grasp_drive_smooth_transport',phases[2]['local_skill'])

    def test_failed_contact_snapshot_is_not_hidden_by_same_step_deduplication(self):
        import tempfile
        from unittest.mock import patch
        task=Task()
        task.step_counter=4351;task.phase='base_6_assembly_clearance';task.phase_step_counter=700
        task._attachments={};task._object_prims={};task._object_collision_enabled={'beam':True}
        pose=(np.zeros(3),np.array([1.,0,0,0]))
        task._resolve_object=lambda *a:SimpleNamespace(get_pose=lambda:pose)
        task._get_robot_task_pose=lambda *a:pose
        task._current_relative_pose=lambda *a:pose
        task._get_robot_gripper_opening=lambda *a:.664
        task._current_gripper_command=lambda *a:'contact_hold'
        task.get_current_phase_spec=lambda:{}
        task._object_velocity_metrics=lambda *a:{}
        metrics={'robot':'right','left_finger':{'force':1.53},'right_finger':{'force':9.25}}
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'contacts.jsonl'
            with patch.dict(os.environ,{'BEAM_CONTACT_TRACE_PATH':str(path),'BEAM_SUPPORT_CONTACT_TRACE':'0'}):
                task._write_physical_contact_trace('beam',metrics,{}, {'physical_contact_ready':True})
                metrics['left_finger']['force']=0.
                task._write_physical_contact_trace('beam',metrics,{}, {'physical_contact_ready':False})
                task._write_physical_contact_trace('beam',metrics,{}, {'physical_contact_ready':False},
                    force_write=True,validation_stage='state_sync')
            rows=[json.loads(l) for l in path.read_text().splitlines()]
        self.assertEqual(len(rows),2)
        self.assertEqual(rows[0]['contact_metrics']['left_finger']['force'],1.53)
        self.assertEqual(rows[1]['contact_metrics']['left_finger']['force'],0.)
        self.assertTrue(rows[1]['failure_snapshot'])
        self.assertEqual(rows[1]['validation_stage'],'state_sync')


class ContinuousRobotStateRegression(unittest.TestCase):
    def setUp(self):
        methods = extracted_definitions('internutopia_extension/robots/ur5e.py', [
            '_bounded_revolute_joint_values', '_current_arm_joint_positions_for_normalization',
            '_renormalize_arm_joint_state_if_needed', '_normalize_arm_joint_controller_action',
            '_normalize_arm_control', 'apply_action', 'action_to_dict'])
        names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        methods['apply_action'].__globals__['_UR5E_ARM_JOINT_NAMES'] = names
        self.robot = type('Robot', (), methods)()
        self.robot.config = SimpleNamespace(name='right', preserve_continuous_arm_joints=True)
        # v146 step 8626: the wrist crosses the legacy pi + .25 threshold.
        self.current = np.array([.865809440612793, -1.1518455743789673, 2.071617841720581,
            -3.3917555809020996, -.9754934906959534, 1.1588830947875977])
        self.command = np.array([.8659598381286736, -1.1486676216897902, 2.0720961476371564,
            -3.397133651215048, -.980940997966121, 1.1594844657350114])
        self.writes = []; self.published = []
        self.robot.articulation = SimpleNamespace(
            get_dof_index=names.index,
            get_joint_positions=lambda **kw:self.current.copy(),
            set_joint_positions=lambda q, **kw:self.writes.append(('position', q.copy())),
            set_joint_velocities=lambda q, **kw:self.writes.append(('velocity', q.copy())),
            apply_action=lambda control:self.published.append(control))
        self.robot.controllers = {'arm_joint_controller':SimpleNamespace(
            action_to_control=lambda action:SimpleNamespace(joint_positions=np.array(action[0]),
                joint_velocities=None, joint_efforts=None, joint_indices=np.arange(6)))}

    def test_loaded_wrist_crossing_preserves_state_and_exact_published_command(self):
        self.robot.apply_action({'arm_joint_controller':[self.command.tolist()]})
        self.assertEqual(self.writes, [])
        np.testing.assert_array_equal(self.published[0].joint_positions, self.command)
        np.testing.assert_array_equal(self.robot.last_action[0]['joint_positions'], self.command)

    def test_empty_terminal_action_cannot_wrap_or_stop_joint_state(self):
        for sign in (-1, 1):
            self.current[3] = sign * (np.pi + .3)
            self.robot.apply_action({})
        self.assertEqual(self.writes, [])
        self.assertEqual(self.published, [])

    def test_continuous_native_ik_control_is_not_rewrapped(self):
        control=SimpleNamespace(joint_positions=self.command.copy())
        self.robot._normalize_arm_control('arm_ik_controller',control)
        np.testing.assert_array_equal(control.joint_positions,self.command)

    def test_legacy_path_reproduces_recorded_one_turn_tracking_discontinuity(self):
        self.robot.config.preserve_continuous_arm_joints=False
        self.robot._renormalize_arm_joint_state_if_needed()
        self.assertEqual([kind for kind, _ in self.writes], ['position','velocity'])
        wrapped=self.writes[0][1]
        self.assertAlmostEqual(wrapped[3]-self.current[3],2*np.pi)
        np.testing.assert_array_equal(self.writes[1][1],np.zeros(6))
        spec={'max_command_tracking_error':.24,'max_wrist_command_tracking_error':.24}
        limited=UR5eAssemblyAtomicSkillAdapter._limit_command_to_measured_state(
            current_q=wrapped,command_q=self.command,spec=spec)
        self.assertAlmostEqual(limited[3]-self.command[3],2*np.pi)
        self.assertLess(np.max(abs(limited-wrapped)),.006)


class LoadedIKRegression(unittest.TestCase):
    @staticmethod
    def forward(q):
        angle=q[5]
        return q[:3].copy(), np.array([[np.cos(angle),-np.sin(angle),0.],
            [np.sin(angle),np.cos(angle),0.], [0.,0.,1.]])

    def test_local_refinement_corrects_translation_and_near_boundary_rotation(self):
        ref=np.array([.3,-.2,1.3,0.,0.,.0009998])
        target=ref[:3]+[0,0,.00025]
        q=UR5eAssemblyAtomicSkillAdapter._bounded_local_pose_ik(forward=self.forward,
            reference_q=ref, target_position=target, target_orientation=np.array([1.,0,0,0]),
            position_tolerance=.000005, orientation_tolerance=.001, joint_radius=.008)
        self.assertIsNotNone(q)
        np.testing.assert_allclose(q[:3],target,atol=.000005)
        self.assertLess(abs(q[5]),.001)
        self.assertLessEqual(np.max(np.abs(q-ref)),.008)

    def test_local_refinement_rejects_solution_outside_command_step_box(self):
        q=UR5eAssemblyAtomicSkillAdapter._bounded_local_pose_ik(forward=self.forward,
            reference_q=np.zeros(6), target_position=np.array([0,0,.03]),
            target_orientation=np.array([1.,0,0,0]), position_tolerance=.000005,
            orientation_tolerance=.001, joint_radius=.008)
        self.assertIsNone(q)

    def test_loaded_motion_does_not_fallback_to_measured_joint_solution(self):
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        calls = []
        def raw_ik(*args, **kwargs):
            calls.append(kwargs)
            return np.ones(6), False
        raw = SimpleNamespace(compute_inverse_kinematics=raw_ik)
        fallback_calls = []
        def fallback(**kw):
            fallback_calls.append(kw)
            return SimpleNamespace(joint_positions=np.full(6, .1)), True
        wrapper = SimpleNamespace(set_robot_base_pose=lambda **kw: None,
            get_kinematics_solver=lambda: raw, get_end_effector_frame=lambda: 'tool0',
            compute_inverse_kinematics=fallback)
        controller = SimpleNamespace(_robot_scale=1., _kinematics_solver=wrapper,
            get_ik_base_world_pose=lambda: (np.zeros(3), np.array([1.,0,0,0])))
        task = SimpleNamespace(robots={'right': SimpleNamespace(controllers={'arm_ik_controller':controller})})
        result = adapter._solve_ik(task=task, robot_name='right',
            target_pose={'position':np.ones(3), 'orientation':np.array([1.,0,0,0])},
            warm_start=np.zeros(6), spec={'require_warm_start_ik':True,
                'ik_position_tolerance':.000005, 'ik_orientation_tolerance':.001})
        self.assertIsNone(result)
        self.assertEqual(fallback_calls, [])
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['position_tolerance'], .000005)
        self.assertEqual(calls[0]['orientation_tolerance'], .001)


class ShallowGraspRegression(unittest.TestCase):
    def test_pickup_offset_rotates_with_part_and_preserves_approach(self):
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import quat_rotate
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        q = np.array([.615241706, .3506414, .3474997, -.61463189])
        q /= np.linalg.norm(q)
        adapter._object_pose = lambda **kw: {'position': np.array([.29, -.26, 1.07]), 'orientation': q}
        spec = {'object': 'beam', 'grasp_relative_position': [-.0174, 0, .175],
                'grasp_relative_orientation': [-.3804, 0, .9248, 0],
                'align_grasp_axis_to_world_down': True, 'grasp_pad_reach': .120,
                'offset': [0, .0116, .1051], 'offset_frame': 'world'}
        args = dict(task=None, robot_name='right', tracked_robots={}, tracked_objects={})
        original = adapter._target_pose(spec=spec, **args)
        shifted = adapter._target_pose(spec={**spec, 'grasp_object_frame_offset': [0, 0, .028]}, **args)
        np.testing.assert_allclose(shifted['position'] - original['position'], quat_rotate(q, [0, 0, .028]), atol=1e-12)
        np.testing.assert_allclose(shifted['orientation'], original['orientation'])
        self.assertIsNone(adapter._target_pose(spec={**spec, 'grasp_object_frame_offset': [0, float('nan'), 0]}, **args))

    def test_offset_only_changes_base_pickup_relation(self):
        phases = [{'name': name, 'local_skill': {'object': obj, **extra}} for name, obj, extra in [
            ('base_6_descend', 'fabrica_beam_6', {'grasp_relative_position': [0, 0, .1]}),
            ('base_6_place', 'fabrica_beam_6', {'requires_held_object': True}),
            ('part_3_descend', 'fabrica_beam_3', {'grasp_relative_position': [0, 0, .1]})]]
        apply_policy(phases)
        self.assertEqual(phases[0]['local_skill']['grasp_object_frame_offset'], [0, 0, .028])
        self.assertNotIn('grasp_object_frame_offset', phases[1]['local_skill'])
        self.assertNotIn('grasp_object_frame_offset', phases[2]['local_skill'])

    def test_failure_expands_all_gripper_and_assembly_contact_pairs(self):
        import tempfile
        from unittest.mock import patch
        task = Task()
        task.step_counter = 7; task.phase = 'place'; task.phase_step_counter = 2
        task._attachments = {}; task._object_collision_enabled = {'beam': True}
        names = ('fabrica_fixture', 'fixture_support', 'factory_tabletop_visual', 'optical_board', 'assembly_support')
        task._object_prims = {n: SimpleNamespace(GetPath=lambda n=n: '/' + n) for n in names}
        pose = (np.zeros(3), np.array([1., 0, 0, 0]))
        task._resolve_object = lambda *a: SimpleNamespace(get_pose=lambda: pose, unwrap=lambda: SimpleNamespace(prim_path='/beam'))
        task._get_robot_task_pose = lambda *a: pose
        task._current_relative_pose = lambda *a: pose
        task._get_robot_gripper_opening = lambda *a: .66
        task._current_gripper_command = lambda *a: 'contact_hold'
        task.get_current_phase_spec = lambda: {}
        task._object_velocity_metrics = lambda *a: {}
        task._robot_rigid_body_by_suffix = lambda r, s: SimpleNamespace(
            unwrap=lambda: SimpleNamespace(prim_path='/robot/' + s), get_pose=lambda: pose)
        task._finger_prim_path = lambda b: b.unwrap().prim_path.lower()
        task._pair_contact_observation = lambda p, q: {'prim_path': p, 'filter_prim_path': q, 'valid': True, 'force': 0}
        metrics = {'robot': 'right', **{s: {'prim_path': '/robot/Robotiq_2F_85/' + n} for s, n in
                   [('left_finger', 'left_inner_finger'), ('right_finger', 'right_inner_finger')]}}
        context = SimpleNamespace(current_time=.1, current_time_step_index=24)
        module = SimpleNamespace(SimulationContext=SimpleNamespace(instance=lambda: context))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'contacts.jsonl'
            with patch.dict(sys.modules, {'isaacsim.core.api': module}), patch.dict(os.environ,
                    {'BEAM_CONTACT_TRACE_PATH': str(path), 'BEAM_SUPPORT_CONTACT_TRACE': '1'}):
                task._write_physical_contact_trace('beam', metrics, {}, {}, force_write=True, validation_stage='state_sync')
            row = json.loads(path.read_text())
        self.assertEqual(len(row['gripper_bodies']), 9)
        self.assertEqual(len(row['support_contacts']), 50)
        self.assertEqual({p['support_object'] for p in row['support_contacts']}, set(names))
        self.assertEqual(row['physics_step_index'], 24)
        self.assertTrue(row['failure_snapshot'])
        self.assertTrue(all('Robotiq_2F_85' in b['path'] for b in row['gripper_bodies']))


class ContinuousRebaseRegression(unittest.TestCase):
    def make_task(self, continuous=True, distance=.049942221447):
        from toolkits.factory_dual_franka_assembly import planner_primitives as primitives
        methods = extracted_definitions('internutopia_extension/tasks/factory_dual_franka_assembly_task.py',
                                        ['_rebase_targets_from_object', '_unlock_object'])
        methods['_rebase_targets_from_object'].__globals__.update({n: getattr(primitives, n) for n in
            ('pose_within_tolerance', 'relative_pose', 'compose_pose', 'normalize_quat')})
        task = type('RebaseTask', (), methods)()
        task.step_counter = 9758; task._rebased_anchors = set()
        task._attachments = {'beam': {'mode': 'pure_physical_grasp', 'robot_name': 'right',
            'attach_spec': {'continuous_physics': continuous}, 'position': [0, 0, .161],
            'orientation': [1, 0, 0, 0], 'attach_step': 1900}}
        task.target_poses = {'anchor': {'position': np.zeros(3), 'orientation': np.array([1., 0, 0, 0])},
                             'other': {'position': np.array([.2, 0, 0]), 'orientation': np.array([1., 0, 0, 0])}}
        task._resolve_object = lambda *a: SimpleNamespace(get_pose=lambda: (np.array([0, 0, distance]), np.array([1., 0, 0, 0])))
        task._as_list = lambda x: x
        task._locked_targets = {}; task._frozen_lock_poses = {}; task._lock_pin_pose = {}
        task._locked_collision_states = {}; task._object_collision_enabled = {'beam': True}
        task.calls = []
        task._set_object_collision = lambda *a: task.calls.append(('collision', a))
        task._set_attachment_gripper_collision_filter = lambda *a, **kw: task.calls.append(('filter', kw)) or ['/finger']
        task._zero_object_velocity = lambda *a: task.calls.append(('zero_velocity', a))
        task._current_relative_pose = lambda *a: ([9, 9, 9], [1, 0, 0, 0])
        task._maybe_write_attach_debug = lambda e: task.calls.append(('event', e))
        return task

    @staticmethod
    def spec():
        return {'object': 'beam', 'anchor_target': 'anchor', 'targets': ['other'],
                'position_tolerance': .05, 'orientation_tolerance': .2, 'preserve_z': True,
                'enable_collision': True, 'filter_gripper_collision': True}

    def test_rebase_keeps_continuous_contacts_even_with_stale_filter_flag(self):
        t = self.make_task(); before = copy.deepcopy(t._attachments)
        t._rebase_targets_from_object(self.spec())
        self.assertEqual(t._rebased_anchors, {'beam'})
        self.assertEqual([k for k, v in t.calls], ['event'])
        self.assertEqual(t._attachments, before)
        np.testing.assert_allclose(t.target_poses['anchor']['position'], [0, 0, 0])

    def test_legacy_rebase_reproduces_filter_exactly_inside_old_failure_boundary(self):
        for distance, triggers in [(.050069836483, False), (.049942221447, True)]:
            t = self.make_task(continuous=False, distance=distance)
            t._rebase_targets_from_object(self.spec())
            self.assertEqual(any(k == 'filter' and v['enabled'] for k, v in t.calls), triggers)

    def test_continuous_unlock_preserves_momentum_and_original_slip_reference(self):
        t = self.make_task(); before = copy.deepcopy(t._attachments)
        t._unlock_object('beam')
        self.assertEqual(t.calls, [])
        self.assertEqual(t._attachments, before)

    def test_companion_and_bookkeeping_holds_keep_captured_drive(self):
        p = {'name': 'base_6_set_down', 'gripper_commands': {'right': 'close'}, 'local_skills': {
            'right': {'name': 'ur5e_hold_part_end', 'robot': 'right', 'object': 'beam'}}}
        q = {'name': 'base_6_clear_support_filters', 'advance': {'conditions': [
            {'type': 'object_attached', 'robot': 'right', 'object': 'beam'}]}}
        apply_policy([p, q])
        for s in [p['local_skills']['right'], q['local_skill']]:
            self.assertTrue(s['requires_held_object'])
            self.assertTrue(s['hold_previous_grasp_command'])
            self.assertEqual(s['gripper_command'], 'contact_hold')
        self.assertEqual(p['gripper_commands']['right'], 'contact_hold')


class LocalCloseIKRegression(unittest.TestCase):
    setUp = SettledCloseRegression.setUp

    def test_alternate_branch_uses_bounded_fk_without_relaxing_guard(self):
        a = self.adapter
        a._target_pose = lambda **kw: copy.deepcopy(self.pose)
        a._current_robot_pose = lambda **kw: copy.deepcopy(self.pose)
        a._current_tcp_pose = lambda **kw: kw['current_pose']
        a._current_arm_q = lambda *args: np.zeros(6)
        a._command_reference_q = lambda **kw: kw['current_q']
        a._ik_target_pose = lambda **kw: kw['target_pose']
        a._solve_ik = lambda **kw: np.ones(6)*3.
        a._continuous_command_q = lambda **kw: kw['command_q']
        a._limit_command_to_measured_state = lambda **kw: kw['command_q']
        a._remember_arm_command = lambda *args: None
        calls = []
        a._local_grasp_ik = lambda **kw: calls.append(kw) or np.zeros(6)
        task = SimpleNamespace(step_counter=0, phase_step_counter=0, phase='close')
        ready, action, detail = a._close_pose_gate_action(phase_key='test', task=task,
            robot_name='right', spec=self.spec, tracked_robots={}, tracked_objects={})
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]['joint_radius'], .04)
        np.testing.assert_allclose(action['arm_joint_controller'], [np.zeros(6)])
        # Failure to find a local solution must keep the jaw open and gate shut.
        a._hold_joint_action = lambda **kw: {}
        a._local_grasp_ik = lambda **kw: None
        ready, action, detail = a._close_pose_gate_action(phase_key='test', task=task,
            robot_name='right', spec=self.spec, tracked_robots={}, tracked_objects={})
        self.assertFalse(ready)
        self.assertEqual(detail['reason'], 'close_gate_ik_failed')
        self.assertEqual(action['gripper_controller'], [1.])


class LoadedForceRegression(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        self.adapter._failure_or_hold = lambda t, r, s, reason, **kw: {'failure': reason}
        self.adapter._current_gripper_q = lambda **kw: .66
        self.adapter._gripper_open_closed_q = lambda **kw: (0., .8)
        self.metrics = ContactControlRegression.metrics(1.3, 1.7)
        self.task = SimpleNamespace(step_counter=10, phase='peel', robots={
            'right': SimpleNamespace(config=SimpleNamespace(gripper_close_openness=.08))},
            _attachments={'beam': {'attach_spec': {'continuous_physics': True, 'hold_force_regulation': True}}},
            _gripper_contact_metrics=lambda *a, **kw: self.metrics)
        self.saved = {'object': 'beam', 'gripper_openness': 1.-.66/.8}
        self.args = dict(task=self.task, robot_name='right', spec={'object': 'beam'},
                         saved=self.saved, command_q=np.array([1, 2, 3, -3.6, 0, 0]))

    def test_loaded_force_recovers_preload_once_per_step_with_original_limits(self):
        from unittest.mock import patch
        with patch.dict(os.environ, {'BEAM_HOLD_TRACE_PATH': ''}):
            a = self.adapter._publish_loaded_action(**self.args)
            q = .8*(1.-a['gripper_controller'][0])
            self.assertAlmostEqual(q-.66, .025/240)
            self.assertEqual(self.adapter._publish_loaded_action(**self.args), a)
            for step in range(11, 50):
                self.task.step_counter = step
                a = self.adapter._publish_loaded_action(**self.args)
                self.assertLessEqual(.8*(1.-a['gripper_controller'][0])-.66, .001+.025/240+1e-12)
        self.assertEqual(a['arm_joint_controller'][0][3], -3.6)

    def test_missing_contact_does_not_seek_or_bypass_hold_failure(self):
        self.metrics['left_finger']['force'] = 0.
        before = copy.deepcopy(self.saved)
        self.assertEqual(self.adapter._publish_loaded_action(**self.args)['failure'], 'loaded_force_contact_missing')
        self.assertEqual(self.saved, before)

    def test_excessive_force_fails_repeated_calls_without_publishing_a_squeeze(self):
        from unittest.mock import patch
        self.metrics = ContactControlRegression.metrics(41., 45.)
        with patch.dict(os.environ, {'BEAM_HOLD_TRACE_PATH': ''}):
            for _ in range(2):
                result = self.adapter._publish_loaded_action(**self.args)
                self.assertEqual(result, {'failure': 'loaded_gripper_force_limit'})

    def test_friction_details_use_count_then_start_and_native_physics_dt(self):
        from unittest.mock import patch
        t = Task(); t._contact_physics_dt = lambda: 1/240
        dts = []
        t._get_contact_probe = lambda *a: SimpleNamespace(
            is_physics_handle_valid=lambda: True,
            get_contact_force_matrix=lambda **kw: np.array([[[3., 0, 0]]]),
            get_contact_force_data=lambda **kw: (np.array([[3.]]), np.zeros((1, 3)),
                np.zeros((1, 3)), np.zeros((1, 1)), np.array([[1]]), np.array([[0]])),
            get_friction_data=lambda **kw: dts.append(kw['dt']) or (np.array([[77., 77., 77.], [0, 1, 2], [0, -1, 0]]),
                np.zeros((3, 3)), np.array([[2]]), np.array([[1]])))
        with patch.dict(os.environ, {'BEAM_SUPPORT_CONTACT_TRACE': '1'}):
            result = t._pair_contact_observation('/finger', '/beam')
        self.assertTrue(result['friction_valid'])
        self.assertEqual(result['friction_force_world'], [0., 0., 2.])
        self.assertEqual(dts, [1/240])


class LaterPickupRegression(unittest.TestCase):
    def test_later_pick_requires_pose_gate_and_bounded_contact_approach(self):
        phases = [
            {'name': 'assemble_00_part_3_descend', 'local_skill': {'name': 'ur5e_descend_to_grasp'}},
            {'name': 'assemble_00_part_3_close_and_attach', 'local_skill': {
                'name': 'ur5e_close_gripper', 'require_close_pose_gate': False}},
            {'name': 'base_6_descend', 'local_skill': {'name': 'ur5e_descend_to_grasp',
                                                      'cartesian_position_step': .004}},
        ]
        apply_policy(phases)
        descend = phases[0]['local_skill']
        self.assertTrue(descend['trace_pregrasp'])
        self.assertTrue(descend['lock_target_orientation'])
        self.assertFalse(descend['use_arm_ik_controller'])
        self.assertEqual(descend['max_command_tracking_error'], .04)
        self.assertEqual(descend['max_command_joint_step'], .004)
        self.assertTrue(phases[1]['local_skill']['require_close_pose_gate'])
        self.assertEqual(phases[2]['local_skill']['cartesian_position_step'], .004)

    def test_pregrasp_motion_is_logged_without_symbolic_attachment(self):
        import tempfile
        from unittest.mock import patch
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        adapter._debug_grasp_enabled = lambda: False
        pose = {'position': np.array([.4, -.2, 1.2]), 'orientation': np.array([0., 1, 0, 0])}
        with tempfile.TemporaryDirectory() as temp:
            trace = Path(temp) / 'motion.jsonl'
            with patch.dict(os.environ, {'BEAM_MOTION_TRACE_PATH': str(trace)}):
                adapter._debug_joint_step(task=SimpleNamespace(step_counter=8, phase='descend'),
                    robot_name='left', skill_name='ur5e_descend_to_grasp', spec={'trace_pregrasp': True},
                    current_pose=pose, target_pose=pose, command_target_pose=pose, ik_target_pose=pose,
                    current_q=np.zeros(6), reference_q=np.zeros(6), target_q=np.ones(6), command_q=np.zeros(6))
            row = json.loads(trace.read_text())
        self.assertEqual(row['robot'], 'left')
        self.assertEqual(row['target_pose']['position'], [.4, -.2, 1.2])


if __name__ == '__main__':
    unittest.main()
