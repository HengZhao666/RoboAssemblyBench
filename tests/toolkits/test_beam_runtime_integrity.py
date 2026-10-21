"""Admission, measured feedback and final-acceptance regressions; no simulator."""
from __future__ import annotations

import ast
import copy
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from toolkits.factory_dual_franka_assembly.beam_support_state import BeamSupportState
from toolkits.factory_dual_franka_assembly.local_skills import LocalSkillExecutor
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import (
    UR5eAssemblyAtomicSkillAdapter as Adapter,
)

ROOT = Path(__file__).resolve().parents[2]


class GripperRateLimits(unittest.TestCase):
    def control(self, *, previous, measured, force, dt, reversed_span=False):
        state = {'contact_control_active': True, 'contact_capture_closure': .66,
                 'contact_command_closure': previous}
        fingers = {s: {'force': force, 'force_probe_valid': True,
                      'force_observation': {'physics_dt': dt}}
                   for s in ('left_finger', 'right_finger')}
        open_q, closed_q = (.8, 0.) if reversed_span else (0., .8)
        q = open_q - measured if reversed_span else measured
        result = Adapter._contact_close_control(state=state,
            spec={'close_force_joint_rate': .0024, 'close_force_max_command_lead': .001},
            metrics=fingers, gripper_q=q, open_q=open_q, closed_q=closed_q,
            requested_openness=.08, min_openness=.08)
        return state['contact_command_closure'], result

    def test_ordinary_unload_rate_survives_tracking_lag_and_joint_sign(self):
        for dt in [1/120, 1/240, 1/480]:
            for measured in [.64, .66, .68]:
                for reverse in [False, True]:
                    with self.subTest(dt=dt, measured=measured, reverse=reverse):
                        q, detail = self.control(previous=.66, measured=measured,
                            force=13., dt=dt, reversed_span=reverse)
                        self.assertEqual(detail['mode'], 'unload')
                        self.assertAlmostEqual(q, .66-.0024*dt)

    def test_preload_does_not_jump_open_when_measurement_lags(self):
        for dt in [1/120, 1/240, 1/480]:
            for measured in [.64, .6595, .68]:
                q, detail = self.control(previous=.66, measured=measured, force=3., dt=dt)
                self.assertGreaterEqual(q, .66)
                self.assertLessEqual(q-.66, .0024*dt+1e-12)
                self.assertEqual(detail['mode'], 'preload')

    def test_emergency_overload_retains_separate_unwind(self):
        q, detail = self.control(previous=.66, measured=.64, force=80., dt=1/240)
        self.assertEqual(detail['mode'], 'force_limit_unload')
        self.assertLess(q, .64)
        self.assertFalse(detail['force_within_limit'])


class SharedAdmission(unittest.TestCase):
    def setUp(self):
        self.task = BeamSupportState()
        self.task._attachments = {}
        self.task._beam_shared_grasps = {}
        self.executor = LocalSkillExecutor()
        self.spec = {'held_object': 'post', 'require_success': True}
        self.primary = {'robot_name': 'left', 'mode': 'pure_physical_grasp',
                        'attach_spec': {'allow_shared_physical_grasp': True}, 'attach_step': 10}
        self.secondary = {**copy.deepcopy(self.primary), 'robot_name': 'right', 'attach_step': 20}
        self.task._store_physical_grasp('post', self.primary)
        self.task._store_physical_grasp('post', self.secondary)

    def guard(self, robot):
        return self.executor._held_object_failure_diagnostics(
            spec=self.spec, robot_name=robot, task=self.task,
            tracked_objects={'post': {'attached_to': 'left', 'grasped_by': 'left'}},
        )

    def test_secondary_admitted_during_release_delay_without_recapturing(self):
        for _ in range(12):
            self.assertIsNone(self.guard('right'))
            self.assertIs(self.task._attachment_for('post', 'right'), self.secondary)
        self.assertEqual(self.secondary['attach_step'], 20)

    def test_release_promotion_keeps_membership_and_rejects_stale_tracked_owner(self):
        self.task._remove_physical_grasp('post', 'left')
        self.assertIsNone(self.guard('right'))
        self.assertIsNotNone(self.guard('left'))
        self.assertIsNotNone(self.guard('third'))


class MeasuredFeedback(unittest.TestCase):
    def setUp(self):
        self.adapter = Adapter({})
        self.names = tuple(f'joint{i}' for i in range(6))
        self.controller = NS(joint_names=self.names, get_joint_subset=lambda: None)
        self.robot = NS(controllers={'arm_joint_controller': self.controller}, articulation=None)
        self.task = NS(robots={'right': self.robot})

    def test_cached_and_observed_commands_cannot_become_measurements(self):
        self.adapter._last_arm_command_q['right'] = np.arange(6.)
        self.robot.get_obs = lambda: {'joint_action': [{'joint_positions': np.arange(6.)}]}
        self.assertIsNone(self.adapter._current_arm_q(self.task, 'right'))
        self.assertFalse(self.adapter._arm_measurement_status['right']['valid'])
        np.testing.assert_array_equal(self.adapter._command_reference_q(
            task=self.task, robot_name='right', current_q=None, spec={}), np.arange(6.))

    def test_named_joint_order_survives_interleaved_gripper_dofs(self):
        names = ['grip', *reversed(self.names)]
        values = np.arange(7.)
        self.robot.articulation = NS(dof_names=names, get_dof_index=lambda n: names.index(n),
            get_joint_positions=lambda **kw: values[kw['joint_indices']] if kw else values)
        np.testing.assert_array_equal(self.adapter._current_arm_q(self.task, 'right'), [6, 5, 4, 3, 2, 1])

    def test_mixed_nameless_or_incomplete_named_buffers_are_unavailable(self):
        def bad_index(_): raise RuntimeError('no native name map')
        for names in [[], ['grip', *self.names[:-1]]]:
            self.robot.articulation = NS(dof_names=names, get_dof_index=bad_index,
                get_joint_positions=lambda **kw: np.arange(7.))
            self.assertIsNone(self.adapter._current_arm_q(self.task, 'right'))

    def test_subset_construction_failure_falls_back_to_actual_articulation(self):
        def bad_subset(): raise RuntimeError('subset unavailable')
        self.controller.get_joint_subset = bad_subset
        self.robot.articulation = NS(dof_names=self.names, get_dof_index=lambda n: self.names.index(n),
            get_joint_positions=lambda **kw: np.arange(6.))
        np.testing.assert_array_equal(self.adapter._current_arm_q(self.task, 'right'), np.arange(6.))

    def test_continuous_action_fails_before_motion_when_measurement_is_missing(self):
        self.adapter._last_arm_command_q['right'] = np.arange(6.)
        action = self.adapter.act(task=self.task, robot_name='right', phase_spec={},
            skill_spec={'name': 'ur5e_move_above_part', 'continuous_physics': True, 'require_success': True},
            tracked_robots={}, tracked_objects={})
        self.assertTrue(action['__local_skill_failure__'])
        self.assertEqual(action['reason'], 'current_joint_state_unavailable')


class JointFinalWindow(unittest.TestCase):
    def setUp(self):
        self.task = BeamSupportState()
        self.task.phase_index, self.task.phase_entry_step = 9, 100
        self.sample = {'valid': True, 'physics_stamp_valid': True, 'physics_epoch': 0,
                       'physics_step_index': 1, 'physics_time': 1 / 240, 'physics_dt': 1 / 240}
        self.task._beam_table_observation = lambda **kw: dict(self.sample)
        self.advance = {'type': 'all_of', 'physics_stable_steps': 240}

    def step(self, ready=True, increment=1):
        self.sample['physics_step_index'] += increment
        self.sample['physics_time'] += increment / 240
        return self.task._beam_joint_conditions_stable(self.advance, ready)

    def test_only_last_frame_ready_does_not_qualify(self):
        for _ in range(239): self.assertFalse(self.step(False))
        self.assertFalse(self.step(True))
        self.assertEqual(self.task._beam_joint_stability['count'], 1)

    def test_all_conditions_need_240_fresh_joint_samples(self):
        for _ in range(239): self.assertFalse(self.step())
        self.assertTrue(self.step())
        self.assertFalse(self.step(False))
        self.assertFalse(self.step())
        self.assertEqual(self.task._beam_joint_stability['count'], 1)

    def test_duplicate_observers_do_not_accumulate_or_requalify_failed_frame(self):
        self.step()
        for _ in range(300): self.assertFalse(self.task._beam_joint_conditions_stable(self.advance, True))
        self.assertEqual(self.task._beam_joint_stability['count'], 1)
        self.assertFalse(self.task._beam_joint_conditions_stable(self.advance, False))
        self.assertFalse(self.task._beam_joint_conditions_stable(self.advance, True))
        self.assertEqual(self.task._beam_joint_stability['count'], 0)

    def test_epoch_gap_invalid_measurement_or_phase_change_reset_window(self):
        for _ in range(3): self.step()
        self.assertFalse(self.step(increment=2))
        self.assertEqual(self.task._beam_joint_stability['count'], 1)
        self.sample['physics_epoch'] = 1
        self.assertFalse(self.step())
        self.assertEqual(self.task._beam_joint_stability['count'], 1)
        self.sample['valid'] = False
        self.assertFalse(self.step())
        self.assertEqual(self.task._beam_joint_stability['count'], 0)
        self.sample['valid'] = True
        self.step()
        self.task.phase_entry_step += 1
        self.assertFalse(self.step())
        self.assertEqual(self.task._beam_joint_stability['count'], 1)

    def test_actual_all_of_dispatch_uses_joint_gate_but_legacy_does_not(self):
        path = ROOT / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
        node = next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n, ast.FunctionDef)
                    and n.name == '_evaluate_advance_condition')
        scope = {}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(path), 'exec'), scope)
        self.task.phase_step_counter = 100
        self.task._evaluate_advance_condition = scope['_evaluate_advance_condition'].__get__(self.task)
        advance = {**self.advance, 'conditions': [{'type': 'timer'}]}
        for _ in range(239):
            self.sample['physics_step_index'] += 1; self.sample['physics_time'] += 1 / 240
            self.assertFalse(self.task._evaluate_advance_condition(phase_spec={}, advance=advance))
        self.sample['physics_step_index'] += 1; self.sample['physics_time'] += 1 / 240
        self.assertTrue(self.task._evaluate_advance_condition(phase_spec={}, advance=advance))
        self.assertTrue(self.task._evaluate_advance_condition(phase_spec={}, advance={'type': 'all_of', 'conditions': [{'type': 'timer'}]}))


class PublicRobotDefaults(unittest.TestCase):
    def test_actual_ur5e_config_uses_upstream_gripper_defaults(self):
        from internutopia_extension.configs.robots.ur5e import UR5eRobotCfg
        cfg = UR5eRobotCfg()
        self.assertEqual((cfg.gripper_drive_kp, cfg.gripper_drive_kd), (7500., 173.))
        self.assertIsNone(cfg.gripper_pad_static_friction)


if __name__ == '__main__':
    unittest.main()
