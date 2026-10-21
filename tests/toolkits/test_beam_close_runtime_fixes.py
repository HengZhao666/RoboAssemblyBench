"""Exercise loaded settle safety and close-cache commands without a simulator."""
from __future__ import annotations

import copy
from collections import OrderedDict
from types import SimpleNamespace as NS
import unittest

import numpy as np

from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import (
    UR5eAssemblyAtomicSkillAdapter as Adapter,
)


ARM = 'arm_joint_controller'
GRIPPER = 'gripper_controller'


def pose(x=0.0):
    return {'position': np.array([x, 0., 1.]), 'orientation': np.array([1., 0., 0., 0.])}


class LoadedSettle(unittest.TestCase):
    def setUp(self):
        self.adapter = Adapter({})
        self.command_q = np.full(6, .02)
        self.measured_q = np.zeros(6)
        self.saved = {'object': 'fabrica_beam_3', 'arm_q': self.command_q.copy(),
                      'pose': pose(), 'gripper_openness': .175}
        self.metrics = {side: {'force_probe_valid': True, 'force': 1.,
                              'force_observation': {'physics_dt': 1 / 240}}
                        for side in ('left_finger', 'right_finger')}
        self.attachment = {'robot_name': 'left', 'attach_spec': {
            'hold_force_regulation': True, 'contact_force_threshold': .1,
            'close_force_limit': 40., 'close_force_joint_rate': .025,
            'close_force_max_command_lead': .001, 'close_force_max_seek': .03,
        }}
        self.task = NS(phase_index=3, phase_entry_step=20, phase_step_counter=10,
                       step_counter=30, _ur5e_contact_hold_commands={'left': self.saved},
                       robots={'left': NS(config=NS(gripper_close_openness=.08))})
        self.task._attachment_for = lambda *args: self.attachment
        self.task._physical_hold_valid = lambda *args, **kwargs: True
        self.metric_calls = 0

        def metrics(*args, **kwargs):
            self.metric_calls += 1
            return copy.deepcopy(self.metrics)

        self.task._gripper_contact_metrics = metrics
        self.task.get_current_phase_spec = lambda: {}
        self.adapter._current_arm_q = lambda *args: self.measured_q.copy()
        self.adapter._current_robot_pose = lambda **kwargs: pose()
        self.adapter._target_pose = lambda **kwargs: pose()
        self.adapter._prealign_action = lambda **kwargs: None
        self.adapter._object_tcp_slip_failure = lambda **kwargs: None
        self.adapter._debug_transport_step = lambda **kwargs: None
        self.adapter._debug_joint_step = lambda **kwargs: None
        self.adapter._debug_target_object_settle = lambda **kwargs: None
        self.adapter._maybe_relax_insertion_attachment = lambda **kwargs: False
        self.adapter._compliant_recovery_allows_target_settle = lambda **kwargs: False
        self.adapter._compliant_motion_requires_hold = lambda **kwargs: False
        self.adapter._target_object_settle_capture_detail = lambda **kwargs: {
            'configured': True, 'capture_ready': True,
        }
        self.adapter._current_gripper_q = lambda **kwargs: .66
        self.adapter._gripper_open_closed_q = lambda **kwargs: (0., .8)
        self.completions = []
        self.adapter._maybe_mark_complete = lambda **kwargs: self.completions.append(kwargs)
        self.spec = {'name': 'ur5e_move_part_to_staging', 'object': 'fabrica_beam_3',
                     'continuous_physics': True, 'beam_coordinated': True,
                     'requires_held_object': True, 'require_success': True,
                     'gripper_command': 'contact_hold', 'hold_for_target_object_settle': True,
                     'require_target_object_static': True, 'target_object_settle_hold_steps': 48,
                     'max_command_tracking_error': .18, 'max_wrist_command_tracking_error': .12}

    def act(self):
        return self.adapter.act(task=self.task, robot_name='left', phase_spec={},
                                skill_spec=self.spec, tracked_robots={}, tracked_objects={})

    def test_overload_fails_before_completing_the_settle_skill(self):
        for finger in self.metrics.values():
            finger['force'] = 80.
        action = self.act()
        self.assertTrue(action['__local_skill_failure__'])
        self.assertEqual(action['reason'], 'loaded_gripper_force_limit')
        self.assertEqual(self.completions, [])

    def test_missing_or_invalid_force_never_publishes_a_settle_action(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                self.metrics['right_finger'] = {} if missing else {
                    'force_probe_valid': False, 'force': 1.,
                }
                action = self.act()
                self.assertEqual(action['reason'], 'loaded_force_contact_missing')
                self.assertEqual(self.completions, [])

    def test_invalid_physical_hold_cannot_complete(self):
        self.task._physical_hold_valid = lambda *args, **kwargs: False
        self.assertEqual(self.act()['reason'], 'held_object_not_grasped')
        self.assertEqual(self.completions, [])

    def test_low_force_is_regulated_while_published_arm_target_is_retained(self):
        action = self.act()
        np.testing.assert_array_equal(action[ARM][0], self.command_q)
        np.testing.assert_array_equal(self.saved['arm_q'], self.command_q)
        closure = .8 * (1. - action[GRIPPER][0])
        self.assertAlmostEqual(closure, .66 + .025 / 240)
        np.testing.assert_array_equal(self.completions[0]['current_q'], self.measured_q)
        np.testing.assert_array_equal(self.completions[0]['target_q'], self.command_q)

    def test_same_task_step_does_not_apply_force_regulation_twice(self):
        first = self.act()
        second = self.act()
        self.assertEqual(first[GRIPPER], second[GRIPPER])
        self.assertAlmostEqual(self.saved['loaded_force_state']['contact_command_closure'],
                               .66 + .025 / 240)
        np.testing.assert_array_equal(second[ARM][0], self.command_q)

    def test_next_step_continues_rate_limited_regulation(self):
        first = self.act()
        self.task.step_counter += 1
        second = self.act()
        first_closure = .8 * (1. - first[GRIPPER][0])
        second_closure = .8 * (1. - second[GRIPPER][0])
        self.assertAlmostEqual(second_closure - first_closure, .025 / 240)

    def test_legacy_settle_path_retains_its_existing_behavior(self):
        self.spec['beam_coordinated'] = False
        action = self.act()
        np.testing.assert_array_equal(action[ARM][0], self.measured_q)
        self.assertEqual(self.metric_calls, 0)
        self.assertEqual(len(self.completions), 1)


class CloseCommandCache(unittest.TestCase):
    def setUp(self):
        self.adapter = Adapter({})
        self.command_q = np.arange(6, dtype=float) * .001
        self.requested_pose = pose(.05)
        self.fk_pose = pose(.004)
        self.task = NS(phase_index=4, phase_entry_step=40, phase_step_counter=10,
                       step_counter=50, _ur5e_contact_hold_commands={})
        self.spec = {'name': 'ur5e_close_gripper', 'object': 'fabrica_beam_3',
                     'continuous_physics': True, 'beam_coordinated': True,
                     'require_success': True, 'require_close_pose_gate': True,
                     'close_until_contact': True, 'gripper_command': 'contact_hold'}
        self.phase_key = (id(self.task), 4, 40, 'left', 'ur5e_close_gripper')
        self.adapter._close_gate_state[self.phase_key] = {
            'hold_q': self.command_q.copy(), 'hold_command_pose': self.requested_pose,
            'hold_gripper_openness': .12, 'close_started_step': 0,
        }
        self.adapter._current_arm_q = lambda *args: np.zeros(6)
        self.adapter._hold_joint_action = lambda **kwargs: OrderedDict([(ARM, [np.zeros(6).tolist()])])
        self.adapter._close_pose_gate_action = lambda **kwargs: (True, {}, {})
        self.adapter._close_until_contact_ready = lambda **kwargs: (True, {'closed': True})
        self.adapter._debug_close_step = lambda **kwargs: None
        self.fk_calls = []

        def fk(**kwargs):
            self.fk_calls.append(np.asarray(kwargs['command_q']).copy())
            return self.fk_pose

        self.adapter._grasp_command_fk_pose = fk
        self.completions = []
        self.task.mark_local_skill_complete = lambda robot_name, skill_name, detail: self.completions.append(detail)

    def act(self):
        return self.adapter.act(task=self.task, robot_name='left', phase_spec={},
                                skill_spec=self.spec, tracked_robots={}, tracked_objects={})

    def test_cache_uses_final_limited_action_fk_instead_of_requested_pose(self):
        action = self.act()
        saved = self.task._ur5e_contact_hold_commands['left']
        np.testing.assert_array_equal(self.fk_calls[0], action[ARM][0])
        np.testing.assert_array_equal(saved['arm_q'], action[ARM][0])
        np.testing.assert_array_equal(saved['pose']['position'], self.fk_pose['position'])
        self.assertFalse(np.array_equal(saved['pose']['position'], self.requested_pose['position']))
        self.assertEqual(saved['pose_source'], 'published_joint_fk')
        self.assertEqual(saved['step'], 50)
        self.assertEqual(saved['gripper_openness'], action[GRIPPER][0])
        self.assertEqual(self.completions[0]['grasp_command_cache_step'], 50)

    def test_cache_copies_fk_pose_and_joint_target_buffers(self):
        self.act()
        saved = self.task._ur5e_contact_hold_commands['left']
        self.fk_pose['position'][0] = 100.
        self.command_q[0] = 100.
        self.assertEqual(saved['pose']['position'][0], .004)
        self.assertEqual(saved['arm_q'][0], 0.)

    def test_beam_cache_uses_action_fk_even_when_requested_pose_is_unavailable(self):
        self.adapter._close_gate_state[self.phase_key].pop('hold_command_pose')
        action = self.act()
        saved = self.task._ur5e_contact_hold_commands['left']
        np.testing.assert_array_equal(saved['pose']['position'], self.fk_pose['position'])
        np.testing.assert_array_equal(saved['arm_q'], action[ARM][0])
        self.assertEqual(len(self.completions), 1)

    def test_unavailable_fk_does_not_create_cache_or_mark_complete(self):
        self.fk_pose = None
        action = self.act()
        self.assertEqual(action['reason'], 'close_grasp_command_cache_unavailable')
        self.assertEqual(self.task._ur5e_contact_hold_commands, {})
        self.assertEqual(self.completions, [])

    def test_nonfinite_or_malformed_fk_fails_without_replacing_existing_cache(self):
        old = {'object': 'old_part', 'step': 1}
        for bad in ({'position': [float('nan'), 0., 0.], 'orientation': [1., 0., 0., 0.]},
                    {'position': [0., 0.], 'orientation': [1., 0., 0., 0.]},
                    {'position': [0., 0., 0.], 'orientation': [0., 0., 0., 0.]}):
            with self.subTest(bad=bad):
                self.fk_pose = bad
                self.task._ur5e_contact_hold_commands = {'left': old}
                self.assertEqual(self.act()['reason'], 'close_grasp_command_cache_unavailable')
                self.assertIs(self.task._ur5e_contact_hold_commands['left'], old)
                self.assertEqual(self.completions, [])

    def test_missing_final_arm_command_fails_before_cache_and_completion(self):
        self.adapter._close_gate_state[self.phase_key]['hold_q'] = None
        self.assertEqual(self.act()['reason'], 'close_grasp_command_cache_unavailable')
        self.assertEqual(self.task._ur5e_contact_hold_commands, {})
        self.assertEqual(self.completions, [])

    def test_legacy_continuous_close_keeps_its_requested_pose_cache(self):
        self.spec['beam_coordinated'] = False
        self.act()
        saved = self.task._ur5e_contact_hold_commands['left']
        np.testing.assert_array_equal(saved['pose']['position'], self.requested_pose['position'])
        self.assertEqual(self.fk_calls, [])
        self.assertNotIn('pose_source', saved)
        self.assertEqual(len(self.completions), 1)


if __name__ == '__main__':
    unittest.main()
