"""Regress physical attachment flow and opt-in weak-side force recentering."""
from __future__ import annotations

import copy
from types import SimpleNamespace as NS
import unittest

import numpy as np

from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import (
    UR5eAssemblyAtomicSkillAdapter as Adapter,
)


def metrics(left=0., right=6., axis='y', orientation=None):
    index = {'x': 0, 'y': 1, 'z': 2}[axis]
    result = {'contact_box_center': [0., 0., 0.],
              'contact_box_orientation': [1., 0., 0., 0.] if orientation is None else orientation,
              'pinch_axis': 'x' if axis == 'y' else 'y'}
    for side, force, sign in (('left_finger', left, 1.), ('right_finger', right, -1.)):
        point = np.zeros(3); point[index] = sign * .009
        result[side] = {'force': force, 'force_probe_valid': True,
                        'force_observation': {'physics_dt': 1 / 240,
                            'contacts': [{'normal_force': force, 'point_world': point.tolist()}]}}
    return result


class WeakSideRecenter(unittest.TestCase):
    def setUp(self):
        self.spec = {'close_force_recenter_track_weak_side': True,
                     'physical_contact_axes': 'y', 'close_force_balance_min': 2.,
                     'close_force_balance_peak': 4., 'close_force_high': 12.,
                     'close_force_limit': 40., 'contact_force_threshold': .1,
                     'close_gate_recenter_stable_steps': 4,
                     'close_gate_recenter_step': .0002, 'close_force_balance_step': .000025,
                     'close_gate_recenter_max_offset': .002,
                     'close_force_recenter_measure_progress': True,
                     'close_force_recenter_adaptive_step': True,
                     'close_force_recenter_min_step': .000025,
                     'close_force_recenter_stop_on_bilateral': True,
                     'close_force_recenter_settle_steps': 8,
                     'close_force_recenter_timeout_steps': 240}
        self.state = {'close_tcp_position_world': np.zeros(3)}

    def update(self, left=0., right=6., observation=None):
        self.state['force_recenter_observation_step'] = self.state.get('force_recenter_observation_step', 0) + 1
        return Adapter._update_force_recenter(
            state=self.state, spec=self.spec,
            metrics=metrics(left, right) if observation is None else observation)

    def confirm_three(self):
        for _ in range(3):
            self.assertFalse(self.update()['updated'])
        self.assertEqual(self.state['force_single_steps'], 3)

    def assert_small_margin_move(self, result):
        self.assertTrue(result['updated'])
        self.assertEqual(result['motion_mode'], 'force_margin')
        np.testing.assert_allclose(result['motion_delta_world'], [0., -.000025, 0.], atol=1e-15)

    def test_single_and_margin_label_changes_confirm_same_side(self):
        for left in (0., .3, 0.):
            self.assertFalse(self.update(left)['updated'])
        self.assert_small_margin_move(self.update(.3))

    def test_qualified_window_peak_does_not_treat_existing_high_phase_as_new_growth(self):
        for left, right in ((0., 7.), (.3, 7.3), (.3, 7.3), (.3, .659)):
            self.update(left, right)
        pending = self.state['force_recenter_pending']
        self.assertAlmostEqual(pending['imbalance'], 7.)
        pending['lifecycle'] = 'final_action_issued'
        self.state['recenter_target_ready'] = True
        for _ in range(8):
            self.update(.3, 7.3)
        self.assertNotIn('force_recenter_fault', self.state)
        self.assertNotIn('force_recenter_pending', self.state)

    def test_true_growth_after_qualified_window_remains_a_fault(self):
        for left, right in ((0., 7.), (.3, 7.3), (.3, 7.3), (.3, .659)):
            self.update(left, right)
        self.state['force_recenter_pending']['lifecycle'] = 'final_action_issued'
        self.state['recenter_target_ready'] = True
        for _ in range(8):
            self.update(.3, 15.)
        self.assertEqual(self.state['force_recenter_fault'], 'force_imbalance_increased')

    def test_clock_gap_drops_old_unload_peak_before_new_window(self):
        for _ in range(3):
            self.update(.3, 28.)
        self.state['force_recenter_observation_step'] += 5
        for _ in range(4):
            self.update(.3, 6.)
        self.assertAlmostEqual(self.state['force_recenter_pending']['imbalance'], 5.7)

    def test_same_task_sample_never_accumulates_confirmation_or_pending_time(self):
        self.confirm_three()
        for _ in range(96):
            repeated = Adapter._update_force_recenter(state=self.state, spec=self.spec, metrics=metrics())
            self.assertEqual(repeated['reason'], 'repeat_observation')
        self.assertEqual(self.state['force_single_steps'], 3)
        self.assert_small_margin_move(self.update())
        for _ in range(96):
            Adapter._update_force_recenter(state=self.state, spec=self.spec, metrics=metrics())
        self.assertNotIn('elapsed_steps', self.state['force_recenter_pending'])

    def test_task_sample_gap_starts_a_new_confirmation(self):
        self.confirm_three()
        self.state['force_recenter_observation_step'] += 5
        self.assertFalse(self.update()['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)

    def test_direct_helper_without_task_stamp_reports_unguarded_interface(self):
        result = Adapter._update_force_recenter(state=self.state, spec=self.spec, metrics=metrics())
        self.assertFalse(result['confirmation_clock_guarded'])

    def test_soft_unload_keeps_evidence_but_cannot_issue_motion(self):
        for right in (6., 13., 6.):
            self.assertFalse(self.update(.3, right)['updated'])
        fourth = self.update(.3, 13.)
        self.assertEqual(fourth['reason'], 'unload_before_recenter')
        self.assertEqual(self.state['force_single_steps'], 4)
        self.assertNotIn('force_recenter_pending', self.state)
        self.assert_small_margin_move(self.update(.3, 6.))

    def test_preload_and_unload_oscillation_qualifies_without_changing_q_control(self):
        for right in (3., 13., 3., 13.):
            self.assertFalse(self.update(.3, right)['updated'])
        self.assert_small_margin_move(self.update(.3, 3.))

    def test_disabled_flag_preserves_legacy_confirmation_and_band_behavior(self):
        self.spec.pop('close_force_recenter_track_weak_side')
        for left in (0., .3) * 6:
            self.assertFalse(self.update(left)['updated'])
        self.state = {'close_tcp_position_world': np.zeros(3)}
        for right in (6., 13.) * 6:
            self.assertFalse(self.update(.3, right)['updated'])
        self.assertNotIn('force_recenter_pending', self.state)

    def test_at_hard_limit_starts_a_fresh_four_observation_confirmation(self):
        self.confirm_three()
        self.assertEqual(self.update(.3, 40.)['reason'], 'force_limit_before_recenter')
        self.assertEqual(self.state['force_single_steps'], 0)
        for _ in range(3):
            self.assertFalse(self.update()['updated'])
        self.assert_small_margin_move(self.update())

    def test_invalid_force_and_missing_probe_clear_confirmation(self):
        for invalid in ({}, {'force_probe_valid': False, 'force': 6.},
                        {'force_probe_valid': True, 'force': float('nan')}):
            with self.subTest(invalid=invalid):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.confirm_three()
                value = metrics(); value['right_finger'] = invalid
                self.assertEqual(self.update(observation=value)['reason'], 'invalid_probe')
                self.assertEqual(self.state['force_single_steps'], 0)
                self.assertFalse(self.update()['updated'])

    def test_strong_side_reversal_requires_four_new_observations(self):
        self.confirm_three()
        for _ in range(3):
            self.assertFalse(self.update(6., .3)['updated'])
        moved = self.update(6., .3)
        self.assertTrue(moved['updated'])
        self.assertEqual(moved['loaded_side'], 0)
        np.testing.assert_allclose(moved['motion_delta_world'], [0., .000025, 0.])

    def test_same_finger_contact_direction_reversal_restarts_confirmation(self):
        self.confirm_three()
        reversed_contact = metrics()
        reversed_contact['right_finger']['force_observation']['contacts'][0]['point_world'] = [0., .009, 0.]
        self.assertFalse(self.update(observation=reversed_contact)['updated'])
        self.assertEqual(self.state['force_single_steps'], 1)

    def test_missing_contact_direction_clears_eligible_observations(self):
        self.confirm_three()
        value = metrics(); value['right_finger']['force_observation']['contacts'] = []
        self.assertEqual(self.update(observation=value)['reason'], 'loaded_contact_frame_unavailable')
        self.assertEqual(self.state['force_single_steps'], 0)

    def test_invalid_contact_frame_cannot_qualify_or_issue_motion(self):
        for field, value in (('contact_box_orientation', [0., 0., 0., 0.]),
                             ('contact_box_orientation', [float('nan'), 0., 0., 0.]),
                             ('contact_box_center', [0., 0.])):
            with self.subTest(field=field, value=value):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.confirm_three()
                observation = metrics(); observation[field] = value
                self.assertEqual(self.update(observation=observation)['reason'], 'loaded_contact_frame_unavailable')
                self.assertEqual(self.state['force_single_steps'], 0)

    def test_reaching_weak_force_target_and_no_contact_clear_confirmation(self):
        for left, right in ((2., 6.), (0., 0.)):
            with self.subTest(forces=(left, right)):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.confirm_three(); self.assertFalse(self.update(left, right)['updated'])
                self.assertEqual(self.state['force_single_steps'], 0)

    def test_margin_motion_continues_through_point_one_bilateral_contact(self):
        self.confirm_three(); self.update(.3)
        self.assertIsNone(Adapter._stop_recenter_on_bilateral(
            state=self.state, spec=self.spec, metrics=metrics(.3, 6.), current_position=[0., 0., 0.]))
        self.assertIn('force_recenter_pending', self.state)
        for forces, expected in (((2., 6.), 'force_margin_reached'),
                                 ((6., .3), 'force_balance_side_reversed')):
            with self.subTest(forces=forces):
                state = copy.deepcopy(self.state)
                stopped = Adapter._stop_recenter_on_bilateral(
                    state=state, spec=self.spec, metrics=metrics(*forces), current_position=[0., -.00001, 0.])
                self.assertEqual(stopped['stop_cause'], expected)
                self.assertNotIn('force_recenter_pending', state)
                self.assertLessEqual(np.linalg.norm(state['recenter_offset_world']), .000025)

    def test_hard_force_cancels_pending_motion_even_with_no_bilateral_contact(self):
        self.confirm_three(); self.update()
        for weak, peak in ((0., 80.), (.3, 40.)):
            with self.subTest(forces=(weak, peak)):
                state = copy.deepcopy(self.state)
                stopped = Adapter._stop_recenter_on_bilateral(
                    state=state, spec=self.spec, metrics=metrics(weak, peak), current_position=[0., -.00001, 0.])
                self.assertEqual(stopped['stop_cause'], 'hard_force_limit')
                self.assertNotIn('force_recenter_pending', state)

    def test_soft_overload_and_disabled_option_keep_original_pending_motion(self):
        self.confirm_three(); self.update()
        for peak, enabled in ((13., True), (39.9, True), (80., False)):
            with self.subTest(peak=peak, enabled=enabled):
                state = copy.deepcopy(self.state)
                spec = {**self.spec, 'close_force_recenter_track_weak_side': enabled}
                self.assertIsNone(Adapter._stop_recenter_on_bilateral(
                    state=state, spec=spec, metrics=metrics(.3, peak), current_position=[0., -.00001, 0.]))
                self.assertIn('force_recenter_pending', state)

    def test_hard_force_cancel_projects_progress_and_keeps_original_anchor_and_travel_charge(self):
        self.state['recenter_offset_world'] = np.array([0., -.0005, 0.])
        self.state['close_tcp_position_world'] = np.array([0., -.0005, 0.])
        self.state['force_recenter_travel'] = .0005
        anchor = {'position': np.zeros(3), 'orientation': np.array([1., 0., 0., 0.])}
        self.state['contact_arm_anchor'] = copy.deepcopy(anchor)
        self.state['recenter_anchor_position_world'] = anchor['position'].copy()
        self.confirm_three(); self.update()
        charged = self.state['force_recenter_travel']
        self.state['force_recenter_pending']['lifecycle'] = 'final_action_issued'
        stopped = Adapter._stop_recenter_on_bilateral(
            state=self.state, spec=self.spec, metrics=metrics(.3, 80.), current_position=[.003, -.00051, .002])
        self.assertAlmostEqual(stopped['progress_fraction'], .4)
        self.assertAlmostEqual(stopped['cancelled_distance'], .000015)
        np.testing.assert_allclose(self.state['recenter_offset_world'], [0., -.00051, 0.])
        self.assertEqual(self.state['force_recenter_travel'], charged)
        self.assertAlmostEqual(charged, .000525)
        np.testing.assert_array_equal(self.state['contact_arm_anchor']['position'], anchor['position'])
        np.testing.assert_array_equal(self.state['contact_arm_anchor']['orientation'], anchor['orientation'])
        np.testing.assert_array_equal(self.state['recenter_anchor_position_world'], anchor['position'])

    def test_actual_configured_single_axis_controls_motion_not_pinch_metric(self):
        for axis, index in (('x', 0), ('y', 1), ('z', 2)):
            with self.subTest(axis=axis):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.spec['physical_contact_axes'] = axis
                for _ in range(4):
                    moved = self.update(observation=metrics(axis=axis))
                expected = np.zeros(3); expected[index] = -.000025
                np.testing.assert_allclose(moved['motion_delta_world'], expected)

    def test_rotated_configured_axis_uses_world_contact_point(self):
        self.spec['physical_contact_axes'] = ['x']
        value = metrics(axis='x', orientation=[2 ** -.5, 0., 0., 2 ** -.5])
        value['right_finger']['force_observation']['contacts'][0]['point_world'] = [0., -.009, 0.]
        for _ in range(4):
            moved = self.update(observation=value)
        np.testing.assert_allclose(moved['motion_delta_world'], [0., -.000025, 0.], atol=1e-15)

    def test_default_axis_retains_legacy_y_direction_and_step(self):
        self.spec.pop('physical_contact_axes'); self.spec.pop('close_force_recenter_track_weak_side')
        for _ in range(4):
            moved = self.update()
        np.testing.assert_allclose(moved['motion_delta_world'], [0., -.0002, 0.])

    def test_ambiguous_or_invalid_axis_never_issues_motion(self):
        for configured in ('x,z', ['x', 'z'], 'invalid', []):
            with self.subTest(configured=configured):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.spec['physical_contact_axes'] = configured
                self.assertEqual(self.update()['reason'], 'recenter_axis_unavailable')
                self.assertEqual(self.state['force_single_steps'], 0)

    def test_legacy_multiple_axes_containing_y_keep_existing_y_recenter(self):
        self.spec.pop('close_force_recenter_track_weak_side')
        for configured in ('x,y', ['x', 'y', 'z']):
            with self.subTest(configured=configured):
                self.state = {'close_tcp_position_world': np.zeros(3)}
                self.spec['physical_contact_axes'] = configured
                for _ in range(4):
                    moved = self.update()
                np.testing.assert_allclose(moved['motion_delta_world'], [0., -.0002, 0.])

    def test_travel_limit_and_pending_timeout_are_preserved(self):
        self.state['force_recenter_travel'] = .002 - .00001
        self.confirm_three(); moved = self.update()
        self.assertAlmostEqual(np.linalg.norm(moved['motion_delta_world']), .00001)
        self.assertAlmostEqual(self.state['force_recenter_travel'], .002)
        self.state['force_recenter_pending']['lifecycle'] = 'final_action_issued'
        for _ in range(240):
            result = self.update()
        self.assertEqual(result['reason'], 'recenter_motion_timeout')

    def test_hard_overload_clears_confirmation_during_existing_motion(self):
        self.confirm_three(); self.update()
        self.state['force_recenter_pending']['lifecycle'] = 'final_action_issued'
        self.state['force_single_steps'] = 3
        self.assertEqual(self.update(.3, 40.)['reason'], 'servo_in_progress')
        self.assertEqual(self.state['force_single_steps'], 0)
        self.assertEqual(self.state['force_recenter_pending']['elapsed_steps'], 1)

    def test_unissued_plan_never_credits_prior_arm_drift_or_accumulates_execution_time(self):
        self.confirm_three(); self.update()
        for _ in range(300):
            self.assertEqual(self.update()['reason'], 'awaiting_final_action')
        self.assertNotIn('elapsed_steps', self.state['force_recenter_pending'])
        stopped = Adapter._stop_recenter_on_bilateral(
            state=self.state, spec=self.spec, metrics=metrics(.3, 80.),
            current_position=[.002, -.001, 0.])
        self.assertEqual(stopped['progress_fraction'], 0.)
        np.testing.assert_array_equal(self.state['recenter_offset_world'], np.zeros(3))
        self.assertAlmostEqual(self.state['force_recenter_travel'], .000025)


class FinalActionRecenterSafety(unittest.TestCase):
    """Exercise production act with a linear IK/FK and explicit toy feedback."""
    def fixture(self, initial=.0003):
        adapter = Adapter({})
        spec = {'name': 'ur5e_close_gripper', 'robot': 'left', 'object': 'part',
                'continuous_physics': True, 'require_success': True,
                'require_close_pose_gate': True, 'close_until_contact': True,
                'close_gate_freeze_at_start': True, 'close_force_regulation': True,
                'close_ready_stable_steps': 24, 'close_position_tolerance': .001,
                'close_orientation_tolerance': .02, 'ik_position_tolerance': .000005,
                'ik_orientation_tolerance': .001, 'max_joint_step': .004,
                'max_command_joint_step': .004, 'close_force_validation_steps': 96,
                'close_until_contact_timeout_steps': 1200,
                'close_gate_recenter_single_finger_contact': True,
                'close_force_recenter_track_weak_side': True,
                'close_force_recenter_stop_on_bilateral': True,
                'close_force_recenter_adaptive_step': True,
                'close_force_recenter_measure_progress': True,
                'close_force_recenter_timeout_steps': 240, 'close_force_recenter_settle_steps': 8,
                'close_gate_recenter_target_tolerance': .0001,
                'close_gate_recenter_stable_steps': 4,
                'close_force_balance_min': 2., 'close_force_balance_step': .000025,
                'close_force_recenter_min_step': .000025, 'close_gate_recenter_step': .0002,
                'close_gate_recenter_max_offset': .002, 'physical_contact_axes': 'y',
                'ik_reference_mode': 'current', 'use_command_warm_start': True,
                'require_warm_start_ik': True}
        native = {'q': np.array([initial, 0., 0., 0., 0., 0.]), 'right_force': 6.}
        raw = NS(compute_inverse_kinematics=lambda frame, position, orientation, **kw:
                 (np.array([position[1], 0., 0., 0., 0., 0.]), True),
                 compute_forward_kinematics=lambda frame, q: (np.array([0., q[0], 0.]), np.eye(3)))
        wrapper = NS(get_kinematics_solver=lambda: raw, get_end_effector_frame=lambda: 'tool0',
                     set_robot_base_pose=lambda **kw: None)
        controller = NS(_kinematics_solver=wrapper, _robot_scale=np.ones(3),
                        get_ik_base_world_pose=lambda: (np.zeros(3), np.array([1., 0., 0., 0.])))
        task = NS(step_counter=100, phase_step_counter=100, phase_index=4, phase_entry_step=0,
                  phase='close', robots={'left': NS(config=NS(gripper_close_openness=.04),
                           controllers={'arm_ik_controller': controller})})
        identity = np.array([1., 0., 0., 0.])
        state = {'close_arm_frozen': True, 'close_started_step': 0, 'ready_steps': 24,
                 'contact_control_active': True, 'contact_capture_closure': .732,
                 'contact_command_closure': .737,
                 'contact_arm_anchor': {'position': np.zeros(3), 'orientation': identity.copy()},
                 'hold_q': np.zeros(6), 'force_single_steps': 3, 'force_single_side': 1,
                 'force_recenter_confirmation_mode': 'force_margin',
                 'force_recenter_confirmation_direction': np.array([0., -1., 0.]),
                 'force_recenter_last_observation_step': 99}
        adapter._close_gate_state[(id(task), 4, 0, 'left', spec['name'])] = state
        adapter._remember_arm_command(task, 'left', np.zeros(6))
        pose = lambda: {'position': np.array([0., native['q'][0], 0.]), 'orientation': identity.copy()}
        adapter._target_pose = lambda **kw: pose()
        adapter._current_robot_pose = lambda **kw: pose()
        adapter._current_arm_q = lambda *args: native['q'].copy()
        adapter._current_gripper_q = lambda **kw: .737
        adapter._gripper_open_closed_q = lambda **kw: (0., .8)
        adapter._grasp_contact_ready = lambda **kw: (False, {'contact_metrics': metrics(.3, native['right_force'])})
        def act():
            return adapter.act(task=task, robot_name='left', phase_spec={}, skill_spec=spec,
                               tracked_robots={}, tracked_objects={})
        return adapter, task, native, state, act

    def test_full_act_hard_force_before_first_issue_returns_no_new_translation(self):
        adapter, task, native, state, act = self.fixture()
        stops = []
        def observe_stop(**kwargs):
            result = Adapter._stop_recenter_on_bilateral(**kwargs)
            if result is not None:
                stops.append(result)
            return result
        adapter._stop_recenter_on_bilateral = observe_stop
        old = act()
        self.assertEqual(state['force_recenter_pending']['lifecycle'], 'planned')
        native['q'] = np.asarray(old['arm_joint_controller'][0])
        task.step_counter = task.phase_step_counter = 101
        native['right_force'] = 80.
        action = act()
        self.assertEqual(action['arm_joint_controller'][0][0], 0.)
        self.assertEqual(stops[-1]['progress_fraction'], 0.)
        self.assertEqual(stops[-1]['lifecycle'], 'planned')
        self.assertAlmostEqual(.8 * (1. - action['gripper_controller'][0]), .737 - .06 / 240)
        self.assertEqual(state['close_contact_stable_steps'], 0)

    def test_full_act_issue_starts_from_issuance_sample_and_retains_final_command_fk(self):
        adapter, task, native, state, act = self.fixture()
        old = act()
        native['q'] = np.asarray(old['arm_joint_controller'][0])
        task.step_counter = task.phase_step_counter = 101
        # Simulate the final joint limiter returning half the IK request.
        adapter._limit_command_to_measured_state = lambda **kw: kw['command_q'] * .5
        action = act()
        pending = state['force_recenter_pending']
        self.assertEqual(pending['lifecycle'], 'final_action_issued')
        self.assertEqual(pending['issued_step'], 101)
        np.testing.assert_array_equal(pending['motion_start_position_world'], [0., 0., 0.])
        np.testing.assert_array_equal(pending['issued_command_q'], action['arm_joint_controller'][0])
        self.assertAlmostEqual(pending['issued_command_tcp_position_world'][1], -.0000125)
        self.assertAlmostEqual(pending['motion_target_position_world'][1], -.000025)
        self.assertEqual(pending['elapsed_steps'], 0)

    def test_missing_final_fk_aborts_before_returning_new_arm_action(self):
        adapter, task, native, state, act = self.fixture()
        old = act(); native['q'] = np.asarray(old['arm_joint_controller'][0])
        task.step_counter = task.phase_step_counter = 101
        adapter._grasp_command_fk_pose = lambda **kw: None
        action = act()
        self.assertTrue(action['__local_skill_failure__'])
        self.assertEqual(action['reason'], 'recenter_final_action_unavailable')
        self.assertEqual(state['force_recenter_pending']['lifecycle'], 'planned')

    def test_aligned_transient_lag_and_fixed_bias_finish_bounded_increment(self):
        for initial, bias in ((0., 0.), (.0003, 0.), (.0003, .0003)):
            with self.subTest(initial=initial, bias=bias):
                adapter, task, native, state, act = self.fixture(initial)
                for step in range(100, 120):
                    task.step_counter = task.phase_step_counter = step
                    action = act()
                    self.assertNotIn('__local_skill_failure__', action)
                    native['q'] = np.asarray(action['arm_joint_controller'][0]); native['q'][0] += bias
                    if step > 101 and 'force_recenter_pending' not in state:
                        break
                self.assertNotIn('force_recenter_pending', state)
                self.assertAlmostEqual(state['force_recenter_travel'], .000025)


class PureAttachmentTransition(unittest.TestCase):
    def setUp(self):
        self.adapter = Adapter({})
        self.spec = {'object': 'part', 'robot': 'left',
                     'target_object_final_target': 'assembled',
                     'relax_fixed_attachment_within_final_position_tolerance': .015,
                     'relax_fixed_attachment_stable_steps': 2}
        self.pose = {'position': np.zeros(3), 'orientation': np.array([1., 0., 0., 0.])}
        self.calls = []
        self.task = NS(phase_step_counter=20, target_poses={'assembled': self.pose},
                       _attachments={'part': {'mode': 'fixed_joint', 'robot_name': 'right'}})
        self.task.relax_fixed_attachment_to_physical_hold = lambda *args, **kwargs: self.calls.append(args) or True
        self.adapter._object_pose = lambda **kwargs: self.pose
        self.adapter._object_motion_detail = lambda **kwargs: {'valid': True, 'linear_speed': 0., 'angular_speed': 0.}

    def update(self, tracked=None):
        return self.adapter._maybe_relax_insertion_attachment(
            task=self.task, spec=self.spec, tracked_objects={} if tracked is None else tracked)

    def test_actual_shared_pure_attachment_never_waits_or_marks_compliance_complete(self):
        requested = []
        self.task._attachment_for = lambda obj, robot: requested.append((obj, robot)) or {
            'mode': 'pure_physical_grasp', 'robot_name': robot}
        key = (id(self.task), 'part')
        self.adapter._insertion_compliance_transition_state[key] = {'stable_steps': 1}
        self.adapter._completed_insertion_compliance_transitions[key] = {'capture_mode': 'compliant_joint'}
        for _ in range(20):
            self.assertFalse(self.update())
        self.assertEqual(requested, [('part', 'left')] * 20)
        self.assertEqual(self.calls, [])
        self.assertNotIn(key, self.adapter._insertion_compliance_transition_state)
        self.assertNotIn(key, self.adapter._completed_insertion_compliance_transitions)

    def test_fixed_joint_legacy_still_waits_then_converts(self):
        self.assertTrue(self.update({'part': {'attachment_mode': 'pure_physical_grasp'}}))
        self.assertEqual(self.calls, [])
        self.assertFalse(self.update())
        self.assertEqual(self.calls, [('part',)])
        self.assertEqual(self.adapter._completed_insertion_compliance_transitions[(id(self.task), 'part')]['capture_mode'],
                         'compliant_joint')

    def test_none_actual_attachment_does_not_use_optimistic_tracked_mode(self):
        self.task._attachment_for = lambda *args: None
        self.assertTrue(self.update({'part': {'attachment_mode': 'pure_physical_grasp'}}))


class PartialProbePendingSafety(unittest.TestCase):
    def fixture(self, observation, enabled=True):
        adapter = Adapter({})
        spec = {'name': 'ur5e_close_gripper', 'object': 'part', 'robot': 'right',
                'continuous_physics': True, 'require_success': True,
                'require_close_pose_gate': True, 'close_until_contact': True,
                'close_force_regulation': True, 'close_gate_freeze_at_start': True,
                'close_ready_stable_steps': 1, 'close_force_validation_steps': 96,
                'close_gate_recenter_single_finger_contact': True,
                'close_force_recenter_stop_on_bilateral': True,
                'close_force_recenter_track_weak_side': enabled,
                'close_force_recenter_measure_progress': True,
                'close_force_balance_min': 2., 'close_force_limit': 40.,
                'close_force_unload_rate': .06}
        pose = {'position': np.array([0., -.00001, 0.]), 'orientation': np.array([1., 0., 0., 0.])}
        task = NS(step_counter=100, phase_step_counter=100, phase_index=4, phase_entry_step=0,
                  phase='close', robots={'right': NS(config=NS(gripper_close_openness=.04))})
        key = (id(task), 4, 0, 'right', 'ur5e_close_gripper')
        state = {'close_arm_frozen': True, 'close_started_step': 0, 'ready_steps': 1,
                 'contact_control_active': True, 'contact_capture_closure': .73,
                 'contact_command_closure': .74,
                 'contact_arm_anchor': {'position': np.zeros(3), 'orientation': pose['orientation'].copy()},
                 'recenter_offset_world': np.array([0., -.000025, 0.]),
                 'force_recenter_travel': .000025,
                 'force_recenter_pending': {'mode': 'force_margin', 'loaded_side': 1,
                    'offset_before': np.zeros(3), 'motion_start_position_world': np.zeros(3),
                    'motion_delta_world': np.array([0., -.000025, 0.]),
                    'motion_target_position_world': np.array([0., -.000025, 0.]),
                    'imbalance': 6., 'settle_steps': 0, 'adaptive_progress': True}}
        adapter._close_gate_state[key] = state
        adapter._target_pose = lambda **kwargs: copy.deepcopy(pose)
        adapter._current_robot_pose = lambda **kwargs: copy.deepcopy(pose)
        adapter._current_tcp_pose = lambda **kwargs: kwargs['current_pose']
        adapter._current_arm_q = lambda *args: np.zeros(6)
        adapter._command_reference_q = lambda **kwargs: kwargs['current_q']
        adapter._ik_target_pose = lambda **kwargs: kwargs['target_pose']
        adapter._solve_ik = lambda **kwargs: np.array([kwargs['target_pose']['position'][1], 0., 0., 0., 0., 0.])
        adapter._ik_branch_jump_detected = lambda **kwargs: False
        adapter._current_gripper_q = lambda **kwargs: .74
        adapter._gripper_open_closed_q = lambda **kwargs: (0., .8)
        adapter._continuous_command_q = lambda **kwargs: kwargs['command_q']
        adapter._limit_command_to_measured_state = lambda **kwargs: kwargs['command_q']
        adapter._grasp_contact_ready = lambda **kwargs: (False, {'contact_metrics': copy.deepcopy(observation)})
        action = adapter.act(task=task, robot_name='right', phase_spec={}, skill_spec=spec,
                             tracked_robots={}, tracked_objects={})
        return action, state

    def test_actual_gate_partial_probe_known_80_cancels_arm_and_hard_unloads_gripper(self):
        value = metrics(0., 80.); value['left_finger'] = {}
        action, state = self.fixture(value)
        self.assertAlmostEqual(action['arm_joint_controller'][0][0], -.00001)
        self.assertAlmostEqual(.8 * (1. - action['gripper_controller'][0]), .74 - .06 / 240)
        self.assertNotIn('force_recenter_pending', state)
        self.assertEqual(state['force_recenter_stop_event']['stop_cause'], 'hard_force_limit')
        self.assertEqual(state['force_recenter_stop_event']['forces'], [None, 80.])
        self.assertEqual(state['force_recenter_travel'], .000025)
        self.assertEqual(state['close_contact_stable_steps'], 0)

    def test_actual_gate_both_missing_cancel_arm_but_keep_invalid_probe_gripper_hold(self):
        value = metrics(); value['left_finger'] = {}; value['right_finger'] = {}
        action, state = self.fixture(value)
        self.assertAlmostEqual(action['arm_joint_controller'][0][0], -.00001)
        self.assertAlmostEqual(.8 * (1. - action['gripper_controller'][0]), .74)
        self.assertEqual(state['force_recenter_stop_event']['stop_cause'], 'contact_probe_unavailable')
        self.assertEqual(state['force_recenter_stop_event']['forces'], [None, None])
        self.assertEqual(state['close_contact_stable_steps'], 0)

    def test_actual_gate_disabled_flag_preserves_partial_probe_pending_and_gripper_hold(self):
        value = metrics(0., 80.); value['left_finger'] = {}
        action, state = self.fixture(value, enabled=False)
        self.assertAlmostEqual(action['arm_joint_controller'][0][0], -.000025)
        self.assertAlmostEqual(.8 * (1. - action['gripper_controller'][0]), .74)
        self.assertIn('force_recenter_pending', state)
        self.assertNotIn('force_recenter_stop_event', state)

    def test_actual_gate_known_40_boundary_cancels_and_unloads_without_qualifying(self):
        value = metrics(0., 40.); value['left_finger']['force_probe_valid'] = False
        value['left_finger']['force'] = None
        action, state = self.fixture(value)
        self.assertAlmostEqual(action['arm_joint_controller'][0][0], -.00001)
        self.assertAlmostEqual(.8 * (1. - action['gripper_controller'][0]), .74 - .06 / 240)
        self.assertEqual(state['force_recenter_stop_event']['forces'], [None, 40.])
        self.assertEqual(state['close_contact_stable_steps'], 0)

    def test_controller_unknown_force_stays_none_and_only_known_hard_force_unloads(self):
        for known, enabled, expected, unknown in ((80., True, 'force_limit_unload', None),
                                                 (40., True, 'force_limit_unload', None),
                                                 (39.9, True, 'invalid_probe_hold', None),
                                                 (6., True, 'invalid_probe_hold', None),
                                                 (80., False, 'invalid_probe_hold', 80.)):
            with self.subTest(known=known, enabled=enabled):
                value = metrics(80., known); value['left_finger']['force_probe_valid'] = False
                control = Adapter._contact_close_control(state={'contact_command_closure': .74},
                    spec={'close_force_recenter_track_weak_side': enabled, 'close_force_limit': 40.,
                          'close_force_unload_rate': .06}, metrics=value, gripper_q=.74,
                    open_q=0., closed_q=.8, requested_openness=0., min_openness=.04)
                self.assertEqual(control['mode'], expected)
                self.assertEqual(control['forces'], [unknown, known])
                self.assertFalse(control['force_within_limit'])
                self.assertFalse(control['probe_valid'])
                expected_q = .74 - .06 / 240 if expected == 'force_limit_unload' else .74
                self.assertAlmostEqual(control['command_q'], expected_q)


class PureInsertionLoadedProtection(unittest.TestCase):
    def fixture(self):
        from tests.toolkits.test_beam_close_runtime_fixes import LoadedSettle

        fixture = LoadedSettle('test_overload_fails_before_completing_the_settle_skill')
        fixture.setUp()
        fixture.attachment['mode'] = 'pure_physical_grasp'
        fixture.spec.update(target_object_final_target='assembled',
                            relax_fixed_attachment_within_final_position_tolerance=.015)
        pose = {'position': np.array([0., 0., 1.]), 'orientation': np.array([1., 0., 0., 0.])}
        fixture.task.target_poses = {'assembled': pose}
        fixture.task.relax_fixed_attachment_to_physical_hold = lambda *args, **kwargs: False
        fixture.adapter._object_pose = lambda **kwargs: pose
        fixture.adapter._maybe_relax_insertion_attachment = Adapter._maybe_relax_insertion_attachment.__get__(fixture.adapter)
        return fixture

    def test_actual_pure_insertion_reaches_force_protection_before_completing(self):
        fixture = self.fixture()
        for finger in fixture.metrics.values():
            finger['force'] = 80.
        self.assertEqual(fixture.act()['reason'], 'loaded_gripper_force_limit')
        self.assertEqual(fixture.completions, [])

    def test_actual_pure_insertion_rejects_missing_force(self):
        fixture = self.fixture(); fixture.metrics['right_finger'] = {}
        self.assertEqual(fixture.act()['reason'], 'loaded_force_contact_missing')
        self.assertEqual(fixture.completions, [])

    def test_actual_pure_insertion_keeps_normal_settle_completion_and_rate(self):
        fixture = self.fixture(); action = fixture.act()
        self.assertEqual(len(fixture.completions), 1)
        closure = .8 * (1. - action['gripper_controller'][0])
        self.assertAlmostEqual(closure, .66 + .025 / 240)


class CloseQualificationUnchanged(unittest.TestCase):
    def test_close_act_supplies_task_tick_and_counts_repeated_calls_once(self):
        from tests.toolkits.test_beam_close_runtime_fixes import CloseCommandCache

        fixture = CloseCommandCache('test_cache_uses_final_limited_action_fk_instead_of_requested_pose')
        fixture.setUp()
        fixture.spec.update(close_force_recenter_track_weak_side=True,
                            close_force_regulation=True,
                            close_gate_recenter_single_finger_contact=True,
                            close_force_balance_min=2., close_gate_recenter_stable_steps=4)
        fixture.adapter._close_until_contact_ready = lambda **kwargs: (
            False, {'contact_detail': {'contact_metrics': metrics()}})
        for _ in range(4):
            fixture.act()
        state = fixture.adapter._close_gate_state[fixture.phase_key]
        self.assertEqual(state['force_recenter_observation_step'], fixture.task.step_counter)
        self.assertEqual(state['force_single_steps'], 1)
        self.assertNotIn('force_recenter_pending', state)

    def test_new_recenter_option_cannot_replace_dual_force_strict_geometry_or_96_samples(self):
        adapter = Adapter({})
        adapter._current_gripper_q = lambda **kwargs: .74
        adapter._gripper_open_closed_q = lambda **kwargs: (0., .8)
        task = NS(step_counter=0, robots={'left': NS(config=NS(gripper_close_openness=.04))})
        spec = {'object': 'part', 'continuous_physics': True, 'close_force_regulation': True,
                'close_force_recenter_track_weak_side': True, 'close_force_min_stable': 1.,
                'close_force_validation_steps': 96, 'close_force_limit': 40.}

        def qualify(left, right, strict, count):
            observation = metrics(left, right)
            adapter._grasp_contact_ready = lambda **kwargs: (strict, {'contact_metrics': observation})
            state = {}
            for _ in range(count):
                task.step_counter += 1
                ready, detail = adapter._close_until_contact_ready(
                    state=state, task=task, robot_name='left', spec=spec, tracked_objects={},
                    close_elapsed_steps=200, gripper_openness=0.)
            return ready, detail

        self.assertFalse(qualify(.9, 6., True, 120)[0])
        self.assertFalse(qualify(1., 6., False, 120)[0])
        self.assertFalse(qualify(1., 40.1, True, 120)[0])
        self.assertFalse(qualify(1., 6., True, 95)[0])
        ready, detail = qualify(1., 40., True, 96)
        self.assertTrue(ready)
        self.assertEqual(detail['required_stable_steps'], 96)


if __name__ == '__main__':
    unittest.main()
