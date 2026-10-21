"""The diagnostic hold keeps clock, publication and safety boundaries explicit."""
import copy
import unittest
from types import SimpleNamespace as NS

import numpy as np

from toolkits.factory_dual_franka_assembly.beam_loaded_target_hold import (
    capture_loaded_native_targets, loaded_target_hold_decision,
)


def sample(index, **changes):
    result = dict(physics_epoch=0, physics_step_index=index,
                  physics_time=index / 240., physics_dt=1 / 240.,
                  physics_stamp_valid=True, valid=True, motion_valid=True,
                  physics_handle_valid=True, vertical_force=2.0, bottom_gap=0.0,
                  linear_speed=.3, angular_speed=5.)
    result.update(changes)
    return result


class LoadedHoldClock(unittest.TestCase):
    def qualify(self, state, duration=.6):
        modes = [loaded_target_hold_decision(state, sample(i), duration)['mode'] for i in range(24)]
        self.assertEqual(modes, ['qualifying'] * 23 + ['capture'])

    def test_default_is_disabled_and_does_not_read_or_mutate_state(self):
        state = {}
        self.assertEqual(loaded_target_hold_decision(state, {}, 0)['mode'], 'disabled')
        self.assertEqual(state, {})

    def test_captures_once_even_when_original_speed_gate_fails(self):
        state = {}
        self.qualify(state)
        duplicate = loaded_target_hold_decision(state, sample(23), .6)
        self.assertEqual(duplicate['mode'], 'holding')
        self.assertEqual(duplicate['elapsed'], 0)
        for index in range(24, 167):
            self.assertEqual(loaded_target_hold_decision(state, sample(index), .6)['mode'], 'holding')
        done = loaded_target_hold_decision(state, sample(167), .6)
        self.assertEqual(done['mode'], 'complete')
        self.assertAlmostEqual(done['elapsed'], .6)
        self.assertEqual(loaded_target_hold_decision(state, sample(2000), .6)['mode'], 'complete')

    def test_duplicate_cannot_qualify_or_task_label_create_a_new_sample(self):
        state = {}
        for label in range(100):
            result = loaded_target_hold_decision(state, sample(0, step=label), .6)
        self.assertEqual(result['qualified_steps'], 1)
        self.assertEqual(result['mode'], 'qualifying')

    def test_load_or_gap_failure_restarts_qualification(self):
        for changes in (dict(vertical_force=1.961), dict(bottom_gap=.00101),
                        dict(bottom_gap=-.00051)):
            with self.subTest(changes=changes):
                state = {}
                for i in range(23):
                    loaded_target_hold_decision(state, sample(i), .6)
                result = loaded_target_hold_decision(state, sample(23, **changes), .6)
                self.assertEqual(result['qualified_steps'], 0)
                self.assertEqual(loaded_target_hold_decision(state, sample(24), .6)['qualified_steps'], 1)

    def test_clock_discontinuity_requalifies_before_capture_but_fails_after(self):
        discontinuities = (sample(25), sample(24, physics_epoch=1),
                           sample(24, physics_time=3.), sample(24, physics_dt=1 / 120.))
        for changed in discontinuities:
            with self.subTest(changed=changed):
                state = {}
                for i in range(23):
                    loaded_target_hold_decision(state, sample(i), .6)
                result = loaded_target_hold_decision(state, changed, .6)
                self.assertEqual(result['mode'], 'qualifying')
                self.assertEqual(result['qualified_steps'], 1)
                captured = {}
                self.qualify(captured)
                result = loaded_target_hold_decision(captured, changed, .6)
                self.assertEqual(result['mode'], 'failure')
                self.assertEqual(result['reason'], 'hold_clock_discontinuity')
                self.assertEqual(loaded_target_hold_decision(captured, sample(26), .6)['mode'], 'failure')

    def test_missing_clock_handle_or_probe_is_failure_and_cannot_recapture(self):
        for field in ('physics_epoch', 'physics_step_index', 'physics_time', 'physics_dt',
                      'physics_stamp_valid', 'physics_handle_valid', 'motion_valid',
                      'valid', 'vertical_force', 'bottom_gap'):
            with self.subTest(field=field):
                state = {}
                self.qualify(state)
                changed = sample(24)
                changed.pop(field)
                self.assertEqual(loaded_target_hold_decision(state, changed, .6)['mode'], 'failure')

    def test_hard_overload_and_penetration_take_precedence_during_hold(self):
        for changes in (dict(vertical_force=10.001), dict(bottom_gap=-.001001),
                        dict(vertical_force=float('nan'))):
            state = {}
            self.qualify(state)
            self.assertEqual(loaded_target_hold_decision(state, sample(24, **changes), .6)['mode'], 'failure')

    def test_load_change_is_observed_without_moving_or_requalifying_target(self):
        state = {}
        self.qualify(state)
        result = loaded_target_hold_decision(state, sample(24, vertical_force=.2, bottom_gap=.002), .6)
        self.assertEqual(result['mode'], 'holding')
        self.assertEqual(result['qualified_steps'], 24)

    def test_same_identity_cannot_change_clock_and_duration_is_bounded(self):
        state = {}
        loaded_target_hold_decision(state, sample(0), .6)
        self.assertEqual(loaded_target_hold_decision(state, sample(0, physics_time=.1), .6)['mode'], 'failure')
        for duration in (-.1, 2.001, float('nan'), float('inf'), None, 'bad'):
            self.assertEqual(loaded_target_hold_decision({}, sample(0), duration)['mode'], 'failure')
        state = {}
        loaded_target_hold_decision(state, sample(0), .6)
        self.assertEqual(loaded_target_hold_decision(state, sample(1), 0)['mode'], 'failure')

    def test_copies_clock_observation_buffers(self):
        state = {}
        current = sample(0)
        loaded_target_hold_decision(state, current, .6)
        current['physics_dt'] = .5
        self.assertEqual(loaded_target_hold_decision(state, sample(1), .6)['qualified_steps'], 2)


class NativeTargets:
    def __init__(self):
        self.position = np.array([[.4, .62, -.7, 1.3, -2., .1, .3, 0.]])
        self.velocity = np.zeros_like(self.position)
        self.effort = np.zeros_like(self.position)

    def get_dof_position_targets(self):
        return self.position

    def get_dof_velocity_targets(self):
        return self.velocity

    def get_dof_actuation_forces(self):
        return self.effort


class NativeCapture(unittest.TestCase):
    def setUp(self):
        self.native = NativeTargets()
        self.indices = np.array([2, 0, 5, 4, 6, 3])
        self.names = ['arm_' + str(i) for i in range(6)]
        mapping = dict(zip(self.names, self.indices))
        mapping['finger_joint'] = 1
        self.view = NS(is_physics_handle_valid=lambda: True,
                       get_joint_positions=lambda clone: np.zeros((1, 8)),
                       _physics_view=self.native)
        self.subset = NS(joint_indices=self.indices)
        self.controller = NS(joint_names=self.names, get_joint_subset=lambda: self.subset)
        self.articulation = NS(_articulation_view=self.view, get_dof_index=lambda name: mapping[name])
        self.robot = NS(articulation=self.articulation,
                        controllers={'arm_joint_controller': self.controller},
                        config=NS(gripper_dof_name='finger_joint'))
        self.saved = dict(arm_q=self.native.position[0, self.indices].copy(),
                          gripper_openness=1 - .62 / .8)

    def capture(self):
        return capture_loaded_native_targets(self.robot, self.saved, 0., .8, .08)

    def test_capture_maps_named_dofs_and_copies_actual_native_targets(self):
        before = copy.deepcopy(self.saved)
        result = self.capture()
        self.assertTrue(result['valid'], result)
        np.testing.assert_array_equal(result['arm_q'], self.native.position[0, self.indices])
        self.assertEqual(result['gripper_index'], 1)
        self.assertAlmostEqual(result['gripper_q'], .62)
        self.native.position[:] = 99
        self.assertEqual(result['full_native_targets']['position_targets'][0][1], .62)
        np.testing.assert_array_equal(self.saved['arm_q'], before['arm_q'])
        self.assertEqual(self.saved['gripper_openness'], before['gripper_openness'])

    def test_rejects_generated_cache_that_was_not_published(self):
        self.saved['arm_q'][2] += .001
        self.assertFalse(self.capture()['valid'])
        self.saved['arm_q'] = self.native.position[0, self.indices].copy()
        self.saved['gripper_openness'] += .001
        self.assertFalse(self.capture()['valid'])

    def test_rejects_wrong_index_order_duplicate_or_missing_handle(self):
        self.subset.joint_indices = self.indices[::-1]
        self.assertFalse(self.capture()['valid'])
        self.subset.joint_indices = self.indices
        self.view.is_physics_handle_valid = lambda: False
        self.assertFalse(self.capture()['valid'])

    def test_invalid_named_index_is_rejected_instead_of_truncated(self):
        original = self.articulation.get_dof_index
        self.articulation.get_dof_index = lambda name: (2.1 if name == self.names[0] else original(name))
        self.assertFalse(self.capture()['valid'])

    def test_rejects_nonzero_velocity_or_effort_for_controlled_joints(self):
        for field in ('velocity', 'effort'):
            with self.subTest(field=field):
                getattr(self.native, field)[0, 1] = .01
                self.assertFalse(self.capture()['valid'])
                getattr(self.native, field)[:] = 0
        self.native.effort[0, 7] = 3  # An untouched mimic DOF is evidence, not commanded by this hold.
        self.assertTrue(self.capture()['valid'])

    def test_rejects_missing_shape_and_nonfinite_native_target_buffers(self):
        self.native.position[0, 1] = float('nan')
        self.assertFalse(self.capture()['valid'])
        self.native.position[0, 1] = .62
        self.native.velocity = np.zeros((1, 7))
        self.assertFalse(self.capture()['valid'])
        self.native.velocity = np.zeros((1, 8))
        self.native.get_dof_actuation_forces = None
        self.assertFalse(self.capture()['valid'])

    def test_rejects_openness_outside_range_without_clipping(self):
        for target in (.8, -.001):
            self.native.position[0, 1] = target
            self.saved['gripper_openness'] = 1 - target / .8
            self.assertFalse(self.capture()['valid'])
        self.assertFalse(capture_loaded_native_targets(self.robot, self.saved, .8, .8, .08)['valid'])

    def test_float32_endpoint_is_captured_without_clipping_the_readback(self):
        self.native.position = self.native.position.astype(np.float32)
        self.native.position[0, 1] = .8 * (1 - .08)
        self.saved['gripper_openness'] = .08
        result = self.capture()
        self.assertTrue(result['valid'], result)
        self.assertEqual(result['gripper_q'], float(self.native.position[0, 1]))
        self.assertAlmostEqual(result['gripper_openness'], 1 - float(self.native.position[0, 1]) / .8)


if __name__ == '__main__':
    unittest.main()
