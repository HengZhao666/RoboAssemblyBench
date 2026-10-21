"""Counterexamples for physical-time pose qualification; no simulator imports."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from toolkits.factory_dual_franka_assembly.beam_pose_stability import PoseStabilityTracker, quaternion_angle


DT = .02
BODY = '/World/env_0/objects/beam_6'


def sample(index, *, dt=DT, epoch=0, time=None, position=None, orientation=None, **changes):
    result = dict(valid=True, motion_valid=True, physics_stamp_valid=True,
                  physics_epoch=epoch, physics_step_index=index,
                  physics_time=index*dt if time is None else time, physics_dt=dt,
                  position=[0., 0., 1.] if position is None else position,
                  orientation=[1., 0., 0., 0.] if orientation is None else orientation)
    result.update(changes)
    return result


def observe(tracker, index, *, com_position=None, body_id=BODY, view_id=1,
            eligible=False, **changes):
    point = sample(index, **changes)
    com = [point['position'][0], point['position'][1], point['position'][2]+.006321] if com_position is None else com_position
    return tracker.observe(point, com_position=com, body_id=body_id,
                           view_id=view_id, anchor_eligible=eligible)


def warm(tracker, *, eligible=False):
    for index in range(101):
        result = observe(tracker, index, eligible=eligible)
    return result


class PoseStability(unittest.TestCase):
    def test_defaults_require_97_short_and_481_long_physical_samples(self):
        tracker = PoseStabilityTracker()
        for index in range(96):
            result = observe(tracker, index, dt=1/240, eligible=True)
        self.assertFalse(result['short_window']['full'])
        result = observe(tracker, 96, dt=1/240, eligible=True)
        self.assertTrue(result['short_window']['ready'])
        self.assertEqual(result['short_window']['samples'], 97)
        for index in range(97, 480):
            result = observe(tracker, index, dt=1/240, eligible=True)
        self.assertFalse(result['ready'])
        self.assertAlmostEqual(result['long_window']['duration'], 479/240)
        result = observe(tracker, 480, dt=1/240, eligible=True)
        self.assertTrue(result['ready'])
        self.assertEqual(result['long_window']['samples'], 481)

    def test_round_trip_motion_cannot_hide_behind_equal_endpoints(self):
        tracker = PoseStabilityTracker()
        # 60 um oscillation: bounded excursion and zero endpoint drift, but
        # repeated travel exceeds the permitted 2 mm/s path rate.
        for index in range(501):
            x = .00006*math.sin(2*math.pi*10*index/240)
            result = observe(tracker, index, dt=1/240, position=[x, 0., 1.])
        motion = result['short_window']
        self.assertLess(motion['metrics']['com_endpoint_drift_speed'], .002)
        self.assertLess(motion['metrics']['com_diameter_upper_bound'], .0003)
        self.assertGreater(motion['metrics']['com_path_speed'], .002)
        self.assertFalse(result['ready'])

    def test_single_step_jump_is_rejected_even_with_small_diameter_and_path(self):
        tracker = PoseStabilityTracker()
        warm(tracker)
        observe(tracker, 101, position=[.0001, 0., 1.])
        result = observe(tracker, 102)
        motion = result['short_window']
        self.assertLess(motion['metrics']['com_diameter_upper_bound'], .0003)
        self.assertLess(motion['metrics']['com_path_speed'], .002)
        self.assertEqual(motion['metrics']['com_endpoint_drift_speed'], 0.)
        self.assertGreater(motion['metrics']['com_max_step_speed'], .002)
        self.assertFalse(result['ready'])

    def test_long_window_rejects_drift_that_every_short_window_accepts(self):
        tracker = PoseStabilityTracker()
        for index in range(151):
            result = observe(tracker, index, position=[.0001*index*DT, 0., 1.])
        self.assertTrue(result['short_window']['ready'])
        self.assertFalse(result['long_window']['ready'])
        self.assertGreater(result['long_window']['metrics']['com_diameter_upper_bound'], .0003)
        self.assertFalse(result['ready'])

    def test_fixed_anchor_catches_even_slower_unbounded_creep(self):
        tracker = PoseStabilityTracker()
        initial = warm(tracker, eligible=True)
        self.assertTrue(initial['anchor']['captured_now'])
        for index in range(101, 422):
            result = observe(tracker, index, position=[.00005*(index-100)*DT, 0., 1.], eligible=True)
        self.assertTrue(result['motion_windows_ready'])
        self.assertGreater(result['anchor']['com_displacement'], .0003)
        self.assertFalse(result['ready'])
        self.assertEqual(result['reason'], 'cumulative_anchor_budget_exceeded')
        # Returning to the old pose cannot erase a previously observed overrun.
        for index in range(422, 523):
            result = observe(tracker, index, eligible=True)
        self.assertTrue(result['motion_windows_ready'])
        self.assertTrue(result['latched_failure'])
        self.assertFalse(result['ready'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_normal_initial_descent_does_not_use_phase_entry_as_anchor(self):
        tracker = PoseStabilityTracker()
        for index in range(176):
            x = .0035*(1-index/175)
            result = observe(tracker, index, position=[x, 0., 1.], eligible=True)
            self.assertFalse(result['anchor']['captured'])
            self.assertFalse(result['latched_failure'])
        for index in range(176, 277):
            result = observe(tracker, index, eligible=True)
        self.assertTrue(result['ready'])
        self.assertTrue(result['anchor']['captured'])
        self.assertGreaterEqual(result['anchor']['physics_step_index'], 175)
        self.assertEqual(result['anchor']['actor_position'], [0., 0., 1.])

    def test_eligibility_controls_first_anchor_without_reading_force(self):
        tracker = PoseStabilityTracker()
        result = warm(tracker)
        self.assertTrue(result['motion_windows_ready'])
        self.assertFalse(result['ready'])
        self.assertFalse(result['anchor']['captured'])
        result = observe(tracker, 101, eligible=True)
        self.assertTrue(result['anchor']['captured_now'])
        result = observe(tracker, 102, eligible=False)
        self.assertEqual(result['anchor']['physics_step_index'], 101)
        self.assertFalse(result['anchor']['captured_now'])

    def test_suspended_stationary_body_only_proves_motion(self):
        tracker = PoseStabilityTracker()
        for index in range(101):
            result = observe(tracker, index, vertical_force=0., bottom_gap=.004)
        self.assertTrue(result['motion_windows_ready'])
        self.assertFalse(result['ready'])
        self.assertEqual(result['scope'], 'motion_only')
        self.assertFalse(result['support_verified'])
        self.assertFalse(result['anchor']['captured'])
        # The independent support predicate must still reject this sample.
        support_ready = 1.962 <= 0. <= 40. and -.0005 <= .004 <= .001
        self.assertFalse(result['ready'] and support_ready)

    def test_duplicate_calls_cannot_fill_a_physical_time_window(self):
        tracker = PoseStabilityTracker()
        observe(tracker, 0)
        for _ in range(500):
            result = observe(tracker, 0)
        self.assertFalse(result['fresh_sample'])
        self.assertEqual(result['clock_transition'], 'duplicate')
        self.assertEqual(result['long_window']['samples'], 1)
        self.assertFalse(result['ready'])

    def test_duplicate_pose_change_invalidates_windows_and_preserves_anchor(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        result = observe(tracker, 100, position=[.00001, 0., 1.])
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'], 'duplicate_pose_changed')
        self.assertEqual(result['long_window']['samples'], 0)
        self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_duplicate_can_capture_once_but_cannot_rebase_anchor(self):
        tracker = PoseStabilityTracker()
        warm(tracker)
        result = observe(tracker, 100, eligible=True)
        self.assertFalse(result['fresh_sample'])
        self.assertTrue(result['anchor']['captured_now'])
        result = observe(tracker, 100, eligible=True)
        self.assertFalse(result['anchor']['captured_now'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_clock_gap_rewarms_without_moving_anchor(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        result = observe(tracker, 103, position=[.00005, 0., 1.], eligible=True)
        self.assertEqual(result['clock_transition'], 'gap')
        self.assertEqual(result['long_window']['samples'], 1)
        self.assertFalse(result['ready'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)
        for index in range(104, 204):
            result = observe(tracker, index, position=[.00005, 0., 1.], eligible=True)
        self.assertTrue(result['ready'])
        self.assertEqual(result['anchor']['actor_position'], [0., 0., 1.])

    def test_repeated_gaps_cannot_reset_accumulated_drift_budget(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        for move in range(1, 8):
            result = observe(tracker, 100+move*2, position=[move*.00005, 0., 1.], eligible=True)
        self.assertTrue(result['latched_failure'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)
        self.assertGreater(result['anchor']['actor_displacement'], .0003)

    def test_epoch_and_view_changes_preserve_anchor_and_require_new_windows(self):
        for change in ('epoch', 'view'):
            with self.subTest(change=change):
                tracker = PoseStabilityTracker()
                warm(tracker, eligible=True)
                if change == 'epoch':
                    result = observe(tracker, 0, epoch=1, eligible=True)
                    expected = 'epoch_change'
                else:
                    result = observe(tracker, 101, view_id=2, eligible=True)
                    expected = 'view_change'
                self.assertEqual(result['clock_transition'], expected)
                self.assertEqual(result['long_window']['samples'], 1)
                self.assertFalse(result['ready'])
                self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_changing_object_is_rejected_until_explicit_new_tracker(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        result = observe(tracker, 101, body_id='/World/other')
        self.assertEqual(result['reason'], 'body_identity_changed_requires_new_tracker')
        self.assertFalse(result['ready'])
        for index in range(102, 203):
            result = observe(tracker, index, eligible=True)
        self.assertFalse(result['ready'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)
        self.assertTrue(warm(PoseStabilityTracker(), eligible=True)['ready'])

    def test_invalid_sample_or_com_rewarms_without_actor_fallback(self):
        for changes in ({'physics_stamp_valid': False}, {'motion_valid': False},
                        {'valid': False}, {'physics_dt': float('nan')},
                        {'physics_step_index': True}, {'orientation': [0., 0., 0., 0.]}):
            with self.subTest(changes=changes):
                tracker = PoseStabilityTracker()
                warm(tracker, eligible=True)
                point = sample(101)
                point.update(changes)
                result = tracker.observe(point, com_position=[0., 0., 1.006321], body_id=BODY, view_id=1)
                self.assertFalse(result['valid'])
                self.assertFalse(result['ready'])
                self.assertEqual(result['long_window']['samples'], 0)
                self.assertEqual(result['anchor']['physics_step_index'], 100)
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        result = tracker.observe(sample(101), com_position=None, body_id=BODY, view_id=1)
        self.assertFalse(result['valid'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_same_index_changed_time_is_not_a_new_motion_sample(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        result = observe(tracker, 100, time=2.01)
        self.assertEqual(result['reason'], 'inconsistent_physics_clock')
        self.assertFalse(result['valid'])
        self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_dt_change_and_time_gap_do_not_join_unobserved_motion(self):
        for changes in ({'dt': .04}, {'time': 2.03}):
            tracker = PoseStabilityTracker()
            warm(tracker, eligible=True)
            result = observe(tracker, 101, **changes)
            self.assertFalse(result['ready'])
            self.assertEqual(result['long_window']['samples'], 1)
            self.assertEqual(result['anchor']['physics_step_index'], 100)

    def test_q_sign_and_float32_size_small_rotation_are_robust(self):
        tiny = 1.e-7
        q = [math.cos(tiny/2.), math.sin(tiny/2.), 0., 0.]
        self.assertAlmostEqual(quaternion_angle(q, [1., 0., 0., 0.]), tiny, delta=1.e-15)
        self.assertEqual(quaternion_angle(q, [-v for v in q]), 0.)
        tracker = PoseStabilityTracker()
        for index in range(101):
            q = [1., 0., 0., 0.] if index % 2 == 0 else [-1., 0., 0., 0.]
            result = observe(tracker, index, orientation=q, eligible=True)
        self.assertTrue(result['ready'])
        self.assertEqual(result['long_window']['metrics']['angular_path_speed'], 0.)
        result = observe(tracker, 101, orientation=[math.cos(tiny/2.), math.sin(tiny/2.), 0., 0.])
        self.assertGreater(result['short_window']['metrics']['angular_max_step_speed'], 0.)

    def test_actor_and_com_are_both_qualified(self):
        for mover in ('actor', 'com'):
            tracker = PoseStabilityTracker()
            warm(tracker)
            if mover == 'com':
                result = observe(tracker, 101, com_position=[.0001, 0., 1.006321])
            else:
                result = observe(tracker, 101, position=[.0001, 0., 1.], com_position=[0., 0., 1.006321])
            self.assertFalse(result['ready'])
            self.assertFalse(result['short_window']['conditions'][mover+'_max_step_speed'])

    def test_angular_jump_is_rejected_without_any_translation(self):
        tracker = PoseStabilityTracker()
        warm(tracker)
        turn = .001
        q = [math.cos(turn/2.), 0., 0., math.sin(turn/2.)]
        observe(tracker, 101, orientation=q)
        result = observe(tracker, 102)
        metrics = result['short_window']['metrics']
        self.assertEqual(metrics['com_max_step_speed'], 0.)
        self.assertLess(metrics['orientation_diameter_upper_bound'], .005)
        self.assertEqual(metrics['angular_endpoint_drift_speed'], 0.)
        self.assertGreater(metrics['angular_max_step_speed'], .03)
        self.assertFalse(result['ready'])

    def test_orientation_anchor_catches_cumulative_slow_rotation(self):
        tracker = PoseStabilityTracker()
        warm(tracker, eligible=True)
        for index in range(101, 602):
            turn = .0005*(index-100)*DT
            result = observe(tracker, index, orientation=[math.cos(turn/2.), 0., 0., math.sin(turn/2.)], eligible=True)
        self.assertTrue(result['motion_windows_ready'])
        self.assertGreater(result['anchor']['orientation_displacement'], .005)
        self.assertEqual(result['anchor']['actor_displacement'], 0.)
        self.assertEqual(result['reason'], 'cumulative_anchor_budget_exceeded')
        self.assertFalse(result['ready'])

    def test_input_and_output_buffers_are_owned_and_jsonable(self):
        tracker = PoseStabilityTracker()
        point, com = sample(0), [0., 0., 1.006321]
        tracker.observe(point, com_position=com, body_id=BODY, view_id=1)
        point['position'][0] = .01
        com[0] = .01
        for index in range(1, 101):
            result = observe(tracker, index, eligible=True)
        self.assertTrue(result['ready'])
        saved = copy.deepcopy(result)
        result['anchor']['actor_position'][0] = 999.
        result['thresholds']['position_budget'] = 999.
        result = observe(tracker, 101, eligible=True)
        self.assertEqual(result['anchor']['actor_position'][0], 0.)
        self.assertEqual(result['thresholds']['position_budget'], .0003)
        json.dumps(saved, allow_nan=False)
        json.dumps(result, allow_nan=False)

    def test_invalid_configuration_is_rejected(self):
        for kwargs in ({'short_window': 0.}, {'long_window': .1},
                       {'position_budget': float('inf')}, {'linear_speed_limit': True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PoseStabilityTracker(**kwargs)


if __name__ == '__main__':
    unittest.main()
