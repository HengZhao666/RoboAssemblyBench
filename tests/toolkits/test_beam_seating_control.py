"""Real-physics-clock contact regressions; no simulator or synthetic success gate."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from toolkits.factory_dual_franka_assembly.beam_seating_control import seating_decision, pose_motion_window
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter

SPEC = dict(seat_payload_mass=.25, seat_contact_force=.1, seat_contact_wait=.2,
            seat_force_deadband=.25, seat_approach_speed=.0015, seat_near_speed=.00015,
            seat_slow_gap=.00075, seat_load_speed=.0001, seat_up_speed=.0005,
            seat_admittance=.00015, seat_max_linear_speed=.002, seat_max_angular_speed=.03,
            seat_max_travel=.008, seat_max_unload_travel=.001,
            seat_motion_window=.1, seat_max_position_excursion=.0003,
            seat_max_orientation_excursion=.005, seat_raw_linear_speed_limit=.05,
            seat_raw_angular_speed_limit=.5)


def measured_sample(index, dt=1/240, epoch=0, task_step=100, physics_time=None, **changes):
    sample = dict(step=task_step, physics_stamp_valid=True, physics_epoch=epoch,
                  physics_step_index=index, physics_time=index*dt if physics_time is None else physics_time,
                  physics_dt=dt, valid=True, motion_valid=True, vertical_force=0.,
                  bottom_gap=.004, linear_speed=0., angular_speed=0.,
                  position=[0., 0, 1], orientation=[1., 0, 0, 0])
    sample.update(changes)
    return sample


def warm_state(index=100, dt=1/240, pose_at=None):
    state = {}
    first = index - int(round(SPEC['seat_motion_window']/dt))
    for i in range(first, index):
        changes = {} if pose_at is None else dict(position=pose_at(i))
        pose_motion_window(state, measured_sample(i, dt=dt, **changes), dt, SPEC['seat_motion_window'])
    return state


class SeatingControl(unittest.TestCase):
    def setUp(self):
        self.dt = 1/240
        self.state = warm_state()
        self.sample = measured_sample(100)

    def decide(self, index=100, z=1.1, **changes):
        return seating_decision(self.state, measured_sample(index, dt=self.dt, **changes), z, self.dt, SPEC)

    def test_near_table_rate_is_ten_times_slower_and_dt_scaled(self):
        far = self.decide()
        near = self.decide(index=101, bottom_gap=.0005)
        self.assertAlmostEqual(far['delta_z'], 10*near['delta_z'])
        double_dt = 2*self.dt
        twice = seating_decision(warm_state(dt=double_dt), measured_sample(100, dt=double_dt),
                                 1.1, double_dt, SPEC)
        self.assertAlmostEqual(twice['delta_z'], 2*far['delta_z'])

    def test_first_contact_latches_and_does_not_chase_force_during_wait(self):
        first = self.decide(vertical_force=.12)
        self.assertEqual(first['mode'], 'first_contact_wait')
        self.assertEqual(first['delta_z'], 0.)
        # A temporarily zero sensor reading must not restart free-space descent.
        for index in range(101, 148):
            self.assertEqual(self.decide(index=index)['delta_z'], 0.)
        later = self.decide(index=148)
        self.assertEqual(later['mode'], 'load')
        self.assertLessEqual(abs(later['delta_z']), SPEC['seat_load_speed']*self.dt)

    def test_upward_unload_preempts_wait_and_motion_without_raising_hard_stop(self):
        result = self.decide(vertical_force=6., angular_speed=.3)
        self.assertEqual(result['mode'], 'unload')
        self.assertGreater(result['delta_z'], 0.)
        self.assertLessEqual(result['delta_z'], SPEC['seat_up_speed']*self.dt)
        result = self.decide(index=101, vertical_force=12.0493)
        self.assertEqual(result['failure'], 'seat_overload_or_penetration')
        self.assertEqual(result['delta_z'], 0.)

    def test_unsettled_object_cannot_receive_additional_downward_command(self):
        for changes in [dict(linear_speed=.06), dict(angular_speed=.6)]:
            self.state.clear()
            self.assertEqual(self.decide(**changes)['mode'], 'motion_wait')
        self.state.clear()
        self.decide(vertical_force=1.)
        for index in range(101, 160):
            self.decide(index=index, vertical_force=1.)
        result = self.decide(index=160, vertical_force=1., angular_speed=.6)
        self.assertEqual(result['delta_z'], 0.)

    def test_holding_weight_is_a_hold_not_an_automatic_success(self):
        self.decide(vertical_force=2.45)
        for index in range(101, 160):
            self.decide(index=index, vertical_force=2.45)
        result = self.decide(index=160, vertical_force=2.45)
        self.assertEqual(result['mode'], 'force_band_hold')
        self.assertEqual(result['delta_z'], 0.)
        self.assertNotIn('success', result)

    def test_same_physics_step_cannot_accumulate_a_second_command(self):
        self.assertLess(self.decide()['delta_z'], 0.)
        self.assertEqual(self.decide(task_step=999)['mode'], 'already_sampled')
        self.assertEqual(self.decide(task_step=1000)['delta_z'], 0.)

    def test_task_label_can_stay_constant_while_physics_advances(self):
        self.state.clear()
        for index in range(100, 124):
            self.assertEqual(self.decide(index=index, task_step=7)['delta_z'], 0.)
        result = self.decide(index=124, task_step=7)
        self.assertLess(result['delta_z'], 0.)
        self.assertAlmostEqual(result['pose_motion']['duration'], .1)

    def test_task_label_changes_cannot_complete_wait_or_motion_window(self):
        self.state.clear()
        self.decide(vertical_force=.12, task_step=1)
        for label in range(2, 200):
            result = self.decide(task_step=label, vertical_force=.12)
            self.assertEqual(result['mode'], 'already_sampled')
        self.assertEqual(len(self.state['motion_history']), 1)
        self.assertEqual(self.state['contact_time'], 100*self.dt)

    def test_v155_stationary_pose_with_solver_velocity_does_not_deadlock(self):
        result = self.decide(linear_speed=.0223, angular_speed=.1354, position=[0., 0, 1.000003])
        self.assertEqual(result['mode'], 'approach')
        self.assertLess(result['pose_motion']['linear_speed'], .0001)

    def test_window_warms_up_before_motion_and_missing_pose_fails(self):
        self.state.clear()
        self.assertEqual(self.decide()['mode'], 'motion_wait')
        for index in range(101, 124):
            self.assertEqual(self.decide(index=index)['delta_z'], 0.)
        self.assertLess(self.decide(index=124)['delta_z'], 0.)
        self.assertEqual(self.decide(index=125, position=None)['failure'], 'seat_pose_probe_invalid')
        result = self.decide(index=126)
        self.assertEqual(result['mode'], 'clock_rewarm')
        self.assertFalse(result['pose_motion']['ready'])

    def test_actual_drift_and_returning_oscillation_both_pause_descent(self):
        result = self.decide(position=[.001, 0, 1])
        self.assertEqual(result['mode'], 'motion_wait')
        self.state = warm_state(pose_at=lambda i: [.001 if i == 88 else 0., 0, 1])
        result = self.decide()
        self.assertEqual(result['pose_motion']['linear_speed'], 0.)
        self.assertEqual(result['mode'], 'motion_wait')
        self.assertGreater(result['pose_motion']['position_excursion'], .0003)

    def test_invalid_observations_dt_and_penetration_fail_closed(self):
        for changes in [dict(valid=False), dict(motion_valid=False),
                        dict(vertical_force=float('nan')), dict(linear_speed=float('inf'))]:
            self.assertEqual(self.decide(**changes)['failure'], 'seat_contact_probe_invalid')
        for dt in [0., -1., float('nan')]:
            self.assertEqual(seating_decision({}, self.sample, 1., dt, SPEC)['failure'],
                             'seat_invalid_control_dt')
        self.assertEqual(self.decide(bottom_gap=-.00101)['failure'], 'seat_overload_or_penetration')

    def test_missing_or_inconsistent_physics_clock_has_no_task_fallback(self):
        for field in ['physics_stamp_valid', 'physics_epoch', 'physics_step_index',
                      'physics_time', 'physics_dt']:
            sample = dict(self.sample)
            sample.pop(field)
            result = seating_decision({}, sample, 1.1, self.dt, SPEC)
            self.assertEqual(result['failure'], 'seat_invalid_physics_stamp', field)
            self.assertEqual(result['delta_z'], 0.)
        self.decide()
        result = self.decide(physics_time=100*self.dt+.001)
        self.assertEqual(result['failure'], 'seat_invalid_physics_stamp')
        result = seating_decision({}, self.sample, 1.1, 2*self.dt, SPEC)
        self.assertEqual(result['failure'], 'seat_invalid_physics_stamp')

    def test_gap_and_time_jump_rewarm_before_descent(self):
        self.decide()
        for sample in [measured_sample(102), measured_sample(101, physics_time=.9)]:
            state = copy.deepcopy(self.state)
            result = seating_decision(state, sample, 1.1, self.dt, SPEC)
            self.assertEqual(result['mode'], 'clock_rewarm')
            self.assertEqual(result['delta_z'], 0.)
            self.assertEqual(len(state['motion_history']), 1)
        result = self.decide(index=102)
        for index in range(103, 126):
            self.assertEqual(self.decide(index=index)['delta_z'], 0.)
        self.assertLess(self.decide(index=126)['delta_z'], 0.)

    def test_reset_keeps_displacement_budgets_and_contact_latch(self):
        self.decide(vertical_force=1.)
        original_start = self.state['start_z']
        original_contact = self.state['contact_z']
        result = self.decide(index=0, epoch=1, z=1.099, vertical_force=0.)
        self.assertEqual(result['mode'], 'clock_rewarm')
        self.assertEqual(self.state['start_z'], original_start)
        self.assertEqual(self.state['contact_z'], original_contact)
        self.assertEqual(self.state['contact_epoch'], 1)
        self.assertEqual(self.state['contact_time'], 0.)
        for index in range(1, 48):
            self.assertEqual(self.decide(index=index, epoch=1, z=1.099)['delta_z'], 0.)
        self.assertEqual(self.decide(index=48, epoch=1, z=1.099)['mode'], 'load')
        self.assertEqual(self.decide(index=49, epoch=1, z=1.101, vertical_force=6.)['failure'],
                         'seat_unload_travel_exhausted')

    def test_index_rollback_without_epoch_is_also_conservative(self):
        self.decide()
        result = self.decide(index=1)
        self.assertEqual(result['mode'], 'clock_rewarm')
        self.assertEqual(result['delta_z'], 0.)
        self.assertFalse(result['pose_motion']['ready'])

    def test_physics_dt_change_rewarms_instead_of_reinterpreting_old_history(self):
        self.decide(vertical_force=1.)
        changed_dt = 2*self.dt
        sample = measured_sample(101, dt=changed_dt, physics_time=100*self.dt+changed_dt)
        result = seating_decision(self.state, sample, 1.1, changed_dt, SPEC)
        self.assertEqual(result['mode'], 'clock_rewarm')
        self.assertEqual(result['delta_z'], 0.)
        self.assertFalse(result['pose_motion']['ready'])
        self.assertEqual(self.state['contact_time'], sample['physics_time'])

    def test_wait_and_motion_use_elapsed_seconds_at_multiple_physics_rates(self):
        for dt in [1/120, 1/240, 1/480]:
            state = {}
            start = 10
            contact = seating_decision(state, measured_sample(start, dt=dt, vertical_force=.12),
                                       1.1, dt, SPEC)
            self.assertEqual(contact['mode'], 'first_contact_wait')
            wait_steps = round(SPEC['seat_contact_wait']/dt)
            for offset in range(1, wait_steps):
                result = seating_decision(state, measured_sample(start+offset, dt=dt), 1.1, dt, SPEC)
                self.assertEqual(result['mode'], 'first_contact_wait')
            result = seating_decision(state, measured_sample(start+wait_steps, dt=dt), 1.1, dt, SPEC)
            self.assertEqual(result['mode'], 'load')
            self.assertAlmostEqual(result['delta_z'], -SPEC['seat_load_speed']*dt)
            motion_state = {}
            for offset in range(round(SPEC['seat_motion_window']/dt)):
                motion = pose_motion_window(motion_state, measured_sample(start+offset, dt=dt),
                                            dt, SPEC['seat_motion_window'])
                self.assertFalse(motion['ready'])
            motion = pose_motion_window(motion_state,
                measured_sample(start+round(SPEC['seat_motion_window']/dt), dt=dt), dt, SPEC['seat_motion_window'])
            self.assertTrue(motion['ready'])
            self.assertAlmostEqual(motion['duration'], SPEC['seat_motion_window'])

    def test_pose_window_copies_buffers_and_ignores_duplicate_task_labels(self):
        state = {}
        sample = measured_sample(0)
        position = np.array(sample['position'])
        sample['position'] = position
        first = pose_motion_window(state, sample, self.dt, .1)
        self.assertFalse(first['ready'])
        position[0] = 4.
        self.assertEqual(state['motion_history'][0][1][0], 0.)
        sample['step'] = 9
        pose_motion_window(state, sample, self.dt, .1)
        self.assertEqual(len(state['motion_history']), 1)

    def test_both_downward_search_and_upward_unload_are_bounded(self):
        self.decide()
        self.assertEqual(self.decide(z=1.092, index=101)['failure'], 'seat_travel_exhausted')
        self.state.clear()
        self.decide(vertical_force=1.)
        self.assertEqual(self.decide(z=1.101, index=101, vertical_force=6.)['failure'],
                         'seat_unload_travel_exhausted')

    def test_paused_ik_does_not_accumulate_an_unpublished_vertical_target(self):
        saved = {'pose': {'position': np.array([.5, -.2, 1.1]), 'orientation': np.array([1., 0, 0, 0])}}
        task = SimpleNamespace(step_counter=100, phase='seat',
            _ur5e_contact_hold_commands={'right': saved}, _beam_table_observation=lambda: self.sample,
            _contact_physics_dt=lambda: self.dt)
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        targets = []
        # Model a tracking guard retaining the last accepted command.
        adapter._grasp_drive_motion = lambda **kw: targets.append(copy.deepcopy(kw['target_pose'])) or {}
        adapter._hold_previous_grasp_command = lambda **kw: {}
        original = saved['pose']['position'].copy()
        for index in range(100, 127):
            self.sample = measured_sample(index)
            task.step_counter = index
            adapter._beam_table_seat_action(phase_key='seat', task=task, robot_name='right',
                spec=SPEC, tracked_robots={}, tracked_objects={})
        self.assertEqual(len(targets), 3)
        np.testing.assert_array_equal(saved['pose']['position'], original)
        for target in targets:
            np.testing.assert_allclose(target['position'], targets[0]['position'], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
