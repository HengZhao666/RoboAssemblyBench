"""Contact response regressions; no simulator or synthetic success gate."""
import copy
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from toolkits.factory_dual_franka_assembly.beam_seating_control import seating_decision
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter

SPEC = dict(seat_payload_mass=.25, seat_contact_force=.1, seat_contact_wait=.2,
            seat_force_deadband=.25, seat_approach_speed=.0015, seat_near_speed=.00015,
            seat_slow_gap=.00075, seat_load_speed=.0001, seat_up_speed=.0005,
            seat_admittance=.00015, seat_max_linear_speed=.002, seat_max_angular_speed=.03,
            seat_max_travel=.008, seat_max_unload_travel=.001,
            seat_motion_window=.1, seat_max_position_excursion=.0003,
            seat_max_orientation_excursion=.005, seat_raw_linear_speed_limit=.05,
            seat_raw_angular_speed_limit=.5)


class SeatingControl(unittest.TestCase):
    def setUp(self):
        self.state = {'motion_history':[(76,np.array([0.,0,1]),np.array([1.,0,0,0]))]}
        self.sample = dict(step=100, valid=True, motion_valid=True, vertical_force=0.,
                           bottom_gap=.004, linear_speed=0., angular_speed=0.,
                           position=[0.,0,1], orientation=[1.,0,0,0])
        self.dt = 1/240

    def decide(self, z=1.1, **changes):
        return seating_decision(self.state, {**self.sample, **changes}, z, self.dt, SPEC)

    def test_near_table_rate_is_ten_times_slower_and_dt_scaled(self):
        far = self.decide()
        near = self.decide(step=101, bottom_gap=.0005)
        self.assertAlmostEqual(far['delta_z'], 10*near['delta_z'])
        twice = seating_decision({'motion_history':[(76,np.array([0.,0,1]),np.array([1.,0,0,0]))]}, self.sample, 1.1, 2*self.dt, SPEC)
        self.assertAlmostEqual(twice['delta_z'], 2*far['delta_z'])

    def test_first_contact_latches_and_does_not_chase_force_during_wait(self):
        first = self.decide(vertical_force=.12)
        self.assertEqual(first['mode'], 'first_contact_wait')
        self.assertEqual(first['delta_z'], 0.)
        # A temporarily zero sensor reading must not restart free-space descent.
        for step in range(101,148):
            self.assertEqual(self.decide(step=step)['delta_z'], 0.)
        later = self.decide(step=148)
        self.assertEqual(later['mode'], 'load')
        self.assertLessEqual(abs(later['delta_z']), SPEC['seat_load_speed']*self.dt)

    def test_upward_unload_preempts_wait_and_motion_without_raising_hard_stop(self):
        result = self.decide(vertical_force=6., angular_speed=.3)
        self.assertEqual(result['mode'], 'unload')
        self.assertGreater(result['delta_z'], 0.)
        self.assertLessEqual(result['delta_z'], SPEC['seat_up_speed']*self.dt)
        result = self.decide(step=101, vertical_force=12.0493)
        self.assertEqual(result['failure'], 'seat_overload_or_penetration')
        self.assertEqual(result['delta_z'], 0.)

    def test_unsettled_object_cannot_receive_additional_downward_command(self):
        for changes in [dict(linear_speed=.06), dict(angular_speed=.6)]:
            self.state.clear()
            self.assertEqual(self.decide(**changes)['mode'], 'motion_wait')
        self.state.clear(); self.decide(vertical_force=1.)
        result = self.decide(step=160, vertical_force=1., angular_speed=.6)
        self.assertEqual(result['delta_z'], 0.)

    def test_holding_weight_is_a_hold_not_an_automatic_success(self):
        result = self.decide(vertical_force=2.45)
        result = self.decide(step=160, vertical_force=2.45)
        self.assertEqual(result['mode'], 'force_band_hold')
        self.assertEqual(result['delta_z'], 0.)
        self.assertNotIn('success', result)

    def test_same_physics_step_cannot_accumulate_a_second_command(self):
        self.assertLess(self.decide()['delta_z'], 0.)
        self.assertEqual(self.decide()['delta_z'], 0.)

    def test_v155_stationary_pose_with_solver_velocity_does_not_deadlock(self):
        result = self.decide(linear_speed=.0223, angular_speed=.1354,
                             position=[0.,0,1.000003])
        self.assertEqual(result['mode'], 'approach')
        self.assertLess(result['pose_motion']['linear_speed'], .0001)

    def test_window_warms_up_before_motion_and_missing_pose_fails(self):
        self.state.clear()
        self.assertEqual(self.decide()['mode'], 'motion_wait')
        for step in range(101,124):
            self.assertEqual(self.decide(step=step)['delta_z'], 0.)
        self.assertLess(self.decide(step=124)['delta_z'], 0.)
        self.assertEqual(self.decide(step=125, position=None)['failure'], 'seat_pose_probe_invalid')

    def test_actual_drift_and_returning_oscillation_both_pause_descent(self):
        result = self.decide(position=[.001,0,1])
        self.assertEqual(result['mode'], 'motion_wait')
        self.state={'motion_history':[(76,np.array([0.,0,1]),np.array([1.,0,0,0])),
                                      (88,np.array([.001,0,1]),np.array([1.,0,0,0]))]}
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

    def test_both_downward_search_and_upward_unload_are_bounded(self):
        self.decide()
        self.assertEqual(self.decide(z=1.092, step=101)['failure'], 'seat_travel_exhausted')
        self.state.clear(); self.decide(vertical_force=1.)
        self.assertEqual(self.decide(z=1.101, step=101, vertical_force=6.)['failure'],
                         'seat_unload_travel_exhausted')

    def test_paused_ik_does_not_accumulate_an_unpublished_vertical_target(self):
        saved = {'pose':{'position':np.array([.5, -.2, 1.1]), 'orientation':np.array([1.,0,0,0])}}
        task = SimpleNamespace(step_counter=100, phase='seat',
            _ur5e_contact_hold_commands={'right':saved}, _beam_table_observation=lambda:self.sample,
            _contact_physics_dt=lambda:self.dt)
        adapter = UR5eAssemblyAtomicSkillAdapter({})
        targets=[]
        # Model a tracking guard retaining the last accepted command.
        adapter._grasp_drive_motion=lambda **kw: targets.append(copy.deepcopy(kw['target_pose'])) or {}
        adapter._hold_previous_grasp_command=lambda **kw:{}
        original = saved['pose']['position'].copy()
        for step in range(100,127):
            self.sample['step']=step; task.step_counter=step
            adapter._beam_table_seat_action(phase_key='seat', task=task, robot_name='right',
                spec=SPEC, tracked_robots={}, tracked_objects={})
        self.assertEqual(len(targets),3)
        np.testing.assert_array_equal(saved['pose']['position'], original)
        for target in targets:
            np.testing.assert_allclose(target['position'], targets[0]['position'], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
