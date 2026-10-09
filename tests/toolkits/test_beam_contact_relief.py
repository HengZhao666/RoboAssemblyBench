"""Measured contact can reduce squeeze, never bypass physical grasp/seating."""
import copy
import os
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from toolkits.factory_dual_franka_assembly.beam_contact_relief import contact_relief_spec
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter

CONFIG = dict(qualification_force=.25, retain_force=.1, stable_steps=24,
              force_low=3., force_high=4., joint_rate=.0024)
CARRY = dict(hold_force_regulation=True, close_force_low=4., close_force_high=12.,
             close_force_limit=40., close_force_max_seek=.03,
             close_force_max_command_lead=.001, contact_force_threshold=.1)


def sample(step, **changes):
    return dict(physics_step_index=step, valid=True, motion_valid=True,
                physics_handle_valid=True, vertical_force=.3, bottom_gap=0., **changes)


class ContactRelief(unittest.TestCase):
    def qualify(self, state):
        for step in range(24):
            selected, detail = contact_relief_spec(state, sample(step), CARRY, CONFIG)
        return selected, detail

    def test_requires_24_distinct_consecutive_physics_steps(self):
        state = {}
        for _ in range(40):
            selected, detail = contact_relief_spec(state, sample(0), CARRY, CONFIG)
        self.assertFalse(detail['active'])
        self.assertEqual(detail['qualified_steps'], 1)
        self.assertEqual(selected, CARRY)
        for step in range(1, 24):
            selected, detail = contact_relief_spec(state, sample(step), CARRY, CONFIG)
        self.assertTrue(detail['active'])
        self.assertEqual(selected['close_force_high'], 4.)

    def test_missed_steps_restart_qualification(self):
        state = {}
        for step in range(23): contact_relief_spec(state, sample(step), CARRY, CONFIG)
        _, detail = contact_relief_spec(state, sample(24), CARRY, CONFIG)
        self.assertFalse(detail['active'])
        self.assertEqual(detail['qualified_steps'], 1)

    def test_real_contact_hysteresis_never_retains_zero_load(self):
        state = {}; self.qualify(state)
        r = sample(24); r['vertical_force'] = .2
        _, detail = contact_relief_spec(state, r, CARRY, CONFIG)
        self.assertTrue(detail['active'])
        r = sample(25); r['vertical_force'] = 0.
        selected, detail = contact_relief_spec(state, r, CARRY, CONFIG)
        self.assertFalse(detail['active'])
        self.assertEqual(selected, CARRY)

    def test_invalid_handle_force_gap_or_missing_clock_restore_carry(self):
        for changes in [dict(physics_handle_valid=False), dict(vertical_force=float('nan')),
                        dict(vertical_force=None), dict(bottom_gap=.002),
                        dict(bottom_gap=-.001), dict(vertical_force=10.1),
                        dict(physics_step_index=None)]:
            with self.subTest(changes=changes):
                state = {}; self.qualify(state); r=sample(24); r.update(changes)
                selected, detail = contact_relief_spec(state, r, CARRY, CONFIG)
                self.assertFalse(detail['active']); self.assertEqual(selected, CARRY)

    def test_preserves_hard_limits_and_does_not_mutate_grasp_or_sample(self):
        state = {}; original = copy.deepcopy(CARRY)
        selected, detail = self.qualify(state)
        for key in ['close_force_limit', 'close_force_max_seek',
                    'close_force_max_command_lead', 'contact_force_threshold']:
            self.assertEqual(selected[key], CARRY[key])
        self.assertEqual(CARRY, original)
        self.assertAlmostEqual(selected['close_force_joint_rate']/240, .00001)
        self.assertNotIn('success', detail)


class ContactReliefIntegration(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        self.adapter._failure_or_hold = lambda t,r,s,reason,**kw: {'failure':reason}
        self.adapter._current_gripper_q = lambda **kw: .66
        self.adapter._gripper_open_closed_q = lambda **kw: (0., .8)
        self.metrics = {side: {'force_probe_valid':True, 'force':7.,
                       'force_observation':{'physics_dt':1/240}} for side in ['left_finger','right_finger']}
        self.calls = 0
        def observe():
            self.calls += 1
            return sample(self.task.step_counter)
        self.task = SimpleNamespace(step_counter=0, phase='base_6_set_down',
            robots={'right':SimpleNamespace(config=SimpleNamespace(gripper_close_openness=.08))},
            _attachments={'fabrica_beam_6':{'attach_spec':copy.deepcopy(CARRY)}},
            _gripper_contact_metrics=lambda *a,**kw:self.metrics,
            get_current_phase_spec=lambda:{'beam_table_contact_force_relief':CONFIG},
            _beam_table_observation=observe)
        self.saved = dict(object='fabrica_beam_6', gripper_openness=1.-.66/.8)
        self.args = dict(task=self.task, robot_name='right', spec={}, saved=self.saved, command_q=np.zeros(6))

    def test_only_qualified_contact_unloads_once_per_step_at_bounded_rate(self):
        with patch.dict(os.environ, {'BEAM_HOLD_TRACE_PATH':''}):
            for step in range(23):
                self.task.step_counter=step; action=self.adapter._publish_loaded_action(**self.args)
                self.assertAlmostEqual(.8*(1-action['gripper_controller'][0]), .66)
            self.task.step_counter=23; action=self.adapter._publish_loaded_action(**self.args)
            self.assertAlmostEqual(.8*(1-action['gripper_controller'][0]), .66-.00001)
            self.assertEqual(self.adapter._publish_loaded_action(**self.args), action)
        self.assertEqual(self.calls, 24)
        self.assertEqual(self.task._attachments['fabrica_beam_6']['attach_spec'], CARRY)

    def test_missing_finger_contact_fails_before_relief_or_seek(self):
        self.metrics['left_finger']['force']=0.
        before=copy.deepcopy(self.saved)
        result=self.adapter._publish_loaded_action(**self.args)
        self.assertEqual(result['failure'], 'loaded_force_contact_missing')
        self.assertEqual(self.saved,before); self.assertEqual(self.calls,0)

    def test_other_objects_and_unconfigured_phases_keep_carry_band(self):
        with patch.dict(os.environ, {'BEAM_HOLD_TRACE_PATH':''}):
            self.saved['object']='other'
            self.task._attachments['other']={'attach_spec':copy.deepcopy(CARRY)}
            action=self.adapter._publish_loaded_action(**self.args)
            self.assertAlmostEqual(.8*(1-action['gripper_controller'][0]), .66)
            self.task.step_counter=1; self.saved['object']='fabrica_beam_6'
            self.task.get_current_phase_spec=lambda:{}
            action=self.adapter._publish_loaded_action(**self.args)
            self.assertAlmostEqual(.8*(1-action['gripper_controller'][0]), .66)
        self.assertEqual(self.calls,0)


if __name__ == '__main__': unittest.main()
