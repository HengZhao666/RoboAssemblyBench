"""Explicit Beam release keeps native angles; all other branches retain baseline behavior."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('native_idle_fixture',ROOT/'tests/toolkits/test_beam_idle_hold.py')
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
Policy,Task=fixture.Policy,fixture.Task


class NativeReleaseBranch(unittest.TestCase):
    def setUp(self):
        self.task=Task();self.policy=Policy()
        self.policy._local_skill_executor.action_for=lambda **kw:None
        self.task.robots['left'].config=NS(preserve_continuous_arm_joints=True)
        self.task.subset.q=np.array([1.419897,-.676096,1.591104,-2.745165,-2.609363,-3.441629])
        self.task.spec.update(name='hold_6_to_3_switch_release',beam_coordinated=True,
            detach=[{'object':'fabrica_beam_6','robot':'left'}])
        self.task.phase=self.task.spec['name']
        self.task._attachments={'fabrica_beam_6':{'robot_name':'left','mode':'pure_physical_grasp'}}
        self.task.objects={'fabrica_beam_6':{'attached_to':'left',
            'attachment':{'robot_name':'left','mode':'pure_physical_grasp'}}}

    def helper(self):
        return self.policy._beam_native_release_hold(self.task,'left',self.task.spec)

    def test_reproduce_old_full_turn_and_keep_native_measured_hold(self):
        old=self.policy._normalized_joint_action(self.task.subset.q,reference=self.task.subset.q)[0]
        self.assertAlmostEqual(old[-1]-self.task.subset.q[-1],2*np.pi)
        np.testing.assert_array_equal(self.helper()[0],self.task.subset.q)

    def test_actual_compose_branch_keeps_arm_native_and_gripper_open(self):
        with patch.dict('os.environ',{'BEAM_IDLE_HOLD_ANCHOR':'0'},clear=True):
            action=self.policy.act(self.task)['left']
        np.testing.assert_array_equal(action['arm_joint_controller'][0],self.task.subset.q)
        self.assertEqual(action['gripper_controller'],[1.0])

    def test_existing_measured_hold_tracks_current_measurement_without_new_anchor(self):
        first=self.helper()[0]
        self.task.subset.q[-1]+=.0002
        second=self.helper()[0]
        self.assertAlmostEqual(second[-1]-first[-1],.0002)
        self.assertFalse(self.policy._beam_idle_anchors)

    def test_positive_and_multi_turn_native_values_are_not_folded(self):
        for value in [-7.,-3.441629,3.55,7.1]:
            self.task.subset.q[-1]=value
            self.assertEqual(self.helper()[0][-1],value)

    def test_unrelated_phases_robots_and_default_noncontinuous_path_are_unchanged(self):
        for field,value in [('beam_coordinated',False),('detach',[]),
                ('detach',[{'object':'fabrica_beam_6','robot':'other'}]),
                ('gripper_commands',{'left':'contact_hold'})]:
            original=self.task.spec[field];self.task.spec[field]=value
            self.assertIsNone(self.helper());self.task.spec[field]=original
        self.task.robots['left'].config.preserve_continuous_arm_joints=False
        self.assertIsNone(self.helper())

    def test_invalid_native_measurements_abort_without_wrapped_fallback(self):
        for value in [None,np.ones(5),np.array([1,2,3,4,5,np.nan])]:
            self.task.subset.q=value
            with self.assertRaises(RuntimeError):self.helper()

    def test_no_writes_to_joint_or_physical_state(self):
        original=self.task.subset.q.copy()
        self.helper()
        np.testing.assert_array_equal(self.task.subset.q,original)
        self.assertEqual(len(self.task._attachments),1)


if __name__=='__main__':unittest.main()
