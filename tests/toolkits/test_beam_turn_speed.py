"""Bound the column-3 turn trial and keep every other physical gate unchanged."""
import copy
import os
import unittest
from unittest.mock import patch
from roboassemblybench.core.beam_physical_compiler import _apply_continuous_beam_physics


class ColumnTurnSpeed(unittest.TestCase):
    def phases(self, speed=None):
        phases = [{'name': name, 'local_skill': {'requires_held_object': True,
                  'robot': 'franka_left', 'name': 'ur5e_move_to_target'},
                  'gripper_commands': {'franka_left': 'contact_hold'}, 'timeout_steps': 1000}
                  for name in ['base_6_assembly_clearance', 'assemble_00_part_3_pickup_clearance',
                               'assemble_00_part_3_assembly_clearance', 'assemble_00_part_3_transport_hover',
                               'assemble_01_part_2_assembly_clearance']]
        env = {'BEAM_COLUMN3_PICKUP_DEPTH_M': '0'}
        if speed is not None:
            env['BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S'] = str(speed)
        with patch.dict(os.environ, env):
            if speed is None:
                with patch.dict(os.environ):
                    os.environ.pop('BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S', None)
                    _apply_continuous_beam_physics(phases)
            else:
                _apply_continuous_beam_physics(phases)
        return phases

    def test_unset_and_explicit_baseline_are_identical(self):
        self.assertEqual(self.phases(), self.phases(.20))

    def test_half_speed_changes_only_column3_turn_angular_limit(self):
        original, trial = self.phases(), self.phases(.10)
        changed = copy.deepcopy(trial)
        phase = next(p for p in changed if p['name'] == 'assemble_00_part_3_assembly_clearance')
        self.assertEqual(phase['local_skill']['grasp_drive_angular_speed'], .10)
        phase['local_skill']['grasp_drive_angular_speed'] = .20
        self.assertEqual(changed, original)

    def test_nonfinite_zero_negative_and_faster_values_are_rejected(self):
        for speed in ['nan', 'inf', 0, -.1, .2001]:
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                self.phases(speed)


if __name__ == '__main__':
    unittest.main()
