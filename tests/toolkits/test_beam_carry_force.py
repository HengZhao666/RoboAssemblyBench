"""Early transport preload retains admission, anchor and hard limits."""
import copy
import os
import unittest
from unittest.mock import patch
from roboassemblybench.core.beam_physical_compiler import _apply_continuous_beam_physics
import test_beam_turn_force as turn_tests

CARRY_PHASES = ['assemble_00_part_3_lift', 'assemble_00_part_3_post_lift_restore_gravity',
                'assemble_00_part_3_pickup_clearance']

class CarryCompilation(unittest.TestCase):
    def compile(self, band=None):
        names = CARRY_PHASES + ['assemble_00_part_3_close_and_attach',
            'assemble_00_part_3_peel_lift', 'assemble_00_part_3_assembly_clearance',
            'assemble_00_part_3_transport_hover', 'base_6_lift', 'assemble_01_part_2_lift']
        phases = [{'name': name, 'local_skill': {'requires_held_object': True,
                  'robot': 'franka_left', 'object': 'fabrica_beam_3', 'name': 'retreat_vertical',
                  'relative_to_current_tcp': True}, 'timeout_steps': 1000} for name in names]
        with patch.dict(os.environ, {'BEAM_COLUMN3_PICKUP_DEPTH_M': '0',
                'BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N': '4,12'}):
            os.environ.pop('BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N', None)
            if band is not None:os.environ['BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N'] = band
            _apply_continuous_beam_physics(phases)
        return phases

    def test_default_is_identical_and_only_three_stages_change(self):
        original = self.compile()
        self.assertEqual(original, self.compile('4,12'))
        trial = self.compile('16,24')
        for phase in trial:
            if phase['name'] in CARRY_PHASES:
                self.assertEqual(phase['local_skill'].pop('beam_carry_peak_force_band_N'), [16., 24.])
        self.assertEqual(trial, original)

    def test_invalid_band_rejected_before_phase_mutation(self):
        for band in ['nan,24', '16,inf', '16', '1,2,3', '3,12', '24,16', '16,16', '16,24.01']:
            with self.subTest(band=band), self.assertRaises(ValueError): self.compile(band)
        phases = [{'name': 'base_6_lift'}]; before = copy.deepcopy(phases)
        with patch.dict(os.environ, {'BEAM_COLUMN3_CARRY_PEAK_FORCE_BAND_N': '3,24'}), self.assertRaises(ValueError):
            _apply_continuous_beam_physics(phases)
        self.assertEqual(phases, before)

class CarryPublisher(unittest.TestCase):
    def setUp(self):
        # Reuse the real bounded publisher setup without inheriting test methods.
        turn_tests.PublisherForceBand.setUp(self)
        self.task.phase = CARRY_PHASES[0]
        self.phase_spec = {'local_skill': {'beam_carry_peak_force_band_N': [16., 24.]}}
    publish = turn_tests.PublisherForceBand.publish
    closure = turn_tests.PublisherForceBand.closure

    def test_preload_reaches_actual_publisher_in_all_three_stages(self):
        for phase in CARRY_PHASES:
            self.task.phase = phase; self.task.step_counter += 1
            action = self.publish()
            self.assertGreater(self.closure(action), .715)
            control = self.saved['loaded_force_state']['last_control']
            self.assertEqual((control['force_low'], control['force_high']), (16., 24.))
        self.assertEqual(self.saved['loaded_force_state']['contact_capture_closure'], .715)

    def test_wrong_part_or_other_stage_does_not_apply_carry_band(self):
        for phase in ['assemble_00_part_3_peel_lift', 'assemble_00_part_3_transport_hover',
                      'base_6_lift', 'assemble_00_part_3_assembly_clearance']:
            self.task.phase = phase; self.task.step_counter += 1
            self.assertAlmostEqual(self.closure(self.publish()), .715)
        self.task.phase = CARRY_PHASES[0]; self.saved['object'] = 'other'
        self.task._attachments['other'] = copy.deepcopy(self.task._attachments['fabrica_beam_3'])
        self.task.step_counter += 1
        self.assertAlmostEqual(self.closure(self.publish()), .715)

    def test_original_40N_and_contact_failures_still_stop_publication(self):
        self.metrics['left_finger']['force'] = 40.01
        self.assertEqual(self.publish()['reason'], 'loaded_gripper_force_limit')
        self.metrics['left_finger']['force'] = 0.
        self.assertEqual(self.publish()['reason'], 'loaded_force_contact_missing')

    def test_runtime_invalid_band_stops_before_closing(self):
        self.phase_spec['local_skill']['beam_carry_peak_force_band_N'] = [16., 41.]
        before = copy.deepcopy(self.saved['gripper_openness'])
        self.assertEqual(self.publish()['reason'], 'loaded_carry_force_band_invalid')
        self.assertEqual(self.saved['gripper_openness'], before)

if __name__ == '__main__':unittest.main()
