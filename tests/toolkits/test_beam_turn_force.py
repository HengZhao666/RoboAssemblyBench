"""A higher turn preload must reach the publisher without changing grasp gates."""
import copy
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from roboassemblybench.core.beam_physical_compiler import _apply_continuous_beam_physics
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter

TURN = 'assemble_00_part_3_assembly_clearance'
CARRY = dict(hold_force_regulation=True, close_force_low=4., close_force_high=12.,
             close_force_limit=40., close_force_max_seek=.03,
             close_force_max_command_lead=.001, close_force_joint_rate=.025,
             contact_force_threshold=.1)


class CompileForceBand(unittest.TestCase):
    def compile(self, band=None):
        phases = [{'name': name, 'local_skill': {'requires_held_object': True,
                  'robot': 'franka_left', 'name': 'ur5e_move_to_target'},
                  'gripper_commands': {'franka_left': 'contact_hold'}, 'timeout_steps': 1000}
                  for name in ['base_6_assembly_clearance', 'assemble_00_part_3_pickup_clearance',
                               TURN, 'assemble_00_part_3_transport_hover',
                               'assemble_01_part_2_assembly_clearance']]
        with patch.dict(os.environ, {'BEAM_COLUMN3_PICKUP_DEPTH_M': '0',
                                     'BEAM_COLUMN3_TURN_ANGULAR_SPEED_RAD_S': '.20'}):
            if band is None:
                os.environ.pop('BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N', None)
            else:
                os.environ['BEAM_COLUMN3_TURN_PEAK_FORCE_BAND_N'] = band
            _apply_continuous_beam_physics(phases)
        return phases

    def test_default_and_explicit_original_are_identical(self):
        self.assertEqual(self.compile(), self.compile('4,12'))

    def test_only_one_phase_gains_the_force_setting(self):
        original, trial = self.compile(), self.compile('16,24')
        changed = copy.deepcopy(trial)
        phase = next(p for p in changed if p['name'] == TURN)
        self.assertEqual(phase['local_skill'].pop('beam_turn_peak_force_band_N'), [16., 24.])
        self.assertEqual(changed, original)

    def test_bad_or_out_of_range_band_is_rejected(self):
        for band in ['nan,24', '16,inf', '16', '1,2,3', 'bad,24', '3,12',
                     '24,16', '16,16', '16,24.01']:
            with self.subTest(band=band), self.assertRaises(ValueError): self.compile(band)


class PublisherForceBand(unittest.TestCase):
    def setUp(self):
        self.adapter = UR5eAssemblyAtomicSkillAdapter({})
        self.adapter._current_gripper_q = lambda **kw: .715
        self.adapter._gripper_open_closed_q = lambda **kw: (0., .802)
        self.adapter._remember_gripper_hold_openness = lambda **kw: None
        self.metrics = {side: dict(force_probe_valid=True, force=force,
                       force_observation={'physics_dt': 1/240}) for side, force in
                       [('left_finger', 8.57), ('right_finger', 7.33)]}
        self.phase_spec = {'local_skill': {'beam_turn_peak_force_band_N': [16., 24.]}}
        self.task = NS(step_counter=100, phase=TURN,
            robots={'left': NS(config=NS(gripper_close_openness=.08))},
            _attachments={'fabrica_beam_3': {'attach_spec': copy.deepcopy(CARRY)}},
            _gripper_contact_metrics=lambda *a, **kw: self.metrics,
            get_current_phase_spec=lambda: self.phase_spec)
        self.saved = dict(object='fabrica_beam_3', gripper_openness=1.-.715/.802)
        self.args = dict(task=self.task, robot_name='left', spec={'require_success': True},
                         saved=self.saved, command_q=np.arange(6.)/10)
        self.env = patch.dict(os.environ, {'BEAM_HOLD_TRACE_PATH': ''})
        self.env.start(); self.addCleanup(self.env.stop)

    def publish(self): return self.adapter._publish_loaded_action(**self.args)
    def closure(self, action): return .802 * (1. - action['gripper_controller'][0])

    def test_actual_publisher_increases_preload_once_per_tick(self):
        action = self.publish()
        self.assertAlmostEqual(self.closure(action), .715+.025/240)
        before = copy.deepcopy(self.saved)
        self.assertEqual(self.publish(), action); self.assertEqual(self.saved, before)
        self.task.step_counter += 1
        self.assertAlmostEqual(self.closure(self.publish()), .715+2*.025/240)
        self.assertEqual(self.task._attachments['fabrica_beam_3']['attach_spec'], CARRY)
        self.assertEqual(self.saved['loaded_force_state']['contact_capture_closure'], .715)
        self.assertEqual(action['arm_joint_controller'][0], list(np.arange(6.)/10))

    def test_peak_inside_band_holds_and_above_band_unloads(self):
        for force, delta, mode in [(18., 0., 'bilateral_hold'), (25., -.025/240, 'unload')]:
            with self.subTest(force=force):
                self.setUpFresh()
                self.metrics['left_finger']['force'] = force
                action = self.publish()
                self.assertAlmostEqual(self.closure(action), .715+delta)
                self.assertEqual(self.saved['loaded_force_state']['last_control']['mode'], mode)

    def setUpFresh(self):
        self.saved = dict(object='fabrica_beam_3', gripper_openness=1.-.715/.802)
        self.args['saved'] = self.saved

    def test_over_40N_still_returns_original_failure_before_publication(self):
        self.metrics['left_finger']['force'] = 40.01
        result = self.publish()
        self.assertTrue(result['__local_skill_failure__'])
        self.assertEqual(result['reason'], 'loaded_gripper_force_limit')
        self.assertNotIn('gripper_controller', result)
        self.assertEqual(self.saved['gripper_openness'], 1.-.715/.802)

    def test_missing_or_invalid_contact_cannot_seek(self):
        for side, change in [('left_finger', {'force': 0.}),
                             ('right_finger', {'force_probe_valid': False}),
                             ('left_finger', {'force': float('nan')})]:
            original = copy.deepcopy(self.metrics)
            self.metrics[side].update(change)
            before = copy.deepcopy(self.saved)
            result = self.publish()
            self.assertEqual(result['reason'], 'loaded_force_contact_missing')
            self.assertEqual(self.saved, before)
            self.metrics = original

    def test_wrong_part_phase_or_unconfigured_phase_keeps_original_band(self):
        for obj, phase, config in [('other', TURN, self.phase_spec),
                                  ('fabrica_beam_3', 'assemble_00_part_3_transport_hover', self.phase_spec),
                                  ('fabrica_beam_3', TURN, {})]:
            with self.subTest(obj=obj, phase=phase):
                self.setUpFresh(); self.saved['object'] = obj
                self.task._attachments[obj] = {'attach_spec': copy.deepcopy(CARRY)}
                self.task.phase = phase; self.phase_spec = config
                self.assertAlmostEqual(self.closure(self.publish()), .715)

    def test_invalid_runtime_band_fails_before_any_preload(self):
        for band in [[16., 41.], [16.], [float('nan'), 24.], 'bad']:
            self.phase_spec = {'local_skill': {'beam_turn_peak_force_band_N': band}}
            before = self.saved['gripper_openness']
            result = self.publish()
            self.assertEqual(result['reason'], 'loaded_turn_force_band_invalid')
            self.assertEqual(self.saved['gripper_openness'], before)
        self.task._attachments['fabrica_beam_3']['attach_spec']['close_force_limit'] = 20.
        self.phase_spec = {'local_skill': {'beam_turn_peak_force_band_N': [16., 24.]}}
        self.assertEqual(self.publish()['reason'], 'loaded_turn_force_band_invalid')

    def test_existing_seek_and_command_lead_caps_are_preserved(self):
        self.saved['loaded_force_state'] = dict(contact_control_active=True,
             contact_capture_closure=.685, contact_command_closure=.715)
        self.assertAlmostEqual(self.closure(self.publish()), .715)
        self.setUpFresh()
        self.saved['loaded_force_state'] = dict(contact_control_active=True,
             contact_capture_closure=.715, contact_command_closure=.716)
        for step in range(100, 130):
            self.task.step_counter = step
            self.assertAlmostEqual(self.closure(self.publish()), .716)

    def test_trace_records_consumed_band_and_force_without_success_claim(self):
        with tempfile.TemporaryDirectory() as td, patch.dict(os.environ,
                {'BEAM_HOLD_TRACE_PATH': str(Path(td)/'hold.jsonl')}):
            self.publish()
            row = json.loads((Path(td)/'loaded_force_trace.jsonl').read_text())
        control = row['control']
        self.assertEqual((control['force_low'], control['force_high']), (16., 24.))
        self.assertEqual(control['force_limit'], 40.)
        self.assertEqual(control['peak_force'], 8.57)
        self.assertNotIn('success', control)


if __name__ == '__main__': unittest.main()
