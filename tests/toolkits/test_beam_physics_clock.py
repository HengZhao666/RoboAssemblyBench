"""Regress the action/physics/observation timing contract without Isaac writes."""
import copy
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from toolkits.factory_dual_franka_assembly.beam_support_state import (
    BeamSupportState, physics_sample_token, physics_samples_consecutive,
)


class ClockContext:
    def __init__(self, index=40, dt=1 / 240):
        self.current_time_step_index = index
        self.current_time = index * dt
        self.dt = dt
        self.stopped = False

    def get_physics_dt(self):
        return self.dt

    def is_stopped(self):
        return self.stopped

    def advance(self, count=1):
        self.current_time_step_index += count
        self.current_time += count * self.dt


class PhysicsClockContract(unittest.TestCase):
    def setUp(self):
        self.context = ClockContext()
        self.view = object()
        self.clock_calls = []
        api = SimpleNamespace(SimulationContext=SimpleNamespace(instance=lambda: self.context))
        manager = SimpleNamespace(SimulationManager=SimpleNamespace(get_physics_sim_view=self.existing_view))
        self.modules = patch.dict(sys.modules, {
            'isaacsim.core.api': api,
            'isaacsim.core.simulation_manager': manager,
        })
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.task = BeamSupportState()
        self.task.step_counter = 20
        self.task.phase = 'base_6_set_down'
        self.task.phase_index = 2
        self.task.phase_entry_step = 15
        self.supports = ['table', 'thick_support']
        self.task.get_current_phase_spec = lambda: {'beam_table_support_objects': self.supports}
        self.reads = []
        self.payload = {'valid': True, 'motion_valid': True, 'physics_handle_valid': True,
                        'vertical_force': 2.4, 'bottom_gap': 0.,
                        'linear_speed': 0., 'angular_speed': 0.,
                        'position': [0., 0., 1.], 'orientation': [1., 0., 0., 0.],
                        'support_surfaces': [{'contact': {'force_world': [0., 0., 2.4]}}]}
        self.task._beam_read_table_support_sample = self.read_payload

    def existing_view(self):
        self.clock_calls.append('read_existing_view')
        return self.view

    def read_payload(self, names):
        self.reads.append(tuple(names))
        return copy.deepcopy(self.payload)

    def observe(self):
        return self.task._beam_table_observation()

    def gate(self, required=3):
        return self.task._beam_table_supported({'stable_steps': required})

    def test_policy_k_post_physics_k_plus_one_with_same_task_label(self):
        before = self.observe()
        self.context.advance()
        self.payload['vertical_force'] = 0.
        after = self.observe()
        self.assertEqual(before['step'], after['step'])
        self.assertEqual(before['physics_step_index'] + 1, after['physics_step_index'])
        self.assertEqual(before['vertical_force'], 2.4)
        self.assertEqual(after['vertical_force'], 0.)
        self.assertEqual(len(self.reads), 2)

    def test_task_labels_do_not_trigger_new_physical_sample_or_stability(self):
        self.assertFalse(self.gate())
        for label in [21, 22, 99, 1024]:
            self.task.step_counter = label
            self.assertFalse(self.gate())
        self.assertEqual(len(self.reads), 1)
        self.assertEqual(self.task._beam_support_stability['count'], 1)
        self.assertEqual(self.observe()['step'], 1024)
        self.assertEqual(self.observe()['sample_task_step'], 20)

    def test_cache_isolated_from_nested_consumer_mutation(self):
        sample = self.observe()
        sample['position'][2] = 100.
        sample['support_surfaces'][0]['contact']['force_world'][2] = 100.
        sample['physics_stamp']['index'] = -1
        reread = self.observe()
        self.assertEqual(reread['position'][2], 1.)
        self.assertEqual(reread['support_surfaces'][0]['contact']['force_world'][2], 2.4)
        self.assertEqual(reread['physics_stamp']['index'], 40)
        self.assertEqual(len(self.reads), 1)

    def test_support_set_changes_recollect_but_order_and_duplicates_do_not(self):
        self.observe()
        self.supports = ['thick_support', 'table', 'table']
        self.observe()
        self.assertEqual(len(self.reads), 1)
        self.supports = ['table']
        self.observe()
        self.assertEqual(self.reads, [('table', 'thick_support'), ('table',)])

    def test_support_set_changes_cannot_inherit_old_qualification_streak(self):
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        self.supports = ['table']
        self.assertFalse(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 1)

    def test_first_motion_probe_retains_configured_support_order(self):
        self.supports = ['z_optical_board', 'a_assembly_support', 'z_optical_board']
        self.observe()
        self.assertEqual(self.reads, [('z_optical_board', 'a_assembly_support')])

    def test_sampling_across_a_physics_advance_is_invalid_and_not_reused(self):
        def torn_read(names):
            result = self.read_payload(names)
            self.context.advance()
            return result
        self.task._beam_read_table_support_sample = torn_read
        sample = self.observe()
        self.assertFalse(sample['valid'])
        self.assertFalse(sample['physics_stamp_valid'])
        self.assertIn('changed during', sample['error'])
        self.assertIsNone(physics_sample_token(sample))
        self.task._beam_read_table_support_sample = self.read_payload
        self.assertTrue(self.observe()['valid'])
        self.assertEqual(len(self.reads), 2)

    def test_context_replacement_with_identical_clock_starts_epoch(self):
        previous = self.observe()
        self.context = ClockContext()
        current = self.observe()
        self.assertEqual(previous['physics_step_index'], current['physics_step_index'])
        self.assertEqual(current['physics_epoch'], previous['physics_epoch'] + 1)
        self.assertEqual(len(self.reads), 2)

    def test_backend_view_replacement_with_identical_clock_starts_epoch(self):
        previous = self.observe()
        self.view = object()
        current = self.observe()
        self.assertEqual(current['physics_epoch'], previous['physics_epoch'] + 1)
        self.assertEqual(len(self.reads), 2)

    def test_clock_rollback_cannot_reuse_a_previous_epoch_or_streak(self):
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        epoch = self.observe()['physics_epoch']
        self.context.current_time_step_index = 40
        self.context.current_time = 40 * self.context.dt
        self.assertFalse(self.gate())
        self.assertEqual(self.observe()['physics_epoch'], epoch + 1)
        self.assertEqual(self.task._beam_support_stability['count'], 1)

    def test_physics_dt_change_starts_epoch_and_rewarms_streak(self):
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        epoch = self.observe()['physics_epoch']
        self.context.dt /= 2
        self.assertFalse(self.gate())
        self.assertEqual(self.observe()['physics_epoch'], epoch + 1)
        self.assertEqual(self.task._beam_support_stability['count'], 1)

    def test_missing_context_fails_closed_without_reading_physics_payload(self):
        self.context = None
        sample = self.observe()
        self.assertFalse(sample['valid'])
        self.assertFalse(sample['physics_stamp_valid'])
        self.assertFalse(self.gate(required=1))
        self.assertEqual(self.reads, [])

    def test_stopped_or_missing_backend_view_invalidates_and_rewarms_epoch(self):
        initial = self.observe()
        self.context.stopped = True
        self.assertFalse(self.observe()['valid'])
        self.context.stopped = False
        recovered = self.observe()
        self.assertEqual(recovered['physics_epoch'], initial['physics_epoch'] + 1)
        self.view = None
        self.assertFalse(self.observe()['valid'])

    def test_nonfinite_clock_or_time_change_without_index_is_rejected(self):
        self.observe()
        self.context.current_time += self.context.dt
        self.assertFalse(self.observe()['valid'])
        self.context.current_time = float('nan')
        self.assertFalse(self.observe()['valid'])
        self.assertFalse(self.gate(required=1))

    def test_index_change_without_elapsed_physics_time_is_rejected(self):
        self.observe()
        self.context.current_time_step_index += 1
        self.assertFalse(self.observe()['valid'])
        self.assertFalse(self.gate(required=1))

    def test_gate_qualifies_only_consecutive_fresh_physical_samples(self):
        self.assertFalse(self.gate())
        for _ in range(10):
            self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertTrue(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 3)

    def test_skipped_index_restarts_streak_with_current_sample(self):
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        self.context.advance(count=2)
        self.assertFalse(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 1)

    def test_invalid_body_sample_or_load_loss_resets_streak(self):
        self.assertFalse(self.gate())
        self.context.advance()
        self.assertFalse(self.gate())
        self.context.advance()
        self.payload['motion_valid'] = False
        self.assertFalse(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 0)
        self.payload['motion_valid'] = True
        self.context.advance()
        self.payload['vertical_force'] = 0.
        self.assertFalse(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 0)

    def test_consecutive_indices_with_wrong_elapsed_time_do_not_qualify(self):
        self.assertFalse(self.gate())
        self.context.current_time_step_index += 1
        self.context.current_time += 2 * self.context.dt
        self.assertFalse(self.gate())
        self.assertEqual(self.task._beam_support_stability['count'], 1)

    def test_gate_does_not_accept_unstamped_legacy_task_step_sample(self):
        self.task._beam_table_observation = lambda **kwargs: {'step': 20, **self.payload}
        self.assertFalse(self.gate(required=1))
        self.assertEqual(self.task._beam_support_stability['count'], 0)

    def test_compact_trace_distinguishes_policy_cache_k_from_gate_k_plus_one(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ,
                {'BEAM_MOTION_TRACE_PATH': str(Path(directory) / 'motion_trace.jsonl')}):
            self.observe()
            self.task.step_counter = 21
            self.observe()
            self.context.advance()
            self.gate()
            rows = [json.loads(line) for line in (Path(directory) / 'table_physics_clock_trace.jsonl').read_text().splitlines()]
        policy = next(row for row in rows if row['event'] == 'cache_read')
        gate = next(row for row in rows if row['event'] == 'support_gate')
        self.assertEqual(policy['task_step_label'], gate['task_step_label'])
        self.assertEqual(policy['physics_stamp']['index'] + 1, gate['physics_step_index'])
        self.assertEqual(gate['consumer'], 'post_physics_gate')

    def test_probe_failure_does_not_make_cached_success_from_same_clock(self):
        def fail(names):
            raise ValueError('No live rigid view')
        self.task._beam_read_table_support_sample = fail
        sample = self.observe()
        self.assertTrue(sample['physics_stamp_valid'])
        self.assertFalse(sample['valid'])
        self.assertFalse(self.gate(required=1))

    def test_watchdog_counts_physics_loss_duration_not_task_labels(self):
        self.task._beam_assembly_integrity = lambda phase: None
        failures = []
        self.task._set_terminal_state = lambda *args, **kwargs: failures.append(kwargs)
        phase = {'beam_require_table_support': True}
        self.payload['vertical_force'] = 0.
        for label in range(20, 100):
            self.task.step_counter = label
            self.task._beam_support_watchdog(phase)
        self.assertEqual(failures, [])
        for _ in range(48):
            self.context.advance()
            self.task._beam_support_watchdog(phase)
        self.assertEqual(len(failures), 1)
        self.assertEqual(failures[0]['reason'], 'beam-table-support-lost')

    def test_watchdog_invalid_clock_is_explicit_failure_not_silent_timeout(self):
        self.task._beam_assembly_integrity = lambda phase: None
        failures = []
        self.task._set_terminal_state = lambda *args, **kwargs: failures.append(kwargs)
        self.context = None
        self.task._beam_support_watchdog({'beam_require_table_support': True})
        self.assertEqual(failures[0]['reason'], 'beam-table-support-clock-invalid')

    def test_clock_getters_only_observe_existing_context_and_view(self):
        sample = self.observe()
        self.assertTrue(sample['physics_stamp_valid'])
        self.assertEqual(self.clock_calls, ['read_existing_view', 'read_existing_view'])


if __name__ == '__main__':
    unittest.main()
