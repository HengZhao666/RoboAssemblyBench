"""Exercise deployed task/gate methods and stop-before-interaction ordering."""
from __future__ import annotations
import ast
import copy
from pathlib import Path
from types import SimpleNamespace
import unittest

from toolkits.factory_dual_franka_assembly.beam_support_state import BeamSupportState
from toolkits.factory_dual_franka_assembly.beam_exploration import STOP_PHASE

SOURCE = Path(__file__).resolve().parents[2] / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
NAMES = {'_update_task_state', 'is_done', 'calculate_metrics', '_handle_phase_timeout'}
nodes = [n for n in ast.walk(ast.parse(SOURCE.read_text()))
         if isinstance(n, ast.FunctionDef) and n.name in NAMES]
scope = {'copy': copy}
exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(SOURCE), 'exec'), scope)
Task = type('Task', (BeamSupportState,), {name: scope[name] for name in NAMES})


def sample(index, force=2.45, raw_speed=.4):
    return {'valid': True, 'motion_valid': True, 'physics_stamp_valid': True,
            'physics_epoch': 0, 'physics_step_index': index, 'physics_time': index / 240., 'physics_dt': 1 / 240.,
            'position': [0., 0., 1.], 'orientation': [1., 0., 0., 0.],
            'linear_speed': raw_speed, 'angular_speed': raw_speed,
            'vertical_force': abs(force), 'bottom_gap': 0., 'support_object_names': ['table'],
            'support_surfaces': [{'object': 'table', 'contact': {'valid': True, 'force_world': [0., 0., force]}}],
            'native_com': {'valid': True, 'world_positions': [[0., 0., 1.006]]},
            'motion_body_id': '/Beam6', 'motion_view_id': 1}


class Integration(unittest.TestCase):
    def task(self, enabled=True):
        t = Task()
        t._beam_exploration_enabled = enabled
        t._beam_exploration_complete = False
        t._beam_exploration_state = {}
        t._continuous_grasp_loss_generation = {}
        t.step_counter = t.phase_entry_step = t.phase_index = t.phase_step_counter = 0
        t.phase = 'base_6_set_down'
        t.phase_specs = [{'name': t.phase, 'beam_coordinated': True}]
        t.success = t.failed = t.policy_evaluation_mode = False
        t.terminal_reason = None
        t.phase_history = [t.phase]
        t.calls = []
        t.current = sample(0)
        t._beam_table_observation = lambda **kwargs: copy.deepcopy(t.current)
        t._beam_trace_clock = lambda event: t.calls.append(('raw_gate', event))
        t._beam_exploration_record = lambda event, s, d, p=None: t.calls.append((event, copy.deepcopy(d)))
        def terminal(phase, **kwargs):
            t.phase = phase
            t.failed = kwargs['status'] == 'failed'
            t.success = kwargs['status'] == 'success'
            t.phase_status = kwargs['status']
            t.terminal_reason = kwargs['reason']
            t.calls.append(('terminal', kwargs['reason']))
        t._set_terminal_state = terminal
        t._beam_assembly_integrity = lambda spec: None
        return t

    def qualify(self, t):
        for i in range(700):
            t.current = sample(i)
            if t._beam_table_supported({'stable_steps': 96}):
                return i
        self.fail('Stable native poses/load must qualify despite high raw velocities')

    def update_stubs(self, t, phase, advance=True):
        t.phase = phase
        t.phase_specs = [{'name': phase}, {'name': 'hold_6_to_3_switch_release'}]
        t.get_current_phase_spec = lambda: t.phase_specs[t.phase_index]
        t._beam_task_clock_event = lambda event: True
        t._initialize_phase = lambda: t.calls.append(('initialize', t.phase))
        t._sync_object_states = lambda: t.calls.append(('sync', t.phase))
        t._beam_support_watchdog = lambda spec: None
        t._process_phase_interactions = lambda spec: t.calls.append(('interactions', spec['name']))
        t._consume_phase_transition_request = lambda: False
        t._advance_condition_met = lambda spec: advance
        t._handle_phase_timeout = lambda spec: False
        def set_phase(index, **kwargs):
            t.phase_index = index
            t.phase = t.phase_specs[index]['name']
            t.calls.append(('set_phase', t.phase))
        t._set_phase = set_phase
        t.calls.clear()

    def test_actual_new_gate_keeps_original_gate_false_and_duplicates_do_not_count(self):
        t = self.task()
        self.qualify(t)
        self.assertEqual(t._beam_support_stability['count'], 0)
        self.assertEqual(t._beam_exploration_state['original_seat_ready'], False)
        count = t._beam_exploration_state['seat_count']
        for _ in range(5):
            self.assertTrue(t._beam_table_supported({'stable_steps': 96}))
        self.assertEqual(t._beam_exploration_state['seat_count'], count)
        raw = [event for name, event in t.calls if name == 'raw_gate']
        self.assertTrue(raw)
        self.assertTrue(all(not event['ready'] and event['stable_count'] == 0 for event in raw))

    def test_default_off_does_not_read_com_or_exploration_state(self):
        t = self.task(False)
        for i in range(96):
            t.current = sample(i, raw_speed=0.)
            del t.current['native_com']
            passed = t._beam_table_supported({'stable_steps': 96})
        self.assertTrue(passed)
        self.assertEqual(t._beam_exploration_state, {})
        self.assertFalse(hasattr(t, '_beam_exploration_pose_tracker'))

    def test_invalid_com_never_uses_actor_fallback(self):
        t = self.task()
        for i in range(600):
            t.current = sample(i)
            t.current['native_com']['valid'] = False
            ready = t._beam_table_supported({'stable_steps': 96})
            if t.failed:
                break
        self.assertFalse(ready)
        self.assertTrue(t.failed)

    def test_first_insertion_completion_stops_before_release_entry(self):
        t = self.task()
        self.qualify(t)
        self.update_stubs(t, STOP_PHASE)
        t._update_task_state()
        self.assertTrue(t._beam_exploration_complete)
        self.assertFalse(t.success or t.failed)
        self.assertTrue(t.is_done())
        self.assertFalse(any(name == 'set_phase' for name, _ in t.calls))
        self.assertFalse(any(name == 'interactions' and 'release' in value for name, value in t.calls))

    def test_continuation_advances_into_actual_release_interaction(self):
        t = self.task()
        t._beam_exploration_state['continue_after_first_insertion'] = True
        self.qualify(t)
        self.update_stubs(t, STOP_PHASE)
        t._update_task_state()
        self.assertFalse(t._beam_exploration_complete)
        self.assertFalse(t.failed)
        self.assertTrue(t._beam_exploration_state['first_insertion_complete'])
        self.assertTrue(any(name == 'set_phase' and value == 'hold_6_to_3_switch_release' for name, value in t.calls))
        self.assertTrue(any(name == 'interactions' and value == 'hold_6_to_3_switch_release' for name, value in t.calls))

    def test_unexpected_loss_stops_before_phase_interactions(self):
        t = self.task()
        self.qualify(t)
        self.update_stubs(t, 'assemble_00_part_3_move_above', advance=False)
        t.current = sample(t.current['physics_step_index'] + 1)
        t._continuous_grasp_loss_generation['fabrica_beam_3'] = 1
        t._update_task_state()
        self.assertTrue(t.failed)
        self.assertFalse(any(name == 'interactions' for name, _ in t.calls))

    def test_loss_discovered_after_interactions_stops_before_returning_next_action(self):
        t = self.task()
        self.qualify(t)
        self.update_stubs(t, 'assemble_00_part_3_move_above', advance=False)
        t.current = sample(t.current['physics_step_index'] + 1)
        sync_count = [0]
        def sync():
            sync_count[0] += 1
            if sync_count[0] == 2:
                t._continuous_grasp_loss_generation['fabrica_beam_3'] = 1
        t._sync_object_states = sync
        t._update_task_state()
        self.assertTrue(t.failed)
        self.assertIn('unexpected_grasp_loss', t.terminal_reason)
        self.assertEqual(t.phase_step_counter, 0)

    def test_negative_force_is_not_support_even_with_stable_pose(self):
        t = self.task()
        t.current = sample(0, force=-2.45)
        self.assertFalse(t._beam_table_supported({'stable_steps': 96}))
        self.assertTrue(t.failed)

    def test_post_seat_com_missing_or_displaced_stops_before_next_action(self):
        for kind in ('missing', 'displaced', 'view_changed'):
            t = self.task()
            self.qualify(t)
            t.current = sample(t.current['physics_step_index'] + 1)
            if kind == 'missing':
                del t.current['native_com']
            elif kind == 'displaced':
                t.current['native_com']['world_positions'][0][0] = .0011
            else:
                t.current['motion_view_id'] = 2
            self.assertTrue(t._beam_exploration_guard({'name': 'assemble_00_part_3_move_above'}))
            self.assertTrue(t.failed)

    def test_timeout_retry_does_not_execute_reset_actions(self):
        t = self.task()
        t.phase_step_counter = 10
        t.phase_timeout_count = 0
        t._recovery_history = t.phase_transition_history = []
        t._resolve_timeout_spec = lambda spec: {'steps': 10, 'reset': {'detach': 'Beam6'}, 'action': 'retry'}
        t._transition_details = lambda **kwargs: kwargs
        t._apply_phase_actions = lambda spec: self.fail('Exploration must not execute timeout reset')
        self.assertTrue(t._handle_phase_timeout({}))
        self.assertTrue(t.failed)
        self.assertEqual(t.terminal_reason, 'beam-exploration-phase-timeout')

    def test_metrics_cannot_report_formal_success_for_exploration(self):
        t = self.task()
        t._check_success = lambda: True
        t.cfg = SimpleNamespace(recipe='beam', seed=0, layout_seed=505, episode_idx=0)
        t.phase_status = 'exploration_stopped'
        t.phase_transition_history = t._handoff_history = t._recovery_history = t._policy_interaction_history = []
        t.phase_attempts = {}
        t.phase_timeout_count = t.phase_recovery_count = t._policy_success_stable_count = 0
        t._policy_success_stable_steps = 1
        t.get_tracked_object_states = t.get_tracked_robot_states = t._success_diagnostics = lambda: {}
        metrics = t.calculate_metrics()
        self.assertFalse(metrics['success'])
        self.assertFalse(metrics['success_detector']['passed'])
        self.assertFalse(metrics['beam_exploration']['formal_success'])
        self.assertTrue(metrics['success_detector']['criteria_passed'])


if __name__ == '__main__':
    unittest.main()
