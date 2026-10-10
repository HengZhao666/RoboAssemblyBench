"""Exercise actual task methods with a fake clock, not a copied state machine."""
from __future__ import annotations

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
NAMES = {'_beam_task_clock_event', '_update_task_state', 'is_done'}
tree = ast.parse(SOURCE.read_text())
nodes = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name in NAMES]
assert {n.name for n in nodes} == NAMES
scope = {}
exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(SOURCE), 'exec'), scope)
Task = type('Task', (), {name: scope[name] for name in NAMES})


class TaskClock(unittest.TestCase):
    def setUp(self):
        self.task = Task()
        t = self.task
        t.phase_specs = [{'beam_coordinated': True}]
        t.success = t.failed = t.policy_evaluation_mode = False
        t.step_counter = t.phase_step_counter = 0
        t.max_steps = 100
        t.clock = {'valid': True, 'epoch': 0, 'index': 100, 'time': 100 / 240, 'dt': 1 / 240}
        t._beam_physics_stamp = lambda: dict(t.clock)
        t.calls = []
        t._initialize_phase = lambda: t.calls.append('init')
        t._sync_object_states = lambda: t.calls.append('sync')
        t.get_current_phase_spec = lambda: t.phase_specs[0]
        t._beam_support_watchdog = lambda spec: t.calls.append('watchdog')
        t._process_phase_interactions = lambda spec: t.calls.append('interactions')
        t._advance_condition_met = lambda spec: False
        t._handle_phase_timeout = lambda spec: False
        def terminal(state, **kwargs):
            t.failed = state == 'failed'
            t.reason = kwargs['reason']
        t._set_terminal_state = terminal

    def advance_clock(self, count=1):
        self.task.clock['index'] += count
        self.task.clock['time'] += count / 240

    def test_duplicate_observation_and_done_queries_are_idempotent(self):
        t = self.task
        t._update_task_state()
        calls = list(t.calls)
        for _ in range(5):
            t._update_task_state()
            self.assertFalse(t.is_done())
        self.assertEqual(t.calls, calls)
        self.assertEqual(t.phase_step_counter, 1)
        self.assertEqual(t.step_counter, 0)

    def test_actual_runner_order_counts_once_per_completed_tick(self):
        t = self.task
        t._update_task_state()  # initial observation / clock priming
        for _ in range(4):
            self.advance_clock()  # actual world.step boundary
            t._update_task_state()  # get_observations after physics
            t._update_task_state()  # extra recorder / observer read
            self.assertFalse(t.is_done())
            self.assertFalse(t.is_done())
        self.assertEqual(t.step_counter, 4)
        self.assertEqual(t.phase_step_counter, 5)
        self.assertEqual(t.calls.count('watchdog'), 5)

    def test_status_query_before_observation_cannot_add_second_tick(self):
        t = self.task
        t._update_task_state()
        self.advance_clock()
        t.is_done()
        t._update_task_state()
        t.is_done()
        self.assertEqual(t.step_counter, 1)
        self.assertEqual(t.phase_step_counter, 2)

    def test_clock_reset_after_execution_stops_without_advancing_phase(self):
        t = self.task
        t._update_task_state()
        t.clock.update(epoch=1, index=0, time=0.)
        t._update_task_state()
        self.assertTrue(t.failed)
        self.assertEqual(t.reason, 'beam-task-clock-reset')
        self.assertEqual(t.phase_step_counter, 1)
        self.assertTrue(t.is_done())

    def test_missing_live_clock_after_initialization_fails_closed(self):
        t = self.task
        t._update_task_state()
        t.clock = {'valid': False, 'reason': 'view_unavailable'}
        t._update_task_state()
        self.assertTrue(t.failed)
        self.assertEqual(t.reason, 'beam-task-clock-unavailable')
        self.assertEqual(t.phase_step_counter, 1)

    def test_scene_bootstrap_initializes_once_without_fabricating_ticks(self):
        t = self.task
        t.clock = {'valid': False, 'reason': 'view_not_ready'}
        for _ in range(3):
            t._update_task_state()
            self.assertFalse(t.is_done())
        self.assertEqual(t.phase_step_counter, 1)
        self.assertEqual(t.step_counter, 0)
        t.clock = {'valid': True, 'epoch': 0, 'index': 3, 'time': 3 / 240, 'dt': 1 / 240}
        t._update_task_state()
        self.assertFalse(t.is_done())
        self.advance_clock()
        t._update_task_state()
        self.assertFalse(t.is_done())
        self.assertEqual(t.step_counter, 1)

    def test_non_beam_task_keeps_legacy_behavior_and_does_not_read_clock(self):
        t = self.task
        t.phase_specs = [{}]
        t._beam_physics_stamp = lambda: self.fail('legacy task must not read Beam clock')
        for _ in range(3):
            t._update_task_state()
            self.assertFalse(t.is_done())
        self.assertEqual(t.phase_step_counter, 3)
        self.assertEqual(t.step_counter, 3)


if __name__ == '__main__':
    unittest.main()
