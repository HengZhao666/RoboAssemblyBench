"""Release/final gates must use live poses and fresh physical time."""
import copy
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

from toolkits.factory_dual_franka_assembly.beam_assisted_state import (
    BeamAssistedState, consecutive_native_gate,
)
from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error, pose_within_tolerance


def stamp(index, *, epoch=0):
    return {'valid': True, 'epoch': epoch, 'index': index,
            'time': index / 240., 'dt': 1 / 240.}


class FakeTask(BeamAssistedState):
    def __init__(self, names=('fabrica_beam_6',)):
        self.step_counter = 1
        self.phase_index = 9
        self.phase_entry_step = 1
        self.phase = 'base_6_release_and_lock'
        self.clock = stamp(1)
        self.events = []
        self._attachments = {}
        self._locked_targets = {n: n + '_assembled' for n in names}
        self.target_poses = {n + '_assembled': {
            'position': [0., 0., 1.], 'orientation': [1., 0., 0., 0.]} for n in names}
        self._assisted_install_alignment = {n: {'complete': True, 'joint_path': n + '_support'} for n in names}
        self.joints_live = {n: True for n in names}
        self.motion = {n: {'position': [0., 0., 1.], 'orientation': [1., 0., 0., 0.],
                          'linear_velocity': [0., 0., 0.], 'angular_velocity': [0., 0., 0.],
                          'linear_speed': 0., 'angular_speed': 0., 'motion_valid': True,
                          'physics_handle_valid': True, 'body_path': '/World/' + n,
                          'motion_view_id': i + 1} for i, n in enumerate(names)}
        self.q = {'right': np.zeros(12), 'left': np.zeros(12)}
        self.robots = {r: NS(articulation=NS(
            get_joint_positions=lambda r=r: self.q[r].copy())) for r in self.q}
        self.phase_spec = {'name': self.phase,
            'lock': [{'object': n} for n in names],
            'gripper_commands': {'right': 'open'},
            'beam_assisted_installed_parts': list(names)}

    def _beam_physics_stamp(self):
        return dict(self.clock)

    def _assisted_native_body_motion(self, name, epoch):
        return copy.deepcopy(self.motion[name])

    def _assisted_support_joint_live(self, name, state, body_path):
        return self.joints_live[name]

    def _assisted_event(self, event, **detail):
        self.events.append((event, detail))

    def _object_velocity_metrics(self, name):
        # This deliberately false cached/lock reading cannot qualify motion.
        return {'valid': True, 'is_static': True, 'pose_stable_override': True}

    def next_tick(self):
        self.clock = stamp(self.clock['index'] + 1, epoch=self.clock['epoch'])
        self.step_counter += 1


class NativeWindowTests(unittest.TestCase):
    def test_same_clock_pose_change_is_not_a_second_valid_observation(self):
        t = FakeTask()
        state = {}
        self.assertTrue(consecutive_native_gate(state, stamp(1), True, 1, t.motion))
        t.motion['fabrica_beam_6']['position'][0] = .000001
        self.assertFalse(consecutive_native_gate(state, stamp(1), True, 1, t.motion))

    def test_repeated_observations_do_not_add_physical_ticks(self):
        state = {}
        for _ in range(100):
            self.assertFalse(consecutive_native_gate(state, stamp(0), True, 12))
        self.assertEqual(state['count'], 1)
        for tick in range(1, 12):
            ready = consecutive_native_gate(state, stamp(tick), True, 12)
        self.assertTrue(ready)

    def test_clock_gap_epoch_and_invalid_time_restart_window(self):
        state = {}
        for tick in range(8):
            consecutive_native_gate(state, stamp(tick), True, 12)
        self.assertFalse(consecutive_native_gate(state, stamp(10), True, 12))
        self.assertEqual(state['count'], 1)
        self.assertFalse(consecutive_native_gate(state, stamp(11, epoch=1), True, 12))
        self.assertEqual(state['count'], 1)
        bad = stamp(11, epoch=1)
        bad['time'] += .01
        self.assertFalse(consecutive_native_gate(state, bad, True, 12))
        self.assertEqual(state['count'], 0)

    def test_false_predicate_clears_qualification(self):
        state = {}
        for tick in range(12):
            consecutive_native_gate(state, stamp(tick), True, 12)
        self.assertFalse(consecutive_native_gate(state, stamp(12), False, 12))
        self.assertEqual(state['count'], 0)

    def test_motion_window_rejects_drift_and_view_replacement(self):
        t = FakeTask()
        state = {}
        for tick in range(10):
            consecutive_native_gate(state, stamp(tick), True, 12, t.motion)
        t.motion['fabrica_beam_6']['position'][0] = .00031
        self.assertFalse(consecutive_native_gate(state, stamp(10), True, 12, t.motion))
        self.assertEqual(state['count'], 0)
        consecutive_native_gate(state, stamp(11), True, 12, t.motion)
        t.motion['fabrica_beam_6']['motion_view_id'] = 99
        self.assertFalse(consecutive_native_gate(state, stamp(12), True, 12, t.motion))


class ReleaseTests(unittest.TestCase):
    def test_opening_and_anchor_command_are_not_release_completion(self):
        t = FakeTask()
        t.q['right'][9] = .2
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        t.q['right'][9] = 0.
        t._assisted_install_alignment['fabrica_beam_6']['complete'] = False
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        t._assisted_install_alignment['fabrica_beam_6']['complete'] = True
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        for _ in range(12):
            t.next_tick()
            result = t._assisted_release_open(t.phase_spec)
        self.assertTrue(result)

    def test_cached_static_flag_cannot_hide_actual_pose_error(self):
        t = FakeTask()
        t.motion['fabrica_beam_6']['position'][0] = .002
        for _ in range(30):
            self.assertFalse(t._assisted_release_open(t.phase_spec))
            t.next_tick()

    def test_velocity_disagreement_is_recorded_and_actual_pose_still_required(self):
        t = FakeTask()
        t.motion['fabrica_beam_6'].update(linear_speed=3., angular_speed=10.,
                                         linear_velocity=[3., 0., 0.], angular_velocity=[10., 0., 0.])
        for _ in range(12):
            result = t._assisted_release_open(t.phase_spec)
            t.next_tick()
        self.assertTrue(result)
        raw = t._assisted_install_gate_sample['objects']['fabrica_beam_6']
        self.assertEqual(raw['linear_speed'], 3.)
        self.assertEqual(raw['velocity_qualification'], 'raw_diagnostic_only_pose_window_required')

    def test_missing_world_support_or_remaining_attachment_rejects_release(self):
        t = FakeTask()
        t.joints_live['fabrica_beam_6'] = False
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        t.joints_live['fabrica_beam_6'] = True
        t._attachments['fabrica_beam_6'] = {'robot_name': 'right'}
        self.assertFalse(t._assisted_release_open(t.phase_spec))

    def test_clock_change_during_native_readback_is_rejected(self):
        t = FakeTask()
        original = t._assisted_native_body_motion
        def read(name, epoch):
            value = original(name, epoch)
            t.next_tick()
            return value
        t._assisted_native_body_motion = read
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        self.assertIn('changed during', t._assisted_install_gate_sample['reason'])

    def test_stale_same_clock_sample_cannot_be_used_on_next_physical_tick(self):
        t = FakeTask()
        t._assisted_release_open(t.phase_spec)
        self.assertTrue(t._assisted_installed_poses_ready(['fabrica_beam_6']))
        t.next_tick()
        self.assertFalse(t._assisted_installed_poses_ready(['fabrica_beam_6']))

    def test_native_reader_checks_live_physics_handle_instead_of_object_pose(self):
        t = FakeTask()
        motion = t.motion['fabrica_beam_6']
        view = NS(is_physics_handle_valid=lambda: True,
            get_world_poses=lambda clone: (np.array([motion['position']]), np.array([motion['orientation']])),
            get_velocities=lambda clone: np.array([[1., 2., 3., 0., 0., 0.]]))
        reader = NS(prim_path=motion['body_path'], _rigid_prim_view=view)
        t._assisted_body_path = lambda name: (NS(unwrap=lambda: reader), motion['body_path'])
        result = BeamAssistedState._assisted_native_body_motion(t, 'fabrica_beam_6', 0)
        self.assertAlmostEqual(result['linear_speed'], np.sqrt(14.))
        view.is_physics_handle_valid = lambda: False
        from toolkits.factory_dual_franka_assembly.beam_support_state import live_body_snapshot
        with self.assertRaises(ValueError):
            live_body_snapshot(view)

    def test_existing_preflight_view_has_priority_and_replacement_is_observed(self):
        t = FakeTask()
        path = '/World/fabrica_beam_6'
        def live_view():
            return NS(is_physics_handle_valid=lambda: True,
                get_world_poses=lambda clone: (np.array([[0., 0., 1.]]), np.array([[1., 0., 0., 0.]])),
                get_velocities=lambda clone: np.zeros((1, 6)))
        old = live_view()
        body = NS(prim_path=path, _rigid_prim_view=old)
        t._assisted_body_path = lambda name: (NS(unwrap=lambda: body), path)
        new = live_view()
        preflight = NS(prim_path=path, _rigid_prim_view=new)
        t._assisted_native_body_readers = {path: NS(unwrap=lambda: preflight)}
        result = BeamAssistedState._assisted_native_body_motion(t, 'fabrica_beam_6', 0)
        self.assertEqual(result['motion_view_id'], id(new))
        self.assertEqual(result['existing_view_source'], 'preflight_native_reader')
        new.is_physics_handle_valid = lambda: False
        result = BeamAssistedState._assisted_native_body_motion(t, 'fabrica_beam_6', 0)
        self.assertEqual(result['motion_view_id'], id(old))
        self.assertEqual(result['existing_view_source'], 'resolved_object')
        old.is_physics_handle_valid = lambda: False
        with self.assertRaisesRegex(ValueError, 'No existing live'):
            BeamAssistedState._assisted_native_body_motion(t, 'fabrica_beam_6', 0)

    def test_native_reader_path_alias_is_rejected_without_fallback(self):
        t = FakeTask()
        path = '/World/fabrica_beam_6'
        t._assisted_body_path = lambda name: (NS(), path)
        t._assisted_native_body_readers = {path: NS(unwrap=lambda: NS(prim_path='/Wrong/body'))}
        with self.assertRaisesRegex(ValueError, 'path alias'):
            BeamAssistedState._assisted_native_body_motion(t, 'fabrica_beam_6', 0)


class FinalTests(unittest.TestCase):
    def test_all_five_bodies_and_both_open_grippers_share_a_native_window(self):
        names = tuple('fabrica_beam_' + n for n in ('6', '3', '2', '1', '0'))
        t = FakeTask(names)
        t.phase_spec = {'name': 'beam_assisted_final_verification',
            'gripper_commands': {'right': 'open', 'left': 'open'},
            'beam_assisted_installed_parts': list(names)}
        t.q['left'][8] = .1
        self.assertFalse(t._assisted_release_open(t.phase_spec))
        t.q['left'][8] = 0.
        advance = {'physics_stable_steps': 240}
        for tick in range(239):
            t.clock = stamp(tick)
            self.assertTrue(t._assisted_release_open(t.phase_spec))
            self.assertFalse(t._assisted_conditions_stable(advance, True))
        t.clock = stamp(239)
        self.assertTrue(t._assisted_release_open(t.phase_spec))
        self.assertTrue(t._assisted_conditions_stable(advance, True))
        t.motion[names[-1]]['position'][0] = .00031
        t.next_tick()
        t._assisted_release_open(t.phase_spec)
        self.assertFalse(t._assisted_conditions_stable(advance, True))

    def test_final_window_does_not_use_table_contact_reader(self):
        t = FakeTask()
        t._beam_table_observation = lambda **kwargs: self.fail('Pure physical support was read')
        t._assisted_install_gate_sample = {'ready': True, 'physics_stamp': t.clock, 'objects': t.motion}
        self.assertTrue(t._assisted_conditions_stable({'physics_stable_steps': 1}, True))


class TaskClockTests(unittest.TestCase):
    @staticmethod
    def clock_method():
        # Execute the real scheduling method without importing Isaac's task
        # base class, so repeated-query behavior can be checked before startup.
        path = Path(__file__).resolve().parents[2] / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
        module = ast.parse(path.read_text())
        task = next(n for n in module.body if isinstance(n, ast.ClassDef)
                    and n.name == 'FactoryDualFrankaAssemblyTask')
        method = next(n for n in task.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_beam_task_clock_event')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        return namespace['_beam_task_clock_event']

    def test_assisted_phase_and_done_queries_are_deduplicated(self):
        run = self.clock_method()
        task = NS(phase_specs=[{'beam_assisted': True}], step_counter=0)
        clock = stamp(0)
        task._beam_physics_stamp = lambda: dict(clock)
        task._set_terminal_state = lambda *a, **k: self.fail('Stable clock was rejected')
        self.assertTrue(run(task, 'update'))
        self.assertFalse(run(task, 'done'))
        for _ in range(20):
            self.assertFalse(run(task, 'update'))
            self.assertFalse(run(task, 'done'))
        clock.update(stamp(1))
        self.assertTrue(run(task, 'update'))
        self.assertTrue(run(task, 'done'))
        self.assertFalse(run(task, 'update'))
        self.assertFalse(run(task, 'done'))

    def test_non_beam_legacy_scheduling_is_preserved(self):
        run = self.clock_method()
        task = NS(phase_specs=[{}])
        self.assertTrue(run(task, 'update'))
        self.assertTrue(run(task, 'update'))


def real_task_method(name):
    path = Path(__file__).resolve().parents[2] / 'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
    source = ast.parse(path.read_text())
    task = next(n for n in source.body if isinstance(n, ast.ClassDef)
                and n.name == 'FactoryDualFrankaAssemblyTask')
    method = next(n for n in task.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {'np': np, 'copy': copy, 'pose_error': pose_error,
                 'pose_within_tolerance': pose_within_tolerance}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class SuccessEntryTask(FakeTask):
    """Use the real task entry methods with controlled existing native snapshots."""
    def __init__(self):
        names = tuple('fabrica_beam_' + n for n in ('6', '3', '2', '1', '0'))
        super().__init__(names)
        criteria = [{'object': n, 'target': n + '_assembled', 'position_tolerance': .003,
                     'orientation_tolerance': .04, 'require_released': True,
                     'require_static': True} for n in names]
        final = {'name': 'beam_assisted_final_verification', 'beam_assisted': True,
            'beam_assisted_installed_parts': list(names),
            'gripper_commands': {'left': 'open', 'right': 'open'},
            'advance': {'type': 'all_of', 'physics_stable_steps': 240, 'conditions': [
                {'type': 'object_targets_reached', 'objects': copy.deepcopy(criteria)},
                {'type': 'objects_static', 'objects': list(names)},
                {'type': 'objects_detached', 'objects': list(names)}]}}
        self.phase_specs = [{'name': 'initial', 'beam_assisted': True}, final]
        self.phase_spec = final
        self.phase = final['name']
        self.phase_index = 1
        self.phase_step_counter = 0
        self.phase_history = [self.phase]
        self.phase_transition_history = []
        self.phase_attempts = {self.phase: 1}
        self.phase_status = 'running'
        self.phase_timeout_count = self.phase_recovery_count = 0
        self._handoff_history = []
        self._recovery_history = []
        self._policy_interaction_history = []
        self._policy_success_stable_count = 0
        self._policy_success_stable_steps = 240
        self.policy_evaluation_mode = False
        self.success = self.failed = False
        self.terminal_reason = None
        self.name = 'test_native_episode'
        self._beam_assisted_enabled = True
        self.cfg = NS(recipe='fabrica_beam_ur5e_assisted', seed=0, layout_seed=505,
                      episode_idx=0, success_criteria=criteria, robot_names=list(self.robots))

    _as_list = staticmethod(lambda v: v if isinstance(v, list) else ([] if v is None else [v]))
    _extract_object_name = staticmethod(lambda v: v.get('object') if isinstance(v, dict) else v)

    def _resolve_object(self, name):
        return NS(get_pose=lambda: (self.motion[name]['position'], self.motion[name]['orientation']))

    def _resolve_target_pose_spec(self, name):
        target = self.target_poses[name]
        return name, target['position'], target['orientation'], target

    def _target_tolerances(self, *, target_spec, default_position_tolerance,
                           default_orientation_tolerance):
        return default_position_tolerance, default_orientation_tolerance

    def _object_velocity_metrics(self, name):
        return {'valid': False, 'is_static': False, 'error': 'legacy velocity unavailable'}

    def _beam_continuing_stability(self):
        return False

    def _phase_payloads_held(self, phase):
        return True

    def _all_physical_grasps(self):
        return []

    def _transition_details(self, **details):
        return details

    def get_tracked_object_states(self):
        return copy.deepcopy(self.motion)

    def get_tracked_robot_states(self):
        return {}

    def _success_diagnostics(self):
        return []


for _method in ('_check_success', '_set_terminal_state', 'calculate_metrics',
                '_evaluate_advance_condition', '_phase_interactions_complete',
                '_advance_condition_met', '_object_targets_reached'):
    setattr(SuccessEntryTask, _method, real_task_method(_method))


class HomeSuccessEntryTask(SuccessEntryTask):
    """The original five-part success fixture with existing native arm views."""
    def __init__(self):
        super().__init__()
        arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        fingers = ['finger_joint', 'right_outer_knuckle_joint', 'left_inner_finger_joint',
                   'right_inner_finger_joint', 'left_inner_finger_knuckle_joint',
                   'right_inner_finger_knuckle_joint']
        self.live = {name: True for name in self.robots}
        self.home_views = {}
        for name, robot in self.robots.items():
            robot.config = NS(initial_joint_positions={j: 0. for j in arm + ['finger_joint']},
                              gripper_open_position=0.)
            view = NS(is_physics_handle_valid=lambda name=name: self.live[name],
                      get_joint_positions=lambda clone, name=name: self.q[name][None].copy())
            robot.articulation.dof_names = list(arm + fingers)
            robot.articulation._articulation_view = view
            self.home_views[name] = view
        self.phase_spec['beam_assisted_home_targets'] = {
            name: {'joint_names': list(arm), 'joint_positions': [0.] * 6,
                   'tolerance_rad': .01} for name in self.robots}


class HomeQualificationTests(unittest.TestCase):
    def sample(self, task):
        return task._assisted_installation_sample(
            task.phase_spec['beam_assisted_installed_parts'], task.clock, final=True)

    def qualify(self, task):
        for index in range(240):
            task.clock = stamp(index)
            task.step_counter = index + 1
            self.assertEqual(task._advance_condition_met(task.phase_spec), index == 239)
        self.assertTrue(task._check_success())

    def test_home_reads_both_existing_native_views_on_raw_initial_branch(self):
        task = HomeSuccessEntryTask()
        sample = self.sample(task)
        self.assertTrue(sample['ready'])
        self.assertEqual(set(sample['home']['robots']), {'left', 'right'})
        for name, arm in sample['home']['robots'].items():
            self.assertEqual(arm['motion_view_id'], id(task.home_views[name]))
            self.assertEqual(arm['angle_comparison'], 'raw_native_branch_no_modulo')
        self.qualify(task)
        self.assertTrue(task._assisted_final_certificate['native_home']['ready'])

    def test_home_rejects_missing_live_view_one_arm_offset_modulo_and_declared_alias(self):
        for change in ('view', 'invalid_handle', 'single_arm', 'modulo', 'target',
                       'missing_robot', 'joint_alias', 'shape', 'nonfinite', 'clock', 'robot_alias'):
            with self.subTest(change=change):
                task = HomeSuccessEntryTask()
                if change == 'view':
                    del task.robots['right'].articulation._articulation_view
                elif change == 'invalid_handle':
                    task.live['left'] = False
                elif change == 'single_arm':
                    task.q['right'][2] = .01001
                elif change == 'modulo':
                    task.q['left'][5] = 2 * np.pi
                elif change == 'target':
                    task.phase_spec['beam_assisted_home_targets']['left']['joint_positions'][0] = .001
                elif change == 'missing_robot':
                    del task.phase_spec['beam_assisted_home_targets']['right']
                elif change == 'joint_alias':
                    task.robots['left'].articulation.dof_names[0] = 'wrist_3_joint'
                elif change == 'robot_alias':
                    task.robots['right'].articulation = task.robots['left'].articulation
                elif change == 'shape':
                    task.home_views['left'].get_joint_positions = lambda clone: np.zeros((12,))
                elif change == 'nonfinite':
                    task.q['right'][0] = np.nan
                elif change == 'clock':
                    task.clock['valid'] = False
                self.assertFalse(self.sample(task)['ready'])
                self.assertFalse(task._check_success())

    def test_home_readback_rejects_physics_clock_change(self):
        task = HomeSuccessEntryTask()
        def read(clone):
            task.next_tick()
            return task.q['left'][None].copy()
        task.home_views['left'].get_joint_positions = read
        self.assertFalse(self.sample(task)['ready'])

    def test_arm_drift_resets_the_original_240_sample_window(self):
        task = HomeSuccessEntryTask()
        for index in range(150):
            task.clock = stamp(index)
            self.assertFalse(task._advance_condition_met(task.phase_spec))
        self.assertEqual(task._assisted_final_gate['count'], 150)
        task.q['left'][1] = .02
        task.clock = stamp(150)
        self.assertFalse(task._advance_condition_met(task.phase_spec))
        self.assertEqual(task._assisted_final_gate['count'], 0)
        task.q['left'][1] = 0.
        for index in range(151, 390):
            task.clock = stamp(index)
            self.assertFalse(task._advance_condition_met(task.phase_spec))
        task.clock = stamp(390)
        self.assertTrue(task._advance_condition_met(task.phase_spec))

    def test_duplicate_home_tick_does_not_advance_and_changed_sample_resets(self):
        task = HomeSuccessEntryTask()
        for _ in range(100):
            self.assertFalse(task._advance_condition_met(task.phase_spec))
        self.assertEqual(task._assisted_final_gate['count'], 1)
        task.q['left'][0] = .0001
        self.assertFalse(task._advance_condition_met(task.phase_spec))
        self.assertEqual(task._assisted_final_gate['count'], 0)

    def test_completed_success_rejects_arm_drift_or_replaced_view_at_same_tick(self):
        for change in ('within_tolerance_drift', 'outside_tolerance', 'view', 'missing_certificate'):
            with self.subTest(change=change):
                task = HomeSuccessEntryTask()
                self.qualify(task)
                task._set_terminal_state('complete', reason='success-criteria-met', status='success')
                self.assertTrue(task.calculate_metrics()['assisted_success'])
                if change == 'within_tolerance_drift':
                    task.q['right'][0] = .001
                elif change == 'outside_tolerance':
                    task.q['left'][5] = .02
                elif change == 'view':
                    task.robots['left'].articulation._articulation_view = NS(
                        is_physics_handle_valid=lambda: True,
                        get_joint_positions=lambda clone: task.q['left'][None].copy())
                else:
                    task._assisted_final_certificate.pop('native_home')
                self.assertFalse(task.calculate_metrics()['assisted_success'])


class SuccessEntryTests(unittest.TestCase):
    def qualify(self, task):
        for index in range(240):
            task.clock = stamp(index)
            task.step_counter = index + 1
            ready = task._advance_condition_met(task.phase_spec)
            self.assertEqual(ready, index == 239)
        self.assertTrue(task._check_success())

    def complete(self, task):
        self.qualify(task)
        task._set_terminal_state('complete', reason='success-criteria-met', status='success')

    def test_real_success_entry_requires_all_240_samples_and_ignores_legacy_false_static(self):
        task = SuccessEntryTask()
        for index in range(239):
            task.clock = stamp(index)
            task.step_counter = index + 1
            self.assertFalse(task._advance_condition_met(task.phase_spec))
            self.assertFalse(task._check_success())
        task.clock = stamp(239)
        self.assertTrue(task._advance_condition_met(task.phase_spec))
        self.assertTrue(task._check_success())
        self.assertFalse(task._assisted_success_native_sample['objects']['fabrica_beam_6']['legacy_static_reading']['valid'])

    def test_real_terminal_transition_preserves_metrics_only_with_current_native_certificate(self):
        task = SuccessEntryTask()
        self.complete(task)
        self.assertEqual(task.phase_index, len(task.phase_specs))
        metrics = task.calculate_metrics()
        self.assertTrue(metrics['assisted_success'])
        self.assertTrue(metrics['success_detector']['criteria_passed'])
        self.assertFalse(metrics['beam_exploration']['formal_success'])
        self.assertEqual(task._assisted_final_gate['count'], 240)
        qualification = metrics['assisted_final_qualification']
        self.assertTrue(qualification['current_native_recheck']['ready'])
        self.assertEqual(len(qualification['current_native_sample']['objects']), 5)
        self.assertEqual(qualification['certificate']['physics_stamp'], task.clock)
        self.assertTrue(task.calculate_metrics()['assisted_success'])
        self.assertEqual(task._assisted_final_gate['count'], 240)
        task.next_tick()
        self.assertFalse(task.calculate_metrics()['assisted_success'])

    def test_incidental_intermediate_pose_and_forged_terminal_flag_do_not_succeed(self):
        task = SuccessEntryTask()
        task.phase_index = 0
        task.phase = 'initial'
        self.assertFalse(task._check_success())
        task._set_terminal_state('complete', reason='success-criteria-met', status='success')
        self.assertFalse(task.calculate_metrics()['assisted_success'])

    def test_terminal_certificate_rejects_closed_gripper_attachment_missing_support_or_clock(self):
        for change in ('gripper', 'attachment', 'support', 'clock', 'view', 'pose', 'episode', 'phase'):
            with self.subTest(change=change):
                task = SuccessEntryTask()
                self.complete(task)
                if change == 'gripper':
                    task.q['left'][9] = .2
                elif change == 'attachment':
                    task._attachments['fabrica_beam_0'] = {'robot_name': 'left'}
                elif change == 'support':
                    task.joints_live['fabrica_beam_6'] = False
                elif change == 'clock':
                    task.clock['valid'] = False
                elif change == 'view':
                    task.motion['fabrica_beam_3']['motion_view_id'] += 1
                elif change == 'pose':
                    task.motion['fabrica_beam_2']['position'][0] = .00001
                elif change == 'episode':
                    task.cfg.episode_idx += 1
                elif change == 'phase':
                    task.phase = 'initial'
                    task.phase_index = 0
                self.assertFalse(task._check_success())
                self.assertFalse(task.calculate_metrics()['assisted_success'])

    def test_native_view_read_failure_cannot_be_hidden_by_completed_flag(self):
        task = SuccessEntryTask()
        self.complete(task)
        def unavailable(name, epoch):
            raise ValueError('existing live native view unavailable')
        task._assisted_native_body_motion = unavailable
        self.assertFalse(task._check_success())
        self.assertIn('readback rejected', task._assisted_success_last_decision['reason'])

    def test_physical_success_entry_keeps_its_original_static_gate(self):
        task = SuccessEntryTask()
        task._beam_assisted_enabled = False
        task._assisted_final_success_ready = lambda: self.fail('Physical route used assisted qualification')
        task._object_velocity_metrics = lambda name, **kwargs: {'valid': False, 'is_static': False}
        self.assertFalse(task._check_success())
        task._object_velocity_metrics = lambda name, **kwargs: {'valid': True, 'is_static': True}
        self.assertTrue(task._check_success())


if __name__ == '__main__':
    unittest.main()
