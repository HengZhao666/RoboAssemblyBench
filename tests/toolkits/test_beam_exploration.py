import copy
import json
import math
import unittest
from pathlib import Path

from toolkits.factory_dual_franka_assembly.beam_exploration import (
    ENV_NAME, FORBIDDEN_RELEASE_PHASE, SEAT_PHASE, STOP_PHASE,
    exploration_guard_decision, exploration_seat_decision, parse_exploration_environment,
)


def sample(index, force=2.45, gap=0., position=(0., 0., 1.), orientation=(1., 0., 0., 0.)):
    return {'valid': True, 'motion_valid': True, 'physics_stamp_valid': True,
            'physics_epoch': 0, 'physics_step_index': index, 'physics_time': index / 240.,
            'physics_dt': 1 / 240., 'vertical_force': abs(force), 'bottom_gap': gap,
            'position': list(position), 'orientation': list(orientation),
            'linear_speed': .01, 'angular_speed': .35,
            'support_surfaces': [{'object': 'board', 'contact': {
                'valid': True, 'force_world': [0., 0., force]}}]}


def pose_result(data, ready=True):
    """A same-frame tracker contract; real window behavior has separate tests."""
    return {'valid': True, 'ready': ready, 'motion_windows_ready': ready,
            'latched_failure': False,
            'physics_stamp': {key: data[key] for key in (
                'physics_epoch', 'physics_step_index', 'physics_time', 'physics_dt')},
            'anchor': {'captured': ready, 'within_budget': True,
                       'actor_position': [0., 0., 1.], 'orientation': [1., 0., 0., 0.],
                       'physics_epoch': data['physics_epoch'],
                       'physics_step_index': 0, 'physics_time': 0.}}


def seat(state, data, phase_name=SEAT_PHASE, generations=None, enabled=True,
         pose=None, original_ready=False):
    return exploration_seat_decision(state, data, phase_name,
        {} if generations is None else generations, enabled,
        pose_result(data) if pose is None and isinstance(data, dict) else pose,
        original_ready=original_ready)


def qualify(state=None, generations=None):
    state = {} if state is None else state
    generations = {} if generations is None else generations
    for index in range(96):
        result = seat(state, sample(index), generations=generations)
    return state, result


class BeamExplorationTests(unittest.TestCase):
    def test_environment_exact_opt_in(self):
        self.assertFalse(parse_exploration_environment({}))
        self.assertFalse(parse_exploration_environment({ENV_NAME: '0'}))
        self.assertTrue(parse_exploration_environment({ENV_NAME: '1'}))
        for value in ('', 'true', 'yes', ' 1', '1 ', '2', 1, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_exploration_environment({ENV_NAME: value})

    def test_continuation_is_explicit_and_requires_pose_exploration(self):
        from toolkits.factory_dual_franka_assembly.beam_exploration import parse_exploration_continuation
        self.assertFalse(parse_exploration_continuation({}, True))
        self.assertTrue(parse_exploration_continuation({'BEAM_EXPLORE_CONTINUE': '1'}, True))
        for enabled, value in [(False, '1'), (True, 'true'), (True, ''), (True, 1)]:
            with self.assertRaises(ValueError):
                parse_exploration_continuation({'BEAM_EXPLORE_CONTINUE': value}, enabled)

    def test_continuation_records_first_insertion_and_allows_all_later_phases(self):
        state, _ = qualify({'continue_after_first_insertion': True})
        decision = exploration_guard_decision(state, sample(96), STOP_PHASE, {}, True, phase_complete=True)
        self.assertEqual(decision['mode'], 'guarding')
        self.assertTrue(state['first_insertion_complete'])
        self.assertFalse(state.get('complete', False))
        for index, phase in enumerate([FORBIDDEN_RELEASE_PHASE, 'hold_3_descend',
                'assemble_01_part_2_move_above', 'beam_final_unloaded_verification'], 97):
            self.assertEqual(exploration_guard_decision(state, sample(index), phase, {}, True)['mode'], 'guarding')

    def test_continuation_preserves_load_pose_clock_and_unexpected_loss_rejections(self):
        for kind in ['load', 'pose', 'clock', 'loss', 'integrity']:
            state, _ = qualify({'continue_after_first_insertion': True})
            data = sample(96)
            generations = {}
            if kind == 'load': data['vertical_force'] = .1; data['support_surfaces'][0]['contact']['force_world'][2] = .1
            if kind == 'pose': data['position'][0] = .0011
            if kind == 'clock': data = sample(98)
            if kind == 'loss': generations['fabrica_beam_3'] = 1
            decision = exploration_guard_decision(state, data, FORBIDDEN_RELEASE_PHASE,
                generations, True, integrity_valid=kind != 'integrity')
            self.assertEqual(decision['mode'], 'failure', kind)

    def test_disabled_is_inert_even_without_a_sample(self):
        state = {'keep': [1, 2]}
        old = copy.deepcopy(state)
        self.assertEqual(exploration_seat_decision(state, None, None, None, False, None)['mode'], 'disabled')
        self.assertEqual(exploration_guard_decision(state, None, None, None, False)['mode'], 'disabled')
        self.assertEqual(state, old)

    def test_pre_seat_wait_does_not_read_physics(self):
        state = {}
        self.assertEqual(exploration_guard_decision(state, None, 'base_6_descend', None, True)['mode'], 'waiting')
        self.assertEqual(state, {})

    def test_qualifies_96_fresh_samples_despite_raw_speed_rejection(self):
        state = {}
        for index in range(95):
            self.assertFalse(seat(state, sample(index))['ready'])
        result = seat(state, sample(95))
        self.assertTrue(result['ready'])
        self.assertEqual(result['qualified_steps'], 96)
        self.assertTrue(result['marker']['exploration_used'])
        self.assertFalse(result['marker']['formal_success'])

    def test_original_gate_never_bypasses_pose_and_load_qualification(self):
        state = {}
        for index in range(96):
            data = sample(index)
            result = seat(state, data, pose=pose_result(data, ready=False), original_ready=True)
            self.assertFalse(result['ready'])
        self.assertEqual(result['qualified_steps'], 0)
        for index in range(96, 192):
            result = seat(state, sample(index), original_ready=True)
            if index < 191:
                self.assertFalse(result['ready'])
        self.assertTrue(result['ready'])
        self.assertFalse(result['marker']['exploration_used'])
        self.assertTrue(result['marker']['original_seat_ready'])

    def test_duplicate_never_accumulates_and_changed_duplicate_fails(self):
        state = {}
        for _ in range(100):
            result = seat(state, sample(0))
        self.assertEqual(result['qualified_steps'], 1)
        changed = sample(0, force=2.5)
        self.assertEqual(seat(state, changed)['mode'], 'failure')

    def test_force_or_gap_breaks_continuous_qualification(self):
        for bad in (sample(70, force=1.96), sample(70, gap=.00101)):
            state = {}
            for index in range(70):
                seat(state, sample(index))
            self.assertEqual(seat(state, bad)['qualified_steps'], 0)
            for index in range(71, 96):
                result = seat(state, sample(index))
            self.assertFalse(result['ready'])
            self.assertEqual(result['qualified_steps'], 25)

    def test_external_pose_failure_and_same_frame_contract_fail_closed(self):
        mutations = [lambda d: d.update(valid=False),
                     lambda d: d.update(latched_failure='anchor_budget_exceeded'),
                     lambda d: d.update(physics_stamp=None),
                     lambda d: d['physics_stamp'].update(physics_step_index=99),
                     lambda d: d.update(motion_windows_ready=False),
                     lambda d: d.update(anchor=None),
                     lambda d: d['anchor'].update(captured=False),
                     lambda d: d['anchor'].update(actor_position=[0., math.nan, 1.]),
                     lambda d: d['anchor'].update(physics_epoch=1)]
        for mutate in mutations:
            data = sample(0); pose = pose_result(data); mutate(pose)
            self.assertEqual(seat({}, data, pose=pose)['mode'], 'failure')

    def test_short_pose_result_never_substitutes_for_long_window(self):
        state = {}
        for index in range(96):
            data = sample(index)
            pose = pose_result(data, ready=False)
            pose.update(short_window={'ready': True}, long_window={'ready': False})
            result = seat(state, data, pose=pose)
        self.assertFalse(result['ready'])
        self.assertEqual(result['qualified_steps'], 0)
        self.assertEqual(result['reason'], 'pose_window_unqualified')
        self.assertFalse(result['pose_motion']['long_window']['ready'])

    def test_one_pose_rejection_resets_joint_96_frame_qualification(self):
        state = {}
        for index in range(95):
            result = seat(state, sample(index))
        self.assertFalse(result['ready'])
        self.assertEqual(result['qualified_steps'], 95)
        data = sample(95)
        result = seat(state, data, pose=pose_result(data, ready=False))
        self.assertFalse(result['ready'])
        self.assertEqual(result['qualified_steps'], 0)
        for index in range(96, 191):
            result = seat(state, sample(index))
        self.assertFalse(result['ready'])
        self.assertEqual(result['qualified_steps'], 95)
        for _ in range(3):
            self.assertEqual(seat(state, sample(190))['qualified_steps'], 95)
        result = seat(state, sample(191))
        self.assertTrue(result['ready'])
        self.assertEqual(result['qualified_steps'], 96)

    def test_real_tracker_requires_two_seconds_and_preserves_actor_anchor(self):
        from toolkits.factory_dual_franka_assembly.beam_pose_stability import PoseStabilityTracker
        tracker, state = PoseStabilityTracker(), {}
        for index in range(576):
            data = sample(index)
            # Deliberately distinct COM and actor origins catch frame mixing.
            pose = tracker.observe(data, com_position=[0., 0., 1.006321],
                                   body_id='/World/beam_6', view_id=7, anchor_eligible=True)
            result = seat(state, data, pose=pose, original_ready=True)
            if index < 575:
                self.assertFalse(result['ready'])
            if index == 479:
                self.assertEqual(result['qualified_steps'], 0)
            if index == 480:
                self.assertEqual(result['qualified_steps'], 1)
            if index == 574:
                self.assertEqual(result['qualified_steps'], 95)
        self.assertTrue(result['ready'])
        self.assertEqual(state['capture_clock'], (0, 480, 2., 1/240.))
        self.assertEqual(state['anchor']['position'], (0., 0., 1.))
        self.assertEqual(state['gate_clock'], (0, 575, 575/240., 1/240.))
        self.assertEqual(exploration_guard_decision(state, sample(576), STOP_PHASE, {}, True)['mode'], 'guarding')

    def test_clock_gaps_epoch_dt_changes_fail_closed(self):
        for kind in ('gap', 'epoch', 'dt', 'time'):
            state = {}
            seat(state, sample(0))
            bad = sample(1)
            if kind == 'gap': bad = sample(2)
            if kind == 'epoch': bad['physics_epoch'] = 1
            if kind == 'dt': bad['physics_dt'] *= 2
            if kind == 'time': bad['physics_time'] += .01
            self.assertEqual(seat(state, bad)['mode'], 'failure')
            self.assertEqual(seat(state, sample(1))['mode'], 'failure')

    def test_abs_downward_force_is_never_support(self):
        result = seat({}, sample(0, force=-2.45))
        self.assertEqual(result['reason'], 'support_force_wrong_direction')

    def test_malformed_pose_contact_and_clock_fail_closed(self):
        mutations = [lambda d: d.update(motion_valid=False),
                     lambda d: d.update(position=[0., math.nan, 1.]),
                     lambda d: d.update(orientation=[0., 0., 0., 0.]),
                     lambda d: d.update(angular_speed=math.inf),
                     lambda d: d['support_surfaces'][0]['contact'].update(valid=False),
                     lambda d: d['support_surfaces'].append(copy.deepcopy(d['support_surfaces'][0])),
                     lambda d: d.update(physics_step_index=True),
                     lambda d: d.update(physics_stamp_after={'valid': True, 'epoch': 0, 'index': 1, 'time': 0., 'dt': 1/240.})]
        for mutate in mutations:
            bad = sample(0); mutate(bad)
            self.assertEqual(seat({}, bad)['mode'], 'failure')
        self.assertEqual(seat({}, None)['mode'], 'failure')

    def test_qualification_is_rechecked_until_phase_advances(self):
        state, _ = qualify()
        self.assertTrue(seat(state, sample(96))['ready'])
        self.assertTrue(seat(state, sample(97))['ready'])
        self.assertEqual(exploration_guard_decision(state, sample(98), STOP_PHASE, {}, True)['mode'], 'guarding')
        for kind in ('load', 'pose', 'generation'):
            state, _ = qualify()
            data = sample(96, force=1.9 if kind == 'load' else 2.45)
            result = seat(state, data, pose=pose_result(data, ready=kind != 'pose'),
                          generations={'fabrica_beam_6': 1} if kind == 'generation' else {})
            self.assertEqual(result['mode'], 'failure')

    def test_guard_allows_moving_part3_and_robot_observations(self):
        state, _ = qualify()
        data = sample(96)
        data.update(part3_position=[8., 9., 10.], robot_joint_motion=[1., 2., 3.])
        result = exploration_guard_decision(state, data, 'assemble_00_part_3_insert_00', {}, True)
        self.assertEqual(result['mode'], 'guarding')

    def test_loss_generation_increment_stops_before_next_action(self):
        state, _ = qualify(generations={'fabrica_beam_6': 2})
        result = exploration_guard_decision(state, sample(96), 'assemble_00_part_3_move_above',
                                            {'fabrica_beam_6': 2, 'fabrica_beam_3': 1}, True)
        self.assertEqual(result['mode'], 'failure')
        self.assertIn('grasp_loss', result['reason'])

    def test_unchanged_generation_allows_explicit_release_contract(self):
        state, _ = qualify(generations={'fabrica_beam_6': 2})
        self.assertEqual(exploration_guard_decision(state, sample(96), STOP_PHASE,
            {'fabrica_beam_6': 2}, True, phase_complete=True)['mode'], 'complete')
        # The helper monitors unexpected-loss generations, not symbolic removal;
        # the phase boundary still prevents any later base release action.
        self.assertEqual(exploration_guard_decision(state, sample(96), FORBIDDEN_RELEASE_PHASE,
            {'fabrica_beam_6': 2}, True)['mode'], 'failure')

    def test_post_guard_uses_40N_ceiling_not_seat_10N(self):
        state, _ = qualify()
        result = exploration_guard_decision(state, sample(96, force=12.26), STOP_PHASE, {}, True)
        self.assertEqual(result['mode'], 'guarding')
        state, _ = qualify()
        self.assertEqual(exploration_guard_decision(state, sample(96, force=40.01), STOP_PHASE, {}, True)['reason'], 'support_overload')

    def test_support_loss_gap_penetration_drift_and_integrity_stop_immediately(self):
        cases = [(sample(96, force=.49), True, 'support_lost'),
                 (sample(96, gap=.00151), True, 'support_gap_out_of_bounds'),
                 (sample(96, gap=-.00101), True, 'support_penetration'),
                 (sample(96, position=(.00101, 0., 1.)), True, 'seated_base_displaced'),
                 (sample(96, orientation=(math.cos(.006), 0., 0., math.sin(.006))), True, 'seated_base_displaced'),
                 (sample(96), False, 'integrity_or_completion_invalid')]
        for data, integrity, reason in cases:
            state, _ = qualify()
            result = exploration_guard_decision(state, data, STOP_PHASE, {}, True, integrity_valid=integrity)
            self.assertEqual(result['reason'], reason)
            self.assertEqual(result['mode'], 'failure')

    def test_completion_requires_correct_phase_original_completion_and_guard(self):
        state, _ = qualify()
        result = exploration_guard_decision(state, sample(96), 'assemble_00_part_3_insert_09', {}, True, phase_complete=True)
        self.assertEqual(result['mode'], 'guarding')
        self.assertEqual(exploration_guard_decision(state, sample(97), STOP_PHASE, {}, True)['mode'], 'guarding')
        result = exploration_guard_decision(state, sample(97), STOP_PHASE, {}, True, phase_complete=True)
        self.assertEqual(result['mode'], 'complete')
        self.assertFalse(result['marker']['formal_success'])
        self.assertTrue(result['marker']['first_insertion_complete'])

    def test_phase_bounds_and_missing_anchor_fail_closed(self):
        self.assertEqual(seat({}, sample(0), STOP_PHASE)['mode'], 'failure')
        self.assertEqual(exploration_guard_decision({}, sample(0), STOP_PHASE, {}, True)['reason'], 'exploration_seat_not_qualified')
        for phase in (FORBIDDEN_RELEASE_PHASE, 'assemble_00_part_3_restore_gravity', 'assemble_01_part_2_move_above'):
            state, _ = qualify()
            self.assertEqual(exploration_guard_decision(state, sample(96), phase, {}, True)['reason'], 'exploration_stop_boundary_exceeded')

    def test_authoritative_compiled_phase_prefix_is_fully_guarded(self):
        names = json.loads((Path(__file__).parent / 'fixtures/beam_v169_recipe_contract.json').read_text())['phase_order']
        seat_index, stop_index = names.index(SEAT_PHASE), names.index(STOP_PHASE)
        state = {}
        for name in names[:seat_index]:
            self.assertEqual(exploration_guard_decision(state, None, name, None, True)['mode'], 'waiting', name)
        state, result = qualify(state)
        self.assertTrue(result['ready'])
        for index, name in enumerate(names[seat_index:stop_index + 1], 96):
            result = exploration_guard_decision(state, sample(index), name, {}, True,
                                                phase_complete=name == STOP_PHASE)
            self.assertEqual(result['mode'], 'complete' if name == STOP_PHASE else 'guarding', name)
        self.assertEqual(exploration_guard_decision(state, sample(index + 1), names[stop_index + 1], {}, True)['mode'], 'failure')


if __name__ == '__main__':
    unittest.main()
