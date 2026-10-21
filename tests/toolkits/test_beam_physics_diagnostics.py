"""Boundary captures must preserve live buffers and never alter physics."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

import numpy as np

from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import (
    record_path_state_write, record_state_write, sample_boundary, _joint_command_targets,
    _body_com_snapshot, _native_scene_gravity, _pair_load_snapshot,
)


class BodyView:
    def __init__(self, valid=True, mutate=False):
        self.valid = valid
        self.positions = np.array([[1., 2., 3.]])
        self.orientations = np.array([[1., 0., 0., 0.]])
        self.velocities = np.array([[.1, .2, .3, .4, .5, .6]])
        self.mutate = mutate
        self.pose_calls = 0
        self.metadata_calls = 0

    def is_physics_handle_valid(self):
        return self.valid

    def get_world_poses(self, clone):
        self.pose_calls += 1
        return self.positions, self.orientations

    def get_velocities(self, clone):
        if self.mutate:
            self.positions[:] = 9
            self.orientations[:] = 9
        return self.velocities

    def get_masses(self, clone):
        self.metadata_calls += 1
        if self.mutate:
            self.velocities[:] = 9
        return np.array([.25])

    def get_inertias(self, clone):
        return np.array([[1., 0., 0., 0., 2., 0., 0., 0., 3.]])

    def get_coms(self, clone):
        return np.array([[.01, .02, .03]]), np.array([[1., 0., 0., 0.]])

    def get_sleep_thresholds(self):
        return np.array([1.e-5])


class JointView:
    def __init__(self, valid=True, mutate=False):
        self.valid = valid
        self.positions = np.arange(12, dtype=float).reshape(1, 12)
        self.velocities = np.arange(12, dtype=float).reshape(1, 12) / 10
        self.mutate = mutate

    def is_physics_handle_valid(self):
        return self.valid

    def get_joint_positions(self, clone):
        return self.positions

    def get_joint_velocities(self, clone):
        if self.mutate:
            self.positions[:] = 9
        return self.velocities

    def get_sleep_thresholds(self):
        return np.array([5.e-5])


def wrapper(view, path='/World/beam_6'):
    raw = NS(_rigid_prim_view=view, prim_path=path)
    return NS(unwrap=lambda: raw)


def task():
    body = BodyView()
    robot = NS(config=NS(left_finger_link_name='left_inner_finger',
                         right_finger_link_name='right_inner_finger'),
               _rigid_body_map={}, articulation=NS(_articulation_view=JointView()))
    for link in ('base_link', 'left_inner_finger', 'right_inner_finger'):
        path = '/World/right/Robotiq_2F_85/' + link
        robot._rigid_body_map[path] = wrapper(BodyView(), path)
    return NS(name='beam_test', step_counter=9000, phase='base_6_set_down',
              _resolved_objects={'fabrica_beam_6': wrapper(body)},
              robots={'right': robot}, _contact_probes={},
              _locked_targets={}, _lock_pin_pose={}, _attachment_joints={},
              _attachments={'fabrica_beam_6': {'mode': 'pure_physical_grasp',
                                            'robot_name': 'right', 'attach_step': 500}},
              _beam_shared_grasps={})


def clock():
    physics = NS(is_stabilization_enabled=lambda: True, get_solver_type=lambda: 'TGS',
                 is_gpu_dynamics_enabled=lambda: False)
    return NS(current_time_step_index=9003, current_time=9003/240,
              get_physics_dt=lambda: 1/240, get_physics_context=lambda: physics)


class PhysicsDiagnostics(unittest.TestCase):
    def setUp(self):
        self.paths_patch = patch.dict(
            'toolkits.factory_dual_franka_assembly.beam_physics_diagnostics._PATH_TASKS', {}, clear=True)
        self.paths_patch.start()
        self.addCleanup(self.paths_patch.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'boundaries.jsonl'
        self.env = {'BEAM_PHYSICS_DIAGNOSTICS': '1',
                    'BEAM_PHYSICS_DIAGNOSTICS_PATH': str(self.path)}
        self.patch = patch.dict(os.environ, self.env, clear=True)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.task = task()
        self.clock = clock()

    def sample(self, boundary='before_action', **kwargs):
        return sample_boundary(self.task, boundary, self.clock, **kwargs)

    def test_continuation_net_loads_cover_release_and_later_parts_without_new_views(self):
        self.task.phase = 'hold_6_to_3_switch_release'
        self.task._beam_exploration_state = {'continue_after_first_insertion': True, 'anchor': {'valid': True}}
        path = '/World/fabrica_beam_2'
        self.task._resolved_objects = {'fabrica_beam_2': wrapper(BodyView(), path)}
        existing = object()
        self.task._contact_probes = {(path, '/World/table'): existing}
        module = 'toolkits.factory_dual_franka_assembly.beam_physics_diagnostics'
        settings = {'BEAM_PHYSICS_DIAGNOSTICS_PHASES': '*', 'BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED': '1',
                    'BEAM_PHYSICS_DIAGNOSTICS_OBJECTS': 'fabrica_beam_2'}
        with patch.dict(os.environ, settings), patch(module + '._pair_load_snapshot', return_value={'valid': True}), \
                patch(module + '._unfiltered_contact_snapshot', return_value={'valid': True, 'sensor_body': path}) as reader:
            for boundary in ('before_action', 'after_action', 'after_physics', 'after_observation'):
                record = self.sample(boundary, cycle_step=9000)
                self.assertEqual(record['contact_loads']['unfiltered_sensors'][0]['sensor_body'], path)
            self.assertEqual(reader.call_count, 4)
        self.assertEqual(self.task._contact_probes, {(path, '/World/table'): existing})

    def test_continuation_net_scope_requires_optin_anchor_and_postseat_phase(self):
        from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import _unfiltered_phase_selected
        self.task._beam_exploration_state = {'continue_after_first_insertion': True, 'anchor': {'valid': True}}
        with patch.dict(os.environ, {'BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED': '1'}):
            for phase in ['hold_3_close_and_attach', 'assemble_01_part_2_lift', 'handover_left_2_release', 'beam_final_unloaded_verification']:
                self.assertTrue(_unfiltered_phase_selected(self.task, phase))
            self.assertFalse(_unfiltered_phase_selected(self.task, 'base_6_set_down'))
            self.task._beam_exploration_state.pop('anchor')
            self.assertFalse(_unfiltered_phase_selected(self.task, 'hold_3_close_and_attach'))
            self.assertTrue(_unfiltered_phase_selected(self.task, 'assemble_00_part_3_assembly_clearance'))
        with patch.dict(os.environ, {'BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED': '0'}):
            self.assertFalse(_unfiltered_phase_selected(self.task, 'assemble_00_part_3_assembly_clearance'))

    def test_unfiltered_turn_loads_exist_at_image_clock_and_following_clock(self):
        self.task.phase = 'assemble_00_part_3_assembly_clearance'
        path = '/World/fabrica_beam_6'
        self.task._resolved_objects = {'fabrica_beam_6': wrapper(BodyView(), path)}
        self.task._contact_probes[(path, '/World/table')] = object()
        settings = {'BEAM_PHYSICS_DIAGNOSTICS_PHASES': '*',
                    'BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED': '1'}
        module = 'toolkits.factory_dual_franka_assembly.beam_physics_diagnostics'
        with patch.dict(os.environ, settings), \
                patch(module + '._pair_load_snapshot', return_value={'valid': True}), \
                patch(module + '._unfiltered_contact_snapshot',
                      return_value={'valid': True, 'sensor_body': path}) as reader:
            for boundary in ('before_action', 'after_action', 'after_physics', 'after_observation'):
                with self.subTest(boundary=boundary):
                    record = self.sample(boundary, cycle_step=9000)
                    self.assertEqual(record['contact_loads']['unfiltered_sensors'][0]['sensor_body'], path)
            self.assertEqual(reader.call_count, 4)

    def test_joint_names_match_native_buffers_and_bad_mapping_stays_unknown(self):
        articulation = self.task.robots['right'].articulation
        articulation.dof_names = ['joint_' + str(i) for i in range(12)]
        record = self.sample()
        joints = record['robots']['right']
        self.assertEqual(joints['joint_names'], articulation.dof_names)
        self.assertTrue(joints['valid'])
        articulation.dof_names = ['same'] * 12
        record = self.sample('after_action')
        joints = record['robots']['right']
        self.assertIn('joint_names_error', joints)
        self.assertNotIn('joint_names', joints)
        self.assertTrue(joints['valid'])

    def test_copies_every_component_with_units_and_live_source(self):
        record = self.sample()
        self.assertTrue(record['valid'])
        self.assertEqual(record['physics_step_index'], 9003)
        self.assertEqual(record['physics_dt'], 1/240)
        body = record['bodies'][0]
        self.assertEqual(body['linear_velocities'], [[.1, .2, .3]])
        self.assertEqual(body['angular_velocities'], [[.4, .5, .6]])
        self.assertEqual(body['velocity_reference'], 'center_of_mass')
        self.assertEqual(body['center_of_mass']['world_positions'], [[1.01, 2.02, 3.03]])
        self.assertEqual(body['metadata']['masses'], [.25])
        self.assertEqual(len(record['robots']['right']['joint_positions'][0]), 12)
        self.assertEqual(len(record['bodies']), 4)
        self.assertTrue(record['global_stabilization']['enabled'])
        self.assertEqual(record['global_physics_settings']['solver_type']['value'], 'TGS')
        self.assertFalse(record['global_physics_settings']['gpu_dynamics_enabled']['value'])
        self.assertEqual(body['metadata']['sleep_thresholds'], [1.e-5])
        self.assertEqual(record['robots']['right']['metadata']['sleep_thresholds'], [5.e-5])
        self.assertEqual(record['task_constraints']['attachments']['fabrica_beam_6']['mode'],
                         'pure_physical_grasp')

    def test_borrowed_buffers_detach_before_next_getter_reuses_memory(self):
        raw = self.task._resolved_objects['fabrica_beam_6'].unwrap()
        raw._rigid_prim_view = BodyView(mutate=True)
        self.task.robots['right'].articulation._articulation_view = JointView(mutate=True)
        record = self.sample()
        body = record['bodies'][0]
        self.assertTrue(record['valid'])
        self.assertEqual(body['positions'], [[1., 2., 3.]])
        self.assertEqual(body['orientations'], [[1., 0., 0., 0.]])
        self.assertEqual(body['linear_velocities'], [[.1, .2, .3]])
        self.assertEqual(record['robots']['right']['joint_positions'][0], list(range(12)))

    def test_cached_pair_view_is_used_for_pose_velocity_and_metadata(self):
        pair = BodyView()
        pair.positions[0, 0] = 42
        self.task._contact_probes[('/World/beam_6', '/World/table')] = pair
        record = self.sample()
        body = record['bodies'][0]
        self.assertEqual(body['source'], 'cached_pair_probe')
        self.assertEqual(body['view_id'], id(pair))
        self.assertEqual(body['positions'][0][0], 42)
        self.assertEqual(pair.metadata_calls, 1)
        self.assertEqual(self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.pose_calls, 0)

    def test_missing_or_invalid_views_are_explicit_and_never_initialized(self):
        view = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view
        view.valid = False
        self.task.robots['right']._rigid_body_map.clear()
        self.task._resolve_object = lambda *_: self.fail('Must not resolve or construct a body')
        self.task._get_contact_probe = lambda *_: self.fail('Must not construct a contact view')
        self.task._robot_rigid_body_by_suffix = lambda *_: self.fail('Must not construct a link')
        record = self.sample()
        self.assertFalse(record['valid'])
        self.assertFalse(record['bodies'][0]['valid'])
        self.assertIn('valid live physics handle', record['bodies'][0]['error'])
        self.assertEqual(view.pose_calls, 0)
        self.assertTrue(all(not body['valid'] for body in record['bodies']))

    def test_sampling_does_not_invoke_task_observation_completion_or_setters(self):
        for name in ('get_observations', 'is_done', '_set_object_pose'):
            setattr(self.task, name, lambda *_: self.fail('Must not advance or write task state'))
        before_step = self.task.step_counter
        before_position = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions.copy()
        self.sample()
        self.assertEqual(self.task.step_counter, before_step)
        np.testing.assert_array_equal(before_position,
            self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions)

    def test_all_four_boundaries_keep_one_cycle_despite_task_counter_increment(self):
        os.environ['BEAM_PHYSICS_DIAGNOSTICS_START_STEP'] = '9000'
        os.environ['BEAM_PHYSICS_DIAGNOSTICS_END_STEP'] = '9000'
        for boundary in ('before_action', 'after_action', 'after_physics', 'after_observation'):
            if boundary == 'after_observation':
                self.task.step_counter += 1
            record = self.sample(boundary, cycle_step=9000)
            self.assertEqual(record['cycle_step'], 9000)
        records = [json.loads(line) for line in self.path.read_text().splitlines()]
        self.assertEqual(len(records), 4)
        self.assertEqual(records[-1]['task_step'], 9001)
        self.assertEqual([record['sequence'] for record in records], [0, 1, 2, 3])

    def test_failure_transition_does_not_drop_last_observation_boundary(self):
        self.sample(cycle_step=9000)
        self.task.phase = 'failed'
        self.task.step_counter = 9001
        record = self.sample('after_observation', cycle_step=9000)
        self.assertIsNotNone(record)
        self.assertEqual(record['phase'], 'failed')
        self.assertEqual(record['cycle_phase'], 'base_6_set_down')
        self.assertIsNone(self.sample('before_action', cycle_step=9001))

    def test_phase_step_window_and_stride_skip_outside_samples(self):
        os.environ.update({'BEAM_PHYSICS_DIAGNOSTICS_START_STEP': '9000',
                           'BEAM_PHYSICS_DIAGNOSTICS_END_STEP': '9004',
                           'BEAM_PHYSICS_DIAGNOSTICS_STRIDE': '2'})
        for step, expected in ((8999, False), (9000, True), (9001, False), (9002, True), (9005, False)):
            self.task.step_counter = step
            self.assertEqual(self.sample() is not None, expected)
        self.task.phase = 'insert_2'
        self.task.step_counter = 9002
        self.assertIsNone(self.sample())

    def _set_handoff_phase(self):
        name='assemble_00_part_3_assembly_clearance'
        self.task.phase=name
        self.task._beam_assisted_enabled=True
        self.task.phase_index=0
        self.task.phase_entry_step=9000
        self.task.phase_specs=[dict(name=name,local_skill=dict(beam_assisted_phase_handoff_ticks=24))]
        path='/World/right/wrist_3_link'
        self.task.robots['right']._rigid_body_map[path]=wrapper(BodyView(),path)
        os.environ.update(BEAM_PHYSICS_DIAGNOSTICS_PHASES='*',BEAM_PHYSICS_DIAGNOSTICS_STRIDE='8')

    def test_off_stride_handoff_captures_all_four_boundaries_through_failure(self):
        self._set_handoff_phase();self.task.step_counter=9001
        before=self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions.copy()
        records=[self.sample('before_action',cycle_step=9001)]
        self.task.phase='failed';self.task.phase_index=1
        for boundary in ('after_action','after_physics','after_observation'):
            records.append(self.sample(boundary,cycle_step=9001))
        self.assertTrue(all(r is not None for r in records))
        self.assertTrue(all(r['cycle_step']==9001 for r in records))
        self.assertEqual(records[-1]['cycle_phase'],'assemble_00_part_3_assembly_clearance')
        self.assertEqual(records[-1]['phase'],'failed')
        np.testing.assert_array_equal(before,self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions)

    def test_handoff_burst_keeps_native_wrist_and_all_boundaries_through_twenty_four_ticks(self):
        self._set_handoff_phase()
        for step,expected in ((8999,False),(9001,True),(9007,True),(9008,True),
                              (9009,True),(9010,True),(9016,True),(9017,True),
                              (9024,True),(9025,False),(9032,True)):
            self.task.step_counter=step
            record=self.sample('before_action',cycle_step=step)
            self.assertEqual(record is not None,expected)
            if record is not None:
                wrist=next(b for b in record['bodies'] if b.get('path')=='/World/right/wrist_3_link')
                self.assertTrue(wrist['valid'])
                self.assertEqual(wrist['angular_velocities'],[[.4,.5,.6]])

    def test_handoff_burst_does_not_bypass_configured_phase_or_step_window(self):
        from toolkits.factory_dual_franka_assembly.beam_physics_diagnostics import BeamPhysicsDiagnostics
        self._set_handoff_phase()
        mon=BeamPhysicsDiagnostics()
        for start,end,phases in ((9002,-1,{'*'}),(0,9000,{'*'}),(0,-1,{'other_phase'})):
            mon.start,mon.end,mon.phases=start,end,phases
            self.assertFalse(mon.selected(self.task,9001,self.task.phase,handoff_cycle=True))

    def test_settled_transition_keeps_four_native_boundaries_during_the_complete_hold(self):
        self._set_handoff_phase()
        self.task.phase_specs[0]['local_skill']['beam_assisted_transition_settle'] = {'stable_native_ticks':24}
        for step in (9001, 9047, 9101, 9479):
            self.task.step_counter = step
            for boundary in ('before_action','after_action','after_physics','after_observation'):
                record = self.sample(boundary, cycle_step=step)
                self.assertIsNotNone(record)
                self.assertTrue(next(b for b in record['bodies'] if b.get('path')=='/World/right/wrist_3_link')['valid'])

    def test_ordinary_nonhandoff_sampling_keeps_original_stride(self):
        self._set_handoff_phase();self.task.phase_specs[0]['local_skill']={}
        for step,expected in ((9001,False),(9007,False),(9008,True),(9009,False)):
            self.task.step_counter=step
            self.assertEqual(self.sample('before_action',cycle_step=step) is not None,expected)

    def test_first_strict_insert_records_96_entry_ticks_then_resumes_stride(self):
        self._set_handoff_phase()
        self.task.phase='assemble_00_part_3_insert_00'
        self.task.phase_specs=[dict(name=self.task.phase,local_skill=dict(beam_assisted_alignment_hold={'enabled':True}))]
        for step,expected in ((9001,True),(9095,True),(9096,True),(9097,False),(9104,True)):
            self.task.step_counter=step
            self.assertEqual(self.sample('before_action',cycle_step=step) is not None,expected)
        self.task.phase='assemble_00_part_3_insert_01'
        self.task.phase_specs[0]['name']=self.task.phase
        self.task.step_counter=9001
        self.assertIsNone(self.sample('before_action',cycle_step=9001))

    def test_metadata_getters_only_run_once_per_live_view(self):
        first = self.sample()
        second = self.sample('after_action')
        self.assertIn('metadata', first['bodies'][0])
        self.assertNotIn('metadata', second['bodies'][0])
        self.assertNotIn('global_stabilization', second)
        self.assertNotIn('global_physics_settings', second)
        self.assertNotIn('metadata', second['robots']['right'])
        self.assertEqual(self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.metadata_calls, 1)

    def test_observation_boundary_reflects_current_locks_and_attachment_modes(self):
        before = self.sample(cycle_step=9000)
        self.task._locked_targets['fabrica_beam_6'] = {'position': np.array([1., 2., 3.])}
        self.task._lock_pin_pose['fabrica_beam_6'] = {'orientation': np.array([1., 0., 0., 0.])}
        self.task._attachment_joints['fabrica_beam_6'] = '/World/joint'
        self.task._attachments['fabrica_beam_6']['mode'] = 'fixed_joint'
        after = self.sample('after_observation', cycle_step=9000)
        self.assertEqual(before['task_constraints']['locked_targets'], {})
        self.assertEqual(before['task_constraints']['attachments']['fabrica_beam_6']['mode'],
                         'pure_physical_grasp')
        self.assertEqual(after['task_constraints']['locked_targets']['fabrica_beam_6']['position'],
                         [1., 2., 3.])
        self.assertEqual(after['task_constraints']['lock_pin_pose']['fabrica_beam_6']['orientation'],
                         [1., 0., 0., 0.])
        self.assertEqual(after['task_constraints']['attachment_joint_paths']['fabrica_beam_6'], '/World/joint')
        self.assertEqual(after['task_constraints']['attachments']['fabrica_beam_6']['mode'], 'fixed_joint')

    def test_nonfinite_or_wrong_shape_physics_cannot_be_valid_evidence(self):
        view = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view
        view.velocities[0, 2] = float('nan')
        record = self.sample()
        self.assertFalse(record['valid'])
        self.assertIn('Non-finite', record['bodies'][0]['error'])
        json.loads(self.path.read_text())  # Strict JSON cannot contain a bare NaN.
        view.velocities = np.zeros((1, 3))
        record = self.sample('after_action')
        self.assertFalse(record['bodies'][0]['valid'])
        self.assertIn('shape', record['bodies'][0]['error'])

    def test_invalid_clock_is_recorded_and_not_invented(self):
        self.clock.get_physics_dt = lambda: (_ for _ in ()).throw(RuntimeError('clock unavailable'))
        record = self.sample()
        self.assertFalse(record['valid'])
        self.assertFalse(record['clock_valid'])
        self.assertIn('clock unavailable', record['clock_error'])
        self.assertNotIn('physics_step_index', record)

    def test_unavailable_metadata_and_stabilization_are_explicit(self):
        self.clock.get_physics_context = lambda: NS()
        view = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view
        view.get_coms = lambda **kw: (_ for _ in ()).throw(RuntimeError('COM unavailable'))
        record = self.sample()
        self.assertTrue(record['bodies'][0]['valid'])
        self.assertFalse(record['bodies'][0]['metadata']['valid'])
        self.assertFalse(record['bodies'][0]['center_of_mass']['valid'])
        self.assertIn('COM unavailable', record['bodies'][0]['metadata']['errors']['center_of_mass'])
        self.assertFalse(record['global_stabilization']['valid'])

    def test_scene_attribute_can_read_global_stabilization_without_setting_it(self):
        prim = NS(GetAttribute=lambda name: NS(Get=lambda: False))
        self.clock.get_physics_context = lambda: NS(_physics_scene=NS(GetPrim=lambda: prim))
        record = self.sample()
        self.assertFalse(record['global_stabilization']['enabled'])
        self.assertIn('existing_scene_attribute', record['global_stabilization']['source'])

    def test_state_write_audit_preserves_arguments_without_writing_physics(self):
        before = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions.copy()
        arguments = {'position': np.array([4., 5., 6.]), 'velocity': [float('nan')]}
        record = record_state_write(self.task, 'before_set_pose', 'fabrica_beam_6',
                                   '/World/beam_6', arguments)
        self.assertEqual(record['kind'], 'state_write_call_site')
        self.assertEqual(record['arguments']['position'], [4., 5., 6.])
        self.assertEqual(record['arguments']['velocity'], [{'invalid_numeric': 'nan'}])
        self.assertTrue(record['callers'])
        np.testing.assert_array_equal(before,
            self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view.positions)

    def test_low_level_paths_match_only_registered_beam_bodies_and_robots(self):
        self.task.robots['right'].config.prim_path = '/World/right'
        self.sample()
        record = record_path_state_write('/World/right/wrist_3_link', 'before_set_joint_positions', [1., 2.])
        self.assertEqual(record['body_path'], '/World/right/wrist_3_link')
        self.assertEqual(record['arguments'], [1., 2.])
        self.assertIsNone(record_path_state_write('/World/right_other/link', 'ignored'))
        os.environ['BEAM_PHYSICS_DIAGNOSTICS'] = '0'
        self.assertIsNone(record_path_state_write('/World/beam_6', 'ignored'))

    def test_other_tasks_and_disabled_capture_have_no_side_effects(self):
        self.task._resolved_objects = {'fabrica_car_6': wrapper(BodyView())}
        self.assertIsNone(self.sample())
        self.assertFalse(self.path.exists())
        self.assertFalse(hasattr(self.task, '_beam_physics_diagnostics'))
        self.task = task()
        os.environ['BEAM_PHYSICS_DIAGNOSTICS'] = '0'
        self.assertIsNone(self.sample())
        self.assertIsNone(record_state_write(self.task, 'test'))
        self.assertFalse(self.path.exists())

    def test_disabling_environment_stops_already_initialized_capture(self):
        self.sample()
        size = self.path.stat().st_size
        os.environ['BEAM_PHYSICS_DIAGNOSTICS'] = '0'
        self.assertIsNone(self.sample('after_action'))
        self.assertIsNone(record_state_write(self.task, 'before_set_pose'))
        self.assertEqual(self.path.stat().st_size, size)

    def test_output_failure_and_invalid_configuration_remain_visible(self):
        os.environ['BEAM_PHYSICS_DIAGNOSTICS_PATH'] = str(Path(self.tmp.name) / 'missing' / 'trace.jsonl')
        with self.assertRaises(FileNotFoundError):
            self.sample()
        self.task = task()
        os.environ['BEAM_PHYSICS_DIAGNOSTICS_STRIDE'] = '0'
        with self.assertRaises(ValueError):
            self.sample()

    def test_duplicate_cached_link_is_not_silently_selected(self):
        robot = self.task.robots['right']
        path = '/World/duplicate/Robotiq_2F_85/base_link'
        robot._rigid_body_map[path] = wrapper(BodyView(), path)
        record = self.sample()
        self.assertFalse(record['valid'])
        self.assertIn('found 2', record['bodies'][1]['error'])

    def test_existing_end_effector_can_supply_base_without_creating_idle_fingers(self):
        robot = self.task.robots['right']
        path = '/World/right/Robotiq_2F_85/base_link'
        robot.articulation.end_effector = robot._rigid_body_map.pop(path)
        robot._rigid_body_map.clear()
        record = self.sample()
        self.assertTrue(record['bodies'][1]['valid'])
        self.assertEqual(record['bodies'][1]['path'], path)
        self.assertTrue(all(not body['valid'] for body in record['bodies'][2:]))
        self.assertEqual(robot._rigid_body_map, {})

    def test_end_effector_fallback_cannot_label_wrist_or_invalid_view_as_gripper(self):
        robot = self.task.robots['right']
        robot._rigid_body_map.clear()
        robot.articulation.end_effector = wrapper(BodyView(), '/World/right/wrist_3_link')
        record = self.sample()
        self.assertFalse(record['bodies'][1]['valid'])
        robot.articulation.end_effector = wrapper(BodyView(valid=False),
            '/World/right/Robotiq_2F_85/base_link')
        record = self.sample('after_action')
        self.assertFalse(record['bodies'][1]['valid'])

    def test_sleep_threshold_readbacks_copy_without_clone_and_never_enable_gate(self):
        sleep = np.array([1.e-5])
        body_view = self.task._resolved_objects['fabrica_beam_6'].unwrap()._rigid_prim_view
        body_view.get_sleep_thresholds = lambda: sleep
        joint_view = self.task.robots['right'].articulation._articulation_view
        joint_view.get_sleep_thresholds = lambda: sleep
        record = self.sample()
        sleep[:] = 12
        self.assertEqual(record['bodies'][0]['metadata']['sleep_thresholds'], [1.e-5])
        self.assertEqual(record['robots']['right']['metadata']['sleep_thresholds'], [1.e-5])
        self.assertNotIn('success', record)
        self.assertNotIn('table_support_ready', record)

    def test_unavailable_solver_gpu_and_sleep_metadata_are_explicit(self):
        self.clock.get_physics_context = lambda: NS()
        self.task.robots['right'].articulation._articulation_view.get_sleep_thresholds = None
        record = self.sample()
        self.assertFalse(record['global_physics_settings']['solver_type']['valid'])
        self.assertFalse(record['global_physics_settings']['gpu_dynamics_enabled']['valid'])
        self.assertFalse(record['robots']['right']['metadata']['valid'])
        self.assertTrue(record['robots']['right']['valid'])

    def test_native_targets_copy_backend_buffers_not_proposed_actions(self):
        values = np.arange(12.).reshape(1, 12)
        def velocity():
            values[:] = 9.
            return np.zeros((1, 12))
        native = NS(get_dof_position_targets=lambda: values,
                    get_dof_velocity_targets=velocity,
                    get_dof_actuation_forces=lambda: np.ones((1, 12)))
        joint_view = self.task.robots['right'].articulation._articulation_view
        joint_view._physics_view = native
        self.task.robots['right'].last_action = [{'joint_positions': [-123.]}]
        record = self.sample('after_action')
        target = record['robots']['right']['native_command_targets']
        self.assertTrue(target['valid'])
        self.assertEqual(target['position_targets'], [list(np.arange(12.))])
        self.assertEqual(record['robots']['right']['sdk_submission']['actions'],
                         [{'joint_positions': [-123.]}])

    def test_missing_nonfinite_or_wrong_shape_native_targets_are_unavailable(self):
        for value in (np.zeros((12,)), np.full((1, 12), np.nan)):
            view = NS(_physics_view=NS(get_dof_position_targets=lambda: value,
                get_dof_velocity_targets=lambda: np.zeros((1, 12)),
                get_dof_actuation_forces=lambda: np.zeros((1, 12))))
            result = _joint_command_targets(view, (1, 12))
            self.assertFalse(result['valid'])
            self.assertNotIn('position_targets', result)
        result = _joint_command_targets(NS(), (1, 12))
        self.assertFalse(result['valid'])
        self.assertEqual(len(result['errors']), 3)

    def test_external_forces_reads_existing_schema_once_without_writing(self):
        reads = []
        def forbidden(*args, **kwargs):
            self.fail('Scene diagnostic must not apply a schema or write an attribute')
        attribute = NS(Get=lambda: True, Set=forbidden)
        api = NS(GetEnableExternalForcesEveryIterationAttr=lambda: reads.append('get') or attribute,
                 CreateEnableExternalForcesEveryIterationAttr=forbidden, Apply=forbidden)
        self.clock.get_physics_context()._physx_scene_api = api
        first = self.sample()
        readback = first['global_physics_settings']['external_forces_every_iteration']
        self.assertTrue(readback['valid'])
        self.assertIs(readback['value'], True)
        self.assertEqual(readback['attribute'], 'physxScene:enableExternalForcesEveryIteration')
        self.assertEqual(readback['scope'], 'existing_scene_attribute')
        self.assertIn('existing_physx_scene_api', readback['source'])
        second = self.sample('after_action')
        self.assertNotIn('global_physics_settings', second)
        self.assertEqual(reads, ['get'])

    def test_external_forces_scene_attribute_fallback_preserves_false(self):
        names = []
        prim = NS(GetAttribute=lambda name: names.append(name) or NS(Get=lambda: False))
        self.clock.get_physics_context()._physics_scene = NS(GetPrim=lambda: prim)
        readback = self.sample()['global_physics_settings']['external_forces_every_iteration']
        self.assertTrue(readback['valid'])
        self.assertIs(readback['value'], False)
        self.assertIn('physxScene:enableExternalForcesEveryIteration', names)
        self.assertIn('existing_scene_attribute:', readback['source'])

    def test_external_forces_missing_invalid_or_throwing_values_are_explicit(self):
        for value in (None, 'false', 0, np.array([True])):
            with self.subTest(value=str(value)):
                self.task = task()
                self.clock.get_physics_context()._physx_scene_api = NS(
                    GetEnableExternalForcesEveryIterationAttr=lambda: NS(Get=lambda: value))
                readback = self.sample()['global_physics_settings']['external_forces_every_iteration']
                self.assertFalse(readback['valid'])
                self.assertIn('boolean', readback['error'])
        self.task = task()
        self.clock.get_physics_context()._physx_scene_api = NS(
            GetEnableExternalForcesEveryIterationAttr=lambda: (_ for _ in ()).throw(RuntimeError('read unavailable')))
        readback = self.sample()['global_physics_settings']['external_forces_every_iteration']
        self.assertFalse(readback['valid'])
        self.assertIn('read unavailable', readback['error'])
        self.task = task()
        self.clock.get_physics_context = lambda: NS()
        readback = self.sample()['global_physics_settings']['external_forces_every_iteration']
        self.assertFalse(readback['valid'])
        self.assertIn('No existing physics scene', readback['error'])

    def test_contact_loads_only_read_cached_pairs_and_reverse_sensor_direction(self):
        calls = []
        probe = BodyView()
        probe.get_contact_force_matrix = lambda **kw: calls.append(('normal', kw['dt'])) or np.array([[[0., 0., -2.]]])
        probe.get_friction_data = lambda **kw: calls.append(('friction', kw['dt'])) or (
            np.array([[0., 0., -.45]]), np.zeros((1, 3)), np.array([[1]]), np.array([[0]]))
        self.task._contact_probes[('/World/finger', '/World/beam_6')] = probe
        self.task._get_contact_probe = lambda *_: self.fail('Must not create a probe')
        self.task.get_observations = lambda: self.fail('Must not sample the task')
        record = self.sample()
        ledger = record['contact_loads']
        self.assertTrue(ledger['valid'])
        self.assertEqual(ledger['pairs'][0]['normal_force_world'], [0., 0., -2.])
        self.assertEqual(ledger['pairs'][0]['total_force_on_selected_world'], [0., 0., 2.45])
        self.assertEqual(calls, [('normal', 1/240), ('friction', 1/240)])


class IndependentReadbacks(unittest.TestCase):
    def test_com_world_position_rotates_local_offset_with_actor_and_combines_frames(self):
        q = np.array([[2 ** -.5, 0., 0., 2 ** -.5]])
        view = NS(get_coms=lambda **kw: (np.array([[[1., 0., 0.]]]),
                                         np.array([[[1., 0., 0., 0.]]])))
        result = _body_com_snapshot(view, np.array([[2., 3., 4.]]), q)
        self.assertTrue(result['valid'])
        np.testing.assert_allclose(result['world_positions'], [[2., 4., 4.]], atol=1e-12)
        np.testing.assert_allclose(result['world_orientations'], q, atol=1e-12)

    def test_bad_com_is_unavailable_without_substituting_actor_origin(self):
        for position in (np.zeros((3,)), np.full((1, 1, 3), np.nan)):
            view = NS(get_coms=lambda **kw: (position, np.array([[[1., 0., 0., 0.]]])))
            result = _body_com_snapshot(view, np.zeros((1, 3)), np.array([[1., 0., 0., 0.]]))
            self.assertFalse(result['valid'])
            self.assertNotIn('world_positions', result)

    def test_native_gravity_uses_existing_view_and_detaches_its_buffer(self):
        gravity = np.array([0., 0., -9.81])
        existing = NS(get_gravity=lambda: gravity)
        manager = NS(get_physics_sim_view=lambda: existing)
        with patch.dict('sys.modules', {'isaacsim.core.simulation_manager': NS(SimulationManager=manager)}):
            result = _native_scene_gravity()
        gravity[:] = 0.
        self.assertTrue(result['valid'])
        self.assertEqual(result['value'], [0., 0., -9.81])
        self.assertEqual(result['frame'], 'world')

    def test_missing_nonfinite_or_wrong_shape_native_gravity_never_uses_context_fallback(self):
        for existing in (None, NS(get_gravity=lambda: np.array([0., 0., -np.inf])),
                         NS(get_gravity=lambda: np.zeros((1, 3)))):
            manager = NS(get_physics_sim_view=lambda: existing)
            with patch.dict('sys.modules', {'isaacsim.core.simulation_manager': NS(SimulationManager=manager)}):
                result = _native_scene_gravity()
            self.assertFalse(result['valid'])
            self.assertNotIn('value', result)

    def test_friction_at_zero_normal_and_unused_padding_remain_visible(self):
        probe = NS(is_physics_handle_valid=lambda: True,
            get_contact_force_matrix=lambda **kw: np.zeros((1, 1, 3)),
            get_friction_data=lambda **kw: (np.array([[np.nan, np.nan, np.nan], [0., 0., 2.45]]),
                                           np.array([[np.nan, np.nan, np.nan], [0., 0., 0.]]),
                                           np.array([[1]]), np.array([[1]])))
        result = _pair_load_snapshot(probe, '/beam', '/table', '/beam', 1/240)
        self.assertTrue(result['valid'])
        self.assertEqual(result['normal_force_world'], [0., 0., 0.])
        self.assertEqual(result['total_force_on_selected_world'], [0., 0., 2.45])
        self.assertEqual(result['friction_anchor_count'], 1)

    def test_bad_friction_ranges_or_active_data_do_not_erase_valid_normal_evidence(self):
        for count, force in ((2, [0., 0., 0.]), (1, [0., 0., np.nan])):
            probe = NS(is_physics_handle_valid=lambda: True,
                get_contact_force_matrix=lambda **kw: np.array([[[0., 0., 2.]]]),
                get_friction_data=lambda **kw: (np.array([force]), np.zeros((1, 3)),
                                               np.array([[count]]), np.array([[0]])))
            result = _pair_load_snapshot(probe, '/beam', '/table', '/beam', 1/240)
            self.assertFalse(result['valid'])
            self.assertTrue(result['normal_valid'])
            self.assertEqual(result['normal_force_world'], [0., 0., 2.])
            self.assertNotIn('total_force_on_selected_world', result)

    def test_zero_count_friction_ranges_ignore_unused_start_sentinels(self):
        probe = NS(is_physics_handle_valid=lambda: True,
            get_contact_force_matrix=lambda **kw: np.zeros((1, 1, 3)),
            get_friction_data=lambda **kw: (np.full((2, 3), np.nan), np.full((2, 3), np.nan),
                                           np.array([[0]], dtype=np.uint32),
                                           np.array([[2 ** 32 - 1]], dtype=np.uint32)))
        result = _pair_load_snapshot(probe, '/beam', '/table', '/beam', 1/240)
        self.assertTrue(result['valid'])
        self.assertEqual(result['friction_force_world'], [0., 0., 0.])
        self.assertEqual(result['friction_anchor_count'], 0)


if __name__ == '__main__':
    unittest.main()
