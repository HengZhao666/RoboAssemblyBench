"""An explicit scene experiment must fail visibly or preserve exact defaults."""
import ast
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from toolkits.factory_dual_franka_assembly.beam_external_force_control import (
    EXTERNAL_FORCE_ATTRIBUTE, BeamExternalForceOverrideError,
    apply_beam_external_force_override,
)


class Attribute:
    def __init__(self, value=False, *, valid=True, type_name='bool'):
        self.value, self.valid, self.type_name = value, valid, type_name
        self.calls = []
        self.ignore_set = False
        self.setter_result = True

    def IsValid(self):
        return self.valid

    def GetTypeName(self):
        return self.type_name

    def Get(self):
        self.calls.append(('get',))
        return self.value

    def Set(self, value):
        self.calls.append(('set', value))
        if not self.ignore_set:
            self.value = value
        return self.setter_result


class Prim:
    def __init__(self, path='/physicsScene', *, valid=True, type_name='PhysicsScene'):
        self.path, self.valid, self.type_name = path, valid, type_name
        self.stage = None
        self.attribute = Attribute()
        self.attribute_names = []

    def IsValid(self):
        return self.valid

    def GetPath(self):
        return self.path

    def GetTypeName(self):
        return self.type_name

    def GetStage(self):
        return self.stage

    def GetAttribute(self, name):
        self.attribute_names.append(name)
        return self.attribute

    def CreateAttribute(self, *args):
        raise AssertionError('Experiment must use the existing schema attribute')


class Stage:
    def __init__(self, prim):
        self.prims = [prim]
        prim.stage = self
        self.path_lookups = []

    def GetPrimAtPath(self, path):
        self.path_lookups.append(path)
        return next((p for p in self.prims if p.path == path), None)

    def Traverse(self):
        return iter(self.prims)


class Context:
    def __init__(self, prim, gpu=False):
        self.prim_path = prim.path
        self._physx_scene_api = NS(GetPrim=lambda: prim)
        self.gpu = gpu
        self.gpu_reads = 0
        self.solver = 'TGS'
        self.solver_reads = 0

    def get_solver_type(self):
        self.solver_reads += 1
        return self.solver

    def set_solver_type(self, *args):
        raise AssertionError('External-force experiment must not change the solver')

    def is_gpu_dynamics_enabled(self):
        self.gpu_reads += 1
        return self.gpu

    def enable_gpu_dynamics(self, *args):
        raise AssertionError('External-force experiment must not change GPU settings')


class BeamExternalForceControl(unittest.TestCase):
    def setUp(self):
        self.prim = Prim()
        self.stage = Stage(self.prim)
        self.context = Context(self.prim)
        self.recipes = ['fabrica_beam_ur5e_staged']

    def apply(self, requested='1', **kwargs):
        return apply_beam_external_force_override(
            self.context, self.stage, self.recipes, requested,
            isolated_scene=kwargs.pop('isolated_scene', True), **kwargs,
        )

    def test_default_is_noop_even_with_unreadable_context_stage_and_recipes(self):
        class Unreadable:
            def __getattr__(self, key):
                raise AssertionError('Default must not access scene/context')

            def __iter__(self):
                raise AssertionError('Default must not inspect recipes')
        audit = apply_beam_external_force_override(Unreadable(), Unreadable(), Unreadable(), None)
        self.assertFalse(audit['applied'])
        self.assertFalse(audit['changed'])
        self.assertEqual(audit['reason'], 'no_override_requested')
        self.assertFalse(apply_beam_external_force_override(None, None, None, None)['applied'])

    def test_only_explicit_zero_one_strings_are_accepted_before_access(self):
        for requested in ('true', 'false', '', ' 1', 1, 0, True, False, 'default'):
            with self.subTest(requested=requested), self.assertRaises(ValueError):
                self.apply(requested)
        self.assertEqual(self.stage.path_lookups, [])
        self.assertEqual(self.context.gpu_reads, 0)

    def test_isolation_and_all_beam_recipes_are_required_before_stage_access(self):
        cases = [([], True), (['fabrica_car'], True),
                 (['fabrica_beam', 'fabrica_car'], True), (['fabrica_beamish'], True),
                 (self.recipes, False), (self.recipes, 1), (None, True),
                 ('fabrica_beam', True), ([1], True)]
        for recipes, isolated in cases:
            with self.subTest(recipes=recipes, isolated=isolated), self.assertRaises(ValueError):
                apply_beam_external_force_override(self.context, self.stage, recipes, '1',
                                                   isolated_scene=isolated)
        self.assertEqual(self.stage.path_lookups, [])
        self.assertEqual(self.context.gpu_reads, 0)

    def test_false_and_true_gpu_states_are_supported_without_writing_them(self):
        for gpu in (False, True):
            with self.subTest(gpu=gpu):
                self.setUp()
                self.context.gpu = gpu
                audit = self.apply()
                self.assertTrue(audit['applied'])
                self.assertTrue(audit['changed'])
                self.assertEqual(audit['original'], False)
                self.assertEqual(audit['readback'], True)
                self.assertEqual(audit['gpu_dynamics_enabled'], gpu)
                self.assertEqual(audit['gpu_dynamics_readback_after'], gpu)
                self.assertEqual(audit['solver_type'], 'TGS')
                self.assertEqual(audit['solver_type_readback_after'], 'TGS')
                self.assertFalse(audit['native_solver_effect_verified'])
                self.assertEqual(audit['configuration_source'], 'existing_usd_scene_attribute')
                self.assertEqual(self.prim.attribute.calls, [('get',), ('set', True), ('get',)])
                self.assertEqual(self.context.gpu_reads, 2)
                self.assertEqual(self.prim.attribute_names, [EXTERNAL_FORCE_ATTRIBUTE])

    def test_already_requested_value_is_verified_without_a_setter(self):
        self.prim.attribute.value = True
        audit = self.apply()
        self.assertTrue(audit['applied'])
        self.assertFalse(audit['changed'])
        self.assertEqual(audit['reason'], 'already_requested_value')
        self.assertEqual(self.prim.attribute.calls, [('get',)])
        self.assertNotIn('setter_called', audit)

    def test_explicit_zero_and_multiple_beam_recipes_are_supported(self):
        self.prim.attribute.value = True
        self.recipes.append('fabrica_beam')
        audit = self.apply('0')
        self.assertTrue(audit['applied'])
        self.assertTrue(audit['changed'])
        self.assertEqual(audit['readback'], False)
        self.assertEqual(audit['scene_prim_path'], '/physicsScene')
        self.assertEqual(audit['scene_schema_source'], 'physics_context._physx_scene_api.GetPrim')

    def test_public_context_path_and_existing_api_must_identify_the_same_prim(self):
        for bad_path in (None, '', 'physicsScene', '/missingScene'):
            with self.subTest(path=bad_path):
                self.context.prim_path = bad_path
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
        self.context.prim_path = self.prim.path
        for schema in (None, NS(), NS(GetPrim=lambda: Prim('/otherScene'))):
            with self.subTest(schema=schema):
                self.context._physx_scene_api = schema
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
        self.assertEqual(self.prim.attribute.calls, [])

    def test_invalid_or_wrong_type_scene_is_not_modified(self):
        for valid, type_name in ((False, 'PhysicsScene'), (True, 'Xform')):
            with self.subTest(valid=valid, type_name=type_name):
                self.prim.valid, self.prim.type_name = valid, type_name
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
        self.assertEqual(self.prim.attribute.calls, [])

    def test_scene_must_belong_to_world_stage_and_be_its_only_physics_scene(self):
        other = Prim('/otherScene')
        other.stage = self.stage
        self.stage.prims.append(other)
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        self.assertEqual(raised.exception.audit['stage_physics_scene_paths'], ['/physicsScene', '/otherScene'])
        self.stage.prims.pop()
        self.prim.stage = Stage(Prim('/anotherWorld'))
        with self.assertRaises(BeamExternalForceOverrideError):
            self.apply()
        self.assertEqual(self.prim.attribute.calls, [])

    def test_missing_invalid_or_nonboolean_schema_attribute_fails_before_set(self):
        for attribute in (None, Attribute(valid=False), Attribute(type_name='int')):
            with self.subTest(attribute=attribute):
                self.prim.attribute = attribute
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
                if attribute:
                    self.assertEqual(attribute.calls, [])

    def test_missing_or_nonboolean_original_readback_is_not_overwritten(self):
        for value in (None, 0, 1, 'false'):
            with self.subTest(value=value):
                self.prim.attribute = Attribute(value)
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
                self.assertEqual(self.prim.attribute.calls, [('get',)])

    def test_missing_gpu_getter_or_nonboolean_gpu_readback_is_failure(self):
        for value in (None, 0, 1, 'false'):
            with self.subTest(value=value):
                self.context.gpu = value
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
        self.context.is_gpu_dynamics_enabled = None
        with self.assertRaises(BeamExternalForceOverrideError):
            self.apply()
        self.assertFalse(any(call[0] == 'set' for call in self.prim.attribute.calls))

    def test_pgs_unknown_or_missing_solver_readback_is_rejected_before_set(self):
        for value in ('PGS', None, 'tgs', ''):
            with self.subTest(value=value):
                self.context.solver = value
                with self.assertRaises(BeamExternalForceOverrideError):
                    self.apply()
        self.context.get_solver_type = None
        with self.assertRaises(BeamExternalForceOverrideError):
            self.apply()
        self.assertFalse(any(call[0] == 'set' for call in self.prim.attribute.calls))

    def test_solver_changing_after_set_cannot_report_success(self):
        values = iter(('TGS', 'PGS'))
        self.context.get_solver_type = lambda: next(values)
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        self.assertEqual(raised.exception.audit['solver_type'], 'TGS')
        self.assertEqual(raised.exception.audit['solver_type_readback_after'], 'PGS')
        self.assertFalse(raised.exception.audit['applied'])
        self.assertIn('requires a TGS readback', raised.exception.audit['error'])

    def test_ignored_setting_cannot_report_success(self):
        self.prim.attribute.ignore_set = True
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        audit = raised.exception.audit
        self.assertFalse(audit['applied'])
        self.assertFalse(audit['changed'])
        self.assertEqual(audit['readback'], False)
        self.assertIn('does not match', audit['error'])

    def test_false_or_missing_setter_confirmation_still_fails_with_actual_readback(self):
        for result in (False, None, 1):
            with self.subTest(result=result):
                self.prim.attribute = Attribute()
                self.prim.attribute.setter_result = result
                with self.assertRaises(BeamExternalForceOverrideError) as raised:
                    self.apply()
                self.assertFalse(raised.exception.audit['applied'])
                self.assertTrue(raised.exception.audit['changed'])
                self.assertTrue(raised.exception.audit['readback'])

    def test_setter_exception_is_a_visible_failure_with_unknown_final_state(self):
        self.prim.attribute.Set = lambda value: (_ for _ in ()).throw(RuntimeError('setter failed'))
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        self.assertFalse(raised.exception.audit['applied'])
        self.assertIsNone(raised.exception.audit['changed'])
        self.assertIn('setter failed', raised.exception.audit['error'])

    def test_readback_unavailable_after_set_is_failure_not_assumed_success(self):
        values = iter((False, None))
        self.prim.attribute.Get = lambda: next(values)
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        self.assertFalse(raised.exception.audit['applied'])
        self.assertIsNone(raised.exception.audit['changed'])

    def test_unexpected_gpu_change_after_setting_aborts_the_experiment(self):
        values = iter((False, True))
        self.context.is_gpu_dynamics_enabled = lambda: next(values)
        with self.assertRaises(BeamExternalForceOverrideError) as raised:
            self.apply()
        self.assertEqual(raised.exception.audit['gpu_dynamics_enabled'], False)
        self.assertEqual(raised.exception.audit['gpu_dynamics_readback_after'], True)
        self.assertFalse(raised.exception.audit['applied'])


class RunnerInitializationHook(unittest.TestCase):
    def create_world(self, *, requested=None, env_num=1, recipes=None):
        root = Path(__file__).resolve().parents[2]
        tree = ast.parse((root / 'internutopia/core/runner.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SimulatorRunner')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'create_world')
        scope = {'os': os, 'json': json, 'log': NS(info=lambda *args: None)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
                     'runner.create_world', 'exec'), scope)
        prim = Prim()
        stage = Stage(prim)
        context = Context(prim)
        events = []

        class World:
            def __init__(self, **kwargs):
                events.append('world_created')
                self.stage = stage

            def get_physics_context(self):
                events.append('get_context')
                return context

            def step(self, *args, **kwargs):
                raise AssertionError('Override must happen before any physics step')

        original_set = prim.attribute.Set

        def set_value(value):
            events.append('attribute_set')
            return original_set(value)

        prim.attribute.Set = set_value
        module = ModuleType('omni.isaac.core')
        module.World = World
        runner = NS(env_num=env_num, config=NS(
            simulator=NS(physics_dt='1 / 240', rendering_dt='1 / 30', use_fabric=True),
            task_configs=[NS(recipe=name) for name in (recipes or ['fabrica_beam_ur5e_staged'])]))
        env = {} if requested is None else {'BEAM_EXTERNAL_FORCES_EVERY_ITERATION': requested}
        with patch.dict(sys.modules, {'omni': ModuleType('omni'), 'omni.isaac': ModuleType('omni.isaac'),
                                     'omni.isaac.core': module}), patch.dict(os.environ, env, clear=True):
            scope['create_world'](runner)
        return runner, context, prim, events

    def test_default_runner_does_not_access_context_or_scene_attributes(self):
        runner, context, prim, events = self.create_world()
        self.assertEqual(events, ['world_created'])
        self.assertEqual(context.gpu_reads, 0)
        self.assertEqual(prim.attribute.calls, [])
        self.assertFalse(hasattr(runner, '_beam_external_force_override_audit'))

    def test_actual_runner_hook_runs_once_after_world_creation_and_before_step(self):
        runner, context, prim, events = self.create_world(requested='1')
        self.assertEqual(events, ['world_created', 'get_context', 'attribute_set'])
        self.assertEqual(prim.attribute.calls, [('get',), ('set', True), ('get',)])
        self.assertTrue(runner._beam_external_force_override_audit['applied'])
        self.assertEqual(runner.dt, 1 / 240)

    def test_runner_rejects_multienvironment_and_other_recipes(self):
        for env_num, recipes in ((2, ['fabrica_beam_ur5e_staged']), (1, ['fabrica_car'])):
            with self.subTest(env_num=env_num, recipes=recipes), self.assertRaises(ValueError):
                self.create_world(requested='1', env_num=env_num, recipes=recipes)


if __name__ == '__main__':
    unittest.main()
