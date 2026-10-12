"""A global solver experiment must remain explicitly scoped and auditable."""
import unittest

from toolkits.factory_dual_franka_assembly.beam_solver_control import (
    BeamSolverOverrideError, apply_beam_solver_override,
)


class PhysicsContext:
    def __init__(self, original='TGS', ignore_setting=False, gpu_dynamics=False):
        self.value = original
        self.calls = []
        self.ignore_setting = ignore_setting
        self.gpu_dynamics = gpu_dynamics

    def get_solver_type(self):
        self.calls.append(('get',))
        return self.value

    def set_solver_type(self, requested):
        self.calls.append(('set', requested))
        if not self.ignore_setting:
            self.value = requested

    def enable_stablization(self, value):
        raise AssertionError('Solver comparison must not change stabilization')

    def is_gpu_dynamics_enabled(self):
        self.calls.append(('get_gpu_dynamics',))
        return self.gpu_dynamics

    def enable_gpu_dynamics(self, value):
        raise AssertionError('Solver comparison must not change GPU dynamics')


class BeamSolverControl(unittest.TestCase):
    def test_none_is_noop_without_getters_setters_or_recipe_iteration(self):
        class UnreadableRecipes:
            def __iter__(self):
                raise AssertionError('No-op must not inspect task configuration')
        context = PhysicsContext()
        audit = apply_beam_solver_override(context, UnreadableRecipes(), None)
        self.assertEqual(context.calls, [])
        self.assertFalse(audit['applied'])
        self.assertFalse(audit['changed'])
        self.assertEqual(audit['reason'], 'no_override_requested')
        self.assertFalse(apply_beam_solver_override(None, None, None)['applied'])

    def test_explicit_solver_change_has_original_and_exact_readback(self):
        context = PhysicsContext()
        audit = apply_beam_solver_override(context, ['fabrica_beam_ur5e_staged'], 'PGS', isolated_scene=True)
        self.assertTrue(audit['applied'])
        self.assertTrue(audit['changed'])
        self.assertEqual(audit['original'], 'TGS')
        self.assertEqual(audit['readback'], 'PGS')
        self.assertEqual(context.calls, [('get',), ('get_gpu_dynamics',), ('set', 'PGS'), ('get',)])

    def test_both_recipes_and_isolation_are_required_before_touching_context(self):
        cases = [([], True), (['fabrica_car_ur5e_staged'], True),
                 (['fabrica_beam_ur5e_staged', 'fabrica_car'], True),
                 (['fabrica_beamish'], True), (['fabrica_beam'], False),
                 (['fabrica_beam'], 1), (None, True), ('fabrica_beam', True), ([12], True)]
        for recipes, isolated in cases:
            with self.subTest(recipes=recipes, isolated=isolated):
                context = PhysicsContext()
                with self.assertRaises(ValueError):
                    apply_beam_solver_override(context, recipes, 'PGS', isolated_scene=isolated)
                self.assertEqual(context.calls, [])

    def test_only_explicit_valid_solver_names_are_accepted(self):
        for requested in ('pgs', 'Tgs', '', 1, False, 'default', 'PGS '):
            with self.subTest(requested=requested):
                context = PhysicsContext()
                with self.assertRaises(ValueError):
                    apply_beam_solver_override(context, ['fabrica_beam'], requested, isolated_scene=True)
                self.assertEqual(context.calls, [])

    def test_readback_mismatch_is_failure_with_audit_not_assumed_success(self):
        context = PhysicsContext(ignore_setting=True)
        with self.assertRaises(BeamSolverOverrideError) as raised:
            apply_beam_solver_override(context, ['fabrica_beam'], 'PGS', isolated_scene=True)
        audit = raised.exception.audit
        self.assertFalse(audit['applied'])
        self.assertFalse(audit['changed'])
        self.assertEqual(audit['original'], 'TGS')
        self.assertEqual(audit['requested'], 'PGS')
        self.assertEqual(audit['readback'], 'TGS')
        self.assertIn('does not match', audit['error'])

    def test_unknown_initial_solver_is_not_overwritten(self):
        context = PhysicsContext(original=None)
        with self.assertRaises(BeamSolverOverrideError):
            apply_beam_solver_override(context, ['fabrica_beam'], 'PGS', isolated_scene=True)
        self.assertEqual(context.calls, [('get',)])

    def test_explicit_tgs_and_multiple_beam_recipes_are_supported_when_isolated(self):
        context = PhysicsContext(original='PGS')
        recipes = ['fabrica_beam', 'fabrica_beam_ur5e_staged']
        audit = apply_beam_solver_override(context, recipes, 'TGS', isolated_scene=True)
        self.assertEqual(audit['recipe_names'], recipes)
        self.assertEqual(audit['readback'], 'TGS')
        self.assertTrue(audit['applied'])

    def test_setter_exception_remains_visible_with_original_solver(self):
        context = PhysicsContext()
        context.set_solver_type = lambda requested: (_ for _ in ()).throw(RuntimeError('setter failure'))
        with self.assertRaises(BeamSolverOverrideError) as raised:
            apply_beam_solver_override(context, ['fabrica_beam'], 'PGS', isolated_scene=True)
        self.assertEqual(raised.exception.audit['original'], 'TGS')
        self.assertIn('setter failure', raised.exception.audit['error'])
        self.assertFalse(raised.exception.audit['applied'])

    def test_pgs_refuses_actual_gpu_dynamics_without_changing_gpu_or_solver(self):
        context = PhysicsContext(gpu_dynamics=True)
        with self.assertRaises(BeamSolverOverrideError) as raised:
            apply_beam_solver_override(context, ['fabrica_beam'], 'PGS', isolated_scene=True)
        self.assertEqual(context.calls, [('get',), ('get_gpu_dynamics',)])
        self.assertEqual(context.value, 'TGS')
        self.assertTrue(raised.exception.audit['gpu_dynamics_enabled'])
        self.assertFalse(raised.exception.audit['changed'])

    def test_same_solver_has_no_setter_and_reports_unchanged(self):
        for solver in ('TGS', 'PGS'):
            with self.subTest(solver=solver):
                context = PhysicsContext(original=solver)
                audit = apply_beam_solver_override(context, ['fabrica_beam'], solver, isolated_scene=True)
                self.assertTrue(audit['applied'])
                self.assertFalse(audit['changed'])
                self.assertEqual(audit['readback'], solver)
                self.assertFalse(any(call[0] == 'set' for call in context.calls))

    def test_missing_gpu_readback_cannot_authorize_pgs(self):
        context = PhysicsContext(gpu_dynamics=None)
        with self.assertRaises(BeamSolverOverrideError):
            apply_beam_solver_override(context, ['fabrica_beam'], 'PGS', isolated_scene=True)
        self.assertFalse(any(call[0] == 'set' for call in context.calls))


if __name__ == '__main__':
    unittest.main()
