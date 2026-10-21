"""Real compiler and final joint publisher checks for the right side route."""
import copy
import os
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np


class SideClearance(unittest.TestCase):
    def test_real_compiler_changes_only_two_stages_and_timed_prealign(self):
        from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe
        import roboassemblybench.core.beam_physical_compiler as compiler
        with patch.object(compiler, '_move_grasp_ik_metrics', return_value={'ik_feasible': True}):
            with patch.dict(os.environ, {'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE': '0'}):
                original = load_task_recipe('fabrica_beam_ur5e_staged', scene_profile='taoyuan_grscenes_tabletop')
            with patch.dict(os.environ, {'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE': '1'}):
                trial = load_task_recipe('fabrica_beam_ur5e_staged', scene_profile='taoyuan_grscenes_tabletop')
        self.assertEqual(trial['targets'], original['targets'])
        new_names = ['hold_3_high_joint_clearance', 'hold_3_high_wrist_reorientation']
        phases = trial['phases']
        i = next(i for i,p in enumerate(phases) if p['name'] == new_names[0])
        self.assertEqual([p['name'] for p in phases[i-1:i+3]],
                         ['hold_3_approach_clearance', *new_names, 'hold_3_move_above'])
        for p in phases[i:i+2]:
            self.assertEqual(p['local_skill']['name'], 'move_arm_to_joint_positions')
            self.assertEqual(p['local_skill']['max_command_joint_step'], .004)
            self.assertEqual(p['local_skills']['franka_left']['object'], 'fabrica_beam_3')
            self.assertTrue(p['local_skills']['franka_left']['requires_held_object'])
            self.assertEqual(p['gripper_commands']['franka_left'], 'contact_hold')
        restored = copy.deepcopy(trial)
        restored['phases'] = [p for p in restored['phases'] if p['name'] not in new_names]
        above = next(p for p in restored['phases'] if p['name']=='hold_3_move_above')
        old = next(p for p in original['phases'] if p['name']=='hold_3_move_above')
        above['local_skill']['prealign_steps'] = old['local_skill']['prealign_steps']
        restored['recipe_fingerprint'] = original['recipe_fingerprint']
        self.assertGreater(trial['max_steps'], original['max_steps'])
        restored['max_steps'] = original['max_steps']  # Derived phase budget.
        self.assertEqual(restored, original)

    def test_actual_joint_action_keeps_positive_turn_cap_and_measured_completion(self):
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
        adapter = Adapter({})
        q = np.array([1.85,-1.5707963268,1.5707963268,-1.5707963268,-1.5707963268,0.])
        end = q.copy(); end[4] = 1.50
        remembered = q.copy(); complete = []
        adapter._current_arm_q = lambda *a: q.copy()
        adapter._command_reference_q = lambda **kw: remembered.copy()
        adapter._last_command_q = lambda **kw: remembered.copy()
        def remember(task, robot, command):
            nonlocal remembered
            remembered = command.copy()
        adapter._remember_arm_command = remember
        adapter._gripper_command_value = lambda **kw: 1.
        adapter._mark_complete = lambda **kw: complete.append(kw['detail'])
        task = SimpleNamespace(phase_step_counter=0)
        spec = {'joint_positions':end.tolist(),'gripper_command':'open',
                'max_joint_step':.004,'max_command_joint_step':.004,
                'joint_position_tolerance':.005,'joint_target_stable_steps':8,
                'limit_command_to_measured_state':True,
                'max_command_tracking_error':.18,'max_wrist_command_tracking_error':.12}
        for i in range(1200):
            action = adapter._move_arm_to_joint_positions_action(phase_key='route',task=task,
                robot_name='franka_right',skill_name='move_arm_to_joint_positions',spec=spec)
            command = np.asarray(next(v[0] for v in action.values() if len(v[0])==6))
            self.assertLessEqual(float(np.max(abs(command-q))),.004+1e-12)
            self.assertGreaterEqual(command[4],q[4]-1e-12)
            if i==239:self.assertFalse(complete)
            q = command;task.phase_step_counter += 1
            if complete:break
        self.assertTrue(complete)
        self.assertGreater(i,240)
        self.assertLessEqual(complete[-1]['joint_error'],.005)
        self.assertGreaterEqual(complete[-1]['stable_steps'],8)

    def test_invalid_route_flag_fails_instead_of_silently_changing_path(self):
        from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe
        import roboassemblybench.core.beam_physical_compiler as compiler
        with patch.object(compiler, '_move_grasp_ik_metrics', return_value={'ik_feasible': True}), patch.dict(os.environ, {'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'maybe'}):
            with self.assertRaisesRegex(ValueError, 'SIDE_CLEARANCE_ROUTE'):
                load_task_recipe('fabrica_beam_ur5e_staged',scene_profile='taoyuan_grscenes_tabletop')


if __name__=='__main__':
    unittest.main()
