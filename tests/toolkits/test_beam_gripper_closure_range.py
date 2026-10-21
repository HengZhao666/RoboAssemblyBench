"""Verify both command clamps admit Beam-left closure without relaxing contact."""
from __future__ import annotations
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
import yaml

from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'internutopia_extension/controllers/gripper_controller.py'
node=next(n for n in ast.walk(ast.parse(SOURCE.read_text())) if isinstance(n,ast.FunctionDef)
          and n.name=='_continuous_forward')
scope={'np':np,'ArticulationAction':SimpleNamespace}
exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),str(SOURCE),'exec'),scope)
Controller=type('Controller',(),{'_continuous_forward':scope['_continuous_forward']})


class ClosureRange(unittest.TestCase):
    def configs(self):
        recipe=yaml.safe_load((ROOT/'roboassemblybench/tasks/fabrica_beam_ur5e_staged/recipe.yaml').read_text())
        specs={r['name']:r for r in recipe['robots']}
        return {name:SimpleNamespace(gripper_close_openness=spec.get('gripper_close_openness',.08))
                for name,spec in specs.items()}

    def controller(self,config):
        c=Controller();c._robot_config=config;c._command_index=0
        c._gripper=SimpleNamespace(joint_opened_positions=[0.],joint_closed_positions=[.8],active_joint_indices=[6])
        return c

    def test_left_both_clamps_can_seek_beyond_previous_joint_limit(self):
        config=self.configs()['franka_left']
        self.assertEqual(config.gripper_close_openness,.04)
        state={'contact_control_active':True,'contact_capture_closure':.74,'contact_command_closure':.745}
        metrics={side:{'force_probe_valid':True,'force':0.,'force_observation':{'physics_dt':1/240}}
                 for side in ('left_finger','right_finger')}
        result=UR5eAssemblyAtomicSkillAdapter._contact_close_control(state=state,
            spec={'close_force_max_seek':.03,'close_force_joint_rate':.025},metrics=metrics,
            gripper_q=.745,open_q=0.,closed_q=.8,requested_openness=0.,min_openness=config.gripper_close_openness)
        self.assertGreater(result['command_q'],.736)
        self.assertLessEqual(result['command_q'],.768)
        c=self.controller(config);action=c._continuous_forward(result['command_openness'])
        self.assertAlmostEqual(action.joint_positions[0],result['command_q'])
        self.assertEqual(c.last_numeric_command['minimum_openness'],.04)

    def test_right_base_retains_both_previous_command_clamps(self):
        config=self.configs()['franka_right'];self.assertEqual(config.gripper_close_openness,.08)
        c=self.controller(config);action=c._continuous_forward(0.)
        self.assertAlmostEqual(action.joint_positions[0],.736)
        self.assertEqual(c.last_numeric_command['minimum_openness'],.08)

    def test_beyond_floor_overload_still_unloads_and_rejects_qualification(self):
        config=self.configs()['franka_left']
        state={'contact_control_active':True,'contact_capture_closure':.74,'contact_command_closure':.75}
        metrics={side:{'force_probe_valid':True,'force':80.,'force_observation':{'physics_dt':1/240}}
                 for side in ('left_finger','right_finger')}
        result=UR5eAssemblyAtomicSkillAdapter._contact_close_control(state=state,
            spec={'close_force_limit':40.},metrics=metrics,gripper_q=.75,open_q=0.,closed_q=.8,
            requested_openness=0.,min_openness=config.gripper_close_openness)
        self.assertFalse(result['force_within_limit'])
        self.assertEqual(result['mode'],'force_limit_unload')
        self.assertLess(result['command_q'],.75)


if __name__=='__main__':unittest.main()
