"""Regression tests for the physical handover and table-load contract."""
import copy
import json
import sys
import unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from toolkits.factory_dual_franka_assembly.beam_support_state import BeamSupportState, table_support_ready, live_body_snapshot
from roboassemblybench.core.beam_coordinated import audited_grasp
from toolkits.factory_dual_franka_assembly.fabrica_online_planner_adapter import _quat_to_matrix_wxyz

class CoordinatedSupport(unittest.TestCase):
    def test_solver_precision_only_changes_beam_rigid_parts(self):
        from roboassemblybench.core.beam_coordinated import configure_beam_velocity_solver
        objects=[{'name': n, 'rigid_body': True, 'solver_velocity_iteration_count': 4,
                  'mass': .25, 'static_friction': .8} for n in
                 ['fabrica_beam_6','fabrica_beam_3','fabrica_car_6','fabrica_fixture','assembly_support']]
        original=copy.deepcopy(objects)
        configure_beam_velocity_solver(objects,16)
        for entry,before in zip(objects,original):
            expected={**before,'solver_velocity_iteration_count':16} if entry['name'].startswith('fabrica_beam_') else before
            self.assertEqual(entry,expected)
        for invalid in [True,3,33,16.5,'16']:
            with self.assertRaises(ValueError): configure_beam_velocity_solver(objects,invalid)

    def test_live_snapshot_rejects_authored_fallback_and_invalid_values(self):
        class View:
            valid = False
            velocity = np.zeros((1,6))
            def is_physics_handle_valid(self): return self.valid
            def get_world_poses(self, clone):
                return np.zeros((1,3)), np.array([[1.,0.,0.,0.]])
            def get_velocities(self, clone): return self.velocity
        v = View()
        with self.assertRaises(ValueError): live_body_snapshot(v)
        v.valid = True; v.velocity[0,0] = np.nan
        with self.assertRaises(ValueError): live_body_snapshot(v)
        v.velocity = np.zeros((2,6))
        with self.assertRaises(ValueError): live_body_snapshot(v)

    def test_live_snapshot_copies_pose_before_velocity_reuses_buffer(self):
        class View:
            position = np.array([[1.,2.,3.]])
            orientation = np.array([[1.,0.,0.,0.]])
            def is_physics_handle_valid(self): return True
            def get_world_poses(self, clone): return self.position, self.orientation
            def get_velocities(self, clone):
                self.position[:] = 100.; self.orientation[:] = 0.
                return np.array([[.001,0.,0.,0.,.01,0.]])
        sample = live_body_snapshot(View())
        self.assertEqual(sample['position'], [1.,2.,3.])
        self.assertEqual(sample['orientation'], [1.,0.,0.,0.])
        self.assertTrue(table_support_ready({**sample,'valid':True,
                           'vertical_force':2.4525,'bottom_gap':0.},1.962))

    def setUp(self):
        self.task=BeamSupportState();self.task._attachments={};self.task._beam_shared_grasps={}
        self.left={'robot_name':'left','position':[1,2,3],'attach_step':10,
                   'mode':'pure_physical_grasp','attach_spec':{'allow_shared_physical_grasp':True}}
        self.right={**copy.deepcopy(self.left),'robot_name':'right','position':[4,5,6],'attach_step':20}
    def attach_both(self):
        self.task._store_physical_grasp('post',self.left)
        self.task._store_physical_grasp('post',self.right)
    def test_preseat_target_keeps_table_height_and_randomization_group(self):
        from roboassemblybench.core.beam_coordinated import add_preseat_target
        targets=[{'name':'part_6_assembled','position':[.7,-.1,1.0025],'orientation':[1,0,0,0]}]
        groups=['part_6_assembled'];place={'local_skill':{}}
        original=copy.deepcopy(targets)
        add_preseat_target(place,targets,groups)
        self.assertEqual(targets[0],original[0])
        self.assertAlmostEqual(targets[1]['position'][2],1.0065)
        self.assertIn('part_6_preseat',groups)
        self.assertEqual(place['local_skill']['target_object_target'],'part_6_preseat')
        self.assertLess(place['local_skill']['relaxed_target_object_position_tolerance'],.004)
    def test_support_uses_actual_collider_and_excludes_distant_surfaces(self):
        from toolkits.factory_dual_franka_assembly.beam_support_state import combine_support_surfaces
        def surface(name,top,force,lo=-1,hi=1,valid=True):
            return {'object':name,'lower':[lo,-1,0],'upper':[hi,1,top],
                    'contact':{'valid':valid,'force_world':[0,0,force]}}
        surfaces=[surface('visual',1.0015,0),surface('physical',1.0025,2.0),
                  surface('board',1.0025,.45),
                  surface('distant',2,40,lo=3,hi=4)]
        result=combine_support_surfaces(surfaces,[[-.07,-.008,1.0025],[.07,.008,1.0152]])
        self.assertAlmostEqual(result['vertical_force'],2.45)
        self.assertAlmostEqual(result['bottom_gap'],0)
        surfaces[1]['contact']['valid']=False
        with self.assertRaises(ValueError):combine_support_surfaces(surfaces,[[-.07,-.008,1.0025],[.07,.008,1.0152]])
    def test_cube_bounds_apply_nonuniform_scale_once(self):
        from toolkits.factory_dual_franka_assembly.beam_support_state import cube_world_bounds
        transform=np.diag([.925,.625,.12,1.]);transform[3,:3]=[.47,-.14,.9425]
        lo,hi=cube_world_bounds(1.,transform)
        np.testing.assert_allclose(lo,[.0075,-.4525,.8825],atol=1e-12)
        np.testing.assert_allclose(hi,[.9325,.1725,1.0025],atol=1e-12)
    def test_new_grasp_preserves_original_anchor(self):
        self.attach_both()
        self.assertIs(self.task._attachment_for('post','left'),self.left)
        self.assertIs(self.task._attachment_for('post','right'),self.right)
        self.assertEqual(len(self.task._all_physical_grasps()),2)
    def test_releasing_primary_promotes_existing_anchor_without_reset(self):
        self.attach_both();self.task._remove_physical_grasp('post','left')
        self.assertIsNone(self.task._attachment_for('post','left'))
        self.assertIs(self.task._attachments['post'],self.right)
        self.assertEqual(self.right['attach_step'],20)
    def test_releasing_secondary_does_not_drop_primary(self):
        self.attach_both();self.task._remove_physical_grasp('post','right')
        self.assertIs(self.task._attachments['post'],self.left)
        self.assertEqual(self.task._beam_shared_grasps,{})
    def test_shared_grasp_requires_explicit_opt_in_on_both_hands(self):
        self.task._store_physical_grasp('post',self.left)
        self.right['attach_spec']={}
        with self.assertRaises(RuntimeError):self.task._store_physical_grasp('post',self.right)
        self.assertIs(self.task._attachments['post'],self.left)
    def test_table_gate_rejects_suspension_contact_only_and_bad_probes(self):
        valid={'valid':True,'motion_valid':True,'vertical_force':2.4,'bottom_gap':.0002,
               'linear_speed':.001,'angular_speed':.01}
        self.assertTrue(table_support_ready(valid,1.962))
        for changes in [{'vertical_force':0},{'vertical_force':.5},{'valid':False},
                        {'motion_valid':False},{'bottom_gap':.0025},{'bottom_gap':-.002},
                        {'vertical_force':50},{'linear_speed':.02},{'angular_speed':.2}]:
            self.assertFalse(table_support_ready({**valid,**changes},1.962),changes)
    def test_stability_counts_distinct_steps_and_resets_after_loss(self):
        self.task.phase_index=1;self.task.phase_entry_step=100
        sample={'step':100,'valid':True,'motion_valid':True,'vertical_force':2.4,
                'bottom_gap':0.,'linear_speed':0.,'angular_speed':0.}
        self.task._beam_table_observation=lambda:sample
        gate={'stable_steps':3}
        for _ in range(8):self.assertFalse(self.task._beam_table_supported(gate))
        sample['step']=101;self.assertFalse(self.task._beam_table_supported(gate))
        sample.update(step=102,vertical_force=0);self.assertFalse(self.task._beam_table_supported(gate))
        sample['vertical_force']=2.4
        for step in [103,104]:
            sample['step']=step;self.assertFalse(self.task._beam_table_supported(gate))
        sample['step']=105;self.assertTrue(self.task._beam_table_supported(gate))
    def test_real_phase_gate_accepts_valid_secondary_and_selective_release(self):
        import ast
        source=ast.parse((ROOT/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py').read_text())
        node=next(n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name=='_phase_interactions_complete')
        scope={};exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'<phase gate>','exec'),scope)
        self.attach_both();t=self.task;t.step_counter=100
        t._as_list=lambda x: [] if x is None else x if isinstance(x,list) else [x]
        t._normalized_attach_spec=lambda x:x;t._phase_payloads_held=lambda ph:True
        t._current_gripper_command=lambda ph,r:ph.get('gripper_commands',{}).get(r,'close')
        t._physical_hold_valid=lambda o,state:state['robot_name']=='right'
        t._extract_object_name=lambda e:e if isinstance(e,str) else e['object']
        gate=scope['_phase_interactions_complete']
        ph={'attach':[{'object':'post','robot':'right'}]}
        self.assertTrue(gate(t,ph))
        t._physical_hold_valid=lambda *a:False;self.assertFalse(gate(t,ph))
        ph={'detach':[{'object':'post','robot':'left'}], 'gripper_commands':{'left':'open','right':'close'}}
        self.assertFalse(gate(t,ph))
        t._remove_physical_grasp('post','left');self.assertTrue(gate(t,ph))
        self.assertIs(t._attachment_for('post','right'),self.right)

    def test_clearance_tcp_and_object_targets_rise_together_once(self):
        from types import SimpleNamespace
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
        task=SimpleNamespace(target_poses={'clearance':{'position':[.7,-.1,1.2]},
                             'assembled':{'position':[.7,-.1,1.0025]}},
             _ur5e_contact_hold_commands={'right':{'pose':{'position':[.5,-.2,1.36]}}})
        spec={'target_object_target':'clearance'}
        pose={'position':np.array([.6,-.1,1.30]),'orientation':np.array([1.,0,0,0])}
        result=Adapter._raise_clearance_target(task=task,robot_name='right',phase_key='one',spec=spec,target_pose=pose)
        self.assertAlmostEqual(result['position'][2],1.36)
        self.assertAlmostEqual(task.target_poses['clearance']['position'][2],1.26)
        self.assertEqual(task.target_poses['assembled']['position'],[.7,-.1,1.0025])
        self.assertAlmostEqual(pose['position'][2],1.30)
        task._ur5e_contact_hold_commands['right']['pose']['position'][2]=1.37
        Adapter._raise_clearance_target(task=task,robot_name='right',phase_key='one',spec=spec,target_pose=result)
        self.assertAlmostEqual(task.target_poses['clearance']['position'][2],1.26)
    def test_insertion_reuses_loaded_force_control_and_rejects_lost_grasp(self):
        from types import SimpleNamespace
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
        published=[]
        adapter=Adapter({})
        adapter._publish_loaded_action=lambda **kw: published.append(kw) or {'regulated':True}
        adapter._failure_or_hold=lambda *a,**kw:{'__local_skill_failure__':True}
        task=SimpleNamespace(_attachment_for=lambda *a:self.left,
             _physical_hold_valid=lambda *a,**kw:True,
             _ur5e_contact_hold_commands={'left':{'object':'post'}})
        spec={'requires_held_object':True,'object':'post'}
        action=adapter._regulated_insertion_action(task=task,robot_name='left',spec=spec,
                                                   command_q=np.zeros(6),fallback_action={'old':True})
        self.assertEqual(action,{'regulated':True});self.assertEqual(len(published),1)
        task._physical_hold_valid=lambda *a,**kw:False
        action=adapter._regulated_insertion_action(task=task,robot_name='left',spec=spec,
                                                   command_q=np.zeros(6),fallback_action={})
        self.assertTrue(action['__local_skill_failure__']);self.assertEqual(len(published),1)

    def test_seating_increment_cannot_mark_the_skill_complete(self):
        from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter
        # Missing ordinary motion arguments are deliberate: seating must return
        # before any TCP-completion or scheduler-side completion update occurs.
        UR5eAssemblyAtomicSkillAdapter._maybe_mark_complete(None, task=None, robot_name='right',
            skill_name='seat', spec={'beam_table_seat':True}, target_pose=None,
            ik_target_pose=None,current_pose=None,tracked_objects={},current_q=None,target_q=None)

    def test_success_cannot_bypass_final_unloaded_gate(self):
        import ast
        from types import SimpleNamespace
        source=ast.parse((ROOT/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py').read_text())
        node=next(n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name=='_check_success')
        scope={};exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),'<gate>','exec'),scope)
        task=SimpleNamespace(phase_specs=[{'beam_coordinated':True},
             {'name':'beam_final_unloaded_verification','advance':{}}],phase_index=0,
             cfg=SimpleNamespace(success_criteria=[]),_evaluate_advance_condition=lambda **kw:True)
        self.assertFalse(scope['_check_success'](task))
        task.phase_index=1;self.assertTrue(scope['_check_success'](task))
        task._evaluate_advance_condition=lambda **kw:False
        self.assertFalse(scope['_check_success'](task))

    def test_exact_grasp_survives_rotated_fixture_and_assembly_frames(self):
        poses=json.loads((ROOT/'roboassemblybench/core/beam_coordinated_poses.json').read_text())
        # This catches mixing assembly-origin coordinates with part COM coordinates.
        world=np.eye(4);world[:3,:3]=[[0,0,1],[1,0,0],[0,1,0]];world[:3,3]=[.4,-.3,1.2]
        for key,matrix in poses.items():
            g=audited_grasp({},key);inv=np.eye(4)
            inv[:3,:3]=_quat_to_matrix_wxyz(g['object_in_tcp_orientation'])
            inv[:3,3]=g['object_in_tcp_position']
            np.testing.assert_allclose(world@np.linalg.inv(inv),world@matrix,atol=1e-12)
            self.assertTrue(g['preserve_audited_tcp'])

if __name__=='__main__':unittest.main()
