from pathlib import Path
import importlib.util,sys,unittest,copy,types,ast
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from toolkits.factory_dual_franka_assembly import beam_support_state as mod
from toolkits.factory_dual_franka_assembly import beam_physics_diagnostics as diag

def sample(index,name='part',moving=False):
 p=[index*.00001 if moving else 0.,0.,1.]
 return {'valid':True,'motion_valid':True,'physics_stamp_valid':True,'physics_epoch':0,
  'physics_step_index':index,'physics_time':index/240.,'physics_dt':1/240.,
  'position':p,'orientation':[1.,0.,0.,0.],
  'native_com':{'valid':True,'world_positions':[[p[0],0.,1.006]]},
  'motion_body_id':name,'motion_view_id':1}

class FinalPose(unittest.TestCase):
 def task(self):
  t=mod.BeamSupportState();t.phase='beam_final_unloaded_verification';t.phase_index=160;t.phase_entry_step=0
  t._beam_exploration_enabled=True;t._beam_exploration_state={'continue_after_first_insertion':True,'anchor':{'position':[0,0,1]}}
  t._as_list=lambda x:x if isinstance(x,list) else [x];t._extract_object_name=lambda x:x
  t._object_velocity_metrics=lambda *a,**kw:{'valid':True,'is_static':False,'linear_speed':.4}
  t._beam_exploration_record=lambda *a,**kw:None
  return t
 def test_pose_exploration_keeps_original_short_grasp_transition_gates(self):
  t=self.task()
  for phase in ['assemble_00_part_3_peel_physical_hold_check',
                'assemble_00_part_3_post_lift_restore_gravity',
                'assemble_00_part_3_physical_hold_check',
                'assemble_01_part_2_post_lift_restore_gravity']:
   t.phase=phase
   self.assertFalse(t._beam_continuing_stability(),phase)
  for phase in ['hold_6_to_3_switch_release','assemble_00_part_3_release',
                'hold_2_final_release','beam_final_unloaded_verification']:
   t.phase=phase
   self.assertTrue(t._beam_continuing_stability(),phase)

 def test_all_bodies_warm_together_and_raw_rejection_remains_separate(self):
  t=self.task();names=['p'+str(i) for i in range(5)]
  for i in range(482):
   t._beam_object_pose_sample=lambda n,i=i:sample(i,n)
   ready=t._beam_pose_objects_static(names,linear_threshold=.002,angular_threshold=.02)
   if i<480:self.assertFalse(ready)
  self.assertTrue(ready);self.assertEqual(len(t._beam_phase_pose_trackers),5)
 def test_one_moving_body_or_missing_com_never_qualifies(self):
  for kind in ['moving','com']:
   t=self.task()
   for i in range(600):
    def get(n):
     r=sample(i,n,moving=kind=='moving' and n=='bad')
     if kind=='com' and n=='bad':r['native_com']['valid']=False
     return r
    t._beam_object_pose_sample=get
    self.assertFalse(t._beam_pose_objects_static(['good','bad'],linear_threshold=.002,angular_threshold=.02))
 def test_captured_anchor_never_rebases_after_motion(self):
  t=self.task()
  for i in range(481):self.last=t._beam_phase_pose_result('p',sample(i))
  self.assertTrue(self.last['ready'])
  r=sample(481);r['position'][0]=.00031;r['native_com']['world_positions'][0][0]=.00031
  self.assertTrue(t._beam_phase_pose_result('p',r)['latched_failure'])
  self.assertFalse(t._beam_phase_pose_result('p',sample(482))['ready'])
 def test_cached_native_views_only_and_missing_views_are_explicit(self):
  t=self.task();t._object_prims={'part':types.SimpleNamespace(GetPath=lambda:'/part')};t._contact_probes={}
  t._beam_physics_stamp=lambda:{'valid':True,'epoch':0,'index':1,'time':1/240.,'dt':1/240.}
  self.assertFalse(t._beam_object_pose_sample('part')['valid'])
  class View:
   def is_physics_handle_valid(self):return True
   def get_world_poses(self,clone):return np.array([[0.,0.,1.]]),np.array([[1.,0.,0.,0.]])
   def get_velocities(self,clone):return np.zeros((1,6))
  v=View();t._contact_probes={('/part','/parent'):v};old=diag._body_com_snapshot
  try:
   diag._body_com_snapshot=lambda *a:{'valid':True,'world_positions':[[0.,0.,1.006]]}
   r=t._beam_object_pose_sample('part');self.assertTrue(r['valid']);self.assertEqual(r['motion_view_id'],id(v))
  finally:diag._body_com_snapshot=old

class FinalTaskMethods(unittest.TestCase):
 def test_short_transition_uses_the_actual_original_static_evaluator(self):
  source=ROOT/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
  node=next(n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_objects_static')
  ns={};exec(compile(ast.Module(body=[node],type_ignores=[]),'task','exec'),ns)
  T=type('T',(mod.BeamSupportState,),{'_objects_static':ns['_objects_static']})
  t=T();t._beam_exploration_enabled=True;t._beam_exploration_state={'continue_after_first_insertion':True,'anchor':{}}
  t._beam_exploration_state['anchor']={'position':[0,0,1]}
  t.phase='assemble_00_part_3_post_lift_restore_gravity'
  t._as_list=lambda x:x;t._extract_object_name=lambda x:x
  t._object_velocity_metrics=lambda *a,**kw:{'valid':True,'is_static':True}
  t._beam_pose_objects_static=lambda *a,**kw:self.fail('Original short transition must not warm a 2s window')
  self.assertTrue(t._objects_static(['part'],linear_threshold=.08,angular_threshold=2.5))

 def test_actual_all_of_warms_static_even_while_support_is_false(self):
  source=ROOT/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
  tree=ast.parse(source.read_text());node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_evaluate_advance_condition')
  ns={};exec(compile(ast.Module(body=[node],type_ignores=[]),'task','exec'),ns)
  T=type('T',(mod.BeamSupportState,),{'_evaluate_advance_condition':ns['_evaluate_advance_condition']})
  for continuation in [True,False]:
   t=T();t.phase='beam_final_unloaded_verification';t.phase_step_counter=0;t._beam_exploration_enabled=True
   t._beam_exploration_state={'continue_after_first_insertion':continuation,'anchor':{'position':[0,0,1]}};calls=[]
   t._beam_table_supported=lambda a:False
   t._objects_static=lambda *a,**kw:calls.append('static') or True
   advance={'type':'all_of','conditions':[{'type':'beam_table_supported'}, {'type':'objects_static','objects':['a','b']}]}
   self.assertFalse(t._evaluate_advance_condition(phase_spec={},advance=advance))
   self.assertEqual(calls,['static'] if continuation else [])


 def test_terminal_check_reuses_complete_final_pose_gate_and_keeps_other_requirements(self):
  source=ROOT/'internutopia_extension/tasks/factory_dual_franka_assembly_task.py'
  node=next(n for n in ast.walk(ast.parse(source.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_check_success')
  ns={'pose_within_tolerance':lambda **kw:kw['current_position']==kw['target_position']}
  exec(compile(ast.Module(body=[node],type_ignores=[]),'task','exec'),ns)
  T=type('T',(mod.BeamSupportState,),{'_check_success':ns['_check_success']})
  def task(continuing=True,final_ready=True):
   t=T();t.phase='beam_final_unloaded_verification';t.phase_index=1
   t.phase_specs=[{'beam_coordinated':True},{'name':t.phase,'advance':{'type':'all_of'}}]
   t._beam_exploration_enabled=True;t._beam_exploration_state={'continue_after_first_insertion':continuing}
   t.cfg=types.SimpleNamespace(success_criteria=[{'object':'part','target':'assembled','require_static':True,'require_released':True}])
   t._evaluate_advance_condition=lambda **kw:final_ready
   t._resolve_object=lambda _:types.SimpleNamespace(get_pose=lambda:([0,0,0],[1,0,0,0]))
   t._resolve_target_pose_spec=lambda _:(None,[0,0,0],[1,0,0,0],{})
   t._target_tolerances=lambda **kw:(.003,.04);t._attachments={}
   t._object_velocity_metrics=lambda *a,**kw:{'valid':True,'is_static':False}
   return t
  self.assertTrue(task()._check_success())
  self.assertFalse(task(continuing=False)._check_success())
  self.assertFalse(task(final_ready=False)._check_success())
  t=task();t._attachments={'part':{}};self.assertFalse(t._check_success())
  t=task();t._resolve_target_pose_spec=lambda _:(None,[1,0,0],[1,0,0,0],{});self.assertFalse(t._check_success())
  t=task();t.phase_index=0;self.assertFalse(t._check_success())
  t=task();t.phase_specs[0]['beam_coordinated']=False;self.assertFalse(t._check_success())


class EpisodeBudget(unittest.TestCase):
 def test_budget_covers_original_timeouts_and_never_shortens_larger_budget(self):
  from toolkits.factory_dual_franka_assembly.beam_exploration import continuation_step_budget
  phases=[{'timeout_steps':360},{'timeout_steps':720},{}];old=copy.deepcopy(phases)
  self.assertEqual(continuation_step_budget(phases,1000,1200),2284)
  self.assertEqual(continuation_step_budget(phases,5000,1200),5000)
  self.assertEqual(phases,old)
 def test_invalid_phase_graph_cannot_silently_get_a_budget(self):
  from toolkits.factory_dual_franka_assembly.beam_exploration import continuation_step_budget
  for phases,budget,default in [([],1,1),([{}],0,1200),([{}],True,1200),([{}],1,0),([{'timeout_steps':0}],1,1200),([{'timeout_steps':None}],1,1200)]:
   with self.assertRaises(ValueError):continuation_step_budget(phases,budget,default)
 def test_actual_compiler_changes_only_episode_budget_in_continuation_mode(self):
  from unittest.mock import patch
  import os
  import roboassemblybench.core.beam_physical_compiler as compiler
  from toolkits.factory_dual_franka_assembly.task_specs import load_task_recipe
  with patch.object(compiler,'_move_grasp_ik_metrics',return_value={'ik_feasible':True}):
   with patch.dict(os.environ,{'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'1','BEAM_EXPLORE_POST_SEAT':'1','BEAM_EXPLORE_CONTINUE':'0'}):
    old=load_task_recipe('fabrica_beam_ur5e_staged',scene_profile='taoyuan_grscenes_tabletop')
   with patch.dict(os.environ,{'BEAM_COLUMN3_SIDE_CLEARANCE_ROUTE':'1','BEAM_EXPLORE_POST_SEAT':'1','BEAM_EXPLORE_CONTINUE':'1'}):
    new=load_task_recipe('fabrica_beam_ur5e_staged',scene_profile='taoyuan_grscenes_tabletop')
  self.assertGreaterEqual(new['max_steps'],sum(p['timeout_steps'] for p in new['phases'])+len(new['phases'])+1)
  self.assertGreater(new['max_steps'],old['max_steps'])
  restored=copy.deepcopy(new);restored['max_steps']=old['max_steps'];restored['recipe_fingerprint']=old['recipe_fingerprint']
  self.assertEqual(restored,old)

if __name__=='__main__':unittest.main()
