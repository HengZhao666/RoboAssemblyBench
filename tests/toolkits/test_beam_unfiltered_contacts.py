import importlib.util
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
from toolkits.factory_dual_franka_assembly import beam_physics_diagnostics as M

class UnfilteredContactRead(unittest.TestCase):
 def test_clone_timestep_and_sensor_direction_are_preserved(self):
  values=np.array([[1.,2.,3.]]);calls=[]
  def net(**kw):calls.append(kw);return values
  probe=NS(is_physics_handle_valid=lambda:True,get_net_contact_forces=net)
  r=M._unfiltered_contact_snapshot(probe,'/finger',1/240)
  values[:]=99.
  self.assertTrue(r['valid']);self.assertEqual(r['force_on_sensor_world'],[1.,2.,3.]);self.assertEqual(calls,[{'clone':True,'dt':1/240}]);self.assertEqual(r['sensor_body'],'/finger');self.assertNotIn('force_on_selected',r)
 def test_unavailable_read_is_labeled_without_synthesizing_force(self):
  p=NS(is_physics_handle_valid=lambda:True)
  r=M._unfiltered_contact_snapshot(p,'/beam',1/240)
  self.assertFalse(r['valid']);self.assertNotIn('force_on_sensor_world',r)
 def test_invalid_shape_and_nonfinite_read_fail(self):
  for value in [None,np.zeros(3),np.zeros((0,3)),np.zeros((1,2)),np.array([[0.,np.nan,0.]])]:
   p=NS(is_physics_handle_valid=lambda:True,get_net_contact_forces=lambda **kw:value)
   self.assertFalse(M._unfiltered_contact_snapshot(p,'/beam',1/240)['valid'])
 def test_invalid_handle_prevents_getter_call(self):
  def bad(**kw):raise AssertionError('must not read')
  r=M._unfiltered_contact_snapshot(NS(is_physics_handle_valid=lambda:False,get_net_contact_forces=bad),'/beam',1/240)
  self.assertFalse(r['valid'])
 def test_filtered_pair_total_stays_independent_of_net_read(self):
  p=NS(is_physics_handle_valid=lambda:True,get_contact_force_matrix=lambda **kw:np.array([[[0.,0.,2.]]]),get_friction_data=lambda **kw:(np.array([[1.,0.,0.]]),np.zeros((1,3)),np.array([[1]]),np.array([[0]])),get_net_contact_forces=lambda **kw:np.array([[8.,9.,10.]]))
  a=M._pair_load_snapshot(p,'/finger','/beam','/beam',1/240)
  n=M._unfiltered_contact_snapshot(p,'/finger',1/240)
  b=M._pair_load_snapshot(p,'/finger','/beam','/beam',1/240)
  self.assertEqual(a,b);self.assertEqual(b['total_force_on_selected_world'],[-1.,0.,-2.]);self.assertEqual(n['force_on_sensor_world'],[8.,9.,10.]);self.assertTrue(b['valid'])

class NetBoundaryIntegration(unittest.TestCase):
 def setUp(self):
  import sys,tempfile
  from unittest.mock import patch
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  sp=importlib.util.spec_from_file_location('fixtures',Path(__file__).with_name('test_beam_physics_diagnostics.py'));self.F=importlib.util.module_from_spec(sp);sp.loader.exec_module(self.F)
  self.task=self.F.task();self.task.phase='assemble_00_part_3_assembly_clearance';self.task._attachments={}
  self.task._resolved_objects={'fabrica_beam_3':self.F.wrapper(self.F.BodyView(),'/World/fabrica_beam_3')};self.calls=[]
  p=self.F.BodyView();p.get_contact_force_matrix=lambda **kw:np.zeros((1,1,3));p.get_friction_data=lambda **kw:(np.zeros((1,3)),np.zeros((1,3)),np.array([[0]]),np.array([[0]]))
  def net(**kw):self.calls.append(kw);return np.array([[0.,0.,3.]])
  p.get_net_contact_forces=net;self.probe=p
  self.task._contact_probes={('/World/fabrica_beam_3','/World/table'):p,('/World/fabrica_beam_3','/World/floor'):p}
  self.env=patch.dict('os.environ',{'BEAM_PHYSICS_DIAGNOSTICS':'1','BEAM_PHYSICS_DIAGNOSTICS_PATH':str(Path(self.tmp.name)/'trace.jsonl'),'BEAM_PHYSICS_DIAGNOSTICS_PHASES':'*','BEAM_PHYSICS_DIAGNOSTICS_OBJECTS':'fabrica_beam_3','BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED':'1'},clear=True);self.env.start();self.addCleanup(self.env.stop)
 def sample(self,boundary='after_physics'):return M.sample_boundary(self.task,boundary,self.F.clock(),cycle_step=9000)
 def test_one_existing_sensor_read_despite_multiple_cached_filters(self):
  r=self.sample();self.assertEqual(len(self.calls),1);self.assertEqual(len(r['contact_loads']['unfiltered_sensors']),1);self.assertTrue(r['contact_loads']['valid']);self.assertEqual(len(r['contact_loads']['pairs']),2)
 def test_disabled_and_wrong_phase_do_not_read(self):
  import os
  os.environ['BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED']='0';self.sample();self.assertEqual(self.calls,[])
  os.environ['BEAM_PHYSICS_DIAGNOSTICS_UNFILTERED']='1'
  self.task.phase='assemble_00_part_3_pickup_clearance';self.task._beam_physics_diagnostics.cycle_step=None;self.sample();self.assertEqual(self.calls,[])
 def test_turn_reads_once_per_sensor_on_all_four_boundaries(self):
  for boundary in ('before_action','after_action','after_physics','after_observation'):
   before=len(self.calls);r=self.sample(boundary)
   self.assertEqual(len(self.calls),before+1)
   self.assertEqual(r['contact_loads']['unfiltered_sensors'][0]['force_on_sensor_world'],[0.,0.,3.])
  self.assertEqual(self.calls,[{'clone':True,'dt':1/240}]*4)
 def test_optional_net_error_does_not_invalidate_valid_filtered_pairs(self):
  def unavailable(**kw):raise RuntimeError('unavailable')
  self.probe.get_net_contact_forces=unavailable;r=self.sample();self.assertTrue(r['contact_loads']['valid']);self.assertFalse(r['contact_loads']['unfiltered_sensors'][0]['valid'])

 def test_cached_gripper_sensor_is_included_and_not_negated_to_payload(self):
  self.task._contact_probes[('/World/robot/Robotiq_2F_85/base_link','/World/fabrica_beam_3')]=self.probe
  r=self.sample();sensors=r['contact_loads']['unfiltered_sensors']
  self.assertEqual(len(self.calls),2);self.assertEqual(len(sensors),2)
  self.assertEqual(sensors[1]['sensor_body'],'/World/robot/Robotiq_2F_85/base_link')
  self.assertEqual(sensors[1]['force_on_sensor_world'],[0.,0.,3.])

if __name__=='__main__':unittest.main()
