"""An incomplete 5.5 mm insertion must not advance into the 3 mm installed gate."""
import copy
import unittest
from types import SimpleNamespace as NS
import numpy as np
from roboassemblybench.core.beam_coordinated import _tighten_final_insertions
from toolkits.factory_dual_franka_assembly.plumbers_block_ur5e_skills import UR5eAssemblyAtomicSkillAdapter as Adapter
from toolkits.factory_dual_franka_assembly.beam_support_state import BeamSupportState
from toolkits.factory_dual_franka_assembly.planner_primitives import pose_error

PREFIXES = {p: f'assemble_{i:02d}_part_{p}' for i,p in enumerate(['3','2','1','0'])}

def spec(pid='3', target=None):
    return dict(name='ur5e_move_part_to_staging', object=f'fabrica_beam_{pid}',
        requires_held_object=True, target_object_target=target or f'part_{pid}_assembled',
        require_target_object_pose_convergence=True, position_tolerance=.006,
        relaxed_position_tolerance=.015, relaxed_position_tolerance_after_steps=600,
        target_object_position_tolerance=.008, relaxed_target_object_position_tolerance=.015,
        orientation_tolerance=.08, target_object_orientation_tolerance=.10,
        target_object_convergence_axis=[0,0,-1],target_object_lateral_position_tolerance=.001)

class FinalInsertionCompletion(unittest.TestCase):
    def tighten(self, original):
        phase={'name':PREFIXES['3']+'_insert_09','local_skill':copy.deepcopy(original)}
        _tighten_final_insertions([phase],PREFIXES)
        return phase['local_skill']

    def completed(self, parameters, position_error, phase_step=1, rotation_error=0.):
        adapter=Adapter({});calls=[]
        adapter._mark_complete=lambda **kw:calls.append(kw)
        adapter._compliant_attachment_allowance=lambda **kw:(0.,0.,'pure_physical_grasp')
        orientation=np.array([np.cos(rotation_error/2),0.,0.,np.sin(rotation_error/2)])
        current={'position':np.array([0.,0.,position_error]),'orientation':orientation}
        target={'position':np.zeros(3),'orientation':np.array([1.,0.,0.,0.])}
        adapter._object_pose=lambda **kw:current
        adapter._target_object_pose=lambda **kw:target
        adapter._maybe_mark_complete(task=NS(phase_step_counter=phase_step),robot_name='left',
            skill_name='ur5e_move_part_to_staging',spec=parameters,target_pose=target,
            ik_target_pose=target,current_pose=current,tracked_objects={},current_q=None,target_q=None)
        return bool(calls)

    def installed_gate_fails(self, position_error, rotation_error=0.):
        task=BeamSupportState();errors=[]
        objects={n:NS(get_pose=lambda p=p:(np.asarray(p),np.array([1.,0.,0.,0.])),
                      unwrap=lambda n=n:NS(prim_path='/'+n)) for n,p in
                 [('fabrica_beam_6',[0.,0.,0.]),('fabrica_beam_3',[0.,0.,position_error])]}
        task._resolve_object=lambda n:objects[n]
        task._pair_contact_observation=lambda *a:{'valid':True,'force':0.}
        task._set_terminal_state=lambda *a,**kw:errors.append(kw)
        task.step_counter=1
        task._beam_assembly_integrity({'beam_installed_parts':['6','3']})
        return bool(errors)

    def test_real_old_predicate_accepts_failure_snapshot_but_next_gate_rejects(self):
        self.assertTrue(self.completed(spec(),.005509))
        self.assertTrue(self.installed_gate_fails(.005509))
        self.assertFalse(self.completed(self.tighten(spec()),.005509))

    def test_strict_completion_agrees_with_installed_budget(self):
        s=self.tighten(spec())
        self.assertTrue(self.completed(s,.001))
        self.assertFalse(self.installed_gate_fails(.001))
        self.assertFalse(self.completed(s,.002))

    def test_old_600step_relaxation_cannot_reenable_early_completion(self):
        self.assertTrue(self.completed(spec(),.010,phase_step=1000))
        self.assertFalse(self.completed(self.tighten(spec()),.010,phase_step=1000))
        self.assertTrue(self.completed(self.tighten(spec()),.001,phase_step=1000))

    def test_orientation_completion_is_inside_unchanged_50mrad_gate(self):
        self.assertTrue(self.completed(spec(),.001,rotation_error=.06))
        self.assertFalse(self.completed(self.tighten(spec()),.001,rotation_error=.06))
        self.assertTrue(self.completed(self.tighten(spec()),.001,rotation_error=.02))

    def test_only_final_four_beam_insertions_change(self):
        phases=[]
        for pid,prefix in PREFIXES.items():
            phases += [{'name':prefix+'_insert_08','local_skill':spec(pid,'waypoint')},
                       {'name':prefix+'_insert_09','local_skill':spec(pid)},
                       {'name':prefix+'_physical_hold_check','local_skill':spec(pid)}]
        phases += [{'name':'car_insert_09','local_skill':spec()},
                   {'name':PREFIXES['3']+'_insert_09','local_skill':{**spec(),'object':'fabrica_car_3'}}]
        before=copy.deepcopy(phases);_tighten_final_insertions(phases,PREFIXES)
        changed=[p['name'] for p,o in zip(phases,before) if p!=o]
        self.assertEqual(changed,[prefix+'_insert_09' for prefix in PREFIXES.values()])
        for p,o in zip(phases,before):
            if p==o:continue
            s=p['local_skill'];self.assertLess(s['target_object_position_tolerance'],.003)
            self.assertLess(s['target_object_orientation_tolerance'],.05)
            restored=copy.deepcopy(p)
            for key in ['position_tolerance','relaxed_position_tolerance','target_object_position_tolerance',
                        'relaxed_target_object_position_tolerance','orientation_tolerance','target_object_orientation_tolerance']:
                restored['local_skill'][key]=o['local_skill'][key]
            self.assertEqual(restored,o)

if __name__=='__main__':unittest.main()
