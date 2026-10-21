from types import SimpleNamespace as NS
import numpy as np
import pytest

from toolkits.constraint_checking.integration.entity_geometry import Collider, EntityGeometryChecker
from toolkits.constraint_checking.integration.contact_policy import AssemblyContactPolicy
from toolkits.constraint_checking.integration.stage_precheck import StageTrajectoryPrechecker


def cube(size):
    h = size/2
    return np.asarray([[x,y,z] for x in [-h,h] for y in [-h,h] for z in [-h,h]])


def test_complete_fingertip_hull_detects_old_style_cross_arm_overlap():
    pytest.importorskip('fcl')
    shapes = [Collider('/left/tip', '/left/finger', 'left/Gripper/Robotiq_2F_85/left_inner_finger', 'robot:left', 'left', cube(.02), 'convexHull'),
              Collider('/right/wrist', '/right/wrist', 'right/wrist_2_link', 'robot:right', 'right', cube(.04), 'convexDecomposition')]
    checker = EntityGeometryChecker(shapes)
    left=np.eye(4); left[0,3]=.0295
    r=checker.check({'/left/finger':left,'/right/wrist':np.eye(4)})
    assert r['closest_nonexempt_pair']['distance'] < 0
    assert r['events'][0]['entity_b']=='right/wrist_2_link'
    left[0,3]=.04
    r=checker.check({'/left/finger':left,'/right/wrist':np.eye(4)})
    assert r['closest_nonexempt_pair']['distance'] == pytest.approx(.01, abs=1e-7)


def test_containment_is_detected_without_triangle_surface_crossing():
    pytest.importorskip('fcl')
    shapes = [Collider('/a','/a','payload','object:p',None,cube(.01),'sdf'),
              Collider('/b','/b','right/wrist_2_link','robot:r','right',cube(.04),'convexHull')]
    r=EntityGeometryChecker(shapes).check({'/a':np.eye(4),'/b':np.eye(4)})
    assert r['closest_nonexempt_pair']['distance'] < 0


def test_missing_body_pose_is_an_error_instead_of_zero_collisions():
    pytest.importorskip('fcl')
    checker=EntityGeometryChecker([Collider('/a','/a','p','object:p',None,cube(.01),'sdf')])
    with pytest.raises(KeyError): checker.check({})


def test_simulation_selected_offset_is_unknown_and_never_negative_infinity():
    import json
    pytest.importorskip('fcl')
    shapes=[Collider('/a','/a','left/wrist','robot:l','left',cube(.02),'convexHull',float('-inf')),
            Collider('/b','/b','right/wrist','robot:r','right',cube(.02),'convexHull')]
    checker=EntityGeometryChecker(shapes)
    b=np.eye(4);b[0,3]=.019
    result=checker.check({'/a':np.eye(4),'/b':b})
    assert result['events'] and result['events'][0]['contact_offset_sum_m']==0
    assert len(checker.coverage()['unknown_contact_offset_colliders'])==2
    json.dumps(result,allow_nan=False)


def test_native_same_named_bases_use_exact_body_paths():
    from toolkits.constraint_checking.integration.entity_geometry import TaskEntityGeometryMonitor
    arm='/robot/base_link';gripper='/robot/Gripper/Robotiq_2F_85/base_link'
    def wrapper(path):
        value=NS(prim_path=path);return NS(unwrap=lambda:value)
    bases={gripper:wrapper(gripper),arm:wrapper(arm)}
    monitor=TaskEntityGeometryMonitor.__new__(TaskEntityGeometryMonitor)
    monitor.task=NS(robots={'left':NS(_rigid_body_map=bases)})
    assert monitor._native_body('left',arm) is bases[arm]
    assert monitor._native_body('left',gripper) is bases[gripper]
    bases[arm]=bases[gripper]
    with pytest.raises(ValueError,match='alias'):monitor._native_body('left',arm)


def test_beam_classification_bypasses_qualification_and_preserves_attachment_state():
    import copy
    attachment={'mode':'pure_physical_grasp','robot_name':'left','hold_valid':True,'position':[1,2,3]}
    task=NS(phase='carry',_attachments={'p':attachment},_beam_shared_grasps={('p','right'):{}})
    def forbidden():raise AssertionError('qualification getter must not run')
    task.get_tracked_object_states=forbidden
    before=copy.deepcopy(task._attachments)
    policy=AssemblyContactPolicy();snapshot=policy.snapshot_from_task(task)
    for robot in ['left','right']:
        decision=policy.classify(NS(entity_a=robot+'/Gripper/Robotiq_2F_85/left_inner_finger',entity_b='p',distance=-.001),task,snapshot=snapshot)
        assert decision.classification=='allowed_contact'
    assert task._attachments==before


def test_many_geometry_pairs_take_one_generic_context_snapshot():
    pytest.importorskip('fcl')
    calls=[]
    task=NS(phase='carry',get_tracked_object_states=lambda:(calls.append(1) or {'p':{'grasped_by':'left'}}))
    shapes=[Collider('/a','/a','left/Gripper/Robotiq_2F_85/left_inner_finger','robot:l','left',cube(.02),'convexHull'),
            Collider('/b','/b','right/wrist_2_link','robot:r','right',cube(.03),'convexHull'),
            Collider('/p','/p','p','object:p',None,cube(.02),'sdf')]
    result=EntityGeometryChecker(shapes).check({k:np.eye(4) for k in ['/a','/b','/p']},policy=AssemblyContactPolicy(),task=task)
    assert len(result['events'])==3 and len(calls)==1


def test_object_pose_monitor_uses_existing_native_reader_without_hold_check():
    from toolkits.constraint_checking.integration.entity_geometry import TaskEntityGeometryMonitor
    path='/objects/p';raw=NS(prim_path=path)
    body=NS(unwrap=lambda:raw,get_pose=lambda:(np.array([.1,.2,.3]),[1,0,0,0]))
    def forbidden():raise AssertionError('qualification getter must not run')
    task=NS(_object_prims={'p':NS(GetPath=lambda:path)},_resolved_objects={'p':body},get_tracked_object_states=forbidden)
    monitor=TaskEntityGeometryMonitor.__new__(TaskEntityGeometryMonitor)
    monitor.task=task;monitor.readers={path:None};monitor.body_robot_links={}
    pose=monitor.current_poses()[path]
    np.testing.assert_allclose(pose[:3,3],[.1,.2,.3])


def test_coarse_collision_states_bypass_beam_qualification_getter():
    from toolkits.constraint_checking.integration.runtime_monitor import collision_object_states
    def forbidden():raise AssertionError('qualification getter must not run')
    task=NS(cfg=NS(tracked_object_names=['p']),_object_prims={'p':object()},
            _resolved_objects={'p':NS(get_pose=lambda:([.1,.2,.3],[1,0,0,0]))},
            _attachments={'p':{'robot_name':'left','mode':'pure_physical_grasp','hold_valid':True}},
            get_tracked_object_states=forbidden)
    state=collision_object_states(task)['p']
    assert state['position']==[.1,.2,.3] and state['grasped_by']=='left'
    assert task._attachments['p']['hold_valid'] is True


def test_native_close_phase_only_exempts_own_target_inner_fingers():
    task=NS(phase='base_6_close_and_attach',_attachments={},
            get_current_phase_spec=lambda:{'local_skill':{'robot':'left','object':'p'}})
    policy=AssemblyContactPolicy()
    def classification(entity,obj='p'):
        return policy.classify(NS(entity_a=entity,entity_b=obj,distance=-.001),task).classification
    assert classification('left/Gripper/Robotiq_2F_85/left_inner_finger')=='allowed_contact'
    assert classification('right/Gripper/Robotiq_2F_85/left_inner_finger')=='collision'
    assert classification('left/Gripper/Robotiq_2F_85/left_inner_finger','other')=='collision'
    assert classification('left/wrist_2_link')=='collision'


def test_held_object_contact_is_exempted_only_for_own_exact_inner_finger():
    task=NS(phase='carry',get_tracked_object_states=lambda:{'p':{'grasped_by':'left'}})
    policy=AssemblyContactPolicy()
    def classify(entity):
        return policy.classify(NS(entity_a=entity,entity_b='p',distance=-.001),task).classification
    assert classify('left/Gripper/Robotiq_2F_85/left_inner_finger')=='allowed_contact'
    for entity in ['right/Gripper/Robotiq_2F_85/left_inner_finger','left/wrist_2_link',
                   'left/Gripper/Robotiq_2F_85/base_link',
                   'left/wrist_3_link->Gripper/Robotiq_2F_85/left_inner_finger']:
        assert classify(entity)=='collision'


def test_mounted_base_link_never_uses_bare_arm_base_link():
    class Solver:
        def set_robot_base_pose(self,**kw):pass
        def get_all_frame_names(self):return ['base_link','wrist_3_link']
        def compute_forward_kinematics(self,name,joint_positions):return np.array([.5,.3,1.]),np.eye(3)
    ik=NS(_kinematics_solver=Solver(),_robot_scale=1.,get_ik_base_world_pose=lambda:(np.zeros(3),[1,0,0,0]))
    robot=NS(controllers={'arm_ik':ik})
    task=NS(robots={'left':robot})
    checker=StageTrajectoryPrechecker()
    result=checker._forward_link_positions(task,'left',np.zeros(6))
    assert 'base_link' in result
    assert 'Gripper/Robotiq_2F_85/base_link' not in result
    assert len([f for f in checker._frame_coverage['left']['missing_model_frames'] if f.startswith('Gripper/')])==9


def test_holding_robot_is_included_when_only_other_arm_has_actions():
    from test_stage_precheck import make_task
    task=make_task(); task.robots['right']=task.robots['left']
    checker=StageTrajectoryPrechecker(check_stride=1,num_waypoints=2)
    checker.observe(task,{'left':{'arm_joint':[[.1,0,0,0,0,0]]}})
    assert checker.finalize()['segments_checked']==2
