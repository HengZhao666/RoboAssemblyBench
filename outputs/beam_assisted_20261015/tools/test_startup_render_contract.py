"""Mock startup contracts only; no Kit, physics, GPU or USD process is started."""
from pathlib import Path
import argparse
import ast
import copy
import hashlib
import json
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np

SOURCE=Path(__file__).with_name('assisted_worker.py')
FUNCTIONS={'native_startup_snapshot','native_snapshot_difference','usd_native_body_comparison',
    'publish_native_results_to_usd','camera_render_parameter_comparison',
    'validate_initial_native_joints','validate_runtime_configuration_binding',
    'derive_expected_scene_configuration'}


def plain(value):
    if isinstance(value,dict):return {key:plain(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)):return [plain(item) for item in value]
    return value.tolist() if hasattr(value,'tolist') else value


class Matrix:
    def __init__(self,m=1):self.m=np.eye(4) if np.isscalar(m) else np.asarray(m,float).copy()
    def __array__(self,dtype=None,copy=None):return np.asarray(self.m,dtype=dtype)
    def ExtractRotationQuat(self):return self.m[:3,:3].T
    def ExtractTranslation(self):return self.m[3,:3]
    def SetRotate(self,rotation):self.m[:3,:3]=rotation.T
    def SetTranslateOnly(self,position):self.m[3,:3]=position


class Prim:
    def __init__(self,m,targets=None):self.m=m;self.targets=targets or []
    def IsValid(self):return True
    def GetRelationship(self,name):return types.SimpleNamespace(GetTargets=lambda:self.targets)


class Cache:
    def __init__(self,*args):pass
    def GetLocalToWorldTransform(self,prim):return Matrix(prim.m.T)


class StartupRenderContractTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.addCleanup(self.directory.cleanup)
        tree=ast.parse(SOURCE.read_text());self.nodes=[node for node in tree.body
            if isinstance(node,ast.FunctionDef) and node.name in FUNCTIONS]
        self.scope={'copy':copy};exec(compile(ast.Module(body=self.nodes,type_ignores=[]),str(SOURCE),'exec'),self.scope)
        self.B=np.eye(4);self.B[:3,3]=[1,2,3];usd=self.B.copy();usd[0,3]+=.4
        self.body=Prim(usd);self.q=np.zeros(12);self.stamp={'valid':True,'epoch':0,'index':3,'time':.0125}
        self.joint_names=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint','finger_joint']+['passive_'+str(i) for i in range(5)]
        initial={name:0. for name in self.joint_names[:7]}
        robot=types.SimpleNamespace(articulation=types.SimpleNamespace(get_joint_positions=lambda:self.q.copy(),dof_names=self.joint_names),
            config=types.SimpleNamespace(name='r_0',gripper_dof_name='finger_joint',initial_joint_positions=initial),
            _resolved_articulation_dof_name=lambda name:name if name in self.joint_names else None)
        self.task=types.SimpleNamespace(_beam_physics_stamp=lambda:dict(self.stamp),
            robots={'r':robot},
            step_counter=0,phase_index=0,phase_entry_step=0,phase_step_counter=0,steps=0,
            phase_transition_history=[{'event':'initialize','phase':'first'}])
        self.monitor=types.SimpleNamespace(current_poses=lambda:{'/body':self.B.copy()},
            stage=types.SimpleNamespace(GetPrimAtPath=lambda path:self.body))
        self.calls=[]
        def publish(*args):self.calls.append(args);self.body.m=self.B.copy()
        physx=types.ModuleType('omni.physx');physx.get_physx_interface=lambda:types.SimpleNamespace(update_transformations=publish)
        omni=types.ModuleType('omni');omni.physx=physx
        pxr=types.SimpleNamespace(Gf=types.SimpleNamespace(Matrix4d=Matrix),
            Usd=types.SimpleNamespace(TimeCode=types.SimpleNamespace(Default=lambda:None)),
            UsdGeom=types.SimpleNamespace(XformCache=Cache))
        self.modules=patch.dict(sys.modules,{'pxr':pxr,'omni':omni,'omni.physx':physx});self.modules.start();self.addCleanup(self.modules.stop)
        self.scope.update(out=Path(self.directory.name),demo=types.SimpleNamespace(
            _write_json=lambda path,value:path.write_text(json.dumps(plain(value))),_to_jsonable=plain))

    def test_existing_result_publication_aligns_usd_without_native_changes(self):
        self.scope['publish_native_results_to_usd'](self.task,self.monitor)
        report=json.loads((Path(self.directory.name)/'native_usd_startup_sync.json').read_text())
        self.assertTrue(report['passed']);self.assertEqual(self.calls,[(False,True,False,False)])
        self.assertAlmostEqual(report['usd_before']['max_position_error_m'],.4)
        self.assertEqual(report['usd_after']['max_position_error_m'],0.)
        forbidden={'step','simulate','set_joint_positions','set_world_pose','set_pose','set_linear_velocity','set_angular_velocity'}
        called={node.func.attr for root in self.nodes for node in ast.walk(root)
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)}
        self.assertFalse(forbidden&called)

    def test_any_native_or_task_change_is_rejected(self):
        before=self.scope['native_startup_snapshot'](self.task,self.monitor)
        for key in ('joint_positions','body_poses','physics_stamp','task_counters','base_task_steps','phase_transition_history'):
            with self.subTest(key=key):
                after=copy.deepcopy(before)
                if key=='joint_positions':after[key]['r'][0]=1e-12
                elif key=='body_poses':after[key]['/body'][0][3]+=1e-12
                elif key=='physics_stamp':after[key]['index']+=1
                elif key=='task_counters':after[key][3]+=1
                elif key=='base_task_steps':after[key]+=1
                else:after[key].append({'event':'unexpected transition'})
                self.assertFalse(self.scope['native_snapshot_difference'](before,after)['unchanged'])
        self.task.phase_transition_history.append({'event':'later change'})
        self.assertEqual(len(before['phase_transition_history']),1)

    def test_rgb_camera_relationship_and_actual_renderer_view_must_match(self):
        camera='/camera';product='/product';world=np.eye(4);world[:3,3]=[.1,.2,.3]
        stage=types.SimpleNamespace(GetPrimAtPath=lambda path:Prim(world if path==camera else np.eye(4),[camera]))
        entries=[{'spec':{'video_key':'camera'+str(i)},'sensor':types.SimpleNamespace(camera_prim_path=camera),
            'backend':types.SimpleNamespace(prim=types.SimpleNamespace(prim_path=camera)),
            'render_product_path':product,'annotator':types.SimpleNamespace(get_data=lambda:{'cameraViewTransform':np.linalg.inv(world).T})}
            for i in range(3)]
        self.assertTrue(self.scope['camera_render_parameter_comparison'](stage,entries)['passed'])
        entries[0]['annotator']=types.SimpleNamespace(get_data=lambda:{'cameraViewTransform':np.eye(4)})
        self.assertFalse(self.scope['camera_render_parameter_comparison'](stage,entries)['passed'])
        entries[0]['sensor'].camera_prim_path='/wrong_camera'
        report=self.scope['camera_render_parameter_comparison'](stage,entries)
        self.assertFalse(report['passed']);self.assertIn('different camera',report['rows'][0]['error'])

    def test_initial_native_joint_check_catches_skipped_reset_and_wrong_branch(self):
        before=self.scope['native_startup_snapshot'](self.task,self.monitor)
        self.scope['validate_initial_native_joints'](self.task,before)
        for error in (.01,2*np.pi):
            changed=copy.deepcopy(before);changed['joint_positions']['r'][0]=error
            with self.subTest(error=error),self.assertRaises(RuntimeError):
                self.scope['validate_initial_native_joints'](self.task,changed)
        self.task.robots['r'].config.initial_joint_positions.pop('wrist_3_joint')
        with self.assertRaises(RuntimeError):self.scope['validate_initial_native_joints'](self.task,before)

    def test_runtime_binding_rejects_wrong_phase_hash_and_wrong_environment(self):
        root='/World/env_0/episode_000000';robot_root=root+'/robots/ur5e_left'
        self.task.env_id=0;self.task.env_offset=[0,0,0]
        config={'robots':[{'prim_path':robot_root}]}
        self.task.config=types.SimpleNamespace(model_dump=lambda **kwargs:copy.deepcopy(config))
        self.task.robots['r'].articulation._root_prim_path=robot_root
        self.scope['expected_startup_configuration']={'runtime_configuration_signature':'config-hash',
            'phase_target_signature':'phase-hash','expected_runtime_config':config,'episode_root':root,
            'scene_resolution_derived':{'already_derived_for_binding_test':True}}
        self.scope['runtime_env']=types.SimpleNamespace(runner=types.SimpleNamespace(current_tasks={'task':self.task}))
        actual={'configuration_signature':'config-hash','phase_target_signature':'phase-hash'}
        self.scope['validate_runtime_configuration_binding'](self.task,actual)
        with self.assertRaises(RuntimeError):self.scope['validate_runtime_configuration_binding'](
            self.task,dict(actual,phase_target_signature='mutated-phase-hash'))
        self.task.env_id=1
        with self.assertRaises(RuntimeError):self.scope['validate_runtime_configuration_binding'](self.task,actual)

    def test_scene_expected_fields_are_derived_without_loading_or_copying_actual(self):
        class Config:
            def __init__(self):
                self.scene_asset_path='declared-primary';self.scene_asset_fallback_path='declared-fallback'
                self.scene_profile_metadata={'scene_family':'primary-family','fallback_scene_family':'fallback-family'}
            def model_dump(self,**kwargs):return copy.deepcopy(vars(self))
        config=Config();calls=[]
        class Resolver:
            @classmethod
            def _resolve_scene_asset_path(cls,model):calls.append('resolve');return '/resolved/fallback'
            @classmethod
            def _scene_asset_preference(cls,model):calls.append('preference');return 'fallback_first'
            @classmethod
            def _resolve_isaac_asset_path(cls,path):calls.append(('resolve_asset',path));return '/resolved/fallback'
            def __init__(self):raise AssertionError('Scene must not be instantiated')
            @classmethod
            def load(cls,*args):raise AssertionError('Scene must not be loaded')
        signatures=types.SimpleNamespace(configuration_signature=lambda model:
            hashlib.sha256(json.dumps(model.model_dump(),sort_keys=True).encode()).hexdigest())
        modules={'internutopia.core.scene.isaacsim.scene':types.SimpleNamespace(IsaacsimScene=Resolver),
            'toolkits.factory_dual_franka_assembly.beam_assisted_preflight':signatures}
        self.scope['expected_runtime_model']=config
        self.scope['expected_startup_configuration']={'runtime_configuration_signature':'manager-only-hash',
            'expected_runtime_config':config.model_dump()}
        with patch.dict(sys.modules,modules):self.scope['derive_expected_scene_configuration']()
        expected=self.scope['expected_startup_configuration']
        derived=expected['expected_runtime_config']
        self.assertEqual(derived['scene_asset_path'],'/resolved/fallback')
        self.assertEqual(derived['scene_asset_source'],'fallback')
        self.assertEqual(derived['scene_asset_selection_policy'],'fallback_first')
        self.assertEqual(derived['scene_asset_selection_reason'],'fallback_preferred_by_scene_profile')
        self.assertEqual(derived['resolved_scene_family'],'fallback-family')
        self.assertEqual(config.scene_asset_path,'declared-primary')
        self.assertEqual(expected['manager_only_runtime_configuration_signature'],'manager-only-hash')
        self.assertEqual(calls,['resolve','preference',('resolve_asset','declared-fallback')])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--report',type=Path);args=parser.parse_args()
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(StartupRenderContractTests))
    report={'passed':result.wasSuccessful(),'tests_run':result.testsRun,
        'failures':len(result.failures),'errors':len(result.errors),
        'worker_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'test_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'scope':'mock executable worker contracts and static forbidden state-write/physics-call check',
        'not_certified':['real SDK result-publication effect','RTX renderer freshness','native unchanged state','actual image visibility']}
    if args.report:args.report.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
