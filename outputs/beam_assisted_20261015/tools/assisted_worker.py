"""Beam assisted runner; passive geometry and original three-view recording."""
from pathlib import Path
import json,os,sys,hashlib,time,copy
C=Path(os.environ['BEAM_ROUTE_SOURCE']);sys.path.insert(0,str(C))
from toolkits.factory_dual_franka_assembly import generate_demos as demo
out=Path(os.environ['BEAM_CONTACT_TRACE_PATH']).parent
original_build=demo._build_env
runtime_env=None
expected_startup_configuration=None
expected_runtime_model=None
def build(*args,**kwargs):
 global runtime_env,expected_startup_configuration,expected_runtime_model
 configs=kwargs.get('task_configs',args[0] if args else [])
 demo._write_json(out/'task_config_snapshot.json',{'task_configs':[c.model_dump(mode='python',serialize_as_any=True) for c in configs]})
 from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration
 reports=[assert_assisted_configuration(c) for c in configs]
 launch=json.loads((out/'launch_gate.json').read_text())['gate']
 expected=launch['actual_configuration_signature']
 if len(reports)!=1 or reports[0]['configuration_signature']!=expected:
  raise RuntimeError('actual launch configuration differs from reviewed randomized episode')
 from internutopia.core.task_config_manager.base import setup_offset_for_assets,runtime_root_path
 expected_config=copy.deepcopy(configs[0])
 setup_offset_for_assets(expected_config,0,[0.,0.,0.])
 expected_runtime_model=copy.deepcopy(expected_config)
 runtime_report=assert_assisted_configuration(expected_config)
 expected_startup_configuration={'build_configuration_signature':reports[0]['configuration_signature'],
   'runtime_configuration_signature':runtime_report['configuration_signature'],
   'phase_target_signature':reports[0]['phase_target_signature'],'env_id':0,'env_offset':[0.,0.,0.],
   'episode_root':runtime_root_path(expected_config,0),
   'method':'deepcopy reviewed config then identical manager setup_offset_for_assets(config,0,[0,0,0])',
   'expected_runtime_config':expected_config.model_dump(mode='python',serialize_as_any=True)}
 if runtime_report['phase_target_signature']!=reports[0]['phase_target_signature']:
  raise RuntimeError('framework environment mapping unexpectedly changed reviewed phase targets')
 demo._write_json(out/'expected_runtime_configuration.json',expected_startup_configuration)
 demo._write_json(out/'compiled_configuration_preflight.json',{'passed':True,'reports':reports,
   'scope':'actual randomized episode configurations before simulator construction; not dynamic or rendered-image proof'})
 runtime_env=original_build(*args,**kwargs)
 bounded_primary=launch.get('bounded_primary_scene_probe')
 if bounded_primary is not None:
  helper=Path(__file__).with_name('bounded_scene_http_probe.py')
  if hashlib.sha256(helper.read_bytes()).hexdigest()!=bounded_primary['helper_sha256']:
   raise RuntimeError('bounded scene transport helper differs from reviewed identity')
  from bounded_scene_http_probe import install
  install({'bounded_primary_scene_probe':bounded_primary},out)
 return runtime_env
demo._build_env=build

def derive_expected_scene_configuration():
 """Mirror only Scene.load's five resolver-derived config assignments."""
 global expected_startup_configuration
 if expected_startup_configuration.get('scene_resolution_derived'):return
 from internutopia.core.scene.isaacsim.scene import IsaacsimScene
 from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import configuration_signature
 config=copy.deepcopy(expected_runtime_model)
 fields=('scene_asset_source','scene_asset_selection_policy','scene_asset_selection_reason',
    'resolved_scene_family','scene_asset_path')
 before={name:getattr(config,name,None) for name in fields}
 # These are the same read-only resolver/preference methods used by load();
 # never instantiate Scene or call load(), validate_scene_file() or create_prim().
 usd_path=IsaacsimScene._resolve_scene_asset_path(config)
 fallback_path=getattr(config,'scene_asset_fallback_path',None)
 preference=IsaacsimScene._scene_asset_preference(config);resolved_fallback=None
 if fallback_path:
  try:resolved_fallback=IsaacsimScene._resolve_isaac_asset_path(fallback_path)
  except Exception:pass
 using_fallback=bool(resolved_fallback and usd_path==resolved_fallback)
 metadata=getattr(config,'scene_profile_metadata',{}) or {}
 config.scene_asset_source='fallback' if using_fallback else 'primary'
 config.scene_asset_selection_policy=preference or 'primary_first'
 config.scene_asset_selection_reason=('fallback_preferred_by_scene_profile'
   if using_fallback and preference in {'fallback','fallback_first'} else 'first_existing_asset')
 config.resolved_scene_family=str(metadata.get('fallback_scene_family' if using_fallback else 'scene_family',''))
 config.scene_asset_path=usd_path
 expected=expected_startup_configuration
 expected['manager_only_runtime_configuration_signature']=expected['runtime_configuration_signature']
 expected['manager_only_expected_runtime_config']=expected['expected_runtime_config']
 expected['runtime_configuration_signature']=configuration_signature(config)
 expected['expected_runtime_config']=config.model_dump(mode='python',serialize_as_any=True)
 expected['scene_resolution_derived']={'method':'Scene.load resolver/preference methods and its five exact assignments; no load or prim creation',
   'fields_before':before,'expected_fields_after':{name:getattr(config,name) for name in fields},
   'resolved_fallback_path':resolved_fallback,'using_fallback':using_fallback,
   'source':'internutopia/core/scene/isaacsim/scene.py:load'}
 demo._write_json(out/'expected_runtime_configuration.json',expected)

def validate_runtime_configuration_binding(task,runtime_config):
 derive_expected_scene_configuration()
 expected=expected_startup_configuration
 actual_config=task.cfg if hasattr(task,'cfg') else task.config
 actual_roots={name:str(robot.articulation._root_prim_path) for name,robot in task.robots.items()}
 expected_roots=sorted(str(robot['prim_path']) for robot in expected['expected_runtime_config']['robots'])
 report={'expected_configuration_signature':expected['runtime_configuration_signature'],
   'actual_configuration_signature':runtime_config['configuration_signature'],
   'expected_phase_target_signature':expected['phase_target_signature'],
   'actual_phase_target_signature':runtime_config['phase_target_signature'],
   'actual_env_id':getattr(task,'env_id',None),'actual_env_offset':getattr(task,'env_offset',None),
   'expected_episode_root':expected['episode_root'],'expected_robot_roots':expected_roots,
   'actual_robot_roots':actual_roots,'active_task_count':len(runtime_env.runner.current_tasks),
   'actual_runtime_config':actual_config.model_dump(mode='python',serialize_as_any=True),'passed':False}
 report['passed']=bool(report['active_task_count']==1 and report['actual_env_id']==0
   and list(report['actual_env_offset'] or [])==[0.,0.,0.]
   and sorted(actual_roots.values())==expected_roots
   and all(path.startswith(expected['episode_root']+'/robots/') for path in actual_roots.values())
   and report['actual_configuration_signature']==report['expected_configuration_signature']
   and report['actual_phase_target_signature']==report['expected_phase_target_signature'])
 demo._write_json(out/'runtime_configuration_binding.json',report)
 if not report['passed']:raise RuntimeError('initialized task/config differs from reviewed configuration after the exact framework env0 mapping')

def native_startup_snapshot(task,monitor):
 import numpy as np
 stamp=task._beam_physics_stamp()
 if stamp.get('valid') is not True:raise RuntimeError('startup native physics clock is invalid')
 q={name:np.asarray(robot.articulation.get_joint_positions(),float).copy()
    for name,robot in task.robots.items()}
 poses={path:np.asarray(value,float).copy() for path,value in monitor.current_poses().items()}
 if any(not np.all(np.isfinite(value)) for value in list(q.values())+list(poses.values())):
  raise RuntimeError('startup native snapshot contains nonfinite q or body pose')
 if task._beam_physics_stamp()!=stamp:raise RuntimeError('physics changed while reading startup native snapshot')
 return {'physics_stamp':stamp,'task_counters':[int(task.step_counter),int(task.phase_index),
     int(task.phase_entry_step),int(task.phase_step_counter)],
   'base_task_steps':getattr(task,'steps',None),
   'phase_transition_history':demo._to_jsonable(copy.deepcopy(getattr(task,'phase_transition_history',None))),
   'joint_positions':{name:value.tolist() for name,value in q.items()},
   'body_poses':{path:value.tolist() for path,value in poses.items()}}

def validate_initial_native_joints(task,snapshot):
 import numpy as np
 arm_names=('shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint')
 rows=[];errors=[]
 for name,robot in task.robots.items():
  config=robot.config;desired=dict(getattr(config,'initial_joint_positions',None) or {})
  driver=str(getattr(config,'gripper_dof_name',None) or 'finger_joint')
  names=list(robot.articulation.dof_names);q=np.asarray(snapshot['joint_positions'][name],float).reshape(-1)
  required=set(arm_names)|{driver};missing=sorted(required-set(desired))
  if missing:errors.append(name+': initial configuration lacks required joints '+str(missing))
  if len(names)!=len(q) or len(set(names))!=len(names):errors.append(name+': ambiguous native DOF membership')
  checks=[]
  for requested,goal in desired.items():
   resolved=robot._resolved_articulation_dof_name(str(requested))
   if resolved is None or names.count(resolved)!=1:
    errors.append(name+': configured initial joint not uniquely resolved '+str(requested));continue
   index=names.index(resolved)
   if index>=len(q):errors.append(name+': native q is incomplete');continue
   error=abs(float(q[index])-float(goal))
   checks.append({'requested_joint':str(requested),'resolved_native_joint':resolved,
     'expected_q':float(goal),'actual_q':float(q[index]),'absolute_error_rad':error})
   if not np.isfinite(error) or error>1e-4:errors.append(name+': initial native q mismatch '+str(requested))
  rows.append({'robot':name,'config_robot_name':str(config.name),'native_dof_names':names,
    'configured_initial_joint_positions':desired,'required_joints':sorted(required),'missing_required':missing,'checks':checks,
    'passive_mimic_joints_without_explicit_initial_targets_are_not_required':True})
 report={'passed':not errors,'errors':errors,'physics_stamp':snapshot['physics_stamp'],
   'tolerance_rad':1e-4,'angle_comparison':'direct configured native branch; no modulo or normalization',
   'scope':'read-only check after existing reset; no state setters, hold or physics step','robots':rows}
 demo._write_json(out/'initial_native_joint_preflight.json',report)
 if errors:raise RuntimeError('initial native q differs from reviewed configured values: '+'; '.join(errors))

def native_snapshot_difference(before,after):
 import numpy as np
 same_keys=(set(before['joint_positions'])==set(after['joint_positions'])
    and set(before['body_poses'])==set(after['body_poses']))
 q_equal=same_keys and all(np.array_equal(before['joint_positions'][k],after['joint_positions'][k])
    for k in before['joint_positions'])
 poses_equal=same_keys and all(np.array_equal(before['body_poses'][k],after['body_poses'][k])
    for k in before['body_poses'])
 return {'unchanged':bool(same_keys and q_equal and poses_equal
     and before['physics_stamp']==after['physics_stamp'] and before['task_counters']==after['task_counters']
     and before['base_task_steps']==after['base_task_steps']
     and before['phase_transition_history']==after['phase_transition_history']),
   'same_fullpath_membership':same_keys,'joint_positions_exactly_equal':bool(q_equal),
   'body_poses_exactly_equal':bool(poses_equal),'physics_stamp_equal':before['physics_stamp']==after['physics_stamp'],
   'task_counters_equal':before['task_counters']==after['task_counters'],
   'base_task_steps_equal':before['base_task_steps']==after['base_task_steps'],
   'phase_transition_history_equal':before['phase_transition_history']==after['phase_transition_history']}

def usd_native_body_comparison(stage,native):
 import numpy as np
 from pxr import Gf,Usd,UsdGeom
 cache=UsdGeom.XformCache(Usd.TimeCode.Default());rows=[]
 for path,body_pose in native['body_poses'].items():
  prim=stage.GetPrimAtPath(path)
  if not prim.IsValid():raise RuntimeError('native startup body is missing from USD: '+path)
  raw=cache.GetLocalToWorldTransform(prim)
  # Native poses carry rigid translation/orientation; authored body scale is
  # already included in exported mesh-local points, as in export_geometry.py.
  rigid=Gf.Matrix4d(1);rigid.SetRotate(raw.ExtractRotationQuat());rigid.SetTranslateOnly(raw.ExtractTranslation())
  actual=np.asarray(rigid,float).T
  expected=np.asarray(body_pose,float)
  if actual.shape!=(4,4) or not np.all(np.isfinite(actual)):
   raise RuntimeError('startup USD body transform is invalid: '+path)
  rotation=actual[:3,:3]@expected[:3,:3].T
  angle=float(np.arccos(np.clip((np.trace(rotation)-1)*.5,-1.,1.)))
  position=float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
  rows.append({'body':path,'position_error_m':position,'angle_error_rad':angle,
    'matrix_max_abs_error':float(np.max(abs(actual-expected))),
    'native_world_matrix':expected.tolist(),'usd_world_matrix':actual.tolist(),
    'usd_scaled_world_matrix':np.asarray(raw,float).T.tolist()})
 return {'bodies':len(rows),'max_position_error_m':max(r['position_error_m'] for r in rows),
   'max_angle_error_rad':max(r['angle_error_rad'] for r in rows),
   'max_matrix_abs_error':max(r['matrix_max_abs_error'] for r in rows),'rows':rows}

def publish_native_results_to_usd(task,monitor):
 """Publish existing PhysX results; never simulate or set a robot/object state."""
 import omni.physx
 before=native_startup_snapshot(task,monitor)
 validate_initial_native_joints(task,before)
 usd_before=usd_native_body_comparison(monitor.stage,before)
 report={'method':'SDK PhysX.update_transformations(False,True,False,False)',
   'scope':'existing native PhysX result publication to USD only; no simulate/step or q/body pose/velocity setters',
   'before':before,'usd_before':usd_before,'passed':False,
   'position_tolerance_m':1e-5,'angle_tolerance_rad':1e-4,'matrix_tolerance':1e-5}
 demo._write_json(out/'native_usd_startup_sync.json',report)
 try:
  omni.physx.get_physx_interface().update_transformations(False,True,False,False)
  after=native_startup_snapshot(task,monitor)
  report['after']=after;report['native_difference']=native_snapshot_difference(before,after)
  report['usd_after']=usd_native_body_comparison(monitor.stage,after)
  comparison=report['usd_after']
  report['passed']=bool(report['native_difference']['unchanged']
    and comparison['max_position_error_m']<=report['position_tolerance_m']
    and comparison['max_angle_error_rad']<=report['angle_tolerance_rad']
    and comparison['max_matrix_abs_error']<=report['matrix_tolerance'])
  if not report['passed']:raise RuntimeError('native-to-USD result publication did not align visible rigid bodies without native changes')
 except Exception as exc:
  report['error']=type(exc).__name__+': '+str(exc)
  demo._write_json(out/'native_usd_startup_sync.json',report)
  raise
 demo._write_json(out/'native_usd_startup_sync.json',report)

def startup_camera_param_annotators(task):
 """Attach read-only camera metadata to each already existing RGB product."""
 import omni.replicator.core as rep
 entries=[]
 try:
  for spec in task.config.camera_metadata:
   robot=spec.get('owner') or spec.get('robot');sensor=task.robots[robot].sensors[spec['name']]
   backend=sensor._camera;render_product=backend.rp
   render_path=str(getattr(render_product,'path',render_product))
   annotator=rep.AnnotatorRegistry.get_annotator('camera_params');annotator.attach(render_product)
   entries.append({'spec':spec,'sensor':sensor,'backend':backend,'render_product':render_product,
     'render_product_path':render_path,'annotator':annotator})
 except Exception:
  for entry in entries:entry['annotator'].detach(entry['render_product'])
  raise
 return entries

def camera_render_parameter_comparison(stage,entries):
 import numpy as np
 from pxr import Usd,UsdGeom
 cache=UsdGeom.XformCache(Usd.TimeCode.Default());rows=[]
 for entry in entries:
  sensor_path=str(entry['sensor'].camera_prim_path)
  backend_path=str(entry['backend'].prim.prim_path)
  product=stage.GetPrimAtPath(entry['render_product_path'])
  camera_paths=[str(p) for p in product.GetRelationship('camera').GetTargets()] if product.IsValid() else []
  row={'camera':entry['spec']['video_key'],'sensor_path':sensor_path,'backend_path':backend_path,
    'render_product_path':entry['render_product_path'],'render_product_camera_targets':camera_paths,
    'passed':False}
  try:
   data=entry['annotator'].get_data();row['camera_params']=demo._to_jsonable(data)
   if camera_paths!=[sensor_path] or backend_path!=sensor_path:
    raise RuntimeError('RGB render product is bound to a different camera')
   actual=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(sensor_path)),float).T
   # SDK camera_params supplies a row-vector cameraViewTransform; the repo's
   # point-cloud path also transposes this matrix before using it as world->camera.
   view=np.asarray(data['cameraViewTransform'],float).reshape(4,4).T
   if not np.all(np.isfinite(view)):raise RuntimeError('render camera view matrix is nonfinite')
   expected=np.linalg.inv(actual);error=float(np.max(abs(view-expected)))
   row.update(usd_camera_world_matrix=actual.tolist(),render_world_to_camera_matrix=view.tolist(),
     expected_world_to_camera_matrix=expected.tolist(),matrix_max_abs_error=error)
   if error>1e-5:raise RuntimeError('RTX render camera view differs from authored USD camera')
   row['passed']=True
  except Exception as exc:row['error']=type(exc).__name__+': '+str(exc)
  rows.append(row)
 return {'passed':len(rows)==3 and all(row['passed'] for row in rows),'rows':rows,
   'matrix_tolerance':1e-5,'scope':'same RGB render products camera relationships and renderer cameraViewTransform; query only, no new render or physics step'}

def review_initial_images(task,monitor):
 """Render at fixed native time, then wait for this run's image review."""
 import cv2
 before=native_startup_snapshot(task,monitor);before_stamp=before['physics_stamp'];before_counters=before['task_counters']
 entries=startup_camera_param_annotators(task)
 try:
  observations=demo._warm_up_runtime_cameras(runtime_env)
  parameters=camera_render_parameter_comparison(monitor.stage,entries)
 finally:
  for entry in entries:entry['annotator'].detach(entry['render_product'])
 after=native_startup_snapshot(task,monitor);difference=native_snapshot_difference(before,after)
 usd_after=usd_native_body_comparison(monitor.stage,after)
 diagnostics={'before':before,'after':after,'native_difference':difference,'usd_after_render':usd_after,
   'render_camera_parameters':parameters,'passed':False}
 demo._write_json(out/'startup_camera_render_diagnostics.json',diagnostics)
 if not difference['unchanged']:raise RuntimeError('startup render-only check changed native q/body pose/physics time/task counters')
 obs=observations[0]
 rows=[]
 for spec in task.config.camera_metadata:
  robot=spec.get('owner') or spec.get('robot')
  sensor=((obs.get(robot) or {}).get('sensors') or {}).get(spec['name'],{})
  rgb=demo.LiveRolloutVideoRecorder._to_uint8_rgb(sensor.get('rgba'))
  if rgb is None:raise RuntimeError('startup camera has no rendered image: '+spec['name'])
  path=out/('startup_'+spec['video_key'].replace('.','_')+'.png')
  if not cv2.imwrite(str(path),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):
   raise RuntimeError('could not preserve startup image')
  rows.append({'camera':spec['video_key'],'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
    'sensor_reference_time':sensor.get('render_reference_time',sensor.get('reference_time'))})
 if len(rows)!=3:raise RuntimeError('startup review requires all original three camera views')
 diagnostics['passed']=bool(parameters['passed'] and usd_after['max_position_error_m']<=1e-5
   and usd_after['max_angle_error_rad']<=1e-4 and usd_after['max_matrix_abs_error']<=1e-5)
 demo._write_json(out/'startup_camera_render_diagnostics.json',diagnostics)
 if not diagnostics['passed']:raise RuntimeError('native/USD/RTX startup rendering diagnostic did not pass')
 stamp=task._beam_physics_stamp()
 demo._write_json(out/'startup_images_ready.json',{'physics_stamp':stamp,'before_render_physics_stamp':before_stamp,
   'task_counters_before_and_after':list(before_counters),'images':rows,
   'native_usd_sync_report':'native_usd_startup_sync.json',
   'render_camera_diagnostics':'startup_camera_render_diagnostics.json',
   'scope':'native initialized; fresh rendering only, no policy action or added physics step; actual image review still required'})
 deadline=time.monotonic()+300.
 ack=out/'startup_image_review.json'
 while not ack.exists():
  if (out/'stop_request.json').exists():raise RuntimeError('startup image review stopped before first action')
  if time.monotonic()>deadline:raise RuntimeError('startup image review timeout before first action')
  time.sleep(.25)
 approved=json.loads(ack.read_text())
 if approved.get('accepted') is not True or approved.get('image_sha256')!={r['camera']:r['sha256'] for r in rows}:
  raise RuntimeError('startup image review rejected or references different images')
 if not native_snapshot_difference(after,native_startup_snapshot(task,monitor))['unchanged']:
  raise RuntimeError('native q/body pose/physics/task counters changed while awaiting startup image review')

original_act=demo.DualFrankaAssemblyDemoPolicy.act
def act(self,task,*args,**kwargs):
 if not getattr(task,'_assisted_initial_preflight_done',False):
  from toolkits.factory_dual_franka_assembly.beam_assisted_preflight import assert_assisted_configuration
  runtime_config=assert_assisted_configuration(task)
  validate_runtime_configuration_binding(task,runtime_config)
  from toolkits.constraint_checking.integration.entity_geometry import TaskEntityGeometryMonitor
  monitor=TaskEntityGeometryMonitor(task)
  publish_native_results_to_usd(task,monitor)
  poses=monitor.current_poses()
  result=monitor.checker.check(poses,step=int(task.step_counter))
  report={'route':'assisted','scope':'native initial colliders; supplementary base and full sequential54 visible-mesh preflights retained at route root; actual full-path visible-mesh monitor required for every recorded state',
   'colliders':len(monitor.checker.colliders),'poses':len(poses),'coverage':monitor.checker.coverage(),
   'gaps':monitor.gaps,'initial_geometry':result,'formal_physical_success':False}
  demo._write_json(out/'initial_entity_preflight.json',report)
  assert len(monitor.checker.colliders)>=48 and len(poses)>=44
  from export_geometry import export_geometry
  geometry=export_geometry(monitor,out)
  from sweep_model import SceneModel
  from visual_mesh_check import ProductionCarryChecker
  carry_checker=ProductionCarryChecker(SceneModel(out/'scene_geometry.json'))
  def production_cad_guard(**command):
   stamp=task._beam_physics_stamp()
   if stamp != command['physics_stamp']:
    raise ValueError('production CAD guard received a stale command clock')
   q={name:np.asarray(robot.articulation.get_joint_positions(),float).copy()
      for name,robot in task.robots.items()}
   native=monitor.current_poses()
   empty=command.pop('empty',False)
   if empty:
    if not task.get_current_phase_spec().get('beam_assisted_empty_return_guard'):
     raise ValueError('empty CAD guard requested outside an empty-return phase')
    allowed,detail=carry_checker.check_empty(native_poses=native,actual_q=q,
     other_targets=getattr(task,'_assisted_batch_arm_candidates',{}),
     phase=task.phase,**command)
   else:
    allowed,detail=carry_checker.check(native_poses=native,actual_q=q,
     other_targets=getattr(task,'_assisted_batch_arm_candidates',{}),
     support=task.get_current_phase_spec().get('beam_assisted_visual_support'),
     phase=task.phase,**command)
   if carry_checker.calls % 64 == 0 or not allowed:
    demo._write_json(out/'production_cad_progress.json',dict(calls=carry_checker.calls,
     samples=carry_checker.samples,last_step=int(task.step_counter),phase=task.phase,
     cad_wall_seconds=carry_checker.wall_seconds,
     mean_seconds_per_guard_call=carry_checker.wall_seconds/max(1,carry_checker.calls),
     last_check=detail))
   if stamp != task._beam_physics_stamp():
    raise ValueError('native clock changed during final production CAD sweep')
   return allowed,detail
  task._assisted_production_cad_guard=production_cad_guard
  task._assisted_geometry_snapshot={k:geometry[k] for k in ['body_rest','joints','robots']}
  task._assisted_geometry_snapshot['visible_meshes']=[m for m in geometry['visible_meshes']
    if str(m['object']).startswith('fabrica_beam_') or m['body'].endswith(('/left_inner_finger','/right_inner_finger'))]
  task._assisted_native_body_readers=dict(monitor._native_readers)
  from toolkits.factory_dual_franka_assembly.beam_assisted_camera import install_native_wrist_cameras
  install_native_wrist_cameras(task,out)
  from camera_visibility_preflight import CameraAudit
  import numpy as np
  visibility=CameraAudit(geometry,9,2,.01)
  native_poses={k:np.asarray(v,float) for k,v in poses.items()}
  cameras=[]
  for camera in task.config.camera_metadata:
   if camera.get('target_source')!='native_fingertip_midpoint':continue
   robot=camera.get('robot',camera['owner'])
   cameras.append({'name':camera['name'],'robot':robot,
     **visibility.audit(native_poses,robot,np.asarray(camera['native_mount_offset'],float))})
  passed=len(cameras)==2 and all(c['pinch_visibility_pass'] for c in cameras)
  demo._write_json(out/'camera_runtime_visibility.json',{'passed':passed,'physics_stamp':task._beam_physics_stamp(),
    'scope':'actual initial native full-path poses and all exported visible triangles; full grippers included; native image still required',
    'cameras':cameras})
  if not passed:raise RuntimeError('native initial camera visibility preflight failed')
  demo._write_json(out/'runtime_initialization_preflight.json',{'passed':True,'configuration':runtime_config,
    'camera_registration_and_native_matrix':'camera_native_initialization.json',
    'full_gripper_visibility':'camera_runtime_visibility.json','colliders':len(monitor.checker.colliders),
    'native_poses':len(poses),'formal_physical_success':False})
  review_initial_images(task,monitor)
  task._assisted_initial_preflight_done=True
 request=out/'stop_request.json'
 if request.exists():
  detail=json.loads(request.read_text())
  task._set_terminal_state('failed',reason='assisted-external-evidence-stop',status='failed',detail=detail)
  return {}
 stop=os.environ.get('BEAM_ASSISTED_STOP_AFTER_PHASE','')
 if stop:
  names=[p['name'] for p in task.phase_specs]
  if stop not in names:raise ValueError('unknown assisted stop phase: '+stop)
  if task.phase_index>names.index(stop):
   task._beam_assisted_boundary_stop={'completed_phase':stop,'next_phase':task.phase,
     'step':int(task.step_counter),'physics_stamp':task._beam_physics_stamp(),
     'success':False,'reason':'authorized_bounded_validation'}
   demo._write_json(out/'bounded_stop.json',task._beam_assisted_boundary_stop)
   return {}
 published_before=copy.deepcopy(getattr(task,'_assisted_published_q',{}))
 stamps_before=copy.deepcopy(getattr(task,'_assisted_published_q_stamps',{}))
 cache_before=copy.deepcopy(getattr(task,'_ur5e_plumbers_last_arm_command_q',{}))
 try:
  return original_act(self,task,*args,**kwargs)
 except ValueError as exc:
  phase=task.get_current_phase_spec()
  if not (phase.get('local_skill',{}).get('beam_assisted_production_cad_guard')
          or phase.get('beam_assisted_empty_return_guard')):
   raise
  # No action was returned to the SDK. Preserve the last accepted batch and
  # enter the existing terminal/tail recorder instead of crashing its loop.
  task._assisted_published_q=published_before
  task._assisted_published_q_stamps=stamps_before
  task._ur5e_plumbers_last_arm_command_q=cache_before
  detail=dict(error=type(exc).__name__+': '+str(exc),phase=task.phase,
   step=int(task.step_counter),physics_stamp=task._beam_physics_stamp(),
   last_published_q=published_before,no_new_sdk_batch=True)
  demo._write_json(out/'production_control_failure.json',detail)
  task._set_terminal_state('failed',reason='assisted-production-control-stop',
    status='failed',detail=detail)
  return {}
demo.DualFrankaAssemblyDemoPolicy.act=act
demo.main()
