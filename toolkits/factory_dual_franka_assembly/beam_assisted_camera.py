"""Virtual wrist views follow native rigid poses, independent of USD link caches."""
from pathlib import Path
import json
import os
import tempfile
import numpy as np


def usd_camera_world_matrix(eye, target, up_hint):
    eye=np.asarray(eye,float);up_hint=np.asarray(up_hint,float);forward=np.asarray(target,float)-eye
    if eye.shape!=(3,) or up_hint.shape!=(3,) or not np.all(np.isfinite(eye)) or not np.all(np.isfinite(up_hint)) or np.linalg.norm(up_hint)<1e-8:
        raise ValueError('invalid native camera eye or up direction')
    if not np.all(np.isfinite(forward)) or np.linalg.norm(forward)<1e-8:
        raise ValueError('invalid native camera target')
    forward/=np.linalg.norm(forward);right=np.cross(forward,up_hint)
    if np.linalg.norm(right)<1e-6:
        right=np.cross(forward,np.eye(3)[int(np.argmin(abs(forward)))])
    right/=np.linalg.norm(right);up=np.cross(right,forward)
    result=np.eye(4);result[:3,:3]=np.column_stack([right,up,-forward]);result[:3,3]=eye
    validate_camera_matrix(result)
    return result


def validate_camera_matrix(matrix):
    matrix=np.asarray(matrix,float)
    if matrix.shape!=(4,4) or not np.all(np.isfinite(matrix)):
        raise ValueError('camera matrix is not finite 4x4')
    rotation=matrix[:3,:3]
    if not np.allclose(matrix[3],[0,0,0,1],rtol=0,atol=1e-8) or not np.allclose(rotation.T@rotation,np.eye(3),rtol=0,atol=1e-6) or abs(np.linalg.det(rotation)-1)>1e-6:
        raise ValueError('camera matrix is not a rigid right-handed transform')
    return matrix


def roll_camera_world_matrix_180(matrix):
    """Turn the image upside down around the optical axis, without mirroring.

    Roll the Camera prim itself so all render outputs and its measured
    extrinsics share the same view. The eye and USD -Z viewing ray stay fixed.
    """
    result=validate_camera_matrix(matrix).copy()
    result[:3,:2]*=-1
    return validate_camera_matrix(result)


def validate_virtual_camera_path(actual_path, sensor_path, requested_path, robot_root, native_paths, valid, prim_type, is_physics):
    """Resolve only the already registered sensor Camera in this exact episode."""
    if '/robots/' not in robot_root:
        raise RuntimeError('camera robot root has no unambiguous episode scope')
    episode_root=robot_root.split('/robots/',1)[0]
    basename=str(requested_path).rsplit('/',1)[-1]
    expected=episode_root+'/cameras/'+basename
    if not basename or actual_path!=expected or sensor_path!=actual_path:
        raise RuntimeError('camera sensor/backend path is not its episode virtual camera')
    if actual_path in native_paths or is_physics or not valid or prim_type!='Camera':
        raise RuntimeError('camera backend must resolve an existing non-physics Camera')
    return actual_path


def pose_matrix(position, quaternion):
    from .planner_primitives import quat_rotate
    position=np.asarray(position,float);quaternion=np.asarray(quaternion,float).copy()
    if position.shape!=(3,) or quaternion.shape!=(4,) or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)) or abs(np.linalg.norm(quaternion)-1)>1e-3:
        raise ValueError('native camera body pose is invalid')
    quaternion/=np.linalg.norm(quaternion)
    result=np.eye(4);result[:3,3]=position
    result[:3,:3]=np.column_stack([quat_rotate(quaternion,a) for a in np.eye(3)])
    validate_camera_matrix(result)
    return result


class NativeWristCameras:
    def __init__(self, task, output):
        from pxr import UsdGeom,UsdPhysics,Gf
        from isaacsim.core.utils.stage import get_current_stage
        self.task=task;self.output=Path(output);self.stage=get_current_stage()
        self.entries=[];self.last_clock=None;self.calls=0;self.closed=False;self.callback_id=None
        data=task._assisted_geometry_snapshot
        for spec in task.config.camera_metadata:
            if spec.get('target_source')!='native_fingertip_midpoint':continue
            robot=spec.get('robot',spec['owner']);root=data['robots'][robot]['root_path']
            wrist=root+'/wrist_3_link';pads=[]
            for finger in ['left_inner_finger','right_inner_finger']:
                body=root+'/Gripper/Robotiq_2F_85/'+finger
                meshes=[m for m in data['visible_meshes'] if m['body']==body and 'fingertipsstep' in m['prim']]
                if len(meshes)!=1:raise RuntimeError('native camera needs one unique visible pad mesh per finger')
                pads.append((body,np.asarray(meshes[0]['points'],float).mean(0)))
            sensor=task.robots[robot].sensors[spec['name']]
            sensor_path=str(getattr(sensor,'camera_prim_path',spec['prim_path']));actual_path=sensor_path
            backend=getattr(sensor,'_camera',None)
            backend_prim=getattr(backend,'prim',None)
            actual_path=str(getattr(backend_prim,'prim_path',actual_path))
            prim=self.stage.GetPrimAtPath(actual_path)
            registration=dict(name=spec['name'],requested_path=spec['prim_path'],sensor_path=sensor_path,backend_path=actual_path,
                valid=bool(prim.IsValid()),prim_type=str(prim.GetTypeName()) if prim.IsValid() else None)
            with (self.output/'camera_native_registration.jsonl').open('a') as f:f.write(json.dumps(registration)+'\n')
            validate_virtual_camera_path(actual_path,sensor_path,spec['prim_path'],root,
                task._assisted_native_body_readers,bool(prim.IsValid()),str(prim.GetTypeName()) if prim.IsValid() else None,
                bool(prim.IsValid() and (prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI))))
            camera=UsdGeom.Camera(prim)
            camera.CreateFocalLengthAttr().Set(float(spec.get('focal_length',12.)))
            aperture=float(spec.get('horizontal_aperture',20.955));width,height=spec.get('resolution',[640,480])
            camera.CreateHorizontalApertureAttr().Set(aperture)
            camera.CreateVerticalApertureAttr().Set(float(spec.get('vertical_aperture') or aperture*height/width))
            camera.CreateClippingRangeAttr().Set(Gf.Vec2f(*map(float,spec.get('clipping_range',[.01,10.]))))
            transform=UsdGeom.Xformable(prim);before=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim),float).T.tolist()
            op=transform.MakeMatrixXform();transform.SetResetXformStack(True)
            self.entries.append(dict(name=spec['name'],robot=robot,path=actual_path,wrist=wrist,pads=pads,
                offset=np.asarray(spec['native_mount_offset'],float),op=op,before=before))
        if len(self.entries)!=2:raise RuntimeError('assisted route requires two native wrist cameras')

    def native_pose(self, path):
        reader=self.task._assisted_native_body_readers.get(path)
        if reader is None or str(reader.unwrap().prim_path)!=path:raise RuntimeError('camera native body mapping mismatch: '+path)
        return pose_matrix(*reader.get_pose())

    def update(self, strict=False):
        if self.closed:return
        from pxr import Gf,UsdGeom
        stamp={'valid':False,'source':'camera physics stamp unavailable'}
        records=[]
        try:
            stamp=self.task._beam_physics_stamp();clock=(stamp.get('epoch'),stamp.get('index'))
            if not stamp.get('valid'):raise RuntimeError('native camera physics clock missing')
            for entry in self.entries:
                wrist=self.native_pose(entry['wrist']);eye=wrist[:3,:3]@entry['offset']+wrist[:3,3]
                points=[]
                for body,local in entry['pads']:
                    T=self.native_pose(body);points.append(T[:3,:3]@local+T[:3,3])
                target=np.mean(points,axis=0);W=roll_camera_world_matrix_180(
                    usd_camera_world_matrix(eye,target,wrist[:3,2]))
                entry['op'].Set(Gf.Matrix4d(*map(float,W.T.reshape(-1))))
                prim=self.stage.GetPrimAtPath(entry['path']);actual=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim),float).T
                validate_camera_matrix(actual)
                error=float(np.max(abs(actual-W)))
                if error>1e-8:raise RuntimeError('authored wrist camera world transform mismatch')
                records.append(dict(name=entry['name'],path=entry['path'],wrist=entry['wrist'],
                    physics_stamp=stamp,eye=eye.tolist(),target=target.tolist(),world_matrix=actual.tolist(),
                    optical_axis_roll_degrees=180,
                    usd_forward_dot=float(np.dot(-actual[:3,2],(target-eye)/np.linalg.norm(target-eye))),
                    actual_world_matrix_error=error))
        except Exception as exc:
            if strict:raise
            with (self.output/'camera_native_update_errors.jsonl').open('a') as f:
                f.write(json.dumps(dict(physics_stamp=stamp,error=type(exc).__name__+': '+str(exc)))+'\n')
            self.request_stop(stamp,exc)
            return
        if self.calls==0:
            (self.output/'camera_native_initialization.json').write_text(json.dumps(dict(
                method='absolute USD camera world matrices from native wrist and visible pad centers; 180 degree optical-axis roll applied to both Camera prims; SDK POST_PHYSICS_STEP pre_step=False callback; no robot or part writes, steps or renders',
                original_camera_world_matrices={e['name']:e['before'] for e in self.entries},cameras=records),indent=2)+'\n')
        if self.calls%8==0:
            with (self.output/'camera_native_pose_trace.jsonl').open('a') as f:
                f.write(json.dumps(dict(physics_stamp=stamp,update_phase='initialization' if strict else 'POST_PHYSICS_STEP',calls=self.calls,cameras=records))+'\n')
        self.last_clock=clock;self.calls+=1

    def request_stop(self,stamp,error):
        detail=dict(reason='native wrist camera update failed during recording',
            physics_stamp=stamp,error=type(error).__name__+': '+str(error),source='native_wrist_camera')
        # Publish a complete JSON atomically without replacing an earlier guard.
        # The policy and an independent CAD watcher can read this concurrently.
        with tempfile.NamedTemporaryFile(mode='w',prefix='.camera_stop_',dir=str(self.output),delete=False) as f:
            temporary=Path(f.name);f.write(json.dumps(detail,indent=2)+'\n')
        try:
            try:os.link(str(temporary),str(self.output/'stop_request.json'))
            except FileExistsError:pass
        finally:
            temporary.unlink()

    def close(self):
        """Must run after failure-tail recording and before native handles close."""
        if self.closed:return
        self.closed=True
        result=dict(calls=self.calls,last_clock=self.last_clock,callback_id=self.callback_id,closed=True)
        if self.callback_id is not None:
            try:
                from isaacsim.core.simulation_manager import SimulationManager
                result['deregister_result']=SimulationManager.deregister_callback(self.callback_id)
            except Exception as exc:
                result['deregister_error']=type(exc).__name__+': '+str(exc)
        (self.output/'camera_native_closed.json').write_text(json.dumps(result,indent=2)+'\n')


def uninstall_native_wrist_cameras(task):
    cameras=getattr(task,'_assisted_native_wrist_cameras',None)
    if cameras is not None:cameras.close()


def install_native_wrist_cameras(task, output):
    from isaacsim.core.simulation_manager import IsaacEvents,SimulationManager
    cameras=NativeWristCameras(task,output);cameras.update(strict=True)
    def after_physics(step_dt):
        cameras.update()
    cameras.callback_id=SimulationManager.register_callback(after_physics,
        event=IsaacEvents.POST_PHYSICS_STEP,order=100,name='beam_assisted_native_wrist_cameras')
    task._assisted_native_wrist_cameras=cameras
