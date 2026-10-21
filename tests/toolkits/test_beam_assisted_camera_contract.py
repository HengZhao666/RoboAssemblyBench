"""Camera scope, readable geometry and shutdown contracts without Isaac startup."""
import json
import sys
import types
import numpy as np
import pytest

from toolkits.factory_dual_franka_assembly.beam_assisted_camera import (
    NativeWristCameras,usd_camera_world_matrix,validate_camera_matrix,
    validate_virtual_camera_path,roll_camera_world_matrix_180,
)


ROOT='/World/env_0/episode_000000'
CAM=ROOT+'/cameras/assisted_left_wrist'
ROBOT=ROOT+'/robots/ur5e_left'


def path_check(**changes):
    values=dict(actual_path=CAM,sensor_path=CAM,
        requested_path='/World/env_0/cameras/assisted_left_wrist',
        robot_root=ROBOT,native_paths={ROBOT+'/wrist_3_link'},
        valid=True,prim_type='Camera',is_physics=False)
    values.update(changes)
    return validate_virtual_camera_path(**values)


def test_existing_registered_camera_is_accepted_after_episode_expansion():
    assert path_check()==CAM


@pytest.mark.parametrize('change',[
    {'actual_path':'/World/env_0/cameras/assisted_left_wrist'},
    {'sensor_path':ROOT+'/cameras/different_camera'},
    {'actual_path':CAM.replace('episode_000000','episode_000001')},
    {'actual_path':ROBOT+'/Gripper/Robotiq_2F_85/base_link'},
    {'valid':False},{'prim_type':'Mesh'},{'is_physics':True},
    {'native_paths':{CAM}},{'robot_root':'/World/unscoped_robot'},
])
def test_camera_path_refuses_wrong_episode_entity_and_unregistered_prim(change):
    with pytest.raises(RuntimeError):path_check(**change)


@pytest.mark.parametrize('bad',[
    np.full((4,4),np.nan),np.full((4,4),np.inf),np.eye(3),
    np.diag([1.,1.,-1.,1.]),np.diag([1.,2.,1.,1.]),
])
def test_camera_readback_cannot_accept_nan_reflection_scale_or_wrong_shape(bad):
    with pytest.raises(ValueError):validate_camera_matrix(bad)


@pytest.mark.parametrize('eye,up',[
    ([np.nan,0,0],[0,0,1]),([0,0,0],[np.nan,0,1]),
    ([0,0,0],[0,0,0]),
])
def test_camera_basis_rejects_invalid_eye_and_up(eye,up):
    with pytest.raises(ValueError):usd_camera_world_matrix(eye,[1,0,0],up)


def test_valid_collinear_up_uses_rigid_fallback():
    matrix=usd_camera_world_matrix([0,0,0],[0,0,1],[0,0,1])
    assert np.allclose(-matrix[:3,2],[0,0,1])
    assert np.linalg.det(matrix[:3,:3])==pytest.approx(1)


@pytest.mark.parametrize('eye,target,up',[
    ([.2,-.4,1.3],[.5,.1,.8],[0,0,1]),
    ([-.3,.6,.9],[.1,-.2,1.1],[.7,.2,-.4]),
    ([0,0,0],[0,0,1],[0,0,1]),
])
def test_wrist_roll_preserves_eye_and_optical_ray_without_reflection(eye,target,up):
    original=usd_camera_world_matrix(eye,target,up);before=original.copy()
    rolled=roll_camera_world_matrix_180(original)
    assert np.array_equal(original,before)
    assert np.array_equal(rolled[:3,3],original[:3,3])
    assert np.array_equal(rolled[:3,2],original[:3,2])
    assert np.allclose(rolled[:3,:3].T@rolled[:3,:3],np.eye(3))
    assert np.linalg.det(rolled[:3,:3])==pytest.approx(1)
    assert np.array_equal(roll_camera_world_matrix_180(rolled),original)


def test_wrist_roll_reverses_both_image_coordinates_and_preserves_depth():
    original=usd_camera_world_matrix([.2,-.4,1.3],[.5,.1,.8],[0,0,1])
    rolled=roll_camera_world_matrix_180(original)
    camera_points=np.array([[.03,.08,-.3,1],[-.07,.02,-.8,1],[.02,-.06,-.5,1]])
    world_points=(original@camera_points.T).T
    after=(np.linalg.inv(rolled)@world_points.T).T
    # Principal-point-centred pinhole coordinates: a 180 degree roll
    # reverses both axes; a vertical-only flip would instead be a mirror.
    before_image=camera_points[:,:2]/-camera_points[:,2,None]
    after_image=after[:,:2]/-after[:,2,None]
    assert np.allclose(after_image,-before_image,rtol=0,atol=1e-14)
    assert np.allclose(after[:,2],camera_points[:,2],rtol=0,atol=1e-14)


@pytest.mark.parametrize('bad',[
    np.full((4,4),np.nan),np.eye(3),np.diag([1.,1.,-1.,1.]),
    np.diag([1.,2.,1.,1.]),
])
def test_wrist_roll_rejects_invalid_or_reflected_camera_transform(bad):
    with pytest.raises(ValueError):roll_camera_world_matrix_180(bad)


def bare_camera(tmp_path):
    camera=NativeWristCameras.__new__(NativeWristCameras)
    camera.output=tmp_path;camera.closed=False;camera.callback_id=123
    camera.calls=19;camera.last_clock=(0,42)
    return camera


def test_recording_error_writes_stop_without_overwriting_first_failure(tmp_path):
    camera=bare_camera(tmp_path)
    camera.request_stop({'valid':True,'index':42},RuntimeError('lost native pose'))
    first=(tmp_path/'stop_request.json').read_bytes()
    assert json.loads(first)['source']=='native_wrist_camera'
    camera.request_stop({'valid':True,'index':43},RuntimeError('second error'))
    assert (tmp_path/'stop_request.json').read_bytes()==first


def test_cad_stop_remains_authoritative_if_camera_error_follows(tmp_path):
    camera=bare_camera(tmp_path);first=b'{"reason":"original CAD stop"}\n'
    (tmp_path/'stop_request.json').write_bytes(first)
    camera.request_stop({'valid':True,'index':42},RuntimeError('lost native pose'))
    assert (tmp_path/'stop_request.json').read_bytes()==first


def test_invalid_recording_clock_is_a_logged_stop(tmp_path,monkeypatch):
    camera=bare_camera(tmp_path);camera.entries=[]
    camera.task=types.SimpleNamespace(_beam_physics_stamp=lambda:{'valid':False})
    monkeypatch.setitem(sys.modules,'pxr',types.SimpleNamespace(Gf=object(),UsdGeom=object()))
    camera.update()
    assert json.loads((tmp_path/'stop_request.json').read_text())['source']=='native_wrist_camera'
    assert 'clock missing' in json.loads((tmp_path/'camera_native_update_errors.jsonl').read_text())['error']


def test_recording_clock_backend_exception_is_a_logged_stop(tmp_path,monkeypatch):
    camera=bare_camera(tmp_path);camera.entries=[]
    def unavailable():raise RuntimeError('clock backend unavailable')
    camera.task=types.SimpleNamespace(_beam_physics_stamp=unavailable)
    monkeypatch.setitem(sys.modules,'pxr',types.SimpleNamespace(Gf=object(),UsdGeom=object()))
    camera.update()
    assert 'clock backend unavailable' in json.loads((tmp_path/'stop_request.json').read_text())['error']


def test_close_deactivates_callback_before_native_handle_teardown(tmp_path,monkeypatch):
    camera=bare_camera(tmp_path);calls=[]
    def deregister(callback_id):
        assert camera.closed
        calls.append(callback_id)
        return True
    module=types.SimpleNamespace(SimulationManager=types.SimpleNamespace(deregister_callback=deregister))
    monkeypatch.setitem(sys.modules,'isaacsim.core.simulation_manager',module)
    camera.close();camera.close()
    assert calls==[123]
    assert json.loads((tmp_path/'camera_native_closed.json').read_text())['deregister_result'] is True
    # No pxr import or native transform read is allowed once handles may be gone.
    camera.task=types.SimpleNamespace(_beam_physics_stamp=lambda:pytest.fail('read after close'))
    camera.update()
    assert not (tmp_path/'stop_request.json').exists()
