"""Validate an opt-in arm-drive experiment against the native readbacks.

This module imports no simulator and changes only position/velocity gains. The
caller supplies an existing articulation and a read-only joint-schema reader.
Native effort limits and force/acceleration drive types are never written.
"""
from __future__ import annotations

import numpy as np


DEFAULT_ARM_KP = 80000.0
DEFAULT_ARM_KD = 4000.0


class ArmDriveReadbackError(RuntimeError):
    def __init__(self, audit):
        self.audit = audit
        super().__init__('Arm drive experiment could not be verified: ' + str(audit['errors']))


def positive_gain(value, name):
    if isinstance(value, (bool, np.bool_)) or np.ndim(value) != 0:
        raise ValueError(f'{name} must be a finite positive scalar')
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite positive scalar') from exc
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be a finite positive scalar')
    return value


def _native_vector(value, count, name):
    # Isaac's native view returns (1, N); wrappers may return (N,). Do not
    # flatten other shapes, which could hide duplicate articulations or axes.
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    result = np.array(value, dtype=float, copy=True)
    if result.shape == (1, count):
        result = result[0].copy()
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain one finite native value per arm joint')
    return result


def configure_arm_drive(articulation, joint_names, kp, kd, drive_metadata_reader):
    """Set gains only, record all readbacks, and reject unverified trials.

    Historical default gains retain the previous error-tolerant behavior.
    Any nondefault gain is an explicit experiment and requires verified gains,
    positive native effort limits and actual angular DriveAPI type readbacks.
    Invalid requested values always fail before accessing the articulation.
    """
    kp, kd = positive_gain(kp, 'arm_drive_kp'), positive_gain(kd, 'arm_drive_kd')
    joint_names = tuple(joint_names)
    if len(joint_names) != 6 or len(set(joint_names)) != 6:
        raise ValueError('Arm gain configuration requires six unique UR5e arm joints')
    strict = kp != DEFAULT_ARM_KP or kd != DEFAULT_ARM_KD
    audit = {'configured': False, 'strict_request': strict,
             'kp_requested': kp, 'kd_requested': kd,
             'joint_names': list(joint_names), 'errors': {},
             'gain_source': 'native_articulation_view',
             'effort_source': 'native_articulation_view',
             'changed_properties': ['stiffness', 'damping']}
    try:
        raw_indices = [articulation.get_dof_index(name) for name in joint_names]
        if (any(isinstance(index, (bool, np.bool_)) or not isinstance(index, (int, np.integer))
                or index < 0 for index in raw_indices) or len(set(raw_indices)) != 6):
            raise ValueError('Arm DOF indices must be six distinct nonnegative integers')
        indices = np.array(raw_indices, dtype=np.int64)
        audit['joint_indices'] = indices.tolist()
        articulation.set_gains(kps=np.full(6, kp, dtype=float),
                               kds=np.full(6, kd, dtype=float), joint_indices=indices)
        audit['setter_completed'] = True
    except Exception as exc:
        audit['errors']['set_gains'] = f'{type(exc).__name__}: {exc}'
        if strict:
            raise ArmDriveReadbackError(audit) from exc
        return audit
    try:
        view = articulation.unwrap()._articulation_view
        audit['native_view_type'] = type(view).__name__
        audit['physics_handle_valid'] = None
        audit['physics_handle_source'] = 'native_articulation_view.is_physics_handle_valid'
        handle_readback = getattr(view, 'is_physics_handle_valid', None)
        if not callable(handle_readback):
            audit['physics_handle_valid'] = None
            raise ValueError('Native arm view has no physical-handle validity getter')
        valid = handle_readback()
        audit['physics_handle_valid'] = bool(valid) if isinstance(valid, (bool, np.bool_)) else None
        if audit['physics_handle_valid'] is not True:
            raise ValueError('Native arm view does not have a verified live physical handle')
    except Exception as exc:
        audit['errors']['native_view'] = f'{type(exc).__name__}: {exc}'
        if strict:
            raise ArmDriveReadbackError(audit) from exc
        return audit
    try:
        # Detach stiffness before reading another buffer, including damping.
        actual_kp, actual_kd = view.get_gains(joint_indices=indices)
        actual_kp = _native_vector(actual_kp, 6, 'native kp')
        actual_kd = _native_vector(actual_kd, 6, 'native kd')
        audit['kp_readback'], audit['kd_readback'] = actual_kp.tolist(), actual_kd.tolist()
        audit['gains_match'] = bool(np.allclose(actual_kp, kp, rtol=1e-6, atol=1e-8)
                                   and np.allclose(actual_kd, kd, rtol=1e-6, atol=1e-8))
        if not audit['gains_match']:
            raise ValueError('Native arm gains do not match the requested experiment')
    except Exception as exc:
        audit['errors']['gains'] = f'{type(exc).__name__}: {exc}'
    try:
        efforts = _native_vector(view.get_max_efforts(joint_indices=indices), 6, 'native max efforts')
        audit['max_effort_readback'] = efforts.tolist()
        if np.any(efforts <= 0):
            raise ValueError('Native arm effort limits must be positive')
    except Exception as exc:
        audit['errors']['max_efforts'] = f'{type(exc).__name__}: {exc}'
    try:
        drive_types = drive_metadata_reader(joint_names, indices)
        audit['drive_types'] = drive_types
        if len(drive_types) != 6:
            raise ValueError('Expected one actual angular DriveAPI readback per arm joint')
        for name, index, metadata in zip(joint_names, indices, drive_types):
            if (metadata.get('joint_name') != name or metadata.get('dof_index') != int(index)
                    or not metadata.get('valid') or not metadata.get('has_angular_drive_api')
                    or not metadata.get('joint_prim_path') or not metadata.get('source')
                    or metadata.get('type') not in ('force', 'acceleration')):
                raise ValueError(f'Actual angular DriveAPI type unavailable for {name}: {metadata}')
    except Exception as exc:
        audit['errors']['drive_types'] = f'{type(exc).__name__}: {exc}'
    audit['configured'] = not audit['errors']
    if strict and not audit['configured']:
        raise ArmDriveReadbackError(audit)
    return audit
