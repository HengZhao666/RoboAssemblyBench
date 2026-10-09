"""Pure helpers for preserving native idle arm targets without wrapping joints."""
import numpy as np


def native_joint_vector(values):
    if values is None:
        return None
    try:
        result = np.array(values, dtype=float, copy=True).reshape(-1)
    except (ValueError, TypeError):
        return None
    return result if result.size and np.all(np.isfinite(result)) else None


def published_arm_target(command, arm_indices):
    """Extract only complete arm positions from an already published command."""
    if command is None or arm_indices is None:
        return None
    positions = command.get('joint_positions') if isinstance(command, dict) else getattr(command, 'joint_positions', None)
    indices = command.get('joint_indices') if isinstance(command, dict) else getattr(command, 'joint_indices', None)
    if positions is None:
        return None
    try:
        positions = np.asarray(positions, dtype=object).reshape(-1)
        raw_required = np.asarray(arm_indices, dtype=float).reshape(-1)
        if not np.all(np.isfinite(raw_required)) or np.any(raw_required != np.floor(raw_required)):
            return None
        required = raw_required.astype(int)
        if not len(required) or len(set(required)) != len(required) or np.any(required < 0):
            return None
        if indices is None:
            if required.max() >= len(positions):
                return None
            selected = positions[required]
        else:
            raw_indices = np.asarray(indices, dtype=float).reshape(-1)
            if not np.all(np.isfinite(raw_indices)) or np.any(raw_indices != np.floor(raw_indices)):
                return None
            indices = raw_indices.astype(int)
            if len(indices) != len(positions) or len(set(indices)) != len(indices):
                return None
            mapping = dict(zip(indices.tolist(), positions.tolist()))
            if not all(int(index) in mapping for index in required):
                return None
            selected = [mapping[int(index)] for index in required]
        return native_joint_vector(selected)
    except (ValueError, TypeError, IndexError):
        return None


def target_in_native_branch(target, measured):
    """Reject a distant historical command instead of changing its 2 pi branch."""
    return bool(target is not None and measured is not None and target.shape == measured.shape
                and np.all(np.abs(target - measured) < np.pi))
