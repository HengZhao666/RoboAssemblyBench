from __future__ import annotations

import argparse
import inspect
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from roboassemblybench.datasets.cartesian_episode import (
    ACTION_NAMES,
    ACTION_SEMANTICS,
    CAMERA_KEYS,
    STATE_NAMES,
    cartesian_trajectory_errors,
    task_progress_annotations,
)

DEFAULT_REPO_ID = 'baiyu858/roboassemblybench_fabrica_plumbers_block_ur5e_2k'
CONVERSION_MANIFEST = 'roboassemblybench_conversion_manifest.json'
COLLECTION_MANIFEST = 'collection_manifest.json'
JOINT_NAMES = tuple(f'{side}_joint_{index}' for side in ('left', 'right') for index in range(7))
EEF_POSE_NAMES = tuple(
    f'{side}_eef_{field}'
    for side in ('left', 'right')
    for field in ('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')
)
GRIPPER_NAMES = ('left_gripper_open', 'right_gripper_open')
WRIST_WRENCH_NAMES = tuple(
    f'{side}_wrist_{field}'
    for side in ('left', 'right')
    for field in ('force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z')
)
COLLISION_SIGNAL_NAMES = (
    'collision_detected',
    'left_gripper_contact',
    'right_gripper_contact',
    'locked_object_count',
)
OUTPUT_VIDEO_SHAPE = (480, 640, 3)
SCALAR_INT_FEATURES = (
    'phase_index',
    'phase_step',
    'subtask_index',
    'substage_index',
    'waiting_state',
    'handoff_state',
    'joint_state_available',
    'joint_velocity_available',
    'joint_effort_available',
    'wrist_wrench_available',
)


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _resolve_episode_asset(metadata: dict[str, Any], raw_path: str | None) -> Path:
    """Resolve assets after a dataset shard has been moved between machines."""
    raw = Path(raw_path or '')
    try:
        if raw.is_file():
            return raw
    except OSError:
        pass

    episode_root = Path(metadata['metadata_path']).resolve().parent
    candidates: list[Path] = []
    parts = raw.parts
    marker_index = next(
        (index for index, part in enumerate(parts) if part.endswith('_cartesian_raw')),
        None,
    )
    if marker_index is not None:
        relative_parts = parts[marker_index + 1 :]
        if relative_parts:
            candidates.append(episode_root.joinpath(*relative_parts))
    if raw.name:
        candidates.extend(
            [
                episode_root / raw.name,
                episode_root / 'videos' / raw.name,
                episode_root / 'annotations' / raw.name,
                episode_root / 'sensors' / 'depth' / raw.name,
            ]
        )
    for candidate in candidates:
        try:
            if candidate.is_file():
                return candidate
        except OSError:
            continue
    return raw


def _resolve_episode_assets(metadata: dict[str, Any]) -> dict[str, Any]:
    """Return metadata whose media paths are valid on the current filesystem."""
    resolved = dict(metadata)
    resolved['videos'] = {
        key: str(_resolve_episode_asset(metadata, value))
        for key, value in (metadata.get('videos') or {}).items()
    }
    for key in ('trajectory_path', 'annotation_path'):
        if metadata.get(key):
            resolved[key] = str(_resolve_episode_asset(metadata, metadata[key]))
    resolved_depth: dict[str, Any] = {}
    for camera_key, stream in (metadata.get('depth') or {}).items():
        stream_copy = dict(stream)
        if stream_copy.get('path'):
            stream_copy['path'] = str(_resolve_episode_asset(metadata, stream_copy['path']))
        resolved_depth[camera_key] = stream_copy
    resolved['depth'] = resolved_depth
    return resolved


def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f'{path.suffix}.tmp')
    temporary.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    temporary.replace(path)


def _load_successful_episode(metadata_path: Path) -> dict[str, Any] | None:
    try:
        if not metadata_path.is_file():
            return None
        metadata = _load_json(metadata_path)
    except (OSError, json.JSONDecodeError):
        return None
    if metadata.get('schema_version') != 'roboassemblybench_raw_cartesian_v1':
        return None
    if not bool((metadata.get('metrics') or {}).get('success', False)):
        return None
    metadata['metadata_path'] = str(metadata_path.resolve())
    return _resolve_episode_assets(metadata)


def _discover_successful_episodes(input_dir: Path) -> list[dict[str, Any]]:
    collection_manifest_path = input_dir / COLLECTION_MANIFEST
    metadata_paths = None
    if collection_manifest_path.is_file():
        collection_manifest = _load_json(collection_manifest_path)
        successful = collection_manifest.get('successful_episodes') or {}
        metadata_paths = [Path(item['metadata_path']) for item in successful.values()]

    episodes = []
    candidates = (
        metadata_paths if metadata_paths is not None else input_dir.rglob('episode_*_cartesian_raw/metadata.json')
    )
    for metadata_path in candidates:
        metadata = _load_successful_episode(metadata_path)
        if metadata is None:
            continue
        episodes.append(metadata)
    if metadata_paths is not None:
        # A collection manifest is an authoritative list of logical episodes.
        # Replayed/rendered variants may intentionally share a seed, while their
        # metadata paths and observations remain distinct.
        unique_paths: dict[str, dict[str, Any]] = {}
        for episode in episodes:
            unique_paths.setdefault(str(episode['metadata_path']), episode)
        return list(unique_paths.values())

    episodes = sorted(
        episodes,
        key=lambda item: (int(item.get('seed', -1)), str(item['metadata_path'])),
    )
    unique_episodes = []
    seen_seeds: set[int] = set()
    for episode in episodes:
        seed = int(episode.get('seed', -1))
        if seed in seen_seeds:
            continue
        seen_seeds.add(seed)
        unique_episodes.append(episode)
    return unique_episodes


def _ordered_collection_source_paths(input_dir: Path, manifest: dict[str, Any]) -> list[str]:
    collection = _load_json(input_dir / COLLECTION_MANIFEST)
    successful = collection.get('successful_episodes') or {}
    source_paths = list(dict.fromkeys(str(Path(item['metadata_path'])) for item in successful.values()))
    converted_sources = [str(item.get('source_metadata')) for item in manifest.get('episodes', [])]
    source_set = set(source_paths)
    missing = [source for source in converted_sources if source not in source_set]
    if missing:
        raise RuntimeError(
            f'{len(missing)} converted source episodes are absent from the authoritative collection manifest.'
        )
    converted_set = set(converted_sources)
    return converted_sources + [path for path in source_paths if path not in converted_set]


def _iter_valid_collection_episodes(
    source_paths: list[str],
    *,
    converted_sources: set[str],
    require_extended_observations: bool,
):
    for path in source_paths:
        if path in converted_sources:
            continue
        metadata = _load_successful_episode(Path(path))
        if metadata is None:
            continue
        try:
            _validate_metadata(
                metadata,
                require_extended_observations=require_extended_observations,
            )
        except Exception as exc:
            print(
                f"Skipping invalid source {metadata.get('metadata_path')}: "
                f'{type(exc).__name__}: {exc}',
                file=sys.stderr,
                flush=True,
            )
            continue
        yield metadata


def _select_collection_manifest_episodes(
    input_dir: Path,
    manifest: dict[str, Any],
    *,
    requested_episode_count: int | None,
    require_extended_observations: bool,
) -> list[dict[str, Any]]:
    """Compatibility helper for callers that still require an in-memory list."""
    ordered_paths = _ordered_collection_source_paths(input_dir, manifest)
    converted_sources = {str(item.get('source_metadata')) for item in manifest.get('episodes', [])}
    selected = []
    for path in ordered_paths:
        if requested_episode_count is not None and len(selected) >= requested_episode_count:
            break
        metadata = _load_successful_episode(Path(path))
        if metadata is None:
            if path in converted_sources:
                raise RuntimeError(f'Converted source episode is no longer readable: {path}')
            continue
        if path not in converted_sources:
            try:
                _validate_metadata(
                    metadata,
                    require_extended_observations=require_extended_observations,
                )
            except Exception as exc:
                print(
                    f"Skipping invalid source {metadata.get('metadata_path')}: "
                    f'{type(exc).__name__}: {exc}',
                    file=sys.stderr,
                    flush=True,
                )
                continue
        selected.append(metadata)
    return selected


def _prioritize_converted_sources(
    episodes: list[dict[str, Any]], manifest: dict[str, Any]
) -> list[dict[str, Any]]:
    """Keep an existing dataset prefix stable when source selection expands."""
    by_source = {str(item['metadata_path']): item for item in episodes}
    converted_sources = [str(item.get('source_metadata')) for item in manifest.get('episodes', [])]
    missing = [source for source in converted_sources if source not in by_source]
    if missing:
        raise RuntimeError(
            f'{len(missing)} converted source episodes are absent from the authoritative collection manifest.'
        )
    converted_set = set(converted_sources)
    return [by_source[source] for source in converted_sources] + [
        episode for episode in episodes if str(episode['metadata_path']) not in converted_set
    ]


def _select_valid_episodes(
    candidates: list[dict[str, Any]],
    manifest: dict[str, Any],
    *,
    requested_episode_count: int | None,
    require_extended_observations: bool,
) -> list[dict[str, Any]]:
    """Select valid candidates while preserving an already converted prefix."""
    converted_count = len(manifest.get('episodes', []))
    selected = list(candidates[:converted_count])
    for metadata in candidates[converted_count:]:
        if requested_episode_count is not None and len(selected) >= requested_episode_count:
            break
        try:
            _validate_metadata(
                metadata,
                require_extended_observations=require_extended_observations,
            )
        except Exception as exc:
            print(
                f"Skipping invalid source {metadata.get('metadata_path')}: {type(exc).__name__}: {exc}",
                file=sys.stderr,
                flush=True,
            )
            continue
        selected.append(metadata)
    return selected


def _validate_timing(metadata: dict[str, Any]) -> None:
    fps = int(metadata.get('fps', 0))
    simulation_fps = int(metadata.get('simulation_fps', 0))
    frame_stride = int(metadata.get('frame_stride', 0))
    timing = metadata.get('timing') or {}
    expected = {
        'physics_fps': simulation_fps,
        'control_fps': simulation_fps,
        'dataset_fps': fps,
        'dataset_frame_stride': frame_stride,
        'rendering_interval': frame_stride - 1,
        'camera_render_period_steps': frame_stride,
    }
    try:
        matches = all(int(timing.get(key, -1)) == value for key, value in expected.items())
    except (TypeError, ValueError):
        matches = False
    if (
        fps <= 0
        or frame_stride <= 0
        or simulation_fps != fps * frame_stride
        or not matches
        or not bool(timing.get('camera_state_action_aligned', False))
    ):
        raise ValueError(f"Camera/state/action timing mismatch in {metadata['metadata_path']}.")


def _validate_metadata(metadata: dict[str, Any], *, require_extended_observations: bool = False) -> None:
    _validate_timing(metadata)
    if list(metadata.get('state_names') or []) != list(STATE_NAMES):
        raise ValueError(f"State schema mismatch in {metadata['metadata_path']}.")
    if list(metadata.get('action_names') or []) != list(ACTION_NAMES):
        raise ValueError(f"Action schema mismatch in {metadata['metadata_path']}.")
    if metadata.get('action_semantics') != ACTION_SEMANTICS:
        raise ValueError(f"Action semantics mismatch in {metadata['metadata_path']}.")
    videos = metadata.get('videos') or {}
    missing = [key for key in CAMERA_KEYS if not videos.get(key) or not Path(videos[key]).is_file()]
    if missing:
        raise FileNotFoundError(f"Episode {metadata['metadata_path']} is missing videos: {missing}.")
    trajectory_path = Path(metadata.get('trajectory_path') or '')
    if not trajectory_path.is_file():
        raise FileNotFoundError(f"Episode {metadata['metadata_path']} is missing {trajectory_path}.")
    annotation_path = Path(metadata.get('annotation_path') or '')
    if require_extended_observations and not annotation_path.is_file():
        raise FileNotFoundError(f"Episode {metadata['metadata_path']} is missing its annotation sidecar.")
    depth = metadata.get('depth') or {}
    if require_extended_observations and set(depth) != set(CAMERA_KEYS):
        raise ValueError(f"Episode {metadata['metadata_path']} does not contain all metric depth streams.")
    if require_extended_observations:
        for camera_key, stream in depth.items():
            stream_path = Path(stream.get('path') or '')
            if (
                stream.get('dtype') != 'uint16'
                or stream.get('compression') != 'zstd'
                or stream.get('filter') != 'bitshuffle'
                or float(stream.get('depth_scale', -1.0)) != 0.001
                or int(stream.get('count', -1)) != int(metadata['frame_count'])
                or not stream_path.is_file()
            ):
                raise ValueError(f'Invalid metric depth stream {camera_key!r} in {metadata["metadata_path"]}.')

    required_arrays = {
        'joint_state': (int(metadata['frame_count']), 14),
        'joint_velocity': (int(metadata['frame_count']), 14),
        'joint_effort': (int(metadata['frame_count']), 14),
        'wrist_wrench': (int(metadata['frame_count']), 12),
        'collision_signal': (int(metadata['frame_count']), 4),
        'phase_index': (int(metadata['frame_count']),),
        'phase_step': (int(metadata['frame_count']),),
        'subtask_index': (int(metadata['frame_count']),),
        'substage_index': (int(metadata['frame_count']),),
        'waiting_state': (int(metadata['frame_count']),),
        'handoff_state': (int(metadata['frame_count']),),
    }
    with np.load(trajectory_path) as trajectory:
        for name, shape in required_arrays.items():
            if name not in trajectory or np.asarray(trajectory[name]).shape != shape:
                raise ValueError(f'Missing or invalid {name!r} in {trajectory_path}.')


def _conversion_entry(metadata: dict[str, Any], episode_index: int) -> dict[str, Any]:
    domain_randomization = metadata.get('domain_randomization') or {}
    return {
        'episode_index': int(episode_index),
        'seed': int(metadata['seed']),
        'layout_seed': int(metadata.get('layout_seed', domain_randomization.get('seed', metadata['seed']))),
        'source_metadata': str(Path(metadata['metadata_path']).resolve()),
        'frame_count': int(metadata['frame_count']),
        'annotation_path': f'annotations/episode_{episode_index:06d}.json',
        'depth_index_path': f'sensors/depth/index/episode_{episode_index:06d}.json',
        'domain_randomization': domain_randomization,
    }


def _reconcile_conversion_manifest(
    manifest: dict[str, Any],
    episodes: list[dict[str, Any]],
    *,
    dataset_episode_count: int,
) -> bool:
    manifest_entries = manifest.setdefault('episodes', [])
    manifest_count = len(manifest_entries)
    if manifest_count > dataset_episode_count:
        raise RuntimeError(
            f'Conversion manifest has {manifest_count} episodes but LeRobot has {dataset_episode_count}.'
        )
    if dataset_episode_count > len(episodes):
        raise RuntimeError(
            f'LeRobot has {dataset_episode_count} episodes but only {len(episodes)} authoritative sources exist.'
        )
    expected_prefix = [str(Path(metadata['metadata_path']).resolve()) for metadata in episodes[:manifest_count]]
    actual_prefix = [str(item.get('source_metadata')) for item in manifest_entries]
    if actual_prefix != expected_prefix:
        raise RuntimeError('Conversion manifest is not a prefix of the authoritative source episode order.')

    changed = False
    for episode_index in range(manifest_count, dataset_episode_count):
        manifest_entries.append(_conversion_entry(episodes[episode_index], episode_index))
        changed = True
    if changed:
        manifest['total_episodes'] = len(manifest_entries)
        manifest['total_frames'] = sum(int(item['frame_count']) for item in manifest_entries)
    return changed


def _features(first_episode: dict[str, Any]) -> dict[str, dict[str, Any]]:
    features: dict[str, dict[str, Any]] = {
        'observation.state': {
            'dtype': 'float32',
            'shape': (len(STATE_NAMES),),
            'names': list(STATE_NAMES),
        },
        'action': {
            'dtype': 'float32',
            'shape': (len(ACTION_NAMES),),
            'names': list(ACTION_NAMES),
        },
        'observation.eef_pose': {
            'dtype': 'float32',
            'shape': (14,),
            'names': list(EEF_POSE_NAMES),
        },
        'observation.gripper_state': {
            'dtype': 'float32',
            'shape': (2,),
            'names': list(GRIPPER_NAMES),
        },
        'observation.joint_state': {
            'dtype': 'float32',
            'shape': (14,),
            'names': list(JOINT_NAMES),
        },
        'observation.joint_velocity': {
            'dtype': 'float32',
            'shape': (14,),
            'names': list(JOINT_NAMES),
        },
        'observation.joint_effort': {
            'dtype': 'float32',
            'shape': (14,),
            'names': list(JOINT_NAMES),
        },
        'observation.wrist_wrench': {
            'dtype': 'float32',
            'shape': (12,),
            'names': list(WRIST_WRENCH_NAMES),
        },
        'observation.collision_signal': {
            'dtype': 'float32',
            'shape': (4,),
            'names': list(COLLISION_SIGNAL_NAMES),
        },
    }
    for feature_name in SCALAR_INT_FEATURES:
        features[f'observation.{feature_name}'] = {
            'dtype': 'int64',
            'shape': (1,),
            'names': [feature_name],
        }
    for camera_key in CAMERA_KEYS:
        features[camera_key] = {
            'dtype': 'video',
            'shape': OUTPUT_VIDEO_SHAPE,
            'names': ['height', 'width', 'channels'],
            'info': {'is_depth_map': False},
        }
    return features


def _open_video_captures(metadata: dict[str, Any]) -> dict[str, cv2.VideoCapture]:
    captures = {}
    for camera_key in CAMERA_KEYS:
        capture = cv2.VideoCapture(str(metadata['videos'][camera_key]))
        if not capture.isOpened():
            raise RuntimeError(f"Cannot open {camera_key} video: {metadata['videos'][camera_key]}")
        captures[camera_key] = capture
    return captures


def _release_video_captures(captures: dict[str, cv2.VideoCapture]) -> None:
    for capture in captures.values():
        capture.release()


def _fit_rgb_frame(frame: np.ndarray) -> np.ndarray:
    """Center-crop and resize source RGB frames to the fixed LeRobot shape."""
    target_height, target_width, target_channels = OUTPUT_VIDEO_SHAPE
    if frame.ndim != 3 or frame.shape[2] != target_channels:
        raise ValueError(f'Invalid RGB frame shape: {frame.shape}.')
    height, width = frame.shape[:2]
    target_ratio = target_width / target_height
    source_ratio = width / height
    if source_ratio > target_ratio:
        crop_width = max(1, int(round(height * target_ratio)))
        left = max(0, (width - crop_width) // 2)
        frame = frame[:, left : left + crop_width]
    elif source_ratio < target_ratio:
        crop_height = max(1, int(round(width / target_ratio)))
        top = max(0, (height - crop_height) // 2)
        frame = frame[top : top + crop_height, :]
    if frame.shape[:2] != (target_height, target_width):
        frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
    return np.asarray(frame, dtype=np.uint8)


def _link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if destination.stat().st_size == source.stat().st_size:
            return
        destination.unlink()
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def _export_episode_sidecars(metadata: dict[str, Any], *, episode_index: int, output_dir: Path) -> None:
    annotation_destination = output_dir / 'annotations' / f'episode_{episode_index:06d}.json'
    annotation_source = Path(metadata.get('annotation_path') or '')
    if annotation_source.is_file():
        annotation = _load_json(annotation_source)
        with np.load(metadata['trajectory_path']) as trajectory:
            annotation.update(
                task_progress_annotations(
                    metadata.get('phase_annotations') or annotation.get('phase_annotations') or {},
                    trajectory['subtask_index'],
                    trajectory['substage_index'],
                )
            )
        _write_json_atomic(annotation_destination, annotation)

    depth_index = {
        'schema_version': 'roboassemblybench_metric_depth_index_v1',
        'episode_index': int(episode_index),
        'frame_count': int(metadata['frame_count']),
        'fps': int(metadata['fps']),
        'streams': {},
    }
    defer_depth = os.environ.get('RAB_DEFER_DEPTH_SIDECARS', '').strip().lower() in {
        '1',
        'true',
        'yes',
    }
    chunk = episode_index // 1000
    for camera_key, stream in (metadata.get('depth') or {}).items():
        source = Path(stream['path'])
        relative_path = (
            Path('sensors')
            / 'depth'
            / camera_key
            / f'chunk-{chunk:03d}'
            / f'episode_{episode_index:06d}.u16.bshuf.zst'
        )
        if not defer_depth:
            _link_or_copy(source, output_dir / relative_path)
        stream_index = {
            **{key: value for key, value in stream.items() if key != 'path'},
            'path': relative_path.as_posix(),
        }
        if defer_depth:
            stream_index['deferred_source_path'] = str(source)
        depth_index['streams'][camera_key] = stream_index
    _write_json_atomic(
        output_dir / 'sensors' / 'depth' / 'index' / f'episode_{episode_index:06d}.json',
        depth_index,
    )


def _append_episode(dataset, metadata: dict[str, Any], *, episode_index: int, output_dir: Path) -> int:
    expected_frames = int(metadata['frame_count'])
    with np.load(metadata['trajectory_path']) as trajectory:
        states = np.asarray(trajectory['observation_state'], dtype=np.float32)
        actions = np.asarray(trajectory['action'], dtype=np.float32)
        if states.shape != (expected_frames, len(STATE_NAMES)):
            raise ValueError(f"Invalid state shape in {metadata['trajectory_path']}: {states.shape}.")
        if actions.shape != (expected_frames, len(ACTION_NAMES)):
            raise ValueError(f"Invalid action shape in {metadata['trajectory_path']}: {actions.shape}.")
        trajectory_errors = cartesian_trajectory_errors(
            states,
            actions,
            simulation_steps=trajectory.get('simulation_step'),
            frame_stride=int(metadata['frame_stride']),
        )
        if trajectory_errors:
            raise ValueError(f"Invalid Cartesian trajectory in {metadata['trajectory_path']}: {trajectory_errors}.")

        captures = _open_video_captures(metadata)
        try:
            for frame_index in range(expected_frames):
                frame = {
                    'observation.state': states[frame_index],
                    'observation.eef_pose': np.concatenate(
                        [states[frame_index, 0:7], states[frame_index, 8:15]]
                    ).astype(np.float32, copy=False),
                    'observation.gripper_state': states[frame_index, [7, 15]],
                    'observation.joint_state': np.asarray(trajectory['joint_state'][frame_index], dtype=np.float32),
                    'observation.joint_velocity': np.asarray(
                        trajectory['joint_velocity'][frame_index], dtype=np.float32
                    ),
                    'observation.joint_effort': np.asarray(trajectory['joint_effort'][frame_index], dtype=np.float32),
                    'observation.wrist_wrench': np.asarray(trajectory['wrist_wrench'][frame_index], dtype=np.float32),
                    'observation.collision_signal': np.asarray(
                        trajectory['collision_signal'][frame_index], dtype=np.float32
                    ),
                    'action': actions[frame_index],
                    'task': str(metadata['task']),
                }
                for source_name in SCALAR_INT_FEATURES:
                    scalar = np.asarray(trajectory[source_name][frame_index])
                    if scalar.size != 1:
                        raise ValueError(
                            f'Expected scalar {source_name!r} at frame {frame_index}, got {scalar.shape}.'
                        )
                    frame[f'observation.{source_name}'] = np.asarray(
                        [scalar.reshape(-1)[0]], dtype=np.int64
                    )
                for camera_key, capture in captures.items():
                    ok, bgr = capture.read()
                    if not ok or bgr is None:
                        raise RuntimeError(
                            f'{camera_key} ended at frame {frame_index}/{expected_frames} for '
                            f"{metadata['metadata_path']}."
                        )
                    frame[camera_key] = _fit_rgb_frame(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                dataset.add_frame(frame)
        finally:
            _release_video_captures(captures)

    # LeRobot 0.4.4 validates shape-(1,) numeric frames as arrays but maps them to
    # Arrow scalar columns. Normalize the validated buffer before its own np.stack().
    for source_name in SCALAR_INT_FEATURES:
        feature_name = f'observation.{source_name}'
        values = dataset.episode_buffer.get(feature_name, [])
        if len(values) != expected_frames:
            raise ValueError(f'Invalid buffered frame count for {feature_name!r}: {len(values)}.')
        dataset.episode_buffer[feature_name] = [int(np.asarray(value).reshape(-1)[0]) for value in values]

    _export_episode_sidecars(metadata, episode_index=episode_index, output_dir=output_dir)
    dataset.save_episode()
    return expected_frames


def _export_collection_manifest_dataset(
    *,
    lerobot,
    dataset_class,
    input_dir: Path,
    output_dir: Path,
    repo_id: str,
    manifest: dict[str, Any],
    manifest_path: Path,
    requested_episode_count: int | None,
    resume: bool,
    streaming_encoding: bool,
    encoder_threads: int | None,
    vcodec: str,
    require_extended_observations: bool,
) -> dict[str, Any]:
    """Stream a frozen collection manifest without retaining episode JSON in memory."""
    if manifest.get('repo_id') != repo_id:
        raise ValueError(
            f"Existing conversion manifest repo_id={manifest.get('repo_id')!r} does not match {repo_id!r}."
        )

    entries = manifest.setdefault('episodes', [])
    if requested_episode_count is not None and len(entries) > requested_episode_count:
        raise RuntimeError(
            f'Existing conversion has {len(entries)} episodes, exceeding the '
            f'current target of {requested_episode_count}.'
        )
    ordered_paths = _ordered_collection_source_paths(input_dir, manifest)
    converted_sources = {str(item.get('source_metadata')) for item in entries}
    candidates = _iter_valid_collection_episodes(
        ordered_paths,
        converted_sources=converted_sources,
        require_extended_observations=require_extended_observations,
    )

    info_path = output_dir / 'meta' / 'info.json'
    has_dataset = info_path.is_file()
    first_episode = None
    if has_dataset:
        episode_metadata_dir = output_dir / 'meta' / 'episodes'
        has_episode_metadata = episode_metadata_dir.is_dir() and any(
            episode_metadata_dir.rglob('*.parquet')
        )
        partial_info = _load_json(info_path)
        if not has_episode_metadata and int(partial_info.get('total_episodes', 0)) == 0:
            raise RuntimeError(
                f'Incomplete LeRobot metadata at {output_dir} conflicts with its conversion manifest.'
            )
        if not resume:
            raise FileExistsError(f'LeRobot dataset already exists at {output_dir}; pass --resume.')
        resume_method = getattr(dataset_class, 'resume', None)
        if callable(resume_method):
            dataset = resume_method(
                repo_id,
                root=output_dir,
                streaming_encoding=streaming_encoding,
                encoder_threads=encoder_threads,
            )
        else:
            resume_kwargs = {
                'repo_id': repo_id,
                'root': output_dir,
                'streaming_encoding': streaming_encoding,
                'encoder_threads': encoder_threads,
            }
            if 'vcodec' in inspect.signature(dataset_class).parameters:
                resume_kwargs['vcodec'] = vcodec
            dataset = dataset_class(**resume_kwargs)
        fps = int(partial_info['fps'])
    else:
        if entries:
            raise RuntimeError(
                f'Incomplete LeRobot metadata at {output_dir} conflicts with its conversion manifest.'
            )
        if requested_episode_count == 0:
            raise RuntimeError('At least one episode is required to create a LeRobot dataset.')
        first_episode = next(candidates, None)
        if first_episode is None:
            raise RuntimeError(f'No successful compact Cartesian episodes found under {input_dir}.')
        fps = int(first_episode['fps'])
        create_kwargs = {
            'repo_id': repo_id,
            'root': output_dir,
            'fps': fps,
            'robot_type': 'dual_ur5e_robotiq_2f85',
            'features': _features(first_episode),
            'use_videos': True,
            'streaming_encoding': streaming_encoding,
            'encoder_threads': encoder_threads,
            'metadata_buffer_size': 10,
        }
        if 'vcodec' in inspect.signature(dataset_class.create).parameters:
            create_kwargs['vcodec'] = vcodec
        dataset = dataset_class.create(**create_kwargs)

    dataset_episode_count = int(dataset.meta.total_episodes)
    if len(entries) > dataset_episode_count:
        raise RuntimeError(
            f'Conversion manifest has {len(entries)} episodes but LeRobot has {dataset_episode_count}.'
        )
    if requested_episode_count is not None and dataset_episode_count > requested_episode_count:
        raise RuntimeError(
            f'LeRobot has {dataset_episode_count} episodes, exceeding the target of '
            f'{requested_episode_count}.'
        )

    def checkpoint_manifest() -> None:
        manifest['total_episodes'] = len(entries)
        manifest['total_frames'] = sum(int(item['frame_count']) for item in entries)
        _write_json_atomic(manifest_path, manifest)

    # save_episode() can complete immediately before a process is interrupted.
    # Rebuild at most the small uncheckpointed suffix from the frozen source order.
    while len(entries) < dataset_episode_count:
        metadata = next(candidates, None)
        if metadata is None:
            raise RuntimeError(
                f'LeRobot has {dataset_episode_count} episodes but its authoritative sources are exhausted.'
            )
        if int(metadata['fps']) != fps:
            raise ValueError(f"FPS mismatch in {metadata['metadata_path']}: {metadata['fps']} != {fps}.")
        episode_index = len(entries)
        _export_episode_sidecars(metadata, episode_index=episode_index, output_dir=output_dir)
        entries.append(_conversion_entry(metadata, episode_index))
    if dataset_episode_count:
        checkpoint_manifest()

    added_episodes = 0
    added_frames = 0

    def remaining_candidates():
        if first_episode is not None:
            yield first_episode
        yield from candidates

    exhausted = True
    try:
        for metadata in remaining_candidates():
            if requested_episode_count is not None and len(entries) >= requested_episode_count:
                exhausted = False
                break
            if int(metadata['fps']) != fps:
                raise ValueError(f"FPS mismatch in {metadata['metadata_path']}: {metadata['fps']} != {fps}.")
            episode_index = len(entries)
            frame_count = _append_episode(
                dataset,
                metadata,
                episode_index=episode_index,
                output_dir=output_dir,
            )
            entries.append(_conversion_entry(metadata, episode_index))
            added_episodes += 1
            added_frames += frame_count
            if len(entries) % 10 == 0:
                checkpoint_manifest()
                print(
                    json.dumps(
                        {
                            'event': 'conversion_progress',
                            'output_dir': str(output_dir),
                            'episodes': len(entries),
                            'target': requested_episode_count,
                        }
                    ),
                    flush=True,
                )
    finally:
        dataset.finalize()
    checkpoint_manifest()

    info = _load_json(info_path)
    info['roboassemblybench_sidecars'] = {
        'metric_depth': {
            'schema_version': 'roboassemblybench_metric_depth_index_v1',
            'index_path': 'sensors/depth/index/episode_{episode_index:06d}.json',
            'dtype': 'uint16',
            'compression': 'zstd',
            'filter': 'bitshuffle',
            'units': 'millimeters',
            'depth_scale': 0.001,
            'camera_keys': list(CAMERA_KEYS),
        },
        'long_horizon_annotations': {
            'schema_version': 'roboassemblybench_long_horizon_annotation_v1',
            'path': 'annotations/episode_{episode_index:06d}.json',
        },
    }
    _write_json_atomic(info_path, info)

    summary = {
        'repo_id': repo_id,
        'input_dir': str(input_dir.resolve()),
        'output_dir': str(output_dir.resolve()),
        'codebase_version': 'v3.0',
        'lerobot_version': str(getattr(lerobot, '__version__', 'unknown')),
        'fps': fps,
        'source_successful_episodes': len(entries),
        'added_episodes': added_episodes,
        'added_frames': added_frames,
        'total_episodes': len(entries),
        'total_frames': int(manifest['total_frames']),
        'camera_keys': list(CAMERA_KEYS),
        'depth_camera_keys': list(CAMERA_KEYS),
        'depth_dtype': 'uint16',
        'depth_compression': 'zstd',
        'depth_filter': 'bitshuffle',
        'depth_scale': 0.001,
        'annotation_schema_version': 'roboassemblybench_long_horizon_annotation_v1',
        'require_extended_observations': bool(require_extended_observations),
        'state_names': list(STATE_NAMES),
        'action_names': list(ACTION_NAMES),
    }
    _write_json_atomic(output_dir / 'roboassemblybench_export_summary.json', summary)
    reached_target = (
        requested_episode_count is None and exhausted
    ) or len(entries) == requested_episode_count
    if reached_target:
        _write_json_atomic(
            output_dir / '.roboassemblybench_export_complete',
            {
                'schema_version': 'roboassemblybench_lerobot_v3_export_complete_v1',
                'total_episodes': len(entries),
                'total_frames': summary['total_frames'],
            },
        )
    return summary


def export_dataset(
    *,
    input_dir: Path,
    output_dir: Path,
    repo_id: str,
    max_episodes: int | None,
    resume: bool,
    streaming_encoding: bool,
    encoder_threads: int | None,
    vcodec: str,
    require_extended_observations: bool = False,
) -> dict[str, Any]:
    try:
        import lerobot
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
        from lerobot.datasets import video_utils
    except ImportError as exc:
        raise RuntimeError(
            'LeRobot with v3 dataset support is required. Run this script in the ' 'roboassemblybench-act environment.'
        ) from exc
    if str(CODEBASE_VERSION) != 'v3.0':
        raise RuntimeError(f'Expected LeRobot dataset codebase v3.0, got {CODEBASE_VERSION!r}.')

    h264_preset = os.environ.get('RAB_H264_PRESET', '').strip()
    if h264_preset and not getattr(video_utils._get_codec_options, '_rab_patched', False):
        original_codec_options = video_utils._get_codec_options

        def codec_options_with_h264_preset(vcodec, g=2, crf=30, preset=None):
            options = original_codec_options(vcodec, g, crf, preset)
            if vcodec in ('h264', 'hevc'):
                options['preset'] = h264_preset
            return options

        codec_options_with_h264_preset._rab_patched = True
        video_utils._get_codec_options = codec_options_with_h264_preset

    manifest_path = output_dir / CONVERSION_MANIFEST
    if manifest_path.exists():
        manifest = _load_json(manifest_path)
    else:
        manifest = {
            'schema_version': 'roboassemblybench_lerobot_v3_conversion_v1',
            'repo_id': repo_id,
            'input_dir': str(input_dir.resolve()),
            'output_dir': str(output_dir.resolve()),
            'episodes': [],
        }
    requested_episode_count = None if max_episodes is None else max(int(max_episodes), 0)
    if (input_dir / COLLECTION_MANIFEST).is_file():
        return _export_collection_manifest_dataset(
            lerobot=lerobot,
            dataset_class=LeRobotDataset,
            input_dir=input_dir,
            output_dir=output_dir,
            repo_id=repo_id,
            manifest=manifest,
            manifest_path=manifest_path,
            requested_episode_count=requested_episode_count,
            resume=resume,
            streaming_encoding=streaming_encoding,
            encoder_threads=encoder_threads,
            vcodec=vcodec,
            require_extended_observations=require_extended_observations,
        )
    candidates = _prioritize_converted_sources(_discover_successful_episodes(input_dir), manifest)
    episodes = _select_valid_episodes(
        candidates,
        manifest,
        requested_episode_count=requested_episode_count,
        require_extended_observations=require_extended_observations,
    )
    if not episodes:
        raise RuntimeError(f'No successful compact Cartesian episodes found under {input_dir}.')
    if len(manifest.get('episodes', [])) > len(episodes):
        raise RuntimeError(
            f'Existing conversion has {len(manifest["episodes"])} episodes, exceeding the '
            f'current target of {len(episodes)}.'
        )
    if not manifest.get('episodes'):
        _validate_metadata(
            episodes[0],
            require_extended_observations=require_extended_observations,
        )
    fps_values = {int(metadata['fps']) for metadata in episodes}
    if len(fps_values) != 1:
        raise ValueError(f'All source episodes must use one FPS, got {sorted(fps_values)}.')
    fps = fps_values.pop()

    if manifest.get('repo_id') != repo_id:
        raise ValueError(
            f"Existing conversion manifest repo_id={manifest.get('repo_id')!r} does not match {repo_id!r}."
        )
    info_path = output_dir / 'meta' / 'info.json'
    has_dataset = info_path.is_file()
    if has_dataset:
        episode_metadata_dir = output_dir / 'meta' / 'episodes'
        has_episode_metadata = episode_metadata_dir.is_dir() and any(
            episode_metadata_dir.rglob('*.parquet')
        )
        partial_info = _load_json(info_path)
        if not has_episode_metadata and int(partial_info.get('total_episodes', 0)) == 0:
            if manifest_path.exists():
                raise RuntimeError(
                    f'Incomplete LeRobot metadata at {output_dir} conflicts with its conversion manifest.'
                )
            shutil.rmtree(output_dir)
            has_dataset = False
    if has_dataset:
        if not resume:
            raise FileExistsError(f'LeRobot dataset already exists at {output_dir}; pass --resume.')
        resume_method = getattr(LeRobotDataset, 'resume', None)
        if callable(resume_method):
            dataset = resume_method(
                repo_id,
                root=output_dir,
                streaming_encoding=streaming_encoding,
                encoder_threads=encoder_threads,
            )
        else:
            resume_kwargs = {
                'repo_id': repo_id,
                'root': output_dir,
                'streaming_encoding': streaming_encoding,
                'encoder_threads': encoder_threads,
            }
            if 'vcodec' in inspect.signature(LeRobotDataset).parameters:
                resume_kwargs['vcodec'] = vcodec
            dataset = LeRobotDataset(**resume_kwargs)
    else:
        create_kwargs = {
            'repo_id': repo_id,
            'root': output_dir,
            'fps': fps,
            'robot_type': 'dual_ur5e_robotiq_2f85',
            'features': _features(episodes[0]),
            'use_videos': True,
            'streaming_encoding': streaming_encoding,
            'encoder_threads': encoder_threads,
            'metadata_buffer_size': 10,
        }
        if 'vcodec' in inspect.signature(LeRobotDataset.create).parameters:
            create_kwargs['vcodec'] = vcodec
        dataset = LeRobotDataset.create(
            **create_kwargs,
        )

    dataset_episode_count = int(dataset.meta.total_episodes)
    if _reconcile_conversion_manifest(
        manifest,
        episodes,
        dataset_episode_count=dataset_episode_count,
    ):
        _write_json_atomic(manifest_path, manifest)
    for episode_index, metadata in enumerate(episodes[:dataset_episode_count]):
        _export_episode_sidecars(metadata, episode_index=episode_index, output_dir=output_dir)
    processed_sources = {str(item['source_metadata']) for item in manifest.get('episodes', [])}

    added_episodes = 0
    added_frames = 0
    try:
        for metadata in episodes:
            source_metadata = str(Path(metadata['metadata_path']).resolve())
            if source_metadata in processed_sources:
                continue
            episode_index = len(manifest['episodes'])
            frame_count = _append_episode(
                dataset,
                metadata,
                episode_index=episode_index,
                output_dir=output_dir,
            )
            manifest['episodes'].append(_conversion_entry(metadata, episode_index))
            processed_sources.add(source_metadata)
            added_episodes += 1
            added_frames += frame_count
            manifest['total_episodes'] = len(manifest['episodes'])
            manifest['total_frames'] = sum(int(item['frame_count']) for item in manifest['episodes'])
            _write_json_atomic(manifest_path, manifest)
    finally:
        dataset.finalize()

    info_path = output_dir / 'meta' / 'info.json'
    info = _load_json(info_path)
    info['roboassemblybench_sidecars'] = {
        'metric_depth': {
            'schema_version': 'roboassemblybench_metric_depth_index_v1',
            'index_path': 'sensors/depth/index/episode_{episode_index:06d}.json',
            'dtype': 'uint16',
            'compression': 'zstd',
            'filter': 'bitshuffle',
            'units': 'millimeters',
            'depth_scale': 0.001,
            'camera_keys': list(CAMERA_KEYS),
        },
        'long_horizon_annotations': {
            'schema_version': 'roboassemblybench_long_horizon_annotation_v1',
            'path': 'annotations/episode_{episode_index:06d}.json',
        },
    }
    _write_json_atomic(info_path, info)

    summary = {
        'repo_id': repo_id,
        'input_dir': str(input_dir.resolve()),
        'output_dir': str(output_dir.resolve()),
        'codebase_version': 'v3.0',
        'lerobot_version': str(getattr(lerobot, '__version__', 'unknown')),
        'fps': fps,
        'source_successful_episodes': len(episodes),
        'added_episodes': added_episodes,
        'added_frames': added_frames,
        'total_episodes': len(manifest['episodes']),
        'total_frames': sum(int(item['frame_count']) for item in manifest['episodes']),
        'camera_keys': list(CAMERA_KEYS),
        'depth_camera_keys': list(CAMERA_KEYS),
        'depth_dtype': 'uint16',
        'depth_compression': 'zstd',
        'depth_filter': 'bitshuffle',
        'depth_scale': 0.001,
        'annotation_schema_version': 'roboassemblybench_long_horizon_annotation_v1',
        'require_extended_observations': bool(require_extended_observations),
        'state_names': list(STATE_NAMES),
        'action_names': list(ACTION_NAMES),
    }
    _write_json_atomic(output_dir / 'roboassemblybench_export_summary.json', summary)
    reached_requested_count = (
        requested_episode_count is None or len(manifest['episodes']) == requested_episode_count
    )
    if len(manifest['episodes']) == len(episodes) and reached_requested_count:
        _write_json_atomic(
            output_dir / '.roboassemblybench_export_complete',
            {
                'schema_version': 'roboassemblybench_lerobot_v3_export_complete_v1',
                'total_episodes': len(manifest['episodes']),
                'total_frames': summary['total_frames'],
            },
        )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description='Export compact RoboAssemblyBench episodes to LeRobot v3.')
    parser.add_argument('--input-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--repo-id', default=DEFAULT_REPO_ID)
    parser.add_argument('--max-episodes', type=int, default=None)
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--no-streaming-encoding', action='store_true')
    parser.add_argument('--encoder-threads', type=int, default=2)
    parser.add_argument('--vcodec', default='h264')
    parser.add_argument('--require-extended-observations', action='store_true')
    args = parser.parse_args()

    summary = export_dataset(
        input_dir=Path(args.input_dir),
        output_dir=Path(args.output_dir),
        repo_id=str(args.repo_id),
        max_episodes=args.max_episodes,
        resume=bool(args.resume),
        streaming_encoding=not bool(args.no_streaming_encoding),
        encoder_threads=max(int(args.encoder_threads), 1),
        vcodec=str(args.vcodec),
        require_extended_observations=bool(args.require_extended_observations),
    )
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
