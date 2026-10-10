from __future__ import annotations

import json
import os
import re
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Iterable


TASKS = (
    'beam',
    'car',
    'cooling_manifold',
    'duct',
    'gamepad',
    'plumbers_block',
    'stool_circular',
)
PROFILES = ('object_distractors', 'texture', 'lighting', 'table_color', 'scene')
POSITION_PROFILE = 'position'
RAW_SCHEMA = 'roboassemblybench_raw_cartesian_v1'
OUTPUT_VIDEO_SHAPE = [480, 640, 3]


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _metadata_summary(path: Path) -> tuple[int, bool] | None:
    """Read only the small metadata regions needed for source selection."""
    try:
        with path.open('rb') as handle:
            header = handle.read(96 * 1024).decode('utf-8', errors='ignore')
            handle.seek(max(0, path.stat().st_size - 256 * 1024))
            trailer = handle.read(256 * 1024).decode('utf-8', errors='ignore')
    except OSError:
        return None
    if '"schema_version": "roboassemblybench_raw_cartesian_v1"' not in header:
        return None
    if not re.search(r'"success"\s*:\s*true', trailer):
        return None
    seed_match = re.search(r'"seed"\s*:\s*(-?\d+)', header)
    if seed_match is None:
        return None
    return int(seed_match.group(1)), True


def _episode_dirs(root: Path) -> Iterable[Path]:
    """Walk only directory metadata and prune each episode's large media tree."""
    if not root.is_dir():
        return
    for current, directories, _files in os.walk(root):
        episode_directories = [
            Path(current) / name
            for name in directories
            if name.startswith('episode_') and name.endswith('_cartesian_raw')
        ]
        for episode_dir in sorted(episode_directories):
            yield episode_dir
        directories[:] = [
            name
            for name in directories
            if not (name.startswith('episode_') and name.endswith('_cartesian_raw'))
        ]


def _record(episode_dir: Path, *, task: str, profile: str, source: str) -> dict[str, Any] | None:
    metadata_path = episode_dir / 'metadata.json'
    return {
        'metadata_path': str(metadata_path.resolve()),
        'seed': None,
        'task': task,
        'profile': profile,
        'source': source,
    }


def _classify_original(episode_dir: Path, root: Path) -> tuple[str, str] | None:
    try:
        parts = episode_dir.relative_to(root).parts
    except ValueError:
        return None
    if root.name in ('stage1', 'rendered'):
        marker = root.name
        task = parts[0] if parts else ''
        profile = POSITION_PROFILE if marker == 'stage1' else (parts[1] if len(parts) > 1 else '')
        if task in TASKS and (profile == POSITION_PROFILE or profile in PROFILES):
            return task, profile
    for marker in ('stage1', 'rendered'):
        if marker not in parts:
            continue
        index = parts.index(marker)
        if index + 1 >= len(parts):
            return None
        task = parts[index + 1]
        profile = POSITION_PROFILE if marker == 'stage1' else (
            parts[index + 2] if index + 2 < len(parts) else ''
        )
        if task in TASKS and (profile == POSITION_PROFILE or profile in PROFILES):
            return task, profile
    return None


def _collect_one_original_root(root: Path) -> dict[tuple[str, str], list[dict[str, Any]]]:
    records: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    excluded = {'task7rand5', 'diagnostics'}
    if not root.is_dir():
        return records
    for child in sorted(root.iterdir()):
        if child.name in excluded or not child.is_dir():
            continue
        for episode_dir in _episode_dirs(child):
            classified = _classify_original(episode_dir, child)
            if classified is None:
                continue
            task, profile = classified
            record = _record(episode_dir, task=task, profile=profile, source=child.name)
            if record is not None:
                records[(task, profile)].append(record)
    return records


def _collect_original(roots: Iterable[Path]) -> dict[tuple[str, str], list[dict[str, Any]]]:
    records: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    roots = list(roots)
    with ThreadPoolExecutor(max_workers=min(8, max(1, len(roots)))) as executor:
        for partial in executor.map(_collect_one_original_root, roots):
            for key, values in partial.items():
                records[key].extend(values)
    return records


def _collect_one_supplemental_category(
    root: Path, task: str, profile: str
) -> tuple[tuple[str, str], list[dict[str, Any]]]:
    records = []
    category_root = root / task / profile
    for episode_dir in _episode_dirs(category_root):
        record = _record(episode_dir, task=task, profile=profile, source='task7rand5')
        if record is not None:
            records.append(record)
    return (task, profile), records


def _collect_supplemental(root: Path) -> dict[tuple[str, str], list[dict[str, Any]]]:
    records: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    categories = [(task, profile) for task in TASKS for profile in PROFILES]
    with ThreadPoolExecutor(max_workers=min(8, len(categories))) as executor:
        for key, values in executor.map(
            lambda item: _collect_one_supplemental_category(root, *item), categories
        ):
            records[key].extend(values)
    return records


def _deduplicate(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    seen: set[int] = set()
    for record in sorted(records, key=lambda item: str(item['metadata_path'])):
        if record['seed'] is None:
            result.append(record)
            continue
        seed = int(record['seed'])
        if seed in seen:
            continue
        seen.add(seed)
        result.append(record)
    return result


def _allocate_position_targets(
    records: dict[tuple[str, str], list[dict[str, Any]]], target_total: int
) -> dict[str, int]:
    available = {task: len(_deduplicate(records.get((task, POSITION_PROFILE), []))) for task in TASKS}
    if sum(available.values()) < target_total:
        raise RuntimeError(f'Only {sum(available.values())} position episodes are available; need {target_total}.')
    raw = {task: target_total * available[task] / sum(available.values()) for task in TASKS}
    targets = {task: min(available[task], int(raw[task])) for task in TASKS}
    remaining = target_total - sum(targets.values())
    for task in sorted(TASKS, key=lambda name: (-(raw[name] - int(raw[name])), name)):
        if remaining <= 0:
            break
        if targets[task] < available[task]:
            targets[task] += 1
            remaining -= 1
    if remaining:
        raise RuntimeError(f'Could not allocate {target_total} position episodes; {remaining} remain.')
    return targets


def _write_manifest(
    path: Path,
    *,
    task: str,
    profile: str,
    records: list[dict[str, Any]],
    source_counts: dict[str, int],
    target_episodes: int,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'schema_version': 'roboassemblybench_lerobot_v3_source_manifest_v1',
        'task': task,
        'profile': profile,
        'target_successful_episodes': target_episodes,
        'candidate_episode_count': len(records),
        'source_counts': source_counts,
        'successful_episodes': {
            f'{index:06d}': {
                'metadata_path': record['metadata_path'],
                'seed': record['seed'],
                'source': record['source'],
            }
            for index, record in enumerate(records)
        },
    }
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    temporary.replace(path)


def prepare_sources(
    *,
    data_root: Path,
    output_root: Path,
    manifest_path: Path,
) -> dict[str, Any]:
    freeze = _load(manifest_path)
    original_roots = [data_root, data_root.parent / 'pro6000' / 'ur5e']
    original = _collect_original(original_roots)
    supplemental = _collect_supplemental(data_root / 'task7rand5')
    expected_rendered = freeze['category_counts_before']
    expected_added = freeze['category_counts_added']
    target_position_total = int(freeze['source_stage1_successful'])

    source_root = output_root / '_conversion_sources'
    source_root.mkdir(parents=True, exist_ok=True)
    summary: dict[str, Any] = {
        'schema_version': 'roboassemblybench_lerobot_v3_freeze_v1',
        'source_manifest': str(manifest_path.resolve()),
        'data_root': str(data_root.resolve()),
        'output_root': str(output_root.resolve()),
        'target_total_episodes': int(freeze['combined_episode_target']),
        'subsets': {},
    }

    position_targets = _allocate_position_targets(original, target_position_total)
    total = 0
    for task in TASKS:
        records = _deduplicate(original.get((task, POSITION_PROFILE), []))
        if len(records) < position_targets[task]:
            raise RuntimeError(f'Insufficient position records for {task}.')
        key = f'{task}/{POSITION_PROFILE}'
        _write_manifest(
            source_root / task / POSITION_PROFILE / 'collection_manifest.json',
            task=task,
            profile=POSITION_PROFILE,
            records=records,
            source_counts={'original_candidates': len(records)},
            target_episodes=position_targets[task],
        )
        summary['subsets'][key] = position_targets[task]
        total += position_targets[task]

    for task in TASKS:
        for profile in PROFILES:
            key = f'{task}/{profile}'
            original_records = _deduplicate(original.get((task, profile), []))
            supplemental_records = _deduplicate(supplemental.get((task, profile), []))
            original_target = int(expected_rendered[key])
            supplemental_target = int(expected_added[key])
            target = original_target + supplemental_target
            candidates = original_records + supplemental_records
            if len(candidates) < target:
                raise RuntimeError(
                    f'{key}: {len(candidates)} canonical records, need {target}.'
                )
            _write_manifest(
                source_root / task / profile / 'collection_manifest.json',
                task=task,
                profile=profile,
                records=candidates,
                source_counts={'original_target': original_target, 'task7rand5_target': supplemental_target},
                target_episodes=target,
            )
            summary['subsets'][key] = target
            total += target

    if total != int(freeze['combined_episode_target']):
        raise RuntimeError(f'Prepared {total} episodes, expected {freeze["combined_episode_target"]}.')
    summary['prepared_total_episodes'] = total
    temporary = (output_root / 'source_selection_manifest.json').with_suffix('.tmp')
    temporary.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    temporary.replace(output_root / 'source_selection_manifest.json')
    return summary


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description='Prepare frozen LeRobot v3 source manifests.')
    parser.add_argument('--data-root', required=True, type=Path)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--freeze-manifest', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare_sources(
        data_root=args.data_root,
        output_root=args.output_root,
        manifest_path=args.freeze_manifest,
    ), indent=2))


if __name__ == '__main__':
    main()
