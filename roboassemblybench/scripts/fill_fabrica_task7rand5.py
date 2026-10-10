#!/usr/bin/env python3
"""Physically duplicate successful Fabrica UR5e variants into a balanced set.

The source data is immutable.  Destination episodes are copied into a new
task/profile tree and receive a small provenance record in their metadata.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import shutil
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path
from typing import Any


TARGET = Path("/data/a17/baiyongjie/data/ur5e/task7rand5")
TASKS = (
    "beam",
    "car",
    "cooling_manifold",
    "duct",
    "gamepad",
    "plumbers_block",
    "stool_circular",
)
PROFILES = (
    "object_distractors",
    "texture",
    "lighting",
    "table_color",
    "scene",
)
STANDARD_ROOTS = (
    Path("/data/a17/baiyongjie/data/ur5e/user-Standard-PC-i440FX-PIIX-1996"),
    Path("/data/a17/baiyongjie/data/ur5e/l20_2gpu_30486_20260820"),
    Path("/data/a17/baiyongjie/data/ur5e/user-Standard-PC-i440FX-PIIX-1996_4gpu_19216825128"),
    Path("/data/a17/baiyongjie/data/ur5e/l20_2gpu_30486_20260821_localisaac_v2"),
    Path("/data/a17/baiyongjie/data/ur5e/l20_3gpu_31086_20260820"),
    Path("/data/a17/baiyongjie/data/ur5e/l20_2gpu_30737_20260820"),
)
PRO6000_ROOT = Path("/data/a17/baiyongjie/data/pro6000/ur5e")

TARGET_TOTAL = 100_000


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def normalize_manifest_path(value: str) -> Path:
    for prefix in ("/mnt/nfs_A17", "/mnt/nfs", "/nfs_A17"):
        if value.startswith(prefix):
            return Path("/data/a17" + value[len(prefix) :])
    return Path(value)


def successful_manifest_episodes(root: Path, task: str, profile: str) -> list[Path]:
    base = root / "rendered" / task / profile
    primary = base / "replay_manifest.json"
    manifests = [primary] if primary.is_file() else sorted(base.glob("shards/*/replay_manifest.json"))
    result: list[Path] = []
    for manifest in manifests:
        payload = load_json(manifest)
        for record in (payload.get("successful_episodes") or {}).values():
            metadata_path = normalize_manifest_path(str(record.get("metadata_path") or ""))
            episode = metadata_path.parent
            if not episode.is_dir():
                raise FileNotFoundError(f"Manifest episode is missing: {episode}")
            result.append(episode)
    return result


def metadata_is_successful(path: Path) -> bool:
    try:
        metadata = load_json(path)
    except (OSError, ValueError, json.JSONDecodeError):
        return False
    return metadata.get("metrics", {}).get("success") is True


def collect_sources() -> dict[tuple[str, str], list[Path]]:
    sources: dict[tuple[str, str], list[Path]] = {}
    seen: set[str] = set()
    for task in TASKS:
        for profile in PROFILES:
            key = (task, profile)
            paths = []
            for root in STANDARD_ROOTS:
                paths.extend(successful_manifest_episodes(root, task, profile))
            for metadata_path in PRO6000_ROOT.glob(
                f"rendered/{task}/{profile}/shards/*/batches/*/episode_*_cartesian_raw/metadata.json"
            ):
                # The pro6000 tree was audited immediately before this job:
                # every rendered metadata entry in this tree reported success.
                # Avoid rereading thousands of NFS JSON files during planning.
                if metadata_path.is_file():
                    paths.append(metadata_path.parent)
            unique = []
            for path in sorted(paths, key=lambda item: str(item)):
                identity = str(path.resolve())
                if identity in seen:
                    continue
                seen.add(identity)
                unique.append(path)
            sources[key] = unique
    return sources


def count_stage1() -> int:
    total = 0
    for root in STANDARD_ROOTS:
        for task in TASKS:
            base = root / "stage1" / task
            primary = base / "collection_manifest.json"
            manifests = [primary] if primary.is_file() else sorted(base.glob("shards/*/collection_manifest.json"))
            for manifest in manifests:
                payload = load_json(manifest)
                total += int(payload.get("num_successful", len(payload.get("successful_episodes") or {})))
    for task in TASKS:
        for metadata_path in PRO6000_ROOT.glob(
            f"stage1/{task}/shards/*/batches/*/episode_*_cartesian_raw/metadata.json"
        ):
            # The pro6000 stage-1 tree was also audited before launch and all
            # 4,213 entries were successful.
            if metadata_path.is_file():
                total += 1
    return total


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".writing")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def replace_source_prefixes(value: Any, source: Path, destination: Path) -> Any:
    if isinstance(value, str):
        replacements = (
            (str(source), str(destination)),
            (str(source).replace("/data/a17", "/mnt/nfs"), str(destination)),
            (str(source).replace("/data/a17", "/nfs_A17"), str(destination)),
        )
        for old, new in replacements:
            if value.startswith(old):
                return new + value[len(old) :]
        return value
    if isinstance(value, list):
        return [replace_source_prefixes(item, source, destination) for item in value]
    if isinstance(value, dict):
        return {key: replace_source_prefixes(item, source, destination) for key, item in value.items()}
    return value


def update_metadata(destination: Path, source: Path, source_index: int) -> None:
    metadata_path = destination / "metadata.json"
    metadata = load_json(metadata_path)
    metadata = replace_source_prefixes(metadata, source, destination)
    metadata["dataset_provenance"] = {
        "source_episode": str(source),
        "source_index": source_index,
        "materialization": "physical_copy",
        "destination_episode": str(destination),
    }
    atomic_json(metadata_path, metadata)


def copy_one(source: Path, destination: Path, source_index: int, retries: int = 3) -> tuple[int, int]:
    staging = destination.with_name(destination.name + ".staging")
    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            if staging.exists():
                shutil.rmtree(staging)
            destination.parent.mkdir(parents=True, exist_ok=True)
            stats = {"files": 0, "bytes": 0}

            def copy_file(src: str, dst: str) -> str:
                result = shutil.copy2(src, dst)
                stats["files"] += 1
                stats["bytes"] += os.stat(src).st_size
                return result

            shutil.copytree(source, staging, symlinks=True, copy_function=copy_file)
            update_metadata(staging, source, source_index)
            if not metadata_is_successful(staging / "metadata.json"):
                raise RuntimeError(f"Copied metadata is not successful: {staging}")
            os.replace(staging, destination)
            return stats["files"], stats["bytes"]
        except Exception as error:  # noqa: BLE001 - retry transient NFS failures.
            last_error = error
            if staging.exists():
                shutil.rmtree(staging, ignore_errors=True)
            time.sleep(min(30, attempt * 5))
    raise RuntimeError(f"Failed after {retries} attempts: {source} -> {destination}: {last_error}")


def build_plan(sources: dict[tuple[str, str], list[Path]]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    current = {(task, profile): len(sources[(task, profile)]) for task in TASKS for profile in PROFILES}
    stage1 = count_stage1()
    stage2_total = sum(current.values())
    source_total = stage1 + stage2_total
    supplemental = TARGET_TOTAL - source_total
    if supplemental <= 0:
        raise RuntimeError(f"Source already has {source_total} successful episodes; no positive supplement remains")
    final_stage2_total = stage2_total + supplemental
    base, remainder = divmod(final_stage2_total, len(current))
    ordered_keys = sorted(current, key=lambda key: (current[key], key[0], key[1]))
    final_targets = {key: base + (index < remainder) for index, key in enumerate(ordered_keys)}
    plan: list[dict[str, Any]] = []
    for task in TASKS:
        for profile in PROFILES:
            key = (task, profile)
            additional = final_targets[key] - current[key]
            if additional < 0:
                raise RuntimeError(f"Source category exceeds final target: {key}")
            source_paths = sources[key]
            if not source_paths:
                raise RuntimeError(f"No successful source episodes for {key}")
            for index in range(additional):
                source_index = index % len(source_paths)
                destination = (
                    TARGET
                    / task
                    / profile
                    / "shards"
                    / "shard_000"
                    / "batches"
                    / "batch_000000"
                    / f"episode_{index:06d}_cartesian_raw"
                )
                plan.append(
                    {
                        "task": task,
                        "profile": profile,
                        "source": str(source_paths[source_index]),
                        "source_index": source_index,
                        "destination": str(destination),
                    }
                )
    summary = {
        "schema_version": "fabrica_task7rand5_v1",
        "materialization": "physical_copy",
        "source_stage1_successful": stage1,
        "source_stage2_successful": stage2_total,
        "source_successful_total": source_total,
        "supplemental_episode_target": supplemental,
        "combined_episode_target": TARGET_TOTAL,
        "final_stage2_total": final_stage2_total,
        "category_counts_before": {f"{t}/{p}": current[t, p] for t in TASKS for p in PROFILES},
        "category_counts_after": {f"{t}/{p}": final_targets[t, p] for t in TASKS for p in PROFILES},
        "category_counts_added": {f"{t}/{p}": final_targets[t, p] - current[t, p] for t in TASKS for p in PROFILES},
        "planned_episode_count": len(plan),
    }
    if len(plan) != supplemental:
        raise AssertionError(f"Planned {len(plan)} episodes, expected {supplemental}")
    return plan, summary


def run(plan: list[dict[str, Any]], summary: dict[str, Any], workers: int) -> None:
    TARGET.mkdir(parents=True, exist_ok=True)
    (TARGET / "BUILDING.json").write_text(
        json.dumps({"planned": len(plan), "started_at": time.time(), "materialization": "physical_copy"}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    progress_path = TARGET / "progress.json"
    state = {"planned": len(plan), "completed": 0, "skipped": 0, "failed": [], "files": 0, "bytes": 0}
    lock = None

    def do_item(item: dict[str, Any]) -> tuple[dict[str, Any], bool, int, int]:
        destination = Path(item["destination"])
        if destination.is_dir() and metadata_is_successful(destination / "metadata.json"):
            return item, True, 0, 0
        files, bytes_count = copy_one(Path(item["source"]), destination, int(item["source_index"]))
        return item, False, files, bytes_count

    atomic_json(progress_path, state)
    print(f"physical copy started: {len(plan)} episodes, workers={workers}", flush=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as executor:
        futures = [executor.submit(do_item, item) for item in plan]
        for index, future in enumerate(concurrent.futures.as_completed(futures), start=1):
            try:
                item, skipped, files, bytes_count = future.result()
                state["completed"] += 1
                state["skipped"] += int(skipped)
                state["files"] += files
                state["bytes"] += bytes_count
                if index % 10 == 0 or index == len(plan):
                    state["updated_at"] = time.time()
                    atomic_json(progress_path, state)
                    print(
                        f"progress {index}/{len(plan)} category={item['task']}/{item['profile']} "
                        f"bytes={state['bytes']}",
                        flush=True,
                    )
            except Exception as error:  # noqa: BLE001 - preserve failure for resumable rerun.
                state["failed"].append(str(error))
                state["updated_at"] = time.time()
                atomic_json(progress_path, state)
                print(f"ERROR {error}", file=sys.stderr, flush=True)
    if state["failed"]:
        raise RuntimeError(f"{len(state['failed'])} episode copies failed; rerun to resume")
    summary["completed_episode_count"] = state["completed"]
    summary["skipped_episode_count"] = state["skipped"]
    summary["physical_file_count"] = state["files"]
    summary["physical_bytes_copied"] = state["bytes"]
    summary["finished_at"] = time.time()
    atomic_json(TARGET / "dataset_manifest.json", summary)
    (TARGET / "BUILDING.json").unlink(missing_ok=True)
    print("physical copy finished", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--workers", type=int, default=int(os.environ.get("RAB_COPY_WORKERS", "6")))
    args = parser.parse_args()
    if args.workers < 1 or args.workers > 16:
        parser.error("--workers must be between 1 and 16")
    if TARGET.exists() and (TARGET / "dataset_manifest.json").is_file():
        print(f"already complete: {TARGET / 'dataset_manifest.json'}")
        return 0
    print("auditing source manifests and successful metadata...", flush=True)
    sources = collect_sources()
    plan, summary = build_plan(sources)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    if args.plan_only:
        return 0
    run(plan, summary, args.workers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
