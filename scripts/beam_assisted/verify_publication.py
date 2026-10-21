#!/usr/bin/env python3
"""Verify this publication and optionally restore exact gzip evidence; no simulator imports."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import tempfile


def local_path(root, relative):
    relative = PurePosixPath(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("Unsafe publication path: " + str(relative))
    path = root / str(relative)
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        raise ValueError("Publication path escapes checkout: " + str(relative))
    return path


def identity(path):
    checksum = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(block)
            size += len(block)
    return size, checksum.hexdigest()


def restore_one(root, row):
    if row["encoding"] != "gzip":
        return False
    stored = local_path(root, row["stored_path"])
    if identity(stored) != (row["stored_bytes"], row["stored_sha256"]):
        raise ValueError("Compressed evidence differs: " + row["stored_path"])
    target = local_path(root, row["original_path"])
    expected = (row["original_bytes"], row["original_sha256"])
    if target.exists():
        if identity(target) != expected:
            raise FileExistsError("Refusing to replace different existing evidence: " + str(target))
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix=".restore-evidence-", delete=False) as output:
            temporary = Path(output.name)
            checksum = hashlib.sha256()
            size = 0
            with gzip.open(stored, "rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    output.write(block)
                    checksum.update(block)
                    size += len(block)
            output.flush()
            os.fsync(output.fileno())
        if (size, checksum.hexdigest()) != expected:
            raise ValueError("Restored evidence differs: " + row["original_path"])
        # The same-directory hard link fails if a destination appeared meanwhile.
        os.link(temporary, target)
        return True
    finally:
        if temporary is not None:
            if temporary.exists():
                temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--restore-evidence", action="store_true", help="Restore indexed .json.gz originals, refusing to replace different files")
    args = parser.parse_args()
    root = args.root.resolve()
    manifest = json.loads((root / "artifacts/beam_assisted/publication_manifest.json").read_text())
    checked = set()
    for row in manifest["files"]:
        if row["path"] in checked:
            raise ValueError("Duplicate publication path: " + row["path"])
        checked.add(row["path"])
        if identity(local_path(root, row["path"])) != (row["bytes"], row["sha256"]):
            raise ValueError("Missing or changed publication file: " + row["path"])
    restored = 0
    if args.restore_evidence:
        index = json.loads((root / "artifacts/beam_assisted/evidence_index.json").read_text())
        restored = sum(restore_one(root, row) for row in index["files"])
    print(json.dumps({"passed": True, "files_verified": len(checked), "evidence_restored": restored, "native_started": False}))


if __name__ == "__main__":
    main()
