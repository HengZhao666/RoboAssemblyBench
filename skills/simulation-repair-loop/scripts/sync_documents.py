#!/usr/bin/env python3
"""Explicit document bundle: pack, atomically copy, then verify exact destinations.

No subprocesses, source discovery, simulation, or implicit destinations. The caller
owns the file list and approved roots. A failed copy is not a successful sync.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import tarfile
import tempfile


def digest(data):
    return hashlib.sha256(data).hexdigest()


def relative(value):
    if not isinstance(value, str) or not value:
        raise ValueError("Relative path must be a nonempty string")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or str(path) != value:
        raise ValueError("Unsafe or noncanonical relative path")
    if value == "bundle.json":
        raise ValueError("Reserved bundle member")
    return value


def destination(root, name):
    root = Path(root)
    if not root.is_absolute() or not root.is_dir():
        raise ValueError("Each destination root must be an existing absolute directory")
    target = root / relative(name)
    if not target.resolve().is_relative_to(root.resolve()):
        raise ValueError("Destination escapes explicit root")
    if target.is_symlink() or any(parent.is_symlink() for parent in target.parents if parent != root and root in parent.parents):
        raise ValueError("Symlink destination refused")
    if target.exists() and not target.is_file():
        raise ValueError("Destination is not a regular file")
    return target


def atomic_write(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".docs-sync-", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def pack(base, specification, output):
    names = json.loads(Path(specification).read_text())["files"]
    if not names or len(names) != len(set(names)):
        raise ValueError("Empty or duplicate document list")
    payload = {}
    for name in names:
        payload[relative(name)] = destination(base, name).read_bytes()
    manifest = {name: {"sha256": digest(data), "bytes": len(data)} for name, data in payload.items()}
    with tarfile.open(output, "w:gz") as archive:
        for name, data in {"bundle.json": json.dumps(manifest, sort_keys=True).encode(), **payload}.items():
            member = tarfile.TarInfo(name)
            member.size = len(data)
            member.mode = 0o644
            archive.addfile(member, io.BytesIO(data))
    return {"packed_files": len(payload), "bundle_sha256": digest(Path(output).read_bytes())}


def unpack(bundle):
    entries = {}
    with tarfile.open(bundle, "r:gz") as archive:
        for member in archive:
            if not member.isfile() or member.name in entries:
                raise ValueError("Non-file or duplicate archive member")
            if member.name != "bundle.json":
                relative(member.name)
            entries[member.name] = archive.extractfile(member).read()
    manifest = json.loads(entries.pop("bundle.json"))
    if not manifest or set(manifest) != set(entries):
        raise ValueError("Bundle member coverage mismatch")
    for name, data in entries.items():
        if manifest[name] != {"sha256": digest(data), "bytes": len(data)}:
            raise ValueError("Bundle hash/size mismatch: " + name)
    return manifest, entries


def apply(bundle, roots, report):
    manifest, entries = unpack(bundle)
    if len(roots) != len(set(roots)) or len(roots) != len({str(Path(root).resolve()) for root in roots}):
        raise ValueError("Duplicate destination roots")
    targets = [(destination(root, name), name, data) for root in roots for name, data in entries.items()]
    rows = []
    for target, name, data in targets:
        atomic_write(target, data)
        actual = target.read_bytes()
        if digest(actual) != manifest[name]["sha256"] or len(actual) != manifest[name]["bytes"]:
            raise ValueError("Written document differs: " + str(target))
        rows.append({"path": str(target), **manifest[name]})
    result = {"passed": True, "bundle_sha256": digest(Path(bundle).read_bytes()),
              "roots": roots, "copies": len(rows), "files": rows,
              "native_started": False, "scope": "Explicit document files only; no control source deployment"}
    atomic_write(report, (json.dumps(result, ensure_ascii=False, indent=2) + "\n").encode())
    return {key: value for key, value in result.items() if key != "files"}


def verify(bundle, roots, report):
    manifest, _ = unpack(bundle)
    expected = {str(Path(root) / name): binding for root in roots for name, binding in manifest.items()}
    actual = json.loads(Path(report).read_text())
    if actual.get("passed") is not True or actual.get("bundle_sha256") != digest(Path(bundle).read_bytes()) or actual.get("roots") != roots:
        raise ValueError("Report status, bundle identity, or destination roots differ")
    rows = actual["files"]
    if len(rows) != len(expected) or actual.get("copies") != len(expected):
        raise ValueError("Missing/duplicate destination coverage")
    if len({row["path"] for row in rows}) != len(rows):
        raise ValueError("Duplicate report destination")
    for row in rows:
        if expected.get(row["path"]) != {"sha256": row["sha256"], "bytes": row["bytes"]}:
            raise ValueError("Exact destination/hash/size mismatch")
    return {"passed": True, "copies_verified": len(expected), "bundle_sha256": actual["bundle_sha256"], "native_started": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    build = sub.add_parser("pack")
    build.add_argument("--base", required=True)
    build.add_argument("--spec", required=True)
    build.add_argument("--output", required=True)
    for command in ("apply", "verify"):
        child = sub.add_parser(command)
        child.add_argument("--bundle", required=True)
        child.add_argument("--root", action="append", required=True)
        child.add_argument("--report", required=True)
    args = parser.parse_args()
    if args.command == "pack":
        result = pack(args.base, args.spec, args.output)
    else:
        result = globals()[args.command](args.bundle, args.root, args.report)
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
