from __future__ import annotations

import json
import os
import socket
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any

OWNER_FILE = 'owner.json'


def _process_start_ticks(pid: int) -> int | None:
    """Return Linux process start ticks, or ``None`` when the process is gone."""
    try:
        fields = (Path(f'/proc/{int(pid)}/stat').read_text(encoding='utf-8')).split()
        return int(fields[21])
    except (FileNotFoundError, OSError, IndexError, ValueError):
        return None


def _process_start_unix(pid: int) -> float | None:
    start_ticks = _process_start_ticks(pid)
    if start_ticks is None:
        return None
    try:
        boot_time = next(
            float(line.split()[1])
            for line in Path('/proc/stat').read_text(encoding='utf-8').splitlines()
            if line.startswith('btime ')
        )
        return boot_time + start_ticks / float(os.sysconf('SC_CLK_TCK'))
    except (OSError, StopIteration, IndexError, ValueError, TypeError):
        return None


def _load_owner(lock_dir: Path) -> dict[str, Any] | None:
    try:
        return json.loads((lock_dir / OWNER_FILE).read_text(encoding='utf-8'))
    except (FileNotFoundError, OSError, TypeError, ValueError):
        return None


def _pid_is_alive(pid: int) -> bool:
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _owner_process_is_same(owner: dict[str, Any]) -> bool:
    """Reject a lock whose PID was recycled after the lock was created."""
    try:
        pid = int(owner['pid'])
    except (KeyError, TypeError, ValueError):
        return False
    if not _pid_is_alive(pid):
        return False

    recorded_ticks = owner.get('proc_start_ticks')
    if recorded_ticks is not None:
        try:
            return _process_start_ticks(pid) == int(recorded_ticks)
        except (TypeError, ValueError):
            return False

    # Legacy locks did not record start ticks.  Compare the live process start
    # time with the lock creation time so a recycled PID is still reclaimable.
    created_at = owner.get('created_at_unix')
    process_started_at = _process_start_unix(pid)
    if created_at is not None and process_started_at is not None:
        try:
            return process_started_at <= float(created_at) + 2.0
        except (TypeError, ValueError):
            return False
    return True


def process_lock_is_held(lock_dir: Path) -> bool:
    if not lock_dir.is_dir():
        return False
    owner = _load_owner(lock_dir)
    if owner is None:
        return True
    if str(owner.get('hostname') or '') != socket.gethostname():
        return True
    return _owner_process_is_same(owner)


def _remove_owned_lock(lock_dir: Path, token: str) -> None:
    owner = _load_owner(lock_dir)
    if owner is None or str(owner.get('token') or '') != token:
        return
    try:
        (lock_dir / OWNER_FILE).unlink()
        lock_dir.rmdir()
    except FileNotFoundError:
        pass


def _discard_stale_lock(lock_dir: Path) -> None:
    stale_path = lock_dir.with_name(f'{lock_dir.name}.stale.{int(time.time())}.{os.getpid()}.{uuid.uuid4().hex[:8]}')
    try:
        lock_dir.rename(stale_path)
    except FileNotFoundError:
        return
    try:
        (stale_path / OWNER_FILE).unlink(missing_ok=True)
        stale_path.rmdir()
    except OSError:
        pass


@contextmanager
def exclusive_process_lock(lock_dir: Path, *, description: str):
    lock_dir = Path(lock_dir)
    lock_dir.parent.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    acquired = False
    for _ in range(5):
        try:
            lock_dir.mkdir()
        except FileExistsError as exc:
            if process_lock_is_held(lock_dir):
                owner = _load_owner(lock_dir)
                raise RuntimeError(f'Another {description} holds {lock_dir}; owner={owner or "unknown"}.') from exc
            _discard_stale_lock(lock_dir)
            continue
        owner = {
            'schema_version': 'roboassemblybench_process_lock_v1',
            'description': description,
            'pid': os.getpid(),
            'hostname': socket.gethostname(),
            'token': token,
            'proc_start_ticks': _process_start_ticks(os.getpid()),
            'created_at_unix': time.time(),
        }
        (lock_dir / OWNER_FILE).write_text(json.dumps(owner, indent=2), encoding='utf-8')
        acquired = True
        break
    if not acquired:
        raise RuntimeError(f'Could not acquire {description} lock at {lock_dir}.')
    try:
        yield
    finally:
        _remove_owned_lock(lock_dir, token)
