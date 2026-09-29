"""Resolve one collision-safe artifact directory for a canonical IAP run."""

from __future__ import annotations

import os
import re
import uuid
from datetime import datetime, timezone
from pathlib import Path

import fcntl


_SAFE_COMPONENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _validated_absolute(path: Path, label: str) -> Path:
    path = path.expanduser()
    if not path.is_absolute():
        raise RuntimeError(f"{label} must be an absolute path")
    path = path.resolve()
    if path == Path("/"):
        raise RuntimeError(f"{label} cannot be filesystem root")
    return path


def _component(value: str, label: str) -> str:
    value = str(value).strip()
    if not _SAFE_COMPONENT.fullmatch(value):
        raise RuntimeError(f"{label} is not a safe run-directory component: {value!r}")
    return value


def _default_run_root() -> Path:
    configured = os.environ.get("IAP_RUN_ROOT", "").strip()
    if configured:
        return _validated_absolute(Path(configured), "IAP_RUN_ROOT")

    # In a source or --symlink-install workspace, keep ordinary development
    # runs in the repository's ignored log tree. A copied package share also
    # has package.xml, so require source-only build files before selecting it.
    package_root = Path(__file__).resolve().parents[2]
    if (package_root / "CMakeLists.txt").is_file() and (package_root / "src").is_dir():
        return package_root / "log" / "runs"

    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    state_base = Path(state_root).expanduser() if state_root else Path.home() / ".local/state"
    return _validated_absolute(state_base / "iap" / "runs", "automatic run root")


def _new_run_id() -> str:
    now = datetime.now(timezone.utc)
    timestamp = now.strftime("%Y%m%dT%H%M%SZ")
    return f"{timestamp}_{now.microsecond // 1000:03d}"


def _update_latest(parent: Path, run_dir: Path) -> None:
    latest = parent / "latest"
    lock_path = parent / ".latest.lock"
    with lock_path.open("a", encoding="utf-8") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        if latest.exists() and not latest.is_symlink():
            raise RuntimeError(f"automatic run latest path is not a symlink: {latest}")
        if latest.is_symlink():
            current_name = Path(os.readlink(latest)).name
            if current_name >= run_dir.name:
                return

        temporary = parent / f".latest.{os.getpid()}.{uuid.uuid4().hex}"
        try:
            temporary.symlink_to(run_dir.name, target_is_directory=True)
            os.replace(temporary, latest)
        finally:
            if temporary.is_symlink():
                temporary.unlink()


def resolve_run_directory(
    requested_output_dir: str,
    *,
    entrypoint: str,
    scenario: str = "",
) -> Path:
    """Return an explicit directory or atomically allocate a timestamped one."""

    requested = str(requested_output_dir).strip()
    if requested:
        run_dir = _validated_absolute(Path(requested), "output_dir")
        run_dir.parent.mkdir(parents=True, exist_ok=True)
        try:
            run_dir.mkdir()
        except FileExistsError as error:
            raise RuntimeError(
                f"output_dir already exists; choose a new directory: {run_dir}"
            ) from error
        return run_dir

    parent = _default_run_root() / _component(entrypoint, "entrypoint")
    if str(scenario).strip():
        parent /= _component(scenario, "scenario")
    parent.mkdir(parents=True, exist_ok=True)

    run_id = _new_run_id()
    for collision in range(100):
        suffix = "" if collision == 0 else f"_{collision:02d}"
        run_dir = parent / f"{run_id}{suffix}"
        try:
            run_dir.mkdir()
        except FileExistsError:
            continue
        run_dir = run_dir.resolve()
        _update_latest(parent, run_dir)
        return run_dir
    raise RuntimeError(f"could not allocate a unique run directory below {parent}")
