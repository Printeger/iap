"""Resolve one collision-safe artifact directory for a canonical IAP run."""

from __future__ import annotations

import os
import re
import uuid
import warnings
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
        return package_root / "log"

    state_root = os.environ.get("XDG_STATE_HOME", "").strip()
    state_base = Path(state_root).expanduser() if state_root else Path.home() / ".local/state"
    return _validated_absolute(state_base / "iap" / "log", "automatic run root")


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
        warnings.warn(
            "output_dir is deprecated; configure IAP_RUN_ROOT instead",
            DeprecationWarning,
            stacklevel=2,
        )
        run_dir = _validated_absolute(Path(requested), "output_dir")
        run_dir.parent.mkdir(parents=True, exist_ok=True)
        try:
            run_dir.mkdir()
        except FileExistsError as error:
            raise RuntimeError(
                f"output_dir already exists; choose a new directory: {run_dir}"
            ) from error
        _create_layout(run_dir)
        return run_dir.resolve()

    _component(entrypoint, "entrypoint")
    if str(scenario).strip():
        _component(scenario, "scenario")
    parent = _default_run_root()
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
        _create_layout(run_dir)
        _update_latest(parent, run_dir)
        return run_dir
    raise RuntimeError(f"could not allocate a unique run directory below {parent}")


def _create_layout(run_dir: Path) -> None:
    for category in ("runtime", "profiling", "export", "metadata"):
        (run_dir / category).mkdir(parents=False, exist_ok=True)
    (run_dir / "runtime" / "ros").mkdir(exist_ok=True)
    for namespace in (
        "glio",
        "current_integrity",
        "advisory",
        "planner",
        "simulation",
        "capture",
        "analysis",
    ):
        (run_dir / "export" / namespace).mkdir(exist_ok=True)
    for namespace in ("config", "processes", "manifests"):
        (run_dir / "metadata" / namespace).mkdir(exist_ok=True)


def adopt_run_directory(requested_run_dir: str) -> Path:
    """Validate and adopt a run allocated by an outer canonical launch."""

    run_dir = _validated_absolute(Path(str(requested_run_dir).strip()), "run_dir")
    if not run_dir.is_dir():
        raise RuntimeError(f"run_dir must name an existing directory: {run_dir}")
    _create_layout(run_dir)
    return run_dir
