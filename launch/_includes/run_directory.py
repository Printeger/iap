"""Resolve one collision-safe artifact directory for a canonical IAP run."""

from __future__ import annotations

import os
import re
import json
import socket
import subprocess
import uuid
import warnings
from datetime import datetime, timezone
from pathlib import Path

import fcntl


_SAFE_COMPONENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_MODULES = {
    "glio": ["GLIO"],
    "glio_integrity": ["GLIO", "Current Integrity Monitor"],
    "iap_sim": [
        "GLIO",
        "Current Integrity Monitor",
        "Advisory Integrity Evaluator",
        "Safety-aware planner",
    ],
    "iap_flight": [
        "GLIO",
        "Current Integrity Monitor",
        "Advisory Integrity Evaluator",
        "Safety-aware planner",
    ],
}


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
        run_dir = run_dir.resolve()
        _write_run_manifest(run_dir, entrypoint=entrypoint, scenario=scenario)
        return run_dir

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
        _write_run_manifest(run_dir, entrypoint=entrypoint, scenario=scenario)
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


def _git_value(*arguments: str) -> str:
    package_root = Path(__file__).resolve().parents[2]
    try:
        return subprocess.run(
            ["git", "-C", str(package_root), *arguments],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _atomic_write_json(path: Path, value: dict) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary, path)


def _write_run_manifest(run_dir: Path, *, entrypoint: str, scenario: str) -> None:
    now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    commit = _git_value("rev-parse", "HEAD")
    dirty = bool(_git_value("status", "--porcelain"))
    manifest = {
        "schema_version": "iap_run_artifact_v1",
        "run_id": run_dir.name,
        "run_root": str(run_dir.parent),
        "run_dir": str(run_dir),
        "entrypoint": entrypoint,
        "scenario": str(scenario).strip() or None,
        "modules": _MODULES.get(entrypoint, []),
        "started_at_utc": now,
        "ended_at_utc": None,
        "lifecycle": "active",
        "safety_outcome": "not_applicable" if entrypoint.startswith("glio") else "unknown",
        "run_class": "development",
        "retention_class": "ordinary",
        "source": {"git_commit": commit or None, "git_worktree_clean": not dirty},
        "build": {},
        "host": {"hostname": socket.gethostname()},
        "config_snapshots": [],
        "subordinate_manifests": [],
        "external_exports": [],
    }
    _atomic_write_json(run_dir / "metadata" / "run_manifest.json", manifest)


def write_subordinate_manifest(run_dir: Path, name: str, value: dict) -> Path:
    safe_name = _component(name, "manifest name")
    path = run_dir / "metadata" / "manifests" / f"{safe_name}.json"
    _atomic_write_json(path, value)
    register_subordinate_manifest(run_dir, path)
    return path


def register_subordinate_manifest(run_dir: Path, path: Path) -> None:
    manifest_path = run_dir / "metadata" / "run_manifest.json"
    if not manifest_path.is_file():
        return
    relative = path.resolve().relative_to(run_dir.resolve()).as_posix()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    references = manifest.setdefault("subordinate_manifests", [])
    if relative not in references:
        references.append(relative)
        references.sort()
        _atomic_write_json(manifest_path, manifest)
