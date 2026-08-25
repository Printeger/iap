#!/usr/bin/env python3
"""Controlled hermetic verification entry point for P4-G0C."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence


ENVIRONMENT_PATHS = {
    "HOME": "home",
    "ROS_HOME": "ros_home",
    "ROS_LOG_DIR": "ros_logs",
    "TMPDIR": "tmp",
    "XDG_RUNTIME_DIR": "xdg_runtime",
}
HERMETIC_ROOT_ENVIRONMENT = "P4_G0C_HERMETIC_TEST_ROOT"
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ALLOWED_RESULTS_ROOT = REPOSITORY_ROOT / "results" / "icra27" / "icra057"
EXTERNAL_ROS_LOG_ROOT = Path("/root/.ros/log")
EXTERNAL_DELTA_EXIT = 86


class HermeticTestEnvironmentError(RuntimeError):
    """Raised before Python tools run when their environment is not hermetic."""


class ExternalLogInventoryError(RuntimeError):
    """Raised when the external ROS log tree cannot be inventoried exactly."""


def _canonical_absolute_path(raw: object, label: str) -> Path:
    if not isinstance(raw, (str, os.PathLike)) or not str(raw):
        raise HermeticTestEnvironmentError(f"{label}:missing_or_wrong_type")
    text = os.fspath(raw)
    path = Path(text)
    if not path.is_absolute():
        raise HermeticTestEnvironmentError(f"{label}:not_absolute")
    if ".." in path.parts:
        raise HermeticTestEnvironmentError(f"{label}:lexical_parent")
    if text != os.path.normpath(text):
        raise HermeticTestEnvironmentError(f"{label}:lexical_alias")
    if path.resolve(strict=False) != path:
        raise HermeticTestEnvironmentError(f"{label}:canonical_alias")
    return path


def _within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _require_no_symlink_chain(path: Path) -> None:
    current = REPOSITORY_ROOT
    try:
        relative = path.relative_to(REPOSITORY_ROOT)
    except ValueError as exc:
        raise HermeticTestEnvironmentError("task_root:outside_repository") from exc
    for part in relative.parts:
        current = current / part
        if current.is_symlink():
            raise HermeticTestEnvironmentError(
                f"task_root:symlink_component:{current}"
            )


def _validate_task_root(task_root: object, *, create: bool) -> Path:
    root = _canonical_absolute_path(task_root, "task_root")
    if not _within(root, ALLOWED_RESULTS_ROOT):
        raise HermeticTestEnvironmentError("task_root:outside_icra057")
    _require_no_symlink_chain(root)
    if root.exists():
        if root.is_symlink() or not root.is_dir():
            raise HermeticTestEnvironmentError("task_root:wrong_type")
    elif create:
        if not root.parent.is_dir() or root.parent.is_symlink():
            raise HermeticTestEnvironmentError("task_root:parent_not_ready")
        root.mkdir(mode=0o700)
    else:
        raise HermeticTestEnvironmentError("task_root:missing")
    metadata = root.stat()
    if metadata.st_uid != os.geteuid() or not os.access(root, os.W_OK | os.X_OK):
        raise HermeticTestEnvironmentError("task_root:owner_or_access")
    return root


def _validate_environment_directory(
    path: Path, name: str, *, exact_mode: int | None = None
) -> None:
    if path.is_symlink() or not path.is_dir():
        raise HermeticTestEnvironmentError(f"{name}:wrong_type_or_symlink")
    metadata = path.stat()
    if metadata.st_uid != os.geteuid() or not os.access(path, os.W_OK | os.X_OK):
        raise HermeticTestEnvironmentError(f"{name}:owner_or_access")
    if exact_mode is not None and stat.S_IMODE(metadata.st_mode) != exact_mode:
        raise HermeticTestEnvironmentError(f"{name}:unsafe_mode")


def require_hermetic_test_environment() -> Path:
    """Guard the launch-test import boundary without mutating the filesystem."""
    root = _validate_task_root(
        _canonical_absolute_path(
            os.environ.get(HERMETIC_ROOT_ENVIRONMENT), HERMETIC_ROOT_ENVIRONMENT
        ),
        create=False,
    )
    for name, relative in ENVIRONMENT_PATHS.items():
        expected = root / relative
        actual = _canonical_absolute_path(os.environ.get(name), name)
        if actual != expected:
            raise HermeticTestEnvironmentError(f"{name}:binding_mismatch")
        _validate_environment_directory(
            actual, name, exact_mode=0o700 if name == "XDG_RUNTIME_DIR" else None
        )
    return root


def _bootstrap_environment(task_root: object) -> tuple[Path, dict[str, str]]:
    task_root = _validate_task_root(task_root, create=True)
    environment = dict(os.environ)
    environment[HERMETIC_ROOT_ENVIRONMENT] = str(task_root)
    open_flags = os.O_RDONLY | os.O_DIRECTORY
    if hasattr(os, "O_NOFOLLOW"):
        open_flags |= os.O_NOFOLLOW
    root_fd = os.open(task_root, open_flags)
    try:
        for name, relative in ENVIRONMENT_PATHS.items():
            path = task_root / relative
            if path.exists() or path.is_symlink():
                _validate_environment_directory(
                    path,
                    name,
                    exact_mode=0o700 if name == "XDG_RUNTIME_DIR" else None,
                )
            else:
                mode = 0o700 if name == "XDG_RUNTIME_DIR" else 0o755
                os.mkdir(relative, mode=mode, dir_fd=root_fd)
                directory_fd = os.open(relative, open_flags, dir_fd=root_fd)
                try:
                    if name == "XDG_RUNTIME_DIR":
                        os.fchmod(directory_fd, 0o700)
                finally:
                    os.close(directory_fd)
                _validate_environment_directory(
                    path,
                    name,
                    exact_mode=0o700 if name == "XDG_RUNTIME_DIR" else None,
                )
            environment[name] = str(path)
    finally:
        os.close(root_fd)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["GIT_CONFIG_COUNT"] = "1"
    environment["GIT_CONFIG_KEY_0"] = "safe.directory"
    environment["GIT_CONFIG_VALUE_0"] = str(REPOSITORY_ROOT)
    return task_root, environment


def _file_type(mode: int) -> str:
    if stat.S_ISREG(mode):
        return "regular"
    if stat.S_ISDIR(mode):
        return "directory"
    if stat.S_ISLNK(mode):
        return "symlink"
    if stat.S_ISCHR(mode):
        return "character"
    if stat.S_ISBLK(mode):
        return "block"
    if stat.S_ISFIFO(mode):
        return "fifo"
    if stat.S_ISSOCK(mode):
        return "socket"
    return "unknown"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def external_log_inventory(
    root: Path = EXTERNAL_ROS_LOG_ROOT,
) -> dict[str, dict[str, object]]:
    """Return an atime-neutral logical inventory of an external tree."""
    if not root.exists() and not root.is_symlink():
        return {".": {"type": "missing"}}
    inventory: dict[str, dict[str, object]] = {}
    pending = [root]
    try:
        while pending:
            path = pending.pop()
            metadata = path.lstat()
            relative = "." if path == root else str(path.relative_to(root))
            kind = _file_type(metadata.st_mode)
            entry: dict[str, object] = {
                "type": kind,
                "mode": stat.S_IMODE(metadata.st_mode),
                "uid": metadata.st_uid,
                "gid": metadata.st_gid,
                "size": metadata.st_size,
                "mtime_ns": metadata.st_mtime_ns,
                "ctime_ns": metadata.st_ctime_ns,
                "symlink_target": os.readlink(path) if kind == "symlink" else None,
                "sha256": _sha256(path) if kind == "regular" else None,
            }
            inventory[relative] = entry
            if kind == "directory":
                with os.scandir(path) as entries:
                    children = [Path(item.path) for item in entries]
                pending.extend(sorted(children, key=str, reverse=True))
    except OSError as exc:
        raise ExternalLogInventoryError(f"inventory_failed:{path}:{exc}") from exc
    return dict(sorted(inventory.items()))


def compare_inventories(
    before: Mapping[str, Mapping[str, object]],
    after: Mapping[str, Mapping[str, object]],
) -> list[dict[str, object]]:
    """Return deterministic added, removed and changed inventory records."""
    delta: list[dict[str, object]] = []
    for path in sorted(set(before) | set(after)):
        if path not in before:
            delta.append({"path": path, "change": "added", "after": after[path]})
        elif path not in after:
            delta.append({"path": path, "change": "removed", "before": before[path]})
        elif before[path] != after[path]:
            changed_fields = sorted(
                key
                for key in set(before[path]) | set(after[path])
                if before[path].get(key) != after[path].get(key)
            )
            delta.append({
                "path": path,
                "change": "changed",
                "fields": changed_fields,
                "before": before[path],
                "after": after[path],
            })
    return delta


def _write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n"
    )


def _repository_files(raw_paths: Sequence[str]) -> list[str]:
    if not raw_paths:
        raise HermeticTestEnvironmentError("verification_files:missing")
    validated: list[str] = []
    for raw in raw_paths:
        lexical = Path(raw)
        if ".." in lexical.parts:
            raise HermeticTestEnvironmentError("verification_file:lexical_parent")
        candidate = lexical if lexical.is_absolute() else REPOSITORY_ROOT / lexical
        resolved = candidate.resolve(strict=True)
        if not _within(resolved, REPOSITORY_ROOT) or not resolved.is_file():
            raise HermeticTestEnvironmentError(
                f"verification_file:outside_or_wrong_type:{raw}"
            )
        validated.append(str(resolved))
    return validated


def _controlled_command(mode: str, arguments: Sequence[str]) -> list[str]:
    args = list(arguments)
    if args[:1] == ["--"]:
        args.pop(0)
    if mode == "unittest":
        if not args:
            raise HermeticTestEnvironmentError("unittest:arguments_missing")
        return [sys.executable, "-m", "unittest", *args]
    files = _repository_files(args)
    if mode == "syntax":
        program = (
            "from pathlib import Path; import sys; "
            "[(compile(Path(p).read_bytes(), p, 'exec')) for p in sys.argv[1:]]"
        )
        return [sys.executable, "-c", program, *files]
    if mode == "flake8":
        return [
            sys.executable, "-m", "flake8", "--select=E9,F63,F7,F82", *files
        ]
    if mode == "canonical-json":
        program = (
            "import json,sys; from pathlib import Path; "
            "bad=[]; "
            "[(bad.append(p) if Path(p).read_text() != "
            "json.dumps(json.loads(Path(p).read_text()),sort_keys=True,"
            "separators=(',',':'))+'\\n' else None) for p in sys.argv[1:]]; "
            "print('NON_CANONICAL_JSON:'+','.join(bad)) if bad else None; "
            "raise SystemExit(bool(bad))"
        )
        return [sys.executable, "-c", program, *files]
    raise HermeticTestEnvironmentError(f"verification_mode:unknown:{mode}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-root", required=True)
    parser.add_argument("mode", choices=(
        "unittest", "syntax", "flake8", "canonical-json",
    ))
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    try:
        task_root, environment = _bootstrap_environment(args.task_root)
        command = _controlled_command(args.mode, args.arguments)
        before = external_log_inventory()
    except (HermeticTestEnvironmentError, ExternalLogInventoryError, OSError) as exc:
        print(f"HERMETIC_VERIFICATION_NOT_READY:{exc}", file=sys.stderr)
        return 2

    evidence_dir = task_root / "external_inventory"
    evidence_dir.mkdir(mode=0o700, exist_ok=True)
    invocation = f"{args.mode}-{os.getpid()}"
    before_path = evidence_dir / f"{invocation}-before.json"
    after_path = evidence_dir / f"{invocation}-after.json"
    result_path = evidence_dir / f"{invocation}-result.json"
    _write_json(before_path, before)
    summary = " ".join(
        f"{name}={environment[name]}" for name in ENVIRONMENT_PATHS
    )
    print(
        "HERMETIC_VERIFICATION_READY "
        f"mode={args.mode} task_root={task_root} {summary}",
        flush=True,
    )
    completed = subprocess.run(command, env=environment, check=False)
    try:
        after = external_log_inventory()
        delta = compare_inventories(before, after)
    except ExternalLogInventoryError as exc:
        _write_json(result_path, {
            "child_exit": completed.returncode,
            "command": command,
            "external_inventory_error": str(exc),
            "mode": args.mode,
            "result": "EXTERNAL_ROS_LOG_INVENTORY_FAILED",
        })
        print(f"EXTERNAL_ROS_LOG_INVENTORY_FAILED:{exc}", file=sys.stderr)
        return 87
    _write_json(after_path, after)
    final_exit = EXTERNAL_DELTA_EXIT if delta else completed.returncode
    result = {
        "child_exit": completed.returncode,
        "command": command,
        "external_delta": delta,
        "external_inventory_count": len(after),
        "final_exit": final_exit,
        "mode": args.mode,
        "result": (
            "BLOCKED_EXTERNAL_ROS_LOG_MUTATION" if delta
            else "CHILD_COMPLETE" if completed.returncode == 0
            else "CHILD_FAILED"
        ),
    }
    _write_json(result_path, result)
    if delta:
        print(
            f"BLOCKED_EXTERNAL_ROS_LOG_MUTATION:changes={len(delta)} "
            f"evidence={result_path}",
            file=sys.stderr,
        )
    else:
        print(
            "EXTERNAL_ROS_LOG_UNCHANGED "
            f"entries={len(after)} child_exit={completed.returncode} "
            f"evidence={result_path}",
            flush=True,
        )
    return final_exit


if __name__ == "__main__":
    raise SystemExit(main())
