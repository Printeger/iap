#!/usr/bin/env python3
"""Repository-local hermetic entry point for P4-G0C Python tests."""

from __future__ import annotations

import argparse
import os
import stat
import sys
from pathlib import Path
from typing import Sequence


ENVIRONMENT_PATHS = {
    "HOME": "home",
    "ROS_HOME": "ros_home",
    "ROS_LOG_DIR": "ros_logs",
    "TMPDIR": "tmp",
    "XDG_RUNTIME_DIR": "xdg_runtime",
}
HERMETIC_ROOT_ENVIRONMENT = "P4_G0C_HERMETIC_TEST_ROOT"
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
ALLOWED_RESULTS_ROOT = REPOSITORY_ROOT / "results" / "icra27"


class HermeticTestEnvironmentError(RuntimeError):
    """Raised before ROS imports when the test environment is not hermetic."""


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


def _strict_descendant(path: Path, root: Path) -> bool:
    try:
        relative = path.relative_to(root)
    except ValueError:
        return False
    return bool(relative.parts)


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
    if not _strict_descendant(root, ALLOWED_RESULTS_ROOT):
        raise HermeticTestEnvironmentError("task_root:outside_allowed_results")
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


def _bootstrap_environment(task_root: object) -> dict[str, str]:
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
    # HOME is intentionally task-local; keep read-only git provenance usable
    # without consulting or writing a user-global configuration file.
    environment["GIT_CONFIG_COUNT"] = "1"
    environment["GIT_CONFIG_KEY_0"] = "safe.directory"
    environment["GIT_CONFIG_VALUE_0"] = str(REPOSITORY_ROOT)
    return environment


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-root", required=True)
    parser.add_argument("unittest_args", nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    unittest_args = list(args.unittest_args)
    if unittest_args[:1] == ["--"]:
        unittest_args.pop(0)
    if not unittest_args:
        parser.error("unittest arguments are required after --")
    try:
        environment = _bootstrap_environment(args.task_root)
    except (HermeticTestEnvironmentError, OSError) as exc:
        print(
            f"HERMETIC_TEST_ENVIRONMENT_NOT_READY:{exc}",
            file=sys.stderr,
        )
        return 2
    summary = " ".join(
        f"{name}={environment[name]}" for name in ENVIRONMENT_PATHS
    )
    print(
        "HERMETIC_TEST_ENVIRONMENT_READY "
        f"task_root={environment[HERMETIC_ROOT_ENVIRONMENT]} {summary}",
        flush=True,
    )
    command = [sys.executable, "-m", "unittest", *unittest_args]
    os.execve(sys.executable, command, environment)
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
