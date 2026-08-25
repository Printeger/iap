"""Immutable Git-object fixtures for historical P4-r6 tests only."""

import hashlib
import subprocess
from pathlib import Path


FROZEN_P4_R6_COMMIT = "564dd6ad8c864f496b63a1b09afd3febe31eef21"
FROZEN_P4_R6_LAUNCH_PATH = "launch/test_planner.launch.py"
FROZEN_P4_R6_LAUNCH_SHA256 = (
    "24f34c6a9d84119c2963819aa77f2f620f906dd344f2179dbab68e4e43044595"
)


def frozen_p4_r6_launch_bytes(repository: Path) -> bytes:
    """Read and verify the launch bytes registered by the historical r6 manifest."""
    repository = Path(repository).resolve()
    try:
        payload = subprocess.check_output(
            [
                "git", "show",
                f"{FROZEN_P4_R6_COMMIT}:{FROZEN_P4_R6_LAUNCH_PATH}",
            ],
            cwd=repository,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError("historical P4-r6 launch Git object unavailable") from exc
    actual = hashlib.sha256(payload).hexdigest()
    if actual != FROZEN_P4_R6_LAUNCH_SHA256:
        raise RuntimeError(
            f"historical P4-r6 launch SHA mismatch: {actual}"
        )
    return payload


def materialize_p4_r6_test_install(repository: Path, root: Path) -> Path:
    """Create a test-local historical install root with the exact r6 launch."""
    install = Path(root).resolve() / "historical-p4-r6-install"
    if install.exists() or install.is_symlink():
        raise RuntimeError("historical P4-r6 test install identity already exists")
    launch = install / "share/iap/launch/test_planner.launch.py"
    launch.parent.mkdir(parents=True)
    launch.write_bytes(frozen_p4_r6_launch_bytes(repository))
    actual = hashlib.sha256(launch.read_bytes()).hexdigest()
    if actual != FROZEN_P4_R6_LAUNCH_SHA256 or install.is_symlink():
        raise RuntimeError("historical P4-r6 test install materialization failed")
    return install
