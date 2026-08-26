#!/usr/bin/env python3
"""One-shot ICRA-068 P0+P5 live runner and raw-evidence normalizer."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
TASK_ROOT = REPOSITORY / "results/icra27/icra068"
INSTALL_ROOT = TASK_ROOT / "install"
CONTRACT_PATH = REPOSITORY / "config/icra27/icra_p0_p5_qualification_v1.json"
INSTALL_MANIFEST_PATH = TASK_ROOT / "icra068_install_manifest.json"
MIN_FREE_BYTES = 40 * 1024 ** 3
INSTALLED_ALIASES = {
    "share/iap/launch/test_planner.launch.py": "launch/test_planner.launch.py",
    "share/iap/launch/icra_p0_p5_qualification.py": "launch/icra_p0_p5_qualification.py",
    "share/iap/config/icra27/icra_p0_p5_qualification_v1.json": (
        "config/icra27/icra_p0_p5_qualification_v1.json"
    ),
}
LOCAL_PACKAGES = (
    "iap", "bspline_opt", "path_searching", "plan_env", "ego_planner",
    "traj_utils", "cmake_utils", "odom_visualization", "pose_utils",
    "quadrotor_msgs", "uav_utils", "poscmd_2_odom", "gnss_sim",
    "local_sensing", "so3_control", "so3_quadrotor_simulator", "gnss_comm",
)
RUNTIME_EXECUTABLES = {
    "iap": (
        "demo11_corridor_map_publisher", "demo4_lidar_body_bridge",
        "iap_rosnode", "planner_evidence_provenance_publisher.py",
        "test_araim_validator.py", "planner_bag_recorder_with_finalizer.py",
    ),
    "ego_planner": ("ego_planner_node", "traj_server"),
    "gnss_sim": ("gnss_sim_node",),
    "local_sensing": ("pcl_render_node",),
    "odom_visualization": ("odom_visualization",),
    "poscmd_2_odom": ("poscmd_2_odom",),
    "so3_quadrotor_simulator": ("so3_quadrotor_simulator",),
}
RUNTIME_LIBRARY_ROOTS = (
    "lib/libglobal_mapping.so", "lib/libgnss_extension.so",
    "lib/libintegrity_extension.so", "lib/libodometry_estimation_gpu.so",
    "lib/libsim_extension.so", "lib/libsub_mapping.so",
    "lib/libso3_control_component.so",
)
RUNTIME_CONFIGS = (
    "share/iap/config/config_odometry_gpu.json",
    "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml",
    "share/iap/config/sim_demo11/config.json",
    "share/iap/config/sim_demo11/config_gnss.json",
    "share/iap/config/sim_demo11/config_ros.json",
    "share/iap/config/sim_ego/config_global_mapping_gpu.json",
    "share/iap/config/sim_ego/config_logging.json",
    "share/iap/config/sim_ego/config_sensors.json",
    "share/iap/config/sim_ego/config_sub_mapping_gpu.json",
    "share/iap/config/sim_ego/config_viewer.json",
    "share/iap/config/sim_ego/fastdds_udp_only.xml",
    "share/local_sensing/config/camera.yaml",
    "share/so3_control/config/corrections_hummingbird.yaml",
    "share/so3_control/config/gains_hummingbird.yaml",
    "share/iap/config/icra27/icra_p0_p5_qualification_v1.json",
)
FORBIDDEN_OVERLAYS = (
    "/home/dev/ws_iap/build", "/home/dev/ws_iap/install",
    "/home/dev/ws_iap/src/iap/build", "/home/dev/ws_iap/src/iap/install",
)
EXPECTED_BUILD_PROFILE = {
    "package_count": len(LOCAL_PACKAGES), "cmake_build_type": "Release",
    "build_testing": False, "build_with_cuda": True,
    "merge_install": True, "symlink_install": False,
}
INSTALL_MANIFEST_KEYS = {
    "schema_version", "git_commit", "install_root", "active_prefixes",
    "packages", "installed_aliases", "runtime_libraries", "file_hashes",
    "linkage_output_sha256", "build_profile", "closure_ready",
}


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


QUALIFICATION = _load(
    "icra_p0_p5_qualification_live",
    REPOSITORY / "launch/icra_p0_p5_qualification.py",
)
GATE_RUNNER = _load(
    "icra_p0_p5_gate_runner",
    REPOSITORY / "scripts/dev_planner/run_gate0_qualification.py",
)
SAFETY_ANALYZER = _load(
    "icra_p0_p5_safety_analyzer",
    REPOSITORY / "scripts/dev_planner/analyze_safety_planner_run.py",
)
LIVE_IDENTITIES = tuple(QUALIFICATION.LIVE_RUN_IDENTITIES.items())


REQUIRED_PROCESSES = {
    "test_planner_corridor_map_publisher": ["test_planner_corridor_map_publisher"],
    "drone_0_pcl_render_node": ["drone_0_pcl_render_node"],
    "test_planner_lidar_body_bridge": ["test_planner_lidar_body_bridge"],
    "test_planner_iap_rosnode": ["test_planner_iap_rosnode"],
    "test_planner_desired_poscmd_to_odom": ["test_planner_desired_poscmd_to_odom"],
    "test_planner_iap_odom_visualization": ["test_planner_iap_odom_visualization"],
    "test_planner_truth_odom_visualization": ["test_planner_truth_odom_visualization"],
    "test_planner_desired_odom_visualization": ["test_planner_desired_odom_visualization"],
    "test_planner_gnss_sim_node": ["test_planner_gnss_sim_node"],
    "drone_0_quadrotor_simulator_so3": ["drone_0_quadrotor_simulator_so3"],
    "test_planner_so3_control_container": ["test_planner_so3_control_container"],
    "drone_0_ego_planner_node": ["drone_0_ego_planner_node"],
    "drone_0_traj_server": ["drone_0_traj_server"],
    "test_planner_evidence_provenance": ["test_planner_evidence_provenance"],
    "test_planner_integrity_validator": ["test_planner_integrity_validator"],
    "test_planner_bag_recorder": ["planner_bag_recorder_with_finalizer.py"],
}


class LiveRunnerError(RuntimeError):
    """The live qualification cannot safely continue."""


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _is_sha256(value: object) -> bool:
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value
    )


def installed_runtime_libraries(install_root: Path) -> tuple[str, ...]:
    """Return every task-local shared library emitted by the proven closure."""
    install_root = Path(install_root)
    return tuple(sorted(
        str(path.relative_to(install_root))
        for path in (install_root / "lib").glob("*.so")
        if path.is_file() and not path.is_symlink()
    ))


def runtime_executable_paths() -> tuple[str, ...]:
    return tuple(
        f"lib/{package}/{executable}"
        for package, executables in RUNTIME_EXECUTABLES.items()
        for executable in executables
    )


def _linkage_ready(relative: str, install_root: Path, environment: dict) -> str:
    completed = subprocess.run(
        ["ldd", str(install_root / relative)], capture_output=True,
        text=True, check=False, env=environment,
    )
    text_output = completed.stdout + completed.stderr
    if completed.returncode != 0 or "not found" in text_output \
            or any(forbidden in text_output for forbidden in FORBIDDEN_OVERLAYS):
        raise LiveRunnerError(f"runtime_linkage_not_ready:{relative}")
    return hashlib.sha256(text_output.encode()).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def verify_installed_aliases(install_root: Path) -> dict[str, dict[str, str]]:
    """Require all current launch/contract aliases to be byte-identical."""
    install_root = Path(install_root).resolve()
    result = {}
    for relative, source_relative in INSTALLED_ALIASES.items():
        installed = install_root / relative
        source = REPOSITORY / source_relative
        if not installed.is_file() or installed.is_symlink():
            raise LiveRunnerError(f"installed_alias_missing_or_symlink:{relative}")
        source_hash = _sha256(source)
        installed_hash = _sha256(installed)
        if source_hash != installed_hash or source.read_bytes() != installed.read_bytes():
            raise LiveRunnerError(f"installed_source_mismatch:{relative}")
        result[relative] = {
            "source_path": source_relative,
            "source_sha256": source_hash,
            "installed_sha256": installed_hash,
        }
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    values = QUALIFICATION.resolve_profile_values(contract, {})
    if not (
        values["p0.enable_risk_grid"] is True
        and values["planner_enable_p5_runtime"] is True
        and values["planner_enable_p5_final"] is True
        and all(values[key] is False for key in (
            "planner_enable_p1", "planner_enable_p2",
            "planner_enable_p3_local", "planner_enable_p3_global",
            "planner_enable_p4",
        ))
    ):
        raise LiveRunnerError("effective_profile_switch_mismatch")
    return result


def audit_and_freeze_install(
    install_root: Path = INSTALL_ROOT,
    output_path: Path = INSTALL_MANIFEST_PATH,
) -> dict:
    """Audit the exact isolated runtime closure and freeze its identity once."""
    install_root = Path(install_root).resolve()
    output_path = Path(output_path).resolve()
    if output_path.exists() or output_path.is_symlink():
        raise LiveRunnerError("install_manifest_already_exists")
    if install_root != INSTALL_ROOT.resolve() or install_root.is_symlink():
        raise LiveRunnerError("install_root_identity_mismatch")
    symlinks = [str(path) for path in install_root.rglob("*") if path.is_symlink()]
    if symlinks:
        raise LiveRunnerError(f"install_contains_symlink:{symlinks[0]}")
    aliases = verify_installed_aliases(install_root)
    git_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPOSITORY, text=True
    ).strip()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=REPOSITORY, text=True,
    ).strip()
    if dirty:
        raise LiveRunnerError("tracked_worktree_not_clean")
    raw_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    active_prefixes = [str(Path(value).resolve()) for value in raw_prefixes if value]
    expected_prefixes = [
        str(install_root), "/root/ros2_ws/install", "/opt/ros/jazzy",
    ]
    if active_prefixes != expected_prefixes:
        raise LiveRunnerError(f"active_prefix_mismatch:{active_prefixes}")
    if any(
        prefix == forbidden or prefix.startswith(forbidden + "/")
        for prefix in active_prefixes for forbidden in FORBIDDEN_OVERLAYS
    ):
        raise LiveRunnerError("forbidden_global_overlay_active")
    from ament_index_python.packages import get_package_prefix

    packages = {}
    for package in (*LOCAL_PACKAGES, "rclcpp_components"):
        resolved = str(Path(get_package_prefix(package)).resolve())
        expected = str(install_root) if package in LOCAL_PACKAGES else "/opt/ros/jazzy"
        if resolved != expected:
            raise LiveRunnerError(f"package_prefix_mismatch:{package}:{resolved}")
        identities = [
            prefix for prefix in active_prefixes
            if (Path(prefix) / "share/ament_index/resource_index/packages" / package).is_file()
        ]
        if identities != [expected]:
            raise LiveRunnerError(f"duplicate_or_missing_package_identity:{package}")
        packages[package] = resolved
    file_hashes = {}
    for relative in runtime_executable_paths():
        path = install_root / relative
        if not path.is_file() or path.is_symlink() or not os.access(path, os.X_OK):
            raise LiveRunnerError(f"runtime_executable_not_ready:{relative}")
        file_hashes[relative] = _sha256(path)
    component = Path("/opt/ros/jazzy/lib/rclcpp_components/component_container")
    if not component.is_file() or not os.access(component, os.X_OK):
        raise LiveRunnerError("component_container_not_ready")
    file_hashes[str(component)] = _sha256(component)
    runtime_libraries = installed_runtime_libraries(install_root)
    if not set(RUNTIME_LIBRARY_ROOTS).issubset(runtime_libraries):
        raise LiveRunnerError("runtime_library_root_missing")
    for relative in (*runtime_libraries, *RUNTIME_CONFIGS):
        path = install_root / relative
        if not path.is_file() or path.is_symlink():
            raise LiveRunnerError(f"runtime_file_not_ready:{relative}")
        file_hashes[relative] = _sha256(path)
    clean_env = dict(os.environ)
    clean_env["LD_LIBRARY_PATH"] = ":".join((
        str(install_root / "lib"), "/root/ros2_ws/install/glim_ros/lib",
        "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
        "/opt/ros/jazzy/lib/x86_64-linux-gnu",
    ))
    linkage = {}
    for relative in runtime_libraries:
        linkage[relative] = _linkage_ready(relative, install_root, clean_env)
    result = {
        "schema_version": "icra068_qualification_install_manifest_v1",
        "git_commit": git_commit,
        "install_root": str(install_root),
        "active_prefixes": active_prefixes,
        "packages": packages,
        "installed_aliases": aliases,
        "runtime_libraries": list(runtime_libraries),
        "file_hashes": file_hashes,
        "linkage_output_sha256": linkage,
        "build_profile": EXPECTED_BUILD_PROFILE,
        "closure_ready": True,
    }
    _write_json(output_path, result)
    return result


def validate_frozen_install_manifest(path: Path, git_commit: str) -> dict:
    """Revalidate the frozen closure without rewriting it before GPU/ROS."""
    path = Path(path).resolve()
    if not path.is_file() or path.is_symlink():
        raise LiveRunnerError("install_manifest_missing_or_symlink")
    try:
        manifest = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveRunnerError("install_manifest_malformed") from exc
    if manifest.get("schema_version") != "icra068_qualification_install_manifest_v1" \
            or manifest.get("closure_ready") is not True \
            or manifest.get("git_commit") != git_commit:
        raise LiveRunnerError("install_manifest_identity_mismatch")
    expected_prefixes = [str(INSTALL_ROOT), "/root/ros2_ws/install", "/opt/ros/jazzy"]
    expected_packages = {
        package: str(INSTALL_ROOT) for package in LOCAL_PACKAGES
    } | {"rclcpp_components": "/opt/ros/jazzy"}
    runtime_libraries = installed_runtime_libraries(INSTALL_ROOT)
    expected_file_keys = set(runtime_executable_paths()) | set(runtime_libraries) \
        | set(RUNTIME_CONFIGS) \
        | {"/opt/ros/jazzy/lib/rclcpp_components/component_container"}
    file_hashes = manifest.get("file_hashes")
    linkage = manifest.get("linkage_output_sha256")
    if (
        set(manifest) != INSTALL_MANIFEST_KEYS
        or manifest.get("install_root") != str(INSTALL_ROOT)
        or manifest.get("active_prefixes") != expected_prefixes
        or manifest.get("active_prefixes") != os.environ.get(
            "AMENT_PREFIX_PATH", ""
        ).split(":")
        or manifest.get("packages") != expected_packages
        or manifest.get("build_profile") != EXPECTED_BUILD_PROFILE
        or manifest.get("runtime_libraries") != list(runtime_libraries)
        or not isinstance(file_hashes, dict)
        or set(file_hashes) != expected_file_keys
        or not all(_is_sha256(value) for value in file_hashes.values())
        or not isinstance(linkage, dict)
        or set(linkage) != set(runtime_libraries)
        or not all(_is_sha256(value) for value in linkage.values())
    ):
        raise LiveRunnerError("install_manifest_inventory_mismatch")
    aliases = verify_installed_aliases(INSTALL_ROOT)
    if manifest.get("installed_aliases") != aliases:
        raise LiveRunnerError("install_manifest_alias_drift")
    for relative, expected_hash in file_hashes.items():
        file_path = Path(relative) if Path(relative).is_absolute() else INSTALL_ROOT / relative
        if not file_path.is_file() or file_path.is_symlink() \
                or _sha256(file_path) != expected_hash:
            raise LiveRunnerError(f"install_manifest_file_drift:{relative}")
    clean_environment = dict(os.environ)
    clean_environment["LD_LIBRARY_PATH"] = expected_live_environment()[
        "LD_LIBRARY_PATH"
    ]
    for relative in runtime_libraries:
        _linkage_ready(relative, INSTALL_ROOT, clean_environment)
    return manifest


def live_config(
    contract: dict, case_id: str, run_id: str, run_dir: Path
) -> dict[str, object]:
    run_dir = Path(run_dir).resolve()
    values = QUALIFICATION.resolve_launch_values(contract, case_id, {})
    return {
        **values,
        "record_bag": True,
        "start_rviz": False,
        "run_validator": True,
        "run_duration_s": 90,
        "validation_duration_s": 90,
        "gate0.qualification_evidence_enable": True,
        "gate0.candidate_events_path": str(run_dir / "candidate_events.csv"),
        "gate0.control_points_path": str(run_dir / "candidate_control_points_raw.csv"),
        "gate0.evidence_run_id": run_id,
        "gate0.evidence_manifest_path": str(run_dir / "gate0_run_manifest.json"),
        "runtime_root_dir": str(run_dir / "runtime"),
        "export_root_dir": str(run_dir / "exports"),
        "bag_output_dir": str(run_dir / "bags"),
        "iap_log_root": str(run_dir / "runtime/iap_logs"),
    }


def expected_live_environment() -> dict[str, str]:
    environment_root = TASK_ROOT / "live_environment"
    return {
        "HOME": str(environment_root / "home"),
        "ROS_HOME": str(environment_root / "ros_home"),
        "ROS_LOG_DIR": str(environment_root / "ros_logs"),
        "TMPDIR": str(environment_root / "tmp"),
        "XDG_RUNTIME_DIR": str(environment_root / "xdg_runtime"),
        "AMENT_PREFIX_PATH": ":".join((
            str(INSTALL_ROOT), "/root/ros2_ws/install", "/opt/ros/jazzy",
        )),
        "CMAKE_PREFIX_PATH": ":".join((
            str(INSTALL_ROOT), "/root/ros2_ws/install/glim_ros",
            "/root/ros2_ws/install/glim", "/opt/ros/jazzy",
        )),
        "LD_LIBRARY_PATH": ":".join((
            str(INSTALL_ROOT / "lib"), "/root/ros2_ws/install/glim_ros/lib",
            "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
            "/opt/ros/jazzy/lib/x86_64-linux-gnu",
        )),
    }


def validate_live_environment() -> dict[str, str]:
    expected = expected_live_environment()
    mismatches = [
        key for key, value in expected.items() if os.environ.get(key) != value
    ]
    if mismatches:
        raise LiveRunnerError(
            "live_environment_mismatch:" + ",".join(mismatches)
        )
    for key in ("HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_RUNTIME_DIR"):
        path = Path(expected[key])
        try:
            path.relative_to(REPOSITORY)
        except ValueError as exc:
            raise LiveRunnerError(f"live_environment_not_repository_local:{key}") from exc
        path.mkdir(parents=True, exist_ok=True)
    Path(expected["XDG_RUNTIME_DIR"]).chmod(0o700)
    return expected


def _one_file(root: Path, pattern: str, label: str) -> Path:
    paths = sorted(root.glob(pattern))
    if len(paths) != 1 or not paths[0].is_file() or paths[0].is_symlink():
        raise LiveRunnerError(f"{label}_identity_count_{len(paths)}")
    return paths[0]


def _candidate_id(row: dict) -> str:
    value = row.get("final_candidate_traj_id", "")
    if value in (None, ""):
        raise LiveRunnerError("p5_candidate_identity_missing")
    return f"traj-{value}"


def _registered_fixture_evidence_present(
    case_id: str, contract: dict, p5_rows: list[dict]
) -> bool:
    if case_id == "SAFE_NORMAL":
        return True
    fixture = contract["cases"][case_id]["fixture_values"]
    prefix = "p5_7" if case_id == "FINAL_REJECT" else "p5_6"
    expected_reason = (
        "p5_7_rejected_trajectory" if case_id == "FINAL_REJECT"
        else "future_unknown"
    )
    expected_source = (
        "final_candidate" if case_id == "FINAL_REJECT"
        else "runtime_committed"
    )

    def finite(value: object) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if result == result and abs(result) != float("inf") else None

    def within(sample: dict, field: str, low: str, high: str) -> bool:
        value = finite(sample.get(field))
        return value is not None and float(fixture[low]) <= value <= float(fixture[high])

    for row in p5_rows:
        for sample in SAFETY_ANALYZER.p5_3_sample_list(row):
            base_match = (
                sample.get("fixture_match") is True
                and sample.get("trajectory_sample_source") == expected_source
                and sample.get("fixture_expected_reason") == expected_reason
                and expected_reason in str(sample.get("reason", ""))
                and within(sample, "x", f"{prefix}.fixture.x_min", f"{prefix}.fixture.x_max")
                and within(sample, "y", f"{prefix}.fixture.y_min", f"{prefix}.fixture.y_max")
                and within(sample, "z", f"{prefix}.fixture.z_min", f"{prefix}.fixture.z_max")
                and within(sample, "query_tau_s", f"{prefix}.fixture.tau_min", f"{prefix}.fixture.tau_max")
            )
            if case_id == "FINAL_REJECT":
                expected_hpl = float(fixture["p5_7.fixture.hpl_pred_m"])
                expected_vpl = float(fixture["p5_7.fixture.vpl_pred_m"])
                if base_match and sample.get("bad") is True and all(
                    finite(sample.get(field)) == expected
                    for field, expected in (
                        ("fixture_expected_hpl", expected_hpl),
                        ("fixture_expected_vpl", expected_vpl),
                        ("hpl", expected_hpl), ("vpl", expected_vpl),
                    )
                ):
                    return True
            elif base_match and sample.get("unknown") is True:
                return True
    return False


def _normalize_events(
    case_id: str, contract: dict, p5_rows: list[dict], bspline_rows: list[dict]
) -> list[dict]:
    if not p5_rows or any(row.get("parse_error") for row in p5_rows):
        raise LiveRunnerError("p5_status_missing_or_malformed")
    final_rows = [
        row for row in p5_rows if row.get("phase") == "final_candidate"
    ]
    rejected = [
        row for row in final_rows
        if row.get("final_candidate_rejected") is True
        or "p5_7_rejected_trajectory" in str(row.get("reason", ""))
    ]
    accepted = [
        row for row in final_rows
        if str(row.get("action", "")) == "OK"
        and row.get("final_candidate_rejected") is not True
    ]
    emergency = [
        row for row in p5_rows
        if row.get("phase") == "runtime_committed"
        and str(row.get("action", "")) == "REQUEST_EMERGENCY_STOP_CANDIDATE"
    ]
    if not _registered_fixture_evidence_present(case_id, contract, p5_rows):
        raise LiveRunnerError("registered_fixture_evidence_mismatch")

    def sequence(row: dict) -> int:
        stamp = float(row.get("bag_time_s", 0.0))
        if stamp <= 0.0:
            raise LiveRunnerError("event_bag_time_missing")
        return int(round(stamp * 1_000_000_000.0))

    if case_id == "FINAL_REJECT":
        if len(rejected) != 1 or accepted or emergency:
            raise LiveRunnerError("final_reject_event_cardinality_mismatch")
        row = rejected[0]
        candidate = _candidate_id(row)
        raw_id = row.get("final_candidate_traj_id")
        if any(item.get("traj_id") == raw_id for item in bspline_rows):
            raise LiveRunnerError("rejected_candidate_was_published")
        return [{
            "sequence": sequence(row), "raw_bag_time_s": row["bag_time_s"],
            "type": "FINAL_REJECT",
            "candidate_id": candidate, "reason": "p5_7_rejected_trajectory",
        }]
    if rejected or len(accepted) != 1:
        raise LiveRunnerError("final_accept_event_cardinality_mismatch")
    accept = accepted[0]
    raw_id = accept.get("final_candidate_traj_id")
    candidate = _candidate_id(accept)
    published = [
        row for row in bspline_rows
        if row.get("traj_id") == raw_id
        and float(row.get("bag_time_s", 0.0)) >= float(accept.get("bag_time_s", 0.0))
    ]
    if len(published) != 1:
        raise LiveRunnerError("matching_normal_publication_cardinality_mismatch")
    publish = published[0]
    if sequence(accept) >= sequence(publish):
        raise LiveRunnerError("normal_publication_not_after_accept")
    events = [
        {"sequence": sequence(accept), "raw_bag_time_s": accept["bag_time_s"],
         "type": "FINAL_ACCEPT", "candidate_id": candidate},
        {"sequence": sequence(publish), "raw_bag_time_s": publish["bag_time_s"],
         "type": "NORMAL_PUBLISH", "candidate_id": candidate},
    ]
    if case_id == "RUNTIME_FAIL":
        if not emergency or any(
            "future_unknown" not in str(row.get("reason", ""))
            for row in emergency
        ):
            raise LiveRunnerError("future_unknown_emergency_missing")
        threshold = float(contract["p5_thresholds"]["p5.future_unknown_to_emergency_s"])
        first_runtime = emergency[0]
        duration = float(first_runtime.get("future_unknown_duration_s", threshold))
        if duration < threshold or sequence(first_runtime) <= sequence(publish):
            raise LiveRunnerError("future_unknown_emergency_before_threshold")
        events.append({
            "sequence": sequence(first_runtime),
            "raw_bag_time_s": first_runtime["bag_time_s"],
            "type": "RUNTIME_ACTION",
            "candidate_id": candidate, "action": "EMERGENCY_STOP",
            "reason": "future_unknown_timeout",
            "raw_matching_status_count": len(emergency),
        })
    elif emergency:
        raise LiveRunnerError("false_runtime_emergency")
    return events


def normalize_live_run(
    contract: dict,
    contract_path: Path,
    case_id: str,
    run_id: str,
    run_dir: Path,
    process_result: dict,
    git_commit: str,
) -> dict:
    run_dir = Path(run_dir).resolve()
    manifest_path = _one_file(
        run_dir, "exports/*/test_planner_manifest.json", "launch_manifest"
    )
    bag_metadata_path = _one_file(
        run_dir, "bags/*/metadata.yaml", "bag_metadata"
    )
    bag_dir = bag_metadata_path.parent
    launch_manifest = json.loads(manifest_path.read_text())
    launch_binding = launch_manifest.get("icra_p0_p5_qualification")
    expected_binding = QUALIFICATION.build_launch_binding(
        contract, contract_path, case_id, git_commit, run_id,
        QUALIFICATION.resolve_launch_values(contract, case_id, {}),
    )
    if launch_binding != expected_binding:
        raise LiveRunnerError("live_launch_binding_mismatch")
    if process_result.get("required_processes_ok") is not True:
        raise LiveRunnerError("required_process_failure")
    observed = process_result.get("required_processes", {})
    if set(observed) != set(REQUIRED_PROCESSES):
        raise LiveRunnerError("required_process_identity_mismatch")
    if not QUALIFICATION.live_process_lifecycle_exact(
        process_result, tuple(REQUIRED_PROCESSES)
    ):
        raise LiveRunnerError("required_process_lifecycle_mismatch")
    metadata = SAFETY_ANALYZER.read_bag_metadata(bag_dir)
    topic_counts = {
        topic: int((metadata.get("topic_counts", {}) or {}).get(topic, 0) or 0)
        for topic in contract["required_topics"]
    }
    if any(count <= 0 for count in topic_counts.values()):
        raise LiveRunnerError("required_topic_missing")
    p0_artifacts, p0_error = SAFETY_ANALYZER.read_p0_bag_artifacts(
        bag_dir, metadata
    )
    p5_rows, p5_error = SAFETY_ANALYZER.read_p5_status_messages(
        bag_dir, metadata
    )
    bspline_rows, bspline_error = SAFETY_ANALYZER.read_bspline_messages(
        bag_dir, metadata
    )
    if p0_error or p5_error or bspline_error:
        raise LiveRunnerError("live_bag_decode_failed")
    health_rows = p0_artifacts.get("health_rows", [])
    stable_rows = [
        row for row in health_rows
        if row.get("ready") is True and row.get("stale") is False
        and int(row.get("generation_id", 0) or 0) > 0
        and int(row.get("predictor_requested_worker_count", 0) or 0) == 4
        and int(row.get("predictor_effective_worker_count", 0) or 0) == 4
    ]
    if len(stable_rows) < 2:
        raise LiveRunnerError("p0_ready_stable_rows_missing")
    p0_samples = [
        {
            "sequence": index,
            "ready": True,
            "stable": True,
            "generation_id": int(row["generation_id"]),
            "worker_count": 4,
            "refresh_s": float(row.get("refresh_duration_ms", 0.0)) / 1000.0,
        }
        for index, row in enumerate(stable_rows, 1)
    ]
    raw_paths = []
    for path in sorted(run_dir.iterdir()):
        if path.is_file() and path.name != "normalized_run.json":
            raw_paths.append(path)
        elif path.is_symlink():
            raise LiveRunnerError(f"raw_topology_symlink:{path.name}")
    for root_name in ("bags", "exports"):
        root = run_dir / root_name
        for path in sorted(root.rglob("*")):
            if path.is_symlink():
                raise LiveRunnerError(
                    f"raw_topology_symlink:{path.relative_to(run_dir)}"
                )
            if path.is_file():
                raw_paths.append(path)
    relative_sources = [
        str(path.relative_to(REPOSITORY)) for path in dict.fromkeys(raw_paths)
    ]
    required_source_names = {
        "process_result.json", "capture_ready.json", "launch_command.json",
        "stdout.log", "metadata.yaml",
    }
    if not required_source_names.issubset({Path(path).name for path in relative_sources}):
        raise LiveRunnerError("required_raw_source_missing")
    if not any(Path(path).suffix in {".db3", ".mcap"} for path in relative_sources):
        raise LiveRunnerError("bag_payload_missing")
    return {
        "validation_only": False,
        "case_id": case_id,
        "run_id": run_id,
        "fixture_alias": contract["cases"][case_id]["fixture_alias"],
        "launch_binding": launch_binding,
        "processes": [
            {"identity": name, "alive_until_controlled_shutdown": True}
            for name in contract["required_processes"]
        ],
        "controlled_shutdown": True,
        "topic_counts": topic_counts,
        "p0_samples": p0_samples,
        "events": _normalize_events(case_id, contract, p5_rows, bspline_rows),
        "raw_sources": relative_sources,
    }


def run_ordered_attempts(execute) -> dict:
    attempted: list[str] = []
    completed: list[str] = []
    failure = ""
    for case_id, run_id in LIVE_IDENTITIES:
        attempted.append(run_id)
        result = execute(case_id, run_id)
        if result.get("completed") is not True:
            failure = str(result.get("failure_reason", "attempt_failed"))
            break
        completed.append(run_id)
    return {
        "attempted": attempted,
        "completed": completed,
        "retries": 0,
        "failure_reason": failure,
        "state": "COMPLETE" if len(completed) == len(LIVE_IDENTITIES) else "FAILED",
    }


def _run_launch(command: list[str], stdout_path: Path) -> tuple[int, dict]:
    def signal_group(pgid: int, signum: int) -> None:
        try:
            os.killpg(pgid, signum)
        except ProcessLookupError:
            pass

    def group_members(pgid: int) -> list[int]:
        import psutil

        members = []
        for process in psutil.process_iter(["pid"]):
            try:
                if os.getpgid(process.pid) == pgid:
                    members.append(int(process.pid))
            except (OSError, psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return members

    with stdout_path.open("w") as output:
        launch = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        pgid = launch.pid
        monitor = GATE_RUNNER.RequiredProcessMonitor(
            launch.pid, REQUIRED_PROCESSES, 90.0
        )
        monitor.start()
        monitor.launch_running = True
        early_exit = False
        forced_orphan_cleanup = False
        try:
            exit_code = launch.wait(timeout=89.5)
        except subprocess.TimeoutExpired:
            monitor.mark_controlled_shutdown()
            signal_group(pgid, signal.SIGINT)
            try:
                exit_code = launch.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                signal_group(pgid, signal.SIGKILL)
                exit_code = launch.wait(timeout=10.0)
        else:
            early_exit = True
            monitor._record_failure(
                "ros2_launch", "top_level_launch_exited_before_controlled_shutdown",
                exit_code, "runtime",
            )
            monitor.mark_controlled_shutdown()
            signal_group(pgid, signal.SIGINT)
        deadline = time.monotonic() + 10.0
        remaining = group_members(pgid)
        while remaining and time.monotonic() < deadline:
            time.sleep(0.05)
            remaining = group_members(pgid)
        if remaining:
            forced_orphan_cleanup = True
            signal_group(pgid, signal.SIGKILL)
            deadline = time.monotonic() + 5.0
            while group_members(pgid) and time.monotonic() < deadline:
                time.sleep(0.05)
        final_members = group_members(pgid)
        result = monitor.finish()
        result.update({
            "controlled_shutdown": not early_exit,
            "process_group_id": pgid,
            "orphan_check_passed": not final_members and not forced_orphan_cleanup,
            "forced_orphan_cleanup": forced_orphan_cleanup,
            "remaining_process_group_pids": final_members,
        })
        if early_exit or forced_orphan_cleanup or final_members:
            result["required_processes_ok"] = False
            result["process_failures"].append({
                "process_name": "process_group",
                "phase": "runtime" if early_exit else "post_shutdown",
                "reason": (
                    "top_level_launch_exited_before_controlled_shutdown"
                    if early_exit else "task_process_orphan_after_shutdown"
                ),
                "exit_code_signal": exit_code,
            })
        return exit_code, result


def _execute_live_attempt(
    contract: dict, case_id: str, run_id: str, live_root: Path, git_commit: str
) -> dict:
    run_dir = live_root / run_id
    if run_dir.exists() or run_dir.is_symlink():
        raise LiveRunnerError(f"identity_already_exists:{run_id}")
    run_dir.mkdir(parents=True)
    config = live_config(contract, case_id, run_id, run_dir)
    command = GATE_RUNNER.launch_command(config)
    _write_json(run_dir / "launch_command.json", {"argv": command})
    ready_path = run_dir / "capture_ready.json"
    capture_command = [
        sys.executable,
        str(REPOSITORY / "scripts/dev_planner/gate0_capture_p0_health.py"),
        "--output", str(run_dir / "risk_grid_health.jsonl"),
        "--integrity-output", str(run_dir / "integrity_report.jsonl"),
        "--duration-s", "95", "--ready-file", str(ready_path),
    ]
    capture, stream = GATE_RUNNER._start_capture(
        capture_command, run_dir / "capture_stdout.log"
    )
    readiness = GATE_RUNNER._wait_for_capture_ready(capture, ready_path)
    if readiness.get("ready") is not True:
        capture_exit = GATE_RUNNER._stop_capture(capture, stream)
        return {
            "completed": False,
            "failure_reason": "capture_not_ready",
            "capture_exit_code": capture_exit,
        }
    try:
        launch_exit, process_result = _run_launch(
            command, run_dir / "stdout.log"
        )
    finally:
        capture_exit = GATE_RUNNER._stop_capture(capture, stream)
    _write_json(run_dir / "process_result.json", {
        "launch_exit_code": launch_exit,
        "capture_exit_code": capture_exit,
        "capture_readiness": readiness,
        **process_result,
    })
    if launch_exit != 0 or process_result.get("required_processes_ok") is not True:
        return {"completed": False, "failure_reason": "launch_or_process_failure"}
    try:
        normalized = normalize_live_run(
            contract, CONTRACT_PATH, case_id, run_id, run_dir,
            process_result, git_commit,
        )
    except (LiveRunnerError, OSError, ValueError, json.JSONDecodeError) as exc:
        return {
            "completed": False,
            "failure_reason": f"normalization_failure:{type(exc).__name__}:{exc}",
        }
    _write_json(run_dir / "normalized_run.json", normalized)
    return {"completed": True, "normalized_run": normalized}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live-root", type=Path, default=TASK_ROOT / "live")
    parser.add_argument("--install-manifest", type=Path, default=INSTALL_MANIFEST_PATH)
    parser.add_argument("--freeze-install-only", action="store_true")
    args = parser.parse_args()
    if args.freeze_install_only:
        audit_and_freeze_install(INSTALL_ROOT, args.install_manifest)
        print("ICRA068_INSTALL_CLOSURE_PASS")
        return 0
    live_root = args.live_root.resolve()
    install_manifest_path = args.install_manifest.resolve()
    if live_root != (TASK_ROOT / "live").resolve():
        raise LiveRunnerError("live_root_identity_mismatch")
    if install_manifest_path != INSTALL_MANIFEST_PATH.resolve():
        raise LiveRunnerError("install_manifest_path_mismatch")
    if live_root.exists() or live_root.is_symlink():
        raise LiveRunnerError("live_root_already_exists")
    preflight_root = TASK_ROOT / "preflight"
    if preflight_root.exists() or preflight_root.is_symlink():
        raise LiveRunnerError("gpu_preflight_evidence_already_exists")
    git_commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPOSITORY, text=True
    ).strip()
    live_environment = validate_live_environment()
    validate_frozen_install_manifest(
        install_manifest_path, git_commit
    )
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    if tuple(contract["required_processes"]) != tuple(REQUIRED_PROCESSES):
        raise LiveRunnerError("required_process_marker_contract_mismatch")
    if tuple(contract["cases"]) != tuple(case_id for case_id, _ in LIVE_IDENTITIES):
        raise LiveRunnerError("live_identity_contract_mismatch")
    free_bytes = shutil.disk_usage(REPOSITORY).free
    if free_bytes < MIN_FREE_BYTES:
        raise LiveRunnerError(f"disk_free_below_40_gib:{free_bytes}")
    live_root.mkdir(parents=True)
    preflight = GATE_RUNNER.run_gpu_preflight(preflight_root)
    if preflight.get("gpu_ready") is not True:
        print("GPU_NOT_READY")
        return 3
    normalized: list[dict] = []

    def execute(case_id: str, run_id: str) -> dict:
        result = _execute_live_attempt(
            contract, case_id, run_id, live_root, git_commit
        )
        if result.get("completed"):
            normalized.append(result["normalized_run"])
        return result

    state = run_ordered_attempts(execute)
    state.update({
        "schema_version": "icra_p0_p5_live_runner_state_v1",
        "gpu_preflight_invocations": 1,
        "launch_invocations": len(state["attempted"]),
        "registered": [run_id for _, run_id in LIVE_IDENTITIES],
        "free_bytes_before_gpu": free_bytes,
        "live_environment": live_environment,
        "install_manifest_sha256": _sha256(install_manifest_path),
    })
    _write_json(live_root / "runner_state.json", state)
    if state["state"] != "COMPLETE":
        return 4
    QUALIFICATION.write_live_bundle(
        contract, CONTRACT_PATH, normalized, git_commit,
        install_manifest_path, live_root / "icra_p0_p5_evidence_v1.json",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
