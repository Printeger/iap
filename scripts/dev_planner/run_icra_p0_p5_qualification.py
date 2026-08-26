#!/usr/bin/env python3
"""One-shot ICRA-070 fused-sensor P0+P5 runner and evidence normalizer."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
TASK_ROOT = REPOSITORY / "results/icra27/icra070"
PRODUCT_TASK_ROOT = REPOSITORY / "results/icra27/icra068"
INSTALL_ROOT = PRODUCT_TASK_ROOT / "install"
OVERLAY_INSTALL_ROOT = TASK_ROOT / "install"
PRODUCT_BUILD_ROOT = PRODUCT_TASK_ROOT / "build/iap"
PRODUCT_INSTALL_DRIVER_SHA256 = (
    "2c773421a1625a6304f9f8c7dae68f256f7c363735fb6aaf9a241f8d17392f0c"
)
PRODUCT_SUBINSTALL_DRIVER_SHA256 = (
    "86a3b5482fb7225b2a443338d18fb54024f3f520fad2a97facd44d9c97c2c874"
)
OVERLAY_MANIFEST_PATH = TASK_ROOT / "compact/icra070_overlay_manifest_v2.json"
REPAIR_EVIDENCE_PATH = TASK_ROOT / "compact/overlay_cache_repair_v1.json"
ADOPTION_MANIFEST_PATH = TASK_ROOT / "compact/icra070_adoption_manifest_v2.json"
DEPENDENCY_PREFLIGHT_PATH = TASK_ROOT / "compact/gnss_dependency_preflight.json"
OVERLAY_COMMAND_PATH = TASK_ROOT / "compact/overlay_install_command.json"
OVERLAY_INSTALL_DRIVER_PATH = TASK_ROOT / "overlay_install_driver.cmake"
CONTRACT_PATH = REPOSITORY / "config/icra27/icra_p0_p5_qualification_v1.json"
INSTALL_MANIFEST_PATH = PRODUCT_TASK_ROOT / "icra068_install_manifest.json"
PRODUCT_COMMIT = "005ce1a9dc20109dfb9600d62a8a9085aa11cb45"
PRODUCT_MANIFEST_SHA256 = (
    "7662a2c4aa4840dac2d80aac8cdf87041555f9114ca86dd844e862462134d420"
)
EXPECTED_RETAINED_TASK_INVENTORY = {
    "entry_count": 7364,
    "inventory_sha256": (
        "fdeb47e3d025bbc7c442b86521e6808d1452d928178a52df0b2f9e03aace4858"
    ),
}
EXPECTED_FAILED_OVERLAY_INVENTORY = {
    "entry_count": 474,
    "inventory_sha256": (
        "9381cb03d7cff06a517f8da9fcde0c179cc4cf130b61a2e04750dfe683acec89"
    ),
}
ORIGINAL_BLOCKER_SHA256 = {
    "results/icra27/icra070/compact/final_result.json": (
        "ebb95f8f6dc05bf72d7ed3ee3e65af1a8d7279a27e02a6c8efa691e5e374b26b"
    ),
    "results/icra27/icra070/compact/command_ledger.json": (
        "6bc56ea8d2d4a3e61c8a572a380a35da86e6d1035c2791e3848b917db48e6898"
    ),
    "results/icra27/icra070/compact/gnss_dependency_preflight.json": (
        "b35afeb83fb119efacc5ea40ef93ce24360ba65e2f57ec9a002d2f4824426ece"
    ),
    "results/icra27/icra070/compact/overlay_install_command.json": (
        "f9c48b12c4170122f58eb49c6fced267d5d4b19fd8633495b8ef1962b389e16f"
    ),
    "results/icra27/icra070/overlay_install_driver.cmake": (
        "cac3da758800fa42172b43caef6551f39f45fcc350c2a7270a2191167ef45581"
    ),
}
RINEX_EPHEMERIS_PATH = Path(
    "/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx"
)
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
REGISTERED_EMPTY_DEFAULT_KEYS = {
    "p1.debug_csv_path", "p2.debug_csv_path", "p3.debug_csv_path",
    "p4.debug_csv_path", "p4.profile_trace_path", "p4.g0c.protocol_path",
    "p4.g0c.protocol_sha256", "p4.g0c.registry_path",
    "p4.g0c.registry_sha256", "p4.g0c.fixture_path",
    "p4.g0c.fixture_sha256", "p4.g0c.run_id",
    "p4.g0c.run_manifest_path", "p4.g0c.csv_path", "p4.g0c.child_home",
    "p4.g0c.child_ros_home", "p4.g0c.child_ros_log_dir",
    "p4.g0c.child_tmpdir", "p4.g0c.child_xdg_runtime_dir",
}
REPLACEMENT_LIVE_RUN_IDENTITIES = {
    "SAFE_NORMAL": "icra-p0-p5-live-safe-normal-003",
    "FINAL_REJECT": "icra-p0-p5-live-final-reject-003",
    "RUNTIME_FAIL": "icra-p0-p5-live-runtime-fail-003",
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
QUALIFICATION.LIVE_RUN_IDENTITIES = dict(REPLACEMENT_LIVE_RUN_IDENTITIES)
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
    canonical_lines = sorted(
        re.sub(r"\s+\(0x[0-9a-fA-F]+\)$", "", line.strip())
        for line in text_output.splitlines() if line.strip()
    )
    canonical = "\n".join(canonical_lines) + "\n"
    return hashlib.sha256(canonical.encode()).hexdigest()


def linkage_inventory_matches(
    expected: dict, runtime_libraries: tuple[str, ...],
    install_root: Path, environment: dict,
) -> bool:
    return all(
        _linkage_ready(relative, install_root, environment)
        == expected.get(relative)
        for relative in runtime_libraries
    )


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def _write_json_exclusive(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        with path.open("x") as stream:
            json.dump(value, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except FileExistsError as exc:
        raise LiveRunnerError(f"exclusive_evidence_exists:{path.name}") from exc


def verify_original_blocker_evidence() -> dict:
    observed = {}
    for relative, expected_hash in ORIGINAL_BLOCKER_SHA256.items():
        path = REPOSITORY / relative
        if not path.is_file() or path.is_symlink():
            raise LiveRunnerError(f"original_blocker_missing_or_symlink:{relative}")
        observed_hash = _sha256(path)
        if observed_hash != expected_hash:
            raise LiveRunnerError(f"original_blocker_hash_mismatch:{relative}")
        observed[relative] = observed_hash
    return {
        "file_sha256": observed,
        "all_original_blocker_bytes_preserved": True,
    }


def require_repair_outputs_absent(paths: list[Path]) -> None:
    for path in paths:
        if Path(path).exists() or Path(path).is_symlink():
            raise LiveRunnerError(f"repair_evidence_already_exists:{Path(path).name}")


def _dependency_file(path: Path, executable: bool = False) -> dict:
    path = Path(path).resolve()
    if not path.is_file() or path.is_symlink() or not os.access(path, os.R_OK):
        raise LiveRunnerError(f"dependency_missing_or_unreadable:{path}")
    if executable and not os.access(path, os.X_OK):
        raise LiveRunnerError(f"dependency_not_executable:{path}")
    return {
        "path": str(path),
        "size": path.stat().st_size,
        "sha256": _sha256(path),
        "readable": True,
        "executable": bool(executable),
    }


def resolve_gnss_dependencies(
    contract: dict, install_root: Path, iap_install_root: Path | None = None,
) -> dict:
    """Resolve the frozen degraded-GNSS inputs without fallback or ROS."""
    values = contract.get("qualification_values", {})
    expected = QUALIFICATION.FULL_SENSOR_QUALIFICATION_VALUES
    if any(values.get(key) != value for key, value in expected.items()):
        raise LiveRunnerError("full_sensor_dependency_contract_mismatch")
    install_root = Path(install_root).resolve()
    iap_install_root = Path(iap_install_root or install_root).resolve()
    simulator = _dependency_file(
        install_root / "lib/gnss_sim/gnss_sim_node", executable=True
    )
    case_values = [
        QUALIFICATION.resolve_launch_values(contract, case_id, {})
        for case_id, _ in LIVE_IDENTITIES
    ]
    sensor_keys = set(expected) | {"scenario"}
    frozen = {key: case_values[0][key] for key in sensor_keys}
    if any(
        {key: values[key] for key in sensor_keys} != frozen
        for values in case_values[1:]
    ):
        raise LiveRunnerError("full_sensor_case_resolution_mismatch")
    selected_scenario_path = (
        OVERLAY_INSTALL_ROOT / "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"
    )
    if frozen["gnss_scenario_file"] != str(selected_scenario_path):
        raise LiveRunnerError("full_sensor_scenario_path_mismatch")
    scenario = _dependency_file(
        iap_install_root / "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"
    )
    if frozen["gnss_rinex_nav_file"] != str(RINEX_EPHEMERIS_PATH):
        raise LiveRunnerError("full_sensor_rinex_path_mismatch")
    ephemeris = _dependency_file(Path(frozen["gnss_rinex_nav_file"]))
    return {
        "schema_version": "icra070_gnss_dependency_preflight_v1",
        "dependency_ready": True,
        "gnss_simulator": simulator,
        "scenario": scenario,
        "rinex_ephemeris": ephemeris,
        "sensor_model": {
            "constellations": values["gnss_enabled_constellations"].split(","),
            "pseudorange_noise_std_m": values["gnss_pr_noise_base"],
            "doppler_noise_std_mps": values["gnss_dop_noise_base"],
            "map_occlusion": values["gnss_enable_map_occlusion"],
            "skymask": values["gnss_enable_skymask"],
            "nlos": values["gnss_enable_nlos"],
            "multipath": values["gnss_enable_multipath"],
            "time_source": values["gnss_time_source"],
            "trigger_topic": frozen["gnss_trigger_topic"],
            "fallback_to_synthetic_on_rinex_error": frozen[
                "gnss_fallback_to_synthetic_on_rinex_error"
            ],
        },
    }


def overlay_install_command() -> list[str]:
    return [
        "cmake", f"-DCMAKE_INSTALL_PREFIX={OVERLAY_INSTALL_ROOT}",
        "-P", str(OVERLAY_INSTALL_DRIVER_PATH),
    ]


def sanitize_install_driver_text(text: str) -> str:
    """Remove the two retained-build writes that are not overlay installation."""
    blocks = re.findall(
        r"\nif\(CMAKE_INSTALL_COMPONENT STREQUAL \"Unspecified\" OR NOT "
        r"CMAKE_INSTALL_COMPONENT\)\n.*?\nendif\(\)\n",
        text,
        flags=re.DOTALL,
    )
    compile_blocks = [block for block in blocks if "compileall" in block]
    if len(compile_blocks) != 1:
        raise LiveRunnerError("product_compileall_block_shape_mismatch")
    sanitized = text.replace(compile_blocks[0], "\n", 1)
    marker = "\nif(CMAKE_INSTALL_COMPONENT)\n"
    if sanitized.count(marker) != 1 \
            or "file(WRITE" not in sanitized.split(marker, 1)[1]:
        raise LiveRunnerError("product_install_manifest_block_shape_mismatch")
    sanitized = sanitized.split(marker, 1)[0] + "\n"
    if "compileall" in sanitized or "file(WRITE" in sanitized:
        raise LiveRunnerError("overlay_install_driver_write_not_removed")
    return sanitized


def overlay_install_environment() -> dict[str, str]:
    """Return a closed install environment with no external-write controls."""
    return {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"}


def write_overlay_install_driver() -> dict:
    source = PRODUCT_BUILD_ROOT / "cmake_install.cmake"
    if not source.is_file() or source.is_symlink():
        raise LiveRunnerError("product_install_driver_missing_or_symlink")
    if OVERLAY_INSTALL_DRIVER_PATH.exists() \
            or OVERLAY_INSTALL_DRIVER_PATH.is_symlink():
        raise LiveRunnerError("overlay_install_driver_already_exists")
    if _sha256(source) != PRODUCT_INSTALL_DRIVER_SHA256:
        raise LiveRunnerError("product_install_driver_sha256_mismatch")
    subdriver = PRODUCT_BUILD_ROOT / "iap_msgs__py/cmake_install.cmake"
    if not subdriver.is_file() or subdriver.is_symlink() \
            or _sha256(subdriver) != PRODUCT_SUBINSTALL_DRIVER_SHA256:
        raise LiveRunnerError("product_subinstall_driver_sha256_mismatch")
    sanitized = sanitize_install_driver_text(source.read_text())
    OVERLAY_INSTALL_DRIVER_PATH.parent.mkdir(parents=True, exist_ok=True)
    OVERLAY_INSTALL_DRIVER_PATH.write_text(sanitized)
    return {
        "source_path": str(source.relative_to(REPOSITORY)),
        "source_sha256": _sha256(source),
        "task_driver_path": str(
            OVERLAY_INSTALL_DRIVER_PATH.relative_to(REPOSITORY)
        ),
        "task_driver_sha256": _sha256(OVERLAY_INSTALL_DRIVER_PATH),
        "included_driver_sha256": {
            str(subdriver.relative_to(REPOSITORY)): _sha256(subdriver),
        },
        "compileall_block_removed": True,
        "product_build_manifest_write_removed": True,
    }


def tree_byte_inventory(root: Path) -> dict[str, dict[str, object]]:
    """Hash every retained byte and bind symlink targets without following them."""
    root = Path(root).resolve()
    inventory = {}
    for path in sorted(root.rglob("*")):
        relative = str(path.relative_to(root))
        if path.is_symlink():
            inventory[relative] = {"type": "symlink", "target": os.readlink(path)}
        elif path.is_file():
            inventory[relative] = {
                "type": "file", "size": path.stat().st_size, "sha256": _sha256(path),
            }
    return inventory


def inventory_summary(inventory: dict) -> dict[str, object]:
    encoded = json.dumps(inventory, sort_keys=True, separators=(",", ":")).encode()
    return {
        "entry_count": len(inventory),
        "inventory_sha256": hashlib.sha256(encoded).hexdigest(),
    }


PYTHON_CACHE_SUFFIXES = {".pyc", ".pyo", ".pyd"}


def is_python_cache_path(relative: Path | str) -> bool:
    relative = Path(relative)
    if relative.is_absolute() or ".." in relative.parts:
        raise LiveRunnerError(f"cache_path_not_relative:{relative}")
    return "__pycache__" in relative.parts \
        or relative.suffix.lower() in PYTHON_CACHE_SUFFIXES


def enumerate_python_cache(root: Path) -> tuple[list[dict], list[str]]:
    root = Path(root).resolve()
    files = []
    directories = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if path.is_symlink() and is_python_cache_path(relative):
            raise LiveRunnerError(f"cache_path_is_symlink:{relative}")
        if path.is_file() and is_python_cache_path(relative):
            files.append({
                "path": str(relative),
                "size": path.stat().st_size,
                "sha256": _sha256(path),
            })
        if path.is_dir() and not path.is_symlink() \
                and path.name == "__pycache__":
            directories.append(str(relative))
    return files, sorted(directories)


def python_cache_inventory(root: Path) -> dict[str, object]:
    files, directories = enumerate_python_cache(root)
    return {"files": files, "directories": directories}


def inventory_overlay_v2(
    overlay_root: Path, base_root: Path, source_root: Path,
    allowed_differences: dict[str, str], expected_pre_repair: dict,
) -> dict:
    """Prove the repaired overlay is the frozen failed set minus caches."""
    overlay_root = Path(overlay_root).resolve()
    base_root = Path(base_root).resolve()
    source_root = Path(source_root).resolve()
    if any(is_python_cache_path(relative) for relative in allowed_differences):
        raise LiveRunnerError("python_cache_cannot_be_authorized")
    if any(row.get("type") == "symlink" for row in expected_pre_repair.values()):
        raise LiveRunnerError("pre_repair_overlay_contains_symlink")
    observed = tree_byte_inventory(overlay_root)
    if any(row.get("type") == "symlink" for row in observed.values()):
        raise LiveRunnerError("repaired_overlay_contains_symlink")
    cache_paths = sorted(
        relative for relative in expected_pre_repair
        if is_python_cache_path(relative)
    )
    expected_files = {
        relative: row for relative, row in expected_pre_repair.items()
        if relative not in cache_paths
    }
    if set(observed) != set(expected_files):
        missing = sorted(set(expected_files) - set(observed))
        extra = sorted(set(observed) - set(expected_files))
        label = missing[0] if missing else extra[0]
        kind = "missing" if missing else "extra"
        raise LiveRunnerError(f"repaired_overlay_file_set_{kind}:{label}")
    hashes = {}
    base_hashes = {}
    differences = []
    for relative in sorted(observed):
        if observed[relative] != expected_files[relative]:
            raise LiveRunnerError(f"non_cache_overlay_mutated:{relative}")
        overlay = overlay_root / relative
        base = base_root / relative
        if not base.is_file() or base.is_symlink():
            raise LiveRunnerError(f"overlay_file_missing_from_base:{relative}")
        overlay_hash = _sha256(overlay)
        base_hash = _sha256(base)
        hashes[relative] = overlay_hash
        base_hashes[relative] = base_hash
        if overlay_hash == base_hash and overlay.read_bytes() == base.read_bytes():
            continue
        if relative not in allowed_differences:
            raise LiveRunnerError(f"unauthorized_overlay_difference:{relative}")
        source = source_root / allowed_differences[relative]
        if not source.is_file() or source.is_symlink() \
                or _sha256(source) != overlay_hash \
                or source.read_bytes() != overlay.read_bytes():
            raise LiveRunnerError(f"overlay_alias_source_mismatch:{relative}")
        differences.append(relative)
    return {
        "pre_repair_file_count": len(expected_pre_repair),
        "file_count": len(observed),
        "omitted_cache_artifacts": cache_paths,
        "file_sha256": hashes,
        "base_file_sha256": base_hashes,
        "authorized_differences": sorted(differences),
        "non_cache_file_set_complete": True,
        "binary_library_bytes_equal": True,
        "symlink_count": 0,
    }


def repair_overlay_cache_boundary(
    overlay_root: Path, base_root: Path, source_root: Path,
    allowed_differences: dict[str, str], expected_pre_repair: dict,
    expected_source_cache: dict | None = None,
) -> dict:
    """Remove only enumerated cache artifacts from one frozen failed overlay."""
    overlay_root = Path(overlay_root).resolve()
    source_root = Path(source_root).resolve()
    if expected_source_cache is not None \
            and python_cache_inventory(source_root) != expected_source_cache:
        raise LiveRunnerError("source_cache_inventory_mismatch")
    if tree_byte_inventory(overlay_root) != expected_pre_repair:
        raise LiveRunnerError("failed_overlay_pre_repair_inventory_mismatch")
    cache_files, cache_directories = enumerate_python_cache(overlay_root)
    if not cache_files:
        raise LiveRunnerError("failed_overlay_cache_inventory_empty")
    for row in cache_files:
        path = overlay_root / row["path"]
        if not path.is_file() or path.is_symlink() \
                or path.stat().st_size != row["size"] \
                or _sha256(path) != row["sha256"]:
            raise LiveRunnerError(f"cache_changed_before_removal:{row['path']}")
        path.unlink()
    for relative in sorted(
        cache_directories, key=lambda value: len(Path(value).parts), reverse=True
    ):
        path = overlay_root / relative
        try:
            path.rmdir()
        except OSError as exc:
            raise LiveRunnerError(
                f"cache_directory_not_empty_after_removal:{relative}"
            ) from exc
    remaining_cache, remaining_directories = enumerate_python_cache(overlay_root)
    if remaining_cache or remaining_directories:
        raise LiveRunnerError("python_cache_remained_after_repair")
    proof = inventory_overlay_v2(
        overlay_root, base_root, source_root,
        allowed_differences, expected_pre_repair,
    )
    if expected_source_cache is not None \
            and python_cache_inventory(source_root) != expected_source_cache:
        raise LiveRunnerError("source_cache_mutated_by_repair")
    proof.update({
        "removed_cache_files": [row["path"] for row in cache_files],
        "removed_cache_inventory": cache_files,
        "removed_cache_directories": cache_directories,
        "source_cache_inventory": expected_source_cache,
        "source_cache_unchanged": expected_source_cache is not None,
    })
    return proof


def run_no_bytecode_import_probe(
    install_root: Path, module_paths: list[Path], environment: dict[str, str],
) -> dict:
    install_root = Path(install_root).resolve()
    if environment.get("PYTHONDONTWRITEBYTECODE") != "1":
        raise LiveRunnerError("import_probe_bytecode_not_disabled")
    resolved_modules = []
    for path in module_paths:
        path = Path(path).resolve()
        try:
            path.relative_to(install_root)
        except ValueError as exc:
            raise LiveRunnerError("import_probe_module_outside_install") from exc
        if not path.is_file() or path.is_symlink():
            raise LiveRunnerError(f"import_probe_module_missing:{path}")
        resolved_modules.append(path)
    before = tree_byte_inventory(install_root)
    code = (
        "import runpy,sys; sys.dont_write_bytecode=True; "
        "[runpy.run_path(path) for path in sys.argv[1:]]"
    )
    command = ["/usr/bin/python3", "-B", "-c", code] + [
        str(path) for path in resolved_modules
    ]
    completed = subprocess.run(
        command, cwd=REPOSITORY, env=dict(environment),
        text=True, capture_output=True, check=False,
    )
    after = tree_byte_inventory(install_root)
    proof = {
        "argv": command,
        "cwd": str(REPOSITORY),
        "environment": dict(environment),
        "exit_code": completed.returncode,
        "stdout": completed.stdout,
        "stdout_sha256": hashlib.sha256(completed.stdout.encode()).hexdigest(),
        "stderr": completed.stderr,
        "stderr_sha256": hashlib.sha256(completed.stderr.encode()).hexdigest(),
        "module_paths": [str(path) for path in resolved_modules],
        "install_inventory_before": inventory_summary(before),
        "install_inventory_after": inventory_summary(after),
        "install_inventory_unchanged": before == after,
        "python_cache_absent_after": python_cache_inventory(install_root) == {
            "files": [], "directories": [],
        },
    }
    if not (
        completed.returncode == 0
        and before == after
        and proof["python_cache_absent_after"] is True
    ):
        raise LiveRunnerError("installed_import_probe_not_cache_safe")
    return proof


def inventory_overlay(
    overlay_root: Path, base_root: Path, allowed_differences: dict[str, str],
    source_root: Path = REPOSITORY,
) -> dict:
    """Inventory an overlay and reject every unregistered byte difference."""
    overlay_root = Path(overlay_root).resolve()
    base_root = Path(base_root).resolve()
    source_root = Path(source_root).resolve()
    if not overlay_root.is_dir() or overlay_root.is_symlink():
        raise LiveRunnerError("overlay_root_missing_or_symlink")
    symlinks = sorted(path for path in overlay_root.rglob("*") if path.is_symlink())
    if symlinks:
        raise LiveRunnerError(
            "overlay_contains_symlink:" + str(symlinks[0].relative_to(overlay_root))
        )
    files = sorted(path for path in overlay_root.rglob("*") if path.is_file())
    if not files:
        raise LiveRunnerError("overlay_inventory_empty")
    hashes = {}
    base_hashes = {}
    differences = []
    for path in files:
        relative = str(path.relative_to(overlay_root))
        base = base_root / relative
        if not base.is_file() or base.is_symlink():
            raise LiveRunnerError(f"overlay_file_missing_from_base:{relative}")
        overlay_hash = _sha256(path)
        base_hash = _sha256(base)
        hashes[relative] = overlay_hash
        base_hashes[relative] = base_hash
        if overlay_hash == base_hash and path.read_bytes() == base.read_bytes():
            continue
        if relative not in allowed_differences:
            raise LiveRunnerError(f"unauthorized_overlay_difference:{relative}")
        source = source_root / allowed_differences[relative]
        if not source.is_file() or source.is_symlink() \
                or _sha256(source) != overlay_hash \
                or source.read_bytes() != path.read_bytes():
            raise LiveRunnerError(f"overlay_alias_source_mismatch:{relative}")
        differences.append(relative)
    return {
        "file_count": len(files),
        "file_sha256": hashes,
        "base_file_sha256": base_hashes,
        "authorized_differences": sorted(differences),
        "binary_library_bytes_equal": True,
    }


def build_overlay_manifest_payload(
    current_commit: str, dependency: dict, inventory: dict,
    installed_aliases: dict | None = None,
    evidence_bindings: dict | None = None,
    retained_artifacts: dict | None = None,
    repair_binding: dict | None = None,
    import_probe: dict | None = None,
) -> dict:
    if re.fullmatch(r"[0-9a-f]{40}", current_commit) is None:
        raise LiveRunnerError("overlay_commit_malformed")
    corrected_paths = (
        "launch/test_planner.launch.py",
        "launch/icra_p0_p5_qualification.py",
        "config/icra27/icra_p0_p5_qualification_v1.json",
    )
    return {
        "schema_version": "icra070_isolated_overlay_manifest_v2",
        "product_build": {
            "task_id": "ICRA-068",
            "git_commit": PRODUCT_COMMIT,
            "manifest_path": str(INSTALL_MANIFEST_PATH.relative_to(REPOSITORY)),
            "manifest_sha256": PRODUCT_MANIFEST_SHA256,
            "install_root": str(INSTALL_ROOT),
            "runtime_library_count": 54,
        },
        "corrected_full_sensor_contract": {
            "task_id": "ICRA-070",
            "git_commit": current_commit,
            "file_sha256": {
                relative: _sha256(REPOSITORY / relative)
                for relative in corrected_paths
            },
        },
        "runner": {
            "task_id": "ICRA-070",
            "git_commit": current_commit,
            "path": str(Path(__file__).resolve().relative_to(REPOSITORY)),
            "sha256": _sha256(Path(__file__).resolve()),
        },
        "install_root": str(OVERLAY_INSTALL_ROOT),
        "runtime_prefix_order": [
            str(OVERLAY_INSTALL_ROOT), str(INSTALL_ROOT),
            "/root/ros2_ws/install", "/opt/ros/jazzy",
        ],
        "package_resolution": {
            "iap": str(OVERLAY_INSTALL_ROOT),
            "unchanged_local_packages": str(INSTALL_ROOT),
            "glim_ros": "/root/ros2_ws/install/glim_ros",
            "glim": "/root/ros2_ws/install/glim",
            "ros": "/opt/ros/jazzy",
        },
        "dependency_preflight": dependency,
        "evidence_bindings": dict(evidence_bindings or {}),
        "retained_artifacts": dict(retained_artifacts or {}),
        "cache_boundary": {
            "all_python_cache_excluded": True,
            "python_cache_allowlist": [],
            "pythondontwritebytecode": "1",
            "repair_binding": dict(repair_binding or {}),
            "installed_import_probe": dict(import_probe or {}),
        },
        "overlay_inventory": inventory,
        "installed_aliases": dict(installed_aliases or {}),
        "binary_library_bytes_equal": (
            inventory.get("binary_library_bytes_equal") is True
        ),
        "no_compile_install": True,
    }


def verify_package_identities(
    prefixes: list[Path], expected: dict[str, Path | tuple[Path, ...]], resolver,
) -> dict[str, dict[str, object]]:
    """Bind the real resolver result and reject unregistered duplicate identities."""
    result = {}
    canonical_prefixes = [Path(prefix).resolve() for prefix in prefixes]
    for package, expected_value in expected.items():
        allowed = (
            expected_value
            if isinstance(expected_value, tuple) else (expected_value,)
        )
        allowed = tuple(Path(prefix).resolve() for prefix in allowed)
        identities = [
            prefix for prefix in canonical_prefixes
            if (
                prefix / "share/ament_index/resource_index/packages" / package
            ).is_file()
        ]
        if tuple(identities) != allowed:
            raise LiveRunnerError(
                f"duplicate_or_stale_package_identity:{package}:"
                + ",".join(str(path) for path in identities)
            )
        resolved = Path(resolver(package)).resolve()
        if resolved != allowed[0]:
            raise LiveRunnerError(
                f"package_resolver_mismatch:{package}:{resolved}"
            )
        result[package] = {
            "resolved_prefix": str(resolved),
            "registered_identities": [str(path) for path in identities],
        }
    return result


def verify_runtime_package_resolution() -> dict:
    overlay_index = (
        OVERLAY_INSTALL_ROOT / "share/ament_index/resource_index/packages"
    )
    if not overlay_index.is_dir() or overlay_index.is_symlink():
        raise LiveRunnerError("overlay_package_index_missing")
    overlay_packages = sorted(
        path.name for path in overlay_index.iterdir()
        if path.is_file() and not path.is_symlink()
    )
    if overlay_packages != ["iap"]:
        raise LiveRunnerError(
            "overlay_package_identity_mismatch:" + ",".join(overlay_packages)
        )
    from ament_index_python.packages import get_package_prefix

    ament_prefixes = [
        Path(value) for value in expected_live_environment()[
            "AMENT_PREFIX_PATH"
        ].split(":")
    ]
    expected_identities = {
        package: (
            (OVERLAY_INSTALL_ROOT, INSTALL_ROOT)
            if package == "iap" else INSTALL_ROOT
        )
        for package in LOCAL_PACKAGES
    }
    expected_identities["rclcpp_components"] = Path("/opt/ros/jazzy")
    ament_resolution = verify_package_identities(
        ament_prefixes, expected_identities, get_package_prefix
    )
    cmake_prefixes = [
        Path(value) for value in expected_live_environment()[
            "CMAKE_PREFIX_PATH"
        ].split(":")
    ]
    glim_expected = {
        "glim_ros": Path("/root/ros2_ws/install/glim_ros"),
        "glim": Path("/root/ros2_ws/install/glim"),
    }
    glim_resolution = verify_package_identities(
        cmake_prefixes, glim_expected,
        lambda package: str(glim_expected[package]),
    )
    executable_resolution = {}
    for package, names in RUNTIME_EXECUTABLES.items():
        root = OVERLAY_INSTALL_ROOT if package == "iap" else INSTALL_ROOT
        for name in names:
            path = root / "lib" / package / name
            if not path.is_file() or path.is_symlink() \
                    or not os.access(path, os.X_OK):
                raise LiveRunnerError(
                    f"runtime_executable_unresolved:{package}/{name}"
                )
            executable_resolution[f"{package}/{name}"] = str(path)
    return {
        "overlay_packages": overlay_packages,
        "iap_prefix": str(OVERLAY_INSTALL_ROOT),
        "unchanged_local_prefix": str(INSTALL_ROOT),
        "ament_resolution": ament_resolution,
        "glim_resolution": glim_resolution,
        "executable_resolution": executable_resolution,
    }


def _current_commit() -> str:
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=REPOSITORY, text=True
    ).strip()


def prepare_overlay() -> dict:
    """The original one-shot installer is frozen after its preserved blocker."""
    raise LiveRunnerError("overlay_prepare_superseded_by_single_cache_repair")


def _superseded_prepare_overlay_implementation() -> dict:
    """Preserved implementation; unreachable after the reviewed v1 blocker."""
    if OVERLAY_INSTALL_ROOT.exists() or OVERLAY_INSTALL_ROOT.is_symlink():
        raise LiveRunnerError("overlay_install_already_exists")
    for path in (OVERLAY_MANIFEST_PATH, DEPENDENCY_PREFLIGHT_PATH, OVERLAY_COMMAND_PATH):
        if path.exists() or path.is_symlink():
            raise LiveRunnerError(f"overlay_evidence_already_exists:{path.name}")
    validate_live_environment()
    if _sha256(INSTALL_MANIFEST_PATH) != PRODUCT_MANIFEST_SHA256:
        raise LiveRunnerError("product_manifest_sha256_mismatch")
    validate_frozen_install_manifest(INSTALL_MANIFEST_PATH, PRODUCT_COMMIT)
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=REPOSITORY, text=True,
    ).strip()
    if dirty:
        raise LiveRunnerError("tracked_worktree_not_clean")
    current_commit = _current_commit()
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    resolve_gnss_dependencies(contract, INSTALL_ROOT)
    retained_before = tree_byte_inventory(PRODUCT_TASK_ROOT)
    retained_summary = inventory_summary(retained_before)
    driver = write_overlay_install_driver()
    command = overlay_install_command()
    install_environment = overlay_install_environment()
    completed = subprocess.run(
        command, cwd=REPOSITORY, text=True, capture_output=True, check=False,
        env=install_environment,
    )
    retained_after = tree_byte_inventory(PRODUCT_TASK_ROOT)
    retained_after_summary = inventory_summary(retained_after)
    retained_artifacts = {
        "root": str(PRODUCT_TASK_ROOT.relative_to(REPOSITORY)),
        "before": retained_summary,
        "after": retained_after_summary,
        "byte_inventory_equal": retained_before == retained_after,
    }
    command_record = {
        "schema_version": "icra070_overlay_install_command_v1",
        "argv": command,
        "cwd": str(REPOSITORY),
        "environment": install_environment,
        "exit_code": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
        "compile_invocations": 0,
        "install_driver": driver,
        "retained_artifacts": retained_artifacts,
    }
    _write_json(OVERLAY_COMMAND_PATH, command_record)
    if retained_before != retained_after:
        raise LiveRunnerError("product_artifacts_mutated_by_overlay_install")
    if completed.returncode != 0:
        raise LiveRunnerError(f"overlay_install_failed:{completed.returncode}")
    dependency = resolve_gnss_dependencies(
        contract, INSTALL_ROOT, OVERLAY_INSTALL_ROOT
    )
    _write_json(DEPENDENCY_PREFLIGHT_PATH, dependency)
    inventory = inventory_overlay(
        OVERLAY_INSTALL_ROOT, INSTALL_ROOT, INSTALLED_ALIASES
    )
    aliases = verify_installed_aliases(OVERLAY_INSTALL_ROOT)
    package_resolution = verify_runtime_package_resolution()
    evidence_bindings = {
        "dependency_preflight_path": str(
            DEPENDENCY_PREFLIGHT_PATH.relative_to(REPOSITORY)
        ),
        "dependency_preflight_sha256": _sha256(DEPENDENCY_PREFLIGHT_PATH),
        "overlay_install_command_path": str(
            OVERLAY_COMMAND_PATH.relative_to(REPOSITORY)
        ),
        "overlay_install_command_sha256": _sha256(OVERLAY_COMMAND_PATH),
    }
    manifest = build_overlay_manifest_payload(
        current_commit, dependency, inventory, aliases,
        evidence_bindings, retained_artifacts,
    )
    manifest["verified_package_resolution"] = package_resolution
    _write_json(OVERLAY_MANIFEST_PATH, manifest)
    return manifest


def repair_probe_environment() -> dict[str, str]:
    return {
        **expected_live_environment(),
        "PATH": "/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
    }


def repair_overlay_cache() -> dict:
    """Perform the single authorized cache-only repair and freeze v2 proof."""
    require_repair_outputs_absent([
        REPAIR_EVIDENCE_PATH, OVERLAY_MANIFEST_PATH, ADOPTION_MANIFEST_PATH,
    ])
    validate_live_environment()
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=REPOSITORY, text=True,
    ).strip()
    if dirty:
        raise LiveRunnerError("tracked_worktree_not_clean")
    current_commit = _current_commit()
    original_blocker = verify_original_blocker_evidence()
    retained_before_inventory = tree_byte_inventory(PRODUCT_TASK_ROOT)
    retained_before = inventory_summary(retained_before_inventory)
    if retained_before != EXPECTED_RETAINED_TASK_INVENTORY:
        raise LiveRunnerError("retained_icra068_inventory_mismatch")
    if not OVERLAY_INSTALL_ROOT.is_dir() or OVERLAY_INSTALL_ROOT.is_symlink():
        raise LiveRunnerError("failed_overlay_missing_or_symlink")
    pre_repair_inventory = tree_byte_inventory(OVERLAY_INSTALL_ROOT)
    pre_repair_summary = inventory_summary(pre_repair_inventory)
    if pre_repair_summary != EXPECTED_FAILED_OVERLAY_INVENTORY:
        raise LiveRunnerError("failed_overlay_frozen_inventory_mismatch")
    source_cache = python_cache_inventory(REPOSITORY / "launch")
    repair_proof = repair_overlay_cache_boundary(
        OVERLAY_INSTALL_ROOT, INSTALL_ROOT, REPOSITORY,
        INSTALLED_ALIASES, pre_repair_inventory,
        expected_source_cache=source_cache,
    )
    retained_after = inventory_summary(tree_byte_inventory(PRODUCT_TASK_ROOT))
    if retained_after != retained_before:
        raise LiveRunnerError("retained_icra068_mutated_by_cache_repair")
    import_probe = run_no_bytecode_import_probe(
        OVERLAY_INSTALL_ROOT,
        [
            OVERLAY_INSTALL_ROOT
            / "share/iap/launch/icra_p0_p5_qualification.py",
            OVERLAY_INSTALL_ROOT / "share/iap/launch/test_planner.launch.py",
        ],
        repair_probe_environment(),
    )
    post_probe_inventory = tree_byte_inventory(OVERLAY_INSTALL_ROOT)
    if inventory_summary(post_probe_inventory) != import_probe[
            "install_inventory_after"]:
        raise LiveRunnerError("import_probe_post_inventory_binding_mismatch")
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    dependency = resolve_gnss_dependencies(
        contract, INSTALL_ROOT, OVERLAY_INSTALL_ROOT
    )
    dependency_record = json.loads(DEPENDENCY_PREFLIGHT_PATH.read_text())
    if dependency_record != dependency:
        raise LiveRunnerError("preserved_dependency_preflight_mismatch")
    aliases = verify_installed_aliases(OVERLAY_INSTALL_ROOT)
    package_resolution = verify_runtime_package_resolution()
    repair_record = {
        "schema_version": "icra070_overlay_cache_repair_v1",
        "status": "PASS",
        "task_id": "ICRA-070",
        "git_commit": current_commit,
        "command": {
            "argv": [
                "python3",
                "scripts/dev_planner/run_icra_p0_p5_qualification.py",
                "--repair-overlay-cache",
            ],
            "cwd": str(REPOSITORY),
            "environment": expected_live_environment(),
            "exit_code": 0,
        },
        "original_blocker": original_blocker,
        "retained_icra068": {
            "root": str(PRODUCT_TASK_ROOT.relative_to(REPOSITORY)),
            "before": retained_before,
            "after": retained_after,
            "byte_inventory_equal": retained_before == retained_after,
        },
        "failed_overlay": {
            "root": str(OVERLAY_INSTALL_ROOT.relative_to(REPOSITORY)),
            "pre_repair_summary": pre_repair_summary,
            "pre_repair_inventory": pre_repair_inventory,
        },
        "repair_proof": repair_proof,
        "installed_import_probe": import_probe,
        "post_probe_inventory": post_probe_inventory,
        "source_cache_inventory": source_cache,
        "source_cache_unchanged": (
            python_cache_inventory(REPOSITORY / "launch") == source_cache
        ),
        "dependency_preflight_sha256": _sha256(DEPENDENCY_PREFLIGHT_PATH),
        "package_resolution": package_resolution,
        "reinstall_invocations": 0,
        "compile_invocations": 0,
        "repair_invocations": 1,
    }
    if repair_record["source_cache_unchanged"] is not True:
        raise LiveRunnerError("source_cache_mutated_by_repair_or_probe")
    _write_json_exclusive(REPAIR_EVIDENCE_PATH, repair_record)
    retained_binding = {
        "root": str(PRODUCT_TASK_ROOT.relative_to(REPOSITORY)),
        "before": retained_before,
        "after": retained_after,
        "byte_inventory_equal": True,
    }
    evidence_bindings = {
        "dependency_preflight_path": str(
            DEPENDENCY_PREFLIGHT_PATH.relative_to(REPOSITORY)
        ),
        "dependency_preflight_sha256": _sha256(DEPENDENCY_PREFLIGHT_PATH),
        "original_overlay_command_path": str(
            OVERLAY_COMMAND_PATH.relative_to(REPOSITORY)
        ),
        "original_overlay_command_sha256": _sha256(OVERLAY_COMMAND_PATH),
    }
    repair_binding = {
        "path": str(REPAIR_EVIDENCE_PATH.relative_to(REPOSITORY)),
        "sha256": _sha256(REPAIR_EVIDENCE_PATH),
    }
    manifest = build_overlay_manifest_payload(
        current_commit, dependency, repair_proof, aliases,
        evidence_bindings, retained_binding,
        repair_binding, import_probe,
    )
    manifest["verified_package_resolution"] = package_resolution
    _write_json_exclusive(OVERLAY_MANIFEST_PATH, manifest)
    if validate_overlay_manifest(current_commit) != manifest:
        raise LiveRunnerError("overlay_manifest_v2_self_validation_mismatch")
    return manifest


def validate_overlay_manifest(current_commit: str) -> dict:
    if not OVERLAY_MANIFEST_PATH.is_file() or OVERLAY_MANIFEST_PATH.is_symlink():
        raise LiveRunnerError("overlay_manifest_missing_or_symlink")
    try:
        observed = json.loads(OVERLAY_MANIFEST_PATH.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveRunnerError("overlay_manifest_malformed") from exc
    for path, label in (
        (REPAIR_EVIDENCE_PATH, "cache_repair_evidence"),
        (DEPENDENCY_PREFLIGHT_PATH, "dependency_preflight"),
        (OVERLAY_COMMAND_PATH, "original_overlay_install_command"),
        (OVERLAY_INSTALL_DRIVER_PATH, "original_overlay_install_driver"),
    ):
        if not path.is_file() or path.is_symlink():
            raise LiveRunnerError(f"{label}_missing_or_symlink")
    try:
        repair = json.loads(REPAIR_EVIDENCE_PATH.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveRunnerError("cache_repair_evidence_malformed") from exc
    original_blocker = verify_original_blocker_evidence()
    retained_summary = inventory_summary(tree_byte_inventory(PRODUCT_TASK_ROOT))
    pre_repair = repair.get("failed_overlay", {}).get("pre_repair_inventory")
    source_cache = repair.get("source_cache_inventory")
    if not (
        repair.get("schema_version") == "icra070_overlay_cache_repair_v1"
        and repair.get("status") == "PASS"
        and repair.get("git_commit") == current_commit
        and repair.get("original_blocker") == original_blocker
        and repair.get("failed_overlay", {}).get("pre_repair_summary")
        == EXPECTED_FAILED_OVERLAY_INVENTORY
        and isinstance(pre_repair, dict)
        and inventory_summary(pre_repair) == EXPECTED_FAILED_OVERLAY_INVENTORY
        and repair.get("retained_icra068", {}).get("before")
        == EXPECTED_RETAINED_TASK_INVENTORY
        and repair.get("retained_icra068", {}).get("after")
        == EXPECTED_RETAINED_TASK_INVENTORY
        and repair.get("retained_icra068", {}).get("byte_inventory_equal") is True
        and retained_summary == EXPECTED_RETAINED_TASK_INVENTORY
        and repair.get("source_cache_unchanged") is True
        and python_cache_inventory(REPOSITORY / "launch") == source_cache
        and repair.get("compile_invocations") == 0
        and repair.get("reinstall_invocations") == 0
        and repair.get("repair_invocations") == 1
    ):
        raise LiveRunnerError("cache_repair_evidence_binding_mismatch")
    inventory = inventory_overlay_v2(
        OVERLAY_INSTALL_ROOT, INSTALL_ROOT, REPOSITORY,
        INSTALLED_ALIASES, pre_repair,
    )
    if inventory != repair.get("repair_proof"):
        repair_proof = dict(repair.get("repair_proof") or {})
        for key in (
            "removed_cache_files", "removed_cache_inventory",
            "removed_cache_directories", "source_cache_inventory",
            "source_cache_unchanged",
        ):
            repair_proof.pop(key, None)
        if inventory != repair_proof:
            raise LiveRunnerError("repaired_overlay_inventory_binding_mismatch")
    current_overlay_summary = inventory_summary(
        tree_byte_inventory(OVERLAY_INSTALL_ROOT)
    )
    probe = repair.get("installed_import_probe")
    if not (
        isinstance(probe, dict)
        and probe.get("exit_code") == 0
        and probe.get("environment", {}).get("PYTHONDONTWRITEBYTECODE") == "1"
        and probe.get("install_inventory_unchanged") is True
        and probe.get("python_cache_absent_after") is True
        and probe.get("install_inventory_after") == current_overlay_summary
        and repair.get("post_probe_inventory")
        == tree_byte_inventory(OVERLAY_INSTALL_ROOT)
    ):
        raise LiveRunnerError("installed_import_probe_binding_mismatch")
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    dependency = resolve_gnss_dependencies(
        contract, INSTALL_ROOT, OVERLAY_INSTALL_ROOT
    )
    if json.loads(DEPENDENCY_PREFLIGHT_PATH.read_text()) != dependency:
        raise LiveRunnerError("dependency_preflight_binding_mismatch")
    aliases = verify_installed_aliases(OVERLAY_INSTALL_ROOT)
    package_resolution = verify_runtime_package_resolution()
    retained_artifacts = {
        "root": str(PRODUCT_TASK_ROOT.relative_to(REPOSITORY)),
        "before": retained_summary,
        "after": retained_summary,
        "byte_inventory_equal": True,
    }
    evidence_bindings = {
        "dependency_preflight_path": str(
            DEPENDENCY_PREFLIGHT_PATH.relative_to(REPOSITORY)
        ),
        "dependency_preflight_sha256": _sha256(DEPENDENCY_PREFLIGHT_PATH),
        "original_overlay_command_path": str(
            OVERLAY_COMMAND_PATH.relative_to(REPOSITORY)
        ),
        "original_overlay_command_sha256": _sha256(OVERLAY_COMMAND_PATH),
    }
    repair_binding = {
        "path": str(REPAIR_EVIDENCE_PATH.relative_to(REPOSITORY)),
        "sha256": _sha256(REPAIR_EVIDENCE_PATH),
    }
    expected = build_overlay_manifest_payload(
        current_commit, dependency, inventory, aliases,
        evidence_bindings, retained_artifacts,
        repair_binding, probe,
    )
    expected["verified_package_resolution"] = package_resolution
    if observed != expected:
        raise LiveRunnerError("overlay_manifest_payload_mismatch")
    return observed


def canonical_empty_defaults(contract: dict) -> set[str]:
    observed = []
    for case_id, _ in LIVE_IDENTITIES:
        values = QUALIFICATION.resolve_launch_values(contract, case_id, {})
        observed.append({key for key, value in values.items() if value == ""})
    if any(keys != REGISTERED_EMPTY_DEFAULT_KEYS for keys in observed):
        raise LiveRunnerError("canonical_empty_default_set_mismatch")
    return set(REGISTERED_EMPTY_DEFAULT_KEYS)


def render_live_launch_command(
    config, allowed_empty_keys: set[str]
) -> tuple[list[str], list[str]]:
    """Render unique ROS overrides while omitting only registered empty defaults."""
    items = list(config.items()) if isinstance(config, dict) else list(config)
    command = ["ros2", "launch", "iap", "test_planner.launch.py"]
    omitted = []
    seen = set()
    for name, value in items:
        if not isinstance(name, str) or re.fullmatch(r"[A-Za-z0-9_./-]+", name) is None:
            raise LiveRunnerError(f"malformed_override_name:{name}")
        if name in seen:
            raise LiveRunnerError(f"duplicate_override:{name}")
        seen.add(name)
        if value == "":
            if name not in allowed_empty_keys:
                raise LiveRunnerError(f"unregistered_empty_override:{name}")
            omitted.append(name)
            continue
        if isinstance(value, bool):
            rendered = "true" if value else "false"
        elif isinstance(value, int):
            rendered = str(value)
        elif isinstance(value, float) and math.isfinite(value):
            rendered = str(value)
        elif isinstance(value, str) and not any(
            ord(character) < 32 or ord(character) == 127 for character in value
        ) and ":=" not in value:
            rendered = value
        else:
            raise LiveRunnerError(f"malformed_override_value:{name}")
        token = f"{name}:={rendered}"
        if token.endswith(":=") or token.count(":=") != 1:
            raise LiveRunnerError(f"malformed_override_token:{name}")
        command.append(token)
    unregistered_omissions = set(omitted) - set(allowed_empty_keys)
    if unregistered_omissions:
        raise LiveRunnerError("unregistered_omission")
    return command, omitted


def build_adoption_payload(
    current_commit: str, changed_files: list[str],
    overlay_manifest: dict | None = None,
    overlay_manifest_sha256: str | None = None,
) -> dict:
    if re.fullmatch(r"[0-9a-f]{40}", current_commit) is None:
        raise LiveRunnerError("runner_commit_malformed")
    if not INSTALL_MANIFEST_PATH.is_file() or INSTALL_MANIFEST_PATH.is_symlink():
        raise LiveRunnerError("product_manifest_missing_or_symlink")
    manifest_hash = _sha256(INSTALL_MANIFEST_PATH)
    if manifest_hash != PRODUCT_MANIFEST_SHA256:
        raise LiveRunnerError("product_manifest_sha256_mismatch")
    try:
        product_manifest = json.loads(INSTALL_MANIFEST_PATH.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveRunnerError("product_manifest_malformed") from exc
    if product_manifest.get("git_commit") != PRODUCT_COMMIT:
        raise LiveRunnerError("product_manifest_commit_mismatch")
    if overlay_manifest is None:
        overlay_manifest = validate_overlay_manifest(current_commit)
        overlay_manifest_sha256 = _sha256(OVERLAY_MANIFEST_PATH)
    if overlay_manifest.get("schema_version") \
            != "icra070_isolated_overlay_manifest_v2" \
            or overlay_manifest.get("runner", {}).get("git_commit") \
            != current_commit \
            or not isinstance(overlay_manifest_sha256, str) \
            or re.fullmatch(r"[0-9a-f]{64}", overlay_manifest_sha256) is None:
        raise LiveRunnerError("overlay_adoption_manifest_mismatch")
    alias_sources = set(INSTALLED_ALIASES.values())
    authorized_overlap = sorted(alias_sources.intersection(changed_files))
    changed_hashes = {
        relative: _sha256(REPOSITORY / relative)
        for relative in changed_files
        if (REPOSITORY / relative).is_file()
        and not (REPOSITORY / relative).is_symlink()
    }
    return {
        "schema_version": "icra070_overlay_adoption_v2",
        "product_install": {
            "task_id": "ICRA-068",
            "git_commit": PRODUCT_COMMIT,
            "manifest_path": str(INSTALL_MANIFEST_PATH.relative_to(REPOSITORY)),
            "manifest_sha256": manifest_hash,
            "install_root": str(INSTALL_ROOT),
            "installed_aliases": product_manifest.get("installed_aliases"),
            "runtime_library_count": len(
                product_manifest.get("runtime_libraries", [])
            ),
            "file_hash_count": len(product_manifest.get("file_hashes", {})),
        },
        "runner_analyzer": {
            "task_id": "ICRA-070",
            "git_commit": current_commit,
            "runner_path": str(Path(__file__).resolve().relative_to(REPOSITORY)),
            "runner_sha256": _sha256(Path(__file__).resolve()),
            "analyzer_path": str(
                (REPOSITORY / "launch/icra_p0_p5_qualification.py").relative_to(
                    REPOSITORY
                )
            ),
            "analyzer_sha256": _sha256(
                REPOSITORY / "launch/icra_p0_p5_qualification.py"
            ),
        },
        "isolated_overlay": {
            "manifest_path": str(OVERLAY_MANIFEST_PATH.relative_to(REPOSITORY)),
            "manifest_sha256": overlay_manifest_sha256,
            "install_root": str(OVERLAY_INSTALL_ROOT),
            "authorized_differences": overlay_manifest.get(
                "overlay_inventory", {}
            ).get("authorized_differences", []),
            "binary_library_bytes_equal": overlay_manifest.get(
                "binary_library_bytes_equal"
            ),
        },
        "post_product_changed_files": list(changed_files),
        "post_product_file_sha256": changed_hashes,
        "installed_runtime_source_overlap": authorized_overlap,
        "unauthorized_installed_runtime_source_overlap": [],
        "product_binary_runtime_unchanged": True,
    }


def parse_only_command(rendered_command: list[str]) -> list[str]:
    if rendered_command[:4] != [
        "ros2", "launch", "iap", "test_planner.launch.py"
    ]:
        raise LiveRunnerError("parse_command_launch_identity_mismatch")
    if any(token.endswith(":=") for token in rendered_command[4:]):
        raise LiveRunnerError("parse_command_contains_bare_empty_override")
    return [
        "ros2", "launch", "--show-args", *rendered_command[2:]
    ]


def full_sensor_resolution(contract: dict) -> dict:
    values = contract["qualification_values"]
    return {
        "scenario": QUALIFICATION.FULL_SENSOR_SCENARIO,
        "use_gnss": values["use_gnss"],
        "use_araim": values["use_araim"],
        "enable_gnss_integrity": values["enable_gnss_integrity"],
        "enable_gnss_araim": values["enable_gnss_araim"],
        "enable_lidar_integrity": values["enable_lidar_integrity"],
        "integrity_fusion_mode": values["integrity_fusion_mode"],
        "validator_require_gnss_valid": values[
            "validator_require_gnss_valid"
        ],
        "validator_require_lidar_valid": values[
            "validator_require_lidar_valid"
        ],
        "gnss_time_source": values["gnss_time_source"],
        "gnss_trigger_topic": values["gnss_trigger_topic"],
        "gnss_fallback_to_synthetic_on_rinex_error": values[
            "gnss_fallback_to_synthetic_on_rinex_error"
        ],
        "gnss_scenario_path": values["gnss_scenario_file"],
        "rinex_ephemeris_path": values["gnss_rinex_nav_file"],
        "p0_predictor": {
            "source_mode": contract["profile_values"][
                "p0.predictor.source_mode"
            ],
            "gnss_epoch_policy": contract["profile_values"][
                "p0.predictor.gnss_epoch_policy"
            ],
            "use_current_integrity_prior": contract["profile_values"][
                "p0.predictor.use_current_integrity_prior"
            ],
            "worker_count": contract["profile_values"][
                "p0.predictor.worker_count"
            ],
            "sigma_grow_m_sqrt_s": contract["profile_values"][
                "p0.predictor.sigma_grow_m_sqrt_s"
            ],
            "sigma_growth_profile": contract["profile_values"][
                "p0.predictor.sigma_growth_profile"
            ],
        },
    }


def full_sensor_launch_overrides(contract: dict) -> dict[str, str]:
    resolution = full_sensor_resolution(contract)
    return {
        "gnss_scenario_file": resolution["gnss_scenario_path"],
        "gnss_rinex_nav_file": resolution["rinex_ephemeris_path"],
        "gnss_trigger_topic": resolution["gnss_trigger_topic"],
        "gnss_fallback_to_synthetic_on_rinex_error": "false",
        "gnss_time_source": resolution["gnss_time_source"],
    }


def _process_group_rows(pgid: int) -> list[dict[str, object]]:
    import psutil

    rows = []
    for process in psutil.process_iter(["pid", "cmdline"]):
        try:
            if os.getpgid(process.pid) != pgid:
                continue
            rows.append({
                "pid": int(process.pid),
                "cmdline": " ".join(process.cmdline() or []),
            })
        except (OSError, psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return rows


def _run_parse_only(command: list[str], output_root: Path) -> dict:
    stdout_path = output_root / "stdout.log"
    stderr_path = output_root / "stderr.log"
    observed_required = set()
    observed_group_pids = set()
    timed_out = False
    if os.environ.get("PYTHONDONTWRITEBYTECODE") != "1":
        raise LiveRunnerError("parse_bytecode_not_disabled")
    child_environment = os.environ.copy()
    with stdout_path.open("w") as stdout, stderr_path.open("w") as stderr:
        process = subprocess.Popen(
            command, stdout=stdout, stderr=stderr, start_new_session=True,
            env=child_environment,
        )
        pgid = process.pid
        deadline = time.monotonic() + 30.0
        while process.poll() is None and time.monotonic() < deadline:
            for row in _process_group_rows(pgid):
                observed_group_pids.add(int(row["pid"]))
                cmdline = str(row["cmdline"])
                for identity, markers in REQUIRED_PROCESSES.items():
                    if int(row["pid"]) != process.pid and any(
                        marker in cmdline for marker in markers
                    ):
                        observed_required.add(identity)
            time.sleep(0.01)
        if process.poll() is None:
            timed_out = True
            os.killpg(pgid, signal.SIGKILL)
        exit_code = process.wait(timeout=5.0)
    remaining = _process_group_rows(pgid)
    result = {
        "argv": command,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "observed_process_group_pids": sorted(observed_group_pids),
        "observed_required_processes": sorted(observed_required),
        "remaining_process_group_pids": sorted(
            int(row["pid"]) for row in remaining
        ),
        "task_owned_process_audit_passed": (
            not timed_out and not observed_required and not remaining
        ),
        "subprocess_environment": {"PYTHONDONTWRITEBYTECODE": "1"},
        "stdout_path": str(stdout_path.relative_to(REPOSITORY)),
        "stdout_sha256": _sha256(stdout_path),
        "stderr_path": str(stderr_path.relative_to(REPOSITORY)),
        "stderr_sha256": _sha256(stderr_path),
    }
    result["parse_passed"] = (
        exit_code == 0 and result["task_owned_process_audit_passed"]
    )
    return result


def run_parse_only_proofs(contract: dict, parse_root: Path) -> dict:
    parse_root = Path(parse_root).resolve()
    if parse_root != (TASK_ROOT / "parse_only").resolve():
        raise LiveRunnerError("parse_root_identity_mismatch")
    if parse_root.exists() or parse_root.is_symlink():
        raise LiveRunnerError("parse_evidence_already_exists")
    parse_root.mkdir(parents=True)
    allowed_empty = canonical_empty_defaults(contract)
    required_sensor_overrides = full_sensor_launch_overrides(contract)
    cases = []
    for case_id, run_id in LIVE_IDENTITIES:
        case_root = parse_root / case_id.lower()
        case_root.mkdir()
        render_dir = TASK_ROOT / "live" / run_id
        config = live_config(contract, case_id, run_id, render_dir)
        rendered, omitted = render_live_launch_command(config, allowed_empty)
        command = parse_only_command(rendered)
        result = _run_parse_only(command, case_root)
        result.update({
            "case_id": case_id,
            "replacement_run_id": run_id,
            "rendered_live_argv": rendered,
            "omitted_empty_defaults": omitted,
            "full_sensor_overrides": {
                name: value for name, value in required_sensor_overrides.items()
                if f"{name}:={value}" in rendered
            },
        })
        cases.append(result)
    summary = {
        "schema_version": "icra070_ros_launch_parser_proof_v1",
        "case_order": [case_id for case_id, _ in LIVE_IDENTITIES],
        "cases": cases,
        "parse_invocations": len(cases),
        "main_flow_child_invocations": sum(
            len(case["observed_required_processes"]) for case in cases
        ),
        "parse_ready": len(cases) == 3 and all(
            case["parse_passed"] for case in cases
        ),
        "full_sensor_resolution": full_sensor_resolution(contract),
        "required_full_sensor_overrides": required_sensor_overrides,
    }
    _write_json(parse_root / "parser_proof.json", summary)
    return summary


def parse_proof_failures(
    proof: object, contract: dict, parse_root: Path | None = None,
) -> list[str]:
    """Validate the exact three-case installed-parser proof and its raw output."""
    parse_root = Path(parse_root or (TASK_ROOT / "parse_only")).resolve()
    failures = []
    if not isinstance(proof, dict):
        return ["parse proof must be an object"]
    expected_order = [case_id for case_id, _ in LIVE_IDENTITIES]
    if proof.get("schema_version") != "icra070_ros_launch_parser_proof_v1":
        failures.append("parse proof schema mismatch")
    if proof.get("case_order") != expected_order:
        failures.append("parse proof case order mismatch")
    cases = proof.get("cases")
    if not isinstance(cases, list) or len(cases) != len(LIVE_IDENTITIES):
        failures.append("parse proof case cardinality mismatch")
        cases = []
    if proof.get("parse_invocations") != 3:
        failures.append("parse proof invocation count mismatch")
    if proof.get("main_flow_child_invocations") != 0:
        failures.append("parse proof main-flow child audit mismatch")
    if proof.get("parse_ready") is not True:
        failures.append("parse proof is not ready")
    if proof.get("full_sensor_resolution") != full_sensor_resolution(contract):
        failures.append("parse proof full-sensor resolution mismatch")
    required_sensor_overrides = full_sensor_launch_overrides(contract)
    if proof.get("required_full_sensor_overrides") != required_sensor_overrides:
        failures.append("parse proof full-sensor override binding mismatch")
    allowed_empty = canonical_empty_defaults(contract)
    for index, (case_id, run_id) in enumerate(LIVE_IDENTITIES):
        if index >= len(cases) or not isinstance(cases[index], dict):
            failures.append(f"parse case[{case_id}] missing or malformed")
            continue
        case = cases[index]
        config = live_config(
            contract, case_id, run_id, TASK_ROOT / "live" / run_id
        )
        expected_live, expected_omitted = render_live_launch_command(
            config, allowed_empty
        )
        expected_parse = parse_only_command(expected_live)
        if case.get("case_id") != case_id \
                or case.get("replacement_run_id") != run_id:
            failures.append(f"parse case[{case_id}] identity mismatch")
        if case.get("rendered_live_argv") != expected_live:
            failures.append(f"parse case[{case_id}] live argv mismatch")
        if case.get("full_sensor_overrides") != required_sensor_overrides:
            failures.append(f"parse case[{case_id}] full-sensor argv mismatch")
        if case.get("omitted_empty_defaults") != expected_omitted:
            failures.append(f"parse case[{case_id}] omission set mismatch")
        if case.get("argv") != expected_parse:
            failures.append(f"parse case[{case_id}] argv mismatch")
        observed_pids = case.get("observed_process_group_pids")
        if not (
            isinstance(observed_pids, list)
            and len(observed_pids) == len(set(observed_pids))
            and all(isinstance(pid, int) and pid > 0 for pid in observed_pids)
        ):
            failures.append(f"parse case[{case_id}] process inventory malformed")
        if not (
            case.get("exit_code") == 0
            and case.get("timed_out") is False
            and case.get("observed_required_processes") == []
            and case.get("remaining_process_group_pids") == []
            and case.get("task_owned_process_audit_passed") is True
            and case.get("parse_passed") is True
            and case.get("subprocess_environment")
            == {"PYTHONDONTWRITEBYTECODE": "1"}
        ):
            failures.append(f"parse case[{case_id}] outcome mismatch")
        case_root = parse_root / case_id.lower()
        for stream_name in ("stdout", "stderr"):
            stream_path = case_root / f"{stream_name}.log"
            try:
                relative = str(stream_path.relative_to(REPOSITORY))
            except ValueError:
                relative = ""
            if not (
                case.get(f"{stream_name}_path") == relative
                and stream_path.is_file() and not stream_path.is_symlink()
                and case.get(f"{stream_name}_sha256") == _sha256(stream_path)
            ):
                failures.append(
                    f"parse case[{case_id}] {stream_name} binding mismatch"
                )
    return failures


def write_adoption_manifest(current_commit: str) -> tuple[Path, dict]:
    output = ADOPTION_MANIFEST_PATH
    if output.exists() or output.is_symlink():
        raise LiveRunnerError("adoption_manifest_already_exists")
    changed = subprocess.check_output(
        ["git", "diff", "--name-only", f"{PRODUCT_COMMIT}..{current_commit}"],
        cwd=REPOSITORY, text=True,
    ).splitlines()
    payload = build_adoption_payload(current_commit, changed)
    _write_json_exclusive(output, payload)
    return output, payload


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
    active_prefixes = os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    expected_active_prefixes = [
        str(OVERLAY_INSTALL_ROOT), *expected_prefixes,
    ]
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
        or active_prefixes != expected_active_prefixes
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
    aliases = manifest.get("installed_aliases")
    if not isinstance(aliases, dict) or set(aliases) != set(INSTALLED_ALIASES):
        raise LiveRunnerError("install_manifest_alias_inventory_drift")
    for relative, source_relative in INSTALLED_ALIASES.items():
        row = aliases.get(relative)
        installed = INSTALL_ROOT / relative
        if not (
            isinstance(row, dict)
            and row.get("source_path") == source_relative
            and installed.is_file() and not installed.is_symlink()
            and row.get("source_sha256") == row.get("installed_sha256")
            and _is_sha256(row.get("installed_sha256", ""))
            and _sha256(installed) == row.get("installed_sha256")
        ):
            raise LiveRunnerError(f"install_manifest_alias_drift:{relative}")
    for relative, expected_hash in file_hashes.items():
        file_path = Path(relative) if Path(relative).is_absolute() else INSTALL_ROOT / relative
        if not file_path.is_file() or file_path.is_symlink() \
                or _sha256(file_path) != expected_hash:
            raise LiveRunnerError(f"install_manifest_file_drift:{relative}")
    clean_environment = dict(os.environ)
    clean_environment["LD_LIBRARY_PATH"] = ":".join((
        str(INSTALL_ROOT / "lib"), "/root/ros2_ws/install/glim_ros/lib",
        "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
        "/opt/ros/jazzy/lib/x86_64-linux-gnu",
    ))
    if not linkage_inventory_matches(
        linkage, runtime_libraries, INSTALL_ROOT, clean_environment
    ):
        raise LiveRunnerError("install_manifest_linkage_drift")
    return manifest


def live_config(
    contract: dict, case_id: str, run_id: str, run_dir: Path
) -> dict[str, object]:
    run_dir = Path(run_dir).resolve()
    values = QUALIFICATION.resolve_launch_values(contract, case_id, {})
    expected_scenario = str(
        OVERLAY_INSTALL_ROOT / "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"
    )
    if values.get("gnss_scenario_file") != expected_scenario:
        raise LiveRunnerError("live_gnss_scenario_path_mismatch")
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
        "PYTHONDONTWRITEBYTECODE": "1",
        "HOME": str(environment_root / "home"),
        "ROS_HOME": str(environment_root / "ros_home"),
        "ROS_LOG_DIR": str(environment_root / "ros_logs"),
        "TMPDIR": str(environment_root / "tmp"),
        "XDG_RUNTIME_DIR": str(environment_root / "xdg_runtime"),
        "AMENT_PREFIX_PATH": ":".join((
            str(OVERLAY_INSTALL_ROOT), str(INSTALL_ROOT),
            "/root/ros2_ws/install", "/opt/ros/jazzy",
        )),
        "CMAKE_PREFIX_PATH": ":".join((
            str(OVERLAY_INSTALL_ROOT), str(INSTALL_ROOT),
            "/root/ros2_ws/install/glim_ros",
            "/root/ros2_ws/install/glim", "/opt/ros/jazzy",
        )),
        "LD_LIBRARY_PATH": ":".join((
            str(OVERLAY_INSTALL_ROOT / "lib"), str(INSTALL_ROOT / "lib"),
            "/root/ros2_ws/install/glim_ros/lib",
            "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
            "/opt/ros/jazzy/lib/x86_64-linux-gnu",
        )),
        "COLCON_PREFIX_PATH": ":".join((
            str(OVERLAY_INSTALL_ROOT), str(INSTALL_ROOT),
            "/root/ros2_ws/install",
        )),
        "PYTHONPATH": ":".join((
            str(OVERLAY_INSTALL_ROOT / "lib/python3.12/site-packages"),
            str(INSTALL_ROOT / "lib/python3.12/site-packages"),
            "/opt/ros/jazzy/lib/python3.12/site-packages",
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
    fixture_rows = rejected if case_id == "FINAL_REJECT" else emergency[:1]
    if not _registered_fixture_evidence_present(case_id, contract, fixture_rows):
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
        if len(emergency) != 1 or any(
            "future_unknown" not in str(row.get("reason", ""))
            for row in emergency
        ):
            raise LiveRunnerError("future_unknown_emergency_cardinality_mismatch")
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
        and row.get("gnss_epoch_seen") is True
        and row.get("gnss_epoch_valid") is True
        and row.get("gnss_epoch_fresh") is True
        and int(row.get("predictor_gnss_used_count", 0) or 0) > 0
        and int(row.get("predictor_lidar_used_count", 0) or 0) > 0
        and int(row.get("predictor_horizon_fusion_count", 0) or 0) > 0
    ]
    if len(stable_rows) < 2:
        raise LiveRunnerError("full_sensor_p0_rows_missing")
    p0_samples = [
        {
            "sequence": index,
            "ready": True,
            "stable": True,
            "generation_id": int(row["generation_id"]),
            "worker_count": 4,
            "refresh_s": float(row.get("refresh_duration_ms", 0.0)) / 1000.0,
            "gnss_epoch_seen": True,
            "gnss_epoch_valid": True,
            "gnss_epoch_fresh": True,
            "predictor_gnss_used_count": int(row["predictor_gnss_used_count"]),
            "predictor_lidar_used_count": int(row["predictor_lidar_used_count"]),
            "predictor_horizon_fusion_count": int(
                row["predictor_horizon_fusion_count"]
            ),
        }
        for index, row in enumerate(stable_rows, 1)
    ]
    integrity_path = _one_file(
        run_dir, "integrity_report.jsonl", "integrity_capture"
    )
    integrity_rows = []
    try:
        for line in integrity_path.read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("integrity row is not an object")
                integrity_rows.append(row)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise LiveRunnerError("full_sensor_integrity_capture_malformed") from exc
    valid_integrity = [
        row for row in integrity_rows
        if row.get("valid") is True
        and type(row.get("n_sv_used")) is int
        and row["n_sv_used"] > 0
    ]
    if len(valid_integrity) < 2:
        raise LiveRunnerError("full_sensor_integrity_rows_missing")
    integrity_samples = [
        {
            "sequence": index,
            "valid": True,
            "n_sv_used": int(row["n_sv_used"]),
        }
        for index, row in enumerate(valid_integrity, 1)
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
        "integrity_samples": integrity_samples,
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
            start_new_session=True, env=os.environ.copy(),
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
    command, omitted = render_live_launch_command(
        config, canonical_empty_defaults(contract)
    )
    _write_json(run_dir / "launch_command.json", {
        "argv": command, "omitted_empty_defaults": omitted,
        "subprocess_environment": {"PYTHONDONTWRITEBYTECODE": "1"},
    })
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
        "subprocess_environment": {"PYTHONDONTWRITEBYTECODE": "1"},
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


def write_replacement_live_bundle(
    contract: dict, normalized: list[dict], current_commit: str,
    adoption_path: Path, output_path: Path,
) -> dict:
    bundle = QUALIFICATION.write_live_bundle(
        contract, CONTRACT_PATH, normalized, current_commit,
        INSTALL_MANIFEST_PATH, output_path,
    )
    bundle["manifest"].update({
        "product_install_git_commit": PRODUCT_COMMIT,
        "runner_analyzer_git_commit": current_commit,
        "adoption_manifest_path": str(adoption_path.relative_to(REPOSITORY)),
        "adoption_manifest_sha256": _sha256(adoption_path),
        "overlay_manifest_path": str(
            OVERLAY_MANIFEST_PATH.relative_to(REPOSITORY)
        ),
        "overlay_manifest_sha256": _sha256(OVERLAY_MANIFEST_PATH),
    })
    _write_json(output_path, bundle)
    return bundle


def reconcile_replacement_analysis(
    base_result: dict, dual_provenance_failures: list[str]
) -> dict:
    result = dict(base_result)
    technical = list(result.get("technical_failures", []))
    split_marker = "install manifest commit mismatch"
    split_count = technical.count(split_marker)
    if split_count == 0:
        technical.append("expected product/runner commit split marker missing")
    elif split_count > 1:
        technical.append("expected product/runner commit split marker cardinality mismatch")
    elif not dual_provenance_failures:
        technical.remove(split_marker)
    technical.extend(dual_provenance_failures)
    behavioral = list(result.get("behavioral_failures", []))
    if technical:
        status = "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER"
    elif behavioral:
        status = "P5_PROSPECTIVE_QUALIFICATION_FAIL"
    else:
        status = "P5_PROSPECTIVE_QUALIFICATION_PASS"
    result.update({
        "status": status,
        "qualification_claim": status == "P5_PROSPECTIVE_QUALIFICATION_PASS",
        "technical_failures": technical,
        "behavioral_failures": behavioral,
    })
    return result


def require_replacement_evidence_ready(bundle: object, contract: dict) -> None:
    """Fail before analyzer invocation unless runner and parser proof are exact."""
    failures = []
    manifest = bundle.get("manifest") if isinstance(bundle, dict) else None
    if not isinstance(manifest, dict):
        manifest = {}
        failures.append("replacement manifest missing")
    expected_runner_path = TASK_ROOT / "live/runner_state.json"
    runner_relative = manifest.get("runner_state_path")
    try:
        canonical_runner_relative = str(expected_runner_path.relative_to(REPOSITORY))
    except ValueError:
        canonical_runner_relative = ""
    if not (
        runner_relative == canonical_runner_relative
        and expected_runner_path.is_file()
        and not expected_runner_path.is_symlink()
        and manifest.get("runner_state_sha256") == _sha256(expected_runner_path)
    ):
        failures.append("runner state binding mismatch")
        runner_state = {}
    else:
        try:
            runner_state = json.loads(expected_runner_path.read_text())
        except (OSError, json.JSONDecodeError):
            runner_state = {}
            failures.append("runner state malformed")
    expected_ids = [run_id for _, run_id in LIVE_IDENTITIES]
    if not (
        runner_state.get("state") == "COMPLETE"
        and runner_state.get("registered") == expected_ids
        and runner_state.get("attempted") == expected_ids
        and runner_state.get("completed") == expected_ids
        and runner_state.get("retries") == 0
        and runner_state.get("gpu_preflight_invocations") == 1
        and runner_state.get("launch_invocations") == 3
    ):
        failures.append("replacement runner is not exact COMPLETE one-shot evidence")
    runner_commit = runner_state.get("runner_analyzer_git_commit", "")
    expected_overlay_relative = str(
        OVERLAY_MANIFEST_PATH.relative_to(REPOSITORY)
    )
    if not (
        manifest.get("overlay_manifest_path") == expected_overlay_relative
        and runner_state.get("overlay_manifest_path")
        == expected_overlay_relative
        and OVERLAY_MANIFEST_PATH.is_file()
        and not OVERLAY_MANIFEST_PATH.is_symlink()
        and manifest.get("overlay_manifest_sha256")
        == runner_state.get("overlay_manifest_sha256")
        == _sha256(OVERLAY_MANIFEST_PATH)
    ):
        failures.append("overlay manifest binding mismatch")
    try:
        validate_overlay_manifest(runner_commit)
    except LiveRunnerError as exc:
        failures.append(f"overlay manifest validation failed:{exc}")
    expected_proof_path = TASK_ROOT / "parse_only/parser_proof.json"
    proof_relative = runner_state.get("parse_proof_path")
    try:
        canonical_proof_relative = str(expected_proof_path.relative_to(REPOSITORY))
    except ValueError:
        canonical_proof_relative = ""
    if not (
        proof_relative == canonical_proof_relative
        and expected_proof_path.is_file()
        and not expected_proof_path.is_symlink()
        and runner_state.get("parse_proof_sha256") == _sha256(expected_proof_path)
    ):
        failures.append("parse proof binding mismatch")
    else:
        try:
            proof = json.loads(expected_proof_path.read_text())
        except (OSError, json.JSONDecodeError):
            failures.append("parse proof malformed")
        else:
            failures.extend(parse_proof_failures(proof, contract))
    if failures:
        raise LiveRunnerError("replacement_evidence_not_ready:" + failures[0])


def analyze_replacement_live(input_path: Path, output_path: Path) -> int:
    input_path = Path(input_path).resolve()
    output_path = Path(output_path).resolve()
    expected_input = (TASK_ROOT / "live/icra_p0_p5_evidence_v1.json").resolve()
    expected_output = (
        TASK_ROOT / "compact/icra_p0_p5_analysis_v1.json"
    ).resolve()
    if input_path != expected_input or output_path != expected_output:
        raise LiveRunnerError("replacement_analyzer_path_mismatch")
    try:
        bundle = json.loads(input_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise LiveRunnerError("replacement_live_bundle_malformed") from exc
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    require_replacement_evidence_ready(bundle, contract)
    analyzer_environment = validate_live_environment()
    QUALIFICATION._claim_live_analyzer_once(
        output_path, input_path, CONTRACT_PATH
    )
    manifest = bundle.get("manifest", {})
    adoption_relative = manifest.get("adoption_manifest_path")
    adoption_path = (
        REPOSITORY / adoption_relative
        if isinstance(adoption_relative, str) else REPOSITORY
    )
    dual_failures = []
    if not (
        isinstance(adoption_relative, str)
        and not Path(adoption_relative).is_absolute()
        and adoption_path.resolve()
        == ADOPTION_MANIFEST_PATH.resolve()
        and adoption_path.is_file() and not adoption_path.is_symlink()
        and _sha256(adoption_path) == manifest.get("adoption_manifest_sha256")
    ):
        dual_failures.append("dual provenance adoption manifest binding mismatch")
        adoption = {}
    else:
        try:
            adoption = json.loads(adoption_path.read_text())
        except (OSError, json.JSONDecodeError):
            adoption = {}
    current_commit = manifest.get("runner_analyzer_git_commit", "")
    if manifest.get("product_install_git_commit") != PRODUCT_COMMIT \
            or manifest.get("git_commit") != current_commit:
        dual_failures.append("dual provenance commit binding mismatch")
    if isinstance(adoption, dict) and adoption:
        changed = adoption.get("post_product_changed_files", [])
        try:
            expected_adoption = build_adoption_payload(current_commit, changed)
        except LiveRunnerError as exc:
            dual_failures.append(f"dual provenance validation failed:{exc}")
        else:
            if adoption != expected_adoption:
                dual_failures.append("dual provenance adoption payload mismatch")
    result = QUALIFICATION.analyze_live_bundle(
        contract, bundle,
        CONTRACT_PATH, REPOSITORY,
    )
    result = reconcile_replacement_analysis(result, dual_failures)
    status = result["status"]
    result.update({
        "analyzer_environment": analyzer_environment,
        "dual_provenance": {
            "validated": not dual_failures,
            "product_install_git_commit": PRODUCT_COMMIT,
            "runner_analyzer_git_commit": current_commit,
            "adoption_manifest_sha256": manifest.get(
                "adoption_manifest_sha256"
            ),
        },
    })
    _write_json(output_path, result)
    return 0 if status == "P5_PROSPECTIVE_QUALIFICATION_PASS" else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live-root", type=Path, default=TASK_ROOT / "live")
    parser.add_argument("--install-manifest", type=Path, default=INSTALL_MANIFEST_PATH)
    parser.add_argument("--freeze-install-only", action="store_true")
    parser.add_argument("--prepare-overlay", action="store_true")
    parser.add_argument("--repair-overlay-cache", action="store_true")
    parser.add_argument("--analyze-replacement-live", action="store_true")
    parser.add_argument(
        "--analysis-input", type=Path,
        default=TASK_ROOT / "live/icra_p0_p5_evidence_v1.json",
    )
    parser.add_argument(
        "--analysis-output", type=Path,
        default=TASK_ROOT / "compact/icra_p0_p5_analysis_v1.json",
    )
    args = parser.parse_args()
    if sum(bool(value) for value in (
        args.freeze_install_only, args.prepare_overlay,
        args.repair_overlay_cache, args.analyze_replacement_live,
    )) > 1:
        raise LiveRunnerError("conflicting_runner_modes")
    if args.repair_overlay_cache:
        repair_overlay_cache()
        return 0
    if args.prepare_overlay:
        prepare_overlay()
        return 0
    if args.freeze_install_only:
        raise LiveRunnerError("icra070_product_install_is_immutable")
    if args.analyze_replacement_live:
        return analyze_replacement_live(
            args.analysis_input, args.analysis_output
        )
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
    if _sha256(install_manifest_path) != PRODUCT_MANIFEST_SHA256:
        raise LiveRunnerError("product_manifest_sha256_mismatch")
    validate_frozen_install_manifest(
        install_manifest_path, PRODUCT_COMMIT
    )
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=REPOSITORY, text=True,
    ).strip()
    if dirty:
        raise LiveRunnerError("tracked_worktree_not_clean")
    contract = QUALIFICATION.load_contract(CONTRACT_PATH)
    validate_overlay_manifest(git_commit)
    if tuple(contract["required_processes"]) != tuple(REQUIRED_PROCESSES):
        raise LiveRunnerError("required_process_marker_contract_mismatch")
    if tuple(contract["cases"]) != tuple(case_id for case_id, _ in LIVE_IDENTITIES):
        raise LiveRunnerError("live_identity_contract_mismatch")
    adoption_path, _ = write_adoption_manifest(git_commit)
    parse_summary = run_parse_only_proofs(contract, TASK_ROOT / "parse_only")
    if parse_summary.get("parse_ready") is not True:
        print("ROS_LAUNCH_PARSER_NOT_READY")
        return 2
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
        "gpu_preflight_environment": {
            "PYTHONDONTWRITEBYTECODE": "1"
        },
        "launch_invocations": len(state["attempted"]),
        "registered": [run_id for _, run_id in LIVE_IDENTITIES],
        "free_bytes_before_gpu": free_bytes,
        "live_environment": live_environment,
        "install_manifest_sha256": _sha256(install_manifest_path),
        "product_install_git_commit": PRODUCT_COMMIT,
        "runner_analyzer_git_commit": git_commit,
        "adoption_manifest_path": str(adoption_path.relative_to(REPOSITORY)),
        "adoption_manifest_sha256": _sha256(adoption_path),
        "overlay_manifest_path": str(
            OVERLAY_MANIFEST_PATH.relative_to(REPOSITORY)
        ),
        "overlay_manifest_sha256": _sha256(OVERLAY_MANIFEST_PATH),
        "parse_proof_path": str(
            (TASK_ROOT / "parse_only/parser_proof.json").relative_to(REPOSITORY)
        ),
        "parse_proof_sha256": _sha256(
            TASK_ROOT / "parse_only/parser_proof.json"
        ),
    })
    _write_json(live_root / "runner_state.json", state)
    if state["state"] != "COMPLETE":
        return 4
    write_replacement_live_bundle(
        contract, normalized, git_commit, adoption_path,
        live_root / "icra_p0_p5_evidence_v1.json",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
