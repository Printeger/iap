#!/usr/bin/env python3
"""Fail-closed runner for the registered P4-G0C calibration matrix.

ICRA-042 registers this tool and tests it with synthetic boundaries only. Live
execution remains unauthorized until a later task explicitly invokes it.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import stat
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from p4_g0c_protocol import (  # noqa: E402
    DECISION_CSV_COLUMNS,
    LAUNCH_ENVIRONMENT_KEYS,
    LAUNCH_ENVIRONMENT_DIRECTORY_MODES,
    LAUNCH_ENVIRONMENT_SCHEMA,
    MUTABLE_OUTPUT_KEYS,
    RUN_ARTIFACT_INVENTORY_FILENAME,
    DecisionSchemaError,
    ProtocolBundle,
    bound_test_planner_manifest_path,
    canonical_bytes,
    decision_identity,
    effective_config_sha256,
    expected_launch_environment_binding,
    exact_json_equal,
    expand_run_plan,
    load_canonical_json,
    load_protocol_bundle,
    make_run_artifact_inventory,
    parse_decision_row,
    sha256_file,
    validate_test_planner_effective_contract,
    validate_launch_environment_binding,
    validate_decision_header,
)
from run_gate0_qualification import (  # noqa: E402
    RequiredProcessMonitor,
    run_gpu_preflight,
)


RUNNER_SCHEMA_V1 = "p4_g0c_runner_state_v3"
RUNNER_SCHEMA_V2 = "p4_g0c_runner_state_v4"
RUNNER_SCHEMA_V3 = "p4_g0c_runner_state_v5"
RUNNER_SCHEMA_V4 = "p4_g0c_runner_state_v6"
RUNNER_SCHEMA_V5 = "p4_g0c_runner_state_v7"
RUNNER_SCHEMA_V6 = "p4_g0c_runner_state_v8"
DEPENDENCY_SCHEMA_V2 = "p4_g0c_runtime_dependencies_v2"
DEPENDENCY_SCHEMA_V3 = "p4_g0c_runtime_dependencies_v3"
DEPENDENCY_SCHEMA_V4 = "p4_g0c_runtime_dependencies_v4"
DEPENDENCY_SCHEMA_V5 = "p4_g0c_runtime_dependencies_v5"
DEPENDENCY_SCHEMA_V6 = "p4_g0c_runtime_dependencies_v6"
LEGACY_PROTOCOL_SCHEMA = "p4_g0c_protocol_v1"
REPLACEMENT_PROTOCOL_SCHEMA = "p4_g0c_protocol_v2"
HARDENED_PROTOCOL_SCHEMA = "p4_g0c_protocol_v3"
PROFILED_PROTOCOL_SCHEMA = "p4_g0c_protocol_v4"
CLOSED_FIXTURE_PROTOCOL_SCHEMA = "p4_g0c_protocol_v5"
TEMPORAL_SUPPORT_PROTOCOL_SCHEMA = "p4_g0c_protocol_v6"
EXPECTED_ACTIVE_PACKAGES = {
    "iap", "ego_planner", "local_sensing", "odom_visualization",
    "poscmd_2_odom", "gnss_sim", "so3_quadrotor_simulator",
    "so3_control", "rclcpp_components",
}
EXPECTED_BUILD_CLOSURE = {
    "bspline_opt", "path_searching", "plan_env", "traj_utils",
    "quadrotor_msgs", "gnss_comm", "cmake_utils", "pose_utils",
    "uav_utils",
}
EXPECTED_RUNTIME_LIBRARIES = {
    "lib/libglobal_mapping.so",
    "lib/libgnss_extension.so",
    "lib/libintegrity_extension.so",
    "lib/libodometry_estimation_gpu.so",
    "lib/libsim_extension.so",
    "lib/libsub_mapping.so",
}
REQUIRED_PROCESSES = {
    "iap_rosnode": ["iap_rosnode", "test_planner_iap_rosnode"],
    "ego_planner_node": ["ego_planner_node", "drone_0_ego_planner_node"],
}
class RunnerError(RuntimeError):
    """The matrix cannot proceed without violating the registered protocol."""


def load_bundle(
    protocol_path: Path,
    registry_path: Path,
    fixture_path: Path,
    expected_protocol_schema: str = REPLACEMENT_PROTOCOL_SCHEMA,
) -> ProtocolBundle:
    bundle = load_protocol_bundle(
        protocol_path,
        registry_path,
        fixture_path,
        expected_protocol_schema=expected_protocol_schema,
    )
    bundle.protocol_path = str(Path(protocol_path).resolve())
    bundle.registry_path = str(Path(registry_path).resolve())
    bundle.fixture_path = str(Path(fixture_path).resolve())
    if bundle.protocol.get("schema_version") in {
        REPLACEMENT_PROTOCOL_SCHEMA, HARDENED_PROTOCOL_SCHEMA,
        PROFILED_PROTOCOL_SCHEMA, CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        repository_root = Path(protocol_path).resolve().parents[2]
        bundle.dependency_manifest_path = str(
            repository_root
            / bundle.protocol["runtime_dependency_manifest"]["path"]
        )
    return bundle


def _dependency_failure(
    reason: str,
    schema_version: str = "p4_g0c_dependency_preflight_result_v2",
) -> dict[str, Any]:
    return {
        "schema_version": schema_version,
        "dependency_ready": False,
        "failure_reason": reason,
        "package_count": 0,
        "executable_count": 0,
        "component_count": 0,
        "config_count": 0,
        "runtime_library_count": 0,
    }


def validate_p0_profile_binding(bundle: ProtocolBundle) -> dict[str, Any]:
    """Fail closed on the exact retained Gate-0B profile before GPU or ROS."""
    schema = "p4_g0c_p0_profile_preflight_v1"
    if bundle.protocol.get("schema_version") not in {
        PROFILED_PROTOCOL_SCHEMA, CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        return {"schema_version": schema, "profile_ready": True,
                "failure_reason": "", "not_applicable": True}
    effective = bundle.protocol.get("effective_values", {})
    sigma = effective.get("p0.predictor.sigma_grow_m_sqrt_s")
    profile = effective.get("p0.predictor.sigma_growth_profile")
    worker_count = effective.get("p0.predictor.worker_count")
    if type(sigma) is not float or not math.isfinite(sigma) or sigma != 0.01:
        return {"schema_version": schema, "profile_ready": False,
                "failure_reason": "P0_SIGMA_BINDING_MISMATCH"}
    if profile != "legacy_iap_rq320_baseline_v1":
        return {"schema_version": schema, "profile_ready": False,
                "failure_reason": "P0_PROFILE_BINDING_MISMATCH"}
    if bundle.protocol.get("schema_version") in {
            CLOSED_FIXTURE_PROTOCOL_SCHEMA, TEMPORAL_SUPPORT_PROTOCOL_SCHEMA} \
            and (type(worker_count) is not int or worker_count != 4):
        return {"schema_version": schema, "profile_ready": False,
                "failure_reason": "P0_WORKER_BINDING_MISMATCH"}
    binding = bundle.protocol.get("p0_profile_binding", {})
    expected = {
        "config_preflight": "6d9ddcc0dd079a3a857a24cf61381441e4260498108077d3be795a8c6ea9b60b",
        "analyzer": "5855368ddc0f89d69c8d13d3f9083b40371678177f2f6eaf3ce7fb68ee0dbaf3",
    }
    repository_root = Path(bundle.protocol_path).resolve().parents[2]
    for key, expected_sha in expected.items():
        artifact = binding.get(key, {})
        path = repository_root / str(artifact.get("path", ""))
        if artifact.get("sha256") != expected_sha or not path.is_file() \
                or sha256_file(path) != expected_sha:
            return {"schema_version": schema, "profile_ready": False,
                    "failure_reason": f"P0_EVIDENCE_BINDING_MISMATCH:{key}"}
    return {"schema_version": schema, "profile_ready": True,
            "failure_reason": "", "sigma_grow_m_sqrt_s": sigma,
            "sigma_growth_profile": profile,
            "predictor_worker_count": worker_count}


def load_runtime_dependency_manifest(
    path: Path, expected_sha256: str, *,
    expected_schema: str = DEPENDENCY_SCHEMA_V2,
    expected_experiment: str = "p4_g0c_metrics_calibration_v2",
) -> dict[str, Any]:
    path = Path(path)
    if sha256_file(path) != expected_sha256:
        raise RunnerError("DEPENDENCY_MANIFEST_HASH_MISMATCH")
    manifest = load_canonical_json(path)
    required_keys = {
        "schema_version", "experiment", "launch_contract", "packages",
        "components", "config_hashes", "inactive_packages", "prefix_policy",
        "runtime_libraries",
    }
    if set(manifest) != required_keys or manifest.get("schema_version") != expected_schema:
        raise RunnerError("DEPENDENCY_MANIFEST_SCHEMA_MISMATCH")
    if manifest.get("experiment") != expected_experiment:
        raise RunnerError("DEPENDENCY_MANIFEST_EXPERIMENT_MISMATCH")
    packages = manifest.get("packages")
    if not isinstance(packages, list) or not packages:
        raise RunnerError("DEPENDENCY_MANIFEST_PACKAGES_MALFORMED")
    package_names = []
    for package in packages:
        if not isinstance(package, dict) or set(package) != {
            "name", "role", "executables", "config_files"
        }:
            raise RunnerError("DEPENDENCY_MANIFEST_PACKAGE_MALFORMED")
        if package.get("role") not in {"active_runtime", "build_closure"}:
            raise RunnerError("DEPENDENCY_MANIFEST_PACKAGE_ROLE_MALFORMED")
        if not isinstance(package.get("name"), str) or not package["name"]:
            raise RunnerError("DEPENDENCY_MANIFEST_PACKAGE_NAME_MALFORMED")
        for field in ("executables", "config_files"):
            values = package.get(field)
            if (
                not isinstance(values, list)
                or any(not isinstance(item, str) or not item for item in values)
                or len(values) != len(set(values))
            ):
                raise RunnerError(f"DEPENDENCY_MANIFEST_{field.upper()}_MALFORMED")
        package_names.append(package["name"])
    if package_names != sorted(package_names) or len(package_names) != len(set(package_names)):
        raise RunnerError("DEPENDENCY_MANIFEST_PACKAGE_ORDER_MALFORMED")
    roles = {
        role: {package["name"] for package in packages if package["role"] == role}
        for role in ("active_runtime", "build_closure")
    }
    if not EXPECTED_ACTIVE_PACKAGES.issubset(roles["active_runtime"]):
        raise RunnerError("DEPENDENCY_MANIFEST_ACTIVE_CLOSURE_INCOMPLETE")
    if not EXPECTED_BUILD_CLOSURE.issubset(roles["build_closure"]):
        raise RunnerError("DEPENDENCY_MANIFEST_BUILD_CLOSURE_INCOMPLETE")
    config_keys = {
        f"{package['name']}:{relative}"
        for package in packages for relative in package["config_files"]
    }
    config_hashes = manifest.get("config_hashes")
    if (
        not isinstance(config_hashes, dict)
        or set(config_hashes) != config_keys
        or any(
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            for digest in config_hashes.values()
        )
    ):
        raise RunnerError("DEPENDENCY_MANIFEST_CONFIG_HASHES_MALFORMED")
    components = manifest.get("components")
    if (
        not isinstance(components, list)
        or len(components) != 1
        or set(components[0]) != {"package", "plugin", "library"}
        or components[0].get("package") != "so3_control"
        or components[0].get("plugin") != "SO3ControlComponent"
    ):
        raise RunnerError("DEPENDENCY_MANIFEST_COMPONENT_MALFORMED")
    if manifest.get("inactive_packages") != ["rosbag2_transport", "rviz2"]:
        raise RunnerError("DEPENDENCY_MANIFEST_INACTIVE_SET_MALFORMED")
    runtime_libraries = manifest.get("runtime_libraries")
    if (
        not isinstance(runtime_libraries, list)
        or any(
            not isinstance(item, dict)
            or set(item) != {"package", "relative_path"}
            or item.get("package") != "iap"
            or not isinstance(item.get("relative_path"), str)
            for item in runtime_libraries
        )
        or [item["relative_path"] for item in runtime_libraries]
        != sorted(EXPECTED_RUNTIME_LIBRARIES)
    ):
        raise RunnerError("DEPENDENCY_MANIFEST_RUNTIME_LIBRARIES_MALFORMED")
    policy = manifest.get("prefix_policy")
    if policy != {
        "allowed_prefixes_environment": "P4_G0C_ALLOWED_PREFIXES",
        "current_prefixes_environment": "AMENT_PREFIX_PATH",
        "forbidden_prefix_roots": [
            "/home/dev/ws_iap/build",
            "/home/dev/ws_iap/install",
            "/home/dev/ws_iap/src/iap/build",
            "/home/dev/ws_iap/src/iap/install",
        ],
        "reject_duplicate_package_identity": True,
        "reject_symlinks": True,
        "reject_undeclared_prefix": True,
    }:
        raise RunnerError("DEPENDENCY_MANIFEST_PREFIX_POLICY_MALFORMED")
    launch_contract = manifest.get("launch_contract")
    if (
        not isinstance(launch_contract, dict)
        or set(launch_contract) != {
            "package", "relative_path", "sha256", "source_path"
        }
        or launch_contract.get("package") != "iap"
        or launch_contract.get("relative_path") != "launch/test_planner.launch.py"
        or launch_contract.get("source_path") != "launch/test_planner.launch.py"
    ):
        raise RunnerError("DEPENDENCY_MANIFEST_LAUNCH_CONTRACT_MALFORMED")
    return manifest


def _environment_prefixes(environment: dict[str, str], name: str) -> list[Path]:
    raw = environment.get(name, "")
    if not raw:
        return []
    return [Path(item) for item in raw.split(os.pathsep) if item]


def _canonical_prefixes(prefixes: list[Path]) -> tuple[list[Path], str]:
    canonical = []
    for requested in prefixes:
        if not requested.is_absolute():
            return [], "DEPENDENCY_PREFIX_NOT_ABSOLUTE"
        resolved = requested.resolve()
        if requested != resolved or requested.is_symlink():
            return [], "DEPENDENCY_PREFIX_SYMLINK_OR_ALIAS"
        if not requested.is_dir():
            return [], "DEPENDENCY_PREFIX_MISSING"
        canonical.append(resolved)
    if len(canonical) != len(set(canonical)):
        return [], "DEPENDENCY_PREFIX_DUPLICATE"
    return canonical, ""


def _ordinary_file_within(path: Path, prefix: Path, *, executable: bool = False) -> bool:
    try:
        resolved = path.resolve(strict=True)
    except OSError:
        return False
    if path.is_symlink() or not path.is_file() or prefix not in resolved.parents:
        return False
    return not executable or os.access(path, os.X_OK)


def _elf_type(path: Path) -> int | None:
    try:
        size = path.stat().st_size
        with path.open("rb") as stream:
            header = stream.read(64)
    except OSError:
        return None
    if len(header) < 24 or header[:4] != b"\x7fELF":
        return None
    if header[4] not in {1, 2} or header[5] not in {1, 2}:
        return None
    byte_order = "<" if header[5] == 1 else ">"
    if header[6] != 1 or struct.unpack(f"{byte_order}I", header[20:24])[0] != 1:
        return None
    if header[4] == 2:
        if len(header) < 64:
            return None
        program_offset = struct.unpack(f"{byte_order}Q", header[32:40])[0]
        header_size, program_entry_size, program_count = struct.unpack(
            f"{byte_order}HHH", header[52:58]
        )
        minimum_header_size = 64
        minimum_program_entry_size = 56
    else:
        if len(header) < 52:
            return None
        program_offset = struct.unpack(f"{byte_order}I", header[28:32])[0]
        header_size, program_entry_size, program_count = struct.unpack(
            f"{byte_order}HHH", header[40:46]
        )
        minimum_header_size = 52
        minimum_program_entry_size = 32
    if header_size < minimum_header_size or size < header_size:
        return None
    if program_count and (
        program_entry_size < minimum_program_entry_size
        or program_offset + program_entry_size * program_count > size
    ):
        return None
    machine = struct.unpack(f"{byte_order}H", header[18:20])[0]
    host_machines = {
        "aarch64": 183,
        "arm64": 183,
        "armv7l": 40,
        "i386": 3,
        "i686": 3,
        "x86_64": 62,
    }
    if machine != host_machines.get(os.uname().machine):
        return None
    return struct.unpack(f"{byte_order}H", header[16:18])[0]


def _dynamic_linkage_ready(path: Path, environment: dict[str, str]) -> bool:
    linkage_environment = dict(environment)
    linkage_environment["LC_ALL"] = "C"
    try:
        completed = subprocess.run(
            ["ldd", str(path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=10.0,
            check=False,
            env=linkage_environment,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    output = completed.stdout.lower()
    return (
        completed.returncode == 0
        and "not found" not in output
        and "not a dynamic executable" not in output
    )


def _is_loadable_elf(
    path: Path, allowed_types: set[int], environment: dict[str, str]
) -> bool:
    return (
        _elf_type(path) in allowed_types
        and _dynamic_linkage_ready(path, environment)
    )


def _is_loadable_executable(path: Path, environment: dict[str, str]) -> bool:
    try:
        with path.open("rb") as stream:
            prefix = stream.read(4)
            if prefix == b"\x7fELF":
                return _is_loadable_elf(path, {2, 3}, environment)
            first_line = (prefix + stream.readline(124)).decode("utf-8").strip()
    except (OSError, UnicodeDecodeError):
        return False
    if not first_line.startswith("#!"):
        return False
    interpreter_tokens = first_line[2:].strip().split()
    if not interpreter_tokens:
        return False
    interpreter = Path(interpreter_tokens[0])
    return (
        interpreter.is_absolute()
        and interpreter.is_file()
        and os.access(interpreter, os.X_OK)
    )


def validate_runtime_dependencies(
    bundle: ProtocolBundle,
    manifest_path: Path | None = None,
    environment: dict[str, str] | None = None,
) -> dict[str, Any]:
    protocol_schema = bundle.protocol.get("schema_version")
    if protocol_schema not in {
        REPLACEMENT_PROTOCOL_SCHEMA, HARDENED_PROTOCOL_SCHEMA,
        PROFILED_PROTOCOL_SCHEMA, CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        return _dependency_failure("DEPENDENCY_PROTOCOL_V2_REQUIRED")
    version = 6 if protocol_schema == TEMPORAL_SUPPORT_PROTOCOL_SCHEMA else 5 \
        if protocol_schema == CLOSED_FIXTURE_PROTOCOL_SCHEMA else 4 \
        if protocol_schema == PROFILED_PROTOCOL_SCHEMA else 3 \
        if protocol_schema == HARDENED_PROTOCOL_SCHEMA else 2
    hardened = version >= 3
    result_schema = (
        f"p4_g0c_dependency_preflight_result_v{version}"
    )

    def dependency_failure(reason: str) -> dict[str, Any]:
        return _dependency_failure(reason, result_schema)

    binding = bundle.protocol["runtime_dependency_manifest"]
    try:
        resolved_manifest_path = Path(
            manifest_path or bundle.dependency_manifest_path
        ).resolve()
        manifest = load_runtime_dependency_manifest(
            resolved_manifest_path,
            binding["sha256"],
            expected_schema=(
                DEPENDENCY_SCHEMA_V6 if version == 6 else
                DEPENDENCY_SCHEMA_V5 if version == 5 else
                DEPENDENCY_SCHEMA_V4 if version == 4 else
                DEPENDENCY_SCHEMA_V3 if version == 3 else DEPENDENCY_SCHEMA_V2
            ),
            expected_experiment=(
                f"p4_g0c_metrics_calibration_v{version}"
            ),
        )
    except (OSError, RuntimeError) as exc:
        reason = str(exc)
        if reason != "DEPENDENCY_MANIFEST_HASH_MISMATCH":
            reason = f"DEPENDENCY_MANIFEST_INVALID:{reason}"
        return dependency_failure(reason)
    environment = dict(os.environ if environment is None else environment)
    current, reason = _canonical_prefixes(_environment_prefixes(
        environment, manifest["prefix_policy"]["current_prefixes_environment"]
    ))
    if reason:
        return dependency_failure(reason)
    allowed, reason = _canonical_prefixes(_environment_prefixes(
        environment, manifest["prefix_policy"]["allowed_prefixes_environment"]
    ))
    if reason:
        return dependency_failure(reason)
    if not current or current != allowed:
        return dependency_failure("DEPENDENCY_PREFIX_UNDECLARED")
    forbidden_roots = [
        Path(value).resolve()
        for value in manifest["prefix_policy"]["forbidden_prefix_roots"]
    ]
    if any(
        prefix == forbidden or forbidden in prefix.parents
        for prefix in current for forbidden in forbidden_roots
    ):
        return dependency_failure("DEPENDENCY_PREFIX_HISTORICAL")

    package_prefixes: dict[str, Path] = {}
    for package in manifest["packages"]:
        name = package["name"]
        matches = [
            prefix for prefix in current
            if _ordinary_file_within(
                prefix / "share/ament_index/resource_index/packages" / name,
                prefix,
            )
        ]
        if not matches:
            return dependency_failure(f"DEPENDENCY_PACKAGE_MISSING:{name}")
        if len(matches) != 1:
            return dependency_failure(f"DEPENDENCY_PACKAGE_DUPLICATE:{name}")
        package_prefixes[name] = matches[0]

    executable_count = 0
    config_count = 0
    for package in manifest["packages"]:
        name = package["name"]
        prefix = package_prefixes[name]
        for executable in package["executables"]:
            executable_path = prefix / "lib" / name / executable
            if not _ordinary_file_within(
                executable_path, prefix, executable=True
            ):
                return dependency_failure(
                    f"DEPENDENCY_EXECUTABLE_MISSING:{name}:{executable}"
                )
            if not _is_loadable_executable(executable_path, environment):
                return dependency_failure(
                    f"DEPENDENCY_EXECUTABLE_INVALID:{name}:{executable}"
                )
            executable_count += 1
        for relative in package["config_files"]:
            config_path = prefix / "share" / name / relative
            if not _ordinary_file_within(config_path, prefix):
                return dependency_failure(
                    f"DEPENDENCY_CONFIG_MISSING:{name}:{relative}"
                )
            if sha256_file(config_path) != manifest["config_hashes"][
                f"{name}:{relative}"
            ]:
                return dependency_failure(
                    f"DEPENDENCY_CONFIG_HASH_MISMATCH:{name}:{relative}"
                )
            config_count += 1

    runtime_library_count = 0
    for library in manifest["runtime_libraries"]:
        prefix = package_prefixes[library["package"]]
        runtime_library_path = prefix / library["relative_path"]
        if not _ordinary_file_within(runtime_library_path, prefix):
            return dependency_failure(
                "DEPENDENCY_RUNTIME_LIBRARY_MISSING:"
                f"{library['package']}:{library['relative_path']}"
            )
        if not _is_loadable_elf(runtime_library_path, {3}, environment):
            return dependency_failure(
                "DEPENDENCY_RUNTIME_LIBRARY_INVALID:"
                f"{library['package']}:{library['relative_path']}"
            )
        runtime_library_count += 1

    for component in manifest["components"]:
        prefix = package_prefixes[component["package"]]
        component_resource_path = (
            prefix / "share/ament_index/resource_index/rclcpp_components"
            / component["package"]
        )
        if not _ordinary_file_within(component_resource_path, prefix):
            return dependency_failure(
                f"DEPENDENCY_COMPONENT_MISSING:{component['package']}:{component['plugin']}"
            )
        try:
            registrations = {
                tuple(line.strip().split(";", 1))
                for line in component_resource_path.read_text().splitlines()
                if line.strip() and ";" in line
            }
        except OSError:
            registrations = set()
        expected = (component["plugin"], component["library"])
        if expected not in registrations:
            return dependency_failure(
                f"DEPENDENCY_COMPONENT_MISMATCH:{component['package']}:{component['plugin']}"
            )
        component_library_path = prefix / component["library"]
        if not _ordinary_file_within(component_library_path, prefix):
            return dependency_failure(
                f"DEPENDENCY_COMPONENT_LIBRARY_MISSING:{component['package']}:{component['plugin']}"
            )
        if not _is_loadable_elf(component_library_path, {3}, environment):
            return dependency_failure(
                f"DEPENDENCY_COMPONENT_LIBRARY_INVALID:{component['package']}:{component['plugin']}"
            )

    launch = manifest["launch_contract"]
    prefix = package_prefixes[launch["package"]]
    launch_contract_path = (
        prefix / "share" / launch["package"] / launch["relative_path"]
    )
    if (
        not _ordinary_file_within(launch_contract_path, prefix)
        or sha256_file(launch_contract_path) != launch["sha256"]
    ):
        return dependency_failure("DEPENDENCY_LAUNCH_CONTRACT_MISMATCH")
    return {
        "schema_version": result_schema,
        "dependency_ready": True,
        "failure_reason": "",
        "manifest_path": str(resolved_manifest_path),
        "manifest_sha256": binding["sha256"],
        "package_count": len(package_prefixes),
        "executable_count": executable_count,
        "component_count": len(manifest["components"]),
        "config_count": config_count,
        "runtime_library_count": runtime_library_count,
        "validated_prefixes": [str(prefix) for prefix in current],
    }


def derive_launch_environment_inventory(
    runs_root: Path, plan: list[dict[str, Any]]
) -> dict[str, Any]:
    """Derive every mutable r3 path from the fresh runs root, never the shell."""
    root = Path(runs_root).resolve()
    run_outputs = []
    child_environment: dict[str, str] | None = None
    for record in plan:
        binding = expected_launch_environment_binding(
            root, Path(record["run_dir"])
        )
        if child_environment is None:
            child_environment = binding["child_environment"]
        elif not exact_json_equal(
            child_environment, binding["child_environment"]
        ):
            raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:environment_drift")
        run_outputs.append({
            "run_id": record["run_id"],
            "mutable_output_paths": binding["mutable_output_paths"],
        })
    return {
        "schema_version": LAUNCH_ENVIRONMENT_SCHEMA,
        "runs_root": str(root),
        "child_environment": child_environment or {},
        "directory_modes": dict(LAUNCH_ENVIRONMENT_DIRECTORY_MODES),
        "run_outputs": run_outputs,
    }


def validate_launch_environment_inventory(
    inventory: Any, runs_root: Path, plan: list[dict[str, Any]],
    *, require_pristine: bool,
) -> dict[str, Any]:
    """Fail closed on unknown, aliased, escaping, symlinked or conflicting paths."""
    if not isinstance(inventory, dict) or set(inventory) != {
        "schema_version", "runs_root", "child_environment",
        "directory_modes", "run_outputs",
    }:
        raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:inventory_schema")
    root = Path(runs_root).resolve()
    if (
        inventory.get("schema_version") != LAUNCH_ENVIRONMENT_SCHEMA
        or inventory.get("runs_root") != str(root)
    ):
        raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:root_binding")
    outputs = inventory.get("run_outputs")
    if not isinstance(outputs, list) or len(outputs) != len(plan):
        raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:run_inventory")
    child_environment = inventory.get("child_environment")
    if inventory.get("directory_modes") != LAUNCH_ENVIRONMENT_DIRECTORY_MODES:
        raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:directory_modes")
    for record, output in zip(plan, outputs):
        if not isinstance(output, dict) or set(output) != {
            "run_id", "mutable_output_paths"
        } or output.get("run_id") != record["run_id"]:
            raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:run_binding")
        try:
            validated = validate_launch_environment_binding(
                child_environment, output.get("mutable_output_paths")
            )
        except RuntimeError as exc:
            raise RunnerError(f"LAUNCH_ENVIRONMENT_NOT_READY:{exc}") from exc
        expected = expected_launch_environment_binding(
            root, Path(record["run_dir"])
        )
        if not exact_json_equal(validated, expected):
            raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:path_mismatch")
        if require_pristine:
            for value in output["mutable_output_paths"].values():
                path = Path(value)
                if path.exists() or path.is_symlink():
                    raise RunnerError(
                        "LAUNCH_ENVIRONMENT_NOT_READY:conflicting_output"
                    )
    if require_pristine:
        environment_root = root / "launch_environment"
        if environment_root.exists() or environment_root.is_symlink():
            raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:conflicting_environment")
    return inventory


def prepare_launch_environment(
    runs_root: Path,
    plan: list[dict[str, Any]],
    *,
    inventory_factory: Callable[[Path, list[dict[str, Any]]], dict[str, Any]] = (
        derive_launch_environment_inventory
    ),
) -> dict[str, Any]:
    inventory = inventory_factory(Path(runs_root), plan)
    validate_launch_environment_inventory(
        inventory, runs_root, plan, require_pristine=True
    )
    child_environment = inventory["child_environment"]
    environment_root = Path(child_environment["HOME"]).parent
    root_fd = None
    try:
        environment_root.mkdir(mode=0o700)
        open_flags = os.O_RDONLY | os.O_DIRECTORY
        if hasattr(os, "O_NOFOLLOW"):
            open_flags |= os.O_NOFOLLOW
        root_fd = os.open(environment_root, open_flags)
        for key in LAUNCH_ENVIRONMENT_KEYS:
            path = Path(child_environment[key])
            mode = 0o700 if key == "XDG_RUNTIME_DIR" else 0o755
            os.mkdir(path.name, mode=mode, dir_fd=root_fd)
            directory_fd = os.open(path.name, open_flags, dir_fd=root_fd)
            try:
                if key == "XDG_RUNTIME_DIR":
                    os.fchmod(directory_fd, 0o700)
                metadata = os.fstat(directory_fd)
            finally:
                os.close(directory_fd)
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != os.geteuid()
                or not os.access(path, os.W_OK | os.X_OK)
            ):
                raise RunnerError(
                    f"LAUNCH_ENVIRONMENT_NOT_READY:directory_access:{key}"
                )
            if key == "XDG_RUNTIME_DIR" and (
                stat.S_IMODE(metadata.st_mode) != 0o700
            ):
                raise RunnerError(
                    "LAUNCH_ENVIRONMENT_NOT_READY:directory_mode:XDG_RUNTIME_DIR"
                )
    except (OSError, RuntimeError) as exc:
        if isinstance(exc, RunnerError):
            raise
        raise RunnerError(
            f"LAUNCH_ENVIRONMENT_NOT_READY:create:{type(exc).__name__}"
        ) from exc
    finally:
        if root_fd is not None:
            os.close(root_fd)
    return inventory


def child_launch_environment(
    base_environment: dict[str, str] | None,
    inventory: dict[str, Any],
) -> dict[str, str]:
    environment = dict(os.environ if base_environment is None else base_environment)
    environment.update(inventory["child_environment"])
    return environment


def _run_environment_binding(
    inventory: dict[str, Any], run_id: str
) -> dict[str, dict[str, str]]:
    matches = [
        output for output in inventory["run_outputs"]
        if output["run_id"] == run_id
    ]
    if len(matches) != 1:
        raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:run_binding_lookup")
    return {
        "child_environment": inventory["child_environment"],
        "mutable_output_paths": matches[0]["mutable_output_paths"],
    }


def launch_command(
    bundle: ProtocolBundle, record: dict[str, Any],
    launch_environment: dict[str, dict[str, str]] | None = None,
) -> list[str]:
    run_dir = Path(record["run_dir"]).resolve()
    manifest_path = run_dir / "p4_g0c_run_manifest.json"
    csv_path = run_dir / "p4_decisions.csv"
    schema = bundle.protocol["schema_version"]
    values = {
        "experiment": (
            "p4_g0c_metrics_calibration_v6" if schema == TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v5" if schema == CLOSED_FIXTURE_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v4" if schema == PROFILED_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v3" if schema == HARDENED_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v2" if schema == REPLACEMENT_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v1"
        ),
        "p4.g0c.protocol_path": bundle.protocol_path,
        "p4.g0c.protocol_sha256": bundle.protocol_sha256,
        "p4.g0c.registry_path": bundle.registry_path,
        "p4.g0c.registry_sha256": bundle.registry_sha256,
        "p4.g0c.fixture_path": bundle.fixture_path,
        "p4.g0c.fixture_sha256": bundle.fixture_sha256,
        "p4.g0c.run_id": record["run_id"],
        "p4.g0c.seed": record["seed"],
        "p4.g0c.repetition": record["repetition"],
        "p4.g0c.run_manifest_path": str(manifest_path),
        "p4.g0c.csv_path": str(csv_path),
        "runtime_root_dir": str(run_dir / "runtime"),
        "export_root_dir": str(run_dir / "exports"),
        "iap_log_root": str(run_dir / "runtime" / "iap_logs"),
    }
    if schema in {
        HARDENED_PROTOCOL_SCHEMA, PROFILED_PROTOCOL_SCHEMA,
        CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        if launch_environment is None:
            raise RunnerError("LAUNCH_ENVIRONMENT_NOT_READY:missing_binding")
        validate_launch_environment_binding(
            launch_environment.get("child_environment"),
            launch_environment.get("mutable_output_paths"),
        )
        child = launch_environment["child_environment"]
        outputs = launch_environment["mutable_output_paths"]
        values.update({
            "runtime_root_dir": outputs["runtime_root_dir"],
            "export_root_dir": outputs["export_root_dir"],
            "iap_log_root": outputs["iap_log_root"],
            "bag_output_dir": outputs["bag_output_dir"],
            "p4.g0c.child_home": child["HOME"],
            "p4.g0c.child_ros_home": child["ROS_HOME"],
            "p4.g0c.child_ros_log_dir": child["ROS_LOG_DIR"],
            "p4.g0c.child_tmpdir": child["TMPDIR"],
            "p4.g0c.child_xdg_runtime_dir": child["XDG_RUNTIME_DIR"],
        })
    return [
        "ros2", "launch", "iap", "test_planner.launch.py",
        *[f"{key}:={value}" for key, value in values.items()],
    ]


def _execute_launch(
    record: dict[str, Any], command: list[str], duration_s: float,
    required: dict[str, list[str]],
    environment: dict[str, str] | None = None,
) -> tuple[int, dict[str, Any]]:
    run_dir = Path(record["run_dir"])
    with (run_dir / "stdout.log").open("w") as output:
        launch = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT,
            env=environment,
        )
        monitor = RequiredProcessMonitor(launch.pid, required, duration_s)

        def controlled_shutdown() -> None:
            monitor.mark_controlled_shutdown()
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)

        monitor.start()
        monitor.launch_running = True
        try:
            exit_code = launch.wait(timeout=max(0.1, duration_s - 0.25))
        except subprocess.TimeoutExpired:
            controlled_shutdown()
            try:
                exit_code = launch.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                launch.kill()
                exit_code = launch.wait(timeout=10.0)
        finally:
            monitor_result = monitor.finish()
    return int(exit_code), monitor_result


def _validate_and_finalize_run(
    bundle: ProtocolBundle,
    record: dict[str, Any],
    monitor_result: dict[str, Any],
    launch_environment: dict[str, dict[str, str]] | None = None,
) -> dict[str, str]:
    run_dir = Path(record["run_dir"])
    manifest_path = run_dir / "p4_g0c_run_manifest.json"
    csv_path = run_dir / "p4_decisions.csv"
    inventory_path = run_dir / RUN_ARTIFACT_INVENTORY_FILENAME
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RunnerError(f"missing or malformed run manifest: {record['run_id']}") from exc
    if not isinstance(manifest, dict):
        raise RunnerError(
            f"missing or malformed run manifest: {record['run_id']}:root"
        )
    try:
        launch_manifest_path = bound_test_planner_manifest_path(
            run_dir, manifest.get("test_planner_manifest_path")
        )
    except RuntimeError as exc:
        raise RunnerError(
            f"run manifest binding mismatch: {record['run_id']}:"
            "test_planner_manifest_path"
        ) from exc
    protocol_schema = bundle.protocol["schema_version"]
    hardened = protocol_schema in {
        HARDENED_PROTOCOL_SCHEMA, PROFILED_PROTOCOL_SCHEMA,
        CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }
    replacement = protocol_schema in {
        REPLACEMENT_PROTOCOL_SCHEMA, HARDENED_PROTOCOL_SCHEMA,
        PROFILED_PROTOCOL_SCHEMA, CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }
    required_manifest = {
        "schema_version": (
            "p4_g0c_run_manifest_v6" if protocol_schema == TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
            else "p4_g0c_run_manifest_v5" if protocol_schema == CLOSED_FIXTURE_PROTOCOL_SCHEMA
            else "p4_g0c_run_manifest_v4" if protocol_schema == PROFILED_PROTOCOL_SCHEMA
            else "p4_g0c_run_manifest_v3" if hardened
            else "p4_g0c_run_manifest_v2" if replacement
            else "p4_g0c_run_manifest_v1"
        ),
        "run_id": record["run_id"],
        "seed": record["seed"],
        "repetition": record["repetition"],
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "csv_path": str(csv_path.resolve()),
        "gate": "G0C",
        "experiment": (
            "p4_g0c_metrics_calibration_v6" if protocol_schema == TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v5" if protocol_schema == CLOSED_FIXTURE_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v4" if protocol_schema == PROFILED_PROTOCOL_SCHEMA
            else "p4_g0c_metrics_calibration_v3" if hardened
            else "p4_g0c_metrics_calibration_v2" if replacement
            else "p4_g0c_metrics_calibration_v1"
        ),
        "scenario": "p4_g0c_free_corridor_v1",
        "decision_schema_version": "p4_collision_guide_decision_v1",
        "effective_values": bundle.protocol["effective_values"],
        "effective_config_sha256": effective_config_sha256(
            bundle.protocol["effective_values"]
        ),
        "required_process_set": list(REQUIRED_PROCESSES),
        "selection_applied": False,
        "record_bag": False,
        "start_rviz": False,
        "immutable_run_id": True,
        "overwrite_allowed": False,
    }
    if replacement:
        required_manifest.update({
            "dependency_manifest_sha256": bundle.protocol[
                "runtime_dependency_manifest"
            ]["sha256"],
            "replacement_lineage_sha256": bundle.protocol[
                "replacement_lineage"
            ]["sha256"],
        })
    if protocol_schema in {
        CLOSED_FIXTURE_PROTOCOL_SCHEMA, TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        required_manifest["admission_parameter"] = {
            "requested": True,
            "effective": True,
        }
    if hardened:
        if launch_environment is None:
            raise RunnerError(
                f"run manifest binding mismatch: {record['run_id']}:"
                "launch_environment"
            )
        try:
            validated_environment = validate_launch_environment_binding(
                manifest.get("child_environment"),
                manifest.get("mutable_output_paths"),
            )
        except RuntimeError as exc:
            raise RunnerError(
                f"run manifest binding mismatch: {record['run_id']}:"
                "launch_environment"
            ) from exc
        if not exact_json_equal(validated_environment, launch_environment):
            raise RunnerError(
                f"run manifest binding mismatch: {record['run_id']}:"
                "launch_environment"
            )
        required_manifest.update(launch_environment)
    manifest_effective = manifest.get("effective_values")
    if (
        not isinstance(manifest_effective, dict)
        or manifest.get("effective_config_sha256")
        != effective_config_sha256(manifest_effective)
    ):
        raise RunnerError(
            f"run manifest binding mismatch: "
            f"{record['run_id']}:effective_config_sha256"
        )
    for key, expected in required_manifest.items():
        if (
            not exact_json_equal(manifest.get(key), expected)
            if replacement
            else manifest.get(key) != expected
        ):
            raise RunnerError(f"run manifest binding mismatch: {record['run_id']}:{key}")
    try:
        with csv_path.open(newline="") as stream:
            reader = csv.DictReader(stream, strict=True)
            try:
                validate_decision_header(reader.fieldnames)
            except DecisionSchemaError as exc:
                raise RunnerError(
                    f"malformed P4 decision CSV: "
                    f"{record['run_id']}:{exc.code}"
                ) from exc
            rows = list(reader)
            if not rows:
                raise RunnerError(f"empty P4 decision CSV: {record['run_id']}")
            identities = set()
            tolerance = bundle.protocol[
                "path_ratio_consistency"
            ]["absolute_tolerance"]
            for row in rows:
                try:
                    typed = parse_decision_row(row, tolerance)
                except DecisionSchemaError as exc:
                    raise RunnerError(
                        f"malformed P4 decision CSV: "
                        f"{record['run_id']}:{exc.code}"
                    ) from exc
                identity = decision_identity(typed)
                if identity in identities:
                    raise RunnerError(
                        f"malformed P4 decision CSV: "
                        f"{record['run_id']}:duplicate_decision_identity"
                    )
                identities.add(identity)
    except (OSError, csv.Error) as exc:
        raise RunnerError(f"missing P4 decision CSV: {record['run_id']}") from exc
    try:
        launch_manifest = json.loads(launch_manifest_path.read_text())
        if not isinstance(launch_manifest, dict):
            raise ValueError("launch manifest root is not an object")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise RunnerError(
            f"missing or malformed launch manifest: {record['run_id']}"
        ) from exc
    try:
        validate_test_planner_effective_contract(bundle, launch_manifest)
    except RuntimeError as exc:
        raise RunnerError(
            f"launch manifest effective contract mismatch: "
            f"{record['run_id']}:{exc}"
        ) from exc
    manifest.update(monitor_result)
    manifest["runner_state"] = "COMPLETE"
    manifest["launch_exit_code"] = 0
    manifest["retry_count"] = 0
    try:
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
    except OSError as exc:
        raise RunnerError(
            f"run manifest finalization failed: {record['run_id']}"
        ) from exc
    if inventory_path.exists() or inventory_path.is_symlink():
        raise RunnerError(
            f"existing run artifact inventory is forbidden: {record['run_id']}"
        )
    try:
        inventory = make_run_artifact_inventory(run_dir, record["run_id"])
        with inventory_path.open("xb") as stream:
            stream.write(canonical_bytes(inventory))
    except (OSError, RuntimeError) as exc:
        raise RunnerError(
            f"run artifact inventory failed: {record['run_id']}:{exc}"
        ) from exc
    return {
        "artifact_inventory_path": str(inventory_path.resolve()),
        "artifact_inventory_sha256": sha256_file(inventory_path),
        "test_planner_manifest_path": str(launch_manifest_path.resolve()),
        "test_planner_manifest_sha256": sha256_file(launch_manifest_path),
    }


def _base_result(bundle: ProtocolBundle, plan: list[dict[str, Any]]) -> dict[str, Any]:
    registered_ids = [record["run_id"] for record in plan]
    return {
        "schema_version": (
            RUNNER_SCHEMA_V6
            if bundle.protocol["schema_version"] == TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
            else RUNNER_SCHEMA_V5 if bundle.protocol["schema_version"] == CLOSED_FIXTURE_PROTOCOL_SCHEMA
            else RUNNER_SCHEMA_V4 if bundle.protocol["schema_version"] == PROFILED_PROTOCOL_SCHEMA
            else RUNNER_SCHEMA_V3 if bundle.protocol["schema_version"] == HARDENED_PROTOCOL_SCHEMA
            else RUNNER_SCHEMA_V2
            if bundle.protocol["schema_version"] == REPLACEMENT_PROTOCOL_SCHEMA
            else RUNNER_SCHEMA_V1
        ),
        "runner_state": "PLANNED",
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "registered_run_ids": registered_ids,
        "attempted_run_ids": [],
        "completed_run_ids": [],
        "attempts": [],
        "runs": plan,
        "completed_run_count": 0,
        "launch_invocations": 0,
        "gpu_preflight_invocations": 0,
        "launch_started": False,
        "retries": 0,
        "failure_reason": "",
        "failed_run_id": "",
    }


def _persist_result(runs_root: Path, result: dict[str, Any]) -> None:
    runs_root.mkdir(parents=True, exist_ok=True)
    (runs_root / "p4_g0c_runner_state.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )


def run(
    bundle: ProtocolBundle,
    runs_root: Path,
    *,
    plan_only: bool = False,
    preflight_only: bool = False,
    dependency_preflight_only: bool = False,
    dependency_manifest_path: Path | None = None,
    dependency_environment: dict[str, str] | None = None,
    gpu_preflight: Callable[[Path], dict[str, Any]] = run_gpu_preflight,
    launch_executor: Callable[..., tuple[int, dict[str, Any]]] = _execute_launch,
    launch_environment_factory: Callable[
        [Path, list[dict[str, Any]]], dict[str, Any]
    ] = derive_launch_environment_inventory,
) -> dict[str, Any]:
    if sum(bool(mode) for mode in (
        plan_only, preflight_only, dependency_preflight_only
    )) > 1:
        raise RunnerError("runner modes are mutually exclusive")
    requested_root = Path(runs_root).expanduser()
    runs_root = requested_root.resolve()
    plan = expand_run_plan(bundle.protocol, runs_root)
    result = _base_result(bundle, plan)
    if plan_only:
        return result
    if requested_root.is_symlink():
        raise RunnerError(f"symlink runs root is forbidden: {requested_root}")
    if runs_root.exists():
        if not runs_root.is_dir() or runs_root.is_symlink():
            raise RunnerError(f"runs root is not an ordinary directory: {runs_root}")
        try:
            existing_children = sorted(entry.name for entry in runs_root.iterdir())
        except OSError as exc:
            raise RunnerError(f"runs root is unreadable: {runs_root}") from exc
        if existing_children:
            if "p4_g0c_runner_state.json" in existing_children:
                raise RunnerError(
                    "existing runner state is forbidden: "
                    f"{runs_root / 'p4_g0c_runner_state.json'}"
                )
            registered_existing = [
                run_id for run_id in result["registered_run_ids"]
                if run_id in existing_children
            ]
            if registered_existing:
                raise RunnerError(
                    "existing run directory is forbidden: "
                    f"{runs_root / registered_existing[0]}"
                )
            raise RunnerError(
                f"dirty runs root is forbidden: {runs_root}:"
                f"{','.join(existing_children)}"
            )

    result["runner_state"] = "DEPENDENCY_PREFLIGHT_RUNNING"
    dependency = validate_runtime_dependencies(
        bundle,
        dependency_manifest_path,
        dependency_environment,
    )
    result["dependency_preflight"] = dependency
    if dependency.get("dependency_ready") is not True:
        result.update({
            "runner_state": "FAILED",
            "failure_reason": dependency.get(
                "failure_reason", "DEPENDENCY_PREFLIGHT_FAILED"
            ),
        })
        _persist_result(runs_root, result)
        return result
    result["runner_state"] = "DEPENDENCY_PREFLIGHT_PASS"
    _persist_result(runs_root, result)
    profile = validate_p0_profile_binding(bundle)
    result["p0_profile_preflight"] = profile
    if profile.get("profile_ready") is not True:
        result.update({"runner_state": "FAILED",
                       "failure_reason": profile["failure_reason"]})
        _persist_result(runs_root, result)
        return result
    if dependency_preflight_only:
        return result

    launch_environment_inventory = None
    launch_process_environment = None
    if bundle.protocol["schema_version"] in {
        HARDENED_PROTOCOL_SCHEMA, PROFILED_PROTOCOL_SCHEMA,
        CLOSED_FIXTURE_PROTOCOL_SCHEMA,
        TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
    }:
        result["runner_state"] = "LAUNCH_ENVIRONMENT_PREFLIGHT_RUNNING"
        _persist_result(runs_root, result)
        try:
            launch_environment_inventory = prepare_launch_environment(
                runs_root,
                plan,
                inventory_factory=launch_environment_factory,
            )
        except (RunnerError, OSError) as exc:
            reason = str(exc)
            if not reason.startswith("LAUNCH_ENVIRONMENT_NOT_READY"):
                reason = f"LAUNCH_ENVIRONMENT_NOT_READY:{type(exc).__name__}"
            result.update({
                "runner_state": "FAILED",
                "failure_reason": reason,
            })
            _persist_result(runs_root, result)
            return result
        result["launch_environment"] = launch_environment_inventory
        result["runner_state"] = "LAUNCH_ENVIRONMENT_PREFLIGHT_PASS"
        _persist_result(runs_root, result)
        launch_process_environment = child_launch_environment(
            dependency_environment, launch_environment_inventory
        )

    preflight_path = runs_root / "preflight"

    result["runner_state"] = "GPU_PREFLIGHT_RUNNING"
    _persist_result(runs_root, result)
    result["gpu_preflight_invocations"] = 1
    try:
        preflight = gpu_preflight(preflight_path)
    except Exception as exc:  # Preflight failure must remain non-overwriteable.
        result.update({
            "runner_state": "FAILED",
            "failure_reason": f"gpu_preflight_error:{type(exc).__name__}",
        })
        _persist_result(runs_root, result)
        return result
    result["gpu_preflight"] = preflight
    if preflight.get("gpu_ready") is not True:
        result.update({
            "runner_state": "FAILED",
            "failure_reason": "GPU_NOT_READY",
            "gpu_failure_reason": preflight.get("failure_reason", "unknown"),
        })
        _persist_result(runs_root, result)
        return result
    result["runner_state"] = "PREFLIGHT_PASS"
    if preflight_only:
        _persist_result(runs_root, result)
        return result

    duration_s = float(bundle.protocol.get("run_duration_s", 90.0))
    for record in plan:
        run_dir = Path(record["run_dir"])
        run_dir.mkdir(parents=True, exist_ok=False)
        run_environment = (
            _run_environment_binding(
                launch_environment_inventory, record["run_id"]
            )
            if launch_environment_inventory is not None else None
        )
        command = launch_command(bundle, record, run_environment)
        (run_dir / "launch_command.json").write_text(
            json.dumps(command, indent=2) + "\n"
        )
        result["runner_state"] = "RUNNING"
        result["launch_started"] = True
        result["attempted_run_ids"].append(record["run_id"])
        result["attempts"].append({
            "attempt_index": len(result["attempted_run_ids"]),
            "run_id": record["run_id"],
            "state": "RUNNING",
        })
        result["launch_invocations"] = len(result["attempted_run_ids"])
        _persist_result(runs_root, result)
        try:
            if bundle.protocol["schema_version"] in {
                HARDENED_PROTOCOL_SCHEMA, PROFILED_PROTOCOL_SCHEMA,
                CLOSED_FIXTURE_PROTOCOL_SCHEMA,
                TEMPORAL_SUPPORT_PROTOCOL_SCHEMA,
            }:
                exit_code, monitor_result = launch_executor(
                    record, command, duration_s, REQUIRED_PROCESSES,
                    launch_process_environment,
                )
            else:
                exit_code, monitor_result = launch_executor(
                    record, command, duration_s, REQUIRED_PROCESSES
                )
        except Exception as exc:  # Launch boundary failures must remain in the ledger.
            result["attempts"][-1]["state"] = "FAILED"
            result.update({
                "runner_state": "FAILED",
                "failure_reason": f"launch_executor_error:{type(exc).__name__}",
                "failed_run_id": record["run_id"],
            })
            _persist_result(runs_root, result)
            return result
        if exit_code != 0 or monitor_result.get("required_processes_ok") is not True:
            result["attempts"][-1]["state"] = "FAILED"
            result.update({
                "runner_state": "FAILED",
                "failure_reason": (
                    f"launch_exit_{exit_code}" if exit_code != 0
                    else "required_process_failure"
                ),
                "failed_run_id": record["run_id"],
                "required_process_monitor": monitor_result,
            })
            _persist_result(runs_root, result)
            return result
        try:
            inventory_binding = _validate_and_finalize_run(
                bundle, record, monitor_result, run_environment
            )
        except (RunnerError, OSError) as exc:
            result["attempts"][-1]["state"] = "FAILED"
            result.update({
                "runner_state": "FAILED",
                "failure_reason": str(exc),
                "failed_run_id": record["run_id"],
            })
            _persist_result(runs_root, result)
            return result
        result["attempts"][-1].update(inventory_binding)
        result["attempts"][-1]["state"] = "COMPLETE"
        result["completed_run_ids"].append(record["run_id"])
        result["completed_run_count"] = len(result["completed_run_ids"])
        _persist_result(runs_root, result)

    result["runner_state"] = "COMPLETE"
    _persist_result(runs_root, result)
    return result


def _parser() -> argparse.ArgumentParser:
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=repo / "config/icra27/p4_g0c_protocol_v6.json")
    parser.add_argument("--registry", type=Path, default=repo / "config/icra27/p4_threshold_registry_v6.json")
    parser.add_argument("--fixture", type=Path, default=repo / "config/icra27/p4_g0c_live_fixture_v2.json")
    parser.add_argument(
        "--dependency-manifest", type=Path,
        default=repo / "config/icra27/p4_g0c_runtime_dependencies_v6.json",
    )
    parser.add_argument("--runs-root", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--plan-only", action="store_true")
    modes.add_argument("--preflight-only", action="store_true")
    modes.add_argument("--dependency-preflight-only", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        registered_v1_path = (
            Path(__file__).resolve().parents[2]
            / "config/icra27/p4_g0c_protocol_v1.json"
        )
        registered_v2_path = (
            Path(__file__).resolve().parents[2]
            / "config/icra27/p4_g0c_protocol_v2.json"
        )
        registered_v3_path = (
            Path(__file__).resolve().parents[2]
            / "config/icra27/p4_g0c_protocol_v3.json"
        )
        registered_v4_path = (
            Path(__file__).resolve().parents[2]
            / "config/icra27/p4_g0c_protocol_v4.json"
        )
        registered_v5_path = (
            Path(__file__).resolve().parents[2]
            / "config/icra27/p4_g0c_protocol_v5.json"
        )
        trusted_schema = (
            LEGACY_PROTOCOL_SCHEMA
            if args.protocol.resolve() == registered_v1_path
            else REPLACEMENT_PROTOCOL_SCHEMA
            if args.protocol.resolve() == registered_v2_path
            else HARDENED_PROTOCOL_SCHEMA
            if args.protocol.resolve() == registered_v3_path
            else PROFILED_PROTOCOL_SCHEMA
            if args.protocol.resolve() == registered_v4_path
            else CLOSED_FIXTURE_PROTOCOL_SCHEMA
            if args.protocol.resolve() == registered_v5_path
            else TEMPORAL_SUPPORT_PROTOCOL_SCHEMA
        )
        bundle = load_bundle(
            args.protocol,
            args.registry,
            args.fixture,
            expected_protocol_schema=trusted_schema,
        )
        result = run(
            bundle,
            args.runs_root,
            plan_only=args.plan_only,
            preflight_only=args.preflight_only,
            dependency_preflight_only=args.dependency_preflight_only,
            dependency_manifest_path=args.dependency_manifest,
        )
    except (RunnerError, RuntimeError) as exc:
        print(f"P4_G0C_RUNNER_FAILED: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    if result["failure_reason"] == "GPU_NOT_READY":
        print("GPU_NOT_READY", file=sys.stderr)
        return 2
    return 0 if result["runner_state"] in {
        "PLANNED", "DEPENDENCY_PREFLIGHT_PASS", "PREFLIGHT_PASS", "COMPLETE"
    } else 2


if __name__ == "__main__":
    raise SystemExit(main())
