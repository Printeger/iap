#!/usr/bin/env python3
"""Fail-closed runner for the registered P4-G0C calibration matrix.

ICRA-042 registers this tool and tests it with synthetic boundaries only. Live
execution remains unauthorized until a later task explicitly invokes it.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
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
    RUN_ARTIFACT_INVENTORY_FILENAME,
    DecisionSchemaError,
    ProtocolBundle,
    bound_test_planner_manifest_path,
    canonical_bytes,
    decision_identity,
    effective_config_sha256,
    expand_run_plan,
    load_canonical_json,
    load_protocol_bundle,
    make_run_artifact_inventory,
    parse_decision_row,
    sha256_file,
    validate_decision_header,
)
from run_gate0_qualification import (  # noqa: E402
    RequiredProcessMonitor,
    run_gpu_preflight,
)


RUNNER_SCHEMA_V1 = "p4_g0c_runner_state_v3"
RUNNER_SCHEMA_V2 = "p4_g0c_runner_state_v4"
DEPENDENCY_SCHEMA = "p4_g0c_runtime_dependencies_v2"
REPLACEMENT_PROTOCOL_SCHEMA = "p4_g0c_protocol_v2"
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
    protocol_path: Path, registry_path: Path, fixture_path: Path
) -> ProtocolBundle:
    bundle = load_protocol_bundle(protocol_path, registry_path, fixture_path)
    bundle.protocol_path = str(Path(protocol_path).resolve())
    bundle.registry_path = str(Path(registry_path).resolve())
    bundle.fixture_path = str(Path(fixture_path).resolve())
    if bundle.protocol.get("schema_version") == REPLACEMENT_PROTOCOL_SCHEMA:
        repository_root = Path(protocol_path).resolve().parents[2]
        bundle.dependency_manifest_path = str(
            repository_root
            / bundle.protocol["runtime_dependency_manifest"]["path"]
        )
    return bundle


def _dependency_failure(reason: str) -> dict[str, Any]:
    return {
        "schema_version": "p4_g0c_dependency_preflight_result_v2",
        "dependency_ready": False,
        "failure_reason": reason,
        "package_count": 0,
        "executable_count": 0,
        "component_count": 0,
        "config_count": 0,
        "runtime_library_count": 0,
    }


def load_runtime_dependency_manifest(
    path: Path, expected_sha256: str
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
    if set(manifest) != required_keys or manifest.get("schema_version") != DEPENDENCY_SCHEMA:
        raise RunnerError("DEPENDENCY_MANIFEST_SCHEMA_MISMATCH")
    if manifest.get("experiment") != "p4_g0c_metrics_calibration_v2":
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
    if bundle.protocol.get("schema_version") != REPLACEMENT_PROTOCOL_SCHEMA:
        return _dependency_failure("DEPENDENCY_PROTOCOL_V2_REQUIRED")
    binding = bundle.protocol["runtime_dependency_manifest"]
    path = Path(manifest_path or bundle.dependency_manifest_path)
    try:
        manifest = load_runtime_dependency_manifest(path, binding["sha256"])
    except (OSError, RuntimeError) as exc:
        reason = str(exc)
        if reason != "DEPENDENCY_MANIFEST_HASH_MISMATCH":
            reason = f"DEPENDENCY_MANIFEST_INVALID:{reason}"
        return _dependency_failure(reason)
    environment = dict(os.environ if environment is None else environment)
    current, reason = _canonical_prefixes(_environment_prefixes(
        environment, manifest["prefix_policy"]["current_prefixes_environment"]
    ))
    if reason:
        return _dependency_failure(reason)
    allowed, reason = _canonical_prefixes(_environment_prefixes(
        environment, manifest["prefix_policy"]["allowed_prefixes_environment"]
    ))
    if reason:
        return _dependency_failure(reason)
    if not current or current != allowed:
        return _dependency_failure("DEPENDENCY_PREFIX_UNDECLARED")
    forbidden_roots = [
        Path(value).resolve()
        for value in manifest["prefix_policy"]["forbidden_prefix_roots"]
    ]
    if any(
        prefix == forbidden or forbidden in prefix.parents
        for prefix in current for forbidden in forbidden_roots
    ):
        return _dependency_failure("DEPENDENCY_PREFIX_HISTORICAL")

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
            return _dependency_failure(f"DEPENDENCY_PACKAGE_MISSING:{name}")
        if len(matches) != 1:
            return _dependency_failure(f"DEPENDENCY_PACKAGE_DUPLICATE:{name}")
        package_prefixes[name] = matches[0]

    executable_count = 0
    config_count = 0
    for package in manifest["packages"]:
        name = package["name"]
        prefix = package_prefixes[name]
        for executable in package["executables"]:
            path = prefix / "lib" / name / executable
            if not _ordinary_file_within(path, prefix, executable=True):
                return _dependency_failure(
                    f"DEPENDENCY_EXECUTABLE_MISSING:{name}:{executable}"
                )
            if not _is_loadable_executable(path, environment):
                return _dependency_failure(
                    f"DEPENDENCY_EXECUTABLE_INVALID:{name}:{executable}"
                )
            executable_count += 1
        for relative in package["config_files"]:
            path = prefix / "share" / name / relative
            if not _ordinary_file_within(path, prefix):
                return _dependency_failure(
                    f"DEPENDENCY_CONFIG_MISSING:{name}:{relative}"
                )
            if sha256_file(path) != manifest["config_hashes"][f"{name}:{relative}"]:
                return _dependency_failure(
                    f"DEPENDENCY_CONFIG_HASH_MISMATCH:{name}:{relative}"
                )
            config_count += 1

    runtime_library_count = 0
    for library in manifest["runtime_libraries"]:
        prefix = package_prefixes[library["package"]]
        path = prefix / library["relative_path"]
        if not _ordinary_file_within(path, prefix):
            return _dependency_failure(
                "DEPENDENCY_RUNTIME_LIBRARY_MISSING:"
                f"{library['package']}:{library['relative_path']}"
            )
        if not _is_loadable_elf(path, {3}, environment):
            return _dependency_failure(
                "DEPENDENCY_RUNTIME_LIBRARY_INVALID:"
                f"{library['package']}:{library['relative_path']}"
            )
        runtime_library_count += 1

    for component in manifest["components"]:
        prefix = package_prefixes[component["package"]]
        resource = (
            prefix / "share/ament_index/resource_index/rclcpp_components"
            / component["package"]
        )
        if not _ordinary_file_within(resource, prefix):
            return _dependency_failure(
                f"DEPENDENCY_COMPONENT_MISSING:{component['package']}:{component['plugin']}"
            )
        try:
            registrations = {
                tuple(line.strip().split(";", 1))
                for line in resource.read_text().splitlines()
                if line.strip() and ";" in line
            }
        except OSError:
            registrations = set()
        expected = (component["plugin"], component["library"])
        if expected not in registrations:
            return _dependency_failure(
                f"DEPENDENCY_COMPONENT_MISMATCH:{component['package']}:{component['plugin']}"
            )
        library = prefix / component["library"]
        if not _ordinary_file_within(library, prefix):
            return _dependency_failure(
                f"DEPENDENCY_COMPONENT_LIBRARY_MISSING:{component['package']}:{component['plugin']}"
            )
        if not _is_loadable_elf(library, {3}, environment):
            return _dependency_failure(
                f"DEPENDENCY_COMPONENT_LIBRARY_INVALID:{component['package']}:{component['plugin']}"
            )

    launch = manifest["launch_contract"]
    prefix = package_prefixes[launch["package"]]
    launch_path = prefix / "share" / launch["package"] / launch["relative_path"]
    if (
        not _ordinary_file_within(launch_path, prefix)
        or sha256_file(launch_path) != launch["sha256"]
    ):
        return _dependency_failure("DEPENDENCY_LAUNCH_CONTRACT_MISMATCH")
    return {
        "schema_version": "p4_g0c_dependency_preflight_result_v2",
        "dependency_ready": True,
        "failure_reason": "",
        "manifest_path": str(path.resolve()),
        "manifest_sha256": binding["sha256"],
        "package_count": len(package_prefixes),
        "executable_count": executable_count,
        "component_count": len(manifest["components"]),
        "config_count": config_count,
        "runtime_library_count": runtime_library_count,
        "validated_prefixes": [str(prefix) for prefix in current],
    }


def launch_command(bundle: ProtocolBundle, record: dict[str, Any]) -> list[str]:
    run_dir = Path(record["run_dir"]).resolve()
    manifest_path = run_dir / "p4_g0c_run_manifest.json"
    csv_path = run_dir / "p4_decisions.csv"
    values = {
        "experiment": (
            "p4_g0c_metrics_calibration_v2"
            if bundle.protocol["schema_version"] == REPLACEMENT_PROTOCOL_SCHEMA
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
    return [
        "ros2", "launch", "iap", "test_planner.launch.py",
        *[f"{key}:={value}" for key, value in values.items()],
    ]


def _execute_launch(
    record: dict[str, Any], command: list[str], duration_s: float,
    required: dict[str, list[str]],
) -> tuple[int, dict[str, Any]]:
    run_dir = Path(record["run_dir"])
    with (run_dir / "stdout.log").open("w") as output:
        launch = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT
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
    replacement = (
        bundle.protocol["schema_version"] == REPLACEMENT_PROTOCOL_SCHEMA
    )
    required_manifest = {
        "schema_version": (
            "p4_g0c_run_manifest_v2" if replacement
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
            "p4_g0c_metrics_calibration_v2" if replacement
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
    for key, expected in required_manifest.items():
        if manifest.get(key) != expected:
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
    try:
        launch_manifest = json.loads(launch_manifest_path.read_text())
        if not isinstance(launch_manifest, dict):
            raise ValueError("launch manifest root is not an object")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise RunnerError(
            f"missing or malformed launch manifest: {record['run_id']}"
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
            RUNNER_SCHEMA_V2
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
    if dependency_preflight_only:
        return result

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
        command = launch_command(bundle, record)
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
                bundle, record, monitor_result
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
    parser.add_argument("--protocol", type=Path, default=repo / "config/icra27/p4_g0c_protocol_v2.json")
    parser.add_argument("--registry", type=Path, default=repo / "config/icra27/p4_threshold_registry_v2.json")
    parser.add_argument("--fixture", type=Path, default=repo / "config/icra27/p4_g0c_live_fixture_v1.json")
    parser.add_argument(
        "--dependency-manifest", type=Path,
        default=repo / "config/icra27/p4_g0c_runtime_dependencies_v2.json",
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
        bundle = load_bundle(args.protocol, args.registry, args.fixture)
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
