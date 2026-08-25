#!/usr/bin/env python3
"""Immutable P4-G0C protocol loading, hashing, and run registration."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path, PurePosixPath
from typing import Any


PROTOCOL_SCHEMA_V1 = "p4_g0c_protocol_v1"
PROTOCOL_SCHEMA_V2 = "p4_g0c_protocol_v2"
PROTOCOL_SCHEMA_V3 = "p4_g0c_protocol_v3"
PROTOCOL_SCHEMA_V4 = "p4_g0c_protocol_v4"
PROTOCOL_SCHEMAS = {PROTOCOL_SCHEMA_V1, PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}
REGISTRY_SCHEMA_V1 = "p4_threshold_registry_v1"
REGISTRY_SCHEMA_V2 = "p4_threshold_registry_v2"
REGISTRY_SCHEMA_V3 = "p4_threshold_registry_v3"
REGISTRY_SCHEMA_V4 = "p4_threshold_registry_v4"
REGISTRY_SCHEMAS = {REGISTRY_SCHEMA_V1, REGISTRY_SCHEMA_V2, REGISTRY_SCHEMA_V3, REGISTRY_SCHEMA_V4}
FIXTURE_SCHEMA = "p4_g0c_fixture_v1"
PROPOSED_STATE = "PROPOSED_UNCALIBRATED"
GATE_NAMES = (
    "mean_improvement_min",
    "max_improvement_min",
    "path_ratio_max",
    "total_search_timeout_s",
)
RUN_ID_PATTERN_V1 = re.compile(r"^p4-g0c-seed([0-9]+)-rep([0-9]{2})$")
RUN_ID_PATTERN_V2 = re.compile(r"^p4-g0c-r2-seed([0-9]+)-rep([0-9]{2})$")
RUN_ID_PATTERN_V3 = re.compile(r"^p4-g0c-r3-seed([0-9]+)-rep([0-9]{2})$")
RUN_ID_PATTERN_V4 = re.compile(r"^p4-g0c-r4-seed([0-9]+)-rep([0-9]{2})$")
RUN_ID_PATTERNS = (RUN_ID_PATTERN_V1, RUN_ID_PATTERN_V2, RUN_ID_PATTERN_V3, RUN_ID_PATTERN_V4)
DECISION_SCHEMA = "p4_collision_guide_decision_v1"
DECISION_CSV_COLUMNS = (
    "schema_version", "stamp", "planning_attempt_id",
    "collision_segment_id", "request_hash", "snapshot_generation_id",
    "snapshot_stamp_s", "snapshot_frame", "query_base_time_s",
    "occupancy_epoch", "status", "reason", "selection_applied",
    "original_hash", "risk_hash", "selected_hash",
    "original_sample_count", "original_valid_count",
    "original_unknown_count", "original_stale_count",
    "original_non_finite_count", "original_mean", "original_max",
    "risk_sample_count", "risk_valid_count", "risk_unknown_count",
    "risk_stale_count", "risk_non_finite_count", "risk_mean", "risk_max",
    "original_path_length", "risk_path_length", "path_length_ratio",
    "original_search_latency_ms", "risk_search_latency_ms",
    "total_search_latency_ms",
)
DECISION_INTEGER_COLUMNS = (
    "planning_attempt_id", "collision_segment_id", "snapshot_generation_id",
    "occupancy_epoch", "selection_applied", "original_sample_count",
    "original_valid_count", "original_unknown_count", "original_stale_count",
    "original_non_finite_count", "risk_sample_count", "risk_valid_count",
    "risk_unknown_count", "risk_stale_count", "risk_non_finite_count",
)
DECISION_FLOAT_COLUMNS = (
    "stamp", "snapshot_stamp_s", "query_base_time_s", "original_mean",
    "original_max", "risk_mean", "risk_max", "original_path_length",
    "risk_path_length", "path_length_ratio", "original_search_latency_ms",
    "risk_search_latency_ms", "total_search_latency_ms",
)
DECISION_NONEMPTY_COLUMNS = (
    "schema_version", "request_hash", "snapshot_frame", "status", "reason",
    "original_hash", "risk_hash", "selected_hash",
)
DECISION_IDENTITY_COLUMNS = (
    "planning_attempt_id", "collision_segment_id", "request_hash",
)
RUN_ARTIFACT_INVENTORY_SCHEMA = "p4_g0c_run_artifact_inventory_v1"
RUN_ARTIFACT_INVENTORY_FILENAME = "p4_g0c_artifact_inventory.json"
# Acyclic trust root: these full-file hashes live only in the shared loader.
# The protocol binds dependency -> launch, so the hash-bound launch must never
# embed either value and create protocol -> dependency -> launch -> protocol.
P4_G0C_PROTOCOL_V2_TRUSTED_SHA256 = (
    "8b0b2c3ed531680c6c8268738cb1bcb9136f39d2b97e68769e54a53afe59de79"
)
P4_G0C_REGISTRY_V2_TRUSTED_SHA256 = (
    "99ccf38c317d45d8605a7e382628a8f0afd32c8097a763d05bfdcc5807beb94f"
)
P4_G0C_PROTOCOL_V3_TRUSTED_SHA256 = (
    "7df40eff39a8c6e40e5f893719df6dd9a21ff97a5941992c075fd5ead4729401"
)
P4_G0C_REGISTRY_V3_TRUSTED_SHA256 = (
    "8825c70c3814b574ba4709a94aac4d5d85d37d617b453a3aaefb4b80da5d82c8"
)
P4_G0C_PROTOCOL_V4_TRUSTED_SHA256 = (
    "4a25348a040730b9d59f3c4709f2e84b8d0234cb1b8324601a8a7955b5432da8"
)
P4_G0C_REGISTRY_V4_TRUSTED_SHA256 = (
    "524a262a823873b2a4cb4c3b454d7eed8f2aac3475bd1385f3ea698a8b291900"
)
LAUNCH_ENVIRONMENT_SCHEMA = "p4_g0c_launch_environment_v1"
LAUNCH_ENVIRONMENT_KEYS = (
    "HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR", "XDG_RUNTIME_DIR",
)
LAUNCH_ENVIRONMENT_DIRECTORY_MODES = {"XDG_RUNTIME_DIR": "0700"}
MUTABLE_OUTPUT_KEYS = (
    "bag_output_dir",
    "decision_csv_path",
    "export_root_dir",
    "iap_log_root",
    "launch_command_path",
    "run_manifest_path",
    "runtime_root_dir",
    "stdout_log_path",
)
FROZEN_NUMERICAL_NOISE_FLOOR = {
    "calibration_mutable": False,
    "derivation": {
        "artifact": "ieee754_binary64_precision_bound_v1",
        "binary64_epsilon": 2.220446049250313e-16,
        "multiplier": 4096,
        "rounding": "round_up_to_1e-12",
        "source": "deterministic_numeric_precision_only",
        "unrounded_bound": 9.094947017729282e-13,
    },
    "unit": "risk_cost",
    "value": 1e-12,
}
FROZEN_QUANTILES = {
    "definition": (
        "sort finite values ascending; h=(n-1)*p; interpolate between "
        "floor(h) and ceil(h)"
    ),
    "interpolation": "linear_lower_plus_fraction_times_upper_minus_lower",
    "method": "TYPE_7_LINEAR",
    "tie_behavior": "stable_input_row_index",
    "units": {
        "improvement": "risk_cost",
        "path_ratio": "dimensionless",
        "total_search": "s",
    },
}
FROZEN_THRESHOLD_FORMULAS = {
    "max_improvement_min": "Q10(original_max-risk_max)",
    "mean_improvement_min": "Q10(original_mean-risk_mean)",
    "path_ratio_max": "min(1.30,Q95(path_ratio)+0.02)",
    "total_search_timeout_s": (
        "min(0.40,Q95(total_search_s)+max(0.01,0.20*Q95(total_search_s)))"
    ),
}
TEST_PLANNER_TOP_LEVEL_EFFECTIVE_MAP = {
    "manager/p1_collision_fanout_clearance_m": (
        "manager/p1_collision_fanout_clearance_m"
    ),
    "manager/p1_collision_fanout_mirror_y": (
        "manager/p1_collision_fanout_mirror_y"
    ),
    "manager/p1_collision_fanout_preserve_homotopies": (
        "manager/p1_collision_fanout_preserve_homotopies"
    ),
    "manager/use_distinctive_trajs": "manager/use_distinctive_trajs",
    "p0.enable_risk_grid": "p0.enable_risk_grid",
    "p1.debug_csv_enable": "p1.debug_csv_enable",
    "p1.metrics_only": "p1.metrics_only",
    "p1.use_integrity_cost": "p1.use_integrity_cost",
    "p2.debug_csv_enable": "p2.debug_csv_enable",
    "p2.enable_candidate_ranking": "p2.enable_candidate_ranking",
    "p2.metrics_only": "p2.metrics_only",
    "p3.debug_csv_enable": "p3.debug_csv_enable",
    "p3.enable_global_reference_bias": "p3.enable_global_reference_bias",
    "p3.enable_local_reference_bias": "p3.enable_local_reference_bias",
    "p4.debug_csv_enable": "p4.debug_csv_enable",
    "p4.enable_risk_aware_astar": "p4.enable_risk_aware_astar",
    "p4.metrics_only": "p4.metrics_only",
    "planner_enable_p1": "planner_enable_p1",
    "planner_enable_p2": "planner_enable_p2",
    "planner_enable_p3_global": "planner_enable_p3_global",
    "planner_enable_p3_local": "planner_enable_p3_local",
    "planner_enable_p4": "planner_enable_p4",
    "planner_enable_p5_final": "planner_enable_p5_final",
    "planner_enable_p5_runtime": "planner_enable_p5_runtime",
    "planner_safety_profile": "planner_safety_profile",
    "record_bag": "record_bag",
    "run_validator": "run_validator",
    "start_rviz": "start_rviz",
}


class ProtocolError(RuntimeError):
    """A versioned G0C artifact violates the frozen pre-data contract."""


class DecisionSchemaError(RuntimeError):
    """A production P4 decision row violates the shared typed schema."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class ProtocolBundle:
    def __init__(
        self,
        protocol: dict[str, Any],
        registry: dict[str, Any],
        fixture: dict[str, Any],
        protocol_sha256: str,
        registry_sha256: str,
        fixture_sha256: str,
    ) -> None:
        self.protocol = protocol
        self.registry = registry
        self.fixture = fixture
        self.protocol_sha256 = protocol_sha256
        self.registry_sha256 = registry_sha256
        self.fixture_sha256 = fixture_sha256


def canonical_bytes(payload: Any) -> bytes:
    return (
        json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        + "\n"
    ).encode("utf-8")


def exact_json_equal(actual: Any, expected: Any) -> bool:
    """Compare JSON values without Python's bool/int or int/float coercion."""
    return canonical_bytes(actual) == canonical_bytes(expected)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def effective_config_sha256(effective_values: dict[str, Any]) -> str:
    payload = json.dumps(
        effective_values,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def expected_launch_environment_binding(
    runs_root: Path, run_dir: Path
) -> dict[str, dict[str, str]]:
    """Return the sole canonical mutable-output contract for one r3 run."""
    root = Path(runs_root).resolve()
    run = Path(run_dir).resolve()
    environment_root = root / "launch_environment"
    return {
        "child_environment": {
            "HOME": str(environment_root / "home"),
            "ROS_HOME": str(environment_root / "ros_home"),
            "ROS_LOG_DIR": str(environment_root / "ros_logs"),
            "TMPDIR": str(environment_root / "tmp"),
            "XDG_RUNTIME_DIR": str(environment_root / "xdg_runtime"),
        },
        "mutable_output_paths": {
            "bag_output_dir": str(run / "bags"),
            "decision_csv_path": str(run / "p4_decisions.csv"),
            "export_root_dir": str(run / "exports"),
            "iap_log_root": str(run / "runtime" / "iap_logs"),
            "launch_command_path": str(run / "launch_command.json"),
            "run_manifest_path": str(run / "p4_g0c_run_manifest.json"),
            "runtime_root_dir": str(run / "runtime"),
            "stdout_log_path": str(run / "stdout.log"),
        },
    }


def validate_launch_environment_binding(
    child_environment: Any,
    mutable_output_paths: Any,
) -> dict[str, dict[str, str]]:
    """Validate exact r3 environment/output evidence without trusting hashes."""
    if (
        not isinstance(child_environment, dict)
        or set(child_environment) != set(LAUNCH_ENVIRONMENT_KEYS)
        or any(not isinstance(value, str) or not value for value in child_environment.values())
    ):
        raise ProtocolError("launch child environment is malformed")
    if (
        not isinstance(mutable_output_paths, dict)
        or set(mutable_output_paths) != set(MUTABLE_OUTPUT_KEYS)
        or any(not isinstance(value, str) or not value for value in mutable_output_paths.values())
    ):
        raise ProtocolError("launch mutable-output inventory is malformed")
    requested_paths = {
        **child_environment,
        **mutable_output_paths,
    }
    canonical_paths: dict[str, Path] = {}
    for key, value in requested_paths.items():
        path = Path(value)
        if not path.is_absolute() or ".." in path.parts:
            raise ProtocolError(f"launch mutable path is not canonical: {key}")
        resolved = path.resolve()
        if str(path) != str(resolved):
            raise ProtocolError(f"launch mutable path is aliased or symlinked: {key}")
        canonical_paths[key] = resolved
    if len(set(canonical_paths.values())) != len(canonical_paths):
        raise ProtocolError("launch mutable paths are duplicate")
    run_manifest = canonical_paths["run_manifest_path"]
    run_dir = run_manifest.parent
    runs_root = run_dir.parent
    expected = expected_launch_environment_binding(runs_root, run_dir)
    actual = {
        "child_environment": child_environment,
        "mutable_output_paths": mutable_output_paths,
    }
    if not exact_json_equal(actual, expected):
        raise ProtocolError("launch environment/output binding is not canonical")
    if runs_root not in run_dir.parents and run_dir.parent != runs_root:
        raise ProtocolError("launch run directory escapes runs root")
    return actual


def validate_test_planner_effective_contract(
    bundle: ProtocolBundle, launch_manifest: dict[str, Any]
) -> None:
    """Bind launch-emitted effective values to the reviewed protocol bundle."""
    schema = bundle.protocol.get("schema_version")
    if schema not in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}:
        return
    binding = launch_manifest.get("p4.g0c")
    if not isinstance(binding, dict):
        raise ProtocolError("test-planner G0C binding is missing")
    expected = {
        "schema_version": (
            "p4_g0c_run_manifest_v4"
            if schema == PROTOCOL_SCHEMA_V4
            else "p4_g0c_run_manifest_v3" if schema == PROTOCOL_SCHEMA_V3
            else "p4_g0c_run_manifest_v2"
        ),
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "effective_values": bundle.protocol["effective_values"],
        "effective_config_sha256": effective_config_sha256(
            bundle.protocol["effective_values"]
        ),
        "selection_applied": False,
        "record_bag": False,
        "start_rviz": False,
    }
    expected.update({
        "dependency_manifest_sha256": bundle.protocol[
            "runtime_dependency_manifest"
        ]["sha256"],
        "replacement_lineage_sha256": bundle.protocol[
            "replacement_lineage"
        ]["sha256"],
    })
    if schema in {PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}:
        expected.update({
            "child_environment": launch_manifest.get("p4.g0c", {}).get(
                "child_environment"
            ),
            "mutable_output_paths": launch_manifest.get("p4.g0c", {}).get(
                "mutable_output_paths"
            ),
        })
        validate_launch_environment_binding(
            binding.get("child_environment"),
            binding.get("mutable_output_paths"),
        )
    binding_effective = binding.get("effective_values")
    if (
        not isinstance(binding_effective, dict)
        or binding.get("effective_config_sha256")
        != effective_config_sha256(binding_effective)
    ):
        raise ProtocolError(
            "test-planner G0C binding effective hash mismatch"
        )
    for key, value in expected.items():
        if not exact_json_equal(binding.get(key), value):
            raise ProtocolError(
                f"test-planner G0C binding mismatch: {key}"
            )
    protocol_effective = bundle.protocol["effective_values"]
    for manifest_key, protocol_key in (
        TEST_PLANNER_TOP_LEVEL_EFFECTIVE_MAP.items()
    ):
        if manifest_key not in launch_manifest:
            raise ProtocolError(
                f"test-planner top-level effective missing: {manifest_key}"
            )
        if not exact_json_equal(
            launch_manifest[manifest_key], protocol_effective[protocol_key]
        ):
            raise ProtocolError(
                f"test-planner top-level effective mismatch: {manifest_key}"
            )


def validate_decision_header(fieldnames: list[str] | None) -> None:
    if tuple(fieldnames or ()) != DECISION_CSV_COLUMNS:
        raise DecisionSchemaError(
            "csv_header",
            "P4 decision CSV header is not the exact production schema",
        )


def _canonical_integer(row: dict[str, str], key: str) -> int:
    raw = row.get(key)
    if not isinstance(raw, str):
        raise DecisionSchemaError("typed_integer", f"{key} is missing")
    stripped = raw.strip()
    try:
        value = int(stripped)
    except ValueError as exc:
        raise DecisionSchemaError(
            "typed_integer", f"{key} is not an integer"
        ) from exc
    if str(value) != stripped:
        raise DecisionSchemaError(
            "typed_integer", f"{key} is not a canonical integer"
        )
    return value


def _finite_float(row: dict[str, str], key: str) -> float:
    raw = row.get(key)
    if not isinstance(raw, str):
        raise DecisionSchemaError("typed_non_finite", f"{key} is missing")
    try:
        value = float(raw)
    except ValueError as exc:
        raise DecisionSchemaError(
            "typed_non_finite", f"{key} is not numeric"
        ) from exc
    if not math.isfinite(value):
        raise DecisionSchemaError(
            "typed_non_finite", f"{key} is not finite"
        )
    return value


def parse_decision_row(
    row: dict[str, str], path_ratio_tolerance: float
) -> dict[str, Any]:
    if set(row) != set(DECISION_CSV_COLUMNS):
        raise DecisionSchemaError(
            "csv_header", "P4 decision row columns do not match the header"
        )
    producer_reason = str(row.get("reason", "")).strip()
    snapshot_failures = []
    if str(row.get("snapshot_generation_id", "")).strip() in {"", "0"}:
        snapshot_failures.append("generation_zero")
    try:
        snapshot_stamp = float(str(row.get("snapshot_stamp_s", "")))
        if not math.isfinite(snapshot_stamp):
            snapshot_failures.append("non_finite_stamp")
    except ValueError:
        snapshot_failures.append("non_finite_stamp")
    if not str(row.get("snapshot_frame", "")).strip():
        snapshot_failures.append("empty_frame")
    if producer_reason == "snapshot_unavailable" and snapshot_failures:
        raise DecisionSchemaError(
            "p0_riskgrid_snapshot",
            f"producer_reason={producer_reason}:" + ":".join(snapshot_failures),
        )
    strings: dict[str, str] = {}
    for key in DECISION_NONEMPTY_COLUMNS:
        value = row.get(key)
        if not isinstance(value, str) or not value.strip():
            raise DecisionSchemaError(
                "typed_identity", f"{key} must be nonempty"
            )
        strings[key] = value
    if strings["schema_version"] != DECISION_SCHEMA:
        raise DecisionSchemaError(
            "typed_schema", "decision schema version is not registered"
        )

    integers = {
        key: _canonical_integer(row, key) for key in DECISION_INTEGER_COLUMNS
    }
    for key in (
        "planning_attempt_id", "collision_segment_id",
        "snapshot_generation_id",
    ):
        if integers[key] <= 0:
            raise DecisionSchemaError(
                "typed_identity", f"{key} must be positive"
            )
    if integers["occupancy_epoch"] < 0:
        raise DecisionSchemaError(
            "typed_identity", "occupancy_epoch must be nonnegative"
        )
    if integers["selection_applied"] not in (0, 1):
        raise DecisionSchemaError(
            "typed_integer", "selection_applied must be 0 or 1"
        )
    for key in set(DECISION_INTEGER_COLUMNS) - {
        "planning_attempt_id", "collision_segment_id",
        "snapshot_generation_id", "occupancy_epoch", "selection_applied",
    }:
        if integers[key] < 0:
            raise DecisionSchemaError(
                "typed_integer", f"{key} must be nonnegative"
            )

    floats = {key: _finite_float(row, key) for key in DECISION_FLOAT_COLUMNS}
    for key in (
        "original_mean", "original_max", "risk_mean", "risk_max",
        "original_search_latency_ms", "risk_search_latency_ms",
        "total_search_latency_ms",
    ):
        if floats[key] < 0.0:
            raise DecisionSchemaError(
                "typed_non_finite", f"{key} must be nonnegative"
            )
    for key in ("original_path_length", "risk_path_length"):
        if floats[key] <= 0.0:
            raise DecisionSchemaError(
                "path_length", f"{key} must be positive"
            )
    if floats["path_length_ratio"] <= 0.0:
        raise DecisionSchemaError(
            "path_length", "path_length_ratio must be positive"
        )
    tolerance = float(path_ratio_tolerance)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise DecisionSchemaError(
            "path_ratio_consistency", "ratio tolerance is invalid"
        )
    recomputed = floats["risk_path_length"] / floats["original_path_length"]
    if abs(floats["path_length_ratio"] - recomputed) > tolerance:
        raise DecisionSchemaError(
            "path_ratio_consistency",
            "path_length_ratio is inconsistent with serialized path lengths",
        )
    return {"strings": strings, "integers": integers, "floats": floats}


def decision_identity(parsed_row: dict[str, Any]) -> tuple[Any, ...]:
    integers = parsed_row["integers"]
    strings = parsed_row["strings"]
    return tuple(
        integers[key] if key in integers else strings[key]
        for key in DECISION_IDENTITY_COLUMNS
    )


def _validate_artifact_relative_path(path: str) -> PurePosixPath:
    if not isinstance(path, str) or not path or "\\" in path:
        raise ProtocolError("artifact inventory path is not normalized")
    pure = PurePosixPath(path)
    if (
        pure.is_absolute()
        or pure.as_posix() != path
        or any(part in {"", ".", ".."} for part in pure.parts)
    ):
        raise ProtocolError(f"artifact inventory path is not normalized: {path}")
    return pure


def _validate_registered_artifact_directory(path: str) -> None:
    pure = _validate_artifact_relative_path(path)
    name = pure.name
    lowered = name.lower()
    if "retry" in lowered or name.startswith("p4-g0c-"):
        raise ProtocolError(f"unregistered nested run directory: {path}")


def _validate_registered_artifact_bytes(path: str, raw: bytes) -> None:
    pure = PurePosixPath(path)
    if pure.suffix.lower() == ".json" and path != "p4_g0c_run_manifest.json":
        try:
            payload = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            payload = None
        if (
            isinstance(payload, dict)
            and payload.get("schema_version") in {
                "p4_g0c_run_manifest_v1", "p4_g0c_run_manifest_v2"
            }
        ):
            raise ProtocolError(f"secondary G0C run manifest: {path}")
    if pure.suffix.lower() == ".csv" and path != "p4_decisions.csv":
        try:
            first_line = raw.decode("utf-8").splitlines()[0]
        except (UnicodeDecodeError, IndexError):
            first_line = ""
        if first_line == ",".join(DECISION_CSV_COLUMNS):
            raise ProtocolError(f"secondary P4 decision CSV: {path}")


def collect_run_artifact_entries(run_dir: Path) -> list[dict[str, Any]]:
    requested_root = Path(run_dir)
    if requested_root.is_symlink():
        raise ProtocolError(f"run directory cannot be a symlink: {requested_root}")
    root = requested_root.resolve()
    if not root.is_dir():
        raise ProtocolError(f"run directory is missing: {root}")
    entries: list[dict[str, Any]] = []

    def visit(directory: Path) -> None:
        try:
            children = sorted(directory.iterdir(), key=lambda item: item.name)
        except OSError as exc:
            raise ProtocolError(f"run artifact directory is unreadable: {directory}") from exc
        for child in children:
            relative = child.relative_to(root).as_posix()
            if child.is_symlink():
                raise ProtocolError(f"run artifact cannot be a symlink: {relative}")
            if relative == RUN_ARTIFACT_INVENTORY_FILENAME:
                if not child.is_file():
                    raise ProtocolError("artifact inventory path is not a regular file")
                continue
            if child.is_dir():
                _validate_registered_artifact_directory(relative)
                entries.append({"path": relative, "type": "directory"})
                visit(child)
            elif child.is_file():
                _validate_artifact_relative_path(relative)
                try:
                    raw = child.read_bytes()
                except OSError as exc:
                    raise ProtocolError(
                        f"run artifact is unreadable: {relative}"
                    ) from exc
                _validate_registered_artifact_bytes(relative, raw)
                entries.append({
                    "path": relative,
                    "type": "regular",
                    "size_bytes": len(raw),
                    "sha256": hashlib.sha256(raw).hexdigest(),
                })
            else:
                raise ProtocolError(f"unsupported run artifact type: {relative}")
    visit(root)
    return sorted(entries, key=lambda entry: entry["path"])


def make_run_artifact_inventory(run_dir: Path, run_id: str) -> dict[str, Any]:
    if not any(pattern.fullmatch(str(run_id)) for pattern in RUN_ID_PATTERNS):
        raise ProtocolError(f"artifact inventory run ID is invalid: {run_id}")
    return {
        "schema_version": RUN_ARTIFACT_INVENTORY_SCHEMA,
        "run_id": run_id,
        "excluded_path": RUN_ARTIFACT_INVENTORY_FILENAME,
        "entries": collect_run_artifact_entries(run_dir),
    }


def validate_run_artifact_inventory(
    inventory: dict[str, Any], run_dir: Path, run_id: str
) -> list[dict[str, Any]]:
    if not isinstance(inventory, dict) or set(inventory) != {
        "schema_version", "run_id", "excluded_path", "entries"
    }:
        raise ProtocolError("run artifact inventory root/schema is malformed")
    if inventory.get("schema_version") != RUN_ARTIFACT_INVENTORY_SCHEMA:
        raise ProtocolError("run artifact inventory version is not registered")
    if inventory.get("run_id") != run_id:
        raise ProtocolError("run artifact inventory ID is not bound")
    if inventory.get("excluded_path") != RUN_ARTIFACT_INVENTORY_FILENAME:
        raise ProtocolError("run artifact inventory self-exclusion is not exact")
    entries = inventory.get("entries")
    if not isinstance(entries, list):
        raise ProtocolError("run artifact inventory entries are missing")
    paths: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ProtocolError("run artifact inventory entry is not an object")
        path = entry.get("path")
        _validate_artifact_relative_path(path)
        if path == RUN_ARTIFACT_INVENTORY_FILENAME:
            raise ProtocolError("run artifact inventory includes itself")
        artifact_type = entry.get("type")
        if artifact_type == "directory":
            if set(entry) != {"path", "type"}:
                raise ProtocolError(f"directory inventory entry is malformed: {path}")
            _validate_registered_artifact_directory(path)
        elif artifact_type == "regular":
            if set(entry) != {"path", "type", "size_bytes", "sha256"}:
                raise ProtocolError(f"file inventory entry is malformed: {path}")
            size = entry.get("size_bytes")
            sha256 = entry.get("sha256")
            if not isinstance(size, int) or isinstance(size, bool) or size < 0:
                raise ProtocolError(f"file inventory size is invalid: {path}")
            if (
                not isinstance(sha256, str)
                or len(sha256) != 64
                or any(character not in "0123456789abcdef" for character in sha256)
            ):
                raise ProtocolError(f"file inventory SHA-256 is invalid: {path}")
        else:
            raise ProtocolError(f"artifact inventory type is invalid: {path}")
        paths.append(path)
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        raise ProtocolError("run artifact inventory paths are unordered or duplicate")
    actual_entries = collect_run_artifact_entries(run_dir)
    if entries != actual_entries:
        raise ProtocolError("run artifact inventory does not match the run tree")
    return actual_entries


def bound_test_planner_manifest_path(run_dir: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value:
        raise ProtocolError("test-planner manifest path is missing")
    candidate = Path(value)
    if not candidate.is_absolute():
        raise ProtocolError("test-planner manifest path is not absolute")
    resolved = candidate.resolve()
    if str(candidate) != str(resolved):
        raise ProtocolError("test-planner manifest path is aliased or symlinked")
    exports_root = (Path(run_dir).resolve() / "exports").resolve()
    if resolved.name != "test_planner_manifest.json":
        raise ProtocolError("test-planner manifest basename is not exact")
    if exports_root not in resolved.parents:
        raise ProtocolError("test-planner manifest path escapes exports")
    return resolved


def load_canonical_json(path: Path) -> dict[str, Any]:
    path = Path(path)
    try:
        raw = path.read_bytes()
        payload = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"unreadable JSON artifact {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ProtocolError(f"canonical JSON root must be an object: {path}")
    if raw != canonical_bytes(payload):
        raise ProtocolError(f"JSON artifact is not canonical: {path}")
    return payload


def registered_run_ids(protocol: dict[str, Any]) -> list[str]:
    seeds = protocol.get("seeds")
    repetitions = protocol.get("repetitions")
    schema = protocol.get("schema_version")
    matrix_matches = (
        exact_json_equal(seeds, [211, 223, 237, 253, 271])
        and exact_json_equal(repetitions, [1, 2, 3])
        if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}
        else seeds == [211, 223, 237, 253, 271]
        and repetitions == [1, 2, 3]
    )
    if not matrix_matches:
        raise ProtocolError("protocol seed/repetition matrix is not frozen")
    if schema == PROTOCOL_SCHEMA_V1:
        template = "p4-g0c-seed{seed}-rep{repetition:02d}"
    elif schema == PROTOCOL_SCHEMA_V2:
        template = "p4-g0c-r2-seed{seed}-rep{repetition:02d}"
    elif schema == PROTOCOL_SCHEMA_V3:
        template = "p4-g0c-r3-seed{seed}-rep{repetition:02d}"
    elif schema == PROTOCOL_SCHEMA_V4:
        template = "p4-g0c-r4-seed{seed}-rep{repetition:02d}"
    else:
        raise ProtocolError("unknown P4-G0C protocol schema")
    return [
        template.format(seed=seed, repetition=repetition)
        for seed in seeds
        for repetition in repetitions
    ]


def validate_protocol(protocol: dict[str, Any]) -> None:
    schema = protocol.get("schema_version")
    if schema not in PROTOCOL_SCHEMAS:
        raise ProtocolError("unknown P4-G0C protocol schema")
    expected_ids = registered_run_ids(protocol)
    if protocol.get("registered_run_ids") != expected_ids:
        raise ProtocolError("registered run IDs are not the exact immutable matrix")
    if protocol.get("matrix_order") != "seed_major_repetition_ascending":
        raise ProtocolError("matrix order is not frozen")
    if (
        not exact_json_equal(protocol.get("minimum_complete_decisions"), 100)
        if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}
        else protocol.get("minimum_complete_decisions") != 100
    ):
        raise ProtocolError("minimum complete-decision count is not 100")
    if any(protocol.get(key) is not True for key in ("no_overwrite", "no_exclusion", "no_retry")):
        raise ProtocolError("no-overwrite/no-exclusion/no-retry rules must be true")
    effective = protocol.get("effective_values")
    if not isinstance(effective, dict):
        raise ProtocolError("effective values are missing")
    required = {
        "gate": "G0C",
        "manager/p1_collision_fanout_clearance_m": 0.0,
        "manager/p1_collision_fanout_mirror_y": False,
        "manager/p1_collision_fanout_preserve_homotopies": False,
        "manager/use_distinctive_trajs": False,
        "p0.enable_risk_grid": True,
        "p0.enabled": True,
        "p1.debug_csv_enable": False,
        "p1.metrics_only": False,
        "p1.use_integrity_cost": False,
        "p2.debug_csv_enable": False,
        "p2.enable_candidate_ranking": False,
        "p2.metrics_only": False,
        "p3.debug_csv_enable": False,
        "p3.enable_global_reference_bias": False,
        "p3.enable_local_reference_bias": False,
        "p4.debug_csv_enable": True,
        "p4.enable_risk_aware_astar": True,
        "p4.enabled": True,
        "p4.metrics_only": True,
        "selection_applied": False,
        "p4.max_extra_path_ratio": 1.3,
        "p4.per_search_timeout_s": 0.2,
        "planner_enable_p1": False,
        "planner_enable_p2": False,
        "planner_enable_p3_global": False,
        "planner_enable_p3_local": False,
        "planner_enable_p4": True,
        "planner_enable_p5_final": False,
        "planner_enable_p5_runtime": False,
        "planner_safety_profile": "p4",
        "record_bag": False,
        "run_validator": True,
        "safety_viz.enable_p1_viz": False,
        "safety_viz.enable_p2_viz": False,
        "safety_viz.enable_p3_viz": False,
        "safety_viz.enable_p4_viz": False,
        "start_rviz": False,
    }
    if schema == PROTOCOL_SCHEMA_V4:
        required.update({
            "p0.predictor.sigma_grow_m_sqrt_s": 0.01,
            "p0.predictor.sigma_growth_profile": (
                "legacy_iap_rq320_baseline_v1"
            ),
        })
    if set(effective) != set(required):
        raise ProtocolError("effective protocol values must match the exact frozen set")
    for key, value in required.items():
        if (
            not exact_json_equal(effective.get(key), value)
            if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}
            else effective.get(key) != value
        ):
            raise ProtocolError(f"effective protocol value is not frozen: {key}")
    floor = protocol.get("numerical_noise_floor")
    quantiles = protocol.get("quantiles")
    if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}:
        if not exact_json_equal(floor, FROZEN_NUMERICAL_NOISE_FLOOR):
            raise ProtocolError(
                "scientific contract numerical-noise floor is not frozen"
            )
        if not exact_json_equal(quantiles, FROZEN_QUANTILES):
            raise ProtocolError("scientific contract quantiles are not frozen")
        if not exact_json_equal(
            protocol.get("threshold_formulas"), FROZEN_THRESHOLD_FORMULAS
        ):
            raise ProtocolError(
                "scientific contract threshold formulas are not frozen"
            )
    else:
        if not isinstance(floor, dict):
            raise ProtocolError("numerical-noise floor is missing")
        value = floor.get("value")
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ProtocolError(
                "numerical-noise floor must be finite and nonnegative"
            )
        if (
            floor.get("unit") != "risk_cost"
            or floor.get("calibration_mutable") is not False
        ):
            raise ProtocolError(
                "numerical-noise floor unit/mutability is invalid"
            )
        if (
            not isinstance(quantiles, dict)
            or quantiles.get("method") != "TYPE_7_LINEAR"
        ):
            raise ProtocolError("quantile method is not frozen")
        if quantiles.get("tie_behavior") != "stable_input_row_index":
            raise ProtocolError("quantile tie behavior is not frozen")
    ratio = protocol.get("path_ratio_consistency")
    expected_ratio = {
        "absolute_tolerance": 2e-5,
        "calibration_mutable": False,
        "derivation": {
            "eligible_ratio_cap": 1.3,
            "per_value_max_relative_rounding": 5e-6,
            "rounding": "round_up_to_2e-5",
            "serialization": (
                "std_ostream_defaultfloat_precision_6_significant_digits"
            ),
            "three_value_worst_case_absolute_bound": 1.95002e-5,
        },
        "unit": "dimensionless",
    }
    if (
        not exact_json_equal(ratio, expected_ratio)
        if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}
        else ratio != expected_ratio
    ):
        raise ProtocolError(
            "scientific contract path-ratio consistency is not frozen"
        )
    if schema in {PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4}:
        expected_template = (
            f"p4-g0c-r{schema[-1]}-seed{{seed}}-rep{{repetition:02d}}"
        )
        if protocol.get("run_id_template") != expected_template:
            raise ProtocolError("replacement run-ID template is not frozen")
        if not exact_json_equal(protocol.get("run_duration_s"), 90):
            raise ProtocolError("replacement run duration is not 90 seconds")
        version = f"v{schema[-1]}"
        for key, expected_schema in (
            ("runtime_dependency_manifest", f"p4_g0c_runtime_dependencies_{version}"),
            ("replacement_lineage", f"p4_g0c_replacement_lineage_{version}"),
        ):
            binding = protocol.get(key)
            if (
                not isinstance(binding, dict)
                or set(binding) != {"path", "schema_version", "sha256"}
                or binding.get("schema_version") != expected_schema
                or not _is_sha256(binding.get("sha256"))
            ):
                raise ProtocolError(f"replacement {key} binding is malformed")
    if schema == PROTOCOL_SCHEMA_V4:
        expected_binding = {
            "analyzer": {
                "path": "results/icra27/icra035/runs/benchmark/analyzer/gate0_analysis.json",
                "sha256": "5855368ddc0f89d69c8d13d3f9083b40371678177f2f6eaf3ce7fb68ee0dbaf3",
            },
            "config_preflight": {
                "path": "results/icra27/icra035/runs/p0_qualification_config_preflight.json",
                "sha256": "6d9ddcc0dd079a3a857a24cf61381441e4260498108077d3be795a8c6ea9b60b",
            },
            "profile": "legacy_iap_rq320_baseline_v1",
            "schema_version": "p0_gate0b_profile_binding_v1",
            "sigma_grow_m_sqrt_s": 0.01,
        }
        if not exact_json_equal(protocol.get("p0_profile_binding"), expected_binding):
            raise ProtocolError("v4 P0 Gate-0B profile binding is not exact")


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def validate_proposed_registry(registry: dict[str, Any]) -> None:
    if registry.get("schema_version") not in REGISTRY_SCHEMAS:
        raise ProtocolError("unknown P4 threshold registry schema")
    if registry.get("state") != PROPOSED_STATE:
        raise ProtocolError("registry must remain proposed and uncalibrated")
    if registry.get("application_enabled") is not False:
        raise ProtocolError("uncalibrated registry cannot enable application")
    if registry.get("calibration_bundle_sha256") is not None:
        raise ProtocolError("uncalibrated registry cannot bind calibration data")
    gates = registry.get("gates")
    if not isinstance(gates, dict) or set(gates) != set(GATE_NAMES):
        raise ProtocolError("registry must contain exactly four data-derived gates")
    if any(gates[name] is not None for name in GATE_NAMES):
        raise ProtocolError("uncalibrated registry gate values must be unset")


def validate_replacement_lineage(
    lineage: dict[str, Any], repository_root: Path
) -> None:
    if set(lineage) != {
        "schema_version", "superseded_protocol", "disqualified_execution",
        "replacement_namespace",
    }:
        raise ProtocolError("replacement lineage root is malformed")
    if lineage.get("schema_version") != "p4_g0c_replacement_lineage_v2":
        raise ProtocolError("replacement lineage schema is unknown")
    if lineage.get("replacement_namespace") != (
        "p4-g0c-r2-seed<seed>-rep<two-digits>"
    ):
        raise ProtocolError("replacement lineage namespace is not frozen")
    superseded = lineage.get("superseded_protocol")
    expected_superseded = {
        "path": "config/icra27/p4_g0c_protocol_v1.json",
        "sha256": "9e89ea42675459a63853d98845f02b7fe5b9434a9f28fcbd6ef5ba1bc5bd906d",
    }
    if superseded != expected_superseded:
        raise ProtocolError("replacement lineage superseded protocol mismatch")
    failed = lineage.get("disqualified_execution")
    if not isinstance(failed, dict):
        raise ProtocolError("replacement lineage execution is malformed")
    expected_scalars = {
        "task_id": "ICRA-046",
        "failed_run_id": "p4-g0c-seed211-rep01",
        "attempted_run_count": 1,
        "complete_run_count": 0,
        "retry_count": 0,
        "analyzer_invocations": 0,
        "failure_reason": "launch_exit_1:package_so3_control_not_found",
        "replacement_reason": (
            "PRELIVE_DEPENDENCY_GATE_VIOLATION_NO_CALIBRATION_DATA"
        ),
        "calibration_data_eligible": False,
        "threshold_draft_exists": False,
        "threshold_application_possible": False,
    }
    if any(failed.get(key) != value for key, value in expected_scalars.items()):
        raise ProtocolError("replacement lineage execution truth mismatch")
    expected_artifacts = {
        "raw_manifest": {
            "path": "results/icra27/icra046/preflight/raw_runs_manifest.tsv",
            "sha256": "f307e61a90707d6da5a38138558a97447c5267ef9a5184f3df92ca8b97079438",
        },
        "runner_state": {
            "path": "results/icra27/icra046/runs/p4_g0c_runner_state.json",
            "sha256": "a6dba6376b225f2fd00c218bdd19f911b9183e5e53a868f55cb0f1914d474ef1",
        },
    }
    for key, expected in expected_artifacts.items():
        if failed.get(key) != expected:
            raise ProtocolError(f"replacement lineage {key} mismatch")
    for artifact in [expected_superseded, *expected_artifacts.values()]:
        path = repository_root / artifact["path"]
        if sha256_file(path) != artifact["sha256"]:
            raise ProtocolError(
                f"replacement lineage retained artifact drift: {artifact['path']}"
            )


def validate_replacement_lineage_v3(
    lineage: dict[str, Any], repository_root: Path
) -> None:
    """Bind both consumed live matrices before registering disjoint r3 IDs."""
    if set(lineage) != {
        "schema_version", "superseded_protocols", "failed_live_executions",
        "replacement_namespace",
    }:
        raise ProtocolError("r3 replacement lineage root is malformed")
    if lineage.get("schema_version") != "p4_g0c_replacement_lineage_v3":
        raise ProtocolError("r3 replacement lineage schema is unknown")
    if lineage.get("replacement_namespace") != (
        "p4-g0c-r3-seed<seed>-rep<two-digits>"
    ):
        raise ProtocolError("r3 replacement lineage namespace is not frozen")
    expected_protocols = [
        {
            "path": "config/icra27/p4_g0c_protocol_v1.json",
            "sha256": "9e89ea42675459a63853d98845f02b7fe5b9434a9f28fcbd6ef5ba1bc5bd906d",
        },
        {
            "path": "config/icra27/p4_g0c_protocol_v2.json",
            "sha256": "8b0b2c3ed531680c6c8268738cb1bcb9136f39d2b97e68769e54a53afe59de79",
        },
    ]
    if not exact_json_equal(lineage.get("superseded_protocols"), expected_protocols):
        raise ProtocolError("r3 superseded protocol lineage mismatch")
    expected_executions = [
        {
            "analyzer_invocations": 0,
            "attempted_run_count": 1,
            "calibration_data_eligible": False,
            "complete_run_count": 0,
            "failed_run_id": "p4-g0c-seed211-rep01",
            "failure_classification": "MISSING_PRELIVE_DEPENDENCY_GATE",
            "failure_reason": "launch_exit_1:package_so3_control_not_found",
            "retry_count": 0,
            "runner_state": {
                "path": "results/icra27/icra046/runs/p4_g0c_runner_state.json",
                "sha256": "a6dba6376b225f2fd00c218bdd19f911b9183e5e53a868f55cb0f1914d474ef1",
            },
            "task_id": "ICRA-046",
            "threshold_application_possible": False,
            "threshold_draft_exists": False,
        },
        {
            "analyzer_invocations": 0,
            "attempted_run_count": 1,
            "calibration_data_eligible": False,
            "complete_run_count": 0,
            "external_log": {
                "path": "/root/.ros/log/2026-08-24-14-34-21-049171-mint-X-965267/launch.log",
                "sha256": "f506e5565d73ad601673c814635797c360f650c7be3c4356e9217449df2458e7",
            },
            "failed_run_id": "p4-g0c-r2-seed211-rep01",
            "failure_classification": (
                "SELF_INDUCED_NON_REPOSITORY_LOCAL_ROS_LOG_ENVIRONMENT"
            ),
            "failure_reason": "launch_exit_1",
            "retry_count": 0,
            "runner_state": {
                "path": "results/icra27/icra051/runs/p4_g0c_runner_state.json",
                "sha256": "7c3cafc505ad33e7e8631a2ed1534bf5e21c6cf4f4d9eb252319a250989846a7",
            },
            "task_id": "ICRA-051",
            "threshold_application_possible": False,
            "threshold_draft_exists": False,
        },
    ]
    if not exact_json_equal(lineage.get("failed_live_executions"), expected_executions):
        raise ProtocolError("r3 failed live-execution lineage mismatch")
    for artifact in expected_protocols:
        path = repository_root / artifact["path"]
        if sha256_file(path) != artifact["sha256"]:
            raise ProtocolError(f"r3 retained protocol drift: {artifact['path']}")
    for execution in expected_executions:
        state = execution["runner_state"]
        if sha256_file(repository_root / state["path"]) != state["sha256"]:
            raise ProtocolError(f"r3 retained runner-state drift: {state['path']}")
        external = execution.get("external_log")
        if external and sha256_file(Path(external["path"])) != external["sha256"]:
            raise ProtocolError("r3 retained external launch-log drift")


def validate_replacement_lineage_v4(
    lineage: dict[str, Any], repository_root: Path
) -> None:
    """Bind the consumed r3 failure and exclude it from the r4 matrix."""
    if lineage.get("schema_version") != "p4_g0c_replacement_lineage_v4":
        raise ProtocolError("r4 replacement lineage schema is unknown")
    if lineage.get("replacement_namespace") != (
        "p4-g0c-r4-seed<seed>-rep<two-digits>"
    ) or lineage.get("excluded_namespaces") != [
        "p4-g0c-r3-seed<seed>-rep<two-digits>"
    ]:
        raise ProtocolError("r4 replacement lineage namespace isolation failed")
    execution = lineage.get("consumed_execution", {})
    expected = {
        "failed_run_id": "p4-g0c-r3-seed211-rep01",
        "failure_classification": "P0_RISKGRID_SNAPSHOT_UNAVAILABLE",
        "typed_verdict": (
            "snapshot_unavailable:generation_zero:non_finite_stamp:empty_frame"
        ),
        "root_cause_verdict": "P0_PROFILE_BINDING_OMITTED",
    }
    if any(execution.get(key) != value for key, value in expected.items()):
        raise ProtocolError("r4 consumed execution verdict mismatch")
    artifacts = list(lineage.get("superseded_artifacts", [])) + [
        execution.get("runner_state", {}), execution.get("compact_results", {})
    ]
    for artifact in artifacts:
        if not isinstance(artifact, dict) or not _is_sha256(artifact.get("sha256")):
            raise ProtocolError("r4 retained artifact binding is malformed")
        if sha256_file(repository_root / artifact["path"]) != artifact["sha256"]:
            raise ProtocolError(f"r4 retained artifact drift: {artifact['path']}")


def load_protocol_bundle(
    protocol_path: Path,
    registry_path: Path,
    fixture_path: Path,
    expected_protocol_schema: str = PROTOCOL_SCHEMA_V2,
) -> ProtocolBundle:
    protocol_path = Path(protocol_path)
    registry_path = Path(registry_path)
    fixture_path = Path(fixture_path)
    protocol = load_canonical_json(protocol_path)
    registry = load_canonical_json(registry_path)
    fixture = load_canonical_json(fixture_path)
    protocol_sha = sha256_file(protocol_path)
    registry_sha = sha256_file(registry_path)
    fixture_sha = sha256_file(fixture_path)
    if expected_protocol_schema not in PROTOCOL_SCHEMAS:
        raise ProtocolError("trusted protocol mode is invalid")
    if protocol.get("schema_version") != expected_protocol_schema:
        raise ProtocolError("protocol schema does not match trusted mode")
    if expected_protocol_schema == PROTOCOL_SCHEMA_V2 and (
        protocol_sha != P4_G0C_PROTOCOL_V2_TRUSTED_SHA256
        or registry_sha != P4_G0C_REGISTRY_V2_TRUSTED_SHA256
    ):
        raise ProtocolError("v2 trust anchor protocol/registry hash mismatch")
    if expected_protocol_schema == PROTOCOL_SCHEMA_V3 and (
        protocol_sha != P4_G0C_PROTOCOL_V3_TRUSTED_SHA256
        or registry_sha != P4_G0C_REGISTRY_V3_TRUSTED_SHA256
    ):
        raise ProtocolError("v3 trust anchor protocol/registry hash mismatch")
    if expected_protocol_schema == PROTOCOL_SCHEMA_V4 and (
        protocol_sha != P4_G0C_PROTOCOL_V4_TRUSTED_SHA256
        or registry_sha != P4_G0C_REGISTRY_V4_TRUSTED_SHA256
    ):
        raise ProtocolError("v4 trust anchor protocol/registry hash mismatch")
    validate_protocol(protocol)
    validate_proposed_registry(registry)
    expected_registry_schema = {
        PROTOCOL_SCHEMA_V1: REGISTRY_SCHEMA_V1,
        PROTOCOL_SCHEMA_V2: REGISTRY_SCHEMA_V2,
        PROTOCOL_SCHEMA_V3: REGISTRY_SCHEMA_V3,
        PROTOCOL_SCHEMA_V4: REGISTRY_SCHEMA_V4,
    }[expected_protocol_schema]
    if registry.get("schema_version") != expected_registry_schema:
        raise ProtocolError("protocol and registry versions do not match")
    if fixture.get("schema_version") != FIXTURE_SCHEMA:
        raise ProtocolError("unknown P4-G0C fixture schema")
    if registry.get("protocol_sha256") != protocol_sha:
        raise ProtocolError("registry protocol hash mismatch")
    live_fixture = protocol.get("live_fixture")
    if not isinstance(live_fixture, dict) or live_fixture.get("sha256") != fixture_sha:
        raise ProtocolError("protocol fixture hash mismatch")
    if registry.get("numerical_noise_floor") != protocol.get("numerical_noise_floor"):
        raise ProtocolError("registry numerical-noise floor mismatch")
    if expected_protocol_schema in {
        PROTOCOL_SCHEMA_V2, PROTOCOL_SCHEMA_V3, PROTOCOL_SCHEMA_V4
    }:
        repository_root = protocol_path.resolve().parents[2]
        for key in ("runtime_dependency_manifest", "replacement_lineage"):
            binding = protocol[key]
            bound_path = repository_root / binding["path"]
            payload = load_canonical_json(bound_path)
            if sha256_file(bound_path) != binding["sha256"]:
                raise ProtocolError(f"replacement {key} hash mismatch")
            if payload.get("schema_version") != binding["schema_version"]:
                raise ProtocolError(f"replacement {key} schema mismatch")
            if key == "replacement_lineage":
                if expected_protocol_schema == PROTOCOL_SCHEMA_V4:
                    validate_replacement_lineage_v4(payload, repository_root)
                elif expected_protocol_schema == PROTOCOL_SCHEMA_V3:
                    validate_replacement_lineage_v3(payload, repository_root)
                else:
                    validate_replacement_lineage(payload, repository_root)
    return ProtocolBundle(
        protocol, registry, fixture, protocol_sha, registry_sha, fixture_sha
    )


def expand_run_plan(protocol: dict[str, Any], runs_root: Path) -> list[dict[str, Any]]:
    validate_protocol(protocol)
    root = Path(runs_root)
    result = []
    for seed in protocol["seeds"]:
        for repetition in protocol["repetitions"]:
            if protocol["schema_version"] == PROTOCOL_SCHEMA_V4:
                run_id = f"p4-g0c-r4-seed{seed}-rep{repetition:02d}"
            elif protocol["schema_version"] == PROTOCOL_SCHEMA_V3:
                run_id = f"p4-g0c-r3-seed{seed}-rep{repetition:02d}"
            elif protocol["schema_version"] == PROTOCOL_SCHEMA_V2:
                run_id = f"p4-g0c-r2-seed{seed}-rep{repetition:02d}"
            else:
                run_id = f"p4-g0c-seed{seed}-rep{repetition:02d}"
            result.append({
                "run_id": run_id,
                "seed": seed,
                "repetition": repetition,
                "run_dir": str(root / run_id),
            })
    return result


def parse_run_id(run_id: str) -> tuple[int, int]:
    for pattern in RUN_ID_PATTERNS:
        match = pattern.fullmatch(str(run_id))
        if match:
            return int(match.group(1)), int(match.group(2))
    raise ProtocolError(f"invalid immutable run ID: {run_id}")
