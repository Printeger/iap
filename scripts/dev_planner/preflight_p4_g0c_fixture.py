#!/usr/bin/env python3
"""No-ROS eligibility preflight for the versioned P4-G0C r5 fixture."""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any


RESULT_SCHEMA = "p4_g0c_fixture_eligibility_preflight_v1"
EXPERIMENT = "p4_g0c_metrics_calibration_v5"
SCANNER_FILTER = (
    "P4G0CExactGeometryEligibility."
    "R5FixtureProducesOneClosedSegmentWithFreeEndpointsAndTail:"
    "P4G0CExactGeometryEligibility."
    "SupersededR4FixtureRemainsOpenEndedCollision"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    raw = path.read_bytes()
    payload = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("JSON_ROOT_NOT_OBJECT")
    canonical = (
        json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    if raw != canonical:
        raise ValueError("JSON_NOT_CANONICAL")
    return payload


def _assignments(tree: ast.Module) -> dict[str, ast.AST]:
    result: dict[str, ast.AST] = {}
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and len(statement.targets) == 1:
            target = statement.targets[0]
            if isinstance(target, ast.Name):
                result[target.id] = statement.value
    return result


def _literal(node: ast.AST, names: dict[str, Any]) -> Any:
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, (ast.Tuple, ast.List)):
        values = [_literal(item, names) for item in node.elts]
        return tuple(values) if isinstance(node, ast.Tuple) else values
    if isinstance(node, ast.Name) and node.id in names:
        return names[node.id]
    if isinstance(node, ast.Subscript):
        return _literal(node.value, names)[_literal(node.slice, names)]
    if (
        isinstance(node, ast.UnaryOp)
        and isinstance(node.op, ast.USub)
        and isinstance(node.operand, ast.Constant)
    ):
        return -node.operand.value
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "str"
        and len(node.args) == 1
    ):
        return str(_literal(node.args[0], names))
    raise ValueError("LAUNCH_VALUE_NOT_STATIC")


def _launch_geometry(launch_path: Path) -> dict[str, Any]:
    tree = ast.parse(launch_path.read_text(encoding="utf-8"), str(launch_path))
    assignments = _assignments(tree)
    obstacle = _literal(
        assignments["P4_G0C_V5_CENTRAL_OBSTACLE_X_M"], {}
    )
    defaults = dict(_literal(assignments["ARG_DEFAULTS"], {}))
    fork_node = assignments["P1_FORK_MAP_PRESET"]
    if not isinstance(fork_node, ast.Dict):
        raise ValueError("LAUNCH_FORK_PRESET_NOT_STATIC")
    fork_map = {
        str(_literal(key, {})): _literal(value, {})
        for key, value in zip(fork_node.keys, fork_node.values)
        if key is not None
    }
    scenario_presets = assignments["SCENARIO_PRESETS"]
    if not isinstance(scenario_presets, ast.Dict):
        raise ValueError("LAUNCH_SCENARIOS_NOT_STATIC")
    scenario_node = None
    for key, value in zip(scenario_presets.keys, scenario_presets.values):
        if isinstance(key, ast.Constant) and key.value == "p4_g0c_free_corridor_v1":
            scenario_node = value
            break
    if not isinstance(scenario_node, ast.Dict):
        raise ValueError("LAUNCH_V5_SCENARIO_MISSING")
    scenario = {
        str(_literal(key, {})): _literal(value, {})
        for key, value in zip(scenario_node.keys, scenario_node.values)
        if key is not None
    }
    presets = assignments["EXPERIMENT_PRESETS"]
    if not isinstance(presets, ast.Dict):
        raise ValueError("LAUNCH_PRESETS_NOT_STATIC")
    experiment_node = None
    for key, value in zip(presets.keys, presets.values):
        if isinstance(key, ast.Constant) and key.value == EXPERIMENT:
            experiment_node = value
            break
    if not isinstance(experiment_node, ast.Dict):
        raise ValueError("LAUNCH_V5_PRESET_MISSING")
    names = {"P4_G0C_V5_CENTRAL_OBSTACLE_X_M": obstacle}
    geometry_overrides: dict[str, Any] = {}
    for key, value in zip(experiment_node.keys, experiment_node.values):
        if isinstance(key, ast.Constant) and key.value in {
            "p1_fixture_central_obstacle_enabled",
            "p1_fixture_central_x_min_m",
            "p1_fixture_central_x_max_m",
            "p1_fixture_central_y_half_width_m",
            "p1_fixture_central_z_max_m",
        }:
            geometry_overrides[str(key.value)] = _literal(value, names)
    effective = {**defaults, **fork_map, **scenario, **geometry_overrides}
    required = {
        "p1_fixture_central_obstacle_enabled",
        "p1_fixture_central_x_min_m",
        "p1_fixture_central_x_max_m",
        "p1_fixture_central_y_half_width_m",
        "p1_fixture_central_z_max_m",
    }
    if not required.issubset(effective):
        raise ValueError("LAUNCH_V5_GEOMETRY_OVERRIDE_MISSING")
    enabled = str(effective["p1_fixture_central_obstacle_enabled"]).lower()
    if enabled not in {"true", "false"}:
        raise ValueError("LAUNCH_V5_GEOMETRY_ENABLED_NOT_BOOLEAN")
    return {
        "start_x_m": float(defaults["init_x"]),
        "horizon_m": float(defaults["manager/planning_horizon"]),
        "control_point_distance_m": float(
            defaults["manager/control_points_distance"]
        ),
        "obstacle_x_m": [
            float(effective["p1_fixture_central_x_min_m"]),
            float(effective["p1_fixture_central_x_max_m"]),
        ],
        "obstacle_enabled": enabled == "true",
        "obstacle_y_half_width_m": float(
            effective["p1_fixture_central_y_half_width_m"]
        ),
        "obstacle_z_m": [
            0.0, float(effective["p1_fixture_central_z_max_m"])
        ],
    }


def _failure(reason: str, **evidence: Any) -> dict[str, Any]:
    return {
        "schema_version": RESULT_SCHEMA,
        "typed_result": "FIXTURE_ELIGIBILITY_FAIL",
        "eligible": False,
        "failure_reason": reason,
        **evidence,
    }


def run_preflight(
    fixture_path: Path,
    protocol_path: Path,
    launch_path: Path,
    scanner_test_executable: Path,
) -> dict[str, Any]:
    fixture_path = Path(fixture_path).resolve()
    protocol_path = Path(protocol_path).resolve()
    launch_path = Path(launch_path).resolve()
    scanner_test_executable = Path(scanner_test_executable).resolve()
    try:
        fixture = _load_json(fixture_path)
        protocol = _load_json(protocol_path)
        launch_geometry = _launch_geometry(launch_path)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, KeyError, ValueError) as exc:
        return _failure(f"MATERIALIZATION_FAILED:{exc}")
    expected_obstacle = {
        "x_m": [-9.0, -7.0],
        "y_half_width_m": 0.65,
        "z_m": [0.0, 2.8],
    }
    if fixture.get("schema_version") != "p4_g0c_fixture_v2" or (
        fixture.get("map", {}).get("central_obstacle") != expected_obstacle
    ):
        return _failure("SOURCE_FIXTURE_GEOMETRY_MISMATCH")
    fixture_binding = protocol.get("live_fixture", {})
    if protocol.get("schema_version") != "p4_g0c_protocol_v5":
        return _failure("PROTOCOL_SCHEMA_MISMATCH")
    if fixture_binding.get("path") != (
        "config/icra27/p4_g0c_live_fixture_v2.json"
    ):
        return _failure("PROTOCOL_FIXTURE_PATH_MISMATCH")
    fixture_sha = _sha256(fixture_path)
    if fixture_binding.get("sha256") != fixture_sha:
        return _failure("PROTOCOL_FIXTURE_HASH_MISMATCH")
    expected_geometry = {
        "start_x_m": -12.0,
        "horizon_m": 7.5,
        "control_point_distance_m": 0.4,
        "obstacle_x_m": [-9.0, -7.0],
        "obstacle_enabled": True,
        "obstacle_y_half_width_m": 0.65,
        "obstacle_z_m": [0.0, 2.8],
    }
    if launch_geometry != expected_geometry:
        return _failure(
            "EFFECTIVE_LAUNCH_GEOMETRY_MISMATCH",
            materialized_geometry=launch_geometry,
        )
    command = [
        str(scanner_test_executable), f"--gtest_filter={SCANNER_FILTER}"
    ]
    try:
        completed = subprocess.run(
            command, check=False, text=True, capture_output=True, timeout=30.0
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return _failure(f"PRODUCTION_SCANNER_EXECUTION_FAILED:{type(exc).__name__}")
    scanner_evidence = {
        "argv": command,
        "exit_code": completed.returncode,
        "stdout": completed.stdout[-8000:],
        "stderr": completed.stderr[-8000:],
        "r5_status": "CLOSED_SEGMENTS",
        "r4_status": "OPEN_ENDED_COLLISION",
        "production_entrypoint": "BsplineOptimizer::scanCollisionSegments",
    }
    if completed.returncode != 0:
        return _failure(
            "PRODUCTION_SCANNER_CONTRACT_FAILED",
            materialized_geometry=launch_geometry,
            scanner_contract=scanner_evidence,
        )
    return {
        "schema_version": RESULT_SCHEMA,
        "typed_result": "FIXTURE_ELIGIBILITY_PASS",
        "eligible": True,
        "failure_reason": "",
        "source_fixture": {"path": str(fixture_path), "sha256": fixture_sha},
        "protocol": {"path": str(protocol_path), "sha256": _sha256(protocol_path)},
        "installed_launch": {"path": str(launch_path), "sha256": _sha256(launch_path)},
        "materialized_geometry": launch_geometry,
        "scanner_contract": scanner_evidence,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", required=True, type=Path)
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--installed-launch", required=True, type=Path)
    parser.add_argument("--scanner-test-executable", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = run_preflight(
        args.fixture, args.protocol, args.installed_launch,
        args.scanner_test_executable,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, sort_keys=True, separators=(",", ":")) + "\n"
    )
    return 0 if result["eligible"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
