#!/usr/bin/env python3
"""Immutable P4-G0C protocol loading, hashing, and run registration."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


PROTOCOL_SCHEMA = "p4_g0c_protocol_v1"
REGISTRY_SCHEMA = "p4_threshold_registry_v1"
FIXTURE_SCHEMA = "p4_g0c_fixture_v1"
PROPOSED_STATE = "PROPOSED_UNCALIBRATED"
GATE_NAMES = (
    "mean_improvement_min",
    "max_improvement_min",
    "path_ratio_max",
    "total_search_timeout_s",
)
RUN_ID_PATTERN = re.compile(r"^p4-g0c-seed([0-9]+)-rep([0-9]{2})$")


class ProtocolError(RuntimeError):
    """A versioned G0C artifact violates the frozen pre-data contract."""


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
    if seeds != [211, 223, 237, 253, 271] or repetitions != [1, 2, 3]:
        raise ProtocolError("protocol seed/repetition matrix is not frozen")
    return [
        f"p4-g0c-seed{seed}-rep{repetition:02d}"
        for seed in seeds
        for repetition in repetitions
    ]


def validate_protocol(protocol: dict[str, Any]) -> None:
    if protocol.get("schema_version") != PROTOCOL_SCHEMA:
        raise ProtocolError("unknown P4-G0C protocol schema")
    expected_ids = registered_run_ids(protocol)
    if protocol.get("registered_run_ids") != expected_ids:
        raise ProtocolError("registered run IDs are not the exact immutable matrix")
    if protocol.get("matrix_order") != "seed_major_repetition_ascending":
        raise ProtocolError("matrix order is not frozen")
    if protocol.get("minimum_complete_decisions") != 100:
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
    if set(effective) != set(required):
        raise ProtocolError("effective protocol values must match the exact frozen set")
    for key, value in required.items():
        if effective.get(key) != value:
            raise ProtocolError(f"effective protocol value is not frozen: {key}")
    floor = protocol.get("numerical_noise_floor")
    if not isinstance(floor, dict):
        raise ProtocolError("numerical-noise floor is missing")
    value = floor.get("value")
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ProtocolError("numerical-noise floor must be finite and nonnegative")
    if floor.get("unit") != "risk_cost" or floor.get("calibration_mutable") is not False:
        raise ProtocolError("numerical-noise floor unit/mutability is invalid")
    quantiles = protocol.get("quantiles")
    if not isinstance(quantiles, dict) or quantiles.get("method") != "TYPE_7_LINEAR":
        raise ProtocolError("quantile method is not frozen")
    if quantiles.get("tie_behavior") != "stable_input_row_index":
        raise ProtocolError("quantile tie behavior is not frozen")


def validate_proposed_registry(registry: dict[str, Any]) -> None:
    if registry.get("schema_version") != REGISTRY_SCHEMA:
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


def load_protocol_bundle(
    protocol_path: Path, registry_path: Path, fixture_path: Path
) -> ProtocolBundle:
    protocol_path = Path(protocol_path)
    registry_path = Path(registry_path)
    fixture_path = Path(fixture_path)
    protocol = load_canonical_json(protocol_path)
    registry = load_canonical_json(registry_path)
    fixture = load_canonical_json(fixture_path)
    validate_protocol(protocol)
    validate_proposed_registry(registry)
    if fixture.get("schema_version") != FIXTURE_SCHEMA:
        raise ProtocolError("unknown P4-G0C fixture schema")
    protocol_sha = sha256_file(protocol_path)
    registry_sha = sha256_file(registry_path)
    fixture_sha = sha256_file(fixture_path)
    if registry.get("protocol_sha256") != protocol_sha:
        raise ProtocolError("registry protocol hash mismatch")
    live_fixture = protocol.get("live_fixture")
    if not isinstance(live_fixture, dict) or live_fixture.get("sha256") != fixture_sha:
        raise ProtocolError("protocol fixture hash mismatch")
    if registry.get("numerical_noise_floor") != protocol.get("numerical_noise_floor"):
        raise ProtocolError("registry numerical-noise floor mismatch")
    return ProtocolBundle(
        protocol, registry, fixture, protocol_sha, registry_sha, fixture_sha
    )


def expand_run_plan(protocol: dict[str, Any], runs_root: Path) -> list[dict[str, Any]]:
    validate_protocol(protocol)
    root = Path(runs_root)
    result = []
    for seed in protocol["seeds"]:
        for repetition in protocol["repetitions"]:
            run_id = f"p4-g0c-seed{seed}-rep{repetition:02d}"
            result.append({
                "run_id": run_id,
                "seed": seed,
                "repetition": repetition,
                "run_dir": str(root / run_id),
            })
    return result


def parse_run_id(run_id: str) -> tuple[int, int]:
    match = RUN_ID_PATTERN.fullmatch(str(run_id))
    if not match:
        raise ProtocolError(f"invalid immutable run ID: {run_id}")
    return int(match.group(1)), int(match.group(2))
