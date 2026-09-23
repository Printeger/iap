#!/usr/bin/env python3
"""Qualify a versioned P4 control-capability profile from controller traces."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


SPEEDS = (0.5, 1.0, 1.5, 2.0)
ACCELERATIONS = (0.75, 1.5, 2.25, 3.0)
JERKS = (1.0, 2.0, 3.0, 4.0)
MANEUVERS = (
    "stationary_start", "nonzero_start", "left_turn", "right_turn",
    "s_turn", "successor_switch", "stop")
PARTITIONS = ("training", "held_out")
TRACKING_ENVELOPE_M = 0.15
QUALIFICATION_FACTOR = 1.2


def _norm(values: list[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def _difference(left: list[float], right: list[float]) -> list[float]:
    return [float(a) - float(b) for a, b in zip(left, right)]


def _records(path: Path) -> list[dict[str, Any]]:
    output = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                row = json.loads(line)
                if row.get("event", row.get("kind")) == "controller_trace":
                    output.append(row.get("payload", row))
    return output


def _evaluate_case(case: dict[str, Any], manifest_dir: Path) -> dict[str, Any]:
    capture = Path(case["capture_path"])
    if not capture.is_absolute():
        capture = manifest_dir / capture
    rows = _records(capture)
    if not rows:
        raise ValueError(f"controller trace missing: {capture}")
    errors = [_norm(_difference(row["position_xyz"],
                                row["feedback_position_xyz"]))
              for row in rows]
    velocity_errors = [
        _norm(_difference(row["velocity_xyz"],
                          row["feedback_velocity_xyz"])) for row in rows]
    switches = []
    for previous, current in zip(rows, rows[1:]):
        previous_identity = (
            previous["execution_instance_id"], previous["trajectory_id"],
            previous["start_time_ns"], previous["curve_hash"])
        current_identity = (
            current["execution_instance_id"], current["trajectory_id"],
            current["start_time_ns"], current["curve_hash"])
        if previous_identity != current_identity:
            switches.append({
                "position_jump_m": _norm(_difference(
                    previous["position_xyz"], current["position_xyz"])),
                "velocity_jump_mps": _norm(_difference(
                    previous["velocity_xyz"], current["velocity_xyz"])),
                "acceleration_jump_mps2": _norm(_difference(
                    previous["acceleration_xyz"],
                    current["acceleration_xyz"])),
            })
    max_error = max(errors)
    switch_continuous = all(
        item["position_jump_m"] <= TRACKING_ENVELOPE_M and
        item["velocity_jump_mps"] <= 0.25 and
        item["acceleration_jump_mps2"] <= 1.0 for item in switches)
    return {
        **{key: case[key] for key in (
            "speed_mps", "acceleration_mps2", "jerk_mps3", "maneuver",
            "partition")},
        "capture_path": str(capture),
        "sample_count": len(rows),
        "maximum_tracking_error_m": max_error,
        "maximum_velocity_error_mps": max(velocity_errors),
        "saturated": any(bool(row["saturated"]) for row in rows),
        "switch_count": len(switches),
        "switch_continuous": switch_continuous,
        "passed": max_error * QUALIFICATION_FACTOR <= TRACKING_ENVELOPE_M and
                  not any(bool(row["saturated"]) for row in rows) and
                  switch_continuous,
    }


def qualify(manifest: dict[str, Any], manifest_dir: Path,
            require_complete: bool = True) -> dict[str, Any]:
    cases = [_evaluate_case(case, manifest_dir)
             for case in manifest.get("cases", [])]
    expected = {
        (speed, acceleration, jerk, maneuver, partition)
        for speed in SPEEDS for acceleration in ACCELERATIONS
        for jerk in JERKS for maneuver in MANEUVERS
        for partition in PARTITIONS
    }
    present = {
        (float(case["speed_mps"]), float(case["acceleration_mps2"]),
         float(case["jerk_mps3"]), case["maneuver"], case["partition"])
        for case in cases
    }
    missing = sorted(expected - present)
    if require_complete and missing:
        raise ValueError(f"control scan incomplete: {len(missing)} cases missing")
    grouped: dict[tuple[float, float, float], list[dict[str, Any]]] = {}
    for case in cases:
        key = (float(case["speed_mps"]),
               float(case["acceleration_mps2"]),
               float(case["jerk_mps3"]))
        grouped.setdefault(key, []).append(case)
    eligible = []
    for key, rows in grouped.items():
        required_rows = len(MANEUVERS) * len(PARTITIONS)
        if len(rows) == required_rows and all(row["passed"] for row in rows):
            eligible.append((key, rows))
    if not eligible:
        raise ValueError("no control configuration passed training and held-out cases")
    (speed, acceleration, jerk), selected_rows = max(
        eligible, key=lambda item: item[0])
    maximum_error = max(row["maximum_tracking_error_m"]
                        for row in selected_rows)
    maximum_velocity_error = max(row["maximum_velocity_error_mps"]
                                 for row in selected_rows)
    return {
        "schema_version": "p4_control_capability_v1",
        "qualification_status": "qualified",
        "maximum_velocity_mps": [speed] * 3,
        "maximum_acceleration_mps2": [acceleration] * 3,
        "maximum_jerk_mps3": [jerk] * 3,
        "measured_latency_bound_s": float(
            manifest["measured_latency_bound_s"]),
        "position_tracking_bound_m": [maximum_error] * 3,
        "velocity_tracking_bound_mps": [maximum_velocity_error] * 3,
        "controller_identity": manifest["controller_identity"],
        "simulator_identity": manifest["simulator_identity"],
        "code_version": manifest["code_version"],
        "qualification_factor": QUALIFICATION_FACTOR,
        "tracking_envelope_m": TRACKING_ENVELOPE_M,
        "case_count": len(cases),
        "missing_case_count": len(missing),
        "cases": cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-incomplete", action="store_true")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    result = qualify(manifest, args.manifest.parent,
                     require_complete=not args.allow_incomplete)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
