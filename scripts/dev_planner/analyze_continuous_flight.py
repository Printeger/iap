#!/usr/bin/env python3
"""Correlate continuous-flight command, control, odometry and channel evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


TIMELINE_FIELDS = (
    "kind", "receive_steady_s", "execution_instance_id", "trajectory_id",
    "start_time_ns", "curve_hash", "state", "event_time_ns",
    "parent_trajectory_id", "parent_start_time_ns", "parent_curve_hash",
)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _legacy_curve_hash(payload: dict[str, Any]) -> str:
    existing = str(payload.get("curve_hash", "")).strip()
    if existing:
        return existing
    physical = {
        "order": payload.get("order", 3),
        "start_time_ns": payload.get("start_time_ns", 0),
        "knots": payload.get("knots", []),
        "control_points_xyz": payload.get("control_points_xyz", []),
    }
    encoded = json.dumps(
        physical, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return "legacy-sha256:" + hashlib.sha256(encoded).hexdigest()


def _identity(row: dict[str, Any]) -> tuple[str, int, int, str] | None:
    payload = row.get("payload", {})
    trajectory_id = int(payload.get("trajectory_id", payload.get("traj_id", 0)) or 0)
    if trajectory_id <= 0:
        legacy_status = str(payload.get("status", ""))
        if ":" in legacy_status:
            try:
                trajectory_id = int(legacy_status.rsplit(":", 1)[1])
            except ValueError:
                trajectory_id = 0
    if trajectory_id <= 0:
        return None
    return (
        str(payload.get("execution_instance_id", "legacy")),
        trajectory_id,
        int(payload.get("start_time_ns", payload.get("trajectory_start_time_ns", 0)) or 0),
        _legacy_curve_hash(payload),
    )


def _timeline(records: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    relevant = {
        "normal_bspline", "pending_guard_bspline", "trajectory_status",
        "pending_guard_status",
        "poscmd", "controller_trace", "iap_odom", "control_output",
    }
    output = []
    for row in records:
        if row.get("kind") not in relevant:
            continue
        payload = row.get("payload", {})
        identity = _identity(row)
        output.append({
            "kind": row.get("kind", ""),
            "receive_steady_s": row.get("receive_steady_s", math.nan),
            "execution_instance_id": identity[0] if identity else "",
            "trajectory_id": identity[1] if identity else "",
            "start_time_ns": identity[2] if identity else "",
            "curve_hash": identity[3] if identity else "",
            "state": payload.get("state", payload.get("status", "")),
            "event_time_ns": payload.get("event_time_ns", ""),
            "parent_trajectory_id": payload.get("parent_trajectory_id", ""),
            "parent_start_time_ns": payload.get("parent_start_time_ns", ""),
            "parent_curve_hash": payload.get("parent_curve_hash", ""),
        })
    return output


def _identity_conflicts(records: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    seen: dict[tuple[str, int], set[str]] = {}
    for row in records:
        if row.get("kind") not in {"normal_bspline", "pending_guard_bspline"}:
            continue
        identity = _identity(row)
        if identity:
            seen.setdefault((identity[0], identity[1]), set()).add(identity[3])
    return [
        {"execution_instance_id": instance, "trajectory_id": trajectory_id,
         "curve_hashes": sorted(hashes)}
        for (instance, trajectory_id), hashes in sorted(seen.items())
        if len(hashes) > 1
    ]


def _nearest_odom(
    stamp_s: float, odometry: list[dict[str, Any]], maximum_delta_s: float = 0.05
) -> dict[str, Any] | None:
    candidates = []
    for row in odometry:
        value = row.get("payload", {}).get("stamp_s")
        if isinstance(value, (int, float)) and math.isfinite(value):
            candidates.append((abs(float(value) - stamp_s), row))
    if not candidates:
        return None
    delta, row = min(candidates, key=lambda item: item[0])
    return row if delta <= maximum_delta_s else None


def _tracking_error(records: list[dict[str, Any]]) -> dict[str, Any]:
    odometry = [row for row in records if row.get("kind") == "iap_odom"]
    samples = []
    for command in (row for row in records if row.get("kind") == "poscmd"):
        payload = command.get("payload", {})
        stamp = payload.get("stamp_s")
        desired = payload.get("position_xyz")
        velocity = payload.get("velocity_xyz")
        if not (isinstance(stamp, (int, float)) and isinstance(desired, list) and
                len(desired) == 3 and isinstance(velocity, list) and len(velocity) == 3):
            continue
        odom = _nearest_odom(float(stamp), odometry)
        odom_payload = odom.get("payload", {}) if odom else {}
        actual = odom_payload.get("position_xyz", odom_payload.get("position_m"))
        if not isinstance(actual, list) or len(actual) != 3:
            continue
        error = [float(actual[i]) - float(desired[i]) for i in range(3)]
        speed = math.sqrt(sum(float(component) ** 2 for component in velocity))
        if speed > 1.0e-9:
            tangent = [float(component) / speed for component in velocity]
            along = sum(error[i] * tangent[i] for i in range(3))
            cross_vector = [error[i] - along * tangent[i] for i in range(3)]
            cross = math.sqrt(sum(component ** 2 for component in cross_vector))
        else:
            along = math.nan
            cross = math.hypot(error[0], error[1])
        samples.append((along, cross, abs(error[2]), math.sqrt(sum(v * v for v in error))))
    finite_along = [abs(sample[0]) for sample in samples if math.isfinite(sample[0])]
    return {
        "sample_count": len(samples),
        "along_track_max_m": max(finite_along, default=None),
        "cross_track_max_m": max((sample[1] for sample in samples), default=None),
        "vertical_max_m": max((sample[2] for sample in samples), default=None),
        "total_max_m": max((sample[3] for sample in samples), default=None),
    }


def _controller_delays(records: list[dict[str, Any]]) -> tuple[str, dict[str, Any]]:
    publications: dict[tuple[str, int, int, str], float] = {}
    for row in records:
        if row.get("kind") not in {"normal_bspline", "pending_guard_bspline"}:
            continue
        identity = _identity(row)
        received = row.get("receive_steady_s")
        if identity and isinstance(received, (int, float)):
            publications.setdefault(identity, float(received))
    delays = []
    for row in records:
        if row.get("kind") != "controller_trace":
            continue
        identity = _identity(row)
        received = row.get("receive_steady_s")
        if identity in publications and isinstance(received, (int, float)):
            delays.append(1000.0 * (float(received) - publications[identity]))
    if not delays:
        return "uncertain", {"count": 0, "max": None, "p50": None}
    ordered = sorted(delays)
    return "measured", {
        "count": len(ordered), "max": max(ordered),
        "p50": ordered[(len(ordered) - 1) // 2],
    }


def _channel_decisions(run: Path) -> list[dict[str, Any]]:
    exports = run / "exports"
    candidates = _read_csv(
        exports / "planner_p4_risk_astar_debug.csv.forward_candidates.csv")
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in candidates:
        grouped.setdefault(row.get("decision_event_id", ""), []).append(row)
    output = []
    for decision_id, rows in sorted(grouped.items(), key=lambda item: item[0]):
        selected = next((row for row in rows if str(row.get("selected", "0")) == "1"), None)
        output.append({
            "decision_event_id": decision_id,
            "candidate_count": len(rows),
            "selected_candidate_id": selected.get("candidate_id", "") if selected else "",
            "selected_channel_id": selected.get("channel_id", "") if selected else "",
            "comparison_complete": all(row.get("reason", "") not in {
                "PENDING", "BUDGET_EXHAUSTED"} for row in rows),
            "candidates": rows,
        })
    return output


def analyze_run(run: Path) -> dict[str, Any]:
    records = _read_jsonl(run / "capture.jsonl")
    timeline = _timeline(records)
    conflicts = _identity_conflicts(records)
    status, delays = _controller_delays(records)
    activated = []
    for row in records:
        if row.get("kind") not in {"trajectory_status", "pending_guard_status"}:
            continue
        payload = row.get("payload", {})
        state = str(payload.get("state", payload.get("status", ""))).upper()
        if not (state == "ACTIVATED" or state.startswith("ACTIVATED:")):
            continue
        identity = _identity(row)
        if not identity:
            continue
        if row.get("kind") == "pending_guard_status" and not activated:
            ack_receive = row.get("receive_steady_s")
            preceding = [
                command for command in records
                if command.get("kind") == "poscmd"
                and isinstance(command.get("receive_steady_s"), (int, float))
                and isinstance(ack_receive, (int, float))
                and command["receive_steady_s"] <= ack_receive
            ]
            if preceding:
                previous_identity = _identity(preceding[-1])
                if previous_identity:
                    activated.append(previous_identity[1])
        if identity[1] not in activated:
            activated.append(identity[1])
    if not activated:
        first_position_command = next(
            (row for row in records if row.get("kind") == "poscmd"), None)
        identity = _identity(first_position_command) if first_position_command else None
        if identity:
            activated.append(identity[1])
    return {
        "schema_version": "iap_continuous_flight_analysis_v1",
        "source_run": str(run),
        "legacy_capture_exact_replay": False,
        "identity_conflict_count": len(conflicts),
        "identity_conflicts": conflicts,
        "controller_delay_status": status,
        "controller_delay_ms": delays,
        "tracking_error": _tracking_error(records),
        "baseline_reproduction": {"trajectory_sequence": activated},
        "timeline": timeline,
        "channel_decisions": _channel_decisions(run),
    }


def _write_csv(path: Path, fieldnames: Iterable[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    arguments = parser.parse_args(argv)
    report = analyze_run(arguments.run)
    _write_csv(arguments.run / "continuous_flight_timeline.csv", TIMELINE_FIELDS,
               report["timeline"])
    channel_fields = (
        "decision_event_id", "candidate_count", "selected_candidate_id",
        "selected_channel_id", "comparison_complete")
    _write_csv(arguments.run / "channel_comparison.csv", channel_fields,
               report["channel_decisions"])
    (arguments.run / "continuous_flight_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
