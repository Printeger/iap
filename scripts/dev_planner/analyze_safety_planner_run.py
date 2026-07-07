#!/usr/bin/env python3
"""Analyze Safety Planner validation artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


CORE_TOPIC_EXPECTATIONS = {
    "/iap/integrity": "continuous",
    "/sim/drone_0/lidar_body": "continuous",
    "/drone_0_planning/bspline": "planner-dependent",
}
P5_STATUS_TOPIC = "/planning/integrity_gate_status"
P5_BAD_ACTIONS = {"REQUEST_REPLAN", "REQUEST_EMERGENCY_STOP_CANDIDATE"}
CONTINUOUS_MIN_COVERAGE_RATIO = 0.8
CONTINUOUS_MAX_GAP_S = 2.0
SAFETY_OFF_BOOL_KEYS = (
    "p0.enable_risk_grid",
    "planner_enable_p1",
    "planner_enable_p2",
    "planner_enable_p3_local",
    "planner_enable_p3_global",
    "planner_enable_p4",
    "planner_enable_p5_runtime",
    "planner_enable_p5_final",
)


def ensure_dirs(export_dir: Path) -> tuple[Path, Path, Path]:
    csv_dir = export_dir / "csv"
    figures_dir = export_dir / "figures"
    metadata_dir = export_dir / "metadata"
    csv_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    return csv_dir, figures_dir, metadata_dir


def read_json_if_exists(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open() as f:
        return json.load(f)


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n")


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def read_bag_metadata(bag_dir: Path) -> dict[str, Any]:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return {"missing": True, "path": str(metadata_path), "topic_counts": {}}
    try:
        import yaml

        with metadata_path.open() as f:
            raw = yaml.safe_load(f) or {}
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return {
            "missing": False,
            "path": str(metadata_path),
            "parse_error": str(exc),
            "topic_counts": {},
        }
    info = raw.get("rosbag2_bagfile_information", {})
    topic_counts: dict[str, int] = {}
    topic_types: dict[str, str] = {}
    for item in info.get("topics_with_message_count", []) or []:
        topic_metadata = item.get("topic_metadata", {}) or {}
        name = str(topic_metadata.get("name", ""))
        if not name:
            continue
        topic_counts[name] = int(item.get("message_count", 0) or 0)
        topic_types[name] = str(topic_metadata.get("type", ""))
    return {
        "missing": False,
        "path": str(metadata_path),
        "storage_identifier": str(info.get("storage_identifier", "")),
        "duration_ns": int((info.get("duration", {}) or {}).get("nanoseconds", 0) or 0),
        "message_count": int(info.get("message_count", 0) or 0),
        "topic_counts": topic_counts,
        "topic_types": topic_types,
    }


def read_topic_timings(
    bag_dir: Path,
    metadata: dict[str, Any],
    topics: list[str],
) -> tuple[dict[str, dict[str, Any]], str]:
    if metadata.get("missing") or not bag_dir:
        return {}, ""

    stats = {
        topic: {
            "count": 0,
            "first_bag_time_s": None,
            "last_bag_time_s": None,
            "span_s": None,
            "max_gap_s": None,
        }
        for topic in topics
    }
    try:
        import rosbag2_py

        storage_id = str(metadata.get("storage_identifier", "")) or ""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        target_topics = set(topics)
        previous: dict[str, float] = {}
        while reader.has_next():
            topic, _, timestamp = reader.read_next()
            if topic not in target_topics:
                continue
            bag_time_s = float(timestamp) * 1.0e-9
            item = stats[topic]
            item["count"] = int(item["count"]) + 1
            if item["first_bag_time_s"] is None:
                item["first_bag_time_s"] = bag_time_s
            if topic in previous:
                gap = bag_time_s - previous[topic]
                item["max_gap_s"] = max(float(item["max_gap_s"] or 0.0), gap)
            previous[topic] = bag_time_s
            item["last_bag_time_s"] = bag_time_s
        for item in stats.values():
            first = item["first_bag_time_s"]
            last = item["last_bag_time_s"]
            if first is not None and last is not None:
                item["span_s"] = float(last) - float(first)
                item["max_gap_s"] = float(item["max_gap_s"] or 0.0)
        return stats, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return stats, str(exc)


def validate_manifest(manifest: dict[str, Any], failures: list[str], inconclusive: list[str]) -> None:
    if not manifest:
        inconclusive.append("missing test_planner_manifest.json")
        return
    if str(manifest.get("planner_safety_profile", "")).lower() != "off":
        failures.append("manifest planner_safety_profile is not off")
    for key in SAFETY_OFF_BOOL_KEYS:
        if manifest.get(key) is not False:
            failures.append(f"manifest {key} is not false")


def validate_validator(summary: dict[str, Any], failures: list[str], inconclusive: list[str]) -> None:
    if not summary:
        inconclusive.append("missing test_planner_validation_summary.json")
        return
    if summary.get("passed") is not True:
        failures.append("validator summary passed is not true")


def read_integrity_csv(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not path.is_file():
        return [], {"missing": True, "path": str(path)}
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    stamps = [value for value in (finite_float(row.get("stamp")) for row in rows) if value is not None]
    hpl_values = [value for value in (finite_float(row.get("hpl")) for row in rows) if value is not None]
    vpl_values = [value for value in (finite_float(row.get("vpl")) for row in rows) if value is not None]
    failure_reasons: dict[str, int] = {}
    for row in rows:
        reason = str(row.get("failure_reason", "")).strip()
        if reason:
            failure_reasons[reason] = failure_reasons.get(reason, 0) + 1
    stats = {
        "missing": False,
        "path": str(path),
        "row_count": len(rows),
        "first_stamp": min(stamps) if stamps else None,
        "last_stamp": max(stamps) if stamps else None,
        "duration_s": (max(stamps) - min(stamps)) if len(stamps) >= 2 else None,
        "hpl_min": min(hpl_values) if hpl_values else None,
        "hpl_mean": (sum(hpl_values) / len(hpl_values)) if hpl_values else None,
        "hpl_max": max(hpl_values) if hpl_values else None,
        "vpl_min": min(vpl_values) if vpl_values else None,
        "vpl_mean": (sum(vpl_values) / len(vpl_values)) if vpl_values else None,
        "vpl_max": max(vpl_values) if vpl_values else None,
        "failure_reasons": failure_reasons,
    }
    return rows, stats


def plot_integrity_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    points: list[tuple[float, float | None, float | None]] = []
    for row in rows:
        stamp = finite_float(row.get("stamp"))
        if stamp is None:
            continue
        points.append((stamp, finite_float(row.get("hpl")), finite_float(row.get("vpl"))))
    if not points:
        return False
    t0 = points[0][0]
    t = [p[0] - t0 for p in points]
    hpl = [math.nan if p[1] is None else p[1] for p in points]
    vpl = [math.nan if p[2] is None else p[2] for p in points]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(t, hpl, label="HPL")
    ax.plot(t, vpl, label="VPL")
    ax.set_xlabel("time since first validator sample [s]")
    ax.set_ylabel("protection level [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def topic_health_from_metadata(
    metadata: dict[str, Any],
    timings: dict[str, dict[str, Any]] | None = None,
) -> dict[str, dict[str, Any]]:
    topic_counts = metadata.get("topic_counts", {}) or {}
    duration_s = float(metadata.get("duration_ns", 0) or 0) * 1.0e-9
    topic_health: dict[str, dict[str, Any]] = {}
    timings = timings or {}
    for topic, expected in CORE_TOPIC_EXPECTATIONS.items():
        count = int(topic_counts.get(topic, 0) or 0)
        hz = count / duration_s if duration_s > 0.0 else None
        timing = timings.get(topic, {}) or {}
        span_s = finite_float(timing.get("span_s"))
        max_gap_s = finite_float(timing.get("max_gap_s"))
        coverage_ratio = (span_s / duration_s) if span_s is not None and duration_s > 0.0 else None
        status = "PASS" if count > 0 else "FAIL"
        if expected == "continuous" and count > 0:
            if coverage_ratio is None or max_gap_s is None:
                status = "CHECK"
            elif (
                coverage_ratio < CONTINUOUS_MIN_COVERAGE_RATIO
                or max_gap_s > CONTINUOUS_MAX_GAP_S
            ):
                status = "FAIL"
        topic_health[topic] = {
            "expected": expected,
            "count": count,
            "hz": hz,
            "span_s": span_s,
            "coverage_ratio": coverage_ratio,
            "max_gap_s": max_gap_s,
            "status": status,
        }
    p5_count = int(topic_counts.get(P5_STATUS_TOPIC, 0) or 0)
    topic_health[P5_STATUS_TOPIC] = {
        "expected": "absent-or-zero when P5 disabled",
        "count": p5_count,
        "hz": (p5_count / duration_s) if duration_s > 0.0 else None,
        "span_s": None,
        "coverage_ratio": None,
        "max_gap_s": None,
        "status": "PASS" if p5_count == 0 else "CHECK",
    }
    return topic_health


def validate_topic_health(
    metadata: dict[str, Any],
    timings: dict[str, dict[str, Any]],
    timing_error: str,
    failures: list[str],
    inconclusive: list[str],
) -> dict[str, dict[str, Any]]:
    if metadata.get("missing"):
        inconclusive.append(f"missing rosbag metadata: {metadata.get('path', '')}")
        return topic_health_from_metadata(metadata, timings)
    if metadata.get("parse_error"):
        inconclusive.append(f"could not parse rosbag metadata: {metadata['parse_error']}")
        return topic_health_from_metadata(metadata, timings)
    if timing_error:
        inconclusive.append(f"could not inspect core topic timing: {timing_error}")
    topic_health = topic_health_from_metadata(metadata, timings)
    for topic in CORE_TOPIC_EXPECTATIONS:
        if topic_health[topic]["status"] == "FAIL":
            failures.append(f"required topic {topic} is missing or not continuous")
        elif topic_health[topic]["status"] == "CHECK":
            inconclusive.append(f"required topic {topic} continuity could not be measured")
    return topic_health


def read_p5_status_messages(bag_dir: Path, metadata: dict[str, Any], limit: int = 2000) -> tuple[list[dict[str, Any]], str]:
    topic_counts = metadata.get("topic_counts", {}) or {}
    if int(topic_counts.get(P5_STATUS_TOPIC, 0) or 0) <= 0:
        return [], ""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        storage_id = str(metadata.get("storage_identifier", "")) or ""
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )
        reader.open(storage_options, converter_options)
        type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if P5_STATUS_TOPIC not in type_map:
            return [], "P5 status topic count was nonzero but topic type is absent"
        msg_type = get_message(type_map[P5_STATUS_TOPIC])
        rows: list[dict[str, Any]] = []
        while reader.has_next() and len(rows) < limit:
            topic, data, timestamp = reader.read_next()
            if topic != P5_STATUS_TOPIC:
                continue
            msg = deserialize_message(data, msg_type)
            payload = getattr(msg, "data", "")
            parsed: dict[str, Any] = {}
            if isinstance(payload, str) and payload.strip():
                try:
                    loaded = json.loads(payload)
                    if isinstance(loaded, dict):
                        parsed = loaded
                except json.JSONDecodeError:
                    parsed = {"parse_error": payload[:120]}
            rows.append(
                {
                    "bag_time_s": float(timestamp) * 1.0e-9,
                    "phase": parsed.get("phase", ""),
                    "action": parsed.get("action", ""),
                    "raw_action": parsed.get("raw_action", ""),
                    "reason": parsed.get("reason", ""),
                    "final_gate_fail_count": parsed.get("final_gate_fail_count", ""),
                    "final_gate_last_reason": parsed.get("final_gate_last_reason", ""),
                    "raw": payload,
                }
            )
        return rows, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return [], str(exc)


def validate_p5_status(
    p5_rows: list[dict[str, Any]],
    p5_error: str,
    failures: list[str],
    inconclusive: list[str],
) -> dict[str, Any]:
    actions: dict[str, int] = {}
    bad_rows: list[dict[str, Any]] = []
    for row in p5_rows:
        action = str(row.get("action", "")).strip()
        raw_action = str(row.get("raw_action", "")).strip()
        for value in (action, raw_action):
            if value:
                actions[value] = actions.get(value, 0) + 1
        fail_count = finite_float(row.get("final_gate_fail_count"))
        if action in P5_BAD_ACTIONS or raw_action in P5_BAD_ACTIONS or (fail_count is not None and fail_count > 0):
            bad_rows.append(row)
    if p5_error:
        inconclusive.append(f"could not inspect P5 status messages: {p5_error}")
    if bad_rows:
        failures.append("P5 status contains replan, emergency, or final gate failure behavior")
    return {
        "status_rows": len(p5_rows),
        "action_counts": actions,
        "bad_action_count": len(bad_rows),
        "inspection_error": p5_error,
        "first_bad_action": bad_rows[0] if bad_rows else None,
    }


def next_debug_branch(status: str, failures: list[str], inconclusive: list[str]) -> str:
    text = " ".join(failures + inconclusive).lower()
    if status == "PASS":
        return "continue_to_B0-2_open_sky_baseline"
    if "manifest" in text:
        return "debug_baseline_launch_manifest_switch_isolation"
    if "validator" in text:
        return "debug_validator_summary_and_integrity_csv"
    if "topic" in text or "bag" in text or "metadata" in text:
        return "debug_bag_recording_and_launch_node_health"
    if "p5" in text:
        return "debug_p5_switch_leakage"
    return "debug_B0-1_baseline"


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    export_dir = Path(args.export_dir).expanduser().resolve()
    bag_dir = Path(args.bag_dir).expanduser().resolve() if args.bag_dir else None
    csv_dir, figures_dir, metadata_dir = ensure_dirs(export_dir)

    failures: list[str] = []
    warnings: list[str] = []
    inconclusive: list[str] = []
    manifest = read_json_if_exists(export_dir / "test_planner_manifest.json")
    validator_summary = read_json_if_exists(export_dir / "test_planner_validation_summary.json")
    metadata = read_bag_metadata(bag_dir) if bag_dir is not None else {"missing": True, "topic_counts": {}}
    topic_timings, topic_timing_error = (
        read_topic_timings(bag_dir, metadata, list(CORE_TOPIC_EXPECTATIONS.keys()))
        if bag_dir is not None
        else ({}, "")
    )
    integrity_rows, integrity_summary = read_integrity_csv(export_dir / "test_planner_integrity_validation.csv")

    validate_manifest(manifest, failures, inconclusive)
    validate_validator(validator_summary, failures, inconclusive)
    topic_health = validate_topic_health(
        metadata,
        topic_timings,
        topic_timing_error,
        failures,
        inconclusive,
    )

    if integrity_summary.get("missing"):
        inconclusive.append("missing test_planner_integrity_validation.csv")
    elif int(integrity_summary.get("row_count", 0) or 0) <= 0:
        failures.append("test_planner_integrity_validation.csv has no data rows")

    topic_count_rows = [
        {
            "topic": topic,
            "expected": data["expected"],
            "count": data["count"],
            "hz": "" if data["hz"] is None else f"{float(data['hz']):.6f}",
            "span_s": "" if data.get("span_s") is None else f"{float(data['span_s']):.6f}",
            "coverage_ratio": ""
            if data.get("coverage_ratio") is None
            else f"{float(data['coverage_ratio']):.6f}",
            "max_gap_s": ""
            if data.get("max_gap_s") is None
            else f"{float(data['max_gap_s']):.6f}",
            "status": data["status"],
        }
        for topic, data in topic_health.items()
    ]
    topic_counts_path = csv_dir / "b0_1_topic_counts.csv"
    write_csv(
        topic_counts_path,
        ["topic", "expected", "count", "hz", "span_s", "coverage_ratio", "max_gap_s", "status"],
        topic_count_rows,
    )

    figures: list[str] = []
    integrity_figure_path = figures_dir / "b0_1_integrity_hpl_vpl_timeline.png"
    if plot_integrity_timeline(integrity_rows, integrity_figure_path):
        figures.append(str(integrity_figure_path))
    else:
        warnings.append("integrity HPL/VPL timeline was not generated because no plottable rows were available")

    p5_rows, p5_error = read_p5_status_messages(bag_dir, metadata) if bag_dir is not None else ([], "")
    p5_summary = validate_p5_status(p5_rows, p5_error, failures, inconclusive)
    csv_artifacts = [str(topic_counts_path)]
    if p5_rows:
        p5_status_path = csv_dir / "b0_1_p5_status.csv"
        write_csv(
            p5_status_path,
            [
                "bag_time_s",
                "phase",
                "action",
                "raw_action",
                "reason",
                "final_gate_fail_count",
                "final_gate_last_reason",
                "raw",
            ],
            p5_rows,
        )
        csv_artifacts.append(str(p5_status_path))

    status = "PASS"
    if failures:
        status = "FAIL"
    elif inconclusive:
        status = "INCONCLUSIVE"

    summary = {
        "experiment_id": args.experiment_id,
        "status": status,
        "passed": status == "PASS",
        "failures": failures,
        "warnings": warnings,
        "inconclusive": inconclusive,
        "next_debug_branch": next_debug_branch(status, failures, inconclusive),
        "export_dir": str(export_dir),
        "bag_dir": str(bag_dir) if bag_dir is not None else "",
        "manifest": manifest,
        "validator_summary": validator_summary,
        "topic_health": topic_health,
        "topic_timing_error": topic_timing_error,
        "integrity_summary": integrity_summary,
        "p0_summary": {},
        "p5_summary": p5_summary,
        "module_metrics": {},
        "artifacts": {
            "csv": csv_artifacts,
            "figures": figures,
        },
    }
    out_path = metadata_dir / "safety_planner_analysis_summary.json"
    write_json(out_path, summary)
    summary["summary_path"] = str(out_path)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment-id", required=True, help="Safety Planner experiment ID, e.g. B0-1")
    parser.add_argument("--export-dir", required=True, help="test_planner export directory")
    parser.add_argument("--bag-dir", default="", help="test_planner rosbag directory")
    parser.add_argument("--baseline-export-dir", default="", help="baseline export directory, recorded for compatibility")
    parser.add_argument("--baseline-bag-dir", default="", help="baseline rosbag directory, recorded for compatibility")
    parser.add_argument("--fail-on-threshold", action="store_true", help="exit non-zero unless analysis status is PASS")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze(args)
    return 0 if (summary["passed"] or not args.fail_on_threshold) else 2


if __name__ == "__main__":
    raise SystemExit(main())
