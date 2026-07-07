#!/usr/bin/env python3
"""Analyze Safety Planner validation artifacts."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import math
import re
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
TOPIC_ACTIVITY_TOPICS = [
    "/iap/integrity",
    "/sim/drone_0/lidar_body",
    "/drone_0_visual_slam/odom",
    "/drone_0_planning/bspline",
]
SCENARIO_MAP_TOPICS = ["/map_generator/global_cloud", "/map_generator/local_cloud"]
SAFETY_OFF_ZERO_TOPICS = [
    "/planning/risk_grid_health",
    "/iap/rviz/risk_grid_health",
    "/iap/rviz/predicted_pl_cloud",
    "/iap/rviz/risk_validity_cloud",
    "/iap/rviz/trajectory_integrity_samples",
    "/iap/rviz/current_traj_integrity_colored",
    "/iap/rviz/p5_gate_status",
    "/iap/rviz/p5_current_im_bars",
]
P5_STATUS_TOPIC = "/planning/integrity_gate_status"
P5_BAD_ACTIONS = {"REQUEST_REPLAN", "REQUEST_EMERGENCY_STOP_CANDIDATE"}
CONTINUOUS_MIN_COVERAGE_RATIO = 0.8
CONTINUOUS_MAX_GAP_S = 2.0
DEFAULT_START_XY = (-12.0, 0.0)
DEFAULT_GOAL_XY = (12.0, 0.0)
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


def artifact_prefix(experiment_id: str) -> str:
    prefix = re.sub(r"[^a-z0-9]+", "_", str(experiment_id).strip().lower()).strip("_")
    return prefix or "run"


def is_experiment(args: argparse.Namespace, experiment_id: str) -> bool:
    return str(args.experiment_id).strip().upper() == experiment_id.upper()


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


def open_bag_reader(bag_dir: Path, metadata: dict[str, Any]) -> Any:
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
    return reader


def pointcloud_xyz(msg: Any, max_points: int = 120_000) -> list[tuple[float, float, float]]:
    try:
        import numpy as np
        from sensor_msgs_py import point_cloud2

        arr = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        xyz = np.asarray(arr, dtype=float).reshape(-1, 3)
        if xyz.shape[0] > max_points:
            stride = int(math.ceil(xyz.shape[0] / float(max_points)))
            xyz = xyz[::stride]
        return [(float(row[0]), float(row[1]), float(row[2])) for row in xyz]
    except Exception:
        try:
            from sensor_msgs_py import point_cloud2

            points: list[tuple[float, float, float]] = []
            for idx, point in enumerate(
                point_cloud2.read_points(
                    msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            ):
                if len(points) >= max_points:
                    break
                if idx == len(points):
                    points.append((float(point[0]), float(point[1]), float(point[2])))
            return points
        except Exception:
            return []


def sample_every(metadata: dict[str, Any], topic: str, max_points: int) -> int:
    count = int((metadata.get("topic_counts", {}) or {}).get(topic, 0) or 0)
    if count <= max_points:
        return 1
    return int(math.ceil(count / float(max_points)))


def read_scenario_plot_data(
    bag_dir: Path,
    metadata: dict[str, Any],
) -> tuple[dict[str, Any], str]:
    if metadata.get("missing") or not bag_dir:
        return {}, "missing rosbag metadata"
    topic_counts = metadata.get("topic_counts", {}) or {}
    map_topic = next(
        (topic for topic in SCENARIO_MAP_TOPICS if int(topic_counts.get(topic, 0) or 0) > 0),
        "",
    )
    topics = {
        "/sim/drone_0/truth_odom",
        "/drone_0_visual_slam/odom",
        "/drone_0_planning/bspline",
    }
    if map_topic:
        topics.add(map_topic)
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = open_bag_reader(bag_dir, metadata)
        type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        msg_types = {
            topic: get_message(type_map[topic])
            for topic in topics
            if topic in type_map
        }
        truth_stride = sample_every(metadata, "/sim/drone_0/truth_odom", 2_000)
        slam_stride = sample_every(metadata, "/drone_0_visual_slam/odom", 1_500)
        truth_seen = 0
        slam_seen = 0
        data: dict[str, Any] = {
            "map_topic": map_topic,
            "map_points": [],
            "truth_xy": [],
            "slam_xy": [],
            "bspline_paths": [],
        }
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic not in msg_types:
                continue
            if topic == map_topic and not data["map_points"]:
                msg = deserialize_message(raw, msg_types[topic])
                data["map_points"] = pointcloud_xyz(msg)
            elif topic == "/sim/drone_0/truth_odom":
                truth_seen += 1
                if (truth_seen - 1) % truth_stride == 0:
                    msg = deserialize_message(raw, msg_types[topic])
                    pos = msg.pose.pose.position
                    data["truth_xy"].append((float(pos.x), float(pos.y)))
            elif topic == "/drone_0_visual_slam/odom":
                slam_seen += 1
                if (slam_seen - 1) % slam_stride == 0:
                    msg = deserialize_message(raw, msg_types[topic])
                    pos = msg.pose.pose.position
                    data["slam_xy"].append((float(pos.x), float(pos.y)))
            elif topic == "/drone_0_planning/bspline":
                msg = deserialize_message(raw, msg_types[topic])
                path = [
                    (float(point.x), float(point.y))
                    for point in getattr(msg, "pos_pts", [])
                    if math.isfinite(float(point.x)) and math.isfinite(float(point.y))
                ]
                if path:
                    data["bspline_paths"].append(path)
        return data, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return {}, str(exc)


def read_topic_timestamps(
    bag_dir: Path,
    metadata: dict[str, Any],
    topics: list[str],
) -> tuple[dict[str, list[float]], str]:
    if metadata.get("missing") or not bag_dir:
        return {}, "missing rosbag metadata"
    timestamps = {topic: [] for topic in topics}
    target_topics = set(topics)
    try:
        reader = open_bag_reader(bag_dir, metadata)
        while reader.has_next():
            topic, _, timestamp = reader.read_next()
            if topic in target_topics:
                timestamps[topic].append(float(timestamp) * 1.0e-9)
        return timestamps, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return timestamps, str(exc)


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


def summarize_integrity_source_fields(rows: list[dict[str, Any]]) -> dict[str, Any]:
    fusion_modes = Counter(source_category(row.get("fusion_mode")) for row in rows)
    hpl_sources = Counter(source_category(row.get("final_hpl_source")) for row in rows)
    vpl_sources = Counter(source_category(row.get("final_vpl_source")) for row in rows)
    fallback_valid_count = sum(1 for row in rows if csv_bool(row.get("fallback_valid")))
    return {
        "fusion_mode_counts": dict(sorted(fusion_modes.items())),
        "final_hpl_source_counts": dict(sorted(hpl_sources.items())),
        "final_vpl_source_counts": dict(sorted(vpl_sources.items())),
        "fallback_valid_seen": fallback_valid_count > 0,
        "fallback_valid_count": fallback_valid_count,
    }


def validate_b0_4_fallback_requirements(
    rows: list[dict[str, Any]],
    validator_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
) -> None:
    if not validator_summary:
        inconclusive.append("B0-4 fallback checks require validator summary")
    else:
        if validator_summary.get("fallback_valid_seen") is not True:
            failures.append("B0-4 validator summary did not see fallback_valid=true")
        required_final_source = str(validator_summary.get("required_final_source", "")).strip()
        if required_final_source != "FALLBACK":
            failures.append("B0-4 validator was not configured with required_final_source=FALLBACK")
        if validator_summary.get("require_fallback_valid") is not True:
            failures.append("B0-4 validator was not configured to require fallback validity")

    if not rows:
        inconclusive.append("B0-4 fallback checks require integrity CSV rows")
        return

    fallback_valid_seen = any(csv_bool(row.get("fallback_valid")) for row in rows)
    if not fallback_valid_seen:
        failures.append("B0-4 integrity CSV never reports fallback_valid=true")

    hpl_sources = {
        source_category(row.get("final_hpl_source"))
        for row in rows
        if source_category(row.get("final_hpl_source")) != "UNKNOWN"
    }
    vpl_sources = {
        source_category(row.get("final_vpl_source"))
        for row in rows
        if source_category(row.get("final_vpl_source")) != "UNKNOWN"
    }
    if not hpl_sources or hpl_sources != {"FALLBACK"}:
        failures.append(
            "B0-4 integrity CSV final_hpl_source is not exclusively FALLBACK: "
            + ",".join(sorted(hpl_sources or {"<none>"}))
        )
    if not vpl_sources or vpl_sources != {"FALLBACK"}:
        failures.append(
            "B0-4 integrity CSV final_vpl_source is not exclusively FALLBACK: "
            + ",".join(sorted(vpl_sources or {"<none>"}))
        )


def validate_safety_off_topic_leakage(
    metadata: dict[str, Any],
    failures: list[str],
) -> dict[str, int]:
    topic_counts = metadata.get("topic_counts", {}) or {}
    safety_counts = {
        topic: int(topic_counts.get(topic, 0) or 0)
        for topic in SAFETY_OFF_ZERO_TOPICS
    }
    leaked = {topic: count for topic, count in safety_counts.items() if count > 0}
    if leaked:
        failures.append(
            "safety-off run recorded nonzero P0/P5 risk/RViz topics: "
            + ", ".join(f"{topic}={count}" for topic, count in sorted(leaked.items()))
        )
    return safety_counts


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


def xy_columns(points: list[tuple[float, float]] | list[tuple[float, float, float]]) -> tuple[list[float], list[float]]:
    x: list[float] = []
    y: list[float] = []
    for point in points:
        if len(point) < 2:
            continue
        px = finite_float(point[0])
        py = finite_float(point[1])
        if px is None or py is None:
            continue
        x.append(px)
        y.append(py)
    return x, y


def choose_start_goal(data: dict[str, Any]) -> tuple[tuple[float, float], tuple[float, float]]:
    for key in ("truth_xy", "slam_xy"):
        points = data.get(key) or []
        if len(points) >= 2:
            return tuple(points[0]), tuple(points[-1])
    for path in data.get("bspline_paths", []) or []:
        if len(path) >= 2:
            return tuple(path[0]), tuple(path[-1])
    return DEFAULT_START_XY, DEFAULT_GOAL_XY


def plot_scenario_topdown(data: dict[str, Any], path: Path) -> bool:
    if not data:
        return False
    has_any = any(
        data.get(key)
        for key in ("map_points", "truth_xy", "slam_xy", "bspline_paths")
    )
    if not has_any:
        return False

    fig, ax = plt.subplots(figsize=(8, 7))
    map_points = data.get("map_points") or []
    if map_points:
        mx, my = xy_columns(map_points)
        ax.scatter(mx, my, s=1.0, c="#9ca3af", alpha=0.25, linewidths=0, label=data.get("map_topic") or "map")

    truth_points = data.get("truth_xy") or []
    if truth_points:
        tx, ty = xy_columns(truth_points)
        ax.plot(tx, ty, color="#2563eb", lw=1.8, label="truth odom")

    slam_points = data.get("slam_xy") or []
    if slam_points:
        sx, sy = xy_columns(slam_points)
        ax.plot(sx, sy, color="#16a34a", lw=1.4, ls="--", label="visual slam odom")

    bspline_paths = data.get("bspline_paths") or []
    for idx, bspline_path in enumerate(bspline_paths):
        bx, by = xy_columns(bspline_path)
        if not bx:
            continue
        ax.plot(
            bx,
            by,
            color="#f97316",
            lw=1.0,
            alpha=0.45,
            label="bspline pos_pts" if idx == 0 else None,
        )

    start, goal = choose_start_goal(data)
    ax.scatter([start[0]], [start[1]], marker="o", s=80, c="#111827", label="start", zorder=5)
    ax.scatter([goal[0]], [goal[1]], marker="*", s=140, c="#dc2626", label="goal", zorder=5)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Scenario top-down")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_topic_activity_timeline(
    timestamps: dict[str, list[float]],
    path: Path,
) -> bool:
    all_stamps = [stamp for values in timestamps.values() for stamp in values]
    if not all_stamps:
        return False
    t0 = min(all_stamps)
    fig, ax = plt.subplots(figsize=(11, 4.8))
    y_positions = list(range(len(TOPIC_ACTIVITY_TOPICS)))
    rel_events: list[list[float]] = []
    labels: list[str] = []
    for topic in TOPIC_ACTIVITY_TOPICS:
        values = sorted(timestamps.get(topic, []))
        rel = [stamp - t0 for stamp in values]
        rel_events.append(rel)
        labels.append(topic)
    ax.eventplot(
        rel_events,
        lineoffsets=y_positions,
        linelengths=0.65,
        linewidths=0.8,
        colors=["#2563eb", "#16a34a", "#9333ea", "#f97316"],
    )
    gap_label_added = False
    for y_pos, values in zip(y_positions, rel_events):
        for prev, curr in zip(values, values[1:]):
            if curr - prev > CONTINUOUS_MAX_GAP_S:
                ax.hlines(
                    y_pos,
                    prev,
                    curr,
                    color="#dc2626",
                    lw=4,
                    alpha=0.45,
                    label=f"gap > {CONTINUOUS_MAX_GAP_S:.1f}s" if not gap_label_added else None,
                )
                gap_label_added = True
    ax.set_yticks(y_positions, labels)
    ax.set_xlabel("run-relative bag time [s]")
    ax.set_title("Topic activity timeline")
    ax.grid(True, axis="x", alpha=0.25)
    if gap_label_added:
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def csv_bool(value: Any) -> bool:
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y"}:
        return True
    number = finite_float(value)
    return bool(number) if number is not None else False


def source_category(value: Any) -> str:
    text = str(value).strip()
    return text if text else "UNKNOWN"


def plot_integrity_source_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    points: list[dict[str, Any]] = []
    for row in rows:
        stamp = finite_float(row.get("stamp"))
        if stamp is None:
            continue
        points.append(
            {
                "stamp": stamp,
                "fusion_mode": source_category(row.get("fusion_mode")),
                "final_hpl_source": source_category(row.get("final_hpl_source")),
                "final_vpl_source": source_category(row.get("final_vpl_source")),
                "fallback_valid": "true" if csv_bool(row.get("fallback_valid")) else "false",
            }
        )
    if not points:
        return False
    t0 = points[0]["stamp"]
    t = [float(row["stamp"]) - float(t0) for row in points]
    lanes = [
        ("fusion_mode", "fusion mode"),
        ("final_hpl_source", "final HPL source"),
        ("final_vpl_source", "final VPL source"),
        ("fallback_valid", "fallback valid"),
    ]
    fig, axes = plt.subplots(len(lanes), 1, figsize=(11, 7), sharex=True)
    for ax, (field, label) in zip(axes, lanes):
        values = [str(row[field]) for row in points]
        categories = sorted(set(values))
        if field == "fallback_valid":
            categories = ["false", "true"]
        elif field.startswith("final_"):
            preferred = ["UNKNOWN", "GNSS", "LIDAR", "FALLBACK", "CONSERVATIVE"]
            categories = [item for item in preferred if item in categories] + [
                item for item in categories if item not in preferred
            ]
        code_by_value = {value: idx for idx, value in enumerate(categories)}
        codes = [code_by_value[value] for value in values]
        ax.step(t, codes, where="post", lw=1.5)
        ax.set_yticks(range(len(categories)), categories)
        ax.set_ylabel(label)
        ax.grid(True, axis="x", alpha=0.25)
    axes[-1].set_xlabel("time since first validator sample [s]")
    fig.suptitle("Integrity source timeline")
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


def next_debug_branch(
    status: str,
    failures: list[str],
    inconclusive: list[str],
    experiment_id: str,
) -> str:
    text = " ".join(failures + inconclusive).lower()
    if status == "PASS":
        pass_branches = {
            "B0-1": "continue_to_B0-2_open_sky_baseline",
            "B0-2": "continue_to_B0-3_corridor_baseline",
            "B0-3": "continue_to_B0-4_fallback_baseline",
            "B0-4": "continue_to_P0-1_open_sky_data_only_validation",
        }
        return pass_branches.get(
            str(experiment_id).strip().upper(),
            "continue_to_next_planned_experiment",
        )
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
    prefix = artifact_prefix(args.experiment_id)

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
    integrity_summary.update(summarize_integrity_source_fields(integrity_rows))

    safety_off_topic_counts = {}
    if is_experiment(args, "B0-4"):
        validate_b0_4_fallback_requirements(
            integrity_rows,
            validator_summary,
            failures,
            inconclusive,
        )
        safety_off_topic_counts = validate_safety_off_topic_leakage(metadata, failures)

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
    topic_counts_path = csv_dir / f"{prefix}_topic_counts.csv"
    write_csv(
        topic_counts_path,
        ["topic", "expected", "count", "hz", "span_s", "coverage_ratio", "max_gap_s", "status"],
        topic_count_rows,
    )

    figures: list[str] = []
    scenario_figure_path = figures_dir / f"{prefix}_scenario_topdown.png"
    topic_activity_figure_path = figures_dir / f"{prefix}_topic_activity_timeline.png"
    source_figure_path = figures_dir / f"{prefix}_integrity_source_timeline.png"
    integrity_figure_path = figures_dir / f"{prefix}_integrity_hpl_vpl_timeline.png"

    if bag_dir is not None:
        scenario_data, scenario_error = read_scenario_plot_data(bag_dir, metadata)
        if scenario_error:
            warnings.append(f"scenario top-down plot data could not be read: {scenario_error}")
        if plot_scenario_topdown(scenario_data, scenario_figure_path):
            figures.append(str(scenario_figure_path))
        else:
            warnings.append("scenario top-down plot was not generated because no plottable bag data was available")

        topic_timestamps, topic_timestamp_error = read_topic_timestamps(
            bag_dir,
            metadata,
            TOPIC_ACTIVITY_TOPICS,
        )
        if topic_timestamp_error:
            warnings.append(f"topic activity timeline data could not be read: {topic_timestamp_error}")
        if plot_topic_activity_timeline(topic_timestamps, topic_activity_figure_path):
            figures.append(str(topic_activity_figure_path))
        else:
            warnings.append("topic activity timeline was not generated because no plottable bag timestamps were available")
    else:
        warnings.append("scenario and topic activity figures were not generated because no bag dir was provided")

    if plot_integrity_source_timeline(integrity_rows, source_figure_path):
        figures.append(str(source_figure_path))
    else:
        warnings.append("integrity source timeline was not generated because no plottable rows were available")

    if plot_integrity_timeline(integrity_rows, integrity_figure_path):
        figures.append(str(integrity_figure_path))
    else:
        warnings.append("integrity HPL/VPL timeline was not generated because no plottable rows were available")

    p5_rows, p5_error = read_p5_status_messages(bag_dir, metadata) if bag_dir is not None else ([], "")
    p5_summary = validate_p5_status(p5_rows, p5_error, failures, inconclusive)
    csv_artifacts = [str(topic_counts_path)]
    if p5_rows:
        p5_status_path = csv_dir / f"{prefix}_p5_status.csv"
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

    if is_experiment(args, "B0-4"):
        required_figures = [
            scenario_figure_path,
            topic_activity_figure_path,
            source_figure_path,
        ]
        for figure_path in required_figures:
            if not figure_path.is_file() or figure_path.stat().st_size <= 0:
                inconclusive.append(
                    f"B0-4 required figure was not generated or is empty: {figure_path}"
                )

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
        "next_debug_branch": next_debug_branch(
            status,
            failures,
            inconclusive,
            args.experiment_id,
        ),
        "export_dir": str(export_dir),
        "bag_dir": str(bag_dir) if bag_dir is not None else "",
        "manifest": manifest,
        "validator_summary": validator_summary,
        "topic_health": topic_health,
        "topic_timing_error": topic_timing_error,
        "integrity_summary": integrity_summary,
        "p0_summary": {},
        "p5_summary": p5_summary,
        "safety_off_topic_counts": safety_off_topic_counts,
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
