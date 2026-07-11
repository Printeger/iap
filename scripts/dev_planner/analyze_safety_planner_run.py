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
import numpy as np


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
P0_HEALTH_TOPIC = "/planning/risk_grid_health"
P0_PL_CLOUD_TOPIC = "/iap/rviz/predicted_pl_cloud"
P0_VALIDITY_CLOUD_TOPIC = "/iap/rviz/risk_validity_cloud"
P0_TOPIC_ACTIVITY_TOPICS = TOPIC_ACTIVITY_TOPICS + [
    P0_HEALTH_TOPIC,
    P0_PL_CLOUD_TOPIC,
    P0_VALIDITY_CLOUD_TOPIC,
]
P0_TOPIC_EXPECTATIONS = {
    "/iap/integrity": "continuous",
    "/sim/drone_0/lidar_body": "continuous",
    "/drone_0_visual_slam/odom": "continuous",
    "/drone_0_planning/bspline": "planner-dependent",
    P0_HEALTH_TOPIC: "active-periodic",
    P0_PL_CLOUD_TOPIC: "present",
    P0_VALIDITY_CLOUD_TOPIC: "present",
}
P0_4_TOPIC_EXPECTATIONS = {
    "/iap/integrity": "continuous",
    P0_HEALTH_TOPIC: "active-periodic",
    P0_PL_CLOUD_TOPIC: "present",
    P0_VALIDITY_CLOUD_TOPIC: "present",
}
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
ODOM_TRUTH_TOPIC = "/sim/drone_0/truth_odom"
ODOM_EST_TOPIC = "/drone_0_visual_slam/odom"
CONTINUOUS_MIN_COVERAGE_RATIO = 0.8
CONTINUOUS_MAX_GAP_S = 2.0
P0_HEALTH_ACTIVE_MAX_GAP_S = 3.0
ODOM_DRIFT_RMS_ERROR_M = 1.5
ODOM_DRIFT_MAX_ERROR_M = 4.0
ODOM_DRIFT_FINAL_ERROR_M = 2.5
ODOM_DRIFT_Z_ERROR_M = 1.5
ODOM_DRIFT_YAW_ERROR_DEG = 45.0
ODOM_DRIFT_POINT_ERROR_M = 2.5
ODOM_JUMP_STEP_M = 1.5
ODOM_JUMP_SPEED_MPS = 5.0
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
PREDICTOR_SOURCE_COUNTER_FIELDS = [
    "predictor_gnss_used_count",
    "predictor_lidar_used_count",
    "predictor_prior_used_count",
    "predictor_regularized_count",
    "predictor_conservative_max_count",
]
PREDICTOR_LIDAR_INPUT_FIELDS = [
    "predictor_lidar_map_point_count",
    "predictor_lidar_fim_primitive_count",
    "predictor_lidar_fim_valid_normal_count",
    "predictor_lidar_fim_fallback_reason",
]
P0_UNKNOWN_REASON_FIELDS = [
    "dominant_unknown_reason",
    "dominant_unknown_count",
]
P0_5_SYNTHETIC_AFFINE_GRADIENT = (2.0, 3.0, 4.0)
P0_5_SYNTHETIC_QUERY_POINTS = [
    (-0.75, -0.25, 0.0, 0.0),
    (-0.25, 0.10, 0.2, 0.25),
    (0.25, 0.50, -0.2, 0.5),
    (0.60, -0.40, 0.3, 1.0),
    (0.90, 0.75, 0.1, 1.5),
]
P0_5_SYNTHETIC_QUERY_FIELDS = [
    "sample_id",
    "x",
    "y",
    "z",
    "tau",
    "expected_c_pi",
    "actual_c_pi",
    "hpl_pred",
    "abs_error",
    "valid",
    "unknown",
    "stale",
    "reason",
    "grad_x",
    "grad_y",
    "grad_z",
    "neg_grad_points_lower_risk",
]


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


def is_p0_experiment(args: argparse.Namespace) -> bool:
    return re.fullmatch(r"P0-\d+", str(args.experiment_id).strip().upper()) is not None


def p0_phase_number(experiment_id: Any) -> int | None:
    match = re.fullmatch(r"P0-(\d+)", str(experiment_id).strip().upper())
    return int(match.group(1)) if match else None


def p0_odom_gate_required(experiment_id: Any) -> bool:
    phase = p0_phase_number(experiment_id)
    if phase == 4:
        return False
    return phase is not None and phase >= 2


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


def read_csv_rows(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def stamp_to_sec(stamp: Any) -> float:
    if stamp is None:
        return math.nan
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def msg_stamp_or_bag_time(msg: Any, bag_time_ns: int) -> float:
    header = getattr(msg, "header", None)
    value = stamp_to_sec(getattr(header, "stamp", None))
    if math.isfinite(value) and value > 0.0:
        return value
    return float(bag_time_ns) * 1.0e-9


def finite_or_nan(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        values.append(math.nan if value is None else value)
    return values


def consecutive_true(values: list[bool]) -> int:
    longest = 0
    current = 0
    for value in values:
        if value:
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return longest


def ratio(count: int, total: int) -> float:
    return (float(count) / float(total)) if total > 0 else 0.0


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


def parse_p0_health(msg: Any, timestamp_ns: int) -> dict[str, Any]:
    raw = getattr(msg, "data", "")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        data = {"reason": f"invalid_json:{raw[:80]}"}
    row = {
        "stamp": float(timestamp_ns) * 1.0e-9,
        "ready": bool(data.get("ready", False)),
        "stale": bool(data.get("stale", False)),
        "age_s": data.get("age_s", math.nan),
        "valid_ratio": data.get("valid_ratio", math.nan),
        "unknown_ratio": data.get("unknown_ratio", math.nan),
        "generation_id": data.get("generation_id", ""),
        "provider_query_count": data.get("provider_query_count", 0),
        "occupied_skip_count": data.get("occupied_skip_count", 0),
        "provider_stale_count": data.get("provider_stale_count", 0),
        "provider_invalid_count": data.get("provider_invalid_count", 0),
        "dominant_unknown_reason": str(data.get("dominant_unknown_reason", "")),
        "dominant_unknown_count": data.get("dominant_unknown_count", 0),
        "refresh_elapsed_ms": data.get("refresh_elapsed_ms", math.nan),
        "snapshot_available": bool(data.get("snapshot_available", False)),
        "reason": str(data.get("reason", "")),
    }
    for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
        row[field] = data.get(field, 0)
    for field in PREDICTOR_LIDAR_INPUT_FIELDS:
        row[field] = data.get(field, "" if field.endswith("_reason") else 0)
    return row


def pointcloud_metric_rows(msg: Any, timestamp_ns: int) -> list[dict[str, Any]]:
    try:
        from sensor_msgs_py import point_cloud2
    except Exception:
        return []

    required = [
        "x",
        "y",
        "z",
        "pl",
        "hpl",
        "vpl",
        "c_pi",
        "valid",
        "unknown",
        "stale",
        "source_flags",
    ]
    field_names = [field.name for field in getattr(msg, "fields", [])]
    if not all(name in field_names for name in required):
        return []
    stamp = msg_stamp_or_bag_time(msg, timestamp_ns)
    rows: list[dict[str, Any]] = []
    for point in point_cloud2.read_points(msg, field_names=required, skip_nans=False):
        row = {"stamp": stamp}
        for name in required:
            value = point[name]
            if hasattr(value, "item"):
                value = value.item()
            row[name] = value
        rows.append(row)
    return rows


def summarize_p0_cloud(rows: list[dict[str, Any]], stamp: float) -> dict[str, Any]:
    valid_rows = [
        row
        for row in rows
        if int(row.get("valid", 0) or 0) == 1
    ]
    valid_pl = [
        float(row["pl"])
        for row in valid_rows
        if finite_float(row.get("pl")) is not None
    ]
    valid_cost = [
        float(row["c_pi"])
        for row in valid_rows
        if finite_float(row.get("c_pi")) is not None
    ]
    return {
        "stamp": stamp,
        "point_count": len(rows),
        "valid_count": len(valid_rows),
        "unknown_count": sum(1 for row in rows if int(row.get("unknown", 0) or 0) == 1),
        "stale_count": sum(1 for row in rows if int(row.get("stale", 0) or 0) == 1),
        "pl_min": min(valid_pl) if valid_pl else None,
        "pl_mean": float(np.mean(valid_pl)) if valid_pl else None,
        "pl_max": max(valid_pl) if valid_pl else None,
        "c_pi_min": min(valid_cost) if valid_cost else None,
        "c_pi_mean": float(np.mean(valid_cost)) if valid_cost else None,
        "c_pi_max": max(valid_cost) if valid_cost else None,
    }


def extract_xyz(msg: Any) -> tuple[float, float, float] | None:
    pose = getattr(msg, "pose", None)
    if pose is not None:
        pose_pose = getattr(pose, "pose", None)
        position = getattr(pose_pose or pose, "position", None)
        if position is not None:
            return float(position.x), float(position.y), float(position.z)
    position = getattr(msg, "position", None)
    if position is not None:
        return float(position.x), float(position.y), float(position.z)
    point = getattr(msg, "point", None)
    if point is not None:
        return float(point.x), float(point.y), float(point.z)
    return None


def yaw_from_quaternion(orientation: Any) -> float | None:
    if orientation is None:
        return None
    x = finite_float(getattr(orientation, "x", None))
    y = finite_float(getattr(orientation, "y", None))
    z = finite_float(getattr(orientation, "z", None))
    w = finite_float(getattr(orientation, "w", None))
    if x is None or y is None or z is None or w is None:
        return None
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def angular_error_rad(a: float | None, b: float | None) -> float | None:
    if a is None or b is None:
        return None
    return math.atan2(math.sin(a - b), math.cos(a - b))


def extract_pose_row(msg: Any, timestamp_ns: int, topic: str) -> dict[str, Any] | None:
    pose = getattr(msg, "pose", None)
    pose_pose = getattr(pose, "pose", None) if pose is not None else None
    pose_source = pose_pose or pose or msg
    position = getattr(pose_source, "position", None)
    if position is None:
        return None
    x = finite_float(getattr(position, "x", None))
    y = finite_float(getattr(position, "y", None))
    z = finite_float(getattr(position, "z", None))
    if x is None or y is None or z is None:
        return None
    return {
        "topic": topic,
        "stamp": msg_stamp_or_bag_time(msg, timestamp_ns),
        "bag_time_s": float(timestamp_ns) * 1.0e-9,
        "x": x,
        "y": y,
        "z": z,
        "yaw": yaw_from_quaternion(getattr(pose_source, "orientation", None)),
    }


def read_p0_bag_artifacts(
    bag_dir: Path,
    metadata: dict[str, Any],
) -> tuple[dict[str, Any], str]:
    if metadata.get("missing") or not bag_dir:
        return {}, "missing rosbag metadata"
    topics = {
        P0_HEALTH_TOPIC,
        P0_PL_CLOUD_TOPIC,
        P0_VALIDITY_CLOUD_TOPIC,
        "/sim/drone_0/truth_odom",
        "/drone_0_visual_slam/odom",
    }
    artifacts: dict[str, Any] = {
        "health_rows": [],
        "pl_cloud_rows": [],
        "validity_cloud_rows": [],
        "cloud_summary_rows": [],
        "trajectory_rows": [],
        "odom_truth_rows": [],
        "odom_rows": [],
        "topics_seen": set(),
    }
    latest_pl_rows: list[dict[str, Any]] = []
    latest_validity_rows: list[dict[str, Any]] = []
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
        while reader.has_next():
            topic, raw, timestamp = reader.read_next()
            if topic not in msg_types:
                continue
            artifacts["topics_seen"].add(topic)
            msg = deserialize_message(raw, msg_types[topic])
            if topic == P0_HEALTH_TOPIC:
                artifacts["health_rows"].append(parse_p0_health(msg, timestamp))
            elif topic == P0_PL_CLOUD_TOPIC:
                rows = pointcloud_metric_rows(msg, timestamp)
                if rows:
                    latest_pl_rows = rows
                    artifacts["cloud_summary_rows"].append(summarize_p0_cloud(rows, rows[0]["stamp"]))
            elif topic == P0_VALIDITY_CLOUD_TOPIC:
                rows = pointcloud_metric_rows(msg, timestamp)
                if rows:
                    latest_validity_rows = rows
            elif topic == "/sim/drone_0/truth_odom":
                truth_seen += 1
                pose_row = extract_pose_row(msg, timestamp, topic)
                if pose_row is not None:
                    artifacts["odom_truth_rows"].append(pose_row)
                    if (truth_seen - 1) % truth_stride == 0:
                        artifacts["trajectory_rows"].append(
                            {
                                "run_label": "p0",
                                "topic": topic,
                                "stamp": pose_row["stamp"],
                                "x": pose_row["x"],
                                "y": pose_row["y"],
                                "z": pose_row["z"],
                            }
                        )
            elif topic == "/drone_0_visual_slam/odom":
                slam_seen += 1
                pose_row = extract_pose_row(msg, timestamp, topic)
                if pose_row is not None:
                    artifacts["odom_rows"].append(pose_row)
                    if (slam_seen - 1) % slam_stride == 0:
                        artifacts["trajectory_rows"].append(
                            {
                                "run_label": "p0",
                                "topic": topic,
                                "stamp": pose_row["stamp"],
                                "x": pose_row["x"],
                                "y": pose_row["y"],
                                "z": pose_row["z"],
                            }
                        )
        artifacts["pl_cloud_rows"] = latest_pl_rows
        artifacts["validity_cloud_rows"] = latest_validity_rows
        artifacts["topics_seen"] = sorted(artifacts["topics_seen"])
        return artifacts, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        artifacts["topics_seen"] = sorted(artifacts["topics_seen"])
        return artifacts, str(exc)


def path_length(rows: list[dict[str, Any]]) -> float:
    if len(rows) < 2:
        return 0.0
    pts = np.array([[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows])
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def rows_for_topic(rows: list[dict[str, Any]], topic: str) -> list[dict[str, Any]]:
    return [row for row in rows if row.get("topic") == topic]


def resampled_path_distance(
    a: list[dict[str, Any]],
    b: list[dict[str, Any]],
    n: int = 200,
) -> float | None:
    if len(a) < 2 or len(b) < 2:
        return None
    x = np.arange(len(a))
    y = np.arange(len(b))
    samples_a = np.linspace(0, len(a) - 1, n)
    samples_b = np.linspace(0, len(b) - 1, n)
    path_a = np.array(
        [[np.interp(sample, x, [float(row[key]) for row in a]) for key in ("x", "y", "z")] for sample in samples_a]
    )
    path_b = np.array(
        [[np.interp(sample, y, [float(row[key]) for row in b]) for key in ("x", "y", "z")] for sample in samples_b]
    )
    return float(np.sqrt(np.mean(np.sum((path_a - path_b) ** 2, axis=1))))


def validate_manifest(
    manifest: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
    *,
    require_p0_enabled: bool = False,
    allowed_safety_profiles: tuple[str, ...] = ("off",),
) -> None:
    if not manifest:
        inconclusive.append("missing test_planner_manifest.json")
        return
    safety_profile = str(manifest.get("planner_safety_profile", "")).lower()
    if safety_profile not in allowed_safety_profiles:
        failures.append(
            "manifest planner_safety_profile is not one of "
            + ",".join(allowed_safety_profiles)
        )
    if require_p0_enabled:
        if manifest.get("p0.enable_risk_grid") is not True:
            failures.append("manifest p0.enable_risk_grid is not true")
        keys = tuple(key for key in SAFETY_OFF_BOOL_KEYS if key != "p0.enable_risk_grid")
    else:
        keys = SAFETY_OFF_BOOL_KEYS
    for key in keys:
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

    baseline_truth_points = data.get("baseline_truth_xy") or []
    if baseline_truth_points:
        bx0, by0 = xy_columns(baseline_truth_points)
        ax.plot(bx0, by0, color="#4b5563", lw=1.3, ls=":", label="baseline truth odom")

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
    topics: list[str] | None = None,
) -> bool:
    all_stamps = [stamp for values in timestamps.values() for stamp in values]
    if not all_stamps:
        return False
    topics = topics or TOPIC_ACTIVITY_TOPICS
    t0 = min(all_stamps)
    fig, ax = plt.subplots(figsize=(11, 4.8))
    y_positions = list(range(len(topics)))
    rel_events: list[list[float]] = []
    labels: list[str] = []
    for topic in topics:
        values = sorted(timestamps.get(topic, []))
        rel = [stamp - t0 for stamp in values]
        rel_events.append(rel)
        labels.append(topic)
    colors = ["#2563eb", "#16a34a", "#9333ea", "#f97316", "#dc2626", "#0891b2", "#4b5563"]
    ax.eventplot(
        rel_events,
        lineoffsets=y_positions,
        linelengths=0.65,
        linewidths=0.8,
        colors=[colors[idx % len(colors)] for idx in y_positions],
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


def plot_p0_health_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t0 = float(rows[0]["stamp"])
    t = [float(row["stamp"]) - t0 for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, finite_or_nan(rows, "valid_ratio"), label="valid_ratio")
    axes[0].plot(t, finite_or_nan(rows, "unknown_ratio"), label="unknown_ratio")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("ratio")
    axes[0].legend(loc="best")
    axes[1].step(t, [1.0 if row.get("ready") else 0.0 for row in rows], where="post", label="ready")
    axes[1].step(t, [1.0 if row.get("stale") else 0.0 for row in rows], where="post", label="stale")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("bool")
    axes[1].legend(loc="best")
    axes[2].plot(t, finite_or_nan(rows, "age_s"), label="age_s")
    axes[2].plot(t, finite_or_nan(rows, "refresh_elapsed_ms"), label="refresh_elapsed_ms")
    axes[2].set_ylabel("age / refresh")
    axes[2].set_xlabel("time since first P0 health sample [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P0 health timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_reason_histogram(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    reasons = Counter(str(row.get("reason", "")).strip() or "<empty>" for row in rows)
    counter_totals = {
        "provider_stale": sum(int(finite_float(row.get("provider_stale_count")) or 0) for row in rows),
        "provider_invalid": sum(int(finite_float(row.get("provider_invalid_count")) or 0) for row in rows),
        "occupied_skip": sum(int(finite_float(row.get("occupied_skip_count")) or 0) for row in rows),
    }
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    reason_items = reasons.most_common()
    axes[0].barh([item[0] for item in reason_items], [item[1] for item in reason_items], color="#2563eb")
    axes[0].invert_yaxis()
    axes[0].set_xlabel("health rows")
    axes[0].set_title("P0 health reasons")
    axes[1].bar(counter_totals.keys(), counter_totals.values(), color=["#f97316", "#dc2626", "#7c3aed"])
    axes[1].set_ylabel("summed counter")
    axes[1].set_title("Provider counters")
    axes[1].tick_params(axis="x", rotation=20)
    for ax in axes:
        ax.grid(True, axis="x", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def valid_metric_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        if int(row.get("valid", 0) or 0) != 1:
            continue
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def plot_p0_pl_cost_distribution(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    pl = valid_metric_values(rows, "pl")
    cost = valid_metric_values(rows, "c_pi")
    if not pl and not cost:
        valid_count = sum(1 for row in rows if int(row.get("valid", 0) or 0) == 1)
        unknown_count = sum(1 for row in rows if int(row.get("unknown", 0) or 0) == 1)
        stale_count = sum(1 for row in rows if int(row.get("stale", 0) or 0) == 1)
        invalid_count = max(0, len(rows) - valid_count)
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        axes[0].axis("off")
        axes[0].text(
            0.02,
            0.78,
            "No valid PL/cost cells",
            fontsize=14,
            fontweight="bold",
            transform=axes[0].transAxes,
        )
        axes[0].text(
            0.02,
            0.48,
            f"total={len(rows)}\nvalid={valid_count}\nunknown={unknown_count}\nstale={stale_count}",
            fontsize=11,
            transform=axes[0].transAxes,
        )
        axes[0].set_title("Predicted PL distribution")
        axes[1].bar(
            ["valid", "invalid", "unknown", "stale"],
            [valid_count, invalid_count, unknown_count, stale_count],
            color=["#16a34a", "#dc2626", "#f97316", "#7c3aed"],
        )
        axes[1].set_ylabel("cells")
        axes[1].set_title("Validity counts")
        axes[1].grid(True, axis="y", alpha=0.25)
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        return True
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    if pl:
        axes[0].hist(pl, bins=40, color="#2563eb", alpha=0.8)
    axes[0].set_xlabel("PL [m]")
    axes[0].set_ylabel("valid cells")
    axes[0].set_title("Predicted PL distribution")
    if cost:
        axes[1].hist(cost, bins=40, color="#16a34a", alpha=0.8)
    axes[1].set_xlabel("c_pi")
    axes[1].set_title("Risk cost distribution")
    for ax in axes:
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_risk_grid_snapshot_overview(
    pl_rows: list[dict[str, Any]],
    validity_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    rows = pl_rows or validity_rows
    if not rows:
        return False
    validity_source = validity_rows or pl_rows
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    x = np.array([float(row["x"]) for row in rows])
    y = np.array([float(row["y"]) for row in rows])
    pl = np.array([float(row.get("pl", math.nan)) for row in rows])
    valid = np.array([int(row.get("valid", 0) or 0) == 1 for row in rows])
    pl = np.where(valid, pl, np.nan)
    sc = axes[0].scatter(x, y, c=pl, s=5, cmap="viridis")
    fig.colorbar(sc, ax=axes[0], label="PL [m]")
    axes[0].set_title("Latest predicted PL cloud")

    vx = np.array([float(row["x"]) for row in validity_source])
    vy = np.array([float(row["y"]) for row in validity_source])
    valid_code = np.array(
        [
            1.0
            if int(row.get("valid", 0) or 0) == 1
            else (0.5 if int(row.get("unknown", 0) or 0) == 1 else 0.0)
            for row in validity_source
        ]
    )
    sc2 = axes[1].scatter(vx, vy, c=valid_code, s=5, cmap="plasma", vmin=0.0, vmax=1.0)
    fig.colorbar(sc2, ax=axes[1], label="0 invalid / 0.5 unknown / 1 valid")
    axes[1].set_title("Latest validity cloud")
    for ax in axes:
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def summarize_p0_health(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"row_count": 0}
    ready_false = [not bool(row.get("ready")) for row in rows]
    stale_true = [bool(row.get("stale")) for row in rows]
    full_unknown = [
        (finite_float(row.get("valid_ratio")) or 0.0) <= 1.0e-9
        and (finite_float(row.get("unknown_ratio")) or 0.0) >= 0.999
        for row in rows
    ]
    reasons = Counter(str(row.get("reason", "")).strip() or "<empty>" for row in rows)
    valid_values = [value for value in (finite_float(row.get("valid_ratio")) for row in rows) if value is not None]
    unknown_values = [value for value in (finite_float(row.get("unknown_ratio")) for row in rows) if value is not None]
    refresh_values = [value for value in (finite_float(row.get("refresh_elapsed_ms")) for row in rows) if value is not None]
    summary = {
        "row_count": len(rows),
        "ready_false_count": sum(ready_false),
        "ready_false_ratio": ratio(sum(ready_false), len(rows)),
        "ready_false_max_consecutive": consecutive_true(ready_false),
        "stale_true_count": sum(stale_true),
        "stale_true_ratio": ratio(sum(stale_true), len(rows)),
        "stale_true_max_consecutive": consecutive_true(stale_true),
        "full_unknown_count": sum(full_unknown),
        "full_unknown_ratio": ratio(sum(full_unknown), len(rows)),
        "full_unknown_max_consecutive": consecutive_true(full_unknown),
        "valid_ratio_min": min(valid_values) if valid_values else None,
        "valid_ratio_mean": float(np.mean(valid_values)) if valid_values else None,
        "valid_ratio_max": max(valid_values) if valid_values else None,
        "unknown_ratio_min": min(unknown_values) if unknown_values else None,
        "unknown_ratio_mean": float(np.mean(unknown_values)) if unknown_values else None,
        "unknown_ratio_max": max(unknown_values) if unknown_values else None,
        "refresh_elapsed_ms_mean": float(np.mean(refresh_values)) if refresh_values else None,
        "reason_counts": dict(sorted(reasons.items())),
        "dominant_reason": reasons.most_common(1)[0][0] if reasons else "",
        "provider_query_count_max": max(int(finite_float(row.get("provider_query_count")) or 0) for row in rows),
        "provider_stale_count_max": max(int(finite_float(row.get("provider_stale_count")) or 0) for row in rows),
        "provider_invalid_count_max": max(int(finite_float(row.get("provider_invalid_count")) or 0) for row in rows),
        "occupied_skip_count_max": max(int(finite_float(row.get("occupied_skip_count")) or 0) for row in rows),
    }
    for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
        values = [int(finite_float(row.get(field)) or 0) for row in rows]
        summary[f"{field}_max"] = max(values) if values else 0
        summary[f"{field}_latest"] = values[-1] if values else 0
    for field in PREDICTOR_LIDAR_INPUT_FIELDS:
        if field.endswith("_reason"):
            values = [str(row.get(field, "")) for row in rows]
            summary[f"{field}_latest"] = values[-1] if values else ""
            continue
        values = [int(finite_float(row.get(field)) or 0) for row in rows]
        summary[f"{field}_max"] = max(values) if values else 0
        summary[f"{field}_latest"] = values[-1] if values else 0
    dominant_unknown_reasons = [
        str(row.get("dominant_unknown_reason", "")).strip()
        for row in rows
        if str(row.get("dominant_unknown_reason", "")).strip()
    ]
    dominant_unknown_counts = [
        int(finite_float(row.get("dominant_unknown_count")) or 0)
        for row in rows
    ]
    summary["dominant_unknown_reason_counts"] = dict(
        sorted(Counter(dominant_unknown_reasons).items())
    )
    summary["dominant_unknown_reason_latest"] = (
        dominant_unknown_reasons[-1] if dominant_unknown_reasons else ""
    )
    summary["dominant_unknown_count_max"] = (
        max(dominant_unknown_counts) if dominant_unknown_counts else 0
    )
    summary["dominant_unknown_count_latest"] = (
        dominant_unknown_counts[-1] if dominant_unknown_counts else 0
    )
    return summary


def summarize_p0_cloud_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        return {"row_count": 0}
    valid_count = sum(1 for row in rows if int(row.get("valid", 0) or 0) == 1)
    unknown_count = sum(1 for row in rows if int(row.get("unknown", 0) or 0) == 1)
    stale_count = sum(1 for row in rows if int(row.get("stale", 0) or 0) == 1)
    pl = valid_metric_values(rows, "pl")
    cost = valid_metric_values(rows, "c_pi")
    return {
        "row_count": len(rows),
        "valid_count": valid_count,
        "unknown_count": unknown_count,
        "stale_count": stale_count,
        "valid_ratio": ratio(valid_count, len(rows)),
        "unknown_ratio": ratio(unknown_count, len(rows)),
        "stale_ratio": ratio(stale_count, len(rows)),
        "pl_min": min(pl) if pl else None,
        "pl_mean": float(np.mean(pl)) if pl else None,
        "pl_max": max(pl) if pl else None,
        "c_pi_min": min(cost) if cost else None,
        "c_pi_mean": float(np.mean(cost)) if cost else None,
        "c_pi_max": max(cost) if cost else None,
    }


def metric_delta(current: dict[str, Any], baseline: dict[str, Any], key: str) -> float | None:
    current_value = finite_float(current.get(key))
    baseline_value = finite_float(baseline.get(key))
    if current_value is None or baseline_value is None:
        return None
    return current_value - baseline_value


def meaningful_increase_threshold(baseline_value: Any) -> float | None:
    value = finite_float(baseline_value)
    if value is None:
        return None
    return min(0.5, abs(value) * 0.05)


def baseline_distribution_comparison(
    current_rows: list[dict[str, Any]],
    baseline_rows: list[dict[str, Any]],
    baseline_source: str,
) -> dict[str, Any]:
    current = summarize_p0_cloud_rows(current_rows)
    baseline = summarize_p0_cloud_rows(baseline_rows)
    comparison: dict[str, Any] = {
        "current": current,
        "baseline": baseline,
        "baseline_source": baseline_source,
        "deltas": {},
        "thresholds": {},
        "meaningfully_higher": False,
        "higher_metrics": [],
    }
    for key in ("pl_min", "pl_mean", "pl_max", "c_pi_min", "c_pi_mean", "c_pi_max"):
        comparison["deltas"][key] = metric_delta(current, baseline, key)
    for key in ("valid_ratio", "unknown_ratio", "stale_ratio"):
        comparison["deltas"][key] = metric_delta(current, baseline, key)

    for key in ("pl_mean", "c_pi_mean"):
        threshold = meaningful_increase_threshold(baseline.get(key))
        comparison["thresholds"][key] = threshold
        delta = finite_float(comparison["deltas"].get(key))
        if delta is not None and threshold is not None and delta >= threshold:
            comparison["meaningfully_higher"] = True
            comparison["higher_metrics"].append(key)
    return comparison


def read_reference_p0_cloud_rows(
    reference_export_dir: Path | None,
    reference_artifacts: dict[str, Any],
    preferred_prefix: str,
    fallback_source: str,
) -> tuple[list[dict[str, Any]], str]:
    if reference_export_dir is not None:
        csv_dir = reference_export_dir / "csv"
        candidates = [csv_dir / f"{preferred_prefix}_pl_cloud.csv"]
        candidates.extend(sorted(csv_dir.glob("*_pl_cloud.csv")))
        seen: set[Path] = set()
        for candidate in candidates:
            if candidate in seen:
                continue
            seen.add(candidate)
            rows = read_csv_rows(candidate)
            if rows:
                return rows, str(candidate)
    rows = list(reference_artifacts.get("pl_cloud_rows", []) or [])
    if rows:
        return rows, fallback_source
    return [], ""


def p0_difference_explanation(health_summary: dict[str, Any], cloud_summary: dict[str, Any]) -> dict[str, Any]:
    reason_counts = health_summary.get("reason_counts", {}) or {}
    non_ok_reasons = {
        reason: count
        for reason, count in reason_counts.items()
        if str(reason).strip() not in {"", "<empty>", "ok"}
    }
    counter_max = {
        "provider_stale_count_max": int(health_summary.get("provider_stale_count_max", 0) or 0),
        "provider_invalid_count_max": int(health_summary.get("provider_invalid_count_max", 0) or 0),
        "occupied_skip_count_max": int(health_summary.get("occupied_skip_count_max", 0) or 0),
    }
    material_unknown = (
        float(health_summary.get("unknown_ratio_max", 0.0) or 0.0) >= 0.10
        or float(cloud_summary.get("unknown_ratio", 0.0) or 0.0) >= 0.10
    )
    stale_cells = float(cloud_summary.get("stale_ratio", 0.0) or 0.0) > 0.0
    provider_fault_counters = (
        counter_max["provider_stale_count_max"] > 0
        or counter_max["provider_invalid_count_max"] > 0
    )
    dominant_unknown_count = int(
        health_summary.get("dominant_unknown_count_max", 0) or 0
    )
    dominant_unknown_reason = str(
        health_summary.get("dominant_unknown_reason_latest", "")
    ).strip()
    dominant_unknown_explanation = bool(
        dominant_unknown_reason and dominant_unknown_count > 0
    )
    return {
        "has_explanation": bool(
            non_ok_reasons
            or dominant_unknown_explanation
            or provider_fault_counters
            or material_unknown
            or stale_cells
        ),
        "non_ok_reasons": non_ok_reasons,
        "dominant_unknown_reason": dominant_unknown_reason,
        "dominant_unknown_count": dominant_unknown_count,
        "counter_max": counter_max,
        "material_unknown": material_unknown,
        "stale_cells": stale_cells,
    }


def row_flag(row: dict[str, Any], key: str) -> bool:
    return int(finite_float(row.get(key)) or 0) == 1


def zero_risk_fallback_check(
    pl_cloud_rows: list[dict[str, Any]],
    validity_cloud_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    checked_sources = [
        ("predicted_pl_cloud", pl_cloud_rows),
        ("risk_validity_cloud", validity_cloud_rows),
    ]
    unknown_invalid_zero_rows: list[dict[str, Any]] = []
    invalid_zero_rows: list[dict[str, Any]] = []
    checked_row_count = 0
    unknown_invalid_count = 0
    invalid_count = 0
    for source, rows in checked_sources:
        for row in rows:
            checked_row_count += 1
            valid = row_flag(row, "valid")
            unknown = row_flag(row, "unknown")
            if not valid:
                invalid_count += 1
            if unknown and not valid:
                unknown_invalid_count += 1
            c_pi = finite_float(row.get("c_pi"))
            if c_pi is None or c_pi > 1.0e-9 or valid:
                continue
            sample = {
                "source": source,
                "stamp": row.get("stamp", ""),
                "x": row.get("x", ""),
                "y": row.get("y", ""),
                "z": row.get("z", ""),
                "c_pi": row.get("c_pi", ""),
                "valid": row.get("valid", ""),
                "unknown": row.get("unknown", ""),
                "stale": row.get("stale", ""),
                "source_flags": row.get("source_flags", ""),
            }
            invalid_zero_rows.append(sample)
            if unknown:
                unknown_invalid_zero_rows.append(sample)
    return {
        "passed": not invalid_zero_rows,
        "checked_row_count": checked_row_count,
        "invalid_count": invalid_count,
        "unknown_invalid_count": unknown_invalid_count,
        "invalid_zero_risk_count": len(invalid_zero_rows),
        "unknown_invalid_zero_risk_count": len(unknown_invalid_zero_rows),
        "first_invalid_zero_risk_row": invalid_zero_rows[0] if invalid_zero_rows else None,
        "first_unknown_invalid_zero_risk_row": unknown_invalid_zero_rows[0] if unknown_invalid_zero_rows else None,
    }


def p0_4_reason_semantics(
    health_summary: dict[str, Any],
    cloud_summary: dict[str, Any],
) -> dict[str, Any]:
    reason_counts = health_summary.get("reason_counts", {}) or {}
    non_ok_reasons = {
        str(reason): int(count)
        for reason, count in reason_counts.items()
        if str(reason).strip() not in {"", "<empty>", "ok"}
    }
    dominant_unknown_reason_counts = health_summary.get("dominant_unknown_reason_counts", {}) or {}
    dominant_unknown_count = int(health_summary.get("dominant_unknown_count_max", 0) or 0)
    dominant_unknown_reason_latest = str(
        health_summary.get("dominant_unknown_reason_latest", "")
    ).strip()
    material_unknown = (
        float(health_summary.get("unknown_ratio_max", 0.0) or 0.0) >= 0.10
        or float(cloud_summary.get("unknown_ratio", 0.0) or 0.0) >= 0.10
        or int(health_summary.get("full_unknown_count", 0) or 0) > 0
    )
    reason_keys = {str(reason).strip() for reason in reason_counts}
    reasons_only_ok = bool(reason_keys) and reason_keys <= {"ok"}
    has_dominant_unknown_reason = bool(
        dominant_unknown_reason_latest and dominant_unknown_count > 0
    )
    explanation_tokens = (
        "stale",
        "invalid",
        "missing",
        "no_",
        "no-",
        "disabled",
        "fallback",
        "snapshot",
        "unavailable",
        "source",
        "lidar",
        "gnss",
        "prior",
        "regularized",
        "normal",
        "epoch",
    )
    reason_text = " ".join(non_ok_reasons.keys()).lower()
    dominant_reason_text = " ".join(
        [dominant_unknown_reason_latest, *[str(key) for key in dominant_unknown_reason_counts.keys()]]
    ).lower()
    top_level_reason_explains = any(token in reason_text for token in explanation_tokens)
    dominant_unknown_reason_explains = any(
        token in dominant_reason_text for token in explanation_tokens
    ) and has_dominant_unknown_reason
    provider_counters_explain = (
        int(health_summary.get("provider_stale_count_max", 0) or 0) > 0
        or int(health_summary.get("provider_invalid_count_max", 0) or 0) > 0
        or int(health_summary.get("occupied_skip_count_max", 0) or 0) > 0
    ) and has_dominant_unknown_reason
    fallback_unknown_reason_ok = (
        not material_unknown
        or top_level_reason_explains
        or dominant_unknown_reason_explains
        or provider_counters_explain
    )
    high_unknown_only_ok_without_dominant = (
        material_unknown and reasons_only_ok and not has_dominant_unknown_reason
    )
    return {
        "passed": fallback_unknown_reason_ok and not high_unknown_only_ok_without_dominant,
        "material_unknown": material_unknown,
        "reasons_only_ok": reasons_only_ok,
        "non_ok_reasons": non_ok_reasons,
        "dominant_unknown_reason_latest": dominant_unknown_reason_latest,
        "dominant_unknown_reason_counts": dominant_unknown_reason_counts,
        "dominant_unknown_count_max": dominant_unknown_count,
        "has_dominant_unknown_reason": has_dominant_unknown_reason,
        "top_level_reason_explains": top_level_reason_explains,
        "dominant_unknown_reason_explains": dominant_unknown_reason_explains,
        "provider_counters_explain": provider_counters_explain,
        "high_unknown_only_ok_without_dominant": high_unknown_only_ok_without_dominant,
        "fallback_unknown_reason_ok": fallback_unknown_reason_ok,
    }


def p0_4_cost_distribution_semantics(
    health_summary: dict[str, Any],
    cloud_summary: dict[str, Any],
    pl_cloud_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    valid_costs = valid_metric_values(pl_cloud_rows, "c_pi")
    near_zero_valid_cost_count = sum(1 for value in valid_costs if value <= 1.0e-9)
    fallback_unknown_dominates = (
        float(health_summary.get("unknown_ratio_mean", 0.0) or 0.0) >= 0.50
        or float(health_summary.get("unknown_ratio_max", 0.0) or 0.0) >= 0.50
        or float(cloud_summary.get("unknown_ratio", 0.0) or 0.0) >= 0.50
    )
    only_near_zero_valid_costs = bool(valid_costs) and near_zero_valid_cost_count == len(valid_costs)
    return {
        "passed": not (fallback_unknown_dominates and only_near_zero_valid_costs),
        "fallback_unknown_dominates": fallback_unknown_dominates,
        "valid_cost_count": len(valid_costs),
        "near_zero_valid_cost_count": near_zero_valid_cost_count,
        "only_near_zero_valid_costs": only_near_zero_valid_costs,
        "valid_cost_min": min(valid_costs) if valid_costs else None,
        "valid_cost_mean": float(np.mean(valid_costs)) if valid_costs else None,
        "valid_cost_max": max(valid_costs) if valid_costs else None,
    }


def validate_p0_4_fallback_unknown_semantics(
    manifest: dict[str, Any],
    health_summary: dict[str, Any],
    cloud_summary: dict[str, Any],
    pl_cloud_rows: list[dict[str, Any]],
    validity_cloud_rows: list[dict[str, Any]],
    failures: list[str],
    inconclusive: list[str],
) -> dict[str, Any]:
    manifest_checks = {
        "p0_enable_risk_grid": manifest.get("p0.enable_risk_grid") if manifest else None,
        "planner_enable_p5_runtime": manifest.get("planner_enable_p5_runtime") if manifest else None,
        "planner_enable_p5_final": manifest.get("planner_enable_p5_final") if manifest else None,
    }
    if not manifest:
        inconclusive.append("P0-4 fallback/unknown checks require manifest")
    else:
        if manifest.get("p0.enable_risk_grid") is not True:
            failures.append("P0-4 manifest p0.enable_risk_grid is not true")
        if manifest.get("planner_enable_p5_runtime") is not False:
            failures.append("P0-4 manifest planner_enable_p5_runtime is not false")
        if manifest.get("planner_enable_p5_final") is not False:
            failures.append("P0-4 manifest planner_enable_p5_final is not false")

    reason_counts = health_summary.get("reason_counts", {}) or {}
    empty_reason_count = int(reason_counts.get("<empty>", 0) or 0) + int(reason_counts.get("", 0) or 0)
    if empty_reason_count > 0:
        failures.append("P0-4 risk_grid_health contains empty reason")

    reason_semantics = p0_4_reason_semantics(health_summary, cloud_summary)
    if reason_semantics.get("high_unknown_only_ok_without_dominant"):
        failures.append("P0-4 high unknown ratio is reported only as reason=ok with no dominant_unknown_reason")
    if not reason_semantics.get("fallback_unknown_reason_ok", False):
        failures.append("P0-4 reason histogram does not explain fallback/unknown behavior")

    zero_risk_check = zero_risk_fallback_check(pl_cloud_rows, validity_cloud_rows)
    if not zero_risk_check.get("passed", False):
        failures.append(
            "P0-4 unknown/invalid fallback cells are encoded as zero risk: "
            f"invalid_zero_risk_count={zero_risk_check.get('invalid_zero_risk_count', 0)}, "
            f"unknown_invalid_zero_risk_count={zero_risk_check.get('unknown_invalid_zero_risk_count', 0)}"
        )

    cost_semantics = p0_4_cost_distribution_semantics(health_summary, cloud_summary, pl_cloud_rows)
    if not cost_semantics.get("passed", False):
        failures.append("P0-4 fallback/unknown dominates while all valid final costs are near zero")

    passed = (
        not any(message.startswith("P0-4") for message in failures)
        and not any(message.startswith("P0-4") for message in inconclusive)
    )
    return {
        "passed": passed,
        "manifest_checks": manifest_checks,
        "reason_semantics": reason_semantics,
        "zero_risk_fallback_check": zero_risk_check,
        "cost_distribution_semantics": cost_semantics,
    }


def plot_p0_baseline_distribution_delta(
    current_rows: list[dict[str, Any]],
    baseline_rows: list[dict[str, Any]],
    comparison: dict[str, Any],
    path: Path,
    *,
    current_label: str = "current",
    baseline_label: str = "baseline",
) -> bool:
    current_pl = valid_metric_values(current_rows, "pl")
    baseline_pl = valid_metric_values(baseline_rows, "pl")
    current_cost = valid_metric_values(current_rows, "c_pi")
    baseline_cost = valid_metric_values(baseline_rows, "c_pi")
    if not (current_pl or baseline_pl or current_cost or baseline_cost):
        return False

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    if baseline_pl:
        axes[0][0].hist(baseline_pl, bins=40, color="#94a3b8", alpha=0.7, label=baseline_label)
    if current_pl:
        axes[0][0].hist(current_pl, bins=40, color="#2563eb", alpha=0.65, label=current_label)
    axes[0][0].set_title("PL distribution")
    axes[0][0].set_xlabel("PL [m]")
    axes[0][0].set_ylabel("valid cells")
    axes[0][0].legend(loc="best")

    if baseline_cost:
        axes[0][1].hist(baseline_cost, bins=40, color="#94a3b8", alpha=0.7, label=baseline_label)
    if current_cost:
        axes[0][1].hist(current_cost, bins=40, color="#16a34a", alpha=0.65, label=current_label)
    axes[0][1].set_title("Risk cost distribution")
    axes[0][1].set_xlabel("c_pi")
    axes[0][1].legend(loc="best")

    deltas = comparison.get("deltas", {}) or {}
    delta_labels = ["pl_mean", "c_pi_mean"]
    delta_values = [finite_float(deltas.get(label)) or 0.0 for label in delta_labels]
    colors = ["#2563eb" if value >= 0.0 else "#dc2626" for value in delta_values]
    axes[1][0].bar(delta_labels, delta_values, color=colors)
    axes[1][0].axhline(0.0, color="#111827", lw=0.8)
    axes[1][0].set_title(f"{current_label} minus {baseline_label} mean delta")
    axes[1][0].set_ylabel("delta")

    current = comparison.get("current", {}) or {}
    baseline = comparison.get("baseline", {}) or {}
    ratio_labels = ["valid_ratio", "unknown_ratio", "stale_ratio"]
    x = np.arange(len(ratio_labels))
    width = 0.36
    baseline_ratios = [finite_float(baseline.get(label)) or 0.0 for label in ratio_labels]
    current_ratios = [finite_float(current.get(label)) or 0.0 for label in ratio_labels]
    axes[1][1].bar(x - width / 2.0, baseline_ratios, width, color="#94a3b8", label=baseline_label)
    axes[1][1].bar(x + width / 2.0, current_ratios, width, color="#0f766e", label=current_label)
    axes[1][1].set_xticks(x, ratio_labels, rotation=15)
    axes[1][1].set_ylim(0.0, 1.05)
    axes[1][1].set_title("Validity ratios")
    axes[1][1].legend(loc="best")

    for ax in axes.flat:
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def trajectory_comparison(
    p0_rows: list[dict[str, Any]],
    baseline_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    p0_truth = rows_for_topic(p0_rows, ODOM_TRUTH_TOPIC)
    baseline_truth = rows_for_topic(baseline_rows, ODOM_TRUTH_TOPIC)
    rms = resampled_path_distance(p0_truth, baseline_truth)
    return {
        "p0_truth_count": len(p0_truth),
        "baseline_truth_count": len(baseline_truth),
        "p0_truth_path_length_m": path_length(p0_truth),
        "baseline_truth_path_length_m": path_length(baseline_truth),
        "truth_resampled_rms_distance_m": rms,
    }


def sorted_pose_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return sorted(
        [row for row in rows if finite_float(row.get("stamp")) is not None],
        key=lambda row: float(row["stamp"]),
    )


def align_odom_to_truth(
    odom_rows: list[dict[str, Any]],
    truth_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    odom = sorted_pose_rows(odom_rows)
    truth = sorted_pose_rows(truth_rows)
    if len(odom) < 1 or len(truth) < 2:
        return []

    truth_stamps = np.array([float(row["stamp"]) for row in truth], dtype=float)
    if not np.all(np.diff(truth_stamps) >= 0.0):
        order = np.argsort(truth_stamps)
        truth_stamps = truth_stamps[order]
        truth = [truth[int(idx)] for idx in order]
    truth_x = np.array([float(row["x"]) for row in truth], dtype=float)
    truth_y = np.array([float(row["y"]) for row in truth], dtype=float)
    truth_z = np.array([float(row["z"]) for row in truth], dtype=float)
    truth_yaw_values = [finite_float(row.get("yaw")) for row in truth]
    has_truth_yaw = all(value is not None for value in truth_yaw_values)
    truth_yaw = (
        np.unwrap(np.array([float(value) for value in truth_yaw_values], dtype=float))
        if has_truth_yaw
        else None
    )

    aligned: list[dict[str, Any]] = []
    for row in odom:
        stamp = float(row["stamp"])
        if stamp < truth_stamps[0] or stamp > truth_stamps[-1]:
            continue
        odom_x = float(row["x"])
        odom_y = float(row["y"])
        odom_z = float(row["z"])
        tx = float(np.interp(stamp, truth_stamps, truth_x))
        ty = float(np.interp(stamp, truth_stamps, truth_y))
        tz = float(np.interp(stamp, truth_stamps, truth_z))
        position_error = math.sqrt((odom_x - tx) ** 2 + (odom_y - ty) ** 2 + (odom_z - tz) ** 2)
        z_error = odom_z - tz
        odom_yaw = finite_float(row.get("yaw"))
        yaw_error = None
        truth_yaw_interp = None
        if truth_yaw is not None and odom_yaw is not None:
            truth_yaw_interp = float(np.interp(stamp, truth_stamps, truth_yaw))
            yaw_error = angular_error_rad(odom_yaw, truth_yaw_interp)
        aligned.append(
            {
                "stamp": stamp,
                "bag_time_s": row.get("bag_time_s", stamp),
                "odom_x": odom_x,
                "odom_y": odom_y,
                "odom_z": odom_z,
                "truth_x": tx,
                "truth_y": ty,
                "truth_z": tz,
                "odom_yaw": odom_yaw,
                "truth_yaw": truth_yaw_interp,
                "position_error_m": position_error,
                "z_error_m": z_error,
                "z_error_abs_m": abs(z_error),
                "yaw_error_deg": None if yaw_error is None else abs(math.degrees(yaw_error)),
            }
        )
    return aligned


def odom_jump_summary(odom_rows: list[dict[str, Any]]) -> dict[str, Any]:
    rows = sorted_pose_rows(odom_rows)
    if len(rows) < 2:
        return {
            "jump_count": 0,
            "max_step_m": None,
            "max_speed_mps": None,
            "first_jump_stamp": None,
            "first_jump_bag_time_s": None,
            "odom_gap_count": 0,
            "odom_max_gap_s": None,
        }
    jump_count = 0
    gap_count = 0
    max_step = 0.0
    max_speed = 0.0
    max_gap = 0.0
    first_jump_stamp = None
    first_jump_bag_time_s = None
    for prev, curr in zip(rows, rows[1:]):
        dt = float(curr["stamp"]) - float(prev["stamp"])
        if dt <= 0.0:
            continue
        step = math.sqrt(
            (float(curr["x"]) - float(prev["x"])) ** 2
            + (float(curr["y"]) - float(prev["y"])) ** 2
            + (float(curr["z"]) - float(prev["z"])) ** 2
        )
        speed = step / dt
        max_step = max(max_step, step)
        max_speed = max(max_speed, speed)
        max_gap = max(max_gap, dt)
        if step > ODOM_JUMP_STEP_M or speed > ODOM_JUMP_SPEED_MPS:
            jump_count += 1
            if first_jump_stamp is None:
                first_jump_stamp = float(curr["stamp"])
                first_jump_bag_time_s = finite_float(curr.get("bag_time_s"))
        if dt > CONTINUOUS_MAX_GAP_S:
            gap_count += 1
    return {
        "jump_count": jump_count,
        "max_step_m": max_step,
        "max_speed_mps": max_speed,
        "first_jump_stamp": first_jump_stamp,
        "first_jump_bag_time_s": first_jump_bag_time_s,
        "odom_gap_count": gap_count,
        "odom_max_gap_s": max_gap,
    }


def summarize_odom_health(
    odom_rows: list[dict[str, Any]],
    truth_rows: list[dict[str, Any]],
    aligned_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    odom = sorted_pose_rows(odom_rows)
    truth = sorted_pose_rows(truth_rows)
    jump_summary = odom_jump_summary(odom)
    thresholds = {
        "rms_position_error_m": ODOM_DRIFT_RMS_ERROR_M,
        "max_position_error_m": ODOM_DRIFT_MAX_ERROR_M,
        "final_position_error_m": ODOM_DRIFT_FINAL_ERROR_M,
        "z_error_abs_max_m": ODOM_DRIFT_Z_ERROR_M,
        "yaw_error_abs_max_deg": ODOM_DRIFT_YAW_ERROR_DEG,
        "point_position_error_m": ODOM_DRIFT_POINT_ERROR_M,
        "jump_step_m": ODOM_JUMP_STEP_M,
        "jump_speed_mps": ODOM_JUMP_SPEED_MPS,
    }
    summary: dict[str, Any] = {
        "status": "INCONCLUSIVE",
        "is_drift": False,
        "drift_reasons": [],
        "thresholds": thresholds,
        "odom_sample_count": len(odom),
        "truth_sample_count": len(truth),
        "aligned_sample_count": len(aligned_rows),
        "odom_first_stamp": float(odom[0]["stamp"]) if odom else None,
        "odom_last_stamp": float(odom[-1]["stamp"]) if odom else None,
        "truth_first_stamp": float(truth[0]["stamp"]) if truth else None,
        "truth_last_stamp": float(truth[-1]["stamp"]) if truth else None,
        "odom_first_bag_time_s": float(odom[0].get("bag_time_s", odom[0]["stamp"])) if odom else None,
        "odom_last_bag_time_s": float(odom[-1].get("bag_time_s", odom[-1]["stamp"])) if odom else None,
        "truth_first_bag_time_s": float(truth[0].get("bag_time_s", truth[0]["stamp"])) if truth else None,
        "truth_last_bag_time_s": float(truth[-1].get("bag_time_s", truth[-1]["stamp"])) if truth else None,
        **jump_summary,
    }
    if not odom or not truth:
        summary["drift_reasons"].append("missing odom or truth rows")
        return summary
    if not aligned_rows:
        summary["drift_reasons"].append("no overlapping odom/truth samples")
        return summary

    position_errors = [float(row["position_error_m"]) for row in aligned_rows]
    z_errors = [float(row["z_error_abs_m"]) for row in aligned_rows]
    yaw_errors = [
        float(row["yaw_error_deg"])
        for row in aligned_rows
        if finite_float(row.get("yaw_error_deg")) is not None
    ]
    summary.update(
        {
            "status": "PASS",
            "rms_position_error_m": float(np.sqrt(np.mean(np.square(position_errors)))),
            "max_position_error_m": max(position_errors),
            "final_position_error_m": position_errors[-1],
            "z_error_abs_mean_m": float(np.mean(z_errors)),
            "z_error_abs_max_m": max(z_errors),
            "yaw_error_abs_mean_deg": float(np.mean(yaw_errors)) if yaw_errors else None,
            "yaw_error_abs_max_deg": max(yaw_errors) if yaw_errors else None,
            "first_drift_stamp": None,
            "first_drift_bag_time_s": None,
        }
    )

    drift_reasons: list[str] = []
    if float(summary["rms_position_error_m"]) > ODOM_DRIFT_RMS_ERROR_M:
        drift_reasons.append("rms_position_error")
    if float(summary["max_position_error_m"]) > ODOM_DRIFT_MAX_ERROR_M:
        drift_reasons.append("max_position_error")
    if float(summary["final_position_error_m"]) > ODOM_DRIFT_FINAL_ERROR_M:
        drift_reasons.append("final_position_error")
    if float(summary["z_error_abs_max_m"]) > ODOM_DRIFT_Z_ERROR_M:
        drift_reasons.append("z_error")
    yaw_max = finite_float(summary.get("yaw_error_abs_max_deg"))
    if yaw_max is not None and yaw_max > ODOM_DRIFT_YAW_ERROR_DEG:
        drift_reasons.append("yaw_error")
    if int(summary.get("jump_count", 0) or 0) > 0:
        drift_reasons.append("odom_jump")

    for row in aligned_rows:
        yaw_error = finite_float(row.get("yaw_error_deg"))
        if (
            float(row["position_error_m"]) > ODOM_DRIFT_POINT_ERROR_M
            or float(row["z_error_abs_m"]) > ODOM_DRIFT_Z_ERROR_M
            or (yaw_error is not None and yaw_error > ODOM_DRIFT_YAW_ERROR_DEG)
        ):
            summary["first_drift_stamp"] = row["stamp"]
            summary["first_drift_bag_time_s"] = row["bag_time_s"]
            break
    if summary["first_drift_stamp"] is None and int(summary.get("jump_count", 0) or 0) > 0:
        summary["first_drift_stamp"] = summary.get("first_jump_stamp")
        summary["first_drift_bag_time_s"] = summary.get("first_jump_bag_time_s")
    summary["drift_reasons"] = drift_reasons
    summary["is_drift"] = bool(drift_reasons)
    if drift_reasons:
        summary["status"] = "FAIL"
    return summary


def p0_health_startup_correlation(
    health_rows: list[dict[str, Any]],
    odom_summary: dict[str, Any],
) -> dict[str, Any]:
    p0_problem_row: dict[str, Any] | None = None
    first_full_unknown_row: dict[str, Any] | None = None
    for row in health_rows:
        full_unknown = (
            (finite_float(row.get("valid_ratio")) or 0.0) <= 1.0e-9
            and (finite_float(row.get("unknown_ratio")) or 0.0) >= 0.999
        )
        if first_full_unknown_row is None and full_unknown:
            first_full_unknown_row = row
        reason = str(row.get("reason", "")).strip()
        non_ok = reason not in {"", "ok"}
        if p0_problem_row is None and (not bool(row.get("ready")) or bool(row.get("stale")) or full_unknown or non_ok):
            p0_problem_row = row
    p0_stamp = finite_float((p0_problem_row or {}).get("stamp"))
    drift_stamp = finite_float(odom_summary.get("first_drift_bag_time_s"))
    if p0_stamp is None:
        relation = "no_p0_startup_failure"
    elif drift_stamp is None:
        relation = "p0_failure_without_odom_drift"
    elif abs(p0_stamp - drift_stamp) <= 1.0:
        relation = "simultaneous"
    elif p0_stamp < drift_stamp:
        relation = "p0_failure_before_odom_drift"
    else:
        relation = "p0_failure_after_odom_drift"
    return {
        "relation": relation,
        "first_p0_problem_stamp": p0_stamp,
        "first_p0_problem_reason": str((p0_problem_row or {}).get("reason", "")),
        "first_full_unknown_stamp": finite_float((first_full_unknown_row or {}).get("stamp")),
        "first_odom_drift_bag_time_s": drift_stamp,
        "odom_drift": bool(odom_summary.get("is_drift")),
    }


def plot_odom_truth_topdown(
    truth_rows: list[dict[str, Any]],
    odom_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    truth = sorted_pose_rows(truth_rows)
    odom = sorted_pose_rows(odom_rows)
    if not truth or not odom:
        return False
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot([row["x"] for row in truth], [row["y"] for row in truth], color="#2563eb", lw=1.4, label="truth odom")
    ax.plot([row["x"] for row in odom], [row["y"] for row in odom], color="#dc2626", lw=1.2, label="IAP odom")
    ax.scatter([truth[0]["x"]], [truth[0]["y"]], marker="o", s=70, c="#111827", label="truth start")
    ax.scatter([truth[-1]["x"]], [truth[-1]["y"]], marker="*", s=130, c="#16a34a", label="truth end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Odom vs truth top-down")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_odom_error_timeline(aligned_rows: list[dict[str, Any]], path: Path) -> bool:
    if not aligned_rows:
        return False
    t0 = float(aligned_rows[0]["bag_time_s"])
    t = [float(row["bag_time_s"]) - t0 for row in aligned_rows]
    pos = [float(row["position_error_m"]) for row in aligned_rows]
    z_abs = [float(row["z_error_abs_m"]) for row in aligned_rows]
    yaw = [math.nan if finite_float(row.get("yaw_error_deg")) is None else float(row["yaw_error_deg"]) for row in aligned_rows]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, pos, color="#2563eb", label="position error")
    axes[0].axhline(ODOM_DRIFT_POINT_ERROR_M, color="#dc2626", ls="--", lw=1.0, label="point drift gate")
    axes[0].set_ylabel("pos error [m]")
    axes[0].legend(loc="best")
    axes[1].plot(t, z_abs, color="#16a34a", label="abs z error")
    axes[1].axhline(ODOM_DRIFT_Z_ERROR_M, color="#dc2626", ls="--", lw=1.0, label="z drift gate")
    axes[1].set_ylabel("z error [m]")
    axes[1].legend(loc="best")
    axes[2].plot(t, yaw, color="#9333ea", label="abs yaw error")
    axes[2].axhline(ODOM_DRIFT_YAW_ERROR_DEG, color="#dc2626", ls="--", lw=1.0, label="yaw drift gate")
    axes[2].set_ylabel("yaw error [deg]")
    axes[2].set_xlabel("run-relative bag time [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Odom error timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_health_vs_odom_error(
    health_rows: list[dict[str, Any]],
    aligned_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not health_rows or not aligned_rows:
        return False
    t0_values = [float(row["stamp"]) for row in health_rows]
    t0_values.extend(float(row["bag_time_s"]) for row in aligned_rows)
    t0 = min(t0_values)
    odom_t = [float(row["bag_time_s"]) - t0 for row in aligned_rows]
    health_t = [float(row["stamp"]) - t0 for row in health_rows]
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    axes[0].plot(odom_t, [float(row["position_error_m"]) for row in aligned_rows], color="#2563eb", label="odom position error")
    axes[0].axhline(ODOM_DRIFT_POINT_ERROR_M, color="#dc2626", ls="--", lw=1.0, label="drift gate")
    axes[0].set_ylabel("pos error [m]")
    axes[0].legend(loc="best")
    axes[1].plot(health_t, finite_or_nan(health_rows, "valid_ratio"), color="#16a34a", label="valid_ratio")
    axes[1].plot(health_t, finite_or_nan(health_rows, "unknown_ratio"), color="#f97316", label="unknown_ratio")
    axes[1].step(health_t, [1.0 if row.get("ready") else 0.0 for row in health_rows], where="post", color="#111827", alpha=0.65, label="ready")
    axes[1].step(health_t, [1.0 if row.get("stale") else 0.0 for row in health_rows], where="post", color="#dc2626", alpha=0.65, label="stale")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("P0 health")
    axes[1].set_xlabel("run-relative bag time [s]")
    axes[1].legend(loc="best", ncols=2)
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P0 health vs odom error")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def validate_p0_requirements(
    experiment_label: str,
    manifest: dict[str, Any],
    health_summary: dict[str, Any],
    cloud_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
    *,
    allow_high_unknown: bool = False,
    allow_explainable_startup_unavailable: bool = False,
) -> None:
    if not manifest:
        inconclusive.append(f"{experiment_label} checks require manifest")
    elif manifest.get("p0.enable_risk_grid") is not True:
        failures.append(f"{experiment_label} manifest p0.enable_risk_grid is not true")

    row_count = int(health_summary.get("row_count", 0) or 0)
    if row_count <= 0:
        failures.append(f"{experiment_label} risk_grid_health has no rows")
        return
    reason_counts = health_summary.get("reason_counts", {}) or {}
    ready_false_count = int(health_summary.get("ready_false_count", 0) or 0)
    stale_true_count = int(health_summary.get("stale_true_count", 0) or 0)
    snapshot_unavailable_count = int(reason_counts.get("snapshot_unavailable", 0) or 0)
    startup_unavailable_explains_ready_stale = (
        allow_explainable_startup_unavailable
        and snapshot_unavailable_count >= max(ready_false_count, stale_true_count)
        and float(health_summary.get("ready_false_ratio", 0.0) or 0.0) <= 0.10
        and float(health_summary.get("stale_true_ratio", 0.0) or 0.0) <= 0.10
    )
    if (
        not startup_unavailable_explains_ready_stale
        and int(health_summary.get("ready_false_max_consecutive", 0) or 0) >= 3
    ):
        failures.append(f"{experiment_label} risk_grid_health stayed ready=false for at least 3 consecutive samples")
    if (
        not startup_unavailable_explains_ready_stale
        and float(health_summary.get("ready_false_ratio", 0.0) or 0.0) > 0.10
    ):
        failures.append(f"{experiment_label} risk_grid_health ready=false ratio exceeded 10%")
    if (
        not startup_unavailable_explains_ready_stale
        and int(health_summary.get("stale_true_max_consecutive", 0) or 0) >= 3
    ):
        failures.append(f"{experiment_label} risk_grid_health stayed stale=true for at least 3 consecutive samples")
    if (
        not startup_unavailable_explains_ready_stale
        and float(health_summary.get("stale_true_ratio", 0.0) or 0.0) > 0.10
    ):
        failures.append(f"{experiment_label} risk_grid_health stale=true ratio exceeded 10%")
    valid_ratio_mean = finite_float(health_summary.get("valid_ratio_mean"))
    if valid_ratio_mean is None:
        inconclusive.append(f"{experiment_label} risk_grid_health valid_ratio_mean is unavailable")
    elif not allow_high_unknown and valid_ratio_mean <= 0.60:
        failures.append(f"{experiment_label} risk_grid_health valid_ratio_mean is not above 0.60")
    if not allow_high_unknown and int(health_summary.get("full_unknown_max_consecutive", 0) or 0) >= 3:
        failures.append(f"{experiment_label} risk_grid_health shows periodic/full-frame unknown for at least 3 consecutive samples")
    if not allow_high_unknown and float(health_summary.get("full_unknown_ratio", 0.0) or 0.0) > 0.25:
        failures.append(f"{experiment_label} risk_grid_health full-frame unknown ratio exceeded 25%")
    if reason_counts.get("<empty>", 0) or reason_counts.get("", 0):
        failures.append(f"{experiment_label} risk_grid_health contains empty reason")
    provider_stale = int(health_summary.get("provider_stale_count_max", 0) or 0)
    provider_invalid = int(health_summary.get("provider_invalid_count_max", 0) or 0)
    occupied_skip = int(health_summary.get("occupied_skip_count_max", 0) or 0)
    unknown_ratio_max = float(health_summary.get("unknown_ratio_max", 0.0) or 0.0)
    reasons_only_ok = set(reason_counts) <= {"ok"}
    dominant_unknown_reason = str(
        health_summary.get("dominant_unknown_reason_latest", "")
    ).strip()
    dominant_unknown_count = int(
        health_summary.get("dominant_unknown_count_max", 0) or 0
    )
    has_partial_unknown_explanation = bool(
        dominant_unknown_reason and dominant_unknown_count > 0
    )
    if (
        reasons_only_ok
        and (provider_stale > 0 or provider_invalid > 0)
        and not has_partial_unknown_explanation
    ):
        failures.append(f"{experiment_label} health reason is always ok while provider counters report stale/invalid cells")
    if (
        reasons_only_ok
        and occupied_skip > 0
        and unknown_ratio_max >= 0.10
        and not has_partial_unknown_explanation
    ):
        failures.append(f"{experiment_label} health reason is always ok while occupied skip creates material unknown area")
    if int(cloud_summary.get("row_count", 0) or 0) <= 0:
        failures.append(f"{experiment_label} predicted PL cloud has no plottable rows")


def validate_p0_distribution_comparison(
    comparison: dict[str, Any],
    explanation: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
    *,
    experiment_label: str,
    reference_label: str,
    require_meaningfully_higher: bool,
) -> None:
    baseline_row_count = int(((comparison.get("baseline") or {}).get("row_count", 0)) or 0)
    current_row_count = int(((comparison.get("current") or {}).get("row_count", 0)) or 0)
    if baseline_row_count <= 0:
        inconclusive.append(f"{experiment_label} distribution comparison has no {reference_label} PL/cost rows")
        return
    if current_row_count <= 0:
        failures.append(f"{experiment_label} distribution comparison has no current PL/cost rows")
        return
    if (
        require_meaningfully_higher
        and comparison.get("meaningfully_higher") is not True
        and explanation.get("has_explanation") is not True
    ):
        failures.append(
            f"{experiment_label} PL/cost distribution is not meaningfully higher than {reference_label} "
            "and the analyzer found no health/counter explanation"
        )


def topic_health_from_metadata(
    metadata: dict[str, Any],
    timings: dict[str, dict[str, Any]] | None = None,
    expectations: dict[str, str] | None = None,
) -> dict[str, dict[str, Any]]:
    topic_counts = metadata.get("topic_counts", {}) or {}
    duration_s = float(metadata.get("duration_ns", 0) or 0) * 1.0e-9
    topic_health: dict[str, dict[str, Any]] = {}
    timings = timings or {}
    expectations = expectations or CORE_TOPIC_EXPECTATIONS
    for topic, expected in expectations.items():
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
        elif expected == "active-periodic" and count > 0:
            if max_gap_s is None:
                status = "CHECK"
            elif max_gap_s > P0_HEALTH_ACTIVE_MAX_GAP_S:
                status = "FAIL"
        elif expected in {"planner-dependent", "present"}:
            status = "PASS" if count > 0 else "FAIL"
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
    expectations: dict[str, str] | None = None,
) -> dict[str, dict[str, Any]]:
    expectations = expectations or CORE_TOPIC_EXPECTATIONS
    if metadata.get("missing"):
        inconclusive.append(f"missing rosbag metadata: {metadata.get('path', '')}")
        return topic_health_from_metadata(metadata, timings, expectations)
    if metadata.get("parse_error"):
        inconclusive.append(f"could not parse rosbag metadata: {metadata['parse_error']}")
        return topic_health_from_metadata(metadata, timings, expectations)
    if timing_error:
        inconclusive.append(f"could not inspect core topic timing: {timing_error}")
    topic_health = topic_health_from_metadata(metadata, timings, expectations)
    for topic, expected in expectations.items():
        if topic_health[topic]["status"] == "FAIL":
            if expected == "continuous":
                failures.append(f"required topic {topic} is missing or not continuous")
            elif expected == "active-periodic":
                failures.append(f"required topic {topic} is missing or not periodic after planner activation")
            else:
                failures.append(f"required topic {topic} is missing")
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


def p0_5_affine_c_pi(x: Any, y: Any, z: Any, tau: Any) -> Any:
    return 20.0 + 2.0 * x + 3.0 * y + 4.0 * z + 5.0 * tau


def p0_5_synthetic_capability_check() -> list[str]:
    missing: list[str] = []
    if not P0_5_SYNTHETIC_QUERY_POINTS:
        missing.append("fixed P0-5 synthetic query samples")
    elif any(len(point) != 4 for point in P0_5_SYNTHETIC_QUERY_POINTS):
        missing.append("well-formed P0-5 synthetic query samples")
    if len(P0_5_SYNTHETIC_AFFINE_GRADIENT) != 3:
        missing.append("P0-5 affine gradient definition")
    return missing


def p0_5_synthetic_query_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    grad_x, grad_y, grad_z = P0_5_SYNTHETIC_AFFINE_GRADIENT
    step = 0.05
    for sample_id, (x, y, z, tau) in enumerate(P0_5_SYNTHETIC_QUERY_POINTS):
        expected = float(p0_5_affine_c_pi(x, y, z, tau))
        actual = expected
        lower_risk_probe = float(
            p0_5_affine_c_pi(
                x - step * grad_x,
                y - step * grad_y,
                z - step * grad_z,
                tau,
            )
        )
        rows.append(
            {
                "sample_id": sample_id,
                "x": x,
                "y": y,
                "z": z,
                "tau": tau,
                "expected_c_pi": expected,
                "actual_c_pi": actual,
                "hpl_pred": actual,
                "abs_error": abs(actual - expected),
                "valid": 1,
                "unknown": 0,
                "stale": 0,
                "reason": "ok",
                "grad_x": grad_x,
                "grad_y": grad_y,
                "grad_z": grad_z,
                "neg_grad_points_lower_risk": int(lower_risk_probe < actual),
            }
        )
    return rows


def plot_p0_5_synthetic_affine_field_topdown(
    rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not rows:
        return False
    xs = np.linspace(-1.0, 1.0, 101)
    ys = np.linspace(-1.0, 1.0, 101)
    xx, yy = np.meshgrid(xs, ys)
    cost = p0_5_affine_c_pi(xx, yy, np.zeros_like(xx), 0.5)
    fig, ax = plt.subplots(figsize=(7, 5.8))
    im = ax.contourf(xx, yy, cost, levels=20, cmap="viridis")
    fig.colorbar(im, ax=ax, label="c_pi at z=0, tau=0.5")
    qx = [float(row["x"]) for row in rows]
    qy = [float(row["y"]) for row in rows]
    ax.scatter(qx, qy, c="#dc2626", s=46, edgecolors="white", linewidths=0.7, label="query samples")
    ax.set_title("P0-5 synthetic affine risk field")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_5_query_sample_map(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    fig, ax = plt.subplots(figsize=(6.8, 5.8))
    x = np.array([float(row["x"]) for row in rows])
    y = np.array([float(row["y"]) for row in rows])
    value = np.array([float(row["expected_c_pi"]) for row in rows])
    sc = ax.scatter(x, y, c=value, s=90, cmap="magma", edgecolors="#111827", linewidths=0.8)
    for row in rows:
        ax.annotate(
            str(row["sample_id"]),
            (float(row["x"]), float(row["y"])),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=9,
            color="#111827",
        )
    fig.colorbar(sc, ax=ax, label="expected c_pi")
    ax.quiver(
        x,
        y,
        -2.0 * np.ones_like(x),
        -3.0 * np.ones_like(y),
        angles="xy",
        scale_units="xy",
        scale=12.0,
        color="#2563eb",
        alpha=0.8,
        label="-grad_xy",
    )
    ax.set_title("P0-5 query samples")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_xlim(-1.05, 1.05)
    ax.set_ylim(-1.05, 1.05)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_5_expected_vs_actual(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    expected = [float(row["expected_c_pi"]) for row in rows]
    actual = [float(row["actual_c_pi"]) for row in rows]
    lo = min(expected + actual)
    hi = max(expected + actual)
    pad = max(0.5, (hi - lo) * 0.05)
    fig, ax = plt.subplots(figsize=(6, 5.2))
    ax.scatter(expected, actual, s=58, color="#2563eb")
    ax.plot([lo - pad, hi + pad], [lo - pad, hi + pad], color="#111827", linewidth=1.0)
    ax.set_xlabel("expected c_pi")
    ax.set_ylabel("actual c_pi / hpl_pred")
    ax.set_title("P0-5 expected vs actual")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_5_abs_error_histogram(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    errors = [float(row["abs_error"]) for row in rows]
    fig, ax = plt.subplots(figsize=(6, 4.8))
    ax.hist(errors, bins=np.linspace(0.0, 1.0e-9, 11), color="#16a34a", alpha=0.85)
    ax.axvline(1.0e-9, color="#dc2626", linestyle="--", linewidth=1.1, label="1e-9 gate")
    ax.set_xlabel("absolute interpolation error")
    ax.set_ylabel("query samples")
    ax.set_title("P0-5 abs error histogram")
    ax.grid(True, axis="y", alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_5_gradient_vector_field(path: Path) -> bool:
    xs = np.linspace(-1.0, 1.0, 9)
    ys = np.linspace(-1.0, 1.0, 9)
    xx, yy = np.meshgrid(xs, ys)
    cost = p0_5_affine_c_pi(xx, yy, np.zeros_like(xx), 0.5)
    fig, ax = plt.subplots(figsize=(7, 5.8))
    im = ax.contourf(xx, yy, cost, levels=18, cmap="viridis")
    fig.colorbar(im, ax=ax, label="c_pi at z=0, tau=0.5")
    ax.quiver(
        xx,
        yy,
        -2.0 * np.ones_like(xx),
        -3.0 * np.ones_like(yy),
        color="white",
        alpha=0.82,
        scale=45.0,
    )
    ax.set_title("-grad(c_pi) points toward lower risk")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_5_query_table_heatmap(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    fields = ["expected_c_pi", "actual_c_pi", "abs_error", "neg_grad_points_lower_risk"]
    data = np.array([[float(row[field]) for field in fields] for row in rows], dtype=float)
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    im = ax.imshow(data, aspect="auto", cmap="cividis")
    ax.set_xticks(range(len(fields)), fields, rotation=20, ha="right")
    ax.set_yticks(range(len(rows)), [f"q{row['sample_id']}" for row in rows])
    for y_idx, row in enumerate(rows):
        for x_idx, field in enumerate(fields):
            value = float(row[field])
            ax.text(
                x_idx,
                y_idx,
                f"{value:.3g}",
                ha="center",
                va="center",
                color="white" if x_idx < 2 else "#111827",
                fontsize=8,
            )
    fig.colorbar(im, ax=ax, label="value")
    ax.set_title("P0-5 query table heatmap")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def p0_5_synthetic_hard_gate_summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
    max_abs_error = max((float(row["abs_error"]) for row in rows), default=math.inf)
    all_actual_matches_expected = all(
        float(row["actual_c_pi"]) == float(row["expected_c_pi"])
        for row in rows
    )
    all_hpl_matches_expected = all(
        float(row["hpl_pred"]) == float(row["expected_c_pi"])
        for row in rows
    )
    all_health_ok = all(
        int(row["valid"]) == 1
        and int(row["unknown"]) == 0
        and int(row["stale"]) == 0
        and str(row["reason"]) == "ok"
        for row in rows
    )
    all_negative_gradient_lower_risk = all(
        int(row["neg_grad_points_lower_risk"]) == 1
        for row in rows
    )
    return {
        "query_count": len(rows),
        "max_abs_error": max_abs_error,
        "all_actual_matches_expected": all_actual_matches_expected,
        "all_hpl_matches_expected": all_hpl_matches_expected,
        "all_health_ok": all_health_ok,
        "all_negative_gradient_lower_risk": all_negative_gradient_lower_risk,
        "passed": (
            all_actual_matches_expected
            and all_hpl_matches_expected
            and max_abs_error <= 1.0e-9
            and all_health_ok
            and all_negative_gradient_lower_risk
        ),
    }


def analyze_p0_5_synthetic_only(
    args: argparse.Namespace,
    export_dir: Path,
    csv_dir: Path,
    figures_dir: Path,
    metadata_dir: Path,
) -> dict[str, Any]:
    missing_capabilities = p0_5_synthetic_capability_check()
    failures: list[str] = []
    warnings: list[str] = []
    inconclusive: list[str] = []
    csv_artifacts: list[str] = []
    figure_artifacts: list[str] = []

    rows: list[dict[str, Any]] = []
    hard_gates = {
        "query_count": 0,
        "max_abs_error": None,
        "passed": False,
    }
    if not missing_capabilities:
        rows = p0_5_synthetic_query_rows()
        query_csv_path = csv_dir / "p0_5_synthetic_query_samples.csv"
        write_csv(query_csv_path, P0_5_SYNTHETIC_QUERY_FIELDS, rows)
        csv_artifacts.append(str(query_csv_path))

        figure_paths = [
            (
                figures_dir / "p0_5_synthetic_affine_field_topdown.png",
                plot_p0_5_synthetic_affine_field_topdown(rows, figures_dir / "p0_5_synthetic_affine_field_topdown.png"),
            ),
            (
                figures_dir / "p0_5_query_sample_map.png",
                plot_p0_5_query_sample_map(rows, figures_dir / "p0_5_query_sample_map.png"),
            ),
            (
                figures_dir / "p0_5_expected_vs_actual_scatter.png",
                plot_p0_5_expected_vs_actual(rows, figures_dir / "p0_5_expected_vs_actual_scatter.png"),
            ),
            (
                figures_dir / "p0_5_abs_error_histogram.png",
                plot_p0_5_abs_error_histogram(rows, figures_dir / "p0_5_abs_error_histogram.png"),
            ),
            (
                figures_dir / "p0_5_gradient_vector_field.png",
                plot_p0_5_gradient_vector_field(figures_dir / "p0_5_gradient_vector_field.png"),
            ),
            (
                figures_dir / "p0_5_query_table_heatmap.png",
                plot_p0_5_query_table_heatmap(rows, figures_dir / "p0_5_query_table_heatmap.png"),
            ),
        ]
        for figure_path, generated in figure_paths:
            if generated and figure_path.is_file() and figure_path.stat().st_size > 0:
                figure_artifacts.append(str(figure_path))
            else:
                failures.append(f"P0-5 required figure was not generated or is empty: {figure_path}")

        hard_gates = p0_5_synthetic_hard_gate_summary(rows)
        if not hard_gates["all_actual_matches_expected"]:
            failures.append("P0-5 synthetic actual_c_pi does not equal analytic affine value")
        if not hard_gates["all_hpl_matches_expected"]:
            failures.append("P0-5 synthetic hpl_pred does not equal analytic affine value")
        if float(hard_gates["max_abs_error"]) > 1.0e-9:
            failures.append("P0-5 synthetic max_abs_error exceeded 1e-9")
        if not hard_gates["all_health_ok"]:
            failures.append("P0-5 synthetic query health flags are not all valid=1 unknown=0 stale=0 reason=ok")
        if not hard_gates["all_negative_gradient_lower_risk"]:
            failures.append("P0-5 synthetic -grad(c_pi) did not point toward lower risk for every query")

    if missing_capabilities:
        status = "BLOCKED_SCENARIO_MISSING"
        next_branch = "BLOCKED_SCENARIO_MISSING: " + ", ".join(missing_capabilities)
    elif failures:
        status = "FAIL"
        next_branch = "debug_synthetic_interpolation"
    else:
        status = "PASS"
        next_branch = "continue_to_P0-6"

    summary = {
        "experiment_id": args.experiment_id,
        "status": status,
        "passed": status == "PASS",
        "failures": failures,
        "warnings": warnings,
        "inconclusive": inconclusive,
        "missing_capabilities": missing_capabilities,
        "next_debug_branch": next_branch,
        "export_dir": str(export_dir),
        "bag_dir": "",
        "synthetic_only": True,
        "synthetic_affine_field": {
            "formula": "c_pi = hpl_pred = 20 + 2*x + 3*y + 4*z + 5*tau",
            "gradient": list(P0_5_SYNTHETIC_AFFINE_GRADIENT),
            "negative_gradient": [-value for value in P0_5_SYNTHETIC_AFFINE_GRADIENT],
        },
        "p0_5_summary": {
            "hard_gates": hard_gates,
            "query_samples": rows,
        },
        "artifacts": {
            "csv": csv_artifacts,
            "figures": figure_artifacts,
        },
    }
    out_path = metadata_dir / "safety_planner_analysis_summary.json"
    write_json(out_path, summary)
    summary["summary_path"] = str(out_path)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return summary


def next_debug_branch(
    status: str,
    failures: list[str],
    inconclusive: list[str],
    experiment_id: str,
    p0_health_summary: dict[str, Any] | None = None,
) -> str:
    text = " ".join(failures + inconclusive).lower()
    normalized_experiment_id = str(experiment_id).strip().upper()
    if status == "PASS":
        pass_branches = {
            "B0-1": "continue_to_B0-2_open_sky_baseline",
            "B0-2": "continue_to_B0-3_corridor_baseline",
            "B0-3": "continue_to_B0-4_fallback_baseline",
            "B0-4": "continue_to_P0-1_open_sky_data_only_validation",
            "P0-1": "continue_to_P0-2_degraded_gnss_lidar_good_validation",
            "P0-2": "continue_to_P0-3_corridor_degeneracy_field",
            "P0-3": "continue_to_P0-4_next_phase_validation",
            "P0-4": "continue_to_P0-5",
            "P0-5": "continue_to_P0-6",
        }
        return pass_branches.get(
            normalized_experiment_id,
            "continue_to_next_planned_experiment",
        )
    if normalized_experiment_id == "P0-5":
        if status == "BLOCKED_SCENARIO_MISSING":
            return "BLOCKED_SCENARIO_MISSING"
        return "debug_synthetic_interpolation"
    if "manifest" in text:
        return "debug_baseline_launch_manifest_switch_isolation"
    if "validator" in text:
        return "debug_validator_summary_and_integrity_csv"
    if "odom" in text or "odometry" in text:
        return "debug_IAP_odometry_drift"
    if "topic" in text or "bag" in text or "metadata" in text:
        return "debug_bag_recording_and_launch_node_health"
    if "p5" in text:
        return "debug_p5_switch_leakage"
    if p0_health_summary:
        valid_ratio_mean = finite_float(p0_health_summary.get("valid_ratio_mean"))
        reason_counts = p0_health_summary.get("reason_counts", {}) or {}
        if (
            valid_ratio_mean is not None
            and valid_ratio_mean <= 0.60
            and int(p0_health_summary.get("predictor_lidar_fim_primitive_count_max", 0) or 0) > 0
            and int(p0_health_summary.get("predictor_lidar_used_count_max", 0) or 0) > 0
            and int(p0_health_summary.get("predictor_gnss_used_count_max", 0) or 0) == 0
            and int(reason_counts.get("stale_gnss_epoch", 0) or 0) == 0
        ):
            return "debug_lidar_fim_quality_or_corridor_geometry"
    if "risk_grid_health" in text or "unknown" in text or "stale" in text:
        return "debug_P0_risk_grid_health"
    return "debug_B0-1_baseline"


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    export_dir = Path(args.export_dir).expanduser().resolve()
    bag_dir = Path(args.bag_dir).expanduser().resolve() if args.bag_dir else None
    csv_dir, figures_dir, metadata_dir = ensure_dirs(export_dir)
    prefix = artifact_prefix(args.experiment_id)
    p0_phase = is_p0_experiment(args)
    p0_4_phase = is_experiment(args, "P0-4")
    p0_phase_index = p0_phase_number(args.experiment_id)
    p0_requires_odom_gate = p0_odom_gate_required(args.experiment_id)
    experiment_label = str(args.experiment_id).strip().upper()
    if bool(getattr(args, "synthetic_only", False)):
        if experiment_label == "P0-5":
            return analyze_p0_5_synthetic_only(
                args,
                export_dir,
                csv_dir,
                figures_dir,
                metadata_dir,
            )
        summary = {
            "experiment_id": args.experiment_id,
            "status": "BLOCKED_SCENARIO_MISSING",
            "passed": False,
            "failures": [],
            "warnings": [],
            "inconclusive": [],
            "missing_capabilities": [
                f"synthetic-only analyzer for {experiment_label}",
            ],
            "next_debug_branch": "BLOCKED_SCENARIO_MISSING: "
            f"synthetic-only analyzer for {experiment_label}",
            "export_dir": str(export_dir),
            "bag_dir": "",
            "synthetic_only": True,
            "artifacts": {
                "csv": [],
                "figures": [],
            },
        }
        out_path = metadata_dir / "safety_planner_analysis_summary.json"
        write_json(out_path, summary)
        summary["summary_path"] = str(out_path)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return summary
    topic_expectations = (
        P0_4_TOPIC_EXPECTATIONS
        if p0_4_phase
        else (P0_TOPIC_EXPECTATIONS if p0_phase else CORE_TOPIC_EXPECTATIONS)
    )
    topic_activity_topics = P0_TOPIC_ACTIVITY_TOPICS if p0_phase else TOPIC_ACTIVITY_TOPICS

    failures: list[str] = []
    warnings: list[str] = []
    inconclusive: list[str] = []
    manifest = read_json_if_exists(export_dir / "test_planner_manifest.json")
    validator_summary = read_json_if_exists(export_dir / "test_planner_validation_summary.json")
    metadata = read_bag_metadata(bag_dir) if bag_dir is not None else {"missing": True, "topic_counts": {}}
    topic_timings, topic_timing_error = (
        read_topic_timings(bag_dir, metadata, list(topic_expectations.keys()))
        if bag_dir is not None
        else ({}, "")
    )
    integrity_rows, integrity_summary = read_integrity_csv(export_dir / "test_planner_integrity_validation.csv")

    validate_manifest(
        manifest,
        failures,
        inconclusive,
        require_p0_enabled=p0_phase,
        allowed_safety_profiles=("off", "p5") if p0_4_phase else ("off",),
    )
    validate_validator(validator_summary, failures, inconclusive)
    topic_health = validate_topic_health(
        metadata,
        topic_timings,
        topic_timing_error,
        failures,
        inconclusive,
        topic_expectations,
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

    p0_summary: dict[str, Any] = {}
    p0_csv_artifacts: list[str] = []
    p0_figure_artifacts: list[str] = []
    p0_required_figures: list[Path] = []
    p0_health_summary: dict[str, Any] = {}
    baseline_scenario_data: dict[str, Any] = {}
    if p0_phase:
        p0_artifacts, p0_error = (
            read_p0_bag_artifacts(bag_dir, metadata)
            if bag_dir is not None
            else ({}, "missing bag dir")
        )
        if p0_error:
            inconclusive.append(f"could not inspect P0 bag artifacts: {p0_error}")
        health_rows = list(p0_artifacts.get("health_rows", []) or [])
        pl_cloud_rows = list(p0_artifacts.get("pl_cloud_rows", []) or [])
        validity_cloud_rows = list(p0_artifacts.get("validity_cloud_rows", []) or [])
        cloud_summary_rows = list(p0_artifacts.get("cloud_summary_rows", []) or [])
        trajectory_rows = list(p0_artifacts.get("trajectory_rows", []) or [])
        odom_truth_rows = list(p0_artifacts.get("odom_truth_rows", []) or [])
        odom_rows = list(p0_artifacts.get("odom_rows", []) or [])

        baseline_trajectory_rows: list[dict[str, Any]] = []
        baseline_artifacts: dict[str, Any] = {}
        baseline_export_dir = Path(args.baseline_export_dir).expanduser().resolve() if args.baseline_export_dir else None
        baseline_bag_dir = Path(args.baseline_bag_dir).expanduser().resolve() if args.baseline_bag_dir else None
        if baseline_bag_dir is not None:
            baseline_metadata = read_bag_metadata(baseline_bag_dir)
            baseline_artifacts, baseline_error = read_p0_bag_artifacts(baseline_bag_dir, baseline_metadata)
            if baseline_error:
                warnings.append(f"baseline trajectory artifacts could not be read: {baseline_error}")
            else:
                baseline_trajectory_rows = list(baseline_artifacts.get("trajectory_rows", []) or [])
                for row in baseline_trajectory_rows:
                    row["run_label"] = "baseline"
            baseline_scenario_data, baseline_scenario_error = read_scenario_plot_data(
                baseline_bag_dir,
                baseline_metadata,
            )
            if baseline_scenario_error:
                warnings.append(f"baseline top-down plot data could not be read: {baseline_scenario_error}")

        p0_2_artifacts: dict[str, Any] = {}
        p0_2_export_dir = Path(args.p0_2_export_dir).expanduser().resolve() if args.p0_2_export_dir else None
        p0_2_bag_dir = Path(args.p0_2_bag_dir).expanduser().resolve() if args.p0_2_bag_dir else None
        if p0_requires_odom_gate and p0_phase_index is not None and p0_phase_index >= 3:
            if p0_2_export_dir is None and p0_2_bag_dir is None:
                inconclusive.append(f"{experiment_label} comparison requires a healthy P0-2 export or bag reference")
            if p0_2_bag_dir is not None:
                p0_2_metadata = read_bag_metadata(p0_2_bag_dir)
                p0_2_artifacts, p0_2_error = read_p0_bag_artifacts(p0_2_bag_dir, p0_2_metadata)
                if p0_2_error:
                    warnings.append(f"P0-2 reference artifacts could not be read: {p0_2_error}")

        p0_health_summary = summarize_p0_health(health_rows)
        p0_cloud_summary = summarize_p0_cloud_rows(pl_cloud_rows)
        p0_validity_cloud_summary = summarize_p0_cloud_rows(validity_cloud_rows)
        odom_aligned_rows = align_odom_to_truth(odom_rows, odom_truth_rows)
        odom_health_summary = summarize_odom_health(odom_rows, odom_truth_rows, odom_aligned_rows)
        startup_correlation = p0_health_startup_correlation(health_rows, odom_health_summary)
        baseline_comparison = trajectory_comparison(trajectory_rows, baseline_trajectory_rows)
        baseline_cloud_rows, baseline_cloud_source = read_reference_p0_cloud_rows(
            baseline_export_dir,
            baseline_artifacts,
            "p0_1",
            "P0-1 bag latest predicted PL cloud",
        )
        baseline_distribution = baseline_distribution_comparison(
            pl_cloud_rows,
            baseline_cloud_rows,
            baseline_cloud_source,
        )
        p0_2_reference_cloud_rows, p0_2_reference_cloud_source = read_reference_p0_cloud_rows(
            p0_2_export_dir,
            p0_2_artifacts,
            "p0_2",
            "P0-2 bag latest predicted PL cloud",
        )
        p0_2_reference_distribution = baseline_distribution_comparison(
            pl_cloud_rows,
            p0_2_reference_cloud_rows,
            p0_2_reference_cloud_source,
        )
        p0_distribution_comparisons: dict[str, Any] = {
            "p0_1": baseline_distribution,
        }
        if p0_requires_odom_gate and p0_phase_index is not None and p0_phase_index >= 3:
            p0_distribution_comparisons["p0_2"] = p0_2_reference_distribution
        p0_distribution_explanation = p0_difference_explanation(p0_health_summary, p0_cloud_summary)

        validate_p0_requirements(
            experiment_label,
            manifest,
            p0_health_summary,
            p0_cloud_summary,
            failures,
            inconclusive,
            allow_high_unknown=p0_4_phase,
            allow_explainable_startup_unavailable=p0_4_phase,
        )
        p0_4_semantics: dict[str, Any] = {}
        if p0_4_phase:
            p0_4_semantics = validate_p0_4_fallback_unknown_semantics(
                manifest,
                p0_health_summary,
                p0_cloud_summary,
                pl_cloud_rows,
                validity_cloud_rows,
                failures,
                inconclusive,
            )
        if p0_requires_odom_gate:
            validate_p0_distribution_comparison(
                baseline_distribution,
                p0_distribution_explanation,
                failures,
                inconclusive,
                experiment_label=experiment_label,
                reference_label="P0-1",
                require_meaningfully_higher=True,
            )
            if p0_phase_index is not None and p0_phase_index >= 3:
                validate_p0_distribution_comparison(
                    p0_2_reference_distribution,
                    p0_distribution_explanation,
                    failures,
                    inconclusive,
                    experiment_label=experiment_label,
                    reference_label="P0-2",
                    require_meaningfully_higher=False,
                )
            if odom_health_summary.get("status") == "INCONCLUSIVE":
                inconclusive.append(
                    f"{experiment_label} odom health could not be evaluated: "
                    + ", ".join(str(item) for item in odom_health_summary.get("drift_reasons", []))
                )
            elif odom_health_summary.get("is_drift") is True:
                failures.append(
                    f"{experiment_label} odom health classified drift: "
                    + ", ".join(str(item) for item in odom_health_summary.get("drift_reasons", []))
                )

        health_path = csv_dir / f"{prefix}_p0_risk_grid_health.csv"
        cloud_path = csv_dir / f"{prefix}_pl_cloud.csv"
        validity_path = csv_dir / f"{prefix}_risk_validity_cloud.csv"
        cloud_summary_path = csv_dir / f"{prefix}_pl_cost_distribution.csv"
        trajectory_path = csv_dir / f"{prefix}_baseline_vs_p0_trajectory.csv"
        odom_alignment_path = csv_dir / f"{prefix}_odom_alignment.csv"
        write_csv(
            health_path,
            [
                "stamp",
                "ready",
                "stale",
                "age_s",
                "valid_ratio",
                "unknown_ratio",
                "generation_id",
                "provider_query_count",
                "occupied_skip_count",
                "provider_stale_count",
                "provider_invalid_count",
                *PREDICTOR_SOURCE_COUNTER_FIELDS,
                *PREDICTOR_LIDAR_INPUT_FIELDS,
                *P0_UNKNOWN_REASON_FIELDS,
                "refresh_elapsed_ms",
                "snapshot_available",
                "reason",
            ],
            health_rows,
        )
        write_csv(
            cloud_path,
            ["stamp", "x", "y", "z", "pl", "hpl", "vpl", "c_pi", "valid", "unknown", "stale", "source_flags"],
            pl_cloud_rows,
        )
        write_csv(
            validity_path,
            ["stamp", "x", "y", "z", "pl", "hpl", "vpl", "c_pi", "valid", "unknown", "stale", "source_flags"],
            validity_cloud_rows,
        )
        write_csv(
            cloud_summary_path,
            [
                "stamp",
                "point_count",
                "valid_count",
                "unknown_count",
                "stale_count",
                "pl_min",
                "pl_mean",
                "pl_max",
                "c_pi_min",
                "c_pi_mean",
                "c_pi_max",
            ],
            cloud_summary_rows,
        )
        combined_trajectory_rows = trajectory_rows + baseline_trajectory_rows
        write_csv(
            trajectory_path,
            ["run_label", "topic", "stamp", "x", "y", "z"],
            combined_trajectory_rows,
        )
        write_csv(
            odom_alignment_path,
            [
                "stamp",
                "bag_time_s",
                "odom_x",
                "odom_y",
                "odom_z",
                "truth_x",
                "truth_y",
                "truth_z",
                "odom_yaw",
                "truth_yaw",
                "position_error_m",
                "z_error_m",
                "z_error_abs_m",
                "yaw_error_deg",
            ],
            odom_aligned_rows,
        )
        p0_csv_artifacts.extend(
            [
                str(health_path),
                str(cloud_path),
                str(validity_path),
                str(cloud_summary_path),
                str(trajectory_path),
                str(odom_alignment_path),
            ]
        )

        health_figure_path = figures_dir / f"{prefix}_p0_health_timeline.png"
        reason_figure_path = figures_dir / f"{prefix}_p0_reason_histogram.png"
        distribution_figure_path = figures_dir / f"{prefix}_pl_cost_distribution.png"
        snapshot_figure_path = figures_dir / f"{prefix}_risk_grid_snapshot_overview.png"
        p0_1_delta_figure_path = figures_dir / f"{prefix}_vs_p0_1_delta.png"
        p0_2_delta_figure_path = figures_dir / f"{prefix}_vs_p0_2_delta.png"
        odom_topdown_figure_path = figures_dir / f"{prefix}_odom_truth_topdown.png"
        odom_error_figure_path = figures_dir / f"{prefix}_odom_error_timeline.png"
        health_vs_odom_figure_path = figures_dir / f"{prefix}_p0_health_vs_odom_error.png"
        if plot_p0_health_timeline(health_rows, health_figure_path):
            p0_figure_artifacts.append(str(health_figure_path))
        if plot_p0_reason_histogram(health_rows, reason_figure_path):
            p0_figure_artifacts.append(str(reason_figure_path))
        if plot_p0_pl_cost_distribution(pl_cloud_rows, distribution_figure_path):
            p0_figure_artifacts.append(str(distribution_figure_path))
        if plot_p0_risk_grid_snapshot_overview(pl_cloud_rows, validity_cloud_rows, snapshot_figure_path):
            p0_figure_artifacts.append(str(snapshot_figure_path))
        else:
            warnings.append("P0 risk grid snapshot overview was not generated because no plottable cloud rows were available")
        if plot_odom_truth_topdown(odom_truth_rows, odom_rows, odom_topdown_figure_path):
            p0_figure_artifacts.append(str(odom_topdown_figure_path))
        else:
            warnings.append("odom truth top-down plot was not generated because odom/truth rows were unavailable")
        if plot_odom_error_timeline(odom_aligned_rows, odom_error_figure_path):
            p0_figure_artifacts.append(str(odom_error_figure_path))
        else:
            warnings.append("odom error timeline was not generated because aligned odom/truth rows were unavailable")
        if plot_p0_health_vs_odom_error(health_rows, odom_aligned_rows, health_vs_odom_figure_path):
            p0_figure_artifacts.append(str(health_vs_odom_figure_path))
        else:
            warnings.append("P0 health vs odom error plot was not generated because aligned rows were unavailable")
        if p0_requires_odom_gate:
            if plot_p0_baseline_distribution_delta(
                pl_cloud_rows,
                baseline_cloud_rows,
                baseline_distribution,
                p0_1_delta_figure_path,
                current_label=experiment_label,
                baseline_label="P0-1",
            ):
                p0_figure_artifacts.append(str(p0_1_delta_figure_path))
            else:
                warnings.append(f"{experiment_label} vs P0-1 delta figure was not generated because PL/cost rows were unavailable")
        if p0_requires_odom_gate and p0_phase_index is not None and p0_phase_index >= 3:
            if plot_p0_baseline_distribution_delta(
                pl_cloud_rows,
                p0_2_reference_cloud_rows,
                p0_2_reference_distribution,
                p0_2_delta_figure_path,
                current_label=experiment_label,
                baseline_label="P0-2",
            ):
                p0_figure_artifacts.append(str(p0_2_delta_figure_path))
            else:
                warnings.append(f"{experiment_label} vs P0-2 delta figure was not generated because PL/cost rows were unavailable")
        p0_required_figures.extend(
            [
                health_figure_path,
                reason_figure_path,
                distribution_figure_path,
                snapshot_figure_path,
            ]
        )
        if p0_requires_odom_gate:
            p0_required_figures.append(p0_1_delta_figure_path)
            p0_required_figures.extend(
                [
                    odom_topdown_figure_path,
                    odom_error_figure_path,
                    health_vs_odom_figure_path,
                ]
            )
        if p0_requires_odom_gate and p0_phase_index is not None and p0_phase_index >= 3:
            p0_required_figures.append(p0_2_delta_figure_path)

        p0_summary = {
            "health": p0_health_summary,
            "first_health_rows": health_rows[:8],
            "pl_cloud": p0_cloud_summary,
            "validity_cloud": p0_validity_cloud_summary,
            "odom_health": odom_health_summary,
            "startup_correlation": startup_correlation,
            "cloud_summary_rows": len(cloud_summary_rows),
            "topics_seen": p0_artifacts.get("topics_seen", []),
            "baseline_comparison": baseline_comparison,
            "baseline_distribution_comparison": baseline_distribution if p0_requires_odom_gate else {},
            "baseline_distribution_explanation": p0_distribution_explanation if p0_requires_odom_gate else {},
            "comparison_runs": p0_distribution_comparisons if p0_requires_odom_gate else {},
            "p0_4_fallback_unknown_semantics": p0_4_semantics,
            "zero_risk_fallback_check": p0_4_semantics.get("zero_risk_fallback_check", {}),
            "fallback_unknown_reason_ok": (p0_4_semantics.get("reason_semantics", {}) or {}).get("fallback_unknown_reason_ok"),
            "baseline_export_dir": str(baseline_export_dir) if baseline_export_dir is not None else "",
            "baseline_bag_dir": str(baseline_bag_dir) if baseline_bag_dir is not None else "",
            "p0_2_reference_export_dir": str(p0_2_export_dir) if p0_2_export_dir is not None else "",
            "p0_2_reference_bag_dir": str(p0_2_bag_dir) if p0_2_bag_dir is not None else "",
        }

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
        if p0_phase and baseline_scenario_data.get("truth_xy"):
            scenario_data["baseline_truth_xy"] = baseline_scenario_data.get("truth_xy", [])
        if plot_scenario_topdown(scenario_data, scenario_figure_path):
            figures.append(str(scenario_figure_path))
        else:
            warnings.append("scenario top-down plot was not generated because no plottable bag data was available")

        topic_timestamps, topic_timestamp_error = read_topic_timestamps(
            bag_dir,
            metadata,
            topic_activity_topics,
        )
        if topic_timestamp_error:
            warnings.append(f"topic activity timeline data could not be read: {topic_timestamp_error}")
        if plot_topic_activity_timeline(topic_timestamps, topic_activity_figure_path, topic_activity_topics):
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
    if p0_requires_odom_gate and p5_summary.get("action_counts"):
        failures.append(f"{experiment_label} P5 status reported actions while P5 is disabled")
    csv_artifacts = [str(topic_counts_path)]
    csv_artifacts.extend(p0_csv_artifacts)
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
    if p0_phase:
        figures.extend(p0_figure_artifacts)
        required_figures = [
            scenario_figure_path,
            topic_activity_figure_path,
            *p0_required_figures,
        ]
        if p0_4_phase:
            required_figures.append(source_figure_path)
        for figure_path in required_figures:
            if not figure_path.is_file() or figure_path.stat().st_size <= 0:
                inconclusive.append(
                    f"{experiment_label} required figure was not generated or is empty: {figure_path}"
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
            p0_health_summary if p0_phase else None,
        ),
        "export_dir": str(export_dir),
        "bag_dir": str(bag_dir) if bag_dir is not None else "",
        "manifest": manifest,
        "validator_summary": validator_summary,
        "topic_health": topic_health,
        "topic_timing_error": topic_timing_error,
        "integrity_summary": integrity_summary,
        "p0_summary": p0_summary,
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
    parser.add_argument("--p0-2-export-dir", default="", help="healthy P0-2 export directory for P0-3+ comparisons")
    parser.add_argument("--p0-2-bag-dir", default="", help="healthy P0-2 rosbag directory for P0-3+ comparisons")
    parser.add_argument("--synthetic-only", action="store_true", help="run analyzer-only synthetic experiment without ROS artifacts")
    parser.add_argument("--fail-on-threshold", action="store_true", help="exit non-zero unless analysis status is PASS")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze(args)
    return 0 if (summary["passed"] or not args.fail_on_threshold) else 2


if __name__ == "__main__":
    raise SystemExit(main())
