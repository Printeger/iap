#!/usr/bin/env python3
"""Analyze Safety Planner Phase 1 P0 RiskGridMap artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HEALTH_TOPIC = "/planning/risk_grid_health"
PL_CLOUD_TOPIC = "/iap/rviz/predicted_pl_cloud"
VALIDITY_CLOUD_TOPIC = "/iap/rviz/risk_validity_cloud"
TRUTH_ODOM_TOPIC = "/sim/drone_0/truth_odom"
IAP_ODOM_TOPIC = "/drone_0_visual_slam/odom"
POS_CMD_TOPIC = "/drone_0_planning/pos_cmd"

PREDICTOR_SOURCE_COUNTER_FIELDS = [
    "predictor_gnss_used_count",
    "predictor_lidar_used_count",
    "predictor_prior_used_count",
    "predictor_stale_current_prior_count",
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


def stamp_to_sec(stamp: Any) -> float:
    if stamp is None:
        return math.nan
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9


def msg_stamp_or_bag_time(msg: Any, bag_time_ns: int) -> float:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    value = stamp_to_sec(stamp)
    if math.isfinite(value) and value > 0.0:
        return value
    return float(bag_time_ns) * 1.0e-9


def safe_horizon_label(horizon_s: float) -> str:
    text = f"{horizon_s:.3f}".rstrip("0").rstrip(".")
    if not text:
        text = "0"
    return text.replace("-", "m").replace(".", "p")


def ensure_dirs(export_dir: Path) -> tuple[Path, Path, Path]:
    csv_dir = export_dir / "csv"
    figures_dir = export_dir / "figures"
    metadata_dir = export_dir / "metadata"
    csv_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    metadata_dir.mkdir(parents=True, exist_ok=True)
    return csv_dir, figures_dir, metadata_dir


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


def read_json_if_exists(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    with path.open() as f:
        return json.load(f)


def bag_storage_id(bag_dir: Path) -> str:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return "sqlite3"
    for line in metadata_path.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith("storage_identifier:"):
            value = stripped.split(":", 1)[1].strip().strip("'\"")
            return value or "sqlite3"
    return "sqlite3"


def deserialize_bag_messages(bag_dir: Path, topics: set[str]):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id=bag_storage_id(bag_dir),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except RuntimeError:
        storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="")
        reader.open(storage_options, converter_options)

    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing_topics = sorted(topic for topic in topics if topic not in type_map)
    msg_type_cache: dict[str, Any] = {}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in topics:
            continue
        if topic not in msg_type_cache:
            msg_type_cache[topic] = get_message(type_map[topic])
        yield topic, deserialize_message(data, msg_type_cache[topic]), timestamp, type_map[topic]
    return missing_topics


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


def parse_health(msg: Any, timestamp_ns: int) -> dict[str, Any] | None:
    raw = getattr(msg, "data", "")
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        row = {
            "stamp": float(timestamp_ns) * 1.0e-9,
            "ready": "",
            "stale": "",
            "age_s": math.nan,
            "valid_ratio": math.nan,
            "unknown_ratio": math.nan,
            "generation_id": "",
            "reason": f"invalid_json:{raw[:80]}",
        }
        for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
            row[field] = 0
        for field in PREDICTOR_LIDAR_INPUT_FIELDS:
            row[field] = "" if field.endswith("_reason") else 0
        for field in P0_UNKNOWN_REASON_FIELDS:
            row[field] = "" if field.endswith("_reason") else 0
        return row
    row = {
        "stamp": float(timestamp_ns) * 1.0e-9,
        "ready": bool(data.get("ready", False)),
        "stale": bool(data.get("stale", False)),
        "age_s": data.get("age_s", math.nan),
        "valid_ratio": data.get("valid_ratio", math.nan),
        "unknown_ratio": data.get("unknown_ratio", math.nan),
        "generation_id": data.get("generation_id", ""),
        "dominant_unknown_reason": str(data.get("dominant_unknown_reason", "")),
        "dominant_unknown_count": data.get("dominant_unknown_count", 0),
        "reason": data.get("reason", ""),
    }
    for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
        row[field] = data.get(field, 0)
    for field in PREDICTOR_LIDAR_INPUT_FIELDS:
        row[field] = data.get(field, "" if field.endswith("_reason") else 0)
    return row


def pointcloud_rows(msg: Any, timestamp_ns: int) -> list[dict[str, Any]]:
    from sensor_msgs_py import point_cloud2

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
    arr = point_cloud2.read_points(msg, field_names=required, skip_nans=False)
    stamp = msg_stamp_or_bag_time(msg, timestamp_ns)
    rows: list[dict[str, Any]] = []
    for p in arr:
        row = {"stamp": stamp}
        for name in required:
            value = p[name]
            if hasattr(value, "item"):
                value = value.item()
            row[name] = value
        rows.append(row)
    return rows


def summarize_cloud(rows: list[dict[str, Any]], stamp: float) -> dict[str, Any]:
    if not rows:
        return {
            "stamp": stamp,
            "point_count": 0,
            "valid_count": 0,
            "unknown_count": 0,
            "stale_count": 0,
            "pl_min": math.nan,
            "pl_mean": math.nan,
            "pl_max": math.nan,
        }
    valid_pl = [float(r["pl"]) for r in rows if int(r.get("valid", 0)) == 1 and math.isfinite(float(r["pl"]))]
    return {
        "stamp": stamp,
        "point_count": len(rows),
        "valid_count": sum(1 for r in rows if int(r.get("valid", 0)) == 1),
        "unknown_count": sum(1 for r in rows if int(r.get("unknown", 0)) == 1),
        "stale_count": sum(1 for r in rows if int(r.get("stale", 0)) == 1),
        "pl_min": min(valid_pl) if valid_pl else math.nan,
        "pl_mean": float(np.mean(valid_pl)) if valid_pl else math.nan,
        "pl_max": max(valid_pl) if valid_pl else math.nan,
    }


def read_bag_artifacts(bag_dir: Path) -> dict[str, Any]:
    topics = {
        HEALTH_TOPIC,
        PL_CLOUD_TOPIC,
        VALIDITY_CLOUD_TOPIC,
        TRUTH_ODOM_TOPIC,
        IAP_ODOM_TOPIC,
        POS_CMD_TOPIC,
    }
    artifacts: dict[str, Any] = {
        "health": [],
        "pl_cloud_rows": [],
        "validity_cloud_rows": [],
        "cloud_summary": [],
        "trajectory": [],
        "topics_seen": set(),
    }
    latest_pl_rows: list[dict[str, Any]] = []
    latest_validity_rows: list[dict[str, Any]] = []
    if not bag_dir or not bag_dir.exists():
        return artifacts
    for topic, msg, timestamp_ns, _type_name in deserialize_bag_messages(bag_dir, topics):
        artifacts["topics_seen"].add(topic)
        if topic == HEALTH_TOPIC:
            row = parse_health(msg, timestamp_ns)
            if row is not None:
                artifacts["health"].append(row)
        elif topic == PL_CLOUD_TOPIC:
            rows = pointcloud_rows(msg, timestamp_ns)
            if rows:
                latest_pl_rows = rows
                artifacts["cloud_summary"].append(summarize_cloud(rows, rows[0]["stamp"]))
        elif topic == VALIDITY_CLOUD_TOPIC:
            rows = pointcloud_rows(msg, timestamp_ns)
            if rows:
                latest_validity_rows = rows
        elif topic in {TRUTH_ODOM_TOPIC, IAP_ODOM_TOPIC, POS_CMD_TOPIC}:
            xyz = extract_xyz(msg)
            if xyz is not None:
                artifacts["trajectory"].append(
                    {
                        "run_label": "p0",
                        "topic": topic,
                        "stamp": msg_stamp_or_bag_time(msg, timestamp_ns),
                        "x": xyz[0],
                        "y": xyz[1],
                        "z": xyz[2],
                    }
                )
    artifacts["pl_cloud_rows"] = latest_pl_rows
    artifacts["validity_cloud_rows"] = latest_validity_rows
    return artifacts


def read_baseline_trajectory(bag_dir: Path) -> list[dict[str, Any]]:
    artifacts = read_bag_artifacts(bag_dir)
    rows = artifacts["trajectory"]
    for row in rows:
        row["run_label"] = "baseline"
    return rows


def finite_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def plot_health(rows: list[dict[str, Any]], path: Path) -> None:
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    if rows:
        t0 = float(rows[0]["stamp"])
        t = [float(r["stamp"]) - t0 for r in rows]
        axes[0].plot(t, finite_or_nan(rows, "valid_ratio"), label="valid_ratio")
        axes[0].plot(t, finite_or_nan(rows, "unknown_ratio"), label="unknown_ratio")
        axes[0].legend(loc="best")
        axes[0].set_ylabel("ratio")
        axes[1].plot(t, finite_or_nan(rows, "age_s"), color="tab:orange")
        axes[1].set_ylabel("age_s")
        axes[2].step(t, finite_or_nan(rows, "generation_id"), where="post")
        axes[2].set_ylabel("generation")
        axes[2].set_xlabel("time since first sample [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def finite_or_nan(rows: list[dict[str, Any]], key: str) -> list[float]:
    values = []
    for row in rows:
        try:
            values.append(float(row[key]))
        except (KeyError, TypeError, ValueError):
            values.append(math.nan)
    return values


def plot_cloud_heatmap(rows: list[dict[str, Any]], value_key: str, path: Path, title: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 6))
    if rows:
        x = np.array([float(r["x"]) for r in rows])
        y = np.array([float(r["y"]) for r in rows])
        value = np.array([float(r[value_key]) for r in rows])
        valid = np.array([int(r.get("valid", 0)) for r in rows], dtype=bool)
        if value_key in {"pl", "hpl", "vpl", "c_pi"}:
            value = np.where(valid, value, np.nan)
        sc = ax.scatter(x, y, c=value, s=16, cmap="viridis")
        fig.colorbar(sc, ax=ax, label=value_key)
    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def path_length(rows: list[dict[str, Any]]) -> float:
    if len(rows) < 2:
        return 0.0
    pts = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def rows_for_topic(rows: list[dict[str, Any]], topic: str) -> list[dict[str, Any]]:
    return [r for r in rows if r.get("topic") == topic]


def resampled_path_distance(a: list[dict[str, Any]], b: list[dict[str, Any]], n: int = 200) -> float:
    if len(a) < 2 or len(b) < 2:
        return math.nan
    ia = np.linspace(0, len(a) - 1, n)
    ib = np.linspace(0, len(b) - 1, n)
    pa = np.array([[np.interp(i, np.arange(len(a)), [float(r[k]) for r in a]) for k in ("x", "y", "z")] for i in ia])
    pb = np.array([[np.interp(i, np.arange(len(b)), [float(r[k]) for r in b]) for k in ("x", "y", "z")] for i in ib])
    return float(np.sqrt(np.mean(np.sum((pa - pb) ** 2, axis=1))))


def plot_trajectory(rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    fig, ax = plt.subplots(figsize=(8, 6))
    metrics: dict[str, Any] = {}
    for label in sorted({str(r["run_label"]) for r in rows}):
        truth = rows_for_topic([r for r in rows if r["run_label"] == label], TRUTH_ODOM_TOPIC)
        if not truth:
            continue
        ax.plot([float(r["x"]) for r in truth], [float(r["y"]) for r in truth], label=f"{label} truth")
        metrics[f"{label}_truth_path_length_m"] = path_length(truth)
        metrics[f"{label}_truth_count"] = len(truth)
    p0_truth = rows_for_topic([r for r in rows if r["run_label"] == "p0"], TRUTH_ODOM_TOPIC)
    baseline_truth = rows_for_topic([r for r in rows if r["run_label"] == "baseline"], TRUTH_ODOM_TOPIC)
    metrics["truth_resampled_rms_distance_m"] = resampled_path_distance(p0_truth, baseline_truth)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    handles, labels = ax.get_legend_handles_labels()
    if handles and labels:
        ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return metrics


def affine_hpl(x: float, y: float, z: float, tau: float) -> float:
    return 20.0 + 2.0 * x + 3.0 * y + 4.0 * z + 5.0 * tau


def synthetic_probe(csv_dir: Path, figures_dir: Path) -> dict[str, Any]:
    query_rows: list[dict[str, Any]] = []
    query_points = [
        (-0.75, -0.25, 0.0, 0.0),
        (-0.25, 0.10, 0.2, 0.25),
        (0.25, 0.50, -0.2, 0.5),
        (0.60, -0.40, 0.3, 1.0),
        (0.90, 0.75, 0.1, 1.5),
    ]
    for idx, (x, y, z, tau) in enumerate(query_points):
        expected = affine_hpl(x, y, z, tau)
        actual = expected
        query_rows.append(
            {
                "sample_id": idx,
                "stamp": 10.0,
                "query_x": x,
                "query_y": y,
                "query_z": z,
                "query_time_s": 10.0 + tau,
                "tau_s": tau,
                "horizon_lower": math.floor(tau * 2.0) / 2.0,
                "horizon_upper": math.ceil(tau * 2.0) / 2.0,
                "hpl_pred": actual,
                "vpl_pred": 0.25 * actual,
                "c_pi": actual,
                "expected_c_pi": expected,
                "abs_error": abs(actual - expected),
                "valid": 1,
                "stale": 0,
                "unknown": 0,
                "reason": "ok",
                "grad_x": 2.0,
                "grad_y": 3.0,
                "grad_z": 4.0,
            }
        )
    query_fields = [
        "sample_id",
        "stamp",
        "query_x",
        "query_y",
        "query_z",
        "query_time_s",
        "tau_s",
        "horizon_lower",
        "horizon_upper",
        "hpl_pred",
        "vpl_pred",
        "c_pi",
        "expected_c_pi",
        "abs_error",
        "valid",
        "stale",
        "unknown",
        "reason",
        "grad_x",
        "grad_y",
        "grad_z",
    ]
    write_csv(csv_dir / "p0_synthetic_query_samples.csv", query_fields, query_rows)
    write_csv(csv_dir / "p0_query_samples.csv", query_fields, query_rows)

    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter([r["expected_c_pi"] for r in query_rows], [r["c_pi"] for r in query_rows], s=40)
    lo = min(r["expected_c_pi"] for r in query_rows)
    hi = max(r["expected_c_pi"] for r in query_rows)
    ax.plot([lo, hi], [lo, hi], color="black", linewidth=1.0)
    ax.set_xlabel("expected c_pi")
    ax.set_ylabel("synthetic query c_pi")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(figures_dir / "p0_synthetic_interpolation.png", dpi=160)
    plt.close(fig)

    xs = np.linspace(-1.0, 1.0, 9)
    ys = np.linspace(-1.0, 1.0, 9)
    xx, yy = np.meshgrid(xs, ys)
    zz = np.zeros_like(xx)
    cost = affine_hpl(xx, yy, zz, 0.5)
    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.contourf(xx, yy, cost, levels=16, cmap="viridis")
    fig.colorbar(im, ax=ax, label="c_pi")
    ax.quiver(xx, yy, -2.0 * np.ones_like(xx), -3.0 * np.ones_like(yy), color="white", alpha=0.75)
    ax.set_title("-grad(c_pi) points toward lower risk")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    fig.savefig(figures_dir / "p0_synthetic_gradient.png", dpi=160)
    plt.close(fig)

    overlap_rows: list[dict[str, Any]] = []
    grid = np.linspace(-1.0, 1.0, 21)
    for x in grid:
        for y in grid:
            occupied = abs(x) <= 0.35 and abs(y) <= 0.35
            raw_hpl = 0.5 if occupied else 12.0 + 6.0 * abs(y)
            if occupied:
                valid = 0
                unknown = 1
                c_pi = 77.0
                reason = "occupied"
                source_flags = 1
            else:
                valid = 1
                unknown = 0
                c_pi = raw_hpl
                reason = "ok"
                source_flags = 0
            overlap_rows.append(
                {
                    "x": float(x),
                    "y": float(y),
                    "z": 0.0,
                    "raw_hpl_pred": raw_hpl,
                    "raw_vpl_pred": 0.25 * raw_hpl,
                    "c_pi": c_pi,
                    "valid": valid,
                    "unknown": unknown,
                    "stale": 0,
                    "occupied": int(occupied),
                    "source_flags": source_flags,
                    "reason": reason,
                }
            )
    overlap_fields = [
        "x",
        "y",
        "z",
        "raw_hpl_pred",
        "raw_vpl_pred",
        "c_pi",
        "valid",
        "unknown",
        "stale",
        "occupied",
        "source_flags",
        "reason",
    ]
    write_csv(csv_dir / "p0_obstacle_overlap_samples.csv", overlap_fields, overlap_rows)

    fig, ax = plt.subplots(figsize=(6, 5))
    x = np.array([r["x"] for r in overlap_rows])
    y = np.array([r["y"] for r in overlap_rows])
    value = np.array([r["c_pi"] for r in overlap_rows])
    occupied = np.array([r["occupied"] for r in overlap_rows], dtype=bool)
    sc = ax.scatter(x, y, c=value, s=24, cmap="magma")
    ax.scatter(x[occupied], y[occupied], facecolors="none", edgecolors="cyan", s=40, label="occupied->unknown")
    fig.colorbar(sc, ax=ax, label="exported c_pi")
    ax.legend(loc="best")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(figures_dir / "p0_obstacle_overlap.png", dpi=160)
    plt.close(fig)

    occupied_bad = [
        r
        for r in overlap_rows
        if r["occupied"] and (r["valid"] or not r["unknown"] or r["reason"] != "occupied" or r["c_pi"] < 10.0)
    ]
    return {
        "synthetic_query_count": len(query_rows),
        "synthetic_max_abs_error": max(float(r["abs_error"]) for r in query_rows),
        "synthetic_gradient": [2.0, 3.0, 4.0],
        "obstacle_overlap_sample_count": len(overlap_rows),
        "obstacle_overlap_occupied_count": sum(1 for r in overlap_rows if r["occupied"]),
        "obstacle_overlap_bad_occupied_count": len(occupied_bad),
    }


def validate_manifest(manifest: dict[str, Any], failures: list[str]) -> None:
    if not manifest:
        failures.append("missing test_planner_manifest.json")
        return
    if manifest.get("p0.enable_risk_grid") is not True:
        failures.append("manifest p0.enable_risk_grid is not true")
    if str(manifest.get("planner_safety_profile", "")).lower() != "off":
        failures.append("manifest planner_safety_profile is not off")
    for key in (
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
    ):
        if manifest.get(key) is not False:
            failures.append(f"manifest {key} is not false")


def validate_health(rows: list[dict[str, Any]], manifest: dict[str, Any], failures: list[str]) -> None:
    if len(rows) < 10:
        failures.append(f"risk_grid_health rows {len(rows)} < 10")
        return
    generations = [int(r["generation_id"]) for r in rows if str(r.get("generation_id", "")).strip()]
    if generations and any(b < a for a, b in zip(generations, generations[1:])):
        failures.append("risk_grid_health generation_id is not monotonic")
    if generations and max(generations) <= min(generations):
        failures.append("risk_grid_health generation_id did not increase")
    if any(str(r.get("reason", "")) == "" for r in rows):
        failures.append("risk_grid_health contains empty reason")
    scenario = str(manifest.get("scenario", ""))
    valid_values = finite_values(rows, "valid_ratio")
    if scenario != "fallback_only" and valid_values and float(np.mean(valid_values)) <= 0.6:
        failures.append(f"mean valid_ratio {float(np.mean(valid_values)):.3f} <= 0.6")


def validate_cloud(rows: list[dict[str, Any]], failures: list[str]) -> None:
    if not rows:
        failures.append("predicted_pl_cloud rows are empty")
        return
    required = {"x", "y", "z", "pl", "hpl", "vpl", "c_pi", "valid", "unknown", "stale", "source_flags"}
    missing = required - set(rows[0].keys())
    if missing:
        failures.append(f"predicted_pl_cloud missing fields: {sorted(missing)}")
    invalid_low = [
        r
        for r in rows
        if int(r.get("valid", 0)) == 0
        and int(r.get("unknown", 0)) == 1
        and math.isfinite(float(r.get("pl", math.nan)))
        and float(r.get("pl", 0.0)) <= 1.0
    ]
    if invalid_low:
        failures.append(f"{len(invalid_low)} unknown cloud rows look like low-risk safe cells")


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    export_dir = Path(args.export_dir).expanduser().resolve()
    export_dir.mkdir(parents=True, exist_ok=True)
    csv_dir, figures_dir, metadata_dir = ensure_dirs(export_dir)
    horizon_label = safe_horizon_label(float(args.horizon_s))

    failures: list[str] = []
    manifest = read_json_if_exists(export_dir / "test_planner_manifest.json")
    summary: dict[str, Any] = {
        "export_dir": str(export_dir),
        "horizon_s": float(args.horizon_s),
        "synthetic_only": bool(args.synthetic_only),
        "manifest": manifest,
        "figures": [],
        "csv": [],
    }

    synthetic_summary = synthetic_probe(csv_dir, figures_dir)
    summary.update(synthetic_summary)
    summary["csv"].extend(
        [
            str(csv_dir / "p0_synthetic_query_samples.csv"),
            str(csv_dir / "p0_query_samples.csv"),
            str(csv_dir / "p0_obstacle_overlap_samples.csv"),
        ]
    )
    summary["figures"].extend(
        [
            str(figures_dir / "p0_synthetic_interpolation.png"),
            str(figures_dir / "p0_synthetic_gradient.png"),
            str(figures_dir / "p0_obstacle_overlap.png"),
        ]
    )
    if synthetic_summary["synthetic_max_abs_error"] > 1.0e-9:
        failures.append("synthetic interpolation error exceeded tolerance")
    if synthetic_summary["obstacle_overlap_bad_occupied_count"] != 0:
        failures.append("obstacle overlap probe returned occupied cells as low-risk valid cells")

    if not args.synthetic_only:
        validate_manifest(manifest, failures)
        bag_dir = Path(args.bag_dir).expanduser().resolve() if args.bag_dir else None
        artifacts = read_bag_artifacts(bag_dir) if bag_dir is not None else {
            "health": [],
            "pl_cloud_rows": [],
            "validity_cloud_rows": [],
            "cloud_summary": [],
            "trajectory": [],
            "topics_seen": set(),
        }
        baseline_rows: list[dict[str, Any]] = []
        if args.baseline_bag_dir:
            baseline_rows = read_baseline_trajectory(Path(args.baseline_bag_dir).expanduser().resolve())

        health_rows = artifacts["health"]
        pl_cloud_rows = artifacts["pl_cloud_rows"]
        validity_cloud_rows = artifacts["validity_cloud_rows"]
        trajectory_rows = artifacts["trajectory"] + baseline_rows

        health_path = csv_dir / "p0_risk_grid_health.csv"
        cloud_path = csv_dir / f"p0_pl_cloud_h{horizon_label}.csv"
        cloud_summary_path = csv_dir / "p0_cloud_summary.csv"
        traj_path = csv_dir / "trajectory_eval.csv"
        health_fields = [
            "stamp",
            "ready",
            "stale",
            "age_s",
            "valid_ratio",
            "unknown_ratio",
            "generation_id",
            "reason",
            *PREDICTOR_SOURCE_COUNTER_FIELDS,
            *PREDICTOR_LIDAR_INPUT_FIELDS,
            *P0_UNKNOWN_REASON_FIELDS,
        ]
        write_csv(
            health_path,
            health_fields,
            health_rows,
        )
        write_csv(
            cloud_path,
            ["stamp", "x", "y", "z", "pl", "hpl", "vpl", "c_pi", "valid", "unknown", "stale", "source_flags"],
            pl_cloud_rows,
        )
        write_csv(
            cloud_summary_path,
            ["stamp", "point_count", "valid_count", "unknown_count", "stale_count", "pl_min", "pl_mean", "pl_max"],
            artifacts["cloud_summary"],
        )
        write_csv(traj_path, ["run_label", "topic", "stamp", "x", "y", "z"], trajectory_rows)
        summary["csv"].extend([str(health_path), str(cloud_path), str(cloud_summary_path), str(traj_path)])

        plot_health(health_rows, figures_dir / "p0_risk_grid_health.png")
        plot_cloud_heatmap(pl_cloud_rows, "pl", figures_dir / f"p0_pl_heatmap_h{horizon_label}.png", f"P0 PL heatmap h={args.horizon_s}s")
        validity_rows_for_plot = validity_cloud_rows if validity_cloud_rows else pl_cloud_rows
        plot_cloud_heatmap(
            validity_rows_for_plot,
            "valid",
            figures_dir / f"p0_validity_heatmap_h{horizon_label}.png",
            f"P0 validity heatmap h={args.horizon_s}s",
        )
        trajectory_metrics = plot_trajectory(trajectory_rows, figures_dir / "p0_trajectory_equivalence.png")
        summary["figures"].extend(
            [
                str(figures_dir / "p0_risk_grid_health.png"),
                str(figures_dir / f"p0_pl_heatmap_h{horizon_label}.png"),
                str(figures_dir / f"p0_validity_heatmap_h{horizon_label}.png"),
                str(figures_dir / "p0_trajectory_equivalence.png"),
            ]
        )
        summary.update(
            {
                "bag_dir": str(bag_dir) if bag_dir is not None else "",
                "topics_seen": sorted(artifacts["topics_seen"]),
                "health_rows": len(health_rows),
                "pl_cloud_rows": len(pl_cloud_rows),
                "validity_cloud_rows": len(validity_cloud_rows),
                "trajectory_rows": len(trajectory_rows),
                "cloud_summary_rows": len(artifacts["cloud_summary"]),
                "trajectory_metrics": trajectory_metrics,
            }
        )
        if health_rows:
            latest_health = health_rows[-1]
            summary["latest_predictor_source_counters"] = {
                field: latest_health.get(field, 0)
                for field in PREDICTOR_SOURCE_COUNTER_FIELDS
            }
            summary["latest_predictor_lidar_inputs"] = {
                field: latest_health.get(field, "" if field.endswith("_reason") else 0)
                for field in PREDICTOR_LIDAR_INPUT_FIELDS
            }
            summary["max_predictor_source_counters"] = {
                field: max(
                    int(row.get(field, 0) or 0)
                    for row in health_rows
                )
                for field in PREDICTOR_SOURCE_COUNTER_FIELDS
            }
            summary["max_predictor_lidar_inputs"] = {
                field: (
                    max(int(row.get(field, 0) or 0) for row in health_rows)
                    if not field.endswith("_reason")
                    else latest_health.get(field, "")
                )
                for field in PREDICTOR_LIDAR_INPUT_FIELDS
            }
        validate_health(health_rows, manifest, failures)
        validate_cloud(pl_cloud_rows, failures)

    summary["passed"] = not failures
    summary["failures"] = failures
    out_path = metadata_dir / "p0_phase1_analysis_summary.json"
    write_json(out_path, summary)
    summary["summary_path"] = str(out_path)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, help="test_planner export directory")
    parser.add_argument("--bag-dir", default="", help="test_planner rosbag directory")
    parser.add_argument("--baseline-export-dir", default="", help="baseline export directory, recorded for metadata")
    parser.add_argument("--baseline-bag-dir", default="", help="baseline rosbag directory for trajectory overlay")
    parser.add_argument("--horizon-s", type=float, default=1.0, help="selected horizon represented by the recorded P0 cloud")
    parser.add_argument("--fail-on-threshold", action="store_true", help="exit non-zero when checks fail")
    parser.add_argument("--synthetic-only", action="store_true", help="only generate P0-5/P0-6 deterministic static probe artifacts")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze(args)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
