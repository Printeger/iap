#!/usr/bin/env python3
"""Analyze ARAIM runtime experiment rosbags.

The script reads a rosbag2 directory directly with rosbag2_py and writes CSV,
JSON, and PNG outputs into <bag_dir>/<output_subdir>.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2


INTEGRITY_TOPIC = "/iap/integrity"
TRAJECTORY_TOPICS = {
    "truth": "/sim/drone_0/truth_odom",
    "iap": "/drone_0_visual_slam/odom",
    "desired": "/demo11_araim/desired/odom",
}
POINT_CLOUD_TOPICS = {
    "global": "/map_generator/global_cloud",
    "trunks": "/demo11/trunk_cloud",
    "canopy": "/demo11/canopy_cloud",
    "terminal_wall": "/demo11/terminal_wall_cloud",
}
PL_DECOMP_CSV_NAME = "iap_araim_pl_decomp.csv"

SOURCE_CODE = {
    "UNKNOWN": 0,
    "": 0,
    "GNSS": 1,
    "LIDAR": 2,
    "FALLBACK": 3,
    "CONSERVATIVE": 4,
}
STATE_CODE = {
    0: "SAFE",
    1: "SAFE_EXCLUDED",
    2: "UNSAFE",
}
HYP_TYPE_CODE = {
    "UNKNOWN": 0,
    "": 0,
    "FAULT_FREE": 1,
    "GNSS_SAT": 2,
    "SATELLITE": 2,
    "CONSTELLATION": 3,
    "SUBSET": 4,
    "TRUNK": 5,
}
CONSTELLATION_CODE = {
    "UNKNOWN": 0,
    "": 0,
    "GPS": 1,
    "GAL": 2,
    "BDS": 3,
    "GLO": 4,
}

INTEGRITY_FIELDS = [
    "integrity_state",
    "hpl",
    "vpl",
    "pl_e",
    "pl_n",
    "pl_u",
    "hal",
    "val",
    "im",
    "im_h",
    "im_v",
    "im_min",
    "pl_ff",
    "pl_ff_v",
    "k_ff_used",
    "k_fa_used",
    "n_sv_used",
    "n_constellations",
    "pdop",
    "sigma_h",
    "n_hypotheses",
    "n_detected",
    "excluded_prns",
    "excluded_trunk_ids",
    "n_trunks_observed",
    "tdop",
    "gnss_valid",
    "gnss_hpl",
    "gnss_vpl",
    "gnss_pl_e",
    "gnss_pl_n",
    "gnss_pl_u",
    "gnss_pl_ff",
    "gnss_k_ff_used",
    "gnss_k_fa_used",
    "gnss_n_hyp",
    "gnss_n_det",
    "lidar_valid",
    "lidar_hpl",
    "lidar_vpl",
    "lidar_pl_e",
    "lidar_pl_n",
    "lidar_pl_u",
    "lidar_n_hyp",
    "lidar_n_det",
    "lidar_worst_mode",
    "fallback_valid",
    "fallback_hpl",
    "fallback_vpl",
    "fusion_mode",
    "final_hpl_source",
    "final_vpl_source",
    "final_pl_source",
    "fallback_pl_invalid",
    "gnss_araim_invalid",
    "lidar_integrity_invalid",
    "hal_invalid",
    "val_invalid",
    "im_invalid",
    "any_nan_rejected",
    "any_inf_rejected",
    "negative_variance_rejected",
    "degenerate_geometry",
    "failure_reason",
]
PL_DECOMP_NUMERIC_FIELDS = {
    "stamp",
    "frame_seq",
    "HPL",
    "VPL",
    "PL_E",
    "PL_N",
    "PL_U",
    "HAL",
    "VAL",
    "im_h",
    "im_v",
    "im_min",
    "gnss_valid",
    "n_used",
    "n_hyp",
    "n_detected",
    "n_constellations",
    "pdop",
    "sigma_H_0",
    "sigma_E_0",
    "sigma_N_0",
    "sigma_V_0",
    "K_ff",
    "K_fa",
    "K_md",
    "hyp_index",
    "hyp_id",
    "row_removed",
    "sat_id",
    "const_id",
    "hyp_PL_axis",
    "hyp_PL_E",
    "hyp_PL_N",
    "hyp_PL_U",
    "abs_d_axis",
    "d_E",
    "d_N",
    "d_U",
    "sigma_ss_axis",
    "sigma_ss_E",
    "sigma_ss_N",
    "sigma_ss_U",
    "sigma_k_axis",
    "sigma_k_E",
    "sigma_k_N",
    "sigma_k_U",
    "term_d",
    "term_fa",
    "term_md",
    "term_bias",
    "total_reconstructed",
    "fault_detected",
    "degenerate",
    "dominant_const_id",
    "n_used_total",
    "n_removed_by_hyp",
    "n_remaining_after_hyp",
    "HDOP_full",
    "VDOP_full",
    "PDOP_full",
    "HDOP_subset",
    "VDOP_subset",
    "PDOP_subset",
    "sigma_H_full",
    "sigma_V_full",
    "sigma_H_subset",
    "sigma_V_subset",
}


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def msg_stamp_sec(msg: Any) -> float | None:
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return stamp_to_sec(msg.header.stamp)
    return None


def read_storage_id(bag_dir: Path) -> str:
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        return ""
    for line in metadata.read_text(encoding="utf-8", errors="replace").splitlines():
        if "storage_identifier:" in line:
            return line.split(":", 1)[1].strip()
    return ""


def sanitize_source(value: Any) -> str:
    text = str(value).strip().upper()
    return text if text in SOURCE_CODE else "UNKNOWN"


def source_to_code(value: Any) -> int:
    return SOURCE_CODE.get(sanitize_source(value), 0)


def sanitize_hyp_type(value: Any) -> str:
    text = str(value).strip().upper()
    return text if text in HYP_TYPE_CODE else "UNKNOWN"


def hyp_type_to_code(value: Any) -> int:
    return HYP_TYPE_CODE.get(sanitize_hyp_type(value), 0)


def sanitize_constellation(value: Any) -> str:
    text = str(value).strip().upper()
    return text if text in CONSTELLATION_CODE else "UNKNOWN"


def constellation_to_code(value: Any) -> int:
    return CONSTELLATION_CODE.get(sanitize_constellation(value), 0)


def array_to_csv(value: Any) -> str:
    try:
        return ";".join(str(int(v)) for v in value)
    except TypeError:
        return ""


def parse_csv_number(value: Any) -> float | int | str:
    text = str(value).strip()
    if text == "":
        return math.nan
    try:
        number = float(text)
    except ValueError:
        return text
    if math.isfinite(number) and number.is_integer():
        return int(number)
    return number


def resolve_pl_decomp_csv(bag_dir: Path, requested_path: Path | None) -> Path | None:
    if requested_path is not None:
        path = requested_path.expanduser().resolve()
        if not path.is_file():
            raise SystemExit(f"PL decomposition CSV does not exist: {path}")
        return path

    candidates = [
        bag_dir / PL_DECOMP_CSV_NAME,
        bag_dir / "export" / PL_DECOMP_CSV_NAME,
        bag_dir / "araim_analysis" / PL_DECOMP_CSV_NAME,
    ]
    for path in candidates:
        if path.is_file():
            return path.resolve()
    return None


def read_pl_decomp_csv(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.DictReader(f)
        for raw_row in reader:
            row: dict[str, Any] = {}
            for key, value in raw_row.items():
                if key is None:
                    continue
                row[key] = parse_csv_number(value) if key in PL_DECOMP_NUMERIC_FIELDS else value
            row["axis"] = str(row.get("axis", "")).strip().upper()
            row["hyp_type"] = sanitize_hyp_type(row.get("hyp_type", ""))
            row["hyp_type_code"] = hyp_type_to_code(row["hyp_type"])
            row["dominant_const_name"] = sanitize_constellation(row.get("dominant_const_name", ""))
            row["dominant_const_name_code"] = constellation_to_code(row["dominant_const_name"])
            row["final_hpl_source_code"] = source_to_code(row.get("final_hpl_source", ""))
            row["final_vpl_source_code"] = source_to_code(row.get("final_vpl_source", ""))
            rows.append(row)
    return rows


def pointcloud_xyz(msg: Any, max_points: int) -> np.ndarray:
    try:
        arr = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        xyz = np.asarray(arr, dtype=float).reshape(-1, 3)
    except Exception:
        rows = list(
            point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
        )
        xyz = np.asarray(rows, dtype=float).reshape(-1, 3) if rows else np.empty((0, 3))

    if xyz.shape[0] > max_points:
        stride = int(math.ceil(xyz.shape[0] / float(max_points)))
        xyz = xyz[::stride]
    return xyz


def open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    storage_id = read_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    return reader


def collect_bag_data(bag_dir: Path, max_cloud_points: int) -> dict[str, Any]:
    reader = open_reader(bag_dir)
    topics_and_types = reader.get_all_topics_and_types()
    topic_types = {item.name: item.type for item in topics_and_types}
    desired_topics = (
        {INTEGRITY_TOPIC}
        | set(TRAJECTORY_TOPICS.values())
        | set(POINT_CLOUD_TOPICS.values())
    )
    present_topics = sorted(desired_topics & set(topic_types))
    missing_topics = sorted(desired_topics - set(topic_types))

    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=present_topics))
    except Exception:
        pass

    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in present_topics
        if topic in topic_types
    }

    integrity_rows: list[dict[str, Any]] = []
    integrity_aux: list[dict[str, Any]] = []
    trajectories: dict[str, list[dict[str, float]]] = {key: [] for key in TRAJECTORY_TOPICS}
    clouds: dict[str, np.ndarray] = {}
    topic_counts: Counter[str] = Counter()
    first_bag_time: float | None = None
    last_bag_time: float | None = None

    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        bag_time = float(bag_time_ns) * 1.0e-9
        first_bag_time = bag_time if first_bag_time is None else min(first_bag_time, bag_time)
        last_bag_time = bag_time if last_bag_time is None else max(last_bag_time, bag_time)
        topic_counts[topic] += 1

        if topic not in msg_types:
            continue

        msg = deserialize_message(raw, msg_types[topic])
        header_time = msg_stamp_sec(msg)

        if topic == INTEGRITY_TOPIC:
            row: dict[str, Any] = {
                "bag_time": bag_time,
                "stamp": header_time if header_time is not None else bag_time,
            }
            aux = {
                "excluded_prns_list": list(getattr(msg, "excluded_prns", [])),
                "excluded_trunk_ids_list": list(getattr(msg, "excluded_trunk_ids", [])),
            }
            for field in INTEGRITY_FIELDS:
                value = getattr(msg, field)
                if field in ("excluded_prns", "excluded_trunk_ids"):
                    row[field] = array_to_csv(value)
                elif isinstance(value, bool):
                    row[field] = int(value)
                else:
                    row[field] = value
            row["final_hpl_source_code"] = source_to_code(row.get("final_hpl_source", ""))
            row["final_vpl_source_code"] = source_to_code(row.get("final_vpl_source", ""))
            row["final_pl_source_code"] = source_to_code(row.get("final_pl_source", ""))
            integrity_rows.append(row)
            integrity_aux.append(aux)
            continue

        for name, traj_topic in TRAJECTORY_TOPICS.items():
            if topic == traj_topic:
                p = msg.pose.pose.position
                trajectories[name].append(
                    {
                        "bag_time": bag_time,
                        "stamp": header_time if header_time is not None else bag_time,
                        "x": float(p.x),
                        "y": float(p.y),
                        "z": float(p.z),
                    }
                )
                break

        for name, cloud_topic in POINT_CLOUD_TOPICS.items():
            if topic == cloud_topic and name not in clouds:
                clouds[name] = pointcloud_xyz(msg, max_cloud_points)
                break

    try:
        reader.close()
    except Exception:
        pass

    return {
        "topic_types": topic_types,
        "topic_counts": dict(topic_counts),
        "missing_topics": missing_topics,
        "integrity_rows": integrity_rows,
        "integrity_aux": integrity_aux,
        "trajectories": trajectories,
        "clouds": clouds,
        "bag_time_range": [first_bag_time, last_bag_time],
    }


def choose_t0(data: dict[str, Any], time_base: str) -> float:
    rows = data["integrity_rows"]
    trajectories = data["trajectories"]
    if time_base == "integrity" and rows:
        return float(rows[0]["stamp"])
    if time_base == "ros_stamp":
        stamps = [float(row["stamp"]) for row in rows]
        for traj in trajectories.values():
            stamps.extend(float(row["stamp"]) for row in traj)
        if stamps:
            return min(stamps)
    start = data["bag_time_range"][0]
    if start is not None:
        return float(start)
    return 0.0


def apply_time_base(data: dict[str, Any], t0: float, time_base: str) -> None:
    for row in data["integrity_rows"]:
        base = row["bag_time"] if time_base == "bag" else row["stamp"]
        row["t"] = float(base) - t0
    for traj in data["trajectories"].values():
        for row in traj:
            base = row["bag_time"] if time_base == "bag" else row["stamp"]
            row["t"] = float(base) - t0
    for row in data.get("pl_decomp_rows", []):
        row["t"] = float(row.get("stamp", t0)) - t0


def finite_array(rows: list[dict[str, Any]], field: str, mask_large: bool = False) -> np.ndarray:
    values = np.asarray([float(row.get(field, np.nan)) for row in rows], dtype=float)
    values[~np.isfinite(values)] = np.nan
    if mask_large:
        values[np.abs(values) > 1.0e7] = np.nan
    return values


def field_array(rows: list[dict[str, Any]], field: str) -> np.ndarray:
    return np.asarray([row.get(field, np.nan) for row in rows])


def time_array(rows: list[dict[str, Any]]) -> np.ndarray:
    return np.asarray([float(row["t"]) for row in rows], dtype=float)


def ensure_axes_equal_3d(ax: Any, xyz_sets: list[np.ndarray]) -> None:
    all_xyz = [xyz for xyz in xyz_sets if xyz.size]
    if not all_xyz:
        return
    xyz = np.vstack(all_xyz)
    mins = np.nanmin(xyz, axis=0)
    maxs = np.nanmax(xyz, axis=0)
    centers = 0.5 * (mins + maxs)
    radius = 0.5 * np.nanmax(maxs - mins)
    if not np.isfinite(radius) or radius <= 0:
        radius = 1.0
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(max(0.0, centers[2] - radius), centers[2] + radius)


def plot_environment(data: dict[str, Any], figures_dir: Path) -> list[str]:
    outputs: list[str] = []
    clouds = data["clouds"]
    trajectories = data["trajectories"]
    cloud_colors = {
        "global": "#9aa0a6",
        "trunks": "#5b3f2a",
        "canopy": "#2e7d32",
        "terminal_wall": "#6a5acd",
    }
    traj_colors = {"truth": "#d62728", "iap": "#2ca02c", "desired": "#ffbf00"}

    if not clouds and not any(trajectories.values()):
        return outputs

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    xyz_sets: list[np.ndarray] = []
    for name, xyz in clouds.items():
        if xyz.size == 0:
            continue
        xyz_sets.append(xyz)
        ax.scatter(
            xyz[:, 0],
            xyz[:, 1],
            xyz[:, 2],
            s=1.0,
            alpha=0.25 if name == "global" else 0.55,
            c=cloud_colors.get(name, "#808080"),
            label=f"cloud:{name}",
        )
    for name, rows in trajectories.items():
        if not rows:
            continue
        traj = np.asarray([[r["x"], r["y"], r["z"]] for r in rows], dtype=float)
        xyz_sets.append(traj)
        ax.plot(
            traj[:, 0],
            traj[:, 1],
            traj[:, 2],
            lw=2.0,
            color=traj_colors.get(name, None),
            label=f"trajectory:{name}",
        )
    ensure_axes_equal_3d(ax, xyz_sets)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Experiment Environment 3D")
    ax.legend(loc="best", fontsize=8)
    path = figures_dir / "environment_3d.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(10, 8))
    for name, xyz in clouds.items():
        if xyz.size == 0:
            continue
        ax.scatter(
            xyz[:, 0],
            xyz[:, 1],
            s=1.0,
            alpha=0.25 if name == "global" else 0.55,
            c=cloud_colors.get(name, "#808080"),
            label=f"cloud:{name}",
        )
    for name, rows in trajectories.items():
        if not rows:
            continue
        traj = np.asarray([[r["x"], r["y"]] for r in rows], dtype=float)
        ax.plot(
            traj[:, 0],
            traj[:, 1],
            lw=2.0,
            color=traj_colors.get(name, None),
            label=f"trajectory:{name}",
        )
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Experiment Environment Top-Down")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    path = figures_dir / "environment_topdown.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))
    return outputs


def save_timeline_plots(
    rows: list[dict[str, Any]], aux: list[dict[str, Any]], figures_dir: Path
) -> list[str]:
    outputs: list[str] = []
    if not rows:
        return outputs
    t = time_array(rows)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, finite_array(rows, "hpl"), label="HPL")
    axes[0].plot(t, finite_array(rows, "hal"), "--", label="HAL")
    axes[0].set_ylabel("Horizontal [m]")
    axes[1].plot(t, finite_array(rows, "vpl"), label="VPL")
    axes[1].plot(t, finite_array(rows, "val"), "--", label="VAL")
    axes[1].set_ylabel("Vertical [m]")
    axes[2].plot(t, finite_array(rows, "im_h"), label="IM_h")
    axes[2].plot(t, finite_array(rows, "im_v"), label="IM_v")
    axes[2].plot(t, finite_array(rows, "im_min"), label="IM_min")
    axes[2].axhline(0.0, color="k", lw=1.0, alpha=0.5)
    axes[2].set_ylabel("Margin [m]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.suptitle("Fig 01: PL / AL / IM Timeline")
    path = figures_dir / "fig01_pl_al_im_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, finite_array(rows, "gnss_hpl", True), label="gnss_hpl")
    ax.plot(t, finite_array(rows, "lidar_hpl", True), label="lidar_hpl")
    ax.plot(t, finite_array(rows, "fallback_hpl", True), label="fallback_hpl")
    ax.plot(t, finite_array(rows, "hpl", True), lw=2.2, label="fused_hpl")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("HPL [m]")
    ax.set_title("Fig 02: GNSS / LiDAR / Fallback / Fused HPL Timeline")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    path = figures_dir / "fig02_hpl_sources_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, finite_array(rows, "gnss_vpl", True), label="gnss_vpl")
    ax.plot(t, finite_array(rows, "lidar_vpl", True), label="lidar_vpl")
    ax.plot(t, finite_array(rows, "fallback_vpl", True), label="fallback_vpl")
    ax.plot(t, finite_array(rows, "vpl", True), lw=2.2, label="fused_vpl")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("VPL [m]")
    ax.set_title("Fig 03: GNSS / LiDAR / Fallback / Fused VPL Timeline")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    path = figures_dir / "fig03_vpl_sources_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.step(t, field_array(rows, "final_hpl_source_code"), where="post", label="final_hpl_source")
    ax.step(t, field_array(rows, "final_vpl_source_code"), where="post", label="final_vpl_source")
    ax.set_yticks([0, 1, 2, 3, 4], ["UNKNOWN", "GNSS", "LIDAR", "FALLBACK", "CONSERVATIVE"])
    ax.set_xlabel("t [s]")
    ax.set_title("Fig 04: Final Source Timeline")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    path = figures_dir / "fig04_final_source_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 5.5))
    flags = [
        "gnss_valid",
        "lidar_valid",
        "fallback_valid",
        "gnss_araim_invalid",
        "lidar_integrity_invalid",
        "fallback_pl_invalid",
    ]
    for idx, field in enumerate(flags):
        values = finite_array(rows, field) + idx * 1.35
        ax.step(t, values, where="post", label=field)
    ax.set_yticks([idx * 1.35 + 0.5 for idx in range(len(flags))], flags)
    ax.set_xlabel("t [s]")
    ax.set_title("Fig 05: Validity Flags Timeline")
    ax.grid(True, axis="x", alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)
    path = figures_dir / "fig05_validity_flags_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].step(t, finite_array(rows, "gnss_n_det"), where="post", label="gnss_n_det")
    axes[0].step(t, finite_array(rows, "n_detected"), where="post", label="n_detected")
    axes[0].set_ylabel("detections")
    excluded_counts = np.asarray([len(a["excluded_prns_list"]) for a in aux], dtype=float)
    axes[1].step(t, excluded_counts, where="post", label="excluded PRN count")
    all_prns = sorted({int(p) for a in aux for p in a["excluded_prns_list"]})
    if 0 < len(all_prns) <= 20:
        for i, a in enumerate(aux):
            for prn in a["excluded_prns_list"]:
                axes[1].scatter(t[i], prn, s=10, alpha=0.7, color="#d62728")
        axes[1].set_ylabel("count / PRN id")
    else:
        axes[1].set_ylabel("excluded count")
    axes[2].plot(t, finite_array(rows, "gnss_hpl", True), label="gnss_hpl")
    axes[2].plot(t, finite_array(rows, "gnss_vpl", True), label="gnss_vpl")
    axes[2].set_ylabel("PL [m]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.suptitle("Fig 06: GNSS Fault Timeline")
    path = figures_dir / "fig06_gnss_fault_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, finite_array(rows, "n_sv_used"), label="n_sv_used")
    axes[1].plot(t, finite_array(rows, "n_constellations"), label="n_constellations")
    axes[2].plot(t, finite_array(rows, "pdop", True), label="pdop")
    axes[3].plot(t, finite_array(rows, "n_hypotheses"), label="n_hypotheses")
    axes[3].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.suptitle("Fig 09: Satellite and GNSS Geometry Timeline")
    path = figures_dir / "fig09_satellite_and_geometry_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, finite_array(rows, "n_trunks_observed"), label="n_trunks_observed")
    axes[1].plot(t, finite_array(rows, "tdop", True), label="tdop (sentinel >1e7 hidden)")
    axes[2].plot(t, finite_array(rows, "lidar_n_hyp"), label="lidar_n_hyp")
    modes = [str(row.get("lidar_worst_mode", "")) for row in rows]
    unique_modes = {mode: idx for idx, mode in enumerate(sorted(set(modes)))}
    axes[3].step(t, [unique_modes[m] for m in modes], where="post", label="lidar_worst_mode")
    axes[3].set_yticks(list(unique_modes.values()), list(unique_modes.keys()))
    axes[3].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best")
    fig.suptitle("Fig 10: LiDAR Geometry Timeline")
    path = figures_dir / "fig10_lidar_geometry_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, ax = plt.subplots(figsize=(12, 4.5))
    state = finite_array(rows, "integrity_state")
    ax.step(t, state, where="post", label="integrity_state")
    ax.set_yticks([0, 1, 2], ["SAFE", "SAFE_EXCLUDED", "UNSAFE"])
    ax.set_xlabel("t [s]")
    ax.set_title("Fig 11: Integrity State Timeline")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    path = figures_dir / "fig11_integrity_state_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    return outputs


def nearest_indices(source_t: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    if len(source_t) == 0:
        return np.zeros(len(query_t), dtype=int)
    idx = np.searchsorted(source_t, query_t)
    idx = np.clip(idx, 0, len(source_t) - 1)
    prev = np.clip(idx - 1, 0, len(source_t) - 1)
    use_prev = np.abs(source_t[prev] - query_t) < np.abs(source_t[idx] - query_t)
    idx[use_prev] = prev[use_prev]
    return idx


def choose_plot_trajectory(trajectories: dict[str, list[dict[str, float]]]) -> tuple[str, list[dict[str, float]]]:
    for name in ("truth", "iap", "desired"):
        if trajectories.get(name):
            return name, trajectories[name]
    return "", []


def plot_colored_trajectory(
    rows: list[dict[str, Any]], trajectories: dict[str, list[dict[str, float]]], figures_dir: Path
) -> list[str]:
    outputs: list[str] = []
    if not rows:
        return outputs
    traj_name, traj_rows = choose_plot_trajectory(trajectories)
    if not traj_rows:
        return outputs

    integrity_t = time_array(rows)
    traj_t = np.asarray([r["t"] for r in traj_rows], dtype=float)
    traj_xy = np.asarray([[r["x"], r["y"]] for r in traj_rows], dtype=float)
    nearest = nearest_indices(integrity_t, traj_t)

    im_values = finite_array(rows, "im_min")[nearest]
    fig, ax = plt.subplots(figsize=(10, 8))
    sc = ax.scatter(traj_xy[:, 0], traj_xy[:, 1], c=im_values, s=10, cmap="coolwarm")
    ax.plot(traj_xy[:, 0], traj_xy[:, 1], color="k", lw=0.6, alpha=0.35)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Fig 07: {traj_name} Trajectory Colored by IM_min")
    ax.grid(True, alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("IM_min [m]")
    path = figures_dir / "fig07_trajectory_colored_by_im.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    source_values = field_array(rows, "final_hpl_source_code")[nearest].astype(float)
    fig, ax = plt.subplots(figsize=(10, 8))
    sc = ax.scatter(
        traj_xy[:, 0],
        traj_xy[:, 1],
        c=source_values,
        s=10,
        cmap="viridis",
        vmin=0,
        vmax=4,
    )
    ax.plot(traj_xy[:, 0], traj_xy[:, 1], color="k", lw=0.6, alpha=0.35)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Fig 08: {traj_name} Trajectory Colored by Final HPL Source")
    ax.grid(True, alpha=0.25)
    cbar = fig.colorbar(sc, ax=ax, ticks=[0, 1, 2, 3, 4])
    cbar.ax.set_yticklabels(["UNKNOWN", "GNSS", "LIDAR", "FALLBACK", "CONSERVATIVE"])
    path = figures_dir / "fig08_trajectory_colored_by_source.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))
    return outputs


def axis_rows(rows: list[dict[str, Any]], axis: str) -> list[dict[str, Any]]:
    return [row for row in rows if str(row.get("axis", "")).upper() == axis]


def split_int_list(text: Any) -> list[int]:
    if text is None:
        return []
    value = str(text).strip()
    if not value:
        return []
    out: list[int] = []
    for item in value.split(";"):
        item = item.strip()
        if not item:
            continue
        try:
            out.append(int(item))
        except ValueError:
            continue
    return out


def rows_have_fields(rows: list[dict[str, Any]], fields: list[str]) -> bool:
    return bool(rows) and all(any(field in row for row in rows) for field in fields)


def plot_pl_decomposition(rows: list[dict[str, Any]], figures_dir: Path) -> list[str]:
    outputs: list[str] = []
    if not rows:
        return outputs

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for ax, axis, pl_field in zip(axes, ("H", "V"), ("HPL", "VPL")):
        selected = axis_rows(rows, axis)
        if not selected:
            ax.set_title(f"{axis} axis: no rows")
            continue
        t = time_array(selected)
        ax.plot(t, finite_array(selected, "term_d", True), label="term_d = |d|")
        ax.plot(t, finite_array(selected, "term_fa", True), label="term_fa = K_fa*sigma_ss")
        ax.plot(t, finite_array(selected, "term_md", True), label="term_md = K_md*sigma_k")
        ax.plot(t, finite_array(selected, "term_bias", True), label="term_bias")
        ax.plot(
            t,
            finite_array(selected, "total_reconstructed", True),
            lw=2.2,
            color="k",
            label="total_reconstructed",
        )
        ax.plot(t, finite_array(selected, pl_field, True), "--", lw=1.6, label=pl_field)
        ax.set_ylabel(f"{axis} terms [m]")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Fig 12: GNSS ARAIM PL Decomposition Terms")
    path = figures_dir / "fig12_pl_decomposition_terms_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    all_hyp_types = sorted({sanitize_hyp_type(row.get("hyp_type", "")) for row in rows})
    type_ticks = sorted({HYP_TYPE_CODE.get(name, 0) for name in all_hyp_types} | {0, 1, 2, 3, 4})
    type_labels = [
        next((name for name, code in HYP_TYPE_CODE.items() if code == tick and name), str(tick))
        for tick in type_ticks
    ]
    for ax, axis in zip(axes, ("H", "V")):
        selected = axis_rows(rows, axis)
        if not selected:
            ax.set_title(f"{axis} axis: no rows")
            continue
        t = time_array(selected)
        hyp_type_code = field_array(selected, "hyp_type_code").astype(float)
        hyp_id = finite_array(selected, "hyp_id")
        ax.step(t, hyp_type_code, where="post", label=f"{axis} hyp_type")
        ax2 = ax.twinx()
        ax2.step(t, hyp_id, where="post", color="#d62728", alpha=0.65, label=f"{axis} hyp_id")
        ax.set_yticks(type_ticks, type_labels)
        ax.set_ylabel("hyp_type")
        ax2.set_ylabel("hyp_id")
        ax.set_title(f"{axis} dominant hypothesis")
        ax.grid(True, alpha=0.25)
        lines, labels = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines + lines2, labels + labels2, loc="best", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Fig 13: GNSS ARAIM Dominant Hypothesis Timeline")
    path = figures_dir / "fig13_pl_decomposition_dominant_hypothesis.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    if not rows_have_fields(
        rows,
        [
            "n_removed_by_hyp",
            "n_remaining_after_hyp",
            "HDOP_full",
            "HDOP_subset",
            "VDOP_full",
            "VDOP_subset",
        ],
    ):
        return outputs

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, linestyle in (("H", "-"), ("V", "--")):
        selected = axis_rows(rows, axis)
        if not selected:
            continue
        t = time_array(selected)
        axes[0].step(
            t,
            finite_array(selected, "n_removed_by_hyp"),
            where="post",
            linestyle=linestyle,
            label=f"{axis} removed",
        )
        axes[0].step(
            t,
            finite_array(selected, "n_remaining_after_hyp"),
            where="post",
            linestyle=linestyle,
            label=f"{axis} remaining",
        )
        axes[1].plot(t, finite_array(selected, "HDOP_full", True), linestyle=linestyle, label=f"{axis} HDOP_full")
        axes[1].plot(t, finite_array(selected, "HDOP_subset", True), linestyle=linestyle, label=f"{axis} HDOP_subset")
        axes[2].plot(t, finite_array(selected, "VDOP_full", True), linestyle=linestyle, label=f"{axis} VDOP_full")
        axes[2].plot(t, finite_array(selected, "VDOP_subset", True), linestyle=linestyle, label=f"{axis} VDOP_subset")
    axes[0].set_ylabel("satellites")
    axes[1].set_ylabel("weighted HDOP [m]")
    axes[2].set_ylabel("weighted VDOP [m]")
    axes[2].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    fig.suptitle("Fig 14: Constellation Removal Geometry Timeline (weighted covariance-derived)")
    path = figures_dir / "fig14_constellation_removal_geometry_timeline.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))

    if not rows_have_fields(rows, ["dominant_const_name", "removed_prn_list"]):
        return outputs

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    const_ticks = sorted(set(CONSTELLATION_CODE.values()))
    const_labels = [
        next((name for name, code in CONSTELLATION_CODE.items() if code == tick and name), str(tick))
        for tick in const_ticks
    ]
    colors = {"H": "#1f77b4", "V": "#d62728"}
    for axis in ("H", "V"):
        selected = axis_rows(rows, axis)
        if not selected:
            continue
        t = time_array(selected)
        axes[0].step(
            t,
            field_array(selected, "dominant_const_name_code").astype(float),
            where="post",
            color=colors[axis],
            label=f"{axis} dominant constellation",
        )
        counts = np.asarray([len(split_int_list(row.get("removed_prn_list", ""))) for row in selected], dtype=float)
        axes[1].step(t, counts, where="post", color=colors[axis], label=f"{axis} removed PRN count")
        for i, row in enumerate(selected):
            for prn in split_int_list(row.get("removed_prn_list", "")):
                axes[1].scatter(t[i], prn, s=9, color=colors[axis], alpha=0.45)
    axes[0].set_yticks(const_ticks, const_labels)
    axes[0].set_ylabel("constellation")
    axes[1].set_ylabel("count / PRN id")
    axes[1].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
    fig.suptitle("Fig 15: Dominant Constellation and Removed PRNs")
    path = figures_dir / "fig15_dominant_constellation_and_prns.png"
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    outputs.append(str(path))
    return outputs


def write_pl_decomp_csv(rows: list[dict[str, Any]], out_path: Path) -> None:
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_integrity_csv(rows: list[dict[str, Any]], out_path: Path) -> None:
    fieldnames = [
        "t",
        "stamp",
        "bag_time",
        *INTEGRITY_FIELDS,
        "final_hpl_source_code",
        "final_vpl_source_code",
        "final_pl_source_code",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_trajectories_csv(trajectories: dict[str, list[dict[str, float]]], out_path: Path) -> None:
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f, fieldnames=["trajectory", "t", "stamp", "bag_time", "x", "y", "z"]
        )
        writer.writeheader()
        for name, rows in trajectories.items():
            for row in rows:
                writer.writerow({"trajectory": name, **row})


def stats(values: np.ndarray) -> dict[str, float | None]:
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"min": None, "max": None, "mean": None}
    return {
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "mean": float(np.mean(values)),
    }


def pl_decomp_summary(rows: list[dict[str, Any]], csv_path: str | None) -> dict[str, Any]:
    if not rows:
        return {
            "csv_path": csv_path,
            "row_count": 0,
            "axis_counts": {},
            "hyp_type_counts": {},
            "unsafe_reason_counts": {},
            "max_reconstruction_error": None,
            "term_mean_by_axis": {},
            "dominant_const_name_counts": {},
            "removed_count_stats": {"min": None, "max": None, "mean": None},
            "remaining_count_stats": {"min": None, "max": None, "mean": None},
            "subset_vdop_stats": {"min": None, "max": None, "mean": None},
            "subset_hdop_stats": {"min": None, "max": None, "mean": None},
            "dominant_removed_prns_top_counts": {},
        }

    errors = []
    for row in rows:
        total = float(row.get("total_reconstructed", np.nan))
        terms = sum(
            float(row.get(field, 0.0))
            for field in ("term_d", "term_fa", "term_md", "term_bias")
            if np.isfinite(float(row.get(field, np.nan)))
        )
        if np.isfinite(total) and np.isfinite(terms):
            errors.append(abs(total - terms))

    term_mean_by_axis: dict[str, dict[str, float | None]] = {}
    for axis in ("H", "V"):
        selected = axis_rows(rows, axis)
        term_mean_by_axis[axis] = {
            field: stats(finite_array(selected, field, True))["mean"]
            for field in ("term_d", "term_fa", "term_md", "term_bias", "total_reconstructed")
        }
    removed_prn_counts: Counter[int] = Counter()
    for row in rows:
        removed_prn_counts.update(split_int_list(row.get("removed_prn_list", "")))
    has_constellation_diag = rows_have_fields(rows, ["n_removed_by_hyp", "HDOP_subset"])

    return {
        "csv_path": csv_path,
        "row_count": len(rows),
        "axis_counts": dict(Counter(str(row.get("axis", "")) for row in rows)),
        "hyp_type_counts": dict(Counter(str(row.get("hyp_type", "")) for row in rows)),
        "unsafe_reason_counts": dict(Counter(str(row.get("unsafe_reason", "")) for row in rows)),
        "max_reconstruction_error": float(max(errors)) if errors else None,
        "term_mean_by_axis": term_mean_by_axis,
        "dominant_const_name_counts": dict(
            Counter(str(row.get("dominant_const_name", "")) for row in rows)
        ) if has_constellation_diag else {},
        "removed_count_stats": stats(finite_array(rows, "n_removed_by_hyp"))
        if has_constellation_diag else {"min": None, "max": None, "mean": None},
        "remaining_count_stats": stats(finite_array(rows, "n_remaining_after_hyp"))
        if has_constellation_diag else {"min": None, "max": None, "mean": None},
        "subset_vdop_stats": stats(finite_array(rows, "VDOP_subset", True))
        if has_constellation_diag else {"min": None, "max": None, "mean": None},
        "subset_hdop_stats": stats(finite_array(rows, "HDOP_subset", True))
        if has_constellation_diag else {"min": None, "max": None, "mean": None},
        "dominant_removed_prns_top_counts": {
            str(prn): count for prn, count in removed_prn_counts.most_common(20)
        } if has_constellation_diag else {},
    }


def make_summary(
    bag_dir: Path,
    output_dir: Path,
    data: dict[str, Any],
    figure_paths: list[str],
    t0: float,
    time_base: str,
) -> dict[str, Any]:
    rows = data["integrity_rows"]
    sources = Counter(str(row.get("final_hpl_source", "")) for row in rows)
    invalid_fields = [
        "fallback_pl_invalid",
        "gnss_araim_invalid",
        "lidar_integrity_invalid",
        "hal_invalid",
        "val_invalid",
        "im_invalid",
        "any_nan_rejected",
        "any_inf_rejected",
        "negative_variance_rejected",
        "degenerate_geometry",
    ]
    invalid_summary = {
        field: bool(any(int(row.get(field, 0)) for row in rows)) for field in invalid_fields
    }
    state_counts = Counter(int(row.get("integrity_state", -1)) for row in rows)
    state_counts_named = {
        STATE_CODE.get(code, str(code)): count for code, count in state_counts.items()
    }
    t_values = [float(row["t"]) for row in rows]
    all_numeric = []
    for row in rows:
        for field in INTEGRITY_FIELDS:
            value = row.get(field)
            if isinstance(value, (int, float)):
                all_numeric.append(float(value))
    numeric = np.asarray(all_numeric, dtype=float) if all_numeric else np.asarray([])

    return {
        "bag_dir": str(bag_dir),
        "output_dir": str(output_dir),
        "time_base": time_base,
        "t0": t0,
        "topic_counts": data["topic_counts"],
        "missing_topics": data["missing_topics"],
        "integrity_message_count": len(rows),
        "trajectory_message_counts": {
            name: len(values) for name, values in data["trajectories"].items()
        },
        "cloud_point_counts": {
            name: int(xyz.shape[0]) for name, xyz in data["clouds"].items()
        },
        "time_range_s": [min(t_values), max(t_values)] if t_values else [None, None],
        "hpl": stats(finite_array(rows, "hpl")) if rows else stats(np.asarray([])),
        "vpl": stats(finite_array(rows, "vpl")) if rows else stats(np.asarray([])),
        "im_min": stats(finite_array(rows, "im_min")) if rows else stats(np.asarray([])),
        "final_hpl_source_counts": dict(sources),
        "integrity_state_counts": state_counts_named,
        "fault_detection_frames": int(
            sum(1 for row in rows if int(row.get("gnss_n_det", 0)) > 0 or int(row.get("n_detected", 0)) > 0)
        ),
        "excluded_prn_frames": int(
            sum(1 for row in rows if str(row.get("excluded_prns", "")))
        ),
        "invalid_flags_seen": invalid_summary,
        "numeric_has_nan": bool(np.isnan(numeric).any()) if numeric.size else False,
        "numeric_has_inf": bool(np.isinf(numeric).any()) if numeric.size else False,
        "pl_decomposition": pl_decomp_summary(
            data.get("pl_decomp_rows", []), data.get("pl_decomp_csv_path")
        ),
        "figures": [os.path.relpath(path, output_dir) for path in figure_paths],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_dir", type=Path, help="rosbag2 directory to analyze")
    parser.add_argument("--output-subdir", default="araim_analysis")
    parser.add_argument("--max-cloud-points", type=int, default=200000)
    parser.add_argument(
        "--time-base",
        choices=("integrity", "bag", "ros_stamp"),
        default="integrity",
        help="time zero source; integrity uses first /iap/integrity header stamp",
    )
    parser.add_argument(
        "--pl-decomp-csv",
        type=Path,
        default=None,
        help=(
            "optional iap_araim_pl_decomp.csv from an IAP run log; if omitted, "
            "the analyzer checks a few common locations under bag_dir"
        ),
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="accepted for CLI compatibility; figures are saved without GUI by default",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_dir = args.bag_dir.expanduser().resolve()
    if not bag_dir.is_dir():
        raise SystemExit(f"bag_dir does not exist or is not a directory: {bag_dir}")

    output_dir = bag_dir / args.output_subdir
    figures_dir = output_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)

    data = collect_bag_data(bag_dir, max(1, args.max_cloud_points))
    pl_decomp_path = resolve_pl_decomp_csv(bag_dir, args.pl_decomp_csv)
    data["pl_decomp_csv_path"] = str(pl_decomp_path) if pl_decomp_path is not None else None
    data["pl_decomp_rows"] = read_pl_decomp_csv(pl_decomp_path) if pl_decomp_path is not None else []
    t0 = choose_t0(data, args.time_base)
    apply_time_base(data, t0, args.time_base)

    write_integrity_csv(data["integrity_rows"], output_dir / "integrity_timeseries.csv")
    write_trajectories_csv(data["trajectories"], output_dir / "trajectories.csv")
    write_pl_decomp_csv(data["pl_decomp_rows"], output_dir / PL_DECOMP_CSV_NAME)

    figure_paths: list[str] = []
    figure_paths.extend(plot_environment(data, figures_dir))
    figure_paths.extend(
        save_timeline_plots(data["integrity_rows"], data["integrity_aux"], figures_dir)
    )
    figure_paths.extend(
        plot_colored_trajectory(data["integrity_rows"], data["trajectories"], figures_dir)
    )
    figure_paths.extend(plot_pl_decomposition(data["pl_decomp_rows"], figures_dir))

    summary = make_summary(bag_dir, output_dir, data, figure_paths, t0, args.time_base)
    with (output_dir / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
        f.write("\n")

    print(f"Wrote analysis to: {output_dir}")
    print(f"Integrity rows: {len(data['integrity_rows'])}")
    print(f"PL decomposition rows: {len(data['pl_decomp_rows'])}")
    print(f"Figures: {len(figure_paths)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
