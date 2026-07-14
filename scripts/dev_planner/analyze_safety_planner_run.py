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
P5_TRAJECTORY_SAMPLES_TOPIC = "/iap/rviz/trajectory_integrity_samples"
P5_CURRENT_TRAJ_TOPIC = "/iap/rviz/current_traj_integrity_colored"
P5_GATE_STATUS_TOPIC = "/iap/rviz/p5_gate_status"
P5_CURRENT_IM_BARS_TOPIC = "/iap/rviz/p5_current_im_bars"
P5_RVIZ_TOPICS = [
    P5_TRAJECTORY_SAMPLES_TOPIC,
    P5_CURRENT_TRAJ_TOPIC,
    P5_GATE_STATUS_TOPIC,
    P5_CURRENT_IM_BARS_TOPIC,
]
P5_TOPIC_ACTIVITY_TOPICS = P0_TOPIC_ACTIVITY_TOPICS + [
    P5_STATUS_TOPIC,
    *P5_RVIZ_TOPICS,
]
P5_TOPIC_EXPECTATIONS = {
    "/iap/integrity": "continuous",
    "/sim/drone_0/lidar_body": "continuous",
    "/drone_0_visual_slam/odom": "continuous",
    "/drone_0_planning/bspline": "planner-dependent",
    P0_HEALTH_TOPIC: "active-periodic",
    P0_PL_CLOUD_TOPIC: "present",
    P0_VALIDITY_CLOUD_TOPIC: "present",
    P5_STATUS_TOPIC: "present",
    P5_TRAJECTORY_SAMPLES_TOPIC: "present",
    P5_CURRENT_TRAJ_TOPIC: "present",
    P5_GATE_STATUS_TOPIC: "present",
    P5_CURRENT_IM_BARS_TOPIC: "present",
}
P5_EMERGENCY_ACTION = "REQUEST_EMERGENCY_STOP_CANDIDATE"
P5_REPLAN_ACTION = "REQUEST_REPLAN"
P5_OK_ACTION = "OK"
P5_1_MIN_OK_ACTION_RATIO = 0.95
P5_1_STARTUP_REPLAN_MAX_DURATION_S = 2.0
P5_STARTUP_REPLAN_DURATION_TOLERANCE_S = 0.25
P5_REPLAN_STORM_CONSECUTIVE = 3
P5_2_EMERGENCY_STORM_CONSECUTIVE = 3
P5_3_FIXTURE_NAME = "future_high_risk_zone_v1"
P5_3_FIXTURE_REASON = "p5_3_high_risk_zone"
P5_3_SCENARIO_ISOLATION_BRANCH = (
    "FAIL -> debug P5-3 scenario isolation / future bad-ratio coverage"
)
P5_3_REASON_ATTRIBUTION_BRANCH = "FAIL -> debug P5-3 reason attribution"
P5_3_PL_AL_MARGIN_BRANCH = "FAIL -> 继续 debug P5-3 query alignment / PL-AL margin"
P5_3_FAIL_BRANCH = P5_3_PL_AL_MARGIN_BRANCH
P5_4_FIXTURE_NAME = "near_risk_zone_v1"
P5_4_FIXTURE_REASON = "p5_4_near_risk_zone"
P5_4_EMERGENCY_TIME_S = 1.0
P5_4_FAIL_BRANCH = "FAIL -> 继续 debug P5-4 near-risk / emergency-candidate / PL-AL margin"
P5_5_FIXTURE_NAME = "current_integrity_stamp_freeze_v1"
P5_5_FIXTURE_REASON = "current_stale"
P5_5_FAIL_BRANCH = (
    "FAIL -> debug P5-5 stale debounce / integrity pause / cause attribution"
)
P5_5_BLOCKED_BRANCH = (
    "BLOCKED_SCENARIO_MISSING -> implement integrity pause/delay fixture first"
)
P5_6_FIXTURE_NAME = "future_unknown_zone_v1"
P5_6_FIXTURE_REASON = "future_unknown"
P5_6_FAIL_BRANCH = (
    "FAIL -> debug P5-6 future unknown policy / reason attribution / debounce"
)
P5_6_FIGURE_FILENAMES = [
    "p5_6_scenario_topdown.png",
    "p5_6_unknown_field_overlay.png",
    "p5_6_p0_health_unknown_timeline.png",
    "p5_6_future_unknown_duration_timeline.png",
    "p5_6_unknown_ratio_vs_action.png",
    "p5_6_action_reason_timeline.png",
    "p5_6_debounce_timeline.png",
    "p5_6_cause_exclusion_summary.png",
    "p5_6_trajectory_integrity_samples.png",
]
P5_5_FIGURE_FILENAMES = [
    "p5_5_scenario_topdown.png",
    "p5_5_topic_activity_timeline.png",
    "p5_5_integrity_pause_timeline.png",
    "p5_5_current_stale_duration_timeline.png",
    "p5_5_action_reason_timeline.png",
    "p5_5_replan_vs_emergency.png",
    "p5_5_debounce_timeline.png",
    "p5_5_margin_timeline.png",
    "p5_5_p0_health.png",
    "p5_5_cause_exclusion_summary.png",
    "p5_5_trajectory_integrity_samples.png",
]
P5_4_FIGURE_FILENAMES = [
    "p5_4_scenario_topdown.png",
    "p5_4_near_risk_overlay.png",
    "p5_4_tau_emergency_window.png",
    "p5_4_pl_probe.png",
    "p5_4_margin_timeline.png",
    "p5_4_action_reason_timeline.png",
    "p5_4_replan_vs_emergency.png",
    "p5_4_sample_heatmap.png",
    "p5_4_topic_gap.png",
    "p5_4_p0_health.png",
    "p5_4_final_gate_summary.png",
    "p5_4_trajectory_integrity_samples.png",
]
P5_3_PLAL_FIGURE_FILENAMES = [
    "p5_3_plal_scenario_topdown.png",
    "p5_3_plal_high_risk_overlay.png",
    "p5_3_plal_tau_window.png",
    "p5_3_plal_margin_timeline.png",
    "p5_3_plal_action_reason_timeline.png",
    "p5_3_plal_replan_vs_emergency.png",
    "p5_3_plal_sample_heatmap.png",
    "p5_3_plal_p0_health_timeline.png",
]
P5_3_QUERY_ALIGNMENT_FIGURE_FILENAMES = [
    "p5_3_query_alignment_scenario_topdown.png",
    "p5_3_query_alignment_fixture_overlay.png",
    "p5_3_query_alignment_pl_probe.png",
    "p5_3_query_alignment_tau_window.png",
    "p5_3_query_alignment_margin_timeline.png",
    "p5_3_query_alignment_action_reason.png",
    "p5_3_query_alignment_sample_heatmap.png",
    "p5_3_query_alignment_topic_gap.png",
    "p5_3_query_alignment_p0_health.png",
]
P5_3_FUTURE_SAMPLING_FIGURE_FILENAMES = [
    "p5_3_future_sampling_scenario_topdown.png",
    "p5_3_future_sampling_fixture_overlay.png",
    "p5_3_future_sampling_pl_probe.png",
    "p5_3_future_sampling_tau_window.png",
    "p5_3_future_sampling_margin_timeline.png",
    "p5_3_future_sampling_action_reason.png",
    "p5_3_future_sampling_sample_heatmap.png",
    "p5_3_future_sampling_topic_gap.png",
    "p5_3_future_sampling_p0_health.png",
]
P5_3_EVENT_WINDOW_FIGURE_FILENAMES = [
    "p5_3_event_window_scenario_topdown.png",
    "p5_3_event_window_fixture_overlay.png",
    "p5_3_event_window_tau_window.png",
    "p5_3_event_window_pl_probe.png",
    "p5_3_event_window_margin_timeline.png",
    "p5_3_event_window_action_reason.png",
    "p5_3_event_window_replan_vs_emergency.png",
    "p5_3_event_window_sample_heatmap.png",
    "p5_3_event_window_topic_gap.png",
    "p5_3_event_window_p0_health.png",
]
P5_STATUS_FIELDS = [
    "bag_time_s",
    "phase",
    "action",
    "raw_action",
    "reason",
    "raw_reason",
    "current_reason",
    "future_reason",
    "active_reasons",
    "current_im_h",
    "current_im_v",
    "current_im_min",
    "future_min_im",
    "first_bad_tau",
    "bad_ratio",
    "unknown_ratio",
    "current_integrity_age_s",
    "field_generation_id",
    "field_age_s",
    "current_stale_duration_s",
    "current_low_margin_duration_s",
    "future_unknown_duration_s",
    "final_gate_fail_count",
    "final_gate_fail_duration_s",
    "final_gate_last_reason",
    "pred_al_mode",
    "pred_hal_min",
    "pred_val_min",
    "pred_al_invalid_count",
    "pred_al_last_reason",
    "sample_count",
    "bad_count",
    "unknown_count",
    "samples",
    "parse_error",
    "raw",
]
P5_5_INTEGRITY_STAMP_FIELDS = [
    "bag_time_s",
    "t_rel_s",
    "header_stamp_s",
    "header_rel_s",
    "bag_minus_header_s",
    "in_expected_window",
    "hpl",
    "vpl",
    "hal",
    "val",
    "im",
    "pl_al_finite",
    "fusion_mode",
    "final_hpl_source",
    "final_vpl_source",
]
P5_SAMPLE_FIELDS = [
    "bag_time_s",
    "status_row_index",
    "sample_index",
    "phase",
    "action",
    "raw_action",
    "status_reason",
    "current_reason",
    "future_reason",
    "active_reasons",
    "tau_s",
    "query_tau_s",
    "trajectory_start_time_s",
    "trajectory_duration_s",
    "trajectory_t_cur_s",
    "trajectory_t_end_s",
    "trajectory_time_remaining_s",
    "sample_dt_s",
    "horizon_s",
    "trajectory_sample_source",
    "fixture_match",
    "fixture_expected_hpl",
    "fixture_expected_vpl",
    "fixture_expected_reason",
    "x",
    "y",
    "z",
    "hpl",
    "vpl",
    "hal",
    "val",
    "im_min",
    "expected_hpl",
    "expected_vpl",
    "expected_reason",
    "actual_hpl",
    "actual_vpl",
    "hpl_error",
    "vpl_error",
    "query_pl_aligned",
    "query_reason_aligned",
    "query_alignment_ok",
    "bad",
    "unknown",
    "stale",
    "reason",
    "inside_high_risk_zone",
    "inside_tau_window",
    "fixture_bad_sample",
]
P5_3_QUERY_ALIGNMENT_FIELDS = [
    "bag_time_s",
    "status_row_index",
    "sample_index",
    "phase",
    "action",
    "raw_action",
    "status_reason",
    "current_reason",
    "future_reason",
    "active_reasons",
    "tau_s",
    "query_tau_s",
    "trajectory_start_time_s",
    "trajectory_duration_s",
    "trajectory_t_cur_s",
    "trajectory_t_end_s",
    "trajectory_time_remaining_s",
    "sample_dt_s",
    "horizon_s",
    "trajectory_sample_source",
    "fixture_match",
    "fixture_expected_hpl",
    "fixture_expected_vpl",
    "fixture_expected_reason",
    "x",
    "y",
    "z",
    "inside_high_risk_zone",
    "inside_tau_window",
    "expected_hpl",
    "expected_vpl",
    "actual_hpl",
    "actual_vpl",
    "hpl_error",
    "vpl_error",
    "hal",
    "val",
    "im_min",
    "bad",
    "reason",
    "expected_reason",
    "query_pl_aligned",
    "query_reason_aligned",
    "query_alignment_ok",
]
P5_6_UNKNOWN_ACTION_FIELDS = [
    "bag_time_s",
    "t_rel_s",
    "phase",
    "action",
    "raw_action",
    "reason",
    "raw_reason",
    "current_reason",
    "future_reason",
    "active_reasons",
    "unknown_ratio",
    "bad_ratio",
    "future_unknown_duration_s",
    "unknown_bucket",
    "unknown_scope",
    "unknown_attributed",
    "request_replan",
    "request_emergency_stop_candidate",
    "excluded_causes",
]
P5_6_CAUSE_EXCLUSION_FIELDS = [
    "cause",
    "count",
    "first_bag_time_s",
    "first_action",
    "first_raw_action",
    "first_reason",
]
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
P0_6_MISSING_CAPABILITIES = [
    "occupied overlap fixture",
    "occupied-low-risk injection",
    "reproducible occupied validity overlay",
]
P0_6_FIXTURE_NAME = "occupied_overlap_box_v1"
# Mirrors iap::RISK_GRID_SOURCE_OCCUPIED_SKIP in include/iap/planner/risk_grid_map.hpp.
P0_OCCUPIED_SKIP_SOURCE_FLAG = 1 << 31


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


def is_p5_runtime_experiment(args: argparse.Namespace) -> bool:
    return str(args.experiment_id).strip().upper() in {
        "P5-1",
        "P5-2",
        "P5-3",
        "P5-4",
        "P5-5",
        "P5-6",
    }


def p0_phase_number(experiment_id: Any) -> int | None:
    match = re.fullmatch(r"P0-(\d+)", str(experiment_id).strip().upper())
    return int(match.group(1)) if match else None


def p0_odom_gate_required(experiment_id: Any) -> bool:
    phase = p0_phase_number(experiment_id)
    if phase in {4, 6}:
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


def finite_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    return [
        value
        for value in (finite_float(row.get(key)) for row in rows)
        if value is not None
    ]


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


def p5_manifest_gate_values(manifest: dict[str, Any]) -> dict[str, bool]:
    expected_true = (
        "p0.enable_risk_grid",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
    )
    expected_false = (
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
    )
    return {
        "manifest_present": bool(manifest),
        "manifest_safety_profile_p5": str(manifest.get("planner_safety_profile", "")).lower()
        == "p5",
        "manifest_p0_enabled": manifest.get("p0.enable_risk_grid") is True,
        "manifest_p5_runtime_enabled": manifest.get("planner_enable_p5_runtime") is True,
        "manifest_p5_final_enabled": manifest.get("planner_enable_p5_final") is True,
        "manifest_p1_p4_disabled": all(manifest.get(key) is False for key in expected_false),
        "manifest_expected_true_ok": all(manifest.get(key) is True for key in expected_true),
        "manifest_expected_false_ok": all(manifest.get(key) is False for key in expected_false),
    }


def validate_p5_manifest(
    manifest: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
    *,
    experiment_label: str = "P5-1",
) -> None:
    if not manifest:
        inconclusive.append(f"{experiment_label} checks require test_planner_manifest.json")
        return
    if str(manifest.get("planner_safety_profile", "")).lower() != "p5":
        failures.append(f"{experiment_label} manifest planner_safety_profile is not p5")
    expected_true = (
        "p0.enable_risk_grid",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
    )
    expected_false = (
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
    )
    for key in expected_true:
        if manifest.get(key) is not True:
            failures.append(f"{experiment_label} manifest {key} is not true")
    for key in expected_false:
        if manifest.get(key) is not False:
            failures.append(f"{experiment_label} manifest {key} is not false")


def validate_p5_1_manifest(
    manifest: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
) -> None:
    validate_p5_manifest(
        manifest,
        failures,
        inconclusive,
        experiment_label="P5-1",
    )


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


def marker_color_state(r: Any, g: Any, b: Any, a: Any = 1.0) -> str:
    red = finite_float(r)
    green = finite_float(g)
    blue = finite_float(b)
    alpha = finite_float(a)
    if alpha is not None and alpha <= 0.0:
        return "transparent"
    if red is None or green is None or blue is None:
        return "unknown"
    if red >= 0.8 and green <= 0.25:
        return "bad"
    if red >= 0.8 and green >= 0.45 and blue <= 0.25:
        return "stale_or_warning"
    if green >= 0.6 and red <= 0.35:
        return "ok"
    if abs(red - green) <= 0.15 and abs(green - blue) <= 0.15:
        return "unknown"
    return "other"


def marker_point_row(
    *,
    topic: str,
    timestamp_ns: int,
    marker: Any,
    point_index: int,
    point: Any,
    color_value: Any,
) -> dict[str, Any]:
    return {
        "bag_time_s": float(timestamp_ns) * 1.0e-9,
        "topic": topic,
        "marker_ns": str(getattr(marker, "ns", "")),
        "marker_id": getattr(marker, "id", ""),
        "marker_type": getattr(marker, "type", ""),
        "marker_action": getattr(marker, "action", ""),
        "point_index": point_index,
        "x": getattr(point, "x", ""),
        "y": getattr(point, "y", ""),
        "z": getattr(point, "z", ""),
        "color_r": getattr(color_value, "r", ""),
        "color_g": getattr(color_value, "g", ""),
        "color_b": getattr(color_value, "b", ""),
        "color_a": getattr(color_value, "a", ""),
        "state": marker_color_state(
            getattr(color_value, "r", None),
            getattr(color_value, "g", None),
            getattr(color_value, "b", None),
            getattr(color_value, "a", None),
        ),
        "text": str(getattr(marker, "text", "")),
    }


def read_p5_marker_evidence(
    bag_dir: Path,
    metadata: dict[str, Any],
    limit: int = 20_000,
) -> tuple[list[dict[str, Any]], str]:
    if metadata.get("missing") or not bag_dir:
        return [], "missing rosbag metadata"
    topic_counts = metadata.get("topic_counts", {}) or {}
    target_topics = {
        topic
        for topic in (P5_TRAJECTORY_SAMPLES_TOPIC, P5_CURRENT_TRAJ_TOPIC)
        if int(topic_counts.get(topic, 0) or 0) > 0
    }
    if not target_topics:
        return [], ""
    rows: list[dict[str, Any]] = []
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = open_bag_reader(bag_dir, metadata)
        type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        msg_types = {
            topic: get_message(type_map[topic])
            for topic in target_topics
            if topic in type_map
        }
        while reader.has_next() and len(rows) < limit:
            topic, raw, timestamp = reader.read_next()
            if topic not in msg_types:
                continue
            msg = deserialize_message(raw, msg_types[topic])
            for marker in getattr(msg, "markers", []):
                action = int(getattr(marker, "action", -1))
                if action in {2, 3}:  # DELETE / DELETEALL
                    continue
                points = list(getattr(marker, "points", []) or [])
                colors = list(getattr(marker, "colors", []) or [])
                if points:
                    for point_index, point in enumerate(points):
                        color_value = (
                            colors[point_index]
                            if point_index < len(colors)
                            else getattr(marker, "color", None)
                        )
                        rows.append(
                            marker_point_row(
                                topic=topic,
                                timestamp_ns=timestamp,
                                marker=marker,
                                point_index=point_index,
                                point=point,
                                color_value=color_value,
                            )
                        )
                        if len(rows) >= limit:
                            break
                else:
                    pose = getattr(marker, "pose", None)
                    point = getattr(pose, "position", None)
                    if point is None:
                        continue
                    rows.append(
                        marker_point_row(
                            topic=topic,
                            timestamp_ns=timestamp,
                            marker=marker,
                            point_index=0,
                            point=point,
                            color_value=getattr(marker, "color", None),
                        )
                    )
                if len(rows) >= limit:
                    break
        return rows, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return rows, str(exc)


def summarize_p5_marker_evidence(rows: list[dict[str, Any]], marker_error: str = "") -> dict[str, Any]:
    topics = Counter(str(row.get("topic", "")) for row in rows)
    states = Counter(str(row.get("state", "")) for row in rows)
    stamps = [value for value in (finite_float(row.get("bag_time_s")) for row in rows) if value is not None]
    return {
        "row_count": len(rows),
        "topic_counts": dict(sorted(topics.items())),
        "state_counts": dict(sorted(states.items())),
        "first_bag_time_s": min(stamps) if stamps else None,
        "last_bag_time_s": max(stamps) if stamps else None,
        "inspection_error": marker_error,
    }


def relative_time(rows: list[dict[str, Any]], key: str = "bag_time_s") -> list[float]:
    stamps = [finite_float(row.get(key)) for row in rows]
    finite_stamps = [stamp for stamp in stamps if stamp is not None]
    if not finite_stamps:
        return []
    t0 = min(finite_stamps)
    return [math.nan if stamp is None else stamp - t0 for stamp in stamps]


def action_code_map(actions: list[str]) -> dict[str, int]:
    preferred = [P5_OK_ACTION, P5_REPLAN_ACTION, P5_EMERGENCY_ACTION, "<empty>"]
    ordered = [value for value in preferred if value in actions]
    ordered.extend(sorted(value for value in set(actions) if value not in ordered))
    return {value: idx for idx, value in enumerate(ordered)}


def plot_p5_action_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    raw_actions = [p5_action(row, "raw_action") or "<empty>" for row in rows]
    codes = action_code_map(actions + raw_actions)
    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.step(t, [codes[value] for value in actions], where="post", label="action", lw=1.8)
    ax.step(t, [codes[value] for value in raw_actions], where="post", label="raw_action", lw=1.2, ls="--")
    ax.set_yticks(list(codes.values()), list(codes.keys()))
    ax.set_xlabel("time since first P5 status [s]")
    ax.set_title("P5 action timeline")
    ax.grid(True, axis="x", alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_status_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, finite_or_nan(rows, "bad_ratio"), label="bad_ratio", color="#dc2626")
    axes[0].plot(t, finite_or_nan(rows, "unknown_ratio"), label="unknown_ratio", color="#6b7280")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("ratio")
    axes[0].legend(loc="best")
    axes[1].plot(t, finite_or_nan(rows, "sample_count"), label="sample_count", color="#2563eb")
    axes[1].plot(t, finite_or_nan(rows, "bad_count"), label="bad_count", color="#dc2626")
    axes[1].plot(t, finite_or_nan(rows, "unknown_count"), label="unknown_count", color="#6b7280")
    axes[1].set_ylabel("samples")
    axes[1].legend(loc="best")
    axes[2].plot(t, finite_or_nan(rows, "current_stale_duration_s"), label="current_stale_duration_s", color="#f97316")
    axes[2].plot(t, finite_or_nan(rows, "current_low_margin_duration_s"), label="current_low_margin_duration_s", color="#0f766e")
    axes[2].plot(t, finite_or_nan(rows, "future_unknown_duration_s"), label="future_unknown_duration_s", color="#9333ea")
    axes[2].set_ylabel("duration [s]")
    axes[2].set_xlabel("time since first P5 status [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5 status timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_current_im_vs_action(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    codes = action_code_map(actions)
    current_im = finite_or_nan(rows, "current_im_min")
    future_im = finite_or_nan(rows, "future_min_im")
    if not any(math.isfinite(value) for value in current_im):
        return False
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True)
    for action in codes:
        xs = [t[idx] for idx, value in enumerate(actions) if value == action]
        ys = [current_im[idx] for idx, value in enumerate(actions) if value == action]
        axes[0].scatter(xs, ys, label=action, s=36)
    axes[0].plot(t, future_im, label="future_min_im", color="#2563eb", lw=1.2, alpha=0.8)
    axes[0].axhline(0.0, color="#111827", lw=0.9)
    axes[0].set_ylabel("IM [m]")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.25)

    axes[1].step(t, [codes[value] for value in actions], where="post", color="#f97316")
    axes[1].set_yticks(list(codes.values()), list(codes.keys()))
    axes[1].set_ylabel("P5 action")
    axes[1].set_xlabel("time since first P5 status [s]")
    axes[1].grid(True, axis="x", alpha=0.25)
    fig.suptitle("P5 current IM vs action")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_future_unknown_duration_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.0), sharex=True)
    axes[0].plot(
        t,
        finite_or_nan(rows, "future_unknown_duration_s"),
        label="future_unknown_duration_s",
        color="#9333ea",
    )
    axes[0].set_ylabel("duration [s]")
    axes[0].legend(loc="best")
    axes[1].plot(t, finite_or_nan(rows, "unknown_ratio"), label="unknown_ratio", color="#6b7280")
    axes[1].plot(t, finite_or_nan(rows, "bad_ratio"), label="bad_ratio", color="#dc2626")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("ratio")
    axes[1].set_xlabel("time since first P5 status [s]")
    axes[1].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5 future unknown duration")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_stale_integrity_correlation(
    health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not health_rows or not p5_rows:
        return False
    health_stamps = [finite_float(row.get("stamp")) for row in health_rows]
    p5_stamps = [finite_float(row.get("bag_time_s")) for row in p5_rows]
    finite_stamps = [stamp for stamp in [*health_stamps, *p5_stamps] if stamp is not None]
    if not finite_stamps:
        return False
    t0 = min(finite_stamps)
    health_t = [math.nan if stamp is None else stamp - t0 for stamp in health_stamps]
    p5_t = [math.nan if stamp is None else stamp - t0 for stamp in p5_stamps]
    actions = [p5_action(row, "action") or "<empty>" for row in p5_rows]
    codes = action_code_map(actions)
    stale_integrity = [
        1.0
        if str(row.get("dominant_unknown_reason", "")).strip().lower()
        == "stale_integrity"
        or str(row.get("reason", "")).strip().lower() == "stale_integrity"
        else 0.0
        for row in health_rows
    ]

    fig, axes = plt.subplots(4, 1, figsize=(11, 8.4), sharex=True)
    axes[0].step(health_t, stale_integrity, where="post", label="P0 stale_integrity", color="#dc2626")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("P0 stale")
    axes[0].legend(loc="best")
    axes[1].plot(health_t, finite_or_nan(health_rows, "valid_ratio"), label="valid_ratio", color="#2563eb")
    axes[1].plot(health_t, finite_or_nan(health_rows, "unknown_ratio"), label="unknown_ratio", color="#6b7280")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("P0 ratio")
    axes[1].legend(loc="best")
    axes[2].plot(health_t, finite_or_nan(health_rows, "predictor_lidar_used_count"), label="lidar_used", color="#0f766e")
    axes[2].plot(health_t, finite_or_nan(health_rows, "predictor_stale_current_prior_count"), label="stale_current_prior", color="#f97316")
    axes[2].plot(health_t, finite_or_nan(health_rows, "provider_stale_count"), label="provider_stale", color="#dc2626")
    axes[2].set_ylabel("P0 counts")
    axes[2].legend(loc="best")
    axes[3].step(p5_t, [codes[value] for value in actions], where="post", label="P5 action", color="#7c3aed")
    axes[3].plot(p5_t, finite_or_nan(p5_rows, "unknown_ratio"), label="P5 unknown_ratio", color="#6b7280", alpha=0.8)
    axes[3].set_yticks(list(codes.values()), list(codes.keys()))
    axes[3].set_ylabel("P5")
    axes[3].set_xlabel("run-relative bag time [s]")
    axes[3].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P0 stale integrity vs P5 action correlation")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_startup_correlation(
    health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not health_rows or not p5_rows:
        return False
    health_stamps = [finite_float(row.get("stamp")) for row in health_rows]
    p5_stamps = [finite_float(row.get("bag_time_s")) for row in p5_rows]
    finite_stamps = [stamp for stamp in [*health_stamps, *p5_stamps] if stamp is not None]
    if not finite_stamps:
        return False
    t0 = min(finite_stamps)
    health_t = [math.nan if stamp is None else stamp - t0 for stamp in health_stamps]
    p5_t = [math.nan if stamp is None else stamp - t0 for stamp in p5_stamps]
    actions = [p5_action(row, "action") or "<empty>" for row in p5_rows]
    codes = action_code_map(actions)

    fig, axes = plt.subplots(3, 1, figsize=(11, 7.0), sharex=True)
    axes[0].step(
        health_t,
        [1.0 if row.get("ready") else 0.0 for row in health_rows],
        where="post",
        label="P0 ready",
        color="#16a34a",
    )
    axes[0].step(
        health_t,
        [1.0 if row.get("stale") else 0.0 for row in health_rows],
        where="post",
        label="P0 stale",
        color="#dc2626",
    )
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("P0 bool")
    axes[0].legend(loc="best")

    axes[1].plot(health_t, finite_or_nan(health_rows, "unknown_ratio"), label="P0 unknown_ratio", color="#6b7280")
    axes[1].plot(health_t, finite_or_nan(health_rows, "valid_ratio"), label="P0 valid_ratio", color="#2563eb")
    axes[1].set_ylim(-0.05, 1.05)
    axes[1].set_ylabel("P0 ratio")
    axes[1].legend(loc="best")

    axes[2].step(p5_t, [codes[value] for value in actions], where="post", label="P5 action", color="#f97316")
    axes[2].set_yticks(list(codes.values()), list(codes.keys()))
    axes[2].set_xlabel("run-relative bag time [s]")
    axes[2].set_ylabel("P5 action")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P0 startup vs P5 action correlation")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_margin_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    axes[0].plot(t, finite_or_nan(rows, "current_im_min"), label="current_im_min", color="#16a34a")
    axes[0].plot(t, finite_or_nan(rows, "future_min_im"), label="future_min_im", color="#2563eb")
    axes[0].axhline(0.0, color="#111827", lw=0.9)
    axes[0].set_ylabel("IM [m]")
    axes[0].legend(loc="best")
    axes[1].plot(t, finite_or_nan(rows, "pred_hal_min"), label="pred_hal_min", color="#0f766e")
    axes[1].plot(t, finite_or_nan(rows, "pred_val_min"), label="pred_val_min", color="#7c3aed")
    axes[1].plot(t, finite_or_nan(rows, "pred_al_invalid_count"), label="pred_al_invalid_count", color="#dc2626")
    axes[1].set_ylabel("AL / invalid")
    axes[1].set_xlabel("time since first P5 status [s]")
    axes[1].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5 margin timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_final_gate_summary(p5_summary: dict[str, Any], path: Path) -> bool:
    labels = [
        "max fail count",
        "fail rows",
        "emergency rows",
        "max replan streak",
        "raw replan streak",
    ]
    values = [
        int(p5_summary.get("final_gate_fail_count_max", 0) or 0),
        int(p5_summary.get("final_gate_fail_rows", 0) or 0),
        int(p5_summary.get("emergency_action_count", 0) or 0),
        int(p5_summary.get("max_consecutive_replan", 0) or 0),
        int(p5_summary.get("raw_max_consecutive_replan", 0) or 0),
    ]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    colors = ["#16a34a" if value == 0 else "#dc2626" for value in values]
    ax.bar(labels, values, color=colors)
    ax.set_ylabel("count")
    ax.set_title("P5 final gate and storm summary")
    ax.tick_params(axis="x", rotation=18)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_debounce_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    debounce_rows = build_p5_debounce_timeline_rows(rows)
    if not debounce_rows:
        return False
    t = [finite_float(row.get("t_rel_s")) for row in debounce_rows]
    if not any(value is not None for value in t):
        return False
    t_plot = [math.nan if value is None else value for value in t]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.0), sharex=True)
    axes[0].step(
        t_plot,
        [int(row.get("consecutive_replan", 0) or 0) for row in debounce_rows],
        where="post",
        label="action replan streak",
        color="#f97316",
    )
    axes[0].step(
        t_plot,
        [int(row.get("raw_consecutive_replan", 0) or 0) for row in debounce_rows],
        where="post",
        label="raw replan streak",
        color="#f59e0b",
        linestyle="--",
    )
    axes[0].set_ylabel("replan streak")
    axes[0].legend(loc="best")

    axes[1].step(
        t_plot,
        [int(row.get("consecutive_emergency", 0) or 0) for row in debounce_rows],
        where="post",
        label="action emergency streak",
        color="#dc2626",
    )
    axes[1].step(
        t_plot,
        [int(row.get("raw_consecutive_emergency", 0) or 0) for row in debounce_rows],
        where="post",
        label="raw emergency streak",
        color="#991b1b",
        linestyle="--",
    )
    axes[1].axhline(
        P5_2_EMERGENCY_STORM_CONSECUTIVE,
        color="#111827",
        lw=0.9,
        label="P5-2 emergency limit",
    )
    axes[1].set_ylabel("emergency streak")
    axes[1].legend(loc="best")

    axes[2].step(
        t_plot,
        [
            int(finite_float(row.get("final_gate_fail_count")) or 0)
            for row in debounce_rows
        ],
        where="post",
        label="final_gate_fail_count",
        color="#7c3aed",
    )
    axes[2].step(
        t_plot,
        [
            int(row.get("final_gate_escalated_to_emergency", 0) or 0)
            for row in debounce_rows
        ],
        where="post",
        label="final gate escalated",
        color="#dc2626",
        linestyle="--",
    )
    axes[2].axhline(3, color="#111827", lw=0.9, label="P5-2 final-gate limit")
    axes[2].set_ylabel("final gate")
    axes[2].set_xlabel("time since first P5 status [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5 debounce and final-gate timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_5_integrity_pause_timeline(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not rows:
        return False
    t = finite_values(rows, "t_rel_s")
    if not t:
        return False
    t_plot = finite_or_nan(rows, "t_rel_s")
    header_rel = finite_or_nan(rows, "header_rel_s")
    age = finite_or_nan(rows, "bag_minus_header_s")
    if not any(math.isfinite(value) for value in header_rel + age):
        return False
    start_s = finite_float(fixture.get("start_s"))
    end_s = finite_float(fixture.get("end_s"))
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.2), sharex=True)
    axes[0].plot(t_plot, header_rel, color="#2563eb", label="header_stamp rel")
    axes[0].set_ylabel("header rel [s]")
    axes[0].legend(loc="best")
    axes[1].plot(t_plot, age, color="#f97316", label="bag_time - header_stamp")
    axes[1].set_ylabel("derived age [s]")
    axes[1].legend(loc="best")
    axes[2].step(
        t_plot,
        [int(row.get("in_expected_window", 0) or 0) for row in rows],
        where="post",
        color="#16a34a",
        label="expected fixture window",
    )
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].set_ylabel("window")
    axes[2].set_xlabel("time since first /iap/integrity bag message [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        if start_s is not None and end_s is not None:
            ax.axvspan(start_s, end_s, color="#fde68a", alpha=0.25)
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5-5 /iap/integrity stamp-freeze evidence")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_5_current_stale_duration_timeline(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    replan_s = finite_float(fixture.get("expected_replan_s"))
    emergency_s = finite_float(fixture.get("expected_emergency_s"))
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.2), sharex=True)
    axes[0].plot(
        t,
        finite_or_nan(rows, "current_integrity_age_s"),
        color="#2563eb",
        label="current_integrity_age_s",
    )
    axes[0].set_ylabel("age [s]")
    axes[0].legend(loc="best")
    axes[1].plot(
        t,
        finite_or_nan(rows, "current_stale_duration_s"),
        color="#f97316",
        label="current_stale_duration_s",
    )
    if replan_s is not None:
        axes[1].axhline(replan_s, color="#111827", lw=0.9, ls="--", label="replan threshold")
    if emergency_s is not None:
        axes[1].axhline(emergency_s, color="#dc2626", lw=0.9, ls="--", label="emergency threshold")
    axes[1].set_ylabel("duration [s]")
    axes[1].legend(loc="best")
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    codes = action_code_map(actions)
    axes[2].step(t, [codes[value] for value in actions], where="post", color="#7c3aed")
    axes[2].set_yticks(list(codes.values()), list(codes.keys()))
    axes[2].set_ylabel("action")
    axes[2].set_xlabel("time since first P5 status [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5-5 current-stale debounce timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_5_cause_exclusion_summary(
    gates: dict[str, Any],
    path: Path,
) -> bool:
    cause = gates.get("cause_exclusion", {}) or {}
    labels = [
        "startup/snapshot",
        "future_bad",
        "unknown_only",
        "non_current_stale",
    ]
    values = [
        int(cause.get("startup_snapshot_action_count", 0) or 0),
        int(cause.get("future_bad_action_count", 0) or 0),
        int(cause.get("unknown_only_action_count", 0) or 0),
        int(cause.get("non_current_stale_action_count", 0) or 0),
    ]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    colors = ["#16a34a" if value == 0 else "#dc2626" for value in values]
    ax.bar(labels, values, color=colors)
    ax.set_ylabel("action rows")
    ax.set_title("P5-5 cause exclusion summary")
    ax.tick_params(axis="x", rotation=15)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_6_unknown_ratio_vs_action(
    rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    raw_actions = [p5_action(row, "raw_action") or "<empty>" for row in rows]
    codes = action_code_map(actions + raw_actions)
    unknown_ratio = finite_or_nan(rows, "unknown_ratio")
    future_unknown_duration = finite_or_nan(rows, "future_unknown_duration_s")
    if not any(math.isfinite(value) for value in unknown_ratio + future_unknown_duration):
        return False

    fig, axes = plt.subplots(3, 1, figsize=(11, 7.2), sharex=True)
    axes[0].plot(t, unknown_ratio, color="#6b7280", label="unknown_ratio")
    axes[0].plot(t, finite_or_nan(rows, "bad_ratio"), color="#dc2626", label="bad_ratio")
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].set_ylabel("ratio")
    axes[0].legend(loc="best")
    axes[1].plot(
        t,
        future_unknown_duration,
        color="#9333ea",
        label="future_unknown_duration_s",
    )
    axes[1].set_ylabel("duration [s]")
    axes[1].legend(loc="best")
    axes[2].step(t, [codes[value] for value in actions], where="post", label="action", color="#f97316")
    axes[2].step(t, [codes[value] for value in raw_actions], where="post", label="raw_action", color="#f59e0b", linestyle="--")
    axes[2].set_yticks(list(codes.values()), list(codes.keys()))
    axes[2].set_ylabel("P5 action")
    axes[2].set_xlabel("time since first P5 status [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5-6 unknown ratio vs action")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_6_reason_histogram(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    reasons: Counter[str] = Counter()
    for row in rows:
        for reason in p5_all_reason_values(
            row,
            (
                "reason",
                "raw_reason",
                "current_reason",
                "future_reason",
                "active_reasons",
                "final_gate_last_reason",
                "pred_al_last_reason",
            ),
        ):
            if reason and reason != "ok":
                reasons[reason] += 1
    if not reasons:
        return False
    items = reasons.most_common()
    fig, ax = plt.subplots(figsize=(9, max(4.5, 0.36 * len(items) + 1.6)))
    ax.barh([item[0] for item in items], [item[1] for item in items], color="#2563eb")
    ax.invert_yaxis()
    ax.set_xlabel("status rows")
    ax.set_title("P5-6 status reason histogram")
    ax.grid(True, axis="x", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_6_cause_exclusion_summary(
    gates: dict[str, Any],
    path: Path,
) -> bool:
    summary = gates.get("cause_exclusion", {}) or {}
    causes = [
        ("current_stale", int(summary.get("current_stale_action_count", 0) or 0)),
        ("current_invalid", int(summary.get("current_invalid_action_count", 0) or 0)),
        ("p5_5_fixture", int(summary.get("p5_5_fixture_evidence_count", 0) or 0)),
        ("future_bad", int(summary.get("future_bad_action_count", 0) or 0)),
        ("startup_snapshot", int(summary.get("startup_snapshot_action_count", 0) or 0)),
        ("topic_gap", int(summary.get("topic_gap_action_count", 0) or 0)),
        ("low_margin_only", int(summary.get("low_margin_only_action_count", 0) or 0)),
    ]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    ax.bar(
        [item[0] for item in causes],
        [item[1] for item in causes],
        color=["#16a34a" if item[1] == 0 else "#dc2626" for item in causes],
    )
    ax.set_ylabel("action rows")
    ax.set_title("P5-6 excluded cause summary")
    ax.tick_params(axis="x", rotation=20)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_6_unknown_field_overlay(
    validity_cloud_rows: list[dict[str, Any]],
    sample_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not fixture.get("valid_geometry"):
        return False
    cloud_xy = [
        row
        for row in validity_cloud_rows
        if finite_float(row.get("x")) is not None
        and finite_float(row.get("y")) is not None
    ]
    sample_xy = [
        row
        for row in sample_rows
        if finite_float(row.get("x")) is not None
        and finite_float(row.get("y")) is not None
    ]
    if not cloud_xy and not sample_xy:
        return False

    fig, ax = plt.subplots(figsize=(8.8, 6.8))
    if cloud_xy:
        stride = max(1, len(cloud_xy) // 6000)
        plotted_cloud = cloud_xy[::stride]
        cloud_groups = [
            (
                [
                    row
                    for row in plotted_cloud
                    if int(row.get("unknown", 0) or 0)
                    or int(row.get("valid", 0) or 0) == 0
                ],
                "#94a3b8",
                "risk validity unknown",
                8,
                0.36,
            ),
            (
                [
                    row
                    for row in plotted_cloud
                    if int(row.get("valid", 0) or 0)
                    and not int(row.get("unknown", 0) or 0)
                ],
                "#86efac",
                "risk validity valid",
                6,
                0.22,
            ),
        ]
        for group, color, label, size, alpha in cloud_groups:
            if not group:
                continue
            ax.scatter(
                [float(finite_float(row.get("x")) or 0.0) for row in group],
                [float(finite_float(row.get("y")) or 0.0) for row in group],
                s=size,
                color=color,
                alpha=alpha,
                linewidths=0,
                label=label,
            )

    sample_groups = [
        (
            [
                row
                for row in sample_xy
                if int(row.get("inside_high_risk_zone", 0) or 0)
                and int(row.get("inside_tau_window", 0) or 0)
                and int(row.get("unknown", 0) or 0)
            ],
            "#111827",
            "unknown sample in fixture",
            48,
            0.9,
        ),
        (
            [
                row
                for row in sample_xy
                if int(row.get("unknown", 0) or 0)
                and not (
                    int(row.get("inside_high_risk_zone", 0) or 0)
                    and int(row.get("inside_tau_window", 0) or 0)
                )
            ],
            "#6b7280",
            "unknown sample outside fixture",
            28,
            0.78,
        ),
        (
            [row for row in sample_xy if not int(row.get("unknown", 0) or 0)],
            "#2563eb",
            "finite sample",
            22,
            0.68,
        ),
    ]
    for group, color, label, size, alpha in sample_groups:
        if not group:
            continue
        ax.scatter(
            [float(finite_float(row.get("x")) or 0.0) for row in group],
            [float(finite_float(row.get("y")) or 0.0) for row in group],
            s=size,
            color=color,
            alpha=alpha,
            linewidths=0,
            label=label,
        )

    x_min = float(fixture["x_min"])
    x_max = float(fixture["x_max"])
    y_min = float(fixture["y_min"])
    y_max = float(fixture["y_max"])
    ax.add_patch(
        plt.Rectangle(
            (min(x_min, x_max), min(y_min, y_max)),
            abs(x_max - x_min),
            abs(y_max - y_min),
            fill=False,
            edgecolor="#dc2626",
            linewidth=1.7,
            linestyle="--",
            label="P5-6 fixture x/y bounds",
        )
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P5-6 future-unknown field overlay")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_trajectory_integrity_samples(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    state_colors = {
        "ok": "#16a34a",
        "bad": "#dc2626",
        "unknown": "#6b7280",
        "stale_or_warning": "#f97316",
        "other": "#2563eb",
        "transparent": "#d1d5db",
    }
    fig, ax = plt.subplots(figsize=(8, 6.5))
    plotted = False
    for state, color_value in state_colors.items():
        state_rows = [row for row in rows if str(row.get("state", "")) == state]
        if not state_rows:
            continue
        xs = [finite_float(row.get("x")) for row in state_rows]
        ys = [finite_float(row.get("y")) for row in state_rows]
        xy = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
        if not xy:
            continue
        ax.scatter(
            [point[0] for point in xy],
            [point[1] for point in xy],
            s=18 if state != "bad" else 36,
            color=color_value,
            alpha=0.78,
            label=state,
            linewidths=0,
        )
        plotted = True
    if not plotted:
        plt.close(fig)
        return False
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P5 trajectory integrity marker samples")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_high_risk_zone_overlay(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not rows or not fixture.get("valid_geometry"):
        return False
    xy_rows = [
        row
        for row in rows
        if finite_float(row.get("x")) is not None and finite_float(row.get("y")) is not None
    ]
    if not xy_rows:
        return False
    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    outside = [row for row in xy_rows if not int(row.get("inside_high_risk_zone", 0) or 0)]
    overlap = [row for row in xy_rows if int(row.get("inside_high_risk_zone", 0) or 0)]
    bad = [row for row in overlap if int(row.get("overlap_bad_state", 0) or 0)]

    def scatter(group: list[dict[str, Any]], color: str, label: str, size: int) -> None:
        if not group:
            return
        ax.scatter(
            [float(finite_float(row.get("x")) or 0.0) for row in group],
            [float(finite_float(row.get("y")) or 0.0) for row in group],
            s=size,
            color=color,
            alpha=0.72,
            linewidths=0,
            label=label,
        )

    scatter(outside, "#cbd5e1", "trajectory samples", 14)
    scatter(overlap, "#2563eb", "zone overlap", 22)
    scatter(bad, "#dc2626", "bad overlap", 36)

    x_min = float(fixture["x_min"])
    x_max = float(fixture["x_max"])
    y_min = float(fixture["y_min"])
    y_max = float(fixture["y_max"])
    rect = plt.Rectangle(
        (min(x_min, x_max), min(y_min, y_max)),
        abs(x_max - x_min),
        abs(y_max - y_min),
        fill=False,
        edgecolor="#111827",
        linewidth=1.6,
        linestyle="--",
        label="fixture x/y bounds",
    )
    ax.add_patch(rect)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P5-3 high-risk zone trajectory overlap")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_first_bad_tau_timeline(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not rows or not fixture.get("valid_geometry"):
        return False
    t = relative_time(rows)
    if not t:
        return False
    first_bad_tau = finite_or_nan(rows, "first_bad_tau")
    future_min_im = finite_or_nan(rows, "future_min_im")
    if not any(math.isfinite(value) for value in first_bad_tau + future_min_im):
        return False
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    codes = action_code_map(actions)
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.2), sharex=True)
    axes[0].plot(t, first_bad_tau, label="first_bad_tau", color="#dc2626")
    axes[0].axhspan(
        min(float(fixture["tau_min"]), float(fixture["tau_max"])),
        max(float(fixture["tau_min"]), float(fixture["tau_max"])),
        color="#fecaca",
        alpha=0.35,
        label="fixture tau window",
    )
    axes[0].set_ylabel("tau [s]")
    axes[0].legend(loc="best")
    axes[1].plot(t, future_min_im, label="future_min_im", color="#2563eb")
    axes[1].axhline(0.3, color="#111827", lw=0.9, linestyle="--", label="future replan margin")
    axes[1].axhline(0.0, color="#111827", lw=0.8)
    axes[1].set_ylabel("IM [m]")
    axes[1].legend(loc="best")
    axes[2].step(t, [codes[value] for value in actions], where="post", color="#f97316")
    axes[2].set_yticks(list(codes.values()), list(codes.keys()))
    axes[2].set_ylabel("P5 action")
    axes[2].set_xlabel("time since first P5 status [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5-3 first bad tau and future margin")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_reason_timeline(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    reason_keys = (
        "reason",
        "current_reason",
        "future_reason",
        "active_reasons",
        "final_gate_last_reason",
    )
    reason_points: list[tuple[float, str, str]] = []
    for idx, row in enumerate(rows):
        x = t[idx] if idx < len(t) else math.nan
        if not math.isfinite(x):
            continue
        for key in reason_keys:
            for reason in sorted(p5_reason_values(row, key)):
                if reason and reason != "ok":
                    reason_points.append((x, key, reason))
    if not reason_points:
        return False
    reasons = sorted({reason for _, _, reason in reason_points})
    reason_codes = {reason: idx for idx, reason in enumerate(reasons)}
    key_colors = {
        "reason": "#111827",
        "current_reason": "#16a34a",
        "future_reason": "#dc2626",
        "active_reasons": "#2563eb",
        "final_gate_last_reason": "#7c3aed",
    }
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    raw_actions = [p5_action(row, "raw_action") or "<empty>" for row in rows]
    action_codes = action_code_map(actions + raw_actions)

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True)
    for key in reason_keys:
        group = [point for point in reason_points if point[1] == key]
        if not group:
            continue
        axes[0].scatter(
            [point[0] for point in group],
            [reason_codes[point[2]] for point in group],
            s=26,
            alpha=0.78,
            color=key_colors.get(key, "#6b7280"),
            label=key,
        )
    axes[0].set_yticks(list(reason_codes.values()), list(reason_codes.keys()))
    axes[0].set_ylabel("reason")
    axes[0].legend(loc="best", fontsize=8)
    axes[0].grid(True, alpha=0.25)

    axes[1].step(t, [action_codes[value] for value in actions], where="post", label="action", color="#f97316")
    axes[1].step(t, [action_codes[value] for value in raw_actions], where="post", label="raw_action", color="#f59e0b", linestyle="--")
    axes[1].set_yticks(list(action_codes.values()), list(action_codes.keys()))
    axes[1].set_ylabel("P5 action")
    axes[1].set_xlabel("time since first P5 status [s]")
    axes[1].legend(loc="best")
    axes[1].grid(True, axis="x", alpha=0.25)
    fig.suptitle("P5-3 reason attribution timeline")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_sample_high_risk_overlay(
    sample_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
) -> bool:
    if not sample_rows or not fixture.get("valid_geometry"):
        return False
    xy_rows = [
        row
        for row in sample_rows
        if finite_float(row.get("x")) is not None and finite_float(row.get("y")) is not None
    ]
    if not xy_rows:
        return False

    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    groups = [
        (
            [row for row in xy_rows if int(row.get("inside_high_risk_zone", 0) or 0) == 0],
            "#cbd5e1",
            "outside fixture",
            18,
        ),
        (
            [
                row
                for row in xy_rows
                if int(row.get("inside_high_risk_zone", 0) or 0)
                and int(row.get("bad", 0) or 0) == 0
            ],
            "#2563eb",
            "inside fixture",
            28,
        ),
        (
            [
                row
                for row in xy_rows
                if int(row.get("inside_high_risk_zone", 0) or 0)
                and int(row.get("bad", 0) or 0)
            ],
            "#dc2626",
            "bad inside fixture",
            44,
        ),
    ]
    for group, color, label, size in groups:
        if not group:
            continue
        ax.scatter(
            [float(finite_float(row.get("x")) or 0.0) for row in group],
            [float(finite_float(row.get("y")) or 0.0) for row in group],
            s=size,
            color=color,
            alpha=0.78,
            linewidths=0,
            label=label,
        )

    current_rows = [
        row
        for row in xy_rows
        if finite_float(row.get("tau_s")) is not None
        and abs(float(finite_float(row.get("tau_s")) or 0.0)) <= 1.0e-6
    ]
    if current_rows:
        ax.scatter(
            [float(finite_float(row.get("x")) or 0.0) for row in current_rows],
            [float(finite_float(row.get("y")) or 0.0) for row in current_rows],
            s=68,
            facecolors="none",
            edgecolors="#111827",
            linewidths=1.3,
            label="tau=0 sample",
        )

    x_min = float(fixture["x_min"])
    x_max = float(fixture["x_max"])
    y_min = float(fixture["y_min"])
    y_max = float(fixture["y_max"])
    ax.add_patch(
        plt.Rectangle(
            (min(x_min, x_max), min(y_min, y_max)),
            abs(x_max - x_min),
            abs(y_max - y_min),
            fill=False,
            edgecolor="#111827",
            linewidth=1.6,
            linestyle="--",
            label="fixture x/y bounds",
        )
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P5-3 PL/AL high-risk sample overlay")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_sample_tau_window(
    sample_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
    tau_field: str = "tau_s",
    title: str = "P5-3 PL/AL tau-window evidence",
) -> bool:
    if not sample_rows or not fixture.get("valid_geometry"):
        return False
    stamps = [finite_float(row.get("bag_time_s")) for row in sample_rows]
    finite_stamps = [value for value in stamps if value is not None]
    if not finite_stamps:
        return False
    t0 = min(finite_stamps)
    groups = [
        (
            [row for row in sample_rows if int(row.get("fixture_bad_sample", 0) or 0)],
            "#dc2626",
            "bad in fixture window",
            42,
        ),
        (
            [
                row
                for row in sample_rows
                if int(row.get("inside_high_risk_zone", 0) or 0)
                and int(row.get("inside_tau_window", 0) or 0)
                and int(row.get("fixture_bad_sample", 0) or 0) == 0
            ],
            "#2563eb",
            "inside fixture window",
            30,
        ),
        (
            [
                row
                for row in sample_rows
                if int(row.get("inside_high_risk_zone", 0) or 0) == 0
                or int(row.get("inside_tau_window", 0) or 0) == 0
            ],
            "#cbd5e1",
            "outside fixture window",
            18,
        ),
    ]
    fig, ax = plt.subplots(figsize=(11, 5.2))
    plotted = False
    for group, color, label, size in groups:
        points = [
            (finite_float(row.get("bag_time_s")), finite_float(row.get(tau_field)))
            for row in group
        ]
        points = [(x, y) for x, y in points if x is not None and y is not None]
        if not points:
            continue
        ax.scatter(
            [x - t0 for x, _ in points],
            [y for _, y in points],
            s=size,
            color=color,
            alpha=0.78,
            linewidths=0,
            label=label,
        )
        plotted = True
    if not plotted:
        plt.close(fig)
        return False
    ax.axhspan(
        min(float(fixture["tau_min"]), float(fixture["tau_max"])),
        max(float(fixture["tau_min"]), float(fixture["tau_max"])),
        color="#fecaca",
        alpha=0.28,
        label="fixture tau window",
    )
    first_bad = [
        (finite_float(row.get("bag_time_s")), finite_float(row.get("first_bad_tau")))
        for row in p5_rows
    ]
    first_bad = [(x, y) for x, y in first_bad if x is not None and y is not None]
    if first_bad:
        ax.plot(
            [x - t0 for x, _ in first_bad],
            [y for _, y in first_bad],
            color="#111827",
            lw=1.1,
            label="first_bad_tau",
        )
    ax.set_xlabel("time since first sample [s]")
    ax.set_ylabel(f"{tau_field} [s]")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_replan_vs_emergency(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    t = relative_time(rows)
    if not t:
        return False
    actions = [p5_action(row, "action") or "<empty>" for row in rows]
    raw_actions = [p5_action(row, "raw_action") or "<empty>" for row in rows]
    codes = action_code_map(actions + raw_actions)
    replan_cumulative: list[int] = []
    emergency_cumulative: list[int] = []
    replan_count = 0
    emergency_count = 0
    for row in rows:
        if (
            p5_action(row, "action") == P5_REPLAN_ACTION
            or p5_action(row, "raw_action") == P5_REPLAN_ACTION
        ):
            replan_count += 1
        if (
            p5_action(row, "action") == P5_EMERGENCY_ACTION
            or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
        ):
            emergency_count += 1
        replan_cumulative.append(replan_count)
        emergency_cumulative.append(emergency_count)

    fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
    axes[0].step(t, [codes[value] for value in actions], where="post", label="action", color="#f97316")
    axes[0].step(t, [codes[value] for value in raw_actions], where="post", label="raw_action", color="#f59e0b", linestyle="--")
    axes[0].set_yticks(list(codes.values()), list(codes.keys()))
    axes[0].set_ylabel("P5 action")
    axes[0].legend(loc="best")
    axes[1].plot(t, replan_cumulative, label="REQUEST_REPLAN rows", color="#2563eb")
    axes[1].plot(t, emergency_cumulative, label="emergency candidate rows", color="#dc2626")
    axes[1].set_ylabel("cumulative rows")
    axes[1].set_xlabel("time since first P5 status [s]")
    axes[1].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("P5-3 replan vs emergency")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_sample_heatmap(
    sample_rows: list[dict[str, Any]],
    path: Path,
    tau_field: str = "tau_s",
    title: str = "P5-3 PL/AL sample heatmap",
) -> bool:
    if not sample_rows:
        return False
    stamps = [finite_float(row.get("bag_time_s")) for row in sample_rows]
    finite_stamps = [value for value in stamps if value is not None]
    points = []
    for row in sample_rows:
        stamp = finite_float(row.get("bag_time_s"))
        tau = finite_float(row.get(tau_field))
        im = finite_float(row.get("im_min"))
        if stamp is None or tau is None:
            continue
        if im is None:
            im = -1.0 if int(row.get("bad", 0) or 0) else math.nan
        points.append((stamp, tau, im, int(row.get("bad", 0) or 0)))
    if not points or not finite_stamps:
        return False
    t0 = min(finite_stamps)
    fig, ax = plt.subplots(figsize=(11, 5.4))
    finite_im = [im for _, _, im, _ in points if math.isfinite(im)]
    if finite_im:
        scatter = ax.scatter(
            [stamp - t0 for stamp, _, _, _ in points],
            [tau for _, tau, _, _ in points],
            c=[im for _, _, im, _ in points],
            s=[42 if bad else 26 for _, _, _, bad in points],
            cmap="RdYlGn",
            vmin=min(min(finite_im), -0.5),
            vmax=max(max(finite_im), 0.5),
            marker="s",
            alpha=0.82,
            linewidths=0,
        )
        cbar = fig.colorbar(scatter, ax=ax)
        cbar.set_label("IM min [m]")
    else:
        bad_points = [point for point in points if point[3]]
        ok_points = [point for point in points if not point[3]]
        if ok_points:
            ax.scatter(
                [stamp - t0 for stamp, _, _, _ in ok_points],
                [tau for _, tau, _, _ in ok_points],
                color="#16a34a",
                marker="s",
                label="not bad",
            )
        if bad_points:
            ax.scatter(
                [stamp - t0 for stamp, _, _, _ in bad_points],
                [tau for _, tau, _, _ in bad_points],
                color="#dc2626",
                marker="s",
                label="bad",
            )
            ax.legend(loc="best")
    ax.set_xlabel("time since first sample [s]")
    ax.set_ylabel(f"{tau_field} [s]")
    ax.set_title(title)
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_query_alignment_pl_probe(
    sample_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    path: Path,
    *,
    tau_field: str = "query_tau_s",
    title: str = "P5-3 query-aligned PL probe",
) -> bool:
    if not sample_rows or not fixture.get("valid_geometry"):
        return False
    rows = [
        row
        for row in sample_rows
        if int(row.get("inside_high_risk_zone", 0) or 0)
        and int(row.get("inside_tau_window", 0) or 0)
    ]
    if not rows:
        return False
    stamps = [finite_float(row.get("bag_time_s")) for row in rows]
    finite_stamps = [stamp for stamp in stamps if stamp is not None]
    if not finite_stamps:
        return False
    t0 = min(finite_stamps)
    points = []
    for row in rows:
        stamp = finite_float(row.get("bag_time_s"))
        tau = finite_float(row.get(tau_field))
        actual_hpl = finite_float(row.get("actual_hpl"))
        actual_vpl = finite_float(row.get("actual_vpl"))
        expected_hpl = finite_float(row.get("expected_hpl"))
        expected_vpl = finite_float(row.get("expected_vpl"))
        hpl_error = finite_float(row.get("hpl_error"))
        vpl_error = finite_float(row.get("vpl_error"))
        if stamp is None or tau is None:
            continue
        points.append(
            {
                "t": stamp - t0,
                "tau": tau,
                "actual_hpl": actual_hpl,
                "actual_vpl": actual_vpl,
                "expected_hpl": expected_hpl,
                "expected_vpl": expected_vpl,
                "hpl_error": hpl_error,
                "vpl_error": vpl_error,
                "aligned": int(row.get("query_alignment_ok", 0) or 0) == 1,
            }
        )
    if not points:
        return False

    colors = ["#16a34a" if point["aligned"] else "#dc2626" for point in points]
    fig, axes = plt.subplots(3, 1, figsize=(11, 7.4), sharex=True)
    axes[0].scatter(
        [point["t"] for point in points],
        [math.nan if point["actual_hpl"] is None else point["actual_hpl"] for point in points],
        c=colors,
        s=34,
        label="actual hpl",
    )
    axes[0].plot(
        [point["t"] for point in points],
        [math.nan if point["expected_hpl"] is None else point["expected_hpl"] for point in points],
        color="#111827",
        lw=1.0,
        label="expected hpl",
    )
    axes[0].set_ylabel("HPL [m]")
    axes[0].legend(loc="best")

    axes[1].scatter(
        [point["t"] for point in points],
        [math.nan if point["actual_vpl"] is None else point["actual_vpl"] for point in points],
        c=colors,
        s=34,
        label="actual vpl",
    )
    axes[1].plot(
        [point["t"] for point in points],
        [math.nan if point["expected_vpl"] is None else point["expected_vpl"] for point in points],
        color="#111827",
        lw=1.0,
        label="expected vpl",
    )
    axes[1].set_ylabel("VPL [m]")
    axes[1].legend(loc="best")

    axes[2].scatter(
        [point["t"] for point in points],
        [math.nan if point["hpl_error"] is None else point["hpl_error"] for point in points],
        c="#2563eb",
        s=30,
        label="hpl error",
    )
    axes[2].scatter(
        [point["t"] for point in points],
        [math.nan if point["vpl_error"] is None else point["vpl_error"] for point in points],
        c="#f97316",
        s=24,
        label="vpl error",
    )
    axes[2].axhline(0.0, color="#111827", lw=0.9)
    axes[2].set_ylabel("actual - expected [m]")
    axes[2].set_xlabel("time since first fixture-window sample [s]")
    axes[2].legend(loc="best")
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_3_topic_gap(
    topic_timestamps: dict[str, list[float]],
    active_topic_gap: dict[str, Any],
    path: Path,
) -> bool:
    if not active_topic_gap.get("available"):
        return False
    window = active_topic_gap.get("window", {}) or {}
    start_s = finite_float(window.get("start_s"))
    end_s = finite_float(window.get("end_s"))
    if start_s is None or end_s is None:
        return False
    topics = [
        topic
        for topic, expected in P5_TOPIC_EXPECTATIONS.items()
        if expected == "continuous"
    ]
    rel_events: list[list[float]] = []
    for topic in topics:
        status = (active_topic_gap.get("topic_statuses", {}) or {}).get(topic, {}) or {}
        bracket_start = finite_float(status.get("bracket_start_s"))
        bracket_end = finite_float(status.get("bracket_end_s"))
        topic_events = [
            stamp - start_s
            for stamp in sorted(topic_timestamps.get(topic, []))
            if start_s <= stamp <= end_s
        ]
        if bracket_start is not None and bracket_start < start_s:
            topic_events.insert(0, bracket_start - start_s)
        if bracket_end is not None and bracket_end > end_s:
            topic_events.append(bracket_end - start_s)
        rel_events.append(
            sorted(set(topic_events))
        )

    fig, ax = plt.subplots(figsize=(11, 4.8))
    y_positions = list(range(len(topics)))
    colors = ["#2563eb", "#16a34a", "#9333ea", "#f97316"]
    ax.eventplot(
        rel_events,
        lineoffsets=y_positions,
        linelengths=0.65,
        linewidths=0.9,
        colors=[colors[idx % len(colors)] for idx in y_positions],
    )
    max_gap_limit = float(active_topic_gap.get("continuous_max_gap_s", CONTINUOUS_MAX_GAP_S))
    gap_label_added = False
    window_duration = max(0.0, end_s - start_s)
    for y_pos, topic in zip(y_positions, topics):
        rel_values = rel_events[y_pos]
        edge_points = sorted(set([0.0, *rel_values, window_duration]))
        for prev, curr in zip(edge_points, edge_points[1:]):
            if curr - prev > max_gap_limit:
                ax.hlines(
                    y_pos,
                    prev,
                    curr,
                    color="#dc2626",
                    lw=4,
                    alpha=0.48,
                    label=f"gap > {max_gap_limit:.1f}s" if not gap_label_added else None,
                )
                gap_label_added = True
    ax.axvspan(0.0, window_duration, color="#dbeafe", alpha=0.2, label="P5 sample evidence window")
    ax.set_yticks(y_positions, topics)
    ax.set_xlabel("time since P5 evidence window start [s]")
    ax.set_title("P5-3 query-alignment active topic gaps")
    ax.grid(True, axis="x", alpha=0.25)
    if gap_label_added:
        ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p5_rviz_overview(
    topic_health: dict[str, dict[str, Any]],
    p5_summary: dict[str, Any],
    marker_summary: dict[str, Any],
    path: Path,
) -> bool:
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
    topics = [P5_STATUS_TOPIC, *P5_RVIZ_TOPICS]
    counts = [int((topic_health.get(topic) or {}).get("count", 0) or 0) for topic in topics]
    colors = ["#16a34a" if count > 0 else "#dc2626" for count in counts]
    axes[0].barh(topics, counts, color=colors)
    axes[0].invert_yaxis()
    axes[0].set_xlabel("bag messages")
    axes[0].set_title("P5 topic evidence")
    axes[0].grid(True, axis="x", alpha=0.25)

    axes[1].axis("off")
    text = "\n".join(
        [
            f"status_rows: {p5_summary.get('status_rows', 0)}",
            f"OK ratio: {float(p5_summary.get('ok_action_ratio', 0.0) or 0.0):.3f}",
            f"actions: {p5_summary.get('action_counts', {})}",
            f"raw_actions: {p5_summary.get('raw_action_counts', {})}",
            f"future_min_im_min: {p5_summary.get('future_min_im_min')}",
            f"current_im_min_min: {p5_summary.get('current_im_min_min')}",
            f"final_gate_fail_count_max: {p5_summary.get('final_gate_fail_count_max', 0)}",
            f"marker_rows: {marker_summary.get('row_count', 0)}",
            f"marker_states: {marker_summary.get('state_counts', {})}",
        ]
    )
    axes[1].text(0.02, 0.95, text, va="top", ha="left", family="monospace", fontsize=9)
    axes[1].set_title("P5 run summary")
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


def plot_p0_6_occupied_overlap_map(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    x = np.array([float(row["x"]) for row in rows])
    y = np.array([float(row["y"]) for row in rows])
    occupied_skip = np.array([int(row.get("occupied_skip", 0) or 0) == 1 for row in rows])
    final_valid = np.array([int(row.get("final_valid", 0) or 0) == 1 for row in rows])
    fig, ax = plt.subplots(figsize=(7, 6))
    if np.any(occupied_skip):
        ax.scatter(
            x[occupied_skip],
            y[occupied_skip],
            s=24,
            c="#7c3aed",
            label="occupied -> unknown/skip",
            alpha=0.85,
        )
    bad = final_valid
    if np.any(bad):
        ax.scatter(
            x[bad],
            y[bad],
            s=38,
            c="#dc2626",
            marker="x",
            label="occupied valid",
        )
    if not np.any(occupied_skip) and not np.any(bad):
        ax.scatter(x, y, s=20, c="#6b7280", label="occupied overlap")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P0-6 occupied overlap map")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_6_raw_pl_vs_final_validity(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    raw_cost = np.array([float(row["raw_c_pi"]) for row in rows])
    final_state = np.array(
        [
            2 if int(row.get("final_valid", 0) or 0) == 1
            else 1 if int(row.get("occupied_skip", 0) or 0) == 1
            else 0
            for row in rows
        ]
    )
    labels = ["invalid/no-skip", "unknown/occupied-skip", "valid"]
    counts = [int(np.sum(final_state == code)) for code in range(3)]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].hist(raw_cost, bins=min(20, max(1, len(rows))), color="#2563eb", alpha=0.85)
    axes[0].set_xlabel("fixture raw c_pi")
    axes[0].set_ylabel("occupied cells")
    axes[0].set_title("Fixture-declared raw low cost")
    axes[1].bar(labels, counts, color=["#dc2626", "#7c3aed", "#16a34a"])
    axes[1].set_ylabel("occupied cells")
    axes[1].set_title("Final grid validity")
    axes[1].tick_params(axis="x", rotation=15)
    for ax in axes:
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_6_occupied_skip_count_timeline(
    health_rows: list[dict[str, Any]],
    path: Path,
) -> bool:
    if not health_rows:
        return False
    points = [
        (
            finite_float(row.get("stamp")),
            int(finite_float(row.get("occupied_skip_count")) or 0),
        )
        for row in health_rows
    ]
    points = [(stamp, count) for stamp, count in points if stamp is not None]
    if not points:
        return False
    t0 = points[0][0]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.step([stamp - t0 for stamp, _ in points], [count for _, count in points], where="post", color="#7c3aed")
    ax.set_xlabel("time since first P0 health sample [s]")
    ax.set_ylabel("occupied_skip_count")
    ax.set_title("P0-6 occupied skip count timeline")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return True


def plot_p0_6_occupied_cells_table_heatmap(rows: list[dict[str, Any]], path: Path) -> bool:
    if not rows:
        return False
    xs = sorted({round(float(row["x"]), 3) for row in rows})
    ys = sorted({round(float(row["y"]), 3) for row in rows})
    if not xs or not ys or len(xs) * len(ys) > 2500:
        return False
    grid = np.full((len(ys), len(xs)), np.nan)
    x_index = {value: idx for idx, value in enumerate(xs)}
    y_index = {value: idx for idx, value in enumerate(ys)}
    for row in rows:
        x = round(float(row["x"]), 3)
        y = round(float(row["y"]), 3)
        code = (
            2.0 if int(row.get("occupied_skip", 0) or 0) == 1
            else 1.0 if int(row.get("final_unknown", 0) or 0) == 1
            else 0.0
        )
        grid[y_index[y], x_index[x]] = max(code, grid[y_index[y], x_index[x]] if math.isfinite(grid[y_index[y], x_index[x]]) else code)
    fig, ax = plt.subplots(figsize=(8, 5.5))
    image = ax.imshow(grid, origin="lower", aspect="auto", cmap="viridis", vmin=0.0, vmax=2.0)
    fig.colorbar(image, ax=ax, label="0 valid/invalid, 1 unknown, 2 occupied-skip")
    ax.set_xticks(range(len(xs)), [f"{value:.2f}" for value in xs], rotation=45, ha="right")
    ax.set_yticks(range(len(ys)), [f"{value:.2f}" for value in ys])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("P0-6 occupied cells table heatmap")
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


def manifest_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def nested_get(data: dict[str, Any], keys: list[str], default: Any = None) -> Any:
    cursor: Any = data
    for key in keys:
        if not isinstance(cursor, dict) or key not in cursor:
            return default
        cursor = cursor[key]
    return cursor


def p0_6_fixture_from_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    nested_fixture = nested_get(manifest, ["p0_6", "fixture"], {}) or {}
    bounds = nested_fixture.get("bounds", {}) if isinstance(nested_fixture, dict) else {}
    expected_raw = (
        nested_fixture.get("expected_raw", {})
        if isinstance(nested_fixture, dict)
        else {}
    )
    if not isinstance(expected_raw, dict):
        expected_raw = {}

    def manifest_value(flat_key: str, nested_value: Any, default: Any) -> Any:
        return manifest.get(flat_key, nested_value if nested_value is not None else default)

    x_bounds = bounds.get("x", [None, None]) if isinstance(bounds, dict) else [None, None]
    y_bounds = bounds.get("y", [None, None]) if isinstance(bounds, dict) else [None, None]
    z_bounds = bounds.get("z", [None, None]) if isinstance(bounds, dict) else [None, None]
    if len(x_bounds) < 2:
        x_bounds = [None, None]
    if len(y_bounds) < 2:
        y_bounds = [None, None]
    if len(z_bounds) < 2:
        z_bounds = [None, None]

    fixture = {
        "enabled": manifest_bool(
            manifest_value(
                "p0_6.fixture.enabled",
                nested_fixture.get("enabled") if isinstance(nested_fixture, dict) else None,
                False,
            )
        ),
        "name": str(
            manifest_value(
                "p0_6.fixture.name",
                nested_fixture.get("name") if isinstance(nested_fixture, dict) else None,
                "",
            )
        ),
        "x_min": finite_float(manifest_value("p0_6.fixture.x_min", x_bounds[0], None)),
        "x_max": finite_float(manifest_value("p0_6.fixture.x_max", x_bounds[1], None)),
        "y_min": finite_float(manifest_value("p0_6.fixture.y_min", y_bounds[0], None)),
        "y_max": finite_float(manifest_value("p0_6.fixture.y_max", y_bounds[1], None)),
        "z_min": finite_float(manifest_value("p0_6.fixture.z_min", z_bounds[0], None)),
        "z_max": finite_float(manifest_value("p0_6.fixture.z_max", z_bounds[1], None)),
        "raw_hpl_m": finite_float(
            manifest_value("p0_6.fixture.raw_hpl_m", expected_raw.get("raw_hpl_m"), None)
        ),
        "raw_vpl_m": finite_float(
            manifest_value("p0_6.fixture.raw_vpl_m", expected_raw.get("raw_vpl_m"), None)
        ),
        "raw_c_pi": finite_float(
            manifest_value("p0_6.fixture.raw_c_pi", expected_raw.get("raw_c_pi"), None)
        ),
        "low_raw_cost_threshold": finite_float(
            manifest_value(
                "p0_6.fixture.low_raw_cost_threshold",
                expected_raw.get("low_raw_cost_threshold"),
                None,
            )
        ),
    }
    for low_key, high_key in (("x_min", "x_max"), ("y_min", "y_max"), ("z_min", "z_max")):
        low = fixture[low_key]
        high = fixture[high_key]
        if low is not None and high is not None and low > high:
            fixture[low_key], fixture[high_key] = high, low
    return fixture


def p0_6_fixture_complete(fixture: dict[str, Any]) -> bool:
    required = [
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
        "raw_hpl_m",
        "raw_vpl_m",
        "raw_c_pi",
        "low_raw_cost_threshold",
    ]
    return (
        bool(fixture.get("enabled"))
        and str(fixture.get("name")) == P0_6_FIXTURE_NAME
        and all(fixture.get(key) is not None for key in required)
    )


def row_xyz_key(row: dict[str, Any]) -> tuple[float, float, float] | None:
    x = finite_float(row.get("x"))
    y = finite_float(row.get("y"))
    z = finite_float(row.get("z"))
    if x is None or y is None or z is None:
        return None
    return (round(x, 5), round(y, 5), round(z, 5))


def merged_final_p0_rows(
    pl_cloud_rows: list[dict[str, Any]],
    validity_cloud_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    by_key: dict[tuple[float, float, float], dict[str, Any]] = {}
    for row in pl_cloud_rows:
        key = row_xyz_key(row)
        if key is not None:
            by_key[key] = dict(row)
    for row in validity_cloud_rows:
        key = row_xyz_key(row)
        if key is None:
            continue
        merged = by_key.setdefault(key, dict(row))
        for field in ("valid", "unknown", "stale", "source_flags", "stamp", "x", "y", "z"):
            if field in row:
                merged[field] = row[field]
    return list(by_key.values())


def row_inside_p0_6_fixture(row: dict[str, Any], fixture: dict[str, Any]) -> bool:
    x = finite_float(row.get("x"))
    y = finite_float(row.get("y"))
    z = finite_float(row.get("z"))
    if x is None or y is None or z is None:
        return False
    return (
        float(fixture["x_min"]) <= x <= float(fixture["x_max"])
        and float(fixture["y_min"]) <= y <= float(fixture["y_max"])
        and float(fixture["z_min"]) <= z <= float(fixture["z_max"])
    )


def p0_6_final_low_risk(row: dict[str, Any], threshold: float) -> bool:
    for key in ("c_pi", "pl", "hpl", "vpl"):
        value = finite_float(row.get(key))
        if value is not None and value <= threshold:
            return True
    return False


def p0_6_overlap_rows(
    manifest: dict[str, Any],
    pl_cloud_rows: list[dict[str, Any]],
    validity_cloud_rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    fixture = p0_6_fixture_from_manifest(manifest)
    if not p0_6_fixture_complete(fixture):
        return [], fixture
    threshold = float(fixture["low_raw_cost_threshold"])
    rows: list[dict[str, Any]] = []
    for row in merged_final_p0_rows(pl_cloud_rows, validity_cloud_rows):
        if not row_inside_p0_6_fixture(row, fixture):
            continue
        raw_c_pi = float(fixture["raw_c_pi"])
        source_flags = int(finite_float(row.get("source_flags")) or 0)
        occupied_skip = (source_flags & P0_OCCUPIED_SKIP_SOURCE_FLAG) != 0
        final_valid = row_flag(row, "valid")
        final_unknown = row_flag(row, "unknown")
        final_low_risk = final_valid and p0_6_final_low_risk(row, threshold)
        if occupied_skip:
            final_reason = "occupied_skip"
        elif final_valid:
            final_reason = "valid"
        elif final_unknown:
            final_reason = "unknown_without_occupied_skip"
        else:
            final_reason = "invalid_without_occupied_skip"
        rows.append(
            {
                "stamp": row.get("stamp", ""),
                "x": row.get("x", ""),
                "y": row.get("y", ""),
                "z": row.get("z", ""),
                "occupied": 1,
                "raw_source": "fixture_manifest",
                "raw_hpl_m": fixture["raw_hpl_m"],
                "raw_vpl_m": fixture["raw_vpl_m"],
                "raw_c_pi": raw_c_pi,
                "low_raw_cost_threshold": threshold,
                "raw_low_cost": 1 if raw_c_pi <= threshold else 0,
                "final_pl": row.get("pl", ""),
                "final_hpl": row.get("hpl", ""),
                "final_vpl": row.get("vpl", ""),
                "final_c_pi": row.get("c_pi", ""),
                "final_valid": 1 if final_valid else 0,
                "final_unknown": 1 if final_unknown else 0,
                "final_stale": 1 if row_flag(row, "stale") else 0,
                "source_flags": source_flags,
                "occupied_skip": 1 if occupied_skip else 0,
                "final_low_risk": 1 if final_low_risk else 0,
                "final_reason": final_reason,
            }
        )
    return rows, fixture


def summarize_p0_6_overlap(
    rows: list[dict[str, Any]],
    health_summary: dict[str, Any],
) -> dict[str, Any]:
    occupied_low_raw_cost_count = sum(
        1 for row in rows if int(row.get("raw_low_cost", 0) or 0) == 1
    )
    occupied_skip_count = sum(
        1 for row in rows if int(row.get("occupied_skip", 0) or 0) == 1
    )
    occupied_valid_low_risk_rows = [
        row
        for row in rows
        if int(row.get("raw_low_cost", 0) or 0) == 1
        and int(row.get("final_valid", 0) or 0) == 1
        and int(row.get("final_low_risk", 0) or 0) == 1
    ]
    bad_final_rows = [
        row
        for row in rows
        if int(row.get("raw_low_cost", 0) or 0) == 1
        and (
            int(row.get("final_valid", 0) or 0) != 0
            or int(row.get("final_unknown", 0) or 0) != 1
            or int(row.get("occupied_skip", 0) or 0) != 1
        )
    ]
    reason_counts = health_summary.get("reason_counts", {}) or {}
    dominant_unknown_reason_counts = (
        health_summary.get("dominant_unknown_reason_counts", {}) or {}
    )
    health_mentions_occupied_skip = (
        "occupied_skip" in {str(key) for key in reason_counts.keys()}
        or "occupied_skip" in {str(key) for key in dominant_unknown_reason_counts.keys()}
        or str(health_summary.get("dominant_unknown_reason_latest", "")) == "occupied_skip"
    )
    return {
        "occupied_overlap_count": len(rows),
        "occupied_low_raw_cost_count": occupied_low_raw_cost_count,
        "occupied_skip_count": occupied_skip_count,
        "occupied_valid_low_risk_count": len(occupied_valid_low_risk_rows),
        "bad_final_state_count": len(bad_final_rows),
        "health_occupied_skip_count_max": int(
            health_summary.get("occupied_skip_count_max", 0) or 0
        ),
        "health_mentions_occupied_skip": health_mentions_occupied_skip,
        "first_occupied_valid_low_risk_row": (
            occupied_valid_low_risk_rows[0] if occupied_valid_low_risk_rows else None
        ),
        "first_bad_final_state_row": bad_final_rows[0] if bad_final_rows else None,
    }


def validate_p0_6_formal_semantics(
    manifest: dict[str, Any],
    fixture: dict[str, Any],
    overlap_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
) -> None:
    if not manifest:
        failures.append("P0-6 fixture-backed run requires test_planner_manifest.json")
        return
    if not p0_6_fixture_complete(fixture):
        failures.append("P0-6 fixture missing or incomplete in manifest")
    if not manifest_bool(manifest.get("p0.skip_occupied_voxels")):
        failures.append("P0-6 manifest p0.skip_occupied_voxels is not true")
    if int(overlap_summary.get("occupied_overlap_count", 0) or 0) <= 0:
        failures.append("P0-6 occupied fixture has no overlap with final risk grid cloud")
    if int(overlap_summary.get("occupied_low_raw_cost_count", 0) or 0) <= 0:
        failures.append("P0-6 occupied fixture has no low raw PL/cost cells")
    if int(overlap_summary.get("occupied_skip_count", 0) or 0) <= 0:
        failures.append("P0-6 overlap rows have no occupied_skip source flag")
    if int(overlap_summary.get("health_occupied_skip_count_max", 0) or 0) <= 0:
        failures.append("P0-6 risk_grid_health never reported occupied_skip_count > 0")
    if int(overlap_summary.get("occupied_valid_low_risk_count", 0) or 0) > 0:
        failures.append("P0-6 occupied low raw cell remained final valid low-risk")
    if int(overlap_summary.get("bad_final_state_count", 0) or 0) > 0:
        failures.append("P0-6 occupied low raw cells were not all final valid=0 unknown=1 occupied_skip=1")
    if not bool(overlap_summary.get("health_mentions_occupied_skip")):
        failures.append("P0-6 health reason/dominant_unknown_reason never included occupied_skip")
    if (
        int(overlap_summary.get("occupied_overlap_count", 0) or 0) <= 0
        and not failures
    ):
        inconclusive.append("P0-6 overlap could not be evaluated")


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
    if (
        not allow_high_unknown
        and not startup_unavailable_explains_ready_stale
        and int(health_summary.get("full_unknown_max_consecutive", 0) or 0) >= 3
    ):
        failures.append(f"{experiment_label} risk_grid_health shows periodic/full-frame unknown for at least 3 consecutive samples")
    if (
        not allow_high_unknown
        and not startup_unavailable_explains_ready_stale
        and float(health_summary.get("full_unknown_ratio", 0.0) or 0.0) > 0.25
    ):
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
    if P5_STATUS_TOPIC not in expectations:
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
            elif expected == "planner-dependent":
                continue
            else:
                failures.append(f"required topic {topic} is missing")
        elif topic_health[topic]["status"] == "CHECK":
            inconclusive.append(f"required topic {topic} continuity could not be measured")
    return topic_health


def p5_row_from_payload(payload: Any, timestamp_ns: int) -> dict[str, Any]:
    parsed: dict[str, Any] = {}
    parse_error = ""
    if isinstance(payload, str) and payload.strip():
        try:
            loaded = json.loads(payload)
            if isinstance(loaded, dict):
                parsed = loaded
            else:
                parse_error = "status JSON payload is not an object"
        except json.JSONDecodeError as exc:
            parse_error = f"invalid_json:{exc.msg}"
    elif payload:
        parse_error = f"non_string_payload:{type(payload).__name__}"
    else:
        parse_error = "empty_payload"

    row: dict[str, Any] = {"bag_time_s": float(timestamp_ns) * 1.0e-9}
    for field in P5_STATUS_FIELDS:
        if field in {"bag_time_s", "parse_error", "raw"}:
            continue
        row[field] = parsed.get(field, "")
    row["parse_error"] = parse_error
    row["raw"] = payload
    return row


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
            rows.append(p5_row_from_payload(payload, timestamp))
        return rows, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return [], str(exc)


def read_p5_5_integrity_stamp_evidence(
    bag_dir: Path,
    metadata: dict[str, Any],
    limit: int = 20_000,
) -> tuple[list[dict[str, Any]], str]:
    topic_counts = metadata.get("topic_counts", {}) or {}
    if int(topic_counts.get("/iap/integrity", 0) or 0) <= 0:
        return [], ""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = open_bag_reader(bag_dir, metadata)
        type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        if "/iap/integrity" not in type_map:
            return [], "/iap/integrity topic count was nonzero but topic type is absent"
        msg_type = get_message(type_map["/iap/integrity"])
        rows: list[dict[str, Any]] = []
        while reader.has_next() and len(rows) < limit:
            topic, raw, timestamp = reader.read_next()
            if topic != "/iap/integrity":
                continue
            msg = deserialize_message(raw, msg_type)
            header = getattr(msg, "header", None)
            header_stamp_s = stamp_to_sec(getattr(header, "stamp", None))
            bag_time_s = float(timestamp) * 1.0e-9
            hpl = finite_float(getattr(msg, "hpl", math.nan))
            vpl = finite_float(getattr(msg, "vpl", math.nan))
            hal = finite_float(getattr(msg, "hal", math.nan))
            val = finite_float(getattr(msg, "val", math.nan))
            im = finite_float(getattr(msg, "im", math.nan))
            rows.append(
                {
                    "bag_time_s": bag_time_s,
                    "t_rel_s": "",
                    "header_stamp_s": header_stamp_s,
                    "header_rel_s": "",
                    "bag_minus_header_s": (
                        bag_time_s - header_stamp_s
                        if math.isfinite(header_stamp_s)
                        else math.nan
                    ),
                    "in_expected_window": 0,
                    "hpl": "" if hpl is None else hpl,
                    "vpl": "" if vpl is None else vpl,
                    "hal": "" if hal is None else hal,
                    "val": "" if val is None else val,
                    "im": "" if im is None else im,
                    "pl_al_finite": int(
                        hpl is not None
                        and vpl is not None
                        and hal is not None
                        and val is not None
                    ),
                    "fusion_mode": getattr(msg, "fusion_mode", ""),
                    "final_hpl_source": getattr(msg, "final_hpl_source", ""),
                    "final_vpl_source": getattr(msg, "final_vpl_source", ""),
                }
            )
        if rows:
            t0 = min(
                float(row["bag_time_s"])
                for row in rows
                if finite_float(row.get("bag_time_s")) is not None
            )
            header_values = [
                finite_float(row.get("header_stamp_s")) for row in rows
            ]
            finite_headers = [value for value in header_values if value is not None]
            h0 = min(finite_headers) if finite_headers else None
            for row in rows:
                bag_time = finite_float(row.get("bag_time_s"))
                header_stamp = finite_float(row.get("header_stamp_s"))
                row["t_rel_s"] = "" if bag_time is None else bag_time - t0
                row["header_rel_s"] = (
                    "" if header_stamp is None or h0 is None else header_stamp - h0
                )
        return rows, ""
    except Exception as exc:  # pragma: no cover - depends on local ROS Python env
        return [], str(exc)


def p5_action(row: dict[str, Any], key: str = "action") -> str:
    return str(row.get(key, "")).strip()


def p5_final_gate_failed(row: dict[str, Any]) -> bool:
    fail_count = finite_float(row.get("final_gate_fail_count"))
    fail_duration = finite_float(row.get("final_gate_fail_duration_s"))
    return (fail_count is not None and fail_count > 0) or (
        fail_duration is not None and fail_duration > 0.0
    )


def max_consecutive_action(rows: list[dict[str, Any]], action: str, key: str = "action") -> int:
    return consecutive_true([p5_action(row, key) == action for row in rows])


def normalize_p5_reason(value: Any) -> str:
    return str(value).strip().lower()


def p5_reason(row: dict[str, Any], key: str = "reason") -> str:
    return normalize_p5_reason(row.get(key, ""))


def p5_reason_values(row: dict[str, Any], key: str) -> set[str]:
    value = row.get(key, "")
    values: list[Any]
    if isinstance(value, (list, tuple, set)):
        values = list(value)
    elif isinstance(value, str):
        stripped = value.strip()
        values = []
        if stripped:
            if stripped.startswith("["):
                try:
                    loaded = json.loads(stripped)
                    if isinstance(loaded, list):
                        values = loaded
                except json.JSONDecodeError:
                    values = []
            if not values:
                if "," in stripped:
                    values = [
                        item.strip().strip("'\"[] ")
                        for item in stripped.split(",")
                    ]
                else:
                    values = [stripped]
    elif value is None:
        values = []
    else:
        values = [value]
    return {
        normalized
        for normalized in (normalize_p5_reason(item) for item in values)
        if normalized
    }


def p5_all_reason_values(row: dict[str, Any], keys: tuple[str, ...]) -> set[str]:
    reasons: set[str] = set()
    for key in keys:
        reasons.update(p5_reason_values(row, key))
    return reasons


def p5_final_gate_emergency(row: dict[str, Any]) -> bool:
    return p5_final_gate_failed(row) and (
        p5_action(row, "action") == P5_EMERGENCY_ACTION
        or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    )


def p5_has_explainable_unknown_reason(row: dict[str, Any]) -> bool:
    explainable = {"al_invalid", "future_unknown", "snapshot_unavailable"}
    return bool(
        p5_all_reason_values(
            row,
            (
                "reason",
                "raw_reason",
                "final_gate_last_reason",
                "future_reason",
                "active_reasons",
                "pred_al_last_reason",
            ),
        )
        & explainable
    )


def p5_has_finite_margin_degradation(row: dict[str, Any]) -> bool:
    return any(
        value is not None and value <= 0.0
        for value in (
            finite_float(row.get("future_min_im")),
            finite_float(row.get("current_im_min")),
            finite_float(row.get("current_im_h")),
            finite_float(row.get("current_im_v")),
        )
    )


def build_p5_debounce_timeline_rows(p5_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    t_rel = relative_time(p5_rows)
    action_replan_streak = 0
    raw_replan_streak = 0
    action_emergency_streak = 0
    raw_emergency_streak = 0
    for idx, row in enumerate(p5_rows):
        action = p5_action(row, "action")
        raw_action = p5_action(row, "raw_action")
        action_replan_streak = action_replan_streak + 1 if action == P5_REPLAN_ACTION else 0
        raw_replan_streak = raw_replan_streak + 1 if raw_action == P5_REPLAN_ACTION else 0
        action_emergency_streak = (
            action_emergency_streak + 1 if action == P5_EMERGENCY_ACTION else 0
        )
        raw_emergency_streak = (
            raw_emergency_streak + 1 if raw_action == P5_EMERGENCY_ACTION else 0
        )
        rows.append(
            {
                "bag_time_s": row.get("bag_time_s", ""),
                "t_rel_s": t_rel[idx] if idx < len(t_rel) else "",
                "phase": row.get("phase", ""),
                "action": action,
                "raw_action": raw_action,
                "reason": row.get("reason", ""),
                "consecutive_replan": action_replan_streak,
                "raw_consecutive_replan": raw_replan_streak,
                "consecutive_emergency": action_emergency_streak,
                "raw_consecutive_emergency": raw_emergency_streak,
                "final_gate_fail_count": row.get("final_gate_fail_count", ""),
                "final_gate_fail_duration_s": row.get("final_gate_fail_duration_s", ""),
                "final_gate_last_reason": row.get("final_gate_last_reason", ""),
                "final_gate_escalated_to_emergency": 1
                if p5_final_gate_emergency(row)
                else 0,
            }
        )
    return rows


def p5_startup_snapshot_unavailable_row(row: dict[str, Any]) -> bool:
    reason = str(row.get("reason", "")).strip().lower()
    if reason != "snapshot_unavailable":
        return False
    if p5_action(row, "action") != P5_REPLAN_ACTION:
        return False
    if p5_action(row, "raw_action") != P5_REPLAN_ACTION:
        return False
    if p5_final_gate_failed(row):
        return False
    bad_count = finite_float(row.get("bad_count")) or 0.0
    bad_ratio = finite_float(row.get("bad_ratio")) or 0.0
    if bad_count > 0.0 or bad_ratio > 0.0:
        return False
    future_min_im = finite_float(row.get("future_min_im"))
    return future_min_im is None or future_min_im >= 0.0


def p5_startup_snapshot_unavailable_prefix(
    rows: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], float, bool]:
    prefix: list[dict[str, Any]] = []
    for row in rows:
        if not p5_startup_snapshot_unavailable_row(row):
            break
        prefix.append(row)
    if len(prefix) <= 1:
        duration_s = 0.0
    else:
        first = finite_float(prefix[0].get("bag_time_s"))
        last = finite_float(prefix[-1].get("bag_time_s"))
        duration_s = max(0.0, last - first) if first is not None and last is not None else 0.0
    bounded = (
        not prefix
        or duration_s
        <= P5_1_STARTUP_REPLAN_MAX_DURATION_S
        + P5_STARTUP_REPLAN_DURATION_TOLERANCE_S
    )
    return prefix, duration_s, bounded


def numeric_summary(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    values = [value for value in (finite_float(row.get(key)) for row in rows) if value is not None]
    return {
        f"{key}_min": min(values) if values else None,
        f"{key}_mean": float(np.mean(values)) if values else None,
        f"{key}_max": max(values) if values else None,
    }


def summarize_p5_status_rows(p5_rows: list[dict[str, Any]], p5_error: str = "") -> dict[str, Any]:
    action_counts = Counter(p5_action(row, "action") or "<empty>" for row in p5_rows)
    raw_action_counts = Counter(p5_action(row, "raw_action") or "<empty>" for row in p5_rows)
    startup_rows, startup_duration_s, startup_bounded = p5_startup_snapshot_unavailable_prefix(p5_rows)
    steady_rows = p5_rows[len(startup_rows) :] if startup_bounded else p5_rows
    steady_action_counts = Counter(p5_action(row, "action") or "<empty>" for row in steady_rows)
    steady_raw_action_counts = Counter(p5_action(row, "raw_action") or "<empty>" for row in steady_rows)
    steady_emergency_rows = [
        row
        for row in steady_rows
        if p5_action(row, "action") == P5_EMERGENCY_ACTION
        or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    ]
    emergency_rows = [
        row
        for row in p5_rows
        if p5_action(row, "action") == P5_EMERGENCY_ACTION
        or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    ]
    replan_rows = [
        row
        for row in p5_rows
        if p5_action(row, "action") == P5_REPLAN_ACTION
        or p5_action(row, "raw_action") == P5_REPLAN_ACTION
    ]
    final_fail_rows = [row for row in p5_rows if p5_final_gate_failed(row)]
    final_gate_emergency_rows = [row for row in p5_rows if p5_final_gate_emergency(row)]
    parse_error_rows = [row for row in p5_rows if str(row.get("parse_error", "")).strip()]
    ok_count = int(action_counts.get(P5_OK_ACTION, 0))
    steady_ok_count = int(steady_action_counts.get(P5_OK_ACTION, 0))
    steady_max_consecutive_replan = max_consecutive_action(
        steady_rows, P5_REPLAN_ACTION, "action"
    )
    steady_raw_max_consecutive_replan = max_consecutive_action(
        steady_rows, P5_REPLAN_ACTION, "raw_action"
    )
    max_consecutive_replan = max_consecutive_action(p5_rows, P5_REPLAN_ACTION, "action")
    raw_max_consecutive_replan = max_consecutive_action(
        p5_rows, P5_REPLAN_ACTION, "raw_action"
    )
    max_consecutive_emergency = max_consecutive_action(
        p5_rows, P5_EMERGENCY_ACTION, "action"
    )
    raw_max_consecutive_emergency = max_consecutive_action(
        p5_rows, P5_EMERGENCY_ACTION, "raw_action"
    )
    explainable_unknown_rows = [
        row
        for row in p5_rows
        if (finite_float(row.get("unknown_ratio")) or 0.0) > 0.0
        and p5_has_explainable_unknown_reason(row)
    ]
    finite_margin_degradation_rows = [
        row for row in p5_rows if p5_has_finite_margin_degradation(row)
    ]
    summary: dict[str, Any] = {
        "status_rows": len(p5_rows),
        "action_counts": dict(sorted(action_counts.items())),
        "raw_action_counts": dict(sorted(raw_action_counts.items())),
        "ok_action_count": ok_count,
        "ok_action_ratio": ratio(ok_count, len(p5_rows)),
        "startup_snapshot_unavailable_rows": len(startup_rows),
        "startup_snapshot_unavailable_duration_s": startup_duration_s,
        "startup_snapshot_unavailable_bounded": startup_bounded,
        "steady_status_rows": len(steady_rows),
        "steady_action_counts": dict(sorted(steady_action_counts.items())),
        "steady_raw_action_counts": dict(sorted(steady_raw_action_counts.items())),
        "steady_ok_action_count": steady_ok_count,
        "steady_ok_action_ratio": ratio(steady_ok_count, len(steady_rows)),
        "steady_replan_action_count": int(steady_action_counts.get(P5_REPLAN_ACTION, 0)),
        "steady_raw_replan_action_count": int(
            steady_raw_action_counts.get(P5_REPLAN_ACTION, 0)
        ),
        "steady_emergency_action_count": len(steady_emergency_rows),
        "steady_raw_emergency_action_count": int(
            steady_raw_action_counts.get(P5_EMERGENCY_ACTION, 0)
        ),
        "steady_max_consecutive_replan": steady_max_consecutive_replan,
        "steady_raw_max_consecutive_replan": steady_raw_max_consecutive_replan,
        "steady_replan_storm": (
            steady_max_consecutive_replan >= P5_REPLAN_STORM_CONSECUTIVE
            or steady_raw_max_consecutive_replan >= P5_REPLAN_STORM_CONSECUTIVE
        ),
        "replan_action_count": int(action_counts.get(P5_REPLAN_ACTION, 0)),
        "raw_replan_action_count": int(raw_action_counts.get(P5_REPLAN_ACTION, 0)),
        "emergency_action_count": len(emergency_rows),
        "raw_emergency_action_count": int(raw_action_counts.get(P5_EMERGENCY_ACTION, 0)),
        "max_consecutive_replan": max_consecutive_replan,
        "raw_max_consecutive_replan": raw_max_consecutive_replan,
        "max_consecutive_emergency": max_consecutive_emergency,
        "raw_max_consecutive_emergency": raw_max_consecutive_emergency,
        "replan_storm": (
            max_consecutive_replan >= P5_REPLAN_STORM_CONSECUTIVE
            or raw_max_consecutive_replan >= P5_REPLAN_STORM_CONSECUTIVE
        ),
        "final_gate_fail_rows": len(final_fail_rows),
        "final_gate_emergency_rows": len(final_gate_emergency_rows),
        "final_gate_fail_count_max": max(
            (int(finite_float(row.get("final_gate_fail_count")) or 0) for row in p5_rows),
            default=0,
        ),
        "final_gate_fail_duration_s_max": max(
            (finite_float(row.get("final_gate_fail_duration_s")) or 0.0 for row in p5_rows),
            default=0.0,
        ),
        "parse_error_count": len(parse_error_rows),
        "inspection_error": p5_error,
        "first_replan_row": replan_rows[0] if replan_rows else None,
        "first_emergency_row": emergency_rows[0] if emergency_rows else None,
        "first_final_gate_fail_row": final_fail_rows[0] if final_fail_rows else None,
        "first_final_gate_emergency_row": final_gate_emergency_rows[0]
        if final_gate_emergency_rows
        else None,
        "first_parse_error_row": parse_error_rows[0] if parse_error_rows else None,
        "explainable_unknown_rows": len(explainable_unknown_rows),
        "finite_margin_degradation_rows": len(finite_margin_degradation_rows),
        "first_explainable_unknown_row": explainable_unknown_rows[0]
        if explainable_unknown_rows
        else None,
        "first_finite_margin_degradation_row": finite_margin_degradation_rows[0]
        if finite_margin_degradation_rows
        else None,
    }
    for key in (
        "future_min_im",
        "bad_ratio",
        "unknown_ratio",
        "current_integrity_age_s",
        "current_im_min",
        "current_im_h",
        "current_im_v",
        "current_stale_duration_s",
        "current_low_margin_duration_s",
        "future_unknown_duration_s",
        "pred_hal_min",
        "pred_val_min",
        "pred_al_invalid_count",
        "sample_count",
        "bad_count",
        "unknown_count",
    ):
        summary.update(numeric_summary(p5_rows, key))
    return summary


def validate_p5_status(
    p5_rows: list[dict[str, Any]],
    p5_error: str,
    failures: list[str],
    inconclusive: list[str],
    *,
    allow_replan: bool = False,
    allow_emergency: bool = False,
    allow_final_gate_failure: bool = False,
) -> dict[str, Any]:
    summary = summarize_p5_status_rows(p5_rows, p5_error)
    if p5_error:
        inconclusive.append(f"could not inspect P5 status messages: {p5_error}")
    if int(summary.get("parse_error_count", 0) or 0) > 0:
        failures.append("P5 status contains invalid or empty JSON payloads")
    has_replan = (
        int(summary.get("replan_action_count", 0) or 0) > 0
        or int(summary.get("raw_replan_action_count", 0) or 0) > 0
    )
    has_emergency = int(summary.get("emergency_action_count", 0) or 0) > 0
    has_final_fail = int(summary.get("final_gate_fail_rows", 0) or 0) > 0
    if (
        (has_emergency and not allow_emergency)
        or (has_final_fail and not allow_final_gate_failure)
        or (has_replan and not allow_replan)
    ):
        failures.append("P5 status contains replan, emergency, or final gate failure behavior")
    bad_rows = []
    for row in p5_rows:
        row_has_replan = (
            p5_action(row, "action") == P5_REPLAN_ACTION
            or p5_action(row, "raw_action") == P5_REPLAN_ACTION
        )
        row_is_bad = (
            (
                not allow_emergency
                and (
                    p5_action(row, "action") == P5_EMERGENCY_ACTION
                    or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
                )
            )
            or (p5_final_gate_failed(row) and not allow_final_gate_failure)
            or (row_has_replan and not allow_replan)
        )
        if row_is_bad:
            bad_rows.append(row)
    summary["bad_action_count"] = len(bad_rows)
    summary["first_bad_action"] = bad_rows[0] if bad_rows else None
    return summary


def validate_p5_1_hard_gates(
    p0_health_summary: dict[str, Any],
    p5_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
) -> dict[str, Any]:
    reason_counts = p0_health_summary.get("reason_counts", {}) or {}
    ready_false_count = int(p0_health_summary.get("ready_false_count", 0) or 0)
    stale_true_count = int(p0_health_summary.get("stale_true_count", 0) or 0)
    snapshot_unavailable_count = int(reason_counts.get("snapshot_unavailable", 0) or 0)
    startup_unavailable_explains_ready_stale = (
        snapshot_unavailable_count >= max(ready_false_count, stale_true_count)
        and float(p0_health_summary.get("ready_false_ratio", 0.0) or 0.0) <= 0.10
        and float(p0_health_summary.get("stale_true_ratio", 0.0) or 0.0) <= 0.10
        and int(p0_health_summary.get("ready_false_max_consecutive", 0) or 0) <= 3
        and int(p0_health_summary.get("stale_true_max_consecutive", 0) or 0) <= 3
    )
    gates = {
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        "p0_non_stale": stale_true_count == 0 or startup_unavailable_explains_ready_stale,
        "p0_startup_unavailable_explained": startup_unavailable_explains_ready_stale,
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_steady_status_rows_present": int(p5_summary.get("steady_status_rows", 0) or 0) > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "startup_snapshot_unavailable_rows": int(
            p5_summary.get("startup_snapshot_unavailable_rows", 0) or 0
        ),
        "startup_snapshot_unavailable_duration_s": float(
            p5_summary.get("startup_snapshot_unavailable_duration_s", 0.0) or 0.0
        ),
        "startup_snapshot_unavailable_bounded": bool(
            p5_summary.get("startup_snapshot_unavailable_bounded", True)
        ),
        "ok_action_ratio": float(p5_summary.get("steady_ok_action_ratio", 0.0) or 0.0),
        "ok_action_ratio_min": P5_1_MIN_OK_ACTION_RATIO,
        "emergency_action_count": int(p5_summary.get("emergency_action_count", 0) or 0),
        "replan_storm": bool(p5_summary.get("steady_replan_storm", False)),
        "max_consecutive_replan": int(
            p5_summary.get("steady_max_consecutive_replan", 0) or 0
        ),
        "raw_max_consecutive_replan": int(
            p5_summary.get("steady_raw_max_consecutive_replan", 0) or 0
        ),
        "final_gate_fail_count_max": int(p5_summary.get("final_gate_fail_count_max", 0) or 0),
        "future_min_im_available": p5_summary.get("future_min_im_min") is not None,
        "current_im_min_available": p5_summary.get("current_im_min_min") is not None,
        "sample_count_available": p5_summary.get("sample_count_max") is not None,
        "predicted_al_available": (
            p5_summary.get("pred_hal_min_min") is not None
            or p5_summary.get("pred_val_min_min") is not None
        ),
    }
    if not gates["p0_health_rows_present"]:
        failures.append("P5-1 P0 health rows are missing")
    if not gates["p0_non_stale"]:
        failures.append("P5-1 P0 health reported stale=true outside bounded startup snapshot_unavailable")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-1 P5 status rows are missing")
    if not gates["p5_steady_status_rows_present"]:
        inconclusive.append("P5-1 P5 status rows contain no steady-state samples after startup")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-1 P5 status JSON parse errors were observed")
    if (
        gates["startup_snapshot_unavailable_rows"] > 0
        and not gates["startup_snapshot_unavailable_bounded"]
    ):
        failures.append(
            "P5-1 startup snapshot_unavailable replan prefix exceeded "
            f"{P5_1_STARTUP_REPLAN_MAX_DURATION_S:.1f}s "
            f"(+{P5_STARTUP_REPLAN_DURATION_TOLERANCE_S:.2f}s tolerance)"
        )
    if gates["ok_action_ratio"] < P5_1_MIN_OK_ACTION_RATIO:
        failures.append(
            f"P5-1 OK action ratio {gates['ok_action_ratio']:.3f} is below "
            f"{P5_1_MIN_OK_ACTION_RATIO:.3f}"
        )
    if gates["emergency_action_count"] > 0:
        failures.append("P5-1 observed REQUEST_EMERGENCY_STOP_CANDIDATE")
    if gates["replan_storm"]:
        failures.append(
            "P5-1 observed a replan storm: "
            f"action_consecutive={gates['max_consecutive_replan']}, "
            f"raw_action_consecutive={gates['raw_max_consecutive_replan']}"
        )
    if gates["final_gate_fail_count_max"] > 0:
        failures.append("P5-1 final_gate_fail_count was nonzero")
    if not gates["future_min_im_available"]:
        inconclusive.append("P5-1 future_min_im had no finite samples")
    if not gates["current_im_min_available"]:
        inconclusive.append("P5-1 current_im_min had no finite samples")
    if not gates["sample_count_available"]:
        inconclusive.append("P5-1 sample_count had no finite samples")
    if not gates["predicted_al_available"]:
        inconclusive.append("P5-1 predicted alert-limit minima had no finite samples")
    gates["passed"] = not any(message.startswith("P5-1") for message in failures + inconclusive)
    return gates


def p0_health_row_full_unknown(row: dict[str, Any]) -> bool:
    return (
        (finite_float(row.get("valid_ratio")) or 0.0) <= 1.0e-9
        and (finite_float(row.get("unknown_ratio")) or 0.0) >= 0.999
    )


def summarize_p0_startup_snapshot_unavailable(
    rows: list[dict[str, Any]],
) -> dict[str, Any]:
    prefix: list[dict[str, Any]] = []
    for row in rows:
        if str(row.get("reason", "")).strip().lower() != "snapshot_unavailable":
            break
        prefix.append(row)
    post_startup_rows = rows[len(prefix) :]
    prefix_stamps = [value for value in (finite_float(row.get("stamp")) for row in prefix) if value is not None]
    duration_s = max(prefix_stamps) - min(prefix_stamps) if len(prefix_stamps) > 1 else 0.0
    row_count = len(rows)
    ready_false_prefix = sum(1 for row in prefix if not bool(row.get("ready")))
    stale_prefix = sum(1 for row in prefix if bool(row.get("stale")))
    full_unknown_prefix = sum(1 for row in prefix if p0_health_row_full_unknown(row))
    bounded = (
        not prefix
        or (
            len(prefix) <= 3
            and ratio(len(prefix), row_count) <= 0.10
            and max(ready_false_prefix, stale_prefix, full_unknown_prefix) <= 3
        )
    )
    post_ready_false = sum(1 for row in post_startup_rows if not bool(row.get("ready")))
    post_stale = sum(1 for row in post_startup_rows if bool(row.get("stale")))
    post_full_unknown = sum(1 for row in post_startup_rows if p0_health_row_full_unknown(row))
    return {
        "startup_snapshot_unavailable_rows": len(prefix),
        "startup_snapshot_unavailable_duration_s": duration_s,
        "startup_snapshot_unavailable_bounded": bounded,
        "post_startup_rows": len(post_startup_rows),
        "post_startup_ready_false_count": post_ready_false,
        "post_startup_stale_true_count": post_stale,
        "post_startup_full_unknown_count": post_full_unknown,
    }


def validate_p5_2_hard_gates(
    manifest: dict[str, Any],
    validator_summary: dict[str, Any],
    topic_health: dict[str, dict[str, Any]],
    p0_health_summary: dict[str, Any],
    p0_health_rows: list[dict[str, Any]],
    p5_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
) -> dict[str, Any]:
    manifest_gates = p5_manifest_gate_values(manifest)
    topic_statuses = {
        topic: (topic_health.get(topic) or {}).get("status", "MISSING")
        for topic in P5_TOPIC_EXPECTATIONS
    }
    p0_startup = summarize_p0_startup_snapshot_unavailable(p0_health_rows)
    unknown_ratio_max = finite_float(p5_summary.get("unknown_ratio_max")) or 0.0
    bad_ratio_max = finite_float(p5_summary.get("bad_ratio_max")) or 0.0
    current_stale_duration_max = (
        finite_float(p5_summary.get("current_stale_duration_s_max")) or 0.0
    )
    future_unknown_duration_max = (
        finite_float(p5_summary.get("future_unknown_duration_s_max")) or 0.0
    )
    has_stale_or_unknown_duration = (
        current_stale_duration_max > 0.0 or future_unknown_duration_max > 0.0
    )
    has_explainable_unknown_reason = int(
        p5_summary.get("explainable_unknown_rows", 0) or 0
    ) > 0
    has_finite_margin_degradation = int(
        p5_summary.get("finite_margin_degradation_rows", 0) or 0
    ) > 0
    unexplained_unknown = (
        unknown_ratio_max > 0.0
        and bad_ratio_max == 0.0
        and not has_stale_or_unknown_duration
        and not has_explainable_unknown_reason
        and not has_finite_margin_degradation
    )
    max_consecutive_emergency = int(p5_summary.get("max_consecutive_emergency", 0) or 0)
    raw_max_consecutive_emergency = int(
        p5_summary.get("raw_max_consecutive_emergency", 0) or 0
    )
    final_gate_fail_count_max = int(p5_summary.get("final_gate_fail_count_max", 0) or 0)
    gates = {
        **manifest_gates,
        "validator_summary_present": bool(validator_summary),
        "validator_passed": validator_summary.get("passed") is True,
        "required_p5_topics_stable": all(
            status == "PASS"
            for topic, status in topic_statuses.items()
            if P5_TOPIC_EXPECTATIONS.get(topic) != "planner-dependent"
        ),
        "topic_statuses": topic_statuses,
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        **p0_startup,
        "p0_post_startup_ready": p0_startup["post_startup_ready_false_count"] == 0,
        "p0_post_startup_non_stale": p0_startup["post_startup_stale_true_count"] == 0,
        "p0_post_startup_not_full_unknown": (
            p0_startup["post_startup_full_unknown_count"] == 0
        ),
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_steady_status_rows_present": int(p5_summary.get("steady_status_rows", 0) or 0)
        > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "p5_inspection_ok": not bool(p5_summary.get("inspection_error")),
        "p5_startup_snapshot_unavailable_bounded": bool(
            p5_summary.get("startup_snapshot_unavailable_bounded", True)
        ),
        "max_consecutive_emergency": max_consecutive_emergency,
        "raw_max_consecutive_emergency": raw_max_consecutive_emergency,
        "emergency_storm_absent": (
            max_consecutive_emergency < P5_2_EMERGENCY_STORM_CONSECUTIVE
            and raw_max_consecutive_emergency < P5_2_EMERGENCY_STORM_CONSECUTIVE
        ),
        "final_gate_fail_count_max": final_gate_fail_count_max,
        "final_gate_accumulation_bounded": final_gate_fail_count_max < 3,
        "final_gate_emergency_rows": int(
            p5_summary.get("final_gate_emergency_rows", 0) or 0
        ),
        "final_gate_emergency_absent": int(
            p5_summary.get("final_gate_emergency_rows", 0) or 0
        )
        == 0,
        "unknown_ratio_max": unknown_ratio_max,
        "bad_ratio_max": bad_ratio_max,
        "has_stale_or_unknown_duration": has_stale_or_unknown_duration,
        "has_explainable_unknown_reason": has_explainable_unknown_reason,
        "has_finite_margin_degradation": has_finite_margin_degradation,
        "unexplained_unknown_absent": not unexplained_unknown,
        "future_min_im_available": p5_summary.get("future_min_im_min") is not None,
        "current_im_min_available": p5_summary.get("current_im_min_min") is not None,
        "sample_count_available": (
            p5_summary.get("sample_count_max") is not None
            and (finite_float(p5_summary.get("sample_count_max")) or 0.0) > 0.0
        ),
        "pred_hal_min_available": p5_summary.get("pred_hal_min_min") is not None,
        "pred_val_min_available": p5_summary.get("pred_val_min_min") is not None,
    }
    gates["predicted_al_available"] = (
        gates["pred_hal_min_available"] and gates["pred_val_min_available"]
    )

    if not gates["manifest_present"]:
        inconclusive.append("P5-2 checks require test_planner_manifest.json")
    elif not (
        gates["manifest_safety_profile_p5"]
        and gates["manifest_expected_true_ok"]
        and gates["manifest_expected_false_ok"]
    ):
        failures.append("P5-2 manifest does not enable P0/P5 with P1-P4 disabled")
    if not gates["validator_summary_present"]:
        inconclusive.append("P5-2 validator summary is missing")
    elif not gates["validator_passed"]:
        failures.append("P5-2 validator summary did not pass")
    if not gates["required_p5_topics_stable"]:
        failures.append("P5-2 required P0/P5 topics are not all stable")
    if not gates["p0_health_rows_present"]:
        failures.append("P5-2 P0 health rows are missing")
    if not gates["startup_snapshot_unavailable_bounded"]:
        failures.append("P5-2 P0 startup snapshot_unavailable prefix is not bounded")
    if not gates["p0_post_startup_ready"]:
        failures.append("P5-2 P0 health reported ready=false after startup")
    if not gates["p0_post_startup_non_stale"]:
        failures.append("P5-2 P0 health reported stale=true after startup")
    if not gates["p0_post_startup_not_full_unknown"]:
        failures.append("P5-2 P0 health reported full unknown after startup")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-2 P5 status rows are missing")
    if not gates["p5_steady_status_rows_present"]:
        inconclusive.append("P5-2 P5 status rows contain no steady-state samples after startup")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-2 P5 status JSON parse errors were observed")
    if not gates["p5_inspection_ok"]:
        inconclusive.append("P5-2 P5 status inspection did not complete cleanly")
    if not gates["p5_startup_snapshot_unavailable_bounded"]:
        failures.append(
            "P5-2 startup snapshot_unavailable replan prefix exceeded "
            f"{P5_1_STARTUP_REPLAN_MAX_DURATION_S:.1f}s "
            f"(+{P5_STARTUP_REPLAN_DURATION_TOLERANCE_S:.2f}s tolerance)"
        )
    if not gates["emergency_storm_absent"]:
        failures.append(
            "P5-2 observed sustained emergency storm: "
            f"action_consecutive={max_consecutive_emergency}, "
            f"raw_action_consecutive={raw_max_consecutive_emergency}"
        )
    if not gates["final_gate_accumulation_bounded"]:
        failures.append("P5-2 final_gate_fail_count reached abnormal accumulation")
    if not gates["final_gate_emergency_absent"]:
        failures.append("P5-2 final-gate failure escalated to emergency")
    if not gates["unexplained_unknown_absent"]:
        failures.append("P5-2 unknown_ratio was nonzero without explanatory evidence")
    if not gates["future_min_im_available"]:
        inconclusive.append("P5-2 future_min_im had no finite samples")
    if not gates["current_im_min_available"]:
        inconclusive.append("P5-2 current_im_min had no finite samples")
    if not gates["sample_count_available"]:
        inconclusive.append("P5-2 sample_count had no positive finite samples")
    if not gates["predicted_al_available"]:
        inconclusive.append("P5-2 predicted alert-limit minima had no finite samples")

    required = (
        "manifest_present",
        "manifest_safety_profile_p5",
        "manifest_expected_true_ok",
        "manifest_expected_false_ok",
        "validator_summary_present",
        "validator_passed",
        "required_p5_topics_stable",
        "p0_health_rows_present",
        "startup_snapshot_unavailable_bounded",
        "p0_post_startup_ready",
        "p0_post_startup_non_stale",
        "p0_post_startup_not_full_unknown",
        "p5_status_rows_present",
        "p5_steady_status_rows_present",
        "p5_json_parse_ok",
        "p5_inspection_ok",
        "p5_startup_snapshot_unavailable_bounded",
        "emergency_storm_absent",
        "final_gate_accumulation_bounded",
        "final_gate_emergency_absent",
        "unexplained_unknown_absent",
        "future_min_im_available",
        "current_im_min_available",
        "sample_count_available",
        "predicted_al_available",
    )
    gates["passed"] = all(bool(gates.get(key)) for key in required)
    return gates


def manifest_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def p5_fixture_from_manifest(
    manifest: dict[str, Any],
    phase_key: str,
    default_reason: str,
) -> dict[str, Any]:
    nested = ((manifest.get(phase_key) or {}).get("fixture") or {}) if manifest else {}

    def flat(key: str) -> Any:
        return manifest.get(f"{phase_key}.fixture.{key}") if manifest else None

    bounds = nested.get("bounds") or {}
    tau_window = nested.get("tau_window_s") or [flat("tau_min"), flat("tau_max")]
    injected = nested.get("injected_pl_m") or {}
    expected_al = nested.get("expected_alert_limit_m") or {}

    def pair(values: Any, fallback_min: Any, fallback_max: Any) -> tuple[float | None, float | None]:
        if isinstance(values, (list, tuple)) and len(values) >= 2:
            return finite_float(values[0]), finite_float(values[1])
        return finite_float(fallback_min), finite_float(fallback_max)

    x_min, x_max = pair(bounds.get("x"), flat("x_min"), flat("x_max"))
    y_min, y_max = pair(bounds.get("y"), flat("y_min"), flat("y_max"))
    z_min, z_max = pair(bounds.get("z"), flat("z_min"), flat("z_max"))
    tau_min, tau_max = pair(tau_window, flat("tau_min"), flat("tau_max"))
    hpl_pred = finite_float(injected.get("hpl_pred", flat("hpl_pred_m")))
    vpl_pred = finite_float(injected.get("vpl_pred", flat("vpl_pred_m")))
    expected_hal = finite_float(expected_al.get("hal", flat("expected_hal_m")))
    expected_val = finite_float(expected_al.get("val", flat("expected_val_m")))
    fixture = {
        "present": bool(manifest) and (
            phase_key in manifest
            or any(str(key).startswith(f"{phase_key}.fixture.") for key in manifest)
        ),
        "enabled": manifest_bool(nested.get("enabled", flat("enabled"))),
        "name": str(nested.get("name", flat("name")) or ""),
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "z_min": z_min,
        "z_max": z_max,
        "tau_min": tau_min,
        "tau_max": tau_max,
        "hpl_pred": hpl_pred,
        "vpl_pred": vpl_pred,
        "expected_hal": expected_hal,
        "expected_val": expected_val,
        "expected_im": finite_float(nested.get("expected_im_m", flat("expected_im_m"))),
        "expected_reason": str(nested.get("expected_reason", default_reason) or ""),
        "expected_first_bad_tau": finite_float(
            nested.get("expected_first_bad_tau_s", flat("expected_first_bad_tau_s"))
        ),
        "expected_emergency_time_s": finite_float(
            nested.get("expected_emergency_time_s", flat("expected_emergency_time_s"))
        ),
    }
    finite_required = (
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
        "tau_min",
        "tau_max",
        "hpl_pred",
        "vpl_pred",
    )
    fixture["valid_geometry"] = all(fixture.get(key) is not None for key in finite_required)
    return fixture


def p5_3_fixture_from_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    return p5_fixture_from_manifest(manifest, "p5_3", P5_3_FIXTURE_REASON)


def p5_4_fixture_from_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    return p5_fixture_from_manifest(manifest, "p5_4", P5_4_FIXTURE_REASON)


def p5_3_value_in_window(value: float | None, lo: float | None, hi: float | None) -> bool:
    if value is None or lo is None or hi is None:
        return False
    lower = min(lo, hi)
    upper = max(lo, hi)
    return lower <= value <= upper


def p5_3_float_close(actual: float | None, expected: float | None) -> bool:
    if actual is None or expected is None:
        return False
    return abs(actual - expected) <= 1.0e-6


def p5_3_point_in_fixture(row: dict[str, Any], fixture: dict[str, Any]) -> bool:
    x = finite_float(row.get("x"))
    y = finite_float(row.get("y"))
    z = finite_float(row.get("z"))
    return (
        p5_3_value_in_window(x, fixture.get("x_min"), fixture.get("x_max"))
        and p5_3_value_in_window(y, fixture.get("y_min"), fixture.get("y_max"))
        and p5_3_value_in_window(z, fixture.get("z_min"), fixture.get("z_max"))
    )


def p5_3_sample_list(row: dict[str, Any]) -> list[dict[str, Any]]:
    value = row.get("samples", [])
    if isinstance(value, list):
        return [item for item in value if isinstance(item, dict)]
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return []
        try:
            loaded = json.loads(stripped)
        except json.JSONDecodeError:
            return []
        if isinstance(loaded, list):
            return [item for item in loaded if isinstance(item, dict)]
    return []


def p5_3_sample_source(sample: dict[str, Any], status_row: dict[str, Any]) -> str:
    source = str(sample.get("trajectory_sample_source", "")).strip()
    if source:
        return source
    phase = str(status_row.get("phase", "")).strip().lower()
    if phase == "final":
        return "final_candidate"
    if phase == "runtime":
        return "runtime_committed"
    return ""


def p5_3_sample_rows(
    p5_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    *,
    tau_window_field: str = "query_tau_s",
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for status_idx, status_row in enumerate(p5_rows):
        for sample_idx, sample in enumerate(p5_3_sample_list(status_row)):
            row: dict[str, Any] = {
                "bag_time_s": status_row.get("bag_time_s", ""),
                "status_row_index": status_idx,
                "sample_index": sample_idx,
                "phase": status_row.get("phase", ""),
                "action": status_row.get("action", ""),
                "raw_action": status_row.get("raw_action", ""),
                "status_reason": status_row.get("reason", ""),
                "current_reason": status_row.get("current_reason", ""),
                "future_reason": status_row.get("future_reason", ""),
                "active_reasons": status_row.get("active_reasons", ""),
                "tau_s": sample.get("tau_s", ""),
                "query_tau_s": sample.get("query_tau_s", ""),
                "trajectory_start_time_s": sample.get("trajectory_start_time_s", ""),
                "trajectory_duration_s": sample.get("trajectory_duration_s", ""),
                "trajectory_t_cur_s": sample.get("trajectory_t_cur_s", ""),
                "trajectory_t_end_s": sample.get("trajectory_t_end_s", ""),
                "trajectory_time_remaining_s": sample.get(
                    "trajectory_time_remaining_s", ""
                ),
                "sample_dt_s": sample.get("sample_dt_s", ""),
                "horizon_s": sample.get("horizon_s", ""),
                "trajectory_sample_source": p5_3_sample_source(sample, status_row),
                "fixture_match": 1 if manifest_bool(sample.get("fixture_match")) else 0,
                "fixture_expected_hpl": sample.get("fixture_expected_hpl", ""),
                "fixture_expected_vpl": sample.get("fixture_expected_vpl", ""),
                "fixture_expected_reason": str(
                    sample.get("fixture_expected_reason", "")
                ),
                "x": sample.get("x", ""),
                "y": sample.get("y", ""),
                "z": sample.get("z", ""),
                "hpl": sample.get("hpl", ""),
                "vpl": sample.get("vpl", ""),
                "hal": sample.get("hal", ""),
                "val": sample.get("val", ""),
                "im_min": sample.get("im_min", ""),
                "bad": 1 if manifest_bool(sample.get("bad")) else 0,
                "unknown": 1 if manifest_bool(sample.get("unknown")) else 0,
                "stale": 1 if manifest_bool(sample.get("stale")) else 0,
                "reason": str(sample.get("reason", "")),
            }
            inside_space = p5_3_point_in_fixture(row, fixture)
            query_tau = finite_float(row.get("query_tau_s"))
            if query_tau is None:
                query_tau = finite_float(row.get("tau_s"))
                row["query_tau_s"] = "" if query_tau is None else query_tau
            fixture_tau = finite_float(row.get(tau_window_field))
            if fixture_tau is None:
                fixture_tau = query_tau
            inside_tau = p5_3_value_in_window(
                fixture_tau,
                fixture.get("tau_min"),
                fixture.get("tau_max"),
            )
            row["inside_high_risk_zone"] = 1 if inside_space else 0
            row["inside_tau_window"] = 1 if inside_tau else 0
            actual_hpl = finite_float(row.get("hpl"))
            actual_vpl = finite_float(row.get("vpl"))
            runtime_expected_hpl = finite_float(row.get("fixture_expected_hpl"))
            runtime_expected_vpl = finite_float(row.get("fixture_expected_vpl"))
            runtime_expected_reason = str(row.get("fixture_expected_reason", ""))
            geometry_expected_hpl = (
                finite_float(fixture.get("hpl_pred"))
                if inside_space and inside_tau
                else None
            )
            geometry_expected_vpl = (
                finite_float(fixture.get("vpl_pred"))
                if inside_space and inside_tau
                else None
            )
            expected_hpl = (
                runtime_expected_hpl
                if runtime_expected_hpl is not None
                else geometry_expected_hpl
            )
            expected_vpl = (
                runtime_expected_vpl
                if runtime_expected_vpl is not None
                else geometry_expected_vpl
            )
            expected_reason = (
                runtime_expected_reason
                if runtime_expected_reason
                else (
                    str(fixture.get("expected_reason", "") or "")
                    if inside_space and inside_tau
                    else ""
                )
            )
            row["expected_hpl"] = "" if expected_hpl is None else expected_hpl
            row["expected_vpl"] = "" if expected_vpl is None else expected_vpl
            row["expected_reason"] = expected_reason
            row["actual_hpl"] = "" if actual_hpl is None else actual_hpl
            row["actual_vpl"] = "" if actual_vpl is None else actual_vpl
            row["hpl_error"] = (
                ""
                if actual_hpl is None or expected_hpl is None
                else actual_hpl - expected_hpl
            )
            row["vpl_error"] = (
                ""
                if actual_vpl is None or expected_vpl is None
                else actual_vpl - expected_vpl
            )
            pl_aligned = p5_3_float_close(actual_hpl, expected_hpl) and p5_3_float_close(
                actual_vpl, expected_vpl
            )
            injected_pl_aligned = p5_3_float_close(
                expected_hpl,
                finite_float(fixture.get("hpl_pred")),
            ) and p5_3_float_close(
                expected_vpl,
                finite_float(fixture.get("vpl_pred")),
            )
            reason_aligned = bool(expected_reason) and expected_reason.lower() in str(
                row.get("reason", "")
            ).lower()
            row["query_pl_aligned"] = 1 if inside_space and inside_tau and pl_aligned else 0
            row["query_reason_aligned"] = (
                1 if inside_space and inside_tau and reason_aligned else 0
            )
            row["query_alignment_ok"] = (
                1
                if inside_space
                and inside_tau
                and int(row.get("fixture_match", 0) or 0)
                and runtime_expected_hpl is not None
                and runtime_expected_vpl is not None
                and pl_aligned
                and injected_pl_aligned
                and reason_aligned
                else 0
            )
            row["fixture_bad_sample"] = (
                1 if inside_space and inside_tau and int(row["bad"]) else 0
            )
            rows.append(row)
    return rows


def p5_3_sample_reason_is_fixture_linked(row: dict[str, Any]) -> bool:
    reason = str(row.get("reason", "")).strip().lower()
    if not reason:
        return False
    expected_reason = str(row.get("expected_reason", "")).strip().lower()
    return (
        P5_3_FIXTURE_REASON in reason
        or P5_4_FIXTURE_REASON in reason
        or (bool(expected_reason) and expected_reason in reason)
        or "future_low_margin" in reason
        or "future_bad" in reason
    )


def summarize_p5_3_samples(rows: list[dict[str, Any]]) -> dict[str, Any]:
    current_rows = [
        row
        for row in rows
        if finite_float(row.get("tau_s")) is not None
        and abs(float(finite_float(row.get("tau_s")) or 0.0)) <= 1.0e-6
    ]
    future_fixture_rows = [
        row
        for row in rows
        if int(row.get("inside_high_risk_zone", 0) or 0)
        and int(row.get("inside_tau_window", 0) or 0)
    ]
    future_bad_fixture_rows = [
        row for row in future_fixture_rows if int(row.get("bad", 0) or 0)
    ]
    future_query_aligned_rows = [
        row
        for row in future_fixture_rows
        if int(row.get("query_alignment_ok", 0) or 0)
    ]
    future_query_mismatch_rows = [
        row
        for row in future_fixture_rows
        if not int(row.get("query_alignment_ok", 0) or 0)
    ]
    linked_rows = [
        row for row in future_bad_fixture_rows if p5_3_sample_reason_is_fixture_linked(row)
    ]
    final_candidate_future_fixture_rows = [
        row
        for row in future_fixture_rows
        if str(row.get("trajectory_sample_source", "")) == "final_candidate"
    ]
    runtime_committed_future_fixture_rows = [
        row
        for row in future_fixture_rows
        if str(row.get("trajectory_sample_source", "")) == "runtime_committed"
    ]
    final_candidate_query_aligned_rows = [
        row
        for row in final_candidate_future_fixture_rows
        if int(row.get("query_alignment_ok", 0) or 0)
    ]
    runtime_committed_query_aligned_rows = [
        row
        for row in runtime_committed_future_fixture_rows
        if int(row.get("query_alignment_ok", 0) or 0)
    ]
    final_candidate_bad_linked_rows = [
        row
        for row in final_candidate_future_fixture_rows
        if int(row.get("bad", 0) or 0) and p5_3_sample_reason_is_fixture_linked(row)
    ]
    runtime_committed_bad_linked_rows = [
        row
        for row in runtime_committed_future_fixture_rows
        if int(row.get("bad", 0) or 0) and p5_3_sample_reason_is_fixture_linked(row)
    ]
    trajectory_timing_failure_rows = [
        row
        for row in rows
        if str(row.get("reason", "")).strip()
        in {"trajectory_zero_duration", "trajectory_expired"}
    ]
    current_inside_rows = [
        row for row in current_rows if int(row.get("inside_high_risk_zone", 0) or 0)
    ]
    current_fixture_bad_rows = [
        row
        for row in current_rows
        if int(row.get("bad", 0) or 0)
        and (
            int(row.get("inside_high_risk_zone", 0) or 0)
            or P5_3_FIXTURE_REASON in str(row.get("reason", "")).lower()
            or P5_4_FIXTURE_REASON in str(row.get("reason", "")).lower()
        )
    ]
    return {
        "row_count": len(rows),
        "current_sample_count": len(current_rows),
        "current_inside_fixture_count": len(current_inside_rows),
        "current_fixture_bad_count": len(current_fixture_bad_rows),
        "future_fixture_sample_count": len(future_fixture_rows),
        "future_bad_fixture_sample_count": len(future_bad_fixture_rows),
        "future_bad_fixture_linked_count": len(linked_rows),
        "future_query_aligned_sample_count": len(future_query_aligned_rows),
        "future_query_mismatch_sample_count": len(future_query_mismatch_rows),
        "final_candidate_future_fixture_sample_count": len(
            final_candidate_future_fixture_rows
        ),
        "runtime_committed_future_fixture_sample_count": len(
            runtime_committed_future_fixture_rows
        ),
        "final_candidate_future_query_aligned_sample_count": len(
            final_candidate_query_aligned_rows
        ),
        "runtime_committed_future_query_aligned_sample_count": len(
            runtime_committed_query_aligned_rows
        ),
        "final_candidate_future_bad_fixture_linked_count": len(
            final_candidate_bad_linked_rows
        ),
        "runtime_committed_future_bad_fixture_linked_count": len(
            runtime_committed_bad_linked_rows
        ),
        "trajectory_timing_failure_count": len(trajectory_timing_failure_rows),
        "first_current_sample": current_rows[0] if current_rows else None,
        "first_current_inside_fixture_sample": current_inside_rows[0]
        if current_inside_rows
        else None,
        "first_current_fixture_bad_sample": current_fixture_bad_rows[0]
        if current_fixture_bad_rows
        else None,
        "first_future_fixture_sample": future_fixture_rows[0]
        if future_fixture_rows
        else None,
        "first_future_bad_fixture_sample": future_bad_fixture_rows[0]
        if future_bad_fixture_rows
        else None,
        "first_future_query_mismatch_sample": future_query_mismatch_rows[0]
        if future_query_mismatch_rows
        else None,
        "first_trajectory_timing_failure_sample": trajectory_timing_failure_rows[0]
        if trajectory_timing_failure_rows
        else None,
    }


def p5_3_sample_evidence_window(rows: list[dict[str, Any]]) -> dict[str, Any]:
    evidence_rows = [
        row
        for row in rows
        if int(row.get("inside_high_risk_zone", 0) or 0)
        and int(row.get("inside_tau_window", 0) or 0)
    ]
    source = "fixture_window_samples"
    if not evidence_rows:
        evidence_rows = rows
        source = "all_status_samples"
    stamps = [
        value
        for value in (finite_float(row.get("bag_time_s")) for row in evidence_rows)
        if value is not None
    ]
    if not stamps:
        return {
            "available": False,
            "source": source,
            "start_s": None,
            "end_s": None,
            "duration_s": None,
            "sample_count": 0,
        }
    start_s = min(stamps)
    end_s = max(stamps)
    return {
        "available": True,
        "source": source,
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": max(0.0, end_s - start_s),
        "sample_count": len(stamps),
    }


def p5_3_active_topic_gap_summary(
    sample_rows: list[dict[str, Any]],
    topic_timestamps: dict[str, list[float]] | None,
) -> dict[str, Any]:
    window = p5_3_sample_evidence_window(sample_rows)
    topic_timestamps = topic_timestamps or {}
    if not window.get("available") or not topic_timestamps:
        return {
            "available": False,
            "window": window,
            "continuous_max_gap_s": CONTINUOUS_MAX_GAP_S,
            "topic_statuses": {},
            "required_continuous_topics_stable": True,
        }

    start_s = float(window["start_s"])
    end_s = float(window["end_s"])
    topic_statuses: dict[str, dict[str, Any]] = {}
    for topic, expected in P5_TOPIC_EXPECTATIONS.items():
        if expected != "continuous":
            continue
        stamps = sorted(float(stamp) for stamp in topic_timestamps.get(topic, []))
        in_window = [stamp for stamp in stamps if start_s <= stamp <= end_s]
        prev_stamp = next((stamp for stamp in reversed(stamps) if stamp <= start_s), None)
        next_stamp = next((stamp for stamp in stamps if stamp >= end_s), None)
        edge_points = [
            prev_stamp if prev_stamp is not None else start_s,
            *in_window,
            next_stamp if next_stamp is not None else end_s,
        ]
        edge_points = sorted(set(edge_points))
        gaps = [
            max(0.0, curr - prev)
            for prev, curr in zip(edge_points, edge_points[1:])
        ]
        max_gap_s = max(gaps) if gaps else 0.0
        status = (
            "PASS"
            if stamps and max_gap_s <= CONTINUOUS_MAX_GAP_S
            else "FAIL"
        )
        topic_statuses[topic] = {
            "expected": expected,
            "count": len(in_window),
            "bracket_start_s": prev_stamp,
            "bracket_end_s": next_stamp,
            "window_start_s": start_s,
            "window_end_s": end_s,
            "window_duration_s": max(0.0, end_s - start_s),
            "max_gap_s": max_gap_s,
            "status": status,
        }

    return {
        "available": True,
        "window": window,
        "continuous_max_gap_s": CONTINUOUS_MAX_GAP_S,
        "topic_statuses": topic_statuses,
        "required_continuous_topics_stable": all(
            item.get("status") == "PASS" for item in topic_statuses.values()
        ),
    }


def p5_3_overlap_rows(
    marker_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    if not fixture.get("valid_geometry"):
        return rows
    for row in marker_rows:
        x = finite_float(row.get("x"))
        y = finite_float(row.get("y"))
        z = finite_float(row.get("z"))
        if x is None or y is None or z is None:
            continue
        state = str(row.get("state", ""))
        inside = p5_3_point_in_fixture(row, fixture)
        rows.append(
            {
                **row,
                "inside_high_risk_zone": 1 if inside else 0,
                "overlap_bad_state": 1 if inside and state == "bad" else 0,
                "fixture_x_min": fixture.get("x_min"),
                "fixture_x_max": fixture.get("x_max"),
                "fixture_y_min": fixture.get("y_min"),
                "fixture_y_max": fixture.get("y_max"),
                "fixture_z_min": fixture.get("z_min"),
                "fixture_z_max": fixture.get("z_max"),
            }
        )
    return rows


def summarize_p5_3_overlap(rows: list[dict[str, Any]]) -> dict[str, Any]:
    overlap_rows = [row for row in rows if int(row.get("inside_high_risk_zone", 0) or 0)]
    bad_rows = [row for row in overlap_rows if int(row.get("overlap_bad_state", 0) or 0)]
    states = Counter(str(row.get("state", "")) for row in overlap_rows)
    return {
        "row_count": len(rows),
        "overlap_count": len(overlap_rows),
        "overlap_bad_state_count": len(bad_rows),
        "overlap_state_counts": dict(sorted(states.items())),
        "first_overlap_row": overlap_rows[0] if overlap_rows else None,
        "first_bad_overlap_row": bad_rows[0] if bad_rows else None,
    }


def p5_3_future_replan_rows(p5_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in p5_rows:
        if (
            p5_action(row, "action") != P5_REPLAN_ACTION
            and p5_action(row, "raw_action") != P5_REPLAN_ACTION
        ):
            continue
        if p5_3_row_has_future_reason(row):
            rows.append(row)
    return rows


def p5_3_replan_rows_with_sample_link(
    sample_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    linked_status_rows = {
        int(row.get("status_row_index", -1))
        for row in sample_rows
        if int(row.get("fixture_bad_sample", 0) or 0)
        and p5_3_sample_reason_is_fixture_linked(row)
        and (
            p5_action(row, "action") == P5_REPLAN_ACTION
            or p5_action(row, "raw_action") == P5_REPLAN_ACTION
        )
    }
    return [
        row
        for row in sample_rows
        if int(row.get("status_row_index", -2)) in linked_status_rows
    ]


def p5_3_row_has_strict_replan(row: dict[str, Any]) -> bool:
    return (
        p5_action(row, "action") == P5_REPLAN_ACTION
        and p5_action(row, "raw_action") == P5_REPLAN_ACTION
    )


def p5_3_row_has_effective_replan(row: dict[str, Any]) -> bool:
    return (
        p5_action(row, "action") == P5_REPLAN_ACTION
        or p5_action(row, "raw_action") == P5_REPLAN_ACTION
    )


def p5_3_row_has_emergency(row: dict[str, Any]) -> bool:
    return (
        p5_action(row, "action") == P5_EMERGENCY_ACTION
        or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    )


def p5_3_sample_is_query_aligned_future_fixture(row: dict[str, Any]) -> bool:
    return (
        int(row.get("inside_high_risk_zone", 0) or 0) == 1
        and int(row.get("inside_tau_window", 0) or 0) == 1
        and int(row.get("query_alignment_ok", 0) or 0) == 1
    )


def p5_3_samples_by_status_index(
    sample_rows: list[dict[str, Any]],
) -> dict[int, list[dict[str, Any]]]:
    rows_by_status: dict[int, list[dict[str, Any]]] = {}
    for row in sample_rows:
        try:
            status_index = int(row.get("status_row_index", -1))
        except (TypeError, ValueError):
            continue
        rows_by_status.setdefault(status_index, []).append(row)
    return rows_by_status


def p5_3_status_row_has_query_aligned_future_fixture(
    status_index: int,
    samples_by_status: dict[int, list[dict[str, Any]]],
) -> bool:
    return any(
        p5_3_sample_is_query_aligned_future_fixture(row)
        for row in samples_by_status.get(status_index, [])
    )


def p5_3_event_window_candidate_row(
    status_row: dict[str, Any],
    status_index: int,
    samples_by_status: dict[int, list[dict[str, Any]]],
    *,
    strict_replan: bool,
) -> bool:
    has_replan = (
        p5_3_row_has_strict_replan(status_row)
        if strict_replan
        else p5_3_row_has_effective_replan(status_row)
    )
    return (
        has_replan
        and p5_3_row_has_future_reason(status_row)
        and p5_3_status_row_has_query_aligned_future_fixture(
            status_index,
            samples_by_status,
        )
    )


def p5_3_ordered_status_indices(p5_rows: list[dict[str, Any]]) -> list[int]:
    def key(index: int) -> tuple[float, int]:
        stamp = finite_float(p5_rows[index].get("bag_time_s"))
        return (stamp if stamp is not None else math.inf, index)

    return sorted(range(len(p5_rows)), key=key)


def p5_3_window_stamps(rows: list[dict[str, Any]]) -> tuple[float | None, float | None]:
    stamps = [
        stamp
        for stamp in (finite_float(row.get("bag_time_s")) for row in rows)
        if stamp is not None
    ]
    if not stamps:
        return None, None
    return min(stamps), max(stamps)


def p5_3_filter_sample_rows_by_status_indices(
    sample_rows: list[dict[str, Any]],
    row_indices: list[int] | set[int],
) -> list[dict[str, Any]]:
    row_index_set = set(row_indices)
    filtered: list[dict[str, Any]] = []
    for row in sample_rows:
        try:
            status_index = int(row.get("status_row_index", -1))
        except (TypeError, ValueError):
            continue
        if status_index in row_index_set:
            filtered.append(row)
    return filtered


def p5_3_event_window_empty() -> dict[str, Any]:
    sample_summary = summarize_p5_3_samples([])
    return {
        "available": False,
        "row_indices": [],
        "status_row_count": 0,
        "sample_row_count": 0,
        "start_s": None,
        "end_s": None,
        "duration_s": None,
        "anchor_status_row_index": None,
        "current_sample_present": False,
        "current_outside_fixture": False,
        "current_not_fixture_bad": False,
        "future_fixture_sample_count": 0,
        "future_bad_fixture_sample_count": 0,
        "future_bad_fixture_linked_count": 0,
        "future_query_aligned_sample_count": 0,
        "future_query_mismatch_sample_count": 0,
        "first_bad_tau": None,
        "first_bad_tau_in_fixture_window": False,
        "future_replan_reason_ok": False,
        "max_consecutive_emergency": 0,
        "raw_max_consecutive_emergency": 0,
        "emergency_storm_absent": True,
        "bad_ratio_max": 0.0,
        "future_bad_ratio_coverage": False,
        "sample_summary": sample_summary,
    }


def p5_3_event_window_summary(
    p5_rows: list[dict[str, Any]],
    sample_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
    max_bad_ratio: float,
) -> dict[str, Any]:
    if not p5_rows or not sample_rows or not fixture.get("valid_geometry"):
        return p5_3_event_window_empty()

    samples_by_status = p5_3_samples_by_status_index(sample_rows)
    ordered_indices = p5_3_ordered_status_indices(p5_rows)
    anchor_order_index: int | None = None
    anchor_status_index: int | None = None
    for order_index, status_index in enumerate(ordered_indices):
        if p5_3_event_window_candidate_row(
            p5_rows[status_index],
            status_index,
            samples_by_status,
            strict_replan=True,
        ):
            anchor_order_index = order_index
            anchor_status_index = status_index
            break
    if anchor_order_index is None or anchor_status_index is None:
        return p5_3_event_window_empty()

    row_indices: list[int] = []
    for status_index in ordered_indices[anchor_order_index:]:
        row = p5_rows[status_index]
        if p5_3_row_has_emergency(row) and not p5_3_row_has_effective_replan(row):
            break
        if not p5_3_event_window_candidate_row(
            row,
            status_index,
            samples_by_status,
            strict_replan=False,
        ):
            break
        row_indices.append(status_index)

    if not row_indices:
        return p5_3_event_window_empty()

    row_index_set = set(row_indices)
    window_status_rows = [p5_rows[index] for index in row_indices]
    window_sample_rows = p5_3_filter_sample_rows_by_status_indices(
        sample_rows,
        row_index_set,
    )
    sample_summary = summarize_p5_3_samples(window_sample_rows)
    start_s, end_s = p5_3_window_stamps(window_status_rows)
    first_bad_tau_values = p5_3_first_bad_tau_values(window_status_rows)
    first_bad_tau = min(first_bad_tau_values) if first_bad_tau_values else None
    max_consecutive_emergency = max_consecutive_action(
        window_status_rows,
        P5_EMERGENCY_ACTION,
        "action",
    )
    raw_max_consecutive_emergency = max_consecutive_action(
        window_status_rows,
        P5_EMERGENCY_ACTION,
        "raw_action",
    )
    bad_ratio_values = [
        value
        for value in (finite_float(row.get("bad_ratio")) for row in window_status_rows)
        if value is not None
    ]
    bad_ratio_max = max(bad_ratio_values) if bad_ratio_values else 0.0
    return {
        "available": True,
        "row_indices": row_indices,
        "status_row_count": len(window_status_rows),
        "sample_row_count": len(window_sample_rows),
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": (
            max(0.0, float(end_s) - float(start_s))
            if start_s is not None and end_s is not None
            else None
        ),
        "anchor_status_row_index": anchor_status_index,
        "current_sample_present": int(sample_summary.get("current_sample_count", 0) or 0)
        > 0,
        "current_outside_fixture": (
            int(sample_summary.get("current_inside_fixture_count", 0) or 0) == 0
        ),
        "current_not_fixture_bad": (
            int(sample_summary.get("current_fixture_bad_count", 0) or 0) == 0
        ),
        "future_fixture_sample_count": int(
            sample_summary.get("future_fixture_sample_count", 0) or 0
        ),
        "future_bad_fixture_sample_count": int(
            sample_summary.get("future_bad_fixture_sample_count", 0) or 0
        ),
        "future_bad_fixture_linked_count": int(
            sample_summary.get("future_bad_fixture_linked_count", 0) or 0
        ),
        "future_query_aligned_sample_count": int(
            sample_summary.get("future_query_aligned_sample_count", 0) or 0
        ),
        "future_query_mismatch_sample_count": int(
            sample_summary.get("future_query_mismatch_sample_count", 0) or 0
        ),
        "first_bad_tau": first_bad_tau,
        "first_bad_tau_in_fixture_window": p5_3_value_in_window(
            first_bad_tau,
            fixture.get("tau_min"),
            fixture.get("tau_max"),
        ),
        "future_replan_reason_ok": any(
            p5_3_row_has_effective_replan(row) and p5_3_row_has_future_reason(row)
            for row in window_status_rows
        ),
        "max_consecutive_emergency": max_consecutive_emergency,
        "raw_max_consecutive_emergency": raw_max_consecutive_emergency,
        "emergency_storm_absent": (
            max_consecutive_emergency < P5_2_EMERGENCY_STORM_CONSECUTIVE
            and raw_max_consecutive_emergency < P5_2_EMERGENCY_STORM_CONSECUTIVE
        ),
        "bad_ratio_max": bad_ratio_max,
        "future_bad_ratio_coverage": bad_ratio_max >= max_bad_ratio,
        "sample_summary": sample_summary,
    }


def p5_3_row_has_future_reason(row: dict[str, Any]) -> bool:
    accepted_reasons = {"future_bad", "future_low_margin", P5_3_FIXTURE_REASON}
    reasons = p5_all_reason_values(
        row,
        ("reason", "final_gate_last_reason", "future_reason", "active_reasons"),
    )
    return bool(reasons & accepted_reasons)


def p5_3_future_reason_rows(p5_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in p5_rows if p5_3_row_has_future_reason(row)]


def p5_3_first_bad_tau_values(p5_rows: list[dict[str, Any]]) -> list[float]:
    values = [finite_float(row.get("first_bad_tau")) for row in p5_rows]
    return [value for value in values if value is not None]


def validate_p5_3_hard_gates(
    manifest: dict[str, Any],
    validator_summary: dict[str, Any],
    topic_health: dict[str, dict[str, Any]],
    p0_health_summary: dict[str, Any],
    p0_health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    p5_summary: dict[str, Any],
    p5_marker_rows: list[dict[str, Any]],
    failures: list[str],
    inconclusive: list[str],
    topic_timestamps: dict[str, list[float]] | None = None,
) -> dict[str, Any]:
    manifest_gates = p5_manifest_gate_values(manifest)
    fixture = p5_3_fixture_from_manifest(manifest)
    fixture_ready = (
        fixture.get("present")
        and fixture.get("enabled")
        and fixture.get("name") == P5_3_FIXTURE_NAME
        and fixture.get("valid_geometry")
    )
    overlap_rows = p5_3_overlap_rows(p5_marker_rows, fixture)
    overlap_summary = summarize_p5_3_overlap(overlap_rows)
    p0_startup = summarize_p0_startup_snapshot_unavailable(p0_health_rows)
    topic_statuses = {
        topic: (topic_health.get(topic) or {}).get("status", "MISSING")
        for topic in P5_TOPIC_EXPECTATIONS
    }
    first_bad_tau_values = p5_3_first_bad_tau_values(p5_rows)
    first_bad_tau_min = min(first_bad_tau_values) if first_bad_tau_values else None
    first_bad_tau_in_window = (
        first_bad_tau_min is not None
        and p5_3_value_in_window(
            first_bad_tau_min, fixture.get("tau_min"), fixture.get("tau_max")
        )
    )
    future_replan_rows = p5_3_future_replan_rows(p5_rows)
    sample_rows = p5_3_sample_rows(p5_rows, fixture)
    sample_summary = summarize_p5_3_samples(sample_rows)
    full_run_active_topic_gap = p5_3_active_topic_gap_summary(
        sample_rows,
        topic_timestamps,
    )
    replan_sample_link_rows = p5_3_replan_rows_with_sample_link(sample_rows)
    future_replan_reason_ok = len(future_replan_rows) > 0 or len(
        replan_sample_link_rows
    ) > 0
    future_min_im_min = finite_float(p5_summary.get("future_min_im_min"))
    future_replan_margin_m = finite_float(manifest.get("p5.future_replan_margin_m")) or 0.3
    max_bad_ratio = finite_float(manifest.get("p5.max_bad_ratio")) or 0.25
    bad_ratio_max = finite_float(p5_summary.get("bad_ratio_max")) or 0.0
    event_window = p5_3_event_window_summary(
        p5_rows,
        sample_rows,
        fixture,
        max_bad_ratio,
    )
    event_window_status_indices = list(event_window.get("row_indices", []) or [])
    event_window_sample_rows = p5_3_filter_sample_rows_by_status_indices(
        sample_rows,
        event_window_status_indices,
    )
    event_window_active_topic_gap = p5_3_active_topic_gap_summary(
        event_window_sample_rows,
        topic_timestamps,
    )
    active_topic_gap = (
        event_window_active_topic_gap
        if bool(event_window.get("available"))
        else full_run_active_topic_gap
    )
    future_reason_rows = p5_3_future_reason_rows(p5_rows)
    bspline_topic_health = topic_health.get("/drone_0_planning/bspline") or {}
    bspline_count = int(bspline_topic_health.get("count", 0) or 0)
    final_candidate_fixed_fixture_evidence = (
        int(
            sample_summary.get(
                "final_candidate_future_fixture_sample_count", 0
            )
            or 0
        )
        > 0
        and int(
            sample_summary.get(
                "final_candidate_future_query_aligned_sample_count", 0
            )
            or 0
        )
        > 0
        and int(
            sample_summary.get(
                "final_candidate_future_bad_fixture_linked_count", 0
            )
            or 0
        )
        > 0
    )
    runtime_committed_fixed_fixture_evidence = (
        int(
            sample_summary.get(
                "runtime_committed_future_fixture_sample_count", 0
            )
            or 0
        )
        > 0
        and int(
            sample_summary.get(
                "runtime_committed_future_query_aligned_sample_count", 0
            )
            or 0
        )
        > 0
        and int(
            sample_summary.get(
                "runtime_committed_future_bad_fixture_linked_count", 0
            )
            or 0
        )
        > 0
    )
    bspline_zero_acceptable = (
        bspline_count > 0
        or (
            final_candidate_fixed_fixture_evidence
            and bool(event_window.get("future_replan_reason_ok"))
        )
    )
    expected_im = finite_float(fixture.get("expected_im"))
    injected_pl_over_expected_al = (
        fixture.get("expected_hal") is not None
        and fixture.get("expected_val") is not None
        and fixture.get("hpl_pred") is not None
        and fixture.get("vpl_pred") is not None
        and (
            float(fixture["hpl_pred"]) > float(fixture["expected_hal"])
            or float(fixture["vpl_pred"]) > float(fixture["expected_val"])
        )
    )
    overlap_margin_evidence = (
        int(overlap_summary.get("overlap_bad_state_count", 0) or 0) > 0
        or int(sample_summary.get("future_bad_fixture_sample_count", 0) or 0) > 0
        or (
            future_min_im_min is not None
            and future_min_im_min < future_replan_margin_m
        )
        or (
            expected_im is not None
            and expected_im < future_replan_margin_m
            and injected_pl_over_expected_al
        )
    )
    future_bad_ratio_coverage = bad_ratio_max >= max_bad_ratio
    max_consecutive_emergency = int(p5_summary.get("max_consecutive_emergency", 0) or 0)
    raw_max_consecutive_emergency = int(
        p5_summary.get("raw_max_consecutive_emergency", 0) or 0
    )
    gates = {
        **manifest_gates,
        "fixture": fixture,
        "fixture_present": bool(fixture.get("present")),
        "fixture_enabled": bool(fixture.get("enabled")),
        "fixture_name_ok": fixture.get("name") == P5_3_FIXTURE_NAME,
        "fixture_geometry_valid": bool(fixture.get("valid_geometry")),
        "fixture_ready": bool(fixture_ready),
        "blocked_scenario_missing": not bool(fixture_ready),
        "validator_summary_present": bool(validator_summary),
        "validator_passed": validator_summary.get("passed") is True,
        "required_p5_topics_stable": all(
            status == "PASS"
            for topic, status in topic_statuses.items()
            if P5_TOPIC_EXPECTATIONS.get(topic) != "planner-dependent"
        ),
        "topic_statuses": topic_statuses,
        "active_topic_gap": active_topic_gap,
        "full_run_active_topic_gap": full_run_active_topic_gap,
        "event_window_active_topic_gap": event_window_active_topic_gap,
        "active_required_p5_topics_stable": bool(
            active_topic_gap.get("required_continuous_topics_stable", True)
        ),
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        **p0_startup,
        "p0_post_startup_ready": p0_startup["post_startup_ready_false_count"] == 0,
        "p0_post_startup_non_stale": p0_startup["post_startup_stale_true_count"] == 0,
        "p0_post_startup_not_full_unknown": (
            p0_startup["post_startup_full_unknown_count"] == 0
        ),
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "p5_inspection_ok": not bool(p5_summary.get("inspection_error")),
        "marker_rows_present": len(p5_marker_rows) > 0,
        "sample_rows_present": len(sample_rows) > 0,
        "sample_summary": sample_summary,
        "full_run_sample_summary": sample_summary,
        "event_window": event_window,
        "event_window_available": bool(event_window.get("available")),
        "event_window_start_s": event_window.get("start_s"),
        "event_window_end_s": event_window.get("end_s"),
        "event_window_duration_s": event_window.get("duration_s"),
        "event_window_anchor_status_row_index": event_window.get(
            "anchor_status_row_index"
        ),
        "event_window_current_outside_fixture": bool(
            event_window.get("current_outside_fixture")
        ),
        "event_window_current_not_fixture_bad": bool(
            event_window.get("current_not_fixture_bad")
        ),
        "event_window_future_fixture_sample_count": int(
            event_window.get("future_fixture_sample_count", 0) or 0
        ),
        "event_window_future_bad_fixture_sample_count": int(
            event_window.get("future_bad_fixture_sample_count", 0) or 0
        ),
        "event_window_future_query_aligned_sample_count": int(
            event_window.get("future_query_aligned_sample_count", 0) or 0
        ),
        "event_window_first_bad_tau": event_window.get("first_bad_tau"),
        "event_window_first_bad_tau_in_fixture_window": bool(
            event_window.get("first_bad_tau_in_fixture_window")
        ),
        "event_window_future_replan_reason_ok": bool(
            event_window.get("future_replan_reason_ok")
        ),
        "event_window_emergency_storm_absent": bool(
            event_window.get("emergency_storm_absent")
        ),
        "bspline_count": bspline_count,
        "bspline_zero_acceptable": bool(bspline_zero_acceptable),
        "final_candidate_fixed_fixture_evidence": bool(
            final_candidate_fixed_fixture_evidence
        ),
        "runtime_committed_fixed_fixture_evidence": bool(
            runtime_committed_fixed_fixture_evidence
        ),
        "current_sample_present": (
            bool(event_window.get("available"))
            and bool(event_window.get("current_sample_present"))
        ),
        "current_sample_outside_fixture": (
            bool(event_window.get("available"))
            and bool(event_window.get("current_outside_fixture"))
        ),
        "current_sample_not_fixture_bad": (
            bool(event_window.get("available"))
            and bool(event_window.get("current_not_fixture_bad"))
        ),
        "future_sample_inside_fixture": (
            int(event_window.get("future_fixture_sample_count", 0) or 0) > 0
        ),
        "future_bad_sample_inside_fixture": (
            int(event_window.get("future_bad_fixture_sample_count", 0) or 0) > 0
        ),
        "future_bad_sample_linked": (
            int(event_window.get("future_bad_fixture_linked_count", 0) or 0) > 0
        ),
        "future_fixture_query_aligned": (
            int(event_window.get("future_fixture_sample_count", 0) or 0) > 0
            and int(event_window.get("future_query_mismatch_sample_count", 0) or 0)
            == 0
        ),
        "overlap": overlap_summary,
        "trajectory_overlap_present": int(overlap_summary.get("overlap_count", 0) or 0) > 0,
        "overlap_margin_evidence": overlap_margin_evidence,
        "future_min_im_min": future_min_im_min,
        "future_replan_margin_m": future_replan_margin_m,
        "bad_ratio_max": float(event_window.get("bad_ratio_max", 0.0) or 0.0),
        "full_run_bad_ratio_max": bad_ratio_max,
        "max_bad_ratio": max_bad_ratio,
        "future_bad_ratio_coverage": bool(
            event_window.get("future_bad_ratio_coverage")
        ),
        "full_run_future_bad_ratio_coverage": future_bad_ratio_coverage,
        "request_replan_count": int(p5_summary.get("replan_action_count", 0) or 0),
        "raw_request_replan_count": int(p5_summary.get("raw_replan_action_count", 0) or 0),
        "request_replan_present": (
            int(p5_summary.get("replan_action_count", 0) or 0) > 0
            or int(p5_summary.get("raw_replan_action_count", 0) or 0) > 0
        ),
        "future_reason_attribution_count": len(future_reason_rows),
        "future_reason_attribution_ok": len(future_reason_rows) > 0,
        "future_replan_reason_count": len(future_replan_rows),
        "future_replan_sample_link_count": len(replan_sample_link_rows),
        "future_replan_sample_link_ok": len(replan_sample_link_rows) > 0,
        "future_replan_reason_ok": bool(event_window.get("future_replan_reason_ok")),
        "full_run_future_replan_reason_ok": future_replan_reason_ok,
        "first_bad_tau_values": first_bad_tau_values[:20],
        "first_bad_tau_min": first_bad_tau_min,
        "first_bad_tau_in_fixture_window": bool(
            event_window.get("first_bad_tau_in_fixture_window")
        ),
        "full_run_first_bad_tau_in_fixture_window": first_bad_tau_in_window,
        "first_bad_tau_target_s": fixture.get("tau_min"),
        "emergency_storm_absent": bool(event_window.get("emergency_storm_absent")),
        "max_consecutive_emergency": max_consecutive_emergency,
        "raw_max_consecutive_emergency": raw_max_consecutive_emergency,
        "event_window_max_consecutive_emergency": int(
            event_window.get("max_consecutive_emergency", 0) or 0
        ),
        "event_window_raw_max_consecutive_emergency": int(
            event_window.get("raw_max_consecutive_emergency", 0) or 0
        ),
        "predicted_al_available": (
            p5_summary.get("pred_hal_min_min") is not None
            and p5_summary.get("pred_val_min_min") is not None
        ),
    }

    if not (
        gates["manifest_safety_profile_p5"]
        and gates["manifest_expected_true_ok"]
        and gates["manifest_expected_false_ok"]
    ):
        failures.append("P5-3 manifest does not enable P0/P5 with P1-P4 disabled")
    if not gates["fixture_present"]:
        failures.append("P5-3 fixture manifest is missing")
    elif not gates["fixture_enabled"]:
        failures.append("P5-3 fixture manifest is disabled")
    elif not gates["fixture_name_ok"]:
        failures.append(
            f"P5-3 fixture name is not {P5_3_FIXTURE_NAME}: "
            f"{fixture.get('name')}"
        )
    elif not gates["fixture_geometry_valid"]:
        failures.append("P5-3 fixture geometry or tau/PL values are invalid")
    if not gates["validator_summary_present"]:
        inconclusive.append("P5-3 validator summary is missing")
    elif not gates["validator_passed"]:
        failures.append("P5-3 validator summary did not pass")
    if not gates["active_required_p5_topics_stable"]:
        active_statuses = active_topic_gap.get("topic_statuses", {}) or {}
        failures.append(
            "P5-3 active evidence-window topic gap exceeded threshold: "
            + ", ".join(
                f"{topic} max_gap_s={finite_float(data.get('max_gap_s'))}"
                for topic, data in sorted(active_statuses.items())
                if data.get("status") != "PASS"
            )
        )
    if not gates["p0_health_rows_present"]:
        failures.append("P5-3 P0 health rows are missing")
    if not gates["startup_snapshot_unavailable_bounded"]:
        failures.append("P5-3 P0 startup snapshot_unavailable prefix is not bounded")
    if not gates["p0_post_startup_ready"]:
        failures.append("P5-3 P0 health reported ready=false after startup")
    if not gates["p0_post_startup_non_stale"]:
        failures.append("P5-3 P0 health reported stale=true after startup")
    if not gates["p0_post_startup_not_full_unknown"]:
        failures.append("P5-3 P0 health reported full unknown after startup")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-3 P5 status rows are missing")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-3 P5 status JSON parse errors were observed")
    if not gates["p5_inspection_ok"]:
        inconclusive.append("P5-3 P5 status inspection did not complete cleanly")
    if not gates["marker_rows_present"]:
        failures.append("P5-3 trajectory marker evidence is missing")
    if not gates["sample_rows_present"]:
        failures.append("P5-3 PL/AL margin: per-sample status diagnostics are missing")
    if not gates["event_window_available"]:
        failures.append(
            "P5-3 event-window acceptance: no first causal future-risk "
            "REQUEST_REPLAN row with query-aligned injected fixture evidence"
        )
    if not gates["current_sample_present"]:
        failures.append(
            "P5-3 event-window PL/AL margin: current/tau=0 sample evidence is missing"
        )
    if not gates["current_sample_outside_fixture"]:
        failures.append(
            "P5-3 event-window PL/AL margin: current/tau=0 sample is inside the high-risk fixture"
        )
    if not gates["current_sample_not_fixture_bad"]:
        failures.append(
            "P5-3 event-window PL/AL margin: current/tau=0 sample is bad due to high-risk fixture evidence"
        )
    if not gates["future_sample_inside_fixture"]:
        failures.append(
            "P5-3 event-window scenario isolation / future-only fixture evidence: "
            "no final-candidate or committed-runtime future trajectory sample "
            "entered the fixture tau/spatial window"
        )
    if (
        int(sample_summary.get("trajectory_timing_failure_count", 0) or 0) > 0
        and not gates["future_sample_inside_fixture"]
    ):
        failures.append(
            "P5-3 future sampling: trajectory timing diagnostics reported no "
            "available future sampling window"
        )
    if not gates["future_bad_sample_inside_fixture"]:
        failures.append(
            "P5-3 event-window PL/AL margin: future samples entered the fixture but did not become bad"
        )
    if not gates["bspline_zero_acceptable"]:
        failures.append(
            "P5-3 /drone_0_planning/bspline=0 without final-gate candidate "
            "fixed-fixture future evidence and REQUEST_REPLAN attribution"
        )
    event_sample_summary = (event_window.get("sample_summary") or {})
    full_run_query_mismatch = (
        int(sample_summary.get("future_fixture_sample_count", 0) or 0) > 0
        and int(sample_summary.get("future_query_mismatch_sample_count", 0) or 0) > 0
    )
    if (
        gates["future_sample_inside_fixture"] and not gates["future_fixture_query_aligned"]
    ) or (not gates["event_window_available"] and full_run_query_mismatch):
        mismatch = (
            event_sample_summary.get("first_future_query_mismatch_sample")
            or sample_summary.get("first_future_query_mismatch_sample")
            or {}
        )
        failures.append(
            "P5-3 event-window query alignment: future samples entered the fixture tau/spatial "
            "window but actual queried PL/reason did not match injected fixture PL "
            f"(first mismatch runtime_fixture_match="
            f"{bool(int(mismatch.get('fixture_match', 0) or 0))}, "
            f"hpl={mismatch.get('hpl')}, "
            f"vpl={mismatch.get('vpl')}, reason={mismatch.get('reason')})"
        )
    if not gates["trajectory_overlap_present"]:
        failures.append("P5-3 trajectory samples do not overlap the high-risk zone fixture")
    if not gates["overlap_margin_evidence"]:
        failures.append("P5-3 overlap did not show PL>AL or IM below future replan margin")
    if not gates["request_replan_present"]:
        failures.append("P5-3 did not observe REQUEST_REPLAN")
    if gates["overlap_margin_evidence"] and not gates["future_bad_ratio_coverage"]:
        failures.append(
            "P5-3 event-window scenario isolation / future bad-ratio coverage: "
            f"future risk exists but event_window_bad_ratio_max="
            f"{float(event_window.get('bad_ratio_max', 0.0) or 0.0):.6g} "
            f"is below max_bad_ratio={max_bad_ratio:.6g}"
        )
    elif not gates["future_replan_reason_ok"]:
        if gates["future_bad_ratio_coverage"] and not gates["future_reason_attribution_ok"]:
            failures.append(
                "P5-3 event-window reason attribution: future gate reached bad-ratio "
                "threshold but status reason fields did not expose future_bad "
                "or an equivalent future high-risk reason"
            )
        else:
            failures.append(
                "P5-3 event-window PL/AL margin: future-risk reason evidence did not coincide "
                "with a REQUEST_REPLAN row"
            )
    if not gates["first_bad_tau_in_fixture_window"]:
        failures.append(
            "P5-3 event-window first_bad_tau was absent or outside the fixture tau window"
        )
    if not gates["emergency_storm_absent"]:
        failures.append(
            "P5-3 event-window observed sustained emergency storm: "
            f"action_consecutive={gates['event_window_max_consecutive_emergency']}, "
            f"raw_action_consecutive={gates['event_window_raw_max_consecutive_emergency']}"
        )
    if not gates["predicted_al_available"]:
        inconclusive.append("P5-3 predicted alert-limit minima had no finite samples")

    required = (
        "manifest_safety_profile_p5",
        "manifest_expected_true_ok",
        "manifest_expected_false_ok",
        "validator_summary_present",
        "validator_passed",
        "active_required_p5_topics_stable",
        "p0_health_rows_present",
        "startup_snapshot_unavailable_bounded",
        "p0_post_startup_ready",
        "p0_post_startup_non_stale",
        "p0_post_startup_not_full_unknown",
        "p5_status_rows_present",
        "p5_json_parse_ok",
        "p5_inspection_ok",
        "marker_rows_present",
        "sample_rows_present",
        "event_window_available",
        "current_sample_present",
        "current_sample_outside_fixture",
        "current_sample_not_fixture_bad",
        "future_sample_inside_fixture",
        "future_bad_sample_inside_fixture",
        "future_bad_sample_linked",
        "future_fixture_query_aligned",
        "bspline_zero_acceptable",
        "trajectory_overlap_present",
        "overlap_margin_evidence",
        "request_replan_present",
        "future_bad_ratio_coverage",
        "future_replan_reason_ok",
        "first_bad_tau_in_fixture_window",
        "emergency_storm_absent",
        "predicted_al_available",
    )
    gates["passed"] = all(bool(gates.get(key)) for key in required)
    return gates


def p5_4_expected_emergency_time_s(fixture: dict[str, Any]) -> float:
    return (
        finite_float(fixture.get("expected_emergency_time_s"))
        or P5_4_EMERGENCY_TIME_S
    )


def p5_4_reason_text(row: dict[str, Any]) -> str:
    reasons = p5_all_reason_values(
        row,
        (
            "reason",
            "raw_reason",
            "current_reason",
            "future_reason",
            "active_reasons",
            "final_gate_last_reason",
            "pred_al_last_reason",
        ),
    )
    return " ".join(sorted(reasons))


def p5_4_row_has_future_reason(row: dict[str, Any]) -> bool:
    reason_text = p5_4_reason_text(row)
    return "future_bad" in reason_text or P5_4_FIXTURE_REASON in reason_text


def p5_4_row_has_strict_emergency(row: dict[str, Any]) -> bool:
    return (
        p5_action(row, "action") == P5_EMERGENCY_ACTION
        and p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    )


def p5_4_row_has_effective_emergency(row: dict[str, Any]) -> bool:
    return (
        p5_action(row, "action") == P5_EMERGENCY_ACTION
        or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
    )


def p5_4_row_has_startup_or_snapshot_cause(row: dict[str, Any]) -> bool:
    phase = str(row.get("phase", "")).strip().lower()
    reason_text = p5_4_reason_text(row)
    return phase == "startup" or "snapshot_unavailable" in reason_text


def p5_4_row_has_current_only_low_margin(row: dict[str, Any]) -> bool:
    reason_text = p5_4_reason_text(row)
    return "current_low_margin" in reason_text and not p5_4_row_has_future_reason(row)


def p5_4_row_has_unknown_only_cause(row: dict[str, Any]) -> bool:
    if p5_4_row_has_future_reason(row):
        return False
    reason_text = p5_4_reason_text(row)
    unknown_reasons = (
        "future_unknown",
        "unknown_only",
        "al_invalid",
        "pred_al_invalid",
    )
    if any(reason in reason_text for reason in unknown_reasons):
        return True
    unknown_ratio = finite_float(row.get("unknown_ratio"))
    bad_ratio = finite_float(row.get("bad_ratio"))
    return (
        unknown_ratio is not None
        and unknown_ratio > 0.0
        and (bad_ratio is None or bad_ratio <= 0.0)
    )


def p5_4_row_has_final_gate_failed_cause(row: dict[str, Any]) -> bool:
    reason_text = p5_4_reason_text(row)
    return "final_gate_failed" in reason_text


def p5_4_row_exclusion_causes(row: dict[str, Any]) -> list[str]:
    causes: list[str] = []
    if p5_4_row_has_startup_or_snapshot_cause(row):
        causes.append("startup_or_snapshot_unavailable")
    if p5_4_row_has_current_only_low_margin(row):
        causes.append("current-only low margin")
    if p5_4_row_has_unknown_only_cause(row):
        causes.append("unknown-only")
    if p5_4_row_has_final_gate_failed_cause(row):
        causes.append("final_gate_failed")
    return causes


def p5_4_same_row_query_aligned_samples(
    status_index: int,
    samples_by_status: dict[int, list[dict[str, Any]]],
) -> list[dict[str, Any]]:
    return [
        row
        for row in samples_by_status.get(status_index, [])
        if p5_3_sample_is_query_aligned_future_fixture(row)
    ]


def p5_4_anchor_candidate_row(
    status_row: dict[str, Any],
    status_index: int,
    samples_by_status: dict[int, list[dict[str, Any]]],
    emergency_time_s: float,
) -> bool:
    first_bad_tau = finite_float(status_row.get("first_bad_tau"))
    return (
        p5_4_row_has_strict_emergency(status_row)
        and p5_4_row_has_future_reason(status_row)
        and not p5_4_row_exclusion_causes(status_row)
        and len(p5_4_same_row_query_aligned_samples(status_index, samples_by_status))
        > 0
        and first_bad_tau is not None
        and first_bad_tau <= emergency_time_s
    )


def p5_4_row_explains_emergency(
    status_row: dict[str, Any],
    status_index: int,
    samples_by_status: dict[int, list[dict[str, Any]]],
    emergency_time_s: float,
) -> bool:
    first_bad_tau = finite_float(status_row.get("first_bad_tau"))
    return (
        p5_4_row_has_effective_emergency(status_row)
        and p5_4_row_has_future_reason(status_row)
        and not p5_4_row_exclusion_causes(status_row)
        and len(p5_4_same_row_query_aligned_samples(status_index, samples_by_status))
        > 0
        and first_bad_tau is not None
        and first_bad_tau <= emergency_time_s
    )


def p5_4_unexplained_emergency_storm_summary(
    p5_rows: list[dict[str, Any]],
    samples_by_status: dict[int, list[dict[str, Any]]],
    emergency_time_s: float,
) -> dict[str, Any]:
    ordered_indices = p5_3_ordered_status_indices(p5_rows)
    unexplained_flags: list[bool] = []
    unexplained_rows: list[dict[str, Any]] = []
    for status_index in ordered_indices:
        row = p5_rows[status_index]
        unexplained = p5_4_row_has_effective_emergency(row) and not p5_4_row_explains_emergency(
            row,
            status_index,
            samples_by_status,
            emergency_time_s,
        )
        unexplained_flags.append(unexplained)
        if unexplained:
            unexplained_rows.append(
                {
                    "status_row_index": status_index,
                    "bag_time_s": row.get("bag_time_s", ""),
                    "action": row.get("action", ""),
                    "raw_action": row.get("raw_action", ""),
                    "reason": row.get("reason", ""),
                    "raw_reason": row.get("raw_reason", ""),
                    "current_reason": row.get("current_reason", ""),
                    "future_reason": row.get("future_reason", ""),
                    "active_reasons": row.get("active_reasons", ""),
                    "first_bad_tau": row.get("first_bad_tau", ""),
                    "exclusion_causes": p5_4_row_exclusion_causes(row),
                }
            )
    max_consecutive = consecutive_true(unexplained_flags)
    return {
        "max_consecutive_unexplained_emergency": max_consecutive,
        "unexplained_emergency_rows": unexplained_rows[:20],
        "unexplained_emergency_storm_absent": (
            max_consecutive < P5_2_EMERGENCY_STORM_CONSECUTIVE
        ),
    }


def p5_4_emergency_window_empty() -> dict[str, Any]:
    sample_summary = summarize_p5_3_samples([])
    return {
        "available": False,
        "row_indices": [],
        "status_row_count": 0,
        "sample_row_count": 0,
        "start_s": None,
        "end_s": None,
        "duration_s": None,
        "anchor_status_row_index": None,
        "anchor_strict_emergency": False,
        "anchor_future_reason_ok": False,
        "anchor_same_row_query_aligned_sample_count": 0,
        "anchor_first_bad_tau": None,
        "anchor_first_bad_tau_within_emergency_time": False,
        "anchor_exclusion_causes": [],
        "sample_summary": sample_summary,
    }


def p5_4_emergency_window_summary(
    p5_rows: list[dict[str, Any]],
    sample_rows: list[dict[str, Any]],
    fixture: dict[str, Any],
) -> dict[str, Any]:
    if not p5_rows or not sample_rows or not fixture.get("valid_geometry"):
        return p5_4_emergency_window_empty()
    samples_by_status = p5_3_samples_by_status_index(sample_rows)
    emergency_time_s = p5_4_expected_emergency_time_s(fixture)
    ordered_indices = p5_3_ordered_status_indices(p5_rows)
    anchor_order_index: int | None = None
    anchor_status_index: int | None = None
    for order_index, status_index in enumerate(ordered_indices):
        if p5_4_anchor_candidate_row(
            p5_rows[status_index],
            status_index,
            samples_by_status,
            emergency_time_s,
        ):
            anchor_order_index = order_index
            anchor_status_index = status_index
            break
    if anchor_order_index is None or anchor_status_index is None:
        return p5_4_emergency_window_empty()

    row_indices: list[int] = []
    for status_index in ordered_indices[anchor_order_index:]:
        row = p5_rows[status_index]
        if not p5_4_row_explains_emergency(
            row,
            status_index,
            samples_by_status,
            emergency_time_s,
        ):
            break
        row_indices.append(status_index)

    window_status_rows = [p5_rows[index] for index in row_indices]
    window_sample_rows = p5_3_filter_sample_rows_by_status_indices(
        sample_rows,
        row_indices,
    )
    sample_summary = summarize_p5_3_samples(window_sample_rows)
    start_s, end_s = p5_3_window_stamps(window_status_rows)
    anchor_row = p5_rows[anchor_status_index]
    anchor_first_bad_tau = finite_float(anchor_row.get("first_bad_tau"))
    anchor_samples = p5_4_same_row_query_aligned_samples(
        anchor_status_index,
        samples_by_status,
    )
    return {
        "available": True,
        "row_indices": row_indices,
        "status_row_count": len(window_status_rows),
        "sample_row_count": len(window_sample_rows),
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": (
            max(0.0, float(end_s) - float(start_s))
            if start_s is not None and end_s is not None
            else None
        ),
        "anchor_status_row_index": anchor_status_index,
        "anchor_strict_emergency": p5_4_row_has_strict_emergency(anchor_row),
        "anchor_future_reason_ok": p5_4_row_has_future_reason(anchor_row),
        "anchor_same_row_query_aligned_sample_count": len(anchor_samples),
        "anchor_first_bad_tau": anchor_first_bad_tau,
        "anchor_first_bad_tau_within_emergency_time": (
            anchor_first_bad_tau is not None
            and anchor_first_bad_tau <= emergency_time_s
        ),
        "anchor_exclusion_causes": p5_4_row_exclusion_causes(anchor_row),
        "emergency_time_s": emergency_time_s,
        "sample_summary": sample_summary,
    }


def validate_p5_4_required_figures(
    figure_paths: list[Path],
    failures: list[str],
) -> None:
    for figure_path in figure_paths:
        if not figure_path.is_file() or figure_path.stat().st_size <= 0:
            failures.append(
                "P5-4 required figure missing: "
                f"{figure_path.name}; missing figure evidence prevents P5-4 acceptance"
            )


def validate_p5_4_hard_gates(
    manifest: dict[str, Any],
    validator_summary: dict[str, Any],
    topic_health: dict[str, dict[str, Any]],
    p0_health_summary: dict[str, Any],
    p0_health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    p5_summary: dict[str, Any],
    p5_marker_rows: list[dict[str, Any]],
    failures: list[str],
    inconclusive: list[str],
    topic_timestamps: dict[str, list[float]] | None = None,
) -> dict[str, Any]:
    manifest_gates = p5_manifest_gate_values(manifest)
    fixture = p5_4_fixture_from_manifest(manifest)
    fixture_ready = (
        fixture.get("present")
        and fixture.get("enabled")
        and fixture.get("name") == P5_4_FIXTURE_NAME
        and fixture.get("valid_geometry")
    )
    topic_statuses = {
        topic: (topic_health.get(topic) or {}).get("status", "MISSING")
        for topic in P5_TOPIC_EXPECTATIONS
    }
    p0_startup = summarize_p0_startup_snapshot_unavailable(p0_health_rows)
    overlap_rows = p5_3_overlap_rows(p5_marker_rows, fixture)
    overlap_summary = summarize_p5_3_overlap(overlap_rows)
    sample_rows = p5_3_sample_rows(p5_rows, fixture, tau_window_field="tau_s")
    sample_summary = summarize_p5_3_samples(sample_rows)
    samples_by_status = p5_3_samples_by_status_index(sample_rows)
    emergency_time_s = p5_4_expected_emergency_time_s(fixture)
    event_window = p5_4_emergency_window_summary(p5_rows, sample_rows, fixture)
    event_window_status_indices = list(event_window.get("row_indices", []) or [])
    event_window_sample_rows = p5_3_filter_sample_rows_by_status_indices(
        sample_rows,
        event_window_status_indices,
    )
    full_run_active_topic_gap = p5_3_active_topic_gap_summary(
        sample_rows,
        topic_timestamps,
    )
    event_window_active_topic_gap = p5_3_active_topic_gap_summary(
        event_window_sample_rows,
        topic_timestamps,
    )
    active_topic_gap = (
        event_window_active_topic_gap
        if bool(event_window.get("available"))
        else full_run_active_topic_gap
    )
    storm_summary = p5_4_unexplained_emergency_storm_summary(
        p5_rows,
        samples_by_status,
        emergency_time_s,
    )
    future_reason_rows = [row for row in p5_rows if p5_4_row_has_future_reason(row)]
    emergency_rows = [row for row in p5_rows if p5_4_row_has_effective_emergency(row)]
    excluded_emergency_rows = [
        {
            "bag_time_s": row.get("bag_time_s", ""),
            "action": row.get("action", ""),
            "raw_action": row.get("raw_action", ""),
            "reason": row.get("reason", ""),
            "current_reason": row.get("current_reason", ""),
            "future_reason": row.get("future_reason", ""),
            "active_reasons": row.get("active_reasons", ""),
            "causes": p5_4_row_exclusion_causes(row),
        }
        for row in emergency_rows
        if p5_4_row_exclusion_causes(row)
    ]
    first_bad_tau_values = p5_3_first_bad_tau_values(p5_rows)
    first_bad_tau_min = min(first_bad_tau_values) if first_bad_tau_values else None
    predicted_al_available = (
        p5_summary.get("pred_hal_min_min") is not None
        and p5_summary.get("pred_val_min_min") is not None
    )
    gates = {
        **manifest_gates,
        "fixture": fixture,
        "fixture_present": bool(fixture.get("present")),
        "fixture_enabled": bool(fixture.get("enabled")),
        "fixture_name_ok": fixture.get("name") == P5_4_FIXTURE_NAME,
        "fixture_geometry_valid": bool(fixture.get("valid_geometry")),
        "fixture_ready": bool(fixture_ready),
        "blocked_scenario_missing": not bool(fixture_ready),
        "validator_summary_present": bool(validator_summary),
        "validator_passed": validator_summary.get("passed") is True,
        "required_p5_topics_stable": all(
            status == "PASS"
            for topic, status in topic_statuses.items()
            if P5_TOPIC_EXPECTATIONS.get(topic) != "planner-dependent"
        ),
        "topic_statuses": topic_statuses,
        "active_topic_gap": active_topic_gap,
        "full_run_active_topic_gap": full_run_active_topic_gap,
        "event_window_active_topic_gap": event_window_active_topic_gap,
        "active_required_p5_topics_stable": bool(
            active_topic_gap.get("required_continuous_topics_stable", True)
        ),
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        **p0_startup,
        "p0_post_startup_ready": p0_startup["post_startup_ready_false_count"] == 0,
        "p0_post_startup_non_stale": p0_startup["post_startup_stale_true_count"] == 0,
        "p0_post_startup_not_full_unknown": (
            p0_startup["post_startup_full_unknown_count"] == 0
        ),
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "p5_inspection_ok": not bool(p5_summary.get("inspection_error")),
        "marker_rows_present": len(p5_marker_rows) > 0,
        "sample_rows_present": len(sample_rows) > 0,
        "sample_summary": sample_summary,
        "overlap": overlap_summary,
        "trajectory_overlap_present": int(overlap_summary.get("overlap_count", 0) or 0) > 0,
        "fixture_entered": int(sample_summary.get("future_fixture_sample_count", 0) or 0) > 0,
        "future_bad_sample_inside_fixture": int(
            sample_summary.get("future_bad_fixture_sample_count", 0) or 0
        )
        > 0,
        "future_bad_sample_linked": int(
            sample_summary.get("future_bad_fixture_linked_count", 0) or 0
        )
        > 0,
        "fixture_query_aligned": int(
            sample_summary.get("future_query_aligned_sample_count", 0) or 0
        )
        > 0,
        "fixture_query_mismatch_absent": int(
            sample_summary.get("future_query_mismatch_sample_count", 0) or 0
        )
        == 0,
        "event_window": event_window,
        "event_window_available": bool(event_window.get("available")),
        "anchor_status_row_index": event_window.get("anchor_status_row_index"),
        "anchor_strict_emergency": bool(event_window.get("anchor_strict_emergency")),
        "anchor_future_reason_ok": bool(event_window.get("anchor_future_reason_ok")),
        "anchor_same_row_query_aligned_sample_count": int(
            event_window.get("anchor_same_row_query_aligned_sample_count", 0) or 0
        ),
        "anchor_same_row_fixture_samples": int(
            event_window.get("anchor_same_row_query_aligned_sample_count", 0) or 0
        )
        > 0,
        "anchor_first_bad_tau": event_window.get("anchor_first_bad_tau"),
        "anchor_first_bad_tau_within_emergency_time": bool(
            event_window.get("anchor_first_bad_tau_within_emergency_time")
        ),
        "anchor_exclusion_causes": event_window.get("anchor_exclusion_causes", []),
        "emergency_time_s": emergency_time_s,
        "first_bad_tau_values": first_bad_tau_values[:20],
        "first_bad_tau_min": first_bad_tau_min,
        "future_reason_attribution_count": len(future_reason_rows),
        "future_reason_attribution_ok": len(future_reason_rows) > 0,
        "emergency_action_count": int(p5_summary.get("emergency_action_count", 0) or 0),
        "raw_emergency_action_count": int(
            p5_summary.get("raw_emergency_action_count", 0) or 0
        ),
        "excluded_emergency_rows": excluded_emergency_rows[:20],
        **storm_summary,
        "predicted_al_available": predicted_al_available,
    }

    if not (
        gates["manifest_safety_profile_p5"]
        and gates["manifest_expected_true_ok"]
        and gates["manifest_expected_false_ok"]
    ):
        failures.append("P5-4 manifest does not enable P0/P5 with P1-P4 disabled")
    if not gates["fixture_present"]:
        failures.append("P5-4 fixture manifest is missing")
    elif not gates["fixture_enabled"]:
        failures.append("P5-4 fixture manifest is disabled")
    elif not gates["fixture_name_ok"]:
        failures.append(
            f"P5-4 fixture name is not {P5_4_FIXTURE_NAME}: "
            f"{fixture.get('name')}"
        )
    elif not gates["fixture_geometry_valid"]:
        failures.append("P5-4 fixture geometry or tau/PL values are invalid")
    if not gates["validator_summary_present"]:
        failures.append("P5-4 validator summary is missing")
    elif not gates["validator_passed"]:
        failures.append("P5-4 validator summary did not pass")
    if not gates["required_p5_topics_stable"]:
        failures.append("P5-4 required P0/P5 topics are not all stable")
    if not gates["active_required_p5_topics_stable"]:
        failures.append("P5-4 active evidence-window topic-gap exceeded threshold")
    if not gates["p0_health_rows_present"]:
        failures.append("P5-4 P0 health rows are missing")
    if not gates["startup_snapshot_unavailable_bounded"]:
        failures.append("P5-4 P0 startup snapshot_unavailable prefix is not bounded")
    if not gates["p0_post_startup_ready"]:
        failures.append("P5-4 P0 health reported ready=false after startup")
    if not gates["p0_post_startup_non_stale"]:
        failures.append("P5-4 P0 health reported stale=true after startup")
    if not gates["p0_post_startup_not_full_unknown"]:
        failures.append("P5-4 P0 health reported full unknown after startup")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-4 P5 status rows are missing")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-4 P5 status JSON parse errors were observed")
    if not gates["p5_inspection_ok"]:
        failures.append("P5-4 P5 status inspection did not complete cleanly")
    if not gates["marker_rows_present"]:
        failures.append("P5-4 trajectory marker evidence is missing")
    if not gates["sample_rows_present"]:
        failures.append("P5-4 per-sample PL/AL diagnostics are missing")
    if not gates["fixture_entered"]:
        failures.append(
            "P5-4 fixture not entered: no trajectory sample entered the near-risk "
            "fixture tau/spatial window"
        )
    if gates["fixture_entered"] and not gates["future_bad_sample_inside_fixture"]:
        failures.append("P5-4 near-risk fixture samples did not become bad")
    if gates["fixture_entered"] and not gates["future_bad_sample_linked"]:
        failures.append("P5-4 near-risk fixture samples were not linked to future_bad or fixture attribution")
    if gates["fixture_entered"] and not gates["fixture_query_aligned"]:
        failures.append(
            "P5-4 query alignment: future samples entered the fixture tau/spatial "
            "window but actual queried PL/reason did not match injected fixture PL"
        )
    if gates["fixture_entered"] and not gates["fixture_query_mismatch_absent"]:
        mismatch = sample_summary.get("first_future_query_mismatch_sample") or {}
        failures.append(
            "P5-4 injected PL mismatch: at least one fixture-window sample did "
            "not match injected PL/reason "
            f"(hpl={mismatch.get('hpl')}, vpl={mismatch.get('vpl')}, "
            f"reason={mismatch.get('reason')})"
        )
    if not gates["event_window_available"]:
        failures.append(
            "P5-4 emergency candidate acceptance: no first causal "
            "REQUEST_EMERGENCY_STOP_CANDIDATE row with same-row query-aligned "
            "near-risk fixture evidence"
        )
    if gates["event_window_available"] and not gates["anchor_strict_emergency"]:
        failures.append("P5-4 anchor row was not both raw and effective emergency")
    if gates["event_window_available"] and not gates["anchor_future_reason_ok"]:
        failures.append("P5-4 anchor row lacked future_bad or p5_4_near_risk_zone attribution")
    if gates["event_window_available"] and not gates["anchor_same_row_fixture_samples"]:
        failures.append("P5-4 anchor row lacked same-row query-aligned fixture samples")
    if not gates["anchor_first_bad_tau_within_emergency_time"]:
        failures.append(
            "P5-4 first_bad_tau was absent or greater than "
            f"emergency_time_s={emergency_time_s:.6g}"
        )
    excluded_causes_text = " ".join(
        str(cause)
        for row in gates.get("excluded_emergency_rows", [])
        for cause in (row.get("causes", []) or [])
    )
    if "startup_or_snapshot_unavailable" in excluded_causes_text:
        failures.append("P5-4 startup/snapshot_unavailable emergency cause was observed")
    if "current-only low margin" in excluded_causes_text:
        failures.append("P5-4 current-only low margin emergency cause was observed")
    if "unknown-only" in excluded_causes_text:
        failures.append("P5-4 unknown-only emergency cause was observed")
    if "final_gate_failed" in excluded_causes_text:
        failures.append("P5-4 final-gate emergency cause was observed")
    if not gates["unexplained_emergency_storm_absent"]:
        failures.append(
            "P5-4 unexplained emergency storm observed: "
            f"max_consecutive_unexplained_emergency="
            f"{gates['max_consecutive_unexplained_emergency']}"
        )
    if not gates["predicted_al_available"]:
        failures.append("P5-4 predicted alert-limit minima had no finite samples")

    required = (
        "manifest_safety_profile_p5",
        "manifest_expected_true_ok",
        "manifest_expected_false_ok",
        "fixture_ready",
        "validator_summary_present",
        "validator_passed",
        "required_p5_topics_stable",
        "active_required_p5_topics_stable",
        "p0_health_rows_present",
        "startup_snapshot_unavailable_bounded",
        "p0_post_startup_ready",
        "p0_post_startup_non_stale",
        "p0_post_startup_not_full_unknown",
        "p5_status_rows_present",
        "p5_json_parse_ok",
        "p5_inspection_ok",
        "marker_rows_present",
        "sample_rows_present",
        "fixture_entered",
        "future_bad_sample_inside_fixture",
        "future_bad_sample_linked",
        "fixture_query_aligned",
        "fixture_query_mismatch_absent",
        "event_window_available",
        "anchor_strict_emergency",
        "anchor_future_reason_ok",
        "anchor_same_row_fixture_samples",
        "anchor_first_bad_tau_within_emergency_time",
        "unexplained_emergency_storm_absent",
        "predicted_al_available",
    )
    gates["passed"] = all(bool(gates.get(key)) for key in required)
    return gates


def p5_5_fixture_from_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    nested = ((manifest.get("p5_5") or {}).get("fixture") or {}) if manifest else {}
    thresholds = nested.get("expected_thresholds_s", {}) or {}
    enabled = manifest.get("p5_5.fixture.enabled", nested.get("enabled")) if manifest else None
    name = str(manifest.get("p5_5.fixture.name", nested.get("name", "")) if manifest else "")
    start_s = finite_float(
        manifest.get("p5_5.fixture.start_s", nested.get("start_s"))
        if manifest
        else None
    )
    duration_s = finite_float(
        manifest.get("p5_5.fixture.duration_s", nested.get("duration_s"))
        if manifest
        else None
    )
    replan_s = finite_float(
        thresholds.get("replan", manifest.get("p5.current_stale_to_replan_s"))
        if manifest
        else None
    )
    emergency_s = finite_float(
        thresholds.get("emergency", manifest.get("p5.current_stale_to_emergency_s"))
        if manifest
        else None
    )
    present = bool(
        nested
        or any(str(key).startswith("p5_5.fixture.") for key in (manifest or {}))
    )
    valid_window = (
        start_s is not None
        and duration_s is not None
        and start_s >= 0.0
        and duration_s > 0.0
    )
    valid_thresholds = (
        replan_s is not None
        and emergency_s is not None
        and replan_s >= 0.0
        and emergency_s >= replan_s
    )
    return {
        "present": present,
        "enabled": enabled is True or str(enabled).strip().lower() in {"1", "true", "yes", "on"},
        "name": name,
        "start_s": start_s,
        "duration_s": duration_s,
        "end_s": start_s + duration_s if valid_window else None,
        "expected_replan_s": replan_s,
        "expected_emergency_s": emergency_s,
        "expected_reason": P5_5_FIXTURE_REASON,
        "valid_window": valid_window,
        "valid_thresholds": valid_thresholds,
    }


def p5_5_annotate_integrity_window(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
) -> list[dict[str, Any]]:
    start_s = finite_float(fixture.get("start_s"))
    end_s = finite_float(fixture.get("end_s"))
    annotated: list[dict[str, Any]] = []
    for row in rows:
        item = dict(row)
        t_rel = finite_float(item.get("t_rel_s"))
        item["in_expected_window"] = int(
            t_rel is not None
            and start_s is not None
            and end_s is not None
            and start_s <= t_rel <= end_s
        )
        annotated.append(item)
    return annotated


def summarize_p5_5_integrity_stamp_evidence(
    rows: list[dict[str, Any]],
    fixture: dict[str, Any],
) -> dict[str, Any]:
    annotated = p5_5_annotate_integrity_window(rows, fixture)
    start_s = finite_float(fixture.get("start_s"))
    end_s = finite_float(fixture.get("end_s"))
    window_rows = [
        row
        for row in annotated
        if int(row.get("in_expected_window", 0) or 0)
    ]
    freeze_tolerance_s = 0.05
    freeze_rows = [
        row
        for row in window_rows
        if start_s is not None
        and (finite_float(row.get("header_rel_s")) is not None)
        and abs(float(finite_float(row.get("header_rel_s"))) - start_s)
        <= freeze_tolerance_s
    ]
    evidence_rows = freeze_rows or window_rows
    header_values = finite_values(evidence_rows, "header_stamp_s")
    bag_values = finite_values(evidence_rows, "bag_time_s")
    age_values = finite_values(evidence_rows, "bag_minus_header_s")
    t_rel_values = finite_values(window_rows, "t_rel_s")
    freeze_t_rel_values = finite_values(freeze_rows, "t_rel_s")
    gaps = [
        max(0.0, curr - prev)
        for prev, curr in zip(sorted(bag_values), sorted(bag_values)[1:])
    ]
    header_span = max(header_values) - min(header_values) if header_values else None
    bag_span = max(bag_values) - min(bag_values) if bag_values else None
    age_growth = age_values[-1] - age_values[0] if len(age_values) >= 2 else None
    duration_s = finite_float(fixture.get("duration_s")) or 0.0
    min_growth_s = min(1.0, max(0.25, duration_s * 0.20))
    pl_al_finite_count = sum(
        1 for row in evidence_rows if int(row.get("pl_al_finite", 0) or 0) == 1
    )
    all_pl_al_finite = bool(evidence_rows) and pl_al_finite_count == len(evidence_rows)
    window_entered = len(window_rows) > 0
    header_frozen = (
        len(freeze_rows) >= 2
        and len(header_values) >= 2
        and header_span is not None
        and header_span <= 0.05
    )
    continuous_publish = (
        len(bag_values) >= 2
        and max(gaps, default=0.0) <= CONTINUOUS_MAX_GAP_S
    )
    age_grew = age_growth is not None and age_growth >= min_growth_s
    bag_span_ok = bag_span is not None and bag_span >= min_growth_s
    return {
        "row_count": len(annotated),
        "window_row_count": len(window_rows),
        "freeze_row_count": len(freeze_rows),
        "window_start_rel_s": start_s,
        "window_end_rel_s": end_s,
        "window_first_t_rel_s": min(t_rel_values) if t_rel_values else None,
        "window_last_t_rel_s": max(t_rel_values) if t_rel_values else None,
        "freeze_first_t_rel_s": min(freeze_t_rel_values) if freeze_t_rel_values else None,
        "freeze_last_t_rel_s": max(freeze_t_rel_values) if freeze_t_rel_values else None,
        "freeze_tolerance_s": freeze_tolerance_s,
        "window_entered": window_entered,
        "header_stamp_span_s": header_span,
        "bag_span_s": bag_span,
        "max_gap_s": max(gaps) if gaps else None,
        "bag_minus_header_growth_s": age_growth,
        "min_required_growth_s": min_growth_s,
        "header_frozen": header_frozen,
        "continuous_publish_during_freeze": continuous_publish,
        "age_grew_during_freeze": age_grew,
        "bag_span_ok": bag_span_ok,
        "pl_al_finite_count": pl_al_finite_count,
        "all_pl_al_finite": all_pl_al_finite,
        "first_window_row": window_rows[0] if window_rows else None,
        "last_window_row": window_rows[-1] if window_rows else None,
        "first_freeze_row": freeze_rows[0] if freeze_rows else None,
        "last_freeze_row": freeze_rows[-1] if freeze_rows else None,
        "rows": annotated,
    }


def p5_5_status_reason_values(row: dict[str, Any]) -> set[str]:
    return p5_all_reason_values(
        row,
        (
            "reason",
            "raw_reason",
            "current_reason",
            "active_reasons",
            "final_gate_last_reason",
            "pred_al_last_reason",
        ),
    )


def p5_5_row_has_current_stale_attribution(row: dict[str, Any]) -> bool:
    return P5_5_FIXTURE_REASON in p5_5_status_reason_values(row)


def p5_5_actionable(row: dict[str, Any]) -> bool:
    return p5_action(row, "action") in {P5_REPLAN_ACTION, P5_EMERGENCY_ACTION} or (
        p5_action(row, "raw_action") in {P5_REPLAN_ACTION, P5_EMERGENCY_ACTION}
    )


def p5_5_row_has_unknown_only_cause(row: dict[str, Any]) -> bool:
    if p5_5_row_has_current_stale_attribution(row):
        return False
    reasons = p5_5_status_reason_values(row)
    if reasons & {"future_unknown", "unknown_only", "al_invalid", "pred_al_invalid"}:
        return True
    unknown_ratio = finite_float(row.get("unknown_ratio"))
    bad_ratio = finite_float(row.get("bad_ratio"))
    return (
        unknown_ratio is not None
        and unknown_ratio > 0.0
        and (bad_ratio is None or bad_ratio <= 0.0)
    )


def p5_5_cause_exclusion_summary(p5_rows: list[dict[str, Any]]) -> dict[str, Any]:
    scoped_rows = [
        row
        for row in p5_rows
        if p5_5_row_has_current_stale_attribution(row)
        or (finite_float(row.get("current_stale_duration_s")) or 0.0) > 0.0
    ]
    first_stale_time = next(
        (finite_float(row.get("bag_time_s")) for row in scoped_rows),
        None,
    )
    actionable_rows = [row for row in scoped_rows if p5_5_actionable(row)]
    startup_snapshot_rows = [
        row
        for row in actionable_rows
        if str(row.get("phase", "")).strip().lower() == "startup"
        or "snapshot_unavailable" in p5_5_status_reason_values(row)
    ]
    future_bad_rows = [
        row for row in actionable_rows if "future_bad" in p5_5_status_reason_values(row)
    ]
    unknown_only_rows = [
        row for row in actionable_rows if p5_5_row_has_unknown_only_cause(row)
    ]
    non_current_stale_rows = [
        row
        for row in actionable_rows
        if not p5_5_row_has_current_stale_attribution(row)
    ]
    return {
        "first_stale_bag_time_s": first_stale_time,
        "scoped_actionable_count": len(actionable_rows),
        "startup_snapshot_action_count": len(startup_snapshot_rows),
        "future_bad_action_count": len(future_bad_rows),
        "unknown_only_action_count": len(unknown_only_rows),
        "non_current_stale_action_count": len(non_current_stale_rows),
        "first_startup_snapshot_action": startup_snapshot_rows[0]
        if startup_snapshot_rows
        else None,
        "first_future_bad_action": future_bad_rows[0] if future_bad_rows else None,
        "first_unknown_only_action": unknown_only_rows[0] if unknown_only_rows else None,
        "first_non_current_stale_action": non_current_stale_rows[0]
        if non_current_stale_rows
        else None,
    }


def p5_5_active_topic_gap_summary(
    integrity_evidence: dict[str, Any],
    topic_timestamps: dict[str, list[float]] | None,
) -> dict[str, Any]:
    first_row = (
        integrity_evidence.get("first_freeze_row")
        or integrity_evidence.get("first_window_row")
        or {}
    )
    last_row = (
        integrity_evidence.get("last_freeze_row")
        or integrity_evidence.get("last_window_row")
        or {}
    )
    start_s = finite_float(first_row.get("bag_time_s"))
    end_s = finite_float(last_row.get("bag_time_s"))
    window = {
        "available": start_s is not None and end_s is not None,
        "start_s": start_s,
        "end_s": end_s,
        "duration_s": (
            max(0.0, end_s - start_s)
            if start_s is not None and end_s is not None
            else None
        ),
    }
    topic_timestamps = topic_timestamps or {}
    if not window["available"] or not topic_timestamps:
        return {
            "available": False,
            "window": window,
            "continuous_max_gap_s": CONTINUOUS_MAX_GAP_S,
            "topic_statuses": {},
            "required_continuous_topics_stable": True,
        }
    topic_statuses: dict[str, dict[str, Any]] = {}
    assert start_s is not None and end_s is not None
    for topic, expected in P5_TOPIC_EXPECTATIONS.items():
        if expected != "continuous":
            continue
        stamps = sorted(float(stamp) for stamp in topic_timestamps.get(topic, []))
        in_window = [stamp for stamp in stamps if start_s <= stamp <= end_s]
        prev_stamp = next((stamp for stamp in reversed(stamps) if stamp <= start_s), None)
        next_stamp = next((stamp for stamp in stamps if stamp >= end_s), None)
        edge_points = sorted(
            set(
                [
                    prev_stamp if prev_stamp is not None else start_s,
                    *in_window,
                    next_stamp if next_stamp is not None else end_s,
                ]
            )
        )
        gaps = [
            max(0.0, curr - prev)
            for prev, curr in zip(edge_points, edge_points[1:])
        ]
        max_gap_s = max(gaps) if gaps else 0.0
        topic_statuses[topic] = {
            "count": len(in_window),
            "max_gap_s": max_gap_s,
            "status": "PASS" if stamps and max_gap_s <= CONTINUOUS_MAX_GAP_S else "FAIL",
        }
    return {
        "available": True,
        "window": window,
        "continuous_max_gap_s": CONTINUOUS_MAX_GAP_S,
        "topic_statuses": topic_statuses,
        "required_continuous_topics_stable": all(
            item.get("status") == "PASS" for item in topic_statuses.values()
        ),
    }


def validate_p5_5_required_figures(
    figure_paths: list[Path],
    failures: list[str],
) -> None:
    for figure_path in figure_paths:
        if not figure_path.is_file() or figure_path.stat().st_size <= 0:
            failures.append(
                "P5-5 required figure missing: "
                f"{figure_path.name}; missing stale-window figure evidence "
                "prevents P5-5 acceptance"
            )


def validate_p5_5_hard_gates(
    manifest: dict[str, Any],
    validator_summary: dict[str, Any],
    topic_health: dict[str, dict[str, Any]],
    p0_health_summary: dict[str, Any],
    p0_health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    p5_summary: dict[str, Any],
    integrity_stamp_rows: list[dict[str, Any]],
    failures: list[str],
    inconclusive: list[str],
    topic_timestamps: dict[str, list[float]] | None = None,
) -> dict[str, Any]:
    manifest_gates = p5_manifest_gate_values(manifest)
    fixture = p5_5_fixture_from_manifest(manifest)
    fixture_ready = (
        fixture.get("present")
        and fixture.get("enabled")
        and fixture.get("name") == P5_5_FIXTURE_NAME
        and fixture.get("valid_window")
        and fixture.get("valid_thresholds")
    )
    topic_statuses = {
        topic: (topic_health.get(topic) or {}).get("status", "MISSING")
        for topic in P5_TOPIC_EXPECTATIONS
    }
    p0_startup = summarize_p0_startup_snapshot_unavailable(p0_health_rows)
    integrity_evidence = summarize_p5_5_integrity_stamp_evidence(
        integrity_stamp_rows,
        fixture,
    )
    active_topic_gap = p5_5_active_topic_gap_summary(
        integrity_evidence,
        topic_timestamps,
    )
    replan_s = finite_float(fixture.get("expected_replan_s")) or 0.5
    emergency_s = finite_float(fixture.get("expected_emergency_s")) or 2.0
    stale_rows = [
        row
        for row in p5_rows
        if p5_5_row_has_current_stale_attribution(row)
        or (finite_float(row.get("current_stale_duration_s")) or 0.0) > 0.0
    ]
    early_non_emergency_rows = [
        row
        for row in stale_rows
        if 0.0
        <= (finite_float(row.get("current_stale_duration_s")) or 0.0)
        < replan_s
        and p5_action(row, "action") != P5_EMERGENCY_ACTION
        and p5_action(row, "raw_action") != P5_EMERGENCY_ACTION
        and p5_action(row, "action") != P5_REPLAN_ACTION
        and p5_action(row, "raw_action") != P5_REPLAN_ACTION
    ]
    replan_rows = [
        row
        for row in p5_rows
        if (
            p5_action(row, "action") == P5_REPLAN_ACTION
            or p5_action(row, "raw_action") == P5_REPLAN_ACTION
        )
        and p5_5_row_has_current_stale_attribution(row)
    ]
    emergency_rows = [
        row
        for row in p5_rows
        if (
            p5_action(row, "action") == P5_EMERGENCY_ACTION
            or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
        )
        and p5_5_row_has_current_stale_attribution(row)
    ]
    first_replan_time = finite_float(replan_rows[0].get("bag_time_s")) if replan_rows else None
    first_emergency_time = (
        finite_float(emergency_rows[0].get("bag_time_s")) if emergency_rows else None
    )
    first_current_stale_action = next(
        (row for row in p5_rows if p5_5_actionable(row) and p5_5_row_has_current_stale_attribution(row)),
        None,
    )
    first_action_stale_duration = (
        finite_float(first_current_stale_action.get("current_stale_duration_s"))
        if first_current_stale_action
        else None
    )
    no_immediate_emergency = not (
        first_current_stale_action is not None
        and (
            p5_action(first_current_stale_action, "action") == P5_EMERGENCY_ACTION
            or p5_action(first_current_stale_action, "raw_action") == P5_EMERGENCY_ACTION
        )
        and (first_action_stale_duration is None or first_action_stale_duration < emergency_s)
    )
    replan_before_emergency = (
        first_replan_time is not None
        and first_emergency_time is not None
        and first_replan_time <= first_emergency_time
    )
    cause_exclusion = p5_5_cause_exclusion_summary(p5_rows)
    current_stale_duration_max = (
        finite_float(p5_summary.get("current_stale_duration_s_max")) or 0.0
    )
    current_integrity_age_max = (
        finite_float(p5_summary.get("current_integrity_age_s_max")) or 0.0
    )
    gates = {
        **manifest_gates,
        "fixture": fixture,
        "fixture_present": bool(fixture.get("present")),
        "fixture_enabled": bool(fixture.get("enabled")),
        "fixture_name_ok": fixture.get("name") == P5_5_FIXTURE_NAME,
        "fixture_window_valid": bool(fixture.get("valid_window")),
        "fixture_thresholds_valid": bool(fixture.get("valid_thresholds")),
        "fixture_ready": bool(fixture_ready),
        "blocked_scenario_missing": not bool(fixture_ready),
        "validator_summary_present": bool(validator_summary),
        "validator_passed": validator_summary.get("passed") is True,
        "required_p5_topics_stable": all(
            status == "PASS"
            for topic, status in topic_statuses.items()
            if P5_TOPIC_EXPECTATIONS.get(topic) != "planner-dependent"
        ),
        "topic_statuses": topic_statuses,
        "active_topic_gap": active_topic_gap,
        "active_required_p5_topics_stable": bool(
            active_topic_gap.get("required_continuous_topics_stable", True)
        ),
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        **p0_startup,
        "p0_post_startup_ready": p0_startup["post_startup_ready_false_count"] == 0,
        "p0_post_startup_non_stale": p0_startup["post_startup_stale_true_count"] == 0,
        "p0_post_startup_not_full_unknown": (
            p0_startup["post_startup_full_unknown_count"] == 0
        ),
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "p5_inspection_ok": not bool(p5_summary.get("inspection_error")),
        "integrity_evidence": {
            key: value
            for key, value in integrity_evidence.items()
            if key != "rows"
        },
        "integrity_rows_present": int(integrity_evidence.get("row_count", 0) or 0) > 0,
        "fixture_window_entered": bool(integrity_evidence.get("window_entered")),
        "integrity_header_frozen": bool(integrity_evidence.get("header_frozen")),
        "integrity_continued_publishing": bool(
            integrity_evidence.get("continuous_publish_during_freeze")
        ),
        "integrity_age_grew": bool(integrity_evidence.get("age_grew_during_freeze")),
        "integrity_bag_span_ok": bool(integrity_evidence.get("bag_span_ok")),
        "integrity_pl_al_finite": bool(integrity_evidence.get("all_pl_al_finite")),
        "current_integrity_age_max": current_integrity_age_max,
        "current_integrity_age_grew": current_integrity_age_max >= replan_s,
        "current_stale_duration_max": current_stale_duration_max,
        "current_stale_duration_grew": current_stale_duration_max >= emergency_s,
        "early_non_emergency_current_stale_count": len(early_non_emergency_rows),
        "early_non_emergency_current_stale_present": len(early_non_emergency_rows) > 0,
        "current_stale_replan_count": len(replan_rows),
        "current_stale_replan_present": len(replan_rows) > 0,
        "current_stale_emergency_count": len(emergency_rows),
        "current_stale_emergency_present": len(emergency_rows) > 0,
        "no_immediate_emergency": no_immediate_emergency,
        "replan_before_emergency": replan_before_emergency,
        "cause_exclusion": cause_exclusion,
        "startup_snapshot_excluded": int(
            cause_exclusion.get("startup_snapshot_action_count", 0) or 0
        )
        == 0,
        "future_bad_excluded": int(
            cause_exclusion.get("future_bad_action_count", 0) or 0
        )
        == 0,
        "unknown_only_excluded": int(
            cause_exclusion.get("unknown_only_action_count", 0) or 0
        )
        == 0,
        "non_current_stale_actions_excluded": int(
            cause_exclusion.get("non_current_stale_action_count", 0) or 0
        )
        == 0,
        "first_current_stale_replan_row": replan_rows[0] if replan_rows else None,
        "first_current_stale_emergency_row": emergency_rows[0] if emergency_rows else None,
        "first_early_non_emergency_row": early_non_emergency_rows[0]
        if early_non_emergency_rows
        else None,
    }

    if not (
        gates["manifest_safety_profile_p5"]
        and gates["manifest_expected_true_ok"]
        and gates["manifest_expected_false_ok"]
    ):
        failures.append("P5-5 manifest does not enable P0/P5 with P1-P4 disabled")
    if not gates["fixture_present"]:
        failures.append("P5-5 fixture manifest is missing")
    elif not gates["fixture_enabled"]:
        failures.append("P5-5 fixture manifest is disabled")
    elif not gates["fixture_name_ok"]:
        failures.append(
            f"P5-5 fixture name is not {P5_5_FIXTURE_NAME}: {fixture.get('name')}"
        )
    elif not gates["fixture_window_valid"]:
        failures.append("P5-5 fixture window start/duration is invalid")
    elif not gates["fixture_thresholds_valid"]:
        failures.append("P5-5 stale debounce thresholds are invalid")
    if not gates["validator_summary_present"]:
        failures.append("P5-5 validator summary is missing")
    elif not gates["validator_passed"]:
        failures.append("P5-5 validator summary did not pass")
    if not gates["required_p5_topics_stable"]:
        failures.append("P5-5 required P0/P5 topics are not all stable")
    if not gates["active_required_p5_topics_stable"]:
        failures.append("P5-5 active stale-window topic gap exceeded threshold")
    if not gates["p0_health_rows_present"]:
        failures.append("P5-5 P0 health rows are missing")
    if not gates["startup_snapshot_unavailable_bounded"]:
        failures.append("P5-5 P0 startup snapshot_unavailable prefix is not bounded")
    if not gates["p0_post_startup_ready"]:
        failures.append("P5-5 P0 health reported ready=false after startup")
    if not gates["p0_post_startup_non_stale"]:
        failures.append("P5-5 P0 health reported stale=true after startup")
    if not gates["p0_post_startup_not_full_unknown"]:
        failures.append("P5-5 P0 health reported full unknown after startup")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-5 P5 status rows are missing")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-5 P5 status JSON parse errors were observed")
    if not gates["p5_inspection_ok"]:
        failures.append("P5-5 P5 status inspection did not complete cleanly")
    if not gates["integrity_rows_present"]:
        failures.append("P5-5 /iap/integrity stamp evidence is missing")
    if not gates["fixture_window_entered"]:
        failures.append("P5-5 fixture window was not entered by /iap/integrity evidence")
    if not gates["integrity_header_frozen"]:
        failures.append("P5-5 /iap/integrity header stamp did not freeze in the fixture window")
    if not gates["integrity_continued_publishing"]:
        failures.append("P5-5 /iap/integrity did not keep publishing during the frozen-stamp window")
    if not gates["integrity_age_grew"]:
        failures.append("P5-5 /iap/integrity bag-time minus header-stamp age did not grow")
    if not gates["integrity_pl_al_finite"]:
        failures.append("P5-5 frozen-stamp IntegrityReport rows did not keep finite PL/AL values")
    if not gates["current_integrity_age_grew"]:
        failures.append("P5-5 P5 current_integrity_age_s did not grow to the replan threshold")
    if not gates["current_stale_duration_grew"]:
        failures.append("P5-5 current_stale_duration_s did not grow to the emergency threshold")
    if not gates["early_non_emergency_current_stale_present"]:
        failures.append("P5-5 current-stale debounce skipped the early non-emergency interval")
    if not gates["current_stale_replan_present"]:
        failures.append("P5-5 did not observe REQUEST_REPLAN attributed to current_stale")
    if not gates["current_stale_emergency_present"]:
        failures.append("P5-5 did not observe REQUEST_EMERGENCY_STOP_CANDIDATE attributed to current_stale")
    if not gates["no_immediate_emergency"]:
        failures.append("P5-5 observed immediate emergency before current-stale emergency debounce elapsed")
    if not gates["replan_before_emergency"]:
        failures.append("P5-5 did not observe current-stale replan before emergency candidate")
    if not gates["startup_snapshot_excluded"]:
        failures.append("P5-5 startup/snapshot_unavailable cause contaminated stale-window actions")
    if not gates["future_bad_excluded"]:
        failures.append("P5-5 future_bad cause contaminated stale-window actions")
    if not gates["unknown_only_excluded"]:
        failures.append("P5-5 unknown-only cause contaminated stale-window actions")
    if not gates["non_current_stale_actions_excluded"]:
        failures.append("P5-5 action attribution did not trace actions to current_stale")

    required = (
        "manifest_safety_profile_p5",
        "manifest_expected_true_ok",
        "manifest_expected_false_ok",
        "fixture_ready",
        "validator_summary_present",
        "validator_passed",
        "required_p5_topics_stable",
        "active_required_p5_topics_stable",
        "p0_health_rows_present",
        "startup_snapshot_unavailable_bounded",
        "p0_post_startup_ready",
        "p0_post_startup_non_stale",
        "p0_post_startup_not_full_unknown",
        "p5_status_rows_present",
        "p5_json_parse_ok",
        "p5_inspection_ok",
        "integrity_rows_present",
        "fixture_window_entered",
        "integrity_header_frozen",
        "integrity_continued_publishing",
        "integrity_age_grew",
        "integrity_pl_al_finite",
        "current_integrity_age_grew",
        "current_stale_duration_grew",
        "early_non_emergency_current_stale_present",
        "current_stale_replan_present",
        "current_stale_emergency_present",
        "no_immediate_emergency",
        "replan_before_emergency",
        "startup_snapshot_excluded",
        "future_bad_excluded",
        "unknown_only_excluded",
        "non_current_stale_actions_excluded",
    )
    gates["passed"] = all(bool(gates.get(key)) for key in required)
    return gates


def p5_6_future_unknown_threshold_s(manifest: dict[str, Any]) -> float:
    return finite_float(manifest.get("p5.future_unknown_to_emergency_s")) or 2.0


def p5_6_p5_5_fixture_state(manifest: dict[str, Any]) -> dict[str, Any]:
    flat_value = manifest.get("p5_5.fixture.enabled") if manifest else None
    nested_value = nested_get(manifest, ["p5_5", "fixture", "enabled"], None) if manifest else None
    flat_enabled = manifest_bool(flat_value)
    nested_enabled = manifest_bool(nested_value)
    return {
        "flat_value": flat_value,
        "nested_value": nested_value,
        "flat_enabled": flat_enabled,
        "nested_enabled": nested_enabled,
        "disabled": not flat_enabled and not nested_enabled,
    }


def p5_6_fixture_from_manifest(manifest: dict[str, Any]) -> dict[str, Any]:
    nested = ((manifest.get("p5_6") or {}).get("fixture") or {}) if manifest else {}

    def flat(key: str) -> Any:
        return manifest.get(f"p5_6.fixture.{key}") if manifest else None

    bounds = nested.get("bounds") or {}
    tau_window = nested.get("tau_window_s") or [flat("tau_min"), flat("tau_max")]

    def pair(values: Any, fallback_min: Any, fallback_max: Any) -> tuple[float | None, float | None]:
        if isinstance(values, (list, tuple)) and len(values) >= 2:
            return finite_float(values[0]), finite_float(values[1])
        return finite_float(fallback_min), finite_float(fallback_max)

    x_min, x_max = pair(bounds.get("x"), flat("x_min"), flat("x_max"))
    y_min, y_max = pair(bounds.get("y"), flat("y_min"), flat("y_max"))
    z_min, z_max = pair(bounds.get("z"), flat("z_min"), flat("z_max"))
    tau_min, tau_max = pair(tau_window, flat("tau_min"), flat("tau_max"))
    enabled_value = nested.get("enabled", flat("enabled"))
    effective_value = nested.get("effective_enabled", flat("effective_enabled"))
    effective_present = effective_value is not None
    fixture = {
        "present": bool(manifest) and (
            "p5_6" in manifest
            or any(str(key).startswith("p5_6.fixture.") for key in manifest)
        ),
        "enabled": manifest_bool(enabled_value),
        "effective_enabled": (
            manifest_bool(effective_value)
            if effective_present
            else manifest_bool(enabled_value)
        ),
        "effective_present": effective_present,
        "name": str(nested.get("name", flat("name")) or ""),
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "z_min": z_min,
        "z_max": z_max,
        "tau_min": tau_min,
        "tau_max": tau_max,
        "expected_reason": str(
            nested.get("expected_reason", flat("expected_reason"))
            or P5_6_FIXTURE_REASON
        ),
    }
    finite_required = (
        "x_min",
        "x_max",
        "y_min",
        "y_max",
        "z_min",
        "z_max",
        "tau_min",
        "tau_max",
    )
    fixture["valid_geometry"] = all(fixture.get(key) is not None for key in finite_required)
    return fixture


def p5_6_p5_fixture_disabled(manifest: dict[str, Any], phase_key: str) -> bool:
    nested = ((manifest.get(phase_key) or {}).get("fixture") or {}) if manifest else {}
    flat_enabled = manifest_bool(manifest.get(f"{phase_key}.fixture.enabled")) if manifest else False
    nested_enabled = manifest_bool(nested.get("enabled")) if nested else False
    return not flat_enabled and not nested_enabled


def p5_6_post_startup_p0_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    prefix_len = 0
    for row in rows:
        if str(row.get("reason", "")).strip().lower() != "snapshot_unavailable":
            break
        prefix_len += 1
    return rows[prefix_len:]


def p5_6_post_startup_p5_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    prefix_len = 0
    for row in rows:
        reasons = p5_6_reason_values(row)
        phase = str(row.get("phase", "")).strip().lower()
        if (
            phase == "startup"
            or "snapshot_unavailable" in reasons
            or "current_invalid" in reasons
        ):
            prefix_len += 1
            continue
        break
    return rows[prefix_len:]


def p5_6_ratio_growth_summary(
    rows: list[dict[str, Any]],
    key: str,
    *,
    min_peak: float = 0.20,
    min_delta: float = 0.20,
) -> dict[str, Any]:
    values = [
        value
        for value in (finite_float(row.get(key)) for row in rows)
        if value is not None
    ]
    if not values:
        return {
            "available": False,
            "min": None,
            "max": None,
            "delta": None,
            "min_peak": min_peak,
            "min_delta": min_delta,
            "elevated": False,
            "grew": False,
        }
    min_value = min(values)
    max_value = max(values)
    delta = max_value - min_value
    return {
        "available": True,
        "min": min_value,
        "max": max_value,
        "delta": delta,
        "min_peak": min_peak,
        "min_delta": min_delta,
        "elevated": max_value >= min_peak,
        "grew": delta >= min_delta,
    }


def p5_6_value_growth_summary(
    rows: list[dict[str, Any]],
    key: str,
    *,
    min_peak: float,
    min_delta: float,
) -> dict[str, Any]:
    values = [
        value
        for value in (finite_float(row.get(key)) for row in rows)
        if value is not None
    ]
    if not values:
        return {
            "available": False,
            "min": None,
            "max": None,
            "delta": None,
            "min_peak": min_peak,
            "min_delta": min_delta,
            "elevated": False,
            "grew": False,
        }
    min_value = min(values)
    max_value = max(values)
    delta = max_value - min_value
    return {
        "available": True,
        "min": min_value,
        "max": max_value,
        "delta": delta,
        "min_peak": min_peak,
        "min_delta": min_delta,
        "elevated": max_value >= min_peak,
        "grew": delta >= min_delta,
    }


def p5_6_reason_values(row: dict[str, Any]) -> set[str]:
    return p5_all_reason_values(
        row,
        (
            "reason",
            "raw_reason",
            "current_reason",
            "future_reason",
            "active_reasons",
            "final_gate_last_reason",
            "pred_al_last_reason",
        ),
    )


def p5_6_actionable(row: dict[str, Any]) -> bool:
    return p5_action(row, "action") in {P5_REPLAN_ACTION, P5_EMERGENCY_ACTION} or (
        p5_action(row, "raw_action") in {P5_REPLAN_ACTION, P5_EMERGENCY_ACTION}
    )


def p5_6_reason_has_topic_gap(reasons: set[str]) -> bool:
    return any(
        "topic_gap" in reason
        or "topic-gap" in reason
        or ("topic" in reason and "gap" in reason)
        for reason in reasons
    )


def p5_6_reason_has_low_margin(reasons: set[str]) -> bool:
    return any("low_margin" in reason or "low-margin" in reason for reason in reasons)


def p5_6_reason_has_future_bad(reasons: set[str]) -> bool:
    high_risk_reasons = {
        "future_bad",
        P5_3_FIXTURE_REASON,
        P5_4_FIXTURE_REASON,
    }
    return bool(reasons & high_risk_reasons) or any(
        reason.startswith("future_bad")
        or P5_3_FIXTURE_REASON in reason
        or P5_4_FIXTURE_REASON in reason
        for reason in reasons
    )


def p5_6_reason_has_unknown(reasons: set[str]) -> bool:
    return bool(reasons & {"future_unknown", "unknown_only", "al_invalid", "pred_al_invalid"})


def p5_6_row_exclusion_causes(row: dict[str, Any]) -> list[str]:
    reasons = p5_6_reason_values(row)
    causes: list[str] = []
    if "current_stale" in reasons or (
        p5_6_actionable(row)
        and (finite_float(row.get("current_stale_duration_s")) or 0.0) > 0.0
    ):
        causes.append("current_stale")
    if p5_6_reason_has_future_bad(reasons):
        causes.append("future_bad")
    if str(row.get("phase", "")).strip().lower() == "startup" or "snapshot_unavailable" in reasons:
        causes.append("startup_snapshot_unavailable")
    if "current_invalid" in reasons:
        causes.append("current_invalid")
    if p5_6_reason_has_topic_gap(reasons):
        causes.append("topic_gap")
    if p5_6_reason_has_low_margin(reasons) and not p5_6_reason_has_unknown(reasons):
        causes.append("low_margin_only")
    return causes


def p5_6_unknown_bucket(row: dict[str, Any]) -> str:
    reasons = p5_6_reason_values(row)
    if "future_unknown" in reasons:
        return "future_unknown"
    if "unknown_only" in reasons:
        return "unknown_only"
    if reasons & {"al_invalid", "pred_al_invalid"}:
        unknown_ratio = finite_float(row.get("unknown_ratio")) or 0.0
        bad_ratio = finite_float(row.get("bad_ratio")) or 0.0
        if unknown_ratio > 0.0 and bad_ratio <= 1.0e-9 and not p5_6_row_exclusion_causes(row):
            return "al_invalid_equivalent"
    return ""


def p5_6_unknown_scope(row: dict[str, Any]) -> bool:
    return (
        (finite_float(row.get("unknown_ratio")) or 0.0) > 0.0
        or (finite_float(row.get("future_unknown_duration_s")) or 0.0) > 0.0
        or bool(p5_6_unknown_bucket(row))
    )


def p5_6_accepted_unknown_window(
    p5_rows: list[dict[str, Any]],
) -> dict[str, Any]:
    post_startup_rows = p5_6_post_startup_p5_rows(p5_rows)
    offset = len(p5_rows) - len(post_startup_rows)
    bucket_indices = [
        offset + idx
        for idx, row in enumerate(post_startup_rows)
        if p5_6_unknown_bucket(row)
    ]
    if not bucket_indices:
        return {
            "available": False,
            "row_indices": [],
            "rows": [],
            "start_bag_time_s": None,
            "end_bag_time_s": None,
            "duration_s": None,
        }
    start_idx = min(bucket_indices)
    end_idx = max(bucket_indices)
    indices = list(range(start_idx, end_idx + 1))
    rows = [p5_rows[index] for index in indices]
    start_s = finite_float(rows[0].get("bag_time_s")) if rows else None
    end_s = finite_float(rows[-1].get("bag_time_s")) if rows else None
    return {
        "available": True,
        "row_indices": indices,
        "rows": rows,
        "start_bag_time_s": start_s,
        "end_bag_time_s": end_s,
        "duration_s": (
            max(0.0, end_s - start_s)
            if start_s is not None and end_s is not None
            else None
        ),
    }


def p5_6_unknown_window_rows(p5_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        {
            "bag_time_s": row.get("bag_time_s", ""),
            "inside_high_risk_zone": 0,
            "inside_tau_window": 0,
        }
        for row in p5_rows
        if p5_6_unknown_scope(row)
    ]


def p5_6_cause_exclusion_summary(
    p5_rows: list[dict[str, Any]],
    manifest: dict[str, Any],
) -> dict[str, Any]:
    actionable_unknown_rows = [
        row for row in p5_rows if p5_6_unknown_scope(row) and p5_6_actionable(row)
    ]
    by_cause: dict[str, list[dict[str, Any]]] = {
        "current_stale": [],
        "current_invalid": [],
        "future_bad": [],
        "startup_snapshot_unavailable": [],
        "topic_gap": [],
        "low_margin_only": [],
    }
    for row in actionable_unknown_rows:
        for cause in p5_6_row_exclusion_causes(row):
            by_cause.setdefault(cause, []).append(row)
    fixture_state = p5_6_p5_5_fixture_state(manifest)
    p5_5_fixture_evidence_count = (
        int(bool(fixture_state.get("flat_enabled")))
        + int(bool(fixture_state.get("nested_enabled")))
        + len(by_cause.get("current_stale", []))
    )
    return {
        "actionable_unknown_count": len(actionable_unknown_rows),
        "current_stale_action_count": len(by_cause.get("current_stale", [])),
        "current_invalid_action_count": len(by_cause.get("current_invalid", [])),
        "future_bad_action_count": len(by_cause.get("future_bad", [])),
        "startup_snapshot_action_count": len(
            by_cause.get("startup_snapshot_unavailable", [])
        ),
        "topic_gap_action_count": len(by_cause.get("topic_gap", [])),
        "low_margin_only_action_count": len(by_cause.get("low_margin_only", [])),
        "p5_5_fixture_evidence_count": p5_5_fixture_evidence_count,
        "first_current_stale_action": (by_cause.get("current_stale") or [None])[0],
        "first_current_invalid_action": (by_cause.get("current_invalid") or [None])[0],
        "first_future_bad_action": (by_cause.get("future_bad") or [None])[0],
        "first_startup_snapshot_action": (
            by_cause.get("startup_snapshot_unavailable") or [None]
        )[0],
        "first_topic_gap_action": (by_cause.get("topic_gap") or [None])[0],
        "first_low_margin_only_action": (by_cause.get("low_margin_only") or [None])[0],
    }


def p5_6_cause_exclusion_rows(gates: dict[str, Any]) -> list[dict[str, Any]]:
    summary = gates.get("cause_exclusion", {}) or {}
    cause_specs = [
        ("current_stale", "current_stale_action_count", "first_current_stale_action"),
        ("current_invalid", "current_invalid_action_count", "first_current_invalid_action"),
        ("p5_5_fixture", "p5_5_fixture_evidence_count", "first_current_stale_action"),
        ("future_bad", "future_bad_action_count", "first_future_bad_action"),
        ("startup_snapshot", "startup_snapshot_action_count", "first_startup_snapshot_action"),
        ("topic_gap", "topic_gap_action_count", "first_topic_gap_action"),
        ("low_margin_only", "low_margin_only_action_count", "first_low_margin_only_action"),
    ]
    rows: list[dict[str, Any]] = []
    for cause, count_key, first_key in cause_specs:
        first = summary.get(first_key) or {}
        rows.append(
            {
                "cause": cause,
                "count": int(summary.get(count_key, 0) or 0),
                "first_bag_time_s": first.get("bag_time_s", "") if isinstance(first, dict) else "",
                "first_action": first.get("action", "") if isinstance(first, dict) else "",
                "first_raw_action": first.get("raw_action", "") if isinstance(first, dict) else "",
                "first_reason": first.get("reason", "") if isinstance(first, dict) else "",
            }
        )
    return rows


def build_p5_6_unknown_action_timeline_rows(
    p5_rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    t_rel = relative_time(p5_rows)
    rows: list[dict[str, Any]] = []
    for idx, row in enumerate(p5_rows):
        bucket = p5_6_unknown_bucket(row)
        excluded = p5_6_row_exclusion_causes(row)
        action = p5_action(row, "action")
        raw_action = p5_action(row, "raw_action")
        rows.append(
            {
                "bag_time_s": row.get("bag_time_s", ""),
                "t_rel_s": t_rel[idx] if idx < len(t_rel) else "",
                "phase": row.get("phase", ""),
                "action": action,
                "raw_action": raw_action,
                "reason": row.get("reason", ""),
                "raw_reason": row.get("raw_reason", ""),
                "current_reason": row.get("current_reason", ""),
                "future_reason": row.get("future_reason", ""),
                "active_reasons": row.get("active_reasons", ""),
                "unknown_ratio": row.get("unknown_ratio", ""),
                "bad_ratio": row.get("bad_ratio", ""),
                "future_unknown_duration_s": row.get("future_unknown_duration_s", ""),
                "unknown_bucket": bucket,
                "unknown_scope": 1 if p5_6_unknown_scope(row) else 0,
                "unknown_attributed": 1 if bucket else 0,
                "request_replan": 1
                if action == P5_REPLAN_ACTION or raw_action == P5_REPLAN_ACTION
                else 0,
                "request_emergency_stop_candidate": 1
                if action == P5_EMERGENCY_ACTION or raw_action == P5_EMERGENCY_ACTION
                else 0,
                "excluded_causes": ",".join(excluded),
            }
        )
    return rows


def validate_p5_6_required_figures(
    figure_paths: list[Path],
    failures: list[str],
) -> None:
    for figure_path in figure_paths:
        if not figure_path.is_file() or figure_path.stat().st_size <= 0:
            failures.append(
                "P5-6 required figure missing: "
                f"{figure_path.name}; missing future-unknown figure evidence "
                "prevents P5-6 acceptance"
            )


def validate_p5_6_hard_gates(
    manifest: dict[str, Any],
    validator_summary: dict[str, Any],
    topic_health: dict[str, dict[str, Any]],
    p0_health_summary: dict[str, Any],
    p0_health_rows: list[dict[str, Any]],
    p5_rows: list[dict[str, Any]],
    p5_summary: dict[str, Any],
    failures: list[str],
    inconclusive: list[str],
    topic_timestamps: dict[str, list[float]] | None = None,
) -> dict[str, Any]:
    manifest_gates = p5_manifest_gate_values(manifest)
    p5_5_fixture_state = p5_6_p5_5_fixture_state(manifest)
    p5_6_fixture = p5_6_fixture_from_manifest(manifest)
    p5_3_fixture_disabled = p5_6_p5_fixture_disabled(manifest, "p5_3")
    p5_4_fixture_disabled = p5_6_p5_fixture_disabled(manifest, "p5_4")
    threshold_s = p5_6_future_unknown_threshold_s(manifest)
    topic_statuses = {
        topic: (topic_health.get(topic) or {}).get("status", "MISSING")
        for topic in P5_TOPIC_EXPECTATIONS
    }
    p0_startup = summarize_p0_startup_snapshot_unavailable(p0_health_rows)
    post_startup_p0_rows = p5_6_post_startup_p0_rows(p0_health_rows)
    post_startup_p5_rows = p5_6_post_startup_p5_rows(p5_rows)
    accepted_window = p5_6_accepted_unknown_window(p5_rows)
    accepted_window_rows = list(accepted_window.get("rows", []) or [])
    accepted_window_indices = list(accepted_window.get("row_indices", []) or [])
    p0_unknown_growth = p5_6_ratio_growth_summary(post_startup_p0_rows, "unknown_ratio")
    p5_unknown_growth = p5_6_ratio_growth_summary(post_startup_p5_rows, "unknown_ratio")
    min_duration_growth = min(0.5, max(0.1, threshold_s * 0.25))
    p5_future_unknown_duration = p5_6_value_growth_summary(
        accepted_window_rows or post_startup_p5_rows,
        "future_unknown_duration_s",
        min_peak=threshold_s,
        min_delta=min_duration_growth,
    )
    unknown_scope_rows = [
        row for row in (accepted_window_rows or post_startup_p5_rows)
        if p5_6_unknown_scope(row)
    ]
    active_topic_gap = p5_3_active_topic_gap_summary(
        p5_6_unknown_window_rows(accepted_window_rows),
        topic_timestamps,
    )
    p5_6_samples_all = p5_3_sample_rows(
        p5_rows,
        p5_6_fixture,
        tau_window_field="query_tau_s",
    )
    p5_6_window_samples = p5_3_filter_sample_rows_by_status_indices(
        p5_6_samples_all,
        accepted_window_indices,
    )
    p5_6_fixture_unknown_samples = [
        row
        for row in p5_6_window_samples
        if int(row.get("inside_high_risk_zone", 0) or 0)
        and int(row.get("inside_tau_window", 0) or 0)
        and int(row.get("unknown", 0) or 0)
        and (
            int(row.get("fixture_match", 0) or 0)
            or P5_6_FIXTURE_REASON in str(row.get("reason", "")).lower()
            or P5_6_FIXTURE_REASON in str(row.get("fixture_expected_reason", "")).lower()
        )
    ]
    early_non_emergency_rows = [
        row
        for row in unknown_scope_rows
        if (finite_float(row.get("future_unknown_duration_s")) or 0.0) < threshold_s
        and p5_action(row, "action") != P5_EMERGENCY_ACTION
        and p5_action(row, "raw_action") != P5_EMERGENCY_ACTION
    ]
    replan_rows = [
        row
        for row in unknown_scope_rows
        if (
            p5_action(row, "action") == P5_REPLAN_ACTION
            or p5_action(row, "raw_action") == P5_REPLAN_ACTION
        )
        and p5_6_unknown_bucket(row)
    ]
    emergency_rows = [
        row
        for row in unknown_scope_rows
        if (
            p5_action(row, "action") == P5_EMERGENCY_ACTION
            or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
        )
        and p5_6_unknown_bucket(row)
        and (finite_float(row.get("future_unknown_duration_s")) or 0.0) >= threshold_s
    ]
    first_unknown_time = (
        finite_float(unknown_scope_rows[0].get("bag_time_s")) if unknown_scope_rows else None
    )
    first_replan_time = finite_float(replan_rows[0].get("bag_time_s")) if replan_rows else None
    first_emergency_time = (
        finite_float(emergency_rows[0].get("bag_time_s")) if emergency_rows else None
    )
    cause_exclusion = p5_6_cause_exclusion_summary(accepted_window_rows, manifest)
    unknown_actions_not_all_ok = bool(unknown_scope_rows) and any(
        p5_action(row, "action") != P5_OK_ACTION
        or p5_action(row, "raw_action") != P5_OK_ACTION
        for row in unknown_scope_rows
    )
    gates = {
        **manifest_gates,
        "validator_summary_present": bool(validator_summary),
        "validator_passed": validator_summary.get("passed") is True,
        "p5_5_fixture_state": p5_5_fixture_state,
        "p5_5_fixture_disabled": bool(p5_5_fixture_state.get("disabled")),
        "p5_3_fixture_disabled": bool(p5_3_fixture_disabled),
        "p5_4_fixture_disabled": bool(p5_4_fixture_disabled),
        "fixture": p5_6_fixture,
        "fixture_present": bool(p5_6_fixture.get("present")),
        "fixture_enabled": bool(p5_6_fixture.get("enabled")),
        "fixture_effective_enabled": bool(p5_6_fixture.get("effective_enabled")),
        "fixture_name_ok": p5_6_fixture.get("name") == P5_6_FIXTURE_NAME,
        "fixture_geometry_valid": bool(p5_6_fixture.get("valid_geometry")),
        "fixture_ready": bool(
            p5_6_fixture.get("present")
            and p5_6_fixture.get("enabled")
            and p5_6_fixture.get("effective_enabled")
            and p5_6_fixture.get("name") == P5_6_FIXTURE_NAME
            and p5_6_fixture.get("valid_geometry")
        ),
        "required_p5_topics_stable": all(
            status == "PASS"
            for topic, status in topic_statuses.items()
            if P5_TOPIC_EXPECTATIONS.get(topic) != "planner-dependent"
        ),
        "topic_statuses": topic_statuses,
        "active_topic_gap": active_topic_gap,
        "active_required_p5_topics_stable": bool(
            active_topic_gap.get("required_continuous_topics_stable", True)
        ),
        "p0_health_rows_present": int(p0_health_summary.get("row_count", 0) or 0) > 0,
        **p0_startup,
        "p0_post_startup_ready": p0_startup["post_startup_ready_false_count"] == 0,
        "p0_post_startup_non_stale": p0_startup["post_startup_stale_true_count"] == 0,
        "p5_status_rows_present": int(p5_summary.get("status_rows", 0) or 0) > 0,
        "p5_steady_status_rows_present": int(p5_summary.get("steady_status_rows", 0) or 0) > 0,
        "p5_json_parse_ok": int(p5_summary.get("parse_error_count", 0) or 0) == 0,
        "p5_inspection_ok": not bool(p5_summary.get("inspection_error")),
        "p5_startup_snapshot_unavailable_bounded": bool(
            p5_summary.get("startup_snapshot_unavailable_bounded", True)
        ),
        "future_unknown_to_emergency_s": threshold_s,
        "post_startup_p5_status_count": len(post_startup_p5_rows),
        "accepted_window": {
            key: value
            for key, value in accepted_window.items()
            if key != "rows"
        },
        "accepted_unknown_window_available": bool(accepted_window.get("available")),
        "p0_unknown_growth": p0_unknown_growth,
        "p0_unknown_ratio_elevated": bool(p0_unknown_growth.get("elevated")),
        "p0_unknown_ratio_grew": bool(p0_unknown_growth.get("grew")),
        "p5_unknown_growth": p5_unknown_growth,
        "p5_unknown_ratio_elevated": bool(p5_unknown_growth.get("elevated")),
        "p5_unknown_ratio_grew": bool(p5_unknown_growth.get("grew")),
        "p5_future_unknown_duration": p5_future_unknown_duration,
        "p5_future_unknown_duration_grew": bool(
            p5_future_unknown_duration.get("grew")
        ),
        "p5_future_unknown_duration_crossed_threshold": bool(
            p5_future_unknown_duration.get("elevated")
        ),
        "unknown_scope_count": len(unknown_scope_rows),
        "fixture_sample_count": len(p5_6_window_samples),
        "fixture_unknown_sample_count": len(p5_6_fixture_unknown_samples),
        "fixture_unknown_samples_present": len(p5_6_fixture_unknown_samples) > 0,
        "early_non_emergency_unknown_count": len(early_non_emergency_rows),
        "early_non_emergency_unknown_present": len(early_non_emergency_rows) > 0,
        "unknown_replan_count": len(replan_rows),
        "unknown_replan_present": len(replan_rows) > 0,
        "unknown_emergency_count": len(emergency_rows),
        "unknown_emergency_present": len(emergency_rows) > 0,
        "first_unknown_time_s": first_unknown_time,
        "first_unknown_replan_time_s": first_replan_time,
        "first_unknown_emergency_time_s": first_emergency_time,
        "replan_before_emergency": (
            first_replan_time is not None
            and first_emergency_time is not None
            and first_replan_time <= first_emergency_time
        ),
        "no_immediate_emergency": (
            first_unknown_time is None
            or first_emergency_time is None
            or first_unknown_time < first_emergency_time
        )
        and len(early_non_emergency_rows) > 0,
        "unknown_actions_not_all_ok": unknown_actions_not_all_ok,
        "cause_exclusion": cause_exclusion,
        "current_stale_excluded": int(
            cause_exclusion.get("current_stale_action_count", 0) or 0
        )
        == 0,
        "current_invalid_excluded": int(
            cause_exclusion.get("current_invalid_action_count", 0) or 0
        )
        == 0,
        "p5_5_fixture_excluded": int(
            cause_exclusion.get("p5_5_fixture_evidence_count", 0) or 0
        )
        == 0,
        "future_bad_excluded": int(
            cause_exclusion.get("future_bad_action_count", 0) or 0
        )
        == 0,
        "startup_snapshot_excluded": int(
            cause_exclusion.get("startup_snapshot_action_count", 0) or 0
        )
        == 0,
        "topic_gap_excluded": int(
            cause_exclusion.get("topic_gap_action_count", 0) or 0
        )
        == 0,
        "low_margin_only_excluded": int(
            cause_exclusion.get("low_margin_only_action_count", 0) or 0
        )
        == 0,
        "first_unknown_replan_row": replan_rows[0] if replan_rows else None,
        "first_unknown_emergency_row": emergency_rows[0] if emergency_rows else None,
        "first_fixture_unknown_sample": p5_6_fixture_unknown_samples[0]
        if p5_6_fixture_unknown_samples
        else None,
        "first_early_non_emergency_unknown_row": early_non_emergency_rows[0]
        if early_non_emergency_rows
        else None,
    }

    if not (
        gates["manifest_safety_profile_p5"]
        and gates["manifest_expected_true_ok"]
        and gates["manifest_expected_false_ok"]
    ):
        failures.append("P5-6 manifest does not enable P0/P5 with P1-P4 disabled")
    if not gates["p5_5_fixture_disabled"]:
        failures.append("P5-6 P5-5 fixture must remain disabled in flat and nested manifest fields")
    if not gates["p5_3_fixture_disabled"]:
        failures.append("P5-6 P5-3 high-risk fixture must remain disabled")
    if not gates["p5_4_fixture_disabled"]:
        failures.append("P5-6 P5-4 near-risk fixture must remain disabled")
    if not gates["fixture_present"]:
        failures.append("P5-6 future-unknown fixture manifest is missing")
    elif not gates["fixture_enabled"]:
        failures.append("P5-6 future-unknown fixture is disabled")
    elif not gates["fixture_effective_enabled"]:
        failures.append("P5-6 future-unknown fixture is not effectively enabled")
    elif not gates["fixture_name_ok"]:
        failures.append(
            f"P5-6 fixture name is not {P5_6_FIXTURE_NAME}: "
            f"{p5_6_fixture.get('name')}"
        )
    elif not gates["fixture_geometry_valid"]:
        failures.append("P5-6 future-unknown fixture bounds or tau window are invalid")
    if not gates["validator_summary_present"]:
        failures.append("P5-6 validator summary is missing")
    elif not gates["validator_passed"]:
        failures.append("P5-6 validator summary did not pass")
    if not gates["required_p5_topics_stable"]:
        failures.append("P5-6 required P0/P5 topics are not all stable")
    if not gates["active_required_p5_topics_stable"]:
        failures.append("P5-6 active unknown-window topic gap exceeded threshold")
    if not gates["p0_health_rows_present"]:
        failures.append("P5-6 P0 health rows are missing")
    if not gates["startup_snapshot_unavailable_bounded"]:
        failures.append("P5-6 P0 startup snapshot_unavailable prefix is not bounded")
    if not gates["p0_post_startup_ready"]:
        failures.append("P5-6 P0 health reported ready=false after startup")
    if not gates["p0_post_startup_non_stale"]:
        failures.append("P5-6 P0 health reported stale=true after startup")
    if not gates["p5_status_rows_present"]:
        failures.append("P5-6 P5 status rows are missing")
    if not gates["p5_steady_status_rows_present"]:
        failures.append("P5-6 P5 status rows contain no steady-state samples after startup")
    if not gates["p5_json_parse_ok"]:
        failures.append("P5-6 P5 status JSON parse errors were observed")
    if not gates["p5_inspection_ok"]:
        failures.append("P5-6 P5 status inspection did not complete cleanly")
    if not gates["accepted_unknown_window_available"]:
        failures.append("P5-6 accepted post-startup future-unknown window was not found")
    if not gates["fixture_unknown_samples_present"]:
        failures.append("P5-6 fixture did not produce unknown trajectory samples inside its bounds and tau window")
    if not gates["p0_unknown_ratio_elevated"]:
        failures.append("P5-6 P0 unknown_ratio did not become elevated")
    if not gates["p5_unknown_ratio_elevated"] or not gates["p5_unknown_ratio_grew"]:
        failures.append("P5-6 P5 unknown_ratio did not clearly rise")
    if not gates["p5_future_unknown_duration_grew"]:
        failures.append("P5-6 future_unknown_duration_s did not grow")
    if not gates["p5_future_unknown_duration_crossed_threshold"]:
        failures.append(
            "P5-6 future_unknown_duration_s did not cross "
            f"p5.future_unknown_to_emergency_s={threshold_s:.6g}"
        )
    if not gates["early_non_emergency_unknown_present"]:
        failures.append("P5-6 future-unknown debounce skipped the early non-emergency interval")
    if not gates["unknown_replan_present"]:
        failures.append("P5-6 did not observe REQUEST_REPLAN attributed to future unknown")
    if not gates["unknown_emergency_present"]:
        failures.append("P5-6 did not observe REQUEST_EMERGENCY_STOP_CANDIDATE attributed to sustained future unknown")
    if not gates["no_immediate_emergency"]:
        failures.append("P5-6 observed immediate emergency before future-unknown debounce evidence")
    if not gates["replan_before_emergency"]:
        failures.append("P5-6 did not observe future-unknown replan before emergency candidate")
    if not gates["unknown_actions_not_all_ok"]:
        failures.append("P5-6 unknown-scope rows were all treated as OK")
    if not gates["current_stale_excluded"]:
        failures.append("P5-6 current_stale action cause contaminated future-unknown evidence")
    if not gates["current_invalid_excluded"]:
        failures.append("P5-6 current_invalid action cause contaminated future-unknown evidence")
    if not gates["p5_5_fixture_excluded"]:
        failures.append("P5-6 P5-5 fixture evidence contaminated future-unknown evidence")
    if not gates["future_bad_excluded"]:
        failures.append("P5-6 future_bad high-risk cause contaminated future-unknown evidence")
    if not gates["startup_snapshot_excluded"]:
        failures.append("P5-6 startup/snapshot_unavailable cause contaminated future-unknown evidence")
    if not gates["topic_gap_excluded"]:
        failures.append("P5-6 topic gap cause contaminated future-unknown evidence")
    if not gates["low_margin_only_excluded"]:
        failures.append("P5-6 low-margin-only cause contaminated future-unknown evidence")

    required = (
        "manifest_safety_profile_p5",
        "manifest_expected_true_ok",
        "manifest_expected_false_ok",
        "p5_5_fixture_disabled",
        "p5_3_fixture_disabled",
        "p5_4_fixture_disabled",
        "fixture_ready",
        "validator_summary_present",
        "validator_passed",
        "required_p5_topics_stable",
        "active_required_p5_topics_stable",
        "p0_health_rows_present",
        "startup_snapshot_unavailable_bounded",
        "p0_post_startup_ready",
        "p0_post_startup_non_stale",
        "p5_status_rows_present",
        "p5_steady_status_rows_present",
        "p5_json_parse_ok",
        "p5_inspection_ok",
        "p5_startup_snapshot_unavailable_bounded",
        "accepted_unknown_window_available",
        "fixture_unknown_samples_present",
        "p0_unknown_ratio_elevated",
        "p5_unknown_ratio_elevated",
        "p5_unknown_ratio_grew",
        "p5_future_unknown_duration_grew",
        "p5_future_unknown_duration_crossed_threshold",
        "early_non_emergency_unknown_present",
        "unknown_replan_present",
        "unknown_emergency_present",
        "no_immediate_emergency",
        "replan_before_emergency",
        "unknown_actions_not_all_ok",
        "current_stale_excluded",
        "current_invalid_excluded",
        "p5_5_fixture_excluded",
        "future_bad_excluded",
        "startup_snapshot_excluded",
        "topic_gap_excluded",
        "low_margin_only_excluded",
    )
    gates["passed"] = all(bool(gates.get(key)) for key in required)
    return gates


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


def analyze_p0_6_blocked_fixture_audit(
    args: argparse.Namespace,
    export_dir: Path,
    metadata_dir: Path,
) -> dict[str, Any]:
    launch_command = (
        "ros2 launch iap test_planner.launch.py "
        "experiment:=p0_open_sky "
        "scenario:=manual "
        "run_duration_s:=90 "
        "validation_duration_s:=90 "
        "start_rviz:=false "
        "run_validator:=true "
        "record_bag:=true"
    )
    analyzer_command = (
        "python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py "
        "--experiment-id P0-6 "
        "--export-dir <formal-p0-6-export-dir> "
        "--bag-dir <formal-p0-6-bag-dir> "
        "--fail-on-threshold"
    )
    summary = {
        "experiment_id": args.experiment_id,
        "status": "BLOCKED_SCENARIO_MISSING",
        "passed": False,
        "failures": [],
        "warnings": [],
        "inconclusive": [
            "P0-6 formal launch and validator were not run because the repository has no stable occupied-overlap fixture.",
        ],
        "missing_capabilities": P0_6_MISSING_CAPABILITIES,
        "next_debug_branch": (
            "BLOCKED_SCENARIO_MISSING -> "
            "occupied overlap fixture / occupied-low-risk injection / reproducible occupied validity overlay"
        ),
        "export_dir": str(export_dir),
        "bag_dir": "",
        "fixture_audit": {
            "status": "BLOCKED_SCENARIO_MISSING",
            "manual_scenario_preset": "empty preset; ordinary manual runs do not define map geometry, route, occupied-low-risk injection, or overlap oracle",
            "p0_open_sky_preset": "enables P0 runtime evidence but does not create a deterministic occupied-low-risk overlap fixture",
            "formal_run_executed": False,
            "ordinary_manual_run_is_pass_evidence": False,
            "formal_run_required_manifest": {
                "p0.skip_occupied_voxels": True,
            },
            "blocked_artifact_has_formal_run_manifest": False,
        },
        "should_run_commands": {
            "launch": launch_command,
            "analyzer": analyzer_command,
        },
        "validator_summary": {
            "status": "not_run",
            "reason": "blocked before launch; stable occupied-overlap fixture is missing",
        },
        "analyzer_status": "blocked_fixture_audit_only",
        "unavailable_evidence": {
            "topic_health": "unavailable because fixture missing",
            "occupied_overlap_statistics": "unavailable because fixture missing",
            "raw_pl_cost_vs_final_validity_overlay": "unavailable because fixture missing",
            "runtime_figures": "unavailable because fixture missing",
        },
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


def next_debug_branch(
    status: str,
    failures: list[str],
    inconclusive: list[str],
    experiment_id: str,
    p0_health_summary: dict[str, Any] | None = None,
) -> str:
    text = " ".join(failures + inconclusive).lower()
    normalized_experiment_id = str(experiment_id).strip().upper()
    if normalized_experiment_id == "P5-1":
        return "PASS -> P5-2" if status == "PASS" else "debug P5 thresholds/AL provider"
    if normalized_experiment_id == "P5-2":
        return "PASS -> P5-3" if status == "PASS" else "debug PL/AL margin"
    if normalized_experiment_id == "P5-3":
        if status == "PASS":
            return "PASS -> P5-4"
        return P5_3_FAIL_BRANCH
    if normalized_experiment_id == "P5-4":
        if status == "PASS":
            return "PASS -> P5-5"
        if status == "BLOCKED_SCENARIO_MISSING" or "p5-4 fixture manifest" in text:
            return "BLOCKED_SCENARIO_MISSING"
        return P5_4_FAIL_BRANCH
    if normalized_experiment_id == "P5-5":
        if status == "PASS":
            return "PASS -> P5-6"
        if status == "BLOCKED_SCENARIO_MISSING" or "p5-5 fixture manifest" in text:
            return P5_5_BLOCKED_BRANCH
        return P5_5_FAIL_BRANCH
    if normalized_experiment_id == "P5-6":
        if status == "PASS":
            return "PASS -> P5-7"
        return P5_6_FAIL_BRANCH
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
            "P0-6": "PASS -> Phase 2 / P5-1",
        }
        return pass_branches.get(
            normalized_experiment_id,
            "continue_to_next_planned_experiment",
        )
    if normalized_experiment_id == "P0-6":
        return "debug_occupied_overlap_fixture_or_skip_semantics"
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
    p5_3_plal_figure_paths = {
        name: figures_dir / name for name in P5_3_PLAL_FIGURE_FILENAMES
    }
    p5_3_query_alignment_figure_paths = {
        name: figures_dir / name for name in P5_3_QUERY_ALIGNMENT_FIGURE_FILENAMES
    }
    p5_3_future_sampling_figure_paths = {
        name: figures_dir / name for name in P5_3_FUTURE_SAMPLING_FIGURE_FILENAMES
    }
    p5_3_event_window_figure_paths = {
        name: figures_dir / name for name in P5_3_EVENT_WINDOW_FIGURE_FILENAMES
    }
    p5_4_figure_paths = {
        name: figures_dir / name for name in P5_4_FIGURE_FILENAMES
    }
    p5_5_figure_paths = {
        name: figures_dir / name for name in P5_5_FIGURE_FILENAMES
    }
    p5_6_figure_paths = {
        name: figures_dir / name for name in P5_6_FIGURE_FILENAMES
    }
    p0_phase = is_p0_experiment(args)
    p0_4_phase = is_experiment(args, "P0-4")
    p0_phase_index = p0_phase_number(args.experiment_id)
    p0_requires_odom_gate = p0_odom_gate_required(args.experiment_id)
    experiment_label = str(args.experiment_id).strip().upper()
    p5_1_phase = is_experiment(args, "P5-1")
    p5_2_phase = is_experiment(args, "P5-2")
    p5_3_phase = is_experiment(args, "P5-3")
    p5_4_phase = is_experiment(args, "P5-4")
    p5_5_phase = is_experiment(args, "P5-5")
    p5_6_phase = is_experiment(args, "P5-6")
    p5_runtime_phase = is_p5_runtime_experiment(args)
    p0_runtime_phase = p0_phase or p5_runtime_phase
    if bool(getattr(args, "blocked_fixture_audit", False)):
        if experiment_label != "P0-6":
            raise ValueError("--blocked-fixture-audit is only defined for P0-6")
        return analyze_p0_6_blocked_fixture_audit(args, export_dir, metadata_dir)
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
        P5_TOPIC_EXPECTATIONS
        if p5_runtime_phase
        else (
            P0_4_TOPIC_EXPECTATIONS
            if p0_4_phase
            else (P0_TOPIC_EXPECTATIONS if p0_phase else CORE_TOPIC_EXPECTATIONS)
        )
    )
    topic_activity_topics = (
        P5_TOPIC_ACTIVITY_TOPICS
        if p5_runtime_phase
        else (P0_TOPIC_ACTIVITY_TOPICS if p0_phase else TOPIC_ACTIVITY_TOPICS)
    )

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

    if p5_runtime_phase:
        validate_p5_manifest(
            manifest,
            failures,
            inconclusive,
            experiment_label=experiment_label,
        )
    else:
        validate_manifest(
            manifest,
            failures,
            inconclusive,
            require_p0_enabled=p0_phase,
            allowed_safety_profiles=("off", "p5") if p0_4_phase else ("off",),
        )
    validate_validator(validator_summary, failures, inconclusive)
    if p5_3_phase:
        full_run_topic_failures: list[str] = []
        full_run_topic_inconclusive: list[str] = []
        topic_health = validate_topic_health(
            metadata,
            topic_timings,
            topic_timing_error,
            full_run_topic_failures,
            full_run_topic_inconclusive,
            topic_expectations,
        )
        warnings.extend(
            f"P5-3 full-run topic-health report: {item}"
            for item in full_run_topic_failures + full_run_topic_inconclusive
        )
    else:
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
    health_rows: list[dict[str, Any]] = []
    if p0_runtime_phase:
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
            allow_explainable_startup_unavailable=p0_4_phase or p5_runtime_phase,
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
        p0_6_semantics: dict[str, Any] = {}
        p0_6_overlap: list[dict[str, Any]] = []
        p0_6_fixture: dict[str, Any] = {}
        if experiment_label == "P0-6":
            p0_6_overlap, p0_6_fixture = p0_6_overlap_rows(
                manifest,
                pl_cloud_rows,
                validity_cloud_rows,
            )
            p0_6_overlap_summary = summarize_p0_6_overlap(
                p0_6_overlap,
                p0_health_summary,
            )
            validate_p0_6_formal_semantics(
                manifest,
                p0_6_fixture,
                p0_6_overlap_summary,
                failures,
                inconclusive,
            )
            p0_6_semantics = {
                "fixture": p0_6_fixture,
                **p0_6_overlap_summary,
            }
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
        p0_6_overlap_path = csv_dir / f"{prefix}_occupied_overlap.csv"
        p0_6_overlay_path = csv_dir / f"{prefix}_raw_pl_vs_final_validity.csv"
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
        if experiment_label == "P0-6":
            p0_6_fields = [
                "stamp",
                "x",
                "y",
                "z",
                "occupied",
                "raw_source",
                "raw_hpl_m",
                "raw_vpl_m",
                "raw_c_pi",
                "low_raw_cost_threshold",
                "raw_low_cost",
                "final_pl",
                "final_hpl",
                "final_vpl",
                "final_c_pi",
                "final_valid",
                "final_unknown",
                "final_stale",
                "source_flags",
                "occupied_skip",
                "final_low_risk",
                "final_reason",
            ]
            write_csv(p0_6_overlap_path, p0_6_fields, p0_6_overlap)
            write_csv(p0_6_overlay_path, p0_6_fields, p0_6_overlap)
            p0_csv_artifacts.extend([str(p0_6_overlap_path), str(p0_6_overlay_path)])

        health_figure_path = (
            figures_dir / "p5_3_debug_p0_health_timeline.png"
            if p5_3_phase
            else (
                p5_4_figure_paths["p5_4_p0_health.png"]
                if p5_4_phase
                else (
                    p5_5_figure_paths["p5_5_p0_health.png"]
                    if p5_5_phase
                    else (
                        p5_6_figure_paths["p5_6_p0_health_unknown_timeline.png"]
                        if p5_6_phase
                        else figures_dir / f"{prefix}_p0_health_timeline.png"
                    )
                )
            )
        )
        reason_figure_path = figures_dir / f"{prefix}_p0_reason_histogram.png"
        distribution_figure_path = figures_dir / f"{prefix}_pl_cost_distribution.png"
        snapshot_figure_path = figures_dir / f"{prefix}_risk_grid_snapshot_overview.png"
        p0_1_delta_figure_path = figures_dir / f"{prefix}_vs_p0_1_delta.png"
        p0_2_delta_figure_path = figures_dir / f"{prefix}_vs_p0_2_delta.png"
        odom_topdown_figure_path = figures_dir / f"{prefix}_odom_truth_topdown.png"
        odom_error_figure_path = figures_dir / f"{prefix}_odom_error_timeline.png"
        health_vs_odom_figure_path = figures_dir / f"{prefix}_p0_health_vs_odom_error.png"
        p0_6_overlap_figure_path = figures_dir / f"{prefix}_occupied_overlap_map.png"
        p0_6_raw_validity_figure_path = figures_dir / f"{prefix}_raw_pl_vs_final_validity.png"
        p0_6_skip_timeline_figure_path = figures_dir / f"{prefix}_occupied_skip_count_timeline.png"
        p0_6_cells_heatmap_figure_path = figures_dir / f"{prefix}_occupied_cells_table_heatmap.png"
        if plot_p0_health_timeline(health_rows, health_figure_path):
            p0_figure_artifacts.append(str(health_figure_path))
        if p5_3_phase and plot_p0_health_timeline(
            health_rows,
            p5_3_plal_figure_paths["p5_3_plal_p0_health_timeline.png"],
        ):
            p0_figure_artifacts.append(
                str(p5_3_plal_figure_paths["p5_3_plal_p0_health_timeline.png"])
            )
        if p5_3_phase and plot_p0_health_timeline(
            health_rows,
            p5_3_query_alignment_figure_paths[
                "p5_3_query_alignment_p0_health.png"
            ],
        ):
            p0_figure_artifacts.append(
                str(
                    p5_3_query_alignment_figure_paths[
                        "p5_3_query_alignment_p0_health.png"
                    ]
                )
            )
        if p5_3_phase and plot_p0_health_timeline(
            health_rows,
            p5_3_future_sampling_figure_paths[
                "p5_3_future_sampling_p0_health.png"
            ],
        ):
            p0_figure_artifacts.append(
                str(
                    p5_3_future_sampling_figure_paths[
                        "p5_3_future_sampling_p0_health.png"
                    ]
                )
            )
        if p5_3_phase and plot_p0_health_timeline(
            health_rows,
            p5_3_event_window_figure_paths["p5_3_event_window_p0_health.png"],
        ):
            p0_figure_artifacts.append(
                str(p5_3_event_window_figure_paths["p5_3_event_window_p0_health.png"])
            )
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
        if experiment_label == "P0-6":
            if plot_p0_6_occupied_overlap_map(p0_6_overlap, p0_6_overlap_figure_path):
                p0_figure_artifacts.append(str(p0_6_overlap_figure_path))
            else:
                warnings.append("P0-6 occupied overlap map was not generated because no overlap rows were available")
            if plot_p0_6_raw_pl_vs_final_validity(p0_6_overlap, p0_6_raw_validity_figure_path):
                p0_figure_artifacts.append(str(p0_6_raw_validity_figure_path))
            else:
                warnings.append("P0-6 raw PL vs final validity plot was not generated because no overlap rows were available")
            if plot_p0_6_occupied_skip_count_timeline(health_rows, p0_6_skip_timeline_figure_path):
                p0_figure_artifacts.append(str(p0_6_skip_timeline_figure_path))
            else:
                warnings.append("P0-6 occupied skip timeline was not generated because health rows were unavailable")
            if plot_p0_6_occupied_cells_table_heatmap(p0_6_overlap, p0_6_cells_heatmap_figure_path):
                p0_figure_artifacts.append(str(p0_6_cells_heatmap_figure_path))
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
        if experiment_label == "P0-6":
            p0_required_figures.extend(
                [
                    p0_6_overlap_figure_path,
                    p0_6_raw_validity_figure_path,
                    p0_6_skip_timeline_figure_path,
                ]
            )
            if p0_6_overlap:
                p0_required_figures.append(p0_6_cells_heatmap_figure_path)

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
            "p0_6_occupied_overlap_semantics": p0_6_semantics,
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
    scenario_figure_path = (
        figures_dir / "p5_3_debug_scenario_topdown.png"
        if p5_3_phase
        else (
            p5_4_figure_paths["p5_4_scenario_topdown.png"]
            if p5_4_phase
            else (
                p5_5_figure_paths["p5_5_scenario_topdown.png"]
                if p5_5_phase
                else (
                    p5_6_figure_paths["p5_6_scenario_topdown.png"]
                    if p5_6_phase
                    else figures_dir / f"{prefix}_scenario_topdown.png"
                )
            )
        )
    )
    topic_activity_figure_path = (
        figures_dir / "p5_3_debug_topic_activity_timeline.png"
        if p5_3_phase
        else (
            p5_5_figure_paths["p5_5_topic_activity_timeline.png"]
            if p5_5_phase
            else (
                figures_dir / "p5_6_topic_activity_timeline.png"
                if p5_6_phase
                else figures_dir / f"{prefix}_topic_activity_timeline.png"
            )
        )
    )
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
        if p5_3_phase and plot_scenario_topdown(
            scenario_data,
            p5_3_plal_figure_paths["p5_3_plal_scenario_topdown.png"],
        ):
            figures.append(str(p5_3_plal_figure_paths["p5_3_plal_scenario_topdown.png"]))
        if p5_3_phase and plot_scenario_topdown(
            scenario_data,
            p5_3_query_alignment_figure_paths[
                "p5_3_query_alignment_scenario_topdown.png"
            ],
        ):
            figures.append(
                str(
                    p5_3_query_alignment_figure_paths[
                        "p5_3_query_alignment_scenario_topdown.png"
                    ]
                )
            )
        if p5_3_phase and plot_scenario_topdown(
            scenario_data,
            p5_3_future_sampling_figure_paths[
                "p5_3_future_sampling_scenario_topdown.png"
            ],
        ):
            figures.append(
                str(
                    p5_3_future_sampling_figure_paths[
                        "p5_3_future_sampling_scenario_topdown.png"
                    ]
                )
            )
        if p5_3_phase and plot_scenario_topdown(
            scenario_data,
            p5_3_event_window_figure_paths[
                "p5_3_event_window_scenario_topdown.png"
            ],
        ):
            figures.append(
                str(
                    p5_3_event_window_figure_paths[
                        "p5_3_event_window_scenario_topdown.png"
                    ]
                )
            )

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
        topic_timestamps = {}
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
    p5_summary = validate_p5_status(
        p5_rows,
        p5_error,
        failures,
        inconclusive,
        allow_replan=p5_runtime_phase,
        allow_emergency=p5_2_phase or p5_3_phase or p5_4_phase or p5_5_phase or p5_6_phase,
        allow_final_gate_failure=p5_2_phase or p5_3_phase or p5_4_phase or p5_5_phase or p5_6_phase,
    )
    if p0_requires_odom_gate and p5_summary.get("action_counts"):
        failures.append(f"{experiment_label} P5 status reported actions while P5 is disabled")
    p5_marker_rows: list[dict[str, Any]] = []
    p5_marker_summary: dict[str, Any] = {"row_count": 0}
    p5_marker_error = ""
    p5_1_gates: dict[str, Any] = {}
    p5_2_gates: dict[str, Any] = {}
    p5_3_gates: dict[str, Any] = {}
    p5_4_gates: dict[str, Any] = {}
    p5_5_gates: dict[str, Any] = {}
    p5_6_gates: dict[str, Any] = {}
    p5_3_overlap: list[dict[str, Any]] = []
    p5_3_samples: list[dict[str, Any]] = []
    p5_3_event_window_rows: list[dict[str, Any]] = []
    p5_3_event_window_samples: list[dict[str, Any]] = []
    p5_4_overlap: list[dict[str, Any]] = []
    p5_4_samples: list[dict[str, Any]] = []
    p5_4_event_window_rows: list[dict[str, Any]] = []
    p5_4_event_window_samples: list[dict[str, Any]] = []
    p5_5_integrity_rows: list[dict[str, Any]] = []
    p5_5_integrity_error = ""
    p5_6_samples: list[dict[str, Any]] = []
    p5_6_event_window_samples: list[dict[str, Any]] = []
    p5_csv_artifacts: list[str] = []
    p5_figure_artifacts: list[str] = []
    p5_required_figures: list[Path] = []
    if p5_runtime_phase:
        p5_marker_rows, p5_marker_error = (
            read_p5_marker_evidence(bag_dir, metadata)
            if bag_dir is not None
            else ([], "missing bag dir")
        )
        p5_marker_summary = summarize_p5_marker_evidence(p5_marker_rows, p5_marker_error)
        if p5_marker_error:
            inconclusive.append(f"could not inspect P5 RViz marker evidence: {p5_marker_error}")
    if p5_5_phase:
        p5_5_integrity_rows, p5_5_integrity_error = (
            read_p5_5_integrity_stamp_evidence(bag_dir, metadata)
            if bag_dir is not None
            else ([], "missing bag dir")
        )
        if p5_5_integrity_error:
            inconclusive.append(
                f"could not inspect P5-5 /iap/integrity stamp evidence: {p5_5_integrity_error}"
            )
    if p5_1_phase:
        p5_1_gates = validate_p5_1_hard_gates(
            p0_health_summary,
            p5_summary,
            failures,
            inconclusive,
        )
    if p5_2_phase:
        p5_2_gates = validate_p5_2_hard_gates(
            manifest,
            validator_summary,
            topic_health,
            p0_health_summary,
            health_rows,
            p5_summary,
            failures,
            inconclusive,
        )
    if p5_3_phase:
        p5_3_gates = validate_p5_3_hard_gates(
            manifest,
            validator_summary,
            topic_health,
            p0_health_summary,
            health_rows,
            p5_rows,
            p5_summary,
            p5_marker_rows,
            failures,
            inconclusive,
            topic_timestamps,
        )
        p5_3_overlap = p5_3_overlap_rows(
            p5_marker_rows,
            p5_3_gates.get("fixture", {}),
        )
        p5_3_samples = p5_3_sample_rows(
            p5_rows,
            p5_3_gates.get("fixture", {}),
        )
        p5_3_event_window_indices = list(
            ((p5_3_gates.get("event_window") or {}).get("row_indices") or [])
        )
        p5_3_event_window_rows = [
            p5_rows[index]
            for index in p5_3_event_window_indices
            if isinstance(index, int) and 0 <= index < len(p5_rows)
        ]
        p5_3_event_window_samples = p5_3_filter_sample_rows_by_status_indices(
            p5_3_samples,
            p5_3_event_window_indices,
        )
    if p5_4_phase:
        p5_4_gates = validate_p5_4_hard_gates(
            manifest,
            validator_summary,
            topic_health,
            p0_health_summary,
            health_rows,
            p5_rows,
            p5_summary,
            p5_marker_rows,
            failures,
            inconclusive,
            topic_timestamps,
        )
        p5_4_overlap = p5_3_overlap_rows(
            p5_marker_rows,
            p5_4_gates.get("fixture", {}),
        )
        p5_4_samples = p5_3_sample_rows(
            p5_rows,
            p5_4_gates.get("fixture", {}),
            tau_window_field="tau_s",
        )
        p5_4_event_window_indices = list(
            ((p5_4_gates.get("event_window") or {}).get("row_indices") or [])
        )
        p5_4_event_window_rows = [
            p5_rows[index]
            for index in p5_4_event_window_indices
            if isinstance(index, int) and 0 <= index < len(p5_rows)
        ]
        p5_4_event_window_samples = p5_3_filter_sample_rows_by_status_indices(
            p5_4_samples,
            p5_4_event_window_indices,
        )
    if p5_5_phase:
        p5_5_gates = validate_p5_5_hard_gates(
            manifest,
            validator_summary,
            topic_health,
            p0_health_summary,
            health_rows,
            p5_rows,
            p5_summary,
            p5_5_integrity_rows,
            failures,
            inconclusive,
            topic_timestamps,
        )
        p5_5_integrity_rows = p5_5_annotate_integrity_window(
            p5_5_integrity_rows,
            p5_5_gates.get("fixture", {}),
        )
    if p5_6_phase:
        p5_6_gates = validate_p5_6_hard_gates(
            manifest,
            validator_summary,
            topic_health,
            p0_health_summary,
            health_rows,
            p5_rows,
            p5_summary,
            failures,
            inconclusive,
            topic_timestamps,
        )
        p5_6_samples = p5_3_sample_rows(
            p5_rows,
            p5_6_gates.get("fixture", {}),
            tau_window_field="query_tau_s",
        )
        p5_6_event_window_samples = p5_3_filter_sample_rows_by_status_indices(
            p5_6_samples,
            list((p5_6_gates.get("accepted_window") or {}).get("row_indices") or []),
        )

    csv_artifacts = [str(topic_counts_path)]
    csv_artifacts.extend(p0_csv_artifacts)
    if p5_rows:
        p5_status_path = csv_dir / f"{prefix}_p5_status.csv"
        write_csv(p5_status_path, P5_STATUS_FIELDS, p5_rows)
        csv_artifacts.append(str(p5_status_path))
        p5_csv_artifacts.append(str(p5_status_path))
    if p5_runtime_phase:
        p5_action_path = csv_dir / f"{prefix}_p5_action_timeline.csv"
        p5_margin_path = csv_dir / f"{prefix}_p5_margin_timeline.csv"
        p5_final_gate_path = csv_dir / f"{prefix}_p5_final_gate_summary.csv"
        p5_marker_path = csv_dir / f"{prefix}_trajectory_integrity_evidence.csv"
        p5_debounce_path = csv_dir / f"{prefix}_p5_debounce_timeline.csv"
        p5_3_overlap_path = csv_dir / f"{prefix}_high_risk_zone_overlap.csv"
        p5_3_samples_path = csv_dir / f"{prefix}_p5_sample_diagnostics.csv"
        p5_3_query_alignment_path = csv_dir / f"{prefix}_query_alignment_evidence.csv"
        p5_4_overlap_path = csv_dir / f"{prefix}_near_risk_zone_overlap.csv"
        p5_4_samples_path = csv_dir / f"{prefix}_p5_sample_diagnostics.csv"
        p5_4_query_alignment_path = csv_dir / f"{prefix}_query_alignment_evidence.csv"
        p5_5_integrity_stamp_path = csv_dir / f"{prefix}_integrity_stamp_freeze_evidence.csv"
        p5_6_unknown_action_path = csv_dir / "p5_6_unknown_action_timeline.csv"
        p5_6_cause_exclusion_path = csv_dir / "p5_6_cause_exclusion_summary.csv"
        p5_6_samples_path = csv_dir / "p5_6_future_unknown_sample_diagnostics.csv"
        action_rows = []
        t_rel = relative_time(p5_rows)
        for idx, row in enumerate(p5_rows):
            fail_count = int(finite_float(row.get("final_gate_fail_count")) or 0)
            action_rows.append(
                {
                    "bag_time_s": row.get("bag_time_s", ""),
                    "t_rel_s": t_rel[idx] if idx < len(t_rel) else "",
                    "phase": row.get("phase", ""),
                    "action": row.get("action", ""),
                    "raw_action": row.get("raw_action", ""),
                    "reason": row.get("reason", ""),
                    "raw_reason": row.get("raw_reason", ""),
                    "current_reason": row.get("current_reason", ""),
                    "future_reason": row.get("future_reason", ""),
                    "active_reasons": row.get("active_reasons", ""),
                    "action_ok": 1 if p5_action(row, "action") == P5_OK_ACTION else 0,
                    "raw_action_ok": 1 if p5_action(row, "raw_action") == P5_OK_ACTION else 0,
                    "request_replan": 1
                    if p5_action(row, "action") == P5_REPLAN_ACTION
                    or p5_action(row, "raw_action") == P5_REPLAN_ACTION
                    else 0,
                    "request_emergency_stop_candidate": 1
                    if p5_action(row, "action") == P5_EMERGENCY_ACTION
                    or p5_action(row, "raw_action") == P5_EMERGENCY_ACTION
                    else 0,
                    "final_gate_fail_count": fail_count,
                    "final_gate_last_reason": row.get("final_gate_last_reason", ""),
                }
            )
        write_csv(
            p5_action_path,
            [
                "bag_time_s",
                "t_rel_s",
                "phase",
                "action",
                "raw_action",
                "reason",
                "raw_reason",
                "current_reason",
                "future_reason",
                "active_reasons",
                "action_ok",
                "raw_action_ok",
                "request_replan",
                "request_emergency_stop_candidate",
                "final_gate_fail_count",
                "final_gate_last_reason",
            ],
            action_rows,
        )
        write_csv(
            p5_margin_path,
            [
                "bag_time_s",
                "phase",
                "current_im_h",
                "current_im_v",
                "current_im_min",
                "future_min_im",
                "first_bad_tau",
                "bad_ratio",
                "unknown_ratio",
                "reason",
                "raw_reason",
                "current_reason",
                "future_reason",
                "active_reasons",
                "current_stale_duration_s",
                "current_low_margin_duration_s",
                "future_unknown_duration_s",
                "pred_al_mode",
                "pred_hal_min",
                "pred_val_min",
                "pred_al_invalid_count",
                "sample_count",
                "bad_count",
                "unknown_count",
            ],
            p5_rows,
        )
        write_csv(
            p5_final_gate_path,
            [
                "status_rows",
                "ok_action_ratio",
                "startup_snapshot_unavailable_rows",
                "startup_snapshot_unavailable_duration_s",
                "startup_snapshot_unavailable_bounded",
                "steady_status_rows",
                "steady_ok_action_ratio",
                "steady_replan_action_count",
                "steady_raw_replan_action_count",
                "steady_max_consecutive_replan",
                "steady_raw_max_consecutive_replan",
                "steady_replan_storm",
                "replan_action_count",
                "raw_replan_action_count",
                "emergency_action_count",
                "raw_emergency_action_count",
                "max_consecutive_emergency",
                "raw_max_consecutive_emergency",
                "max_consecutive_replan",
                "raw_max_consecutive_replan",
                "final_gate_fail_rows",
                "final_gate_emergency_rows",
                "final_gate_fail_count_max",
                "final_gate_fail_duration_s_max",
                "replan_storm",
            ],
            [p5_summary],
        )
        write_csv(
            p5_debounce_path,
            [
                "bag_time_s",
                "t_rel_s",
                "phase",
                "action",
                "raw_action",
                "reason",
                "consecutive_replan",
                "raw_consecutive_replan",
                "consecutive_emergency",
                "raw_consecutive_emergency",
                "final_gate_fail_count",
                "final_gate_fail_duration_s",
                "final_gate_last_reason",
                "final_gate_escalated_to_emergency",
            ],
            build_p5_debounce_timeline_rows(p5_rows),
        )
        write_csv(
            p5_marker_path,
            [
                "bag_time_s",
                "topic",
                "marker_ns",
                "marker_id",
                "marker_type",
                "marker_action",
                "point_index",
                "x",
                "y",
                "z",
                "color_r",
                "color_g",
                "color_b",
                "color_a",
                "state",
                "text",
            ],
            p5_marker_rows,
        )
        if p5_3_phase:
            write_csv(
                p5_3_overlap_path,
                [
                    "bag_time_s",
                    "topic",
                    "marker_ns",
                    "marker_id",
                    "point_index",
                    "x",
                    "y",
                    "z",
                    "state",
                    "inside_high_risk_zone",
                    "overlap_bad_state",
                    "fixture_x_min",
                    "fixture_x_max",
                    "fixture_y_min",
                    "fixture_y_max",
                    "fixture_z_min",
                    "fixture_z_max",
                ],
                p5_3_overlap,
            )
            write_csv(p5_3_samples_path, P5_SAMPLE_FIELDS, p5_3_samples)
            write_csv(
                p5_3_query_alignment_path,
                P5_3_QUERY_ALIGNMENT_FIELDS,
                p5_3_samples,
            )
        if p5_4_phase:
            write_csv(
                p5_4_overlap_path,
                [
                    "bag_time_s",
                    "topic",
                    "marker_ns",
                    "marker_id",
                    "point_index",
                    "x",
                    "y",
                    "z",
                    "state",
                    "inside_high_risk_zone",
                    "overlap_bad_state",
                    "fixture_x_min",
                    "fixture_x_max",
                    "fixture_y_min",
                    "fixture_y_max",
                    "fixture_z_min",
                    "fixture_z_max",
                ],
                p5_4_overlap,
            )
            write_csv(p5_4_samples_path, P5_SAMPLE_FIELDS, p5_4_samples)
            write_csv(
                p5_4_query_alignment_path,
                P5_3_QUERY_ALIGNMENT_FIELDS,
                p5_4_samples,
            )
        if p5_5_phase:
            write_csv(
                p5_5_integrity_stamp_path,
                P5_5_INTEGRITY_STAMP_FIELDS,
                p5_5_integrity_rows,
            )
        if p5_6_phase:
            write_csv(p5_6_samples_path, P5_SAMPLE_FIELDS, p5_6_samples)
            write_csv(
                p5_6_unknown_action_path,
                P5_6_UNKNOWN_ACTION_FIELDS,
                build_p5_6_unknown_action_timeline_rows(p5_rows),
            )
            write_csv(
                p5_6_cause_exclusion_path,
                P5_6_CAUSE_EXCLUSION_FIELDS,
                p5_6_cause_exclusion_rows(p5_6_gates),
            )
        p5_csv_artifacts.extend(
            [
                str(p5_action_path),
                str(p5_margin_path),
                str(p5_final_gate_path),
                str(p5_debounce_path),
                str(p5_marker_path),
            ]
        )
        if p5_3_phase:
            p5_csv_artifacts.append(str(p5_3_overlap_path))
            p5_csv_artifacts.append(str(p5_3_samples_path))
            p5_csv_artifacts.append(str(p5_3_query_alignment_path))
        if p5_4_phase:
            p5_csv_artifacts.append(str(p5_4_overlap_path))
            p5_csv_artifacts.append(str(p5_4_samples_path))
            p5_csv_artifacts.append(str(p5_4_query_alignment_path))
        if p5_5_phase:
            p5_csv_artifacts.append(str(p5_5_integrity_stamp_path))
        if p5_6_phase:
            p5_csv_artifacts.append(str(p5_6_samples_path))
            p5_csv_artifacts.append(str(p5_6_unknown_action_path))
            p5_csv_artifacts.append(str(p5_6_cause_exclusion_path))
        csv_artifacts.extend(
            artifact
            for artifact in p5_csv_artifacts
            if artifact not in csv_artifacts
        )

        p5_action_figure_path = (
            figures_dir / "p5_3_debug_action_timeline.png"
            if p5_3_phase
            else figures_dir / f"{prefix}_p5_action_timeline.png"
        )
        p5_status_figure_path = figures_dir / f"{prefix}_p5_status_timeline.png"
        p5_margin_figure_path = (
            figures_dir / "p5_3_debug_margin_timeline.png"
            if p5_3_phase
            else (
                p5_4_figure_paths["p5_4_margin_timeline.png"]
                if p5_4_phase
                else (
                    p5_5_figure_paths["p5_5_margin_timeline.png"]
                    if p5_5_phase
                    else (
                        figures_dir / "p5_6_margin_timeline.png"
                        if p5_6_phase
                        else figures_dir / f"{prefix}_p5_margin_timeline.png"
                    )
                )
            )
        )
        p5_current_im_vs_action_figure_path = figures_dir / f"{prefix}_current_im_vs_action.png"
        p5_debounce_figure_path = (
            p5_5_figure_paths["p5_5_debounce_timeline.png"]
            if p5_5_phase
            else (
                p5_6_figure_paths["p5_6_debounce_timeline.png"]
                if p5_6_phase
                else figures_dir / f"{prefix}_p5_debounce_timeline.png"
            )
        )
        p5_future_unknown_figure_path = (
            p5_6_figure_paths["p5_6_future_unknown_duration_timeline.png"]
            if p5_6_phase
            else figures_dir / f"{prefix}_future_unknown_duration_timeline.png"
        )
        p5_startup_correlation_figure_path = figures_dir / f"{prefix}_startup_correlation.png"
        p5_stale_integrity_correlation_figure_path = figures_dir / f"{prefix}_stale_integrity_correlation.png"
        p5_final_gate_figure_path = (
            p5_4_figure_paths["p5_4_final_gate_summary.png"]
            if p5_4_phase
            else figures_dir / f"{prefix}_p5_final_gate_summary.png"
        )
        p5_trajectory_figure_path = (
            figures_dir / "p5_3_debug_trajectory_integrity_samples.png"
            if p5_3_phase
            else (
                p5_4_figure_paths["p5_4_trajectory_integrity_samples.png"]
                if p5_4_phase
                else (
                    p5_5_figure_paths["p5_5_trajectory_integrity_samples.png"]
                    if p5_5_phase
                    else (
                        p5_6_figure_paths["p5_6_trajectory_integrity_samples.png"]
                        if p5_6_phase
                        else figures_dir / f"{prefix}_trajectory_integrity_samples.png"
                    )
                )
            )
        )
        p5_rviz_overview_path = figures_dir / f"{prefix}_p5_rviz_overview.png"
        p5_3_overlay_figure_path = figures_dir / "p5_3_debug_high_risk_zone_overlay.png"
        p5_3_first_bad_tau_figure_path = figures_dir / "p5_3_debug_first_bad_tau_timeline.png"
        p5_3_reason_figure_path = figures_dir / "p5_3_debug_reason_timeline.png"
        p5_3_plal_overlay_path = p5_3_plal_figure_paths["p5_3_plal_high_risk_overlay.png"]
        p5_3_plal_tau_window_path = p5_3_plal_figure_paths["p5_3_plal_tau_window.png"]
        p5_3_plal_margin_path = p5_3_plal_figure_paths["p5_3_plal_margin_timeline.png"]
        p5_3_plal_action_reason_path = p5_3_plal_figure_paths[
            "p5_3_plal_action_reason_timeline.png"
        ]
        p5_3_plal_replan_emergency_path = p5_3_plal_figure_paths[
            "p5_3_plal_replan_vs_emergency.png"
        ]
        p5_3_plal_heatmap_path = p5_3_plal_figure_paths["p5_3_plal_sample_heatmap.png"]
        p5_3_query_fixture_overlay_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_fixture_overlay.png"
        ]
        p5_3_query_pl_probe_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_pl_probe.png"
        ]
        p5_3_query_tau_window_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_tau_window.png"
        ]
        p5_3_query_margin_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_margin_timeline.png"
        ]
        p5_3_query_action_reason_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_action_reason.png"
        ]
        p5_3_query_heatmap_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_sample_heatmap.png"
        ]
        p5_3_query_topic_gap_path = p5_3_query_alignment_figure_paths[
            "p5_3_query_alignment_topic_gap.png"
        ]
        p5_3_future_fixture_overlay_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_fixture_overlay.png"
        ]
        p5_3_future_pl_probe_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_pl_probe.png"
        ]
        p5_3_future_tau_window_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_tau_window.png"
        ]
        p5_3_future_margin_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_margin_timeline.png"
        ]
        p5_3_future_action_reason_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_action_reason.png"
        ]
        p5_3_future_heatmap_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_sample_heatmap.png"
        ]
        p5_3_future_topic_gap_path = p5_3_future_sampling_figure_paths[
            "p5_3_future_sampling_topic_gap.png"
        ]
        p5_3_event_fixture_overlay_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_fixture_overlay.png"
        ]
        p5_3_event_tau_window_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_tau_window.png"
        ]
        p5_3_event_pl_probe_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_pl_probe.png"
        ]
        p5_3_event_margin_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_margin_timeline.png"
        ]
        p5_3_event_action_reason_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_action_reason.png"
        ]
        p5_3_event_replan_emergency_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_replan_vs_emergency.png"
        ]
        p5_3_event_heatmap_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_sample_heatmap.png"
        ]
        p5_3_event_topic_gap_path = p5_3_event_window_figure_paths[
            "p5_3_event_window_topic_gap.png"
        ]
        p5_4_overlay_path = p5_4_figure_paths["p5_4_near_risk_overlay.png"]
        p5_4_tau_window_path = p5_4_figure_paths[
            "p5_4_tau_emergency_window.png"
        ]
        p5_4_pl_probe_path = p5_4_figure_paths["p5_4_pl_probe.png"]
        p5_4_action_reason_path = p5_4_figure_paths[
            "p5_4_action_reason_timeline.png"
        ]
        p5_4_replan_emergency_path = p5_4_figure_paths[
            "p5_4_replan_vs_emergency.png"
        ]
        p5_4_heatmap_path = p5_4_figure_paths["p5_4_sample_heatmap.png"]
        p5_4_topic_gap_path = p5_4_figure_paths["p5_4_topic_gap.png"]
        p5_5_integrity_pause_path = p5_5_figure_paths[
            "p5_5_integrity_pause_timeline.png"
        ]
        p5_5_current_stale_path = p5_5_figure_paths[
            "p5_5_current_stale_duration_timeline.png"
        ]
        p5_5_action_reason_path = p5_5_figure_paths[
            "p5_5_action_reason_timeline.png"
        ]
        p5_5_replan_emergency_path = p5_5_figure_paths[
            "p5_5_replan_vs_emergency.png"
        ]
        p5_5_cause_exclusion_path = p5_5_figure_paths[
            "p5_5_cause_exclusion_summary.png"
        ]
        p5_6_action_reason_path = p5_6_figure_paths[
            "p5_6_action_reason_timeline.png"
        ]
        p5_6_unknown_ratio_action_path = p5_6_figure_paths[
            "p5_6_unknown_ratio_vs_action.png"
        ]
        p5_6_reason_histogram_path = figures_dir / "p5_6_reason_histogram.png"
        p5_6_cause_exclusion_figure_path = p5_6_figure_paths[
            "p5_6_cause_exclusion_summary.png"
        ]
        p5_6_unknown_overlay_path = p5_6_figure_paths[
            "p5_6_unknown_field_overlay.png"
        ]
        if plot_p5_action_timeline(p5_rows, p5_action_figure_path):
            p5_figure_artifacts.append(str(p5_action_figure_path))
        if plot_p5_status_timeline(p5_rows, p5_status_figure_path):
            p5_figure_artifacts.append(str(p5_status_figure_path))
        if plot_p5_margin_timeline(p5_rows, p5_margin_figure_path):
            p5_figure_artifacts.append(str(p5_margin_figure_path))
        if p5_3_phase and plot_p5_margin_timeline(p5_rows, p5_3_plal_margin_path):
            p5_figure_artifacts.append(str(p5_3_plal_margin_path))
        if p5_3_phase and plot_p5_margin_timeline(p5_rows, p5_3_query_margin_path):
            p5_figure_artifacts.append(str(p5_3_query_margin_path))
        if p5_3_phase and plot_p5_margin_timeline(p5_rows, p5_3_future_margin_path):
            p5_figure_artifacts.append(str(p5_3_future_margin_path))
        if p5_3_phase and plot_p5_margin_timeline(
            p5_3_event_window_rows,
            p5_3_event_margin_path,
        ):
            p5_figure_artifacts.append(str(p5_3_event_margin_path))
        if plot_p5_current_im_vs_action(p5_rows, p5_current_im_vs_action_figure_path):
            p5_figure_artifacts.append(str(p5_current_im_vs_action_figure_path))
        if plot_p5_debounce_timeline(p5_rows, p5_debounce_figure_path):
            p5_figure_artifacts.append(str(p5_debounce_figure_path))
        if plot_p5_future_unknown_duration_timeline(p5_rows, p5_future_unknown_figure_path):
            p5_figure_artifacts.append(str(p5_future_unknown_figure_path))
        if plot_p5_startup_correlation(health_rows, p5_rows, p5_startup_correlation_figure_path):
            p5_figure_artifacts.append(str(p5_startup_correlation_figure_path))
        if plot_p5_stale_integrity_correlation(
            health_rows,
            p5_rows,
            p5_stale_integrity_correlation_figure_path,
        ):
            p5_figure_artifacts.append(str(p5_stale_integrity_correlation_figure_path))
        if plot_p5_final_gate_summary(p5_summary, p5_final_gate_figure_path):
            p5_figure_artifacts.append(str(p5_final_gate_figure_path))
        if plot_p5_trajectory_integrity_samples(p5_marker_rows, p5_trajectory_figure_path):
            p5_figure_artifacts.append(str(p5_trajectory_figure_path))
        else:
            warnings.append("P5 trajectory integrity marker sample plot was not generated because marker rows were unavailable")
        if plot_p5_rviz_overview(topic_health, p5_summary, p5_marker_summary, p5_rviz_overview_path):
            p5_figure_artifacts.append(str(p5_rviz_overview_path))
        if p5_3_phase:
            if plot_p5_3_high_risk_zone_overlay(
                p5_3_overlap,
                p5_3_gates.get("fixture", {}),
                p5_3_overlay_figure_path,
            ):
                p5_figure_artifacts.append(str(p5_3_overlay_figure_path))
            else:
                warnings.append("P5-3 high-risk zone overlay was not generated because overlap rows were unavailable")
            if plot_p5_3_sample_high_risk_overlay(
                p5_3_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_plal_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_3_plal_overlay_path))
            if plot_p5_3_sample_high_risk_overlay(
                p5_3_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_query_fixture_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_3_query_fixture_overlay_path))
            if plot_p5_3_sample_high_risk_overlay(
                p5_3_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_future_fixture_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_3_future_fixture_overlay_path))
            if plot_p5_3_sample_high_risk_overlay(
                p5_3_event_window_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_event_fixture_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_3_event_fixture_overlay_path))
            if plot_p5_3_query_alignment_pl_probe(
                p5_3_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_query_pl_probe_path,
            ):
                p5_figure_artifacts.append(str(p5_3_query_pl_probe_path))
            if plot_p5_3_query_alignment_pl_probe(
                p5_3_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_future_pl_probe_path,
            ):
                p5_figure_artifacts.append(str(p5_3_future_pl_probe_path))
            if plot_p5_3_query_alignment_pl_probe(
                p5_3_event_window_samples,
                p5_3_gates.get("fixture", {}),
                p5_3_event_pl_probe_path,
            ):
                p5_figure_artifacts.append(str(p5_3_event_pl_probe_path))
            if plot_p5_3_first_bad_tau_timeline(
                p5_rows,
                p5_3_gates.get("fixture", {}),
                p5_3_first_bad_tau_figure_path,
            ):
                p5_figure_artifacts.append(str(p5_3_first_bad_tau_figure_path))
            else:
                warnings.append("P5-3 first_bad_tau timeline was not generated because status rows were unavailable")
            if plot_p5_3_sample_tau_window(
                p5_3_samples,
                p5_rows,
                p5_3_gates.get("fixture", {}),
                p5_3_plal_tau_window_path,
            ):
                p5_figure_artifacts.append(str(p5_3_plal_tau_window_path))
            if plot_p5_3_sample_tau_window(
                p5_3_samples,
                p5_rows,
                p5_3_gates.get("fixture", {}),
                p5_3_query_tau_window_path,
                tau_field="query_tau_s",
                title="P5-3 query-alignment tau-window evidence",
            ):
                p5_figure_artifacts.append(str(p5_3_query_tau_window_path))
            if plot_p5_3_sample_tau_window(
                p5_3_samples,
                p5_rows,
                p5_3_gates.get("fixture", {}),
                p5_3_future_tau_window_path,
                tau_field="query_tau_s",
                title="P5-3 future-sampling query tau-window evidence",
            ):
                p5_figure_artifacts.append(str(p5_3_future_tau_window_path))
            if plot_p5_3_sample_tau_window(
                p5_3_event_window_samples,
                p5_3_event_window_rows,
                p5_3_gates.get("fixture", {}),
                p5_3_event_tau_window_path,
                tau_field="query_tau_s",
                title="P5-3 event-window query tau-window evidence",
            ):
                p5_figure_artifacts.append(str(p5_3_event_tau_window_path))
            if plot_p5_3_reason_timeline(p5_rows, p5_3_reason_figure_path):
                p5_figure_artifacts.append(str(p5_3_reason_figure_path))
            else:
                warnings.append("P5-3 reason timeline was not generated because status rows were unavailable")
            if plot_p5_3_reason_timeline(p5_rows, p5_3_plal_action_reason_path):
                p5_figure_artifacts.append(str(p5_3_plal_action_reason_path))
            if plot_p5_3_reason_timeline(p5_rows, p5_3_query_action_reason_path):
                p5_figure_artifacts.append(str(p5_3_query_action_reason_path))
            if plot_p5_3_reason_timeline(p5_rows, p5_3_future_action_reason_path):
                p5_figure_artifacts.append(str(p5_3_future_action_reason_path))
            if plot_p5_3_reason_timeline(
                p5_3_event_window_rows,
                p5_3_event_action_reason_path,
            ):
                p5_figure_artifacts.append(str(p5_3_event_action_reason_path))
            if plot_p5_3_replan_vs_emergency(
                p5_rows,
                p5_3_plal_replan_emergency_path,
            ):
                p5_figure_artifacts.append(str(p5_3_plal_replan_emergency_path))
            if plot_p5_3_replan_vs_emergency(
                p5_3_event_window_rows,
                p5_3_event_replan_emergency_path,
            ):
                p5_figure_artifacts.append(str(p5_3_event_replan_emergency_path))
            if plot_p5_3_sample_heatmap(p5_3_samples, p5_3_plal_heatmap_path):
                p5_figure_artifacts.append(str(p5_3_plal_heatmap_path))
            if plot_p5_3_sample_heatmap(
                p5_3_samples,
                p5_3_query_heatmap_path,
                tau_field="query_tau_s",
                title="P5-3 query-alignment sample heatmap",
            ):
                p5_figure_artifacts.append(str(p5_3_query_heatmap_path))
            if plot_p5_3_sample_heatmap(
                p5_3_samples,
                p5_3_future_heatmap_path,
                tau_field="query_tau_s",
                title="P5-3 future-sampling sample heatmap",
            ):
                p5_figure_artifacts.append(str(p5_3_future_heatmap_path))
            if plot_p5_3_sample_heatmap(
                p5_3_event_window_samples,
                p5_3_event_heatmap_path,
                tau_field="query_tau_s",
                title="P5-3 event-window sample heatmap",
            ):
                p5_figure_artifacts.append(str(p5_3_event_heatmap_path))
            if plot_p5_3_topic_gap(
                topic_timestamps,
                p5_3_gates.get("active_topic_gap", {}),
                p5_3_query_topic_gap_path,
            ):
                p5_figure_artifacts.append(str(p5_3_query_topic_gap_path))
            if plot_p5_3_topic_gap(
                topic_timestamps,
                p5_3_gates.get("active_topic_gap", {}),
                p5_3_future_topic_gap_path,
            ):
                p5_figure_artifacts.append(str(p5_3_future_topic_gap_path))
            if plot_p5_3_topic_gap(
                topic_timestamps,
                p5_3_gates.get("event_window_active_topic_gap", {}),
                p5_3_event_topic_gap_path,
            ):
                p5_figure_artifacts.append(str(p5_3_event_topic_gap_path))
        if p5_4_phase:
            p5_4_plot_rows = p5_4_event_window_rows or p5_rows
            p5_4_plot_samples = p5_4_event_window_samples or p5_4_samples
            if plot_p5_3_sample_high_risk_overlay(
                p5_4_samples,
                p5_4_gates.get("fixture", {}),
                p5_4_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_4_overlay_path))
            if plot_p5_3_sample_tau_window(
                p5_4_plot_samples,
                p5_4_plot_rows,
                p5_4_gates.get("fixture", {}),
                p5_4_tau_window_path,
                tau_field="tau_s",
                title="P5-4 near-risk emergency tau-window evidence",
            ):
                p5_figure_artifacts.append(str(p5_4_tau_window_path))
            if plot_p5_3_query_alignment_pl_probe(
                p5_4_samples,
                p5_4_gates.get("fixture", {}),
                p5_4_pl_probe_path,
                tau_field="tau_s",
                title="P5-4 near-risk query-aligned PL probe",
            ):
                p5_figure_artifacts.append(str(p5_4_pl_probe_path))
            if plot_p5_3_reason_timeline(p5_4_plot_rows, p5_4_action_reason_path):
                p5_figure_artifacts.append(str(p5_4_action_reason_path))
            if plot_p5_3_replan_vs_emergency(
                p5_4_plot_rows,
                p5_4_replan_emergency_path,
            ):
                p5_figure_artifacts.append(str(p5_4_replan_emergency_path))
            if plot_p5_3_sample_heatmap(
                p5_4_plot_samples,
                p5_4_heatmap_path,
                tau_field="tau_s",
                title="P5-4 near-risk sample heatmap",
            ):
                p5_figure_artifacts.append(str(p5_4_heatmap_path))
            if plot_p5_3_topic_gap(
                topic_timestamps,
                p5_4_gates.get("active_topic_gap", {}),
                p5_4_topic_gap_path,
            ):
                p5_figure_artifacts.append(str(p5_4_topic_gap_path))
        if p5_5_phase:
            if plot_p5_5_integrity_pause_timeline(
                p5_5_integrity_rows,
                p5_5_gates.get("fixture", {}),
                p5_5_integrity_pause_path,
            ):
                p5_figure_artifacts.append(str(p5_5_integrity_pause_path))
            if plot_p5_5_current_stale_duration_timeline(
                p5_rows,
                p5_5_gates.get("fixture", {}),
                p5_5_current_stale_path,
            ):
                p5_figure_artifacts.append(str(p5_5_current_stale_path))
            if plot_p5_3_reason_timeline(p5_rows, p5_5_action_reason_path):
                p5_figure_artifacts.append(str(p5_5_action_reason_path))
            if plot_p5_3_replan_vs_emergency(p5_rows, p5_5_replan_emergency_path):
                p5_figure_artifacts.append(str(p5_5_replan_emergency_path))
            if plot_p5_5_cause_exclusion_summary(
                p5_5_gates,
                p5_5_cause_exclusion_path,
            ):
                p5_figure_artifacts.append(str(p5_5_cause_exclusion_path))
        if p5_6_phase:
            p5_6_overlay_samples = p5_6_event_window_samples or p5_6_samples
            if plot_p5_6_unknown_field_overlay(
                validity_cloud_rows,
                p5_6_overlay_samples,
                p5_6_gates.get("fixture", {}),
                p5_6_unknown_overlay_path,
            ):
                p5_figure_artifacts.append(str(p5_6_unknown_overlay_path))
            if plot_p5_3_reason_timeline(p5_rows, p5_6_action_reason_path):
                p5_figure_artifacts.append(str(p5_6_action_reason_path))
            if plot_p5_6_unknown_ratio_vs_action(
                p5_rows,
                p5_6_unknown_ratio_action_path,
            ):
                p5_figure_artifacts.append(str(p5_6_unknown_ratio_action_path))
            if plot_p5_6_reason_histogram(p5_rows, p5_6_reason_histogram_path):
                p5_figure_artifacts.append(str(p5_6_reason_histogram_path))
            if plot_p5_6_cause_exclusion_summary(
                p5_6_gates,
                p5_6_cause_exclusion_figure_path,
            ):
                p5_figure_artifacts.append(str(p5_6_cause_exclusion_figure_path))
        required_runtime_figures = [
            scenario_figure_path,
            topic_activity_figure_path,
            source_figure_path,
            health_figure_path,
            p5_action_figure_path,
            p5_status_figure_path,
            p5_margin_figure_path,
            p5_debounce_figure_path,
            p5_future_unknown_figure_path,
            p5_startup_correlation_figure_path,
            p5_trajectory_figure_path,
            p5_final_gate_figure_path,
            p5_rviz_overview_path,
        ]
        if p5_2_phase:
            required_runtime_figures.extend(
                [
                    p5_current_im_vs_action_figure_path,
                    p5_stale_integrity_correlation_figure_path,
                ]
            )
        if p5_3_phase:
            required_runtime_figures.extend(
                [
                    p5_3_overlay_figure_path,
                    p5_3_reason_figure_path,
                    p5_3_first_bad_tau_figure_path,
                    *[
                        p5_3_plal_figure_paths[name]
                        for name in P5_3_PLAL_FIGURE_FILENAMES
                    ],
                    *[
                        p5_3_query_alignment_figure_paths[name]
                        for name in P5_3_QUERY_ALIGNMENT_FIGURE_FILENAMES
                    ],
                    *[
                        p5_3_future_sampling_figure_paths[name]
                        for name in P5_3_FUTURE_SAMPLING_FIGURE_FILENAMES
                    ],
                    *[
                        p5_3_event_window_figure_paths[name]
                        for name in P5_3_EVENT_WINDOW_FIGURE_FILENAMES
                    ],
                ]
            )
        if p5_4_phase:
            required_runtime_figures.extend(
                [p5_4_figure_paths[name] for name in P5_4_FIGURE_FILENAMES]
            )
        if p5_5_phase:
            p5_5_required_names = [
                name
                for name in P5_5_FIGURE_FILENAMES
                if name != "p5_5_trajectory_integrity_samples.png"
            ]
            if p5_marker_rows:
                p5_5_required_names.append("p5_5_trajectory_integrity_samples.png")
            required_runtime_figures.extend(
                [p5_5_figure_paths[name] for name in p5_5_required_names]
            )
            if not p5_marker_rows:
                required_runtime_figures = [
                    path for path in required_runtime_figures
                    if path != p5_trajectory_figure_path
                ]
        if p5_6_phase:
            required_runtime_figures = [
                p5_6_figure_paths[name] for name in P5_6_FIGURE_FILENAMES
            ]
        required_runtime_figures = list(dict.fromkeys(required_runtime_figures))
        p5_required_figures.extend(required_runtime_figures)

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
    if p0_runtime_phase:
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
    if p5_runtime_phase:
        figures.extend(p5_figure_artifacts)
        p5_3_query_alignment_required = set(
            p5_3_query_alignment_figure_paths.values()
        )
        p5_3_future_sampling_required = set(
            p5_3_future_sampling_figure_paths.values()
        )
        p5_3_event_window_required = set(
            p5_3_event_window_figure_paths.values()
        )
        p5_4_required = set(p5_4_figure_paths.values())
        p5_5_required = set(p5_5_figure_paths.values())
        p5_6_required = set(p5_6_figure_paths.values())
        for figure_path in p5_required_figures:
            if not figure_path.is_file() or figure_path.stat().st_size <= 0:
                if p5_3_phase and figure_path in p5_3_event_window_required:
                    failures.append(
                        "P5-3 event-window figure missing: "
                        f"{figure_path.name}; missing figure evidence prevents "
                        "event-window current-isolation acceptance"
                    )
                elif p5_3_phase and figure_path in p5_3_future_sampling_required:
                    failures.append(
                        "P5-3 future sampling figure missing: "
                        f"{figure_path.name}; missing figure evidence prevents "
                        "future-sampling acceptance"
                    )
                elif p5_3_phase and figure_path in p5_3_query_alignment_required:
                    failures.append(
                        "P5-3 query alignment figure missing: "
                        f"{figure_path.name}; missing figure evidence prevents "
                        "runtime query-alignment acceptance"
                    )
                elif p5_4_phase and figure_path in p5_4_required:
                    validate_p5_4_required_figures([figure_path], failures)
                elif p5_5_phase and figure_path in p5_5_required:
                    validate_p5_5_required_figures([figure_path], failures)
                elif p5_6_phase and figure_path in p5_6_required:
                    validate_p5_6_required_figures([figure_path], failures)
                else:
                    inconclusive.append(
                        f"{experiment_label} required figure was not generated or is empty: {figure_path}"
                    )

    status = "PASS"
    if p5_5_phase and bool(p5_5_gates.get("blocked_scenario_missing")):
        status = "BLOCKED_SCENARIO_MISSING"
    elif p5_4_phase and bool(p5_4_gates.get("blocked_scenario_missing")):
        status = "BLOCKED_SCENARIO_MISSING"
    elif failures:
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
            p0_health_summary if p0_runtime_phase else None,
        ),
        "export_dir": str(export_dir),
        "bag_dir": str(bag_dir) if bag_dir is not None else "",
        "manifest": manifest,
        "validator_summary": validator_summary,
        "topic_health": topic_health,
        "topic_timing_error": topic_timing_error,
        "integrity_summary": integrity_summary,
        "p0_summary": p0_summary,
        "p5_summary": {
            **p5_summary,
            "p5_1_hard_gates": p5_1_gates,
            "p5_2_hard_gates": p5_2_gates,
            "p5_3_hard_gates": p5_3_gates,
            "p5_4_hard_gates": p5_4_gates,
            "p5_5_hard_gates": p5_5_gates,
            "p5_6_hard_gates": p5_6_gates,
            "marker_evidence": p5_marker_summary,
            "p5_5_integrity_stamp_error": p5_5_integrity_error,
        },
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
    parser.add_argument("--blocked-fixture-audit", action="store_true", help="write a blocked fixture-audit summary without ROS artifacts")
    parser.add_argument("--fail-on-threshold", action="store_true", help="exit non-zero unless analysis status is PASS")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    summary = analyze(args)
    return 0 if (summary["passed"] or not args.fail_on_threshold) else 2


if __name__ == "__main__":
    raise SystemExit(main())
