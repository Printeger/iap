#!/usr/bin/env python3
"""
Run-level log analyzer for IAP unified logs.

This script scans a single run directory (defaults to src/iap/log/latest),
parses metadata/runtime/profiling/export artifacts, renders a compact report,
and generates supporting analysis figures.

First-version scope:
- Config summary from metadata snapshot and copied launch config directory
- Artifact coverage table with found / empty / disabled / missing states
- Runtime warning/error summary with notable messages
- Frontend-only timing attribution analysis when frontend_frame_profile.csv exists
- Generic pipeline timing analysis from pipeline_timing.csv
- Optional delegation to existing ICP / GNSS / ARAIM plotting scripts
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RUN = PACKAGE_ROOT / "log" / "latest"

RUNTIME_LOG_RE = re.compile(
    r"^\[(?P<ts>[^\]]+)\]\s+\[(?P<logger>[^\]]+)\]\s+\[(?P<level>[^\]]+)\]\s+(?P<message>.*)$"
)
KV_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
NUMBER_RE = re.compile(r"\b\d+(\.\d+)?\b")

FRONTEND_STAGE_COLUMNS = [
    "preprocess_ms",
    "target_map_prep_ms",
    "bucket_build_ms",
    "lidar_factor_build_ms",
    "imu_factor_build_ms",
    "lm_solve_ms",
    "marginalization_ms",
    "backend_update_ms",
    "backend_optimize_ms",
    "publish_ms",
    "local_mapping_update_ms",
    "global_mapping_update_ms",
    "submap_registration_ms",
]

CSV_ARTIFACTS = [
    ("frontend_frame_profile", "profiling/frontend_frame_profile.csv"),
    ("lidar_factor_profile", "profiling/lidar_factor_profile.csv"),
    ("solver_update_profile", "profiling/solver_update_profile.csv"),
    ("lidar_factor_internal_profile", "profiling/lidar_factor_internal_profile.csv"),
    ("frontend_lm_iteration", "profiling/frontend_lm_iteration.csv"),
    ("frame_warning_profile", "profiling/frame_warning_profile.csv"),
    ("jump_diagnostics", "profiling/jump_diagnostics.csv"),
    ("pipeline_timing", "profiling/pipeline_timing.csv"),
    ("numeric_reference", "profiling/numeric_reference.csv"),
    ("linearization_check", "profiling/linearization_check.csv"),
    ("ct_lidar_baseline", "export/ct_lidar_baseline.csv"),
    ("icp_quality", "export/icp_quality.csv"),
    ("gnss_factor_debug", "export/gnss_factor_debug.csv"),
    ("araim", "export/araim.csv"),
    ("integrity_trajectory", "export/integrity_trajectory.csv"),
    ("local_frontend_trajectory", "export/local_frontend_trajectory.csv"),
    ("backend_summary", "export/backend_summary.csv"),
    ("bucket_stats", "export/bucket_stats.csv"),
]

METADATA_ARTIFACTS = [
    ("run_info", "metadata/run_info.json"),
    ("config_snapshot", "metadata/config_snapshot.json"),
    ("git_rev", "metadata/git_rev.txt"),
    ("build_info", "metadata/build_info.txt"),
    ("mode_manifest", "metadata/mode_manifest.json"),
]


@dataclass
class ArtifactStatus:
    name: str
    rel_path: str
    status: str
    reason: str = "availability_unclear"
    rows: int | None = None
    size_bytes: int = 0
    note: str = ""


def strip_json_comments(text: str) -> str:
    result: list[str] = []
    i = 0
    n = len(text)
    in_string = False
    string_quote = ""
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if in_string:
            result.append(ch)
            if ch == "\\" and i + 1 < n:
                result.append(text[i + 1])
                i += 2
                continue
            if ch == string_quote:
                in_string = False
            i += 1
            continue

        if ch in ('"', "'"):
            in_string = True
            string_quote = ch
            result.append(ch)
            i += 1
            continue

        if ch == "/" and nxt == "/":
            i += 2
            while i < n and text[i] != "\n":
                i += 1
            continue

        if ch == "/" and nxt == "*":
            i += 2
            while i + 1 < n and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            i += 2
            continue

        result.append(ch)
        i += 1

    return "".join(result)


def load_jsonish(path: Path) -> dict[str, Any] | list[Any] | None:
    if not path.exists():
        return None
    text = path.read_text(encoding="utf-8", errors="replace")
    if not text.strip():
        return None
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return json.loads(strip_json_comments(text))


def safe_read_csv(path: Path) -> pd.DataFrame | None:
    if not path.exists() or path.stat().st_size == 0:
        return None
    try:
        return pd.read_csv(path)
    except pd.errors.EmptyDataError:
        return None
    except pd.errors.ParserError:
        return pd.read_csv(path, on_bad_lines="skip", engine="python")


def get_nested(obj: Any, *keys: str) -> Any:
    cur = obj
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def maybe_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        v = value.strip().lower()
        if v in {"1", "true", "yes", "on"}:
            return True
        if v in {"0", "false", "no", "off"}:
            return False
    return None


def pct(series: pd.Series, q: float) -> float:
    if series.empty:
        return 0.0
    return float(series.quantile(q))


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def resolve_run_dir(run_arg: str) -> Path:
    candidate = Path(run_arg)
    if not candidate.is_absolute():
        if candidate.exists():
            candidate = candidate
        else:
            candidate = PACKAGE_ROOT / run_arg
    candidate = candidate.resolve()
    if not candidate.exists():
        raise FileNotFoundError(f"Run path does not exist: {candidate}")
    if not candidate.is_dir():
        raise NotADirectoryError(f"Run path is not a directory: {candidate}")
    return candidate


def load_metadata(run_dir: Path) -> dict[str, Any]:
    metadata_dir = run_dir / "metadata"
    result: dict[str, Any] = {}
    result["run_info"] = load_jsonish(metadata_dir / "run_info.json") or {}
    result["config_snapshot"] = load_jsonish(metadata_dir / "config_snapshot.json") or {}
    result["git_rev"] = (metadata_dir / "git_rev.txt").read_text(encoding="utf-8", errors="replace").strip() \
        if (metadata_dir / "git_rev.txt").exists() else ""
    result["build_info"] = (metadata_dir / "build_info.txt").read_text(encoding="utf-8", errors="replace").strip() \
        if (metadata_dir / "build_info.txt").exists() else ""
    result["mode_manifest"] = load_jsonish(metadata_dir / "mode_manifest.json") or {}
    return result


def load_runtime_configs(metadata: dict[str, Any]) -> dict[str, Any]:
    configs: dict[str, Any] = {}
    snapshot = metadata.get("config_snapshot", {})
    if isinstance(snapshot, dict):
        for key, value in snapshot.items():
            if key.startswith("config_") and isinstance(value, dict):
                configs[key] = value

    config_path = snapshot.get("config_path") if isinstance(snapshot, dict) else None
    if not config_path:
        return configs

    config_dir = Path(config_path)
    if not config_dir.exists():
        return configs

    root_cfg = load_jsonish(config_dir / "config.json")
    if isinstance(root_cfg, dict):
        configs.setdefault("config_json", root_cfg)
        global_cfg = get_nested(root_cfg, "global") or {}
        mapping = {
            "config_logging": global_cfg.get("config_logging"),
            "config_ros": global_cfg.get("config_ros"),
            "config_viewer": global_cfg.get("config_viewer"),
            "config_preprocess": global_cfg.get("config_preprocess"),
            "config_sensors": global_cfg.get("config_sensors"),
            "config_odometry": global_cfg.get("config_odometry"),
            "config_sub_mapping": global_cfg.get("config_sub_mapping"),
            "config_global_mapping": global_cfg.get("config_global_mapping"),
            "config_gnss": global_cfg.get("config_gnss"),
        }
        for key, file_name in mapping.items():
            if key in configs or not file_name:
                continue
            path = config_dir / str(file_name)
            loaded = load_jsonish(path)
            if loaded is not None:
                configs[key] = loaded

    for fallback in config_dir.glob("config*.json"):
        key = fallback.stem
        configs.setdefault(key, load_jsonish(fallback))

    return configs


def normalize_message(message: str) -> str:
    msg = NUMBER_RE.sub("<n>", message)
    msg = re.sub(r"/[A-Za-z0-9_./-]+", "<path>", msg)
    return msg[:200]


def parse_runtime_logs(run_dir: Path) -> dict[str, Any]:
    runtime_dir = run_dir / "runtime"
    summary: dict[str, Any] = {
        "files": [],
        "total_lines": 0,
        "level_counts": Counter(),
        "issues": [],
        "pattern_counts": Counter(),
        "loaded_modules": [],
        "waiting_messages": [],
        "odometry_init": {},
    }

    if not runtime_dir.exists():
        return summary

    for log_path in sorted(runtime_dir.glob("*.log")):
        file_summary = {
            "file": log_path.name,
            "lines": 0,
            "level_counts": Counter(),
        }
        for line_no, raw in enumerate(log_path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1):
            file_summary["lines"] += 1
            summary["total_lines"] += 1
            match = RUNTIME_LOG_RE.match(raw)
            if not match:
                continue
            level = match.group("level").lower()
            message = match.group("message")
            file_summary["level_counts"][level] += 1
            summary["level_counts"][level] += 1

            if "load lib" in message:
                summary["loaded_modules"].append(message.replace("load ", "", 1))
            if "waiting for" in message:
                summary["waiting_messages"].append(message)
            if "odometry_bspline initialized" in message:
                summary["odometry_init"] = {k: v for k, v in KV_RE.findall(message)}

            if level in {"warn", "warning", "error", "critical"}:
                issue = {
                    "file": log_path.name,
                    "line": line_no,
                    "level": level,
                    "message": message,
                }
                summary["issues"].append(issue)
                summary["pattern_counts"][normalize_message(message)] += 1

        summary["files"].append(file_summary)

    summary["top_patterns"] = summary["pattern_counts"].most_common(10)
    return summary


def config_value(configs: dict[str, Any], *keys: str) -> Any:
    for root_key in keys:
        if root_key in configs and isinstance(configs[root_key], dict):
            return configs[root_key]
    return None


def build_config_summary(configs: dict[str, Any], runtime_summary: dict[str, Any], metadata: dict[str, Any]) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    root_cfg = configs.get("config_json", {})
    odom_cfg = configs.get("config_odometry", {})
    gnss_cfg = configs.get("config_gnss", {})
    log_cfg = get_nested(root_cfg, "log") or {}
    odom_block = get_nested(odom_cfg, "odometry_estimation") or {}
    gnss_block = get_nested(gnss_cfg, "gnss") or {}
    integrity_block = get_nested(gnss_cfg, "integrity") or {}

    summary["log_root_dir"] = get_nested(log_cfg, "root_dir")
    summary["run_dir_mode"] = get_nested(log_cfg, "run_dir_mode") or get_nested(metadata.get("run_info", {}), "run_dir_mode")
    summary["frontend_mode"] = odom_block.get("frontend_mode")
    summary["frontend_only_mode"] = odom_block.get("frontend_only_mode")
    summary["final_pose_surface"] = odom_block.get("final_pose_surface")
    summary["gravity_state_mode"] = odom_block.get("gravity_state_mode")
    summary["gravity_reference_source"] = odom_block.get("gravity_reference_source")
    summary["gravity_reference_vector"] = odom_block.get("gravity_reference_vector")
    summary["gravity_mode"] = odom_block.get("exp.gravity_mode")
    summary["gravity_fixed_norm_value"] = odom_block.get("exp.gravity_fixed_norm_value")
    summary["gravity_tilt_limit_rad"] = odom_block.get("exp.gravity_tilt_limit_rad")
    summary["gravity_warmup_freeze_frames"] = odom_block.get("exp.gravity_warmup_freeze_frames")
    summary["velocity_state_mode"] = odom_block.get("velocity_state_mode")
    summary["velocity_mode_policy"] = odom_block.get("velocity_mode_policy")
    summary["bias_state_mode"] = odom_block.get("bias_state_mode")
    summary["frontend_seed_mode"] = odom_block.get("frontend_seed_mode")
    summary["exp_freeze_gravity"] = odom_block.get("exp_freeze_gravity")
    summary["exp_freeze_gyro_bias"] = odom_block.get("exp_freeze_gyro_bias")
    summary["exp_freeze_accel_bias"] = odom_block.get("exp_freeze_accel_bias")
    summary["exp_disable_velocity_factor"] = odom_block.get("exp_disable_velocity_factor")
    summary["exp_disable_current_velocity_prior"] = odom_block.get("exp_disable_current_velocity_prior")
    summary["bucket_mode"] = odom_block.get("ct_lidar_bucket_mode")
    summary["bucket_time_eps"] = odom_block.get("ct_lidar_bucket_time_eps")
    summary["bucket_fixed_count"] = odom_block.get("ct_lidar_fixed_buckets_per_scan")
    summary["bucket_max_count"] = odom_block.get("ct_lidar_max_buckets_per_scan")
    summary["target_mode"] = odom_block.get("ct_lidar_target_mode")
    summary["pipeline_profile"] = (
        get_nested(log_cfg, "profiling", "pipeline")
        if get_nested(log_cfg, "profiling", "pipeline") is not None
        else odom_block.get("ct_profile_pipeline")
    )
    summary["frontend_frame_profile"] = get_nested(log_cfg, "profiling", "frontend_frame")
    summary["lidar_factor_profile"] = (
        get_nested(log_cfg, "profiling", "lidar_factor")
        if get_nested(log_cfg, "profiling", "lidar_factor") is not None
        else odom_block.get("ct_lidar_profile_factor")
    )
    summary["solver_update_profile"] = get_nested(log_cfg, "profiling", "solver_update_profile")
    summary["lidar_factor_internal_profile"] = get_nested(log_cfg, "profiling", "lidar_factor_internal_profile")
    summary["frontend_lm_iteration_profile"] = get_nested(log_cfg, "profiling", "frontend_lm_iteration")
    summary["frame_warning_profile"] = get_nested(log_cfg, "profiling", "frame_warning_profile")
    summary["jump_diagnostics"] = get_nested(log_cfg, "profiling", "jump_diagnostics")
    summary["target_map_prep_breakdown"] = get_nested(log_cfg, "profiling", "target_map_prep_breakdown")
    summary["graph_problem_size"] = get_nested(log_cfg, "profiling", "graph_problem_size")
    summary["gnss_debug_csv"] = (
        get_nested(log_cfg, "export", "gnss_factor_debug_csv")
        if get_nested(log_cfg, "export", "gnss_factor_debug_csv") is not None
        else gnss_block.get("enable_debug_csv")
    )
    summary["araim_csv"] = (
        get_nested(log_cfg, "export", "araim_csv")
        if get_nested(log_cfg, "export", "araim_csv") is not None
        else integrity_block.get("enable_araim_csv")
    )
    summary["integrity_traj_csv"] = (
        get_nested(log_cfg, "export", "integrity_trajectory_csv")
        if get_nested(log_cfg, "export", "integrity_trajectory_csv") is not None
        else integrity_block.get("enable_traj_csv")
    )
    ros_block = get_nested(configs.get("config_ros", {}), "glim_ros") or {}
    summary["enable_local_mapping"] = ros_block.get("enable_local_mapping")
    summary["enable_global_mapping"] = ros_block.get("enable_global_mapping")
    summary["extension_modules"] = ros_block.get("extension_modules")
    summary["write_mode_manifest"] = get_nested(log_cfg, "metadata", "write_mode_manifest")
    run_info = metadata.get("run_info", {}) if isinstance(metadata.get("run_info", {}), dict) else {}
    for key in [
        "config_log_profiling_solver_update_profile",
        "runtime_log_profiling_solver_update_profile",
        "config_log_profiling_lidar_factor_internal_profile",
        "runtime_log_profiling_lidar_factor_internal_profile",
        "config_log_profiling_jump_diagnostics",
        "runtime_log_profiling_jump_diagnostics",
        "config_final_pose_surface",
        "runtime_final_pose_surface",
        "config_gravity_state_mode",
        "runtime_gravity_state_mode",
        "config_gravity_reference_source",
        "runtime_gravity_reference_source",
        "config_gravity_reference_vector",
        "runtime_gravity_reference_vector",
        "config_gravity_mode",
        "runtime_gravity_mode",
        "config_gravity_fixed_norm_value",
        "runtime_gravity_fixed_norm_value",
        "config_gravity_tilt_limit_rad",
        "runtime_gravity_tilt_limit_rad",
        "config_gravity_warmup_freeze_frames",
        "runtime_gravity_warmup_freeze_frames",
        "config_exp_freeze_gravity",
        "runtime_exp_freeze_gravity",
        "config_exp_freeze_gyro_bias",
        "runtime_exp_freeze_gyro_bias",
        "config_exp_freeze_accel_bias",
        "runtime_exp_freeze_accel_bias",
        "config_exp_disable_velocity_factor",
        "runtime_exp_disable_velocity_factor",
        "config_exp_disable_current_velocity_prior",
        "runtime_exp_disable_current_velocity_prior",
        "config_velocity_state_mode",
        "runtime_velocity_state_mode",
        "config_velocity_mode_policy",
        "runtime_velocity_mode_policy",
        "config_bias_state_mode",
        "runtime_bias_state_mode",
        "runtime_bias_optimized",
        "runtime_bias_source_of_truth",
        "runtime_bias_transition_prior_enabled",
        "runtime_bias_transition_prior_strength",
        "runtime_bias_can_be_survivor_anchor",
        "runtime_bias_writeback_mode",
        "config_frontend_seed_mode",
        "runtime_frontend_seed_mode",
        "runtime_imu_forward_prediction_enabled",
        "runtime_frontend_seed_fallback_used",
        "runtime_frontend_seed_source",
        "runtime_frontend_seed_imu_sample_count",
        "runtime_has_gnss_constraints",
        "runtime_velocity_optimized",
        "runtime_experiment_name",
    ]:
        if key in run_info:
            summary[key] = run_info[key]

    init_kv = runtime_summary.get("odometry_init", {})
    for key in ["frontend_mode", "frontend_only_mode", "lidar_bucket_mode", "lidar_target_mode", "final_pose_surface"]:
        if key in init_kv:
            summary[f"runtime_{key}"] = init_kv[key]

    return {k: v for k, v in summary.items() if v is not None}


def frontend_only_effective(config_summary: dict[str, Any]) -> bool | None:
    runtime_value = maybe_bool(config_summary.get("runtime_frontend_only_mode"))
    if runtime_value is not None:
        return runtime_value
    return maybe_bool(config_summary.get("frontend_only_mode"))


def mapping_expected_active(config_summary: dict[str, Any]) -> bool | None:
    local = maybe_bool(config_summary.get("enable_local_mapping"))
    global_ = maybe_bool(config_summary.get("enable_global_mapping"))
    if local is None and global_ is None:
        return None
    return bool(local or global_)


def mapping_modules_loaded(runtime_summary: dict[str, Any]) -> list[str]:
    loaded = runtime_summary.get("loaded_modules", [])
    return [name for name in loaded if "sub_mapping" in name or "global_mapping" in name]


def artifact_enabled(name: str, configs: dict[str, Any], config_summary: dict[str, Any]) -> bool | None:
    root_cfg = configs.get("config_json", {})
    log_cfg = get_nested(root_cfg, "log") or {}
    odom_cfg = get_nested(configs.get("config_odometry", {}), "odometry_estimation") or {}
    gnss_cfg = get_nested(configs.get("config_gnss", {}), "gnss") or {}
    integrity_cfg = get_nested(configs.get("config_gnss", {}), "integrity") or {}
    global_cfg = get_nested(root_cfg, "global") or {}

    mapping: dict[str, Any] = {
        "pipeline_timing": get_nested(log_cfg, "profiling", "pipeline") if get_nested(log_cfg, "profiling", "pipeline") is not None else global_cfg.get("enable_timing_csv"),
        "lidar_factor_profile": get_nested(log_cfg, "profiling", "lidar_factor") if get_nested(log_cfg, "profiling", "lidar_factor") is not None else (True if maybe_bool(config_summary.get("frontend_only_mode")) else odom_cfg.get("ct_lidar_profile_factor")),
        "solver_update_profile": get_nested(log_cfg, "profiling", "solver_update_profile"),
        "lidar_factor_internal_profile": get_nested(log_cfg, "profiling", "lidar_factor_internal_profile"),
        "frontend_frame_profile": get_nested(log_cfg, "profiling", "frontend_frame") if get_nested(log_cfg, "profiling", "frontend_frame") is not None else maybe_bool(config_summary.get("frontend_only_mode")),
        "frontend_lm_iteration": get_nested(log_cfg, "profiling", "frontend_lm_iteration") if get_nested(log_cfg, "profiling", "frontend_lm_iteration") is not None else maybe_bool(config_summary.get("frontend_only_mode")),
        "frame_warning_profile": get_nested(log_cfg, "profiling", "frame_warning_profile"),
        "jump_diagnostics": get_nested(log_cfg, "profiling", "jump_diagnostics"),
        "numeric_reference": get_nested(log_cfg, "profiling", "numeric_reference") if get_nested(log_cfg, "profiling", "numeric_reference") is not None else odom_cfg.get("ct_lidar_profile_numeric_reference"),
        "linearization_check": get_nested(log_cfg, "profiling", "linearization_check") if get_nested(log_cfg, "profiling", "linearization_check") is not None else odom_cfg.get("ct_lidar_validate_linearization"),
        "ct_lidar_baseline": get_nested(log_cfg, "export", "baseline_csv") if get_nested(log_cfg, "export", "baseline_csv") is not None else odom_cfg.get("ct_lidar_export_baseline_csv"),
        "icp_quality": get_nested(log_cfg, "export", "icp_quality_csv") if get_nested(log_cfg, "export", "icp_quality_csv") is not None else odom_cfg.get("enable_icp_csv"),
        "gnss_factor_debug": get_nested(log_cfg, "export", "gnss_factor_debug_csv") if get_nested(log_cfg, "export", "gnss_factor_debug_csv") is not None else gnss_cfg.get("enable_debug_csv"),
        "araim": get_nested(log_cfg, "export", "araim_csv") if get_nested(log_cfg, "export", "araim_csv") is not None else integrity_cfg.get("enable_araim_csv"),
        "integrity_trajectory": get_nested(log_cfg, "export", "integrity_trajectory_csv") if get_nested(log_cfg, "export", "integrity_trajectory_csv") is not None else integrity_cfg.get("enable_traj_csv"),
        "local_frontend_trajectory": get_nested(log_cfg, "export", "local_frontend_trajectory"),
        "backend_summary": get_nested(log_cfg, "export", "backend_summary"),
        "bucket_stats": get_nested(log_cfg, "export", "bucket_stats"),
        "mode_manifest": get_nested(log_cfg, "metadata", "write_mode_manifest"),
    }
    return maybe_bool(mapping.get(name))


def artifact_reason(
    name: str,
    exists: bool,
    has_data: bool,
    enabled: bool | None,
    config_summary: dict[str, Any],
) -> str:
    frontend_only = frontend_only_effective(config_summary)
    if exists and has_data:
        if enabled is False:
            return "present_but_unreferenced"
        return "found_with_data"

    if exists and not has_data:
        if frontend_only and name in {"gnss_factor_debug", "araim", "integrity_trajectory"}:
            return "expected_empty_by_mode"
        if enabled is False:
            return "config_disabled"
        if name == "frontend_lm_iteration":
            return "availability_unclear"
        return "empty_unexpected"

    if not exists:
        if frontend_only and name in {"backend_summary"}:
            return "expected_missing_by_mode"
        if enabled is False:
            return "expected_missing_by_config"
        if enabled is True:
            return "missing_unexpected"
        return "availability_unclear"

    return "availability_unclear"


def scan_artifacts(run_dir: Path, configs: dict[str, Any], config_summary: dict[str, Any]) -> list[ArtifactStatus]:
    statuses: list[ArtifactStatus] = []
    for name, rel_path in METADATA_ARTIFACTS + CSV_ARTIFACTS:
        path = run_dir / rel_path
        exists = path.exists()
        size_bytes = path.stat().st_size if exists else 0
        rows: int | None = None
        note = ""

        if rel_path.endswith(".csv") and exists:
            df = safe_read_csv(path)
            rows = 0 if df is None else len(df)
            if size_bytes > 0 and rows == 0:
                note = "file exists but contains no data rows"
        elif rel_path.endswith(".json") and exists and size_bytes == 0:
            note = "empty json file"

        enabled = artifact_enabled(name, configs, config_summary)

        if exists:
            if size_bytes == 0:
                status = "empty"
            elif rows == 0 and rel_path.endswith(".csv"):
                status = "empty"
            else:
                status = "found"
        else:
            if enabled is False:
                status = "disabled"
                note = "disabled by configuration"
            elif enabled is True:
                status = "expected_missing"
                note = "enabled or implied by runtime, but file is missing"
            else:
                status = "missing"
                note = "not found; config availability unclear"

        has_data = exists and ((rows is None and size_bytes > 0) or (rows is not None and rows > 0))
        reason = artifact_reason(name, exists, has_data, enabled, config_summary)
        if reason == "expected_empty_by_mode" and not note:
            note = "empty because the current mode does not exercise this producer"
        elif reason == "expected_missing_by_mode" and not note:
            note = "not expected to be emitted in the current mode"
        elif reason == "present_but_unreferenced" and not note:
            note = "artifact exists even though config suggests it was disabled"

        statuses.append(
            ArtifactStatus(
                name=name,
                rel_path=rel_path,
                status=status,
                reason=reason,
                rows=rows,
                size_bytes=size_bytes,
                note=note,
            )
        )
    return statuses


def load_available_frames(run_dir: Path) -> dict[str, pd.DataFrame]:
    dfs: dict[str, pd.DataFrame] = {}
    for name, rel_path in CSV_ARTIFACTS:
        path = run_dir / rel_path
        df = safe_read_csv(path)
        if df is not None:
            dfs[name] = df
    return dfs


def reconcile_artifact_statuses(artifact_statuses: list[ArtifactStatus], dataframes: dict[str, pd.DataFrame]) -> None:
    status_map = {artifact.name: artifact for artifact in artifact_statuses}
    frame_df = dataframes.get("frontend_frame_profile")
    lm_artifact = status_map.get("frontend_lm_iteration")
    if frame_df is not None and not frame_df.empty and lm_artifact and lm_artifact.status == "expected_missing":
        if "lm_iteration_count" in frame_df.columns:
            max_iters = int(frame_df["lm_iteration_count"].fillna(0).max())
            if max_iters <= 0:
                lm_artifact.status = "disabled"
                lm_artifact.reason = "config_disabled"
                lm_artifact.note = "not emitted because lm_iteration_count stayed zero for all frames"


def stage_names_from_df(frame_df: pd.DataFrame) -> list[str]:
    return [col for col in FRONTEND_STAGE_COLUMNS if col in frame_df.columns]


def top_stage_per_row(frame_df: pd.DataFrame, stage_cols: list[str]) -> pd.Series:
    if frame_df.empty or not stage_cols:
        return pd.Series(dtype="object")
    return frame_df[stage_cols].idxmax(axis=1)


def analyze_frontend_timing(
    frame_df: pd.DataFrame,
    lm_df: pd.DataFrame | None,
    out_dir: Path,
    render_plots: bool = True,
) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if frame_df is None or frame_df.empty:
        return analysis

    df = frame_df.copy()
    stage_cols = stage_names_from_df(df)
    if not stage_cols:
        return analysis

    df["frame_total_ms"] = df[stage_cols].sum(axis=1)
    top_stage = top_stage_per_row(df, stage_cols)
    analysis["available"] = True
    analysis["frame_count"] = int(len(df))
    analysis["first_frame_id"] = int(df["frame_id"].min()) if "frame_id" in df else 0
    analysis["last_frame_id"] = int(df["frame_id"].max()) if "frame_id" in df else 0
    analysis["stamp_span_s"] = float(df["stamp"].max() - df["stamp"].min()) if "stamp" in df and len(df) > 1 else 0.0
    analysis["total_mean_ms"] = float(df["frame_total_ms"].mean())
    analysis["total_p50_ms"] = pct(df["frame_total_ms"], 0.5)
    analysis["total_p95_ms"] = pct(df["frame_total_ms"], 0.95)
    analysis["total_p99_ms"] = pct(df["frame_total_ms"], 0.99)
    analysis["total_max_ms"] = float(df["frame_total_ms"].max())
    analysis["bucket_mode"] = sorted(df["bucket_mode"].dropna().astype(str).unique().tolist()) if "bucket_mode" in df else []
    analysis["frontend_only_mode_values"] = sorted(df["frontend_only_mode"].dropna().astype(int).unique().tolist()) if "frontend_only_mode" in df else []

    stage_summary = []
    total_mean = max(float(df["frame_total_ms"].mean()), 1e-9)
    for col in stage_cols:
        col_series = df[col].fillna(0.0)
        stage_summary.append({
            "stage": col,
            "mean_ms": float(col_series.mean()),
            "p95_ms": pct(col_series, 0.95),
            "max_ms": float(col_series.max()),
            "mean_share": float(col_series.mean() / total_mean),
        })
    stage_summary.sort(key=lambda item: item["mean_ms"], reverse=True)
    analysis["stage_summary"] = stage_summary
    analysis["top_stage_counts"] = dict(Counter(top_stage).most_common())

    slow = df.sort_values("frame_total_ms", ascending=False).head(10)
    analysis["slow_frames"] = slow[
        [c for c in ["frame_id", "stamp", "frame_total_ms", "actual_bucket_count", "lm_solve_ms", "lidar_factor_build_ms", "imu_factor_build_ms"] if c in slow.columns]
    ].to_dict(orient="records")

    if render_plots:
        ensure_dir(out_dir)

        fig, ax = plt.subplots(figsize=(12, 5))
        x = df["frame_id"] if "frame_id" in df else np.arange(len(df))
        ax.plot(x, df["frame_total_ms"], label="frame_total_ms", color="#111111", linewidth=1.2)
        for col, color in [
            ("lm_solve_ms", "#d62728"),
            ("lidar_factor_build_ms", "#1f77b4"),
            ("imu_factor_build_ms", "#2ca02c"),
            ("target_map_prep_ms", "#9467bd"),
            ("preprocess_ms", "#ff7f0e"),
        ]:
            if col in df.columns:
                ax.plot(x, df[col], label=col, linewidth=0.9, alpha=0.85, color=color)
        ax.set_title("Frontend Frame Timing Timeline")
        ax.set_xlabel("frame_id")
        ax.set_ylabel("ms")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=3)
        fig.tight_layout()
        timeline_path = out_dir / "frontend_timing_timeline.png"
        fig.savefig(timeline_path, dpi=150)
        plt.close(fig)
        analysis["fig_frontend_timing_timeline"] = str(timeline_path)

        stage_labels = [item["stage"] for item in stage_summary[:8]]
        stage_means = [item["mean_ms"] for item in stage_summary[:8]]
        stage_p95s = [item["p95_ms"] for item in stage_summary[:8]]
        fig, ax = plt.subplots(figsize=(12, 5))
        xs = np.arange(len(stage_labels))
        ax.bar(xs - 0.18, stage_means, width=0.36, label="mean_ms", color="#4c78a8")
        ax.bar(xs + 0.18, stage_p95s, width=0.36, label="p95_ms", color="#f58518")
        ax.set_xticks(xs)
        ax.set_xticklabels(stage_labels, rotation=30, ha="right")
        ax.set_ylabel("ms")
        ax.set_title("Frontend Stage Summary")
        ax.grid(True, alpha=0.3, axis="y")
        ax.legend()
        fig.tight_layout()
        stage_path = out_dir / "frontend_stage_summary.png"
        fig.savefig(stage_path, dpi=150)
        plt.close(fig)
        analysis["fig_frontend_stage_summary"] = str(stage_path)

        if "actual_bucket_count" in df.columns and "lm_solve_ms" in df.columns:
            fig, ax = plt.subplots(figsize=(7, 5))
            scatter = ax.scatter(
                df["actual_bucket_count"],
                df["lm_solve_ms"],
                c=df["frame_total_ms"],
                cmap="viridis",
                s=18,
                alpha=0.75,
            )
            ax.set_xlabel("actual_bucket_count")
            ax.set_ylabel("lm_solve_ms")
            ax.set_title("Bucket Count vs LM Solve Time")
            ax.grid(True, alpha=0.3)
            cbar = fig.colorbar(scatter, ax=ax)
            cbar.set_label("frame_total_ms")
            fig.tight_layout()
            bucket_path = out_dir / "bucket_vs_lm_solve.png"
            fig.savefig(bucket_path, dpi=150)
            plt.close(fig)
            analysis["fig_bucket_vs_lm_solve"] = str(bucket_path)

        if lm_df is not None and not lm_df.empty:
            ldf = lm_df.copy()
            fig, ax = plt.subplots(figsize=(12, 5))
            ax.plot(ldf["iteration_index"], ldf["cost_after"], ".", alpha=0.6, label="cost_after")
            if "linear_solve_ms" in ldf.columns:
                ax2 = ax.twinx()
                ax2.plot(ldf["iteration_index"], ldf["linear_solve_ms"], color="#d62728", alpha=0.6, label="linear_solve_ms")
                ax2.set_ylabel("linear_solve_ms")
            ax.set_title("Frontend LM Iteration Behavior")
            ax.set_xlabel("iteration_index")
            ax.set_ylabel("cost_after")
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            lm_path = out_dir / "frontend_lm_iterations.png"
            fig.savefig(lm_path, dpi=150)
            plt.close(fig)
            analysis["fig_frontend_lm_iterations"] = str(lm_path)

    return analysis


def analyze_lidar_bucket_profile(bucket_df: pd.DataFrame, out_dir: Path, render_plots: bool = True) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if bucket_df is None or bucket_df.empty:
        return analysis

    df = bucket_df.copy()
    analysis["available"] = True
    analysis["bucket_rows"] = int(len(df))
    analysis["points_in_bucket_mean"] = float(df["points_in_bucket"].mean()) if "points_in_bucket" in df else 0.0
    analysis["points_in_bucket_p95"] = pct(df["points_in_bucket"], 0.95) if "points_in_bucket" in df else 0.0
    analysis["factor_total_mean_ms"] = float(df["factor_total_ms"].mean()) if "factor_total_ms" in df else 0.0
    analysis["factor_total_p95_ms"] = pct(df["factor_total_ms"], 0.95) if "factor_total_ms" in df else 0.0
    analysis["match_ratio_mean"] = float(df["match_ratio"].mean()) if "match_ratio" in df else 0.0
    analysis["inlier_ratio_mean"] = float(df["inlier_ratio"].mean()) if "inlier_ratio" in df else 0.0

    if render_plots:
        ensure_dir(out_dir)

        if {"points_in_bucket", "factor_total_ms"}.issubset(df.columns):
            fig, ax = plt.subplots(figsize=(7, 5))
            scatter = ax.scatter(
                df["points_in_bucket"],
                df["factor_total_ms"],
                c=df["match_ratio"] if "match_ratio" in df else "#4c78a8",
                cmap="viridis",
                s=18,
                alpha=0.75,
            )
            ax.set_xlabel("points_in_bucket")
            ax.set_ylabel("factor_total_ms")
            ax.set_title("LiDAR Bucket Load")
            ax.grid(True, alpha=0.3)
            if "match_ratio" in df:
                cbar = fig.colorbar(scatter, ax=ax)
                cbar.set_label("match_ratio")
            fig.tight_layout()
            out_path = out_dir / "lidar_bucket_load.png"
            fig.savefig(out_path, dpi=150)
            plt.close(fig)
            analysis["fig_lidar_bucket_load"] = str(out_path)

    return analysis


def analyze_solver_update(
    solver_df: pd.DataFrame | None,
    frame_df: pd.DataFrame | None,
    out_dir: Path,
    render_plots: bool = True,
) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if solver_df is None or solver_df.empty:
        return analysis

    df = solver_df.copy()
    analysis["available"] = True
    analysis["row_count"] = int(len(df))
    analysis["solver_modes"] = sorted(df["solver_mode"].dropna().astype(str).unique().tolist()) if "solver_mode" in df else []
    analysis["used_incremental_solver_values"] = (
        sorted(df["used_incremental_solver"].dropna().astype(int).unique().tolist())
        if "used_incremental_solver" in df else []
    )
    analysis["local_layer_enabled_values"] = (
        sorted(df["local_layer_enabled"].dropna().astype(int).unique().tolist())
        if "local_layer_enabled" in df else []
    )
    analysis["navigation_layer_enabled_values"] = (
        sorted(df["navigation_layer_enabled"].dropna().astype(int).unique().tolist())
        if "navigation_layer_enabled" in df else []
    )
    analysis["fallback_used_values"] = (
        sorted(df["fallback_used"].dropna().astype(int).unique().tolist())
        if "fallback_used" in df else []
    )

    for col in [
        "solver_update_ms",
        "estimate_query_ms",
        "fallback_rebuild_ms",
        "relinearization_ms",
        "linearization_ms",
        "elimination_ms",
        "delta_solve_ms",
        "new_factor_count",
        "new_value_count",
        "retired_key_count",
        "active_control_point_count",
        "active_pose_key_count",
        "active_aux_key_count",
        "persistent_key_count",
        "relinearized_variable_count",
        "reeliminated_variable_count",
        "relinearized_factor_count",
        "linearized_factor_count",
        "bayes_tree_clique_count",
        "affected_variable_count",
        "observed_key_count",
        "new_factor_index_count",
        "current_nonlinear_factor_count",
        "active_window_imu_factor_count",
        "active_window_velocity_factor_count",
        "active_window_lidar_factor_count",
        "active_window_lidar_current_segment_factor_count",
        "active_window_lidar_old_segment_factor_count",
        "active_window_prior_factor_count",
        "active_window_shared_jkg_touching_factor_count",
        "recalculated_imu_factor_count",
        "recalculated_velocity_factor_count",
        "recalculated_lidar_factor_count",
        "recalculated_lidar_current_segment_factor_count",
        "recalculated_lidar_old_segment_factor_count",
        "recalculated_lidar_same_support_factor_count",
        "recalculated_lidar_cross_support_factor_count",
        "recalculated_prior_factor_count",
        "recalculated_shared_jkg_touching_factor_count",
        "relinearized_pose_variable_count",
        "relinearized_aux_variable_count",
        "relinearized_shared_variable_count",
        "affected_pose_key_count",
        "affected_aux_key_count",
        "affected_shared_key_count",
        "isam_reported_update_ms",
    ]:
        if col in df.columns:
            series = df[col].fillna(0)
            analysis[f"{col}_mean"] = float(series.mean())
            analysis[f"{col}_p95"] = pct(series, 0.95)
            analysis[f"{col}_max"] = float(series.max())

    if {"active_window_shared_jkg_touching_factor_count", "active_window_imu_factor_count"}.issubset(df.columns):
        shared = df["active_window_shared_jkg_touching_factor_count"].fillna(0)
        imu = df["active_window_imu_factor_count"].fillna(0)
        non_imu_shared = (shared - imu).clip(lower=0)
        analysis["active_window_non_imu_shared_jkg_factor_count_mean"] = float(non_imu_shared.mean())
        analysis["active_window_non_imu_shared_jkg_factor_count_p95"] = pct(non_imu_shared, 0.95)
        analysis["active_window_non_imu_shared_jkg_factor_count_max"] = float(non_imu_shared.max())

    if {
        "relinearized_factor_count",
        "recalculated_imu_factor_count",
        "recalculated_velocity_factor_count",
        "recalculated_lidar_factor_count",
        "recalculated_prior_factor_count",
    }.issubset(df.columns):
        unclassified = (
            df["relinearized_factor_count"].fillna(0)
            - df["recalculated_imu_factor_count"].fillna(0)
            - df["recalculated_velocity_factor_count"].fillna(0)
            - df["recalculated_lidar_factor_count"].fillna(0)
            - df["recalculated_prior_factor_count"].fillna(0)
        ).clip(lower=0)
        analysis["recalculated_unclassified_factor_count_mean"] = float(unclassified.mean())
        analysis["recalculated_unclassified_factor_count_p95"] = pct(unclassified, 0.95)
        analysis["recalculated_unclassified_factor_count_max"] = float(unclassified.max())
        if "solver_update_ms" in df.columns and len(df) >= 3:
            pair = pd.DataFrame({
                "recalculated_unclassified_factor_count": unclassified,
                "solver_update_ms": df["solver_update_ms"].fillna(0),
            }).dropna()
            if (
                len(pair) >= 3
                and float(pair["recalculated_unclassified_factor_count"].std(ddof=0)) > 0.0
                and float(pair["solver_update_ms"].std(ddof=0)) > 0.0
            ):
                analysis["recalculated_unclassified_factor_vs_solver_update_corr"] = float(
                    pair["recalculated_unclassified_factor_count"].corr(pair["solver_update_ms"], method="pearson")
                )

    if {"solver_update_ms", "estimate_query_ms", "fallback_rebuild_ms", "relinearization_ms", "linearization_ms", "elimination_ms", "delta_solve_ms"}.issubset(df.columns):
        stage_cols = [
            "estimate_query_ms",
            "fallback_rebuild_ms",
            "relinearization_ms",
            "linearization_ms",
            "elimination_ms",
            "delta_solve_ms",
        ]
        stage_summary = []
        denom = max(float(df["solver_update_ms"].fillna(0).mean()), 1e-9)
        for col in stage_cols:
            col_series = df[col].fillna(0)
            stage_summary.append({
                "stage": col,
                "mean_ms": float(col_series.mean()),
                "p95_ms": pct(col_series, 0.95),
                "max_ms": float(col_series.max()),
                "mean_share": float(col_series.mean() / denom),
            })
        stage_summary.sort(key=lambda item: item["mean_ms"], reverse=True)
        analysis["stage_summary"] = stage_summary
        unavailable = [
            item["stage"] for item in stage_summary
            if item["mean_ms"] == 0.0 and item["p95_ms"] == 0.0 and item["max_ms"] == 0.0
        ]
        analysis["unavailable_internal_fields"] = unavailable
        analysis["unavailable_internal_timing"] = bool(
            {"relinearization_ms", "linearization_ms", "elimination_ms"}.issubset(set(unavailable))
        )

    if {"active_control_point_count", "new_factor_count"}.issubset(df.columns) and len(df) >= 3:
        pair = df[["active_control_point_count", "new_factor_count"]].dropna()
        if len(pair) >= 3 and float(pair["active_control_point_count"].std(ddof=0)) > 0.0 and float(pair["new_factor_count"].std(ddof=0)) > 0.0:
            analysis["new_factor_vs_active_window_corr"] = float(pair["active_control_point_count"].corr(pair["new_factor_count"], method="pearson"))
    if {"active_control_point_count", "new_value_count"}.issubset(df.columns) and len(df) >= 3:
        pair = df[["active_control_point_count", "new_value_count"]].dropna()
        if len(pair) >= 3 and float(pair["active_control_point_count"].std(ddof=0)) > 0.0 and float(pair["new_value_count"].std(ddof=0)) > 0.0:
            analysis["new_value_vs_active_window_corr"] = float(pair["active_control_point_count"].corr(pair["new_value_count"], method="pearson"))

    correlation_specs = [
        ("reeliminated_variable_count", "reeliminated_variable_vs_solver_update_corr"),
        ("relinearized_variable_count", "relinearized_variable_vs_solver_update_corr"),
        ("relinearized_factor_count", "recalculated_factor_vs_solver_update_corr"),
        ("affected_variable_count", "affected_variable_vs_solver_update_corr"),
        ("active_window_imu_factor_count", "active_window_imu_factor_vs_solver_update_corr"),
        ("active_window_lidar_current_segment_factor_count", "active_window_lidar_current_segment_factor_vs_solver_update_corr"),
        ("active_window_lidar_old_segment_factor_count", "active_window_lidar_old_segment_factor_vs_solver_update_corr"),
        ("active_window_shared_jkg_touching_factor_count", "active_window_shared_jkg_touching_factor_vs_solver_update_corr"),
        ("recalculated_imu_factor_count", "recalculated_imu_factor_vs_solver_update_corr"),
        ("recalculated_velocity_factor_count", "recalculated_velocity_factor_vs_solver_update_corr"),
        ("recalculated_lidar_factor_count", "recalculated_lidar_factor_vs_solver_update_corr"),
        ("recalculated_lidar_current_segment_factor_count", "recalculated_lidar_current_segment_factor_vs_solver_update_corr"),
        ("recalculated_lidar_old_segment_factor_count", "recalculated_lidar_old_segment_factor_vs_solver_update_corr"),
        ("recalculated_lidar_same_support_factor_count", "recalculated_lidar_same_support_factor_vs_solver_update_corr"),
        ("recalculated_lidar_cross_support_factor_count", "recalculated_lidar_cross_support_factor_vs_solver_update_corr"),
        ("recalculated_prior_factor_count", "recalculated_prior_factor_vs_solver_update_corr"),
        ("recalculated_shared_jkg_touching_factor_count", "recalculated_shared_jkg_touching_factor_vs_solver_update_corr"),
    ]
    for col, out_key in correlation_specs:
        if {col, "solver_update_ms"}.issubset(df.columns) and len(df) >= 3:
            pair = df[[col, "solver_update_ms"]].dropna()
            if len(pair) >= 3 and float(pair[col].std(ddof=0)) > 0.0 and float(pair["solver_update_ms"].std(ddof=0)) > 0.0:
                analysis[out_key] = float(pair[col].corr(pair["solver_update_ms"], method="pearson"))

    if "solver_update_ms" in df.columns:
        slow = df.sort_values("solver_update_ms", ascending=False).head(10)
        keep_cols = [
            "frame_id",
            "frame_stamp",
            "solver_mode",
            "solver_update_ms",
            "delta_solve_ms",
            "relinearization_ms",
            "linearization_ms",
            "elimination_ms",
            "new_factor_count",
            "new_value_count",
            "active_control_point_count",
            "active_aux_key_count",
            "active_window_imu_factor_count",
            "active_window_velocity_factor_count",
            "active_window_lidar_factor_count",
            "active_window_lidar_current_segment_factor_count",
            "active_window_lidar_old_segment_factor_count",
            "active_window_prior_factor_count",
            "active_window_shared_jkg_touching_factor_count",
            "recalculated_imu_factor_count",
            "recalculated_velocity_factor_count",
            "recalculated_lidar_factor_count",
            "recalculated_lidar_current_segment_factor_count",
            "recalculated_lidar_old_segment_factor_count",
            "recalculated_lidar_same_support_factor_count",
            "recalculated_lidar_cross_support_factor_count",
            "recalculated_prior_factor_count",
            "recalculated_shared_jkg_touching_factor_count",
            "affected_variable_count",
            "reeliminated_variable_count",
            "relinearized_variable_count",
            "relinearized_factor_count",
            "solver_status",
        ]
        analysis["slow_updates"] = slow[[c for c in keep_cols if c in slow.columns]].to_dict(orient="records")

    dominant_candidates = []
    for family_name, corr_key, mean_key in [
        ("IMU", "recalculated_imu_factor_vs_solver_update_corr", "recalculated_imu_factor_count_mean"),
        ("VELOCITY", "recalculated_velocity_factor_vs_solver_update_corr", "recalculated_velocity_factor_count_mean"),
        ("LIDAR", "recalculated_lidar_factor_vs_solver_update_corr", "recalculated_lidar_factor_count_mean"),
        ("PRIOR", "recalculated_prior_factor_vs_solver_update_corr", "recalculated_prior_factor_count_mean"),
    ]:
        corr = analysis.get(corr_key)
        if isinstance(corr, float):
            dominant_candidates.append({
                "family": family_name,
                "corr": corr,
                "mean": float(analysis.get(mean_key, 0.0)),
            })
    dominant_candidates.sort(key=lambda item: item["corr"], reverse=True)
    if dominant_candidates:
        analysis["recalculated_family_candidates"] = dominant_candidates
        analysis["dominant_recalculated_family"] = dominant_candidates[0]["family"]
        analysis["dominant_recalculated_family_corr"] = dominant_candidates[0]["corr"]
        if len(dominant_candidates) > 1:
            analysis["second_recalculated_family"] = dominant_candidates[1]["family"]
            analysis["second_recalculated_family_corr"] = dominant_candidates[1]["corr"]

    if render_plots and "solver_update_ms" in df.columns:
        ensure_dir(out_dir)
        x = df["frame_id"] if "frame_id" in df.columns else np.arange(len(df))

        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(x, df["solver_update_ms"], label="solver_update_ms", color="#111111", linewidth=1.2)
        for col, color in [
            ("delta_solve_ms", "#d62728"),
            ("estimate_query_ms", "#1f77b4"),
            ("relinearization_ms", "#2ca02c"),
            ("linearization_ms", "#9467bd"),
            ("elimination_ms", "#ff7f0e"),
        ]:
            if col in df.columns:
                ax.plot(x, df[col].fillna(0), label=col, linewidth=0.9, alpha=0.85, color=color)
        ax.set_title("Solver Update Timeline")
        ax.set_xlabel("frame_id")
        ax.set_ylabel("ms")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8, ncol=3)
        fig.tight_layout()
        out_path = out_dir / "solver_update_timeline.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        analysis["fig_solver_update_timeline"] = str(out_path)

        if {"active_control_point_count", "new_factor_count"}.issubset(df.columns):
            fig, ax = plt.subplots(figsize=(7, 5))
            scatter = ax.scatter(
                df["active_control_point_count"],
                df["new_factor_count"],
                c=df["solver_update_ms"],
                cmap="viridis",
                s=18,
                alpha=0.75,
            )
            ax.set_xlabel("active_control_point_count")
            ax.set_ylabel("new_factor_count")
            ax.set_title("Delta Size vs Active Window")
            ax.grid(True, alpha=0.3)
            cbar = fig.colorbar(scatter, ax=ax)
            cbar.set_label("solver_update_ms")
            fig.tight_layout()
            out_path = out_dir / "solver_delta_vs_active_window.png"
            fig.savefig(out_path, dpi=150)
            plt.close(fig)
            analysis["fig_solver_delta_vs_active_window"] = str(out_path)

    return analysis


def analyze_lidar_factor_internal(internal_df: pd.DataFrame | None, out_dir: Path, render_plots: bool = True) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if internal_df is None or internal_df.empty:
        return analysis

    df = internal_df.copy()
    analysis["available"] = True
    analysis["row_count"] = int(len(df))
    analysis["bucket_modes"] = sorted(df["bucket_mode"].dropna().astype(str).unique().tolist()) if "bucket_mode" in df else []
    for col in [
        "points_in_bucket",
        "valid_correspondence_count",
        "effective_residual_count",
        "factor_total_ms",
        "correspondence_ms",
        "match_ratio",
        "inlier_ratio",
        "best_distance_mean",
        "best_second_gap_mean",
    ]:
        if col in df.columns:
            series = df[col].fillna(0)
            analysis[f"{col}_mean"] = float(series.mean())
            analysis[f"{col}_p95"] = pct(series, 0.95)
            analysis[f"{col}_max"] = float(series.max())

    if {"factor_total_ms", "correspondence_ms"}.issubset(df.columns):
        total = df["factor_total_ms"].fillna(0).sum()
        corr = df["correspondence_ms"].fillna(0).sum()
        analysis["correspondence_share_of_factor_total"] = float(corr / total) if total > 0 else 0.0

    if "factor_total_ms" in df.columns:
        slow = df.sort_values("factor_total_ms", ascending=False).head(10)
        keep_cols = [
            "frame_id",
            "factor_index",
            "bucket_mode",
            "points_in_bucket",
            "valid_correspondence_count",
            "effective_residual_count",
            "factor_total_ms",
            "correspondence_ms",
            "match_ratio",
            "inlier_ratio",
        ]
        analysis["slow_factors"] = slow[[c for c in keep_cols if c in slow.columns]].to_dict(orient="records")

    if render_plots and {"points_in_bucket", "factor_total_ms"}.issubset(df.columns):
        ensure_dir(out_dir)

        fig, ax = plt.subplots(figsize=(7, 5))
        scatter = ax.scatter(
            df["points_in_bucket"],
            df["factor_total_ms"],
            c=df["valid_correspondence_count"] if "valid_correspondence_count" in df else "#4c78a8",
            cmap="viridis",
            s=18,
            alpha=0.75,
        )
        ax.set_xlabel("points_in_bucket")
        ax.set_ylabel("factor_total_ms")
        ax.set_title("LiDAR Factor Internal Load")
        ax.grid(True, alpha=0.3)
        if "valid_correspondence_count" in df:
            cbar = fig.colorbar(scatter, ax=ax)
            cbar.set_label("valid_correspondence_count")
        fig.tight_layout()
        out_path = out_dir / "lidar_factor_internal_load.png"
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        analysis["fig_lidar_factor_internal_load"] = str(out_path)

    return analysis


def analyze_pipeline_timing(pipeline_df: pd.DataFrame, out_dir: Path, render_plots: bool = True) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if pipeline_df is None or pipeline_df.empty:
        return analysis

    df = pipeline_df.copy()
    analysis["available"] = True
    analysis["row_count"] = int(len(df))
    modules = sorted(df["module"].dropna().astype(str).unique().tolist()) if "module" in df else []
    analysis["modules"] = modules
    module_stats = []
    for mod in modules:
        sub = df[df["module"] == mod]["elapsed_ms"]
        module_stats.append({
            "module": mod,
            "count": int(len(sub)),
            "mean_ms": float(sub.mean()),
            "p95_ms": pct(sub, 0.95),
            "max_ms": float(sub.max()),
        })
    module_stats.sort(key=lambda item: item["mean_ms"], reverse=True)
    analysis["module_stats"] = module_stats

    if render_plots:
        ensure_dir(out_dir)
        if modules:
            fig, ax = plt.subplots(figsize=(12, 5))
            for mod in modules[:8]:
                sub = df[df["module"] == mod].copy()
                x = np.arange(len(sub))
                ax.plot(x, sub["elapsed_ms"], label=mod, linewidth=0.9)
            ax.set_title("Pipeline Module Timing")
            ax.set_xlabel("sample index")
            ax.set_ylabel("elapsed_ms")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8, ncol=4)
            fig.tight_layout()
            out_path = out_dir / "pipeline_module_timing.png"
            fig.savefig(out_path, dpi=150)
            plt.close(fig)
            analysis["fig_pipeline_module_timing"] = str(out_path)
    return analysis


def analyze_jump_diagnostics(
    jump_df: pd.DataFrame | None,
    runtime_final_pose_surface: str = "active_window",
    experiment_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if jump_df is None or jump_df.empty:
        return analysis

    df = jump_df.copy()
    metric_columns = [
        "frame_stamp",
        "raw_frame_stamp",
        "scan_begin_time",
        "scan_end_time",
        "representative_time",
        "bucket_representative_time",
        "start_pose_query_time",
        "frontend_pose_query_time",
        "frontend_seed_fallback_used",
        "frontend_seed_imu_sample_count",
        "delta_start_to_frontend_translation_norm",
        "delta_start_to_frontend_rotation_rad",
        "delta_start_to_frontend_pitch_rad",
        "delta_start_to_frontend_roll_rad",
        "delta_frontend_to_postsolve_query_translation_norm",
        "delta_frontend_to_postsolve_query_rotation_rad",
        "delta_frontend_to_postsolve_query_pitch_rad",
        "delta_frontend_to_postsolve_query_roll_rad",
        "delta_frontend_to_postsolve_strict_local_translation_norm",
        "delta_frontend_to_postsolve_strict_local_rotation_rad",
        "delta_frontend_to_postsolve_strict_local_pitch_rad",
        "delta_frontend_to_postsolve_strict_local_roll_rad",
        "delta_postsolve_query_to_final_translation_norm",
        "delta_postsolve_query_to_final_rotation_rad",
        "delta_postsolve_query_to_final_pitch_rad",
        "delta_postsolve_query_to_final_roll_rad",
        "delta_postsolve_strict_local_to_final_translation_norm",
        "delta_postsolve_strict_local_to_final_rotation_rad",
        "delta_postsolve_strict_local_to_final_pitch_rad",
        "delta_postsolve_strict_local_to_final_roll_rad",
        "delta_postsolve_active_window_to_postsolve_strict_local_translation_norm",
        "delta_postsolve_active_window_to_postsolve_strict_local_rotation_rad",
        "delta_postsolve_active_window_to_postsolve_strict_local_pitch_rad",
        "delta_postsolve_active_window_to_postsolve_strict_local_roll_rad",
        "delta_frontend_to_final_translation_norm",
        "delta_frontend_to_final_rotation_rad",
        "delta_frontend_to_final_yaw_rad",
        "delta_frontend_to_final_pitch_rad",
        "delta_frontend_to_final_roll_rad",
        "delta_frontend_to_final_dx",
        "delta_frontend_to_final_dy",
        "delta_frontend_to_final_dz",
        "current_velocity_norm",
        "current_velocity_heading_rad",
        "current_velocity_heading_valid",
        "velocity_factor_count",
        "prior_factor_count",
        "uses_shared_imu_state",
        "frontend_world_to_lidar_yaw",
        "frontend_world_to_imu_yaw",
        "final_world_to_lidar_yaw",
        "final_world_to_imu_yaw",
        "lidar_to_imu_extrinsic_yaw",
        "gyro_bias_norm",
        "accel_bias_norm",
        "gravity_world_x",
        "gravity_world_y",
        "gravity_world_z",
        "gravity_dir_tilt_rad",
        "start_pose_frozen_before_factor_injection",
        "start_pose_frozen_before_solver_update",
        "start_pose_support_key_count",
        "start_pose_support_mismatch_flag",
        "postsolve_query_support_key_count",
        "postsolve_strict_local_support_key_count",
        "strict_local_query_support_key_count",
        "match_ratio",
        "inlier_ratio",
        "points_in_bucket",
        "candidate_correspondence_count",
        "accepted_correspondence_count",
        "accept_ratio",
        "registration_delta_translation_norm",
        "registration_delta_rotation_rad",
        "target_point_count",
        "target_voxel_count",
        "target_snapshot_clone_ms",
        "target_voxel_lookup_prep_ms",
        "target_covariance_prep_ms",
        "source_to_target_transform_ms",
        "factor_total_ms",
        "solver_update_ms",
        "reeliminated_variable_count",
        "relinearized_pose_variable_count",
        "relinearized_aux_variable_count",
        "relinearized_shared_variable_count",
        "recalculated_velocity_factor_count",
        "recalculated_prior_factor_count",
        "recalculated_imu_factor_count",
        "recalculated_lidar_factor_count",
        "recalculated_lidar_same_support_factor_count",
        "recalculated_lidar_cross_support_factor_count",
        "recalculated_lidar_current_segment_factor_count",
    ]
    for column in metric_columns:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

    if (
        "strict_local_query_support_key_count" not in df.columns
        and "postsolve_strict_local_support_key_count" in df.columns
    ):
        df["strict_local_query_support_key_count"] = pd.to_numeric(
            df["postsolve_strict_local_support_key_count"], errors="coerce"
        )

    analysis["available"] = True
    analysis["jump_rows"] = int(len(df))
    analysis["runtime_final_pose_surface"] = runtime_final_pose_surface
    experiment_config = experiment_config or {}
    analysis["runtime_experiment_name"] = (
        experiment_config.get("runtime_experiment_name")
        or experiment_config.get("experiment_name")
        or "baseline"
    )
    analysis["runtime_frontend_seed_mode"] = (
        experiment_config.get("runtime_frontend_seed_mode")
        or experiment_config.get("config_frontend_seed_mode")
        or "last_pose_copy"
    )
    analysis["runtime_frontend_seed_source"] = (
        experiment_config.get("runtime_frontend_seed_source")
        or "last_pose_copy"
    )
    analysis["runtime_frontend_seed_fallback_used"] = maybe_bool(
        experiment_config.get("runtime_frontend_seed_fallback_used")
    )
    analysis["runtime_frontend_seed_imu_sample_count"] = experiment_config.get(
        "runtime_frontend_seed_imu_sample_count"
    )
    analysis["runtime_gravity_mode"] = experiment_config.get("runtime_gravity_mode", "normal")
    analysis["runtime_gravity_fixed_norm_value"] = experiment_config.get("runtime_gravity_fixed_norm_value")
    analysis["runtime_gravity_tilt_limit_rad"] = experiment_config.get("runtime_gravity_tilt_limit_rad")
    analysis["runtime_gravity_warmup_freeze_frames"] = experiment_config.get("runtime_gravity_warmup_freeze_frames")
    for key in [
        "runtime_exp_freeze_gravity",
        "runtime_exp_freeze_gyro_bias",
        "runtime_exp_freeze_accel_bias",
        "runtime_exp_disable_velocity_factor",
        "runtime_exp_disable_current_velocity_prior",
    ]:
        analysis[key] = maybe_bool(experiment_config.get(key))

    string_columns = [
        "start_pose_source_kind",
        "frontend_seed_mode",
        "frontend_seed_source",
        "start_pose_support_keys_summary",
        "start_pose_support_mismatch_reason",
        "lidar_support_keys_summary",
        "frontend_pose_support_keys_summary",
        "frontend_pose_query_support_keys_summary",
        "postsolve_query_support_keys_summary",
        "postsolve_query_layout_name",
        "postsolve_query_support_mismatch_reason",
        "postsolve_strict_local_support_keys_summary",
        "postsolve_strict_local_layout_name",
        "postsolve_strict_local_support_mismatch_reason",
        "strict_local_query_support_keys_summary",
        "strict_local_query_reason",
        "yaw_chain_consistency_flag",
        "carried_boundary_oldest_key_summary",
        "oldest_survivor_key_summary",
    ]
    for column in string_columns:
        if column in df.columns:
            df[column] = df[column].fillna("").astype(str)

    if (
        "strict_local_query_support_keys_summary" not in df.columns
        and "postsolve_strict_local_support_keys_summary" in df.columns
    ):
        df["strict_local_query_support_keys_summary"] = (
            df["postsolve_strict_local_support_keys_summary"].fillna("").astype(str)
        )
    if "strict_local_query_reason" not in df.columns:
        if "postsolve_strict_local_support_mismatch_reason" in df.columns:
            df["strict_local_query_reason"] = (
                df["postsolve_strict_local_support_mismatch_reason"]
                .fillna("")
                .astype(str)
                .replace(
                    {
                        "support_keys_different": "support_mismatch",
                        "layout_unavailable": "other",
                        "query_time_outside_layout": "other",
                    }
                )
            )
        else:
            df["strict_local_query_reason"] = "other"

    def summarize_series(series: pd.Series, prefix: str) -> None:
        clean = pd.to_numeric(series, errors="coerce").dropna()
        if clean.empty:
            return
        analysis[f"{prefix}_mean"] = float(clean.mean())
        analysis[f"{prefix}_p95"] = pct(clean, 0.95)
        analysis[f"{prefix}_max"] = float(clean.max())

    def summarize_abs_series(series: pd.Series, prefix: str) -> None:
        clean = pd.to_numeric(series, errors="coerce").dropna().abs()
        if clean.empty:
            return
        analysis[f"{prefix}_abs_mean"] = float(clean.mean())
        analysis[f"{prefix}_abs_p95"] = pct(clean, 0.95)
        analysis[f"{prefix}_abs_max"] = float(clean.max())

    def safe_corr(
        frame: pd.DataFrame,
        lhs: str,
        rhs: str,
        *,
        lhs_abs: bool = False,
    ) -> float | None:
        if lhs not in frame.columns or rhs not in frame.columns:
            return None
        lhs_series = pd.to_numeric(frame[lhs], errors="coerce")
        rhs_series = pd.to_numeric(frame[rhs], errors="coerce")
        if lhs_abs:
            lhs_series = lhs_series.abs()
        pair = (
            pd.DataFrame({"lhs": lhs_series, "rhs": rhs_series})
            .replace([np.inf, -np.inf], np.nan)
            .dropna()
        )
        if len(pair) < 2 or pair["lhs"].nunique() < 2 or pair["rhs"].nunique() < 2:
            return None
        corr_value = pair["lhs"].corr(pair["rhs"], method="pearson")
        if pd.isna(corr_value):
            return None
        return float(corr_value)

    def normalize_angle_rad(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def angle_diff_series(lhs: pd.Series, rhs: pd.Series) -> pd.Series:
        lhs_numeric = pd.to_numeric(lhs, errors="coerce")
        rhs_numeric = pd.to_numeric(rhs, errors="coerce")
        raw = lhs_numeric - rhs_numeric
        return raw.map(
            lambda value: normalize_angle_rad(float(value)) if pd.notna(value) else math.nan
        )

    def sorted_frame_delta(
        frame: pd.DataFrame,
        column: str,
        *,
        angular: bool = False,
        valid_column: str | None = None,
    ) -> pd.Series:
        if column not in frame.columns:
            return pd.Series(np.nan, index=frame.index, dtype=float)

        sort_columns = ["frame_stamp"] if "frame_stamp" in frame.columns else []
        if "frame_id" in frame.columns:
            sort_columns.append("frame_id")
        ordered = frame.sort_values(sort_columns) if sort_columns else frame.copy()
        current = pd.to_numeric(ordered[column], errors="coerce")
        previous = current.shift(1)
        if angular:
            delta = angle_diff_series(current, previous)
        else:
            delta = current - previous

        valid_mask = current.notna() & previous.notna()
        if valid_column and valid_column in ordered.columns:
            valid_values = pd.to_numeric(ordered[valid_column], errors="coerce").fillna(0.0) != 0.0
            valid_mask &= valid_values & valid_values.shift(1, fill_value=False)
        delta = delta.where(valid_mask, np.nan)
        return delta.reindex(frame.index)

    def numeric_mean(frame: pd.DataFrame, column: str) -> float:
        if column not in frame.columns:
            return math.nan
        clean = pd.to_numeric(frame[column], errors="coerce").dropna()
        if clean.empty:
            return math.nan
        return float(clean.mean())

    analysis["solver_update_ms_mean"] = numeric_mean(df, "solver_update_ms")
    analysis["accept_ratio_mean"] = numeric_mean(df, "accept_ratio")
    analysis["match_ratio_mean"] = numeric_mean(df, "match_ratio")
    analysis["inlier_ratio_mean"] = numeric_mean(df, "inlier_ratio")
    analysis["target_point_count_mean"] = numeric_mean(df, "target_point_count")
    analysis["target_voxel_count_mean"] = numeric_mean(df, "target_voxel_count")

    def top_frame_score(translation_value: Any, rotation_value: Any) -> float:
        translation = float(translation_value or 0.0)
        rotation = float(rotation_value or 0.0)
        return max(translation / 1.0, rotation / 0.3)

    def is_nonempty_text(value: Any) -> bool:
        return isinstance(value, str) and value.strip() != ""

    summary_specs = [
        ("delta_start_to_frontend_translation_norm", "start_to_frontend_translation"),
        ("delta_start_to_frontend_rotation_rad", "start_to_frontend_rotation"),
        ("delta_frontend_to_postsolve_query_translation_norm", "frontend_to_postsolve_query_translation"),
        ("delta_frontend_to_postsolve_query_rotation_rad", "frontend_to_postsolve_query_rotation"),
        ("delta_frontend_to_postsolve_strict_local_translation_norm", "frontend_to_postsolve_strict_local_translation"),
        ("delta_frontend_to_postsolve_strict_local_rotation_rad", "frontend_to_postsolve_strict_local_rotation"),
        ("delta_postsolve_query_to_final_translation_norm", "postsolve_query_to_final_translation"),
        ("delta_postsolve_query_to_final_rotation_rad", "postsolve_query_to_final_rotation"),
        ("delta_postsolve_strict_local_to_final_translation_norm", "postsolve_strict_local_to_final_translation"),
        ("delta_postsolve_strict_local_to_final_rotation_rad", "postsolve_strict_local_to_final_rotation"),
        (
            "delta_postsolve_active_window_to_postsolve_strict_local_translation_norm",
            "postsolve_active_window_to_postsolve_strict_local_translation",
        ),
        (
            "delta_postsolve_active_window_to_postsolve_strict_local_rotation_rad",
            "postsolve_active_window_to_postsolve_strict_local_rotation",
        ),
        ("delta_frontend_to_final_translation_norm", "frontend_to_final_translation"),
        ("delta_frontend_to_final_rotation_rad", "frontend_to_final_rotation"),
    ]
    for column, prefix in summary_specs:
        if column in df.columns:
            summarize_series(df[column], prefix)

    strict_local_residual_specs = [
        ("delta_frontend_to_final_yaw_rad", "frontend_to_final_yaw"),
        ("delta_frontend_to_final_pitch_rad", "frontend_to_final_pitch"),
        ("delta_frontend_to_final_roll_rad", "frontend_to_final_roll"),
        ("delta_frontend_to_final_dx", "frontend_to_final_dx"),
        ("delta_frontend_to_final_dy", "frontend_to_final_dy"),
        ("delta_frontend_to_final_dz", "frontend_to_final_dz"),
    ]
    for column, prefix in strict_local_residual_specs:
        if column in df.columns:
            summarize_series(df[column], prefix)
            summarize_abs_series(df[column], prefix)

    if {"delta_frontend_to_final_dx", "delta_frontend_to_final_dy"} <= set(df.columns):
        xy_norm = np.hypot(
            pd.to_numeric(df["delta_frontend_to_final_dx"], errors="coerce"),
            pd.to_numeric(df["delta_frontend_to_final_dy"], errors="coerce"),
        )
        summarize_series(pd.Series(xy_norm), "frontend_to_final_xy_norm")
    if "delta_frontend_to_final_dz" in df.columns:
        summarize_abs_series(df["delta_frontend_to_final_dz"], "frontend_to_final_dz")
        summarize_abs_series(df["delta_frontend_to_final_dz"], "frontend_to_final_abs_dz")

    if {
        "frontend_world_to_lidar_yaw",
        "frontend_world_to_imu_yaw",
        "final_world_to_lidar_yaw",
        "final_world_to_imu_yaw",
    } <= set(df.columns):
        frontend_lidar_delta = angle_diff_series(
            df["final_world_to_lidar_yaw"], df["frontend_world_to_lidar_yaw"]
        )
        frontend_imu_delta = angle_diff_series(
            df["final_world_to_imu_yaw"], df["frontend_world_to_imu_yaw"]
        )
        df["yaw_space_gap_rad"] = angle_diff_series(frontend_imu_delta, frontend_lidar_delta).abs()
        summarize_series(df["yaw_space_gap_rad"], "yaw_space_gap")
        summarize_abs_series(df["yaw_space_gap_rad"], "yaw_space_gap")

    derived_delta_specs = [
        ("current_velocity_heading_rad", "delta_velocity_heading_rad", True, "current_velocity_heading_valid"),
        ("final_world_to_lidar_yaw", "world_to_lidar_yaw_shift", True, None),
        ("final_world_to_imu_yaw", "world_to_imu_yaw_shift", True, None),
        ("gyro_bias_norm", "delta_gyro_bias_norm", False, None),
        ("accel_bias_norm", "delta_accel_bias_norm", False, None),
        ("gravity_dir_tilt_rad", "delta_gravity_dir_rad", True, None),
    ]
    for source_column, out_column, angular, valid_column in derived_delta_specs:
        if source_column in df.columns:
            df[out_column] = sorted_frame_delta(
                df,
                source_column,
                angular=angular,
                valid_column=valid_column,
            )

    alias_prefixes = [
        ("frontend_to_postsolve_query_translation", "frontend_to_postsolve_active_window_translation"),
        ("frontend_to_postsolve_query_rotation", "frontend_to_postsolve_active_window_rotation"),
        ("postsolve_query_to_final_translation", "postsolve_active_window_to_final_translation"),
        ("postsolve_query_to_final_rotation", "postsolve_active_window_to_final_rotation"),
    ]
    for source_prefix, alias_prefix in alias_prefixes:
        for suffix in ["mean", "p95", "max"]:
            key = f"{source_prefix}_{suffix}"
            if key in analysis:
                analysis[f"{alias_prefix}_{suffix}"] = analysis[key]

    pitch_roll_stage_specs = [
        ("delta_start_to_frontend_pitch_rad", "start_to_frontend_pitch"),
        ("delta_start_to_frontend_roll_rad", "start_to_frontend_roll"),
        ("delta_frontend_to_postsolve_query_pitch_rad", "frontend_to_postsolve_query_pitch"),
        ("delta_frontend_to_postsolve_query_roll_rad", "frontend_to_postsolve_query_roll"),
        ("delta_frontend_to_postsolve_strict_local_pitch_rad", "frontend_to_postsolve_strict_local_pitch"),
        ("delta_frontend_to_postsolve_strict_local_roll_rad", "frontend_to_postsolve_strict_local_roll"),
        ("delta_postsolve_query_to_final_pitch_rad", "postsolve_query_to_final_pitch"),
        ("delta_postsolve_query_to_final_roll_rad", "postsolve_query_to_final_roll"),
        ("delta_postsolve_strict_local_to_final_pitch_rad", "postsolve_strict_local_to_final_pitch"),
        ("delta_postsolve_strict_local_to_final_roll_rad", "postsolve_strict_local_to_final_roll"),
        (
            "delta_postsolve_active_window_to_postsolve_strict_local_pitch_rad",
            "postsolve_active_window_to_postsolve_strict_local_pitch",
        ),
        (
            "delta_postsolve_active_window_to_postsolve_strict_local_roll_rad",
            "postsolve_active_window_to_postsolve_strict_local_roll",
        ),
    ]
    for column, prefix in pitch_roll_stage_specs:
        if column in df.columns:
            summarize_series(df[column], prefix)
            summarize_abs_series(df[column], prefix)

    if "start_pose_source_kind" in df.columns:
        source_counts = df["start_pose_source_kind"].replace({"": "unknown"}).value_counts()
        analysis["start_pose_source_kind_counts"] = {
            str(key): int(value) for key, value in source_counts.items()
        }
    if "frontend_seed_source" in df.columns:
        seed_source_counts = df["frontend_seed_source"].replace({"": "unknown"}).value_counts()
        analysis["frontend_seed_source_counts"] = {
            str(key): int(value) for key, value in seed_source_counts.items()
        }
    if "frontend_seed_fallback_used" in df.columns:
        fallback_values = pd.to_numeric(df["frontend_seed_fallback_used"], errors="coerce").fillna(0.0)
        analysis["seed_fallback_frame_count"] = int((fallback_values != 0).sum())
        analysis["seed_fallback_frame_ratio"] = (
            analysis["seed_fallback_frame_count"] / len(df)
            if len(df) > 0
            else 0.0
        )
    if "frontend_seed_imu_sample_count" in df.columns:
        summarize_series(df["frontend_seed_imu_sample_count"], "frontend_seed_imu_sample_count")

    for flag_column in [
        "start_pose_frozen_before_factor_injection",
        "start_pose_frozen_before_solver_update",
    ]:
        if flag_column not in df.columns:
            continue
        values = pd.to_numeric(df[flag_column], errors="coerce").fillna(0.0)
        analysis[f"{flag_column}_counts"] = {
            "true": int((values != 0).sum()),
            "false": int((values == 0).sum()),
        }

    if "start_pose_support_mismatch_reason" in df.columns:
        reason_counts = (
            df["start_pose_support_mismatch_reason"]
            .replace({"": "none"})
            .value_counts()
        )
        analysis["start_pose_support_mismatch_reason_counts"] = {
            str(key): int(value) for key, value in reason_counts.items()
        }

    if "strict_local_query_reason" in df.columns:
        strict_local_reason_counts = (
            df["strict_local_query_reason"].replace({"": "other"}).value_counts()
        )
        analysis["strict_local_query_reason_counts"] = {
            str(key): int(value) for key, value in strict_local_reason_counts.items()
        }

    time_delta_specs = [
        (
            "start_pose_query_time",
            "representative_time",
            "start_pose_query_time_minus_representative_time",
        ),
        (
            "frontend_pose_query_time",
            "representative_time",
            "frontend_pose_query_time_minus_representative_time",
        ),
        (
            "start_pose_query_time",
            "frontend_pose_query_time",
            "start_pose_query_time_minus_frontend_pose_query_time",
        ),
    ]
    for lhs, rhs, prefix in time_delta_specs:
        if lhs not in df.columns or rhs not in df.columns:
            continue
        delta = pd.to_numeric(df[lhs], errors="coerce") - pd.to_numeric(df[rhs], errors="coerce")
        summarize_series(delta, prefix)
        summarize_abs_series(delta, prefix)

    def top_frames(column: str, limit: int = 10) -> list[dict[str, Any]]:
        if column not in df.columns:
            return []
        keep_columns = [
            "frame_id",
            "frame_stamp",
            "raw_frame_stamp",
            "scan_begin_time",
            "scan_end_time",
            "representative_time",
            "bucket_representative_time",
            "start_pose_query_time",
            "frontend_pose_query_time",
            "current_segment_id",
            "start_pose_source_kind",
            "frontend_seed_mode",
            "frontend_seed_source",
            "frontend_seed_fallback_used",
            "frontend_seed_imu_sample_count",
            "start_pose_frozen_before_factor_injection",
            "start_pose_frozen_before_solver_update",
            "delta_start_to_frontend_translation_norm",
            "delta_start_to_frontend_rotation_rad",
            "delta_start_to_frontend_pitch_rad",
            "delta_start_to_frontend_roll_rad",
            "delta_frontend_to_postsolve_query_translation_norm",
            "delta_frontend_to_postsolve_query_rotation_rad",
            "delta_frontend_to_postsolve_query_pitch_rad",
            "delta_frontend_to_postsolve_query_roll_rad",
            "delta_frontend_to_postsolve_strict_local_translation_norm",
            "delta_frontend_to_postsolve_strict_local_rotation_rad",
            "delta_frontend_to_postsolve_strict_local_pitch_rad",
            "delta_frontend_to_postsolve_strict_local_roll_rad",
            "delta_postsolve_query_to_final_translation_norm",
            "delta_postsolve_query_to_final_rotation_rad",
            "delta_postsolve_query_to_final_pitch_rad",
            "delta_postsolve_query_to_final_roll_rad",
            "delta_postsolve_strict_local_to_final_translation_norm",
            "delta_postsolve_strict_local_to_final_rotation_rad",
            "delta_postsolve_strict_local_to_final_pitch_rad",
            "delta_postsolve_strict_local_to_final_roll_rad",
            "delta_postsolve_active_window_to_postsolve_strict_local_translation_norm",
            "delta_postsolve_active_window_to_postsolve_strict_local_rotation_rad",
            "delta_postsolve_active_window_to_postsolve_strict_local_pitch_rad",
            "delta_postsolve_active_window_to_postsolve_strict_local_roll_rad",
            "delta_frontend_to_final_translation_norm",
            "delta_frontend_to_final_rotation_rad",
            "delta_frontend_to_final_yaw_rad",
            "delta_frontend_to_final_pitch_rad",
            "delta_frontend_to_final_roll_rad",
            "delta_frontend_to_final_dx",
            "delta_frontend_to_final_dy",
            "delta_frontend_to_final_dz",
            "current_velocity_norm",
            "current_velocity_heading_rad",
            "current_velocity_heading_valid",
            "velocity_factor_count",
            "prior_factor_count",
            "uses_shared_imu_state",
            "frontend_world_to_lidar_yaw",
            "frontend_world_to_imu_yaw",
            "final_world_to_lidar_yaw",
            "final_world_to_imu_yaw",
            "lidar_to_imu_extrinsic_yaw",
            "yaw_chain_consistency_flag",
            "gyro_bias_norm",
            "accel_bias_norm",
            "gravity_world_x",
            "gravity_world_y",
            "gravity_world_z",
            "gravity_dir_tilt_rad",
            "lidar_layout_domain_begin",
            "lidar_layout_domain_end",
            "start_pose_support_key_count",
            "start_pose_support_keys_summary",
            "start_pose_support_mismatch_flag",
            "start_pose_support_mismatch_reason",
            "lidar_support_keys_summary",
            "frontend_pose_support_keys_summary",
            "frontend_pose_query_support_keys_summary",
            "postsolve_query_support_key_count",
            "postsolve_query_support_keys_summary",
            "postsolve_query_layout_name",
            "postsolve_query_support_mismatch_reason",
            "postsolve_strict_local_support_key_count",
            "postsolve_strict_local_support_keys_summary",
            "postsolve_strict_local_layout_name",
            "postsolve_strict_local_support_mismatch_reason",
            "strict_local_query_support_key_count",
            "strict_local_query_support_keys_summary",
            "strict_local_query_reason",
            "carried_boundary_oldest_key_summary",
            "oldest_survivor_key_summary",
            "solver_update_ms",
            "reeliminated_variable_count",
            "relinearized_pose_variable_count",
            "relinearized_aux_variable_count",
            "relinearized_shared_variable_count",
            "recalculated_velocity_factor_count",
            "recalculated_prior_factor_count",
            "recalculated_imu_factor_count",
            "recalculated_lidar_factor_count",
            "recalculated_lidar_same_support_factor_count",
            "recalculated_lidar_cross_support_factor_count",
            "recalculated_lidar_current_segment_factor_count",
            "match_ratio",
            "inlier_ratio",
            "points_in_bucket",
            "candidate_correspondence_count",
            "accepted_correspondence_count",
            "accept_ratio",
            "registration_delta_translation_norm",
            "registration_delta_rotation_rad",
            "target_point_count",
            "target_voxel_count",
            "target_snapshot_clone_ms",
            "target_voxel_lookup_prep_ms",
            "target_covariance_prep_ms",
            "source_to_target_transform_ms",
        ]
        available_columns = [c for c in keep_columns if c in df.columns]
        ranked = df.sort_values(column, ascending=False).head(limit)
        return ranked[available_columns].replace({np.nan: None}).to_dict(orient="records")

    analysis["top_start_to_frontend_translation_frames"] = top_frames(
        "delta_start_to_frontend_translation_norm"
    )
    analysis["top_frontend_to_final_translation_frames"] = top_frames(
        "delta_frontend_to_final_translation_norm"
    )

    top_start_df = (
        df.sort_values("delta_start_to_frontend_translation_norm", ascending=False).head(10)
        if "delta_start_to_frontend_translation_norm" in df.columns
        else pd.DataFrame()
    )
    top_final_df = (
        df.sort_values("delta_frontend_to_final_translation_norm", ascending=False).head(10)
        if "delta_frontend_to_final_translation_norm" in df.columns
        else pd.DataFrame()
    )
    top_final_rotation_df = (
        df.sort_values("delta_frontend_to_final_rotation_rad", ascending=False).head(10)
        if "delta_frontend_to_final_rotation_rad" in df.columns
        else pd.DataFrame()
    )
    top_yaw_df = (
        df.assign(
            delta_frontend_to_final_yaw_abs=lambda frame: pd.to_numeric(
                frame["delta_frontend_to_final_yaw_rad"], errors="coerce"
            ).abs()
        )
        .sort_values("delta_frontend_to_final_yaw_abs", ascending=False)
        .head(10)
        if "delta_frontend_to_final_yaw_rad" in df.columns
        else pd.DataFrame()
    )
    top_pitch_df = (
        df.assign(
            delta_frontend_to_final_pitch_abs=lambda frame: pd.to_numeric(
                frame["delta_frontend_to_final_pitch_rad"], errors="coerce"
            ).abs()
        )
        .sort_values("delta_frontend_to_final_pitch_abs", ascending=False)
        .head(10)
        if "delta_frontend_to_final_pitch_rad" in df.columns
        else pd.DataFrame()
    )
    top_roll_df = (
        df.assign(
            delta_frontend_to_final_roll_abs=lambda frame: pd.to_numeric(
                frame["delta_frontend_to_final_roll_rad"], errors="coerce"
            ).abs()
        )
        .sort_values("delta_frontend_to_final_roll_abs", ascending=False)
        .head(10)
        if "delta_frontend_to_final_roll_rad" in df.columns
        else pd.DataFrame()
    )

    top_strict_local_residual_df = pd.DataFrame()
    if not top_final_df.empty or not top_final_rotation_df.empty:
        combined = pd.concat([top_final_df, top_final_rotation_df], ignore_index=False)
        if "frame_id" in combined.columns:
            combined = combined.loc[~combined["frame_id"].duplicated(keep="first")]
        combined = combined.copy()
        combined["strict_local_residual_score"] = combined.apply(
            lambda row: top_frame_score(
                row.get("delta_frontend_to_final_translation_norm", 0.0),
                row.get("delta_frontend_to_final_rotation_rad", 0.0),
            ),
            axis=1,
        )
        top_strict_local_residual_df = combined.sort_values(
            "strict_local_residual_score", ascending=False
        ).head(20)

    top_yaw_residual_df = pd.DataFrame()
    if not top_yaw_df.empty or not top_final_rotation_df.empty:
        combined = pd.concat([top_yaw_df, top_final_rotation_df], ignore_index=False)
        if "frame_id" in combined.columns:
            combined = combined.loc[~combined["frame_id"].duplicated(keep="first")]
        combined = combined.copy()
        combined["yaw_residual_score"] = combined.apply(
            lambda row: max(
                abs(float(row.get("delta_frontend_to_final_yaw_rad", 0.0) or 0.0)) / 0.3,
                float(row.get("delta_frontend_to_final_rotation_rad", 0.0) or 0.0) / 0.3,
            ),
            axis=1,
        )
        top_yaw_residual_df = combined.sort_values("yaw_residual_score", ascending=False).head(20)

    if not top_start_df.empty and "start_pose_support_mismatch_reason" in top_start_df.columns:
        top_reason_counts = (
            top_start_df["start_pose_support_mismatch_reason"]
            .replace({"": "none"})
            .value_counts()
        )
        analysis["top_start_pose_support_mismatch_reason_counts"] = {
            str(key): int(value) for key, value in top_reason_counts.items()
        }

    if not top_start_df.empty:
        for column, prefix in [
            ("accept_ratio", "top_start_to_frontend_accept_ratio"),
            ("match_ratio", "top_start_to_frontend_match_ratio"),
            ("inlier_ratio", "top_start_to_frontend_inlier_ratio"),
            ("target_point_count", "top_start_to_frontend_target_point_count"),
            ("target_voxel_count", "top_start_to_frontend_target_voxel_count"),
            ("candidate_correspondence_count", "top_start_to_frontend_candidate_correspondence_count"),
            ("accepted_correspondence_count", "top_start_to_frontend_accepted_correspondence_count"),
        ]:
            if column in top_start_df.columns:
                summarize_series(top_start_df[column], prefix)

    if not top_final_df.empty and "postsolve_query_support_mismatch_reason" in top_final_df.columns:
        postsolve_reason_counts = (
            top_final_df["postsolve_query_support_mismatch_reason"]
            .replace({"": "none"})
            .value_counts()
        )
        analysis["top_postsolve_query_support_mismatch_reason_counts"] = {
            str(key): int(value) for key, value in postsolve_reason_counts.items()
        }
    if not top_final_df.empty and "postsolve_strict_local_support_mismatch_reason" in top_final_df.columns:
        postsolve_strict_reason_counts = (
            top_final_df["postsolve_strict_local_support_mismatch_reason"]
            .replace({"": "none"})
            .value_counts()
        )
        analysis["top_postsolve_strict_local_support_mismatch_reason_counts"] = {
            str(key): int(value) for key, value in postsolve_strict_reason_counts.items()
        }
    if not top_strict_local_residual_df.empty and "strict_local_query_reason" in top_strict_local_residual_df.columns:
        strict_local_top_reason_counts = (
            top_strict_local_residual_df["strict_local_query_reason"]
            .replace({"": "other"})
            .value_counts()
        )
        analysis["top_strict_local_query_reason_counts"] = {
            str(key): int(value) for key, value in strict_local_top_reason_counts.items()
        }
    if not top_yaw_residual_df.empty and "yaw_chain_consistency_flag" in top_yaw_residual_df.columns:
        top_yaw_chain_counts = (
            top_yaw_residual_df["yaw_chain_consistency_flag"]
            .replace({"": "other"})
            .value_counts()
        )
        analysis["top_yaw_chain_consistency_flag_counts"] = {
            str(key): int(value) for key, value in top_yaw_chain_counts.items()
        }
        inconsistent_top_yaw = int(
            top_yaw_chain_counts.drop(labels=["none"], errors="ignore").sum()
        )
        analysis["yaw_chain_inconsistency_count_on_top_frames"] = inconsistent_top_yaw
        analysis["top_yaw_chain_inconsistency_ratio"] = (
            inconsistent_top_yaw / len(top_yaw_residual_df)
            if len(top_yaw_residual_df) > 0
            else 0.0
        )

    if not top_strict_local_residual_df.empty:
        residual_keep_columns = [
            "frame_id",
            "frame_stamp",
            "delta_frontend_to_final_translation_norm",
            "delta_frontend_to_final_rotation_rad",
            "delta_frontend_to_final_yaw_rad",
            "delta_frontend_to_final_pitch_rad",
            "delta_frontend_to_final_roll_rad",
            "delta_frontend_to_final_dx",
            "delta_frontend_to_final_dy",
            "delta_frontend_to_final_dz",
            "current_velocity_norm",
            "current_velocity_heading_rad",
            "current_velocity_heading_valid",
            "velocity_factor_count",
            "prior_factor_count",
            "uses_shared_imu_state",
            "frontend_world_to_lidar_yaw",
            "frontend_world_to_imu_yaw",
            "final_world_to_lidar_yaw",
            "final_world_to_imu_yaw",
            "lidar_to_imu_extrinsic_yaw",
            "yaw_chain_consistency_flag",
            "gyro_bias_norm",
            "accel_bias_norm",
            "gravity_world_x",
            "gravity_world_y",
            "gravity_world_z",
            "gravity_dir_tilt_rad",
            "solver_update_ms",
            "reeliminated_variable_count",
            "relinearized_pose_variable_count",
            "relinearized_aux_variable_count",
            "relinearized_shared_variable_count",
            "recalculated_velocity_factor_count",
            "recalculated_prior_factor_count",
            "recalculated_imu_factor_count",
            "recalculated_lidar_factor_count",
            "recalculated_lidar_same_support_factor_count",
            "recalculated_lidar_cross_support_factor_count",
            "recalculated_lidar_current_segment_factor_count",
            "frontend_seed_mode",
            "frontend_seed_source",
            "frontend_seed_fallback_used",
            "frontend_seed_imu_sample_count",
            "strict_local_query_reason",
            "strict_local_query_support_keys_summary",
        ]
        available_residual_columns = [
            column for column in residual_keep_columns if column in top_strict_local_residual_df.columns
        ]
        analysis["top_strict_local_residual_frames"] = (
            top_strict_local_residual_df[available_residual_columns]
            .replace({np.nan: None})
            .to_dict(orient="records")
        )
    if not top_yaw_residual_df.empty:
        yaw_keep_columns = [
            "frame_id",
            "frame_stamp",
            "delta_frontend_to_final_translation_norm",
            "delta_frontend_to_final_rotation_rad",
            "delta_frontend_to_final_yaw_rad",
            "delta_frontend_to_final_pitch_rad",
            "delta_frontend_to_final_roll_rad",
            "delta_frontend_to_final_dx",
            "delta_frontend_to_final_dy",
            "delta_frontend_to_final_dz",
            "solver_update_ms",
            "reeliminated_variable_count",
            "relinearized_pose_variable_count",
            "relinearized_aux_variable_count",
            "relinearized_shared_variable_count",
            "recalculated_velocity_factor_count",
            "recalculated_prior_factor_count",
            "recalculated_imu_factor_count",
            "recalculated_lidar_factor_count",
            "recalculated_lidar_same_support_factor_count",
            "recalculated_lidar_cross_support_factor_count",
            "recalculated_lidar_current_segment_factor_count",
            "current_velocity_norm",
            "current_velocity_heading_rad",
            "current_velocity_heading_valid",
            "velocity_factor_count",
            "prior_factor_count",
            "uses_shared_imu_state",
            "frontend_world_to_lidar_yaw",
            "frontend_world_to_imu_yaw",
            "final_world_to_lidar_yaw",
            "final_world_to_imu_yaw",
            "lidar_to_imu_extrinsic_yaw",
            "yaw_chain_consistency_flag",
            "gyro_bias_norm",
            "accel_bias_norm",
            "gravity_world_x",
            "gravity_world_y",
            "gravity_world_z",
            "gravity_dir_tilt_rad",
            "frontend_seed_mode",
            "frontend_seed_source",
            "frontend_seed_fallback_used",
            "frontend_seed_imu_sample_count",
            "strict_local_query_reason",
        ]
        available_yaw_columns = [
            column for column in yaw_keep_columns if column in top_yaw_residual_df.columns
        ]
        analysis["top_yaw_residual_frames"] = (
            top_yaw_residual_df[available_yaw_columns]
            .replace({np.nan: None})
            .to_dict(orient="records")
        )
    if not top_pitch_df.empty:
        pitch_keep_columns = [
            "frame_id",
            "frame_stamp",
            "delta_frontend_to_final_pitch_rad",
            "delta_frontend_to_final_roll_rad",
            "delta_frontend_to_final_translation_norm",
            "solver_update_ms",
            "recalculated_imu_factor_count",
            "recalculated_prior_factor_count",
            "relinearized_shared_variable_count",
            "recalculated_lidar_factor_count",
            "match_ratio",
            "inlier_ratio",
            "target_point_count",
            "target_voxel_count",
            "frontend_seed_mode",
            "frontend_seed_source",
            "frontend_seed_fallback_used",
            "frontend_seed_imu_sample_count",
            "strict_local_query_reason",
            "postsolve_query_support_mismatch_reason",
        ]
        available_pitch_columns = [
            column for column in pitch_keep_columns if column in top_pitch_df.columns
        ]
        analysis["top_pitch_residual_frames"] = (
            top_pitch_df[available_pitch_columns]
            .replace({np.nan: None})
            .to_dict(orient="records")
        )
    if not top_roll_df.empty:
        roll_keep_columns = [
            "frame_id",
            "frame_stamp",
            "delta_frontend_to_final_pitch_rad",
            "delta_frontend_to_final_roll_rad",
            "delta_frontend_to_final_translation_norm",
            "solver_update_ms",
            "recalculated_imu_factor_count",
            "recalculated_prior_factor_count",
            "relinearized_shared_variable_count",
            "recalculated_lidar_factor_count",
            "match_ratio",
            "inlier_ratio",
            "target_point_count",
            "target_voxel_count",
            "frontend_seed_mode",
            "frontend_seed_source",
            "frontend_seed_fallback_used",
            "frontend_seed_imu_sample_count",
            "strict_local_query_reason",
            "postsolve_query_support_mismatch_reason",
        ]
        available_roll_columns = [
            column for column in roll_keep_columns if column in top_roll_df.columns
        ]
        analysis["top_roll_residual_frames"] = (
            top_roll_df[available_roll_columns]
            .replace({np.nan: None})
            .to_dict(orient="records")
        )

    def stage_score(translation_prefix: str, rotation_prefix: str) -> float:
        components = [
            float(analysis.get(f"{translation_prefix}_p95", 0.0) or 0.0),
            float(analysis.get(f"{translation_prefix}_max", 0.0) or 0.0),
            float(analysis.get(f"{rotation_prefix}_p95", 0.0) or 0.0) / 0.3,
            float(analysis.get(f"{rotation_prefix}_max", 0.0) or 0.0) / 0.3,
        ]
        finite_components = [value for value in components if math.isfinite(value)]
        return max(finite_components) if finite_components else 0.0

    frontend_score = stage_score("start_to_frontend_translation", "start_to_frontend_rotation")
    solver_score = stage_score("frontend_to_final_translation", "frontend_to_final_rotation")
    analysis["frontend_stage_score"] = frontend_score
    analysis["solver_stage_score"] = solver_score
    if frontend_score > 1.2 * max(solver_score, 1e-9) and frontend_score > 1.0:
        analysis["dominance"] = "jump appears frontend-dominated"
    elif solver_score > 1.2 * max(frontend_score, 1e-9) and solver_score > 1.0:
        analysis["dominance"] = "jump appears solver/boundary-dominated"
    elif max(frontend_score, solver_score) > 1.0:
        analysis["dominance"] = "jump appears mixed; frontend likely first cause"
    else:
        analysis["dominance"] = "no strong jump dominance detected"

    top_reason_counts = analysis.get("top_start_pose_support_mismatch_reason_counts", {})
    top_reason_total = max(sum(top_reason_counts.values()), 1)
    query_reason_share = top_reason_counts.get("query_time_layout_domain_mismatch", 0) / top_reason_total
    semantics_reason_share = (
        top_reason_counts.get("keys_mismatch", 0)
        + top_reason_counts.get("strict_layout_unavailable", 0)
    ) / top_reason_total
    frozen_false_count = (
        analysis.get("start_pose_frozen_before_factor_injection_counts", {}).get("false", 0)
        + analysis.get("start_pose_frozen_before_solver_update_counts", {}).get("false", 0)
    )
    overall_accept = numeric_mean(df, "accept_ratio")
    top_accept = numeric_mean(top_start_df, "accept_ratio")
    overall_target_points = numeric_mean(df, "target_point_count")
    top_target_points = numeric_mean(top_start_df, "target_point_count")
    overall_target_voxels = numeric_mean(df, "target_voxel_count")
    top_target_voxels = numeric_mean(top_start_df, "target_voxel_count")
    query_time_abs_p95 = float(
        analysis.get("start_pose_query_time_minus_frontend_pose_query_time_abs_p95", 0.0)
    )
    semantics_suspicious = semantics_reason_share >= 0.4 or frozen_false_count > 0
    query_suspicious = query_reason_share >= 0.4 or query_time_abs_p95 > 0.01
    quality_suspicious = (
        (math.isfinite(overall_accept) and math.isfinite(top_accept) and top_accept < 0.8 * max(overall_accept, 1e-6))
        or (math.isfinite(overall_target_points) and math.isfinite(top_target_points) and top_target_points < 0.8 * max(overall_target_points, 1.0))
        or (math.isfinite(overall_target_voxels) and math.isfinite(top_target_voxels) and top_target_voxels < 0.8 * max(overall_target_voxels, 1.0))
    )

    if query_suspicious and query_reason_share >= max(0.4, semantics_reason_share):
        analysis["frontend_jump_cause"] = "frontend jump appears query-time mismatch dominated"
    elif semantics_suspicious and quality_suspicious:
        analysis["frontend_jump_cause"] = "frontend jump appears mixed; start-pose and target quality both suspicious"
    elif semantics_suspicious:
        analysis["frontend_jump_cause"] = "frontend jump appears start-pose-semantics dominated"
    elif quality_suspicious:
        analysis["frontend_jump_cause"] = "frontend jump appears target/correspondence-quality dominated"
    else:
        analysis["frontend_jump_cause"] = "frontend jump cause not strongly isolated"

    evidence_parts: list[str] = []
    if top_reason_counts:
        evidence_parts.append(
            "top mismatch reasons="
            + ", ".join(f"{key}:{value}" for key, value in top_reason_counts.items())
        )
    if math.isfinite(top_accept):
        evidence_parts.append(f"top accept_ratio_mean={top_accept:.3f}")
    if math.isfinite(top_target_points):
        evidence_parts.append(f"top target_point_count_mean={top_target_points:.1f}")
    if math.isfinite(top_target_voxels):
        evidence_parts.append(f"top target_voxel_count_mean={top_target_voxels:.1f}")
    if query_time_abs_p95 > 0.0:
        evidence_parts.append(
            f"|start_query-frontend_query| p95={query_time_abs_p95:.6f}"
        )
    analysis["frontend_jump_cause_evidence"] = "; ".join(evidence_parts)

    top_postsolve_reason_counts = analysis.get("top_postsolve_query_support_mismatch_reason_counts", {})
    top_postsolve_reason_total = max(sum(top_postsolve_reason_counts.values()), 1)
    boundary_reason_share = top_postsolve_reason_counts.get("boundary_shift", 0) / top_postsolve_reason_total
    support_diff_share = (
        top_postsolve_reason_counts.get("boundary_shift", 0)
        + top_postsolve_reason_counts.get("support_keys_different", 0)
    ) / top_postsolve_reason_total
    top_postsolve_strict_reason_counts = analysis.get(
        "top_postsolve_strict_local_support_mismatch_reason_counts",
        {},
    )

    assignment_negligible = (
        float(analysis.get("postsolve_query_to_final_translation_p95", 0.0)) < 0.1
        and float(analysis.get("postsolve_query_to_final_translation_max", 0.0)) < 0.5
        and float(analysis.get("postsolve_query_to_final_rotation_p95", 0.0)) < 0.03
        and float(analysis.get("postsolve_query_to_final_rotation_max", 0.0)) < 0.1
    )

    churn_columns = [
        "solver_update_ms",
        "reeliminated_variable_count",
        "relinearized_pose_variable_count",
        "relinearized_aux_variable_count",
        "relinearized_shared_variable_count",
        "recalculated_velocity_factor_count",
        "recalculated_prior_factor_count",
        "recalculated_imu_factor_count",
        "recalculated_lidar_cross_support_factor_count",
    ]
    elevated_churn_columns: list[str] = []
    for column in churn_columns:
        if column not in df.columns or top_final_df.empty:
            continue
        overall = pd.to_numeric(df[column], errors="coerce").dropna()
        top = pd.to_numeric(top_final_df[column], errors="coerce").dropna()
        if overall.empty or top.empty:
            continue
        overall_p95 = pct(overall, 0.95)
        overall_median = float(overall.median())
        top_mean = float(top.mean())
        analysis[f"top_frontend_to_final_{column}_mean"] = top_mean
        if top_mean > 0.0 and top_mean > max(overall_p95, overall_median) * 1.05:
            elevated_churn_columns.append(column)

    solver_churn_suspicious = len(elevated_churn_columns) >= 2
    cross_support_elevated = "recalculated_lidar_cross_support_factor_count" in elevated_churn_columns
    boundary_key_present = (
        not top_final_df.empty
        and (
            top_final_df.get("carried_boundary_oldest_key_summary", pd.Series(dtype=str)).map(is_nonempty_text).any()
            or top_final_df.get("oldest_survivor_key_summary", pd.Series(dtype=str)).map(is_nonempty_text).any()
        )
    )
    boundary_amplified = (
        (boundary_reason_share >= 0.3 or support_diff_share >= 0.5)
        and (cross_support_elevated or boundary_key_present)
    )

    active_window_score = stage_score(
        "frontend_to_postsolve_query_translation",
        "frontend_to_postsolve_query_rotation",
    )
    strict_local_score = stage_score(
        "frontend_to_postsolve_strict_local_translation",
        "frontend_to_postsolve_strict_local_rotation",
    )
    active_vs_strict_score = stage_score(
        "postsolve_active_window_to_postsolve_strict_local_translation",
        "postsolve_active_window_to_postsolve_strict_local_rotation",
    )
    analysis["frontend_to_postsolve_active_window_stage_score"] = active_window_score
    analysis["frontend_to_postsolve_strict_local_stage_score"] = strict_local_score
    analysis["postsolve_active_window_to_postsolve_strict_local_stage_score"] = active_vs_strict_score

    surface_dominated = (
        assignment_negligible
        and active_window_score > 1.0
        and strict_local_score < active_window_score * 0.35
        and active_vs_strict_score > 1.0
    )
    solver_result_dominated = (
        active_window_score > 1.0
        and strict_local_score > 1.0
        and strict_local_score >= active_window_score * 0.7
    )
    mixed_surface_and_solver = (
        active_window_score > 1.0
        and strict_local_score > 1.0
        and strict_local_score < active_window_score * 0.7
    )

    if surface_dominated:
        analysis["frontend_to_final_jump_cause"] = "frontend->final jump appears active-window postsolve-surface dominated"
    elif solver_result_dominated:
        analysis["frontend_to_final_jump_cause"] = "frontend->final jump appears solver-result dominated"
    elif mixed_surface_and_solver:
        analysis["frontend_to_final_jump_cause"] = "frontend->final jump appears mixed; strict-local postsolve still far from frontend"
    else:
        analysis["frontend_to_final_jump_cause"] = "frontend->final jump cause not strongly isolated"

    postsolve_evidence: list[str] = []
    if top_postsolve_reason_counts:
        postsolve_evidence.append(
            "top active-window reasons="
            + ", ".join(f"{key}:{value}" for key, value in top_postsolve_reason_counts.items())
        )
    if top_postsolve_strict_reason_counts:
        postsolve_evidence.append(
            "top strict-local reasons="
            + ", ".join(f"{key}:{value}" for key, value in top_postsolve_strict_reason_counts.items())
        )
    postsolve_evidence.append(
        f"frontend->postsolve_active_window p95={analysis.get('frontend_to_postsolve_query_translation_p95', 0.0):.3f} m"
    )
    postsolve_evidence.append(
        f"frontend->postsolve_strict_local p95={analysis.get('frontend_to_postsolve_strict_local_translation_p95', 0.0):.3f} m"
    )
    postsolve_evidence.append(
        f"active_window->strict_local p95={analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_p95', 0.0):.3f} m"
    )
    postsolve_evidence.append(
        f"postsolve_active_window->final p95={analysis.get('postsolve_query_to_final_translation_p95', 0.0):.3f} m"
    )
    if boundary_amplified:
        postsolve_evidence.append("boundary/carry markers elevated")
    if elevated_churn_columns:
        postsolve_evidence.append(
            "elevated churn=" + ", ".join(elevated_churn_columns)
        )
    analysis["frontend_to_final_jump_cause_evidence"] = "; ".join(postsolve_evidence)

    final_score = stage_score(
        "frontend_to_final_translation",
        "frontend_to_final_rotation",
    )
    analysis["frontend_to_final_stage_score"] = final_score
    if runtime_final_pose_surface == "strict_local":
        strict_local_surface_reduces_jump = (
            active_window_score > 1.0
            and final_score < active_window_score * 0.35
            and active_vs_strict_score > 1.0
        )
        strict_local_surface_partial_mitigation = (
            active_window_score > 1.0
            and final_score > 1.0
            and final_score < active_window_score * 0.7
        )
        if strict_local_surface_reduces_jump:
            analysis["final_pose_surface_effect"] = "strict-local final pose surface significantly reduces frontend->final jump"
        elif strict_local_surface_partial_mitigation:
            analysis["final_pose_surface_effect"] = "final pose surface switch mitigates jump but solver-result residual remains"
        elif final_score > 1.0:
            analysis["final_pose_surface_effect"] = "strict-local final pose surface does not sufficiently reduce jump"
        else:
            analysis["final_pose_surface_effect"] = "strict-local final pose surface keeps frontend->final jump low"

        effect_evidence = [
            f"runtime_final_pose_surface={runtime_final_pose_surface}",
            f"frontend->final p95={analysis.get('frontend_to_final_translation_p95', 0.0):.3f} m",
            f"frontend->final rot_p95={analysis.get('frontend_to_final_rotation_p95', 0.0):.3f} rad",
            f"frontend->postsolve_active_window p95={analysis.get('frontend_to_postsolve_query_translation_p95', 0.0):.3f} m",
            f"postsolve_active_window->strict_local p95={analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_p95', 0.0):.3f} m",
        ]
        analysis["final_pose_surface_effect_evidence"] = "; ".join(effect_evidence)

    def experiment_action_label(experiment_name: str) -> str:
        if experiment_name == "baseline":
            return "baseline strict-local run"
        mapping = {
            "legacy_freeze_gravity": "legacy freeze-style gravity",
            "gravity_fixed_norm": "fixed-norm gravity",
            "gravity_limited_tilt": "limited-tilt gravity",
            "gravity_warmup_freeze_then_release": "warmup-freeze gravity",
            "freeze_gyro_bias": "freezing gyro bias",
            "freeze_accel_bias": "freezing accel bias",
            "disable_velocity_factor": "disabling velocity factor",
            "disable_current_velocity_prior": "disabling current velocity prior",
        }
        parts = [mapping.get(part, part.replace("_", " ")) for part in experiment_name.split("+") if part]
        if not parts:
            return "baseline strict-local run"
        return parts[0] if len(parts) == 1 else " + ".join(parts)

    experiment_name = str(analysis.get("runtime_experiment_name") or "baseline")
    yaw_p95 = float(analysis.get("frontend_to_final_yaw_abs_p95", 0.0) or 0.0)
    yaw_max = float(analysis.get("frontend_to_final_yaw_abs_max", 0.0) or 0.0)
    rotation_p95 = float(analysis.get("frontend_to_final_rotation_p95", 0.0) or 0.0)
    translation_p95 = float(analysis.get("frontend_to_final_translation_p95", 0.0) or 0.0)
    jump_rows = int(analysis.get("jump_rows", 0) or 0)
    accept_ratio_mean = analysis.get("accept_ratio_mean")
    match_ratio_mean = analysis.get("match_ratio_mean")
    inlier_ratio_mean = analysis.get("inlier_ratio_mean")
    solver_update_ms_mean = analysis.get("solver_update_ms_mean")
    run_incomplete = jump_rows < 200
    registration_degraded = (
        (isinstance(accept_ratio_mean, float) and math.isfinite(accept_ratio_mean) and accept_ratio_mean < 0.85)
        or (isinstance(match_ratio_mean, float) and math.isfinite(match_ratio_mean) and match_ratio_mean < 0.35)
        or (isinstance(inlier_ratio_mean, float) and math.isfinite(inlier_ratio_mean) and inlier_ratio_mean < 0.35)
        or translation_p95 > 2.0
        or (isinstance(solver_update_ms_mean, float) and math.isfinite(solver_update_ms_mean) and solver_update_ms_mean > 250.0)
        or run_incomplete
    )
    yaw_low = yaw_p95 <= 0.25 and rotation_p95 <= 0.45
    yaw_partial = yaw_p95 <= 0.45 and rotation_p95 <= 0.75
    if experiment_name == "baseline":
        analysis["isolation_effect"] = "baseline strict-local run for yaw isolation comparison"
    else:
        action = experiment_action_label(experiment_name)
        if experiment_name == "legacy_freeze_gravity" and registration_degraded:
            analysis["isolation_effect"] = "freeze-style gravity remains unstable and should not be used as the fix"
        elif registration_degraded:
            analysis["isolation_effect"] = (
                f"{action} changes residual behavior but introduces instability or incompleteness"
            )
        elif yaw_low:
            analysis["isolation_effect"] = (
                f"{action} significantly reduces yaw residual and keeps run stability acceptable"
            )
        elif yaw_partial:
            analysis["isolation_effect"] = (
                f"{action} mildly reduces yaw residual without harming completeness"
            )
        else:
            analysis["isolation_effect"] = f"{action} has little effect"
    analysis["isolation_effect_evidence"] = "; ".join(
        [
            f"experiment_name={experiment_name}",
            f"yaw_p95={yaw_p95:.3f}",
            f"yaw_max={yaw_max:.3f}",
            f"rotation_p95={rotation_p95:.3f}",
            f"translation_p95={translation_p95:.3f}",
            (
                f"solver_update_ms_mean={solver_update_ms_mean:.3f}"
                if isinstance(solver_update_ms_mean, float) and math.isfinite(solver_update_ms_mean)
                else "solver_update_ms_mean=n/a"
            ),
            (
                f"accept_ratio_mean={accept_ratio_mean:.3f}"
                if isinstance(accept_ratio_mean, float) and math.isfinite(accept_ratio_mean)
                else "accept_ratio_mean=n/a"
            ),
            (
                f"match_ratio_mean={match_ratio_mean:.3f}"
                if isinstance(match_ratio_mean, float) and math.isfinite(match_ratio_mean)
                else "match_ratio_mean=n/a"
            ),
            (
                f"inlier_ratio_mean={inlier_ratio_mean:.3f}"
                if isinstance(inlier_ratio_mean, float) and math.isfinite(inlier_ratio_mean)
                else "inlier_ratio_mean=n/a"
            ),
        ]
    )

    translation_score = max(
        float(analysis.get("frontend_to_final_translation_p95", 0.0) or 0.0) / 1.0,
        float(analysis.get("frontend_to_final_translation_max", 0.0) or 0.0) / 1.0,
    )
    rotation_score = max(
        float(analysis.get("frontend_to_final_rotation_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_rotation_max", 0.0) or 0.0) / 0.3,
    )
    analysis["strict_local_translation_score"] = translation_score
    analysis["strict_local_rotation_score"] = rotation_score
    if rotation_score > 1.25 * max(translation_score, 1e-9):
        analysis["strict_local_residual_dominance"] = "strict-local residual appears rotation-dominated"
    elif translation_score > 1.25 * max(rotation_score, 1e-9):
        analysis["strict_local_residual_dominance"] = "strict-local residual appears translation-dominated"
    else:
        analysis["strict_local_residual_dominance"] = "strict-local residual appears mixed"

    yaw_score = max(
        float(analysis.get("frontend_to_final_yaw_abs_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_yaw_abs_max", 0.0) or 0.0) / 0.3,
    )
    pitchroll_score = max(
        float(analysis.get("frontend_to_final_pitch_abs_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_roll_abs_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_pitch_abs_max", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_roll_abs_max", 0.0) or 0.0) / 0.3,
    )
    analysis["strict_local_yaw_score"] = yaw_score
    analysis["strict_local_pitchroll_score"] = pitchroll_score
    if analysis["strict_local_residual_dominance"] != "strict-local residual appears rotation-dominated":
        analysis["strict_local_rotation_subtype"] = "strict-local residual orientation split is secondary"
    elif yaw_score > 1.25 * max(pitchroll_score, 1e-9):
        analysis["strict_local_rotation_subtype"] = "strict-local residual appears yaw-dominated"
    elif pitchroll_score > 1.25 * max(yaw_score, 1e-9):
        analysis["strict_local_rotation_subtype"] = "strict-local residual appears pitch/roll-dominated"
    else:
        analysis["strict_local_rotation_subtype"] = "strict-local residual orientation split is mixed"

    pitch_score = max(
        float(analysis.get("frontend_to_final_pitch_abs_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_pitch_abs_max", 0.0) or 0.0) / 0.3,
    )
    roll_score = max(
        float(analysis.get("frontend_to_final_roll_abs_p95", 0.0) or 0.0) / 0.3,
        float(analysis.get("frontend_to_final_roll_abs_max", 0.0) or 0.0) / 0.3,
    )
    analysis["strict_local_pitch_score"] = pitch_score
    analysis["strict_local_roll_score"] = roll_score
    if pitch_score > 1.25 * max(roll_score, 1e-9):
        analysis["pitch_vs_roll_dominance"] = "pitch_dominated"
    elif roll_score > 1.25 * max(pitch_score, 1e-9):
        analysis["pitch_vs_roll_dominance"] = "roll_dominated"
    else:
        analysis["pitch_vs_roll_dominance"] = "mixed_pitch_roll"

    def pitchroll_stage_score(pitch_prefix: str, roll_prefix: str) -> float:
        components = [
            float(analysis.get(f"{pitch_prefix}_abs_p95", 0.0) or 0.0) / 0.3,
            float(analysis.get(f"{pitch_prefix}_abs_max", 0.0) or 0.0) / 0.3,
            float(analysis.get(f"{roll_prefix}_abs_p95", 0.0) or 0.0) / 0.3,
            float(analysis.get(f"{roll_prefix}_abs_max", 0.0) or 0.0) / 0.3,
        ]
        finite_components = [value for value in components if math.isfinite(value)]
        return max(finite_components) if finite_components else 0.0

    stage_pitchroll_scores = {
        "start->frontend": pitchroll_stage_score("start_to_frontend_pitch", "start_to_frontend_roll"),
        "frontend->postsolve_strict_local": pitchroll_stage_score(
            "frontend_to_postsolve_strict_local_pitch",
            "frontend_to_postsolve_strict_local_roll",
        ),
        "frontend->postsolve_active_window": pitchroll_stage_score(
            "frontend_to_postsolve_query_pitch",
            "frontend_to_postsolve_query_roll",
        ),
        "postsolve_active_window->strict_local": pitchroll_stage_score(
            "postsolve_active_window_to_postsolve_strict_local_pitch",
            "postsolve_active_window_to_postsolve_strict_local_roll",
        ),
        "postsolve_strict_local->final": pitchroll_stage_score(
            "postsolve_strict_local_to_final_pitch",
            "postsolve_strict_local_to_final_roll",
        ),
    }
    analysis["pitch_roll_stage_scores"] = stage_pitchroll_scores

    strict_local_correlation_specs = {
        "delta_rotation_vs_solver_update_ms": (
            "delta_frontend_to_final_rotation_rad",
            "solver_update_ms",
            False,
        ),
        "delta_rotation_vs_recalc_imu": (
            "delta_frontend_to_final_rotation_rad",
            "recalculated_imu_factor_count",
            False,
        ),
        "delta_rotation_vs_recalc_prior": (
            "delta_frontend_to_final_rotation_rad",
            "recalculated_prior_factor_count",
            False,
        ),
        "delta_rotation_vs_recalc_velocity": (
            "delta_frontend_to_final_rotation_rad",
            "recalculated_velocity_factor_count",
            False,
        ),
        "delta_rotation_vs_relin_shared": (
            "delta_frontend_to_final_rotation_rad",
            "relinearized_shared_variable_count",
            False,
        ),
        "delta_translation_vs_solver_update_ms": (
            "delta_frontend_to_final_translation_norm",
            "solver_update_ms",
            False,
        ),
        "delta_translation_vs_recalc_lidar": (
            "delta_frontend_to_final_translation_norm",
            "recalculated_lidar_factor_count",
            False,
        ),
        "delta_z_vs_recalc_imu": (
            "delta_frontend_to_final_dz",
            "recalculated_imu_factor_count",
            True,
        ),
        "delta_yaw_vs_recalc_imu": (
            "delta_frontend_to_final_yaw_rad",
            "recalculated_imu_factor_count",
            True,
        ),
        "delta_yaw_vs_recalc_prior": (
            "delta_frontend_to_final_yaw_rad",
            "recalculated_prior_factor_count",
            True,
        ),
        "delta_yaw_vs_relin_shared": (
            "delta_frontend_to_final_yaw_rad",
            "relinearized_shared_variable_count",
            True,
        ),
        "delta_yaw_vs_recalc_velocity": (
            "delta_frontend_to_final_yaw_rad",
            "recalculated_velocity_factor_count",
            True,
        ),
        "delta_yaw_vs_current_velocity_norm": (
            "delta_frontend_to_final_yaw_rad",
            "current_velocity_norm",
            True,
        ),
        "delta_yaw_vs_delta_velocity_heading": (
            "delta_frontend_to_final_yaw_rad",
            "delta_velocity_heading_rad",
            True,
        ),
        "delta_yaw_vs_world_to_lidar_yaw_shift": (
            "delta_frontend_to_final_yaw_rad",
            "world_to_lidar_yaw_shift",
            True,
        ),
        "delta_yaw_vs_world_to_imu_yaw_shift": (
            "delta_frontend_to_final_yaw_rad",
            "world_to_imu_yaw_shift",
            True,
        ),
        "delta_yaw_vs_extrinsic_yaw_probe": (
            "delta_frontend_to_final_yaw_rad",
            "lidar_to_imu_extrinsic_yaw",
            True,
        ),
        "delta_yaw_vs_gyro_bias_norm": (
            "delta_frontend_to_final_yaw_rad",
            "gyro_bias_norm",
            True,
        ),
        "delta_yaw_vs_accel_bias_norm": (
            "delta_frontend_to_final_yaw_rad",
            "accel_bias_norm",
            True,
        ),
        "delta_yaw_vs_gravity_dir_tilt": (
            "delta_frontend_to_final_yaw_rad",
            "gravity_dir_tilt_rad",
            True,
        ),
        "delta_yaw_vs_delta_gravity_dir": (
            "delta_frontend_to_final_yaw_rad",
            "delta_gravity_dir_rad",
            True,
        ),
    }
    for out_key, (lhs, rhs, lhs_abs) in strict_local_correlation_specs.items():
        corr_value = safe_corr(df, lhs, rhs, lhs_abs=lhs_abs)
        if corr_value is not None:
            analysis[out_key] = corr_value

    if {
        "delta_frontend_to_final_pitch_rad",
        "delta_frontend_to_final_roll_rad",
    } <= set(df.columns):
        pitch_series = pd.to_numeric(df["delta_frontend_to_final_pitch_rad"], errors="coerce").abs()
        roll_series = pd.to_numeric(df["delta_frontend_to_final_roll_rad"], errors="coerce").abs()
        df["delta_frontend_to_final_pitchroll_abs"] = pd.concat(
            [pitch_series, roll_series], axis=1
        ).max(axis=1)
        pitchroll_corr_specs = {
            "delta_pitchroll_vs_recalc_imu": "recalculated_imu_factor_count",
            "delta_pitchroll_vs_recalc_prior": "recalculated_prior_factor_count",
            "delta_pitchroll_vs_relin_shared": "relinearized_shared_variable_count",
            "delta_pitchroll_vs_current_velocity_norm": "current_velocity_norm",
            "delta_pitchroll_vs_match_ratio": "match_ratio",
            "delta_pitchroll_vs_inlier_ratio": "inlier_ratio",
            "delta_pitchroll_vs_target_point_count": "target_point_count",
            "delta_pitchroll_vs_target_voxel_count": "target_voxel_count",
            "delta_pitchroll_vs_cross_support_lidar": "recalculated_lidar_cross_support_factor_count",
            "delta_pitchroll_vs_solver_update_ms": "solver_update_ms",
        }
        for out_key, rhs in pitchroll_corr_specs.items():
            corr_value = safe_corr(df, "delta_frontend_to_final_pitchroll_abs", rhs, lhs_abs=False)
            if corr_value is not None:
                analysis[out_key] = corr_value

    def dominant_corr(items: dict[str, str]) -> tuple[str | None, float | None]:
        best_name: str | None = None
        best_value: float | None = None
        for label, key in items.items():
            value = analysis.get(key)
            if not isinstance(value, float) or not math.isfinite(value):
                continue
            if best_value is None or abs(value) > abs(best_value):
                best_name = label
                best_value = value
        return best_name, best_value

    rotation_corr_name, rotation_corr_value = dominant_corr(
        {
            "solver_update_ms": "delta_rotation_vs_solver_update_ms",
            "recalc_imu": "delta_rotation_vs_recalc_imu",
            "recalc_prior": "delta_rotation_vs_recalc_prior",
            "recalc_velocity": "delta_rotation_vs_recalc_velocity",
            "relin_shared": "delta_rotation_vs_relin_shared",
            "delta_yaw_vs_recalc_imu": "delta_yaw_vs_recalc_imu",
            "delta_yaw_vs_recalc_prior": "delta_yaw_vs_recalc_prior",
            "delta_yaw_vs_relin_shared": "delta_yaw_vs_relin_shared",
            "delta_yaw_vs_recalc_velocity": "delta_yaw_vs_recalc_velocity",
            "delta_yaw_vs_current_velocity_norm": "delta_yaw_vs_current_velocity_norm",
            "delta_yaw_vs_delta_velocity_heading": "delta_yaw_vs_delta_velocity_heading",
        }
    )
    translation_corr_name, translation_corr_value = dominant_corr(
        {
            "solver_update_ms": "delta_translation_vs_solver_update_ms",
            "recalc_lidar": "delta_translation_vs_recalc_lidar",
            "delta_z_vs_recalc_imu": "delta_z_vs_recalc_imu",
        }
    )
    if rotation_corr_name is not None and rotation_corr_value is not None:
        analysis["strict_local_rotation_primary_correlation"] = (
            f"{rotation_corr_name}:{rotation_corr_value:.3f}"
        )
    if translation_corr_name is not None and translation_corr_value is not None:
        analysis["strict_local_translation_primary_correlation"] = (
            f"{translation_corr_name}:{translation_corr_value:.3f}"
        )

    strict_local_reason_counts = analysis.get("top_strict_local_query_reason_counts", {})
    strict_local_reason_total = sum(strict_local_reason_counts.values())
    strict_local_query_non_none_share = (
        (strict_local_reason_total - strict_local_reason_counts.get("none", 0)) / strict_local_reason_total
        if strict_local_reason_total > 0
        else 0.0
    )
    strict_local_query_side_suspicious = strict_local_query_non_none_share >= 0.4

    dominance_label = analysis.get("strict_local_residual_dominance", "")
    if strict_local_query_side_suspicious:
        analysis["strict_local_residual_cause"] = "strict-local residual likely includes query-side orientation inconsistency"
    elif (
        dominance_label == "strict-local residual appears rotation-dominated"
        and rotation_corr_name in {
            "recalc_imu",
            "recalc_prior",
            "recalc_velocity",
            "relin_shared",
            "delta_yaw_vs_recalc_imu",
            "delta_yaw_vs_recalc_prior",
            "delta_yaw_vs_relin_shared",
            "delta_yaw_vs_recalc_velocity",
            "delta_yaw_vs_current_velocity_norm",
            "delta_yaw_vs_delta_velocity_heading",
        }
        and rotation_corr_value is not None
        and abs(rotation_corr_value) >= 0.3
    ):
        analysis["strict_local_residual_cause"] = "strict-local residual more likely reflects solver-side orientation drift"
    elif (
        dominance_label == "strict-local residual appears translation-dominated"
        and translation_corr_name == "recalc_lidar"
        and translation_corr_value is not None
        and abs(translation_corr_value) >= 0.3
    ):
        analysis["strict_local_residual_cause"] = "strict-local residual more likely reflects translation-side factor consistency"
    elif (
        dominance_label == "strict-local residual appears mixed"
        and rotation_corr_value is not None
        and translation_corr_value is not None
        and abs(rotation_corr_value) >= 0.2
        and abs(translation_corr_value) >= 0.2
    ):
        analysis["strict_local_residual_cause"] = "strict-local residual appears mixed; IMU/prior and LiDAR both suspicious"
    elif dominance_label == "strict-local residual appears rotation-dominated":
        analysis["strict_local_residual_cause"] = "strict-local residual may still reflect orientation-side semantics or frame-convention mismatch"
    else:
        analysis["strict_local_residual_cause"] = "strict-local residual cause not strongly isolated"

    residual_evidence: list[str] = []
    if strict_local_reason_counts:
        residual_evidence.append(
            "top strict-local reasons="
            + ", ".join(f"{key}:{value}" for key, value in strict_local_reason_counts.items())
        )
    if rotation_corr_name is not None and rotation_corr_value is not None:
        residual_evidence.append(f"rotation corr={rotation_corr_name}:{rotation_corr_value:.3f}")
    if translation_corr_name is not None and translation_corr_value is not None:
        residual_evidence.append(f"translation corr={translation_corr_name}:{translation_corr_value:.3f}")
    residual_evidence.append(
        f"frontend->final t_p95={analysis.get('frontend_to_final_translation_p95', 0.0):.3f} m"
    )
    residual_evidence.append(
        f"frontend->final r_p95={analysis.get('frontend_to_final_rotation_p95', 0.0):.3f} rad"
    )
    residual_evidence.append(
        f"yaw_abs_p95={analysis.get('frontend_to_final_yaw_abs_p95', 0.0):.3f} rad"
    )
    residual_evidence.append(
        f"abs_dz_p95={analysis.get('frontend_to_final_abs_dz_abs_p95', 0.0):.3f} m"
    )
    analysis["strict_local_residual_cause_evidence"] = "; ".join(residual_evidence)

    def available_weighted_score(weighted_values: list[tuple[float, float | None]]) -> float:
        available = [(weight, value) for weight, value in weighted_values if value is not None and math.isfinite(value)]
        if not available:
            return 0.0
        total_weight = sum(weight for weight, _ in available)
        if total_weight <= 0.0:
            return 0.0
        return 100.0 * sum(weight * value for weight, value in available) / total_weight

    def norm_corr(value: Any) -> float | None:
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return None
        return min(abs(float(value)) / 0.60, 1.0)

    def norm_gap(value: Any, threshold: float) -> float | None:
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return None
        return min(abs(float(value)) / threshold, 1.0)

    yaw_space_gap_p95 = analysis.get("yaw_space_gap_p95")
    top_yaw_chain_inconsistency_ratio = analysis.get("top_yaw_chain_inconsistency_ratio")

    candidate_a_score = available_weighted_score([
        (0.30, norm_corr(analysis.get("delta_yaw_vs_recalc_velocity"))),
        (0.25, norm_corr(analysis.get("delta_yaw_vs_recalc_prior"))),
        (0.20, norm_corr(analysis.get("delta_yaw_vs_relin_shared"))),
        (0.15, norm_corr(analysis.get("delta_yaw_vs_current_velocity_norm"))),
        (0.10, norm_corr(analysis.get("delta_yaw_vs_delta_velocity_heading"))),
    ])
    candidate_b_score = available_weighted_score([
        (0.40, float(top_yaw_chain_inconsistency_ratio) if isinstance(top_yaw_chain_inconsistency_ratio, (int, float)) and math.isfinite(float(top_yaw_chain_inconsistency_ratio)) else None),
        (0.35, norm_gap(yaw_space_gap_p95, 0.20)),
        (0.15, norm_corr(analysis.get("delta_yaw_vs_world_to_lidar_yaw_shift"))),
        (0.10, norm_corr(analysis.get("delta_yaw_vs_world_to_imu_yaw_shift"))),
    ])
    candidate_c_score = available_weighted_score([
        (0.25, norm_corr(analysis.get("delta_yaw_vs_recalc_imu"))),
        (0.20, norm_corr(analysis.get("delta_yaw_vs_gyro_bias_norm"))),
        (0.15, norm_corr(analysis.get("delta_yaw_vs_accel_bias_norm"))),
        (0.20, norm_corr(analysis.get("delta_yaw_vs_gravity_dir_tilt"))),
        (0.20, norm_corr(analysis.get("delta_yaw_vs_delta_gravity_dir"))),
    ])

    analysis["candidate_A_velocity_prior_shared_score"] = candidate_a_score
    analysis["candidate_B_orientation_semantics_score"] = candidate_b_score
    analysis["candidate_C_imu_bias_gravity_score"] = candidate_c_score

    candidate_labels = {
        "A": "velocity / prior / shared-state yaw coupling",
        "B": "orientation semantics / frame-convention / extrinsic yaw usage",
        "C": "IMU / bias / gravity",
    }
    candidate_next_fix_targets = {
        "A": "velocity/prior/shared-state yaw handling",
        "B": "orientation semantics / frame-convention / extrinsic yaw usage",
        "C": "IMU/bias/gravity handling",
    }
    candidate_evidence = {
        "A": "; ".join(
            part
            for part in [
                f"delta_yaw_vs_recalc_velocity={analysis.get('delta_yaw_vs_recalc_velocity', 'n/a')}",
                f"delta_yaw_vs_recalc_prior={analysis.get('delta_yaw_vs_recalc_prior', 'n/a')}",
                f"delta_yaw_vs_relin_shared={analysis.get('delta_yaw_vs_relin_shared', 'n/a')}",
                f"delta_yaw_vs_current_velocity_norm={analysis.get('delta_yaw_vs_current_velocity_norm', 'n/a')}",
                f"delta_yaw_vs_delta_velocity_heading={analysis.get('delta_yaw_vs_delta_velocity_heading', 'n/a')}",
            ]
            if part
        ),
        "B": "; ".join(
            part
            for part in [
                f"top_yaw_chain_inconsistency_ratio={analysis.get('top_yaw_chain_inconsistency_ratio', 0.0):.3f}" if isinstance(analysis.get("top_yaw_chain_inconsistency_ratio"), (int, float)) else "top_yaw_chain_inconsistency_ratio=n/a",
                f"yaw_space_gap_p95={analysis.get('yaw_space_gap_p95', 'n/a')}",
                f"delta_yaw_vs_world_to_lidar_yaw_shift={analysis.get('delta_yaw_vs_world_to_lidar_yaw_shift', 'n/a')}",
                f"delta_yaw_vs_world_to_imu_yaw_shift={analysis.get('delta_yaw_vs_world_to_imu_yaw_shift', 'n/a')}",
                f"delta_yaw_vs_extrinsic_yaw_probe={analysis.get('delta_yaw_vs_extrinsic_yaw_probe', 'n/a')}",
            ]
            if part
        ),
        "C": "; ".join(
            part
            for part in [
                f"delta_yaw_vs_recalc_imu={analysis.get('delta_yaw_vs_recalc_imu', 'n/a')}",
                f"delta_yaw_vs_gyro_bias_norm={analysis.get('delta_yaw_vs_gyro_bias_norm', 'n/a')}",
                f"delta_yaw_vs_accel_bias_norm={analysis.get('delta_yaw_vs_accel_bias_norm', 'n/a')}",
                f"delta_yaw_vs_gravity_dir_tilt={analysis.get('delta_yaw_vs_gravity_dir_tilt', 'n/a')}",
                f"delta_yaw_vs_delta_gravity_dir={analysis.get('delta_yaw_vs_delta_gravity_dir', 'n/a')}",
            ]
            if part
        ),
    }

    ranked_candidates = sorted(
        [
            ("A", candidate_a_score),
            ("B", candidate_b_score),
            ("C", candidate_c_score),
        ],
        key=lambda item: item[1],
        reverse=True,
    )
    leader_key, leader_score = ranked_candidates[0]
    second_key, second_score = ranked_candidates[1]
    weakest_key, weakest_score = ranked_candidates[2]
    analysis["yaw_root_cause_leader"] = leader_key
    analysis["yaw_root_cause_second"] = second_key
    analysis["yaw_root_cause_weakest"] = weakest_key
    analysis["yaw_root_cause_leader_label"] = candidate_labels[leader_key]
    analysis["yaw_root_cause_second_label"] = candidate_labels[second_key]
    analysis["yaw_root_cause_weakest_label"] = candidate_labels[weakest_key]
    analysis["yaw_root_cause_next_fix_target"] = candidate_next_fix_targets[leader_key]
    analysis["yaw_root_cause_leader_evidence"] = candidate_evidence[leader_key]
    analysis["yaw_root_cause_second_evidence"] = candidate_evidence[second_key]
    analysis["yaw_root_cause_weakest_evidence"] = candidate_evidence[weakest_key]

    if leader_score - second_score >= 15.0:
        if leader_key == "A":
            analysis["yaw_root_cause_summary"] = "yaw residual appears velocity/prior/shared-state dominated"
        elif leader_key == "B":
            analysis["yaw_root_cause_summary"] = "yaw residual appears orientation semantics / frame-convention dominated"
        else:
            analysis["yaw_root_cause_summary"] = "yaw residual appears IMU/bias/gravity dominated"
    else:
        analysis["yaw_root_cause_summary"] = (
            f"yaw residual appears mixed; {leader_key} strongest, {second_key} second, {weakest_key} weakest"
        )
    analysis["yaw_root_cause_evidence"] = (
        f"A={candidate_a_score:.1f} ({candidate_evidence['A']}); "
        f"B={candidate_b_score:.1f} ({candidate_evidence['B']}); "
        f"C={candidate_c_score:.1f} ({candidate_evidence['C']})"
    )

    if dominance_label == "strict-local residual appears rotation-dominated":
        if strict_local_query_side_suspicious:
            analysis["strict_local_residual_cause"] = (
                "strict-local residual likely includes query-side orientation inconsistency"
            )
        elif leader_key == "B":
            analysis["strict_local_residual_cause"] = (
                "strict-local residual may still reflect orientation-side semantics or frame-convention mismatch"
            )
        else:
            analysis["strict_local_residual_cause"] = (
                "strict-local residual more likely reflects solver-side orientation drift"
            )
        analysis["strict_local_residual_cause_evidence"] = (
            analysis.get("strict_local_residual_cause_evidence", "")
            + ("; " if analysis.get("strict_local_residual_cause_evidence") else "")
            + f"yaw shootout={analysis['yaw_root_cause_summary']}"
        )

    dominant_pitchroll_stage = max(
        stage_pitchroll_scores.items(),
        key=lambda item: item[1],
        default=("unknown", 0.0),
    )
    analysis["pitch_roll_dominant_stage"] = dominant_pitchroll_stage[0]
    analysis["pitch_roll_dominant_stage_score"] = dominant_pitchroll_stage[1]

    pitchroll_solver_corr = max(
        abs(float(analysis.get("delta_pitchroll_vs_solver_update_ms", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_recalc_imu", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_recalc_prior", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_relin_shared", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_cross_support_lidar", 0.0) or 0.0)),
    )
    pitchroll_quality_corr = max(
        abs(float(analysis.get("delta_pitchroll_vs_match_ratio", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_inlier_ratio", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_target_point_count", 0.0) or 0.0)),
        abs(float(analysis.get("delta_pitchroll_vs_target_voxel_count", 0.0) or 0.0)),
    )
    if (
        dominant_pitchroll_stage[0] == "start->frontend"
        and dominant_pitchroll_stage[1] > 1.0
    ):
        analysis["pitch_roll_root_cause_summary"] = (
            "pitch/roll residual more likely reflects frontend seed / IMU motion-prior mismatch"
        )
    elif (
        dominant_pitchroll_stage[0] in {
            "frontend->postsolve_strict_local",
            "postsolve_active_window->strict_local",
        }
        and pitchroll_solver_corr >= 0.25
    ):
        analysis["pitch_roll_root_cause_summary"] = (
            "pitch/roll residual more likely reflects backend/boundary orientation amplification"
        )
    elif pitchroll_quality_corr >= 0.25 and pitchroll_solver_corr < 0.25:
        analysis["pitch_roll_root_cause_summary"] = (
            "pitch/roll residual more likely couples to LiDAR target/correspondence quality"
        )
    else:
        analysis["pitch_roll_root_cause_summary"] = (
            "pitch/roll residual may still reflect orientation semantics / extrinsic roll-pitch mismatch"
        )
    analysis["pitch_roll_root_cause_evidence"] = "; ".join(
        [
            f"pitch_vs_roll_dominance={analysis.get('pitch_vs_roll_dominance', 'n/a')}",
            f"dominant_stage={dominant_pitchroll_stage[0]}:{dominant_pitchroll_stage[1]:.3f}",
            f"seed_mode={analysis.get('runtime_frontend_seed_mode', 'n/a')}",
            f"seed_source={analysis.get('runtime_frontend_seed_source', 'n/a')}",
            f"seed_fallback_frames={analysis.get('seed_fallback_frame_count', 0)}",
            f"solver_corr={pitchroll_solver_corr:.3f}",
            f"quality_corr={pitchroll_quality_corr:.3f}",
        ]
    )

    return analysis


def analyze_mode_consistency(
    metadata: dict[str, Any],
    config_summary: dict[str, Any],
    runtime_summary: dict[str, Any],
    artifact_statuses: list[ArtifactStatus],
    frame_df: pd.DataFrame | None,
) -> dict[str, Any]:
    frontend_cfg = maybe_bool(config_summary.get("frontend_only_mode"))
    frontend_runtime = maybe_bool(config_summary.get("runtime_frontend_only_mode"))
    frontend_effective = frontend_only_effective(config_summary)
    mapping_expected = mapping_expected_active(config_summary)
    loaded_mapping_modules = mapping_modules_loaded(runtime_summary)
    mode_manifest = metadata.get("mode_manifest", {})
    gnss_expected = None
    gnss_observed = None

    backend_observed_active = False
    mapping_observed_active = bool(loaded_mapping_modules)
    if frame_df is not None and not frame_df.empty:
        for col in ["backend_update_ms", "backend_optimize_ms", "backend_factor_count", "backend_state_count"]:
            if col in frame_df.columns and float(frame_df[col].fillna(0).max()) > 0.0:
                backend_observed_active = True
        for col in ["local_mapping_update_ms", "global_mapping_update_ms", "submap_registration_ms"]:
            if col in frame_df.columns and float(frame_df[col].fillna(0).max()) > 0.0:
                mapping_observed_active = True

    backend_expected = None if frontend_effective is None else (not frontend_effective)
    if isinstance(mode_manifest, dict) and mode_manifest:
        frontend_cfg = maybe_bool(mode_manifest.get("frontend_only_expected")) if frontend_cfg is None else frontend_cfg
        frontend_runtime = maybe_bool(mode_manifest.get("frontend_only_observed")) if frontend_runtime is None else frontend_runtime
        frontend_effective = maybe_bool(mode_manifest.get("frontend_only_observed")) if maybe_bool(mode_manifest.get("frontend_only_observed")) is not None else frontend_effective
        backend_expected = maybe_bool(mode_manifest.get("backend_expected_active")) if maybe_bool(mode_manifest.get("backend_expected_active")) is not None else backend_expected
        backend_observed_active = maybe_bool(mode_manifest.get("backend_observed_active")) if maybe_bool(mode_manifest.get("backend_observed_active")) is not None else backend_observed_active
        mapping_expected = maybe_bool(mode_manifest.get("mapping_expected_active")) if maybe_bool(mode_manifest.get("mapping_expected_active")) is not None else mapping_expected
        mapping_observed_active = maybe_bool(mode_manifest.get("mapping_observed_active")) if maybe_bool(mode_manifest.get("mapping_observed_active")) is not None else mapping_observed_active
        gnss_expected = maybe_bool(mode_manifest.get("gnss_expected_active"))
        gnss_observed = maybe_bool(mode_manifest.get("gnss_observed_active"))

    unexpected_runtime_modules: list[str] = []
    loaded_contradictions: list[str] = []
    if frontend_effective:
        loaded_contradictions.extend(loaded_mapping_modules)
        unexpected_runtime_modules.extend(loaded_mapping_modules)

    artifact_groups = {
        "expected_empty_artifacts": [],
        "unexpected_empty_artifacts": [],
        "should_not_exist_artifacts_but_found": [],
    }
    for artifact in artifact_statuses:
        if artifact.reason == "expected_empty_by_mode":
            artifact_groups["expected_empty_artifacts"].append(artifact.name)
        if artifact.status == "empty" and artifact.reason not in {"expected_empty_by_mode", "config_disabled"}:
            artifact_groups["unexpected_empty_artifacts"].append(artifact.name)
        if frontend_effective and artifact.name == "backend_summary" and artifact.status == "found":
            artifact_groups["should_not_exist_artifacts_but_found"].append(artifact.name)

    warnings_inconsistent_with_mode: list[str] = []
    if frontend_effective:
        for issue in runtime_summary.get("issues", []):
            msg = issue["message"].lower()
            if "local mapping" in msg or "global mapping" in msg or "submap" in msg:
                warnings_inconsistent_with_mode.append(issue["message"])

    return {
        "frontend_only_mode_config": frontend_cfg,
        "frontend_only_mode_runtime": frontend_runtime,
        "frontend_only_mode_effective": frontend_effective,
        "backend_expected_active": backend_expected,
        "backend_observed_active": backend_observed_active,
        "mapping_expected_active": mapping_expected,
        "mapping_observed_active": mapping_observed_active,
        "gnss_expected_active": gnss_expected,
        "gnss_observed_active": gnss_observed,
        "loaded_modules_that_contradict_mode": loaded_contradictions,
        "unexpected_runtime_modules": unexpected_runtime_modules,
        "expected_empty_artifacts": artifact_groups["expected_empty_artifacts"],
        "unexpected_empty_artifacts": artifact_groups["unexpected_empty_artifacts"],
        "should_not_exist_artifacts_but_found": artifact_groups["should_not_exist_artifacts_but_found"],
        "warnings_inconsistent_with_mode": warnings_inconsistent_with_mode[:20],
    }


def analyze_slowest_frames(
    frame_df: pd.DataFrame | None,
    bucket_df: pd.DataFrame | None,
    warning_df: pd.DataFrame | None,
) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if frame_df is None or frame_df.empty:
        return analysis

    df = frame_df.copy()
    stage_cols = stage_names_from_df(df)
    if not stage_cols:
        return analysis
    df["frame_total_ms"] = df[stage_cols].sum(axis=1)

    if bucket_df is not None and not bucket_df.empty and "frame_id" in bucket_df.columns:
        grouped = bucket_df.groupby("frame_id").agg(
            points_in_bucket=("points_in_bucket", "sum"),
            factor_total_ms=("factor_total_ms", "sum"),
            match_ratio=("match_ratio", "mean"),
            inlier_ratio=("inlier_ratio", "mean"),
        )
        df = df.merge(grouped, on="frame_id", how="left")

    if warning_df is not None and not warning_df.empty and "frame_id" in warning_df.columns and "warning_count" in warning_df.columns:
        warning_counts = warning_df[["frame_id", "warning_count"]].copy()
        df = df.merge(warning_counts, on="frame_id", how="left")
        df["warning_count_for_frame"] = df["warning_count"]
        df = df.drop(columns=["warning_count"])
    elif "warning_count_for_frame" not in df.columns:
        df["warning_count_for_frame"] = np.nan
    top_stage = top_stage_per_row(df, stage_cols)
    df["top_stage"] = top_stage
    top10 = df.sort_values("frame_total_ms", ascending=False).head(10)
    analysis["available"] = True
    analysis["slowest_frames"] = top10[
        [c for c in [
            "frame_id",
            "frame_total_ms",
            "lm_solve_ms",
            "target_map_prep_ms",
            "preprocess_ms",
            "bucket_build_ms",
            "lidar_factor_build_ms",
            "imu_factor_build_ms",
            "points_in_bucket",
            "factor_total_ms",
            "match_ratio",
            "inlier_ratio",
            "warning_count_for_frame",
            "top_stage",
        ] if c in top10.columns]
    ].to_dict(orient="records")
    analysis["lm_dominant_count"] = int((top10["top_stage"] == "lm_solve_ms").sum()) if "top_stage" in top10 else 0
    analysis["target_map_prep_dominant_count"] = int((top10["top_stage"] == "target_map_prep_ms").sum()) if "top_stage" in top10 else 0
    analysis["warning_count_available"] = "warning_count_for_frame" in df.columns and not df["warning_count_for_frame"].dropna().empty
    return analysis


def analyze_optimizer_behavior(
    frame_df: pd.DataFrame | None,
    lm_df: pd.DataFrame | None,
    artifact_statuses: list[ArtifactStatus],
) -> dict[str, Any]:
    analysis: dict[str, Any] = {
        "optimization_diagnostics_available": False,
        "lm_iteration_rows_present": False,
    }
    status_map = {artifact.name: artifact for artifact in artifact_statuses}
    lm_artifact = status_map.get("frontend_lm_iteration")

    if frame_df is not None and not frame_df.empty:
        analysis["frames_with_nonzero_iterations"] = int((frame_df.get("lm_iteration_count", pd.Series(dtype=float)).fillna(0) > 0).sum()) if "lm_iteration_count" in frame_df else 0
        if "lm_iteration_count" in frame_df:
            analysis["lm_iteration_mean"] = float(frame_df["lm_iteration_count"].fillna(0).mean())
            analysis["lm_iteration_p95"] = pct(frame_df["lm_iteration_count"].fillna(0), 0.95)
            analysis["lm_iteration_max"] = int(frame_df["lm_iteration_count"].fillna(0).max())
        if "lm_trace_expected" in frame_df:
            analysis["frames_with_trace_expected"] = int((frame_df["lm_trace_expected"].fillna(0).astype(int) > 0).sum())
        if "lm_trace_emitted" in frame_df:
            analysis["frames_with_trace_emitted"] = int((frame_df["lm_trace_emitted"].fillna(0).astype(int) > 0).sum())
        if "lm_trace_row_count" in frame_df:
            analysis["lm_trace_row_count_total"] = int(frame_df["lm_trace_row_count"].fillna(0).sum())
        if "lm_initial_cost" in frame_df:
            analysis["initial_cost_mean"] = float(frame_df["lm_initial_cost"].fillna(0).mean())
        if "lm_final_cost" in frame_df:
            analysis["final_cost_mean"] = float(frame_df["lm_final_cost"].fillna(0).mean())
        if {"lm_initial_cost", "lm_final_cost"}.issubset(frame_df.columns):
            denom = frame_df["lm_initial_cost"].replace(0, np.nan)
            drop_ratio = ((frame_df["lm_initial_cost"] - frame_df["lm_final_cost"]) / denom).replace([np.inf, -np.inf], np.nan)
            analysis["cost_drop_ratio_mean"] = float(drop_ratio.dropna().mean()) if not drop_ratio.dropna().empty else 0.0
        if "lm_rejected_step_count" in frame_df:
            analysis["rejected_steps_count"] = int(frame_df["lm_rejected_step_count"].fillna(0).sum())
        analysis["optimization_diagnostics_available"] = any(
            key in analysis for key in ["lm_iteration_mean", "initial_cost_mean", "final_cost_mean", "rejected_steps_count"]
        )

    if lm_df is not None and not lm_df.empty:
        analysis["lm_iteration_rows_present"] = True
        analysis["lm_iteration_file_rows"] = int(len(lm_df))
        analysis["optimization_diagnostics_available"] = True
        analysis["lm_iteration_trace_reason"] = "trace_rows_present"
    else:
        trace_expected = analysis.get("frames_with_trace_expected", 0)
        trace_emitted = analysis.get("frames_with_trace_emitted", 0)
        trace_rows = analysis.get("lm_trace_row_count_total", 0)
        if lm_artifact is None:
            analysis["lm_iteration_trace_reason"] = "artifact_unknown"
        elif trace_expected > 0 and trace_emitted == 0 and trace_rows == 0:
            analysis["lm_iteration_trace_reason"] = "instrumentation_bug_suspected"
        elif lm_artifact.reason == "config_disabled":
            analysis["lm_iteration_trace_reason"] = "config_disabled_or_zero_iteration_run"
        elif lm_artifact.status in {"missing", "expected_missing"}:
            analysis["lm_iteration_trace_reason"] = "file_missing"
        elif lm_artifact.status == "empty":
            analysis["lm_iteration_trace_reason"] = "emitted_but_empty"
        else:
            analysis["lm_iteration_trace_reason"] = "instrumentation_bug_suspected"

    return analysis


def analyze_target_map_prep(frame_df: pd.DataFrame | None) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if frame_df is None or frame_df.empty or "target_map_prep_ms" not in frame_df.columns:
        return analysis
    df = frame_df.copy()
    analysis["available"] = True
    analysis["mean_ms"] = float(df["target_map_prep_ms"].fillna(0).mean())
    analysis["p95_ms"] = pct(df["target_map_prep_ms"].fillna(0), 0.95)
    analysis["max_ms"] = float(df["target_map_prep_ms"].fillna(0).max())
    substage_cols = [
        "target_snapshot_clone_ms",
        "target_voxel_lookup_prep_ms",
        "target_covariance_prep_ms",
        "source_to_target_transform_ms",
    ]
    available = [col for col in substage_cols if col in df.columns]
    analysis["substage_columns_present"] = available
    analysis["substage_columns_missing"] = [col for col in substage_cols if col not in df.columns]
    analysis["is_black_box"] = len(available) == 0
    if available:
        analysis["substage_summary"] = [
            {
                "stage": col,
                "mean_ms": float(df[col].fillna(0).mean()),
                "p95_ms": pct(df[col].fillna(0), 0.95),
                "max_ms": float(df[col].fillna(0).max()),
            }
            for col in available
        ]
    return analysis


def analyze_graph_problem_size(frame_df: pd.DataFrame | None) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if frame_df is None or frame_df.empty:
        return analysis
    df = frame_df.copy()
    analysis["available"] = True
    available_fields = []
    for col in [
        "active_control_point_count",
        "active_pose_key_count",
        "local_state_dimension",
        "imu_factor_count",
        "imu_residual_count",
        "lidar_factor_count",
        "gnss_factor_count",
        "local_residual_count",
        "backend_factor_count",
        "backend_state_count",
    ]:
        if col in df.columns:
            available_fields.append({
                "field": col,
                "mean": float(df[col].fillna(0).mean()),
                "p95": pct(df[col].fillna(0), 0.95),
                "max": float(df[col].fillna(0).max()),
            })
    analysis["available_fields"] = available_fields
    analysis["missing_graph_size_telemetry"] = [
        col for col in ["local_state_dimension", "imu_factor_count", "local_residual_count"]
        if col not in df.columns
    ]
    return analysis


def analyze_bucket_diagnostics(frame_df: pd.DataFrame | None, bucket_df: pd.DataFrame | None) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False}
    if frame_df is None or frame_df.empty:
        return analysis
    df = frame_df.copy()
    analysis["available"] = True
    analysis["bucket_mode"] = sorted(df["bucket_mode"].dropna().astype(str).unique().tolist()) if "bucket_mode" in df.columns else []
    if "actual_bucket_count" in df.columns:
        counts = df["actual_bucket_count"].fillna(0)
        analysis["actual_bucket_count_mean"] = float(counts.mean())
        analysis["actual_bucket_count_p95"] = pct(counts, 0.95)
        analysis["actual_bucket_count_max"] = float(counts.max())
        if analysis["bucket_mode"] == ["SINGLE_BUCKET"]:
            analysis["bucket_count_consistent_with_mode"] = bool((counts == 1).all())
        else:
            analysis["bucket_count_consistent_with_mode"] = None
    if bucket_df is not None and not bucket_df.empty and "points_in_bucket" in bucket_df.columns:
        points = bucket_df["points_in_bucket"].fillna(0)
        analysis["points_in_bucket_mean"] = float(points.mean())
        analysis["points_in_bucket_p95"] = pct(points, 0.95)
        analysis["points_in_bucket_max"] = float(points.max())
        mean_points = float(points.mean())
        analysis["points_in_bucket_cv"] = float(points.std(ddof=0) / mean_points) if mean_points > 0 else 0.0
        if "representative_time" in bucket_df.columns:
            reps = bucket_df["representative_time"].fillna(0)
            analysis["representative_time_mean"] = float(reps.mean())
            analysis["representative_time_min"] = float(reps.min())
            analysis["representative_time_max"] = float(reps.max())
    return analysis


def analyze_correlations(
    frame_df: pd.DataFrame | None,
    bucket_df: pd.DataFrame | None,
    warning_df: pd.DataFrame | None,
) -> dict[str, Any]:
    analysis: dict[str, Any] = {"available": False, "method": "pearson"}
    if frame_df is None or frame_df.empty:
        return analysis
    df = frame_df.copy()
    stage_cols = stage_names_from_df(df)
    if stage_cols:
        df["frame_total_ms"] = df[stage_cols].sum(axis=1)
    if bucket_df is not None and not bucket_df.empty and "frame_id" in bucket_df.columns:
        grouped = bucket_df.groupby("frame_id").agg(
            points_in_bucket=("points_in_bucket", "sum"),
            factor_total_ms=("factor_total_ms", "sum"),
            match_ratio=("match_ratio", "mean"),
            inlier_ratio=("inlier_ratio", "mean"),
        )
        df = df.merge(grouped, on="frame_id", how="left")
    if warning_df is not None and not warning_df.empty and "frame_id" in warning_df.columns and "warning_count" in warning_df.columns:
        df = df.merge(warning_df[["frame_id", "warning_count"]], on="frame_id", how="left")
        df["warning_count_for_frame"] = df["warning_count"]
        df = df.drop(columns=["warning_count"])

    def corr(col_a: str, col_b: str) -> str:
        if col_a not in df.columns or col_b not in df.columns:
            return "missing_field"
        pair = df[[col_a, col_b]].dropna()
        if len(pair) < 3:
            return "insufficient_samples"
        if float(pair[col_a].std(ddof=0)) == 0.0 or float(pair[col_b].std(ddof=0)) == 0.0:
            return "constant_series"
        return f"{pair[col_a].corr(pair[col_b], method='pearson'):.3f}"

    pairs = {
        "frame_total_ms_vs_lm_solve_ms": ("frame_total_ms", "lm_solve_ms"),
        "frame_total_ms_vs_target_map_prep_ms": ("frame_total_ms", "target_map_prep_ms"),
        "frame_total_ms_vs_points_in_bucket": ("frame_total_ms", "points_in_bucket"),
        "lm_solve_ms_vs_points_in_bucket": ("lm_solve_ms", "points_in_bucket"),
        "factor_total_ms_vs_points_in_bucket": ("factor_total_ms", "points_in_bucket"),
        "frame_total_ms_vs_warning_count": ("frame_total_ms", "warning_count_for_frame"),
        "target_map_prep_ms_vs_warning_count": ("target_map_prep_ms", "warning_count_for_frame"),
    }
    analysis["available"] = True
    analysis["correlations"] = {name: corr(a, b) for name, (a, b) in pairs.items()}
    return analysis


def run_external_plot(script: Path, args: list[str]) -> dict[str, Any]:
    proc = subprocess.run(
        [sys.executable, str(script), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    return {
        "script": script.name,
        "returncode": proc.returncode,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip(),
    }


def integrate_existing_plots(run_dir: Path, dataframes: dict[str, pd.DataFrame], out_dir: Path) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    ensure_dir(out_dir)

    icp_csv = run_dir / "export" / "icp_quality.csv"
    pipeline_csv = run_dir / "profiling" / "pipeline_timing.csv"
    if "icp_quality" in dataframes and "pipeline_timing" in dataframes:
        icp_out = out_dir / "icp"
        ensure_dir(icp_out)
        results.append(run_external_plot(PACKAGE_ROOT / "tools" / "plot_icp_timing.py", [str(icp_csv), str(pipeline_csv), str(icp_out)]))

    gnss_csv = run_dir / "export" / "gnss_factor_debug.csv"
    if "gnss_factor_debug" in dataframes and not dataframes["gnss_factor_debug"].empty:
        gnss_out = out_dir / "gnss_factor_debug"
        ensure_dir(gnss_out)
        results.append(run_external_plot(PACKAGE_ROOT / "tools" / "plot_gnss_factor_debug.py", ["--csv", str(gnss_csv), "--out", str(gnss_out)]))

    araim_csv = run_dir / "export" / "araim.csv"
    if "araim" in dataframes and not dataframes["araim"].empty:
        araim_out = out_dir / "araim"
        ensure_dir(araim_out)
        results.append(run_external_plot(PACKAGE_ROOT / "tools" / "plot_araim_timeline.py", [str(araim_csv), str(araim_out)]))

    return results


def analyze_seed_mode_comparison(
    current_run_dir: Path,
    current_config_summary: dict[str, Any],
    current_jump_analysis: dict[str, Any],
    baseline_run_dir: Path | None,
) -> dict[str, Any]:
    comparison: dict[str, Any] = {"available": False}
    if baseline_run_dir is None:
        return comparison

    baseline_metadata = load_metadata(baseline_run_dir)
    baseline_runtime_summary = parse_runtime_logs(baseline_run_dir)
    baseline_configs = load_runtime_configs(baseline_metadata)
    baseline_config_summary = build_config_summary(
        baseline_configs, baseline_runtime_summary, baseline_metadata
    )
    baseline_dataframes = load_available_frames(baseline_run_dir)
    baseline_jump_analysis = analyze_jump_diagnostics(
        baseline_dataframes.get("jump_diagnostics"),
        str(
            baseline_config_summary.get("runtime_final_pose_surface")
            or baseline_config_summary.get("final_pose_surface")
            or "active_window"
        ),
        baseline_config_summary,
    )
    if not baseline_jump_analysis.get("available") or not current_jump_analysis.get("available"):
        return comparison

    def metric_row(label: str, key: str) -> list[Any]:
        baseline_value = baseline_jump_analysis.get(key)
        current_value = current_jump_analysis.get(key)
        delta_value = None
        if isinstance(baseline_value, (int, float)) and isinstance(current_value, (int, float)):
            delta_value = float(current_value) - float(baseline_value)
        return [label, baseline_value, current_value, delta_value]

    metric_rows = [
        metric_row("start->frontend rotation p95", "start_to_frontend_rotation_p95"),
        metric_row("start->frontend rotation max", "start_to_frontend_rotation_max"),
        metric_row("frontend->final rotation p95", "frontend_to_final_rotation_p95"),
        metric_row("frontend->final rotation max", "frontend_to_final_rotation_max"),
        metric_row("frontend->final pitch p95", "frontend_to_final_pitch_abs_p95"),
        metric_row("frontend->final pitch max", "frontend_to_final_pitch_abs_max"),
        metric_row("frontend->final roll p95", "frontend_to_final_roll_abs_p95"),
        metric_row("frontend->final roll max", "frontend_to_final_roll_abs_max"),
        metric_row("frontend->final translation p95", "frontend_to_final_translation_p95"),
        metric_row("match_ratio mean", "match_ratio_mean"),
        metric_row("inlier_ratio mean", "inlier_ratio_mean"),
        metric_row("target_point_count mean", "target_point_count_mean"),
        metric_row("target_voxel_count mean", "target_voxel_count_mean"),
        metric_row("seed fallback frame count", "seed_fallback_frame_count"),
    ]

    comparison["available"] = True
    comparison["baseline_run_dir"] = str(baseline_run_dir)
    comparison["current_run_dir"] = str(current_run_dir)
    comparison["baseline_frontend_seed_mode"] = baseline_config_summary.get(
        "runtime_frontend_seed_mode", "last_pose_copy"
    )
    comparison["current_frontend_seed_mode"] = current_config_summary.get(
        "runtime_frontend_seed_mode", "last_pose_copy"
    )
    comparison["metric_rows"] = metric_rows
    if (
        comparison["baseline_frontend_seed_mode"] == "last_pose_copy"
        and comparison["current_frontend_seed_mode"] == "imu_forward_prediction"
    ):
        rotation_delta = current_jump_analysis.get("start_to_frontend_rotation_p95")
        baseline_rotation = baseline_jump_analysis.get("start_to_frontend_rotation_p95")
        if isinstance(rotation_delta, (int, float)) and isinstance(baseline_rotation, (int, float)):
            delta = float(rotation_delta) - float(baseline_rotation)
            if delta < -1e-6:
                comparison["summary"] = (
                    "imu_forward_prediction improves start->frontend rotation versus last_pose_copy baseline"
                )
            elif delta > 1e-6:
                comparison["summary"] = (
                    "imu_forward_prediction does not improve start->frontend rotation versus last_pose_copy baseline"
                )
            else:
                comparison["summary"] = (
                    "imu_forward_prediction and last_pose_copy are neutral on start->frontend rotation in this pair"
                )
    return comparison


def detect_seed_compare_findings(seed_compare_analysis: dict[str, Any]) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    if not seed_compare_analysis.get("available"):
        return findings
    summary = seed_compare_analysis.get("summary")
    if summary:
        evidence = []
        for label, baseline_value, current_value, delta_value in seed_compare_analysis.get("metric_rows", []):
            if label in {
                "start->frontend rotation p95",
                "frontend->final rotation p95",
                "frontend->final pitch p95",
                "frontend->final roll p95",
                "frontend->final translation p95",
            }:
                evidence.append(
                    f"{label}: baseline={baseline_value}, current={current_value}, delta={delta_value}"
                )
        findings.append({
            "severity": "info" if "improves" in summary else "warn",
            "title": "Seed Improvement Comparison",
            "evidence": f"{summary}; " + "; ".join(evidence),
        })
    return findings


def detect_findings(
    config_summary: dict[str, Any],
    runtime_summary: dict[str, Any],
    artifact_statuses: list[ArtifactStatus],
    mode_consistency: dict[str, Any],
    frontend_analysis: dict[str, Any],
    jump_analysis: dict[str, Any],
    solver_update_analysis: dict[str, Any],
    lidar_factor_internal_analysis: dict[str, Any],
    optimizer_analysis: dict[str, Any],
    target_map_analysis: dict[str, Any],
    bucket_analysis: dict[str, Any],
    pipeline_analysis: dict[str, Any],
) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []
    runtime_gravity_state_mode = str(config_summary.get("runtime_gravity_state_mode") or "shared_optimized")
    runtime_velocity_state_mode = str(config_summary.get("runtime_velocity_state_mode") or "optimize")
    runtime_bias_state_mode = str(config_summary.get("runtime_bias_state_mode") or "shared_singleton")
    runtime_bias_optimized = maybe_bool(config_summary.get("runtime_bias_optimized"))
    runtime_bias_source_of_truth = str(config_summary.get("runtime_bias_source_of_truth") or "shared_singleton_registry")
    runtime_bias_transition_prior_enabled = maybe_bool(
        config_summary.get("runtime_bias_transition_prior_enabled")
    )
    runtime_bias_transition_prior_strength = config_summary.get("runtime_bias_transition_prior_strength", "n/a")
    runtime_bias_can_be_survivor_anchor = maybe_bool(
        config_summary.get("runtime_bias_can_be_survivor_anchor")
    )
    runtime_bias_writeback_mode = str(
        config_summary.get("runtime_bias_writeback_mode") or "n/a"
    )
    runtime_frontend_seed_mode = str(config_summary.get("runtime_frontend_seed_mode") or "last_pose_copy")
    runtime_imu_forward_prediction_enabled = maybe_bool(
        config_summary.get("runtime_imu_forward_prediction_enabled")
    )
    runtime_frontend_seed_fallback_used = maybe_bool(
        config_summary.get("runtime_frontend_seed_fallback_used")
    )
    runtime_frontend_seed_source = str(
        config_summary.get("runtime_frontend_seed_source") or "last_pose_copy"
    )
    runtime_frontend_seed_imu_sample_count = config_summary.get("runtime_frontend_seed_imu_sample_count", "n/a")
    runtime_velocity_optimized = maybe_bool(config_summary.get("runtime_velocity_optimized"))
    runtime_has_gnss_constraints = maybe_bool(config_summary.get("runtime_has_gnss_constraints"))
    top_jump_frames = jump_analysis.get("top_frontend_to_final_translation_frames", [])
    delta_yaw_vs_gyro_bias_norm = jump_analysis.get("delta_yaw_vs_gyro_bias_norm")
    delta_yaw_vs_accel_bias_norm = jump_analysis.get("delta_yaw_vs_accel_bias_norm")

    def is_bias_anchor(text: Any) -> bool:
        summary = str(text or "").strip().lower()
        return summary.startswith("j") or summary.startswith("k")

    bias_anchor_present_in_top_jumps = any(
        is_bias_anchor(item.get("oldest_survivor_key_summary"))
        or is_bias_anchor(item.get("carried_boundary_oldest_key_summary"))
        for item in top_jump_frames
    )

    if runtime_gravity_state_mode == "external_reference":
        findings.append({
            "severity": "info",
            "title": "gravity now runs in external-reference mode; solver no longer optimizes gravity as a shared state",
            "evidence": (
                f"runtime_gravity_state_mode={runtime_gravity_state_mode}, "
                f"runtime_gravity_reference_source={config_summary.get('runtime_gravity_reference_source', 'n/a')}, "
                f"runtime_gravity_reference_vector={config_summary.get('runtime_gravity_reference_vector', 'n/a')}"
            ),
        })

    if runtime_velocity_state_mode == "keep_but_not_optimize" and runtime_velocity_optimized is False:
        findings.append({
            "severity": "info",
            "title": "velocity is kept for publish/bookkeeping but not optimized in no-GNSS mode",
            "evidence": (
                f"runtime_velocity_state_mode={runtime_velocity_state_mode}, "
                f"runtime_velocity_optimized={runtime_velocity_optimized}, "
                f"runtime_has_gnss_constraints={runtime_has_gnss_constraints}"
            ),
        })

    if runtime_bias_state_mode == "lagged_keyed" and runtime_bias_optimized is True:
        findings.append({
            "severity": "info",
            "title": "bias now runs as lagged keyed state; persistent shared singleton semantics removed",
            "evidence": (
                f"runtime_bias_state_mode={runtime_bias_state_mode}, "
                f"runtime_bias_optimized={runtime_bias_optimized}, "
                f"runtime_bias_source_of_truth={runtime_bias_source_of_truth}"
            ),
        })
        if (
            runtime_bias_source_of_truth == "active_lagged_bias_keys"
            and runtime_bias_writeback_mode == "lagged_authoritative_with_mirror_cache"
        ):
            findings.append({
                "severity": "info",
                "title": "lagged bias keys are initialized from authoritative previous lagged bias",
                "evidence": (
                    f"runtime_bias_source_of_truth={runtime_bias_source_of_truth}, "
                    f"runtime_bias_writeback_mode={runtime_bias_writeback_mode}"
                ),
            })
        if runtime_bias_transition_prior_enabled is True:
            findings.append({
                "severity": "info",
                "title": "lagged bias transition prior is present and appears sufficiently constraining",
                "evidence": (
                    f"runtime_bias_transition_prior_enabled={runtime_bias_transition_prior_enabled}, "
                    f"runtime_bias_transition_prior_strength={runtime_bias_transition_prior_strength}"
                ),
            })
        if runtime_bias_can_be_survivor_anchor is False and not bias_anchor_present_in_top_jumps:
            findings.append({
                "severity": "info",
                "title": "bias no longer serves as boundary survivor anchor",
                "evidence": (
                    f"runtime_bias_can_be_survivor_anchor={runtime_bias_can_be_survivor_anchor}, "
                    f"top_jump_bias_anchor_present={bias_anchor_present_in_top_jumps}"
                ),
            })
        if runtime_bias_writeback_mode == "lagged_authoritative_with_mirror_cache":
            findings.append({
                "severity": "info",
                "title": "bias write-back no longer forms a bad-value reseed loop",
                "evidence": (
                    f"runtime_bias_source_of_truth={runtime_bias_source_of_truth}, "
                    f"runtime_bias_writeback_mode={runtime_bias_writeback_mode}"
                ),
            })
        if (
            isinstance(delta_yaw_vs_gyro_bias_norm, (int, float))
            and isinstance(delta_yaw_vs_accel_bias_norm, (int, float))
            and delta_yaw_vs_gyro_bias_norm < 0.3
            and delta_yaw_vs_accel_bias_norm < 0.3
        ):
            findings.append({
                "severity": "info",
                "title": "bias norm growth is reduced after lifecycle fix",
                "evidence": (
                    f"delta_yaw_vs_gyro_bias_norm={delta_yaw_vs_gyro_bias_norm}, "
                    f"delta_yaw_vs_accel_bias_norm={delta_yaw_vs_accel_bias_norm}"
                ),
            })

    if (
        runtime_frontend_seed_mode == "imu_forward_prediction"
        and runtime_imu_forward_prediction_enabled is True
        and runtime_frontend_seed_source != "last_pose_copy"
    ):
        findings.append({
            "severity": "info",
            "title": "frontend seed now uses IMU forward prediction instead of last-pose copy",
            "evidence": (
                f"runtime_frontend_seed_mode={runtime_frontend_seed_mode}, "
                f"runtime_imu_forward_prediction_enabled={runtime_imu_forward_prediction_enabled}, "
                f"runtime_frontend_seed_source={runtime_frontend_seed_source}, "
                f"runtime_frontend_seed_imu_sample_count={runtime_frontend_seed_imu_sample_count}"
            ),
        })
        fallback_frames = int(jump_analysis.get("seed_fallback_frame_count", 0) or 0)
        if runtime_frontend_seed_fallback_used or fallback_frames > 0:
            findings.append({
                "severity": "warn" if fallback_frames > 0 else "info",
                "title": f"frontend seed fell back to last-pose copy on {fallback_frames} frames",
                "evidence": (
                    f"runtime_frontend_seed_fallback_used={runtime_frontend_seed_fallback_used}, "
                    f"runtime_frontend_seed_source={runtime_frontend_seed_source}, "
                    f"seed_fallback_frame_count={fallback_frames}"
                ),
            })

    if runtime_gravity_state_mode == "external_reference" and runtime_velocity_state_mode == "keep_but_not_optimize":
        findings.append({
            "severity": "info",
            "title": "current run is closer to GLIM-style reference-gravity / reduced-velocity-state semantics",
            "evidence": (
                f"gravity={runtime_gravity_state_mode}, "
                f"velocity={runtime_velocity_state_mode}, "
                f"runtime_velocity_optimized={runtime_velocity_optimized}"
            ),
        })

    if runtime_bias_state_mode == "lagged_keyed" and runtime_frontend_seed_mode == "imu_forward_prediction":
        findings.append({
            "severity": "info",
            "title": "CT frontend is now closer to GLIM-style motion-prior seeding",
            "evidence": (
                f"runtime_bias_state_mode={runtime_bias_state_mode}, "
                f"runtime_frontend_seed_mode={runtime_frontend_seed_mode}, "
                f"runtime_imu_forward_prediction_enabled={runtime_imu_forward_prediction_enabled}"
            ),
        })

    issue_count = len(runtime_summary.get("issues", []))
    if issue_count == 0:
        findings.append({
            "severity": "info",
            "title": "No runtime warnings or errors were found",
            "evidence": "All scanned runtime logs were free of warn/error/critical lines.",
        })
    else:
        findings.append({
            "severity": "warn",
            "title": f"Runtime logs contain {issue_count} warning/error lines",
            "evidence": "See report section 'Errors And Warnings' for the most common message patterns.",
        })

    expected_missing = [a for a in artifact_statuses if a.status == "expected_missing"]
    if expected_missing:
        findings.append({
            "severity": "warn",
            "title": f"{len(expected_missing)} enabled artifacts are missing",
            "evidence": ", ".join(a.name for a in expected_missing[:6]),
        })

    empties = [a for a in artifact_statuses if a.status == "empty"]
    if empties:
        findings.append({
            "severity": "info",
            "title": f"{len(empties)} artifacts exist but contain no data",
            "evidence": ", ".join(a.name for a in empties[:6]),
        })

    if mode_consistency.get("loaded_modules_that_contradict_mode"):
        findings.append({
            "severity": "warn",
            "title": "Loaded modules contradict the effective runtime mode",
            "evidence": ", ".join(mode_consistency.get("loaded_modules_that_contradict_mode", [])),
        })

    if frontend_analysis.get("available"):
        stage_summary = frontend_analysis.get("stage_summary", [])
        if stage_summary:
            top = stage_summary[0]
            findings.append({
                "severity": "info",
                "title": f"Dominant frontend stage is {top['stage']}",
                "evidence": f"mean={top['mean_ms']:.3f} ms, p95={top['p95_ms']:.3f} ms",
            })

        mode_values = frontend_analysis.get("frontend_only_mode_values", [])
        if mode_values == [1]:
            findings.append({
                "severity": "info",
                "title": "All profiled frames ran in frontend-only mode",
                "evidence": "frontend_only_mode column is consistently 1 in frontend_frame_profile.csv",
            })

        slow_frames = frontend_analysis.get("slow_frames", [])
        if slow_frames:
            worst = slow_frames[0]
            findings.append({
                "severity": "info",
                "title": f"Slowest frontend frame is frame {worst.get('frame_id', '?')}",
                "evidence": f"frame_total_ms={worst.get('frame_total_ms', 0.0):.3f}",
            })

        if "bucket_mode" in frontend_analysis and frontend_analysis["bucket_mode"] == ["SINGLE_BUCKET"] and bucket_analysis.get("available"):
            findings.append({
                "severity": "info",
                "title": "Frontend ran in SINGLE_BUCKET mode",
                "evidence": (
                    f"mean points_per_bucket={bucket_analysis.get('points_in_bucket_mean', 0.0):.1f}, "
                    f"mean factor_total_ms={bucket_analysis.get('factor_total_mean_ms', 0.0):.3f}"
                ),
            })

    if jump_analysis.get("available"):
        dominance = jump_analysis.get("dominance", "jump diagnostics available")
        frontend_cause = jump_analysis.get("frontend_jump_cause")
        frontend_to_final_cause = jump_analysis.get("frontend_to_final_jump_cause")
        top_front = jump_analysis.get("top_start_to_frontend_translation_frames", [])
        top_solver = jump_analysis.get("top_frontend_to_final_translation_frames", [])
        dominance_evidence_parts = [
            f"jump_rows={jump_analysis.get('jump_rows', 0)}",
            f"start->frontend p95={jump_analysis.get('start_to_frontend_translation_p95', 0.0):.3f} m",
            f"frontend->final p95={jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f} m",
        ]
        if top_front:
            frame = top_front[0]
            dominance_evidence_parts.append(
                f"top start->frontend frame={frame.get('frame_id', '?')} layout=[{(frame.get('lidar_layout_domain_begin') or 0.0):.6f},{(frame.get('lidar_layout_domain_end') or 0.0):.6f}]"
            )
            dominance_evidence_parts.append(
                f"support={frame.get('lidar_support_keys_summary', '')} query={frame.get('frontend_pose_support_keys_summary', '')}"
            )
            dominance_evidence_parts.append(
                f"same_support_recalc={frame.get('recalculated_lidar_same_support_factor_count', 'n/a')}"
            )
        findings.append({
            "severity": "warn" if "jump appears" in dominance else "info",
            "title": dominance,
            "evidence": "; ".join(part for part in dominance_evidence_parts if part),
        })
        if frontend_cause:
            findings.append({
                "severity": "warn" if "dominated" in frontend_cause or "suspicious" in frontend_cause else "info",
                "title": frontend_cause,
                "evidence": jump_analysis.get("frontend_jump_cause_evidence", ""),
            })
        if frontend_to_final_cause:
            findings.append({
                "severity": "warn" if "dominated" in frontend_to_final_cause or "suspicious" in frontend_to_final_cause or "amplified" in frontend_to_final_cause or "mixed" in frontend_to_final_cause else "info",
                "title": frontend_to_final_cause,
                "evidence": jump_analysis.get("frontend_to_final_jump_cause_evidence", ""),
            })
        final_pose_surface_effect = jump_analysis.get("final_pose_surface_effect")
        if final_pose_surface_effect:
            findings.append({
                "severity": "info" if "keeps" in final_pose_surface_effect else "warn",
                "title": final_pose_surface_effect,
                "evidence": jump_analysis.get("final_pose_surface_effect_evidence", ""),
            })
        strict_local_residual_dominance = jump_analysis.get("strict_local_residual_dominance")
        if strict_local_residual_dominance:
            findings.append({
                "severity": "warn" if "dominated" in strict_local_residual_dominance or "mixed" in strict_local_residual_dominance else "info",
                "title": strict_local_residual_dominance,
                "evidence": jump_analysis.get("strict_local_residual_cause_evidence", ""),
            })
        strict_local_rotation_subtype = jump_analysis.get("strict_local_rotation_subtype")
        if strict_local_rotation_subtype and strict_local_rotation_subtype in {
            "strict-local residual appears yaw-dominated",
            "strict-local residual appears pitch/roll-dominated",
        }:
            findings.append({
                "severity": "info",
                "title": strict_local_rotation_subtype,
                "evidence": jump_analysis.get("strict_local_residual_cause_evidence", ""),
            })
        strict_local_residual_cause = jump_analysis.get("strict_local_residual_cause")
        if strict_local_residual_cause and "not strongly isolated" not in strict_local_residual_cause:
            findings.append({
                "severity": "warn" if "suspicious" in strict_local_residual_cause or "drift" in strict_local_residual_cause or "inconsistency" in strict_local_residual_cause else "info",
                "title": strict_local_residual_cause,
                "evidence": jump_analysis.get("strict_local_residual_cause_evidence", ""),
            })
        pitch_roll_root_cause_summary = jump_analysis.get("pitch_roll_root_cause_summary")
        if pitch_roll_root_cause_summary:
            findings.append({
                "severity": "warn" if "mismatch" in pitch_roll_root_cause_summary or "amplification" in pitch_roll_root_cause_summary else "info",
                "title": pitch_roll_root_cause_summary,
                "evidence": jump_analysis.get("pitch_roll_root_cause_evidence", ""),
            })
        yaw_root_cause_summary = jump_analysis.get("yaw_root_cause_summary")
        if yaw_root_cause_summary:
            findings.append({
                "severity": "warn" if "dominated" in yaw_root_cause_summary or "mixed" in yaw_root_cause_summary else "info",
                "title": yaw_root_cause_summary,
                "evidence": jump_analysis.get("yaw_root_cause_evidence", ""),
            })
        isolation_effect = jump_analysis.get("isolation_effect")
        if isolation_effect and "baseline strict-local run" not in isolation_effect:
            findings.append({
                "severity": "warn" if "unacceptable" in isolation_effect or "little effect" in isolation_effect else "info",
                "title": isolation_effect,
                "evidence": jump_analysis.get("isolation_effect_evidence", ""),
            })
        runtime_final_pose_surface = jump_analysis.get("runtime_final_pose_surface")
        if (
            runtime_final_pose_surface == "strict_local"
            and final_pose_surface_effect == "strict-local final pose surface significantly reduces frontend->final jump"
        ):
            findings.append({
                "severity": "info",
                "title": "strict-local final pose surface is now the intended default",
                "evidence": (
                    f"runtime_final_pose_surface={runtime_final_pose_surface}; "
                    f"frontend->final p95={(jump_analysis.get('frontend_to_final_translation_p95') or 0.0):.3f} m; "
                    f"active_window->strict_local p95={(jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_p95') or 0.0):.3f} m"
                ),
            })
            findings.append({
                "severity": "info",
                "title": "active-window surface remains available for debug/regression only",
                "evidence": (
                    "active-window postsolve diagnostics and surface-difference telemetry remain enabled; "
                    "use final_pose_surface=active_window only for regression reproduction and boundary-surface comparison"
                ),
            })
        top_strict_local = jump_analysis.get("top_strict_local_residual_frames", [])
        if top_strict_local:
            frame = top_strict_local[0]
            findings.append({
                "severity": "info",
                "title": f"largest strict-local residual frame is {frame.get('frame_id', '?')}",
                "evidence": (
                    f"delta_t={(frame.get('delta_frontend_to_final_translation_norm') or 0.0):.3f} m; "
                    f"delta_r={(frame.get('delta_frontend_to_final_rotation_rad') or 0.0):.3f} rad; "
                    f"yaw/pitch/roll="
                    f"{(frame.get('delta_frontend_to_final_yaw_rad') or 0.0):.3f}/"
                    f"{(frame.get('delta_frontend_to_final_pitch_rad') or 0.0):.3f}/"
                    f"{(frame.get('delta_frontend_to_final_roll_rad') or 0.0):.3f} rad; "
                    f"dx/dy/dz="
                    f"{(frame.get('delta_frontend_to_final_dx') or 0.0):.3f}/"
                    f"{(frame.get('delta_frontend_to_final_dy') or 0.0):.3f}/"
                    f"{(frame.get('delta_frontend_to_final_dz') or 0.0):.3f} m; "
                    f"strict_local_query_reason={frame.get('strict_local_query_reason', '')}; "
                    f"recalc_imu={frame.get('recalculated_imu_factor_count', 'n/a')}; "
                    f"recalc_prior={frame.get('recalculated_prior_factor_count', 'n/a')}; "
                    f"relin_shared={frame.get('relinearized_shared_variable_count', 'n/a')}"
                ),
            })
        if top_solver:
            frame = top_solver[0]
            findings.append({
                "severity": "info",
                "title": f"largest frontend->final jump frame is {frame.get('frame_id', '?')}",
                "evidence": (
                    f"delta={(frame.get('delta_frontend_to_final_translation_norm') or 0.0):.3f} m; "
                    f"frontend->postsolve_active_window={(frame.get('delta_frontend_to_postsolve_query_translation_norm') or 0.0):.3f} m; "
                    f"frontend->postsolve_strict_local={(frame.get('delta_frontend_to_postsolve_strict_local_translation_norm') or 0.0):.3f} m; "
                    f"active_window->strict_local={(frame.get('delta_postsolve_active_window_to_postsolve_strict_local_translation_norm') or 0.0):.3f} m; "
                    f"postsolve_active_window->final={(frame.get('delta_postsolve_query_to_final_translation_norm') or 0.0):.3f} m; "
                    f"postsolve_reason={frame.get('postsolve_query_support_mismatch_reason', '')}; "
                    f"strict_reason={frame.get('postsolve_strict_local_support_mismatch_reason', '')}; "
                    f"same_support_recalc={frame.get('recalculated_lidar_same_support_factor_count', 'n/a')}; "
                    f"cross_support_recalc={frame.get('recalculated_lidar_cross_support_factor_count', 'n/a')}"
                ),
            })

    if frontend_analysis.get("available") and optimizer_analysis.get("lm_iteration_rows_present") is False:
        stage_summary = frontend_analysis.get("stage_summary", [])
        if stage_summary and stage_summary[0]["stage"] == "lm_solve_ms":
            findings.append({
                "severity": "warn",
                "title": "LM solve dominates runtime but iteration-level telemetry is unavailable",
                "evidence": f"lm_iteration_trace_reason={optimizer_analysis.get('lm_iteration_trace_reason', 'unknown')}",
            })

    if solver_update_analysis.get("available"):
        stage_summary = solver_update_analysis.get("stage_summary", [])
        if stage_summary:
            top = stage_summary[0]
            findings.append({
                "severity": "info",
                "title": f"Dominant solver-update substage is {top['stage']}",
                "evidence": f"mean={top['mean_ms']:.3f} ms, share={100.0 * top['mean_share']:.1f}%",
            })

        unavailable = solver_update_analysis.get("unavailable_internal_fields", [])
        if unavailable and float(solver_update_analysis.get("solver_update_ms_mean", 0.0)) > 0.0:
            findings.append({
                "severity": "warn",
                "title": "solver 内部 telemetry blind spot",
                "evidence": ", ".join(unavailable[:6]),
            })

        if (
            bool(solver_update_analysis.get("unavailable_internal_timing"))
            and (
                (isinstance(solver_update_analysis.get("reeliminated_variable_vs_solver_update_corr"), float)
                 and solver_update_analysis.get("reeliminated_variable_vs_solver_update_corr", 0.0) > 0.7)
                or
                (isinstance(solver_update_analysis.get("recalculated_factor_vs_solver_update_corr"), float)
                 and solver_update_analysis.get("recalculated_factor_vs_solver_update_corr", 0.0) > 0.7)
            )
        ):
            findings.append({
                "severity": "warn",
                "title": "likely pseudo-incremental heavy reelimination/relinearization pressure",
                "evidence": (
                    f"corr(reeliminated,solver_update)="
                    f"{solver_update_analysis.get('reeliminated_variable_vs_solver_update_corr', 'n/a')}; "
                    f"corr(recalculated_factor,solver_update)="
                    f"{solver_update_analysis.get('recalculated_factor_vs_solver_update_corr', 'n/a')}"
                ),
            })

        fallback_used_values = solver_update_analysis.get("fallback_used_values", [])
        if (
            "fallback_rebuild_ms_mean" in solver_update_analysis
            and float(solver_update_analysis.get("fallback_rebuild_ms_mean", 0.0)) == 0.0
            and all(int(v) == 0 for v in fallback_used_values)
        ):
            findings.append({
                "severity": "info",
                "title": "no evidence of fallback/reseed cost",
                "evidence": "fallback_used stayed 0 and fallback_rebuild_ms stayed 0 for this run",
            })

        new_factor_corr = solver_update_analysis.get("new_factor_vs_active_window_corr")
        new_value_corr = solver_update_analysis.get("new_value_vs_active_window_corr")
        if (
            isinstance(new_factor_corr, float) and new_factor_corr > 0.85
        ) or (
            isinstance(new_value_corr, float) and new_value_corr > 0.85
        ):
            findings.append({
                "severity": "warn",
                "title": "incremental delta path may be drifting toward window-wide rebuild",
                "evidence": (
                    f"corr(new_factor,active_window)={new_factor_corr if new_factor_corr is not None else 'n/a'}; "
                    f"corr(new_value,active_window)={new_value_corr if new_value_corr is not None else 'n/a'}"
                ),
            })
        elif isinstance(new_factor_corr, float) and isinstance(new_value_corr, float):
            findings.append({
                "severity": "info",
                "title": "delta-only structure still healthy",
                "evidence": (
                    f"corr(new_factor,active_window)={new_factor_corr:.3f}; "
                    f"corr(new_value,active_window)={new_value_corr:.3f}"
                ),
            })

        dominant_family = solver_update_analysis.get("dominant_recalculated_family")
        dominant_family_corr = solver_update_analysis.get("dominant_recalculated_family_corr")
        second_family_corr = solver_update_analysis.get("second_recalculated_family_corr")
        if (
            isinstance(dominant_family, str)
            and isinstance(dominant_family_corr, float)
            and dominant_family_corr >= 0.7
            and (
                second_family_corr is None
                or not isinstance(second_family_corr, float)
                or dominant_family_corr - second_family_corr >= 0.1
            )
        ):
            findings.append({
                "severity": "warn",
                "title": f"solver churn appears {dominant_family}-family dominated",
                "evidence": f"corr(recalculated_{dominant_family.lower()}_factor_count,solver_update_ms)={dominant_family_corr:.3f}",
            })

        active_family_corrs = [
            solver_update_analysis.get("active_window_imu_factor_vs_solver_update_corr"),
            solver_update_analysis.get("active_window_shared_jkg_touching_factor_vs_solver_update_corr"),
        ]
        max_active_family_corr = max(
            [corr for corr in active_family_corrs if isinstance(corr, float)],
            default=None,
        )
        if (
            isinstance(max_active_family_corr, float)
            and max_active_family_corr < 0.4
            and isinstance(dominant_family_corr, float)
            and dominant_family_corr >= 0.7
        ):
            findings.append({
                "severity": "info",
                "title": "active factor family size does not explain runtime; recalculated family mix does",
                "evidence": (
                    f"max corr(active_window_family,solver_update_ms)={max_active_family_corr:.3f}; "
                    f"dominant recalculated family={dominant_family} "
                    f"(corr={dominant_family_corr:.3f})"
                ),
            })

        recalculated_shared_jkg_corr = solver_update_analysis.get("recalculated_shared_jkg_touching_factor_vs_solver_update_corr")
        active_window_shared_jkg_mean = solver_update_analysis.get("active_window_shared_jkg_touching_factor_count_mean", 0.0)
        if (
            isinstance(active_window_shared_jkg_mean, (int, float))
            and float(active_window_shared_jkg_mean) > 0.0
            and isinstance(recalculated_shared_jkg_corr, float)
            and isinstance(dominant_family_corr, float)
            and recalculated_shared_jkg_corr < dominant_family_corr - 0.2
        ):
            findings.append({
                "severity": "info",
                "title": "shared j/k/g touching factors are numerous but not the dominant recalculated family",
                "evidence": (
                    f"active_window_shared_jkg_touching_factor_count_mean={float(active_window_shared_jkg_mean):.3f}; "
                    f"corr(recalculated_shared_jkg_touching_factor_count,solver_update_ms)={recalculated_shared_jkg_corr:.3f}; "
                    f"dominant_family={dominant_family} "
                    f"(corr={dominant_family_corr:.3f})"
                ),
            })

        unclassified_recalculated_mean = solver_update_analysis.get("recalculated_unclassified_factor_count_mean")
        if isinstance(unclassified_recalculated_mean, (int, float)) and float(unclassified_recalculated_mean) > 0.0:
            findings.append({
                "severity": "info",
                "title": "non-odometry factor families contribute to recalculation pressure",
                "evidence": (
                    f"recalculated_unclassified_factor_count_mean={float(unclassified_recalculated_mean):.3f}; "
                    f"corr(recalculated_unclassified_factor_count,solver_update_ms)="
                    f"{solver_update_analysis.get('recalculated_unclassified_factor_vs_solver_update_corr', 'n/a')}"
                ),
            })

        old_segment_corr = solver_update_analysis.get("recalculated_lidar_old_segment_factor_vs_solver_update_corr")
        current_segment_corr = solver_update_analysis.get("recalculated_lidar_current_segment_factor_vs_solver_update_corr")
        if (
            isinstance(old_segment_corr, float)
            and old_segment_corr >= 0.7
            and (
                not isinstance(current_segment_corr, float)
                or old_segment_corr - current_segment_corr >= 0.1
            )
        ):
            findings.append({
                "severity": "warn",
                "title": "solver churn appears old-segment LiDAR dominated",
                "evidence": (
                    f"corr(recalculated_lidar_old_segment_factor_count,solver_update_ms)={old_segment_corr:.3f}; "
                    f"corr(recalculated_lidar_current_segment_factor_count,solver_update_ms)="
                    f"{current_segment_corr if isinstance(current_segment_corr, float) else 'n/a'}"
                ),
            })

        cross_support_corr = solver_update_analysis.get("recalculated_lidar_cross_support_factor_vs_solver_update_corr")
        same_support_corr = solver_update_analysis.get("recalculated_lidar_same_support_factor_vs_solver_update_corr")
        if (
            isinstance(cross_support_corr, float)
            and cross_support_corr >= 0.7
            and (
                not isinstance(same_support_corr, float)
                or cross_support_corr - same_support_corr >= 0.1
            )
        ):
            findings.append({
                "severity": "warn",
                "title": "solver churn appears cross-support LiDAR dominated",
                "evidence": (
                    f"corr(recalculated_lidar_cross_support_factor_count,solver_update_ms)={cross_support_corr:.3f}; "
                    f"corr(recalculated_lidar_same_support_factor_count,solver_update_ms)="
                    f"{same_support_corr if isinstance(same_support_corr, float) else 'n/a'}"
                ),
            })

    if lidar_factor_internal_analysis.get("available"):
        if (
            float(lidar_factor_internal_analysis.get("points_in_bucket_p95", 0.0)) > 5000
            or float(lidar_factor_internal_analysis.get("factor_total_ms_p95", 0.0)) > 20.0
        ):
            findings.append({
                "severity": "warn",
                "title": "single-bucket lidar factor overload is visible",
                "evidence": (
                    f"points_in_bucket_p95={lidar_factor_internal_analysis.get('points_in_bucket_p95', 0.0):.1f}, "
                    f"factor_total_ms_p95={lidar_factor_internal_analysis.get('factor_total_ms_p95', 0.0):.3f}"
                ),
            })

        if "correspondence_share_of_factor_total" in lidar_factor_internal_analysis:
            findings.append({
                "severity": "info",
                "title": "LiDAR factor correspondence cost share captured",
                "evidence": f"correspondence_share={100.0 * lidar_factor_internal_analysis.get('correspondence_share_of_factor_total', 0.0):.1f}%",
            })

    if target_map_analysis.get("available") and target_map_analysis.get("is_black_box"):
        findings.append({
            "severity": "warn",
            "title": "target_map_prep_ms is observable only as a black-box stage",
            "evidence": (
                f"mean={target_map_analysis.get('mean_ms', 0.0):.3f} ms, "
                f"missing={', '.join(target_map_analysis.get('substage_columns_missing', []))}"
            ),
        })

    if pipeline_analysis.get("available") and pipeline_analysis.get("module_stats"):
        top_mod = pipeline_analysis["module_stats"][0]
        findings.append({
            "severity": "info",
            "title": f"Highest-mean pipeline module is {top_mod['module']}",
            "evidence": f"mean={top_mod['mean_ms']:.3f} ms, p95={top_mod['p95_ms']:.3f} ms",
        })

    return findings


def recommend_next_steps(
    artifact_statuses: list[ArtifactStatus],
    mode_consistency: dict[str, Any],
    frontend_analysis: dict[str, Any],
    jump_analysis: dict[str, Any],
    solver_update_analysis: dict[str, Any],
    lidar_factor_internal_analysis: dict[str, Any],
    optimizer_analysis: dict[str, Any],
    slow_frame_analysis: dict[str, Any],
    target_map_analysis: dict[str, Any],
    graph_analysis: dict[str, Any],
    correlation_analysis: dict[str, Any],
    runtime_summary: dict[str, Any],
) -> dict[str, list[str]]:
    recommendations: dict[str, list[str]] = {
        "Immediate next checks": [],
        "Likely code changes": [],
        "Instrumentation gaps to fill": [],
    }

    status_map = {artifact.name: artifact for artifact in artifact_statuses}
    if not frontend_analysis.get("available"):
        recommendations["Immediate next checks"].append(
            "Enable or preserve frontend_frame_profile.csv for frontend-only runs; it provides the strongest per-frame cost breakdown."
        )
    if status_map.get("lidar_factor_profile") and status_map["lidar_factor_profile"].status in {"missing", "expected_missing"}:
        recommendations["Immediate next checks"].append(
            "Capture lidar_factor_profile.csv when investigating SINGLE_BUCKET performance so bucket load can be separated from optimizer cost."
        )
    if status_map.get("solver_update_profile") and status_map["solver_update_profile"].status in {"missing", "expected_missing"}:
        recommendations["Immediate next checks"].append(
            "Enable log.profiling.solver_update_profile when debugging unified BSpline solver bottlenecks."
        )
    if status_map.get("jump_diagnostics") and status_map["jump_diagnostics"].status in {"missing", "expected_missing"}:
        recommendations["Immediate next checks"].append(
            "Enable log.profiling.jump_diagnostics when you need frame-by-frame localization of start/frontend/final pose jumps."
        )
    if status_map.get("lidar_factor_internal_profile") and status_map["lidar_factor_internal_profile"].status in {"missing", "expected_missing"}:
        recommendations["Immediate next checks"].append(
            "Enable log.profiling.lidar_factor_internal_profile when investigating SINGLE_BUCKET factor overload."
        )
    if status_map.get("frontend_lm_iteration") and optimizer_analysis.get("lm_iteration_rows_present") is False:
        recommendations["Instrumentation gaps to fill"].append(
            "Gate optimizer iteration trace behind a dedicated config flag such as log.profiling.frontend_lm_iteration and emit rows whenever LM callback activity occurs."
        )
    if solver_update_analysis.get("available") and solver_update_analysis.get("unavailable_internal_fields"):
        recommendations["Instrumentation gaps to fill"].append(
            "Keep unavailable solver-internal fields explicit. If more detail is needed, extend repo-local BSpline solver telemetry instead of inferring hidden iSAM2 phases."
        )
    if (
        solver_update_analysis.get("available")
        and bool(solver_update_analysis.get("unavailable_internal_timing"))
        and (
            (isinstance(solver_update_analysis.get("reeliminated_variable_vs_solver_update_corr"), float)
             and solver_update_analysis.get("reeliminated_variable_vs_solver_update_corr", 0.0) > 0.7)
            or
            (isinstance(solver_update_analysis.get("recalculated_factor_vs_solver_update_corr"), float)
             and solver_update_analysis.get("recalculated_factor_vs_solver_update_corr", 0.0) > 0.7)
        )
    ):
        recommendations["Immediate next checks"].append(
            "High solver-update correlation with reeliminated/recalculated counts suggests Bayes-tree churn; test smaller smoother_lag and lower active-window coupling before changing math."
        )
    shared_jkg_corr = solver_update_analysis.get("active_window_shared_jkg_touching_factor_vs_solver_update_corr")
    if isinstance(shared_jkg_corr, float) and shared_jkg_corr > 0.7:
        recommendations["Immediate next checks"].append(
            "active_window_shared_jkg_touching_factor_count now tracks solver_update_ms; focus next on reducing active IMU factors or their shared j/k/g sensitivity before spending effort on query-side trimming."
        )
    dominance = jump_analysis.get("dominance")
    if dominance == "jump appears frontend-dominated":
        recommendations["Immediate next checks"].append(
            "Top jump frames already show start->frontend motion first; inspect current-segment LiDAR local layout, support keys, representative/query time, and evaluate_frontend_pose semantics before changing solver math."
        )
    elif dominance == "jump appears solver/boundary-dominated":
        recommendations["Immediate next checks"].append(
            "Top jump frames already show frontend->final motion first; inspect carried prior, oldest survivor, boundary overlap semantics, and LiDAR same-support recalculation churn before touching frontend registration."
        )
    elif dominance == "jump appears mixed; frontend likely first cause":
        recommendations["Immediate next checks"].append(
            "Jump diagnostics show both stages moving; fix start->frontend inconsistencies first, then verify whether solver/boundary still amplifies the corrected frontend pose."
        )
    frontend_cause = jump_analysis.get("frontend_jump_cause")
    if frontend_cause == "frontend jump appears start-pose-semantics dominated":
        recommendations["Immediate next checks"].append(
            "Top start->frontend jump frames now point at start-pose semantics; verify strict-local pre-solve start_pose capture, freeze timing, and the query support used to form start_pose."
        )
    elif frontend_cause == "frontend jump appears query-time mismatch dominated":
        recommendations["Immediate next checks"].append(
            "Top start->frontend jump frames now point at query-time/layout-domain mismatch; compare raw_frame_stamp, start/frontend query times, representative_time, and strict local layout domain on the worst frames."
        )
    elif frontend_cause == "frontend jump appears target/correspondence-quality dominated":
        recommendations["Immediate next checks"].append(
            "Top start->frontend jump frames now point at target/correspondence degradation; inspect accept_ratio, matched/candidate counts, and target point/voxel counts before changing solver logic."
        )
    elif frontend_cause == "frontend jump appears mixed; start-pose and target quality both suspicious":
        recommendations["Immediate next checks"].append(
            "Top start->frontend jump frames now implicate both start-pose semantics and target quality; validate the frozen start_pose path first, then inspect target snapshot/voxel degradation on the same frames."
        )
    frontend_to_final_cause = jump_analysis.get("frontend_to_final_jump_cause")
    if frontend_to_final_cause == "frontend->final jump appears active-window postsolve-surface dominated":
        recommendations["Immediate next checks"].append(
            "Top frontend->final jump frames now implicate the active-window post-solve query surface; compare active-window and strict-local postsolve support on the same frames before changing solver math."
        )
    elif frontend_to_final_cause == "frontend->final jump appears solver-result dominated":
        recommendations["Immediate next checks"].append(
            "Top frontend->final jump frames now still look far from frontend even on strict-local postsolve; inspect solver-side state movement and churn before changing the final query surface."
        )
    elif frontend_to_final_cause == "frontend->final jump appears mixed; strict-local postsolve still far from frontend":
        recommendations["Immediate next checks"].append(
            "Top frontend->final jump frames improve on strict-local postsolve but are still not clean; compare boundary oldest/survivor drift against the remaining strict-local solver movement before choosing between boundary semantics and solver-state fixes."
        )
    final_pose_surface_effect = jump_analysis.get("final_pose_surface_effect")
    if final_pose_surface_effect == "strict-local final pose surface significantly reduces frontend->final jump":
        recommendations["Immediate next checks"].append(
            "Strict-local final pose already suppresses most frontend->final jump; next narrow the fix to final pose query semantics and boundary/survivor alignment before touching solver math."
        )
        recommendations["Immediate next checks"].append(
            "Keep active_window available only as a regression/debug switch when reproducing boundary-shift failures; do not use it as the default final pose surface."
        )
    elif final_pose_surface_effect == "final pose surface switch mitigates jump but solver-result residual remains":
        recommendations["Immediate next checks"].append(
            "Strict-local final pose helps but does not fully clean the run; compare remaining frontend->final residual against boundary oldest/survivor drift before deciding between surface alignment and solver-side cleanup."
        )
    elif final_pose_surface_effect == "strict-local final pose surface does not sufficiently reduce jump":
        recommendations["Immediate next checks"].append(
            "Strict-local final pose did not suppress frontend->final jump enough; shift attention to solver-result motion and boundary carry semantics rather than publish-surface selection alone."
        )
    strict_local_residual_dominance = jump_analysis.get("strict_local_residual_dominance")
    strict_local_rotation_subtype = jump_analysis.get("strict_local_rotation_subtype")
    strict_local_residual_cause = jump_analysis.get("strict_local_residual_cause")
    if strict_local_residual_cause == "strict-local residual more likely reflects solver-side orientation drift":
        recommendations["Immediate next checks"].append(
            "Strict-local residual now looks solver-side and orientation-led; inspect IMU / prior / velocity / shared-state updates on the top residual frames before changing query surfaces again."
        )
    elif strict_local_residual_cause == "strict-local residual likely includes query-side orientation inconsistency":
        recommendations["Immediate next checks"].append(
            "Strict-local residual still shows query-side inconsistency; compare strict-local support, fallback paths, and orientation extraction semantics on the top residual frames before touching solver math."
        )
    elif strict_local_residual_cause == "strict-local residual more likely reflects translation-side factor consistency":
        recommendations["Immediate next checks"].append(
            "Strict-local residual is now more translation-led; inspect LiDAR-side factor consistency and frame-to-frame translation corrections before changing IMU-side states."
        )
    elif strict_local_residual_cause == "strict-local residual appears mixed; IMU/prior and LiDAR both suspicious":
        recommendations["Immediate next checks"].append(
            "Strict-local residual still looks mixed; split the next audit between IMU/prior/shared-state churn and LiDAR translation consistency on the same top residual frames."
        )
    elif strict_local_residual_dominance == "strict-local residual appears rotation-dominated":
        recommendations["Immediate next checks"].append(
            "Strict-local residual is now rotation-led; prioritize orientation-side telemetry and state updates over translation-only tuning."
        )
    elif strict_local_residual_dominance == "strict-local residual appears translation-dominated":
        recommendations["Immediate next checks"].append(
            "Strict-local residual is now translation-led; prioritize LiDAR/translation consistency and vertical-vs-planar drift shape before revisiting orientation semantics."
        )
    if strict_local_rotation_subtype == "strict-local residual appears yaw-dominated":
        recommendations["Likely code changes"].append(
            "If the next audit confirms solver-side yaw drift with stable strict-local support, narrow the fix to IMU/prior/shared-state yaw handling before considering extrinsics."
        )
    elif strict_local_rotation_subtype == "strict-local residual appears pitch/roll-dominated":
        recommendations["Likely code changes"].append(
            "If the next audit confirms pitch/roll-led residual with weak solver correlations, check strict-local orientation semantics and LiDAR-IMU frame convention before changing LiDAR factors."
        )
    yaw_root_cause_leader = jump_analysis.get("yaw_root_cause_leader")
    if yaw_root_cause_leader == "A":
        recommendations["Immediate next checks"].append(
            "Yaw shootout now ranks velocity/prior/shared-state coupling highest; the next minimum fix should focus on yaw handling around velocity, prior, and shared-state updates before revisiting frame semantics."
        )
    elif yaw_root_cause_leader == "B":
        recommendations["Immediate next checks"].append(
            "Yaw shootout now ranks orientation semantics highest; the next minimum fix should focus on world->lidar/world->imu yaw interpretation and extrinsic/frame-convention handling before touching solver weights."
        )
    elif yaw_root_cause_leader == "C":
        recommendations["Immediate next checks"].append(
            "Yaw shootout now ranks IMU/bias/gravity highest; the next minimum fix should focus on gyro/accel bias and gravity handling before revisiting velocity/prior coupling."
        )
    new_factor_corr = solver_update_analysis.get("new_factor_vs_active_window_corr")
    new_value_corr = solver_update_analysis.get("new_value_vs_active_window_corr")
    if (
        isinstance(new_factor_corr, float) and new_factor_corr > 0.85
    ) or (
        isinstance(new_value_corr, float) and new_value_corr > 0.85
    ):
        recommendations["Likely code changes"].append(
            "Re-check delta assembly so new_factor_count/new_value_count stay tied to newly appended segment(s), not active-window size."
        )
    if (
        lidar_factor_internal_analysis.get("available")
        and float(lidar_factor_internal_analysis.get("correspondence_share_of_factor_total", 0.0)) > 0.6
    ):
        recommendations["Immediate next checks"].append(
            "Correspondence dominates LiDAR factor cost; inspect candidate count, accept ratio, and target density before changing solver settings."
        )
    if slow_frame_analysis.get("warning_count_available") is False:
        recommendations["Instrumentation gaps to fill"].append(
            "Add config-gated per-frame warning telemetry, for example profiling/frame_warning_profile.csv behind log.profiling.frame_warning_profile, so slow-frame diagnostics can correlate warning density with timing outliers."
        )
    if runtime_summary.get("issues"):
        recommendations["Immediate next checks"].append(
            "Review runtime warning/error patterns before tuning performance; correctness and data availability issues often dominate timing outliers."
        )
    if mode_consistency.get("loaded_modules_that_contradict_mode"):
        recommendations["Likely code changes"].append(
            "Clean frontend-only runtime mode so mapping/backend modules are not loaded when they are not expected to participate."
        )
    if target_map_analysis.get("available") and target_map_analysis.get("is_black_box"):
        recommendations["Instrumentation gaps to fill"].append(
            "Split target_map_prep_ms into config-gated substage columns, for example target_snapshot_clone_ms, target_voxel_lookup_prep_ms, target_covariance_prep_ms, and source_to_target_transform_ms."
        )
    missing_graph = graph_analysis.get("missing_graph_size_telemetry", [])
    if missing_graph:
        recommendations["Instrumentation gaps to fill"].append(
            "Extend frontend_frame_profile.csv with config-gated graph/problem-size columns, for example under log.profiling.graph_problem_size, covering "
            + ", ".join(missing_graph)
            + "."
        )
    if correlation_analysis.get("available") and all(
        value in {"missing_field", "insufficient_samples", "constant_series"}
        for value in correlation_analysis.get("correlations", {}).values()
    ):
        recommendations["Immediate next checks"].append(
            "Correlation analysis was inconclusive for this run; compare against another run or enable more profiling before drawing optimization conclusions."
        )
    if not mode_consistency.get("frontend_only_mode_effective") and status_map.get("backend_summary") and status_map["backend_summary"].status in {"missing", "expected_missing"}:
        recommendations["Instrumentation gaps to fill"].append(
            "If backend-active runs should remain diagnosable, add a config-gated backend summary export such as export/backend_summary.csv behind log.export.backend_summary."
        )
    recommendations["Instrumentation gaps to fill"].append(
        "Keep every new high-volume telemetry output behind an explicit config flag. Prefer log.profiling.* for per-frame or per-iteration CSVs and log.export.* only for user-facing result exports."
    )

    if not any(recommendations.values()):
        recommendations["Immediate next checks"].append(
            "The run contains enough profiling to support first-pass diagnosis; compare this run against another bag or config next."
        )
    return recommendations


def md_table(headers: list[str], rows: list[list[Any]]) -> str:
    if not rows:
        return "_none_\n"
    header_line = "| " + " | ".join(headers) + " |"
    sep_line = "| " + " | ".join(["---"] * len(headers)) + " |"
    body_lines = ["| " + " | ".join(str(cell) for cell in row) + " |" for row in rows]
    return "\n".join([header_line, sep_line, *body_lines]) + "\n"


def render_report_markdown(
    run_dir: Path,
    metadata: dict[str, Any],
    config_summary: dict[str, Any],
    artifact_statuses: list[ArtifactStatus],
    runtime_summary: dict[str, Any],
    mode_consistency: dict[str, Any],
    frontend_analysis: dict[str, Any],
    jump_analysis: dict[str, Any],
    solver_update_analysis: dict[str, Any],
    lidar_factor_internal_analysis: dict[str, Any],
    slow_frame_analysis: dict[str, Any],
    optimizer_analysis: dict[str, Any],
    target_map_analysis: dict[str, Any],
    graph_analysis: dict[str, Any],
    bucket_analysis: dict[str, Any],
    bucket_diagnostics: dict[str, Any],
    correlation_analysis: dict[str, Any],
    pipeline_analysis: dict[str, Any],
    seed_compare_analysis: dict[str, Any],
    findings: list[dict[str, str]],
    recommendations: dict[str, list[str]],
    external_results: list[dict[str, Any]],
) -> str:
    lines: list[str] = []
    run_info = metadata.get("run_info", {})
    lines.append("# IAP Run Analysis")
    lines.append("")
    lines.append("## Run Summary")
    lines.append("")
    lines.append(md_table(
        ["Key", "Value"],
        [
            ["run_dir", run_dir],
            ["timestamp", run_info.get("timestamp", "")],
            ["package_root", run_info.get("package_root", "")],
            ["git_rev", metadata.get("git_rev", "")],
            ["config_final_pose_surface", run_info.get("config_final_pose_surface", "")],
            ["runtime_final_pose_surface", run_info.get("runtime_final_pose_surface", "")],
            ["build_info", metadata.get("build_info", "").replace("\n", "; ")],
        ],
    ))

    lines.append("## Config Summary")
    lines.append("")
    config_rows = [[k, v] for k, v in config_summary.items()]
    lines.append(md_table(["Key", "Value"], config_rows))

    lines.append("## Structural Mode Summary")
    lines.append("")
    lines.append(md_table(
        ["Key", "Value"],
        [
            ["runtime_gravity_state_mode", config_summary.get("runtime_gravity_state_mode", "n/a")],
            ["runtime_gravity_reference_source", config_summary.get("runtime_gravity_reference_source", "n/a")],
            ["runtime_gravity_reference_vector", config_summary.get("runtime_gravity_reference_vector", "n/a")],
            ["runtime_bias_state_mode", config_summary.get("runtime_bias_state_mode", "n/a")],
            ["runtime_bias_optimized", config_summary.get("runtime_bias_optimized", "n/a")],
            ["runtime_bias_source_of_truth", config_summary.get("runtime_bias_source_of_truth", "n/a")],
            ["runtime_bias_transition_prior_enabled", config_summary.get("runtime_bias_transition_prior_enabled", "n/a")],
            ["runtime_bias_transition_prior_strength", config_summary.get("runtime_bias_transition_prior_strength", "n/a")],
            ["runtime_bias_can_be_survivor_anchor", config_summary.get("runtime_bias_can_be_survivor_anchor", "n/a")],
            ["runtime_bias_writeback_mode", config_summary.get("runtime_bias_writeback_mode", "n/a")],
            ["runtime_frontend_seed_mode", config_summary.get("runtime_frontend_seed_mode", "n/a")],
            ["runtime_imu_forward_prediction_enabled", config_summary.get("runtime_imu_forward_prediction_enabled", "n/a")],
            ["runtime_frontend_seed_fallback_used", config_summary.get("runtime_frontend_seed_fallback_used", "n/a")],
            ["runtime_frontend_seed_source", config_summary.get("runtime_frontend_seed_source", "n/a")],
            ["runtime_frontend_seed_imu_sample_count", config_summary.get("runtime_frontend_seed_imu_sample_count", "n/a")],
            ["runtime_velocity_state_mode", config_summary.get("runtime_velocity_state_mode", "n/a")],
            ["runtime_velocity_mode_policy", config_summary.get("runtime_velocity_mode_policy", "n/a")],
            ["runtime_velocity_optimized", config_summary.get("runtime_velocity_optimized", "n/a")],
            ["runtime_has_gnss_constraints", config_summary.get("runtime_has_gnss_constraints", "n/a")],
        ],
    ))

    if seed_compare_analysis.get("available"):
        lines.append("## Seed Improvement Comparison")
        lines.append("")
        lines.append(md_table(
            ["Key", "Value"],
            [
                ["baseline_run_dir", seed_compare_analysis.get("baseline_run_dir", "n/a")],
                ["baseline_frontend_seed_mode", seed_compare_analysis.get("baseline_frontend_seed_mode", "n/a")],
                ["current_frontend_seed_mode", seed_compare_analysis.get("current_frontend_seed_mode", "n/a")],
                ["summary", seed_compare_analysis.get("summary", "n/a")],
            ],
        ))
        metric_rows = []
        for label, baseline_value, current_value, delta_value in seed_compare_analysis.get("metric_rows", []):
            metric_rows.append([
                label,
                baseline_value if baseline_value is not None else "n/a",
                current_value if current_value is not None else "n/a",
                delta_value if delta_value is not None else "n/a",
            ])
        lines.append("")
        lines.append(md_table(["Metric", "Baseline", "Current", "Delta"], metric_rows))

    lines.append("## Artifact Coverage")
    lines.append("")
    art_rows = []
    for artifact in artifact_statuses:
        size_kb = f"{artifact.size_bytes / 1024.0:.1f}" if artifact.size_bytes else "0.0"
        rows = "" if artifact.rows is None else artifact.rows
        art_rows.append([artifact.name, artifact.status, artifact.reason, rows, size_kb, artifact.rel_path, artifact.note])
    lines.append(md_table(["Artifact", "Status", "Reason", "Rows", "Size KB", "Path", "Note"], art_rows))

    lines.append("## Mode Consistency Check")
    lines.append("")
    lines.append(md_table(
        ["Key", "Value"],
        [
            ["frontend_only_mode_config", mode_consistency.get("frontend_only_mode_config")],
            ["frontend_only_mode_runtime", mode_consistency.get("frontend_only_mode_runtime")],
            ["frontend_only_mode_effective", mode_consistency.get("frontend_only_mode_effective")],
            ["backend_expected_active", mode_consistency.get("backend_expected_active")],
            ["backend_observed_active", mode_consistency.get("backend_observed_active")],
            ["mapping_expected_active", mode_consistency.get("mapping_expected_active")],
            ["mapping_observed_active", mode_consistency.get("mapping_observed_active")],
            ["gnss_expected_active", mode_consistency.get("gnss_expected_active")],
            ["gnss_observed_active", mode_consistency.get("gnss_observed_active")],
        ],
    ))
    lines.append("Mode-sensitive details:")
    lines.append("")
    lines.append(md_table(
        ["Field", "Value"],
        [
            ["loaded_modules_that_contradict_mode", ", ".join(mode_consistency.get("loaded_modules_that_contradict_mode", [])) or "_none_"],
            ["unexpected_runtime_modules", ", ".join(mode_consistency.get("unexpected_runtime_modules", [])) or "_none_"],
            ["expected_empty_artifacts", ", ".join(mode_consistency.get("expected_empty_artifacts", [])) or "_none_"],
            ["unexpected_empty_artifacts", ", ".join(mode_consistency.get("unexpected_empty_artifacts", [])) or "_none_"],
            ["should_not_exist_artifacts_but_found", ", ".join(mode_consistency.get("should_not_exist_artifacts_but_found", [])) or "_none_"],
            ["warnings_inconsistent_with_mode", len(mode_consistency.get("warnings_inconsistent_with_mode", []))],
        ],
    ))

    lines.append("## Pipeline Health")
    lines.append("")
    pipeline_health_rows = [
        ["loaded_modules", len(runtime_summary.get("loaded_modules", []))],
        ["runtime_log_files", len(runtime_summary.get("files", []))],
        ["runtime_total_lines", runtime_summary.get("total_lines", 0)],
        ["frontend_frame_count", frontend_analysis.get("frame_count", 0)],
        ["jump_rows", jump_analysis.get("jump_rows", 0)],
        ["pipeline_timing_rows", pipeline_analysis.get("row_count", 0)],
        ["waiting_messages", len(runtime_summary.get("waiting_messages", []))],
    ]
    lines.append(md_table(["Key", "Value"], pipeline_health_rows))

    if runtime_summary.get("loaded_modules"):
        lines.append("Loaded modules:")
        for item in runtime_summary["loaded_modules"]:
            lines.append(f"- `{item}`")
        lines.append("")

    if runtime_summary.get("waiting_messages"):
        lines.append("Observed waiting messages:")
        for item in runtime_summary["waiting_messages"][:10]:
            lines.append(f"- `{item}`")
        lines.append("")

    lines.append("## Errors And Warnings")
    lines.append("")
    issue_rows = [
        [issue["level"], issue["file"], issue["line"], issue["message"]]
        for issue in runtime_summary.get("issues", [])[:20]
    ]
    lines.append(md_table(["Level", "File", "Line", "Message"], issue_rows))

    pattern_rows = [[count, pattern] for pattern, count in runtime_summary.get("top_patterns", [])]
    if pattern_rows:
        lines.append("Top repeated issue patterns:")
        lines.append("")
        lines.append(md_table(["Count", "Pattern"], pattern_rows))

    lines.append("## Timing Analysis")
    lines.append("")
    if frontend_analysis.get("available"):
        lines.append("Frontend frame profile summary:")
        lines.append("")
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["frame_count", frontend_analysis.get("frame_count", 0)],
                ["total_mean_ms", f"{frontend_analysis.get('total_mean_ms', 0.0):.3f}"],
                ["total_p95_ms", f"{frontend_analysis.get('total_p95_ms', 0.0):.3f}"],
                ["total_p99_ms", f"{frontend_analysis.get('total_p99_ms', 0.0):.3f}"],
                ["total_max_ms", f"{frontend_analysis.get('total_max_ms', 0.0):.3f}"],
                ["bucket_mode", ", ".join(frontend_analysis.get("bucket_mode", []))],
            ],
        ))
        stage_rows = [
            [
                item["stage"],
                f"{item['mean_ms']:.3f}",
                f"{item['p95_ms']:.3f}",
                f"{item['max_ms']:.3f}",
                f"{100.0 * item['mean_share']:.1f}%",
            ]
            for item in frontend_analysis.get("stage_summary", [])
        ]
        lines.append("Frontend stage breakdown:")
        lines.append("")
        lines.append(md_table(["Stage", "Mean ms", "P95 ms", "Max ms", "Mean Share"], stage_rows))
    else:
        lines.append("Frontend frame profiling was not available for this run.")
        lines.append("")

    if bucket_analysis.get("available"):
        lines.append("LiDAR bucket profile summary:")
        lines.append("")
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["bucket_rows", bucket_analysis.get("bucket_rows", 0)],
                ["points_in_bucket_mean", f"{bucket_analysis.get('points_in_bucket_mean', 0.0):.1f}"],
                ["points_in_bucket_p95", f"{bucket_analysis.get('points_in_bucket_p95', 0.0):.1f}"],
                ["factor_total_mean_ms", f"{bucket_analysis.get('factor_total_mean_ms', 0.0):.3f}"],
                ["factor_total_p95_ms", f"{bucket_analysis.get('factor_total_p95_ms', 0.0):.3f}"],
                ["match_ratio_mean", f"{bucket_analysis.get('match_ratio_mean', 0.0):.3f}"],
                ["inlier_ratio_mean", f"{bucket_analysis.get('inlier_ratio_mean', 0.0):.3f}"],
            ],
        ))
    else:
        lines.append("LiDAR bucket profiling was not available for this run.")
        lines.append("")

    lines.append("## Jump Diagnostics")
    lines.append("")
    if jump_analysis.get("available"):
        source_counts = jump_analysis.get("start_pose_source_kind_counts", {})
        source_counts_text = ", ".join(f"{k}:{v}" for k, v in source_counts.items()) or "_none_"
        frontend_seed_counts = jump_analysis.get("frontend_seed_source_counts", {})
        frontend_seed_counts_text = ", ".join(f"{k}:{v}" for k, v in frontend_seed_counts.items()) or "_none_"
        mismatch_counts = jump_analysis.get("start_pose_support_mismatch_reason_counts", {})
        mismatch_counts_text = ", ".join(f"{k}:{v}" for k, v in mismatch_counts.items()) or "_none_"
        strict_local_query_counts = jump_analysis.get("strict_local_query_reason_counts", {})
        strict_local_query_counts_text = ", ".join(f"{k}:{v}" for k, v in strict_local_query_counts.items()) or "_none_"
        frozen_before_factor = jump_analysis.get("start_pose_frozen_before_factor_injection_counts", {})
        frozen_before_solver = jump_analysis.get("start_pose_frozen_before_solver_update_counts", {})
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["jump_rows", jump_analysis.get("jump_rows", 0)],
                ["dominance", jump_analysis.get("dominance", "n/a")],
                ["frontend_jump_cause", jump_analysis.get("frontend_jump_cause", "n/a")],
                ["start_to_frontend_translation_mean", f"{jump_analysis.get('start_to_frontend_translation_mean', 0.0):.3f}"],
                ["start_to_frontend_translation_p95", f"{jump_analysis.get('start_to_frontend_translation_p95', 0.0):.3f}"],
                ["start_to_frontend_translation_max", f"{jump_analysis.get('start_to_frontend_translation_max', 0.0):.3f}"],
                ["start_to_frontend_rotation_mean", f"{jump_analysis.get('start_to_frontend_rotation_mean', 0.0):.3f}"],
                ["start_to_frontend_rotation_p95", f"{jump_analysis.get('start_to_frontend_rotation_p95', 0.0):.3f}"],
                ["start_to_frontend_rotation_max", f"{jump_analysis.get('start_to_frontend_rotation_max', 0.0):.3f}"],
                ["frontend_to_final_translation_mean", f"{jump_analysis.get('frontend_to_final_translation_mean', 0.0):.3f}"],
                ["frontend_to_final_translation_p95", f"{jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f}"],
                ["frontend_to_final_translation_max", f"{jump_analysis.get('frontend_to_final_translation_max', 0.0):.3f}"],
                ["frontend_to_final_rotation_mean", f"{jump_analysis.get('frontend_to_final_rotation_mean', 0.0):.3f}"],
                ["frontend_to_final_rotation_p95", f"{jump_analysis.get('frontend_to_final_rotation_p95', 0.0):.3f}"],
                ["frontend_to_final_rotation_max", f"{jump_analysis.get('frontend_to_final_rotation_max', 0.0):.3f}"],
                ["frontend_to_final_jump_cause", jump_analysis.get("frontend_to_final_jump_cause", "n/a")],
                ["strict_local_residual_dominance", jump_analysis.get("strict_local_residual_dominance", "n/a")],
                ["strict_local_residual_cause", jump_analysis.get("strict_local_residual_cause", "n/a")],
            ],
        ))
        lines.append("Frontend jump consistency summary:")
        lines.append("")
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["start_pose_source_kind_counts", source_counts_text],
                ["frontend_seed_source_counts", frontend_seed_counts_text],
                ["seed_fallback_frame_count", jump_analysis.get("seed_fallback_frame_count", 0)],
                ["seed_fallback_frame_ratio", f"{jump_analysis.get('seed_fallback_frame_ratio', 0.0):.3f}"],
                [
                    "frontend_seed_imu_sample_count (mean/p95/max)",
                    f"{jump_analysis.get('frontend_seed_imu_sample_count_mean', 0.0):.1f} / "
                    f"{jump_analysis.get('frontend_seed_imu_sample_count_p95', 0.0):.1f} / "
                    f"{jump_analysis.get('frontend_seed_imu_sample_count_max', 0.0):.1f}",
                ],
                [
                    "start_pose_frozen_before_factor_injection_counts",
                    f"true:{frozen_before_factor.get('true', 0)}, false:{frozen_before_factor.get('false', 0)}",
                ],
                [
                    "start_pose_frozen_before_solver_update_counts",
                    f"true:{frozen_before_solver.get('true', 0)}, false:{frozen_before_solver.get('false', 0)}",
                ],
                ["start_pose_support_mismatch_reason_counts", mismatch_counts_text],
                [
                    "start_pose_query_time - representative_time (mean/p95/max)",
                    f"{jump_analysis.get('start_pose_query_time_minus_representative_time_mean', 0.0):.6f} / "
                    f"{jump_analysis.get('start_pose_query_time_minus_representative_time_p95', 0.0):.6f} / "
                    f"{jump_analysis.get('start_pose_query_time_minus_representative_time_max', 0.0):.6f}",
                ],
                [
                    "frontend_pose_query_time - representative_time (mean/p95/max)",
                    f"{jump_analysis.get('frontend_pose_query_time_minus_representative_time_mean', 0.0):.6f} / "
                    f"{jump_analysis.get('frontend_pose_query_time_minus_representative_time_p95', 0.0):.6f} / "
                    f"{jump_analysis.get('frontend_pose_query_time_minus_representative_time_max', 0.0):.6f}",
                ],
                ["strict_local_query_reason_counts", strict_local_query_counts_text],
                [
                    "start_pose_query_time - frontend_pose_query_time (mean/p95/max)",
                    f"{jump_analysis.get('start_pose_query_time_minus_frontend_pose_query_time_mean', 0.0):.6f} / "
                    f"{jump_analysis.get('start_pose_query_time_minus_frontend_pose_query_time_p95', 0.0):.6f} / "
                    f"{jump_analysis.get('start_pose_query_time_minus_frontend_pose_query_time_max', 0.0):.6f}",
                ],
            ],
        ))
        lines.append("Postsolve surface comparison:")
        lines.append("")
        lines.append(md_table(
            ["Metric", "Value"],
            [
                [
                    "frontend->postsolve_active_window translation (mean/p95/max)",
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_translation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_translation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_translation_max', 0.0):.3f}",
                ],
                [
                    "frontend->postsolve_active_window rotation (mean/p95/max)",
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_rotation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_rotation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_active_window_rotation_max', 0.0):.3f}",
                ],
                [
                    "frontend->postsolve_strict_local translation (mean/p95/max)",
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_translation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_translation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_translation_max', 0.0):.3f}",
                ],
                [
                    "frontend->postsolve_strict_local rotation (mean/p95/max)",
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_rotation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_rotation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('frontend_to_postsolve_strict_local_rotation_max', 0.0):.3f}",
                ],
                [
                    "postsolve_active_window->postsolve_strict_local translation (mean/p95/max)",
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_max', 0.0):.3f}",
                ],
                [
                    "postsolve_active_window->postsolve_strict_local rotation (mean/p95/max)",
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_max', 0.0):.3f}",
                ],
                [
                    "postsolve_active_window->final translation (mean/p95/max)",
                    f"{jump_analysis.get('postsolve_active_window_to_final_translation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_final_translation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_final_translation_max', 0.0):.3f}",
                ],
                [
                    "postsolve_active_window->final rotation (mean/p95/max)",
                    f"{jump_analysis.get('postsolve_active_window_to_final_rotation_mean', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_final_rotation_p95', 0.0):.3f} / "
                    f"{jump_analysis.get('postsolve_active_window_to_final_rotation_max', 0.0):.3f}",
                ],
            ],
        ))
        if jump_analysis.get("runtime_final_pose_surface") == "strict_local":
            lines.append("Strict-local final pose experiment:")
            lines.append("")
            lines.append(md_table(
                ["Metric", "Value"],
                [
                    ["runtime_final_pose_surface", jump_analysis.get("runtime_final_pose_surface", "")],
                    ["surface_policy", "strict_local intended default; active_window retained for debug/regression only"],
                    ["final_pose_surface_effect", jump_analysis.get("final_pose_surface_effect", "n/a")],
                    [
                        "frontend->final translation (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_translation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_translation_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final rotation (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_rotation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_rotation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_rotation_max', 0.0):.3f}",
                    ],
                    [
                        "postsolve_active_window->strict_local translation (mean/p95/max)",
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_translation_max', 0.0):.3f}",
                    ],
                    [
                        "postsolve_active_window->strict_local rotation (mean/p95/max)",
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('postsolve_active_window_to_postsolve_strict_local_rotation_max', 0.0):.3f}",
                    ],
                ],
            ))
            lines.append("Strict-local residual diagnostics:")
            lines.append("")
            lines.append(md_table(
                ["Metric", "Value"],
                [
                    ["runtime_experiment_name", jump_analysis.get("runtime_experiment_name", "baseline")],
                    ["runtime_gravity_mode", jump_analysis.get("runtime_gravity_mode", "n/a")],
                    ["runtime_gravity_fixed_norm_value", jump_analysis.get("runtime_gravity_fixed_norm_value", "n/a")],
                    ["runtime_gravity_tilt_limit_rad", jump_analysis.get("runtime_gravity_tilt_limit_rad", "n/a")],
                    ["runtime_gravity_warmup_freeze_frames", jump_analysis.get("runtime_gravity_warmup_freeze_frames", "n/a")],
                    ["runtime_exp_freeze_gravity", jump_analysis.get("runtime_exp_freeze_gravity", "n/a")],
                    ["runtime_exp_freeze_gyro_bias", jump_analysis.get("runtime_exp_freeze_gyro_bias", "n/a")],
                    ["runtime_exp_freeze_accel_bias", jump_analysis.get("runtime_exp_freeze_accel_bias", "n/a")],
                    ["runtime_exp_disable_velocity_factor", jump_analysis.get("runtime_exp_disable_velocity_factor", "n/a")],
                    ["runtime_exp_disable_current_velocity_prior", jump_analysis.get("runtime_exp_disable_current_velocity_prior", "n/a")],
                    ["strict_local_residual_dominance", jump_analysis.get("strict_local_residual_dominance", "n/a")],
                    ["pitch_vs_roll_dominance", jump_analysis.get("pitch_vs_roll_dominance", "n/a")],
                    ["strict_local_rotation_subtype", jump_analysis.get("strict_local_rotation_subtype", "n/a")],
                    ["strict_local_residual_cause", jump_analysis.get("strict_local_residual_cause", "n/a")],
                    ["pitch_roll_root_cause_summary", jump_analysis.get("pitch_roll_root_cause_summary", "n/a")],
                    [
                        "frontend->final translation (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_translation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_translation_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final rotation (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_rotation_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_rotation_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_rotation_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final yaw residual abs (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_yaw_abs_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_yaw_abs_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_yaw_abs_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final pitch residual abs (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_pitch_abs_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_pitch_abs_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_pitch_abs_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final roll residual abs (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_roll_abs_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_roll_abs_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_roll_abs_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final dx (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_dx_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dx_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dx_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final dy (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_dy_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dy_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dy_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final dz (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_dz_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dz_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_dz_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final xy_norm (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_xy_norm_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_xy_norm_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_xy_norm_max', 0.0):.3f}",
                    ],
                    [
                        "frontend->final abs_dz (mean/p95/max)",
                        f"{jump_analysis.get('frontend_to_final_abs_dz_abs_mean', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_abs_dz_abs_p95', 0.0):.3f} / "
                        f"{jump_analysis.get('frontend_to_final_abs_dz_abs_max', 0.0):.3f}",
                    ],
                    ["strict_local_query_reason_counts", strict_local_query_counts_text],
                    ["rotation_primary_corr", jump_analysis.get("strict_local_rotation_primary_correlation", "n/a")],
                    ["translation_primary_corr", jump_analysis.get("strict_local_translation_primary_correlation", "n/a")],
                    ["delta_rotation_vs_solver_update_ms", f"{jump_analysis.get('delta_rotation_vs_solver_update_ms', 'n/a')}"],
                    ["delta_rotation_vs_recalc_imu", f"{jump_analysis.get('delta_rotation_vs_recalc_imu', 'n/a')}"],
                    ["delta_rotation_vs_recalc_prior", f"{jump_analysis.get('delta_rotation_vs_recalc_prior', 'n/a')}"],
                    ["delta_rotation_vs_recalc_velocity", f"{jump_analysis.get('delta_rotation_vs_recalc_velocity', 'n/a')}"],
                    ["delta_rotation_vs_relin_shared", f"{jump_analysis.get('delta_rotation_vs_relin_shared', 'n/a')}"],
                    ["delta_translation_vs_solver_update_ms", f"{jump_analysis.get('delta_translation_vs_solver_update_ms', 'n/a')}"],
                    ["delta_translation_vs_recalc_lidar", f"{jump_analysis.get('delta_translation_vs_recalc_lidar', 'n/a')}"],
                    ["delta_z_vs_recalc_imu", f"{jump_analysis.get('delta_z_vs_recalc_imu', 'n/a')}"],
                    ["delta_yaw_vs_recalc_imu", f"{jump_analysis.get('delta_yaw_vs_recalc_imu', 'n/a')}"],
                    ["delta_yaw_vs_recalc_prior", f"{jump_analysis.get('delta_yaw_vs_recalc_prior', 'n/a')}"],
                    ["delta_yaw_vs_relin_shared", f"{jump_analysis.get('delta_yaw_vs_relin_shared', 'n/a')}"],
                    ["delta_yaw_vs_recalc_velocity", f"{jump_analysis.get('delta_yaw_vs_recalc_velocity', 'n/a')}"],
                    ["delta_yaw_vs_current_velocity_norm", f"{jump_analysis.get('delta_yaw_vs_current_velocity_norm', 'n/a')}"],
                    ["delta_yaw_vs_delta_velocity_heading", f"{jump_analysis.get('delta_yaw_vs_delta_velocity_heading', 'n/a')}"],
                    ["delta_yaw_vs_world_to_lidar_yaw_shift", f"{jump_analysis.get('delta_yaw_vs_world_to_lidar_yaw_shift', 'n/a')}"],
                    ["delta_yaw_vs_world_to_imu_yaw_shift", f"{jump_analysis.get('delta_yaw_vs_world_to_imu_yaw_shift', 'n/a')}"],
                    ["delta_yaw_vs_extrinsic_yaw_probe", f"{jump_analysis.get('delta_yaw_vs_extrinsic_yaw_probe', 'n/a')}"],
                    ["delta_yaw_vs_gyro_bias_norm", f"{jump_analysis.get('delta_yaw_vs_gyro_bias_norm', 'n/a')}"],
                    ["delta_yaw_vs_accel_bias_norm", f"{jump_analysis.get('delta_yaw_vs_accel_bias_norm', 'n/a')}"],
                    ["delta_yaw_vs_gravity_dir_tilt", f"{jump_analysis.get('delta_yaw_vs_gravity_dir_tilt', 'n/a')}"],
                    ["delta_yaw_vs_delta_gravity_dir", f"{jump_analysis.get('delta_yaw_vs_delta_gravity_dir', 'n/a')}"],
                ],
            ))
            lines.append("Gravity Experiment Summary:")
            lines.append("")
            lines.append(md_table(
                ["Metric", "Value"],
                [
                    ["runtime_experiment_name", jump_analysis.get("runtime_experiment_name", "baseline")],
                    ["runtime_final_pose_surface", jump_analysis.get("runtime_final_pose_surface", "n/a")],
                    ["runtime_gravity_mode", jump_analysis.get("runtime_gravity_mode", "n/a")],
                    ["runtime_gravity_fixed_norm_value", jump_analysis.get("runtime_gravity_fixed_norm_value", "n/a")],
                    ["runtime_gravity_tilt_limit_rad", jump_analysis.get("runtime_gravity_tilt_limit_rad", "n/a")],
                    ["runtime_gravity_warmup_freeze_frames", jump_analysis.get("runtime_gravity_warmup_freeze_frames", "n/a")],
                    ["frontend_frame_count", frontend_analysis.get("frame_count", 0)],
                    ["jump_rows", jump_analysis.get("jump_rows", 0)],
                    ["yaw_p95", f"{jump_analysis.get('frontend_to_final_yaw_abs_p95', 0.0):.3f}"],
                    ["yaw_max", f"{jump_analysis.get('frontend_to_final_yaw_abs_max', 0.0):.3f}"],
                    ["rotation_p95", f"{jump_analysis.get('frontend_to_final_rotation_p95', 0.0):.3f}"],
                    ["translation_p95", f"{jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f}"],
                    ["xy_norm_p95", f"{jump_analysis.get('frontend_to_final_xy_norm_p95', 0.0):.3f}"],
                    ["abs_dz_p95", f"{jump_analysis.get('frontend_to_final_abs_dz_abs_p95', 0.0):.3f}"],
                    ["solver_update_ms_mean", f"{jump_analysis.get('solver_update_ms_mean', 0.0):.3f}" if isinstance(jump_analysis.get('solver_update_ms_mean'), float) and math.isfinite(jump_analysis.get('solver_update_ms_mean')) else "n/a"],
                    ["accept_ratio_mean", f"{jump_analysis.get('accept_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('accept_ratio_mean'), float) and math.isfinite(jump_analysis.get('accept_ratio_mean')) else "n/a"],
                    ["match_ratio_mean", f"{jump_analysis.get('match_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('match_ratio_mean'), float) and math.isfinite(jump_analysis.get('match_ratio_mean')) else "n/a"],
                    ["inlier_ratio_mean", f"{jump_analysis.get('inlier_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('inlier_ratio_mean'), float) and math.isfinite(jump_analysis.get('inlier_ratio_mean')) else "n/a"],
                    ["isolation_effect", jump_analysis.get("isolation_effect", "n/a")],
                ],
            ))
            lines.append("Isolation Experiment Summary:")
            lines.append("")
            lines.append(md_table(
                ["Metric", "Value"],
                [
                    ["experiment_name", jump_analysis.get("runtime_experiment_name", "baseline")],
                    ["runtime_final_pose_surface", jump_analysis.get("runtime_final_pose_surface", "n/a")],
                    ["freeze_gravity", jump_analysis.get("runtime_exp_freeze_gravity", "n/a")],
                    ["freeze_gyro_bias", jump_analysis.get("runtime_exp_freeze_gyro_bias", "n/a")],
                    ["freeze_accel_bias", jump_analysis.get("runtime_exp_freeze_accel_bias", "n/a")],
                    ["disable_velocity_factor", jump_analysis.get("runtime_exp_disable_velocity_factor", "n/a")],
                    ["disable_current_velocity_prior", jump_analysis.get("runtime_exp_disable_current_velocity_prior", "n/a")],
                    ["yaw_p95", f"{jump_analysis.get('frontend_to_final_yaw_abs_p95', 0.0):.3f}"],
                    ["yaw_max", f"{jump_analysis.get('frontend_to_final_yaw_abs_max', 0.0):.3f}"],
                    ["rotation_p95", f"{jump_analysis.get('frontend_to_final_rotation_p95', 0.0):.3f}"],
                    ["translation_p95", f"{jump_analysis.get('frontend_to_final_translation_p95', 0.0):.3f}"],
                    ["solver_update_ms_mean", f"{jump_analysis.get('solver_update_ms_mean', 0.0):.3f}" if isinstance(jump_analysis.get('solver_update_ms_mean'), float) and math.isfinite(jump_analysis.get('solver_update_ms_mean')) else "n/a"],
                    ["accept_ratio_mean", f"{jump_analysis.get('accept_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('accept_ratio_mean'), float) and math.isfinite(jump_analysis.get('accept_ratio_mean')) else "n/a"],
                    ["match_ratio_mean", f"{jump_analysis.get('match_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('match_ratio_mean'), float) and math.isfinite(jump_analysis.get('match_ratio_mean')) else "n/a"],
                    ["inlier_ratio_mean", f"{jump_analysis.get('inlier_ratio_mean', 0.0):.3f}" if isinstance(jump_analysis.get('inlier_ratio_mean'), float) and math.isfinite(jump_analysis.get('inlier_ratio_mean')) else "n/a"],
                    ["isolation_effect", jump_analysis.get("isolation_effect", "n/a")],
                ],
            ))
            lines.append("Yaw root-cause shootout summary:")
            lines.append("")
            lines.append(md_table(
                ["Metric", "Value"],
                [
                    ["yaw_root_cause_summary", jump_analysis.get("yaw_root_cause_summary", "n/a")],
                    ["candidate_A_velocity_prior_shared_score", f"{jump_analysis.get('candidate_A_velocity_prior_shared_score', 0.0):.1f}"],
                    ["candidate_B_orientation_semantics_score", f"{jump_analysis.get('candidate_B_orientation_semantics_score', 0.0):.1f}"],
                    ["candidate_C_imu_bias_gravity_score", f"{jump_analysis.get('candidate_C_imu_bias_gravity_score', 0.0):.1f}"],
                    ["leader", f"{jump_analysis.get('yaw_root_cause_leader', 'n/a')}: {jump_analysis.get('yaw_root_cause_leader_label', 'n/a')}"],
                    ["second", f"{jump_analysis.get('yaw_root_cause_second', 'n/a')}: {jump_analysis.get('yaw_root_cause_second_label', 'n/a')}"],
                    ["weakest", f"{jump_analysis.get('yaw_root_cause_weakest', 'n/a')}: {jump_analysis.get('yaw_root_cause_weakest_label', 'n/a')}"],
                    ["next_minimum_fix_target", jump_analysis.get("yaw_root_cause_next_fix_target", "n/a")],
                    ["top_yaw_chain_inconsistency_ratio", f"{jump_analysis.get('top_yaw_chain_inconsistency_ratio', 0.0):.3f}"],
                    ["yaw_chain_inconsistency_count_on_top_frames", jump_analysis.get("yaw_chain_inconsistency_count_on_top_frames", 0)],
                    ["yaw_space_gap_p95", f"{jump_analysis.get('yaw_space_gap_p95', 0.0):.3f}"],
                    ["leader_evidence", jump_analysis.get("yaw_root_cause_leader_evidence", "")],
                    ["second_evidence", jump_analysis.get("yaw_root_cause_second_evidence", "")],
                    ["weakest_evidence", jump_analysis.get("yaw_root_cause_weakest_evidence", "")],
                ],
            ))
        pitch_roll_stage_rows = [
            [stage, f"{score:.3f}"]
            for stage, score in jump_analysis.get("pitch_roll_stage_scores", {}).items()
        ]
        if pitch_roll_stage_rows:
            lines.append("Pitch/Roll stage audit:")
            lines.append("")
            lines.append(md_table(
                ["Stage", "Pitch/Roll Score"],
                pitch_roll_stage_rows,
            ))
        start_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_start_to_frontend_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_start_to_frontend_rotation_rad') or 0.0):.3f}",
                item.get("start_pose_source_kind", ""),
                item.get("start_pose_support_mismatch_reason", ""),
                f"{(item.get('lidar_layout_domain_begin') or 0.0):.6f}",
                f"{(item.get('lidar_layout_domain_end') or 0.0):.6f}",
                f"{(item.get('candidate_correspondence_count') or 0):.0f}",
                f"{(item.get('accepted_correspondence_count') or 0):.0f}",
                f"{(item.get('accept_ratio') or 0.0):.3f}",
                f"{(item.get('match_ratio') or 0.0):.3f}",
                f"{(item.get('inlier_ratio') or 0.0):.3f}",
                f"{(item.get('target_point_count') or 0):.0f}",
                f"{(item.get('target_voxel_count') or 0):.0f}",
            ]
            for item in jump_analysis.get("top_start_to_frontend_translation_frames", [])
        ]
        lines.append("Top jump frames by start->frontend translation:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_t_m",
                "delta_r_rad",
                "start_source",
                "mismatch_reason",
                "layout_begin",
                "layout_end",
                "candidates",
                "accepted",
                "accept_ratio",
                "match_ratio",
                "inlier_ratio",
                "target_points",
                "target_voxels",
            ],
            start_rows,
        ))
        solver_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_frontend_to_final_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_rotation_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_postsolve_query_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_postsolve_strict_local_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_postsolve_active_window_to_postsolve_strict_local_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_postsolve_query_to_final_translation_norm') or 0.0):.3f}",
                item.get("postsolve_query_support_mismatch_reason", ""),
                item.get("postsolve_strict_local_support_mismatch_reason", ""),
                item.get("postsolve_query_layout_name", ""),
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                item.get("carried_boundary_oldest_key_summary", ""),
                item.get("oldest_survivor_key_summary", ""),
                item.get("postsolve_query_support_keys_summary", ""),
                item.get("postsolve_strict_local_support_keys_summary", ""),
                item.get("reeliminated_variable_count", ""),
                item.get("relinearized_pose_variable_count", ""),
                item.get("relinearized_aux_variable_count", ""),
                item.get("relinearized_shared_variable_count", ""),
                item.get("recalculated_lidar_current_segment_factor_count", ""),
                item.get("recalculated_lidar_same_support_factor_count", ""),
                item.get("recalculated_lidar_cross_support_factor_count", ""),
            ]
            for item in jump_analysis.get("top_frontend_to_final_translation_frames", [])
        ]
        lines.append("Top jump frames by frontend->final translation:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_t_m",
                "delta_r_rad",
                "front->postsolve_active_window",
                "front->postsolve_strict_local",
                "active_window->strict_local",
                "postsolve_active_window->final",
                "postsolve_reason",
                "postsolve_strict_reason",
                "postsolve_layout",
                "solver_update_ms",
                "carried_boundary_oldest",
                "oldest_survivor",
                "postsolve_active_window_support",
                "postsolve_strict_local_support",
                "reelim",
                "relin_pose",
                "relin_aux",
                "relin_shared",
                "current_segment_recalc",
                "same_support_recalc",
                "cross_support_recalc",
            ],
            solver_rows,
        ))
        boundary_rows = [
            [
                item.get("frame_id", ""),
                item.get("carried_boundary_oldest_key_summary", ""),
                item.get("oldest_survivor_key_summary", ""),
                item.get("postsolve_query_support_keys_summary", ""),
                item.get("postsolve_query_support_mismatch_reason", ""),
                item.get("postsolve_strict_local_support_keys_summary", ""),
                item.get("postsolve_strict_local_support_mismatch_reason", ""),
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                f"{(item.get('reeliminated_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_pose_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_aux_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_shared_variable_count') or 0):.0f}",
                f"{(item.get('recalculated_velocity_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_prior_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_imu_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_same_support_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_cross_support_factor_count') or 0):.0f}",
            ]
            for item in jump_analysis.get("top_frontend_to_final_translation_frames", [])
        ]
        lines.append("Boundary amplification summary on top frontend->final jump frames:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "carried_boundary_oldest",
                "oldest_survivor",
                "postsolve_active_window_support",
                "postsolve_reason",
                "postsolve_strict_local_support",
                "postsolve_strict_reason",
                "solver_update_ms",
                "reelim",
                "relin_pose",
                "relin_aux",
                "relin_shared",
                "recalc_velocity",
                "recalc_prior",
                "recalc_imu",
                "recalc_same",
                "recalc_cross",
            ],
            boundary_rows,
        ))
        strict_local_residual_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_frontend_to_final_translation_norm') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_rotation_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_yaw_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_pitch_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_roll_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_dx') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_dy') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_dz') or 0.0):.3f}",
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                f"{(item.get('reeliminated_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_pose_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_aux_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_shared_variable_count') or 0):.0f}",
                f"{(item.get('recalculated_velocity_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_prior_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_imu_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_same_support_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_cross_support_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_current_segment_factor_count') or 0):.0f}",
                item.get("frontend_seed_source", ""),
                f"{int(item.get('frontend_seed_fallback_used') or 0)}",
                item.get("strict_local_query_reason", ""),
                item.get("strict_local_query_support_keys_summary", ""),
            ]
            for item in jump_analysis.get("top_strict_local_residual_frames", [])
        ]
        lines.append("Top strict-local residual frames:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_t_m",
                "delta_r_rad",
                "delta_yaw",
                "delta_pitch",
                "delta_roll",
                "delta_dx",
                "delta_dy",
                "delta_dz",
                "solver_update_ms",
                "reelim",
                "relin_pose",
                "relin_aux",
                "relin_shared",
                "recalc_velocity",
                "recalc_prior",
                "recalc_imu",
                "recalc_lidar",
                "same_support",
                "cross_support",
                "current_segment_recalc",
                "seed_source",
                "seed_fallback",
                "strict_local_query_reason",
                "strict_local_query_support",
            ],
            strict_local_residual_rows,
        ))
        top_yaw_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_frontend_to_final_yaw_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_pitch_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_roll_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_translation_norm') or 0.0):.3f}",
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                f"{(item.get('reeliminated_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_pose_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_aux_variable_count') or 0):.0f}",
                f"{(item.get('relinearized_shared_variable_count') or 0):.0f}",
                f"{(item.get('recalculated_velocity_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_prior_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_imu_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_same_support_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_cross_support_factor_count') or 0):.0f}",
                f"{(item.get('current_velocity_norm') or 0.0):.3f}",
                f"{(item.get('current_velocity_heading_rad') or 0.0):.3f}",
                f"{(item.get('velocity_factor_count') or 0):.0f}",
                f"{(item.get('prior_factor_count') or 0):.0f}",
                f"{(item.get('uses_shared_imu_state') or 0):.0f}",
                f"{(item.get('frontend_world_to_lidar_yaw') or 0.0):.3f}",
                f"{(item.get('frontend_world_to_imu_yaw') or 0.0):.3f}",
                f"{(item.get('final_world_to_lidar_yaw') or 0.0):.3f}",
                f"{(item.get('final_world_to_imu_yaw') or 0.0):.3f}",
                f"{(item.get('lidar_to_imu_extrinsic_yaw') or 0.0):.3f}",
                item.get("yaw_chain_consistency_flag", ""),
                f"{(item.get('gyro_bias_norm') or 0.0):.6f}",
                f"{(item.get('accel_bias_norm') or 0.0):.6f}",
                (
                    f"[{(item.get('gravity_world_x') or 0.0):.3f},"
                    f"{(item.get('gravity_world_y') or 0.0):.3f},"
                    f"{(item.get('gravity_world_z') or 0.0):.3f}] / "
                    f"tilt={(item.get('gravity_dir_tilt_rad') or 0.0):.3f}"
                ),
                item.get("frontend_seed_source", ""),
                f"{int(item.get('frontend_seed_fallback_used') or 0)}",
                item.get("strict_local_query_reason", ""),
            ]
            for item in jump_analysis.get("top_yaw_residual_frames", [])
        ]
        lines.append("Top yaw residual frames:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_yaw",
                "delta_pitch",
                "delta_roll",
                "delta_t_m",
                "solver_update_ms",
                "reelim",
                "relin_pose",
                "relin_aux",
                "relin_shared",
                "recalc_velocity",
                "recalc_prior",
                "recalc_imu",
                "recalc_lidar",
                "same_support",
                "cross_support",
                "current_velocity_norm",
                "current_velocity_heading",
                "velocity_factor_count",
                "prior_factor_count",
                "uses_shared_imu_state",
                "frontend_w_l_yaw",
                "frontend_w_i_yaw",
                "final_w_l_yaw",
                "final_w_i_yaw",
                "lidar_to_imu_extrinsic_yaw",
                "yaw_chain_consistency_flag",
                "gyro_bias_norm",
                "accel_bias_norm",
                "gravity_probe",
                "seed_source",
                "seed_fallback",
                "strict_local_query_reason",
            ],
            top_yaw_rows,
        ))
        top_pitch_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_frontend_to_final_pitch_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_roll_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_translation_norm') or 0.0):.3f}",
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                f"{(item.get('recalculated_imu_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_prior_factor_count') or 0):.0f}",
                f"{(item.get('relinearized_shared_variable_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_factor_count') or 0):.0f}",
                f"{(item.get('match_ratio') or 0.0):.3f}",
                f"{(item.get('inlier_ratio') or 0.0):.3f}",
                f"{(item.get('target_point_count') or 0):.0f}",
                f"{(item.get('target_voxel_count') or 0):.0f}",
                item.get("frontend_seed_mode", ""),
                item.get("frontend_seed_source", ""),
                f"{int(item.get('frontend_seed_fallback_used') or 0)}",
                f"{(item.get('frontend_seed_imu_sample_count') or 0):.0f}",
                item.get("strict_local_query_reason", ""),
                item.get("postsolve_query_support_mismatch_reason", ""),
            ]
            for item in jump_analysis.get("top_pitch_residual_frames", [])
        ]
        lines.append("Top pitch residual frames:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_pitch",
                "delta_roll",
                "delta_t_m",
                "solver_update_ms",
                "recalc_imu",
                "recalc_prior",
                "relin_shared",
                "recalc_lidar",
                "match_ratio",
                "inlier_ratio",
                "target_points",
                "target_voxels",
                "seed_mode",
                "seed_source",
                "seed_fallback",
                "seed_imu_samples",
                "strict_local_query_reason",
                "postsolve_reason",
            ],
            top_pitch_rows,
        ))
        top_roll_rows = [
            [
                item.get("frame_id", ""),
                f"{(item.get('delta_frontend_to_final_pitch_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_roll_rad') or 0.0):.3f}",
                f"{(item.get('delta_frontend_to_final_translation_norm') or 0.0):.3f}",
                f"{(item.get('solver_update_ms') or 0.0):.3f}",
                f"{(item.get('recalculated_imu_factor_count') or 0):.0f}",
                f"{(item.get('recalculated_prior_factor_count') or 0):.0f}",
                f"{(item.get('relinearized_shared_variable_count') or 0):.0f}",
                f"{(item.get('recalculated_lidar_factor_count') or 0):.0f}",
                f"{(item.get('match_ratio') or 0.0):.3f}",
                f"{(item.get('inlier_ratio') or 0.0):.3f}",
                f"{(item.get('target_point_count') or 0):.0f}",
                f"{(item.get('target_voxel_count') or 0):.0f}",
                item.get("frontend_seed_mode", ""),
                item.get("frontend_seed_source", ""),
                f"{int(item.get('frontend_seed_fallback_used') or 0)}",
                f"{(item.get('frontend_seed_imu_sample_count') or 0):.0f}",
                item.get("strict_local_query_reason", ""),
                item.get("postsolve_query_support_mismatch_reason", ""),
            ]
            for item in jump_analysis.get("top_roll_residual_frames", [])
        ]
        lines.append("Top roll residual frames:")
        lines.append("")
        lines.append(md_table(
            [
                "frame_id",
                "delta_pitch",
                "delta_roll",
                "delta_t_m",
                "solver_update_ms",
                "recalc_imu",
                "recalc_prior",
                "relin_shared",
                "recalc_lidar",
                "match_ratio",
                "inlier_ratio",
                "target_points",
                "target_voxels",
                "seed_mode",
                "seed_source",
                "seed_fallback",
                "seed_imu_samples",
                "strict_local_query_reason",
                "postsolve_reason",
            ],
            top_roll_rows,
        ))
    else:
        lines.append("jump_diagnostics.csv was not available for this run.")
        lines.append("")

    lines.append("## Solver Update Analysis")
    lines.append("")
    if solver_update_analysis.get("available"):
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["row_count", solver_update_analysis.get("row_count", 0)],
                ["solver_modes", ", ".join(solver_update_analysis.get("solver_modes", [])) or "_none_"],
                ["used_incremental_solver_values", ", ".join(str(v) for v in solver_update_analysis.get("used_incremental_solver_values", [])) or "_none_"],
                ["fallback_used_values", ", ".join(str(v) for v in solver_update_analysis.get("fallback_used_values", [])) or "_none_"],
                ["solver_update_ms_mean", f"{solver_update_analysis.get('solver_update_ms_mean', 0.0):.3f}" if "solver_update_ms_mean" in solver_update_analysis else "n/a"],
                ["solver_update_ms_p95", f"{solver_update_analysis.get('solver_update_ms_p95', 0.0):.3f}" if "solver_update_ms_p95" in solver_update_analysis else "n/a"],
                ["isam_reported_update_ms_mean", f"{solver_update_analysis.get('isam_reported_update_ms_mean', 0.0):.3f}" if "isam_reported_update_ms_mean" in solver_update_analysis else "n/a"],
                ["new_factor_count_mean", f"{solver_update_analysis.get('new_factor_count_mean', 0.0):.3f}" if "new_factor_count_mean" in solver_update_analysis else "n/a"],
                ["new_value_count_mean", f"{solver_update_analysis.get('new_value_count_mean', 0.0):.3f}" if "new_value_count_mean" in solver_update_analysis else "n/a"],
                ["reeliminated_variable_count_mean", f"{solver_update_analysis.get('reeliminated_variable_count_mean', 0.0):.3f}" if "reeliminated_variable_count_mean" in solver_update_analysis else "n/a"],
                ["observed_key_count_mean", f"{solver_update_analysis.get('observed_key_count_mean', 0.0):.3f}" if "observed_key_count_mean" in solver_update_analysis else "n/a"],
                ["current_nonlinear_factor_count_mean", f"{solver_update_analysis.get('current_nonlinear_factor_count_mean', 0.0):.3f}" if "current_nonlinear_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_imu_factor_count_mean", f"{solver_update_analysis.get('active_window_imu_factor_count_mean', 0.0):.3f}" if "active_window_imu_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_velocity_factor_count_mean", f"{solver_update_analysis.get('active_window_velocity_factor_count_mean', 0.0):.3f}" if "active_window_velocity_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_lidar_factor_count_mean", f"{solver_update_analysis.get('active_window_lidar_factor_count_mean', 0.0):.3f}" if "active_window_lidar_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_lidar_current_segment_factor_count_mean", f"{solver_update_analysis.get('active_window_lidar_current_segment_factor_count_mean', 0.0):.3f}" if "active_window_lidar_current_segment_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_lidar_old_segment_factor_count_mean", f"{solver_update_analysis.get('active_window_lidar_old_segment_factor_count_mean', 0.0):.3f}" if "active_window_lidar_old_segment_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_prior_factor_count_mean", f"{solver_update_analysis.get('active_window_prior_factor_count_mean', 0.0):.3f}" if "active_window_prior_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_shared_jkg_touching_factor_count_mean", f"{solver_update_analysis.get('active_window_shared_jkg_touching_factor_count_mean', 0.0):.3f}" if "active_window_shared_jkg_touching_factor_count_mean" in solver_update_analysis else "n/a"],
                ["active_window_non_imu_shared_jkg_factor_count_mean", f"{solver_update_analysis.get('active_window_non_imu_shared_jkg_factor_count_mean', 0.0):.3f}" if "active_window_non_imu_shared_jkg_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_imu_factor_count_mean", f"{solver_update_analysis.get('recalculated_imu_factor_count_mean', 0.0):.3f}" if "recalculated_imu_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_velocity_factor_count_mean", f"{solver_update_analysis.get('recalculated_velocity_factor_count_mean', 0.0):.3f}" if "recalculated_velocity_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_lidar_factor_count_mean", f"{solver_update_analysis.get('recalculated_lidar_factor_count_mean', 0.0):.3f}" if "recalculated_lidar_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_lidar_current_segment_factor_count_mean", f"{solver_update_analysis.get('recalculated_lidar_current_segment_factor_count_mean', 0.0):.3f}" if "recalculated_lidar_current_segment_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_lidar_old_segment_factor_count_mean", f"{solver_update_analysis.get('recalculated_lidar_old_segment_factor_count_mean', 0.0):.3f}" if "recalculated_lidar_old_segment_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_lidar_same_support_factor_count_mean", f"{solver_update_analysis.get('recalculated_lidar_same_support_factor_count_mean', 0.0):.3f}" if "recalculated_lidar_same_support_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_lidar_cross_support_factor_count_mean", f"{solver_update_analysis.get('recalculated_lidar_cross_support_factor_count_mean', 0.0):.3f}" if "recalculated_lidar_cross_support_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_prior_factor_count_mean", f"{solver_update_analysis.get('recalculated_prior_factor_count_mean', 0.0):.3f}" if "recalculated_prior_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_shared_jkg_touching_factor_count_mean", f"{solver_update_analysis.get('recalculated_shared_jkg_touching_factor_count_mean', 0.0):.3f}" if "recalculated_shared_jkg_touching_factor_count_mean" in solver_update_analysis else "n/a"],
                ["recalculated_unclassified_factor_count_mean", f"{solver_update_analysis.get('recalculated_unclassified_factor_count_mean', 0.0):.3f}" if "recalculated_unclassified_factor_count_mean" in solver_update_analysis else "n/a"],
                ["relinearized_pose_variable_count_mean", f"{solver_update_analysis.get('relinearized_pose_variable_count_mean', 0.0):.3f}" if "relinearized_pose_variable_count_mean" in solver_update_analysis else "n/a"],
                ["relinearized_aux_variable_count_mean", f"{solver_update_analysis.get('relinearized_aux_variable_count_mean', 0.0):.3f}" if "relinearized_aux_variable_count_mean" in solver_update_analysis else "n/a"],
                ["relinearized_shared_variable_count_mean", f"{solver_update_analysis.get('relinearized_shared_variable_count_mean', 0.0):.3f}" if "relinearized_shared_variable_count_mean" in solver_update_analysis else "n/a"],
                ["affected_pose_key_count_mean", f"{solver_update_analysis.get('affected_pose_key_count_mean', 0.0):.3f}" if "affected_pose_key_count_mean" in solver_update_analysis else "n/a"],
                ["affected_aux_key_count_mean", f"{solver_update_analysis.get('affected_aux_key_count_mean', 0.0):.3f}" if "affected_aux_key_count_mean" in solver_update_analysis else "n/a"],
                ["affected_shared_key_count_mean", f"{solver_update_analysis.get('affected_shared_key_count_mean', 0.0):.3f}" if "affected_shared_key_count_mean" in solver_update_analysis else "n/a"],
                ["new_factor_vs_active_window_corr", f"{solver_update_analysis.get('new_factor_vs_active_window_corr', 'n/a')}"],
                ["new_value_vs_active_window_corr", f"{solver_update_analysis.get('new_value_vs_active_window_corr', 'n/a')}"],
                ["active_window_imu_factor_vs_solver_update_corr", f"{solver_update_analysis.get('active_window_imu_factor_vs_solver_update_corr', 'n/a')}"],
                ["active_window_lidar_current_segment_factor_vs_solver_update_corr", f"{solver_update_analysis.get('active_window_lidar_current_segment_factor_vs_solver_update_corr', 'n/a')}"],
                ["active_window_lidar_old_segment_factor_vs_solver_update_corr", f"{solver_update_analysis.get('active_window_lidar_old_segment_factor_vs_solver_update_corr', 'n/a')}"],
                ["active_window_shared_jkg_touching_factor_vs_solver_update_corr", f"{solver_update_analysis.get('active_window_shared_jkg_touching_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_imu_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_imu_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_velocity_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_velocity_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_lidar_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_lidar_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_lidar_current_segment_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_lidar_current_segment_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_lidar_old_segment_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_lidar_old_segment_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_lidar_same_support_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_lidar_same_support_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_lidar_cross_support_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_lidar_cross_support_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_prior_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_prior_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_shared_jkg_touching_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_shared_jkg_touching_factor_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_unclassified_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_unclassified_factor_vs_solver_update_corr', 'n/a')}"],
                ["reeliminated_variable_vs_solver_update_corr", f"{solver_update_analysis.get('reeliminated_variable_vs_solver_update_corr', 'n/a')}"],
                ["relinearized_variable_vs_solver_update_corr", f"{solver_update_analysis.get('relinearized_variable_vs_solver_update_corr', 'n/a')}"],
                ["recalculated_factor_vs_solver_update_corr", f"{solver_update_analysis.get('recalculated_factor_vs_solver_update_corr', 'n/a')}"],
                ["dominant_recalculated_family", solver_update_analysis.get("dominant_recalculated_family", "n/a")],
                ["dominant_recalculated_family_corr", f"{solver_update_analysis.get('dominant_recalculated_family_corr', 'n/a')}"],
                ["unavailable_internal_timing", solver_update_analysis.get("unavailable_internal_timing", False)],
                ["unavailable_internal_fields", ", ".join(solver_update_analysis.get("unavailable_internal_fields", [])) or "_none_"],
            ],
        ))
        if solver_update_analysis.get("stage_summary"):
            stage_rows = [
                [
                    item["stage"],
                    f"{item['mean_ms']:.3f}",
                    f"{item['p95_ms']:.3f}",
                    f"{item['max_ms']:.3f}",
                    f"{100.0 * item['mean_share']:.1f}%",
                ]
                for item in solver_update_analysis.get("stage_summary", [])
            ]
            lines.append("Solver-update stage breakdown:")
            lines.append("")
            lines.append(md_table(["Stage", "Mean ms", "P95 ms", "Max ms", "Mean Share"], stage_rows))
        recalculated_family_rows = []
        for label, key_prefix, corr_key in [
            ("IMU", "recalculated_imu_factor_count", "recalculated_imu_factor_vs_solver_update_corr"),
            ("VELOCITY", "recalculated_velocity_factor_count", "recalculated_velocity_factor_vs_solver_update_corr"),
            ("LIDAR", "recalculated_lidar_factor_count", "recalculated_lidar_factor_vs_solver_update_corr"),
            ("PRIOR", "recalculated_prior_factor_count", "recalculated_prior_factor_vs_solver_update_corr"),
            ("SHARED_JKG_TOUCHING", "recalculated_shared_jkg_touching_factor_count", "recalculated_shared_jkg_touching_factor_vs_solver_update_corr"),
            ("UNCLASSIFIED", "recalculated_unclassified_factor_count", "recalculated_unclassified_factor_vs_solver_update_corr"),
        ]:
            mean_key = f"{key_prefix}_mean"
            if mean_key not in solver_update_analysis:
                continue
            recalculated_family_rows.append([
                label,
                f"{solver_update_analysis.get(mean_key, 0.0):.3f}",
                f"{solver_update_analysis.get(f'{key_prefix}_p95', 0.0):.3f}",
                f"{solver_update_analysis.get(f'{key_prefix}_max', 0.0):.3f}",
                f"{solver_update_analysis.get(corr_key, 'n/a')}",
            ])
        if recalculated_family_rows:
            lines.append("Recalculated factor family breakdown:")
            lines.append("")
            lines.append(md_table(["Family", "Mean", "P95", "Max", "Corr vs solver_update_ms"], recalculated_family_rows))
        lidar_churn_rows = []
        for label, key_prefix, corr_key in [
            ("ACTIVE_CURRENT_SEGMENT", "active_window_lidar_current_segment_factor_count", "active_window_lidar_current_segment_factor_vs_solver_update_corr"),
            ("ACTIVE_OLD_SEGMENT", "active_window_lidar_old_segment_factor_count", "active_window_lidar_old_segment_factor_vs_solver_update_corr"),
            ("RECALC_CURRENT_SEGMENT", "recalculated_lidar_current_segment_factor_count", "recalculated_lidar_current_segment_factor_vs_solver_update_corr"),
            ("RECALC_OLD_SEGMENT", "recalculated_lidar_old_segment_factor_count", "recalculated_lidar_old_segment_factor_vs_solver_update_corr"),
            ("RECALC_SUPPORT_OVERLAP", "recalculated_lidar_same_support_factor_count", "recalculated_lidar_same_support_factor_vs_solver_update_corr"),
            ("RECALC_CROSS_SUPPORT", "recalculated_lidar_cross_support_factor_count", "recalculated_lidar_cross_support_factor_vs_solver_update_corr"),
        ]:
            mean_key = f"{key_prefix}_mean"
            if mean_key not in solver_update_analysis:
                continue
            lidar_churn_rows.append([
                label,
                f"{solver_update_analysis.get(mean_key, 0.0):.3f}",
                f"{solver_update_analysis.get(f'{key_prefix}_p95', 0.0):.3f}",
                f"{solver_update_analysis.get(f'{key_prefix}_max', 0.0):.3f}",
                f"{solver_update_analysis.get(corr_key, 'n/a')}",
            ])
        if lidar_churn_rows:
            lines.append("LiDAR churn split:")
            lines.append("")
            lines.append(md_table(["Bucket", "Mean", "P95", "Max", "Corr vs solver_update_ms"], lidar_churn_rows))
    else:
        lines.append("solver_update_profile.csv was not available for this run.")
        lines.append("")

    lines.append("## LiDAR Factor Internal Load")
    lines.append("")
    if lidar_factor_internal_analysis.get("available"):
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["row_count", lidar_factor_internal_analysis.get("row_count", 0)],
                ["bucket_modes", ", ".join(lidar_factor_internal_analysis.get("bucket_modes", [])) or "_none_"],
                ["points_in_bucket_mean", f"{lidar_factor_internal_analysis.get('points_in_bucket_mean', 0.0):.1f}" if "points_in_bucket_mean" in lidar_factor_internal_analysis else "n/a"],
                ["points_in_bucket_p95", f"{lidar_factor_internal_analysis.get('points_in_bucket_p95', 0.0):.1f}" if "points_in_bucket_p95" in lidar_factor_internal_analysis else "n/a"],
                ["factor_total_ms_mean", f"{lidar_factor_internal_analysis.get('factor_total_ms_mean', 0.0):.3f}" if "factor_total_ms_mean" in lidar_factor_internal_analysis else "n/a"],
                ["factor_total_ms_p95", f"{lidar_factor_internal_analysis.get('factor_total_ms_p95', 0.0):.3f}" if "factor_total_ms_p95" in lidar_factor_internal_analysis else "n/a"],
                ["correspondence_share_of_factor_total", f"{100.0 * lidar_factor_internal_analysis.get('correspondence_share_of_factor_total', 0.0):.1f}%" if "correspondence_share_of_factor_total" in lidar_factor_internal_analysis else "n/a"],
                ["match_ratio_mean", f"{lidar_factor_internal_analysis.get('match_ratio_mean', 0.0):.3f}" if "match_ratio_mean" in lidar_factor_internal_analysis else "n/a"],
                ["inlier_ratio_mean", f"{lidar_factor_internal_analysis.get('inlier_ratio_mean', 0.0):.3f}" if "inlier_ratio_mean" in lidar_factor_internal_analysis else "n/a"],
            ],
        ))
    else:
        lines.append("lidar_factor_internal_profile.csv was not available for this run.")
        lines.append("")

    if pipeline_analysis.get("available"):
        module_rows = [
            [item["module"], item["count"], f"{item['mean_ms']:.3f}", f"{item['p95_ms']:.3f}", f"{item['max_ms']:.3f}"]
            for item in pipeline_analysis.get("module_stats", [])
        ]
        lines.append("Pipeline timing summary:")
        lines.append("")
        lines.append(md_table(["Module", "Count", "Mean ms", "P95 ms", "Max ms"], module_rows))
    else:
        lines.append("pipeline_timing.csv was not available for this run.")
        lines.append("")

    lines.append("## Slowest Frames / Outliers")
    lines.append("")
    if slow_frame_analysis.get("available"):
        lines.append(md_table(
            [
                "frame_id",
                "frame_total_ms",
                "lm_solve_ms",
                "target_map_prep_ms",
                "preprocess_ms",
                "bucket_build_ms",
                "lidar_factor_build_ms",
                "imu_factor_build_ms",
                "points_in_bucket",
                "factor_total_ms",
                "match_ratio",
                "inlier_ratio",
                "warning_count_for_frame",
                "top_stage",
            ],
            [
                [
                    row.get("frame_id", ""),
                    f"{row.get('frame_total_ms', 0.0):.3f}",
                    f"{row.get('lm_solve_ms', 0.0):.3f}",
                    f"{row.get('target_map_prep_ms', 0.0):.3f}",
                    f"{row.get('preprocess_ms', 0.0):.3f}",
                    f"{row.get('bucket_build_ms', 0.0):.3f}",
                    f"{row.get('lidar_factor_build_ms', 0.0):.3f}",
                    f"{row.get('imu_factor_build_ms', 0.0):.3f}",
                    row.get("points_in_bucket", ""),
                    f"{row.get('factor_total_ms', 0.0):.3f}" if row.get("factor_total_ms") is not None else "",
                    f"{row.get('match_ratio', 0.0):.3f}" if row.get("match_ratio") is not None else "",
                    f"{row.get('inlier_ratio', 0.0):.3f}" if row.get("inlier_ratio") is not None else "",
                    row.get("warning_count_for_frame", "n/a"),
                    row.get("top_stage", ""),
                ]
                for row in slow_frame_analysis.get("slowest_frames", [])
            ],
        ))
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["top10_lm_dominant_count", slow_frame_analysis.get("lm_dominant_count", 0)],
                ["top10_target_map_prep_dominant_count", slow_frame_analysis.get("target_map_prep_dominant_count", 0)],
                ["warning_count_available", slow_frame_analysis.get("warning_count_available", False)],
            ],
        ))
    else:
        lines.append("Slow-frame analysis was not available because frontend_frame_profile.csv was unavailable.")
        lines.append("")

    lines.append("## Optimizer Behavior Analysis")
    lines.append("")
    lines.append(md_table(
        ["Metric", "Value"],
        [
            ["optimization_diagnostics_available", optimizer_analysis.get("optimization_diagnostics_available", False)],
            ["lm_iteration_rows_present", optimizer_analysis.get("lm_iteration_rows_present", False)],
            ["lm_iteration_trace_reason", optimizer_analysis.get("lm_iteration_trace_reason", "unavailable")],
            ["frames_with_nonzero_iterations", optimizer_analysis.get("frames_with_nonzero_iterations", 0)],
            ["lm_iteration_mean", f"{optimizer_analysis.get('lm_iteration_mean', 0.0):.3f}" if "lm_iteration_mean" in optimizer_analysis else "n/a"],
            ["lm_iteration_p95", f"{optimizer_analysis.get('lm_iteration_p95', 0.0):.3f}" if "lm_iteration_p95" in optimizer_analysis else "n/a"],
            ["lm_iteration_max", optimizer_analysis.get("lm_iteration_max", "n/a")],
            ["initial_cost_mean", f"{optimizer_analysis.get('initial_cost_mean', 0.0):.3f}" if "initial_cost_mean" in optimizer_analysis else "n/a"],
            ["final_cost_mean", f"{optimizer_analysis.get('final_cost_mean', 0.0):.3f}" if "final_cost_mean" in optimizer_analysis else "n/a"],
            ["cost_drop_ratio_mean", f"{optimizer_analysis.get('cost_drop_ratio_mean', 0.0):.3f}" if "cost_drop_ratio_mean" in optimizer_analysis else "n/a"],
            ["rejected_steps_count", optimizer_analysis.get("rejected_steps_count", "n/a")],
        ],
    ))

    lines.append("## Target Map Preparation Breakdown")
    lines.append("")
    if target_map_analysis.get("available"):
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["mean_ms", f"{target_map_analysis.get('mean_ms', 0.0):.3f}"],
                ["p95_ms", f"{target_map_analysis.get('p95_ms', 0.0):.3f}"],
                ["max_ms", f"{target_map_analysis.get('max_ms', 0.0):.3f}"],
                ["is_black_box", target_map_analysis.get("is_black_box", True)],
                ["substage_columns_present", ", ".join(target_map_analysis.get("substage_columns_present", [])) or "_none_"],
                ["substage_columns_missing", ", ".join(target_map_analysis.get("substage_columns_missing", [])) or "_none_"],
            ],
        ))
        if target_map_analysis.get("is_black_box", False):
            lines.append("target_map_prep_ms is currently a dominant black-box stage; finer-grained target preparation profiling is still missing.")
            lines.append("")
    else:
        lines.append("target_map_prep analysis was not available.")
        lines.append("")

    lines.append("## Graph/Problem Size Summary")
    lines.append("")
    if graph_analysis.get("available"):
        rows = [
            [item["field"], f"{item['mean']:.3f}", f"{item['p95']:.3f}", f"{item['max']:.3f}"]
            for item in graph_analysis.get("available_fields", [])
        ]
        lines.append(md_table(["Field", "Mean", "P95", "Max"], rows))
        lines.append(f"Missing graph size telemetry: {', '.join(graph_analysis.get('missing_graph_size_telemetry', [])) or '_none_'}")
        lines.append("")
    else:
        lines.append("Graph/problem size summary was not available.")
        lines.append("")

    lines.append("## Bucket Diagnostics")
    lines.append("")
    if bucket_diagnostics.get("available"):
        lines.append(md_table(
            ["Metric", "Value"],
            [
                ["bucket_mode", ", ".join(bucket_diagnostics.get("bucket_mode", []))],
                ["actual_bucket_count_mean", f"{bucket_diagnostics.get('actual_bucket_count_mean', 0.0):.3f}" if "actual_bucket_count_mean" in bucket_diagnostics else "n/a"],
                ["actual_bucket_count_p95", f"{bucket_diagnostics.get('actual_bucket_count_p95', 0.0):.3f}" if "actual_bucket_count_p95" in bucket_diagnostics else "n/a"],
                ["actual_bucket_count_max", f"{bucket_diagnostics.get('actual_bucket_count_max', 0.0):.3f}" if "actual_bucket_count_max" in bucket_diagnostics else "n/a"],
                ["points_in_bucket_mean", f"{bucket_diagnostics.get('points_in_bucket_mean', 0.0):.3f}" if "points_in_bucket_mean" in bucket_diagnostics else "n/a"],
                ["points_in_bucket_p95", f"{bucket_diagnostics.get('points_in_bucket_p95', 0.0):.3f}" if "points_in_bucket_p95" in bucket_diagnostics else "n/a"],
                ["points_in_bucket_max", f"{bucket_diagnostics.get('points_in_bucket_max', 0.0):.3f}" if "points_in_bucket_max" in bucket_diagnostics else "n/a"],
                ["points_in_bucket_cv", f"{bucket_diagnostics.get('points_in_bucket_cv', 0.0):.3f}" if "points_in_bucket_cv" in bucket_diagnostics else "n/a"],
                ["bucket_count_consistent_with_mode", bucket_diagnostics.get("bucket_count_consistent_with_mode", "n/a")],
                ["representative_time_mean", f"{bucket_diagnostics.get('representative_time_mean', 0.0):.6f}" if "representative_time_mean" in bucket_diagnostics else "n/a"],
            ],
        ))
    else:
        lines.append("Bucket diagnostics were not available.")
        lines.append("")

    lines.append("## Correlation Analysis")
    lines.append("")
    if correlation_analysis.get("available"):
        corr_rows = [[name, value] for name, value in correlation_analysis.get("correlations", {}).items()]
        lines.append(f"Method: `{correlation_analysis.get('method', 'unknown')}`")
        lines.append("")
        lines.append(md_table(["Pair", "Correlation"], corr_rows))
    else:
        lines.append("Correlation analysis was not available.")
        lines.append("")

    lines.append("## Findings")
    lines.append("")
    for finding in findings:
        lines.append(f"- **{finding['severity']}**: {finding['title']}  ")
        lines.append(f"  Evidence: {finding['evidence']}")
    lines.append("")

    lines.append("## Recommendations")
    lines.append("")
    for heading, items in recommendations.items():
        lines.append(f"### {heading}")
        lines.append("")
        if items:
            for rec in items:
                lines.append(f"- {rec}")
        else:
            lines.append("_none_")
        lines.append("")

    lines.append("## External Plot Integrations")
    lines.append("")
    if external_results:
        external_rows = [
            [result["script"], result["returncode"], result["stdout"][:120], result["stderr"][:120]]
            for result in external_results
        ]
        lines.append(md_table(["Script", "Return Code", "Stdout", "Stderr"], external_rows))
    else:
        lines.append("No external analysis scripts were invoked.")
        lines.append("")

    return "\n".join(lines)


def write_json_report(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze one IAP run directory and generate a report.")
    parser.add_argument("--run", default=str(DEFAULT_RUN), help=f"Run directory to analyze (default: {DEFAULT_RUN})")
    parser.add_argument("--baseline-run", default="", help="Optional baseline run directory for seed-mode comparison")
    parser.add_argument("--out", default="", help="Output analysis directory (default: <run>/analysis)")
    parser.add_argument("--no-plots", action="store_true", help="Skip new plots generated by ana_log.py")
    parser.add_argument("--skip-external-tools", action="store_true", help="Do not invoke existing ICP/GNSS/ARAIM plot scripts")
    parser.add_argument("--strict", action="store_true", help="Exit non-zero if enabled artifacts are missing or runtime errors exist")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(args.run)
    baseline_run_dir = resolve_run_dir(args.baseline_run) if args.baseline_run else None
    out_dir = Path(args.out).resolve() if args.out else (run_dir / "analysis")
    figs_dir = out_dir / "figs"
    ensure_dir(figs_dir)

    metadata = load_metadata(run_dir)
    runtime_summary = parse_runtime_logs(run_dir)
    configs = load_runtime_configs(metadata)
    config_summary = build_config_summary(configs, runtime_summary, metadata)
    artifact_statuses = scan_artifacts(run_dir, configs, config_summary)
    dataframes = load_available_frames(run_dir)
    reconcile_artifact_statuses(artifact_statuses, dataframes)

    frontend_analysis = analyze_frontend_timing(
        dataframes.get("frontend_frame_profile"),
        dataframes.get("frontend_lm_iteration"),
        figs_dir,
        render_plots=not args.no_plots,
    )
    bucket_analysis = analyze_lidar_bucket_profile(
        dataframes.get("lidar_factor_profile"),
        figs_dir,
        render_plots=not args.no_plots,
    )
    solver_update_analysis = analyze_solver_update(
        dataframes.get("solver_update_profile"),
        dataframes.get("frontend_frame_profile"),
        figs_dir,
        render_plots=not args.no_plots,
    )
    jump_analysis = analyze_jump_diagnostics(
        dataframes.get("jump_diagnostics"),
        str(config_summary.get("runtime_final_pose_surface") or config_summary.get("final_pose_surface") or "active_window"),
        config_summary,
    )
    lidar_factor_internal_analysis = analyze_lidar_factor_internal(
        dataframes.get("lidar_factor_internal_profile"),
        figs_dir,
        render_plots=not args.no_plots,
    )
    pipeline_analysis = analyze_pipeline_timing(
        dataframes.get("pipeline_timing"),
        figs_dir,
        render_plots=not args.no_plots,
    )
    mode_consistency = analyze_mode_consistency(
        metadata=metadata,
        config_summary=config_summary,
        runtime_summary=runtime_summary,
        artifact_statuses=artifact_statuses,
        frame_df=dataframes.get("frontend_frame_profile"),
    )
    slow_frame_analysis = analyze_slowest_frames(
        frame_df=dataframes.get("frontend_frame_profile"),
        bucket_df=dataframes.get("lidar_factor_profile"),
        warning_df=dataframes.get("frame_warning_profile"),
    )
    optimizer_analysis = analyze_optimizer_behavior(
        frame_df=dataframes.get("frontend_frame_profile"),
        lm_df=dataframes.get("frontend_lm_iteration"),
        artifact_statuses=artifact_statuses,
    )
    target_map_analysis = analyze_target_map_prep(dataframes.get("frontend_frame_profile"))
    graph_analysis = analyze_graph_problem_size(dataframes.get("frontend_frame_profile"))
    bucket_diagnostics = analyze_bucket_diagnostics(
        frame_df=dataframes.get("frontend_frame_profile"),
        bucket_df=dataframes.get("lidar_factor_profile"),
    )
    correlation_analysis = analyze_correlations(
        frame_df=dataframes.get("frontend_frame_profile"),
        bucket_df=dataframes.get("lidar_factor_profile"),
        warning_df=dataframes.get("frame_warning_profile"),
    )

    external_results: list[dict[str, Any]] = []
    if not args.skip_external_tools:
        external_results = integrate_existing_plots(run_dir, dataframes, figs_dir / "external")

    seed_compare_analysis = analyze_seed_mode_comparison(
        current_run_dir=run_dir,
        current_config_summary=config_summary,
        current_jump_analysis=jump_analysis,
        baseline_run_dir=baseline_run_dir,
    )

    findings = detect_findings(
        config_summary=config_summary,
        runtime_summary=runtime_summary,
        artifact_statuses=artifact_statuses,
        mode_consistency=mode_consistency,
        frontend_analysis=frontend_analysis,
        jump_analysis=jump_analysis,
        solver_update_analysis=solver_update_analysis,
        lidar_factor_internal_analysis=lidar_factor_internal_analysis,
        optimizer_analysis=optimizer_analysis,
        target_map_analysis=target_map_analysis,
        bucket_analysis=bucket_analysis,
        pipeline_analysis=pipeline_analysis,
    )
    findings.extend(detect_seed_compare_findings(seed_compare_analysis))
    recommendations = recommend_next_steps(
        artifact_statuses=artifact_statuses,
        mode_consistency=mode_consistency,
        frontend_analysis=frontend_analysis,
        jump_analysis=jump_analysis,
        solver_update_analysis=solver_update_analysis,
        lidar_factor_internal_analysis=lidar_factor_internal_analysis,
        optimizer_analysis=optimizer_analysis,
        slow_frame_analysis=slow_frame_analysis,
        target_map_analysis=target_map_analysis,
        graph_analysis=graph_analysis,
        correlation_analysis=correlation_analysis,
        runtime_summary=runtime_summary,
    )

    report_md = render_report_markdown(
        run_dir=run_dir,
        metadata=metadata,
        config_summary=config_summary,
        artifact_statuses=artifact_statuses,
        runtime_summary=runtime_summary,
        mode_consistency=mode_consistency,
        frontend_analysis=frontend_analysis,
        jump_analysis=jump_analysis,
        solver_update_analysis=solver_update_analysis,
        lidar_factor_internal_analysis=lidar_factor_internal_analysis,
        slow_frame_analysis=slow_frame_analysis,
        optimizer_analysis=optimizer_analysis,
        target_map_analysis=target_map_analysis,
        graph_analysis=graph_analysis,
        bucket_analysis=bucket_analysis,
        bucket_diagnostics=bucket_diagnostics,
        correlation_analysis=correlation_analysis,
        pipeline_analysis=pipeline_analysis,
        seed_compare_analysis=seed_compare_analysis,
        findings=findings,
        recommendations=recommendations,
        external_results=external_results,
    )
    ensure_dir(out_dir)
    report_md_path = out_dir / "report.md"
    report_json_path = out_dir / "report.json"
    report_md_path.write_text(report_md, encoding="utf-8")

    json_payload = {
        "run_dir": str(run_dir),
        "metadata": metadata,
        "config_summary": config_summary,
        "artifact_statuses": [artifact.__dict__ for artifact in artifact_statuses],
        "runtime_summary": {
            "total_lines": runtime_summary.get("total_lines", 0),
            "level_counts": dict(runtime_summary.get("level_counts", {})),
            "loaded_modules": runtime_summary.get("loaded_modules", []),
            "waiting_messages": runtime_summary.get("waiting_messages", []),
            "issues": runtime_summary.get("issues", []),
            "top_patterns": runtime_summary.get("top_patterns", []),
            "odometry_init": runtime_summary.get("odometry_init", {}),
        },
        "mode_consistency": mode_consistency,
        "frontend_analysis": frontend_analysis,
        "jump_analysis": jump_analysis,
        "solver_update_analysis": solver_update_analysis,
        "lidar_factor_internal_analysis": lidar_factor_internal_analysis,
        "slow_frame_analysis": slow_frame_analysis,
        "optimizer_analysis": optimizer_analysis,
        "target_map_analysis": target_map_analysis,
        "graph_analysis": graph_analysis,
        "bucket_analysis": bucket_analysis,
        "bucket_diagnostics": bucket_diagnostics,
        "correlation_analysis": correlation_analysis,
        "pipeline_analysis": pipeline_analysis,
        "seed_compare_analysis": seed_compare_analysis,
        "findings": findings,
        "recommendations": recommendations,
        "external_results": external_results,
    }
    write_json_report(report_json_path, json_payload)

    print(f"Analyzed run: {run_dir}")
    print(f"Markdown report: {report_md_path}")
    print(f"JSON report    : {report_json_path}")
    print(f"Figures dir     : {figs_dir}")

    strict_fail = False
    if args.strict:
        if runtime_summary.get("issues"):
            strict_fail = True
        if any(artifact.status == "expected_missing" for artifact in artifact_statuses):
            strict_fail = True
    return 1 if strict_fail else 0


if __name__ == "__main__":
    sys.exit(main())
