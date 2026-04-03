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
    ("frontend_lm_iteration", "profiling/frontend_lm_iteration.csv"),
    ("frame_warning_profile", "profiling/frame_warning_profile.csv"),
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
    summary["frontend_lm_iteration_profile"] = get_nested(log_cfg, "profiling", "frontend_lm_iteration")
    summary["frame_warning_profile"] = get_nested(log_cfg, "profiling", "frame_warning_profile")
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

    init_kv = runtime_summary.get("odometry_init", {})
    for key in ["frontend_mode", "frontend_only_mode", "lidar_bucket_mode", "lidar_target_mode"]:
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
        "frontend_frame_profile": get_nested(log_cfg, "profiling", "frontend_frame") if get_nested(log_cfg, "profiling", "frontend_frame") is not None else maybe_bool(config_summary.get("frontend_only_mode")),
        "frontend_lm_iteration": get_nested(log_cfg, "profiling", "frontend_lm_iteration") if get_nested(log_cfg, "profiling", "frontend_lm_iteration") is not None else maybe_bool(config_summary.get("frontend_only_mode")),
        "frame_warning_profile": get_nested(log_cfg, "profiling", "frame_warning_profile"),
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


def detect_findings(
    runtime_summary: dict[str, Any],
    artifact_statuses: list[ArtifactStatus],
    mode_consistency: dict[str, Any],
    frontend_analysis: dict[str, Any],
    optimizer_analysis: dict[str, Any],
    target_map_analysis: dict[str, Any],
    bucket_analysis: dict[str, Any],
    pipeline_analysis: dict[str, Any],
) -> list[dict[str, str]]:
    findings: list[dict[str, str]] = []

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

    if frontend_analysis.get("available") and optimizer_analysis.get("lm_iteration_rows_present") is False:
        stage_summary = frontend_analysis.get("stage_summary", [])
        if stage_summary and stage_summary[0]["stage"] == "lm_solve_ms":
            findings.append({
                "severity": "warn",
                "title": "LM solve dominates runtime but iteration-level telemetry is unavailable",
                "evidence": f"lm_iteration_trace_reason={optimizer_analysis.get('lm_iteration_trace_reason', 'unknown')}",
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
    if status_map.get("frontend_lm_iteration") and optimizer_analysis.get("lm_iteration_rows_present") is False:
        recommendations["Instrumentation gaps to fill"].append(
            "Gate optimizer iteration trace behind a dedicated config flag such as log.profiling.frontend_lm_iteration and emit rows whenever LM callback activity occurs."
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
    slow_frame_analysis: dict[str, Any],
    optimizer_analysis: dict[str, Any],
    target_map_analysis: dict[str, Any],
    graph_analysis: dict[str, Any],
    bucket_analysis: dict[str, Any],
    bucket_diagnostics: dict[str, Any],
    correlation_analysis: dict[str, Any],
    pipeline_analysis: dict[str, Any],
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
            ["build_info", metadata.get("build_info", "").replace("\n", "; ")],
        ],
    ))

    lines.append("## Config Summary")
    lines.append("")
    config_rows = [[k, v] for k, v in config_summary.items()]
    lines.append(md_table(["Key", "Value"], config_rows))

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
    parser.add_argument("--out", default="", help="Output analysis directory (default: <run>/analysis)")
    parser.add_argument("--no-plots", action="store_true", help="Skip new plots generated by ana_log.py")
    parser.add_argument("--skip-external-tools", action="store_true", help="Do not invoke existing ICP/GNSS/ARAIM plot scripts")
    parser.add_argument("--strict", action="store_true", help="Exit non-zero if enabled artifacts are missing or runtime errors exist")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = resolve_run_dir(args.run)
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

    findings = detect_findings(
        runtime_summary=runtime_summary,
        artifact_statuses=artifact_statuses,
        mode_consistency=mode_consistency,
        frontend_analysis=frontend_analysis,
        optimizer_analysis=optimizer_analysis,
        target_map_analysis=target_map_analysis,
        bucket_analysis=bucket_analysis,
        pipeline_analysis=pipeline_analysis,
    )
    recommendations = recommend_next_steps(
        artifact_statuses=artifact_statuses,
        mode_consistency=mode_consistency,
        frontend_analysis=frontend_analysis,
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
        slow_frame_analysis=slow_frame_analysis,
        optimizer_analysis=optimizer_analysis,
        target_map_analysis=target_map_analysis,
        graph_analysis=graph_analysis,
        bucket_analysis=bucket_analysis,
        bucket_diagnostics=bucket_diagnostics,
        correlation_analysis=correlation_analysis,
        pipeline_analysis=pipeline_analysis,
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
        "slow_frame_analysis": slow_frame_analysis,
        "optimizer_analysis": optimizer_analysis,
        "target_map_analysis": target_map_analysis,
        "graph_analysis": graph_analysis,
        "bucket_analysis": bucket_analysis,
        "bucket_diagnostics": bucket_diagnostics,
        "correlation_analysis": correlation_analysis,
        "pipeline_analysis": pipeline_analysis,
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
