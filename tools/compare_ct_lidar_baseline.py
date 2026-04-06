#!/usr/bin/env python3
# IAP-RQ-300 / IAP-RQ-410: compare CT LiDAR baseline CSV exports across BUCKET/KERNEL and runtime/diagnostic runs.
# Usage:
#   python3 tools/compare_ct_lidar_baseline.py bucket=/tmp/bucket.csv kernel=/tmp/kernel.csv

from __future__ import annotations

import argparse
import pathlib
import sys
from dataclasses import dataclass

import numpy as np
import pandas as pd


WINDOW_METRICS = [
    "summary_result_count",
    "summary_detailed_profile_count",
    "summary_minimal_profile_count",
    "summary_weighted_match_ratio",
    "summary_weighted_inlier_ratio",
    "summary_total_pose_ms",
    "summary_total_corr_ms",
    "summary_total_accum_ms",
    "summary_total_factor_ms",
    "summary_total_kernel_pose_query_ms",
    "summary_total_kernel_correspondence_ms",
    "summary_total_kernel_residual_weight_ms",
    "summary_total_kernel_reduction_ms",
    "summary_total_host_sync_ms",
    "summary_total_host_result_pack_ms",
    "summary_mean_candidates_per_source",
    "summary_mean_time_bucket_population",
    "summary_max_time_bucket_population",
    "summary_max_numeric_rel_error",
    "summary_max_rotation_axis_rel_error",
]


CURRENT_FACTOR_METRICS = [
    "factor_error",
    "rmse",
    "inlier_fraction",
    "pose_ms",
    "correspondence_ms",
    "accumulation_ms",
    "total_ms",
    "kernel_pose_query_ms",
    "kernel_correspondence_ms",
    "kernel_residual_weight_ms",
    "kernel_reduction_ms",
    "host_sync_ms",
    "host_result_pack_ms",
    "numeric_rotation_rel_error",
    "numeric_translation_rel_error",
]


@dataclass(frozen=True)
class BaselineRun:
    label: str
    path: pathlib.Path


def parse_run_spec(spec: str) -> BaselineRun:
    if "=" in spec:
        label, raw_path = spec.split("=", 1)
    else:
        raw_path = spec
        label = pathlib.Path(raw_path).stem
    path = pathlib.Path(raw_path)
    if not path.exists():
        raise FileNotFoundError(f"Baseline CSV not found: {path}")
    return BaselineRun(label=label, path=path)


def summarize_metric(series: pd.Series) -> dict[str, float]:
    clean = pd.to_numeric(series, errors="coerce").dropna()
    if clean.empty:
        return {"mean": np.nan, "p50": np.nan, "p95": np.nan, "max": np.nan}
    return {
        "mean": float(clean.mean()),
        "p50": float(clean.quantile(0.50)),
        "p95": float(clean.quantile(0.95)),
        "max": float(clean.max()),
    }


def print_metric_block(title: str, df: pd.DataFrame, metrics: list[str]) -> None:
    print(f"\n{title}")
    print("metric,mean,p50,p95,max")
    for metric in metrics:
        if metric not in df.columns:
            continue
        stats = summarize_metric(df[metric])
        if np.isnan(stats["mean"]):
            continue
        print(
            f"{metric},{stats['mean']:.6f},{stats['p50']:.6f},{stats['p95']:.6f},{stats['max']:.6f}"
        )


def compare_against_reference(
    reference_label: str,
    reference_df: pd.DataFrame,
    current_label: str,
    current_df: pd.DataFrame,
    metrics: list[str],
) -> None:
    print(f"\nDelta vs {reference_label} -> {current_label}")
    print("metric,mean_delta,p50_delta,p95_delta,max_delta")
    for metric in metrics:
        if metric not in reference_df.columns or metric not in current_df.columns:
            continue
        ref = summarize_metric(reference_df[metric])
        cur = summarize_metric(current_df[metric])
        if np.isnan(ref["mean"]) or np.isnan(cur["mean"]):
            continue
        print(
            f"{metric},"
            f"{cur['mean'] - ref['mean']:.6f},"
            f"{cur['p50'] - ref['p50']:.6f},"
            f"{cur['p95'] - ref['p95']:.6f},"
            f"{cur['max'] - ref['max']:.6f}"
        )


def load_rows(path: pathlib.Path) -> tuple[pd.DataFrame, pd.DataFrame]:
    df = pd.read_csv(path)
    window_df = df[df["row_type"] == "window_summary"].copy()
    factor_df = df[(df["row_type"] == "factor_result") & (df["is_current"] == 1)].copy()
    return window_df, factor_df


def describe_run(run: BaselineRun, window_df: pd.DataFrame, factor_df: pd.DataFrame) -> None:
    backend = ",".join(sorted(str(v) for v in window_df["backend"].dropna().unique()))
    frontend = ",".join(sorted(str(v) for v in window_df["frontend_mode"].dropna().unique()))
    print(f"\n=== {run.label} ===")
    print(f"path: {run.path}")
    print(f"frontend: {frontend or 'n/a'}")
    print(f"backend: {backend or 'n/a'}")
    print(f"window_rows: {len(window_df)}")
    print(f"current_factor_rows: {len(factor_df)}")
    print_metric_block("window_summary_stats", window_df, WINDOW_METRICS)
    if not factor_df.empty:
        print_metric_block("current_factor_stats", factor_df, CURRENT_FACTOR_METRICS)


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare IAP CT LiDAR baseline CSV exports.")
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run specs in the form LABEL=/path/to/baseline.csv or just /path/to/baseline.csv",
    )
    args = parser.parse_args(argv)

    runs = [parse_run_spec(spec) for spec in args.runs]
    loaded: list[tuple[BaselineRun, pd.DataFrame, pd.DataFrame]] = []

    for run in runs:
        window_df, factor_df = load_rows(run.path)
        describe_run(run, window_df, factor_df)
        loaded.append((run, window_df, factor_df))

    if len(loaded) >= 2:
        ref_run, ref_window, ref_factor = loaded[0]
        for run, window_df, factor_df in loaded[1:]:
            compare_against_reference(ref_run.label, ref_window, run.label, window_df, WINDOW_METRICS)
            if not ref_factor.empty and not factor_df.empty:
                compare_against_reference(
                    ref_run.label,
                    ref_factor,
                    run.label,
                    factor_df,
                    CURRENT_FACTOR_METRICS,
                )

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
