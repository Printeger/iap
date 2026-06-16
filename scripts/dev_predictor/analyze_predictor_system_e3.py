#!/usr/bin/env python3
"""Analyze Predictor system experiment 3 artifacts and generate report figures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def finite_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def percentile(values: list[float], q: float) -> float | None:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    pos = max(0.0, min(1.0, q)) * (len(clean) - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return clean[lo]
    w = pos - lo
    return clean[lo] * (1.0 - w) + clean[hi] * w


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def resolve_dirs(args: argparse.Namespace) -> tuple[Path, Path, Path, Path]:
    if args.run_dir:
        run_dir = Path(args.run_dir)
        export_dir = run_dir / "export"
    else:
        export_dir = Path(args.export_dir)
        run_dir = export_dir.parent
    profiling_dir = run_dir / "profiling"
    figures_dir = run_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, export_dir, profiling_dir, figures_dir


def values(rows: list[dict[str, str]], key: str) -> list[float]:
    converted = []
    for row in rows:
        value = finite_float(row.get(key))
        converted.append(value if value is not None else math.nan)
    return converted


def finite_values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [v for v in values(rows, key) if math.isfinite(v)]


def x_series(rows: list[dict[str, str]]) -> list[float]:
    stamps = [finite_float(row.get("stamp")) for row in rows]
    finite = [v for v in stamps if v is not None]
    if not finite:
        return list(range(len(rows)))
    t0 = finite[0]
    return [(v - t0) if v is not None else float(i) for i, v in enumerate(stamps)]


def save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def reason_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        for key in ("selected_fallback_reason", "module_fallback_reason", "gnss_reason", "lidar_reason", "fused_reason"):
            reason = (row.get(key) or "").strip()
            if reason:
                counter[reason] += 1
    return dict(sorted(counter.items()))


def source_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    return dict(sorted(Counter(row.get("selected_source", "") for row in rows).items()))


def plot_source_hpl_vpl(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#111827", linewidth=1.5)
    axes[0].plot(x, values(rows, "fused_hpl"), label="fused HPL", color="#7c3aed", alpha=0.85)
    axes[0].plot(x, values(rows, "gnss_hpl"), label="GNSS HPL", color="#2563eb", alpha=0.75)
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#111827", linewidth=1.5)
    axes[1].plot(x, values(rows, "fused_vpl"), label="fused VPL", color="#7c3aed", alpha=0.85)
    axes[1].plot(x, values(rows, "gnss_vpl"), label="GNSS VPL", color="#f97316", alpha=0.75)
    axes[1].set_ylabel("VPL [m]")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 3 source HPL/VPL timeline")
    save(fig, figures_dir / "E3_source_hpl_vpl_timeline.png")


def plot_lambda_contribution(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, values(rows, "fused_lambda_prior_trace"), label="prior trace", color="#64748b")
    ax.plot(x, values(rows, "fused_lambda_gnss_trace"), label="GNSS trace", color="#2563eb")
    ax.plot(x, values(rows, "fused_lambda_lidar_trace"), label="LiDAR trace", color="#16a34a")
    ax.plot(x, values(rows, "fused_lambda_pred_trace"), label="pred trace", color="#7c3aed", linewidth=1.5)
    ax.set_title("Experiment 3 lambda contribution timeline")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("trace(lambda)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E3_lambda_contribution_timeline.png")


def plot_lambda_sum_error(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    data = values(rows, "lambda_sum_error")
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, data, color="#dc2626", label="lambda_sum_error")
    finite = [v for v in data if math.isfinite(v) and v > 0.0]
    if finite:
        ax.set_yscale("log")
    ax.set_title("Experiment 3 lambda sum error")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("max abs error")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E3_lambda_sum_error_timeline.png")


def plot_lambda_condition(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, values(rows, "fused_lambda_pred_condition"), label="fused condition", color="#7c3aed")
    ax.plot(x, values(rows, "gnss_lambda_condition"), label="GNSS condition", color="#2563eb", alpha=0.75)
    ax.plot(x, values(rows, "lidar_lambda_condition"), label="LiDAR condition", color="#16a34a", alpha=0.75)
    ax.set_yscale("log")
    ax.set_title("Experiment 3 lambda condition timeline")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("condition number")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E3_lambda_condition_timeline.png")


def plot_source_selection_histogram(rows: list[dict[str, str]], figures_dir: Path) -> None:
    counts = source_histogram(rows)
    fig, ax = plt.subplots(figsize=(7, 5))
    if counts:
        ax.bar(list(counts.keys()), list(counts.values()), color="#7c3aed", alpha=0.8)
    ax.set_title("Experiment 3 source selection histogram")
    ax.set_xlabel("selected source")
    ax.set_ylabel("rows")
    ax.grid(True, axis="y", alpha=0.25)
    save(fig, figures_dir / "E3_source_selection_histogram.png")


def plot_latency(rows: list[dict[str, str]], figures_dir: Path) -> None:
    data = finite_values(rows, "module_total_us")
    fig, ax = plt.subplots(figsize=(9, 5))
    if data:
        ax.hist(data, bins=min(40, max(8, int(math.sqrt(len(data))))), color="#7c3aed", alpha=0.78)
        p95 = percentile(data, 0.95)
        if p95 is not None:
            ax.axvline(p95, color="#dc2626", linestyle="--", label=f"p95={p95:.1f} us")
            ax.legend()
    ax.set_title("Experiment 3 Predictor fusion query latency")
    ax.set_xlabel("module_total_us")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.25)
    save(fig, figures_dir / "E3_latency_distribution.png")


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    lambda_sum_error_p95_threshold: float,
    lambda_min_eig_threshold: float,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    valid_rows = [row for row in rows if row.get("selected_valid") == "1"]
    fusion_source_count = sum(1 for row in rows if row.get("selected_source") == "FUSION")
    gnss_valid_count = sum(1 for row in rows if row.get("gnss_valid") == "1")
    lidar_valid_count = sum(1 for row in rows if row.get("lidar_valid") == "1")
    fused_valid_count = sum(1 for row in rows if row.get("fused_valid") == "1")
    fallback_count = sum(1 for row in rows if row.get("selected_fallback") == "1")

    source_ratio = ratio(fusion_source_count, query_count)
    gnss_valid_ratio = ratio(gnss_valid_count, query_count)
    lidar_valid_ratio = ratio(lidar_valid_count, query_count)
    fused_valid_ratio = ratio(fused_valid_count, query_count)
    fallback_ratio = ratio(fallback_count, query_count)

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)

    lambda_sum_errors = finite_values(rows, "lambda_sum_error")
    lambda_sum_error_p95 = percentile(lambda_sum_errors, 0.95)
    fused_min_eigs = finite_values(rows, "fused_lambda_pred_min_eig")
    fused_min_eig_min = min(fused_min_eigs) if fused_min_eigs else None
    fused_conditions = finite_values(rows, "fused_lambda_pred_condition")

    nonfinite_valid = []
    for index, row in enumerate(valid_rows, start=1):
        for key in (
            "selected_hpl",
            "selected_vpl",
            "selected_pl",
            "fused_hpl",
            "fused_vpl",
            "fused_lambda_pred_trace",
            "fused_lambda_pred_min_eig",
            "fused_lambda_pred_condition",
            "lambda_sum_error",
        ):
            if finite_float(row.get(key)) is None:
                nonfinite_valid.append({"row": index, "field": key})

    missing_or_empty = [
        name for name, path in required_files.items()
        if not path.is_file() or path.stat().st_size == 0
    ]

    if query_count == 0:
        failures.append("no predictor rows found")
    if source_ratio <= 0.80:
        failures.append(f"selected_source_FUSION_ratio {source_ratio:.3f} <= 0.80")
    if gnss_valid_ratio <= 0.80:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} <= 0.80")
    if lidar_valid_ratio <= 0.80:
        failures.append(f"lidar_valid_ratio {lidar_valid_ratio:.3f} <= 0.80")
    if fused_valid_ratio <= 0.80:
        failures.append(f"fused_valid_ratio {fused_valid_ratio:.3f} <= 0.80")
    if fallback_ratio >= 0.10:
        failures.append(f"fallback_ratio {fallback_ratio:.3f} >= 0.10")
    if lambda_sum_error_p95 is None or lambda_sum_error_p95 > lambda_sum_error_p95_threshold:
        failures.append(
            f"lambda_sum_error p95 {lambda_sum_error_p95} > {lambda_sum_error_p95_threshold}"
        )
    if fused_min_eig_min is None or fused_min_eig_min <= lambda_min_eig_threshold:
        failures.append(
            f"fused_lambda_pred_min_eig min {fused_min_eig_min} <= {lambda_min_eig_threshold}"
        )
    if module_total_p95 is None or module_total_p95 >= 10000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 10000")
    if nonfinite_valid:
        failures.append(f"non-finite valid fields: {len(nonfinite_valid)}")
    if missing_or_empty:
        failures.append("missing or empty required files: " + ",".join(missing_or_empty))

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "selected_source_FUSION_ratio": source_ratio,
        "gnss_valid_ratio": gnss_valid_ratio,
        "lidar_valid_ratio": lidar_valid_ratio,
        "fused_valid_ratio": fused_valid_ratio,
        "fallback_ratio": fallback_ratio,
        "lambda_sum_error_p95": lambda_sum_error_p95,
        "lambda_sum_error_p95_threshold": lambda_sum_error_p95_threshold,
        "fused_lambda_pred_min_eig_min": fused_min_eig_min,
        "lambda_min_eig_threshold": lambda_min_eig_threshold,
        "median_lambda_prior_trace": median(finite_values(rows, "fused_lambda_prior_trace")) if finite_values(rows, "fused_lambda_prior_trace") else None,
        "median_lambda_gnss_trace": median(finite_values(rows, "fused_lambda_gnss_trace")) if finite_values(rows, "fused_lambda_gnss_trace") else None,
        "median_lambda_lidar_trace": median(finite_values(rows, "fused_lambda_lidar_trace")) if finite_values(rows, "fused_lambda_lidar_trace") else None,
        "median_lambda_pred_trace": median(finite_values(rows, "fused_lambda_pred_trace")) if finite_values(rows, "fused_lambda_pred_trace") else None,
        "median_fused_lambda_condition": median(fused_conditions) if fused_conditions else None,
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "source_histogram": source_histogram(rows),
        "fallback_reason_histogram": reason_histogram(rows),
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "query_labels": sorted({row.get("query_label", "") for row in rows}),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--lambda-sum-error-p95-threshold", type=float, default=1.0e-6)
    parser.add_argument("--lambda-min-eig-threshold", type=float, default=-1.0e-9)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    fusion_debug_path = export_dir / "predictor_fusion_debug.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    gnss_visibility_path = export_dir / "gnss_visibility_by_query.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_debug.csv": timing_path,
    }

    summary = analyze(
        rows,
        timing_rows,
        required_files,
        args.lambda_sum_error_p95_threshold,
        args.lambda_min_eig_threshold,
    )
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
        "fusion_debug_path": str(fusion_debug_path),
        "source_selection_path": str(source_selection_path),
        "gnss_visibility_path": str(gnss_visibility_path),
        "lidar_debug_path": str(lidar_debug_path),
        "fallback_path": str(fallback_path),
    })

    figures: list[str] = []
    if rows:
        plot_source_hpl_vpl(rows, figures_dir)
        plot_lambda_contribution(rows, figures_dir)
        plot_lambda_sum_error(rows, figures_dir)
        plot_lambda_condition(rows, figures_dir)
        plot_source_selection_histogram(rows, figures_dir)
        plot_latency(timing_rows if timing_rows else rows, figures_dir)
        figures = [
            str(figures_dir / "E3_source_hpl_vpl_timeline.png"),
            str(figures_dir / "E3_lambda_contribution_timeline.png"),
            str(figures_dir / "E3_lambda_sum_error_timeline.png"),
            str(figures_dir / "E3_lambda_condition_timeline.png"),
            str(figures_dir / "E3_source_selection_histogram.png"),
            str(figures_dir / "E3_latency_distribution.png"),
        ]
    summary["figures"] = figures

    out_path = export_dir / "predictor_e3_analysis_summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
