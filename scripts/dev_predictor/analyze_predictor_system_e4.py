#!/usr/bin/env python3
"""Analyze Predictor system experiment 4 artifacts and generate report figures."""

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


DEFAULT_OPEN_SKY_RUN = Path(
    "/home/dev/ws_iap/src/iap/results/predictor_validation/"
    "predictor_system_predictor_gnss_open_sky_only_run1"
)


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


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


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
    converted: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        converted.append(value if value is not None else math.nan)
    return converted


def finite_values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [v for v in values(rows, key) if math.isfinite(v)]


def median_or_none(rows: list[dict[str, str]], key: str) -> float | None:
    vals = finite_values(rows, key)
    return median(vals) if vals else None


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


def source_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    return dict(sorted(Counter(row.get("selected_source", "") for row in rows).items()))


def reason_text(row: dict[str, str]) -> str:
    parts = []
    for prefix, key in (
        ("selected", "selected_fallback_reason"),
        ("module", "module_fallback_reason"),
        ("gnss", "gnss_reason"),
        ("lidar", "lidar_reason"),
        ("fused", "fused_reason"),
    ):
        value = (row.get(key) or "").strip()
        if value:
            parts.append(f"{prefix}:{value}")
    return ";".join(parts)


def reason_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        reason = reason_text(row)
        if reason:
            counter[reason] += 1
    return dict(sorted(counter.items()))


def load_open_sky_rows(args: argparse.Namespace) -> tuple[Path | None, list[dict[str, str]]]:
    candidates: list[Path] = []
    if args.open_sky_run_dir:
        candidates.append(Path(args.open_sky_run_dir))
    candidates.append(DEFAULT_OPEN_SKY_RUN)
    for run_dir in candidates:
        csv_path = run_dir / "export" / "test_predictor_query_probe.csv"
        rows = read_csv(csv_path)
        if rows:
            return run_dir, rows
    return (Path(args.open_sky_run_dir) if args.open_sky_run_dir else DEFAULT_OPEN_SKY_RUN), []


def region_label(row: dict[str, str], baseline: dict[str, float | None]) -> str:
    hpl = finite_float(row.get("gnss_hpl"))
    pdop = finite_float(row.get("gnss_pdop"))
    eff_sigma = finite_float(row.get("effective_sigma_mean"))
    hpl_base = baseline.get("median_gnss_hpl")
    pdop_base = baseline.get("median_gnss_pdop")
    eff_base = baseline.get("median_effective_sigma_mean")
    degraded_votes = 0
    if hpl is not None and hpl_base is not None and hpl > hpl_base:
        degraded_votes += 1
    if pdop is not None and pdop_base is not None and pdop > pdop_base:
        degraded_votes += 1
    if eff_sigma is not None and eff_base is not None and eff_sigma > eff_base:
        degraded_votes += 1
    if row.get("gnss_valid") != "1":
        degraded_votes += 1
    return "gnss_degraded" if degraded_votes else "open_sky_like"


def write_derived_outputs(
    rows: list[dict[str, str]],
    export_dir: Path,
    baseline: dict[str, float | None],
) -> tuple[Path, Path]:
    sat_rows: list[dict[str, object]] = []
    region_rows: list[dict[str, object]] = []
    for row in rows:
        label = region_label(row, baseline)
        sat_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "query_x": row.get("query_x", ""),
            "query_y": row.get("query_y", ""),
            "query_z": row.get("query_z", ""),
            "gnss_n_visible": row.get("gnss_n_visible", ""),
            "gnss_n_used": row.get("gnss_n_used", ""),
            "gnss_n_excluded": row.get("gnss_n_excluded", ""),
            "sat_visible_mask": row.get("sat_visible_mask", ""),
            "sat_used_mask": row.get("sat_used_mask", ""),
            "excluded_prn_mask": row.get("excluded_prn_mask", ""),
            "scenario_region_label": label,
        })
        region_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "query_x": row.get("query_x", ""),
            "query_y": row.get("query_y", ""),
            "query_z": row.get("query_z", ""),
            "scenario_region_label": label,
            "gnss_hpl": row.get("gnss_hpl", ""),
            "gnss_vpl": row.get("gnss_vpl", ""),
            "gnss_pdop": row.get("gnss_pdop", ""),
            "effective_sigma_mean": row.get("effective_sigma_mean", ""),
            "selected_source": row.get("selected_source", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
        })
    sat_path = export_dir / "satellite_used_mask.csv"
    region_path = export_dir / "scenario_region_labels.csv"
    write_csv(sat_path, [
        "stamp", "query_index", "query_label", "query_x", "query_y", "query_z",
        "gnss_n_visible", "gnss_n_used", "gnss_n_excluded", "sat_visible_mask",
        "sat_used_mask", "excluded_prn_mask", "scenario_region_label",
    ], sat_rows)
    write_csv(region_path, [
        "stamp", "query_index", "query_label", "query_x", "query_y", "query_z",
        "scenario_region_label", "gnss_hpl", "gnss_vpl", "gnss_pdop",
        "effective_sigma_mean", "selected_source", "selected_hpl", "selected_vpl",
    ], region_rows)
    return sat_path, region_path


def plot_gnss_degradation(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(rows, "gnss_n_visible"), label="visible", color="#2563eb")
    axes[0].plot(x, values(rows, "gnss_n_used"), label="used", color="#16a34a")
    axes[0].set_ylabel("satellites")
    axes[1].plot(x, values(rows, "gnss_pdop"), label="PDOP", color="#7c3aed")
    axes[1].plot(x, values(rows, "effective_sigma_mean"), label="effective sigma mean", color="#f97316")
    axes[1].set_ylabel("PDOP / sigma")
    axes[2].plot(x, values(rows, "gnss_hpl"), label="GNSS HPL", color="#dc2626")
    axes[2].plot(x, values(rows, "gnss_vpl"), label="GNSS VPL", color="#111827")
    axes[2].set_ylabel("PL [m]")
    axes[2].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 4 GNSS degradation timeline")
    save(fig, figures_dir / "E4_gnss_degradation_timeline.png")


def plot_lidar_support(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(rows, "lidar_n_primitives"), label="primitives", color="#16a34a")
    axes[0].plot(x, values(rows, "lidar_n_valid_normals"), label="valid normals", color="#2563eb")
    axes[0].set_ylabel("count")
    axes[1].plot(x, values(rows, "lidar_lambda_trace"), label="LiDAR trace", color="#7c3aed")
    axes[1].set_ylabel("trace(lambda)")
    axes[2].plot(x, values(rows, "lidar_lambda_condition"), label="LiDAR condition", color="#f97316")
    axes[2].set_yscale("log")
    axes[2].set_ylabel("condition")
    axes[2].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 4 LiDAR support timeline")
    save(fig, figures_dir / "E4_lidar_support_timeline.png")


def plot_gnss_vs_fusion(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "gnss_hpl"), label="GNSS HPL", color="#2563eb", alpha=0.8)
    axes[0].plot(x, values(rows, "fused_hpl"), label="Fusion HPL", color="#7c3aed", alpha=0.85)
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#111827", linewidth=1.5)
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(rows, "gnss_vpl"), label="GNSS VPL", color="#f97316", alpha=0.8)
    axes[1].plot(x, values(rows, "fused_vpl"), label="Fusion VPL", color="#7c3aed", alpha=0.85)
    axes[1].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#111827", linewidth=1.5)
    axes[1].set_ylabel("VPL [m]")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 4 GNSS vs Fusion HPL/VPL")
    save(fig, figures_dir / "E4_gnss_vs_fusion_hpl_vpl.png")


def plot_lambda_contribution(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, values(rows, "fused_lambda_prior_trace"), label="prior trace", color="#64748b")
    ax.plot(x, values(rows, "fused_lambda_gnss_trace"), label="GNSS trace", color="#2563eb")
    ax.plot(x, values(rows, "fused_lambda_lidar_trace"), label="LiDAR trace", color="#16a34a")
    ax.plot(x, values(rows, "fused_lambda_pred_trace"), label="pred trace", color="#7c3aed", linewidth=1.5)
    ax.set_title("Experiment 4 lambda contribution timeline")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("trace(lambda)")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E4_lambda_contribution_timeline.png")


def plot_spatial(rows: list[dict[str, str]], field: str, title: str, out_path: Path) -> None:
    xs = values(rows, "query_x")
    ys = values(rows, "query_y")
    colors = values(rows, field)
    fig, ax = plt.subplots(figsize=(7, 6))
    sc = ax.scatter(xs, ys, c=colors, s=18, cmap="viridis", alpha=0.85)
    ax.set_title(title)
    ax.set_xlabel("query_x [m]")
    ax.set_ylabel("query_y [m]")
    ax.grid(True, alpha=0.25)
    fig.colorbar(sc, ax=ax, label=field)
    save(fig, out_path)


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    open_sky_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    lambda_sum_error_p95_threshold: float,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    valid_rows = [row for row in rows if row.get("selected_valid") == "1"]
    fallback_rows = [row for row in rows if row.get("selected_fallback") == "1"]

    source_ratio = ratio(sum(1 for row in rows if row.get("selected_source") == "FUSION"), query_count)
    gnss_valid_ratio = ratio(sum(1 for row in rows if row.get("gnss_valid") == "1"), query_count)
    lidar_valid_ratio = ratio(sum(1 for row in rows if row.get("lidar_valid") == "1"), query_count)
    fused_valid_ratio = ratio(sum(1 for row in rows if row.get("fused_valid") == "1"), query_count)
    fallback_ratio = ratio(len(fallback_rows), query_count)

    baseline = {
        "median_gnss_hpl": median_or_none(open_sky_rows, "gnss_hpl"),
        "median_gnss_pdop": median_or_none(open_sky_rows, "gnss_pdop"),
        "median_effective_sigma_mean": median_or_none(open_sky_rows, "effective_sigma_mean"),
    }
    e4_median_gnss_hpl = median_or_none(rows, "gnss_hpl")
    e4_median_gnss_pdop = median_or_none(rows, "gnss_pdop")
    e4_median_effective_sigma_mean = median_or_none(rows, "effective_sigma_mean")

    lidar_conditions = finite_values(rows, "lidar_lambda_condition")
    median_lidar_condition = median(lidar_conditions) if lidar_conditions else None
    lambda_sum_error_p95 = percentile(finite_values(rows, "lambda_sum_error"), 0.95)

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)

    nonfinite_valid = []
    for index, row in enumerate(valid_rows, start=1):
        for key in (
            "selected_hpl", "selected_vpl", "selected_pl",
            "gnss_hpl", "gnss_vpl", "gnss_pdop",
            "lidar_lambda_trace", "lidar_lambda_min_eig", "lidar_lambda_condition",
            "fused_hpl", "fused_vpl", "fused_pl",
            "fused_lambda_pred_trace", "fused_lambda_pred_min_eig",
            "fused_lambda_pred_condition", "lambda_sum_error",
        ):
            if finite_float(row.get(key)) is None:
                nonfinite_valid.append({"row": index, "field": key})

    fallback_empty = [
        int(finite_float(row.get("query_index")) or index)
        for index, row in enumerate(fallback_rows, start=1)
        if not reason_text(row)
    ]

    missing_or_empty = [
        name for name, path in required_files.items()
        if not path.is_file() or path.stat().st_size == 0
    ]

    if query_count == 0:
        failures.append("no predictor rows found")
    if not open_sky_rows:
        failures.append("open-sky baseline rows missing")
    if source_ratio <= 0.70:
        failures.append(f"selected_source_FUSION_ratio {source_ratio:.3f} <= 0.70")
    if gnss_valid_ratio <= 0.70:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} <= 0.70")
    if lidar_valid_ratio <= 0.70:
        failures.append(f"lidar_valid_ratio {lidar_valid_ratio:.3f} <= 0.70")
    if fused_valid_ratio <= 0.70:
        failures.append(f"fused_valid_ratio {fused_valid_ratio:.3f} <= 0.70")
    if fallback_ratio >= 0.10:
        failures.append(f"fallback_ratio {fallback_ratio:.3f} >= 0.10")
    if (
        e4_median_gnss_hpl is None
        or baseline["median_gnss_hpl"] is None
        or e4_median_gnss_hpl <= baseline["median_gnss_hpl"]
    ):
        failures.append(
            "median gnss_hpl did not exceed open-sky baseline "
            f"({e4_median_gnss_hpl} <= {baseline['median_gnss_hpl']})"
        )
    if (
        e4_median_gnss_pdop is None
        or baseline["median_gnss_pdop"] is None
        or e4_median_gnss_pdop <= baseline["median_gnss_pdop"]
    ):
        failures.append(
            "median gnss_pdop did not exceed open-sky baseline "
            f"({e4_median_gnss_pdop} <= {baseline['median_gnss_pdop']})"
        )
    if median_lidar_condition is None or median_lidar_condition >= 1.0e6:
        failures.append(f"median lidar_lambda_condition {median_lidar_condition} >= 1000000")
    if lambda_sum_error_p95 is None or lambda_sum_error_p95 > lambda_sum_error_p95_threshold:
        failures.append(
            f"lambda_sum_error p95 {lambda_sum_error_p95} > {lambda_sum_error_p95_threshold}"
        )
    if module_total_p95 is None or module_total_p95 >= 10000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 10000")
    if nonfinite_valid:
        failures.append(f"non-finite valid fields: {len(nonfinite_valid)}")
    if fallback_empty:
        failures.append(f"fallback rows with empty reason: {len(fallback_empty)}")
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
        "median_gnss_hpl": e4_median_gnss_hpl,
        "median_open_sky_gnss_hpl": baseline["median_gnss_hpl"],
        "median_gnss_pdop": e4_median_gnss_pdop,
        "median_open_sky_gnss_pdop": baseline["median_gnss_pdop"],
        "median_effective_sigma_mean": e4_median_effective_sigma_mean,
        "median_open_sky_effective_sigma_mean": baseline["median_effective_sigma_mean"],
        "median_lidar_lambda_condition": median_lidar_condition,
        "median_lidar_lambda_trace": median_or_none(rows, "lidar_lambda_trace"),
        "median_lidar_n_primitives": median_or_none(rows, "lidar_n_primitives"),
        "lambda_sum_error_p95": lambda_sum_error_p95,
        "lambda_sum_error_p95_threshold": lambda_sum_error_p95_threshold,
        "median_lambda_prior_trace": median_or_none(rows, "fused_lambda_prior_trace"),
        "median_lambda_gnss_trace": median_or_none(rows, "fused_lambda_gnss_trace"),
        "median_lambda_lidar_trace": median_or_none(rows, "fused_lambda_lidar_trace"),
        "median_lambda_pred_trace": median_or_none(rows, "fused_lambda_pred_trace"),
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "source_histogram": source_histogram(rows),
        "fallback_reason_histogram": reason_histogram(rows),
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "fallback_empty_reason_query_indices": fallback_empty[:50],
        "query_labels": sorted({row.get("query_label", "") for row in rows}),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--open-sky-run-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--lambda-sum-error-p95-threshold", type=float, default=1.0e-6)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    open_sky_run_dir, open_sky_rows = load_open_sky_rows(args)

    csv_path = export_dir / "test_predictor_query_probe.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    fusion_debug_path = export_dir / "predictor_fusion_debug.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    gnss_visibility_path = export_dir / "gnss_visibility_by_query.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    lidar_primitives_path = export_dir / "predictor_lidar_primitives_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "predictor_lidar_primitives_debug.csv": lidar_primitives_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_debug.csv": timing_path,
    }

    baseline = {
        "median_gnss_hpl": median_or_none(open_sky_rows, "gnss_hpl"),
        "median_gnss_pdop": median_or_none(open_sky_rows, "gnss_pdop"),
        "median_effective_sigma_mean": median_or_none(open_sky_rows, "effective_sigma_mean"),
    }
    sat_path, region_path = write_derived_outputs(rows, export_dir, baseline)

    summary = analyze(
        rows,
        timing_rows,
        open_sky_rows,
        required_files,
        args.lambda_sum_error_p95_threshold,
    )
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "open_sky_run_dir": str(open_sky_run_dir) if open_sky_run_dir else None,
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
        "fusion_debug_path": str(fusion_debug_path),
        "source_selection_path": str(source_selection_path),
        "gnss_visibility_path": str(gnss_visibility_path),
        "lidar_debug_path": str(lidar_debug_path),
        "lidar_primitives_debug_path": str(lidar_primitives_path),
        "fallback_path": str(fallback_path),
        "satellite_used_mask_path": str(sat_path),
        "scenario_region_labels_path": str(region_path),
    })

    figures: list[str] = []
    if rows:
        plot_gnss_degradation(rows, figures_dir)
        plot_lidar_support(rows, figures_dir)
        plot_gnss_vs_fusion(rows, figures_dir)
        plot_lambda_contribution(rows, figures_dir)
        plot_spatial(
            rows,
            "gnss_hpl",
            "Experiment 4 spatial query map colored by GNSS HPL",
            figures_dir / "E4_spatial_query_map_colored_by_gnss_hpl.png",
        )
        plot_spatial(
            rows,
            "selected_hpl",
            "Experiment 4 spatial query map colored by selected HPL",
            figures_dir / "E4_spatial_query_map_colored_by_selected_hpl.png",
        )
        figures = [
            str(figures_dir / "E4_gnss_degradation_timeline.png"),
            str(figures_dir / "E4_lidar_support_timeline.png"),
            str(figures_dir / "E4_gnss_vs_fusion_hpl_vpl.png"),
            str(figures_dir / "E4_lambda_contribution_timeline.png"),
            str(figures_dir / "E4_spatial_query_map_colored_by_gnss_hpl.png"),
            str(figures_dir / "E4_spatial_query_map_colored_by_selected_hpl.png"),
        ]
    summary["figures"] = figures

    out_path = export_dir / "predictor_e4_analysis_summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
