#!/usr/bin/env python3
"""Analyze Predictor system experiment 2 artifacts and generate report figures."""

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


def resolve_dirs(args: argparse.Namespace) -> tuple[Path, Path, Path, Path, Path]:
    if args.run_dir:
        run_dir = Path(args.run_dir)
        export_dir = run_dir / "export"
    else:
        export_dir = Path(args.export_dir)
        run_dir = export_dir.parent
    profiling_dir = run_dir / "profiling"
    metadata_dir = run_dir / "metadata"
    figures_dir = run_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, export_dir, profiling_dir, metadata_dir, figures_dir


def values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [finite_float(row.get(key)) or math.nan for row in rows]


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


def load_min_median_primitives(metadata_dir: Path) -> int:
    path = metadata_dir / "predictor_probe_config.json"
    if not path.is_file():
        return 6
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError:
        return 6
    value = data.get("predictor_params", {}).get("lidar_fim_min_voxels")
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return 6
    return max(parsed, 1)


def plot_selected_pl(rows: list[dict[str, str]], figures_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    x = x_series(rows)
    ax.plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#2563eb")
    ax.plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#f97316")
    ax.plot(x, values(rows, "selected_pl"), label="selected PL", color="#111827", alpha=0.7)
    ax.set_title("Experiment 2 selected LiDAR-only advisory PL")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("protection level [m]")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E2_selected_pl_timeline.png")


def plot_lidar_diagnostics(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(rows, "lidar_n_primitives"), label="n primitives", color="#16a34a")
    axes[0].plot(x, values(rows, "lidar_n_valid_normals"), label="valid normals", color="#0891b2")
    axes[0].set_ylabel("count")
    axes[0].legend()
    axes[1].plot(x, values(rows, "lidar_lambda_trace"), label="lambda trace", color="#7c3aed")
    axes[1].set_ylabel("trace")
    axes[1].legend()
    axes[2].plot(x, values(rows, "lidar_lambda_condition"), label="condition", color="#dc2626")
    axes[2].set_yscale("log")
    axes[2].set_ylabel("condition")
    axes[2].set_xlabel("time since first query [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Experiment 2 LiDAR diagnostics")
    save(fig, figures_dir / "E2_lidar_diagnostics_timeline.png")


def plot_lidar_eigenvalues(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    min_eig = values(rows, "lidar_lambda_min_eig")
    condition = values(rows, "lidar_lambda_condition")
    max_eig = [
        a * b if math.isfinite(a) and math.isfinite(b) else math.nan
        for a, b in zip(min_eig, condition)
    ]
    trace = values(rows, "lidar_lambda_trace")
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, min_eig, label="lambda min eig", color="#2563eb")
    ax.plot(x, max_eig, label="lambda max eig estimate", color="#f97316", alpha=0.8)
    ax.plot(x, trace, label="lambda trace", color="#111827", alpha=0.6)
    ax.set_yscale("log")
    ax.set_title("Experiment 2 LiDAR FIM eigenvalue diagnostics")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("information")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E2_lidar_eigenvalues_timeline.png")


def plot_primitives_topdown(primitive_rows: list[dict[str, str]], figures_dir: Path) -> None:
    xs = finite_values(primitive_rows, "center_x")
    ys = finite_values(primitive_rows, "center_y")
    weights = finite_values(primitive_rows, "weight")
    fig, ax = plt.subplots(figsize=(7, 6))
    if xs and len(xs) == len(ys):
        colors = weights if len(weights) == len(xs) else None
        sc = ax.scatter(xs, ys, s=16, c=colors, cmap="viridis", alpha=0.75)
        if colors is not None:
            fig.colorbar(sc, ax=ax, label="primitive weight")
    ax.set_title("Experiment 2 LiDAR primitives top-down")
    ax.set_xlabel("center_x [m]")
    ax.set_ylabel("center_y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    save(fig, figures_dir / "E2_lidar_primitives_topdown.png")


def plot_normal_distribution(primitive_rows: list[dict[str, str]], figures_dir: Path) -> None:
    normals = {
        "normal_x": finite_values(primitive_rows, "normal_x"),
        "normal_y": finite_values(primitive_rows, "normal_y"),
        "normal_z": finite_values(primitive_rows, "normal_z"),
    }
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True)
    for ax, (name, data) in zip(axes, normals.items()):
        if data:
            ax.hist(data, bins=30, range=(-1.0, 1.0), color="#2563eb", alpha=0.75)
        ax.set_title(name)
        ax.set_xlabel("component")
        ax.grid(True, alpha=0.25)
    axes[0].set_ylabel("count")
    fig.suptitle("Experiment 2 LiDAR primitive normal distribution")
    save(fig, figures_dir / "E2_lidar_normal_distribution.png")


def reason_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        for key in ("selected_fallback_reason", "module_fallback_reason", "lidar_reason"):
            reason = (row.get(key) or "").strip()
            if reason:
                counter[reason] += 1
    return dict(sorted(counter.items()))


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    primitive_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    min_median_primitives: int,
    condition_threshold: float,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    valid_rows = [row for row in rows if row.get("selected_valid") == "1"]
    lidar_source_count = sum(1 for row in rows if row.get("selected_source") == "LIDAR")
    lidar_valid_count = sum(1 for row in rows if row.get("lidar_valid") == "1")
    lidar_fim_valid_count = sum(1 for row in rows if row.get("lidar_fim_valid") == "1")
    fallback_count = sum(1 for row in rows if row.get("selected_fallback") == "1")
    source_ratio = ratio(lidar_source_count, query_count)
    lidar_valid_ratio = ratio(lidar_valid_count, query_count)
    lidar_fim_valid_ratio = ratio(lidar_fim_valid_count, query_count)
    fallback_ratio = ratio(fallback_count, query_count)

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)

    n_primitives = finite_values(rows, "lidar_n_primitives")
    n_valid_normals = finite_values(rows, "lidar_n_valid_normals")
    lambda_trace = finite_values(rows, "lidar_lambda_trace")
    lambda_min = finite_values(rows, "lidar_lambda_min_eig")
    condition = finite_values(rows, "lidar_lambda_condition")
    median_primitives = median(n_primitives) if n_primitives else None
    median_valid_normals = median(n_valid_normals) if n_valid_normals else None
    median_trace = median(lambda_trace) if lambda_trace else None
    median_min = median(lambda_min) if lambda_min else None
    median_condition = median(condition) if condition else None

    nonfinite_valid = []
    for index, row in enumerate(valid_rows, start=1):
        for key in (
            "selected_hpl",
            "selected_vpl",
            "selected_pl",
            "lidar_lambda_trace",
            "lidar_lambda_min_eig",
            "lidar_lambda_condition",
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
        failures.append(f"selected_source_LIDAR_ratio {source_ratio:.3f} <= 0.80")
    if lidar_valid_ratio <= 0.80:
        failures.append(f"lidar_valid_ratio {lidar_valid_ratio:.3f} <= 0.80")
    if median_primitives is None or median_primitives < min_median_primitives:
        failures.append(
            f"median(lidar_n_primitives) {median_primitives} < {min_median_primitives}"
        )
    if median_condition is None or median_condition >= condition_threshold:
        failures.append(
            f"median(lidar_lambda_condition) {median_condition} >= {condition_threshold}"
        )
    if module_total_p95 is None or module_total_p95 >= 10000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 10000")
    if nonfinite_valid:
        failures.append(f"non-finite valid fields: {len(nonfinite_valid)}")
    if missing_or_empty:
        failures.append("missing or empty required files: " + ",".join(missing_or_empty))
    if not primitive_rows:
        failures.append("no lidar primitive rows found")

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "selected_source_LIDAR_ratio": source_ratio,
        "lidar_valid_ratio": lidar_valid_ratio,
        "lidar_fim_valid_ratio": lidar_fim_valid_ratio,
        "fallback_ratio": fallback_ratio,
        "median_lidar_lambda_trace": median_trace,
        "median_lidar_lambda_min_eig": median_min,
        "median_lidar_condition": median_condition,
        "median_lidar_n_primitives": median_primitives,
        "median_lidar_n_valid_normals": median_valid_normals,
        "min_median_primitives": min_median_primitives,
        "condition_threshold": condition_threshold,
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "fallback_reason_histogram": reason_histogram(rows),
        "primitive_row_count": len(primitive_rows),
        "query_labels": sorted({row.get("query_label", "") for row in rows}),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--condition-threshold", type=float, default=1.0e6)
    parser.add_argument("--min-median-primitives", type=int)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, metadata_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    primitives_path = export_dir / "predictor_lidar_primitives_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    map_snapshot_path = export_dir / "downsampled_map.csv"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    primitive_rows = read_csv(primitives_path)
    min_median_primitives = (
        args.min_median_primitives
        if args.min_median_primitives is not None
        else load_min_median_primitives(metadata_dir)
    )

    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "predictor_lidar_primitives_debug.csv": primitives_path,
        "fallback_reason_by_time.csv": fallback_path,
        "downsampled_map.csv": map_snapshot_path,
        "latency_debug.csv": timing_path,
    }
    summary = analyze(
        rows,
        timing_rows,
        primitive_rows,
        required_files,
        min_median_primitives,
        args.condition_threshold,
    )
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
        "lidar_debug_path": str(lidar_debug_path),
        "primitives_path": str(primitives_path),
        "fallback_path": str(fallback_path),
        "map_snapshot_path": str(map_snapshot_path),
    })

    figures: list[str] = []
    if rows:
        plot_selected_pl(rows, figures_dir)
        plot_lidar_diagnostics(rows, figures_dir)
        plot_lidar_eigenvalues(rows, figures_dir)
        figures.extend([
            str(figures_dir / "E2_selected_pl_timeline.png"),
            str(figures_dir / "E2_lidar_diagnostics_timeline.png"),
            str(figures_dir / "E2_lidar_eigenvalues_timeline.png"),
        ])
    if primitive_rows:
        plot_primitives_topdown(primitive_rows, figures_dir)
        plot_normal_distribution(primitive_rows, figures_dir)
        figures.extend([
            str(figures_dir / "E2_lidar_primitives_topdown.png"),
            str(figures_dir / "E2_lidar_normal_distribution.png"),
        ])
    summary["figures"] = figures

    out_path = export_dir / "predictor_e2_analysis_summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
