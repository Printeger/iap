#!/usr/bin/env python3
"""Analyze Predictor system experiment 1 artifacts and generate report figures."""

from __future__ import annotations

import argparse
import csv
import json
import math
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


def x_series(rows: list[dict[str, str]]) -> list[float]:
    stamps = [finite_float(row.get("stamp")) for row in rows]
    finite = [v for v in stamps if v is not None]
    if not finite:
        return list(range(len(rows)))
    t0 = finite[0]
    return [(v - t0) if v is not None else float(i) for i, v in enumerate(stamps)]


def values(rows: list[dict[str, str]], key: str) -> list[float]:
    return [finite_float(row.get(key)) or math.nan for row in rows]


def save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_selected_pl(rows: list[dict[str, str]], figures_dir: Path) -> None:
    fig, ax = plt.subplots(figsize=(11, 5))
    x = x_series(rows)
    for label in sorted({row.get("query_label", "query") for row in rows}):
        subset = [row for row in rows if row.get("query_label", "query") == label]
        ax.plot(x_series(subset), values(subset, "selected_hpl"), linewidth=1.1, label=f"{label} HPL")
    ax.plot(x, values(rows, "selected_vpl"), color="#111827", linewidth=1.4, alpha=0.8, label="VPL")
    ax.set_title("Experiment 1 selected GNSS advisory PL")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("protection level [m]")
    ax.grid(True, alpha=0.25)
    ax.legend(ncol=2, fontsize=8)
    save(fig, figures_dir / "E1_selected_pl_timeline.png")


def plot_gnss_geometry(rows: list[dict[str, str]], figures_dir: Path) -> None:
    p0_rows = [row for row in rows if row.get("query_label") == "p0"] or rows
    x = x_series(p0_rows)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(p0_rows, "gnss_n_visible"), label="visible", color="#2563eb")
    axes[0].plot(x, values(p0_rows, "gnss_n_used"), label="used", color="#16a34a")
    axes[0].set_ylabel("satellites")
    axes[0].legend()
    axes[1].plot(x, values(p0_rows, "gnss_pdop"), label="PDOP", color="#7c3aed")
    axes[1].plot(x, values(p0_rows, "gnss_hdop"), label="HDOP", color="#0891b2")
    axes[1].plot(x, values(p0_rows, "gnss_vdop"), label="VDOP", color="#f97316")
    axes[1].set_ylabel("DOP")
    axes[1].legend()
    axes[2].plot(x, values(p0_rows, "effective_sigma_mean"), label="sigma mean", color="#475569")
    axes[2].plot(x, values(p0_rows, "effective_sigma_max"), label="sigma max", color="#dc2626")
    axes[2].set_ylabel("sigma [m]")
    axes[2].set_xlabel("time since first query [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Experiment 1 GNSS geometry diagnostics")
    save(fig, figures_dir / "E1_gnss_geometry_timeline.png")


def plot_current_vs_advisory(rows: list[dict[str, str]], figures_dir: Path) -> None:
    p0_rows = [row for row in rows if row.get("query_label") == "p0"] or rows
    x = x_series(p0_rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(p0_rows, "current_hpl"), label="current HPL", color="#64748b")
    axes[0].plot(x, values(p0_rows, "selected_hpl"), label="advisory HPL", color="#2563eb")
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(p0_rows, "current_vpl"), label="current VPL", color="#64748b")
    axes[1].plot(x, values(p0_rows, "selected_vpl"), label="advisory VPL", color="#f97316")
    axes[1].set_ylabel("VPL [m]")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 1 current ARAIM vs advisory Predictor")
    save(fig, figures_dir / "E1_current_vs_advisory.png")


def plot_latency(rows: list[dict[str, str]], figures_dir: Path) -> None:
    data = [v for v in values(rows, "module_total_us") if math.isfinite(v)]
    fig, ax = plt.subplots(figsize=(9, 5))
    if data:
        ax.hist(data, bins=min(40, max(8, int(math.sqrt(len(data))))), color="#2563eb", alpha=0.78)
        p95 = percentile(data, 0.95)
        if p95 is not None:
            ax.axvline(p95, color="#dc2626", linestyle="--", label=f"p95={p95:.1f} us")
            ax.legend()
    ax.set_title("Experiment 1 Predictor query latency")
    ax.set_xlabel("module_total_us")
    ax.set_ylabel("count")
    ax.grid(True, alpha=0.25)
    save(fig, figures_dir / "E1_latency_distribution.png")


def plot_spatial(rows: list[dict[str, str]], figures_dir: Path) -> None:
    labels = sorted({row.get("query_label", "query") for row in rows})
    fig, ax = plt.subplots(figsize=(7, 6))
    for label in labels:
        subset = [row for row in rows if row.get("query_label", "query") == label]
        xs = values(subset, "query_x")
        ys = values(subset, "query_y")
        color_values = values(subset, "selected_hpl")
        finite_points = [(x, y, c) for x, y, c in zip(xs, ys, color_values)
                         if math.isfinite(x) and math.isfinite(y)]
        if not finite_points:
            continue
        px, py, pc = zip(*finite_points)
        ax.scatter(px, py, s=18, alpha=0.75, label=label,
                   c=pc if any(math.isfinite(c) for c in pc) else None,
                   cmap="viridis")
    ax.set_title("Experiment 1 spatial query map")
    ax.set_xlabel("query_x [m]")
    ax.set_ylabel("query_y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8)
    save(fig, figures_dir / "E1_query_spatial_map.png")


def analyze(rows: list[dict[str, str]], timing_rows: list[dict[str, str]]) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    valid_rows = [row for row in rows if row.get("selected_valid") == "1"]
    gnss_source_count = sum(1 for row in rows if row.get("selected_source") == "GNSS")
    gnss_valid_count = sum(1 for row in rows if row.get("gnss_valid") == "1")
    fallback_count = sum(1 for row in rows if row.get("selected_fallback") == "1")

    latency_rows = timing_rows if timing_rows else rows
    module_total = [v for v in values(latency_rows, "module_total_us") if math.isfinite(v)]
    module_total_p95 = percentile(module_total, 0.95)

    nonfinite_valid = []
    for index, row in enumerate(valid_rows, start=1):
        for key in ("selected_hpl", "selected_vpl", "selected_pl", "gnss_hpl", "gnss_vpl"):
            if finite_float(row.get(key)) is None:
                nonfinite_valid.append({"row": index, "field": key})

    copied_current = 0
    for row in valid_rows:
        h_sel = finite_float(row.get("selected_hpl"))
        v_sel = finite_float(row.get("selected_vpl"))
        h_cur = finite_float(row.get("current_hpl"))
        v_cur = finite_float(row.get("current_vpl"))
        if None not in (h_sel, v_sel, h_cur, v_cur):
            if abs(h_sel - h_cur) < 1.0e-6 and abs(v_sel - v_cur) < 1.0e-6:
                copied_current += 1

    copied_current_ratio = ratio(copied_current, len(valid_rows))
    source_ratio = ratio(gnss_source_count, query_count)
    gnss_valid_ratio = ratio(gnss_valid_count, query_count)
    fallback_ratio = ratio(fallback_count, query_count)

    if query_count == 0:
        failures.append("no predictor rows found")
    if source_ratio <= 0.95:
        failures.append(f"selected_source_GNSS_ratio {source_ratio:.3f} <= 0.95")
    if gnss_valid_ratio <= 0.95:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} <= 0.95")
    if fallback_ratio >= 0.05:
        failures.append(f"fallback_ratio {fallback_ratio:.3f} >= 0.05")
    if module_total_p95 is None or module_total_p95 >= 2000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 2000")
    if nonfinite_valid:
        failures.append(f"non-finite valid fields: {len(nonfinite_valid)}")
    if copied_current_ratio >= 0.05:
        failures.append(f"copied_current_flag ratio {copied_current_ratio:.3f} >= 0.05")

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "selected_source_GNSS_ratio": source_ratio,
        "gnss_valid_ratio": gnss_valid_ratio,
        "fallback_ratio": fallback_ratio,
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "copied_current_flag_count": copied_current,
        "copied_current_flag_ratio": copied_current_ratio,
        "query_labels": sorted({row.get("query_label", "") for row in rows}),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)

    summary = analyze(rows, timing_rows)
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
    })

    if rows:
        plot_selected_pl(rows, figures_dir)
        plot_gnss_geometry(rows, figures_dir)
        plot_current_vs_advisory(rows, figures_dir)
        plot_latency(timing_rows if timing_rows else rows, figures_dir)
        plot_spatial(rows, figures_dir)
        summary["figures"] = [
            str(figures_dir / "E1_selected_pl_timeline.png"),
            str(figures_dir / "E1_gnss_geometry_timeline.png"),
            str(figures_dir / "E1_current_vs_advisory.png"),
            str(figures_dir / "E1_latency_distribution.png"),
            str(figures_dir / "E1_query_spatial_map.png"),
        ]
    else:
        summary["figures"] = []

    out_path = export_dir / "predictor_e1_analysis_summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
