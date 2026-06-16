#!/usr/bin/env python3
"""Analyze Predictor system experiment 9 GNSS sigma sweep artifacts."""

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


REQUIRED_SCALES = (1.0, 2.0, 4.0)
VALID_FIELDS = (
    "effective_sigma_mean",
    "effective_sigma_max",
    "gnss_hpl",
    "gnss_vpl",
    "selected_hpl",
    "selected_vpl",
    "gnss_lambda_trace",
    "gnss_pdop",
)


def finite_float(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def is_true(value: object) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


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


def write_json(path: Path, data: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


def percentile(values: list[float], q: float) -> float | None:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    pos = max(0.0, min(1.0, q)) * (len(clean) - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return clean[lo]
    weight = pos - lo
    return clean[lo] * (1.0 - weight) + clean[hi] * weight


def finite_values(rows: list[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def median_value(rows: list[dict[str, str]], key: str) -> float | None:
    values = finite_values(rows, key)
    return median(values) if values else None


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def parse_scale_run(raw: str) -> tuple[float, Path]:
    if ":" not in raw:
        raise argparse.ArgumentTypeError(
            f"--scale-run must be '<scale>:<run_dir>', got '{raw}'"
        )
    scale_text, run_dir_text = raw.split(":", 1)
    scale = finite_float(scale_text)
    if scale is None or scale <= 0.0:
        raise argparse.ArgumentTypeError(f"invalid positive scale in '{raw}'")
    run_dir = Path(run_dir_text)
    return scale, run_dir


def scale_key(scale: float) -> str:
    if abs(scale - round(scale)) < 1.0e-9:
        return str(int(round(scale)))
    return f"{scale:g}"


def required_missing_or_empty(run_dir: Path) -> list[str]:
    required = [
        run_dir / "export" / "test_predictor_query_probe.csv",
        run_dir / "export" / "source_selection_debug.csv",
        run_dir / "export" / "gnss_epoch_debug.csv",
        run_dir / "export" / "gnss_visibility_by_query.csv",
        run_dir / "export" / "fallback_reason_by_time.csv",
        run_dir / "profiling" / "latency_debug.csv",
        run_dir / "metadata" / "run_manifest.json",
        run_dir / "metadata" / "predictor_launch_config.json",
        run_dir / "metadata" / "predictor_probe_config.json",
    ]
    return [str(path) for path in required if not path.is_file() or path.stat().st_size == 0]


def run_stats(scale: float, run_dir: Path) -> dict[str, object]:
    export_dir = run_dir / "export"
    profiling_dir = run_dir / "profiling"
    rows = read_csv(export_dir / "test_predictor_query_probe.csv")
    timing_rows = read_csv(profiling_dir / "latency_debug.csv")
    query_count = len(rows)
    valid_rows = [
        row for row in rows
        if is_true(row.get("selected_valid")) and is_true(row.get("gnss_valid"))
    ]
    selected_source_gnss_count = sum(
        1 for row in rows if row.get("selected_source") == "GNSS"
    )
    gnss_valid_count = sum(1 for row in rows if is_true(row.get("gnss_valid")))
    fallback_count = sum(1 for row in rows if is_true(row.get("selected_fallback")))
    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    nonfinite_valid_fields: list[dict[str, object]] = []
    for index, row in enumerate(valid_rows, start=1):
        for field in VALID_FIELDS:
            if finite_float(row.get(field)) is None:
                nonfinite_valid_fields.append({
                    "row": index,
                    "query_index": row.get("query_index", ""),
                    "field": field,
                })

    return {
        "sigma_scale": scale,
        "run_dir": str(run_dir),
        "csv_path": str(export_dir / "test_predictor_query_probe.csv"),
        "query_count": query_count,
        "selected_source_GNSS_ratio": ratio(selected_source_gnss_count, query_count),
        "gnss_valid_ratio": ratio(gnss_valid_count, query_count),
        "fallback_ratio": ratio(fallback_count, query_count),
        "valid_row_count": len(valid_rows),
        "nonfinite_valid_fields": nonfinite_valid_fields[:50],
        "required_files_missing_or_empty": required_missing_or_empty(run_dir),
        "median_effective_sigma_mean": median_value(valid_rows, "effective_sigma_mean"),
        "median_effective_sigma_max": median_value(valid_rows, "effective_sigma_max"),
        "median_gnss_hpl": median_value(valid_rows, "gnss_hpl"),
        "median_gnss_vpl": median_value(valid_rows, "gnss_vpl"),
        "median_selected_hpl": median_value(valid_rows, "selected_hpl"),
        "median_selected_vpl": median_value(valid_rows, "selected_vpl"),
        "median_gnss_pdop": median_value(valid_rows, "gnss_pdop"),
        "median_gnss_lambda_trace": median_value(valid_rows, "gnss_lambda_trace"),
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": percentile(module_total, 0.95),
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
    }


def strictly_increasing(values: list[float | None]) -> bool:
    if any(value is None for value in values):
        return False
    return all(float(a) < float(b) for a, b in zip(values, values[1:]))


def strictly_decreasing(values: list[float | None]) -> bool:
    if any(value is None for value in values):
        return False
    return all(float(a) > float(b) for a, b in zip(values, values[1:]))


def line_plot(
    summary_rows: list[dict[str, object]],
    figures_dir: Path,
    filename: str,
    title: str,
    ylabel: str,
    series: list[tuple[str, str]],
) -> Path:
    x = [float(row["sigma_scale"]) for row in summary_rows]
    fig, ax = plt.subplots(figsize=(9, 5))
    for field, label in series:
        y = [row.get(field) for row in summary_rows]
        ax.plot(x, y, marker="o", linewidth=1.8, label=label)
    ax.set_title(title)
    ax.set_xlabel("sigma scale")
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.tight_layout()
    path = figures_dir / filename
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def write_outputs(
    output_run_dir: Path,
    stats_by_scale: list[dict[str, object]],
) -> tuple[Path, list[str]]:
    export_dir = output_run_dir / "export"
    figures_dir = output_run_dir / "figures"
    export_dir.mkdir(parents=True, exist_ok=True)
    figures_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = []
    for stat in stats_by_scale:
        summary_rows.append({
            "sigma_scale": stat["sigma_scale"],
            "run_dir": stat["run_dir"],
            "query_count": stat["query_count"],
            "selected_source_GNSS_ratio": stat["selected_source_GNSS_ratio"],
            "gnss_valid_ratio": stat["gnss_valid_ratio"],
            "fallback_ratio": stat["fallback_ratio"],
            "effective_sigma_mean": stat["median_effective_sigma_mean"],
            "effective_sigma_max": stat["median_effective_sigma_max"],
            "gnss_hpl": stat["median_gnss_hpl"],
            "gnss_vpl": stat["median_gnss_vpl"],
            "selected_hpl": stat["median_selected_hpl"],
            "selected_vpl": stat["median_selected_vpl"],
            "gnss_pdop": stat["median_gnss_pdop"],
            "gnss_lambda_trace": stat["median_gnss_lambda_trace"],
            "module_total_us_p95": stat["module_total_us"]["p95"],
        })
    csv_path = export_dir / "gnss_sigma_sweep_system.csv"
    write_csv(csv_path, [
        "sigma_scale",
        "run_dir",
        "query_count",
        "selected_source_GNSS_ratio",
        "gnss_valid_ratio",
        "fallback_ratio",
        "effective_sigma_mean",
        "effective_sigma_max",
        "gnss_hpl",
        "gnss_vpl",
        "selected_hpl",
        "selected_vpl",
        "gnss_pdop",
        "gnss_lambda_trace",
        "module_total_us_p95",
    ], summary_rows)
    figures = [
        line_plot(
            summary_rows,
            figures_dir,
            "E9_sigma_vs_pl.png",
            "Experiment 9 GNSS sigma scale vs protection level",
            "protection level [m]",
            [
                ("gnss_hpl", "GNSS HPL"),
                ("gnss_vpl", "GNSS VPL"),
                ("selected_hpl", "selected HPL"),
                ("selected_vpl", "selected VPL"),
            ],
        ),
        line_plot(
            summary_rows,
            figures_dir,
            "E9_sigma_vs_lambda_trace.png",
            "Experiment 9 GNSS sigma scale vs FIM trace",
            "lambda_gnss trace",
            [("gnss_lambda_trace", "GNSS lambda trace")],
        ),
        line_plot(
            summary_rows,
            figures_dir,
            "E9_effective_sigma_by_scale.png",
            "Experiment 9 effective pseudorange sigma",
            "effective sigma [m]",
            [
                ("effective_sigma_mean", "effective sigma mean"),
                ("effective_sigma_max", "effective sigma max"),
            ],
        ),
        line_plot(
            summary_rows,
            figures_dir,
            "E9_latency_by_scale.png",
            "Experiment 9 Predictor query latency",
            "module_total_us p95",
            [("module_total_us_p95", "p95")],
        ),
    ]
    return csv_path, [str(path) for path in figures]


def analyze(scale_runs: list[tuple[float, Path]], output_run_dir: Path) -> dict[str, object]:
    failures: list[str] = []
    unique: dict[float, Path] = {}
    for scale, run_dir in scale_runs:
        if scale in unique:
            failures.append(f"duplicate sigma scale {scale_key(scale)}")
        unique[scale] = run_dir
    missing_scales = [
        scale_key(scale) for scale in REQUIRED_SCALES
        if scale not in unique
    ]
    if missing_scales:
        failures.append("missing required scales: " + ",".join(missing_scales))

    stats = [run_stats(scale, run_dir) for scale, run_dir in sorted(unique.items())]
    by_scale = {float(stat["sigma_scale"]): stat for stat in stats}
    required_stats = [by_scale.get(scale) for scale in REQUIRED_SCALES]
    required_stats = [stat for stat in required_stats if stat is not None]

    for stat in stats:
        scale = scale_key(float(stat["sigma_scale"]))
        if stat["query_count"] <= 0:
            failures.append(f"scale {scale}: no predictor rows found")
        if stat["selected_source_GNSS_ratio"] <= 0.95:
            failures.append(
                f"scale {scale}: selected_source_GNSS_ratio "
                f"{stat['selected_source_GNSS_ratio']:.3f} <= 0.95"
            )
        if stat["gnss_valid_ratio"] <= 0.95:
            failures.append(
                f"scale {scale}: gnss_valid_ratio {stat['gnss_valid_ratio']:.3f} <= 0.95"
            )
        if stat["fallback_ratio"] >= 0.05:
            failures.append(
                f"scale {scale}: fallback_ratio {stat['fallback_ratio']:.3f} >= 0.05"
            )
        if stat["nonfinite_valid_fields"]:
            failures.append(
                f"scale {scale}: non-finite valid fields "
                f"{len(stat['nonfinite_valid_fields'])}"
            )
        module_total_p95 = stat["module_total_us"]["p95"]
        if module_total_p95 is None or module_total_p95 >= 2000.0:
            failures.append(f"scale {scale}: module_total_us p95 {module_total_p95} >= 2000")
        if stat["required_files_missing_or_empty"]:
            failures.append(
                f"scale {scale}: missing or empty required files "
                + ",".join(stat["required_files_missing_or_empty"])
            )

    monotonic = {
        "effective_sigma_mean_increasing": strictly_increasing([
            stat.get("median_effective_sigma_mean") for stat in required_stats
        ]),
        "effective_sigma_max_increasing": strictly_increasing([
            stat.get("median_effective_sigma_max") for stat in required_stats
        ]),
        "gnss_hpl_increasing": strictly_increasing([
            stat.get("median_gnss_hpl") for stat in required_stats
        ]),
        "gnss_vpl_increasing": strictly_increasing([
            stat.get("median_gnss_vpl") for stat in required_stats
        ]),
        "selected_hpl_increasing": strictly_increasing([
            stat.get("median_selected_hpl") for stat in required_stats
        ]),
        "selected_vpl_increasing": strictly_increasing([
            stat.get("median_selected_vpl") for stat in required_stats
        ]),
        "gnss_lambda_trace_decreasing": strictly_decreasing([
            stat.get("median_gnss_lambda_trace") for stat in required_stats
        ]),
    }
    for key, passed in monotonic.items():
        if not passed:
            failures.append(f"monotonic check failed: {key}")

    csv_path, figures = write_outputs(output_run_dir, stats)
    return {
        "passed": not failures,
        "failures": failures,
        "scale_count": len(stats),
        "scales": [scale_key(float(stat["sigma_scale"])) for stat in stats],
        "required_scales": [scale_key(scale) for scale in REQUIRED_SCALES],
        "per_scale": {scale_key(float(stat["sigma_scale"])): stat for stat in stats},
        "monotonic_checks": monotonic,
        "output_run_dir": str(output_run_dir),
        "export_dir": str(output_run_dir / "export"),
        "figures_dir": str(output_run_dir / "figures"),
        "gnss_sigma_sweep_system_csv": str(csv_path),
        "figures": figures,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scale-run",
        action="append",
        type=parse_scale_run,
        required=True,
        help="Sigma scale and run dir formatted as '<scale>:<run_dir>'. Repeat for 1/2/4.",
    )
    parser.add_argument("--output-run-dir", required=True)
    parser.add_argument("--fail-on-threshold", action="store_true")
    args = parser.parse_args()

    output_run_dir = Path(args.output_run_dir)
    summary = analyze(args.scale_run, output_run_dir)
    out_path = output_run_dir / "export" / "predictor_e9_analysis_summary.json"
    write_json(out_path, summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
