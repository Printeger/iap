#!/usr/bin/env python3
"""Analyze Predictor system experiment 12 query latency stress artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


LATENCY_FIELDS = ("gnss_us", "lidar_us", "fusion_us", "total_step_us", "module_total_us")


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


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def batch_value(row: dict[str, str]) -> int | None:
    try:
        value = int(str(row.get("query_batch_size", "")).strip())
    except ValueError:
        return None
    return value if value > 0 else None


def group_by_batch(rows: list[dict[str, str]]) -> dict[int, list[dict[str, str]]]:
    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        batch = batch_value(row)
        if batch is not None:
            grouped[batch].append(row)
    return dict(sorted(grouped.items()))


def bool_count(rows: list[dict[str, str]], key: str) -> int:
    return sum(1 for row in rows if is_true(row.get(key)))


def latency_values(rows: list[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def stats(values: list[float]) -> dict[str, float | None]:
    return {
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
    }


def save(fig, path: Path) -> Path:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


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


def parse_expected_batches(value: str) -> list[int]:
    out: list[int] = []
    for token in value.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            parsed = int(token)
        except ValueError:
            continue
        if parsed > 0:
            out.append(parsed)
    return sorted(set(out))


def nonfinite_latency_rows(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    bad: list[dict[str, object]] = []
    for i, row in enumerate(rows):
        for key in LATENCY_FIELDS:
            if finite_float(row.get(key)) is None:
                bad.append({
                    "row": i,
                    "query_index": row.get("query_index", ""),
                    "query_batch_size": row.get("query_batch_size", ""),
                    "field": key,
                    "value": row.get(key, ""),
                })
                break
    return bad


def summarize_batch(
    batch: int,
    rows: list[dict[str, str]],
    tick_rows: list[dict[str, str]],
) -> dict[str, object]:
    query_count = len(rows)
    source_hist = Counter((row.get("selected_source") or "").strip() for row in rows)
    selected_valid = bool_count(rows, "selected_valid")
    selected_fallback = bool_count(rows, "selected_fallback")
    gnss_valid = bool_count(rows, "gnss_valid")
    lidar_valid = bool_count(rows, "lidar_valid")
    fused_valid = bool_count(rows, "fused_valid")
    summary: dict[str, object] = {
        "query_batch_size": batch,
        "query_count": query_count,
        "selected_source_FUSION_ratio": ratio(source_hist.get("FUSION", 0), query_count),
        "selected_valid_ratio": ratio(selected_valid, query_count),
        "gnss_valid_ratio": ratio(gnss_valid, query_count),
        "lidar_valid_ratio": ratio(lidar_valid, query_count),
        "fused_valid_ratio": ratio(fused_valid, query_count),
        "fallback_ratio": ratio(selected_fallback, query_count),
        "source_histogram": dict(sorted(source_hist.items())),
    }
    for key in LATENCY_FIELDS:
        summary[key] = stats(latency_values(rows, key))
    batch_total_values = latency_values(tick_rows, "batch_total_us")
    summary["batch_total_us"] = stats(batch_total_values)
    summary["tick_count"] = len(tick_rows)
    return summary


def write_latency_stress_csv(path: Path, per_batch: dict[int, dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "query_batch_size",
        "query_count",
        "selected_source_FUSION_ratio",
        "selected_valid_ratio",
        "gnss_valid_ratio",
        "lidar_valid_ratio",
        "fused_valid_ratio",
        "fallback_ratio",
        "gnss_us_p50",
        "gnss_us_p95",
        "gnss_us_p99",
        "lidar_us_p50",
        "lidar_us_p95",
        "lidar_us_p99",
        "fusion_us_p50",
        "fusion_us_p95",
        "fusion_us_p99",
        "module_total_us_p50",
        "module_total_us_p95",
        "module_total_us_p99",
        "total_step_us_p50",
        "total_step_us_p95",
        "total_step_us_p99",
        "batch_total_us_p50",
        "batch_total_us_p95",
        "batch_total_us_p99",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for batch, summary in sorted(per_batch.items()):
            row: dict[str, object] = {
                "query_batch_size": batch,
                "query_count": summary["query_count"],
                "selected_source_FUSION_ratio": summary["selected_source_FUSION_ratio"],
                "selected_valid_ratio": summary["selected_valid_ratio"],
                "gnss_valid_ratio": summary["gnss_valid_ratio"],
                "lidar_valid_ratio": summary["lidar_valid_ratio"],
                "fused_valid_ratio": summary["fused_valid_ratio"],
                "fallback_ratio": summary["fallback_ratio"],
            }
            for key in ("gnss_us", "lidar_us", "fusion_us", "module_total_us", "total_step_us", "batch_total_us"):
                value = summary.get(key, {})
                assert isinstance(value, dict)
                row[f"{key}_p50"] = value.get("p50")
                row[f"{key}_p95"] = value.get("p95")
                row[f"{key}_p99"] = value.get("p99")
            writer.writerow(row)


def metric(per_batch: dict[int, dict[str, object]], batch: int, key: str, stat_key: str) -> float:
    value = per_batch.get(batch, {}).get(key, {})
    if isinstance(value, dict):
        number = value.get(stat_key)
        if isinstance(number, (int, float)) and math.isfinite(float(number)):
            return float(number)
    return math.nan


def plot_latency_vs_query_count(per_batch: dict[int, dict[str, object]], figures_dir: Path) -> Path:
    batches = sorted(per_batch)
    fig, ax = plt.subplots(figsize=(9, 5))
    for key, label in [
        ("gnss_us", "GNSS p95"),
        ("lidar_us", "LiDAR p95"),
        ("fusion_us", "Fusion p95"),
        ("module_total_us", "Module p95"),
    ]:
        ax.plot(batches, [metric(per_batch, b, key, "p95") for b in batches],
                marker="o", label=label)
    ax.set_xlabel("Query batch size")
    ax.set_ylabel("Latency per query [us]")
    ax.set_title("E12 latency vs query count")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return save(fig, figures_dir / "E12_latency_vs_query_count.png")


def plot_p95_p99_latency(per_batch: dict[int, dict[str, object]], figures_dir: Path) -> Path:
    batches = sorted(per_batch)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(batches, [metric(per_batch, b, "module_total_us", "p95") for b in batches],
            marker="o", label="Module p95")
    ax.plot(batches, [metric(per_batch, b, "module_total_us", "p99") for b in batches],
            marker="s", label="Module p99")
    ax.plot(batches, [metric(per_batch, b, "total_step_us", "p95") for b in batches],
            marker="^", label="Direct step p95")
    ax.set_xlabel("Query batch size")
    ax.set_ylabel("Latency per query [us]")
    ax.set_title("E12 p95/p99 latency")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return save(fig, figures_dir / "E12_p95_p99_latency.png")


def plot_batch_total_latency(per_batch: dict[int, dict[str, object]], figures_dir: Path) -> Path:
    batches = sorted(per_batch)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(batches, [metric(per_batch, b, "batch_total_us", "p50") for b in batches],
            marker="o", label="Batch p50")
    ax.plot(batches, [metric(per_batch, b, "batch_total_us", "p95") for b in batches],
            marker="s", label="Batch p95")
    ax.plot(batches, [metric(per_batch, b, "batch_total_us", "p99") for b in batches],
            marker="^", label="Batch p99")
    ax.set_xlabel("Query batch size")
    ax.set_ylabel("Batch total latency [us]")
    ax.set_title("E12 batch total latency")
    ax.grid(True, alpha=0.3)
    ax.legend()
    return save(fig, figures_dir / "E12_batch_total_latency.png")


def plot_source_validity_by_batch(per_batch: dict[int, dict[str, object]], figures_dir: Path) -> Path:
    batches = sorted(per_batch)
    fig, ax = plt.subplots(figsize=(9, 5))
    for key, label in [
        ("selected_source_FUSION_ratio", "FUSION source"),
        ("selected_valid_ratio", "Selected valid"),
        ("gnss_valid_ratio", "GNSS valid"),
        ("lidar_valid_ratio", "LiDAR valid"),
        ("fused_valid_ratio", "Fused valid"),
    ]:
        ax.plot(batches, [float(per_batch[b].get(key, math.nan)) for b in batches],
                marker="o", label=label)
    ax.set_xlabel("Query batch size")
    ax.set_ylabel("Ratio")
    ax.set_ylim(-0.02, 1.05)
    ax.set_title("E12 source validity by batch")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    return save(fig, figures_dir / "E12_source_validity_by_batch.png")


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    tick_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    expected_batches: list[int],
    module_p99_threshold_us: float,
    batch_p95_threshold_us: float,
    per_query_growth_factor: float,
) -> dict[str, object]:
    failures: list[str] = []
    missing = [
        name for name, path in required_files.items()
        if not path.is_file() or path.stat().st_size == 0
    ]
    failures.extend(f"missing or empty required file: {name}" for name in missing)

    grouped = group_by_batch(rows)
    tick_grouped = group_by_batch(tick_rows)
    observed_batches = sorted(grouped)
    missing_batches = [b for b in expected_batches if b not in grouped]
    if missing_batches:
        failures.append(f"missing expected batch sizes: {missing_batches}")

    if not rows:
        failures.append("no predictor rows found")
    if not timing_rows:
        failures.append("no latency rows found")
    if not tick_rows:
        failures.append("no latency stress tick rows found")

    bad_latency = nonfinite_latency_rows(rows)
    if bad_latency:
        failures.append(f"nonfinite latency rows: {len(bad_latency)}")

    per_batch: dict[int, dict[str, object]] = {}
    for batch in observed_batches:
        summary = summarize_batch(batch, grouped[batch], tick_grouped.get(batch, []))
        per_batch[batch] = summary
        if int(summary["query_count"]) <= 0:
            failures.append(f"batch {batch} has no query rows")
        if float(summary["selected_source_FUSION_ratio"]) <= 0.95:
            failures.append(
                f"batch {batch} selected_source_FUSION_ratio "
                f"{summary['selected_source_FUSION_ratio']:.3f} <= 0.95"
            )
        if float(summary["selected_valid_ratio"]) <= 0.95:
            failures.append(
                f"batch {batch} selected_valid_ratio {summary['selected_valid_ratio']:.3f} <= 0.95"
            )
        for key in ("gnss_valid_ratio", "lidar_valid_ratio", "fused_valid_ratio"):
            if float(summary[key]) <= 0.95:
                failures.append(f"batch {batch} {key} {summary[key]:.3f} <= 0.95")
        if float(summary["fallback_ratio"]) >= 0.05:
            failures.append(f"batch {batch} fallback_ratio {summary['fallback_ratio']:.3f} >= 0.05")
        module_stats = summary["module_total_us"]
        batch_stats = summary["batch_total_us"]
        if isinstance(module_stats, dict):
            p99 = module_stats.get("p99")
            if p99 is None or float(p99) >= module_p99_threshold_us:
                failures.append(f"batch {batch} module_total_us p99 {p99} >= {module_p99_threshold_us}")
        if isinstance(batch_stats, dict):
            p95 = batch_stats.get("p95")
            if p95 is None or float(p95) >= batch_p95_threshold_us:
                failures.append(f"batch {batch} batch_total_us p95 {p95} >= {batch_p95_threshold_us}")

    if 1 in per_batch and 100 in per_batch:
        base = metric(per_batch, 1, "module_total_us", "p95")
        stress = metric(per_batch, 100, "module_total_us", "p95")
        if not math.isfinite(base) or not math.isfinite(stress):
            failures.append("cannot evaluate per-query latency growth")
        elif stress > base * per_query_growth_factor:
            failures.append(
                f"batch 100 module_total_us p95 {stress:.3f} > "
                f"{per_query_growth_factor:.3f}x batch 1 p95 {base:.3f}"
            )

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": len(rows),
        "latency_row_count": len(timing_rows),
        "latency_stress_tick_row_count": len(tick_rows),
        "expected_batches": expected_batches,
        "observed_batches": observed_batches,
        "missing_batches": missing_batches,
        "per_batch": per_batch,
        "nonfinite_latency_rows": bad_latency[:50],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--expected-batches", default="1,10,50,100")
    parser.add_argument("--module-p99-threshold-us", type=float, default=10000.0)
    parser.add_argument("--batch-p95-threshold-us", type=float, default=200000.0)
    parser.add_argument("--per-query-growth-factor", type=float, default=3.0)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    gnss_visibility_path = export_dir / "gnss_visibility_by_query.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    fusion_debug_path = export_dir / "predictor_fusion_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    tick_path = export_dir / "latency_stress_tick_debug.csv"
    timing_path = profiling_dir / "latency_debug.csv"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    tick_rows = read_csv(tick_path)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_stress_tick_debug.csv": tick_path,
        "latency_debug.csv": timing_path,
    }
    expected_batches = parse_expected_batches(args.expected_batches)
    summary = analyze(
        rows,
        timing_rows,
        tick_rows,
        required_files,
        expected_batches,
        args.module_p99_threshold_us,
        args.batch_p95_threshold_us,
        args.per_query_growth_factor,
    )

    per_batch = summary["per_batch"]
    assert isinstance(per_batch, dict)
    write_latency_stress_csv(export_dir / "latency_stress.csv", per_batch)

    figures: list[str] = []
    if per_batch:
        figures.extend([
            str(plot_latency_vs_query_count(per_batch, figures_dir)),
            str(plot_p95_p99_latency(per_batch, figures_dir)),
            str(plot_batch_total_latency(per_batch, figures_dir)),
            str(plot_source_validity_by_batch(per_batch, figures_dir)),
        ])

    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
        "latency_stress_tick_path": str(tick_path),
        "latency_stress_csv_path": str(export_dir / "latency_stress.csv"),
        "figures": figures,
    })
    write_json(export_dir / "predictor_e12_analysis_summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
