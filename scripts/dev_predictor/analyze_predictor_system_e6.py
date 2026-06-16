#!/usr/bin/env python3
"""Analyze Predictor system experiment 6 artifacts and generate report figures."""

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


def read_json(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text())
    except json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


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


def first_stamp(rows: list[dict[str, str]]) -> float:
    stamps = [finite_float(row.get("stamp")) for row in rows]
    finite = [stamp for stamp in stamps if stamp is not None]
    return finite[0] if finite else 0.0


def event_time(row: dict[str, str], t0: float) -> float:
    stamp = finite_float(row.get("stamp"))
    if stamp is None:
        return math.nan
    # Sim-time runs use experiment seconds directly; host/epoch stamps need relative time.
    return stamp if abs(t0) < 1.0e6 else stamp - t0


def x_series(rows: list[dict[str, str]], t0: float) -> list[float]:
    return [event_time(row, t0) for row in rows]


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


def save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


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


def source_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    return dict(sorted(Counter(row.get("selected_source", "") for row in rows).items()))


def split_windows(
    rows: list[dict[str, str]],
    outage_start_s: float,
    outage_end_s: float,
    t0: float,
) -> tuple[list[dict[str, str]], list[dict[str, str]], list[dict[str, str]]]:
    before: list[dict[str, str]] = []
    outage: list[dict[str, str]] = []
    after: list[dict[str, str]] = []
    for row in rows:
        t = event_time(row, t0)
        if not math.isfinite(t):
            continue
        if t < outage_start_s:
            before.append(row)
        elif t <= outage_end_s:
            outage.append(row)
        else:
            after.append(row)
    return before, outage, after


def valid_selected(row: dict[str, str]) -> bool:
    return row.get("selected_valid") == "1"


def fallback_with_reason(row: dict[str, str]) -> bool:
    return row.get("selected_fallback") == "1" and bool(reason_text(row))


def invalid_or_explained_fallback(row: dict[str, str]) -> bool:
    source = (row.get("selected_source") or "").strip().upper()
    return (
        not valid_selected(row)
        or source in ("NONE", "INVALID")
        or fallback_with_reason(row)
    )


def first_time_matching(
    rows: list[dict[str, str]],
    t0: float,
    predicate,
) -> float | None:
    for row in rows:
        t = event_time(row, t0)
        if math.isfinite(t) and predicate(row):
            return t
    return None


def nonfinite_valid_fields(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    bad: list[dict[str, object]] = []
    for index, row in enumerate(rows, start=1):
        if not valid_selected(row):
            continue
        keys = ["selected_hpl", "selected_vpl", "selected_pl", "module_total_us"]
        if (row.get("selected_source") or "").upper() == "FUSION":
            keys.extend([
                "fused_hpl",
                "fused_vpl",
                "fused_pl",
                "fused_lambda_pred_trace",
                "fused_lambda_pred_min_eig",
                "fused_lambda_pred_condition",
                "lambda_sum_error",
            ])
        for key in keys:
            if finite_float(row.get(key)) is None:
                bad.append({"row": index, "query_index": row.get("query_index", ""), "field": key})
    return bad


def required_file_status(required_files: dict[str, Path]) -> dict[str, dict[str, object]]:
    status: dict[str, dict[str, object]] = {}
    for name, path in required_files.items():
        status[name] = {
            "path": str(path),
            "exists": path.is_file(),
            "size_bytes": path.stat().st_size if path.is_file() else 0,
        }
    return status


def required_missing_or_empty(status: dict[str, dict[str, object]]) -> list[str]:
    return [
        name for name, item in status.items()
        if not item["exists"] or int(item["size_bytes"]) <= 0
    ]


def draw_window(ax, outage_start_s: float, outage_end_s: float) -> None:
    ax.axvspan(outage_start_s, outage_end_s, color="#fee2e2", alpha=0.6)
    ax.axvline(outage_start_s, color="#dc2626", linestyle="--", linewidth=1.0)
    ax.axvline(outage_end_s, color="#16a34a", linestyle="--", linewidth=1.0)


def categorical_codes(values_in: list[str]) -> tuple[list[int], list[str]]:
    labels = sorted({value or "UNKNOWN" for value in values_in})
    index = {label: i for i, label in enumerate(labels)}
    return [index[value or "UNKNOWN"] for value in values_in], labels


def plot_outage_window(
    rows: list[dict[str, str]],
    figures_dir: Path,
    outage_start_s: float,
    outage_end_s: float,
    recovery_s: float | None,
    t0: float,
) -> None:
    x = x_series(rows, t0)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].step(x, values(rows, "gnss_valid"), where="post", label="gnss_valid", color="#2563eb")
    axes[0].step(x, values(rows, "selected_valid"), where="post", label="selected_valid", color="#111827")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel("valid")
    axes[1].plot(x, values(rows, "gnss_n_visible"), label="GNSS visible", color="#7c3aed")
    axes[1].plot(x, values(rows, "gnss_n_used"), label="GNSS used", color="#16a34a")
    axes[1].set_ylabel("satellites")
    axes[2].plot(x, values(rows, "module_total_us"), label="module_total_us", color="#f97316")
    axes[2].set_ylabel("latency [us]")
    axes[2].set_xlabel("experiment time [s]")
    for ax in axes:
        draw_window(ax, outage_start_s, outage_end_s)
        if recovery_s is not None:
            ax.axvline(recovery_s, color="#0891b2", linestyle=":", linewidth=1.4)
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 6 outage window timeline")
    save(fig, figures_dir / "E6_outage_window_timeline.png")


def plot_fallback_reason(
    rows: list[dict[str, str]],
    figures_dir: Path,
    outage_start_s: float,
    outage_end_s: float,
    t0: float,
) -> None:
    x = x_series(rows, t0)
    reasons = [reason_text(row) or "OK" for row in rows]
    codes, labels = categorical_codes(reasons)
    fig, ax = plt.subplots(figsize=(11, max(4, min(8, 0.35 * len(labels) + 2))))
    ax.scatter(x, codes, s=14, color="#dc2626", alpha=0.75)
    draw_window(ax, outage_start_s, outage_end_s)
    ax.set_yticks(list(range(len(labels))))
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel("experiment time [s]")
    ax.set_title("Experiment 6 fallback reason timeline")
    ax.grid(True, axis="x", alpha=0.25)
    save(fig, figures_dir / "E6_fallback_reason_timeline.png")


def plot_source_selection(
    rows: list[dict[str, str]],
    figures_dir: Path,
    outage_start_s: float,
    outage_end_s: float,
    t0: float,
) -> None:
    x = x_series(rows, t0)
    sources = [row.get("selected_source", "") or "UNKNOWN" for row in rows]
    codes, labels = categorical_codes(sources)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.step(x, codes, where="post", color="#2563eb", linewidth=1.5)
    draw_window(ax, outage_start_s, outage_end_s)
    ax.set_yticks(list(range(len(labels))))
    ax.set_yticklabels(labels)
    ax.set_xlabel("experiment time [s]")
    ax.set_title("Experiment 6 source selection timeline")
    ax.grid(True, alpha=0.25)
    save(fig, figures_dir / "E6_source_selection_timeline.png")


def plot_recovery_latency(
    metrics: dict[str, object],
    figures_dir: Path,
) -> None:
    labels = ["outage duration", "recovery delay"]
    outage_start = finite_float(metrics.get("gnss_outage_start_s"))
    outage_end = finite_float(metrics.get("gnss_outage_end_s"))
    recovery_delay = finite_float(metrics.get("recovery_delay_s"))
    values_out = [
        outage_end - outage_start if outage_start is not None and outage_end is not None else math.nan,
        recovery_delay if recovery_delay is not None else math.nan,
    ]
    fig, ax = plt.subplots(figsize=(8, 5))
    bars = ax.bar(labels, values_out, color=["#64748b", "#0891b2"])
    for bar, value in zip(bars, values_out):
        if math.isfinite(value):
            ax.text(bar.get_x() + bar.get_width() / 2.0, value, f"{value:.2f}s",
                    ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("seconds")
    ax.set_title("Experiment 6 recovery latency")
    ax.grid(True, axis="y", alpha=0.25)
    save(fig, figures_dir / "E6_recovery_latency.png")


def plot_gnss_visible(
    rows: list[dict[str, str]],
    figures_dir: Path,
    outage_start_s: float,
    outage_end_s: float,
    t0: float,
) -> None:
    x = x_series(rows, t0)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "gnss_n_visible"), label="visible", color="#2563eb")
    axes[0].plot(x, values(rows, "gnss_n_used"), label="used", color="#16a34a")
    axes[0].plot(x, values(rows, "gnss_n_excluded"), label="excluded", color="#dc2626")
    axes[0].set_ylabel("satellites")
    axes[1].plot(x, values(rows, "gnss_pdop"), label="PDOP", color="#7c3aed")
    axes[1].plot(x, values(rows, "effective_sigma_mean"), label="effective sigma mean", color="#f97316")
    axes[1].set_ylabel("DOP / sigma")
    axes[1].set_xlabel("experiment time [s]")
    for ax in axes:
        draw_window(ax, outage_start_s, outage_end_s)
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 6 GNSS visibility timeline")
    save(fig, figures_dir / "E6_gnss_n_visible_timeline.png")


def plot_selected_pl(
    rows: list[dict[str, str]],
    figures_dir: Path,
    outage_start_s: float,
    outage_end_s: float,
    t0: float,
) -> None:
    x = x_series(rows, t0)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#111827", linewidth=1.4)
    axes[0].plot(x, values(rows, "gnss_hpl"), label="GNSS HPL", color="#2563eb", alpha=0.75)
    axes[0].plot(x, values(rows, "fused_hpl"), label="fused HPL", color="#7c3aed", alpha=0.75)
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#111827", linewidth=1.4)
    axes[1].plot(x, values(rows, "gnss_vpl"), label="GNSS VPL", color="#f97316", alpha=0.75)
    axes[1].plot(x, values(rows, "fused_vpl"), label="fused VPL", color="#7c3aed", alpha=0.75)
    axes[1].set_ylabel("VPL [m]")
    axes[1].set_xlabel("experiment time [s]")
    for ax in axes:
        draw_window(ax, outage_start_s, outage_end_s)
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 6 selected PL timeline")
    save(fig, figures_dir / "E6_selected_pl_timeline.png")


def build_metrics(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    policy: str,
    outage_start_s: float,
    outage_end_s: float,
    module_total_p95_threshold_us: float,
) -> tuple[dict[str, object], dict[str, object], dict[str, object]]:
    failures: list[str] = []
    t0 = first_stamp(rows)
    before, outage, after = split_windows(rows, outage_start_s, outage_end_s, t0)
    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    nonfinite_valid = nonfinite_valid_fields(rows)
    file_status = required_file_status(required_files)
    missing_or_empty = required_missing_or_empty(file_status)

    before_valid_ratio = ratio(sum(1 for row in before if valid_selected(row)), len(before))
    outage_gnss_valid_ratio = ratio(sum(1 for row in outage if row.get("gnss_valid") == "1"), len(outage))
    outage_valid_ratio = ratio(sum(1 for row in outage if valid_selected(row)), len(outage))
    outage_safe_ratio = ratio(sum(1 for row in outage if invalid_or_explained_fallback(row)), len(outage))
    after_valid_ratio = ratio(sum(1 for row in after if valid_selected(row)), len(after))

    outage_rows_ordered = sorted(outage, key=lambda row: event_time(row, t0))
    after_rows_ordered = sorted(after, key=lambda row: event_time(row, t0))
    predictor_invalid_start_s = first_time_matching(
        outage_rows_ordered,
        t0,
        lambda row: not valid_selected(row) or (row.get("selected_source") or "").upper() in ("NONE", "INVALID"),
    )
    predictor_valid_recovery_s = first_time_matching(after_rows_ordered, t0, valid_selected)
    recovery_delay_s = (
        predictor_valid_recovery_s - outage_end_s
        if predictor_valid_recovery_s is not None else None
    )

    bad_outage_rows = [
        {
            "query_index": row.get("query_index", ""),
            "stamp": row.get("stamp", ""),
            "event_time_s": event_time(row, t0),
            "selected_source": row.get("selected_source", ""),
            "selected_valid": row.get("selected_valid", ""),
            "selected_fallback": row.get("selected_fallback", ""),
            "reason": reason_text(row),
        }
        for row in outage
        if not invalid_or_explained_fallback(row)
    ]

    if not rows:
        failures.append("no predictor rows found")
    if len(before) < 3:
        failures.append(f"too few pre-outage rows: {len(before)} < 3")
    if len(outage) < 3:
        failures.append(f"too few outage rows: {len(outage)} < 3")
    if len(after) < 3:
        failures.append(f"too few post-recovery rows: {len(after)} < 3")
    if before and before_valid_ratio < 0.50:
        failures.append(f"pre-outage selected valid ratio {before_valid_ratio:.3f} < 0.50")
    if outage and outage_gnss_valid_ratio > 0.20:
        failures.append(f"outage gnss_valid ratio {outage_gnss_valid_ratio:.3f} > 0.20")
    if policy == "gnss_required" and outage and outage_safe_ratio < 0.95:
        failures.append(
            f"outage invalid/fallback-with-reason ratio {outage_safe_ratio:.3f} < 0.95"
        )
    if after and after_valid_ratio < 0.50:
        failures.append(f"post-recovery selected valid ratio {after_valid_ratio:.3f} < 0.50")
    if module_total_p95 is None or module_total_p95 >= module_total_p95_threshold_us:
        failures.append(
            f"module_total_us p95 {module_total_p95} >= {module_total_p95_threshold_us}"
        )
    if nonfinite_valid:
        failures.append(f"non-finite fields in valid selected rows: {len(nonfinite_valid)}")
    if missing_or_empty:
        failures.append("missing or empty required files: " + ",".join(missing_or_empty))

    outage_event_times = {
        "time_basis": "raw_stamp_when_first_stamp_abs_lt_1e6_else_stamp_minus_first_stamp",
        "first_stamp_s": t0,
        "gnss_outage_start_s": outage_start_s,
        "gnss_outage_end_s": outage_end_s,
        "predictor_invalid_start_s": predictor_invalid_start_s,
        "predictor_valid_recovery_s": predictor_valid_recovery_s,
        "recovery_delay_s": recovery_delay_s,
    }
    recovery_metrics = {
        "gnss_outage_start_s": outage_start_s,
        "gnss_outage_end_s": outage_end_s,
        "predictor_invalid_start_s": predictor_invalid_start_s,
        "predictor_valid_recovery_s": predictor_valid_recovery_s,
        "recovery_delay_s": recovery_delay_s,
        "outage_valid_ratio": outage_valid_ratio,
        "fallback_reason_histogram": reason_histogram(rows),
    }
    summary = {
        "passed": not failures,
        "failures": failures,
        "policy": policy,
        "query_count": len(rows),
        "window_counts": {
            "before": len(before),
            "outage": len(outage),
            "after": len(after),
        },
        "ratios": {
            "before_selected_valid_ratio": before_valid_ratio,
            "outage_gnss_valid_ratio": outage_gnss_valid_ratio,
            "outage_valid_ratio": outage_valid_ratio,
            "outage_invalid_or_explained_fallback_ratio": outage_safe_ratio,
            "after_selected_valid_ratio": after_valid_ratio,
        },
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
            "threshold_p95_us": module_total_p95_threshold_us,
        },
        "median_selected_hpl": median_or_none(rows, "selected_hpl"),
        "median_selected_vpl": median_or_none(rows, "selected_vpl"),
        "median_gnss_n_visible_outage": median_or_none(outage, "gnss_n_visible"),
        "median_gnss_n_visible_before": median_or_none(before, "gnss_n_visible"),
        "median_gnss_n_visible_after": median_or_none(after, "gnss_n_visible"),
        "source_histogram": source_histogram(rows),
        "outage_source_histogram": source_histogram(outage),
        "fallback_reason_histogram": reason_histogram(rows),
        "outage_bad_rows": bad_outage_rows[:50],
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "required_file_status": file_status,
        "outage_event_times": outage_event_times,
        "recovery_metrics": recovery_metrics,
    }
    return summary, outage_event_times, recovery_metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--policy", choices=["gnss_required"], default="gnss_required")
    parser.add_argument("--outage-start-s", type=float, default=35.0)
    parser.add_argument("--outage-end-s", type=float, default=60.0)
    parser.add_argument("--module-total-p95-threshold-us", type=float, default=15000.0)
    parser.add_argument("--fail-on-threshold", action="store_true")
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, metadata_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    gnss_visibility_path = export_dir / "gnss_visibility_by_query.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    manifest_path = metadata_dir / "run_manifest.json"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    manifest = read_json(manifest_path)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "fallback_reason_by_time.csv": fallback_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "latency_debug.csv": timing_path,
        "run_manifest.json": manifest_path,
    }

    summary, outage_event_times, recovery_metrics = build_metrics(
        rows,
        timing_rows,
        required_files,
        args.policy,
        args.outage_start_s,
        args.outage_end_s,
        args.module_total_p95_threshold_us,
    )
    t0 = first_stamp(rows)
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "profiling_dir": str(profiling_dir),
        "metadata_dir": str(metadata_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "fallback_path": str(fallback_path),
        "source_selection_path": str(source_selection_path),
        "gnss_visibility_path": str(gnss_visibility_path),
        "lidar_debug_path": str(lidar_debug_path),
        "timing_path": str(timing_path),
        "manifest_path": str(manifest_path),
        "manifest": manifest,
    })

    figures: list[str] = []
    if rows:
        recovery_s = finite_float(recovery_metrics.get("predictor_valid_recovery_s"))
        plot_outage_window(rows, figures_dir, args.outage_start_s, args.outage_end_s, recovery_s, t0)
        plot_fallback_reason(rows, figures_dir, args.outage_start_s, args.outage_end_s, t0)
        plot_source_selection(rows, figures_dir, args.outage_start_s, args.outage_end_s, t0)
        plot_recovery_latency(recovery_metrics, figures_dir)
        plot_gnss_visible(rows, figures_dir, args.outage_start_s, args.outage_end_s, t0)
        plot_selected_pl(rows, figures_dir, args.outage_start_s, args.outage_end_s, t0)
        figures = [
            str(figures_dir / "E6_outage_window_timeline.png"),
            str(figures_dir / "E6_fallback_reason_timeline.png"),
            str(figures_dir / "E6_source_selection_timeline.png"),
            str(figures_dir / "E6_recovery_latency.png"),
            str(figures_dir / "E6_gnss_n_visible_timeline.png"),
            str(figures_dir / "E6_selected_pl_timeline.png"),
        ]
    summary["figures"] = figures

    write_json(export_dir / "outage_event_times.json", outage_event_times)
    write_json(export_dir / "recovery_metrics.json", recovery_metrics)
    write_json(export_dir / "predictor_e6_analysis_summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
