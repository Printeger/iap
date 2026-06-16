#!/usr/bin/env python3
"""Analyze Predictor system experiment 7 artifacts and generate report figures."""

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
    converted: list[float] = []
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


def is_true(row: dict[str, str], key: str) -> bool:
    return str(row.get(key, "")).strip() == "1"


def reason_text(row: dict[str, str]) -> str:
    parts = []
    for prefix, keys in (
        ("selected", ("selected_fallback_reason", "selected_reason")),
        ("module", ("module_fallback_reason", "module_reason")),
        ("gnss", ("gnss_reason",)),
        ("lidar", ("lidar_reason",)),
        ("fused", ("fused_reason",)),
    ):
        for key in keys:
            value = (row.get(key) or "").strip()
            if value:
                parts.append(f"{prefix}:{value}")
                break
    return ";".join(parts)


def reason_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        reason = reason_text(row)
        if reason:
            counter[reason] += 1
    return dict(sorted(counter.items()))


def source_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    return dict(sorted(Counter((row.get("selected_source") or "").strip() for row in rows).items()))


def selected_pl_is_finite(row: dict[str, str]) -> bool:
    return any(
        finite_float(row.get(key)) is not None
        for key in ("selected_hpl", "selected_vpl", "selected_pl")
    )


def copied_current(row: dict[str, str], tolerance_m: float) -> bool:
    comparisons = (
        ("selected_hpl", "current_hpl"),
        ("selected_vpl", "current_vpl"),
    )
    for selected_key, current_key in comparisons:
        selected = finite_float(row.get(selected_key))
        current = finite_float(row.get(current_key))
        if selected is not None and current is not None and abs(selected - current) <= tolerance_m:
            return True
    return False


def current_selected_delta(row: dict[str, str], selected_key: str, current_key: str) -> float | str:
    selected = finite_float(row.get(selected_key))
    current = finite_float(row.get(current_key))
    if selected is None or current is None:
        return ""
    return selected - current


def write_negative_case_rows(
    rows: list[dict[str, str]],
    export_dir: Path,
    copy_tolerance_m: float,
) -> tuple[Path, list[dict[str, object]]]:
    out_rows: list[dict[str, object]] = []
    for row in rows:
        out_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "selected_source": row.get("selected_source", ""),
            "selected_available": row.get("selected_available", ""),
            "selected_valid": row.get("selected_valid", ""),
            "selected_fallback": row.get("selected_fallback", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "selected_pl": row.get("selected_pl", ""),
            "selected_fallback_reason": row.get("selected_fallback_reason", ""),
            "module_available": row.get("module_available", ""),
            "module_valid": row.get("module_valid", ""),
            "module_fallback": row.get("module_fallback", ""),
            "module_fallback_reason": row.get("module_fallback_reason", ""),
            "gnss_valid": row.get("gnss_valid", ""),
            "gnss_reason": row.get("gnss_reason", ""),
            "lidar_valid": row.get("lidar_valid", ""),
            "lidar_reason": row.get("lidar_reason", ""),
            "fused_valid": row.get("fused_valid", ""),
            "fused_reason": row.get("fused_reason", ""),
            "has_pose": row.get("has_pose", ""),
            "has_epoch": row.get("has_epoch", ""),
            "has_lambda_base": row.get("has_lambda_base", ""),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "current_pl_e": row.get("current_pl_e", ""),
            "current_pl_n": row.get("current_pl_n", ""),
            "current_pl_u": row.get("current_pl_u", ""),
            "finite_selected_pl": int(selected_pl_is_finite(row)),
            "copied_current_flag": int(copied_current(row, copy_tolerance_m)),
            "reason_text": reason_text(row),
        })
    fieldnames = [
        "stamp", "query_index", "query_label", "selected_source",
        "selected_available", "selected_valid", "selected_fallback",
        "selected_hpl", "selected_vpl", "selected_pl", "selected_fallback_reason",
        "module_available", "module_valid", "module_fallback", "module_fallback_reason",
        "gnss_valid", "gnss_reason", "lidar_valid", "lidar_reason",
        "fused_valid", "fused_reason", "has_pose", "has_epoch", "has_lambda_base",
        "current_hpl", "current_vpl", "current_pl_e", "current_pl_n", "current_pl_u",
        "finite_selected_pl", "copied_current_flag", "reason_text",
    ]
    path = export_dir / "negative_case_rows.csv"
    write_csv(path, fieldnames, out_rows)
    return path, out_rows


def write_snapshot_debug(
    rows: list[dict[str, str]],
    export_dir: Path,
) -> tuple[Path, list[dict[str, object]]]:
    out_rows: list[dict[str, object]] = []
    for row in rows:
        out_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "has_pose": row.get("has_pose", ""),
            "has_epoch": row.get("has_epoch", ""),
            "has_lambda_base": row.get("has_lambda_base", ""),
            "odom_stamp_s": row.get("odom_stamp_s", ""),
            "integrity_stamp_s": row.get("integrity_stamp_s", ""),
            "gnss_epoch_stamp_s": row.get("gnss_epoch_stamp_s", ""),
            "snapshot_stamp_s": row.get("snapshot_stamp_s", ""),
            "odom_age_s": row.get("odom_age_s", ""),
            "integrity_age_s": row.get("integrity_age_s", ""),
            "gnss_age_s": row.get("gnss_age_s", ""),
            "snapshot_age_s": row.get("snapshot_age_s", ""),
            "current_state": row.get("current_state", ""),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "current_pdop": row.get("current_pdop", ""),
            "current_n_sv_used": row.get("current_n_sv_used", ""),
        })
    fieldnames = [
        "stamp", "query_index", "query_label", "has_pose", "has_epoch",
        "has_lambda_base", "odom_stamp_s", "integrity_stamp_s",
        "gnss_epoch_stamp_s", "snapshot_stamp_s", "odom_age_s",
        "integrity_age_s", "gnss_age_s", "snapshot_age_s", "current_state",
        "current_hpl", "current_vpl", "current_pdop", "current_n_sv_used",
    ]
    path = export_dir / "snapshot_debug.csv"
    write_csv(path, fieldnames, out_rows)
    return path, out_rows


def write_current_vs_advisory_debug(
    rows: list[dict[str, str]],
    export_dir: Path,
    copy_tolerance_m: float,
) -> tuple[Path, list[dict[str, object]]]:
    out_rows: list[dict[str, object]] = []
    for row in rows:
        out_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "selected_source": row.get("selected_source", ""),
            "selected_valid": row.get("selected_valid", ""),
            "selected_fallback": row.get("selected_fallback", ""),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "current_pl_e": row.get("current_pl_e", ""),
            "current_pl_n": row.get("current_pl_n", ""),
            "current_pl_u": row.get("current_pl_u", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "selected_pl": row.get("selected_pl", ""),
            "selected_minus_current_hpl": current_selected_delta(
                row, "selected_hpl", "current_hpl"
            ),
            "selected_minus_current_vpl": current_selected_delta(
                row, "selected_vpl", "current_vpl"
            ),
            "finite_selected_pl": int(selected_pl_is_finite(row)),
            "copied_current_flag": int(copied_current(row, copy_tolerance_m)),
            "selected_fallback_reason": row.get("selected_fallback_reason", ""),
            "module_fallback_reason": row.get("module_fallback_reason", ""),
        })
    fieldnames = [
        "stamp", "query_index", "query_label", "selected_source",
        "selected_valid", "selected_fallback", "current_hpl", "current_vpl",
        "current_pl_e", "current_pl_n", "current_pl_u", "selected_hpl",
        "selected_vpl", "selected_pl", "selected_minus_current_hpl",
        "selected_minus_current_vpl", "finite_selected_pl",
        "copied_current_flag", "selected_fallback_reason",
        "module_fallback_reason",
    ]
    path = export_dir / "current_vs_advisory_debug.csv"
    write_csv(path, fieldnames, out_rows)
    return path, out_rows


def categorical_codes(values_in: list[str]) -> tuple[list[int], list[str]]:
    labels = sorted({value or "UNKNOWN" for value in values_in})
    index = {label: i for i, label in enumerate(labels)}
    return [index[value or "UNKNOWN"] for value in values_in], labels


def plot_valid_fallback_flags(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(x, values(rows, "selected_valid"), where="post", label="selected valid", color="#dc2626")
    axes[0].step(x, values(rows, "selected_fallback"), where="post", label="selected fallback", color="#2563eb")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel("selected flag")
    axes[1].step(x, values(rows, "gnss_valid"), where="post", label="GNSS valid", color="#16a34a")
    axes[1].step(x, values(rows, "lidar_valid"), where="post", label="LiDAR valid", color="#f97316")
    axes[1].step(x, values(rows, "fused_valid"), where="post", label="fused valid", color="#7c3aed")
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_ylabel("source flag")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 7 valid/fallback flags")
    save(fig, figures_dir / "E7_valid_fallback_flags.png")


def plot_fallback_reason_histogram(rows: list[dict[str, str]], figures_dir: Path) -> None:
    hist = reason_histogram(rows)
    labels = list(hist.keys()) or ["NO_REASON"]
    counts = list(hist.values()) or [0]
    fig, ax = plt.subplots(figsize=(11, max(4.0, 0.35 * len(labels) + 2.0)))
    y = list(range(len(labels)))
    ax.barh(y, counts, color="#2563eb")
    ax.set_yticks(y)
    ax.set_yticklabels(labels)
    ax.set_xlabel("rows")
    ax.set_title("Experiment 7 fallback reason histogram")
    ax.grid(True, axis="x", alpha=0.25)
    save(fig, figures_dir / "E7_fallback_reason_histogram.png")


def plot_selected_pl_timeline(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    finite_flags = [1.0 if selected_pl_is_finite(row) else 0.0 for row in rows]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#2563eb")
    axes[0].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#f97316")
    axes[0].plot(x, values(rows, "selected_pl"), label="selected PL", color="#111827")
    axes[0].set_ylabel("selected PL [m]")
    axes[1].step(x, finite_flags, where="post", label="finite selected PL present", color="#dc2626")
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_ylabel("finite flag")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 7 selected PL timeline")
    save(fig, figures_dir / "E7_selected_pl_timeline.png")


def plot_current_vs_selected(
    rows: list[dict[str, str]],
    figures_dir: Path,
    copy_tolerance_m: float,
) -> None:
    x = x_series(rows)
    copied_flags = [
        1.0 if copied_current(row, copy_tolerance_m) else 0.0 for row in rows
    ]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(rows, "current_hpl"), label="current HPL", color="#2563eb")
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#dc2626")
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(rows, "current_vpl"), label="current VPL", color="#16a34a")
    axes[1].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#f97316")
    axes[1].set_ylabel("VPL [m]")
    axes[2].step(x, copied_flags, where="post", label="copied current flag", color="#7c3aed")
    axes[2].set_ylim(-0.1, 1.1)
    axes[2].set_ylabel("copy flag")
    axes[2].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 7 current vs selected PL")
    save(fig, figures_dir / "E7_current_vs_selected_pl.png")


def plot_source_selection(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    codes, labels = categorical_codes([
        (row.get("selected_source") or "").strip() for row in rows
    ])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(x, codes, where="post", label="selected source", color="#2563eb")
    axes[0].set_yticks(list(range(len(labels))))
    axes[0].set_yticklabels(labels)
    axes[0].set_ylabel("source")
    axes[1].step(x, values(rows, "module_valid"), where="post", label="module valid", color="#16a34a")
    axes[1].step(x, values(rows, "module_fallback"), where="post", label="module fallback", color="#f97316")
    axes[1].set_ylim(-0.1, 1.1)
    axes[1].set_ylabel("module flag")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 7 source selection timeline")
    save(fig, figures_dir / "E7_source_selection_timeline.png")


def required_missing_or_empty(required_files: dict[str, Path]) -> list[str]:
    return [
        name for name, path in required_files.items()
        if not path.is_file() or path.stat().st_size <= 0
    ]


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    copy_tolerance_m: float,
    module_total_p95_threshold_us: float,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    invalid_rows = [row for row in rows if not is_true(row, "selected_valid")]
    fallback_rows = [row for row in rows if is_true(row, "selected_fallback")]
    selected_valid_ratio = ratio(query_count - len(invalid_rows), query_count)
    selected_fallback_ratio = ratio(len(fallback_rows), query_count)
    gnss_valid_ratio = ratio(sum(1 for row in rows if is_true(row, "gnss_valid")), query_count)
    lidar_valid_ratio = ratio(sum(1 for row in rows if is_true(row, "lidar_valid")), query_count)
    fused_valid_ratio = ratio(sum(1 for row in rows if is_true(row, "fused_valid")), query_count)
    module_valid_ratio = ratio(sum(1 for row in rows if is_true(row, "module_valid")), query_count)
    module_fallback_ratio = ratio(sum(1 for row in rows if is_true(row, "module_fallback")), query_count)
    finite_invalid_rows = [
        {
            "row": index,
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "selected_pl": row.get("selected_pl", ""),
        }
        for index, row in enumerate(rows, start=1)
        if not is_true(row, "selected_valid") and selected_pl_is_finite(row)
    ]
    copied_rows = [
        {
            "row": index,
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
        }
        for index, row in enumerate(rows, start=1)
        if copied_current(row, copy_tolerance_m)
    ]
    fallback_empty_rows = [
        {
            "row": index,
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
        }
        for index, row in enumerate(rows, start=1)
        if is_true(row, "selected_fallback") and not reason_text(row)
    ]
    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    missing_or_empty = required_missing_or_empty(required_files)

    if query_count == 0:
        failures.append("no predictor rows found")
    if selected_valid_ratio != 0.0:
        failures.append(f"selected_valid_ratio {selected_valid_ratio:.3f} != 0")
    if selected_fallback_ratio != 1.0:
        failures.append(f"selected_fallback_ratio {selected_fallback_ratio:.3f} != 1")
    if gnss_valid_ratio != 0.0:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} != 0")
    if lidar_valid_ratio != 0.0:
        failures.append(f"lidar_valid_ratio {lidar_valid_ratio:.3f} != 0")
    if fused_valid_ratio != 0.0:
        failures.append(f"fused_valid_ratio {fused_valid_ratio:.3f} != 0")
    if finite_invalid_rows:
        failures.append(f"finite selected PL in invalid rows: {len(finite_invalid_rows)}")
    if fallback_empty_rows:
        failures.append(f"fallback rows with empty reason: {len(fallback_empty_rows)}")
    if copied_rows:
        failures.append(f"selected PL copied from current integrity: {len(copied_rows)}")
    if module_total_p95 is None or module_total_p95 >= module_total_p95_threshold_us:
        failures.append(
            f"module_total_us p95 {module_total_p95} >= {module_total_p95_threshold_us:g}"
        )
    if missing_or_empty:
        failures.append("missing or empty required files: " + ",".join(missing_or_empty))

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "selected_valid_ratio": selected_valid_ratio,
        "selected_fallback_ratio": selected_fallback_ratio,
        "gnss_valid_ratio": gnss_valid_ratio,
        "lidar_valid_ratio": lidar_valid_ratio,
        "fused_valid_ratio": fused_valid_ratio,
        "module_valid_ratio": module_valid_ratio,
        "module_fallback_ratio": module_fallback_ratio,
        "finite_selected_pl_count": len(finite_invalid_rows),
        "finite_selected_pl_rows": finite_invalid_rows[:50],
        "fallback_reason_empty_count": len(fallback_empty_rows),
        "fallback_reason_empty_rows": fallback_empty_rows[:50],
        "copied_current_flag_count": len(copied_rows),
        "copied_current_rows": copied_rows[:50],
        "copy_tolerance_m": copy_tolerance_m,
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "module_total_p95_threshold_us": module_total_p95_threshold_us,
        "source_histogram": source_histogram(rows),
        "fallback_reason_histogram": reason_histogram(rows),
        "query_labels": sorted({row.get("query_label", "") for row in rows}),
        "required_files_missing_or_empty": missing_or_empty,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--copy-tolerance-m", type=float, default=1.0e-6)
    parser.add_argument("--module-total-p95-threshold-us", type=float, default=10000.0)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, metadata_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    launch_config_path = metadata_dir / "predictor_launch_config.json"
    probe_config_path = metadata_dir / "predictor_probe_config.json"
    run_manifest_path = metadata_dir / "run_manifest.json"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)

    negative_path, negative_rows = write_negative_case_rows(
        rows, export_dir, args.copy_tolerance_m
    )
    snapshot_path, snapshot_rows = write_snapshot_debug(rows, export_dir)
    current_vs_path, current_vs_rows = write_current_vs_advisory_debug(
        rows, export_dir, args.copy_tolerance_m
    )

    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "source_selection_debug.csv": source_selection_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_debug.csv": timing_path,
        "predictor_launch_config.json": launch_config_path,
        "predictor_probe_config.json": probe_config_path,
        "run_manifest.json": run_manifest_path,
        "negative_case_rows.csv": negative_path,
        "snapshot_debug.csv": snapshot_path,
        "current_vs_advisory_debug.csv": current_vs_path,
    }

    summary = analyze(
        rows,
        timing_rows,
        required_files,
        args.copy_tolerance_m,
        args.module_total_p95_threshold_us,
    )
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "profiling_dir": str(profiling_dir),
        "metadata_dir": str(metadata_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "timing_path": str(timing_path),
        "source_selection_path": str(source_selection_path),
        "fallback_path": str(fallback_path),
        "negative_case_rows_path": str(negative_path),
        "snapshot_debug_path": str(snapshot_path),
        "current_vs_advisory_debug_path": str(current_vs_path),
        "negative_case_row_count": len(negative_rows),
        "snapshot_debug_row_count": len(snapshot_rows),
        "current_vs_advisory_row_count": len(current_vs_rows),
    })

    figures: list[str] = []
    if rows:
        plot_valid_fallback_flags(rows, figures_dir)
        plot_fallback_reason_histogram(rows, figures_dir)
        plot_selected_pl_timeline(rows, figures_dir)
        plot_current_vs_selected(rows, figures_dir, args.copy_tolerance_m)
        plot_source_selection(rows, figures_dir)
        figures = [
            str(figures_dir / "E7_valid_fallback_flags.png"),
            str(figures_dir / "E7_fallback_reason_histogram.png"),
            str(figures_dir / "E7_selected_pl_timeline.png"),
            str(figures_dir / "E7_current_vs_selected_pl.png"),
            str(figures_dir / "E7_source_selection_timeline.png"),
        ]
    summary["figures"] = figures

    audit = {
        "passed": summary["passed"],
        "failures": summary["failures"],
        "query_count": summary["query_count"],
        "selected_valid_ratio": summary["selected_valid_ratio"],
        "selected_fallback_ratio": summary["selected_fallback_ratio"],
        "gnss_valid_ratio": summary["gnss_valid_ratio"],
        "lidar_valid_ratio": summary["lidar_valid_ratio"],
        "fused_valid_ratio": summary["fused_valid_ratio"],
        "finite_selected_pl_count": summary["finite_selected_pl_count"],
        "fallback_reason_empty_count": summary["fallback_reason_empty_count"],
        "copied_current_flag_count": summary["copied_current_flag_count"],
        "fallback_reason_histogram": summary["fallback_reason_histogram"],
        "figures": figures,
    }
    audit_path = export_dir / "invalid_output_audit.json"
    write_json(audit_path, audit)
    summary["invalid_output_audit_path"] = str(audit_path)

    out_path = export_dir / "predictor_e7_analysis_summary.json"
    write_json(out_path, summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
