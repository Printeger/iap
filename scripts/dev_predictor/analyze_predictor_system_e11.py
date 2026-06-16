#!/usr/bin/env python3
"""Analyze Predictor system experiment 11 stale snapshot guard artifacts."""

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


EXPECTED_VARIANTS = (
    "normal",
    "stale_odom",
    "stale_integrity",
    "stale_gnss",
    "stale_snapshot",
)

STALE_REASON_BY_VARIANT = {
    "stale_odom": "stale_odom",
    "stale_integrity": "stale_integrity",
    "stale_gnss": "stale_gnss_epoch",
    "stale_snapshot": "stale_snapshot",
}

AGE_FIELD_BY_VARIANT = {
    "stale_odom": "odom_age_s",
    "stale_integrity": "integrity_age_s",
    "stale_gnss": "gnss_age_s",
    "stale_snapshot": "snapshot_age_s",
}


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


def finite_values(rows: list[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def value_series(rows: list[dict[str, str]], key: str) -> list[float]:
    out: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        out.append(value if value is not None else math.nan)
    return out


def x_series(rows: list[dict[str, str]]) -> list[float]:
    stamps = [finite_float(row.get("stamp")) for row in rows]
    finite = [value for value in stamps if value is not None]
    if not finite:
        return list(range(len(rows)))
    t0 = finite[0]
    return [(value - t0) if value is not None else float(i)
            for i, value in enumerate(stamps)]


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


def reason_text(row: dict[str, str]) -> str:
    for key in (
        "freshness_guard_reason",
        "selected_fallback_reason",
        "fallback_reason",
        "module_fallback_reason",
        "module_reason",
    ):
        value = (row.get(key) or "").strip()
        if value:
            return value
    return ""


def variant_rows(rows: list[dict[str, str]], variant: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("stale_variant_label") == variant]


def stale_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [row for row in rows if row.get("stale_variant_label") in STALE_REASON_BY_VARIANT]


def selected_pl_is_nonfinite(row: dict[str, str]) -> bool:
    return (
        finite_float(row.get("selected_hpl")) is None
        and finite_float(row.get("selected_vpl")) is None
        and finite_float(row.get("selected_pl")) is None
    )


def source_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    return dict(sorted(Counter(row.get("stale_variant_label", "") for row in rows).items()))


def reason_histogram(rows: list[dict[str, str]]) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for row in rows:
        reason = reason_text(row)
        if reason:
            counter[reason] += 1
    return dict(sorted(counter.items()))


def required_missing_or_empty(required: dict[str, Path]) -> list[str]:
    return [
        name for name, path in required.items()
        if not path.is_file() or path.stat().st_size == 0
    ]


def plot_age_vs_validity(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for key, label in (
        ("odom_age_s", "odom age"),
        ("integrity_age_s", "integrity age"),
        ("gnss_age_s", "GNSS age"),
        ("snapshot_age_s", "snapshot age"),
    ):
        axes[0].plot(x, value_series(rows, key), label=label, linewidth=1.2)
    valid = [1.0 if is_true(row.get("selected_valid")) else 0.0 for row in rows]
    fallback = [1.0 if is_true(row.get("selected_fallback")) else 0.0 for row in rows]
    axes[1].step(x, valid, where="post", label="selected valid", color="#16a34a")
    axes[1].step(x, fallback, where="post", label="selected fallback", color="#dc2626")
    axes[0].set_ylabel("age [s]")
    axes[1].set_ylabel("flag")
    axes[1].set_yticks([0, 1], ["false", "true"])
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 11 stale age vs selected validity")
    return save(fig, figures_dir / "E11_age_vs_validity.png")


def plot_reason_timeline(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    stale = stale_rows(rows)
    x = x_series(stale)
    reasons = sorted({reason_text(row) for row in stale if reason_text(row)})
    reason_to_y = {reason: idx for idx, reason in enumerate(reasons)}
    y = [reason_to_y.get(reason_text(row), math.nan) for row in stale]
    colors = {
        "stale_odom": "#2563eb",
        "stale_integrity": "#f97316",
        "stale_gnss": "#7c3aed",
        "stale_snapshot": "#dc2626",
    }
    fig, ax = plt.subplots(figsize=(11, 5))
    for variant in STALE_REASON_BY_VARIANT:
        xs = [x[i] for i, row in enumerate(stale) if row.get("stale_variant_label") == variant]
        ys = [y[i] for i, row in enumerate(stale) if row.get("stale_variant_label") == variant]
        ax.scatter(xs, ys, s=12, alpha=0.7, label=variant, color=colors.get(variant))
    ax.set_yticks(list(reason_to_y.values()), list(reason_to_y.keys()))
    ax.set_xlabel("time since first stale query [s]")
    ax.set_title("Experiment 11 stale reason timeline")
    ax.grid(True, alpha=0.25)
    ax.legend()
    return save(fig, figures_dir / "E11_stale_reason_timeline.png")


def plot_stale_source_histogram(rows: list[dict[str, str]], figures_dir: Path) -> Path:
    counts = Counter(row.get("stale_variant_label", "") for row in stale_rows(rows))
    labels = list(STALE_REASON_BY_VARIANT.keys())
    values = [counts.get(label, 0) for label in labels]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(labels, values, color=["#2563eb", "#f97316", "#7c3aed", "#dc2626"])
    ax.set_ylabel("row count")
    ax.set_title("Experiment 11 stale source histogram")
    ax.grid(axis="y", alpha=0.25)
    return save(fig, figures_dir / "E11_stale_source_histogram.png")


def plot_latency_distribution(timing_rows: list[dict[str, str]],
                              rows: list[dict[str, str]],
                              figures_dir: Path) -> Path:
    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    fig, ax = plt.subplots(figsize=(9, 5))
    if module_total:
        ax.hist(module_total, bins=35, color="#2563eb", alpha=0.75)
    ax.set_xlabel("module_total_us")
    ax.set_ylabel("count")
    ax.set_title("Experiment 11 latency distribution")
    ax.grid(True, alpha=0.25)
    return save(fig, figures_dir / "E11_latency_distribution.png")


def analyze(
    rows: list[dict[str, str]],
    stale_debug_rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    required_files: dict[str, Path],
    max_age_s: float,
) -> tuple[dict[str, object], dict[str, object]]:
    failures: list[str] = []
    query_count = len(rows)
    variants = sorted({row.get("stale_variant_label", "") for row in rows})
    missing_variants = [variant for variant in EXPECTED_VARIANTS if variant not in variants]
    normal = variant_rows(rows, "normal")
    stale = stale_rows(rows)
    normal_valid_ratio = ratio(sum(1 for row in normal if is_true(row.get("selected_valid"))), len(normal))
    stale_valid_count = sum(1 for row in stale if is_true(row.get("selected_valid")))
    stale_fallback_count = sum(1 for row in stale if is_true(row.get("selected_fallback")))
    stale_nonfinite_pl_count = sum(1 for row in stale if selected_pl_is_nonfinite(row))
    stale_reason_count = 0
    bad_stale_rows: list[dict[str, object]] = []
    bad_age_rows: list[dict[str, object]] = []
    bad_reason_rows: list[dict[str, object]] = []
    finite_pl_rows: list[dict[str, object]] = []

    for index, row in enumerate(stale, start=1):
        variant = row.get("stale_variant_label", "")
        expected_reason = STALE_REASON_BY_VARIANT.get(variant, "")
        age_field = AGE_FIELD_BY_VARIANT.get(variant, "")
        age = finite_float(row.get(age_field))
        reason = reason_text(row)
        if reason:
            stale_reason_count += 1
        if age is None or age <= max_age_s:
            bad_age_rows.append({
                "row": index,
                "query_index": row.get("query_index", ""),
                "variant": variant,
                "age_field": age_field,
                "age": age,
            })
        if expected_reason not in reason:
            bad_reason_rows.append({
                "row": index,
                "query_index": row.get("query_index", ""),
                "variant": variant,
                "expected_reason": expected_reason,
                "reason": reason,
            })
        if is_true(row.get("selected_valid")) or not is_true(row.get("selected_fallback")):
            bad_stale_rows.append({
                "row": index,
                "query_index": row.get("query_index", ""),
                "variant": variant,
                "selected_valid": row.get("selected_valid", ""),
                "selected_fallback": row.get("selected_fallback", ""),
                "reason": reason,
            })
        if not selected_pl_is_nonfinite(row):
            finite_pl_rows.append({
                "row": index,
                "query_index": row.get("query_index", ""),
                "variant": variant,
                "selected_hpl": row.get("selected_hpl", ""),
                "selected_vpl": row.get("selected_vpl", ""),
                "selected_pl": row.get("selected_pl", ""),
            })

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    missing_files = required_missing_or_empty(required_files)

    if query_count == 0:
        failures.append("no predictor rows found")
    if missing_files:
        failures.append("missing or empty required files: " + ",".join(missing_files))
    if missing_variants:
        failures.append("missing stale variants: " + ",".join(missing_variants))
    if normal and normal_valid_ratio <= 0.80:
        failures.append(f"normal selected valid ratio {normal_valid_ratio:.3f} <= 0.80")
    if not normal:
        failures.append("no normal variant rows found")
    if not stale:
        failures.append("no stale variant rows found")
    if stale_valid_count != 0:
        failures.append(f"stale selected valid count {stale_valid_count} != 0")
    if stale and stale_fallback_count != len(stale):
        failures.append(
            f"stale selected fallback count {stale_fallback_count} != {len(stale)}"
        )
    if stale and stale_reason_count != len(stale):
        failures.append(f"stale fallback reason count {stale_reason_count} != {len(stale)}")
    if bad_age_rows:
        failures.append(f"stale age rows not above threshold: {len(bad_age_rows)}")
    if bad_reason_rows:
        failures.append(f"stale reason mismatch rows: {len(bad_reason_rows)}")
    if bad_stale_rows:
        failures.append(f"stale validity/fallback mismatch rows: {len(bad_stale_rows)}")
    if finite_pl_rows:
        failures.append(f"stale rows with finite selected PL: {len(finite_pl_rows)}")
    if module_total_p95 is None or module_total_p95 >= 10000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 10000")

    per_variant: dict[str, object] = {}
    for variant in EXPECTED_VARIANTS:
        vrows = variant_rows(rows, variant)
        per_variant[variant] = {
            "row_count": len(vrows),
            "selected_valid_ratio": ratio(
                sum(1 for row in vrows if is_true(row.get("selected_valid"))),
                len(vrows),
            ),
            "selected_fallback_ratio": ratio(
                sum(1 for row in vrows if is_true(row.get("selected_fallback"))),
                len(vrows),
            ),
            "median_odom_age_s": median(finite_values(vrows, "odom_age_s"))
            if finite_values(vrows, "odom_age_s") else None,
            "median_integrity_age_s": median(finite_values(vrows, "integrity_age_s"))
            if finite_values(vrows, "integrity_age_s") else None,
            "median_gnss_age_s": median(finite_values(vrows, "gnss_age_s"))
            if finite_values(vrows, "gnss_age_s") else None,
            "median_snapshot_age_s": median(finite_values(vrows, "snapshot_age_s"))
            if finite_values(vrows, "snapshot_age_s") else None,
        }

    audit = {
        "passed": not failures,
        "failures": failures,
        "max_age_s": max_age_s,
        "variant_histogram": source_histogram(rows),
        "reason_histogram": reason_histogram(rows),
        "bad_age_rows": bad_age_rows[:50],
        "bad_reason_rows": bad_reason_rows[:50],
        "bad_stale_rows": bad_stale_rows[:50],
        "finite_pl_rows": finite_pl_rows[:50],
    }
    summary = {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "stale_debug_row_count": len(stale_debug_rows),
        "variants": variants,
        "missing_variants": missing_variants,
        "normal_selected_valid_ratio": normal_valid_ratio,
        "stale_selected_valid_count": stale_valid_count,
        "stale_selected_fallback_ratio": ratio(stale_fallback_count, len(stale)),
        "stale_reason_nonempty_ratio": ratio(stale_reason_count, len(stale)),
        "stale_nonfinite_selected_pl_ratio": ratio(stale_nonfinite_pl_count, len(stale)),
        "per_variant": per_variant,
        "reason_histogram": reason_histogram(rows),
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "bad_age_rows": bad_age_rows[:50],
        "bad_reason_rows": bad_reason_rows[:50],
        "bad_stale_rows": bad_stale_rows[:50],
        "finite_pl_rows": finite_pl_rows[:50],
    }
    return summary, audit


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--max-age-s", type=float, default=0.5)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    stale_debug_path = export_dir / "stale_snapshot_debug.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    fusion_debug_path = export_dir / "predictor_fusion_debug.csv"
    timing_path = profiling_dir / "latency_debug.csv"

    rows = read_csv(csv_path)
    stale_debug_rows = read_csv(stale_debug_path)
    timing_rows = read_csv(timing_path)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "stale_snapshot_debug.csv": stale_debug_path,
        "source_selection_debug.csv": source_selection_path,
        "fallback_reason_by_time.csv": fallback_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "latency_debug.csv": timing_path,
    }
    summary, audit = analyze(rows, stale_debug_rows, timing_rows, required_files, args.max_age_s)

    figures: list[str] = []
    if rows:
        figures.extend([
            str(plot_age_vs_validity(rows, figures_dir)),
            str(plot_reason_timeline(rows, figures_dir)),
            str(plot_stale_source_histogram(rows, figures_dir)),
            str(plot_latency_distribution(timing_rows, rows, figures_dir)),
        ])

    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "stale_debug_path": str(stale_debug_path),
        "timing_path": str(timing_path),
        "figures": figures,
    })
    audit["figures"] = figures

    write_json(export_dir / "predictor_e11_analysis_summary.json", summary)
    write_json(export_dir / "stale_snapshot_audit.json", audit)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
