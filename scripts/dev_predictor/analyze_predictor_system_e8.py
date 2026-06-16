#!/usr/bin/env python3
"""Analyze Predictor system experiment 8 artifacts and generate report figures."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


REQUIRED_VARIANTS = ("normal", "current_high", "current_unsafe")


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
    out: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        out.append(value if value is not None else math.nan)
    return out


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


def is_true(row: dict[str, str], key: str) -> bool:
    value = str(row.get(key, "")).strip().lower()
    return value in ("1", "true", "yes", "on")


def variant_label(row: dict[str, str]) -> str:
    return (row.get("current_variant_label") or "").strip() or "missing"


def current_state(row: dict[str, str]) -> int | None:
    value = finite_float(row.get("current_state"))
    if value is None:
        return None
    return int(value)


def selected_matches_gnss(row: dict[str, str], tolerance_m: float) -> bool:
    for selected_key, gnss_key in (("selected_hpl", "gnss_hpl"), ("selected_vpl", "gnss_vpl")):
        selected = finite_float(row.get(selected_key))
        gnss = finite_float(row.get(gnss_key))
        if selected is None or gnss is None or abs(selected - gnss) > tolerance_m:
            return False
    return True


def copied_current(row: dict[str, str], tolerance_m: float) -> bool:
    comparisons = (("selected_hpl", "current_hpl"), ("selected_vpl", "current_vpl"))
    for selected_key, current_key in comparisons:
        selected = finite_float(row.get(selected_key))
        current = finite_float(row.get(current_key))
        if selected is None or current is None or abs(selected - current) > tolerance_m:
            return False
    return True


def delta(row: dict[str, str], left: str, right: str) -> float | str:
    lhs = finite_float(row.get(left))
    rhs = finite_float(row.get(right))
    if lhs is None or rhs is None:
        return ""
    return lhs - rhs


def write_separation_csv(
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
            "current_variant_label": variant_label(row),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "current_state": row.get("current_state", ""),
            "current_valid": row.get("current_valid", ""),
            "gnss_hpl": row.get("gnss_hpl", ""),
            "gnss_vpl": row.get("gnss_vpl", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "selected_minus_gnss_hpl": delta(row, "selected_hpl", "gnss_hpl"),
            "selected_minus_gnss_vpl": delta(row, "selected_vpl", "gnss_vpl"),
            "selected_minus_current_hpl": delta(row, "selected_hpl", "current_hpl"),
            "selected_minus_current_vpl": delta(row, "selected_vpl", "current_vpl"),
            "copied_current_flag": int(copied_current(row, copy_tolerance_m)),
        })
    fieldnames = [
        "stamp", "query_index", "query_label", "current_variant_label",
        "current_hpl", "current_vpl", "current_state", "current_valid",
        "gnss_hpl", "gnss_vpl", "selected_hpl", "selected_vpl",
        "selected_minus_gnss_hpl", "selected_minus_gnss_vpl",
        "selected_minus_current_hpl", "selected_minus_current_vpl",
        "copied_current_flag",
    ]
    path = export_dir / "current_advisory_separation.csv"
    write_csv(path, fieldnames, out_rows)
    return path, out_rows


def variant_summary(rows: list[dict[str, str]]) -> dict[str, dict[str, object]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[variant_label(row)].append(row)
    summary: dict[str, dict[str, object]] = {}
    for label, label_rows in sorted(grouped.items()):
        summary[label] = {
            "count": len(label_rows),
            "selected_source_histogram": dict(sorted(Counter(
                row.get("selected_source", "") for row in label_rows
            ).items())),
            "selected_valid_ratio": ratio(
                sum(1 for row in label_rows if is_true(row, "selected_valid")),
                len(label_rows),
            ),
            "gnss_valid_ratio": ratio(
                sum(1 for row in label_rows if is_true(row, "gnss_valid")),
                len(label_rows),
            ),
            "median_current_hpl": median_or_none(label_rows, "current_hpl"),
            "median_current_vpl": median_or_none(label_rows, "current_vpl"),
            "median_gnss_hpl": median_or_none(label_rows, "gnss_hpl"),
            "median_gnss_vpl": median_or_none(label_rows, "gnss_vpl"),
            "median_selected_hpl": median_or_none(label_rows, "selected_hpl"),
            "median_selected_vpl": median_or_none(label_rows, "selected_vpl"),
            "current_states": dict(sorted(Counter(
                str(current_state(row)) for row in label_rows
            ).items())),
        }
    return summary


def plot_current_vs_advisory_bar(rows: list[dict[str, str]], figures_dir: Path) -> None:
    labels = [label for label in REQUIRED_VARIANTS if any(variant_label(row) == label for row in rows)]
    if not labels:
        labels = sorted({variant_label(row) for row in rows})
    grouped = {label: [row for row in rows if variant_label(row) == label] for label in labels}
    x = list(range(len(labels)))
    width = 0.22
    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for ax, suffix in ((axes[0], "hpl"), (axes[1], "vpl")):
        ax.bar([v - width for v in x], [median_or_none(grouped[l], f"current_{suffix}") or math.nan for l in labels], width, label=f"current {suffix.upper()}", color="#f97316")
        ax.bar(x, [median_or_none(grouped[l], f"gnss_{suffix}") or math.nan for l in labels], width, label=f"GNSS {suffix.upper()}", color="#2563eb")
        ax.bar([v + width for v in x], [median_or_none(grouped[l], f"selected_{suffix}") or math.nan for l in labels], width, label=f"selected {suffix.upper()}", color="#111827")
        ax.set_ylabel(f"{suffix.upper()} [m]")
        ax.grid(True, axis="y", alpha=0.25)
        ax.legend()
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(labels)
    axes[1].set_xlabel("current variant")
    fig.suptitle("Experiment 8 current vs advisory PL medians")
    save(fig, figures_dir / "E8_current_vs_advisory_bar.png")


def plot_copied_current_timeline(rows: list[dict[str, str]], figures_dir: Path, copy_tolerance_m: float) -> None:
    x = x_series(rows)
    flags = [1.0 if copied_current(row, copy_tolerance_m) else 0.0 for row in rows]
    codes, labels = categorical_codes([variant_label(row) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(x, flags, where="post", label="copied current flag", color="#dc2626")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel("copy flag")
    axes[1].scatter(x, codes, s=10, label="current variant", color="#2563eb")
    axes[1].set_yticks(list(range(len(labels))))
    axes[1].set_yticklabels(labels)
    axes[1].set_ylabel("variant")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 8 copied current flag timeline")
    save(fig, figures_dir / "E8_copied_current_flag_timeline.png")


def plot_selected_vs_gnss(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, values(rows, "gnss_hpl"), label="GNSS HPL", color="#2563eb", alpha=0.75)
    axes[0].plot(x, values(rows, "selected_hpl"), label="selected HPL", color="#111827", linewidth=1.4)
    axes[0].plot(x, values(rows, "current_hpl"), label="current HPL", color="#f97316", alpha=0.45)
    axes[0].set_ylabel("HPL [m]")
    axes[1].plot(x, values(rows, "gnss_vpl"), label="GNSS VPL", color="#2563eb", alpha=0.75)
    axes[1].plot(x, values(rows, "selected_vpl"), label="selected VPL", color="#111827", linewidth=1.4)
    axes[1].plot(x, values(rows, "current_vpl"), label="current VPL", color="#f97316", alpha=0.45)
    axes[1].set_ylabel("VPL [m]")
    axes[1].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 8 selected vs GNSS timeline")
    save(fig, figures_dir / "E8_selected_vs_gnss_timeline.png")


def plot_variant_latency(rows: list[dict[str, str]], figures_dir: Path) -> None:
    labels = [label for label in REQUIRED_VARIANTS if any(variant_label(row) == label for row in rows)]
    if not labels:
        labels = sorted({variant_label(row) for row in rows})
    data = [finite_values([row for row in rows if variant_label(row) == label], "module_total_us") for label in labels]
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.boxplot(data, tick_labels=labels, showfliers=False)
    ax.set_title("Experiment 8 module latency by current variant")
    ax.set_ylabel("module_total_us")
    ax.grid(True, axis="y", alpha=0.25)
    save(fig, figures_dir / "E8_variant_latency_distribution.png")


def categorical_codes(values_in: list[str]) -> tuple[list[int], list[str]]:
    labels = sorted({value or "UNKNOWN" for value in values_in})
    index = {label: i for i, label in enumerate(labels)}
    return [index[value or "UNKNOWN"] for value in values_in], labels


def required_missing_or_empty(required_files: dict[str, Path]) -> list[str]:
    return [
        name for name, path in required_files.items()
        if not path.is_file() or path.stat().st_size <= 0
    ]


def analyze(
    rows: list[dict[str, str]],
    required_files: dict[str, Path],
    copy_tolerance_m: float,
    advisory_match_tolerance_m: float,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    valid_rows = [row for row in rows if is_true(row, "selected_valid")]
    artificial_rows = [
        row for row in rows
        if variant_label(row) in ("current_high", "current_unsafe")
    ]
    variants = sorted({variant_label(row) for row in rows})
    missing_variants = [label for label in REQUIRED_VARIANTS if label not in variants]
    selected_source_gnss_ratio = ratio(
        sum(1 for row in rows if row.get("selected_source") == "GNSS"),
        query_count,
    )
    selected_valid_ratio = ratio(len(valid_rows), query_count)
    gnss_valid_ratio = ratio(sum(1 for row in rows if is_true(row, "gnss_valid")), query_count)
    copied_artificial = [
        {
            "row": index,
            "query_index": row.get("query_index", ""),
            "current_variant_label": variant_label(row),
            "current_hpl": row.get("current_hpl", ""),
            "current_vpl": row.get("current_vpl", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
        }
        for index, row in enumerate(rows, start=1)
        if row in artificial_rows and copied_current(row, copy_tolerance_m)
    ]
    mismatch_rows = [
        {
            "row": index,
            "query_index": row.get("query_index", ""),
            "current_variant_label": variant_label(row),
            "selected_hpl": row.get("selected_hpl", ""),
            "gnss_hpl": row.get("gnss_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "gnss_vpl": row.get("gnss_vpl", ""),
        }
        for index, row in enumerate(valid_rows, start=1)
        if not selected_matches_gnss(row, advisory_match_tolerance_m)
    ]
    nonfinite_valid: list[dict[str, object]] = []
    for index, row in enumerate(valid_rows, start=1):
        for key in (
            "selected_hpl", "selected_vpl", "gnss_hpl", "gnss_vpl",
            "current_hpl", "current_vpl",
        ):
            if finite_float(row.get(key)) is None:
                nonfinite_valid.append({
                    "row": index,
                    "query_index": row.get("query_index", ""),
                    "field": key,
                })
    high_rows = [row for row in rows if variant_label(row) == "current_high"]
    unsafe_rows = [row for row in rows if variant_label(row) == "current_unsafe"]
    median_high_hpl = median_or_none(high_rows, "current_hpl")
    median_high_vpl = median_or_none(high_rows, "current_vpl")
    unsafe_bad_state_count = sum(1 for row in unsafe_rows if current_state(row) != 2)
    module_total = finite_values(rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    missing_or_empty = required_missing_or_empty(required_files)

    if query_count == 0:
        failures.append("no predictor rows found")
    if missing_variants:
        failures.append("missing required variants: " + ",".join(missing_variants))
    if selected_source_gnss_ratio <= 0.95:
        failures.append(f"selected_source_GNSS_ratio {selected_source_gnss_ratio:.3f} <= 0.95")
    if selected_valid_ratio <= 0.95:
        failures.append(f"selected_valid_ratio {selected_valid_ratio:.3f} <= 0.95")
    if gnss_valid_ratio <= 0.95:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} <= 0.95")
    if nonfinite_valid:
        failures.append(f"non-finite valid fields: {len(nonfinite_valid)}")
    if mismatch_rows:
        failures.append(f"selected PL does not match GNSS advisory: {len(mismatch_rows)} rows")
    if copied_artificial:
        failures.append(f"artificial current copied into selected output: {len(copied_artificial)} rows")
    if median_high_hpl is None or abs(median_high_hpl - 1000.0) > 1.0e-3:
        failures.append(f"current_high median HPL {median_high_hpl} not near 1000")
    if median_high_vpl is None or abs(median_high_vpl - 1000.0) > 1.0e-3:
        failures.append(f"current_high median VPL {median_high_vpl} not near 1000")
    if unsafe_bad_state_count:
        failures.append(f"current_unsafe rows with state != 2: {unsafe_bad_state_count}")
    if module_total_p95 is None or module_total_p95 >= 2000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 2000")
    if missing_or_empty:
        failures.append("missing or empty required files: " + ",".join(missing_or_empty))

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "variants": variants,
        "missing_variants": missing_variants,
        "selected_source_GNSS_ratio": selected_source_gnss_ratio,
        "selected_valid_ratio": selected_valid_ratio,
        "gnss_valid_ratio": gnss_valid_ratio,
        "copied_current_flag_count": len(copied_artificial),
        "copied_current_rows": copied_artificial[:50],
        "selected_gnss_mismatch_count": len(mismatch_rows),
        "selected_gnss_mismatch_rows": mismatch_rows[:50],
        "nonfinite_valid_fields": nonfinite_valid[:50],
        "current_high_median_hpl": median_high_hpl,
        "current_high_median_vpl": median_high_vpl,
        "current_unsafe_bad_state_count": unsafe_bad_state_count,
        "copy_tolerance_m": copy_tolerance_m,
        "advisory_match_tolerance_m": advisory_match_tolerance_m,
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "source_histogram": dict(sorted(Counter(row.get("selected_source", "") for row in rows).items())),
        "variant_histogram": dict(sorted(Counter(variant_label(row) for row in rows).items())),
        "required_files_missing_or_empty": missing_or_empty,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--copy-tolerance-m", type=float, default=1.0e-6)
    parser.add_argument("--advisory-match-tolerance-m", type=float, default=1.0e-6)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, metadata_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    source_selection_path = export_dir / "source_selection_debug.csv"
    gnss_visibility_path = export_dir / "gnss_visibility_by_query.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    timing_path = profiling_dir / "latency_debug.csv"
    launch_config_path = metadata_dir / "predictor_launch_config.json"
    probe_config_path = metadata_dir / "predictor_probe_config.json"
    run_manifest_path = metadata_dir / "run_manifest.json"

    rows = read_csv(csv_path)
    separation_path, separation_rows = write_separation_csv(
        rows,
        export_dir,
        args.copy_tolerance_m,
    )
    variant_summary_data = variant_summary(rows)
    variant_summary_path = export_dir / "current_advisory_variant_summary.json"
    write_json(variant_summary_path, variant_summary_data)
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_debug.csv": timing_path,
        "predictor_launch_config.json": launch_config_path,
        "predictor_probe_config.json": probe_config_path,
        "run_manifest.json": run_manifest_path,
        "current_advisory_separation.csv": separation_path,
        "current_advisory_variant_summary.json": variant_summary_path,
    }
    summary = analyze(
        rows,
        required_files,
        args.copy_tolerance_m,
        args.advisory_match_tolerance_m,
    )
    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "profiling_dir": str(profiling_dir),
        "metadata_dir": str(metadata_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "source_selection_path": str(source_selection_path),
        "gnss_visibility_path": str(gnss_visibility_path),
        "fallback_path": str(fallback_path),
        "timing_path": str(timing_path),
        "current_advisory_separation_path": str(separation_path),
        "current_advisory_separation_row_count": len(separation_rows),
        "current_advisory_variant_summary_path": str(variant_summary_path),
        "variant_summary": variant_summary_data,
    })

    figures: list[str] = []
    if rows:
        plot_current_vs_advisory_bar(rows, figures_dir)
        plot_copied_current_timeline(rows, figures_dir, args.copy_tolerance_m)
        plot_selected_vs_gnss(rows, figures_dir)
        plot_variant_latency(rows, figures_dir)
        figures = [
            str(figures_dir / "E8_current_vs_advisory_bar.png"),
            str(figures_dir / "E8_copied_current_flag_timeline.png"),
            str(figures_dir / "E8_selected_vs_gnss_timeline.png"),
            str(figures_dir / "E8_variant_latency_distribution.png"),
        ]
    summary["figures"] = figures

    out_path = export_dir / "predictor_e8_analysis_summary.json"
    write_json(out_path, summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
