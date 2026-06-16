#!/usr/bin/env python3
"""Analyze Predictor system experiment 5 artifacts and generate report figures."""

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


def degradation_score(row: dict[str, str]) -> float | None:
    condition = finite_float(row.get("lidar_lambda_condition"))
    min_eig = finite_float(row.get("lidar_lambda_min_eig"))
    if condition is None or min_eig is None:
        return None
    return condition / max(abs(min_eig), 1.0e-12)


def pl_not_optimistic(
    row: dict[str, str],
    prefix: str,
    tolerance_m: float,
) -> bool:
    hpl = finite_float(row.get(f"{prefix}_hpl"))
    vpl = finite_float(row.get(f"{prefix}_vpl"))
    gnss_hpl = finite_float(row.get("gnss_hpl"))
    gnss_vpl = finite_float(row.get("gnss_vpl"))
    if None in (hpl, vpl, gnss_hpl, gnss_vpl):
        return False
    return hpl + tolerance_m >= gnss_hpl and vpl + tolerance_m >= gnss_vpl


def gate_reason(
    row: dict[str, str],
    condition_threshold: float,
    tolerance_m: float,
) -> str:
    direct = reason_text(row)
    if direct:
        return direct
    condition = finite_float(row.get("lidar_lambda_condition"))
    if row.get("lidar_valid") != "1":
        return "lidar_invalid"
    if condition is None:
        return "lidar_condition_missing"
    if condition >= condition_threshold:
        return "lidar_condition_high"
    if not pl_not_optimistic(row, "selected", tolerance_m):
        return "selected_below_gnss_reference"
    if not pl_not_optimistic(row, "fused", tolerance_m):
        return "fused_below_gnss_reference"
    return "lidar_valid_but_conservative_gate_ok"


def lidar_allowed_for_fusion(
    row: dict[str, str],
    condition_threshold: float,
    tolerance_m: float,
) -> bool:
    condition = finite_float(row.get("lidar_lambda_condition"))
    return (
        row.get("lidar_valid") == "1"
        and condition is not None
        and condition < condition_threshold
        and pl_not_optimistic(row, "selected", tolerance_m)
        and pl_not_optimistic(row, "fused", tolerance_m)
    )


def write_gate_debug(
    rows: list[dict[str, str]],
    export_dir: Path,
    condition_threshold: float,
    tolerance_m: float,
) -> tuple[Path, list[dict[str, object]]]:
    gate_rows: list[dict[str, object]] = []
    for row in rows:
        selected_hpl = finite_float(row.get("selected_hpl"))
        selected_vpl = finite_float(row.get("selected_vpl"))
        gnss_hpl = finite_float(row.get("gnss_hpl"))
        gnss_vpl = finite_float(row.get("gnss_vpl"))
        gate_rows.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "lidar_condition": row.get("lidar_lambda_condition", ""),
            "lidar_min_eig": row.get("lidar_lambda_min_eig", ""),
            "lidar_trace": row.get("lidar_lambda_trace", ""),
            "lidar_n_primitives": row.get("lidar_n_primitives", ""),
            "lidar_n_valid_normals": row.get("lidar_n_valid_normals", ""),
            "lidar_degeneracy_score": degradation_score(row),
            "lidar_allowed_for_fusion": int(
                lidar_allowed_for_fusion(row, condition_threshold, tolerance_m)
            ),
            "lidar_fusion_gate_reason": gate_reason(row, condition_threshold, tolerance_m),
            "gnss_hpl": row.get("gnss_hpl", ""),
            "gnss_vpl": row.get("gnss_vpl", ""),
            "fused_hpl": row.get("fused_hpl", ""),
            "fused_vpl": row.get("fused_vpl", ""),
            "selected_hpl": row.get("selected_hpl", ""),
            "selected_vpl": row.get("selected_vpl", ""),
            "selected_source": row.get("selected_source", ""),
            "selected_minus_gnss_hpl": (
                selected_hpl - gnss_hpl
                if selected_hpl is not None and gnss_hpl is not None else ""
            ),
            "selected_minus_gnss_vpl": (
                selected_vpl - gnss_vpl
                if selected_vpl is not None and gnss_vpl is not None else ""
            ),
        })
    fieldnames = [
        "stamp", "query_index", "query_label", "lidar_condition", "lidar_min_eig",
        "lidar_trace", "lidar_n_primitives", "lidar_n_valid_normals",
        "lidar_degeneracy_score", "lidar_allowed_for_fusion",
        "lidar_fusion_gate_reason", "gnss_hpl", "gnss_vpl", "fused_hpl",
        "fused_vpl", "selected_hpl", "selected_vpl", "selected_source",
        "selected_minus_gnss_hpl", "selected_minus_gnss_vpl",
    ]
    path = export_dir / "lidar_gate_debug.csv"
    write_csv(path, fieldnames, gate_rows)
    return path, gate_rows


def plot_lidar_degradation(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    scores = []
    for row in rows:
        score = degradation_score(row)
        scores.append(score if score is not None else math.nan)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, values(rows, "lidar_n_primitives"), label="primitives", color="#16a34a")
    axes[0].plot(x, values(rows, "lidar_n_valid_normals"), label="valid normals", color="#2563eb")
    axes[0].set_ylabel("count")
    axes[1].plot(x, values(rows, "lidar_lambda_condition"), label="condition", color="#dc2626")
    axes[1].set_yscale("log")
    axes[1].set_ylabel("condition")
    axes[2].plot(x, scores, label="condition / min eig", color="#7c3aed")
    axes[2].set_yscale("log")
    axes[2].set_ylabel("score")
    axes[2].set_xlabel("time since first query [s]")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 5 LiDAR degradation timeline")
    save(fig, figures_dir / "E5_lidar_degradation_timeline.png")


def plot_gnss_stability(rows: list[dict[str, str]], figures_dir: Path) -> None:
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
    fig.suptitle("Experiment 5 GNSS stability timeline")
    save(fig, figures_dir / "E5_gnss_stability_timeline.png")


def plot_fusion_gate(gate_rows: list[dict[str, object]], figures_dir: Path) -> None:
    x = list(range(len(gate_rows)))
    allowed = [finite_float(row.get("lidar_allowed_for_fusion")) or 0.0 for row in gate_rows]
    h_margin = [
        finite_float(row.get("selected_minus_gnss_hpl")) or math.nan
        for row in gate_rows
    ]
    v_margin = [
        finite_float(row.get("selected_minus_gnss_vpl")) or math.nan
        for row in gate_rows
    ]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].step(x, allowed, where="post", label="lidar_allowed_for_fusion", color="#16a34a")
    axes[0].set_ylim(-0.1, 1.1)
    axes[0].set_ylabel("allowed")
    axes[1].plot(x, h_margin, label="selected HPL - GNSS HPL", color="#2563eb")
    axes[1].plot(x, v_margin, label="selected VPL - GNSS VPL", color="#f97316")
    axes[1].axhline(0.0, color="#111827", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("margin [m]")
    axes[1].set_xlabel("query index")
    for ax in axes:
        ax.grid(True, alpha=0.25)
        ax.legend()
    fig.suptitle("Experiment 5 fusion gate timeline")
    save(fig, figures_dir / "E5_fusion_gate_timeline.png")


def plot_selected_vs_gnss(rows: list[dict[str, str]], figures_dir: Path) -> None:
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
    fig.suptitle("Experiment 5 selected vs GNSS PL")
    save(fig, figures_dir / "E5_selected_vs_gnss_pl.png")


def plot_lidar_primitives_topdown(rows: list[dict[str, str]], figures_dir: Path) -> None:
    xs = values(rows, "query_x")
    ys = values(rows, "query_y")
    colors = values(rows, "lidar_n_primitives")
    fig, ax = plt.subplots(figsize=(7, 6))
    sc = ax.scatter(xs, ys, c=colors, s=18, cmap="viridis", alpha=0.85)
    ax.set_title("Experiment 5 query map colored by LiDAR primitives")
    ax.set_xlabel("query_x [m]")
    ax.set_ylabel("query_y [m]")
    ax.grid(True, alpha=0.25)
    fig.colorbar(sc, ax=ax, label="lidar_n_primitives")
    save(fig, figures_dir / "E5_lidar_primitives_topdown.png")


def plot_lidar_eigenvalues(rows: list[dict[str, str]], figures_dir: Path) -> None:
    x = x_series(rows)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, values(rows, "lidar_lambda_trace"), label="LiDAR trace", color="#16a34a")
    ax.plot(x, values(rows, "lidar_lambda_min_eig"), label="LiDAR min eig", color="#2563eb")
    ax.plot(x, values(rows, "fused_lambda_pred_min_eig"), label="fused min eig", color="#7c3aed")
    ax.set_title("Experiment 5 LiDAR eigenvalues timeline")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("lambda diagnostic")
    ax.grid(True, alpha=0.25)
    ax.legend()
    save(fig, figures_dir / "E5_lidar_eigenvalues_timeline.png")


def conservative_violations(
    rows: list[dict[str, str]],
    tolerance_m: float,
) -> list[dict[str, object]]:
    violations: list[dict[str, object]] = []
    for index, row in enumerate(rows, start=1):
        if row.get("selected_valid") != "1":
            continue
        for prefix in ("selected", "fused"):
            hpl = finite_float(row.get(f"{prefix}_hpl"))
            vpl = finite_float(row.get(f"{prefix}_vpl"))
            gnss_hpl = finite_float(row.get("gnss_hpl"))
            gnss_vpl = finite_float(row.get("gnss_vpl"))
            if None in (hpl, vpl, gnss_hpl, gnss_vpl):
                continue
            if hpl + tolerance_m < gnss_hpl:
                violations.append({
                    "row": index,
                    "field": f"{prefix}_hpl",
                    "value": hpl,
                    "gnss_reference": gnss_hpl,
                })
            if vpl + tolerance_m < gnss_vpl:
                violations.append({
                    "row": index,
                    "field": f"{prefix}_vpl",
                    "value": vpl,
                    "gnss_reference": gnss_vpl,
                })
    return violations


def analyze(
    rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    open_sky_rows: list[dict[str, str]],
    gate_rows: list[dict[str, object]],
    required_files: dict[str, Path],
    selected_optimism_tolerance_m: float,
    lidar_condition_threshold: float,
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
    }
    median_gnss_hpl = median_or_none(rows, "gnss_hpl")
    median_gnss_pdop = median_or_none(rows, "gnss_pdop")

    median_lidar_n_primitives = median_or_none(rows, "lidar_n_primitives")
    median_lidar_condition = median_or_none(rows, "lidar_lambda_condition")
    gate_disallowed_ratio = ratio(
        sum(1 for row in gate_rows if str(row.get("lidar_allowed_for_fusion")) != "1"),
        len(gate_rows),
    )
    lidar_degraded_triggered = any((
        median_lidar_n_primitives is not None and median_lidar_n_primitives < 200.0,
        median_lidar_condition is not None and median_lidar_condition >= 1.0e3,
        lidar_valid_ratio < 0.80,
        gate_disallowed_ratio > 0.20,
    ))

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    lambda_sum_error_p95 = percentile(finite_values(rows, "lambda_sum_error"), 0.95)
    optimism_violations = conservative_violations(rows, selected_optimism_tolerance_m)

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
    if source_ratio <= 0.80:
        failures.append(f"selected_source_FUSION_ratio {source_ratio:.3f} <= 0.80")
    if gnss_valid_ratio <= 0.95:
        failures.append(f"gnss_valid_ratio {gnss_valid_ratio:.3f} <= 0.95")
    if fused_valid_ratio <= 0.80:
        failures.append(f"fused_valid_ratio {fused_valid_ratio:.3f} <= 0.80")
    if fallback_ratio >= 0.10:
        failures.append(f"fallback_ratio {fallback_ratio:.3f} >= 0.10")
    if (
        median_gnss_hpl is None
        or baseline["median_gnss_hpl"] is None
        or median_gnss_hpl > baseline["median_gnss_hpl"] * 1.10
    ):
        failures.append(
            "median gnss_hpl is not stable vs open-sky baseline "
            f"({median_gnss_hpl} > {baseline['median_gnss_hpl']} * 1.10)"
        )
    if (
        median_gnss_pdop is None
        or baseline["median_gnss_pdop"] is None
        or median_gnss_pdop > baseline["median_gnss_pdop"] * 1.10
    ):
        failures.append(
            "median gnss_pdop is not stable vs open-sky baseline "
            f"({median_gnss_pdop} > {baseline['median_gnss_pdop']} * 1.10)"
        )
    if not lidar_degraded_triggered:
        failures.append(
            "lidar degraded indicator not triggered "
            f"(median_primitives={median_lidar_n_primitives}, "
            f"median_condition={median_lidar_condition}, "
            f"lidar_valid_ratio={lidar_valid_ratio:.3f}, "
            f"gate_disallowed_ratio={gate_disallowed_ratio:.3f})"
        )
    if optimism_violations:
        failures.append(
            "selected/fused PL below GNSS reference: "
            f"{len(optimism_violations)} values"
        )
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
        "median_gnss_hpl": median_gnss_hpl,
        "median_open_sky_gnss_hpl": baseline["median_gnss_hpl"],
        "median_gnss_pdop": median_gnss_pdop,
        "median_open_sky_gnss_pdop": baseline["median_gnss_pdop"],
        "median_lidar_n_primitives": median_lidar_n_primitives,
        "median_lidar_n_valid_normals": median_or_none(rows, "lidar_n_valid_normals"),
        "median_lidar_lambda_trace": median_or_none(rows, "lidar_lambda_trace"),
        "median_lidar_lambda_min_eig": median_or_none(rows, "lidar_lambda_min_eig"),
        "median_lidar_lambda_condition": median_lidar_condition,
        "lidar_degraded_indicator_triggered": lidar_degraded_triggered,
        "lidar_gate_disallowed_ratio": gate_disallowed_ratio,
        "conservative_gate_violation_count": len(optimism_violations),
        "conservative_gate_violations": optimism_violations[:50],
        "selected_optimism_tolerance_m": selected_optimism_tolerance_m,
        "lidar_condition_threshold": lidar_condition_threshold,
        "lambda_sum_error_p95": lambda_sum_error_p95,
        "lambda_sum_error_p95_threshold": lambda_sum_error_p95_threshold,
        "median_lambda_prior_trace": median_or_none(rows, "fused_lambda_prior_trace"),
        "median_lambda_gnss_trace": median_or_none(rows, "fused_lambda_gnss_trace"),
        "median_lambda_lidar_trace": median_or_none(rows, "fused_lambda_lidar_trace"),
        "median_lambda_pred_trace": median_or_none(rows, "fused_lambda_pred_trace"),
        "median_fused_lambda_condition": median_or_none(rows, "fused_lambda_pred_condition"),
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "source_histogram": source_histogram(rows),
        "fallback_reason_histogram": reason_histogram(rows),
        "lidar_gate_reason_histogram": dict(sorted(Counter(
            str(row.get("lidar_fusion_gate_reason", "")) for row in gate_rows
        ).items())),
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
    parser.add_argument("--selected-optimism-tolerance-m", type=float, default=1.0e-6)
    parser.add_argument("--lidar-condition-threshold", type=float, default=1.0e6)
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
    map_snapshot_path = export_dir / "downsampled_map.csv"

    rows = read_csv(csv_path)
    timing_rows = read_csv(timing_path)
    gate_path, gate_rows = write_gate_debug(
        rows,
        export_dir,
        args.lidar_condition_threshold,
        args.selected_optimism_tolerance_m,
    )
    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "source_selection_debug.csv": source_selection_path,
        "gnss_visibility_by_query.csv": gnss_visibility_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "predictor_lidar_primitives_debug.csv": lidar_primitives_path,
        "fallback_reason_by_time.csv": fallback_path,
        "downsampled_map.csv": map_snapshot_path,
        "latency_debug.csv": timing_path,
        "lidar_gate_debug.csv": gate_path,
    }

    summary = analyze(
        rows,
        timing_rows,
        open_sky_rows,
        gate_rows,
        required_files,
        args.selected_optimism_tolerance_m,
        args.lidar_condition_threshold,
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
        "map_snapshot_path": str(map_snapshot_path),
        "lidar_gate_debug_path": str(gate_path),
    })

    figures: list[str] = []
    if rows:
        plot_lidar_degradation(rows, figures_dir)
        plot_gnss_stability(rows, figures_dir)
        plot_fusion_gate(gate_rows, figures_dir)
        plot_selected_vs_gnss(rows, figures_dir)
        plot_lidar_primitives_topdown(rows, figures_dir)
        plot_lidar_eigenvalues(rows, figures_dir)
        figures = [
            str(figures_dir / "E5_lidar_degradation_timeline.png"),
            str(figures_dir / "E5_gnss_stability_timeline.png"),
            str(figures_dir / "E5_fusion_gate_timeline.png"),
            str(figures_dir / "E5_selected_vs_gnss_pl.png"),
            str(figures_dir / "E5_lidar_primitives_topdown.png"),
            str(figures_dir / "E5_lidar_eigenvalues_timeline.png"),
        ]
    summary["figures"] = figures

    out_path = export_dir / "predictor_e5_analysis_summary.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
