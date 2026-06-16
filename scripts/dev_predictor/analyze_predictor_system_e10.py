#!/usr/bin/env python3
"""Analyze Predictor system experiment 10 corridor LiDAR degeneracy artifacts."""

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


DIAGNOSTIC_FIELDS = (
    "lidar_lambda_trace",
    "lidar_lambda_min_eig",
    "lidar_lambda_condition",
    "lidar_alpha",
    "lidar_tdop",
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


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total else 0.0


def finite_values(rows: list[dict[str, object]] | list[dict[str, str]], key: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = finite_float(row.get(key))
        if value is not None:
            values.append(value)
    return values


def value_series(rows: list[dict[str, object]], key: str) -> list[float]:
    return [
        value if (value := finite_float(row.get(key))) is not None else math.nan
        for row in rows
    ]


def median_value(rows: list[dict[str, object]], key: str) -> float | None:
    values = finite_values(rows, key)
    return median(values) if values else None


def x_series(rows: list[dict[str, object]]) -> list[float]:
    stamps = [finite_float(row.get("stamp")) for row in rows]
    finite = [v for v in stamps if v is not None]
    if not finite:
        return list(range(len(rows)))
    t0 = finite[0]
    return [(v - t0) if v is not None else float(i) for i, v in enumerate(stamps)]


def save(fig, path: Path) -> Path:
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return path


def normalize_axis(x: float, y: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= 1.0e-12:
        return (1.0, 0.0)
    return (x / norm, y / norm)


def matrix_from_row(row: dict[str, str], prefix: str) -> list[list[float]] | None:
    matrix: list[list[float]] = []
    for r in range(3):
        values: list[float] = []
        for c in range(3):
            value = finite_float(row.get(f"{prefix}{r}{c}"))
            if value is None:
                return None
            values.append(value)
        matrix.append(values)
    return matrix


def quadratic_form(matrix: list[list[float]], vector: tuple[float, float, float]) -> float:
    tmp = [
        sum(matrix[r][c] * vector[c] for c in range(3))
        for r in range(3)
    ]
    return sum(vector[i] * tmp[i] for i in range(3))


def symmetric_eigenvalues_3x3(matrix: list[list[float]] | None) -> tuple[float, float, float]:
    if matrix is None:
        return (math.nan, math.nan, math.nan)
    a = [[0.5 * (matrix[r][c] + matrix[c][r]) for c in range(3)] for r in range(3)]
    for _ in range(24):
        pairs = ((0, 1), (0, 2), (1, 2))
        p, q = max(pairs, key=lambda item: abs(a[item[0]][item[1]]))
        if abs(a[p][q]) < 1.0e-12:
            break
        tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q])
        t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau))
        c = 1.0 / math.sqrt(1.0 + t * t)
        s = t * c
        app = a[p][p]
        aqq = a[q][q]
        apq = a[p][q]
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = 0.0
        a[q][p] = 0.0
        for k in range(3):
            if k in (p, q):
                continue
            akp = a[k][p]
            akq = a[k][q]
            a[k][p] = c * akp - s * akq
            a[p][k] = a[k][p]
            a[k][q] = s * akp + c * akq
            a[q][k] = a[k][q]
    return tuple(sorted((a[0][0], a[1][1], a[2][2])))


def reason_text(row: dict[str, str]) -> str:
    parts = []
    for prefix, key in (
        ("selected", "selected_fallback_reason"),
        ("module", "module_fallback_reason"),
        ("lidar", "lidar_reason"),
        ("fused", "fused_reason"),
    ):
        value = (row.get(key) or "").strip()
        if value:
            parts.append(f"{prefix}:{value}")
    return ";".join(parts)


def derive_gate_reason(
    main_row: dict[str, str],
    weak_axis_ratio: float | None,
    degeneracy_score: float | None,
    condition_threshold: float,
    degeneracy_threshold: float,
    weak_axis_ratio_threshold: float,
) -> str:
    direct = reason_text(main_row)
    if direct:
        return direct
    if not is_true(main_row.get("lidar_valid")):
        return "lidar_invalid"
    condition = finite_float(main_row.get("lidar_lambda_condition"))
    if condition is None:
        return "lidar_condition_missing"
    if condition >= condition_threshold:
        return "lidar_condition_high"
    if degeneracy_score is not None and degeneracy_score >= degeneracy_threshold:
        return "lidar_degeneracy_score_high"
    if weak_axis_ratio is not None and weak_axis_ratio < weak_axis_ratio_threshold:
        return "along_corridor_information_weak"
    return "lidar_allowed"


def derive_rows(
    rows: list[dict[str, str]],
    fusion_rows: list[dict[str, str]],
    axis_x: float,
    axis_y: float,
    condition_threshold: float,
    degeneracy_threshold: float,
    weak_axis_ratio_threshold: float,
) -> list[dict[str, object]]:
    axis_x, axis_y = normalize_axis(axis_x, axis_y)
    along_axis = (axis_x, axis_y, 0.0)
    cross_axis = (-axis_y, axis_x, 0.0)
    fusion_by_query = {row.get("query_index", ""): row for row in fusion_rows}
    derived: list[dict[str, object]] = []
    for row in rows:
        fusion_row = fusion_by_query.get(row.get("query_index", ""), {})
        matrix = matrix_from_row(fusion_row, "lambda_lidar_") if fusion_row else None
        along = quadratic_form(matrix, along_axis) if matrix is not None else math.nan
        cross = quadratic_form(matrix, cross_axis) if matrix is not None else math.nan
        z_info = matrix[2][2] if matrix is not None else math.nan
        eig_min, eig_mid, eig_max = symmetric_eigenvalues_3x3(matrix)
        weak_axis_ratio = (
            along / max(cross, 1.0e-12)
            if math.isfinite(along) and math.isfinite(cross)
            else math.nan
        )
        degeneracy_score = (
            max(cross, z_info, eig_max, 0.0) / max(along, 1.0e-12)
            if math.isfinite(along) and math.isfinite(cross)
            else math.nan
        )
        condition = finite_float(row.get("lidar_lambda_condition"))
        weak_ratio_value = weak_axis_ratio if math.isfinite(weak_axis_ratio) else None
        degeneracy_value = degeneracy_score if math.isfinite(degeneracy_score) else None
        gate_reason = derive_gate_reason(
            row,
            weak_ratio_value,
            degeneracy_value,
            condition_threshold,
            degeneracy_threshold,
            weak_axis_ratio_threshold,
        )
        lidar_allowed = (
            is_true(row.get("lidar_valid"))
            and condition is not None
            and condition < condition_threshold
            and weak_ratio_value is not None
            and weak_ratio_value >= weak_axis_ratio_threshold
            and degeneracy_value is not None
            and degeneracy_value < degeneracy_threshold
        )
        derived.append({
            "stamp": row.get("stamp", ""),
            "query_index": row.get("query_index", ""),
            "query_label": row.get("query_label", ""),
            "query_x": row.get("query_x", ""),
            "query_y": row.get("query_y", ""),
            "query_z": row.get("query_z", ""),
            "corridor_axis_x": axis_x,
            "corridor_axis_y": axis_y,
            "selected_source": row.get("selected_source", ""),
            "selected_valid": row.get("selected_valid", ""),
            "selected_fallback": row.get("selected_fallback", ""),
            "lidar_valid": row.get("lidar_valid", ""),
            "lidar_fallback": row.get("lidar_fallback", ""),
            "fused_valid": row.get("fused_valid", ""),
            "fused_lidar_used": row.get("fused_lidar_used", ""),
            "lidar_n_primitives": row.get("lidar_n_primitives", ""),
            "lidar_n_valid_normals": row.get("lidar_n_valid_normals", ""),
            "lidar_condition": row.get("lidar_lambda_condition", ""),
            "lidar_min_eig": row.get("lidar_lambda_min_eig", ""),
            "lidar_lambda_trace": row.get("lidar_lambda_trace", ""),
            "lidar_alpha": row.get("lidar_alpha", ""),
            "lidar_tdop": row.get("lidar_tdop", ""),
            "along_corridor_information": along,
            "cross_corridor_information": cross,
            "vertical_information": z_info,
            "weak_axis_ratio": weak_axis_ratio,
            "degeneracy_score": degeneracy_score,
            "lidar_eig_min": eig_min,
            "lidar_eig_mid": eig_mid,
            "lidar_eig_max": eig_max,
            "lidar_allowed_for_fusion": "1" if lidar_allowed else "0",
            "gate_reason": gate_reason,
        })
    return derived


def reason_histogram(rows: list[dict[str, object]]) -> dict[str, int]:
    counter = Counter(str(row.get("gate_reason", "")) for row in rows)
    counter.pop("", None)
    return dict(sorted(counter.items()))


def missing_or_empty(required: dict[str, Path]) -> list[str]:
    return [
        name for name, path in required.items()
        if not path.is_file() or path.stat().st_size == 0
    ]


def plot_corridor_map(
    derived_rows: list[dict[str, object]],
    map_rows: list[dict[str, str]],
    figures_dir: Path,
    axis_x: float,
    axis_y: float,
) -> Path:
    fig, ax = plt.subplots(figsize=(8, 6))
    xs = finite_values(map_rows, "x")
    ys = finite_values(map_rows, "y")
    if xs and len(xs) == len(ys):
        ax.scatter(xs, ys, s=5, color="#64748b", alpha=0.35, label="downsampled map")
    qx = finite_values(derived_rows, "query_x")
    qy = finite_values(derived_rows, "query_y")
    if qx and len(qx) == len(qy):
        ax.scatter(qx, qy, s=22, color="#dc2626", alpha=0.8, label="query")
    axis_x, axis_y = normalize_axis(axis_x, axis_y)
    ax.arrow(0.0, 0.0, 4.0 * axis_x, 4.0 * axis_y, color="#f97316",
             width=0.03, length_includes_head=True, label="corridor axis")
    ax.set_title("Experiment 10 corridor map top-down")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend()
    return save(fig, figures_dir / "E10_corridor_map_topdown.png")


def plot_condition_timeline(rows: list[dict[str, object]], figures_dir: Path) -> Path:
    x = x_series(rows)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(x, value_series(rows, "lidar_condition"), color="#dc2626",
                 label="LiDAR condition")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("condition")
    axes[0].legend()
    axes[1].plot(x, value_series(rows, "degeneracy_score"), color="#7c3aed",
                 label="degeneracy score")
    axes[1].set_yscale("log")
    axes[1].set_ylabel("score")
    axes[1].legend()
    axes[2].plot(x, value_series(rows, "lidar_alpha"), color="#16a34a",
                 label="lidar alpha")
    axes[2].plot(x, value_series(rows, "lidar_tdop"), color="#0891b2",
                 label="lidar TDOP")
    axes[2].set_ylabel("alpha / TDOP")
    axes[2].set_xlabel("time since first query [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Experiment 10 LiDAR corridor degeneracy diagnostics")
    return save(fig, figures_dir / "E10_lidar_condition_timeline.png")


def plot_axis_information(rows: list[dict[str, object]], figures_dir: Path) -> Path:
    x = x_series(rows)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(x, value_series(rows, "along_corridor_information"),
                 color="#f97316", label="along corridor information")
    axes[0].plot(x, value_series(rows, "cross_corridor_information"),
                 color="#2563eb", label="cross corridor information")
    axes[0].set_yscale("symlog", linthresh=1.0e-9)
    axes[0].set_ylabel("information")
    axes[0].legend()
    axes[1].plot(x, value_series(rows, "weak_axis_ratio"),
                 color="#111827", label="along / cross ratio")
    axes[1].set_ylabel("ratio")
    axes[1].set_xlabel("time since first query [s]")
    axes[1].legend()
    for ax in axes:
        ax.grid(True, alpha=0.25)
    fig.suptitle("Experiment 10 along-vs-cross corridor information")
    return save(fig, figures_dir / "E10_along_vs_cross_corridor_information.png")


def plot_gate_timeline(rows: list[dict[str, object]], figures_dir: Path) -> Path:
    x = x_series(rows)
    allowed = [1.0 if str(row.get("lidar_allowed_for_fusion")) == "1" else 0.0 for row in rows]
    selected_fallback = [1.0 if is_true(row.get("selected_fallback")) else 0.0 for row in rows]
    lidar_valid = [1.0 if is_true(row.get("lidar_valid")) else 0.0 for row in rows]
    fig, ax = plt.subplots(figsize=(11, 4.8))
    ax.step(x, lidar_valid, where="post", label="lidar valid", color="#16a34a")
    ax.step(x, allowed, where="post", label="lidar allowed for fusion", color="#2563eb")
    ax.step(x, selected_fallback, where="post", label="selected fallback", color="#dc2626")
    ax.set_ylim(-0.1, 1.1)
    ax.set_yticks([0, 1], ["false", "true"])
    ax.set_xlabel("time since first query [s]")
    ax.set_title("Experiment 10 LiDAR gate/fallback timeline")
    ax.grid(True, alpha=0.25)
    ax.legend()
    return save(fig, figures_dir / "E10_lidar_gate_timeline.png")


def plot_eigenvalues(rows: list[dict[str, object]], figures_dir: Path) -> Path:
    x = x_series(rows)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(x, value_series(rows, "lidar_eig_min"), label="min eig", color="#2563eb")
    ax.plot(x, value_series(rows, "lidar_eig_mid"), label="mid eig", color="#f97316")
    ax.plot(x, value_series(rows, "lidar_eig_max"), label="max eig", color="#16a34a")
    ax.set_yscale("symlog", linthresh=1.0e-9)
    ax.set_title("Experiment 10 LiDAR matrix eigenvalues")
    ax.set_xlabel("time since first query [s]")
    ax.set_ylabel("information eigenvalue")
    ax.grid(True, alpha=0.25)
    ax.legend()
    return save(fig, figures_dir / "E10_lidar_eigenvalues_timeline.png")


def analyze(
    rows: list[dict[str, str]],
    lidar_rows: list[dict[str, str]],
    fusion_rows: list[dict[str, str]],
    map_rows: list[dict[str, str]],
    timing_rows: list[dict[str, str]],
    derived_rows: list[dict[str, object]],
    required_files: dict[str, Path],
    args: argparse.Namespace,
) -> dict[str, object]:
    failures: list[str] = []
    query_count = len(rows)
    derived_count = len(derived_rows)
    diagnostic_rows = rows
    nonfinite_fields: list[dict[str, object]] = []
    for index, row in enumerate(diagnostic_rows, start=1):
        for field in DIAGNOSTIC_FIELDS:
            if finite_float(row.get(field)) is None:
                nonfinite_fields.append({
                    "row": index,
                    "query_index": row.get("query_index", ""),
                    "field": field,
                })

    weak_axis_ratio_median = median_value(derived_rows, "weak_axis_ratio")
    condition_median = median_value(derived_rows, "lidar_condition")
    degeneracy_median = median_value(derived_rows, "degeneracy_score")
    allowed_false_count = sum(
        1 for row in derived_rows if str(row.get("lidar_allowed_for_fusion")) != "1"
    )
    gate_reason_count = sum(1 for row in derived_rows if str(row.get("gate_reason", "")).strip())
    gate_reason_empty_rows = [
        {
            "query_index": row.get("query_index", ""),
            "selected_fallback": row.get("selected_fallback", ""),
            "lidar_allowed_for_fusion": row.get("lidar_allowed_for_fusion", ""),
        }
        for row in derived_rows
        if (
            str(row.get("lidar_allowed_for_fusion")) != "1"
            or is_true(row.get("selected_fallback"))
        )
        and not str(row.get("gate_reason", "")).strip()
    ]

    latency_rows = timing_rows if timing_rows else rows
    module_total = finite_values(latency_rows, "module_total_us")
    module_total_p95 = percentile(module_total, 0.95)
    missing = missing_or_empty(required_files)

    if query_count == 0:
        failures.append("no predictor rows found")
    if not lidar_rows:
        failures.append("no lidar debug rows found")
    if not fusion_rows:
        failures.append("no fusion debug rows found")
    if not map_rows:
        failures.append("no downsampled map rows found")
    if missing:
        failures.append("missing or empty required files: " + ",".join(missing))
    if nonfinite_fields:
        failures.append(f"non-finite diagnostic fields: {len(nonfinite_fields)}")
    if weak_axis_ratio_median is None or weak_axis_ratio_median >= args.weak_axis_ratio_threshold:
        failures.append(
            "median weak_axis_ratio "
            f"{weak_axis_ratio_median} >= {args.weak_axis_ratio_threshold}"
        )
    condition_ok = condition_median is not None and condition_median >= args.degeneracy_threshold
    degeneracy_ok = (
        degeneracy_median is not None
        and degeneracy_median >= args.degeneracy_threshold
    )
    if not (condition_ok or degeneracy_ok):
        failures.append(
            "median lidar condition/degeneracy below threshold: "
            f"condition={condition_median}, degeneracy={degeneracy_median}, "
            f"threshold={args.degeneracy_threshold}"
        )
    allowed_false_ratio = ratio(allowed_false_count, derived_count)
    gate_reason_ratio = ratio(gate_reason_count, derived_count)
    if allowed_false_ratio <= 0.80 and gate_reason_ratio <= 0.80:
        failures.append(
            "lidar gate evidence too weak: "
            f"allowed_false_ratio={allowed_false_ratio:.3f}, "
            f"gate_reason_ratio={gate_reason_ratio:.3f}"
        )
    if gate_reason_empty_rows:
        failures.append(f"empty gate reason rows: {len(gate_reason_empty_rows)}")
    if module_total_p95 is None or module_total_p95 >= 10000.0:
        failures.append(f"module_total_us p95 {module_total_p95} >= 10000")

    return {
        "passed": not failures,
        "failures": failures,
        "query_count": query_count,
        "lidar_debug_rows": len(lidar_rows),
        "fusion_debug_rows": len(fusion_rows),
        "downsampled_map_rows": len(map_rows),
        "derived_row_count": derived_count,
        "corridor_axis_x": args.corridor_axis_x,
        "corridor_axis_y": args.corridor_axis_y,
        "weak_axis_ratio_threshold": args.weak_axis_ratio_threshold,
        "condition_threshold": args.condition_threshold,
        "degeneracy_threshold": args.degeneracy_threshold,
        "median_weak_axis_ratio": weak_axis_ratio_median,
        "median_along_corridor_information": median_value(
            derived_rows, "along_corridor_information"
        ),
        "median_cross_corridor_information": median_value(
            derived_rows, "cross_corridor_information"
        ),
        "median_lidar_condition": condition_median,
        "median_lidar_min_eig": median_value(derived_rows, "lidar_min_eig"),
        "median_lidar_alpha": median_value(derived_rows, "lidar_alpha"),
        "median_lidar_tdop": median_value(derived_rows, "lidar_tdop"),
        "median_degeneracy_score": degeneracy_median,
        "median_lidar_n_primitives": median_value(derived_rows, "lidar_n_primitives"),
        "lidar_allowed_for_fusion_false_ratio": allowed_false_ratio,
        "gate_reason_nonempty_ratio": gate_reason_ratio,
        "gate_reason_empty_rows": gate_reason_empty_rows[:50],
        "gate_reason_histogram": reason_histogram(derived_rows),
        "module_total_us": {
            "p50": percentile(module_total, 0.50),
            "p95": module_total_p95,
            "max": max(module_total) if module_total else None,
            "median": median(module_total) if module_total else None,
        },
        "nonfinite_diagnostic_fields": nonfinite_fields[:50],
    }


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
    export_dir.mkdir(parents=True, exist_ok=True)
    return run_dir, export_dir, profiling_dir, figures_dir


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--run-dir")
    group.add_argument("--export-dir")
    parser.add_argument("--fail-on-threshold", action="store_true")
    parser.add_argument("--corridor-axis-x", type=float, default=1.0)
    parser.add_argument("--corridor-axis-y", type=float, default=0.0)
    parser.add_argument("--condition-threshold", type=float, default=1.0e6)
    parser.add_argument("--degeneracy-threshold", type=float, default=1.0e3)
    parser.add_argument("--weak-axis-ratio-threshold", type=float, default=0.25)
    args = parser.parse_args()

    run_dir, export_dir, profiling_dir, figures_dir = resolve_dirs(args)
    csv_path = export_dir / "test_predictor_query_probe.csv"
    lidar_debug_path = export_dir / "predictor_lidar_debug.csv"
    fusion_debug_path = export_dir / "predictor_fusion_debug.csv"
    map_snapshot_path = export_dir / "downsampled_map.csv"
    fallback_path = export_dir / "fallback_reason_by_time.csv"
    timing_path = profiling_dir / "latency_debug.csv"

    rows = read_csv(csv_path)
    lidar_rows = read_csv(lidar_debug_path)
    fusion_rows = read_csv(fusion_debug_path)
    map_rows = read_csv(map_snapshot_path)
    timing_rows = read_csv(timing_path)
    derived_rows = derive_rows(
        rows,
        fusion_rows,
        args.corridor_axis_x,
        args.corridor_axis_y,
        args.condition_threshold,
        args.degeneracy_threshold,
        args.weak_axis_ratio_threshold,
    )

    derived_csv_path = export_dir / "lidar_corridor_degeneracy_system.csv"
    derived_fields = [
        "stamp",
        "query_index",
        "query_label",
        "query_x",
        "query_y",
        "query_z",
        "corridor_axis_x",
        "corridor_axis_y",
        "selected_source",
        "selected_valid",
        "selected_fallback",
        "lidar_valid",
        "lidar_fallback",
        "fused_valid",
        "fused_lidar_used",
        "lidar_n_primitives",
        "lidar_n_valid_normals",
        "lidar_condition",
        "lidar_min_eig",
        "lidar_lambda_trace",
        "lidar_alpha",
        "lidar_tdop",
        "along_corridor_information",
        "cross_corridor_information",
        "vertical_information",
        "weak_axis_ratio",
        "degeneracy_score",
        "lidar_eig_min",
        "lidar_eig_mid",
        "lidar_eig_max",
        "lidar_allowed_for_fusion",
        "gate_reason",
    ]
    write_csv(derived_csv_path, derived_fields, derived_rows)

    required_files = {
        "test_predictor_query_probe.csv": csv_path,
        "predictor_lidar_debug.csv": lidar_debug_path,
        "predictor_fusion_debug.csv": fusion_debug_path,
        "downsampled_map.csv": map_snapshot_path,
        "fallback_reason_by_time.csv": fallback_path,
        "latency_debug.csv": timing_path,
    }
    summary = analyze(
        rows,
        lidar_rows,
        fusion_rows,
        map_rows,
        timing_rows,
        derived_rows,
        required_files,
        args,
    )

    figures: list[str] = []
    if derived_rows:
        figures.extend([
            str(plot_condition_timeline(derived_rows, figures_dir)),
            str(plot_axis_information(derived_rows, figures_dir)),
            str(plot_gate_timeline(derived_rows, figures_dir)),
            str(plot_eigenvalues(derived_rows, figures_dir)),
        ])
    if derived_rows or map_rows:
        figures.append(str(plot_corridor_map(
            derived_rows,
            map_rows,
            figures_dir,
            args.corridor_axis_x,
            args.corridor_axis_y,
        )))

    summary.update({
        "run_dir": str(run_dir),
        "export_dir": str(export_dir),
        "figures_dir": str(figures_dir),
        "csv_path": str(csv_path),
        "lidar_debug_path": str(lidar_debug_path),
        "fusion_debug_path": str(fusion_debug_path),
        "map_snapshot_path": str(map_snapshot_path),
        "fallback_path": str(fallback_path),
        "timing_path": str(timing_path),
        "derived_csv_path": str(derived_csv_path),
        "figures": figures,
    })
    out_path = export_dir / "predictor_e10_analysis_summary.json"
    write_json(out_path, summary)
    print(json.dumps(summary, indent=2))
    return 2 if args.fail_on_threshold and not summary["passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
