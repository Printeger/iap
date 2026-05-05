#!/usr/bin/env python3
import argparse
import bisect
import csv
import json
import math
import statistics
import sys
from pathlib import Path


ALIGNED_FIELDS = [
    "traj_id",
    "sample_index",
    "sample_abs_time",
    "pred_x",
    "pred_y",
    "pred_z",
    "executed_truth_x",
    "executed_truth_y",
    "executed_truth_z",
    "executed_iap_x",
    "executed_iap_y",
    "executed_iap_z",
    "pred_AL",
    "pred_PL",
    "pred_IM",
    "actual_HPL",
    "actual_VPL",
    "actual_PL",
    "actual_AL",
    "actual_IM",
    "time_alignment_error_s",
    "spatial_tracking_error",
    "estimation_error",
    "pred_actual_PL_error",
    "pred_actual_IM_error",
    "same_safe_unsafe_label",
]


def load_csv(path):
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def load_json(path):
    if not path.exists() or path.stat().st_size == 0:
        return {}
    with path.open() as f:
        return json.load(f)


def as_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def finite(value):
    return math.isfinite(as_float(value))


def fmt(value):
    value = as_float(value)
    return f"{value:.9f}" if math.isfinite(value) else "nan"


def mean(values):
    vals = [as_float(v) for v in values if finite(v)]
    return sum(vals) / len(vals) if vals else None


def quantile(values, q):
    vals = sorted(as_float(v) for v in values if finite(v))
    if not vals:
        return None
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - pos) + vals[hi] * (pos - lo)


def build_index(rows, time_key):
    indexed = []
    for row in rows:
        t = as_float(row.get(time_key))
        if math.isfinite(t):
            indexed.append((t, row))
    indexed.sort(key=lambda item: item[0])
    return indexed


def nearest(indexed, target, tolerance):
    target = as_float(target)
    if not indexed or not math.isfinite(target):
        return None, math.nan
    times = [item[0] for item in indexed]
    pos = bisect.bisect_left(times, target)
    candidates = []
    if pos < len(indexed):
        candidates.append(indexed[pos])
    if pos > 0:
        candidates.append(indexed[pos - 1])
    if not candidates:
        return None, math.nan
    best_t, best_row = min(candidates, key=lambda item: abs(item[0] - target))
    dt = abs(best_t - target)
    if dt <= tolerance:
        return best_row, dt
    return None, math.nan


def norm3(ax, ay, az, bx, by, bz):
    vals = [as_float(v) for v in (ax, ay, az, bx, by, bz)]
    if not all(math.isfinite(v) for v in vals):
        return math.nan
    return math.sqrt(
        (vals[0] - vals[3]) ** 2 + (vals[1] - vals[4]) ** 2 + (vals[2] - vals[5]) ** 2
    )


def label_from_im(value):
    v = as_float(value)
    if not math.isfinite(v):
        return ""
    if v >= 0.0:
        return "SAFE"
    return "UNSAFE"


def safe_label_agreement(pred_im, actual_im):
    pred = label_from_im(pred_im)
    actual = label_from_im(actual_im)
    if not pred or not actual:
        return ""
    return "true" if pred == actual else "false"


def araim_values(row):
    if row is None:
        return (math.nan,) * 5
    hpl = as_float(row.get("HPL"))
    vpl = as_float(row.get("VPL"))
    hal = as_float(row.get("HAL"))
    val = as_float(row.get("VAL"))
    actual_pl = max(hpl, vpl) if math.isfinite(hpl) and math.isfinite(vpl) else math.nan
    actual_al = min(hal, val) if math.isfinite(hal) and math.isfinite(val) else math.nan
    actual_im = as_float(row.get("IM"))
    if not math.isfinite(actual_im) and math.isfinite(actual_al) and math.isfinite(actual_pl):
        actual_im = actual_al - actual_pl
    return hpl, vpl, actual_pl, actual_al, actual_im


def estimate_desired_offset(pred_rows, desired_index, tolerance):
    if not pred_rows or not desired_index:
        return 0.0
    sample = pred_rows[: min(100, len(pred_rows))]
    direct = 0
    pred_times = []
    for row in sample:
        t = as_float(row.get("stamp")) + as_float(row.get("sample_t_from_now"))
        if math.isfinite(t):
            pred_times.append(t)
            match, _ = nearest(desired_index, t, tolerance)
            if match is not None:
                direct += 1
    if not pred_times or direct / max(1, len(pred_times)) >= 0.10:
        return 0.0
    desired_times = [item[0] for item in desired_index[: min(len(desired_index), len(pred_times))]]
    if not desired_times:
        return 0.0
    return statistics.median(desired_times) - statistics.median(pred_times)


def summarize(pred_rows, aligned_rows, online_summary, run_dir, warnings, errors):
    im_values = [as_float(r.get("IM_pred")) for r in pred_rows if finite(r.get("IM_pred"))]
    pl_values = [as_float(r.get("PL_pred")) for r in pred_rows if finite(r.get("PL_pred"))]
    risk_counts = {}
    for row in pred_rows:
        key = row.get("risk_state_pred") or "UNKNOWN_PL"
        risk_counts[key] = risk_counts.get(key, 0) + 1

    matched_rows = [r for r in aligned_rows if finite(r.get("actual_PL"))]
    agreement_values = [
        r.get("same_safe_unsafe_label")
        for r in aligned_rows
        if r.get("same_safe_unsafe_label") in ("true", "false")
    ]
    agreement = None
    if agreement_values:
        agreement = sum(1 for v in agreement_values if v == "true") / len(agreement_values)

    predicted = {
        "safe_count": int(risk_counts.get("SAFE_PRED", 0)),
        "marginal_count": int(risk_counts.get("MARGINAL_PRED", 0)),
        "unsafe_count": int(risk_counts.get("UNSAFE_PRED", 0)),
        "unknown_count": int(risk_counts.get("UNKNOWN_PL", 0) + risk_counts.get("UNKNOWN_AL", 0)),
        "min_IM": min(im_values) if im_values else None,
        "mean_IM": mean(im_values),
        "p05_IM": quantile(im_values, 0.05),
        "p50_IM": quantile(im_values, 0.50),
        "p95_PL": quantile(pl_values, 0.95),
        "max_PL": max(pl_values) if pl_values else None,
    }

    return {
        "available": True,
        "run_dir": str(run_dir),
        "traj_count": int(online_summary.get("traj_count") or len(set(r.get("traj_id") for r in pred_rows))),
        "sample_count": len(pred_rows),
        "aligned_sample_count": len(matched_rows),
        "online_truth_used": bool(online_summary.get("online_truth_used", False)),
        "odom_source": online_summary.get("odom_source", "/drone_0_visual_slam/odom"),
        "map_source": online_summary.get("map_source", ""),
        "pl_model": online_summary.get("pl_model", ""),
        "al_model": online_summary.get("al_model", ""),
        "sampling": online_summary.get(
            "sampling",
            {"horizon_s": 0.0, "dt_s": 0.0, "max_samples_per_traj": 0},
        ),
        "predicted_integrity": predicted,
        "actual_alignment": {
            "matched_count": len(matched_rows),
            "match_ratio": len(matched_rows) / len(pred_rows) if pred_rows else 0.0,
            "mean_time_alignment_error_s": mean(r.get("time_alignment_error_s") for r in matched_rows),
            "mean_spatial_tracking_error": mean(r.get("spatial_tracking_error") for r in aligned_rows),
            "mean_estimation_error": mean(r.get("estimation_error") for r in aligned_rows),
            "mean_pred_actual_PL_error": mean(r.get("pred_actual_PL_error") for r in matched_rows),
            "mean_pred_actual_IM_error": mean(r.get("pred_actual_IM_error") for r in matched_rows),
            "safe_unsafe_label_agreement_ratio": agreement,
        },
        "warnings": list(dict.fromkeys((online_summary.get("warnings") or []) + warnings)),
        "errors": errors,
    }


def analyze(run_dir, match_tolerance_s):
    export_dir = run_dir / "export"
    pred_path = export_dir / "integrity_along_planner_traj.csv"
    online_summary_path = export_dir / "phase2_summary.json"
    aligned_path = export_dir / "phase2_integrity_eval_aligned.csv"

    pred_rows = load_csv(pred_path)
    online_summary = load_json(online_summary_path)
    warnings = []
    errors = []

    if not pred_rows:
        errors.append("export/integrity_along_planner_traj.csv is missing or empty")

    araim_rows = [
        row for row in load_csv(export_dir / "iap_araim.csv")
        if (row.get("row_type") or "").strip() == "epoch"
    ]
    desired_rows = load_csv(export_dir / "desired_vs_truth.csv")
    sim_rows = load_csv(export_dir / "iap_sim_truth_vs_est.csv")
    if not araim_rows:
        warnings.append("export/iap_araim.csv unavailable; actual PL/IM alignment is NaN")
    if not desired_rows:
        warnings.append("export/desired_vs_truth.csv unavailable; tracking alignment is NaN")
    if not sim_rows:
        warnings.append("export/iap_sim_truth_vs_est.csv unavailable; estimation alignment is NaN")

    araim_index = build_index(araim_rows, "stamp")
    desired_index = build_index(desired_rows, "stamp")
    sim_index = build_index(sim_rows, "est_stamp")
    desired_offset = estimate_desired_offset(pred_rows, desired_index, match_tolerance_s)
    if abs(desired_offset) > 1.0:
        warnings.append(f"applied desired/truth time offset {desired_offset:.6f}s")

    aligned_rows = []
    for pred in pred_rows:
        sample_abs_time = as_float(pred.get("sample_abs_time"))
        planner_sample_time = as_float(pred.get("stamp")) + as_float(pred.get("sample_t_from_now"))

        araim_row, araim_dt = nearest(araim_index, sample_abs_time, match_tolerance_s)
        desired_row, desired_dt = nearest(
            desired_index, planner_sample_time + desired_offset, match_tolerance_s
        )
        sim_row, sim_dt = nearest(sim_index, sample_abs_time, match_tolerance_s)

        actual_hpl, actual_vpl, actual_pl, actual_al, actual_im = araim_values(araim_row)
        pred_al = as_float(pred.get("AL_pred"))
        pred_pl = as_float(pred.get("PL_pred"))
        if not math.isfinite(pred_pl) and math.isfinite(actual_pl):
            pred_pl = actual_pl
        pred_im = as_float(pred.get("IM_pred"))
        if not math.isfinite(pred_im) and math.isfinite(pred_al) and math.isfinite(pred_pl):
            pred_im = pred_al - pred_pl

        truth_x = desired_row.get("truth_x") if desired_row else math.nan
        truth_y = desired_row.get("truth_y") if desired_row else math.nan
        truth_z = desired_row.get("truth_z") if desired_row else math.nan
        iap_x = desired_row.get("iap_x") if desired_row else math.nan
        iap_y = desired_row.get("iap_y") if desired_row else math.nan
        iap_z = desired_row.get("iap_z") if desired_row else math.nan
        if sim_row is not None:
            if not finite(iap_x):
                iap_x = sim_row.get("est_x")
            if not finite(iap_y):
                iap_y = sim_row.get("est_y")
            if not finite(iap_z):
                iap_z = sim_row.get("est_z")
            if not finite(truth_x):
                truth_x = sim_row.get("truth_x")
            if not finite(truth_y):
                truth_y = sim_row.get("truth_y")
            if not finite(truth_z):
                truth_z = sim_row.get("truth_z")

        spatial_tracking_error = norm3(
            pred.get("x"), pred.get("y"), pred.get("z"), truth_x, truth_y, truth_z
        )
        estimation_error = as_float(desired_row.get("estimation_position_error")) if desired_row else math.nan
        if not math.isfinite(estimation_error) and sim_row is not None:
            estimation_error = as_float(sim_row.get("position_error_m"))

        pred_actual_pl_error = pred_pl - actual_pl if math.isfinite(pred_pl) and math.isfinite(actual_pl) else math.nan
        pred_actual_im_error = pred_im - actual_im if math.isfinite(pred_im) and math.isfinite(actual_im) else math.nan
        time_error_candidates = [v for v in (araim_dt, desired_dt, sim_dt) if math.isfinite(v)]
        time_alignment_error = max(time_error_candidates) if time_error_candidates else math.nan

        aligned_rows.append({
            "traj_id": pred.get("traj_id", ""),
            "sample_index": pred.get("sample_index", ""),
            "sample_abs_time": fmt(sample_abs_time),
            "pred_x": fmt(pred.get("x")),
            "pred_y": fmt(pred.get("y")),
            "pred_z": fmt(pred.get("z")),
            "executed_truth_x": fmt(truth_x),
            "executed_truth_y": fmt(truth_y),
            "executed_truth_z": fmt(truth_z),
            "executed_iap_x": fmt(iap_x),
            "executed_iap_y": fmt(iap_y),
            "executed_iap_z": fmt(iap_z),
            "pred_AL": fmt(pred_al),
            "pred_PL": fmt(pred_pl),
            "pred_IM": fmt(pred_im),
            "actual_HPL": fmt(actual_hpl),
            "actual_VPL": fmt(actual_vpl),
            "actual_PL": fmt(actual_pl),
            "actual_AL": fmt(actual_al),
            "actual_IM": fmt(actual_im),
            "time_alignment_error_s": fmt(time_alignment_error),
            "spatial_tracking_error": fmt(spatial_tracking_error),
            "estimation_error": fmt(estimation_error),
            "pred_actual_PL_error": fmt(pred_actual_pl_error),
            "pred_actual_IM_error": fmt(pred_actual_im_error),
            "same_safe_unsafe_label": safe_label_agreement(pred_im, actual_im),
        })

    export_dir.mkdir(parents=True, exist_ok=True)
    with aligned_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=ALIGNED_FIELDS)
        writer.writeheader()
        writer.writerows(aligned_rows)

    if not any(finite(r.get("actual_AL")) for r in aligned_rows):
        warnings.append("actual_AL could not be computed for aligned samples")
    if not any(finite(r.get("actual_IM")) for r in aligned_rows):
        warnings.append("actual_IM could not be computed for aligned samples")

    summary = summarize(pred_rows, aligned_rows, online_summary, run_dir, warnings, errors)
    (export_dir / "phase2_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main():
    parser = argparse.ArgumentParser(description="Analyze Phase 2 PI-lite trajectory integrity evaluation.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--match-tolerance-s", type=float, default=0.10)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.exists():
        print(f"Run directory does not exist: {run_dir}", file=sys.stderr)
        return 2

    summary = analyze(run_dir, args.match_tolerance_s)
    print(f"Analyzed run: {run_dir}")
    print(f"sample_count: {summary['sample_count']}")
    print(f"aligned_sample_count: {summary['aligned_sample_count']}")
    print(f"match_ratio: {summary['actual_alignment']['match_ratio']:.3f}")
    if summary["warnings"]:
        print("Warnings:")
        for warning in summary["warnings"]:
            print(f"  - {warning}")
    if summary["errors"]:
        print("Errors:")
        for error in summary["errors"]:
            print(f"  - {error}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
