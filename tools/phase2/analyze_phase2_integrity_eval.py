#!/usr/bin/env python3
import argparse
import bisect
import csv
import html
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


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


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


def max_abs(values):
    vals = [as_float(v) for v in values if finite(v)]
    return max((abs(v) for v in vals), default=None)


def write_svg(path, title, body, width=900, height=320):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join([
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="white"/>',
            f'<text x="18" y="26" font-family="sans-serif" font-size="16">{html.escape(title)}</text>',
            body,
            "</svg>",
            "",
        ])
    )


def svg_line_plot(path, title, series):
    width, height = 900, 320
    left, right, top, bottom = 56, 18, 44, 34
    points = []
    for label, xs, ys, color in series:
        clean = [(as_float(x), as_float(y)) for x, y in zip(xs, ys)]
        clean = [(x, y) for x, y in clean if math.isfinite(x) and math.isfinite(y)]
        if clean:
            points.extend(clean)
    if not points:
        write_svg(path, title, '<text x="56" y="160" font-family="sans-serif" font-size="13">no finite data</text>', width, height)
        return
    xs_all = [p[0] for p in points]
    ys_all = [p[1] for p in points]
    x0, x1 = min(xs_all), max(xs_all)
    y0, y1 = min(ys_all), max(ys_all)
    if x0 == x1:
        x1 = x0 + 1.0
    if y0 == y1:
        y1 = y0 + 1.0
    def sx(x):
        return left + (x - x0) / (x1 - x0) * (width - left - right)
    def sy(y):
        return height - bottom - (y - y0) / (y1 - y0) * (height - top - bottom)
    body = [f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#888"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="#888"/>']
    legend_x = left
    for label, xs, ys, color in series:
        clean = [(as_float(x), as_float(y)) for x, y in zip(xs, ys)]
        clean = [(x, y) for x, y in clean if math.isfinite(x) and math.isfinite(y)]
        if not clean:
            continue
        path_d = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in clean)
        body.append(f'<polyline points="{path_d}" fill="none" stroke="{color}" stroke-width="1.6"/>')
        body.append(f'<rect x="{legend_x}" y="292" width="10" height="10" fill="{color}"/>')
        body.append(f'<text x="{legend_x + 14}" y="302" font-family="sans-serif" font-size="11">{html.escape(label)}</text>')
        legend_x += 110
    write_svg(path, title, "\n".join(body), width, height)


def svg_scatter_plot(path, title, rows, x_key, y_key):
    width, height = 640, 420
    left, right, top, bottom = 58, 20, 44, 42
    pts = [(as_float(r.get(x_key)), as_float(r.get(y_key))) for r in rows]
    pts = [(x, y) for x, y in pts if math.isfinite(x) and math.isfinite(y)]
    if not pts:
        write_svg(path, title, '<text x="58" y="190" font-family="sans-serif" font-size="13">no finite data</text>', width, height)
        return
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    if x0 == x1:
        x1 = x0 + 1.0
    if y0 == y1:
        y1 = y0 + 1.0
    def sx(x):
        return left + (x - x0) / (x1 - x0) * (width - left - right)
    def sy(y):
        return height - bottom - (y - y0) / (y1 - y0) * (height - top - bottom)
    body = [f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#888"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="#888"/>',
            f'<text x="{left}" y="{height-10}" font-family="sans-serif" font-size="11">{html.escape(x_key)}</text>',
            f'<text x="8" y="{top+12}" font-family="sans-serif" font-size="11">{html.escape(y_key)}</text>']
    body.extend(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="2.4" fill="#2b6cb0" opacity="0.55"/>' for x, y in pts)
    write_svg(path, title, "\n".join(body), width, height)


def svg_histogram(path, title, counts):
    width, height = 640, 360
    if not counts:
        write_svg(path, title, '<text x="48" y="170" font-family="sans-serif" font-size="13">no fallback samples</text>', width, height)
        return
    items = sorted(counts.items(), key=lambda item: (-item[1], item[0]))
    max_count = max(count for _, count in items)
    body = []
    for idx, (label, count) in enumerate(items[:12]):
        y = 54 + idx * 24
        w = 480 * count / max_count if max_count else 0
        body.append(f'<rect x="130" y="{y}" width="{w:.1f}" height="16" fill="#c05621"/>')
        body.append(f'<text x="12" y="{y+13}" font-family="sans-serif" font-size="11">{html.escape(label)}</text>')
        body.append(f'<text x="{136+w:.1f}" y="{y+13}" font-family="sans-serif" font-size="11">{count}</text>')
    write_svg(path, title, "\n".join(body), width, height)


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


def summarize_snapshot(snapshot_rows, araim_index, match_tolerance_s, warnings):
    def consistency_dict(ratios, hpl_errors, vpl_errors, matched_hpl_errors=None, matched_vpl_errors=None, matched_im_errors=None):
        matched_hpl_errors = matched_hpl_errors or []
        matched_vpl_errors = matched_vpl_errors or []
        matched_im_errors = matched_im_errors or []
        return {
            "available": bool(ratios),
            "finite_count": len(ratios),
            "warning_threshold_ratio": 0.10,
            "mean_pl_ratio": mean(ratios),
            "max_pl_ratio": max(ratios) if ratios else None,
            "mean_hpl_error": mean(hpl_errors),
            "mean_vpl_error": mean(vpl_errors),
            "max_abs_hpl_error": max_abs(hpl_errors),
            "max_abs_vpl_error": max_abs(vpl_errors),
            "matched_current_araim_count": max(
                len(matched_hpl_errors), len(matched_vpl_errors), len(matched_im_errors)
            ),
            "mean_current_hpl_alignment_error": mean(matched_hpl_errors),
            "mean_current_vpl_alignment_error": mean(matched_vpl_errors),
            "mean_current_im_alignment_error": mean(matched_im_errors),
        }

    if not snapshot_rows:
        return (
            {
                "available": False,
                "sample_count": 0,
                "valid_count": 0,
                "has_epoch_count": 0,
                "missing_epoch_count": 0,
                "pred_now_finite_count": 0,
                "pred_now_fallback_count": 0,
            },
            consistency_dict([], [], []),
            consistency_dict([], [], []),
        )

    def finite_values(column, fallback_column=None):
        values = []
        for row in snapshot_rows:
            value = row.get(column)
            if value is None and fallback_column is not None:
                value = row.get(fallback_column)
            if finite(value):
                values.append(as_float(value))
        return values

    raw_ratios = finite_values("raw_consistency_pl_ratio", "consistency_pl_ratio")
    raw_hpl_errors = finite_values("raw_consistency_hpl_error", "consistency_hpl_error")
    raw_vpl_errors = finite_values("raw_consistency_vpl_error", "consistency_vpl_error")
    anchored_ratios = finite_values("consistency_pl_ratio")
    anchored_hpl_errors = finite_values("consistency_hpl_error")
    anchored_vpl_errors = finite_values("consistency_vpl_error")
    matched_hpl_errors = []
    matched_vpl_errors = []
    matched_im_errors = []
    for row in snapshot_rows:
        araim_row, _ = nearest(araim_index, row.get("stamp"), match_tolerance_s)
        if araim_row is None:
            continue
        actual_hpl, actual_vpl, _, _, actual_im = araim_values(araim_row)
        hpl = as_float(row.get("current_HPL"))
        vpl = as_float(row.get("current_VPL"))
        im = as_float(row.get("current_IM"))
        if math.isfinite(hpl) and math.isfinite(actual_hpl):
            matched_hpl_errors.append(hpl - actual_hpl)
        if math.isfinite(vpl) and math.isfinite(actual_vpl):
            matched_vpl_errors.append(vpl - actual_vpl)
        if math.isfinite(im) and math.isfinite(actual_im):
            matched_im_errors.append(im - actual_im)

    max_ratio = max(raw_ratios) if raw_ratios else None
    max_ratio_row = None
    if max_ratio is not None:
        max_ratio_row = max(
            snapshot_rows,
            key=lambda r: (
                as_float(r.get("raw_consistency_pl_ratio"))
                if finite(r.get("raw_consistency_pl_ratio"))
                else float("-inf")
            ),
        )
    if max_ratio is not None and max_ratio > 0.10:
        warnings.append(f"raw current PL consistency max ratio is {max_ratio:.3f}")

    integrity_snapshot = {
        "available": True,
        "sample_count": len(snapshot_rows),
        "valid_count": sum(1 for r in snapshot_rows if as_bool(r.get("snapshot_valid"))),
        "has_epoch_count": sum(1 for r in snapshot_rows if as_bool(r.get("has_epoch"))),
        "missing_epoch_count": sum(1 for r in snapshot_rows if not as_bool(r.get("has_epoch"))),
        "pred_now_finite_count": sum(
            1
            for r in snapshot_rows
            if finite(r.get("pred_now_raw_pl")) or finite(r.get("pred_now_pl"))
        ),
        "pred_now_fallback_count": sum(1 for r in snapshot_rows if as_bool(r.get("pred_now_fallback"))),
        "csv": "future_integrity_snapshot.csv",
    }
    current_consistency_raw = consistency_dict(
        raw_ratios,
        raw_hpl_errors,
        raw_vpl_errors,
        matched_hpl_errors,
        matched_vpl_errors,
        matched_im_errors,
    )
    if max_ratio_row is not None:
        excluded_prns = max_ratio_row.get("excluded_prns") or ""
        n_detected = as_float(max_ratio_row.get("n_detected"))
        likely_reason = "geometry/noise-state mismatch"
        if as_bool(max_ratio_row.get("pred_now_fallback")):
            likely_reason = "future predictor fallback at current pose"
        elif excluded_prns.strip() and excluded_prns.strip().lower() not in ("nan", "none", "null"):
            likely_reason = "current ARAIM exclusion is not represented in raw future geometry prediction"
        elif math.isfinite(n_detected) and n_detected > 0:
            likely_reason = "current ARAIM detection is not represented in raw future geometry prediction"
        current_consistency_raw["max_ratio_context"] = {
            "stamp": as_float(max_ratio_row.get("stamp")),
            "likely_reason": likely_reason,
            "current_PL": as_float(max_ratio_row.get("current_PL")),
            "pred_now_raw_pl": as_float(max_ratio_row.get("pred_now_raw_pl")),
            "current_HPL": as_float(max_ratio_row.get("current_HPL")),
            "current_VPL": as_float(max_ratio_row.get("current_VPL")),
            "pred_now_raw_hpl": as_float(max_ratio_row.get("pred_now_raw_hpl")),
            "pred_now_raw_vpl": as_float(max_ratio_row.get("pred_now_raw_vpl")),
            "n_sv_used": as_float(max_ratio_row.get("n_sv_used")),
            "pdop": as_float(max_ratio_row.get("pdop")),
            "n_hypotheses": as_float(max_ratio_row.get("n_hypotheses")),
            "n_detected": as_float(max_ratio_row.get("n_detected")),
            "excluded_prns": excluded_prns,
            "has_epoch": as_bool(max_ratio_row.get("has_epoch")),
            "epoch_sat_count": as_float(max_ratio_row.get("epoch_sat_count")),
            "pred_now_n_vis": as_float(max_ratio_row.get("pred_now_n_vis")),
            "pred_now_pdop": as_float(max_ratio_row.get("pred_now_pdop")),
            "pred_now_fallback": as_bool(max_ratio_row.get("pred_now_fallback")),
            "pred_now_fallback_reason": max_ratio_row.get("pred_now_fallback_reason") or "",
        }
    current_consistency_anchored = consistency_dict(
        anchored_ratios,
        anchored_hpl_errors,
        anchored_vpl_errors,
        matched_hpl_errors,
        matched_vpl_errors,
        matched_im_errors,
    )
    return integrity_snapshot, current_consistency_raw, current_consistency_anchored


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
    pi_cost_values = [
        as_float(r.get("pi_cost_total"))
        for r in pred_rows
        if finite(r.get("pi_cost_total"))
    ]
    risk_counts = {}
    pi_risk_counts = {}
    pi_axis_counts = {}
    for row in pred_rows:
        key = row.get("risk_state_pred") or "UNKNOWN_PL"
        risk_counts[key] = risk_counts.get(key, 0) + 1
        pi_key = row.get("pi_risk_band") or "UNKNOWN_PI"
        pi_risk_counts[pi_key] = pi_risk_counts.get(pi_key, 0) + 1
        axis_key = row.get("pi_dominant_axis") or "unknown"
        pi_axis_counts[axis_key] = pi_axis_counts.get(axis_key, 0) + 1
    online_predicted = online_summary.get("predicted_integrity") or {}
    online_pi = online_summary.get("pi_cost") or {}
    fallback_count = sum(1 for row in pred_rows if as_bool(row.get("fallback")))
    fallback_hist = {}
    for row in pred_rows:
        if as_bool(row.get("fallback")):
            reason = row.get("fallback_reason") or "unknown"
            fallback_hist[reason] = fallback_hist.get(reason, 0) + 1
    finite_gnss_prediction_count = sum(
        1
        for row in pred_rows
        if as_bool(row.get("valid")) and row.get("query_source") in ("direct", "grid")
    )

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
        "fallback_count": int(online_predicted.get("fallback_count", fallback_count)),
        "fallback_rate": float(
            online_predicted.get(
                "fallback_rate",
                fallback_count / len(pred_rows) if pred_rows else 0.0,
            )
        ),
        "fallback_reason_histogram": online_predicted.get(
            "fallback_reason_histogram",
            fallback_hist,
        ),
        "finite_gnss_prediction_count": int(
            online_predicted.get(
                "finite_gnss_prediction_count",
                finite_gnss_prediction_count,
            )
        ),
    }
    fallback_rate = predicted["fallback_rate"]
    fallback_reason_histogram = predicted["fallback_reason_histogram"]
    finite_gnss_prediction_count = predicted["finite_gnss_prediction_count"]
    pi_cost = {
        "available": bool(pi_cost_values),
        "count": len(pi_cost_values),
        "weight_h": online_pi.get("weight_h", 1.0),
        "weight_v": online_pi.get("weight_v", 1.0),
        "marginal_margin_m": online_pi.get("marginal_margin_m", 1.0),
        "mean": online_pi.get("mean", mean(pi_cost_values)),
        "max": online_pi.get("max", max(pi_cost_values) if pi_cost_values else None),
        "p05": online_pi.get("p05", quantile(pi_cost_values, 0.05)),
        "p50": online_pi.get("p50", quantile(pi_cost_values, 0.50)),
        "p95": online_pi.get("p95", quantile(pi_cost_values, 0.95)),
        "risk_band_histogram": online_pi.get("risk_band_histogram", pi_risk_counts),
        "dominant_axis_histogram": online_pi.get(
            "dominant_axis_histogram",
            pi_axis_counts,
        ),
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
        "fallback_count": predicted["fallback_count"],
        "fallback_rate": fallback_rate,
        "fallback_reason_histogram": fallback_reason_histogram,
        "finite_gnss_prediction_count": finite_gnss_prediction_count,
        "pl_grid": online_summary.get("pl_grid", {"enabled": False}),
        "lidar_observability": online_summary.get("lidar_observability", {"enabled": False}),
        "predicted_integrity": predicted,
        "pi_cost": pi_cost,
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


def svg_grid_update_timing(path, pl_grid):
    build = pl_grid.get("build_time_ms") or {}
    values = [
        ("last", as_float(build.get("last")), "#2b6cb0"),
        ("mean", as_float(build.get("mean")), "#2f855a"),
        ("max", as_float(build.get("max")), "#c05621"),
    ]
    finite_values = [v for _, v, _ in values if math.isfinite(v)]
    width, height = 640, 360
    if not finite_values:
        write_svg(
            path,
            "PL grid build timing",
            '<text x="48" y="170" font-family="sans-serif" font-size="13">no finite build timing</text>',
            width,
            height,
        )
        return
    max_value = max(finite_values)
    if max_value <= 0.0:
        max_value = 1.0
    body = [
        f'<text x="48" y="54" font-family="sans-serif" font-size="12">updates: {int(pl_grid.get("update_count") or 0)}  skips: {int(pl_grid.get("skip_count") or 0)}</text>'
    ]
    for idx, (label, value, color) in enumerate(values):
        y = 88 + idx * 42
        width_px = 450.0 * value / max_value if math.isfinite(value) else 0.0
        body.append(f'<text x="48" y="{y+14}" font-family="sans-serif" font-size="12">{html.escape(label)}</text>')
        body.append(f'<rect x="112" y="{y}" width="{width_px:.1f}" height="20" fill="{color}"/>')
        text = f"{value:.3f} ms" if math.isfinite(value) else "nan"
        body.append(f'<text x="{118+width_px:.1f}" y="{y+15}" font-family="sans-serif" font-size="12">{html.escape(text)}</text>')
    counts = pl_grid.get("query_counts") or {}
    query_text = "queries grid/direct/fallback: {}/{}/{}".format(
        int(counts.get("grid") or 0),
        int(counts.get("direct") or 0),
        int(counts.get("fallback") or 0),
    )
    body.append(f'<text x="48" y="248" font-family="sans-serif" font-size="12">{html.escape(query_text)}</text>')
    ratio = as_float((pl_grid.get("grid_vs_direct_self_check") or {}).get("last_pl_ratio"))
    ratio_text = f"grid-vs-direct self-check ratio: {ratio:.3f}" if math.isfinite(ratio) else "grid-vs-direct self-check ratio: nan"
    body.append(f'<text x="48" y="274" font-family="sans-serif" font-size="12">{html.escape(ratio_text)}</text>')
    write_svg(path, "PL grid build timing", "\n".join(body), width, height)


def svg_gnss_vs_fused_pl(path, pred_rows):
    xs = list(range(len(pred_rows)))
    svg_line_plot(
        path,
        "GNSS vs fused future PL",
        [
            ("GNSS HPL", xs, [r.get("gnss_hpl") for r in pred_rows], "#2b6cb0"),
            ("GNSS VPL", xs, [r.get("gnss_vpl") for r in pred_rows], "#805ad5"),
            ("Fused HPL", xs, [r.get("fused_hpl") for r in pred_rows], "#2f855a"),
            ("Fused VPL", xs, [r.get("fused_vpl") for r in pred_rows], "#c05621"),
        ],
    )


def generate_h_lite_plots(export_dir, pred_rows, snapshot_rows, online_summary):
    figs = export_dir / "figs"
    xs = list(range(len(pred_rows)))
    svg_line_plot(
        figs / "future_hpl_vpl_al_im_timeline.svg",
        "Future HPL/VPL/AL/IM timeline",
        [
            ("HPL", xs, [r.get("PL_H_pred") for r in pred_rows], "#2b6cb0"),
            ("VPL", xs, [r.get("PL_V_pred") for r in pred_rows], "#805ad5"),
            ("AL", xs, [r.get("AL_pred") for r in pred_rows], "#2f855a"),
            ("IM", xs, [r.get("IM_pred") for r in pred_rows], "#c05621"),
        ],
    )
    svg_scatter_plot(
        figs / "future_pl_vs_nvis_scatter.svg",
        "Future PL vs n_vis",
        pred_rows,
        "n_vis",
        "PL_pred",
    )
    svg_scatter_plot(
        figs / "future_pl_vs_pdop_scatter.svg",
        "Future PL vs PDOP",
        pred_rows,
        "pdop",
        "PL_pred",
    )
    svg_line_plot(
        figs / "future_im_trajectory_xy.svg",
        "Future trajectory XY",
        [("XY", [r.get("x") for r in pred_rows], [r.get("y") for r in pred_rows], "#2b6cb0")],
    )
    fallback_counts = {}
    for row in pred_rows:
        if as_bool(row.get("fallback")):
            reason = row.get("fallback_reason") or "unknown"
            fallback_counts[reason] = fallback_counts.get(reason, 0) + 1
    svg_histogram(
        figs / "fallback_reason_histogram.svg",
        "Fallback reason histogram",
        fallback_counts,
    )
    svg_line_plot(
        figs / "current_consistency_timeline.svg",
        "Current PL consistency ratio",
        [
            (
                "PL ratio",
                [r.get("stamp") for r in snapshot_rows],
                [r.get("consistency_pl_ratio") for r in snapshot_rows],
                "#c05621",
            )
        ],
    )
    pl_grid = online_summary.get("pl_grid") or {}
    grid_update_timing = "skipped_not_applicable"
    if as_bool(pl_grid.get("enabled", False)):
        svg_grid_update_timing(figs / "grid_update_timing.svg", pl_grid)
        grid_update_timing = "figs/grid_update_timing.svg"
    lidar = online_summary.get("lidar_observability") or {}
    lidar_observability = "skipped_not_applicable"
    fused_fim_grid = "skipped_not_applicable"
    if as_bool(lidar.get("enabled", False)):
        lidar_xs = list(range(len(pred_rows)))
        svg_line_plot(
            figs / "future_lidar_alpha_tdop_timeline.svg",
            "Future LiDAR alpha and TDOP",
            [
                ("alpha", lidar_xs, [r.get("lidar_alpha") for r in pred_rows], "#2f855a"),
                ("tdop", lidar_xs, [r.get("lidar_tdop") for r in pred_rows], "#c05621"),
            ],
        )
        svg_gnss_vs_fused_pl(figs / "future_gnss_vs_fused_pl.svg", pred_rows)
        lidar_observability = "figs/future_lidar_alpha_tdop_timeline.svg"
        if as_bool(lidar.get("fused_fim_grid", False)):
            fused_fim_grid = "figs/future_gnss_vs_fused_pl.svg"
    return {
        "future_hpl_vpl_al_im_timeline": "figs/future_hpl_vpl_al_im_timeline.svg",
        "future_pl_vs_nvis_scatter": "figs/future_pl_vs_nvis_scatter.svg",
        "future_pl_vs_pdop_scatter": "figs/future_pl_vs_pdop_scatter.svg",
        "future_im_trajectory_xy": "figs/future_im_trajectory_xy.svg",
        "fallback_reason_histogram": "figs/fallback_reason_histogram.svg",
        "current_consistency_timeline": "figs/current_consistency_timeline.svg",
        "grid_update_timing": grid_update_timing,
        "future_lidar_alpha_tdop_timeline": lidar_observability,
        "future_gnss_vs_fused_pl": fused_fim_grid,
    }


def analyze(run_dir, match_tolerance_s):
    export_dir = run_dir / "export"
    pred_path = export_dir / "integrity_along_planner_traj.csv"
    snapshot_path = export_dir / "future_integrity_snapshot.csv"
    online_summary_path = export_dir / "phase2_summary.json"
    aligned_path = export_dir / "phase2_integrity_eval_aligned.csv"

    pred_rows = load_csv(pred_path)
    snapshot_rows = load_csv(snapshot_path)
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

    integrity_snapshot, current_consistency_raw, current_consistency_anchored = summarize_snapshot(
        snapshot_rows, araim_index, match_tolerance_s, warnings
    )
    plots = generate_h_lite_plots(export_dir, pred_rows, snapshot_rows, online_summary)
    summary = summarize(pred_rows, aligned_rows, online_summary, run_dir, warnings, errors)
    summary["integrity_snapshot"] = integrity_snapshot
    summary["current_consistency_raw"] = current_consistency_raw
    summary["current_consistency_anchored"] = current_consistency_anchored
    summary["current_consistency"] = current_consistency_raw
    capabilities = dict(online_summary.get("stage1_capabilities") or {})
    capabilities.update(summary.get("stage1_capabilities") or {})
    summary["stage1_capabilities"] = capabilities
    capabilities["fused_araim_style"] = "deferred_after_rc"
    capabilities["self_consistency_rc_metric"] = "current_consistency_raw"
    summary["phase_h_lite"] = {
        "plots": plots,
        "grid_update_timing": plots.get("grid_update_timing", "skipped_not_applicable"),
        "lidar_observability": plots.get("future_lidar_alpha_tdop_timeline", "skipped_not_applicable"),
        "fused_fim_grid": plots.get("future_gnss_vs_fused_pl", "skipped_not_applicable"),
    }
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
