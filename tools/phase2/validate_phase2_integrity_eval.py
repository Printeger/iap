#!/usr/bin/env python3
import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path


REQUIRED_PHASE1_FILES = [
    "desired_vs_truth.csv",
    "planner_traj.csv",
    "planner_cmd.csv",
    "iap_sim_truth_vs_est.csv",
    "phase1_summary.json",
]

REQUIRED_ONLINE_FINITE_COLUMNS = [
    "stamp",
    "traj_id",
    "sample_index",
    "sample_abs_time",
    "x",
    "y",
    "z",
    "AL_H_pred",
    "AL_V_pred",
    "AL_pred",
]

NEW_PREDICTION_COLUMNS = [
    "hpl_pred",
    "vpl_pred",
    "pl_pred_scalar",
    "pl_ff_h",
    "pl_ff_v",
    "sigma_h",
    "sigma_v",
    "n_vis",
    "pdop",
    "n_hypotheses",
    "valid",
    "fallback",
    "fallback_reason",
    "query_source",
    "grid_enabled",
    "grid_generation",
    "grid_age_s",
    "grid_build_time_ms",
    "gnss_hpl",
    "gnss_vpl",
    "fused_hpl",
    "fused_vpl",
    "lidar_valid",
    "lidar_alpha",
    "lidar_tdop",
    "lidar_condition",
    "lidar_n_primitives",
    "lidar_bias_h",
    "lidar_bias_v",
    "lidar_fallback_reason",
    "pi_cost_h",
    "pi_cost_v",
    "pi_cost_total",
    "pi_risk_band",
    "pi_margin_h",
    "pi_margin_v",
    "pi_dominant_axis",
    "pi_risk_band_code",
    "pi_grad_x",
    "pi_grad_y",
    "pi_grad_z",
]

OFFICIAL_PREDICTION_FINITE_COLUMNS = [
    "PL_H_pred",
    "PL_V_pred",
    "PL_pred",
    "IM_H_pred",
    "IM_V_pred",
    "IM_pred_axis_min",
    "IM_pred_scalar",
    "IM_pred",
]

OFFICIAL_ODOM_SOURCE = "/drone_0_visual_slam/odom"

SNAPSHOT_REQUIRED_COLUMNS = [
    "stamp",
    "snapshot_valid",
    "has_pose",
    "p_x",
    "p_y",
    "p_z",
    "current_HPL",
    "current_VPL",
    "current_PL",
    "current_HAL",
    "current_VAL",
    "current_IM",
    "n_sv_used",
    "pdop",
    "n_hypotheses",
    "n_detected",
    "excluded_prns",
    "n_trunks_observed",
    "current_tdop",
    "lidar_modulation_alpha",
    "has_epoch",
    "epoch_sat_count",
    "has_lambda_base",
    "has_lidar_snapshot",
    "has_lidar_araim_result",
    "pred_now_hpl",
    "pred_now_vpl",
    "pred_now_pl",
    "pred_now_n_vis",
    "pred_now_pdop",
    "pred_now_valid",
    "pred_now_fallback",
    "pred_now_fallback_reason",
    "consistency_pl_ratio",
    "consistency_hpl_error",
    "consistency_vpl_error",
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


def finite_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def check_online_csv(export_dir, failures, warnings):
    path = export_dir / "integrity_along_planner_traj.csv"
    if not path.exists() or path.stat().st_size == 0:
        failures.append("missing or empty export/integrity_along_planner_traj.csv")
        return []
    rows = load_csv(path)
    if not rows:
        failures.append("integrity_along_planner_traj.csv has no data rows")
        return rows
    missing = [col for col in NEW_PREDICTION_COLUMNS if col not in rows[0]]
    if missing:
        failures.append(
            "integrity_along_planner_traj.csv missing new prediction column(s): "
            + ", ".join(missing)
        )
        return rows
    for row_idx, row in enumerate(rows, start=2):
        for col in REQUIRED_ONLINE_FINITE_COLUMNS + OFFICIAL_PREDICTION_FINITE_COLUMNS:
            if finite_float(row.get(col)) is None:
                failures.append(f"integrity_along_planner_traj.csv row {row_idx}: non-finite {col}")
                return rows
        al_h = finite_float(row.get("AL_H_pred"))
        al_v = finite_float(row.get("AL_V_pred"))
        hpl = finite_float(row.get("PL_H_pred"))
        vpl = finite_float(row.get("PL_V_pred"))
        im_axis_min = finite_float(row.get("IM_pred_axis_min"))
        if None not in (al_h, al_v, hpl, vpl, im_axis_min):
            expected = min(al_h - hpl, al_v - vpl)
            if abs(im_axis_min - expected) > 1.0e-6:
                failures.append(
                    "integrity_along_planner_traj.csv row "
                    f"{row_idx}: IM_pred_axis_min={im_axis_min:.9f} "
                    f"does not match min(AL_H-HPL, AL_V-VPL)={expected:.9f}"
                )
                return rows
        query_source = row.get("query_source")
        if query_source not in ("current", "direct", "fallback", "grid"):
            failures.append(
                f"integrity_along_planner_traj.csv row {row_idx}: invalid query_source {query_source!r}"
            )
            return rows
        pi_cost_h = finite_float(row.get("pi_cost_h"))
        pi_cost_v = finite_float(row.get("pi_cost_v"))
        pi_cost_total = finite_float(row.get("pi_cost_total"))
        pi_margin_h = finite_float(row.get("pi_margin_h"))
        pi_margin_v = finite_float(row.get("pi_margin_v"))
        pi_grad_x = finite_float(row.get("pi_grad_x"))
        pi_grad_y = finite_float(row.get("pi_grad_y"))
        pi_grad_z = finite_float(row.get("pi_grad_z"))
        pi_risk_band_code = finite_float(row.get("pi_risk_band_code"))
        if None in (
            pi_cost_h,
            pi_cost_v,
            pi_cost_total,
            pi_margin_h,
            pi_margin_v,
            pi_grad_x,
            pi_grad_y,
            pi_grad_z,
            pi_risk_band_code,
        ):
            failures.append(f"integrity_along_planner_traj.csv row {row_idx}: non-finite PI cost")
            return rows
        if abs(pi_margin_h - (al_h - hpl)) > 1.0e-6 or abs(pi_margin_v - (al_v - vpl)) > 1.0e-6:
            failures.append(
                f"integrity_along_planner_traj.csv row {row_idx}: PI margins do not match AL-PL"
            )
            return rows
        expected_pi = pi_cost_h + pi_cost_v
        if abs(pi_cost_total - expected_pi) > 1.0e-6:
            failures.append(
                "integrity_along_planner_traj.csv row "
                f"{row_idx}: pi_cost_total={pi_cost_total:.9f} "
                f"does not match pi_cost_h+pi_cost_v={expected_pi:.9f}"
            )
            return rows
        if row.get("pi_risk_band") not in ("SAFE_PI", "MARGINAL_PI", "UNSAFE_PI", "UNKNOWN_PI"):
            failures.append(
                f"integrity_along_planner_traj.csv row {row_idx}: invalid pi_risk_band {row.get('pi_risk_band')!r}"
            )
            return rows
        expected_code = {
            "UNKNOWN_PI": 0,
            "SAFE_PI": 1,
            "MARGINAL_PI": 2,
            "UNSAFE_PI": 3,
        }[row.get("pi_risk_band")]
        if int(round(pi_risk_band_code)) != expected_code:
            failures.append(
                "integrity_along_planner_traj.csv row "
                f"{row_idx}: pi_risk_band_code={pi_risk_band_code} does not match {row.get('pi_risk_band')}"
            )
            return rows
        if row.get("pi_dominant_axis") not in ("horizontal", "vertical", "balanced", "unknown"):
            failures.append(
                "integrity_along_planner_traj.csv row "
                f"{row_idx}: invalid pi_dominant_axis {row.get('pi_dominant_axis')!r}"
            )
            return rows
    if not any(finite_float(row.get("AL_pred")) is not None for row in rows):
        failures.append("AL_pred contains only NaN/non-finite values")
    pl_models = {row.get("pl_model", "") for row in rows}
    if any("gnss_geometry_araim" in model or model == "fused_fim_grid" for model in pl_models):
        if not any(finite_float(row.get("n_vis")) is not None for row in rows):
            failures.append("GNSS PL mode produced no finite n_vis values")
        if not any(finite_float(row.get("pdop")) is not None for row in rows):
            failures.append("GNSS PL mode produced no finite pdop values")
        fallback_count = sum(1 for row in rows if as_bool(row.get("fallback")))
        fallback_rate = fallback_count / len(rows)
        if fallback_rate > 0.05:
            warnings.append(f"GNSS prediction fallback rate is {fallback_rate:.3f}")
    def traj_id(row, default):
        value = finite_float(row.get("traj_id"))
        return value if value is not None else default

    bspline_rows = [row for row in rows if traj_id(row, -1.0) >= 0]
    fallback_rows = [row for row in rows if traj_id(row, 0.0) < 0]
    if not bspline_rows:
        failures.append("official Phase 2 requires at least one B-spline trajectory sample")
    if fallback_rows:
        warnings.append(f"{len(fallback_rows)} pos_cmd fallback sample(s) present")
    return rows


def check_snapshot_csv(export_dir, pl_models, failures, warnings):
    path = export_dir / "future_integrity_snapshot.csv"
    if not path.exists() or path.stat().st_size == 0:
        failures.append("missing or empty export/future_integrity_snapshot.csv")
        return []
    rows = load_csv(path)
    if not rows:
        failures.append("future_integrity_snapshot.csv has no data rows")
        return rows
    missing = [col for col in SNAPSHOT_REQUIRED_COLUMNS if col not in rows[0]]
    if missing:
        failures.append(
            "future_integrity_snapshot.csv missing column(s): " + ", ".join(missing)
        )
        return rows

    finite_consistency = 0
    max_consistency = 0.0
    for row_idx, row in enumerate(rows, start=2):
        for col in (
            "stamp",
            "p_x",
            "p_y",
            "p_z",
            "current_HPL",
            "current_VPL",
            "current_PL",
            "current_HAL",
            "current_VAL",
            "current_IM",
        ):
            if finite_float(row.get(col)) is None:
                failures.append(f"future_integrity_snapshot.csv row {row_idx}: non-finite {col}")
                return rows
        ratio = finite_float(row.get("consistency_pl_ratio"))
        if ratio is not None:
            finite_consistency += 1
            max_consistency = max(max_consistency, ratio)

    gnss_mode = any("gnss_geometry_araim" in model or model == "fused_fim_grid" for model in pl_models)
    if gnss_mode:
        if not any(finite_float(row.get("pred_now_n_vis")) is not None for row in rows):
            failures.append("GNSS snapshot prediction produced no finite pred_now_n_vis values")
        if not any(finite_float(row.get("pred_now_pdop")) is not None for row in rows):
            failures.append("GNSS snapshot prediction produced no finite pred_now_pdop values")
        for row_idx, row in enumerate(rows, start=2):
            if not as_bool(row.get("has_epoch")) and as_bool(row.get("pred_now_fallback")):
                if row.get("pred_now_fallback_reason") != "no_gnss_epoch":
                    failures.append(
                        "future_integrity_snapshot.csv row "
                        f"{row_idx}: missing GNSS epoch fallback reason is "
                        f"{row.get('pred_now_fallback_reason')!r}"
                    )
                    return rows

    if finite_consistency == 0:
        failures.append("future_integrity_snapshot.csv has no finite current consistency samples")
    elif max_consistency > 0.10:
        warnings.append(f"current PL consistency max ratio is {max_consistency:.3f}")
    return rows


def check_summary(export_dir, failures):
    path = export_dir / "phase2_summary.json"
    if not path.exists() or path.stat().st_size == 0:
        failures.append("missing or empty export/phase2_summary.json")
        return {}
    summary = load_json(path)
    if int(summary.get("sample_count") or 0) <= 0:
        failures.append("phase2_summary.json sample_count <= 0")
    if as_bool(summary.get("online_truth_used", False)):
        failures.append("phase2_summary.json online_truth_used is true")
    predicted = summary.get("predicted_integrity") or {}
    for field in (
        "fallback_count",
        "fallback_rate",
        "fallback_reason_histogram",
        "finite_gnss_prediction_count",
    ):
        if field not in summary and field not in predicted:
            failures.append(f"phase2_summary.json missing {field}")
    if "integrity_snapshot" not in summary:
        failures.append("phase2_summary.json missing integrity_snapshot")
    if "current_consistency" not in summary:
        failures.append("phase2_summary.json missing current_consistency")
    if "phase_h_lite" not in summary:
        failures.append("phase2_summary.json missing phase_h_lite")
    if "pl_grid" not in summary:
        failures.append("phase2_summary.json missing pl_grid")
    if "lidar_observability" not in summary:
        failures.append("phase2_summary.json missing lidar_observability")
    if "pi_cost" not in summary:
        failures.append("phase2_summary.json missing pi_cost")
    odom_source = summary.get("odom_source")
    if odom_source != OFFICIAL_ODOM_SOURCE:
        failures.append(
            f"official Phase 2 requires odom_source {OFFICIAL_ODOM_SOURCE}, got {odom_source!r}"
        )
    return summary


def check_pl_grid(summary, online_rows, failures, warnings):
    pl_grid = summary.get("pl_grid") or {}
    enabled = as_bool(pl_grid.get("enabled", False))
    if not enabled:
        return
    if "pl_grid" not in summary:
        failures.append("phase2_summary.json missing pl_grid")
        return
    if not as_bool(pl_grid.get("active", False)):
        failures.append("phase2_summary.json pl_grid.enabled=true but pl_grid.active=false")
    if int(pl_grid.get("update_count") or 0) <= 0:
        failures.append("phase2_summary.json pl_grid.update_count <= 0")

    query_counts = pl_grid.get("query_counts") or {}
    grid_query_count = int(query_counts.get("grid") or 0)
    csv_grid_count = sum(1 for row in online_rows if row.get("query_source") == "grid")
    if grid_query_count <= 0 and csv_grid_count <= 0:
        failures.append("phase2_use_pl_grid=true but no grid query was recorded")

    build = pl_grid.get("build_time_ms") or {}
    mean_build = finite_float(build.get("mean"))
    if mean_build is None or finite_float(build.get("max")) is None:
        failures.append("phase2_summary.json pl_grid build_time_ms mean/max are not finite")
    elif mean_build > 500.0:
        warnings.append(f"PL grid rebuild mean time is {mean_build:.1f} ms (>500 ms initial target)")

    ratio = finite_float((pl_grid.get("grid_vs_direct_self_check") or {}).get("last_pl_ratio"))
    if ratio is not None and ratio > 0.10:
        warnings.append(f"PL grid self-check ratio is {ratio:.3f}")


def check_current_consistency(summary, failures):
    consistency = summary.get("current_consistency") or {}
    max_ratio = finite_float(consistency.get("max_pl_ratio"))
    threshold = finite_float(consistency.get("warning_threshold_ratio")) or 0.10
    if max_ratio is None:
        failures.append("phase2_summary.json current_consistency has no finite max_pl_ratio")
    elif max_ratio > threshold:
        failures.append(
            f"phase2_summary.json current_consistency max_pl_ratio={max_ratio:.3f} exceeds {threshold:.3f}"
        )


def check_lidar_observability(summary, online_rows, failures, warnings):
    lidar = summary.get("lidar_observability") or {}
    enabled = as_bool(lidar.get("enabled", False))
    use_lidar = as_bool(lidar.get("use_lidar_observability", enabled))
    fused_mode = summary.get("pl_model") == "fused_fim_grid"
    if not enabled and not fused_mode:
        return

    if not online_rows:
        failures.append("LiDAR observability validation has no online rows")
        return

    finite_lidar_debug = 0
    conservative_checks = 0
    for row_idx, row in enumerate(online_rows, start=2):
        pl_h = finite_float(row.get("PL_H_pred"))
        pl_v = finite_float(row.get("PL_V_pred"))
        gnss_h = finite_float(row.get("gnss_hpl"))
        gnss_v = finite_float(row.get("gnss_vpl"))
        if None not in (pl_h, gnss_h):
            conservative_checks += 1
            if pl_h + 1.0e-6 < gnss_h:
                failures.append(
                    f"integrity_along_planner_traj.csv row {row_idx}: "
                    f"PL_H_pred={pl_h:.9f} is below gnss_hpl={gnss_h:.9f}"
                )
                return
        if None not in (pl_v, gnss_v) and pl_v + 1.0e-6 < gnss_v:
            failures.append(
                f"integrity_along_planner_traj.csv row {row_idx}: "
                f"PL_V_pred={pl_v:.9f} is below gnss_vpl={gnss_v:.9f}"
            )
            return
        if (
            finite_float(row.get("lidar_alpha")) is not None
            and finite_float(row.get("lidar_tdop")) is not None
            and finite_float(row.get("lidar_condition")) is not None
        ):
            finite_lidar_debug += 1

    if conservative_checks == 0:
        failures.append("fused_fim_grid produced no finite GNSS conservative checks")
    if use_lidar and int(lidar.get("valid_count") or 0) <= 0:
        failures.append("LiDAR observability enabled but summary valid_count <= 0")
    if use_lidar and finite_lidar_debug <= 0:
        failures.append("LiDAR observability enabled but no finite lidar debug samples exist")
    if int(lidar.get("conservative_fusion_violation_count") or 0) > 0:
        failures.append("phase2_summary.json reports conservative fusion violations")
    if use_lidar and float(lidar.get("valid_rate") or 0.0) < 0.05:
        warnings.append(f"LiDAR observability valid rate is low: {float(lidar.get('valid_rate') or 0.0):.3f}")


def check_phase1_files(export_dir, failures):
    for name in REQUIRED_PHASE1_FILES:
        path = export_dir / name
        if not path.exists() or path.stat().st_size == 0:
            failures.append(f"missing or empty Phase 1 required log export/{name}")


def check_offline_alignment(export_dir, failures, warnings):
    araim_path = export_dir / "iap_araim.csv"
    aligned_path = export_dir / "phase2_integrity_eval_aligned.csv"
    if not araim_path.exists() or araim_path.stat().st_size == 0:
        warnings.append("iap_araim.csv is unavailable; finite IM after offline analysis was not required")
        return
    if not aligned_path.exists() or aligned_path.stat().st_size == 0:
        failures.append("iap_araim.csv exists but export/phase2_integrity_eval_aligned.csv is missing")
        return
    rows = load_csv(aligned_path)
    if not any(finite_float(row.get("pred_IM")) is not None for row in rows):
        failures.append("no finite pred_IM exists after offline analysis despite available iap_araim.csv")
    if not any(finite_float(row.get("actual_PL")) is not None for row in rows):
        failures.append("no finite actual_PL exists after offline analysis despite available iap_araim.csv")


def run_phase1_validator(run_dir, failures):
    script = Path(__file__).resolve().parents[1] / "phase1" / "validate_phase1_closed_loop.py"
    if not script.exists():
        failures.append(f"Phase 1 validator not found: {script}")
        return
    cmd = [sys.executable, str(script), "--run-dir", str(run_dir), "--official"]
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if proc.returncode != 0:
        failures.append("demo10 did not pass Phase 1 official validation")
        output = "\n".join(f"    {line}" for line in proc.stdout.strip().splitlines()[-20:])
        if output:
            failures.append(f"Phase 1 validator output:\n{output}")


def warning_checks(summary, warnings):
    predicted = summary.get("predicted_integrity") or {}
    sample_count = int(summary.get("sample_count") or 0)
    unsafe = int(predicted.get("unsafe_count") or 0)
    if sample_count and unsafe / sample_count > 0.8:
        warnings.append("predicted IM is mostly unsafe")
    actual = summary.get("actual_alignment") or {}
    tracking = actual.get("mean_spatial_tracking_error")
    if finite_float(tracking) is not None and float(tracking) > 3.0:
        warnings.append(f"mean spatial tracking error is high: {float(tracking):.3f} m")
    if actual.get("mean_pred_actual_IM_error") is None:
        warnings.append("predicted and actual IM could not be compared")


def main():
    parser = argparse.ArgumentParser(description="Validate a Phase 2 PI-lite integrity evaluation run.")
    parser.add_argument("--run-dir", required=True)
    args = parser.parse_args()

    run_dir = Path(args.run_dir).expanduser().resolve()
    export_dir = run_dir / "export"
    failures = []
    warnings = []

    if not run_dir.exists():
        print(f"Run directory does not exist: {run_dir}", file=sys.stderr)
        return 2
    if not export_dir.exists():
        print(f"Run export directory does not exist: {export_dir}", file=sys.stderr)
        return 2

    online_rows = check_online_csv(export_dir, failures, warnings)
    summary = check_summary(export_dir, failures)
    check_current_consistency(summary, failures)
    check_pl_grid(summary, online_rows, failures, warnings)
    check_lidar_observability(summary, online_rows, failures, warnings)
    pl_models = {row.get("pl_model", "") for row in online_rows}
    if not pl_models and summary.get("pl_model"):
        pl_models = {summary.get("pl_model", "")}
    snapshot_rows = check_snapshot_csv(export_dir, pl_models, failures, warnings)
    check_phase1_files(export_dir, failures)
    check_offline_alignment(export_dir, failures, warnings)
    run_phase1_validator(run_dir, failures)
    warning_checks(summary, warnings)

    print(f"Validated Phase 2 run: {run_dir}")
    print(f"sample_count: {len(online_rows)}")
    print(f"snapshot_count: {len(snapshot_rows)}")
    print(f"traj_count: {summary.get('traj_count', 0)}")
    print(f"aligned_sample_count: {summary.get('aligned_sample_count', 0)}")
    print(f"odom_source: {summary.get('odom_source')}")
    print(f"map_source: {summary.get('map_source')}")

    if warnings:
        print("\nWarnings:")
        for warning in warnings:
            print(f"  - {warning}")

    if failures:
        print("\nFailures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("Phase 2 validation passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
