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
    "AL_V_pred",
]

OFFICIAL_ODOM_SOURCE = "/drone_0_visual_slam/odom"


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
    for row_idx, row in enumerate(rows, start=2):
        for col in REQUIRED_ONLINE_FINITE_COLUMNS:
            if finite_float(row.get(col)) is None:
                failures.append(f"integrity_along_planner_traj.csv row {row_idx}: non-finite {col}")
                return rows
    if not any(finite_float(row.get("AL_pred")) is not None for row in rows):
        failures.append("AL_pred contains only NaN/non-finite values")
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
    odom_source = summary.get("odom_source")
    if odom_source != OFFICIAL_ODOM_SOURCE:
        failures.append(
            f"official Phase 2 requires odom_source {OFFICIAL_ODOM_SOURCE}, got {odom_source!r}"
        )
    return summary


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
    check_phase1_files(export_dir, failures)
    check_offline_alignment(export_dir, failures, warnings)
    run_phase1_validator(run_dir, failures)
    warning_checks(summary, warnings)

    print(f"Validated Phase 2 run: {run_dir}")
    print(f"sample_count: {len(online_rows)}")
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
