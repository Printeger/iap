#!/usr/bin/env python3
import argparse
import csv
import json
import math
import sys
from pathlib import Path


REQUIRED_FILES = [
    "desired_vs_truth.csv",
    "planner_traj.csv",
    "planner_cmd.csv",
    "iap_sim_truth_vs_est.csv",
    "phase1_summary.json",
]

TRUTH_ODOM_TOPIC = "/sim/drone_0/truth_odom"

FINITE_COLUMNS = [
    "desired_x",
    "desired_y",
    "desired_z",
    "desired_vx",
    "desired_vy",
    "desired_vz",
    "desired_ax",
    "desired_ay",
    "desired_az",
    "truth_x",
    "truth_y",
    "truth_z",
    "truth_vx",
    "truth_vy",
    "truth_vz",
    "iap_x",
    "iap_y",
    "iap_z",
]


def load_json(path):
    with path.open() as f:
        return json.load(f)


def load_csv_rows(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def finite_float(text):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def check_required_files(export_dir, failures):
    for name in REQUIRED_FILES:
        path = export_dir / name
        if not path.exists() or path.stat().st_size == 0:
            failures.append(f"missing or empty export/{name}")


def check_finite_desired_vs_truth(export_dir, failures):
    path = export_dir / "desired_vs_truth.csv"
    if not path.exists() or path.stat().st_size == 0:
        return
    rows = load_csv_rows(path)
    if not rows:
        failures.append("desired_vs_truth.csv has no data rows")
        return
    for row_idx, row in enumerate(rows, start=2):
        for col in FINITE_COLUMNS:
            value = finite_float(row.get(col))
            if value is None:
                failures.append(f"desired_vs_truth.csv row {row_idx}: non-finite {col}")
                return


def check_official_mode(summary, failures):
    checks = [
        ("allow_truth_alignment", False),
        ("use_so3_dynamics", True),
        ("use_iap_odom_for_planner", True),
    ]
    for key, expected in checks:
        if key not in summary:
            failures.append(f"official mode requires {key} in phase1_summary.json")
            continue
        value = as_bool(summary.get(key))
        if value is not expected:
            failures.append(f"official mode requires {key} == {expected}, got {value}")

    plant_mode = summary.get("plant_mode")
    if plant_mode != "so3_quadrotor_simulator":
        failures.append(
            "official mode requires plant_mode == so3_quadrotor_simulator, "
            f"got {plant_mode!r}"
        )

    for key in ("planner_odom_topic", "controller_odom_topic"):
        topic = summary.get(key)
        if not topic:
            failures.append(f"official mode requires {key} in phase1_summary.json")
        elif topic == TRUTH_ODOM_TOPIC:
            failures.append(f"official mode forbids {key}={TRUTH_ODOM_TOPIC}")


def main():
    parser = argparse.ArgumentParser(description="Validate a Phase 1 closed-loop run directory.")
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--use-gnss", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--use-araim", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--official", action="store_true")
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

    check_required_files(export_dir, failures)
    summary_path = export_dir / "phase1_summary.json"
    summary = load_json(summary_path) if summary_path.exists() and summary_path.stat().st_size else {}

    use_gnss = as_bool(summary.get("use_gnss", True)) if args.use_gnss is None else args.use_gnss
    use_araim = as_bool(summary.get("use_araim", True)) if args.use_araim is None else args.use_araim
    if args.official:
        check_official_mode(summary, failures)

    araim_path = export_dir / "iap_araim.csv"
    if use_gnss and use_araim:
        if not araim_path.exists() or araim_path.stat().st_size == 0:
            failures.append("GNSS+ARAIM enabled but export/iap_araim.csv is missing or empty")
    elif not araim_path.exists() or araim_path.stat().st_size == 0:
        warnings.append("ARAIM CSV not present; launch/config indicates GNSS or ARAIM disabled")

    run_duration = float(summary.get("run_duration_s") or 0.0)
    if run_duration < 30.0:
        failures.append(f"run_duration_s {run_duration:.2f} < 30.00")

    count_checks = [
        ("planner_trajectory_count", 1),
        ("planner_command_count", 100),
        ("truth_odom_count", 100),
        ("iap_odom_count", 50),
    ]
    for key, minimum in count_checks:
        value = int(summary.get(key) or 0)
        if value < minimum:
            failures.append(f"{key} {value} < {minimum}")

    check_finite_desired_vs_truth(export_dir, failures)

    movement = float(summary.get("simulator_movement_m") or 0.0)
    if movement < 0.1:
        failures.append(f"simulator movement {movement:.3f} m < 0.100 m")

    initial_goal_distance = summary.get("initial_distance_to_goal_m")
    final_goal_distance = summary.get("final_distance_to_goal_m")
    if initial_goal_distance is None or final_goal_distance is None:
        failures.append("initial/final distance to goal missing from phase1_summary.json")
    elif float(final_goal_distance) >= float(initial_goal_distance):
        failures.append(
            "final distance to goal did not improve "
            f"({float(final_goal_distance):.3f} >= {float(initial_goal_distance):.3f})"
        )

    tracking = summary.get("tracking") or {}
    tracking_rmse = tracking.get("rmse")
    if tracking_rmse is not None and float(tracking_rmse) > 3.0:
        warnings.append(f"tracking RMSE is large: {float(tracking_rmse):.3f} m")

    print(f"Validated run: {run_dir}")
    print(f"run_duration_s: {run_duration:.2f}")
    print(f"planner_trajectory_count: {int(summary.get('planner_trajectory_count') or 0)}")
    print(f"planner_command_count: {int(summary.get('planner_command_count') or 0)}")
    print(f"truth_odom_count: {int(summary.get('truth_odom_count') or 0)}")
    print(f"iap_odom_count: {int(summary.get('iap_odom_count') or 0)}")
    print(f"simulator_movement_m: {movement:.3f}")
    print(f"official: {args.official}")
    if args.official:
        print(f"allow_truth_alignment: {summary.get('allow_truth_alignment')}")
        print(f"use_so3_dynamics: {summary.get('use_so3_dynamics')}")
        print(f"use_iap_odom_for_planner: {summary.get('use_iap_odom_for_planner')}")
        print(f"plant_mode: {summary.get('plant_mode')}")
        print(f"planner_odom_topic: {summary.get('planner_odom_topic')}")
        print(f"controller_odom_topic: {summary.get('controller_odom_topic')}")

    if warnings:
        print("\nWarnings:")
        for warning in warnings:
            print(f"  - {warning}")

    if failures:
        print("\nFailures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("Phase 1 validation passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
