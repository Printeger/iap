#!/usr/bin/env python3
import argparse
import csv
import json
import math
import statistics
import subprocess
import sys
from collections import Counter
from pathlib import Path


SCENARIOS = [
    {
        "name": "constant_current_open_sky",
        "pl_model": "constant_current",
        "scenario": "demo7_open_sky.yaml",
    },
    {
        "name": "gnss_open_sky",
        "pl_model": "gnss_geometry_araim",
        "scenario": "demo7_open_sky.yaml",
    },
    {
        "name": "gnss_skymask_nlos",
        "pl_model": "gnss_geometry_araim",
        "scenario": "demo7_skymask_nlos.yaml",
        "launch_args": {
            "gnss_enable_nlos": "false",
        },
    },
    {
        "name": "gnss_fault_injection",
        "pl_model": "gnss_geometry_araim",
        "scenario": "demo7_fault_injection.yaml",
    },
]


def run(cmd, cwd):
    proc = subprocess.run(
        cmd,
        cwd=cwd,
        shell=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        executable="/bin/bash",
    )
    return proc.returncode, proc.stdout


def latest_run(workspace):
    latest = workspace / "src" / "iap" / "log" / "latest"
    if latest.exists():
        return latest.resolve()
    return None


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


def finite_values(rows, column):
    return [as_float(r.get(column)) for r in rows if finite(r.get(column))]


def p95(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    pos = 0.95 * (len(values) - 1)
    lo = math.floor(pos)
    hi = math.ceil(pos)
    frac = pos - lo
    return values[lo] * (1.0 - frac) + values[hi] * frac


def median(values):
    values = [v for v in values if math.isfinite(v)]
    return statistics.median(values) if values else None


def nonempty_prn_list(value):
    if value is None:
        return False
    text = str(value).strip()
    return bool(text and text.lower() not in ("nan", "none", "null", "[]"))


def scenario_metrics(run_dir):
    if run_dir is None:
        return {}
    export_dir = run_dir / "export"
    future_rows = load_csv(export_dir / "integrity_along_planner_traj.csv")
    snapshot_rows = load_csv(export_dir / "future_integrity_snapshot.csv")
    summary = load_json(export_dir / "phase2_summary.json")

    fallback_reasons = Counter(
        (r.get("fallback_reason") or "").strip() or "<missing>"
        for r in future_rows
        if as_bool(r.get("fallback"))
    )
    snapshot_fallback_reasons = Counter(
        (r.get("pred_now_fallback_reason") or "").strip() or "<missing>"
        for r in snapshot_rows
        if as_bool(r.get("pred_now_fallback"))
    )

    prediction_columns = ("hpl_pred", "vpl_pred", "pl_pred_scalar")
    nonfinite_prediction_count = sum(
        1
        for r in future_rows
        if any(not finite(r.get(col)) for col in prediction_columns)
    )
    silent_success_count = sum(
        1
        for r in future_rows
        if not as_bool(r.get("fallback"))
        and any(not finite(r.get(col)) for col in prediction_columns)
    )
    unexplained_fallback_count = fallback_reasons.get("<missing>", 0)
    snapshot_unexplained_fallback_count = snapshot_fallback_reasons.get("<missing>", 0)

    n_detected_values = finite_values(snapshot_rows, "n_detected")
    excluded_count = sum(1 for r in snapshot_rows if nonempty_prn_list(r.get("excluded_prns")))
    n_vis = finite_values(future_rows, "n_vis")
    pdop = finite_values(future_rows, "pdop")
    hpl = finite_values(future_rows, "hpl_pred")
    vpl = finite_values(future_rows, "vpl_pred")
    pl = finite_values(future_rows, "pl_pred_scalar")

    return {
        "run_dir": str(run_dir),
        "sample_count": len(future_rows) or summary.get("sample_count"),
        "summary_sample_count": summary.get("sample_count"),
        "fallback_rate": summary.get("fallback_rate"),
        "fallback_count": sum(1 for r in future_rows if as_bool(r.get("fallback"))),
        "fallback_reason_histogram": dict(fallback_reasons),
        "snapshot_fallback_reason_histogram": dict(snapshot_fallback_reasons),
        "finite_gnss_prediction_count": summary.get("finite_gnss_prediction_count"),
        "future_nonfinite_prediction_count": nonfinite_prediction_count,
        "future_silent_success_count": silent_success_count,
        "future_unexplained_fallback_count": unexplained_fallback_count,
        "snapshot_unexplained_fallback_count": snapshot_unexplained_fallback_count,
        "median_n_vis": median(n_vis),
        "p95_pdop": p95(pdop),
        "p95_hpl_pred": p95(hpl),
        "p95_vpl_pred": p95(vpl),
        "p95_pl_pred_scalar": p95(pl),
        "snapshot_count": len(snapshot_rows),
        "snapshot_n_detected_max": max(n_detected_values) if n_detected_values else None,
        "snapshot_excluded_prns_count": excluded_count,
        "current_consistency_raw": summary.get("current_consistency_raw", {}),
        "current_consistency_anchored": summary.get("current_consistency_anchored", {}),
        "stage1_capabilities": summary.get("stage1_capabilities", {}),
    }


def ratio_at_least(new_value, base_value, factor):
    if new_value is None or base_value is None:
        return False
    if not math.isfinite(new_value) or not math.isfinite(base_value):
        return False
    return new_value >= base_value * factor


def compare_cross_scenarios(results):
    by_name = {r["name"]: r for r in results}
    checks = {
        "available": True,
        "passed": True,
        "failures": [],
        "skipped": [],
    }

    open_sky = by_name.get("gnss_open_sky")
    skymask = by_name.get("gnss_skymask_nlos")
    if open_sky and skymask:
        open_m = open_sky.get("metrics") or {}
        sky_m = skymask.get("metrics") or {}
        median_n_vis_drop = None
        if open_m.get("median_n_vis") is not None and sky_m.get("median_n_vis") is not None:
            median_n_vis_drop = open_m["median_n_vis"] - sky_m["median_n_vis"]
        pdop_increase_ok = ratio_at_least(sky_m.get("p95_pdop"), open_m.get("p95_pdop"), 1.10)
        hpl_increase_ok = ratio_at_least(sky_m.get("p95_hpl_pred"), open_m.get("p95_hpl_pred"), 1.10)
        n_vis_drop_ok = median_n_vis_drop is not None and median_n_vis_drop >= 1.0
        passed = n_vis_drop_ok or pdop_increase_ok or hpl_increase_ok
        checks["skymask_nlos_vs_open_sky"] = {
            "passed": passed,
            "median_n_vis_drop": median_n_vis_drop,
            "n_vis_drop_ok": n_vis_drop_ok,
            "p95_pdop_open_sky": open_m.get("p95_pdop"),
            "p95_pdop_skymask_nlos": sky_m.get("p95_pdop"),
            "pdop_increase_ok": pdop_increase_ok,
            "p95_hpl_pred_open_sky": open_m.get("p95_hpl_pred"),
            "p95_hpl_pred_skymask_nlos": sky_m.get("p95_hpl_pred"),
            "hpl_increase_ok": hpl_increase_ok,
        }
        if not passed:
            checks["failures"].append(
                "gnss_skymask_nlos did not reduce median n_vis by >=1 or raise p95 pdop/hpl by >=10%"
            )
    else:
        checks["skipped"].append("skymask_nlos_vs_open_sky requires gnss_open_sky and gnss_skymask_nlos")

    fault = by_name.get("gnss_fault_injection")
    if fault:
        metrics = fault.get("metrics") or {}
        reacted = (metrics.get("snapshot_n_detected_max") or 0) > 0 or (
            metrics.get("snapshot_excluded_prns_count") or 0
        ) > 0
        future_no_nan = (metrics.get("future_nonfinite_prediction_count") or 0) == 0
        future_no_silent_success = (metrics.get("future_silent_success_count") or 0) == 0
        fallback_explained = (
            (metrics.get("future_unexplained_fallback_count") or 0) == 0
            and (metrics.get("snapshot_unexplained_fallback_count") or 0) == 0
        )
        passed = reacted and future_no_nan and future_no_silent_success and fallback_explained
        checks["fault_injection"] = {
            "passed": passed,
            "current_araim_reacted": reacted,
            "snapshot_n_detected_max": metrics.get("snapshot_n_detected_max"),
            "snapshot_excluded_prns_count": metrics.get("snapshot_excluded_prns_count"),
            "future_no_nan": future_no_nan,
            "future_nonfinite_prediction_count": metrics.get("future_nonfinite_prediction_count"),
            "future_no_silent_success": future_no_silent_success,
            "future_silent_success_count": metrics.get("future_silent_success_count"),
            "fallback_explained": fallback_explained,
            "future_unexplained_fallback_count": metrics.get("future_unexplained_fallback_count"),
            "snapshot_unexplained_fallback_count": metrics.get("snapshot_unexplained_fallback_count"),
        }
        if not passed:
            checks["failures"].append(
                "gnss_fault_injection did not show current ARAIM reaction with finite, explained future predictions"
            )
    else:
        checks["skipped"].append("fault_injection requires gnss_fault_injection")

    checks["passed"] = not checks["failures"]
    return checks


def main():
    parser = argparse.ArgumentParser(description="Run Phase H-lite demo10 scenario matrix.")
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--only", choices=[s["name"] for s in SCENARIOS], action="append")
    parser.add_argument("--output", default="")
    args = parser.parse_args()

    script = Path(__file__).resolve()
    workspace = script.parents[4]
    selected = [s for s in SCENARIOS if not args.only or s["name"] in args.only]
    results = []

    for scenario in selected:
        scenario_file = workspace / "src" / "iap" / "config" / "gnss_sim" / scenario["scenario"]
        launch_cmd = " ".join([
            "source install/setup.bash &&",
            "ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py",
            "start_rviz:=false",
            f"run_duration_s:={args.duration_s}",
            "allow_truth_alignment:=false",
            "use_so3_dynamics:=true",
            "use_gnss:=true",
            "use_araim:=true",
            f"phase2_pl_model:={scenario['pl_model']}",
            "phase2_al_model:=cloud_clearance",
            f"gnss_scenario_file:={scenario_file}",
        ])
        for key, value in (scenario.get("launch_args") or {}).items():
            launch_cmd += f" {key}:={value}"
        launch_code, launch_output = run(launch_cmd, workspace)
        run_dir = latest_run(workspace)
        analyze_code = validate_code = 2
        summary = {}
        if run_dir is not None:
            analyze_code, analyze_output = run(
                f"python3 src/iap/tools/phase2/analyze_phase2_integrity_eval.py --run-dir {run_dir}",
                workspace,
            )
            validate_code, validate_output = run(
                f"python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py --run-dir {run_dir}",
                workspace,
            )
            summary_path = run_dir / "export" / "phase2_summary.json"
            if summary_path.exists():
                summary = load_json(summary_path)
        else:
            analyze_output = "latest run directory not found"
            validate_output = "latest run directory not found"

        metrics = scenario_metrics(run_dir)
        results.append({
            "name": scenario["name"],
            "pl_model": scenario["pl_model"],
            "scenario_file": str(scenario_file),
            "run_dir": str(run_dir) if run_dir else "",
            "passed": launch_code == 0 and analyze_code == 0 and validate_code == 0,
            "launch_returncode": launch_code,
            "analyze_returncode": analyze_code,
            "validate_returncode": validate_code,
            "sample_count": summary.get("sample_count"),
            "fallback_rate": summary.get("fallback_rate"),
            "finite_gnss_prediction_count": summary.get("finite_gnss_prediction_count"),
            "integrity_snapshot": summary.get("integrity_snapshot", {}),
            "current_consistency": summary.get("current_consistency", {}),
            "current_consistency_raw": summary.get(
                "current_consistency_raw", summary.get("current_consistency", {})
            ),
            "current_consistency_anchored": summary.get("current_consistency_anchored", {}),
            "metrics": metrics,
            "warnings": summary.get("warnings", []),
            "skipped_not_applicable": [
                "grid_update_timing",
                "lidar_observability",
                "fused_fim_grid",
            ],
            "tail": {
                "launch": "\n".join(launch_output.splitlines()[-20:]),
                "analyze": "\n".join(analyze_output.splitlines()[-20:]),
                "validate": "\n".join(validate_output.splitlines()[-20:]),
            },
        })

    cross_scenario = compare_cross_scenarios(results)
    aggregate = {
        "available": True,
        "duration_s": args.duration_s,
        "results": results,
        "cross_scenario": cross_scenario,
        "passed": all(r["passed"] for r in results) and cross_scenario["passed"],
        "skipped_not_applicable": [
            "Phase D PLGrid",
            "Phase F LiDAR observability",
            "Phase G fused FIM / planner cost",
        ],
    }
    output = Path(args.output) if args.output else (
        workspace / "src" / "iap" / "log" / "phase2_h_lite_scenarios.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(aggregate, indent=2) + "\n")
    print(f"Wrote {output}")
    for result in results:
        status = "PASS" if result["passed"] else "FAIL"
        print(f"{status} {result['name']} run_dir={result['run_dir']}")
    return 0 if aggregate["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
