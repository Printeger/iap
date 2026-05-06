#!/usr/bin/env python3
import argparse
import json
import subprocess
import sys
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
                with summary_path.open() as f:
                    summary = json.load(f)
        else:
            analyze_output = "latest run directory not found"
            validate_output = "latest run directory not found"

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
            "integrity_snapshot": summary.get("integrity_snapshot", {}),
            "current_consistency": summary.get("current_consistency", {}),
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

    aggregate = {
        "available": True,
        "duration_s": args.duration_s,
        "results": results,
        "passed": all(r["passed"] for r in results),
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
