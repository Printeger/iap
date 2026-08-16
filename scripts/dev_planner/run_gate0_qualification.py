#!/usr/bin/env python3
"""Run the fixed, no-bag ICRA Gate 0A/0B qualification protocol."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


SCENARIOS = {
    "primary": "p1_fork_fused_v1",
    "mirror": "p1_fork_fused_mirror_v1",
    "flat-null": "p1_fork_symmetric_null_v1",
}


def gate0a_matrix() -> list[tuple[str, str, int]]:
    return [
        (label, scenario, repeat)
        for label, scenario in SCENARIOS.items()
        for repeat in range(1, 4)
    ]


def _argument(name: str, value: object) -> str:
    if isinstance(value, bool):
        value = "true" if value else "false"
    return f"{name}:={value}"


def gate0a_effective_config(
    run_id: str, scenario: str, run_dir: Path
) -> dict[str, object]:
    return {
        "experiment": "p1_fork_formal",
        "scenario": scenario,
        "logical_seed": 11,
        "forest_random_seed": 11,
        "gnss_random_seed": 20260011,
        "terminal_wall_feature_seed": 11022,
        "planner_safety_profile": "off",
        "planner_enable_all_safety": False,
        "planner_enable_p1": False,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
        "planner_enable_p5_runtime": False,
        "planner_enable_p5_final": False,
        "p0.enable_risk_grid": False,
        "p1.use_integrity_cost": False,
        "p1.metrics_only": False,
        "p1.lambda_integrity": 0.0,
        "p1.debug_csv_enable": False,
        "manager/p1_collision_fanout_clearance_m": 0.0,
        "manager/p1_collision_fanout_preserve_homotopies": False,
        "manager/p1_collision_fanout_mirror_y": False,
        "p2.enable_candidate_ranking": False,
        "p2.debug_csv_enable": False,
        "p3.enable_local_reference_bias": False,
        "p3.enable_global_reference_bias": False,
        "p3.debug_csv_enable": False,
        "p4.enable_risk_aware_astar": False,
        "p4.debug_csv_enable": False,
        "p5.enable_runtime_gate": False,
        "p5.enable_final_gate": False,
        "manager/use_distinctive_trajs": True,
        "record_bag": False,
        "start_rviz": False,
        "run_validator": True,
        "run_duration_s": 90,
        "validation_duration_s": 85,
        "gate0.qualification_evidence_enable": True,
        "gate0.candidate_events_path": str(run_dir / "candidate_events.csv"),
        "gate0.control_points_path": str(
            run_dir / "candidate_control_points_raw.csv"
        ),
        "gate0.evidence_run_id": run_id,
        "gate0.evidence_manifest_path": str(
            run_dir / "gate0_run_manifest.json"
        ),
        "runtime_root_dir": str(run_dir / "runtime"),
        "export_root_dir": str(run_dir / "exports"),
    }


def p0_effective_config(root: Path) -> dict[str, object]:
    return {
        "experiment": "p0_open_sky",
        "scenario": "gnss_open_sky",
        "planner_safety_profile": "off",
        "planner_enable_all_safety": False,
        "planner_enable_p1": False,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
        "planner_enable_p5_runtime": False,
        "planner_enable_p5_final": False,
        "p0.enable_risk_grid": True,
        "p0.size_x_m": 30.0,
        "p0.size_y_m": 30.0,
        "p0.size_z_m": 6.0,
        "p0.resolution_m": 0.75,
        "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5",
        "p0.refresh_period_s": 0.5,
        "p0.predictor.worker_count": 1,
        "p0.skip_occupied_voxels": True,
        "p1.use_integrity_cost": False,
        "p1.metrics_only": False,
        "p2.enable_candidate_ranking": False,
        "p3.enable_local_reference_bias": False,
        "p3.enable_global_reference_bias": False,
        "p4.enable_risk_aware_astar": False,
        "p5.enable_runtime_gate": False,
        "p5.enable_final_gate": False,
        "record_bag": False,
        "start_rviz": False,
        "run_validator": False,
        "run_duration_s": 60,
        "validation_duration_s": 55,
        "runtime_root_dir": str(root / "runtime"),
        "export_root_dir": str(root / "exports"),
    }


def launch_command(config: dict[str, object]) -> list[str]:
    return [
        "ros2", "launch", "iap", "test_planner.launch.py",
        *[_argument(key, value) for key, value in config.items()],
    ]


def _write_run_files(
    run_dir: Path, run_id: str, scenario_label: str,
    config: dict[str, object], command: list[str]
) -> None:
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema_version": "gate0_run_manifest_v1",
        "run_id": run_id,
        "scenario": scenario_label,
        "fixed_protocol": True,
        "effective_config": config,
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    (run_dir / "gate0_run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    (run_dir / "command.txt").write_text(" ".join(command) + "\n")


def _run(command: list[str], stdout_path: Path) -> int:
    with stdout_path.open("w") as output:
        completed = subprocess.run(
            command,
            stdout=output,
            stderr=subprocess.STDOUT,
            check=False,
        )
    return completed.returncode


def run_gate0a(root: Path) -> list[int]:
    exit_codes = []
    for label, scenario, repeat in gate0a_matrix():
        run_id = f"{label}-r{repeat}"
        run_dir = root / "gate0a" / run_id
        config = gate0a_effective_config(run_id, scenario, run_dir)
        command = launch_command(config)
        _write_run_files(run_dir, run_id, label, config, command)
        exit_code = _run(command, run_dir / "stdout.log")
        exit_codes.append(exit_code)
        manifest_path = run_dir / "gate0_run_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["exit_code"] = exit_code
        manifest["planner_crash"] = exit_code != 0
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n"
        )
    return exit_codes


def run_gate0b(root: Path, capture_script: Path) -> int:
    run_dir = root / "p0"
    config = p0_effective_config(run_dir)
    command = launch_command(config)
    _write_run_files(run_dir, "p0-full-grid", "gnss_open_sky", config, command)
    capture_command = [
        sys.executable,
        str(capture_script),
        "--output", str(run_dir / "risk_grid_health.jsonl"),
        "--duration-s", "65",
    ]
    with (run_dir / "capture_stdout.log").open("w") as capture_output:
        capture = subprocess.Popen(
            capture_command,
            stdout=capture_output,
            stderr=subprocess.STDOUT,
        )
        exit_code = _run(command, run_dir / "stdout.log")
        try:
            capture.wait(timeout=75)
        except subprocess.TimeoutExpired:
            capture.terminate()
            capture.wait(timeout=10)
    manifest_path = run_dir / "gate0_run_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["exit_code"] = exit_code
    manifest["capture_exit_code"] = capture.returncode
    manifest["planner_crash"] = exit_code != 0
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return exit_code


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--gate0a-only", action="store_true")
    parser.add_argument("--gate0b-only", action="store_true")
    args = parser.parse_args()
    if args.gate0a_only and args.gate0b_only:
        parser.error("--gate0a-only and --gate0b-only are mutually exclusive")
    root = args.output_root.resolve()
    # A/B may be executed as separate fixed phases into one closure root.
    # Individual run directories remain fail-closed against overwrite.
    root.mkdir(parents=True, exist_ok=True)
    capture_script = Path(__file__).with_name("gate0_capture_p0_health.py")
    exit_codes = []
    if not args.gate0b_only:
        exit_codes.extend(run_gate0a(root))
    if not args.gate0a_only:
        exit_codes.append(run_gate0b(root, capture_script))
    return 0 if all(code == 0 for code in exit_codes) else 2


if __name__ == "__main__":
    raise SystemExit(main())
