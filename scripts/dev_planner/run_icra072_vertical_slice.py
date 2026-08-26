#!/usr/bin/env python3
"""Run the single registered ICRA-072 development vertical-slice smoke."""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
TASK_RESULTS_ROOT = (REPOSITORY / "results/icra27/icra072").resolve()
RUN_ID = "icra072-dev-smoke-001"
REQUIRED_PROCESSES = {
    "corridor_map": ["test_planner_corridor_map_publisher"],
    "pcl_render": ["drone_0_pcl_render_node"],
    "lidar_bridge": ["test_planner_lidar_body_bridge"],
    "iap": ["test_planner_iap_rosnode"],
    "odom_bridge": ["test_planner_desired_poscmd_to_odom"],
    "iap_odom_viz": ["test_planner_iap_odom_visualization"],
    "truth_odom_viz": ["test_planner_truth_odom_visualization"],
    "desired_odom_viz": ["test_planner_desired_odom_visualization"],
    "gnss_sim": ["test_planner_gnss_sim_node"],
    "simulator": ["drone_0_quadrotor_simulator_so3"],
    "controller": ["test_planner_so3_control_container"],
    "ego": ["drone_0_ego_planner_node"],
    "traj_server": ["drone_0_traj_server"],
    "provenance": ["test_planner_evidence_provenance"],
    "validator": ["test_planner_integrity_validator"],
}


def _load_gate_runner():
    path = REPOSITORY / "scripts/dev_planner/run_gate0_qualification.py"
    spec = importlib.util.spec_from_file_location("icra072_gate_runner", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load GPU/process monitor")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _task_local(path: Path, label: str) -> Path:
    resolved = path.resolve()
    try:
        resolved.relative_to(TASK_RESULTS_ROOT)
    except ValueError as exc:
        raise SystemExit(f"{label} must be under {TASK_RESULTS_ROOT}") from exc
    return resolved


def _stop_owned(process: subprocess.Popen | None, timeout_s: float = 10.0) -> int | None:
    if process is None:
        return None
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        return process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.wait(timeout=5.0)


def _owned_group_cleared(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return True
    except PermissionError:
        return False
    return False


def _wait_ready(process: subprocess.Popen, path: Path) -> dict:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return {"ready": False, "reason": "capture_exited_before_ready"}
        if path.exists():
            try:
                payload = json.loads(path.read_text())
            except (OSError, json.JSONDecodeError):
                payload = {}
            if (payload.get("schema_version")
                    == "icra072_capture_readiness_v1"
                    and payload.get("ready") is True):
                return payload
        time.sleep(0.05)
    return {"ready": False, "reason": "capture_readiness_timeout"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--install-root", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=45.0)
    args = parser.parse_args()
    run_root = _task_local(args.run_root, "run root")
    install_root = _task_local(args.install_root, "install root")
    if not install_root.is_dir():
        raise SystemExit("install root must be an existing task-local directory")
    if args.duration_s < 30.0:
        raise SystemExit("duration must be at least 30 seconds")
    run_root.mkdir(parents=True, exist_ok=False)
    exports = run_root / "exports"
    runtime = run_root / "runtime"
    bags = run_root / "bags"
    for path in (exports, runtime, bags):
        path.mkdir(parents=True)

    gate = _load_gate_runner()
    preflight = gate.run_gpu_preflight(run_root / "preflight")
    manifest = {
        "schema_version": "icra072_registered_dev_smoke_v1",
        "run_id": RUN_ID,
        "registered": True,
        "development_only": True,
        "effect_claim": False,
        "install_root": str(install_root),
        "duration_s": args.duration_s,
        "gpu_ready": bool(preflight.get("gpu_ready")),
        "launch_started": False,
        "required_process_set": list(REQUIRED_PROCESSES),
    }
    manifest_path = run_root / "run_manifest.json"
    if not preflight.get("gpu_ready"):
        manifest["result"] = "GPU_NOT_READY"
        _write(manifest_path, manifest)
        print("GPU_NOT_READY")
        return 4

    capture_command = [
        sys.executable,
        str(REPOSITORY / "scripts/dev_planner/capture_icra072_vertical_slice.py"),
        "--output", str(run_root / "lineage_capture.jsonl"),
        "--ready-file", str(run_root / "capture_ready.json"),
        "--duration-s", str(args.duration_s + 15.0),
    ]
    capture_log = (run_root / "capture_stdout.log").open("x")
    capture = subprocess.Popen(
        capture_command, stdout=capture_log, stderr=subprocess.STDOUT,
        start_new_session=True)
    atexit.register(_stop_owned, capture)
    readiness = _wait_ready(capture, run_root / "capture_ready.json")
    manifest["capture_readiness"] = readiness
    if readiness.get("ready") is not True:
        _stop_owned(capture)
        capture_log.close()
        manifest["result"] = "CAPTURE_NOT_READY"
        _write(manifest_path, manifest)
        return 5

    launch_args = [
        "experiment:=icra_p0_p4_v2_p5_dev",
        f"run_duration_s:={args.duration_s + 30.0}",
        f"validation_duration_s:={args.duration_s + 20.0}",
        f"runtime_root_dir:={runtime}",
        f"export_root_dir:={exports}",
        f"iap_log_root:={runtime / 'iap_logs'}",
        f"bag_output_dir:={bags}",
        f"p4.debug_csv_path:={exports / 'planner_p4_risk_astar_debug.csv'}",
        "record_bag:=false",
        "start_rviz:=false",
    ]
    setup = install_root / "setup.bash"
    shell_command = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/dev/ws_iap/install/setup.bash && "
        f"source {shlex.quote(str(setup))} && exec "
        + shlex.join([
            "ros2", "launch", "iap", "test_planner.launch.py", *launch_args])
    )
    _write(run_root / "launch_command.json", {
        "argv": ["bash", "-lc", shell_command],
        "profile": "icra_p0_p4_v2_p5_dev",
        "scenario": "icra_p0_p4_v2_p5_dev_fixture_v1",
    })
    launch_log = (run_root / "stdout.log").open("x")
    launch = subprocess.Popen(
        ["bash", "-lc", shell_command], stdout=launch_log,
        stderr=subprocess.STDOUT, start_new_session=True)
    atexit.register(_stop_owned, launch)
    monitor = gate.RequiredProcessMonitor(
        launch.pid, REQUIRED_PROCESSES, args.duration_s)
    try:
        monitor.start()
        monitor.launch_running = True
        manifest["launch_started"] = True
        start = time.monotonic()
        early_exit = False
        while time.monotonic() - start < args.duration_s:
            if launch.poll() is not None:
                early_exit = True
                break
            time.sleep(0.1)
    finally:
        monitor.mark_controlled_shutdown()
        exit_code = _stop_owned(launch, 15.0)
        process_result = monitor.finish()
        capture_exit = _stop_owned(capture, 5.0)
        launch_log.close()
        capture_log.close()
    groups_cleared = (
        _owned_group_cleared(launch.pid) and
        _owned_group_cleared(capture.pid))
    manifest.update({
        "launch_exit_code": exit_code,
        "launch_early_exit": early_exit,
        "capture_exit_code": capture_exit,
        "owned_process_groups_cleared": groups_cleared,
        "process_result": process_result,
        "result": "RECORDED",
    })
    _write(manifest_path, manifest)
    return 0 if (not early_exit and process_result["required_processes_ok"]
                 and groups_cleared) else 6


if __name__ == "__main__":
    raise SystemExit(main())
