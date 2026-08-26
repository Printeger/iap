#!/usr/bin/env python3
"""Run one fresh ICRA-072A Layer-1 iterative development attempt."""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
DEV_RUNS_ROOT = (REPOSITORY / "results/icra27/dev_runs/layer1").resolve()
SHARED_INSTALL_ROOT = Path("/home/dev/ws_iap/install").resolve()
RUN_ID_PATTERN = re.compile(r"run-[0-9]{3,}")
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


def validate_cli_paths(run_root: Path, install_root: Path) -> tuple[Path, Path]:
    resolved_run = (run_root if run_root.is_absolute()
                    else REPOSITORY / run_root).resolve()
    resolved_install = install_root.resolve()
    if (resolved_run.parent != DEV_RUNS_ROOT or
            RUN_ID_PATTERN.fullmatch(resolved_run.name) is None):
        raise SystemExit(
            f"run root must match {DEV_RUNS_ROOT}/run-[0-9]{{3,}}")
    if resolved_run.exists():
        raise SystemExit("run root already exists")
    if resolved_install != SHARED_INSTALL_ROOT:
        raise SystemExit(f"install root must be {SHARED_INSTALL_ROOT}")
    if not resolved_install.is_dir() or not (resolved_install / "setup.bash").is_file():
        raise SystemExit("shared install root is not ready")
    return resolved_run, resolved_install


def _git_commit() -> str:
    completed = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=REPOSITORY,
        capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        raise SystemExit("cannot resolve repository commit")
    return completed.stdout.strip()


def _start_process(*args, **kwargs) -> subprocess.Popen:
    """Start one runner-owned child; kept as the test isolation seam."""
    return subprocess.Popen(*args, **kwargs)


def _stage_observations(run_root: Path) -> dict[str, int]:
    observations: dict[str, int] = {}
    capture = run_root / "lineage_capture.jsonl"
    if capture.is_file():
        for line in capture.read_text().splitlines():
            if not line.strip():
                continue
            try:
                kind = json.loads(line).get("kind")
            except json.JSONDecodeError:
                kind = "malformed_capture"
            if kind:
                observations[kind] = observations.get(kind, 0) + 1
    lineage_paths = list((run_root / "exports").glob(
        "**/planner_p4_risk_astar_debug.csv.lineage.csv"))
    if len(lineage_paths) == 1:
        import csv
        with lineage_paths[0].open(newline="") as stream:
            for row in csv.DictReader(stream):
                stage = row.get("stage")
                if stage:
                    key = f"lineage:{stage}"
                    observations[key] = observations.get(key, 0) + 1
    return dict(sorted(observations.items()))


def _stop_owned(process: subprocess.Popen | None, timeout_s: float = 10.0) -> int | None:
    if process is None:
        return None
    if getattr(process, "_icra072_group_cleared", False):
        return process.returncode
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        exit_code = process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        exit_code = process.wait(timeout=5.0)
    cleared = _wait_owned_group_cleared(process.pid, 0.25)
    if not cleared:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        cleared = _wait_owned_group_cleared(process.pid, 5.0)
    if cleared:
        process._icra072_group_cleared = True
    return exit_code


def _unregister_cleanup_if_cleared(*processes: subprocess.Popen) -> bool:
    if not all(getattr(process, "_icra072_group_cleared", False)
               for process in processes):
        return False
    atexit.unregister(_stop_owned)
    return True


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


def _wait_owned_group_cleared(pid: int | None, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _owned_group_cleared(pid):
            return True
        time.sleep(0.01)
    return _owned_group_cleared(pid)


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
    run_root, install_root = validate_cli_paths(
        args.run_root, args.install_root)
    if args.duration_s < 30.0:
        raise SystemExit("duration must be at least 30 seconds")
    run_root.mkdir(parents=True, exist_ok=False)
    exports = run_root / "exports"
    runtime = run_root / "runtime"
    bags = run_root / "bags"
    ros_logs = runtime / "ros_logs"
    for path in (exports, runtime, bags, ros_logs):
        path.mkdir(parents=True)

    gate = _load_gate_runner()
    preflight = gate.run_gpu_preflight(run_root / "preflight")
    manifest = {
        "schema_version": "icra072_layer1_dev_run_v1",
        "run_id": run_root.name,
        "iterative_development": True,
        "development_only": True,
        "effect_claim": False,
        "install_root": str(install_root),
        "argv": list(sys.argv),
        "cwd": str(Path.cwd().resolve()),
        "commit": _git_commit(),
        "duration_s": args.duration_s,
        "gpu_ready": bool(preflight.get("gpu_ready")),
        "launch_started": False,
        "required_process_set": list(REQUIRED_PROCESSES),
        "stage_observations": {},
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
    child_environment = {**os.environ, "ROS_LOG_DIR": str(ros_logs)}
    capture = _start_process(
        capture_command, stdout=capture_log, stderr=subprocess.STDOUT,
        start_new_session=True, env=child_environment)
    atexit.register(_stop_owned, capture)
    readiness = _wait_ready(capture, run_root / "capture_ready.json")
    manifest["capture_readiness"] = readiness
    if readiness.get("ready") is not True:
        _stop_owned(capture)
        _unregister_cleanup_if_cleared(capture)
        capture_log.close()
        manifest["result"] = "CAPTURE_NOT_READY"
        _write(manifest_path, manifest)
        return 5

    launch_args = [
        "experiment:=icra_p0_p4_v2_p5_dev",
        "scenario:=icra072_p4_selection_trigger_v1",
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
        f"source {shlex.quote(str(setup))} && exec "
        + shlex.join([
            "ros2", "launch", "iap", "test_planner.launch.py", *launch_args])
    )
    _write(run_root / "launch_command.json", {
        "argv": ["bash", "-lc", shell_command],
        "profile": "icra_p0_p4_v2_p5_dev",
        "scenario": "icra072_p4_selection_trigger_v1",
    })
    launch_log = (run_root / "stdout.log").open("x")
    launch = _start_process(
        ["bash", "-lc", shell_command], stdout=launch_log,
        stderr=subprocess.STDOUT, start_new_session=True,
        env=child_environment)
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
        _unregister_cleanup_if_cleared(launch, capture)
    groups_cleared = (
        _owned_group_cleared(launch.pid) and
        _owned_group_cleared(capture.pid))
    runner_ok = (not early_exit and process_result["required_processes_ok"]
                 and groups_cleared)
    manifest.update({
        "launch_exit_code": exit_code,
        "launch_early_exit": early_exit,
        "capture_exit_code": capture_exit,
        "owned_process_groups_cleared": groups_cleared,
        "process_result": process_result,
        "stage_observations": _stage_observations(run_root),
        "result": "PASS" if runner_ok else "FAIL",
    })
    _write(manifest_path, manifest)
    return 0 if runner_ok else 6


if __name__ == "__main__":
    raise SystemExit(main())
