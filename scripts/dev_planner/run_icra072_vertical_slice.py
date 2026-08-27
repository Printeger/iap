#!/usr/bin/env python3
"""Run one fresh ICRA-072A Layer-1 iterative development attempt."""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import hashlib
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from types import SimpleNamespace
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
PROTECTED_UNTRACKED_PATH = "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"
PROTECTED_UNTRACKED_SHA256 = (
    "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6")


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


def _finalize_attempt(run_root: Path, manifest: dict,
                      runner_exit_code: int) -> int:
    manifest_path = run_root / "run_manifest.json"
    _write(manifest_path, manifest)
    command = [
        sys.executable,
        str(REPOSITORY / "scripts/dev_planner/analyze_icra072_vertical_slice.py"),
        "--run-root", str(run_root),
    ]
    try:
        completed = subprocess.run(
            command, cwd=REPOSITORY, capture_output=True, text=True,
            check=False)
    except OSError as exc:
        completed = SimpleNamespace(
            returncode=127, stdout="", stderr=f"{type(exc).__name__}: {exc}")
    _write(run_root / "analyzer_invocation.json", {
        "argv": command,
        "cwd": str(REPOSITORY),
        "exit_code": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    })
    try:
        analysis = json.loads((run_root / "analysis.json").read_text())
    except (OSError, json.JSONDecodeError) as exc:
        analysis = {
            "schema_version": "icra072_layer1_analysis_v1",
            "run_id": run_root.name,
            "result": "FAIL",
            "first_missing_stage": "p0_snapshot",
            "failures": [f"automatic_analyzer_failed:{exc}"],
        }
        if not (run_root / "analysis.json").exists():
            _write(run_root / "analysis.json", analysis)
    outcome = {
        "schema_version": "icra072_layer1_orchestration_outcome_v1",
        "run_id": run_root.name,
        "runner_result": manifest.get("result", "FAIL"),
        "runner_exit_code": runner_exit_code,
        "analyzer_result": analysis.get("result", "FAIL"),
        "analyzer_exit_code": completed.returncode,
        "first_missing_stage": analysis.get("first_missing_stage"),
        "failures": analysis.get("failures", []),
        "result": (
            "PASS" if runner_exit_code == 0 and completed.returncode == 0
            else "FAIL"),
    }
    _write(run_root / "orchestration_outcome.json", outcome)
    manifest["automatic_analyzer"] = {
        "argv": command,
        "exit_code": completed.returncode,
        "result": outcome["analyzer_result"],
        "first_missing_stage": outcome["first_missing_stage"],
    }
    _write(manifest_path, manifest)
    if runner_exit_code != 0:
        return runner_exit_code
    return 0 if completed.returncode == 0 else 7


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


def _capture_source_binding() -> dict:
    """Capture the committed, pushed tracked source admitted for a live run."""
    checks = {}
    for name, argv in (
            ("head", ["git", "rev-parse", "HEAD"]),
            ("origin_dev_icra",
             ["git", "rev-parse", "origin/dev/icra"]),
            ("status",
             ["git", "status", "--porcelain=v1", "--untracked-files=all"])):
        try:
            completed = subprocess.run(
                argv, cwd=REPOSITORY, capture_output=True, text=True,
                check=False)
            checks[name] = {
                "argv": argv,
                "exit_code": completed.returncode,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
        except OSError as exc:
            checks[name] = {
                "argv": argv, "exit_code": 127, "stdout": "",
                "stderr": f"{type(exc).__name__}: {exc}",
            }
    head = checks["head"]["stdout"].strip()
    origin = checks["origin_dev_icra"]["stdout"].strip()
    status_porcelain = checks["status"]["stdout"]
    status_entries = status_porcelain.splitlines()
    untracked_paths = [entry[3:] for entry in status_entries
                       if entry.startswith("?? ")]
    tracked_entries = [entry for entry in status_entries
                       if not entry.startswith("?? ")]
    rejected_untracked = [path for path in untracked_paths
                          if path != PROTECTED_UNTRACKED_PATH]
    protected_path = REPOSITORY / PROTECTED_UNTRACKED_PATH
    protected_hash = None
    if (protected_path.is_file() and not protected_path.is_symlink()):
        protected_hash = hashlib.sha256(protected_path.read_bytes()).hexdigest()
    protected_exact = (
        untracked_paths.count(PROTECTED_UNTRACKED_PATH) == 1
        and protected_hash == PROTECTED_UNTRACKED_SHA256)
    if (PROTECTED_UNTRACKED_PATH in untracked_paths
            and protected_hash != PROTECTED_UNTRACKED_SHA256):
        rejected_untracked.append(PROTECTED_UNTRACKED_PATH)
    tracked_status = "".join(entry + "\n" for entry in tracked_entries)
    failures = []
    if checks["head"]["exit_code"] != 0 or not re.fullmatch(
            r"[0-9a-f]{40}", head):
        failures.append("head_commit_unavailable")
    if (checks["origin_dev_icra"]["exit_code"] != 0 or
            not re.fullmatch(r"[0-9a-f]{40}", origin)):
        failures.append("origin_dev_icra_unavailable")
    if checks["status"]["exit_code"] != 0:
        failures.append("worktree_status_unavailable")
    else:
        if tracked_entries:
            failures.append("tracked_worktree_dirty")
        if rejected_untracked:
            failures.append("untracked_path_not_allowlisted")
        if not protected_exact:
            failures.append("protected_pdf_missing_or_hash_mismatch")
    if head and origin and head != origin:
        failures.append("head_origin_mismatch")
    return {
        "schema_version": "icra072_source_binding_v2",
        "repository": str(REPOSITORY),
        "head_commit": head,
        "origin_dev_icra_commit": origin,
        "status_porcelain": status_porcelain,
        "tracked_status": tracked_status,
        "tracked_worktree_clean": not tracked_entries,
        "untracked_allowlist": [{
            "path": PROTECTED_UNTRACKED_PATH,
            "sha256": PROTECTED_UNTRACKED_SHA256,
        }],
        "observed_untracked_paths": untracked_paths,
        "observed_protected_pdf_sha256": protected_hash,
        "rejected_untracked_paths": rejected_untracked,
        "rejected_tracked_entries": tracked_entries,
        "head_matches_origin": bool(head) and head == origin,
        "accepted": not failures,
        "failure_reasons": failures,
        "checks": checks,
    }


def _same_accepted_source(initial: dict, current: dict) -> bool:
    return (
        current.get("accepted") is True
        and current.get("head_commit") == initial.get("head_commit")
        and current.get("origin_dev_icra_commit") ==
        initial.get("origin_dev_icra_commit"))


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


def _finalize_runner_exception(
        run_root: Path, manifest: dict, exc: Exception,
        capture: subprocess.Popen | None = None,
        launch: subprocess.Popen | None = None,
        capture_log=None, launch_log=None, monitor=None) -> int:
    """Fail closed after any consumed-attempt orchestration exception."""
    cleanup_errors: list[str] = []
    process_result = {
        "required_processes_ok": False,
        "failure_reason": "runner_exception",
    }
    if monitor is not None:
        try:
            monitor.mark_controlled_shutdown()
            process_result = monitor.finish()
        except Exception as cleanup_exc:  # preserve the initiating failure
            cleanup_errors.append(
                f"monitor:{type(cleanup_exc).__name__}:{cleanup_exc}")
    exit_codes = {}
    for name, process, timeout_s in (
            ("launch", launch, 15.0), ("capture", capture, 5.0)):
        if process is None:
            continue
        try:
            exit_codes[name] = _stop_owned(process, timeout_s)
        except Exception as cleanup_exc:  # still emit typed evidence
            cleanup_errors.append(
                f"{name}:{type(cleanup_exc).__name__}:{cleanup_exc}")
    for stream in (launch_log, capture_log):
        if stream is not None:
            try:
                stream.close()
            except OSError as cleanup_exc:
                cleanup_errors.append(
                    f"log:{type(cleanup_exc).__name__}:{cleanup_exc}")
    processes = tuple(process for process in (launch, capture)
                      if process is not None)
    if processes:
        _unregister_cleanup_if_cleared(*processes)
    groups_cleared = all(
        _owned_group_cleared(process.pid) for process in processes)
    manifest.update({
        "launch_early_exit": bool(manifest.get("launch_started")),
        "launch_exit_code": exit_codes.get("launch"),
        "capture_exit_code": exit_codes.get("capture"),
        "owned_process_groups_cleared": groups_cleared,
        "process_result": process_result,
        "stage_observations": _stage_observations(run_root),
        "runner_exception": {
            "type": type(exc).__name__,
            "message": str(exc),
        },
        "cleanup_errors": cleanup_errors,
        "result": "RUNNER_EXCEPTION",
    })
    return _finalize_attempt(run_root, manifest, 8)


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
    exports = run_root / "exports"
    runtime = run_root / "runtime"
    bags = run_root / "bags"
    ros_logs = runtime / "ros_logs"
    manifest = {
        "schema_version": "icra072_layer1_dev_run_v1",
        "run_id": run_root.name,
        "iterative_development": True,
        "development_only": True,
        "effect_claim": False,
        "install_root": str(install_root),
        "argv": list(sys.argv),
        "cwd": str(Path.cwd().resolve()),
        "commit": None,
        "duration_s": args.duration_s,
        "gpu_ready": False,
        "launch_started": False,
        "required_process_set": list(REQUIRED_PROCESSES),
        "stage_observations": {},
    }
    run_root.mkdir(parents=True, exist_ok=False)
    capture = None
    launch = None
    capture_log = None
    launch_log = None
    monitor = None
    try:
        for path in (exports, runtime, bags, ros_logs):
            path.mkdir(parents=True)
        source_binding = _capture_source_binding()
        manifest["source_binding"] = source_binding
        manifest["commit"] = source_binding.get("head_commit")
        _write(run_root / "source_binding.json", source_binding)
        if source_binding.get("accepted") is not True:
            raise RuntimeError("source_binding_rejected:" + ",".join(
                source_binding.get("failure_reasons", [])))
        gate = _load_gate_runner()
        try:
            preflight = gate.run_gpu_preflight(run_root / "preflight")
        except Exception as exc:
            manifest["gpu_preflight_exception"] = {
                "type": type(exc).__name__, "message": str(exc)}
            print("GPU_NOT_READY")
            raise
        manifest["gpu_ready"] = bool(preflight.get("gpu_ready"))
        if not preflight.get("gpu_ready"):
            manifest["result"] = "GPU_NOT_READY"
            print("GPU_NOT_READY")
            return _finalize_attempt(run_root, manifest, 4)

        source_binding_recheck = _capture_source_binding()
        manifest["source_binding_recheck"] = source_binding_recheck
        if not _same_accepted_source(
                source_binding, source_binding_recheck):
            raise RuntimeError("source_binding_changed_before_ros")

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
            return _finalize_attempt(run_root, manifest, 5)

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
                "ros2", "launch", "iap", "test_planner.launch.py",
                *launch_args])
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
        monitor.mark_controlled_shutdown()
        exit_code = _stop_owned(launch, 15.0)
        process_result = monitor.finish()
        capture_exit = _stop_owned(capture, 5.0)
        launch_log.close()
        capture_log.close()
        _unregister_cleanup_if_cleared(launch, capture)
        source_binding_final = _capture_source_binding()
        manifest["source_binding_final"] = source_binding_final
        if not _same_accepted_source(source_binding, source_binding_final):
            raise RuntimeError("source_binding_changed_during_run")
    except Exception as exc:
        return _finalize_runner_exception(
            run_root, manifest, exc, capture, launch,
            capture_log, launch_log, monitor)
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
    return _finalize_attempt(run_root, manifest, 0 if runner_ok else 6)


if __name__ == "__main__":
    raise SystemExit(main())
