#!/usr/bin/env python3
"""Run the exact non-overwriting 40-row ICRA-075 development matrix."""

from __future__ import annotations

import argparse
import atexit
import importlib.util
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
RESULTS_ROOT = (REPOSITORY / "results/icra27/icra075").resolve()
SHARED_INSTALL = Path("/home/dev/ws_iap/install").resolve()
CORE_PATH = REPOSITORY / "scripts/dev_planner/icra075_exploratory.py"
BASE_RUNNER_PATH = REPOSITORY / "scripts/dev_planner/run_icra072_vertical_slice.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CORE = _load(CORE_PATH, "icra075_core")
BASE = _load(BASE_RUNNER_PATH, "icra075_base_runner")
GATE = BASE._load_gate_runner()


def _write(path: Path, value: dict):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def _resolve_new_matrix(path: Path) -> Path:
    result = (path if path.is_absolute() else REPOSITORY / path).resolve()
    try:
        relative = result.relative_to(RESULTS_ROOT)
    except ValueError as exc:
        raise SystemExit("matrix root must be repository-local under results/icra27/icra075") from exc
    if len(relative.parts) != 1 or not relative.name.startswith("matrix-") or result.exists():
        raise SystemExit("matrix root must be one new direct matrix-* directory")
    return result


def _same_source(first: dict, second: dict) -> bool:
    return (first.get("accepted") is True and second.get("accepted") is True and
            first.get("head_commit") == second.get("head_commit") and
            first.get("origin_dev_icra_commit") == second.get("origin_dev_icra_commit") and
            first.get("known_retained_inventory") ==
            second.get("known_retained_inventory"))


def _capture_source_binding(matrix_root: Path) -> dict:
    """Bind pushed tracked bytes while permitting this batch's own outputs."""
    checks = {}
    for name, argv in (
            ("head", ["git", "rev-parse", "HEAD"]),
            ("origin", ["git", "rev-parse", "origin/dev/icra"]),
            ("status", ["git", "status", "--porcelain=v1", "--untracked-files=all"])):
        completed = subprocess.run(argv, cwd=REPOSITORY, capture_output=True,
                                   text=True, check=False)
        checks[name] = {"argv": argv, "exit_code": completed.returncode,
                        "stdout": completed.stdout, "stderr": completed.stderr}
    head = checks["head"]["stdout"].strip()
    origin = checks["origin"]["stdout"].strip()
    entries = checks["status"]["stdout"].splitlines()
    tracked = [entry for entry in entries if not entry.startswith("?? ")]
    output_prefix = str(matrix_root.relative_to(REPOSITORY)) + "/"
    allowed_untracked = {"docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"}
    rejected_untracked = [entry[3:] for entry in entries if entry.startswith("?? ")
                          and entry[3:] not in allowed_untracked
                          and not entry[3:].startswith(output_prefix)]
    inventory = CORE.known_retained_inventory()
    accepted = (all(item["exit_code"] == 0 for item in checks.values()) and
                bool(head) and head == origin and not tracked and not rejected_untracked and
                all(item["accepted"] for item in inventory))
    return {
        "schema_version": "icra075_development_source_binding_v1",
        "accepted": accepted,
        "head_commit": head,
        "origin_dev_icra_commit": origin,
        "tracked_entries": tracked,
        "rejected_untracked_paths": rejected_untracked,
        "allowed_repository_output_prefix": output_prefix,
        "known_retained_inventory": inventory,
        "skipped_icra073_source_admission_debt": True,
        "checks": checks,
    }


def _finalize_row(root: Path, manifest: dict, runner_code: int) -> int:
    _write(root / "run_manifest.json", manifest)
    command = [sys.executable,
               str(REPOSITORY / "scripts/dev_planner/analyze_icra075_exploratory.py"),
               "--run-root", str(root)]
    completed = subprocess.run(command, cwd=REPOSITORY, capture_output=True,
                               text=True, check=False)
    _write(root / "analyzer_invocation.json", {
        "argv": command, "cwd": str(REPOSITORY), "exit_code": completed.returncode,
        "stdout": completed.stdout, "stderr": completed.stderr,
    })
    return runner_code if runner_code else completed.returncode


def _run_row(matrix_root: Path, row: dict, duration_s: float,
             source: dict) -> int:
    root = matrix_root / row["run_id"]
    root.mkdir()
    for name in ("exports", "runtime", "bags", "runtime/ros_logs"):
        (root / name).mkdir(parents=True, exist_ok=True)
    override = row["overrides"]
    launch_contract = {
        "planner_enable_p4": override["planner_enable_p4"],
        "p4.metrics_only": override["p4.metrics_only"],
        "p4.objective": override["p4.objective"],
        "manager/max_vel": 1.0,
        "manager/max_acc": 3.0,
        "manager/feasibility_tolerance": 0.05,
    }
    manifest = {
        "schema_version": "icra075_development_row_v1",
        "matrix_row": row,
        "development_only": True,
        "effect_claim": False,
        "source_bound_claim": False,
        "skipped_icra073_source_admission_debt": True,
        "source_binding": source,
        "launch_contract": launch_contract,
        "result": "FAIL",
        "owned_process_groups_cleared": False,
    }
    _write(root / "scene_binding.json", {
        **CORE.validate_scene_assets(),
        "scene": row["scene"], "scene_identity": row["scene_identity"],
        "descriptor_sha256": row["descriptor_sha256"],
    })
    recheck = _capture_source_binding(matrix_root)
    if not _same_source(source, recheck):
        manifest["first_missing_stage"] = "SOURCE_CHANGED_BEFORE_ROW"
        return _finalize_row(root, manifest, 3)
    capture = launch = None
    capture_log = launch_log = None
    monitor = None
    try:
        environment = {**os.environ, "ROS_LOG_DIR": str(root / "runtime/ros_logs")}
        capture_command = [
            sys.executable, str(REPOSITORY / "scripts/dev_planner/icra075_capture.py"),
            "--output", str(root / "lineage_capture.jsonl"),
            "--ready-file", str(root / "capture_ready.json"),
            "--duration-s", str(duration_s + 15.0),
            "--scene", row["scene"],
            "--descriptor-sha256", row["descriptor_sha256"],
        ]
        capture_log = (root / "capture_stdout.log").open("x")
        capture = BASE._start_process(capture_command, stdout=capture_log,
                                      stderr=subprocess.STDOUT,
                                      start_new_session=True, env=environment)
        atexit.register(BASE._stop_owned, capture)
        readiness = BASE._wait_ready(capture, root / "capture_ready.json")
        manifest["capture_readiness"] = readiness
        if readiness.get("ready") is not True:
            BASE._stop_owned(capture, 5.0)
            BASE._unregister_cleanup_if_cleared(capture)
            capture_log.close()
            manifest["owned_process_groups_cleared"] = BASE._owned_group_cleared(capture.pid)
            manifest["first_missing_stage"] = "CAPTURE_NOT_READY"
            return _finalize_row(root, manifest, 5)
        gnss_config = (SHARED_INSTALL /
                       "gnss_sim/share/gnss_sim/config/icra075_inverse_corridor_provider_v2.json")
        launch_args = [
            "experiment:=icra_p0_p4_v2_p5_dev",
            "scenario:=icra_p0_p4_v2_p5_dev_fixture_v1",
            f"icra075_scene_variant:={row['scene']}",
            f"icra075_descriptor_sha256:={row['descriptor_sha256']}",
            f"forest_random_seed:={row['seed']}",
            "init_x:=-12.0", "init_y:=0.0", "init_z:=1.5",
            "goal_x:=12.0", "goal_y:=0.0", "goal_z:=1.5",
            "manager/max_vel:=1.0", "optimization/max_vel:=1.0",
            "bspline/limit_vel:=1.0", "manager/max_acc:=3.0",
            "optimization/max_acc:=3.0", "bspline/limit_acc:=3.0",
            f"planner_enable_p4:={'true' if override['planner_enable_p4'] else 'false'}",
            f"p4.metrics_only:={'true' if override['p4.metrics_only'] else 'false'}",
            f"p4.objective:={override['p4.objective']}",
            f"gnss_scenario_file:={gnss_config}",
            "gnss_enable_map_occlusion:=true",
            f"run_duration_s:={duration_s + 30.0}",
            f"validation_duration_s:={duration_s + 20.0}",
            f"runtime_root_dir:={root / 'runtime'}",
            f"export_root_dir:={root / 'exports'}",
            f"iap_log_root:={root / 'runtime/iap_logs'}",
            f"bag_output_dir:={root / 'bags'}",
            f"p4.debug_csv_path:={root / 'exports/planner_p4_risk_astar_debug.csv'}",
            "record_bag:=false", "start_rviz:=false",
        ]
        shell_command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {shlex.quote(str(SHARED_INSTALL / 'setup.bash'))} && exec "
            + shlex.join(["ros2", "launch", "iap", "icra075_exploratory.launch.py",
                          *launch_args]))
        _write(root / "launch_command.json", {"argv": ["bash", "-lc", shell_command],
                                               "launch_arguments": launch_args})
        launch_log = (root / "stdout.log").open("x")
        launch = BASE._start_process(["bash", "-lc", shell_command], stdout=launch_log,
                                     stderr=subprocess.STDOUT,
                                     start_new_session=True, env=environment)
        atexit.register(BASE._stop_owned, launch)
        monitor = GATE.RequiredProcessMonitor(launch.pid, BASE.REQUIRED_PROCESSES, duration_s)
        monitor.start()
        monitor.launch_running = True
        manifest["launch_started"] = True
        started = time.monotonic()
        early_exit = False
        while time.monotonic() - started < duration_s:
            if launch.poll() is not None:
                early_exit = True
                break
            time.sleep(0.1)
        monitor.mark_controlled_shutdown()
        launch_exit = BASE._stop_owned(launch, 15.0)
        process_result = monitor.finish()
        capture_exit = BASE._stop_owned(capture, 5.0)
        launch_log.close()
        capture_log.close()
        BASE._unregister_cleanup_if_cleared(launch, capture)
        cleared = BASE._owned_group_cleared(launch.pid) and BASE._owned_group_cleared(capture.pid)
        final_source = _capture_source_binding(matrix_root)
        source_ok = _same_source(source, final_source)
        passed = (not early_exit and process_result["required_processes_ok"] and
                  cleared and source_ok)
        manifest.update({
            "launch_early_exit": early_exit, "launch_exit_code": launch_exit,
            "capture_exit_code": capture_exit, "process_result": process_result,
            "owned_process_groups_cleared": cleared,
            "source_binding_final": final_source,
            "result": "PASS" if passed else "FAIL",
            "first_missing_stage": None if passed else (
                "SOURCE_CHANGED_DURING_ROW" if not source_ok else
                "REQUIRED_PROCESS_OR_CLEANUP_FAILURE"),
        })
        return _finalize_row(root, manifest, 0 if passed else 6)
    except Exception as exc:
        for process in (launch, capture):
            if process is not None:
                BASE._stop_owned(process, 5.0)
        for stream in (launch_log, capture_log):
            if stream is not None and not stream.closed:
                stream.close()
        cleared = all(process is None or BASE._owned_group_cleared(process.pid)
                      for process in (launch, capture))
        manifest.update({
            "result": "FAIL", "owned_process_groups_cleared": cleared,
            "first_missing_stage": "RUNNER_EXCEPTION",
            "runner_exception": {"type": type(exc).__name__, "message": str(exc)},
        })
        return _finalize_row(root, manifest, 8)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-root", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=45.0)
    args = parser.parse_args()
    if args.duration_s < 30.0:
        raise SystemExit("ICRA-075 row duration must be at least 30 seconds")
    root = _resolve_new_matrix(args.matrix_root)
    root.mkdir(parents=True)
    protocol = CORE.load_protocol()
    matrix = CORE.build_matrix(protocol)
    scene_binding = CORE.validate_scene_assets()
    source = _capture_source_binding(root)
    batch = {
        "schema_version": "icra075_development_matrix_v1",
        "task": "ICRA-075", "development_only": True, "effect_claim": False,
        "source_bound_claim": False,
        "skipped_icra073_source_admission_debt": True,
        "matrix_cardinality": 40, "matrix": matrix,
        "scene_binding": scene_binding, "source_binding": source,
        "known_retained_inventory": CORE.known_retained_inventory(),
        "gpu_ready": False, "ros_started": False, "attempts": [],
        "result": "FAIL", "first_missing_stage": None,
    }
    _write(root / "protocol_snapshot.json", protocol)
    _write(root / "matrix_plan.json", {"rows": matrix})
    if (source.get("accepted") is not True or scene_binding.get("result") != "PASS" or
            not all(item["accepted"] for item in batch["known_retained_inventory"])):
        batch["first_missing_stage"] = "SOURCE_OR_SCENE_ADMISSION"
        _write(root / "batch_manifest.json", batch)
        return 3
    preflight = GATE.run_gpu_preflight(root / "preflight")
    batch["gpu_ready"] = preflight.get("gpu_ready") is True
    if not batch["gpu_ready"]:
        batch["result"] = "GPU_NOT_READY"
        batch["first_missing_stage"] = "GPU_PREFLIGHT"
        _write(root / "batch_manifest.json", batch)
        print("GPU_NOT_READY")
        return 4
    batch["ros_started"] = True
    for row in matrix:
        code = _run_row(root, row, args.duration_s, source)
        analysis_path = root / row["run_id"] / "analysis.json"
        analysis = _read_analysis(analysis_path)
        batch["attempts"].append({
            "run_id": row["run_id"], "exit_code": code,
            "result": analysis.get("result", "FAIL"),
            "first_missing_stage": analysis.get("first_missing_stage"),
        })
        _write(root / "batch_manifest.json", batch)
        if code != 0:
            batch["first_missing_stage"] = analysis.get("first_missing_stage") or "ROW_FAILURE"
            _write(root / "batch_manifest.json", batch)
            return code
    power_command = [
        sys.executable, str(REPOSITORY / "scripts/dev_planner/analyze_icra075_exploratory.py"),
        "--matrix-root", str(root)]
    power = subprocess.run(power_command, cwd=REPOSITORY, capture_output=True,
                           text=True, check=False)
    _write(root / "power_analyzer_invocation.json", {
        "argv": power_command, "exit_code": power.returncode,
        "stdout": power.stdout, "stderr": power.stderr,
    })
    final_source = _capture_source_binding(root)
    complete = (power.returncode == 0 and len(batch["attempts"]) == 40 and
                _same_source(source, final_source))
    batch.update({
        "result": "PASS" if complete else "FAIL",
        "first_missing_stage": None if complete else "POWER_INPUTS_OR_FINAL_SOURCE",
        "source_binding_final": final_source,
    })
    _write(root / "batch_manifest.json", batch)
    return 0 if complete else 9


def _read_analysis(path: Path) -> dict:
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


if __name__ == "__main__":
    raise SystemExit(main())
