#!/usr/bin/env python3
"""Run the fixed, no-bag ICRA Gate 0A/0B qualification protocol."""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil


SCENARIOS = {
    "primary": "p1_fork_fused_v1",
    "mirror": "p1_fork_fused_mirror_v1",
    "flat-null": "p1_fork_symmetric_null_v1",
}


REQUIRED_GATE0B_PROCESSES = {
    "iap_rosnode": ["iap_rosnode", "test_planner_iap_rosnode"],
}


class RequiredProcessMonitor:
    """Observe required descendants of the launch process started by this runner."""

    def __init__(
        self,
        launch_pid: int,
        required: dict[str, list[str]],
        run_duration_s: float,
    ):
        self.launch_pid = int(launch_pid)
        self.required = required
        self.run_duration_s = float(run_duration_s)
        self.launch_running = False
        self.start_time = None
        self.controlled_shutdown = False
        self._stop_event = threading.Event()
        self._seen: dict[str, bool] = {name: False for name in required}
        self._tracked: dict[str, dict[int, psutil.Process]] = {
            name: {} for name in required
        }
        self.failures: list[dict[str, object]] = []
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _descendants_matching(
        self, markers: list[str]
    ) -> list[tuple[int, str]]:
        try:
            root = psutil.Process(self.launch_pid)
            descendants = root.children(recursive=True)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return []
        matches = []
        for process in descendants:
            try:
                cmdline = " ".join(process.cmdline() or [])
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
            if any(marker in cmdline for marker in markers):
                matches.append((int(process.pid), cmdline))
        return matches

    def _record_failure(
        self,
        name: str,
        reason: str,
        exit_code_signal: object = None,
        phase: str = "runtime",
    ) -> None:
        with self._lock:
            self.failures.append({
                "process_name": name,
                "exit_code_signal": exit_code_signal,
                "phase": phase,
                "reason": reason,
                "elapsed_s": (
                    time.monotonic() - self.start_time
                    if self.start_time is not None else None
                ),
            })

    def _run(self) -> None:
        while not self._stop_event.wait(0.1):
            if not self.launch_running or self.start_time is None:
                continue
            for name, markers in self.required.items():
                current = self._descendants_matching(markers)
                current_pids = {pid for pid, _ in current}
                if current_pids:
                    with self._lock:
                        self._seen[name] = True
                        for pid, _ in current:
                            if pid not in self._tracked[name]:
                                try:
                                    self._tracked[name][pid] = psutil.Process(pid)
                                except psutil.NoSuchProcess:
                                    pass
                tracked_pids = list(self._tracked[name])
                for pid in tracked_pids:
                    if pid in current_pids:
                        continue
                    process = self._tracked[name].pop(pid, None)
                    exit_code_signal = None
                    if process is not None:
                        try:
                            exit_code_signal = process.wait(timeout=0.0)
                        except (psutil.NoSuchProcess, psutil.TimeoutExpired):
                            pass
                    with self._lock:
                        phase = (
                            "controlled_shutdown"
                            if self.controlled_shutdown else "runtime"
                        )
                    self._record_failure(
                        name,
                        "required_process_stopped_during_controlled_shutdown"
                        if phase == "controlled_shutdown"
                        else "required_process_died_before_controlled_shutdown",
                        exit_code_signal,
                        phase,
                    )

    def start(self) -> None:
        self.start_time = time.monotonic()
        self._thread.start()

    def mark_controlled_shutdown(self) -> None:
        with self._lock:
            self.controlled_shutdown = True

    def finish(self) -> dict[str, object]:
        self.launch_running = False
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        with self._lock:
            for name in self.required:
                if not self._seen.get(name, False):
                    self.failures.append({
                        "process_name": name,
                        "exit_code_signal": None,
                        "phase": "launch",
                        "reason": "required_process_never_started",
                        "elapsed_s": (
                            time.monotonic() - self.start_time
                            if self.start_time is not None else None
                        ),
                    })
            failures = [dict(item) for item in self.failures]
        return self.result(failures)

    def result(
        self, failures: list[dict[str, object]] | None = None
    ) -> dict[str, object]:
        failures = failures or [dict(item) for item in self.failures]
        runtime_failures = [
            item for item in failures if item.get("phase") == "runtime"
        ]
        required_processes_ok = (
            all(self._seen.get(name, False) for name in self.required)
            and not runtime_failures
        )
        return {
            "required_processes_ok": required_processes_ok,
            "process_failures": failures,
            "required_processes": {
                name: {
                    "seen": self._seen.get(name, False),
                    "runtime_failure": any(
                        item.get("process_name") == name
                        for item in runtime_failures
                    ),
                }
                for name in self.required
            },
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


def p0_effective_config(
    root: Path,
    run_duration_s: float = 60,
    validation_duration_s: float = 55,
) -> dict[str, object]:
    return {
        "experiment": "p0_open_sky",
        "scenario": "gnss_open_sky",
        "iap_mapping_backend": "cpu",
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
        "run_duration_s": run_duration_s,
        "validation_duration_s": validation_duration_s,
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


def _run_launch_with_monitor(
    command: list[str],
    stdout_path: Path,
    duration_s: float,
) -> tuple[int, dict[str, object]]:
    """Run a ROS launch and monitor only its descendants.

    The runner owns the controlled-shutdown transition: a timer marks it at
    ``duration_s`` and sends SIGINT to the launch process. Early descendant
    death before that transition is a runtime failure.
    """
    with stdout_path.open("w") as output:
        launch = subprocess.Popen(
            command,
            stdout=output,
            stderr=subprocess.STDOUT,
        )
        monitor = RequiredProcessMonitor(
            launch.pid, REQUIRED_GATE0B_PROCESSES, duration_s
        )

        def request_controlled_shutdown() -> None:
            monitor.mark_controlled_shutdown()
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)

        monitor.start()
        monitor.launch_running = True
        try:
            exit_code = launch.wait(timeout=max(0.1, duration_s - 0.25))
        except subprocess.TimeoutExpired:
            request_controlled_shutdown()
            try:
                exit_code = launch.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                launch.kill()
                exit_code = launch.wait(timeout=10.0)
        finally:
            monitor_result = monitor.finish()
    return exit_code, monitor_result


def _start_capture(command: list[str], stdout_path: Path):
    stream = stdout_path.open("w")
    capture = subprocess.Popen(
        command,
        stdout=stream,
        stderr=subprocess.STDOUT,
    )
    return capture, stream


def _finish_capture(
    capture: subprocess.Popen, stream, duration_s: float
) -> int:
    try:
        try:
            return capture.wait(timeout=duration_s + 10.0)
        except subprocess.TimeoutExpired:
            capture.terminate()
            try:
                return capture.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                capture.kill()
                return capture.wait(timeout=10.0)
    finally:
        stream.close()


def _finalize_manifest(run_dir: Path, **fields: object) -> int:
    manifest_path = run_dir / "gate0_run_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest.update(fields)
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return 0


def _gate0b_runner_exit(
    launch_exit: int,
    capture_exit: int,
    finalize_exit: int,
    monitor_result: dict[str, object],
) -> int:
    ok = (
        launch_exit == 0
        and capture_exit == 0
        and finalize_exit == 0
        and monitor_result.get("required_processes_ok") is True
    )
    return 0 if ok else 2


def run_gate0_smoke(root: Path, capture_script: Path) -> int:
    run_dir = root / "smoke"
    config = p0_effective_config(run_dir, run_duration_s=20, validation_duration_s=15)
    command = launch_command(config)
    _write_run_files(run_dir, "p0-smoke", "gnss_open_sky", config, command)
    capture_command = [
        sys.executable,
        str(capture_script),
        "--output", str(run_dir / "risk_grid_health.jsonl"),
        "--integrity-output", str(run_dir / "integrity_report.jsonl"),
        "--duration-s", "25",
    ]
    capture, capture_stream = _start_capture(
        capture_command, run_dir / "capture_stdout.log"
    )
    try:
        launch_exit, monitor_result = _run_launch_with_monitor(
            command, run_dir / "stdout.log", 20.0
        )
    finally:
        capture_exit = _finish_capture(capture, capture_stream, 25.0)
    manifest_fields = {
        "exit_code": launch_exit,
        "capture_exit_code": capture_exit,
        "planner_crash": launch_exit != 0,
        **monitor_result,
    }
    finalize_exit = _finalize_manifest(run_dir, **manifest_fields)
    return _gate0b_runner_exit(
        launch_exit, capture_exit, finalize_exit, monitor_result
    )


def run_gate0_benchmark(root: Path, capture_script: Path) -> int:
    run_dir = root / "benchmark"
    config = p0_effective_config(run_dir)
    command = launch_command(config)
    _write_run_files(run_dir, "p0-full-grid", "gnss_open_sky", config, command)
    capture_command = [
        sys.executable,
        str(capture_script),
        "--output", str(run_dir / "risk_grid_health.jsonl"),
        "--integrity-output", str(run_dir / "integrity_report.jsonl"),
        "--duration-s", "65",
    ]
    capture, capture_stream = _start_capture(
        capture_command, run_dir / "capture_stdout.log"
    )
    try:
        launch_exit, monitor_result = _run_launch_with_monitor(
            command, run_dir / "stdout.log", 60.0
        )
    finally:
        capture_exit = _finish_capture(capture, capture_stream, 65.0)
    manifest_fields = {
        "exit_code": launch_exit,
        "capture_exit_code": capture_exit,
        "planner_crash": launch_exit != 0,
        **monitor_result,
    }
    finalize_exit = _finalize_manifest(run_dir, **manifest_fields)
    return _gate0b_runner_exit(
        launch_exit, capture_exit, finalize_exit, monitor_result
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--gate0a-only", action="store_true")
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    args = parser.parse_args()
    if args.smoke and args.benchmark:
        parser.error("--smoke and --benchmark are mutually exclusive")
    if args.gate0a_only and (args.smoke or args.benchmark):
        parser.error("--gate0a-only cannot be combined with --smoke/--benchmark")
    root = args.output_root.resolve()
    # A/B may be executed as separate fixed phases into one closure root.
    # Individual run directories remain fail-closed against overwrite.
    root.mkdir(parents=True, exist_ok=True)
    capture_script = Path(__file__).with_name("gate0_capture_p0_health.py")
    exit_codes = []
    if args.gate0a_only or (not args.smoke and not args.benchmark):
        exit_codes.extend(run_gate0a(root))
    if args.smoke:
        exit_codes.append(run_gate0_smoke(root, capture_script))
    if args.benchmark:
        exit_codes.append(run_gate0_benchmark(root, capture_script))
    return 0 if all(code == 0 for code in exit_codes) else 2


if __name__ == "__main__":
    raise SystemExit(main())
