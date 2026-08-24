#!/usr/bin/env python3
"""Fail-closed runner for the registered P4-G0C calibration matrix.

ICRA-042 registers this tool and tests it with synthetic boundaries only. Live
execution remains unauthorized until a later task explicitly invokes it.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from p4_g0c_protocol import (  # noqa: E402
    ProtocolBundle,
    effective_config_sha256,
    expand_run_plan,
    load_protocol_bundle,
)
from run_gate0_qualification import (  # noqa: E402
    RequiredProcessMonitor,
    run_gpu_preflight,
)


RUNNER_SCHEMA = "p4_g0c_runner_state_v1"
REQUIRED_PROCESSES = {
    "iap_rosnode": ["iap_rosnode", "test_planner_iap_rosnode"],
    "ego_planner_node": ["ego_planner_node", "drone_0_ego_planner_node"],
}
REQUIRED_CSV_COLUMNS = {
    "schema_version",
    "request_hash",
    "status",
    "reason",
    "selection_applied",
    "original_hash",
    "risk_hash",
    "selected_hash",
    "original_sample_count",
    "original_valid_count",
    "original_unknown_count",
    "original_stale_count",
    "original_non_finite_count",
    "risk_sample_count",
    "risk_valid_count",
    "risk_unknown_count",
    "risk_stale_count",
    "risk_non_finite_count",
    "original_mean",
    "original_max",
    "risk_mean",
    "risk_max",
    "path_length_ratio",
    "original_search_latency_ms",
    "risk_search_latency_ms",
    "total_search_latency_ms",
}
INTEGER_CSV_COLUMNS = {
    "selection_applied",
    "original_sample_count",
    "original_valid_count",
    "original_unknown_count",
    "original_stale_count",
    "original_non_finite_count",
    "risk_sample_count",
    "risk_valid_count",
    "risk_unknown_count",
    "risk_stale_count",
    "risk_non_finite_count",
}
FLOAT_CSV_COLUMNS = {
    "original_mean",
    "original_max",
    "risk_mean",
    "risk_max",
    "path_length_ratio",
    "original_search_latency_ms",
    "risk_search_latency_ms",
    "total_search_latency_ms",
}


class RunnerError(RuntimeError):
    """The matrix cannot proceed without violating the registered protocol."""


def _validate_typed_csv_row(row: dict[str, str], run_id: str) -> None:
    if row.get("schema_version") != "p4_collision_guide_decision_v1":
        raise RunnerError(f"malformed P4 decision CSV: {run_id}:schema")
    if any(row.get(key, "") == "" for key in REQUIRED_CSV_COLUMNS):
        raise RunnerError(f"malformed P4 decision CSV: {run_id}:empty_field")
    for key in INTEGER_CSV_COLUMNS:
        raw = row[key].strip()
        try:
            value = int(raw)
        except ValueError as exc:
            raise RunnerError(
                f"malformed P4 decision CSV: {run_id}:{key}"
            ) from exc
        if str(value) != raw:
            raise RunnerError(f"malformed P4 decision CSV: {run_id}:{key}")
    for key in FLOAT_CSV_COLUMNS:
        try:
            value = float(row[key])
        except ValueError as exc:
            raise RunnerError(
                f"malformed P4 decision CSV: {run_id}:{key}"
            ) from exc
        if not math.isfinite(value):
            raise RunnerError(f"malformed P4 decision CSV: {run_id}:{key}")


def load_bundle(
    protocol_path: Path, registry_path: Path, fixture_path: Path
) -> ProtocolBundle:
    bundle = load_protocol_bundle(protocol_path, registry_path, fixture_path)
    bundle.protocol_path = str(Path(protocol_path).resolve())
    bundle.registry_path = str(Path(registry_path).resolve())
    bundle.fixture_path = str(Path(fixture_path).resolve())
    return bundle


def launch_command(bundle: ProtocolBundle, record: dict[str, Any]) -> list[str]:
    run_dir = Path(record["run_dir"]).resolve()
    manifest_path = run_dir / "p4_g0c_run_manifest.json"
    csv_path = run_dir / "p4_decisions.csv"
    values = {
        "experiment": "p4_g0c_metrics_calibration_v1",
        "p4.g0c.protocol_path": bundle.protocol_path,
        "p4.g0c.protocol_sha256": bundle.protocol_sha256,
        "p4.g0c.registry_path": bundle.registry_path,
        "p4.g0c.registry_sha256": bundle.registry_sha256,
        "p4.g0c.fixture_path": bundle.fixture_path,
        "p4.g0c.fixture_sha256": bundle.fixture_sha256,
        "p4.g0c.run_id": record["run_id"],
        "p4.g0c.seed": record["seed"],
        "p4.g0c.repetition": record["repetition"],
        "p4.g0c.run_manifest_path": str(manifest_path),
        "p4.g0c.csv_path": str(csv_path),
        "runtime_root_dir": str(run_dir / "runtime"),
        "export_root_dir": str(run_dir / "exports"),
        "iap_log_root": str(run_dir / "runtime" / "iap_logs"),
    }
    return [
        "ros2", "launch", "iap", "test_planner.launch.py",
        *[f"{key}:={value}" for key, value in values.items()],
    ]


def _execute_launch(
    record: dict[str, Any], command: list[str], duration_s: float,
    required: dict[str, list[str]],
) -> tuple[int, dict[str, Any]]:
    run_dir = Path(record["run_dir"])
    with (run_dir / "stdout.log").open("w") as output:
        launch = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT
        )
        monitor = RequiredProcessMonitor(launch.pid, required, duration_s)

        def controlled_shutdown() -> None:
            monitor.mark_controlled_shutdown()
            if launch.poll() is None:
                launch.send_signal(signal.SIGINT)

        monitor.start()
        monitor.launch_running = True
        try:
            exit_code = launch.wait(timeout=max(0.1, duration_s - 0.25))
        except subprocess.TimeoutExpired:
            controlled_shutdown()
            try:
                exit_code = launch.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                launch.kill()
                exit_code = launch.wait(timeout=10.0)
        finally:
            monitor_result = monitor.finish()
    return int(exit_code), monitor_result


def _validate_and_finalize_run(
    bundle: ProtocolBundle,
    record: dict[str, Any],
    monitor_result: dict[str, Any],
) -> None:
    run_dir = Path(record["run_dir"])
    manifest_path = run_dir / "p4_g0c_run_manifest.json"
    csv_path = run_dir / "p4_decisions.csv"
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RunnerError(f"missing or malformed run manifest: {record['run_id']}") from exc
    required_manifest = {
        "schema_version": "p4_g0c_run_manifest_v1",
        "run_id": record["run_id"],
        "seed": record["seed"],
        "repetition": record["repetition"],
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "csv_path": str(csv_path.resolve()),
        "gate": "G0C",
        "experiment": "p4_g0c_metrics_calibration_v1",
        "scenario": "p4_g0c_free_corridor_v1",
        "decision_schema_version": "p4_collision_guide_decision_v1",
        "effective_values": bundle.protocol["effective_values"],
        "effective_config_sha256": effective_config_sha256(
            bundle.protocol["effective_values"]
        ),
        "required_process_set": list(REQUIRED_PROCESSES),
        "selection_applied": False,
        "record_bag": False,
        "start_rviz": False,
        "immutable_run_id": True,
        "overwrite_allowed": False,
    }
    for key, expected in required_manifest.items():
        if manifest.get(key) != expected:
            raise RunnerError(f"run manifest binding mismatch: {record['run_id']}:{key}")
    try:
        with csv_path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None or not REQUIRED_CSV_COLUMNS.issubset(reader.fieldnames):
                raise RunnerError(f"malformed P4 decision CSV: {record['run_id']}")
            rows = list(reader)
            if not rows:
                raise RunnerError(f"empty P4 decision CSV: {record['run_id']}")
            for row in rows:
                _validate_typed_csv_row(row, record["run_id"])
    except OSError as exc:
        raise RunnerError(f"missing P4 decision CSV: {record['run_id']}") from exc
    manifest.update(monitor_result)
    manifest["runner_state"] = "COMPLETE"
    manifest["launch_exit_code"] = 0
    manifest["retry_count"] = 0
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def _base_result(bundle: ProtocolBundle, plan: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": RUNNER_SCHEMA,
        "runner_state": "PLANNED",
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "runs": plan,
        "completed_run_count": 0,
        "launch_invocations": 0,
        "launch_started": False,
        "retries": 0,
        "failure_reason": "",
    }


def _persist_result(runs_root: Path, result: dict[str, Any]) -> None:
    runs_root.mkdir(parents=True, exist_ok=True)
    (runs_root / "p4_g0c_runner_state.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )


def run(
    bundle: ProtocolBundle,
    runs_root: Path,
    *,
    plan_only: bool = False,
    preflight_only: bool = False,
    gpu_preflight: Callable[[Path], dict[str, Any]] = run_gpu_preflight,
    launch_executor: Callable[..., tuple[int, dict[str, Any]]] = _execute_launch,
) -> dict[str, Any]:
    if plan_only and preflight_only:
        raise RunnerError("plan-only and preflight-only are mutually exclusive")
    runs_root = Path(runs_root).expanduser().resolve()
    plan = expand_run_plan(bundle.protocol, runs_root)
    result = _base_result(bundle, plan)
    if plan_only:
        return result
    for record in plan:
        if Path(record["run_dir"]).exists():
            raise RunnerError(f"existing run directory is forbidden: {record['run_dir']}")

    preflight = gpu_preflight(runs_root / "preflight")
    result["gpu_preflight"] = preflight
    if preflight.get("gpu_ready") is not True:
        result.update({
            "runner_state": "FAILED",
            "failure_reason": "GPU_NOT_READY",
            "gpu_failure_reason": preflight.get("failure_reason", "unknown"),
        })
        _persist_result(runs_root, result)
        return result
    result["runner_state"] = "PREFLIGHT_PASS"
    if preflight_only:
        _persist_result(runs_root, result)
        return result

    duration_s = 90.0
    for record in plan:
        run_dir = Path(record["run_dir"])
        run_dir.mkdir(parents=True, exist_ok=False)
        command = launch_command(bundle, record)
        (run_dir / "launch_command.json").write_text(
            json.dumps(command, indent=2) + "\n"
        )
        result["runner_state"] = "RUNNING"
        result["launch_started"] = True
        result["launch_invocations"] += 1
        exit_code, monitor_result = launch_executor(
            record, command, duration_s, REQUIRED_PROCESSES
        )
        if exit_code != 0 or monitor_result.get("required_processes_ok") is not True:
            result.update({
                "runner_state": "FAILED",
                "failure_reason": (
                    f"launch_exit_{exit_code}" if exit_code != 0
                    else "required_process_failure"
                ),
                "failed_run_id": record["run_id"],
                "required_process_monitor": monitor_result,
            })
            _persist_result(runs_root, result)
            return result
        try:
            _validate_and_finalize_run(bundle, record, monitor_result)
        except RunnerError as exc:
            result.update({
                "runner_state": "FAILED",
                "failure_reason": str(exc),
                "failed_run_id": record["run_id"],
            })
            _persist_result(runs_root, result)
            return result
        result["completed_run_count"] += 1

    result["runner_state"] = "COMPLETE"
    _persist_result(runs_root, result)
    return result


def _parser() -> argparse.ArgumentParser:
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=repo / "config/icra27/p4_g0c_protocol_v1.json")
    parser.add_argument("--registry", type=Path, default=repo / "config/icra27/p4_threshold_registry_v1.json")
    parser.add_argument("--fixture", type=Path, default=repo / "config/icra27/p4_g0c_live_fixture_v1.json")
    parser.add_argument("--runs-root", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--plan-only", action="store_true")
    modes.add_argument("--preflight-only", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        bundle = load_bundle(args.protocol, args.registry, args.fixture)
        result = run(
            bundle,
            args.runs_root,
            plan_only=args.plan_only,
            preflight_only=args.preflight_only,
        )
    except (RunnerError, RuntimeError) as exc:
        print(f"P4_G0C_RUNNER_FAILED: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    if result["failure_reason"] == "GPU_NOT_READY":
        print("GPU_NOT_READY", file=sys.stderr)
        return 2
    return 0 if result["runner_state"] in {"PLANNED", "PREFLIGHT_PASS", "COMPLETE"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
