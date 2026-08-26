#!/usr/bin/env python3
"""Fail-closed analyzer for the registered ICRA-072 development smoke."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
TASK_RESULTS_ROOT = (REPOSITORY / "results/icra27/icra072").resolve()


def _contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _task_local(path: Path, label: str) -> Path:
    resolved = path.resolve()
    if not _contained(resolved, TASK_RESULTS_ROOT):
        raise SystemExit(f"{label} must be under {TASK_RESULTS_ROOT}")
    return resolved


def _json(path: Path) -> dict:
    return json.loads(path.read_text())


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = _task_local(args.run_root, "run root")
    if not root.is_dir():
        raise SystemExit("run root must be an existing task-local directory")
    output = _task_local(args.output, "analysis output") if args.output else (
        root / "analysis.json")
    if not _contained(output, root):
        raise SystemExit("analysis output must be inside the run root")
    if output.exists():
        raise SystemExit("analysis output already exists")
    failures: list[str] = []
    try:
        run = _json(root / "run_manifest.json")
    except (OSError, json.JSONDecodeError) as exc:
        failures.append(f"run_manifest_missing_or_malformed:{exc}")
        run = {}
    launch_manifests = list(
        (root / "exports").glob("*/test_planner_manifest.json"))
    if len(launch_manifests) != 1:
        failures.append(f"launch_manifest_cardinality:{len(launch_manifests)}")
        launch = {}
    else:
        try:
            launch = _json(launch_manifests[0])
        except (OSError, json.JSONDecodeError) as exc:
            failures.append(f"launch_manifest_malformed:{exc}")
            launch = {}

    if run.get("run_id") != "icra072-dev-smoke-002" or not run.get("registered"):
        failures.append("registered_run_identity_mismatch")
    if not run.get("gpu_ready") or not run.get("launch_started"):
        failures.append("gpu_or_launch_admission_failed")
    if run.get("launch_early_exit") is not False:
        failures.append("launch_ended_before_registered_duration")
    if not run.get("process_result", {}).get("required_processes_ok"):
        failures.append("required_process_set_unhealthy")
    expected_launch = {
        "experiment": "icra_p0_p4_v2_p5_dev",
        "scenario": "icra_p0_p4_v2_p5_dev_fixture_v1",
        "planner_safety_profile": "icra_p0_p4_v2_p5_dev",
        "p0.enable_risk_grid": True,
        "p1.use_integrity_cost": False,
        "p2.enable_candidate_ranking": False,
        "p3.enable_local_reference_bias": False,
        "p3.enable_global_reference_bias": False,
        "p4.enable_risk_aware_astar": True,
        "p4.metrics_only": False,
        "p4.objective": "PROVIDER_BOTTLENECK_V2",
        "p5.enable_runtime_gate": True,
        "p5.enable_final_gate": True,
    }
    for key, value in expected_launch.items():
        if launch.get(key) != value:
            failures.append(f"launch_contract_mismatch:{key}")

    captured: list[dict] = []
    try:
        captured = [json.loads(line) for line in
                    (root / "lineage_capture.jsonl").read_text().splitlines()
                    if line.strip()]
    except (OSError, json.JSONDecodeError) as exc:
        failures.append(f"capture_missing_or_malformed:{exc}")
    health = [row["payload"] for row in captured
              if row.get("kind") == "p0_health"]
    ready_health = [row for row in health if row.get("ready") is True
                    and row.get("stale") is False
                    and int(row.get("generation_id", 0)) > 0]
    if not ready_health:
        failures.append("p0_valid_immutable_generation_missing")

    p4_binding = launch.get("p4.debug_csv_path")
    if "p4.debug_csv_path" not in launch:
        failures.append("p4_debug_path_missing")
        p4_path = None
    elif not isinstance(p4_binding, str) or not p4_binding.strip():
        failures.append("p4_debug_path_empty")
        p4_path = None
    else:
        p4_path = Path(p4_binding).resolve()
    if p4_path is not None and not _contained(p4_path, root):
        failures.append("p4_debug_path_outside_run_root")
        p4, lineage = [], []
    elif p4_path is None:
        p4, lineage = [], []
    elif not p4_path.is_file():
        failures.append("p4_debug_path_not_file")
        p4, lineage = [], []
    else:
        lineage_path = Path(str(p4_path) + ".lineage.csv")
        try:
            p4 = _rows(p4_path)
            lineage = _rows(lineage_path)
        except OSError as exc:
            failures.append(f"p4_evidence_missing:{exc}")
            p4, lineage = [], []
    selected = [row for row in p4
                if row.get("schema_version") == "p4_collision_guide_decision_v2"
                and row.get("status") == "RISK_SELECTED"
                and row.get("selection_applied") == "1"
                and all(row.get(field) for field in (
                    "planning_attempt_id", "collision_segment_id", "request_hash",
                    "snapshot_generation_id", "snapshot_config_hash",
                    "occupancy_epoch", "original_hash", "risk_hash",
                    "selected_hash"))]
    if not selected:
        failures.append("p4_v2_selected_guide_missing")

    decision_fields = (
        "planning_attempt_id", "collision_segment_id", "request_hash",
        "snapshot_generation_id", "snapshot_config_hash", "occupancy_epoch",
        "original_hash", "risk_hash", "selected_hash",
    )
    selected_by_key = {
        tuple(row.get(field, "") for field in decision_fields): row
        for row in selected
    }

    stages = {
        "final_bspline_before_p5", "p5_final_pass_before_publish",
        "normal_publish_authorized",
    }
    selected_lineage = [
        row for row in lineage if row.get("stage") in stages
        and row.get("selection_applied") == "1"]
    groups: dict[tuple[str, ...], list[dict[str, str]]] = defaultdict(list)
    for row in selected_lineage:
        decision_key = tuple(row.get(field, "") for field in (
            "planning_attempt_id", "collision_segment_id", "request_hash",
            "snapshot_generation_id", "snapshot_config_hash", "occupancy_epoch",
            "original_guide_hash", "risk_guide_hash", "selected_guide_hash",
        ))
        key = decision_key + tuple(row.get(field, "") for field in (
            "control_points_hash", "trajectory_id",
            "final_bspline_identity",
        ))
        groups[key].append(row)
    complete_groups = [
        (key, rows) for key, rows in groups.items()
        if {row.get("stage") for row in rows} == stages
        and key[:len(decision_fields)] in selected_by_key]
    if not complete_groups:
        failures.append("p4_ego_p5_publish_lineage_identity_mismatch")
    linked_key, linked = complete_groups[-1] if complete_groups else ((), [])
    linked_decision = selected_by_key.get(
        linked_key[:len(decision_fields)]) if linked_key else None
    if linked_decision:
        generation = linked_decision.get("snapshot_generation_id")
        if generation not in {str(row.get("generation_id")) for row in ready_health}:
            failures.append("p0_p4_generation_identity_mismatch")
    trajectory_ids = {row.get("trajectory_id") for row in linked}
    if linked:
        lineage_stamps = {
            row["stage"]: float(row["stamp_s"]) for row in linked}
        if not (lineage_stamps["final_bspline_before_p5"] <=
                lineage_stamps["p5_final_pass_before_publish"] <=
                lineage_stamps["normal_publish_authorized"]):
            failures.append("lineage_stage_order_invalid")

    final_records = [row for row in captured if row.get("kind") == "p5_status"
                     and row["payload"].get("phase") == "final"
                     and row["payload"].get("action") == "OK"
                     and not row["payload"].get("final_candidate_rejected")]
    final_ok = [row["payload"] for row in final_records]
    runtime_records = [row for row in captured if row.get("kind") == "p5_status"
                       and row["payload"].get("phase") == "runtime"
                       and any(sample.get("trajectory_sample_source")
                               == "runtime_committed"
                               for sample in row["payload"].get("samples", []))]
    runtime_bound = [row["payload"] for row in runtime_records]
    if not final_ok:
        failures.append("p5_final_before_publish_pass_missing")
    if not runtime_bound:
        failures.append("p5_runtime_committed_binding_missing")
    bsplines = [row["payload"] for row in captured
                if row.get("kind") == "normal_bspline"]
    bspline_records = [row for row in captured
                       if row.get("kind") == "normal_bspline"]
    if not bsplines:
        failures.append("normal_bspline_publish_missing")
    if len(trajectory_ids) == 1:
        expected_id = int(next(iter(trajectory_ids)))
        if not any(row.get("trajectory_id") == expected_id for row in bsplines):
            failures.append("lineage_bspline_trajectory_id_mismatch")
        if final_ok and not any(row.get("final_candidate_traj_id") == expected_id
                                for row in final_ok):
            failures.append("lineage_p5_final_trajectory_id_mismatch")
        matching_runtime = [
            row for row in runtime_records
            if row["payload"].get("final_candidate_traj_id") == expected_id]
        if not matching_runtime:
            failures.append("lineage_p5_runtime_trajectory_id_mismatch")
        matching_final = [
            row for row in final_records
            if row["payload"].get("final_candidate_traj_id") == expected_id]
        matching_bspline = [
            row for row in bspline_records
            if row["payload"].get("trajectory_id") == expected_id]
        if (matching_final and matching_bspline and matching_runtime and
            not (matching_final[-1]["receive_steady_s"] <=
                 matching_bspline[0]["receive_steady_s"] <=
                 matching_runtime[-1]["receive_steady_s"])):
            failures.append("p5_publish_runtime_capture_order_invalid")

    result = {
        "schema_version": "icra072_vertical_slice_analysis_v1",
        "run_id": run.get("run_id", ""),
        "development_only": True,
        "effect_claim": False,
        "result": "PASS" if not failures else "FAIL",
        "failures": failures,
        "counts": {
            "p0_ready": len(ready_health), "p4_selected": len(selected),
            "lineage": len(linked), "p5_final_ok": len(final_ok),
            "p5_runtime_bound": len(runtime_bound),
            "normal_bspline": len(bsplines),
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(result["result"])
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
