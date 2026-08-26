#!/usr/bin/env python3
"""Fail-closed analyzer for one ICRA-072A Layer-1 development run."""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
DEV_RUNS_ROOT = (REPOSITORY / "results/icra27/dev_runs/layer1").resolve()
RUN_ID_PATTERN = re.compile(r"run-[0-9]{3,}")
STAGE_ORDER = (
    "p0_snapshot", "closed_collision", "p4_selection_application",
    "ego_final_bspline", "p5_final_pass_before_publish",
    "normal_publication", "p5_runtime_committed",
)


def _contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _task_local(path: Path, label: str) -> Path:
    resolved = (path if path.is_absolute() else REPOSITORY / path).resolve()
    if not _contained(resolved, DEV_RUNS_ROOT):
        raise SystemExit(f"{label} must be under {DEV_RUNS_ROOT}")
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
    if (root.parent != DEV_RUNS_ROOT or
            RUN_ID_PATTERN.fullmatch(root.name) is None or
            not root.is_dir()):
        raise SystemExit(
            "run root must be an existing direct run-[0-9]{3,} directory")
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

    if (run.get("schema_version") != "icra072_layer1_dev_run_v1" or
            run.get("run_id") != root.name or
            run.get("iterative_development") is not True):
        failures.append("development_run_identity_mismatch")
    if not run.get("gpu_ready") or not run.get("launch_started"):
        failures.append("gpu_or_launch_admission_failed")
    if run.get("launch_early_exit") is not False:
        failures.append("launch_ended_before_registered_duration")
    if not run.get("process_result", {}).get("required_processes_ok"):
        failures.append("required_process_set_unhealthy")
    if (run.get("owned_process_groups_cleared") is not True or
            run.get("result") != "PASS"):
        failures.append("owned_process_group_cleanup_failed")
    expected_launch = {
        "experiment": "icra_p0_p4_v2_p5_dev",
        "scenario": "icra072_p4_selection_trigger_v1",
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
        "p5.current_pl_source": "LIDAR_CERTIFIED",
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

    def complete_support(row: dict[str, str], prefix: str) -> bool:
        fields = tuple(f"{prefix}_{suffix}" for suffix in (
            "sample_count", "valid_count", "unknown_count", "stale_count",
            "non_finite_count"))
        if any(field not in row or row[field] == "" for field in fields):
            return False
        try:
            sample_count = int(row.get(f"{prefix}_sample_count", "0"))
            valid_count = int(row.get(f"{prefix}_valid_count", "0"))
            unknown_count = int(row.get(f"{prefix}_unknown_count", "0"))
            stale_count = int(row.get(f"{prefix}_stale_count", "0"))
            non_finite_count = int(
                row.get(f"{prefix}_non_finite_count", "0"))
        except ValueError:
            return False
        return (sample_count > 0 and valid_count == sample_count and
                unknown_count == 0 and stale_count == 0 and
                non_finite_count == 0)

    original_complete = [row for row in p4
                         if complete_support(row, "original")]
    risk_complete = [row for row in p4 if complete_support(row, "risk")]
    both_complete = [row for row in p4
                     if complete_support(row, "original") and
                     complete_support(row, "risk")]
    selected = [row for row in selected
                if complete_support(row, "original") and
                complete_support(row, "risk")]
    selection_blockers = dict(sorted(Counter(
        row.get("reason", "missing_reason") or "missing_reason"
        for row in p4
        if not (row.get("status") == "RISK_SELECTED" and
                row.get("selection_applied") == "1")
    ).items()))
    decision_fields = (
        "planning_attempt_id", "collision_segment_id", "request_hash",
        "snapshot_generation_id", "snapshot_config_hash", "occupancy_epoch",
        "original_hash", "risk_hash", "selected_hash",
    )
    selected_by_key = {
        tuple(row.get(field, "") for field in decision_fields): row
        for row in selected
    }

    lineage_stages = {
        "final_bspline_before_p5", "p5_final_pass_before_publish",
        "normal_publish_authorized",
    }
    closed_collision = any(
        row.get("closed_collision_observed") == "1" and
        row.get("no_collision_refinement_observed") == "1"
        for row in lineage)
    selected_lineage = [
        row for row in lineage if row.get("stage") in lineage_stages
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
        if (row.get("closed_collision_observed") == "1" and
                row.get("no_collision_refinement_observed") == "1"):
            groups[key].append(row)
    identity_groups = [
        (key, rows) for key, rows in groups.items()
        if key[:len(decision_fields)] in selected_by_key]
    ego_groups = [
        (key, rows) for key, rows in identity_groups
        if "final_bspline_before_p5" in
        {row.get("stage") for row in rows}]
    if selected and not ego_groups:
        failures.append("p4_ego_p5_publish_lineage_identity_mismatch")
    linked_key, linked = ego_groups[-1] if ego_groups else ((), [])
    linked_decision = selected_by_key.get(
        linked_key[:len(decision_fields)]) if linked_key else None
    generation_identity_ok = True
    if linked_decision:
        generation = linked_decision.get("snapshot_generation_id")
        if generation not in {str(row.get("generation_id")) for row in ready_health}:
            failures.append("p0_p4_generation_identity_mismatch")
            generation_identity_ok = False
    trajectory_ids = {row.get("trajectory_id") for row in linked}
    linked_stages = {row.get("stage") for row in linked}
    if lineage_stages.issubset(linked_stages):
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
    bsplines = [row["payload"] for row in captured
                if row.get("kind") == "normal_bspline"]
    bspline_records = [row for row in captured
                       if row.get("kind") == "normal_bspline"]
    matching_final: list[dict] = []
    matching_bspline: list[dict] = []
    matching_runtime: list[dict] = []
    if len(trajectory_ids) == 1:
        expected_id = int(next(iter(trajectory_ids)))
        if not any(row.get("trajectory_id") == expected_id for row in bsplines):
            failures.append("lineage_bspline_trajectory_id_mismatch")
        if final_ok and not any(row.get("final_candidate_traj_id") == expected_id
                                for row in final_ok):
            failures.append("lineage_p5_final_trajectory_id_mismatch")
        matching_final = [
            row for row in final_records
            if row["payload"].get("final_candidate_traj_id") == expected_id]
        matching_bspline = [
            row for row in bspline_records
            if row["payload"].get("trajectory_id") == expected_id]
        try:
            lineage_starts = {
                float(row["trajectory_start_s"]) for row in linked}
            published_starts = {
                float(row["payload"]["start_time_s"])
                for row in matching_bspline}
        except (KeyError, TypeError, ValueError):
            lineage_starts, published_starts = set(), set()
        expected_starts = lineage_starts & published_starts
        matching_runtime = [
            row for row in runtime_records
            if any(
                sample.get("trajectory_sample_source") ==
                "runtime_committed" and
                any(abs(float(sample["trajectory_start_time_s"]) - start)
                    <= 0.02 for start in expected_starts)
                for sample in row["payload"].get("samples", [])
                if isinstance(sample.get("trajectory_start_time_s"),
                              (int, float)))]
        if not matching_runtime:
            failures.append("lineage_p5_runtime_trajectory_id_mismatch")
        if (matching_final and matching_bspline and matching_runtime and
            not (matching_final[-1]["receive_steady_s"] <=
                 matching_bspline[0]["receive_steady_s"] <=
                 matching_runtime[-1]["receive_steady_s"])):
            failures.append("p5_publish_runtime_capture_order_invalid")

    stage_status = {
        "p0_snapshot": bool(ready_health),
        "closed_collision": closed_collision,
        "p4_selection_application": bool(selected),
        "ego_final_bspline": bool(ego_groups) and generation_identity_ok,
        "p5_final_pass_before_publish": (
            "p5_final_pass_before_publish" in linked_stages and
            bool(matching_final)),
        "normal_publication": (
            "normal_publish_authorized" in linked_stages and
            bool(matching_bspline)),
        "p5_runtime_committed": bool(matching_runtime),
    }
    stage_failures = {
        "p0_snapshot": "p0_valid_immutable_generation_missing",
        "closed_collision": "truthful_closed_collision_missing",
        "p4_selection_application":
            "p4_v2_selected_guide_with_complete_support_missing",
        "ego_final_bspline": "ego_final_bspline_lineage_missing",
        "p5_final_pass_before_publish":
            "p5_final_before_publish_pass_missing",
        "normal_publication": "normal_bspline_publish_missing",
        "p5_runtime_committed": "p5_runtime_committed_binding_missing",
    }
    for stage in STAGE_ORDER:
        if not stage_status[stage] and stage_failures[stage] not in failures:
            failures.append(stage_failures[stage])
    first_missing_stage = next(
        (stage for stage in STAGE_ORDER if not stage_status[stage]), None)

    result = {
        "schema_version": "icra072_layer1_analysis_v1",
        "run_id": run.get("run_id", ""),
        "development_only": True,
        "effect_claim": False,
        "result": "PASS" if not failures else "FAIL",
        "failures": failures,
        "stage_order": list(STAGE_ORDER),
        "stage_status": stage_status,
        "first_missing_stage": first_missing_stage,
        "counts": {
            "p0_ready": len(ready_health), "p4_selected": len(selected),
            "lineage": len(linked), "p5_final_ok": len(final_ok),
            "p5_runtime_bound": len(runtime_bound),
            "normal_bspline": len(bsplines),
        },
        "provider_support": {
            "decision_count": len(p4),
            "original_complete_count": len(original_complete),
            "risk_complete_count": len(risk_complete),
            "both_complete_count": len(both_complete),
            "selection_blockers": selection_blockers,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(result["result"])
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
