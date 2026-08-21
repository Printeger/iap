#!/usr/bin/env python3
"""Aggregate fixed ICRA Gate 0 qualification evidence."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


GATE0A_SCENARIO_MAP = {
    "primary": "p1_fork_fused_v1",
    "mirror": "p1_fork_fused_mirror_v1",
    "flat-null": "p1_fork_symmetric_null_v1",
}
GATE0A_SCENARIOS = tuple(GATE0A_SCENARIO_MAP)
EXPECTED_REFRESH_QUERY_COUNT = 76800
P0_FIELDS = (
    "generation_id",
    "refresh_query_count",
    "provider_query_count",
    "predictor_unique_positions",
    "predictor_effective_worker_count",
    "refresh_elapsed_ms",
    "provider_batch_duration_ms",
    "predictor_lidar_evaluations",
    "predictor_lidar_cache_hits",
    "ready",
    "stale",
    "valid_ratio",
    "unknown_ratio",
    "reason",
    "generation_interval_ms",
    "refresh_stamp_s",
    "failed_refresh",
    "snapshot_failure_reason",
    "odom_seen",
    "odom_valid",
    "odom_fresh",
    "odom_stamp_s",
    "current_integrity_seen",
    "current_integrity_valid",
    "current_integrity_fresh",
    "current_integrity_stamp_s",
    "gnss_epoch_seen",
    "gnss_epoch_valid",
    "gnss_epoch_fresh",
    "gnss_epoch_stamp_s",
    "gnss_epoch_satellite_count",
    "origin_seen",
    "origin_valid",
    "origin_fresh",
    "origin_stamp_s",
    "map_seen",
    "map_valid",
    "map_fresh",
    "map_stamp_s",
    "map_point_count",
)


def _float(value: Any, default: float = math.nan) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def canonical_control_points_hash(
    *, degree: int, ts: float, rows: int, cols: int, values: Iterable[float]
) -> tuple[str, str]:
    serialized_values = ",".join(format(float(value), ".17g") for value in values)
    canonical = (
        "gate0_control_points_v1\n"
        f"degree={int(degree)}\n"
        f"ts={format(float(ts), '.17g')}\n"
        f"rows={int(rows)}\n"
        f"cols={int(cols)}\n"
        f"values={serialized_values}\n"
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest(), canonical


def aggregate_candidate_events(
    rows: Iterable[dict[str, Any]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    grouped: dict[tuple[str, int], dict[str, Any]] = {}
    for row in rows:
        run_id = str(row.get("run_id", "")).strip()
        attempt_id = _int(row.get("planning_attempt_id"))
        if not run_id or attempt_id <= 0:
            continue
        key = (run_id, attempt_id)
        attempt = grouped.setdefault(
            key,
            {
                "run_id": run_id,
                "planning_attempt_id": attempt_id,
                "collision_segments": -1,
                "generated": -1,
                "input_candidates": set(),
                "success_candidates": set(),
                "candidate_costs": {},
                "original_candidate_id": 0,
                "selected_candidate_id": 0,
                "selection_count": 0,
                "refinement_candidates": set(),
                "feasible_candidates": set(),
                "updated_candidates": set(),
                "publish_count": 0,
                "attempt_start_count": 0,
                "attempt_complete_count": 0,
                "critical_reasons": [],
            },
        )
        event = str(row.get("event", ""))
        candidate_id = _int(row.get("candidate_id"))
        if event == "attempt_start":
            attempt["attempt_start_count"] += 1
        elif event == "attempt_candidates_complete":
            attempt["attempt_complete_count"] += 1
        for source, target in (
            ("collision_segment_count", "collision_segments"),
            ("base_generated_count", "generated"),
        ):
            value = _int(row.get(source), -1)
            if value >= 0:
                attempt[target] = max(attempt[target], value)
        if event == "optimizer_input" and candidate_id > 0:
            attempt["input_candidates"].add(candidate_id)
        elif event == "optimizer_result" and candidate_id > 0:
            if _int(row.get("optimizer_success"), -1) == 1:
                attempt["success_candidates"].add(candidate_id)
            original_cost = _float(row.get("original_cost"))
            final_cost = _float(row.get("final_cost"))
            if not math.isfinite(original_cost):
                reason = "candidate_original_cost_nonfinite"
                if reason not in attempt["critical_reasons"]:
                    attempt["critical_reasons"].append(reason)
            attempt["candidate_costs"][candidate_id] = (
                original_cost,
                final_cost,
            )
        elif event == "selection":
            attempt["selection_count"] += 1
            attempt["original_candidate_id"] = _int(
                row.get("original_candidate_id")
            )
            attempt["selected_candidate_id"] = _int(
                row.get("selected_candidate_id")
            )
        elif event == "refinement_result" and candidate_id > 0:
            attempt["refinement_candidates"].add(candidate_id)
            if _int(row.get("ego_feasible"), -1) == 1:
                attempt["feasible_candidates"].add(candidate_id)
        elif event == "update_traj_info" and candidate_id > 0:
            attempt["updated_candidates"].add(candidate_id)
        elif event == "normal_bspline_publish":
            attempt["publish_count"] = max(
                attempt["publish_count"],
                _int(row.get("bspline_publish_count")),
            )
        elif event in {"p1_fanout", "p1_supplement"}:
            attempt["critical_reasons"].append(event)

    candidate_rows: list[dict[str, Any]] = []
    attempt_rows: list[dict[str, Any]] = []
    for (run_id, attempt_id), attempt in sorted(grouped.items()):
        inputs = attempt["input_candidates"]
        successes = attempt["success_candidates"]
        generated = attempt["generated"]
        if generated < 0:
            attempt["critical_reasons"].append("generated_count_missing")
        if generated >= 0 and len(inputs) != generated:
            attempt["critical_reasons"].append("optimizer_input_count_mismatch")
        if attempt["attempt_start_count"] != 1:
            attempt["critical_reasons"].append("attempt_start_count_invalid")
        if attempt["attempt_complete_count"] != 1:
            attempt["critical_reasons"].append("attempt_complete_count_invalid")
        if successes and attempt["selection_count"] != 1:
            attempt["critical_reasons"].append("selection_count_invalid")
        if successes and attempt["original_candidate_id"] not in successes:
            attempt["critical_reasons"].append("original_candidate_not_successful")
        if successes and attempt["selected_candidate_id"] not in successes:
            attempt["critical_reasons"].append("selected_candidate_not_successful")
        selected = attempt["selected_candidate_id"]
        selected_refined = selected > 0 and selected in attempt["refinement_candidates"]
        selected_update_reached = selected > 0 and selected in attempt["updated_candidates"]
        selected_publish_reached = attempt["publish_count"] > 0
        reached_downstream = selected_update_reached or selected_publish_reached
        critical = bool(attempt["critical_reasons"])
        qualified = (
            generated >= 2
            and len(successes) >= 2
            and len(inputs) == generated
            and not critical
        )
        attempt_row = {
            "run_id": run_id,
            "planning_attempt_id": attempt_id,
            "collision_segments": attempt["collision_segments"],
            "generated": generated,
            "optimizer_input": len(inputs),
            "optimizer_success": len(successes),
            "original_candidate_id": attempt["original_candidate_id"],
            "selected_candidate_id": selected,
            "selected_refinement_reached": selected_refined,
            "selected_update_reached": selected_update_reached,
            "selected_publish_reached": selected_publish_reached,
            "selected_reached_downstream": reached_downstream,
            "normal_bspline_publish_count": attempt["publish_count"],
            "critical_violation": critical,
            "critical_reasons": ";".join(attempt["critical_reasons"]),
            "qualified": qualified,
        }
        attempt_rows.append(attempt_row)
        lineage_ids = {
            candidate_id
            for candidate_id in (
                attempt["original_candidate_id"], attempt["selected_candidate_id"]
            )
            if candidate_id > 0
        }
        candidate_ids = sorted(
            inputs | successes | set(attempt["candidate_costs"]) | lineage_ids
        ) or [0]
        for candidate_id in candidate_ids:
            original_cost, final_cost = attempt["candidate_costs"].get(
                candidate_id, (math.nan, math.nan)
            )
            candidate_rows.append({
                **attempt_row,
                "candidate_id": candidate_id,
                "candidate_optimizer_success": candidate_id in successes,
                "original_cost": original_cost,
                "final_cost": final_cost,
                "is_original": candidate_id == attempt["original_candidate_id"],
                "is_selected": candidate_id == selected,
            })
    return candidate_rows, attempt_rows


def decide_gate0a(run_rows: Iterable[dict[str, Any]]) -> dict[str, Any]:
    runs = [
        row for row in run_rows
        if row.get("scenario") in GATE0A_SCENARIOS
    ]
    if not runs:
        return {"status": "NO-GO-P2", "reason": "no_runs", "stable_scenarios": []}
    if any(_bool(row.get("critical_violation")) for row in runs):
        return {
            "status": "NO-GO-P2",
            "reason": "critical_violation",
            "stable_scenarios": [],
        }
    total_attempts = sum(_int(row.get("attempt_count")) for row in runs)
    under_two = sum(_int(row.get("under_two_success_attempts")) for row in runs)
    if total_attempts > 0 and under_two > total_attempts / 2:
        return {
            "status": "NO-GO-P2",
            "reason": "majority_attempts_under_two_success_candidates",
            "stable_scenarios": [],
        }
    stable = []
    for scenario in GATE0A_SCENARIOS:
        scenario_runs = [row for row in runs if row.get("scenario") == scenario]
        if (
            len(scenario_runs) == 3
            and {row.get("run_id") for row in scenario_runs} == {
                f"{scenario}-r{repeat}" for repeat in range(1, 4)
            }
            and all(_bool(row.get("qualified")) for row in scenario_runs)
        ):
            stable.append(scenario)
    if len(stable) == len(GATE0A_SCENARIOS) and all(
        _bool(row.get("selected_reached_downstream")) for row in runs
    ):
        status, reason = "GO", "all_scenarios_stable"
    elif stable:
        status, reason = "CONDITIONAL", "partial_scenario_stability"
    else:
        status, reason = "NO-GO-P2", "no_stable_scenario"
    return {"status": status, "reason": reason, "stable_scenarios": stable}


def validate_gate0a_manifest(
    manifest: dict[str, Any],
    runtime_manifest: dict[str, Any] | None = None,
    runtime_manifest_count: int = 0,
) -> list[str]:
    config = manifest.get("effective_config", {})
    failures = []
    scenario_label = str(manifest.get("scenario", ""))
    expected_scenario = GATE0A_SCENARIO_MAP.get(scenario_label)
    if expected_scenario is None or config.get("scenario") != expected_scenario:
        failures.append("scenario_mapping_mismatch")
    if manifest.get("run_id") not in {
        f"{scenario_label}-r{repeat}" for repeat in range(1, 4)
    }:
        failures.append("run_id_matrix_mismatch")
    expected_false = (
        "planner_enable_all_safety",
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
        "p0.enable_risk_grid",
        "p1.use_integrity_cost",
        "p1.metrics_only",
        "p1.debug_csv_enable",
        "manager/p1_collision_fanout_preserve_homotopies",
        "manager/p1_collision_fanout_mirror_y",
        "p2.enable_candidate_ranking",
        "p2.debug_csv_enable",
        "p3.enable_local_reference_bias",
        "p3.enable_global_reference_bias",
        "p3.debug_csv_enable",
        "p4.enable_risk_aware_astar",
        "p4.debug_csv_enable",
        "p5.enable_runtime_gate",
        "p5.enable_final_gate",
    )
    for field in expected_false:
        if config.get(field) is not False:
            failures.append(f"{field}_not_false")
    if _float(config.get("p1.lambda_integrity")) != 0.0:
        failures.append("p1.lambda_integrity_not_zero")
    if _float(config.get("manager/p1_collision_fanout_clearance_m")) != 0.0:
        failures.append("p1_fanout_clearance_not_zero")
    if config.get("manager/use_distinctive_trajs") is not True:
        failures.append("manager.use_distinctive_trajs_not_true")
    if config.get("record_bag") is not False:
        failures.append("record_bag_not_false")
    if (
        manifest.get("planner_crash") is not False
        or manifest.get("exit_code") != 0
    ):
        failures.append("planner_crash")
    if runtime_manifest_count != 1 or runtime_manifest is None:
        failures.append("runtime_manifest_count_invalid")
        return failures
    runtime_false = (
        "planner_enable_all_safety",
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
        "p0.enable_risk_grid",
        "p1.use_integrity_cost",
        "p1.metrics_only",
        "p1.debug_csv_enable",
        "manager/p1_collision_fanout_preserve_homotopies",
        "manager/p1_collision_fanout_mirror_y",
        "p2.enable_candidate_ranking",
        "p2.debug_csv_enable",
        "p3.enable_local_reference_bias",
        "p3.enable_global_reference_bias",
        "p3.debug_csv_enable",
        "p4.enable_risk_aware_astar",
        "p4.debug_csv_enable",
        "p5.enable_runtime_gate",
        "p5.enable_final_gate",
        "record_bag",
        "start_rviz",
    )
    for field in runtime_false:
        if runtime_manifest.get(field) is not False:
            failures.append(f"runtime_{field}_not_false")
    runtime_exact = {
        "scenario": config.get("scenario"),
        "experiment": "p1_fork_formal",
        "planner_safety_profile": "off",
        "manager/use_distinctive_trajs": True,
        "gate0.qualification_evidence_enable": True,
        "gate0.evidence_run_id": manifest.get("run_id"),
        "run_validator": True,
        "run_duration_s": 90.0,
        "validation_duration_s": 85.0,
        "forest_random_seed": 11,
        "gnss_random_seed": 20260011,
        "terminal_wall_feature_seed": 11022,
    }
    for field, expected in runtime_exact.items():
        if runtime_manifest.get(field) != expected:
            failures.append(f"runtime_{field}_mismatch")
    geometry_mirror = config.get("scenario") == "p1_fork_fused_mirror_v1"
    if runtime_manifest.get("p1_fixture_mirror_y") is not geometry_mirror:
        failures.append("runtime_geometry_mirror_mismatch")
    for field in (
        "gate0.candidate_events_path",
        "gate0.control_points_path",
        "gate0.evidence_manifest_path",
    ):
        if runtime_manifest.get(field) != config.get(field):
            failures.append(f"runtime_{field}_mismatch")
    if _float(runtime_manifest.get("p1.lambda_integrity")) != 0.0:
        failures.append("runtime_p1.lambda_integrity_not_zero")
    if _float(runtime_manifest.get("manager/p1_collision_fanout_clearance_m")) != 0.0:
        failures.append("runtime_p1_fanout_clearance_not_zero")
    return failures


def validate_gate0b_manifest(
    manifest: dict[str, Any] | None,
    runtime_manifest: dict[str, Any] | None,
    runtime_manifest_count: int,
) -> list[str]:
    if manifest is None:
        return ["p0_runner_manifest_missing"]
    failures = []
    config = manifest.get("effective_config", {})
    expected_run_duration = 20 if manifest.get("run_id") == "p0-smoke" else 60
    expected_validation_duration = (
        15 if manifest.get("run_id") == "p0-smoke" else 55
    )
    expected = {
        "experiment": "p0_open_sky",
        "scenario": "gnss_open_sky",
        "iap_mapping_backend": "cpu",
        "planner_safety_profile": "off",
        "p0.enable_risk_grid": True,
        "p0.size_x_m": 30.0,
        "p0.size_y_m": 30.0,
        "p0.size_z_m": 6.0,
        "p0.resolution_m": 0.75,
        "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5",
        "p0.refresh_period_s": 0.5,
        "p0.predictor.worker_count": 1,
        "p0.skip_occupied_voxels": True,
        "record_bag": False,
        "start_rviz": False,
        "run_duration_s": expected_run_duration,
        "validation_duration_s": expected_validation_duration,
    }
    for field, value in expected.items():
        if config.get(field) != value:
            failures.append(f"p0_{field}_mismatch")
    isolation_fields = (
        "planner_enable_all_safety",
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
        "p1.use_integrity_cost",
        "p1.metrics_only",
        "p2.enable_candidate_ranking",
        "p3.enable_local_reference_bias",
        "p3.enable_global_reference_bias",
        "p4.enable_risk_aware_astar",
        "p5.enable_runtime_gate",
        "p5.enable_final_gate",
    )
    for field in isolation_fields:
        if config.get(field) is not False:
            failures.append(f"p0_{field}_not_false")
    if (
        manifest.get("planner_crash") is not False
        or manifest.get("exit_code") != 0
        or manifest.get("capture_exit_code") != 0
    ):
        failures.append("p0_process_failure")
    if manifest.get("required_processes_ok") is not True:
        failures.append("p0_required_process_failure")
    gpu_preflight = manifest.get("gpu_preflight", {})
    if (
        gpu_preflight.get("schema_version") != "iap_gpu_preflight_v1"
        or gpu_preflight.get("gpu_ready") is not True
        or gpu_preflight.get("failure_reason") not in ("", None)
    ):
        failures.append("p0_gpu_preflight_not_ready")
    capture_readiness = manifest.get("capture_readiness", {})
    if (
        capture_readiness.get("schema_version")
        != "gate0_capture_readiness_v1"
        or capture_readiness.get("ready") is not True
    ):
        failures.append("p0_capture_not_ready_before_launch")
    required_processes = manifest.get("required_processes", {})
    iap_rosnode_seen = bool(
        required_processes.get("iap_rosnode", {}).get("seen", False)
    )
    iap_rosnode_runtime_failure = bool(
        required_processes.get("iap_rosnode", {}).get("runtime_failure", False)
    )
    if not iap_rosnode_seen or iap_rosnode_runtime_failure:
        failures.append("p0_iap_rosnode_not_alive_through_runtime")
    if any(
        item.get("phase") == "runtime"
        for item in manifest.get("process_failures", [])
    ):
        failures.append("p0_runtime_process_failure")
    if runtime_manifest_count != 1 or runtime_manifest is None:
        failures.append("p0_runtime_manifest_count_invalid")
        return failures
    runtime_expected = {
        "experiment": "p0_open_sky",
        "scenario": "gnss_open_sky",
        "planner_safety_profile": "off",
        "p0.enable_risk_grid": True,
        "p0.size_x_m": 30.0,
        "p0.size_y_m": 30.0,
        "p0.size_z_m": 6.0,
        "p0.resolution_m": 0.75,
        "p0.horizons_s": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],
        "p0.refresh_period_s": 0.5,
        "p0.predictor.effective_worker_count": 1,
        "p0.skip_occupied_voxels": True,
        "record_bag": False,
        "start_rviz": False,
        "run_duration_s": float(expected_run_duration),
        "validation_duration_s": float(expected_validation_duration),
    }
    for field, value in runtime_expected.items():
        if runtime_manifest.get(field) != value:
            failures.append(f"p0_runtime_{field}_mismatch")
    if runtime_manifest.get("iap_mapping_backend") != "cpu":
        failures.append("p0_runtime_mapping_backend_mismatch")
    mapping_effective = runtime_manifest.get("mapping_effective_config")
    if not isinstance(mapping_effective, dict) or mapping_effective.get(
        "selected"
    ) != "cpu":
        failures.append("p0_runtime_mapping_effective_missing_or_wrong")
    for field in (
        "odometry_config",
        "sub_mapping_config",
        "global_mapping_config",
    ):
        entry = mapping_effective.get(field) if isinstance(mapping_effective, dict) else None
        if not isinstance(entry, dict):
            failures.append(f"p0_runtime_mapping_{field}_missing")
            continue
        if not str(entry.get("path", "")).strip():
            failures.append(f"p0_runtime_mapping_{field}_path_missing")
        sha = str(entry.get("sha256", ""))
        if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
            failures.append(f"p0_runtime_mapping_{field}_sha256_invalid")
    runtime_isolation_fields = (
        "planner_enable_all_safety",
        "planner_enable_p1",
        "planner_enable_p2",
        "planner_enable_p3_local",
        "planner_enable_p3_global",
        "planner_enable_p4",
        "planner_enable_p5_runtime",
        "planner_enable_p5_final",
        "p1.use_integrity_cost",
        "p1.metrics_only",
        "p1.debug_csv_enable",
        "p2.enable_candidate_ranking",
        "p2.debug_csv_enable",
        "p3.enable_local_reference_bias",
        "p3.enable_global_reference_bias",
        "p3.debug_csv_enable",
        "p4.enable_risk_aware_astar",
        "p4.debug_csv_enable",
        "p5.enable_runtime_gate",
        "p5.enable_final_gate",
        "gate0.qualification_evidence_enable",
    )
    for field in runtime_isolation_fields:
        if runtime_manifest.get(field) is not False:
            failures.append(f"p0_runtime_{field}_not_false")
    return failures


def type7_quantile(values: Iterable[float], probability: float) -> float:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return math.nan
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * float(probability)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    fraction = position - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def analyze_p0_messages(
    messages: Iterable[dict[str, Any]],
    protocol: str = "benchmark",
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if protocol not in {"smoke", "benchmark"}:
        raise ValueError(f"unsupported P0 protocol: {protocol}")
    callbacks: dict[float, dict[str, Any]] = {}
    for message in messages:
        callback_key = _float(message.get("refresh_callback_end_steady_s"))
        if not math.isfinite(callback_key):
            callback_key = _float(message.get("refresh_stamp_s"))
        if math.isfinite(callback_key):
            callbacks[callback_key] = message

    rows: list[dict[str, Any]] = []
    successful_generations: dict[int, dict[str, Any]] = {}
    for _, message in sorted(callbacks.items()):
        generation_id = _int(message.get("generation_id"))
        success = (
            generation_id > 0
            and _bool(message.get("ready"))
            and _int(message.get("refresh_query_count")) > 0
        )
        row = {field: message.get(field, "") for field in P0_FIELDS}
        row["generation_id"] = generation_id
        row["failed_refresh"] = 0 if success else 1
        if success:
            successful_generations[generation_id] = row
        else:
            rows.append(row)
    rows.extend(successful_generations[generation] for generation in sorted(successful_generations))
    rows.sort(key=lambda row: (_float(row.get("refresh_stamp_s")), _int(row.get("generation_id"))))

    successful = [row for row in rows if not _bool(row["failed_refresh"])]
    latencies = [_float(row.get("refresh_elapsed_ms")) for row in successful]
    latency_evidence_failure = bool(successful) and any(
        not math.isfinite(value) for value in latencies
    )
    intervals = [_float(row.get("generation_interval_ms")) for row in successful]
    intervals = [value for value in intervals if math.isfinite(value)]
    shape_ok = bool(successful) and all(
        _int(row.get("refresh_query_count")) == EXPECTED_REFRESH_QUERY_COUNT
        for row in successful
    )
    p95 = (
        math.nan
        if latency_evidence_failure
        else type7_quantile(latencies, 0.95)
    )
    failures = []
    if len(successful) == 0:
        failures.append("zero_successful_generations")
    minimum_successful_generations = 1 if protocol == "smoke" else 20
    if len(successful) < minimum_successful_generations:
        failures.append("fewer_than_20_successful_generations")
    if not shape_ok:
        failures.append("refresh_query_shape_mismatch")
    if latency_evidence_failure:
        failures.append("successful_generation_latency_nonfinite")
    if protocol == "benchmark" and (not math.isfinite(p95) or p95 > 400.0):
        failures.append("refresh_p95_over_400_ms")
    if len(successful) == 0:
        gate = "P0_INPUT_AVAILABILITY_FAIL"
        recommendations: list[str] = []
    else:
        gate = "PASS" if not failures else "P0_PERFORMANCE_GATE_FAIL"
        recommendations = [] if (
            protocol == "smoke"
            or len(successful) < 20
            or not failures
        ) else [
            "evaluate predictor worker_count",
            "reduce ICRA ROI",
            "reduce frozen horizons",
            "increase refresh period",
        ]
    summary = {
        "schema_version": "gate0_p0_summary_v1",
        "protocol": protocol,
        "minimum_successful_generations": minimum_successful_generations,
        "expected_refresh_query_count": EXPECTED_REFRESH_QUERY_COUNT,
        "successful_generation_count": len(successful),
        "failed_refresh_count": len(rows) - len(successful),
        "refresh_elapsed_ms_p50": type7_quantile(latencies, 0.50),
        "refresh_elapsed_ms_p95": p95,
        "refresh_elapsed_ms_max": max(latencies) if latencies else math.nan,
        "stale_ratio": (
            sum(_bool(row.get("stale")) for row in rows) / len(rows) if rows else 1.0
        ),
        "failed_ratio": (
            (len(rows) - len(successful)) / len(rows) if rows else 1.0
        ),
        "generation_interval_ms_p50": type7_quantile(intervals, 0.50),
        "generation_interval_ms_p95": type7_quantile(intervals, 0.95),
        "p95_over_refresh_period": p95 / 500.0 if math.isfinite(p95) else math.nan,
        "query_shape_ok": shape_ok,
        "gate": gate,
        "failures": failures,
        "recommendations": recommendations,
    }
    return rows, summary


def _read_csvs(paths: Iterable[Path]) -> list[dict[str, Any]]:
    rows = []
    for path in paths:
        with path.open(newline="") as stream:
            rows.extend(csv.DictReader(stream))
    return rows


def aggregate_control_points(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, int, int, str], list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        key = (
            str(row.get("run_id", "")),
            _int(row.get("planning_attempt_id")),
            _int(row.get("candidate_id")),
            str(row.get("stage", "")),
        )
        grouped[key].append(row)
    output = []
    for key, points in sorted(grouped.items()):
        points.sort(key=lambda row: (_int(row.get("point_row")), _int(row.get("point_col"))))
        first = points[0]
        values = [_float(row.get("value")) for row in points]
        digest, canonical = canonical_control_points_hash(
            degree=_int(first.get("degree")),
            ts=_float(first.get("ts")),
            rows=_int(first.get("rows")),
            cols=_int(first.get("cols")),
            values=values,
        )
        output.append({
            "run_id": key[0],
            "planning_attempt_id": key[1],
            "candidate_id": key[2],
            "stage": key[3],
            "degree": _int(first.get("degree")),
            "ts": _float(first.get("ts")),
            "rows": _int(first.get("rows")),
            "cols": _int(first.get("cols")),
            "original_cost": _float(first.get("original_cost")),
            "final_cost": _float(first.get("final_cost")),
            "sha256": digest,
            "control_points_17g": canonical.split("values=", 1)[1].strip(),
        })
    return output


def validate_control_point_evidence(
    rows: Iterable[dict[str, Any]],
    candidate_rows: Iterable[dict[str, Any]],
) -> tuple[dict[tuple[str, int], list[str]], bool]:
    candidates = list(candidate_rows)
    known_candidate_keys = {
        (
            str(candidate.get("run_id", "")),
            _int(candidate.get("planning_attempt_id")),
            _int(candidate.get("candidate_id")),
        )
        for candidate in candidates
        if _int(candidate.get("candidate_id")) > 0
    }
    violations: dict[tuple[str, int], list[str]] = defaultdict(list)
    grouped: dict[tuple[str, int, int, str], list[dict[str, Any]]] = defaultdict(list)
    global_identity_failure = False
    for row in rows:
        run_id = str(row.get("run_id", "")).strip()
        attempt_id = _int(row.get("planning_attempt_id"))
        candidate_id = _int(row.get("candidate_id"))
        stage = str(row.get("stage", "")).strip()
        if not run_id or attempt_id <= 0 or candidate_id <= 0 or not stage:
            global_identity_failure = True
            continue
        grouped[(run_id, attempt_id, candidate_id, stage)].append(row)

    stages: dict[tuple[str, int, int], set[str]] = defaultdict(set)
    for (run_id, attempt_id, candidate_id, stage), points in grouped.items():
        if (run_id, attempt_id, candidate_id) not in known_candidate_keys:
            global_identity_failure = True
        key = (run_id, attempt_id)
        stages[(run_id, attempt_id, candidate_id)].add(stage)
        metadata = {
            (
                _int(row.get("degree")),
                format(_float(row.get("ts")), ".17g"),
                _int(row.get("rows")),
                _int(row.get("cols")),
            )
            for row in points
        }
        if len(metadata) != 1:
            violations[key].append(f"{candidate_id}:{stage}:metadata_mismatch")
            continue
        degree, ts_text, row_count, col_count = next(iter(metadata))
        ts = _float(ts_text)
        coordinates = [
            (_int(row.get("point_row"), -1), _int(row.get("point_col"), -1))
            for row in points
        ]
        expected = {
            (row, column)
            for row in range(row_count)
            for column in range(col_count)
        }
        if (
            degree < 1
            or not math.isfinite(ts)
            or ts <= 0.0
            or row_count <= 0
            or col_count <= 0
            or len(coordinates) != len(set(coordinates))
            or set(coordinates) != expected
            or any(not math.isfinite(_float(row.get("value"))) for row in points)
        ):
            violations[key].append(f"{candidate_id}:{stage}:matrix_incomplete")

    for candidate in candidates:
        candidate_id = _int(candidate.get("candidate_id"))
        if candidate_id <= 0:
            continue
        run_id = str(candidate.get("run_id", ""))
        attempt_id = _int(candidate.get("planning_attempt_id"))
        required = {"generated", "optimizer_input"}
        if _bool(candidate.get("candidate_optimizer_success")):
            required.add("optimized")
        if _bool(candidate.get("is_selected")):
            required.add("selected")
            if _bool(candidate.get("selected_reached_downstream")):
                required.add("post_refinement")
        missing = required - stages.get((run_id, attempt_id, candidate_id), set())
        if missing:
            violations[(run_id, attempt_id)].append(
                f"{candidate_id}:missing_stages:{','.join(sorted(missing))}"
            )
    return violations, global_identity_failure


def apply_attempt_violations(
    candidate_rows: list[dict[str, Any]],
    attempt_rows: list[dict[str, Any]],
    violations: dict[tuple[str, int], list[str]],
    global_violation: str = "",
) -> None:
    by_key = {
        (row["run_id"], row["planning_attempt_id"]): row
        for row in attempt_rows
    }
    for key, attempt in by_key.items():
        reasons = list(violations.get(key, []))
        if global_violation:
            reasons.append(global_violation)
        if not reasons:
            continue
        existing = [reason for reason in attempt["critical_reasons"].split(";") if reason]
        attempt["critical_reasons"] = ";".join([*existing, *sorted(set(reasons))])
        attempt["critical_violation"] = True
        attempt["qualified"] = False
    for candidate in candidate_rows:
        attempt = by_key[(candidate["run_id"], candidate["planning_attempt_id"])]
        for field in ("critical_reasons", "critical_violation", "qualified"):
            candidate[field] = attempt[field]


def _write_csv(path: Path, rows: list[dict[str, Any]], fields: Iterable[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(fields or (rows[0].keys() if rows else []))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    messages = []
    if not path.exists():
        return messages
    for line in path.read_text().splitlines():
        if line.strip():
            messages.append(json.loads(line))
    return messages


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def analyze_directory(root: Path, output: Path) -> dict[str, Any]:
    event_paths = sorted(root.glob("**/candidate_events.csv"))
    point_paths = sorted(root.glob("**/candidate_control_points_raw.csv"))
    event_rows = _read_csvs(event_paths)
    raw_point_rows = _read_csvs(point_paths)
    candidate_rows, attempts = aggregate_candidate_events(event_rows)
    point_rows = aggregate_control_points(raw_point_rows)
    point_violations, point_identity_failure = validate_control_point_evidence(
        raw_point_rows, candidate_rows
    )
    event_identity_failure = any(
        not str(row.get("run_id", "")).strip()
        or _int(row.get("planning_attempt_id")) <= 0
        for row in event_rows
    )
    apply_attempt_violations(
        candidate_rows,
        attempts,
        point_violations,
        "attempt_identity_unprovable"
        if event_identity_failure or point_identity_failure else "",
    )
    manifests = []
    manifest_paths: dict[str, Path] = {}
    for path in sorted(root.glob("**/gate0_run_manifest.json")):
        manifest = json.loads(path.read_text())
        runtime_paths = sorted(
            path.parent.glob("exports/**/test_planner_manifest.json")
        )
        manifest["runtime_manifest_count"] = len(runtime_paths)
        manifest["runtime_manifest"] = (
            json.loads(runtime_paths[0].read_text())
            if len(runtime_paths) == 1 else None
        )
        manifest_paths[manifest["run_id"]] = path.parent
        manifests.append(manifest)
    scenarios = {manifest["run_id"]: manifest.get("scenario", "") for manifest in manifests}
    manifest_failures = {
        manifest["run_id"]: validate_gate0a_manifest(
            manifest,
            manifest.get("runtime_manifest"),
            _int(manifest.get("runtime_manifest_count")),
        )
        for manifest in manifests
        if manifest.get("scenario") in GATE0A_SCENARIOS
    }
    run_rows = []
    for run_id in sorted({row["run_id"] for row in attempts} | set(scenarios)):
        run_attempts = [row for row in attempts if row["run_id"] == run_id]
        run_rows.append({
            "run_id": run_id,
            "scenario": scenarios.get(run_id, ""),
            "qualified": any(row["qualified"] for row in run_attempts),
            "selected_reached_downstream": any(
                row["selected_reached_downstream"] for row in run_attempts
            ),
            "selected_refinement_reached": any(
                row["selected_refinement_reached"] for row in run_attempts
            ),
            "selected_update_reached": any(
                row["selected_update_reached"] for row in run_attempts
            ),
            "selected_publish_reached": any(
                row["selected_publish_reached"] for row in run_attempts
            ),
            "critical_violation": (
                any(row["critical_violation"] for row in run_attempts)
                or bool(manifest_failures.get(run_id))
            ),
            "manifest_failures": ";".join(manifest_failures.get(run_id, [])),
            "attempt_count": len(run_attempts),
            "under_two_success_attempts": sum(
                row["optimizer_success"] < 2 for row in run_attempts
            ),
        })
    gate0a = decide_gate0a(run_rows)
    _write_csv(output / "candidate_qualification.csv", candidate_rows)
    _write_csv(output / "candidate_control_points.csv", point_rows)
    effective = {"schema_version": "gate0_effective_config_v1", "runs": manifests}
    (output / "effective_config.json").write_text(
        json.dumps(_json_safe(effective), indent=2, sort_keys=True) + "\n"
    )

    p0_run_ids = [
        run_id
        for run_id in ("p0-smoke", "p0-full-grid")
        if run_id in manifest_paths
    ]
    p0_manifest = (
        next(
            manifest
            for manifest in manifests
            if manifest.get("run_id") == p0_run_ids[-1]
        )
        if p0_run_ids else None
    )
    p0_run_dir = (
        manifest_paths[p0_run_ids[-1]]
        if p0_run_ids else None
    )
    p0_messages = (
        _read_jsonl(p0_run_dir / "risk_grid_health.jsonl")
        if p0_run_dir else []
    )
    p0_protocol = (
        "smoke" if p0_run_ids and p0_run_ids[-1] == "p0-smoke"
        else "benchmark"
    )
    p0_rows, p0_summary = analyze_p0_messages(
        p0_messages, protocol=p0_protocol
    )
    integrity_messages = (
        _read_jsonl(p0_run_dir / "integrity_report.jsonl")
        if p0_run_dir else []
    )
    valid_integrity_count = sum(
        _bool(item.get("valid")) for item in integrity_messages
    )
    p0_summary["integrity_report_count"] = len(integrity_messages)
    p0_summary["valid_integrity_report_count"] = valid_integrity_count
    if p0_run_ids and p0_run_ids[-1] == "p0-smoke" and valid_integrity_count == 0:
        p0_summary["failures"].append("no_valid_integrity_report")
        p0_summary["gate"] = "P0_INPUT_AVAILABILITY_FAIL"
    p0_manifest_failures = validate_gate0b_manifest(
        p0_manifest,
        p0_manifest.get("runtime_manifest") if p0_manifest else None,
        _int(p0_manifest.get("runtime_manifest_count")) if p0_manifest else 0,
    )
    if p0_manifest_failures:
        p0_summary["failures"] = list(dict.fromkeys([
            *p0_summary["failures"], *p0_manifest_failures,
        ]))
        if p0_summary.get("gate") != "P0_INPUT_AVAILABILITY_FAIL":
            p0_summary["gate"] = "P0_PERFORMANCE_GATE_FAIL"
    p0_summary["manifest_failures"] = p0_manifest_failures
    p0_stem = (
        "p0_full_grid" if p0_run_ids and p0_run_ids[-1] == "p0-full-grid"
        else "p0_smoke"
    )
    _write_csv(output / f"{p0_stem}_benchmark.csv", p0_rows, P0_FIELDS)
    (output / f"{p0_stem}_summary.json").write_text(
        json.dumps(_json_safe(p0_summary), indent=2, sort_keys=True) + "\n"
    )
    result = {"gate0a": gate0a, "gate0b": p0_summary, "runs": run_rows}
    (output / "gate0_analysis.json").write_text(
        json.dumps(_json_safe(result), indent=2, sort_keys=True) + "\n"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate0-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    result = analyze_directory(args.gate0_root.resolve(), args.output_dir.resolve())
    print(json.dumps(_json_safe(result), indent=2, sort_keys=True))
    return 0 if result.get("gate0b", {}).get("gate") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
