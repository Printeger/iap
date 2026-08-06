#!/usr/bin/env python3

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "analyze_safety_planner_run.py"
)
spec = importlib.util.spec_from_file_location("analyze_safety_planner_run", SCRIPT_PATH)
analyzer = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(analyzer)


def p1_2_manifest(**overrides):
    manifest = {
        "experiment": "p1_degraded_lidar_good",
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "p1",
        "run_duration_s": 90.0,
        "validation_duration_s": 90.0,
        "timebase": {
            "planning_timeline": {"domain": "sim_message", "field": "stamp_s"},
            "p0_health_payload": {
                "domain": "sim_message",
                "field": "health_callback_stamp_s",
            },
            "bag_receive": {"domain": "system_receive", "field": "stamp"},
        },
        "p0.enable_risk_grid": True,
        "p0.resolution_m": 0.75,
        "p0.stale_timeout_s": 1.0,
        "planner_enable_p1": True,
        "p1.metrics_only": False,
        "p1.use_integrity_cost": True,
        "p1.lambda_integrity": 0.00001,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
        "planner_enable_p5_runtime": False,
        "planner_enable_p5_final": False,
    }
    manifest.update(overrides)
    return manifest


def p1_1_reference_manifest(**overrides):
    manifest = p1_2_manifest(**{"p1.metrics_only": True})
    manifest.update(overrides)
    return manifest


def topic_health(**overrides):
    health = {
        topic: {"status": "PASS", "count": 3}
        for topic in analyzer.P1_TOPIC_EXPECTATIONS
    }
    health.update(overrides)
    return health


def metadata(**topic_count_overrides):
    topic_counts = {
        analyzer.P5_STATUS_TOPIC: 0,
        **{topic: 0 for topic in analyzer.P5_RVIZ_TOPICS},
        "/iap/rviz/p2_candidate_trajectories": 0,
        "/iap/rviz/p3_reference_bias": 0,
        "/iap/rviz/p4_astar_guides": 0,
    }
    topic_counts.update(topic_count_overrides)
    return {"missing": False, "topic_counts": topic_counts}


def p0_row(idx, **overrides):
    row = {
        "stamp": float(idx),
        "health_callback_stamp_s": float(idx),
        "ready": True,
        "stale": False,
        "valid_ratio": 1.0,
        "unknown_ratio": 0.0,
        "reason": "ok",
        "refresh_duration_ms": 400.0,
        "refresh_queue_delay_ms": 10.0,
        "provider_batch_duration_ms": 350.0,
        "generation_interval_ms": 500.0,
        "input_callback_count": idx + 1,
        "health_callback_count": idx + 1,
        "process_cpu_delta_ms": 25.0,
        "generation_id": 4,
        "last_grid_stamp_s": 9.0,
    }
    row.update(overrides)
    return row


def p1_debug_row(**overrides):
    row = {
        "stamp": 1.0,
        "lbfgs_iter": 1,
        "planning_attempt_id": 11,
        "candidate_id": 7,
        "snapshot_generation_id": 4,
        "query_base_time_s": 9.0,
        "sample_count": 4,
        "hit_count": 3,
        "miss_count": 1,
        "stale_count": 0,
        "miss_ratio": 0.25,
        "stale_ratio": 0.0,
        "f_integrity": 1.2,
        "weighted_f_integrity": 0.02,
        "grad_norm_integrity": 0.1,
        "grad_norm_original": 2.0,
        "grad_ratio": 0.05,
        "clipped_grad_count": 0,
        "fallback_reason": "ok",
        "applied_to_objective": 1,
    }
    row.update(overrides)
    return row


def p1_debug_summary(rows=None, **kwargs):
    return analyzer.summarize_p1_integrity_debug_rows(
        [p1_debug_row()] if rows is None else rows,
        **kwargs,
    )


def candidate_optimization_row(**overrides):
    row = {field: 1.0 for field in analyzer.P1_CANDIDATE_OPTIMIZATION_FIELDS}
    row.update({
        "schema_version": analyzer.P1_EVIDENCE_SCHEMA_VERSION,
        "run_id": "0123456789abcdef0123456789abcdef",
        "manifest_path": "/tmp/test_planner_manifest.json",
        "planning_attempt_id": 11, "candidate_id": 7,
        "snapshot_generation_id": 4, "query_base_time_s": 9.0,
        "pre_base_objective": 10.0, "post_base_objective": 9.0,
        "pre_total_objective": 10.1, "post_total_objective": 9.1,
        "pre_raw_p1_cost": 100.0, "post_raw_p1_cost": 90.0,
        "pre_weighted_p1_cost": 0.001, "post_weighted_p1_cost": 0.0009,
        "pre_base_gradient_norm": 2.0, "post_base_gradient_norm": 1.0,
        "pre_raw_p1_gradient_norm": 0.1, "post_raw_p1_gradient_norm": 0.05,
        "pre_weighted_p1_gradient_norm": 0.000001,
        "post_weighted_p1_gradient_norm": 0.0000005,
        "pre_total_gradient_norm": 2.0, "post_total_gradient_norm": 1.0,
        "pre_normalized_p1_cost": 0.1, "post_normalized_p1_cost": -0.05,
        "pre_anchor_cost": 0.0, "post_anchor_cost": 0.15,
        "normalization_mode": "base_improvement_budget_v1",
        "normalization_reference_lambda": 0.00001,
        "normalization_scale": 1000.0,
        "normalization_budget_fraction": 0.1,
        "normalization_base_improvement_budget": 1.0,
        "normalization_reference_displacement_m": 0.025,
        "base_prepass_pre_objective": 11.0,
        "base_prepass_post_objective": 10.0,
        "base_prepass_duration_ms": 1.0,
        "base_prepass_solver_result": 0,
        "base_prepass_iteration_count": 4,
        "base_prepass_success": 1,
        "base_prepass_termination_reason": "converged",
        "displacement_norm": 0.1, "grad_integrity_dot_displacement": -0.01,
        "pre_mean_c_pi": 5.0, "pre_max_c_pi": 6.0,
        "post_mean_c_pi": 4.0, "post_max_c_pi": 5.0,
        "support_sample_count": 200, "pre_support_valid_count": 200,
        "post_support_valid_count": 200, "pre_support_coverage": 1.0,
        "post_support_coverage": 1.0, "support_full_valid": 1,
        "support_signature": "snapshot-lattice-v1",
        "initial_control_points_hash": "initial-v1", "p1_config_hash": "config-v1",
        "optimization_success": 1, "selected": 1, "solver_result": 0,
        "iteration_count": 4, "termination_reason": "converged",
        "objective_allowed": 1, "objective_applied": 1,
        "selection_score": 9.1, "selection_reason": "single_candidate",
        "candidate_rank": 1, "p1_descent": 1, "rank_eligible": 1,
        "replacement_accepted": 1, "replacement_reason": "initial_p1_candidate",
        "incumbent_available": 0,
        "incumbent_mean_c_pi": 5.1, "incumbent_max_c_pi": 6.1,
        "fallback_reason": "none", "max_candidates_per_attempt": 8,
    })
    row.update(overrides)
    return row


def bspline_rows(y=0.0, *, zigzag=False):
    if zigzag:
        pts = [
            (0.0, 0.0, 1.0),
            (1.0, 3.0, 1.0),
            (2.0, -3.0, 1.0),
            (3.0, 3.0, 1.0),
            (4.0, -3.0, 1.0),
            (5.0, 0.0, 1.0),
        ]
    else:
        pts = [(float(x), y, 1.0) for x in range(11)]
    return [
        {
            "bag_time_s": 1.0,
            "traj_id": 1,
            "start_time_s": 0.0,
            "duration_s": 3.0,
            "order": 3,
            "pos_pts_count": len(pts),
            "knots_count": len(pts) + 3,
            "pos_pts_xyz": pts,
            "pos_pts_xy": [(x, y) for x, y, _ in pts],
        }
    ]


def cloud_rows(y, *, c_pi=1.0, pl=1.0, include_c_pi=True):
    rows = []
    for idx in range(21):
        x = idx * 0.5
        row = {
            "stamp": 9.0,
            "frame_id": "map",
            "x": x,
            "y": y,
            "z": 1.0,
            "pl": pl,
            "hpl": pl,
            "vpl": pl,
            "valid": 1,
            "unknown": 0,
            "stale": 0,
            "source_flags": 0,
            "generation_id": 4,
        }
        row["c_pi"] = c_pi if include_c_pi else ""
        rows.append(row)
    return rows


def accepted_profile_rows(
    y,
    *,
    c_pi=1.0,
    valid_count=200,
    c_pi_values=None,
    profile_seq=3,
    applied_to_objective=1,
    metrics_only=0,
):
    values = list(c_pi_values or [])
    rows = []
    for idx in range(200):
        x = 10.0 * idx / 199.0
        valid = idx < valid_count
        value = values[idx] if idx < len(values) else c_pi
        rows.append(
            {
                "profile_seq": profile_seq,
                "stamp": 10.0,
                "trajectory_id": 1,
                "planning_attempt_id": 11,
                "candidate_id": 7,
                "applied_to_objective": applied_to_objective,
                "metrics_only": metrics_only,
                "lambda_integrity": 0.00001,
                "snapshot_generation_id": 4,
                "query_base_time_s": 9.0,
                "sample_index": idx,
                "arc_fraction": idx / 199.0,
                "t_s": 3.0 * idx / 199.0,
                "x": x,
                "y": y,
                "z": 1.0,
                "hit": 1 if valid else 0,
                "valid": 1 if valid else 0,
                "stale": 0,
                "c_pi": value if valid else "",
                "reason": "ok" if valid else "query_miss",
                "trace_available": 1,
                "grad_x": 1.0,
                "grad_y": 0.0,
                "grad_z": 0.0,
                "neg_grad_x": -1.0,
                "neg_grad_y": 0.0,
                "neg_grad_z": 0.0,
                "pre_x": x + 0.1,
                "pre_y": y,
                "pre_z": 1.0,
                "disp_x": -0.1,
                "disp_y": 0.0,
                "disp_z": 0.0,
                "grad_dot_displacement": -0.1,
                "delta_c_pi": -0.1,
                "objective_requested": 0 if metrics_only else 1,
                "objective_applied": applied_to_objective,
                "p1_fallback": 0 if applied_to_objective else 1,
                "fallback_reason": "none" if applied_to_objective else "metrics_only",
            }
        )
    return rows


def accepted_profile_context(rows):
    first = rows[0]
    matched = sum(int(row["hit"]) and int(row["valid"]) and not int(row["stale"]) for row in rows)
    return [{
        "profile_seq": first["profile_seq"], "trajectory_id": first["trajectory_id"],
        "planning_attempt_id": first["planning_attempt_id"], "candidate_id": first["candidate_id"],
        "planning_start_s": 9.9, "accepted_stamp_s": 10.0, "planning_duration_s": 0.1,
        "snapshot_generation_id": first["snapshot_generation_id"], "snapshot_stamp_s": 9.0,
        "query_base_time_s": first["query_base_time_s"],
        "snapshot_x_min": -1.0, "snapshot_x_max": 11.0,
        "snapshot_y_min": -1.0, "snapshot_y_max": 11.0,
        "snapshot_z_min": 0.0, "snapshot_z_max": 2.0,
        "snapshot_time_min_s": 9.0, "snapshot_time_max_s": 13.0,
        "trajectory_x_min": 0.0, "trajectory_x_max": 10.0,
        "trajectory_y_min": y_min(rows), "trajectory_y_max": y_max(rows),
        "trajectory_z_min": 1.0, "trajectory_z_max": 1.0,
        "trajectory_time_min_s": 0.0, "trajectory_time_max_s": 3.0,
        "expected_sample_count": 200, "matched_sample_count": matched,
        "match_ratio": matched / 200.0, "query_miss_count": 200 - matched,
        "stale_count": 0, "invalid_count": 0, "miss_reason_histogram": "",
        "snapshot_frame_id": "map", "trajectory_frame_id": "map",
        "spatial_in_bounds": 1, "temporal_in_horizon": 1,
        "frame_match": 1, "generation_match": 1, "query_time_match": 1,
        "fresh": 1, "coverage_ok": 1, "spatial_miss_count": 0,
        "temporal_miss_count": 0, "occupied_miss_count": 0,
        "stale_miss_count": 0, "invalid_miss_count": 200 - matched,
        "trajectory_start_stamp_s": 10.0,
        "objective_requested": first["objective_requested"],
        "objective_applied": first["objective_applied"],
        "p1_fallback": first["p1_fallback"],
        "fallback_reason": first["fallback_reason"],
    }]


def y_min(rows):
    return min(row["y"] for row in rows)


def y_max(rows):
    return max(row["y"] for row in rows)


def run_gate(
    *,
    manifest=None,
    validator=None,
    health=None,
    meta=None,
    p0_rows=None,
    debug_summary=None,
    p1_2_bspline=None,
    baseline_export=Path("/tmp/p1_1_export"),
    baseline_bag=Path("/tmp/p1_1_bag"),
    baseline_manifest_data=None,
    baseline_bspline=None,
    p1_2_cloud=None,
    baseline_cloud=None,
    p1_2_profile=None,
    baseline_profile=None,
    p1_2_profile_missing=False,
    baseline_profile_missing=False,
    risk_scene_alignment=None,
):
    manifest = p1_2_manifest() if manifest is None else manifest
    validator = {"passed": True} if validator is None else validator
    health = topic_health() if health is None else health
    meta = metadata() if meta is None else meta
    p0_rows = [p0_row(idx) for idx in range(5)] if p0_rows is None else p0_rows
    p0_summary = analyzer.summarize_p0_health(p0_rows)
    debug_summary = p1_debug_summary() if debug_summary is None else debug_summary
    p1_2_bspline = bspline_rows(0.0) if p1_2_bspline is None else p1_2_bspline
    baseline_manifest_data = (
        p1_1_reference_manifest()
        if baseline_manifest_data is None
        else baseline_manifest_data
    )
    baseline_bspline = bspline_rows(5.0) if baseline_bspline is None else baseline_bspline
    p1_2_cloud = cloud_rows(0.0, c_pi=1.0, pl=1.0) if p1_2_cloud is None else p1_2_cloud
    baseline_cloud = (
        cloud_rows(5.0, c_pi=5.0, pl=5.0)
        if baseline_cloud is None
        else baseline_cloud
    )
    if p1_2_profile is None and not p1_2_profile_missing:
        p1_2_profile = accepted_profile_rows(0.0, c_pi=1.0)
    if baseline_profile is None and not baseline_profile_missing:
        baseline_profile = accepted_profile_rows(5.0, c_pi=5.0, applied_to_objective=0, metrics_only=1)
    risk_comparison = analyzer.compare_p1_2_risk_profiles(
        p1_2_bspline,
        baseline_bspline,
        p1_2_cloud,
        baseline_cloud,
        p0_resolution_m=0.75,
        p1_2_accepted_profile_rows=None if p1_2_profile_missing else p1_2_profile,
        p1_1_accepted_profile_rows=None if baseline_profile_missing else baseline_profile,
        p1_2_context_rows=None if p1_2_profile_missing else accepted_profile_context(p1_2_profile),
        p1_1_context_rows=None if baseline_profile_missing else accepted_profile_context(baseline_profile),
        p1_2_context_info={"missing": p1_2_profile_missing, "path": "current"},
        p1_1_context_info={"missing": baseline_profile_missing, "path": "baseline"},
        stale_timeout_s=1.0,
    )
    risk_comparison["risk_scene_alignment"] = risk_scene_alignment or {
        "available": True, "reasons": [], "source_status": {},
    }
    failures = []
    inconclusive = []
    gates = analyzer.validate_p1_2_hard_gates(
        manifest,
        validator,
        health,
        meta,
        p0_summary,
        p0_rows,
        debug_summary,
        p1_2_bspline,
        "",
        baseline_export,
        baseline_bag,
        baseline_manifest_data,
        baseline_bspline,
        "",
        risk_comparison,
        failures,
        inconclusive,
        reference_p0_health_summary=analyzer.summarize_p0_health(p0_rows),
        reference_p0_health_rows=p0_rows,
    )
    return gates, failures, inconclusive, risk_comparison


class P1_2AnalyzerTest(unittest.TestCase):
    def test_p0_startup_boundary_excludes_only_pre_activation_rows(self):
        rows = [
            p0_row(index, stamp=float(index), ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable")
            for index in range(4)
        ] + [p0_row(4, stamp=4.0)]
        before_activation = analyzer.summarize_p0_startup_snapshot_unavailable(
            rows, activation_stamp_s=4.0)
        self.assertEqual(before_activation["pre_activation_rows"], 4)
        self.assertTrue(before_activation["startup_snapshot_unavailable_bounded"])
        self.assertEqual(before_activation["post_startup_ready_false_count"], 0)

        active_prefix = analyzer.summarize_p0_startup_snapshot_unavailable(
            rows, activation_stamp_s=0.0)
        self.assertFalse(active_prefix["startup_snapshot_unavailable_bounded"])

    def test_p0_startup_accepts_rate_independent_not_ready_prefix(self):
        rows = [
            p0_row(0, stamp=0.0, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0, reason="not_ready"),
            p0_row(1, stamp=0.0, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
            p0_row(2, stamp=0.5, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
            p0_row(3, stamp=0.5, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
            p0_row(4, stamp=1.0, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
            p0_row(5, stamp=1.0, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
            p0_row(6, stamp=1.5, ready=False, stale=True,
                   valid_ratio=0.0, unknown_ratio=1.0,
                   reason="snapshot_unavailable"),
        ] + [p0_row(index, stamp=1.75 + 0.25 * (index - 7))
             for index in range(7, 107)]
        summary = analyzer.summarize_p0_startup_snapshot_unavailable(rows)
        self.assertEqual(summary["startup_snapshot_unavailable_rows"], 7)
        self.assertTrue(summary["startup_snapshot_unavailable_bounded"])
        self.assertEqual(summary["post_startup_ready_false_count"], 0)
        self.assertEqual(summary["post_startup_stale_true_count"], 0)

        failures = []
        inconclusive = []
        analyzer.validate_p0_requirements(
            "P1-2",
            p1_2_manifest(),
            analyzer.summarize_p0_health(rows),
            analyzer.summarize_p0_cloud_rows(cloud_rows(0.0)),
            failures,
            inconclusive,
            allow_explainable_startup_unavailable=True,
        )
        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])

    def test_candidate_trace_reconciles_fixed_lattice_timeline_and_profile(self):
        profile = accepted_profile_rows(0.0)
        profile_summary = analyzer.summarize_p1_accepted_profile_rows(profile)
        rows = [candidate_optimization_row()]
        evidence = analyzer.validate_p1_candidate_optimization_evidence(
            rows, {"valid": True},
            [{"stage": "optimizer_start", "snapshot_generation_id": 4,
              "planning_attempt_id": 11, "candidate_id": 7}],
            profile_summary,
        )
        self.assertTrue(evidence["passed"])

    def test_candidate_trace_rejects_refined_accepted_profile_risk_regression(self):
        profile_summary = analyzer.summarize_p1_accepted_profile_rows(
            accepted_profile_rows(0.0, c_pi=5.5)
        )
        evidence = analyzer.validate_p1_candidate_optimization_evidence(
            [candidate_optimization_row()], {"valid": True},
            [{"stage": "optimizer_start", "snapshot_generation_id": 4,
              "planning_attempt_id": 11, "candidate_id": 7}],
            profile_summary,
        )
        self.assertFalse(evidence["accepted_profile_preference_preserved"])
        self.assertFalse(evidence["passed"])

    def test_candidate_trace_fails_closed_without_one_selected_success_or_full_support(self):
        profile_summary = analyzer.summarize_p1_accepted_profile_rows(
            accepted_profile_rows(0.0)
        )
        row = candidate_optimization_row(selected=0, post_support_valid_count=199,
                                         post_support_coverage=0.995,
                                         support_full_valid=0)
        evidence = analyzer.validate_p1_candidate_optimization_evidence(
            [row], {"valid": True}, [], profile_summary,
        )
        self.assertFalse(evidence["passed"])
        self.assertFalse(evidence["support_full_valid"])
        self.assertFalse(evidence["selected_success_per_attempt"])

    def test_candidate_trace_requires_rejected_replacement_lifecycle_and_artifact_binding(self):
        row = candidate_optimization_row(
            replacement_accepted=0,
            replacement_reason="p1_replacement_risk_regression",
        )
        timeline = [
            {"stage": "optimizer_start", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 7},
        ]
        evidence = analyzer.validate_p1_candidate_optimization_evidence(
            [row], {"valid": True}, timeline, None)
        self.assertFalse(evidence["replacement_rejections_reconciled"])
        self.assertFalse(evidence["passed"])
        timeline.append(
            {"stage": "replacement", "outcome": "rejected",
             "snapshot_generation_id": 4, "planning_attempt_id": 11,
             "candidate_id": 7})
        evidence = analyzer.validate_p1_candidate_optimization_evidence(
            [row], {"valid": True}, timeline, None)
        self.assertTrue(evidence["replacement_rejections_reconciled"])
        self.assertFalse(evidence["retained_artifacts_reconciled"])
        self.assertFalse(evidence["passed"])

    def test_candidate_trace_reconciles_retained_incumbent_without_counting_it_as_publish(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            accepted = accepted_profile_rows(0.0)
            for item in accepted:
                item.update({"snapshot_generation_id": 5, "planning_attempt_id": 12,
                             "candidate_id": 99})
            profile_summary = analyzer.summarize_p1_accepted_profile_rows(accepted)
            profile_summary["path"] = str(directory / "planner_p1_accepted_trajectory_risk_profile.csv")
            rejected = candidate_optimization_row(replacement_accepted=0,
                                                   replacement_reason="risk_regression",
                                                   incumbent_available=1)
            published = candidate_optimization_row(snapshot_generation_id=5,
                                                    planning_attempt_id=12, candidate_id=99)
            decision = directory / "planner_p1_replacement_decision.csv"
            decision.write_text(
                "snapshot_generation_id,planning_attempt_id,optimizer_selected_candidate_id,replacement_accepted,final_trajectory_source\n"
                "4,11,7,0,retained_incumbent\n")
            retained = directory / "planner_p1_candidate_retained_profile.csv"
            rows = []
            for role in ("optimizer_selected_candidate", "retained_incumbent"):
                rows.extend({"snapshot_generation_id": "4", "planning_attempt_id": "11",
                             "candidate_id": "7", "trajectory_role": role,
                             "final_trajectory_source": "retained_incumbent"}
                            for _ in range(200))
            with retained.open("w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=sorted(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            timeline = [
                {"stage": "optimizer_start", "snapshot_generation_id": 4,
                 "planning_attempt_id": 11, "candidate_id": 7},
                {"stage": "replacement", "outcome": "rejected",
                 "snapshot_generation_id": 4, "planning_attempt_id": 11,
                 "candidate_id": 7},
                {"stage": "optimizer_start", "snapshot_generation_id": 5,
                 "planning_attempt_id": 12, "candidate_id": 99},
            ]
            evidence = analyzer.validate_p1_candidate_optimization_evidence(
                [rejected, published],
                {"valid": True,
                 "path": str(directory / "planner_p1_candidate_optimization.csv")},
                timeline, profile_summary)
        self.assertTrue(evidence["retained_artifacts_reconciled"])
        self.assertTrue(evidence["accepted_profile_is_not_rejected_candidate"])
        self.assertTrue(evidence["passed"])

    def test_candidate_trace_closes_rejected_startup_without_fabricated_incumbent(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            rejected = candidate_optimization_row(
                replacement_accepted=0,
                replacement_reason="p1_refinement_self_risk_regression",
                incumbent_available=0)
            decision = directory / "planner_p1_replacement_decision.csv"
            decision.write_text(
                "snapshot_generation_id,planning_attempt_id,optimizer_selected_candidate_id,replacement_accepted,final_trajectory_source\n"
                "4,11,7,0,no_publish_no_incumbent\n")
            retained = directory / "planner_p1_candidate_retained_profile.csv"
            rows = [
                {"snapshot_generation_id": "4", "planning_attempt_id": "11",
                 "candidate_id": "7", "trajectory_role": "optimizer_selected_candidate",
                 "final_trajectory_source": "no_publish_no_incumbent"}
                for _ in range(200)
            ]
            with retained.open("w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=sorted(rows[0]))
                writer.writeheader(); writer.writerows(rows)
            timeline = [
                {"stage": "optimizer_start", "snapshot_generation_id": 4,
                 "planning_attempt_id": 11, "candidate_id": 7},
                {"stage": "replacement", "outcome": "rejected",
                 "snapshot_generation_id": 4, "planning_attempt_id": 11,
                 "candidate_id": 7},
            ]
            evidence = analyzer.validate_p1_candidate_optimization_evidence(
                [rejected],
                {"valid": True,
                 "path": str(directory / "planner_p1_candidate_optimization.csv")},
                timeline, None)
        self.assertTrue(evidence["retained_artifacts_reconciled"])
        self.assertTrue(evidence["passed"])

    def test_candidate_csv_requires_nonempty_termination_reason(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "candidate.csv"
            fieldnames = analyzer.P1_CANDIDATE_OPTIMIZATION_FIELDS
            row = candidate_optimization_row(termination_reason="")
            with path.open("w", newline="") as output:
                writer = csv.DictWriter(output, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(row)
            _, summary = analyzer.read_p1_candidate_optimization_csv(path)
        self.assertFalse(summary["valid"])
        error = next(item for item in summary["empty_identity_fields"]
                     if item["field"] == "termination_reason")
        self.assertEqual(error["row"], 0)
        self.assertEqual(error["planning_attempt_id"], "11")
        self.assertEqual(error["candidate_id"], "7")
        self.assertEqual(error["expected_type"], "nonempty string")

    def test_candidate_csv_provenance_strings_are_not_numeric_finite_fields(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "candidate.csv"
            with path.open("w", newline="") as output:
                writer = csv.DictWriter(
                    output, fieldnames=analyzer.P1_CANDIDATE_OPTIMIZATION_FIELDS)
                writer.writeheader()
                writer.writerow(candidate_optimization_row())
            _, summary = analyzer.read_p1_candidate_optimization_csv(path)
        self.assertTrue(summary["valid"], summary)
        self.assertEqual(summary["finite_errors"], [])

    def test_candidate_csv_nan_inf_and_empty_numeric_report_full_identity(self):
        for raw in ("nan", "inf", ""):
            with self.subTest(raw=raw), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / "candidate.csv"
                with path.open("w", newline="") as output:
                    writer = csv.DictWriter(
                        output, fieldnames=analyzer.P1_CANDIDATE_OPTIMIZATION_FIELDS)
                    writer.writeheader()
                    writer.writerow(candidate_optimization_row(post_total_objective=raw))
                _, summary = analyzer.read_p1_candidate_optimization_csv(path)
                self.assertFalse(summary["valid"])
                error = next(item for item in summary["finite_errors"]
                             if item["field"] == "post_total_objective")
                self.assertEqual(error["row"], 0)
                self.assertEqual(error["planning_attempt_id"], "11")
                self.assertEqual(error["candidate_id"], "7")
                self.assertEqual(error["raw_value"], raw)
                self.assertEqual(error["expected_type"], "finite numeric")

    def test_singleflight_rejects_duplicate_same_attempt_and_acquisition_records(self):
        timeline = [
            {"stage": "p1_admission", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "p1_admission", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "acquire", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "acquire", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "optimizer_start", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 7},
        ]
        summary = analyzer.summarize_p1_generation_singleflight(timeline)
        self.assertFalse(summary["passed"])
        self.assertEqual(summary["max_admission_per_generation"], 2)
        self.assertEqual(summary["max_acquisition_per_generation"], 2)

    def test_singleflight_counts_pending_and_final_admission_as_one_decision(self):
        timeline = [
            {"stage": "acquire", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "p1_admission_pending", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "p1_admission", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 1},
            {"stage": "optimizer_start", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 1},
        ]
        summary = analyzer.summarize_p1_generation_singleflight(timeline)
        self.assertTrue(summary["passed"], summary)
        self.assertEqual(summary["max_admission_per_generation"], 1)

    def test_singleflight_allows_admission_rejection_without_acquisition(self):
        timeline = [
            {"stage": "p1_admission", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
            {"stage": "p1_admission", "snapshot_generation_id": 5,
             "planning_attempt_id": 12, "candidate_id": 0},
            {"stage": "acquire", "snapshot_generation_id": 5,
             "planning_attempt_id": 12, "candidate_id": 0},
            {"stage": "optimizer_start", "snapshot_generation_id": 5,
             "planning_attempt_id": 12, "candidate_id": 1},
        ]

        summary = analyzer.summarize_p1_generation_singleflight(timeline)

        self.assertTrue(summary["passed"], summary)
        self.assertEqual(summary["generation_counts"]["4"]["acquire"], 0)

    def test_singleflight_allows_context_acquisition_before_close_to_goal(self):
        summary = analyzer.summarize_p1_generation_singleflight([
            {"stage": "acquire", "snapshot_generation_id": 4,
             "planning_attempt_id": 11, "candidate_id": 0},
        ])

        self.assertTrue(summary["passed"], summary)
        self.assertEqual(summary["generation_counts"]["4"]["p1_admission"], 0)

    def test_terminal_common_arc_comparison_excludes_unshared_history(self):
        current = accepted_profile_rows(0.0, c_pi=4.0)
        for index, row in enumerate(current):
            row["x"] = 8.0 + 2.0 * index / 199.0
        reference_values = [0.1] * 150 + [5.0] * 50
        reference = accepted_profile_rows(
            5.0,
            c_pi_values=reference_values,
            applied_to_objective=0,
            metrics_only=1,
        )

        comparison = analyzer.compare_p1_2_risk_profiles(
            bspline_rows(0.0), bspline_rows(5.0),
            cloud_rows(0.0), cloud_rows(5.0),
            p0_resolution_m=0.75,
            p1_2_accepted_profile_rows=current,
            p1_1_accepted_profile_rows=reference,
        )

        self.assertEqual(comparison["comparison_mode"], "terminal_common_arc_window")
        self.assertLess(comparison["current_mean"], comparison["reference_mean"])
        self.assertLess(comparison["current_max"], comparison["reference_max"])
        self.assertTrue(comparison["risk_reduced"], comparison)

    def test_metrics_only_reference_does_not_require_displacement_descent_trace(self):
        reference = accepted_profile_rows(
            5.0, c_pi=5.0, applied_to_objective=0, metrics_only=1)
        for row in reference[31:42]:
            for field in (
                "disp_x", "disp_y", "disp_z",
                "grad_dot_displacement", "delta_c_pi",
            ):
                row[field] = ""

        gates, failures, inconclusive, _ = run_gate(
            baseline_profile=reference,
            baseline_cloud=cloud_rows(100.0, c_pi=5.0, pl=5.0),
        )

        self.assertFalse(gates["p1_1_gradient_trace_complete"])
        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])

    def test_happy_path_passes_gate(self):
        gates, failures, inconclusive, risk = run_gate(
            baseline_cloud=cloud_rows(100.0, c_pi=5.0, pl=5.0)
        )

        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["p1_applied_to_objective"])
        self.assertTrue(gates["risk_profile_reduced"])
        self.assertEqual(risk["comparison_metric"], "c_pi")
        self.assertEqual(
            risk["risk_source"],
            "accepted_trajectory_profile_csv_terminal_common_arc",
        )

    def test_manifest_rejects_metrics_only_wrong_lambda_disabled_p1_and_enabled_later_phases(self):
        bad_manifests = [
            p1_2_manifest(planner_safety_profile="off"),
            p1_2_manifest(**{"p0.enable_risk_grid": False}),
            p1_2_manifest(**{"p1.metrics_only": True}),
            p1_2_manifest(**{"p1.use_integrity_cost": False}),
            p1_2_manifest(**{"p1.lambda_integrity": 0.0}),
            p1_2_manifest(**{"planner_enable_p1": False}),
            p1_2_manifest(**{"planner_enable_p2": True}),
            p1_2_manifest(**{"planner_enable_p3_local": True}),
            p1_2_manifest(**{"planner_enable_p3_global": True}),
            p1_2_manifest(**{"planner_enable_p4": True}),
            p1_2_manifest(**{"planner_enable_p5_runtime": True}),
            p1_2_manifest(**{"planner_enable_p5_final": True}),
        ]

        for manifest in bad_manifests:
            with self.subTest(manifest=manifest):
                _, failures, _, _ = run_gate(manifest=manifest)
                self.assertTrue(any("manifest" in item for item in failures), failures)

    def test_missing_and_wrong_p1_1_reference_fail(self):
        _, missing_failures, _, _ = run_gate(
            baseline_export=None,
            baseline_bag=None,
            baseline_manifest_data={},
            baseline_bspline=[],
            baseline_cloud=[],
        )
        self.assertTrue(any("P1-1 reference" in item for item in missing_failures), missing_failures)

        _, wrong_failures, _, _ = run_gate(
            baseline_manifest_data=p1_1_reference_manifest(
                scenario="other",
                **{"p1.metrics_only": False},
            )
        )
        self.assertTrue(any("reference manifest" in item for item in wrong_failures), wrong_failures)

    def test_missing_and_empty_csv_fail(self):
        _, missing_failures, _, _ = run_gate(debug_summary=p1_debug_summary([], missing=True))
        _, empty_failures, _, _ = run_gate(debug_summary=p1_debug_summary([]))

        self.assertTrue(any("missing" in item for item in missing_failures), missing_failures)
        self.assertTrue(any("no data rows" in item for item in empty_failures), empty_failures)

    def test_no_applied_to_objective_true_fails(self):
        debug_summary = p1_debug_summary([p1_debug_row(applied_to_objective=0)])

        _, failures, _, _ = run_gate(debug_summary=debug_summary)

        self.assertTrue(any("applied_to_objective=true" in item for item in failures), failures)

    def test_nonfinite_cost_or_gradient_fails(self):
        debug_summary = p1_debug_summary([p1_debug_row(grad_norm_integrity="nan")])

        _, failures, _, _ = run_gate(debug_summary=debug_summary)

        self.assertTrue(any("non-finite" in item for item in failures), failures)

    def test_p0_health_failure_fails(self):
        bad_rows = [
            ([p0_row(0), p0_row(1, ready=False)], "ready=false"),
            ([p0_row(0), p0_row(1, stale=True)], "stale=true"),
            ([p0_row(0), p0_row(1, valid_ratio=0.0, unknown_ratio=1.0)], "full unknown"),
        ]

        for rows, expected in bad_rows:
            with self.subTest(expected=expected):
                _, failures, _, _ = run_gate(p0_rows=rows)
                self.assertTrue(any(expected in item for item in failures), failures)

    def test_validator_failure_fails(self):
        _, failures, _, _ = run_gate(validator={"passed": False})

        self.assertTrue(any("validator" in item for item in failures), failures)

    def test_p1_rviz_missing_fails(self):
        health = topic_health(
            **{analyzer.P1_SAMPLES_TOPIC: {"status": "FAIL", "count": 0}}
        )

        _, failures, _, _ = run_gate(health=health)

        self.assertTrue(any("RViz" in item for item in failures), failures)

    def test_bspline_missing_fails(self):
        _, failures, _, _ = run_gate(p1_2_bspline=[])

        self.assertTrue(any("bspline" in item for item in failures), failures)

    def test_xy_only_bspline_does_not_satisfy_p1_2_xyz_risk_path(self):
        xy_only = [
            {
                "bag_time_s": 1.0,
                "traj_id": 1,
                "pos_pts_count": 3,
                "pos_pts_xy": [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
                "pos_pts_xyz": [],
            }
        ]

        gates, failures, _, risk = run_gate(p1_2_bspline=xy_only)

        self.assertFalse(gates["p1_2_nonempty_bspline_path_present"])
        self.assertEqual(risk["p1_2_path_xyz"], [])
        self.assertTrue(
            any("trajectory stability" in item for item in failures),
            failures,
        )

    def test_risk_profile_not_reduced_fails(self):
        _, failures, _, risk = run_gate(
            p1_2_profile=accepted_profile_rows(0.0, c_pi=6.0),
            baseline_profile=accepted_profile_rows(5.0, c_pi=5.0),
        )

        self.assertFalse(risk["risk_reduced"])
        self.assertTrue(any("risk profile" in item for item in failures), failures)

    def test_risk_profile_max_not_lower_fails_even_when_mean_is_lower(self):
        current_values = [1.0] * 199 + [10.0]
        _, failures, _, risk = run_gate(
            p1_2_profile=accepted_profile_rows(0.0, c_pi_values=current_values),
            baseline_profile=accepted_profile_rows(5.0, c_pi=5.0),
        )

        self.assertLess(risk["current_mean"], risk["reference_mean"])
        self.assertGreaterEqual(risk["current_max"], risk["reference_max"])
        self.assertFalse(risk["risk_reduced"])
        self.assertTrue(any("risk profile" in item for item in failures), failures)

    def test_risk_profile_without_c_pi_or_pl_fails(self):
        bad_current = [{**row, "c_pi": ""} for row in accepted_profile_rows(0.0)]
        bad_baseline = [{**row, "c_pi": ""} for row in accepted_profile_rows(5.0)]

        _, failures, _, risk = run_gate(
            p1_2_profile=bad_current,
            baseline_profile=bad_baseline,
        )

        self.assertIsNone(risk["comparison_metric"])
        self.assertTrue(any("risk metric" in item for item in failures), failures)

    def test_missing_profile_csv_fails(self):
        gates, failures, _, risk = run_gate(p1_2_profile_missing=True)

        self.assertTrue(gates["accepted_profile_missing"])
        self.assertTrue(risk["accepted_profile_missing"])
        self.assertTrue(any("accepted trajectory risk profile CSV missing" in item for item in failures), failures)

    def test_risk_profile_match_coverage_failure_fails(self):
        _, failures, _, risk = run_gate(
            baseline_profile=accepted_profile_rows(5.0, c_pi=5.0, valid_count=10),
        )

        self.assertFalse(risk["p1_1_match_ok"])
        self.assertTrue(
            any("accepted trajectory risk profile coverage" in item for item in failures),
            failures,
        )

    def test_trajectory_oscillation_gate_fails(self):
        _, failures, _, risk = run_gate(p1_2_bspline=bspline_rows(0.0, zigzag=True))

        self.assertFalse(risk["trajectory_stability"]["passed"])
        self.assertTrue(any("trajectory stability" in item for item in failures), failures)

    def test_p5_leakage_fails(self):
        _, failures, _, _ = run_gate(meta=metadata(**{analyzer.P5_GATE_STATUS_TOPIC: 2}))

        self.assertTrue(any("P5 leakage" in item for item in failures), failures)

    def test_final_profile_never_falls_back_to_earlier_complete_profile(self):
        good = accepted_profile_rows(0.0, profile_seq=2)
        bad = accepted_profile_rows(0.0, profile_seq=3, valid_count=10)
        gates, failures, _, risk = run_gate(p1_2_profile=good + bad)

        self.assertEqual(risk["p1_2_profile"]["selected_profile_seq"], 3.0)
        self.assertFalse(gates["p1_2_risk_match_ok"])
        self.assertTrue(any("coverage is insufficient" in item for item in failures), failures)

    def test_duplicate_or_missing_final_sample_index_fails_closed(self):
        profile = accepted_profile_rows(0.0)
        profile[-1]["sample_index"] = 0
        gates, failures, _, _ = run_gate(p1_2_profile=profile)

        self.assertFalse(gates["p1_2_accepted_profile_format_ok"])
        self.assertTrue(any("unique sample_index" in item for item in failures), failures)

    def test_mixed_final_candidate_tuple_fails_closed(self):
        profile = accepted_profile_rows(0.0)
        profile[-1]["candidate_id"] = 99
        gates, failures, _, _ = run_gate(p1_2_profile=profile)

        self.assertFalse(gates["p1_2_accepted_profile_format_ok"])
        self.assertTrue(any("one metadata tuple" in item for item in failures), failures)

    def test_final_profile_requires_matching_objective_debug_tuple(self):
        debug_rows = [p1_debug_row(candidate_id=99)]
        gates, failures, _, _ = run_gate(
            debug_summary=p1_debug_summary(debug_rows)
        )

        self.assertFalse(gates["p1_final_profile_context_in_debug"])
        self.assertTrue(any("matching objective-applied debug" in item for item in failures), failures)

    def test_required_topic_failure_blocks_nested_gate(self):
        health = topic_health(**{analyzer.P0_HEALTH_LEGACY_TOPIC: {"status": "FAIL", "count": 3}})
        gates, failures, _, _ = run_gate(health=health)

        self.assertFalse(gates["required_topics_passed"])
        self.assertFalse(gates["passed"])
        self.assertTrue(any("required topics" in item for item in failures), failures)

    def test_stale_or_out_of_bounds_context_fails_closed(self):
        profile = accepted_profile_rows(0.0)
        context = accepted_profile_context(profile)
        context[0]["snapshot_stamp_s"] = 8.0
        risk = analyzer.compare_p1_2_risk_profiles(
            bspline_rows(0.0), bspline_rows(5.0), cloud_rows(0.0), cloud_rows(5.0),
            p0_resolution_m=0.75, p1_2_accepted_profile_rows=profile,
            p1_1_accepted_profile_rows=accepted_profile_rows(5.0, applied_to_objective=0, metrics_only=1),
            p1_2_context_rows=context,
            p1_1_context_rows=accepted_profile_context(accepted_profile_rows(5.0, applied_to_objective=0, metrics_only=1)),
            p1_2_context_info={"path": "current"}, p1_1_context_info={"path": "reference"},
            stale_timeout_s=1.0,
        )
        self.assertFalse(risk["p1_2_context"]["valid"])
        self.assertFalse(risk["p1_2_context"]["fresh"])

    def test_next_branch_is_exact_for_p1_2(self):
        self.assertEqual(
            analyzer.next_debug_branch("PASS", [], [], "P1-2"),
            "PASS -> P1-3",
        )
        self.assertEqual(
            analyzer.next_debug_branch("FAIL", ["x"], [], "P1-2"),
            "FAIL -> artifact provenance/runtime install/recording completeness debug",
        )
        self.assertEqual(
            analyzer.next_debug_branch("INCONCLUSIVE", [], ["x"], "P1-2"),
            "FAIL -> artifact provenance/runtime install/recording completeness debug",
        )
        self.assertEqual(
            analyzer.next_debug_branch("FAIL", ["weighted_f_integrity_max is not positive"], [], "P1-2"),
            "FAIL -> lambda/gradient debug",
        )

    def test_required_figure_filenames_are_exact(self):
        self.assertEqual(
            analyzer.P1_2_FIGURE_FILENAMES,
            [
                "p1_2_artifact_provenance.png",
                "p1_2_scenario_topdown.png",
                "p1_2_risk_trajectory_scene_overlay.png",
                "p1_2_topic_activity_timeline.png",
                "p1_2_p0_health_and_context_freshness.png",
                "p1_2_snapshot_candidate_binding.png",
                "p1_2_objective_gradient_timeline.png",
                "p1_2_risk_profile_vs_p1_1.png",
                "p1_2_accepted_profile_coverage_overlay.png",
                "p1_2_trajectory_overlay_vs_p1_1.png",
                "p1_2_result_dashboard.png",
                "p1_2_artifact_completeness.png",
                "p1_2_cause_exclusion_summary.png",
                "p1_2_snapshot_spatial_bounds_overlay.png",
                "p1_2_gradient_direction_field_overlay.png",
                "p1_2_refresh_latency_vs_stale_threshold.png",
                "p1_2_replan_load_correlation.png",
                "p1_2_timebase_alignment.png",
                "p1_2_temporal_horizon_admission.png",
                "p1_2_generation_singleflight_timeline.png",
                "p1_2_candidate_optimization_funnel.png",
                "p1_2_objective_contribution_ratio.png",
                "p1_2_gradient_displacement_alignment.png",
            ],
        )

    def test_supplementary_figure_filenames_retain_diagnostics(self):
        self.assertEqual(
            analyzer.P1_2_OPTIONAL_FIGURE_FILENAMES,
            [
                "p1_2_p0_callback_timeline.png",
                "p1_2_planning_context_age_timeline.png",
                "p1_2_replan_fallback_timeline.png",
                "p1_2_integrity_cost_debug_summary.png",
                "p1_2_bspline_publish_timeline.png",
                "p1_2_manifest_switch_summary.png",
                "p1_2_validation_summary.png",
            ],
        )

    def test_required_figure_check_fails_closed_for_missing_or_empty_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            present = root / "present.png"
            empty = root / "empty.png"
            missing = root / "missing.png"
            present.write_bytes(b"png")
            empty.touch()
            self.assertEqual(
                analyzer.missing_required_figure_paths([present, empty, missing]),
                [empty, missing],
            )

    def test_binding_and_objective_gradient_figures_render_from_both_runs(self):
        current = accepted_profile_rows(0.0)
        reference = accepted_profile_rows(5.0, applied_to_objective=0, metrics_only=1)
        current_summary = analyzer.summarize_p1_accepted_profile_rows(current)
        reference_summary = analyzer.summarize_p1_accepted_profile_rows(reference)
        current_context = analyzer.validate_p1_profile_context(
            current_summary, accepted_profile_context(current), {"path": "current"}
        )
        reference_context = analyzer.validate_p1_profile_context(
            reference_summary, accepted_profile_context(reference), {"path": "reference"}
        )
        current_debug = p1_debug_summary([p1_debug_row()])
        reference_debug = p1_debug_summary([p1_debug_row(applied_to_objective=0)])
        self.assertEqual(current_debug["context_tuples"], [["11", "7", "4", "9.0"]])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binding = root / "binding.png"
            objective = root / "objective.png"
            self.assertTrue(analyzer.plot_p1_2_snapshot_candidate_binding(
                current_summary, reference_summary, current_context, reference_context,
                current_debug, reference_debug,
                {"p1_final_profile_context_in_debug": True}, binding,
            ))
            self.assertTrue(analyzer.plot_p1_2_objective_gradient_timeline(
                [p1_debug_row()], [p1_debug_row(applied_to_objective=0)],
                current_summary, reference_summary, objective,
            ))
            self.assertGreater(binding.stat().st_size, 0)
            self.assertGreater(objective.stat().st_size, 0)

    def test_risk_scene_alignment_fails_closed_and_unavailable_plot_is_nonempty(self):
        comparison = analyzer.compare_p1_2_risk_profiles(
            bspline_rows(0.0), bspline_rows(5.0), cloud_rows(0.0), cloud_rows(5.0),
            p0_resolution_m=0.75,
            p1_2_accepted_profile_rows=accepted_profile_rows(0.0),
            p1_1_accepted_profile_rows=accepted_profile_rows(
                5.0, applied_to_objective=0, metrics_only=1
            ),
        )
        current_scene = {
            "map_points": [(0.0, 0.0, 0.0)],
            "truth_xy": [(0.0, 0.0), (10.0, 0.0)],
            "slam_xy": [(0.0, 0.0), (10.0, 0.0)],
        }
        reference_scene = {
            "map_points": [(0.0, 5.0, 0.0)],
            "truth_xy": [(0.0, 5.0), (10.0, 5.0)],
            "slam_xy": [],
        }

        alignment = analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene
        )

        self.assertFalse(alignment["available"])
        self.assertTrue(any("P1-1 bag odom" in item for item in alignment["reasons"]))
        with tempfile.TemporaryDirectory() as tmp:
            figure = Path(tmp) / "unavailable.png"
            self.assertTrue(analyzer.plot_p1_2_risk_trajectory_scene_overlay(
                comparison, current_scene, reference_scene, alignment, figure
            ))
            self.assertGreater(figure.stat().st_size, 0)
        gates, failures, _, _ = run_gate(risk_scene_alignment=alignment)
        self.assertFalse(gates["risk_scene_overlay_available"])
        self.assertTrue(any("cannot be aligned" in item for item in failures), failures)

    def test_new_health_freshness_scene_and_dashboard_plots_are_nonempty(self):
        comparison = analyzer.compare_p1_2_risk_profiles(
            bspline_rows(0.0), bspline_rows(5.0), cloud_rows(0.0), cloud_rows(5.0),
            p0_resolution_m=0.75,
            p1_2_accepted_profile_rows=accepted_profile_rows(0.0),
            p1_1_accepted_profile_rows=accepted_profile_rows(
                5.0, applied_to_objective=0, metrics_only=1
            ),
            p1_2_context_rows=accepted_profile_context(accepted_profile_rows(0.0)),
            p1_1_context_rows=accepted_profile_context(accepted_profile_rows(
                5.0, applied_to_objective=0, metrics_only=1
            )),
            p1_2_context_info={"path": "current"},
            p1_1_context_info={"path": "reference"},
            stale_timeout_s=1.0,
        )
        health_context = analyzer.p1_health_snapshot_context([p0_row(0)])
        comparison["p1_2_health_context"] = health_context
        comparison["p1_1_health_context"] = health_context
        current_scene = {
            "map_points": [(float(x), 0.0, 0.0) for x in range(11)],
            "map_frame_ids": ["map"], "map_stamps": [9.0],
            "truth_xy": [(0.0, 0.0), (10.0, 0.0)],
            "truth_frame_ids": ["map"], "truth_stamps": [9.0, 10.0],
            "slam_xy": [(0.0, 0.0), (10.0, 0.0)],
            "slam_frame_ids": ["map"], "slam_stamps": [9.0, 10.0],
            "bspline_start_stamps": [10.0],
        }
        reference_scene = {
            "map_points": [(float(x), 5.0, 0.0) for x in range(11)],
            "map_frame_ids": ["map"], "map_stamps": [9.0],
            "truth_xy": [(0.0, 5.0), (10.0, 5.0)],
            "truth_frame_ids": ["map"], "truth_stamps": [9.0, 10.0],
            "slam_xy": [(0.0, 5.0), (10.0, 5.0)],
            "slam_frame_ids": ["map"], "slam_stamps": [9.0, 10.0],
            "bspline_start_stamps": [10.0],
        }
        alignment = analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene
        )
        self.assertTrue(alignment["available"], alignment)
        reference_scene["slam_xy"] = [(-120.0, -60.0), (-110.0, -55.0)]
        divergent_odom_alignment = analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene
        )
        self.assertTrue(divergent_odom_alignment["available"], divergent_odom_alignment)
        self.assertFalse(
            divergent_odom_alignment["spatial_overlap"]["P1-1 bag odom"])
        reference_scene["slam_xy"] = [(0.0, 5.0), (10.0, 5.0)]
        comparison["p1_2_health_context"]["snapshots"][0]["generation_id"] = 99
        self.assertFalse(analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene)["available"])
        comparison["p1_2_health_context"]["snapshots"][0]["generation_id"] = 4
        current_scene["bspline_start_stamps"] = [10.1]
        self.assertFalse(analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene)["available"])
        current_scene["bspline_start_stamps"] = [10.0]
        current_scene["slam_stamps"] = [10.101]
        self.assertFalse(analyzer.p1_2_risk_scene_alignment(
            comparison, current_scene, reference_scene)["available"])
        current_scene["slam_stamps"] = [10.0]
        gates, failures, inconclusive, _ = run_gate()
        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])
        gate_rows = analyzer.p1_2_hard_gate_rows(gates)
        self.assertTrue(gate_rows)
        self.assertTrue(all({"gate", "passed", "governing_values"} <= set(row) for row in gate_rows))
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            paths = [root / name for name in ("scene.png", "health.png", "dashboard.png")]
            self.assertTrue(analyzer.plot_p1_2_risk_trajectory_scene_overlay(
                comparison, current_scene, reference_scene, alignment, paths[0]
            ))
            self.assertTrue(analyzer.plot_p1_2_p0_health_and_context_freshness(
                [p0_row(idx) for idx in range(5)],
                [p0_row(idx) for idx in range(5)],
                comparison.get("p1_2_context", {}),
                comparison.get("p1_1_context", {}),
                stale_timeout_s=1.0,
                path=paths[1],
            ))
            self.assertTrue(analyzer.plot_p1_2_result_dashboard(gates, paths[2]))
            self.assertTrue(all(path.stat().st_size > 0 for path in paths))

    def test_artifact_completeness_figure_shows_missing_inputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            current_export = root / "p1_2_export"
            current_bag = root / "p1_2_bag"
            reference_export = root / "p1_1_export"
            reference_bag = root / "p1_1_bag"
            for directory in (current_export, current_bag, reference_export, reference_bag):
                directory.mkdir()
            for export in (current_export, reference_export):
                for name in (
                    "test_planner_manifest.json", "test_planner_validation_summary.json",
                    analyzer.P1_ACCEPTED_PROFILE_CSV_NAME,
                    analyzer.P1_ACCEPTED_PROFILE_CONTEXT_CSV_NAME,
                    analyzer.P1_1_DEBUG_CSV_NAME,
                ):
                    (export / name).write_text("evidence")
            for bag in (current_bag, reference_bag):
                (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
            figure = root / "artifact_completeness.png"
            self.assertTrue(analyzer.plot_p1_2_artifact_completeness(
                current_export, current_bag, reference_export, reference_bag,
                {}, {}, [figure], figure,
            ))
            self.assertGreater(figure.stat().st_size, 0)
            (reference_export / analyzer.P1_1_DEBUG_CSV_NAME).unlink()
            rows = analyzer.p1_2_artifact_completeness_rows(
                current_export, current_bag, reference_export, reference_bag,
                {}, {}, [figure], figure,
            )
            self.assertIn(("P1-1", "P1 debug CSV", False), rows)

    def test_sidecar_trajectory_mismatch_fails_profile_context_validation(self):
        profile = accepted_profile_rows(0.0)
        summary = analyzer.summarize_p1_accepted_profile_rows(profile)
        context = accepted_profile_context(profile)
        context[0]["trajectory_id"] = 999

        result = analyzer.validate_p1_profile_context(
            summary, context, {"path": "context"}
        )

        self.assertFalse(result["bindings_ok"])
        self.assertFalse(result["valid"])

    def test_profile_context_separates_spatial_bounds_from_time_horizon(self):
        profile = accepted_profile_rows(0.0)
        for row in profile:
            row["t_s"] = 5.7 * row["sample_index"] / 199.0
        summary = analyzer.summarize_p1_accepted_profile_rows(profile)
        context = accepted_profile_context(profile)
        context[0]["trajectory_time_max_s"] = 5.7
        context[0]["snapshot_time_max_s"] = 11.0

        result = analyzer.validate_p1_profile_context(
            summary, context, {"path": "context"}, stale_timeout_s=1.0
        )

        self.assertTrue(result["spatial_in_bounds"])
        self.assertFalse(result["temporal_in_horizon"])
        self.assertFalse(result["valid"])
        self.assertIn("temporal_out_of_horizon", result["reasons"])

    def test_p1_2_required_figure_contract_has_twenty_three_nonempty_names(self):
        self.assertEqual(len(analyzer.P1_2_FIGURE_FILENAMES), 23)
        self.assertEqual(len(set(analyzer.P1_2_FIGURE_FILENAMES)), 23)
        self.assertIn("p1_2_artifact_provenance.png", analyzer.P1_2_FIGURE_FILENAMES)
        self.assertIn(
            "p1_2_snapshot_spatial_bounds_overlay.png",
            analyzer.P1_2_FIGURE_FILENAMES,
        )
        self.assertIn(
            "p1_2_gradient_direction_field_overlay.png",
            analyzer.P1_2_FIGURE_FILENAMES,
        )
        self.assertIn(
            "p1_2_refresh_latency_vs_stale_threshold.png",
            analyzer.P1_2_FIGURE_FILENAMES,
        )
        self.assertIn(
            "p1_2_replan_load_correlation.png",
            analyzer.P1_2_FIGURE_FILENAMES,
        )

    def test_gradient_trace_schema_and_data_fail_closed(self):
        rows = accepted_profile_rows(0.0)
        summary = analyzer.summarize_p1_accepted_profile_rows(rows)
        self.assertTrue(summary["gradient_trace_complete"])
        rows[10]["trace_available"] = 0
        summary = analyzer.summarize_p1_accepted_profile_rows(rows)
        self.assertFalse(summary["gradient_trace_complete"])

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "profile.csv"
            legacy_fields = [field for field in analyzer.P1_ACCEPTED_PROFILE_FIELDS
                             if not field.startswith("grad_") and field != "trace_available"]
            path.write_text(",".join(legacy_fields) + "\n")
            _, parsed = analyzer.read_p1_accepted_profile_csv(path)
            self.assertIn("trace_available", parsed["missing_fields"])
            self.assertIn("grad_x", parsed["missing_fields"])

    def test_hard_gate_rows_are_structured_and_preserve_long_failure_reason(self):
        gates, _, _, _ = run_gate()
        long_reason = "frame mismatch: " + "x" * 500
        gates["risk_scene_overlay_available"] = False
        gates["risk_scene_alignment_reasons"] = [long_reason]
        rows = analyzer.p1_2_hard_gate_rows(gates)
        scene = next(row for row in rows if row["id"] == "recorded_risk_scene_alignment")
        self.assertEqual(
            set(("id", "label", "verdict", "governing_values", "failure_reason"))
            <= set(scene), True,
        )
        self.assertEqual(scene["verdict"], "FAIL")
        self.assertIn(long_reason, scene["failure_reason"])

    def test_four_new_required_diagnostic_plots_are_nonempty(self):
        comparison = analyzer.compare_p1_2_risk_profiles(
            bspline_rows(0.0), bspline_rows(5.0), cloud_rows(0.0), cloud_rows(5.0),
            p0_resolution_m=0.75,
            p1_2_accepted_profile_rows=accepted_profile_rows(0.0),
            p1_1_accepted_profile_rows=accepted_profile_rows(
                5.0, applied_to_objective=0, metrics_only=1),
            p1_2_context_rows=accepted_profile_context(accepted_profile_rows(0.0)),
            p1_1_context_rows=accepted_profile_context(accepted_profile_rows(
                5.0, applied_to_objective=0, metrics_only=1)),
            p1_2_context_info={"path": "current"},
            p1_1_context_info={"path": "reference"},
            stale_timeout_s=1.0,
        )
        timeline = [
            {"stamp_s": 1.0, "stage": "optimizer_start", "outcome": "started", "reason": "ok"},
            {"stamp_s": 1.1, "stage": "accept", "outcome": "rejected", "reason": "context_stale"},
            {"stamp_s": 1.2, "stage": "retry", "outcome": "deferred", "reason": "retry_deferred_same_generation"},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / f"plot_{idx}.png" for idx in range(4)]
            self.assertTrue(analyzer.plot_p1_2_snapshot_spatial_bounds_overlay(comparison, paths[0]))
            self.assertTrue(analyzer.plot_p1_2_gradient_direction_field_overlay(comparison, paths[1]))
            self.assertTrue(analyzer.plot_p1_2_refresh_latency_vs_stale_threshold(
                [p0_row(idx) for idx in range(5)], 1.0, paths[2]))
            self.assertTrue(analyzer.plot_p1_2_replan_load_correlation(
                timeline, [p0_row(idx) for idx in range(5)], paths[3]))
            self.assertTrue(all(path.stat().st_size > 0 for path in paths))

    def test_mixed_timebases_are_bounded_and_unmapped_fail_closed(self):
        planning_epoch = 1657065600.0
        health_epoch = 1784221650.0
        timeline = [
            {"stamp_s": planning_epoch + second, "stage": "acquire", "outcome": "started"}
            for second in range(91)
        ]
        health = [
            p0_row(
                second,
                stamp=health_epoch + second,
                health_callback_stamp_s=planning_epoch + second,
            )
            for second in range(91)
        ]
        # The legacy absolute-epoch implementation would create this many
        # one-second bins before plotting and was killed by the cgroup OOM.
        self.assertEqual(
            int(max(row["stamp"] for row in health)
                - min(row["stamp_s"] for row in timeline)) + 1,
            127156141,
        )
        manifest = {"run_duration_s": 90.0}
        alignment = analyzer.normalize_p1_2_timebases(timeline, health, manifest)
        self.assertFalse(alignment["causal_alignment_proven"])
        self.assertEqual(alignment["verdict"], "UNAVAILABLE")
        self.assertEqual(alignment["mapping_mode"], "independent_subplots")
        self.assertLessEqual(alignment["bin_count"], 96)
        self.assertLessEqual(alignment["estimated_bin_bytes"], 1024 * 1024)
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            figure = tmp_path / "p1_2_replan_load_correlation.png"
            summary = tmp_path / "safety_planner_analysis_summary.json"
            self.assertTrue(analyzer.plot_p1_2_replan_load_correlation(
                timeline, health, figure, timebase_alignment=alignment))
            analyzer.write_p1_2_provisional_summary(summary, alignment)
            self.assertGreater(figure.stat().st_size, 0)
            payload = json.loads(summary.read_text())
            self.assertFalse(payload["passed"])
            self.assertFalse(payload["analysis_complete"])
            self.assertIn("timebase", payload["failures"][0])

    def test_explicit_common_sim_clock_uses_one_shared_origin(self):
        planning_epoch = 1657065600.0
        receive_epoch = 1784221650.0
        timeline = [
            {"stamp_s": planning_epoch + second, "stage": "acquire"}
            for second in range(91)
        ]
        health = [
            p0_row(
                second,
                stamp=receive_epoch + second,
                health_callback_stamp_s=planning_epoch + second + 0.25,
            )
            for second in range(91)
        ]
        manifest = {
            "run_duration_s": 90.0,
            "timebase": {
                "planning_timeline": {"domain": "sim_message", "field": "stamp_s"},
                "p0_health_payload": {
                    "domain": "sim_message",
                    "field": "health_callback_stamp_s",
                },
                "bag_receive": {"domain": "system_receive", "field": "stamp"},
            },
        }
        alignment = analyzer.normalize_p1_2_timebases(timeline, health, manifest)
        self.assertTrue(alignment["causal_alignment_proven"])
        self.assertEqual(alignment["verdict"], "PASS")
        self.assertEqual(alignment["mapping_mode"], "explicit_common_clock")
        self.assertAlmostEqual(alignment["health_relative_s"][0], 0.25)
        self.assertAlmostEqual(alignment["timeline_relative_s"][-1], 90.0)
        self.assertLessEqual(alignment["bin_count"], 96)

    def test_timebase_normalization_keeps_rows_bound_after_invalid_startup_stamp(self):
        timeline = [
            {"stamp_s": "", "stage": "startup"},
            {"stamp_s": 100.0, "stage": "acquire"},
        ]
        health = [
            p0_row(0, stamp="", health_callback_stamp_s=""),
            p0_row(1, stamp=200.0, health_callback_stamp_s=100.25),
        ]
        manifest = {
            "run_duration_s": 90.0,
            "timebase": {
                "planning_timeline": {"domain": "sim", "field": "stamp_s"},
                "p0_health_payload": {
                    "domain": "sim", "field": "health_callback_stamp_s"
                },
            },
        }
        alignment = analyzer.normalize_p1_2_timebases(timeline, health, manifest)
        self.assertEqual(alignment["timeline_row_indices"], [1])
        self.assertEqual(alignment["health_row_indices"], [1])
        self.assertEqual(alignment["timeline_relative_s"], [0.0])
        self.assertEqual(alignment["health_relative_s"], [0.25])

    def test_required_png_completeness_is_a_final_hard_gate(self):
        self.assertEqual(len(analyzer.P1_2_FIGURE_FILENAMES), 23)
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / name for name in analyzer.P1_2_FIGURE_FILENAMES]
            for path in paths[:-1]:
                path.write_bytes(b"png")
            gates = {}
            failures = []
            analyzer.apply_p1_2_required_figure_gate(gates, paths, failures)
            self.assertFalse(gates["required_png_completeness"])
            self.assertTrue(any(paths[-1].name in failure for failure in failures))
            paths[-1].write_bytes(b"png")
            failures.clear()
            analyzer.apply_p1_2_required_figure_gate(gates, paths, failures)
            self.assertTrue(gates["required_png_completeness"])
            self.assertEqual(failures, [])

    def test_analyzer_exception_replaces_provisional_summary_with_fresh_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            old_summary = root / "metadata" / "safety_planner_analysis_summary.json"
            old_summary.parent.mkdir(parents=True)
            old_summary.write_text(json.dumps({"passed": True, "analysis_run_id": "old"}))
            original = analyzer.read_bag_metadata
            analyzer.read_bag_metadata = lambda _bag: (_ for _ in ()).throw(
                RuntimeError("fixture exact-cloud extraction failed")
            )
            try:
                result = analyzer.analyze(argparse.Namespace(
                    experiment_id="P1-2", export_dir=str(root), bag_dir=str(root / "bag"),
                    baseline_export_dir="", baseline_bag_dir="", p0_2_export_dir="",
                    p0_2_bag_dir="", synthetic_only=False, blocked_fixture_audit=False,
                    fail_on_threshold=True,
                ))
            finally:
                analyzer.read_bag_metadata = original
            self.assertEqual(result["status"], "FAIL")
            self.assertEqual(result["exception"]["type"], "RuntimeError")
            self.assertNotEqual(result["analysis_run_id"], "old")
            persisted = json.loads(old_summary.read_text())
            self.assertEqual(persisted["analysis_run_id"], result["analysis_run_id"])
            self.assertEqual(persisted["artifacts"], {"csv": [], "figures": []})


if __name__ == "__main__":
    unittest.main()
