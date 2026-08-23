import csv
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "gate0_analyzer.py"
)
SPEC = importlib.util.spec_from_file_location("gate0_analyzer", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


REQUIRED_COUNTER_FIELDS = (
    "refresh_query_count",
    "provider_query_count",
    "occupied_skip_count",
    "predictor_unique_positions",
    "predictor_requested_worker_count",
    "predictor_effective_worker_count",
    "predictor_spatial_advisory_recompute_count",
    "predictor_spatial_advisory_reuse_count",
    "predictor_gnss_advisory_invocation_count",
    "predictor_lidar_advisory_invocation_count",
    "predictor_horizon_fusion_count",
    "predictor_spatial_retained_position_count",
    "predictor_spatial_entered_position_count",
    "predictor_spatial_evicted_position_count",
    "predictor_spatial_full_invalidation_count",
    "predictor_spatial_exact_retained_position_count",
    "predictor_spatial_ttl_retained_position_count",
    "predictor_spatial_gnss_ttl_expired_position_count",
    "predictor_spatial_legacy_current_ttl_expired_position_count",
    "predictor_spatial_watchdog_forced_full_rebuild_count",
    "predictor_spatial_invalid_source_provenance_count",
)


def valid_p0_message(**overrides):
    generation_id = overrides.get("generation_id", 1)
    default_attempt_id = (
        generation_id
        if isinstance(generation_id, int)
        and not isinstance(generation_id, bool)
        and generation_id > 0
        else 1
    )
    attempt_id = overrides.get("refresh_attempt_id", default_attempt_id)
    previous_generation_id = (
        generation_id - 1
        if isinstance(generation_id, int)
        and not isinstance(generation_id, bool)
        and generation_id > 1
        else 0
    )
    message = {
        "refresh_attempt_id": attempt_id,
        "refresh_evidence_state": "COMPLETED_SUCCESS",
        "generation_id": generation_id,
        "result_generation_id": generation_id,
        "previous_successful_generation_id": previous_generation_id,
        "refresh_callback_start_stamp_s": 0.5,
        "refresh_callback_start_steady_s": 0.5,
        "refresh_callback_end_steady_s": 1.0,
        "refresh_callback_end_stamp_s": 1.0,
        "refresh_query_count": 76800,
        "provider_query_count": 76800,
        "occupied_skip_count": 0,
        "predictor_unique_positions": 12800,
        "predictor_requested_worker_count": 4,
        "predictor_effective_worker_count": 4,
        "predictor_spatial_advisory_recompute_count": 12800,
        "predictor_spatial_advisory_reuse_count": 64000,
        "predictor_gnss_advisory_invocation_count": 12800,
        "predictor_lidar_advisory_invocation_count": 12800,
        "predictor_horizon_fusion_count": 76800,
        "predictor_spatial_retained_position_count": 0,
        "predictor_spatial_entered_position_count": 12800,
        "predictor_spatial_evicted_position_count": 0,
        "predictor_spatial_full_invalidation_count": 0,
        "predictor_spatial_exact_retained_position_count": 0,
        "predictor_spatial_ttl_retained_position_count": 0,
        "predictor_spatial_gnss_ttl_expired_position_count": 0,
        "predictor_spatial_legacy_current_ttl_expired_position_count": 0,
        "predictor_spatial_watchdog_forced_full_rebuild_count": 0,
        "predictor_spatial_invalid_source_provenance_count": 0,
        "predictor_spatial_invalidation_reason": "uninitialized",
        "ready": True,
        "stale": False,
        "valid_ratio": 1.0,
        "unknown_ratio": 0.0,
        "reason": "ok",
        "refresh_elapsed_ms": 100.0,
        "provider_batch_duration_ms": 80.0,
        "generation_interval_ms": None if generation_id == 1 else 500.0,
        "refresh_stamp_s": 1.0,
        "snapshot_available": True,
        "snapshot_failure_reason": "none",
        "odom_seen": True,
        "odom_valid": True,
        "odom_fresh": True,
        "odom_stamp_s": 1.0,
        "current_integrity_seen": True,
        "current_integrity_valid": True,
        "current_integrity_fresh": True,
        "current_integrity_stamp_s": 1.0,
        "gnss_epoch_seen": True,
        "gnss_epoch_valid": True,
        "gnss_epoch_fresh": True,
        "gnss_epoch_stamp_s": 1.0,
        "gnss_epoch_satellite_count": 31,
        "origin_seen": True,
        "origin_valid": True,
        "origin_fresh": True,
        "origin_stamp_s": 1.0,
        "map_seen": True,
        "map_valid": True,
        "map_fresh": True,
        "map_stamp_s": 1.0,
        "map_point_count": 23309,
    }
    message.update(overrides)
    return message


def message_stamp_unavailable_failure(**overrides):
    message = valid_p0_message(
        refresh_attempt_id=4,
        refresh_evidence_state="COMPLETED_FAILURE",
        generation_id=0,
        result_generation_id=0,
        previous_successful_generation_id=0,
        refresh_stamp_s=None,
        refresh_callback_start_stamp_s=None,
        refresh_callback_start_steady_s=2.0005731,
        refresh_callback_end_stamp_s=None,
        refresh_callback_end_steady_s=2.00057666,
        refresh_elapsed_ms=0.003615,
        provider_batch_duration_ms=None,
        generation_interval_ms=None,
        provider_invalid_count=0,
        provider_stale_count=0,
        predictor_conservative_max_count=0,
        predictor_gnss_used_count=0,
        predictor_spatial_invalidation_reason="none",
        predictor_lidar_evaluations=0,
        predictor_lidar_cache_hits=0,
        predictor_lidar_fim_primitive_count=0,
        predictor_lidar_fim_valid_normal_count=0,
        predictor_lidar_map_point_count=0,
        predictor_lidar_used_count=0,
        predictor_prior_used_count=0,
        predictor_regularized_count=0,
        predictor_stale_current_prior_count=0,
        ready=False,
        stale=True,
        valid_ratio=0.0,
        unknown_ratio=1.0,
        reason="message_stamp_unavailable",
        snapshot_available=False,
        snapshot_failure_reason="message_stamp_unavailable",
        odom_seen=False,
        odom_valid=False,
        odom_fresh=False,
        odom_stamp_s=None,
        current_integrity_seen=False,
        current_integrity_valid=False,
        current_integrity_fresh=False,
        current_integrity_stamp_s=None,
        gnss_epoch_seen=False,
        gnss_epoch_valid=False,
        gnss_epoch_fresh=False,
        gnss_epoch_stamp_s=None,
        gnss_epoch_satellite_count=0,
        origin_seen=False,
        origin_valid=False,
        origin_fresh=False,
        origin_stamp_s=None,
        map_seen=False,
        map_valid=False,
        map_fresh=False,
        map_stamp_s=None,
        map_point_count=0,
    )
    for field in REQUIRED_COUNTER_FIELDS:
        if field not in {
            "predictor_requested_worker_count",
            "predictor_effective_worker_count",
        }:
            message[field] = 0
    message.update(overrides)
    return message


class Gate0AnalyzerTest(unittest.TestCase):
    def test_canonical_control_point_hash_is_frozen(self):
        digest, canonical = MODULE.canonical_control_points_hash(
            degree=3,
            ts=0.4,
            rows=3,
            cols=2,
            values=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
        self.assertEqual(
            canonical,
            "gate0_control_points_v1\n"
            "degree=3\n"
            "ts=0.40000000000000002\n"
            "rows=3\n"
            "cols=2\n"
            "values=1,2,3,4,5,6\n",
        )
        self.assertEqual(
            digest,
            "d4c6389b49b9a415101295329292411197948292f58d1087b4e4a22575f3879b",
        )

    def test_attempt_grouping_repeats_aggregate_counts_per_candidate(self):
        rows = [
            {
                "run_id": "primary-r1", "event": "attempt_start",
                "planning_attempt_id": "4", "candidate_id": "0",
                "collision_segment_count": "2", "base_generated_count": "2",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_input",
                "planning_attempt_id": "4", "candidate_id": "1",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_result",
                "planning_attempt_id": "4", "candidate_id": "1",
                "optimizer_success": "1", "original_cost": "5",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_input",
                "planning_attempt_id": "4", "candidate_id": "2",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_result",
                "planning_attempt_id": "4", "candidate_id": "2",
                "optimizer_success": "1", "original_cost": "6",
            },
            {
                "run_id": "primary-r1", "event": "selection",
                "planning_attempt_id": "4", "candidate_id": "1",
                "original_candidate_id": "1", "selected_candidate_id": "1",
            },
            {
                "run_id": "primary-r1", "event": "update_traj_info",
                "planning_attempt_id": "4", "candidate_id": "1",
                "update_traj_info": "1",
            },
            {
                "run_id": "primary-r1", "event": "attempt_candidates_complete",
                "planning_attempt_id": "4", "candidate_id": "0",
            },
        ]
        candidates, attempts = MODULE.aggregate_candidate_events(rows)
        self.assertEqual(len(attempts), 1)
        self.assertEqual(len(candidates), 2)
        self.assertEqual({row["generated"] for row in candidates}, {2})
        self.assertEqual({row["optimizer_input"] for row in candidates}, {2})
        self.assertEqual({row["optimizer_success"] for row in candidates}, {2})
        self.assertTrue(attempts[0]["qualified"])
        self.assertTrue(attempts[0]["selected_reached_downstream"])

        invalid_rows = [dict(row) for row in rows]
        for row in invalid_rows:
            if row["event"] in {"selection", "update_traj_info"}:
                row["candidate_id"] = "3"
            if row["event"] == "selection":
                row["original_candidate_id"] = "3"
                row["selected_candidate_id"] = "3"
        _, invalid_attempts = MODULE.aggregate_candidate_events(invalid_rows)
        self.assertTrue(invalid_attempts[0]["critical_violation"])

    def test_gate0a_go_conditional_and_no_go(self):
        stable = []
        for scenario in MODULE.GATE0A_SCENARIOS:
            for repeat in range(1, 4):
                stable.append({
                    "scenario": scenario,
                    "run_id": f"{scenario}-r{repeat}",
                    "qualified": True,
                    "selected_reached_downstream": True,
                    "critical_violation": False,
                    "attempt_count": 2,
                    "under_two_success_attempts": 0,
                })
        self.assertEqual(MODULE.decide_gate0a(stable)["status"], "GO")

        with_p0 = [*stable, {
            "scenario": "gnss_open_sky",
            "run_id": "p0-full-grid",
            "qualified": False,
            "selected_reached_downstream": False,
            "critical_violation": False,
            "attempt_count": 0,
            "under_two_success_attempts": 0,
        }]
        self.assertEqual(MODULE.decide_gate0a(with_p0)["status"], "GO")

        partial = [dict(row) for row in stable]
        for row in partial:
            if row["scenario"] == "flat-null":
                row["qualified"] = False
        self.assertEqual(MODULE.decide_gate0a(partial)["status"], "CONDITIONAL")

        violated = [dict(row) for row in stable]
        violated[0]["critical_violation"] = True
        self.assertEqual(MODULE.decide_gate0a(violated)["status"], "NO-GO-P2")

    def test_control_point_evidence_fails_closed_on_truncated_matrix(self):
        candidate = {
            "run_id": "primary-r1",
            "planning_attempt_id": 1,
            "candidate_id": 1,
            "candidate_optimizer_success": False,
            "is_selected": False,
            "selected_reached_downstream": False,
        }
        points = []
        for stage in ("generated", "optimizer_input"):
            for row in range(3):
                for column in range(2):
                    points.append({
                        "run_id": "primary-r1",
                        "planning_attempt_id": "1",
                        "candidate_id": "1",
                        "stage": stage,
                        "degree": "3",
                        "ts": "0.4",
                        "rows": "3",
                        "cols": "2",
                        "point_row": str(row),
                        "point_col": str(column),
                        "value": str(row + column),
                    })
        violations, global_failure = MODULE.validate_control_point_evidence(
            points, [candidate]
        )
        self.assertFalse(global_failure)
        self.assertEqual(violations, {})
        violations, _ = MODULE.validate_control_point_evidence(
            points[:-1], [candidate]
        )
        self.assertIn(("primary-r1", 1), violations)
        _, orphan_failure = MODULE.validate_control_point_evidence(points, [])
        self.assertTrue(orphan_failure)

    def test_manifest_checks_fail_closed_on_mapping_and_missing_process_codes(self):
        gate0a_failures = MODULE.validate_gate0a_manifest({
            "scenario": "primary",
            "run_id": "primary-r1",
            "effective_config": {"scenario": "p1_fork_fused_mirror_v1"},
        })
        self.assertIn("scenario_mapping_mismatch", gate0a_failures)
        self.assertIn("planner_crash", gate0a_failures)
        gate0b_failures = MODULE.validate_gate0b_manifest(
            {"effective_config": {}}, None, 0
        )
        self.assertIn("p0_process_failure", gate0b_failures)
        self.assertIn("p0_required_process_failure", gate0b_failures)

    def test_gate0b_manifest_requires_cpu_mapping_and_process_evidence(self):
        config = {
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
            "p0.predictor.worker_count": 4,
            "p0.skip_occupied_voxels": True,
            "record_bag": False,
            "start_rviz": False,
            "run_duration_s": 60,
            "validation_duration_s": 55,
            "planner_enable_all_safety": False,
            "planner_enable_p1": False,
            "planner_enable_p2": False,
            "planner_enable_p3_local": False,
            "planner_enable_p3_global": False,
            "planner_enable_p4": False,
            "planner_enable_p5_runtime": False,
            "planner_enable_p5_final": False,
            "p1.use_integrity_cost": False,
            "p1.metrics_only": False,
            "p2.enable_candidate_ranking": False,
            "p3.enable_local_reference_bias": False,
            "p3.enable_global_reference_bias": False,
            "p4.enable_risk_aware_astar": False,
            "p5.enable_runtime_gate": False,
            "p5.enable_final_gate": False,
        }
        manifest = {
            "effective_config": config,
            "scenario": "gnss_open_sky",
            "run_id": "p0-full-grid",
            "exit_code": 0,
            "capture_exit_code": 0,
            "planner_crash": False,
            "required_processes_ok": True,
            "required_processes": {
                "iap_rosnode": {
                    "seen": True,
                    "runtime_failure": False,
                }
            },
            "process_failures": [],
            "gpu_preflight": {
                "schema_version": "iap_gpu_preflight_v1",
                "gpu_ready": True,
                "failure_reason": "",
            },
            "capture_readiness": {
                "schema_version": "gate0_capture_readiness_v1",
                "ready": True,
            },
        }
        runtime = {
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
            "p0.predictor.requested_worker_count": 4,
            "p0.predictor.effective_worker_count": 4,
            "p0.skip_occupied_voxels": True,
            "record_bag": False,
            "start_rviz": False,
            "run_duration_s": 60.0,
            "validation_duration_s": 55.0,
            "planner_enable_all_safety": False,
            "planner_enable_p1": False,
            "planner_enable_p2": False,
            "planner_enable_p3_local": False,
            "planner_enable_p3_global": False,
            "planner_enable_p4": False,
            "planner_enable_p5_runtime": False,
            "planner_enable_p5_final": False,
            "p1.use_integrity_cost": False,
            "p1.metrics_only": False,
            "p1.debug_csv_enable": False,
            "p2.enable_candidate_ranking": False,
            "p2.debug_csv_enable": False,
            "p3.enable_local_reference_bias": False,
            "p3.enable_global_reference_bias": False,
            "p3.debug_csv_enable": False,
            "p4.enable_risk_aware_astar": False,
            "p4.debug_csv_enable": False,
            "p5.enable_runtime_gate": False,
            "p5.enable_final_gate": False,
            "gate0.qualification_evidence_enable": False,
            "iap_mapping_backend": "cpu",
            "mapping_effective_config": {
                "selected": "cpu",
                "odometry_config": {"path": "/tmp/o.json", "sha256": "a" * 64},
                "sub_mapping_config": {"path": "/tmp/s.json", "sha256": "b" * 64},
                "global_mapping_config": {"path": "/tmp/g.json", "sha256": "c" * 64},
            },
        }
        self.assertEqual(
            MODULE.validate_gate0b_manifest(manifest, runtime, 1), []
        )
        runtime["p0.predictor.requested_worker_count"] = 1
        self.assertIn(
            "p0_runtime_p0.predictor.requested_worker_count_mismatch",
            MODULE.validate_gate0b_manifest(manifest, runtime, 1),
        )
        runtime["p0.predictor.requested_worker_count"] = 4
        runtime["p0.predictor.effective_worker_count"] = 2
        self.assertIn(
            "p0_runtime_p0.predictor.effective_worker_count_mismatch",
            MODULE.validate_gate0b_manifest(manifest, runtime, 1),
        )
        runtime["p0.predictor.effective_worker_count"] = 4
        runtime["iap_mapping_backend"] = "gpu"
        self.assertIn(
            "p0_runtime_mapping_backend_mismatch",
            MODULE.validate_gate0b_manifest(manifest, runtime, 1),
        )

    def test_gate0b_manifest_rejects_one_and_every_non_four_worker_value(self):
        for worker_count in (1, 2, 3, 5, 8):
            with self.subTest(worker_count=worker_count):
                manifest = {
                    "run_id": "p0-smoke",
                    "effective_config": {
                        "p0.predictor.worker_count": worker_count,
                        "run_duration_s": 20,
                        "validation_duration_s": 15,
                    },
                    "gpu_preflight": {"gpu_ready": True},
                    "capture_readiness": {"ready": True},
                }
                failures = MODULE.validate_gate0b_manifest(manifest, None, 0)
                self.assertIn("p0_p0.predictor.worker_count_mismatch", failures)

    def test_smoke_and_benchmark_use_distinct_fixed_contracts(self):
        messages = [valid_p0_message(refresh_elapsed_ms=900.0)]
        _, smoke = MODULE.analyze_p0_messages(messages, protocol="smoke")
        _, benchmark = MODULE.analyze_p0_messages(messages, protocol="benchmark")
        self.assertEqual(smoke["gate"], "PASS")
        self.assertNotIn(
            "fewer_than_required_successful_generations", smoke["failures"]
        )
        self.assertNotIn("refresh_p95_over_400_ms", smoke["failures"])
        self.assertIn(
            "fewer_than_required_successful_generations", benchmark["failures"]
        )
        self.assertNotIn("refresh_p95_over_400_ms", benchmark["failures"])
        self.assertEqual(benchmark["gate"], "P0_EVIDENCE_CONTRACT_FAIL")
        self.assertEqual(benchmark["recommendations"], [])

    def test_smoke_manifest_requires_20_and_15_seconds(self):
        manifest = {
            "run_id": "p0-smoke",
            "effective_config": {
                "run_duration_s": 60,
                "validation_duration_s": 55,
            },
            "gpu_preflight": {"gpu_ready": True},
            "capture_readiness": {"ready": True},
        }
        failures = MODULE.validate_gate0b_manifest(manifest, None, 0)
        self.assertIn("p0_run_duration_s_mismatch", failures)
        self.assertIn("p0_validation_duration_s_mismatch", failures)

    def test_zero_p0_generations_are_input_availability_failure(self):
        _, summary = MODULE.analyze_p0_messages([], protocol="smoke")
        self.assertEqual(summary["successful_generation_count"], 0)
        self.assertEqual(summary["minimum_successful_generations"], 1)
        self.assertIn(
            "fewer_than_required_successful_generations", summary["failures"]
        )
        self.assertNotIn("fewer_than_20_successful_generations", summary["failures"])
        self.assertEqual(summary["gate"], "P0_INPUT_AVAILABILITY_FAIL")
        self.assertEqual(summary["recommendations"], [])

    def test_malformed_successful_generation_is_evidence_contract_failure(self):
        defects = (
            (
                {"provider_query_count": "76800"},
                "required_counter_non_integral:provider_query_count",
            ),
            ({"odom_seen": False}, "source_readiness_not_true:odom_seen"),
            ({"snapshot_available": False}, "snapshot_unavailable"),
            (
                {"provider_batch_duration_ms": math.nan},
                "successful_generation_timing_nonfinite:provider_batch_duration_ms",
            ),
        )
        for override, failure in defects:
            with self.subTest(failure=failure):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**override)], protocol="smoke"
                )
                self.assertIn(failure, summary["failures"])
                self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")
                self.assertEqual(summary["recommendations"], [])

    def test_insufficient_benchmark_is_evidence_contract_failure(self):
        messages = [
            valid_p0_message(
                generation_id=generation,
                refresh_callback_end_steady_s=float(generation),
                refresh_stamp_s=float(generation),
            )
            for generation in range(1, 20)
        ]
        _, summary = MODULE.analyze_p0_messages(messages, protocol="benchmark")
        self.assertEqual(summary["minimum_successful_generations"], 20)
        self.assertIn(
            "fewer_than_required_successful_generations", summary["failures"]
        )
        self.assertNotIn("refresh_p95_over_400_ms", summary["failures"])
        self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")
        self.assertEqual(summary["recommendations"], [])

    def test_complete_over_threshold_benchmark_is_performance_failure(self):
        messages = [
            valid_p0_message(
                generation_id=generation,
                refresh_callback_end_steady_s=float(generation),
                refresh_elapsed_ms=500.0,
                refresh_stamp_s=float(generation),
            )
            for generation in range(1, 21)
        ]
        _, summary = MODULE.analyze_p0_messages(messages, protocol="benchmark")
        self.assertEqual(summary["failures"], ["refresh_p95_over_400_ms"])
        self.assertEqual(summary["gate"], "P0_PERFORMANCE_GATE_FAIL")
        self.assertNotEqual(summary["recommendations"], [])

    def test_benchmark_zero_integrity_rows_fail_input_availability(self):
        summary = MODULE.apply_integrity_evidence_gate(
            {"gate": "PASS", "failures": []}, [], protocol="benchmark"
        )
        self.assertEqual(summary["integrity_report_count"], 0)
        self.assertEqual(summary["valid_integrity_report_count"], 0)
        self.assertIn("no_valid_integrity_report", summary["failures"])
        self.assertEqual(summary["gate"], "P0_INPUT_AVAILABILITY_FAIL")
        self.assertEqual(MODULE.analyzer_exit_code({"gate0b": summary}), 1)

    def test_benchmark_invalid_or_nonfinite_integrity_rows_fail_closed(self):
        messages = [
            {
                "valid": False,
                "hpl": 1.0,
                "vpl": 2.0,
                "hal": 10.0,
                "val": 10.0,
                "im": 8.0,
            },
            {
                "valid": True,
                "hpl": math.nan,
                "vpl": 2.0,
                "hal": 10.0,
                "val": 10.0,
                "im": 8.0,
            },
        ]
        summary = MODULE.apply_integrity_evidence_gate(
            {"gate": "PASS", "failures": []},
            messages,
            protocol="benchmark",
        )
        self.assertEqual(summary["integrity_report_count"], 2)
        self.assertEqual(summary["valid_integrity_report_count"], 0)
        self.assertEqual(summary["gate"], "P0_INPUT_AVAILABILITY_FAIL")
        self.assertEqual(MODULE.analyzer_exit_code({"gate0b": summary}), 1)

    def test_evidence_contract_failure_survives_missing_integrity_and_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "runs"
            smoke = root / "smoke"
            output = smoke / "analyzer"
            smoke.mkdir(parents=True)
            manifest = {
                "run_id": "p0-smoke",
                "scenario": "gnss_open_sky",
                "effective_config": {},
                "gpu_preflight": {"gpu_ready": True},
                "capture_readiness": {"ready": True},
            }
            (smoke / "gate0_run_manifest.json").write_text(
                json.dumps(manifest) + "\n"
            )
            malformed = valid_p0_message()
            del malformed["refresh_callback_end_steady_s"]
            (smoke / "risk_grid_health.jsonl").write_text(
                json.dumps(malformed) + "\n"
            )
            (smoke / "integrity_report.jsonl").write_text("")

            result = MODULE.analyze_directory(root, output)

            self.assertEqual(
                result["gate0b"]["gate"], "P0_EVIDENCE_CONTRACT_FAIL"
            )
            self.assertIn(
                "completed_attempt_identity_invalid:refresh_callback_end_steady_s",
                result["gate0b"]["failures"],
            )
            self.assertIn("no_valid_integrity_report", result["gate0b"]["failures"])
            self.assertNotEqual(MODULE.analyzer_exit_code(result), 0)

    def test_success_to_failure_remains_nonpassing_through_directory_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "runs"
            smoke = root / "smoke"
            output = smoke / "analyzer"
            smoke.mkdir(parents=True)
            manifest = {
                "run_id": "p0-smoke",
                "scenario": "gnss_open_sky",
                "effective_config": {},
                "gpu_preflight": {"gpu_ready": True},
                "capture_readiness": {"ready": True},
                "exit_code": 0,
                "planner_crash": False,
                "required_processes_ok": True,
                "required_processes": {
                    "iap_rosnode": {"seen": True, "runtime_failure": False}
                },
            }
            (smoke / "gate0_run_manifest.json").write_text(
                json.dumps(manifest) + "\n"
            )
            health = [
                valid_p0_message(
                    generation_id=1,
                    refresh_callback_end_steady_s=1.0,
                    refresh_stamp_s=1.0,
                    refresh_elapsed_ms=100.0,
                ),
                valid_p0_message(
                    refresh_attempt_id=2,
                    refresh_evidence_state="COMPLETED_FAILURE",
                    generation_id=1,
                    result_generation_id=0,
                    previous_successful_generation_id=1,
                    refresh_callback_end_steady_s=2.0,
                    refresh_stamp_s=2.0,
                    snapshot_available=False,
                    snapshot_failure_reason="occupancy_stale",
                    ready=False,
                    reason="occupancy_stale",
                    refresh_elapsed_ms=900.0,
                ),
            ]
            (smoke / "risk_grid_health.jsonl").write_text(
                "".join(json.dumps(item) + "\n" for item in health)
            )
            integrity = {
                "valid": True,
                "hpl": 1.0,
                "vpl": 1.0,
                "hal": 10.0,
                "val": 10.0,
                "im": 9.0,
            }
            (smoke / "integrity_report.jsonl").write_text(
                json.dumps(integrity) + "\n"
            )

            result = MODULE.analyze_directory(root, output)
            with (output / "p0_smoke_benchmark.csv").open(newline="") as stream:
                rows = list(csv.DictReader(stream))

            self.assertNotEqual(result["gate0b"]["gate"], "PASS")
            self.assertEqual(result["gate0b"]["successful_generation_count"], 1)
            self.assertEqual(result["gate0b"]["completed_failure_count"], 1)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["generation_id"], "1")
            self.assertEqual(rows[1]["failed_refresh"], "1")

    def test_successful_generation_with_nonfinite_latency_fails_closed(self):
        messages = []
        for generation in range(1, 21):
            messages.append(valid_p0_message(
                generation_id=generation,
                refresh_callback_end_steady_s=float(generation),
                refresh_stamp_s=float(generation),
            ))
        messages[-1]["refresh_elapsed_ms"] = "null"
        _, summary = MODULE.analyze_p0_messages(messages)
        self.assertIn("successful_generation_latency_nonfinite", summary["failures"])
        self.assertNotEqual(summary["gate"], "PASS")

    def test_selected_reachability_stages_are_reported_independently(self):
        rows = [
            {
                "run_id": "primary-r1", "event": "attempt_start",
                "planning_attempt_id": "1", "candidate_id": "0",
                "collision_segment_count": "1", "base_generated_count": "1",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_input",
                "planning_attempt_id": "1", "candidate_id": "1",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_result",
                "planning_attempt_id": "1", "candidate_id": "1",
                "optimizer_success": "1", "original_cost": "5",
            },
            {
                "run_id": "primary-r1", "event": "selection",
                "planning_attempt_id": "1", "candidate_id": "1",
                "original_candidate_id": "1", "selected_candidate_id": "1",
            },
            {
                "run_id": "primary-r1", "event": "refinement_result",
                "planning_attempt_id": "1", "candidate_id": "1",
                "ego_feasible": "1",
            },
            {
                "run_id": "primary-r1", "event": "normal_bspline_publish",
                "planning_attempt_id": "1", "candidate_id": "0",
                "bspline_publish_count": "1",
            },
            {
                "run_id": "primary-r1", "event": "attempt_candidates_complete",
                "planning_attempt_id": "1", "candidate_id": "0",
            },
        ]
        _, attempts = MODULE.aggregate_candidate_events(rows)
        self.assertTrue(attempts[0]["selected_refinement_reached"])
        self.assertTrue(attempts[0]["selected_publish_reached"])
        self.assertFalse(attempts[0]["selected_update_reached"])
        self.assertTrue(attempts[0]["selected_reached_downstream"])

    def test_nonfinite_original_cost_fails_closed(self):
        rows = [
            {
                "run_id": "primary-r1", "event": "attempt_start",
                "planning_attempt_id": "1", "candidate_id": "0",
                "collision_segment_count": "1", "base_generated_count": "1",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_input",
                "planning_attempt_id": "1", "candidate_id": "1",
            },
            {
                "run_id": "primary-r1", "event": "optimizer_result",
                "planning_attempt_id": "1", "candidate_id": "1",
                "optimizer_success": "1", "original_cost": "NaN",
            },
            {
                "run_id": "primary-r1", "event": "attempt_candidates_complete",
                "planning_attempt_id": "1", "candidate_id": "0",
            },
        ]
        _, attempts = MODULE.aggregate_candidate_events(rows)
        self.assertTrue(attempts[0]["critical_violation"])
        self.assertIn(
            "candidate_original_cost_nonfinite",
            attempts[0]["critical_reasons"],
        )
        self.assertFalse(attempts[0]["qualified"])

    def test_type7_quantile_and_p0_deduplication_failure_gate(self):
        self.assertEqual(MODULE.type7_quantile([0.0, 10.0, 20.0, 30.0], 0.5), 15.0)
        self.assertAlmostEqual(
            MODULE.type7_quantile([0.0, 10.0, 20.0, 30.0], 0.95), 28.5
        )
        messages = []
        for generation in range(1, 22):
            messages.append(valid_p0_message(
                generation_id=generation,
                refresh_callback_end_steady_s=float(generation),
                refresh_elapsed_ms=100.0 + generation,
                refresh_stamp_s=float(generation),
            ))
        messages.append(dict(messages[-1]))
        rows, summary = MODULE.analyze_p0_messages(messages)
        self.assertEqual(len(rows), 21)
        self.assertEqual(summary["successful_generation_count"], 21)
        self.assertEqual(summary["gate"], "PASS")

        slow = [dict(message) for message in messages[:-1]]
        slow[-1]["refresh_elapsed_ms"] = 1000.0
        for message in slow[-3:]:
            message["refresh_elapsed_ms"] = 500.0
        _, failed = MODULE.analyze_p0_messages(slow)
        self.assertEqual(failed["gate"], "P0_PERFORMANCE_GATE_FAIL")
        self.assertGreater(failed["refresh_elapsed_ms_p95"], 400.0)

    def test_successful_generation_requires_every_rolling_counter(self):
        self.assertEqual(tuple(MODULE.REQUIRED_P0_COUNTER_FIELDS), REQUIRED_COUNTER_FIELDS)
        self.assertTrue(set(REQUIRED_COUNTER_FIELDS).issubset(MODULE.P0_FIELDS))
        self.assertIn("predictor_spatial_invalidation_reason", MODULE.P0_FIELDS)
        self.assertTrue(set(MODULE.REQUIRED_P0_TIMING_FIELDS).issubset(MODULE.P0_FIELDS))
        for field in REQUIRED_COUNTER_FIELDS:
            with self.subTest(field=field, defect="missing"):
                message = valid_p0_message()
                del message[field]
                _, summary = MODULE.analyze_p0_messages([message], protocol="smoke")
                self.assertIn(f"required_counter_missing:{field}", summary["failures"])
            with self.subTest(field=field, defect="malformed"):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**{field: "not-an-integer"})], protocol="smoke"
                )
                self.assertIn(f"required_counter_non_integral:{field}", summary["failures"])
            with self.subTest(field=field, defect="negative"):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**{field: -1})], protocol="smoke"
                )
                self.assertIn(f"required_counter_negative:{field}", summary["failures"])

    def test_successful_generation_rejects_each_counter_identity_violation(self):
        contradictions = {
            "refresh_query_count_mismatch": {"refresh_query_count": 76799},
            "provider_plus_occupied_skip_mismatch": {"provider_query_count": 76799},
            "spatial_recompute_plus_reuse_mismatch": {
                "predictor_spatial_advisory_reuse_count": 63999,
            },
            "horizon_fusion_mismatch": {"predictor_horizon_fusion_count": 76799},
            "gnss_advisory_invocation_mismatch": {
                "predictor_gnss_advisory_invocation_count": 12799,
            },
            "lidar_advisory_invocation_mismatch": {
                "predictor_lidar_advisory_invocation_count": 12799,
            },
            "retained_plus_entered_mismatch": {
                "predictor_spatial_entered_position_count": 12799,
            },
            "predictor_unique_positions_out_of_range": {
                "predictor_unique_positions": 12801,
            },
            "predictor_spatial_retained_position_count_out_of_range": {
                "predictor_spatial_retained_position_count": 12801,
            },
            "predictor_spatial_entered_position_count_out_of_range": {
                "predictor_spatial_entered_position_count": 12801,
            },
            "predictor_spatial_evicted_position_count_out_of_range": {
                "predictor_spatial_evicted_position_count": 12801,
            },
            "requested_worker_count_mismatch": {"predictor_requested_worker_count": 1},
            "effective_worker_count_mismatch": {"predictor_effective_worker_count": 2},
        }
        for failure, override in contradictions.items():
            with self.subTest(failure=failure):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**override)], protocol="smoke"
                )
                self.assertIn(failure, summary["failures"])

    def test_successful_generation_requires_finite_timings_and_invalidation_reason(self):
        for field in ("refresh_elapsed_ms", "provider_batch_duration_ms"):
            with self.subTest(field=field, defect="missing"):
                message = valid_p0_message()
                del message[field]
                _, summary = MODULE.analyze_p0_messages([message], protocol="smoke")
                self.assertIn(f"successful_generation_timing_nonfinite:{field}", summary["failures"])
            with self.subTest(field=field, defect="nonfinite"):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**{field: math.inf})], protocol="smoke"
                )
                self.assertIn(f"successful_generation_timing_nonfinite:{field}", summary["failures"])
        for value in (None, ""):
            with self.subTest(invalidation_reason=value):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(predictor_spatial_invalidation_reason=value)],
                    protocol="smoke",
                )
                self.assertIn("spatial_invalidation_reason_missing", summary["failures"])

    def test_successful_generation_requires_source_readiness_and_stamps(self):
        self.assertIn("snapshot_available", MODULE.P0_FIELDS)
        for prefix in MODULE.REQUIRED_P0_SOURCE_PREFIXES:
            for suffix in ("seen", "valid", "fresh"):
                field = f"{prefix}_{suffix}"
                with self.subTest(field=field, defect="missing"):
                    message = valid_p0_message()
                    del message[field]
                    _, summary = MODULE.analyze_p0_messages([message], protocol="smoke")
                    self.assertIn(
                        f"source_readiness_not_true:{field}", summary["failures"]
                    )
                with self.subTest(field=field, defect="malformed"):
                    _, summary = MODULE.analyze_p0_messages(
                        [valid_p0_message(**{field: "true"})], protocol="smoke"
                    )
                    self.assertIn(
                        f"source_readiness_not_true:{field}", summary["failures"]
                    )
                with self.subTest(field=field, defect="false"):
                    _, summary = MODULE.analyze_p0_messages(
                        [valid_p0_message(**{field: False})], protocol="smoke"
                    )
                    self.assertIn(
                        f"source_readiness_not_true:{field}", summary["failures"]
                    )
            stamp_field = f"{prefix}_stamp_s"
            for value in (None, "not-a-number", math.inf, 0.0, -1.0):
                with self.subTest(field=stamp_field, value=value):
                    _, summary = MODULE.analyze_p0_messages(
                        [valid_p0_message(**{stamp_field: value})], protocol="smoke"
                    )
                    self.assertIn(
                        f"source_stamp_invalid:{stamp_field}", summary["failures"]
                    )

    def test_successful_generation_requires_clean_snapshot_failure_reason(self):
        for value in (None, "", "odom_missing"):
            with self.subTest(snapshot_failure_reason=value):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(snapshot_failure_reason=value)], protocol="smoke"
                )
                self.assertIn("snapshot_failure_reason_not_none", summary["failures"])
        for value in (None, "", 7, "occupancy_stale"):
            with self.subTest(reason=value):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(reason=value)], protocol="smoke"
                )
                self.assertIn("health_reason_not_ok", summary["failures"])
        for value in (None, False, "true"):
            with self.subTest(snapshot_available=value):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(snapshot_available=value)], protocol="smoke"
                )
                self.assertIn("snapshot_unavailable", summary["failures"])

    def test_completed_records_deduplicate_only_when_field_equivalent(self):
        completed = valid_p0_message()
        rows, summary = MODULE.analyze_p0_messages(
            [completed, dict(completed)], protocol="smoke"
        )
        self.assertEqual(len(rows), 1)
        self.assertEqual(summary["duplicate_completed_observation_count"], 1)
        self.assertEqual(summary["gate"], "PASS")

        conflict = dict(completed, refresh_elapsed_ms=101.0)
        _, failed = MODULE.analyze_p0_messages(
            [completed, conflict], protocol="smoke"
        )
        self.assertEqual(failed["conflicting_completed_observation_count"], 1)
        self.assertIn("conflicting_completed_attempt_record", failed["failures"])
        self.assertEqual(failed["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

    def test_icra032_interleaving_keeps_attempts_distinct(self):
        success = valid_p0_message()
        in_progress = {
            "refresh_attempt_id": 2,
            "refresh_evidence_state": "IN_PROGRESS",
            "generation_id": 1,
            "result_generation_id": 0,
            "previous_successful_generation_id": 1,
            "refresh_stamp_s": 2.0,
            "refresh_callback_start_stamp_s": 2.0,
            "refresh_callback_start_steady_s": 2.0,
            "ready": True,
            "reason": "ok",
        }
        failure = dict(
            in_progress,
            refresh_evidence_state="COMPLETED_FAILURE",
            refresh_callback_end_stamp_s=2.5,
            refresh_callback_end_steady_s=2.5,
            snapshot_available=False,
            snapshot_failure_reason="occupancy_stale",
            ready=False,
            reason="occupancy_stale",
        )
        rows, summary = MODULE.analyze_p0_messages(
            [success, in_progress, failure], protocol="smoke"
        )
        self.assertEqual([row["refresh_attempt_id"] for row in rows], [1, 2])
        self.assertEqual(summary["successful_generation_count"], 1)
        self.assertEqual(summary["in_progress_observation_count"], 1)
        self.assertEqual(summary["completed_failure_count"], 1)
        self.assertEqual(summary["gate"], "PASS")

    def test_message_stamp_unavailable_completed_failures_accept_full_startup_shape(self):
        attempt_4 = message_stamp_unavailable_failure()
        attempt_5 = message_stamp_unavailable_failure(
            refresh_attempt_id=5,
            refresh_callback_start_steady_s=2.500798263,
            refresh_callback_end_steady_s=2.500802107,
            refresh_elapsed_ms=0.003847,
        )
        success = valid_p0_message(
            refresh_attempt_id=6,
            generation_id=1,
            result_generation_id=1,
            previous_successful_generation_id=0,
        )

        rows, summary = MODULE.analyze_p0_messages(
            [attempt_4, attempt_5, success], protocol="smoke"
        )

        self.assertEqual([row["refresh_attempt_id"] for row in rows], [4, 5, 6])
        self.assertEqual(summary["completed_failure_count"], 2)
        self.assertEqual(summary["message_stamp_unavailable_failure_count"], 2)
        self.assertEqual(summary["successful_generation_count"], 1)
        self.assertEqual(summary["gate"], "PASS")

    def test_message_stamp_unavailable_completed_failure_fails_closed(self):
        defects = (
            (
                {"refresh_stamp_s": 2.0},
                "message_stamp_unavailable_timestamps_not_all_null",
            ),
            (
                {
                    "refresh_stamp_s": 2.0,
                    "refresh_callback_start_stamp_s": 2.0,
                    "refresh_callback_end_stamp_s": 2.1,
                },
                "message_stamp_unavailable_timestamps_not_all_null",
            ),
            (
                {"refresh_callback_start_steady_s": None},
                "message_stamp_unavailable_steady_invalid:refresh_callback_start_steady_s",
            ),
            (
                {"refresh_callback_end_steady_s": math.inf},
                "message_stamp_unavailable_steady_invalid:refresh_callback_end_steady_s",
            ),
            (
                {
                    "refresh_callback_start_steady_s": 3.0,
                    "refresh_callback_end_steady_s": 2.0,
                },
                "message_stamp_unavailable_steady_not_ordered",
            ),
            (
                {"refresh_elapsed_ms": math.nan},
                "message_stamp_unavailable_elapsed_invalid",
            ),
            (
                {"refresh_elapsed_ms": -0.1},
                "message_stamp_unavailable_elapsed_invalid",
            ),
            (
                {"refresh_query_count": 1},
                "message_stamp_unavailable_counter_not_zero:refresh_query_count",
            ),
            (
                {"provider_query_count": 1},
                "message_stamp_unavailable_counter_not_zero:provider_query_count",
            ),
            (
                {"provider_invalid_count": 1},
                "message_stamp_unavailable_counter_not_zero:provider_invalid_count",
            ),
            (
                {"predictor_lidar_evaluations": 1},
                "message_stamp_unavailable_counter_not_zero:predictor_lidar_evaluations",
            ),
            (
                {"predictor_regularized_count": 1},
                "message_stamp_unavailable_counter_not_zero:predictor_regularized_count",
            ),
            (
                {"result_generation_id": 1},
                "completed_failure_result_generation_not_zero",
            ),
            (
                {"generation_id": 1},
                "completed_failure_active_previous_generation_mismatch",
            ),
            (
                {"generation_id": 1, "previous_successful_generation_id": 1},
                "completed_failure_previous_generation_mismatch",
            ),
            (
                {"snapshot_available": True},
                "message_stamp_unavailable_snapshot_available",
            ),
            (
                {"snapshot_failure_reason": "occupancy_stale"},
                "message_stamp_unavailable_snapshot_reason_mismatch",
            ),
            (
                {"reason": "occupancy_stale"},
                "message_stamp_unavailable_reason_mismatch",
            ),
        )
        success = valid_p0_message(refresh_attempt_id=6)
        for overrides, failure in defects:
            with self.subTest(failure=failure):
                _, summary = MODULE.analyze_p0_messages(
                    [message_stamp_unavailable_failure(**overrides), success],
                    protocol="smoke",
                )
                self.assertIn(failure, summary["failures"])
                self.assertEqual(
                    summary["message_stamp_unavailable_failure_count"], 0
                )
                self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

        missing_stamp = message_stamp_unavailable_failure()
        del missing_stamp["refresh_callback_end_stamp_s"]
        _, missing_summary = MODULE.analyze_p0_messages(
            [missing_stamp, success], protocol="smoke"
        )
        self.assertIn(
            "message_stamp_unavailable_timestamp_missing:refresh_callback_end_stamp_s",
            missing_summary["failures"],
        )
        self.assertEqual(
            missing_summary["message_stamp_unavailable_failure_count"], 0
        )
        self.assertEqual(missing_summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

        completed = message_stamp_unavailable_failure()
        conflicting_duplicate = dict(completed, provider_invalid_count=1)
        _, conflict_summary = MODULE.analyze_p0_messages(
            [completed, conflicting_duplicate, success], protocol="smoke"
        )
        self.assertEqual(
            conflict_summary["duplicate_completed_observation_count"], 0
        )
        self.assertEqual(
            conflict_summary["conflicting_completed_observation_count"], 1
        )
        self.assertIn(
            "conflicting_completed_attempt_record", conflict_summary["failures"]
        )
        self.assertEqual(conflict_summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

    def test_null_message_timestamps_are_not_exempt_for_success_or_other_failure(self):
        success_with_nulls = valid_p0_message(
            refresh_stamp_s=None,
            refresh_callback_start_stamp_s=None,
            refresh_callback_end_stamp_s=None,
        )
        _, success_summary = MODULE.analyze_p0_messages(
            [success_with_nulls], protocol="smoke"
        )
        self.assertIn(
            "completed_attempt_identity_invalid:refresh_stamp_s",
            success_summary["failures"],
        )
        self.assertEqual(success_summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

        other_failure = message_stamp_unavailable_failure(
            reason="occupancy_stale",
            snapshot_failure_reason="occupancy_stale",
        )
        _, other_summary = MODULE.analyze_p0_messages(
            [other_failure, valid_p0_message(refresh_attempt_id=6)],
            protocol="smoke",
        )
        self.assertIn(
            "completed_attempt_identity_invalid:refresh_stamp_s",
            other_summary["failures"],
        )
        self.assertEqual(other_summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

    def test_in_progress_allows_active_map_cumulative_predictor_counters(self):
        in_progress = {
            "refresh_attempt_id": 1,
            "refresh_evidence_state": "IN_PROGRESS",
            "generation_id": 0,
            "result_generation_id": 0,
            "previous_successful_generation_id": 0,
            "refresh_stamp_s": 1.0,
            "refresh_callback_start_stamp_s": 1.0,
            "refresh_callback_start_steady_s": 1.0,
            "predictor_gnss_used_count": 11,
            "predictor_lidar_used_count": 12,
            "predictor_prior_used_count": 13,
            "predictor_regularized_count": 14,
            "ready": False,
            "reason": "not_ready",
        }
        _, summary = MODULE.analyze_p0_messages(
            [in_progress, valid_p0_message(refresh_attempt_id=2)],
            protocol="smoke",
        )

        self.assertEqual(summary["in_progress_observation_count"], 1)
        self.assertEqual(summary["successful_generation_count"], 1)
        self.assertEqual(summary["gate"], "PASS")

    def test_attempt_and_result_id_contracts_fail_closed(self):
        defects = (
            ({"refresh_attempt_id": None}, "refresh_attempt_id_invalid"),
            ({"refresh_attempt_id": 0}, "refresh_attempt_id_invalid"),
            ({"refresh_evidence_state": "UNKNOWN"}, "refresh_evidence_state_unknown"),
            ({"result_generation_id": 2}, "result_generation_active_generation_mismatch"),
        )
        for override, failure in defects:
            with self.subTest(failure=failure):
                _, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**override)], protocol="smoke"
                )
                self.assertIn(failure, summary["failures"])
                self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

        messages = [
            valid_p0_message(),
            valid_p0_message(
                refresh_attempt_id=2,
                generation_id=1,
                result_generation_id=1,
                previous_successful_generation_id=1,
                generation_interval_ms=500.0,
            ),
        ]
        _, reused = MODULE.analyze_p0_messages(messages, protocol="smoke")
        self.assertIn("result_generation_reused_or_regressed", reused["failures"])

        _, regressed = MODULE.analyze_p0_messages(
            [valid_p0_message(refresh_attempt_id=2), valid_p0_message()],
            protocol="smoke",
        )
        self.assertIn("refresh_attempt_id_regressed", regressed["failures"])

    def test_every_non_completed_forbidden_field_is_enforced_from_inventory(self):
        self.assertIn("generation_interval_ms", MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS)
        self.assertIn("predictor_lidar_evaluations", MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS)
        self.assertIn("predictor_lidar_cache_hits", MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS)
        covered = set(MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS) | set(
            MODULE.P0_ACTIVE_MAP_OBSERVABILITY_FIELDS
        ) | set(MODULE.P0_ATTEMPT_IDENTITY_FIELDS) | set(
            MODULE.P0_ATTEMPT_START_FIELDS
        )
        self.assertEqual(covered, set(MODULE.P0_COMPLETED_QUALIFICATION_FIELDS))
        base = {
            "refresh_attempt_id": 1,
            "refresh_evidence_state": "IN_PROGRESS",
            "generation_id": 0,
            "result_generation_id": 0,
            "previous_successful_generation_id": 0,
            "refresh_callback_start_stamp_s": 1.0,
            "refresh_callback_start_steady_s": 1.0,
            "ready": False,
            "reason": "not_ready",
        }
        for field in MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS:
            with self.subTest(field=field):
                claim = True if field.endswith(("_available", "_seen", "_valid", "_fresh")) else 1
                _, summary = MODULE.analyze_p0_messages(
                    [dict(base, **{field: claim})], protocol="smoke"
                )
                self.assertIn(
                    f"in_progress_completion_claim:{field}", summary["failures"]
                )
        pre_refresh = dict(
            base,
            refresh_evidence_state="PRE_REFRESH",
        )
        for field in MODULE.P0_PRE_REFRESH_FORBIDDEN_FIELDS:
            with self.subTest(pre_refresh_field=field):
                claim = True if field.endswith(
                    ("_available", "_seen", "_valid", "_fresh")
                ) else 1
                _, summary = MODULE.analyze_p0_messages(
                    [dict(pre_refresh, **{field: claim})], protocol="smoke"
                )
                self.assertIn(
                    f"pre_refresh_completion_claim:{field}", summary["failures"]
                )

    def test_non_completed_and_failure_active_generation_matches_previous_success(self):
        first = valid_p0_message()
        for state in ("IN_PROGRESS", "COMPLETED_FAILURE"):
            with self.subTest(state=state):
                next_attempt = valid_p0_message(
                    refresh_attempt_id=2,
                    refresh_evidence_state=state,
                    generation_id=999,
                    result_generation_id=0,
                    previous_successful_generation_id=1,
                    ready=False,
                    reason="snapshot_unavailable",
                    snapshot_available=False,
                    snapshot_failure_reason="snapshot_unavailable",
                )
                if state == "IN_PROGRESS":
                    for field in MODULE.P0_NON_COMPLETED_FORBIDDEN_FIELDS:
                        next_attempt[field] = None
                    next_attempt["refresh_callback_start_stamp_s"] = 2.0
                    next_attempt["refresh_callback_start_steady_s"] = 2.0
                    next_attempt["refresh_stamp_s"] = 2.0
                _, summary = MODULE.analyze_p0_messages(
                    [first, next_attempt], protocol="smoke"
                )
                self.assertIn(
                    f"{state.lower()}_active_previous_generation_mismatch",
                    summary["failures"],
                )

        self_consistent_but_unlinked = {
            "refresh_attempt_id": 2,
            "refresh_evidence_state": "IN_PROGRESS",
            "generation_id": 999,
            "result_generation_id": 0,
            "previous_successful_generation_id": 999,
            "refresh_stamp_s": 2.0,
            "refresh_callback_start_stamp_s": 2.0,
            "refresh_callback_start_steady_s": 2.0,
            "ready": False,
            "reason": "not_ready",
        }
        _, summary = MODULE.analyze_p0_messages(
            [first, self_consistent_but_unlinked], protocol="smoke"
        )
        self.assertIn(
            "in_progress_previous_generation_mismatch", summary["failures"]
        )

    def test_cold_and_later_success_interval_contract(self):
        first = valid_p0_message()
        second = valid_p0_message(generation_id=2, refresh_attempt_id=2)
        _, summary = MODULE.analyze_p0_messages([first, second], protocol="smoke")
        self.assertEqual(summary["gate"], "PASS")

        _, cold_bad = MODULE.analyze_p0_messages(
            [valid_p0_message(generation_interval_ms=500.0)], protocol="smoke"
        )
        self.assertIn("cold_start_generation_interval_not_null", cold_bad["failures"])
        _, later_bad = MODULE.analyze_p0_messages(
            [first, dict(second, generation_interval_ms=None)], protocol="smoke"
        )
        self.assertIn("later_generation_interval_not_positive_finite", later_bad["failures"])

    def test_icra031_health_replay_without_explicit_schema_fails_closed(self):
        replay_path = (
            Path(__file__).resolve().parents[1]
            / "results"
            / "icra27"
            / "icra031"
            / "runs"
            / "smoke"
            / "risk_grid_health.jsonl"
        )
        messages = [
            json.loads(line)
            for line in replay_path.read_text().splitlines()
            if line.strip()
        ]

        _, summary = MODULE.analyze_p0_messages(messages, protocol="smoke")

        self.assertIn("refresh_evidence_state_unknown", summary["failures"])
        self.assertEqual(summary["successful_generation_count"], 0)
        self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")

    def test_icra032_raw_replay_is_formally_closed_but_diagnostically_complete(self):
        replay_path = (
            Path(__file__).resolve().parents[1]
            / "results"
            / "icra27"
            / "icra032"
            / "runs"
            / "smoke"
            / "risk_grid_health.jsonl"
        )
        messages = [
            json.loads(line)
            for line in replay_path.read_text().splitlines()
            if line.strip()
        ]
        _, formal = MODULE.analyze_p0_messages(messages, protocol="smoke")
        self.assertEqual(formal["gate"], "P0_EVIDENCE_CONTRACT_FAIL")
        self.assertEqual(formal["successful_generation_count"], 0)
        self.assertIn("refresh_evidence_state_unknown", formal["failures"])

        complete_success = [
            row
            for row in messages
            if row.get("generation_id", 0) > 0
            and row.get("ready") is True
            and row.get("snapshot_available") is True
            and row.get("refresh_query_count") == 76800
            and math.isfinite(MODULE._float(row.get("provider_batch_duration_ms")))
        ]
        complete_generations = {row["generation_id"] for row in complete_success}
        real_failures = {
            row.get("refresh_callback_end_steady_s")
            for row in messages
            if row.get("generation_id") == 5
            and row.get("reason") == "snapshot_unavailable"
            and row.get("snapshot_failure_reason") == "snapshot_builder_invalid"
        }
        ambiguous_interleaving = [
            row
            for row in messages
            if row.get("generation_id", 0) > 0
            and row.get("refresh_query_count") == 0
            and row.get("provider_batch_duration_ms") is None
            and MODULE._float(row.get("refresh_callback_start_steady_s"))
            > MODULE._float(row.get("refresh_callback_end_steady_s"))
        ]
        self.assertEqual(len(complete_success), 19)
        self.assertEqual(len(complete_generations), 13)
        self.assertEqual(real_failures, {5.500132527})
        self.assertEqual(len(ambiguous_interleaving), 7)

    def test_p0_success_claim_requires_strict_complete_evidence(self):
        defects = (
            (
                {"generation_id": "1"},
                "successful_generation_id_not_positive_integral",
            ),
            (
                {"generation_id": True},
                "successful_generation_id_not_positive_integral",
            ),
            (
                {"generation_id": 0},
                "successful_generation_id_not_positive_integral",
            ),
            ({"reason": "occupancy_stale"}, "health_reason_not_ok"),
            ({"snapshot_available": False}, "snapshot_unavailable"),
            (
                {"refresh_query_count": "76800"},
                "required_counter_non_integral:refresh_query_count",
            ),
            (
                {"refresh_elapsed_ms": math.nan},
                "successful_generation_latency_nonfinite",
            ),
        )
        for override, failure in defects:
            with self.subTest(failure=failure):
                rows, summary = MODULE.analyze_p0_messages(
                    [valid_p0_message(**override)], protocol="smoke"
                )
                self.assertEqual(summary["successful_generation_count"], 0)
                self.assertEqual(summary["failed_refresh_count"], 1)
                self.assertIn(failure, summary["failures"])
                self.assertEqual(summary["gate"], "P0_EVIDENCE_CONTRACT_FAIL")
                self.assertEqual(rows[0]["failed_refresh"], 1)

    def test_p0_all_successful_generation_classes_share_one_distribution(self):
        messages = [
            valid_p0_message(
                generation_id=1,
                refresh_callback_end_steady_s=1.0,
                refresh_stamp_s=1.0,
                predictor_spatial_invalidation_reason="uninitialized",
                refresh_elapsed_ms=50.0,
            ),
            valid_p0_message(
                generation_id=2,
                refresh_callback_end_steady_s=2.0,
                refresh_stamp_s=2.0,
                predictor_spatial_invalidation_reason="rolling_shift",
                predictor_spatial_advisory_recompute_count=320,
                predictor_spatial_advisory_reuse_count=76480,
                predictor_gnss_advisory_invocation_count=320,
                predictor_lidar_advisory_invocation_count=320,
                predictor_spatial_retained_position_count=12480,
                predictor_spatial_entered_position_count=320,
                predictor_spatial_evicted_position_count=320,
                refresh_elapsed_ms=100.0,
            ),
            valid_p0_message(
                generation_id=3,
                refresh_callback_end_steady_s=3.0,
                refresh_stamp_s=3.0,
                predictor_spatial_invalidation_reason="exact_reuse",
                predictor_spatial_advisory_recompute_count=0,
                predictor_spatial_advisory_reuse_count=76800,
                predictor_gnss_advisory_invocation_count=0,
                predictor_lidar_advisory_invocation_count=0,
                predictor_spatial_retained_position_count=12800,
                predictor_spatial_entered_position_count=0,
                predictor_spatial_exact_retained_position_count=12800,
                refresh_elapsed_ms=150.0,
            ),
            valid_p0_message(
                generation_id=4,
                refresh_callback_end_steady_s=4.0,
                refresh_stamp_s=4.0,
                predictor_spatial_invalidation_reason="gnss_ttl_reuse",
                predictor_spatial_advisory_recompute_count=0,
                predictor_spatial_advisory_reuse_count=76800,
                predictor_gnss_advisory_invocation_count=0,
                predictor_lidar_advisory_invocation_count=0,
                predictor_spatial_retained_position_count=12800,
                predictor_spatial_entered_position_count=0,
                predictor_spatial_ttl_retained_position_count=12800,
                refresh_elapsed_ms=200.0,
            ),
            valid_p0_message(
                generation_id=5,
                refresh_callback_end_steady_s=5.0,
                refresh_stamp_s=5.0,
                predictor_spatial_invalidation_reason="occupancy_full_rebuild",
                predictor_spatial_full_invalidation_count=1,
                refresh_elapsed_ms=250.0,
            ),
            valid_p0_message(
                generation_id=6,
                refresh_callback_end_steady_s=6.0,
                refresh_stamp_s=6.0,
                predictor_spatial_invalidation_reason="warm_empty_delta",
                predictor_spatial_advisory_recompute_count=0,
                predictor_spatial_advisory_reuse_count=76800,
                predictor_gnss_advisory_invocation_count=0,
                predictor_lidar_advisory_invocation_count=0,
                predictor_spatial_retained_position_count=12800,
                predictor_spatial_entered_position_count=0,
                predictor_spatial_exact_retained_position_count=12800,
                refresh_elapsed_ms=900.0,
            ),
        ]

        rows, summary = MODULE.analyze_p0_messages(messages, protocol="smoke")

        self.assertEqual([row["generation_id"] for row in rows], list(range(1, 7)))
        self.assertEqual(summary["successful_generation_count"], 6)
        self.assertEqual(summary["refresh_elapsed_ms_p50"], 175.0)
        self.assertEqual(summary["refresh_elapsed_ms_p95"], 737.5)
        self.assertEqual(summary["refresh_elapsed_ms_max"], 900.0)
        self.assertEqual(summary["gate"], "PASS")

    def test_p0_complete_type7_distribution_does_not_trim_tail_or_outlier(self):
        messages = [
            valid_p0_message(
                generation_id=generation,
                refresh_callback_end_steady_s=float(generation),
                refresh_stamp_s=float(generation),
                refresh_elapsed_ms=(1000.0 if generation == 20 else float(generation)),
            )
            for generation in range(1, 21)
        ]

        rows, summary = MODULE.analyze_p0_messages(messages, protocol="benchmark")

        self.assertEqual(len(rows), 20)
        self.assertEqual(summary["minimum_successful_generations"], 20)
        self.assertEqual(summary["successful_generation_count"], 20)
        self.assertEqual(summary["refresh_elapsed_ms_p50"], 10.5)
        self.assertAlmostEqual(summary["refresh_elapsed_ms_p95"], 68.05)
        self.assertEqual(summary["refresh_elapsed_ms_max"], 1000.0)
        self.assertEqual(summary["gate"], "PASS")

    def test_malformed_ready_value_is_not_a_successful_generation(self):
        _, summary = MODULE.analyze_p0_messages(
            [valid_p0_message(ready="true")], protocol="smoke"
        )
        self.assertEqual(summary["successful_generation_count"], 0)
        self.assertIn("zero_successful_generations", summary["failures"])


if __name__ == "__main__":
    unittest.main()
