import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts" / "dev_planner" / "run_icra_interface_integration.py"
SPEC = importlib.util.spec_from_file_location("icra_interface_integration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def healthy(generation, received):
    return {
        "kind": "p0_health",
        "receive_steady_s": received,
        "payload": {
            "ready": True,
            "stale": False,
            "reason": "ok",
            "generation_id": generation,
        },
    }


def selected_decision():
    return {
        "status": "RISK_SELECTED",
        "selection_applied": "1",
        "planning_attempt_id": "7",
        "collision_segment_id": "9",
        "request_hash": "request",
        "snapshot_generation_id": "5",
        "snapshot_config_hash": "config",
        "occupancy_epoch": "3",
        "original_hash": "original",
        "risk_hash": "risk",
        "selected_hash": "risk",
        "original_sample_count": "10",
        "original_valid_count": "10",
        "original_unknown_count": "0",
        "original_stale_count": "0",
        "original_non_finite_count": "0",
        "risk_sample_count": "10",
        "risk_valid_count": "10",
        "risk_unknown_count": "0",
        "risk_stale_count": "0",
        "risk_non_finite_count": "0",
    }


def lineage_for(decision, trajectory_id, start_ns):
    common = {
        "schema_version": "p4_v2_end_to_end_lineage_v2",
        "planning_attempt_id": decision["planning_attempt_id"],
        "collision_segment_id": decision["collision_segment_id"],
        "request_hash": decision["request_hash"],
        "snapshot_generation_id": decision["snapshot_generation_id"],
        "snapshot_config_hash": decision["snapshot_config_hash"],
        "occupancy_epoch": decision["occupancy_epoch"],
        "original_guide_hash": decision["original_hash"],
        "risk_guide_hash": decision["risk_hash"],
        "selected_guide_hash": decision["selected_hash"],
        "trajectory_id": str(trajectory_id),
        "trajectory_start_ns": str(start_ns),
        "control_points_hash": "points",
        "final_bspline_identity": "bspline",
        "selection_applied": "1",
    }
    return [
        {**common, "stage": stage, "stamp_s": str(10.0 + index)}
        for index, stage in enumerate((
            "final_bspline_before_p5",
            "p5_final_pass_before_publish",
            "normal_publish_authorized",
        ))
    ]


class TestStageContracts(unittest.TestCase):
    def test_stage_switches_keep_forbidden_layers_off(self):
        for stage in MODULE.STAGE_ORDER:
            spec = MODULE.STAGES[stage]
            self.assertEqual(spec.launch_args["planner_enable_p1"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p2"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p3_local"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p3_global"], "false")

        self.assertEqual(MODULE.STAGES["estimator"].duration_s, 20.0)
        self.assertEqual(MODULE.STAGES["p0"].duration_s, 35.0)
        self.assertEqual(MODULE.STAGES["p4"].duration_s, 45.0)
        self.assertEqual(MODULE.STAGES["p5-final"].duration_s, 60.0)
        self.assertEqual(MODULE.STAGES["full"].duration_s, 75.0)

        expected = {
            "estimator": ("false", "false", "false", "false"),
            "p0": ("true", "false", "false", "false"),
            "p4": ("true", "true", "false", "false"),
            "p5-final": ("true", "true", "true", "false"),
            "full": ("true", "true", "true", "true"),
            "shutdown": ("true", "true", "true", "true"),
        }
        keys = (
            "start_planner", "planner_enable_p4",
            "planner_enable_p5_final", "planner_enable_p5_runtime",
        )
        for stage, values in expected.items():
            self.assertEqual(
                tuple(MODULE.STAGES[stage].launch_args[key] for key in keys),
                values,
            )
            self.assertEqual(
                MODULE.STAGES[stage].launch_args["odometry_acc_scale"], "1.0")
            self.assertEqual(
                MODULE.STAGES[stage].launch_args[
                    "odometry_initialization_mode"], "NAIVE")

    def test_final_pass_is_reserved_for_full_plus_shutdown(self):
        self.assertEqual(MODULE._successful_session_result(("full",)),
                         "STAGE_PASS")
        self.assertEqual(MODULE._successful_session_result(("shutdown",)),
                         "STAGE_PASS")
        self.assertEqual(MODULE._successful_session_result(
            MODULE.STAGE_ORDER), "PASS")


class TestStageAnalyzer(unittest.TestCase):
    def test_p0_deadline_uses_launch_start_not_earlier_capture_start(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_root = Path(temporary_directory)
            (run_root / "capture_ready.json").write_text(json.dumps({
                "ready_steady_s": 85.0,
            }))
            (run_root / "launch_started.json").write_text(json.dumps({
                "started_steady_s": 85.2,
            }))
            self.assertEqual(MODULE._stage_start_steady_s(run_root), 85.2)

        rows = [healthy(index + 1, 100.1 + index * 0.5)
                for index in range(31)]
        summary = MODULE.analyze_p0(rows, 85.2)
        self.assertEqual(summary["result"], "PASS")
        self.assertAlmostEqual(summary["first_healthy_delay_s"], 14.9)

    def test_estimator_rejects_wrong_scale_and_large_state(self):
        summary = MODULE.analyze_estimator({
            "acc_scale": 9.80665,
            "rotation_deg": 103.9,
            "velocity_norm_mps": 23.5,
            "bias_norm": 1.4,
            "odom_count": 20,
            "position_error_p95_m": 0.01,
            "finite": True,
        })
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("acc_scale_not_si", summary["failures"])
        self.assertIn("initial_rotation_exceeded", summary["failures"])

    def test_p0_requires_an_unbroken_fifteen_second_healthy_window(self):
        rows = [healthy(index + 1, float(index)) for index in range(16)]
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")
        rows[8]["payload"]["stale"] = True
        rows[8]["payload"]["reason"] = "occupancy_stale"
        failed = MODULE.analyze_p0(rows)
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", failed["failures"])

    def test_p0_window_includes_first_observation_past_boundary(self):
        rows = [healthy(index + 1, index * 0.47) for index in range(34)]
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "PASS")
        self.assertGreaterEqual(summary["window_span_s"], 15.0)

    def test_p0_does_not_restart_after_first_healthy_generation(self):
        rows = [healthy(1, 0.0)]
        rows.append(healthy(1, 0.5))
        rows[-1]["payload"]["reason"] = "occupancy_stale"
        rows.extend(healthy(index + 2, 1.0 + index * 0.5)
                    for index in range(31))
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", summary["failures"])

    def test_p0_ignores_unhealthy_startup_before_first_generation(self):
        rows = [healthy(0, 0.0)]
        rows[0]["payload"].update({
            "ready": False, "stale": True, "reason": "startup",
        })
        rows.extend(healthy(index + 1, 1.0 + index * 0.5)
                    for index in range(31))
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")

    def test_p4_requires_selected_lineage_and_stable_publication(self):
        summary = MODULE.analyze_stage_records(
            "p4", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[], lineage=[], bsplines=[], p5_status=[], poscmd_times=[])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p4_risk_selected_missing", summary["failures"])
        self.assertIn("stable_bspline_missing", summary["failures"])

    def test_full_rejects_mixed_p5_identity_and_unsafe_runtime(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        statuses = [
            {"receive_steady_s": 19.0,
             "payload": {"phase": "final", "action": "OK", "reason": "ok",
                         "current_integrity_source": "FUSED",
                         "final_candidate_traj_id": 12,
                         "final_candidate_start_time_ns": 34,
                         "samples": []}},
            {"receive_steady_s": 30.0,
             "payload": {"phase": "runtime", "action": "REQUEST_REPLAN",
                         "raw_action": "REQUEST_REPLAN", "reason": "future_bad",
                         "raw_reason": "future_bad", "active_reasons": ["future_bad"],
                         "current_integrity_source": "FUSED",
                         "samples": [{"trajectory_sample_source": "runtime_committed",
                                      "trajectory_id": 99,
                                      "trajectory_start_time_ns": 34}]}}
        ]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses, poscmd_times=[25.0 + i * 0.01 for i in range(600)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_runtime_identity_or_status_invalid", summary["failures"])

    def test_full_reports_the_existing_seven_stage_chain(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
            row["no_collision_refinement_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        safe_common = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok",
            "active_reasons": [], "current_reason": "",
            "future_reason": "", "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 19.0,
            "payload": {
                **safe_common, "phase": "final",
                "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34,
                "samples": [],
            },
        }, {
            "receive_steady_s": 19.5,
            "payload": {
                **safe_common, "phase": "final_publish_authorized",
                "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34,
                "final_evaluation_stamp_s": 18.0,
                "final_publish_authorization_stamp_s": 19.0,
            },
        }, {
            "receive_steady_s": 21.0,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 12,
                    "trajectory_start_time_ns": 34,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "PASS")
        self.assertIsNone(summary["seven_stage"]["first_missing_stage"])
        self.assertTrue(all(summary["seven_stage"]["stage_status"].values()))
        self.assertEqual(summary["p5_runtime_committed_count"], 1)

    def test_full_rejects_capture_missing_selected_terminal_identity(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 13, "start_time_ns": 35}}
            for index in range(3)
        ]
        safe_common = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok",
            "active_reasons": [], "current_reason": "",
            "future_reason": "", "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 19.0,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 11,
                    "trajectory_start_time_ns": 33,
                }],
            },
        }, {
            "receive_steady_s": 19.5,
            "payload": {
                **safe_common, "phase": "final",
                "final_candidate_traj_id": 13,
                "final_candidate_start_time_ns": 35,
                "samples": [],
            },
        }, {
            "receive_steady_s": 20.1,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 13,
                    "trajectory_start_time_ns": 35,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_final_identity_or_status_invalid",
                      summary["failures"])
        self.assertIn("existing_seven_stage_analyzer_failed",
                      summary["failures"])

    def test_p5_final_ok_must_precede_same_identity_publication(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
            row["no_collision_refinement_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        safe = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok", "active_reasons": [],
            "current_reason": "", "future_reason": "",
            "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 29.0,
            "payload": {
                **safe, "phase": "final", "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34, "samples": [],
            },
        }, {
            "receive_steady_s": 30.0,
            "payload": {
                **safe, "phase": "runtime", "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 12, "trajectory_start_time_ns": 34,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_published_candidate_without_prior_final_ok",
                      summary["failures"])

    def test_shutdown_rejects_escalation_and_nonzero_process_exit(self):
        summary = MODULE.analyze_shutdown(
            launch_exit_code=0,
            stdout="failed to terminate after SIGINT, escalating to SIGTERM\n"
                   "process has died [exit code -11]",
            owned_group_cleared=True,
            residual_nodes=[],
        )
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("shutdown_escalated", summary["failures"])
        self.assertIn("child_process_nonzero", summary["failures"])

    def test_shutdown_rejects_runner_kill_escalation(self):
        summary = MODULE.analyze_shutdown(
            launch_exit_code=0,
            stdout="",
            owned_group_cleared=True,
            residual_nodes=[],
            runner_escalated=True,
        )
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("shutdown_escalated", summary["failures"])


if __name__ == "__main__":
    unittest.main()
