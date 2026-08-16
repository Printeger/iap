import importlib.util
import math
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

    def test_type7_quantile_and_p0_deduplication_failure_gate(self):
        self.assertEqual(MODULE.type7_quantile([0.0, 10.0, 20.0, 30.0], 0.5), 15.0)
        self.assertAlmostEqual(
            MODULE.type7_quantile([0.0, 10.0, 20.0, 30.0], 0.95), 28.5
        )
        messages = []
        for generation in range(1, 22):
            messages.append({
                "generation_id": generation,
                "refresh_callback_end_steady_s": float(generation),
                "refresh_query_count": 76800,
                "provider_query_count": 76000,
                "predictor_unique_positions": 12800,
                "predictor_effective_worker_count": 1,
                "refresh_elapsed_ms": 100.0 + generation,
                "provider_batch_duration_ms": 80.0,
                "predictor_lidar_evaluations": 100,
                "predictor_lidar_cache_hits": 50,
                "ready": True,
                "stale": False,
                "valid_ratio": 1.0,
                "unknown_ratio": 0.0,
                "reason": "ok",
                "generation_interval_ms": 500.0,
                "refresh_stamp_s": float(generation),
            })
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


if __name__ == "__main__":
    unittest.main()
