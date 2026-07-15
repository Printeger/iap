#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace
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


def p1_manifest(**overrides):
    manifest = {
        "experiment": "p1_degraded_lidar_good",
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "p1",
        "p0.enable_risk_grid": True,
        "planner_enable_p1": True,
        "p1.metrics_only": True,
        "p1.use_integrity_cost": True,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
        "planner_enable_p5_runtime": False,
        "planner_enable_p5_final": False,
    }
    manifest.update(overrides)
    return manifest


def baseline_manifest(**overrides):
    manifest = {
        "experiment": "p0_open_sky",
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "off",
        "p0.enable_risk_grid": True,
        "planner_enable_p1": False,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
        "planner_enable_p5_runtime": False,
        "planner_enable_p5_final": False,
    }
    manifest.update(overrides)
    return manifest


def topic_health(**overrides):
    health = {
        topic: {"status": "PASS", "count": 3}
        for topic in analyzer.P1_TOPIC_EXPECTATIONS
    }
    health.update(overrides)
    return health


def metadata():
    return {
        "missing": False,
        "topic_counts": {
            analyzer.P5_STATUS_TOPIC: 0,
            **{topic: 0 for topic in analyzer.P5_RVIZ_TOPICS},
            "/iap/rviz/p2_candidate_trajectories": 0,
            "/iap/rviz/p3_reference_bias": 0,
            "/iap/rviz/p4_astar_guides": 0,
        },
    }


def p0_row(idx, **overrides):
    row = {
        "stamp": float(idx),
        "ready": True,
        "stale": False,
        "valid_ratio": 1.0,
        "unknown_ratio": 0.0,
        "reason": "ok",
    }
    row.update(overrides)
    return row


def p1_debug_row(**overrides):
    row = {
        "stamp": 1.0,
        "lbfgs_iter": 1,
        "snapshot_generation_id": 2,
        "query_base_time_s": 1.0,
        "sample_count": 4,
        "hit_count": 3,
        "miss_count": 1,
        "stale_count": 0,
        "miss_ratio": 0.25,
        "stale_ratio": 0.0,
        "f_integrity": 1.2,
        "weighted_f_integrity": 0.0,
        "grad_norm_integrity": 0.1,
        "grad_norm_original": 2.0,
        "grad_ratio": 0.05,
        "clipped_grad_count": 0,
        "fallback_reason": "ok",
        "applied_to_objective": 0,
    }
    row.update(overrides)
    return row


def p1_debug_summary(rows=None, **kwargs):
    return analyzer.summarize_p1_integrity_debug_rows(
        [p1_debug_row()] if rows is None else rows,
        **kwargs,
    )


def bspline_rows(offset_y=0.0):
    return [
        {
            "bag_time_s": 1.0,
            "traj_id": 1,
            "start_time_s": 0.0,
            "duration_s": 3.0,
            "order": 3,
            "pos_pts_count": 3,
            "knots_count": 6,
            "pos_pts_xy": [(-1.0, offset_y), (0.0, offset_y), (1.0, offset_y)],
        }
    ]


def run_gate(
    *,
    manifest=None,
    validator=None,
    health=None,
    meta=None,
    p0_rows=None,
    debug_summary=None,
    p1_bspline=None,
    baseline_export=Path("/tmp/p0_export"),
    baseline_bag=Path("/tmp/p0_bag"),
    baseline_manifest_data=None,
    baseline_bspline=None,
):
    manifest = p1_manifest() if manifest is None else manifest
    validator = {"passed": True} if validator is None else validator
    health = topic_health() if health is None else health
    meta = metadata() if meta is None else meta
    p0_rows = [p0_row(idx) for idx in range(5)] if p0_rows is None else p0_rows
    p0_summary = analyzer.summarize_p0_health(p0_rows)
    debug_summary = p1_debug_summary() if debug_summary is None else debug_summary
    p1_bspline = bspline_rows() if p1_bspline is None else p1_bspline
    baseline_manifest_data = (
        baseline_manifest()
        if baseline_manifest_data is None
        else baseline_manifest_data
    )
    baseline_bspline = bspline_rows() if baseline_bspline is None else baseline_bspline
    comparison = analyzer.compare_final_bspline_paths(p1_bspline, baseline_bspline)
    failures = []
    inconclusive = []
    gates = analyzer.validate_p1_1_hard_gates(
        manifest,
        validator,
        health,
        meta,
        p0_summary,
        p0_rows,
        debug_summary,
        p1_bspline,
        "",
        baseline_export,
        baseline_bag,
        baseline_manifest_data,
        baseline_bspline,
        "",
        comparison,
        failures,
        inconclusive,
    )
    return gates, failures, inconclusive


class P1_1AnalyzerTest(unittest.TestCase):
    def test_p0_health_parser_reads_rviz_marker_text(self):
        msg = SimpleNamespace(
            markers=[
                SimpleNamespace(
                    text=(
                        "RiskGridMap\n"
                        "gen: 42\n"
                        "age: 0.25 s\n"
                        "valid: 96.6%\n"
                        "unknown: 3.4%\n"
                        "status: READY\n"
                        "reason: ok"
                    )
                )
            ]
        )

        row = analyzer.parse_p0_health(msg, 1_500_000_000)

        self.assertEqual(row["stamp"], 1.5)
        self.assertTrue(row["ready"])
        self.assertFalse(row["stale"])
        self.assertAlmostEqual(row["valid_ratio"], 0.966)
        self.assertAlmostEqual(row["unknown_ratio"], 0.034)
        self.assertEqual(row["generation_id"], 42)
        self.assertTrue(row["snapshot_available"])
        self.assertEqual(row["reason"], "ok")

    def test_happy_path_passes_gate(self):
        gates, failures, inconclusive = run_gate()

        self.assertEqual(failures, [])
        self.assertEqual(inconclusive, [])
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["trajectory_vs_baseline_passed"])

    def test_manifest_gate_requires_metrics_only_p1(self):
        _, failures, _ = run_gate(
            manifest=p1_manifest(**{"planner_enable_p2": True})
        )

        self.assertTrue(any("manifest" in failure for failure in failures), failures)

    def test_missing_baseline_is_hard_failure(self):
        _, failures, _ = run_gate(
            baseline_export=None,
            baseline_bag=None,
            baseline_manifest_data={},
            baseline_bspline=[],
        )

        self.assertTrue(any("baseline" in failure for failure in failures), failures)

    def test_wrong_baseline_manifest_is_hard_failure(self):
        _, failures, _ = run_gate(
            baseline_manifest_data=baseline_manifest(
                scenario="other_scenario",
                planner_enable_p1=True,
            )
        )

        self.assertTrue(any("baseline manifest" in failure for failure in failures), failures)

    def test_missing_and_empty_p1_csv_fail(self):
        missing_summary = p1_debug_summary([], missing=True)
        _, missing_failures, _ = run_gate(debug_summary=missing_summary)

        empty_summary = p1_debug_summary([])
        _, empty_failures, _ = run_gate(debug_summary=empty_summary)

        self.assertTrue(any("missing" in failure for failure in missing_failures), missing_failures)
        self.assertTrue(any("no data rows" in failure for failure in empty_failures), empty_failures)

    def test_applied_to_objective_true_fails(self):
        debug_summary = p1_debug_summary([p1_debug_row(applied_to_objective=1)])

        _, failures, _ = run_gate(debug_summary=debug_summary)

        self.assertTrue(any("explicit false" in failure for failure in failures), failures)

    def test_applied_to_objective_invalid_fails(self):
        debug_summary = p1_debug_summary(
            [p1_debug_row(applied_to_objective="maybe")]
        )

        _, failures, _ = run_gate(debug_summary=debug_summary)

        self.assertEqual(debug_summary["applied_to_objective_invalid_count"], 1)
        self.assertTrue(any("not parseable" in failure for failure in failures), failures)
        self.assertTrue(any("explicit false" in failure for failure in failures), failures)

    def test_p0_health_failure_fails(self):
        p0_rows = [p0_row(0), p0_row(1, stale=True), p0_row(2)]

        _, failures, _ = run_gate(p0_rows=p0_rows)

        self.assertTrue(any("stale=true" in failure for failure in failures), failures)

    def test_validator_failure_fails(self):
        _, failures, _ = run_gate(validator={"passed": False})

        self.assertTrue(any("validator" in failure for failure in failures), failures)

    def test_p1_rviz_missing_fails(self):
        health = topic_health(
            **{
                analyzer.P1_METRICS_TOPIC: {"status": "FAIL", "count": 0},
            }
        )

        _, failures, _ = run_gate(health=health)

        self.assertTrue(any("RViz" in failure for failure in failures), failures)

    def test_bspline_missing_fails(self):
        _, failures, _ = run_gate(p1_bspline=[])

        self.assertTrue(any("bspline" in failure for failure in failures), failures)

    def test_trajectory_threshold_failure_fails(self):
        _, failures, _ = run_gate(p1_bspline=bspline_rows(offset_y=3.0))

        self.assertTrue(any("trajectory differs" in failure for failure in failures), failures)

    def test_next_branch_is_exact_for_p1_1(self):
        self.assertEqual(
            analyzer.next_debug_branch("PASS", [], [], "P1-1"),
            "PASS -> P1-2",
        )
        self.assertEqual(
            analyzer.next_debug_branch("FAIL", ["x"], [], "P1-1"),
            "FAIL -> debug metrics-only gate",
        )
        self.assertEqual(
            analyzer.next_debug_branch("INCONCLUSIVE", [], ["x"], "P1-1"),
            "FAIL -> debug metrics-only gate",
        )

    def test_required_figure_filenames_are_exact(self):
        self.assertEqual(
            analyzer.P1_1_FIGURE_FILENAMES,
            [
                "p1_1_scenario_topdown.png",
                "p1_1_topic_activity_timeline.png",
                "p1_1_p0_health.png",
                "p1_1_p1_metrics_timeline.png",
                "p1_1_integrity_cost_debug_summary.png",
                "p1_1_trajectory_overlay_vs_baseline.png",
                "p1_1_bspline_publish_timeline.png",
                "p1_1_manifest_switch_summary.png",
                "p1_1_validation_summary.png",
                "p1_1_cause_exclusion_summary.png",
            ],
        )


if __name__ == "__main__":
    unittest.main()
