#!/usr/bin/env python3

import importlib.util
from pathlib import Path
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


def p5_row(**overrides):
    row = {
        "bag_time_s": 1.0,
        "phase": "runtime",
        "action": "OK",
        "raw_action": "OK",
        "reason": "OK",
        "current_reason": "",
        "future_reason": "",
        "active_reasons": [],
        "current_im_h": 5.0,
        "current_im_v": 5.5,
        "current_im_min": 5.0,
        "future_min_im": 4.5,
        "first_bad_tau": "nan",
        "bad_ratio": 0.0,
        "unknown_ratio": 0.0,
        "current_integrity_age_s": 0.1,
        "field_generation_id": 3,
        "field_age_s": 0.1,
        "current_stale_duration_s": 0.0,
        "current_low_margin_duration_s": 0.0,
        "future_unknown_duration_s": 0.0,
        "final_gate_fail_count": 0,
        "final_gate_fail_duration_s": 0.0,
        "final_gate_last_reason": "",
        "pred_al_mode": "current_msg_constant",
        "pred_hal_min": 10.0,
        "pred_val_min": 10.0,
        "pred_al_invalid_count": 0,
        "pred_al_last_reason": "",
        "sample_count": 10,
        "bad_count": 0,
        "unknown_count": 0,
        "parse_error": "",
    }
    row.update(overrides)
    return row


def healthy_p0():
    return {
        "row_count": 10,
        "stale_true_count": 0,
        "ready_false_count": 0,
        "reason_counts": {"ok": 10},
    }


def p5_manifest():
    return {
        "planner_safety_profile": "p5",
        "p0.enable_risk_grid": True,
        "planner_enable_p5_runtime": True,
        "planner_enable_p5_final": True,
        "planner_enable_p1": False,
        "planner_enable_p2": False,
        "planner_enable_p3_local": False,
        "planner_enable_p3_global": False,
        "planner_enable_p4": False,
    }


def p5_3_manifest(enabled=True):
    manifest = p5_manifest()
    manifest.update(
        {
            "p5.future_replan_margin_m": 0.3,
            "p5.max_bad_ratio": 0.25,
            "p5_3.fixture.enabled": enabled,
            "p5_3.fixture.name": "future_high_risk_zone_v1",
            "p5_3.fixture.x_min": -10.8,
            "p5_3.fixture.x_max": -8.7,
            "p5_3.fixture.y_min": -0.75,
            "p5_3.fixture.y_max": 0.75,
            "p5_3.fixture.z_min": 1.0,
            "p5_3.fixture.z_max": 1.35,
            "p5_3.fixture.tau_min": 1.2,
            "p5_3.fixture.tau_max": 2.0,
            "p5_3.fixture.hpl_pred_m": 10.2,
            "p5_3.fixture.vpl_pred_m": 10.2,
            "p5_3.fixture.expected_hal_m": 10.0,
            "p5_3.fixture.expected_val_m": 10.0,
            "p5_3.fixture.expected_im_m": -0.2,
            "p5_3": {
                "fixture": {
                    "enabled": enabled,
                    "name": "future_high_risk_zone_v1",
                    "bounds": {
                        "x": [-10.8, -8.7],
                        "y": [-0.75, 0.75],
                        "z": [1.0, 1.35],
                    },
                    "tau_window_s": [1.2, 2.0],
                    "injected_pl_m": {"hpl_pred": 10.2, "vpl_pred": 10.2},
                    "expected_alert_limit_m": {
                        "mode": "current_msg_constant",
                        "hal": 10.0,
                        "val": 10.0,
                    },
                    "expected_im_m": -0.2,
                    "expected_reason": "p5_3_high_risk_zone",
                }
            },
        }
    )
    return manifest


def p5_topic_health():
    return {
        topic: {"status": "PASS", "count": 1}
        for topic in analyzer.P5_TOPIC_EXPECTATIONS
    }


def p0_health_row(idx, **overrides):
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


def healthy_p0_rows():
    return [p0_health_row(idx) for idx in range(20)]


def bounded_startup_p0_rows():
    rows = [
        p0_health_row(
            idx,
            ready=False,
            stale=True,
            valid_ratio=0.0,
            unknown_ratio=1.0,
            reason="snapshot_unavailable",
        )
        for idx in range(2)
    ]
    rows.extend(p0_health_row(idx + 2) for idx in range(20))
    return rows


def startup_snapshot_unavailable_row(**overrides):
    row = p5_row(
        bag_time_s=0.0,
        phase="final",
        action="REQUEST_REPLAN",
        raw_action="REQUEST_REPLAN",
        reason="snapshot_unavailable",
        current_im_h="",
        current_im_v="",
        current_im_min="",
        future_min_im="",
        unknown_ratio=1.0,
        sample_count=1,
        unknown_count=1,
    )
    row.update(overrides)
    return row


def p5_3_marker_row(**overrides):
    row = {
        "bag_time_s": 10.0,
        "topic": analyzer.P5_TRAJECTORY_SAMPLES_TOPIC,
        "marker_ns": "trajectory_integrity_samples",
        "marker_id": 1,
        "marker_type": 4,
        "marker_action": 0,
        "point_index": 0,
        "x": -10.2,
        "y": 0.0,
        "z": 1.2,
        "color_r": 1.0,
        "color_g": 0.0,
        "color_b": 0.0,
        "color_a": 1.0,
        "state": "bad",
        "text": "",
    }
    row.update(overrides)
    return row


def p5_3_sample(**overrides):
    row = {
        "tau_s": 0.0,
        "query_tau_s": 0.0,
        "x": -12.0,
        "y": 0.0,
        "z": 1.2,
        "hpl": 1.0,
        "vpl": 1.0,
        "hal": 10.0,
        "val": 10.0,
        "im_min": 9.0,
        "bad": False,
        "unknown": False,
        "stale": False,
        "reason": "ok",
    }
    row.update(overrides)
    if "query_tau_s" not in overrides:
        row["query_tau_s"] = row["tau_s"]
    return row


def p5_3_future_only_samples(linked=True):
    reason = (
        "future_low_margin:p5_3_high_risk_zone"
        if linked
        else "low_margin_without_fixture_source"
    )
    return [
        p5_3_sample(),
        p5_3_sample(
            tau_s=1.2,
            x=-10.5,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
        ),
        p5_3_sample(
            tau_s=1.4,
            x=-10.1,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
        ),
        p5_3_sample(
            tau_s=1.6,
            x=-9.7,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
        ),
    ]


def p5_3_replan_row(**overrides):
    row = p5_row(
        bag_time_s=10.0,
        action="REQUEST_REPLAN",
        raw_action="REQUEST_REPLAN",
        reason="future_bad",
        future_min_im=-0.2,
        first_bad_tau=1.2,
        bad_ratio=0.3,
        unknown_ratio=0.0,
        pred_hal_min=10.0,
        pred_val_min=10.0,
        sample_count=10,
        bad_count=3,
        unknown_count=0,
        samples=p5_3_future_only_samples(),
    )
    row.update(overrides)
    return row


class P5_1AnalyzerTest(unittest.TestCase):
    def validate_rows(self, rows, p0_summary=None):
        p5_summary = analyzer.summarize_p5_status_rows(rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_1_hard_gates(
            p0_summary or healthy_p0(),
            p5_summary,
            failures,
            inconclusive,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_1_passes_with_ok_actions_and_zero_final_gate_failures(self):
        rows = [p5_row(bag_time_s=1.0 + idx) for idx in range(20)]

        summary, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertEqual(summary["action_counts"], {"OK": 20})
        self.assertEqual(summary["final_gate_fail_count_max"], 0)

    def test_p5_1_fails_on_emergency_action(self):
        rows = [p5_row(), p5_row(action="REQUEST_EMERGENCY_STOP_CANDIDATE")]

        _, _, failures, _ = self.validate_rows(rows)

        self.assertTrue(
            any("REQUEST_EMERGENCY_STOP_CANDIDATE" in failure for failure in failures),
            failures,
        )

    def test_p5_1_fails_on_repeated_replan_storm(self):
        rows = [
            p5_row(action="REQUEST_REPLAN", raw_action="REQUEST_REPLAN", bag_time_s=idx)
            for idx in range(analyzer.P5_REPLAN_STORM_CONSECUTIVE)
        ]

        summary, _, failures, _ = self.validate_rows(rows)

        self.assertTrue(summary["replan_storm"])
        self.assertTrue(any("replan storm" in failure for failure in failures), failures)

    def test_p5_1_allows_bounded_startup_snapshot_unavailable_replan_prefix(self):
        rows = [
            startup_snapshot_unavailable_row(bag_time_s=0.001 * idx)
            for idx in range(analyzer.P5_REPLAN_STORM_CONSECUTIVE + 2)
        ]
        rows.extend(p5_row(bag_time_s=1.0 + idx) for idx in range(20))

        summary, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertEqual(
            summary["startup_snapshot_unavailable_rows"],
            analyzer.P5_REPLAN_STORM_CONSECUTIVE + 2,
        )
        self.assertTrue(summary["startup_snapshot_unavailable_bounded"])
        self.assertEqual(summary["steady_action_counts"], {"OK": 20})
        self.assertEqual(gates["ok_action_ratio"], 1.0)

    def test_p5_1_allows_startup_snapshot_unavailable_timing_jitter(self):
        rows = [
            startup_snapshot_unavailable_row(bag_time_s=0.0),
            startup_snapshot_unavailable_row(
                bag_time_s=analyzer.P5_1_STARTUP_REPLAN_MAX_DURATION_S + 0.01
            ),
        ]
        rows.extend(p5_row(bag_time_s=3.0 + idx) for idx in range(20))

        summary, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(summary["startup_snapshot_unavailable_bounded"])

    def test_p5_1_fails_on_long_startup_snapshot_unavailable_prefix(self):
        rows = [
            startup_snapshot_unavailable_row(bag_time_s=idx)
            for idx in range(analyzer.P5_REPLAN_STORM_CONSECUTIVE + 1)
        ]
        rows.extend(p5_row(bag_time_s=10.0 + idx) for idx in range(20))

        summary, _, failures, _ = self.validate_rows(rows)

        self.assertFalse(summary["startup_snapshot_unavailable_bounded"])
        self.assertTrue(
            any("startup snapshot_unavailable replan prefix" in failure for failure in failures),
            failures,
        )

    def test_p5_1_fails_on_nonzero_final_gate_fail_count(self):
        rows = [p5_row(final_gate_fail_count=1, final_gate_fail_duration_s=0.1)]

        summary, _, failures, _ = self.validate_rows(rows)

        self.assertEqual(summary["final_gate_fail_count_max"], 1)
        self.assertTrue(any("final_gate_fail_count" in failure for failure in failures), failures)

    def test_p5_1_fails_on_p0_stale_health(self):
        rows = [p5_row()]

        _, _, failures, _ = self.validate_rows(
            rows,
            {"row_count": 3, "stale_true_count": 1},
        )

        self.assertTrue(any("P0 health reported stale=true" in failure for failure in failures), failures)

    def test_p5_1_allows_bounded_startup_snapshot_unavailable(self):
        rows = [p5_row(bag_time_s=1.0 + idx) for idx in range(20)]

        _, gates, failures, inconclusive = self.validate_rows(
            rows,
            {
                "row_count": 46,
                "ready_false_count": 3,
                "ready_false_ratio": 3 / 46,
                "ready_false_max_consecutive": 3,
                "stale_true_count": 3,
                "stale_true_ratio": 3 / 46,
                "stale_true_max_consecutive": 3,
                "reason_counts": {"snapshot_unavailable": 3, "ok": 43},
            },
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["p0_non_stale"])
        self.assertTrue(gates["p0_startup_unavailable_explained"])

    def test_p5_1_fails_on_missing_p5_required_topic(self):
        topic_counts = {topic: 1 for topic in analyzer.P5_TOPIC_EXPECTATIONS}
        topic_counts[analyzer.P5_STATUS_TOPIC] = 0
        metadata = {
            "missing": False,
            "duration_ns": int(60.0 * 1.0e9),
            "topic_counts": topic_counts,
        }
        failures = []
        inconclusive = []

        analyzer.validate_topic_health(
            metadata,
            {},
            "",
            failures,
            inconclusive,
            analyzer.P5_TOPIC_EXPECTATIONS,
        )

        self.assertTrue(
            any(analyzer.P5_STATUS_TOPIC in failure for failure in failures),
            failures,
        )


class P5_2AnalyzerTest(unittest.TestCase):
    def validate_rows(self, rows, p0_rows=None, topic_health=None):
        p0_rows = p0_rows or bounded_startup_p0_rows()
        p5_summary = analyzer.summarize_p5_status_rows(rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_2_hard_gates(
            p5_manifest(),
            {"passed": True},
            topic_health or p5_topic_health(),
            analyzer.summarize_p0_health(p0_rows),
            p0_rows,
            p5_summary,
            failures,
            inconclusive,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_2_passes_with_replan_rows_and_finite_margins(self):
        rows = [p5_row(bag_time_s=1.0 + idx) for idx in range(5)]
        rows.extend(
            [
                p5_row(
                    bag_time_s=10.0,
                    action="REQUEST_REPLAN",
                    raw_action="REQUEST_REPLAN",
                    reason="future_unknown",
                    unknown_ratio=0.2,
                    future_unknown_duration_s=0.3,
                ),
                p5_row(
                    bag_time_s=10.1,
                    action="REQUEST_REPLAN",
                    raw_action="REQUEST_REPLAN",
                    reason="future_unknown",
                    unknown_ratio=0.2,
                    future_unknown_duration_s=0.4,
                ),
            ]
        )
        rows.extend(p5_row(bag_time_s=12.0 + idx) for idx in range(5))

        summary, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertEqual(summary["max_consecutive_replan"], 2)
        self.assertEqual(summary["final_gate_fail_count_max"], 0)

    def test_p5_2_allows_missing_planner_dependent_bspline_topic(self):
        topic_health = p5_topic_health()
        topic_health["/drone_0_planning/bspline"] = {"status": "FAIL", "count": 0}

        _, gates, failures, inconclusive = self.validate_rows(
            [p5_row(bag_time_s=1.0 + idx) for idx in range(5)],
            topic_health=topic_health,
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["required_p5_topics_stable"])
        self.assertTrue(gates["passed"])

    def test_planner_dependent_bspline_missing_is_not_a_topic_failure(self):
        topic_counts = {topic: 1 for topic in analyzer.P5_TOPIC_EXPECTATIONS}
        topic_counts["/drone_0_planning/bspline"] = 0
        metadata = {
            "missing": False,
            "duration_ns": int(60.0 * 1.0e9),
            "topic_counts": topic_counts,
        }
        failures = []
        inconclusive = []
        timings = {
            "/iap/integrity": {"span_s": 59.0, "max_gap_s": 0.1},
            "/sim/drone_0/lidar_body": {"span_s": 59.0, "max_gap_s": 0.1},
            "/drone_0_visual_slam/odom": {"span_s": 59.0, "max_gap_s": 0.1},
            "/planning/risk_grid_health": {"span_s": 59.0, "max_gap_s": 0.5},
        }

        topic_health = analyzer.validate_topic_health(
            metadata,
            timings,
            "",
            failures,
            inconclusive,
            analyzer.P5_TOPIC_EXPECTATIONS,
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertEqual("FAIL", topic_health["/drone_0_planning/bspline"]["status"])

    def test_p5_2_fails_on_sustained_emergency_storm(self):
        rows = [
            p5_row(action="REQUEST_EMERGENCY_STOP_CANDIDATE", bag_time_s=idx)
            for idx in range(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE)
        ]

        summary, gates, failures, _ = self.validate_rows(rows)

        self.assertEqual(
            analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE,
            summary["max_consecutive_emergency"],
        )
        self.assertFalse(gates["passed"])
        self.assertTrue(any("emergency storm" in failure for failure in failures), failures)

    def test_p5_2_fails_on_unexplained_unknown(self):
        rows = [p5_row(unknown_ratio=0.25, reason="ok")]

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertTrue(any("unknown_ratio was nonzero" in failure for failure in failures), failures)

    def test_p5_2_fails_on_p0_post_startup_stale_or_full_unknown(self):
        p0_rows = bounded_startup_p0_rows()
        p0_rows.append(
            p0_health_row(
                30,
                stale=True,
                valid_ratio=0.0,
                unknown_ratio=1.0,
                reason="ok",
            )
        )

        _, gates, failures, _ = self.validate_rows([p5_row()], p0_rows=p0_rows)

        self.assertFalse(gates["passed"])
        self.assertTrue(any("stale=true after startup" in failure for failure in failures), failures)
        self.assertTrue(any("full unknown after startup" in failure for failure in failures), failures)

    def test_p5_2_fails_on_final_gate_fail_count_accumulation(self):
        rows = [p5_row(final_gate_fail_count=3, final_gate_fail_duration_s=0.2)]

        summary, gates, failures, _ = self.validate_rows(rows)

        self.assertEqual(3, summary["final_gate_fail_count_max"])
        self.assertFalse(gates["passed"])
        self.assertTrue(any("final_gate_fail_count" in failure for failure in failures), failures)

    def test_p5_2_next_branch(self):
        self.assertEqual(
            "PASS -> P5-3",
            analyzer.next_debug_branch("PASS", [], [], "P5-2"),
        )
        self.assertEqual(
            "debug PL/AL margin",
            analyzer.next_debug_branch("FAIL", ["P5-2 failed"], [], "P5-2"),
        )


class P5_3AnalyzerTest(unittest.TestCase):
    def validate_rows(self, rows, manifest=None, marker_rows=None, topic_timestamps=None):
        p5_summary = analyzer.summarize_p5_status_rows(rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_3_hard_gates(
            manifest if manifest is not None else p5_3_manifest(),
            {"passed": True},
            p5_topic_health(),
            analyzer.summarize_p0_health(bounded_startup_p0_rows()),
            bounded_startup_p0_rows(),
            rows,
            p5_summary,
            marker_rows if marker_rows is not None else [p5_3_marker_row()],
            failures,
            inconclusive,
            topic_timestamps,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_3_passes_with_fixture_overlap_and_future_bad_replan(self):
        rows = [p5_row(bag_time_s=idx) for idx in range(3)]
        rows.append(p5_3_replan_row())

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["trajectory_overlap_present"])
        self.assertTrue(gates["overlap_margin_evidence"])
        self.assertTrue(gates["future_replan_reason_ok"])
        self.assertTrue(gates["current_sample_outside_fixture"])
        self.assertTrue(gates["current_sample_not_fixture_bad"])
        self.assertTrue(gates["future_sample_inside_fixture"])
        self.assertTrue(gates["future_bad_sample_inside_fixture"])
        self.assertTrue(gates["future_fixture_query_aligned"])

    def test_p5_3_query_alignment_uses_snapshot_relative_tau(self):
        samples = [
            p5_3_sample(),
            p5_3_sample(
                tau_s=1.2,
                query_tau_s=0.6,
                x=-10.5,
                hpl=1.0,
                vpl=1.0,
                im_min=9.0,
                bad=False,
                reason="ok",
            ),
            p5_3_sample(
                tau_s=1.8,
                query_tau_s=1.2,
                x=-9.3,
                hpl=10.2,
                vpl=10.2,
                im_min=-0.2,
                bad=True,
                reason="future_low_margin:p5_3_high_risk_zone",
            ),
        ]

        _, gates, failures, inconclusive = self.validate_rows(
            [p5_3_replan_row(samples=samples)]
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertEqual(1, gates["sample_summary"]["future_fixture_sample_count"])
        self.assertEqual(0, gates["sample_summary"]["future_query_mismatch_sample_count"])
        self.assertTrue(gates["future_fixture_query_aligned"])

    def test_p5_3_accepts_future_reason_for_replan_evidence(self):
        rows = [
            p5_3_replan_row(
                reason="current_low_margin",
                current_reason="current_low_margin",
                future_reason="future_bad",
            )
        ]

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["future_replan_reason_ok"])

    def test_p5_3_accepts_active_reasons_for_replan_evidence(self):
        rows = [
            p5_3_replan_row(
                reason="current_low_margin",
                current_reason="current_low_margin",
                active_reasons=["current_low_margin", "future_bad"],
            )
        ]

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["future_replan_reason_ok"])

    def test_p5_3_accepts_same_row_sample_link_for_replan_evidence(self):
        rows = [
            p5_3_replan_row(
                reason="current_low_margin",
                current_reason="current_low_margin",
                future_reason="",
                active_reasons=["current_low_margin"],
            )
        ]

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["future_replan_sample_link_ok"])
        self.assertTrue(gates["future_replan_reason_ok"])

    def test_p5_3_blocks_when_fixture_manifest_is_missing_or_disabled(self):
        _, gates, failures, inconclusive = self.validate_rows(
            [p5_3_replan_row()],
            manifest=p5_manifest(),
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertFalse(gates["passed"])
        self.assertTrue(gates["blocked_scenario_missing"])

        _, disabled_gates, _, _ = self.validate_rows(
            [p5_3_replan_row()],
            manifest=p5_3_manifest(enabled=False),
        )
        self.assertTrue(disabled_gates["blocked_scenario_missing"])

    def test_p5_3_blocks_when_query_alignment_evidence_is_missing(self):
        row = p5_3_replan_row(samples=[])

        _, gates, failures, inconclusive = self.validate_rows(
            [row],
            marker_rows=[],
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["blocked_scenario_missing"])
        self.assertTrue(
            any("per-sample status diagnostics are missing" in failure for failure in failures),
            failures,
        )
        self.assertTrue(
            any("trajectory marker evidence is missing" in item for item in inconclusive),
            inconclusive,
        )

    def test_p5_3_fails_when_overlap_has_no_replan(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(action="OK", raw_action="OK")]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(any("did not observe REQUEST_REPLAN" in failure for failure in failures), failures)

    def test_p5_3_fails_when_replan_has_no_future_reason_or_sample_link(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_3_replan_row(
                    reason="current_low_margin",
                    samples=p5_3_future_only_samples(linked=False),
                )
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["future_replan_reason_ok"])
        self.assertTrue(any("reason attribution" in failure for failure in failures), failures)

    def test_p5_3_visible_future_reason_without_replan_coincidence_is_not_attribution(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_3_replan_row(
                    reason="current_low_margin",
                    current_reason="current_low_margin",
                    future_reason="",
                    active_reasons=["current_low_margin"],
                    samples=p5_3_future_only_samples(linked=False),
                ),
                p5_3_replan_row(
                    bag_time_s=10.2,
                    action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                    raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                    reason="future_bad",
                    future_reason="future_bad",
                    active_reasons=["future_bad"],
                ),
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["request_replan_present"])
        self.assertTrue(gates["future_reason_attribution_ok"])
        self.assertFalse(gates["future_replan_reason_ok"])
        self.assertFalse(any("reason attribution" in failure for failure in failures), failures)
        self.assertTrue(any("PL/AL margin" in failure for failure in failures), failures)

    def test_p5_3_fails_scenario_isolation_when_bad_ratio_below_threshold(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_3_replan_row(
                    bad_ratio=0.2,
                    bad_count=2,
                    active_reasons=["future_bad"],
                )
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["future_bad_ratio_coverage"])
        self.assertTrue(any("future bad-ratio coverage" in failure for failure in failures), failures)

    def test_p5_3_fails_when_first_bad_tau_is_outside_fixture_window(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(first_bad_tau=0.0)]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(any("first_bad_tau" in failure for failure in failures), failures)

    def test_p5_3_fails_when_current_sample_is_inside_fixture(self):
        samples = p5_3_future_only_samples()
        samples[0] = p5_3_sample(
            tau_s=0.0,
            x=-10.2,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason="future_low_margin:p5_3_high_risk_zone",
        )

        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(samples=samples)]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["current_sample_outside_fixture"])
        self.assertFalse(gates["current_sample_not_fixture_bad"])
        self.assertTrue(any("current/tau=0" in failure for failure in failures), failures)

    def test_p5_3_fails_when_future_fixture_sample_query_pl_does_not_align(self):
        samples = p5_3_future_only_samples()
        samples[1] = p5_3_sample(
            tau_s=1.2,
            x=-10.5,
            hpl=1.0,
            vpl=1.0,
            im_min=9.0,
            bad=False,
            reason="ok",
        )

        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(samples=samples)]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["future_fixture_query_aligned"])
        self.assertEqual(
            1,
            gates["sample_summary"]["future_query_mismatch_sample_count"],
        )
        self.assertTrue(any("query alignment" in failure for failure in failures), failures)

    def test_p5_3_active_window_topic_gap_fails_on_odom_gap(self):
        rows = [
            p5_3_replan_row(bag_time_s=10.0),
            p5_3_replan_row(bag_time_s=13.0),
        ]
        topic_timestamps = {
            "/iap/integrity": [10.0, 11.0, 12.0, 13.0],
            "/sim/drone_0/lidar_body": [10.0, 11.0, 12.0, 13.0],
            "/drone_0_visual_slam/odom": [10.0, 13.0],
        }

        _, gates, failures, _ = self.validate_rows(
            rows,
            topic_timestamps=topic_timestamps,
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["active_required_p5_topics_stable"])
        self.assertEqual(
            "FAIL",
            gates["active_topic_gap"]["topic_statuses"][
                "/drone_0_visual_slam/odom"
            ]["status"],
        )
        self.assertTrue(any("active evidence-window topic gap" in failure for failure in failures), failures)

    def test_p5_3_fails_on_emergency_storm(self):
        rows = [p5_3_replan_row()]
        rows.extend(
            p5_3_replan_row(
                bag_time_s=11.0 + idx,
                action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            )
            for idx in range(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE)
        )

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertTrue(any("emergency storm" in failure for failure in failures), failures)

    def test_p5_3_next_branch(self):
        self.assertEqual(
            "PASS -> P5-4",
            analyzer.next_debug_branch("PASS", [], [], "P5-3"),
        )
        self.assertEqual(
            analyzer.P5_3_FAIL_BRANCH,
            analyzer.next_debug_branch("FAIL", ["P5-3 failed"], [], "P5-3"),
        )
        self.assertEqual(
            analyzer.P5_3_FAIL_BRANCH,
            analyzer.next_debug_branch(
                "FAIL",
                ["P5-3 scenario isolation / future bad-ratio coverage"],
                [],
                "P5-3",
            ),
        )
        self.assertEqual(
            analyzer.P5_3_FAIL_BRANCH,
            analyzer.next_debug_branch(
                "FAIL",
                ["P5-3 reason attribution"],
                [],
                "P5-3",
            ),
        )
        self.assertEqual(
            analyzer.P5_3_BLOCKED_BRANCH,
            analyzer.next_debug_branch("BLOCKED_SCENARIO_MISSING", [], [], "P5-3"),
        )

    def test_p5_3_plal_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_3_plal_scenario_topdown.png",
                "p5_3_plal_high_risk_overlay.png",
                "p5_3_plal_tau_window.png",
                "p5_3_plal_margin_timeline.png",
                "p5_3_plal_action_reason_timeline.png",
                "p5_3_plal_replan_vs_emergency.png",
                "p5_3_plal_sample_heatmap.png",
                "p5_3_plal_p0_health_timeline.png",
            ],
            analyzer.P5_3_PLAL_FIGURE_FILENAMES,
        )

    def test_p5_3_query_alignment_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_3_query_alignment_scenario_topdown.png",
                "p5_3_query_alignment_fixture_overlay.png",
                "p5_3_query_alignment_pl_probe.png",
                "p5_3_query_alignment_tau_window.png",
                "p5_3_query_alignment_margin_timeline.png",
                "p5_3_query_alignment_action_reason.png",
                "p5_3_query_alignment_sample_heatmap.png",
                "p5_3_query_alignment_topic_gap.png",
                "p5_3_query_alignment_p0_health.png",
            ],
            analyzer.P5_3_QUERY_ALIGNMENT_FIGURE_FILENAMES,
        )


if __name__ == "__main__":
    unittest.main()
