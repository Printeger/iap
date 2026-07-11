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


if __name__ == "__main__":
    unittest.main()
