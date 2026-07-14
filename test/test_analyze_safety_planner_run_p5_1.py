#!/usr/bin/env python3

import importlib.util
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


def p5_row(**overrides):
    row = {
        "bag_time_s": 1.0,
        "phase": "runtime",
        "action": "OK",
        "raw_action": "OK",
        "reason": "OK",
        "raw_reason": "OK",
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
        "fixture_match": False,
        "fixture_expected_hpl": "",
        "fixture_expected_vpl": "",
        "fixture_expected_reason": "",
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
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
        ),
        p5_3_sample(
            tau_s=1.4,
            x=-10.1,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
        ),
        p5_3_sample(
            tau_s=1.6,
            x=-9.7,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
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


def p5_4_manifest(enabled=True):
    manifest = p5_manifest()
    manifest.update(
        {
            "p5.future_emergency_margin_m": 0.0,
            "p5_4.fixture.enabled": enabled,
            "p5_4.fixture.name": "near_risk_zone_v1",
            "p5_4.fixture.x_min": -11.7,
            "p5_4.fixture.x_max": -11.1,
            "p5_4.fixture.y_min": -0.75,
            "p5_4.fixture.y_max": 0.75,
            "p5_4.fixture.z_min": 1.0,
            "p5_4.fixture.z_max": 1.35,
            "p5_4.fixture.tau_min": 0.6,
            "p5_4.fixture.tau_max": 0.95,
            "p5_4.fixture.hpl_pred_m": 10.2,
            "p5_4.fixture.vpl_pred_m": 10.2,
            "p5_4.fixture.expected_hal_m": 10.0,
            "p5_4.fixture.expected_val_m": 10.0,
            "p5_4.fixture.expected_im_m": -0.2,
            "p5_4": {
                "fixture": {
                    "enabled": enabled,
                    "name": "near_risk_zone_v1",
                    "bounds": {
                        "x": [-11.7, -11.1],
                        "y": [-0.75, 0.75],
                        "z": [1.0, 1.35],
                    },
                    "tau_window_s": [0.6, 0.95],
                    "injected_pl_m": {"hpl_pred": 10.2, "vpl_pred": 10.2},
                    "expected_alert_limit_m": {
                        "mode": "current_msg_constant",
                        "hal": 10.0,
                        "val": 10.0,
                    },
                    "expected_im_m": -0.2,
                    "expected_reason": "p5_4_near_risk_zone",
                    "expected_first_bad_tau_s": 0.6,
                    "expected_emergency_time_s": 1.0,
                }
            },
        }
    )
    return manifest


def p5_4_marker_row(**overrides):
    row = {
        "bag_time_s": 10.0,
        "topic": analyzer.P5_TRAJECTORY_SAMPLES_TOPIC,
        "marker_ns": "trajectory_integrity_samples",
        "marker_id": 1,
        "marker_type": 4,
        "marker_action": 0,
        "point_index": 0,
        "x": -11.4,
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


def p5_4_sample(**overrides):
    row = p5_3_sample()
    row.update(overrides)
    if "query_tau_s" not in overrides:
        row["query_tau_s"] = row["tau_s"]
    return row


def p5_4_future_only_samples(linked=True):
    reason = (
        "future_bad:p5_4_near_risk_zone"
        if linked
        else "low_margin_without_fixture_source"
    )
    return [
        p5_4_sample(),
        p5_4_sample(
            tau_s=0.6,
            x=-11.4,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_4_near_risk_zone",
        ),
        p5_4_sample(
            tau_s=0.8,
            x=-11.3,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason=reason,
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_4_near_risk_zone",
        ),
    ]


def p5_4_emergency_row(**overrides):
    row = p5_row(
        bag_time_s=10.0,
        action="REQUEST_EMERGENCY_STOP_CANDIDATE",
        raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
        reason="future_bad",
        future_reason="future_bad",
        active_reasons=["future_bad", "p5_4_near_risk_zone"],
        future_min_im=-0.2,
        first_bad_tau=0.6,
        bad_ratio=0.3,
        unknown_ratio=0.0,
        pred_hal_min=10.0,
        pred_val_min=10.0,
        sample_count=10,
        bad_count=2,
        unknown_count=0,
        samples=p5_4_future_only_samples(),
    )
    row.update(overrides)
    return row


def p5_5_manifest(enabled=True):
    manifest = p5_manifest()
    manifest.update(
        {
            "p5.current_stale_to_replan_s": 0.5,
            "p5.current_stale_to_emergency_s": 2.0,
            "p5_5.fixture.enabled": enabled,
            "p5_5.fixture.name": "current_integrity_stamp_freeze_v1",
            "p5_5.fixture.start_s": 30.0,
            "p5_5.fixture.duration_s": 12.0,
            "p5_5": {
                "fixture": {
                    "enabled": enabled,
                    "name": "current_integrity_stamp_freeze_v1",
                    "start_s": 30.0,
                    "duration_s": 12.0,
                    "window_s": [30.0, 42.0],
                    "expected_thresholds_s": {
                        "replan": 0.5,
                        "emergency": 2.0,
                    },
                    "expected_reason": "current_stale",
                }
            },
        }
    )
    return manifest


def p5_5_integrity_rows(
    *,
    freeze=True,
    entered=True,
    finite_pl_al=True,
):
    rows = []
    if not entered:
        rel_times = [0.0, 1.0, 2.0]
    else:
        rel_times = [29.5, 30.0, 30.5, 31.5, 32.5, 42.5]
    for rel in rel_times:
        in_window = 30.0 <= rel <= 42.0
        header_rel = 30.0 if freeze and in_window else rel
        hpl = 1.0 if finite_pl_al else ""
        vpl = 1.0 if finite_pl_al else ""
        hal = 10.0 if finite_pl_al else ""
        val = 10.0 if finite_pl_al else ""
        rows.append(
            {
                "bag_time_s": 100.0 + rel,
                "t_rel_s": rel,
                "header_stamp_s": 100.0 + header_rel,
                "header_rel_s": header_rel,
                "bag_minus_header_s": rel - header_rel,
                "in_expected_window": int(in_window),
                "hpl": hpl,
                "vpl": vpl,
                "hal": hal,
                "val": val,
                "im": 9.0 if finite_pl_al else "",
                "pl_al_finite": int(finite_pl_al),
                "fusion_mode": "fallback_only",
                "final_hpl_source": "FALLBACK",
                "final_vpl_source": "FALLBACK",
            }
        )
    return rows


def p5_5_status_rows():
    return [
        p5_row(bag_time_s=129.9, current_integrity_age_s=0.1),
        p5_row(
            bag_time_s=130.1,
            current_integrity_age_s=0.6,
            current_stale_duration_s=0.1,
        ),
        p5_row(
            bag_time_s=130.8,
            action="REQUEST_REPLAN",
            raw_action="REQUEST_REPLAN",
            reason="current_stale",
            raw_reason="current_stale",
            current_reason="current_stale",
            active_reasons=["current_stale"],
            current_integrity_age_s=1.3,
            current_stale_duration_s=0.8,
        ),
        p5_row(
            bag_time_s=132.2,
            action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            reason="current_stale",
            raw_reason="current_stale",
            current_reason="current_stale",
            active_reasons=["current_stale"],
            current_integrity_age_s=2.7,
            current_stale_duration_s=2.2,
        ),
    ]


def p5_6_manifest(p5_5_enabled=False, nested_enabled=None):
    nested = p5_5_enabled if nested_enabled is None else nested_enabled
    manifest = p5_manifest()
    manifest.update(
        {
            "p5.future_unknown_to_emergency_s": 2.0,
            "p5_3.fixture.enabled": False,
            "p5_4.fixture.enabled": False,
            "p5_5.fixture.enabled": p5_5_enabled,
            "p5_5.fixture.name": "current_integrity_stamp_freeze_v1",
            "p5_6.fixture.enabled": True,
            "p5_6.fixture.effective_enabled": not p5_5_enabled,
            "p5_6.fixture.name": "future_unknown_zone_v1",
            "p5_6.fixture.x_min": -1.0,
            "p5_6.fixture.x_max": 12.5,
            "p5_6.fixture.y_min": -15.0,
            "p5_6.fixture.y_max": 15.0,
            "p5_6.fixture.z_min": -3.0,
            "p5_6.fixture.z_max": 3.0,
            "p5_6.fixture.tau_min": 0.2,
            "p5_6.fixture.tau_max": 2.0,
            "p5_5": {
                "fixture": {
                    "enabled": nested,
                    "name": "current_integrity_stamp_freeze_v1",
                }
            },
            "p5_6": {
                "fixture": {
                    "enabled": True,
                    "effective_enabled": not p5_5_enabled,
                    "name": "future_unknown_zone_v1",
                    "bounds": {
                        "x": [-1.0, 12.5],
                        "y": [-15.0, 15.0],
                        "z": [-3.0, 3.0],
                    },
                    "tau_window_s": [0.2, 2.0],
                    "expected_reason": "future_unknown",
                }
            },
        }
    )
    return manifest


def p5_6_p0_rows():
    return [
        p0_health_row(0, unknown_ratio=0.0, valid_ratio=1.0),
        p0_health_row(1, unknown_ratio=0.05, valid_ratio=0.95),
        p0_health_row(2, unknown_ratio=0.35, valid_ratio=0.65),
        p0_health_row(3, unknown_ratio=0.45, valid_ratio=0.55),
    ]


def p5_6_status_rows():
    unknown_samples = [
        p5_3_sample(tau_s=0.0, x=-12.0, y=0.0, z=1.2),
        p5_3_sample(
            tau_s=0.2,
            x=0.0,
            y=0.0,
            z=1.2,
            hpl="",
            vpl="",
            im_min="",
            unknown=True,
            reason="future_unknown",
            fixture_match=True,
            fixture_expected_hpl="",
            fixture_expected_vpl="",
            fixture_expected_reason="future_unknown",
        ),
        p5_3_sample(
            tau_s=0.8,
            x=1.0,
            y=0.0,
            z=1.2,
            hpl="",
            vpl="",
            im_min="",
            unknown=True,
            reason="future_unknown",
            fixture_match=True,
            fixture_expected_hpl="",
            fixture_expected_vpl="",
            fixture_expected_reason="future_unknown",
        ),
    ]
    return [
        p5_row(bag_time_s=9.0, unknown_ratio=0.0, future_unknown_duration_s=0.0),
        p5_row(
            bag_time_s=10.0,
            action="OK",
            raw_action="OK",
            reason="future_unknown",
            raw_reason="future_unknown",
            future_reason="future_unknown",
            active_reasons=["future_unknown"],
            unknown_ratio=0.35,
            bad_ratio=0.0,
            future_unknown_duration_s=0.2,
            unknown_count=4,
            bad_count=0,
            samples=unknown_samples,
        ),
        p5_row(
            bag_time_s=10.8,
            action="REQUEST_REPLAN",
            raw_action="REQUEST_REPLAN",
            reason="future_unknown",
            raw_reason="future_unknown",
            future_reason="future_unknown",
            active_reasons=["future_unknown"],
            unknown_ratio=0.45,
            bad_ratio=0.0,
            future_unknown_duration_s=0.8,
            unknown_count=5,
            bad_count=0,
            samples=unknown_samples,
        ),
        p5_row(
            bag_time_s=12.4,
            action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            reason="final_gate_failed",
            raw_reason="future_unknown",
            future_reason="future_unknown",
            active_reasons=["future_unknown"],
            final_gate_fail_count=2,
            final_gate_fail_duration_s=2.0,
            final_gate_last_reason="future_unknown",
            unknown_ratio=0.50,
            bad_ratio=0.0,
            future_unknown_duration_s=2.4,
            unknown_count=5,
            bad_count=0,
            samples=unknown_samples,
        ),
    ]


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
    def validate_rows(
        self,
        rows,
        manifest=None,
        marker_rows=None,
        topic_timestamps=None,
        topic_health=None,
    ):
        p5_summary = analyzer.summarize_p5_status_rows(rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_3_hard_gates(
            manifest if manifest is not None else p5_3_manifest(),
            {"passed": True},
            topic_health if topic_health is not None else p5_topic_health(),
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
        self.assertTrue(gates["event_window_available"])
        self.assertEqual(3, gates["event_window_future_query_aligned_sample_count"])
        self.assertEqual(1.2, gates["event_window_first_bad_tau"])
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
                fixture_match=True,
                fixture_expected_hpl=10.2,
                fixture_expected_vpl=10.2,
                fixture_expected_reason="p5_3_high_risk_zone",
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

    def test_p5_3_requires_future_reason_on_event_window_anchor(self):
        rows = [
            p5_3_replan_row(
                reason="current_low_margin",
                current_reason="current_low_margin",
                future_reason="",
                active_reasons=["current_low_margin"],
            )
        ]

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], inconclusive)
        self.assertFalse(gates["passed"])
        self.assertFalse(gates["event_window_available"])
        self.assertTrue(gates["future_replan_sample_link_ok"])
        self.assertFalse(gates["future_replan_reason_ok"])
        self.assertTrue(any("event-window acceptance" in failure for failure in failures), failures)

    def test_p5_3_accepts_final_candidate_fixture_evidence_when_bspline_missing(self):
        topic_health = p5_topic_health()
        topic_health["/drone_0_planning/bspline"] = {
            "status": "FAIL",
            "count": 0,
        }

        _, gates, failures, inconclusive = self.validate_rows(
            [p5_3_replan_row(phase="final")],
            topic_health=topic_health,
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertEqual(0, gates["bspline_count"])
        self.assertTrue(gates["final_candidate_fixed_fixture_evidence"])
        self.assertTrue(gates["bspline_zero_acceptable"])

    def test_p5_3_rejects_bspline_missing_without_final_candidate_evidence(self):
        topic_health = p5_topic_health()
        topic_health["/drone_0_planning/bspline"] = {
            "status": "FAIL",
            "count": 0,
        }

        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(phase="runtime")],
            topic_health=topic_health,
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["bspline_zero_acceptable"])
        self.assertFalse(gates["final_candidate_fixed_fixture_evidence"])
        self.assertTrue(any("bspline=0" in failure for failure in failures), failures)

    def test_p5_3_fails_when_fixture_manifest_is_missing_or_disabled(self):
        _, gates, failures, inconclusive = self.validate_rows(
            [p5_3_replan_row()],
            manifest=p5_manifest(),
        )

        self.assertTrue(failures)
        self.assertEqual([], inconclusive)
        self.assertFalse(gates["passed"])
        self.assertTrue(gates["blocked_scenario_missing"])

        _, disabled_gates, _, _ = self.validate_rows(
            [p5_3_replan_row()],
            manifest=p5_3_manifest(enabled=False),
        )
        self.assertTrue(disabled_gates["blocked_scenario_missing"])

    def test_p5_3_fails_when_query_alignment_evidence_is_missing(self):
        row = p5_3_replan_row(samples=[])

        _, gates, failures, inconclusive = self.validate_rows(
            [row],
            marker_rows=[],
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["blocked_scenario_missing"])
        self.assertTrue(
            any("per-sample status diagnostics are missing" in failure for failure in failures),
            failures,
        )
        self.assertTrue(
            any("trajectory marker evidence is missing" in failure for failure in failures),
            failures,
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
        self.assertTrue(any("event-window acceptance" in failure for failure in failures), failures)

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

    def test_p5_3_fails_when_first_bad_tau_is_absent(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(first_bad_tau="nan")]
        )

        self.assertFalse(gates["passed"])
        self.assertIsNone(gates["event_window_first_bad_tau"])
        self.assertFalse(gates["event_window_first_bad_tau_in_fixture_window"])
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
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
        )

        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(samples=samples)]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["current_sample_outside_fixture"])
        self.assertFalse(gates["current_sample_not_fixture_bad"])
        self.assertTrue(any("current/tau=0" in failure for failure in failures), failures)

    def test_p5_3_passes_when_later_full_run_current_contamination_is_outside_event_window(self):
        contaminated_samples = p5_3_future_only_samples()
        contaminated_samples[0] = p5_3_sample(
            tau_s=0.0,
            x=-10.2,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason="future_low_margin:p5_3_high_risk_zone",
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
        )
        rows = [
            p5_3_replan_row(bag_time_s=10.0),
            p5_row(bag_time_s=10.1),
        ]
        rows.extend(
            p5_3_replan_row(
                bag_time_s=11.0 + idx,
                action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                first_bad_tau=0.0,
                samples=contaminated_samples,
            )
            for idx in range(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE)
        )

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["event_window_available"])
        self.assertTrue(gates["event_window_current_outside_fixture"])
        self.assertEqual(3, gates["full_run_sample_summary"]["current_inside_fixture_count"])
        self.assertEqual(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE, gates["max_consecutive_emergency"])
        self.assertEqual(0, gates["event_window_max_consecutive_emergency"])

    def test_p5_3_fails_when_event_window_future_samples_do_not_enter_fixture_tau_window(self):
        samples = [
            p5_3_sample(),
            p5_3_sample(
                tau_s=0.8,
                query_tau_s=0.8,
                x=-10.5,
                hpl=10.2,
                vpl=10.2,
                im_min=-0.2,
                bad=True,
                reason="future_low_margin:p5_3_high_risk_zone",
                fixture_match=True,
                fixture_expected_hpl=10.2,
                fixture_expected_vpl=10.2,
                fixture_expected_reason="p5_3_high_risk_zone",
            ),
        ]

        _, gates, failures, _ = self.validate_rows(
            [p5_3_replan_row(samples=samples)]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["event_window_available"])
        self.assertEqual(0, gates["event_window_future_fixture_sample_count"])
        self.assertTrue(any("future-only fixture evidence" in failure for failure in failures), failures)

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
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
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

    def test_p5_3_fails_when_runtime_fixture_match_is_false_in_fixture_window(self):
        samples = p5_3_future_only_samples()
        samples[1] = p5_3_sample(
            tau_s=1.2,
            x=-10.5,
            hpl=10.2,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason="future_low_margin:p5_3_high_risk_zone",
            fixture_match=False,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_3_high_risk_zone",
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
        self.assertTrue(any("runtime_fixture_match=False" in failure for failure in failures), failures)

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

    def test_p5_3_fails_on_event_window_emergency_storm(self):
        rows = [p5_3_replan_row(bag_time_s=10.0)]
        rows.extend(
            p5_3_replan_row(
                bag_time_s=11.0 + idx,
                action="REQUEST_REPLAN",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
            )
            for idx in range(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE)
        )

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertEqual(
            analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE,
            gates["event_window_raw_max_consecutive_emergency"],
        )
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
            analyzer.P5_3_FAIL_BRANCH,
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

    def test_p5_3_future_sampling_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_3_future_sampling_scenario_topdown.png",
                "p5_3_future_sampling_fixture_overlay.png",
                "p5_3_future_sampling_pl_probe.png",
                "p5_3_future_sampling_tau_window.png",
                "p5_3_future_sampling_margin_timeline.png",
                "p5_3_future_sampling_action_reason.png",
                "p5_3_future_sampling_sample_heatmap.png",
                "p5_3_future_sampling_topic_gap.png",
                "p5_3_future_sampling_p0_health.png",
            ],
            analyzer.P5_3_FUTURE_SAMPLING_FIGURE_FILENAMES,
        )

    def test_p5_3_event_window_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_3_event_window_scenario_topdown.png",
                "p5_3_event_window_fixture_overlay.png",
                "p5_3_event_window_tau_window.png",
                "p5_3_event_window_pl_probe.png",
                "p5_3_event_window_margin_timeline.png",
                "p5_3_event_window_action_reason.png",
                "p5_3_event_window_replan_vs_emergency.png",
                "p5_3_event_window_sample_heatmap.png",
                "p5_3_event_window_topic_gap.png",
                "p5_3_event_window_p0_health.png",
            ],
            analyzer.P5_3_EVENT_WINDOW_FIGURE_FILENAMES,
        )


class P5_4AnalyzerTest(unittest.TestCase):
    def validate_rows(
        self,
        rows,
        manifest=None,
        marker_rows=None,
        topic_timestamps=None,
        topic_health=None,
    ):
        p5_summary = analyzer.summarize_p5_status_rows(rows)
        failures = []
        inconclusive = []
        p0_rows = bounded_startup_p0_rows()
        gates = analyzer.validate_p5_4_hard_gates(
            manifest if manifest is not None else p5_4_manifest(),
            {"passed": True},
            topic_health if topic_health is not None else p5_topic_health(),
            analyzer.summarize_p0_health(p0_rows),
            p0_rows,
            rows,
            p5_summary,
            marker_rows if marker_rows is not None else [p5_4_marker_row()],
            failures,
            inconclusive,
            topic_timestamps,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_4_passes_with_near_risk_emergency_candidate(self):
        rows = [p5_row(bag_time_s=idx) for idx in range(3)]
        rows.append(p5_4_emergency_row())

        _, gates, failures, inconclusive = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["event_window_available"])
        self.assertTrue(gates["anchor_strict_emergency"])
        self.assertTrue(gates["anchor_same_row_fixture_samples"])
        self.assertEqual(0.6, gates["anchor_first_bad_tau"])
        self.assertTrue(gates["anchor_first_bad_tau_within_emergency_time"])
        self.assertTrue(gates["fixture_query_aligned"])

    def test_p5_4_allows_final_candidate_future_bad_bookkeeping(self):
        _, gates, failures, inconclusive = self.validate_rows(
            [
                p5_4_emergency_row(
                    phase="final",
                    final_gate_fail_count=1,
                    final_gate_fail_duration_s=0.1,
                    final_gate_last_reason="future_bad",
                )
            ]
        )

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["event_window_available"])
        self.assertEqual([], gates["anchor_exclusion_causes"])

    def test_p5_4_reports_blocked_when_fixture_manifest_is_missing_or_disabled(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_4_emergency_row()],
            manifest=p5_manifest(),
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["blocked_scenario_missing"])
        self.assertTrue(
            any("P5-4 fixture manifest is missing" in failure for failure in failures),
            failures,
        )

        _, disabled_gates, _, _ = self.validate_rows(
            [p5_4_emergency_row()],
            manifest=p5_4_manifest(enabled=False),
        )
        self.assertTrue(disabled_gates["blocked_scenario_missing"])

    def test_p5_4_fails_when_fixture_is_not_entered(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_4_emergency_row(samples=[p5_4_sample()])]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["fixture_entered"])
        self.assertTrue(any("fixture not entered" in failure for failure in failures), failures)

    def test_p5_4_fails_when_injected_pl_does_not_align(self):
        samples = p5_4_future_only_samples()
        samples[1] = p5_4_sample(
            tau_s=0.6,
            x=-11.4,
            hpl=9.9,
            vpl=10.2,
            im_min=-0.2,
            bad=True,
            reason="future_bad:p5_4_near_risk_zone",
            fixture_match=True,
            fixture_expected_hpl=10.2,
            fixture_expected_vpl=10.2,
            fixture_expected_reason="p5_4_near_risk_zone",
        )

        _, gates, failures, _ = self.validate_rows(
            [p5_4_emergency_row(samples=samples)]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["fixture_entered"])
        self.assertTrue(gates["fixture_query_aligned"])
        self.assertFalse(gates["fixture_query_mismatch_absent"])
        self.assertTrue(any("injected PL mismatch" in failure for failure in failures), failures)

    def test_p5_4_fails_when_first_bad_tau_exceeds_emergency_horizon(self):
        _, gates, failures, _ = self.validate_rows(
            [p5_4_emergency_row(first_bad_tau=1.2)]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["event_window_available"])
        self.assertFalse(gates["anchor_first_bad_tau_within_emergency_time"])
        self.assertTrue(any("first_bad_tau" in failure for failure in failures), failures)

    def test_p5_4_rejects_current_only_emergency_cause(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_4_emergency_row(
                    reason="current_low_margin",
                    current_reason="current_low_margin",
                    future_reason="",
                    active_reasons=["current_low_margin"],
                )
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(
            any("current-only low margin" in failure for failure in failures),
            failures,
        )

    def test_p5_4_rejects_startup_and_unknown_only_emergency_cause(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_4_emergency_row(
                    phase="startup",
                    reason="snapshot_unavailable",
                    future_reason="",
                    active_reasons=["snapshot_unavailable", "future_unknown"],
                    unknown_ratio=1.0,
                    bad_ratio=0.0,
                )
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertTrue(
            any("startup/snapshot_unavailable" in failure for failure in failures),
            failures,
        )
        self.assertTrue(any("unknown-only" in failure for failure in failures), failures)

    def test_p5_4_rejects_final_gate_failed_emergency_cause(self):
        _, gates, failures, _ = self.validate_rows(
            [
                p5_4_emergency_row(
                    reason="final_gate_failed",
                    final_gate_last_reason="final_gate_failed",
                    active_reasons=["final_gate_failed"],
                    final_gate_fail_count=1,
                    final_gate_fail_duration_s=0.1,
                )
            ]
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["event_window_available"])
        self.assertTrue(
            any("final-gate emergency cause" in failure for failure in failures),
            failures,
        )

    def test_p5_4_fails_on_unexplained_emergency_storm(self):
        rows = [
            p5_4_emergency_row(
                bag_time_s=10.0 + idx,
                reason="current_low_margin",
                current_reason="current_low_margin",
                future_reason="",
                active_reasons=["current_low_margin"],
            )
            for idx in range(analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE)
        ]

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertEqual(
            analyzer.P5_2_EMERGENCY_STORM_CONSECUTIVE,
            gates["max_consecutive_unexplained_emergency"],
        )
        self.assertTrue(
            any("unexplained emergency storm" in failure for failure in failures),
            failures,
        )

    def test_p5_4_required_figures_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            failures = []
            analyzer.validate_p5_4_required_figures(
                [Path(tmpdir) / "p5_4_missing.png"],
                failures,
            )

        self.assertEqual(1, len(failures))
        self.assertIn("P5-4 required figure missing", failures[0])

    def test_p5_4_next_branch_is_exact(self):
        self.assertEqual(
            "PASS -> P5-5",
            analyzer.next_debug_branch("PASS", [], [], "P5-4"),
        )
        self.assertEqual(
            "BLOCKED_SCENARIO_MISSING",
            analyzer.next_debug_branch(
                "BLOCKED_SCENARIO_MISSING",
                ["P5-4 fixture manifest is missing"],
                [],
                "P5-4",
            ),
        )
        self.assertEqual(
            analyzer.P5_4_FAIL_BRANCH,
            analyzer.next_debug_branch("FAIL", ["P5-4 evidence failed"], [], "P5-4"),
        )

    def test_p5_4_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_4_scenario_topdown.png",
                "p5_4_near_risk_overlay.png",
                "p5_4_tau_emergency_window.png",
                "p5_4_pl_probe.png",
                "p5_4_margin_timeline.png",
                "p5_4_action_reason_timeline.png",
                "p5_4_replan_vs_emergency.png",
                "p5_4_sample_heatmap.png",
                "p5_4_topic_gap.png",
                "p5_4_p0_health.png",
                "p5_4_final_gate_summary.png",
                "p5_4_trajectory_integrity_samples.png",
            ],
            analyzer.P5_4_FIGURE_FILENAMES,
        )


class P5_5AnalyzerTest(unittest.TestCase):
    def validate_rows(
        self,
        rows=None,
        manifest=None,
        integrity_rows=None,
        topic_timestamps=None,
        p0_rows=None,
        topic_health=None,
    ):
        status_rows = rows if rows is not None else p5_5_status_rows()
        p0_health_rows = p0_rows if p0_rows is not None else bounded_startup_p0_rows()
        p5_summary = analyzer.summarize_p5_status_rows(status_rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_5_hard_gates(
            manifest if manifest is not None else p5_5_manifest(),
            {"passed": True},
            topic_health if topic_health is not None else p5_topic_health(),
            analyzer.summarize_p0_health(p0_health_rows),
            p0_health_rows,
            status_rows,
            p5_summary,
            integrity_rows if integrity_rows is not None else p5_5_integrity_rows(),
            failures,
            inconclusive,
            topic_timestamps,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_5_passes_with_stamp_freeze_and_current_stale_sequence(self):
        _, gates, failures, inconclusive = self.validate_rows()

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["fixture_window_entered"])
        self.assertTrue(gates["integrity_header_frozen"])
        self.assertTrue(gates["integrity_continued_publishing"])
        self.assertTrue(gates["current_stale_replan_present"])
        self.assertTrue(gates["current_stale_emergency_present"])
        self.assertTrue(gates["replan_before_emergency"])

    def test_p5_5_accepts_release_edge_after_frozen_plateau(self):
        integrity_rows = p5_5_integrity_rows()
        integrity_rows.insert(
            -1,
            {
                "bag_time_s": 141.99,
                "t_rel_s": 41.99,
                "header_stamp_s": 142.0,
                "header_rel_s": 42.0,
                "bag_minus_header_s": -0.01,
                "in_expected_window": 1,
                "hpl": 1.0,
                "vpl": 1.0,
                "hal": 10.0,
                "val": 10.0,
                "im": 9.0,
                "pl_al_finite": 1,
                "fusion_mode": "fallback_only",
                "final_hpl_source": "FALLBACK",
                "final_vpl_source": "FALLBACK",
            },
        )

        _, gates, failures, _ = self.validate_rows(integrity_rows=integrity_rows)

        self.assertEqual([], failures)
        self.assertTrue(gates["integrity_header_frozen"])
        self.assertGreaterEqual(gates["integrity_evidence"]["freeze_row_count"], 2)

    def test_p5_5_reports_blocked_when_fixture_manifest_missing_or_disabled(self):
        _, gates, failures, _ = self.validate_rows(manifest=p5_manifest())

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["blocked_scenario_missing"])
        self.assertTrue(
            any("P5-5 fixture manifest is missing" in failure for failure in failures),
            failures,
        )

        _, disabled_gates, _, _ = self.validate_rows(
            manifest=p5_5_manifest(enabled=False)
        )
        self.assertTrue(disabled_gates["blocked_scenario_missing"])

    def test_p5_5_fails_when_fixture_window_not_entered_or_stamp_not_frozen(self):
        _, gates, failures, _ = self.validate_rows(
            integrity_rows=p5_5_integrity_rows(entered=False)
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["fixture_window_entered"])
        self.assertTrue(any("fixture window was not entered" in failure for failure in failures), failures)

        _, frozen_gates, frozen_failures, _ = self.validate_rows(
            integrity_rows=p5_5_integrity_rows(freeze=False)
        )
        self.assertFalse(frozen_gates["passed"])
        self.assertFalse(frozen_gates["integrity_header_frozen"])
        self.assertTrue(any("header stamp did not freeze" in failure for failure in frozen_failures), frozen_failures)

    def test_p5_5_fails_when_stale_duration_does_not_grow(self):
        rows = p5_5_status_rows()
        for row in rows:
            row["current_stale_duration_s"] = min(
                float(row.get("current_stale_duration_s") or 0.0),
                0.4,
            )

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["current_stale_duration_grew"])
        self.assertTrue(any("current_stale_duration_s did not grow" in failure for failure in failures), failures)

    def test_p5_5_fails_on_immediate_emergency_or_missing_replan(self):
        emergency_only = [
            p5_row(
                bag_time_s=130.1,
                action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                reason="current_stale",
                raw_reason="current_stale",
                current_reason="current_stale",
                active_reasons=["current_stale"],
                current_integrity_age_s=0.7,
                current_stale_duration_s=0.2,
            )
        ]
        _, gates, failures, _ = self.validate_rows(emergency_only)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["no_immediate_emergency"])
        self.assertFalse(gates["replan_before_emergency"])
        self.assertTrue(any("immediate emergency" in failure for failure in failures), failures)

        no_replan = [row for row in p5_5_status_rows() if row.get("action") != "REQUEST_REPLAN"]
        _, no_replan_gates, no_replan_failures, _ = self.validate_rows(no_replan)
        self.assertFalse(no_replan_gates["current_stale_replan_present"])
        self.assertTrue(any("did not observe REQUEST_REPLAN" in failure for failure in no_replan_failures), no_replan_failures)

    def test_p5_5_fails_without_emergency_after_threshold(self):
        no_emergency = [
            row
            for row in p5_5_status_rows()
            if row.get("action") != "REQUEST_EMERGENCY_STOP_CANDIDATE"
        ]

        _, gates, failures, _ = self.validate_rows(no_emergency)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["current_stale_emergency_present"])
        self.assertTrue(any("did not observe REQUEST_EMERGENCY_STOP_CANDIDATE" in failure for failure in failures), failures)

    def test_p5_5_rejects_wrong_cause_attribution_and_contamination(self):
        wrong = [
            p5_row(
                bag_time_s=130.8,
                action="REQUEST_REPLAN",
                raw_action="REQUEST_REPLAN",
                reason="future_unknown",
                raw_reason="future_unknown",
                future_reason="future_unknown",
                active_reasons=["future_unknown"],
                unknown_ratio=1.0,
                bad_ratio=0.0,
                current_integrity_age_s=1.3,
                current_stale_duration_s=0.8,
            ),
            p5_row(
                bag_time_s=132.2,
                action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                reason="future_bad",
                raw_reason="future_bad",
                future_reason="future_bad",
                active_reasons=["future_bad"],
                bad_ratio=0.3,
                current_integrity_age_s=2.7,
                current_stale_duration_s=2.2,
            ),
        ]

        _, gates, failures, _ = self.validate_rows(wrong)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["future_bad_excluded"])
        self.assertFalse(gates["unknown_only_excluded"])
        self.assertFalse(gates["non_current_stale_actions_excluded"])
        self.assertTrue(any("future_bad cause contaminated" in failure for failure in failures), failures)
        self.assertTrue(any("unknown-only cause contaminated" in failure for failure in failures), failures)
        self.assertTrue(any("did not trace actions to current_stale" in failure for failure in failures), failures)

    def test_p5_5_ignores_post_window_non_stale_actions_for_cause_exclusion(self):
        rows = p5_5_status_rows()
        rows.append(
            p5_row(
                bag_time_s=145.0,
                action="REQUEST_REPLAN",
                raw_action="REQUEST_REPLAN",
                reason="current_low_margin",
                raw_reason="current_low_margin",
                current_reason="current_low_margin",
                active_reasons=["current_low_margin"],
                current_integrity_age_s=0.1,
                current_stale_duration_s=0.0,
            )
        )

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertTrue(gates["non_current_stale_actions_excluded"])

    def test_p5_5_rejects_topic_gap_and_p0_full_unknown(self):
        bad_timestamps = {
            topic: [130.0, 140.0]
            for topic, expected in analyzer.P5_TOPIC_EXPECTATIONS.items()
            if expected == "continuous"
        }

        _, gap_gates, gap_failures, _ = self.validate_rows(
            topic_timestamps=bad_timestamps
        )

        self.assertFalse(gap_gates["active_required_p5_topics_stable"])
        self.assertTrue(any("topic gap exceeded" in failure for failure in gap_failures), gap_failures)

        p0_rows = bounded_startup_p0_rows()
        p0_rows.append(
            p0_health_row(
                100,
                ready=True,
                stale=False,
                valid_ratio=0.0,
                unknown_ratio=1.0,
                reason="ok",
            )
        )
        _, p0_gates, p0_failures, _ = self.validate_rows(p0_rows=p0_rows)
        self.assertFalse(p0_gates["p0_post_startup_not_full_unknown"])
        self.assertTrue(any("full unknown after startup" in failure for failure in p0_failures), p0_failures)

    def test_p5_5_required_figures_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            failures = []
            analyzer.validate_p5_5_required_figures(
                [Path(tmpdir) / "p5_5_missing.png"],
                failures,
            )

        self.assertEqual(1, len(failures))
        self.assertIn("P5-5 required figure missing", failures[0])

    def test_p5_5_next_branch_is_exact(self):
        self.assertEqual(
            "PASS -> P5-6",
            analyzer.next_debug_branch("PASS", [], [], "P5-5"),
        )
        self.assertEqual(
            analyzer.P5_5_BLOCKED_BRANCH,
            analyzer.next_debug_branch(
                "BLOCKED_SCENARIO_MISSING",
                ["P5-5 fixture manifest is missing"],
                [],
                "P5-5",
            ),
        )
        self.assertEqual(
            analyzer.P5_5_FAIL_BRANCH,
            analyzer.next_debug_branch("FAIL", ["P5-5 evidence failed"], [], "P5-5"),
        )

    def test_p5_5_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_5_scenario_topdown.png",
                "p5_5_topic_activity_timeline.png",
                "p5_5_integrity_pause_timeline.png",
                "p5_5_current_stale_duration_timeline.png",
                "p5_5_action_reason_timeline.png",
                "p5_5_replan_vs_emergency.png",
                "p5_5_debounce_timeline.png",
                "p5_5_margin_timeline.png",
                "p5_5_p0_health.png",
                "p5_5_cause_exclusion_summary.png",
                "p5_5_trajectory_integrity_samples.png",
            ],
            analyzer.P5_5_FIGURE_FILENAMES,
        )


class P5_6AnalyzerTest(unittest.TestCase):
    def validate_rows(
        self,
        rows=None,
        manifest=None,
        p0_rows=None,
        topic_timestamps=None,
        topic_health=None,
    ):
        status_rows = rows if rows is not None else p5_6_status_rows()
        p0_health_rows = p0_rows if p0_rows is not None else p5_6_p0_rows()
        p5_summary = analyzer.summarize_p5_status_rows(status_rows)
        failures = []
        inconclusive = []
        gates = analyzer.validate_p5_6_hard_gates(
            manifest if manifest is not None else p5_6_manifest(),
            {"passed": True},
            topic_health if topic_health is not None else p5_topic_health(),
            analyzer.summarize_p0_health(p0_health_rows),
            p0_health_rows,
            status_rows,
            p5_summary,
            failures,
            inconclusive,
            topic_timestamps,
        )
        return p5_summary, gates, failures, inconclusive

    def test_p5_6_passes_with_future_unknown_replan_then_emergency(self):
        _, gates, failures, inconclusive = self.validate_rows()

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["passed"])
        self.assertTrue(gates["p5_5_fixture_disabled"])
        self.assertTrue(gates["fixture_ready"])
        self.assertTrue(gates["fixture_unknown_samples_present"])
        self.assertTrue(gates["p0_unknown_ratio_grew"])
        self.assertTrue(gates["p5_unknown_ratio_grew"])
        self.assertTrue(gates["p5_future_unknown_duration_crossed_threshold"])
        self.assertTrue(gates["early_non_emergency_unknown_present"])
        self.assertTrue(gates["unknown_replan_present"])
        self.assertTrue(gates["unknown_emergency_present"])
        self.assertTrue(gates["replan_before_emergency"])

    def test_p5_6_accepts_elevated_p0_unknown_without_large_delta(self):
        p0_rows = [
            p0_health_row(0, unknown_ratio=0.25, valid_ratio=0.75),
            p0_health_row(1, unknown_ratio=0.31, valid_ratio=0.69),
            p0_health_row(2, unknown_ratio=0.33, valid_ratio=0.67),
        ]

        _, gates, failures, inconclusive = self.validate_rows(p0_rows=p0_rows)

        self.assertEqual([], failures)
        self.assertEqual([], inconclusive)
        self.assertTrue(gates["p0_unknown_ratio_elevated"])
        self.assertFalse(gates["p0_unknown_ratio_grew"])
        self.assertTrue(gates["passed"])

    def test_p5_6_fails_when_future_unknown_fixture_is_missing(self):
        manifest = p5_6_manifest()
        for key in list(manifest):
            if str(key).startswith("p5_6.fixture."):
                manifest.pop(key)
        manifest.pop("p5_6", None)

        _, gates, failures, _ = self.validate_rows(manifest=manifest)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["fixture_present"])
        self.assertTrue(any("fixture manifest is missing" in failure for failure in failures), failures)

    def test_p5_6_fails_when_future_unknown_fixture_is_not_effective(self):
        manifest = p5_6_manifest()
        manifest["p5_6.fixture.effective_enabled"] = False
        manifest["p5_6"]["fixture"]["effective_enabled"] = False

        _, gates, failures, _ = self.validate_rows(manifest=manifest)

        self.assertFalse(gates["passed"])
        self.assertTrue(gates["fixture_enabled"])
        self.assertFalse(gates["fixture_effective_enabled"])
        self.assertTrue(any("not effectively enabled" in failure for failure in failures), failures)

    def test_p5_6_fails_when_p5_3_or_p5_4_fixture_is_enabled(self):
        for key, gate_key, text in (
            ("p5_3.fixture.enabled", "p5_3_fixture_disabled", "P5-3"),
            ("p5_4.fixture.enabled", "p5_4_fixture_disabled", "P5-4"),
        ):
            manifest = p5_6_manifest()
            manifest[key] = True

            _, gates, failures, _ = self.validate_rows(manifest=manifest)

            self.assertFalse(gates["passed"])
            self.assertFalse(gates[gate_key])
            self.assertTrue(any(text in failure for failure in failures), failures)

    def test_p5_6_accepts_al_invalid_as_unknown_when_bad_ratio_is_zero(self):
        rows = p5_6_status_rows()
        for row in rows[1:]:
            row["reason"] = "al_invalid"
            row["raw_reason"] = "al_invalid"
            row["future_reason"] = "al_invalid"
            row["active_reasons"] = ["al_invalid"]
            row["pred_al_last_reason"] = "pred_al_invalid"
        rows[-1]["reason"] = "final_gate_failed"
        rows[-1]["final_gate_last_reason"] = "al_invalid"

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertEqual([], failures)
        self.assertTrue(gates["passed"])

    def test_p5_6_fails_when_p5_5_fixture_is_enabled(self):
        _, gates, failures, _ = self.validate_rows(
            manifest=p5_6_manifest(p5_5_enabled=True)
        )

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["p5_5_fixture_disabled"])
        self.assertFalse(gates["p5_5_fixture_excluded"])
        self.assertTrue(any("P5-5 fixture" in failure for failure in failures), failures)

    def test_p5_6_fails_on_missing_p0_health_or_p5_status(self):
        _, p0_gates, p0_failures, _ = self.validate_rows(p0_rows=[])
        self.assertFalse(p0_gates["p0_health_rows_present"])
        self.assertTrue(any("P0 health rows are missing" in failure for failure in p0_failures), p0_failures)

        _, p5_gates, p5_failures, _ = self.validate_rows(rows=[])
        self.assertFalse(p5_gates["p5_status_rows_present"])
        self.assertTrue(any("P5 status rows are missing" in failure for failure in p5_failures), p5_failures)

    def test_p5_6_fails_when_future_unknown_duration_does_not_grow_or_cross(self):
        rows = p5_6_status_rows()
        for row in rows:
            row["future_unknown_duration_s"] = min(
                float(row.get("future_unknown_duration_s") or 0.0),
                0.4,
            )

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["passed"])
        self.assertFalse(gates["p5_future_unknown_duration_grew"])
        self.assertFalse(gates["p5_future_unknown_duration_crossed_threshold"])
        self.assertTrue(any("future_unknown_duration_s did not grow" in failure for failure in failures), failures)

    def test_p5_6_fails_when_unknown_ratio_is_not_elevated(self):
        rows = p5_6_status_rows()
        for row in rows:
            row["unknown_ratio"] = min(float(row.get("unknown_ratio") or 0.0), 0.1)

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["p5_unknown_ratio_elevated"])
        self.assertTrue(any("P5 unknown_ratio did not clearly rise" in failure for failure in failures), failures)

    def test_p5_6_fails_when_unknown_rows_are_all_ok(self):
        rows = [
            p5_row(
                bag_time_s=10.0 + idx,
                reason="future_unknown",
                raw_reason="future_unknown",
                future_reason="future_unknown",
                active_reasons=["future_unknown"],
                unknown_ratio=0.35 + 0.05 * idx,
                bad_ratio=0.0,
                future_unknown_duration_s=0.2 + idx,
                unknown_count=4,
                bad_count=0,
            )
            for idx in range(3)
        ]

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["unknown_actions_not_all_ok"])
        self.assertTrue(any("all treated as OK" in failure for failure in failures), failures)

    def test_p5_6_fails_without_replan_or_emergency(self):
        no_replan = [
            row
            for row in p5_6_status_rows()
            if row.get("action") != "REQUEST_REPLAN"
            and row.get("raw_action") != "REQUEST_REPLAN"
        ]
        _, replan_gates, replan_failures, _ = self.validate_rows(no_replan)
        self.assertFalse(replan_gates["unknown_replan_present"])
        self.assertTrue(any("did not observe REQUEST_REPLAN" in failure for failure in replan_failures), replan_failures)

        no_emergency = [
            row
            for row in p5_6_status_rows()
            if row.get("action") != "REQUEST_EMERGENCY_STOP_CANDIDATE"
            and row.get("raw_action") != "REQUEST_EMERGENCY_STOP_CANDIDATE"
        ]
        _, emergency_gates, emergency_failures, _ = self.validate_rows(no_emergency)
        self.assertFalse(emergency_gates["unknown_emergency_present"])
        self.assertTrue(any("REQUEST_EMERGENCY_STOP_CANDIDATE" in failure for failure in emergency_failures), emergency_failures)

    def test_p5_6_fails_on_immediate_emergency(self):
        rows = [
            p5_row(bag_time_s=9.0, unknown_ratio=0.0, future_unknown_duration_s=0.0),
            p5_row(
                bag_time_s=10.0,
                action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                raw_action="REQUEST_EMERGENCY_STOP_CANDIDATE",
                reason="future_unknown",
                raw_reason="future_unknown",
                future_reason="future_unknown",
                active_reasons=["future_unknown"],
                unknown_ratio=0.4,
                bad_ratio=0.0,
                future_unknown_duration_s=2.1,
                unknown_count=4,
                bad_count=0,
            ),
        ]

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["early_non_emergency_unknown_present"])
        self.assertFalse(gates["no_immediate_emergency"])
        self.assertTrue(any("immediate emergency" in failure for failure in failures), failures)

    def test_p5_6_fails_wrong_attribution(self):
        rows = p5_6_status_rows()
        for row in rows[1:]:
            row["reason"] = "mystery_unknown"
            row["raw_reason"] = "mystery_unknown"
            row["future_reason"] = "mystery_unknown"
            row["active_reasons"] = ["mystery_unknown"]
        rows[-1]["reason"] = "final_gate_failed"
        rows[-1]["final_gate_last_reason"] = "mystery_unknown"

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertFalse(gates["unknown_replan_present"])
        self.assertFalse(gates["unknown_emergency_present"])
        self.assertTrue(any("attributed to future unknown" in failure for failure in failures), failures)

    def test_p5_6_rejects_excluded_causes(self):
        cases = [
            (
                {
                    "current_reason": "current_stale",
                    "active_reasons": ["future_unknown", "current_stale"],
                    "current_stale_duration_s": 0.8,
                },
                "current_stale",
                "current_stale_excluded",
            ),
            (
                {
                    "reason": "current_invalid",
                    "raw_reason": "current_invalid",
                    "current_reason": "current_invalid",
                    "active_reasons": ["current_invalid"],
                },
                "current_invalid",
                "current_invalid_excluded",
            ),
            (
                {
                    "reason": "future_bad",
                    "raw_reason": "future_bad",
                    "future_reason": "future_bad",
                    "active_reasons": ["future_bad"],
                    "bad_ratio": 0.3,
                },
                "future_bad",
                "future_bad_excluded",
            ),
            (
                {
                    "phase": "startup",
                    "reason": "snapshot_unavailable",
                    "raw_reason": "snapshot_unavailable",
                    "future_reason": "",
                    "active_reasons": ["snapshot_unavailable"],
                },
                "snapshot_unavailable",
                "startup_snapshot_excluded",
            ),
            (
                {
                    "reason": "topic_gap",
                    "raw_reason": "topic_gap",
                    "future_reason": "",
                    "active_reasons": ["topic_gap"],
                },
                "topic gap",
                "topic_gap_excluded",
            ),
            (
                {
                    "reason": "future_low_margin",
                    "raw_reason": "future_low_margin",
                    "future_reason": "future_low_margin",
                    "active_reasons": ["future_low_margin"],
                    "bad_ratio": 0.3,
                },
                "low-margin-only",
                "low_margin_only_excluded",
            ),
        ]
        for overrides, failure_text, gate_key in cases:
            rows = p5_6_status_rows()
            rows[2].update(overrides)
            if "future_unknown" not in rows[2].get("active_reasons", []):
                rows[2]["unknown_ratio"] = 0.5
                rows[2]["future_unknown_duration_s"] = 0.8

            _, gates, failures, _ = self.validate_rows(rows)

            self.assertFalse(gates[gate_key], gate_key)
            self.assertTrue(any(failure_text in failure for failure in failures), (failure_text, failures))

    def test_p5_6_fails_on_active_unknown_window_topic_gap(self):
        bad_timestamps = {
            topic: [10.0, 20.0]
            for topic, expected in analyzer.P5_TOPIC_EXPECTATIONS.items()
            if expected == "continuous"
        }

        _, gates, failures, _ = self.validate_rows(topic_timestamps=bad_timestamps)

        self.assertFalse(gates["active_required_p5_topics_stable"])
        self.assertTrue(any("topic gap" in failure for failure in failures), failures)

    def test_p5_6_ignores_snapshot_and_current_invalid_outside_unknown_window(self):
        rows = [
            p5_row(
                bag_time_s=1.0,
                phase="startup",
                action="REQUEST_REPLAN",
                raw_action="REQUEST_REPLAN",
                reason="snapshot_unavailable",
                raw_reason="snapshot_unavailable",
                unknown_ratio=1.0,
            ),
            *p5_6_status_rows(),
            p5_row(
                bag_time_s=30.0,
                action="REQUEST_REPLAN",
                raw_action="REQUEST_REPLAN",
                reason="current_invalid",
                raw_reason="current_invalid",
                current_reason="current_invalid",
                active_reasons=["current_invalid"],
                unknown_ratio=0.0,
                future_unknown_duration_s=0.0,
            ),
        ]

        _, gates, failures, _ = self.validate_rows(rows)

        self.assertTrue(gates["passed"], failures)
        self.assertTrue(gates["startup_snapshot_excluded"])
        self.assertTrue(gates["current_invalid_excluded"])

    def test_p5_6_required_figures_are_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            failures = []
            analyzer.validate_p5_6_required_figures(
                [Path(tmpdir) / "p5_6_missing.png"],
                failures,
            )

        self.assertEqual(1, len(failures))
        self.assertIn("P5-6 required figure missing", failures[0])

    def test_p5_6_next_branch_is_exact(self):
        self.assertEqual(
            "PASS -> P5-7",
            analyzer.next_debug_branch("PASS", [], [], "P5-6"),
        )
        self.assertEqual(
            analyzer.P5_6_FAIL_BRANCH,
            analyzer.next_debug_branch("FAIL", ["P5-6 evidence failed"], [], "P5-6"),
        )

    def test_p5_6_required_figure_filenames_are_exact(self):
        self.assertEqual(
            [
                "p5_6_scenario_topdown.png",
                "p5_6_unknown_field_overlay.png",
                "p5_6_p0_health_unknown_timeline.png",
                "p5_6_future_unknown_duration_timeline.png",
                "p5_6_unknown_ratio_vs_action.png",
                "p5_6_action_reason_timeline.png",
                "p5_6_debounce_timeline.png",
                "p5_6_cause_exclusion_summary.png",
                "p5_6_trajectory_integrity_samples.png",
            ],
            analyzer.P5_6_FIGURE_FILENAMES,
        )


if __name__ == "__main__":
    unittest.main()
