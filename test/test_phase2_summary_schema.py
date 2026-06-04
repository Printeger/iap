#!/usr/bin/env python3
import copy
import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
PHASE2_TOOLS = PACKAGE_ROOT / "tools" / "phase2"
TOOLS = PACKAGE_ROOT / "tools"
for path in (str(PHASE2_TOOLS), str(TOOLS)):
    if path not in sys.path:
        sys.path.insert(0, path)

from phase2_summary_schema import (  # noqa: E402
    REQUIRED_ONLINE_SUMMARY_FIELDS,
    merge_online_and_offline_summary,
    missing_required_online_fields,
)
from validate_phase2_integrity_eval import (  # noqa: E402
    check_advisory_fim,
    check_lidar_observability,
    check_urg,
)


def base_summary() -> dict:
    return {
        "available": True,
        "run_dir": "/tmp/run",
        "traj_count": 1,
        "sample_count": 1,
        "aligned_sample_count": 0,
        "online_truth_used": False,
        "odom_source": "/drone_0_visual_slam/odom",
        "map_source": "/map",
        "pl_model": "fused_fim_grid",
        "al_model": "corridor",
        "fallback_count": 0,
        "fallback_rate": 0.0,
        "fallback_reason_histogram": {},
        "finite_gnss_prediction_count": 1,
        "integrity_snapshot": {"available": True, "sample_count": 1},
        "current_consistency_raw": {
            "available": True,
            "finite_count": 1,
            "warning_threshold_ratio": 0.10,
            "max_pl_ratio": 0.0,
        },
        "current_consistency_anchored": {
            "available": True,
            "finite_count": 1,
            "warning_threshold_ratio": 0.10,
            "max_pl_ratio": 0.0,
        },
        "current_consistency": {
            "available": True,
            "finite_count": 1,
            "warning_threshold_ratio": 0.10,
            "max_pl_ratio": 0.0,
        },
        "phase_h_lite": {"fused_fim_grid": "available"},
        "stage1_capabilities": {"fused_araim_style": "deferred_after_rc"},
        "pl_grid": {"enabled": False},
        "lidar_observability": {
            "enabled": True,
            "use_lidar_observability": True,
            "fused_fim_grid": True,
            "valid_count": 1,
            "valid_rate": 1.0,
            "conservative_fusion_violation_count": 0,
        },
        "pi_cost": {"available": True, "count": 1},
        "predicted_integrity": {
            "safe_count": 1,
            "fallback_count": 0,
            "fallback_rate": 0.0,
            "fallback_reason_histogram": {},
            "finite_gnss_prediction_count": 1,
        },
        "actual_alignment": {},
        "warnings": [],
        "errors": [],
    }


def online_row(**overrides: str) -> dict:
    row = {
        "PL_H_pred": "25.0",
        "PL_V_pred": "25.0",
        "gnss_hpl": "20.0",
        "gnss_vpl": "20.0",
        "lidar_alpha": "1.0",
        "lidar_tdop": "2.0",
        "lidar_condition": "3.0",
        "advisory_fusion_mode": "legacy",
        "lambda_adv_trace": "",
        "hpl_adv": "",
        "vpl_adv": "",
        "fim_epsilon_applied": "",
        "fim_degeneracy_regularized": "",
        "fim_fallback_reason": "",
    }
    row.update(overrides)
    return row


class Phase2SummarySchemaTest(unittest.TestCase):
    def test_legacy_summary_keeps_conservative_check(self) -> None:
        summary = base_summary()
        failures: list[str] = []
        warnings: list[str] = []

        check_lidar_observability(summary, [online_row()], failures, warnings)

        self.assertEqual([], failures)

    def test_fim_add_allows_advisory_pl_below_gnss_proxy(self) -> None:
        summary = base_summary()
        summary["stage1_predictor_config"] = {"phase2_use_advisory_fim_add": True}
        summary["advisory_fim"] = {
            "enabled": True,
            "use_lidar_advisory_fim": True,
            "fusion_mode": "fim_add",
            "query_count": 4,
            "epsilon_applied_count": 4,
            "degeneracy_regularized_count": 1,
            "regularized_count": 1,
            "fallback_reason_histogram": {},
            "gnss_fim_valid_count": 2,
            "lidar_fim_valid_count": 4,
        }
        rows = [
            online_row(
                PL_H_pred="1.0",
                PL_V_pred="0.5",
                gnss_hpl="20.0",
                gnss_vpl="20.0",
                advisory_fusion_mode="fim_add",
                lambda_adv_trace="189.6",
                hpl_adv="1.0",
                vpl_adv="0.5",
            )
        ]
        failures: list[str] = []
        warnings: list[str] = []

        check_advisory_fim(summary, rows, failures)
        check_lidar_observability(summary, rows, failures, warnings)

        self.assertEqual([], failures)

    def test_urg_enabled_summary_validates_required_fields(self) -> None:
        summary = base_summary()
        summary["urg"] = {
            "urg_enabled": True,
            "urg_active": True,
            "urg_query_count": 10,
            "urg_front_field_points": 5,
            "urg_backend_field_points": 6,
            "urg_unknown_count": 1,
            "urg_stale_count": 2,
            "urg_mean_update_ms": 12.5,
            "urg_p95_update_ms": 18.0,
            "urg_time_pl_query_ms": 4.0,
            "urg_time_al_esdf_ms": 2.0,
            "urg_time_pi_ms": 1.0,
            "urg_time_gradient_ms": 3.0,
            "urg_time_csv_ms": 0.5,
            "urg_time_total_ms": 10.5,
        }
        failures: list[str] = []

        check_urg(summary, failures)

        self.assertEqual([], failures)

    def test_urg_disabled_summary_has_zero_counters_and_no_csv(self) -> None:
        summary = base_summary()
        summary["urg"] = {
            "urg_enabled": False,
            "urg_active": False,
            "urg_update_count": 0,
            "urg_query_count": 0,
            "urg_direct_query_count": 0,
            "urg_grid_hit_count": 0,
            "urg_grid_miss_count": 0,
            "urg_stale_count": 0,
            "urg_unknown_count": 0,
            "urg_valid_pi_count": 0,
            "urg_unknown_penalty_count": 0,
            "urg_front_field_points": 0,
            "urg_backend_field_points": 0,
            "urg_voxel_csv": "",
        }
        failures: list[str] = []

        check_urg(summary, failures)

        self.assertEqual([], failures)
        self.assertEqual(0, summary["urg"]["urg_update_count"])
        self.assertEqual(0, summary["urg"]["urg_query_count"])
        self.assertEqual("", summary["urg"]["urg_voxel_csv"])

    def test_ana_log_merge_preserves_online_validator_schema(self) -> None:
        online = base_summary()
        online["advisory_fim"] = {
            "enabled": True,
            "use_lidar_advisory_fim": True,
            "fusion_mode": "fim_add",
            "query_count": 4,
            "epsilon_applied_count": 4,
            "degeneracy_regularized_count": 1,
            "regularized_count": 1,
            "fallback_reason_histogram": {},
            "gnss_fim_valid_count": 2,
            "lidar_fim_valid_count": 4,
        }
        online["pi_stage3"] = {"enabled": True, "input_invalid_count": 0}
        online["urg"] = {
            "urg_enabled": False,
            "urg_active": False,
            "urg_query_count": 0,
            "urg_front_field_points": 0,
            "urg_backend_field_points": 0,
            "urg_unknown_count": 0,
            "urg_stale_count": 0,
            "urg_mean_update_ms": None,
            "urg_p95_update_ms": None,
            "urg_time_pl_query_ms": None,
            "urg_time_al_esdf_ms": None,
            "urg_time_pi_ms": None,
            "urg_time_gradient_ms": None,
            "urg_time_csv_ms": None,
            "urg_time_total_ms": None,
        }
        offline = {
            "aligned_sample_count": 1,
            "predicted_integrity": {"mean_IM": 4.0},
            "actual_alignment": {"matched_count": 1},
            "validation": {"passed": True},
            "rows": [{"sample_abs_time": "1.0"}],
            "online_rows": [{"PL_H_pred": 1.0}],
        }

        merged = merge_online_and_offline_summary(copy.deepcopy(online), offline)

        self.assertEqual([], missing_required_online_fields(merged))
        for field in REQUIRED_ONLINE_SUMMARY_FIELDS:
            self.assertIn(field, merged)
        self.assertEqual("fim_add", merged["advisory_fim"]["fusion_mode"])
        self.assertTrue(merged["pi_stage3"]["enabled"])
        self.assertIn("urg_enabled", merged["urg"])
        self.assertEqual(1, merged["aligned_sample_count"])
        self.assertEqual(1, merged["actual_alignment"]["matched_count"])
        self.assertEqual(4.0, merged["predicted_integrity"]["mean_IM"])
        self.assertIn("fallback_count", merged["predicted_integrity"])


if __name__ == "__main__":
    unittest.main()
