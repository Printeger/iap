#!/usr/bin/env python3
"""Fail-closed contract checks for the committed ICRA-007 profile."""

import json
import math
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = REPOSITORY_ROOT / "results/icra27/icra007/p0_provider_profile.json"


def type7(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


class Icra007ProviderProfileTest(unittest.TestCase):
    def test_profile_separates_runtime_mode_timing_and_semantics(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))

        self.assertEqual(profile["schema_version"], "p0_provider_offline_profile_v2")
        self.assertEqual(profile["diagnostic_execution_status"], "PASS")
        self.assertEqual(profile["p0_horizon_semantic_status"], "MISSING_SIGMA_GROWTH")
        self.assertEqual(
            profile["standards_conformance_status"],
            "BLOCKED_MISSING_SIGMA_GROWTH_AND_PRODUCTION_MAP_LOS",
        )
        self.assertTrue(all(profile["validation"].values()))
        self.assertGreaterEqual(profile["warmup_iterations"], 1)
        self.assertGreaterEqual(profile["measured_iterations"], 5)

        workload = profile["workload"]
        self.assertEqual(workload["grid_shape"], [40, 40, 8])
        self.assertEqual(workload["logical_position_count"], 12_800)
        self.assertEqual(workload["horizons_s"], [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
        self.assertEqual(workload["logical_query_count"], 76_800)
        self.assertEqual(workload["map_los_candidate_occupancy_point_count"], 704)
        self.assertTrue(profile["synthetic_inputs"]["values_are_synthetic"])
        self.assertFalse(
            profile["frozen_runtime_contract"]["production_gnss_local_occupancy_binding"]
        )
        self.assertIn(
            "makeRiskPredictionResult",
            profile["frozen_runtime_contract"]["production_result_conversion"],
        )

        horizon = profile["horizon_semantics"]
        self.assertTrue(horizon["frozen_scientific_fields_invariant"])
        self.assertTrue(horizon["observation_is_not_conformance"])
        self.assertTrue(horizon["whole_result_cross_horizon_reuse_prohibited"])
        whitelist = horizon["scientific_field_whitelist"]
        self.assertEqual(len(whitelist), 91)
        self.assertEqual(len(whitelist), len(set(whitelist)))
        self.assertEqual(
            horizon["metadata_fields_excluded"],
            ["query_position_map", "query_time_s", "horizon_s", "frame_id"],
        )

        gap = profile["production_gap"]
        self.assertTrue(gap["map_based_gnss_occlusion_required_by_conventions"])
        self.assertFalse(gap["current_production_installs_gnss_local_occupancy"])
        self.assertTrue(gap["non_occupancy_inputs_and_parameters_identical_between_modes"])
        self.assertFalse(gap["map_los_candidate_repairs_product_behavior"])
        self.assertFalse(gap["map_los_candidate_absolute_latency_characterizes_icra005"])
        self.assertEqual(
            profile["retained_icra005_authority"]["gate_status"],
            "P0_PERFORMANCE_GATE_FAIL",
        )

        modes = {mode["mode"]: mode for mode in profile["modes"]}
        self.assertEqual(set(modes), {"frozen_runtime", "map_los_candidate"})
        self.assertEqual(modes["frozen_runtime"]["production_label"], "CURRENT_PRODUCTION")
        self.assertTrue(modes["frozen_runtime"]["current_production_contract"])
        self.assertFalse(modes["frozen_runtime"]["gnss_local_occupancy_installed"])
        self.assertEqual(
            modes["map_los_candidate"]["production_label"], "NOT_CURRENT_PRODUCTION"
        )
        self.assertFalse(modes["map_los_candidate"]["current_production_contract"])
        self.assertTrue(modes["map_los_candidate"]["gnss_local_occupancy_installed"])

        for mode in modes.values():
            self.assertTrue(all(mode["validation"].values()))
            self.assertTrue(mode["horizon_scientific_fields_invariant"])
            self.assertEqual([cell["worker_count"] for cell in mode["workers"]], [1, 2, 4])
            worker_one_perturbation = abs(
                mode["workers"][0]["component_timer_perturbation"]["p50_percent"]
            )
            expected_percentage_status = (
                "PERTURBING_DIAGNOSTIC"
                if worker_one_perturbation > 5.0
                else "COST_RANKING_DIAGNOSTIC"
            )
            for cell in mode["workers"]:
                self.assertEqual(
                    cell["component_percentage_status"], expected_percentage_status
                )
                counter = cell["counter_only"]
                component = cell["component_timed"]
                self.assertEqual(counter["phase"], "counter_only_budget")
                self.assertFalse(counter["collect_component_timing"])
                self.assertTrue(counter["budget_timing_authority"])
                self.assertEqual(component["phase"], "component_timed_cost_ranking")
                self.assertTrue(component["collect_component_timing"])
                self.assertFalse(component["budget_timing_authority"])
                self.assertNotIn("p95_within_400_ms_diagnostic_budget", component)

                for phase, timed in ((counter, False), (component, True)):
                    self.assertEqual(len(phase["iterations"]), profile["measured_iterations"])
                    totals = []
                    for iteration in phase["iterations"]:
                        totals.append(iteration["timings_ms"]["total_predictor_provider"])
                        self.assertGreater(
                            iteration["timings_ms"][
                                "scientific_validation_replay_outside_provider_timer"
                            ],
                            0.0,
                        )
                        self.assertTrue(iteration["finite"])
                        self.assertEqual(iteration["collect_component_timing"], timed)
                        self.assertEqual(iteration["logical_query_count"], 76_800)
                        self.assertEqual(iteration["dispatched_predictor_query_count"], 76_800)
                        self.assertEqual(iteration["production_result_conversion_count"], 76_800)
                        self.assertEqual(iteration["scientific_checksum_fnv1a64"], mode["scientific_checksum_fnv1a64"])
                        self.assertEqual(iteration["production_result_checksum_fnv1a64"], mode["production_result_checksum_fnv1a64"])
                        self.assertEqual(iteration["horizon_scientific_mismatch_count"], 0)
                        self.assertEqual(
                            iteration["counts"],
                            {
                                "fusion_advisory_invocations": 76_800,
                                "gnss_advisory_invocations": 76_800,
                                "lidar_advisory_invocations": 12_800,
                                "lidar_cache_hits": 64_000,
                                "lidar_evaluations": 12_800,
                                "unique_positions": 12_800,
                            },
                        )
                        component_durations = [
                            iteration["timings_ms"]["cumulative_gnss_advisory"],
                            iteration["timings_ms"]["cumulative_lidar_advisory"],
                            iteration["timings_ms"]["cumulative_fusion_advisory"],
                        ]
                        if timed:
                            self.assertTrue(all(value > 0.0 for value in component_durations))
                        else:
                            self.assertEqual(component_durations, [0.0, 0.0, 0.0])
                    summary = phase["summary_ms"]["total_predictor_provider"]
                    self.assertAlmostEqual(summary["p50_ms"], type7(totals, 0.50))
                    self.assertAlmostEqual(summary["p95_ms"], type7(totals, 0.95))

                perturbation = cell["component_timer_perturbation"]
                counter_p50 = counter["summary_ms"]["total_predictor_provider"]["p50_ms"]
                component_p50 = component["summary_ms"]["total_predictor_provider"]["p50_ms"]
                self.assertAlmostEqual(
                    perturbation["p50_delta_ms"], component_p50 - counter_p50
                )
                self.assertAlmostEqual(
                    perturbation["p50_percent"],
                    100.0 * (component_p50 - counter_p50) / counter_p50,
                )


if __name__ == "__main__":
    unittest.main()
