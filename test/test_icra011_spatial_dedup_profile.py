#!/usr/bin/env python3
"""Fail-closed contract for the ICRA-011 phase-2 offline diagnostic."""

import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROFILE = (
    ROOT
    / "results"
    / "icra27"
    / "icra011"
    / "p0_phase2_spatial_dedup_profile.json"
)

LOGICAL_POSITIONS = 12_800
LOGICAL_QUERIES = 76_800
SPATIAL_REUSES = 64_000


class Icra011SpatialDedupProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not PROFILE.is_file():
            raise AssertionError(f"missing ICRA-011 profile: {PROFILE}")
        cls.profile = json.loads(PROFILE.read_text(encoding="utf-8"))

    def test_schema_status_and_workload_are_phase2_truthful(self):
        profile = self.profile
        self.assertEqual(
            profile["schema_version"], "p0_phase2_spatial_dedup_profile_v1"
        )
        self.assertEqual(profile["diagnostic_execution_status"], "PASS")
        self.assertEqual(
            profile["phase2_semantic_status"],
            "WITHIN_REFRESH_SPATIAL_ADVISORY_DEDUP_VALIDATED",
        )
        self.assertEqual(
            profile["production_semantic_status"],
            "PHASE1_MAP_LOS_AND_COVARIANCE_GROWTH_PRESENT",
        )
        self.assertEqual(profile["latency_status"], "COST_RANKING_DIAGNOSTIC")
        self.assertEqual(profile["gate0b_qualification_status"], "NOT_RUN")
        self.assertEqual(
            profile["production_calibration_status"],
            "UNSET_PROFILE_SYNTHETIC_ONLY",
        )

        workload = profile["workload"]
        self.assertEqual(workload["grid_shape"], [40, 40, 8])
        self.assertEqual(workload["resolution_m"], 0.75)
        self.assertEqual(workload["horizons_s"], [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
        self.assertEqual(workload["logical_position_count"], LOGICAL_POSITIONS)
        self.assertEqual(workload["logical_query_count"], LOGICAL_QUERIES)
        self.assertEqual(workload["map_los_occupancy_point_count"], 704)
        self.assertEqual(
            profile["synthetic_inputs"]["sigma_grow_m_sqrt_s"], 0.15
        )
        self.assertTrue(profile["phase2_profile_contract"]["map_los_installed"])
        self.assertTrue(
            profile["phase2_profile_contract"]["covariance_growth_enabled"]
        )

        fields = set(profile["scalar_equivalence_field_whitelist"])
        for required in {
            "PredictorQueryResult.query_position_map",
            "PredictorQueryResult.query_time_s",
            "PredictorQueryResult.horizon_s",
            "PredictorQueryResult.frame_id",
            "PredictorQueryResult.covariance_growth_status",
        }:
            self.assertIn(required, fields)

    def test_workers_iterations_counts_and_scalar_science_fail_closed(self):
        profile = self.profile
        self.assertGreaterEqual(profile["warmup_iterations"], 1)
        measured = profile["measured_iterations"]
        self.assertGreaterEqual(measured, 5)
        self.assertEqual(len(profile["modes"]), 1)
        mode = profile["modes"][0]
        self.assertEqual(mode["mode"], "phase2_map_los")
        self.assertEqual(mode["latency_status"], "COST_RANKING_DIAGNOSTIC")
        self.assertTrue(mode["validation"]["scalar_equivalence_exact"])
        self.assertTrue(mode["validation"]["exact_phase2_counts"])
        self.assertEqual(
            {worker["worker_count"] for worker in mode["workers"]}, {1, 2, 4}
        )

        science_hashes = set()
        production_hashes = set()
        for worker in mode["workers"]:
            self.assertEqual(
                worker["latency_status"], "COST_RANKING_DIAGNOSTIC"
            )
            for phase_name in ("counter_only", "component_timed"):
                phase = worker[phase_name]
                self.assertEqual(
                    phase["latency_status"], "COST_RANKING_DIAGNOSTIC"
                )
                self.assertFalse(phase["qualification_authority"])
                self.assertEqual(len(phase["iterations"]), measured)
                for summary in phase["summary_ms"].values():
                    self.assertEqual(
                        summary["percentile_method"],
                        "R-7 linear interpolation",
                    )
                    for key in ("p50_ms", "p95_ms"):
                        self.assertTrue(math.isfinite(summary[key]))
                        self.assertGreaterEqual(summary[key], 0.0)
                for iteration in phase["iterations"]:
                    self.assertTrue(iteration["finite"])
                    self.assertEqual(
                        iteration["logical_query_count"], LOGICAL_QUERIES
                    )
                    self.assertEqual(
                        iteration["dispatched_predictor_query_count"],
                        LOGICAL_QUERIES,
                    )
                    self.assertEqual(
                        iteration["production_result_conversion_count"],
                        LOGICAL_QUERIES,
                    )
                    self.assertEqual(
                        iteration["scalar_scientific_mismatch_count"], 0
                    )
                    counts = iteration["counts"]
                    self.assertEqual(counts["query_count"], LOGICAL_QUERIES)
                    self.assertEqual(
                        counts["unique_positions"], LOGICAL_POSITIONS
                    )
                    self.assertEqual(
                        counts["spatial_advisory_recompute_count"],
                        LOGICAL_POSITIONS,
                    )
                    self.assertEqual(
                        counts["spatial_advisory_reuse_count"], SPATIAL_REUSES
                    )
                    self.assertEqual(
                        counts["gnss_advisory_invocations"], LOGICAL_POSITIONS
                    )
                    self.assertEqual(
                        counts["lidar_advisory_invocations"], LOGICAL_POSITIONS
                    )
                    self.assertEqual(
                        counts["lidar_evaluations"], LOGICAL_POSITIONS
                    )
                    self.assertEqual(counts["lidar_cache_hits"], SPATIAL_REUSES)
                    self.assertEqual(
                        counts["fusion_advisory_invocations"], LOGICAL_QUERIES
                    )
                    science_hashes.add(iteration["scientific_checksum_fnv1a64"])
                    production_hashes.add(
                        iteration["production_result_checksum_fnv1a64"]
                    )

        self.assertEqual(len(science_hashes), 1)
        self.assertEqual(len(production_hashes), 1)
        self.assertTrue(
            profile["validation"]["scientific_checksums_stable_across_workers"]
        )
        self.assertTrue(
            profile["validation"]["production_checksums_stable_across_workers"]
        )
        self.assertTrue(profile["validation"]["zero_scalar_mismatches"])


if __name__ == "__main__":
    unittest.main()
