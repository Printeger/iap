#!/usr/bin/env python3
"""Fail-closed contract checks for the committed ICRA-006 offline profile."""

import json
import math
from pathlib import Path
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = (
    REPOSITORY_ROOT / "results/icra27/icra006/p0_provider_profile.json"
)


class Icra006ProviderProfileTest(unittest.TestCase):
    def test_profile_has_exact_shape_and_stable_worker_results(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))

        self.assertEqual(profile["schema_version"], "p0_provider_offline_profile_v1")
        self.assertEqual(profile["status"], "PASS")
        self.assertEqual(profile["build_type"], "RelWithDebInfo")
        self.assertEqual(profile["clock"], "std::chrono::steady_clock")
        self.assertGreaterEqual(profile["cpu_count"], 1)
        self.assertEqual(profile["warmup_iterations"], 2)
        self.assertEqual(profile["measured_iterations"], 7)
        self.assertTrue(all(profile["validation"].values()))

        workload = profile["workload"]
        self.assertEqual(workload["grid_shape"], [40, 40, 8])
        self.assertEqual(workload["logical_position_count"], 12_800)
        self.assertEqual(workload["horizons_s"], [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
        self.assertEqual(workload["logical_query_count"], 76_800)

        reference_checksum = profile["scientific_checksum_fnv1a64"]
        reference_counts = None
        self.assertEqual([item["worker_count"] for item in profile["workers"]], [1, 2, 4])
        for worker in profile["workers"]:
            self.assertEqual(worker["failed_or_nonfinite_iteration_count"], 0)
            self.assertEqual(len(worker["iterations"]), 7)
            for iteration in worker["iterations"]:
                self.assertTrue(iteration["finite"])
                self.assertEqual(iteration["logical_query_count"], 76_800)
                self.assertEqual(iteration["dispatched_predictor_query_count"], 76_800)
                self.assertEqual(iteration["horizon_scientific_mismatch_count"], 0)
                self.assertEqual(iteration["scientific_checksum_fnv1a64"], reference_checksum)
                counts = {
                    "valid_count": iteration["valid_count"],
                    "available_count": iteration["available_count"],
                    "fallback_count": iteration["fallback_count"],
                    "gnss_valid_count": iteration["gnss_valid_count"],
                    "lidar_valid_count": iteration["lidar_valid_count"],
                    "fusion_valid_count": iteration["fusion_valid_count"],
                    "source_flags_histogram": iteration["source_flags_histogram"],
                }
                if reference_counts is None:
                    reference_counts = counts
                self.assertEqual(counts, reference_counts)
                self.assertEqual(iteration["counts"], {
                    "fusion_advisory_invocations": 76_800,
                    "gnss_advisory_invocations": 76_800,
                    "lidar_advisory_invocations": 12_800,
                    "lidar_cache_hits": 64_000,
                    "lidar_evaluations": 12_800,
                    "unique_positions": 12_800,
                })
            for region in worker["summary_ms"].values():
                self.assertTrue(math.isfinite(region["p50_ms"]))
                self.assertTrue(math.isfinite(region["p95_ms"]))
                self.assertGreaterEqual(region["p50_ms"], 0.0)
                self.assertGreaterEqual(region["p95_ms"], region["p50_ms"])


if __name__ == "__main__":
    unittest.main()
