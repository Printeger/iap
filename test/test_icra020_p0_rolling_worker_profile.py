#!/usr/bin/env python3
"""Fail-closed contract for the ICRA-020 Stage-5 P0 diagnostic."""

import hashlib
import json
import math
from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
PROFILE = (
    ROOT
    / "results"
    / "icra27"
    / "icra020"
    / "p0_rolling_worker_profile.json"
)
PROFILE_SHA256 = (
    "2f68e3123426b5a1117e86bb5abc7c69117a070bcf583ec759974fddeb71a0bd"
)

SCHEMA = "p0_rolling_stage5_profile_v1"
WORKERS = {1, 2, 4}
SCENARIOS = {
    "cold_full_rebuild": {
        "spatial_recompute_count": 12_800,
        "spatial_reuse_count": 64_000,
        "retained_position_count": 0,
        "entered_position_count": 12_800,
        "evicted_position_count": 0,
        "gnss_advisory_invocation_count": 12_800,
        "lidar_advisory_invocation_count": 12_800,
        "horizon_fusion_count": 76_800,
        "full_rebuild": True,
        "full_invalidation_count": 0,
        "invalidation_reason": "uninitialized",
        "prior_generation": 1,
        "occupancy_generation": 2,
        "occupancy_content_identity": 1,
        "snapshot_occupancy_generation": 2,
        "snapshot_occupancy_stamp_s": 100.0,
    },
    "stationary_empty_delta": {
        "spatial_recompute_count": 0,
        "spatial_reuse_count": 76_800,
        "retained_position_count": 12_800,
        "entered_position_count": 0,
        "evicted_position_count": 0,
        "gnss_advisory_invocation_count": 0,
        "lidar_advisory_invocation_count": 0,
        "horizon_fusion_count": 76_800,
        "full_rebuild": False,
        "full_invalidation_count": 0,
        "invalidation_reason": "none",
        "prior_generation": 2,
        "occupancy_generation": 3,
        "occupancy_content_identity": 1,
        "snapshot_occupancy_generation": 3,
        "snapshot_occupancy_stamp_s": 100.5,
    },
    "shift_plus_one_x_empty_delta": {
        "spatial_recompute_count": 320,
        "spatial_reuse_count": 76_480,
        "retained_position_count": 12_480,
        "entered_position_count": 320,
        "evicted_position_count": 320,
        "gnss_advisory_invocation_count": 320,
        "lidar_advisory_invocation_count": 320,
        "horizon_fusion_count": 76_800,
        "full_rebuild": False,
        "full_invalidation_count": 0,
        "invalidation_reason": "none",
        "prior_generation": 2,
        "occupancy_generation": 3,
        "occupancy_content_identity": 1,
        "snapshot_occupancy_generation": 3,
        "snapshot_occupancy_stamp_s": 100.5,
    },
    "stationary_nonempty_delta": {
        "spatial_recompute_count": 12_800,
        "spatial_reuse_count": 64_000,
        "retained_position_count": 0,
        "entered_position_count": 12_800,
        "evicted_position_count": 12_800,
        "gnss_advisory_invocation_count": 12_800,
        "lidar_advisory_invocation_count": 12_800,
        "horizon_fusion_count": 76_800,
        "full_rebuild": True,
        "full_invalidation_count": 1,
        "invalidation_reason": "occupancy_source_changed",
        "prior_generation": 2,
        "occupancy_generation": 3,
        "occupancy_content_identity": 2,
        "snapshot_occupancy_generation": 3,
        "snapshot_occupancy_stamp_s": 100.5,
    },
}

TIMING_FIELDS = (
    "wall_ms",
    "refresh_elapsed_ms",
    "provider_batch_duration_ms",
)

IMPLEMENTATION_FILES = (
    "CMakeLists.txt",
    "src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp",
)

TEST_BINARY_PATH = "results/icra27/icra020/build_ego/test_p0_risk_grid_runtime"
LIBIAP_PATH = "results/icra27/icra020/install/lib/libiap.so"
PROFILE_OUTPUT_PATH = "results/icra27/icra020/p0_rolling_worker_profile.json"

ROOT_KEYS = {
    "schema_version",
    "diagnostic_execution_status",
    "latency_status",
    "gate0b_qualification_status",
    "production_worker_selection_status",
    "reverse_ray_decision_status",
    "gpu_status",
    "percentile_method",
    "warmup_samples_per_scenario",
    "measured_samples_per_scenario",
    "provenance",
    "workload",
    "workers",
    "validation",
}

SAMPLE_KEYS = {
    "sample_index",
    "refresh_succeeded",
    "scientifically_equal_to_fresh",
    *TIMING_FIELDS,
    "logical_query_count",
    "provider_query_count",
    "spatial_recompute_count",
    "spatial_reuse_count",
    "retained_position_count",
    "entered_position_count",
    "evicted_position_count",
    "gnss_advisory_invocation_count",
    "lidar_advisory_invocation_count",
    "horizon_fusion_count",
    "full_rebuild",
    "full_invalidation_count",
    "invalidation_reason",
    "gnss_generation",
    "lidar_generation",
    "prior_generation",
    "occupancy_generation",
    "occupancy_content_identity",
    "snapshot_occupancy_generation",
    "snapshot_occupancy_stamp_s",
    "snapshot_scientific_hash_fnv1a64",
    "fresh_scientific_hash_fnv1a64",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile_r7(values, probability):
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = math.floor(position)
    fraction = position - lower
    if lower + 1 == len(ordered):
        return ordered[lower]
    return ordered[lower] + fraction * (ordered[lower + 1] - ordered[lower])


def validate_recorded_commit_provenance(implementation_sha, implementation_files):
    subprocess.run(
        ["git", "cat-file", "-e", f"{implementation_sha}^{{commit}}"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    for path in implementation_files:
        object_type = subprocess.run(
            ["git", "cat-file", "-t", f"{implementation_sha}:{path}"],
            cwd=ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        if object_type != "blob":
            raise AssertionError(
                f"recorded implementation path is not a blob: {path}"
            )


class Icra020P0RollingWorkerProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not PROFILE.is_file():
            raise AssertionError(f"missing ICRA-020 profile: {PROFILE}")
        if sha256(PROFILE) != PROFILE_SHA256:
            raise AssertionError(f"modified ICRA-020 profile: {PROFILE}")
        cls.profile = json.loads(PROFILE.read_text(encoding="utf-8"))

    def test_recorded_commit_provenance_accepts_required_blobs(self):
        validate_recorded_commit_provenance(
            self.profile["provenance"]["implementation_sha"],
            IMPLEMENTATION_FILES,
        )

    def test_recorded_commit_provenance_rejects_nonexistent_commit(self):
        with self.assertRaises(subprocess.CalledProcessError):
            validate_recorded_commit_provenance(
                "0" * 40,
                IMPLEMENTATION_FILES,
            )

    def test_recorded_commit_provenance_rejects_missing_path(self):
        with self.assertRaises(subprocess.CalledProcessError):
            validate_recorded_commit_provenance(
                self.profile["provenance"]["implementation_sha"],
                (*IMPLEMENTATION_FILES, "missing/icra020/source.cpp"),
            )

    def test_canonical_profile_hash_is_frozen(self):
        self.assertEqual(sha256(PROFILE), PROFILE_SHA256)

    def test_profile_is_complete_truthful_and_reproducible(self):
        profile = self.profile
        self.assertEqual(set(profile), ROOT_KEYS)
        self.assertEqual(profile["schema_version"], SCHEMA)
        self.assertEqual(profile["diagnostic_execution_status"], "PASS")
        self.assertEqual(profile["latency_status"], "COST_RANKING_DIAGNOSTIC")
        self.assertEqual(profile["gate0b_qualification_status"], "NOT_RUN")
        self.assertEqual(profile["production_worker_selection_status"], "NOT_SELECTED")
        self.assertEqual(
            profile["reverse_ray_decision_status"], "SUPERVISOR_REVIEW_PENDING"
        )
        self.assertEqual(profile["gpu_status"], "NOT_EVALUATED")
        self.assertNotIn("400", json.dumps(profile.get("validation", {})))
        self.assertEqual(profile["percentile_method"], "R-7 linear interpolation")
        self.assertGreaterEqual(profile["warmup_samples_per_scenario"], 2)
        self.assertGreaterEqual(profile["measured_samples_per_scenario"], 10)

        workload = profile["workload"]
        self.assertEqual(
            set(workload),
            {
                "profile_path",
                "grid_shape",
                "resolution_m",
                "fixed_map_lattice_anchor",
                "horizons_s",
                "logical_position_count",
                "logical_risk_voxel_count",
                "predictor_source_mode",
                "gnss_epoch_policy",
                "satellite_count",
                "map_los_occupied_voxel_count",
                "lidar_fim_primitive_count",
                "lidar_map_point_count",
                "sigma_grow_m_sqrt_s",
                "skip_occupied_voxels",
                "requested_effective_worker_pairs",
                "ttl_watchdog_policy",
                "affinity_scheduler_clock_manipulation",
                "fresh_runtime_per_sample",
                "untimed_base_per_non_cold_sample",
                "fresh_scientific_validation_outside_measured_interval",
                "fixture_sources",
            },
        )
        self.assertEqual(
            workload["profile_path"],
            "production P0RiskGridRuntime::refreshOnceForTest",
        )
        self.assertEqual(workload["grid_shape"], [40, 40, 8])
        self.assertEqual(workload["resolution_m"], 0.75)
        self.assertEqual(workload["fixed_map_lattice_anchor"], [0.0, 0.0, 0.0])
        self.assertEqual(workload["horizons_s"], [0.0, 0.5, 1.0, 1.5, 2.0, 2.5])
        self.assertEqual(workload["logical_position_count"], 12_800)
        self.assertEqual(workload["logical_risk_voxel_count"], 76_800)
        self.assertEqual(workload["predictor_source_mode"], "Fusion")
        self.assertEqual(workload["gnss_epoch_policy"], "Required")
        self.assertEqual(workload["satellite_count"], 31)
        self.assertEqual(workload["map_los_occupied_voxel_count"], 704)
        self.assertEqual(workload["lidar_fim_primitive_count"], 704)
        self.assertEqual(workload["lidar_map_point_count"], 23_309)
        self.assertEqual(workload["sigma_grow_m_sqrt_s"], 0.15)
        self.assertFalse(workload["skip_occupied_voxels"])
        self.assertEqual(workload["requested_effective_worker_pairs"], [[1, 1], [2, 2], [4, 4]])
        self.assertEqual(workload["ttl_watchdog_policy"], "DISABLED_TEST_ONLY")
        self.assertEqual(workload["affinity_scheduler_clock_manipulation"], "NONE")
        self.assertTrue(workload["fresh_runtime_per_sample"])
        self.assertTrue(workload["untimed_base_per_non_cold_sample"])
        self.assertTrue(
            workload["fresh_scientific_validation_outside_measured_interval"]
        )
        self.assertEqual(
            set(workload["fixture_sources"]),
            {"gnss", "occupancy", "lidar_fim", "lidar_map"},
        )

        provenance = profile["provenance"]
        self.assertEqual(
            set(provenance),
            {
                "implementation_sha",
                "compiler",
                "build_type",
                "clock",
                "cpu_model",
                "logical_core_count",
                "exact_command",
                "test_binary",
                "libiap",
            },
        )
        implementation_sha = provenance["implementation_sha"]
        self.assertRegex(implementation_sha, r"^[0-9a-f]{40}$")
        validate_recorded_commit_provenance(
            implementation_sha,
            IMPLEMENTATION_FILES,
        )
        for key in ("test_binary", "libiap"):
            item = provenance[key]
            self.assertEqual(set(item), {"repository_relative_path", "sha256"})
            self.assertRegex(item["sha256"], r"^[0-9a-f]{64}$")
            path = (ROOT / item["repository_relative_path"]).resolve()
            self.assertTrue(path.is_relative_to(ROOT.resolve()))
            if path.exists():
                self.assertTrue(path.is_file(), path)
                self.assertEqual(sha256(path), item["sha256"])
        self.assertEqual(
            provenance["test_binary"]["repository_relative_path"],
            TEST_BINARY_PATH,
        )
        self.assertEqual(
            provenance["libiap"]["repository_relative_path"], LIBIAP_PATH
        )
        self.assertTrue(provenance["compiler"])
        self.assertEqual(provenance["build_type"], "RelWithDebInfo")
        self.assertTrue(provenance["cpu_model"])
        self.assertGreaterEqual(provenance["logical_core_count"], 1)
        profile_filter = (
            "P0RiskGridRuntimeStampTest."
            "DISABLED_ICRA020_ProductionRuntimeWorkerScalingProfile"
        )
        expected_command = (
            f"env IAP_ICRA020_PROFILE_OUTPUT={PROFILE_OUTPUT_PATH} "
            f"IAP_ICRA020_IMPLEMENTATION_SHA={implementation_sha} "
            "IAP_ICRA020_BUILD_TYPE=RelWithDebInfo "
            f"IAP_ICRA020_TEST_BINARY_PATH={TEST_BINARY_PATH} "
            "IAP_ICRA020_TEST_BINARY_SHA256="
            f"{provenance['test_binary']['sha256']} "
            f"IAP_ICRA020_LIBIAP_PATH={LIBIAP_PATH} "
            f"IAP_ICRA020_LIBIAP_SHA256={provenance['libiap']['sha256']} "
            f"{TEST_BINARY_PATH} --gtest_also_run_disabled_tests "
            f"--gtest_filter={profile_filter}"
        )
        self.assertEqual(provenance["exact_command"], expected_command)

        workers = profile["workers"]
        self.assertEqual({row["requested_worker_count"] for row in workers}, WORKERS)
        self.assertEqual(len(workers), 3)
        scenario_hashes = {name: set() for name in SCENARIOS}
        measured_count = profile["measured_samples_per_scenario"]
        for worker in workers:
            self.assertEqual(
                set(worker),
                {"requested_worker_count", "effective_worker_count", "scenarios"},
            )
            requested = worker["requested_worker_count"]
            self.assertEqual(worker["effective_worker_count"], requested)
            scenarios = worker["scenarios"]
            self.assertEqual({row["scenario"] for row in scenarios}, set(SCENARIOS))
            self.assertEqual(len(scenarios), 4)
            for scenario in scenarios:
                self.assertEqual(
                    set(scenario),
                    {
                        "scenario",
                        "samples",
                        "summary_ms",
                        "speedup_vs_worker_1_p50",
                    },
                )
                expected = SCENARIOS[scenario["scenario"]]
                samples = scenario["samples"]
                self.assertEqual(len(samples), measured_count)
                self.assertEqual(
                    [sample["sample_index"] for sample in samples],
                    list(range(measured_count)),
                )
                for sample in samples:
                    self.assertEqual(set(sample), SAMPLE_KEYS)
                    self.assertTrue(sample["refresh_succeeded"])
                    self.assertTrue(sample["scientifically_equal_to_fresh"])
                    self.assertEqual(sample["logical_query_count"], 76_800)
                    self.assertEqual(sample["provider_query_count"], 76_800)
                    self.assertEqual(sample["gnss_generation"], 1)
                    self.assertEqual(sample["lidar_generation"], 1)
                    for field, value in expected.items():
                        self.assertEqual(sample[field], value)
                    for field in TIMING_FIELDS:
                        self.assertTrue(math.isfinite(sample[field]))
                        self.assertGreaterEqual(sample[field], 0.0)
                    self.assertRegex(sample["snapshot_scientific_hash_fnv1a64"], r"^[0-9a-f]{16}$")
                    self.assertEqual(
                        sample["snapshot_scientific_hash_fnv1a64"],
                        sample["fresh_scientific_hash_fnv1a64"],
                    )
                    scenario_hashes[scenario["scenario"]].add(
                        sample["snapshot_scientific_hash_fnv1a64"]
                    )

                summary = scenario["summary_ms"]
                self.assertEqual(set(summary), set(TIMING_FIELDS))
                for field in TIMING_FIELDS:
                    self.assertEqual(
                        set(summary[field]),
                        {"percentile_method", "p50_ms", "p95_ms", "max_ms"},
                    )
                    values = [sample[field] for sample in samples]
                    self.assertEqual(summary[field]["percentile_method"], profile["percentile_method"])
                    self.assertAlmostEqual(summary[field]["p50_ms"], percentile_r7(values, 0.50), places=9)
                    self.assertAlmostEqual(summary[field]["p95_ms"], percentile_r7(values, 0.95), places=9)
                    self.assertAlmostEqual(summary[field]["max_ms"], max(values), places=9)
                speedup = scenario["speedup_vs_worker_1_p50"]
                self.assertEqual(set(speedup), set(TIMING_FIELDS))
                for value in speedup.values():
                    self.assertTrue(math.isfinite(value))
                    self.assertGreater(value, 0.0)

        for hashes in scenario_hashes.values():
            self.assertEqual(len(hashes), 1)
        worker_by_count = {
            worker["requested_worker_count"]: worker for worker in workers
        }
        baseline_by_scenario = {
            scenario["scenario"]: scenario
            for scenario in worker_by_count[1]["scenarios"]
        }
        for worker in workers:
            for scenario in worker["scenarios"]:
                baseline = baseline_by_scenario[scenario["scenario"]]
                for field in TIMING_FIELDS:
                    expected_speedup = (
                        baseline["summary_ms"][field]["p50_ms"]
                        / scenario["summary_ms"][field]["p50_ms"]
                    )
                    self.assertAlmostEqual(
                        scenario["speedup_vs_worker_1_p50"][field],
                        expected_speedup,
                        places=9,
                    )
        validation = profile["validation"]
        self.assertEqual(
            set(validation),
            {
                "complete_worker_scenario_matrix",
                "all_refreshes_succeeded",
                "all_timings_finite",
                "exact_work_contracts",
                "scientific_equivalence_to_fresh",
                "scientific_hashes_stable_across_samples_and_workers",
                "current_source_provenance_recorded",
            },
        )
        for key in (
            "complete_worker_scenario_matrix",
            "all_refreshes_succeeded",
            "all_timings_finite",
            "exact_work_contracts",
            "scientific_equivalence_to_fresh",
            "scientific_hashes_stable_across_samples_and_workers",
            "current_source_provenance_recorded",
        ):
            self.assertIs(validation[key], True)
        for forbidden in (
            "gate_pass",
            "gate_go",
            "latency_pass",
            "worker_selected",
            "reverse_ray_authorized",
            "gpu_ready",
        ):
            self.assertNotIn(forbidden, validation)


if __name__ == "__main__":
    unittest.main()
