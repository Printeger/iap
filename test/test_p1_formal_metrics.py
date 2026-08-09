#!/usr/bin/env python3

import importlib.util
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "p1_formal_metrics.py"
)
SPEC = importlib.util.spec_from_file_location("p1_formal_metrics", MODULE_PATH)
metrics = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(metrics)


def profile(count=200, *, length=10.0, offset=0.0, value=lambda fraction: fraction):
    rows = []
    for index in range(count):
        fraction = index / max(1, count - 1)
        rows.append({
            "sample_index": index,
            "x": offset + length * fraction,
            "y": 0.0,
            "z": 1.0,
            "t_s": 2.0 * fraction,
            "c_pi": value(fraction),
            "matched": 1,
        })
    return rows


class SmoothCvarContractTest(unittest.TestCase):
    def test_decision_checkpoint_requires_one_truth_sample_and_stable_estimate(self):
        truth = [
            {"stamp_s": 1.0, "x": -10.0, "y": 0.0, "z": 1.5},
            {"stamp_s": 2.0, "x": -9.5, "y": 0.0, "z": 1.5},
            {"stamp_s": 3.0, "x": -9.0, "y": 0.0, "z": 1.5},
        ]
        estimate = [{"stamp_s": 2.01, "x": -9.2, "y": 0.0, "z": 1.5}]
        result = metrics.select_decision_checkpoint(truth, estimate)
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["truth"]["x"], -9.5)
        self.assertLessEqual(result["localization_error_m"], 0.5)

        ambiguous = metrics.select_decision_checkpoint(
            truth + [{"stamp_s": 2.1, "x": -9.45, "y": 0.0, "z": 1.5}], estimate
        )
        self.assertFalse(ambiguous["passed"])
        self.assertEqual(ambiguous["status"], "INCONCLUSIVE")

    def test_candidate_route_precheck_requires_both_lanes_and_full_support(self):
        rows = []
        for candidate_id, lane, risk in ((1, "lower", 0.2), (2, "upper", 0.7)):
            for index in range(200):
                rows.append({
                    "candidate_id": candidate_id, "lane": lane,
                    "sample_index": index, "collision_free": 1,
                    "matched": 1, "c_pi": risk,
                    "selected": candidate_id == 1,
                })
        result = metrics.candidate_route_precheck(rows)
        self.assertTrue(result["passed"])
        self.assertEqual(result["lanes"]["lower"]["candidate_count"], 1)
        self.assertEqual(result["lanes"]["upper"]["candidate_count"], 1)
        self.assertEqual(result["selected"]["lane"], "lower")

        insufficient = [row for row in rows if not (
            row["candidate_id"] == 2 and row["sample_index"] == 199
        )]
        self.assertFalse(metrics.candidate_route_precheck(insufficient)["passed"])

    def test_production_golden_vectors_cover_tie_peak_and_extreme_ranges(self):
        self.assertAlmostEqual(metrics.smooth_cvar([7.25] * 200), 7.25, places=12)
        self.assertAlmostEqual(
            metrics.smooth_cvar([1.0] * 199 + [2.0]),
            1.0488372458387318,
            places=12,
        )
        self.assertAlmostEqual(
            metrics.smooth_cvar([-1000.0] * 180 + [1000.0] * 20),
            999.9674917026617,
            places=10,
        )
        result = metrics.smooth_cvar([-1.0e6] * 180 + [1.0e6] * 20)
        self.assertTrue(math.isfinite(result))
        self.assertAlmostEqual(result / 1.0e6, 1.0, places=7)


class FixedArcLatticeContractTest(unittest.TestCase):
    def test_decision_checkpoint_comparison_rejects_non_fixed_support(self):
        current = profile(317, length=12.0, value=lambda f: 10.0 - f)
        reference = profile(73, length=8.0, offset=4.0, value=lambda f: 12.0 - 2.0 * f)
        with self.assertRaisesRegex(ValueError, "200/200"):
            metrics.compare_profiles(current, reference)
        comparison = metrics.compare_profiles(profile(), profile(value=lambda f: 2.0 - f))
        self.assertEqual(comparison["comparison_mode"], "first_deterministic_decision_checkpoint_fixed_200")
        self.assertFalse(comparison["derived"])
        self.assertEqual(comparison["sample_count"], 200)
        self.assertIsNone(comparison["common_terminal_arc_length_m"])
        self.assertEqual(len(comparison["current_lattice"]), 200)
        self.assertEqual(len(comparison["reference_lattice"]), 200)

    def test_zero_length_and_low_coverage_fail_closed(self):
        zero = profile(200, length=0.0, value=lambda _f: 1.0)
        with self.assertRaisesRegex(ValueError, "positive arc length"):
            metrics.compare_profiles(zero, profile())
        low_coverage = profile()
        low_coverage[-1]["matched"] = 0
        with self.assertRaisesRegex(ValueError, "complete matched support"):
            metrics.compare_profiles(low_coverage, profile())

    def test_sampling_sufficiency_and_resampling_residual_are_reported(self):
        comparison = metrics.compare_profiles(profile(), profile(value=lambda f: 1.0 + f))
        sufficiency = metrics.sampling_sufficiency(
            profile(), resolution_m=0.75, horizons_s=[0.0, 0.5, 1.0, 1.5, 2.0]
        )
        self.assertTrue(sufficiency["passed"])
        self.assertLessEqual(sufficiency["max_spatial_gap_m"], 0.375)
        self.assertLessEqual(sufficiency["max_temporal_gap_s"], 0.25)
        self.assertLess(comparison["epsilon_resample"], 1.0e-12)


class FormalEffectivenessContractTest(unittest.TestCase):
    def test_improvement_thresholds_are_strict_and_max_bound_is_inclusive(self):
        thresholds = {"tau_mean": 0.1, "tau_cvar": 0.2, "tau_max": 0.3}
        equal_improvement = metrics.evaluate_effectiveness(
            current_mean=0.9,
            reference_mean=1.0,
            current_cvar=1.8,
            reference_cvar=2.0,
            current_max=2.3,
            reference_max=2.0,
            thresholds=thresholds,
        )
        self.assertFalse(equal_improvement["passed"])
        self.assertAlmostEqual(equal_improvement["max_remaining_margin"], 0.0)

        strict_improvement = metrics.evaluate_effectiveness(
            current_mean=0.899,
            reference_mean=1.0,
            current_cvar=1.799,
            reference_cvar=2.0,
            current_max=2.3,
            reference_max=2.0,
            thresholds=thresholds,
        )
        self.assertTrue(strict_improvement["passed"])

        max_exceeded = metrics.evaluate_effectiveness(
            current_mean=0.8,
            reference_mean=1.0,
            current_cvar=1.7,
            reference_cvar=2.0,
            current_max=2.3000001,
            reference_max=2.0,
            thresholds=thresholds,
        )
        self.assertFalse(max_exceeded["passed"])

        max_reduced = metrics.evaluate_effectiveness(
            current_mean=0.8,
            reference_mean=1.0,
            current_cvar=1.7,
            reference_cvar=2.0,
            current_max=1.9,
            reference_max=2.0,
            thresholds=thresholds,
        )
        self.assertTrue(max_reduced["passed"])


class CalibrationBindingContractTest(unittest.TestCase):
    def test_binding_requires_sha_identity_prelaunch_generation_and_matching_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            calibration = {
                "schema_version": "p1_formal_tolerance_calibration_v1",
                "calibration_id": "cal-1",
                "generated_at_epoch_s": 100.0,
                "git_commit": "head",
                "baseline_commit": "baseline",
                "scenario": "scenario",
                "scenario_fingerprint": "sha256:scenario",
                "scenario_contract": {"fixture": "fork"},
                "experiment": "p1_degraded_lidar_good",
                "runtime_hashes": {
                    "launch": "launch", "planner_executable": "planner",
                    "bspline_library": "bspline",
                },
                "p0": {"resolution_m": 0.75, "horizons_s": [0.0, 0.5, 1.0]},
                "smooth_cvar": {
                    "mode": "fixed_200_smooth_cvar", "alpha": 0.90,
                    "temperature": 0.01, "eta_bisection_iterations": 100,
                },
                "lambda_integrity": 0.00001,
                "normalization_budget_fraction": 0.30,
                "configuration_identity": {
                    "experiment": "p1_degraded_lidar_good",
                    "planner_safety_profile": "p1",
                    "p0.enable_risk_grid": True,
                    "p0.resolution_m": 0.75,
                    "p0.horizons_s": [0.0, 0.5, 1.0],
                    "p1.use_integrity_cost": True,
                    "p1.max_candidates_per_attempt": 8,
                    "p1.lambda_integrity": 0.00001,
                    "p1.objective_aggregation_mode": "fixed_200_smooth_cvar",
                    "p1.smooth_cvar_alpha": 0.90,
                    "p1.smooth_max_temperature": 0.01,
                    "p1.normalization_budget_fraction": 0.30,
                    "grid_map/local_update_range_x": 10.0,
                    "scenario_fingerprint": "sha256:scenario",
                    "scenario_contract": {"fixture": "fork"},
                    "run_duration_s": 90.0,
                    "validation_duration_s": 90.0,
                    "planner_start_delay_s": 10.0,
                    "manager/max_vel": 1.0,
                    "optimization/max_vel": 1.0,
                    "bspline/limit_vel": 1.0,
                    "fsm.thresh_replan_time": 0.5,
                    "record_bag": False,
                    "run_validator": True,
                },
                "conformal": {"pair_count": 10, "coverage": 0.90},
                "run_ids": [f"null-{index}" for index in range(20)],
                "pairs": [
                    {
                        "pair_id": f"pair-{index}", "valid": True,
                        "run_a_id": f"null-{2 * index}",
                        "run_b_id": f"null-{2 * index + 1}",
                        "run_a_mean": 1.0, "run_b_mean": 0.996, "s_mean": 0.004,
                        "run_a_cvar": 1.0, "run_b_cvar": 0.986, "s_cvar": 0.014,
                        "run_a_max": 1.0, "run_b_max": 0.976, "s_max": 0.024,
                    }
                    for index in range(10)
                ],
                "null_effect_maxima": {
                    "s_mean": 0.004, "s_cvar": 0.014, "s_max": 0.024,
                },
                "deterministic_error": {
                    "epsilon_grid": 0.001, "epsilon_resample": 0.002,
                    "epsilon_det": 0.006,
                },
                "thresholds": {"tau_mean": 0.01, "tau_cvar": 0.02, "tau_max": 0.03},
            }
            path.write_text(json.dumps(calibration))
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            manifest = {
                "scenario": "scenario",
                "scenario_fingerprint": "sha256:scenario",
                "scenario_contract": {"fixture": "fork"},
                "experiment": "p1_degraded_lidar_good",
                "p0.resolution_m": 0.75,
                "p0.horizons_s": [0.0, 0.5, 1.0],
                "p1.lambda_integrity": 0.00001,
                "p1.objective_aggregation_mode": "fixed_200_smooth_cvar",
                "p1.smooth_cvar_alpha": 0.90,
                "p1.smooth_max_temperature": 0.01,
                "p1.normalization_budget_fraction": 0.30,
                "grid_map/local_update_range_x": 10.0,
                "planner_safety_profile": "p1",
                "p0.enable_risk_grid": True,
                "p1.use_integrity_cost": True,
                "p1.max_candidates_per_attempt": 8,
                "run_duration_s": 90.0,
                "validation_duration_s": 90.0,
                "planner_start_delay_s": 10.0,
                "manager/max_vel": 1.0,
                "optimization/max_vel": 1.0,
                "bspline/limit_vel": 1.0,
                "fsm.thresh_replan_time": 0.5,
                "record_bag": True,
                "run_validator": True,
                "artifact_provenance": {
                    "run_id": "formal-1", "git_commit": "head",
                    "baseline_commit": "baseline", "process_start_epoch_s": 101.0,
                    "runtime_paths": {
                        "launch": {"sha256": "launch"},
                        "planner_executable": {"sha256": "planner"},
                        "bspline_library": {"sha256": "bspline"},
                    },
                },
                "p1.formal_calibration": {
                    "calibration_id": "cal-1", "path": str(path), "sha256": digest,
                },
            }
            result = metrics.validate_calibration_binding(path, manifest)
            self.assertTrue(result["passed"], result["errors"])

            manifest["artifact_provenance"]["process_start_epoch_s"] = 99.0
            result = metrics.validate_calibration_binding(path, manifest)
            self.assertFalse(result["passed"])
            self.assertTrue(any("before formal run" in error for error in result["errors"]))

            manifest["artifact_provenance"]["process_start_epoch_s"] = 101.0
            manifest["p1.formal_calibration"]["sha256"] = "wrong"
            result = metrics.validate_calibration_binding(path, manifest)
            self.assertFalse(result["passed"])
            self.assertTrue(any("SHA256" in error for error in result["errors"]))

            manifest["p1.formal_calibration"]["sha256"] = digest
            manifest["run_duration_s"] = 30.0
            result = metrics.validate_calibration_binding(path, manifest)
            self.assertFalse(result["passed"])
            self.assertTrue(any("run_duration_s" in error for error in result["errors"]))

            manifest["run_duration_s"] = 90.0
            calibration["thresholds"]["tau_max"] = 99.0
            path.write_text(json.dumps(calibration))
            manifest["p1.formal_calibration"]["sha256"] = hashlib.sha256(
                path.read_bytes()).hexdigest()
            result = metrics.validate_calibration_binding(path, manifest)
            self.assertFalse(result["passed"])
            self.assertTrue(any("threshold formula" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
