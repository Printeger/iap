#!/usr/bin/env python3

import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "calibrate_p1_formal_tolerances.py"
)
SPEC = importlib.util.spec_from_file_location("calibrate_p1_formal_tolerances", MODULE_PATH)
calibration = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(calibration)


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


class CalibrationToolContractTest(unittest.TestCase):
    def test_decision_profile_rejects_multiple_sequences_in_window(self):
        manifest = {"scenario_contract": {"decision_checkpoint": {
            "truth_source_topic": "/sim/drone_0/truth_odom",
            "profile_sample_zero_binding": "planner_truth_odom_state_at_planning_start",
        }}}
        rows = []
        contexts = []
        for sequence, x in ((1, -9.8), (2, -9.4)):
            contexts.append({"profile_seq": str(sequence), "planning_start_s": "1.0"})
            rows.extend({"profile_seq": str(sequence), "sample_index": str(index), "x": str(x)}
                        for index in range(200))
        with self.assertRaisesRegex(calibration.CalibrationError, "missing/ambiguous"):
            calibration._decision_profile(rows, contexts, manifest)

    def test_decision_profile_selects_only_authoritative_reference_at_one_event(self):
        manifest = {"scenario_contract": {"decision_checkpoint": {
            "truth_source_topic": "/sim/drone_0/truth_odom",
            "profile_sample_zero_binding": "planner_truth_odom_state_at_planning_start",
        }}}
        rows = []
        contexts = []
        for sequence, reason in ((1, "metrics_only_reference_observation"),
                                 (2, "metrics_only_coverage_insufficient")):
            contexts.append({"profile_seq": str(sequence), "planning_start_s": "1.0"})
            rows.extend({"profile_seq": str(sequence), "sample_index": str(index),
                         "x": "-9.5", "fallback_reason": reason}
                        for index in range(200))
        selected = calibration._decision_profile(rows, contexts, manifest)
        self.assertEqual({row["profile_seq"] for row in selected}, {"1"})

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.exports = []
        for index in range(20):
            self.exports.append(self.make_export(index))
        pairs = [
            {
                "pair_id": f"null-{index + 1:02d}",
                "run_a_export": str(self.exports[2 * index]),
                "run_b_export": str(self.exports[2 * index + 1]),
            }
            for index in range(10)
        ]
        self.pairs_path = self.root / "pairs.json"
        self.pairs_path.write_text(json.dumps({
            "schema_version": "p1_formal_calibration_pairs_v1",
            "calibration_id": "p1-null-20260807",
            "pairs": pairs,
        }))

    def make_export(self, index):
        export = self.root / f"run-{index:02d}"
        export.mkdir()
        run_id = f"run-{index:02d}"
        manifest_path = export / "test_planner_manifest.json"
        provenance = {
            "schema_version": "p1_evidence_provenance_v4",
            "run_id": run_id,
            "git_commit": "head-abc",
            "baseline_commit": "baseline-123",
            "git_worktree_clean": True,
            "process_start_epoch_s": 1000.0 + 100.0 * index,
            "process_start_stamp_utc": "2026-08-07T00:00:00Z",
            "process_end_epoch_s": 1090.0 + 100.0 * index,
            "process_end_stamp_utc": "2026-08-07T00:01:30Z",
            "validator_summary_complete": True,
            "runtime_paths": {
                "launch": {"path": "/install/test_planner.launch.py", "sha256": "launch-sha"},
                "planner_executable": {"path": "/install/ego_planner", "sha256": "planner-sha"},
                "bspline_library": {"path": "/install/libbspline_opt.a", "sha256": "bspline-sha"},
            },
        }
        manifest = {
            "artifact_provenance": provenance,
            "export_dir": str(export),
            "experiment": "p1_degraded_lidar_good",
            "scenario": "p1_fork_fused_v1",
            "scenario_fingerprint": "sha256:p1-fork-test",
            "scenario_contract": {
                "fixture": "p1_fork_fused_v1", "seed": 41021,
                "decision_checkpoint": {
                    "truth_source_topic": "/sim/drone_0/truth_odom",
                    "profile_sample_zero_binding": "planner_truth_odom_state_at_planning_start",
                },
            },
            "planner_safety_profile": "p1",
            "run_duration_s": 90.0,
            "validation_duration_s": 90.0,
            "planner_start_delay_s": 10.0,
            "manager/max_vel": 1.0,
            "optimization/max_vel": 1.0,
            "bspline/limit_vel": 1.0,
            "fsm.thresh_replan_time": 0.9,
            "manager/planning_horizon": 10.5,
            "manager/p1_collision_fanout_clearance_m": 2.5,
            "grid_map/local_update_range_x": 11.0,
            "record_bag": False,
            "run_validator": True,
            "p0.enable_risk_grid": True,
            "p0.resolution_m": 0.75,
            "p0.horizons_s": [0.0, 0.5, 1.0, 1.5, 2.0, 2.5],
            "p1.use_integrity_cost": True,
            "p1.max_candidates_per_attempt": 8,
            "p1.metrics_only": True,
            "p1.lambda_integrity": 0.00001,
            "p1.objective_aggregation_mode": "fixed_200_smooth_cvar",
            "p1.smooth_cvar_alpha": 0.90,
            "p1.smooth_max_temperature": 0.01,
            "p1.normalization_budget_fraction": 0.30,
            "p1.accepted_profile_path": str(export / "planner_p1_accepted_trajectory_risk_profile.csv"),
            "p1.accepted_profile_context_path": str(export / "planner_p1_accepted_trajectory_risk_profile_context.csv"),
            "p0.occupancy_query_evidence_path": str(export / "planner_p0_occupancy_query_evidence.csv"),
        }
        manifest_path.write_text(json.dumps(manifest))
        pair_number = index // 2
        null_offset = (0.001 * (pair_number + 1)) if index % 2 else 0.0
        common = {
            "schema_version": "p1_evidence_provenance_v4",
            "run_id": run_id,
            "manifest_path": str(manifest_path),
        }
        profiles = []
        corners = []
        for sample_index in range(200):
            fraction = sample_index / 199.0
            c_pi = 1.0 + 0.2 * fraction + null_offset
            profiles.append({
                **common,
                "profile_seq": 1,
                "trajectory_id": 9,
                "planning_attempt_id": 11,
                "candidate_id": 7,
                "snapshot_generation_id": 4,
                "query_base_time_s": 9.0,
                "sample_index": sample_index,
                "x": -9.5 + 10.0 * fraction,
                "y": 0.0,
                "z": 1.0,
                "t_s": 2.0 * fraction,
                "hit": 1,
                "valid": 1,
                "stale": 0,
                "c_pi": c_pi,
                "metrics_only": 1,
                "applied_to_objective": 0,
                "lambda_integrity": 0.00001,
            })
            corners.append({
                **common,
                "planning_attempt_id": 11,
                "candidate_id": 7,
                "snapshot_generation_id": 4,
                "query_base_time_s": 9.0,
                "phase": "accepted",
                "sample_index": sample_index,
                "temporal_weight": 1.0,
                "corner_weight": 1.0,
                "c_pi": c_pi,
            })
        write_csv(export / "planner_p1_accepted_trajectory_risk_profile.csv", profiles)
        write_csv(export / "planner_p0_occupancy_query_evidence.csv", corners)
        context = [{
            **common,
            "profile_seq": 1,
            "trajectory_id": 9,
            "planning_attempt_id": 11,
            "candidate_id": 7,
            "snapshot_generation_id": 4,
            "query_base_time_s": 9.0,
            "planning_start_s": 8.9,
            "expected_sample_count": 200,
            "matched_sample_count": 200,
            "match_ratio": 1.0,
            "fresh": 1,
            "coverage_ok": 1,
            "spatial_in_bounds": 1,
            "temporal_in_horizon": 1,
            "frame_match": 1,
            "generation_match": 1,
            "query_time_match": 1,
        }]
        write_csv(export / "planner_p1_accepted_trajectory_risk_profile_context.csv", context)
        (export / "test_planner_validation_summary.json").write_text(json.dumps({
            **common, "passed": True,
        }))
        return export

    def calibrate(self):
        return calibration.calibrate(
            self.pairs_path,
            self.root / "output",
            generated_at_epoch_s=4000.0,
            generated_at_utc="2026-08-07T00:33:20Z",
        )

    def test_freezes_max_null_scores_plus_two_run_deterministic_budget(self):
        result = self.calibrate()
        self.assertEqual(result["schema_version"], "p1_formal_tolerance_calibration_v1")
        self.assertEqual(result["calibration_id"], "p1-null-20260807")
        self.assertEqual(len(result["run_ids"]), 20)
        self.assertEqual(result["conformal"]["pair_count"], 10)
        self.assertEqual(result["conformal"]["coverage"], 0.90)
        self.assertAlmostEqual(result["thresholds"]["tau_mean"], 0.010)
        self.assertAlmostEqual(result["thresholds"]["tau_cvar"], 0.010)
        self.assertAlmostEqual(result["thresholds"]["tau_max"], 0.010)
        output = self.root / "output"
        for name in (
            "p1_formal_tolerance_calibration_v1.json",
            "p1_formal_calibration_pairs.csv",
            "p1_formal_calibration_validity.csv",
            "p1_formal_null_effect_distribution.png",
            "p1_formal_error_budget.png",
        ):
            path = output / name
            self.assertTrue(path.is_file(), name)
            self.assertGreater(path.stat().st_size, 0, name)

    def test_rejects_wrong_pair_count_duplicate_run_and_invalid_run_contracts(self):
        payload = json.loads(self.pairs_path.read_text())
        payload["pairs"] = payload["pairs"][:9]
        self.pairs_path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(calibration.CalibrationError, "exactly 10"):
            self.calibrate()

        self.setUp()
        payload = json.loads(self.pairs_path.read_text())
        payload["pairs"][0]["run_b_export"] = payload["pairs"][0]["run_a_export"]
        self.pairs_path.write_text(json.dumps(payload))
        with self.assertRaisesRegex(calibration.CalibrationError, "duplicate run ID"):
            self.calibrate()

    def test_rejects_config_hash_cleanliness_and_support_mismatches(self):
        manifest_path = self.exports[-1] / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["artifact_provenance"]["git_worktree_clean"] = False
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(calibration.CalibrationError, "clean git"):
            self.calibrate()

        manifest["artifact_provenance"]["git_worktree_clean"] = True
        manifest["artifact_provenance"]["runtime_paths"]["planner_executable"]["sha256"] = "other"
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(calibration.CalibrationError, "runtime identity"):
            self.calibrate()

        manifest["artifact_provenance"]["runtime_paths"]["planner_executable"]["sha256"] = "planner-sha"
        manifest["p0.resolution_m"] = 1.0
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(calibration.CalibrationError, "configuration identity"):
            self.calibrate()

        manifest["p0.resolution_m"] = 0.75
        manifest_path.write_text(json.dumps(manifest))
        profile_path = self.exports[-1] / "planner_p1_accepted_trajectory_risk_profile.csv"
        with profile_path.open() as handle:
            rows = list(csv.DictReader(handle))
        rows[-1]["valid"] = "0"
        write_csv(profile_path, rows)
        with self.assertRaisesRegex(calibration.CalibrationError, "complete matched support"):
            self.calibrate()

    def test_rejects_overlapping_nonserial_runs(self):
        manifest_path = self.exports[1] / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["artifact_provenance"]["process_start_epoch_s"] = 1050.0
        manifest["artifact_provenance"]["process_end_epoch_s"] = 1140.0
        manifest_path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(calibration.CalibrationError, "must be serial"):
            self.calibrate()


if __name__ == "__main__":
    unittest.main()
