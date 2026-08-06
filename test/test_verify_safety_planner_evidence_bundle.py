#!/usr/bin/env python3

import csv
import importlib.util
import json
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "dev_planner" / "verify_safety_planner_evidence_bundle.py"
SPEC = importlib.util.spec_from_file_location("p1_preflight", MODULE_PATH)
preflight = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(preflight)


class EvidenceBundlePreflightTest(unittest.TestCase):
    def make_bundle(self):
        root = tempfile.TemporaryDirectory()
        export = Path(root.name) / "export"
        bag = Path(root.name) / "bag"
        export.mkdir()
        bag.mkdir()
        manifest_path = export / "test_planner_manifest.json"
        started = time.time() - 1.0
        manifest = {
            "experiment": "p1_smoke",
            "scenario": "degraded_lidar_good",
            "export_dir": str(export),
            "planner_safety_profile": "p1",
            "p1.metrics_only": False,
            "p1.lambda_integrity": 0.00001,
            "p1.debug_csv_path": str(export / "planner_p1_integrity_cost_debug.csv"),
            "p1.candidate_optimization_path": str(export / "planner_p1_candidate_optimization.csv"),
            **{key: str(export / filename) for filename, key in preflight.CSV_KEYS.items()
               if filename in {
                   "planner_p1_candidate_control_points.csv",
                   "planner_p1_candidate_profile.csv",
                   "planner_p1_candidate_pairwise.csv",
                   "planner_p1_optimizer_checkpoint.csv",
                   "planner_p0_occupancy_query_evidence.csv",
               }},
            "p1.accepted_profile_path": str(export / "planner_p1_accepted_trajectory_risk_profile.csv"),
            "p1.accepted_profile_context_path": str(export / "planner_p1_accepted_trajectory_risk_profile_context.csv"),
            "p1.planning_context_timeline_path": str(export / "planner_p1_planning_context_timeline.csv"),
            "artifact_provenance": {
                "schema_version": preflight.SCHEMA, "run_id": "run-1", "git_commit": "abc",
                "git_worktree_clean": True, "bag_path": str(bag), "process_start_epoch_s": started,
                "process_start_stamp_utc": "2026-08-01T00:00:00Z", "process_end_stamp_utc": "2026-08-01T00:00:30Z",
                "validator_summary_complete": True, "bag_metadata_complete": True,
                "runtime_paths": {
                    key: {"path": str(MODULE_PATH), "sha256": __import__("hashlib").sha256(MODULE_PATH.read_bytes()).hexdigest()}
                    for key in ("launch", "planner_executable", "bspline_library")
                },
            },
        }
        manifest_path.write_text(json.dumps(manifest))
        common = {"schema_version": preflight.SCHEMA, "run_id": "run-1", "manifest_path": str(manifest_path)}
        candidate_base = {
            **common, "planning_attempt_id": "1", "snapshot_generation_id": "4",
            "query_base_time_s": "9.0", "optimization_success": "1",
            "replacement_accepted": "1", "pre_mean_c_pi": "0.5",
            "pre_max_c_pi": "0.6", "post_mean_c_pi": "0.4",
            "post_max_c_pi": "0.5", "support_full_valid": "1",
            "support_sample_count": "200", "pre_support_valid_count": "200",
            "post_support_valid_count": "200", "pre_support_coverage": "1",
            "post_support_coverage": "1", "rank_eligible": "1",
            "grad_integrity_dot_displacement": "-0.01",
            "normalization_mode": "base_improvement_budget_v1",
            "base_prepass_success": "1",
        }
        candidate_rows = [
            {**candidate_base, "candidate_id": "1", "selected": "1"},
            {**candidate_base, "candidate_id": "2", "selected": "0"},
        ]
        control_rows = [
            {**common, "planning_attempt_id": "1", "candidate_id": candidate,
             "phase": phase, "control_point_index": str(index), "x": str(index),
             "y": "0", "z": "1", "control_points_hash": f"{candidate}-{phase}"}
            for candidate in ("1", "2") for phase in ("initial", "final")
            for index in range(4)
        ]
        profile_rows = [
            {**common, "planning_attempt_id": "1", "candidate_id": candidate,
             "phase": phase, "sample_index": str(index), "valid": "1",
             "c_pi": str(0.5 - 0.01 * (candidate == "2")), "invalid_reason": "none"}
            for candidate in ("1", "2") for phase in ("initial", "final")
            for index in range(200)
        ]
        checkpoint_rows = [
            {**common, "planning_attempt_id": "1", "candidate_id": candidate,
             "stage": stage, "checkpoint": checkpoint}
            for candidate in ("1", "2")
            for stage, checkpoint in (("base_prepass", "start"),
                                      ("base_prepass", "terminal"),
                                      ("p1_stage", "first_direction"),
                                      ("p1_stage", "terminal"))
        ]
        payloads = {
            "planner_p1_integrity_cost_debug.csv": [{**common, "applied_to_objective": "1"}],
            "planner_p1_candidate_optimization.csv": candidate_rows,
            "planner_p1_candidate_control_points.csv": control_rows,
            "planner_p1_candidate_profile.csv": profile_rows,
            "planner_p1_candidate_pairwise.csv": [
                {**common, "planning_attempt_id": "1", "candidate_id_a": "1",
                 "candidate_id_b": "2", "phase": phase,
                 "control_point_distance": "0.1", "risk_profile_distance": "0.01",
                 "profile_valid": "1", "invalid_reason": "none"}
                for phase in ("initial", "final")
            ],
            "planner_p1_optimizer_checkpoint.csv": checkpoint_rows,
            "planner_p0_occupancy_query_evidence.csv": [
                {**common, "planning_attempt_id": "1", "candidate_id": "1",
                 "phase": "initial", "sample_index": "0", "corner_id": "0"}
            ],
            "planner_p1_accepted_trajectory_risk_profile.csv": [{**common, "sample_index": str(i)} for i in range(200)],
            "planner_p1_accepted_trajectory_risk_profile_context.csv": [{**common, "profile_seq": "1"}],
            "planner_p1_planning_context_timeline.csv": [{**common, "stage": "publish"}],
        }
        for name, rows in payloads.items():
            with (export / name).open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
                writer.writeheader(); writer.writerows(rows)
        (export / "test_planner_validation_summary.json").write_text(json.dumps({**common, "passed": True}))
        (bag / "metadata.yaml").write_text("\n".join(("/planning/evidence_provenance", "/planning/risk_grid_health", "/drone_0_planning/bspline", *preflight.P1_RVIZ_TOPICS)))
        return root, export, bag

    def test_accepts_bound_fresh_bundle_and_rejects_mixed_run(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1", "manifest_path": str(export / "test_planner_manifest.json"), "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertTrue(result["passed"], result["errors"])
        path = export / "planner_p1_candidate_optimization.csv"
        content = path.read_text().replace("run-1", "other-run", 1)
        path.write_text(content)
        with mock.patch.object(preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("run ID mismatch" in error for error in result["errors"]))

    def test_rejects_partial_candidate_lattice_support(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1", "manifest_path": str(export / "test_planner_manifest.json"), "export_dir": str(export), "bag_path": str(bag)}
        path = export / "planner_p1_candidate_optimization.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        rows[0]["pre_support_valid_count"] = "199"
        rows[0]["pre_support_coverage"] = "0.995"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        with mock.patch.object(preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertIn("candidate lacks full valid fixed-lattice support", result["errors"])


if __name__ == "__main__":
    unittest.main()
