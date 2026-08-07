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
            "p1.objective_aggregation_mode": "fixed_200_smooth_cvar",
            "p1.smooth_max_temperature": 0.01,
            "p1.smooth_cvar_alpha": 0.90,
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
                "baseline_commit": preflight.BASELINE_COMMIT,
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
            "aggregation_mode": "fixed_200_smooth_cvar",
            "aggregation_temperature": "0.01",
            "aggregation_tail_fraction": "0.90",
            "fixed_sample_count": "200", "peak_contribution": "0.02",
            "grad_integrity_dot_displacement": "-0.01",
            "normalization_mode": "base_improvement_budget_v1",
            "base_prepass_success": "1",
            "incumbent_available": "0",
            "incumbent_mean_c_pi": "0.6",
            "incumbent_max_c_pi": "0.7",
            "replacement_comparison_mode": "full_profile",
            "replacement_comparison_duration_s": "0",
            "replacement_candidate_mean_c_pi": "0.4",
            "replacement_candidate_max_c_pi": "0.5",
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
        def checkpoint_payload(candidate, stage, checkpoint):
            row = {
                **common, "planning_attempt_id": "1",
                "candidate_id": candidate, "snapshot_generation_id": "4",
                "query_base_time_s": "9.0", "stage": stage,
                "checkpoint": checkpoint, "restart_index": "0",
                "iteration": "0", "line_search_count": "0", "step": "0",
                "objective": "1.0", "base_objective": "0.8",
                "raw_p1_objective": "nan",
                "normalized_p1_objective": "0.0", "anchor_objective": "0.0",
                "x_norm": "nan", "gradient_norm": "nan",
                "directional_derivative": "nan", "solver_result": "0",
                "reason": "none",
            }
            if stage == "p1_stage":
                row.update({
                    "raw_p1_objective": "0.2", "x_norm": "1.0",
                    "gradient_norm": "0.5",
                })
            if checkpoint == "first_direction":
                row["directional_derivative"] = "-0.25"
            elif checkpoint == "first_accepted_step":
                row.update({
                    "iteration": "1", "line_search_count": "1",
                    "step": "0.1", "objective": "0.9",
                    "directional_derivative": "-0.01",
                })
            elif checkpoint == "terminal":
                row.update({"iteration": "2", "objective": "0.8",
                            "reason": "success"})
            return row

        checkpoint_rows = [
            checkpoint_payload(candidate, stage, checkpoint)
            for candidate in ("1", "2")
            for stage, checkpoint in (("base_prepass", "start"),
                                      ("base_prepass", "terminal"),
                                      ("p1_stage", "first_direction"),
                                      ("p1_stage", "first_accepted_step"),
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
            "planner_p1_accepted_trajectory_risk_profile.csv": [
                {**common, "profile_seq": "1", "sample_index": str(i),
                 "planning_attempt_id": "1", "candidate_id": "1",
                 "snapshot_generation_id": "4", "c_pi": "0.3",
                 "valid": "1", "stale": "0"}
                for i in range(200)
            ],
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

    def test_rejects_wrong_fixed_baseline_commit(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["artifact_provenance"]["baseline_commit"] = "ca82cb52"
        manifest_path.write_text(json.dumps(manifest))
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertIn(
            "manifest baseline commit does not match fixed P1 baseline",
            result["errors"])

    def test_rejects_missing_first_accepted_optimizer_checkpoint(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        path = export / "planner_p1_optimizer_checkpoint.csv"
        with path.open(newline="") as handle:
            rows = [row for row in csv.DictReader(handle)
                    if row["checkpoint"] != "first_accepted_step"]
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("lacks optimizer checkpoints" in error
                            for error in result["errors"]))

    def test_rejects_unrecorded_optimizer_restart(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        path = export / "planner_p1_optimizer_checkpoint.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            if (row["candidate_id"] == "1" and
                    row["stage"] == "p1_stage" and
                    row["checkpoint"] == "terminal"):
                row["restart_index"] = "1"
            else:
                row["restart_index"] = "0"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("lacks optimizer restart checkpoint" in error
                            for error in result["errors"]))

    def test_rejects_malformed_optimizer_checkpoint_index(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        path = export / "planner_p1_optimizer_checkpoint.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            row["restart_index"] = "not-an-int"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("malformed integer fields" in error
                            for error in result["errors"]))

    def test_rejects_incomplete_optimizer_checkpoint_header(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        path = export / "planner_p1_optimizer_checkpoint.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            del row["line_search_count"]
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("CSV header is missing required fields" in error
                            for error in result["errors"]))

    def test_rejects_empty_first_accepted_checkpoint_payload(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        path = export / "planner_p1_optimizer_checkpoint.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            if row["checkpoint"] == "first_accepted_step":
                row["objective"] = ""
                row["directional_derivative"] = ""
                row["step"] = ""
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(manifest_path),
                   "export_dir": str(export), "bag_path": str(bag)}
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any("lacks accepted-step payload" in error
                            for error in result["errors"]))

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

    def test_rejects_wrong_smooth_cvar_provenance(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(export / "test_planner_manifest.json"),
                   "export_dir": str(export), "bag_path": str(bag)}
        path = export / "planner_p1_candidate_optimization.csv"
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        rows[0]["aggregation_tail_fraction"] = "0.80"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        with mock.patch.object(preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertIn("candidate smooth-CVaR aggregation evidence is invalid",
                      result["errors"])

    def test_rejects_authoritative_profile_that_regresses_selected_seed(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        payload = {"schema_version": preflight.SCHEMA, "run_id": "run-1",
                   "manifest_path": str(export / "test_planner_manifest.json"),
                   "export_dir": str(export), "bag_path": str(bag)}
        profile = export / "planner_p1_accepted_trajectory_risk_profile.csv"
        with profile.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            row["c_pi"] = "0.55"
        with profile.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertIn(
            "authoritative accepted profile regresses selected P1 seed",
            result["errors"])

    def test_shared_forward_window_closes_incumbent_replacement(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        payload = {
            "schema_version": preflight.SCHEMA,
            "run_id": "run-1",
            "manifest_path": str(manifest_path),
            "export_dir": str(export),
            "bag_path": str(bag),
        }
        candidate_path = export / "planner_p1_candidate_optimization.csv"
        with candidate_path.open(newline="") as handle:
            candidates = list(csv.DictReader(handle))
        winner = next(row for row in candidates if row["selected"] == "1")
        winner.update({
            "incumbent_available": "1",
            # The accepted full candidate profile is 0.3 and would regress
            # this shorter incumbent if the unequal domains were compared.
            "incumbent_mean_c_pi": "0.25",
            "incumbent_max_c_pi": "0.35",
            "replacement_comparison_mode": "shared_forward_time_window",
            "replacement_comparison_duration_s": "0.4",
            "replacement_candidate_mean_c_pi": "0.20",
            "replacement_candidate_max_c_pi": "0.30",
        })
        with candidate_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(candidates[0]))
            writer.writeheader()
            writer.writerows(candidates)

        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)

        self.assertTrue(result["passed"], result["errors"])

    def test_metrics_only_candidates_do_not_require_replacement_closure(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["p1.metrics_only"] = True
        manifest_path.write_text(json.dumps(manifest))
        common = {
            "schema_version": preflight.SCHEMA,
            "run_id": "run-1",
            "manifest_path": str(manifest_path),
        }
        payload = {
            **common,
            "export_dir": str(export),
            "bag_path": str(bag),
        }
        debug_path = export / "planner_p1_integrity_cost_debug.csv"
        with debug_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted({
                **common, "applied_to_objective": "0"}))
            writer.writeheader()
            writer.writerow({**common, "applied_to_objective": "0"})
        candidate_path = export / "planner_p1_candidate_optimization.csv"
        with candidate_path.open(newline="") as handle:
            candidates = list(csv.DictReader(handle))
        for candidate in candidates:
            candidate["replacement_accepted"] = "0"
        with candidate_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted(candidates[0]))
            writer.writeheader()
            writer.writerows(candidates)

        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=True, lambda_value=0.00001)

        self.assertTrue(result["passed"], result["errors"])

    def test_metrics_only_without_optimizer_attempt_does_not_require_attempt_sidecars(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["p1.metrics_only"] = True
        manifest_path.write_text(json.dumps(manifest))
        common = {
            "schema_version": preflight.SCHEMA,
            "run_id": "run-1",
            "manifest_path": str(manifest_path),
        }
        debug_path = export / "planner_p1_integrity_cost_debug.csv"
        with debug_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=sorted({
                **common, "applied_to_objective": "0"}))
            writer.writeheader()
            writer.writerow({**common, "applied_to_objective": "0"})
        for filename in preflight.OPTIMIZER_ATTEMPT_CSVS:
            (export / filename).unlink()
        payload = {
            **common,
            "export_dir": str(export),
            "bag_path": str(bag),
        }

        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=True, lambda_value=0.00001)

        self.assertTrue(result["passed"], result["errors"])

    def test_metrics_only_partial_optimizer_attempt_sidecars_fail_closed(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest["p1.metrics_only"] = True
        manifest_path.write_text(json.dumps(manifest))
        for filename in preflight.OPTIMIZER_ATTEMPT_CSVS:
            if filename != "planner_p0_occupancy_query_evidence.csv":
                (export / filename).unlink()
        payload = {
            "schema_version": preflight.SCHEMA,
            "run_id": "run-1",
            "manifest_path": str(manifest_path),
            "export_dir": str(export),
            "bag_path": str(bag),
        }

        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=True, lambda_value=0.00001)

        self.assertFalse(result["passed"])
        self.assertTrue(any(
            "planner_p1_candidate_optimization.csv" in error
            for error in result["errors"]), result["errors"])

    def test_formal_preflight_requires_bound_prelaunch_calibration(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        provenance = manifest["artifact_provenance"]
        manifest.update({
            "scenario": "degraded_lidar_good",
            "p0.resolution_m": 0.75,
            "p0.horizons_s": [0.0, 0.5, 1.0],
            "p0.enable_risk_grid": True,
            "p1.use_integrity_cost": True,
            "p1.max_candidates_per_attempt": 8,
            "planner_safety_profile": "p1",
            "run_duration_s": 90.0,
            "validation_duration_s": 90.0,
            "record_bag": True,
            "run_validator": True,
        })
        runtime_hashes = {
            key: provenance["runtime_paths"][key]["sha256"]
            for key in ("launch", "planner_executable", "bspline_library")
        }
        calibration_path = export / "calibration.json"
        calibration_payload = {
            "schema_version": "p1_formal_tolerance_calibration_v1",
            "calibration_id": "cal-1",
            "generated_at_epoch_s": provenance["process_start_epoch_s"] - 1.0,
            "git_commit": provenance["git_commit"],
            "baseline_commit": provenance["baseline_commit"],
            "scenario": manifest["scenario"],
            "runtime_hashes": runtime_hashes,
            "p0": {"resolution_m": 0.75, "horizons_s": [0.0, 0.5, 1.0]},
            "smooth_cvar": {
                "mode": "fixed_200_smooth_cvar", "alpha": 0.90,
                "temperature": 0.01, "eta_bisection_iterations": 100,
            },
            "lambda_integrity": 0.00001,
            "configuration_identity": {
                "experiment": manifest["experiment"],
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
                "run_duration_s": 90.0,
                "validation_duration_s": 90.0,
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
                    "run_a_mean": 1.0, "run_b_mean": 0.99, "s_mean": 0.01,
                    "run_a_cvar": 1.0, "run_b_cvar": 0.98, "s_cvar": 0.02,
                    "run_a_max": 1.0, "run_b_max": 0.97, "s_max": 0.03,
                }
                for index in range(10)
            ],
            "null_effect_maxima": {
                "s_mean": 0.01, "s_cvar": 0.02, "s_max": 0.03,
            },
            "deterministic_error": {
                "epsilon_grid": 0.0, "epsilon_resample": 0.0,
                "epsilon_det": 0.0,
            },
            "thresholds": {"tau_mean": 0.01, "tau_cvar": 0.02, "tau_max": 0.03},
        }
        calibration_path.write_text(json.dumps(calibration_payload))
        manifest_path.write_text(json.dumps(manifest))
        payload = {
            "schema_version": preflight.SCHEMA, "run_id": "run-1",
            "manifest_path": str(manifest_path), "export_dir": str(export),
            "bag_path": str(bag),
        }
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001,
                calibration_path=calibration_path)
        self.assertFalse(result["passed"])
        self.assertTrue(any("calibration" in error.lower() for error in result["errors"]))

        digest = __import__("hashlib").sha256(calibration_path.read_bytes()).hexdigest()
        manifest["p1.formal_calibration"] = {
            "calibration_id": "cal-1", "path": str(calibration_path),
            "sha256": digest,
        }
        manifest_path.write_text(json.dumps(manifest))
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001,
                calibration_path=calibration_path)
        self.assertTrue(result["passed"], result["errors"])

        calibration_payload["generated_at_epoch_s"] = provenance["process_start_epoch_s"] + 1.0
        calibration_path.write_text(json.dumps(calibration_payload))
        manifest["p1.formal_calibration"]["sha256"] = __import__("hashlib").sha256(
            calibration_path.read_bytes()).hexdigest()
        manifest_path.write_text(json.dumps(manifest))
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001,
                calibration_path=calibration_path)
        self.assertFalse(result["passed"])
        self.assertTrue(any("before formal run" in error for error in result["errors"]))

    def test_formal_p1_preflight_without_calibration_fails_closed(self):
        root, export, bag = self.make_bundle()
        self.addCleanup(root.cleanup)
        manifest_path = export / "test_planner_manifest.json"
        manifest = json.loads(manifest_path.read_text())
        manifest.update({"record_bag": True, "run_duration_s": 90.0})
        manifest_path.write_text(json.dumps(manifest))
        payload = {
            "schema_version": preflight.SCHEMA, "run_id": "run-1",
            "manifest_path": str(manifest_path), "export_dir": str(export),
            "bag_path": str(bag),
        }
        with mock.patch.object(
                preflight, "read_bag_provenance", return_value=([payload], "")):
            result = preflight.validate_bundle(
                export, bag, metrics_only=False, lambda_value=0.00001)
        self.assertFalse(result["passed"])
        self.assertTrue(any(
            "pre-frozen calibration" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
