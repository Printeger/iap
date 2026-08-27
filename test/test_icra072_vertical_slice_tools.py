import csv
import contextlib
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
ANALYZER = REPO / "scripts/dev_planner/analyze_icra072_vertical_slice.py"
RUNNER = REPO / "scripts/dev_planner/run_icra072_vertical_slice.py"
INDEXER = REPO / "scripts/dev_planner/index_icra072_layer1_iterations.py"
TASK_RESULTS_ROOT = REPO / "results/icra27/icra072"
BUILD_ENTRY = REPO / "scripts/dev_planner/build_iap_dev.sh"
DEV_RUNS_ROOT = REPO / "results/icra27/dev_runs/layer1"
SHARED_INSTALL = Path("/home/dev/ws_iap/install")


class Icra072VerticalSliceToolsTest(unittest.TestCase):
    _run_counter = 0

    @classmethod
    @contextlib.contextmanager
    def _fresh_run_root(cls):
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        cls._run_counter += 1
        root = DEV_RUNS_ROOT / f"run-{os.getpid()}{cls._run_counter:03d}"
        shutil.rmtree(root, ignore_errors=True)
        root.mkdir()
        try:
            yield root
        finally:
            shutil.rmtree(root, ignore_errors=True)

    @staticmethod
    def _load_runner():
        spec = importlib.util.spec_from_file_location("icra072_layer1_runner", RUNNER)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    @staticmethod
    def _load_analyzer():
        spec = importlib.util.spec_from_file_location(
            "icra072_layer1_analyzer", ANALYZER)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        return module

    @staticmethod
    def _accepted_source_binding():
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=REPO, check=True,
            capture_output=True, text=True).stdout.strip()
        return {
            "schema_version": "icra072_source_binding_v2",
            "repository": str(REPO),
            "head_commit": commit,
            "origin_dev_icra_commit": commit,
            "status_porcelain": (
                "?? docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf\n"),
            "tracked_status": "",
            "tracked_worktree_clean": True,
            "untracked_allowlist": [{
                "path": "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf",
                "sha256": (
                    "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6"),
            }],
            "observed_untracked_paths": [
                "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"],
            "observed_protected_pdf_sha256": (
                "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6"),
            "rejected_untracked_paths": [],
            "rejected_tracked_entries": [],
            "head_matches_origin": True,
            "accepted": True,
            "failure_reasons": [],
        }

    def test_source_binding_allows_only_exact_protected_pdf(self):
        module = self._load_runner()
        commit = "a" * 40

        def capture(status):
            responses = iter((
                types.SimpleNamespace(returncode=0, stdout=commit + "\n",
                                      stderr=""),
                types.SimpleNamespace(returncode=0, stdout=commit + "\n",
                                      stderr=""),
                types.SimpleNamespace(returncode=0, stdout=status,
                                      stderr=""),
            ))
            with mock.patch.object(module.subprocess, "run",
                                   side_effect=lambda *args, **kwargs:
                                   next(responses)):
                return module._capture_source_binding()

        protected = "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"
        accepted = capture(f"?? {protected}\n")
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["observed_untracked_paths"], [protected])
        self.assertEqual(accepted["untracked_allowlist"], [{
            "path": protected,
            "sha256": hashlib.sha256(
                (REPO / protected).read_bytes()).hexdigest(),
        }])
        self.assertEqual(accepted["rejected_untracked_paths"], [])
        self.assertEqual(accepted["rejected_tracked_entries"], [])
        self.assertEqual(
            accepted["observed_protected_pdf_sha256"],
            accepted["untracked_allowlist"][0]["sha256"])

        with mock.patch.object(module.hashlib, "sha256") as sha256:
            sha256.return_value.hexdigest.return_value = "0" * 64
            wrong_hash = capture(f"?? {protected}\n")
        self.assertFalse(wrong_hash["accepted"])
        self.assertEqual(wrong_hash["observed_protected_pdf_sha256"],
                         "0" * 64)
        self.assertEqual(wrong_hash["rejected_untracked_paths"], [protected])
        self.assertIn("protected_pdf_missing_or_hash_mismatch",
                      wrong_hash["failure_reasons"])

        for status, rejected_path in (
                (f"?? {protected}\n?? rogue.py\n", "rogue.py"),
                (f"?? {protected}\n?? config/rogue.yaml\n",
                 "config/rogue.yaml")):
            with self.subTest(status=status):
                rejected = capture(status)
                self.assertFalse(rejected["accepted"])
                self.assertEqual(rejected["rejected_untracked_paths"],
                                 [rejected_path])
                self.assertIn("untracked_path_not_allowlisted",
                              rejected["failure_reasons"])

        for entry in (" M scripts/dev_planner/example.py",
                      "M  scripts/dev_planner/example.py",
                      "R  old.py -> new.py", " D deleted.py"):
            with self.subTest(entry=entry):
                rejected = capture(f"?? {protected}\n{entry}\n")
                self.assertFalse(rejected["accepted"])
                self.assertEqual(rejected["rejected_tracked_entries"],
                                 [entry])
                self.assertIn("tracked_worktree_dirty",
                              rejected["failure_reasons"])

    def test_shared_dev_build_entry_has_exact_packages_and_workspace_roots(self):
        source = BUILD_ENTRY.read_text()
        self.assertIn("source /opt/ros/jazzy/setup.bash", source)
        self.assertLess(source.index("source /opt/ros/jazzy/setup.bash"),
                        source.index("set -u"))
        self.assertIn("--symlink-install", source)
        self.assertIn("-DBUILD_TESTING=ON", source)
        self.assertIn("--build-base /home/dev/ws_iap/build", source)
        self.assertIn("--install-base /home/dev/ws_iap/install", source)
        self.assertIn("--log-base /home/dev/ws_iap/log", source)
        self.assertIn(
            "--packages-select iap plan_env traj_utils path_searching "
            "bspline_opt ego_planner", source)
        self.assertNotIn("results/icra27", source)
        self.assertNotIn("attempt_", source)

    def test_analyzer_accepts_one_complete_identity_ordered_end_to_end(self):
        with self._fresh_run_root() as root:
            export = root / "exports/run"
            export.mkdir(parents=True)
            p4_path = export / "planner_p4_risk_astar_debug.csv"
            source_binding = self._accepted_source_binding()
            (root / "source_binding.json").write_text(
                json.dumps(source_binding))
            (root / "run_manifest.json").write_text(json.dumps({
                "schema_version": "icra072_layer1_dev_run_v1",
                "run_id": root.name, "iterative_development": True,
                "commit": source_binding["head_commit"],
                "source_binding": source_binding,
                "source_binding_recheck": source_binding,
                "source_binding_final": source_binding,
                "gpu_ready": True, "launch_started": True,
                "launch_early_exit": False,
                "owned_process_groups_cleared": True,
                "result": "PASS",
                "process_result": {"required_processes_ok": True},
            }))
            launch = {
                "experiment": "icra_p0_p4_v2_p5_dev",
                "scenario": "icra072_p4_selection_trigger_v1",
                "planner_safety_profile": "icra_p0_p4_v2_p5_dev",
                "p0.enable_risk_grid": True,
                "p1.use_integrity_cost": False,
                "p2.enable_candidate_ranking": False,
                "p3.enable_local_reference_bias": False,
                "p3.enable_global_reference_bias": False,
                "p4.enable_risk_aware_astar": True,
                "p4.metrics_only": False,
                "p4.objective": "PROVIDER_BOTTLENECK_V2",
                "p4.debug_csv_path": str(p4_path),
                "p5.enable_runtime_gate": True,
                "p5.enable_final_gate": True,
            }
            (export / "test_planner_manifest.json").write_text(json.dumps(launch))
            with p4_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=[
                    "schema_version", "status", "selection_applied",
                    "planning_attempt_id", "collision_segment_id",
                    "request_hash", "snapshot_generation_id",
                    "snapshot_config_hash", "occupancy_epoch",
                    "original_hash", "risk_hash", "selected_hash",
                    "original_sample_count", "original_valid_count",
                    "original_unknown_count", "original_stale_count",
                    "original_non_finite_count", "risk_sample_count",
                    "risk_valid_count", "risk_unknown_count",
                    "risk_stale_count", "risk_non_finite_count", "reason"])
                writer.writeheader()
                writer.writerow({
                    "schema_version": "p4_collision_guide_decision_v2",
                    "status": "RISK_SELECTED", "selection_applied": "1",
                    "planning_attempt_id": "2", "collision_segment_id": "3",
                    "request_hash": "req", "occupancy_epoch": "4",
                    "snapshot_generation_id": "7",
                    "snapshot_config_hash": "cfg",
                    "original_hash": "original", "risk_hash": "risk",
                    "selected_hash": "guide",
                    "original_sample_count": "200",
                    "original_valid_count": "200",
                    "original_unknown_count": "0",
                    "original_stale_count": "0",
                    "original_non_finite_count": "0",
                    "risk_sample_count": "200", "risk_valid_count": "200",
                    "risk_unknown_count": "0", "risk_stale_count": "0",
                    "risk_non_finite_count": "0",
                    "reason": "provider_bottleneck_selected",
                })
            lineage_fields = [
                "schema_version", "stage", "stamp_s", "planning_attempt_id",
                "collision_segment_id", "request_hash",
                "snapshot_generation_id", "snapshot_config_hash",
                "occupancy_epoch", "original_guide_hash", "risk_guide_hash",
                "selected_guide_hash",
                "selection_applied", "control_points_hash",
                "closed_collision_observed",
                "no_collision_refinement_observed", "trajectory_id",
                "final_bspline_identity",
                "trajectory_start_s",
                "trajectory_start_ns",
            ]
            with Path(str(p4_path) + ".lineage.csv").open(
                    "w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=lineage_fields)
                writer.writeheader()
                common = {
                    "schema_version": "p4_v2_end_to_end_lineage_v2",
                    "planning_attempt_id": "2", "collision_segment_id": "3",
                    "request_hash": "req", "snapshot_generation_id": "7",
                    "snapshot_config_hash": "cfg", "occupancy_epoch": "4",
                    "original_guide_hash": "original",
                    "risk_guide_hash": "risk",
                    "selected_guide_hash": "guide", "selection_applied": "1",
                    "control_points_hash": "cp", "trajectory_id": "9",
                    "closed_collision_observed": "1",
                    "no_collision_refinement_observed": "1",
                    "final_bspline_identity": "bs",
                    "trajectory_start_s": "123.504278",
                    "trajectory_start_ns": "123504278000",
                }
                for stage, stamp in (
                    ("final_bspline_before_p5", 10.0),
                    ("p5_final_pass_before_publish", 10.1),
                    ("normal_publish_authorized", 10.2),
                ):
                    writer.writerow({**common, "stage": stage, "stamp_s": stamp})
            capture = [
                {"kind": "p0_health", "receive_steady_s": 1.0,
                 "payload": {"ready": True, "stale": False,
                             "generation_id": 7}},
                {"kind": "p5_status", "receive_steady_s": 2.0,
                 "payload": {"phase": "final", "action": "OK",
                             "final_candidate_rejected": False,
                             "final_candidate_traj_id": 9,
                             "final_candidate_start_time_ns": 123504278000,
                             "current_integrity_source": "FUSED"}},
                {"kind": "normal_bspline", "receive_steady_s": 3.0,
                 "payload": {"trajectory_id": 9,
                             "start_time_s": 123.504278,
                             "start_time_ns": 123504278000}},
                {"kind": "p5_status", "receive_steady_s": 4.0,
                 "payload": {"phase": "runtime", "action": "OK",
                             "raw_action": "OK", "reason": "ok",
                             "raw_reason": "ok", "active_reasons": [],
                             "current_reason": "", "future_reason": "",
                             "final_candidate_traj_id": -1,
                             "current_integrity_source": "FUSED",
                             "samples": [{"trajectory_sample_source":
                                          "runtime_committed",
                                          "trajectory_id": 9,
                                          "trajectory_start_time_s": 123.51,
                                          "trajectory_start_time_ns":
                                              123504278000}]}},
            ]
            (root / "lineage_capture.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in capture))
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root)],
                capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
            analysis = json.loads((root / "analysis.json").read_text())
            self.assertEqual(analysis["result"], "PASS")
            self.assertIsNone(analysis["first_missing_stage"])
            self.assertEqual(analysis["provider_support"], {
                "decision_count": 1,
                "original_complete_count": 1,
                "risk_complete_count": 1,
                "both_complete_count": 1,
                "selection_blockers": {},
            })
            self.assertEqual(analysis["exercised_commit"],
                             source_binding["head_commit"])
            mismatched_binding = {
                **source_binding,
                "origin_dev_icra_commit": "0" * 40,
                "head_matches_origin": False,
            }
            (root / "source_binding.json").write_text(
                json.dumps(mismatched_binding))
            source_output = root / "analysis_source_mismatch.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(source_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "source_binding_manifest_mismatch",
                json.loads(source_output.read_text())["failures"])
            (root / "source_binding.json").write_text(
                json.dumps(source_binding))
            capture_path = root / "lineage_capture.jsonl"
            captured_rows = [json.loads(line) for line in
                             capture_path.read_text().splitlines()]
            runtime_row = next(
                row for row in captured_rows
                if row.get("kind") == "p5_status"
                and row["payload"].get("phase") == "runtime")
            for action in ("REQUEST_REPLAN",
                           "REQUEST_EMERGENCY_STOP_CANDIDATE"):
                with self.subTest(runtime_action=action):
                    runtime_row["payload"]["action"] = action
                    capture_path.write_text(
                        "".join(json.dumps(row, sort_keys=True) + "\n"
                                for row in captured_rows))
                    rejected_runtime_output = (
                        root / f"analysis_runtime_{action.lower()}.json")
                    completed = subprocess.run(
                        [sys.executable, str(ANALYZER), "--run-root",
                         str(root), "--output", str(rejected_runtime_output)],
                        capture_output=True, text=True, check=False)
                    self.assertNotEqual(completed.returncode, 0)
                    rejected_runtime = json.loads(
                        rejected_runtime_output.read_text())
                    self.assertFalse(rejected_runtime["stage_status"][
                        "p5_runtime_committed"])
                    self.assertIn("p5_runtime_action_not_ok",
                                  rejected_runtime["failures"])
            runtime_row["payload"]["action"] = "OK"
            runtime_row["payload"]["raw_action"] = "REQUEST_REPLAN"
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in captured_rows))
            latent_output = root / "analysis_runtime_latent_replan.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(latent_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("p5_runtime_action_not_ok",
                          json.loads(latent_output.read_text())["failures"])
            runtime_row["payload"]["raw_action"] = "OK"
            later_emergency = json.loads(json.dumps(runtime_row))
            later_emergency["receive_steady_s"] = 4.5
            later_emergency["payload"]["action"] = (
                "REQUEST_EMERGENCY_STOP_CANDIDATE")
            later_emergency["payload"]["raw_action"] = (
                "REQUEST_EMERGENCY_STOP_CANDIDATE")
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in captured_rows + [later_emergency]))
            later_emergency_output = (
                root / "analysis_runtime_later_emergency.json")
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(later_emergency_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "p5_runtime_action_not_ok",
                json.loads(later_emergency_output.read_text())["failures"])
            mixed_identity_cases = {
                "missing": {"trajectory_sample_source": "runtime_committed"},
                "malformed": {
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": "not-an-id",
                    "trajectory_start_time_ns": 123504278000,
                },
                "sentinel": {
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": -1,
                    "trajectory_start_time_ns": -9223372036854775808,
                },
                "mismatched_id": {
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 10,
                    "trajectory_start_time_ns": 123504278000,
                },
                "mismatched_start": {
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 9,
                    "trajectory_start_time_ns": 123504278001,
                },
            }
            for case, additional_sample in mixed_identity_cases.items():
                with self.subTest(mixed_runtime_identity=case):
                    mixed_rows = json.loads(json.dumps(captured_rows))
                    mixed_runtime = next(
                        row for row in mixed_rows
                        if row.get("kind") == "p5_status"
                        and row["payload"].get("phase") == "runtime")
                    mixed_runtime["payload"]["samples"].append(
                        additional_sample)
                    capture_path.write_text(
                        "".join(json.dumps(row, sort_keys=True) + "\n"
                                for row in mixed_rows))
                    mixed_identity_output = (
                        root / f"analysis_runtime_mixed_{case}.json")
                    completed = subprocess.run(
                        [sys.executable, str(ANALYZER), "--run-root",
                         str(root), "--output", str(mixed_identity_output)],
                        capture_output=True, text=True, check=False)
                    self.assertNotEqual(completed.returncode, 0)
                    mixed_analysis = json.loads(
                        mixed_identity_output.read_text())
                    self.assertFalse(
                        mixed_analysis["stage_status"]
                        ["p5_runtime_committed"])
                    self.assertIn(
                        "p5_runtime_mixed_committed_identity",
                        mixed_analysis["failures"])
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in captured_rows))
            lineage_path = Path(str(p4_path) + ".lineage.csv")
            with lineage_path.open(newline="") as stream:
                original_lineage = list(csv.DictReader(stream))
            lineage_fieldnames = list(original_lineage[0])
            original_capture = json.loads(json.dumps(captured_rows))
            sentinel = -9223372036854775808
            for case in ("missing", "malformed", "sentinel", "mismatched",
                         "control_points", "final_identity"):
                with self.subTest(identity_case=case):
                    lineage_rows = json.loads(json.dumps(original_lineage))
                    capture_rows = json.loads(json.dumps(original_capture))
                    if case == "missing":
                        for row in lineage_rows:
                            row["trajectory_start_ns"] = ""
                        for row in capture_rows:
                            payload = row.get("payload", {})
                            payload.pop("final_candidate_start_time_ns", None)
                            payload.pop("start_time_ns", None)
                            for sample in payload.get("samples", []):
                                sample.pop("trajectory_start_time_ns", None)
                    elif case == "malformed":
                        for row in lineage_rows:
                            row["trajectory_start_ns"] = "not-a-nanosecond"
                    elif case == "sentinel":
                        for row in lineage_rows:
                            row["trajectory_start_ns"] = str(sentinel)
                        for row in capture_rows:
                            payload = row.get("payload", {})
                            if payload.get("phase") == "final":
                                payload["final_candidate_start_time_ns"] = sentinel
                            if row.get("kind") == "normal_bspline":
                                payload["start_time_ns"] = sentinel
                            for sample in payload.get("samples", []):
                                sample["trajectory_start_time_ns"] = sentinel
                    else:
                        if case == "mismatched":
                            lineage_rows[-1]["trajectory_start_ns"] = (
                                "123504278001")
                        elif case == "control_points":
                            for row in lineage_rows:
                                row["control_points_hash"] = ""
                        else:
                            for row in lineage_rows:
                                row["final_bspline_identity"] = ""
                    with lineage_path.open("w", newline="") as stream:
                        writer = csv.DictWriter(
                            stream, fieldnames=lineage_fieldnames)
                        writer.writeheader()
                        writer.writerows(lineage_rows)
                    capture_path.write_text(
                        "".join(json.dumps(row, sort_keys=True) + "\n"
                                for row in capture_rows))
                    identity_output = root / f"analysis_identity_{case}.json"
                    completed = subprocess.run(
                        [sys.executable, str(ANALYZER), "--run-root",
                         str(root), "--output", str(identity_output)],
                        capture_output=True, text=True, check=False)
                    self.assertNotEqual(completed.returncode, 0)
                    identity_analysis = json.loads(identity_output.read_text())
                    expected_failure = (
                        "lineage_trajectory_identity_inconsistent"
                        if case == "mismatched" else
                        "lineage_trajectory_identity_missing_or_invalid")
                    self.assertIn(expected_failure,
                                  identity_analysis["failures"])
                    self.assertEqual(identity_analysis["first_missing_stage"],
                                     "ego_final_bspline")
            with lineage_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=lineage_fieldnames)
                writer.writeheader()
                writer.writerows(original_lineage)
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in original_capture))
            float_capture = json.loads(json.dumps(original_capture))
            for row in float_capture:
                payload = row.get("payload", {})
                if payload.get("phase") == "final":
                    payload["final_candidate_start_time_ns"] = 123504278000.0
                if row.get("kind") == "normal_bspline":
                    payload["start_time_ns"] = 123504278000.0
                for sample in payload.get("samples", []):
                    sample["trajectory_start_time_ns"] = 123504278000.0
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in float_capture))
            float_output = root / "analysis_identity_float.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(float_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertEqual(
                json.loads(float_output.read_text())["first_missing_stage"],
                "p5_final_pass_before_publish")
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in original_capture))
            unselected = {
                **original_lineage[0],
                "planning_attempt_id": "99",
                "request_hash": "unselected-request",
                "selection_applied": "0",
                "trajectory_id": "17",
                "trajectory_start_ns": "123504279000",
                "final_bspline_identity": "unselected-bs",
            }
            with lineage_path.open("a", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=lineage_fieldnames)
                for stage, stamp in (
                        ("final_bspline_before_p5", 11.0),
                        ("p5_final_pass_before_publish", 11.1),
                        ("normal_publish_authorized", 11.2)):
                    writer.writerow({**unselected, "stage": stage,
                                     "stamp_s": stamp})
            later_capture = original_capture + [
                {"kind": "p5_status", "receive_steady_s": 5.0,
                 "payload": {"phase": "final", "action": "OK",
                             "final_candidate_rejected": False,
                             "final_candidate_traj_id": 17,
                             "final_candidate_start_time_ns": 123504279000,
                             "current_integrity_source": "FUSED"}},
                {"kind": "normal_bspline", "receive_steady_s": 6.0,
                 "payload": {"trajectory_id": 17,
                             "start_time_ns": 123504279000}},
                {"kind": "p5_status", "receive_steady_s": 7.0,
                 "payload": {"phase": "runtime", "action": "OK",
                             "current_integrity_source": "FUSED",
                             "samples": [{"trajectory_sample_source":
                                          "runtime_committed",
                                          "trajectory_id": 17,
                                          "trajectory_start_time_ns":
                                              123504279000}]}},
            ]
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in later_capture))
            selected_output = root / "analysis_selected_terminal.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(selected_output)],
                capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0)
            selected_analysis = json.loads(selected_output.read_text())
            self.assertEqual(selected_analysis["accepted_terminal_identity"], {
                "trajectory_id": 9,
                "trajectory_start_ns": 123504278000,
                "final_bspline_identity": "bs",
                "selection_applied": True,
            })
            with lineage_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=lineage_fieldnames)
                writer.writeheader()
                writer.writerows(original_lineage)
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in original_capture))
            launch["p5.current_pl_source"] = "LIDAR_CERTIFIED"
            (export / "test_planner_manifest.json").write_text(
                json.dumps(launch))
            authority_output = root / "analysis_p5_authority_bypass.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(authority_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            authority_analysis = json.loads(authority_output.read_text())
            self.assertEqual(authority_analysis["first_missing_stage"],
                             "p5_final_pass_before_publish")
            self.assertIn("p5_authoritative_fused_current_bypassed",
                          authority_analysis["failures"])
            launch.pop("p5.current_pl_source")
            (export / "test_planner_manifest.json").write_text(
                json.dumps(launch))
            manifest_path = root / "run_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["owned_process_groups_cleared"] = False
            manifest["result"] = "FAIL"
            manifest_path.write_text(json.dumps(manifest))
            cleanup_output = root / "analysis_cleanup_failed.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(cleanup_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            cleanup_analysis = json.loads(cleanup_output.read_text())
            self.assertIn("owned_process_group_cleanup_failed",
                          cleanup_analysis["failures"])
            self.assertEqual(cleanup_analysis["first_missing_stage"],
                             "runner_cleanup")
            manifest["owned_process_groups_cleared"] = True
            manifest["result"] = "PASS"
            manifest_path.write_text(json.dumps(manifest))
            capture_path.write_text(
                capture_path.read_text().replace(
                    '"trajectory_start_time_ns": 123504278000',
                    '"trajectory_start_time_ns": 123504278001'))
            exact_output = root / "analysis_runtime_exact_mismatch.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(exact_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "lineage_p5_runtime_trajectory_identity_mismatch",
                json.loads(exact_output.read_text())["failures"])
            capture_path.write_text(
                capture_path.read_text().replace(
                    '"trajectory_start_time_ns": 123504278001',
                    '"trajectory_start_time_ns": 123504278000'))
            captured_rows = [json.loads(line) for line in
                             capture_path.read_text().splitlines()]
            runtime_row = next(
                row for row in captured_rows
                if row.get("kind") == "p5_status"
                and row["payload"].get("phase") == "runtime")
            runtime_row["payload"]["samples"][0].pop("trajectory_id")
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in captured_rows))
            runtime_output = root / "analysis_runtime_mismatch.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(runtime_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            runtime_analysis = json.loads(runtime_output.read_text())
            self.assertEqual(runtime_analysis["first_missing_stage"],
                             "p5_runtime_committed")
            self.assertIn("lineage_p5_runtime_trajectory_identity_mismatch",
                          runtime_analysis["failures"])
            runtime_row["payload"]["samples"][0]["trajectory_id"] = 9
            capture_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n"
                        for row in captured_rows))
            lineage_path.write_text(
                lineage_path.read_text().replace(",1,1,9,bs", ",0,1,9,bs"))
            closed_output = root / "analysis_closed_missing.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(closed_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            closed_analysis = json.loads(closed_output.read_text())
            self.assertEqual(closed_analysis["first_missing_stage"],
                             "closed_collision")
            self.assertEqual(closed_analysis["stage_order"], [
                "p0_snapshot", "closed_collision", "p4_selection_application",
                "ego_final_bspline", "p5_final_pass_before_publish",
                "normal_publication", "p5_runtime_committed",
            ])
            self.assertIn("truthful_closed_collision_missing",
                          closed_analysis["failures"])
            lineage_path.write_text(
                lineage_path.read_text().replace(",0,1,9,bs", ",1,1,9,bs"))
            lineage_path.write_text(
                lineage_path.read_text().replace(",req,", ",mixed_request,"))
            mixed_output = root / "analysis_mixed.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(mixed_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "p4_ego_p5_publish_lineage_identity_mismatch",
                json.loads(mixed_output.read_text())["failures"])

            # A selected row is not admissible unless every typed support
            # counter is present and proves complete finite support.
            lineage_path.write_text(
                lineage_path.read_text().replace(",mixed_request,", ",req,"))
            with p4_path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            fields = [field for field in rows[0]
                      if field != "risk_unknown_count"]
            with p4_path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                writer.writerows({field: row[field] for field in fields}
                                 for row in rows)
            missing_support_output = root / "analysis_missing_support.json"
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root),
                 "--output", str(missing_support_output)],
                capture_output=True, text=True, check=False)
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "p4_v2_selected_guide_with_complete_support_missing",
                json.loads(missing_support_output.read_text())["failures"])
            self.assertEqual(
                json.loads(missing_support_output.read_text())["first_missing_stage"],
                "p4_selection_application")

    def test_tools_reject_repository_external_evidence_roots(self):
        outside = Path("/tmp/icra072_external_evidence_forbidden")
        completed = subprocess.run(
            [sys.executable, str(ANALYZER), "--run-root", str(outside)],
            capture_output=True, text=True, check=False)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must be under", completed.stderr)
        runner = (REPO / "scripts/dev_planner/run_icra072_vertical_slice.py").read_text()
        capture = (REPO / "scripts/dev_planner/capture_icra072_vertical_slice.py").read_text()
        self.assertIn("validate_cli_paths(", runner)
        self.assertIn("DEV_RUNS_ROOT", runner)
        self.assertIn("DEV_RUNS_ROOT", capture)
        self.assertIn("validate_capture_paths(", capture)

    def test_analyzer_types_missing_empty_and_non_file_p4_bindings(self):
        cases = (
            ("missing", None, "p4_debug_path_missing"),
            ("empty", "", "p4_debug_path_empty"),
            ("non_file", "exports/not_a_file.csv", "p4_debug_path_not_file"),
        )
        for name, binding, expected in cases:
            with self.subTest(name=name), self._fresh_run_root() as root:
                export = root / "exports/run"
                export.mkdir(parents=True)
                (root / "run_manifest.json").write_text(json.dumps({
                    "schema_version": "icra072_layer1_dev_run_v1",
                    "run_id": root.name, "iterative_development": True,
                }))
                launch = {}
                if binding is not None:
                    launch["p4.debug_csv_path"] = (
                        str(root / binding) if binding else binding)
                (export / "test_planner_manifest.json").write_text(
                    json.dumps(launch))
                (root / "lineage_capture.jsonl").write_text("")
                completed = subprocess.run(
                    [sys.executable, str(ANALYZER), "--run-root", str(root)],
                    capture_output=True, text=True, check=False)
                self.assertNotEqual(completed.returncode, 0)
                analysis = json.loads((root / "analysis.json").read_text())
                self.assertIn(expected, analysis["failures"])
                if name == "empty":
                    self.assertNotIn(
                        "p4_debug_path_outside_run_root", analysis["failures"])

    def test_iteration_index_classifies_retained_runs_without_rewriting_them(self):
        output = DEV_RUNS_ROOT / f"iteration_index_test_{os.getpid()}.json"
        self.assertFalse(output.exists())
        before = {
            run_id: (DEV_RUNS_ROOT / run_id / "run_manifest.json").read_bytes()
            for run_id in ("run-001", "run-019", "run-020")
        }
        try:
            completed = subprocess.run(
                [sys.executable, str(INDEXER), "--runs-root",
                 str(DEV_RUNS_ROOT), "--through-run", "20", "--output",
                 str(output)], capture_output=True, text=True, check=False)
            self.assertEqual(
                completed.returncode, 0, completed.stdout + completed.stderr)
            payload = json.loads(output.read_text())
            self.assertEqual(len(payload["iterations"]), 20)
            by_id = {row["run_id"]: row for row in payload["iterations"]}
            self.assertEqual(by_id["run-001"]["classification"],
                             "CAPTURE_NOT_READY")
            self.assertEqual(by_id["run-001"]["first_missing_stage"],
                             "p0_snapshot")
            self.assertEqual(by_id["run-011"]["classification"],
                             "FAILED_STAGE")
            self.assertEqual(by_id["run-011"]["first_missing_stage"],
                             "p4_selection_application")
            self.assertEqual(by_id["run-016"]["classification"],
                             "FAILED_STAGE")
            self.assertEqual(by_id["run-016"]["first_missing_stage"],
                             "ego_final_bspline")
            self.assertEqual(by_id["run-019"]["classification"],
                             "REJECTED_RUNNER_CLEANUP")
            self.assertEqual(by_id["run-019"]["first_missing_stage"],
                             "runner_cleanup")
            self.assertEqual(by_id["run-020"]["classification"],
                             "REJECTED_P5_AUTHORITY_BYPASS")
            self.assertEqual(by_id["run-020"]["first_missing_stage"],
                             "p5_final_pass_before_publish")
            repeated = subprocess.run(
                [sys.executable, str(INDEXER), "--runs-root",
                 str(DEV_RUNS_ROOT), "--through-run", "20", "--output",
                 str(output)], capture_output=True, text=True, check=False)
            self.assertNotEqual(repeated.returncode, 0)
            self.assertIn("already exists", repeated.stderr)
            for run_id, content in before.items():
                self.assertEqual(
                    (DEV_RUNS_ROOT / run_id / "run_manifest.json").read_bytes(),
                    content)
        finally:
            if output.exists():
                output.unlink()

    def test_iterative_runner_accepts_only_fresh_layer1_run_and_shared_install(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}"
        shutil.rmtree(run_root, ignore_errors=True)
        fake_gate = types.SimpleNamespace(
            run_gpu_preflight=lambda _root: {"gpu_ready": False})
        popen = mock.Mock(side_effect=AssertionError("ROS must not start"))
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      return_value=self._accepted_source_binding()), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=fake_gate), \
                    mock.patch.object(module, "_start_process", popen):
                self.assertEqual(module.main(), 4)
            popen.assert_not_called()
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(manifest["schema_version"],
                             "icra072_layer1_dev_run_v1")
            self.assertEqual(manifest["run_id"], run_root.name)
            self.assertEqual(manifest["cwd"], str(REPO))
            self.assertEqual(manifest["install_root"], str(SHARED_INSTALL))
            self.assertEqual(manifest["argv"], argv)
            self.assertTrue(manifest["commit"])
            self.assertEqual(manifest["stage_observations"], {})
            self.assertFalse(manifest["launch_started"])
            analysis = json.loads((run_root / "analysis.json").read_text())
            self.assertEqual(analysis["result"], "FAIL")
            self.assertEqual(analysis["first_missing_stage"], "p0_snapshot")
            outcome = json.loads(
                (run_root / "orchestration_outcome.json").read_text())
            self.assertEqual(outcome["runner_result"], "GPU_NOT_READY")
            self.assertEqual(outcome["analyzer_result"], "FAIL")
            self.assertEqual(outcome["first_missing_stage"], "p0_snapshot")
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

        invalid = (
            (REPO / "results/icra27/icra072/run-999", SHARED_INSTALL,
             "run root must match"),
            (Path("/tmp/run-999"), SHARED_INSTALL, "run root must match"),
            (DEV_RUNS_ROOT / "run-999", REPO / "results", "install root must be"),
        )
        for root, install, reason in invalid:
            with self.subTest(root=root, install=install), self.assertRaisesRegex(
                    SystemExit, reason):
                module.validate_cli_paths(root, install)

        existing = DEV_RUNS_ROOT / "run-999999"
        existing.mkdir(parents=True, exist_ok=True)
        try:
            with self.assertRaisesRegex(SystemExit, "run root already exists"):
                module.validate_cli_paths(existing, SHARED_INSTALL)
        finally:
            existing.rmdir()

    def test_runner_finalizes_capture_spawn_exception(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}001"
        shutil.rmtree(run_root, ignore_errors=True)
        fake_gate = types.SimpleNamespace(
            run_gpu_preflight=lambda _root: {"gpu_ready": True})
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      return_value=self._accepted_source_binding()), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=fake_gate), \
                    mock.patch.object(module, "_start_process",
                                      side_effect=OSError("spawn denied")):
                self.assertEqual(module.main(), 8)
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(manifest["result"], "RUNNER_EXCEPTION")
            self.assertEqual(manifest["runner_exception"]["type"], "OSError")
            analysis = json.loads((run_root / "analysis.json").read_text())
            self.assertEqual(analysis["first_missing_stage"], "p0_snapshot")
            outcome = json.loads(
                (run_root / "orchestration_outcome.json").read_text())
            self.assertEqual(outcome["runner_exit_code"], 8)
            self.assertEqual(outcome["result"], "FAIL")
            self.assertIn("runner_exception:OSError", outcome["failures"])
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

    def test_runner_finalizes_gate_import_exception_before_gpu_or_ros(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}002"
        shutil.rmtree(run_root, ignore_errors=True)
        popen = mock.Mock(side_effect=AssertionError("ROS must not start"))
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      return_value=self._accepted_source_binding()), \
                    mock.patch.object(module, "_load_gate_runner",
                                      side_effect=ImportError("gate missing")), \
                    mock.patch.object(module, "_start_process", popen):
                self.assertEqual(module.main(), 8)
            popen.assert_not_called()
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(manifest["result"], "RUNNER_EXCEPTION")
            self.assertEqual(manifest["runner_exception"]["type"],
                             "ImportError")
            self.assertFalse(manifest["gpu_ready"])
            invocation = json.loads(
                (run_root / "analyzer_invocation.json").read_text())
            self.assertIsInstance(invocation["exit_code"], int)
            outcome = json.loads(
                (run_root / "orchestration_outcome.json").read_text())
            self.assertEqual(outcome["runner_exit_code"], 8)
            self.assertEqual(outcome["result"], "FAIL")
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

    def test_runner_rejects_unbound_source_before_gpu_or_ros(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}003"
        shutil.rmtree(run_root, ignore_errors=True)
        gate = mock.Mock()
        popen = mock.Mock(side_effect=AssertionError("ROS must not start"))
        rejected_binding = {
            **self._accepted_source_binding(),
            "tracked_status": " M scripts/dev_planner/example.py",
            "tracked_worktree_clean": False,
            "accepted": False,
            "failure_reasons": ["tracked_worktree_dirty"],
        }
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      return_value=rejected_binding), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=gate), \
                    mock.patch.object(module, "_start_process", popen):
                self.assertEqual(module.main(), 8)
            gate.run_gpu_preflight.assert_not_called()
            popen.assert_not_called()
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertFalse(manifest["source_binding"]["accepted"])
            binding = json.loads((run_root / "source_binding.json").read_text())
            self.assertEqual(binding["failure_reasons"],
                             ["tracked_worktree_dirty"])
            outcome = json.loads(
                (run_root / "orchestration_outcome.json").read_text())
            self.assertEqual(outcome["result"], "FAIL")
            self.assertIn("runner_exception:RuntimeError",
                          outcome["failures"])
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

    def test_runner_finalizes_preflight_exception_before_ros(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}004"
        shutil.rmtree(run_root, ignore_errors=True)
        gate = types.SimpleNamespace(
            run_gpu_preflight=mock.Mock(
                side_effect=OSError("preflight evidence unavailable")))
        popen = mock.Mock(side_effect=AssertionError("ROS must not start"))
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      return_value=self._accepted_source_binding()), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=gate), \
                    mock.patch.object(module, "_start_process", popen), \
                    mock.patch("builtins.print") as print_mock:
                self.assertEqual(module.main(), 8)
            print_mock.assert_any_call("GPU_NOT_READY")
            popen.assert_not_called()
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(manifest["runner_exception"]["type"], "OSError")
            self.assertFalse(manifest["gpu_ready"])
            self.assertTrue(
                (run_root / "orchestration_outcome.json").is_file())
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

    def test_runner_rechecks_source_binding_before_ros(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}005"
        shutil.rmtree(run_root, ignore_errors=True)
        accepted = self._accepted_source_binding()
        changed = {
            **accepted,
            "head_commit": "1" * 40,
            "head_matches_origin": False,
            "accepted": False,
            "failure_reasons": ["head_origin_mismatch"],
        }
        gate = types.SimpleNamespace(
            run_gpu_preflight=lambda _root: {"gpu_ready": True})
        popen = mock.Mock(side_effect=AssertionError("ROS must not start"))
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      side_effect=(accepted, changed)), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=gate), \
                    mock.patch.object(module, "_start_process", popen):
                self.assertEqual(module.main(), 8)
            popen.assert_not_called()
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(
                manifest["runner_exception"]["message"],
                "source_binding_changed_before_ros")
            self.assertFalse(manifest["source_binding_recheck"]["accepted"])
        finally:
            shutil.rmtree(run_root, ignore_errors=True)

    def test_runner_finalizes_source_change_after_owned_cleanup(self):
        module = self._load_runner()
        DEV_RUNS_ROOT.mkdir(parents=True, exist_ok=True)
        run_root = DEV_RUNS_ROOT / f"run-{os.getpid()}006"
        self.assertFalse(run_root.exists())
        accepted = self._accepted_source_binding()
        changed = {
            **accepted,
            "status_porcelain": (
                accepted["status_porcelain"] + "?? rogue_config.yaml\n"),
            "observed_untracked_paths": (
                accepted["observed_untracked_paths"]
                + ["rogue_config.yaml"]),
            "rejected_untracked_paths": ["rogue_config.yaml"],
            "accepted": False,
            "failure_reasons": ["untracked_path_not_allowlisted"],
        }

        class FakeProcess:
            def __init__(self, pid):
                self.pid = pid
                self.returncode = 0
                self._icra072_group_cleared = False

            def poll(self):
                return None

        capture = FakeProcess(72001)
        launch = FakeProcess(72002)
        stopped = []

        def stop_owned(process, _timeout_s=10.0):
            stopped.append(process)
            process._icra072_group_cleared = True
            return 0

        monitor = mock.Mock()
        monitor.finish.return_value = {"required_processes_ok": True}
        gate = types.SimpleNamespace(
            run_gpu_preflight=lambda _root: {"gpu_ready": True},
            RequiredProcessMonitor=mock.Mock(return_value=monitor))
        argv = [str(RUNNER), "--run-root", str(run_root.relative_to(REPO)),
                "--install-root", str(SHARED_INSTALL), "--duration-s", "30"]
        try:
            with mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "_capture_source_binding",
                                      side_effect=(accepted, accepted,
                                                   changed)), \
                    mock.patch.object(module, "_load_gate_runner",
                                      return_value=gate), \
                    mock.patch.object(module, "_start_process",
                                      side_effect=(capture, launch)), \
                    mock.patch.object(module, "_wait_ready", return_value={
                        "schema_version": "icra072_capture_readiness_v1",
                        "ready": True}), \
                    mock.patch.object(module, "_stop_owned",
                                      side_effect=stop_owned), \
                    mock.patch.object(module, "_owned_group_cleared",
                                      side_effect=lambda pid: pid in {
                                          capture.pid, launch.pid}), \
                    mock.patch.object(module.time, "monotonic",
                                      side_effect=(0.0, 31.0)):
                self.assertEqual(module.main(), 8)
            self.assertEqual(set(stopped), {capture, launch})
            self.assertTrue(capture._icra072_group_cleared)
            self.assertTrue(launch._icra072_group_cleared)
            manifest = json.loads((run_root / "run_manifest.json").read_text())
            self.assertEqual(manifest["runner_exception"]["message"],
                             "source_binding_changed_during_run")
            self.assertTrue(manifest["owned_process_groups_cleared"])
            self.assertEqual(manifest["cleanup_errors"], [])
            analysis = json.loads((run_root / "analysis.json").read_text())
            outcome = json.loads(
                (run_root / "orchestration_outcome.json").read_text())
            self.assertIn("source_binding_changed_during_run",
                          analysis["failures"])
            self.assertIn("source_binding_changed_during_run",
                          outcome["failures"])
            self.assertEqual(outcome["runner_exit_code"], 8)
            self.assertEqual(outcome["result"], "FAIL")
        finally:
            if run_root.is_dir():
                manifest = json.loads(
                    (run_root / "run_manifest.json").read_text())
                self.assertEqual(manifest.get("run_id"), run_root.name)
                shutil.rmtree(run_root)

    def test_runner_and_publish_path_are_fail_closed(self):
        runner = (REPO / "scripts/dev_planner/run_icra072_vertical_slice.py").read_text()
        self.assertIn("p4.debug_csv_path:=", runner)
        self.assertIn("start_new_session=True", runner)
        self.assertIn("owned_process_groups_cleared", runner)
        self.assertIn("shlex.quote(str(setup))", runner)
        self.assertIn("shlex.join([", runner)
        source = (REPO / "src/iap/planner/plan_manage/src/ego_replan_fsm.cpp").read_text()
        self.assertLess(source.index('"final_bspline_before_p5"'),
                        source.index('"p5_final_pass_before_publish"'))
        self.assertLess(source.index('"p5_final_pass_before_publish"'),
                        source.index('"normal_publish_authorized"'))
        self.assertLess(source.index('"normal_publish_authorized"'),
                        source.index("bspline_pub_->publish(bspline)"))

    def test_stop_owned_clears_an_actual_child_process_group(self):
        module = self._load_runner()
        process = subprocess.Popen(
            ["bash", "-lc",
             "trap 'exit 0' INT; "
             "bash -c 'trap \"\" INT TERM; sleep 60' & wait"],
            start_new_session=True)
        try:
            module._stop_owned(process, timeout_s=0.05)
            self.assertTrue(module._owned_group_cleared(process.pid))
            with mock.patch.object(module.os, "killpg") as killpg:
                module._stop_owned(process, timeout_s=0.05)
            killpg.assert_not_called()
        finally:
            if not module._owned_group_cleared(process.pid):
                os.killpg(process.pid, 9)

    def test_cleanup_callback_is_retained_until_every_group_is_cleared(self):
        module = self._load_runner()
        cleared = types.SimpleNamespace(_icra072_group_cleared=True)
        uncleared = types.SimpleNamespace(_icra072_group_cleared=False)
        with mock.patch.object(module.atexit, "unregister") as unregister:
            self.assertFalse(
                module._unregister_cleanup_if_cleared(cleared, uncleared))
            unregister.assert_not_called()
            uncleared._icra072_group_cleared = True
            self.assertTrue(
                module._unregister_cleanup_if_cleared(cleared, uncleared))
            unregister.assert_called_once_with(module._stop_owned)

    def test_repository_relative_roots_resolve_from_repository_not_cwd(self):
        module = self._load_runner()
        analyzer = self._load_analyzer()
        run = DEV_RUNS_ROOT / f"run-{os.getpid()}777"
        relative = run.relative_to(REPO)
        with contextlib.chdir("/tmp"):
            resolved, install = module.validate_cli_paths(
                relative, SHARED_INSTALL)
        self.assertEqual(resolved, run.resolve())
        self.assertEqual(install, SHARED_INSTALL.resolve())
        with contextlib.chdir("/tmp"):
            self.assertEqual(
                analyzer._task_local(relative, "run root"), run.resolve())


if __name__ == "__main__":
    unittest.main()
