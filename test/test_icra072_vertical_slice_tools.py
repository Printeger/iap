import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
ANALYZER = REPO / "scripts/dev_planner/analyze_icra072_vertical_slice.py"
TASK_RESULTS_ROOT = REPO / "results/icra27/icra072"


class Icra072VerticalSliceToolsTest(unittest.TestCase):
    def test_analyzer_accepts_one_complete_identity_ordered_end_to_end(self):
        with tempfile.TemporaryDirectory(dir=TASK_RESULTS_ROOT) as directory:
            root = Path(directory)
            export = root / "exports/run"
            export.mkdir(parents=True)
            p4_path = export / "planner_p4_risk_astar_debug.csv"
            (root / "run_manifest.json").write_text(json.dumps({
                "run_id": "icra072-dev-smoke-001", "registered": True,
                "gpu_ready": True, "launch_started": True,
                "launch_early_exit": False,
                "process_result": {"required_processes_ok": True},
            }))
            launch = {
                "experiment": "icra_p0_p4_v2_p5_dev",
                "scenario": "icra_p0_p4_v2_p5_dev_fixture_v1",
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
                    "original_hash", "risk_hash", "selected_hash"])
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
                })
            lineage_fields = [
                "stage", "stamp_s", "planning_attempt_id",
                "collision_segment_id", "request_hash",
                "snapshot_generation_id", "snapshot_config_hash",
                "occupancy_epoch", "original_guide_hash", "risk_guide_hash",
                "selected_guide_hash",
                "selection_applied", "control_points_hash", "trajectory_id",
                "final_bspline_identity",
            ]
            with Path(str(p4_path) + ".lineage.csv").open(
                    "w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=lineage_fields)
                writer.writeheader()
                common = {
                    "planning_attempt_id": "2", "collision_segment_id": "3",
                    "request_hash": "req", "snapshot_generation_id": "7",
                    "snapshot_config_hash": "cfg", "occupancy_epoch": "4",
                    "original_guide_hash": "original",
                    "risk_guide_hash": "risk",
                    "selected_guide_hash": "guide", "selection_applied": "1",
                    "control_points_hash": "cp", "trajectory_id": "9",
                    "final_bspline_identity": "bs",
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
                             "final_candidate_traj_id": 9}},
                {"kind": "normal_bspline", "receive_steady_s": 3.0,
                 "payload": {"trajectory_id": 9}},
                {"kind": "p5_status", "receive_steady_s": 4.0,
                 "payload": {"phase": "runtime", "action": "OK",
                             "final_candidate_traj_id": 9,
                             "samples": [{"trajectory_sample_source":
                                          "runtime_committed"}]}},
            ]
            (root / "lineage_capture.jsonl").write_text(
                "".join(json.dumps(row) + "\n" for row in capture))
            completed = subprocess.run(
                [sys.executable, str(ANALYZER), "--run-root", str(root)],
                capture_output=True, text=True, check=False)
            self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
            self.assertEqual(json.loads((root / "analysis.json").read_text())["result"],
                             "PASS")
            lineage_path = Path(str(p4_path) + ".lineage.csv")
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

    def test_tools_reject_repository_external_evidence_roots(self):
        outside = Path("/tmp/icra072_external_evidence_forbidden")
        completed = subprocess.run(
            [sys.executable, str(ANALYZER), "--run-root", str(outside)],
            capture_output=True, text=True, check=False)
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("must be under", completed.stderr)
        runner = (REPO / "scripts/dev_planner/run_icra072_vertical_slice.py").read_text()
        capture = (REPO / "scripts/dev_planner/capture_icra072_vertical_slice.py").read_text()
        self.assertIn("_task_local(args.run_root", runner)
        self.assertIn("_task_local(args.output", capture)
        self.assertIn("_task_local(args.ready_file", capture)

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


if __name__ == "__main__":
    unittest.main()
