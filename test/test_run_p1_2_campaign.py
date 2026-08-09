import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "dev_planner" / "run_p1_2_campaign.py"
SPEC = importlib.util.spec_from_file_location("run_p1_2_campaign", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class P12CampaignTest(unittest.TestCase):
    def test_plan_has_prescribed_serial_runs_and_single_formal_analysis(self):
        plan = MODULE.build_plan(Path("/campaign"))
        prequalification = [item for item in plan if item["phase"] == "prequalification"]
        calibration = [item for item in plan if item["phase"] == "calibration"]
        formal = [item for item in plan if item["phase"] == "formal_run"]
        analyzer = [item for item in plan if item["phase"] == "formal_analyzer"]
        self.assertEqual(len(prequalification), 10)
        self.assertEqual(len(calibration), 20)
        self.assertEqual(len(formal), 2)
        self.assertEqual(len(analyzer), 1)
        self.assertEqual(
            [(item["scenario"], item["metrics_only"]) for item in prequalification],
            [
                ("p1_fork_fused_v1", True), ("p1_fork_fused_v1", False),
                ("p1_fork_fused_v1", True), ("p1_fork_fused_v1", False),
                ("p1_fork_fused_mirror_v1", True), ("p1_fork_fused_mirror_v1", False),
                ("p1_fork_symmetric_null_v1", True), ("p1_fork_symmetric_null_v1", False),
                ("p1_soft_risk_island_v1", True), ("p1_soft_risk_island_v1", False),
            ],
        )

    def test_state_refuses_resume_on_different_sha_and_preserves_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "campaign.json"
            state = MODULE.new_state("aaa", MODULE.build_plan(Path(tmp)))
            state["steps"][0].update({"status": "failed", "exit_code": 2})
            MODULE.save_state(path, state)
            loaded = MODULE.load_state(path, "aaa")
            self.assertEqual(loaded["steps"][0]["status"], "failed")
            with self.assertRaisesRegex(MODULE.CampaignError, "code SHA"):
                MODULE.load_state(path, "bbb")

    def test_dry_run_writes_commands_without_executing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = MODULE.run_campaign(root, git_sha="abc", dry_run=True)
            self.assertEqual(result["status"], "dry_run")
            self.assertTrue((root / "campaign.json").is_file())
            self.assertTrue(all(step.get("command") for step in result["steps"]))
            self.assertTrue(all(step["status"] == "planned" for step in result["steps"]))

    def test_nonzero_launch_still_indexes_finalized_run_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            export = root / "runs/failed/exports/export"
            export.mkdir(parents=True)
            (export / "test_planner_manifest.json").write_text(json.dumps({
                "artifact_provenance": {"run_id": "failed-run", "bag_path": ""}
            }))
            (export / "test_planner_validation_summary.json").write_text(json.dumps({
                "passed": False, "errors": ["intentional failure"]
            }))
            step = {"id": "failed", "phase": "prequalification"}
            MODULE._capture_launch_evidence(step, root)
            self.assertEqual(step["run_id"], "failed-run")
            self.assertEqual(step["export_dir"], str(export.resolve()))
            self.assertFalse(step["gate_result"]["validator_passed"])


if __name__ == "__main__":
    unittest.main()
