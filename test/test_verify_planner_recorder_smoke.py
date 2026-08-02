#!/usr/bin/env python3
"""Regression tests for the standalone recorder finalization preflight."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "dev_planner" / "verify_planner_recorder_smoke.py"
SPEC = importlib.util.spec_from_file_location("recorder_smoke", MODULE_PATH)
recorder_smoke = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(recorder_smoke)


class RecorderSmokeTest(unittest.TestCase):
    def make_bundle(self):
        root = tempfile.TemporaryDirectory()
        export, bag = Path(root.name) / "export", Path(root.name) / "bag"
        export.mkdir(); bag.mkdir()
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
        manifest = {"artifact_provenance": {
            "bag_path": str(bag), "process_end_stamp_utc": "2026-08-02T00:00:01Z",
            "bag_metadata_complete": True, "recorder_completed": True,
            "recorder_exit_code": 0, "recorder_command": "ros2 bag record ..."}}
        (export / "test_planner_manifest.json").write_text(json.dumps(manifest))
        return root, export, bag

    def test_accepts_finalized_bundle(self):
        root, export, bag = self.make_bundle(); self.addCleanup(root.cleanup)
        self.assertTrue(recorder_smoke.validate(export, bag)["passed"])

    def test_rejects_malformed_manifest_and_nonzero_recorder(self):
        root, export, bag = self.make_bundle(); self.addCleanup(root.cleanup)
        manifest = export / "test_planner_manifest.json"
        manifest.write_text("not-json")
        result = recorder_smoke.validate(export, bag)
        self.assertFalse(result["passed"])
        self.assertTrue(any("invalid manifest" in error for error in result["errors"]))
        manifest.write_text(json.dumps({"artifact_provenance": {"bag_path": str(bag),
            "process_end_stamp_utc": "done", "bag_metadata_complete": True,
            "recorder_completed": False, "recorder_exit_code": 2,
            "recorder_command": "ros2 bag record ..."}}))
        result = recorder_smoke.validate(export, bag)
        self.assertFalse(result["passed"])
        self.assertTrue(any("did not complete" in error for error in result["errors"]))

    def test_rejects_bag_mismatch(self):
        root, export, bag = self.make_bundle(); self.addCleanup(root.cleanup)
        manifest = export / "test_planner_manifest.json"
        data = json.loads(manifest.read_text())
        data["artifact_provenance"]["bag_path"] = str(export / "wrong-bag")
        manifest.write_text(json.dumps(data))
        result = recorder_smoke.validate(export, bag)
        self.assertFalse(result["passed"])
        self.assertTrue(any("does not match" in error for error in result["errors"]))


if __name__ == "__main__":
    unittest.main()
