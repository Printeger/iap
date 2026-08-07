import importlib.util
import hashlib
import json
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "launch" / "test_planner.launch.py"
SPEC = importlib.util.spec_from_file_location("test_planner_launch", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class TestPlannerLaunchTest(unittest.TestCase):
    def test_p1_fixed_lattice_keeps_replanning_to_planner_goal_boundary(self):
        self.assertEqual(
            MODULE._fixed_lattice_no_replan_threshold({"p1": True}), 0.2
        )
        self.assertEqual(
            MODULE._fixed_lattice_no_replan_threshold({"p1": False}), 1.0
        )

    def test_formal_calibration_is_manifest_only_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            path.write_text(json.dumps({
                "schema_version": "p1_formal_tolerance_calibration_v1",
                "calibration_id": "cal-1",
                "generated_at_epoch_s": 10.0,
            }))
            evidence = MODULE._formal_calibration_provenance(str(path))
            self.assertEqual(evidence["calibration_id"], "cal-1")
            self.assertEqual(evidence["path"], str(path.resolve()))
            self.assertEqual(
                evidence["sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
            )
            with self.assertRaisesRegex(RuntimeError, "calibration"):
                MODULE._formal_calibration_provenance(str(path.with_name("missing.json")))


if __name__ == "__main__":
    unittest.main()
