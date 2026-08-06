import importlib.util
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


if __name__ == "__main__":
    unittest.main()
