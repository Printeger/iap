import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "dev_planner" / "analyze_p1_prequalification.py"
SPEC = importlib.util.spec_from_file_location("analyze_p1_prequalification", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def run(lane, mean, cvar, maximum, length=10.0):
    return {
        "passed": True,
        "selected_lane": lane,
        "mean": mean,
        "cvar": cvar,
        "max": maximum,
        "path_length_m": length,
        "localization_error_m": 0.1,
        "hard_gates": {"passed": True},
    }


class P1PrequalificationTest(unittest.TestCase):
    def test_primary_and_mirror_direction_and_effectiveness(self):
        reference = run("upper", 1.0, 1.1, 1.2)
        primary = MODULE.evaluate_pair("primary", reference, run("lower", .98, 1.08, 1.19))
        mirror = MODULE.evaluate_pair("mirror", reference, run("upper", .98, 1.08, 1.19))
        self.assertTrue(primary["passed"])
        self.assertTrue(mirror["passed"])

    def test_null_uses_prefrozen_limits_and_path_growth(self):
        reference = run("lower", 1.0, 1.0, 1.0, 10.0)
        accepted = MODULE.evaluate_pair(
            "null", reference,
            run("upper", 1.005, 1.004, 1.002, 10.5),
        )
        rejected = MODULE.evaluate_pair(
            "null", reference,
            run("upper", 1.006, 1.004, 1.002, 10.5),
        )
        self.assertTrue(accepted["passed"])
        self.assertFalse(rejected["passed"])

    def test_soft_risk_requires_lower_route_and_nonregressing_max(self):
        reference = run("upper", 1.0, 1.0, 1.0)
        self.assertTrue(MODULE.evaluate_pair(
            "soft_risk", reference, run("lower", .99, .99, 1.0)
        )["passed"])
        self.assertFalse(MODULE.evaluate_pair(
            "soft_risk", reference, run("upper", .99, .99, 1.01)
        )["passed"])


if __name__ == "__main__":
    unittest.main()
