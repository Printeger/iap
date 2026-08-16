import importlib.util
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "dev_planner"
    / "run_gate0_qualification.py"
)
SPEC = importlib.util.spec_from_file_location("run_gate0_qualification", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Gate0RunnerTest(unittest.TestCase):
    def test_gate0a_matrix_is_fixed_to_three_scenarios_three_repeats(self):
        matrix = MODULE.gate0a_matrix()
        self.assertEqual(len(matrix), 9)
        self.assertEqual({row[0] for row in matrix}, set(MODULE.SCENARIOS))
        for label in MODULE.SCENARIOS:
            self.assertEqual(
                [row[2] for row in matrix if row[0] == label], [1, 2, 3]
            )

    def test_gate0a_effective_config_is_isolated_and_manifest_complete(self):
        config = MODULE.gate0a_effective_config(
            "mirror-r1", "p1_fork_fused_mirror_v1", Path("/tmp/gate0")
        )
        self.assertEqual(config["forest_random_seed"], 11)
        self.assertEqual(config["gnss_random_seed"], 20260011)
        self.assertEqual(config["terminal_wall_feature_seed"], 11022)
        self.assertFalse(config["manager/p1_collision_fanout_mirror_y"])
        for field in (
            "planner_enable_p1",
            "planner_enable_p2",
            "planner_enable_p3_local",
            "planner_enable_p3_global",
            "planner_enable_p4",
            "planner_enable_p5_runtime",
            "planner_enable_p5_final",
            "p1.use_integrity_cost",
            "p2.enable_candidate_ranking",
            "p3.enable_local_reference_bias",
            "p3.enable_global_reference_bias",
            "p4.enable_risk_aware_astar",
        ):
            self.assertIs(config[field], False, field)
        self.assertTrue(config["gate0.qualification_evidence_enable"])
        self.assertFalse(config["record_bag"])
        self.assertTrue(config["run_validator"])

    def test_p0_config_has_exact_query_shape(self):
        config = MODULE.p0_effective_config(Path("/tmp/p0"))
        self.assertEqual(config["p0.size_x_m"], 30.0)
        self.assertEqual(config["p0.size_y_m"], 30.0)
        self.assertEqual(config["p0.size_z_m"], 6.0)
        self.assertEqual(config["p0.resolution_m"], 0.75)
        self.assertEqual(config["p0.horizons_s"].split(","), [
            "0.0", "0.5", "1.0", "1.5", "2.0", "2.5"
        ])
        self.assertEqual(config["p0.predictor.worker_count"], 1)


if __name__ == "__main__":
    unittest.main()
