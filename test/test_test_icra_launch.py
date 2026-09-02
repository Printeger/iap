import importlib.util
import sys
import unittest
from pathlib import Path


REPO = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO / "scripts" / "dev_planner"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_p4_g0c_tests import require_hermetic_test_environment  # noqa: E402


require_hermetic_test_environment()

MODULE_PATH = REPO / "launch" / "test_icra.launch.py"
SPEC = importlib.util.spec_from_file_location("test_icra_launch", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class TestIcraLaunchTest(unittest.TestCase):
    def test_interactive_defaults_allow_map_and_odometry_warmup(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["planner_start_delay_s"], "3.0")

    def test_empty_logging_arguments_resolve_inside_generated_runtime(self):
        runtime = Path("/tmp/iap_sim_demo11_test_planner_123")
        runtime_base, log_root = MODULE._resolve_runtime_logging_roots(runtime)
        self.assertEqual(runtime_base, runtime)
        self.assertEqual(log_root, str(runtime / "iap_logs"))

    def test_explicit_runtime_base_and_log_root_are_preserved(self):
        runtime = Path("/tmp/icra/runtime/iap_sim_demo11_test_planner_123")
        runtime_base, log_root = MODULE._resolve_runtime_logging_roots(
            runtime,
            "/tmp/icra/runtime",
            "/tmp/icra/runtime/logs",
        )
        self.assertEqual(runtime_base, Path("/tmp/icra/runtime"))
        self.assertEqual(log_root, "/tmp/icra/runtime/logs")

    def test_simulation_acceleration_units_are_explicit(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["odometry_acc_scale"], "1.0")
        self.assertEqual(defaults["planner_executor_thread_count"], "4")

    def test_icra_vertical_slice_uses_deterministic_initialization(self):
        profile = MODULE.EXPERIMENT_PRESETS["icra_p0_p4_v2_p5_dev"]
        self.assertEqual(profile["odometry_initialization_mode"], "NAIVE")
        self.assertEqual(profile["lidar_start_delay_s"], "2.0")
        self.assertEqual(profile["planner_start_delay_s"], "10.0")
        for key in (
            "planner_enable_p1",
            "planner_enable_p2",
            "planner_enable_p3_local",
            "planner_enable_p3_global",
        ):
            self.assertEqual(profile[key], "false")
        self.assertEqual(profile["safety_viz.enable_p4_viz"], "true")
        self.assertEqual(
            profile["grid_map/independent_cloud_min_interval_s"], "0.5")
        self.assertEqual(
            profile["grid_map/independent_cloud_clock_guard_s"], "0.5")
        self.assertEqual(profile["p0.refresh_start_delay_s"], "0.05")
        self.assertEqual(profile["p0.online_mapping_mode"], "true")
        self.assertEqual(profile["p0.fit_grid_to_map_cloud"], "false")
        self.assertEqual(profile["p0.map_topic"], "")
        self.assertEqual(profile["planner_executor_thread_count"], "6")
        scenario = MODULE.SCENARIO_PRESETS["icra072_p4_selection_trigger_v1"]
        self.assertEqual(scenario["lidar_sensing_rate_hz"], "10.0")
        self.assertEqual(
            {key: profile[key] for key in MODULE.ICRA_DEV_FIXED_VALUES},
            MODULE.ICRA_DEV_FIXED_VALUES,
        )

    def test_icra_rviz_config_is_selected_without_changing_shared_config(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(
            defaults["rviz_config"], "config/sim_demo11/test_icra.rviz"
        )
        rviz = (REPO / defaults["rviz_config"]).read_text()
        self.assertIn("Name: P4 AStar Guides", rviz)
        self.assertIn("Value: /iap/rviz/p4_astar_guides", rviz)
        self.assertIn("Name: P4 Topology Channels", rviz)
        self.assertIn("Value: /iap/rviz/p4_topology_channels", rviz)

    def test_all_icra_runs_default_to_regular_spherical_first_hit_lidar(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(
            defaults["lidar_renderer_mode"], "spherical_first_hit_v1")
        self.assertEqual(defaults["lidar_horizontal_samples"], "512")
        self.assertEqual(defaults["lidar_vertical_samples"], "40")
        self.assertEqual(defaults["lidar_horizontal_fov_deg"], "360.0")
        self.assertEqual(defaults["lidar_vertical_min_deg"], "-7.0")
        self.assertEqual(defaults["lidar_vertical_max_deg"], "52.0")
        self.assertEqual(defaults["lidar_min_range_m"], "0.1")
        self.assertEqual(defaults["lidar_max_range_m"], "10.0")
        self.assertEqual(
            defaults["lidar_world_voxel_resolution_m"], "0.1")


if __name__ == "__main__":
    unittest.main()
