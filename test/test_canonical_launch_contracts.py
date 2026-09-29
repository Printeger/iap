import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from launch import LaunchContext


REPO = Path(__file__).resolve().parents[1]
LAUNCH = REPO / "launch"


class CanonicalLaunchContractsTest(unittest.TestCase):
    @staticmethod
    def _load_launch(filename: str):
        path = LAUNCH / filename
        spec = importlib.util.spec_from_file_location(
            filename.replace(".", "_"), path
        )
        module = importlib.util.module_from_spec(spec)
        if spec.loader is None:
            raise RuntimeError(f"cannot load {path}")
        spec.loader.exec_module(module)
        return module

    def test_all_historical_scenario_names_are_in_catalog(self):
        catalog = json.loads(
            (REPO / "config/scenarios/catalog.json").read_text(encoding="utf-8")
        )
        expected = {
            "manual",
            "gnss_open_sky",
            "lidar_feature_rich",
            "lidar_corridor_degenerate",
            "fallback_only",
            "fused_nominal",
            "gnss_degraded_lidar_good",
            "icra_dense_forest_four_fork_v1",
            "icra_dense_forest_four_fork_v2",
            "icra_p0_p5_fused_degraded_corridor_v1",
            "p1_fork_fused_v1",
            "p1_fork_fused_mirror_v1",
            "p1_fork_symmetric_null_v1",
            "p1_soft_risk_island_v1",
            "p4_g0c_free_corridor_v1",
            "icra_p0_p4_v2_p5_dev_fixture_v1",
            "icra072_p4_selection_trigger_v1",
            "icra072_p4_selection_trigger_mirror_v1",
        }
        self.assertEqual(set(catalog), expected)

    def test_canonical_scenario_resources_are_package_relative_and_present(self):
        source = (LAUNCH / "_includes/full_stack_runtime.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("results/icra27/icra070/install_v2", source)
        self.assertIn(
            '"gnss_scenario_file": "config/gnss_sim/demo7_skymask_nlos.yaml"',
            source,
        )
        self.assertTrue((REPO / "config/gnss_sim/demo7_skymask_nlos.yaml").is_file())

    def test_module_launches_do_not_start_environments(self):
        forbidden = (
            "gnss_sim_node",
            "so3_quadrotor_simulator",
            "pcl_render_node",
            "random_forest",
            "ros2\", \"bag",
            "traj_server",
        )
        for filename in ("glio.launch.py", "glio_integrity.launch.py"):
            source = (LAUNCH / filename).read_text(encoding="utf-8")
            for token in forbidden:
                self.assertNotIn(token, source, f"{filename} contains {token}")

    def test_environment_starts_no_iap_algorithm_module(self):
        source = (
            LAUNCH / "_includes/simulation_environment.launch.py"
        ).read_text(encoding="utf-8")
        for token in (
            'executable="iap_rosnode"',
            'executable="ego_planner_node"',
            "phase2_planner_integrity_evaluator",
            "test_araim_validator",
        ):
            self.assertNotIn(token, source)

    def test_canonical_graphs_never_reference_phase2_evaluator(self):
        for filename in (
            "glio.launch.py",
            "glio_integrity.launch.py",
            "iap_sim.launch.py",
            "iap_flight.launch.py",
        ):
            source = (LAUNCH / filename).read_text(encoding="utf-8")
            self.assertNotIn(
                'executable="phase2_planner_integrity_evaluator"', source
            )

    def test_flight_graph_has_no_simulator_or_bag_process(self):
        source = (LAUNCH / "iap_flight.launch.py").read_text(encoding="utf-8")
        for token in (
            'package="gnss_sim"',
            'package="so3_quadrotor_simulator"',
            'package="map_generator"',
            'executable="poscmd_2_odom"',
            "ExecuteProcess",
        ):
            self.assertNotIn(token, source)
        self.assertIn('"forbid_sim_extensions": "true"', source)
        self.assertIn('"realworld_experiment": "true"', source)
        self.assertIn('"p0_online_mapping_mode": "true"', source)
        self.assertIn('"p0_fit_grid_to_map_cloud": "false"', source)
        self.assertIn('"p0_map_topic": ""', source)
        self.assertIn('"p4_require_risk_grid_ready_before_planning": "false"', source)
        for axis in "xyz":
            self.assertIn(f'"grid_map_origin_{axis}"', source)
            self.assertIn(f'"p0_origin_{axis}_m"', source)

    def test_profiles_select_exact_extension_sets(self):
        glio = json.loads(
            (REPO / "config/profiles/glio/config_ros.json").read_text(
                encoding="utf-8"
            )
        )
        integrity = json.loads(
            (REPO / "config/profiles/glio_integrity/config_ros.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            glio["glim_ros"]["extension_modules"], ["libgnss_extension.so"]
        )
        self.assertEqual(
            integrity["glim_ros"]["extension_modules"],
            ["libgnss_extension.so", "libintegrity_extension.so"],
        )
        flight = json.loads(
            (REPO / "config/profiles/full_stack_flight/config_ros.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            flight["glim_ros"]["extension_modules"],
            [
                "libgnss_extension.so",
                "libintegrity_extension.so",
                "libplanner_local_map_extension.so",
            ],
        )

    def test_runtime_profile_redirects_outputs(self):
        helper_path = LAUNCH / "_includes/profile_runtime.py"
        spec = importlib.util.spec_from_file_location("profile_runtime_test", helper_path)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "run"
            runtime, manifest = module.materialize_profile(
                source_config_dir=str(REPO / "config/profiles/glio_integrity"),
                output_dir=str(output),
                contract="glio_integrity",
                integrity_profile="fused",
                forbid_sim_extensions=True,
            )
            root = json.loads((Path(runtime) / "config.json").read_text())
            ros = json.loads((Path(runtime) / "config_ros.json").read_text())
            gnss = json.loads((Path(runtime) / "config_gnss.json").read_text())
            self.assertTrue(
                Path(root["global"]["timing_csv_path"]).is_relative_to(output)
            )
            self.assertTrue(
                Path(ros["glim_ros"]["dump_path"]).is_relative_to(output)
            )
            self.assertTrue(
                Path(gnss["gnss"]["debug_csv_path"]).is_relative_to(output)
            )
            self.assertEqual(manifest["contract"], "glio_integrity")
            for key, path in manifest["materialized_secondary_configs"].items():
                self.assertTrue(Path(path).is_relative_to(output), key)
                secondary = json.loads(Path(path).read_text(encoding="utf-8"))

                def assert_contained(value):
                    if isinstance(value, dict):
                        for child_key, child in value.items():
                            if isinstance(child, str) and child_key.endswith("_csv_path"):
                                self.assertTrue(Path(child).is_relative_to(output))
                            else:
                                assert_contained(child)
                    elif isinstance(value, list):
                        for child in value:
                            assert_contained(child)

                assert_contained(secondary)

    def test_sim_profile_reuses_exact_scenarios_without_test_processes(self):
        canonical = (LAUNCH / "iap_sim.launch.py").read_text(encoding="utf-8")
        internal = (
            LAUNCH / "_includes/full_stack_simulation.launch.py"
        ).read_text(encoding="utf-8")
        runtime = (LAUNCH / "_includes/full_stack_runtime.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("full_stack_simulation.launch.py", canonical)
        self.assertIn('"experiment": "canonical_full_stack_sim"', internal)
        self.assertIn('"run_validator": "false"', internal)
        self.assertIn('"record_bag": "false"', internal)
        self.assertIn('"planner_local_map_enable": "true"', runtime)
        self.assertIn('"sim_time_enable": "false"', runtime)
        self.assertIn(
            '"p4.require_risk_grid_ready_before_planning": "false"', runtime
        )
        self.assertIn(
            '{"sim_time/enable": _param_bool(context, "sim_time_enable")}', runtime
        )
        self.assertNotIn(' / "bp" / ', runtime)
        self.assertNotIn("icra_p0_p5_qualification.py", runtime)

    def test_catalog_task_mode_is_applied_to_every_canonical_scenario(self):
        catalog = json.loads(
            (REPO / "config/scenarios/catalog.json").read_text(encoding="utf-8")
        )
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        for scenario, entry in catalog.items():
            context = LaunchContext()
            for name, default in runtime.ARG_DEFAULTS:
                context.launch_configurations[name] = str(default)
            context.launch_configurations.update(
                {
                    "experiment": "canonical_full_stack_sim",
                    "scenario": scenario,
                }
            )
            runtime._apply_presets(context, str(REPO))
            self.assertEqual(
                context.launch_configurations["p4.assurance.task_mode"],
                entry["task_mode"],
                scenario,
            )

    def test_hidden_legacy_cli_cannot_override_canonical_safety(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        context = LaunchContext()
        for name, default in runtime.ARG_DEFAULTS:
            context.launch_configurations[name] = str(default)
        context.launch_configurations.update(
            {
                "experiment": "canonical_full_stack_sim",
                "scenario": "fused_nominal",
                "planner_enable_p5_runtime": "false",
                "p4.require_risk_grid_ready_before_planning": "true",
            }
        )
        with mock.patch.object(
            sys,
            "argv",
            [
                "iap_sim.launch.py",
                "planner_enable_p5_runtime:=false",
                "p4.require_risk_grid_ready_before_planning:=true",
            ],
        ):
            runtime._apply_presets(context, str(REPO))
        self.assertEqual(context.launch_configurations["planner_enable_p5_runtime"], "true")
        self.assertEqual(
            context.launch_configurations[
                "p4.require_risk_grid_ready_before_planning"
            ],
            "false",
        )

    def test_flight_requires_retained_calibration_manifest(self):
        helper_path = LAUNCH / "iap_flight.launch.py"
        spec = importlib.util.spec_from_file_location("iap_flight_test", helper_path)
        module = importlib.util.module_from_spec(spec)
        self.assertIsNotNone(spec.loader)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "calibration.json"
            path.write_text(
                json.dumps(
                    {
                        "schema_version": "iap_local_surface_calibration_v1",
                        "calibration_id": "vehicle_01_heldout_2026_09",
                        "local_surface_error_bound_m": 0.04,
                        "calibration_run_count": 3,
                        "held_out_run_count": 1,
                        "held_out_passed": True,
                    }
                ),
                encoding="utf-8",
            )
            retained, digest = module._verify_calibration_manifest(
                str(path), "vehicle_01_heldout_2026_09", 0.04
            )
            self.assertEqual(retained, path.resolve())
            self.assertEqual(len(digest), 64)

    def test_bp_is_installed_for_frozen_script_compatibility(self):
        cmake = (REPO / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertNotIn('PATTERN "bp" EXCLUDE', cmake)

    def test_each_canonical_entrypoint_constructs_a_graph(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)

            glio = self._load_launch("glio.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "config_path": str(REPO / "config/profiles/glio"),
                    "output_dir": str(root / "glio"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "use_sim_time": "false",
                }
            )
            self.assertEqual(len(glio._setup(context)), 2)

            integrity = self._load_launch("glio_integrity.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "config_path": str(REPO / "config/profiles/glio_integrity"),
                    "output_dir": str(root / "integrity"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "use_sim_time": "false",
                    "integrity_profile": "fused",
                    "forbid_sim_extensions": "true",
                    "runtime_contract": "glio_integrity",
                }
            )
            self.assertEqual(len(integrity._setup(context)), 2)

            simulation = self._load_launch("iap_sim.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "scenario": "fused_nominal",
                    "output_dir": str(root / "sim"),
                    "start_rviz": "false",
                    "planner_start_delay_s": "0",
                    "run_duration_s": "0",
                }
            )
            with mock.patch.object(
                simulation, "get_package_share_directory", return_value=str(REPO)
            ):
                self.assertEqual(len(simulation._setup(context)), 1)

            flight = self._load_launch("iap_flight.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "flight_authorized": "true",
                    "controller_handshake_confirmed": "true",
                    "output_dir": str(root / "flight"),
                    "config_path": str(REPO / "config/profiles/full_stack_flight"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "odometry_topic": "/odom",
                    "planner_cloud_topic": "/points",
                    "beam_evidence_topic": "/vehicle/lidar/beam_evidence",
                    "camera_pose_topic": "/camera_pose",
                    "depth_topic": "/depth",
                    "goal_x": "1",
                    "goal_y": "0",
                    "goal_z": "1",
                    "drone_id": "0",
                    "map_size_x": "42",
                    "map_size_y": "30",
                    "map_size_z": "8",
                    "max_velocity_mps": "1",
                    "max_acceleration_mps2": "1.5",
                    "planning_horizon_m": "8",
                    "planner_start_delay_s": "0",
                    "local_surface_error_bound_m": "0.04",
                    "local_surface_error_calibration_id": "test_vehicle_heldout_v1",
                    "local_surface_error_calibration_manifest": str(
                        REPO
                        / "test/fixtures/valid_local_surface_calibration.json"
                    ),
                }
            )

            def package_share(name):
                return str(REPO if name == "iap" else REPO / "src/iap/planner")

            with mock.patch.object(
                flight, "get_package_share_directory", side_effect=package_share
            ):
                self.assertEqual(len(flight._setup(context)), 3)

    def test_historical_launches_exist_only_in_backup(self):
        canonical = {
            "glio.launch.py",
            "glio_integrity.launch.py",
            "iap_sim.launch.py",
            "iap_flight.launch.py",
        }
        backup = {
            path.name for path in (LAUNCH / "bp").iterdir() if path.is_file()
        }
        self.assertTrue(backup - {"README.md"})
        for name in backup - {"README.md"}:
            self.assertFalse((LAUNCH / name).exists(), name)
            self.assertNotIn(name, canonical)
        for name in canonical:
            self.assertTrue((LAUNCH / name).is_file(), name)


if __name__ == "__main__":
    unittest.main()
