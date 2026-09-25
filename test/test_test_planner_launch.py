import importlib.util
import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO / "scripts" / "dev_planner"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from run_p4_g0c_tests import require_hermetic_test_environment  # noqa: E402

require_hermetic_test_environment()

from launch import LaunchContext  # noqa: E402


MODULE_PATH = REPO / "launch" / "test_planner.launch.py"
SPEC = importlib.util.spec_from_file_location("test_planner_launch", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class TestPlannerLaunchTest(unittest.TestCase):
    def test_guard_command_and_ack_topics_are_isolated_with_bspline(self):
        source = MODULE_PATH.read_text()
        self.assertGreaterEqual(source.count("planning/pending_guard_bspline"), 2)
        self.assertGreaterEqual(source.count("planning/pending_guard_status"), 2)
        self.assertIn(
            'bspline_topic.replace("/bspline", "/pending_guard_bspline")',
            source,
        )
        self.assertIn(
            'bspline_topic.replace("/bspline", "/pending_guard_status")',
            source,
        )

    @staticmethod
    def _runtime_logging_fixture(root: Path):
        runtime_base = root / "runtime"
        config_dir = runtime_base / "iap_sim_demo11_test_planner_fixture" / "sim_demo11"
        logging_path = config_dir.parent / "sim_ego" / "config_logging.json"
        config_dir.mkdir(parents=True)
        logging_path.parent.mkdir(parents=True)
        config_path = config_dir / "config.json"
        config_path.write_text(json.dumps({
            "global": {
                "config_logging": "../sim_ego/config_logging.json",
                "enable_timing_csv": True,
                "timing_csv_path": "/repo/log/profiling/iap_timing.csv",
            },
            "logging": {
                "log_dir": "/repo/log/",
                "save_logs": True,
                "rotate_logs": True,
                "max_file_size_kb": 8192,
                "max_files": 10,
            },
        }, indent=2) + "\n")
        logging_path.write_text(json.dumps({
            "logging": {
                "log_dir": "/repo/log/",
                "save_logs": True,
                "rotate_logs": True,
                "max_file_size_kb": 8192,
                "max_files": 10,
            }
        }, indent=2) + "\n")
        return runtime_base, config_path, logging_path

    def test_so3_feedback_uses_world_linear_acceleration_not_iap_specific_force(self):
        self.assertEqual(
            MODULE._so3_feedback_imu_topic(
                "/sim/drone_0/imu", "/sim/drone_0/imu_iap"
            ),
            "/sim/drone_0/imu",
        )

    def test_runtime_and_export_roots_are_optional_and_resolved_per_run(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["runtime_root_dir"], "")
        self.assertEqual(defaults["export_root_dir"], "")

    def test_non_icra_launch_keeps_legacy_lidar_renderer_opt_in(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["lidar_renderer_mode"],
                         "legacy_radius_crop_v1")
        source = MODULE_PATH.read_text()
        self.assertIn('"renderer_mode": LaunchConfiguration(', source)
        self.assertIn('"lidar.horizontal_samples": _param_int(', source)
        self.assertEqual(
            MODULE._lidar_output_semantics("legacy_radius_crop_v1"),
            "legacy_radius_crop_world_points",
        )
        self.assertEqual(
            MODULE._lidar_output_semantics("spherical_first_hit_v1"),
            "hit_only_first_return_pointcloud2",
        )
        runtime, export = MODULE._resolve_run_roots(
            "sim_demo11", "p1_fork_formal", "p1_fork_fused_v1", 1234,
            runtime_root_dir="/work/runtime", export_root_dir="/work/exports",
        )
        self.assertEqual(runtime, Path("/work/runtime/iap_sim_demo11_test_planner_1234"))
        self.assertEqual(
            export,
            Path("/work/exports/test_planner_p1_fork_formal_p1_fork_fused_v1_1234"),
        )

    def test_runtime_logging_materialization_routes_root_reference_and_timing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            runtime_base, config_path, logging_path = self._runtime_logging_fixture(root)
            requested = runtime_base / "iap_logs"

            effective = MODULE._materialize_iap_logging_config(
                config_path, runtime_base, requested
            )
            root_config = json.loads(config_path.read_text())
            referenced = json.loads(logging_path.read_text())

        self.assertEqual(root_config["logging"]["log_dir"], str(requested.resolve()))
        self.assertEqual(
            referenced["logging"]["log_dir"], str(requested.resolve())
        )
        self.assertEqual(
            root_config["global"]["timing_csv_path"],
            str((requested / "profiling" / "iap_timing.csv").resolve()),
        )
        self.assertTrue(root_config["logging"]["save_logs"])
        self.assertTrue(root_config["logging"]["rotate_logs"])
        self.assertEqual(root_config["logging"]["max_file_size_kb"], 8192)
        self.assertEqual(root_config["logging"]["max_files"], 10)
        self.assertEqual(effective["log_root"], str(requested.resolve()))
        self.assertNotIn("/repo/log", json.dumps(root_config))
        self.assertNotIn("/repo/log", json.dumps(referenced))

    def test_runtime_logging_materialization_rejects_missing_relative_and_escape(self):
        for requested in ("", "relative/log", "/outside/runtime/log"):
            with self.subTest(requested=requested), tempfile.TemporaryDirectory() as tmp:
                runtime_base, config_path, logging_path = self._runtime_logging_fixture(
                    Path(tmp)
                )
                root_before = config_path.read_text()
                logging_before = logging_path.read_text()
                with self.assertRaisesRegex(RuntimeError, "iap_log_root"):
                    MODULE._materialize_iap_logging_config(
                        config_path, runtime_base, requested
                    )
                self.assertEqual(config_path.read_text(), root_before)
                self.assertEqual(logging_path.read_text(), logging_before)

    def test_frozen_map_wiring_selects_truth_odom_stamp_authority(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(
            defaults["corridor_map_stamp_authority_topic"],
            "/sim/drone_0/truth_odom",
        )
        self.assertEqual(defaults["iap_log_root"], "")

    def test_covariance_growth_is_fail_closed_by_default_and_bound_at_ros_seam(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertTrue(math.isnan(float(
            defaults["p0.predictor.sigma_grow_m_sqrt_s"]
        )))
        self.assertEqual(
            defaults["p0.predictor.sigma_growth_profile"],
            "unconfigured_fail_closed",
        )
        source = Path(MODULE.__file__).read_text()
        self.assertIn(
            '{"p0.predictor.sigma_grow_m_sqrt_s": '
            'p0_covariance_growth["sigma_grow_m_sqrt_s"]}',
            source,
        )
        self.assertIn(
            '"p0.predictor.sigma_growth_profile": '
            'p0_covariance_growth["profile"]',
            source,
        )

    def test_covariance_growth_launch_materialization_is_exact_and_locale_free(self):
        context = LaunchContext()
        context.launch_configurations[
            "p0.predictor.sigma_grow_m_sqrt_s"
        ] = "0.01"
        context.launch_configurations[
            "p0.predictor.sigma_growth_profile"
        ] = "legacy_iap_rq320_baseline_v1"
        contract = MODULE._p0_covariance_growth_launch_contract(context)
        self.assertEqual(contract, {
            "sigma_grow_m_sqrt_s": 0.01,
            "profile": "legacy_iap_rq320_baseline_v1",
        })

        context.launch_configurations[
            "p0.predictor.sigma_grow_m_sqrt_s"
        ] = "0,01"
        with self.assertRaises(ValueError):
            MODULE._p0_covariance_growth_launch_contract(context)

    def test_p1_redesign_scenarios_are_named_and_have_fixed_contracts(self):
        expected = {
            "p1_fork_fused_v1",
            "p1_fork_fused_mirror_v1",
            "p1_fork_symmetric_null_v1",
            "p1_soft_risk_island_v1",
        }
        self.assertTrue(expected.issubset(MODULE.SCENARIO_PRESETS))
        for name in expected:
            preset = MODULE.SCENARIO_PRESETS[name]
            self.assertEqual(float(preset["init_z"]), 1.5)
            self.assertEqual(float(preset["goal_z"]), 1.5)
            self.assertEqual(preset["terminal_wall_enabled"], "false")
            self.assertEqual(int(preset["forest_random_seed"]), 41021)
            self.assertEqual(preset["p1_map_fixture"], name)
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["planner_start_delay_s"],
            "10.0",
        )
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["lidar_start_delay_s"], "0.0")
        self.assertEqual(defaults["odometry_initialization_mode"], "")
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]
            ["odometry_initialization_mode"],
            "NAIVE",
        )
        formal_lidar_delay = float(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["lidar_start_delay_s"]
        )
        self.assertGreater(formal_lidar_delay, 1.0)
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["manager/max_vel"],
            "1.0",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["optimization/max_vel"],
            "1.0",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["bspline/limit_vel"],
            "1.0",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["p0.horizons_s"],
            "0.0,0.5,1.0,1.5,2.0,2.5,3.5,4.5,5.5,6.5,7.5,8.5,9.5,10.5,11.5,12.5,13.5,14.5,15.5,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0",
        )
        horizons = [float(value) for value in
                    MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["p0.horizons_s"].split(",")]
        self.assertLessEqual(max(b - a for a, b in zip(horizons, horizons[1:])), 1.0)
        self.assertGreaterEqual(horizons[-1], 13.2)
        self.assertEqual(MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["p0.size_y_m"], "12.0")
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["grid_map/local_update_range_x"],
            "11.0",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["manager/planning_horizon"],
            "10.5",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]
            ["manager/p1_collision_fanout_clearance_m"],
            "2.5",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]
            ["p1_fixture_lane_center_m"],
            "2.5",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]
            ["manager/p1_collision_fanout_preserve_homotopies"],
            "true",
        )
        self.assertEqual(
            MODULE.EXPERIMENT_PRESETS["p1_fork_formal"]["fsm.thresh_replan_time"],
            "0.9",
        )
        self.assertEqual(
            MODULE.SCENARIO_PRESETS["p1_soft_risk_island_v1"]
            ["p1_fixture_central_obstacle_enabled"],
            "true",
        )

    def test_icra072_selection_trigger_is_separately_named_and_provider_complete(self):
        fixture_name = "icra072_p4_selection_trigger_v1"
        self.assertIn(fixture_name, MODULE.SCENARIO_PRESETS)
        fixture = MODULE.SCENARIO_PRESETS[fixture_name]
        self.assertEqual(fixture["p1_map_fixture"], fixture_name)
        self.assertEqual(fixture["p1_fixture_central_obstacle_enabled"], "true")
        self.assertEqual(fixture["p0.skip_occupied_voxels"], "false")
        self.assertEqual(fixture["forest_size_x_m"], "28.0")
        self.assertEqual(fixture["forest_size_y_m"], "10.0")
        self.assertEqual(fixture["corridor_x_min_m"], "-14.0")
        self.assertEqual(fixture["corridor_x_max_m"], "14.0")
        self.assertEqual(fixture["corridor_half_width_y_m"], "5.0")
        self.assertEqual(fixture["p1_fixture_central_x_min_m"], "-9.0")
        self.assertEqual(fixture["p1_fixture_central_x_max_m"], "-7.0")
        self.assertEqual(fixture["p1_fixture_central_y_half_width_m"], "0.4")
        self.assertEqual(fixture["p1_fixture_central_z_max_m"], "2.8")
        self.assertEqual(fixture["p1_fixture_lane_center_m"], "2.0")
        self.assertEqual(fixture["p1_fixture_safe_tree_density_per_m2"], "0.25")
        self.assertEqual(fixture["p1_fixture_risky_tree_density_per_m2"], "0.75")
        self.assertEqual(fixture["p1_fixture_safe_canopy_probability"], "0.05")
        self.assertEqual(fixture["p1_fixture_risky_canopy_probability"], "0.85")
        self.assertEqual(fixture["integrity_fusion_mode"], "max_pl")
        self.assertEqual(fixture["enable_gnss_integrity"], "true")
        self.assertEqual(fixture["enable_lidar_integrity"], "true")
        self.assertEqual(fixture["lidar_sensing_rate_hz"], "2.0")
        self.assertEqual(fixture["gnss_scenario_file"],
                         "config/gnss_sim/demo7_open_sky.yaml")
        self.assertEqual(fixture["gnss_pr_noise_base"], "1.0")
        self.assertEqual(fixture["gnss_dop_noise_base"], "0.03")
        self.assertEqual(fixture["gnss_enable_map_occlusion"], "false")
        self.assertEqual(fixture["gnss_enable_skymask"], "false")
        self.assertEqual(fixture["gnss_enable_nlos"], "false")
        self.assertEqual(fixture["gnss_enable_multipath"], "false")
        self.assertEqual(fixture["p0.predictor.use_current_integrity_prior"],
                         "true")
        self.assertNotIn("p5.current_pl_source", fixture)
        self.assertEqual(fixture["fsm.thresh_replan_time"], "0.2")
        self.assertEqual(fixture["manager/max_vel"], "0.5")
        self.assertEqual(fixture["optimization/max_vel"], "0.5")
        self.assertEqual(fixture["bspline/limit_vel"], "0.5")
        self.assertNotIn("inverse_corridor", fixture_name)
        profile = MODULE.EXPERIMENT_PRESETS["icra_p0_p4_v2_p5_dev"]
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertNotIn("p5.current_pl_source", defaults)
        self.assertEqual(profile["scenario"], fixture_name)
        self.assertEqual(
            profile["p0.predictor.sigma_grow_m_sqrt_s"], "0.01")
        self.assertEqual(
            profile["p0.predictor.sigma_growth_profile"],
            "legacy_iap_rq320_baseline_v1")
        self.assertEqual(defaults["p0.resolution_m"], "0.75")
        self.assertEqual(defaults["p0.size_x_m"], "30.0")
        self.assertEqual(defaults["p0.size_y_m"], "30.0")
        self.assertEqual(defaults["p0.size_z_m"], "6.0")
        self.assertEqual(defaults["p0.stale_timeout_s"], "1.0")
        self.assertEqual(
            profile["p0.horizons_s"],
            "0.0,0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0,6.0")
        self.assertEqual(defaults["map_size_x"], "30.0")
        self.assertEqual(defaults["map_size_y"], "30.0")
        self.assertEqual(defaults["map_size_z"], "3.5")
        self.assertEqual(defaults["p4.lambda_p4_risk"], "0.05")
        self.assertEqual(defaults["p4.max_extra_path_ratio"], "1.3")
        self.assertEqual(defaults["manager/max_vel"], "2.0")

    def test_dense_forest_four_fork_preset_is_real_sensor_driven_and_switchable(self):
        name = "icra_dense_forest_four_fork_v1"
        self.assertIn(name, MODULE.SCENARIO_PRESETS)
        preset = MODULE.SCENARIO_PRESETS[name]
        expected = {
            "forest_layout_mode": "forked_s_forest_v1",
            "map_size_x": "42.0",
            "map_size_y": "22.0",
            "map_size_z": "8.0",
            "forest_size_x_m": "40.0",
            "forest_size_y_m": "20.0",
            "forest_random_seed": "41021",
            "forked_forest.fork_count": "4",
            "forked_forest.fork_x_min_m": "-16.0",
            "forked_forest.fork_length_m": "8.0",
            "forked_forest.low_risk_amplitude_m": "4.0",
            "forked_forest.high_risk_amplitude_m": "2.8",
            "forked_forest.corridor_width_m": "2.4",
            "forked_forest.risk_seed": "21",
            "forked_forest.edge_tree_spacing_m": "0.5",
            "forked_forest.side_boundary_tree_spacing_m": "0.28",
            "init_x": "-18.0",
            "goal_x": "18.0",
            "p0.skip_occupied_voxels": "true",
            "p0.predictor.use_current_integrity_prior": "true",
            "p0.predictor.conservative_max_with_gnss": "true",
            "integrity_fusion_mode": "max_pl",
            "gnss_enable_map_occlusion": "true",
            "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
            "gnss_enable_skymask": "false",
            "gnss_enable_nlos": "true",
            "gnss_enable_multipath": "true",
            "gnss_enable_fault_injection": "false",
        }
        for key, value in expected.items():
            self.assertEqual(preset[key], value, key)
        self.assertEqual(preset["tree_density_lower_left_per_m2"], "0.25")
        self.assertEqual(preset["canopy_density_upper_right"], "0.65")
        self.assertEqual(preset["p1_map_fixture"], "")
        self.assertNotIn("p5.current_pl_source", preset)

        context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev", scenario=name)
        with mock.patch.object(
            sys, "argv",
            ["test", "experiment:=icra_p0_p4_v2_p5_dev", f"scenario:={name}"],
        ):
            scenario, experiment, _ = MODULE._apply_presets(context, REPO)
        self.assertEqual(scenario, name)
        self.assertEqual(experiment, "icra_p0_p4_v2_p5_dev")
        self.assertEqual(context.launch_configurations["forest_layout_mode"],
                         "forked_s_forest_v1")

    def test_icra_online_profile_uses_the_execution_map_geometry(self):
        """Online P0 overlays the same fixed lattice captured by GridMap."""
        profile = MODULE.EXPERIMENT_PRESETS["icra_p0_p4_v2_p5_dev"]
        self.assertEqual(profile["grid_map/origin_x"], "-15.0")
        self.assertEqual(profile["grid_map/origin_y"], "-15.0")
        self.assertEqual(profile["grid_map/origin_z"], "0.0")
        self.assertEqual(profile["p0.resolution_m"], "0.5")
        self.assertEqual(profile["p0.size_x_m"], "30.0")
        self.assertEqual(profile["p0.size_y_m"], "30.0")
        self.assertEqual(profile["p0.size_z_m"], "3.5")
        self.assertEqual(profile["p0.origin_x_m"], "-15.0")
        self.assertEqual(profile["p0.origin_y_m"], "-15.0")
        self.assertEqual(profile["p0.origin_z_m"], "0.0")

        context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev",
            scenario="icra072_p4_selection_trigger_v1",
        )
        with mock.patch.object(
            sys,
            "argv",
            [
                "test",
                "experiment:=icra_p0_p4_v2_p5_dev",
                "scenario:=icra072_p4_selection_trigger_v1",
            ],
        ):
            MODULE._apply_presets(context, REPO)
        self.assertEqual(context.launch_configurations["p0.resolution_m"], "0.5")
        self.assertEqual(context.launch_configurations["p0.size_z_m"], "3.5")
        self.assertEqual(context.launch_configurations["p0.origin_x_m"], "-15.0")

    def test_dense_forest_v2_is_online_and_truth_isolated(self):
        name = "icra_dense_forest_four_fork_v2"
        self.assertIn(name, MODULE.SCENARIO_PRESETS)
        preset = MODULE.SCENARIO_PRESETS[name]
        expected = {
            "forest_layout_mode": "forked_s_forest_v2",
            "forked_forest.start_canopy_clearance_radius_m": "5.0",
            "grid_map/origin_x": "-21.0",
            "grid_map/origin_y": "-11.0",
            "grid_map/origin_z": "0.0",
            "grid_map/unknown_as_occupied": "false",
            "manager/planning_horizon": "8.0",
            "fsm/planning_horizon": "8.0",
            "grid_map/local_update_range_x": "9.0",
            "grid_map/local_update_range_y": "9.0",
            "grid_map/local_update_range_z": "4.5",
            "planner_frame_mode": "glim_world",
            "planner_local_map_enable": "true",
            "allow_truth_alignment": "false",
            "p0.online_mapping_mode": "true",
            "p0.fit_grid_to_map_cloud": "false",
            "p0.map_topic": "",
            "p0.resolution_m": "0.5",
            "p0.size_x_m": "42.0",
            "p0.size_y_m": "22.0",
            "p0.size_z_m": "8.0",
            "p0.origin_x_m": "-21.0",
            "p0.origin_y_m": "-11.0",
            "p0.origin_z_m": "0.0",
            "p0.provider_cost_source": "pre_conservative_fim_ratio",
            "p0.require_safety_ratio_below_one_for_cost": "true",
            "p0.predictor.gnss_measured_epoch_support_radius_m": "0.0",
            "p4.assurance.task_mode": "mission_best_effort",
            "p0.predictor.gnss_measured_epoch_integrity_max_delta_s": "0.25",
            "p4.fallback_to_original_when_risk_not_ready": "false",
            "p4.forward.max_lookahead_m": "8.0",
            "p4.forward.sensing_range_m": "10.0",
            "p4.forward.nominal_query_speed_mps": "1.5",
            "p4.forward.route_compute_budget_ms": "1200.0",
            "p4.forward.compute_budget_ms": "150.0",
            "p4.execution.successor_prepare_wcet_s": "1.2",
            "p4.execution.successor_max_parent_execution_s": "2.5",
            "p4.forward.max_channel_searches": "32",
            "p4.forward.channel_enumeration_budget_ms": "250.0",
            "p4.forward.advisory_min_relative_improvement": "0.10",
            "p4.forward.min_creep_progress_m": "0.25",
            "p4.forward.max_limited_prefix_progress_m": "8.0",
            "p4.execution.marginal_unsafe_ratio_max": "1.005",
            "p4.execution.marginal_confirm_distinct_evidence": "3",
            "p4.execution.marginal_confirm_max_s": "0.35",
            "p4.forward.max_creep_progress_m": "-1.0",
            "p4.forward.max_observe_speed_mps": "0.5",
            "integrity_dynamic_alert_limits": "false",
            "integrity_hal_m": "20.0",
            "integrity_val_m": "40.0",
            "p0.alert_limit_policy_id": "fixed_hal20_val40_v1",
            "p0.alert_limit_h_m": "20.0",
            "p0.alert_limit_v_m": "40.0",
            "p5.pred_alert_limit_constant_hal_m": "20.0",
            "p5.pred_alert_limit_constant_val_m": "40.0",
        }
        for key, value in expected.items():
            self.assertEqual(preset[key], value, key)

        context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev", scenario=name)
        with mock.patch.object(
            sys, "argv",
            ["test", "experiment:=icra_p0_p4_v2_p5_dev", f"scenario:={name}"],
        ):
            scenario, _, _ = MODULE._apply_presets(context, REPO)
        self.assertEqual(scenario, name)
        MODULE._validate_online_truth_isolation(context, scenario)
        contract, contract_id = MODULE._planner_local_map_contract(context)
        self.assertEqual(
            contract["static_planner_from_glim_translation_m"],
            [-18.0, 0.0, 1.5],
        )
        self.assertEqual(contract["geofence_origin_m"], [-21.0, -11.0, 0.0])
        self.assertRegex(contract_id, r"^sha256:[0-9a-f]{64}$")
        self.assertEqual(
            MODULE._planner_local_map_contract(context)[1], contract_id)

    def test_dense_forest_v2_has_task_specific_mission_exposure_horizon(self):
        continuous_key = "p4.assurance.maximum_continuous_exceedance_s"
        integral_key = "p4.assurance.maximum_exceedance_integral_ratio_s"

        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults[continuous_key], "2.3")
        self.assertEqual(defaults[integral_key], "0.115")

        name = "icra_dense_forest_four_fork_v2"
        preset = MODULE.SCENARIO_PRESETS[name]
        self.assertEqual(preset.get(continuous_key), "8.0")
        self.assertEqual(preset.get(integral_key), "0.4")

        context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev", scenario=name)
        with mock.patch.object(
            sys, "argv",
            ["test", "experiment:=icra_p0_p4_v2_p5_dev", f"scenario:={name}"],
        ):
            MODULE._apply_presets(context, REPO)
        self.assertEqual(context.launch_configurations[continuous_key], "8.0")
        self.assertEqual(context.launch_configurations[integral_key], "0.4")

        other_context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev",
            scenario="icra_dense_forest_four_fork_v1",
        )
        with mock.patch.object(
            sys,
            "argv",
            [
                "test",
                "experiment:=icra_p0_p4_v2_p5_dev",
                "scenario:=icra_dense_forest_four_fork_v1",
            ],
        ):
            MODULE._apply_presets(other_context, REPO)
        self.assertEqual(other_context.launch_configurations[continuous_key], "2.3")
        self.assertEqual(other_context.launch_configurations[integral_key], "0.115")

    def test_online_truth_isolation_rejects_simulator_topics(self):
        context = self._launch_context_with_defaults(
            experiment="icra_p0_p4_v2_p5_dev",
            scenario="icra_dense_forest_four_fork_v2",
        )
        context.launch_configurations["p0.online_mapping_mode"] = "true"
        for topic in ("/map_generator/global_cloud", "/sim/world/forest"):
            with self.subTest(topic=topic):
                context.launch_configurations["p0.map_topic"] = topic
                with self.assertRaisesRegex(RuntimeError, "truth topic"):
                    MODULE._validate_online_truth_isolation(
                        context, "icra_dense_forest_four_fork_v2")

    def test_forest_v2_rejects_truth_alignment_or_missing_local_map(self):
        for key, value in (
            ("allow_truth_alignment", "true"),
            ("planner_local_map_enable", "false"),
            ("planner_frame_mode", "legacy_truth_aligned"),
        ):
            with self.subTest(key=key):
                context = self._launch_context_with_defaults(
                    experiment="icra_p0_p4_v2_p5_dev",
                    scenario="icra_dense_forest_four_fork_v2",
                )
                with mock.patch.object(
                    sys,
                    "argv",
                    [
                        "test",
                        "experiment:=icra_p0_p4_v2_p5_dev",
                        "scenario:=icra_dense_forest_four_fork_v2",
                    ],
                ):
                    scenario, _, _ = MODULE._apply_presets(context, REPO)
                context.launch_configurations[key] = value
                with self.assertRaises(RuntimeError):
                    MODULE._validate_online_truth_isolation(context, scenario)

    def test_fork_and_mirror_share_geometry_identity_except_mirror_flag(self):
        primary = MODULE.SCENARIO_PRESETS["p1_fork_fused_v1"]
        mirror = MODULE.SCENARIO_PRESETS["p1_fork_fused_mirror_v1"]
        differing = {
            key for key in set(primary) | set(mirror)
            if primary.get(key) != mirror.get(key)
        }
        self.assertEqual(differing, {"p1_map_fixture", "p1_fixture_mirror_y"})

    def test_fanout_mirror_preserves_legacy_fallback_but_accepts_gate0_override(self):
        self.assertTrue(
            MODULE._resolve_fanout_mirror_value(
                fixture_mirror=True,
                manager_mirror=False,
                explicit_overrides=set(),
            )
        )
        self.assertFalse(
            MODULE._resolve_fanout_mirror_value(
                fixture_mirror=True,
                manager_mirror=False,
                explicit_overrides={"manager/p1_collision_fanout_mirror_y"},
            )
        )

    def test_gate0_launch_contract_declares_all_read_only_evidence_fields(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["gate0.qualification_evidence_enable"], "false")
        self.assertEqual(defaults["iap_mapping_backend"], "gpu")
        for field in (
            "gate0.candidate_events_path",
            "gate0.control_points_path",
            "gate0.evidence_run_id",
            "gate0.evidence_manifest_path",
        ):
            self.assertIn(field, defaults)
            self.assertEqual(defaults[field], "")
        source = Path(MODULE.__file__).read_text()
        for field in (
            '"p1_fixture_mirror_y":',
            '"p2.enable_candidate_ranking":',
            '"p3.enable_local_reference_bias":',
            '"p4.enable_risk_aware_astar":',
            '"manager/use_distinctive_trajs":',
        ):
            self.assertIn(field, source)

    def test_iap_mapping_backend_is_declared_with_gpu_default_and_hashes_effective_config(self):
        defaults = dict(MODULE.ARG_DEFAULTS)
        self.assertEqual(defaults["iap_mapping_backend"], "gpu")
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "effective.json"
            path.write_text("{}\n")
            provenance = MODULE._mapping_backend_config_provenance(str(path))
            self.assertEqual(provenance["path"], str(path.resolve()))
            self.assertEqual(
                provenance["sha256"], hashlib.sha256(path.read_bytes()).hexdigest()
            )
        self.assertEqual(MODULE._normalize_mapping_backend("CPU"), "cpu")
        with self.assertRaisesRegex(RuntimeError, "gpu or cpu"):
            MODULE._normalize_mapping_backend("auto")

    def test_scenario_fingerprint_is_canonical_and_sensitive(self):
        payload = {"geometry": {"seed": 11, "density": 0.25}, "risk": ["gnss", "lidar"]}
        first = MODULE._scenario_fingerprint("p1_fork_fused_v1", payload)
        reordered = MODULE._scenario_fingerprint(
            "p1_fork_fused_v1",
            {"risk": ["gnss", "lidar"], "geometry": {"density": 0.25, "seed": 11}},
        )
        changed = MODULE._scenario_fingerprint(
            "p1_fork_fused_v1",
            {"geometry": {"seed": 12, "density": 0.25}, "risk": ["gnss", "lidar"]},
        )
        self.assertEqual(first, reordered)
        self.assertNotEqual(first, changed)
        self.assertRegex(first, r"^sha256:[0-9a-f]{64}$")

    def test_p1_fixed_lattice_keeps_replanning_to_planner_goal_boundary(self):
        self.assertEqual(
            MODULE._fixed_lattice_no_replan_threshold({"p1": True}), 0.2
        )
        self.assertEqual(
            MODULE._fixed_lattice_no_replan_threshold({"p1": False}), 1.0
        )

    def test_runtime_odometry_mode_override_preserves_commented_config(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "config_odometry_gpu.json"
            path.write_text(
                '{\n  "odometry_estimation": {\n'
                '    "initialization_mode": "LOOSE", // permitted modes\n'
                '    "initialization_window_size": 1.0\n  }\n}\n'
            )
            MODULE._override_odometry_initialization_mode(path, "NAIVE")
            updated = path.read_text()
            self.assertIn('"initialization_mode": "NAIVE", // permitted modes', updated)
            self.assertIn('"initialization_window_size": 1.0', updated)

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

    @staticmethod
    def _launch_context_with_defaults(**updates):
        context = LaunchContext()
        context.launch_configurations.update(dict(MODULE.ARG_DEFAULTS))
        context.launch_configurations.update(updates)
        return context

    def test_icra_p0_p5_qualification_arm_resolves_from_one_contract(self):
        experiment = "icra_p0_p5_qualification_final_reject"
        context = self._launch_context_with_defaults(experiment=experiment)
        with mock.patch.object(sys, "argv", ["test", f"experiment:={experiment}"]):
            scenario, selected, applied = MODULE._apply_presets(context, REPO)
            profile, enabled, p0_enabled, conflict, _ = MODULE._resolve_safety_switches(
                context, applied
            )
        self.assertEqual(selected, experiment)
        self.assertEqual(scenario, "icra_p0_p5_fused_degraded_corridor_v1")
        self.assertEqual(profile, "icra_p0_p5")
        self.assertEqual(enabled, {
            "p1": False, "p2": False, "p3_local": False,
            "p3_global": False, "p4": False,
            "p5_runtime": True, "p5_final": True,
        })
        self.assertTrue(p0_enabled)
        self.assertFalse(conflict)
        self.assertEqual(context.launch_configurations["p0.predictor.worker_count"], "4")
        self.assertEqual(context.launch_configurations["p5_7.fixture.enabled"], "true")
        self.assertEqual(context.launch_configurations["p5_6.fixture.enabled"], "false")

    def test_icra_p0_p4_v2_p5_dev_profile_enables_only_active_vertical_slice(self):
        experiment = "icra_p0_p4_v2_p5_dev"
        context = self._launch_context_with_defaults(experiment=experiment)
        with mock.patch.object(sys, "argv", ["test", f"experiment:={experiment}"]):
            scenario, selected, applied = MODULE._apply_presets(context, REPO)
            profile, enabled, p0_enabled, conflict, _ = (
                MODULE._resolve_safety_switches(context, applied)
            )
        self.assertEqual(selected, experiment)
        self.assertEqual(scenario, "icra072_p4_selection_trigger_v1")
        self.assertEqual(profile, "icra_p0_p4_v2_p5_dev")
        self.assertEqual(enabled, {
            "p1": False, "p2": False, "p3_local": False,
            "p3_global": False, "p4": True,
            "p5_runtime": True, "p5_final": True,
        })
        self.assertTrue(p0_enabled)
        self.assertFalse(conflict)
        self.assertEqual(
            context.launch_configurations["p4.objective"],
            "PROVIDER_BOTTLENECK_V2",
        )
        self.assertEqual(context.launch_configurations["p4.metrics_only"], "false")
        self.assertEqual(
            context.launch_configurations["p0.predictor.sigma_grow_m_sqrt_s"],
            "0.01",
        )
        self.assertEqual(
            context.launch_configurations["p0.predictor.sigma_growth_profile"],
            "legacy_iap_rq320_baseline_v1",
        )
        self.assertEqual(
            context.launch_configurations["p0.predictor.worker_count"], "8"
        )
        self.assertEqual(context.launch_configurations["record_bag"], "false")
        self.assertEqual(context.launch_configurations["start_rviz"], "false")
        for fixture in ("p5_3", "p5_4", "p5_5", "p5_6", "p5_7"):
            self.assertEqual(
                context.launch_configurations[f"{fixture}.fixture.enabled"],
                "false",
            )

    def test_p4_debug_manifest_binding_matches_effective_node_path(self):
        source = Path(MODULE.__file__).read_text()
        self.assertIn(
            '"p4.debug_csv_path": p4_debug_path_for_manifest', source)
        self.assertIn(
            'p4_debug_path_for_manifest = '
            'LaunchConfiguration("p4.debug_csv_path").perform(context)',
            source,
        )
        self.assertIn(
            'p4_debug_path_for_manifest = str('
            'Path(export_dir) / "planner_p4_risk_astar_debug.csv")',
            source,
        )

    def test_icra_p0_p5_all_cases_resolve_exact_full_sensor_scenario(self):
        frozen = {
            "use_gnss": "true",
            "use_araim": "true",
            "enable_gnss_integrity": "true",
            "enable_gnss_araim": "true",
            "enable_lidar_integrity": "true",
            "integrity_fusion_mode": "max_pl",
            "validator_require_gnss_valid": "true",
            "validator_require_lidar_valid": "true",
            "gnss_time_source": "trigger_topic",
            "gnss_ephemeris_source": "rinex",
            "gnss_scenario_file": str(
                REPO / "results/icra27/icra070/install_v2/share/iap/"
                "config/gnss_sim/demo7_skymask_nlos.yaml"
            ),
            "gnss_rinex_nav_file": (
                "/home/dev/ws_iap/src/LIGO./Data/"
                "BRDM00DLR_S_20221870000_01D_MN.rnx"
            ),
            "gnss_trigger_topic": "/sim/drone_0/lidar",
            "gnss_fallback_to_synthetic_on_rinex_error": "false",
            "gnss_pr_noise_base": "5.0",
            "gnss_dop_noise_base": "0.5",
            "gnss_enable_map_occlusion": "true",
            "gnss_enable_skymask": "true",
            "gnss_enable_nlos": "true",
            "gnss_enable_multipath": "true",
            "gnss_enable_fault_injection": "false",
            "init_x": "-12.0",
            "init_y": "0.0",
            "init_z": "1.2",
            "goal_x": "12.0",
            "goal_y": "0.0",
            "goal_z": "1.2",
            "corridor_x_min_m": "-14.0",
            "corridor_x_max_m": "14.0",
            "corridor_half_width_y_m": "2.0",
        }
        experiments = (
            "icra_p0_p5_qualification_safe_normal",
            "icra_p0_p5_qualification_final_reject",
            "icra_p0_p5_qualification_runtime_fail",
        )
        for experiment in experiments:
            with self.subTest(experiment=experiment):
                context = self._launch_context_with_defaults(experiment=experiment)
                with mock.patch.object(
                    sys, "argv", ["test", f"experiment:={experiment}"]
                ):
                    scenario, _, _ = MODULE._apply_presets(context, REPO)
                self.assertEqual(
                    scenario, "icra_p0_p5_fused_degraded_corridor_v1"
                )
                for key, expected in frozen.items():
                    self.assertEqual(
                        context.launch_configurations[key], expected, key
                    )

    def test_icra_p0_p5_launch_rejects_equal_level_and_lower_level_conflicts(self):
        experiment = "icra_p0_p5_qualification_safe_normal"
        for key, value in (
            ("planner_enable_p1", "true"),
            ("p1.metrics_only", "true"),
            ("planner_enable_p5_final", "false"),
            ("p5_7.fixture.enabled", "true"),
        ):
            with self.subTest(key=key):
                context = self._launch_context_with_defaults(
                    experiment=experiment, **{key: value}
                )
                with mock.patch.object(
                    sys, "argv", ["test", f"experiment:={experiment}", f"{key}:={value}"]
                ), self.assertRaisesRegex(RuntimeError, "conflicting explicit override"):
                    MODULE._apply_presets(context, REPO)

    def test_icra_p0_p5_launch_binding_carries_prospective_identity(self):
        experiment = "icra_p0_p5_qualification_runtime_fail"
        context = self._launch_context_with_defaults(experiment=experiment)
        with mock.patch.object(sys, "argv", ["test", f"experiment:={experiment}"]):
            MODULE._apply_presets(context, REPO)
        binding = MODULE._icra_p0_p5_launch_binding(
            context,
            experiment,
            REPO,
            {"git_commit": "a" * 40, "run_id": "prospective-run"},
        )
        self.assertEqual(binding["case_id"], "RUNTIME_FAIL")
        self.assertEqual(binding["git_commit"], "a" * 40)
        self.assertEqual(binding["run_id"], "prospective-run")
        self.assertEqual(binding["fixture_alias"], "p5_6_future_unknown_zone_v1")
        self.assertEqual(binding["raw_artifact_hashes"], "REQUIRED_AT_ANALYSIS")
        self.assertEqual(binding["p0_profile"]["worker_count"], 4)
        self.assertEqual(binding["p5_thresholds"]["p5.horizon_s"], 2.0)

    def test_named_icra_p0_p5_profile_does_not_arm_a_qualification_case(self):
        context = self._launch_context_with_defaults(
            experiment="baseline_corridor_off",
            planner_safety_profile="icra_p0_p5",
        )
        with mock.patch.object(
            sys, "argv",
            ["test", "experiment:=baseline_corridor_off", "planner_safety_profile:=icra_p0_p5"],
        ):
            scenario, experiment, _ = MODULE._apply_presets(context, REPO)
        self.assertEqual(experiment, "baseline_corridor_off")
        self.assertEqual(scenario, "lidar_corridor_degenerate")
        self.assertEqual(context.launch_configurations["experiment"], experiment)
        self.assertEqual(context.launch_configurations["scenario"], scenario)
        self.assertEqual(context.launch_configurations["p5_7.fixture.enabled"], "false")
        self.assertEqual(context.launch_configurations["p5_6.fixture.enabled"], "false")


if __name__ == "__main__":
    unittest.main()
