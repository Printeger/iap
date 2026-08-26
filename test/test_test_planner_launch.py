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
        self.assertEqual(scenario, "icra_p0_p4_v2_p5_dev_fixture_v1")
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
