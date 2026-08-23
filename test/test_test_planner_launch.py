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


if __name__ == "__main__":
    unittest.main()
