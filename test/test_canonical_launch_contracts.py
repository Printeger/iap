import importlib.util
import fcntl
import json
import os
import sys
import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
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

    @staticmethod
    def _make_retention_run(
        root: Path,
        name: str,
        *,
        ended_at: datetime,
        lifecycle: str = "completed",
        run_class: str = "development",
        retention_class: str = "ordinary",
    ) -> Path:
        run = root / name
        (run / "metadata").mkdir(parents=True)
        manifest = {
            "schema_version": "iap_run_artifact_v1",
            "run_id": name,
            "run_root": str(root),
            "run_dir": str(run),
            "started_at_utc": (ended_at - timedelta(hours=1)).isoformat().replace(
                "+00:00", "Z"
            ),
            "ended_at_utc": ended_at.isoformat().replace("+00:00", "Z"),
            "lifecycle": lifecycle,
            "safety_outcome": "not_applicable",
            "run_class": run_class,
            "retention_class": retention_class,
        }
        (run / "metadata/run_manifest.json").write_text(
            json.dumps(manifest), encoding="utf-8"
        )
        return run

    def test_stage1_parameters_have_one_map_and_no_retired_planner_controls(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        catalog = json.loads((REPO / "config/scenarios/catalog.json").read_text())
        for entry in catalog.values():
            params = runtime.planner_parameters(entry)
            self.assertEqual(params["grid_map/resolution"], 0.1)
            self.assertTrue(params["grid_map/registered_lidar_window_enabled"])
            self.assertEqual(params["grid_map/frame_id"], "map")
            self.assertEqual(params["grid_map/visualization_period_s"], 1.0)
            self.assertTrue(params["risk_viz/enabled"])
            self.assertEqual(params["risk_viz/metric"], "hpl")
            self.assertLess(params["risk_viz/hpl_max_m"], 1.0)
            self.assertEqual(params["risk_viz/vpl_min_m"], 0.20)
            self.assertEqual(params["risk_viz/vpl_max_m"], 0.55)
            self.assertEqual(params["risk_viz/history_lifetime_s"], 60.0)
            self.assertFalse(any(key.startswith(("p0.", "p1.", "p2.", "p3.", "p4.", "p5.")) for key in params))
            self.assertNotIn("manager/use_distinctive_trajs", params)
            for i, axis in enumerate("xyz"):
                self.assertEqual(params[f"grid_map/map_size_{axis}"], entry["map_size"][i])

    def test_stage1_graph_materializes_same_registered_lattice_and_preserves_artifacts(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        runs = self._load_launch("_includes/run_directory.py")
        catalog = json.loads((REPO / "config/scenarios/catalog.json").read_text())
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
            run = runs.resolve_run_directory(entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2")
            with mock.patch.dict(os.environ, {"IAP_RUN_DIR": str(run)}), mock.patch.object(runtime, "get_package_share_directory", return_value=str(REPO)):
                context = LaunchContext()
                context.launch_configurations.update({"scenario": "icra_dense_forest_four_fork_v2", "start_rviz": "false",
                    "planner_start_delay_s": "0", "run_duration_s": "0", "p0.enable_risk_grid": "true"})
                actions = runtime._setup(context)
            self.assertEqual(len(actions), 4)
            ros = json.loads((run / "metadata/config/iap/config_ros.json").read_text())["glim_ros"]
            local = ros["planner_local_map"]
            params = runtime.planner_parameters(catalog["icra_dense_forest_four_fork_v2"])
            self.assertEqual(local["planning_lattice_resolution_m"],params["grid_map/resolution"])
            self.assertEqual(local["frame_contract_id"],params["grid_map/registered_frame_contract_id"])
            self.assertEqual(local["planning_lattice_extent_m"],catalog["icra_dense_forest_four_fork_v2"]["map_size"])
            self.assertEqual(ros["acc_scale"], 1.0)
            odometry = json.loads((run / "metadata/config/iap/config_odometry.json").read_text())
            self.assertEqual(odometry["odometry_estimation"]["initialization_mode"], "NAIVE")
            self.assertFalse(ros["sim"]["align_planner_odom_to_truth"])
            self.assertEqual(ros["sim"]["static_planner_translation_m"],local["static_planner_translation_m"])
            self.assertIn("libplanner_local_map_extension.so",ros["extension_modules"])
            manifest=json.loads((run / "metadata/run_manifest.json").read_text())
            self.assertIn("metadata/config/iap",manifest["config_snapshots"])

    def test_sim_controller_and_rviz_use_the_current_grid_map(self):
        for filename in ("iap_sim.launch.py", "_includes/simulation_environment.launch.py"):
            self.assertIn(
                'DeclareLaunchArgument("scenario", default_value="icra_dense_forest_four_fork_v2")',
                (LAUNCH / filename).read_text(),
            )
        environment = (LAUNCH / "_includes/simulation_environment.launch.py").read_text()
        self.assertIn('("odom", "/drone_0_visual_slam/odom")', environment)
        module = self._load_launch("_includes/simulation_environment.launch.py")
        profile = module._map_parameters("feature_rich", [-12.0, 0.0, 1.2], [12.0, 0.0, 1.2])
        self.assertEqual(profile["endpoint_clearance_radius_m"], 1.0)
        self.assertEqual((profile["endpoint_start_x_m"], profile["endpoint_goal_x_m"]),
                         (-12.0, 12.0))
        rviz = (REPO / "config/sim_ego/grid_map_stage1.rviz").read_text()
        for topic in ("/grid_map/occupancy", "/grid_map/occupancy_inflate",
                      "/grid_map/risk_slice", "/grid_map/risk_status",
                      "/grid_map/risk_surface", "/grid_map/risk_history",
                      "/grid_map/risk_legend", "/grid_map/glio_path",
                      "/planning/trajectory_curve"):
            self.assertIn(topic, rviz)
        self.assertNotIn("/map_generator/global_cloud", rviz)
        self.assertNotIn("/iap/rviz/p1_", rviz)

    def test_flight_is_unavailable_during_execution_rebuild(self):
        flight=self._load_launch("iap_flight.launch.py")
        context=LaunchContext()
        context.launch_configurations.update({"flight_authorized":"true", "controller_handshake_confirmed":"true"})
        with self.assertRaisesRegex(RuntimeError,"stages 4/5"):
            flight._setup(context)

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
        environment = self._load_launch("_includes/simulation_environment.launch.py")
        for profile in ("open_sky", "degraded", "open_sky_occlusion"):
            params = environment._gnss_parameters(REPO, profile)
            path = Path(params["scenario_file"])
            self.assertTrue(path.is_file())
            self.assertTrue(path.is_relative_to(REPO))


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

    def test_glio_starts_maintained_rviz_by_default(self):
        source = (LAUNCH / "glio.launch.py").read_text(encoding="utf-8")
        self.assertIn('package="rviz2"', source)
        self.assertIn('"start_rviz",', source)
        self.assertIn('default_value="true"', source)
        self.assertTrue((REPO / "config/profiles/glio/glio.rviz").is_file())

    def test_glio_integrity_starts_maintained_rviz_by_default(self):
        source = (LAUNCH / "glio_integrity.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('package="rviz2"', source)
        self.assertIn('"start_rviz",', source)
        self.assertIn('default_value="true"', source)
        rviz = REPO / "config/profiles/glio_integrity/glio_integrity.rviz"
        self.assertTrue(rviz.is_file())
        rviz_source = rviz.read_text(encoding="utf-8")
        self.assertIn("/glio_integrity/odom", rviz_source)
        self.assertIn("/iap/araim_envelopes", rviz_source)
        gnss = json.loads(
            (REPO / "config/profiles/glio_integrity/config_gnss.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertTrue(gnss["integrity"]["enable_araim_markers"])
        self.assertEqual(
            gnss["integrity"]["araim_marker_topic"], "/iap/araim_envelopes"
        )

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

    def test_each_canonical_entrypoint_has_one_owner_and_propagates_both_run_envs(self):
        for filename in (
            "glio.launch.py",
            "glio_integrity.launch.py",
            "iap_sim.launch.py",
        ):
            source = (LAUNCH / filename).read_text(encoding="utf-8")
            self.assertEqual(source.count("resolve_run_directory("), 1, filename)
            self.assertIn('"IAP_RUN_DIR"', source, filename)
            self.assertIn('"ROS_LOG_DIR"', source, filename)
        for filename in ("glio.launch.py", "glio_integrity.launch.py"):
            source = (LAUNCH / filename).read_text(encoding="utf-8")
            self.assertIn("adopt_run_directory(internal_run)", source, filename)


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
            glio["glim_ros"]["extension_modules"],
            ["libgnss_extension.so", "librviz_viewer.so"],
        )
        self.assertEqual(
            integrity["glim_ros"]["extension_modules"],
            [
                "libgnss_extension.so",
                "libintegrity_extension.so",
                "librviz_viewer.so",
            ],
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
            self.assertEqual(
                Path(root["global"]["timing_csv_path"]),
                output / "profiling/iap_timing.csv",
            )
            self.assertTrue(
                Path(ros["glim_ros"]["dump_path"]).is_relative_to(output)
            )
            self.assertEqual(
                Path(ros["glim_ros"]["dump_path"]),
                output / "export/glio/dump",
            )
            self.assertTrue(
                Path(gnss["gnss"]["debug_csv_path"]).is_relative_to(output)
            )
            self.assertEqual(
                Path(gnss["integrity"]["araim_csv_path"]),
                output / "export/current_integrity/iap_araim.csv",
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

    def test_legacy_artifact_paths_use_only_basename_and_warn(self):
        helper = self._load_launch("_includes/profile_runtime.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            with self.assertWarns(DeprecationWarning):
                redirected = helper._redirect_artifact_paths(
                    {
                        "debug_csv_path": "/tmp/legacy/debug.csv",
                        "dump_path": "/tmp/legacy/dump-v1",
                        "log_dir": "/tmp/legacy/logs",
                    },
                    export_dir=root / "export/glio",
                    log_dir=root / "runtime",
                    dump_dir=root / "export/glio/dump",
                )
            self.assertEqual(
                Path(redirected["debug_csv_path"]), root / "export/glio/debug.csv"
            )
            self.assertEqual(
                Path(redirected["dump_path"]), root / "export/glio/dump-v1"
            )
            self.assertEqual(Path(redirected["log_dir"]), root / "runtime")



    def test_sim_paths_follow_canonical_run_categories(self):
        simulation = self._load_launch("iap_sim.launch.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ), mock.patch.object(
            simulation, "get_package_share_directory", return_value=str(REPO)
        ):
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "scenario": "lidar_corridor_degenerate",
                    "start_rviz": "true",
                    "planner_start_delay_s": "10.0",
                    "run_duration_s": "60.0",
                }
            )
            actions = simulation._setup(context)
            action_names = [type(action).__name__ for action in actions]
            self.assertLess(
                action_names.index("RegisterEventHandler"),
                action_names.index("IncludeLaunchDescription"),
            )
            include = next(
                action
                for action in actions
                if type(action).__name__ == "IncludeLaunchDescription"
            )
            arguments = dict(include.launch_arguments)
            runtime_root = Path(arguments["runtime_root_dir"])
            log_root = Path(arguments["iap_log_root"])
            run_dir = log_root.parent
            self.assertEqual(
                runtime_root, run_dir / "metadata" / "config" / "full_stack"
            )
            self.assertEqual(log_root, run_dir / "runtime")






    def test_bp_is_installed_for_frozen_script_compatibility(self):
        cmake = (REPO / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertNotIn('PATTERN "bp" EXCLUDE', cmake)

    def test_automatic_run_directories_are_timestamped_and_collision_safe(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ), mock.patch.object(helper, "_new_run_id", return_value="20260929T120000Z_000"):
            first = helper.resolve_run_directory(entrypoint="glio")
            second = helper.resolve_run_directory(entrypoint="glio")
            self.assertEqual(first, Path(temporary) / "20260929T120000Z_000")
            self.assertEqual(second, Path(temporary) / "20260929T120000Z_000_01")
            self.assertTrue(first.is_dir())
            self.assertTrue(second.is_dir())
            self.assertEqual((Path(temporary) / "latest").resolve(), second)
            self.assertEqual(
                {path.name for path in second.iterdir() if path.is_dir()},
                {"runtime", "profiling", "export", "metadata"},
            )
            manifest = json.loads(
                (second / "metadata/run_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["schema_version"], "iap_run_artifact_v1")
            self.assertEqual(manifest["run_id"], second.name)
            self.assertEqual(manifest["entrypoint"], "glio")
            self.assertEqual(manifest["lifecycle"], "active")

    def test_concurrent_allocation_is_unique_and_latest_is_atomic(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ), mock.patch.object(helper, "_new_run_id", return_value="20260929T120000Z_000"):
            with ThreadPoolExecutor(max_workers=8) as executor:
                runs = list(
                    executor.map(
                        lambda _: helper.resolve_run_directory(entrypoint="glio"),
                        range(8),
                    )
                )

            self.assertEqual(len(set(runs)), 8)
            self.assertEqual(
                (Path(temporary) / "latest").resolve(), max(runs, key=lambda p: p.name)
            )
            for run in runs:
                helper.finalize_run(run, lifecycle="completed")

    def test_automatic_sim_run_directory_is_grouped_by_scenario(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ):
            run_dir = helper.resolve_run_directory(
                entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2"
            )
            self.assertEqual(run_dir.parent, Path(temporary))

    def test_source_workspace_automatic_root_uses_repository_log_tree(self):
        helper = self._load_launch("_includes/run_directory.py")
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("IAP_RUN_ROOT", None)
            self.assertEqual(helper._default_run_root(), REPO / "log")

    def test_output_dir_is_not_a_canonical_launch_argument(self):
        for filename in (
            "glio.launch.py",
            "glio_integrity.launch.py",
            "iap_sim.launch.py",
            "iap_flight.launch.py",
        ):
            source = (LAUNCH / filename).read_text(encoding="utf-8")
            self.assertNotIn('LaunchConfiguration("output_dir")', source, filename)
            self.assertNotIn('"output_dir",\n', source, filename)

    def test_latest_never_regresses_to_an_older_automatic_run(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            older = parent / "20260929T010101Z_001"
            newer = parent / "20260929T010101Z_002"
            older.mkdir()
            newer.mkdir()
            helper._update_latest(parent, newer)
            helper._update_latest(parent, older)
            self.assertEqual((parent / "latest").resolve(), newer)

    def test_latest_refuses_to_replace_an_ordinary_file(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            run = parent / "20260929T010101Z_001"
            run.mkdir()
            latest = parent / "latest"
            latest.write_text("operator-owned", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "not a symlink"):
                helper._update_latest(parent, run)
            self.assertEqual(latest.read_text(encoding="utf-8"), "operator-owned")

    def test_retention_keeps_latest_three_and_all_runs_from_last_seven_days(self):
        helper = self._load_launch("_includes/run_directory.py")
        now = datetime(2026, 9, 29, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runs = []
            for day in (1, 2, 3, 4, 5):
                runs.append(
                    self._make_retention_run(
                        root,
                        f"2026090{day}T000000Z_000",
                        ended_at=datetime(2026, 9, day, tzinfo=timezone.utc),
                    )
                )
            recent = self._make_retention_run(
                root,
                "20260801T000000Z_000",
                ended_at=now - timedelta(days=6),
            )
            boundary = self._make_retention_run(
                root,
                "20260701T000000Z_000",
                ended_at=now - timedelta(days=7),
            )

            removed = helper.prune_development_runs(root, now=now)

            self.assertEqual(
                {path.name for path in removed},
                {"20260901T000000Z_000", "20260902T000000Z_000"},
            )
            self.assertTrue(recent.is_dir())
            self.assertTrue(boundary.is_dir())
            for run in runs[2:]:
                self.assertTrue(run.is_dir())
            self.assertFalse(any(root.glob(".retention-trash.*")))

    def test_retention_skips_nonordinary_unfinished_invalid_symlink_and_locked_runs(self):
        helper = self._load_launch("_includes/run_directory.py")
        now = datetime(2026, 9, 29, tzinfo=timezone.utc)
        old = now - timedelta(days=30)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            ordinary = [
                self._make_retention_run(
                    root, f"2026080{day}T000000Z_000", ended_at=old
                )
                for day in (1, 2, 3, 4)
            ]
            active = self._make_retention_run(
                root,
                "20260805T000000Z_000",
                ended_at=old,
                lifecycle="active",
            )
            formal = self._make_retention_run(
                root,
                "20260806T000000Z_000",
                ended_at=old,
                run_class="formal",
            )
            protected = self._make_retention_run(
                root,
                "20260807T000000Z_000",
                ended_at=old,
                retention_class="protected",
            )
            invalid = root / "20260808T000000Z_000"
            (invalid / "metadata").mkdir(parents=True)
            (invalid / "metadata/run_manifest.json").write_text("not-json")
            outside = root / "outside"
            outside.mkdir()
            linked = root / "20260809T000000Z_000"
            linked.symlink_to(outside, target_is_directory=True)
            ordinary_file = root / "20260811T000000Z_000"
            ordinary_file.write_text("do not delete", encoding="utf-8")
            locked = self._make_retention_run(
                root, "20260710T000000Z_000", ended_at=old
            )
            lock_path = locked / "metadata/.active.lock"
            with lock_path.open("a", encoding="utf-8") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                removed = helper.prune_development_runs(root, now=now)

            self.assertEqual({path.name for path in removed}, {ordinary[0].name})
            for run in (
                active,
                formal,
                protected,
                invalid,
                linked,
                locked,
                ordinary_file,
            ):
                self.assertTrue(run.exists() or run.is_symlink())
            self.assertTrue(outside.is_dir())

    def test_external_root_retention_requires_explicit_opt_in(self):
        helper = self._load_launch("_includes/run_directory.py")
        old = datetime.now(timezone.utc) - timedelta(days=30)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            old_runs = [
                self._make_retention_run(
                    root, f"2026080{day}T000000Z_000", ended_at=old
                )
                for day in (1, 2, 3, 4)
            ]
            with mock.patch.dict(
                os.environ,
                {"IAP_RUN_ROOT": str(root)},
                clear=False,
            ), mock.patch.object(
                helper, "_new_run_id", return_value="20260929T120000Z_000"
            ):
                os.environ.pop("IAP_RETENTION_ENABLED", None)
                helper.resolve_run_directory(entrypoint="glio")
                self.assertTrue(all(run.is_dir() for run in old_runs))

                os.environ["IAP_RETENTION_ENABLED"] = "1"
                helper.resolve_run_directory(entrypoint="glio")
                self.assertFalse(old_runs[0].exists())
                self.assertTrue(all(run.is_dir() for run in old_runs[1:]))

    def test_retention_failure_does_not_block_run_allocation(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ,
            {"IAP_RUN_ROOT": temporary, "IAP_RETENTION_ENABLED": "1"},
        ), mock.patch.object(
            helper, "prune_development_runs", side_effect=OSError("lock failed")
        ):
            with self.assertWarnsRegex(UserWarning, "retention skipped"):
                run = helper.resolve_run_directory(entrypoint="glio")
            self.assertTrue(run.is_dir())

    def test_finalize_run_updates_lifecycle_and_safety_outcome_atomically(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ):
            run = helper.resolve_run_directory(entrypoint="iap_sim")
            helper.finalize_run(
                run, lifecycle="completed", safety_outcome="hold"
            )
            manifest = json.loads(
                (run / "metadata/run_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["lifecycle"], "completed")
            self.assertEqual(manifest["safety_outcome"], "hold")
            self.assertIsNotNone(manifest["ended_at_utc"])
            with self.assertRaisesRegex(ValueError, "lifecycle"):
                helper.finalize_run(run, lifecycle="holding")

    def test_shutdown_mapping_and_config_snapshot_index(self):
        helper = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ):
            interrupted = helper.resolve_run_directory(entrypoint="glio")
            config_dir = interrupted / "metadata/config/iap"
            helper.register_config_snapshot(interrupted, config_dir)
            helper.finalize_run_from_shutdown(
                interrupted,
                SimpleNamespace(reason="ctrl-c (SIGINT)", due_to_sigint=True),
            )
            manifest = json.loads(
                (interrupted / "metadata/run_manifest.json").read_text()
            )
            self.assertEqual(manifest["lifecycle"], "interrupted")
            self.assertEqual(manifest["config_snapshots"], ["metadata/config/iap"])

            failed = helper.resolve_run_directory(entrypoint="iap_sim")
            helper.finalize_run_from_shutdown(
                failed,
                SimpleNamespace(
                    reason="Caught exception in launch (see debug for traceback): boom",
                    due_to_sigint=False,
                ),
            )
            manifest = json.loads((failed / "metadata/run_manifest.json").read_text())
            self.assertEqual(manifest["lifecycle"], "failed")

    def test_glio_omitted_output_uses_automatic_run_directory(self):
        glio = self._load_launch("glio.launch.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(
            os.environ, {"IAP_RUN_ROOT": temporary}
        ):
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "config_path": str(REPO / "config/profiles/glio"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "use_sim_time": "false",
                }
            )
            self.assertGreaterEqual(len(glio._setup(context)), 2)
            runs = [
                path
                for path in Path(temporary).iterdir()
                if path.is_dir() and not path.is_symlink() and path.name[0].isdigit()
            ]
            self.assertEqual(len(runs), 1)
            self.assertTrue((runs[0] / "metadata/config/iap/config.json").is_file())

    def test_each_canonical_entrypoint_constructs_a_graph(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            environment = mock.patch.dict(
                os.environ, {"IAP_RUN_ROOT": str(root / "runs")}
            )
            environment.start()
            self.addCleanup(environment.stop)

            glio = self._load_launch("glio.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "config_path": str(REPO / "config/profiles/glio"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "use_sim_time": "false",
                }
            )
            self.assertEqual(len(glio._setup(context)), 4)

            integrity = self._load_launch("glio_integrity.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "config_path": str(REPO / "config/profiles/glio_integrity"),
                    "imu_topic": "/imu",
                    "points_topic": "/points",
                    "use_sim_time": "false",
                    "integrity_profile": "fused",
                    "forbid_sim_extensions": "true",
                    "runtime_contract": "glio_integrity",
                }
            )
            self.assertEqual(len(integrity._setup(context)), 4)

            simulation = self._load_launch("iap_sim.launch.py")
            context = LaunchContext()
            context.launch_configurations.update(
                {
                    "scenario": "icra_dense_forest_four_fork_v2",
                    "start_rviz": "false",
                    "planner_start_delay_s": "0",
                    "run_duration_s": "0",
                }
            )
            with mock.patch.object(
                simulation, "get_package_share_directory", return_value=str(REPO)
            ):
                self.assertEqual(len(simulation._setup(context)), 4)

            flight = self._load_launch("iap_flight.launch.py")
            with self.assertRaisesRegex(RuntimeError, "stages 4/5"):
                flight._setup(LaunchContext())


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
