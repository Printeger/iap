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

import yaml
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

    def test_historical_input_freezes_nav_and_owns_one_clock_policy(self):
        canonical = self._load_launch("iap_sim.launch.py")
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        environment = self._load_launch("_includes/simulation_environment.launch.py")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            nav = root/"mixed.rnx"
            nav.write_bytes(b"unit NAV identity; decoder qualification is independently tested")
            self.assertEqual(runtime.historical_gnss_parameters("") , {})
            with self.assertRaisesRegex(ValueError, "absolute NAV"):
                runtime.historical_gnss_parameters("relative.rnx")
            with self.assertRaisesRegex(ValueError, "absolute NAV"):
                runtime.historical_gnss_parameters(str(root/"missing.rnx"))
            context = LaunchContext()
            context.launch_configurations.update(scenario="icra_dense_forest_four_fork_v2",
                rinex_nav_file=str(nav), start_rviz="false", start_grid_map_visualizer="false",
                planner_start_delay_s="0", run_duration_s="0")
            with mock.patch.dict(os.environ, {"IAP_RUN_ROOT": str(root/"runs")}), mock.patch.object(
                    canonical, "get_package_share_directory", return_value=str(REPO)):
                canonical._setup(context)
            run = next((root/"runs").glob("20*"))
            info = json.loads((run/"metadata/manifests/full_stack.json").read_text())
            frozen = Path(info["gnss_input"]["nav_file"])
            self.assertTrue(frozen.is_relative_to(run))
            self.assertEqual(frozen.read_bytes(), nav.read_bytes())
            self.assertEqual(info["clock_contract"], "historical_clock_2022-07-06T12:00:00Z")
            self.assertEqual(info["gnss_input"]["constellations"], ["GPS", "BDS"])
            self.assertFalse(info["gnss_input"]["formal_advisory_qualified"])
            context.launch_configurations.update(output_dir=str(run), rinex_nav_file=str(frozen))
            nodes = []
            original = environment.Node
            def capture(**kwargs):
                nodes.append(kwargs)
                return original(**kwargs)
            with mock.patch.object(environment, "get_package_share_directory", return_value=str(REPO)), \
                    mock.patch.object(environment, "Node", side_effect=capture):
                actions = environment._setup(context)
            actions[0].execute(context)
            self.assertIn(("use_sim_time", True), context.launch_configurations["global_params"])
            producer = next(n for n in nodes if n.get("executable")=="so3_quadrotor_simulator")
            params = {k:v for item in producer["parameters"] for k,v in item.items()}
            self.assertFalse(params["use_sim_time"])
            self.assertTrue(params["sim_time/enable"])
            self.assertEqual(params["sim_time/start_utc"], "2022-07-06T12:00:00Z")
            gnss = next(n for n in nodes if n.get("executable")=="gnss_sim_node")["parameters"][0]
            self.assertEqual(gnss["ephemeris_source"], "rinex")
            self.assertEqual(gnss["enabled_constellations_csv"], "GPS,BDS")
            self.assertFalse(gnss["fallback_to_synthetic_on_rinex_error"])

    def test_historical_input_loss_fails_both_owners_and_preserves_first_failure(self):
        environment = self._load_launch("_includes/simulation_environment.launch.py")
        runs = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
            for owner in ["launch", "driver"]:
                run = runs.resolve_run_directory(entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2")
                environment._historical_process_exit(SimpleNamespace(returncode=2),
                    SimpleNamespace(is_shutdown=False), run, "gnss_sim")
                first = (run/"metadata/manifests/historical_input_failure.json").read_bytes()
                environment._historical_process_exit(SimpleNamespace(returncode=0),
                    SimpleNamespace(is_shutdown=False), run, "historical_clock_owner")
                self.assertEqual((run/"metadata/manifests/historical_input_failure.json").read_bytes(), first)
                if owner == "launch":
                    runs.finalize_run_from_shutdown(run, SimpleNamespace(reason="historical_input_failure", due_to_sigint=False))
                else:
                    self.assertEqual(runs.finalize_run(run, lifecycle="completed"), "failed")
                self.assertEqual(json.loads((run/"metadata/run_manifest.json").read_text())["lifecycle"], "failed")
                self.assertEqual(environment._historical_process_exit(SimpleNamespace(returncode=-2),
                    SimpleNamespace(is_shutdown=True), run, "gnss_sim"), [])

    def test_recorder_clock_policy_comes_from_run_owner(self):
        runs = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            directory = run/"metadata/manifests"
            directory.mkdir(parents=True)
            with self.assertRaisesRegex(RuntimeError, "clock identity unavailable"):
                runs.canonical_run_uses_sim_time(run, 0)
            for contract, expected in [("system_clock_for_ros_and_simulated_sensor_stamps", False),
                                       ("historical_clock_2022-07-06T12:00:00Z", True)]:
                (directory/"full_stack.json").write_text(json.dumps({"clock_contract": contract}))
                self.assertIs(runs.canonical_run_uses_sim_time(run), expected)
            (directory/"full_stack.json").write_text(json.dumps({"clock_contract": "unknown"}))
            with self.assertRaisesRegex(ValueError, "unsupported canonical clock"):
                runs.canonical_run_uses_sim_time(run)

    def test_stage1_parameters_have_one_map_and_no_retired_planner_controls(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        catalog = json.loads((REPO / "config/scenarios/catalog.json").read_text())
        for entry in catalog.values():
            params = runtime.planner_parameters(entry)
            self.assertEqual(params["grid_map/resolution"], 0.1)
            self.assertTrue(params["grid_map/registered_lidar_window_enabled"])
            self.assertEqual(params["grid_map/frame_id"], "map")
            self.assertEqual(params["grid_map/visualization_period_s"], 1.0)
            self.assertFalse(any(key.startswith("risk_viz/") for key in params))
            display = runtime.visualizer_parameters()
            self.assertEqual(display["risk_viz/metric"], "hpl")
            self.assertLess(display["risk_viz/hpl_max_m"], 1.0)
            self.assertEqual(display["risk_viz/vpl_min_m"], 0.20)
            self.assertEqual(display["risk_viz/vpl_max_m"], 0.55)
            self.assertEqual(display["risk_viz/surface_lifetime_s"], 60.0)
            self.assertEqual(display["risk_viz/surface_snapshot_step_m"], 4.0)
            self.assertFalse(params["planning/capture_failure_map"])
            self.assertFalse(params["risk/use_posterior_prior"])
            self.assertFalse(any(key.startswith(("p0.", "p1.", "p2.", "p3.", "p4.", "p5.")) for key in params))
            self.assertNotIn("manager/use_distinctive_trajs", params)
            for i, axis in enumerate("xyz"):
                self.assertEqual(params[f"grid_map/map_size_{axis}"], entry["map_size"][i])
        self.assertTrue(runtime.planner_parameters(
            catalog["icra_dense_forest_four_fork_v2"], True)[
                "planning/capture_failure_map"])

    def test_advisory_prior_explicit_enable_changes_only_shared_input_parameter(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        scene = json.loads((REPO / "config/scenarios/catalog.json").read_text())["icra_dense_forest_four_fork_v2"]
        off = runtime.planner_parameters(scene)
        on = runtime.planner_parameters(scene, advisory_posterior_prior=True)
        self.assertEqual([key for key in off if off[key] != on[key]], ["risk/use_posterior_prior"])
        self.assertTrue(on["risk/use_posterior_prior"])
        self.assertNotIn("risk/use_posterior_prior", runtime.visualizer_parameters())

    def test_explicit_calibration_is_hashed_and_cannot_change_planning_checks(self):
        runtime=self._load_launch("_includes/full_stack_runtime.py")
        import hashlib
        scene=json.loads((REPO/"config/scenarios/catalog.json").read_text())["icra_dense_forest_four_fork_v2"]
        base=runtime.planner_parameters(scene)
        value={"parameters":{"risk/gnss_noise_scale":2.,"risk/lidar_noise_scale":3.,
                             "risk/K_H_adv":4.,"risk/K_V_adv":4.},"identity":"LIVE_CALIBRATION_CANDIDATE","stage":"conversion","scene":"icra_dense_forest_four_fork_v2",
               "calibration_evidence":[{"run_id":"unit-fixture"}],
               "contract":{"map_seed":41021,"route_sha256":"route","coordinates_sha256":"frame",
                           "degradation_schedule_sha256":"schedule"}}
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"parameters.json"
            value["sha256"]=hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
            path.write_text(json.dumps(value))
            loaded=runtime.planner_parameters(scene,advisory_calibration=str(path))
            for key,old in base.items(): self.assertEqual(loaded[key],old)
            self.assertEqual(loaded["risk/gnss_noise_scale"],2.)
            value["parameters"]["planning/body_radius_m"]=0.
            unsigned={k:v for k,v in value.items() if k!="sha256"}
            value["sha256"]=hashlib.sha256(json.dumps(unsigned,sort_keys=True,allow_nan=False).encode()).hexdigest()
            path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,"only positive"): runtime.load_advisory_calibration(str(path))
            value["sha256"]="altered";path.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,"checksum"):runtime.load_advisory_calibration(str(path))

    def test_guidance_switch_changes_only_planning_preference(self):
        runtime=self._load_launch("_includes/full_stack_runtime.py")
        scene=json.loads((REPO/"config/scenarios/catalog.json").read_text())["icra_dense_forest_four_fork_v2"]
        on=runtime.planner_parameters(scene);off=runtime.planner_parameters(scene,advisory_guidance=False)
        self.assertEqual([k for k in on if on[k]!=off[k]],["planning/advisory_guidance_enabled"])
        self.assertTrue(on["planning/advisory_guidance_enabled"])
        self.assertFalse(off["risk/use_posterior_prior"])

    def test_fixed_trial_injects_waypoints_observation_seed_and_preserves_map(self):
        import hashlib
        runtime=self._load_launch("_includes/full_stack_runtime.py")
        environment=self._load_launch("_includes/simulation_environment.launch.py")
        canonical=self._load_launch("iap_sim.launch.py")
        scene=json.loads((REPO/"config/scenarios/catalog.json").read_text())["icra_dense_forest_four_fork_v2"]
        digest=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            # Contract fixture only; this never starts processes or supplies live evidence.
            route={"waypoints":[[-17.,0.,1.5],[-16.,1.,1.5]],"speed_mps":scene["max_velocity_mps"]}
            frame=root/"coordinates.json";frame.write_text(json.dumps({"verified":True,"provenance":"unit fixture","alignment_policy":"known_fixed_transform",
                "prediction_frame":"map","prediction_body":"body","truth_frame":"world","truth_body":"body",
                "T_truth_map":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                "T_truthbody_predictionbody":[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                "world_to_enu_verified":True,"body_extrinsics_verified":True,"time_verified":True,
                "time_contract":{"clock":"ros_system_time","reference":"saved_pose_stamp",
                                 "max_truth_bracket_dt_s":.05,"max_reference_pose_dt_s":.05}}))
            proof=root/"route_proof.json";proof.write_text(json.dumps({"identity":"REAL_REPLAY","physical_valid":True,"route_sha256":digest(route),"prediction_input_identity":"unit fixture"}))
            schedule=runtime.advisory_observation_schedule()
            trial={"schema":"iap_advisory_validation_trial_v1","scene":"icra_dense_forest_four_fork_v2",
                   "map_seed":41021,"phase":"calibration","condition":"gnss_degraded","seed":1101,
                   "reference_route":route,"route_sha256":digest(route),"coordinates":str(frame),
                   "coordinates_sha256":hashlib.sha256(frame.read_bytes()).hexdigest(),"physical_route_evidence":str(proof),
                   "physical_route_evidence_sha256":hashlib.sha256(proof.read_bytes()).hexdigest(),
                   "degradation_schedule":schedule,"degradation_schedule_sha256":digest(schedule)}
            path=root/"trial.json";path.write_text(json.dumps(trial))
            base=runtime.planner_parameters(scene,advisory_guidance=False)
            params=runtime.planner_parameters(scene,advisory_guidance=False,advisory_trial=str(path))
            self.assertEqual(params["fsm/waypoint_num"],2)
            self.assertEqual(params["fsm/waypoint1_y"],1.)
            for key,value in base.items():
                if not key.startswith("fsm/waypoint"):self.assertEqual(params[key],value,key)
            with self.assertRaisesRegex(ValueError,"calibration guidance OFF"):
                runtime.planner_parameters(scene,advisory_trial=str(path))
            context=LaunchContext();context.launch_configurations.update(scenario="icra_dense_forest_four_fork_v2",
                output_dir=str(root/"environment"),advisory_trial=str(path))
            nodes=[];original=environment.Node
            def capture(**kwargs):nodes.append(kwargs);return original(**kwargs)
            with mock.patch.object(environment,"get_package_share_directory",return_value=str(REPO)),mock.patch.object(environment,"Node",side_effect=capture):
                environment._setup(context)
            gnss=next(n for n in nodes if n.get("executable")=="gnss_sim_node")["parameters"][0]
            self.assertEqual(gnss["random_seed"],1101);self.assertEqual(gnss["pseudorange_noise_std_m"],5.)
            self.assertEqual(environment._map_parameters(scene["map_profile"],scene["initial"],scene["goal"])["random_seed"],41021)
            with mock.patch.dict(os.environ,{"IAP_RUN_ROOT":str(root/"runs")}),mock.patch.object(canonical,"get_package_share_directory",return_value=str(REPO)):
                context.launch_configurations.update(advisory_guidance="false",start_rviz="false",start_grid_map_visualizer="false",run_duration_s="0",planner_start_delay_s="0")
                actions=canonical._setup(context)
            run=next((root/"runs").glob("20*"))
            primary=json.loads((run/"metadata/run_manifest.json").read_text())
            self.assertEqual(primary["validation_trial"]["seed"],1101)
            self.assertIn("metadata/config/advisory_trial.json",primary["config_snapshots"])
            self.assertTrue((run/"metadata/config/coordinates.json").exists())
            coordinates=json.loads(frame.read_text());coordinates.pop("T_truth_map");frame.write_text(json.dumps(coordinates))
            trial["coordinates_sha256"]=hashlib.sha256(frame.read_bytes()).hexdigest();path.write_text(json.dumps(trial))
            with self.assertRaisesRegex(ValueError,"fixed transform missing"):runtime.load_advisory_trial(str(path),scene)
            trial["seed"]=2101;path.write_text(json.dumps(trial))
            with self.assertRaisesRegex(ValueError,"predeclared split"):runtime.load_advisory_trial(str(path),scene)

    def test_stage1_graph_materializes_same_registered_lattice_and_preserves_artifacts(self):
        runtime = self._load_launch("_includes/full_stack_runtime.py")
        runs = self._load_launch("_includes/run_directory.py")
        catalog = json.loads((REPO / "config/scenarios/catalog.json").read_text())
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
            run = runs.resolve_run_directory(entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2")
            with mock.patch.dict(os.environ, {"IAP_RUN_DIR": str(run)}), mock.patch.object(runtime, "get_package_share_directory", return_value=str(REPO)):
                context = LaunchContext()
                context.launch_configurations.update({"scenario": "icra_dense_forest_four_fork_v2", "start_rviz": "false",
                    "start_grid_map_visualizer": "true",
                    "planner_start_delay_s": "0", "run_duration_s": "0", "p0.enable_risk_grid": "true"})
                actions = runtime._setup(context)
            from launch_ros.actions import SetParameter
            self.assertEqual(len([a for a in actions if not isinstance(a, SetParameter)]), 5)
            context.launch_configurations["start_grid_map_visualizer"] = "false"
            with mock.patch.dict(os.environ, {"IAP_RUN_DIR": str(run)}), mock.patch.object(runtime, "get_package_share_directory", return_value=str(REPO)):
                self.assertEqual(len([a for a in runtime._setup(context) if not isinstance(a, SetParameter)]), 4)
            ros = json.loads((run / "metadata/config/iap/config_ros.json").read_text())["glim_ros"]
            local = ros["planner_local_map"]
            params = runtime.planner_parameters(catalog["icra_dense_forest_four_fork_v2"])
            self.assertEqual(local["planning_lattice_resolution_m"],params["grid_map/resolution"])
            self.assertEqual(local["frame_contract_id"],params["grid_map/registered_frame_contract_id"])
            self.assertEqual(local["planning_lattice_extent_m"],catalog["icra_dense_forest_four_fork_v2"]["map_size"])
            self.assertTrue(local["publish_current_hits_map"])
            self.assertEqual(ros["acc_scale"], 1.0)
            odometry = json.loads((run / "metadata/config/iap/config_odometry.json").read_text())
            self.assertEqual(odometry["odometry_estimation"]["initialization_mode"], "NAIVE")
            self.assertFalse(ros["sim"]["align_planner_odom_to_truth"])
            self.assertEqual(ros["sim"]["static_planner_translation_m"],local["static_planner_translation_m"])
            self.assertIn("libplanner_local_map_extension.so",ros["extension_modules"])
            manifest=json.loads((run / "metadata/run_manifest.json").read_text())
            self.assertIn("metadata/config/iap",manifest["config_snapshots"])

    def test_installed_full_stack_connects_command_feedback_to_the_server(self):
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("iap"))
        path = share / "launch/_includes/full_stack_runtime.py"
        spec = importlib.util.spec_from_file_location("installed_full_stack", path)
        runtime = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(runtime)
        runs = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
                run = runs.resolve_run_directory(entrypoint="feedback_contract")
            context = LaunchContext()
            context.launch_configurations.update({
                "scenario": "icra_dense_forest_four_fork_v2",
                "start_rviz": "false", "start_grid_map_visualizer": "false",
                "planner_start_delay_s": "0", "run_duration_s": "0"})
            with mock.patch.dict(os.environ, {"IAP_RUN_DIR": str(run)}):
                with mock.patch.object(runtime, "Node", wraps=runtime.Node) as nodes:
                    runtime._setup(context)
            declarations = {call.kwargs["executable"]: call.kwargs
                            for call in nodes.call_args_list}
            planner = dict(declarations["ego_planner_node"]["remappings"])
            server = dict(declarations["traj_server"]["remappings"])
            self.assertEqual(planner.get("/position_cmd"),
                             server["/position_cmd"])
            self.assertEqual(planner["planning/bspline"],
                             server["planning/bspline"])

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
                      "/grid_map/risk_surface", "/iap/local_map/current_hits_map",
                      "/grid_map/risk_legend", "/grid_map/glio_path",
                      "/planning/trajectory_curve"):
            self.assertIn(topic, rviz)
        displays = yaml.safe_load(rviz)["Visualization Manager"]["Displays"]
        current = next(display for display in displays
                       if display.get("Topic", {}).get("Value") == "/iap/local_map/current_hits_map")
        self.assertEqual(current["Topic"]["Reliability Policy"], "Best Effort")
        self.assertEqual(current["Decay Time"], 0.2)
        self.assertNotIn("/grid_map/risk_history", rviz)
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

    def test_canonical_sim_adopts_active_owner_without_allocating_second_run(self):
        canonical = self._load_launch("iap_sim.launch.py")
        runs = self._load_launch("_includes/run_directory.py")
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
            run = runs.resolve_run_directory(entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2")
            context = LaunchContext()
            context.launch_configurations.update(run_dir=str(run), scenario="icra_dense_forest_four_fork_v2",
                start_rviz="false", start_grid_map_visualizer="true", run_duration_s="180", planner_start_delay_s="10")
            with mock.patch.object(canonical, "get_package_share_directory", return_value=str(REPO)):
                canonical._setup(context)
            self.assertEqual(len(list(Path(temporary).glob("20*"))), 1)
            primary_path = run / "metadata/run_manifest.json"
            primary = json.loads(primary_path.read_text())
            primary["lifecycle"] = "completed"
            primary_path.write_text(json.dumps(primary))
            with mock.patch.object(canonical, "get_package_share_directory", return_value=str(REPO)):
                with self.assertRaisesRegex(ValueError, "active resolver"):
                    canonical._setup(context)

    def test_driver_owns_shutdown_and_finalizes_startup_failure(self):
        canonical = self._load_launch("iap_sim.launch.py")
        runs = self._load_launch("_includes/run_directory.py")
        from launch.actions import RegisterEventHandler
        with tempfile.TemporaryDirectory() as temporary, mock.patch.dict(os.environ, {"IAP_RUN_ROOT": temporary}):
            run = runs.resolve_run_directory(entrypoint="iap_sim", scenario="icra_dense_forest_four_fork_v2")
            context = LaunchContext()
            context.launch_configurations.update(run_dir=str(run), run_lifecycle_owner="driver",
                scenario="icra_dense_forest_four_fork_v2", start_rviz="false", start_grid_map_visualizer="true",
                run_duration_s="30", planner_start_delay_s="10")
            with mock.patch.object(canonical, "get_package_share_directory", return_value=str(REPO)):
                actions = canonical._setup(context)
            self.assertFalse(any(isinstance(a, RegisterEventHandler) for a in actions))
            spec = importlib.util.spec_from_file_location("curve_live", REPO / "scripts/dev_planner/run_curve_channel_live.py")
            driver = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(driver)
            with mock.patch.object(driver, "source_identity", return_value={"revision": "test", "dirty": ""}), \
                    mock.patch.object(driver, "resolve_run_directory", return_value=run), \
                    mock.patch("ament_index_python.packages.get_package_share_directory", side_effect=RuntimeError("missing installation")), \
                    mock.patch.object(sys, "argv", ["curve_live", "--duration", "30"]):
                with self.assertRaisesRegex(RuntimeError, "missing installation"):
                    driver.main()
            primary = json.loads((run / "metadata/run_manifest.json").read_text())
            self.assertEqual(primary["lifecycle"], "failed")
            self.assertIn("metadata/manifests/forest_process_result.json", primary["subordinate_manifests"])
            result = json.loads((run / "metadata/manifests/forest_process_result.json").read_text())
            self.assertIn("missing installation", result["error"])

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
                    "start_grid_map_visualizer": "true",
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
                    "start_grid_map_visualizer": "true",
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
