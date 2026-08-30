import argparse
import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock


REPO = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO / "scripts" / "dev_planner" / "run_icra_interface_integration.py"
SPEC = importlib.util.spec_from_file_location("icra_interface_integration", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def healthy(generation, received):
    return {
        "kind": "p0_health",
        "receive_steady_s": received,
        "payload": {
            "ready": True,
            "stale": False,
            "reason": "ok",
            "generation_id": generation,
        },
    }


def selected_decision():
    return {
        "status": "RISK_SELECTED",
        "selection_applied": "1",
        "planning_attempt_id": "7",
        "collision_segment_id": "9",
        "request_hash": "request",
        "snapshot_generation_id": "5",
        "snapshot_config_hash": "config",
        "source_identity_hash": "sources",
        "occupancy_epoch": "3",
        "geometry_id": "geometry",
        "occupancy_stamp_s": "12.5",
        "original_hash": "original",
        "risk_hash": "risk",
        "selected_hash": "risk",
        "original_sample_count": "10",
        "original_valid_count": "10",
        "original_unknown_count": "0",
        "original_stale_count": "0",
        "original_non_finite_count": "0",
        "risk_sample_count": "10",
        "risk_valid_count": "10",
        "risk_unknown_count": "0",
        "risk_stale_count": "0",
        "risk_non_finite_count": "0",
    }


def lineage_for(decision, trajectory_id, start_ns):
    common = {
        "schema_version": "p4_v2_end_to_end_lineage_v2",
        "planning_attempt_id": decision["planning_attempt_id"],
        "collision_segment_id": decision["collision_segment_id"],
        "request_hash": decision["request_hash"],
        "snapshot_generation_id": decision["snapshot_generation_id"],
        "snapshot_config_hash": decision["snapshot_config_hash"],
        "source_identity_hash": decision["source_identity_hash"],
        "occupancy_epoch": decision["occupancy_epoch"],
        "geometry_id": decision["geometry_id"],
        "occupancy_stamp_s": decision["occupancy_stamp_s"],
        "original_guide_hash": decision["original_hash"],
        "risk_guide_hash": decision["risk_hash"],
        "selected_guide_hash": decision["selected_hash"],
        "trajectory_id": str(trajectory_id),
        "trajectory_start_ns": str(start_ns),
        "control_points_hash": "points",
        "final_bspline_identity": "bspline",
        "selection_applied": "1",
    }
    return [
        {**common, "stage": stage, "stamp_s": str(10.0 + index)}
        for index, stage in enumerate((
            "final_bspline_before_p5",
            "p5_final_pass_before_publish",
            "normal_publish_authorized",
        ))
    ]


class TestStageContracts(unittest.TestCase):
    def test_icra_rviz_uses_sensor_qos_for_predicted_pl_cloud(self):
        rviz = (REPO / "config/sim_demo11/test_icra.rviz").read_text()
        predicted_pl = rviz.split("Name: Predicted PL Cloud", 1)[1]
        predicted_pl = predicted_pl.split("- Class:", 1)[0]
        self.assertIn("Reliability Policy: Best Effort", predicted_pl)
        self.assertIn("Style: Points", predicted_pl)
        self.assertIn("Size (Pixels): 3", predicted_pl)

    def test_stage_switches_keep_forbidden_layers_off(self):
        for stage in MODULE.STAGE_ORDER:
            spec = MODULE.STAGES[stage]
            self.assertEqual(spec.launch_args["planner_enable_p1"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p2"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p3_local"], "false")
            self.assertEqual(spec.launch_args["planner_enable_p3_global"], "false")

        self.assertEqual(MODULE.STAGES["estimator"].duration_s, 20.0)
        self.assertEqual(MODULE.STAGES["p0"].duration_s, 35.0)
        self.assertEqual(MODULE.STAGES["p4"].duration_s, 45.0)
        self.assertEqual(MODULE.STAGES["p5-final"].duration_s, 60.0)
        self.assertEqual(MODULE.STAGES["full"].duration_s, 75.0)

        expected = {
            "estimator": ("false", "false", "false", "false"),
            "p0": ("true", "false", "false", "false"),
            "p4": ("true", "true", "false", "false"),
            "p5-final": ("true", "true", "true", "false"),
            "full": ("true", "true", "true", "true"),
            "shutdown": ("true", "true", "true", "true"),
        }
        keys = (
            "start_planner", "planner_enable_p4",
            "planner_enable_p5_final", "planner_enable_p5_runtime",
        )
        for stage, values in expected.items():
            self.assertEqual(
                tuple(MODULE.STAGES[stage].launch_args[key] for key in keys),
                values,
            )
            self.assertEqual(
                MODULE.STAGES[stage].launch_args["odometry_acc_scale"], "1.0")
            self.assertEqual(
                MODULE.STAGES[stage].launch_args[
                    "odometry_initialization_mode"], "NAIVE")

    def test_final_pass_is_reserved_for_full_plus_shutdown(self):
        self.assertEqual(MODULE._successful_session_result(("full",)),
                         "STAGE_PASS")
        self.assertEqual(MODULE._successful_session_result(("shutdown",)),
                         "STAGE_PASS")
        self.assertEqual(MODULE._successful_session_result(
            MODULE.STAGE_ORDER), "PASS")

    def test_forest_scenario_switch_preserves_icra072_default(self):
        self.assertEqual(MODULE.DEFAULT_SCENARIO,
                         "icra072_p4_selection_trigger_v1")
        default_args = MODULE.stage_launch_args("full", MODULE.DEFAULT_SCENARIO)
        self.assertEqual(default_args["scenario"], MODULE.DEFAULT_SCENARIO)

        forest_args = MODULE.stage_launch_args(
            "full", MODULE.FOREST_SCENARIO, forest_variant="risk")
        self.assertEqual(forest_args["scenario"], MODULE.FOREST_SCENARIO)
        self.assertEqual(forest_args["planner_enable_p4"], "true")
        self.assertEqual(forest_args["planner_enable_p5_final"], "true")
        self.assertEqual(forest_args["planner_enable_p5_runtime"], "true")

        baseline = MODULE.stage_launch_args(
            "full", MODULE.FOREST_SCENARIO, forest_variant="baseline")
        self.assertEqual(baseline["planner_enable_p4"], "false")
        self.assertEqual(baseline["planner_enable_p5_final"], "false")
        self.assertEqual(baseline["planner_enable_p5_runtime"], "false")
        self.assertEqual(baseline["p0.enable_risk_grid"], "true")

    def test_forest_scene_contract_is_expanded_and_fingerprinted(self):
        contract = MODULE.forest_scene_contract()
        self.assertEqual(contract["schema_version"],
                         "icra_dense_forest_four_fork_v2")
        self.assertEqual(contract["forest_seed"], 41021)
        self.assertEqual(contract["risk_seed"], 21)
        self.assertEqual(contract["low_risk_y_signs"], [-1, 1, -1, 1])
        self.assertEqual(contract["gnss"]["enabled_constellations"],
                         ["GPS", "GAL", "GLO"])
        self.assertEqual(contract["planner_executor_thread_count"], 6)
        self.assertTrue(contract["p0_conservative_max_with_gnss"])
        self.assertEqual([fork["x_min_m"] for fork in contract["forks"]],
                         [-16.0, -8.0, 0.0, 8.0])
        self.assertRegex(contract["fingerprint"], r"^sha256:[0-9a-f]{64}$")

        legacy = MODULE.forest_scene_contract(MODULE.FOREST_V1_SCENARIO)
        self.assertEqual(legacy["schema_version"],
                         "icra_dense_forest_four_fork_v1")
        self.assertFalse(legacy["online_mapping"]["enabled"])

    def test_forest_manifest_binds_effective_launch_contract(self):
        expected = MODULE.forest_scene_contract()
        scene_map = {
            "layout_mode": "forked_s_forest_v2",
            "map_size_m": [42.0, 22.0, 8.0],
            "forest_size_m": [40.0, 20.0],
            "forest_seed": 41021,
            "fork_risk_seed": 21,
            "fork_count": 4,
            "fork_x_min_m": -16.0,
            "fork_length_m": 8.0,
            "low_risk_amplitude_m": 4.0,
            "high_risk_amplitude_m": 2.8,
            "corridor_width_m": 2.4,
            "junction_clearance_radius_m": 2.0,
            "start_canopy_clearance_radius_m": 5.0,
            "flight_clearance_z_m": 2.8,
            "side_boundary_tree_spacing_m": 0.28,
            "expanded_low_risk_sides": [
                "right", "left", "right", "left"],
        }
        contract = {
            "scene_map": scene_map,
            "geometry": {
                "start_m": expected["start_xyz_m"],
                "goal_m": expected["goal_xyz_m"],
            },
            "gnss": {
                "ephemeris_source": "rinex",
                "enabled_constellations": "GPS,GAL,GLO",
                "map_occlusion": True,
                "skymask": False,
                "nlos": True,
                "multipath": True,
            },
            "p0_prediction": {
                "online_mapping_mode": True,
                "fit_grid_to_map_cloud": False,
                "map_topic": "",
                "origin_m": [-21.0, -11.0, 0.0],
                "extent_m": [42.0, 22.0, 8.0],
                "risk_resolution_m": 0.5,
                "ego_resolution_m": 0.1,
                "ego_origin_m": [-21.0, -11.0, 0.0],
                "unknown_as_occupied": True,
                "provider_cost_source": "pre_conservative_fim_ratio",
                "require_safety_ratio_below_one_for_cost": True,
                "alert_limit_policy_id": "fixed_hal10_val20_v1",
                "alert_limit_h_m": 10.0,
                "alert_limit_v_m": 20.0,
                "skip_occupied_voxels": True,
                "use_current_integrity_prior": True,
                "conservative_max_with_gnss": True,
                "executor_thread_count": 6,
            },
        }
        with tempfile.TemporaryDirectory() as raw:
            run_root = Path(raw)
            manifest = run_root / "exports" / "run" / "test_planner_manifest.json"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({
                "scenario_contract": contract,
                "scenario_fingerprint": "sha256:effective",
            }))
            evidence = MODULE.forest_manifest_evidence(run_root)
            self.assertTrue(evidence["matches_expected"])
            self.assertEqual(evidence["scenario_fingerprint"],
                             "sha256:effective")
            contract["scene_map"]["corridor_width_m"] = 3.0
            manifest.write_text(json.dumps({
                "scenario_contract": contract,
                "scenario_fingerprint": "sha256:drifted",
            }))
            evidence = MODULE.forest_manifest_evidence(run_root)
            self.assertFalse(evidence["matches_expected"])
            self.assertEqual(evidence["failures"],
                             ["forest_contract_mismatch"])
            self.assertIn("scene_map.corridor_width_m",
                          evidence["mismatches"])

    def test_forest_full_runs_for_ninety_seconds(self):
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.FOREST_SCENARIO, "risk"), 90.0)
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.FOREST_SCENARIO, "baseline"), 90.0)
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.DEFAULT_SCENARIO), 75.0)

    def test_live_truth_audit_rejects_planner_world_subscription(self):
        with mock.patch.object(
                MODULE, "_node_names",
                return_value={"/drone_0_ego_planner_node"}), mock.patch.object(
                    MODULE, "_node_subscriptions",
                    return_value=(["/sim/drone_0/lidar",
                                   "/map_generator/global_cloud"], "")):
            evidence = MODULE.audit_planner_truth_isolation({}, timeout_s=0.1)
        self.assertFalse(evidence["pass"])
        self.assertEqual(evidence["forbidden_subscriptions"],
                         ["/map_generator/global_cloud"])
        self.assertIn("planner_truth_subscription_detected",
                      evidence["failures"])


class TestRunnerLifecycle(unittest.TestCase):
    @staticmethod
    def runner_args(root, *, through=None, rviz=False):
        install_root = root / "install"
        install_root.mkdir()
        (install_root / "setup.bash").write_text("")
        return argparse.Namespace(
            install_root=install_root,
            results_root=root / "results",
            stage="full",
            through=through,
            repetitions=1,
            rviz=rviz,
        )

    def test_cli_reports_session_stage_and_log_before_running(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root, rviz=True)
            output = io.StringIO()
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        return_value=MODULE._result([])), contextlib.redirect_stdout(output):
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 0)
            stdout = output.getvalue()
            self.assertIn("[icra] SESSION ", stdout)
            self.assertIn(
                "[icra] START stage=full repetition=1/1 duration=75s rviz=true",
                stdout,
            )
            self.assertIn("[icra] LOG ", stdout)
            self.assertIn("/full-r01/stdout.log", stdout)

    def test_process_wait_reports_progress_periodically(self):
        now = [0.0]
        progress = []

        class Process:
            def poll(self):
                return None if now[0] < 5.0 else 0

        exited = MODULE._wait_for_exit(
            Process(), deadline_s=10.0,
            on_progress=progress.append, progress_interval_s=2.0,
            clock=lambda: now[0], sleep=lambda delay: now.__setitem__(
                0, now[0] + delay),
        )

        self.assertTrue(exited)
        self.assertEqual([round(value, 1) for value in progress], [2.0, 4.0])

    def test_interrupt_cleans_owned_process_groups_and_writes_summary(self):
        class Process:
            def __init__(self, pid):
                self.pid = pid
                self.returncode = None

            def poll(self):
                return None

        capture = Process(101)
        launch = Process(202)
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_root = Path(temporary_directory) / "full-r01"
            with mock.patch.object(
                    MODULE, "_node_names", return_value=set()), mock.patch.object(
                        MODULE.subprocess, "Popen",
                        side_effect=[capture, launch]), mock.patch.object(
                            MODULE, "_wait_capture_ready",
                            return_value=True), mock.patch.object(
                                MODULE, "_wait_for_exit",
                                side_effect=KeyboardInterrupt), mock.patch.object(
                                    MODULE, "_stop_group",
                                    return_value=(-2, True, False)) as stop_group:
                summary = MODULE._run_one(
                    "full", run_root, Path(temporary_directory) / "install")

            self.assertEqual(summary["result"], "INTERRUPTED")
            self.assertEqual(summary["failures"], ["interrupted"])
            self.assertTrue(summary["launch_group_cleared"])
            self.assertTrue(summary["capture_group_cleared"])
            self.assertEqual(
                [call.args[0] for call in stop_group.call_args_list],
                [launch, capture],
            )
            persisted = json.loads((run_root / "summary.json").read_text())
            self.assertEqual(persisted["result"], "INTERRUPTED")

    def test_cli_returns_130_and_persists_interrupted_session(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root)
            interrupted = MODULE._result(["interrupted"])
            interrupted["result"] = "INTERRUPTED"
            output = io.StringIO()
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        return_value=interrupted), contextlib.redirect_stdout(output):
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 130)
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertEqual(session["result"], "INTERRUPTED")
            self.assertEqual(session["interrupted_stage"], "full")
            self.assertIn("[icra] INTERRUPTED stage=full", output.getvalue())

    def test_forest_ab_runs_baseline_then_risk_with_frozen_contract(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root)
            args.scenario = MODULE.FOREST_SCENARIO
            args.forest_ab = True
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        return_value=MODULE._result([])) as run_one:
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 0)
            self.assertEqual(
                [call.kwargs["forest_variant"]
                 for call in run_one.call_args_list],
                ["baseline", "risk"],
            )
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertEqual(session["scenario"], MODULE.FOREST_SCENARIO)
            self.assertEqual(session["forest_scene"]["forest_seed"], 41021)
            self.assertEqual(len(session["forest_pairs"]), 1)

    def test_forest_pair_summary_records_branch_and_lineage_delta(self):
        baseline = MODULE._result([], forest_path={
            "selected_arms": {"0": "high", "1": "high"},
            "selected_low_risk_forks": 1,
            "selected_high_risk_forks": 3,
        })
        risk = MODULE._result(
            [],
            forest_path={
                "selected_arms": {str(index): "low" for index in range(4)},
                "selected_low_risk_forks": 4,
                "selected_high_risk_forks": 0,
            },
            forest_risk={"passing_generation_id": 17},
            selected_count=2,
            lineage_group_count=1,
        )

        pair = MODULE.summarize_forest_pair(baseline, risk, 2)

        self.assertTrue(pair["paired_pass"])
        self.assertEqual(pair["repetition"], 2)
        self.assertEqual(pair["delta_selected_low_risk_forks"], 3)
        self.assertEqual(pair["risk_contrast_generation_id"], 17)
        self.assertEqual(pair["risk_selected_lineage_count"], 2)

    def test_forest_ab_finishes_risk_variant_after_baseline_failure(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root)
            args.scenario = MODULE.FOREST_SCENARIO
            args.forest_ab = True
            failed_baseline = MODULE._result(
                ["forest_baseline_branch_selection_failed"],
                forest_path={"selected_low_risk_forks": 2})
            passing_risk = MODULE._result(
                [], forest_path={"selected_low_risk_forks": 4})
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        side_effect=[failed_baseline, passing_risk]) as run_one:
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 1)
            self.assertEqual(run_one.call_count, 2)
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertEqual(session["result"], "FAIL")
            self.assertEqual(len(session["forest_pairs"]), 1)
            self.assertFalse(session["forest_pairs"][0]["paired_pass"])

    def test_preflight_interrupt_persists_interrupted_session(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root)
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    side_effect=KeyboardInterrupt):
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 130)
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertEqual(session["result"], "INTERRUPTED")
            self.assertEqual(session["interrupted_stage"], "preflight")

    def test_between_stage_interrupt_persists_completed_runs(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root, through="p0")
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        side_effect=[MODULE._result([]), KeyboardInterrupt]):
                exit_code = MODULE._run_main(args)

            self.assertEqual(exit_code, 130)
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertEqual(session["result"], "INTERRUPTED")
            self.assertEqual(session["interrupted_stage"], "p0")
            self.assertEqual(len(session["runs"]), 1)
            self.assertEqual(session["runs"][0]["stage"], "estimator")

    def test_progress_after_nominal_duration_reports_waiting_for_exit(self):
        message = MODULE._progress_message(
            "full", elapsed_s=80.0, duration_s=75.0, timeout_s=95.0,
            stdout_path=Path("/tmp/full/stdout.log"),
        )
        self.assertIn("WAITING_EXIT stage=full", message)
        self.assertIn("elapsed=80s timeout=95s", message)
        self.assertNotIn("80s/75s", message)


class TestStageAnalyzer(unittest.TestCase):
    @staticmethod
    def forest_generation(low_multiplier=0.8):
        return {
            "kind": "forest_risk_generation",
            "payload": {
                "generation_id": 7,
                "forks": [{
                    "fork_index": index,
                    "low": {
                        "valid_count": 20, "sample_count": 20,
                        "mean_c_pi": 10.0 * low_multiplier,
                        "max_c_pi": 12.0 * low_multiplier,
                        "mean_pl": 7.0 * low_multiplier,
                        "mean_gnss_ratio": 0.8,
                        "mean_lidar_ratio": 0.4 * low_multiplier,
                        "mean_fim_ratio": 0.5 * low_multiplier,
                        "max_fim_ratio": 0.6 * low_multiplier,
                        "mean_risk_ratio": 0.8,
                    },
                    "high": {
                        "valid_count": 20, "sample_count": 20,
                        "mean_c_pi": 10.0, "max_c_pi": 12.0,
                        "mean_pl": 7.0,
                        "mean_gnss_ratio": 0.8,
                        "mean_lidar_ratio": 0.4,
                        "mean_fim_ratio": 0.5,
                        "max_fim_ratio": 0.6,
                        "mean_risk_ratio": 0.8,
                    },
                } for index in range(4)],
            },
        }

    def test_forest_risk_gate_requires_all_four_real_low_risk_arms(self):
        passed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.8)])
        self.assertEqual(passed["result"], "PASS")
        failed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.95)])
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("forest_risk_contrast_missing", failed["failures"])

    def test_forest_risk_gate_accepts_progressive_same_generation_forks(self):
        records = []
        for selected_index in range(4):
            record = self.forest_generation(low_multiplier=0.8)
            record["payload"]["generation_id"] = 20 + selected_index
            for fork in record["payload"]["forks"]:
                if fork["fork_index"] == selected_index:
                    continue
                for arm in (fork["low"], fork["high"]):
                    arm["sample_count"] = 0
                    arm["valid_count"] = 0
            records.append(record)

        passed = MODULE.analyze_forest_risk(records)

        self.assertEqual(passed["result"], "PASS")
        self.assertEqual(passed["passing_generation_ids"], {
            "0": 20, "1": 21, "2": 22, "3": 23,
        })
        self.assertEqual(passed["missing_forks"], [])

    def test_forest_cloud_capture_reduces_points_by_generation_and_arm(self):
        points = []
        for fork in MODULE.forest_scene_contract()["forks"]:
            x = fork["x_min_m"] + 0.5 * fork["length_m"]
            for y, scale in (
                (4.0 * fork["low_risk_y_sign"], 0.7),
                (-2.8 * fork["low_risk_y_sign"], 1.0),
            ):
                points.append({
                    "x": x, "y": y, "z": 1.5,
                    "pl": 10.0 * scale, "hpl": 8.0 * scale,
                    "vpl": 10.0 * scale, "c_pi": 10.0 * scale,
                    "risk_ratio": scale,
                    "gnss_risk_ratio": scale,
                    "lidar_risk_ratio": scale,
                    "fim_risk_ratio": scale,
                    "floor_increment_h": 0.0,
                    "floor_increment_v": 0.0,
                    "floor_source_h": 0,
                    "floor_source_v": 0,
                    "valid": 1, "unknown": 0, "stale": 0,
                    "occupied": 0, "observed": 1,
                    "generation_id": 9,
                })
        summary = MODULE.summarize_forest_risk_cloud(points)
        self.assertEqual(summary["generation_id"], 9)
        self.assertEqual(len(summary["forks"]), 4)
        self.assertTrue(all(
            fork["low"]["mean_c_pi"] < fork["high"]["mean_c_pi"]
            for fork in summary["forks"]))

    def test_forest_path_gate_distinguishes_baseline_and_risk_variants(self):
        contract = MODULE.forest_scene_contract()
        low_points = []
        high_points = []
        for fork in contract["forks"]:
            x = fork["x_min_m"] + 0.5 * fork["length_m"]
            low_points.append([x, 4.0 * fork["low_risk_y_sign"], 1.5])
            high_points.append([x, -2.8 * fork["low_risk_y_sign"], 1.5])
        low = MODULE.analyze_forest_path([
            {"kind": "poscmd", "payload": {"position_xyz": point}}
            for point in low_points], "risk")
        high = MODULE.analyze_forest_path([
            {"kind": "poscmd", "payload": {"position_xyz": point}}
            for point in high_points], "baseline")
        self.assertEqual(low["result"], "PASS")
        self.assertEqual(high["result"], "PASS")
        self.assertEqual(low["selected_low_risk_forks"], 4)
        self.assertEqual(high["selected_high_risk_forks"], 4)
    def test_p0_deadline_uses_launch_start_not_earlier_capture_start(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_root = Path(temporary_directory)
            (run_root / "capture_ready.json").write_text(json.dumps({
                "ready_steady_s": 85.0,
            }))
            (run_root / "launch_started.json").write_text(json.dumps({
                "started_steady_s": 85.2,
            }))
            self.assertEqual(MODULE._stage_start_steady_s(run_root), 85.2)

        rows = [healthy(index + 1, 100.1 + index * 0.5)
                for index in range(31)]
        summary = MODULE.analyze_p0(rows, 85.2)
        self.assertEqual(summary["result"], "PASS")
        self.assertAlmostEqual(summary["first_healthy_delay_s"], 14.9)

    def test_estimator_rejects_wrong_scale_and_large_state(self):
        summary = MODULE.analyze_estimator({
            "acc_scale": 9.80665,
            "rotation_deg": 103.9,
            "velocity_norm_mps": 23.5,
            "bias_norm": 1.4,
            "odom_count": 20,
            "position_error_p95_m": 0.01,
            "finite": True,
        })
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("acc_scale_not_si", summary["failures"])
        self.assertIn("initial_rotation_exceeded", summary["failures"])

    def test_p0_requires_an_unbroken_fifteen_second_healthy_window(self):
        rows = [healthy(index + 1, float(index)) for index in range(16)]
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")
        rows[8]["payload"]["stale"] = True
        rows[8]["payload"]["reason"] = "occupancy_stale"
        failed = MODULE.analyze_p0(rows)
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", failed["failures"])

    def test_p0_window_includes_first_observation_past_boundary(self):
        rows = [healthy(index + 1, index * 0.47) for index in range(34)]
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "PASS")
        self.assertGreaterEqual(summary["window_span_s"], 15.0)

    def test_p0_does_not_restart_after_first_healthy_generation(self):
        rows = [healthy(1, 0.0)]
        rows.append(healthy(1, 0.5))
        rows[-1]["payload"]["reason"] = "occupancy_stale"
        rows.extend(healthy(index + 2, 1.0 + index * 0.5)
                    for index in range(31))
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", summary["failures"])

    def test_p0_ignores_unhealthy_startup_before_first_generation(self):
        rows = [healthy(0, 0.0)]
        rows[0]["payload"].update({
            "ready": False, "stale": True, "reason": "startup",
        })
        rows.extend(healthy(index + 1, 1.0 + index * 0.5)
                    for index in range(31))
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")

    def test_p4_requires_selected_lineage_and_stable_publication(self):
        summary = MODULE.analyze_stage_records(
            "p4", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[], lineage=[], bsplines=[], p5_status=[], poscmd_times=[])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p4_risk_selected_missing", summary["failures"])
        self.assertIn("stable_bspline_missing", summary["failures"])

    def test_full_rejects_mixed_p5_identity_and_unsafe_runtime(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        statuses = [
            {"receive_steady_s": 19.0,
             "payload": {"phase": "final", "action": "OK", "reason": "ok",
                         "current_integrity_source": "FUSED",
                         "final_candidate_traj_id": 12,
                         "final_candidate_start_time_ns": 34,
                         "samples": []}},
            {"receive_steady_s": 30.0,
             "payload": {"phase": "runtime", "action": "REQUEST_REPLAN",
                         "raw_action": "REQUEST_REPLAN", "reason": "future_bad",
                         "raw_reason": "future_bad", "active_reasons": ["future_bad"],
                         "current_integrity_source": "FUSED",
                         "samples": [{"trajectory_sample_source": "runtime_committed",
                                      "trajectory_id": 99,
                                      "trajectory_start_time_ns": 34}]}}
        ]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses, poscmd_times=[25.0 + i * 0.01 for i in range(600)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_runtime_identity_or_status_invalid", summary["failures"])

    def test_full_reports_the_existing_seven_stage_chain(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
            row["no_collision_refinement_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        safe_common = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok",
            "active_reasons": [], "current_reason": "",
            "future_reason": "", "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 19.0,
            "payload": {
                **safe_common, "phase": "final",
                "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34,
                "samples": [],
            },
        }, {
            "receive_steady_s": 19.5,
            "payload": {
                **safe_common, "phase": "final_publish_authorized",
                "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34,
                "final_evaluation_stamp_s": 18.0,
                "final_publish_authorization_stamp_s": 19.0,
            },
        }, {
            "receive_steady_s": 21.0,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 12,
                    "trajectory_start_time_ns": 34,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "PASS")
        self.assertIsNone(summary["seven_stage"]["first_missing_stage"])
        self.assertTrue(all(summary["seven_stage"]["stage_status"].values()))
        self.assertEqual(summary["p5_runtime_committed_count"], 1)

    def test_full_rejects_capture_missing_selected_terminal_identity(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 13, "start_time_ns": 35}}
            for index in range(3)
        ]
        safe_common = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok",
            "active_reasons": [], "current_reason": "",
            "future_reason": "", "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 19.0,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 11,
                    "trajectory_start_time_ns": 33,
                }],
            },
        }, {
            "receive_steady_s": 19.5,
            "payload": {
                **safe_common, "phase": "final",
                "final_candidate_traj_id": 13,
                "final_candidate_start_time_ns": 35,
                "samples": [],
            },
        }, {
            "receive_steady_s": 20.1,
            "payload": {
                **safe_common, "phase": "runtime",
                "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 13,
                    "trajectory_start_time_ns": 35,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_final_identity_or_status_invalid",
                      summary["failures"])
        self.assertIn("existing_seven_stage_analyzer_failed",
                      summary["failures"])

    def test_p5_final_ok_must_precede_same_identity_publication(self):
        decision = selected_decision()
        lineage = lineage_for(decision, trajectory_id=12, start_ns=34)
        for row in lineage:
            row["closed_collision_observed"] = "1"
            row["no_collision_refinement_observed"] = "1"
        bsplines = [
            {"receive_steady_s": 20.0 + index * 3.0,
             "payload": {"trajectory_id": 12, "start_time_ns": 34}}
            for index in range(3)
        ]
        safe = {
            "action": "OK", "raw_action": "OK",
            "reason": "ok", "raw_reason": "ok", "active_reasons": [],
            "current_reason": "", "future_reason": "",
            "final_candidate_rejected": False,
            "current_integrity_source": "FUSED",
        }
        statuses = [{
            "receive_steady_s": 29.0,
            "payload": {
                **safe, "phase": "final", "final_candidate_traj_id": 12,
                "final_candidate_start_time_ns": 34, "samples": [],
            },
        }, {
            "receive_steady_s": 30.0,
            "payload": {
                **safe, "phase": "runtime", "samples": [{
                    "trajectory_sample_source": "runtime_committed",
                    "trajectory_id": 12, "trajectory_start_time_ns": 34,
                }],
            },
        }]
        summary = MODULE.analyze_stage_records(
            "full", [healthy(index + 1, float(index)) for index in range(16)],
            decisions=[decision], lineage=lineage, bsplines=bsplines,
            p5_status=statuses,
            poscmd_times=[20.0 + index * 0.01 for index in range(601)])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p5_published_candidate_without_prior_final_ok",
                      summary["failures"])

    def test_shutdown_rejects_escalation_and_nonzero_process_exit(self):
        summary = MODULE.analyze_shutdown(
            launch_exit_code=0,
            stdout="failed to terminate after SIGINT, escalating to SIGTERM\n"
                   "process has died [exit code -11]",
            owned_group_cleared=True,
            residual_nodes=[],
        )
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("shutdown_escalated", summary["failures"])
        self.assertIn("child_process_nonzero", summary["failures"])

    def test_shutdown_rejects_runner_kill_escalation(self):
        summary = MODULE.analyze_shutdown(
            launch_exit_code=0,
            stdout="",
            owned_group_cleared=True,
            residual_nodes=[],
            runner_escalated=True,
        )
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("shutdown_escalated", summary["failures"])


if __name__ == "__main__":
    unittest.main()
