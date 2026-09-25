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


def forest_v2_healthy(generation, received):
    return {
        "kind": "p0_health",
        "receive_steady_s": received,
        "payload": {
            "ready": True,
            "stale": False,
            "reason": "ok",
            "generation_id": generation,
            "result_generation_id": generation,
            "refresh_evidence_state": "COMPLETED_SUCCESS",
            "snapshot_config_hash": "config-v2",
            "source_identity_hash": f"sources-{generation}",
            "geometry_id": "planning-lattice-v2",
            "frame_id": "map",
            "grid_origin_m": [-21.0, -11.0, 0.0],
            "grid_extent_m": [42.0, 22.0, 8.0],
            "grid_dimensions": [84, 44, 16],
            "grid_resolution_m": 0.5,
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "alert_limit_h_m": 20.0,
            "alert_limit_v_m": 40.0,
            "source_occupancy_generation": 100 + generation,
            "source_occupancy_stamp_s": received - 0.1,
            "source_prior_generation": 200 + generation,
            "source_prior_stamp_s": received - 0.2,
            "source_gnss_generation": 300 + generation,
            "source_gnss_stamp_s": received - 0.3,
            "source_lidar_generation": 400 + generation,
            "source_lidar_stamp_s": received - 0.4,
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
        "segment_start_x": "-15.5",
        "segment_start_y": "0.0",
        "segment_start_z": "1.5",
        "segment_end_x": "-14.5",
        "segment_end_y": "0.0",
        "segment_end_z": "1.5",
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
    def test_current_p4_schema_is_accepted_as_formal_risk_evidence(self):
        self.assertIn(
            "p4_forward_route_decision_v17",
            MODULE.P4_FORWARD_DECISION_SCHEMAS)
        self.assertIn(
            "p4_forward_route_decision_v17",
            MODULE.P4_FORMAL_RISK_SAMPLE_SCHEMAS)

    def test_limited_prefix_is_independent_not_part_of_through_ladder(self):
        self.assertIn("limited-prefix", MODULE.STAGE_CHOICES)
        self.assertNotIn("limited-prefix", MODULE.STAGE_ORDER)
        args = MODULE.stage_launch_args(
            "limited-prefix", MODULE.FOREST_SCENARIO)
        self.assertEqual(args["p4.debug_generation_probe_enable"], "true")

    def test_continuous_flight_stage_has_180_second_budget(self):
        self.assertIn("continuous-flight", MODULE.STAGE_CHOICES)
        self.assertNotIn("continuous-flight", MODULE.STAGE_ORDER)
        self.assertEqual(
            MODULE.stage_duration_s("continuous-flight"), 180.0)

    def test_continuous_flight_uses_command_controller_and_odom_evidence(self):
        records = []
        identities = [(7, trajectory, 1_000_000_000 * trajectory,
                       f"curve-{trajectory}")
                      for trajectory in (10, 11, 12)]
        for execution, trajectory, start_ns, curve_hash in identities:
            common = {
                "execution_instance_id": execution,
                "trajectory_id": trajectory,
                "start_time_ns": start_ns,
                "curve_hash": curve_hash,
            }
            records.extend([
                {"kind": "normal_bspline", "payload": common},
                {"kind": "trajectory_status", "payload": {
                    **common, "state": "ACTIVATED"}},
                {"kind": "poscmd", "payload": {
                    **common, "position_xyz": [0.0, 0.0, 1.5]}},
                {"kind": "controller_trace", "payload": {
                    **common, "position_xyz": [0.0, 0.0, 1.5],
                    "feedback_position_xyz": [0.05, 0.0, 1.5],
                    "saturated": False}},
            ])
        records.extend([
            {"kind": "iap_odom", "payload": {
                "stamp_s": 1.0, "position_m": [-18.0, 0.0, 1.5],
                "velocity_mps": [1.0, 0.0, 0.0]}},
            {"kind": "iap_odom", "payload": {
                "stamp_s": 10.0, "position_m": [0.0, 0.0, 1.5],
                "velocity_mps": [1.0, 0.0, 0.0]}},
            {"kind": "iap_odom", "payload": {
                "stamp_s": 20.0, "position_m": [18.0, 0.0, 1.5],
                "velocity_mps": [0.0, 0.0, 0.0]}},
        ])

        summary = MODULE.analyze_continuous_flight(records)

        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["successor_switch_count"], 2)
        self.assertAlmostEqual(summary["maximum_tracking_error_m"], 0.05)

    def test_continuous_flight_allows_unbound_hover_before_first_activation(self):
        records = [{
            "kind": "poscmd", "receive_steady_s": 0.5,
            "payload": {
                "execution_instance_id": 0, "trajectory_id": 0,
                "start_time_ns": 0, "curve_hash": "",
                "position_xyz": [-18.0, 0.0, 1.5],
            },
        }, {
            "kind": "controller_trace", "receive_steady_s": 0.6,
            "payload": {
                "execution_instance_id": 0, "trajectory_id": 0,
                "start_time_ns": 0, "curve_hash": "",
                "position_xyz": [-18.0, 0.0, 1.5],
                "feedback_position_xyz": [-18.0, 0.0, 1.5],
                "saturated": False,
            },
        }]
        for index, trajectory in enumerate((10, 11, 12)):
            common = {
                "execution_instance_id": 7,
                "trajectory_id": trajectory,
                "start_time_ns": 1_000_000_000 * trajectory,
                "curve_hash": f"curve-{trajectory}",
            }
            stamp = 1.0 + index
            records.extend([
                {"kind": "normal_bspline", "receive_steady_s": stamp - .1,
                 "payload": common},
                {"kind": "trajectory_status", "receive_steady_s": stamp,
                 "payload": {**common, "state": "ACTIVATED"}},
                {"kind": "poscmd", "receive_steady_s": stamp + .01,
                 "payload": {**common,
                             "position_xyz": [float(index), 0.0, 1.5]}},
                {"kind": "controller_trace", "receive_steady_s": stamp + .02,
                 "payload": {
                     **common,
                     "position_xyz": [float(index), 0.0, 1.5],
                     "feedback_position_xyz": [float(index), 0.0, 1.5],
                     "saturated": False,
                 }},
            ])
            if index == 0:
                for receive_s in (stamp + .005, stamp + .03):
                    records.append({
                        "kind": "controller_trace",
                        "receive_steady_s": receive_s,
                        "payload": {
                            "execution_instance_id": 0, "trajectory_id": 0,
                            "start_time_ns": 0, "curve_hash": "",
                            "position_xyz": [-18.0, 0.0, 1.5],
                            "feedback_position_xyz": [-18.0, 0.0, 1.5],
                            "saturated": False,
                        },
                    })
        records.extend([
            {"kind": "iap_odom", "payload": {
                "stamp_s": 1.0, "position_m": [-18.0, 0.0, 1.5],
                "velocity_mps": [1.0, 0.0, 0.0]}},
            {"kind": "iap_odom", "payload": {
                "stamp_s": 2.0, "position_m": [0.0, 0.0, 1.5],
                "velocity_mps": [1.0, 0.0, 0.0]}},
            {"kind": "iap_odom", "payload": {
                "stamp_s": 3.0, "position_m": [18.0, 0.0, 1.5],
                "velocity_mps": [0.0, 0.0, 0.0]}},
        ])

        summary = MODULE.analyze_continuous_flight(records)

        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["startup_hover_command_count"], 1)
        self.assertEqual(summary["startup_hover_trace_count"], 3)

        records.append({
            "kind": "poscmd", "receive_steady_s": 4.0,
            "payload": {
                "execution_instance_id": 0, "trajectory_id": 0,
                "start_time_ns": 0, "curve_hash": "",
                "position_xyz": [18.0, 0.0, 1.5],
            },
        })
        failed = MODULE.analyze_continuous_flight(records)
        self.assertIn("position_command_identity_incomplete",
                      failed["failures"])

    def test_continuous_stage_process_result_ignores_launch_exit(self):
        failures = MODULE.run_process_failures(
            "continuous-flight", early_exit=True, launch_exit_code=-9,
            launch_group_cleared=True, capture_group_cleared=True,
            graph_failures=[])
        self.assertEqual(failures, [])

        failures = MODULE.run_process_failures(
            "full", early_exit=True, launch_exit_code=-9,
            launch_group_cleared=True, capture_group_cleared=True,
            graph_failures=[])
        self.assertEqual(
            failures, ["launch_exited_early", "launch_exit_nonzero"])

    def test_process_group_resource_stats_reports_peak_and_cpu_cores(self):
        stats = MODULE.process_group_resource_stats([
            {"elapsed_s": 0.0, "process_count": 2,
             "rss_bytes": 100 * 1024 * 1024, "cpu_seconds": 1.0},
            {"elapsed_s": 5.0, "process_count": 4,
             "rss_bytes": 160 * 1024 * 1024, "cpu_seconds": 6.0},
            {"elapsed_s": 10.0, "process_count": 3,
             "rss_bytes": 140 * 1024 * 1024, "cpu_seconds": 8.5},
        ])
        self.assertEqual(stats["sample_count"], 3)
        self.assertEqual(stats["peak_process_count"], 4)
        self.assertAlmostEqual(stats["peak_rss_mib"], 160.0)
        self.assertAlmostEqual(stats["mean_cpu_cores"], 0.75)
        self.assertAlmostEqual(stats["peak_cpu_cores"], 1.0)

    def test_planner_local_map_runtime_stats_tracks_rate_bandwidth_and_delta_chain(self):
        records = [
            {"kind": "planner_local_map_current", "receive_steady_s": 1.0,
             "payload": {"payload_bytes": 120,
                         "position_m": [0.0, 0.0, 0.0]}},
            {"kind": "planner_local_map_current", "receive_steady_s": 1.1,
             "payload": {"payload_bytes": 120,
                         "position_m": [3.0, 4.0, 0.0]}},
            {"kind": "planner_local_map_delta", "receive_steady_s": 1.0,
             "payload": {"base_generation": 0, "generation": 1,
                         "complete": True, "active_frame_count": 1,
                         "added_payload_bytes": 120,
                         "frame_contract_id": "sha256:a"}},
            {"kind": "planner_local_map_delta", "receive_steady_s": 1.5,
             "payload": {"base_generation": 1, "generation": 2,
                         "complete": True, "active_frame_count": 2,
                         "added_payload_bytes": 240,
                         "frame_contract_id": "sha256:a"}},
        ]
        stats = MODULE.planner_local_map_runtime_stats(records)
        self.assertAlmostEqual(stats["current_rate_hz"], 10.0)
        self.assertAlmostEqual(stats["window_delta_rate_hz"], 2.0)
        self.assertGreater(stats["total_xyz_payload_mib_s"], 0.0)
        self.assertEqual(stats["max_active_frame_count"], 2)
        self.assertEqual(stats["frame_contract_ids"], ["sha256:a"])
        self.assertTrue(stats["generation_contiguous"])
        self.assertEqual(stats["first_position_m"], [0.0, 0.0, 0.0])
        self.assertEqual(stats["last_position_m"], [3.0, 4.0, 0.0])
        self.assertAlmostEqual(stats["displacement_m"], 5.0)

    def test_planner_local_map_latency_stats_extracts_component_windows(self):
        stats = MODULE.planner_local_map_latency_stats("\n".join([
            "GLIM callback snapshot latency count=100 p95_ms=0.12 "
            "max_ms=0.18 budget_ms=0.200",
            "adapter deskew serialize latency count=100 p95_ms=1.2 "
            "max_ms=1.8 budget_ms=2.000",
            "registered current frame latency count=100 p95_ms=8.2 "
            "max_ms=12.3 budget_ms=10.000",
            "registered current frame latency count=100 p95_ms=9.1 "
            "max_ms=15.0 budget_ms=10.000",
            "registered keyframe delta latency count=20 p95_ms=31.0 "
            "max_ms=39.0 budget_ms=40.000",
            "sensor to occupancy latency count=100 p95_ms=62.0 "
            "max_ms=75.0 budget_ms=80.000",
        ]))
        self.assertAlmostEqual(stats["callback_p95_ms_max"], 0.12)
        self.assertAlmostEqual(stats["adapter_p95_ms_max"], 1.2)
        self.assertEqual(stats["current_windows"], 2)
        self.assertAlmostEqual(stats["current_p95_ms_max"], 9.1)
        self.assertAlmostEqual(stats["current_max_ms"], 15.0)
        self.assertEqual(stats["delta_windows"], 1)
        self.assertAlmostEqual(stats["delta_p95_ms_max"], 31.0)
        self.assertAlmostEqual(
            stats["sensor_to_occupancy_p95_ms_max"], 62.0)

    def test_planner_local_map_acceptance_gates_missing_and_slow_evidence(self):
        runtime = {
            "current_rate_hz": 10.0,
            "window_delta_rate_hz": 2.0,
            "max_active_frame_count": 15,
            "generation_contiguous": True,
            "frame_contract_ids": ["sha256:a"],
            "total_xyz_payload_mib_s": 0.9,
        }
        latency = {
            "callback_windows": 1, "callback_p95_ms_max": 0.1,
            "adapter_windows": 1, "adapter_p95_ms_max": 1.0,
            "current_windows": 1, "current_p95_ms_max": 9.0,
            "delta_windows": 1, "delta_p95_ms_max": 39.0,
            "sensor_to_occupancy_windows": 1,
            "sensor_to_occupancy_p95_ms_max": 70.0,
        }
        self.assertEqual(
            MODULE.planner_local_map_acceptance_failures(runtime, latency),
            [])
        latency["current_p95_ms_max"] = 10.0
        latency["adapter_windows"] = 0
        self.assertEqual(
            MODULE.planner_local_map_acceptance_failures(runtime, latency),
            ["planner_local_map_adapter_latency_missing",
             "planner_local_map_current_latency_exceeded"])

    def test_icra_rviz_keeps_environment_faint_and_risk_cloud_legible(self):
        rviz = (REPO / "config/sim_demo11/test_icra.rviz").read_text()

        def display_block(name):
            before, after = rviz.split(f"Name: {name}", 1)
            return ("- Alpha:" + before.rsplit("- Alpha:", 1)[1] +
                    f"Name: {name}" + after.split("- Alpha:", 1)[0])

        def display_alpha(block):
            return float(block.splitlines()[0].split(":", 1)[1].strip())

        environment = display_block("Global Obstacle Map")
        self.assertIn("Color: 190; 195; 200", environment)
        self.assertIn("Color Transformer: FlatColor", environment)
        self.assertIn("Style: Flat Squares", environment)
        self.assertAlmostEqual(display_alpha(environment), 0.55, places=6)

        current_lidar = display_block("Current First-Hit LiDAR")
        self.assertIn("Value: /sim/drone_0/lidar", current_lidar)
        self.assertIn("Reliability Policy: Best Effort", current_lidar)
        self.assertIn("Durability Policy: Volatile", current_lidar)
        self.assertIn("Color Transformer: FlatColor", current_lidar)
        self.assertIn("Style: Points", current_lidar)
        self.assertAlmostEqual(display_alpha(current_lidar), 0.9, places=6)

        registered_lidar = display_block("GLIM Registered Current LiDAR")
        self.assertIn("Value: /iap/local_map/current_hits_map", registered_lidar)
        self.assertIn("Reliability Policy: Best Effort", registered_lidar)
        self.assertIn("Durability Policy: Volatile", registered_lidar)
        self.assertIn("Color Transformer: FlatColor", registered_lidar)
        self.assertIn("Style: Points", registered_lidar)
        self.assertAlmostEqual(display_alpha(registered_lidar), 0.8, places=6)

        predicted_pl = display_block("Predicted PL Cloud")
        self.assertIn("Reliability Policy: Best Effort", predicted_pl)
        self.assertIn("Color Transformer: RGB8", predicted_pl)
        self.assertIn("Style: Spheres", predicted_pl)
        self.assertIn("Size (Pixels): 6", predicted_pl)
        self.assertIn("Size (m): 0.28", predicted_pl)
        self.assertAlmostEqual(display_alpha(predicted_pl), 0.45, places=6)

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

        mirror_args = MODULE.stage_launch_args(
            "p4", MODULE.MIRROR_SCENARIO)
        self.assertEqual(mirror_args["scenario"], MODULE.MIRROR_SCENARIO)

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

        bds = MODULE.stage_launch_args(
            "limited-prefix", MODULE.FOREST_SCENARIO, gnss_arm="bds")
        self.assertEqual(bds["gnss_enabled_constellations"],
                         "GPS,BDS,GAL,GLO")
        self.assertNotIn("p4.forward.gnss_core_policy", bds)
        with self.assertRaises(ValueError):
            MODULE.stage_launch_args(
                "limited-prefix", MODULE.FOREST_SCENARIO,
                gnss_arm="bds", gnss_core_policy="whole_curve_common_core")
        self.assertEqual(
            MODULE.forest_scene_contract(
                MODULE.FOREST_SCENARIO, "bds")["gnss"]
            ["enabled_constellations"],
            ["GPS", "BDS", "GAL", "GLO"])
        with self.assertRaises(ValueError):
            MODULE.stage_launch_args(
                "limited-prefix", MODULE.FOREST_SCENARIO,
                gnss_arm="unsupported")
        with self.assertRaises(ValueError):
            MODULE.stage_launch_args(
                "limited-prefix", MODULE.FOREST_SCENARIO,
                gnss_core_policy="per_point_pick_best")

    def test_forest_scene_contract_is_expanded_and_fingerprinted(self):
        contract = MODULE.forest_scene_contract()
        self.assertEqual(contract["schema_version"],
                         "icra_dense_forest_four_fork_v2")
        self.assertEqual(contract["forest_seed"], 41021)
        self.assertEqual(contract["risk_seed"], 21)
        self.assertEqual(contract["low_risk_y_signs"], [-1, 1, -1, 1])
        self.assertEqual(contract["gnss"]["enabled_constellations"],
                         ["GPS", "BDS", "GAL", "GLO"])
        self.assertEqual(
            contract["gnss"]["measured_epoch_support_radius_m"], 0.0)
        self.assertEqual(
            contract["gnss"]["measured_epoch_integrity_max_delta_s"], 0.25)
        self.assertEqual(contract["gnss"]["clearance_transition_m"], 0.4)
        self.assertEqual(contract["planner_executor_thread_count"], 6)
        self.assertTrue(contract["p0_conservative_max_with_gnss"])
        self.assertFalse(
            contract["online_mapping"]["unknown_as_occupied"])
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
                "enabled_constellations": "GPS,BDS,GAL,GLO",
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
                "unknown_as_occupied": False,
                "current_vehicle_clearance_radius_m": 0.35,
                "provider_cost_source": "pre_conservative_fim_ratio",
                "require_safety_ratio_below_one_for_cost": True,
                "alert_limit_policy_id": "fixed_hal20_val40_v1",
                "alert_limit_h_m": 20.0,
                "alert_limit_v_m": 40.0,
                "skip_occupied_voxels": True,
                "use_current_integrity_prior": True,
                "conservative_max_with_gnss": True,
                "gnss_clearance_transition_m": 0.4,
                "executor_thread_count": 6,
            },
            "integrity_alert_limits": {
                "dynamic": False,
                "hal_m": 20.0,
                "val_m": 40.0,
            },
            "p5_alert_limits": {
                "mode": "config_constant",
                "hal_m": 20.0,
                "val_m": 40.0,
            },
        }
        with tempfile.TemporaryDirectory() as raw:
            run_root = Path(raw)
            manifest = run_root / "exports" / "run" / "test_planner_manifest.json"
            manifest.parent.mkdir(parents=True)
            manifest.write_text(json.dumps({
                "scenario_contract": contract,
                "scenario_fingerprint": "sha256:effective",
                "lidar_renderer": {
                    "mode": "spherical_first_hit_v1",
                    "ray_count": 20480,
                    "output_semantics": "hit_only_first_return_pointcloud2",
                },
            }))
            evidence = MODULE.forest_manifest_evidence(run_root)
            self.assertTrue(evidence["matches_expected"])
            self.assertEqual(evidence["scenario_fingerprint"],
                             "sha256:effective")
            self.assertEqual(evidence["lidar_renderer"]["ray_count"], 20480)
            self.assertEqual(
                MODULE.effective_lidar_renderer_evidence(run_root)["mode"],
                "spherical_first_hit_v1")
            contract["p5_alert_limits"]["val_m"] = 20.0
            manifest.write_text(json.dumps({
                "scenario_contract": contract,
                "scenario_fingerprint": "sha256:p5-drifted",
            }))
            evidence = MODULE.forest_manifest_evidence(run_root)
            self.assertFalse(evidence["matches_expected"])
            self.assertIn("p5_alert_limits.val_m", evidence["mismatches"])
            contract["p5_alert_limits"]["val_m"] = 40.0
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

    def test_forest_full_stage_configures_ninety_second_runtime(self):
        self.assertEqual(MODULE.stage_duration_s(
            "p4", MODULE.FOREST_SCENARIO, "risk"), 90.0)
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.FOREST_SCENARIO, "risk"), 90.0)
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.FOREST_SCENARIO, "baseline"), 90.0)
        self.assertEqual(MODULE.stage_duration_s(
            "full", MODULE.DEFAULT_SCENARIO), 75.0)

    def test_limited_prefix_runtime_includes_terminal_hold_window(self):
        self.assertEqual(MODULE.stage_duration_s(
            "limited-prefix", MODULE.FOREST_SCENARIO), 100.0)

    def test_live_truth_audit_rejects_planner_world_subscription(self):
        with mock.patch.object(
                MODULE, "_node_names",
                return_value={"/drone_0_ego_planner_node"}), mock.patch.object(
                    MODULE, "_node_subscriptions",
                    return_value=(["/sim/drone_0/lidar",
                                   "/map_generator/global_cloud",
                                   "/sim/drone_0/truth_odom"], "")):
            evidence = MODULE.audit_planner_truth_isolation({}, timeout_s=0.1)
        self.assertFalse(evidence["pass"])
        self.assertEqual(evidence["forbidden_subscriptions"],
                         ["/map_generator/global_cloud",
                          "/sim/drone_0/truth_odom"])
        self.assertIn("planner_truth_subscription_detected",
                      evidence["failures"])


class TestRunnerLifecycle(unittest.TestCase):
    def test_raw_evidence_retention_is_summary_first_and_run_scoped(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            run_root = root / "continuous-flight-r01"
            exports = run_root / "exports"
            exports.mkdir(parents=True)
            prefix = exports / "planner_p4_risk_astar_debug.csv"
            raw_suffixes = (
                ".forward_risk_samples.csv",
                ".gnss_risk_detail.csv",
                ".runtime_window.csv",
                ".runtime_window_satellite.csv",
            )
            for suffix in raw_suffixes:
                Path(f"{prefix}{suffix}").write_text("raw\n")
            compact_paths = (
                Path(f"{prefix}.forward_candidates.csv"),
                Path(f"{prefix}.forward_channel_decisions.csv"),
            )
            for compact in compact_paths:
                compact.write_text("compact\n")
            outside = root / "planner_p4_risk_astar_debug.csv.gnss_risk_detail.csv"
            outside.write_text("outside\n")

            with self.assertRaises(RuntimeError):
                MODULE.apply_raw_evidence_retention(
                    run_root, retain_raw=False)

            (run_root / "summary.json").write_text("{}\n")
            retention = MODULE.apply_raw_evidence_retention(
                run_root, retain_raw=False)
            self.assertEqual(retention["policy"], "compact_default")
            self.assertEqual(len(retention["removed"]), len(raw_suffixes))
            self.assertTrue(all(path.is_file() for path in compact_paths))
            self.assertTrue(outside.is_file())

            for suffix in raw_suffixes:
                Path(f"{prefix}{suffix}").write_text("raw\n")
            retained = MODULE.apply_raw_evidence_retention(
                run_root, retain_raw=True)
            self.assertEqual(retained["policy"], "retain_full_detail")
            self.assertEqual(retained["removed"], [])
            for suffix in raw_suffixes:
                self.assertTrue(Path(f"{prefix}{suffix}").is_file())

    def test_first_hit_runtime_logs_are_summarized(self):
        stdout = "\n".join((
            "first-hit lidar frame=1 stamp=1.0 rays=20480 hits=123 "
            "dda_visits=100 latency_ms=41.5",
            "first-hit lidar frame=10 stamp=1.9 rays=20480 hits=456 "
            "dda_visits=200 latency_ms=52.0",
        ))
        stats = MODULE.lidar_runtime_stats(stdout)
        self.assertEqual(stats["sample_count"], 2)
        self.assertEqual(stats["ray_count"], 20480)
        self.assertEqual(stats["hit_count_min"], 123)
        self.assertEqual(stats["hit_count_max"], 456)
        self.assertEqual(stats["render_latency_ms_max"], 52.0)
        self.assertAlmostEqual(stats["effective_rate_hz"], 10.0)
        self.assertAlmostEqual(stats["frame_interval_s_max"], 0.1)
        renderer = {"mode": "spherical_first_hit_v1", "ray_count": 20480}
        self.assertEqual(MODULE.lidar_runtime_failures(renderer, stats), [])

        slow = dict(stats, render_latency_ms_p95=80.0,
                    effective_rate_hz=9.0, ray_count_min=1024,
                    frame_interval_s_max=0.25)
        self.assertEqual(
            MODULE.lidar_runtime_failures(renderer, slow),
            ["lidar_ray_count_drift", "lidar_render_p95_exceeded",
             "lidar_effective_rate_below_9_5hz",
             "lidar_frame_gap_exceeded_0_2s"],
        )

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

    def test_rviz_run_loads_installed_test_icra_config(self):
        install_root = Path("/workspace/install")

        path = MODULE.icra_rviz_config_path(install_root)

        self.assertEqual(
            path,
            Path("/workspace/install/iap/share/iap/config/sim_demo11/"
                 "test_icra.rviz"))

    def test_rviz_config_path_preserves_symlink_install_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source/test_icra.rviz"
            installed = root / "install/iap/share/iap/config/sim_demo11/"
            installed.mkdir(parents=True)
            source.parent.mkdir(parents=True)
            source.write_text("Visualization Manager: {}\n")
            installed_config = installed / "test_icra.rviz"
            installed_config.symlink_to(source)

            path = MODULE.icra_rviz_config_path(root / "install").absolute()

            self.assertEqual(path, installed_config.absolute())
            self.assertNotEqual(path, installed_config.resolve())

    def test_cli_forwards_explicit_raw_evidence_retention(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            args = self.runner_args(root)
            args.retain_raw_risk_detail = True
            with mock.patch.object(
                    MODULE, "_gpu_preflight",
                    return_value={"gpu_ready": True}), mock.patch.object(
                        MODULE, "_run_one",
                        return_value=MODULE._result([])) as run_one:
                self.assertEqual(MODULE._run_main(args), 0)

            self.assertTrue(
                run_one.call_args.kwargs["retain_raw_risk_detail"])
            session_path = next((root / "results").glob(
                "run-*/session_summary.json"))
            session = json.loads(session_path.read_text())
            self.assertTrue(session["retain_raw_risk_detail"])

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
            forest_risk={
                "contrast_generation_ids": {"0": 17},
                "passing_generation_ids": {"0": 17},
                "fork_evidence": {"0": {"geometry_id": "geometry"}},
            },
            selected_count=2,
            lineage_group_count=1,
        )

        pair = MODULE.summarize_forest_pair(baseline, risk, 2)

        self.assertTrue(pair["paired_pass"])
        self.assertEqual(pair["repetition"], 2)
        self.assertEqual(pair["delta_selected_low_risk_forks"], 3)
        self.assertEqual(pair["risk_contrast_generation_ids"], {"0": 17})
        self.assertEqual(
            pair["risk_identity_bound_generation_ids"], {"0": 17})
        self.assertEqual(
            pair["risk_fork_evidence"],
            {"0": {"geometry_id": "geometry"}})
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
    def test_v7_formal_selection_requires_frame_support_and_gnss_identity(self):
        decision = {
            "schema_version": "p4_forward_route_decision_v7",
            "stage": "forward_decision",
            "action": "RISK_SELECTED",
            "selection_authority": "FORMAL",
            "formal_support": "1",
            "selected_candidate_id": "2",
            "candidate_count": "2",
            "geometry_id": "planning-lattice-v2",
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "occupancy_generation": "3",
            "risk_generation": "5",
            "geometry_commit_verdict": "CLEAR_AFTER_UPDATE",
            "frame_contract_id": "planner-map-contract-a",
            "local_map_support_identity": "support-envelope-a",
            "gnss_epoch_identity": "71",
            "gnss_epoch_stamp_s": "100.25",
        }
        self.assertEqual(MODULE._selected_decisions([decision]), [decision])
        missing_support = dict(decision, local_map_support_identity="")
        self.assertEqual(MODULE._selected_decisions([missing_support]), [])
        missing_epoch = dict(decision, gnss_epoch_identity="0")
        self.assertEqual(MODULE._selected_decisions([missing_epoch]), [])

    def test_v6_formal_selection_accepts_distinct_local_satellite_sets(self):
        decision = {
            "schema_version": "p4_forward_route_decision_v6",
            "result_status": "READY",
            "stage": "forward_decision",
            "action": "RISK_SELECTED",
            "selection_authority": "FORMAL",
            "formal_support": "1",
            "selected_candidate_id": "2",
            "candidate_count": "2",
            "geometry_id": "planning-lattice-v2",
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "occupancy_generation": "3",
            "risk_generation": "5",
            "geometry_commit_verdict": "CLEAR_AFTER_UPDATE",
            "decision_event_id": "77",
        }
        base = {
            "schema_version": "p4_forward_route_decision_v6",
            "decision_event_id": "77",
            "sample_index": "0", "arc_length_m": "0",
            "x": "1", "y": "0", "z": "1", "query_time_s": "100.1",
            "gnss_known_count": "6", "gnss_visible_count": "5",
            "gnss_blocked_count": "1", "gnss_attenuated_count": "2",
            "gnss_unknown_count": "3", "gnss_used_count": "5",
            "gnss_raw_hpl": "5", "gnss_raw_vpl": "6",
            "gnss_receiver_raw_hpl": "4", "gnss_receiver_raw_vpl": "5",
            "gnss_spatial_delta_h": "1", "gnss_spatial_delta_v": "1",
            "gnss_anchor_hpl": "13", "gnss_anchor_vpl": "29",
            "gnss_anchored_hpl": "14", "gnss_anchored_vpl": "30",
            "gnss_temporal_growth_h": "0", "gnss_temporal_growth_v": "0",
            "hpl": "14", "vpl": "30", "hal": "20", "val": "40",
            "safety_ratio": "0.75", "fim_ratio": "0.4",
            "gnss_supported": "1", "lidar_supported": "1",
            "fim_supported": "1", "safety_state": "SAFE",
            "support_authority": "TRUSTED_LOCAL_MAP",
            "support_status": "MODEL_COMPLETE",
            "ranking_state": "COMPARABLE", "reason": "NONE",
        }
        first = dict(base, candidate_id="1", local_satellite_set_hash="41")
        second = dict(base, candidate_id="2", local_satellite_set_hash="42")
        rejected = dict(
            base,
            candidate_id="3",
            local_satellite_set_hash="43",
            support_status="OUTSIDE_ENVELOPE",
            gnss_supported="0",
            safety_state="UNKNOWN",
            ranking_state="INCOMPLETE",
            reason="GNSS_SKY_UNKNOWN",
        )

        candidates = [
            {"decision_event_id": "77", "candidate_id": "1",
             "length_m": "0"},
            {"decision_event_id": "77", "candidate_id": "2",
             "length_m": "0"},
        ]
        result = MODULE.analyze_forward_risk_samples(
            [decision], [first, second, rejected], candidates)

        self.assertEqual(result["failures"], [])
        self.assertEqual(result["local_satellite_set_count"], 3)

        truncated = dict(second, sample_index="1", arc_length_m="0.5")
        invalid = MODULE.analyze_forward_risk_samples(
            [decision], [first, truncated], candidates)
        self.assertIn(
            "p4_forward_risk_route_sample_coverage_invalid",
            invalid["failures"])

        wrong_length = [dict(candidates[0]), dict(candidates[1])]
        wrong_length[1]["length_m"] = "0.25"
        invalid = MODULE.analyze_forward_risk_samples(
            [decision], [first, second], wrong_length)
        self.assertIn(
            "p4_forward_risk_route_sample_coverage_invalid",
            invalid["failures"])

    def test_v5_formal_selection_allows_unselected_unsafe_candidate(self):
        decision = {
            "schema_version": "p4_forward_route_decision_v5",
            "stage": "forward_decision",
            "action": "RISK_SELECTED",
            "selection_authority": "FORMAL",
            "formal_support": "1",
            "selected_candidate_id": "2",
            "decision_event_id": "78",
        }
        base = {
            "schema_version": "p4_forward_route_decision_v5",
            "decision_event_id": "78", "sample_index": "0",
            "arc_length_m": "0", "x": "1", "y": "0", "z": "1",
            "query_time_s": "100.1", "gnss_known_count": "6",
            "gnss_visible_count": "5", "gnss_blocked_count": "1",
            "gnss_attenuated_count": "2", "gnss_unknown_count": "3",
            "gnss_used_count": "5", "gnss_raw_hpl": "5",
            "gnss_raw_vpl": "6", "gnss_receiver_raw_hpl": "4",
            "gnss_receiver_raw_vpl": "5", "gnss_spatial_delta_h": "1",
            "gnss_spatial_delta_v": "1", "hpl": "14", "vpl": "30",
            "gnss_anchor_hpl": "13", "gnss_anchor_vpl": "29",
            "gnss_anchored_hpl": "14", "gnss_anchored_vpl": "30",
            "gnss_temporal_growth_h": "0", "gnss_temporal_growth_v": "0",
            "hal": "20", "val": "40", "safety_ratio": "0.75",
            "fim_ratio": "0.4", "gnss_supported": "1",
            "lidar_supported": "1", "fim_supported": "1",
            "safety_state": "SAFE", "ranking_state": "COMPARABLE",
            "reason": "NONE",
        }
        safe_one = dict(
            base, candidate_id="1", local_satellite_set_hash="41")
        safe_two = dict(
            base, candidate_id="2", local_satellite_set_hash="42")
        unsafe = dict(
            base, candidate_id="3", local_satellite_set_hash="43",
            safety_state="UNSAFE", hpl="25", safety_ratio="1.25")

        result = MODULE.analyze_forward_risk_samples(
            [decision], [safe_one, safe_two, unsafe])

        self.assertEqual(result["failures"], [])

    def test_advisory_v3_is_not_counted_as_formal_risk_selection(self):
        advisory = {
            "schema_version": "p4_forward_route_decision_v3",
            "stage": "forward_decision",
            "decision_event_id": "901",
            "action": "ADVISORY_SELECTED",
            "selection_authority": "ADVISORY_NON_CERTIFIED",
            "formal_support": "0",
            "selection_applied": "1",
            "selected_candidate_id": "2",
            "candidate_count": "2",
            "geometry_id": "planning-lattice-v2",
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "occupancy_generation": "3",
            "risk_generation": "5",
        }

        self.assertEqual(MODULE._selected_decisions([advisory]), [])

    def test_v3_risk_selected_requires_formal_authority_and_support(self):
        row = {
            "schema_version": "p4_forward_route_decision_v3",
            "stage": "forward_decision",
            "action": "RISK_SELECTED",
            "selection_authority": "ADVISORY_NON_CERTIFIED",
            "formal_support": "0",
            "selected_candidate_id": "2",
            "candidate_count": "2",
            "geometry_id": "planning-lattice-v2",
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "occupancy_generation": "3",
            "risk_generation": "5",
        }
        self.assertEqual(MODULE._selected_decisions([row]), [])
        row["selection_authority"] = "FORMAL"
        row["formal_support"] = "1"
        self.assertEqual(MODULE._selected_decisions([row]), [row])

    def test_v4_risk_selected_requires_accepted_geometry_commit(self):
        row = {
            "schema_version": "p4_forward_route_decision_v4",
            "stage": "forward_decision",
            "action": "RISK_SELECTED",
            "selection_authority": "FORMAL",
            "formal_support": "1",
            "selected_candidate_id": "2",
            "candidate_count": "2",
            "geometry_id": "planning-lattice-v2",
            "alert_limit_policy_id": "fixed_hal20_val40_v1",
            "occupancy_generation": "3",
            "risk_generation": "5",
            "geometry_commit_verdict": "NEW_ROUTE_COLLISION",
        }
        self.assertEqual(MODULE._selected_decisions([row]), [])
        row["geometry_commit_verdict"] = "CLEAR_AFTER_UPDATE"
        self.assertEqual(MODULE._selected_decisions([row]), [row])

    def test_v3_forward_timing_missing_fails_closed(self):
        result = MODULE.analyze_stage_records(
            "p4", [], [{
                "schema_version": "p4_forward_route_decision_v3",
                "stage": "forward_decision",
                "action": "DEFER_RISK_SELECTION",
            }], [], [], [], [])
        self.assertIn("p4_forward_timing_missing", result["failures"])
        self.assertIn(
            "p4_configuration_space_timing_missing", result["failures"])

    def test_v4_geometry_commit_budget_and_generation_only_hold_are_gated(self):
        row = {
            "schema_version": "p4_forward_route_decision_v4",
            "stage": "forward_decision",
            "action": "DEFER_RISK_SELECTION",
            "compute_latency_ms": "20.0",
            "configuration_space_prepare_ms": "1.0",
            "geometry_commit_verdict": "CLEAR_AFTER_UPDATE",
            "geometry_commit_latency_ms": "10.5",
            "reason": "live_occupancy_generation_changed_before_decision_reuse",
        }

        result = MODULE.analyze_stage_records(
            "p4", [], [row], [], [], [], [])

        self.assertIn("p4_geometry_commit_budget_exceeded", result["failures"])
        self.assertIn("p4_generation_only_hold_detected", result["failures"])
        self.assertEqual(result["generation_only_hold_count"], 1)

    def test_v4_final_commit_budget_is_not_hidden_by_stage_filter(self):
        forward = {
            "schema_version": "p4_forward_route_decision_v4",
            "stage": "forward_decision",
            "action": "DEFER_RISK_SELECTION",
            "compute_latency_ms": "20.0",
            "configuration_space_prepare_ms": "1.0",
            "geometry_commit_verdict": "CLEAR_UNCHANGED",
            "geometry_commit_latency_ms": "2.0",
            "reason": "route_clear_no_semantic_change",
        }
        final = dict(forward)
        final["stage"] = "final_bspline_before_p5"
        final["geometry_commit_verdict"] = "COMPUTE_BUDGET_EXCEEDED"
        final["geometry_commit_latency_ms"] = "10.25"

        result = MODULE.analyze_stage_records(
            "p4", [], [forward, final], [], [], [], [])

        self.assertIn("p4_geometry_commit_budget_exceeded", result["failures"])
        self.assertEqual(result["p4_geometry_commit_max_latency_ms"], 10.25)

    def test_v4_invalid_path_rejection_is_in_commit_timing_summary(self):
        row = {
            "schema_version": "p4_forward_route_decision_v4",
            "stage": "final_bspline_before_p5_geometry_commit_rejected",
            "action": "DEFER_RISK_SELECTION",
            "geometry_commit_verdict": "INVALID_PATH",
            "geometry_commit_latency_ms": "10.1",
            "geometry_commit_reason":
                "optimized_bspline_left_committed_guide_corridor",
        }

        result = MODULE.analyze_stage_records(
            "p4", [], [row], [], [], [], [])

        self.assertIn("p4_geometry_commit_budget_exceeded", result["failures"])
        self.assertEqual(result["p4_geometry_commit_max_latency_ms"], 10.1)

    def test_v4_clear_after_update_records_commit_latency(self):
        row = {
            "schema_version": "p4_forward_route_decision_v4",
            "stage": "forward_decision",
            "action": "DEFER_RISK_SELECTION",
            "compute_latency_ms": "20.0",
            "configuration_space_prepare_ms": "1.0",
            "geometry_commit_verdict": "CLEAR_AFTER_UPDATE",
            "geometry_commit_latency_ms": "2.5",
            "reason": "route_clear_after_map_update",
        }

        result = MODULE.analyze_stage_records(
            "p4", [], [row], [], [], [], [])

        self.assertNotIn("p4_geometry_commit_budget_exceeded",
                         result["failures"])
        self.assertNotIn("p4_generation_only_hold_detected",
                         result["failures"])
        self.assertEqual(result["p4_geometry_commit_max_latency_ms"], 2.5)

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

    @staticmethod
    def identity_evidence(generation_ids):
        health_rows = []
        decisions = []
        lineage = []
        for index, generation_id in enumerate(generation_ids):
            occupancy_generation = 100 + generation_id
            occupancy_stamp_s = 20.0 + generation_id
            decision = selected_decision()
            decision.update({
                "planning_attempt_id": str(10 + index),
                "collision_segment_id": str(20 + index),
                "request_hash": f"request-{generation_id}",
                "snapshot_generation_id": str(generation_id),
                "snapshot_config_hash": "config-v2",
                "source_identity_hash": f"sources-{generation_id}",
                "occupancy_epoch": str(occupancy_generation),
                "geometry_id": "planning-lattice-v2",
                "occupancy_stamp_s": str(occupancy_stamp_s),
            })
            fork = MODULE.forest_scene_contract()["forks"][index % 4]
            segment_midpoint_x = (
                float(fork["x_min_m"]) + 0.5 * float(fork["length_m"]))
            decision.update({
                "segment_start_x": str(segment_midpoint_x - 0.5),
                "segment_start_y": "0.0",
                "segment_start_z": "1.5",
                "segment_end_x": str(segment_midpoint_x + 0.5),
                "segment_end_y": "0.0",
                "segment_end_z": "1.5",
            })
            health_row = forest_v2_healthy(generation_id, 100.0 + index)
            health_row["payload"].update({
                "snapshot_config_hash": decision["snapshot_config_hash"],
                "source_identity_hash": decision["source_identity_hash"],
                "geometry_id": decision["geometry_id"],
                "alert_limit_policy_id": "fixed_hal20_val40_v1",
                "alert_limit_h_m": 20.0,
                "alert_limit_v_m": 40.0,
                "source_occupancy_generation": occupancy_generation,
                "source_occupancy_stamp_s": occupancy_stamp_s,
                "source_prior_generation": 200 + generation_id,
                "source_prior_stamp_s": occupancy_stamp_s - 0.1,
                "source_gnss_generation": 300 + generation_id,
                "source_gnss_stamp_s": occupancy_stamp_s - 0.2,
                "source_lidar_generation": 400 + generation_id,
                "source_lidar_stamp_s": occupancy_stamp_s - 0.3,
            })
            health_rows.append(health_row)
            decisions.append(decision)
            lineage.extend(lineage_for(
                decision, trajectory_id=500 + index,
                start_ns=600 + index))
        return health_rows, decisions, lineage

    def test_forest_risk_gate_requires_all_four_real_low_risk_arms(self):
        health_rows, decisions, lineage = self.identity_evidence([7, 7, 7, 7])
        passed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.8)],
            health_rows, decisions, lineage)
        self.assertEqual(passed["result"], "PASS")
        failed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.95)],
            health_rows, decisions, lineage)
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("forest_risk_contrast_missing", failed["failures"])

    def test_forest_risk_gate_does_not_reuse_one_decision_for_four_forks(self):
        health_rows, decisions, lineage = self.identity_evidence([7])
        failed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.8)],
            health_rows, decisions, lineage)

        self.assertEqual(failed["result"], "FAIL")
        self.assertEqual(failed["passing_generation_ids"], {"0": 7})
        self.assertIn(
            "forest_risk_identity_lineage_missing", failed["failures"])

    def test_forest_risk_gate_rejects_contrast_without_composite_lineage(self):
        failed = MODULE.analyze_forest_risk(
            [self.forest_generation(low_multiplier=0.8)])
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn(
            "forest_risk_identity_lineage_missing", failed["failures"])
        self.assertEqual(failed["contrast_generation_ids"], {
            "0": 7, "1": 7, "2": 7, "3": 7,
        })
        self.assertEqual(failed["fork_evidence"], {})

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

        health_rows, decisions, lineage = self.identity_evidence(
            [20, 21, 22, 23])
        passed = MODULE.analyze_forest_risk(
            records, health_rows, decisions, lineage)

        self.assertEqual(passed["result"], "PASS")
        self.assertEqual(passed["passing_generation_ids"], {
            "0": 20, "1": 21, "2": 22, "3": 23,
        })
        self.assertEqual(passed["missing_forks"], [])
        self.assertEqual(set(passed["fork_evidence"]), {"0", "1", "2", "3"})
        self.assertTrue(all(
            evidence["geometry_id"] == "planning-lattice-v2"
            and "normal_publish_authorized" in evidence["lineage_stages"]
            for evidence in passed["fork_evidence"].values()))

    def test_forest_risk_gate_accepts_forward_v2_decision_lineage(self):
        records = []
        for selected_index in range(4):
            record = self.forest_generation(low_multiplier=0.8)
            record["payload"]["generation_id"] = 30 + selected_index
            for fork in record["payload"]["forks"]:
                if fork["fork_index"] != selected_index:
                    for arm in (fork["low"], fork["high"]):
                        arm["sample_count"] = 0
                        arm["valid_count"] = 0
            records.append(record)

        health_rows, legacy_decisions, _ = self.identity_evidence(
            [30, 31, 32, 33])
        decisions = []
        lineage = []
        for index, legacy in enumerate(legacy_decisions):
            event_id = str(900 + index)
            decision = {
                "schema_version": "p4_forward_route_decision_v2",
                "stage": "forward_decision",
                "decision_event_id": event_id,
                "planning_attempt_id": legacy["planning_attempt_id"],
                "action": "RISK_SELECTED",
                "selected_candidate_id": "2",
                "candidate_count": "2",
                "risk_generation": legacy["snapshot_generation_id"],
                "snapshot_config_hash": legacy["snapshot_config_hash"],
                "source_identity_hash": legacy["source_identity_hash"],
                "geometry_id": legacy["geometry_id"],
                "alert_limit_policy_id": "fixed_hal20_val40_v1",
                "occupancy_generation": legacy["occupancy_epoch"],
                "occupancy_stamp_s": legacy["occupancy_stamp_s"],
                "request_x": legacy["segment_start_x"],
                "request_y": legacy["segment_start_y"],
                "request_z": legacy["segment_start_z"],
                "anchor_x": legacy["segment_end_x"],
                "anchor_y": legacy["segment_end_y"],
                "anchor_z": legacy["segment_end_z"],
            }
            decisions.append(decision)
            lineage.extend({
                **decision,
                "stage": stage,
                "trajectory_id": str(700 + index),
                "trajectory_start_ns": str(800 + index),
                "control_points_hash": f"control-{index}",
            } for stage in (
                "final_bspline_before_p5",
                "p5_final_pass_before_publish",
                "normal_publish_authorized",
                "p5_runtime_committed",
            ))

        result = MODULE.analyze_forest_risk(
            records, health_rows, decisions, lineage)

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(set(result["fork_evidence"]), {"0", "1", "2", "3"})
        self.assertTrue(all(
            evidence["decision_event_id"] is not None
            for evidence in result["fork_evidence"].values()))

    def test_forest_risk_gate_rejects_cross_fork_config_change(self):
        records = []
        for selected_index in range(4):
            record = self.forest_generation(low_multiplier=0.8)
            record["payload"]["generation_id"] = 20 + selected_index
            for fork in record["payload"]["forks"]:
                if fork["fork_index"] != selected_index:
                    fork["low"]["sample_count"] = 0
                    fork["low"]["valid_count"] = 0
                    fork["high"]["sample_count"] = 0
                    fork["high"]["valid_count"] = 0
            records.append(record)
        health_rows, decisions, lineage = self.identity_evidence(
            [20, 21, 22, 23])
        health_rows[-1]["payload"]["snapshot_config_hash"] = "changed"
        decisions[-1]["snapshot_config_hash"] = "changed"
        for row in lineage[-3:]:
            row["snapshot_config_hash"] = "changed"

        failed = MODULE.analyze_forest_risk(
            records, health_rows, decisions, lineage)

        self.assertEqual(failed["result"], "FAIL")
        self.assertIn(
            "forest_risk_cross_fork_identity_mismatch", failed["failures"])
        self.assertFalse(failed["cross_fork_identity"]["consistent"])

    def test_forest_risk_gate_rejects_cross_fork_alert_limit_change(self):
        records = []
        for selected_index in range(4):
            record = self.forest_generation(low_multiplier=0.8)
            record["payload"]["generation_id"] = 20 + selected_index
            for fork in record["payload"]["forks"]:
                if fork["fork_index"] != selected_index:
                    fork["low"]["sample_count"] = 0
                    fork["low"]["valid_count"] = 0
                    fork["high"]["sample_count"] = 0
                    fork["high"]["valid_count"] = 0
            records.append(record)
        health_rows, decisions, lineage = self.identity_evidence(
            [20, 21, 22, 23])
        health_rows[-1]["payload"]["alert_limit_v_m"] = 45.0

        failed = MODULE.analyze_forest_risk(
            records, health_rows, decisions, lineage)

        self.assertEqual(failed["result"], "FAIL")
        self.assertIn(
            "forest_risk_cross_fork_identity_mismatch", failed["failures"])

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

        rows = [forest_v2_healthy(index + 1, 100.1 + index * 0.5)
                for index in range(31)]
        summary = MODULE.analyze_p0(rows, 85.2)
        self.assertEqual(summary["result"], "PASS")
        self.assertAlmostEqual(summary["first_healthy_delay_s"], 14.9)

    def test_p0_summary_uses_completed_identity_not_in_progress_tail(self):
        rows = [forest_v2_healthy(index + 1, index * 0.5)
                for index in range(30)]
        in_progress = forest_v2_healthy(30, 15.0)
        in_progress["payload"].update({
            "refresh_evidence_state": "IN_PROGRESS",
            "result_generation_id": 0,
            "snapshot_config_hash": "",
            "source_identity_hash": "transient-empty-source",
            "source_occupancy_generation": 0,
        })
        rows.append(in_progress)

        summary = MODULE.analyze_p0(rows)

        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["snapshot_identity"]["generation_id"], 30)
        self.assertEqual(
            summary["source_identity"]["occupancy_generation"], 130)

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
        rows = [forest_v2_healthy(index + 1, float(index))
                for index in range(16)]
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")
        rows[8]["payload"]["stale"] = True
        rows[8]["payload"]["reason"] = "occupancy_stale"
        failed = MODULE.analyze_p0(rows)
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", failed["failures"])

    def test_p0_retains_a_fresh_grid_across_transient_background_failure(self):
        rows = [forest_v2_healthy(index + 1, float(index))
                for index in range(16)]
        rows[8]["payload"].update({
            "reason": "predictor_spatial_source_changed",
            "refresh_evidence_state": "COMPLETED_FAILURE",
        })
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "PASS")

        rows[8]["payload"]["stale"] = True
        failed = MODULE.analyze_p0(rows)
        self.assertEqual(failed["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", failed["failures"])

    def test_p0_window_includes_first_observation_past_boundary(self):
        rows = [forest_v2_healthy(index + 1, index * 0.47)
                for index in range(34)]
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "PASS")
        self.assertGreaterEqual(summary["window_span_s"], 15.0)

    def test_p0_does_not_restart_after_first_healthy_generation(self):
        rows = [forest_v2_healthy(1, 0.0)]
        rows.append(forest_v2_healthy(1, 0.5))
        rows[-1]["payload"]["reason"] = "occupancy_stale"
        rows.extend(forest_v2_healthy(index + 2, 1.0 + index * 0.5)
                    for index in range(31))
        summary = MODULE.analyze_p0(rows)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p0_health_not_continuous", summary["failures"])

    def test_p0_ignores_unhealthy_startup_before_first_generation(self):
        rows = [forest_v2_healthy(0, 0.0)]
        rows[0]["payload"].update({
            "ready": False, "stale": True, "reason": "startup",
        })
        rows.extend(forest_v2_healthy(index + 1, 1.0 + index * 0.5)
                    for index in range(31))
        self.assertEqual(MODULE.analyze_p0(rows)["result"], "PASS")

    def test_p4_requires_selected_lineage_and_stable_publication(self):
        summary = MODULE.analyze_stage_records(
            "p4", [forest_v2_healthy(index + 1, float(index))
                   for index in range(16)],
            decisions=[], lineage=[], bsplines=[], p5_status=[], poscmd_times=[])
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("p4_risk_selected_missing", summary["failures"])
        self.assertIn("stable_bspline_missing", summary["failures"])

    @staticmethod
    def limited_prefix_fixture():
        start_ns = 12_000_000_000
        spline_payload = {
            "trajectory_id": 31, "start_time_ns": start_ns,
            "control_points_xyz": [[0.0, 0.0, 1.0],
                                   [0.5, 0.0, 1.0],
                                   [1.0, 0.0, 1.0]],
            "knots": [-1.0, 0.0, 1.0, 2.0, 3.0],
        }
        control_hash, knot_hash = MODULE._captured_bspline_hashes(
            spline_payload)
        lineage = [{
            "schema_version": "p4_forward_route_decision_v13",
            "stage": "normal_publish_authorized",
            "action": "DEFER_RISK_SELECTION",
            "deferred_motion_mode": "COMMON_PREFIX",
            "reason": "safe_limited_common_prefix",
            "trajectory_id": "31",
            "trajectory_start_ns": str(start_ns),
            "control_points_hash": control_hash,
            "knot_vector_hash": knot_hash,
            "approved_endpoint_x": "1.0",
            "approved_endpoint_y": "0.0",
            "approved_endpoint_z": "1.0",
            "trajectory_duration_s": "2.0",
            "risk_generation": "8",
            "occupancy_generation": "18",
        }]
        bsplines = [{
            "receive_steady_s": 12.0,
            "payload": spline_payload,
        }]
        poscmd = []
        odom = []
        for index in range(31):
            alpha = min(1.0, index / 20.0)
            stamp = 12.0 + index * 0.1
            poscmd.append({
                "receive_steady_s": stamp,
                "payload": {
                    "trajectory_id": 31,
                    "stamp_s": stamp,
                    "position_xyz": [alpha, 0.0, 1.0],
                    "velocity_xyz": ([0.5, 0.0, 0.0] if alpha < 1.0
                                     else [0.0, 0.0, 0.0]),
                    "acceleration_xyz": [0.0, 0.0, 0.0],
                },
            })
            odom.append({
                "receive_steady_s": stamp,
                "payload": {
                    "stamp_s": stamp,
                    "position_m": [alpha, 0.0, 1.0],
                    "velocity_mps": ([0.5, 0.0, 0.0] if alpha < 1.0
                                     else [0.0, 0.0, 0.0]),
                },
            })
        events = [{
            "schema_version": "p4_execution_event_v1",
            "event": "AUTHORIZED",
            "authority": "LIMITED_PREFIX",
            "trajectory_id": "31",
            "trajectory_start_ns": str(start_ns),
            "control_points_hash": control_hash,
            "knot_vector_hash": knot_hash,
            "certificate_risk_generation": "8",
            "current_risk_generation": "8",
            "allowed": "1",
            "reason": "normal_publish_authorized",
            "direct_batch_duration_ms": "12.0",
            "stamp_s": "12.0",
        }, {
            "schema_version": "p4_execution_event_v1",
            "event": "ENDPOINT_HOLD",
            "authority": "LIMITED_PREFIX",
            "trajectory_id": "31",
            "trajectory_start_ns": str(start_ns),
            "control_points_hash": control_hash,
            "knot_vector_hash": knot_hash,
            "certificate_risk_generation": "8",
            "current_risk_generation": "10",
            "allowed": "1",
            "endpoint_reached": "1",
            "reason": "approved_endpoint_reached",
            "direct_batch_duration_ms": "18.0",
            "stamp_s": "14.2",
        }]
        return lineage, bsplines, poscmd, odom, events

    def test_limited_prefix_accepts_real_motion_and_endpoint_hold(self):
        args = self.limited_prefix_fixture()
        summary = MODULE.analyze_limited_prefix_records(*args)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(
            summary["limited_prefix_outcome"],
            "LIMITED_PREFIX_EXECUTED_TO_ENDPOINT")

    def test_limited_prefix_reports_confirmation_and_reauthorization_stats(
            self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        lineage[0]["actual_curve_certification_status"] = "SAFE"
        events[0]["execution_mode"] = "CONTROLLED_DEGRADED_EXECUTION"
        armed = dict(events[0])
        armed.update({
            "event": "MARGINAL_UNSAFE_ARMED",
            "risk_confirmation_state": "MARGINAL_UNSAFE_ARMED",
            "stamp_s": "12.4",
        })
        recovered = dict(events[0])
        recovered.update({
            "event": "MARGINAL_UNSAFE_RECOVERED",
            "risk_confirmation_state": "SAFE",
            "stamp_s": "12.5",
        })
        armed["guard_braking_preschedule_requested"] = "1"
        cancel_requested = dict(events[0])
        cancel_requested.update({
            "event": "FAILSAFE_BRAKING_CANCEL_REQUESTED",
            "guard_braking_cancel_requested": "1",
            "stamp_s": "12.45",
        })
        recovered["event"] = "FAILSAFE_BRAKING_CANCELED_RECOVERED"
        reauthorized = {
            "event": "PREPARED_SUCCESSOR_REAUTHORIZED",
            "risk_confirmation_state": "SAFE",
        }
        events.extend([armed, cancel_requested, recovered, reauthorized])
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["marginal_unsafe_armed_count"], 1)
        self.assertEqual(summary["marginal_unsafe_recovered_count"], 0)
        self.assertEqual(summary["successor_reauthorization_count"], 1)
        self.assertGreaterEqual(
            summary["execution_mode_counts"].get(
                "CONTROLLED_DEGRADED_EXECUTION", 0), 1)
        self.assertTrue(summary["controlled_degraded_motion_proven"])
        self.assertEqual(summary["guard_prequeue_count"], 1)
        self.assertEqual(summary["guard_cancel_request_count"], 1)
        self.assertEqual(summary["guard_cancel_ack_count"], 1)
        self.assertEqual(
            summary["actual_curve_certification_status_counts"],
            {"SAFE": 1})

    def test_limited_prefix_satellite_median_uses_pointwise_distribution(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        first = lineage[0]
        first.update({
            "actual_curve_core_policy": "braking_window_pointwise",
            "actual_curve_window_layout_hash": "layout-a",
            "actual_curve_certification_status": "SAFE",
            "actual_curve_first_failure_index": "0",
            "actual_curve_total_ms": "10",
            "actual_curve_window_count": "2",
            "actual_curve_transition_count": "1",
            "actual_curve_point_sat_min": "2",
            "actual_curve_point_sat_median": "8",
            "actual_curve_point_sat_max": "8",
            "actual_curve_window_point_satellite_sets_hashes":
                "1:2:41/2:8:42",
        })
        second = dict(first)
        second.update({
            "stage": "actual_curve_certified",
            "execution_snapshot_id": "9",
            "trajectory_id": "32",
            "trajectory_start_ns": "13000000000",
            "control_points_hash": "other-control-points",
            "actual_curve_window_layout_hash": "layout-b",
            "actual_curve_point_sat_min": "4",
            "actual_curve_point_sat_median": "6",
            "actual_curve_point_sat_max": "6",
            "actual_curve_window_point_satellite_sets_hashes":
                "1:4:51/2:6:52",
        })
        lineage.append(second)

        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)

        self.assertEqual(summary["braking_window_satellite_median"], 7.0)

    def test_limited_prefix_accepts_evidenced_new_generation_revoke(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        events[-1].update({
            "event": "RISK_REVOKED", "allowed": "0",
            "endpoint_reached": "0",
            "reason": "runtime_known_future_integrity_unsafe",
            "certificate_risk_generation": "8",
            "current_risk_generation": "9",
            "violation_hpl_m": "21.0", "violation_vpl_m": "35.0",
            "alert_limit_h_m": "20.0", "alert_limit_v_m": "40.0",
            "violation_query_time_s": "14.5",
        })
        # The old trajectory stops being commanded immediately after revoke.
        poscmd[:] = [row for row in poscmd
                     if row["receive_steady_s"] <= 14.2]
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(
            summary["limited_prefix_outcome"],
            "LIMITED_PREFIX_EXECUTED_THEN_RISK_REVOKED")

    def test_limited_prefix_accepts_bound_failsafe_braking_stop(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        braking_payload = {
            "trajectory_id": 32, "start_time_ns": 13_000_000_000,
            "control_points_xyz": [[0.5, 0.0, 1.0],
                                   [0.75, 0.0, 1.0],
                                   [1.0, 0.0, 1.0]],
            "knots": [-1.0, 0.0, 1.0, 2.0, 3.0],
        }
        braking_hash, braking_knot_hash = \
            MODULE._captured_bspline_hashes(braking_payload)
        bsplines.append({"receive_steady_s": 13.0,
                         "payload": braking_payload})
        poscmd.append({
            "receive_steady_s": 13.1,
            "payload": {"trajectory_id": 32, "stamp_s": 13.1,
                        "position_xyz": [0.55, 0.0, 1.0],
                        "velocity_xyz": [0.4, 0.0, 0.0],
                        "acceleration_xyz": [0.0, 0.0, 0.0]},
        })
        for stamp_s in (14.0, 14.4, 14.8):
            poscmd.append({
                "receive_steady_s": stamp_s,
                "payload": {"trajectory_id": 32, "stamp_s": stamp_s,
                            "position_xyz": [1.0, 0.0, 1.0],
                            "velocity_xyz": [0.0, 0.0, 0.0],
                            "acceleration_xyz": [0.0, 0.0, 0.0]},
            })
        events[-1].update({
            "event": "FAILSAFE_BRAKED_TO_STOP",
            "authority": "LIMITED_PREFIX_BRAKING",
            "parent_trajectory_id": "31",
            "parent_trajectory_start_ns": "12000000000",
            "trajectory_id": "32",
            "trajectory_start_ns": "13000000000",
            "control_points_hash": braking_hash,
            "knot_vector_hash": braking_knot_hash,
            "allowed": "1", "endpoint_reached": "1",
            "approved_endpoint_x": "1.0",
            "approved_endpoint_y": "0.0",
            "approved_endpoint_z": "1.0",
            "stamp_s": "14.0",
        })
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(
            summary["limited_prefix_outcome"],
            "LIMITED_PREFIX_EXECUTED_THEN_FAILSAFE_BRAKED_TO_STOP")

    def test_limited_prefix_rejects_braking_without_sustained_terminal_cmd(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        braking_payload = {
            "trajectory_id": 32, "start_time_ns": 13_000_000_000,
            "control_points_xyz": [[0.5, 0.0, 1.0],
                                   [0.75, 0.0, 1.0],
                                   [1.0, 0.0, 1.0]],
            "knots": [-1.0, 0.0, 1.0, 2.0, 3.0],
        }
        braking_hash, braking_knot_hash = \
            MODULE._captured_bspline_hashes(braking_payload)
        bsplines.append({"receive_steady_s": 13.0,
                         "payload": braking_payload})
        poscmd.append({
            "receive_steady_s": 13.1,
            "payload": {"trajectory_id": 32, "stamp_s": 13.1,
                        "position_xyz": [0.55, 0.0, 1.0],
                        "velocity_xyz": [0.4, 0.0, 0.0],
                        "acceleration_xyz": [0.0, 0.0, 0.0]},
        })
        events[-1].update({
            "event": "FAILSAFE_BRAKED_TO_STOP",
            "authority": "LIMITED_PREFIX_BRAKING",
            "parent_trajectory_id": "31",
            "parent_trajectory_start_ns": "12000000000",
            "trajectory_id": "32",
            "trajectory_start_ns": "13000000000",
            "control_points_hash": braking_hash,
            "knot_vector_hash": braking_knot_hash,
            "allowed": "1", "endpoint_reached": "1",
            "approved_endpoint_x": "1.0",
            "approved_endpoint_y": "0.0",
            "approved_endpoint_z": "1.0",
            "stamp_s": "14.0",
        })
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertFalse(summary["failsafe_braked_to_stop_proven"])

    def test_limited_prefix_rejects_slow_direct_execution_check(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        events[-1]["direct_batch_duration_ms"] = "150.0"
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn(
            "limited_prefix_direct_batch_p95_exceeded",
            summary["failures"])
        self.assertEqual(summary["direct_batch_ms_p95"], 150.0)

    def test_limited_prefix_accepts_any_bound_execution_not_only_latest(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        events[-1].update({
            "event": "RISK_REVOKED", "allowed": "0",
            "endpoint_reached": "0",
            "reason": "runtime_known_future_integrity_unsafe",
            "certificate_risk_generation": "8",
            "current_risk_generation": "9",
            "violation_hpl_m": "21.0", "violation_vpl_m": "35.0",
            "alert_limit_h_m": "20.0", "alert_limit_v_m": "40.0",
            "violation_query_time_s": "14.5",
        })
        poscmd[:] = [row for row in poscmd
                     if row["receive_steady_s"] <= 14.2]
        later = dict(lineage[0])
        later.update({
            "trajectory_id": "32", "trajectory_start_ns": "15000000000",
            "control_points_hash": "cp32",
        })
        lineage.append(later)
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["limited_publish_count"], 2)
        self.assertEqual(summary["trajectory_identity"], [31, 12000000000])
        self.assertEqual(len(summary["limited_attempts"]), 2)

    def test_limited_prefix_rejects_identity_mismatch_and_command_only(self):
        lineage, bsplines, poscmd, _odom, events = \
            self.limited_prefix_fixture()
        bsplines[0]["payload"]["trajectory_id"] = 99
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, [], events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("limited_prefix_bspline_identity_missing",
                      summary["failures"])
        self.assertIn("limited_prefix_actual_motion_missing",
                      summary["failures"])

    def test_limited_prefix_does_not_accept_formal_route(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        lineage[0].update({
            "action": "RISK_SELECTED", "deferred_motion_mode": "NONE",
            "selection_applied": "1", "reason": "risk_selected",
        })
        events[0]["authority"] = "FORMAL_RISK_SELECTED"
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertEqual(summary["limited_prefix_outcome"],
                         "FORMAL_ROUTE_SELECTED")
        self.assertIn("limited_prefix_not_exercised", summary["failures"])

    def test_limited_prefix_rejects_unauthenticated_revoke(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        events[-1].update({
            "event": "RISK_REVOKED", "allowed": "0",
            "endpoint_reached": "0",
            "reason": "runtime_known_future_integrity_unsafe",
            "certificate_risk_generation": "8",
            "current_risk_generation": "8",
            "violation_hpl_m": "19", "violation_vpl_m": "39",
            "alert_limit_h_m": "20", "alert_limit_v_m": "40",
            "violation_query_time_s": "14.5",
        })
        poscmd[:] = [row for row in poscmd
                     if row["receive_steady_s"] <= 14.2]
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertFalse(summary["legal_risk_revoke_proven"])

    def test_limited_prefix_rejects_short_endpoint_observation_window(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        poscmd[:] = [row for row in poscmd
                     if row["receive_steady_s"] <= 14.4]
        odom[:] = [row for row in odom
                   if row["receive_steady_s"] <= 14.4]
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertFalse(summary["endpoint_hold_proven"])

    def test_limited_prefix_rejects_approved_endpoint_overrun(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        odom[-1]["payload"]["position_m"] = [1.25, 0.0, 1.0]
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("limited_prefix_approved_endpoint_overrun",
                      summary["failures"])

    def test_limited_prefix_parent_handoff_stops_at_bound_successor(self):
        lineage, bsplines, poscmd, odom, events = \
            self.limited_prefix_fixture()
        child = dict(lineage[0])
        child.update({
            "trajectory_id": "32",
            "trajectory_start_ns": "13000000000",
            "control_points_hash": "child-control-points",
            "knot_vector_hash": "child-knots",
            "approved_endpoint_x": "2.0",
        })
        lineage.append(child)
        events.append({
            "event": "AUTHORIZED",
            "authority": "LIMITED_PREFIX",
            "trajectory_id": "32",
            "trajectory_start_ns": "13000000000",
            "parent_trajectory_id": "31",
            "parent_trajectory_start_ns": "12000000000",
            "control_points_hash": "child-control-points",
            "knot_vector_hash": "child-knots",
            "allowed": "1",
            "reason": "normal_publish_authorized",
            "stamp_s": "13.0",
        })
        # This is child motion and may pass the old parent's endpoint.  It
        # must not be reported as an overrun of the authenticated parent.
        for row in odom:
            if row["payload"]["stamp_s"] > 13.0:
                row["payload"]["position_m"][0] += 1.0
        summary = MODULE.analyze_limited_prefix_records(
            lineage, bsplines, poscmd, odom, events)
        parent = summary["limited_attempts"][0]
        self.assertEqual(parent["result"], "PASS")
        self.assertEqual(
            parent["outcome"], "LIMITED_PREFIX_ROLLED_TO_SUCCESSOR")
        self.assertNotIn("limited_prefix_approved_endpoint_overrun",
                         parent["failures"])

    def test_execution_snapshot_attempt_metrics_separate_recovery_and_gap(self):
        records = [
            {"kind": "execution_snapshot_attempt", "payload": {
                "status": "PUBLISHED", "finish_ros_stamp_s": 10.0,
                "queue_delay_ms": 4.0, "build_duration_ms": 25.0,
                "publish_age_s": 0.08, "pending_overwrite_count": 1,
                "published_execution_snapshot_id": 41,
                "support_stamp_s": 10.0,
                "risk_grid_yield_count": 2,
                "risk_grid_yield_duration_ms": 12.0}},
            {"kind": "execution_snapshot_attempt", "payload": {
                "status": "PUBLISHED", "finish_ros_stamp_s": 10.2,
                "queue_delay_ms": 5.0, "build_duration_ms": 30.0,
                "publish_age_s": 0.09, "pending_overwrite_count": 1,
                "risk_grid_yield_count": 3,
                "risk_grid_yield_duration_ms": 18.0}},
            {"kind": "occupancy_input", "payload": {"stamp_s": 10.0}},
            {"kind": "occupancy_input", "payload": {"stamp_s": 10.1}},
        ]
        health = [{"payload": {"latency_primary_cause": "NONE"}}]
        events = [
            {"event": "FAILSAFE_BRAKING_SCHEDULED",
             "stamp_s": 10.15,
             "execution_snapshot_id": 41,
             "reason": "failsafe_braking_scheduled:runtime_local_map_"
                       "support_stale_or_invalid"},
            {"event": "FAILSAFE_BRAKING_SCHEDULED",
             "stamp_s": 10.15,
             "reason": "failsafe_braking_scheduled:runtime_corridor_"
                       "support_stale_or_invalid:EXPIRED"},
            {"event": "FAILSAFE_BRAKING_SCHEDULED",
             "stamp_s": 10.15,
             "reason": "failsafe_braking_scheduled:runtime_corridor_"
                       "support_stale_or_invalid:OUTSIDE_ENVELOPE"},
            {"event": "FAILSAFE_BRAKING_CANCELED_RECOVERED"},
        ]
        summary = MODULE.analyze_execution_snapshot_attempts(
            records, health, events)
        self.assertEqual(summary["attempt_count"], 2)
        self.assertAlmostEqual(summary["maximum_success_interval_s"], .2)
        self.assertEqual(summary["false_support_stale_braking_count"], 1)
        self.assertEqual(summary["local_map_support_stale_braking_count"], 1)
        self.assertEqual(summary["corridor_support_stale_braking_count"], 1)
        self.assertEqual(
            summary["corridor_support_outside_envelope_braking_count"], 1)
        self.assertEqual(summary["support_stale_braking_count"], 2)
        self.assertEqual(summary["recovery_cancel_count"], 1)
        self.assertEqual(summary["risk_grid_yield_count"], 3)

    def test_execution_snapshot_summary_decomposes_global_budget_braking(self):
        events = [
            {"event": "FAILSAFE_BRAKING_SCHEDULED",
             "reason": "failsafe_braking_scheduled:runtime_trajectory_"
                       "assurance_rejected:global_navigation_budget_"
                       "exceeded:safe",
             "global_budget_failure_causes": "CONTINUOUS_DURATION|"
                                               "EXCESS_INTEGRAL",
             "runtime_global_peak_ratio": "1.0011",
             "global_peak_ratio_limit": "1.05",
             "global_maximum_continuous_exceedance_s": "1.31",
             "global_continuous_exceedance_limit_s": "1.0",
             "runtime_global_exceedance_integral_ratio_s": "0.041",
             "global_exceedance_integral_limit_ratio_s": "0.025",
             "global_prior_episode_active": "1",
             "global_prior_episode_budget_exhausted": "0",
             "violation_x": "-14.2", "violation_y": "0.3",
             "violation_z": "1.5", "violation_query_time_s": "12.4",
             "violation_hpl_m": "8.1", "violation_vpl_m": "40.044",
             "alert_limit_h_m": "20.0", "alert_limit_v_m": "40.0"},
            {"event": "FAILSAFE_BRAKING_SCHEDULED",
             "reason": "failsafe_braking_scheduled:runtime_trajectory_"
                       "assurance_rejected:global_navigation_episode_"
                       "budget_exceeded:safe",
             "global_budget_failure_causes":
                 "PRIOR_EPISODE_EXHAUSTED",
             "global_prior_episode_active": "1",
             "global_prior_episode_budget_exhausted": "1"},
        ]

        summary = MODULE.analyze_execution_snapshot_attempts([], [], events)

        self.assertEqual(summary["global_budget_braking_count"], 2)
        self.assertEqual(
            summary["global_budget_failure_cause_counts"],
            {"CONTINUOUS_DURATION": 1, "EXCESS_INTEGRAL": 1,
             "PRIOR_EPISODE_EXHAUSTED": 1})
        self.assertEqual(
            summary["global_budget_braking_events"][0]["dominant_cause"],
            "EXCESS_INTEGRAL")
        self.assertEqual(
            summary["global_budget_braking_events"][0]["trigger_scope"],
            "ACCUMULATED_EXPOSURE")
        self.assertEqual(
            summary["global_budget_braking_events"][0]
                   ["first_unsafe_position_xyz"],
            [-14.2, 0.3, 1.5])
        self.assertAlmostEqual(
            summary["global_budget_braking_events"][0]
                   ["first_unsafe_vpl_m"], 40.044)
        self.assertEqual(
            summary["global_budget_braking_events"][1]["dominant_cause"],
            "PRIOR_EPISODE_EXHAUSTED")

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
            "full", [forest_v2_healthy(index + 1, float(index))
                     for index in range(16)],
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
            "full", [forest_v2_healthy(index + 1, float(index))
                     for index in range(16)],
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
            "full", [forest_v2_healthy(index + 1, float(index))
                     for index in range(16)],
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
            "full", [forest_v2_healthy(index + 1, float(index))
                     for index in range(16)],
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

    def test_runtime_window_evidence_requires_stable_layout_and_event_link(self):
        batches = [{
            "evidence_sequence_id": "1", "trajectory_id": "7",
            "trajectory_start_ns": "9", "window_layout_hash": "fixed",
            "active_window_count": "2", "result_window_count": "2",
            "point_count": "5", "complete": "1", "total_ms": "12.5",
        }, {
            "evidence_sequence_id": "2", "trajectory_id": "7",
            "trajectory_start_ns": "9", "window_layout_hash": "fixed",
            "active_window_count": "2", "result_window_count": "2",
            "point_count": "5", "complete": "1", "total_ms": "14.0",
        }]
        events = [{
            "event": "FAILSAFE_BRAKING_SCHEDULED",
            "gnss_core_policy": "braking_window_pointwise",
            "reason": "failsafe_braking_scheduled:"
                      "runtime_trajectory_assurance_rejected:unsafe",
            "runtime_window_evidence_sequence_id": "2",
        }]
        probes = [{"classification": "GNSS_EPOCH_OR_SET"}]
        windows = [
            {"evidence_sequence_id": sequence, "window_id": window,
             "point_satellite_sets_hash": "17", "point_count": "2",
             "maximum_hpl_over_hal": "0.5",
             "maximum_vpl_over_val": "0.4",
             "first_failure_index": "18446744073709551615",
             "complete": "1", "failure_reason": "NONE"}
            for sequence in ("1", "2") for window in ("10", "11")]
        summary = MODULE.analyze_runtime_window_evidence(
            batches, windows, probes, events)
        self.assertEqual(summary["result"], "PASS")
        self.assertEqual(summary["runtime_window_layout_change_count"], 0)
        self.assertEqual(summary[
            "fixed_layout_generation_classification_counts"],
            {"GNSS_EPOCH_OR_SET": 1})

        for event_name in (
                "MARGINAL_UNSAFE_ARMED", "MARGINAL_UNSAFE_RECOVERED",
                "FAILSAFE_BRAKING_CANCEL_REQUESTED"):
            incomplete_batches = [dict(row) for row in batches]
            incomplete_batches[1]["complete"] = "0"
            continuing_event = dict(events[0])
            continuing_event["event"] = event_name
            summary = MODULE.analyze_runtime_window_evidence(
                incomplete_batches, windows, probes, [continuing_event])
            self.assertEqual(summary["result"], "FAIL")
            self.assertIn("runtime_risk_decision_missing_window_evidence",
                          summary["failures"])

        batches[1]["window_layout_hash"] = "rebuilt"
        events[0]["runtime_window_evidence_sequence_id"] = "0"
        summary = MODULE.analyze_runtime_window_evidence(
            batches, [], [], events)
        self.assertEqual(summary["result"], "FAIL")
        self.assertIn("runtime_window_layout_changed_within_trajectory",
                      summary["failures"])
        self.assertIn("runtime_risk_decision_missing_window_evidence",
                      summary["failures"])

    def test_successor_preparation_reports_deadline_latency_and_typed_failure(self):
        rows = [{
            "successor_fast_path": "1",
            "reason": "successor_worker_pending",
            "successor_prepare_duration_ms": "nan",
            "successor_queue_delay_ms": "nan",
            "successor_failure": "NONE",
        }, {
            "successor_fast_path": "1",
            "reason": "successor_fast_path_ready",
            "successor_prepare_duration_ms": "420",
            "successor_queue_delay_ms": "12",
            "successor_required_progress_m": "0.17",
            "successor_actual_progress_m": "0.31",
            "successor_failure": "NONE",
        }, {
            "successor_fast_path": "1",
            "reason": "successor_deadline_missed",
            "successor_prepare_duration_ms": "810",
            "successor_queue_delay_ms": "8",
            "successor_failure": "DEADLINE_MISSED",
        }]
        summary = MODULE.analyze_successor_preparation(rows)
        self.assertEqual(summary["failures"], [
            "successor_deadline_missed",
            "successor_fast_path_p95_exceeded",
            "successor_prepare_wcet_exceeded",
        ])
        self.assertEqual(summary["request_count"], 3)
        self.assertEqual(summary["deadline_miss_count"], 1)
        self.assertEqual(summary["ordinary_rate_limited_count"], 0)
        self.assertEqual(summary["prepare_duration_ms_p95"], 810.0)
        self.assertEqual(summary["progress"], [{
            "required_m": 0.17, "actual_m": 0.31}])

    def test_successor_preparation_rejects_ordinary_rate_limit(self):
        summary = MODULE.analyze_successor_preparation([{
            "successor_fast_path": "0",
            "successor_latest_prepare_start_s": "12.5",
            "reason": "forward_decision_rate_limited",
            "successor_failure": "NONE",
        }])
        self.assertEqual(
            summary["failures"], ["successor_hit_ordinary_rate_limit"])

    def test_successor_preparation_reports_refinement_root_cause_and_cache(self):
        summary = MODULE.analyze_successor_preparation([{
            "successor_fast_path": "1",
            "stage": "forward_decision",
            "reason": "no_native_refined_candidate:target_suffix_blocked=1",
            "successor_failure": "CORRIDOR_INVALID",
            "refinement_diagnostics":
                "target_suffix_blocked:0:2.1:1:0:1:0.02:tree:1:0:0",
        }, {
            "successor_fast_path": "1",
            "stage": "successor_prepared_certified",
            "reason": "successor_fast_path_ready",
            "successor_failure": "NONE",
            "refinement_diagnostics":
                "success:18446744073709551615:3.0:nan:nan:nan:0.08::0:0:0",
        }])

        self.assertEqual(summary["prepared_certified_count"], 1)
        self.assertEqual(summary["refinement_failure_counts"], {
            "target_suffix_blocked": 1})

    def test_successor_preparation_reports_final_curve_failure(self):
        summary = MODULE.analyze_successor_preparation([{
            "successor_fast_path": "1",
            "stage": "successor_curve_preparation_failed",
            "reason": "successor_curve_preparation_failed:"
                      "terminal_bspline_refinement_collision_or_dynamics",
            "successor_failure": "COLLISION_CHANGED",
        }])

        self.assertIn(
            "successor_final_curve_collision_or_dynamics",
            summary["failures"])
        self.assertEqual(
            summary["failure_counts"].get("COLLISION_CHANGED"), 1)

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
