"""EGO baseline simulation with one GridMap and spatial advisory PL.

Environment and estimation retain their existing input contracts. Stage 1 does
not claim risk-aware route selection or the future execution checks.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction, SetEnvironmentVariable)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from profile_runtime import materialize_profile
from run_directory import register_config_snapshot


def planner_parameters(scenario, capture_failure_map=False):
    size = [float(v) for v in scenario["map_size"]]
    goal = [float(v) for v in scenario["goal"]]
    velocity = float(scenario["max_velocity_mps"])
    profile = scenario["integrity_profile"]
    params = {
        "fsm/flight_type": 1 if scenario.get("manual_goal", False) else 2,
        "fsm/thresh_replan_time": 1.0, "fsm/thresh_no_replan_meter": 1.0,
        "fsm/planning_horizon": 5.0, "fsm/planning_horizen_time": 3.0,
        "fsm/emergency_time": 1.0, "fsm/realworld_experiment": False,
        "fsm/fail_safe": True, "fsm/waypoint_num": 1,
        "grid_map/resolution": 0.1, "grid_map/origin_x": -size[0]/2,
        "grid_map/origin_y": -size[1]/2, "grid_map/origin_z": 0.0,
        "grid_map/local_update_range_x": 5.5, "grid_map/local_update_range_y": 5.5,
        "grid_map/local_update_range_z": 4.5, "grid_map/obstacles_inflation": 0.3,
        "grid_map/local_map_margin": 10, "grid_map/ground_height": 0.0,
        "grid_map/virtual_ceil_height": size[2] - 0.1,
        "grid_map/visualization_truncate_height": size[2],
        "grid_map/visualization_period_s": 1.0,
        "grid_map/pose_type": 2, "grid_map/frame_id": "map",
        "grid_map/registered_lidar_window_enabled": True,
        "grid_map/registered_frame_contract_id": "ego_grid_map_sim_v1",
        "grid_map/registered_current_topic": "/iap/local_map/current_frame",
        "grid_map/registered_delta_topic": "/iap/local_map/window_delta",
        "grid_map/registered_recovery_service": "/iap/local_map/get_active_window",
        "grid_map/registered_lidar_reference_frame_id": "iap_lidar_reference",
        "manager/max_vel": velocity, "manager/max_acc": 2.0,
        "manager/max_jerk": 4.0, "manager/control_points_distance": 0.4,
        "manager/feasibility_tolerance": 0.05, "manager/planning_horizon": 5.0,
        "manager/drone_id": 0,
        "optimization/lambda_smooth": 1.0, "optimization/lambda_collision": 0.5,
        "optimization/lambda_feasibility": 0.1, "optimization/lambda_fitness": 1.0,
        "optimization/dist0": 0.5, "optimization/swarm_clearance": 0.5,
        "optimization/max_vel": velocity, "optimization/max_acc": 2.0,
        "bspline/limit_vel": velocity, "bspline/limit_acc": 2.0,
        "bspline/limit_ratio": 1.1, "risk/validity_s": 0.5,
        # Experimental full-stack defaults; four-fork is the standard test
        # scene. These are preferences/proxy budgets, not certified PL.
        "planning/advisory_hpl_budget_m": 0.55,
        "planning/advisory_vpl_budget_m": 0.60,
        "planning/advisory_hpl_reserve_m": 0.10,
        "planning/advisory_vpl_reserve_m": 0.10,
        "planning/advisory_unknown_multiplier": 1.5,
        "planning/advisory_stale_soft_s": 1.0,
        "planning/body_radius_m": 0.35,
        "planning/tracking_reserve_m": 0.10,
        "planning/current_motion_budget_m": 0.55,
        "planning/current_motion_max_age_s": 0.5,
        "planning/environment_max_age_s": 0.5,
        "planning/tracking_error_limit_m": 0.30,
        "planning/capture_failure_map": capture_failure_map,
        "risk/gnss_max_age_s": 2.0,
        "risk/source": {"lidar_only": "lidar", "gnss_only": "gnss"}.get(profile, "fusion"),
        "risk_viz/enabled": True,
        "risk_viz/metric": "hpl",
        "risk_viz/z_mode": "follow",
        "risk_viz/hpl_min_m": 0.25,
        "risk_viz/hpl_max_m": 0.65,
        "risk_viz/vpl_min_m": 0.20,
        "risk_viz/vpl_max_m": 0.55,
        "risk_viz/surface_lifetime_s": 60.0,
        "risk_viz/surface_snapshot_step_m": 4.0,
    }
    for i, axis in enumerate("xyz"):
        params[f"grid_map/map_size_{axis}"] = size[i]
        params[f"fsm/waypoint0_{axis}"] = goal[i]
    return params


def _setup(context):
    share = Path(get_package_share_directory("iap"))
    name = context.launch_configurations["scenario"]
    scenario = json.loads((share / "config/scenarios/catalog.json").read_text())[name]
    scenario = {**scenario, "manual_goal": name == "manual"}
    run = Path(os.environ["IAP_RUN_DIR"])
    runtime, _ = materialize_profile(
        source_config_dir=str(share / "config/profiles/full_stack_flight"),
        output_dir=str(run), contract="full_stack",
        integrity_profile=scenario["integrity_profile"], simulation_scenario=scenario)
    register_config_snapshot(run, Path(runtime))
    remaps = [("odom_world", "/drone_0_visual_slam/odom"),
              ("grid_map/odom", "/drone_0_visual_slam/odom"),
              ("planning/bspline", "/drone_0_planning/bspline"),
              ("risk/integrity", "/iap/integrity")]
    for local, remote in [("range", "range_meas"), ("ephem", "ephem"),
                          ("glo_ephem", "glo_ephem"), ("receiver_lla", "receiver_lla"),
                          ("iono", "iono_params")]:
        remaps.append((f"risk/{local}", f"/ublox_driver/{remote}"))
    planner = Node(package="ego_planner", executable="ego_planner_node",
                   name="drone_0_ego_planner_node", output="screen",
                   parameters=[planner_parameters(
                       scenario,
                       context.launch_configurations.get(
                           "capture_failure_map", "false").lower() == "true")],
                   remappings=remaps)
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / "launch/_includes/simulation_environment.launch.py")),
            launch_arguments={"scenario": name, "output_dir": str(run)}.items()),
        Node(package="iap", executable="iap_rosnode", name="glio_integrity", output="screen",
             parameters=[{"config_path": runtime, "imu_topic": "/sim/drone_0/imu_iap",
                          "points_topic": "/sim/drone_0/lidar_body"}]),
        TimerAction(period=float(context.launch_configurations.get("planner_start_delay_s", "10")),
                    actions=[planner]),
        Node(package="ego_planner", executable="traj_server", name="drone_0_traj_server",
             output="screen", parameters=[{"frame_id": "map", "traj_server/time_forward": 1.0}],
             remappings=[("planning/bspline", "/drone_0_planning/bspline"),
                         ("/position_cmd", "/drone_0_planning/pos_cmd")]),
    ]
    if context.launch_configurations.get("start_rviz", "true").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", name="iap_rviz",
                            arguments=["-d", str(share / "config/sim_ego/grid_map_stage1.rviz")]))
    duration = float(context.launch_configurations.get("run_duration_s", "0"))
    if duration > 0:
        actions.append(TimerAction(period=duration, actions=[EmitEvent(event=Shutdown(reason="run duration reached"))]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("iap"))
    return LaunchDescription([
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", str(share / "config/sim_ego/fastdds_udp_only.xml")),
        OpaqueFunction(function=_setup),
    ])
