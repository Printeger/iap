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
from advisory_coordinates import checked_coordinates


def load_advisory_calibration(path):
    """Explicit frozen empirical parameters only; defaults are never promoted here."""
    if not path: return {}
    import hashlib
    import json
    target=Path(path)
    if not target.is_absolute(): raise ValueError("advisory calibration must use an absolute path")
    value=json.loads(target.read_text())
    unsigned={k:v for k,v in value.items() if k != "sha256"}
    actual=hashlib.sha256(json.dumps(unsigned,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if actual != value.get("sha256"): raise ValueError("advisory calibration checksum mismatch")
    required={"route_sha256","coordinates_sha256","degradation_schedule_sha256","map_seed"}
    if (value.get("identity")!="LIVE_CALIBRATION_CANDIDATE" or value.get("stage") not in ("noise","conversion") or
            value.get("scene")!="icra_dense_forest_four_fork_v2" or
            not required.issubset(value.get("contract",{})) or value["contract"]["map_seed"]!=41021 or
            not value.get("calibration_evidence")):
        raise ValueError("frozen real calibration provenance required")
    params=value["parameters"]
    allowed={"risk/gnss_noise_scale","risk/lidar_noise_scale","risk/K_H_adv","risk/K_V_adv"}
    import math
    if set(params)!=allowed or any(not math.isfinite(v) or v<=0 for v in params.values()):
        raise ValueError("only positive observation noise and common PL conversion parameters allowed")
    return params


def advisory_observation_schedule():
    return {"policy":"constant_for_entire_trial","conditions":{
        "normal":{"gnss_pseudorange_sigma_m":1.,"lidar_max_range_m":10.},
        "gnss_degraded":{"gnss_pseudorange_sigma_m":5.,"lidar_max_range_m":10.},
        "lidar_degraded":{"gnss_pseudorange_sigma_m":1.,"lidar_max_range_m":3.}}}


def load_advisory_trial(path, scenario):
    """Optional fixed-route experiment inputs; never bypass live curve checks."""
    if not path: return {}
    import hashlib
    import math
    target=Path(path)
    if not target.is_absolute(): raise ValueError("validation trial must use an absolute path")
    trial=json.loads(target.read_text())
    if (trial.get("schema")!="iap_advisory_validation_trial_v1" or
            trial.get("scene")!="icra_dense_forest_four_fork_v2" or trial.get("map_seed")!=41021 or
            trial.get("condition") not in ("normal","gnss_degraded","lidar_degraded") or
            trial.get("phase") not in ("calibration","validation","mission")):
        raise ValueError("canonical predeclared trial required")
    seeds=(1101,1102,1103) if trial["phase"]=="calibration" else (2101,2102,2103)
    if trial.get("seed") not in seeds: raise ValueError("trial seed outside predeclared split")
    route=trial["reference_route"]
    digest=lambda x:hashlib.sha256(json.dumps(x,sort_keys=True,allow_nan=False).encode()).hexdigest()
    if digest(route)!=trial.get("route_sha256"): raise ValueError("reference route checksum mismatch")
    points=route.get("waypoints",[])
    if (not 1<=len(points)<=50 or any(len(p)!=3 or any(not math.isfinite(float(x)) for x in p) for p in points) or
            route.get("speed_mps")!=scenario["max_velocity_mps"]):
        raise ValueError("fixed finite waypoint route at canonical speed required")
    for key in ("coordinates","physical_route_evidence"):
        resource=Path(trial[key])
        if not resource.is_absolute() or hashlib.sha256(resource.read_bytes()).hexdigest()!=trial[key+"_sha256"]:
            raise ValueError(key+" checksum mismatch")
    coordinates=json.loads(Path(trial["coordinates"]).read_text())
    checked_coordinates(coordinates)
    proof=json.loads(Path(trial["physical_route_evidence"]).read_text())
    if (proof.get("identity")!="REAL_REPLAY" or proof.get("physical_valid") is not True or
            proof.get("route_sha256")!=trial["route_sha256"] or not proof.get("prediction_input_identity")):
        raise ValueError("physical route proof from real frozen map required")
    schedule=advisory_observation_schedule()
    if trial.get("degradation_schedule")!=schedule or digest(schedule)!=trial.get("degradation_schedule_sha256"):
        raise ValueError("predeclared constant observation schedule required")
    trial["frame_verified"]=True
    return trial


def planner_parameters(scenario, capture_failure_map=False, advisory_posterior_prior=False, advisory_calibration="", advisory_guidance=True, advisory_trial=""):
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
        "planning/advisory_guidance_enabled": advisory_guidance,
        "risk/gnss_max_age_s": 2.0,
        "risk/use_posterior_prior": advisory_posterior_prior,
        "risk/source": {"lidar_only": "lidar", "gnss_only": "gnss"}.get(profile, "fusion"),
    }
    for i, axis in enumerate("xyz"):
        params[f"grid_map/map_size_{axis}"] = size[i]
        params[f"fsm/waypoint0_{axis}"] = goal[i]
    params.update(load_advisory_calibration(advisory_calibration))
    trial=load_advisory_trial(advisory_trial,scenario)
    if trial:
        if advisory_posterior_prior or (trial["phase"]!="mission" and advisory_guidance):
            raise ValueError("trial requires posterior OFF and calibration guidance OFF")
        params["fsm/flight_type"]=2
        params["fsm/waypoint_num"]=len(trial["reference_route"]["waypoints"])
        for i,point in enumerate(trial["reference_route"]["waypoints"]):
            for j,axis in enumerate("xyz"): params[f"fsm/waypoint{i}_{axis}"]=float(point[j])
    return params


def visualizer_parameters():
    return {
        "risk_viz/metric": "hpl",
        "risk_viz/z_mode": "follow",
        "risk_viz/hpl_min_m": 0.25,
        "risk_viz/hpl_max_m": 0.65,
        "risk_viz/vpl_min_m": 0.20,
        "risk_viz/vpl_max_m": 0.55,
        "risk_viz/surface_lifetime_s": 60.0,
        "risk_viz/surface_snapshot_step_m": 4.0,
    }


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
              ("/position_cmd", "/drone_0_planning/pos_cmd"),
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
                           "capture_failure_map", "false").lower() == "true",
                       context.launch_configurations.get(
                           "advisory_posterior_prior", "false").lower() == "true",
                       context.launch_configurations.get("advisory_calibration", ""),
                       context.launch_configurations.get("advisory_guidance", "true").lower()=="true",
                       context.launch_configurations.get("advisory_trial", ""))],
                   remappings=remaps)
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share / "launch/_includes/simulation_environment.launch.py")),
            launch_arguments={"scenario": name, "output_dir": str(run),
                              "advisory_trial":context.launch_configurations.get("advisory_trial", "")}.items()),
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
    if context.launch_configurations.get("start_grid_map_visualizer", "true").lower() == "true":
        actions.append(Node(package="ego_planner", executable="grid_map_visualizer",
                            name="grid_map_visualizer", output="screen",
                            parameters=[visualizer_parameters()],
                            remappings=[("odom_world", "/drone_0_visual_slam/odom")]))
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
