"""Canonical fail-closed real-flight profile for all four IAP modules."""

from __future__ import annotations

import json
import hashlib
import math
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_INCLUDES = Path(__file__).resolve().parent / "_includes"
if str(_INCLUDES) not in sys.path:
    sys.path.insert(0, str(_INCLUDES))
from run_directory import resolve_run_directory  # noqa: E402


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _load_json_object(path: Path, description: str) -> dict:
    if not path.is_file():
        raise RuntimeError(f"{description} does not exist: {path}")
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise RuntimeError(f"{description} must contain a JSON object: {path}")
    return value


def _verify_calibration_manifest(path_text: str, calibration_id: str, bound: float):
    path = Path(path_text).expanduser().resolve()
    manifest = _load_json_object(path, "local-surface calibration manifest")
    if manifest.get("schema_version") != "iap_local_surface_calibration_v1":
        raise RuntimeError("unsupported local-surface calibration manifest schema")
    if str(manifest.get("calibration_id", "")).strip() != calibration_id:
        raise RuntimeError("calibration ID does not match calibration manifest")
    retained_bound = float(manifest.get("local_surface_error_bound_m", float("nan")))
    if not math.isfinite(retained_bound) or retained_bound <= 0.0:
        raise RuntimeError("calibration manifest contains an invalid surface-error bound")
    if not math.isclose(retained_bound, bound, rel_tol=0.0, abs_tol=1.0e-12):
        raise RuntimeError("surface-error bound does not match calibration manifest")
    if int(manifest.get("calibration_run_count", 0)) < 3:
        raise RuntimeError("calibration manifest must retain at least three calibration runs")
    if int(manifest.get("held_out_run_count", 0)) < 1 or not bool(
        manifest.get("held_out_passed", False)
    ):
        raise RuntimeError("calibration manifest lacks a passing independent held-out run")
    return path, hashlib.sha256(path.read_bytes()).hexdigest()


def _load_local_map_contract(config_path_text: str) -> dict:
    config_path = Path(config_path_text).expanduser().resolve()
    root = _load_json_object(config_path / "config.json", "flight profile config")
    ros_reference = str(root.get("global", {}).get("config_ros", "")).strip()
    if not ros_reference:
        raise RuntimeError("flight profile does not declare global.config_ros")
    ros_path = Path(ros_reference).expanduser()
    if not ros_path.is_absolute():
        ros_path = config_path / ros_path
    ros = _load_json_object(ros_path.resolve(), "flight profile ROS config")
    local_map = ros.get("glim_ros", {}).get("planner_local_map")
    if not isinstance(local_map, dict):
        raise RuntimeError("flight profile lacks planner_local_map contract")
    if not str(local_map.get("frame_contract_id", "")).strip():
        raise RuntimeError("flight profile local-map frame contract ID is empty")
    return local_map


def _setup(context):
    if not _as_bool(LaunchConfiguration("flight_authorized").perform(context)):
        raise RuntimeError(
            "iap_flight is fail-closed; pass flight_authorized:=true only after "
            "the launch contract and vehicle preflight have been checked"
        )
    if not _as_bool(
        LaunchConfiguration("controller_handshake_confirmed").perform(context)
    ):
        raise RuntimeError(
            "iap_flight requires controller_handshake_confirmed:=true after the "
            "vehicle-side command/feedback handshake has passed"
        )
    output_dir = resolve_run_directory(
        LaunchConfiguration("output_dir").perform(context), entrypoint="iap_flight"
    )
    calibration_id = LaunchConfiguration(
        "local_surface_error_calibration_id"
    ).perform(context).strip()
    if not calibration_id or calibration_id in {
        "uncalibrated_default_v1",
        "simulation_v1",
    }:
        raise RuntimeError(
            "real flight requires a retained deployment local-surface calibration ID"
        )
    surface_bound = float(
        LaunchConfiguration("local_surface_error_bound_m").perform(context)
    )
    calibration_path, calibration_sha256 = _verify_calibration_manifest(
        LaunchConfiguration("local_surface_error_calibration_manifest").perform(context),
        calibration_id,
        surface_bound,
    )

    iap_share = Path(get_package_share_directory("iap"))
    ego_share = Path(get_package_share_directory("ego_planner"))
    goal = [
        float(LaunchConfiguration("goal_x").perform(context)),
        float(LaunchConfiguration("goal_y").perform(context)),
        float(LaunchConfiguration("goal_z").perform(context)),
    ]
    map_size = [
        float(LaunchConfiguration("map_size_x").perform(context)),
        float(LaunchConfiguration("map_size_y").perform(context)),
        float(LaunchConfiguration("map_size_z").perform(context)),
    ]
    local_map_contract = _load_local_map_contract(
        LaunchConfiguration("config_path").perform(context)
    )
    beam_evidence_topic = LaunchConfiguration("beam_evidence_topic").perform(
        context
    ).strip()
    if not beam_evidence_topic:
        raise RuntimeError("flight requires a non-empty external beam_evidence_topic")
    if beam_evidence_topic != str(local_map_contract.get("beam_evidence_topic", "")):
        raise RuntimeError(
            "beam_evidence_topic must exactly match the deployment local-map profile"
        )
    local_map_extent = [
        float(value)
        for value in local_map_contract.get("planning_lattice_extent_m", [])
    ]
    if len(local_map_extent) != 3 or any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-9)
        for actual, expected in zip(map_size, local_map_extent)
    ):
        raise RuntimeError(
            "flight map_size must exactly match the retained planner-local-map extent"
        )
    local_map_origin = [
        float(value)
        for value in local_map_contract.get("planning_lattice_origin_m", [])
    ]
    if len(local_map_origin) != 3 or not all(
        math.isfinite(value) for value in local_map_origin
    ):
        raise RuntimeError("flight profile has an invalid planner-local-map origin")
    planner_output = output_dir / "planner"
    planner_output.mkdir(parents=True, exist_ok=True)
    (output_dir / "flight_launch_manifest.json").write_text(
        json.dumps(
            {
                "schema_version": "iap_canonical_flight_v1",
                "modules": [
                    "GLIO",
                    "Current Integrity Monitor",
                    "Advisory Integrity Evaluator (P0)",
                    "Safety-aware planner (P4/P5)",
                ],
                "task_mode": "strict_global",
                "goal": goal,
                "local_surface_error_calibration_id": calibration_id,
                "local_surface_error_bound_m": surface_bound,
                "local_surface_error_calibration_manifest": str(calibration_path),
                "local_surface_error_calibration_manifest_sha256": calibration_sha256,
                "planner_local_map_frame_contract_id": local_map_contract[
                    "frame_contract_id"
                ],
                "controller_handshake_confirmed": True,
                "beam_evidence_topic": beam_evidence_topic,
                "simulation_nodes_allowed": False,
                "rosbag_play_allowed": False,
                "phase2_planner_integrity_evaluator_enabled": False,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(iap_share / "launch" / "glio_integrity.launch.py")),
        launch_arguments={
            "config_path": LaunchConfiguration("config_path").perform(context),
            "output_dir": str(output_dir / "estimator"),
            "imu_topic": LaunchConfiguration("imu_topic").perform(context),
            "points_topic": LaunchConfiguration("points_topic").perform(context),
            "use_sim_time": "false",
            "integrity_profile": "fused",
            "forbid_sim_extensions": "true",
            "runtime_contract": "full_stack",
        }.items(),
    )
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ego_share / "launch" / "advanced_param.launch.py")
        ),
        launch_arguments={
            "drone_id": LaunchConfiguration("drone_id").perform(context),
            "realworld_experiment": "true",
            "map_size_x_": str(map_size[0]),
            "map_size_y_": str(map_size[1]),
            "map_size_z_": str(map_size[2]),
            "grid_map_origin_x": str(local_map_origin[0]),
            "grid_map_origin_y": str(local_map_origin[1]),
            "grid_map_origin_z": str(local_map_origin[2]),
            "odometry_topic": LaunchConfiguration("odometry_topic").perform(context),
            "camera_pose_topic": LaunchConfiguration("camera_pose_topic").perform(context),
            "depth_topic": LaunchConfiguration("depth_topic").perform(context),
            "cloud_topic": LaunchConfiguration("planner_cloud_topic").perform(context),
            "max_vel": LaunchConfiguration("max_velocity_mps").perform(context),
            "max_acc": LaunchConfiguration("max_acceleration_mps2").perform(context),
            "planning_horizon": LaunchConfiguration("planning_horizon_m").perform(context),
            "flight_type": "2",
            "point_num": "1",
            "point0_x": str(goal[0]),
            "point0_y": str(goal[1]),
            "point0_z": str(goal[2]),
            "obj_num_set": "0",
            "use_integrity_cost": "false",
            "integrity_debug_csv_path": str(planner_output / "integrity_cost.csv"),
            "p1_use_integrity_cost": "false",
            "p1_debug_csv_path": str(planner_output / "p1.csv"),
            "p2_enable_candidate_ranking": "false",
            "p2_debug_csv_path": str(planner_output / "p2.csv"),
            "p3_enable_local_reference_bias": "false",
            "p3_enable_global_reference_bias": "false",
            "p3_debug_csv_path": str(planner_output / "p3.csv"),
            "p0_enable_risk_grid": "true",
            "p0_size_x_m": str(map_size[0]),
            "p0_size_y_m": str(map_size[1]),
            "p0_size_z_m": str(map_size[2]),
            "p0_origin_x_m": str(local_map_origin[0]),
            "p0_origin_y_m": str(local_map_origin[1]),
            "p0_origin_z_m": str(local_map_origin[2]),
            "p0_online_mapping_mode": "true",
            "p0_fit_grid_to_map_cloud": "false",
            "p0_map_topic": "",
            "p0_debug_metrics_enable": "true",
            "p4_enable_risk_aware_astar": "true",
            "p4_require_risk_grid_ready_before_planning": "false",
            "p4_fallback_to_original_when_risk_not_ready": "false",
            "p4_debug_csv_enable": "true",
            "p4_debug_csv_path": str(planner_output / "p4.csv"),
            "p4_assurance_task_mode": "strict_global",
            "p4_assurance_local_surface_error_bound_m": LaunchConfiguration(
                "local_surface_error_bound_m"
            ).perform(context),
            "p4_assurance_local_surface_error_calibration_id": calibration_id,
            "p5_enable_runtime_gate": "true",
            "p5_enable_final_gate": "true",
            "p5_debug_metrics_enable": "true",
            "registered_lidar_window_enabled": "true",
            "registered_frame_contract_id": str(
                local_map_contract["frame_contract_id"]
            ),
            "registered_current_topic": str(local_map_contract["current_topic"]),
            "registered_delta_topic": str(local_map_contract["delta_topic"]),
            "registered_recovery_service": str(
                local_map_contract["recovery_service"]
            ),
            "registered_lidar_reference_frame_id": str(
                local_map_contract["lidar_reference_frame_id"]
            ),
            "trusted_local_map_support_enabled": "true",
        }.items(),
    )
    drone_id = LaunchConfiguration("drone_id").perform(context)
    traj_server = Node(
        package="ego_planner",
        executable="traj_server",
        name=f"drone_{drone_id}_traj_server",
        output="screen",
        remappings=[
            ("planning/bspline", f"/drone_{drone_id}_planning/bspline"),
            (
                "planning/pending_guard_bspline",
                f"/drone_{drone_id}_planning/pending_guard_bspline",
            ),
            (
                "planning/pending_guard_status",
                f"/drone_{drone_id}_planning/pending_guard_status",
            ),
            ("position_cmd", f"/drone_{drone_id}_planning/pos_cmd"),
            ("/position_cmd", f"/drone_{drone_id}_planning/pos_cmd"),
        ],
        parameters=[{"traj_server/time_forward": 1.0}, {"use_sim_time": False}],
    )
    delay = max(
        0.0, float(LaunchConfiguration("planner_start_delay_s").perform(context))
    )
    return [
        LogInfo(
            msg=(
                f"[iap_flight] strict-global goal={goal} output={output_dir}; "
                "simulation, truth, fake odometry, rosbag, and Phase-2 evaluator are forbidden"
            )
        ),
        estimator,
        TimerAction(period=delay, actions=[planner, traj_server]),
    ]


def generate_launch_description():
    iap_share = Path(get_package_share_directory("iap"))
    default_config = str(iap_share / "config" / "profiles" / "full_stack_flight")
    return LaunchDescription(
        [
            DeclareLaunchArgument("flight_authorized", default_value="false"),
            DeclareLaunchArgument(
                "controller_handshake_confirmed", default_value="false"
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="",
                description="Optional absolute override; empty creates a timestamped flight run.",
            ),
            DeclareLaunchArgument("config_path", default_value=default_config),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("odometry_topic", default_value="/glim_ros/odom"),
            DeclareLaunchArgument("planner_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument(
                "beam_evidence_topic",
                description=(
                    "Required external hardware ray/return evidence topic; must match "
                    "the deployment planner-local-map profile."
                ),
            ),
            DeclareLaunchArgument("camera_pose_topic", default_value="/camera/pose"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth"),
            DeclareLaunchArgument("goal_x"),
            DeclareLaunchArgument("goal_y"),
            DeclareLaunchArgument("goal_z"),
            DeclareLaunchArgument("drone_id", default_value="0"),
            DeclareLaunchArgument("map_size_x", default_value="42.0"),
            DeclareLaunchArgument("map_size_y", default_value="30.0"),
            DeclareLaunchArgument("map_size_z", default_value="8.0"),
            DeclareLaunchArgument("max_velocity_mps", default_value="1.0"),
            DeclareLaunchArgument("max_acceleration_mps2", default_value="1.5"),
            DeclareLaunchArgument("planning_horizon_m", default_value="8.0"),
            DeclareLaunchArgument("planner_start_delay_s", default_value="5.0"),
            DeclareLaunchArgument("local_surface_error_bound_m"),
            DeclareLaunchArgument("local_surface_error_calibration_id"),
            DeclareLaunchArgument("local_surface_error_calibration_manifest"),
            OpaqueFunction(function=_setup),
        ]
    )
