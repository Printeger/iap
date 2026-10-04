"""Private simulator adapter used by canonical simulation profiles.

This file owns simulated world, sensor, GNSS, vehicle, and controller nodes.
It deliberately starts no GLIO, integrity monitor, advisory evaluator, planner,
validator, rosbag recorder, or RViz process.
"""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _catalog(iap_share: Path) -> dict:
    path = iap_share / "config" / "scenarios" / "catalog.json"
    with path.open(encoding="utf-8") as stream:
        catalog = json.load(stream)
    if not isinstance(catalog, dict) or not catalog:
        raise RuntimeError(f"invalid or empty scenario catalog: {path}")
    return catalog


def _map_parameters(profile: str, initial: list[float], goal: list[float]) -> dict:
    base = {
        "resolution_m": 0.10,
        "publish_rate_hz": 2.0,
        "frame_id": "map",
        "stamp_authority_topic": "/sim/drone_0/truth_odom",
        "forest_size_x_m": 30.0,
        "forest_size_y_m": 20.0,
        "random_seed": 41021,
        "corridor_floor_enabled": True,
        "corridor_x_min_m": -14.0,
        "corridor_x_max_m": 14.0,
        "corridor_half_width_y_m": 2.0,
        "corridor_floor_thickness_z_m": 0.05,
        "corridor_surface_resolution_m": 0.10,
    }
    if profile == "open":
        return {
            **base,
            "tree_density_lower_left_per_m2": 0.02,
            "tree_density_lower_right_per_m2": 0.02,
            "tree_density_upper_left_per_m2": 0.02,
            "tree_density_upper_right_per_m2": 0.02,
            "stratified_cell_size_m": 4.0,
            "clear_corridor_enabled": True,
            "clear_corridor_center_y_m": 0.0,
            "clear_corridor_half_width_y_m": 1.8,
            "clear_corridor_x_min_m": -12.5,
            "clear_corridor_x_max_m": 12.5,
            "trunk_radius_m": 0.10,
            "trunk_min_height_m": 0.30,
            "trunk_max_height_m": 1.15,
            "canopy_density_lower_left": 0.0,
            "canopy_density_lower_right": 0.0,
            "canopy_density_upper_left": 0.0,
            "canopy_density_upper_right": 0.0,
            "terminal_wall_enabled": False,
        }
    if profile == "feature_rich":
        return {
            **base,
            "endpoint_clearance_radius_m": 1.0,
            "endpoint_start_x_m": initial[0],
            "endpoint_start_y_m": initial[1],
            "endpoint_goal_x_m": goal[0],
            "endpoint_goal_y_m": goal[1],
            "tree_density_lower_left_per_m2": 0.75,
            "tree_density_lower_right_per_m2": 0.75,
            "tree_density_upper_left_per_m2": 0.75,
            "tree_density_upper_right_per_m2": 0.75,
            "canopy_density_lower_left": 0.08,
            "canopy_density_lower_right": 0.08,
            "canopy_density_upper_left": 0.25,
            "canopy_density_upper_right": 0.25,
            "terminal_wall_enabled": True,
            "terminal_wall_feature_count": 64,
            "corridor_walls_enabled": False,
        }
    if profile == "corridor":
        return {
            **base,
            "forest_size_y_m": 6.0,
            "tree_density_lower_left_per_m2": 0.0,
            "tree_density_lower_right_per_m2": 0.0,
            "tree_density_upper_left_per_m2": 0.0,
            "tree_density_upper_right_per_m2": 0.0,
            "canopy_density_lower_left": 0.0,
            "canopy_density_lower_right": 0.0,
            "canopy_density_upper_left": 0.0,
            "canopy_density_upper_right": 0.0,
            "terminal_wall_enabled": False,
            "corridor_walls_enabled": True,
            "corridor_wall_z_min_m": 0.0,
            "corridor_wall_z_max_m": 3.0,
            "corridor_wall_thickness_y_m": 0.10,
        }
    if profile in {"dense_forest_v1", "dense_forest_v2"}:
        return {
            **base,
            "forest_layout_mode": (
                "forked_s_forest_v2" if profile.endswith("v2") else "forked_s_forest_v1"
            ),
            "forest_size_x_m": 40.0,
            "forest_size_y_m": 20.0,
            "tree_density_lower_left_per_m2": 0.25,
            "tree_density_lower_right_per_m2": 0.25,
            "tree_density_upper_left_per_m2": 0.25,
            "tree_density_upper_right_per_m2": 0.25,
            "canopy_density_lower_left": 0.65,
            "canopy_density_lower_right": 0.65,
            "canopy_density_upper_left": 0.65,
            "canopy_density_upper_right": 0.65,
            "canopy_hemisphere_radius_min_m": 0.8,
            "canopy_hemisphere_radius_max_m": 1.5,
            "canopy_ball_spacing_ratio": 2.0,
            "canopy_resolution_m": 0.2,
            "trunk_radius_m": 0.14,
            "trunk_min_height_m": 3.2,
            "trunk_max_height_m": 5.0,
            "forked_forest.start_canopy_clearance_radius_m": (
                5.0 if profile.endswith("v2") else 0.0
            ),
            "terminal_wall_enabled": False,
            "corridor_walls_enabled": False,
            "corridor_x_min_m": -20.0,
            "corridor_x_max_m": 20.0,
            "corridor_half_width_y_m": 10.0,
            "corridor_surface_resolution_m": 0.15,
        }
    if profile.startswith(("p1_", "p4_", "icra_p0_p4_", "icra072_")):
        mirror = "mirror" in profile
        parameters = {
            **base,
            "forest_size_x_m": 28.0,
            "forest_size_y_m": 10.0,
            "tree_density_lower_left_per_m2": 0.0,
            "tree_density_lower_right_per_m2": 0.0,
            "tree_density_upper_left_per_m2": 0.0,
            "tree_density_upper_right_per_m2": 0.0,
            "canopy_density_lower_left": 0.0,
            "canopy_density_lower_right": 0.0,
            "canopy_density_upper_left": 0.0,
            "canopy_density_upper_right": 0.0,
            "terminal_wall_enabled": False,
            "corridor_half_width_y_m": 5.0,
            "p1_map_fixture": profile,
            "p1_fixture_mirror_y": mirror,
            "p1_fixture_central_obstacle_enabled": True,
        }
        if profile == "p1_fork_symmetric_null_v1":
            parameters["p1_fixture_risky_tree_density_per_m2"] = 0.25
            parameters["p1_fixture_risky_canopy_probability"] = 0.05
        if profile == "p1_soft_risk_island_v1":
            parameters["p1_fixture_central_obstacle_enabled"] = True
        if profile == "icra_p0_p4_v2_p5_dev_fixture_v1":
            parameters["p1_fixture_central_x_min_m"] = -9.0
            parameters["p1_fixture_central_x_max_m"] = -7.0
        return parameters
    raise RuntimeError(f"unsupported map profile: {profile}")


def _gnss_parameters(iap_share: Path, profile: str) -> dict:
    degraded = profile == "degraded"
    occlusion = profile in {"degraded", "open_sky_occlusion"}
    return {
        "truth_odom_topic": "/sim/drone_0/truth_odom",
        "origin_lat_deg": 31.2304,
        "origin_lon_deg": 121.4737,
        "origin_alt_m": 25.0,
        "pseudorange_noise_std_m": 5.0 if degraded else 1.0,
        "doppler_noise_std_mps": 0.5 if degraded else 0.03,
        "random_seed": 20260502,
        "time_source": "odom_stamp",
        "scenario_file": str(
            iap_share
            / "config"
            / "gnss_sim"
            / ("demo7_skymask_nlos.yaml" if degraded else "demo7_open_sky.yaml")
        ),
        "num_gps_sats": 24,
        "gps_prn_min": 1,
        "gps_prn_max": 24,
        "ephemeris_source": "synthetic",
        "enabled_constellations_csv": "GPS,BDS,GAL,GLO",
        "enable_visualization": True,
        "enable_map_occlusion": occlusion,
        "enable_skymask": degraded,
        "enable_nlos": occlusion,
        "enable_multipath": degraded,
        "enable_fault_injection": False,
        "map_cloud_topic": "/map_generator/global_cloud",
    }


def _setup(context):
    iap_share = Path(get_package_share_directory("iap"))
    local_sensing_share = Path(get_package_share_directory("local_sensing"))
    so3_control_share = Path(get_package_share_directory("so3_control"))
    scenario_name = LaunchConfiguration("scenario").perform(context)
    catalog = _catalog(iap_share)
    if scenario_name not in catalog:
        valid = ", ".join(sorted(catalog))
        raise RuntimeError(f"unknown IAP simulation scenario '{scenario_name}'; valid: {valid}")
    scenario = catalog[scenario_name]
    initial = [float(value) for value in scenario["initial"]]
    output_dir = Path(LaunchConfiguration("output_dir").perform(context)).expanduser()
    if not output_dir.is_absolute():
        raise RuntimeError("output_dir must be an absolute path")
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_dir = output_dir / "metadata" / "manifests"
    manifest_dir.mkdir(parents=True, exist_ok=True)
    (manifest_dir / "scenario.json").write_text(
        json.dumps(
            {
                "schema_version": "iap_simulation_scenario_v1",
                "scenario": scenario_name,
                "contract": scenario,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    truth_odom = "/sim/drone_0/truth_odom"
    sim_imu = "/sim/drone_0/imu"
    iap_imu = "/sim/drone_0/imu_iap"
    sim_lidar = "/sim/drone_0/lidar"
    iap_lidar = "/sim/drone_0/lidar_body"
    depth = "/sim/drone_0/depth"
    camera_pose = "/drone_0_pcl_render_node/camera_pose"
    position_cmd = "/drone_0_planning/pos_cmd"
    so3_cmd = "/iap_sim/so3_cmd"
    map_size = [float(value) for value in scenario["map_size"]]

    actions = [
        LogInfo(msg=f"[iap_sim/environment] scenario={scenario_name} output={output_dir}"),
        Node(
            package="iap",
            executable="demo11_corridor_map_publisher",
            name="iap_sim_map_publisher",
            output="screen",
            parameters=[_map_parameters(str(scenario["map_profile"]), initial, scenario["goal"])],
        ),
        TimerAction(period=2.0, actions=[Node(
            package="local_sensing",
            executable="pcl_render_node",
            name="drone_0_pcl_render_node",
            output="screen",
            remappings=[
                ("global_map", "/map_generator/global_cloud"),
                ("local_map", "/map_generator/local_cloud"),
                ("odometry", truth_odom),
                ("pcl_render_node/cloud", sim_lidar),
                ("depth", depth),
                ("camera_pose", camera_pose),
            ],
            parameters=[
                {"sensing_horizon": 10.0},
                {"renderer_mode": "spherical_first_hit_v1"},
                {"lidar.horizontal_samples": 512},
                {"lidar.vertical_samples": 40},
                {"lidar.horizontal_fov_deg": 360.0},
                {"lidar.vertical_min_deg": -7.0},
                {"lidar.vertical_max_deg": 52.0},
                {"lidar.min_range_m": 0.1},
                {"lidar.max_range_m": 10.0},
                {"lidar.world_voxel_resolution_m": 0.1},
                {"sensing_rate": 10.0},
                {"estimation_rate": 15.0},
                {"map/x_size": map_size[0]},
                {"map/y_size": map_size[1]},
                {"map/z_size": map_size[2]},
                {"map/resolution": 0.1},
                str(local_sensing_share / "config" / "camera.yaml"),
            ],
        )]),
        Node(
            package="iap",
            executable="demo4_lidar_body_bridge",
            name="iap_sim_lidar_body_bridge",
            output="screen",
            parameters=[
                {"input_cloud_topic": sim_lidar},
                {"input_odom_topic": truth_odom},
                {"output_cloud_topic": iap_lidar},
                {"output_frame_id": "lidar"},
                {"max_odom_lookup_dt": 0.05},
            ],
        ),
        Node(
            package="so3_quadrotor_simulator",
            executable="so3_quadrotor_simulator",
            name="drone_0_quadrotor_simulator_so3",
            output="screen",
            remappings=[
                ("odom", truth_odom),
                ("imu", sim_imu),
                ("cmd", so3_cmd),
                ("force_disturbance", "/iap_sim/force_disturbance"),
                ("moment_disturbance", "/iap_sim/moment_disturbance"),
            ],
            parameters=[
                {"quadrotor_name": "drone_0"},
                {"rate/simulation": 1000.0},
                {"rate/odom": 100.0},
                {"simulator/init_state_x": initial[0]},
                {"simulator/init_state_y": initial[1]},
                {"simulator/init_state_z": initial[2]},
                {"simulator/hold_until_cmd": True},
                # No /clock publisher is part of this environment. Keep ROS
                # nodes and simulated message stamps on the system clock.
                {"sim_time/enable": False},
                {"sim_time/start_utc": ""},
                {"iap_imu/enable": True},
                {"iap_imu/topic": iap_imu},
            ],
        ),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="iap_sim_so3_control_container",
            namespace="",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="so3_control",
                    plugin="SO3ControlComponent",
                    name="iap_sim_so3_control",
                    parameters=[
                        {"quadrotor_name": "drone_0"},
                        {"so3_control/init_state_x": initial[0]},
                        {"so3_control/init_state_y": initial[1]},
                        {"so3_control/init_state_z": initial[2]},
                        {"mass": 0.98},
                        {"use_angle_corrections": False},
                        {"use_external_yaw": False},
                        {"gains/rot/z": 1.0},
                        {"gains/ang/z": 0.1},
                        str(so3_control_share / "config" / "gains_hummingbird.yaml"),
                        str(so3_control_share / "config" / "corrections_hummingbird.yaml"),
                    ],
                    remappings=[
                        ("odom", "/drone_0_visual_slam/odom"),
                        ("position_cmd", position_cmd),
                        ("motors", "/iap_sim/motors"),
                        ("corrections", "/iap_sim/corrections"),
                        ("so3_cmd", so3_cmd),
                        ("controller_trace", "/drone_0_controller_trace"),
                        ("imu", sim_imu),
                    ],
                )
            ],
        ),
    ]
    if scenario["gnss_profile"] != "disabled":
        actions.append(
            Node(
                package="gnss_sim",
                executable="gnss_sim_node",
                name="iap_sim_gnss",
                output="screen",
                parameters=[_gnss_parameters(iap_share, str(scenario["gnss_profile"]))],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="fused_nominal"),
            DeclareLaunchArgument(
                "output_dir",
                description="Required absolute directory for scenario manifest.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
