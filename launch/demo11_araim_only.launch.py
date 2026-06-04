import json
import os
import shutil
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _runtime_config(context, use_gnss, use_araim, allow_truth_alignment):
    iap_share = Path(get_package_share_directory("iap"))
    base_config = iap_share / "config"
    config_subdir = LaunchConfiguration("config_subdir").perform(context)
    config_name = config_subdir.replace("/", "_").replace("\\", "_")
    runtime_root = Path("/tmp") / f"iap_{config_name}_araim_only_{os.getpid()}_{int(time.time() * 1000)}"
    runtime_config_dir = runtime_root / config_subdir

    shutil.copytree(
        base_config / config_subdir,
        runtime_config_dir,
        ignore_dangling_symlinks=True,
    )
    shutil.copytree(
        base_config / "sim_ego",
        runtime_root / "sim_ego",
        ignore_dangling_symlinks=True,
    )

    config_ros_path = runtime_config_dir / "config_ros.json"
    config_gnss_path = runtime_config_dir / "config_gnss.json"

    with config_ros_path.open() as f:
        config_ros = json.load(f)

    modules = ["libsim_extension.so"]
    if use_gnss:
        modules.insert(0, "libgnss_extension.so")
    if use_araim:
        insert_at = 1 if use_gnss else 0
        modules.insert(insert_at, "libintegrity_extension.so")

    config_ros["glim_ros"]["extension_modules"] = modules
    config_ros["glim_ros"]["sim"]["align_planner_odom_to_truth"] = allow_truth_alignment
    with config_ros_path.open("w") as f:
        json.dump(config_ros, f, indent=2)
        f.write("\n")

    with config_gnss_path.open() as f:
        config_gnss = json.load(f)
    config_gnss["integrity"]["enable"] = bool(use_araim)
    config_gnss["integrity"]["enable_araim"] = bool(use_araim and use_gnss)
    config_gnss["integrity"]["enable_araim_csv"] = bool(use_araim and use_gnss)
    with config_gnss_path.open("w") as f:
        json.dump(config_gnss, f, indent=2)
        f.write("\n")

    return str(runtime_config_dir)


def _odom_visualization_node(
    name,
    odom_topic,
    cmd_topic,
    topic_prefix,
    color,
    drone_id,
    sensor_text_use_fixed_position=False,
    sensor_text_fixed_position=(0.0, 0.0, 14.0),
):
    r, g, b = color
    text_x, text_y, text_z = sensor_text_fixed_position
    return Node(
        package="odom_visualization",
        executable="odom_visualization",
        name=name,
        output="screen",
        remappings=[
            ("odom", odom_topic),
            ("cmd", cmd_topic),
            ("pose", f"{topic_prefix}/pose"),
            ("path", f"{topic_prefix}/path"),
            ("robot", f"{topic_prefix}/robot"),
            ("velocity", f"{topic_prefix}/velocity"),
            ("sensor", f"{topic_prefix}/sensor_status"),
            ("time_gap", f"{topic_prefix}/time_gap"),
        ],
        parameters=[
            {"frame_id": "map"},
            {"robot_scale": 1.0},
            {"color/r": r},
            {"color/g": g},
            {"color/b": b},
            {"color/a": 1.0},
            {"tf45": False},
            {"drone_id": drone_id},
            {"sensor_text_use_fixed_position": bool(sensor_text_use_fixed_position)},
            {"sensor_text_fixed_x": float(text_x)},
            {"sensor_text_fixed_y": float(text_y)},
            {"sensor_text_fixed_z": float(text_z)},
        ],
    )


def _launch_setup(context):
    iap_share = get_package_share_directory("iap")
    so3_control_share = get_package_share_directory("so3_control")
    local_sensing_share = get_package_share_directory("local_sensing")

    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    use_gnss = _as_bool(LaunchConfiguration("use_gnss").perform(context))
    use_araim = _as_bool(LaunchConfiguration("use_araim").perform(context))
    use_so3_dynamics = _as_bool(LaunchConfiguration("use_so3_dynamics").perform(context))
    allow_truth_alignment = _as_bool(LaunchConfiguration("allow_truth_alignment").perform(context))
    enable_preflight_takeoff = _as_bool(LaunchConfiguration("enable_preflight_takeoff").perform(context))
    log_phase1 = _as_bool(LaunchConfiguration("log_phase1").perform(context))
    use_iap_odom = _as_bool(LaunchConfiguration("use_iap_odom_for_planner").perform(context))

    drone_id = LaunchConfiguration("drone_id").perform(context)
    init_x = float(LaunchConfiguration("init_x").perform(context))
    init_y = float(LaunchConfiguration("init_y").perform(context))
    init_z = float(LaunchConfiguration("init_z").perform(context))
    goal_x = LaunchConfiguration("goal_x").perform(context)
    goal_y = LaunchConfiguration("goal_y").perform(context)
    goal_z = LaunchConfiguration("goal_z").perform(context)
    point_num = LaunchConfiguration("point_num").perform(context)
    preflight_ground_z = float(LaunchConfiguration("preflight_ground_z").perform(context))
    plant_init_z = preflight_ground_z if enable_preflight_takeoff else init_z

    run_duration_s = float(LaunchConfiguration("run_duration_s").perform(context))
    map_size_x = float(LaunchConfiguration("map_size_x").perform(context))
    map_size_y = float(LaunchConfiguration("map_size_y").perform(context))
    map_size_z = float(LaunchConfiguration("map_size_z").perform(context))
    fixed_status_text = _as_bool(
        LaunchConfiguration("viz_status_text_use_fixed_position").perform(context)
    )
    status_text_position = (
        float(LaunchConfiguration("viz_status_text_x").perform(context)),
        float(LaunchConfiguration("viz_status_text_y").perform(context)),
        float(LaunchConfiguration("viz_status_text_z").perform(context)),
    )

    gnss_ephemeris_source = LaunchConfiguration("gnss_ephemeris_source").perform(context)
    gnss_enabled_constellations = LaunchConfiguration("gnss_enabled_constellations").perform(context)
    gnss_scenario_file = LaunchConfiguration("gnss_scenario_file").perform(context)
    gnss_rinex_nav_file = LaunchConfiguration("gnss_rinex_nav_file").perform(context)
    gnss_rinex_ephem_max_age_s = LaunchConfiguration("gnss_rinex_ephem_max_age_s").perform(context)
    gnss_fallback_to_synthetic = _as_bool(
        LaunchConfiguration("gnss_fallback_to_synthetic_on_rinex_error").perform(context)
    )
    gnss_sky_dome_center_enu = [
        float(LaunchConfiguration("gnss_sky_dome_center_x").perform(context)),
        float(LaunchConfiguration("gnss_sky_dome_center_y").perform(context)),
        float(LaunchConfiguration("gnss_sky_dome_center_z").perform(context)),
    ]
    gnss_skyplot_origin_enu = [
        float(LaunchConfiguration("gnss_skyplot_origin_x").perform(context)),
        float(LaunchConfiguration("gnss_skyplot_origin_y").perform(context)),
        float(LaunchConfiguration("gnss_skyplot_origin_z").perform(context)),
    ]

    if (
        use_gnss
        and gnss_ephemeris_source.strip().lower() == "rinex"
        and not gnss_fallback_to_synthetic
        and not Path(gnss_rinex_nav_file).expanduser().is_file()
    ):
        raise RuntimeError(
            "gnss_ephemeris_source:=rinex requires an existing "
            "gnss_rinex_nav_file when "
            "gnss_fallback_to_synthetic_on_rinex_error:=false; "
            f"got '{gnss_rinex_nav_file}'"
        )

    runtime_config_path = _runtime_config(
        context,
        use_gnss=use_gnss,
        use_araim=use_araim,
        allow_truth_alignment=allow_truth_alignment,
    )

    truth_odom_topic = "/sim/drone_0/truth_odom"
    iap_odom_topic = "/drone_0_visual_slam/odom"
    planner_odom_topic = iap_odom_topic if use_iap_odom else truth_odom_topic
    sim_imu_topic = "/sim/drone_0/imu"
    iap_imu_topic = "/sim/drone_0/imu_iap"
    sim_lidar_topic = "/sim/drone_0/lidar"
    iap_lidar_topic = "/sim/drone_0/lidar_body"
    sim_depth_topic = "/sim/drone_0/depth"
    pos_cmd_topic = "/drone_0_planning/pos_cmd"
    desired_odom_topic = "/demo11_araim/desired/odom"
    so3_cmd_topic = "/demo11_araim/so3_cmd"

    camera_file = os.path.join(local_sensing_share, "config", "camera.yaml")
    gains_file = os.path.join(so3_control_share, "config", "gains_hummingbird.yaml")
    corrections_file = os.path.join(so3_control_share, "config", "corrections_hummingbird.yaml")

    actions = [
        LogInfo(msg="[demo11_araim] Simulation + IAP localization + ARAIM + native ego planner through forest"),
        LogInfo(msg=f"[demo11_araim] runtime IAP config: {runtime_config_path}"),
        LogInfo(msg="[demo11_araim] Planner uses truth odom; all custom integrity features OFF"),
        Node(
            package="iap",
            executable="demo11_corridor_map_publisher",
            name="demo11_araim_corridor_map_publisher",
            output="screen",
            parameters=[
                {"resolution_m": float(LaunchConfiguration("corridor_map_resolution_m").perform(context))},
                {"publish_rate_hz": float(LaunchConfiguration("corridor_map_publish_rate_hz").perform(context))},
                {"frame_id": "map"},
                {"forest_size_x_m": float(LaunchConfiguration("forest_size_x_m").perform(context))},
                {"forest_size_y_m": float(LaunchConfiguration("forest_size_y_m").perform(context))},
                {"tree_density_lower_left_per_m2": float(LaunchConfiguration("tree_density_lower_left_per_m2").perform(context))},
                {"tree_density_lower_right_per_m2": float(LaunchConfiguration("tree_density_lower_right_per_m2").perform(context))},
                {"tree_density_upper_left_per_m2": float(LaunchConfiguration("tree_density_upper_left_per_m2").perform(context))},
                {"tree_density_upper_right_per_m2": float(LaunchConfiguration("tree_density_upper_right_per_m2").perform(context))},
                {"stratified_cell_size_m": float(LaunchConfiguration("stratified_cell_size_m").perform(context))},
                {"canopy_density_lower_left": float(LaunchConfiguration("canopy_density_lower_left").perform(context))},
                {"canopy_density_lower_right": float(LaunchConfiguration("canopy_density_lower_right").perform(context))},
                {"canopy_density_upper_left": float(LaunchConfiguration("canopy_density_upper_left").perform(context))},
                {"canopy_density_upper_right": float(LaunchConfiguration("canopy_density_upper_right").perform(context))},
                {"canopy_hemisphere_radius_min_m": float(LaunchConfiguration("canopy_hemisphere_radius_min_m").perform(context))},
                {"canopy_hemisphere_radius_max_m": float(LaunchConfiguration("canopy_hemisphere_radius_max_m").perform(context))},
                {"canopy_leaf_ball_radius_m": float(LaunchConfiguration("canopy_leaf_ball_radius_m").perform(context))},
                {"canopy_ball_spacing_ratio": float(LaunchConfiguration("canopy_ball_spacing_ratio").perform(context))},
                {"canopy_resolution_m": float(LaunchConfiguration("canopy_resolution_m").perform(context))},
                {"random_seed": int(LaunchConfiguration("forest_random_seed").perform(context))},
                {"trunk_radius_m": float(LaunchConfiguration("trunk_radius_m").perform(context))},
                {"trunk_min_height_m": float(LaunchConfiguration("trunk_min_height_m").perform(context))},
                {"trunk_max_height_m": float(LaunchConfiguration("trunk_max_height_m").perform(context))},
                {"terminal_wall_enabled": _as_bool(LaunchConfiguration("terminal_wall_enabled").perform(context))},
                {"terminal_wall_x_m": float(LaunchConfiguration("terminal_wall_x_m").perform(context))},
                {"terminal_wall_y_m": float(LaunchConfiguration("terminal_wall_y_m").perform(context))},
                {"terminal_wall_width_y_m": float(LaunchConfiguration("terminal_wall_width_y_m").perform(context))},
                {"terminal_wall_z_min_m": float(LaunchConfiguration("terminal_wall_z_min_m").perform(context))},
                {"terminal_wall_z_max_m": float(LaunchConfiguration("terminal_wall_z_max_m").perform(context))},
                {"terminal_wall_thickness_x_m": float(LaunchConfiguration("terminal_wall_thickness_x_m").perform(context))},
                {"terminal_wall_resolution_m": float(LaunchConfiguration("terminal_wall_resolution_m").perform(context))},
                {"terminal_wall_feature_depth_x_m": float(LaunchConfiguration("terminal_wall_feature_depth_x_m").perform(context))},
                {"terminal_wall_feature_count": int(LaunchConfiguration("terminal_wall_feature_count").perform(context))},
                {"terminal_wall_feature_seed": int(LaunchConfiguration("terminal_wall_feature_seed").perform(context))},
            ],
        ),
        Node(
            package="local_sensing",
            executable="pcl_render_node",
            name="drone_0_pcl_render_node",
            output="screen",
            remappings=[
                ("global_map", "/map_generator/global_cloud"),
                ("local_map", "/map_generator/local_cloud"),
                ("odometry", truth_odom_topic),
                ("pcl_render_node/cloud", sim_lidar_topic),
                ("depth", sim_depth_topic),
                ("camera_pose", "/drone_0_pcl_render_node/camera_pose"),
            ],
            parameters=[
                {"sensing_horizon": 10.0},
                {"sensing_rate": 15.0},
                {"estimation_rate": 15.0},
                {"map/x_size": map_size_x},
                {"map/y_size": map_size_y},
                {"map/z_size": map_size_z},
                {"map/resolution": 0.1},
                camera_file,
            ],
        ),
        Node(
            package="iap",
            executable="demo4_lidar_body_bridge",
            name="demo11_araim_lidar_body_bridge",
            output="screen",
            parameters=[
                {"input_cloud_topic": sim_lidar_topic},
                {"input_odom_topic": truth_odom_topic},
                {"output_cloud_topic": iap_lidar_topic},
                {"output_frame_id": "lidar"},
                {"max_odom_lookup_dt": 0.05},
            ],
        ),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="demo11_araim_iap_rosnode",
            output="screen",
            parameters=[
                {"config_path": runtime_config_path},
                {"imu_topic": iap_imu_topic},
                {"points_topic": iap_lidar_topic},
            ],
        ),
        Node(
            package="iap",
            executable="demo_takeoff_cmd_publisher",
            name="demo11_araim_preflight_takeoff_cmd_publisher",
            output="screen",
            condition=IfCondition("true" if enable_preflight_takeoff else "false"),
            remappings=[("position_cmd", pos_cmd_topic)],
            parameters=[
                {"ground_x": init_x},
                {"ground_y": init_y},
                {"ground_z": preflight_ground_z},
                {"hover_x": init_x},
                {"hover_y": init_y},
                {"hover_z": init_z},
                {"ground_hold_duration_s": float(LaunchConfiguration("preflight_ground_hold_s").perform(context))},
                {"takeoff_duration_s": float(LaunchConfiguration("preflight_takeoff_duration_s").perform(context))},
                {"hover_duration_s": float(LaunchConfiguration("preflight_hover_s").perform(context))},
                {"publish_rate_hz": float(LaunchConfiguration("preflight_cmd_rate_hz").perform(context))},
                {"trajectory_id": 11001},
                {"frame_id": "map"},
                {"stop_after_sequence": True},
            ],
        ),
        Node(
            package="poscmd_2_odom",
            executable="poscmd_2_odom",
            name="demo11_araim_desired_poscmd_to_odom",
            output="screen",
            remappings=[
                ("command", pos_cmd_topic),
                ("odometry", desired_odom_topic),
            ],
            parameters=[
                {"init_x": init_x},
                {"init_y": init_y},
                {"init_z": plant_init_z},
            ],
        ),
        _odom_visualization_node(
            "demo11_araim_iap_odom_visualization",
            iap_odom_topic,
            pos_cmd_topic,
            "/demo9/drone",
            (0.2, 1.0, 0.4),
            int(drone_id),
            fixed_status_text,
            status_text_position,
        ),
        _odom_visualization_node(
            "demo11_araim_truth_odom_visualization",
            truth_odom_topic,
            pos_cmd_topic,
            "/demo9/truth",
            (1.0, 0.15, 0.1),
            10,
        ),
        _odom_visualization_node(
            "demo11_araim_desired_odom_visualization",
            desired_odom_topic,
            pos_cmd_topic,
            "/demo9/desired",
            (1.0, 0.86, 0.05),
            20,
        ),
        Node(
            package="gnss_sim",
            executable="gnss_sim_node",
            name="demo11_araim_gnss_sim_node",
            output="screen",
            condition=IfCondition("true" if use_gnss else "false"),
            parameters=[
                {"truth_odom_topic": truth_odom_topic},
                {"origin_lat_deg": 31.2304},
                {"origin_lon_deg": 121.4737},
                {"origin_alt_m": 25.0},
                {"time_source": "trigger_topic"},
                {"trigger_topic": sim_lidar_topic},
                {"scenario_file": gnss_scenario_file},
                {"num_gps_sats": 24},
                {"gps_prn_min": 1},
                {"gps_prn_max": 24},
                {"ephemeris_source": gnss_ephemeris_source},
                {"rinex_nav_file": gnss_rinex_nav_file},
                {"rinex_ephem_max_age_s": float(gnss_rinex_ephem_max_age_s)},
                {"rinex_gps_only": False},
                {"fallback_to_synthetic_on_rinex_error": gnss_fallback_to_synthetic},
                {"enabled_constellations_csv": gnss_enabled_constellations},
                {"enable_visualization": True},
                {"signal_ray_width_m": 0.025},
                {"signal_ray_alpha": 0.3},
                {"nlos_path_width_m": 0.04},
                {"nlos_path_alpha": 0.3},
                {"enable_sky_dome_visualization": True},
                {"sky_dome_show_cardinal_labels": True},
                {"sky_dome_ring_count": 3},
                {"sky_dome_meridian_count": 12},
                {"sky_dome_follow_receiver": _as_bool(LaunchConfiguration("gnss_sky_dome_follow_receiver").perform(context))},
                {"sky_dome_center_enu": gnss_sky_dome_center_enu},
                {"skyplot_origin_enu": gnss_skyplot_origin_enu},
                {"status_text_use_fixed_position": fixed_status_text},
                {"status_text_position_enu": list(status_text_position)},
                {"enable_map_occlusion": _as_bool(LaunchConfiguration("gnss_enable_map_occlusion").perform(context))},
                {"enable_skymask": _as_bool(LaunchConfiguration("gnss_enable_skymask").perform(context))},
                {"enable_nlos": _as_bool(LaunchConfiguration("gnss_enable_nlos").perform(context))},
                {"enable_multipath": _as_bool(LaunchConfiguration("gnss_enable_multipath").perform(context))},
                {"enable_fault_injection": _as_bool(LaunchConfiguration("gnss_enable_fault_injection").perform(context))},
                {"map_cloud_topic": "/map_generator/global_cloud"},
                {"multipath_amp_m": 0.5},
            ],
        ),
    ]

    if use_so3_dynamics:
        actions.extend([
            Node(
                package="so3_quadrotor_simulator",
                executable="so3_quadrotor_simulator",
                name=f"drone_{drone_id}_quadrotor_simulator_so3",
                output="screen",
                remappings=[
                    ("odom", truth_odom_topic),
                    ("imu", sim_imu_topic),
                    ("cmd", so3_cmd_topic),
                    ("force_disturbance", "/demo11_araim/force_disturbance"),
                    ("moment_disturbance", "/demo11_araim/moment_disturbance"),
                ],
                parameters=[
                    {"quadrotor_name": f"drone_{drone_id}"},
                    {"rate/simulation": 1000.0},
                    {"rate/odom": 100.0},
                    {"simulator/init_state_x": init_x},
                    {"simulator/init_state_y": init_y},
                    {"simulator/init_state_z": plant_init_z},
                    {"simulator/hold_until_cmd": True},
                    {"sim_time/enable": True},
                    {"sim_time/start_utc": "2022-07-06T00:00:00Z"},
                    {"iap_imu/enable": True},
                    {"iap_imu/topic": iap_imu_topic},
                ],
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container",
                name="demo11_araim_so3_control_container",
                namespace="",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="so3_control",
                        plugin="SO3ControlComponent",
                        name="demo11_araim_so3_control_component",
                        parameters=[
                            {"quadrotor_name": f"drone_{drone_id}"},
                            {"so3_control/init_state_x": init_x},
                            {"so3_control/init_state_y": init_y},
                            {"so3_control/init_state_z": plant_init_z},
                            {"mass": 0.98},
                            {"use_angle_corrections": False},
                            {"use_external_yaw": False},
                            {"gains/rot/z": 1.0},
                            {"gains/ang/z": 0.1},
                            gains_file,
                            corrections_file,
                        ],
                        remappings=[
                            ("odom", truth_odom_topic),
                            ("position_cmd", pos_cmd_topic),
                            ("motors", "/demo11_araim/motors"),
                            ("corrections", "/demo11_araim/corrections"),
                            ("so3_cmd", so3_cmd_topic),
                            ("imu", iap_imu_topic),
                        ],
                    )
                ],
            ),
        ])
    else:
        actions.append(
            Node(
                package="poscmd_2_odom",
                executable="poscmd_2_odom",
                name="demo11_araim_poscmd_2_odom_debug_plant",
                output="screen",
                remappings=[
                    ("command", pos_cmd_topic),
                    ("odometry", truth_odom_topic),
                ],
                parameters=[
                    {"init_x": init_x},
                    {"init_y": init_y},
                    {"init_z": plant_init_z},
                ],
            )
        )

    if log_phase1:
        actions.append(
            Node(
                package="iap_phase1_tools",
                executable="phase1_closed_loop_logger",
                name="demo11_araim_phase1_logger",
                output="screen",
                parameters=[
                    {"log_root": "/home/dev/ws_iap/src/iap/log"},
                    {"run_duration_s": run_duration_s},
                    {"goal_x": float(goal_x) if goal_x else init_x},
                    {"goal_y": float(goal_y) if goal_y else init_y},
                    {"goal_z": float(goal_z) if goal_z else init_z},
                    {"use_gnss": use_gnss},
                    {"use_araim": use_araim},
                    {"allow_truth_alignment": allow_truth_alignment},
                    {"use_so3_dynamics": use_so3_dynamics},
                    {"use_iap_odom_for_planner": use_iap_odom},
                    {"planner_odom_topic": planner_odom_topic},
                    {"controller_odom_topic": truth_odom_topic if use_so3_dynamics else ""},
                    {"plant_mode": "so3_quadrotor_simulator" if use_so3_dynamics else "poscmd_2_odom_debug"},
                    {"map_source": "global_cloud_direct"},
                    {"gnss_ephemeris_source": gnss_ephemeris_source},
                    {"gnss_enabled_constellations": gnss_enabled_constellations},
                    {"gnss_rinex_nav_file": gnss_rinex_nav_file},
                    {"truth_odom_topic": truth_odom_topic},
                    {"iap_odom_topic": iap_odom_topic},
                    {"pos_cmd_topic": pos_cmd_topic},
                    {"bspline_topic": ""},
                    {"desired_odom_topic": desired_odom_topic},
                    {"integrity_topic": "/iap/integrity"},
                ],
            )
        )

    # =========================================================================
    # Ego planner (native, no custom integrity features enabled)
    # =========================================================================
    demo9_args = {
        "start_rviz": "false",
        "config_subdir": "sim_demo11",
        "use_gnss": LaunchConfiguration("use_gnss"),
        "use_araim": LaunchConfiguration("use_araim"),
        "use_iap_odom_for_planner": LaunchConfiguration("use_iap_odom_for_planner"),
        "goal_x": goal_x,
        "goal_y": goal_y,
        "goal_z": goal_z,
        "point_num": point_num,
        "point1_x": LaunchConfiguration("point1_x"),
        "point1_y": LaunchConfiguration("point1_y"),
        "point1_z": LaunchConfiguration("point1_z"),
        "point2_x": LaunchConfiguration("point2_x"),
        "point2_y": LaunchConfiguration("point2_y"),
        "point2_z": LaunchConfiguration("point2_z"),
        "point3_x": LaunchConfiguration("point3_x"),
        "point3_y": LaunchConfiguration("point3_y"),
        "point3_z": LaunchConfiguration("point3_z"),
        "point4_x": LaunchConfiguration("point4_x"),
        "point4_y": LaunchConfiguration("point4_y"),
        "point4_z": LaunchConfiguration("point4_z"),
        "point5_x": LaunchConfiguration("point5_x"),
        "point5_y": LaunchConfiguration("point5_y"),
        "point5_z": LaunchConfiguration("point5_z"),
        "point6_x": LaunchConfiguration("point6_x"),
        "point6_y": LaunchConfiguration("point6_y"),
        "point6_z": LaunchConfiguration("point6_z"),
        "run_duration_s": LaunchConfiguration("run_duration_s"),
        "use_so3_dynamics": LaunchConfiguration("use_so3_dynamics"),
        "planner_use_dynamic": "false",
        "use_dynamic_obstacles": "false",
        "map_source": LaunchConfiguration("map_source"),
        "map_generator_mode": "off",
        "allow_truth_alignment": LaunchConfiguration("allow_truth_alignment"),
        "log_phase1": LaunchConfiguration("log_phase1"),
        "enable_truth_araim_compare": "false",
        "truth_araim_compare_csv_name": "",
        "gnss_ephemeris_source": LaunchConfiguration("gnss_ephemeris_source"),
        "gnss_enabled_constellations": LaunchConfiguration("gnss_enabled_constellations"),
        "gnss_scenario_file": LaunchConfiguration("gnss_scenario_file"),
        "gnss_rinex_nav_file": LaunchConfiguration("gnss_rinex_nav_file"),
        "gnss_rinex_ephem_max_age_s": LaunchConfiguration("gnss_rinex_ephem_max_age_s"),
        "gnss_fallback_to_synthetic_on_rinex_error": LaunchConfiguration(
            "gnss_fallback_to_synthetic_on_rinex_error"
        ),
        "gnss_enable_map_occlusion": LaunchConfiguration("gnss_enable_map_occlusion"),
        "gnss_enable_skymask": LaunchConfiguration("gnss_enable_skymask"),
        "gnss_enable_nlos": LaunchConfiguration("gnss_enable_nlos"),
        "gnss_enable_multipath": LaunchConfiguration("gnss_enable_multipath"),
        "gnss_enable_fault_injection": LaunchConfiguration("gnss_enable_fault_injection"),
        "gnss_sky_dome_follow_receiver": LaunchConfiguration("gnss_sky_dome_follow_receiver"),
        "gnss_sky_dome_center_x": LaunchConfiguration("gnss_sky_dome_center_x"),
        "gnss_sky_dome_center_y": LaunchConfiguration("gnss_sky_dome_center_y"),
        "gnss_sky_dome_center_z": LaunchConfiguration("gnss_sky_dome_center_z"),
        "gnss_skyplot_origin_x": LaunchConfiguration("gnss_skyplot_origin_x"),
        "gnss_skyplot_origin_y": LaunchConfiguration("gnss_skyplot_origin_y"),
        "gnss_skyplot_origin_z": LaunchConfiguration("gnss_skyplot_origin_z"),
        "viz_status_text_use_fixed_position": "true",
        "viz_status_text_x": LaunchConfiguration("viz_status_text_x"),
        "viz_status_text_y": LaunchConfiguration("viz_status_text_y"),
        "viz_status_text_z": LaunchConfiguration("viz_status_text_z"),
        "drone_id": LaunchConfiguration("drone_id"),
        "init_x": LaunchConfiguration("init_x"),
        "init_y": LaunchConfiguration("init_y"),
        "init_z": LaunchConfiguration("init_z"),
        "map_size_x": LaunchConfiguration("map_size_x"),
        "map_size_y": LaunchConfiguration("map_size_y"),
        "map_size_z": LaunchConfiguration("map_size_z"),
        # ── Integrity features: ALL OFF (native ego planner) ──
        "planner_use_integrity_cost": "false",
        "planner_lambda_integrity": "0.0",
        "planner_use_integrity_front_search": "false",
        "planner_use_integrity_global_search": "false",
        "planner_lambda_integrity_front": "0.0",
        "planner_integrity_front_cost_topic": "",
        "planner_integrity_front_nearest_radius_m": "0.0",
        "planner_integrity_front_stale_timeout_s": "0.0",
        "planner_integrity_front_cost_max": "0.0",
        "planner_integrity_global_astar_step_m": "0.0",
        "planner_integrity_global_max_waypoints": "0",
        "planner_integrity_field_stale_timeout_s": "0.0",
        "planner_integrity_nearest_radius_m": "0.0",
        "planner_integrity_cost_max": "0.0",
        "planner_integrity_grad_norm_max": "0.0",
        "risk_overlay_enable": "false",
        "risk_overlay_use_for_astar": "false",
        "risk_overlay_use_for_bspline": "false",
        "risk_overlay_topic": "",
        "risk_overlay_lambda_unknown": "0.0",
        "risk_overlay_lambda_stale": "0.0",
        "risk_overlay_stale_timeout_s": "0.0",
        "risk_overlay_stale_tau_s": "0.0",
        "risk_overlay_r_soft": "0.0",
        "risk_overlay_w_soft": "0.0",
        "risk_overlay_w_hard": "0.0",
        "risk_overlay_c_unsafe": "0.0",
        "risk_overlay_eps_al_m": "0.0",
        "risk_overlay_gamma_h": "0.0",
        "risk_overlay_gamma_v": "0.0",
        "risk_overlay_drone_radius_m": "0.0",
        "risk_overlay_safety_buffer_m": "0.0",
        "risk_overlay_clearance_max_m": "0.0",
        "risk_overlay_clearance_unknown_m": "0.0",
        "risk_overlay_edge_sample_alpha": "0.0",
        "risk_overlay_bspline_samples_per_segment": "0",
        "risk_overlay_debug_publish": "false",
        "risk_overlay_debug_topic": "",
        "risk_overlay_debug_publish_hz": "0",
        "risk_overlay_debug_color_mode": "",
        "risk_overlay_debug_cost_max": "0.0",
        "planner_start_delay_s": LaunchConfiguration("planner_start_delay_s"),
        "enable_preflight_takeoff": LaunchConfiguration("enable_preflight_takeoff"),
        "preflight_ground_z": LaunchConfiguration("preflight_ground_z"),
        "preflight_ground_hold_s": LaunchConfiguration("preflight_ground_hold_s"),
        "preflight_takeoff_duration_s": LaunchConfiguration("preflight_takeoff_duration_s"),
        "preflight_hover_s": LaunchConfiguration("preflight_hover_s"),
        "preflight_cmd_rate_hz": LaunchConfiguration("preflight_cmd_rate_hz"),
        "iap_odom_freshness_sec": LaunchConfiguration("iap_odom_freshness_sec"),
    }
    actions.append(
        LogInfo(
            msg=(
                f"[demo11_araim] planner: goal=({goal_x},{goal_y},{goal_z}), "
                f"point_num={point_num}, odom={'iap' if use_iap_odom else 'truth'}"
            )
        )
    )
    actions.append(
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            iap_share, "launch", "demo9_ego_planner_closed_loop.launch.py"
                        )
                    ),
                    launch_arguments=demo9_args.items(),
                ),
            ],
        )
    )

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="demo11_araim_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(iap_share, "config", "sim_demo11", "demo11_integrity_corridor.rviz"),
                ],
            )
        )

    if run_duration_s > 0.0:
        actions.append(
            TimerAction(
                period=run_duration_s,
                actions=[EmitEvent(event=Shutdown(reason="demo11_araim run_duration_s elapsed"))],
            )
        )

    return actions


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    fastdds_profile = os.path.join(iap_share, "config", "sim_ego", "fastdds_udp_only.xml")
    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("config_subdir", default_value="sim_demo11"),
        DeclareLaunchArgument("use_gnss", default_value="true"),
        DeclareLaunchArgument("use_araim", default_value="true"),
        DeclareLaunchArgument("use_iap_odom_for_planner", default_value="false"),
        DeclareLaunchArgument("init_x", default_value="-12.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="1.2"),
        DeclareLaunchArgument("goal_x", default_value="12.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        DeclareLaunchArgument("point_num", default_value="1"),
        DeclareLaunchArgument("point1_x", default_value="9.5"),
        DeclareLaunchArgument("point1_y", default_value="0.0"),
        DeclareLaunchArgument("point1_z", default_value="1.2"),
        DeclareLaunchArgument("point2_x", default_value="16.0"),
        DeclareLaunchArgument("point2_y", default_value="0.0"),
        DeclareLaunchArgument("point2_z", default_value="1.2"),
        DeclareLaunchArgument("point3_x", default_value="16.0"),
        DeclareLaunchArgument("point3_y", default_value="0.0"),
        DeclareLaunchArgument("point3_z", default_value="1.2"),
        DeclareLaunchArgument("point4_x", default_value="16.0"),
        DeclareLaunchArgument("point4_y", default_value="0.0"),
        DeclareLaunchArgument("point4_z", default_value="1.2"),
        DeclareLaunchArgument("point5_x", default_value="16.0"),
        DeclareLaunchArgument("point5_y", default_value="0.0"),
        DeclareLaunchArgument("point5_z", default_value="1.2"),
        DeclareLaunchArgument("point6_x", default_value="16.0"),
        DeclareLaunchArgument("point6_y", default_value="0.0"),
        DeclareLaunchArgument("point6_z", default_value="1.2"),
        DeclareLaunchArgument("run_duration_s", default_value="90"),
        DeclareLaunchArgument("use_so3_dynamics", default_value="true"),
        DeclareLaunchArgument("allow_truth_alignment", default_value="true"),
        DeclareLaunchArgument("log_phase1", default_value="true"),
        DeclareLaunchArgument("gnss_ephemeris_source", default_value="rinex"),
        DeclareLaunchArgument("gnss_enabled_constellations", default_value="GPS,BDS,GAL,GLO"),
        DeclareLaunchArgument(
            "gnss_scenario_file",
            default_value=os.path.join(iap_share, "config", "gnss_sim", "demo7_skymask_nlos.yaml"),
        ),
        DeclareLaunchArgument(
            "gnss_rinex_nav_file",
            default_value="/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx",
        ),
        DeclareLaunchArgument("gnss_rinex_ephem_max_age_s", default_value="7200.0"),
        DeclareLaunchArgument("gnss_fallback_to_synthetic_on_rinex_error", default_value="false"),
        DeclareLaunchArgument("gnss_enable_map_occlusion", default_value="true"),
        DeclareLaunchArgument("gnss_enable_skymask", default_value="true"),
        DeclareLaunchArgument("gnss_enable_nlos", default_value="true"),
        DeclareLaunchArgument("gnss_enable_multipath", default_value="true"),
        DeclareLaunchArgument("gnss_enable_fault_injection", default_value="true"),
        DeclareLaunchArgument("gnss_sky_dome_follow_receiver", default_value="false"),
        DeclareLaunchArgument("gnss_sky_dome_center_x", default_value="0.0"),
        DeclareLaunchArgument("gnss_sky_dome_center_y", default_value="0.0"),
        DeclareLaunchArgument("gnss_sky_dome_center_z", default_value="0.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_x", default_value="0.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_y", default_value="22.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_z", default_value="5.0"),
        DeclareLaunchArgument("viz_status_text_use_fixed_position", default_value="true"),
        DeclareLaunchArgument("viz_status_text_x", default_value="15.0"),
        DeclareLaunchArgument("viz_status_text_y", default_value="0.0"),
        DeclareLaunchArgument("viz_status_text_z", default_value="5.0"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="30.0"),
        DeclareLaunchArgument("map_size_z", default_value="3.5"),
        DeclareLaunchArgument("enable_preflight_takeoff", default_value="true"),
        DeclareLaunchArgument("preflight_ground_z", default_value="0.0"),
        DeclareLaunchArgument("preflight_ground_hold_s", default_value="10.0"),
        DeclareLaunchArgument("preflight_takeoff_duration_s", default_value="5.0"),
        DeclareLaunchArgument("preflight_hover_s", default_value="30.0"),
        DeclareLaunchArgument("preflight_cmd_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("corridor_map_resolution_m", default_value="0.1"),
        DeclareLaunchArgument("corridor_map_publish_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("forest_size_x_m", default_value="20.0"),
        DeclareLaunchArgument("forest_size_y_m", default_value="20.0"),
        DeclareLaunchArgument("tree_density_lower_left_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_lower_right_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_upper_left_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_upper_right_per_m2", default_value="0.5"),
        DeclareLaunchArgument("stratified_cell_size_m", default_value="1.0"),
        DeclareLaunchArgument("canopy_density_lower_left", default_value="0.1"),
        DeclareLaunchArgument("canopy_density_lower_right", default_value="0.1"),
        DeclareLaunchArgument("canopy_density_upper_left", default_value="0.6"),
        DeclareLaunchArgument("canopy_density_upper_right", default_value="0.6"),
        DeclareLaunchArgument("canopy_hemisphere_radius_min_m", default_value="0.5"),
        DeclareLaunchArgument("canopy_hemisphere_radius_max_m", default_value="1.5"),
        DeclareLaunchArgument("canopy_leaf_ball_radius_m", default_value="0.22"),
        DeclareLaunchArgument("canopy_ball_spacing_ratio", default_value="1.2"),
        DeclareLaunchArgument("canopy_resolution_m", default_value="0.15"),
        DeclareLaunchArgument("forest_random_seed", default_value="11"),
        DeclareLaunchArgument("trunk_radius_m", default_value="0.14"),
        DeclareLaunchArgument("trunk_min_height_m", default_value="1.5"),
        DeclareLaunchArgument("trunk_max_height_m", default_value="3.0"),
        DeclareLaunchArgument("terminal_wall_enabled", default_value="true"),
        DeclareLaunchArgument("terminal_wall_x_m", default_value="13.5"),
        DeclareLaunchArgument("terminal_wall_y_m", default_value="0.0"),
        DeclareLaunchArgument("terminal_wall_width_y_m", default_value="10.0"),
        DeclareLaunchArgument("terminal_wall_z_min_m", default_value="0.0"),
        DeclareLaunchArgument("terminal_wall_z_max_m", default_value="3.2"),
        DeclareLaunchArgument("terminal_wall_thickness_x_m", default_value="0.20"),
        DeclareLaunchArgument("terminal_wall_resolution_m", default_value="0.10"),
        DeclareLaunchArgument("terminal_wall_feature_depth_x_m", default_value="0.65"),
        DeclareLaunchArgument("terminal_wall_feature_count", default_value="48"),
        DeclareLaunchArgument("terminal_wall_feature_seed", default_value="11022"),
        DeclareLaunchArgument("map_source", default_value="global_cloud_direct"),
        DeclareLaunchArgument("planner_start_delay_s", default_value="0.0"),
        DeclareLaunchArgument("iap_odom_freshness_sec", default_value="0.5"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        OpaqueFunction(function=_launch_setup),
    ])
