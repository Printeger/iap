import json
import os
import shutil
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _runtime_config(
    context,
    use_gnss,
    use_araim,
    allow_truth_alignment,
    enable_truth_araim_compare=False,
    truth_araim_compare_csv_name="demo9_araim_truth_compare.csv",
    config_subdir="sim_demo9",
):
    iap_share = Path(get_package_share_directory("iap"))
    base_config = iap_share / "config"
    config_name = config_subdir.replace("/", "_").replace("\\", "_")
    runtime_root = Path("/tmp") / f"iap_{config_name}_config_{os.getpid()}_{int(time.time() * 1000)}"
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
    if enable_truth_araim_compare:
        insert_at = len(modules) - 1 if "libsim_extension.so" in modules else len(modules)
        modules.insert(insert_at, "libdemo8_truth_araim_extension.so")
    config_ros["glim_ros"]["extension_modules"] = modules
    config_ros["glim_ros"]["sim"]["align_planner_odom_to_truth"] = allow_truth_alignment
    config_ros["glim_ros"]["sim"]["demo8_truth_araim"] = {
        "enable": bool(enable_truth_araim_compare),
        "csv_name": str(truth_araim_compare_csv_name),
        "truth_odom_topic": "/sim/drone_0/truth_odom",
        "truth_match_tolerance_s": 0.05,
        "truth_cache_duration_s": 8.0,
        "origin_lat_deg": 31.2304,
        "origin_lon_deg": 121.4737,
        "origin_alt_m": 25.0,
        "enable_truth_araim_markers": False,
        "truth_araim_marker_topic": "/iap/araim_truth_envelopes",
        "truth_araim_marker_history_size": 60,
        "truth_araim_marker_publish_period_s": 0.5,
        "truth_araim_marker_min_pl_m": 0.05,
        "truth_araim_marker_max_pl_m": 130.0,
        "truth_araim_marker_show_gnss": True,
        "truth_araim_marker_show_lidar": True,
        "truth_araim_marker_show_final": False,
        "truth_araim_marker_mode": "fault_only",
        "truth_araim_marker_fault_history_size": 20,
        "truth_araim_marker_fault_lifetime_s": 8.0,
        "truth_araim_marker_show_fault_envelope": False,
        "truth_araim_marker_fault_radius_m": 0.8,
        "truth_araim_marker_final_hal_m": 10.0,
        "truth_araim_marker_final_val_m": 20.0,
    }
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
    ego_share = get_package_share_directory("ego_planner")
    so3_control_share = get_package_share_directory("so3_control")
    local_sensing_share = get_package_share_directory("local_sensing")

    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    use_gnss = _as_bool(LaunchConfiguration("use_gnss").perform(context))
    use_araim = _as_bool(LaunchConfiguration("use_araim").perform(context))
    use_iap_odom = _as_bool(LaunchConfiguration("use_iap_odom_for_planner").perform(context))
    use_so3_dynamics = _as_bool(LaunchConfiguration("use_so3_dynamics").perform(context))
    planner_use_dynamic = _as_bool(LaunchConfiguration("planner_use_dynamic").perform(context))
    planner_use_integrity_cost = LaunchConfiguration("planner_use_integrity_cost").perform(context)
    planner_lambda_integrity = LaunchConfiguration("planner_lambda_integrity").perform(context)
    planner_integrity_field_stale_timeout_s = LaunchConfiguration(
        "planner_integrity_field_stale_timeout_s"
    ).perform(context)
    planner_integrity_nearest_radius_m = LaunchConfiguration(
        "planner_integrity_nearest_radius_m"
    ).perform(context)
    planner_integrity_cost_max = LaunchConfiguration("planner_integrity_cost_max").perform(context)
    planner_integrity_grad_norm_max = LaunchConfiguration(
        "planner_integrity_grad_norm_max"
    ).perform(context)
    planner_use_integrity_front_search = LaunchConfiguration(
        "planner_use_integrity_front_search"
    ).perform(context)
    planner_use_integrity_global_search = LaunchConfiguration(
        "planner_use_integrity_global_search"
    ).perform(context)
    planner_lambda_integrity_front = LaunchConfiguration(
        "planner_lambda_integrity_front"
    ).perform(context)
    planner_integrity_front_cost_topic = LaunchConfiguration(
        "planner_integrity_front_cost_topic"
    ).perform(context)
    planner_integrity_front_nearest_radius_m = LaunchConfiguration(
        "planner_integrity_front_nearest_radius_m"
    ).perform(context)
    planner_integrity_front_stale_timeout_s = LaunchConfiguration(
        "planner_integrity_front_stale_timeout_s"
    ).perform(context)
    planner_integrity_front_cost_max = LaunchConfiguration(
        "planner_integrity_front_cost_max"
    ).perform(context)
    planner_integrity_global_astar_step_m = LaunchConfiguration(
        "planner_integrity_global_astar_step_m"
    ).perform(context)
    planner_integrity_global_max_waypoints = LaunchConfiguration(
        "planner_integrity_global_max_waypoints"
    ).perform(context)
    planner_start_delay_s = max(
        0.0, float(LaunchConfiguration("planner_start_delay_s").perform(context))
    )
    enable_preflight_takeoff = _as_bool(
        LaunchConfiguration("enable_preflight_takeoff").perform(context)
    )
    preflight_ground_z = float(LaunchConfiguration("preflight_ground_z").perform(context))
    preflight_ground_hold_s = max(
        0.0, float(LaunchConfiguration("preflight_ground_hold_s").perform(context))
    )
    preflight_takeoff_duration_s = max(
        0.1, float(LaunchConfiguration("preflight_takeoff_duration_s").perform(context))
    )
    preflight_hover_s = max(
        0.0, float(LaunchConfiguration("preflight_hover_s").perform(context))
    )
    preflight_cmd_rate_hz = max(
        1.0, float(LaunchConfiguration("preflight_cmd_rate_hz").perform(context))
    )
    iap_odom_freshness_sec = float(
        LaunchConfiguration("iap_odom_freshness_sec").perform(context)
    )
    use_dynamic_obstacles = _as_bool(LaunchConfiguration("use_dynamic_obstacles").perform(context))
    allow_truth_alignment = _as_bool(LaunchConfiguration("allow_truth_alignment").perform(context))
    log_phase1 = _as_bool(LaunchConfiguration("log_phase1").perform(context))
    enable_truth_araim_compare = _as_bool(
        LaunchConfiguration("enable_truth_araim_compare").perform(context)
    )
    truth_araim_compare_csv_name = LaunchConfiguration(
        "truth_araim_compare_csv_name"
    ).perform(context)

    drone_id = LaunchConfiguration("drone_id").perform(context)
    goal_x = LaunchConfiguration("goal_x").perform(context)
    goal_y = LaunchConfiguration("goal_y").perform(context)
    goal_z = LaunchConfiguration("goal_z").perform(context)
    point_num = LaunchConfiguration("point_num").perform(context)
    waypoint_values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "point1_x",
            "point1_y",
            "point1_z",
            "point2_x",
            "point2_y",
            "point2_z",
            "point3_x",
            "point3_y",
            "point3_z",
            "point4_x",
            "point4_y",
            "point4_z",
            "point5_x",
            "point5_y",
            "point5_z",
            "point6_x",
            "point6_y",
            "point6_z",
        )
    }
    run_duration_s = float(LaunchConfiguration("run_duration_s").perform(context))
    map_source = LaunchConfiguration("map_source").perform(context)
    gnss_ephemeris_source = LaunchConfiguration("gnss_ephemeris_source").perform(context)
    gnss_enabled_constellations = LaunchConfiguration("gnss_enabled_constellations").perform(context)
    gnss_scenario_file = LaunchConfiguration("gnss_scenario_file").perform(context)
    gnss_rinex_nav_file = LaunchConfiguration("gnss_rinex_nav_file").perform(context)
    gnss_rinex_ephem_max_age_s = LaunchConfiguration("gnss_rinex_ephem_max_age_s").perform(context)
    gnss_fallback_to_synthetic = _as_bool(
        LaunchConfiguration("gnss_fallback_to_synthetic_on_rinex_error").perform(context)
    )
    gnss_enable_map_occlusion = _as_bool(
        LaunchConfiguration("gnss_enable_map_occlusion").perform(context)
    )
    gnss_enable_skymask = _as_bool(
        LaunchConfiguration("gnss_enable_skymask").perform(context)
    )
    gnss_enable_nlos = _as_bool(
        LaunchConfiguration("gnss_enable_nlos").perform(context)
    )
    gnss_enable_multipath = _as_bool(
        LaunchConfiguration("gnss_enable_multipath").perform(context)
    )
    gnss_enable_fault_injection = _as_bool(
        LaunchConfiguration("gnss_enable_fault_injection").perform(context)
    )
    gnss_sky_dome_follow_receiver = _as_bool(
        LaunchConfiguration("gnss_sky_dome_follow_receiver").perform(context)
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
    fixed_status_text = _as_bool(
        LaunchConfiguration("viz_status_text_use_fixed_position").perform(context)
    )
    status_text_position = (
        float(LaunchConfiguration("viz_status_text_x").perform(context)),
        float(LaunchConfiguration("viz_status_text_y").perform(context)),
        float(LaunchConfiguration("viz_status_text_z").perform(context)),
    )

    map_size_x = LaunchConfiguration("map_size_x").perform(context)
    map_size_y = LaunchConfiguration("map_size_y").perform(context)
    map_size_z = LaunchConfiguration("map_size_z").perform(context)
    map_generator_mode = LaunchConfiguration("map_generator_mode").perform(context).strip().lower()
    init_x = LaunchConfiguration("init_x").perform(context)
    init_y = LaunchConfiguration("init_y").perform(context)
    init_z = LaunchConfiguration("init_z").perform(context)
    goal_x_f = float(goal_x)
    goal_y_f = float(goal_y)
    goal_z_f = float(goal_z)
    init_x_f = float(init_x)
    init_y_f = float(init_y)
    init_z_f = float(init_z)
    plant_init_z_f = preflight_ground_z if enable_preflight_takeoff else init_z_f
    effective_planner_start_delay_s = planner_start_delay_s
    if enable_preflight_takeoff:
        effective_planner_start_delay_s += (
            preflight_ground_hold_s + preflight_takeoff_duration_s + preflight_hover_s
        )
    map_size_x_f = float(map_size_x)
    map_size_y_f = float(map_size_y)
    map_size_z_f = float(map_size_z)

    truth_odom_topic = "/sim/drone_0/truth_odom"
    iap_odom_topic = "/drone_0_visual_slam/odom"
    preflight_control_odom_topic = "/demo9/preflight_control_odom"
    planner_odom_topic = iap_odom_topic if use_iap_odom else truth_odom_topic
    use_preflight_control_odom_mux = bool(
        enable_preflight_takeoff and use_iap_odom and use_so3_dynamics
    )
    controller_odom_topic = (
        preflight_control_odom_topic
        if use_preflight_control_odom_mux
        else (planner_odom_topic if use_so3_dynamics else "")
    )
    plant_mode = "so3_quadrotor_simulator" if use_so3_dynamics else "poscmd_2_odom_debug"
    sim_imu_topic = "/sim/drone_0/imu"
    iap_imu_topic = "/sim/drone_0/imu_iap"
    sim_lidar_topic = "/sim/drone_0/lidar"
    iap_lidar_topic = "/sim/drone_0/lidar_body"
    sim_depth_topic = "/sim/drone_0/depth"
    pos_cmd_topic = "/drone_0_planning/pos_cmd"
    bspline_topic = "/drone_0_planning/bspline"
    desired_odom_topic = "/demo9/desired/odom"
    so3_cmd_topic = "/demo9/so3_cmd"

    if map_source == "local_sensing_cloud":
        ego_cloud_topic = sim_lidar_topic
    elif map_source == "global_cloud_direct":
        ego_cloud_topic = "/map_generator/global_cloud"
    else:
        raise RuntimeError(
            "map_source must be 'local_sensing_cloud' or 'global_cloud_direct'"
        )

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

    config_subdir = LaunchConfiguration("config_subdir").perform(context)
    runtime_config_path = _runtime_config(
        context,
        use_gnss=use_gnss,
        use_araim=use_araim,
        allow_truth_alignment=allow_truth_alignment,
        enable_truth_araim_compare=enable_truth_araim_compare,
        truth_araim_compare_csv_name=truth_araim_compare_csv_name,
        config_subdir=config_subdir,
    )

    camera_file = os.path.join(local_sensing_share, "config", "camera.yaml")
    gains_file = os.path.join(so3_control_share, "config", "gains_hummingbird.yaml")
    corrections_file = os.path.join(so3_control_share, "config", "corrections_hummingbird.yaml")

    ego_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ego_share, "launch", "advanced_param.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
            "map_size_x_": map_size_x,
            "map_size_y_": map_size_y,
            "map_size_z_": map_size_z,
            "odometry_topic": planner_odom_topic,
            "obj_num_set": "10" if use_dynamic_obstacles else "0",
            "camera_pose_topic": "/drone_0_pcl_render_node/camera_pose",
            "depth_topic": sim_depth_topic,
            "cloud_topic": ego_cloud_topic,
            "cx": "321.04638671875",
            "cy": "243.44969177246094",
            "fx": "387.229248046875",
            "fy": "387.229248046875",
            "max_vel": "2.0",
            "max_acc": "3.0",
            "planning_horizon": "7.5",
            "use_distinctive_trajs": "True",
            "flight_type": "2",
            "point_num": point_num,
            "point0_x": goal_x,
            "point0_y": goal_y,
            "point0_z": goal_z,
            "point1_x": waypoint_values["point1_x"],
            "point1_y": waypoint_values["point1_y"],
            "point1_z": waypoint_values["point1_z"],
            "point2_x": waypoint_values["point2_x"],
            "point2_y": waypoint_values["point2_y"],
            "point2_z": waypoint_values["point2_z"],
            "point3_x": waypoint_values["point3_x"],
            "point3_y": waypoint_values["point3_y"],
            "point3_z": waypoint_values["point3_z"],
            "point4_x": waypoint_values["point4_x"],
            "point4_y": waypoint_values["point4_y"],
            "point4_z": waypoint_values["point4_z"],
            "point5_x": waypoint_values["point5_x"],
            "point5_y": waypoint_values["point5_y"],
            "point5_z": waypoint_values["point5_z"],
            "point6_x": waypoint_values["point6_x"],
            "point6_y": waypoint_values["point6_y"],
            "point6_z": waypoint_values["point6_z"],
            "use_integrity_cost": planner_use_integrity_cost,
            "lambda_integrity": planner_lambda_integrity,
            "integrity_field_stale_timeout_s": planner_integrity_field_stale_timeout_s,
            "integrity_nearest_radius_m": planner_integrity_nearest_radius_m,
            "integrity_cost_max": planner_integrity_cost_max,
            "integrity_grad_norm_max": planner_integrity_grad_norm_max,
            "use_integrity_front_search": planner_use_integrity_front_search,
            "use_integrity_global_search": planner_use_integrity_global_search,
            "lambda_integrity_front": planner_lambda_integrity_front,
            "integrity_front_cost_topic": planner_integrity_front_cost_topic,
            "integrity_front_nearest_radius_m": planner_integrity_front_nearest_radius_m,
            "integrity_front_stale_timeout_s": planner_integrity_front_stale_timeout_s,
            "integrity_front_cost_max": planner_integrity_front_cost_max,
            "integrity_global_astar_step_m": planner_integrity_global_astar_step_m,
            "integrity_global_max_waypoints": planner_integrity_global_max_waypoints,
        }.items(),
    )
    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        name=f"drone_{drone_id}_traj_server",
        output="screen",
        remappings=[
            ("planning/bspline", bspline_topic),
            ("position_cmd", pos_cmd_topic),
            ("/position_cmd", pos_cmd_topic),
        ],
        parameters=[{"traj_server/time_forward": 1.0}],
    )
    planner_launch_actions = [ego_planner_launch, traj_server_node]
    if effective_planner_start_delay_s > 0.0:
        planner_launch_actions = [
            LogInfo(
                msg=(
                    f"[demo9] delaying EGO planner start by "
                    f"{effective_planner_start_delay_s:.2f}s"
                )
            ),
            TimerAction(period=effective_planner_start_delay_s, actions=planner_launch_actions),
        ]

    actions = [
        LogInfo(msg=f"[demo9] runtime IAP config: {runtime_config_path}"),
        LogInfo(msg=f"[demo9] planner/controller odom feedback: {planner_odom_topic}"),
        LogInfo(
            msg=(
                f"[demo9] preset waypoint0 from goal argument: "
                f"({goal_x}, {goal_y}, {goal_z}), point_num={point_num}"
            )
        ),
    ]
    if enable_preflight_takeoff:
        actions.append(
            LogInfo(
                msg=(
                    f"[demo9] preflight enabled: ground hold {preflight_ground_hold_s:.2f}s "
                    f"at z={preflight_ground_z:.2f}, takeoff {preflight_takeoff_duration_s:.2f}s "
                    f"to ({init_x_f:.2f}, {init_y_f:.2f}, {init_z_f:.2f}), hover "
                    f"{preflight_hover_s:.2f}s before planner"
                )
            )
        )
    if use_preflight_control_odom_mux:
        actions.append(
            LogInfo(
                msg=(
                    f"[demo9] SO3 controller uses truth odom during preflight, then "
                    f"switches to IAP odom on {controller_odom_topic}"
                )
            )
        )
    if map_generator_mode not in ("random_forest", "off"):
        raise RuntimeError("map_generator_mode must be 'random_forest' or 'off'")
    if map_generator_mode == "off":
        actions.append(LogInfo(msg="[demo9] map_generator random_forest disabled by launch arg"))

    if not planner_use_dynamic:
        actions.append(
            LogInfo(
                msg="[demo9] WARNING: planner_use_dynamic is deprecated and no longer controls use_so3_dynamics"
            )
        )
    if not use_iap_odom:
        actions.append(
            LogInfo(msg="[demo9] DEBUG: use_iap_odom_for_planner=false, planner/controller use truth odom")
        )
    if not use_so3_dynamics:
        actions.append(
            LogInfo(msg="[demo9] DEBUG: use_so3_dynamics=false is not the Phase 1 acceptance path")
        )
    if map_source == "global_cloud_direct":
        actions.append(
            LogInfo(msg="[demo9] DEBUG: ego planner receives /map_generator/global_cloud directly")
        )

    actions.extend([
        Node(
            package="map_generator",
            executable="random_forest",
            name="demo9_random_forest",
            output="screen",
            condition=IfCondition("true" if map_generator_mode == "random_forest" else "false"),
            remappings=[("odometry", truth_odom_topic)],
            parameters=[
                {"init_state_x": init_x_f},
                {"init_state_y": init_y_f},
                {"target_state_x": goal_x_f},
                {"target_state_y": goal_y_f},
                {"clear_radius": 3.0},
                {"map/x_size": map_size_x_f},
                {"map/y_size": map_size_y_f},
                {"map/z_size": map_size_z_f},
                {"map/resolution": 0.1},
                {"ObstacleShape/seed": 20260502},
                {"map/obs_num": 55},
                {"ObstacleShape/lower_rad": 0.45},
                {"ObstacleShape/upper_rad": 0.75},
                {"ObstacleShape/lower_hei": 0.0},
                {"ObstacleShape/upper_hei": 2.8},
                {"map/circle_num": 55},
                {"ObstacleShape/radius_l": 0.6},
                {"ObstacleShape/radius_h": 1.0},
                {"ObstacleShape/z_l": 0.7},
                {"ObstacleShape/z_h": 1.3},
                {"ObstacleShape/theta": 0.5},
                {"ObstacleShape/grow_from_ground": True},
                {"ObstacleShape/ground_z": 0.0},
                {"ZAnchor/enable": True},
                {"ZAnchor/center_x": init_x_f},
                {"ZAnchor/center_y": init_y_f},
                {"ZAnchor/radius": 4.0},
                {"ZAnchor/z_low": 0.7},
                {"ZAnchor/z_high": 3.1},
                {"ZAnchor/post_num": 12},
                {"ZAnchor/post_step": 0.1},
                {"sensing/radius": 10.0},
                {"sensing/rate": 1.0},
                {"min_distance": 1.0},
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
                {"map/x_size": map_size_x_f},
                {"map/y_size": map_size_y_f},
                {"map/z_size": map_size_z_f},
                {"map/resolution": 0.1},
                camera_file,
            ],
        ),
        Node(
            package="iap",
            executable="demo4_lidar_body_bridge",
            name="demo9_lidar_body_bridge",
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
            name="demo9_iap_rosnode",
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
            name="demo9_preflight_takeoff_cmd_publisher",
            output="screen",
            condition=IfCondition("true" if enable_preflight_takeoff else "false"),
            remappings=[("position_cmd", pos_cmd_topic)],
            parameters=[
                {"ground_x": init_x_f},
                {"ground_y": init_y_f},
                {"ground_z": preflight_ground_z},
                {"hover_x": init_x_f},
                {"hover_y": init_y_f},
                {"hover_z": init_z_f},
                {"ground_hold_duration_s": preflight_ground_hold_s},
                {"takeoff_duration_s": preflight_takeoff_duration_s},
                {"hover_duration_s": preflight_hover_s},
                {"publish_rate_hz": preflight_cmd_rate_hz},
                {"trajectory_id": 9001},
                {"frame_id": "map"},
                {"stop_after_sequence": True},
            ],
        ),
        Node(
            package="iap",
            executable="demo3_odom_mux",
            name="demo9_preflight_control_odom_mux",
            output="screen",
            condition=IfCondition("true" if use_preflight_control_odom_mux else "false"),
            parameters=[
                {"truth_odom_topic": truth_odom_topic},
                {"iap_odom_topic": iap_odom_topic},
                {"control_odom_topic": controller_odom_topic},
                {"iap_lock_sample_count": 3},
                {"iap_freshness_sec": iap_odom_freshness_sec},
                {"truth_bootstrap_min_duration_sec": effective_planner_start_delay_s},
            ],
        ),
        *planner_launch_actions,
        Node(
            package="poscmd_2_odom",
            executable="poscmd_2_odom",
            name="demo9_desired_poscmd_to_odom",
            output="screen",
            remappings=[
                ("command", pos_cmd_topic),
                ("odometry", desired_odom_topic),
            ],
            parameters=[
                {"init_x": init_x_f},
                {"init_y": init_y_f},
                {"init_z": plant_init_z_f},
            ],
        ),
        _odom_visualization_node(
            "demo9_odom_visualization",
            planner_odom_topic,
            pos_cmd_topic,
            "/demo9/drone",
            (0.2, 1.0, 0.4),
            int(drone_id),
            fixed_status_text,
            status_text_position,
        ),
        _odom_visualization_node(
            "demo9_truth_odom_visualization",
            truth_odom_topic,
            pos_cmd_topic,
            "/demo9/truth",
            (1.0, 0.15, 0.1),
            10,
        ),
        _odom_visualization_node(
            "demo9_desired_odom_visualization",
            desired_odom_topic,
            pos_cmd_topic,
            "/demo9/desired",
            (1.0, 0.86, 0.05),
            20,
        ),
        Node(
            package="gnss_sim",
            executable="gnss_sim_node",
            name="demo9_gnss_sim_node",
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
                {"sky_dome_follow_receiver": gnss_sky_dome_follow_receiver},
                {"sky_dome_center_enu": gnss_sky_dome_center_enu},
                {"skyplot_origin_enu": gnss_skyplot_origin_enu},
                {"status_text_use_fixed_position": fixed_status_text},
                {"status_text_position_enu": list(status_text_position)},
                {"enable_map_occlusion": gnss_enable_map_occlusion},
                {"enable_skymask": gnss_enable_skymask},
                {"enable_nlos": gnss_enable_nlos},
                {"enable_multipath": gnss_enable_multipath},
                {"enable_fault_injection": gnss_enable_fault_injection},
                {"map_cloud_topic": "/map_generator/global_cloud"},
                {"multipath_amp_m": 0.5},
            ],
        ),
    ])

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
                    ("force_disturbance", "/demo9/force_disturbance"),
                    ("moment_disturbance", "/demo9/moment_disturbance"),
                ],
                parameters=[
                    {"quadrotor_name": f"drone_{drone_id}"},
                    {"rate/simulation": 1000.0},
                    {"rate/odom": 100.0},
                    {"simulator/init_state_x": init_x_f},
                    {"simulator/init_state_y": init_y_f},
                    {"simulator/init_state_z": plant_init_z_f},
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
                name="demo9_so3_control_container",
                namespace="",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="so3_control",
                        plugin="SO3ControlComponent",
                        name="demo9_so3_control_component",
                        parameters=[
                            {"quadrotor_name": f"drone_{drone_id}"},
                            {"so3_control/init_state_x": init_x_f},
                            {"so3_control/init_state_y": init_y_f},
                            {"so3_control/init_state_z": plant_init_z_f},
                            {"mass": 0.98},
                            {"use_angle_corrections": False},
                            {"use_external_yaw": False},
                            {"gains/rot/z": 1.0},
                            {"gains/ang/z": 0.1},
                            gains_file,
                            corrections_file,
                        ],
                        remappings=[
                            ("odom", controller_odom_topic),
                            ("position_cmd", pos_cmd_topic),
                            ("motors", "/demo9/motors"),
                            ("corrections", "/demo9/corrections"),
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
                name="demo9_poscmd_2_odom_debug_plant",
                output="screen",
                remappings=[
                    ("command", pos_cmd_topic),
                    ("odometry", truth_odom_topic),
                ],
                parameters=[
                    {"init_x": init_x_f},
                    {"init_y": init_y_f},
                    {"init_z": plant_init_z_f},
                ],
            )
        )

    if use_dynamic_obstacles:
        actions.append(
            Node(
                package="plan_env",
                executable="obj_generator",
                name="demo9_obj_generator",
                output="screen",
                parameters=[
                    {"obj_generator/obj_num": 10},
                    {"obj_generator/x_size": 12.0},
                    {"obj_generator/y_size": 12.0},
                    {"obj_generator/h_size": 1.0},
                    {"obj_generator/vel": 1.5},
                    {"obj_generator/yaw_dot": 2.0},
                    {"obj_generator/acc_r1": 1.0},
                    {"obj_generator/acc_r2": 1.0},
                    {"obj_generator/acc_z": 0.0},
                    {"obj_generator/scale1": 0.5},
                    {"obj_generator/scale2": 1.0},
                    {"obj_generator/interval": 100.0},
                    {"obj_generator/input_type": 1},
                ],
            )
        )

    if log_phase1:
        actions.append(
            Node(
                package="iap_phase1_tools",
                executable="phase1_closed_loop_logger",
                name="phase1_closed_loop_logger",
                output="screen",
                parameters=[
                    {"log_root": "/home/dev/ws_iap/src/iap/log"},
                    {"run_duration_s": run_duration_s},
                    {"goal_x": goal_x_f},
                    {"goal_y": goal_y_f},
                    {"goal_z": goal_z_f},
                    {"use_gnss": use_gnss},
                    {"use_araim": use_araim},
                    {"allow_truth_alignment": allow_truth_alignment},
                    {"use_so3_dynamics": use_so3_dynamics},
                    {"use_iap_odom_for_planner": use_iap_odom},
                    {"planner_odom_topic": planner_odom_topic},
                    {"controller_odom_topic": controller_odom_topic},
                    {"plant_mode": plant_mode},
                    {"map_source": map_source},
                    {"gnss_ephemeris_source": gnss_ephemeris_source},
                    {"gnss_enabled_constellations": gnss_enabled_constellations},
                    {"gnss_rinex_nav_file": gnss_rinex_nav_file},
                    {"truth_odom_topic": truth_odom_topic},
                    {"iap_odom_topic": iap_odom_topic},
                    {"pos_cmd_topic": pos_cmd_topic},
                    {"bspline_topic": bspline_topic},
                    {"desired_odom_topic": desired_odom_topic},
                    {"integrity_topic": "/iap/integrity"},
                ],
            )
        )

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="demo9_rviz",
                output="screen",
                arguments=["-d", os.path.join(iap_share, "config", "sim_demo9", "demo9_gnss.rviz")],
            )
        )

    if run_duration_s > 0.0:
        actions.append(
            TimerAction(
                period=run_duration_s,
                actions=[EmitEvent(event=Shutdown(reason="demo9 run_duration_s elapsed"))],
            )
        )

    return actions


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    fastdds_profile = os.path.join(iap_share, "config", "sim_ego", "fastdds_udp_only.xml")

    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("config_subdir", default_value="sim_demo9"),
        DeclareLaunchArgument("use_gnss", default_value="true"),
        DeclareLaunchArgument("use_araim", default_value="true"),
        DeclareLaunchArgument("use_iap_odom_for_planner", default_value="true"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_z", default_value="2.0"),
        DeclareLaunchArgument("point_num", default_value="7"),

        DeclareLaunchArgument("point1_x", default_value="6.0"),
        DeclareLaunchArgument("point1_y", default_value="6.0"),
        DeclareLaunchArgument("point1_z", default_value="2.0"),

        DeclareLaunchArgument("point2_x", default_value="-6.0"),
        DeclareLaunchArgument("point2_y", default_value="6.0"),
        DeclareLaunchArgument("point2_z", default_value="2.0"),

        DeclareLaunchArgument("point3_x", default_value="-6.0"),
        DeclareLaunchArgument("point3_y", default_value="-6.0"),
        DeclareLaunchArgument("point3_z", default_value="2.0"),

        DeclareLaunchArgument("point4_x", default_value="6.0"),
        DeclareLaunchArgument("point4_y", default_value="-6.0"),
        DeclareLaunchArgument("point4_z", default_value="2.0"),

        DeclareLaunchArgument("point5_x", default_value="6.0"),
        DeclareLaunchArgument("point5_y", default_value="6.0"),
        DeclareLaunchArgument("point5_z", default_value="2.0"),
        DeclareLaunchArgument("point6_x", default_value=LaunchConfiguration("goal_x")),
        DeclareLaunchArgument("point6_y", default_value=LaunchConfiguration("goal_y")),
        DeclareLaunchArgument("point6_z", default_value=LaunchConfiguration("goal_z")),

        DeclareLaunchArgument("run_duration_s", default_value="120"),
        DeclareLaunchArgument("use_so3_dynamics", default_value="true"),
        DeclareLaunchArgument("planner_use_dynamic", default_value="true"),
        DeclareLaunchArgument("planner_use_integrity_cost", default_value="false"),
        DeclareLaunchArgument("planner_lambda_integrity", default_value="0.00001"),
        DeclareLaunchArgument("planner_integrity_field_stale_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("planner_integrity_nearest_radius_m", default_value="1.0"),
        DeclareLaunchArgument("planner_integrity_cost_max", default_value="1000.0"),
        DeclareLaunchArgument("planner_integrity_grad_norm_max", default_value="0.1"),
        DeclareLaunchArgument("planner_use_integrity_front_search", default_value="false"),
        DeclareLaunchArgument("planner_use_integrity_global_search", default_value="false"),
        DeclareLaunchArgument("planner_lambda_integrity_front", default_value="2.0"),
        DeclareLaunchArgument("planner_integrity_front_cost_topic", default_value="/iap/integrity_front_cost_field"),
        DeclareLaunchArgument("planner_integrity_front_nearest_radius_m", default_value="1.5"),
        DeclareLaunchArgument("planner_integrity_front_stale_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("planner_integrity_front_cost_max", default_value="10.0"),
        DeclareLaunchArgument("planner_integrity_global_astar_step_m", default_value="0.5"),
        DeclareLaunchArgument("planner_integrity_global_max_waypoints", default_value="80"),
        DeclareLaunchArgument("planner_start_delay_s", default_value="0.0"),
        DeclareLaunchArgument("enable_preflight_takeoff", default_value="false"),
        DeclareLaunchArgument("preflight_ground_z", default_value="0.0"),
        DeclareLaunchArgument("preflight_ground_hold_s", default_value="10.0"),
        DeclareLaunchArgument("preflight_takeoff_duration_s", default_value="5.0"),
        DeclareLaunchArgument("preflight_hover_s", default_value="2.0"),
        DeclareLaunchArgument("preflight_cmd_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("iap_odom_freshness_sec", default_value="0.3"),
        DeclareLaunchArgument("use_dynamic_obstacles", default_value="false"),
        DeclareLaunchArgument("map_source", default_value="local_sensing_cloud"),
        DeclareLaunchArgument("map_generator_mode", default_value="random_forest"),
        DeclareLaunchArgument("allow_truth_alignment", default_value="true"),
        DeclareLaunchArgument("log_phase1", default_value="true"),
        DeclareLaunchArgument("enable_truth_araim_compare", default_value="false"),
        DeclareLaunchArgument("truth_araim_compare_csv_name", default_value="demo9_araim_truth_compare.csv"),
        DeclareLaunchArgument("gnss_ephemeris_source", default_value="rinex"),
        DeclareLaunchArgument("gnss_enabled_constellations", default_value="GPS,BDS,GAL,GLO"),
        DeclareLaunchArgument(
            "gnss_scenario_file",
            default_value=os.path.join(iap_share, "config", "gnss_sim", "demo7_open_sky.yaml"),
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
        DeclareLaunchArgument("gnss_skyplot_origin_y", default_value="12.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_z", default_value="8.0"),
        DeclareLaunchArgument("viz_status_text_use_fixed_position", default_value="false"),
        DeclareLaunchArgument("viz_status_text_x", default_value="0.0"),
        DeclareLaunchArgument("viz_status_text_y", default_value="0.0"),
        DeclareLaunchArgument("viz_status_text_z", default_value="14.0"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="0.0"),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="20.0"),
        DeclareLaunchArgument("map_size_z", default_value="4.0"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        OpaqueFunction(function=_launch_setup),
    ])
