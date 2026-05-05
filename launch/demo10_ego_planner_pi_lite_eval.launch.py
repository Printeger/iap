import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context):
    iap_share = get_package_share_directory("iap")
    map_source = LaunchConfiguration("map_source").perform(context)
    if map_source == "local_sensing_cloud":
        phase2_map_topic = "/sim/drone_0/lidar"
    elif map_source == "global_cloud_direct":
        phase2_map_topic = "/map_generator/global_cloud"
    else:
        raise RuntimeError("map_source must be 'local_sensing_cloud' or 'global_cloud_direct'")

    demo9_args = {
        name: LaunchConfiguration(name)
        for name in (
            "start_rviz",
            "use_gnss",
            "use_araim",
            "use_iap_odom_for_planner",
            "goal_x",
            "goal_y",
            "goal_z",
            "point_num",
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
            "run_duration_s",
            "use_so3_dynamics",
            "planner_use_dynamic",
            "use_dynamic_obstacles",
            "map_source",
            "allow_truth_alignment",
            "log_phase1",
            "enable_truth_araim_compare",
            "truth_araim_compare_csv_name",
            "gnss_ephemeris_source",
            "gnss_enabled_constellations",
            "gnss_rinex_nav_file",
            "gnss_rinex_ephem_max_age_s",
            "gnss_fallback_to_synthetic_on_rinex_error",
            "drone_id",
            "init_x",
            "init_y",
            "init_z",
            "map_size_x",
            "map_size_y",
            "map_size_z",
        )
    }

    return [
        LogInfo(msg=f"[demo10] Phase 2 PI-lite evaluator map topic: {phase2_map_topic}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(iap_share, "launch", "demo9_ego_planner_closed_loop.launch.py")
            ),
            launch_arguments=demo9_args.items(),
        ),
        Node(
            package="iap_phase1_tools",
            executable="phase2_planner_integrity_evaluator",
            name="phase2_planner_integrity_evaluator",
            output="screen",
            parameters=[
                {"log_root": "/home/dev/ws_iap/src/iap/log"},
                {"odom_topic": "/drone_0_visual_slam/odom"},
                {"bspline_topic": "/drone_0_planning/bspline"},
                {"pos_cmd_topic": "/drone_0_planning/pos_cmd"},
                {"map_topic": phase2_map_topic},
                {"integrity_topic": "/iap/integrity"},
                {"eval_horizon_s": float(LaunchConfiguration("phase2_eval_horizon_s").perform(context))},
                {"eval_dt_s": float(LaunchConfiguration("phase2_eval_dt_s").perform(context))},
                {"max_samples_per_traj": int(LaunchConfiguration("phase2_max_samples_per_traj").perform(context))},
                {"pl_model": LaunchConfiguration("phase2_pl_model").perform(context)},
                {"al_model": LaunchConfiguration("phase2_al_model").perform(context)},
                {"drone_radius": float(LaunchConfiguration("phase2_drone_radius").perform(context))},
                {"safety_buffer": float(LaunchConfiguration("phase2_safety_buffer").perform(context))},
                {"gamma_h": float(LaunchConfiguration("phase2_gamma_h").perform(context))},
                {"gamma_v": float(LaunchConfiguration("phase2_gamma_v").perform(context))},
                {"z_min": float(LaunchConfiguration("phase2_z_min").perform(context))},
                {"z_max": float(LaunchConfiguration("phase2_z_max").perform(context))},
                {"safe_margin": float(LaunchConfiguration("phase2_safe_margin").perform(context))},
                {"publish_markers": _as_bool(LaunchConfiguration("phase2_publish_markers").perform(context))},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
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
        DeclareLaunchArgument("use_dynamic_obstacles", default_value="false"),
        DeclareLaunchArgument("map_source", default_value="local_sensing_cloud"),
        DeclareLaunchArgument("allow_truth_alignment", default_value="false"),
        DeclareLaunchArgument("log_phase1", default_value="true"),
        DeclareLaunchArgument("enable_truth_araim_compare", default_value="true"),
        DeclareLaunchArgument("truth_araim_compare_csv_name", default_value="demo10_araim_truth_compare.csv"),
        DeclareLaunchArgument("gnss_ephemeris_source", default_value="rinex"),
        DeclareLaunchArgument("gnss_enabled_constellations", default_value="GPS,BDS,GAL,GLO"),
        DeclareLaunchArgument(
            "gnss_rinex_nav_file",
            default_value="/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx",
        ),
        DeclareLaunchArgument("gnss_rinex_ephem_max_age_s", default_value="7200.0"),
        DeclareLaunchArgument("gnss_fallback_to_synthetic_on_rinex_error", default_value="false"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("init_x", default_value="0.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="0.0"),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="20.0"),
        DeclareLaunchArgument("map_size_z", default_value="4.0"),
        DeclareLaunchArgument("phase2_eval_horizon_s", default_value="5.0"),
        DeclareLaunchArgument("phase2_eval_dt_s", default_value="0.2"),
        DeclareLaunchArgument("phase2_max_samples_per_traj", default_value="30"),
        DeclareLaunchArgument("phase2_pl_model", default_value="constant_current"),
        DeclareLaunchArgument("phase2_al_model", default_value="cloud_clearance"),
        DeclareLaunchArgument("phase2_drone_radius", default_value="0.35"),
        DeclareLaunchArgument("phase2_safety_buffer", default_value="0.20"),
        DeclareLaunchArgument("phase2_gamma_h", default_value="0.8"),
        DeclareLaunchArgument("phase2_gamma_v", default_value="0.8"),
        DeclareLaunchArgument("phase2_z_min", default_value="0.5"),
        DeclareLaunchArgument("phase2_z_max", default_value="5.0"),
        DeclareLaunchArgument("phase2_safe_margin", default_value="0.0"),
        DeclareLaunchArgument("phase2_publish_markers", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
