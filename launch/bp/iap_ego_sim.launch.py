import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_repo_path(path_value: str) -> str:
    normalized = path_value.strip()
    if not normalized:
        return normalized

    if normalized.startswith("@"):
        return os.path.abspath(os.path.join(os.getcwd(), normalized[1:]))

    return os.path.abspath(os.path.expanduser(normalized))


def _prepare_runtime_args(context):
    raw_config_path = LaunchConfiguration("config_path").perform(context)
    resolved_config_path = _resolve_repo_path(raw_config_path)

    return [
        SetLaunchConfiguration("resolved_config_path", resolved_config_path),
        LogInfo(msg=f"[iap_ego_sim.launch] config_path={raw_config_path} -> resolved={resolved_config_path}"),
    ]


def generate_launch_description():
    config_path = LaunchConfiguration("resolved_config_path")
    start_iap = LaunchConfiguration("start_iap")
    use_dynamic = LaunchConfiguration("use_dynamic")
    use_mockamap = LaunchConfiguration("use_mockamap")
    drone_id = LaunchConfiguration("drone_id")
    map_size_x = LaunchConfiguration("map_size_x")
    map_size_y = LaunchConfiguration("map_size_y")
    map_size_z = LaunchConfiguration("map_size_z")
    init_x = LaunchConfiguration("init_x")
    init_y = LaunchConfiguration("init_y")
    init_z = LaunchConfiguration("init_z")
    target_x = LaunchConfiguration("target_x")
    target_y = LaunchConfiguration("target_y")
    target_z = LaunchConfiguration("target_z")
    obj_num = LaunchConfiguration("obj_num")
    plant_odom_topic = LaunchConfiguration("plant_odom_topic")
    planner_odom_topic = LaunchConfiguration("planner_odom_topic")
    sim_imu_topic = LaunchConfiguration("sim_imu_topic")
    sim_lidar_topic = LaunchConfiguration("sim_lidar_topic")
    sim_depth_topic = LaunchConfiguration("sim_depth_topic")

    ego_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ego_planner"),
            "launch",
            "single_run_in_sim.launch.py",
        )),
        launch_arguments={
            "drone_id": drone_id,
            "obj_num": obj_num,
            "map_size_x": map_size_x,
            "map_size_y": map_size_y,
            "map_size_z": map_size_z,
            "init_x": init_x,
            "init_y": init_y,
            "init_z": init_z,
            "target_x": target_x,
            "target_y": target_y,
            "target_z": target_z,
            "use_dynamic": use_dynamic,
            "use_mockamap": use_mockamap,
            "plant_odom_topic": plant_odom_topic,
            "planner_odom_topic": planner_odom_topic,
            "sim_imu_topic": sim_imu_topic,
            "sim_lidar_topic": sim_lidar_topic,
            "sim_depth_topic": sim_depth_topic,
        }.items(),
    )

    iap_node = Node(
        package="iap",
        executable="iap_rosnode",
        name="iap_rosnode",
        output="screen",
        parameters=[{"config_path": config_path}],
        condition=IfCondition(start_iap),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/dev/ws_iap/src/iap/config/sim_ego",
            description="IAP sim config directory. '@' prefix is resolved from current working directory.",
        ),
        DeclareLaunchArgument(
            "start_iap",
            default_value="true",
            description="Start iap_rosnode with config/sim_ego.",
        ),
        DeclareLaunchArgument(
            "use_dynamic",
            default_value="true",
            description="Use SO3 dynamics for the EGO plant.",
        ),
        DeclareLaunchArgument(
            "use_mockamap",
            default_value="false",
            description="Use mockamap instead of map_generator.",
        ),
        DeclareLaunchArgument("drone_id", default_value="0", description="Drone ID."),
        DeclareLaunchArgument("obj_num", default_value="10", description="Number of planner objects."),
        DeclareLaunchArgument("map_size_x", default_value="50.0", description="Map size along x."),
        DeclareLaunchArgument("map_size_y", default_value="25.0", description="Map size along y."),
        DeclareLaunchArgument("map_size_z", default_value="5.0", description="Map size along z."),
        DeclareLaunchArgument("init_x", default_value="-15.0", description="Initial x position."),
        DeclareLaunchArgument("init_y", default_value="0.0", description="Initial y position."),
        DeclareLaunchArgument("init_z", default_value="1.0", description="Initial z position."),
        DeclareLaunchArgument("target_x", default_value="15.0", description="Target x position."),
        DeclareLaunchArgument("target_y", default_value="0.0", description="Target y position."),
        DeclareLaunchArgument("target_z", default_value="1.0", description="Target z position."),
        DeclareLaunchArgument(
            "plant_odom_topic",
            default_value="/sim/drone_0/truth_odom",
            description="Truth odometry topic for plant, local_sensing, and metrics.",
        ),
        DeclareLaunchArgument(
            "planner_odom_topic",
            default_value="/drone_0_visual_slam/odom",
            description="Planner odometry topic produced by libsim_extension.so.",
        ),
        DeclareLaunchArgument(
            "sim_imu_topic",
            default_value="/sim/drone_0/imu",
            description="SO3 simulator IMU topic consumed by IAP.",
        ),
        DeclareLaunchArgument(
            "sim_lidar_topic",
            default_value="/sim/drone_0/lidar",
            description="local_sensing LiDAR topic consumed by IAP and EGO grid map.",
        ),
        DeclareLaunchArgument(
            "sim_depth_topic",
            default_value="/sim/drone_0/depth",
            description="Optional local_sensing depth topic.",
        ),
        OpaqueFunction(function=_prepare_runtime_args),
        ego_launch,
        iap_node,
    ])
