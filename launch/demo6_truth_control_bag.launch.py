import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_path(path_value: str) -> str:
    value = path_value.strip()
    if value.startswith("@"):
        return os.path.abspath(os.path.join(os.getcwd(), value[1:]))
    return os.path.abspath(os.path.expanduser(value))


def _prepare_paths(context):
    raw_config_path = LaunchConfiguration("config_path").perform(context)
    raw_bag_path = LaunchConfiguration("bag_path").perform(context)
    config_path = _resolve_path(raw_config_path)
    bag_path = _resolve_path(raw_bag_path)
    return [
        SetLaunchConfiguration("resolved_config_path", config_path),
        SetLaunchConfiguration("resolved_bag_path", bag_path),
        LogInfo(msg=f"[demo6_truth_control_bag] config_path={config_path}"),
        LogInfo(msg=f"[demo6_truth_control_bag] bag_path={bag_path}"),
    ]


def generate_launch_description():
    config_path = LaunchConfiguration("resolved_config_path")
    bag_path = LaunchConfiguration("resolved_bag_path")
    bag_rate = LaunchConfiguration("bag_rate")
    bag_start_delay = LaunchConfiguration("bag_start_delay")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value="@src/iap/config/demo6_truth_control_bag",
            description="Dedicated IAP config directory for the demo6 truth-control bag.",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value="@src/iap/data/demo6_truth_control_iap_input",
            description="Path to the recorded demo6 truth-control rosbag2 directory.",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="Playback rate passed to ros2 bag play --rate.",
        ),
        DeclareLaunchArgument(
            "bag_start_delay",
            default_value="2.0",
            description="Delay in seconds before rosbag playback starts, giving IAP time to subscribe.",
        ),
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            os.path.join(os.getcwd(), "src/iap/config/sim_ego/fastdds_udp_only.xml"),
        ),
        OpaqueFunction(function=_prepare_paths),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="demo6_truth_control_bag_iap_rosnode",
            output="screen",
            parameters=[{"config_path": config_path}],
        ),
        TimerAction(
            period=bag_start_delay,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "bag", "play", bag_path, "--rate", bag_rate],
                    output="screen",
                )
            ],
        ),
    ])
