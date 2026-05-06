import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_repo_path(path_value: str) -> str:
    """Resolve path strings and support '@' workspace-relative shorthand."""
    normalized = path_value.strip()
    if not normalized:
        return normalized

    if normalized.startswith("@"):
        return os.path.abspath(os.path.join(os.getcwd(), normalized[1:]))

    return os.path.abspath(os.path.expanduser(normalized))


def _prepare_runtime_args(context):
    raw_config_path = LaunchConfiguration("config_path").perform(context)
    raw_bag_path = LaunchConfiguration("bag_path").perform(context)

    resolved_config_path = _resolve_repo_path(raw_config_path)
    resolved_bag_path = _resolve_repo_path(raw_bag_path)

    return [
        SetLaunchConfiguration("resolved_config_path", resolved_config_path),
        SetLaunchConfiguration("resolved_bag_path", resolved_bag_path),
        LogInfo(msg=f"[iap_rosnode.launch] config_path={raw_config_path} -> resolved={resolved_config_path}"),
        LogInfo(msg=f"[iap_rosnode.launch] bag_path={raw_bag_path} -> resolved={resolved_bag_path}"),
    ]


def generate_launch_description():
    bag_rate = LaunchConfiguration("bag_rate")
    mode = LaunchConfiguration("mode")
    resolved_config_path = LaunchConfiguration("resolved_config_path")
    resolved_bag_path = LaunchConfiguration("resolved_bag_path")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/dev/ws_iap/src/iap/config",
            description="Path to IAP config directory. Matches direct ros2 run usage.",
        ),
        DeclareLaunchArgument(
            "bag_path",
            default_value="@src/iap/data/realsense_ros2",
            description="Path to rosbag2 directory. '@' prefix is resolved from current working directory.",
        ),
        DeclareLaunchArgument(
            "bag_rate",
            default_value="1.0",
            description="Playback rate passed to ros2 bag play --rate",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="bag",
            description="Run mode: 'bag' to play rosbag, 'realtime' for live sensor input",
        ),
        OpaqueFunction(function=_prepare_runtime_args),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="iap_rosnode",
            output="screen",
            parameters=[{"config_path": resolved_config_path}],
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "play", resolved_bag_path, "--rate", bag_rate],
            output="screen",
            condition=IfCondition(PythonExpression(["'", mode, "' == 'bag'"])),
        ),
    ])
