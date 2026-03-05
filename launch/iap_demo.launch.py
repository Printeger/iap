"""
iap_demo.launch.py  –  Minimal IAP demo launch file  (IAP-RQ-002)

Starts the iap_status smoke-check node for system verification.
For a full SLAM demo, extend this file to launch the ROS2-compatible
GLIM/IAP node stack (see docs/quickstart.md).

Usage:
  source install/setup.bash
  ros2 launch iap iap_demo.launch.py
  # Optional overrides:
  ros2 launch iap iap_demo.launch.py config_dir:=/path/to/config
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_dir",
            default_value=PathJoinSubstitution([FindPackageShare("iap"), "config"]),
            description="Path to the IAP config directory",
        ),

        LogInfo(msg=["[iap_demo] config_dir = ", config_dir]),

        # Smoke-test node: verifies the library loads correctly and config is readable.
        Node(
            package="iap",
            executable="iap_status",
            name="iap_status",
            output="screen",
            arguments=[config_dir],
        ),
    ])
