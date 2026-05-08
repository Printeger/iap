import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    return LaunchDescription([
        DeclareLaunchArgument("off_run_dir", default_value="/home/dev/ws_iap/src/iap/log/latest"),
        DeclareLaunchArgument("on_run_dir", default_value="/home/dev/ws_iap/src/iap/log/latest"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        Node(
            package="iap",
            executable="demo11_compare_paths_visualizer",
            name="demo11_compare_paths_visualizer",
            output="screen",
            parameters=[
                {"off_run_dir": LaunchConfiguration("off_run_dir")},
                {"on_run_dir": LaunchConfiguration("on_run_dir")},
                {"frame_id": "map"},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="demo11_compare_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            arguments=[
                "-d",
                os.path.join(iap_share, "config", "sim_demo11", "demo11_compare.rviz"),
            ],
        ),
    ])
