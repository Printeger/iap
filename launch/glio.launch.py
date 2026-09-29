"""Canonical GLIO-only launch.

This launch starts no simulator, map generator, rosbag player, fake odometry,
or planner. Live sensors and rosbag playback intentionally share this interface.
"""

from __future__ import annotations

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_INCLUDES = Path(__file__).resolve().parent / "_includes"
if str(_INCLUDES) not in sys.path:
    sys.path.insert(0, str(_INCLUDES))
from profile_runtime import materialize_profile  # noqa: E402


def _setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    output_dir = LaunchConfiguration("output_dir").perform(context).strip()
    runtime_config, manifest = materialize_profile(
        source_config_dir=config_path,
        output_dir=output_dir,
        contract="glio",
        integrity_profile="fallback_only",
    )
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    points_topic = LaunchConfiguration("points_topic").perform(context)
    return [
        LogInfo(
            msg=(
                "[glio] contract=GLIO-only "
                f"config={runtime_config} output={manifest['output_dir']}"
            )
        ),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="glio",
            output="screen",
            parameters=[
                {"config_path": runtime_config},
                {"imu_topic": imu_topic},
                {"points_topic": points_topic},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ]


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("iap")) / "config" / "profiles" / "glio"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="GLIO-only profile; integrity extension is rejected.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                description="Required absolute directory for this run's logs and CSVs.",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
