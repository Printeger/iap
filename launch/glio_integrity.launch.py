"""Canonical GLIO + Current Integrity Monitor launch.

The monitor is currently an in-process IAP extension. This launch selects and
validates a profile containing both GNSS and integrity extensions; it starts no
environment, rosbag player, or planner.
"""

from __future__ import annotations

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_INCLUDES = Path(__file__).resolve().parent / "_includes"
if str(_INCLUDES) not in sys.path:
    sys.path.insert(0, str(_INCLUDES))
from profile_runtime import materialize_profile  # noqa: E402
from run_directory import (  # noqa: E402
    adopt_run_directory,
    finalize_run_from_shutdown,
    register_config_snapshot,
    register_subordinate_manifest,
    resolve_run_directory,
)


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _setup(context):
    iap_share = Path(get_package_share_directory("iap"))
    config_path = LaunchConfiguration("config_path").perform(context)
    internal_run = str(context.launch_configurations.get("run_dir", "")).strip()
    output_dir = (
        adopt_run_directory(internal_run)
        if internal_run
        else resolve_run_directory(entrypoint="glio_integrity")
    )
    integrity_profile = LaunchConfiguration("integrity_profile").perform(context)
    forbid_sim = _as_bool(
        LaunchConfiguration("forbid_sim_extensions").perform(context)
    )
    runtime_contract = LaunchConfiguration("runtime_contract").perform(context).strip()
    if runtime_contract not in {"glio_integrity", "full_stack"}:
        raise RuntimeError("runtime_contract must be glio_integrity or full_stack")
    runtime_config, manifest = materialize_profile(
        source_config_dir=config_path,
        output_dir=output_dir,
        contract=runtime_contract,
        integrity_profile=integrity_profile,
        forbid_sim_extensions=forbid_sim,
    )
    if not internal_run:
        register_config_snapshot(output_dir, Path(runtime_config))
        register_subordinate_manifest(
            output_dir,
            output_dir / "metadata" / "manifests" / "launch_profile_manifest.json",
        )
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    points_topic = LaunchConfiguration("points_topic").perform(context)
    actions = [
        LogInfo(
            msg=(
                "[glio_integrity] contract=GLIO+CurrentIntegrityMonitor "
                f"profile={integrity_profile} config={runtime_config} "
                f"output={manifest['output_dir']}"
            )
        ),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="glio_integrity",
            output="screen",
            additional_env={
                "IAP_RUN_DIR": str(output_dir),
                "ROS_LOG_DIR": str(output_dir / "runtime" / "ros"),
            },
            parameters=[
                {"config_path": runtime_config},
                {"imu_topic": imu_topic},
                {"points_topic": points_topic},
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="glio_integrity_rviz",
            output="screen",
            arguments=[
                "-d",
                str(
                    iap_share
                    / "config"
                    / "profiles"
                    / "glio_integrity"
                    / "glio_integrity.rviz"
                ),
            ],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            additional_env={
                "IAP_RUN_DIR": str(output_dir),
                "ROS_LOG_DIR": str(output_dir / "runtime" / "ros"),
            },
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]
    if not internal_run:
        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=lambda event, _context: finalize_run_from_shutdown(
                        output_dir, event
                    )
                )
            )
        )
    return actions


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("iap"))
        / "config"
        / "profiles"
        / "glio_integrity"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="Profile containing GNSS and integrity extensions.",
            ),
            DeclareLaunchArgument(
                "run_dir",
                default_value="",
                description="Internal outer-run adoption; not a user-facing override.",
            ),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description=(
                    "Start RViz with GLIO outputs and current-integrity "
                    "envelopes."
                ),
            ),
            DeclareLaunchArgument(
                "integrity_profile",
                default_value="fused",
                description="fused, gnss_only, lidar_only, or fallback_only",
            ),
            DeclareLaunchArgument(
                "forbid_sim_extensions",
                default_value="false",
                description="Reject simulator/truth extensions (forced by iap_flight).",
            ),
            DeclareLaunchArgument(
                "runtime_contract",
                default_value="glio_integrity",
                description="Internal composition contract; full_stack also requires local-map authority.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
