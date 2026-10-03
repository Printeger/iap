"""Canonical all-module IAP simulation entrypoint.

The current production planner graph is provided by a private runtime include
extracted from the maintained validation graph. The maintained four-fork ICRA
scene uses the same single-run continuous-flight runtime and visualization
profile as the development runner. Automatic bag recording, repetition,
capture, and runner-side PASS/FAIL analysis remain outside this entrypoint.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


_INCLUDES = Path(__file__).resolve().parent / "_includes"
if str(_INCLUDES) not in sys.path:
    sys.path.insert(0, str(_INCLUDES))
from run_directory import (  # noqa: E402
    finalize_run_from_shutdown,
    register_config_snapshot,
    resolve_run_directory,
    write_subordinate_manifest,
)


def _catalog(iap_share: Path) -> dict:
    with (iap_share / "config" / "scenarios" / "catalog.json").open(
        encoding="utf-8"
    ) as stream:
        value = json.load(stream)
    if not isinstance(value, dict) or not value:
        raise RuntimeError("IAP simulation scenario catalog is empty or invalid")
    return value


def _setup(context):
    iap_share = Path(get_package_share_directory("iap"))
    scenario = LaunchConfiguration("scenario").perform(context).strip()
    catalog = _catalog(iap_share)
    if scenario not in catalog:
        valid = ", ".join(sorted(catalog))
        raise RuntimeError(f"unknown IAP simulation scenario '{scenario}'; valid: {valid}")

    icra_continuous_flight = scenario == "icra_dense_forest_four_fork_v2"

    output_dir = resolve_run_directory(entrypoint="iap_sim", scenario=scenario)
    register_config_snapshot(
        output_dir, output_dir / "metadata" / "config" / "full_stack"
    )

    manifest = {
        "schema_version": "iap_canonical_sim_v3",
        "scenario": scenario,
        "scenario_runtime_preset": scenario,
        "task_mode": str(catalog[scenario]["task_mode"]),
        "runtime_profile": (
            "icra_continuous_flight"
            if icra_continuous_flight
            else "canonical_full_stack_sim"
        ),
        "modules": [
            "GLIO",
            "Current Integrity Monitor",
            "Advisory Integrity Evaluator (P0)",
            "Safety-aware planner (P4/P5)",
        ],
        "test_validator_enabled": icra_continuous_flight,
        "rviz_profile": (
            "config/sim_demo11/test_icra.rviz"
            if icra_continuous_flight
            else "config/sim_demo11/demo11_integrity_corridor.rviz"
        ),
        "rosbag_recording_enabled": False,
        "phase2_planner_integrity_evaluator_enabled": False,
        "clock_contract": (
            "icra_simulated_sensor_time"
            if icra_continuous_flight
            else "system_clock_for_ros_and_simulated_sensor_stamps"
        ),
    }
    write_subordinate_manifest(output_dir, "full_stack", manifest)

    return [
        SetEnvironmentVariable("IAP_RUN_DIR", str(output_dir)),
        SetEnvironmentVariable(
            "ROS_LOG_DIR", str(output_dir / "runtime" / "ros")
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, _context: finalize_run_from_shutdown(
                    output_dir, event
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    iap_share
                    / "launch"
                    / "_includes"
                    / "full_stack_simulation.launch.py"
                )
            ),
            launch_arguments={
                "scenario": scenario,
                "start_rviz": LaunchConfiguration("start_rviz").perform(context),
                "runtime_root_dir": str(output_dir / "metadata" / "config" / "full_stack"),
                "export_root_dir": str(output_dir / "export" / "planner"),
                "iap_log_root": str(output_dir / "runtime"),
                "bag_output_dir": str(output_dir / "export" / "capture"),
                "run_duration_s": LaunchConfiguration("run_duration_s").perform(context),
                "planner_start_delay_s": LaunchConfiguration(
                    "planner_start_delay_s"
                ).perform(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="fused_nominal"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("planner_start_delay_s", default_value="10.0"),
            DeclareLaunchArgument("run_duration_s", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
