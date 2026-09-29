"""Canonical all-module IAP simulation entrypoint.

The current production planner graph is provided by a private runtime include
extracted from the maintained validation graph. This profile fixes it to its
non-test contract: no validator, no automatic bag recording, the maintained
P0/P4/P5 safety profile, and run-local artifacts only. Keeping this thin
boundary lets scenarios retain their exact established semantics while the
large runtime is split into private includes incrementally.
"""

from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    output_dir = Path(LaunchConfiguration("output_dir").perform(context)).expanduser()
    if not output_dir.is_absolute():
        raise RuntimeError("output_dir must be an absolute path")
    output_dir = output_dir.resolve()
    if output_dir == Path("/"):
        raise RuntimeError("output_dir cannot be filesystem root")
    output_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "schema_version": "iap_canonical_sim_v2",
        "scenario": scenario,
        "scenario_runtime_preset": scenario,
        "task_mode": str(catalog[scenario]["task_mode"]),
        "runtime_profile": "canonical_full_stack_sim",
        "modules": [
            "GLIO",
            "Current Integrity Monitor",
            "Advisory Integrity Evaluator (P0)",
            "Safety-aware planner (P4/P5)",
        ],
        "test_validator_enabled": False,
        "rosbag_recording_enabled": False,
        "phase2_planner_integrity_evaluator_enabled": False,
        "clock_contract": "system_clock_for_ros_and_simulated_sensor_stamps",
    }
    (output_dir / "full_stack_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    return [
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
                "runtime_root_dir": str(output_dir / "runtime"),
                "export_root_dir": str(output_dir / "export"),
                "iap_log_root": str(output_dir / "runtime" / "iap_logs"),
                "bag_output_dir": str(output_dir / "bags"),
                "run_duration_s": LaunchConfiguration("run_duration_s").perform(context),
                "planner_start_delay_s": LaunchConfiguration(
                    "planner_start_delay_s"
                ).perform(context),
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="fused_nominal"),
            DeclareLaunchArgument(
                "output_dir",
                description="Required absolute directory for all runtime artifacts.",
            ),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("planner_start_delay_s", default_value="10.0"),
            DeclareLaunchArgument("run_duration_s", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
