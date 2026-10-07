"""Canonical EGO baseline simulation with a shared GridMap PL layer."""

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
    adopt_run_directory,
    finalize_run_from_shutdown,
    register_config_snapshot,
    register_validation_trial,
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


    internal_run = str(context.launch_configurations.get("run_dir", "")).strip()
    output_dir = (adopt_run_directory(internal_run) if internal_run else
                  resolve_run_directory(entrypoint="iap_sim", scenario=scenario))
    if internal_run:
        primary = json.loads((output_dir / "metadata/run_manifest.json").read_text())
        if (primary.get("schema_version") != "iap_run_artifact_v1" or
                primary.get("entrypoint") != "iap_sim" or primary.get("scenario") != scenario or
                primary.get("lifecycle") != "active"):
            raise ValueError("iap_sim run_dir requires its own active resolver-allocated run")
    lifecycle_owner = context.launch_configurations.get("run_lifecycle_owner", "launch")
    if lifecycle_owner not in ("launch", "driver") or (lifecycle_owner == "driver" and not internal_run):
        raise ValueError("driver lifecycle owner requires a preallocated run_dir")
    register_config_snapshot(
        output_dir, output_dir / "metadata" / "config"
    )

    calibration=context.launch_configurations.get("advisory_calibration", "")
    nav_path = context.launch_configurations.get("rinex_nav_file", "").strip()
    if nav_path:
        from full_stack_runtime import historical_gnss_parameters
        historical_gnss_parameters(nav_path)
        if catalog[scenario]["gnss_profile"] == "disabled":
            raise ValueError("historical GPS+BDS requires an enabled GNSS scenario")
        frozen_nav = output_dir / "metadata/config/historical_nav.rnx"
        frozen_nav.write_bytes(Path(nav_path).read_bytes())
        register_config_snapshot(output_dir, frozen_nav)
        nav_path = str(frozen_nav)
    if calibration:
        from full_stack_runtime import load_advisory_calibration
        load_advisory_calibration(calibration)
        frozen=output_dir/"metadata/config/advisory_calibration.json"
        frozen.write_bytes(Path(calibration).read_bytes())
        load_advisory_calibration(str(frozen))
        register_config_snapshot(output_dir,frozen)
        calibration=str(frozen)

    trial_path=context.launch_configurations.get("advisory_trial", "")
    if trial_path:
        from full_stack_runtime import load_advisory_trial, planner_parameters
        trial=load_advisory_trial(trial_path,catalog[scenario])
        if scenario!="icra_dense_forest_four_fork_v2": raise ValueError("validation trial requires canonical forest")
        for key in ("coordinates","physical_route_evidence"):
            frozen=output_dir/"metadata/config"/(key+".json")
            frozen.write_bytes(Path(trial[key]).read_bytes());register_config_snapshot(output_dir,frozen)
            trial[key]=str(frozen)
        frozen=output_dir/"metadata/config/advisory_trial.json"
        frozen.write_text(json.dumps(trial,sort_keys=True,allow_nan=False,indent=2));register_config_snapshot(output_dir,frozen)
        trial_path=str(frozen)
        params=planner_parameters(catalog[scenario],False,
            context.launch_configurations.get("advisory_posterior_prior","false")=="true",calibration,
            context.launch_configurations.get("advisory_guidance","false")=="true",trial_path)
        import hashlib
        trial["parameter_sha256"]=json.loads(Path(calibration).read_text())["sha256"] if calibration else hashlib.sha256(
            json.dumps(params,sort_keys=True,allow_nan=False).encode()).hexdigest()
        register_validation_trial(output_dir,trial)

    manifest = {
        "schema_version": "iap_canonical_sim_v3",
        "scenario": scenario,
        "scenario_runtime_preset": scenario,
        "task_mode": str(catalog[scenario]["task_mode"]),
        "runtime_profile": (
            "ego_grid_map_stage1"
        ),
        "modules": [
            "GLIO",
            "Current Integrity Monitor",
            "GridMap spatial PL (PredictorModule)",
            "EGO planner with frozen curve/corridor checks",
            *(["Independent GridMap PL visualizer"] if context.launch_configurations.get("start_grid_map_visualizer", "true").lower() == "true" else []),
        ],
        "grid_map_visualizer_enabled": context.launch_configurations.get("start_grid_map_visualizer", "true").lower() == "true",
        "advisory_calibration": calibration,
        "advisory_trial": trial_path,
        "advisory_guidance_enabled": context.launch_configurations.get("advisory_guidance", "false").lower()=="true",
        "advisory_posterior_prior_enabled": context.launch_configurations.get("advisory_posterior_prior", "false").lower() == "true",
        "test_validator_enabled": False,
        "rviz_profile": (
            "config/sim_ego/grid_map_stage1.rviz"
        ),
        "rosbag_recording_enabled": False,
        "phase2_planner_integrity_evaluator_enabled": False,
        "lifecycle_owner": lifecycle_owner,
        "clock_contract": (
            "historical_clock_2022-07-06T12:00:00Z" if nav_path else
            "system_clock_for_ros_and_simulated_sensor_stamps"
        ),
        "gnss_input": {"source": "rinex" if nav_path else "synthetic",
                       "nav_file": nav_path,
                       "nav_sha256": __import__("hashlib").sha256(Path(nav_path).read_bytes()).hexdigest() if nav_path else None,
                       "constellations": ["GPS", "BDS"] if nav_path else ["GPS", "BDS", "GAL", "GLO"],
                       "formal_advisory_qualified": False},
    }
    write_subordinate_manifest(output_dir, "full_stack", manifest)

    actions = [
        SetEnvironmentVariable("IAP_RUN_DIR", str(output_dir)),
        SetEnvironmentVariable(
            "ROS_LOG_DIR", str(output_dir / "runtime" / "ros")
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
                "rinex_nav_file": nav_path,
                "advisory_calibration": calibration,
                "advisory_trial": trial_path,
                "advisory_guidance": context.launch_configurations.get("advisory_guidance", "false"),
                "advisory_posterior_prior": context.launch_configurations.get("advisory_posterior_prior", "false"),
                "start_rviz": LaunchConfiguration("start_rviz").perform(context),
                "start_grid_map_visualizer": LaunchConfiguration("start_grid_map_visualizer").perform(context),
                "runtime_root_dir": str(output_dir / "metadata" / "config" / "full_stack"),
                "export_root_dir": str(output_dir / "export" / "planner"),
                "iap_log_root": str(output_dir / "runtime"),
                "bag_output_dir": str(output_dir / "export" / "capture"),
                "run_duration_s": LaunchConfiguration("run_duration_s").perform(context),
                "planner_start_delay_s": LaunchConfiguration(
                    "planner_start_delay_s"
                ).perform(context),
                "capture_failure_map": context.launch_configurations.get(
                    "capture_failure_map", "false"
                ),
            }.items(),
        ),
    ]
    if lifecycle_owner == "launch":
        actions.insert(2, RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, _context: finalize_run_from_shutdown(
                    output_dir, event
                )
            )
        ))
    return actions



def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_lifecycle_owner", default_value="launch", description="Private driver owns finalization after all run captures finish"),
            DeclareLaunchArgument("run_dir", default_value="", description="Private preallocated canonical owner run; contains startup ROS logs"),
            DeclareLaunchArgument("scenario", default_value="icra_dense_forest_four_fork_v2"),
            DeclareLaunchArgument("rinex_nav_file", default_value="",
                                 description="Absolute historical NAV: strict GPS+BDS and unique 2022-07-06T12:00:00Z clock; empty selects synthetic mechanism input"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_grid_map_visualizer", default_value="true"),
            DeclareLaunchArgument("advisory_guidance", default_value="false",choices=["true","false"],
                                 description="Planning preference only; prediction/recording/display stay active"),
            DeclareLaunchArgument("advisory_calibration", default_value="",
                                 description="Absolute frozen empirical Advisory JSON; empty preserves uncalibrated defaults"),
            DeclareLaunchArgument("advisory_trial",default_value="",
                                 description="Absolute fixed-route/seed/observation trial JSON with real physical/frame proof"),
            DeclareLaunchArgument("advisory_posterior_prior", default_value="false",
                                 choices=["true", "false"],
                                 description="Include FGO posterior proxy in Advisory (legacy A/B only)"),
            DeclareLaunchArgument("planner_start_delay_s", default_value="10.0"),
            DeclareLaunchArgument("capture_failure_map", default_value="false"),
            DeclareLaunchArgument("run_duration_s", default_value="0.0"),
            OpaqueFunction(function=_setup),
        ]
    )
