import json
import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXPERIMENT_PRESETS = {
    "manual": {},
    "predictor_gnss_open_sky_only": {
        "araim_experiment": "gnss_open_sky",
        "predictor_output_mode": "gnss_only",
        "validator_required_selected_source": "GNSS",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
        "probe_query_set": "e1_fixed_neighborhood",
    },
    "predictor_lidar_feature_rich_only": {
        "araim_experiment": "lidar_feature_rich",
        "predictor_output_mode": "lidar_only",
        "validator_required_selected_source": "LIDAR",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "true",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
    },
    "predictor_fusion_nominal": {
        "araim_experiment": "fused_nominal",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "FUSION",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fusion_valid": "true",
        "probe_use_lidar_primitives": "true",
    },
    "predictor_gnss_degraded_lidar_good": {
        "araim_experiment": "gnss_degraded_lidar_good",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "FUSION",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fusion_valid": "true",
        "probe_use_lidar_primitives": "true",
        "start_planner": "true",
    },
    "predictor_lidar_sparse_gnss_good": {
        "araim_experiment": "lidar_degraded_gnss_good",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "FUSION",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "true",
        "probe_use_lidar_primitives": "true",
        "probe_lidar_map_max_points": "500",
        "predictor_fusion_conservative_max_with_gnss": "true",
    },
    "predictor_gnss_outage_lidar_recovery": {
        # Keep E6 self-contained in this launch file.  Use the included ARAIM
        # node flow in manual mode so test_araim's own preset cannot overwrite
        # the GPS-only synthetic outage settings below.
        "araim_experiment": "manual",
        "predictor_output_mode": "fusion",
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "init_x": "-12.0",
        "init_y": "0.0",
        "init_z": "1.2",
        "goal_x": "12.0",
        "goal_y": "0.0",
        "goal_z": "1.2",
        "point_num": "1",
        "tree_density_lower_left_per_m2": "0.75",
        "tree_density_lower_right_per_m2": "0.75",
        "tree_density_upper_left_per_m2": "0.75",
        "tree_density_upper_right_per_m2": "0.75",
        "canopy_density_lower_left": "0.08",
        "canopy_density_lower_right": "0.08",
        "canopy_density_upper_left": "0.25",
        "canopy_density_upper_right": "0.25",
        "terminal_wall_enabled": "true",
        "terminal_wall_feature_count": "64",
        "corridor_walls_enabled": "false",
        "corridor_floor_enabled": "true",
        "corridor_x_min_m": "-14.0",
        "corridor_x_max_m": "14.0",
        "corridor_half_width_y_m": "2.0",
        "corridor_floor_thickness_z_m": "0.05",
        "corridor_surface_resolution_m": "0.10",
        "gnss_ephemeris_source": "synthetic",
        "gnss_enabled_constellations": "GPS",
        "gnss_scenario_file": "generated:gnss_outage",
        "gnss_pr_noise_base": "5.0",
        "gnss_dop_noise_base": "0.5",
        "gnss_enable_map_occlusion": "false",
        "gnss_enable_skymask": "false",
        "gnss_enable_nlos": "false",
        "gnss_enable_multipath": "false",
        "gnss_enable_fault_injection": "true",
        "validator_required_selected_source": "",
        "validator_require_selected_valid": "false",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "true",
        "validator_require_fusion_valid": "false",
        "validator_max_lambda_sum_error": "0.05",
        "probe_use_lidar_primitives": "true",
        "probe_require_gnss_for_selected_output": "true",
    },
    "predictor_no_source_negative": {
        "araim_experiment": "fallback_only",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "",
        "validator_require_selected_valid": "false",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "false",
    },
    "predictor_current_advisory_separation": {
        "araim_experiment": "gnss_open_sky",
        "predictor_output_mode": "gnss_only",
        "validator_required_selected_source": "GNSS",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
        "probe_current_variant_set": "e8_current_advisory_separation",
    },
    "predictor_gnss_sigma_degradation": {
        "araim_experiment": "gnss_open_sky",
        "predictor_output_mode": "gnss_only",
        "validator_required_selected_source": "GNSS",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
        "probe_gnss_sigma_scale": "1.0",
        "start_planner": "false",
    },
    "predictor_corridor_lidar_degeneracy": {
        "araim_experiment": "lidar_corridor_degenerate",
        "predictor_output_mode": "lidar_only",
        "validator_required_selected_source": "",
        "validator_require_selected_valid": "false",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
        "start_planner": "false",
    },
    "predictor_stale_snapshot_guard": {
        "araim_experiment": "fused_nominal",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "",
        "validator_require_selected_valid": "false",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "false",
        "validator_require_fusion_valid": "false",
        "probe_use_lidar_primitives": "true",
        "probe_stale_variant_set": "e11_stale_snapshot_guard",
        "predictor_enable_freshness_guard": "true",
        "predictor_max_odom_age_s": "0.5",
        "predictor_max_integrity_age_s": "0.5",
        "predictor_max_gnss_age_s": "0.5",
        "predictor_max_snapshot_age_s": "0.5",
        "start_planner": "false",
    },
    "predictor_query_latency_stress": {
        "araim_experiment": "fused_nominal",
        "predictor_output_mode": "fusion",
        "validator_required_selected_source": "FUSION",
        "validator_require_selected_valid": "true",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fusion_valid": "true",
        "probe_use_lidar_primitives": "true",
        "probe_query_set": "e12_latency_stress",
        "probe_latency_stress_batch_sizes": "1,10,50,100",
        "probe_query_min_period_s": "0.0",
        "predictor_debug_max_lidar_primitives": "1",
        "start_planner": "false",
    },
}


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_arg_overrides():
    keys = set()
    for arg in os.sys.argv:
        if ":=" in arg:
            keys.add(arg.split(":=", 1)[0])
    return keys


def _apply_experiment_preset(context):
    experiment = LaunchConfiguration("experiment").perform(context).strip() or "manual"
    if experiment not in EXPERIMENT_PRESETS:
        valid = ", ".join(sorted(EXPERIMENT_PRESETS.keys()))
        raise RuntimeError(f"unknown test_predictor experiment '{experiment}'. Valid: {valid}")

    user_overrides = _launch_arg_overrides()
    for key, value in EXPERIMENT_PRESETS[experiment].items():
        if key not in user_overrides:
            context.launch_configurations[key] = str(value)
    return experiment


def _drop_empty_araim_passthrough_defaults(context):
    # The included test_araim launch has real defaults for these names.  Keeping
    # the predictor wrapper's empty-string placeholders in the shared launch
    # context causes child-side float/int conversions to see "" instead.
    user_overrides = _launch_arg_overrides()
    for name in ARAIM_PASSTHROUGH_ARGUMENTS:
        if name in user_overrides:
            continue
        value = context.launch_configurations.get(name)
        if value is not None and not str(value).strip():
            context.launch_configurations.pop(name, None)


def _read_config_log_root(iap_share, config_subdir):
    config_path = Path(iap_share) / "config" / config_subdir / "config.json"
    try:
        with config_path.open() as f:
            config = json.load(f)
        log_dir = config.get("logging", {}).get("log_dir", "")
        if log_dir:
            return str(log_dir)
    except Exception:
        pass
    return "/tmp/iap_predictor_log"


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")


ARAIM_PASSTHROUGH_ARGUMENTS = [
    "use_gnss",
    "use_araim",
    "enable_gnss_integrity",
    "enable_gnss_araim",
    "enable_lidar_integrity",
    "integrity_fusion_mode",
    "init_x",
    "init_y",
    "init_z",
    "goal_x",
    "goal_y",
    "goal_z",
    "point_num",
    "tree_density_lower_left_per_m2",
    "tree_density_lower_right_per_m2",
    "tree_density_upper_left_per_m2",
    "tree_density_upper_right_per_m2",
    "canopy_density_lower_left",
    "canopy_density_lower_right",
    "canopy_density_upper_left",
    "canopy_density_upper_right",
    "terminal_wall_enabled",
    "terminal_wall_feature_count",
    "corridor_walls_enabled",
    "corridor_floor_enabled",
    "corridor_x_min_m",
    "corridor_x_max_m",
    "corridor_half_width_y_m",
    "corridor_floor_thickness_z_m",
    "corridor_surface_resolution_m",
    "gnss_ephemeris_source",
    "gnss_enabled_constellations",
    "gnss_scenario_file",
    "gnss_pr_noise_base",
    "gnss_dop_noise_base",
    "gnss_enable_map_occlusion",
    "gnss_enable_skymask",
    "gnss_enable_nlos",
    "gnss_enable_multipath",
    "gnss_enable_fault_injection",
]


def _launch_setup(context):
    iap_share = get_package_share_directory("iap")
    experiment = _apply_experiment_preset(context)
    _drop_empty_araim_passthrough_defaults(context)
    config_subdir = LaunchConfiguration("config_subdir").perform(context)
    debug_enabled = _as_bool(
        LaunchConfiguration("predictor_enable_debug_log").perform(context)
    )
    run_validator_enabled = _as_bool(
        LaunchConfiguration("run_validator").perform(context)
    )
    log_root = LaunchConfiguration("predictor_log_root").perform(context).strip()
    if not log_root:
        log_root = _read_config_log_root(iap_share, config_subdir)
    export_dir = LaunchConfiguration("predictor_export_dir").perform(context).strip()
    if not export_dir:
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        run_dir = Path(log_root) / f"{stamp}_{int(time.time() * 1000) % 1000:03d}_test_predictor"
        export_dir_path = run_dir / "export"
    else:
        export_dir_path = Path(export_dir)
        run_dir = export_dir_path.parent
    runtime_dir = run_dir / "runtime"
    profiling_dir = run_dir / "profiling"
    metadata_dir = run_dir / "metadata"
    for path in (runtime_dir, export_dir_path, profiling_dir, metadata_dir):
        path.mkdir(parents=True, exist_ok=True)

    csv_path = str(export_dir_path / "test_predictor_query_probe.csv")
    probe_summary_path = str(export_dir_path / "test_predictor_query_probe_summary.json")
    validation_summary_path = str(
        export_dir_path / "test_predictor_validation_summary.json"
    )
    query_debug_csv_path = str(export_dir_path / "source_selection_debug.csv")
    gnss_debug_csv_path = str(export_dir_path / "gnss_epoch_debug.csv")
    gnss_visibility_csv_path = str(export_dir_path / "gnss_visibility_by_query.csv")
    lidar_debug_csv_path = str(export_dir_path / "predictor_lidar_debug.csv")
    lidar_primitives_debug_csv_path = str(
        export_dir_path / "predictor_lidar_primitives_debug.csv"
    )
    fusion_debug_csv_path = str(export_dir_path / "predictor_fusion_debug.csv")
    fallback_reason_csv_path = str(export_dir_path / "fallback_reason_by_time.csv")
    stale_debug_csv_path = str(export_dir_path / "stale_snapshot_debug.csv")
    latency_stress_tick_csv_path = str(export_dir_path / "latency_stress_tick_debug.csv")
    map_snapshot_csv_path = str(export_dir_path / "downsampled_map.csv")
    timing_csv_path = str(profiling_dir / "latency_debug.csv")
    launch_metadata_path = metadata_dir / "predictor_launch_config.json"
    run_manifest_path = metadata_dir / "run_manifest.json"
    probe_metadata_path = str(metadata_dir / "predictor_probe_config.json")
    araim_launch = Path(iap_share) / "launch" / "test_araim.launch.py"
    run_duration_s = LaunchConfiguration("run_duration_s").perform(context)
    validation_duration_s = LaunchConfiguration("validation_duration_s").perform(context)
    start_planner = _as_bool(LaunchConfiguration("start_planner").perform(context))
    launch_metadata = {
        "experiment": experiment,
        "araim_experiment": LaunchConfiguration("araim_experiment").perform(context),
        "predictor_output_mode": LaunchConfiguration("predictor_output_mode").perform(context),
        "predictor_enable_debug_log": debug_enabled,
        "predictor_fusion_conservative_max_with_gnss": _as_bool(
            LaunchConfiguration("predictor_fusion_conservative_max_with_gnss").perform(context)
        ),
        "start_planner": start_planner,
        "planner_odom_topic": "/sim/drone_0/truth_odom",
        "planner_uses_predictor": False,
        "planner_risk_overlay_enabled": False,
        "config_subdir": config_subdir,
        "log_root": str(log_root),
        "run_dir": str(run_dir),
        "runtime_dir": str(runtime_dir),
        "export_dir": str(export_dir_path),
        "profiling_dir": str(profiling_dir),
        "metadata_dir": str(metadata_dir),
        "csv_path": csv_path,
        "probe_summary_path": probe_summary_path,
        "validation_summary_path": validation_summary_path,
        "validator_required_selected_source": LaunchConfiguration("validator_required_selected_source").perform(context),
        "validator_require_debug_logs": _as_bool(
            LaunchConfiguration("validator_require_debug_logs").perform(context)
        ),
        "probe_current_variant_set": LaunchConfiguration("probe_current_variant_set").perform(context),
        "probe_current_high_hpl_m": float(
            LaunchConfiguration("probe_current_high_hpl_m").perform(context)
        ),
        "probe_current_high_vpl_m": float(
            LaunchConfiguration("probe_current_high_vpl_m").perform(context)
        ),
        "probe_current_unsafe_hpl_m": float(
            LaunchConfiguration("probe_current_unsafe_hpl_m").perform(context)
        ),
        "probe_current_unsafe_vpl_m": float(
            LaunchConfiguration("probe_current_unsafe_vpl_m").perform(context)
        ),
        "probe_current_unsafe_state": int(
            LaunchConfiguration("probe_current_unsafe_state").perform(context)
        ),
        "probe_stale_variant_set": LaunchConfiguration("probe_stale_variant_set").perform(context),
        "probe_stale_age_s": float(
            LaunchConfiguration("probe_stale_age_s").perform(context)
        ),
        "probe_latency_stress_batch_sizes": LaunchConfiguration(
            "probe_latency_stress_batch_sizes"
        ).perform(context),
        "probe_gnss_sigma_scale": float(
            LaunchConfiguration("probe_gnss_sigma_scale").perform(context)
        ),
        "predictor_enable_freshness_guard": _as_bool(
            LaunchConfiguration("predictor_enable_freshness_guard").perform(context)
        ),
        "predictor_max_odom_age_s": float(
            LaunchConfiguration("predictor_max_odom_age_s").perform(context)
        ),
        "predictor_max_integrity_age_s": float(
            LaunchConfiguration("predictor_max_integrity_age_s").perform(context)
        ),
        "predictor_max_gnss_age_s": float(
            LaunchConfiguration("predictor_max_gnss_age_s").perform(context)
        ),
        "predictor_max_snapshot_age_s": float(
            LaunchConfiguration("predictor_max_snapshot_age_s").perform(context)
        ),
    }
    if debug_enabled:
        launch_metadata["debug_files"] = {
            "query_debug_csv_path": query_debug_csv_path,
            "gnss_debug_csv_path": gnss_debug_csv_path,
            "gnss_visibility_csv_path": gnss_visibility_csv_path,
            "lidar_debug_csv_path": lidar_debug_csv_path,
            "lidar_primitives_debug_csv_path": lidar_primitives_debug_csv_path,
            "fusion_debug_csv_path": fusion_debug_csv_path,
            "fallback_reason_csv_path": fallback_reason_csv_path,
            "stale_debug_csv_path": stale_debug_csv_path,
            "latency_stress_tick_csv_path": latency_stress_tick_csv_path,
            "map_snapshot_csv_path": map_snapshot_csv_path,
            "timing_csv_path": timing_csv_path,
        }
    for name in ARAIM_PASSTHROUGH_ARGUMENTS:
        value = context.launch_configurations.get(name, "").strip()
        if value:
            launch_metadata[name] = value
    _write_json(launch_metadata_path, launch_metadata)
    _write_json(run_manifest_path, launch_metadata)

    araim_launch_arguments = {
        "experiment": LaunchConfiguration("araim_experiment"),
        "config_subdir": config_subdir,
        "start_rviz": LaunchConfiguration("start_rviz"),
        "record_bag": LaunchConfiguration("record_bag"),
        "run_validator": "false",
        "run_duration_s": run_duration_s,
        "validation_duration_s": validation_duration_s,
        "bag_output_dir": LaunchConfiguration("bag_output_dir"),
        "start_planner": LaunchConfiguration("start_planner"),
    }
    for name in ARAIM_PASSTHROUGH_ARGUMENTS:
        value = context.launch_configurations.get(name, "").strip()
        if value:
            araim_launch_arguments[name] = LaunchConfiguration(name)

    return [
        LogInfo(msg=f"[test_predictor] experiment preset: {experiment}"),
        LogInfo(msg=f"[test_predictor] run dir: {run_dir}"),
        LogInfo(msg=f"[test_predictor] export dir: {export_dir_path}"),
        LogInfo(msg=f"[test_predictor] debug log: {debug_enabled}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(araim_launch)),
            launch_arguments=araim_launch_arguments.items(),
        ),
        Node(
            package="iap",
            executable="test_predictor_query_probe",
            name="test_predictor_query_probe",
            output="screen",
            parameters=[
                {"predictor_output_mode": LaunchConfiguration("predictor_output_mode")},
                {"odom_topic": LaunchConfiguration("predictor_odom_topic")},
                {"map_topic": LaunchConfiguration("predictor_map_topic")},
                {"integrity_topic": "/iap/integrity"},
                {"csv_path": csv_path},
                {"summary_path": probe_summary_path},
                {"enable_debug_log": debug_enabled},
                {"query_set": LaunchConfiguration("probe_query_set")},
                {"query_debug_csv_path": query_debug_csv_path},
                {"gnss_debug_csv_path": gnss_debug_csv_path},
                {"gnss_visibility_csv_path": gnss_visibility_csv_path},
                {"lidar_debug_csv_path": lidar_debug_csv_path},
                {"lidar_primitives_debug_csv_path": lidar_primitives_debug_csv_path},
                {"fusion_debug_csv_path": fusion_debug_csv_path},
                {"fallback_reason_csv_path": fallback_reason_csv_path},
                {"stale_debug_csv_path": stale_debug_csv_path},
                {"latency_stress_tick_csv_path": latency_stress_tick_csv_path},
                {"map_snapshot_csv_path": map_snapshot_csv_path},
                {"timing_csv_path": timing_csv_path},
                {"probe_metadata_path": probe_metadata_path},
                {"debug_max_lidar_primitives": int(
                    LaunchConfiguration("predictor_debug_max_lidar_primitives").perform(context)
                )},
                {"use_lidar_primitives": _as_bool(
                    LaunchConfiguration("probe_use_lidar_primitives").perform(context)
                )},
                {"enable_current_prior": _as_bool(
                    LaunchConfiguration("probe_enable_current_prior").perform(context)
                )},
                {"require_gnss_for_selected_output": _as_bool(
                    LaunchConfiguration("probe_require_gnss_for_selected_output").perform(context)
                )},
                {"current_variant_set": LaunchConfiguration("probe_current_variant_set")},
                {"current_high_hpl_m": float(
                    LaunchConfiguration("probe_current_high_hpl_m").perform(context)
                )},
                {"current_high_vpl_m": float(
                    LaunchConfiguration("probe_current_high_vpl_m").perform(context)
                )},
                {"current_unsafe_hpl_m": float(
                    LaunchConfiguration("probe_current_unsafe_hpl_m").perform(context)
                )},
                {"current_unsafe_vpl_m": float(
                    LaunchConfiguration("probe_current_unsafe_vpl_m").perform(context)
                )},
                {"current_unsafe_state": int(
                    LaunchConfiguration("probe_current_unsafe_state").perform(context)
                )},
                {"stale_variant_set": LaunchConfiguration("probe_stale_variant_set")},
                {"stale_age_s": float(
                    LaunchConfiguration("probe_stale_age_s").perform(context)
                )},
                {"enable_map_snapshot": _as_bool(
                    LaunchConfiguration("probe_enable_map_snapshot").perform(context)
                )},
                {"query_min_period_s": float(
                    LaunchConfiguration("probe_query_min_period_s").perform(context)
                )},
                {"latency_stress_batch_sizes": LaunchConfiguration(
                    "probe_latency_stress_batch_sizes"
                )},
                {"gnss_epoch_max_age_s": float(
                    LaunchConfiguration("probe_gnss_epoch_max_age_s").perform(context)
                )},
                {"gnss_sigma_scale": float(
                    LaunchConfiguration("probe_gnss_sigma_scale").perform(context)
                )},
                {"lidar_map_max_points": int(
                    LaunchConfiguration("probe_lidar_map_max_points").perform(context)
                )},
                {"lidar_enable_legacy_observability": _as_bool(
                    LaunchConfiguration("probe_lidar_enable_legacy_observability").perform(context)
                )},
                {"fusion_conservative_max_with_gnss": _as_bool(
                    LaunchConfiguration("predictor_fusion_conservative_max_with_gnss").perform(context)
                )},
                {"enable_freshness_guard": _as_bool(
                    LaunchConfiguration("predictor_enable_freshness_guard").perform(context)
                )},
                {"max_odom_age_s": float(
                    LaunchConfiguration("predictor_max_odom_age_s").perform(context)
                )},
                {"max_integrity_age_s": float(
                    LaunchConfiguration("predictor_max_integrity_age_s").perform(context)
                )},
                {"max_gnss_age_s": float(
                    LaunchConfiguration("predictor_max_gnss_age_s").perform(context)
                )},
                {"max_snapshot_age_s": float(
                    LaunchConfiguration("predictor_max_snapshot_age_s").perform(context)
                )},
            ],
        ),
        Node(
            package="iap",
            executable="test_predictor_validator.py",
            name="test_predictor_validator",
            output="screen",
            condition=IfCondition("true" if run_validator_enabled else "false"),
            parameters=[
                {"csv_path": csv_path},
                {"probe_summary_path": probe_summary_path},
                {"validation_summary_path": validation_summary_path},
                {"duration_s": float(validation_duration_s)},
                {"min_queries": int(LaunchConfiguration("validator_min_queries").perform(context))},
                {"predictor_output_mode": LaunchConfiguration("predictor_output_mode")},
                {"required_selected_source": LaunchConfiguration("validator_required_selected_source")},
                {"debug_enabled": debug_enabled},
                {"require_debug_logs": _as_bool(
                    LaunchConfiguration("validator_require_debug_logs").perform(context)
                )},
                {"debug_file_paths": ",".join([
                    query_debug_csv_path,
                    gnss_debug_csv_path,
                    gnss_visibility_csv_path,
                    lidar_debug_csv_path,
                    lidar_primitives_debug_csv_path,
                    fusion_debug_csv_path,
                    fallback_reason_csv_path,
                    latency_stress_tick_csv_path,
                    map_snapshot_csv_path,
                    timing_csv_path,
                    probe_metadata_path,
                    str(launch_metadata_path),
                    str(run_manifest_path),
                ])},
                {"require_selected_valid": _as_bool(
                    LaunchConfiguration("validator_require_selected_valid").perform(context)
                )},
                {"require_gnss_valid": _as_bool(
                    LaunchConfiguration("validator_require_gnss_valid").perform(context)
                )},
                {"require_lidar_valid": _as_bool(
                    LaunchConfiguration("validator_require_lidar_valid").perform(context)
                )},
                {"require_fusion_valid": _as_bool(
                    LaunchConfiguration("validator_require_fusion_valid").perform(context)
                )},
                {"max_lambda_sum_error": float(
                    LaunchConfiguration("validator_max_lambda_sum_error").perform(context)
                )},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("experiment", default_value="predictor_fusion_nominal"),
            DeclareLaunchArgument("araim_experiment", default_value="fused_nominal"),
            DeclareLaunchArgument("config_subdir", default_value="sim_demo11"),
            DeclareLaunchArgument("predictor_output_mode", default_value="fusion"),
            DeclareLaunchArgument("predictor_enable_debug_log", default_value="false"),
            DeclareLaunchArgument(
                "predictor_fusion_conservative_max_with_gnss",
                default_value="false",
            ),
            DeclareLaunchArgument("predictor_log_root", default_value=""),
            DeclareLaunchArgument("predictor_export_dir", default_value=""),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_planner", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("run_validator", default_value="true"),
            DeclareLaunchArgument("use_gnss", default_value=""),
            DeclareLaunchArgument("use_araim", default_value=""),
            DeclareLaunchArgument("enable_gnss_integrity", default_value=""),
            DeclareLaunchArgument("enable_gnss_araim", default_value=""),
            DeclareLaunchArgument("enable_lidar_integrity", default_value=""),
            DeclareLaunchArgument("integrity_fusion_mode", default_value=""),
            DeclareLaunchArgument("init_x", default_value=""),
            DeclareLaunchArgument("init_y", default_value=""),
            DeclareLaunchArgument("init_z", default_value=""),
            DeclareLaunchArgument("goal_x", default_value=""),
            DeclareLaunchArgument("goal_y", default_value=""),
            DeclareLaunchArgument("goal_z", default_value=""),
            DeclareLaunchArgument("point_num", default_value=""),
            DeclareLaunchArgument("tree_density_lower_left_per_m2", default_value=""),
            DeclareLaunchArgument("tree_density_lower_right_per_m2", default_value=""),
            DeclareLaunchArgument("tree_density_upper_left_per_m2", default_value=""),
            DeclareLaunchArgument("tree_density_upper_right_per_m2", default_value=""),
            DeclareLaunchArgument("canopy_density_lower_left", default_value=""),
            DeclareLaunchArgument("canopy_density_lower_right", default_value=""),
            DeclareLaunchArgument("canopy_density_upper_left", default_value=""),
            DeclareLaunchArgument("canopy_density_upper_right", default_value=""),
            DeclareLaunchArgument("terminal_wall_enabled", default_value=""),
            DeclareLaunchArgument("terminal_wall_feature_count", default_value=""),
            DeclareLaunchArgument("corridor_walls_enabled", default_value=""),
            DeclareLaunchArgument("corridor_floor_enabled", default_value=""),
            DeclareLaunchArgument("corridor_x_min_m", default_value=""),
            DeclareLaunchArgument("corridor_x_max_m", default_value=""),
            DeclareLaunchArgument("corridor_half_width_y_m", default_value=""),
            DeclareLaunchArgument("corridor_floor_thickness_z_m", default_value=""),
            DeclareLaunchArgument("corridor_surface_resolution_m", default_value=""),
            DeclareLaunchArgument("gnss_ephemeris_source", default_value=""),
            DeclareLaunchArgument("gnss_enabled_constellations", default_value=""),
            DeclareLaunchArgument("gnss_scenario_file", default_value=""),
            DeclareLaunchArgument("gnss_pr_noise_base", default_value=""),
            DeclareLaunchArgument("gnss_dop_noise_base", default_value=""),
            DeclareLaunchArgument("gnss_enable_map_occlusion", default_value=""),
            DeclareLaunchArgument("gnss_enable_skymask", default_value=""),
            DeclareLaunchArgument("gnss_enable_nlos", default_value=""),
            DeclareLaunchArgument("gnss_enable_multipath", default_value=""),
            DeclareLaunchArgument("gnss_enable_fault_injection", default_value=""),
            DeclareLaunchArgument(
                "bag_output_dir",
                default_value="/home/dev/ws_iap/src/iap/results/predictor_validation/bags",
            ),
            DeclareLaunchArgument("run_duration_s", default_value="90"),
            DeclareLaunchArgument("validation_duration_s", default_value="85"),
            DeclareLaunchArgument(
                "predictor_odom_topic", default_value="/drone_0_visual_slam/odom"
            ),
            DeclareLaunchArgument(
                "predictor_map_topic", default_value="/map_generator/global_cloud"
            ),
            DeclareLaunchArgument("probe_use_lidar_primitives", default_value="true"),
            DeclareLaunchArgument("probe_query_set", default_value="current_pose"),
            DeclareLaunchArgument("probe_enable_current_prior", default_value="true"),
            DeclareLaunchArgument(
                "probe_require_gnss_for_selected_output", default_value="false"
            ),
            DeclareLaunchArgument("probe_current_variant_set", default_value="observed"),
            DeclareLaunchArgument("probe_current_high_hpl_m", default_value="1000.0"),
            DeclareLaunchArgument("probe_current_high_vpl_m", default_value="1000.0"),
            DeclareLaunchArgument("probe_current_unsafe_hpl_m", default_value="500.0"),
            DeclareLaunchArgument("probe_current_unsafe_vpl_m", default_value="600.0"),
            DeclareLaunchArgument("probe_current_unsafe_state", default_value="2"),
            DeclareLaunchArgument("probe_stale_variant_set", default_value="observed"),
            DeclareLaunchArgument("probe_stale_age_s", default_value="2.0"),
            DeclareLaunchArgument("probe_enable_map_snapshot", default_value="true"),
            DeclareLaunchArgument("probe_query_min_period_s", default_value="0.0"),
            DeclareLaunchArgument("probe_latency_stress_batch_sizes", default_value=""),
            DeclareLaunchArgument("probe_gnss_epoch_max_age_s", default_value="0.5"),
            DeclareLaunchArgument("probe_gnss_sigma_scale", default_value="1.0"),
            DeclareLaunchArgument("probe_lidar_map_max_points", default_value="2500"),
            DeclareLaunchArgument("predictor_debug_max_lidar_primitives", default_value="500"),
            DeclareLaunchArgument("predictor_enable_freshness_guard", default_value="false"),
            DeclareLaunchArgument("predictor_max_odom_age_s", default_value="0.5"),
            DeclareLaunchArgument("predictor_max_integrity_age_s", default_value="0.5"),
            DeclareLaunchArgument("predictor_max_gnss_age_s", default_value="0.5"),
            DeclareLaunchArgument("predictor_max_snapshot_age_s", default_value="0.5"),
            DeclareLaunchArgument(
                "probe_lidar_enable_legacy_observability", default_value="false"
            ),
            DeclareLaunchArgument("validator_min_queries", default_value="10"),
            DeclareLaunchArgument(
                "validator_required_selected_source", default_value="FUSION"
            ),
            DeclareLaunchArgument("validator_require_selected_valid", default_value="true"),
            DeclareLaunchArgument("validator_require_gnss_valid", default_value="true"),
            DeclareLaunchArgument("validator_require_lidar_valid", default_value="true"),
            DeclareLaunchArgument("validator_require_fusion_valid", default_value="true"),
            DeclareLaunchArgument("validator_require_debug_logs", default_value="false"),
            DeclareLaunchArgument(
                "validator_max_lambda_sum_error", default_value="1.0e-8"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
