import json
import os
import shutil
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


OPEN_MAP_PRESET = {
    "tree_density_lower_left_per_m2": "0.02",
    "tree_density_lower_right_per_m2": "0.02",
    "tree_density_upper_left_per_m2": "0.02",
    "tree_density_upper_right_per_m2": "0.02",
    "stratified_cell_size_m": "4.0",
    "clear_corridor_enabled": "true",
    "clear_corridor_center_y_m": "0.0",
    "clear_corridor_half_width_y_m": "1.8",
    "clear_corridor_x_min_m": "-12.5",
    "clear_corridor_x_max_m": "12.5",
    "trunk_radius_m": "0.10",
    "trunk_min_height_m": "0.30",
    "trunk_max_height_m": "1.15",
    "canopy_density_lower_left": "0.0",
    "canopy_density_lower_right": "0.0",
    "canopy_density_upper_left": "0.0",
    "canopy_density_upper_right": "0.0",
    "terminal_wall_enabled": "false",
}


FEATURE_RICH_MAP_PRESET = {
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
}


SPARSE_CORRIDOR_MAP_PRESET = {
    "tree_density_lower_left_per_m2": "0.06",
    "tree_density_lower_right_per_m2": "0.06",
    "tree_density_upper_left_per_m2": "0.06",
    "tree_density_upper_right_per_m2": "0.06",
    "canopy_density_lower_left": "0.0",
    "canopy_density_lower_right": "0.0",
    "canopy_density_upper_left": "0.02",
    "canopy_density_upper_right": "0.02",
    "terminal_wall_enabled": "false",
}


CORRIDOR_DEGENERATE_MAP_PRESET = {
    "forest_size_x_m": "30.0",
    "forest_size_y_m": "6.0",
    "tree_density_lower_left_per_m2": "0.0",
    "tree_density_lower_right_per_m2": "0.0",
    "tree_density_upper_left_per_m2": "0.0",
    "tree_density_upper_right_per_m2": "0.0",
    "canopy_density_lower_left": "0.0",
    "canopy_density_lower_right": "0.0",
    "canopy_density_upper_left": "0.0",
    "canopy_density_upper_right": "0.0",
    "terminal_wall_enabled": "false",
    "corridor_walls_enabled": "true",
    "corridor_floor_enabled": "true",
    "corridor_x_min_m": "-14.0",
    "corridor_x_max_m": "14.0",
    "corridor_half_width_y_m": "2.0",
    "corridor_wall_z_min_m": "0.0",
    "corridor_wall_z_max_m": "3.0",
    "corridor_wall_thickness_y_m": "0.10",
    "corridor_floor_thickness_z_m": "0.05",
    "corridor_surface_resolution_m": "0.10",
}


DEFAULT_ROUTE_PRESET = {
    "init_x": "-12.0",
    "init_y": "0.0",
    "init_z": "1.2",
    "goal_x": "12.0",
    "goal_y": "0.0",
    "goal_z": "1.2",
    "point_num": "1",
}


FULL_ROUTE_PRESET = {
    "init_x": "-12.0",
    "init_y": "-6.0",
    "init_z": "1.2",
    "goal_x": "-8.0",
    "goal_y": "-6.0",
    "goal_z": "1.2",
    "point_num": "6",
    "point1_x": "-3.0",
    "point1_y": "-4.0",
    "point1_z": "1.2",
    "point2_x": "1.0",
    "point2_y": "0.0",
    "point2_z": "1.2",
    "point3_x": "5.0",
    "point3_y": "3.0",
    "point3_z": "1.2",
    "point4_x": "9.0",
    "point4_y": "0.0",
    "point4_z": "1.2",
    "point5_x": "12.0",
    "point5_y": "-4.0",
    "point5_z": "1.2",
    "point6_x": "12.0",
    "point6_y": "4.0",
    "point6_z": "1.2",
}


GNSS_OPEN_SKY_PRESET = {
    "gnss_ephemeris_source": "rinex",
    "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
    "gnss_scenario_file": "config/gnss_sim/demo7_open_sky.yaml",
    "gnss_pr_noise_base": "1.0",
    "gnss_dop_noise_base": "0.03",
    "gnss_enable_map_occlusion": "false",
    "gnss_enable_skymask": "false",
    "gnss_enable_nlos": "false",
    "gnss_enable_multipath": "false",
    "gnss_enable_fault_injection": "false",
}


GNSS_DEGRADED_PRESET = {
    "gnss_ephemeris_source": "rinex",
    "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
    "gnss_scenario_file": "config/gnss_sim/demo7_skymask_nlos.yaml",
    "gnss_pr_noise_base": "5.0",
    "gnss_dop_noise_base": "0.5",
    "gnss_enable_map_occlusion": "true",
    "gnss_enable_skymask": "true",
    "gnss_enable_nlos": "true",
    "gnss_enable_multipath": "true",
    "gnss_enable_fault_injection": "false",
}


GNSS_FAULT_PRESET = {
    "gnss_ephemeris_source": "rinex",
    "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
    "gnss_scenario_file": "config/gnss_sim/demo7_fault_injection.yaml",
    "gnss_pr_noise_base": "5.0",
    "gnss_dop_noise_base": "0.5",
    "gnss_enable_map_occlusion": "false",
    "gnss_enable_skymask": "true",
    "gnss_enable_nlos": "false",
    "gnss_enable_multipath": "false",
    "gnss_enable_fault_injection": "true",
}


EXPERIMENT_PRESETS = {
    "manual": {},
    "gnss_open_sky": {
        **DEFAULT_ROUTE_PRESET,
        **OPEN_MAP_PRESET,
        **GNSS_OPEN_SKY_PRESET,
        "init_z": "1.5",
        "goal_z": "1.5",
        "use_gnss": "true",
        "use_araim": "true",
        "gnss_time_source": "odom_stamp",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "false",
        "integrity_fusion_mode": "gnss_only",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "GNSS",
    },
    "lidar_feature_rich": {
        **DEFAULT_ROUTE_PRESET,
        **FEATURE_RICH_MAP_PRESET,
        "use_gnss": "false",
        "use_araim": "true",
        "enable_gnss_integrity": "false",
        "enable_gnss_araim": "false",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "lidar_only",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "LIDAR",
    },
    "lidar_corridor_degenerate": {
        **DEFAULT_ROUTE_PRESET,
        **CORRIDOR_DEGENERATE_MAP_PRESET,
        "use_gnss": "false",
        "use_araim": "true",
        "enable_gnss_integrity": "false",
        "enable_gnss_araim": "false",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "lidar_only",
        "integrity_require_valid_gnss": "false",
        "integrity_require_valid_lidar": "true",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "LIDAR",
    },
    "fallback_only": {
        **DEFAULT_ROUTE_PRESET,
        **OPEN_MAP_PRESET,
        "use_gnss": "false",
        "use_araim": "true",
        "enable_gnss_integrity": "false",
        "enable_gnss_araim": "false",
        "enable_lidar_integrity": "false",
        "integrity_fusion_mode": "fallback_only",
        "validator_require_gnss_valid": "false",
        "validator_require_lidar_valid": "false",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "FALLBACK",
    },
    "fused_nominal": {
        **DEFAULT_ROUTE_PRESET,
        **FEATURE_RICH_MAP_PRESET,
        **GNSS_OPEN_SKY_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "",
    },
    "gnss_degraded_lidar_good": {
        **DEFAULT_ROUTE_PRESET,
        **FEATURE_RICH_MAP_PRESET,
        **GNSS_DEGRADED_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "",
    },
    "lidar_degraded_gnss_good": {
        **DEFAULT_ROUTE_PRESET,
        **SPARSE_CORRIDOR_MAP_PRESET,
        **GNSS_OPEN_SKY_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "",
    },
    "gnss_outage_lidar_recovery": {
        **DEFAULT_ROUTE_PRESET,
        **FEATURE_RICH_MAP_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "gnss_ephemeris_source": "rinex",
        "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
        "gnss_scenario_file": "generated:gnss_outage",
        "gnss_pr_noise_base": "5.0",
        "gnss_dop_noise_base": "0.5",
        "gnss_enable_map_occlusion": "false",
        "gnss_enable_skymask": "false",
        "gnss_enable_nlos": "false",
        "gnss_enable_multipath": "false",
        "gnss_enable_fault_injection": "true",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "",
    },
    "gnss_single_fault": {
        **DEFAULT_ROUTE_PRESET,
        **OPEN_MAP_PRESET,
        **GNSS_FAULT_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "false",
        "integrity_fusion_mode": "gnss_only",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "GNSS",
    },
    "full_route_transition": {
        **FULL_ROUTE_PRESET,
        **FEATURE_RICH_MAP_PRESET,
        "use_gnss": "true",
        "use_araim": "true",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "true",
        "integrity_fusion_mode": "max_pl",
        "gnss_ephemeris_source": "rinex",
        "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
        "gnss_scenario_file": "generated:full_route_transition",
        "gnss_pr_noise_base": "5.0",
        "gnss_dop_noise_base": "0.5",
        "gnss_enable_map_occlusion": "true",
        "gnss_enable_skymask": "true",
        "gnss_enable_nlos": "true",
        "gnss_enable_multipath": "true",
        "gnss_enable_fault_injection": "true",
        "run_duration_s": "130",
        "validation_duration_s": "120",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "true",
        "validator_require_fallback_valid": "true",
        "validator_required_final_source": "",
    },
}


def _launch_arg_overrides():
    keys = set()
    for arg in sys.argv:
        if ":=" in arg:
            keys.add(arg.split(":=", 1)[0])
    return keys


def _launch_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _apply_experiment_preset(context, iap_share):
    experiment = LaunchConfiguration("experiment").perform(context).strip()
    if not experiment:
        experiment = "manual"
    if experiment not in EXPERIMENT_PRESETS:
        valid = ", ".join(sorted(EXPERIMENT_PRESETS.keys()))
        raise RuntimeError(f"unknown test_araim experiment '{experiment}'. Valid: {valid}")

    user_overrides = _launch_arg_overrides()
    preset = EXPERIMENT_PRESETS[experiment]
    for key, value in preset.items():
        if key in user_overrides:
            continue
        if key == "gnss_scenario_file" and str(value).startswith("config/"):
            value = str(Path(iap_share) / str(value))
        context.launch_configurations[key] = _launch_value(value)
    return experiment


def _generated_gnss_scenario(name):
    header = """anchor:
  lat_deg: 31.2304
  lon_deg: 121.4737
  alt_m: 25.0

"""
    if name == "gnss_outage":
        faults = "\n".join(
            [
                f"  - {{constellation: GPS, sat: {prn}, start_time_s: 35.0, duration_s: 25.0, drop: true}}"
                for prn in range(1, 25)
            ]
        )
        return header + """skymask:
  enabled: false
  default_min_elevation_deg: 10.0
  az_el_deg: []

faults:
""" + faults + "\n"
    if name == "full_route_transition":
        bias_fault = "  - {constellation: GPS, sat: 7, start_time_s: 30.0, duration_s: 18.0, pseudorange_bias_m: 35.0, drop: false, cn0_degrade_dbhz: 8.0}"
        outage_faults = "\n".join(
            [
                f"  - {{constellation: GPS, sat: {prn}, start_time_s: 65.0, duration_s: 18.0, drop: true}}"
                for prn in range(1, 17)
            ]
        )
        return header + """skymask:
  enabled: true
  default_min_elevation_deg: 10.0
  az_el_deg:
    - {az_deg: 0, min_el_deg: 20}
    - {az_deg: 90, min_el_deg: 45}
    - {az_deg: 180, min_el_deg: 15}
    - {az_deg: 270, min_el_deg: 30}

faults:
""" + bias_fault + "\n" + outage_faults + "\n"
    raise RuntimeError(f"unknown generated GNSS scenario '{name}'")


def _materialize_gnss_scenario(scenario_file, export_dir):
    if not str(scenario_file).startswith("generated:"):
        return scenario_file
    scenario_name = str(scenario_file).split(":", 1)[1]
    path = Path(export_dir) / f"{scenario_name}.yaml"
    path.write_text(_generated_gnss_scenario(scenario_name))
    return str(path)


def _runtime_config(context, use_gnss, use_araim, allow_truth_alignment):
    iap_share = Path(get_package_share_directory("iap"))
    base_config = iap_share / "config"
    config_subdir = LaunchConfiguration("config_subdir").perform(context)
    config_name = config_subdir.replace("/", "_").replace("\\", "_")
    runtime_root = Path("/tmp") / (
        f"iap_{config_name}_test_araim_{os.getpid()}_{int(time.time() * 1000)}"
    )
    runtime_config_dir = runtime_root / config_subdir
    export_dir = runtime_root / "export"
    export_dir.mkdir(parents=True, exist_ok=True)

    shutil.copytree(
        base_config / config_subdir,
        runtime_config_dir,
        ignore_dangling_symlinks=True,
    )
    shutil.copytree(
        base_config / "sim_ego",
        runtime_root / "sim_ego",
        ignore_dangling_symlinks=True,
    )

    config_ros_path = runtime_config_dir / "config_ros.json"
    config_gnss_path = runtime_config_dir / "config_gnss.json"

    with config_ros_path.open() as f:
        config_ros = json.load(f)
    modules = ["libsim_extension.so"]
    if use_gnss:
        modules.insert(0, "libgnss_extension.so")
    if use_araim:
        insert_at = 1 if use_gnss else 0
        modules.insert(insert_at, "libintegrity_extension.so")
    config_ros["glim_ros"]["extension_modules"] = modules
    config_ros["glim_ros"]["imu_topic"] = "/sim/drone_0/imu_iap"
    config_ros["glim_ros"]["points_topic"] = "/sim/drone_0/lidar_body"
    config_ros["glim_ros"]["dump_path"] = str(runtime_root / "dump")
    config_ros["glim_ros"]["sim"]["align_planner_odom_to_truth"] = allow_truth_alignment
    config_ros["glim_ros"]["sim"]["metrics_csv_path"] = str(
        export_dir / "iap_sim_truth_vs_est.csv"
    )
    with config_ros_path.open("w") as f:
        json.dump(config_ros, f, indent=2)
        f.write("\n")

    with config_gnss_path.open() as f:
        config_gnss = json.load(f)
    gnss = config_gnss["gnss"]
    gnss["pr_noise_base"] = float(
        LaunchConfiguration("gnss_pr_noise_base").perform(context)
    )
    gnss["dop_noise_base"] = float(
        LaunchConfiguration("gnss_dop_noise_base").perform(context)
    )
    config_gnss["gnss"]["debug_csv_path"] = str(export_dir / "iap_gnss_factor_debug.csv")
    integrity = config_gnss["integrity"]
    enable_gnss_integrity = _as_bool(
        LaunchConfiguration("enable_gnss_integrity").perform(context)
    )
    enable_gnss_araim = _as_bool(
        LaunchConfiguration("enable_gnss_araim").perform(context)
    )
    enable_lidar_integrity = _as_bool(
        LaunchConfiguration("enable_lidar_integrity").perform(context)
    )
    enable_pl_decomp_csv = _as_bool(
        LaunchConfiguration("enable_araim_pl_decomp_csv").perform(context)
    )
    fusion_mode = LaunchConfiguration("integrity_fusion_mode").perform(context)
    integrity["enable"] = bool(use_araim)
    integrity["enable_araim"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_araim_csv"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_araim_pl_decomp_csv"] = bool(
        use_araim
        and use_gnss
        and enable_gnss_araim
        and enable_pl_decomp_csv
    )
    integrity["publish_topic"] = "/iap/integrity"
    integrity["araim_csv_path"] = str(export_dir / "iap_araim.csv")
    integrity["araim_pl_decomp_csv_path"] = str(export_dir / "iap_araim_pl_decomp.csv")
    integrity["traj_csv_path"] = str(export_dir / "traj_with_gnss.csv")
    integrity["enable_lidar_araim_stage0_csv"] = bool(
        use_araim and enable_lidar_integrity and enable_pl_decomp_csv
    )
    integrity["lidar_araim_stage0_csv_path"] = str(
        export_dir / "iap_lidar_araim_stage0.csv"
    )
    integrity["enable_gnss_integrity"] = bool(use_gnss and enable_gnss_integrity)
    integrity["enable_gnss_araim"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_lidar_integrity"] = bool(use_araim and enable_lidar_integrity)
    integrity["integrity_fusion_mode"] = fusion_mode
    integrity["integrity_require_valid_gnss"] = _as_bool(
        LaunchConfiguration("integrity_require_valid_gnss").perform(context)
    )
    integrity["integrity_require_valid_lidar"] = _as_bool(
        LaunchConfiguration("integrity_require_valid_lidar").perform(context)
    )
    integrity["integrity_conservative_hpl_m"] = float(
        LaunchConfiguration("integrity_conservative_hpl_m").perform(context)
    )
    integrity["integrity_conservative_vpl_m"] = float(
        LaunchConfiguration("integrity_conservative_vpl_m").perform(context)
    )
    with config_gnss_path.open("w") as f:
        json.dump(config_gnss, f, indent=2)
        f.write("\n")

    return str(runtime_config_dir), str(runtime_root), str(export_dir)


def _odom_visualization_node(
    name,
    odom_topic,
    cmd_topic,
    topic_prefix,
    color,
    drone_id,
    sensor_text_use_fixed_position=False,
    sensor_text_fixed_position=(0.0, 0.0, 14.0),
):
    r, g, b = color
    text_x, text_y, text_z = sensor_text_fixed_position
    return Node(
        package="odom_visualization",
        executable="odom_visualization",
        name=name,
        output="screen",
        remappings=[
            ("odom", odom_topic),
            ("cmd", cmd_topic),
            ("pose", f"{topic_prefix}/pose"),
            ("path", f"{topic_prefix}/path"),
            ("robot", f"{topic_prefix}/robot"),
            ("velocity", f"{topic_prefix}/velocity"),
            ("sensor", f"{topic_prefix}/sensor_status"),
            ("time_gap", f"{topic_prefix}/time_gap"),
        ],
        parameters=[
            {"frame_id": "map"},
            {"robot_scale": 1.0},
            {"color/r": r},
            {"color/g": g},
            {"color/b": b},
            {"color/a": 1.0},
            {"tf45": False},
            {"drone_id": drone_id},
            {"sensor_text_use_fixed_position": bool(sensor_text_use_fixed_position)},
            {"sensor_text_fixed_x": float(text_x)},
            {"sensor_text_fixed_y": float(text_y)},
            {"sensor_text_fixed_z": float(text_z)},
        ],
    )


def _ego_planner_node(
    drone_id,
    planner_odom_topic,
    cloud_topic,
    camera_pose_topic,
    depth_topic,
    bspline_topic,
    map_size_x,
    map_size_y,
    map_size_z,
    goal_x,
    goal_y,
    goal_z,
    point_num,
    context,
):
    def lc(name):
        return LaunchConfiguration(name)

    return Node(
        package="ego_planner",
        executable="ego_planner_node",
        name=f"drone_{drone_id}_ego_planner_node",
        output="screen",
        remappings=[
            ("odom_world", planner_odom_topic),
            ("planning/bspline", bspline_topic),
            ("planning/data_display", f"/drone_{drone_id}_planning/data_display"),
            ("planning/broadcast_bspline_from_planner", "/broadcast_bspline"),
            ("planning/broadcast_bspline_to_planner", "/broadcast_bspline"),
            ("goal_point", f"/drone_{drone_id}_plan_vis/goal_point"),
            ("global_list", f"/drone_{drone_id}_plan_vis/global_list"),
            ("init_list", f"/drone_{drone_id}_plan_vis/init_list"),
            ("optimal_list", f"/drone_{drone_id}_plan_vis/optimal_list"),
            ("a_star_list", f"/drone_{drone_id}_plan_vis/a_star_list"),
            ("grid_map/odom", planner_odom_topic),
            ("grid_map/cloud", cloud_topic),
            ("grid_map/pose", camera_pose_topic),
            ("grid_map/depth", depth_topic),
            ("grid_map/occupancy_inflate", f"/drone_{drone_id}_grid/grid_map/occupancy_inflate"),
        ],
        parameters=[
            {"fsm/flight_type": 2},
            {"fsm/thresh_replan_time": 1.0},
            {"fsm/thresh_no_replan_meter": 1.0},
            {"fsm/planning_horizon": 7.5},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": int(point_num)},
            {"fsm/waypoint0_x": float(goal_x)},
            {"fsm/waypoint0_y": float(goal_y)},
            {"fsm/waypoint0_z": float(goal_z)},
            {"fsm/waypoint1_x": float(lc("point1_x").perform(context))},
            {"fsm/waypoint1_y": float(lc("point1_y").perform(context))},
            {"fsm/waypoint1_z": float(lc("point1_z").perform(context))},
            {"fsm/waypoint2_x": float(lc("point2_x").perform(context))},
            {"fsm/waypoint2_y": float(lc("point2_y").perform(context))},
            {"fsm/waypoint2_z": float(lc("point2_z").perform(context))},
            {"fsm/waypoint3_x": float(lc("point3_x").perform(context))},
            {"fsm/waypoint3_y": float(lc("point3_y").perform(context))},
            {"fsm/waypoint3_z": float(lc("point3_z").perform(context))},
            {"fsm/waypoint4_x": float(lc("point4_x").perform(context))},
            {"fsm/waypoint4_y": float(lc("point4_y").perform(context))},
            {"fsm/waypoint4_z": float(lc("point4_z").perform(context))},
            {"fsm/waypoint5_x": float(lc("point5_x").perform(context))},
            {"fsm/waypoint5_y": float(lc("point5_y").perform(context))},
            {"fsm/waypoint5_z": float(lc("point5_z").perform(context))},
            {"fsm/waypoint6_x": float(lc("point6_x").perform(context))},
            {"fsm/waypoint6_y": float(lc("point6_y").perform(context))},
            {"fsm/waypoint6_z": float(lc("point6_z").perform(context))},
            {"grid_map/resolution": 0.1},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": 5.5},
            {"grid_map/local_update_range_y": 5.5},
            {"grid_map/local_update_range_z": 4.5},
            {"grid_map/obstacles_inflation": 0.099},
            {"grid_map/local_map_margin": 10},
            {"grid_map/ground_height": -0.01},
            {"grid_map/cx": 321.04638671875},
            {"grid_map/cy": 243.44969177246094},
            {"grid_map/fx": 387.229248046875},
            {"grid_map/fy": 387.229248046875},
            {"grid_map/use_depth_filter": True},
            {"grid_map/depth_filter_tolerance": 0.15},
            {"grid_map/depth_filter_maxdist": 5.0},
            {"grid_map/depth_filter_mindist": 0.2},
            {"grid_map/depth_filter_margin": 2},
            {"grid_map/k_depth_scaling_factor": 1000.0},
            {"grid_map/skip_pixel": 2},
            {"grid_map/p_hit": 0.65},
            {"grid_map/p_miss": 0.35},
            {"grid_map/p_min": 0.12},
            {"grid_map/p_max": 0.90},
            {"grid_map/p_occ": 0.80},
            {"grid_map/min_ray_length": 0.1},
            {"grid_map/max_ray_length": 4.5},
            {"grid_map/virtual_ceil_height": 2.9},
            {"grid_map/visualization_truncate_height": 1.8},
            {"grid_map/show_occ_time": False},
            {"grid_map/pose_type": 1},
            {"grid_map/frame_id": "map"},
            {"p0.enable_risk_grid": _as_bool(lc("p0.enable_risk_grid").perform(context))},
            {"p0.resolution_m": 0.75},
            {"p0.size_x_m": 30.0},
            {"p0.size_y_m": 30.0},
            {"p0.size_z_m": 6.0},
            {"p0.horizons_s": [0.0, 0.5, 1.0, 1.5, 2.0]},
            {"p0.refresh_period_s": 0.5},
            {"p0.stale_timeout_s": 1.0},
            {"p0.debug_metrics_enable": _as_bool(lc("p0.debug_metrics_enable").perform(context))},
            {"p0.odom_topic": planner_odom_topic},
            {"p0.integrity_topic": "/iap/integrity"},
            {"p0.range_meas_topic": "/ublox_driver/range_meas"},
            {"p0.ephem_topic": "/ublox_driver/ephem"},
            {"p0.glo_ephem_topic": "/ublox_driver/glo_ephem"},
            {"p0.receiver_lla_topic": "/ublox_driver/receiver_lla"},
            {"p0.iono_topic": "/ublox_driver/iono_params"},
            {"p0.map_topic": "/map_generator/global_cloud"},
            {"p0.health_topic": "planning/risk_grid_health"},
            {"p1.use_integrity_cost": _as_bool(lc("p1.use_integrity_cost").perform(context))},
            {"p1.metrics_only": _as_bool(lc("p1.metrics_only").perform(context))},
            {"p1.lambda_integrity": float(lc("p1.lambda_integrity").perform(context))},
            {"p1.sample_dt_min_s": float(lc("p1.sample_dt_min_s").perform(context))},
            {"p1.sample_dt_scale": float(lc("p1.sample_dt_scale").perform(context))},
            {"p1.max_samples_per_eval": int(lc("p1.max_samples_per_eval").perform(context))},
            {"p1.integrity_cost_max": float(lc("p1.integrity_cost_max").perform(context))},
            {"p1.integrity_grad_norm_max": float(lc("p1.integrity_grad_norm_max").perform(context))},
            {"p1.unknown_policy": lc("p1.unknown_policy").perform(context)},
            {"p1.unknown_soft_penalty": float(lc("p1.unknown_soft_penalty").perform(context))},
            {"p1.debug_csv_enable": _as_bool(lc("p1.debug_csv_enable").perform(context))},
            {"p1.debug_csv_path": lc("p1.debug_csv_path").perform(context)},
            {"p2.enable_candidate_ranking": _as_bool(lc("p2.enable_candidate_ranking").perform(context))},
            {"p2.metrics_only": _as_bool(lc("p2.metrics_only").perform(context))},
            {"p2.sample_dt_s": float(lc("p2.sample_dt_s").perform(context))},
            {"p2.lambda_candidate_integrity": float(lc("p2.lambda_candidate_integrity").perform(context))},
            {"p2.w_max_cost": float(lc("p2.w_max_cost").perform(context))},
            {"p2.w_unknown": float(lc("p2.w_unknown").perform(context))},
            {"p2.w_stale": float(lc("p2.w_stale").perform(context))},
            {"p2.min_valid_ratio": float(lc("p2.min_valid_ratio").perform(context))},
            {"p2.debug_csv_enable": _as_bool(lc("p2.debug_csv_enable").perform(context))},
            {"p2.debug_csv_path": lc("p2.debug_csv_path").perform(context)},
            {"p3.enable_local_reference_bias": _as_bool(lc("p3.enable_local_reference_bias").perform(context))},
            {"p3.enable_global_reference_bias": _as_bool(lc("p3.enable_global_reference_bias").perform(context))},
            {"p3.local_bias_radius_m": float(lc("p3.local_bias_radius_m").perform(context))},
            {"p3.min_improvement_ratio": float(lc("p3.min_improvement_ratio").perform(context))},
            {"p3.w_risk": float(lc("p3.w_risk").perform(context))},
            {"p3.w_detour": float(lc("p3.w_detour").perform(context))},
            {"p3.w_unknown": float(lc("p3.w_unknown").perform(context))},
            {"p3.min_corridor_valid_ratio": float(lc("p3.min_corridor_valid_ratio").perform(context))},
            {"p3.station_spacing_m": float(lc("p3.station_spacing_m").perform(context))},
            {"p3.lateral_sample_step_m": float(lc("p3.lateral_sample_step_m").perform(context))},
            {"p3.lateral_sample_count_each_side": int(lc("p3.lateral_sample_count_each_side").perform(context))},
            {"p3.beam_width": int(lc("p3.beam_width").perform(context))},
            {"p3.max_detour_ratio": float(lc("p3.max_detour_ratio").perform(context))},
            {"p3.debug_csv_enable": _as_bool(lc("p3.debug_csv_enable").perform(context))},
            {"p3.debug_csv_path": lc("p3.debug_csv_path").perform(context)},
            {"p4.enable_risk_aware_astar": _as_bool(lc("p4.enable_risk_aware_astar").perform(context))},
            {"p4.lambda_p4_risk": float(lc("p4.lambda_p4_risk").perform(context))},
            {"p4.risk_cost_max": float(lc("p4.risk_cost_max").perform(context))},
            {"p4.unknown_edge_penalty": float(lc("p4.unknown_edge_penalty").perform(context))},
            {"p4.max_extra_path_ratio": float(lc("p4.max_extra_path_ratio").perform(context))},
            {"p4.fallback_to_original_when_risk_not_ready": _as_bool(lc("p4.fallback_to_original_when_risk_not_ready").perform(context))},
            {"p4.debug_csv_enable": _as_bool(lc("p4.debug_csv_enable").perform(context))},
            {"p4.debug_csv_path": lc("p4.debug_csv_path").perform(context)},
            {"p5.enable_runtime_gate": False},
            {"p5.enable_final_gate": False},
            {"p5.horizon_s": 2.0},
            {"p5.sample_dt_s": 0.2},
            {"p5.current_stale_to_replan_s": 0.5},
            {"p5.current_stale_to_emergency_s": 2.0},
            {"p5.future_unknown_to_emergency_s": 2.0},
            {"p5.current_replan_margin_m": 0.3},
            {"p5.current_emergency_margin_m": -0.2},
            {"p5.future_replan_margin_m": 0.3},
            {"p5.future_emergency_margin_m": -0.5},
            {"p5.max_bad_ratio": 0.25},
            {"p5.max_unknown_ratio": 0.30},
            {"p5.bad_tick_to_replan": 2},
            {"p5.good_tick_to_clear": 2},
            {"p5.integrity_topic": "/iap/integrity"},
            {"p5.status_topic": "planning/integrity_gate_status"},
            {"p5.debug_metrics_enable": False},
            {"risk_overlay/enable": False},
            {"risk_overlay/use_for_astar": False},
            {"risk_overlay/use_for_bspline": False},
            {"risk_overlay/topic": ""},
            {"manager/max_vel": 2.0},
            {"manager/max_acc": 3.0},
            {"manager/max_jerk": 4.0},
            {"manager/control_points_distance": 0.4},
            {"manager/feasibility_tolerance": 0.05},
            {"manager/planning_horizon": 7.5},
            {"manager/use_distinctive_trajs": True},
            {"manager/drone_id": int(drone_id)},
            {"manager/use_integrity_global_search": False},
            {"optimization/lambda_smooth": 1.0},
            {"optimization/lambda_collision": 0.5},
            {"optimization/lambda_feasibility": 0.1},
            {"optimization/lambda_fitness": 1.0},
            {"optimization/dist0": 0.5},
            {"optimization/swarm_clearance": 0.5},
            {"optimization/max_vel": 2.0},
            {"optimization/max_acc": 3.0},
            {"optimization/use_integrity_cost": False},
            {"optimization/use_integrity_front_search": False},
            {"optimization/use_integrity_global_search": False},
            {"bspline/limit_vel": 2.0},
            {"bspline/limit_acc": 3.0},
            {"bspline/limit_ratio": 1.1},
            {"prediction/obj_num": 0},
            {"prediction/lambda": 1.0},
            {"prediction/predict_rate": 1.0},
        ],
    )


def _launch_setup(context):
    iap_share = get_package_share_directory("iap")
    so3_control_share = get_package_share_directory("so3_control")
    local_sensing_share = get_package_share_directory("local_sensing")
    experiment = _apply_experiment_preset(context, iap_share)

    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    run_validator = _as_bool(LaunchConfiguration("run_validator").perform(context))
    start_planner = _as_bool(LaunchConfiguration("start_planner").perform(context))
    use_gnss = _as_bool(LaunchConfiguration("use_gnss").perform(context))
    use_araim = _as_bool(LaunchConfiguration("use_araim").perform(context))
    allow_truth_alignment = _as_bool(
        LaunchConfiguration("allow_truth_alignment").perform(context)
    )
    enable_preflight_takeoff = _as_bool(
        LaunchConfiguration("enable_preflight_takeoff").perform(context)
    )

    drone_id = LaunchConfiguration("drone_id").perform(context)
    init_x = float(LaunchConfiguration("init_x").perform(context))
    init_y = float(LaunchConfiguration("init_y").perform(context))
    init_z = float(LaunchConfiguration("init_z").perform(context))
    goal_x = LaunchConfiguration("goal_x").perform(context)
    goal_y = LaunchConfiguration("goal_y").perform(context)
    goal_z = LaunchConfiguration("goal_z").perform(context)
    point_num = LaunchConfiguration("point_num").perform(context)
    preflight_ground_z = float(LaunchConfiguration("preflight_ground_z").perform(context))
    plant_init_z = preflight_ground_z if enable_preflight_takeoff else init_z

    run_duration_s = float(LaunchConfiguration("run_duration_s").perform(context))
    validation_duration_s = float(
        LaunchConfiguration("validation_duration_s").perform(context)
    )
    map_size_x = float(LaunchConfiguration("map_size_x").perform(context))
    map_size_y = float(LaunchConfiguration("map_size_y").perform(context))
    map_size_z = float(LaunchConfiguration("map_size_z").perform(context))
    fixed_status_text = _as_bool(
        LaunchConfiguration("viz_status_text_use_fixed_position").perform(context)
    )
    status_text_position = (
        float(LaunchConfiguration("viz_status_text_x").perform(context)),
        float(LaunchConfiguration("viz_status_text_y").perform(context)),
        float(LaunchConfiguration("viz_status_text_z").perform(context)),
    )

    gnss_ephemeris_source = LaunchConfiguration("gnss_ephemeris_source").perform(context)
    gnss_time_source = LaunchConfiguration("gnss_time_source").perform(context)
    gnss_enabled_constellations = LaunchConfiguration(
        "gnss_enabled_constellations"
    ).perform(context)
    gnss_scenario_file = LaunchConfiguration("gnss_scenario_file").perform(context)
    gnss_rinex_nav_file = LaunchConfiguration("gnss_rinex_nav_file").perform(context)
    gnss_rinex_ephem_max_age_s = LaunchConfiguration(
        "gnss_rinex_ephem_max_age_s"
    ).perform(context)
    gnss_fallback_to_synthetic = _as_bool(
        LaunchConfiguration("gnss_fallback_to_synthetic_on_rinex_error").perform(
            context
        )
    )
    gnss_sky_dome_center_enu = [
        float(LaunchConfiguration("gnss_sky_dome_center_x").perform(context)),
        float(LaunchConfiguration("gnss_sky_dome_center_y").perform(context)),
        float(LaunchConfiguration("gnss_sky_dome_center_z").perform(context)),
    ]
    gnss_skyplot_origin_enu = [
        float(LaunchConfiguration("gnss_skyplot_origin_x").perform(context)),
        float(LaunchConfiguration("gnss_skyplot_origin_y").perform(context)),
        float(LaunchConfiguration("gnss_skyplot_origin_z").perform(context)),
    ]

    if (
        use_gnss
        and gnss_ephemeris_source.strip().lower() == "rinex"
        and not gnss_fallback_to_synthetic
        and not Path(gnss_rinex_nav_file).expanduser().is_file()
    ):
        raise RuntimeError(
            "gnss_ephemeris_source:=rinex requires an existing "
            "gnss_rinex_nav_file when "
            "gnss_fallback_to_synthetic_on_rinex_error:=false; "
            f"got '{gnss_rinex_nav_file}'"
        )

    runtime_config_path, runtime_root, export_dir = _runtime_config(
        context,
        use_gnss=use_gnss,
        use_araim=use_araim,
        allow_truth_alignment=allow_truth_alignment,
    )
    gnss_scenario_file = _materialize_gnss_scenario(gnss_scenario_file, export_dir)

    truth_odom_topic = "/sim/drone_0/truth_odom"
    iap_odom_topic = "/drone_0_visual_slam/odom"
    planner_odom_topic = truth_odom_topic
    sim_imu_topic = "/sim/drone_0/imu"
    iap_imu_topic = "/sim/drone_0/imu_iap"
    sim_lidar_topic = "/sim/drone_0/lidar"
    iap_lidar_topic = "/sim/drone_0/lidar_body"
    sim_depth_topic = "/sim/drone_0/depth"
    pos_cmd_topic = "/drone_0_planning/pos_cmd"
    desired_odom_topic = "/demo11_araim/desired/odom"
    bspline_topic = "/drone_0_planning/bspline"
    so3_cmd_topic = "/test_araim/so3_cmd"
    camera_pose_topic = "/drone_0_pcl_render_node/camera_pose"

    camera_file = os.path.join(local_sensing_share, "config", "camera.yaml")
    gains_file = os.path.join(so3_control_share, "config", "gains_hummingbird.yaml")
    corrections_file = os.path.join(
        so3_control_share, "config", "corrections_hummingbird.yaml"
    )

    preflight_delay = 0.0
    if enable_preflight_takeoff:
        preflight_delay = (
            float(LaunchConfiguration("preflight_ground_hold_s").perform(context))
            + float(LaunchConfiguration("preflight_takeoff_duration_s").perform(context))
            + float(LaunchConfiguration("preflight_hover_s").perform(context))
        )
    planner_start_delay_s = preflight_delay + max(
        0.0, float(LaunchConfiguration("planner_start_delay_s").perform(context))
    )

    planner_nodes = [
        _ego_planner_node(
            drone_id,
            planner_odom_topic,
            "/map_generator/global_cloud",
            camera_pose_topic,
            sim_depth_topic,
            bspline_topic,
            map_size_x,
            map_size_y,
            map_size_z,
            goal_x,
            goal_y,
            goal_z,
            point_num,
            context,
        ),
        Node(
            package="ego_planner",
            executable="traj_server",
            name=f"drone_{drone_id}_traj_server",
            output="screen",
            remappings=[
                ("planning/bspline", bspline_topic),
                ("position_cmd", pos_cmd_topic),
                ("/position_cmd", pos_cmd_topic),
            ],
            parameters=[{"traj_server/time_forward": 1.0}],
        ),
    ]
    if not start_planner:
        planner_actions = [
            LogInfo(msg="[test_araim] EGO planner/traj_server disabled by start_planner:=false")
        ]
    elif planner_start_delay_s > 0.0:
        planner_actions = [
            LogInfo(
                msg=(
                    f"[test_araim] delaying EGO planner/traj_server by "
                    f"{planner_start_delay_s:.2f}s"
                )
            ),
            TimerAction(period=planner_start_delay_s, actions=planner_nodes),
        ]
    else:
        planner_actions = planner_nodes

    bag_root_dir = LaunchConfiguration("bag_output_dir").perform(context).strip()
    if not bag_root_dir:
        bag_root_dir = str(Path(runtime_root) / "bag")
    bag_stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    bag_experiment = "".join(
        c if c.isalnum() or c in ("-", "_") else "_" for c in experiment
    ).strip("_") or "manual"
    bag_output_dir = str(
        Path(bag_root_dir) / f"test_araim_{bag_experiment}_{bag_stamp}"
    )
    if record_bag:
        os.makedirs(bag_root_dir, exist_ok=True)

    bag_record_topics = [
        "/iap/integrity",
        "/gnss_sim/diagnostics",
        "/gnss_sim/visualization/nlos_paths",
        "/gnss_sim/visualization/occlusion_points",
        "/gnss_sim/visualization/satellite_markers",
        "/gnss_sim/visualization/signal_rays",
        "/gnss_sim/visualization/sky_dome",
        "/gnss_sim/visualization/skyplot",
        "/gnss_sim/visualization/status_text",
        truth_odom_topic,
        iap_odom_topic,
        desired_odom_topic,
        sim_imu_topic,
        iap_imu_topic,
        sim_lidar_topic,
        iap_lidar_topic,
        sim_depth_topic,
        "/map_generator/global_cloud",
        "/map_generator/local_cloud",
        "/demo11/trunk_cloud",
        "/demo11/canopy_cloud",
        "/demo11/terminal_wall_cloud",
        camera_pose_topic,
        pos_cmd_topic,
        bspline_topic,
        so3_cmd_topic,
        "/covariance",
        "/covariance_velocity",
        "/height",
        "/trajectory",
        "/ublox_driver/range_meas",
        "/ublox_driver/ephem",
        "/ublox_driver/glo_ephem",
        "/ublox_driver/receiver_lla",
        "/ublox_driver/iono_params",
        "/tf",
    ]

    actions = [
        LogInfo(msg="[test_araim] single-owner ARAIM validation chain"),
        LogInfo(msg=f"[test_araim] experiment preset: {experiment}"),
        LogInfo(msg=f"[test_araim] runtime IAP config: {runtime_config_path}"),
        LogInfo(msg=f"[test_araim] export dir: {export_dir}"),
        LogInfo(msg=f"[test_araim] GNSS scenario: {gnss_scenario_file}"),
        LogInfo(msg=f"[test_araim] GNSS time_source: {gnss_time_source}"),
        LogInfo(msg=f"[test_araim] rosbag output: {bag_output_dir}"),
        Node(
            package="iap",
            executable="demo11_corridor_map_publisher",
            name="test_araim_corridor_map_publisher",
            output="screen",
            parameters=[
                {"resolution_m": float(LaunchConfiguration("corridor_map_resolution_m").perform(context))},
                {"publish_rate_hz": float(LaunchConfiguration("corridor_map_publish_rate_hz").perform(context))},
                {"frame_id": "map"},
                {"forest_size_x_m": float(LaunchConfiguration("forest_size_x_m").perform(context))},
                {"forest_size_y_m": float(LaunchConfiguration("forest_size_y_m").perform(context))},
                {"tree_density_lower_left_per_m2": float(LaunchConfiguration("tree_density_lower_left_per_m2").perform(context))},
                {"tree_density_lower_right_per_m2": float(LaunchConfiguration("tree_density_lower_right_per_m2").perform(context))},
                {"tree_density_upper_left_per_m2": float(LaunchConfiguration("tree_density_upper_left_per_m2").perform(context))},
                {"tree_density_upper_right_per_m2": float(LaunchConfiguration("tree_density_upper_right_per_m2").perform(context))},
                {"stratified_cell_size_m": float(LaunchConfiguration("stratified_cell_size_m").perform(context))},
                {"clear_corridor_enabled": _as_bool(LaunchConfiguration("clear_corridor_enabled").perform(context))},
                {"clear_corridor_center_y_m": float(LaunchConfiguration("clear_corridor_center_y_m").perform(context))},
                {"clear_corridor_half_width_y_m": float(LaunchConfiguration("clear_corridor_half_width_y_m").perform(context))},
                {"clear_corridor_x_min_m": float(LaunchConfiguration("clear_corridor_x_min_m").perform(context))},
                {"clear_corridor_x_max_m": float(LaunchConfiguration("clear_corridor_x_max_m").perform(context))},
                {"canopy_density_lower_left": float(LaunchConfiguration("canopy_density_lower_left").perform(context))},
                {"canopy_density_lower_right": float(LaunchConfiguration("canopy_density_lower_right").perform(context))},
                {"canopy_density_upper_left": float(LaunchConfiguration("canopy_density_upper_left").perform(context))},
                {"canopy_density_upper_right": float(LaunchConfiguration("canopy_density_upper_right").perform(context))},
                {"canopy_hemisphere_radius_min_m": float(LaunchConfiguration("canopy_hemisphere_radius_min_m").perform(context))},
                {"canopy_hemisphere_radius_max_m": float(LaunchConfiguration("canopy_hemisphere_radius_max_m").perform(context))},
                {"canopy_leaf_ball_radius_m": float(LaunchConfiguration("canopy_leaf_ball_radius_m").perform(context))},
                {"canopy_ball_spacing_ratio": float(LaunchConfiguration("canopy_ball_spacing_ratio").perform(context))},
                {"canopy_resolution_m": float(LaunchConfiguration("canopy_resolution_m").perform(context))},
                {"random_seed": int(LaunchConfiguration("forest_random_seed").perform(context))},
                {"trunk_radius_m": float(LaunchConfiguration("trunk_radius_m").perform(context))},
                {"trunk_min_height_m": float(LaunchConfiguration("trunk_min_height_m").perform(context))},
                {"trunk_max_height_m": float(LaunchConfiguration("trunk_max_height_m").perform(context))},
                {"terminal_wall_enabled": _as_bool(LaunchConfiguration("terminal_wall_enabled").perform(context))},
                {"terminal_wall_x_m": float(LaunchConfiguration("terminal_wall_x_m").perform(context))},
                {"terminal_wall_y_m": float(LaunchConfiguration("terminal_wall_y_m").perform(context))},
                {"terminal_wall_width_y_m": float(LaunchConfiguration("terminal_wall_width_y_m").perform(context))},
                {"terminal_wall_z_min_m": float(LaunchConfiguration("terminal_wall_z_min_m").perform(context))},
                {"terminal_wall_z_max_m": float(LaunchConfiguration("terminal_wall_z_max_m").perform(context))},
                {"terminal_wall_thickness_x_m": float(LaunchConfiguration("terminal_wall_thickness_x_m").perform(context))},
                {"terminal_wall_resolution_m": float(LaunchConfiguration("terminal_wall_resolution_m").perform(context))},
                {"terminal_wall_feature_depth_x_m": float(LaunchConfiguration("terminal_wall_feature_depth_x_m").perform(context))},
                {"terminal_wall_feature_count": int(LaunchConfiguration("terminal_wall_feature_count").perform(context))},
                {"terminal_wall_feature_seed": int(LaunchConfiguration("terminal_wall_feature_seed").perform(context))},
                {"corridor_walls_enabled": _as_bool(LaunchConfiguration("corridor_walls_enabled").perform(context))},
                {"corridor_floor_enabled": _as_bool(LaunchConfiguration("corridor_floor_enabled").perform(context))},
                {"corridor_x_min_m": float(LaunchConfiguration("corridor_x_min_m").perform(context))},
                {"corridor_x_max_m": float(LaunchConfiguration("corridor_x_max_m").perform(context))},
                {"corridor_half_width_y_m": float(LaunchConfiguration("corridor_half_width_y_m").perform(context))},
                {"corridor_wall_z_min_m": float(LaunchConfiguration("corridor_wall_z_min_m").perform(context))},
                {"corridor_wall_z_max_m": float(LaunchConfiguration("corridor_wall_z_max_m").perform(context))},
                {"corridor_wall_thickness_y_m": float(LaunchConfiguration("corridor_wall_thickness_y_m").perform(context))},
                {"corridor_floor_thickness_z_m": float(LaunchConfiguration("corridor_floor_thickness_z_m").perform(context))},
                {"corridor_surface_resolution_m": float(LaunchConfiguration("corridor_surface_resolution_m").perform(context))},
            ],
        ),
        Node(
            package="local_sensing",
            executable="pcl_render_node",
            name="drone_0_pcl_render_node",
            output="screen",
            remappings=[
                ("global_map", "/map_generator/global_cloud"),
                ("local_map", "/map_generator/local_cloud"),
                ("odometry", truth_odom_topic),
                ("pcl_render_node/cloud", sim_lidar_topic),
                ("depth", sim_depth_topic),
                ("camera_pose", camera_pose_topic),
            ],
            parameters=[
                {"sensing_horizon": 10.0},
                {"sensing_rate": 15.0},
                {"estimation_rate": 15.0},
                {"map/x_size": map_size_x},
                {"map/y_size": map_size_y},
                {"map/z_size": map_size_z},
                {"map/resolution": 0.1},
                camera_file,
            ],
        ),
        Node(
            package="iap",
            executable="demo4_lidar_body_bridge",
            name="test_araim_lidar_body_bridge",
            output="screen",
            parameters=[
                {"input_cloud_topic": sim_lidar_topic},
                {"input_odom_topic": truth_odom_topic},
                {"output_cloud_topic": iap_lidar_topic},
                {"output_frame_id": "lidar"},
                {"max_odom_lookup_dt": 0.05},
            ],
        ),
        Node(
            package="iap",
            executable="iap_rosnode",
            name="test_araim_iap_rosnode",
            output="screen",
            parameters=[
                {"config_path": runtime_config_path},
                {"imu_topic": iap_imu_topic},
                {"points_topic": iap_lidar_topic},
            ],
        ),
        Node(
            package="iap",
            executable="demo_takeoff_cmd_publisher",
            name="test_araim_preflight_takeoff_cmd_publisher",
            output="screen",
            condition=IfCondition("true" if enable_preflight_takeoff else "false"),
            remappings=[("position_cmd", pos_cmd_topic)],
            parameters=[
                {"ground_x": init_x},
                {"ground_y": init_y},
                {"ground_z": preflight_ground_z},
                {"hover_x": init_x},
                {"hover_y": init_y},
                {"hover_z": init_z},
                {"ground_hold_duration_s": float(LaunchConfiguration("preflight_ground_hold_s").perform(context))},
                {"takeoff_duration_s": float(LaunchConfiguration("preflight_takeoff_duration_s").perform(context))},
                {"hover_duration_s": float(LaunchConfiguration("preflight_hover_s").perform(context))},
                {"publish_rate_hz": float(LaunchConfiguration("preflight_cmd_rate_hz").perform(context))},
                {"trajectory_id": 11001},
                {"frame_id": "map"},
                {"stop_after_sequence": True},
            ],
        ),
        Node(
            package="poscmd_2_odom",
            executable="poscmd_2_odom",
            name="test_araim_desired_poscmd_to_odom",
            output="screen",
            remappings=[
                ("command", pos_cmd_topic),
                ("odometry", desired_odom_topic),
            ],
            parameters=[
                {"init_x": init_x},
                {"init_y": init_y},
                {"init_z": plant_init_z},
            ],
        ),
        _odom_visualization_node(
            "test_araim_iap_odom_visualization",
            iap_odom_topic,
            pos_cmd_topic,
            "/demo9/drone",
            (0.2, 1.0, 0.4),
            int(drone_id),
            fixed_status_text,
            status_text_position,
        ),
        _odom_visualization_node(
            "test_araim_truth_odom_visualization",
            truth_odom_topic,
            pos_cmd_topic,
            "/demo9/truth",
            (1.0, 0.15, 0.1),
            10,
        ),
        _odom_visualization_node(
            "test_araim_desired_odom_visualization",
            desired_odom_topic,
            pos_cmd_topic,
            "/demo9/desired",
            (1.0, 0.86, 0.05),
            20,
        ),
        Node(
            package="gnss_sim",
            executable="gnss_sim_node",
            name="test_araim_gnss_sim_node",
            output="screen",
            condition=IfCondition("true" if use_gnss else "false"),
            parameters=[
                {"truth_odom_topic": truth_odom_topic},
                {"origin_lat_deg": 31.2304},
                {"origin_lon_deg": 121.4737},
                {"origin_alt_m": 25.0},
                {"pseudorange_noise_std_m": float(LaunchConfiguration("gnss_pr_noise_base").perform(context))},
                {"doppler_noise_std_mps": float(LaunchConfiguration("gnss_dop_noise_base").perform(context))},
                {"time_source": gnss_time_source},
                {"trigger_topic": sim_lidar_topic},
                {"scenario_file": gnss_scenario_file},
                {"num_gps_sats": 24},
                {"gps_prn_min": 1},
                {"gps_prn_max": 24},
                {"ephemeris_source": gnss_ephemeris_source},
                {"rinex_nav_file": gnss_rinex_nav_file},
                {"rinex_ephem_max_age_s": float(gnss_rinex_ephem_max_age_s)},
                {"rinex_gps_only": False},
                {"fallback_to_synthetic_on_rinex_error": gnss_fallback_to_synthetic},
                {"enabled_constellations_csv": gnss_enabled_constellations},
                {"enable_visualization": _as_bool(LaunchConfiguration("gnss_enable_visualization").perform(context))},
                {"signal_ray_width_m": 0.025},
                {"signal_ray_alpha": 0.3},
                {"nlos_path_width_m": 0.04},
                {"nlos_path_alpha": 0.3},
                {"enable_sky_dome_visualization": _as_bool(LaunchConfiguration("gnss_enable_sky_dome_visualization").perform(context))},
                {"sky_dome_show_cardinal_labels": _as_bool(LaunchConfiguration("gnss_sky_dome_show_cardinal_labels").perform(context))},
                {"sky_dome_ring_count": 3},
                {"sky_dome_meridian_count": 12},
                {"sky_dome_follow_receiver": _as_bool(LaunchConfiguration("gnss_sky_dome_follow_receiver").perform(context))},
                {"sky_dome_center_enu": gnss_sky_dome_center_enu},
                {"skyplot_origin_enu": gnss_skyplot_origin_enu},
                {"status_text_use_fixed_position": fixed_status_text},
                {"status_text_position_enu": list(status_text_position)},
                {"enable_map_occlusion": _as_bool(LaunchConfiguration("gnss_enable_map_occlusion").perform(context))},
                {"enable_skymask": _as_bool(LaunchConfiguration("gnss_enable_skymask").perform(context))},
                {"enable_nlos": _as_bool(LaunchConfiguration("gnss_enable_nlos").perform(context))},
                {"enable_multipath": _as_bool(LaunchConfiguration("gnss_enable_multipath").perform(context))},
                {"enable_fault_injection": _as_bool(LaunchConfiguration("gnss_enable_fault_injection").perform(context))},
                {"map_cloud_topic": "/map_generator/global_cloud"},
                {"multipath_amp_m": 0.5},
            ],
        ),
        Node(
            package="so3_quadrotor_simulator",
            executable="so3_quadrotor_simulator",
            name=f"drone_{drone_id}_quadrotor_simulator_so3",
            output="screen",
            remappings=[
                ("odom", truth_odom_topic),
                ("imu", sim_imu_topic),
                ("cmd", so3_cmd_topic),
                ("force_disturbance", "/test_araim/force_disturbance"),
                ("moment_disturbance", "/test_araim/moment_disturbance"),
            ],
            parameters=[
                {"quadrotor_name": f"drone_{drone_id}"},
                {"rate/simulation": 1000.0},
                {"rate/odom": 100.0},
                {"simulator/init_state_x": init_x},
                {"simulator/init_state_y": init_y},
                {"simulator/init_state_z": plant_init_z},
                {"simulator/hold_until_cmd": True},
                {"sim_time/enable": True},
                {"sim_time/start_utc": "2022-07-06T00:00:00Z"},
                {"iap_imu/enable": True},
                {"iap_imu/topic": iap_imu_topic},
            ],
        ),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="test_araim_so3_control_container",
            namespace="",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="so3_control",
                    plugin="SO3ControlComponent",
                    name="test_araim_so3_control_component",
                    parameters=[
                        {"quadrotor_name": f"drone_{drone_id}"},
                        {"so3_control/init_state_x": init_x},
                        {"so3_control/init_state_y": init_y},
                        {"so3_control/init_state_z": plant_init_z},
                        {"mass": 0.98},
                        {"use_angle_corrections": False},
                        {"use_external_yaw": False},
                        {"gains/rot/z": 1.0},
                        {"gains/ang/z": 0.1},
                        gains_file,
                        corrections_file,
                    ],
                    remappings=[
                        ("odom", truth_odom_topic),
                        ("position_cmd", pos_cmd_topic),
                        ("motors", "/test_araim/motors"),
                        ("corrections", "/test_araim/corrections"),
                        ("so3_cmd", so3_cmd_topic),
                        ("imu", iap_imu_topic),
                    ],
                )
            ],
        ),
        *planner_actions,
        Node(
            package="iap",
            executable="test_araim_validator.py",
            name="test_araim_validator",
            output="screen",
            condition=IfCondition("true" if run_validator else "false"),
            parameters=[
                {"integrity_topic": "/iap/integrity"},
                {"duration_s": validation_duration_s},
                {"min_messages": 10},
                {"csv_path": str(Path(export_dir) / "test_araim_integrity_validation.csv")},
                {"summary_path": str(Path(export_dir) / "test_araim_validation_summary.json")},
                {"required_fusion_mode": LaunchConfiguration("integrity_fusion_mode").perform(context)},
                {"require_gnss_valid": _as_bool(LaunchConfiguration("validator_require_gnss_valid").perform(context))},
                {"require_lidar_valid": _as_bool(LaunchConfiguration("validator_require_lidar_valid").perform(context))},
                {"require_fallback_valid": _as_bool(LaunchConfiguration("validator_require_fallback_valid").perform(context))},
                {"required_final_source": LaunchConfiguration("validator_required_final_source").perform(context)},
                {"allowed_final_sources_csv": LaunchConfiguration("validator_allowed_final_sources").perform(context)},
            ],
        ),
    ]

    if record_bag:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    bag_output_dir,
                    *bag_record_topics,
                ],
                output="screen",
            )
        )

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="test_araim_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        iap_share,
                        "config",
                        "sim_demo11",
                        "demo11_integrity_corridor.rviz",
                    ),
                ],
            )
        )

    if run_duration_s > 0.0:
        actions.append(
            TimerAction(
                period=run_duration_s,
                actions=[EmitEvent(event=Shutdown(reason="test_araim run_duration_s elapsed"))],
            )
        )

    return actions


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    fastdds_profile = os.path.join(iap_share, "config", "sim_ego", "fastdds_udp_only.xml")
    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument("run_validator", default_value="true"),
        DeclareLaunchArgument("start_planner", default_value="true"),
        DeclareLaunchArgument("bag_output_dir", default_value="/home/dev/ws_iap/src/iap/results/araim_validation/real_time_test"),
        DeclareLaunchArgument("experiment", default_value="manual"),
        DeclareLaunchArgument("config_subdir", default_value="sim_demo11"),
        DeclareLaunchArgument("use_gnss", default_value="true"),
        DeclareLaunchArgument("use_araim", default_value="true"),
        DeclareLaunchArgument("enable_gnss_integrity", default_value="true"),
        DeclareLaunchArgument("enable_gnss_araim", default_value="true"),
        DeclareLaunchArgument("enable_lidar_integrity", default_value="true"),
        DeclareLaunchArgument("enable_araim_pl_decomp_csv", default_value="false"),
        DeclareLaunchArgument("integrity_fusion_mode", default_value="max_pl"),
        DeclareLaunchArgument("integrity_require_valid_gnss", default_value="false"),
        DeclareLaunchArgument("integrity_require_valid_lidar", default_value="false"),
        DeclareLaunchArgument("integrity_conservative_hpl_m", default_value="999.0"),
        DeclareLaunchArgument("integrity_conservative_vpl_m", default_value="999.0"),
        DeclareLaunchArgument("validator_require_gnss_valid", default_value="true"),
        DeclareLaunchArgument("validator_require_lidar_valid", default_value="true"),
        DeclareLaunchArgument("validator_require_fallback_valid", default_value="true"),
        DeclareLaunchArgument("validator_required_final_source", default_value=""),
        DeclareLaunchArgument("validator_allowed_final_sources", default_value="GNSS,LIDAR,FALLBACK,CONSERVATIVE"),
        DeclareLaunchArgument("init_x", default_value="-12.0"),
        DeclareLaunchArgument("init_y", default_value="0.0"),
        DeclareLaunchArgument("init_z", default_value="1.2"),
        DeclareLaunchArgument("goal_x", default_value="12.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
        DeclareLaunchArgument("goal_z", default_value="1.2"),
        DeclareLaunchArgument("point_num", default_value="1"),
        DeclareLaunchArgument("point1_x", default_value="9.5"),
        DeclareLaunchArgument("point1_y", default_value="0.0"),
        DeclareLaunchArgument("point1_z", default_value="1.2"),
        DeclareLaunchArgument("point2_x", default_value="16.0"),
        DeclareLaunchArgument("point2_y", default_value="0.0"),
        DeclareLaunchArgument("point2_z", default_value="1.2"),
        DeclareLaunchArgument("point3_x", default_value="16.0"),
        DeclareLaunchArgument("point3_y", default_value="0.0"),
        DeclareLaunchArgument("point3_z", default_value="1.2"),
        DeclareLaunchArgument("point4_x", default_value="16.0"),
        DeclareLaunchArgument("point4_y", default_value="0.0"),
        DeclareLaunchArgument("point4_z", default_value="1.2"),
        DeclareLaunchArgument("point5_x", default_value="16.0"),
        DeclareLaunchArgument("point5_y", default_value="0.0"),
        DeclareLaunchArgument("point5_z", default_value="1.2"),
        DeclareLaunchArgument("point6_x", default_value="16.0"),
        DeclareLaunchArgument("point6_y", default_value="0.0"),
        DeclareLaunchArgument("point6_z", default_value="1.2"),
        DeclareLaunchArgument("run_duration_s", default_value="90"),
        DeclareLaunchArgument("validation_duration_s", default_value="85"),
        DeclareLaunchArgument("allow_truth_alignment", default_value="true"),
        DeclareLaunchArgument("planner_start_delay_s", default_value="0.0"),
        DeclareLaunchArgument("p0.enable_risk_grid", default_value="false"),
        DeclareLaunchArgument("p0.debug_metrics_enable", default_value="false"),
        DeclareLaunchArgument("p1.use_integrity_cost", default_value="false"),
        DeclareLaunchArgument("p1.metrics_only", default_value="true"),
        DeclareLaunchArgument("p1.lambda_integrity", default_value="0.0"),
        DeclareLaunchArgument("p1.sample_dt_min_s", default_value="0.1"),
        DeclareLaunchArgument("p1.sample_dt_scale", default_value="1.0"),
        DeclareLaunchArgument("p1.max_samples_per_eval", default_value="30"),
        DeclareLaunchArgument("p1.integrity_cost_max", default_value="100.0"),
        DeclareLaunchArgument("p1.integrity_grad_norm_max", default_value="0.1"),
        DeclareLaunchArgument("p1.unknown_policy", default_value="skip"),
        DeclareLaunchArgument("p1.unknown_soft_penalty", default_value="1.0"),
        DeclareLaunchArgument("p1.debug_csv_enable", default_value="false"),
        DeclareLaunchArgument("p1.debug_csv_path", default_value=""),
        DeclareLaunchArgument("p2.enable_candidate_ranking", default_value="false"),
        DeclareLaunchArgument("p2.metrics_only", default_value="true"),
        DeclareLaunchArgument("p2.sample_dt_s", default_value="0.2"),
        DeclareLaunchArgument("p2.lambda_candidate_integrity", default_value="1.0"),
        DeclareLaunchArgument("p2.w_max_cost", default_value="0.25"),
        DeclareLaunchArgument("p2.w_unknown", default_value="5.0"),
        DeclareLaunchArgument("p2.w_stale", default_value="2.0"),
        DeclareLaunchArgument("p2.min_valid_ratio", default_value="0.3"),
        DeclareLaunchArgument("p2.debug_csv_enable", default_value="false"),
        DeclareLaunchArgument("p2.debug_csv_path", default_value=""),
        DeclareLaunchArgument("p3.enable_local_reference_bias", default_value="false"),
        DeclareLaunchArgument("p3.enable_global_reference_bias", default_value="false"),
        DeclareLaunchArgument("p3.local_bias_radius_m", default_value="1.5"),
        DeclareLaunchArgument("p3.min_improvement_ratio", default_value="0.05"),
        DeclareLaunchArgument("p3.w_risk", default_value="1.0"),
        DeclareLaunchArgument("p3.w_detour", default_value="0.25"),
        DeclareLaunchArgument("p3.w_unknown", default_value="5.0"),
        DeclareLaunchArgument("p3.min_corridor_valid_ratio", default_value="0.8"),
        DeclareLaunchArgument("p3.station_spacing_m", default_value="2.0"),
        DeclareLaunchArgument("p3.lateral_sample_step_m", default_value="1.0"),
        DeclareLaunchArgument("p3.lateral_sample_count_each_side", default_value="3"),
        DeclareLaunchArgument("p3.beam_width", default_value="5"),
        DeclareLaunchArgument("p3.max_detour_ratio", default_value="1.5"),
        DeclareLaunchArgument("p3.debug_csv_enable", default_value="false"),
        DeclareLaunchArgument("p3.debug_csv_path", default_value=""),
        DeclareLaunchArgument("p4.enable_risk_aware_astar", default_value="false"),
        DeclareLaunchArgument("p4.lambda_p4_risk", default_value="0.05"),
        DeclareLaunchArgument("p4.risk_cost_max", default_value="100.0"),
        DeclareLaunchArgument("p4.unknown_edge_penalty", default_value="1.0"),
        DeclareLaunchArgument("p4.max_extra_path_ratio", default_value="1.3"),
        DeclareLaunchArgument("p4.fallback_to_original_when_risk_not_ready", default_value="true"),
        DeclareLaunchArgument("p4.debug_csv_enable", default_value="false"),
        DeclareLaunchArgument("p4.debug_csv_path", default_value=""),
        DeclareLaunchArgument("enable_preflight_takeoff", default_value="true"),
        DeclareLaunchArgument("preflight_ground_z", default_value="0.0"),
        DeclareLaunchArgument("preflight_ground_hold_s", default_value="10.0"),
        DeclareLaunchArgument("preflight_takeoff_duration_s", default_value="5.0"),
        DeclareLaunchArgument("preflight_hover_s", default_value="30.0"),
        DeclareLaunchArgument("preflight_cmd_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("gnss_pr_noise_base", default_value="5.0"),
        DeclareLaunchArgument("gnss_dop_noise_base", default_value="0.5"),
        DeclareLaunchArgument("gnss_ephemeris_source", default_value="rinex"),
        DeclareLaunchArgument("gnss_time_source", default_value="trigger_topic"),
        DeclareLaunchArgument("gnss_enabled_constellations", default_value="GPS,BDS,GAL,GLO"),
        DeclareLaunchArgument(
            "gnss_scenario_file",
            default_value=os.path.join(iap_share, "config", "gnss_sim", "demo7_skymask_nlos.yaml"),
        ),
        DeclareLaunchArgument(
            "gnss_rinex_nav_file",
            default_value="/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx",
        ),
        DeclareLaunchArgument("gnss_rinex_ephem_max_age_s", default_value="7200.0"),
        DeclareLaunchArgument("gnss_fallback_to_synthetic_on_rinex_error", default_value="false"),
        DeclareLaunchArgument("gnss_enable_map_occlusion", default_value="true"),
        DeclareLaunchArgument("gnss_enable_skymask", default_value="true"),
        DeclareLaunchArgument("gnss_enable_nlos", default_value="true"),
        DeclareLaunchArgument("gnss_enable_multipath", default_value="true"),
        DeclareLaunchArgument("gnss_enable_fault_injection", default_value="true"),
        DeclareLaunchArgument("gnss_enable_visualization", default_value="true"),
        DeclareLaunchArgument("gnss_enable_sky_dome_visualization", default_value="true"),
        DeclareLaunchArgument("gnss_sky_dome_show_cardinal_labels", default_value="true"),
        DeclareLaunchArgument("gnss_sky_dome_follow_receiver", default_value="false"),
        DeclareLaunchArgument("gnss_sky_dome_center_x", default_value="0.0"),
        DeclareLaunchArgument("gnss_sky_dome_center_y", default_value="0.0"),
        DeclareLaunchArgument("gnss_sky_dome_center_z", default_value="0.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_x", default_value="0.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_y", default_value="22.0"),
        DeclareLaunchArgument("gnss_skyplot_origin_z", default_value="5.0"),
        DeclareLaunchArgument("viz_status_text_use_fixed_position", default_value="true"),
        DeclareLaunchArgument("viz_status_text_x", default_value="15.0"),
        DeclareLaunchArgument("viz_status_text_y", default_value="0.0"),
        DeclareLaunchArgument("viz_status_text_z", default_value="5.0"),
        DeclareLaunchArgument("drone_id", default_value="0"),
        DeclareLaunchArgument("map_size_x", default_value="30.0"),
        DeclareLaunchArgument("map_size_y", default_value="30.0"),
        DeclareLaunchArgument("map_size_z", default_value="3.5"),
        DeclareLaunchArgument("corridor_map_resolution_m", default_value="0.1"),
        DeclareLaunchArgument("corridor_map_publish_rate_hz", default_value="2.0"),
        DeclareLaunchArgument("forest_size_x_m", default_value="20.0"),
        DeclareLaunchArgument("forest_size_y_m", default_value="20.0"),
        DeclareLaunchArgument("tree_density_lower_left_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_lower_right_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_upper_left_per_m2", default_value="0.5"),
        DeclareLaunchArgument("tree_density_upper_right_per_m2", default_value="0.5"),
        DeclareLaunchArgument("stratified_cell_size_m", default_value="1.0"),
        DeclareLaunchArgument("clear_corridor_enabled", default_value="false"),
        DeclareLaunchArgument("clear_corridor_center_y_m", default_value="0.0"),
        DeclareLaunchArgument("clear_corridor_half_width_y_m", default_value="0.0"),
        DeclareLaunchArgument("clear_corridor_x_min_m", default_value="-1000000000.0"),
        DeclareLaunchArgument("clear_corridor_x_max_m", default_value="1000000000.0"),
        DeclareLaunchArgument("canopy_density_lower_left", default_value="0.1"),
        DeclareLaunchArgument("canopy_density_lower_right", default_value="0.1"),
        DeclareLaunchArgument("canopy_density_upper_left", default_value="0.6"),
        DeclareLaunchArgument("canopy_density_upper_right", default_value="0.6"),
        DeclareLaunchArgument("canopy_hemisphere_radius_min_m", default_value="0.5"),
        DeclareLaunchArgument("canopy_hemisphere_radius_max_m", default_value="1.5"),
        DeclareLaunchArgument("canopy_leaf_ball_radius_m", default_value="0.22"),
        DeclareLaunchArgument("canopy_ball_spacing_ratio", default_value="1.2"),
        DeclareLaunchArgument("canopy_resolution_m", default_value="0.15"),
        DeclareLaunchArgument("forest_random_seed", default_value="11"),
        DeclareLaunchArgument("trunk_radius_m", default_value="0.14"),
        DeclareLaunchArgument("trunk_min_height_m", default_value="1.5"),
        DeclareLaunchArgument("trunk_max_height_m", default_value="3.0"),
        DeclareLaunchArgument("terminal_wall_enabled", default_value="true"),
        DeclareLaunchArgument("terminal_wall_x_m", default_value="13.5"),
        DeclareLaunchArgument("terminal_wall_y_m", default_value="0.0"),
        DeclareLaunchArgument("terminal_wall_width_y_m", default_value="10.0"),
        DeclareLaunchArgument("terminal_wall_z_min_m", default_value="0.0"),
        DeclareLaunchArgument("terminal_wall_z_max_m", default_value="3.2"),
        DeclareLaunchArgument("terminal_wall_thickness_x_m", default_value="0.20"),
        DeclareLaunchArgument("terminal_wall_resolution_m", default_value="0.10"),
        DeclareLaunchArgument("terminal_wall_feature_depth_x_m", default_value="0.65"),
        DeclareLaunchArgument("terminal_wall_feature_count", default_value="48"),
        DeclareLaunchArgument("terminal_wall_feature_seed", default_value="11022"),
        DeclareLaunchArgument("corridor_walls_enabled", default_value="false"),
        DeclareLaunchArgument("corridor_floor_enabled", default_value="false"),
        DeclareLaunchArgument("corridor_x_min_m", default_value="-14.0"),
        DeclareLaunchArgument("corridor_x_max_m", default_value="14.0"),
        DeclareLaunchArgument("corridor_half_width_y_m", default_value="2.0"),
        DeclareLaunchArgument("corridor_wall_z_min_m", default_value="0.0"),
        DeclareLaunchArgument("corridor_wall_z_max_m", default_value="3.0"),
        DeclareLaunchArgument("corridor_wall_thickness_y_m", default_value="0.10"),
        DeclareLaunchArgument("corridor_floor_thickness_z_m", default_value="0.05"),
        DeclareLaunchArgument("corridor_surface_resolution_m", default_value="0.10"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        OpaqueFunction(function=_launch_setup),
    ])
