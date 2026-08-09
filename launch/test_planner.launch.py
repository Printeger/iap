import json
import hashlib
import os
import shutil
import subprocess
import sys
import sys
import time
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


P1_EVIDENCE_SCHEMA_VERSION = "p1_evidence_provenance_v4"


def _sha256_file(path):
    path = Path(path).resolve()
    if not path.is_file():
        return ""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _formal_calibration_provenance(value):
    """Read immutable experiment provenance; never forward it to the planner."""
    raw = str(value).strip()
    if not raw:
        return {}
    path = Path(raw).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"P1 formal calibration file is missing: {path}")
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"P1 formal calibration is unreadable: {path}: {exc}") from exc
    if (
        not isinstance(payload, dict)
        or payload.get("schema_version") != "p1_formal_tolerance_calibration_v1"
        or not str(payload.get("calibration_id", "")).strip()
        or not isinstance(payload.get("generated_at_epoch_s"), (int, float))
    ):
        raise RuntimeError(f"P1 formal calibration has an invalid contract: {path}")
    return {
        "calibration_id": payload["calibration_id"],
        "path": str(path),
        "sha256": _sha256_file(path),
        "generated_at_epoch_s": float(payload["generated_at_epoch_s"]),
        "generated_at_utc": str(payload.get("generated_at_utc", "")),
    }


def _command_text(command, cwd):
    try:
        return subprocess.check_output(command, cwd=str(cwd), text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return ""


def _git_repository(iap_prefix):
    candidates = [
        Path(__file__).resolve().parents[1],
        Path(iap_prefix).resolve().parents[1] / "src" / "iap",
    ]
    for candidate in candidates:
        if (candidate / ".git").exists():
            return candidate
    return None


def _git_provenance(repo):
    if repo is None:
        return "", False
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=str(repo), text=True
        ).strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"], cwd=str(repo), text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=True,
        ).stdout
        return commit, not bool(status.strip())
    except (OSError, subprocess.CalledProcessError):
        return "", False


def _runtime_provenance(iap_share, export_dir, bag_output_dir, experiment, scenario):
    iap_prefix = Path(get_package_prefix("iap")).resolve()
    repo = _git_repository(iap_prefix)
    git_commit, git_worktree_clean = _git_provenance(repo)
    ego_prefix = Path(get_package_prefix("ego_planner")).resolve()
    bspline_prefix = Path(get_package_prefix("bspline_opt")).resolve()
    workspace_install = iap_prefix.parent
    planner_executable = (ego_prefix / "lib" / "ego_planner" / "ego_planner_node").resolve()
    bspline_library = (bspline_prefix / "lib" / "libbspline_opt.a").resolve()
    launch_path = Path(__file__).resolve()
    return {
        "schema_version": P1_EVIDENCE_SCHEMA_VERSION,
        "run_id": uuid.uuid4().hex,
        "git_commit": git_commit,
        "baseline_commit": "34fa22f17c3778c2f98a777e01516b878c183120",
        "git_worktree_clean": git_worktree_clean,
        "source_repository": str(repo) if repo else "",
        "workspace_root": str(iap_prefix.parents[1]),
        "install_prefix": str(workspace_install),
        "export_dir": str(Path(export_dir).resolve()),
        "bag_path": str(Path(bag_output_dir).resolve()),
        "experiment": experiment,
        "scenario": scenario,
        "process_start_stamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "process_start_epoch_s": time.time(),
        "runtime_paths": {
            "iap_prefix": str(iap_prefix),
            "ego_planner_prefix": str(ego_prefix),
            "bspline_opt_prefix": str(bspline_prefix),
            "launch": {"path": str(launch_path), "sha256": _sha256_file(launch_path)},
            "planner_executable": {"path": str(planner_executable), "sha256": _sha256_file(planner_executable)},
            "bspline_library": {"path": str(bspline_library), "sha256": _sha256_file(bspline_library)},
        },
    }


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


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


def _csv_floats(value):
    return [float(v.strip()) for v in str(value).split(",") if v.strip()]


def _safe_path_component(value, fallback):
    safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in str(value)).strip("_")
    return safe or fallback


def _scenario_fingerprint(scenario, expanded_contract):
    """Return a stable identity for the fully expanded scenario contract."""
    canonical = json.dumps(
        {"scenario": str(scenario), "contract": expanded_contract},
        sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(canonical).hexdigest()


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
    "corridor_floor_enabled": "true",
    "corridor_x_min_m": "-14.0",
    "corridor_x_max_m": "14.0",
    "corridor_half_width_y_m": "2.0",
    "corridor_floor_thickness_z_m": "0.05",
    "corridor_surface_resolution_m": "0.10",
}


P5_5_FIXTURE_ROUTE_WAYPOINTS = [
    (12.0, 0.0, 1.2),
    (-12.0, 0.0, 1.2),
    (12.0, 0.0, 1.2),
    (-12.0, 0.0, 1.2),
]


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


P1_FORK_MAP_PRESET = {
    **DEFAULT_ROUTE_PRESET,
    "init_z": "1.5",
    "goal_z": "1.5",
    "forest_size_x_m": "28.0",
    "forest_size_y_m": "10.0",
    "tree_density_lower_left_per_m2": "0.0",
    "tree_density_lower_right_per_m2": "0.0",
    "tree_density_upper_left_per_m2": "0.0",
    "tree_density_upper_right_per_m2": "0.0",
    "canopy_density_lower_left": "0.0",
    "canopy_density_lower_right": "0.0",
    "canopy_density_upper_left": "0.0",
    "canopy_density_upper_right": "0.0",
    "forest_random_seed": "41021",
    "terminal_wall_enabled": "false",
    "corridor_floor_enabled": "true",
    "corridor_x_min_m": "-14.0",
    "corridor_x_max_m": "14.0",
    "corridor_half_width_y_m": "5.0",
    "p1_fixture_mirror_y": "false",
    "p1_fixture_central_obstacle_enabled": "true",
    "p1_fixture_central_x_min_m": "-7.0",
    "p1_fixture_central_x_max_m": "-2.0",
    "p1_fixture_central_y_half_width_m": "0.65",
    "grid_map/local_update_range_x": "10.0",
    "p1_fixture_central_z_max_m": "2.8",
    "p1_fixture_lane_center_m": "2.0",
    "p1_fixture_lane_half_width_m": "0.75",
    "p1_fixture_safe_tree_density_per_m2": "0.25",
    "p1_fixture_risky_tree_density_per_m2": "0.75",
    "p1_fixture_safe_canopy_probability": "0.05",
    "p1_fixture_risky_canopy_probability": "0.85",
}


P1_FUSED_SENSOR_PRESET = {
    **GNSS_DEGRADED_PRESET,
    "use_gnss": "true", "use_araim": "true",
    "gnss_time_source": "odom_stamp",
    "enable_gnss_integrity": "true", "enable_gnss_araim": "true",
    "enable_lidar_integrity": "true", "integrity_fusion_mode": "max_pl",
    "validator_require_gnss_valid": "true",
    "validator_require_lidar_valid": "true",
    "validator_required_final_source": "",
}


P0_6_OCCUPIED_OVERLAP_FIXTURE_PRESET = {
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
    "validator_required_final_source": "GNSS",
    "p0.skip_occupied_voxels": "true",
    "p0_6.fixture.enabled": "true",
    "p0_6.fixture.name": "occupied_overlap_box_v1",
    "p0_6.fixture.x_min": "-1.5",
    "p0_6.fixture.x_max": "1.5",
    "p0_6.fixture.y_min": "-0.75",
    "p0_6.fixture.y_max": "0.75",
    "p0_6.fixture.z_min": "1.0",
    "p0_6.fixture.z_max": "2.0",
    "p0_6.fixture.raw_hpl_m": "1.0",
    "p0_6.fixture.raw_vpl_m": "1.2",
    "p0_6.fixture.raw_c_pi": "1.2",
    "p0_6.fixture.low_raw_cost_threshold": "2.0",
}


COMBO_PRESETS = {
    ("p0_open_sky", "manual"): P0_6_OCCUPIED_OVERLAP_FIXTURE_PRESET,
    ("p5_corridor", "manual"): {
        **DEFAULT_ROUTE_PRESET,
        **CORRIDOR_DEGENERATE_MAP_PRESET,
        **GNSS_OPEN_SKY_PRESET,
        "planner_start_delay_s": "10.0",
        "use_gnss": "true",
        "use_araim": "true",
        "gnss_time_source": "odom_stamp",
        "enable_gnss_integrity": "true",
        "enable_gnss_araim": "true",
        "enable_lidar_integrity": "false",
        "integrity_fusion_mode": "gnss_only",
        "validator_require_gnss_valid": "true",
        "validator_require_lidar_valid": "false",
        "validator_required_final_source": "GNSS",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p0.predictor.source_mode": "fusion",
        "p0.predictor.gnss_epoch_policy": "auto",
        "p0.predictor.use_current_integrity_prior": "true",
        "p0.predictor.conservative_max_with_gnss": "false",
        "p5.debug_metrics_enable": "true",
        "p5.pred_alert_limit_mode": "current_msg_constant",
        "p5_7.fixture.enabled": "true",
    },
}


SCENARIO_PRESETS = {
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
        "validator_required_final_source": "",
    },
    "p1_fork_fused_v1": {
        **P1_FORK_MAP_PRESET, **P1_FUSED_SENSOR_PRESET,
        "p1_map_fixture": "p1_fork_fused_v1",
    },
    "p1_fork_fused_mirror_v1": {
        **P1_FORK_MAP_PRESET, **P1_FUSED_SENSOR_PRESET,
        "p1_map_fixture": "p1_fork_fused_mirror_v1",
        "p1_fixture_mirror_y": "true",
    },
    "p1_fork_symmetric_null_v1": {
        **P1_FORK_MAP_PRESET, **P1_FUSED_SENSOR_PRESET,
        "p1_map_fixture": "p1_fork_symmetric_null_v1",
        "p1_fixture_risky_tree_density_per_m2": "0.25",
        "p1_fixture_risky_canopy_probability": "0.05",
    },
    "p1_soft_risk_island_v1": {
        **P1_FORK_MAP_PRESET, **P1_FUSED_SENSOR_PRESET,
        "p1_map_fixture": "p1_soft_risk_island_v1",
        "p1_fixture_central_obstacle_enabled": "false",
    },
}


EXPERIMENT_PRESETS = {
    "baseline_fused_nominal_off": {
        "scenario": "fused_nominal",
        "planner_safety_profile": "off",
        "p0.debug_metrics_enable": "false",
        "p1.debug_csv_enable": "false",
        "p2.debug_csv_enable": "false",
        "p3.debug_csv_enable": "false",
        "p4.debug_csv_enable": "false",
        "p5.debug_metrics_enable": "false",
    },
    "baseline_corridor_off": {
        "scenario": "lidar_corridor_degenerate",
        "planner_safety_profile": "off",
    },
    "p0_open_sky": {
        "scenario": "gnss_open_sky",
        "planner_safety_profile": "off",
        "planner_start_delay_s": "10.0",
        "corridor_floor_enabled": "true",
        "corridor_x_min_m": "-14.0",
        "corridor_x_max_m": "13.0",
        "corridor_half_width_y_m": "2.0",
        "corridor_floor_thickness_z_m": "0.05",
        "terminal_wall_enabled": "true",
        "terminal_wall_x_m": "12.8",
        "terminal_wall_y_m": "0.0",
        "terminal_wall_width_y_m": "5.0",
        "terminal_wall_z_min_m": "0.0",
        "terminal_wall_z_max_m": "3.2",
        "terminal_wall_thickness_x_m": "0.20",
        "terminal_wall_resolution_m": "0.10",
        "terminal_wall_feature_depth_x_m": "0.35",
        "terminal_wall_feature_count": "24",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p1.debug_csv_enable": "false",
        "p2.debug_csv_enable": "false",
        "p3.debug_csv_enable": "false",
        "p4.debug_csv_enable": "false",
        "p5.debug_metrics_enable": "false",
    },
    "p0_degraded_lidar_good": {
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "off",
        "planner_start_delay_s": "10.0",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p1.debug_csv_enable": "false",
        "p2.debug_csv_enable": "false",
        "p3.debug_csv_enable": "false",
        "p4.debug_csv_enable": "false",
        "p5.debug_metrics_enable": "false",
    },
    "p0_corridor_degenerate": {
        "scenario": "lidar_corridor_degenerate",
        "planner_safety_profile": "off",
        "planner_start_delay_s": "10.0",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p1.debug_csv_enable": "false",
        "p2.debug_csv_enable": "false",
        "p3.debug_csv_enable": "false",
        "p4.debug_csv_enable": "false",
        "p5.debug_metrics_enable": "false",
    },
    "p0_fallback_only": {
        "scenario": "fallback_only",
        "planner_safety_profile": "off",
        "planner_start_delay_s": "10.0",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p1.debug_csv_enable": "false",
        "p2.debug_csv_enable": "false",
        "p3.debug_csv_enable": "false",
        "p4.debug_csv_enable": "false",
        "p5.debug_metrics_enable": "false",
    },
    "p5_corridor": {
        "scenario": "lidar_corridor_degenerate",
        "planner_safety_profile": "p5",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p5.debug_metrics_enable": "true",
    },
    "p5_fallback_unknown": {
        "scenario": "fallback_only",
        "planner_safety_profile": "p5",
        "p0.enable_risk_grid": "true",
        "p0.debug_metrics_enable": "true",
        "p5.debug_metrics_enable": "true",
        "p5_3.fixture.enabled": "false",
        "p5_4.fixture.enabled": "false",
        "p5_6.fixture.enabled": "true",
        "p5_6.fixture.name": "future_unknown_zone_v1",
        "p5_6.fixture.x_min": "-1.0",
        "p5_6.fixture.x_max": "12.5",
        "p5_6.fixture.y_min": "-15.0",
        "p5_6.fixture.y_max": "15.0",
        "p5_6.fixture.z_min": "-3.0",
        "p5_6.fixture.z_max": "3.0",
        "p5_6.fixture.tau_min": "0.2",
        "p5_6.fixture.tau_max": "2.0",
    },
    "p1_degraded_lidar_good": {
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "p1",
        "p0.size_x_m": "42.0",
        # P1-2 keeps the existing 1 s stale timeout and grid geometry. Four
        # deterministic worker-local predictors address the measured refresh
        # bottleneck without changing P0 acceptance semantics.
        "p0.predictor.worker_count": "4",
        "p1.debug_csv_enable": "true",
        "safety_viz.enable_p1_viz": "true",
    },
    "p1_fork_formal": {
        "scenario": "p1_fork_fused_v1",
        "planner_safety_profile": "p1",
        "p0.size_x_m": "42.0",
        "p0.predictor.worker_count": "4",
        # Cover the complete fixed-200 local trajectory at the 1 m/s formal
        # checkpoint. The original 0--2.5 s prefix is unchanged; sparse future
        # layers through 16 s extend only this experiment's immutable
        # prediction contract.
        "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5,3.5,4.5,5.5,6.5,7.5,8.5,9.5,10.5,11.5,12.5,13.5,14.5,15.5,16.0",
        "p0.size_y_m": "12.0",
        "planner_start_delay_s": "10.0",
        "fsm.thresh_replan_time": "0.5",
        # Observe the central fork before the first formal decision checkpoint.
        # The default remains 5.5 m for every other experiment.
        "grid_map/local_update_range_x": "10.0",
        # Keep successive replans closer than the fixed 0.8 m decision window.
        # This is part of the formal configuration identity, not an analyzer
        # relaxation: the checkpoint remains truth x=-9.5+/-0.4 m.
        "manager/max_vel": "1.0",
        "optimization/max_vel": "1.0",
        "bspline/limit_vel": "1.0",
        "p1.debug_csv_enable": "true",
        "p1.lambda_integrity": "0.00001",
        "p1.normalization_budget_fraction": "0.30",
        "safety_viz.enable_p1_viz": "true",
    },
    "p2_degraded_lidar_good": {
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "p2",
        "manager/use_distinctive_trajs": "true",
        "p2.debug_csv_enable": "true",
        "safety_viz.enable_p2_viz": "true",
    },
    "p3_corridor": {
        "scenario": "lidar_corridor_degenerate",
        "planner_safety_profile": "p3",
        "p3.debug_csv_enable": "true",
        "safety_viz.enable_p3_viz": "true",
    },
    "p4_manual_collision_guide": {
        "scenario": "manual",
        "planner_safety_profile": "p4",
        "p4.debug_csv_enable": "true",
        "safety_viz.enable_p4_viz": "true",
    },
    "all_degraded_lidar_good": {
        "scenario": "gnss_degraded_lidar_good",
        "planner_safety_profile": "all",
        "p0.debug_metrics_enable": "true",
        "p1.debug_csv_enable": "true",
        "p2.debug_csv_enable": "true",
        "p3.debug_csv_enable": "true",
        "p4.debug_csv_enable": "true",
        "p5.debug_metrics_enable": "true",
        "safety_viz.enable_p1_viz": "true",
        "safety_viz.enable_p2_viz": "true",
        "safety_viz.enable_p3_viz": "true",
        "safety_viz.enable_p4_viz": "true",
    },
}


ARG_DEFAULTS = [
    ("experiment", "baseline_fused_nominal_off"),
    ("scenario", "fused_nominal"),
    ("config_subdir", "sim_demo11"),
    ("start_rviz", "true"),
    ("start_planner", "true"),
    ("record_bag", "false"),
    ("run_validator", "true"),
    ("bag_output_dir", "/home/dev/ws_iap/src/iap/results/planner_validation/bags"),
    ("runtime_root_dir", ""),
    ("export_root_dir", ""),
    ("run_duration_s", "90"),
    ("validation_duration_s", "85"),
    ("allow_truth_alignment", "true"),
    ("planner_start_delay_s", "0.0"),
    ("fsm.thresh_replan_time", "1.0"),
    ("enable_preflight_takeoff", "false"),
    ("preflight_ground_z", "0.0"),
    ("preflight_ground_hold_s", "10.0"),
    ("preflight_takeoff_duration_s", "5.0"),
    ("preflight_hover_s", "30.0"),
    ("preflight_cmd_rate_hz", "50.0"),
    ("use_gnss", "true"),
    ("use_araim", "true"),
    ("enable_gnss_integrity", "true"),
    ("enable_gnss_araim", "true"),
    ("enable_lidar_integrity", "true"),
    ("enable_araim_pl_decomp_csv", "false"),
    ("integrity_fusion_mode", "max_pl"),
    ("integrity_require_valid_gnss", "false"),
    ("integrity_require_valid_lidar", "false"),
    ("integrity_conservative_hpl_m", "999.0"),
    ("integrity_conservative_vpl_m", "999.0"),
    ("validator_require_gnss_valid", "true"),
    ("validator_require_lidar_valid", "true"),
    ("validator_require_fallback_valid", "true"),
    ("validator_required_final_source", ""),
    ("validator_allowed_final_sources", "GNSS,LIDAR,FALLBACK,CONSERVATIVE"),
    ("init_x", "-12.0"),
    ("init_y", "0.0"),
    ("init_z", "1.2"),
    ("goal_x", "12.0"),
    ("goal_y", "0.0"),
    ("goal_z", "1.2"),
    ("point_num", "1"),
    ("point1_x", "9.5"),
    ("point1_y", "0.0"),
    ("point1_z", "1.2"),
    ("point2_x", "16.0"),
    ("point2_y", "0.0"),
    ("point2_z", "1.2"),
    ("point3_x", "16.0"),
    ("point3_y", "0.0"),
    ("point3_z", "1.2"),
    ("point4_x", "16.0"),
    ("point4_y", "0.0"),
    ("point4_z", "1.2"),
    ("point5_x", "16.0"),
    ("point5_y", "0.0"),
    ("point5_z", "1.2"),
    ("point6_x", "16.0"),
    ("point6_y", "0.0"),
    ("point6_z", "1.2"),
    ("drone_id", "0"),
    ("map_size_x", "30.0"),
    ("map_size_y", "30.0"),
    ("map_size_z", "3.5"),
    ("corridor_map_resolution_m", "0.1"),
    ("corridor_map_publish_rate_hz", "2.0"),
    ("forest_size_x_m", "20.0"),
    ("forest_size_y_m", "20.0"),
    ("tree_density_lower_left_per_m2", "0.5"),
    ("tree_density_lower_right_per_m2", "0.5"),
    ("tree_density_upper_left_per_m2", "0.5"),
    ("tree_density_upper_right_per_m2", "0.5"),
    ("stratified_cell_size_m", "1.0"),
    ("clear_corridor_enabled", "false"),
    ("clear_corridor_center_y_m", "0.0"),
    ("clear_corridor_half_width_y_m", "0.0"),
    ("clear_corridor_x_min_m", "-1000000000.0"),
    ("clear_corridor_x_max_m", "1000000000.0"),
    ("canopy_density_lower_left", "0.1"),
    ("canopy_density_lower_right", "0.1"),
    ("canopy_density_upper_left", "0.6"),
    ("canopy_density_upper_right", "0.6"),
    ("canopy_hemisphere_radius_min_m", "0.5"),
    ("canopy_hemisphere_radius_max_m", "1.5"),
    ("canopy_leaf_ball_radius_m", "0.22"),
    ("canopy_ball_spacing_ratio", "1.2"),
    ("canopy_resolution_m", "0.15"),
    ("forest_random_seed", "11"),
    ("trunk_radius_m", "0.14"),
    ("trunk_min_height_m", "1.5"),
    ("trunk_max_height_m", "3.0"),
    ("terminal_wall_enabled", "true"),
    ("terminal_wall_x_m", "13.5"),
    ("terminal_wall_y_m", "0.0"),
    ("terminal_wall_width_y_m", "10.0"),
    ("terminal_wall_z_min_m", "0.0"),
    ("terminal_wall_z_max_m", "3.2"),
    ("terminal_wall_thickness_x_m", "0.20"),
    ("terminal_wall_resolution_m", "0.10"),
    ("terminal_wall_feature_depth_x_m", "0.65"),
    ("terminal_wall_feature_count", "48"),
    ("terminal_wall_feature_seed", "11022"),
    ("corridor_walls_enabled", "false"),
    ("corridor_floor_enabled", "false"),
    ("corridor_x_min_m", "-14.0"),
    ("corridor_x_max_m", "14.0"),
    ("corridor_half_width_y_m", "2.0"),
    ("corridor_wall_z_min_m", "0.0"),
    ("corridor_wall_z_max_m", "3.0"),
    ("corridor_wall_thickness_y_m", "0.10"),
    ("corridor_floor_thickness_z_m", "0.05"),
    ("corridor_surface_resolution_m", "0.10"),
    ("p1_map_fixture", ""),
    ("p1_fixture_mirror_y", "false"),
    ("p1_fixture_central_obstacle_enabled", "false"),
    ("p1_fixture_central_x_min_m", "-7.0"),
    ("p1_fixture_central_x_max_m", "-2.0"),
    ("p1_fixture_central_y_half_width_m", "0.65"),
    ("p1_fixture_central_z_max_m", "2.8"),
    ("p1_fixture_lane_center_m", "2.0"),
    ("p1_fixture_lane_half_width_m", "0.75"),
    ("p1_fixture_safe_tree_density_per_m2", "0.25"),
    ("p1_fixture_risky_tree_density_per_m2", "0.75"),
    ("p1_fixture_safe_canopy_probability", "0.05"),
    ("p1_fixture_risky_canopy_probability", "0.85"),
    ("grid_map/local_update_range_x", "5.5"),
    ("gnss_pr_noise_base", "5.0"),
    ("gnss_dop_noise_base", "0.5"),
    ("gnss_random_seed", "20260429"),
    ("gnss_ephemeris_source", "rinex"),
    ("gnss_time_source", "trigger_topic"),
    ("gnss_enabled_constellations", "GPS,BDS,GAL,GLO"),
    ("gnss_scenario_file", "config/gnss_sim/demo7_skymask_nlos.yaml"),
    ("gnss_rinex_nav_file", "/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx"),
    ("gnss_rinex_ephem_max_age_s", "7200.0"),
    ("gnss_fallback_to_synthetic_on_rinex_error", "false"),
    ("gnss_enable_map_occlusion", "true"),
    ("gnss_enable_skymask", "true"),
    ("gnss_enable_nlos", "true"),
    ("gnss_enable_multipath", "true"),
    ("gnss_enable_fault_injection", "true"),
    ("gnss_enable_visualization", "true"),
    ("gnss_enable_sky_dome_visualization", "true"),
    ("gnss_sky_dome_show_cardinal_labels", "true"),
    ("gnss_sky_dome_follow_receiver", "false"),
    ("gnss_sky_dome_center_x", "0.0"),
    ("gnss_sky_dome_center_y", "0.0"),
    ("gnss_sky_dome_center_z", "0.0"),
    ("gnss_skyplot_origin_x", "0.0"),
    ("gnss_skyplot_origin_y", "22.0"),
    ("gnss_skyplot_origin_z", "5.0"),
    ("viz_status_text_use_fixed_position", "true"),
    ("viz_status_text_x", "15.0"),
    ("viz_status_text_y", "0.0"),
    ("viz_status_text_z", "5.0"),
    ("planner_safety_profile", "off"),
    ("planner_enable_all_safety", "false"),
    ("planner_enable_p1", "false"),
    ("planner_enable_p2", "false"),
    ("planner_enable_p3_local", "false"),
    ("planner_enable_p3_global", "false"),
    ("planner_enable_p4", "false"),
    ("planner_enable_p5_runtime", "false"),
    ("planner_enable_p5_final", "false"),
    ("planner_enable_safety_viz", "true"),
    ("safety_viz.selected_horizon_s", "1.0"),
    ("safety_viz.z_slice_mode", "current_altitude"),
    ("safety_viz.z_slice_half_thickness_m", "0.75"),
    ("safety_viz.publish_rate_hz", "2.0"),
    ("safety_viz.max_cloud_points", "20000"),
    ("safety_viz.enable_im_bars", "true"),
    ("safety_viz.enable_validity_cloud", "true"),
    ("safety_viz.enable_p1_viz", "false"),
    ("safety_viz.enable_p2_viz", "false"),
    ("safety_viz.enable_p3_viz", "false"),
    ("safety_viz.enable_p4_viz", "false"),
    ("p0.enable_risk_grid", "false"),
    ("p0.resolution_m", "0.75"),
    ("p0.size_x_m", "30.0"),
    ("p0.size_y_m", "30.0"),
    ("p0.size_z_m", "6.0"),
    # The fixed P1 lattice must cover every sample of the initial
    # degraded-LiDAR B-spline (about 2.1 s) before P1 can be admitted.
    ("p0.horizons_s", "0.0,0.5,1.0,1.5,2.0,2.5"),
    ("p0.refresh_period_s", "0.5"),
    ("p0.stale_timeout_s", "1.0"),
    ("p0.skip_occupied_voxels", "true"),
    ("p0.debug_metrics_enable", "false"),
    ("p0.health_topic", "planning/risk_grid_health"),
    ("p0.gnss_epoch_max_age_s", "2.0"),
    ("p0.predictor.source_mode", "fusion"),
    ("p0.predictor.gnss_epoch_policy", "auto"),
    ("p0.predictor.use_current_integrity_prior", "true"),
    ("p0.predictor.conservative_max_with_gnss", "false"),
    ("p0.predictor.lidar_legacy_observability", "true"),
    ("p0.predictor.lidar_fim_radius_m", "12.0"),
    ("p0.predictor.worker_count", "1"),
    ("p0_6.fixture.enabled", "false"),
    ("p0_6.fixture.name", ""),
    ("p0_6.fixture.x_min", "-1.5"),
    ("p0_6.fixture.x_max", "1.5"),
    ("p0_6.fixture.y_min", "-0.75"),
    ("p0_6.fixture.y_max", "0.75"),
    ("p0_6.fixture.z_min", "1.0"),
    ("p0_6.fixture.z_max", "2.0"),
    ("p0_6.fixture.raw_hpl_m", "1.0"),
    ("p0_6.fixture.raw_vpl_m", "1.2"),
    ("p0_6.fixture.raw_c_pi", "1.2"),
    ("p0_6.fixture.low_raw_cost_threshold", "2.0"),
    ("p5_3.fixture.enabled", "false"),
    ("p5_3.fixture.name", "future_high_risk_zone_v1"),
    ("p5_3.fixture.x_min", "-10.8"),
    ("p5_3.fixture.x_max", "-8.7"),
    ("p5_3.fixture.y_min", "-0.75"),
    ("p5_3.fixture.y_max", "0.75"),
    ("p5_3.fixture.z_min", "1.0"),
    ("p5_3.fixture.z_max", "1.35"),
    ("p5_3.fixture.tau_min", "1.2"),
    ("p5_3.fixture.tau_max", "2.0"),
    ("p5_3.fixture.hpl_pred_m", "10.2"),
    ("p5_3.fixture.vpl_pred_m", "10.2"),
    ("p5_4.fixture.enabled", "false"),
    ("p5_4.fixture.name", "near_risk_zone_v1"),
    ("p5_4.fixture.x_min", "-11.7"),
    ("p5_4.fixture.x_max", "-11.1"),
    ("p5_4.fixture.y_min", "-0.75"),
    ("p5_4.fixture.y_max", "0.75"),
    ("p5_4.fixture.z_min", "1.0"),
    ("p5_4.fixture.z_max", "1.35"),
    ("p5_4.fixture.tau_min", "0.6"),
    ("p5_4.fixture.tau_max", "0.95"),
    ("p5_4.fixture.hpl_pred_m", "10.2"),
    ("p5_4.fixture.vpl_pred_m", "10.2"),
    ("p5_5.fixture.enabled", "false"),
    ("p5_5.fixture.name", "current_integrity_stamp_freeze_v1"),
    ("p5_5.fixture.start_s", "30.0"),
    ("p5_5.fixture.duration_s", "12.0"),
    ("p5_6.fixture.enabled", "false"),
    ("p5_6.fixture.name", "future_unknown_zone_v1"),
    ("p5_6.fixture.x_min", "-1.0"),
    ("p5_6.fixture.x_max", "12.5"),
    ("p5_6.fixture.y_min", "-15.0"),
    ("p5_6.fixture.y_max", "15.0"),
    ("p5_6.fixture.z_min", "-3.0"),
    ("p5_6.fixture.z_max", "3.0"),
    ("p5_6.fixture.tau_min", "0.2"),
    ("p5_6.fixture.tau_max", "2.0"),
    ("p5_7.fixture.enabled", "false"),
    ("p5_7.fixture.effective_enabled", "false"),
    ("p5_7.fixture.name", "rejected_trajectory_zone_v1"),
    ("p5_7.fixture.x_min", "-11.7"),
    ("p5_7.fixture.x_max", "-8.7"),
    ("p5_7.fixture.y_min", "-0.75"),
    ("p5_7.fixture.y_max", "0.75"),
    ("p5_7.fixture.z_min", "1.0"),
    ("p5_7.fixture.z_max", "1.35"),
    ("p5_7.fixture.tau_min", "0.6"),
    ("p5_7.fixture.tau_max", "2.0"),
    ("p5_7.fixture.hpl_pred_m", "10.2"),
    ("p5_7.fixture.vpl_pred_m", "10.2"),
    ("p1.use_integrity_cost", "false"),
    ("p1.metrics_only", "true"),
    ("p1.lambda_integrity", "0.0"),
    ("p1.sample_dt_min_s", "0.1"),
    ("p1.sample_dt_scale", "1.0"),
    ("p1.max_samples_per_eval", "30"),
    ("p1.integrity_cost_max", "100.0"),
    ("p1.integrity_grad_norm_max", "0.1"),
    ("p1.unknown_policy", "skip"),
    ("p1.unknown_soft_penalty", "1.0"),
    ("p1.debug_csv_enable", "false"),
    ("p1.debug_csv_path", ""),
    ("p1.max_candidates_per_attempt", "8"),
    ("p1.objective_aggregation_mode", "fixed_200_smooth_cvar"),
    ("p1.smooth_max_temperature", "0.01"),
    ("p1.smooth_cvar_alpha", "0.90"),
    ("p1.normalization_budget_fraction", "0.30"),
    ("p1.formal_calibration_manifest", ""),
    ("p2.enable_candidate_ranking", "false"),
    ("p2.metrics_only", "true"),
    ("p2.sample_dt_s", "0.2"),
    ("p2.lambda_candidate_integrity", "1.0"),
    ("p2.w_max_cost", "0.25"),
    ("p2.w_unknown", "5.0"),
    ("p2.w_stale", "2.0"),
    ("p2.min_valid_ratio", "0.3"),
    ("p2.debug_csv_enable", "false"),
    ("p2.debug_csv_path", ""),
    ("p3.enable_local_reference_bias", "false"),
    ("p3.enable_global_reference_bias", "false"),
    ("p3.local_bias_radius_m", "1.5"),
    ("p3.min_improvement_ratio", "0.05"),
    ("p3.w_risk", "1.0"),
    ("p3.w_detour", "0.25"),
    ("p3.w_unknown", "5.0"),
    ("p3.min_corridor_valid_ratio", "0.8"),
    ("p3.station_spacing_m", "2.0"),
    ("p3.lateral_sample_step_m", "1.0"),
    ("p3.lateral_sample_count_each_side", "3"),
    ("p3.beam_width", "5"),
    ("p3.max_detour_ratio", "1.5"),
    ("p3.debug_csv_enable", "false"),
    ("p3.debug_csv_path", ""),
    ("p4.enable_risk_aware_astar", "false"),
    ("p4.lambda_p4_risk", "0.05"),
    ("p4.risk_cost_max", "100.0"),
    ("p4.unknown_edge_penalty", "1.0"),
    ("p4.max_extra_path_ratio", "1.3"),
    ("p4.fallback_to_original_when_risk_not_ready", "true"),
    ("p4.debug_csv_enable", "false"),
    ("p4.debug_csv_path", ""),
    ("p5.enable_runtime_gate", "false"),
    ("p5.enable_final_gate", "false"),
    ("p5.horizon_s", "2.0"),
    ("p5.sample_dt_s", "0.2"),
    ("p5.current_stale_to_replan_s", "0.5"),
    ("p5.current_stale_to_emergency_s", "2.0"),
    ("p5.current_low_margin_to_emergency_s", "2.0"),
    ("p5.future_unknown_to_emergency_s", "2.0"),
    ("p5.final_gate_max_consecutive_failures", "3"),
    ("p5.final_gate_max_failure_duration_s", "1.0"),
    ("p5.current_replan_margin_m", "0.3"),
    ("p5.current_emergency_margin_m", "-0.2"),
    ("p5.future_replan_margin_m", "0.3"),
    ("p5.future_emergency_margin_m", "-0.5"),
    ("p5.max_bad_ratio", "0.25"),
    ("p5.max_unknown_ratio", "0.30"),
    ("p5.bad_tick_to_replan", "2"),
    ("p5.good_tick_to_clear", "2"),
    ("p5.pred_alert_limit_mode", "current_msg_constant"),
    ("p5.pred_alert_limit_constant_hal_m", "10.0"),
    ("p5.pred_alert_limit_constant_val_m", "10.0"),
    ("p5.pred_alert_limit_min_hal_m", "0.1"),
    ("p5.pred_alert_limit_max_hal_m", "50.0"),
    ("p5.pred_alert_limit_min_val_m", "0.1"),
    ("p5.pred_alert_limit_max_val_m", "50.0"),
    ("p5.pred_alert_limit_clearance_search_radius_m", "5.0"),
    ("p5.pred_alert_limit_clearance_step_m", "0.25"),
    ("p5.pred_alert_limit_drone_radius_m", "0.35"),
    ("p5.pred_alert_limit_clearance_scale", "1.0"),
    ("p5.pred_alert_limit_vertical_scale", "1.0"),
    ("p5.status_topic", "planning/integrity_gate_status"),
    ("p5.debug_metrics_enable", "false"),
    ("manager/max_vel", "2.0"),
    ("manager/max_acc", "3.0"),
    ("manager/max_jerk", "4.0"),
    ("manager/control_points_distance", "0.4"),
    ("manager/feasibility_tolerance", "0.05"),
    ("manager/planning_horizon", "7.5"),
    ("manager/use_distinctive_trajs", "true"),
    ("optimization/lambda_smooth", "1.0"),
    ("optimization/lambda_collision", "0.5"),
    ("optimization/lambda_feasibility", "0.1"),
    ("optimization/lambda_fitness", "1.0"),
    ("optimization/dist0", "0.5"),
    ("optimization/swarm_clearance", "0.5"),
    ("optimization/max_vel", "2.0"),
    ("optimization/max_acc", "3.0"),
    ("bspline/limit_vel", "2.0"),
    ("bspline/limit_acc", "3.0"),
    ("bspline/limit_ratio", "1.1"),
]


def _maybe_resolve_iap_config_path(key, value, iap_share):
    if key == "gnss_scenario_file" and str(value).startswith("config/"):
        return str(Path(iap_share) / str(value))
    return value


def _apply_preset_values(context, preset, user_overrides, iap_share, applied_keys):
    for key, value in preset.items():
        if key in user_overrides:
            continue
        value = _maybe_resolve_iap_config_path(key, value, iap_share)
        context.launch_configurations[key] = _launch_value(value)
        applied_keys.add(key)


def _apply_presets(context, iap_share):
    user_overrides = _launch_arg_overrides()

    experiment = LaunchConfiguration("experiment").perform(context).strip()
    if not experiment:
        experiment = "baseline_fused_nominal_off"
    if experiment not in EXPERIMENT_PRESETS:
        valid = ", ".join(sorted(EXPERIMENT_PRESETS.keys()))
        raise RuntimeError(
            f"unknown test_planner experiment '{experiment}'. Valid: {valid}"
        )

    experiment_preset = EXPERIMENT_PRESETS[experiment]
    experiment_scenario = str(experiment_preset.get("scenario", "")).strip()
    if experiment_scenario and experiment_scenario not in SCENARIO_PRESETS:
        valid = ", ".join(sorted(SCENARIO_PRESETS.keys()))
        raise RuntimeError(
            f"test_planner experiment '{experiment}' references unknown "
            f"scenario '{experiment_scenario}'. Valid: {valid}"
        )

    if experiment_scenario and "scenario" not in user_overrides:
        context.launch_configurations["scenario"] = experiment_scenario

    scenario = LaunchConfiguration("scenario").perform(context).strip() or "manual"
    if scenario not in SCENARIO_PRESETS:
        valid = ", ".join(sorted(SCENARIO_PRESETS.keys()))
        raise RuntimeError(f"unknown test_planner scenario '{scenario}'. Valid: {valid}")

    applied_keys = set()
    _apply_preset_values(
        context, SCENARIO_PRESETS[scenario], user_overrides, iap_share, applied_keys
    )
    _apply_preset_values(context, experiment_preset, user_overrides, iap_share, applied_keys)
    combo_preset = COMBO_PRESETS.get((experiment, scenario))
    if combo_preset:
        _apply_preset_values(context, combo_preset, user_overrides, iap_share, applied_keys)
    context.launch_configurations["experiment"] = experiment
    context.launch_configurations["scenario"] = scenario
    return scenario, experiment, applied_keys


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
    raise RuntimeError(f"unknown generated GNSS scenario '{name}'")


def _materialize_gnss_scenario(scenario_file, export_dir):
    if not str(scenario_file).startswith("generated:"):
        return scenario_file
    scenario_name = str(scenario_file).split(":", 1)[1]
    path = Path(export_dir) / f"{scenario_name}.yaml"
    path.write_text(_generated_gnss_scenario(scenario_name))
    return str(path)


def _resolve_run_roots(config_name, experiment_name, scenario_name, run_token,
                       *, runtime_root_dir="", export_root_dir=""):
    runtime_base = Path(runtime_root_dir).expanduser() if runtime_root_dir else Path("/tmp")
    export_base = (
        Path(export_root_dir).expanduser()
        if export_root_dir
        else Path("/home/dev/ws_iap/src/iap/results/planner_validation/exports")
    )
    runtime_root = runtime_base / f"iap_{config_name}_test_planner_{run_token}"
    export_dir = export_base / (
        f"test_planner_{experiment_name}_{scenario_name}_{run_token}"
    )
    return runtime_root, export_dir


def _runtime_config(context, use_gnss, use_araim, allow_truth_alignment):
    iap_share = Path(get_package_share_directory("iap"))
    base_config = iap_share / "config"
    config_subdir = LaunchConfiguration("config_subdir").perform(context)
    config_name = config_subdir.replace("/", "_").replace("\\", "_")
    run_stamp = int(time.time() * 1000)
    experiment_name = _safe_path_component(
        LaunchConfiguration("experiment").perform(context), "experiment"
    )
    scenario_name = _safe_path_component(
        LaunchConfiguration("scenario").perform(context), "scenario"
    )
    runtime_root, export_dir = _resolve_run_roots(
        config_name, experiment_name, scenario_name,
        f"{os.getpid()}_{run_stamp}",
        runtime_root_dir=LaunchConfiguration("runtime_root_dir").perform(context).strip(),
        export_root_dir=LaunchConfiguration("export_root_dir").perform(context).strip(),
    )
    runtime_config_dir = runtime_root / config_subdir
    export_dir.mkdir(parents=True, exist_ok=True)

    shutil.copytree(base_config / config_subdir, runtime_config_dir, ignore_dangling_symlinks=True)
    shutil.copytree(base_config / "sim_ego", runtime_root / "sim_ego", ignore_dangling_symlinks=True)

    config_ros_path = runtime_config_dir / "config_ros.json"
    config_gnss_path = runtime_config_dir / "config_gnss.json"

    with config_ros_path.open() as f:
        config_ros = json.load(f)
    modules = ["libsim_extension.so"]
    if use_gnss:
        modules.insert(0, "libgnss_extension.so")
    if use_araim:
        modules.insert(1 if use_gnss else 0, "libintegrity_extension.so")
    config_ros["glim_ros"]["extension_modules"] = modules
    config_ros["glim_ros"]["imu_topic"] = "/sim/drone_0/imu_iap"
    config_ros["glim_ros"]["points_topic"] = "/sim/drone_0/lidar_body"
    config_ros["glim_ros"]["dump_path"] = str(runtime_root / "dump")
    config_ros["glim_ros"]["sim"]["align_planner_odom_to_truth"] = allow_truth_alignment
    config_ros["glim_ros"]["sim"]["metrics_csv_path"] = str(export_dir / "iap_sim_truth_vs_est.csv")
    with config_ros_path.open("w") as f:
        json.dump(config_ros, f, indent=2)
        f.write("\n")

    with config_gnss_path.open() as f:
        config_gnss = json.load(f)
    gnss = config_gnss["gnss"]
    gnss["pr_noise_base"] = float(LaunchConfiguration("gnss_pr_noise_base").perform(context))
    gnss["dop_noise_base"] = float(LaunchConfiguration("gnss_dop_noise_base").perform(context))
    gnss["debug_csv_path"] = str(export_dir / "iap_gnss_factor_debug.csv")

    integrity = config_gnss["integrity"]
    enable_gnss_integrity = _as_bool(LaunchConfiguration("enable_gnss_integrity").perform(context))
    enable_gnss_araim = _as_bool(LaunchConfiguration("enable_gnss_araim").perform(context))
    enable_lidar_integrity = _as_bool(LaunchConfiguration("enable_lidar_integrity").perform(context))
    enable_pl_decomp_csv = _as_bool(LaunchConfiguration("enable_araim_pl_decomp_csv").perform(context))
    fusion_mode = LaunchConfiguration("integrity_fusion_mode").perform(context)
    integrity["enable"] = bool(use_araim)
    integrity["enable_araim"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_araim_csv"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_araim_pl_decomp_csv"] = bool(
        use_araim and use_gnss and enable_gnss_araim and enable_pl_decomp_csv
    )
    integrity["publish_topic"] = "/iap/integrity"
    integrity["araim_csv_path"] = str(export_dir / "iap_araim.csv")
    integrity["araim_pl_decomp_csv_path"] = str(export_dir / "iap_araim_pl_decomp.csv")
    integrity["traj_csv_path"] = str(export_dir / "traj_with_gnss.csv")
    integrity["enable_lidar_araim_stage0_csv"] = bool(use_araim and enable_lidar_integrity and enable_pl_decomp_csv)
    integrity["lidar_araim_stage0_csv_path"] = str(export_dir / "iap_lidar_araim_stage0.csv")
    integrity["enable_gnss_integrity"] = bool(use_gnss and enable_gnss_integrity)
    integrity["enable_gnss_araim"] = bool(use_araim and use_gnss and enable_gnss_araim)
    integrity["enable_lidar_integrity"] = bool(use_araim and enable_lidar_integrity)
    integrity["integrity_fusion_mode"] = fusion_mode
    integrity["integrity_require_valid_gnss"] = _as_bool(LaunchConfiguration("integrity_require_valid_gnss").perform(context))
    integrity["integrity_require_valid_lidar"] = _as_bool(LaunchConfiguration("integrity_require_valid_lidar").perform(context))
    integrity["integrity_conservative_hpl_m"] = float(LaunchConfiguration("integrity_conservative_hpl_m").perform(context))
    integrity["integrity_conservative_vpl_m"] = float(LaunchConfiguration("integrity_conservative_vpl_m").perform(context))
    integrity["p5_5.fixture.enabled"] = _as_bool(LaunchConfiguration("p5_5.fixture.enabled").perform(context))
    integrity["p5_5.fixture.name"] = LaunchConfiguration("p5_5.fixture.name").perform(context)
    integrity["p5_5.fixture.start_s"] = float(LaunchConfiguration("p5_5.fixture.start_s").perform(context))
    integrity["p5_5.fixture.duration_s"] = float(LaunchConfiguration("p5_5.fixture.duration_s").perform(context))
    with config_gnss_path.open("w") as f:
        json.dump(config_gnss, f, indent=2)
        f.write("\n")

    return str(runtime_config_dir), str(runtime_root), str(export_dir)


def _resolve_safety_switches(context, preset_keys=None):
    preset_keys = preset_keys or set()
    overrides = _launch_arg_overrides()
    profile = LaunchConfiguration("planner_safety_profile").perform(context).strip().lower() or "off"
    if _as_bool(LaunchConfiguration("planner_enable_all_safety").perform(context)):
        profile = "all"
    valid_profiles = {"off", "p1", "p2", "p3", "p4", "p5", "all"}
    if profile not in valid_profiles:
        raise RuntimeError(f"unknown planner_safety_profile '{profile}'. Valid: {', '.join(sorted(valid_profiles))}")

    profile_defaults = {
        "p1": profile in ("p1", "all"),
        "p2": profile in ("p2", "all"),
        "p3_local": profile in ("p3", "all"),
        "p3_global": profile in ("p3", "all"),
        "p4": profile in ("p4", "all"),
        "p5_runtime": profile in ("p5", "all"),
        "p5_final": profile in ("p5", "all"),
    }
    arg_names = {
        "p1": "planner_enable_p1",
        "p2": "planner_enable_p2",
        "p3_local": "planner_enable_p3_local",
        "p3_global": "planner_enable_p3_global",
        "p4": "planner_enable_p4",
        "p5_runtime": "planner_enable_p5_runtime",
        "p5_final": "planner_enable_p5_final",
    }
    enabled = {}
    for key, arg_name in arg_names.items():
        if arg_name in overrides or arg_name in preset_keys:
            enabled[key] = _as_bool(LaunchConfiguration(arg_name).perform(context))
        else:
            enabled[key] = profile_defaults[key]

    needs_p0 = any(enabled.values())
    requested_p0 = _as_bool(LaunchConfiguration("p0.enable_risk_grid").perform(context))
    p0_explicit = "p0.enable_risk_grid" in overrides or "p0.enable_risk_grid" in preset_keys
    p0_enabled = requested_p0
    p0_conflict = False
    if needs_p0 and not p0_explicit:
        p0_enabled = True
    elif needs_p0 and p0_explicit and not requested_p0:
        p0_conflict = True

    return profile, enabled, p0_enabled, p0_conflict, overrides


def _param_bool(context, name):
    return _as_bool(LaunchConfiguration(name).perform(context))


def _param_float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _param_int(context, name):
    return int(LaunchConfiguration(name).perform(context))


def _fixed_lattice_no_replan_threshold(safety_enabled):
    # P1 formal evidence needs one last receding-horizon plan after the
    # trajectory fits the 2.5 s snapshot horizon.  Match the manager's own
    # 0.2 m "Close to goal" boundary so timing cannot end the fixture first.
    return 0.2 if safety_enabled.get("p1", False) else 1.0


def _p5_6_fixture_effective_enabled(context):
    return _param_bool(context, "p5_6.fixture.enabled") and not _param_bool(context, "p5_5.fixture.enabled")


def _p5_7_fixture_effective_enabled(context, p5_final_enabled):
    p5_6_effective = _p5_6_fixture_effective_enabled(context)
    return (
        _param_bool(context, "p5_7.fixture.enabled")
        and bool(p5_final_enabled)
        and not _param_bool(context, "p5_3.fixture.enabled")
        and not _param_bool(context, "p5_4.fixture.enabled")
        and not _param_bool(context, "p5_5.fixture.enabled")
        and not p5_6_effective
    )


def _odom_visualization_node(name, odom_topic, cmd_topic, topic_prefix, color, drone_id, fixed_text=False, fixed_position=(0.0, 0.0, 14.0)):
    r, g, b = color
    text_x, text_y, text_z = fixed_position
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
            {"sensor_text_use_fixed_position": bool(fixed_text)},
            {"sensor_text_fixed_x": float(text_x)},
            {"sensor_text_fixed_y": float(text_y)},
            {"sensor_text_fixed_z": float(text_z)},
        ],
    )


def _ego_planner_node(context, drone_id, planner_odom_topic, cloud_topic, camera_pose_topic, depth_topic, bspline_topic, map_size, goal, point_num, safety_profile, safety_enabled, p0_enabled, export_dir, evidence):
    p1_enabled = safety_enabled["p1"]
    p2_enabled = safety_enabled["p2"]
    p3_local_enabled = safety_enabled["p3_local"]
    p3_global_enabled = safety_enabled["p3_global"]
    p4_enabled = safety_enabled["p4"]
    p5_runtime_enabled = safety_enabled["p5_runtime"]
    p5_final_enabled = safety_enabled["p5_final"]
    overrides = _launch_arg_overrides()

    p1_use = _param_bool(context, "p1.use_integrity_cost") if "p1.use_integrity_cost" in overrides else p1_enabled
    p1_metrics_only = _param_bool(context, "p1.metrics_only") if "p1.metrics_only" in overrides else (not p1_enabled)
    p2_use = _param_bool(context, "p2.enable_candidate_ranking") if "p2.enable_candidate_ranking" in overrides else p2_enabled
    p2_metrics_only = _param_bool(context, "p2.metrics_only") if "p2.metrics_only" in overrides else (not p2_enabled)
    p3_local = _param_bool(context, "p3.enable_local_reference_bias") if "p3.enable_local_reference_bias" in overrides else p3_local_enabled
    p3_global = _param_bool(context, "p3.enable_global_reference_bias") if "p3.enable_global_reference_bias" in overrides else p3_global_enabled
    p4_use = _param_bool(context, "p4.enable_risk_aware_astar") if "p4.enable_risk_aware_astar" in overrides else p4_enabled
    p5_runtime = _param_bool(context, "p5.enable_runtime_gate") if "p5.enable_runtime_gate" in overrides else p5_runtime_enabled
    p5_final = _param_bool(context, "p5.enable_final_gate") if "p5.enable_final_gate" in overrides else p5_final_enabled
    p5_6_fixture_effective_enabled = _p5_6_fixture_effective_enabled(context)
    p5_7_fixture_requested = _param_bool(context, "p5_7.fixture.enabled")
    p5_7_fixture_effective_enabled = _p5_7_fixture_effective_enabled(context, p5_final)

    p1_debug_path = LaunchConfiguration("p1.debug_csv_path").perform(context)
    if not p1_debug_path:
        p1_debug_path = str(Path(export_dir) / "planner_p1_integrity_cost_debug.csv")
    p2_debug_path = LaunchConfiguration("p2.debug_csv_path").perform(context)
    if not p2_debug_path:
        p2_debug_path = str(Path(export_dir) / "planner_p2_candidate_ranking_debug.csv")
    p3_debug_path = LaunchConfiguration("p3.debug_csv_path").perform(context)
    if not p3_debug_path:
        p3_debug_path = str(Path(export_dir) / "planner_p3_reference_bias_debug.csv")
    p4_debug_path = LaunchConfiguration("p4.debug_csv_path").perform(context)
    if not p4_debug_path:
        p4_debug_path = str(Path(export_dir) / "planner_p4_risk_astar_debug.csv")

    map_size_x, map_size_y, map_size_z = map_size
    goal_x, goal_y, goal_z = goal
    waypoint_values = [(float(goal_x), float(goal_y), float(goal_z))]
    for i in range(1, max(1, min(int(point_num), 7))):
        waypoint_values.append(
            (
                float(LaunchConfiguration(f"point{i}_x").perform(context)),
                float(LaunchConfiguration(f"point{i}_y").perform(context)),
                float(LaunchConfiguration(f"point{i}_z").perform(context)),
            )
        )
    if _param_bool(context, "p5_5.fixture.enabled"):
        waypoint_values = list(P5_5_FIXTURE_ROUTE_WAYPOINTS)

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
            {"fsm/thresh_replan_time": _param_float(context, "fsm.thresh_replan_time")},
            {"fsm/thresh_no_replan_meter": _fixed_lattice_no_replan_threshold(safety_enabled)},
            {"fsm/planning_horizon": _param_float(context, "manager/planning_horizon")},
            {"fsm/planning_horizen_time": 3.0},
            {"fsm/emergency_time": 1.0},
            {"fsm/realworld_experiment": False},
            {"fsm/fail_safe": True},
            {"fsm/waypoint_num": len(waypoint_values)},
            *[
                {f"fsm/waypoint{i}_{axis}": value}
                for i, waypoint in enumerate(waypoint_values)
                for axis, value in zip(("x", "y", "z"), waypoint)
            ],
            {"grid_map/resolution": 0.1},
            {"grid_map/map_size_x": map_size_x},
            {"grid_map/map_size_y": map_size_y},
            {"grid_map/map_size_z": map_size_z},
            {"grid_map/local_update_range_x": _param_float(context, "grid_map/local_update_range_x")},
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
            {"p0.enable_risk_grid": p0_enabled},
            {"p0.resolution_m": _param_float(context, "p0.resolution_m")},
            {"p0.size_x_m": _param_float(context, "p0.size_x_m")},
            {"p0.size_y_m": _param_float(context, "p0.size_y_m")},
            {"p0.size_z_m": _param_float(context, "p0.size_z_m")},
            {"p0.horizons_s": _csv_floats(LaunchConfiguration("p0.horizons_s").perform(context))},
            {"p0.refresh_period_s": _param_float(context, "p0.refresh_period_s")},
            {"p0.stale_timeout_s": _param_float(context, "p0.stale_timeout_s")},
            {"p0.skip_occupied_voxels": _param_bool(context, "p0.skip_occupied_voxels")},
            {"p0.debug_metrics_enable": _param_bool(context, "p0.debug_metrics_enable")},
            {"p0.odom_topic": planner_odom_topic},
            {"p0.integrity_topic": "/iap/integrity"},
            {"p0.range_meas_topic": "/ublox_driver/range_meas"},
            {"p0.ephem_topic": "/ublox_driver/ephem"},
            {"p0.glo_ephem_topic": "/ublox_driver/glo_ephem"},
            {"p0.receiver_lla_topic": "/ublox_driver/receiver_lla"},
            {"p0.iono_topic": "/ublox_driver/iono_params"},
            {"p0.map_topic": "/map_generator/global_cloud"},
            {"p0.health_topic": LaunchConfiguration("p0.health_topic").perform(context)},
            {"p0.gnss_epoch_max_age_s": _param_float(context, "p0.gnss_epoch_max_age_s")},
            {"p0.predictor.source_mode": LaunchConfiguration("p0.predictor.source_mode").perform(context)},
            {"p0.predictor.gnss_epoch_policy": LaunchConfiguration("p0.predictor.gnss_epoch_policy").perform(context)},
            {"p0.predictor.use_current_integrity_prior": _param_bool(context, "p0.predictor.use_current_integrity_prior")},
            {"p0.predictor.conservative_max_with_gnss": _param_bool(context, "p0.predictor.conservative_max_with_gnss")},
            {"p0.predictor.lidar_legacy_observability": _param_bool(context, "p0.predictor.lidar_legacy_observability")},
            {"p0.predictor.lidar_fim_radius_m": _param_float(context, "p0.predictor.lidar_fim_radius_m")},
            {"p0.predictor.worker_count": _param_int(context, "p0.predictor.worker_count")},
            {"p0_6.fixture.enabled": _param_bool(context, "p0_6.fixture.enabled")},
            {"p0_6.fixture.name": LaunchConfiguration("p0_6.fixture.name").perform(context)},
            {"p0_6.fixture.x_min": _param_float(context, "p0_6.fixture.x_min")},
            {"p0_6.fixture.x_max": _param_float(context, "p0_6.fixture.x_max")},
            {"p0_6.fixture.y_min": _param_float(context, "p0_6.fixture.y_min")},
            {"p0_6.fixture.y_max": _param_float(context, "p0_6.fixture.y_max")},
            {"p0_6.fixture.z_min": _param_float(context, "p0_6.fixture.z_min")},
            {"p0_6.fixture.z_max": _param_float(context, "p0_6.fixture.z_max")},
            {"p0_6.fixture.raw_hpl_m": _param_float(context, "p0_6.fixture.raw_hpl_m")},
            {"p0_6.fixture.raw_vpl_m": _param_float(context, "p0_6.fixture.raw_vpl_m")},
            {"p0_6.fixture.raw_c_pi": _param_float(context, "p0_6.fixture.raw_c_pi")},
            {"p0_6.fixture.low_raw_cost_threshold": _param_float(context, "p0_6.fixture.low_raw_cost_threshold")},
            {"p5_3.fixture.enabled": _param_bool(context, "p5_3.fixture.enabled")},
            {"p5_3.fixture.name": LaunchConfiguration("p5_3.fixture.name").perform(context)},
            {"p5_3.fixture.x_min": _param_float(context, "p5_3.fixture.x_min")},
            {"p5_3.fixture.x_max": _param_float(context, "p5_3.fixture.x_max")},
            {"p5_3.fixture.y_min": _param_float(context, "p5_3.fixture.y_min")},
            {"p5_3.fixture.y_max": _param_float(context, "p5_3.fixture.y_max")},
            {"p5_3.fixture.z_min": _param_float(context, "p5_3.fixture.z_min")},
            {"p5_3.fixture.z_max": _param_float(context, "p5_3.fixture.z_max")},
            {"p5_3.fixture.tau_min": _param_float(context, "p5_3.fixture.tau_min")},
            {"p5_3.fixture.tau_max": _param_float(context, "p5_3.fixture.tau_max")},
            {"p5_3.fixture.hpl_pred_m": _param_float(context, "p5_3.fixture.hpl_pred_m")},
            {"p5_3.fixture.vpl_pred_m": _param_float(context, "p5_3.fixture.vpl_pred_m")},
            {"p5_4.fixture.enabled": _param_bool(context, "p5_4.fixture.enabled")},
            {"p5_4.fixture.name": LaunchConfiguration("p5_4.fixture.name").perform(context)},
            {"p5_4.fixture.x_min": _param_float(context, "p5_4.fixture.x_min")},
            {"p5_4.fixture.x_max": _param_float(context, "p5_4.fixture.x_max")},
            {"p5_4.fixture.y_min": _param_float(context, "p5_4.fixture.y_min")},
            {"p5_4.fixture.y_max": _param_float(context, "p5_4.fixture.y_max")},
            {"p5_4.fixture.z_min": _param_float(context, "p5_4.fixture.z_min")},
            {"p5_4.fixture.z_max": _param_float(context, "p5_4.fixture.z_max")},
            {"p5_4.fixture.tau_min": _param_float(context, "p5_4.fixture.tau_min")},
            {"p5_4.fixture.tau_max": _param_float(context, "p5_4.fixture.tau_max")},
            {"p5_4.fixture.hpl_pred_m": _param_float(context, "p5_4.fixture.hpl_pred_m")},
            {"p5_4.fixture.vpl_pred_m": _param_float(context, "p5_4.fixture.vpl_pred_m")},
            {"p5_6.fixture.enabled": p5_6_fixture_effective_enabled},
            {"p5_6.fixture.name": LaunchConfiguration("p5_6.fixture.name").perform(context)},
            {"p5_6.fixture.x_min": _param_float(context, "p5_6.fixture.x_min")},
            {"p5_6.fixture.x_max": _param_float(context, "p5_6.fixture.x_max")},
            {"p5_6.fixture.y_min": _param_float(context, "p5_6.fixture.y_min")},
            {"p5_6.fixture.y_max": _param_float(context, "p5_6.fixture.y_max")},
            {"p5_6.fixture.z_min": _param_float(context, "p5_6.fixture.z_min")},
            {"p5_6.fixture.z_max": _param_float(context, "p5_6.fixture.z_max")},
            {"p5_6.fixture.tau_min": _param_float(context, "p5_6.fixture.tau_min")},
            {"p5_6.fixture.tau_max": _param_float(context, "p5_6.fixture.tau_max")},
            {"p5_7.fixture.enabled": p5_7_fixture_requested},
            {"p5_7.fixture.effective_enabled": p5_7_fixture_effective_enabled},
            {"p5_7.fixture.name": LaunchConfiguration("p5_7.fixture.name").perform(context)},
            {"p5_7.fixture.x_min": _param_float(context, "p5_7.fixture.x_min")},
            {"p5_7.fixture.x_max": _param_float(context, "p5_7.fixture.x_max")},
            {"p5_7.fixture.y_min": _param_float(context, "p5_7.fixture.y_min")},
            {"p5_7.fixture.y_max": _param_float(context, "p5_7.fixture.y_max")},
            {"p5_7.fixture.z_min": _param_float(context, "p5_7.fixture.z_min")},
            {"p5_7.fixture.z_max": _param_float(context, "p5_7.fixture.z_max")},
            {"p5_7.fixture.tau_min": _param_float(context, "p5_7.fixture.tau_min")},
            {"p5_7.fixture.tau_max": _param_float(context, "p5_7.fixture.tau_max")},
            {"p5_7.fixture.hpl_pred_m": _param_float(context, "p5_7.fixture.hpl_pred_m")},
            {"p5_7.fixture.vpl_pred_m": _param_float(context, "p5_7.fixture.vpl_pred_m")},
            {"p1.use_integrity_cost": p1_use},
            {"p1.metrics_only": p1_metrics_only},
            {"p1.lambda_integrity": _param_float(context, "p1.lambda_integrity")},
            {"p1.sample_dt_min_s": _param_float(context, "p1.sample_dt_min_s")},
            {"p1.sample_dt_scale": _param_float(context, "p1.sample_dt_scale")},
            {"p1.max_samples_per_eval": _param_int(context, "p1.max_samples_per_eval")},
            {"p1.integrity_cost_max": _param_float(context, "p1.integrity_cost_max")},
            {"p1.integrity_grad_norm_max": _param_float(context, "p1.integrity_grad_norm_max")},
            {"p1.unknown_policy": LaunchConfiguration("p1.unknown_policy").perform(context)},
            {"p1.unknown_soft_penalty": _param_float(context, "p1.unknown_soft_penalty")},
            {"p1.debug_csv_enable": _param_bool(context, "p1.debug_csv_enable")},
            {"p1.debug_csv_path": p1_debug_path},
            {"p1.evidence_schema_version": evidence["schema_version"]},
            {"p1.evidence_run_id": evidence["run_id"]},
            {"p1.evidence_manifest_path": evidence["manifest_path"]},
            {"p1.max_candidates_per_attempt": max(1, min(8, _param_int(context, "p1.max_candidates_per_attempt")))},
            {"p1.objective_aggregation_mode": LaunchConfiguration("p1.objective_aggregation_mode").perform(context)},
            {"p1.smooth_max_temperature": _param_float(context, "p1.smooth_max_temperature")},
            {"p1.smooth_cvar_alpha": _param_float(context, "p1.smooth_cvar_alpha")},
            {"p1.normalization_budget_fraction": _param_float(context, "p1.normalization_budget_fraction")},
            {"p2.enable_candidate_ranking": p2_use},
            {"p2.metrics_only": p2_metrics_only},
            {"p2.sample_dt_s": _param_float(context, "p2.sample_dt_s")},
            {"p2.lambda_candidate_integrity": _param_float(context, "p2.lambda_candidate_integrity")},
            {"p2.w_max_cost": _param_float(context, "p2.w_max_cost")},
            {"p2.w_unknown": _param_float(context, "p2.w_unknown")},
            {"p2.w_stale": _param_float(context, "p2.w_stale")},
            {"p2.min_valid_ratio": _param_float(context, "p2.min_valid_ratio")},
            {"p2.debug_csv_enable": _param_bool(context, "p2.debug_csv_enable")},
            {"p2.debug_csv_path": p2_debug_path},
            {"p3.enable_local_reference_bias": p3_local},
            {"p3.enable_global_reference_bias": p3_global},
            {"p3.local_bias_radius_m": _param_float(context, "p3.local_bias_radius_m")},
            {"p3.min_improvement_ratio": _param_float(context, "p3.min_improvement_ratio")},
            {"p3.w_risk": _param_float(context, "p3.w_risk")},
            {"p3.w_detour": _param_float(context, "p3.w_detour")},
            {"p3.w_unknown": _param_float(context, "p3.w_unknown")},
            {"p3.min_corridor_valid_ratio": _param_float(context, "p3.min_corridor_valid_ratio")},
            {"p3.station_spacing_m": _param_float(context, "p3.station_spacing_m")},
            {"p3.lateral_sample_step_m": _param_float(context, "p3.lateral_sample_step_m")},
            {"p3.lateral_sample_count_each_side": _param_int(context, "p3.lateral_sample_count_each_side")},
            {"p3.beam_width": _param_int(context, "p3.beam_width")},
            {"p3.max_detour_ratio": _param_float(context, "p3.max_detour_ratio")},
            {"p3.debug_csv_enable": _param_bool(context, "p3.debug_csv_enable")},
            {"p3.debug_csv_path": p3_debug_path},
            {"p4.enable_risk_aware_astar": p4_use},
            {"p4.lambda_p4_risk": _param_float(context, "p4.lambda_p4_risk")},
            {"p4.risk_cost_max": _param_float(context, "p4.risk_cost_max")},
            {"p4.unknown_edge_penalty": _param_float(context, "p4.unknown_edge_penalty")},
            {"p4.max_extra_path_ratio": _param_float(context, "p4.max_extra_path_ratio")},
            {"p4.fallback_to_original_when_risk_not_ready": _param_bool(context, "p4.fallback_to_original_when_risk_not_ready")},
            {"p4.debug_csv_enable": _param_bool(context, "p4.debug_csv_enable")},
            {"p4.debug_csv_path": p4_debug_path},
            {"p5.enable_runtime_gate": p5_runtime},
            {"p5.enable_final_gate": p5_final},
            {"p5.horizon_s": _param_float(context, "p5.horizon_s")},
            {"p5.sample_dt_s": _param_float(context, "p5.sample_dt_s")},
            {"p5.current_stale_to_replan_s": _param_float(context, "p5.current_stale_to_replan_s")},
            {"p5.current_stale_to_emergency_s": _param_float(context, "p5.current_stale_to_emergency_s")},
            {"p5.current_low_margin_to_emergency_s": _param_float(context, "p5.current_low_margin_to_emergency_s")},
            {"p5.future_unknown_to_emergency_s": _param_float(context, "p5.future_unknown_to_emergency_s")},
            {"p5.final_gate_max_consecutive_failures": _param_int(context, "p5.final_gate_max_consecutive_failures")},
            {"p5.final_gate_max_failure_duration_s": _param_float(context, "p5.final_gate_max_failure_duration_s")},
            {"p5.current_replan_margin_m": _param_float(context, "p5.current_replan_margin_m")},
            {"p5.current_emergency_margin_m": _param_float(context, "p5.current_emergency_margin_m")},
            {"p5.future_replan_margin_m": _param_float(context, "p5.future_replan_margin_m")},
            {"p5.future_emergency_margin_m": _param_float(context, "p5.future_emergency_margin_m")},
            {"p5.max_bad_ratio": _param_float(context, "p5.max_bad_ratio")},
            {"p5.max_unknown_ratio": _param_float(context, "p5.max_unknown_ratio")},
            {"p5.bad_tick_to_replan": _param_int(context, "p5.bad_tick_to_replan")},
            {"p5.good_tick_to_clear": _param_int(context, "p5.good_tick_to_clear")},
            {"p5.pred_alert_limit_mode": LaunchConfiguration("p5.pred_alert_limit_mode").perform(context)},
            {"p5.pred_alert_limit_constant_hal_m": _param_float(context, "p5.pred_alert_limit_constant_hal_m")},
            {"p5.pred_alert_limit_constant_val_m": _param_float(context, "p5.pred_alert_limit_constant_val_m")},
            {"p5.pred_alert_limit_min_hal_m": _param_float(context, "p5.pred_alert_limit_min_hal_m")},
            {"p5.pred_alert_limit_max_hal_m": _param_float(context, "p5.pred_alert_limit_max_hal_m")},
            {"p5.pred_alert_limit_min_val_m": _param_float(context, "p5.pred_alert_limit_min_val_m")},
            {"p5.pred_alert_limit_max_val_m": _param_float(context, "p5.pred_alert_limit_max_val_m")},
            {"p5.pred_alert_limit_clearance_search_radius_m": _param_float(context, "p5.pred_alert_limit_clearance_search_radius_m")},
            {"p5.pred_alert_limit_clearance_step_m": _param_float(context, "p5.pred_alert_limit_clearance_step_m")},
            {"p5.pred_alert_limit_drone_radius_m": _param_float(context, "p5.pred_alert_limit_drone_radius_m")},
            {"p5.pred_alert_limit_clearance_scale": _param_float(context, "p5.pred_alert_limit_clearance_scale")},
            {"p5.pred_alert_limit_vertical_scale": _param_float(context, "p5.pred_alert_limit_vertical_scale")},
            {"p5.integrity_topic": "/iap/integrity"},
            {"p5.status_topic": LaunchConfiguration("p5.status_topic").perform(context)},
            {"p5.debug_metrics_enable": _param_bool(context, "p5.debug_metrics_enable")},
            {"risk_overlay/enable": False},
            {"risk_overlay/use_for_astar": False},
            {"risk_overlay/use_for_bspline": False},
            {"risk_overlay/topic": ""},
            {"manager/max_vel": _param_float(context, "manager/max_vel")},
            {"manager/max_acc": _param_float(context, "manager/max_acc")},
            {"manager/max_jerk": _param_float(context, "manager/max_jerk")},
            {"manager/control_points_distance": _param_float(context, "manager/control_points_distance")},
            {"manager/feasibility_tolerance": _param_float(context, "manager/feasibility_tolerance")},
            {"manager/planning_horizon": _param_float(context, "manager/planning_horizon")},
            {"manager/use_distinctive_trajs": _param_bool(context, "manager/use_distinctive_trajs")},
            {"manager/drone_id": int(drone_id)},
            {"manager/use_integrity_global_search": False},
            {"optimization/lambda_smooth": _param_float(context, "optimization/lambda_smooth")},
            {"optimization/lambda_collision": _param_float(context, "optimization/lambda_collision")},
            {"optimization/lambda_feasibility": _param_float(context, "optimization/lambda_feasibility")},
            {"optimization/lambda_fitness": _param_float(context, "optimization/lambda_fitness")},
            {"optimization/dist0": _param_float(context, "optimization/dist0")},
            {"optimization/swarm_clearance": _param_float(context, "optimization/swarm_clearance")},
            {"optimization/max_vel": _param_float(context, "optimization/max_vel")},
            {"optimization/max_acc": _param_float(context, "optimization/max_acc")},
            {"optimization/use_integrity_cost": False},
            {"optimization/use_integrity_front_search": False},
            {"optimization/use_integrity_global_search": False},
            {"bspline/limit_vel": _param_float(context, "bspline/limit_vel")},
            {"bspline/limit_acc": _param_float(context, "bspline/limit_acc")},
            {"bspline/limit_ratio": _param_float(context, "bspline/limit_ratio")},
            {"prediction/obj_num": 0},
            {"prediction/lambda": 1.0},
            {"prediction/predict_rate": 1.0},
            {"test_planner/safety_profile": safety_profile},
            {"planner_enable_safety_viz": _param_bool(context, "planner_enable_safety_viz")},
            {"safety_viz.selected_horizon_s": _param_float(context, "safety_viz.selected_horizon_s")},
            {"safety_viz.z_slice_mode": LaunchConfiguration("safety_viz.z_slice_mode").perform(context)},
            {"safety_viz.z_slice_half_thickness_m": _param_float(context, "safety_viz.z_slice_half_thickness_m")},
            {"safety_viz.publish_rate_hz": _param_float(context, "safety_viz.publish_rate_hz")},
            {"safety_viz.max_cloud_points": _param_int(context, "safety_viz.max_cloud_points")},
            {"safety_viz.enable_im_bars": _param_bool(context, "safety_viz.enable_im_bars")},
            {"safety_viz.enable_validity_cloud": _param_bool(context, "safety_viz.enable_validity_cloud")},
            {"safety_viz.enable_p1_viz": _param_bool(context, "safety_viz.enable_p1_viz")},
            {"safety_viz.enable_p2_viz": _param_bool(context, "safety_viz.enable_p2_viz")},
            {"safety_viz.enable_p3_viz": _param_bool(context, "safety_viz.enable_p3_viz")},
            {"safety_viz.enable_p4_viz": _param_bool(context, "safety_viz.enable_p4_viz")},
        ],
    )


def _launch_setup(context):
    iap_share = get_package_share_directory("iap")
    so3_control_share = get_package_share_directory("so3_control")
    local_sensing_share = get_package_share_directory("local_sensing")
    scenario, experiment, preset_keys = _apply_presets(context, iap_share)
    safety_profile, safety_enabled, p0_enabled, p0_conflict, _ = _resolve_safety_switches(context, preset_keys)

    start_rviz = _as_bool(LaunchConfiguration("start_rviz").perform(context))
    record_bag = _as_bool(LaunchConfiguration("record_bag").perform(context))
    run_validator = _as_bool(LaunchConfiguration("run_validator").perform(context))
    start_planner = _as_bool(LaunchConfiguration("start_planner").perform(context))
    use_gnss = _as_bool(LaunchConfiguration("use_gnss").perform(context))
    use_araim = _as_bool(LaunchConfiguration("use_araim").perform(context))
    allow_truth_alignment = _as_bool(LaunchConfiguration("allow_truth_alignment").perform(context))
    enable_preflight_takeoff = _as_bool(LaunchConfiguration("enable_preflight_takeoff").perform(context))

    drone_id = LaunchConfiguration("drone_id").perform(context)
    init_x = float(LaunchConfiguration("init_x").perform(context))
    init_y = float(LaunchConfiguration("init_y").perform(context))
    init_z = float(LaunchConfiguration("init_z").perform(context))
    goal = (
        LaunchConfiguration("goal_x").perform(context),
        LaunchConfiguration("goal_y").perform(context),
        LaunchConfiguration("goal_z").perform(context),
    )
    point_num = LaunchConfiguration("point_num").perform(context)
    map_size = (
        float(LaunchConfiguration("map_size_x").perform(context)),
        float(LaunchConfiguration("map_size_y").perform(context)),
        float(LaunchConfiguration("map_size_z").perform(context)),
    )
    preflight_ground_z = float(LaunchConfiguration("preflight_ground_z").perform(context))
    plant_init_z = preflight_ground_z if enable_preflight_takeoff else init_z
    run_duration_s = float(LaunchConfiguration("run_duration_s").perform(context))
    validation_duration_s = float(LaunchConfiguration("validation_duration_s").perform(context))
    fixed_status_text = _as_bool(LaunchConfiguration("viz_status_text_use_fixed_position").perform(context))
    status_text_position = (
        float(LaunchConfiguration("viz_status_text_x").perform(context)),
        float(LaunchConfiguration("viz_status_text_y").perform(context)),
        float(LaunchConfiguration("viz_status_text_z").perform(context)),
    )

    gnss_ephemeris_source = LaunchConfiguration("gnss_ephemeris_source").perform(context)
    gnss_time_source = LaunchConfiguration("gnss_time_source").perform(context)
    gnss_enabled_constellations = LaunchConfiguration("gnss_enabled_constellations").perform(context)
    gnss_scenario_file = LaunchConfiguration("gnss_scenario_file").perform(context)
    gnss_rinex_nav_file = LaunchConfiguration("gnss_rinex_nav_file").perform(context)
    gnss_rinex_ephem_max_age_s = LaunchConfiguration("gnss_rinex_ephem_max_age_s").perform(context)
    gnss_fallback_to_synthetic = _as_bool(LaunchConfiguration("gnss_fallback_to_synthetic_on_rinex_error").perform(context))
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

    if use_gnss and gnss_ephemeris_source.strip().lower() == "rinex" and not gnss_fallback_to_synthetic and not Path(gnss_rinex_nav_file).expanduser().is_file():
        raise RuntimeError(
            "gnss_ephemeris_source:=rinex requires an existing gnss_rinex_nav_file "
            "when gnss_fallback_to_synthetic_on_rinex_error:=false; "
            f"got '{gnss_rinex_nav_file}'"
        )

    runtime_config_path, runtime_root, export_dir = _runtime_config(context, use_gnss, use_araim, allow_truth_alignment)
    gnss_scenario_file = _materialize_gnss_scenario(gnss_scenario_file, export_dir)

    bag_root_dir = LaunchConfiguration("bag_output_dir").perform(context).strip()
    if not bag_root_dir:
        bag_root_dir = str(Path(runtime_root) / "bag")
    bag_stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    bag_scenario = _safe_path_component(scenario, "manual")
    bag_experiment = _safe_path_component(experiment, "experiment")
    bag_output_dir = str(Path(bag_root_dir) / f"test_planner_{bag_experiment}_{bag_scenario}_{bag_stamp}")
    if record_bag:
        os.makedirs(bag_root_dir, exist_ok=True)
    evidence = _runtime_provenance(iap_share, export_dir, bag_output_dir, experiment, scenario)
    evidence["manifest_path"] = str((Path(export_dir) / "test_planner_manifest.json").resolve())
    formal_calibration = _formal_calibration_provenance(
        LaunchConfiguration("p1.formal_calibration_manifest").perform(context)
    )

    truth_odom_topic = "/sim/drone_0/truth_odom"
    iap_odom_topic = "/drone_0_visual_slam/odom"
    planner_odom_topic = truth_odom_topic
    sim_imu_topic = "/sim/drone_0/imu"
    iap_imu_topic = "/sim/drone_0/imu_iap"
    sim_lidar_topic = "/sim/drone_0/lidar"
    iap_lidar_topic = "/sim/drone_0/lidar_body"
    sim_depth_topic = "/sim/drone_0/depth"
    pos_cmd_topic = "/drone_0_planning/pos_cmd"
    desired_odom_topic = "/test_planner/desired/odom"
    bspline_topic = "/drone_0_planning/bspline"
    so3_cmd_topic = "/test_planner/so3_cmd"
    camera_pose_topic = "/drone_0_pcl_render_node/camera_pose"

    camera_file = os.path.join(local_sensing_share, "config", "camera.yaml")
    gains_file = os.path.join(so3_control_share, "config", "gains_hummingbird.yaml")
    corrections_file = os.path.join(so3_control_share, "config", "corrections_hummingbird.yaml")

    preflight_delay = 0.0
    if enable_preflight_takeoff:
        preflight_delay = (
            float(LaunchConfiguration("preflight_ground_hold_s").perform(context))
            + float(LaunchConfiguration("preflight_takeoff_duration_s").perform(context))
            + float(LaunchConfiguration("preflight_hover_s").perform(context))
        )
    planner_start_delay_s = preflight_delay + max(0.0, float(LaunchConfiguration("planner_start_delay_s").perform(context)))
    simulator_hold_until_cmd = planner_start_delay_s > 0.0

    planner_nodes = [
        _ego_planner_node(
            context,
            drone_id,
            planner_odom_topic,
            "/map_generator/global_cloud",
            camera_pose_topic,
            sim_depth_topic,
            bspline_topic,
            map_size,
            goal,
            point_num,
            safety_profile,
            safety_enabled,
            p0_enabled,
            export_dir,
            evidence,
        ),
        Node(
            package="ego_planner",
            executable="traj_server",
            name=f"drone_{drone_id}_traj_server",
            output="screen",
            remappings=[
                ("planning/bspline", bspline_topic),
                ("odometry", planner_odom_topic),
                ("position_cmd", pos_cmd_topic),
                ("/position_cmd", pos_cmd_topic),
            ],
            parameters=[{"traj_server/time_forward": 1.0}],
        ),
    ]
    if not start_planner:
        planner_actions = [LogInfo(msg="[test_planner] EGO planner/traj_server disabled by start_planner:=false")]
    elif planner_start_delay_s > 0.0:
        planner_actions = [
            LogInfo(msg=f"[test_planner] delaying EGO planner/traj_server by {planner_start_delay_s:.2f}s"),
            TimerAction(period=planner_start_delay_s, actions=planner_nodes),
        ]
    else:
        planner_actions = planner_nodes

    p5_3_fixture_hpl = _param_float(context, "p5_3.fixture.hpl_pred_m")
    p5_3_fixture_vpl = _param_float(context, "p5_3.fixture.vpl_pred_m")
    p5_4_fixture_hpl = _param_float(context, "p5_4.fixture.hpl_pred_m")
    p5_4_fixture_vpl = _param_float(context, "p5_4.fixture.vpl_pred_m")
    p5_7_fixture_hpl = _param_float(context, "p5_7.fixture.hpl_pred_m")
    p5_7_fixture_vpl = _param_float(context, "p5_7.fixture.vpl_pred_m")
    p5_pred_al_mode = LaunchConfiguration("p5.pred_alert_limit_mode").perform(context)
    if p5_pred_al_mode == "config_constant":
        p5_expected_hal = _param_float(context, "p5.pred_alert_limit_constant_hal_m")
        p5_expected_val = _param_float(context, "p5.pred_alert_limit_constant_val_m")
    elif p5_pred_al_mode == "current_msg_constant":
        p5_expected_hal = 10.0
        p5_expected_val = 10.0
    else:
        p5_expected_hal = None
        p5_expected_val = None
    p5_3_expected_hal = p5_expected_hal
    p5_3_expected_val = p5_expected_val
    p5_3_expected_im = (
        min(p5_3_expected_hal - p5_3_fixture_hpl, p5_3_expected_val - p5_3_fixture_vpl)
        if p5_3_expected_hal is not None and p5_3_expected_val is not None
        else None
    )
    p5_4_expected_hal = p5_expected_hal
    p5_4_expected_val = p5_expected_val
    p5_4_expected_im = (
        min(p5_4_expected_hal - p5_4_fixture_hpl, p5_4_expected_val - p5_4_fixture_vpl)
        if p5_4_expected_hal is not None and p5_4_expected_val is not None
        else None
    )
    p5_7_expected_hal = p5_expected_hal
    p5_7_expected_val = p5_expected_val
    p5_7_expected_im = (
        min(p5_7_expected_hal - p5_7_fixture_hpl, p5_7_expected_val - p5_7_fixture_vpl)
        if p5_7_expected_hal is not None and p5_7_expected_val is not None
        else None
    )
    p5_6_fixture_requested = _param_bool(context, "p5_6.fixture.enabled")
    p5_6_fixture_effective = _p5_6_fixture_effective_enabled(context)
    overrides = _launch_arg_overrides()
    p1_enabled_for_manifest = bool(safety_enabled.get("p1"))
    p1_use_for_manifest = (
        _param_bool(context, "p1.use_integrity_cost")
        if "p1.use_integrity_cost" in overrides
        else p1_enabled_for_manifest
    )
    p1_metrics_only_for_manifest = (
        _param_bool(context, "p1.metrics_only")
        if "p1.metrics_only" in overrides
        else (not p1_enabled_for_manifest)
    )
    p1_debug_path_for_manifest = LaunchConfiguration("p1.debug_csv_path").perform(context)
    if not p1_debug_path_for_manifest:
        p1_debug_path_for_manifest = str(Path(export_dir) / "planner_p1_integrity_cost_debug.csv")
    p1_accepted_profile_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_accepted_trajectory_risk_profile.csv")
    )
    p1_candidate_optimization_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_candidate_optimization.csv")
    )
    p1_candidate_control_points_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_candidate_control_points.csv")
    )
    p1_candidate_profile_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_candidate_profile.csv")
    )
    p1_candidate_pairwise_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_candidate_pairwise.csv")
    )
    p1_optimizer_checkpoint_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_optimizer_checkpoint.csv")
    )
    p0_occupancy_query_evidence_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p0_occupancy_query_evidence.csv")
    )
    p1_accepted_profile_context_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_accepted_trajectory_risk_profile_context.csv")
    )
    p1_planning_context_timeline_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_planning_context_timeline.csv")
    )
    p1_pre_admission_attempt_path_for_manifest = str(
        Path(p1_debug_path_for_manifest).with_name("planner_p1_pre_admission_attempt.csv")
    )
    p5_final_for_fixture = (
        _param_bool(context, "p5.enable_final_gate")
        if "p5.enable_final_gate" in overrides
        else bool(safety_enabled.get("p5_final"))
    )
    p5_7_fixture_requested = _param_bool(context, "p5_7.fixture.enabled")
    p5_7_fixture_effective = _p5_7_fixture_effective_enabled(
        context, p5_final_for_fixture
    )

    scenario_contract = {
        "geometry": {
            "fixture_algorithm_version": "p1_deterministic_fork_geometry_v1",
            "fixture": LaunchConfiguration("p1_map_fixture").perform(context),
            "mirror_y": _param_bool(context, "p1_fixture_mirror_y"),
            "start_m": [init_x, init_y, init_z],
            "goal_m": [float(goal[0]), float(goal[1]), float(goal[2])],
            "central_obstacle": {
                "enabled": _param_bool(context, "p1_fixture_central_obstacle_enabled"),
                "x_m": [_param_float(context, "p1_fixture_central_x_min_m"),
                        _param_float(context, "p1_fixture_central_x_max_m")],
                "y_half_width_m": _param_float(context, "p1_fixture_central_y_half_width_m"),
                "z_m": [0.0, _param_float(context, "p1_fixture_central_z_max_m")],
            },
            "lanes": {
                "center_abs_y_m": _param_float(context, "p1_fixture_lane_center_m"),
                "half_width_m": _param_float(context, "p1_fixture_lane_half_width_m"),
                "safe_tree_density_per_m2": _param_float(context, "p1_fixture_safe_tree_density_per_m2"),
                "risky_tree_density_per_m2": _param_float(context, "p1_fixture_risky_tree_density_per_m2"),
                "safe_canopy_probability": _param_float(context, "p1_fixture_safe_canopy_probability"),
                "risky_canopy_probability": _param_float(context, "p1_fixture_risky_canopy_probability"),
            },
            "forest_seed": _param_int(context, "forest_random_seed"),
            "map_resolution_m": _param_float(context, "corridor_map_resolution_m"),
            "trunk_radius_m": _param_float(context, "trunk_radius_m"),
            "canopy_resolution_m": _param_float(context, "canopy_resolution_m"),
            "fixture_short_trunk_height_m": 1.05,
            "fixture_risky_trunk_height_m": 2.85,
            "fixture_canopy_base_z_m": 2.85,
            "fixture_canopy_ball_center_z_m": 3.25,
            "fixture_canopy_ball_radius_m": 0.42,
            "fixture_canopy_clip_radius_m": 1.10,
            "fixture_lane_x_start_m": -7.8,
            "fixture_lane_x_span_m": 16.0,
            "fixture_lane_boundary_offset_m": 0.35,
            "fixture_lane_density_area_m2": 32.0,
            "fixture_soft_island_density_area_m2": 24.0,
            "fixture_soft_island_x_m": [-6.0, 2.0],
            "fixture_soft_island_center_y_m": 0.9,
            "fixture_soft_island_trunk_offset_y_m": 1.15,
            "terminal_wall_enabled": _param_bool(context, "terminal_wall_enabled"),
        },
        "risk_sources": ["gnss_map_occlusion", "lidar_observability"],
        "gnss": {
            "scenario_file": str(gnss_scenario_file),
            "scenario_file_sha256": _sha256_file(gnss_scenario_file),
            "random_seed": _param_int(context, "gnss_random_seed"),
            "ephemeris_source": gnss_ephemeris_source,
            "enabled_constellations": gnss_enabled_constellations,
            "pseudorange_noise_std_m": _param_float(context, "gnss_pr_noise_base"),
            "doppler_noise_std_mps": _param_float(context, "gnss_dop_noise_base"),
            "map_occlusion": _param_bool(context, "gnss_enable_map_occlusion"),
            "skymask": _param_bool(context, "gnss_enable_skymask"),
            "nlos": _param_bool(context, "gnss_enable_nlos"),
            "multipath": _param_bool(context, "gnss_enable_multipath"),
        },
        "p1": {
            "lambda_integrity": _param_float(context, "p1.lambda_integrity"),
            "normalization_budget_fraction": _param_float(context, "p1.normalization_budget_fraction"),
            "aggregation_mode": LaunchConfiguration("p1.objective_aggregation_mode").perform(context),
            "smooth_cvar_alpha": _param_float(context, "p1.smooth_cvar_alpha"),
            "smooth_max_temperature": _param_float(context, "p1.smooth_max_temperature"),
        },
        "planner_dynamics": {
            "manager_max_velocity_mps": _param_float(context, "manager/max_vel"),
            "optimizer_max_velocity_mps": _param_float(context, "optimization/max_vel"),
            "bspline_limit_velocity_mps": _param_float(context, "bspline/limit_vel"),
            "replan_period_s": _param_float(context, "fsm.thresh_replan_time"),
            "local_update_range_x_m": _param_float(context, "grid_map/local_update_range_x"),
        },
        "p0_prediction": {
            "horizons_s": _csv_floats(
                LaunchConfiguration("p0.horizons_s").perform(context)
            ),
        },
        "decision_checkpoint": {
            "truth_x_m": -9.5, "truth_x_tolerance_m": 0.4,
            "truth_source_topic": truth_odom_topic,
            "profile_sample_zero_binding": "planner_truth_odom_state_at_planning_start",
            "localization_error_limit_m": 0.5,
            "pair_error_delta_limit_m": 0.25,
        },
    }
    scenario_fingerprint = _scenario_fingerprint(scenario, scenario_contract)

    manifest = {
        "artifact_provenance": evidence,
        "experiment": experiment,
        "scenario": scenario,
        "scenario_contract": scenario_contract,
        "scenario_fingerprint": scenario_fingerprint,
        "runtime_config_path": runtime_config_path,
        "export_dir": export_dir,
        "run_duration_s": run_duration_s,
        "validation_duration_s": validation_duration_s,
        "planner_start_delay_s": max(
            0.0, float(LaunchConfiguration("planner_start_delay_s").perform(context))
        ),
        "manager/max_vel": _param_float(context, "manager/max_vel"),
        "optimization/max_vel": _param_float(context, "optimization/max_vel"),
        "bspline/limit_vel": _param_float(context, "bspline/limit_vel"),
        "fsm.thresh_replan_time": _param_float(context, "fsm.thresh_replan_time"),
        "grid_map/local_update_range_x": _param_float(context, "grid_map/local_update_range_x"),
        "record_bag": record_bag,
        "run_validator": run_validator,
        "timebase": {
            "planning_timeline": {"domain": "sim_message", "field": "stamp_s"},
            "p0_health_payload": {
                "domain": "sim_message",
                "field": "health_callback_stamp_s",
            },
            "bag_receive": {"domain": "system_receive", "field": "stamp"},
        },
        "planner_safety_profile": safety_profile,
        "fsm.thresh_no_replan_meter": _fixed_lattice_no_replan_threshold(safety_enabled),
        "p0.enable_risk_grid": p0_enabled,
        "p1.use_integrity_cost": p1_use_for_manifest,
        "p1.metrics_only": p1_metrics_only_for_manifest,
        "p1.lambda_integrity": _param_float(context, "p1.lambda_integrity"),
        "p1.debug_csv_path": p1_debug_path_for_manifest,
        "p1.max_candidates_per_attempt": max(1, min(8, _param_int(context, "p1.max_candidates_per_attempt"))),
        "p1.candidate_optimization_path": p1_candidate_optimization_path_for_manifest,
        "p1.candidate_control_points_path": p1_candidate_control_points_path_for_manifest,
        "p1.candidate_profile_path": p1_candidate_profile_path_for_manifest,
        "p1.candidate_pairwise_path": p1_candidate_pairwise_path_for_manifest,
        "p1.optimizer_checkpoint_path": p1_optimizer_checkpoint_path_for_manifest,
        "p0.occupancy_query_evidence_path": p0_occupancy_query_evidence_path_for_manifest,
        "p1.accepted_profile_path": p1_accepted_profile_path_for_manifest,
        "p1.accepted_profile_context_path": p1_accepted_profile_context_path_for_manifest,
        "p1.planning_context_timeline_path": p1_planning_context_timeline_path_for_manifest,
        "p1.pre_admission_attempt_path": p1_pre_admission_attempt_path_for_manifest,
        "p1.candidate_route_precheck_path": str(
            Path(export_dir) / "metadata" / "p1_candidate_route_precheck.json"),
        "p1.replacement_decision_path": str(
            Path(p1_debug_path_for_manifest).with_name("planner_p1_replacement_decision.csv")),
        "p1.candidate_retained_profile_path": str(
            Path(p1_debug_path_for_manifest).with_name("planner_p1_candidate_retained_profile.csv")),
        "p1.objective_aggregation_mode": LaunchConfiguration("p1.objective_aggregation_mode").perform(context),
        "p1.smooth_max_temperature": _param_float(context, "p1.smooth_max_temperature"),
        "p1.smooth_cvar_alpha": _param_float(context, "p1.smooth_cvar_alpha"),
        "p1.normalization_budget_fraction": _param_float(context, "p1.normalization_budget_fraction"),
        "p1.formal_calibration": formal_calibration,
        "p1.reference_identity": "metrics_only_lambda_0.00001_not_applied",
        "p0.raw_health_topic": "/planning/risk_grid_health",
        "p0.resolution_m": _param_float(context, "p0.resolution_m"),
        "p0.size_x_m": _param_float(context, "p0.size_x_m"),
        "p0.size_y_m": _param_float(context, "p0.size_y_m"),
        "p0.size_z_m": _param_float(context, "p0.size_z_m"),
        "p0.horizons_s": _csv_floats(LaunchConfiguration("p0.horizons_s").perform(context)),
        "p0.refresh_period_s": _param_float(context, "p0.refresh_period_s"),
        "p0.stale_timeout_s": _param_float(context, "p0.stale_timeout_s"),
        "p0.batch_worker_count": 1,
        "p0.predictor.requested_worker_count": _param_int(context, "p0.predictor.worker_count"),
        "p0.predictor.effective_worker_count": _param_int(context, "p0.predictor.worker_count"),
        "p0.skip_occupied_voxels": _param_bool(context, "p0.skip_occupied_voxels"),
        "p0.predictor.source_mode": LaunchConfiguration("p0.predictor.source_mode").perform(context),
        "p0.predictor.gnss_epoch_policy": LaunchConfiguration("p0.predictor.gnss_epoch_policy").perform(context),
        "p0.predictor.use_current_integrity_prior": _param_bool(context, "p0.predictor.use_current_integrity_prior"),
        "p0.predictor.conservative_max_with_gnss": _param_bool(context, "p0.predictor.conservative_max_with_gnss"),
        "p0.predictor.lidar_legacy_observability": _param_bool(context, "p0.predictor.lidar_legacy_observability"),
        "p0.predictor.lidar_fim_radius_m": _param_float(context, "p0.predictor.lidar_fim_radius_m"),
        "p0_6.fixture.enabled": _param_bool(context, "p0_6.fixture.enabled"),
        "p0_6.fixture.name": LaunchConfiguration("p0_6.fixture.name").perform(context),
        "p0_6.fixture.x_min": _param_float(context, "p0_6.fixture.x_min"),
        "p0_6.fixture.x_max": _param_float(context, "p0_6.fixture.x_max"),
        "p0_6.fixture.y_min": _param_float(context, "p0_6.fixture.y_min"),
        "p0_6.fixture.y_max": _param_float(context, "p0_6.fixture.y_max"),
        "p0_6.fixture.z_min": _param_float(context, "p0_6.fixture.z_min"),
        "p0_6.fixture.z_max": _param_float(context, "p0_6.fixture.z_max"),
        "p0_6.fixture.raw_hpl_m": _param_float(context, "p0_6.fixture.raw_hpl_m"),
        "p0_6.fixture.raw_vpl_m": _param_float(context, "p0_6.fixture.raw_vpl_m"),
        "p0_6.fixture.raw_c_pi": _param_float(context, "p0_6.fixture.raw_c_pi"),
        "p0_6.fixture.low_raw_cost_threshold": _param_float(context, "p0_6.fixture.low_raw_cost_threshold"),
        "p5_3.fixture.enabled": _param_bool(context, "p5_3.fixture.enabled"),
        "p5_3.fixture.name": LaunchConfiguration("p5_3.fixture.name").perform(context),
        "p5_3.fixture.x_min": _param_float(context, "p5_3.fixture.x_min"),
        "p5_3.fixture.x_max": _param_float(context, "p5_3.fixture.x_max"),
        "p5_3.fixture.y_min": _param_float(context, "p5_3.fixture.y_min"),
        "p5_3.fixture.y_max": _param_float(context, "p5_3.fixture.y_max"),
        "p5_3.fixture.z_min": _param_float(context, "p5_3.fixture.z_min"),
        "p5_3.fixture.z_max": _param_float(context, "p5_3.fixture.z_max"),
        "p5_3.fixture.tau_min": _param_float(context, "p5_3.fixture.tau_min"),
        "p5_3.fixture.tau_max": _param_float(context, "p5_3.fixture.tau_max"),
        "p5_3.fixture.hpl_pred_m": p5_3_fixture_hpl,
        "p5_3.fixture.vpl_pred_m": p5_3_fixture_vpl,
        "p5_3.fixture.expected_hal_m": p5_3_expected_hal,
        "p5_3.fixture.expected_val_m": p5_3_expected_val,
        "p5_3.fixture.expected_im_m": p5_3_expected_im,
        "p5_4.fixture.enabled": _param_bool(context, "p5_4.fixture.enabled"),
        "p5_4.fixture.name": LaunchConfiguration("p5_4.fixture.name").perform(context),
        "p5_4.fixture.x_min": _param_float(context, "p5_4.fixture.x_min"),
        "p5_4.fixture.x_max": _param_float(context, "p5_4.fixture.x_max"),
        "p5_4.fixture.y_min": _param_float(context, "p5_4.fixture.y_min"),
        "p5_4.fixture.y_max": _param_float(context, "p5_4.fixture.y_max"),
        "p5_4.fixture.z_min": _param_float(context, "p5_4.fixture.z_min"),
        "p5_4.fixture.z_max": _param_float(context, "p5_4.fixture.z_max"),
        "p5_4.fixture.tau_min": _param_float(context, "p5_4.fixture.tau_min"),
        "p5_4.fixture.tau_max": _param_float(context, "p5_4.fixture.tau_max"),
        "p5_4.fixture.hpl_pred_m": p5_4_fixture_hpl,
        "p5_4.fixture.vpl_pred_m": p5_4_fixture_vpl,
        "p5_4.fixture.expected_hal_m": p5_4_expected_hal,
        "p5_4.fixture.expected_val_m": p5_4_expected_val,
        "p5_4.fixture.expected_im_m": p5_4_expected_im,
        "p5_5.fixture.enabled": _param_bool(context, "p5_5.fixture.enabled"),
        "p5_5.fixture.name": LaunchConfiguration("p5_5.fixture.name").perform(context),
        "p5_5.fixture.start_s": _param_float(context, "p5_5.fixture.start_s"),
        "p5_5.fixture.duration_s": _param_float(context, "p5_5.fixture.duration_s"),
        "p5_6.fixture.enabled": p5_6_fixture_requested,
        "p5_6.fixture.effective_enabled": p5_6_fixture_effective,
        "p5_6.fixture.name": LaunchConfiguration("p5_6.fixture.name").perform(context),
        "p5_6.fixture.x_min": _param_float(context, "p5_6.fixture.x_min"),
        "p5_6.fixture.x_max": _param_float(context, "p5_6.fixture.x_max"),
        "p5_6.fixture.y_min": _param_float(context, "p5_6.fixture.y_min"),
        "p5_6.fixture.y_max": _param_float(context, "p5_6.fixture.y_max"),
        "p5_6.fixture.z_min": _param_float(context, "p5_6.fixture.z_min"),
        "p5_6.fixture.z_max": _param_float(context, "p5_6.fixture.z_max"),
        "p5_6.fixture.tau_min": _param_float(context, "p5_6.fixture.tau_min"),
        "p5_6.fixture.tau_max": _param_float(context, "p5_6.fixture.tau_max"),
        "p5_7.fixture.enabled": p5_7_fixture_requested,
        "p5_7.fixture.effective_enabled": p5_7_fixture_effective,
        "p5_7.fixture.name": LaunchConfiguration("p5_7.fixture.name").perform(context),
        "p5_7.fixture.x_min": _param_float(context, "p5_7.fixture.x_min"),
        "p5_7.fixture.x_max": _param_float(context, "p5_7.fixture.x_max"),
        "p5_7.fixture.y_min": _param_float(context, "p5_7.fixture.y_min"),
        "p5_7.fixture.y_max": _param_float(context, "p5_7.fixture.y_max"),
        "p5_7.fixture.z_min": _param_float(context, "p5_7.fixture.z_min"),
        "p5_7.fixture.z_max": _param_float(context, "p5_7.fixture.z_max"),
        "p5_7.fixture.tau_min": _param_float(context, "p5_7.fixture.tau_min"),
        "p5_7.fixture.tau_max": _param_float(context, "p5_7.fixture.tau_max"),
        "p5_7.fixture.hpl_pred_m": p5_7_fixture_hpl,
        "p5_7.fixture.vpl_pred_m": p5_7_fixture_vpl,
        "p5_7.fixture.expected_hal_m": p5_7_expected_hal,
        "p5_7.fixture.expected_val_m": p5_7_expected_val,
        "p5_7.fixture.expected_im_m": p5_7_expected_im,
        "p5.current_stale_to_replan_s": _param_float(context, "p5.current_stale_to_replan_s"),
        "p5.current_stale_to_emergency_s": _param_float(context, "p5.current_stale_to_emergency_s"),
        "p5.future_unknown_to_emergency_s": _param_float(context, "p5.future_unknown_to_emergency_s"),
        "p5.pred_alert_limit_mode": p5_pred_al_mode,
        "p5.future_replan_margin_m": _param_float(context, "p5.future_replan_margin_m"),
        "p5.future_emergency_margin_m": _param_float(context, "p5.future_emergency_margin_m"),
        "p5.max_bad_ratio": _param_float(context, "p5.max_bad_ratio"),
        "p0_6": {
            "fixture": {
                "enabled": _param_bool(context, "p0_6.fixture.enabled"),
                "name": LaunchConfiguration("p0_6.fixture.name").perform(context),
                "bounds": {
                    "x": [
                        _param_float(context, "p0_6.fixture.x_min"),
                        _param_float(context, "p0_6.fixture.x_max"),
                    ],
                    "y": [
                        _param_float(context, "p0_6.fixture.y_min"),
                        _param_float(context, "p0_6.fixture.y_max"),
                    ],
                    "z": [
                        _param_float(context, "p0_6.fixture.z_min"),
                        _param_float(context, "p0_6.fixture.z_max"),
                    ],
                },
                "expected_raw": {
                    "raw_hpl_m": _param_float(context, "p0_6.fixture.raw_hpl_m"),
                    "raw_vpl_m": _param_float(context, "p0_6.fixture.raw_vpl_m"),
                    "raw_c_pi": _param_float(context, "p0_6.fixture.raw_c_pi"),
                    "low_raw_cost_threshold": _param_float(context, "p0_6.fixture.low_raw_cost_threshold"),
                },
            },
        },
        "p5_3": {
            "fixture": {
                "enabled": _param_bool(context, "p5_3.fixture.enabled"),
                "name": LaunchConfiguration("p5_3.fixture.name").perform(context),
                "bounds": {
                    "x": [
                        _param_float(context, "p5_3.fixture.x_min"),
                        _param_float(context, "p5_3.fixture.x_max"),
                    ],
                    "y": [
                        _param_float(context, "p5_3.fixture.y_min"),
                        _param_float(context, "p5_3.fixture.y_max"),
                    ],
                    "z": [
                        _param_float(context, "p5_3.fixture.z_min"),
                        _param_float(context, "p5_3.fixture.z_max"),
                    ],
                },
                "tau_window_s": [
                    _param_float(context, "p5_3.fixture.tau_min"),
                    _param_float(context, "p5_3.fixture.tau_max"),
                ],
                "injected_pl_m": {
                    "hpl_pred": p5_3_fixture_hpl,
                    "vpl_pred": p5_3_fixture_vpl,
                },
                "expected_alert_limit_m": {
                    "mode": p5_pred_al_mode,
                    "hal": p5_3_expected_hal,
                    "val": p5_3_expected_val,
                },
                "expected_im_m": p5_3_expected_im,
                "expected_reason": "p5_3_high_risk_zone",
            },
        },
        "p5_4": {
            "fixture": {
                "enabled": _param_bool(context, "p5_4.fixture.enabled"),
                "name": LaunchConfiguration("p5_4.fixture.name").perform(context),
                "bounds": {
                    "x": [
                        _param_float(context, "p5_4.fixture.x_min"),
                        _param_float(context, "p5_4.fixture.x_max"),
                    ],
                    "y": [
                        _param_float(context, "p5_4.fixture.y_min"),
                        _param_float(context, "p5_4.fixture.y_max"),
                    ],
                    "z": [
                        _param_float(context, "p5_4.fixture.z_min"),
                        _param_float(context, "p5_4.fixture.z_max"),
                    ],
                },
                "tau_window_s": [
                    _param_float(context, "p5_4.fixture.tau_min"),
                    _param_float(context, "p5_4.fixture.tau_max"),
                ],
                "injected_pl_m": {
                    "hpl_pred": p5_4_fixture_hpl,
                    "vpl_pred": p5_4_fixture_vpl,
                },
                "expected_alert_limit_m": {
                    "mode": p5_pred_al_mode,
                    "hal": p5_4_expected_hal,
                    "val": p5_4_expected_val,
                },
                "expected_im_m": p5_4_expected_im,
                "expected_reason": "p5_4_near_risk_zone",
                "expected_first_bad_tau_s": _param_float(context, "p5_4.fixture.tau_min"),
                "expected_emergency_time_s": 1.0,
            },
        },
        "p5_5": {
            "fixture": {
                "enabled": _param_bool(context, "p5_5.fixture.enabled"),
                "name": LaunchConfiguration("p5_5.fixture.name").perform(context),
                "window_s": [
                    _param_float(context, "p5_5.fixture.start_s"),
                    _param_float(context, "p5_5.fixture.start_s")
                    + _param_float(context, "p5_5.fixture.duration_s"),
                ],
                "start_s": _param_float(context, "p5_5.fixture.start_s"),
                "duration_s": _param_float(context, "p5_5.fixture.duration_s"),
                "expected_thresholds_s": {
                    "replan": _param_float(context, "p5.current_stale_to_replan_s"),
                    "emergency": _param_float(context, "p5.current_stale_to_emergency_s"),
                },
                "expected_reason": "current_stale",
                "route": {
                    "enabled": _param_bool(context, "p5_5.fixture.enabled"),
                    "waypoints": [list(waypoint) for waypoint in P5_5_FIXTURE_ROUTE_WAYPOINTS],
                    "reason": "keep P5 runtime gate active through the stale fixture window",
                },
            },
        },
        "p5_6": {
            "fixture": {
                "enabled": p5_6_fixture_requested,
                "effective_enabled": p5_6_fixture_effective,
                "name": LaunchConfiguration("p5_6.fixture.name").perform(context),
                "bounds": {
                    "x": [
                        _param_float(context, "p5_6.fixture.x_min"),
                        _param_float(context, "p5_6.fixture.x_max"),
                    ],
                    "y": [
                        _param_float(context, "p5_6.fixture.y_min"),
                        _param_float(context, "p5_6.fixture.y_max"),
                    ],
                    "z": [
                        _param_float(context, "p5_6.fixture.z_min"),
                        _param_float(context, "p5_6.fixture.z_max"),
                    ],
                },
                "tau_window_s": [
                    _param_float(context, "p5_6.fixture.tau_min"),
                    _param_float(context, "p5_6.fixture.tau_max"),
                ],
                "expected_reason": "future_unknown",
                "expected_unknown": {
                    "available": True,
                    "valid": False,
                    "stale": False,
                    "finite_pl": False,
                },
            },
        },
        "p5_7": {
            "fixture": {
                "enabled": p5_7_fixture_requested,
                "effective_enabled": p5_7_fixture_effective,
                "name": LaunchConfiguration("p5_7.fixture.name").perform(context),
                "bounds": {
                    "x": [
                        _param_float(context, "p5_7.fixture.x_min"),
                        _param_float(context, "p5_7.fixture.x_max"),
                    ],
                    "y": [
                        _param_float(context, "p5_7.fixture.y_min"),
                        _param_float(context, "p5_7.fixture.y_max"),
                    ],
                    "z": [
                        _param_float(context, "p5_7.fixture.z_min"),
                        _param_float(context, "p5_7.fixture.z_max"),
                    ],
                },
                "tau_window_s": [
                    _param_float(context, "p5_7.fixture.tau_min"),
                    _param_float(context, "p5_7.fixture.tau_max"),
                ],
                "injected_pl_m": {
                    "hpl_pred": p5_7_fixture_hpl,
                    "vpl_pred": p5_7_fixture_vpl,
                },
                "expected_alert_limit_m": {
                    "mode": p5_pred_al_mode,
                    "hal": p5_7_expected_hal,
                    "val": p5_7_expected_val,
                },
                "expected_im_m": p5_7_expected_im,
                "expected_reason": "p5_7_rejected_trajectory",
                "sample_source": "final_candidate",
            },
        },
        **{f"planner_enable_{key}": value for key, value in safety_enabled.items()},
    }
    manifest_path = Path(export_dir) / "test_planner_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    actions = [
        LogInfo(msg="[test_planner] self-contained planner closed-loop demo"),
        LogInfo(msg=f"[test_planner] experiment: {experiment}"),
        LogInfo(msg=f"[test_planner] scenario: {scenario}"),
        LogInfo(msg=f"[test_planner] runtime IAP config: {runtime_config_path}"),
        LogInfo(msg=f"[test_planner] export dir: {export_dir}"),
        LogInfo(msg=f"[test_planner] evidence run_id: {evidence['run_id']} schema: {evidence['schema_version']}"),
        LogInfo(msg=f"[test_planner] planner_safety_profile: {safety_profile}"),
        LogInfo(msg=f"[test_planner] safety switches: {safety_enabled}, p0={p0_enabled}"),
        LogInfo(msg=f"[test_planner] GNSS scenario: {gnss_scenario_file}"),
        LogInfo(msg=f"[test_planner] rosbag output: {bag_output_dir}"),
    ]
    if p0_conflict:
        actions.append(LogInfo(msg="[test_planner] WARNING: safety feature enabled but p0.enable_risk_grid:=false was explicit; fallback paths will be used"))

    actions.extend([
        Node(
            package="iap",
            executable="demo11_corridor_map_publisher",
            name="test_planner_corridor_map_publisher",
            output="screen",
            parameters=[
                {"resolution_m": _param_float(context, "corridor_map_resolution_m")},
                {"publish_rate_hz": _param_float(context, "corridor_map_publish_rate_hz")},
                {"frame_id": "map"},
                {"forest_size_x_m": _param_float(context, "forest_size_x_m")},
                {"forest_size_y_m": _param_float(context, "forest_size_y_m")},
                {"tree_density_lower_left_per_m2": _param_float(context, "tree_density_lower_left_per_m2")},
                {"tree_density_lower_right_per_m2": _param_float(context, "tree_density_lower_right_per_m2")},
                {"tree_density_upper_left_per_m2": _param_float(context, "tree_density_upper_left_per_m2")},
                {"tree_density_upper_right_per_m2": _param_float(context, "tree_density_upper_right_per_m2")},
                {"stratified_cell_size_m": _param_float(context, "stratified_cell_size_m")},
                {"clear_corridor_enabled": _param_bool(context, "clear_corridor_enabled")},
                {"clear_corridor_center_y_m": _param_float(context, "clear_corridor_center_y_m")},
                {"clear_corridor_half_width_y_m": _param_float(context, "clear_corridor_half_width_y_m")},
                {"clear_corridor_x_min_m": _param_float(context, "clear_corridor_x_min_m")},
                {"clear_corridor_x_max_m": _param_float(context, "clear_corridor_x_max_m")},
                {"canopy_density_lower_left": _param_float(context, "canopy_density_lower_left")},
                {"canopy_density_lower_right": _param_float(context, "canopy_density_lower_right")},
                {"canopy_density_upper_left": _param_float(context, "canopy_density_upper_left")},
                {"canopy_density_upper_right": _param_float(context, "canopy_density_upper_right")},
                {"canopy_hemisphere_radius_min_m": _param_float(context, "canopy_hemisphere_radius_min_m")},
                {"canopy_hemisphere_radius_max_m": _param_float(context, "canopy_hemisphere_radius_max_m")},
                {"canopy_leaf_ball_radius_m": _param_float(context, "canopy_leaf_ball_radius_m")},
                {"canopy_ball_spacing_ratio": _param_float(context, "canopy_ball_spacing_ratio")},
                {"canopy_resolution_m": _param_float(context, "canopy_resolution_m")},
                {"random_seed": _param_int(context, "forest_random_seed")},
                {"trunk_radius_m": _param_float(context, "trunk_radius_m")},
                {"trunk_min_height_m": _param_float(context, "trunk_min_height_m")},
                {"trunk_max_height_m": _param_float(context, "trunk_max_height_m")},
                {"terminal_wall_enabled": _param_bool(context, "terminal_wall_enabled")},
                {"terminal_wall_x_m": _param_float(context, "terminal_wall_x_m")},
                {"terminal_wall_y_m": _param_float(context, "terminal_wall_y_m")},
                {"terminal_wall_width_y_m": _param_float(context, "terminal_wall_width_y_m")},
                {"terminal_wall_z_min_m": _param_float(context, "terminal_wall_z_min_m")},
                {"terminal_wall_z_max_m": _param_float(context, "terminal_wall_z_max_m")},
                {"terminal_wall_thickness_x_m": _param_float(context, "terminal_wall_thickness_x_m")},
                {"terminal_wall_resolution_m": _param_float(context, "terminal_wall_resolution_m")},
                {"terminal_wall_feature_depth_x_m": _param_float(context, "terminal_wall_feature_depth_x_m")},
                {"terminal_wall_feature_count": _param_int(context, "terminal_wall_feature_count")},
                {"terminal_wall_feature_seed": _param_int(context, "terminal_wall_feature_seed")},
                {"corridor_walls_enabled": _param_bool(context, "corridor_walls_enabled")},
                {"corridor_floor_enabled": _param_bool(context, "corridor_floor_enabled")},
                {"corridor_x_min_m": _param_float(context, "corridor_x_min_m")},
                {"corridor_x_max_m": _param_float(context, "corridor_x_max_m")},
                {"corridor_half_width_y_m": _param_float(context, "corridor_half_width_y_m")},
                {"corridor_wall_z_min_m": _param_float(context, "corridor_wall_z_min_m")},
                {"corridor_wall_z_max_m": _param_float(context, "corridor_wall_z_max_m")},
                {"corridor_wall_thickness_y_m": _param_float(context, "corridor_wall_thickness_y_m")},
                {"corridor_floor_thickness_z_m": _param_float(context, "corridor_floor_thickness_z_m")},
                {"corridor_surface_resolution_m": _param_float(context, "corridor_surface_resolution_m")},
                {"p1_map_fixture": LaunchConfiguration("p1_map_fixture").perform(context)},
                {"p1_fixture_mirror_y": _param_bool(context, "p1_fixture_mirror_y")},
                {"p1_fixture_central_obstacle_enabled": _param_bool(context, "p1_fixture_central_obstacle_enabled")},
                {"p1_fixture_central_x_min_m": _param_float(context, "p1_fixture_central_x_min_m")},
                {"p1_fixture_central_x_max_m": _param_float(context, "p1_fixture_central_x_max_m")},
                {"p1_fixture_central_y_half_width_m": _param_float(context, "p1_fixture_central_y_half_width_m")},
                {"p1_fixture_central_z_max_m": _param_float(context, "p1_fixture_central_z_max_m")},
                {"p1_fixture_lane_center_m": _param_float(context, "p1_fixture_lane_center_m")},
                {"p1_fixture_lane_half_width_m": _param_float(context, "p1_fixture_lane_half_width_m")},
                {"p1_fixture_safe_tree_density_per_m2": _param_float(context, "p1_fixture_safe_tree_density_per_m2")},
                {"p1_fixture_risky_tree_density_per_m2": _param_float(context, "p1_fixture_risky_tree_density_per_m2")},
                {"p1_fixture_safe_canopy_probability": _param_float(context, "p1_fixture_safe_canopy_probability")},
                {"p1_fixture_risky_canopy_probability": _param_float(context, "p1_fixture_risky_canopy_probability")},
                {"p0_6.fixture.enabled": _param_bool(context, "p0_6.fixture.enabled")},
                {"p0_6.fixture.name": LaunchConfiguration("p0_6.fixture.name").perform(context)},
                {"p0_6.fixture.x_min": _param_float(context, "p0_6.fixture.x_min")},
                {"p0_6.fixture.x_max": _param_float(context, "p0_6.fixture.x_max")},
                {"p0_6.fixture.y_min": _param_float(context, "p0_6.fixture.y_min")},
                {"p0_6.fixture.y_max": _param_float(context, "p0_6.fixture.y_max")},
                {"p0_6.fixture.z_min": _param_float(context, "p0_6.fixture.z_min")},
                {"p0_6.fixture.z_max": _param_float(context, "p0_6.fixture.z_max")},
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
                {"sensing_rate": 10.0},
                {"estimation_rate": 15.0},
                {"map/x_size": map_size[0]},
                {"map/y_size": map_size[1]},
                {"map/z_size": map_size[2]},
                {"map/resolution": 0.1},
                camera_file,
            ],
        ),
        Node(
            package="iap",
            executable="demo4_lidar_body_bridge",
            name="test_planner_lidar_body_bridge",
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
            name="test_planner_iap_rosnode",
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
            name="test_planner_preflight_takeoff_cmd_publisher",
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
                {"ground_hold_duration_s": _param_float(context, "preflight_ground_hold_s")},
                {"takeoff_duration_s": _param_float(context, "preflight_takeoff_duration_s")},
                {"hover_duration_s": _param_float(context, "preflight_hover_s")},
                {"publish_rate_hz": _param_float(context, "preflight_cmd_rate_hz")},
                {"trajectory_id": 12001},
                {"frame_id": "map"},
                {"stop_after_sequence": True},
            ],
        ),
        Node(
            package="poscmd_2_odom",
            executable="poscmd_2_odom",
            name="test_planner_desired_poscmd_to_odom",
            output="screen",
            remappings=[("command", pos_cmd_topic), ("odometry", desired_odom_topic)],
            parameters=[{"init_x": init_x}, {"init_y": init_y}, {"init_z": plant_init_z}],
        ),
        _odom_visualization_node("test_planner_iap_odom_visualization", iap_odom_topic, pos_cmd_topic, "/test_planner/drone", (0.2, 1.0, 0.4), int(drone_id), fixed_status_text, status_text_position),
        _odom_visualization_node("test_planner_truth_odom_visualization", truth_odom_topic, pos_cmd_topic, "/test_planner/truth", (1.0, 0.15, 0.1), 10),
        _odom_visualization_node("test_planner_desired_odom_visualization", desired_odom_topic, pos_cmd_topic, "/test_planner/desired", (1.0, 0.86, 0.05), 20),
        Node(
            package="gnss_sim",
            executable="gnss_sim_node",
            name="test_planner_gnss_sim_node",
            output="screen",
            condition=IfCondition("true" if use_gnss else "false"),
            parameters=[
                {"truth_odom_topic": truth_odom_topic},
                {"origin_lat_deg": 31.2304},
                {"origin_lon_deg": 121.4737},
                {"origin_alt_m": 25.0},
                {"pseudorange_noise_std_m": _param_float(context, "gnss_pr_noise_base")},
                {"doppler_noise_std_mps": _param_float(context, "gnss_dop_noise_base")},
                {"random_seed": _param_int(context, "gnss_random_seed")},
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
                {"enable_visualization": _param_bool(context, "gnss_enable_visualization")},
                {"signal_ray_width_m": 0.025},
                {"signal_ray_alpha": 0.3},
                {"nlos_path_width_m": 0.04},
                {"nlos_path_alpha": 0.3},
                {"enable_sky_dome_visualization": _param_bool(context, "gnss_enable_sky_dome_visualization")},
                {"sky_dome_show_cardinal_labels": _param_bool(context, "gnss_sky_dome_show_cardinal_labels")},
                {"sky_dome_ring_count": 3},
                {"sky_dome_meridian_count": 12},
                {"sky_dome_follow_receiver": _param_bool(context, "gnss_sky_dome_follow_receiver")},
                {"sky_dome_center_enu": gnss_sky_dome_center_enu},
                {"skyplot_origin_enu": gnss_skyplot_origin_enu},
                {"status_text_use_fixed_position": fixed_status_text},
                {"status_text_position_enu": list(status_text_position)},
                {"enable_map_occlusion": _param_bool(context, "gnss_enable_map_occlusion")},
                {"enable_skymask": _param_bool(context, "gnss_enable_skymask")},
                {"enable_nlos": _param_bool(context, "gnss_enable_nlos")},
                {"enable_multipath": _param_bool(context, "gnss_enable_multipath")},
                {"enable_fault_injection": _param_bool(context, "gnss_enable_fault_injection")},
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
                ("force_disturbance", "/test_planner/force_disturbance"),
                ("moment_disturbance", "/test_planner/moment_disturbance"),
            ],
            parameters=[
                {"quadrotor_name": f"drone_{drone_id}"},
                {"rate/simulation": 1000.0},
                {"rate/odom": 100.0},
                {"simulator/init_state_x": init_x},
                {"simulator/init_state_y": init_y},
                {"simulator/init_state_z": plant_init_z},
                {"simulator/hold_until_cmd": simulator_hold_until_cmd},
                {"sim_time/enable": True},
                {"sim_time/start_utc": "2022-07-06T00:00:00Z"},
                {"iap_imu/enable": True},
                {"iap_imu/topic": iap_imu_topic},
            ],
        ),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
            name="test_planner_so3_control_container",
            namespace="",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="so3_control",
                    plugin="SO3ControlComponent",
                    name="test_planner_so3_control_component",
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
                        ("motors", "/test_planner/motors"),
                        ("corrections", "/test_planner/corrections"),
                        ("so3_cmd", so3_cmd_topic),
                        ("imu", iap_imu_topic),
                    ],
                )
            ],
        ),
        *planner_actions,
        Node(
            package="iap",
            executable="planner_evidence_provenance_publisher.py",
            name="test_planner_evidence_provenance",
            output="screen",
            parameters=[{"manifest_path": evidence["manifest_path"]}],
        ),
        Node(
            package="iap",
            executable="test_araim_validator.py",
            name="test_planner_integrity_validator",
            output="screen",
            condition=IfCondition("true" if run_validator else "false"),
            parameters=[
                {"integrity_topic": "/iap/integrity"},
                {"duration_s": validation_duration_s},
                {"min_messages": 10},
                {"csv_path": str(Path(export_dir) / "test_planner_integrity_validation.csv")},
                {"summary_path": str(Path(export_dir) / "test_planner_validation_summary.json")},
                {"schema_version": evidence["schema_version"]},
                {"run_id": evidence["run_id"]},
                {"manifest_path": evidence["manifest_path"]},
                {"required_fusion_mode": LaunchConfiguration("integrity_fusion_mode").perform(context)},
                {"require_gnss_valid": _param_bool(context, "validator_require_gnss_valid")},
                {"require_lidar_valid": _param_bool(context, "validator_require_lidar_valid")},
                {"require_fallback_valid": _param_bool(context, "validator_require_fallback_valid")},
                {"required_final_source": LaunchConfiguration("validator_required_final_source").perform(context)},
                {"allowed_final_sources_csv": LaunchConfiguration("validator_allowed_final_sources").perform(context)},
            ],
        ),
    ])

    if record_bag:
        bag_recorder = ExecuteProcess(
                cmd=[
                    sys.executable,
                    str(Path(get_package_prefix("iap")) / "lib" / "iap" /
                        "planner_bag_recorder_with_finalizer.py"),
                    "--manifest", evidence["manifest_path"],
                    "--bag-output", bag_output_dir,
                    "--",
                    "/iap/integrity",
                    "/gnss_sim/diagnostics",
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
                    # The planner and P0 both consult this exact inflated map.
                    # Record it so a strict-support rejection near the start can
                    # be checked against the source occupancy rather than
                    # inferred from a risk-profile reason alone.
                    f"/drone_{drone_id}_grid/grid_map/occupancy_inflate",
                    camera_pose_topic,
                    pos_cmd_topic,
                    bspline_topic,
                    so3_cmd_topic,
                    "/test_planner/truth/path",
                    "/test_planner/drone/path",
                    "/test_planner/desired/path",
                    "/test_planner/truth/robot",
                    "/test_planner/drone/robot",
                    "/test_planner/desired/robot",
                    "/ublox_driver/range_meas",
                    "/ublox_driver/ephem",
                    "/ublox_driver/glo_ephem",
                    "/ublox_driver/receiver_lla",
                    "/ublox_driver/iono_params",
                    "/tf",
                    "/planning/risk_grid_health",
                    "/planning/evidence_provenance",
                    "/planning/integrity_gate_status",
                    "/iap/rviz/risk_grid_health",
                    "/iap/rviz/predicted_pl_cloud",
                    "/iap/rviz/risk_validity_cloud",
                    "/iap/rviz/trajectory_integrity_samples",
                    "/iap/rviz/current_traj_integrity_colored",
                    "/iap/rviz/p5_gate_status",
                    "/iap/rviz/p5_current_im_bars",
                    "/iap/rviz/p1_integrity_samples",
                    "/iap/rviz/p1_integrity_push_vectors",
                    "/iap/rviz/p1_integrity_metrics",
                    "/iap/rviz/p2_candidate_trajectories",
                    "/iap/rviz/p3_reference_bias",
                    "/iap/rviz/p4_astar_guides",
                ],
                output="screen",
        )
        actions.append(bag_recorder)

    if start_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="test_planner_rviz",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(iap_share, "config", "sim_demo11", "demo11_integrity_corridor.rviz"),
                ],
            )
        )

    if run_duration_s > 0.0:
        actions.append(
            TimerAction(
                period=run_duration_s,
                actions=[EmitEvent(event=Shutdown(reason="test_planner run_duration_s elapsed"))],
            )
        )

    return actions


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    fastdds_profile = os.path.join(iap_share, "config", "sim_ego", "fastdds_udp_only.xml")
    return LaunchDescription(
        [
            *[DeclareLaunchArgument(name, default_value=default) for name, default in ARG_DEFAULTS],
            SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
            SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
            OpaqueFunction(function=_launch_setup),
        ]
    )
