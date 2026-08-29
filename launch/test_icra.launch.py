"""ICRA development entrypoint layered on the maintained planner launch."""

import importlib.util
from pathlib import Path


_BASE_PATH = Path(__file__).resolve().with_name("test_planner.launch.py")
_BASE_SPEC = importlib.util.spec_from_file_location(
    "iap_test_planner_launch_for_icra", _BASE_PATH
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
assert _BASE_SPEC.loader is not None
_BASE_SPEC.loader.exec_module(_BASE)


def _set_default(key, value):
    for index, (name, _) in enumerate(_BASE.ARG_DEFAULTS):
        if name == key:
            _BASE.ARG_DEFAULTS[index] = (key, value)
            return
    raise KeyError(f"unknown planner launch argument: {key}")


ICRA_DEV_FIXED_VALUES = {
    "odometry_initialization_mode": "NAIVE",
    "lidar_start_delay_s": "2.0",
    "planner_start_delay_s": "10.0",
    "planner_enable_p1": "false",
    "planner_enable_p2": "false",
    "planner_enable_p3_local": "false",
    "planner_enable_p3_global": "false",
}

_set_default("odometry_acc_scale", "1.0")
_set_default("planner_start_delay_s", "3.0")
_set_default("rviz_config", "config/sim_demo11/test_icra.rviz")
_set_default("planner_occupancy_cloud_topic", "/sim/drone_0/lidar")

_BASE.SCENARIO_PRESETS["icra072_p4_selection_trigger_v1"] = {
    **_BASE.SCENARIO_PRESETS["icra072_p4_selection_trigger_v1"],
    "lidar_sensing_rate_hz": "10.0",
}
_BASE.EXPERIMENT_PRESETS["icra_p0_p4_v2_p5_dev"] = {
    **_BASE.EXPERIMENT_PRESETS["icra_p0_p4_v2_p5_dev"],
    **ICRA_DEV_FIXED_VALUES,
    "grid_map/independent_cloud_min_interval_s": "0.5",
    "grid_map/independent_cloud_clock_guard_s": "0.5",
    "p0.refresh_start_delay_s": "0.05",
    "planner_occupancy_cloud_topic": "/sim/drone_0/lidar",
    "safety_viz.enable_p4_viz": "true",
}

_base_apply_presets = _BASE._apply_presets


def _apply_presets(context, iap_share):
    scenario, experiment, applied_keys = _base_apply_presets(context, iap_share)
    if experiment == "icra_p0_p4_v2_p5_dev":
        # Stage runners may independently switch P4/P5. Initialization,
        # startup ordering, and P1/P2/P3 isolation are fixed by this profile.
        for key, value in ICRA_DEV_FIXED_VALUES.items():
            context.launch_configurations[key] = value
            applied_keys.add(key)
    return scenario, experiment, applied_keys


_BASE._apply_presets = _apply_presets

# Export helpers and contracts used by the launch tests and developer tooling.
ARG_DEFAULTS = _BASE.ARG_DEFAULTS
SCENARIO_PRESETS = _BASE.SCENARIO_PRESETS
EXPERIMENT_PRESETS = _BASE.EXPERIMENT_PRESETS
_resolve_runtime_logging_roots = _BASE._resolve_runtime_logging_roots


def generate_launch_description():
    return _BASE.generate_launch_description()
