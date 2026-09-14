#!/usr/bin/env python3
"""Run and fail-closed analyze the development-only ICRA interface ladder."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Callable, TextIO

try:
    import psutil
except ImportError:  # pragma: no cover - launch package declares dependency
    psutil = None


REPOSITORY = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = (
    REPOSITORY / "results/icra27/dev_runs/interface_integration"
).resolve()
DEFAULT_INSTALL_ROOT = (REPOSITORY.parents[1] / "install").resolve()
STAGE_ORDER = ("estimator", "p0", "p4", "p5-final", "full", "shutdown")
STAGE_CHOICES = (*STAGE_ORDER, "limited-prefix")
DEFAULT_SCENARIO = "icra072_p4_selection_trigger_v1"
FOREST_V1_SCENARIO = "icra_dense_forest_four_fork_v1"
FOREST_SCENARIO = "icra_dense_forest_four_fork_v2"
P4_FORWARD_DECISION_SCHEMAS = {
    "p4_forward_route_decision_v1",
    "p4_forward_route_decision_v2",
    "p4_forward_route_decision_v3",
    "p4_forward_route_decision_v4",
    "p4_forward_route_decision_v5",
    "p4_forward_route_decision_v6",
    "p4_forward_route_decision_v7",
    "p4_forward_route_decision_v8",
}
P4_FORMAL_RISK_SAMPLE_SCHEMAS = {
    "p4_forward_route_decision_v5",
    "p4_forward_route_decision_v6",
    "p4_forward_route_decision_v7",
    "p4_forward_route_decision_v8",
}
FOREST_SCENARIOS = (FOREST_V1_SCENARIO, FOREST_SCENARIO)
SEVEN_STAGE_ORDER = (
    "p0_snapshot", "closed_collision", "p4_selection_application",
    "ego_final_bspline", "p5_final_pass_before_publish",
    "normal_publication", "p5_runtime_committed",
)
_VERTICAL_ANALYZER_PATH = (
    REPOSITORY / "scripts/dev_planner/analyze_icra072_vertical_slice.py")
_VERTICAL_ANALYZER_SPEC = importlib.util.spec_from_file_location(
    "icra072_vertical_slice_for_interface_runner", _VERTICAL_ANALYZER_PATH)
_VERTICAL_ANALYZER = importlib.util.module_from_spec(_VERTICAL_ANALYZER_SPEC)
assert _VERTICAL_ANALYZER_SPEC.loader is not None
_VERTICAL_ANALYZER_SPEC.loader.exec_module(_VERTICAL_ANALYZER)
FORBIDDEN_LAYER_ARGS = {
    "planner_enable_p1": "false",
    "planner_enable_p2": "false",
    "planner_enable_p3_local": "false",
    "planner_enable_p3_global": "false",
}
COMMON_ARGS = {
    "experiment": "icra_p0_p4_v2_p5_dev",
    "scenario": DEFAULT_SCENARIO,
    "odometry_acc_scale": "1.0",
    "odometry_initialization_mode": "NAIVE",
    "lidar_start_delay_s": "2.0",
    "planner_start_delay_s": "10.0",
    "record_bag": "false",
    "start_rviz": "false",
    "run_validator": "true",
    **FORBIDDEN_LAYER_ARGS,
}


class StageSpec:
    def __init__(self, duration_s: float, **launch_args: str) -> None:
        self.duration_s = duration_s
        self.launch_args = {**COMMON_ARGS, **launch_args}


STAGES = {
    "estimator": StageSpec(
        20.0,
        start_planner="false",
        planner_enable_p4="false",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
    ),
    "p0": StageSpec(
        35.0,
        start_planner="true",
        planner_enable_p4="false",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
    ),
    "p4": StageSpec(
        45.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "limited-prefix": StageSpec(
        90.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
        **{
            "safety_viz.enable_p4_viz": "true",
            "p4.debug_generation_probe_enable": "true",
        },
    ),
    "p5-final": StageSpec(
        60.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="false",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "full": StageSpec(
        75.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="true",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "shutdown": StageSpec(
        20.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="true",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
}


def _is_forest_scenario(scenario: str) -> bool:
    return scenario in FOREST_SCENARIOS


def forest_scene_contract(scenario: str = FOREST_SCENARIO) -> dict:
    """Return the frozen, expanded geometry used by the forest preset."""
    if not _is_forest_scenario(scenario):
        raise ValueError(f"unsupported forest scenario: {scenario}")
    online = scenario == FOREST_SCENARIO
    signs = [-1, 1, -1, 1]
    contract = {
        "schema_version": scenario,
        "forest_seed": 41021,
        "risk_seed": 21,
        "scene_bbox_m": {
            "min": [-21.0, -11.0, 0.0],
            "max": [21.0, 11.0, 8.0],
        },
        "start_xyz_m": [-18.0, 0.0, 1.5],
        "goal_xyz_m": [18.0, 0.0, 1.5],
        "low_risk_y_signs": signs,
        "corridor_width_m": 2.4,
        "junction_buffer_radius_m": 2.0,
        "start_canopy_clearance_radius_m": 5.0 if online else 2.0,
        "forest_density_per_m2": 0.25,
        "stratified_cell_size_m": 2.0,
        "canopy_probability": 0.65,
        "canopy_radius_range_m": [0.8, 1.5],
        "trunk_radius_m": 0.14,
        "trunk_height_range_m": [3.2, 5.0],
        "edge_tree_spacing_m": 0.5,
        "flight_clearance_z_m": 2.8,
        "side_boundary_tree_spacing_m": 0.28,
        "p0_use_current_integrity_prior": True,
        "p0_conservative_max_with_gnss": True,
        "planner_executor_thread_count": 6,
        "online_mapping": {
            "enabled": online,
            "truth_map_topic": "",
            "planner_occupancy_topic": (
                "/sim/drone_0/lidar" if online
                else "/map_generator/global_cloud"),
            "frame_id": "map",
            "origin_m": [-21.0, -11.0, 0.0],
            "extent_m": [42.0, 22.0, 8.0],
            "ego_resolution_m": 0.1,
            "risk_resolution_m": 0.5,
            "risk_voxels_per_ego_axis": 5,
            # Base EGO keeps its original optimistic exploration semantics.
            # P0 still records UNKNOWN and risk-aware P4 remains fail-closed.
            "unknown_as_occupied": False,
            "current_vehicle_clearance_radius_m": 0.35 if online else 0.0,
            "fit_grid_to_map_cloud": not online,
            "provider_cost_source": (
                "pre_conservative_fim_ratio" if online
                else "legacy_safety_pl"),
            "require_safety_ratio_below_one_for_cost": online,
            "alert_limit_policy": (
                "fixed_hal20_val40_v1" if online
                else "fixed_hal10_val20_v1"),
            "hal_m": 20.0 if online else 10.0,
            "val_m": 40.0 if online else 20.0,
        },
        "gnss": {
            "ephemeris_source": "rinex",
            "enabled_constellations": ["GPS", "GAL", "GLO"],
            "map_occlusion": True,
            "nlos": True,
            "multipath": True,
            "skymask": False,
            "fault_injection": False,
            "measured_epoch_support_radius_m": 0.45 if online else 0.0,
            "measured_epoch_integrity_max_delta_s": 0.25,
        },
        "forks": [{
            "fork_index": index,
            "x_min_m": -16.0 + 8.0 * index,
            "length_m": 8.0,
            "low_risk_amplitude_m": 4.0,
            "high_risk_amplitude_m": 2.8,
            "low_risk_y_sign": sign,
            "low_risk_side": "right" if sign < 0 else "left",
        } for index, sign in enumerate(signs)],
    }
    canonical = json.dumps(contract, sort_keys=True, separators=(",", ":"))
    contract["fingerprint"] = (
        "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest())
    return contract


def forest_manifest_evidence(
        run_root: Path, scenario: str = FOREST_SCENARIO) -> dict:
    """Bind analyzer assumptions to the effective launch manifest."""
    manifests = sorted((run_root / "exports").glob(
        "**/test_planner_manifest.json"))
    if len(manifests) != 1:
        return {
            "matches_expected": False,
            "failures": ["forest_effective_contract_missing"],
            "manifest_count": len(manifests),
        }
    manifest_path = manifests[0]
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError):
        return {
            "matches_expected": False,
            "failures": ["forest_effective_contract_invalid"],
            "manifest_path": str(manifest_path.relative_to(run_root)),
        }
    actual = manifest.get("scenario_contract")
    scene_map = actual.get("scene_map", {}) if isinstance(actual, dict) else {}
    geometry = actual.get("geometry", {}) if isinstance(actual, dict) else {}
    gnss = actual.get("gnss", {}) if isinstance(actual, dict) else {}
    p0 = actual.get("p0_prediction", {}) if isinstance(actual, dict) else {}
    integrity_limits = (
        actual.get("integrity_alert_limits", {})
        if isinstance(actual, dict) else {})
    p5_limits = (
        actual.get("p5_alert_limits", {})
        if isinstance(actual, dict) else {})
    expected = forest_scene_contract(scenario)
    online = expected["online_mapping"]
    expected_values = {
        "layout_mode": (
            "forked_s_forest_v2" if scenario == FOREST_SCENARIO
            else "forked_s_forest_v1"),
        "map_size_m": [42.0, 22.0, 8.0],
        "forest_size_m": [40.0, 20.0],
        "forest_seed": expected["forest_seed"],
        "fork_risk_seed": expected["risk_seed"],
        "fork_count": len(expected["forks"]),
        "fork_x_min_m": expected["forks"][0]["x_min_m"],
        "fork_length_m": expected["forks"][0]["length_m"],
        "low_risk_amplitude_m": expected["forks"][0][
            "low_risk_amplitude_m"],
        "high_risk_amplitude_m": expected["forks"][0][
            "high_risk_amplitude_m"],
        "corridor_width_m": expected["corridor_width_m"],
        "junction_clearance_radius_m": expected[
            "junction_buffer_radius_m"],
        "start_canopy_clearance_radius_m": expected[
            "start_canopy_clearance_radius_m"],
        "flight_clearance_z_m": expected["flight_clearance_z_m"],
        "side_boundary_tree_spacing_m": expected[
            "side_boundary_tree_spacing_m"],
        "expanded_low_risk_sides": [
            fork["low_risk_side"] for fork in expected["forks"]],
    }
    comparisons = {
        **{f"scene_map.{key}": (scene_map.get(key), value)
           for key, value in expected_values.items()},
        "geometry.start_m": (geometry.get("start_m"),
                             expected["start_xyz_m"]),
        "geometry.goal_m": (geometry.get("goal_m"),
                            expected["goal_xyz_m"]),
        "gnss.ephemeris_source": (
            gnss.get("ephemeris_source"), "rinex"),
        "gnss.enabled_constellations": (
            gnss.get("enabled_constellations"),
            ",".join(expected["gnss"]["enabled_constellations"])),
        "gnss.map_occlusion": (gnss.get("map_occlusion"), True),
        "gnss.skymask": (gnss.get("skymask"), False),
        "gnss.nlos": (gnss.get("nlos"), True),
        "gnss.multipath": (gnss.get("multipath"), True),
        "p0.online_mapping_mode": (
            p0.get("online_mapping_mode"), online["enabled"]),
        "p0.fit_grid_to_map_cloud": (
            p0.get("fit_grid_to_map_cloud"),
            online["fit_grid_to_map_cloud"]),
        "p0.map_topic": (p0.get("map_topic"), online["truth_map_topic"]),
        "p0.origin_m": (p0.get("origin_m"), online["origin_m"]),
        "p0.extent_m": (p0.get("extent_m"), online["extent_m"]),
        "p0.risk_resolution_m": (
            p0.get("risk_resolution_m"), online["risk_resolution_m"]),
        "p0.ego_resolution_m": (
            p0.get("ego_resolution_m"), online["ego_resolution_m"]),
        "p0.ego_origin_m": (p0.get("ego_origin_m"), online["origin_m"]),
        "p0.unknown_as_occupied": (
            p0.get("unknown_as_occupied"), online["unknown_as_occupied"]),
        "p0.current_vehicle_clearance_radius_m": (
            p0.get("current_vehicle_clearance_radius_m"),
            online["current_vehicle_clearance_radius_m"]),
        "p0.provider_cost_source": (
            p0.get("provider_cost_source"), online["provider_cost_source"]),
        "p0.require_safety_ratio_below_one_for_cost": (
            p0.get("require_safety_ratio_below_one_for_cost"),
            online["require_safety_ratio_below_one_for_cost"]),
        "p0.alert_limit_policy_id": (
            p0.get("alert_limit_policy_id"), online["alert_limit_policy"]),
        "p0.alert_limit_h_m": (
            p0.get("alert_limit_h_m"), online["hal_m"]),
        "p0.alert_limit_v_m": (
            p0.get("alert_limit_v_m"), online["val_m"]),
        "p0.skip_occupied_voxels": (
            p0.get("skip_occupied_voxels"), True),
        "p0.use_current_integrity_prior": (
            p0.get("use_current_integrity_prior"), True),
        "p0.conservative_max_with_gnss": (
            p0.get("conservative_max_with_gnss"),
            expected["p0_conservative_max_with_gnss"]),
        "p0.executor_thread_count": (
            p0.get("executor_thread_count"),
            expected["planner_executor_thread_count"]),
    }
    if online["enabled"]:
        comparisons.update({
            "integrity_alert_limits.dynamic": (
                integrity_limits.get("dynamic"), False),
            "integrity_alert_limits.hal_m": (
                integrity_limits.get("hal_m"), online["hal_m"]),
            "integrity_alert_limits.val_m": (
                integrity_limits.get("val_m"), online["val_m"]),
            "p5_alert_limits.mode": (
                p5_limits.get("mode"), "config_constant"),
            "p5_alert_limits.hal_m": (
                p5_limits.get("hal_m"), online["hal_m"]),
            "p5_alert_limits.val_m": (
                p5_limits.get("val_m"), online["val_m"]),
        })
    mismatches = {
        key: {"actual": values[0], "expected": values[1]}
        for key, values in comparisons.items() if values[0] != values[1]
    }
    failures = ["forest_contract_mismatch"] if mismatches else []
    return {
        "matches_expected": not failures,
        "failures": failures,
        "manifest_path": str(manifest_path.relative_to(run_root)),
        "scenario_fingerprint": manifest.get("scenario_fingerprint"),
        "scenario_contract": actual,
        "lidar_renderer": manifest.get("lidar_renderer"),
        "mismatches": mismatches,
    }


def effective_lidar_renderer_evidence(run_root: Path) -> dict | None:
    """Return the immutable LiDAR renderer contract recorded by launch."""
    manifests = sorted((run_root / "exports").glob(
        "**/test_planner_manifest.json"))
    if len(manifests) != 1:
        return None
    try:
        manifest = json.loads(manifests[0].read_text())
    except (OSError, json.JSONDecodeError):
        return None
    renderer = manifest.get("lidar_renderer")
    return renderer if isinstance(renderer, dict) else None


def lidar_runtime_stats(stdout: str) -> dict:
    rows = [
        (int(frame), float(stamp), int(rays), int(hits), float(latency))
        for frame, stamp, rays, hits, latency in re.findall(
            r"first-hit lidar frame=(\d+) stamp=([0-9]+(?:\.[0-9]+)?)"
            r"[^\n]*rays=(\d+) hits=(\d+)"
            r"[^\n]*latency_ms=([0-9]+(?:\.[0-9]+)?)",
            stdout,
        )
    ]
    if not rows:
        return {"sample_count": 0}
    latencies = sorted(row[4] for row in rows)
    p95_index = max(0, math.ceil(0.95 * len(latencies)) - 1)
    frame_span = rows[-1][0] - rows[0][0]
    stamp_span_s = rows[-1][1] - rows[0][1]
    frame_intervals_s = [
        (current[1] - previous[1]) / (current[0] - previous[0])
        for previous, current in zip(rows, rows[1:])
        if current[0] > previous[0] and current[1] > previous[1]
    ]
    sorted_intervals = sorted(frame_intervals_s)
    interval_p95_index = max(
        0, math.ceil(0.95 * len(sorted_intervals)) - 1)
    return {
        "sample_count": len(rows),
        "first_frame": rows[0][0],
        "last_frame": rows[-1][0],
        "ray_count": rows[-1][2],
        "ray_count_min": min(row[2] for row in rows),
        "ray_count_max": max(row[2] for row in rows),
        "hit_count_min": min(row[3] for row in rows),
        "hit_count_max": max(row[3] for row in rows),
        "effective_rate_hz": (
            frame_span / stamp_span_s
            if frame_span > 0 and stamp_span_s > 0.0 else None
        ),
        "frame_interval_s_p95": (
            sorted_intervals[interval_p95_index]
            if sorted_intervals else None
        ),
        "frame_interval_s_max": (
            sorted_intervals[-1] if sorted_intervals else None
        ),
        "render_latency_ms_p95": latencies[p95_index],
        "render_latency_ms_max": latencies[-1],
    }


def lidar_runtime_failures(renderer: dict | None, stats: dict) -> list[str]:
    """Fail ICRA first-hit runs when the declared real-time contract drifts."""
    if not renderer or renderer.get("mode") != "spherical_first_hit_v1":
        return []
    failures = []
    expected_rays = renderer.get("ray_count")
    if stats.get("sample_count", 0) < 2:
        failures.append("lidar_runtime_samples_insufficient")
    if (not isinstance(expected_rays, int) or expected_rays <= 0 or
            stats.get("ray_count_min") != expected_rays or
            stats.get("ray_count_max") != expected_rays):
        failures.append("lidar_ray_count_drift")
    latency_p95 = stats.get("render_latency_ms_p95")
    if not isinstance(latency_p95, (int, float)) or latency_p95 >= 80.0:
        failures.append("lidar_render_p95_exceeded")
    effective_rate_hz = stats.get("effective_rate_hz")
    if (not isinstance(effective_rate_hz, (int, float)) or
            effective_rate_hz < 9.5):
        failures.append("lidar_effective_rate_below_9_5hz")
    max_frame_interval_s = stats.get("frame_interval_s_max")
    if (not isinstance(max_frame_interval_s, (int, float)) or
            max_frame_interval_s > 0.2):
        failures.append("lidar_frame_gap_exceeded_0_2s")
    return failures


def stage_launch_args(
        stage: str, scenario: str = DEFAULT_SCENARIO,
        forest_variant: str | None = None) -> dict[str, str]:
    if stage not in STAGES:
        raise ValueError(f"unknown stage: {stage}")
    if scenario not in (DEFAULT_SCENARIO, *FOREST_SCENARIOS):
        raise ValueError(f"unsupported scenario: {scenario}")
    if forest_variant not in (None, "risk", "baseline"):
        raise ValueError(f"unsupported forest variant: {forest_variant}")
    if forest_variant is not None and not _is_forest_scenario(scenario):
        raise ValueError("forest variants require the dense forest scenario")
    launch_args = dict(STAGES[stage].launch_args)
    launch_args["scenario"] = scenario
    if forest_variant == "baseline":
        launch_args.update({
            "p0.enable_risk_grid": "true",
            "planner_enable_p4": "false",
            "planner_enable_p5_final": "false",
            "planner_enable_p5_runtime": "false",
        })
    launch_args.update(FORBIDDEN_LAYER_ARGS)
    return launch_args


def stage_duration_s(
        stage: str, scenario: str = DEFAULT_SCENARIO,
        forest_variant: str | None = None) -> float:
    if stage == "p4" and _is_forest_scenario(scenario):
        return 90.0
    if (stage == "full" and _is_forest_scenario(scenario)
            and forest_variant in ("risk", "baseline")):
        return 90.0
    return STAGES[stage].duration_s


def _result(failures: list[str], **details) -> dict:
    unique = list(dict.fromkeys(failures))
    return {
        "schema_version": "icra_interface_stage_summary_v1",
        "development_only": True,
        "qualification_claim": False,
        "scientific_effect_claim": False,
        "result": "PASS" if not unique else "FAIL",
        "failures": unique,
        **details,
    }


def _finite_number(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite_csv_number(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def analyze_forest_risk(
        records: list[dict], health: list[dict] | None = None,
        decisions: list[dict] | None = None,
        lineage: list[dict] | None = None) -> dict:
    """Bind each progressive fork contrast to one published P4 identity."""
    health_by_generation = {}
    for row in health or []:
        payload = row.get("payload", row)
        try:
            generation_id = int(payload.get("generation_id", 0) or 0)
            source_identity = {
                "occupancy_generation": int(payload[
                    "source_occupancy_generation"]),
                "occupancy_stamp_s": float(payload[
                    "source_occupancy_stamp_s"]),
                "prior_generation": int(payload[
                    "source_prior_generation"]),
                "prior_stamp_s": float(payload["source_prior_stamp_s"]),
                "gnss_generation": int(payload[
                    "source_gnss_generation"]),
                "gnss_stamp_s": float(payload["source_gnss_stamp_s"]),
                "lidar_generation": int(payload[
                    "source_lidar_generation"]),
                "lidar_stamp_s": float(payload["source_lidar_stamp_s"]),
            }
        except (KeyError, TypeError, ValueError):
            continue
        required_strings = (
            "snapshot_config_hash", "source_identity_hash", "geometry_id",
            "alert_limit_policy_id")
        try:
            alert_limits = (
                float(payload["alert_limit_h_m"]),
                float(payload["alert_limit_v_m"]),
            )
        except (KeyError, TypeError, ValueError):
            continue
        if (generation_id <= 0
                or not all(payload.get(field) for field in required_strings)
                or not all(math.isfinite(value) and value > 0.0
                           for value in alert_limits)
                or not all(math.isfinite(value) for key, value
                           in source_identity.items() if key.endswith("_s"))):
            continue
        health_by_generation[generation_id] = {
            "snapshot_generation_id": generation_id,
            "snapshot_config_hash": payload["snapshot_config_hash"],
            "source_identity_hash": payload["source_identity_hash"],
            "geometry_id": payload["geometry_id"],
            "alert_limit_policy_id": payload["alert_limit_policy_id"],
            "alert_limit_h_m": alert_limits[0],
            "alert_limit_v_m": alert_limits[1],
            "source_identity": source_identity,
        }

    selected = _selected_decisions(decisions or [])
    published_lineage_keys = {
        tuple(group["key"][:len(DECISION_ID_FIELDS) + 3]): group
        for group in _lineage_groups(decisions or [], lineage or [])
        if {"final_bspline_before_p5", "normal_publish_authorized"}
        .issubset(group["stages"])
    }

    def bound_identity(generation_id: int, fork_index: int) -> dict | None:
        health_identity = health_by_generation.get(generation_id)
        if health_identity is None:
            return None
        source = health_identity["source_identity"]
        for decision in selected:
            if _decision_fork_index(decision) != fork_index:
                continue
            try:
                if decision.get("schema_version") in \
                        P4_FORWARD_DECISION_SCHEMAS:
                    identity_matches = (
                        int(decision["risk_generation"]) == generation_id
                        and decision["snapshot_config_hash"] ==
                        health_identity["snapshot_config_hash"]
                        and decision["source_identity_hash"] ==
                        health_identity["source_identity_hash"]
                        and decision["geometry_id"] ==
                        health_identity["geometry_id"]
                        and decision["alert_limit_policy_id"] ==
                        health_identity["alert_limit_policy_id"]
                        and int(decision["occupancy_generation"]) ==
                        source["occupancy_generation"]
                        and math.isclose(
                            float(decision["occupancy_stamp_s"]),
                            source["occupancy_stamp_s"],
                            rel_tol=0.0, abs_tol=1.0e-6))
                    decision_key = (str(decision["decision_event_id"]),)
                else:
                    identity_matches = (
                        int(decision["snapshot_generation_id"]) == generation_id
                        and decision["snapshot_config_hash"] ==
                        health_identity["snapshot_config_hash"]
                        and decision["source_identity_hash"] ==
                        health_identity["source_identity_hash"]
                        and decision["geometry_id"] ==
                        health_identity["geometry_id"]
                        and int(decision["occupancy_epoch"]) ==
                        source["occupancy_generation"]
                        and math.isclose(
                            float(decision["occupancy_stamp_s"]),
                            source["occupancy_stamp_s"],
                            rel_tol=0.0, abs_tol=1.0e-6))
                    decision_key = _decision_lineage_key(decision)
            except (KeyError, TypeError, ValueError):
                continue
            group = published_lineage_keys.get(decision_key)
            if not identity_matches or group is None:
                continue
            return {
                **health_identity,
                "planning_attempt_id": decision["planning_attempt_id"],
                "decision_event_id": decision.get("decision_event_id"),
                "collision_segment_id": decision.get(
                    "collision_segment_id"),
                "request_hash": decision.get("request_hash"),
                "occupancy_epoch": int(decision.get(
                    "occupancy_generation", decision.get(
                        "occupancy_epoch", 0))),
                "occupancy_stamp_s": float(decision["occupancy_stamp_s"]),
                "lineage_stages": sorted(group["stages"]),
                "trajectory_id": group["trajectory_id"],
                "trajectory_start_ns": group["start_ns"],
            }
        return None

    generation_results = []
    for row in records:
        if row.get("kind") != "forest_risk_generation":
            continue
        payload = row.get("payload", row)
        generation_id = int(payload.get("generation_id", 0) or 0)
        fork_results = []
        for fork in payload.get("forks", []):
            fork_index = fork.get("fork_index")
            low = fork.get("low", {})
            high = fork.get("high", {})
            complete = all(
                int(arm.get("sample_count", 0) or 0) > 0
                and int(arm.get("valid_count", 0) or 0)
                == int(arm.get("sample_count", 0) or 0)
                for arm in (low, high))
            finite = all(_finite_number(arm.get(field))
                         for arm in (low, high)
                         for field in (
                             "mean_c_pi", "max_c_pi", "mean_pl",
                             "mean_fim_ratio", "max_fim_ratio"))
            contrast = bool(complete and finite) and (
                float(low["mean_fim_ratio"])
                <= 0.9 * float(high["mean_fim_ratio"])
                and float(low["max_fim_ratio"])
                < float(high["max_fim_ratio"]))
            source_contrast = {}
            for source, field in (
                    ("gnss", "mean_gnss_ratio"),
                    ("lidar", "mean_lidar_ratio"),
                    ("fim", "mean_fim_ratio"),
                    ("safety", "mean_risk_ratio")):
                low_value = low.get(field)
                high_value = high.get(field)
                source_contrast[source] = {
                    "low": low_value,
                    "high": high_value,
                    "lower_on_open_arm": bool(
                        _finite_number(low_value)
                        and _finite_number(high_value)
                        and float(low_value) < float(high_value)),
                }
            composite_identity = (
                bound_identity(generation_id, int(fork_index))
                if fork_index in {0, 1, 2, 3} else None)
            fork_results.append({
                "fork_index": fork_index,
                "complete": complete,
                "contrast_pass": contrast,
                "source_contrast": source_contrast,
                "composite_identity": composite_identity,
                "identity_lineage_pass": composite_identity is not None,
                "low": low,
                "high": high,
            })
        generation_results.append({
            "generation_id": generation_id,
            "forks": fork_results,
            "pass": len(fork_results) == 4
            and {item["fork_index"] for item in fork_results}
            == {0, 1, 2, 3}
            and all(item["contrast_pass"] for item in fork_results),
        })
    contrast_forks = {}
    passing_forks = {}
    for generation in generation_results:
        for fork in generation["forks"]:
            fork_index = fork.get("fork_index")
            if (fork_index in {0, 1, 2, 3}
                    and fork["contrast_pass"]
                    and fork_index not in contrast_forks):
                contrast_forks[fork_index] = {
                    **fork,
                    "generation_id": generation["generation_id"],
                }
            if (fork_index in {0, 1, 2, 3}
                    and fork["contrast_pass"]
                    and fork["identity_lineage_pass"]
                    and fork_index not in passing_forks):
                passing_forks[fork_index] = {
                    **fork,
                    "generation_id": generation["generation_id"],
                }
    cross_fork_geometry_ids = {
        item["composite_identity"]["geometry_id"]
        for item in passing_forks.values()}
    cross_fork_config_hashes = {
        item["composite_identity"]["snapshot_config_hash"]
        for item in passing_forks.values()}
    cross_fork_alert_limits = {
        (item["composite_identity"]["alert_limit_h_m"],
         item["composite_identity"]["alert_limit_v_m"])
        for item in passing_forks.values()}
    cross_fork_identity_consistent = (
        len(cross_fork_geometry_ids) == 1
        and len(cross_fork_config_hashes) == 1
        and len(cross_fork_alert_limits) == 1)
    passed = (set(passing_forks) == {0, 1, 2, 3}
              and cross_fork_identity_consistent)
    failures = []
    if set(contrast_forks) != {0, 1, 2, 3}:
        failures.append("forest_risk_contrast_missing")
    if set(passing_forks) != {0, 1, 2, 3}:
        failures.append("forest_risk_identity_lineage_missing")
    if (set(passing_forks) == {0, 1, 2, 3}
            and not cross_fork_identity_consistent):
        failures.append("forest_risk_cross_fork_identity_mismatch")
    source_assessment = {}
    if passed:
        for source in ("gnss", "lidar", "fim", "safety"):
            lower_forks = [
                item["fork_index"] for item in passing_forks.values()
                if item["source_contrast"][source]["lower_on_open_arm"]]
            source_assessment[source] = {
                "lower_risk_fork_count": len(lower_forks),
                "lower_risk_forks": lower_forks,
                "all_four_lower": len(lower_forks) == 4,
            }
    return _result(
        failures,
        generation_count=len(generation_results),
        contrast_generation_ids={
            str(index): contrast_forks[index]["generation_id"]
            for index in sorted(contrast_forks)},
        passing_generation_ids={
            str(index): passing_forks[index]["generation_id"]
            for index in sorted(passing_forks)},
        missing_forks=sorted({0, 1, 2, 3} - set(passing_forks)),
        missing_contrast_forks=sorted(
            {0, 1, 2, 3} - set(contrast_forks)),
        missing_identity_lineage_forks=sorted(
            set(contrast_forks) - set(passing_forks)),
        fork_evidence={
            str(index): passing_forks[index]["composite_identity"]
            for index in sorted(passing_forks)},
        cross_fork_identity={
            "consistent": cross_fork_identity_consistent,
            "geometry_ids": sorted(cross_fork_geometry_ids),
            "snapshot_config_hashes": sorted(cross_fork_config_hashes),
            "alert_limits_m": [list(value)
                               for value in sorted(cross_fork_alert_limits)],
        },
        source_assessment=source_assessment,
        generations=generation_results,
    )


def _forest_arm_centers(fork: dict, x_m: float) -> tuple[float, float]:
    t = min(1.0, max(0.0, (
        x_m - float(fork["x_min_m"])) / float(fork["length_m"])))
    shape = math.sin(math.pi * t) ** 2
    sign = float(fork["low_risk_y_sign"])
    return (
        sign * float(fork["low_risk_amplitude_m"]) * shape,
        -sign * float(fork["high_risk_amplitude_m"]) * shape,
    )


def analyze_forest_path(records: list[dict], variant: str) -> dict:
    """Classify captured trajectory samples by the nearer fork centerline."""
    if variant not in ("risk", "baseline"):
        raise ValueError(f"unsupported forest path variant: {variant}")
    forks = forest_scene_contract()["forks"]
    votes = {index: {"low": 0, "high": 0}
             for index in range(len(forks))}
    for row in records:
        payload = row.get("payload", row)
        point_sets = []
        point = payload.get("position_xyz")
        if isinstance(point, list):
            point_sets.append(point)
        points = payload.get("control_points_xyz")
        if isinstance(points, list):
            point_sets.extend(points)
        for xyz in point_sets:
            if not isinstance(xyz, list) or len(xyz) < 2:
                continue
            try:
                x_m, y_m = float(xyz[0]), float(xyz[1])
            except (TypeError, ValueError):
                continue
            for fork in forks:
                x_min = float(fork["x_min_m"])
                length = float(fork["length_m"])
                t = (x_m - x_min) / length
                if not 0.15 <= t <= 0.85:
                    continue
                low_y, high_y = _forest_arm_centers(fork, x_m)
                arm = "low" if abs(y_m - low_y) < abs(y_m - high_y) else "high"
                votes[int(fork["fork_index"])][arm] += 1
                break
    selected = {}
    for index, arm_votes in votes.items():
        if arm_votes["low"] == arm_votes["high"]:
            selected[index] = "unresolved"
        else:
            selected[index] = (
                "low" if arm_votes["low"] > arm_votes["high"] else "high")
    low_count = sum(arm == "low" for arm in selected.values())
    high_count = sum(arm == "high" for arm in selected.values())
    passed = low_count == 4 if variant == "risk" else high_count >= 3
    return _result(
        [] if passed else [f"forest_{variant}_branch_selection_failed"],
        forest_variant=variant,
        selected_arms={str(key): value for key, value in selected.items()},
        selected_low_risk_forks=low_count,
        selected_high_risk_forks=high_count,
        sample_votes={str(key): value for key, value in votes.items()},
    )


def analyze_estimator(metrics: dict) -> dict:
    failures: list[str] = []
    required = (
        "acc_scale", "rotation_deg", "velocity_norm_mps", "bias_norm",
        "odom_count", "position_error_p95_m", "finite",
    )
    if any(key not in metrics for key in required):
        failures.append("estimator_metrics_incomplete")
    if not _finite_number(metrics.get("acc_scale")) or not math.isclose(
            float(metrics.get("acc_scale", math.nan)), 1.0,
            rel_tol=0.0, abs_tol=1.0e-9):
        failures.append("acc_scale_not_si")
    limits = (
        ("rotation_deg", 5.0, "initial_rotation_exceeded"),
        ("velocity_norm_mps", 0.1, "initial_velocity_exceeded"),
        ("bias_norm", 0.05, "initial_bias_exceeded"),
        ("position_error_p95_m", 0.25, "aligned_position_error_exceeded"),
    )
    for key, upper, failure in limits:
        if not _finite_number(metrics.get(key)) or float(metrics[key]) > upper:
            failures.append(failure)
    if int(metrics.get("odom_count", 0) or 0) < 10:
        failures.append("iap_odom_not_continuous")
    if metrics.get("finite") is not True:
        failures.append("estimator_non_finite")
    return _result(failures, metrics=metrics)


def _health_payload(row: dict) -> tuple[float, dict]:
    payload = row.get("payload", row)
    received = row.get(
        "receive_steady_s", payload.get("capture_receive_steady_s", math.nan))
    try:
        return float(received), payload
    except (TypeError, ValueError):
        return math.nan, payload


def _healthy(payload: dict) -> bool:
    return (
        payload.get("ready") is True
        and payload.get("stale") is False
        and payload.get("reason") == "ok"
        and int(payload.get("generation_id", 0) or 0) > 0
    )


def analyze_p0(rows: list[dict], capture_start_s: float | None = None) -> dict:
    observations = sorted(
        (_health_payload(row) for row in rows), key=lambda item: item[0])
    observations = [item for item in observations if math.isfinite(item[0])]
    failures: list[str] = []
    first_index = next(
        (index for index, (_, payload) in enumerate(observations)
         if _healthy(payload)), None)
    if first_index is None:
        return _result(["p0_healthy_generation_missing"], health_count=len(rows))
    first_time = observations[first_index][0]
    start_time = (
        float(capture_start_s) if capture_start_s is not None
        else observations[0][0]
    )
    if first_time - start_time > 15.0:
        failures.append("p0_first_healthy_generation_late")
    # The contract starts at the first healthy generation.  A later healthy
    # streak cannot erase a stale/unready report that occurred during the
    # required following 15 seconds.
    window: list[tuple[float, dict]] = []
    for observation in observations[first_index:]:
        window.append(observation)
        if observation[0] - first_time >= 15.0:
            break
    span = window[-1][0] - window[0][0] if window else 0.0
    if len(window) < 10 or span < 15.0:
        failures.append("p0_healthy_window_too_short")
    if not window or any(not _healthy(payload) for _, payload in window):
        failures.append("p0_health_not_continuous")
    generations = [int(payload.get("generation_id", 0) or 0)
                   for _, payload in window]
    if (not generations or any(right < left for left, right in
                               zip(generations, generations[1:]))):
        failures.append("p0_generation_not_monotonic")
    if not generations or max(generations) - min(generations) < 3:
        failures.append("p0_generation_did_not_advance")
    latest = window[-1][1] if window else {}
    identity_payload = next((
        payload for _, payload in reversed(window)
        if payload.get("refresh_evidence_state") == "COMPLETED_SUCCESS"
        and payload.get("snapshot_config_hash")
        and payload.get("source_identity_hash")
        and int(payload.get("result_generation_id", 0) or 0)
        == int(payload.get("generation_id", 0) or 0)
    ), None)
    if identity_payload is None:
        failures.append("p0_completed_snapshot_identity_missing")
        identity_payload = latest
    return _result(
        failures,
        health_count=len(observations),
        window_count=len(window),
        window_span_s=span,
        first_healthy_delay_s=first_time - start_time,
        generation_min=min(generations) if generations else None,
        generation_max=max(generations) if generations else None,
        geometry={
            "geometry_id": identity_payload.get("geometry_id"),
            "frame_id": identity_payload.get("frame_id"),
            "origin_m": identity_payload.get("grid_origin_m"),
            "extent_m": identity_payload.get("grid_extent_m"),
            "dimensions": identity_payload.get("grid_dimensions"),
            "resolution_m": identity_payload.get("grid_resolution_m"),
        },
        alert_limits={
            "policy_id": identity_payload.get("alert_limit_policy_id"),
            "hal_m": identity_payload.get("alert_limit_h_m"),
            "val_m": identity_payload.get("alert_limit_v_m"),
        },
        snapshot_identity={
            "generation_id": identity_payload.get("generation_id"),
            "config_hash": identity_payload.get("snapshot_config_hash"),
            "source_hash": identity_payload.get("source_identity_hash"),
        },
        source_identity={
            "occupancy_generation": identity_payload.get(
                "source_occupancy_generation"),
            "occupancy_stamp_s": identity_payload.get(
                "source_occupancy_stamp_s"),
            "prior_generation": identity_payload.get(
                "source_prior_generation"),
            "prior_stamp_s": identity_payload.get("source_prior_stamp_s"),
            "gnss_generation": identity_payload.get(
                "source_gnss_generation"),
            "gnss_stamp_s": identity_payload.get("source_gnss_stamp_s"),
            "lidar_generation": identity_payload.get(
                "source_lidar_generation"),
            "lidar_stamp_s": identity_payload.get("source_lidar_stamp_s"),
        },
    )


DECISION_ID_FIELDS = (
    "planning_attempt_id", "collision_segment_id", "request_hash",
    "snapshot_generation_id", "snapshot_config_hash",
    "source_identity_hash", "occupancy_epoch", "geometry_id",
    "occupancy_stamp_s",
)

DECISION_SEGMENT_FIELDS = (
    "segment_start_x", "segment_start_y", "segment_start_z",
    "segment_end_x", "segment_end_y", "segment_end_z",
)


def _decision_fork_index(row: dict) -> int | None:
    """Map one local collision segment to exactly one frozen forest fork."""
    try:
        if row.get("schema_version") in P4_FORWARD_DECISION_SCHEMAS:
            coordinates = [
                float(row["request_x"]), float(row["request_y"]),
                float(row["request_z"]), float(row["anchor_x"]),
                float(row["anchor_y"]), float(row["anchor_z"]),
            ]
        else:
            coordinates = [float(row[field])
                           for field in DECISION_SEGMENT_FIELDS]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in coordinates):
        return None
    midpoint_x = 0.5 * (coordinates[0] + coordinates[3])
    forks = forest_scene_contract()["forks"]
    matches = []
    for index, fork in enumerate(forks):
        x_min = float(fork["x_min_m"])
        x_max = x_min + float(fork["length_m"])
        if x_min <= midpoint_x < x_max or (
                index == len(forks) - 1 and midpoint_x == x_max):
            matches.append(int(fork["fork_index"]))
    return matches[0] if len(matches) == 1 else None


def _support_complete(row: dict, prefix: str) -> bool:
    try:
        samples = int(row[f"{prefix}_sample_count"])
        valid = int(row[f"{prefix}_valid_count"])
        unknown = int(row[f"{prefix}_unknown_count"])
        stale = int(row[f"{prefix}_stale_count"])
        non_finite = int(row[f"{prefix}_non_finite_count"])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        samples > 0 and valid == samples and unknown == 0 and stale == 0
        and non_finite == 0
    )


def _selected_decisions(decisions: list[dict]) -> list[dict]:
    if any(row.get("schema_version") in P4_FORWARD_DECISION_SCHEMAS
           for row in decisions):
        return [
            row for row in decisions
            if row.get("schema_version") in P4_FORWARD_DECISION_SCHEMAS
            and row.get("stage") == "forward_decision"
            and row.get("action") == "RISK_SELECTED"
            and (row.get("schema_version") not in {
                 "p4_forward_route_decision_v3",
                 "p4_forward_route_decision_v4",
                 "p4_forward_route_decision_v5",
                 "p4_forward_route_decision_v6",
                 "p4_forward_route_decision_v7",
                 "p4_forward_route_decision_v8",
                 }
                 or (row.get("selection_authority") == "FORMAL"
                     and str(row.get("formal_support")) == "1"))
            and (row.get("schema_version") not in {
                     "p4_forward_route_decision_v4",
                     "p4_forward_route_decision_v5",
                     "p4_forward_route_decision_v6",
                     "p4_forward_route_decision_v7",
                     "p4_forward_route_decision_v8",
                 }
                 or row.get("geometry_commit_verdict") in {
                     "CLEAR_UNCHANGED", "CLEAR_AFTER_UPDATE",
                 })
            and int(row.get("selected_candidate_id", 0) or 0) > 0
            and int(row.get("candidate_count", 0) or 0) >= 2
            and row.get("geometry_id")
            and row.get("alert_limit_policy_id")
            and int(row.get("occupancy_generation", 0) or 0) > 0
            and int(row.get("risk_generation", 0) or 0) > 0
            and (row.get("schema_version") not in {
                 "p4_forward_route_decision_v7",
                 "p4_forward_route_decision_v8"}
                 or (row.get("frame_contract_id")
                     and row.get("local_map_support_identity")
                     and int(row.get("gnss_epoch_identity", 0) or 0) > 0
                     and _finite_csv_number(row.get("gnss_epoch_stamp_s"))))
        ]
    return [
        row for row in decisions
        if row.get("status") == "RISK_SELECTED"
        and str(row.get("selection_applied")) == "1"
        and all(row.get(field) for field in DECISION_ID_FIELDS)
        and all(row.get(field) is not None for field in DECISION_SEGMENT_FIELDS)
        and all(row.get(field) for field in
                ("original_hash", "risk_hash", "selected_hash"))
        and _support_complete(row, "original")
        and _support_complete(row, "risk")
    ]


def _decision_lineage_key(row: dict, lineage: bool = False) -> tuple[str, ...]:
    values = [str(row.get(field, "")) for field in DECISION_ID_FIELDS]
    if lineage:
        values.extend(str(row.get(field, "")) for field in (
            "original_guide_hash", "risk_guide_hash", "selected_guide_hash"))
    else:
        values.extend(str(row.get(field, "")) for field in (
            "original_hash", "risk_hash", "selected_hash"))
    return tuple(values)


def _lineage_groups(decisions: list[dict], lineage: list[dict]) -> list[dict]:
    if any(row.get("schema_version") in P4_FORWARD_DECISION_SCHEMAS
           for row in lineage):
        selected_ids = {
            str(row.get("decision_event_id")): row
            for row in _selected_decisions(decisions)
        }
        grouped: dict[str, list[dict]] = {}
        for row in lineage:
            event_id = str(row.get("decision_event_id", ""))
            if event_id in selected_ids:
                grouped.setdefault(event_id, []).append(row)
        result = []
        for event_id, rows in grouped.items():
            bound = [row for row in rows
                     if int(row.get("trajectory_id", 0) or 0) > 0
                     and int(row.get("trajectory_start_ns", 0) or 0) > 0
                     and row.get("control_points_hash")]
            if not bound:
                continue
            identity_row = bound[-1]
            result.append({
                "key": (event_id,),
                "rows": rows,
                "ordered_stages": [row.get("stage") for row in rows],
                "stages": {row.get("stage") for row in rows},
                "closed_collision_observed": False,
                "trajectory_id": int(identity_row["trajectory_id"]),
                "start_ns": int(identity_row["trajectory_start_ns"]),
            })
        return result
    selected_keys = {_decision_lineage_key(row): row
                     for row in _selected_decisions(decisions)}
    groups: dict[tuple[str, ...], list[dict]] = {}
    for row in lineage:
        if str(row.get("selection_applied")) != "1":
            continue
        key = _decision_lineage_key(row, lineage=True)
        if key not in selected_keys:
            continue
        identity = (
            str(row.get("trajectory_id", "")),
            str(row.get("trajectory_start_ns", "")),
            str(row.get("control_points_hash", "")),
            str(row.get("final_bspline_identity", "")),
        )
        if not all(identity):
            continue
        groups.setdefault(key + identity, []).append(row)
    return [
        {
            "key": key,
            "rows": rows,
            "ordered_stages": [row.get("stage") for row in rows],
            "stages": {row.get("stage") for row in rows},
            "closed_collision_observed": any(
                str(row.get("closed_collision_observed")) == "1"
                for row in rows),
            "trajectory_id": int(key[-4]),
            "start_ns": int(key[-3]),
        }
        for key, rows in groups.items()
        if key[-4].isdigit() and key[-3].isdigit()
    ]


def _stable_bspline(bsplines: list[dict]) -> tuple[bool, float]:
    valid = [row for row in bsplines
             if int(row.get("payload", row).get("trajectory_id", 0) or 0) > 0
             and int(row.get("payload", row).get("start_time_ns", 0) or 0) > 0]
    times = sorted(float(row.get("receive_steady_s", math.nan)) for row in valid)
    times = [value for value in times if math.isfinite(value)]
    span = times[-1] - times[0] if len(times) >= 2 else 0.0
    return len(valid) >= 3 and span >= 5.0, span


def _poscmd_sustained(times: list[float]) -> tuple[bool, float]:
    finite = sorted(float(value) for value in times if _finite_number(value))
    if len(finite) < 2:
        return False, 0.0
    span = finite[-1] - finite[0]
    rate = (len(finite) - 1) / span if span > 0.0 else 0.0
    return span >= 5.0 and rate >= 80.0, rate


def _p5_safe(payload: dict) -> bool:
    return (
        payload.get("action") == "OK"
        and payload.get("raw_action") == "OK"
        and payload.get("reason") == "ok"
        and payload.get("raw_reason") == "ok"
        and payload.get("active_reasons") == []
        and payload.get("current_reason") == ""
        and payload.get("future_reason") == ""
        and payload.get("final_candidate_rejected") is False
        and payload.get("current_integrity_source") == "FUSED"
    )


def _message_identity(row: dict) -> tuple[int, int] | None:
    payload = row.get("payload", row)
    try:
        identity = (
            int(payload.get("trajectory_id", 0) or 0),
            int(payload.get("start_time_ns", 0) or 0),
        )
    except (TypeError, ValueError):
        return None
    return identity if identity[0] > 0 and identity[1] > 0 else None


def _runtime_identity(payload: dict) -> tuple[int, int] | None:
    try:
        identities = {
            (int(sample.get("trajectory_id", 0) or 0),
             int(sample.get("trajectory_start_time_ns", 0) or 0))
            for sample in payload.get("samples", [])
            if sample.get("trajectory_sample_source") == "runtime_committed"
        }
    except (TypeError, ValueError):
        return None
    if len(identities) != 1:
        return None
    identity = next(iter(identities))
    return identity if identity[0] > 0 and identity[1] > 0 else None


def _xyz_vector(payload: dict, key: str) -> tuple[float, float, float] | None:
    try:
        values = tuple(float(value) for value in payload[key])
    except (KeyError, TypeError, ValueError):
        return None
    return values if len(values) == 3 and all(map(math.isfinite, values)) \
        else None


def _distance(a: tuple[float, float, float],
              b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left - right) ** 2
                         for left, right in zip(a, b)))


def _cpp_hexfloat(value: float) -> str:
    mantissa, exponent = float(value).hex().split("p")
    return mantissa.rstrip("0").rstrip(".") + "p" + exponent


def _fnv1a64(text: str) -> str:
    value = 1469598103934665603
    for byte in text.encode():
        value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return f"{value:016x}"


def _captured_bspline_hashes(payload: dict) -> tuple[str, str] | None:
    try:
        points = [[float(value) for value in point]
                  for point in payload["control_points_xyz"]]
        knots = [float(value) for value in payload["knots"]]
    except (KeyError, TypeError, ValueError):
        return None
    if (not points or any(len(point) != 3 for point in points) or
            not knots or not all(math.isfinite(value)
                                 for point in points for value in point) or
            not all(map(math.isfinite, knots))):
        return None
    point_canonical = f"3;{len(points)};" + "".join(
        _cpp_hexfloat(value) + ";" for point in points for value in point)
    knot_canonical = f"{len(knots)};" + "".join(
        _cpp_hexfloat(value) + ";" for value in knots)
    return _fnv1a64(point_canonical), _fnv1a64(knot_canonical)


def analyze_limited_prefix_records(
        lineage: list[dict], bsplines: list[dict], poscmd: list[dict],
        odometry: list[dict], execution_events: list[dict]) -> dict:
    """Classify real LIMITED_PREFIX execution without weakening P4 gates."""
    failures: list[str] = []
    formal = [row for row in lineage
              if row.get("stage") == "normal_publish_authorized"
              and row.get("action") == "RISK_SELECTED"
              and str(row.get("selection_applied", "0")) == "1"]
    limited = [row for row in lineage
               if row.get("stage") == "normal_publish_authorized"
               and row.get("action") in {
                   "DEFER_RISK_SELECTION", "OBSERVE_ONLY"}
               and row.get("deferred_motion_mode") == "COMMON_PREFIX"
               and row.get("reason") == "safe_limited_common_prefix"]
    if not limited:
        outcome = "FORMAL_ROUTE_SELECTED" if formal else "HOLD_NO_EXECUTION"
        return _result(
            ["limited_prefix_not_exercised"],
            stage="limited-prefix", limited_prefix_outcome=outcome,
            formal_route_selected_count=len(formal), limited_publish_count=0)

    if len(limited) > 1:
        attempts = [analyze_limited_prefix_records(
            formal + [candidate], bsplines, poscmd, odometry,
            execution_events) for candidate in limited]
        successful = [attempt for attempt in attempts
                      if attempt["result"] == "PASS"]
        selected = successful[-1] if successful else attempts[-1]
        selected = dict(selected)
        selected["limited_publish_count"] = len(limited)
        selected["limited_attempts"] = [{
            "trajectory_identity": attempt.get("trajectory_identity"),
            "outcome": attempt.get("limited_prefix_outcome"),
            "result": attempt["result"],
            "actual_displacement_m": attempt.get("actual_displacement_m"),
            "failures": attempt["failures"],
        } for attempt in attempts]
        return selected

    # Evaluate one certificate. A repeated latched publication with the same
    # identity is one execution, not several successes.
    row = limited[-1]
    try:
        identity = (int(row["trajectory_id"]),
                    int(row["trajectory_start_ns"]))
        approved_endpoint = tuple(float(row[f"approved_endpoint_{axis}"])
                                  for axis in "xyz")
        duration_s = float(row["trajectory_duration_s"])
        expected_hash = str(row["control_points_hash"])
        expected_knot_hash = str(row["knot_vector_hash"])
    except (KeyError, TypeError, ValueError):
        return _result(
            ["limited_prefix_certificate_identity_invalid"],
            stage="limited-prefix",
            limited_prefix_outcome="HOLD_NO_EXECUTION",
            formal_route_selected_count=len(formal),
            limited_publish_count=len(limited))
    if (identity[0] <= 0 or identity[1] <= 0 or
            not all(map(math.isfinite, approved_endpoint)) or
            not math.isfinite(duration_s) or duration_s <= 0.0 or
            not expected_hash or not expected_knot_hash):
        failures.append("limited_prefix_certificate_identity_invalid")

    matching_events = []
    for event in execution_events:
        try:
            event_identity = (int(event.get("trajectory_id", 0) or 0),
                              int(event.get("trajectory_start_ns", 0) or 0))
        except (TypeError, ValueError):
            continue
        if (event_identity == identity and
                str(event.get("control_points_hash", "")) == expected_hash and
                str(event.get("knot_vector_hash", "")) ==
                expected_knot_hash and
                event.get("authority") == "LIMITED_PREFIX"):
            matching_events.append(event)
    endpoint_events = [event for event in matching_events
                       if event.get("event") == "ENDPOINT_HOLD"
                       and str(event.get("allowed", "0")) == "1"
                       and str(event.get("endpoint_reached", "0")) == "1"
                       and event.get("reason") == "approved_endpoint_reached"]
    revoke_events = [event for event in matching_events
                     if event.get("event") == "RISK_REVOKED"
                     and str(event.get("allowed", "1")) == "0"
                     and event.get("reason") ==
                     "runtime_known_future_integrity_unsafe"]

    matching_splines = []
    for candidate in bsplines:
        payload = candidate.get("payload", candidate)
        hashes = _captured_bspline_hashes(payload)
        if (_message_identity(candidate) == identity and hashes ==
                (expected_hash, expected_knot_hash)):
            matching_splines.append(candidate)
    if not matching_splines:
        failures.append("limited_prefix_bspline_identity_missing")

    start_s = identity[1] * 1.0e-9
    end_s = start_s + duration_s
    revoke_stamps = []
    for event in revoke_events:
        try:
            revoke_stamp = float(event["stamp_s"])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(revoke_stamp):
            revoke_stamps.append(revoke_stamp)
    observation_end_s = min(
        [end_s + 3.0] + [stamp + 0.5 for stamp in revoke_stamps])
    commands = []
    for command in poscmd:
        payload = command.get("payload", command)
        try:
            trajectory_id = int(payload.get("trajectory_id", 0) or 0)
            stamp_s = float(payload.get(
                "stamp_s", command.get("receive_steady_s", math.nan)))
        except (TypeError, ValueError):
            continue
        if trajectory_id == identity[0] and stamp_s >= start_s - 0.2:
            commands.append((stamp_s, payload))
    if not commands:
        failures.append("limited_prefix_position_command_identity_missing")

    odom_samples = []
    for sample in odometry:
        payload = sample.get("payload", sample)
        try:
            stamp_s = float(payload.get(
                "stamp_s", sample.get("receive_steady_s", math.nan)))
        except (TypeError, ValueError):
            continue
        position = _xyz_vector(payload, "position_m")
        velocity = _xyz_vector(payload, "velocity_mps")
        if (position is not None and velocity is not None and
                start_s - 0.2 <= stamp_s <= observation_end_s):
            odom_samples.append((stamp_s, position, velocity))
    actual_displacement = max(
        (_distance(odom_samples[0][1], sample[1])
         for sample in odom_samples[1:]), default=0.0)
    if actual_displacement < 0.05:
        failures.append("limited_prefix_actual_motion_missing")

    initial_position = odom_samples[0][1] if odom_samples else None
    approved_distance = (_distance(initial_position, approved_endpoint)
                         if initial_position is not None else math.nan)
    endpoint_overrun_m = 0.0
    if initial_position is not None and approved_distance > 1.0e-6:
        direction = tuple((approved_endpoint[index] - initial_position[index])
                          / approved_distance for index in range(3))
        endpoint_overrun_m = max((
            sum((position[index] - initial_position[index]) * direction[index]
                for index in range(3)) - approved_distance
            for _, position, _ in odom_samples), default=0.0)
        if endpoint_overrun_m > 0.10:
            failures.append("limited_prefix_approved_endpoint_overrun")

    endpoint_hold_ok = False
    if endpoint_events and commands and odom_samples:
        endpoint_cmd = []
        for stamp_s, payload in commands:
            position = _xyz_vector(payload, "position_xyz")
            velocity = _xyz_vector(payload, "velocity_xyz")
            acceleration = _xyz_vector(payload, "acceleration_xyz")
            if (stamp_s >= end_s and position is not None and
                    velocity is not None and acceleration is not None and
                    _distance(position, approved_endpoint) <= 0.02 and
                    math.sqrt(sum(value * value for value in velocity)) <= .02
                    and math.sqrt(sum(value * value for value in acceleration))
                    <= .05):
                endpoint_cmd.append(stamp_s)
        endpoint_odom = [stamp_s for stamp_s, position, velocity in odom_samples
                         if stamp_s >= end_s
                         and _distance(position, approved_endpoint) <= .15
                         and math.sqrt(sum(value * value for value in velocity))
                         <= .10]
        endpoint_hold_ok = (
            endpoint_cmd and endpoint_odom and
            max(endpoint_cmd) - min(endpoint_cmd) >= 0.8 and
            max(endpoint_odom) - min(endpoint_odom) >= 0.8)

    legal_revoke = False
    legal_revoke_displacement = 0.0
    for event in revoke_events:
        try:
            certificate_generation = int(
                event["certificate_risk_generation"])
            current_generation = int(event["current_risk_generation"])
            hpl = float(event["violation_hpl_m"])
            vpl = float(event["violation_vpl_m"])
            hal = float(event["alert_limit_h_m"])
            val = float(event["alert_limit_v_m"])
            revoke_stamp = float(event["stamp_s"])
            query_stamp = float(event["violation_query_time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        old_commands_after_grace = [
            stamp for stamp, _ in commands if stamp > revoke_stamp + 0.5]
        before_revoke = [sample for sample in odom_samples
                         if sample[0] <= revoke_stamp + 1.0e-6]
        legal_revoke_displacement = max((
            _distance(before_revoke[0][1], sample[1])
            for sample in before_revoke[1:]), default=0.0)
        legal_revoke = (
            current_generation > certificate_generation and
            all(map(math.isfinite, (hpl, vpl, hal, val, query_stamp))) and
            query_stamp >= revoke_stamp and (hpl >= hal or vpl >= val) and
            legal_revoke_displacement >= 0.05 and
            not old_commands_after_grace)
        if legal_revoke:
            break

    if endpoint_hold_ok:
        outcome = "LIMITED_PREFIX_EXECUTED_TO_ENDPOINT"
    elif legal_revoke:
        outcome = "LIMITED_PREFIX_EXECUTED_THEN_RISK_REVOKED"
    else:
        outcome = "HOLD_NO_EXECUTION"
        failures.append("limited_prefix_terminal_outcome_unproven")
    return _result(
        failures, stage="limited-prefix", limited_prefix_outcome=outcome,
        formal_route_selected_count=len(formal),
        limited_publish_count=len(limited), trajectory_identity=list(identity),
        actual_displacement_m=actual_displacement,
        approved_distance_m=approved_distance,
        endpoint_overrun_m=max(0.0, endpoint_overrun_m),
        endpoint_hold_proven=endpoint_hold_ok,
        legal_risk_revoke_proven=legal_revoke,
        displacement_before_revoke_m=legal_revoke_displacement,
        execution_event_count=len(matching_events))


def analyze_stage_records(
        stage: str, health: list[dict], decisions: list[dict],
        lineage: list[dict], bsplines: list[dict], p5_status: list[dict],
        poscmd_times: list[float], capture_start_s: float | None = None) -> dict:
    if stage not in ("p4", "p5-final", "full"):
        raise ValueError(f"unsupported record stage: {stage}")
    p0 = analyze_p0(health, capture_start_s)
    failures = list(p0["failures"])
    current_forward = [
        row for row in decisions
        if row.get("schema_version") in {
            "p4_forward_route_decision_v3",
            "p4_forward_route_decision_v4",
            "p4_forward_route_decision_v5",
            "p4_forward_route_decision_v6",
            "p4_forward_route_decision_v7",
            "p4_forward_route_decision_v8",
        }
        and row.get("stage") == "forward_decision"
    ]
    def csv_finite_values(field: str) -> list[float]:
        values = []
        for row in current_forward:
            try:
                value = float(row.get(field, "nan"))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                values.append(value)
        return values

    forward_latencies = csv_finite_values("compute_latency_ms")
    cspace_latencies = csv_finite_values(
        "configuration_space_prepare_ms")
    # Rate-limited/pending rows intentionally carry no worker timing. Gate the
    # completed decisions only; absence of formal selection is reported by its
    # own lineage gate below.
    if current_forward and not forward_latencies:
        failures.append("p4_forward_timing_missing")
    elif max(forward_latencies, default=-math.inf) >= 150.0:
        failures.append("p4_forward_compute_budget_exceeded")
    if current_forward and not cspace_latencies:
        failures.append("p4_configuration_space_timing_missing")
    elif max(cspace_latencies, default=-math.inf) > 25.0:
        failures.append("p4_configuration_space_prepare_budget_exceeded")
    commit_records = [
        row for row in decisions
        if row.get("schema_version") in {
            "p4_forward_route_decision_v4",
            "p4_forward_route_decision_v5",
            "p4_forward_route_decision_v6",
            "p4_forward_route_decision_v7",
            "p4_forward_route_decision_v8",
        }
    ]
    commit_latencies = []
    attempted_commit_verdicts = {
        "CLEAR_UNCHANGED", "CLEAR_AFTER_UPDATE", "BASE_COLLISION",
        "NEW_ROUTE_COLLISION", "OUT_OF_BOUNDS", "HISTORY_GAP",
        "POLICY_MISMATCH", "INVALID_PATH", "COMPUTE_BUDGET_EXCEEDED",
    }
    for row in commit_records:
        if row.get("geometry_commit_verdict") not in attempted_commit_verdicts:
            continue
        try:
            latency = float(row["geometry_commit_latency_ms"])
            if not math.isfinite(latency):
                raise ValueError("non-finite commit latency")
            commit_latencies.append(latency)
        except (KeyError, TypeError, ValueError):
            failures.append("p4_geometry_commit_timing_missing")
            break
    if max(commit_latencies, default=-math.inf) > 10.0:
        failures.append("p4_geometry_commit_budget_exceeded")
    generation_only_hold_count = sum(
        1 for row in commit_records
        if str(row.get("reason", "")).startswith(
            "live_occupancy_generation_changed_"))
    if generation_only_hold_count:
        failures.append("p4_generation_only_hold_detected")
    selected = _selected_decisions(decisions)
    if not selected:
        failures.append("p4_risk_selected_missing")
    groups = _lineage_groups(decisions, lineage)
    published_groups = [group for group in groups if {
        "final_bspline_before_p5", "normal_publish_authorized",
    }.issubset(group["stages"])]
    if not published_groups:
        failures.append("p4_selected_lineage_missing")
    stable, bspline_span = _stable_bspline(bsplines)
    if not stable:
        failures.append("stable_bspline_missing")
    poscmd_ok, poscmd_rate = _poscmd_sustained(poscmd_times)
    if not poscmd_ok:
        failures.append("poscmd_not_sustained")

    if stage in ("p5-final", "full"):
        final_groups = [group for group in published_groups
                        if "p5_final_pass_before_publish" in group["stages"]]
        final_records = [row for row in p5_status
                         if row.get("payload", row).get("phase") == "final"
                         and _p5_safe(row.get("payload", row))]
        final_admissions = [
            row for row in p5_status
            if row.get("payload", row).get("phase") ==
            "final_publish_authorized"
            and _p5_safe(row.get("payload", row))
            and _finite_number(row.get("payload", row).get(
                "final_evaluation_stamp_s"))
            and _finite_number(row.get("payload", row).get(
                "final_publish_authorization_stamp_s"))
            and float(row.get("payload", row)[
                "final_evaluation_stamp_s"]) <= float(
                    row.get("payload", row)[
                        "final_publish_authorization_stamp_s"])
        ]
        matched = []
        for group in final_groups:
            identity = (group["trajectory_id"], group["start_ns"])
            if any((
                    int(payload.get("final_candidate_traj_id", 0) or 0),
                    int(payload.get("final_candidate_start_time_ns", 0) or 0),
                    ) == identity
                   for row in final_records
                   for payload in [row.get("payload", row)]):
                matched.append(group)
        if not matched:
            failures.append("p5_final_identity_or_status_invalid")
        published_identities = {
            identity for row in bsplines
            if (identity := _message_identity(row)) is not None
        }
        admitted_identities: set[tuple[int, int]] = set()
        for row in final_admissions:
            payload = row.get("payload", row)
            identity = (
                int(payload.get("final_candidate_traj_id", 0) or 0),
                int(payload.get("final_candidate_start_time_ns", 0) or 0),
            )
            if identity[0] > 0 and identity[1] > 0:
                admitted_identities.add(identity)
        published_candidates_final_ok = (
            bool(published_identities)
            and all(
                identity in admitted_identities
                for row in bsplines
                if (identity := _message_identity(row)) is not None
            )
        )
        if not published_candidates_final_ok:
            failures.append("p5_published_candidate_without_prior_final_ok")
        runtime_committed = [
            row for row in p5_status
            if row.get("payload", row).get("phase") == "runtime"
            and any(sample.get("trajectory_sample_source") == "runtime_committed"
                    for sample in row.get("payload", row).get("samples", []))
        ]
        if stage == "p5-final" and runtime_committed:
            failures.append("p5_runtime_leaked_into_final_only_stage")
        if stage == "full":
            runtime_identities: set[tuple[int, int]] = set()
            runtime_ok = bool(runtime_committed)
            publication_times = [
                float(row.get("receive_steady_s")) for row in bsplines
                if _finite_number(row.get("receive_steady_s"))
                and _message_identity(row) is not None
            ]
            first_publication_s = min(publication_times, default=math.inf)
            in_window_runtime_count = 0
            for row in runtime_committed:
                payload = row.get("payload", row)
                identity = _runtime_identity(payload)
                if not _p5_safe(payload) or identity is None:
                    runtime_ok = False
                    continue
                receive_s = float(row.get("receive_steady_s", math.inf))
                if receive_s >= first_publication_s:
                    in_window_runtime_count += 1
                    if identity not in published_identities:
                        runtime_ok = False
                        continue
                runtime_identities.add(identity)
            published_runtime_match = bool(
                runtime_identities.intersection(published_identities))
            runtime_ok = (
                runtime_ok
                and in_window_runtime_count > 0
                and published_runtime_match
            )
            if not runtime_ok:
                failures.append("p5_runtime_identity_or_status_invalid")
            if any(row.get("schema_version") in P4_FORWARD_DECISION_SCHEMAS
                   for row in decisions):
                runtime_lineage_identities = {
                    (group["trajectory_id"], group["start_ns"])
                    for group in groups
                    if "p5_runtime_committed" in group["stages"]
                }
                if not runtime_lineage_identities.intersection(
                        published_identities):
                    failures.append("p4_forward_runtime_lineage_missing")

            seven_stage = _VERTICAL_ANALYZER.analyze_seven_stage_evidence(
                health, decisions, lineage,
                [{**row, "kind": row.get("kind", "normal_bspline")}
                 for row in bsplines] +
                [{**row, "kind": row.get("kind", "p5_status")}
                 for row in p5_status])
            if seven_stage["result"] != "PASS":
                failures.append("existing_seven_stage_analyzer_failed")
        else:
            runtime_identities = set()
            runtime_ok = False
            seven_stage = None
    else:
        final_records = []
        runtime_committed = []
        runtime_identities = set()
        runtime_ok = False
        published_candidates_final_ok = False
        seven_stage = None

    return _result(
        failures,
        stage=stage,
        p0=p0,
        p4_forward_max_latency_ms=max(forward_latencies, default=None),
        p4_configuration_space_prepare_max_ms=max(
            cspace_latencies, default=None),
        p4_geometry_commit_max_latency_ms=max(
            commit_latencies, default=None),
        generation_only_hold_count=generation_only_hold_count,
        selected_count=len(selected),
        lineage_group_count=len(groups),
        published_group_count=len(published_groups),
        bspline_count=len(bsplines),
        bspline_span_s=bspline_span,
        poscmd_count=len(poscmd_times),
        poscmd_rate_hz=poscmd_rate,
        p5_final_ok_count=len(final_records),
        p5_published_candidates_final_ok=published_candidates_final_ok,
        p5_runtime_committed_count=len(runtime_committed),
        p5_runtime_ok_count=(len(runtime_committed) if runtime_ok else 0),
        p5_runtime_identities=[list(identity)
                               for identity in sorted(runtime_identities)],
        seven_stage=seven_stage,
    )


def analyze_shutdown(
        launch_exit_code: int | None, stdout: str,
        owned_group_cleared: bool, residual_nodes: list[str],
        runner_escalated: bool = False) -> dict:
    failures: list[str] = []
    if launch_exit_code != 0:
        failures.append("launch_exit_nonzero")
    if runner_escalated or re.search(
            r"escalating to '?(?:SIGTERM|SIGKILL)|sending signal 'SIGKILL'",
            stdout, flags=re.IGNORECASE):
        failures.append("shutdown_escalated")
    if re.search(r"process has died .*exit code (?:-[0-9]+|[1-9][0-9]*)",
                 stdout):
        failures.append("child_process_nonzero")
    if re.search(r"segmentation fault|RCLError|terminate called|system_error",
                 stdout, flags=re.IGNORECASE):
        failures.append("shutdown_exception_or_crash")
    if not owned_group_cleared:
        failures.append("owned_process_group_remaining")
    if residual_nodes:
        failures.append("task_ros_nodes_remaining")
    return _result(
        failures,
        launch_exit_code=launch_exit_code,
        owned_group_cleared=owned_group_cleared,
        residual_nodes=residual_nodes,
    )


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            rows.append({"kind": "capture_parse_error", "raw": line})
    return rows


def _read_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _vector(text: str, label: str) -> list[float] | None:
    match = re.search(rf"{re.escape(label)}=vec\(([^)]+)\)", text)
    if not match:
        return None
    try:
        return [float(value) for value in match.group(1).split(",")]
    except ValueError:
        return None


def _estimator_metrics(run_root: Path, records: list[dict]) -> dict:
    main_logs = list((run_root / "runtime/iap_logs").glob("**/runtime/iap_main.log"))
    odom_logs = list((run_root / "runtime/iap_logs").glob("**/runtime/iap_odom.log"))
    main_text = main_logs[-1].read_text() if main_logs else ""
    odom_text = odom_logs[-1].read_text() if odom_logs else ""
    scale_match = re.search(r"auto-detected acc_scale=([0-9.eE+-]+)", main_text)
    if not scale_match:
        scale_match = re.search(r"configured acc_scale=([0-9.eE+-]+)", main_text)
    configured_scale = math.nan
    config_paths = sorted((run_root / "runtime").glob("**/config_ros.json"))
    for config_path in config_paths:
        try:
            config = json.loads(config_path.read_text())
            candidate = float(config["glim_ros"]["acc_scale"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if math.isfinite(candidate):
            configured_scale = candidate
            break
    pose_match = re.search(r"T_world_imu=se3\(([^)]+)\)", odom_text)
    pose = []
    if pose_match:
        try:
            pose = [float(value) for value in pose_match.group(1).split(",")]
        except ValueError:
            pose = []
    velocity = _vector(odom_text, "v_world_imu") or []
    bias = _vector(odom_text, "imu_bias") or []
    rotation_deg = math.inf
    if len(pose) == 7 and all(math.isfinite(value) for value in pose):
        norm = math.sqrt(sum(value * value for value in pose[3:]))
        if norm > 0.0:
            qw = max(-1.0, min(1.0, abs(pose[6] / norm)))
            rotation_deg = math.degrees(2.0 * math.acos(qw))
    metrics_paths = list((run_root / "exports").glob("**/iap_sim_truth_vs_est.csv"))
    errors = []
    if metrics_paths:
        for row in _read_csv(metrics_paths[-1]):
            try:
                value = float(row["position_error_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                errors.append(value)
    p95 = math.inf
    if errors:
        ordered = sorted(errors)
        p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
    odom_count = sum(row.get("kind") == "iap_odom" for row in records)
    values = pose + velocity + bias + errors
    return {
        "acc_scale": float(scale_match.group(1)) if scale_match
        else configured_scale,
        "rotation_deg": rotation_deg,
        "velocity_norm_mps": math.sqrt(sum(value * value for value in velocity))
        if velocity else math.inf,
        "bias_norm": math.sqrt(sum(value * value for value in bias))
        if bias else math.inf,
        "odom_count": odom_count,
        "position_error_p95_m": p95,
        "finite": bool(values) and all(math.isfinite(value) for value in values),
    }


def _stage_start_steady_s(run_root: Path) -> float | None:
    """Return the functional-stage origin, falling back for old captures."""
    for path, key in (
            (run_root / "launch_started.json", "started_steady_s"),
            (run_root / "capture_ready.json", "ready_steady_s")):
        if not path.is_file():
            continue
        try:
            value = float(json.loads(path.read_text()).get(key))
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def analyze_forward_risk_samples(
        decisions: list[dict], samples: list[dict],
        candidates: list[dict] | None = None) -> dict:
    """Validate current formal selections against their per-point evidence."""
    failures: list[str] = []
    selected = [
        row for row in _selected_decisions(decisions)
        if row.get("schema_version") in P4_FORMAL_RISK_SAMPLE_SCHEMAS
    ]
    rows_by_event: dict[str, list[dict]] = {}
    for row in samples:
        if row.get("schema_version") not in P4_FORMAL_RISK_SAMPLE_SCHEMAS:
            continue
        rows_by_event.setdefault(str(row.get("decision_event_id", "")), []).append(row)
    required = {
        "candidate_id", "sample_index", "arc_length_m", "x", "y", "z",
        "query_time_s", "gnss_known_count", "gnss_visible_count",
        "gnss_blocked_count", "gnss_attenuated_count", "gnss_unknown_count",
        "gnss_used_count", "local_satellite_set_hash", "gnss_anchor_hpl",
        "gnss_anchor_vpl", "gnss_anchored_hpl", "gnss_anchored_vpl",
        "gnss_raw_hpl", "gnss_raw_vpl", "gnss_receiver_raw_hpl",
        "gnss_receiver_raw_vpl", "gnss_temporal_growth_h",
        "gnss_temporal_growth_v",
        "gnss_spatial_delta_h", "gnss_spatial_delta_v", "hpl", "vpl",
        "hal", "val", "safety_ratio", "fim_ratio", "gnss_supported",
        "lidar_supported", "fim_supported", "safety_state",
        "ranking_state", "reason",
    }
    for decision in selected:
        event_id = str(decision.get("decision_event_id", ""))
        event_rows = rows_by_event.get(event_id, [])
        if not event_rows:
            failures.append("p4_forward_risk_samples_missing")
            continue
        if any(not required.issubset(row) for row in event_rows):
            failures.append("p4_forward_risk_sample_fields_missing")
            continue
        if decision.get("schema_version") in {
                "p4_forward_route_decision_v6",
                "p4_forward_route_decision_v7",
                "p4_forward_route_decision_v8"}:
            if decision.get("result_status") != "READY":
                failures.append("p4_forward_result_not_ready")
                continue
        rows_by_candidate: dict[str, list[dict]] = {}
        for row in event_rows:
            rows_by_candidate.setdefault(
                str(row.get("candidate_id", "")), []).append(row)
        if len(rows_by_candidate) < 2:
            failures.append("p4_forward_risk_candidate_coverage_incomplete")

        eligible_candidates = {
            candidate_id: candidate_rows
            for candidate_id, candidate_rows in rows_by_candidate.items()
            if candidate_rows and all(
                all(str(row.get(field)) == "1" for field in (
                    "gnss_supported", "lidar_supported", "fim_supported"))
                and row.get("safety_state") == "SAFE"
                and row.get("ranking_state") == "COMPARABLE"
                for row in candidate_rows)
        }
        if len(eligible_candidates) < 2:
            failures.append("p4_formal_selection_safe_candidate_count_lt_two")
        selected_candidate_id = str(
            decision.get("selected_candidate_id", ""))
        if selected_candidate_id not in eligible_candidates:
            failures.append("p4_formal_selected_candidate_not_safe_complete")
        elif decision.get("schema_version") in {
                "p4_forward_route_decision_v6",
                "p4_forward_route_decision_v7",
                "p4_forward_route_decision_v8"}:
            formal_rows = [
                row
                for candidate_rows in eligible_candidates.values()
                for row in candidate_rows
            ]
            if any(
                    row.get("support_status") != "MODEL_COMPLETE"
                    or row.get("support_authority") not in {
                        "TRUSTED_LOCAL_MAP", "STRICT_OBSERVATION"}
                    for row in formal_rows):
                failures.append("p4_forward_model_support_incomplete")
        eligible_rows = [
            row
            for candidate_rows in eligible_candidates.values()
            for row in candidate_rows
        ]
        try:
            local_sets_valid = all(
                int(row["gnss_used_count"]) >= 4
                and int(row["local_satellite_set_hash"]) != 0
                for row in eligible_rows)
            limits = {
                (float(row["hal"]), float(row["val"]))
                for row in event_rows
            }
            deltas_reproducible = all(
                math.isclose(
                    float(row[axis_delta]),
                    max(0.0, float(row[candidate_raw]) -
                        float(row[receiver_raw])),
                    rel_tol=1.0e-9, abs_tol=1.0e-9)
                for row in eligible_rows
                for axis_delta, candidate_raw, receiver_raw in (
                    ("gnss_spatial_delta_h", "gnss_raw_hpl",
                     "gnss_receiver_raw_hpl"),
                    ("gnss_spatial_delta_v", "gnss_raw_vpl",
                     "gnss_receiver_raw_vpl"),
                ))
            anchored_reproducible = all(
                math.isclose(
                    float(row[anchored]),
                    float(row[anchor]) + float(row[delta]) +
                    float(row[growth]),
                    rel_tol=1.0e-9, abs_tol=1.0e-9)
                for row in eligible_rows
                for anchored, anchor, delta, growth in (
                    ("gnss_anchored_hpl", "gnss_anchor_hpl",
                     "gnss_spatial_delta_h", "gnss_temporal_growth_h"),
                    ("gnss_anchored_vpl", "gnss_anchor_vpl",
                     "gnss_spatial_delta_v", "gnss_temporal_growth_v"),
                ))
            ratios_reproducible = all(
                math.isclose(
                    float(row["safety_ratio"]),
                    max(float(row["hpl"]) / float(row["hal"]),
                        float(row["vpl"]) / float(row["val"])),
                    rel_tol=1.0e-9, abs_tol=1.0e-9)
                for row in eligible_rows)
            counts_consistent = all(
                int(row["gnss_known_count"]) ==
                int(row["gnss_visible_count"]) +
                int(row["gnss_blocked_count"])
                and int(row["gnss_used_count"]) ==
                int(row["gnss_visible_count"])
                and 0 <= int(row["gnss_attenuated_count"]) <=
                int(row["gnss_visible_count"])
                for row in eligible_rows)
        except (KeyError, TypeError, ValueError):
            local_sets_valid = False
            limits = set()
            deltas_reproducible = False
            anchored_reproducible = False
            ratios_reproducible = False
            counts_consistent = False
        if not local_sets_valid:
            failures.append("p4_formal_selection_local_satellite_set_invalid")
        if len(limits) != 1:
            failures.append("p4_formal_selection_alert_limits_mismatch")
        if not deltas_reproducible:
            failures.append("p4_forward_risk_spatial_delta_not_reproducible")
        if not anchored_reproducible:
            failures.append("p4_forward_risk_anchored_pl_not_reproducible")
        if not ratios_reproducible:
            failures.append("p4_forward_risk_safety_ratio_not_reproducible")
        if not counts_consistent:
            failures.append("p4_forward_risk_satellite_counts_inconsistent")

        candidate_rows_by_id = {
            str(row.get("candidate_id", "")): row
            for row in (candidates or [])
            if str(row.get("decision_event_id", "")) == event_id
        }
        for candidate_id, candidate_samples in eligible_candidates.items():
            try:
                ordered = sorted(
                    candidate_samples, key=lambda row: int(row["sample_index"]))
                indexes = [int(row["sample_index"]) for row in ordered]
                arcs = [float(row["arc_length_m"]) for row in ordered]
                numeric_values = [
                    float(row[field])
                    for row in ordered
                    for field in (
                        "arc_length_m", "x", "y", "z", "query_time_s",
                        "gnss_anchor_hpl", "gnss_anchor_vpl",
                        "gnss_anchored_hpl", "gnss_anchored_vpl",
                        "gnss_raw_hpl", "gnss_raw_vpl",
                        "gnss_receiver_raw_hpl", "gnss_receiver_raw_vpl",
                        "gnss_spatial_delta_h", "gnss_spatial_delta_v",
                        "gnss_temporal_growth_h", "gnss_temporal_growth_v",
                        "hpl", "vpl", "hal", "val", "safety_ratio",
                        "fim_ratio",
                    )
                ]
                coverage_valid = (
                    indexes == list(range(len(ordered)))
                    and bool(arcs) and math.isclose(arcs[0], 0.0, abs_tol=1e-9)
                    and all(math.isfinite(value) for value in numeric_values)
                    and all(
                        0.0 < right - left <= 0.250001
                        for left, right in zip(arcs, arcs[1:]))
                )
                expected = candidate_rows_by_id.get(candidate_id)
                if candidates is not None:
                    coverage_valid = coverage_valid and expected is not None
                    if expected is not None:
                        coverage_valid = coverage_valid and math.isclose(
                            arcs[-1], float(expected["length_m"]),
                            rel_tol=1.0e-9, abs_tol=1.0e-6)
            except (KeyError, TypeError, ValueError):
                coverage_valid = False
            if not coverage_valid:
                failures.append("p4_forward_risk_route_sample_coverage_invalid")
    hashes = {
        str(row.get("local_satellite_set_hash")) for row in samples
        if str(row.get("local_satellite_set_hash", "")) not in ("", "0")
    }
    return {
        "sample_count": len(samples),
        "local_satellite_set_count": len(hashes),
        # Preserve the public summary key used by existing reports while
        # counting every schema that carries the formal per-sample contract.
        "formal_v5_selection_count": len(selected),
        "failures": list(dict.fromkeys(failures)),
    }


def analyze_run(
        stage: str, run_root: Path, scenario: str = DEFAULT_SCENARIO,
        forest_variant: str | None = None) -> dict:
    records = _read_jsonl(run_root / "capture.jsonl")
    health = [row for row in records if row.get("kind") == "p0_health"]
    bsplines = [row for row in records if row.get("kind") == "normal_bspline"]
    p5_status = [row for row in records if row.get("kind") == "p5_status"]
    poscmd_times = [row.get("receive_steady_s") for row in records
                    if row.get("kind") == "poscmd"]
    stage_start = _stage_start_steady_s(run_root)
    decisions = _read_csv(run_root / "exports/planner_p4_risk_astar_debug.csv")
    lineage = _read_csv(
        run_root / "exports/planner_p4_risk_astar_debug.csv.lineage.csv")
    forward_lineage = _read_csv(
        run_root /
        "exports/planner_p4_risk_astar_debug.csv.forward_lineage.csv")
    forward_risk_samples = _read_csv(
        run_root /
            "exports/planner_p4_risk_astar_debug.csv.forward_risk_samples.csv")
    forward_candidates = _read_csv(
        run_root /
        "exports/planner_p4_risk_astar_debug.csv.forward_candidates.csv")
    gnss_risk_detail = _read_csv(
        run_root /
        "exports/planner_p4_risk_astar_debug.csv.gnss_risk_detail.csv")
    execution_events = _read_csv(
        run_root /
        "exports/planner_p4_risk_astar_debug.csv.execution_events.csv")
    if forward_lineage:
        decisions = forward_lineage
        lineage = forward_lineage
    if stage == "estimator":
        return analyze_estimator(_estimator_metrics(run_root, records))
    if stage == "p0":
        return analyze_p0(health, stage_start)
    if stage == "limited-prefix":
        p0 = analyze_p0(health, stage_start)
        limited = analyze_limited_prefix_records(
            lineage, bsplines,
            [row for row in records if row.get("kind") == "poscmd"],
            [row for row in records if row.get("kind") == "iap_odom"],
            execution_events)
        return _result(
            [*p0["failures"], *limited["failures"]],
            **{key: value for key, value in limited.items()
               if key not in ("result", "failures")},
            p0=p0,
            generation_probe_rows=len(_read_csv(
                run_root / "exports/planner_p4_risk_astar_debug.csv."
                "generation_probe.csv")))
    if stage in ("p4", "p5-final", "full"):
        if _is_forest_scenario(scenario) and forest_variant == "baseline":
            p0 = analyze_p0(health, stage_start)
            stable, bspline_span = _stable_bspline(bsplines)
            poscmd_ok, poscmd_rate = _poscmd_sustained(poscmd_times)
            path = analyze_forest_path(records, "baseline")
            failures = list(p0["failures"])
            if not stable:
                failures.append("stable_bspline_missing")
            if not poscmd_ok:
                failures.append("poscmd_not_sustained")
            failures.extend(path["failures"])
            return _result(
                failures, stage=stage, scenario=scenario,
                forest_variant="baseline", p0=p0, forest_path=path,
                bspline_count=len(bsplines), bspline_span_s=bspline_span,
                poscmd_count=len(poscmd_times), poscmd_rate_hz=poscmd_rate,
            )
        base = analyze_stage_records(
            stage, health, decisions, lineage, bsplines, p5_status,
            poscmd_times, stage_start)
        sample_analysis = analyze_forward_risk_samples(
            decisions, forward_risk_samples, forward_candidates)
        if sample_analysis["failures"]:
            base["failures"] = list(dict.fromkeys([
                *base["failures"], *sample_analysis["failures"]]))
            base["result"] = "FAIL"
        base["forward_risk_samples"] = sample_analysis
        base["gnss_risk_detail_rows"] = len(gnss_risk_detail)
        if not _is_forest_scenario(scenario):
            return base
        risk = analyze_forest_risk(records, health, decisions, lineage)
        path = analyze_forest_path(records, forest_variant or "risk")
        failures = [*base["failures"], *risk["failures"], *path["failures"]]
        return _result(
            failures,
            **{key: value for key, value in base.items()
               if key not in ("result", "failures")},
            scenario=scenario,
            forest_variant=forest_variant or "risk",
            forest_risk=risk,
            forest_path=path,
        )
    raise ValueError(f"unsupported functional stage: {stage}")


def planner_local_map_runtime_stats(records: list[dict]) -> dict:
    """Summarize the wire-level adapter evidence without decoding big clouds."""
    current = [row for row in records
               if row.get("kind") == "planner_local_map_current"]
    deltas = [row for row in records
              if row.get("kind") == "planner_local_map_delta"]

    def rate_hz(rows: list[dict]) -> float:
        stamps = [float(row.get("receive_steady_s", math.nan)) for row in rows]
        stamps = [stamp for stamp in stamps if math.isfinite(stamp)]
        if len(stamps) < 2 or max(stamps) <= min(stamps):
            return 0.0
        return (len(stamps) - 1) / (max(stamps) - min(stamps))

    duration_s = 0.0
    if len(current) >= 2:
        duration_s = max(
            0.0,
            float(current[-1]["receive_steady_s"])
            - float(current[0]["receive_steady_s"]),
        )
    delta_duration_s = 0.0
    if len(deltas) >= 2:
        delta_duration_s = max(
            0.0,
            float(deltas[-1]["receive_steady_s"])
            - float(deltas[0]["receive_steady_s"]),
        )
    current_bytes = sum(int(row.get("payload", {}).get(
        "payload_bytes", 0) or 0) for row in current)
    delta_bytes = sum(int(row.get("payload", {}).get(
        "added_payload_bytes", 0) or 0) for row in deltas)
    contract_ids = sorted({
        str(row.get("payload", {}).get("frame_contract_id", ""))
        for row in [*current, *deltas]
        if row.get("payload", {}).get("frame_contract_id")
    })
    generation_contiguous = True
    previous_generation = None
    for row in deltas:
        payload = row.get("payload", {})
        base = int(payload.get("base_generation", 0) or 0)
        generation = int(payload.get("generation", 0) or 0)
        if (not bool(payload.get("complete", False))
                or generation != base + 1
                or (previous_generation is not None
                    and base != previous_generation)):
            generation_contiguous = False
            break
        previous_generation = generation
    positions = []
    for row in current:
        position = row.get("payload", {}).get("position_m")
        if (isinstance(position, list) and len(position) == 3
                and all(_finite_number(value) for value in position)):
            positions.append([float(value) for value in position])
    displacement_m = None
    if len(positions) >= 2:
        displacement_m = math.dist(positions[0], positions[-1])
    current_mib_s = (
        current_bytes / duration_s / (1024.0 * 1024.0)
        if duration_s > 0.0 else 0.0
    )
    delta_mib_s = (
        delta_bytes / delta_duration_s / (1024.0 * 1024.0)
        if delta_duration_s > 0.0 else 0.0
    )
    return {
        "current_frame_count": len(current),
        "current_rate_hz": rate_hz(current),
        "window_delta_count": len(deltas),
        "window_delta_rate_hz": rate_hz(deltas),
        "current_xyz_payload_mib_s": current_mib_s,
        "keyframe_xyz_payload_mib_s": delta_mib_s,
        "total_xyz_payload_mib_s": current_mib_s + delta_mib_s,
        "keyframe_xyz_payload_mib": delta_bytes / (1024.0 * 1024.0),
        "max_active_frame_count": max(
            (int(row.get("payload", {}).get("active_frame_count", 0) or 0)
             for row in deltas),
            default=0,
        ),
        "frame_contract_ids": contract_ids,
        "generation_contiguous": generation_contiguous,
        "first_position_m": positions[0] if positions else None,
        "last_position_m": positions[-1] if positions else None,
        "displacement_m": displacement_m,
    }


def planner_local_map_latency_stats(stdout: str) -> dict:
    """Extract component-side latency windows emitted by the map consumer."""
    callback = [
        (float(p95), float(maximum))
        for p95, maximum in re.findall(
            r"GLIM callback snapshot latency count=100 "
            r"p95_ms=([0-9.]+) max_ms=([0-9.]+)", stdout)
    ]
    adapter = [
        (float(p95), float(maximum))
        for p95, maximum in re.findall(
            r"adapter deskew serialize latency count=100 "
            r"p95_ms=([0-9.]+) max_ms=([0-9.]+)", stdout)
    ]
    current = [
        (float(p95), float(maximum))
        for p95, maximum in re.findall(
            r"registered current frame latency count=100 "
            r"p95_ms=([0-9.]+) max_ms=([0-9.]+)", stdout)
    ]
    delta = [
        (float(p95), float(maximum))
        for p95, maximum in re.findall(
            r"registered keyframe delta latency count=20 "
            r"p95_ms=([0-9.]+) max_ms=([0-9.]+)", stdout)
    ]
    end_to_end = [
        (float(p95), float(maximum))
        for p95, maximum in re.findall(
            r"sensor to occupancy latency count=100 "
            r"p95_ms=([0-9.]+) max_ms=([0-9.]+)", stdout)
    ]
    return {
        "callback_windows": len(callback),
        "callback_p95_ms_max": max(
            (row[0] for row in callback), default=None),
        "callback_budget_ms": 0.2,
        "adapter_windows": len(adapter),
        "adapter_p95_ms_max": max(
            (row[0] for row in adapter), default=None),
        "adapter_budget_ms": 2.0,
        "current_windows": len(current),
        "current_p95_ms_max": max((row[0] for row in current), default=None),
        "current_max_ms": max((row[1] for row in current), default=None),
        "current_budget_ms": 10.0,
        "delta_windows": len(delta),
        "delta_p95_ms_max": max((row[0] for row in delta), default=None),
        "delta_max_ms": max((row[1] for row in delta), default=None),
        "delta_budget_ms": 40.0,
        "sensor_to_occupancy_windows": len(end_to_end),
        "sensor_to_occupancy_p95_ms_max": max(
            (row[0] for row in end_to_end), default=None),
        "sensor_to_occupancy_budget_ms": 80.0,
    }


def planner_local_map_acceptance_failures(
        runtime: dict, latency: dict) -> list[str]:
    """Apply the forest-v2 local-map wire and latency hard gates."""
    failures = []
    current_rate = runtime.get("current_rate_hz")
    delta_rate = runtime.get("window_delta_rate_hz")
    if not _finite_number(current_rate) or not 8.0 <= current_rate <= 12.0:
        failures.append("planner_local_map_current_rate_out_of_bounds")
    if not _finite_number(delta_rate) or not 0.1 <= delta_rate <= 2.2:
        failures.append("planner_local_map_delta_rate_out_of_bounds")
    if runtime.get("max_active_frame_count", 0) > 15:
        failures.append("planner_local_map_active_window_exceeded")
    if not runtime.get("generation_contiguous", False):
        failures.append("planner_local_map_generation_gap")
    if len(runtime.get("frame_contract_ids", [])) != 1:
        failures.append("planner_local_map_contract_inconsistent")
    if runtime.get("total_xyz_payload_mib_s", math.inf) >= 1.5:
        failures.append("planner_local_map_bandwidth_exceeded")
    latency_gates = (
        ("callback", 0.2),
        ("adapter", 2.0),
        ("current", 10.0),
        ("delta", 40.0),
        ("sensor_to_occupancy", 80.0),
    )
    for name, budget in latency_gates:
        value = latency.get(f"{name}_p95_ms_max")
        if latency.get(f"{name}_windows", 0) < 1:
            failures.append(f"planner_local_map_{name}_latency_missing")
        elif not _finite_number(value) or value >= budget:
            failures.append(f"planner_local_map_{name}_latency_exceeded")
    return failures


def summarize_forest_risk_cloud(points: list[dict]) -> dict | None:
    """Reduce a PL cloud to per-fork metrics without retaining cloud points."""
    if not points:
        return None
    generation_ids = {
        int(point.get("generation_id", 0) or 0) for point in points}
    generation_ids.discard(0)
    if len(generation_ids) != 1:
        return None
    forks = forest_scene_contract()["forks"]
    buckets = {
        (index, arm): [] for index in range(4) for arm in ("low", "high")}
    for point in points:
        try:
            x_m = float(point["x"])
            y_m = float(point["y"])
            z_m = float(point["z"])
        except (KeyError, TypeError, ValueError):
            continue
        if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
            continue
        if abs(z_m - 1.5) > 0.75:
            continue
        for fork in forks:
            x_min = float(fork["x_min_m"])
            length = float(fork["length_m"])
            t = (x_m - x_min) / length
            if not 0.2 <= t <= 0.8:
                continue
            low_y, high_y = _forest_arm_centers(fork, x_m)
            distances = {"low": abs(y_m - low_y),
                         "high": abs(y_m - high_y)}
            arm = min(distances, key=distances.get)
            if distances[arm] <= 1.25:
                buckets[(int(fork["fork_index"]), arm)].append(point)
            break

    def arm_summary(rows: list[dict]) -> dict:
        # The fixed geofence cloud includes UNKNOWN cells by design.  Arm
        # completeness is therefore evaluated inside the currently observed,
        # non-occupied ROI; requiring the whole four-fork geofence to be valid
        # would reintroduce the forbidden global-knowledge assumption.
        supported = [row for row in rows
                     if int(row.get("observed", 0) or 0) == 1
                     and int(row.get("occupied", 0) or 0) == 0]
        valid = [row for row in supported
                 if int(row.get("valid", 0) or 0) == 1
                 and int(row.get("unknown", 0) or 0) == 0
                 and int(row.get("stale", 0) or 0) == 0
                 and int(row.get("occupied", 0) or 0) == 0]

        def values(field: str) -> list[float]:
            output = []
            for row in valid:
                try:
                    value = float(row[field])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(value):
                    output.append(value)
            return output

        c_pi = values("c_pi")
        pl = values("pl")
        hpl = values("hpl")
        vpl = values("vpl")
        risk_ratio = values("risk_ratio")
        gnss_ratio = values("gnss_risk_ratio")
        lidar_ratio = values("lidar_risk_ratio")
        fim_ratio = values("fim_risk_ratio")
        floor_h = values("floor_increment_h")
        floor_v = values("floor_increment_v")
        floor_source_h = [int(row.get("floor_source_h", 0) or 0)
                          for row in valid]
        floor_source_v = [int(row.get("floor_source_v", 0) or 0)
                          for row in valid]
        return {
            "sample_count": len(supported),
            "valid_count": len(valid),
            "mean_c_pi": statistics.fmean(c_pi) if c_pi else None,
            "max_c_pi": max(c_pi) if c_pi else None,
            "mean_pl": statistics.fmean(pl) if pl else None,
            "mean_hpl": statistics.fmean(hpl) if hpl else None,
            "mean_vpl": statistics.fmean(vpl) if vpl else None,
            "mean_risk_ratio": (
                statistics.fmean(risk_ratio) if risk_ratio else None),
            "mean_gnss_ratio": (
                statistics.fmean(gnss_ratio) if gnss_ratio else None),
            "mean_lidar_ratio": (
                statistics.fmean(lidar_ratio) if lidar_ratio else None),
            "mean_fim_ratio": (
                statistics.fmean(fim_ratio) if fim_ratio else None),
            "max_fim_ratio": max(fim_ratio) if fim_ratio else None,
            "mean_floor_increment_h_m": (
                statistics.fmean(floor_h) if floor_h else None),
            "mean_floor_increment_v_m": (
                statistics.fmean(floor_v) if floor_v else None),
            "gnss_floor_h_count": floor_source_h.count(1),
            "gnss_floor_v_count": floor_source_v.count(1),
        }

    total_count = len(points)
    observed_rows = [point for point in points
                     if int(point.get("observed", 0) or 0) == 1]
    occupied_count = sum(
        int(point.get("occupied", 0) or 0) == 1 for point in points)
    unknown_count = sum(
        int(point.get("unknown", 0) or 0) == 1 for point in points)
    observed_free_count = sum(
        int(point.get("observed", 0) or 0) == 1
        and int(point.get("occupied", 0) or 0) == 0 for point in points)
    observed_bbox = None
    if observed_rows:
        observed_bbox = {
            "min_xyz_m": [min(float(row[axis]) for row in observed_rows)
                          for axis in ("x", "y", "z")],
            "max_xyz_m": [max(float(row[axis]) for row in observed_rows)
                          for axis in ("x", "y", "z")],
        }
    return {
        "generation_id": next(iter(generation_ids)),
        "coverage": {
            "total_count": total_count,
            "observed_count": len(observed_rows),
            "observed_free_count": observed_free_count,
            "occupied_count": occupied_count,
            "unknown_count": unknown_count,
            "observed_ratio": (
                len(observed_rows) / total_count if total_count else 0.0),
            "unknown_ratio": (
                unknown_count / total_count if total_count else 1.0),
        },
        "observed_bbox": observed_bbox,
        "forks": [{
            "fork_index": index,
            "low": arm_summary(buckets[(index, "low")]),
            "high": arm_summary(buckets[(index, "high")]),
        } for index in range(4)],
    }


def _capture_main(args: argparse.Namespace) -> int:
    import rclpy
    from iap.msg import (
        ActiveLidarWindowDelta, IntegrityReport, RegisteredLidarFrame,
    )
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String
    from traj_utils.msg import Bspline

    output = args.capture_output.resolve()
    ready = args.capture_ready.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    class Capture(Node):
        def __init__(self) -> None:
            super().__init__("icra_interface_integration_capture")
            self.stream = output.open("x", buffering=1)
            reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
            retained = QoSProfile(
                depth=200, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, "/planning/risk_grid_health",
                lambda message: self.json_record("p0_health", message.data),
                reliable)
            self.create_subscription(
                String, "/planning/integrity_gate_status",
                lambda message: self.json_record("p5_status", message.data),
                retained)
            self.create_subscription(
                Bspline, "/drone_0_planning/bspline", self.bspline, retained)
            self.create_subscription(
                PositionCommand, "/drone_0_planning/pos_cmd",
                self.poscmd, reliable)
            self.create_subscription(
                Odometry, "/drone_0_visual_slam/odom",
                lambda message: self.record("iap_odom", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "position_m": [
                        float(message.pose.pose.position.x),
                        float(message.pose.pose.position.y),
                        float(message.pose.pose.position.z),
                    ],
                    "velocity_mps": [
                        float(message.twist.twist.linear.x),
                        float(message.twist.twist.linear.y),
                        float(message.twist.twist.linear.z),
                    ],
                }), qos_profile_sensor_data)
            self.create_subscription(
                IntegrityReport, "/iap/integrity",
                lambda message: self.record("integrity", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "hpl": float(message.hpl),
                    "vpl": float(message.vpl),
                    "fusion_mode": message.fusion_mode,
                }), reliable)
            self.create_subscription(
                PointCloud2, "/sim/drone_0/lidar",
                lambda message: self.record("occupancy_input", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "point_count": int(message.width) * int(message.height),
                }), qos_profile_sensor_data)
            self.create_subscription(
                RegisteredLidarFrame, "/iap/local_map/current_frame",
                lambda message: self.record("planner_local_map_current", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "frame_id": message.header.frame_id,
                    "source_frame_id": int(message.frame_id),
                    "frame_contract_id": message.frame_contract_id,
                    "position_m": [
                        float(message.t_map_lidar.position.x),
                        float(message.t_map_lidar.position.y),
                        float(message.t_map_lidar.position.z),
                    ],
                    "point_count": (
                        int(message.deskewed_hits_lidar.width)
                        * int(message.deskewed_hits_lidar.height)
                    ),
                    "payload_bytes": len(
                        message.deskewed_hits_lidar.data),
                }), qos_profile_sensor_data)
            self.active_local_map_frame_ids = set()
            self.create_subscription(
                ActiveLidarWindowDelta, "/iap/local_map/window_delta",
                self.local_map_delta, reliable)
            self.imu_count = 0
            self.create_subscription(
                Imu, "/sim/drone_0/imu_iap", self.imu,
                qos_profile_sensor_data)
            self.scene_bbox_recorded = False
            self.forest_risk_generations = set()
            if _is_forest_scenario(args.capture_scenario):
                self.create_subscription(
                    PointCloud2, "/map_generator/global_cloud",
                    self.scene_cloud, qos_profile_sensor_data)
                self.create_subscription(
                    PointCloud2, "/iap/rviz/predicted_pl_cloud",
                    self.forest_risk_cloud, qos_profile_sensor_data)
            ready.write_text(json.dumps({
                "schema_version": "icra_interface_capture_ready_v1",
                "ready": True,
                "ready_steady_s": time.monotonic(),
                "pid": os.getpid(),
            }, indent=2, sort_keys=True) + "\n")
            self.create_timer(args.capture_duration, rclpy.shutdown)

        def record(self, kind: str, payload: dict) -> None:
            self.stream.write(json.dumps({
                "kind": kind,
                "receive_steady_s": time.monotonic(),
                "payload": payload,
            }, sort_keys=True) + "\n")

        def json_record(self, kind: str, raw: str) -> None:
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                payload = {"parse_error": True, "raw": raw}
            self.record(kind, payload)

        def bspline(self, message: Bspline) -> None:
            self.record("normal_bspline", {
                "trajectory_id": int(message.traj_id),
                "start_time_ns": int(message.start_time.sec) * 1_000_000_000
                + int(message.start_time.nanosec),
                "control_point_count": len(message.pos_pts),
                "control_points_xyz": [
                    [float(point.x), float(point.y), float(point.z)]
                    for point in message.pos_pts
                ],
                "knot_count": len(message.knots),
                "knots": [float(knot) for knot in message.knots],
            })

        def poscmd(self, message: PositionCommand) -> None:
            self.record("poscmd", {
                "trajectory_id": int(message.trajectory_id),
                "stamp_s": float(message.header.stamp.sec)
                + 1.0e-9 * float(message.header.stamp.nanosec),
                "position_xyz": [
                    float(message.position.x), float(message.position.y),
                    float(message.position.z),
                ],
                "velocity_xyz": [
                    float(message.velocity.x), float(message.velocity.y),
                    float(message.velocity.z),
                ],
                "acceleration_xyz": [
                    float(message.acceleration.x),
                    float(message.acceleration.y),
                    float(message.acceleration.z),
                ],
                "trajectory_flag": int(message.trajectory_flag),
            })

        def local_map_delta(self, message: ActiveLidarWindowDelta) -> None:
            for frame_id in message.removed_frame_ids:
                self.active_local_map_frame_ids.discard(int(frame_id))
            for frame in message.added:
                self.active_local_map_frame_ids.add(int(frame.frame_id))
            self.record("planner_local_map_delta", {
                "stamp_s": float(message.header.stamp.sec)
                + 1.0e-9 * float(message.header.stamp.nanosec),
                "frame_contract_id": message.frame_contract_id,
                "base_generation": int(message.base_generation),
                "generation": int(message.generation),
                "complete": bool(message.complete),
                "added_count": len(message.added),
                "removed_count": len(message.removed_frame_ids),
                "pose_updated_count": len(message.pose_updated_frame_ids),
                "active_frame_count": len(self.active_local_map_frame_ids),
                "added_payload_bytes": sum(
                    len(frame.deskewed_hits_lidar.data)
                    for frame in message.added
                ),
            })

        def scene_cloud(self, message: PointCloud2) -> None:
            if self.scene_bbox_recorded:
                return
            points = point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)
            minima = [math.inf, math.inf, math.inf]
            maxima = [-math.inf, -math.inf, -math.inf]
            count = 0
            for point in points:
                values = [float(point[name]) for name in ("x", "y", "z")]
                for index, value in enumerate(values):
                    minima[index] = min(minima[index], value)
                    maxima[index] = max(maxima[index], value)
                count += 1
            if count == 0:
                return
            self.scene_bbox_recorded = True
            self.record("scene_cloud_bbox", {
                "point_count": count, "min_xyz_m": minima,
                "max_xyz_m": maxima,
                "pointcloud_sha256": hashlib.sha256(
                    bytes(message.data)).hexdigest(),
            })

        def forest_risk_cloud(self, message: PointCloud2) -> None:
            required = (
                "x", "y", "z", "pl", "hpl", "vpl", "c_pi", "valid",
                "risk_ratio", "gnss_risk_ratio", "lidar_risk_ratio",
                "fim_risk_ratio", "floor_increment_h", "floor_increment_v",
                "floor_source_h", "floor_source_v",
                "unknown", "stale", "occupied", "observed",
                "generation_id")
            if not set(required).issubset(
                    {field.name for field in message.fields}):
                self.record("forest_risk_cloud_error", {
                    "reason": "required_fields_missing",
                    "fields": [field.name for field in message.fields],
                })
                return
            points = []
            for point in point_cloud2.read_points(
                    message, field_names=required, skip_nans=False):
                points.append({
                    name: (point[name].item() if hasattr(point[name], "item")
                           else point[name]) for name in required
                })
            summary = summarize_forest_risk_cloud(points)
            if (summary is not None
                    and summary["generation_id"]
                    not in self.forest_risk_generations):
                self.forest_risk_generations.add(summary["generation_id"])
                self.record("forest_risk_generation", summary)

        def imu(self, message: Imu) -> None:
            if self.imu_count >= 100:
                return
            self.imu_count += 1
            acc = message.linear_acceleration
            self.record("raw_imu", {
                "norm_mps2": math.sqrt(acc.x * acc.x + acc.y * acc.y
                                        + acc.z * acc.z),
            })

        def destroy_node(self):
            self.stream.close()
            return super().destroy_node()

    rclpy.init()
    node = Capture()
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _json_write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _emit(message: str) -> None:
    print(f"[icra] {message}", flush=True)


def _node_names(environment: dict[str, str]) -> set[str]:
    completed = subprocess.run(
        ["ros2", "node", "list"], cwd=REPOSITORY, env=environment,
        capture_output=True, text=True, check=False, timeout=5.0)
    return {line.strip() for line in completed.stdout.splitlines()
            if line.strip()}


def _node_subscriptions(
        node_name: str, environment: dict[str, str]) -> tuple[list[str], str]:
    completed = subprocess.run(
        ["ros2", "node", "info", node_name], cwd=REPOSITORY,
        env=environment, capture_output=True, text=True, check=False,
        timeout=5.0)
    subscriptions = []
    in_subscribers = False
    for line in completed.stdout.splitlines():
        stripped = line.strip()
        if stripped == "Subscribers:":
            in_subscribers = True
            continue
        if in_subscribers and stripped.endswith(":"):
            break
        if in_subscribers and stripped.startswith("/"):
            subscriptions.append(stripped.split(":", 1)[0])
    return subscriptions, completed.stderr.strip()


def audit_planner_truth_isolation(
        environment: dict[str, str], timeout_s: float = 18.0) -> dict:
    """Audit planner and GLIM-adapter processes; simulators may use truth."""
    targets = (
        "/drone_0_ego_planner_node",
        "/test_planner_iap_rosnode",
    )
    deadline = time.monotonic() + timeout_s
    nodes: set[str] = set()
    while time.monotonic() < deadline:
        nodes = _node_names(environment)
        if all(target in nodes or target.lstrip("/") in nodes
               for target in targets):
            break
        time.sleep(0.25)
    forbidden_prefixes = ("/map_generator/", "/sim/world/")
    forbidden_exact = {"/sim/drone_0/truth_odom"}
    failures = []
    forbidden = set()
    audited_nodes = []
    for target in targets:
        matched = target if target in nodes else target.lstrip("/")
        if matched not in nodes:
            failures.append("planner_node_missing_for_truth_audit")
            continue
        subscriptions, error = _node_subscriptions(matched, environment)
        node_forbidden = sorted(
            topic for topic in subscriptions
            if topic in forbidden_exact or topic.startswith(forbidden_prefixes)
        )
        forbidden.update(node_forbidden)
        audited_nodes.append({
            "name": matched,
            "subscriptions": subscriptions,
            "forbidden_subscriptions": node_forbidden,
            "stderr": error,
        })
        if error:
            failures.append("planner_graph_audit_error")
    if forbidden:
        failures.append("planner_truth_subscription_detected")
    failures = list(dict.fromkeys(failures))
    return {
        "schema_version": "planner_truth_isolation_audit_v1",
        "pass": not failures,
        "failures": failures,
        "audited_nodes": audited_nodes,
        "forbidden_prefixes": list(forbidden_prefixes),
        "forbidden_exact": sorted(forbidden_exact),
        "forbidden_subscriptions": sorted(forbidden),
    }


def _group_cleared(pid: int) -> bool:
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return True
    except PermissionError:
        return False
    return False


def _capture_group_diagnostics(pgid: int, path: Path) -> None:
    lines = [
        f"process_group={pgid}",
        f"captured_utc={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
    ]
    try:
        completed = subprocess.run(
            ["ps", "-e", "-o", "pid=,ppid=,pgid=,stat=,comm=,wchan:48="],
            capture_output=True, text=True, check=False, timeout=3.0)
        lines.extend(["", "processes:", completed.stdout.rstrip()])
        pids = []
        for row in completed.stdout.splitlines():
            fields = row.split(None, 5)
            if len(fields) >= 3 and fields[2] == str(pgid):
                pids.append(int(fields[0]))
        for pid in pids:
            task_root = Path(f"/proc/{pid}/task")
            for task in sorted(task_root.glob("*")):
                lines.extend(["", f"pid={pid} tid={task.name} kernel_stack:"])
                try:
                    lines.append((task / "stack").read_text().rstrip())
                except OSError as error:
                    lines.append(f"unavailable: {error}")
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        lines.extend(["", f"diagnostic_error: {error}"])
    path.write_text("\n".join(lines) + "\n")


def _stop_group(
        process: subprocess.Popen, timeout_s: float,
        diagnostics_path: Path | None = None) -> tuple[int, bool, bool]:
    escalated = False
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        code = process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        escalated = True
        if diagnostics_path is not None:
            _capture_group_diagnostics(process.pid, diagnostics_path)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        code = process.wait(timeout=5.0)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not _group_cleared(process.pid):
        time.sleep(0.05)
    return code, _group_cleared(process.pid), escalated


def _wait_capture_ready(process: subprocess.Popen, path: Path) -> bool:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False
        if path.is_file():
            try:
                if json.loads(path.read_text()).get("ready") is True:
                    return True
            except json.JSONDecodeError:
                pass
        time.sleep(0.05)
    return False


def _wait_for_exit(
        process: subprocess.Popen, deadline_s: float,
        on_progress: Callable[[float], None], progress_interval_s: float = 5.0,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None) -> bool:
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    next_progress_s = clock() + progress_interval_s
    while process.poll() is None:
        now_s = clock()
        if now_s >= deadline_s:
            return False
        if now_s >= next_progress_s:
            on_progress(now_s)
            next_progress_s = now_s + progress_interval_s
        sleep(min(0.1, deadline_s - now_s))
    return True


def _progress_message(
        stage: str, elapsed_s: float, duration_s: float, timeout_s: float,
        stdout_path: Path) -> str:
    if elapsed_s <= duration_s:
        return (
            f"RUNNING stage={stage} elapsed={elapsed_s:.0f}s "
            f"target={duration_s:g}s log={stdout_path}")
    return (
        f"WAITING_EXIT stage={stage} elapsed={elapsed_s:.0f}s "
        f"timeout={timeout_s:g}s log={stdout_path}")


def _sample_process_group(pgid: int, elapsed_s: float) -> dict:
    """Take a low-overhead aggregate resource sample for the owned launch."""
    sample = {
        "elapsed_s": elapsed_s,
        "process_count": 0,
        "rss_bytes": 0,
        "cpu_seconds": 0.0,
    }
    if psutil is None:
        sample["unavailable_reason"] = "python3-psutil_not_installed"
        return sample
    for process in psutil.process_iter(["pid", "memory_info", "cpu_times"]):
        try:
            if os.getpgid(process.pid) != pgid:
                continue
            memory = process.info["memory_info"]
            cpu = process.info["cpu_times"]
            sample["process_count"] += 1
            sample["rss_bytes"] += int(memory.rss)
            sample["cpu_seconds"] += float(cpu.user + cpu.system)
        except (OSError, psutil.Error):
            continue
    return sample


def process_group_resource_stats(samples: list[dict]) -> dict:
    usable = [sample for sample in samples
              if "unavailable_reason" not in sample]
    cpu_core_samples = []
    for previous, current in zip(usable, usable[1:]):
        wall_delta = (float(current["elapsed_s"])
                      - float(previous["elapsed_s"]))
        cpu_delta = (float(current["cpu_seconds"])
                     - float(previous["cpu_seconds"]))
        if wall_delta > 0.0 and cpu_delta >= 0.0:
            cpu_core_samples.append(cpu_delta / wall_delta)
    return {
        "sample_count": len(usable),
        "peak_process_count": max(
            (int(sample["process_count"]) for sample in usable), default=0),
        "peak_rss_mib": max(
            (int(sample["rss_bytes"]) for sample in usable), default=0)
            / (1024.0 * 1024.0),
        "mean_cpu_cores": (statistics.fmean(cpu_core_samples)
                           if cpu_core_samples else None),
        "peak_cpu_cores": (max(cpu_core_samples)
                           if cpu_core_samples else None),
        "samples": samples,
    }


def _launch_shell(install_root: Path, launch_args: dict[str, str]) -> str:
    argv = ["ros2", "launch", "iap", "test_icra.launch.py"]
    argv.extend(f"{key}:={value}" for key, value in launch_args.items())
    return (
        f"source {shlex.quote(str(install_root / 'setup.bash'))} && exec "
        + shlex.join(argv)
    )


def _run_one_impl(
        stage: str, run_root: Path, install_root: Path,
        start_rviz: bool, shutdown_variant: str | None,
        owned_processes: dict[str, subprocess.Popen],
        owned_streams: dict[str, TextIO],
        scenario: str = DEFAULT_SCENARIO,
        forest_variant: str | None = None) -> dict:
    spec = STAGES[stage]
    duration_s = stage_duration_s(stage, scenario, forest_variant)
    run_root.mkdir(parents=True, exist_ok=False)
    for child in ("runtime/ros_logs", "exports", "bags"):
        (run_root / child).mkdir(parents=True, exist_ok=True)
    environment = {**os.environ, "ROS_LOG_DIR": str(run_root / "runtime/ros_logs")}
    baseline_nodes = _node_names(environment)
    capture_command = [
        sys.executable, str(Path(__file__).resolve()),
        "--capture-output", str(run_root / "capture.jsonl"),
        "--capture-ready", str(run_root / "capture_ready.json"),
        "--capture-duration", str(duration_s + 20.0),
        "--capture-scenario", scenario,
    ]
    capture_stream = (run_root / "capture_stdout.log").open("x")
    owned_streams["capture"] = capture_stream
    capture = subprocess.Popen(
        capture_command, cwd=REPOSITORY, env=environment,
        stdout=capture_stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    owned_processes["capture"] = capture
    if not _wait_capture_ready(capture, run_root / "capture_ready.json"):
        capture_code, capture_cleared, _capture_escalated = _stop_group(
            capture, 2.0)
        capture_stream.close()
        summary = _result(
            ["capture_not_ready"], stage=stage,
            capture_exit_code=capture_code,
            capture_group_cleared=capture_cleared)
        _json_write(run_root / "summary.json", summary)
        return summary

    launch_args = stage_launch_args(stage, scenario, forest_variant)
    if shutdown_variant == "baseline":
        launch_args.update({
            "experiment": "baseline_fused_nominal_off",
            "scenario": "fused_nominal",
            "start_planner": "true",
            "planner_enable_p4": "false",
            "planner_enable_p5_final": "false",
            "planner_enable_p5_runtime": "false",
            "planner_start_delay_s": "3.0",
            "lidar_start_delay_s": "2.0",
            "odometry_initialization_mode": "NAIVE",
        })
    launch_args.update({
        "start_rviz": "true" if start_rviz else "false",
        "run_duration_s": "0" if stage == "shutdown" else str(duration_s),
        "validation_duration_s": str(max(5.0, duration_s - 5.0)),
        "runtime_root_dir": str(run_root / "runtime"),
        "export_root_dir": str(run_root / "exports"),
        "iap_log_root": str(run_root / "runtime/iap_logs"),
        "bag_output_dir": str(run_root / "bags"),
        "p4.debug_csv_path": str(
            run_root / "exports/planner_p4_risk_astar_debug.csv"),
    })
    shell_command = _launch_shell(install_root, launch_args)
    _json_write(run_root / "launch_command.json", {
        "stage": stage,
        "scenario": scenario,
        "forest_variant": forest_variant,
        "forest_scene": (
            forest_scene_contract(scenario)
            if _is_forest_scenario(scenario) else None),
        "shutdown_variant": shutdown_variant,
        "argv": ["bash", "-lc", shell_command],
        "launch_args": launch_args,
    })
    stdout_path = run_root / "stdout.log"
    launch_stream = stdout_path.open("x")
    owned_streams["launch"] = launch_stream
    launch = subprocess.Popen(
        ["bash", "-lc", shell_command], cwd=REPOSITORY, env=environment,
        stdout=launch_stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    owned_processes["launch"] = launch
    started = time.monotonic()
    _json_write(run_root / "launch_started.json", {
        "schema_version": "icra_interface_launch_started_v1",
        "started_steady_s": started,
    })
    resource_samples = [_sample_process_group(launch.pid, 0.0)]
    graph_audit = None
    if (_is_forest_scenario(scenario)
            and scenario == FOREST_SCENARIO
            and launch_args.get("start_planner") == "true"):
        graph_audit = audit_planner_truth_isolation(environment)
        _json_write(run_root / "planner_truth_isolation_audit.json",
                    graph_audit)
    timeout_s = (
        duration_s if stage == "shutdown" else duration_s + 20.0)

    def report_progress(now_s: float) -> None:
        resource_samples.append(_sample_process_group(
            launch.pid, now_s - started))
        _emit(_progress_message(
            stage, now_s - started, duration_s, timeout_s, stdout_path))

    early_exit = False
    if stage == "shutdown":
        early_exit = _wait_for_exit(
            launch, started + duration_s, report_progress)
        launch_code, launch_cleared, launch_escalated = _stop_group(
            launch, 5.0, run_root / "shutdown_stacks.txt")
    else:
        deadline = started + duration_s + 20.0
        exited = _wait_for_exit(launch, deadline, report_progress)
        if not exited:
            launch_code, launch_cleared, launch_escalated = _stop_group(
                launch, 5.0, run_root / "timeout_stacks.txt")
        else:
            launch_code = launch.returncode
            launch_cleared = _group_cleared(launch.pid)
            launch_escalated = False
            early_exit = time.monotonic() - started < duration_s - 1.0
    resource_samples.append(_sample_process_group(
        launch.pid, time.monotonic() - started))
    process_resources = process_group_resource_stats(resource_samples)
    _json_write(run_root / "process_resources.json", process_resources)
    capture_code, capture_cleared, capture_escalated = _stop_group(
        capture, 3.0, run_root / "capture_stacks.txt")
    launch_stream.close()
    capture_stream.close()
    time.sleep(0.5)
    residual_nodes = sorted(_node_names(environment) - baseline_nodes)
    stdout = stdout_path.read_text(errors="replace")
    if stage == "shutdown":
        summary = analyze_shutdown(
            launch_code, stdout, launch_cleared, residual_nodes,
            runner_escalated=launch_escalated)
    else:
        summary = analyze_run(stage, run_root, scenario, forest_variant)
        extra = []
        if early_exit:
            extra.append("launch_exited_early")
        if launch_code != 0:
            extra.append("launch_exit_nonzero")
        if not launch_cleared or not capture_cleared:
            extra.append("owned_process_group_remaining")
        if graph_audit and not graph_audit["pass"]:
            extra.extend(graph_audit["failures"])
        summary = _result(
            [*summary["failures"], *extra],
            **{key: value for key, value in summary.items()
               if key not in ("result", "failures")})
    forest_manifest = None
    if _is_forest_scenario(scenario) and shutdown_variant is None:
        forest_manifest = forest_manifest_evidence(run_root, scenario)
        if forest_manifest["failures"]:
            summary = _result(
                [*summary["failures"], *forest_manifest["failures"]],
                **{key: value for key, value in summary.items()
                   if key not in ("result", "failures")})
    lidar_renderer = effective_lidar_renderer_evidence(run_root)
    lidar_stats = lidar_runtime_stats(stdout)
    lidar_failures = lidar_runtime_failures(lidar_renderer, lidar_stats)
    if lidar_failures:
        summary = _result(
            [*summary["failures"], *lidar_failures],
            **{key: value for key, value in summary.items()
               if key not in ("result", "failures")})
    capture_records = _read_jsonl(run_root / "capture.jsonl")
    local_map_runtime = planner_local_map_runtime_stats(capture_records)
    local_map_latency = planner_local_map_latency_stats(stdout)
    if scenario == FOREST_SCENARIO and shutdown_variant is None:
        local_map_failures = planner_local_map_acceptance_failures(
            local_map_runtime, local_map_latency)
        if local_map_failures:
            summary = _result(
                [*summary["failures"], *local_map_failures],
                **{key: value for key, value in summary.items()
                   if key not in ("result", "failures")})
    summary.update({
        "stage": stage,
        "scenario": scenario,
        "forest_variant": forest_variant,
        "forest_scene": (
            forest_scene_contract(scenario)
            if _is_forest_scenario(scenario) else None),
        "forest_effective_manifest": forest_manifest,
        "lidar_renderer": lidar_renderer,
        "lidar_runtime_stats": lidar_stats,
        "planner_truth_isolation_audit": graph_audit,
        "planner_local_map_runtime": local_map_runtime,
        "planner_local_map_latency": local_map_latency,
        "process_group_resources": process_resources,
        "scene_cloud_bbox": next((
            row.get("payload") for row in capture_records
            if row.get("kind") == "scene_cloud_bbox"), None),
        "shutdown_variant": shutdown_variant,
        "launch_exit_code": launch_code,
        "capture_exit_code": capture_code,
        "launch_group_cleared": launch_cleared,
        "capture_group_cleared": capture_cleared,
        "launch_runner_escalated": launch_escalated,
        "capture_runner_escalated": capture_escalated,
        "residual_nodes": residual_nodes,
        "elapsed_s": time.monotonic() - started,
    })
    _json_write(run_root / "summary.json", summary)
    return summary


def _run_one(
        stage: str, run_root: Path, install_root: Path,
        start_rviz: bool = False, shutdown_variant: str | None = None,
        scenario: str = DEFAULT_SCENARIO,
        forest_variant: str | None = None) -> dict:
    owned_processes: dict[str, subprocess.Popen] = {}
    owned_streams: dict[str, TextIO] = {}
    started = time.monotonic()
    try:
        return _run_one_impl(
            stage, run_root, install_root, start_rviz, shutdown_variant,
            owned_processes, owned_streams, scenario, forest_variant)
    except KeyboardInterrupt:
        _emit(f"INTERRUPT stage={stage} cleanup=starting")
        process_status = {
            "launch": {"exit_code": None, "cleared": True,
                       "escalated": False},
            "capture": {"exit_code": None, "cleared": True,
                        "escalated": False},
        }
        cleanup_failures = []
        for role, timeout_s in (("launch", 5.0), ("capture", 3.0)):
            process = owned_processes.get(role)
            if process is None:
                continue
            code, cleared, escalated = _stop_group(
                process, timeout_s,
                run_root / f"interrupt_{role}_stacks.txt")
            process_status[role] = {
                "exit_code": code,
                "cleared": cleared,
                "escalated": escalated,
            }
            if not cleared:
                cleanup_failures.append(f"{role}_process_group_remaining")
        for stream in owned_streams.values():
            stream.close()
        run_root.mkdir(parents=True, exist_ok=True)
        summary = _result(
            ["interrupted", *cleanup_failures],
            stage=stage,
            scenario=scenario,
            forest_variant=forest_variant,
            shutdown_variant=shutdown_variant,
            launch_exit_code=process_status["launch"]["exit_code"],
            capture_exit_code=process_status["capture"]["exit_code"],
            launch_group_cleared=process_status["launch"]["cleared"],
            capture_group_cleared=process_status["capture"]["cleared"],
            launch_runner_escalated=process_status["launch"]["escalated"],
            capture_runner_escalated=process_status["capture"]["escalated"],
            elapsed_s=time.monotonic() - started,
        )
        summary["result"] = "INTERRUPTED"
        _json_write(run_root / "summary.json", summary)
        _emit(
            f"INTERRUPTED stage={stage} "
            f"launch_cleared={str(summary['launch_group_cleared']).lower()} "
            f"capture_cleared={str(summary['capture_group_cleared']).lower()}")
        return summary


def _gpu_preflight(root: Path) -> dict:
    path = REPOSITORY / "scripts/dev_planner/run_gate0_qualification.py"
    spec = importlib.util.spec_from_file_location("icra_interface_gpu", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load GPU preflight")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root.mkdir(parents=True, exist_ok=False)
    return module.run_gpu_preflight(root)


def _session_root(results_root: Path) -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    candidate = results_root / f"run-{stamp}-{os.getpid()}"
    candidate.parent.mkdir(parents=True, exist_ok=True)
    return candidate


def _successful_session_result(stages) -> str:
    requested = set(stages)
    return "PASS" if {"full", "shutdown"}.issubset(requested) else "STAGE_PASS"


def summarize_forest_pair(
        baseline: dict, risk: dict, repetition: int) -> dict:
    """Reduce one completed A/B pair to directly comparable evidence."""
    baseline_path = baseline.get("forest_path", {})
    risk_path = risk.get("forest_path", {})
    risk_contrast = risk.get("forest_risk", {})
    baseline_low = baseline_path.get("selected_low_risk_forks")
    risk_low = risk_path.get("selected_low_risk_forks")
    low_delta = (
        int(risk_low) - int(baseline_low)
        if isinstance(baseline_low, int) and isinstance(risk_low, int)
        else None)
    pair_pass = baseline.get("result") == "PASS" and risk.get("result") == "PASS"
    return {
        "repetition": repetition,
        "baseline_result": baseline.get("result"),
        "risk_result": risk.get("result"),
        "baseline_failures": baseline.get("failures", []),
        "risk_failures": risk.get("failures", []),
        "baseline_selected_arms": baseline_path.get("selected_arms", {}),
        "risk_selected_arms": risk_path.get("selected_arms", {}),
        "baseline_selected_low_risk_forks": baseline_low,
        "baseline_selected_high_risk_forks": baseline_path.get(
            "selected_high_risk_forks"),
        "risk_selected_low_risk_forks": risk_low,
        "risk_selected_high_risk_forks": risk_path.get(
            "selected_high_risk_forks"),
        "delta_selected_low_risk_forks": low_delta,
        "risk_contrast_generation_ids": risk_contrast.get(
            "contrast_generation_ids", {}),
        "risk_identity_bound_generation_ids": risk_contrast.get(
            "passing_generation_ids", {}),
        "risk_fork_evidence": risk_contrast.get("fork_evidence", {}),
        "risk_selected_lineage_count": risk.get("selected_count"),
        "risk_lineage_group_count": risk.get("lineage_group_count"),
        "paired_pass": pair_pass,
    }


def _persist_interrupted_session(
        session: Path, session_summary: dict, stage: str) -> int:
    session_summary["result"] = "INTERRUPTED"
    session_summary["interrupted_stage"] = stage
    _json_write(session / "session_summary.json", session_summary)
    _emit(f"INTERRUPTED stage={stage} {session}")
    return 130


def _run_main(args: argparse.Namespace) -> int:
    install_root = args.install_root.resolve()
    if not (install_root / "setup.bash").is_file():
        raise SystemExit(f"install root is not ready: {install_root}")
    requested = args.through or args.stage
    if args.through:
        stages = STAGE_ORDER[:STAGE_ORDER.index(requested) + 1]
    else:
        stages = (requested,)
    scenario = getattr(args, "scenario", DEFAULT_SCENARIO)
    forest_ab = bool(getattr(args, "forest_ab", False))
    results_root = args.results_root.resolve()
    session = _session_root(results_root)
    session.mkdir(parents=True, exist_ok=False)
    _emit(f"SESSION {session}")
    session_summary = {
        "schema_version": "icra_interface_integration_session_v1",
        "development_only": True,
        "qualification_claim": False,
        "scientific_effect_claim": False,
        "stage_order": list(stages),
        "scenario": scenario,
        "forest_ab": forest_ab,
        "forest_scene": (
            forest_scene_contract(scenario)
            if _is_forest_scenario(scenario) else None),
        "repetitions": args.repetitions,
        "runs": [],
        "forest_pairs": [],
        # Do not leave a partial or interrupted --through session looking like
        # a completed acceptance.  PASS is written only after every requested
        # functional and shutdown repetition has finished successfully.
        "result": "RUNNING",
    }
    _json_write(session / "session_summary.json", session_summary)
    current_stage = "preflight"
    try:
        preflight = _gpu_preflight(session / "preflight")
        if preflight.get("gpu_ready") is not True:
            session_summary["result"] = "GPU_NOT_READY"
            _json_write(session / "session_summary.json", session_summary)
            _emit("GPU_NOT_READY")
            return 4
        for stage in stages:
            current_stage = stage
            for repetition in range(1, args.repetitions + 1):
                forest_pair_runs = {}
                if forest_ab and stage == "full":
                    run_variants = (
                        (None, "baseline"), (None, "risk"))
                elif stage == "shutdown":
                    run_variants = (("baseline", None), ("full", None))
                else:
                    forest_variant = (
                        "risk" if _is_forest_scenario(scenario)
                        and stage in ("p4", "p5-final", "full") else None)
                    run_variants = ((None, forest_variant),)
                for shutdown_variant, forest_variant in run_variants:
                    variant = forest_variant or shutdown_variant
                    suffix = f"-{variant}" if variant else ""
                    run_root = session / f"{stage}-r{repetition:02d}{suffix}"
                    rviz_enabled = (
                        args.rviz and stage == "full"
                        and forest_variant != "baseline")
                    duration_s = stage_duration_s(
                        stage, scenario, forest_variant)
                    start_message = (
                        f"START stage={stage} repetition={repetition}/"
                        f"{args.repetitions} duration={duration_s:g}s "
                        f"rviz={'true' if rviz_enabled else 'false'}")
                    if scenario != DEFAULT_SCENARIO or variant is not None:
                        start_message += (
                            f" scenario={scenario} "
                            f"variant={variant or 'default'}")
                    _emit(start_message)
                    _emit(f"LOG {run_root / 'stdout.log'}")
                    summary = _run_one(
                        stage, run_root, install_root,
                        start_rviz=rviz_enabled,
                        shutdown_variant=shutdown_variant,
                        scenario=scenario,
                        forest_variant=forest_variant)
                    session_summary["runs"].append({
                        "stage": stage,
                        "repetition": repetition,
                        "variant": variant,
                        "scenario": scenario,
                        "path": str(run_root),
                        "result": summary["result"],
                        "failures": summary["failures"],
                    })
                    if forest_ab and stage == "full":
                        forest_pair_runs[forest_variant] = summary
                        if forest_variant == "risk":
                            pair = summarize_forest_pair(
                                forest_pair_runs.get("baseline", {}),
                                summary, repetition)
                            session_summary["forest_pairs"].append(pair)
                    _json_write(
                        session / "session_summary.json", session_summary)
                    if summary["result"] == "INTERRUPTED":
                        return _persist_interrupted_session(
                            session, session_summary, stage)
                    if summary["result"] != "PASS":
                        # A paired run remains diagnostically useful only when
                        # both variants complete. Defer a baseline failure
                        # until its matching risk run has written the delta.
                        if (forest_ab and stage == "full"
                                and forest_variant == "baseline"):
                            continue
                        session_summary["result"] = "FAIL"
                        session_summary["first_failed_stage"] = stage
                        _json_write(
                            session / "session_summary.json", session_summary)
                        _emit(f"FAIL {stage} {run_root}")
                        return 1
                    if (forest_ab and stage == "full"
                            and forest_variant == "risk"
                            and forest_pair_runs.get("baseline", {}).get(
                                "result") != "PASS"):
                        session_summary["result"] = "FAIL"
                        session_summary["first_failed_stage"] = stage
                        _json_write(
                            session / "session_summary.json", session_summary)
                        _emit(f"FAIL {stage} paired-baseline")
                        return 1
        current_stage = "session"
        session_summary["functional_gate_complete"] = "full" in stages
        session_summary["shutdown_gate_complete"] = "shutdown" in stages
        session_summary["acceptance_complete"] = {
            "full", "shutdown"}.issubset(stages)
        session_summary["result"] = _successful_session_result(stages)
        _json_write(session / "session_summary.json", session_summary)
        _emit(f"{session_summary['result']} {session}")
        return 0
    except KeyboardInterrupt:
        return _persist_interrupted_session(
            session, session_summary, current_stage)


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--stage", choices=STAGE_CHOICES, default="full")
    mode.add_argument("--through", choices=STAGE_ORDER)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--results-root", type=Path,
                        default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--install-root", type=Path,
                        default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument(
        "--scenario", choices=(DEFAULT_SCENARIO, *FOREST_SCENARIOS),
        default=DEFAULT_SCENARIO)
    parser.add_argument(
        "--forest-ab", action="store_true",
        help=("run paired P4-off baseline and P4/P5-on risk variants; "
              "requires the dense forest full stage"))
    parser.add_argument("--capture-output", type=Path)
    parser.add_argument("--capture-ready", type=Path)
    parser.add_argument("--capture-duration", type=float, default=120.0)
    parser.add_argument(
        "--capture-scenario", choices=(DEFAULT_SCENARIO, *FOREST_SCENARIOS),
        default=DEFAULT_SCENARIO)
    args = parser.parse_args()
    if args.capture_output or args.capture_ready:
        if not args.capture_output or not args.capture_ready:
            raise SystemExit("capture output and ready paths are both required")
        return _capture_main(args)
    if args.repetitions < 1:
        raise SystemExit("repetitions must be positive")
    if args.rviz and (args.stage != "full" or args.through):
        raise SystemExit("--rviz is valid only with --stage full")
    if args.forest_ab and (
            not _is_forest_scenario(args.scenario)
            or args.stage != "full" or args.through):
        raise SystemExit(
            "--forest-ab requires --scenario "
            f"{FOREST_SCENARIO} --stage full")
    try:
        return _run_main(args)
    except KeyboardInterrupt:
        _emit("INTERRUPTED")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
