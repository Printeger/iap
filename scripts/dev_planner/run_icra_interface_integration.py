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


REPOSITORY = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = (
    REPOSITORY / "results/icra27/dev_runs/interface_integration"
).resolve()
DEFAULT_INSTALL_ROOT = (REPOSITORY.parents[1] / "install").resolve()
STAGE_ORDER = ("estimator", "p0", "p4", "p5-final", "full", "shutdown")
DEFAULT_SCENARIO = "icra072_p4_selection_trigger_v1"
FOREST_V1_SCENARIO = "icra_dense_forest_four_fork_v1"
FOREST_SCENARIO = "icra_dense_forest_four_fork_v2"
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
        "mismatches": mismatches,
    }


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
                if decision.get("schema_version") == \
                        "p4_forward_route_decision_v1":
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
        if row.get("schema_version") == "p4_forward_route_decision_v1":
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
    if any(row.get("schema_version") == "p4_forward_route_decision_v1"
           for row in decisions):
        return [
            row for row in decisions
            if row.get("schema_version") == "p4_forward_route_decision_v1"
            and row.get("stage") == "forward_decision"
            and row.get("action") == "RISK_SELECTED"
            and int(row.get("selected_candidate_id", 0) or 0) > 0
            and int(row.get("candidate_count", 0) or 0) >= 2
            and row.get("geometry_id")
            and row.get("alert_limit_policy_id")
            and int(row.get("occupancy_generation", 0) or 0) > 0
            and int(row.get("risk_generation", 0) or 0) > 0
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
    if any(row.get("schema_version") == "p4_forward_route_decision_v1"
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


def analyze_stage_records(
        stage: str, health: list[dict], decisions: list[dict],
        lineage: list[dict], bsplines: list[dict], p5_status: list[dict],
        poscmd_times: list[float], capture_start_s: float | None = None) -> dict:
    if stage not in ("p4", "p5-final", "full"):
        raise ValueError(f"unsupported record stage: {stage}")
    p0 = analyze_p0(health, capture_start_s)
    failures = list(p0["failures"])
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
            if any(row.get("schema_version") ==
                   "p4_forward_route_decision_v1" for row in decisions):
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
    if forward_lineage:
        decisions = forward_lineage
        lineage = forward_lineage
    if stage == "estimator":
        return analyze_estimator(_estimator_metrics(run_root, records))
    if stage == "p0":
        return analyze_p0(health, stage_start)
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
    from iap.msg import IntegrityReport
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
            })

        def poscmd(self, message: PositionCommand) -> None:
            self.record("poscmd", {
                "position_xyz": [
                    float(message.position.x), float(message.position.y),
                    float(message.position.z),
                ],
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
    """Audit the live planner graph; simulators may still consume truth."""
    target = "/drone_0_ego_planner_node"
    deadline = time.monotonic() + timeout_s
    nodes: set[str] = set()
    while time.monotonic() < deadline:
        nodes = _node_names(environment)
        if target in nodes or target.lstrip("/") in nodes:
            break
        time.sleep(0.25)
    matched = target if target in nodes else target.lstrip("/")
    if matched not in nodes:
        return {
            "schema_version": "planner_truth_isolation_audit_v1",
            "pass": False,
            "failures": ["planner_node_missing_for_truth_audit"],
            "audited_nodes": [],
            "forbidden_subscriptions": [],
        }
    subscriptions, error = _node_subscriptions(matched, environment)
    forbidden = sorted(topic for topic in subscriptions if topic.startswith(
        ("/map_generator/", "/sim/world/")))
    failures = (["planner_truth_subscription_detected"] if forbidden else [])
    if error:
        failures.append("planner_graph_audit_error")
    return {
        "schema_version": "planner_truth_isolation_audit_v1",
        "pass": not failures,
        "failures": failures,
        "audited_nodes": [{
            "name": matched,
            "subscriptions": subscriptions,
        }],
        "forbidden_prefixes": ["/map_generator/", "/sim/world/"],
        "forbidden_subscriptions": forbidden,
        "stderr": error,
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
    summary.update({
        "stage": stage,
        "scenario": scenario,
        "forest_variant": forest_variant,
        "forest_scene": (
            forest_scene_contract(scenario)
            if _is_forest_scenario(scenario) else None),
        "forest_effective_manifest": forest_manifest,
        "planner_truth_isolation_audit": graph_audit,
        "scene_cloud_bbox": next((
            row.get("payload") for row in _read_jsonl(run_root / "capture.jsonl")
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
    mode.add_argument("--stage", choices=STAGE_ORDER, default="full")
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
