#!/usr/bin/env python3
"""Pure, deterministic ICRA-075 scene, analysis and power-input seams."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
import statistics
from pathlib import Path
from typing import Any


REPOSITORY = Path(__file__).resolve().parents[2]
PROTOCOL_PATH = REPOSITORY / "config/icra27/icra075_exploratory_protocol_v1.json"
MAP_ASSET_PATH = (REPOSITORY /
                  "src/uav_simulator/map_generator/config/icra075_inverse_corridor_v2.json")
GNSS_ASSET_PATH = (REPOSITORY /
                   "src/uav_simulator/gnss_sim/config/icra075_inverse_corridor_provider_v2.json")
FIXTURE_TOOL_PATH = REPOSITORY / "scripts/dev_planner/icra073_inverse_corridor_fixture.py"
STAGE_ORDER = (
    ("p0_snapshot", "P0_SNAPSHOT_MISSING"),
    ("p4_selection", "P4_SELECTION_MISSING"),
    ("ego_final", "EGO_FINAL_MISSING"),
    ("p5_final", "P5_FINAL_MISSING"),
    ("publication", "NORMAL_PUBLICATION_MISSING"),
    ("p5_runtime", "P5_RUNTIME_MISSING"),
)


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def known_retained_inventory() -> list[dict[str, Any]]:
    expected = (
        (".claude/settings.local.json", 72,
         "27aac0ccca0ad0ab573578864cf27b9560d3f819bdeeae62378f8c20e62a8f64"),
        ("src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~", 2962,
         "29a73228c1d58d5c4983dc217230d6cbc6ff6e295a77a004d8e6a207fd242028"),
        ("docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf", 243368,
         "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6"),
    )
    inventory = []
    for relative, size, digest in expected:
        path = REPOSITORY / relative
        observed_size = path.stat().st_size if path.is_file() and not path.is_symlink() else None
        observed_hash = _sha256(path) if observed_size is not None else None
        inventory.append({
            "path": relative, "expected_size_bytes": size, "observed_size_bytes": observed_size,
            "expected_sha256": digest, "observed_sha256": observed_hash,
            "accepted": observed_size == size and observed_hash == digest,
        })
    return inventory


def _load_fixture_tool():
    spec = importlib.util.spec_from_file_location("icra075_fixture_tool", FIXTURE_TOOL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load frozen V2 fixture tool")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_protocol(path: Path = PROTOCOL_PATH) -> dict[str, Any]:
    protocol = _json(path)
    if protocol.get("schema_version") != "icra075_exploratory_protocol_v1":
        raise ValueError("ICRA-075 protocol schema mismatch")
    if protocol.get("development_only") is not True or protocol.get("formal_freeze") is not False:
        raise ValueError("ICRA-075 protocol must remain development-only and non-freezing")
    seeds = protocol.get("development_seeds")
    if seeds != list(range(75001, 75006)) or seeds != protocol.get("future_held_out_excluded_seeds"):
        raise ValueError("ICRA-075 seed exclusion mismatch")
    return protocol


def load_v2_descriptor(variant: str) -> dict[str, Any]:
    return _load_fixture_tool().build_v2_descriptor(variant)


def load_map_asset() -> dict[str, Any]:
    return _json(MAP_ASSET_PATH)


def build_matrix(protocol: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for scene in protocol["scenes"]:
        descriptor = load_v2_descriptor(scene)
        for seed in protocol["development_seeds"]:
            for configuration in protocol["formal_configurations"]:
                rows.append(_matrix_row(protocol, descriptor, scene, seed,
                                        configuration, "FORMAL_ARM"))
    primary = load_v2_descriptor("PRIMARY")
    for seed in protocol["development_seeds"]:
        for configuration in protocol["primary_exploratory_configurations"]:
            rows.append(_matrix_row(protocol, primary, "PRIMARY", seed,
                                    configuration, "EXPLORATORY_ABLATION"))
    if len(rows) != 40 or len({row["run_id"] for row in rows}) != 40:
        raise ValueError("ICRA-075 matrix cardinality mismatch")
    return rows


def _matrix_row(protocol: dict[str, Any], descriptor: dict[str, Any], scene: str,
                seed: int, configuration: str, kind: str) -> dict[str, Any]:
    token = configuration.lower()
    return {
        "run_id": f"{scene.lower()}-{seed}-{token}",
        "scene": scene,
        "scene_identity": descriptor["scene_identity"],
        "descriptor_sha256": descriptor["descriptor_sha256"],
        "seed": seed,
        "held_out_eligible": False,
        "configuration": configuration,
        "comparison_kind": kind,
        "start_m": copy.deepcopy(protocol["start_m"]),
        "goal_m": copy.deepcopy(protocol["goal_m"]),
        "overrides": copy.deepcopy(protocol["configuration_overrides"][configuration]),
    }


def validate_scene_assets() -> dict[str, Any]:
    map_asset = _json(MAP_ASSET_PATH)
    gnss_asset = _json(GNSS_ASSET_PATH)
    descriptors = {variant: load_v2_descriptor(variant)
                   for variant in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL")}
    observed = {key: value["descriptor_sha256"] for key, value in descriptors.items()}
    provider_binding = gnss_asset.get("icra075_binding", {})
    forbidden_names = {
        "centre_line", "centre_lines", "tube_label", "tube_membership",
        "oracle_risk", "expected_route", "analysis_output",
    }
    present: set[str] = set()

    def visit(value: Any) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                if key in forbidden_names:
                    present.add(key)
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(map_asset)
    visit(gnss_asset)
    result = (
        map_asset.get("schema_version") == "icra075_runtime_map_fixture_v1"
        and map_asset.get("design_record") == "ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V2"
        and map_asset.get("descriptor_sha256") == observed
        and provider_binding.get("descriptor_sha256") == observed
        and provider_binding.get("finite_complete_route_support") == {
            "PRIMARY": True, "EXACT_MIRROR": True, "FLAT_NULL": True}
        and provider_binding.get("gnss_mask_enabled") ==
        map_asset.get("gnss_only_mask", {}).get("enabled")
        and provider_binding.get("gnss_mask_enabled", {}).get("FLAT_NULL") is False
        and sorted(map_asset.get("decision_plane_inputs", [])) ==
        ["immutable_p0_snapshot", "ordinary_occupancy"]
        and not present
        and gnss_asset.get("faults") == []
    )
    return {
        "schema_version": "icra075_scene_asset_binding_v1",
        "result": "PASS" if result else "FAIL",
        "descriptor_sha256": observed,
        "map_asset_sha256": _sha256(MAP_ASSET_PATH),
        "gnss_asset_sha256": _sha256(GNSS_ASSET_PATH),
        "decision_plane_inputs": sorted(map_asset.get("decision_plane_inputs", [])),
        "runtime_asset_contains_route_or_oracle_truth": any(
            key in json.dumps(gnss_asset, sort_keys=True)
            for key in ('"safe"', '"risky"', '"provider_truth"', '"oracle"')),
        "gnss_mask_enabled": provider_binding.get("gnss_mask_enabled"),
        "forbidden_decision_fields_present": sorted(present),
    }


def _de_boor(control: list[list[float]], knots: list[float], degree: int,
             parameter: float) -> list[float]:
    count = len(control)
    if degree < 1 or len(knots) != count + degree + 1:
        raise ValueError("invalid committed B-spline degree/knot cardinality")
    low, high = knots[degree], knots[-degree - 1]
    if parameter <= low:
        return list(control[0])
    if parameter >= high:
        return list(control[-1])
    span = degree
    for index in range(degree, count):
        if knots[index] <= parameter < knots[index + 1]:
            span = index
            break
    work = [list(control[span - degree + index]) for index in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            knot_index = span - degree + index
            denominator = knots[knot_index + degree - level + 1] - knots[knot_index]
            alpha = 0.0 if denominator == 0.0 else (parameter - knots[knot_index]) / denominator
            work[index] = [(1.0 - alpha) * a + alpha * b
                           for a, b in zip(work[index - 1], work[index])]
    return work[degree]


def equal_arc_samples(final: dict[str, Any], count: int = 200,
                      return_evidence: bool = False):
    if count != 200:
        raise ValueError("ICRA-075 analysis requires exactly 200 samples")
    control = final.get("position_control_points")
    knots = final.get("knots")
    degree = final.get("order")
    if (not isinstance(control, list) or len(control) < 2 or
            not all(isinstance(point, list) and len(point) == 3 for point in control) or
            not isinstance(knots, list) or type(degree) is not int):
        raise ValueError("committed final B-spline payload incomplete")
    low, high = float(knots[degree]), float(knots[-degree - 1])
    interior = knots[degree + 1:-degree - 1]
    if any(interior.count(value) >= degree for value in set(interior)):
        raise ValueError("equal-arc certificate requires a continuously differentiable spline")
    velocity_control, velocity_knots, velocity_degree = _derivative_curve(
        control, knots, degree)
    if degree >= 2:
        acceleration_control, _, _ = _derivative_curve(
            velocity_control, velocity_knots, velocity_degree)
        speed_lipschitz = max(
            math.sqrt(sum(component * component for component in vector))
            for vector in acceleration_control)
    else:
        speed_lipschitz = 0.0
    requested_position_error_bound = 2.0e-4
    integration_budget = requested_position_error_bound / 2.0
    interval_count = max(4096, math.ceil(
        speed_lipschitz * (high - low) ** 2 / (4.0 * integration_budget)))
    if interval_count > 2_000_000:
        raise ValueError("equal-arc certificate interval budget exceeded")
    parameters = [low + (high - low) * index / interval_count
                  for index in range(interval_count + 1)]
    dense = [_de_boor(control, knots, degree, value) for value in parameters]
    interval_width = (high - low) / interval_count

    def speed(parameter: float) -> float:
        vector = (velocity_control[0] if velocity_degree == 0 else
                  _de_boor(velocity_control, velocity_knots,
                           velocity_degree, parameter))
        return math.sqrt(sum(component * component for component in vector))

    cumulative = [0.0]
    for index in range(interval_count):
        midpoint = (parameters[index] + parameters[index + 1]) * 0.5
        cumulative.append(cumulative[-1] + speed(midpoint) * interval_width)
    total = cumulative[-1]
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("committed final B-spline has non-positive arc length")
    samples: list[list[float]] = []
    cursor = 0
    for index in range(count):
        target = total * index / (count - 1)
        while cursor + 1 < len(cumulative) and cumulative[cursor + 1] < target:
            cursor += 1
        if cursor + 1 == len(cumulative):
            samples.append(list(dense[-1]))
            continue
        width = cumulative[cursor + 1] - cumulative[cursor]
        alpha = 0.0 if width == 0.0 else (target - cumulative[cursor]) / width
        parameter = ((1.0 - alpha) * parameters[cursor] +
                     alpha * parameters[cursor + 1])
        samples.append(_de_boor(control, knots, degree, parameter))
    samples[0], samples[-1] = list(dense[0]), list(dense[-1])
    evidence = {
        "method": "certified_midpoint_speed_inversion_v1",
        "arc_position_error_bound_m": (
            speed_lipschitz * (high - low) ** 2 / (2.0 * interval_count)),
        "certificate_basis": "speed_Lipschitz_bound_from_second_derivative_control_hull",
        "speed_lipschitz_bound_mps2": speed_lipschitz,
        "integration_interval_count": interval_count,
        "sample_count": count,
        "includes_endpoints": True,
    }
    return (samples, evidence) if return_evidence else samples


def _derivative_curve(control: list[list[float]], knots: list[float], degree: int):
    derived = []
    for index in range(len(control) - 1):
        denominator = knots[index + degree + 1] - knots[index + 1]
        scale = 0.0 if denominator == 0.0 else degree / denominator
        derived.append([(b - a) * scale for a, b in zip(control[index], control[index + 1])])
    return derived, knots[1:-1], degree - 1


def _distance_to_box(point: list[float], bounds: dict[str, list[float]]) -> float:
    squared = 0.0
    for coordinate, axis in zip(point, ("x", "y", "z")):
        low, high = bounds[axis]
        delta = low - coordinate if coordinate < low else coordinate - high if coordinate > high else 0.0
        squared += delta * delta
    return math.sqrt(squared)


def evaluate_committed_final_safety(final: dict[str, Any], map_asset: dict[str, Any],
                                    max_velocity_mps: float,
                                    max_acceleration_mps2: float,
                                    feasibility_tolerance: float) -> dict[str, Any]:
    control = final.get("position_control_points")
    knots = final.get("knots")
    degree = final.get("order")
    if not isinstance(control, list) or not isinstance(knots, list) or type(degree) is not int:
        raise ValueError("committed final is incomplete for safety evaluation")
    low, high = float(knots[degree]), float(knots[-degree - 1])
    parameters = [low + (high - low) * index / 8000 for index in range(8001)]
    positions = [_de_boor(control, knots, degree, value) for value in parameters]
    clearance = 0.35
    minimum_clearance = min(
        _distance_to_box(point, map_asset["central_cuboid_bounds_m"])
        for point in positions)
    cylinders = [tuple(item[:2]) for item in map_asset["outer_tree_centres_m"]]
    cylinders += [tuple(item[:2]) for pair in map_asset["lidar_landmark_pairs_m"] for item in pair]
    cylinder_radius = max(float(map_asset["outer_tree_trunk_radius_m"]),
                          float(map_asset["lidar_landmark_radius_m"]))
    for point in positions:
        if 0.0 <= point[2] <= 3.0:
            for x, y in cylinders:
                minimum_clearance = min(minimum_clearance,
                                        math.hypot(point[0] - x, point[1] - y) - cylinder_radius)
    velocity_control, velocity_knots, velocity_degree = _derivative_curve(control, knots, degree)
    velocities = ([_de_boor(velocity_control, velocity_knots, velocity_degree, value)
                   for value in parameters] if velocity_degree >= 1 else velocity_control)
    maximum_velocity = max(math.sqrt(sum(component * component for component in item))
                           for item in velocities)
    if degree >= 2:
        acceleration_control, acceleration_knots, acceleration_degree = _derivative_curve(
            velocity_control, velocity_knots, velocity_degree)
        accelerations = ([_de_boor(acceleration_control, acceleration_knots,
                                   acceleration_degree, value) for value in parameters]
                         if acceleration_degree >= 1 else acceleration_control)
        maximum_acceleration = max(math.sqrt(sum(component * component for component in item))
                                   for item in accelerations)
    else:
        maximum_acceleration = 0.0
    return {
        "collision_free": minimum_clearance >= clearance,
        "minimum_ordinary_occupancy_clearance_m": minimum_clearance,
        "required_clearance_m": clearance,
        "collision_source": "independent_v2_geometry_evaluator",
        "dynamics_feasible": (
            maximum_velocity <= max_velocity_mps * (1.0 + feasibility_tolerance) and
            maximum_acceleration <= max_acceleration_mps2 * (1.0 + feasibility_tolerance)),
        "maximum_velocity_mps": maximum_velocity,
        "maximum_acceleration_mps2": maximum_acceleration,
        "velocity_limit_mps": max_velocity_mps,
        "acceleration_limit_mps2": max_acceleration_mps2,
        "feasibility_tolerance": feasibility_tolerance,
        "dynamics_source": "exact_bspline_derivative_evaluator",
    }


def validate_complete_analysis(row: dict[str, Any]) -> None:
    required_finite = ("minimum_al_minus_pl_m", "p4_search_latency_ms")
    if any(not isinstance(row.get(key), (int, float)) or
           not math.isfinite(float(row[key])) for key in required_finite):
        raise ValueError("analysis has missing/non-finite margin or latency")
    if row.get("collision_free") is not True or row.get("dynamics_feasible") is not True:
        raise ValueError("analysis safety is incomplete or failed")


def _curve_distance(point: list[float], amplitude: float) -> float:
    best = math.inf
    for index in range(1201):
        u = index / 1200.0
        curve = [-12.0 + 24.0 * u, amplitude * math.sin(math.pi * u), 1.5]
        best = min(best, math.dist(point, curve))
    return best


def analyze_committed_final(descriptor: dict[str, Any], final: dict[str, Any],
                            p5_final: dict[str, Any],
                            p5_runtime: dict[str, Any]) -> dict[str, Any]:
    trajectory_id = final.get("trajectory_id")
    start_ns = final.get("start_time_ns")
    if (type(trajectory_id) is not int or trajectory_id <= 0 or
            type(start_ns) is not int or start_ns <= 0 or
            p5_runtime.get("trajectory_id") != trajectory_id or
            p5_runtime.get("trajectory_start_time_ns") != start_ns):
        raise ValueError("committed final/P5 runtime identity mismatch")
    positions, arc_evidence = equal_arc_samples(final, 200, return_evidence=True)
    safe_amp = descriptor["centre_lines"]["safe"]["amplitude_y_m"]
    risky_amp = descriptor["centre_lines"]["risky"]["amplitude_y_m"]
    tube_radius = descriptor["tube_radius_m"]
    values = descriptor["provider_truth"]["route_interior_values"]
    x_low, x_high = descriptor["provider_truth"]["controllable_x_interval_inclusive_m"]
    rows = []
    interior_values = []
    whole_values = []
    safe_count = risky_count = 0
    safe_excursion = 0.0
    for index, position in enumerate(positions):
        safe_distance = _curve_distance(position, safe_amp)
        risky_distance = _curve_distance(position, risky_amp)
        route = "safe" if safe_distance <= risky_distance else "risky"
        provider = float(values[route])
        interior = x_low <= position[0] <= x_high
        if interior:
            interior_values.append(provider)
        whole_values.append(provider)
        safe_count += safe_distance <= tube_radius
        risky_count += risky_distance <= tube_radius
        safe_excursion = max(safe_excursion, safe_distance)
        rows.append({
            "sample_index": index,
            "arc_fraction": index / 199.0,
            "position_m": [float(value) for value in position],
            "controllable_interior": interior,
            "provider_only_risk": provider,
            "safe_centre_line_distance_m": safe_distance,
            "risky_centre_line_distance_m": risky_distance,
        })
    if not interior_values:
        raise ValueError("committed final has no controllable-interior samples")
    safe_fraction, risky_fraction = safe_count / 200.0, risky_count / 200.0
    route_label = (
        "SAFE_TUBE" if safe_fraction > risky_fraction and safe_fraction >= 0.5
        else "RISKY_TUBE" if risky_fraction > safe_fraction and risky_fraction >= 0.5
        else "MIXED_OR_OUTSIDE")
    return {
        "schema_version": "p4_v2_inverse_corridor_analysis_v1",
        "result": "PASS",
        "descriptor_sha256": descriptor["descriptor_sha256"],
        "scene": descriptor["scene_variant"],
        "trajectory_source": "committed_final_bspline_only",
        "trajectory_id": trajectory_id,
        "trajectory_start_time_ns": start_ns,
        "final_bspline_identity": final.get("final_bspline_identity"),
        "publication_identity": final.get("publication_identity"),
        "sample_count": 200,
        "equal_arc_evidence": arc_evidence,
        "samples": rows,
        "provider_interior_peak": max(interior_values),
        "provider_interior_mean": statistics.fmean(interior_values),
        "whole_path_provider_peak": max(whole_values),
        "path_length_m": sum(math.dist(a, b) for a, b in zip(positions, positions[1:])),
        "safe_tube_fraction": safe_fraction,
        "risky_tube_fraction": risky_fraction,
        "maximum_safe_centre_line_excursion_m": safe_excursion,
        "route_label": route_label,
        "minimum_al_minus_pl_m": p5_final.get("min_al_minus_pl_m"),
        "collision_free": final.get("collision_free") is True,
        "dynamics_feasible": final.get("dynamics_feasible") is True,
        "p4_search_latency_ms": final.get("p4_search_latency_ms"),
        "p5_final_action": p5_final.get("action"),
        "p5_runtime_action": p5_runtime.get("action"),
        "oracle_forbidden_p4_inputs_consumed": [],
    }


def pair_identity_matches(first: dict[str, Any], second: dict[str, Any]) -> bool:
    fields = ("scene", "scene_identity", "descriptor_sha256", "seed", "start_m", "goal_m")
    return all(first.get(field) == second.get(field) for field in fields)


def ablation_isolation_passes(rows: list[dict[str, Any]], protocol: dict[str, Any]) -> bool:
    allowed = set(protocol["ablation_allowed_override_keys"])
    expected = set(protocol["primary_exploratory_configurations"])
    if {row.get("configuration") for row in rows} != expected:
        return False
    for row in rows:
        if row.get("scene") != "PRIMARY" or set(row.get("overrides", {})) - allowed:
            return False
        if any(key not in {
                "run_id", "scene", "scene_identity", "descriptor_sha256", "seed",
                "held_out_eligible", "configuration", "comparison_kind", "start_m",
                "goal_m", "overrides"} for key in row):
            return False
    return True


def first_missing_stage(stage_status: dict[str, bool]) -> str | None:
    return next((reason for stage, reason in STAGE_ORDER
                 if stage_status.get(stage) is not True), None)


def _sample_variance(values: list[float]) -> float:
    return statistics.variance(values) if len(values) > 1 else 0.0


def _correlation(first: list[float], second: list[float]) -> float | None:
    if len(first) < 2:
        return None
    first_mean, second_mean = statistics.fmean(first), statistics.fmean(second)
    numerator = sum((a - first_mean) * (b - second_mean) for a, b in zip(first, second))
    denominator = math.sqrt(sum((a - first_mean) ** 2 for a in first) *
                            sum((b - second_mean) ** 2 for b in second))
    return None if denominator == 0.0 else numerator / denominator


def compute_power_inputs(analyses: list[dict[str, Any]]) -> dict[str, Any]:
    if not analyses:
        raise ValueError("no exploratory analyses")
    indexed = {(row["scene"], row["seed"], row["configuration"]): row
               for row in analyses if row.get("result") == "PASS"}
    pairs = []
    for scene in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL"):
        for seed in range(75001, 75006):
            control = indexed.get((scene, seed, "P0_P5_CONTROL"))
            treatment = indexed.get((scene, seed, "P0_P4_V2_P5_TREATMENT"))
            if control is None or treatment is None:
                raise ValueError(f"missing formal pair: {scene}/{seed}")
            validate_complete_analysis(control)
            validate_complete_analysis(treatment)
            pairs.append({
                "scene": scene, "seed": seed,
                "D_peak": control["provider_interior_peak"] - treatment["provider_interior_peak"],
                "D_mean": control["provider_interior_mean"] - treatment["provider_interior_mean"],
                "length_delta_m": treatment["path_length_m"] - control["path_length_m"],
                "latency_delta_ms": treatment["p4_search_latency_ms"] - control["p4_search_latency_ms"],
                "minimum_treatment_al_minus_pl_m": treatment["minimum_al_minus_pl_m"],
                "safety_complete": all((control["collision_free"], treatment["collision_free"],
                                        control["dynamics_feasible"], treatment["dynamics_feasible"])),
            })
    summaries = {}
    for scene in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL"):
        selected = [row for row in pairs if row["scene"] == scene]
        peak = [row["D_peak"] for row in selected]
        mean = [row["D_mean"] for row in selected]
        control_peaks = [indexed[(scene, row["seed"], "P0_P5_CONTROL")]
                         ["provider_interior_peak"] for row in selected]
        treatment_peaks = [indexed[(scene, row["seed"], "P0_P4_V2_P5_TREATMENT")]
                           ["provider_interior_peak"] for row in selected]
        zero_tolerance = 1.0e-12
        summaries[scene] = {
            "D_peak_mean": statistics.fmean(peak),
            "D_peak_between_seed_variance": _sample_variance(peak),
            "D_mean_mean": statistics.fmean(mean),
            "D_mean_between_seed_variance": _sample_variance(mean),
            "length_delta_mean_m": statistics.fmean(row["length_delta_m"] for row in selected),
            "latency_delta_mean_ms": statistics.fmean(row["latency_delta_ms"] for row in selected),
            "all_safety_complete": all(row["safety_complete"] for row in selected),
            "D_peak_zero_mass": sum(abs(value) <= zero_tolerance for value in peak) / len(peak),
            "D_mean_zero_mass": sum(abs(value) <= zero_tolerance for value in mean) / len(mean),
            "repeatability": ({
                "status": "AVAILABLE_FLAT_NULL_PROXY",
                "definition": "absolute paired effect under equal provider truth",
                "absolute_D_peak_distribution": [abs(value) for value in peak],
                "absolute_D_mean_distribution": [abs(value) for value in mean],
                "maximum_absolute_D_peak": max(abs(value) for value in peak),
                "maximum_absolute_D_mean": max(abs(value) for value in mean),
            } if scene == "FLAT_NULL" else {
                "status": "UNAVAILABLE_NO_BYTE_IDENTICAL_REPLAY",
                "reason": "one execution per scene/seed/configuration",
            }),
            "intra_run_correlation": {
                "status": "UNAVAILABLE_SINGLE_TERMINAL_EVENT_PER_RUN",
                "reason": "no repeated within-run effect observations",
            },
            "matched_arm_cross_seed_correlation": {
                "control_treatment_peak": _correlation(control_peaks, treatment_peaks),
                "estimator": "pearson_across_matched_development_seeds",
            },
            "candidate_n_range": [30, 60],
        }
    sensitivity = {}
    for scene in ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL"):
        scene_values = [row["D_peak"] for row in pairs if row["scene"] == scene]
        observed_sd = math.sqrt(_sample_variance(scene_values))
        observed_scale = abs(statistics.fmean(scene_values))
        sensitivity[scene] = []
        for fraction in (0.5, 0.75, 1.0):
            candidate_effect = observed_scale * fraction
            approximate_n = (60 if candidate_effect <= 0.0 else math.ceil(
                ((1.6448536269514722 + 1.2815515655446004) * observed_sd /
                 candidate_effect) ** 2))
            sensitivity[scene].append({
                "observed_effect_fraction": fraction,
                "candidate_effect": candidate_effect,
                "unclamped_normal_approximation_n": approximate_n,
                "candidate_n_within_route_range": min(60, max(30, approximate_n)),
            })
    return {
        "schema_version": "icra075_exploratory_power_inputs_v1",
        "development_only": True,
        "effect_claim": False,
        "freezes_sesoi": False,
        "freezes_threshold": False,
        "freezes_sample_size": False,
        "success_verdict": "NOT_EVALUATED_EXPLORATORY_ONLY",
        "formal_pair_count": len(pairs),
        "pairs": pairs,
        "scene_summaries": summaries,
        "mirror_consistency": summaries["PRIMARY"]["D_peak_mean"] == summaries["EXACT_MIRROR"]["D_peak_mean"],
        "flat_null_consistency": abs(summaries["FLAT_NULL"]["D_peak_mean"]) <= 1e-12,
        "candidate_confirmatory_sample_size_range": [30, 60],
        "sensitivity": sensitivity,
        "insufficiency": [
            "five development seeds per scene are not confirmatory",
            "SESOI, threshold and exact sample size remain unfrozen",
            "ICRA-072B/073 source-admission and paired-runtime debt remains NOT PASS"
        ]
    }
