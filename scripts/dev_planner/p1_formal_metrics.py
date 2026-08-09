#!/usr/bin/env python3
"""Shared fixed-lattice metrics for P1 formal calibration and acceptance.

The smooth-CVaR implementation intentionally mirrors
``BsplineOptimizer::calcP1IntegrityCost`` one operation at a time: the same
alpha/temperature defaults, 64*T bracket, 100 bisection iterations, stable
sigmoid/softplus branches, and tied-profile normalization are used here.
"""

from __future__ import annotations

from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable


FIXED_SAMPLE_COUNT = 200
SMOOTH_CVAR_ALPHA = 0.90
SMOOTH_CVAR_TEMPERATURE = 0.01
ETA_BISECTION_ITERATIONS = 100
P0_CONFIGURATION_KEYS = (
    "p0.enable_risk_grid",
    "p0.raw_health_topic",
    "p0.resolution_m",
    "p0.size_x_m",
    "p0.size_y_m",
    "p0.size_z_m",
    "p0.horizons_s",
    "p0.refresh_period_s",
    "p0.stale_timeout_s",
    "p0.batch_worker_count",
    "p0.predictor.requested_worker_count",
    "p0.predictor.effective_worker_count",
    "p0.skip_occupied_voxels",
    "p0.predictor.source_mode",
    "p0.predictor.gnss_epoch_policy",
    "p0.predictor.use_current_integrity_prior",
    "p0.predictor.conservative_max_with_gnss",
    "p0.predictor.lidar_legacy_observability",
    "p0.predictor.lidar_fim_radius_m",
)


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def select_decision_checkpoint(
    truth_rows: Iterable[dict[str, Any]],
    estimate_rows: Iterable[dict[str, Any]],
    *,
    target_x_m: float = -9.5,
    x_tolerance_m: float = 0.4,
    localization_limit_m: float = 0.5,
) -> dict[str, Any]:
    """Select the one deterministic truth-progress checkpoint, fail closed."""
    candidates = []
    for row in truth_rows:
        values = {key: _finite(row.get(key)) for key in ("stamp_s", "x", "y", "z")}
        if all(value is not None for value in values.values()) and abs(
            float(values["x"]) - target_x_m
        ) <= x_tolerance_m:
            candidates.append({key: float(value) for key, value in values.items()})
    if len(candidates) != 1:
        return {
            "passed": False, "status": "INCONCLUSIVE",
            "reason": "decision_checkpoint_not_unique",
            "matching_truth_count": len(candidates),
        }
    truth = candidates[0]
    estimates = []
    for row in estimate_rows:
        values = {key: _finite(row.get(key)) for key in ("stamp_s", "x", "y", "z")}
        if all(value is not None for value in values.values()):
            estimates.append({key: float(value) for key, value in values.items()})
    if not estimates:
        return {
            "passed": False, "status": "INCONCLUSIVE",
            "reason": "decision_checkpoint_estimate_missing", "truth": truth,
        }
    estimate = min(estimates, key=lambda row: abs(row["stamp_s"] - truth["stamp_s"]))
    error = math.dist(
        (truth["x"], truth["y"], truth["z"]),
        (estimate["x"], estimate["y"], estimate["z"]),
    )
    return {
        "passed": error <= localization_limit_m,
        "status": "PASS" if error <= localization_limit_m else "FAIL",
        "reason": "ok" if error <= localization_limit_m else "localization_error_exceeded",
        "truth": truth, "estimate": estimate,
        "localization_error_m": error,
        "localization_limit_m": localization_limit_m,
    }


def localization_pair_gate(
    current: dict[str, Any], reference: dict[str, Any], *, delta_limit_m: float = 0.25
) -> dict[str, Any]:
    current_error = _finite(current.get("localization_error_m"))
    reference_error = _finite(reference.get("localization_error_m"))
    delta = (
        abs(float(current_error) - float(reference_error))
        if current_error is not None and reference_error is not None else math.inf
    )
    passed = bool(current.get("passed")) and bool(reference.get("passed")) and delta <= delta_limit_m
    return {"passed": passed, "error_delta_m": delta, "delta_limit_m": delta_limit_m}


def candidate_route_precheck(
    rows: Iterable[dict[str, Any]], *, support_count: int = FIXED_SAMPLE_COUNT
) -> dict[str, Any]:
    """Summarize collision-feasible full-support upper/lower candidates."""
    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in rows:
        candidate_id = str(row.get("candidate_id", "")).strip()
        if candidate_id:
            grouped.setdefault(candidate_id, []).append(row)
    lane_candidates: dict[str, list[dict[str, Any]]] = {"lower": [], "upper": []}
    selected: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    required_indices = set(range(support_count))
    for candidate_id, candidate_rows in grouped.items():
        explicit_lanes = {str(row.get("lane", "")).strip().lower() for row in candidate_rows}
        explicit_lanes.discard("")
        if len(explicit_lanes) == 1:
            lane = explicit_lanes.pop()
        else:
            y_values = [_finite(row.get("y")) for row in candidate_rows]
            finite_y = [float(value) for value in y_values if value is not None]
            lane = "upper" if finite_y and sum(finite_y) / len(finite_y) > 0.0 else "lower"
        indices = {_finite(row.get("sample_index")) for row in candidate_rows}
        integer_indices = {int(value) for value in indices if value is not None and value.is_integer()}
        values = [_finite(row.get("c_pi")) for row in candidate_rows]
        costs = [float(value) for value in values if value is not None]
        full_support = (
            len(candidate_rows) == support_count
            and integer_indices == required_indices
            and len(costs) == support_count
            and all(_usable(row) for row in candidate_rows)
        )
        collision_free = all(_truthy(row.get("collision_free")) for row in candidate_rows)
        summary = {
            "candidate_id": candidate_id, "lane": lane,
            "collision_free": collision_free, "full_support": full_support,
            "sample_count": len(candidate_rows),
        }
        if full_support:
            summary.update({
                "mean": sum(costs) / support_count,
                "cvar": smooth_cvar(costs), "max": max(costs),
            })
        if lane not in lane_candidates or not collision_free or not full_support:
            rejected.append(summary)
            continue
        lane_candidates[lane].append(summary)
        if any(_truthy(row.get("selected")) for row in candidate_rows):
            selected.append(summary)
    lane_summary = {}
    for lane, candidates in lane_candidates.items():
        lane_summary[lane] = {
            "candidate_count": len(candidates),
            "mean_risk": (sum(item["mean"] for item in candidates) / len(candidates)) if candidates else None,
            "mean_cvar": (sum(item["cvar"] for item in candidates) / len(candidates)) if candidates else None,
            "max_risk": max((item["max"] for item in candidates), default=None),
            "candidates": candidates,
        }
    passed = all(lane_summary[lane]["candidate_count"] > 0 for lane in ("lower", "upper")) and len(selected) == 1
    return {
        "passed": passed, "status": "PASS" if passed else "INCONCLUSIVE",
        "lanes": lane_summary, "selected": selected[0] if len(selected) == 1 else None,
        "rejected": rejected,
    }


def _stable_sigmoid(value: float) -> float:
    if value >= 0.0:
        if value > 709.0:
            return 1.0
        return 1.0 / (1.0 + math.exp(-value))
    if value < -745.0:
        return 0.0
    exponential = math.exp(value)
    return exponential / (1.0 + exponential)


def _stable_softplus(value: float) -> float:
    if value > 0.0:
        return value + (0.0 if value > 745.0 else math.log1p(math.exp(-value)))
    return 0.0 if value < -745.0 else math.log1p(math.exp(value))


def smooth_cvar(
    values: Iterable[float],
    *,
    alpha: float = SMOOTH_CVAR_ALPHA,
    temperature: float = SMOOTH_CVAR_TEMPERATURE,
) -> float:
    """Return the production-equivalent entropy-normalized smooth CVaR."""
    costs = [float(value) for value in values]
    if not costs or not all(math.isfinite(value) for value in costs):
        raise ValueError("smooth CVaR requires finite non-empty values")
    if not 0.0 < alpha < 1.0:
        raise ValueError("smooth CVaR alpha must be in (0, 1)")
    if not math.isfinite(temperature) or temperature <= 0.0:
        raise ValueError("smooth CVaR temperature must be positive and finite")
    minimum_cost = min(costs)
    maximum_cost = max(costs)
    if minimum_cost == maximum_cost:
        return minimum_cost

    tail_probability = 1.0 - alpha
    target_tail_mass = tail_probability * len(costs)
    eta_lower = minimum_cost - 64.0 * temperature
    eta_upper = maximum_cost + 64.0 * temperature
    for _ in range(ETA_BISECTION_ITERATIONS):
        eta_midpoint = 0.5 * (eta_lower + eta_upper)
        sigmoid_sum = sum(
            _stable_sigmoid((cost - eta_midpoint) / temperature)
            for cost in costs
        )
        if sigmoid_sum > target_tail_mass:
            eta_lower = eta_midpoint
        else:
            eta_upper = eta_midpoint
    eta = 0.5 * (eta_lower + eta_upper)
    tail_scale = 1.0 / target_tail_mass
    binary_entropy = (
        -tail_probability * math.log(tail_probability)
        - alpha * math.log(alpha)
    )
    result = eta - temperature * binary_entropy / tail_probability
    for cost in costs:
        standardized = (cost - eta) / temperature
        result += tail_scale * temperature * _stable_softplus(standardized)
    return result


def _usable(row: dict[str, Any]) -> bool:
    if "matched" in row:
        matched = _truthy(row.get("matched"))
    else:
        matched = (
            _truthy(row.get("hit", 1))
            and _truthy(row.get("valid", 1))
            and not _truthy(row.get("stale", 0))
        )
    return matched and _finite(row.get("c_pi")) is not None


def _profile_points(rows: Iterable[dict[str, Any]]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for row in rows:
        values = {key: _finite(row.get(key)) for key in ("x", "y", "z", "t_s", "c_pi")}
        if any(value is None for value in values.values()) or not _usable(row):
            raise ValueError("profile requires complete matched support and finite x/y/z/t_s/c_pi")
        points.append({key: float(value) for key, value in values.items() if value is not None})
    if len(points) < 2:
        raise ValueError("profile requires at least two samples")
    cumulative = 0.0
    points[0]["distance_m"] = 0.0
    for previous, current in zip(points, points[1:]):
        gap = math.dist(
            (previous["x"], previous["y"], previous["z"]),
            (current["x"], current["y"], current["z"]),
        )
        cumulative += gap
        current["distance_m"] = cumulative
    if not math.isfinite(cumulative) or cumulative <= 0.0:
        raise ValueError("profile must have positive arc length")
    return points


def _interpolate(points: list[dict[str, float]], target_m: float) -> dict[str, float]:
    distances = [point["distance_m"] for point in points]
    if target_m <= distances[0]:
        return dict(points[0])
    if target_m >= distances[-1]:
        return dict(points[-1])
    right = bisect_right(distances, target_m)
    left = right - 1
    while right < len(points) and distances[right] <= distances[left]:
        right += 1
    if right >= len(points):
        return dict(points[left])
    width = distances[right] - distances[left]
    fraction = 0.0 if width <= 0.0 else (target_m - distances[left]) / width
    result = {
        key: points[left][key] + fraction * (points[right][key] - points[left][key])
        for key in ("x", "y", "z", "t_s", "c_pi")
    }
    result["distance_m"] = target_m
    return result


def _fixed_terminal_lattice(
    points: list[dict[str, float]],
    terminal_arc_length_m: float,
    *,
    count: int,
) -> list[dict[str, Any]]:
    if count < 2:
        raise ValueError("fixed lattice requires at least two samples")
    total = points[-1]["distance_m"]
    start = total - terminal_arc_length_m
    lattice = []
    for index in range(count):
        fraction = index / (count - 1)
        sample = _interpolate(points, start + terminal_arc_length_m * fraction)
        lattice.append({
            "sample_index": index,
            "arc_fraction": fraction,
            "arc_distance_m": terminal_arc_length_m * fraction,
            "derived": True,
            **{key: sample[key] for key in ("x", "y", "z", "t_s", "c_pi")},
        })
    return lattice


def _resampling_residual(
    points: list[dict[str, float]],
    lattice: list[dict[str, Any]],
    terminal_arc_length_m: float,
) -> float:
    start = points[-1]["distance_m"] - terminal_arc_length_m
    lattice_points = [
        {
            "distance_m": float(row["arc_distance_m"]),
            **{key: float(row[key]) for key in ("x", "y", "z", "t_s", "c_pi")},
        }
        for row in lattice
    ]
    residual = 0.0
    for point in points:
        if point["distance_m"] + 1.0e-12 < start:
            continue
        reconstructed = _interpolate(lattice_points, point["distance_m"] - start)
        residual = max(residual, abs(reconstructed["c_pi"] - point["c_pi"]))
    return residual


def compare_profiles(
    current_rows: Iterable[dict[str, Any]],
    reference_rows: Iterable[dict[str, Any]],
    *,
    count: int = FIXED_SAMPLE_COUNT,
    alpha: float = SMOOTH_CVAR_ALPHA,
    temperature: float = SMOOTH_CVAR_TEMPERATURE,
) -> dict[str, Any]:
    """Compare two authoritative fixed-200 profiles at one decision checkpoint."""
    current_points = _profile_points(current_rows)
    reference_points = _profile_points(reference_rows)
    if len(current_points) != count or len(reference_points) != count:
        raise ValueError(f"decision-checkpoint profiles require {count}/{count} support")
    def as_lattice(points: list[dict[str, float]]) -> list[dict[str, Any]]:
        return [
            {
                "sample_index": index,
                "arc_fraction": index / (count - 1),
                "arc_distance_m": point["distance_m"],
                "derived": False,
                **{key: point[key] for key in ("x", "y", "z", "t_s", "c_pi")},
            }
            for index, point in enumerate(points)
        ]
    current_lattice = as_lattice(current_points)
    reference_lattice = as_lattice(reference_points)
    current_values = [float(row["c_pi"]) for row in current_lattice]
    reference_values = [float(row["c_pi"]) for row in reference_lattice]
    return {
        "comparison_mode": "first_deterministic_decision_checkpoint_fixed_200",
        "derived": False,
        "sample_count": count,
        "common_terminal_arc_length_m": None,
        "current_lattice": current_lattice,
        "reference_lattice": reference_lattice,
        "current_mean": sum(current_values) / count,
        "reference_mean": sum(reference_values) / count,
        "current_cvar": smooth_cvar(current_values, alpha=alpha, temperature=temperature),
        "reference_cvar": smooth_cvar(reference_values, alpha=alpha, temperature=temperature),
        "current_max": max(current_values),
        "reference_max": max(reference_values),
        "current_resampling_residual": 0.0,
        "reference_resampling_residual": 0.0,
        "epsilon_resample": 0.0,
    }


def sampling_sufficiency(
    rows: Iterable[dict[str, Any]],
    *,
    resolution_m: float,
    horizons_s: Iterable[float],
) -> dict[str, Any]:
    points = _profile_points(rows)
    spatial_gaps = [
        current["distance_m"] - previous["distance_m"]
        for previous, current in zip(points, points[1:])
    ]
    temporal_gaps = [
        abs(current["t_s"] - previous["t_s"])
        for previous, current in zip(points, points[1:])
    ]
    horizons = sorted(set(float(value) for value in horizons_s))
    horizon_gaps = [right - left for left, right in zip(horizons, horizons[1:]) if right > left]
    spatial_limit = float(resolution_m) / 2.0
    temporal_limit = min(horizon_gaps) / 2.0 if horizon_gaps else None
    max_spatial = max(spatial_gaps, default=math.inf)
    max_temporal = max(temporal_gaps, default=math.inf)
    passed = (
        math.isfinite(spatial_limit)
        and spatial_limit > 0.0
        and temporal_limit is not None
        and max_spatial <= spatial_limit + 1.0e-12
        and max_temporal <= temporal_limit + 1.0e-12
    )
    return {
        "passed": passed,
        "max_spatial_gap_m": max_spatial,
        "spatial_gap_limit_m": spatial_limit,
        "max_temporal_gap_s": max_temporal,
        "temporal_gap_limit_s": temporal_limit,
    }


def corner_reconstruction_residual(
    profile_rows: Iterable[dict[str, Any]],
    corner_rows: Iterable[dict[str, Any]],
) -> dict[str, Any]:
    """Reconstruct accepted query c_pi from temporal and spatial corner weights."""
    profiles = list(profile_rows)
    corners = [row for row in corner_rows if str(row.get("phase", "")) == "accepted"]
    grouped: dict[int, list[dict[str, Any]]] = {}
    for row in corners:
        index = _finite(row.get("sample_index"))
        if index is not None and index.is_integer():
            grouped.setdefault(int(index), []).append(row)
    residuals: list[float] = []
    missing: list[int] = []
    for row in profiles:
        index_value = _finite(row.get("sample_index"))
        if index_value is None or not index_value.is_integer() or not _usable(row):
            continue
        index = int(index_value)
        source = grouped.get(index, [])
        terms = []
        for corner in source:
            temporal = _finite(corner.get("temporal_weight"))
            spatial = _finite(corner.get("corner_weight"))
            value = _finite(corner.get("c_pi"))
            if temporal is not None and spatial is not None and value is not None:
                terms.append((temporal * spatial, value))
        total_weight = sum(weight for weight, _ in terms)
        if not terms or abs(total_weight - 1.0) > 1.0e-9:
            missing.append(index)
            continue
        reconstructed = sum(weight * value for weight, value in terms)
        residuals.append(abs(reconstructed - float(row["c_pi"])))
    complete = len(residuals) == len(profiles) and not missing
    return {
        "complete": complete,
        "sample_count": len(profiles),
        "reconstructed_sample_count": len(residuals),
        "missing_sample_indices": missing,
        "epsilon_grid": max(residuals, default=math.inf),
    }


def evaluate_effectiveness(
    *,
    current_mean: float,
    reference_mean: float,
    current_cvar: float,
    reference_cvar: float,
    current_max: float,
    reference_max: float,
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    tau_mean = float(thresholds["tau_mean"])
    tau_cvar = float(thresholds["tau_cvar"])
    tau_max = float(thresholds["tau_max"])
    mean_improvement = reference_mean - current_mean
    cvar_improvement = reference_cvar - current_cvar
    max_regression = current_max - reference_max
    mean_margin = mean_improvement - tau_mean
    cvar_margin = cvar_improvement - tau_cvar
    max_margin = tau_max - max_regression
    return {
        "mean_improvement": mean_improvement,
        "cvar_improvement": cvar_improvement,
        "max_regression": max_regression,
        "tau_mean": tau_mean,
        "tau_cvar": tau_cvar,
        "tau_max": tau_max,
        "mean_remaining_margin": mean_margin,
        "cvar_remaining_margin": cvar_margin,
        "max_remaining_margin": max_margin,
        "mean_significant": mean_improvement > tau_mean,
        "cvar_significant": cvar_improvement > tau_cvar,
        "max_regression_bounded": max_regression <= tau_max,
        "passed": (
            mean_improvement > tau_mean
            and cvar_improvement > tau_cvar
            and max_regression <= tau_max
        ),
    }


def _same_number(left: Any, right: Any, tolerance: float = 1.0e-12) -> bool:
    left_value = _finite(left)
    right_value = _finite(right)
    return (
        left_value is not None
        and right_value is not None
        and abs(left_value - right_value) <= tolerance
    )


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_calibration_binding(
    calibration_path: str | Path,
    manifest: dict[str, Any],
) -> dict[str, Any]:
    """Validate one formal run against its immutable pre-run calibration."""
    errors: list[str] = []
    path = Path(calibration_path).expanduser().resolve()
    if not path.is_file():
        return {"passed": False, "errors": [f"P1 calibration file is missing: {path}"]}
    try:
        calibration = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return {"passed": False, "errors": [f"P1 calibration JSON is unreadable: {exc}"]}
    if not isinstance(calibration, dict):
        return {"passed": False, "errors": ["P1 calibration JSON root is not an object"]}
    digest = _sha256_file(path)
    binding = manifest.get("p1.formal_calibration", {}) or {}
    if calibration.get("schema_version") != "p1_formal_tolerance_calibration_v1":
        errors.append("P1 calibration schema is not p1_formal_tolerance_calibration_v1")
    if binding.get("calibration_id") != calibration.get("calibration_id"):
        errors.append("P1 calibration ID does not match manifest binding")
    if Path(str(binding.get("path", ""))).expanduser().resolve() != path:
        errors.append("P1 calibration path does not match manifest binding")
    if binding.get("sha256") != digest:
        errors.append("P1 calibration SHA256 does not match manifest binding")
    conformal = calibration.get("conformal", {}) or {}
    if conformal.get("pair_count") != 10 or not _same_number(conformal.get("coverage"), 0.90):
        errors.append("P1 calibration does not contain the frozen 10-pair/90% contract")
    run_ids = calibration.get("run_ids", [])
    if not isinstance(run_ids, list) or len(run_ids) != 20 or len(set(run_ids)) != 20:
        errors.append("P1 calibration does not contain 20 unique run IDs")
    thresholds = calibration.get("thresholds", {}) or {}
    for key in ("tau_mean", "tau_cvar", "tau_max"):
        value = _finite(thresholds.get(key))
        if value is None or value < 0.0:
            errors.append(f"P1 calibration threshold is invalid: {key}")
    deterministic = calibration.get("deterministic_error", {}) or {}
    epsilon_grid = _finite(deterministic.get("epsilon_grid"))
    epsilon_resample = _finite(deterministic.get("epsilon_resample"))
    epsilon_det = _finite(deterministic.get("epsilon_det"))
    if (
        epsilon_grid is None or epsilon_grid < 0.0
        or epsilon_resample is None or epsilon_resample < 0.0
        or epsilon_det is None
        or abs(epsilon_det - 2.0 * (epsilon_grid + epsilon_resample)) > 1.0e-12
    ):
        errors.append("P1 calibration deterministic error decomposition is invalid")
    pairs = calibration.get("pairs")
    null_maxima = calibration.get("null_effect_maxima", {}) or {}
    if not isinstance(pairs, list) or len(pairs) != 10:
        errors.append("P1 calibration does not contain exactly 10 pair score records")
    else:
        pair_ids: set[str] = set()
        pair_run_ids: list[str] = []
        recomputed_maxima = {key: 0.0 for key in ("s_mean", "s_cvar", "s_max")}
        metric_fields = (
            ("mean", "s_mean"), ("cvar", "s_cvar"), ("max", "s_max"),
        )
        for pair in pairs:
            pair_id = str(pair.get("pair_id", "")) if isinstance(pair, dict) else ""
            if not pair_id or pair_id in pair_ids or pair.get("valid") is not True:
                errors.append("P1 calibration pair IDs/validity are inconsistent")
                break
            pair_ids.add(pair_id)
            pair_run_ids.extend((str(pair.get("run_a_id", "")), str(pair.get("run_b_id", ""))))
            for metric, score_key in metric_fields:
                run_a = _finite(pair.get(f"run_a_{metric}"))
                run_b = _finite(pair.get(f"run_b_{metric}"))
                score = _finite(pair.get(score_key))
                if (
                    run_a is None or run_b is None or score is None or score < 0.0
                    or abs(score - abs(run_a - run_b)) > 1.0e-12
                ):
                    errors.append(f"P1 calibration pair score is invalid: {pair_id}/{score_key}")
                    continue
                recomputed_maxima[score_key] = max(recomputed_maxima[score_key], score)
        if set(pair_run_ids) != set(run_ids if isinstance(run_ids, list) else []):
            errors.append("P1 calibration pair run IDs do not match frozen run IDs")
        for score_key, maximum in recomputed_maxima.items():
            if not _same_number(null_maxima.get(score_key), maximum):
                errors.append(f"P1 calibration null-effect maximum is invalid: {score_key}")
        threshold_sources = (
            ("tau_mean", "s_mean"), ("tau_cvar", "s_cvar"), ("tau_max", "s_max"),
        )
        if epsilon_det is not None:
            for threshold_key, score_key in threshold_sources:
                expected_threshold = recomputed_maxima[score_key] + epsilon_det
                if not _same_number(thresholds.get(threshold_key), expected_threshold):
                    errors.append(f"P1 calibration frozen threshold formula is invalid: {threshold_key}")

    provenance = manifest.get("artifact_provenance", {}) or {}
    generated = _finite(calibration.get("generated_at_epoch_s"))
    started = _finite(provenance.get("process_start_epoch_s"))
    if generated is None or started is None or not generated < started:
        errors.append("P1 calibration was not generated before formal run start")
    if provenance.get("run_id") in set(run_ids if isinstance(run_ids, list) else []):
        errors.append("formal run ID is reused from the calibration sample")
    for key in ("git_commit", "baseline_commit"):
        if provenance.get(key) != calibration.get(key):
            errors.append(f"P1 calibration {key} does not match formal run")
    if manifest.get("scenario") != calibration.get("scenario"):
        errors.append("P1 calibration scenario does not match formal run")
    fingerprint = str(manifest.get("scenario_fingerprint", "")).strip()
    if not fingerprint or fingerprint != calibration.get("scenario_fingerprint"):
        errors.append("P1 calibration scenario fingerprint does not match formal run")
    if manifest.get("scenario_contract") != calibration.get("scenario_contract"):
        errors.append("P1 calibration expanded scenario contract does not match formal run")
    p0 = calibration.get("p0", {}) or {}
    if not _same_number(manifest.get("p0.resolution_m"), p0.get("resolution_m")):
        errors.append("P1 calibration P0 resolution does not match formal run")
    if manifest.get("p0.horizons_s") != p0.get("horizons_s"):
        errors.append("P1 calibration P0 horizons do not match formal run")
    smooth = calibration.get("smooth_cvar", {}) or {}
    expected_values = (
        ("p1.lambda_integrity", calibration.get("lambda_integrity")),
        ("p1.smooth_cvar_alpha", smooth.get("alpha")),
        ("p1.smooth_max_temperature", smooth.get("temperature")),
        ("p1.normalization_budget_fraction", calibration.get("normalization_budget_fraction")),
        ("grid_map/local_update_range_x", 10.0),
    )
    for key, expected in expected_values:
        if not _same_number(manifest.get(key), expected):
            errors.append(f"P1 calibration {key} does not match formal run")
    if manifest.get("p1.objective_aggregation_mode") != smooth.get("mode"):
        errors.append("P1 calibration aggregation mode does not match formal run")
    if smooth.get("eta_bisection_iterations") != ETA_BISECTION_ITERATIONS:
        errors.append("P1 calibration eta bisection contract does not match production")
    calibration_config = calibration.get("configuration_identity")
    if not isinstance(calibration_config, dict):
        errors.append("P1 calibration configuration identity is missing")
    else:
        # metrics_only and record_bag intentionally differ: they describe the
        # comparison arm/evidence capture, not planner/P0 identity. Everything
        # below must stay identical to calibration.
        exact_config_keys = (
            "experiment",
            "planner_safety_profile",
            "p1.use_integrity_cost",
            "p1.max_candidates_per_attempt",
            "p1.objective_aggregation_mode",
            "scenario_fingerprint",
            "scenario_contract",
            "run_validator",
            *P0_CONFIGURATION_KEYS,
        )
        numeric_config_keys = (
            "p1.lambda_integrity",
            "p1.smooth_cvar_alpha",
            "p1.smooth_max_temperature",
            "run_duration_s",
            "validation_duration_s",
            "planner_start_delay_s",
            "manager/max_vel",
            "optimization/max_vel",
            "bspline/limit_vel",
            "fsm.thresh_replan_time",
            "p1.normalization_budget_fraction",
        )
        for key in exact_config_keys:
            if manifest.get(key) != calibration_config.get(key):
                errors.append(f"P1 calibration configuration does not match formal run: {key}")
        for key in numeric_config_keys:
            if not _same_number(manifest.get(key), calibration_config.get(key)):
                errors.append(f"P1 calibration configuration does not match formal run: {key}")
    runtime = provenance.get("runtime_paths", {}) or {}
    calibration_runtime = calibration.get("runtime_hashes", {}) or {}
    for key in ("launch", "planner_executable", "bspline_library"):
        if str((runtime.get(key, {}) or {}).get("sha256", "")) != str(calibration_runtime.get(key, "")):
            errors.append(f"P1 calibration runtime hash does not match formal run: {key}")
    return {
        "passed": not errors,
        "errors": errors,
        "path": str(path),
        "sha256": digest,
        "calibration_id": calibration.get("calibration_id"),
        "calibration": calibration,
    }
