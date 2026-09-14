#!/usr/bin/env python3
"""Reproduce and decompose P4 GNSS risk evidence without changing policy."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

import numpy as np


def _finite(row: dict, name: str) -> float:
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(name)
    return value


def _q_inv(probability: float) -> float:
    if probability <= 0.0:
        return 1.0e9
    if probability >= 0.5:
        return 0.0
    value = math.sqrt(-2.0 * math.log(probability))
    numerator = 2.515517 + .802853 * value + .010328 * value * value
    denominator = (1.0 + 1.432788 * value + .189269 * value * value
                   + .001308 * value * value * value)
    return value - numerator / denominator


def _canopy_sigma(elevation: float, kappa: float) -> float:
    sin_el = max(math.sin(elevation), .052)
    bounded_kappa = min(1.0, max(0.0, kappa))
    return math.sqrt(
        1.0 + .25 / (sin_el * sin_el)
        + 25.0 * math.exp(2.0 * bounded_kappa / sin_el))


def _geometry_pl(satellites: list[dict], sigmas: list[float]) -> dict:
    count = len(satellites)
    if count < 4 or count != len(sigmas):
        raise ValueError("insufficient_satellites")
    design = np.asarray([[
        math.cos(_finite(row, "elevation_rad")) *
        math.sin(_finite(row, "azimuth_rad")),
        math.cos(_finite(row, "elevation_rad")) *
        math.cos(_finite(row, "azimuth_rad")),
        math.sin(_finite(row, "elevation_rad")), 1.0,
    ] for row in satellites], dtype=float)
    weights = np.asarray([1.0 / max(float(sigma), .01) ** 2
                          for sigma in sigmas])
    normal = design.T @ np.diag(weights) @ design
    covariance = np.linalg.inv(normal)
    k_ff = max(1.0, _q_inv((1.0e-7 / 2.0) / 2.0))
    k_fa = max(1.0, _q_inv((1.0e-5 / count) / 2.0))
    k_md = max(1.0, _q_inv(1.0e-7 / (2.0 * count)))
    best = [k_ff * math.sqrt(max(0.0, covariance[index, index]))
            for index in range(3)]
    worst = [-1, -1, -1]
    for excluded in range(count):
        subset = [index for index in range(count) if index != excluded]
        subset_design = design[subset, :]
        subset_weights = np.diag(weights[subset])
        subset_covariance = np.linalg.inv(
            subset_design.T @ subset_weights @ subset_design)
        for axis in range(3):
            # This intentionally mirrors the production comparison direction.
            sigma_ss = math.sqrt(max(
                0.0, covariance[axis, axis] -
                subset_covariance[axis, axis]))
            sigma_k = math.sqrt(max(0.0, subset_covariance[axis, axis]))
            protection = k_fa * sigma_ss + k_md * sigma_k
            if protection > best[axis]:
                best[axis] = protection
                worst[axis] = int(satellites[excluded]["sat_id"])
    return {
        "hpl": max(best[0], best[1]), "vpl": best[2],
        "pl_e": best[0], "pl_n": best[1], "pl_u": best[2],
        "worst_h": worst[0] if best[0] >= best[1] else worst[1],
        "worst_v": worst[2],
        "condition": float(np.linalg.cond(normal)),
        "satellite_count": count,
    }


def _sample_key(row: dict) -> tuple:
    return tuple(row.get(name, "") for name in (
        "decision_event_id", "planning_attempt_id", "candidate_id",
        "sample_index"))


def _percentiles(values: list[float]) -> dict:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return {"min": None, "median": None, "p95": None, "max": None}
    p95_index = min(len(finite) - 1, math.ceil(.95 * len(finite)) - 1)
    return {"min": finite[0], "median": statistics.median(finite),
            "p95": finite[p95_index], "max": finite[-1]}


def analyze_detail_rows(rows: list[dict], replay_tolerance_m: float = 1e-8) -> dict:
    failures = []
    deduplicated = {}
    for row in rows:
        key = (*_sample_key(row), row.get("sat_id", ""))
        deduplicated.setdefault(key, row)
    groups = {}
    for row in deduplicated.values():
        groups.setdefault(_sample_key(row), []).append(row)
    samples = []
    required = {"sat_id", "used", "elevation_rad", "azimuth_rad",
                "kappa", "sigma_eff_m", "candidate_raw_hpl",
                "candidate_raw_vpl"}
    for key, group in groups.items():
        used = [row for row in group if str(row.get("used", "0")) == "1"]
        if not used or any(not required.issubset(row) for row in used):
            failures.append("required_satellite_fields_missing")
            continue
        try:
            used.sort(key=lambda row: int(row["sat_id"]))
            actual_sigmas = [_finite(row, "sigma_eff_m") for row in used]
            actual = _geometry_pl(used, actual_sigmas)
            production_h = _finite(group[0], "candidate_raw_hpl")
            production_v = _finite(group[0], "candidate_raw_vpl")
        except (KeyError, TypeError, ValueError, np.linalg.LinAlgError):
            failures.append("production_replay_input_invalid")
            continue
        replay_error = max(abs(actual["hpl"] - production_h),
                           abs(actual["vpl"] - production_v))
        if replay_error > replay_tolerance_m:
            failures.append("production_raw_pl_replay_mismatch")
        unit = _geometry_pl(used, [1.0] * len(used))

        open_sigmas = []
        canopy_identifiable = True
        sigma_sources = []
        for row, actual_sigma in zip(used, actual_sigmas):
            elevation = _finite(row, "elevation_rad")
            kappa = _finite(row, "kappa")
            canopy = _canopy_sigma(elevation, kappa)
            if "epoch_pr_sigma_m" in row and row["epoch_pr_sigma_m"] != "":
                epoch_sigma = _finite(row, "epoch_pr_sigma_m")
                open_sigmas.append(max(epoch_sigma, _canopy_sigma(elevation, 0)))
                sigma_sources.append(
                    "epoch" if epoch_sigma >= canopy else "canopy")
            elif math.isclose(actual_sigma, canopy, rel_tol=1e-9,
                              abs_tol=1e-9):
                open_sigmas.append(_canopy_sigma(elevation, 0))
                sigma_sources.append("canopy_inferred")
            else:
                canopy_identifiable = False
                sigma_sources.append("unidentified")
        open_canopy = (_geometry_pl(used, open_sigmas)
                       if canopy_identifiable else None)
        sample = {
            "key": list(key),
            "position_m": [_finite(group[0], name) for name in ("x", "y", "z")]
            if all(name in group[0] for name in ("x", "y", "z")) else None,
            "arc_length_m": float(group[0].get("arc_length_m", "nan")),
            "query_time_s": float(group[0].get("query_time_s", "nan")),
            "risk_generation": int(group[0].get("risk_generation", 0) or 0),
            "occupancy_generation": int(
                group[0].get("occupancy_generation", 0) or 0),
            "gnss_epoch_identity": str(group[0].get("gnss_epoch_identity", "")),
            "satellite_set_hash": str(group[0].get("satellite_set_hash", "")),
            "satellite_ids": [int(row["sat_id"]) for row in used],
            "satellite_count": len(used),
            "sigma_sources": sigma_sources,
            "sigma_eff_m": actual_sigmas,
            "kappa": [_finite(row, "kappa") for row in used],
            "production_raw_hpl_m": production_h,
            "production_raw_vpl_m": production_v,
            "replayed_raw_hpl_m": actual["hpl"],
            "replayed_raw_vpl_m": actual["vpl"],
            "replay_error_m": replay_error,
            "weighted_geometry_condition": actual["condition"],
            "unit_sigma_geometry_condition": unit["condition"],
            "worst_excluded_sat_h": actual["worst_h"],
            "worst_excluded_sat_v": actual["worst_v"],
            "unit_sigma_hpl_m": unit["hpl"],
            "unit_sigma_vpl_m": unit["vpl"],
            "kappa_zero_identifiable": canopy_identifiable,
            "kappa_zero_hpl_m": open_canopy["hpl"] if open_canopy else None,
            "kappa_zero_vpl_m": open_canopy["vpl"] if open_canopy else None,
        }
        for source, target in (
                ("receiver_raw_hpl", "receiver_raw_hpl_m"),
                ("receiver_raw_vpl", "receiver_raw_vpl_m"),
                ("anchor_hpl", "certified_anchor_hpl_m"),
                ("anchor_vpl", "certified_anchor_vpl_m"),
                ("spatial_delta_h", "spatial_delta_h_m"),
                ("spatial_delta_v", "spatial_delta_v_m"),
                ("temporal_growth_h", "temporal_growth_h_m"),
                ("temporal_growth_v", "temporal_growth_v_m"),
                ("final_hpl", "final_hpl_m"),
                ("final_vpl", "final_vpl_m")):
            sample[target] = float(group[0].get(source, "nan"))
        samples.append(sample)

    samples.sort(key=lambda item: (
        item["risk_generation"], item["arc_length_m"], item["key"]))
    jumps = []
    same_set_jumps = []
    changed_set_jumps = []
    set_changes = 0
    satellite_additions = {}
    satellite_removals = {}
    routes = {}
    for sample in samples:
        routes.setdefault(tuple(sample["key"][:3]), []).append(sample)
    for route in routes.values():
        route.sort(key=lambda item: (item["arc_length_m"], item["key"]))
        for previous, current in zip(route, route[1:]):
            jump = abs(current["replayed_raw_hpl_m"] -
                       previous["replayed_raw_hpl_m"])
            jumps.append(jump)
            previous_ids = set(previous["satellite_ids"])
            current_ids = set(current["satellite_ids"])
            if current_ids == previous_ids:
                same_set_jumps.append(jump)
            else:
                set_changes += 1
                changed_set_jumps.append(jump)
                for sat_id in current_ids - previous_ids:
                    satellite_additions[str(sat_id)] = (
                        satellite_additions.get(str(sat_id), 0) + 1)
                for sat_id in previous_ids - current_ids:
                    satellite_removals[str(sat_id)] = (
                        satellite_removals.get(str(sat_id), 0) + 1)
    return {
        "schema_version": "p4_gnss_sensitivity_report_v1",
        "attribution_valid": not failures,
        "failures": list(dict.fromkeys(failures)),
        "input_row_count": len(rows),
        "deduplicated_row_count": len(deduplicated),
        "sample_count": len(samples),
        "samples": samples,
        "weighted_geometry_condition": _percentiles([
            sample["weighted_geometry_condition"] for sample in samples]),
        "raw_hpl_m": _percentiles([
            sample["replayed_raw_hpl_m"] for sample in samples]),
        "raw_vpl_m": _percentiles([
            sample["replayed_raw_vpl_m"] for sample in samples]),
        "satellite_count": _percentiles([
            float(sample["satellite_count"]) for sample in samples]),
        "noise_scale_contribution_h_m": _percentiles([
            sample["replayed_raw_hpl_m"] - sample["unit_sigma_hpl_m"]
            for sample in samples]),
        "noise_scale_contribution_v_m": _percentiles([
            sample["replayed_raw_vpl_m"] - sample["unit_sigma_vpl_m"]
            for sample in samples]),
        "canopy_contribution_h_m": _percentiles([
            sample["replayed_raw_hpl_m"] - sample["kappa_zero_hpl_m"]
            for sample in samples
            if sample["kappa_zero_hpl_m"] is not None]),
        "canopy_contribution_v_m": _percentiles([
            sample["replayed_raw_vpl_m"] - sample["kappa_zero_vpl_m"]
            for sample in samples
            if sample["kappa_zero_vpl_m"] is not None]),
        "spatial_delta_h_m": _percentiles([
            sample["spatial_delta_h_m"] for sample in samples]),
        "spatial_delta_v_m": _percentiles([
            sample["spatial_delta_v_m"] for sample in samples]),
        "spatial_discrimination": {
            "within_route_raw_hpl_jump_m": _percentiles(jumps),
            "within_route_raw_hpl_jump_max_m": max(jumps, default=0.0),
            "satellite_set_change_count": int(set_changes),
            "same_satellite_set_hpl_jump_m": _percentiles(same_set_jumps),
            "changed_satellite_set_hpl_jump_m": _percentiles(
                changed_set_jumps),
            "satellite_addition_counts": satellite_additions,
            "satellite_removal_counts": satellite_removals,
        },
    }


def _read_csv(path: Path | None) -> list[dict]:
    if path is None or not path.is_file():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _probe_summary(rows: list[dict]) -> dict:
    classes = {}
    map_shifts = []
    gnss_shifts = []
    combined_shifts = []
    for row in rows:
        name = row.get("classification", "UNKNOWN")
        classes[name] = classes.get(name, 0) + 1
        try:
            old = _finite(row, "old_map_old_epoch_first_arc_m")
            new_map = _finite(row, "new_map_old_epoch_first_arc_m")
            new_epoch = _finite(row, "old_map_new_epoch_first_arc_m")
            combined = _finite(row, "new_map_new_epoch_first_arc_m")
        except (KeyError, TypeError, ValueError):
            continue
        map_shifts.append(abs(new_map - old))
        gnss_shifts.append(abs(new_epoch - old))
        combined_shifts.append(abs(combined - old))
    return {"row_count": len(rows), "classification_counts": classes,
            "fixed_epoch_map_first_unsafe_shift_m": _percentiles(map_shifts),
            "fixed_map_gnss_first_unsafe_shift_m": _percentiles(gnss_shifts),
            "combined_first_unsafe_shift_m": _percentiles(combined_shifts)}


def _forward_discrimination(rows: list[dict]) -> dict:
    unique = {}
    for row in rows:
        key = tuple(row.get(name, "") for name in (
            "decision_event_id", "planning_attempt_id", "candidate_id",
            "sample_index"))
        unique.setdefault(key, row)
    ratios = []
    routes = {}
    for row in unique.values():
        try:
            ratio = _finite(row, "safety_ratio")
        except (KeyError, TypeError, ValueError):
            continue
        ratios.append(ratio)
        key = tuple(row.get(name, "") for name in (
            "decision_event_id", "planning_attempt_id", "candidate_id"))
        routes.setdefault(key, []).append(ratio)
    spans = [max(values) - min(values) for values in routes.values() if values]
    return {"sample_count": len(ratios), "safety_ratio": _percentiles(ratios),
            "unique_safety_ratio_count": len(set(ratios)),
            "route_internal_span": _percentiles(spans)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path)
    parser.add_argument("--detail-csv", type=Path)
    parser.add_argument("--forward-risk-csv", type=Path)
    parser.add_argument("--generation-probe-csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.run_root:
        base = args.run_root / "exports/planner_p4_risk_astar_debug.csv"
        args.detail_csv = args.detail_csv or Path(str(base) + ".gnss_risk_detail.csv")
        args.forward_risk_csv = args.forward_risk_csv or Path(
            str(base) + ".forward_risk_samples.csv")
        args.generation_probe_csv = args.generation_probe_csv or Path(
            str(base) + ".generation_probe.csv")
    report = analyze_detail_rows(_read_csv(args.detail_csv))
    report["forward_risk_discrimination"] = _forward_discrimination(
        _read_csv(args.forward_risk_csv))
    report["generation_probe"] = _probe_summary(
        _read_csv(args.generation_probe_csv))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 0 if report["attribution_valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
