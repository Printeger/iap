#!/usr/bin/env python3
"""Fail-closed P1-2 prequalification analysis; never invokes the formal analyzer."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


formal_metrics = _load_module("p1_prequalification_formal_metrics", HERE / "p1_formal_metrics.py")
calibration = _load_module(
    "p1_prequalification_calibration", HERE / "calibrate_p1_formal_tolerances.py"
)

TAU_MEAN = 0.005574670273862936
TAU_CVAR = 0.004511997578310001
PRIMARY_MEAN_IMPROVEMENT = 0.00836
PRIMARY_CVAR_IMPROVEMENT = 0.00677


class PrequalificationError(ValueError):
    pass


def _read_csv(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _artifact(export: Path, manifest: dict[str, Any], key: str, filename: str) -> Path:
    raw = Path(str(manifest.get(key, filename))).expanduser()
    path = raw.resolve() if raw.is_absolute() else (export / raw).resolve()
    if path.parent != export.resolve() or path.name != filename:
        raise PrequalificationError(f"invalid artifact binding for {key}: {path}")
    return path


def _candidate_evidence_paths(
    export: Path, manifest: dict[str, Any]
) -> tuple[Path | None, Path]:
    return None, _artifact(
        export, manifest, "p1.prequalification_candidate_profile_path",
        "planner_p1_prequalification_candidate_profile.csv",
    )


def _identity(row: dict[str, Any]) -> tuple[str, ...]:
    return tuple(str(row.get(key, "")) for key in (
        "planning_attempt_id", "candidate_id", "snapshot_generation_id", "query_base_time_s"
    ))


def _same_attempt_context(left: dict[str, Any], right: dict[str, Any]) -> bool:
    return all(str(left.get(key, "")) == str(right.get(key, "")) for key in (
        "planning_attempt_id", "snapshot_generation_id", "query_base_time_s"
    ))


def _prequalification_candidate_rows(
    candidates: list[dict[str, Any]], context: dict[str, Any]
) -> list[dict[str, Any]]:
    return [
        {
            **row,
            "matched": _truthy(row.get("valid")) and not _truthy(row.get("stale")),
            "collision_free": _truthy(row.get("collision_free")),
            "selected": _truthy(row.get("selected")),
        }
        for row in candidates
        if _same_attempt_context(row, context) and row.get("phase") == "admitted"
    ]


def _validate_provenance_rows(
    rows: list[dict[str, Any]], run_id: str, manifest_path: Path, label: str
) -> None:
    if not rows:
        raise PrequalificationError(f"{label} provenance is empty")
    expected_manifest = manifest_path.resolve()
    for row in rows:
        if (
            row.get("schema_version") != "p1_evidence_provenance_v4"
            or row.get("run_id") != run_id
            or Path(str(row.get("manifest_path", ""))).resolve() != expected_manifest
        ):
            raise PrequalificationError(f"{label} provenance binding failed")


def _scan_occupancy_evidence(
    path: Path, run_id: str, manifest_path: Path, context: dict[str, Any]
) -> tuple[list[dict[str, Any]], dict[tuple[str, ...], list[dict[str, Any]]]]:
    """Validate the complete stream while retaining only one attempt's join rows."""
    accepted: list[dict[str, Any]] = []
    final: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    row_count = 0
    expected_manifest = manifest_path.resolve()
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            row_count += 1
            if (
                row.get("schema_version") != "p1_evidence_provenance_v4"
                or row.get("run_id") != run_id
                or Path(str(row.get("manifest_path", ""))).resolve()
                != expected_manifest
            ):
                raise PrequalificationError("occupancy provenance binding failed")
            if not _same_attempt_context(row, context):
                continue
            if row.get("phase") == "accepted" and _identity(row) == _identity(context):
                accepted.append(row)
            elif row.get("phase") == "final":
                key = (*_identity(row), str(row.get("sample_index", "")))
                final.setdefault(key, []).append(row)
    if row_count == 0:
        raise PrequalificationError("occupancy provenance is empty")
    return accepted, final


def analyze_run(export_value: str | Path) -> dict[str, Any]:
    export = Path(export_value).expanduser().resolve()
    manifest_path = export / "test_planner_manifest.json"
    errors: list[str] = []
    occupancy_final: dict[tuple[str, ...], list[dict[str, Any]]] = {}
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise PrequalificationError(f"manifest unreadable: {exc}") from exc
    provenance = manifest.get("artifact_provenance", {}) or {}
    run_id = str(provenance.get("run_id", ""))
    expected = {
        "experiment": "p1_fork_formal", "planner_safety_profile": "p1",
        "run_validator": True, "record_bag": False,
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            errors.append(f"contract mismatch: {key}")
    for key, value in (("p1.lambda_integrity", 1e-5),
                       ("p1.normalization_budget_fraction", 0.30),
                       ("run_duration_s", 90.0), ("validation_duration_s", 90.0),
                       ("planner_start_delay_s", 10.0),
                       ("p0.size_y_m", 12.0),
                       ("manager/max_vel", 1.0),
                       ("optimization/max_vel", 1.0),
                       ("bspline/limit_vel", 1.0),
                       ("fsm.thresh_replan_time", 0.9),
                       ("manager/planning_horizon", 10.5),
                       ("manager/p1_collision_fanout_clearance_m", 2.5),
                       ("grid_map/local_update_range_x", 11.0)):
        actual = _number(manifest.get(key))
        if actual is None or abs(actual - value) > 1e-12:
            errors.append(f"contract mismatch: {key}")
    if manifest.get("p0.horizons_s") != [
        0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.5, 4.5, 5.5, 6.5,
        7.5, 8.5, 9.5, 10.5, 11.5, 12.5, 13.5, 14.5, 15.5, 16.0,
        17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0
    ]:
        errors.append("contract mismatch: p0.horizons_s")
    if manifest.get("manager/p1_collision_fanout_preserve_homotopies") is not True:
        errors.append("contract mismatch: preserve collision homotopies")
    expected_mirror = manifest.get("scenario") == "p1_fork_fused_mirror_v1"
    if manifest.get("manager/p1_collision_fanout_mirror_y") is not expected_mirror:
        errors.append("contract mismatch: collision fanout mirror binding")
    if provenance.get("schema_version") != "p1_evidence_provenance_v4" or not run_id:
        errors.append("invalid provenance schema/run ID")
    if provenance.get("git_worktree_clean") is not True:
        errors.append("run does not bind a clean worktree")
    runtime = provenance.get("runtime_paths", {}) or {}
    if not all(str((runtime.get(key, {}) or {}).get("sha256", "")) for key in (
        "launch", "planner_executable", "bspline_library"
    )):
        errors.append("runtime hashes are incomplete")
    try:
        validator = json.loads((export / "test_planner_validation_summary.json").read_text())
    except (OSError, json.JSONDecodeError):
        validator = {}
    if validator.get("passed") is not True or validator.get("run_id") != run_id:
        errors.append("validator failed or is not provenance-bound")

    try:
        profile_rows = _read_csv(_artifact(
            export, manifest, "p1.accepted_profile_path",
            "planner_p1_accepted_trajectory_risk_profile.csv"))
        context_rows = _read_csv(_artifact(
            export, manifest, "p1.accepted_profile_context_path",
            "planner_p1_accepted_trajectory_risk_profile_context.csv"))
        profile = calibration._decision_profile(profile_rows, context_rows, manifest)
        sequence = str(profile[0].get("profile_seq", ""))
        contexts = [row for row in context_rows if str(row.get("profile_seq", "")) == sequence]
        if len(contexts) != 1 or _identity(contexts[0]) != _identity(profile[0]) or any(
            not _truthy(contexts[0].get(key)) for key in (
                "fresh", "coverage_ok", "spatial_in_bounds", "temporal_in_horizon",
                "frame_match", "generation_match", "query_time_match"
            )
        ) or _number(contexts[0].get("matched_sample_count")) != 200.0:
            errors.append("accepted context is not fresh, complete, and identity-bound")
        if any(row.get("schema_version") != "p1_evidence_provenance_v4"
               or row.get("run_id") != run_id
               or Path(str(row.get("manifest_path", ""))).resolve() != manifest_path.resolve()
               for row in profile):
            errors.append("accepted profile provenance binding failed")
        metrics_only = bool(manifest.get("p1.metrics_only"))
        if any(_truthy(row.get("metrics_only")) != metrics_only for row in profile):
            errors.append("accepted profile metrics-only binding failed")
        if metrics_only and any(_truthy(row.get("applied_to_objective")) for row in profile):
            errors.append("reference profile was applied to the objective")
        if not metrics_only and not all(_truthy(row.get("applied_to_objective")) for row in profile):
            errors.append("enabled profile was not applied to the objective")
        values = [float(row["c_pi"]) for row in profile]
        if len(values) != 200 or any(
            not _truthy(row.get("hit")) or not _truthy(row.get("valid"))
            or _truthy(row.get("stale")) or _truthy(row.get("base_collision_occupied"))
            for row in profile
        ):
            errors.append("accepted profile lacks collision-feasible 200/200 support")
        sufficiency = formal_metrics.sampling_sufficiency(
            profile, resolution_m=float(manifest["p0.resolution_m"]),
            horizons_s=manifest["p0.horizons_s"])
        if not sufficiency.get("passed"):
            errors.append("P0 sampling sufficiency failed")
        occupancy_path = _artifact(
            export, manifest, "p0.occupancy_query_evidence_path",
            "planner_p0_occupancy_query_evidence.csv")
        accepted_corners, occupancy_final = _scan_occupancy_evidence(
            occupancy_path, run_id, manifest_path, profile[0])
        if not formal_metrics.corner_reconstruction_residual(
                profile, accepted_corners).get("complete"):
            errors.append("P0 accepted-corner provenance is incomplete")
        selected_lane = "upper" if sum(float(row["y"]) for row in profile) > 0 else "lower"
        path_length = sum(math.dist(
            tuple(float(a[key]) for key in ("x", "y", "z")),
            tuple(float(b[key]) for key in ("x", "y", "z")),
        ) for a, b in zip(profile, profile[1:]))
        metrics = {
            "mean": sum(values) / len(values),
            "cvar": formal_metrics.smooth_cvar(values), "max": max(values),
            "path_length_m": path_length, "selected_lane": selected_lane,
        }
        truth_estimate_rows = _read_csv(export / "iap_sim_truth_vs_est.csv")
        checkpoint_stamp = float(profile[0]["stamp"])
        nearest = min(
            truth_estimate_rows,
            key=lambda row: abs(float(row["truth_stamp"]) - checkpoint_stamp),
        )
        checkpoint_dt = abs(float(nearest["truth_stamp"]) - checkpoint_stamp)
        localization_error = float(nearest["position_error_m"])
        metrics.update({"checkpoint_truth_dt_s": checkpoint_dt,
                        "localization_error_m": localization_error})
        if checkpoint_dt > 0.10:
            errors.append("truth/estimate checkpoint is not time-aligned within 0.10 s")
        if localization_error > 0.50:
            errors.append("checkpoint localization error exceeds 0.50 m")
    except (OSError, ValueError, KeyError, calibration.CalibrationError) as exc:
        errors.append(f"accepted profile invalid: {exc}")
        profile, metrics = [], {"mean": None, "cvar": None, "max": None,
                                "path_length_m": None, "selected_lane": None,
                                "checkpoint_truth_dt_s": None,
                                "localization_error_m": None}

    precheck = {"passed": False, "status": "INCONCLUSIVE"}
    try:
        optimization_path, candidates_path = _candidate_evidence_paths(
            export, manifest)
        optimization = (_read_csv(optimization_path)
                        if optimization_path is not None else [])
        candidates = _read_csv(candidates_path)
        if optimization_path is not None:
            _validate_provenance_rows(
                optimization, run_id, manifest_path, "candidate optimization")
        _validate_provenance_rows(candidates, run_id, manifest_path, "candidate profile")
        context = profile[0]
        precheck_rows = _prequalification_candidate_rows(candidates, context)
        precheck = formal_metrics.candidate_route_precheck(
            precheck_rows, require_selected=False
        )
        if not precheck.get("passed"):
            errors.append("candidate precheck lacks collision-feasible full-200 upper/lower routes")
    except (OSError, ValueError, KeyError, IndexError) as exc:
        errors.append(f"candidate/occupancy precheck invalid: {exc}")

    result = {
        "run_id": run_id, "export_dir": str(export), "scenario": manifest.get("scenario"),
        "metrics_only": manifest.get("p1.metrics_only"), **metrics,
        "candidate_precheck": precheck, "errors": errors,
        "hard_gates": {"passed": not errors}, "passed": not errors,
    }
    return result


def evaluate_pair(kind: str, reference: dict[str, Any], enabled: dict[str, Any]) -> dict[str, Any]:
    failures = []
    if not reference.get("passed") or not enabled.get("passed"):
        failures.append("one or both run hard gates failed")
    values = [_number(run.get(key)) for run in (reference, enabled)
              for key in ("mean", "cvar", "max", "path_length_m")]
    if any(value is None for value in values) or float(reference.get("path_length_m") or 0) <= 0:
        failures.append("pair metrics are incomplete")
        return {"kind": kind, "passed": False, "failures": failures,
                "mean_improvement": None, "cvar_improvement": None,
                "max_regression": None, "path_growth": None,
                "localization_error_delta_m": None,
                "reference_run_id": reference.get("run_id"),
                "enabled_run_id": enabled.get("run_id")}
    mean_improvement = float(reference["mean"]) - float(enabled["mean"])
    cvar_improvement = float(reference["cvar"]) - float(enabled["cvar"])
    max_regression = float(enabled["max"]) - float(reference["max"])
    path_growth = (float(enabled["path_length_m"]) / float(reference["path_length_m"]) - 1.0)
    localization_delta = abs(
        float(enabled.get("localization_error_m", math.inf))
        - float(reference.get("localization_error_m", math.inf))
    )
    if not math.isfinite(localization_delta) or localization_delta > 0.25:
        failures.append("pairwise localization-error delta exceeded 0.25 m")
    lane = enabled.get("selected_lane")
    if kind == "primary":
        if lane != "lower": failures.append("enabled primary did not select lower route")
        if mean_improvement <= PRIMARY_MEAN_IMPROVEMENT: failures.append("primary mean improvement too small")
        if cvar_improvement <= PRIMARY_CVAR_IMPROVEMENT: failures.append("primary CVaR improvement too small")
        if max_regression > 0: failures.append("primary max regressed")
    elif kind == "mirror":
        if lane != "upper": failures.append("enabled mirror did not select upper route")
        if mean_improvement <= 0 or cvar_improvement <= 0 or max_regression > 0:
            failures.append("mirror risk direction is inconsistent")
    elif kind == "null":
        if abs(mean_improvement) > TAU_MEAN: failures.append("null mean change exceeded limit")
        if abs(cvar_improvement) > TAU_CVAR: failures.append("null CVaR change exceeded limit")
        if path_growth > 0.05 + 1e-12: failures.append("null path growth exceeded 5%")
    elif kind == "soft_risk":
        if lane != "lower": failures.append("soft-risk enabled did not route below island")
        if mean_improvement <= 0 or cvar_improvement <= 0 or max_regression > 0:
            failures.append("soft-risk profile did not improve without max regression")
    else:
        failures.append(f"unknown pair kind: {kind}")
    return {"kind": kind, "passed": not failures, "failures": failures,
            "mean_improvement": mean_improvement, "cvar_improvement": cvar_improvement,
            "max_regression": max_regression, "path_growth": path_growth,
            "localization_error_delta_m": localization_delta,
            "reference_run_id": reference.get("run_id"), "enabled_run_id": enabled.get("run_id")}


def analyze_manifest(path: Path, output_dir: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text())
    runs, pairs = [], []
    for pair in payload.get("pairs", []):
        reference = analyze_run(pair["reference_export"])
        enabled = analyze_run(pair["enabled_export"])
        runs.extend((reference, enabled))
        pairs.append(evaluate_pair(pair["kind"], reference, enabled))
    result = {"schema_version": "p1_2_prequalification_v1",
              "passed": len(pairs) == 5 and all(pair["passed"] for pair in pairs),
              "runs": runs, "pairs": pairs}
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "p1_2_prequalification_summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    with (output_dir / "p1_2_prequalification_runs.csv").open("w", newline="") as handle:
        fields = ["run_id", "scenario", "metrics_only", "passed", "selected_lane",
                  "mean", "cvar", "max", "path_length_m", "localization_error_m",
                  "checkpoint_truth_dt_s", "export_dir"]
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(runs)
    with (output_dir / "p1_2_prequalification_pairs.csv").open("w", newline="") as handle:
        fields = ["kind", "reference_run_id", "enabled_run_id", "passed",
                  "mean_improvement", "cvar_improvement", "max_regression", "path_growth",
                  "localization_error_delta_m"]
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader(); writer.writerows(pairs)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-manifest", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    try:
        result = analyze_manifest(args.runs_manifest, args.output_dir)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(json.dumps({"passed": False, "error": str(exc)}, indent=2))
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
