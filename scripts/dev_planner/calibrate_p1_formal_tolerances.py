#!/usr/bin/env python3
"""Freeze P1 formal null-effect tolerances from ten predeclared run pairs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


_METRICS_PATH = Path(__file__).with_name("p1_formal_metrics.py")
_METRICS_SPEC = importlib.util.spec_from_file_location("p1_formal_metrics", _METRICS_PATH)
formal_metrics = importlib.util.module_from_spec(_METRICS_SPEC)
assert _METRICS_SPEC.loader is not None
_METRICS_SPEC.loader.exec_module(formal_metrics)


PAIR_SCHEMA = "p1_formal_calibration_pairs_v1"
CALIBRATION_SCHEMA = "p1_formal_tolerance_calibration_v1"
EVIDENCE_SCHEMA = "p1_evidence_provenance_v4"
PAIR_COUNT = 10


class CalibrationError(ValueError):
    pass


def _read_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise CalibrationError(f"cannot read JSON {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise CalibrationError(f"JSON root must be an object: {path}")
    return payload


def _read_csv(path: Path) -> list[dict[str, Any]]:
    try:
        with path.open(newline="") as handle:
            return list(csv.DictReader(handle))
    except OSError as exc:
        raise CalibrationError(f"cannot read CSV {path}: {exc}") from exc


def _write_csv(path: Path, fields: list[str], rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _artifact_path(export: Path, manifest: dict[str, Any], key: str, filename: str) -> Path:
    raw = Path(str(manifest.get(key, filename))).expanduser()
    path = raw.resolve() if raw.is_absolute() else (export / raw).resolve()
    if path.parent != export or path.name != filename:
        raise CalibrationError(f"artifact path escapes export or has wrong name: {path}")
    return path


def _decision_profile(
    rows: list[dict[str, Any]],
    contexts: list[dict[str, Any]],
    manifest: dict[str, Any],
) -> list[dict[str, Any]]:
    checkpoint = ((manifest.get("scenario_contract", {}) or {}).get(
        "decision_checkpoint", {}) or {})
    if (
        checkpoint.get("truth_source_topic") != "/sim/drone_0/truth_odom"
        or checkpoint.get("profile_sample_zero_binding")
        != "planner_truth_odom_state_at_planning_start"
    ):
        raise CalibrationError("decision checkpoint is not bound to planner truth odometry")
    sequences = sorted({
        value for value in (_float(row.get("profile_seq")) for row in rows)
        if value is not None
    })
    if not sequences:
        raise CalibrationError("accepted profile has no finite profile_seq")
    sequence_distance = {}
    for sequence in sequences:
        sequence_rows = [row for row in rows if _float(row.get("profile_seq")) == sequence]
        first = min(sequence_rows, key=lambda row: _float(row.get("sample_index")) or 0.0)
        first_x = _float(first.get("x"))
        if first_x is not None and abs(first_x - (-9.5)) <= 0.4:
            sequence_distance[sequence] = abs(first_x - (-9.5))
    sequence_events = {
        sequence: _float(context.get("planning_start_s"))
        for sequence in sequence_distance
        for context in contexts
        if _float(context.get("profile_seq")) == sequence
    }
    if set(sequence_events) != set(sequence_distance) or any(
            event is None for event in sequence_events.values()):
        raise CalibrationError(
            "accepted profile decision checkpoint at truth x=-9.5+/-0.4 m is missing/ambiguous"
        )
    event_distance = {}
    for sequence, event in sequence_events.items():
        event_distance[event] = min(
            event_distance.get(event, math.inf), sequence_distance[sequence])
    if not event_distance:
        raise CalibrationError(
            "accepted profile decision checkpoint at truth x=-9.5+/-0.4 m is missing/ambiguous"
        )
    minimum_distance = min(event_distance.values())
    nearest_events = [event for event, distance in event_distance.items()
                      if abs(distance - minimum_distance) <= 1e-12]
    if len(nearest_events) != 1:
        raise CalibrationError(
            "accepted profile decision checkpoint at truth x=-9.5+/-0.4 m is missing/ambiguous"
        )
    qualifying = [sequence for sequence, event in sequence_events.items()
                  if event == nearest_events[0]]
    if len(qualifying) > 1:
        observations = [
            sequence for sequence in qualifying
            if any(
                str(row.get("fallback_reason", "")) in {
                    "metrics_only_reference_observation",
                    "p1_enabled_retained_incumbent_observation",
                }
                for row in rows if _float(row.get("profile_seq")) == sequence
            )
        ]
        if len(observations) != 1:
            raise CalibrationError(
                "accepted profile decision checkpoint at truth x=-9.5+/-0.4 m is missing/ambiguous"
            )
        qualifying = observations
    selected = qualifying[0]
    result = [row for row in rows if _float(row.get("profile_seq")) == selected]
    matching_contexts = [
        row for row in contexts if _float(row.get("profile_seq")) == selected
    ]
    if (
        len(matching_contexts) != 1
        or _float(matching_contexts[0].get("planning_start_s")) is None
    ):
        raise CalibrationError("decision profile lacks one truth-time planning context")
    indices = [_float(row.get("sample_index")) for row in result]
    integer_indices = [int(value) for value in indices if value is not None and value.is_integer()]
    if len(result) != 200 or set(integer_indices) != set(range(200)):
        raise CalibrationError("accepted profile does not have 200/200 support")
    return sorted(result, key=lambda row: int(float(row["sample_index"])))


def _identity_tuple(row: dict[str, Any]) -> tuple[str, ...]:
    return tuple(str(row.get(key, "")).strip() for key in (
        "planning_attempt_id", "candidate_id", "snapshot_generation_id", "query_base_time_s"
    ))


def _runtime_identity(manifest: dict[str, Any]) -> dict[str, str]:
    provenance = manifest.get("artifact_provenance", {}) or {}
    runtime = provenance.get("runtime_paths", {}) or {}
    return {
        key: str((runtime.get(key, {}) or {}).get("sha256", ""))
        for key in ("launch", "planner_executable", "bspline_library")
    }


def _configuration_identity(manifest: dict[str, Any]) -> dict[str, Any]:
    provenance = manifest.get("artifact_provenance", {}) or {}
    identity = {
        "git_commit": provenance.get("git_commit"),
        "baseline_commit": provenance.get("baseline_commit"),
        "experiment": manifest.get("experiment"),
        "scenario": manifest.get("scenario"),
        "scenario_fingerprint": manifest.get("scenario_fingerprint"),
        "scenario_contract": manifest.get("scenario_contract"),
        "planner_safety_profile": manifest.get("planner_safety_profile"),
        "p1.use_integrity_cost": manifest.get("p1.use_integrity_cost"),
        "p1.max_candidates_per_attempt": manifest.get("p1.max_candidates_per_attempt"),
        "p1.lambda_integrity": manifest.get("p1.lambda_integrity"),
        "p1.objective_aggregation_mode": manifest.get("p1.objective_aggregation_mode"),
        "p1.smooth_cvar_alpha": manifest.get("p1.smooth_cvar_alpha"),
        "p1.smooth_max_temperature": manifest.get("p1.smooth_max_temperature"),
        "p1.normalization_budget_fraction": manifest.get("p1.normalization_budget_fraction"),
        "grid_map/local_update_range_x": manifest.get("grid_map/local_update_range_x"),
        "manager/planning_horizon": manifest.get("manager/planning_horizon"),
        "manager/p1_collision_fanout_clearance_m": manifest.get(
            "manager/p1_collision_fanout_clearance_m"),
        "manager/p1_collision_fanout_preserve_homotopies": manifest.get(
            "manager/p1_collision_fanout_preserve_homotopies"),
        "manager/p1_collision_fanout_mirror_y": manifest.get(
            "manager/p1_collision_fanout_mirror_y"),
        "run_duration_s": manifest.get("run_duration_s"),
        "validation_duration_s": manifest.get("validation_duration_s"),
        "planner_start_delay_s": manifest.get("planner_start_delay_s"),
        "manager/max_vel": manifest.get("manager/max_vel"),
        "optimization/max_vel": manifest.get("optimization/max_vel"),
        "bspline/limit_vel": manifest.get("bspline/limit_vel"),
        "fsm.thresh_replan_time": manifest.get("fsm.thresh_replan_time"),
        "record_bag": manifest.get("record_bag"),
        "run_validator": manifest.get("run_validator"),
    }
    identity.update({key: manifest.get(key) for key in formal_metrics.P0_CONFIGURATION_KEYS})
    return identity


def _validate_fixed_contract(manifest: dict[str, Any]) -> None:
    expected = {
        "planner_safety_profile": "p1",
        "p0.enable_risk_grid": True,
        "p1.use_integrity_cost": True,
        "p1.metrics_only": True,
        "p1.objective_aggregation_mode": "fixed_200_smooth_cvar",
        "record_bag": False,
        "run_validator": True,
        "scenario": "p1_fork_fused_v1",
        "manager/p1_collision_fanout_preserve_homotopies": True,
        "manager/p1_collision_fanout_mirror_y": False,
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise CalibrationError(f"calibration run contract mismatch: {key}")
    numeric = {
        "run_duration_s": 90.0,
        "validation_duration_s": 90.0,
        "planner_start_delay_s": 10.0,
        "manager/max_vel": 1.0,
        "optimization/max_vel": 1.0,
        "bspline/limit_vel": 1.0,
        "fsm.thresh_replan_time": 0.9,
        "manager/planning_horizon": 10.5,
        "manager/p1_collision_fanout_clearance_m": 2.5,
        "p1.lambda_integrity": 1.0e-5,
        "p1.smooth_cvar_alpha": 0.90,
        "p1.smooth_max_temperature": 0.01,
        "p1.normalization_budget_fraction": 0.30,
        "grid_map/local_update_range_x": 11.0,
    }
    for key, expected_value in numeric.items():
        value = _float(manifest.get(key))
        if value is None or abs(value - expected_value) > 1.0e-12:
            raise CalibrationError(f"calibration run contract mismatch: {key}")


def _load_run(export_value: Any) -> dict[str, Any]:
    export = Path(str(export_value)).expanduser().resolve()
    manifest_path = export / "test_planner_manifest.json"
    manifest = _read_json(manifest_path)
    provenance = manifest.get("artifact_provenance", {}) or {}
    run_id = str(provenance.get("run_id", ""))
    if provenance.get("schema_version") != EVIDENCE_SCHEMA or not run_id:
        raise CalibrationError(f"invalid evidence schema/run ID in {export}")
    if provenance.get("git_worktree_clean") is not True:
        raise CalibrationError(f"run does not prove a clean git worktree: {run_id}")
    if Path(str(manifest.get("export_dir", ""))).resolve() != export:
        raise CalibrationError(f"manifest export binding mismatch: {run_id}")
    _validate_fixed_contract(manifest)
    runtime_identity = _runtime_identity(manifest)
    if not all(runtime_identity.values()):
        raise CalibrationError(f"runtime identity is incomplete: {run_id}")

    profile_path = _artifact_path(
        export, manifest, "p1.accepted_profile_path",
        "planner_p1_accepted_trajectory_risk_profile.csv",
    )
    context_path = _artifact_path(
        export, manifest, "p1.accepted_profile_context_path",
        "planner_p1_accepted_trajectory_risk_profile_context.csv",
    )
    corner_path = _artifact_path(
        export, manifest, "p0.occupancy_query_evidence_path",
        "planner_p0_occupancy_query_evidence.csv",
    )
    contexts = _read_csv(context_path)
    profile_rows = _decision_profile(
        _read_csv(profile_path), contexts, manifest
    )
    for row in profile_rows:
        if row.get("schema_version") != EVIDENCE_SCHEMA or row.get("run_id") != run_id:
            raise CalibrationError(f"profile provenance mismatch: {run_id}")
        if Path(str(row.get("manifest_path", ""))).resolve() != manifest_path.resolve():
            raise CalibrationError(f"profile manifest binding mismatch: {run_id}")
        if not _bool(row.get("metrics_only")) or _bool(row.get("applied_to_objective")):
            raise CalibrationError(f"profile is not metrics-only/not-applied: {run_id}")
    # This call owns the fail-closed finite/full-support check.
    try:
        sufficiency = formal_metrics.sampling_sufficiency(
            profile_rows,
            resolution_m=float(manifest["p0.resolution_m"]),
            horizons_s=manifest["p0.horizons_s"],
        )
    except ValueError as exc:
        raise CalibrationError(f"complete matched support failed: {run_id}: {exc}") from exc
    if not sufficiency["passed"]:
        raise CalibrationError(f"sampling sufficiency failed: {run_id}")

    sequence = str(profile_rows[0].get("profile_seq", ""))
    matching_contexts = [row for row in contexts if str(row.get("profile_seq", "")) == sequence]
    if len(matching_contexts) != 1:
        raise CalibrationError(f"accepted profile context is missing/ambiguous: {run_id}")
    context = matching_contexts[0]
    if (
        _identity_tuple(context) != _identity_tuple(profile_rows[0])
        or any(not _bool(context.get(key)) for key in (
            "fresh", "coverage_ok", "spatial_in_bounds", "temporal_in_horizon",
            "frame_match", "generation_match", "query_time_match",
        ))
        or _float(context.get("expected_sample_count")) != 200.0
        or _float(context.get("matched_sample_count")) != 200.0
        or _float(context.get("match_ratio")) != 1.0
    ):
        raise CalibrationError(f"accepted profile context is unhealthy: {run_id}")

    validator_path = export / "test_planner_validation_summary.json"
    validator = _read_json(validator_path)
    if (
        validator.get("passed") is not True
        or validator.get("schema_version") != EVIDENCE_SCHEMA
        or validator.get("run_id") != run_id
        or Path(str(validator.get("manifest_path", ""))).resolve() != manifest_path.resolve()
    ):
        raise CalibrationError(f"validator identity/health failed: {run_id}")

    accepted_corners = [
        row for row in _read_csv(corner_path)
        if str(row.get("phase", "")) == "accepted"
        and _identity_tuple(row) == _identity_tuple(profile_rows[0])
    ]
    grid = formal_metrics.corner_reconstruction_residual(profile_rows, accepted_corners)
    if not grid["complete"]:
        raise CalibrationError(f"corner reconstruction is incomplete: {run_id}")
    return {
        "export": export,
        "manifest_path": manifest_path,
        "manifest": manifest,
        "run_id": run_id,
        "profile_rows": profile_rows,
        "runtime_identity": runtime_identity,
        "configuration_identity": _configuration_identity(manifest),
        "sampling_sufficiency": sufficiency,
        "epsilon_grid": float(grid["epsilon_grid"]),
        # record_bag=false has no recorder-owned finalizer.  Prefer its
        # explicit end epoch when present; otherwise the latest required
        # artifact mtime is the fail-closed completion boundary.
        "process_end_epoch_s": (
            _float(provenance.get("process_end_epoch_s"))
            or max(path.stat().st_mtime for path in (
                manifest_path, profile_path, context_path, corner_path,
                validator_path,
            ))
        ),
    }


def _plot_scores(rows: list[dict[str, Any]], path: Path) -> None:
    x = list(range(1, len(rows) + 1))
    fig, axis = plt.subplots(figsize=(8.2, 4.6))
    for key, label, color in (
        ("s_mean", "|mean A-B|", "#2563eb"),
        ("s_cvar", "|CVaR A-B|", "#7c3aed"),
        ("s_max", "|max A-B|", "#dc2626"),
    ):
        axis.plot(x, [row[key] for row in rows], marker="o", label=label, color=color)
    axis.set_xlabel("null-effect pair")
    axis.set_ylabel("absolute score")
    axis.set_title("P1-1/P1-1 null-effect distribution (10-pair 90% conformal max)")
    axis.grid(True, alpha=0.25)
    axis.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _plot_error_budget(payload: dict[str, Any], path: Path) -> None:
    errors = payload["deterministic_error"]
    thresholds = payload["thresholds"]
    labels = ["epsilon_grid", "epsilon_resample", "epsilon_det", "tau_mean", "tau_cvar", "tau_max"]
    values = [errors["epsilon_grid"], errors["epsilon_resample"], errors["epsilon_det"],
              thresholds["tau_mean"], thresholds["tau_cvar"], thresholds["tau_max"]]
    fig, axis = plt.subplots(figsize=(8.2, 4.6))
    axis.bar(labels, values, color=["#64748b", "#94a3b8", "#334155", "#2563eb", "#7c3aed", "#dc2626"])
    axis.set_ylabel("c_pi")
    axis.set_title("Frozen P1 formal tolerance error budget")
    axis.tick_params(axis="x", rotation=25)
    axis.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def calibrate(
    pairs_manifest: str | Path,
    output_dir: str | Path,
    *,
    generated_at_epoch_s: float | None = None,
    generated_at_utc: str | None = None,
) -> dict[str, Any]:
    pairs_path = Path(pairs_manifest).expanduser().resolve()
    declaration = _read_json(pairs_path)
    pairs = declaration.get("pairs")
    if declaration.get("schema_version") != PAIR_SCHEMA or not isinstance(pairs, list):
        raise CalibrationError(f"pairs manifest must use {PAIR_SCHEMA}")
    if len(pairs) != PAIR_COUNT:
        raise CalibrationError("calibration requires exactly 10 declared pairs")
    calibration_id = str(declaration.get("calibration_id", "")).strip()
    if not calibration_id:
        raise CalibrationError("pairs manifest requires calibration_id")
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    run_ids: set[str] = set()
    loaded_pairs: list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]] = []
    all_runs: list[dict[str, Any]] = []
    validity_rows: list[dict[str, Any]] = []
    validation_errors: list[str] = []
    for pair_index, pair in enumerate(pairs, start=1):
        if not isinstance(pair, dict):
            raise CalibrationError(f"pair {pair_index} must be an object")
        pair_id = str(pair.get("pair_id", "")).strip() or f"pair-{pair_index:02d}"
        pair_runs: dict[str, dict[str, Any]] = {}
        for role, export_key in (("A", "run_a_export"), ("B", "run_b_export")):
            export_value = pair.get(export_key)
            try:
                run = _load_run(export_value)
            except CalibrationError as exc:
                validation_errors.append(f"{pair_id}/{role}: {exc}")
                validity_rows.append({
                    "pair_id": pair_id, "role": role, "run_id": "",
                    "export_dir": str(export_value or ""), "valid": False,
                    "reason": str(exc),
                })
                continue
            if run["run_id"] in run_ids:
                error = f"duplicate run ID in calibration manifest: {run['run_id']}"
                validation_errors.append(error)
                validity_rows.append({
                    "pair_id": pair_id, "role": role, "run_id": run["run_id"],
                    "export_dir": str(run["export"]), "valid": False,
                    "reason": error,
                })
                continue
            run_ids.add(run["run_id"])
            all_runs.append(run)
            pair_runs[role] = run
            validity_rows.append({
                "pair_id": pair_id, "role": role, "run_id": run["run_id"],
                "export_dir": str(run["export"]), "valid": True, "reason": "",
                **run["sampling_sufficiency"], "epsilon_grid": run["epsilon_grid"],
            })
        if set(pair_runs) == {"A", "B"}:
            loaded_pairs.append(({"pair_id": pair_id}, pair_runs["A"], pair_runs["B"]))

    validity_fields = [
        "pair_id", "role", "run_id", "export_dir", "valid", "reason",
        "passed", "max_spatial_gap_m", "spatial_gap_limit_m",
        "max_temporal_gap_s", "temporal_gap_limit_s", "epsilon_grid",
    ]
    _write_csv(
        output / "p1_formal_calibration_validity.csv",
        validity_fields,
        validity_rows,
    )
    if validation_errors:
        raise CalibrationError("; ".join(validation_errors))

    reference_config = all_runs[0]["configuration_identity"]
    reference_runtime = all_runs[0]["runtime_identity"]
    if any(run["configuration_identity"] != reference_config for run in all_runs[1:]):
        raise CalibrationError("calibration runs do not share one configuration identity")
    if any(run["runtime_identity"] != reference_runtime for run in all_runs[1:]):
        raise CalibrationError("calibration runs do not share one runtime identity")
    chronological_runs = sorted(
        all_runs,
        key=lambda run: float(
            run["manifest"]["artifact_provenance"]["process_start_epoch_s"]),
    )
    for previous, current in zip(chronological_runs, chronological_runs[1:]):
        previous_end = float(previous["process_end_epoch_s"])
        current_start = float(
            current["manifest"]["artifact_provenance"]["process_start_epoch_s"])
        if current_start < previous_end:
            raise CalibrationError(
                "calibration runs overlap; all 20 null-effect runs must be serial")

    generated_epoch = time.time() if generated_at_epoch_s is None else float(generated_at_epoch_s)
    ended = [run["process_end_epoch_s"] for run in all_runs]
    if any(value is None for value in ended) or generated_epoch <= max(float(value) for value in ended if value is not None):
        raise CalibrationError("calibration must be generated after all calibration runs finish")
    generated_utc = generated_at_utc or time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(generated_epoch))

    pair_rows: list[dict[str, Any]] = []
    epsilon_resample = 0.0
    for pair_info, run_a, run_b in loaded_pairs:
        comparison = formal_metrics.compare_profiles(run_a["profile_rows"], run_b["profile_rows"])
        epsilon_resample = max(epsilon_resample, comparison["epsilon_resample"])
        pair_rows.append({
            "pair_id": pair_info["pair_id"],
            "run_a_id": run_a["run_id"], "run_b_id": run_b["run_id"],
            "run_a_mean": comparison["current_mean"], "run_b_mean": comparison["reference_mean"],
            "run_a_cvar": comparison["current_cvar"], "run_b_cvar": comparison["reference_cvar"],
            "run_a_max": comparison["current_max"], "run_b_max": comparison["reference_max"],
            "s_mean": abs(comparison["current_mean"] - comparison["reference_mean"]),
            "s_cvar": abs(comparison["current_cvar"] - comparison["reference_cvar"]),
            "s_max": abs(comparison["current_max"] - comparison["reference_max"]),
            "common_terminal_arc_length_m": comparison["common_terminal_arc_length_m"],
            "epsilon_grid_a": run_a["epsilon_grid"], "epsilon_grid_b": run_b["epsilon_grid"],
            "epsilon_resample": comparison["epsilon_resample"], "valid": True,
        })
    epsilon_grid = max(run["epsilon_grid"] for run in all_runs)
    epsilon_det = 2.0 * (epsilon_grid + epsilon_resample)
    max_scores = {
        key: max(row[key] for row in pair_rows)
        for key in ("s_mean", "s_cvar", "s_max")
    }
    thresholds = {
        "tau_mean": max_scores["s_mean"] + epsilon_det,
        "tau_cvar": max_scores["s_cvar"] + epsilon_det,
        "tau_max": max_scores["s_max"] + epsilon_det,
    }
    payload = {
        "schema_version": CALIBRATION_SCHEMA,
        "calibration_id": calibration_id,
        "generated_at_utc": generated_utc,
        "generated_at_epoch_s": generated_epoch,
        "pairs_manifest": str(pairs_path),
        "pairs_manifest_sha256": _sha256(pairs_path),
        "git_commit": reference_config["git_commit"],
        "baseline_commit": reference_config["baseline_commit"],
        "scenario": reference_config["scenario"],
        "scenario_fingerprint": reference_config["scenario_fingerprint"],
        "scenario_contract": reference_config["scenario_contract"],
        "experiment": reference_config["experiment"],
        "runtime_hashes": reference_runtime,
        "configuration_identity": reference_config,
        "p0": {
            "resolution_m": reference_config["p0.resolution_m"],
            "horizons_s": reference_config["p0.horizons_s"],
        },
        "smooth_cvar": {
            "mode": "fixed_200_smooth_cvar", "alpha": 0.90,
            "temperature": 0.01, "eta_bisection_iterations": 100,
        },
        "lambda_integrity": 1.0e-5,
        "normalization_budget_fraction": 0.30,
        "comparison": {
            "mode": "first_deterministic_decision_checkpoint_fixed_200", "derived": False,
            "sample_count": 200,
        },
        "conformal": {
            "pair_count": 10, "coverage": 0.90,
            "upper_bound_order_statistic": "maximum",
        },
        "run_ids": [run["run_id"] for run in all_runs],
        "pairs": pair_rows,
        "null_effect_maxima": max_scores,
        "deterministic_error": {
            "epsilon_grid": epsilon_grid,
            "epsilon_resample": epsilon_resample,
            "epsilon_det": epsilon_det,
            "formula": "2*(epsilon_grid + epsilon_resample)",
        },
        "thresholds": thresholds,
        "threshold_formulas": {
            "tau_mean": "max_j |mean_A_j-mean_B_j| + epsilon_det",
            "tau_cvar": "max_j |cvar_A_j-cvar_B_j| + epsilon_det",
            "tau_max": "max_j |max_A_j-max_B_j| + epsilon_det",
        },
    }
    json_path = output / "p1_formal_tolerance_calibration_v1.json"
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    _write_csv(output / "p1_formal_calibration_pairs.csv", list(pair_rows[0]), pair_rows)
    _plot_scores(pair_rows, output / "p1_formal_null_effect_distribution.png")
    _plot_error_budget(payload, output / "p1_formal_error_budget.png")
    return payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairs-manifest", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = calibrate(args.pairs_manifest, args.output_dir)
    except CalibrationError as exc:
        print(json.dumps({"passed": False, "error": str(exc)}, indent=2))
        return 2
    print(json.dumps({
        "passed": True,
        "calibration_id": result["calibration_id"],
        "thresholds": result["thresholds"],
        "output": str((args.output_dir / "p1_formal_tolerance_calibration_v1.json").resolve()),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
