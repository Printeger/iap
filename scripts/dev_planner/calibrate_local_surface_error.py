#!/usr/bin/env python3
"""Offline-only calibration of the registered-surface error bound.

The planner never imports this module and never reads simulation truth.  A run
directory must contain iap_sim_truth_vs_est.csv, iap_registered_surface_offset.csv
and iap_icp.csv.  The resulting value is accepted only when an independent
held-out run stays within the proposed bound.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Iterable

HORIZONS_S = (0.1, 0.2, 0.5, 1.0)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _quantile(values: Iterable[float], probability: float) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        raise ValueError("quantile requires at least one finite value")
    rank = probability * (len(ordered) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _truth_samples(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    samples = []
    for row in _rows(path):
        stamp = float(row.get("est_stamp", row.get("stamp_s", "nan")))
        error = tuple(
            float(row[f"truth_{axis}"]) - float(row[f"est_{axis}"])
            for axis in "xyz"
        )
        if math.isfinite(stamp) and all(math.isfinite(value) for value in error):
            samples.append((stamp, error))
    return sorted(samples)


def relative_pose_errors(
    path: Path, horizons: Iterable[float] = HORIZONS_S
) -> dict[str, list[float]]:
    return {
        horizon: [value for _, value in samples]
        for horizon, samples in relative_pose_error_samples(path, horizons).items()
    }


def relative_pose_error_samples(
    path: Path, horizons: Iterable[float] = HORIZONS_S
) -> dict[str, list[tuple[float, float]]]:
    samples = _truth_samples(path)
    if len(samples) < 2:
        raise ValueError(f"not enough truth/estimate samples in {path}")
    result: dict[str, list[tuple[float, float]]] = {
        f"{horizon:.1f}": [] for horizon in horizons
    }
    for index, (stamp, error) in enumerate(samples):
        for horizon in horizons:
            target = stamp + horizon
            nearest = min(
                range(index + 1, len(samples)),
                key=lambda candidate: abs(samples[candidate][0] - target),
                default=-1,
            )
            if nearest < 0 or abs(samples[nearest][0] - target) > 0.05:
                continue
            later = samples[nearest][1]
            result[f"{horizon:.1f}"].append(
                (
                    stamp,
                    math.sqrt(
                        sum((later[axis] - error[axis]) ** 2 for axis in range(3))
                    ),
                )
            )
    if not any(result.values()):
        raise ValueError(f"no horizon-aligned truth/estimate pairs in {path}")
    return result


def _column(path: Path, names: tuple[str, ...]) -> list[float]:
    values = []
    for row in _rows(path):
        raw = next((row[name] for name in names if name in row), None)
        if raw is None:
            continue
        value = float(raw)
        if math.isfinite(value):
            values.append(value)
    if not values:
        raise ValueError(f"none of {names} has finite data in {path}")
    return values


def _timed_column(
    path: Path,
    names: tuple[str, ...],
    stamp_names: tuple[str, ...] = ("stamp_s", "stamp", "est_stamp"),
) -> list[tuple[float, float]]:
    values = []
    for row in _rows(path):
        raw_value = next((row[name] for name in names if name in row), None)
        raw_stamp = next((row[name] for name in stamp_names if name in row), None)
        if raw_value is None or raw_stamp is None:
            continue
        stamp, value = float(raw_stamp), float(raw_value)
        if math.isfinite(stamp) and math.isfinite(value):
            values.append((stamp, value))
    return sorted(values)


def _pearson(lhs: list[float], rhs: list[float]) -> float | None:
    size = min(len(lhs), len(rhs))
    if size < 2:
        return None
    lhs, rhs = lhs[:size], rhs[:size]
    lhs_mean, rhs_mean = sum(lhs) / size, sum(rhs) / size
    numerator = sum((x - lhs_mean) * (y - rhs_mean) for x, y in zip(lhs, rhs))
    lhs_energy = sum((x - lhs_mean) ** 2 for x in lhs)
    rhs_energy = sum((y - rhs_mean) ** 2 for y in rhs)
    if lhs_energy <= 0.0 or rhs_energy <= 0.0:
        return None
    return numerator / math.sqrt(lhs_energy * rhs_energy)


def _aligned_pearson(
    lhs: list[tuple[float, float]],
    rhs: list[tuple[float, float]],
    maximum_delta_s: float = 0.05,
) -> float | None:
    if not lhs or not rhs:
        return None
    aligned_lhs, aligned_rhs = [], []
    for stamp, value in lhs:
        nearest_stamp, nearest_value = min(rhs, key=lambda item: abs(item[0] - stamp))
        if abs(nearest_stamp - stamp) <= maximum_delta_s:
            aligned_lhs.append(value)
            aligned_rhs.append(nearest_value)
    return _pearson(aligned_lhs, aligned_rhs)


def load_run(directory: Path) -> dict[str, object]:
    pose_timed_by_horizon = relative_pose_error_samples(
        directory / "iap_sim_truth_vs_est.csv"
    )
    pose_by_horizon = {
        horizon: [value for _, value in samples]
        for horizon, samples in pose_timed_by_horizon.items()
    }
    pose_errors = [value for values in pose_by_horizon.values() for value in values]
    surface_offsets = _column(
        directory / "iap_registered_surface_offset.csv",
        ("surface_offset_m", "one_sided_surface_offset_m"),
    )
    icp_rmse = _column(directory / "iap_icp.csv", ("rmse", "icp_rmse_m"))
    icp_rmse_timed = _timed_column(
        directory / "iap_icp.csv", ("rmse", "icp_rmse_m")
    )
    surface_offsets_timed = _timed_column(
        directory / "iap_registered_surface_offset.csv",
        ("surface_offset_m", "one_sided_surface_offset_m"),
    )
    return {
        "directory": str(directory),
        "pose_by_horizon_m": pose_by_horizon,
        "pose_errors_m": pose_errors,
        "surface_offsets_m": surface_offsets,
        "icp_rmse_m": icp_rmse,
        "icp_pose_error_pearson_by_horizon": {
            horizon: _aligned_pearson(icp_rmse_timed, samples)
            for horizon, samples in pose_timed_by_horizon.items()
        },
        "icp_surface_offset_pearson": _aligned_pearson(
            icp_rmse_timed, surface_offsets_timed
        ),
    }


def calibrate(calibration_runs: list[dict[str, object]], held_out: dict[str, object]) -> dict[str, object]:
    if len(calibration_runs) != 3:
        raise ValueError("exactly three calibration runs are required")
    pose = [value for run in calibration_runs for value in run["pose_errors_m"]]
    surface = [value for run in calibration_runs for value in run["surface_offsets_m"]]
    pose_q999 = _quantile(pose, 0.999)
    surface_q999 = _quantile(surface, 0.999)
    bound = max(0.02, pose_q999, surface_q999) + 0.01
    held_out_max = max(
        max(held_out["pose_errors_m"]), max(held_out["surface_offsets_m"])
    )
    passed = held_out_max <= bound
    payload = {
        "schema_version": "iap-local-surface-calibration-v1",
        "calibration_run_count": 3,
        "pose_error_q99_9_m": pose_q999,
        "surface_offset_q99_9_m": surface_q999,
        "local_surface_error_bound_m": bound,
        "held_out_max_error_m": held_out_max,
        "held_out_passed": passed,
        "status": "PASS" if passed else "FAIL_HELD_OUT_EXCEEDED_BOUND",
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    payload["calibration_id"] = "local-surface-v1-" + hashlib.sha256(
        canonical.encode("utf-8")
    ).hexdigest()[:16]
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration-run", action="append", type=Path, required=True)
    parser.add_argument("--held-out-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        run_directories = [path.resolve() for path in args.calibration_run]
        held_out_directory = args.held_out_run.resolve()
        all_directories = run_directories + [held_out_directory]
        if len(run_directories) != 3:
            raise ValueError("exactly three calibration-run directories are required")
        if len(set(all_directories)) != len(all_directories):
            raise ValueError(
                "three calibration runs and the held-out run must be distinct directories"
            )
        calibration_runs = [load_run(path) for path in run_directories]
        held_out = load_run(held_out_directory)
        result = calibrate(calibration_runs, held_out)
        result["runs"] = calibration_runs
        result["held_out"] = held_out
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    except (OSError, KeyError, ValueError) as error:
        parser.error(str(error))
    return 0 if result["held_out_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
