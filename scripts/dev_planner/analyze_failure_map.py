#!/usr/bin/env python3
"""Replay a frozen failure map through the planner's C++ clearance and A*."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess

from replay_failure_map import load_snapshot


def _backend_path():
    override = os.environ.get("IAP_FAILURE_MAP_REPLAY_BIN")
    if override:
        return Path(override)
    installed = (Path(__file__).resolve().parents[4] / "install" /
                 "ego_planner/lib/ego_planner/failure_map_replay")
    if installed.is_file():
        return installed
    executable = shutil.which("failure_map_replay")
    if executable:
        return Path(executable)
    raise FileNotFoundError("build ego_planner to install failure_map_replay")


def _point(value):
    if len(value) != 3 or not all(isinstance(x, (int, float)) and
                                  math.isfinite(x) for x in value):
        raise ValueError("invalid three-dimensional snapshot coordinate")
    return " ".join(format(float(x), ".17g") for x in value)


def _number(value):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("snapshot has unavailable motion or time evidence")
    return format(float(value), ".17g")


def _inconclusive(directory, reason):
    return {"schema_version": "iap_failure_reachability_v2",
            "classification": reason, "snapshot": str(directory)}


def inspect(directory, budget_s=120.0, backend=None):
    directory, meta, _ = load_snapshot(directory)
    if meta.get("schema_version") != "iap_gridmap_failure_v2":
        raise ValueError("same-rule replay requires a v2 failure snapshot")
    if meta.get("kind") == "candidate":
        return _inconclusive(directory, "INCONCLUSIVE_CANDIDATE_ONLY")
    if not math.isfinite(budget_s) or budget_s <= 0:
        raise ValueError("budget must be a finite positive number")
    required = ("planning_time_s", "cloud_stamp_s", "motion_stamp_s",
                "motion_error_proxy_m", "motion_body_radius_m",
                "motion_tracking_reserve_m", "motion_budget_m",
                "motion_max_age_s", "environment_max_age_s")
    try:
        motion_values = [_number(meta[name]) for name in required]
    except (KeyError, ValueError):
        return _inconclusive(directory,
                             "INCONCLUSIVE_STALE_OR_INVALID_EVIDENCE")
    points = meta.get("control_points_m")
    if (not isinstance(points, list) or not points or
            meta.get("search_failure") is None):
        return _inconclusive(directory,
                             "INCONCLUSIVE_MISSING_SEGMENT_CONTEXT")
    if len(points) > 100000:
        raise ValueError("too many control points in failure snapshot")
    try:
        lines = [" ".join(str(int(x)) for x in meta["dimensions"]),
                 _point(meta["origin_m"]), _point(meta["max_boundary_m"]),
                 " ".join((_number(meta["resolution_m"]),
                           motion_values[1], str(int(meta["generation"])),
                           json.dumps(meta["frame_id"]), motion_values[0])),
                 " ".join((str(int(meta["motion_quality"])),
                           str(int(bool(meta["motion_allow_bridged"]))),
                           *motion_values[2:8], motion_values[8])),
                 " ".join((*map(str, meta["search_pool_dimensions"]),
                           _number(meta["search_step_size_m"]))),
                 _point(meta["search_pool_center_m"]),
                 _point(meta["search_requested_start_m"]),
                 _point(meta["search_requested_end_m"]),
                 " ".join((str(int(meta["segment_start_index"])),
                           str(int(meta["segment_end_index"])),
                           str(len(points))))]
        lines.extend(_point(point) for point in points)
        lines.append(f"{meta['search_failure']} {_number(budget_s)}")
    except (KeyError, TypeError, ValueError):
        return _inconclusive(directory,
                             "INCONCLUSIVE_MISSING_SEGMENT_CONTEXT")
    binary = Path(backend) if backend else _backend_path()
    try:
        completed = subprocess.run(
            [str(binary), str(directory / meta["cell_flags_file"])],
            input="\n".join(lines) + "\n", text=True, capture_output=True,
            check=False, timeout=max(10.0, budget_s + 30.0))
    except subprocess.TimeoutExpired:
        return _inconclusive(directory, "INCONCLUSIVE_OFFLINE_BUDGET")
    if completed.returncode:
        raise RuntimeError(completed.stderr.strip() or "C++ replay failed")
    report = json.loads(completed.stdout)
    report.update(snapshot=str(directory), generation=meta["generation"],
                  online_failure=meta["search_failure"],
                  search_stage=meta.get("search_stage"),
                  required_clearance_m=meta.get("required_clearance_m"),
                  scope="original A* pool; observed physical map and saved motion",
                  advisory="saved queried PL samples are diagnostic only")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--budget-s", type=float, default=120.0)
    args = parser.parse_args()
    report = inspect(args.snapshot, args.budget_s)
    print(json.dumps(report, indent=2))
    directory = args.snapshot.resolve()
    if directory.parents[2].name == "export":
        run = directory.parents[3]
        destination = run / "export" / "analysis" / (
            "failure_map_" + directory.name + ".json")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
