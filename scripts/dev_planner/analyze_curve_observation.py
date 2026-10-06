#!/usr/bin/env python3
"""Locate the saved actual curve's first unknown voxel and explain its evidence.

The unthinned mask was computed by RegisteredLidarWindow's own traversal,
outside the map lock. It is diagnostic and cannot authorize execution.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from replay_failure_map import load_snapshot


def _curve_point(curve, time_s):
    points = np.asarray(curve["control_points_m"], dtype=float)
    knots = np.asarray(curve["knots_s"], dtype=float)
    degree = int(curve["degree"])
    if (points.ndim != 2 or points.shape[1] != 3 or degree < 1 or
            len(knots) != len(points) + degree + 1 or
            not np.isfinite(points).all() or not np.isfinite(knots).all() or
            np.any(np.diff(knots) < 0)):
        raise ValueError("invalid saved curve")
    u = np.clip(time_s + knots[degree], knots[degree], knots[len(points)])
    span = min(len(points) - 1,
               max(degree, int(np.searchsorted(knots, u, side="left")) - 1))
    values = points[span - degree:span + 1].copy()
    for level in range(1, degree + 1):
        for j in range(degree, level - 1, -1):
            a = span - degree + j
            width = knots[a + degree + 1 - level] - knots[a]
            if width <= 0:
                raise ValueError("degenerate saved knot interval")
            alpha = (u - knots[a]) / width
            values[j] = (1 - alpha) * values[j - 1] + alpha * values[j]
    return values[degree]


def inspect(directory):
    directory, meta, cells = load_snapshot(directory)
    report = {"schema_version": "iap_curve_observation_v1",
              "snapshot": str(directory), "generation": meta["generation"],
              "classification": "INCONCLUSIVE_MISSING_CURVE_AND_FRAME"}
    curve = meta.get("actual_curve")
    first = meta.get("first_unobserved_position_m")
    if not curve or first is None:
        return report
    origin = np.asarray(meta["origin_m"], dtype=float)
    resolution = float(meta["resolution_m"])
    saved_index = tuple(meta["first_unobserved_voxel_index"])
    # Match GridMap's multiply-by-resolution_inv, including boundary rounding.
    index = tuple(int(v) for v in np.floor(
        (np.asarray(first) - origin) * (1.0 / resolution)))
    time_s = float(meta["first_unobserved_time_s"])
    reconstructed = _curve_point(curve, time_s)
    if (index != saved_index or any(i < 0 or i >= d for i, d in zip(index, cells.shape)) or
            not np.allclose(reconstructed, first, atol=1e-8, rtol=0)):
        raise ValueError("saved unknown point, voxel and actual curve disagree")
    start = float(meta["curve_checked_from_time_s"])
    end = float(meta["curve_checked_to_time_s"])
    step = float(meta["curve_sample_step_s"])
    if not all(math.isfinite(v) for v in (start, end, step)) or step <= 0 or end < start:
        raise ValueError("invalid curve assessment interval")
    # Reproduce the assessment schedule, including an earlier physical failure
    # which must not conceal a subsequent unknown voxel.
    found = None
    intervals = math.ceil((end - start) / step)
    if intervals > 1000000:
        report["classification"] = "INCONCLUSIVE_REPLAY_SAMPLE_LIMIT"
        return report
    for sample in range(intervals + 1):
        t = min(end, start + sample * step)
        p = _curve_point(curve, t)
        idx = tuple(np.floor((p - origin) * (1.0 / resolution)).astype(int))
        if all(0 <= i < d for i, d in zip(idx, cells.shape)) and not (int(cells[idx]) & 7):
            found = (t, idx)
            break
    if found is None or found[1] != index or abs(found[0] - time_s) > 1e-8:
        raise ValueError("saved first unknown point disagrees with same-mask curve replay")
    report.update(first_unobserved_time_s=time_s, first_unobserved_position_m=first,
                  voxel_index=list(index), voxel_center_m=
                  (origin + (np.asarray(index) + 0.5) * resolution).tolist(),
                  resolution_m=resolution, cloud_stamp_s=meta["cloud_stamp_s"],
                  cell_flags=int(cells[index]), curve_replay_matches=True)
    if not meta.get("observation_evidence_available") or not meta.get("current_frame"):
        report["classification"] = "INCONCLUSIVE_MISSING_OBSERVATION_PROVENANCE"
        return report
    sources = np.fromfile(directory / meta["observation_sources_file"], dtype=np.uint8)
    if sources.size != cells.size:
        raise ValueError("observation source mask dimensions disagree")
    # Require the registered raw evidence artifacts as well as the derived mask.
    frame = meta["current_frame"]
    evaluation_time = float(meta.get("curve_evaluation_time_s", meta["planning_time_s"]))
    frame_age = evaluation_time - float(frame.get("scan_end_stamp_s", frame["stamp_s"]))
    map_age = evaluation_time - float(meta["cloud_stamp_s"])
    max_age = float(meta["environment_max_age_s"])
    report.update(current_frame_age_s=frame_age, map_age_s=map_age)
    if (not all(math.isfinite(v) for v in (frame_age, map_age, max_age)) or
            min(frame_age, map_age) < 0 or max(frame_age, map_age) > max_age):
        report["classification"] = "INCONCLUSIVE_STALE_OBSERVATION_EVIDENCE"
        return report
    for field in ("hits_file", "beams_file"):
        if not (directory / frame[field]).is_file():
            raise ValueError("current frame evidence file missing")
    flags = int(sources.reshape(cells.shape)[index])
    loss = (flags & 48) >> 4
    report.update(current_frame_id=frame["frame_id"], current_frame_stamp_s=frame["stamp_s"],
                  beam_evidence_complete=frame["beam_evidence_complete"],
                  current_hit=bool(flags & 1), current_free=bool(flags & 2),
                  active_hit=bool(flags & 4), active_free=bool(flags & 8),
                  unthinned_current_observed=bool(flags & 128),
                  last_observation_loss_producer=
                  {0: None, 1: "current_replace", 2: "active_delta", 3: "active_replace"}[loss])
    if flags & 15:
        report["classification"] = "OBSERVATION_MASK_INCONSISTENT"
    elif flags & 128:
        report["classification"] = "ENDPOINT_DEDUPLICATION_GAP"
    elif loss:
        report["classification"] = {
            1: "OBSERVATION_REMOVED_CURRENT_REPLACE",
            2: "OBSERVATION_REMOVED_ACTIVE_DELTA",
            3: "OBSERVATION_REMOVED_ACTIVE_REPLACE"}[loss]
    else:
        report["classification"] = "CURRENT_RAY_COVERAGE_GAP"
    report["scope"] = ("saved actual-curve sample and same-generation registered evidence; "
                       "no-return coverage requires complete beam evidence; "
                       "loss producer is historical, not a timestamp or global no-route claim")
    report["neighbor_voxels"] = [
        {"index": [x, y, z], "cell_flags": int(cells[x, y, z])}
        for x in range(max(0, index[0] - 1), min(cells.shape[0], index[0] + 2))
        for y in range(max(0, index[1] - 1), min(cells.shape[1], index[1] + 2))
        for z in range(max(0, index[2] - 1), min(cells.shape[2], index[2] + 2))]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    args = parser.parse_args()
    report = inspect(args.snapshot)
    print(json.dumps(report, indent=2))
    directory = args.snapshot.resolve()
    if directory.parents[2].name == "export":
        destination = directory.parents[3] / "export/analysis" / (
            "curve_observation_" + directory.name + ".json")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
