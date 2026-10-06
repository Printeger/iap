#!/usr/bin/env python3
"""Compare receiver-local replay PL with recorded truth using a fixed contract.

Truth is read only. No alignment is fitted here. Invalid and temporally
unmatched requests remain in the CSV denominator; runs are independent units.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import numpy as np
from advisory_validation import (adopt_run_directory, artifact, json_write,
                                 manifest, numeric, rows, sha)


def rigid(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all() or
            not np.allclose(t[3], [0, 0, 0, 1], atol=1e-12, rtol=0) or
            not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-9, rtol=0) or
            abs(np.linalg.det(t[:3, :3]) - 1) > 1e-9):
        raise ValueError("fixed transform must be rigid SE(3)")
    return t


def quaternion(row):
    q = np.array([float(row[k]) for k in ("qx", "qy", "qz", "qw")])
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12:
        raise ValueError("invalid_truth_orientation")
    return q / norm


def rotation_matrix(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def interpolate_truth(samples, stamp, max_dt=.05):
    """No extrapolation; both endpoints must be within the declared tolerance."""
    samples = sorted(samples, key=lambda r: float(r["stamp"]))
    times = np.asarray([float(r["stamp"]) for r in samples])
    if not np.isfinite(times).all() or not math.isfinite(stamp):
        raise ValueError("invalid_truth_timestamp")
    i = np.searchsorted(times, stamp)
    if i < len(times) and times[i] == stamp:
        pair = [samples[i]]
    elif i == 0 or i == len(times):
        raise ValueError("truth_time_unmatched")
    else:
        pair = samples[i-1:i+1]
        if max(abs(float(r["stamp"])-stamp) for r in pair) > max_dt:
            raise ValueError("truth_time_gap")
    positions = np.asarray([[float(r[k]) for k in ("x", "y", "z")] for r in pair])
    if not np.isfinite(positions).all(): raise ValueError("invalid_truth_position")
    q0 = quaternion(pair[0])
    if len(pair) == 1:
        return positions[0], rotation_matrix(q0)
    t0, t1 = (float(r["stamp"]) for r in pair)
    u = (stamp-t0)/(t1-t0)
    q1 = quaternion(pair[1]); dot = float(q0 @ q1)
    if dot < 0: q1 = -q1; dot = -dot
    if dot > .9995:
        q = (1-u)*q0 + u*q1; q /= np.linalg.norm(q)
    else:
        theta = math.acos(max(-1., min(1., dot)))
        q = (math.sin((1-u)*theta)*q0+math.sin(u*theta)*q1)/math.sin(theta)
    return positions[0]*(1-u)+positions[1]*u, rotation_matrix(q)


def errors(position, truth_position, truth_rotation, contract):
    world = rigid(contract["T_truth_map"])
    body = rigid(contract["T_truthbody_predictionbody"])
    estimate = world[:3, :3] @ np.asarray(position) + world[:3, 3]
    truth = np.asarray(truth_position) + truth_rotation @ body[:3, 3]
    d = estimate - truth
    if not np.isfinite(d).all(): raise ValueError("invalid_position_error")
    return math.hypot(d[0], d[1]), abs(d[2])


def compare(run, contract_path=None):
    contract = json.loads(contract_path.read_text()) if contract_path else None
    if contract:
        if contract.get("alignment_policy") not in ("known_fixed_transform", "single_initial_sim_se3"):
            raise ValueError("a known fixed or single initial transform is required; fitting is forbidden")
        if not contract.get("provenance"):
            raise ValueError("coordinate/extrinsic provenance required")
        if contract["alignment_policy"] == "single_initial_sim_se3" and "initial_alignment_stamp_s" not in contract:
            raise ValueError("initial simulation alignment identity required")
        for k in ("T_truth_map", "T_truthbody_predictionbody"):
            rigid(contract[k])
    odom = [r for p in sorted((run / "export/advisory/validation/recordings").glob("*_odometry.csv")) for r in rows(p)]
    truth = [r for r in odom if r["source"] == "truth"]
    result = []
    for p in sorted((run / "export/advisory/validation").glob("*_current/points.csv")):
        meta = json.loads(p.with_name("input.json").read_text())
        for row in rows(p):
            if row["identity"] != "REAL_REPLAY":
                continue
            item = {"run_id": run.name, "identity": "LIVE_MEASUREMENT", "input": str(p.relative_to(run)),
                    "reference_time_s": row["reference_time_s"], "pose_stamp_s": row["pose_stamp_s"],
                    "reference_pose_delta_s": numeric(row, "reference_time_s") - numeric(row, "pose_stamp_s"),
                    "valid": False, "reason": "", "error_h": "", "error_v": "", "hpl": row["fused_hpl"], "vpl": row["fused_vpl"]}
            try:
                if row["valid"] != "1": raise ValueError("prediction_unavailable:" + row["reason"])
                if contract is None: raise ValueError("coordinate_contract_missing")
                if abs(item["reference_pose_delta_s"]) > .05: raise ValueError("reference_pose_not_same_time")
                if meta["frame_id"] != contract["prediction_frame"]: raise ValueError("prediction_frame_mismatch")
                glio = [r for r in odom if r["source"] == "glio" and
                        r["frame"] == contract["prediction_frame"] and r["body"] == contract["prediction_body"] and
                        abs(float(r["stamp"])-float(row["pose_stamp_s"])) <= 1e-6]
                if not glio: raise ValueError("glio_pose_identity_unmatched")
                if not any(np.allclose([float(r[k]) for k in ("x", "y", "z")], meta["position"], atol=1e-8, rtol=0) for r in glio):
                    raise ValueError("glio_snapshot_position_mismatch")
                selected = [r for r in truth if r["frame"] == contract["truth_frame"] and r["body"] == contract["truth_body"]]
                if not selected: raise ValueError("truth_frame_or_body_mismatch")
                position, rotation = interpolate_truth(selected, float(row["pose_stamp_s"]))
                # Snapshot pose is the exact predicted receiver position. No
                # GLIO re-interpolation or piecewise registration hides its error.
                eh, ev = errors(meta["position"], position, rotation, contract)
                item.update(valid=True, error_h=eh, error_v=ev)
            except ValueError as e:
                item["reason"] = str(e)
            result.append(item)
    path = artifact(run, "export/advisory/validation/error_requests.csv")
    fields = ["run_id", "identity", "input", "reference_time_s", "pose_stamp_s", "reference_pose_delta_s", "valid", "reason", "error_h", "error_v", "hpl", "vpl"]
    with path.open("x") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(result)
    good = [r for r in result if r["valid"]]
    summary = {"identity": "LIVE_MEASUREMENT", "requested": len(result), "valid": len(good),
               "independent_runs": len({r["run_id"] for r in good}),
               "status": "INCONCLUSIVE_LIVE_NOT_RUN" if not result else "INCONCLUSIVE_TOO_FEW_RUNS",
               "exceed_h": sum(r["error_h"] > float(r["hpl"]) for r in good),
               "exceed_v": sum(r["error_v"] > float(r["vpl"]) for r in good),
               "run_blocks": [{"run_id": run.name, "valid": len(good),
                               "p95_error_h": float(np.quantile([r["error_h"] for r in good], .95)) if good else None,
                               "p95_error_v": float(np.quantile([r["error_v"] for r in good], .95)) if good else None}],
               "confidence_intervals": None, "all_map_coverage": False,
               "time_policy": "truth interpolation at saved pose stamp; both ends within 0.05 s; reference-pose lag <=0.05 s"}
    target = artifact(run, "export/analysis/advisory_validation/error_summary.json")
    json_write(target, summary)
    manifest(run, "advisory_errors", {"coordinate_contract": str(contract_path) if contract_path else None,
             "coordinate_contract_sha256": sha(contract_path) if contract_path else None,
             "artifacts_sha256": {str(p.relative_to(run)): sha(p) for p in (path, target)}})
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alignment", type=Path, help="fixed coordinate/extrinsic JSON with provenance")
    args = parser.parse_args()
    if not os.environ.get("IAP_RUN_DIR"): parser.error("adopt the existing IAP_RUN_DIR")
    print(json.dumps(compare(adopt_run_directory(os.environ["IAP_RUN_DIR"]), args.alignment), indent=2))
