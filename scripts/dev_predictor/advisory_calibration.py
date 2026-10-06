#!/usr/bin/env python3
"""Offline staged calibration: independent runs, fixed route/frame, joint H/V coverage.

No truth reaches a predictor. No calibration result promotes production defaults.
"""
import argparse
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import numpy as np
from advisory_validation import adopt_run_directory, artifact, json_write, manifest, rows, sha

CONDITIONS = ("normal", "gnss_degraded", "lidar_degraded")
CAL_SEEDS = (1101, 1102, 1103)
VALIDATION_SEEDS = (2101, 2102, 2103)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def protocol():
    return {"schema": "iap_advisory_calibration_protocol_v1", "scene": "icra_dense_forest_four_fork_v2",
            "map_seed": 41021, "posterior_prior": False, "calibration_guidance": False,
            "joint_coverage_target": .95, "reference_route": None,
            "route_status": "PENDING_PHYSICAL_CHECK", "coordinates_status": "PENDING_VERIFICATION",
            "degradation_schedule": None, "observation_seed_injection_status": "PENDING_LIVE_INTEGRATION",
            "calibration": [{"condition": c, "seed": s} for c in CONDITIONS for s in CAL_SEEDS],
            "validation": [{"condition": c, "seed": s} for c in CONDITIONS for s in VALIDATION_SEEDS],
            "mission": [{"condition": "paired", "seed": s, "guidance": g}
                        for s in VALIDATION_SEEDS for g in (False, True)]}


def checked_dataset(path, phase, parameter_hash=None):
    data = json.loads(path.read_text())
    if data.get("identity") != "LIVE_MEASUREMENT":
        raise ValueError("independent real measurement identity required")
    for key in ("route_sha256", "coordinates_sha256", "degradation_schedule_sha256"):
        if not data.get(key): raise ValueError("fixed contract missing: " + key)
    if data.get("map_seed") != 41021 or data.get("posterior_prior") is not False:
        raise ValueError("canonical map and disabled posterior required")
    if data.get("guidance") is not False or data.get("frame_verified") is not True:
        raise ValueError("calibration requires unguided physically checked route and verified frame")
    allowed = CAL_SEEDS if phase == "calibration" else VALIDATION_SEEDS
    trials = data.get("trials", [])
    pairs = [(t["condition"], t["seed"]) for t in trials]
    required = {(c, s) for c in CONDITIONS for s in allowed}
    if len(pairs) != len(set(pairs)) or set(pairs) != required:
        raise ValueError("three independent predeclared seeds per condition required")
    ids = [t["run_id"] for t in trials]
    if len(ids) != len(set(ids)): raise ValueError("a run cannot count as several trials")
    if parameter_hash and data.get("parameter_sha256") != parameter_hash:
        raise ValueError("replay parameters differ from frozen calibration")
    csv_paths, csv_hashes = set(), set()
    revisions = set()
    for t in trials:
        if t.get("phase") != phase: raise ValueError("held-out/calibration reuse")
        p = (path.parent / t["csv"]).resolve()
        if sha(p) != t["sha256"]: raise ValueError("trial checksum mismatch")
        if p in csv_paths or t["sha256"] in csv_hashes:
            raise ValueError("duplicate trial evidence")
        csv_paths.add(p); csv_hashes.add(t["sha256"])
        run_path=(path.parent/t["run_manifest"]).resolve()
        if sha(run_path)!=t["run_manifest_sha256"]: raise ValueError("run manifest checksum mismatch")
        run=json.loads(run_path.read_text())
        source=run.get("source",{})
        if (run.get("schema_version")!="iap_run_artifact_v1" or run.get("run_id")!=t["run_id"] or
                run.get("entrypoint")!="iap_sim" or run.get("scenario")!="icra_dense_forest_four_fork_v2" or
                run.get("lifecycle")!="completed" or source.get("git_worktree_clean") is not True or
                not source.get("git_commit")):
            raise ValueError("real completed clean canonical run required")
        trial=run.get("validation_trial",{})
        for key,value in {"condition":t["condition"],"seed":t["seed"],"phase":phase,
                          "map_seed":41021,"route_sha256":data["route_sha256"],
                          "coordinates_sha256":data["coordinates_sha256"],
                          "degradation_schedule_sha256":data["degradation_schedule_sha256"]}.items():
            if trial.get(key)!=value: raise ValueError("recorded trial identity mismatch: "+key)
        if parameter_hash and trial.get("parameter_sha256")!=parameter_hash:
            raise ValueError("recorded trial parameter mismatch")
        revisions.add(source["git_commit"])
        request_path=(path.parent/t["requests_manifest"]).resolve()
        if sha(request_path)!=t["requests_manifest_sha256"]: raise ValueError("request manifest checksum mismatch")
        recorded=json.loads(request_path.read_text())
        if recorded.get("run_id")!=t["run_id"] or recorded.get("identity")!="LIVE_MEASUREMENT":
            raise ValueError("request recording identity mismatch")
        request_ids=recorded.get("request_ids",[])
        if not request_ids or len(request_ids)!=len(set(request_ids)):
            raise ValueError("complete unique recorded request list required")
        table = rows(p)
        if len(table)!=len(request_ids) or {r.get("request_id") for r in table}!=set(request_ids):
            raise ValueError("missing or duplicate recorded request")
        if any(r.get("run_id")!=t["run_id"] for r in table):
            raise ValueError("CSV run identity mismatch")
        if any(r.get("identity") != "LIVE_MEASUREMENT" for r in table):
            raise ValueError("synthetic or historical rows cannot calibrate real error")
        t["rows"] = table
    if len(revisions)!=1: raise ValueError("mixed source revisions within dataset")
    data["source_revisions"]=sorted(revisions)
    return data


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0: raise ValueError("positive finite prediction/noise required")
    return number


def usable(table):
    good = []
    reasons = collections.Counter()
    for row in table:
        try:
            if str(row.get("valid")).lower() not in ("1", "true"):
                raise ValueError(row.get("reason") or "prediction_unavailable")
            h, v = positive(row["hpl"]), positive(row["vpl"])
            eh, ev = float(row["error_h"]), float(row["error_v"])
            stamp = float(row["reference_time_s"])
            if not all(math.isfinite(x) for x in (eh, ev, stamp)) or min(eh, ev) < 0:
                raise ValueError("invalid_error_or_time")
            good.append((stamp, h, v, eh, ev))
        except (ValueError, KeyError) as e:
            reasons[str(e)] += 1
    return good, dict(reasons)


def fit_noise(data):
    """Calibration-only normalized residual RMS, weighted equally by run.

    Residuals must come from actual measurement support, not truth position errors.
    Missing residuals are a blocker, not permission to fit a scale to legacy PL.
    """
    scales = {}
    for source in ("gnss", "lidar"):
        per_run = []
        for t in data["trials"]:
            values = [float(r[source + "_residual_m"]) / positive(r[source + "_nominal_sigma_m"])
                      for r in t["rows"] if r.get(source + "_residual_m", "")]
            if not values or not np.isfinite(values).all():
                raise ValueError("measurement residuals missing: " + source + "/" + t["run_id"])
            per_run.append(float(np.mean(np.square(values))))
        scales[source] = positive(math.sqrt(float(np.mean(per_run))))
    return {"risk/gnss_noise_scale": scales["gnss"],
            "risk/lidar_noise_scale": scales["lidar"], "risk/K_H_adv": 5., "risk/K_V_adv": 5.}


def contract(data):
    return {k: data[k] for k in ("route_sha256", "coordinates_sha256", "degradation_schedule_sha256", "map_seed")}


def fit_conversion(data, noise):
    if data["parameter_sha256"] != noise["sha256"] or contract(data) != noise["contract"]:
        raise ValueError("noise-scaled replay and unchanged fixed contract required")
    ratios = []
    for trial in data["trials"]:
        good, reasons = usable(trial["rows"])
        if reasons or not good: raise ValueError("calibration coverage incomplete: " + trial["run_id"])
        ratios.append(float(np.quantile([max(r[3]/r[1], r[4]/r[2]) for r in good], .95, method="higher")))
    scale = positive(max(ratios))
    params = dict(noise["parameters"])
    params["risk/K_H_adv"] *= scale; params["risk/K_V_adv"] *= scale
    return {"identity": "LIVE_CALIBRATION_CANDIDATE", "stage": "conversion", "scene": "icra_dense_forest_four_fork_v2",
            "parameters": params, "contract": contract(data), "calibration_seeds": list(CAL_SEEDS),
            "calibration_runs": [t["run_id"] for t in data["trials"]],
            "source_revisions":data.get("source_revisions",[]),
            "calibration_evidence":[{k:v for k,v in t.items() if k!="rows"} for t in data["trials"]], "method": "max per-run joint p95 ratio"}


def correlation(a, b):
    if len(a) < 5 or np.ptp(a) <= 1e-9 or np.ptp(b) <= 1e-9: return None
    def ranks(v):
        return np.asarray([np.count_nonzero(v < x) + (np.count_nonzero(v == x)-1)/2 for x in v])
    return float(np.corrcoef(ranks(np.asarray(a)), ranks(np.asarray(b)))[0, 1])


def evaluate(data, frozen):
    expected = frozen["sha256"]
    unsigned = {k: v for k, v in frozen.items() if k != "sha256"}
    if digest(unsigned) != expected: raise ValueError("frozen parameters altered")
    if contract(data) != frozen["contract"]: raise ValueError("validation route/frame/schedule changed")
    results = []
    for trial in data["trials"]:
        if trial["run_id"] in frozen["calibration_runs"]: raise ValueError("held-out run reused")
        good, reasons = usable(trial["rows"])
        covered = sum(r[3] <= r[1] and r[4] <= r[2] for r in good)
        # Invalid/unmatched requests remain denominator; no interpolation into low risk.
        denominator = len(trial["rows"])
        coverage = covered / denominator if denominator else None
        blocks = collections.defaultdict(list)
        origin = min((r[0] for r in good), default=0)
        for row in good: blocks[int((row[0]-origin)//5)].append(row)
        summaries = [np.mean(v, axis=0) for v in blocks.values()]
        trends, intervals = {}, {}
        rng = np.random.default_rng(20261006)
        for metric, p, e in (("horizontal", 1, 3), ("vertical", 2, 4)):
            x = np.asarray([v[p] for v in summaries]); y = np.asarray([v[e] for v in summaries])
            rho = correlation(x, y); trends[metric] = rho
            boot = []
            if rho is not None:
                for _ in range(500):
                    i = rng.integers(0, len(x), len(x)); value = correlation(x[i], y[i])
                    if value is not None: boot.append(value)
            intervals[metric] = np.quantile(boot, [.025, .975]).tolist() if boot else None
        trend_status = ("INCONCLUSIVE_LOW_VARIATION_OR_BLOCKS" if any(v is None for v in trends.values()) else
                        "PASS" if all(v > 0 for v in trends.values()) and all(v[0] > 0 for v in intervals.values()) else
                        "INCONCLUSIVE_TREND_UNCERTAINTY" if all(v > 0 for v in trends.values()) else "FAIL")
        results.append({"run_id": trial["run_id"], "condition": trial["condition"], "seed": trial["seed"],
                        "requests": denominator, "valid": len(good), "failure_reasons": reasons,
                        "joint_coverage": coverage, "coverage_status": "PASS" if coverage is not None and coverage >= .95 else "FAIL",
                        "blocks": len(blocks), "trend": trends, "block_bootstrap_95": intervals, "trend_status": trend_status})
    return {"identity": "LIVE_MEASUREMENT", "independent_runs": len(results), "runs": results,
            "coverage_status": "PASS" if all(r["coverage_status"] == "PASS" for r in results) else "FAIL",
            "trend_status": "PASS" if all(r["trend_status"] == "PASS" for r in results) else "INCONCLUSIVE",
            "integrity_guarantee": False, "future_error_guarantee": False, "default_promotion": False}


def frozen(value):
    return {**value, "sha256": digest(value)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("protocol", "noise", "conversion", "validate"))
    parser.add_argument("--dataset", type=Path); parser.add_argument("--parameters", type=Path)
    parser.add_argument("--label", default="calibration")
    args = parser.parse_args()
    run = adopt_run_directory(os.environ["IAP_RUN_DIR"])
    if args.mode == "protocol": result = protocol()
    else:
        if args.dataset is None: parser.error("--dataset required")
        params = json.loads(args.parameters.read_text()) if args.parameters else None
        if params and digest({k:v for k,v in params.items() if k != "sha256"}) != params.get("sha256"):
            raise ValueError("parameter checksum mismatch")
        phase = "validation" if args.mode == "validate" else "calibration"
        data = checked_dataset(args.dataset, phase, params["sha256"] if params else None)
        if args.mode == "noise": result = frozen({"identity":"LIVE_CALIBRATION_CANDIDATE", "scene":"icra_dense_forest_four_fork_v2",
                "parameters": fit_noise(data), "contract": contract(data), "stage": "noise",
                "source_revisions":data["source_revisions"],
                "calibration_evidence":[{k:v for k,v in t.items() if k!="rows"} for t in data["trials"]]})
        elif params is None: parser.error("--parameters required")
        elif args.mode == "conversion": result = frozen(fit_conversion(data, params))
        else: result = evaluate(data, params)
    from advisory_validation import safe_label
    target = artifact(run, "export/analysis/advisory_validation/" + safe_label(args.label) + "/" + args.mode + ".json")
    json_write(target, result)
    manifest(run, "advisory_calibration_" + args.mode + "_" + safe_label(args.label),
             {"command": __import__("sys").argv, "dataset_sha256": sha(args.dataset) if args.dataset else None,
              "parameters_sha256": sha(args.parameters) if args.parameters else None,
              "artifacts_sha256": {str(target.relative_to(run)): sha(target)}})
    print(target)


if __name__ == "__main__": main()
