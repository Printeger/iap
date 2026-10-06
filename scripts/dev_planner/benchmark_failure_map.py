#!/usr/bin/env python3
"""Warm then repeat the same frozen physical A*; no real advisory prediction."""

import argparse
import hashlib
import json
import math
import os
import re
from pathlib import Path
import statistics
import subprocess
import sys

from analyze_failure_map import inspect, _backend_path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "launch" / "_includes"))
from run_directory import (resolve_run_directory, adopt_run_directory, finalize_run,
                           write_subordinate_manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--label", required=True)
    parser.add_argument("--repeats", type=int, default=7)
    parser.add_argument("--budget-s", type=float, default=120.0)
    parser.add_argument("--no-diagnostics", action="store_true")
    parser.add_argument("--differential", action="store_true")
    parser.add_argument("--full-epoch", action="store_true", help="exercise the current shared FrozenOccupancyEpoch path")
    args = parser.parse_args()
    if args.repeats < 3:
        parser.error("at least three measured repetitions required")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.label):
        parser.error("label must be a safe artifact component")
    inherited = os.environ.get("IAP_RUN_DIR")
    owner = not inherited
    run = (resolve_run_directory(entrypoint="search_benchmark") if owner
           else adopt_run_directory(inherited))
    os.environ["IAP_RUN_DIR"] = str(run)
    os.environ["ROS_LOG_DIR"] = str(run / "runtime" / "ros")
    try:
        binary = _backend_path()
        report = inspect(args.snapshot, args.budget_s, binary, args.repeats, not args.no_diagnostics, args.differential, args.full_epoch)
        report.update(label=args.label, differential=args.differential, full_epoch=args.full_epoch, diagnostics=not args.no_diagnostics, warmup_rounds=1,
                      budget_s=args.budget_s,
                      backend_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                      snapshot_sha256={name: hashlib.sha256(
                          (args.snapshot / name).read_bytes()).hexdigest()
                          for name in ("snapshot.json", "cells.bin")},
                      revision=subprocess.check_output(
                          ["git", "rev-parse", "HEAD"], text=True).strip(),
                      timing="input file decode excluded; freeze includes reconstructed map copy/index and optional complete epoch capture; init includes pool allocation; search includes result logging; backend/final check not run",
                      cache_memory="estimated payload/key/links/buckets, excludes allocator; sizes reported in hit/miss counters: voxel/lattice/midpoint; after consolidation entry/byte slot 0 contains the shared cache",
                      advisory="physical replay only; zero real predictor calls")
        saved = json.loads((args.snapshot / "snapshot.json").read_text())
        report["fixed_input"] = {key: value for key, value in saved.items()
                                 if key.startswith("motion_") or key in (
                                     "frame_id", "resolution_m", "dimensions", "origin_m",
                                     "planning_time_s", "environment_max_age_s",
                                     "search_step_size_m", "search_pool_dimensions",
                                     "search_pool_center_m", "search_requested_start_m",
                                     "search_requested_end_m")}
        flags = binary.parent / "CMakeFiles" / "failure_map_replay.dir" / "flags.make"
        if flags.is_file():
            report["build_flags"] = flags.read_text()
        report["linked_library_sha256"] = {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            for name in ("plan_env", "path_searching")
            if (path := binary.parent.parent / name / ("lib" + name + ".so")).is_file()}
        report["summary"] = {}
        for key in ("freeze_s", "init_s", "search_s", "total_s", "clearance_s",
                    "query_management_s", "edge_s"):
            values = sorted(row[key] for row in report["samples"])
            report["summary"][key] = {"median": statistics.median(values),
                                      "p95_nearest_rank": values[math.ceil(0.95 * len(values))-1],
                                      "max": max(values)}
        name = "search_benchmark" if owner else "search_benchmark_" + args.label
        destination = run / "export" / "analysis" / (name + ".json")
        if destination.exists():
            raise FileExistsError(f"benchmark evidence already exists: {destination}")
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(report, indent=2) + "\n")
        manifest = {
            "schema_version": "iap_search_benchmark_manifest_v1",
            "label": args.label, "report": str(destination.relative_to(run)),
            "snapshot": str(args.snapshot.resolve()), "revision": report["revision"]}
        if owner:
            write_subordinate_manifest(run, name, manifest)
            finalize_run(run, lifecycle="completed", safety_outcome="not_applicable")
        else:
            # Adopters publish an immutable child manifest; only the outer
            # owner may change the primary manifest or finalize its run.
            path = run / "metadata" / "manifests" / (name + ".json")
            with path.open("x") as stream:
                stream.write(json.dumps(manifest, indent=2) + "\n")
        print(destination)
        print(json.dumps(report["summary"], indent=2))
    except Exception:
        if owner:
            finalize_run(run, lifecycle="failed", safety_outcome="unknown")
        raise


if __name__ == "__main__":
    main()
