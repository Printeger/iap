#!/usr/bin/env python3
"""Create one immutable index over retained ICRA-072A Layer-1 attempts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
DEV_RUNS_ROOT = (REPOSITORY / "results/icra27/dev_runs/layer1").resolve()


def _json(path: Path) -> dict:
    return json.loads(path.read_text())


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def classify_iteration(run_root: Path) -> dict:
    manifest_path = run_root / "run_manifest.json"
    analysis_path = run_root / "analysis.json"
    manifest = _json(manifest_path)
    analysis = _json(analysis_path) if analysis_path.is_file() else {}
    launch_paths = sorted((run_root / "exports").glob(
        "*/test_planner_manifest.json"))
    launch = _json(launch_paths[0]) if len(launch_paths) == 1 else {}

    if manifest.get("result") == "CAPTURE_NOT_READY":
        classification = "CAPTURE_NOT_READY"
        first_missing_stage = "p0_snapshot"
    elif analysis.get("first_missing_stage") is not None:
        classification = "FAILED_STAGE"
        first_missing_stage = analysis.get("first_missing_stage")
    elif manifest.get("owned_process_groups_cleared") is False:
        classification = "REJECTED_RUNNER_CLEANUP"
        first_missing_stage = "runner_cleanup"
    elif launch.get("p5.current_pl_source") == "LIDAR_CERTIFIED":
        classification = "REJECTED_P5_AUTHORITY_BYPASS"
        first_missing_stage = "p5_final_pass_before_publish"
    elif analysis.get("result") == "PASS":
        classification = "PASS"
        first_missing_stage = analysis.get("first_missing_stage")
    else:
        classification = "FAILED_STAGE"
        first_missing_stage = analysis.get("first_missing_stage")
        if first_missing_stage is None:
            first_missing_stage = "p0_snapshot"

    return {
        "run_id": run_root.name,
        "classification": classification,
        "first_missing_stage": first_missing_stage,
        "original_manifest_result": manifest.get("result"),
        "original_analysis_result": analysis.get("result"),
        "original_analysis_first_missing_stage": analysis.get(
            "first_missing_stage"),
        "bindings": {
            "run_manifest_sha256": _sha256(manifest_path),
            "analysis_sha256": _sha256(analysis_path),
            "launch_manifest_sha256": (
                _sha256(launch_paths[0]) if len(launch_paths) == 1 else None),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--through-run", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    runs_root = args.runs_root.resolve()
    output = args.output.resolve()
    if runs_root != DEV_RUNS_ROOT:
        raise SystemExit(f"runs root must be {DEV_RUNS_ROOT}")
    if output.parent != DEV_RUNS_ROOT:
        raise SystemExit("iteration index output must be directly under runs root")
    if output.exists():
        raise SystemExit("iteration index output already exists")
    if args.through_run < 1:
        raise SystemExit("through-run must be positive")

    iterations = []
    for number in range(1, args.through_run + 1):
        run_root = runs_root / f"run-{number:03d}"
        if not run_root.is_dir() or run_root.is_symlink():
            raise SystemExit(f"retained run missing or invalid: {run_root.name}")
        iterations.append(classify_iteration(run_root))

    payload = {
        "schema_version": "icra072_layer1_iteration_index_v2",
        "development_only": True,
        "effect_claim": False,
        "through_run": f"run-{args.through_run:03d}",
        "iterations": iterations,
    }
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
