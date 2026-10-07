#!/usr/bin/env python3
"""Replay a captured production Curve against its original frozen physical time."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "scripts/dev_predictor"))
from advisory_validation import source_identity, binary_identity, sha
from run_directory import resolve_run_directory, finalize_run, write_subordinate_manifest, register_config_snapshot


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--parameters", type=Path, required=True,
                        help="explicit frozen ROS parameters YAML; no current-default substitution")
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--mode", choices=("retime", "refine", "backend"), default="backend")
    parser.add_argument("--isolated-budget", action="store_true", help="separate mechanism experiment; does not reproduce captured remaining resources")
    args = parser.parse_args()
    data = json.loads(args.snapshot.read_text())
    if data["kind"] not in ("attempt_failure_curve", "attempt_failure"):
        raise ValueError("requires final candidate disposition")
    run = resolve_run_directory(entrypoint="curve_backend_replay", scenario="icra_dense_forest_four_fork_v2")
    os.environ["IAP_RUN_DIR"] = str(run)
    os.environ["ROS_LOG_DIR"] = str(run / "runtime/ros")
    status = "failed"
    result = {"schema": "iap_curve_backend_replay_manifest_v1", "identity": "OFFLINE_MECHANISM_REPLAY",
              "mode": args.mode, "mission_pass": False}
    try:
        frozen = run / "metadata/config/replay_parameters.yaml"
        frozen.write_bytes(args.parameters.read_bytes())
        register_config_snapshot(run, frozen)
        binary = args.binary.resolve(strict=True)
        command = [str(binary), str(args.snapshot.resolve()), args.mode,
                   *(["isolated-budget"] if args.isolated_budget else []),
                   "--ros-args", "--params-file", str(frozen)]
        result.update(source_identity())
        result["backend_source_sha256"] = {str(p.relative_to(REPO)): sha(p)
            for folder in ("plan_manage", "bspline_opt", "plan_env", "path_searching")
            for p in sorted((REPO / "src/iap/planner" / folder).rglob("*"))
            if p.suffix in (".cpp", ".h") and p.is_file()}
        result.update(command=command, binary=binary_identity(binary),
                      input_sha256={str(args.snapshot.resolve()): sha(args.snapshot),
                                    str((args.snapshot.parent / data["cell_flags_file"]).resolve()): sha(args.snapshot.parent / data["cell_flags_file"])},
                      parameters_sha256=sha(frozen))
        with (run / "runtime/curve_replay.log").open("x") as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=15)
        result["exit_code"] = process.returncode
        output = run / "export/planner/curve_replay/result.json"
        if output.exists():
            result["result_sha256"] = sha(output)
            status = "completed"  # A reproduced rejection is a completed offline experiment.
            result["verdict"] = json.loads(output.read_text())
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        write_subordinate_manifest(run, "curve_backend_replay", result)
        finalize_run(run, lifecycle=status)
    print(json.dumps({"run": str(run), "verdict": result.get("verdict", {}).get("dynamics_feasible"), "lifecycle": status}))
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
