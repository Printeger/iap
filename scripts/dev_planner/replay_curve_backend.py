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
    parser.add_argument("--payload", type=Path, help="same-attempt complete Predictor input for ON backend replay")
    parser.add_argument("--require-candidate", action="store_true", help="fail unless complete backend candidate passes captured checks")
    parser.add_argument("--mode", choices=("retime", "refine", "backend", "initialize", "audit"), default="backend")
    parser.add_argument("--isolated-budget", action="store_true", help="separate mechanism experiment; does not reproduce captured remaining resources")
    args = parser.parse_args()
    if args.payload and (args.mode not in ("backend", "initialize") or args.isolated_budget):
        parser.error("complete ON payload requires backend/initialize mode and captured remaining budget")
    if args.mode == "audit" and args.isolated_budget:
        parser.error("audit diagnoses captured stages without running an online budget")
    data = json.loads(args.snapshot.read_text())
    if data["kind"] not in ("attempt_failure_curve", "attempt_failure") and not data["kind"].startswith("committed_"):
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
        if args.payload:
            from replay_on_search import verify_planning_capture
            result["capture_authority"] = verify_planning_capture(args.snapshot.parent, args.payload, data)
            result["payload_sha256"] = sha(args.payload)
        command = [str(binary), str(args.snapshot.resolve()), args.mode,
                   *(["isolated-budget"] if args.isolated_budget else []),
                   *(["--payload", str(args.payload.resolve())] if args.payload else []),
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
        if any(sha(path) != digest for path, digest in result["input_sha256"].items()):
            raise RuntimeError("captured snapshot or map changed during replay")
        if args.payload and sha(args.payload) != result["payload_sha256"]:
            raise RuntimeError("captured Predictor payload changed during replay")
        output = run / "export/planner/curve_replay/result.json"
        if output.exists():
            result["result_sha256"] = sha(output)
            result["verdict"] = json.loads(output.read_text())
            if process.returncode in (0, 1):
                status = "completed"  # A reproduced rejection is a completed offline experiment.
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        write_subordinate_manifest(run, "curve_backend_replay", result)
        finalize_run(run, lifecycle=status)
    verdict = result.get("verdict", {})
    summary = {"run": str(run), "lifecycle": status}
    if args.mode == "audit":
        summary.update(all_stages_checked=verdict.get("all_stages_checked"),
                       all_stages_preserve_route=verdict.get("all_stages_preserve_route"))
    else:
        summary.update(verdict=verdict.get("physical_geometric_candidate_valid"),
                       dynamics_feasible=verdict.get("dynamics_feasible"),
                       physical_executable=verdict.get("physical_executable"),
                       guide_route_preserved=verdict.get("guide_route_preserved"))
    print(json.dumps(summary))
    return 0 if status == "completed" and (not args.require_candidate or
        verdict.get("physical_geometric_candidate_valid") is True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
