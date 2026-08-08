#!/usr/bin/env python3
"""Run the resumable, serial, one-shot P1-2 qualification/formal campaign."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


SCHEMA = "p1_2_campaign_v1"
REPO = Path(__file__).resolve().parents[2]


class CampaignError(RuntimeError):
    pass


def _launch_step(step_id: str, phase: str, scenario: str, metrics_only: bool,
                 campaign_root: Path, *, record_bag: bool = False,
                 calibration: bool = False) -> dict[str, Any]:
    run_root = campaign_root / "runs" / step_id
    command = [
        "ros2", "launch", "iap", "test_planner.launch.py",
        "experiment:=p1_fork_formal", f"scenario:={scenario}",
        "planner_safety_profile:=p1", f"p1.metrics_only:={'true' if metrics_only else 'false'}",
        "p1.lambda_integrity:=0.00001", "p1.normalization_budget_fraction:=0.30",
        "run_duration_s:=90", "validation_duration_s:=90", "start_rviz:=false",
        "run_validator:=true", f"record_bag:={'true' if record_bag else 'false'}",
        f"runtime_root_dir:={run_root / 'runtime'}",
        f"export_root_dir:={run_root / 'exports'}",
        f"bag_output_dir:={run_root / 'bags'}",
    ]
    if calibration:
        command.append(
            f"p1.formal_calibration_manifest:={campaign_root / 'calibration' / 'p1_formal_tolerance_calibration_v1.json'}"
        )
    return {"id": step_id, "phase": phase, "scenario": scenario,
            "metrics_only": metrics_only, "record_bag": record_bag,
            "status": "planned", "command": command}


def build_plan(campaign_root: Path) -> list[dict[str, Any]]:
    root = Path(campaign_root)
    steps = []
    prequalification = [
        ("pre_primary_1_reference", "p1_fork_fused_v1", True),
        ("pre_primary_1_enabled", "p1_fork_fused_v1", False),
        ("pre_primary_2_reference", "p1_fork_fused_v1", True),
        ("pre_primary_2_enabled", "p1_fork_fused_v1", False),
        ("pre_mirror_reference", "p1_fork_fused_mirror_v1", True),
        ("pre_mirror_enabled", "p1_fork_fused_mirror_v1", False),
        ("pre_null_reference", "p1_fork_symmetric_null_v1", True),
        ("pre_null_enabled", "p1_fork_symmetric_null_v1", False),
        ("pre_soft_reference", "p1_soft_risk_island_v1", True),
        ("pre_soft_enabled", "p1_soft_risk_island_v1", False),
    ]
    steps.extend(_launch_step(name, "prequalification", scenario, metrics, root)
                 for name, scenario, metrics in prequalification)
    steps.append({"id": "prequalification_analysis", "phase": "prequalification_analysis",
                  "status": "planned", "command": [
                      sys.executable, str(REPO / "scripts/dev_planner/analyze_p1_prequalification.py"),
                      "--runs-manifest", str(root / "prequalification_runs.json"),
                      "--output-dir", str(root / "prequalification")],})
    for index in range(20):
        steps.append(_launch_step(f"calibration_{index + 1:02d}", "calibration",
                                  "p1_fork_fused_v1", True, root))
    steps.append({"id": "freeze_calibration", "phase": "calibration_freeze",
                  "status": "planned", "command": [
                      sys.executable, str(REPO / "scripts/dev_planner/calibrate_p1_formal_tolerances.py"),
                      "--pairs-manifest", str(root / "calibration_pairs.json"),
                      "--output-dir", str(root / "calibration")],})
    steps.append(_launch_step("formal_reference", "formal_run", "p1_fork_fused_v1",
                              True, root, record_bag=True, calibration=True))
    steps.append({"id": "formal_reference_preflight", "phase": "preflight",
                  "run_step": "formal_reference", "metrics_only": True,
                  "status": "planned", "command": ["<materialized-after-formal-reference>"]})
    steps.append(_launch_step("formal_enabled", "formal_run", "p1_fork_fused_v1",
                              False, root, record_bag=True, calibration=True))
    steps.append({"id": "formal_enabled_preflight", "phase": "preflight",
                  "run_step": "formal_enabled", "metrics_only": False,
                  "status": "planned", "command": ["<materialized-after-formal-enabled>"]})
    steps.append({"id": "formal_analysis", "phase": "formal_analyzer",
                  "status": "planned", "invocation_count": 0,
                  "command": ["<materialized-after-formal-preflights>"]})
    return steps


def new_state(git_sha: str, plan: list[dict[str, Any]]) -> dict[str, Any]:
    return {"schema_version": SCHEMA, "git_sha": git_sha,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "status": "running", "steps": plan}


def save_state(path: Path, state: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(state, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def load_state(path: Path, git_sha: str) -> dict[str, Any]:
    state = json.loads(path.read_text())
    if state.get("schema_version") != SCHEMA:
        raise CampaignError("campaign schema is not supported")
    if state.get("git_sha") != git_sha:
        raise CampaignError("campaign cannot resume under a different code SHA")
    return state


def _git(*args: str) -> str:
    result = subprocess.run(["git", *args], cwd=REPO, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if result.returncode:
        raise CampaignError(result.stderr.strip() or "git command failed")
    return result.stdout.strip()


def _discover_run(step: dict[str, Any], campaign_root: Path) -> None:
    run_root = campaign_root / "runs" / step["id"]
    manifests = list((run_root / "exports").glob("*/test_planner_manifest.json"))
    if len(manifests) != 1:
        raise CampaignError(f"{step['id']} produced {len(manifests)} manifests, expected one")
    manifest = json.loads(manifests[0].read_text())
    provenance = manifest.get("artifact_provenance", {}) or {}
    step["manifest_path"] = str(manifests[0].resolve())
    step["export_dir"] = str(manifests[0].parent.resolve())
    step["run_id"] = provenance.get("run_id")
    step["bag_dir"] = str(Path(str(provenance.get("bag_path", ""))).resolve())
    validator_path = manifests[0].parent / "test_planner_validation_summary.json"
    try:
        validator = json.loads(validator_path.read_text())
        step["gate_result"] = {"validator_passed": validator.get("passed"),
                               "validator_errors": validator.get("errors", [])}
    except (OSError, json.JSONDecodeError):
        step["gate_result"] = {"validator_passed": False,
                               "validator_errors": ["validator summary unreadable"]}


def _capture_gate_result(step: dict[str, Any], root: Path) -> None:
    candidate: Path | None = None
    if step["phase"] == "prequalification_analysis":
        candidate = root / "prequalification/p1_2_prequalification_summary.json"
    elif step["phase"] == "calibration_freeze":
        candidate = root / "calibration/p1_formal_tolerance_calibration_v1.json"
    elif step["phase"] == "preflight":
        candidate = Path(step["output"])
    if step["phase"] == "formal_analyzer":
        # The analyzer owns this stable path in the enabled export.
        command = step.get("command", [])
        export_index = command.index("--export-dir") + 1
        candidate = Path(command[export_index]) / "metadata/safety_planner_analysis_summary.json"
    if candidate and candidate.is_file():
        try:
            payload = json.loads(candidate.read_text())
            step["gate_result"] = {
                key: payload.get(key) for key in (
                    "passed", "status", "failures", "inconclusive", "thresholds",
                    "calibration_id"
                ) if key in payload
            }
            step["result_path"] = str(candidate.resolve())
        except json.JSONDecodeError:
            step["gate_result"] = {"passed": False, "error": "result JSON unreadable"}


def _recover_interrupted_step(step: dict[str, Any], state: dict[str, Any], root: Path) -> bool:
    """Recover only finalized evidence; never rerun a partially observed run."""
    try:
        if step["phase"] in {"prequalification", "calibration", "formal_run"}:
            _discover_run(step, root)
            complete = step.get("gate_result", {}).get("validator_passed") is True
            if step.get("record_bag"):
                manifest = json.loads(Path(step["manifest_path"]).read_text())
                provenance = manifest.get("artifact_provenance", {}) or {}
                complete = complete and bool(provenance.get("process_end_epoch_s")) \
                    and (Path(step["bag_dir"]) / "metadata.yaml").is_file()
        else:
            _materialize_command(step, state, root)
            _capture_gate_result(step, root)
            complete = bool(step.get("gate_result"))
            if step["phase"] in {"prequalification_analysis", "preflight", "formal_analyzer"}:
                complete = complete and step["gate_result"].get("passed") is not None
    except (CampaignError, OSError, ValueError, KeyError, json.JSONDecodeError):
        complete = False
    verdict = step.get("gate_result", {}).get("passed")
    successful = complete and verdict is not False
    step["recovered_after_interrupt"] = complete
    step["exit_code"] = 0 if successful else (2 if complete else None)
    step["status"] = "completed" if successful else "failed"
    if not complete:
        step["interruption_error"] = "interrupted step has no finalized, gate-readable evidence"
    save_state(root / "campaign.json", state)
    return complete


def _completed_step(state: dict[str, Any], step_id: str) -> dict[str, Any]:
    matches = [step for step in state["steps"] if step["id"] == step_id]
    if len(matches) != 1 or matches[0].get("status") != "completed":
        raise CampaignError(f"required step is not complete: {step_id}")
    return matches[0]


def _materialize_command(step: dict[str, Any], state: dict[str, Any], root: Path) -> None:
    if step["phase"] == "prequalification_analysis":
        pair_ids = [
            ("primary", "pre_primary_1_reference", "pre_primary_1_enabled"),
            ("primary", "pre_primary_2_reference", "pre_primary_2_enabled"),
            ("mirror", "pre_mirror_reference", "pre_mirror_enabled"),
            ("null", "pre_null_reference", "pre_null_enabled"),
            ("soft_risk", "pre_soft_reference", "pre_soft_enabled"),
        ]
        payload = {"schema_version": "p1_2_prequalification_runs_v1", "pairs": [
            {"kind": kind, "reference_export": _completed_step(state, ref)["export_dir"],
             "enabled_export": _completed_step(state, enabled)["export_dir"]}
            for kind, ref, enabled in pair_ids]}
        (root / "prequalification_runs.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n")
    elif step["phase"] == "calibration_freeze":
        runs = [_completed_step(state, f"calibration_{index:02d}")
                for index in range(1, 21)]
        payload = {"schema_version": "p1_formal_calibration_pairs_v1",
                   "calibration_id": f"p1-null-{time.strftime('%Y%m%d', time.gmtime())}-{state['git_sha'][:7]}",
                   "pairs": [{"pair_id": f"null-{index + 1:02d}",
                              "run_a_export": runs[index * 2]["export_dir"],
                              "run_b_export": runs[index * 2 + 1]["export_dir"]}
                             for index in range(10)]}
        (root / "calibration_pairs.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n")
    elif step["phase"] == "preflight":
        run = _completed_step(state, step["run_step"])
        output = root / "formal" / f"{step['id']}.json"
        output.parent.mkdir(parents=True, exist_ok=True)
        step["command"] = [
            sys.executable, str(REPO / "scripts/dev_planner/verify_safety_planner_evidence_bundle.py"),
            "--export-dir", run["export_dir"], "--bag-dir", run["bag_dir"],
            "--metrics-only", "true" if step["metrics_only"] else "false",
            "--lambda-integrity", "0.00001", "--p1-calibration",
            str(root / "calibration/p1_formal_tolerance_calibration_v1.json"),
            "--json-out", str(output)]
        step["output"] = str(output)
    elif step["phase"] == "formal_analyzer":
        reference = _completed_step(state, "formal_reference")
        enabled = _completed_step(state, "formal_enabled")
        step["command"] = [
            sys.executable, str(REPO / "scripts/dev_planner/analyze_safety_planner_run.py"),
            "--experiment-id", "P1-2", "--export-dir", enabled["export_dir"],
            "--bag-dir", enabled["bag_dir"], "--baseline-export-dir", reference["export_dir"],
            "--baseline-bag-dir", reference["bag_dir"], "--p1-calibration",
            str(root / "calibration/p1_formal_tolerance_calibration_v1.json"),
            "--fail-on-threshold"]


def run_campaign(campaign_root: Path, *, git_sha: str | None = None,
                 dry_run: bool = False) -> dict[str, Any]:
    root = Path(campaign_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    state_path = root / "campaign.json"
    sha = git_sha or _git("rev-parse", "HEAD")
    if state_path.exists():
        state = load_state(state_path, sha)
    else:
        state = new_state(sha, build_plan(root))
        save_state(state_path, state)
    if dry_run:
        state["status"] = "dry_run"
        save_state(state_path, state)
        return state
    if _git("status", "--porcelain"):
        raise CampaignError("campaign requires a clean git worktree")

    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(root / "ros_logs")
    environment["XDG_RUNTIME_DIR"] = str(root / "xdg_runtime")
    Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    Path(environment["XDG_RUNTIME_DIR"]).mkdir(parents=True, exist_ok=True)
    os.chmod(environment["XDG_RUNTIME_DIR"], 0o700)
    for step in state["steps"]:
        if step.get("status") == "running":
            finalized = _recover_interrupted_step(step, state, root)
            if not finalized or step.get("status") == "failed":
                state["status"] = (
                    "formal_blocked" if step["phase"] == "formal_analyzer" else "failed"
                )
                save_state(state_path, state)
                if step["phase"] == "formal_analyzer" and finalized:
                    return state
                raise CampaignError(
                    f"interrupted evidence is retained and cannot be rerun: {step['id']}")
        if step.get("status") == "completed":
            continue
        if step.get("status") == "failed":
            raise CampaignError(f"failed evidence is retained; start a fresh campaign after repair: {step['id']}")
        _materialize_command(step, state, root)
        if step["phase"] == "formal_analyzer":
            if step.get("invocation_count", 0) != 0:
                raise CampaignError("formal analyzer invocation is already consumed")
            step["invocation_count"] = 1
        step["status"] = "running"
        step["started_at_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        save_state(state_path, state)
        log_path = root / "commands" / f"{step['id']}.log"
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w") as log:
            result = subprocess.run(step["command"], cwd=REPO, env=environment,
                                    stdout=log, stderr=subprocess.STDOUT, check=False)
        step["exit_code"] = result.returncode
        step["log_path"] = str(log_path)
        step["ended_at_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        if step["phase"] in {"prequalification", "calibration", "formal_run"} and result.returncode == 0:
            _discover_run(step, root)
        _capture_gate_result(step, root)
        if result.returncode != 0:
            step["status"] = "failed"
            state["status"] = "formal_blocked" if step["phase"] == "formal_analyzer" else "failed"
            save_state(state_path, state)
            if step["phase"] == "formal_analyzer":
                return state
            raise CampaignError(f"campaign step failed: {step['id']} (exit {result.returncode})")
        step["status"] = "completed"
        save_state(state_path, state)
    state["status"] = "formal_pass"
    save_state(state_path, state)
    return state


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-root", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    root = args.campaign_root or (
        REPO / "results/planner_validation/campaigns" /
        time.strftime("p1-2-%Y%m%dT%H%M%SZ", time.gmtime())
    )
    try:
        state = run_campaign(root, dry_run=args.dry_run)
    except CampaignError as exc:
        print(json.dumps({"passed": False, "error": str(exc), "campaign_root": str(root)}, indent=2))
        return 2
    print(json.dumps({"passed": state["status"] == "formal_pass" or args.dry_run,
                      "status": state["status"], "campaign_root": str(root.resolve())}, indent=2))
    return 0 if state["status"] in {"formal_pass", "dry_run"} else 2


if __name__ == "__main__":
    sys.exit(main())
