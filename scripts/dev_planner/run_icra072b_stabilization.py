#!/usr/bin/env python3
"""Run the canonical offline ICRA-072B production stabilization matrix."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path


SCHEMA_VERSION = "icra072b_stabilization_matrix_v1"
REPOSITORY = Path(__file__).resolve().parents[2]
RESULTS_ROOT = (REPOSITORY / "results/icra27/icra072b").resolve()
PROTECTED_PDF = "docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"
PROTECTED_PDF_SHA256 = (
    "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6")
MATRIX_ROWS = (
    {
        "name": "happy_path",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "ProductionFsmPublishesCompleteSameAttemptManagerP5RuntimeChain"}],
    },
    {
        "name": "occupancy_epoch",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "OccupancyChangeAfterReleaseBlocksFirstManagerWriterAndDownstreamRows"}],
    },
    {
        "name": "attempt_request_identity",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "MismatchedAttemptBlocksFinalLineageAndPublication"},
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "MismatchedSegmentOrRequestBlocksFinalLineageAndPublication"},
            {"suite": "p4_identity_decision", "name":
             "P4CollisionGuideDecision."
             "InjectionRechecksRequestAndEpochIdentity"}],
    },
    {
        "name": "snapshot_guide_lineage",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "MismatchedSnapshotOrGuideBlocksFinalLineageAndPublication"},
            {"suite": "p4_identity_decision", "name":
             "P4CollisionGuideDecision."
             "V2IdentityBindsImmutableSnapshotConfiguration"},
            {"suite": "p4_identity_integration", "name":
             "P4CollisionGuideIntegration."
             "InjectionEpochMismatchInvalidatesDecision"},
            {"suite": "p4_identity_integration", "name":
             "P4CollisionGuideIntegration."
             "ReleasedSnapshotLineageRevalidatesAttemptAndLiveOccupancyBeforeTerminalUse"}],
    },
    {
        "name": "final_trajectory_identity",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "InvalidTrajectoryIdentityBlocksFinalLineageAndPublication"},
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "MalformedControlPointsBlockTheProductionTerminalWriter"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_analyzer_accepts_one_complete_identity_ordered_end_to_end"}],
    },
    {
        "name": "p5_final_authority",
        "assertions": [
            {"suite": "production_terminal", "name":
             "P4VerticalSliceTerminalLineageTest."
             "ProductionFsmPublishesNothingWhenFusedCurrentIsUnsafeDespiteSafeLidar"}],
    },
    {
        "name": "p5_runtime_authority",
        "assertions": [
            {"suite": "p5_runtime", "name":
             "P5RuntimeIntegrityGateTest."
             "AuthoritativeFusedCurrentRejectsUnsafeDespiteValidFiniteLidar"},
            {"suite": "p5_runtime", "name":
             "P5RuntimeIntegrityGateTest."
             "FutureBadInsideEmergencyTimeRequestsCandidate"},
            {"suite": "p5_runtime", "name":
             "P5RuntimeIntegrityGateTest."
             "RuntimeSustainedCurrentLowMarginEscalates"},
            {"suite": "p5_runtime", "name":
             "P5RuntimeIntegrityGateTest."
             "RuntimeAndFinalCarryExactTrajectoryIdAndNanosecondStart"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_analyzer_accepts_one_complete_identity_ordered_end_to_end"}],
    },
    {
        "name": "operational_closure",
        "assertions": [
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_analyzer_accepts_one_complete_identity_ordered_end_to_end"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_iterative_runner_accepts_only_fresh_layer1_run_and_shared_install"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_source_binding_allows_only_exact_protected_pdf"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_runner_finalizes_source_change_after_owned_cleanup"},
            {"suite": "tools", "name":
             "Icra072VerticalSliceToolsTest."
             "test_stop_owned_clears_an_actual_child_process_group"}],
    },
)
REQUIRED_ROWS = (
    "happy_path", "occupancy_epoch", "attempt_request_identity",
    "snapshot_guide_lineage", "final_trajectory_identity",
    "p5_final_authority", "p5_runtime_authority", "operational_closure")
REQUIRED_SUITES = (
    "production_terminal", "p4_identity_decision",
    "p4_identity_integration", "p5_runtime", "tools")


def matrix_description() -> dict:
    return {"schema_version": SCHEMA_VERSION, "rows": list(MATRIX_ROWS)}


def _assertion_observed(expected: str, observed: set[str]) -> bool:
    return any(item == expected or item.startswith(expected + ".")
               for item in observed)


def build_summary(source_head: str, suites: list[dict]) -> dict:
    """Build a fail-closed matrix verdict from typed suite observations."""
    suite_cardinality = {
        name: sum(suite.get("name") == name for suite in suites)
        for name in REQUIRED_SUITES}
    by_name = {suite.get("name"): suite for suite in suites
               if suite.get("name") in REQUIRED_SUITES}
    row_cardinality = {
        name: sum(row.get("name") == name for row in MATRIX_ROWS)
        for name in REQUIRED_ROWS}
    rows = []
    for declaration in MATRIX_ROWS:
        required_suites = tuple(dict.fromkeys(
            assertion["suite"] for assertion in declaration["assertions"]))
        missing = [
            assertion["name"] for assertion in declaration["assertions"]
            if assertion["suite"] not in by_name or
            not _assertion_observed(
                assertion["name"], set(by_name[assertion["suite"]].get(
                    "observed_assertions", [])))]
        suites_ok = all(
            suite_cardinality[name] == 1
            and by_name[name].get("exit_code") == 0
            and by_name[name].get("test_count", 0) > 0
            and "expected_test_count" in by_name[name]
            and by_name[name].get("test_count") ==
            by_name[name].get("expected_test_count")
            for name in required_suites)
        rows.append({
            **declaration,
            "required_suites": list(required_suites),
            "missing_assertions": missing,
            "result": "PASS" if suites_ok and not missing else "FAIL",
        })
    source_ok = re.fullmatch(r"[0-9a-f]{40}", source_head) is not None
    suite_set_ok = (
        len(suites) == len(REQUIRED_SUITES)
        and all(count == 1 for count in suite_cardinality.values()))
    row_set_ok = (
        len(MATRIX_ROWS) == len(REQUIRED_ROWS)
        and all(count == 1 for count in row_cardinality.values()))
    result = (
        "PASS" if source_ok and suite_set_ok and row_set_ok
        and all(row["result"] == "PASS" for row in rows) else "FAIL")
    return {
        "schema_version": "icra072b_stabilization_summary_v1",
        "development_stabilization_only": True,
        "qualification_claim": False,
        "scientific_effect_claim": False,
        "source_head": source_head,
        "required_row_cardinality": row_cardinality,
        "required_suite_cardinality": suite_cardinality,
        "suites": suites,
        "matrix_rows": rows,
        "result": result,
    }


def validate_output_paths(output: Path, log_root: Path) -> tuple[Path, Path]:
    resolved_output = output.resolve()
    resolved_logs = log_root.resolve()
    for path in (resolved_output, resolved_logs):
        try:
            path.relative_to(RESULTS_ROOT)
        except ValueError as exc:
            raise SystemExit("ICRA-072B outputs must be under icra072b") from exc
    if resolved_output == RESULTS_ROOT or resolved_logs == RESULTS_ROOT:
        raise SystemExit("ICRA-072B outputs must be below icra072b")
    if resolved_output.exists() or resolved_logs.exists():
        raise SystemExit("ICRA-072B outputs must be new")
    if not resolved_output.parent.is_dir():
        raise SystemExit("ICRA-072B output parent is not ready")
    if not resolved_logs.parent.is_dir():
        raise SystemExit("ICRA-072B log parent is not ready")
    return resolved_output, resolved_logs


def _source_binding() -> dict:
    checks = []
    for argv in (["git", "rev-parse", "HEAD"],
                 ["git", "rev-parse", "origin/dev/icra"],
                 ["git", "status", "--porcelain=v1",
                  "--untracked-files=all"]):
        completed = subprocess.run(
            argv, cwd=REPOSITORY, capture_output=True, text=True,
            check=False)
        checks.append({
            "argv": argv, "exit_code": completed.returncode,
            "stdout": completed.stdout, "stderr": completed.stderr})
    head = checks[0]["stdout"].strip()
    origin = checks[1]["stdout"].strip()
    status = checks[2]["stdout"]
    pdf_path = REPOSITORY / PROTECTED_PDF
    observed_hash = (
        hashlib.sha256(pdf_path.read_bytes()).hexdigest()
        if pdf_path.is_file() and not pdf_path.is_symlink() else None)
    accepted = (
        all(item["exit_code"] == 0 for item in checks)
        and re.fullmatch(r"[0-9a-f]{40}", head) is not None
        and head == origin
        and status == f"?? {PROTECTED_PDF}\n"
        and observed_hash == PROTECTED_PDF_SHA256)
    return {
        "schema_version": "icra072b_source_binding_v1",
        "head_commit": head,
        "origin_dev_icra_commit": origin,
        "status_porcelain": status,
        "protected_pdf": {
            "path": PROTECTED_PDF,
            "expected_sha256": PROTECTED_PDF_SHA256,
            "observed_sha256": observed_hash,
        },
        "checks": checks,
        "accepted": accepted,
    }


def _suite_definitions() -> tuple[dict, ...]:
    planning = "/home/dev/ws_iap/build/ego_planner/test_planning_risk_context"
    p4_decision = "/home/dev/ws_iap/build/bspline_opt/test_p4_collision_guide"
    p4_integration = (
        "/home/dev/ws_iap/build/bspline_opt/"
        "test_p4_collision_guide_integration")
    p5 = "/home/dev/ws_iap/build/ego_planner/test_p5_runtime_integrity_gate"
    return (
        {
            "name": "production_terminal", "expected_test_count": 8,
            "argv": [planning, "--gtest_filter="
                     "P4VerticalSliceTerminalLineageTest.*"],
        },
        {
            "name": "p4_identity_decision", "expected_test_count": 2,
            "argv": [p4_decision, "--gtest_filter="
                     "P4CollisionGuideDecision."
                     "V2IdentityBindsImmutableSnapshotConfiguration:"
                     "P4CollisionGuideDecision."
                     "InjectionRechecksRequestAndEpochIdentity"],
        },
        {
            "name": "p4_identity_integration", "expected_test_count": 2,
            "argv": [p4_integration, "--gtest_filter="
                     "P4CollisionGuideIntegration."
                     "ReleasedSnapshotLineageRevalidatesAttemptAndLiveOccupancyBeforeTerminalUse:"
                     "P4CollisionGuideIntegration."
                     "InjectionEpochMismatchInvalidatesDecision"],
        },
        {
            "name": "p5_runtime", "expected_test_count": 4,
            "argv": [p5, "--gtest_filter="
                     "P5RuntimeIntegrityGateTest."
                     "AuthoritativeFusedCurrentRejectsUnsafeDespiteValidFiniteLidar:"
                     "P5RuntimeIntegrityGateTest."
                     "FutureBadInsideEmergencyTimeRequestsCandidate:"
                     "P5RuntimeIntegrityGateTest."
                     "RuntimeSustainedCurrentLowMarginEscalates:"
                     "P5RuntimeIntegrityGateTest."
                     "RuntimeAndFinalCarryExactTrajectoryIdAndNanosecondStart"],
        },
        {
            "name": "tools", "expected_test_count": 17,
            "argv": [sys.executable,
                     str(REPOSITORY / "test/test_icra072_vertical_slice_tools.py"),
                     "-v"],
        },
    )


def _observations(output: str) -> tuple[int, list[str]]:
    assertions = set(re.findall(
        r"^\[\s+OK\s+\] ([^ ]+)", output, flags=re.MULTILINE))
    for method, class_name in re.findall(
            r"^(test_[A-Za-z0-9_]+) \(__main__\.([A-Za-z0-9_]+)\.",
            output, flags=re.MULTILINE):
        assertions.add(f"{class_name}.{method}")
    counts = [int(value) for value in re.findall(
        r"\[  PASSED  \] ([0-9]+) tests?\.", output)]
    counts.extend(int(value) for value in re.findall(
        r"^Ran ([0-9]+) tests? in ", output, flags=re.MULTILINE))
    return (counts[-1] if counts else 0), sorted(assertions)


def _run_suites(log_root: Path) -> list[dict]:
    environment_root = log_root / "environment"
    environment = dict(os.environ)
    for name in ("home", "ros_home", "ros_logs", "tmp", "xdg_runtime"):
        path = environment_root / name
        path.mkdir(parents=True)
        if name == "xdg_runtime":
            path.chmod(0o700)
    environment.update({
        "HOME": str(environment_root / "home"),
        "ROS_HOME": str(environment_root / "ros_home"),
        "ROS_LOG_DIR": str(environment_root / "ros_logs"),
        "TMPDIR": str(environment_root / "tmp"),
        "XDG_RUNTIME_DIR": str(environment_root / "xdg_runtime"),
    })
    results = []
    for suite in _suite_definitions():
        completed = subprocess.run(
            suite["argv"], cwd=REPOSITORY, env=environment,
            capture_output=True, text=True, check=False)
        combined = completed.stdout + completed.stderr
        log_path = log_root / f"{suite['name']}.log"
        log_path.write_text(combined)
        count, assertions = _observations(combined)
        results.append({
            **suite,
            "cwd": str(REPOSITORY),
            "environment": {
                key: environment[key] for key in
                ("HOME", "ROS_HOME", "ROS_LOG_DIR", "TMPDIR",
                 "XDG_RUNTIME_DIR")},
            "exit_code": completed.returncode,
            "test_count": count,
            "observed_assertions": assertions,
            "log_path": str(log_path.relative_to(REPOSITORY)),
        })
        print(f"{suite['name']} exit={completed.returncode} tests={count}")
    return results


def run(output: Path, log_root: Path) -> int:
    output, log_root = validate_output_paths(output, log_root)
    source = _source_binding()
    if source["accepted"] is not True:
        print("SOURCE_BINDING_NOT_READY", file=sys.stderr)
        return 2
    log_root.mkdir()
    suites = _run_suites(log_root)
    summary = build_summary(source["head_commit"], suites)
    summary["source_binding"] = source
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(summary["result"])
    return 0 if summary["result"] == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--describe-matrix", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--log-root", type=Path)
    args = parser.parse_args()
    if args.describe_matrix:
        print(json.dumps(matrix_description(), indent=2, sort_keys=True))
        return 0
    if args.output is None or args.log_root is None:
        parser.error("canonical execution requires --output and --log-root")
    return run(args.output, args.log_root)


if __name__ == "__main__":
    raise SystemExit(main())
