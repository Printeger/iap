#!/usr/bin/env python3
"""Fail-closed, seconds-scale replay for P1 pre-admission evidence.

The command intentionally accepts one explicit exported bundle.  It proves
that a base-planning lifecycle and a P1 objective lifecycle are different
things; it never treats a base publish as a P1 candidate.
"""

import argparse
import csv
import json
from collections import Counter
from pathlib import Path


SCHEMA = "p1_evidence_provenance_v3"
TIMELINE = "planner_p1_planning_context_timeline.csv"
PRE_ADMISSION = "planner_p1_pre_admission_attempt.csv"
CANDIDATE = "planner_p1_candidate_optimization.csv"


def read_rows(path: Path):
    if not path.is_file() or not path.stat().st_size:
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def truthy(value):
    return str(value).strip().lower() in {"1", "true"}


def validate(export_dir: Path, run_id: str, require_instrumented: bool):
    export_dir = export_dir.resolve()
    errors = []
    manifest_path = export_dir / "test_planner_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        return {"passed": False, "errors": [f"invalid manifest: {exc}"], "counts": {}}
    provenance = manifest.get("artifact_provenance", {})
    if provenance.get("schema_version") != SCHEMA or provenance.get("run_id") != run_id:
        errors.append("explicit run/schema identity mismatch")
    timeline = read_rows(export_dir / TIMELINE)
    if not timeline:
        errors.append("missing planning context timeline")
    if any(row.get("schema_version") != SCHEMA or row.get("run_id") != run_id
           for row in timeline):
        errors.append("timeline provenance mismatch")
    stage = Counter(row.get("stage", "") for row in timeline)
    outcome = Counter(row.get("outcome", "") for row in timeline)
    reason = Counter(row.get("reason", "") for row in timeline)
    candidate_rows = read_rows(export_dir / CANDIDATE)
    pre_rows = read_rows(export_dir / PRE_ADMISSION)
    counts = {
        "base_optimizer_start": stage["base_optimizer_start"],
        "base_optimizer_success": outcome["candidate_success"],
        "base_optimizer_failure": outcome["candidate_failure"],
        "base_publish": stage["publish"],
        "p1_admission_temporal_out_of_horizon": sum(
            1 for row in timeline if row.get("stage") == "p1_admission" and
            row.get("reason") == "temporal_out_of_horizon"),
        "p1_admission_coverage_insufficient": sum(
            1 for row in timeline if row.get("stage") == "p1_admission" and
            row.get("reason") == "coverage_insufficient"),
        "p1_admission_snapshot_unavailable": sum(
            1 for row in timeline if row.get("stage") == "p1_admission" and
            row.get("reason") == "snapshot_unavailable"),
        "p1_optimizer_start": stage["optimizer_start"],
        "candidate_rows": len(candidate_rows),
        "pre_admission_rows": len(pre_rows),
    }
    # The frozen 2026-08-02 bundle is a characterization fixture.  Its
    # expected failure must remain distinguishable from first-trajectory loss.
    if counts["base_optimizer_success"] <= 0 or counts["base_publish"] <= 0:
        errors.append("fixture does not prove base planning partially succeeded")
    if counts["p1_optimizer_start"] != 0 or counts["candidate_rows"] != 0:
        errors.append("fixture no longer represents zero-P1-candidate failure")
    if (counts["p1_admission_temporal_out_of_horizon"] +
            counts["p1_admission_coverage_insufficient"]) <= 0:
        errors.append("fixture lacks direct temporal/full-support rejection")
    if require_instrumented:
        required = {
            "initial_duration_s", "initial_temporal_margin_s",
            "expected_sample_count", "matched_sample_count",
            "occupied_miss_count", "base_duration_s", "base_full_p1_support",
        }
        if not pre_rows:
            errors.append("missing pre-admission feedback artifact")
        elif not required.issubset(pre_rows[0]):
            errors.append("pre-admission artifact lacks required feedback fields")
    return {"passed": not errors, "errors": errors, "counts": counts,
            "manifest": str(manifest_path), "timeline": str(export_dir / TIMELINE)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--require-instrumented", action="store_true")
    args = parser.parse_args()
    result = validate(args.export_dir, args.run_id, args.require_instrumented)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
