#!/usr/bin/env python3
"""Fail-closed, fast provenance preflight for one fresh P1 evidence bundle.

This intentionally does not call the expensive P1-2 analyzer.  It is used by
the short plumbing smoke and by the two formal bundles before their single
authoritative analyzer invocation.
"""

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path


SCHEMA = "p1_evidence_provenance_v3"
CSV_KEYS = {
    "planner_p1_integrity_cost_debug.csv": "p1.debug_csv_path",
    "planner_p1_candidate_optimization.csv": "p1.candidate_optimization_path",
    "planner_p1_accepted_trajectory_risk_profile.csv": "p1.accepted_profile_path",
    "planner_p1_accepted_trajectory_risk_profile_context.csv": "p1.accepted_profile_context_path",
    "planner_p1_planning_context_timeline.csv": "p1.planning_context_timeline_path",
}
P1_RVIZ_TOPICS = (
    "/iap/rviz/p1_integrity_samples",
    "/iap/rviz/p1_integrity_push_vectors",
    "/iap/rviz/p1_integrity_metrics",
)


def read_bag_provenance(bag_dir):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
                    rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
        types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        message_type = get_message(types["/planning/evidence_provenance"])
        payloads = []
        while reader.has_next():
            topic, raw, _ = reader.read_next()
            if topic == "/planning/evidence_provenance":
                payload = json.loads(str(deserialize_message(raw, message_type).data))
                if isinstance(payload, dict):
                    payloads.append(payload)
        return payloads, ""
    except Exception as exc:  # pragma: no cover - requires ROS bag bindings
        return [], str(exc)


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def truthy(value):
    return str(value).strip().lower() in {"1", "true"}


def read_csv(path, expected_manifest, expected_run):
    errors = []
    if not path.is_file() or path.stat().st_size == 0:
        return [], [f"missing or empty CSV: {path}"]
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        rows = list(reader)
    required = {"schema_version", "run_id", "manifest_path"}
    if not required.issubset(fields):
        errors.append(f"legacy provenance CSV header: {path}")
    if not rows:
        errors.append(f"CSV has no rows: {path}")
    for index, row in enumerate(rows):
        if row.get("schema_version") != SCHEMA:
            errors.append(f"schema mismatch in {path} row {index}")
        if row.get("run_id") != expected_run:
            errors.append(f"run ID mismatch in {path} row {index}")
        try:
            actual_manifest = Path(row.get("manifest_path", "")).resolve()
        except OSError:
            actual_manifest = Path("")
        if actual_manifest != expected_manifest:
            errors.append(f"manifest binding mismatch in {path} row {index}")
    return rows, errors


def validate_bundle(export_dir, bag_dir, *, metrics_only, lambda_value):
    errors = []
    export_dir, bag_dir = Path(export_dir).resolve(), Path(bag_dir).resolve()
    manifest_path = export_dir / "test_planner_manifest.json"
    if not manifest_path.is_file():
        return {"passed": False, "errors": [f"missing manifest: {manifest_path}"]}
    manifest = json.loads(manifest_path.read_text())
    provenance = manifest.get("artifact_provenance")
    if not isinstance(provenance, dict):
        return {"passed": False, "errors": ["manifest has no artifact_provenance"]}
    run_id = str(provenance.get("run_id", ""))
    if provenance.get("schema_version") != SCHEMA or not run_id:
        errors.append("manifest schema version or run ID is invalid")
    if Path(str(manifest.get("export_dir", ""))).resolve() != export_dir:
        errors.append("manifest export path does not bind this export")
    if Path(str(provenance.get("bag_path", ""))).resolve() != bag_dir:
        errors.append("manifest bag path does not bind this bag")
    if not str(provenance.get("git_commit", "")) or not provenance.get("git_worktree_clean"):
        errors.append("manifest does not prove a clean git commit")
    for key in ("launch", "planner_executable", "bspline_library"):
        entry = (provenance.get("runtime_paths", {}) or {}).get(key, {}) or {}
        path, digest = Path(str(entry.get("path", ""))), str(entry.get("sha256", ""))
        if not digest or not path.is_file():
            errors.append(f"runtime {key} path/hash is unavailable")
        elif hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append(f"runtime {key} hash no longer matches manifest")
    if not provenance.get("process_start_stamp_utc") or not provenance.get("process_end_stamp_utc"):
        errors.append("manifest lacks process start/end stamps; run finalizer after recorder exit")
    if manifest.get("planner_safety_profile") != "p1":
        errors.append("manifest planner_safety_profile is not p1")
    if bool(manifest.get("p1.metrics_only")) != bool(metrics_only):
        errors.append("manifest metrics_only does not match requested mode")
    if abs(float(manifest.get("p1.lambda_integrity", float("nan"))) - lambda_value) > 1e-12:
        errors.append("manifest lambda_integrity does not match requested lambda")
    if not provenance.get("validator_summary_complete"):
        errors.append("validator summary has not been finalized")
    if not provenance.get("bag_metadata_complete"):
        errors.append("bag metadata has not been finalized")

    csv_rows = {}
    for filename, manifest_key in CSV_KEYS.items():
        raw_path = Path(str(manifest.get(manifest_key, export_dir / filename)))
        path = raw_path.resolve() if raw_path.is_absolute() else (export_dir / raw_path).resolve()
        if path.parent != export_dir:
            errors.append(f"artifact escapes export directory: {path}")
        if path.name != filename:
            errors.append(f"manifest path has wrong artifact name: {path}")
        if provenance.get("process_start_epoch_s") and path.exists() and path.stat().st_mtime + 1e-3 < float(provenance["process_start_epoch_s"]):
            errors.append(f"artifact predates current run: {path}")
        rows, row_errors = read_csv(path, manifest_path, run_id)
        csv_rows[filename] = rows
        errors.extend(row_errors)

    validator_path = export_dir / "test_planner_validation_summary.json"
    if not validator_path.is_file():
        errors.append("validator summary is missing")
    else:
        validator = json.loads(validator_path.read_text())
        if validator.get("schema_version") != SCHEMA or validator.get("run_id") != run_id:
            errors.append("validator provenance identity mismatch")
        if Path(str(validator.get("manifest_path", ""))).resolve() != manifest_path:
            errors.append("validator manifest binding mismatch")
        if not validator.get("passed"):
            errors.append("validator did not pass")

    candidate = csv_rows.get("planner_p1_candidate_optimization.csv", [])
    attempts = {}
    for row in candidate:
        attempts.setdefault(row.get("planning_attempt_id"), []).append(row)
        for field in ("pre_mean_c_pi", "pre_max_c_pi", "post_mean_c_pi", "post_max_c_pi"):
            if not finite(row.get(field)):
                errors.append(f"candidate {field} is non-finite")
        sample_count = row.get("support_sample_count")
        pre_count = row.get("pre_support_valid_count")
        post_count = row.get("post_support_valid_count")
        pre_coverage = row.get("pre_support_coverage")
        post_coverage = row.get("post_support_coverage")
        if not truthy(row.get("support_full_valid")) or \
                not all(finite(value) for value in (sample_count, pre_count, post_count,
                                                    pre_coverage, post_coverage)) or \
                float(sample_count) <= 0.0 or \
                float(pre_count) != float(sample_count) or \
                float(post_count) != float(sample_count) or \
                float(pre_coverage) != 1.0 or float(post_coverage) != 1.0:
            errors.append("candidate lacks full valid fixed-lattice support")
    if not candidate:
        errors.append("candidate optimization CSV is empty")
    for attempt, rows in attempts.items():
        selected = [row for row in rows if truthy(row.get("selected")) and truthy(row.get("optimization_success"))]
        if not 1 <= len(rows) <= 8 or len(selected) != 1:
            errors.append(f"attempt {attempt} lacks exactly one selected successful candidate")
    rejected = [row for row in candidate if truthy(row.get("selected")) and
                not truthy(row.get("replacement_accepted"))]
    if rejected:
        decision_path = Path(str(manifest.get(
            "p1.replacement_decision_path", export_dir / "planner_p1_replacement_decision.csv")))
        profile_path = Path(str(manifest.get(
            "p1.candidate_retained_profile_path", export_dir / "planner_p1_candidate_retained_profile.csv")))
        decision_rows, decision_errors = read_csv(decision_path, manifest_path, run_id)
        profile_rows, profile_errors = read_csv(profile_path, manifest_path, run_id)
        errors.extend(decision_errors)
        errors.extend(profile_errors)
        for row in rejected:
            attempt, candidate_id = row.get("planning_attempt_id"), row.get("candidate_id")
            decisions = [item for item in decision_rows
                if item.get("planning_attempt_id") == attempt and
                item.get("optimizer_selected_candidate_id") == candidate_id and
                not truthy(item.get("replacement_accepted")) and
                item.get("final_trajectory_source") == "retained_incumbent"]
            compared = [item for item in profile_rows
                if item.get("planning_attempt_id") == attempt and item.get("candidate_id") == candidate_id]
            if len(decisions) != 1 or sum(item.get("trajectory_role") == "optimizer_selected_candidate" for item in compared) != 200 or \
                    sum(item.get("trajectory_role") == "retained_incumbent" for item in compared) != 200:
                errors.append(f"rejected candidate {attempt}/{candidate_id} lacks retained-incumbent closure")
    debug = csv_rows.get("planner_p1_integrity_cost_debug.csv", [])
    if not metrics_only and not any(truthy(row.get("applied_to_objective")) for row in debug):
        errors.append("enabled P1 debug has no applied_to_objective=true")
    profile = csv_rows.get("planner_p1_accepted_trajectory_risk_profile.csv", [])
    if len(profile) < 200:
        errors.append("accepted profile has fewer than 200 samples")

    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file() or metadata_path.stat().st_size == 0:
        errors.append("bag metadata is missing or empty")
    else:
        metadata = metadata_path.read_text(errors="replace")
        for topic in ("/planning/evidence_provenance", "/planning/risk_grid_health", "/drone_0_planning/bspline", *P1_RVIZ_TOPICS):
            if topic not in metadata:
                errors.append(f"bag metadata has no recorded topic: {topic}")
        payloads, payload_error = read_bag_provenance(bag_dir)
        if payload_error:
            errors.append("bag provenance unreadable: " + payload_error)
        elif not any(
            payload.get("schema_version") == SCHEMA and payload.get("run_id") == run_id
            and Path(str(payload.get("manifest_path", ""))).resolve() == manifest_path
            and Path(str(payload.get("export_dir", ""))).resolve() == export_dir
            and Path(str(payload.get("bag_path", ""))).resolve() == bag_dir
            for payload in payloads
        ):
            errors.append("bag provenance payload does not bind this manifest/run/export/bag")
    return {"passed": not errors, "errors": errors, "run_id": run_id,
            "schema_version": provenance.get("schema_version"), "manifest": str(manifest_path),
            "export_dir": str(export_dir), "bag_dir": str(bag_dir)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--export-dir", required=True)
    parser.add_argument("--bag-dir", required=True)
    parser.add_argument("--metrics-only", choices=("true", "false"), required=True)
    parser.add_argument("--lambda-integrity", type=float, default=0.00001)
    parser.add_argument("--json-out")
    args = parser.parse_args()
    result = validate_bundle(args.export_dir, args.bag_dir,
                             metrics_only=args.metrics_only == "true",
                             lambda_value=args.lambda_integrity)
    encoded = json.dumps(result, indent=2, sort_keys=True)
    if args.json_out:
        Path(args.json_out).write_text(encoded + "\n")
    print(encoded)
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    sys.exit(main())
