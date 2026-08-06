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


SCHEMA = "p1_evidence_provenance_v4"
CSV_KEYS = {
    "planner_p1_integrity_cost_debug.csv": "p1.debug_csv_path",
    "planner_p1_candidate_optimization.csv": "p1.candidate_optimization_path",
    "planner_p1_candidate_control_points.csv": "p1.candidate_control_points_path",
    "planner_p1_candidate_profile.csv": "p1.candidate_profile_path",
    "planner_p1_candidate_pairwise.csv": "p1.candidate_pairwise_path",
    "planner_p1_optimizer_checkpoint.csv": "p1.optimizer_checkpoint_path",
    "planner_p0_occupancy_query_evidence.csv": "p0.occupancy_query_evidence_path",
    "planner_p1_accepted_trajectory_risk_profile.csv": "p1.accepted_profile_path",
    "planner_p1_accepted_trajectory_risk_profile_context.csv": "p1.accepted_profile_context_path",
    "planner_p1_planning_context_timeline.csv": "p1.planning_context_timeline_path",
}
OPTIMIZER_ATTEMPT_CSVS = frozenset({
    "planner_p1_candidate_optimization.csv",
    "planner_p1_candidate_control_points.csv",
    "planner_p1_candidate_profile.csv",
    "planner_p1_candidate_pairwise.csv",
    "planner_p1_optimizer_checkpoint.csv",
    "planner_p0_occupancy_query_evidence.csv",
})
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

    csv_paths = {}
    for filename, manifest_key in CSV_KEYS.items():
        raw_path = Path(str(manifest.get(manifest_key, export_dir / filename)))
        path = raw_path.resolve() if raw_path.is_absolute() else (export_dir / raw_path).resolve()
        csv_paths[filename] = path
        if path.parent != export_dir:
            errors.append(f"artifact escapes export directory: {path}")
        if path.name != filename:
            errors.append(f"manifest path has wrong artifact name: {path}")
        if provenance.get("process_start_epoch_s") and path.exists() and path.stat().st_mtime + 1e-3 < float(provenance["process_start_epoch_s"]):
            errors.append(f"artifact predates current run: {path}")

    optimizer_attempt_artifacts_present = any(
        csv_paths[filename].exists() for filename in OPTIMIZER_ATTEMPT_CSVS)
    enforce_optimizer_attempt_artifacts = (
        not metrics_only or optimizer_attempt_artifacts_present)

    csv_rows = {}
    for filename, path in csv_paths.items():
        if filename in OPTIMIZER_ATTEMPT_CSVS and not enforce_optimizer_attempt_artifacts:
            csv_rows[filename] = []
            continue
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
    if enforce_optimizer_attempt_artifacts and not candidate:
        errors.append("candidate optimization CSV is empty")
    for attempt, rows in attempts.items():
        selected = [row for row in rows if truthy(row.get("selected")) and truthy(row.get("optimization_success"))]
        if not 1 <= len(rows) <= 8 or len(selected) != 1:
            errors.append(f"attempt {attempt} lacks exactly one selected successful candidate")
        if not metrics_only:
            eligible = [row for row in rows if truthy(row.get("rank_eligible"))]
            if not eligible:
                errors.append(f"attempt {attempt} has no rank-eligible candidate")
            for winner in selected:
                numeric = {
                    field: float(winner[field]) for field in (
                        "pre_mean_c_pi", "pre_max_c_pi", "post_mean_c_pi",
                        "post_max_c_pi", "grad_integrity_dot_displacement")
                    if finite(winner.get(field))
                }
                if len(numeric) != 5 or \
                        numeric["post_mean_c_pi"] > numeric["pre_mean_c_pi"] + 1e-12 or \
                        numeric["post_max_c_pi"] > numeric["pre_max_c_pi"] + 1e-12 or \
                        not (numeric["post_mean_c_pi"] < numeric["pre_mean_c_pi"] - 1e-12 or
                             numeric["post_max_c_pi"] < numeric["pre_max_c_pi"] - 1e-12) or \
                        numeric["grad_integrity_dot_displacement"] >= 0.0:
                    errors.append(f"attempt {attempt} selected winner lacks fixed-200 P1 descent")
                if winner.get("normalization_mode") != "base_improvement_budget_v1" or \
                        not truthy(winner.get("base_prepass_success")):
                    errors.append(f"attempt {attempt} selected winner lacks frozen normalization evidence")

    control_rows = csv_rows.get("planner_p1_candidate_control_points.csv", [])
    profile_rows = csv_rows.get("planner_p1_candidate_profile.csv", [])
    pairwise_rows = csv_rows.get("planner_p1_candidate_pairwise.csv", [])
    checkpoint_rows = csv_rows.get("planner_p1_optimizer_checkpoint.csv", [])
    occupancy_rows = csv_rows.get("planner_p0_occupancy_query_evidence.csv", [])
    candidate_keys = {
        (row.get("planning_attempt_id"), row.get("candidate_id"))
        for row in candidate
    }
    for attempt, candidate_id in candidate_keys:
        points = [row for row in control_rows
                  if row.get("planning_attempt_id") == attempt and
                  row.get("candidate_id") == candidate_id]
        phases = {phase: [row for row in points if row.get("phase") == phase]
                  for phase in ("initial", "final")}
        if any(not rows for rows in phases.values()) or \
                len(phases["initial"]) != len(phases["final"]):
            errors.append(f"candidate {attempt}/{candidate_id} lacks initial/final control points")
        samples = [row for row in profile_rows
                   if row.get("planning_attempt_id") == attempt and
                   row.get("candidate_id") == candidate_id]
        for phase in ("initial", "final"):
            phase_rows = [row for row in samples if row.get("phase") == phase]
            if len(phase_rows) != 200 or \
                    {int(row["sample_index"]) for row in phase_rows
                     if str(row.get("sample_index", "")).isdigit()} != set(range(200)):
                errors.append(f"candidate {attempt}/{candidate_id} {phase} profile is not fixed-200")
            for row in phase_rows:
                if truthy(row.get("valid")):
                    if not finite(row.get("c_pi")) or row.get("invalid_reason") != "none":
                        errors.append(f"candidate {attempt}/{candidate_id} has malformed valid profile sample")
                elif str(row.get("c_pi", "")).strip() or not str(row.get("invalid_reason", "")).strip():
                    errors.append(f"candidate {attempt}/{candidate_id} mixes invalid reason with c_pi")
        checkpoints = [row for row in checkpoint_rows
                       if row.get("planning_attempt_id") == attempt and
                       row.get("candidate_id") == candidate_id]
        names = {(row.get("stage"), row.get("checkpoint")) for row in checkpoints}
        if ("p1_stage", "first_direction") not in names or \
                ("p1_stage", "terminal") not in names:
            errors.append(f"candidate {attempt}/{candidate_id} lacks optimizer checkpoints")
        if not metrics_only and (("base_prepass", "start") not in names or
                                 ("base_prepass", "terminal") not in names):
            errors.append(f"candidate {attempt}/{candidate_id} lacks base-prepass checkpoints")
    if enforce_optimizer_attempt_artifacts and not occupancy_rows:
        errors.append("P0 occupancy query evidence is empty")
    if candidate and not pairwise_rows:
        errors.append("candidate pairwise evidence is empty")
    if not metrics_only and pairwise_rows:
        final_distances = [float(row["control_point_distance"])
                           for row in pairwise_rows
                           if row.get("phase") == "final" and
                           finite(row.get("control_point_distance")) and
                           row.get("candidate_id_a") != row.get("candidate_id_b")]
        if not final_distances or max(final_distances) <= 1e-4:
            errors.append("final candidates collapse in control-point space")
    rejected = [] if metrics_only else [
        row for row in candidate if truthy(row.get("selected")) and
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
            incumbent_available = truthy(row.get("incumbent_available"))
            expected_source = ("retained_incumbent" if incumbent_available
                               else "no_publish_no_incumbent")
            decisions = [item for item in decision_rows
                if item.get("planning_attempt_id") == attempt and
                item.get("optimizer_selected_candidate_id") == candidate_id and
                not truthy(item.get("replacement_accepted")) and
                item.get("final_trajectory_source") == expected_source]
            compared = [item for item in profile_rows
                if item.get("planning_attempt_id") == attempt and item.get("candidate_id") == candidate_id]
            expected_incumbent_samples = 200 if incumbent_available else 0
            if len(decisions) != 1 or sum(item.get("trajectory_role") == "optimizer_selected_candidate" for item in compared) != 200 or \
                    sum(item.get("trajectory_role") == "retained_incumbent" for item in compared) != expected_incumbent_samples:
                errors.append(f"rejected candidate {attempt}/{candidate_id} lacks retained-incumbent closure")
    debug = csv_rows.get("planner_p1_integrity_cost_debug.csv", [])
    if not metrics_only and not any(truthy(row.get("applied_to_objective")) for row in debug):
        errors.append("enabled P1 debug has no applied_to_objective=true")
    profile = csv_rows.get("planner_p1_accepted_trajectory_risk_profile.csv", [])
    if len(profile) < 200:
        errors.append("accepted profile has fewer than 200 samples")
    elif not metrics_only:
        sequence_values = [float(row.get("profile_seq"))
                           for row in profile if finite(row.get("profile_seq"))]
        latest_sequence = max(sequence_values) if sequence_values else None
        accepted_rows = [row for row in profile
                         if latest_sequence is not None and
                         finite(row.get("profile_seq")) and
                         float(row["profile_seq"]) == latest_sequence]
        accepted_values = [float(row["c_pi"]) for row in accepted_rows
                           if truthy(row.get("valid")) and
                           not truthy(row.get("stale")) and
                           finite(row.get("c_pi"))]
        accepted_identity = {
            (row.get("snapshot_generation_id"), row.get("planning_attempt_id"),
             row.get("candidate_id")) for row in accepted_rows
        }
        matching_winners = [row for row in candidate
            if truthy(row.get("selected")) and
            (row.get("snapshot_generation_id"), row.get("planning_attempt_id"),
             row.get("candidate_id")) in accepted_identity]
        if len(accepted_rows) != 200 or len(accepted_values) != 200 or \
                len(accepted_identity) != 1 or len(matching_winners) != 1:
            errors.append("authoritative accepted profile does not bind one selected fixed-200 candidate")
        else:
            winner = matching_winners[0]
            accepted_mean = sum(accepted_values) / len(accepted_values)
            accepted_max = max(accepted_values)
            seed_mean = float(winner["pre_mean_c_pi"])
            seed_max = float(winner["pre_max_c_pi"])
            if accepted_mean > seed_mean + 1e-12 or \
                    accepted_max > seed_max + 1e-12 or \
                    not (accepted_mean < seed_mean - 1e-12 or
                         accepted_max < seed_max - 1e-12):
                errors.append("authoritative accepted profile regresses selected P1 seed")
            if truthy(winner.get("incumbent_available")):
                incumbent_mean = float(winner["incumbent_mean_c_pi"])
                incumbent_max = float(winner["incumbent_max_c_pi"])
                if accepted_mean > incumbent_mean + 1e-12 or \
                        accepted_max > incumbent_max + 1e-12 or \
                        not (accepted_mean < incumbent_mean - 1e-12 or
                             accepted_max < incumbent_max - 1e-12):
                    errors.append("authoritative accepted profile no longer replaces incumbent")

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
