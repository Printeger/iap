#!/usr/bin/env python3
"""Fail-closed, explicit-bundle P1 candidate-generation diagnostic smoke.

This is deliberately separate from the formal P1-2 analyzer.  It never
grants progression: its job is to make candidate fan-out, retained incumbent,
and objective/admission disagreement inspectable for one named fresh run.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCHEMA = "p1_evidence_provenance_v3"
NAMES = {
    "candidate": "planner_p1_candidate_optimization.csv",
    "decision": "planner_p1_replacement_decision.csv",
    "profile": "planner_p1_candidate_retained_profile.csv",
    "timeline": "planner_p1_planning_context_timeline.csv",
}


def rows(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def num(row, key, default=math.nan):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return default


def truthy(value):
    return str(value).strip().lower() in {"1", "true"}


def verify_identity(records, run_id, manifest):
    return all(row.get("schema_version") == SCHEMA and row.get("run_id") == run_id
               and Path(row.get("manifest_path", "")).resolve() == manifest
               for row in records)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, type=Path)
    parser.add_argument("--bag-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    export_dir, bag_dir = args.export_dir.resolve(), args.bag_dir.resolve()
    out_dir = (args.output_dir or export_dir / "p1_candidate_diagnostic_smoke").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = export_dir / "test_planner_manifest.json"
    errors = []
    if not bag_dir.is_dir(): errors.append(f"missing bag: {bag_dir}")
    if not manifest_path.is_file(): errors.append(f"missing manifest: {manifest_path}")
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    provenance = manifest.get("artifact_provenance", {})
    if provenance.get("schema_version") != SCHEMA or provenance.get("run_id") != args.run_id:
        errors.append("manifest schema/run-id does not match explicit bundle")
    if Path(str(provenance.get("bag_path", ""))).resolve() != bag_dir:
        errors.append("manifest bag path does not match explicit bag")
    data = {}
    for key, name in NAMES.items():
        path = export_dir / name
        if not path.is_file() or not path.stat().st_size:
            if key in {"decision", "profile"}:
                data[key] = []
                continue
            errors.append(f"missing artifact: {path}")
            data[key] = []
            continue
        data[key] = rows(path)
        if not verify_identity(data[key], args.run_id, manifest_path):
            errors.append(f"provenance mismatch: {path.name}")

    candidates = data["candidate"]
    decisions, profiles, timeline = data["decision"], data["profile"], data["timeline"]
    selected = [r for r in candidates if truthy(r.get("selected"))]
    if not candidates: errors.append("candidate artifact has no rows")
    selected_by_attempt = {}
    for row in selected:
        selected_by_attempt.setdefault(row.get("planning_attempt_id"), []).append(row)
    if not selected_by_attempt or any(len(value) != 1 for value in selected_by_attempt.values()):
        errors.append("candidate artifact lacks exactly one optimizer-selected row per attempt")
    retained = [r for r in selected if not truthy(r.get("replacement_accepted"))]
    if retained:
        if not decisions:
            errors.append("retained optimizer selection has no replacement-decision artifact")
        if not profiles:
            errors.append("retained optimizer selection has no comparison profile")
        for candidate in retained:
            matching_decisions = [r for r in decisions
                                  if r.get("planning_attempt_id") == candidate.get("planning_attempt_id")
                                  and r.get("optimizer_selected_candidate_id") == candidate.get("candidate_id")]
            if len(matching_decisions) != 1:
                errors.append("retained optimizer selection does not have one matching replacement decision")
                continue
            decision = matching_decisions[0]
            if truthy(decision.get("replacement_accepted")) or decision.get("final_trajectory_source") != "retained_incumbent":
                errors.append("replacement decision does not identify retained incumbent final source")
            matching = [r for r in profiles
                        if r.get("planning_attempt_id") == decision.get("planning_attempt_id")
                        and r.get("candidate_id") == decision.get("optimizer_selected_candidate_id")]
            candidate_samples = [r for r in matching if r.get("trajectory_role") == "optimizer_selected_candidate"]
            incumbent_samples = [r for r in matching if r.get("trajectory_role") == "retained_incumbent"]
            if len(candidate_samples) != 200 or len(incumbent_samples) != 200:
                errors.append("retained decision does not have paired 200-sample profiles")
            if any(r.get("final_trajectory_source") != "retained_incumbent" for r in matching):
                errors.append("retained comparison profile does not identify incumbent final source")

    # 1: top-down scene
    fig, ax = plt.subplots(figsize=(7, 5))
    for role, color in (("optimizer_selected_candidate", "#dc2626"),
                        ("retained_incumbent", "#2563eb")):
        group = [r for r in profiles if r.get("trajectory_role") == role]
        if group:
            ax.plot([num(r, "x") for r in group], [num(r, "y") for r in group],
                    color=color, label=role)
    ax.set(title="P1 diagnostic top-down scene", xlabel="x (m)", ylabel="y (m)")
    ax.legend() if profiles else ax.text(.5, .5, "no retained comparison", ha="center")
    save(fig, out_dir / "p1_diag_topdown_scene.png")

    # 2: funnel
    fig, ax = plt.subplots(figsize=(7, 4))
    latest = candidates[-1] if candidates else {}
    counts = [num(latest, "fanout_input_segments", 0), num(latest, "fanout_returned_count", 0),
              num(latest, "fanout_optimizer_successes", 0), num(latest, "fanout_full_support", 0),
              num(latest, "fanout_p1_descent_eligible", 0)]
    ax.bar(["segments", "generated", "optimized", "support", "eligible"], counts, color="#7c3aed")
    ax.set(title="P1 candidate fan-out funnel", ylabel="count")
    save(fig, out_dir / "p1_diag_fanout_funnel.png")

    # 3: fixed lattice deltas
    fig, ax = plt.subplots(figsize=(6, 5))
    for row in candidates:
        x = num(row, "post_mean_c_pi") - num(row, "pre_mean_c_pi")
        y = num(row, "post_max_c_pi") - num(row, "pre_max_c_pi")
        ax.scatter(x, y, c="#16a34a" if truthy(row.get("rank_eligible")) else "#dc2626")
        ax.annotate(row.get("candidate_id", "?"), (x, y))
    ax.axhline(0, color="black", lw=.7); ax.axvline(0, color="black", lw=.7)
    ax.set(title="Fixed-200 mean/max delta", xlabel="Δ mean c_pi", ylabel="Δ max c_pi")
    save(fig, out_dir / "p1_diag_mean_max_delta_scatter.png")

    # 4: paired profile
    fig, ax = plt.subplots(figsize=(8, 4))
    for role, color in (("optimizer_selected_candidate", "#dc2626"), ("retained_incumbent", "#2563eb")):
        group = [r for r in profiles if r.get("trajectory_role") == role]
        if group: ax.plot([num(r, "sample_index") for r in group], [num(r, "c_pi") for r in group], color=color, label=role)
    ax.set(title="Fixed-200 candidate / retained-incumbent profile", xlabel="sample", ylabel="c_pi")
    ax.legend() if profiles else ax.text(.5, .5, "not retained", ha="center")
    save(fig, out_dir / "p1_diag_profile_comparison.png")

    # 5: objective vs gate
    fig, ax = plt.subplots(figsize=(8, 4))
    ids = [r.get("candidate_id", "?") for r in candidates]
    ax.plot(ids, [num(r, "post_raw_p1_cost") - num(r, "pre_raw_p1_cost") for r in candidates], "o-", label="objective Δ")
    ax.plot(ids, [num(r, "post_mean_c_pi") - num(r, "pre_mean_c_pi") for r in candidates], "s-", label="gate mean Δ")
    ax.plot(ids, [num(r, "post_max_c_pi") - num(r, "pre_max_c_pi") for r in candidates], "^-", label="gate max Δ")
    ax.axhline(0, color="black", lw=.7); ax.legend(); ax.set(title="Objective / admission-gate alignment", xlabel="candidate")
    save(fig, out_dir / "p1_diag_objective_gate_alignment.png")

    # 6: lifecycle swimlane
    fig, ax = plt.subplots(figsize=(9, 4))
    stages = {stage: index for index, stage in enumerate(sorted({r.get("stage", "") for r in timeline}))}
    for row in timeline:
        ax.scatter(num(row, "stamp_s"), stages[row.get("stage", "")], c="#2563eb")
    ax.set_yticks(list(stages.values()), list(stages.keys())); ax.set(title="P1 lifecycle swimlane", xlabel="stamp (s)")
    save(fig, out_dir / "p1_diag_lifecycle_swimlane.png")

    status = "PASS (diagnostic only)" if not errors else "FAIL"
    report = out_dir / "p1_candidate_diagnostic_smoke.md"
    report.write_text("\n".join([
        "# P1 candidate diagnostic smoke", "",
        f"Observation: {len(candidates)} candidate rows, {len(retained)} retained decisions.",
        f"Verdict: {status}.",
        "Conclusion: diagnostic evidence never satisfies P1-2 effectiveness or progression.", "",
        f"- Run ID: `{args.run_id}`", f"- Export: `{export_dir}`", f"- Bag: `{bag_dir}`",
        f"- Time window: {provenance.get('process_start_stamp_utc', 'unknown')} to {provenance.get('process_end_stamp_utc', 'unknown')}",
        f"- Diagnostic status: {status}", *(f"- Error: {error}" for error in errors), "",
    ]))
    print(report)
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
