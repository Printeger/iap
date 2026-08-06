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
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCHEMA = "p1_evidence_provenance_v4"
NAMES = {
    "candidate": "planner_p1_candidate_optimization.csv",
    "control_points": "planner_p1_candidate_control_points.csv",
    "candidate_profile": "planner_p1_candidate_profile.csv",
    "pairwise": "planner_p1_candidate_pairwise.csv",
    "checkpoint": "planner_p1_optimizer_checkpoint.csv",
    "occupancy": "planner_p0_occupancy_query_evidence.csv",
    "decision": "planner_p1_replacement_decision.csv",
    "retained_profile": "planner_p1_candidate_retained_profile.csv",
    "accepted_profile": "planner_p1_accepted_trajectory_risk_profile.csv",
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


def plot_lifecycle(ax, timeline):
    """Render any timeline size with one vectorized scatter collection."""
    stages = {stage: index for index, stage in
              enumerate(sorted({row.get("stage", "") for row in timeline}))}
    if timeline:
        ax.scatter([num(row, "stamp_s") for row in timeline],
                   [stages[row.get("stage", "")] for row in timeline],
                   c="#2563eb", s=8)
        ax.set_yticks(list(stages.values()), list(stages.keys()))
    else:
        ax.axis("off")
        ax.text(.5, .5, "UNAVAILABLE", ha="center")
    ax.set(title="P1 lifecycle swimlane", xlabel="stamp (s)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, type=Path)
    parser.add_argument("--bag-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--main-report", type=Path,
                        help="append per-figure diagnostic evidence to this primary report")
    args = parser.parse_args()
    export_dir, bag_dir = args.export_dir.resolve(), args.bag_dir.resolve()
    out_dir = (args.output_dir or export_dir / "p1_candidate_diagnostic_smoke").resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = export_dir / "test_planner_manifest.json"
    errors = []
    if not bag_dir.is_dir(): errors.append(f"missing bag: {bag_dir}")
    if not manifest_path.is_file(): errors.append(f"missing manifest: {manifest_path}")
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            manifest = {}
            errors.append(f"invalid manifest: {manifest_path}: {exc}")
    else:
        manifest = {}
    provenance = manifest.get("artifact_provenance", {})
    if provenance.get("schema_version") != SCHEMA or provenance.get("run_id") != args.run_id:
        errors.append("manifest schema/run-id does not match explicit bundle")
    if Path(str(provenance.get("bag_path", ""))).resolve() != bag_dir:
        errors.append("manifest bag path does not match explicit bag")
    data = {}
    for key, name in NAMES.items():
        path = export_dir / name
        if not path.is_file() or not path.stat().st_size:
            if key in {"decision", "retained_profile"}:
                data[key] = []
                continue
            errors.append(f"missing artifact: {path}")
            data[key] = []
            continue
        data[key] = rows(path)
        if not verify_identity(data[key], args.run_id, manifest_path):
            errors.append(f"provenance mismatch: {path.name}")

    candidates = data["candidate"]
    decisions = data["decision"]
    profiles = data["retained_profile"]
    candidate_profiles = data["candidate_profile"]
    pairwise = data["pairwise"]
    occupancy = data["occupancy"]
    timeline = data["timeline"]
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
            incumbent_available = truthy(candidate.get("incumbent_available"))
            expected_source = ("retained_incumbent" if incumbent_available
                               else "no_publish_no_incumbent")
            matching_decisions = [r for r in decisions
                                  if r.get("planning_attempt_id") == candidate.get("planning_attempt_id")
                                  and r.get("optimizer_selected_candidate_id") == candidate.get("candidate_id")
                                  and r.get("snapshot_generation_id") == candidate.get("snapshot_generation_id")
                                  and r.get("query_base_time_s") == candidate.get("query_base_time_s")]
            if len(matching_decisions) != 1:
                errors.append("retained optimizer selection does not have one matching replacement decision")
                continue
            decision = matching_decisions[0]
            if truthy(decision.get("replacement_accepted")) or decision.get("final_trajectory_source") != expected_source:
                errors.append("replacement decision does not identify retained incumbent final source")
            expected_publish_identity = (f"incumbent:{decision.get('incumbent_trajectory_id', '')}"
                                         if incumbent_available else "none")
            if decision.get("publish_identity") != expected_publish_identity:
                errors.append("retained replacement decision does not publish the incumbent identity")
            matching = [r for r in profiles
                        if r.get("planning_attempt_id") == decision.get("planning_attempt_id")
                        and r.get("candidate_id") == decision.get("optimizer_selected_candidate_id")
                        and r.get("snapshot_generation_id") == decision.get("snapshot_generation_id")
                        and r.get("query_base_time_s") == decision.get("query_base_time_s")]
            candidate_samples = [r for r in matching if r.get("trajectory_role") == "optimizer_selected_candidate"]
            incumbent_samples = [r for r in matching if r.get("trajectory_role") == "retained_incumbent"]
            if len(candidate_samples) != 200 or len(incumbent_samples) != (200 if incumbent_available else 0):
                errors.append("retained decision does not have paired 200-sample profiles")
            if any(r.get("final_trajectory_source") != expected_source for r in matching):
                errors.append("retained comparison profile does not identify incumbent final source")
            rejected_tuple = (candidate.get("planning_attempt_id"), candidate.get("candidate_id"),
                              candidate.get("snapshot_generation_id"), candidate.get("query_base_time_s"))
            for published in data["accepted_profile"]:
                published_tuple = (published.get("planning_attempt_id"), published.get("candidate_id"),
                                   published.get("snapshot_generation_id"), published.get("query_base_time_s"))
                if published_tuple == rejected_tuple:
                    errors.append("rejected candidate appears in accepted-profile publish identity")
                if (published.get("trajectory_id") == decision.get("incumbent_trajectory_id") and
                    published.get("final_trajectory_source") == "retained_incumbent"):
                    break

    # 1: top-down scene
    fig, ax = plt.subplots(figsize=(7, 5))
    for phase, color in (("initial", "#f97316"), ("final", "#2563eb")):
        group = [r for r in candidate_profiles
                 if r.get("phase") == phase and truthy(r.get("valid"))]
        if group:
            ax.plot([num(r, "x") for r in group], [num(r, "y") for r in group],
                    color=color, label=phase)
    ax.set(title="P1 diagnostic top-down scene", xlabel="x (m)", ylabel="y (m)")
    if candidate_profiles:
        ax.legend()
    else:
        ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_topdown_scene.png")

    # 2: funnel
    fig, ax = plt.subplots(figsize=(7, 4))
    latest = candidates[-1] if candidates else {}
    counts = [num(latest, "fanout_input_segments", 0), num(latest, "fanout_returned_count", 0),
              num(latest, "fanout_optimizer_successes", 0), num(latest, "fanout_full_support", 0),
              num(latest, "fanout_p1_descent_eligible", 0)]
    ax.bar(["segments", "generated", "optimized", "support", "eligible"], counts, color="#7c3aed")
    ax.set(title="P1 candidate fan-out funnel", ylabel="count")
    if not candidates:
        ax.clear(); ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_fanout_funnel.png")

    # 3: fixed lattice deltas; attempt markers prevent cross-attempt confusion.
    fig, ax = plt.subplots(figsize=(6, 5))
    markers = ("o", "s", "^", "D", "P", "X")
    attempts = sorted({row.get("planning_attempt_id", "") for row in candidates})
    for row in candidates:
        x = num(row, "post_mean_c_pi") - num(row, "pre_mean_c_pi")
        y = num(row, "post_max_c_pi") - num(row, "pre_max_c_pi")
        attempt_index = attempts.index(row.get("planning_attempt_id", ""))
        color = plt.cm.tab10(attempt_index % 10)
        ax.scatter(x, y, marker=markers[attempt_index % len(markers)], c=[color],
                   edgecolors="black" if truthy(row.get("selected")) else "none", s=70)
        ax.annotate(f"{row.get('planning_attempt_id','?')}/{row.get('candidate_id','?')}", (x, y))
    ax.axhline(0, color="black", lw=.7); ax.axvline(0, color="black", lw=.7)
    ax.axvspan(-1e9, 0, ymax=.5, color="#dcfce7", alpha=.25)
    ax.set(title="Fixed-200 mean/max delta", xlabel="Δ mean c_pi", ylabel="Δ max c_pi")
    if not candidates:
        ax.clear(); ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_mean_max_delta_scatter.png")

    # 4: initial/final control-point and risk-profile pairwise matrices.
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    pair_ids = sorted({row.get("candidate_id_a", "") for row in pairwise} |
                      {row.get("candidate_id_b", "") for row in pairwise})
    for column, phase in enumerate(("initial", "final")):
        phase_rows = [row for row in pairwise if row.get("phase") == phase]
        for axis, field, title in (
                (axes[0, column], "control_point_distance", "control-point distance"),
                (axes[1, column], "risk_profile_distance", "risk-profile distance")):
            if pair_ids and phase_rows:
                matrix = [[math.nan for _ in pair_ids] for _ in pair_ids]
                positions = {value: index for index, value in enumerate(pair_ids)}
                for row in phase_rows:
                    a, b = positions[row["candidate_id_a"]], positions[row["candidate_id_b"]]
                    matrix[a][b] = matrix[b][a] = num(row, field)
                image = axis.imshow(matrix, cmap="viridis")
                fig.colorbar(image, ax=axis, fraction=.046)
                axis.set_xticks(range(len(pair_ids)), pair_ids)
                axis.set_yticks(range(len(pair_ids)), pair_ids)
            else:
                axis.axis("off")
                axis.text(.5, .5, "UNAVAILABLE", ha="center", va="center")
            axis.set_title(f"{phase} {title}")
    save(fig, out_dir / "p1_diag_pairwise_matrices.png")

    # 5: every candidate's deterministic fixed-200 initial/final profile.
    fig, ax = plt.subplots(figsize=(8, 4))
    for candidate_id in sorted({row.get("candidate_id", "") for row in candidate_profiles}):
        for phase, style in (("initial", "--"), ("final", "-")):
            group = [row for row in candidate_profiles
                     if row.get("candidate_id") == candidate_id and
                     row.get("phase") == phase and truthy(row.get("valid"))]
            if group:
                ax.plot([num(row, "sample_index") for row in group],
                        [num(row, "c_pi") for row in group], linestyle=style,
                        label=f"{candidate_id}/{phase}")
    ax.set(title="Per-candidate fixed-200 risk profiles", xlabel="sample", ylabel="c_pi")
    if candidate_profiles:
        ax.legend(fontsize=7, ncol=2)
    else:
        ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_profile_comparison.png")

    # 5: gradient/displacement direction against the fixed-lattice gate.
    fig, ax = plt.subplots(figsize=(8, 4))
    ids = [r.get("candidate_id", "?") for r in candidates]
    ax.plot(ids, [num(r, "grad_integrity_dot_displacement") for r in candidates], "o-", label="raw P1 gradient · displacement")
    ax.plot(ids, [num(r, "total_gradient_dot_displacement") for r in candidates], "d-", label="total gradient · displacement")
    ax.plot(ids, [num(r, "post_mean_c_pi") - num(r, "pre_mean_c_pi") for r in candidates], "s-", label="gate mean Δ")
    ax.plot(ids, [num(r, "post_max_c_pi") - num(r, "pre_max_c_pi") for r in candidates], "^-", label="gate max Δ")
    ax.axhline(0, color="black", lw=.7); ax.legend(); ax.set(title="Gradient / displacement and admission-gate alignment", xlabel="candidate")
    if not candidates:
        ax.clear(); ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_gradient_displacement.png")

    # 7: objective decomposition is intentionally per-attempt, never linked across attempts.
    fig, axes = plt.subplots(max(1, len(attempts)), 1, figsize=(8, 3 * max(1, len(attempts))), squeeze=False)
    fields = (("base", "pre_base_objective", "post_base_objective"),
              ("raw P1", "pre_raw_p1_cost", "post_raw_p1_cost"),
              ("normalized P1", "pre_normalized_p1_cost", "post_normalized_p1_cost"),
              ("anchor", "pre_anchor_cost", "post_anchor_cost"),
              ("total", "pre_total_objective", "post_total_objective"))
    for axis, attempt in zip(axes[:, 0], attempts):
        group = [r for r in candidates if r.get("planning_attempt_id") == attempt]
        labels = [r.get("candidate_id", "?") for r in group]
        for offset, (label, pre, post) in enumerate(fields):
            axis.plot(labels, [num(r, pre) for r in group], marker="o", label=f"{label} pre")
            axis.plot(labels, [num(r, post) for r in group], marker="x", linestyle="--", label=f"{label} post")
        axis.set_title(f"Attempt {attempt}: objective decomposition")
        axis.legend(ncol=2, fontsize=7)
    if not attempts:
        axes[0, 0].axis("off")
        axes[0, 0].text(.5, .5, "UNAVAILABLE", ha="center")
    save(fig, out_dir / "p1_diag_objective_decomposition.png")

    # 8: P0 interpolation-corner attribution, separate from query points.
    fig, ax = plt.subplots(figsize=(8, 6))
    if occupancy:
        query_rows = occupancy[::max(1, len(occupancy) // 1000)]
        ax.scatter([num(row, "query_x") for row in query_rows],
                   [num(row, "query_y") for row in query_rows], s=5,
                   color="#2563eb", alpha=.35, label="query points")
        corner_rows = [row for row in occupancy
                       if truthy(row.get("inflated_occupied"))]
        ax.scatter([num(row, "corner_x") for row in corner_rows],
                   [num(row, "corner_y") for row in corner_rows], s=25,
                   color="#dc2626", label="inflated occupied corners")
        ax.legend()
    else:
        ax.axis("off"); ax.text(.5, .5, "UNAVAILABLE", ha="center")
    ax.set(title="P0 query points and interpolation-corner occupancy",
           xlabel="x (m)", ylabel="y (m)")
    save(fig, out_dir / "p1_diag_p0_occupancy_overlay.png")

    # 6: lifecycle swimlane
    fig, ax = plt.subplots(figsize=(9, 4))
    plot_lifecycle(ax, timeline)
    save(fig, out_dir / "p1_diag_lifecycle_swimlane.png")

    # 9: artifact/provenance timeline.
    fig, ax = plt.subplots(figsize=(9, 3))
    events = [("launch start", provenance.get("process_start_epoch_s")),
              ("launch end", provenance.get("process_end_epoch_s"))]
    for label, stamp in events:
        if stamp is not None:
            ax.axvline(float(stamp), label=label)
    ax.set(title="Artifact / provenance timeline", xlabel="epoch seconds")
    ax.legend() if any(stamp is not None for _, stamp in events) else ax.text(.5, .5, "timestamps unavailable", ha="center")
    save(fig, out_dir / "p1_diag_artifact_provenance_timeline.png")

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
    if args.main_report:
        main_report = args.main_report.resolve()
        main_report.parent.mkdir(parents=True, exist_ok=True)
        runtime = provenance.get("runtime_paths", {})
        runtime_hashes = ", ".join(
            f"{name}={value.get('sha256', 'unknown')}" for name, value in runtime.items()
            if isinstance(value, dict)) or "unknown"
        metadata = (
            f"Run ID: `{args.run_id}`; HEAD: `{provenance.get('git_commit', 'unknown')}`; "
            f"runtime hashes: `{runtime_hashes}`; export: `{export_dir}`; bag: `{bag_dir}`; "
            f"attempt/candidate/generation/query-base: `"
            f"{','.join('/'.join(str(row.get(k, '?')) for k in ('planning_attempt_id', 'candidate_id', 'snapshot_generation_id', 'query_base_time_s')) for row in candidates) or 'none'}`; "
            f"snapshot/query-base window: `{provenance.get('process_start_stamp_utc', 'unknown')} to {provenance.get('process_end_stamp_utc', 'unknown')}`; "
            "diagnostic (non-authoritative).")
        empty = not candidates
        candidate_observation = (
            "No candidate trajectory was emitted; candidate-specific charts are unavailable."
            if empty else
            f"{len(candidates)} fixed-200 candidate rows were emitted across {len(attempts)} attempts.")
        lifecycle_conclusion = (
            "The lifecycle contains P1 optimizer evidence, but this diagnostic remains non-authoritative."
            if candidates else
            "Base planning partially succeeded, but no strict P1 candidate was emitted.")
        figures = [
            ("Full scenario top-down", "p1_diag_topdown_scene.png", candidate_observation),
            ("Fan-out funnel", "p1_diag_fanout_funnel.png", candidate_observation),
            ("Mean/max delta scatter", "p1_diag_mean_max_delta_scatter.png", "No point can meet an effectiveness gate when candidate rows are absent." if empty else "Fixed-200 pre/post mean and max deltas are plotted per attempt/candidate."),
            ("Initial/final pairwise matrices", "p1_diag_pairwise_matrices.png", "Pairwise evidence is unavailable." if not pairwise else "Initial/final control-point and fixed-200 risk-profile distance matrices are shown."),
            ("Objective decomposition", "p1_diag_objective_decomposition.png", "No optimizer objective row was emitted." if empty else "Pre/post objective components are shown separately for each attempt."),
            ("Gradient/displacement", "p1_diag_gradient_displacement.png", "Gradient/gate direction is unassessed without candidate rows." if empty else "Raw/total gradient-to-displacement and fixed-lattice deltas are shown."),
            ("Per-attempt candidate profile", "p1_diag_profile_comparison.png", "Candidate profiles are unavailable." if not candidate_profiles else "Every candidate initial/final fixed-200 profile is plotted."),
            ("P0 occupied/base-collision overlay", "p1_diag_p0_occupancy_overlay.png", "P0 corner evidence is unavailable." if not occupancy else "Query points are plotted separately from the interpolation corners that own occupancy attribution."),
            ("Lifecycle swimlane", "p1_diag_lifecycle_swimlane.png", "Lifecycle events are shown only from the explicit run timeline."),
            ("Artifact/provenance timeline", "p1_diag_artifact_provenance_timeline.png", "Recorder and launch bounds are shown from manifest provenance."),
        ]
        run_date = str(provenance.get("process_start_stamp_utc", "unknown"))[:10]
        fragment = [
            f"\n## {run_date} diagnostic-smoke figure evidence (non-authoritative)\n\n"]
        for title, filename, observation in figures:
            image = Path(os.path.relpath(out_dir / filename, main_report.parent))
            conclusion = lifecycle_conclusion if title == "Lifecycle swimlane" else (
                "This figure is diagnostic only and cannot establish P1-2 effectiveness or progression.")
            fragment.append(
                f"### {title}\n\n![{title}]({image})\n\nObservation: {observation}\n\n"
                f"{metadata}\n\nVerdict: {status}.\n\nConclusion: {conclusion}\n\n")
        fragment.append(
            "### Diagnostic terminal record\n\n"
            f"Observation: Analyzer completed once for run `{args.run_id}`.\n\n"
            f"Verdict: {status}.\n\n"
            "Conclusion: This terminal record supersedes no other run and does not grant formal progression.\n")
        with main_report.open("a") as handle:
            handle.write("".join(fragment))
    print(report)
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
