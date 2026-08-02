#!/usr/bin/env python3
"""Render an explicit, fail-closed P1 pre-admission diagnostic bundle.

This tool is deliberately diagnostic only.  It separates a successful base
trajectory publish from a P1 admission and makes missing candidate evidence a
visible failure rather than silently plotting an empty candidate chart.
"""

import argparse
import csv
import json
import math
import os
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCHEMA = "p1_evidence_provenance_v3"
FILES = {
    "timeline": "planner_p1_planning_context_timeline.csv",
    "attempt": "planner_p1_pre_admission_attempt.csv",
    "profile": "planner_p1_accepted_trajectory_risk_profile.csv",
}
REQUIRED_ATTEMPT_FIELDS = {
    "initial_duration_s", "initial_temporal_margin_s", "expected_sample_count",
    "matched_sample_count", "occupied_miss_count", "base_duration_s",
    "base_full_p1_support", "snapshot_time_max_s", "query_base_time_s",
}


def rows(path):
    if not path.is_file() or not path.stat().st_size:
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def number(row, name, default=math.nan):
    try:
        return float(row.get(name, default))
    except (TypeError, ValueError):
        return default


def truthy(value):
    return str(value).strip().lower() in {"1", "true"}


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def provenance_rows(records, run_id, manifest):
    return all(row.get("schema_version") == SCHEMA and row.get("run_id") == run_id
               and Path(row.get("manifest_path", "")).resolve() == manifest
               for row in records)


def read_map_points(bag_dir, errors):
    """Read one recorded obstacle cloud, returning an empty array on a real gap."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:
        errors.append(f"bag point-cloud decoder unavailable: {exc}")
        return np.empty((0, 3))
    target = "/drone_0_grid/grid_map/occupancy_inflate"
    fallback = "/map_generator/global_cloud"
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        topic = target if target in topic_types else fallback if fallback in topic_types else ""
        if not topic:
            errors.append("bag lacks occupancy-inflate and global-cloud topics")
            return np.empty((0, 3))
        msg_type = get_message(topic_types[topic])
        while reader.has_next():
            name, raw, _ = reader.read_next()
            if name != topic:
                continue
            msg = deserialize_message(raw, msg_type)
            values = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                                    skip_nans=True))
            # A map may contain millions of points; deterministic decimation
            # keeps the report legible and bounded without changing evidence.
            values = values[::max(1, len(values) // 15000)]
            return np.asarray(values, dtype=float).reshape((-1, 3))
        errors.append(f"bag topic {topic} has no decodable message")
    except Exception as exc:  # A malformed diagnostic bag must be visible.
        errors.append(f"bag point-cloud read failed: {exc}")
    return np.empty((0, 3))


def figure_topdown(path, map_points, profiles, attempts):
    fig, ax = plt.subplots(figsize=(8, 6))
    if len(map_points):
        ax.scatter(map_points[:, 0], map_points[:, 1], s=1, c="#9ca3af", alpha=.30,
                   label="recorded inflated occupancy")
    groups = defaultdict(list)
    for row in profiles:
        groups[row.get("profile_seq", "?")].append(row)
    for seq, group in groups.items():
        ax.plot([number(r, "x") for r in group], [number(r, "y") for r in group],
                lw=1.2, alpha=.85, label=f"published base trajectory {seq}")
    failed = [r for r in profiles if r.get("fallback_reason") in
              {"temporal_out_of_horizon", "coverage_insufficient"}]
    if failed:
        starts = [r for r in failed if number(r, "sample_index") == 0]
        ax.scatter([number(r, "x") for r in starts], [number(r, "y") for r in starts],
                   marker="x", c="#dc2626", label="published base starts with P1 rejection")
    ax.set(title="Recorded scenario: map, published base trajectories, P1 failures",
           xlabel="x (m)", ylabel="y (m)")
    ax.legend(fontsize=7, loc="best")
    save(fig, path)


def figure_funnel(path, timeline):
    stage = Counter(row.get("stage") for row in timeline)
    admission = [r for r in timeline if r.get("stage") == "p1_admission"]
    temporal = sum(r.get("reason") != "temporal_out_of_horizon" for r in admission)
    coverage = sum(r.get("reason") not in {"temporal_out_of_horizon", "coverage_insufficient"}
                   for r in admission)
    objective = sum(r.get("outcome") == "p1_objective" for r in admission)
    values = [stage["acquire"], len(admission), temporal, coverage, objective,
              stage["optimizer_start"], stage["publish"]]
    labels = ["acquire", "snapshot", "temporal", "coverage", "P1 admitted", "P1 optimizer", "publish"]
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.bar(labels, values, color=["#64748b", "#64748b", "#f59e0b", "#f59e0b", "#dc2626", "#dc2626", "#2563eb"])
    ax.set(title="P1 pre-admission funnel", ylabel="attempt/event count")
    ax.tick_params(axis="x", rotation=20)
    save(fig, path)


def figure_duration(path, attempts):
    fig, ax = plt.subplots(figsize=(9, 4))
    ids = [r.get("planning_attempt_id", "?") for r in attempts]
    initial = [number(r, "initial_duration_s") for r in attempts]
    base = [number(r, "base_duration_s") for r in attempts]
    remaining = [number(r, "snapshot_time_max_s") - number(r, "query_base_time_s") for r in attempts]
    ax.plot(ids, initial, "o-", label="initial duration")
    ax.plot(ids, base, "s-", label="base-optimized duration")
    ax.plot(ids, remaining, "^-", label="snapshot remaining horizon")
    ax.axhline(2.5, color="black", lw=.7, ls="--", label="2.5 s threshold")
    for index, row in enumerate(attempts):
        if row.get("p1_admission_reason") == "temporal_out_of_horizon":
            ax.scatter(ids[index], initial[index], c="#dc2626", s=80, zorder=3)
    ax.set(title="Initial/base duration versus immutable snapshot horizon", xlabel="planning attempt", ylabel="seconds")
    ax.legend(fontsize=8)
    save(fig, path)


def figure_coverage(path, profiles):
    by_attempt = defaultdict(dict)
    code = {"ok": 0, "temporal": 1, "occupied": 2, "stale": 3, "invalid": 4, "other": 5}
    for row in profiles:
        reason = row.get("reason", "other")
        if reason not in code: reason = "other"
        by_attempt[row.get("planning_attempt_id", "?")][int(number(row, "sample_index", 0))] = code[reason]
    attempt_ids = sorted(by_attempt, key=lambda value: int(value) if value.isdigit() else value)
    matrix = np.full((max(1, len(attempt_ids)), 200), np.nan)
    for i, attempt in enumerate(attempt_ids):
        for index, value in by_attempt[attempt].items():
            if 0 <= index < 200: matrix[i, index] = value
    fig, ax = plt.subplots(figsize=(10, max(2.5, .35 * len(attempt_ids))))
    cmap = plt.matplotlib.colors.ListedColormap(["#22c55e", "#f59e0b", "#dc2626", "#7c3aed", "#64748b", "#111827"])
    image = ax.imshow(matrix, aspect="auto", interpolation="nearest", cmap=cmap, vmin=0, vmax=5)
    ax.set(title="Fixed-200 profile reason heatmap", xlabel="sample index", ylabel="planning attempt",
           yticks=range(len(attempt_ids)), yticklabels=attempt_ids)
    colorbar = fig.colorbar(image, ax=ax, ticks=range(6)); colorbar.ax.set_yticklabels(list(code))
    save(fig, path)


def figure_occupied(path, map_points, profiles):
    occupied = [row for row in profiles if row.get("reason") == "occupied" and number(row, "sample_index") <= 5]
    fig, ax = plt.subplots(figsize=(7, 6))
    if occupied:
        x0, y0 = number(occupied[0], "x"), number(occupied[0], "y")
        local = map_points
        if len(map_points):
            local = map_points[np.hypot(map_points[:, 0] - x0, map_points[:, 1] - y0) < 2.0]
        if len(local): ax.scatter(local[:, 0], local[:, 1], s=5, c="#64748b", label="recorded inflated voxel cloud")
        ax.scatter([number(r, "x") for r in occupied], [number(r, "y") for r in occupied],
                   s=55, c="#dc2626", label="P0 occupied samples 0–5")
        for row in occupied:
            ax.annotate(str(row.get("sample_index")), (number(row, "x"), number(row, "y")))
        base = [truthy(r.get("base_collision_occupied")) for r in occupied]
        ax.set_title(f"Occupied-start close-up; base inflated-map predicate true={sum(base)}/{len(base)}")
    else:
        ax.text(.5, .5, "UNAVAILABLE: no recorded occupied samples 0–5", ha="center")
        ax.set_title("Occupied-start close-up")
    ax.set(xlabel="x (m)", ylabel="y (m)"); ax.legend(fontsize=8) if occupied else None
    save(fig, path)


def figure_base_lifecycle(path, timeline):
    starts = sum(r.get("stage") == "base_optimizer_start" for r in timeline)
    success = sum(r.get("stage") == "base_optimizer_end" and r.get("outcome") == "candidate_success" for r in timeline)
    failure = sum(r.get("stage") == "base_optimizer_end" and r.get("outcome") == "candidate_failure" for r in timeline)
    publish = sum(r.get("stage") == "publish" for r in timeline)
    fig, ax = plt.subplots(figsize=(7, 4)); ax.bar(["start", "success", "failure", "publish"], [starts, success, failure, publish], color="#2563eb")
    ax.set(title="Base optimizer lifecycle", ylabel="count")
    save(fig, path)


def figure_swimlane(path, timeline):
    lanes = ["acquire", "p1_admission", "base_optimizer_start", "base_optimizer_end", "optimizer_start", "optimizer_end", "publish"]
    lane = {value: index for index, value in enumerate(lanes)}
    stamps = [number(r, "stamp_s") for r in timeline if r.get("stage") in lane]
    origin = min(stamps) if stamps else 0.0
    fig, ax = plt.subplots(figsize=(10, 4))
    colors = {"p1_objective": "#16a34a", "base_fallback": "#dc2626", "candidate_success": "#2563eb", "candidate_failure": "#f59e0b", "published": "#2563eb"}
    for row in timeline:
        if row.get("stage") not in lane: continue
        ax.scatter(number(row, "stamp_s") - origin, lane[row["stage"]],
                   c=colors.get(row.get("outcome"), "#64748b"), s=25)
    ax.set(yticks=range(len(lanes)), yticklabels=lanes, title="P1/base lifecycle swimlane", xlabel="relative seconds")
    save(fig, path)


def figure_artifact(path, provenance):
    start = number(provenance, "process_start_epoch_s", 0.0); end = number(provenance, "process_end_epoch_s", start)
    fig, ax = plt.subplots(figsize=(9, 2.5))
    for label, stamp, color in (("launch/recorder start", start, "#2563eb"), ("launch/recorder end", end, "#16a34a")):
        ax.axvline(stamp - start, color=color, label=label)
    ax.scatter([max(0.0, end - start)], [0], c="#7c3aed", label="metadata + manifest finalize")
    ax.set(title="Artifact/provenance timeline", xlabel="relative seconds", yticks=[]); ax.legend(fontsize=8)
    save(fig, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, type=Path)
    parser.add_argument("--bag-dir", required=True, type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--main-report", type=Path, required=True)
    args = parser.parse_args()
    export, bag = args.export_dir.resolve(), args.bag_dir.resolve()
    output = (args.output_dir or export / "p1_pre_admission_diagnostic").resolve(); output.mkdir(parents=True, exist_ok=True)
    errors = []
    manifest_path = export / "test_planner_manifest.json"
    try: manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc: manifest = {}; errors.append(f"invalid manifest: {exc}")
    provenance = manifest.get("artifact_provenance", {})
    if provenance.get("schema_version") != SCHEMA or provenance.get("run_id") != args.run_id: errors.append("manifest identity mismatch")
    if Path(str(provenance.get("bag_path", ""))).resolve() != bag: errors.append("manifest bag path mismatch")
    if not bag.is_dir(): errors.append("bag directory missing")
    data = {name: rows(export / filename) for name, filename in FILES.items()}
    for name, record in data.items():
        if not record: errors.append(f"missing {name} artifact")
        elif not provenance_rows(record, args.run_id, manifest_path): errors.append(f"{name} provenance mismatch")
    if data["attempt"] and not REQUIRED_ATTEMPT_FIELDS.issubset(data["attempt"][0]): errors.append("attempt artifact lacks required fields")
    map_errors = []; map_points = read_map_points(bag, map_errors) if bag.is_dir() else np.empty((0, 3))
    # A missing map cloud is reportable but does not turn a well-formed
    # evidence bundle into a candidate-success result.
    errors.extend(map_errors)
    paths = [
        ("Full recorded scenario top-down", "pre_admission_topdown.png", figure_topdown, (map_points, data["profile"], data["attempt"])),
        ("P1 admission funnel", "pre_admission_funnel.png", figure_funnel, (data["timeline"],)),
        ("Duration versus snapshot horizon", "pre_admission_duration.png", figure_duration, (data["attempt"],)),
        ("Coverage reason heatmap", "pre_admission_coverage.png", figure_coverage, (data["profile"],)),
        ("Occupied-start close-up", "pre_admission_occupied_start.png", figure_occupied, (map_points, data["profile"])),
        ("Base optimizer lifecycle", "pre_admission_base_lifecycle.png", figure_base_lifecycle, (data["timeline"],)),
        ("P1 lifecycle swimlane", "pre_admission_swimlane.png", figure_swimlane, (data["timeline"],)),
        ("Artifact/provenance timeline", "pre_admission_provenance.png", figure_artifact, (provenance,)),
    ]
    for _, filename, draw, values in paths: draw(output / filename, *values)
    candidate_path = export / "planner_p1_candidate_optimization.csv"
    candidate_rows = rows(candidate_path)
    entry = bool(candidate_rows) and any(r.get("stage") == "optimizer_start" for r in data["timeline"])
    if not entry: errors.append("ENTRY FAIL: no P1 optimizer/candidate evidence; do not run formal pair")
    runtime = provenance.get("runtime_paths", {})
    hashes = ", ".join(f"{name}={item.get('sha256', 'unknown')}" for name, item in runtime.items() if isinstance(item, dict)) or "unknown"
    status = "PASS" if not errors else "FAIL"
    metadata = (f"Run ID: `{args.run_id}`; HEAD: `{provenance.get('git_commit', 'unknown')}`; runtime hashes: `{hashes}`; "
                f"export: `{export}`; bag: `{bag}`; attempts/generations/query-base: `" +
                "; ".join(f"{r.get('planning_attempt_id')}/{r.get('snapshot_generation_id')}/{r.get('query_base_time_s')}" for r in data["attempt"]) +
                f"`; time window: `{provenance.get('process_start_stamp_utc', 'unknown')} to {provenance.get('process_end_stamp_utc', 'unknown')}`; diagnostic (non-authoritative).")
    report = output / "p1_pre_admission_diagnostic.md"
    lines = ["# P1 pre-admission diagnostic smoke", "", metadata, "", f"Diagnostic status: {status}.", *[f"- {error}" for error in errors]]
    report.write_text("\n".join(lines) + "\n")
    with args.main_report.resolve().open("a") as handle:
        handle.write("\n## 2026-08-02 P1 pre-admission diagnostic figures (non-authoritative)\n\n")
        for title, filename, _, _ in paths:
            image = Path(os.path.relpath(output / filename, args.main_report.resolve().parent))
            if title == "P1 admission funnel": observation = f"{len(data['attempt'])} instrumented rows; candidate rows={len(candidate_rows)}."
            elif title == "Duration versus snapshot horizon": observation = "Initial, base-optimized, and immutable remaining-horizon durations are plotted per attempt."
            elif title == "Occupied-start close-up": observation = f"Recorded occupied samples 0–5={sum(r.get('reason') == 'occupied' and number(r, 'sample_index') <= 5 for r in data['profile'])}."
            else: observation = f"Rendered directly from the explicit bundle; {len(data['timeline'])} lifecycle rows and {len(data['profile'])} profile samples."
            conclusion = ("It confirms base planning partially succeeded but strict P1 pre-admission did not produce candidate evidence."
                          if title in {"P1 admission funnel", "Base optimizer lifecycle", "P1 lifecycle swimlane"} else
                          "It supports pre-admission root-cause diagnosis only; it cannot establish P1-2 effectiveness.")
            handle.write(f"### {title}\n\n![{title}]({image})\n\nObservation: {observation}\n\n{metadata}\n\nVerdict: {status if title != 'Occupied-start close-up' or map_points.size else 'UNAVAILABLE'}.\n\nConclusion: {conclusion}\n\n")
    print(report)
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
