#!/usr/bin/env python3
"""Build a read-only P1-2 historical archive without invoking formal analysis.

This tool only reads retained compact c31--c38 artifacts and the retained c38
raw campaign.  It does not import, execute, or emulate the formal analyzer.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import io
import json
import math
import shutil
import sys
import tempfile
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


CAMPAIGNS = tuple(range(31, 39))
COMPLETE_CAMPAIGNS = {31, 32, 38}
OLD_MEAN_THRESHOLD = 0.00836
OLD_CVAR_THRESHOLD = 0.00677
FIGURES = (
    "p1_2_campaign_completeness_overview.png",
    "p1_2_primary_threshold_comparison.png",
    "c38_pair_metric_dashboard.png",
    "c38_same_snapshot_mechanism.png",
    "c38_primary_trajectory_risk_profiles.png",
)
RUN_REQUIRED = {
    "run_id", "scenario", "metrics_only", "passed", "selected_lane",
    "mean", "cvar", "max", "path_length_m", "localization_error_m",
    "checkpoint_truth_dt_s", "export_dir",
}
PAIR_REQUIRED = {
    "kind", "reference_run_id", "enabled_run_id", "passed",
    "mean_improvement", "cvar_improvement", "max_regression", "path_growth",
    "localization_error_delta_m",
}
OPTIMIZATION_REQUIRED = {
    "run_id", "planning_attempt_id", "candidate_id", "snapshot_generation_id",
    "query_base_time_s", "pre_raw_p1_cost", "post_raw_p1_cost",
    "pre_mean_c_pi", "post_mean_c_pi", "pre_max_c_pi", "post_max_c_pi",
    "grad_integrity_dot_displacement", "support_full_valid",
    "objective_applied", "optimization_success", "initial_control_points_hash",
    "final_control_points_hash", "support_signature",
}
PROFILE_REQUIRED = {
    "run_id", "profile_seq", "planning_attempt_id", "candidate_id",
    "snapshot_generation_id", "query_base_time_s", "sample_index",
    "arc_fraction", "x", "y", "z", "valid", "stale", "c_pi",
}
MECHANISM_FIELDS = (
    "record_type", "pair_index", "pair_kind", "role", "run_id",
    "planning_attempt_id", "candidate_id", "snapshot_generation_id",
    "query_base_time_s", "sample_index", "arc_fraction", "x", "y", "z",
    "c_pi", "gradient_dot_displacement", "delta_c_pi", "pre_raw_p1_cost",
    "post_raw_p1_cost", "raw_p1_cost_delta", "pre_mean_c_pi",
    "post_mean_c_pi", "mean_c_pi_delta", "pre_max_c_pi", "post_max_c_pi",
    "max_c_pi_delta", "initial_control_points_hash",
    "final_control_points_hash", "support_signature", "objective_applied",
)


class ArchiveError(ValueError):
    pass


def _truthy(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _number(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ArchiveError(f"{label} is not numeric: {value!r}") from exc
    if not math.isfinite(result):
        raise ArchiveError(f"{label} is not finite: {value!r}")
    return result


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ArchiveError(f"unreadable JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ArchiveError(f"JSON root must be an object: {path}")
    return value


def _plain_or_gzip(parent: Path, stem: str, label: str) -> Path:
    plain = parent / f"{stem}.csv"
    compressed = parent / f"{stem}.csv.gz"
    matches = [path for path in (plain, compressed) if path.is_file()]
    if len(matches) != 1:
        raise ArchiveError(
            f"{label}: expected exactly one of {plain.name} or {compressed.name}; "
            f"found {len(matches)}"
        )
    return matches[0]


def _open_csv(path: Path):
    return gzip.open(path, "rt", newline="") if path.suffix == ".gz" else path.open(newline="")


def _read_csv(path: Path, required: set[str], label: str) -> list[dict[str, str]]:
    try:
        with _open_csv(path) as handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing = required - fields
            if missing:
                raise ArchiveError(f"{label} schema missing {sorted(missing)}: {path}")
            rows = list(reader)
    except (OSError, gzip.BadGzipFile, csv.Error) as exc:
        raise ArchiveError(f"unreadable CSV {path}: {exc}") from exc
    if not rows:
        raise ArchiveError(f"{label} is empty: {path}")
    return rows


def _find_one(root: Path, pattern: str, label: str) -> Path:
    matches = sorted(root.rglob(pattern))
    if len(matches) != 1:
        raise ArchiveError(f"{label}: expected one {pattern}, found {len(matches)}")
    return matches[0]


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n")


def _write_gzip_csv(path: Path, fieldnames: Iterable[str], rows: Iterable[dict[str, Any]]) -> None:
    with path.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as zipped:
            with io.TextIOWrapper(zipped, encoding="utf-8", newline="") as text:
                writer = csv.DictWriter(text, fieldnames=list(fieldnames), extrasaction="ignore")
                writer.writeheader()
                writer.writerows(rows)


def _savefig(path: Path) -> None:
    plt.tight_layout()
    plt.savefig(path, dpi=150, metadata={"Software": "IAP retrospective archive"})
    plt.close()


def _load_compact(root: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]], list[Path]]:
    campaigns: list[dict[str, Any]] = []
    merged_runs: list[dict[str, Any]] = []
    merged_pairs: list[dict[str, Any]] = []
    sources: list[Path] = []
    for number in CAMPAIGNS:
        prefix = f"c{number}"
        campaign_path = _find_one(root, f"{prefix}_campaign.json", prefix)
        summary_path = _find_one(root, f"{prefix}_prequalification_summary.json", prefix)
        bundle = summary_path.parent
        runs_path = _plain_or_gzip(
            bundle, f"{prefix}_prequalification_runs", f"{prefix} runs"
        )
        pairs_path = _plain_or_gzip(
            bundle, f"{prefix}_prequalification_pairs", f"{prefix} pairs"
        )
        sources.extend((campaign_path, summary_path, runs_path, pairs_path))
        _json(campaign_path)
        summary = _json(summary_path)
        runs = _read_csv(runs_path, RUN_REQUIRED, f"{prefix} runs")
        pairs = _read_csv(pairs_path, PAIR_REQUIRED, f"{prefix} pairs")
        if summary.get("schema_version") is None or not isinstance(summary.get("runs"), list) or not isinstance(summary.get("pairs"), list):
            raise ArchiveError(f"{prefix} summary schema is incomplete")
        if len(runs) != 10 or len(pairs) != 5:
            raise ArchiveError(f"{prefix} requires 10 runs and 5 pairs; found {len(runs)}/{len(pairs)}")
        hard_gate_passes = sum(_truthy(row["passed"]) for row in runs)
        inferred_complete = hard_gate_passes == 10
        expected_complete = number in COMPLETE_CAMPAIGNS
        if inferred_complete != expected_complete:
            raise ArchiveError(
                f"{prefix} retained classification disagrees with frozen history: "
                f"hard-gate passes={hard_gate_passes}/10"
            )
        classification = "complete_comparable_failure" if inferred_complete else "incomplete_diagnostic"
        campaigns.append({
            "campaign_id": prefix,
            "classification": classification,
            "hard_gate_passes": hard_gate_passes,
            "run_count": len(runs),
            "pair_count": len(pairs),
            "old_protocol_passed": _truthy(summary.get("passed")),
            "compact_bundle": bundle.name,
        })
        for index, row in enumerate(runs):
            merged_runs.append({"campaign_id": prefix, "classification": classification, "run_index": index, **row})
        for index, row in enumerate(pairs):
            merged_pairs.append({"campaign_id": prefix, "classification": classification, "pair_index": index, **row})
    return campaigns, merged_runs, merged_pairs, sources


def _profile_for_summary(
    path: Path, expected_run_id: str, expected_mean: float
) -> list[dict[str, str]]:
    rows = _read_csv(path, PROFILE_REQUIRED, "accepted trajectory profile")
    if {row["run_id"] for row in rows} != {expected_run_id}:
        raise ArchiveError(f"profile run ID does not match {expected_run_id}: {path}")
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if _truthy(row["valid"]) and not _truthy(row["stale"]):
            grouped[row["profile_seq"]].append(row)
    candidates: list[tuple[float, int, list[dict[str, str]]]] = []
    for profile_seq, profile in grouped.items():
        profile.sort(key=lambda row: int(_number(row["sample_index"], "sample_index")))
        observed = sum(_number(row["c_pi"], "c_pi") for row in profile) / len(profile)
        candidates.append((abs(observed - expected_mean), int(profile_seq), profile))
    if not candidates:
        raise ArchiveError(f"no valid profile rows for {expected_run_id}")
    difference, _, selected = min(candidates, key=lambda value: (value[0], value[1]))
    if difference > 1e-9:
        raise ArchiveError(
            f"no profile mean matches compact summary for {expected_run_id}: delta={difference}"
        )
    return selected


def _mechanism_row_base(pair_index: int, kind: str, role: str, run_id: str) -> dict[str, Any]:
    return {
        field: "" for field in MECHANISM_FIELDS
    } | {
        "pair_index": pair_index,
        "pair_kind": kind,
        "role": role,
        "run_id": run_id,
    }


def _load_c38_raw(
    raw_root: Path, compact_runs: list[dict[str, Any]], compact_pairs: list[dict[str, Any]]
) -> tuple[list[dict[str, Any]], list[Path]]:
    campaign_path = raw_root / "campaign.json"
    pair_manifest_path = raw_root / "prequalification_runs.json"
    if not campaign_path.is_file() or not pair_manifest_path.is_file():
        raise ArchiveError("c38 raw campaign requires campaign.json and prequalification_runs.json")
    _json(campaign_path)
    pair_manifest = _json(pair_manifest_path)
    raw_pairs = pair_manifest.get("pairs")
    if not isinstance(raw_pairs, list) or len(raw_pairs) != 5:
        raise ArchiveError("c38 raw prequalification_runs.json requires five pairs")
    summary_runs = {row["run_id"]: row for row in compact_runs if row["campaign_id"] == "c38"}
    summary_pairs = [row for row in compact_pairs if row["campaign_id"] == "c38"]
    mechanism: list[dict[str, Any]] = []
    sources = [campaign_path, pair_manifest_path]
    for pair_index, (raw_pair, compact_pair) in enumerate(zip(raw_pairs, summary_pairs)):
        kind = str(raw_pair.get("kind", ""))
        if kind != compact_pair["kind"]:
            raise ArchiveError(f"c38 pair {pair_index} kind mismatch")
        for role, run_key, export_key in (
            ("reference", "reference_run_id", "reference_export"),
            ("enabled", "enabled_run_id", "enabled_export"),
        ):
            run_id = compact_pair[run_key]
            summary_run = summary_runs.get(run_id)
            if summary_run is None:
                raise ArchiveError(f"c38 pair references unknown run {run_id}")
            export = Path(str(raw_pair.get(export_key, ""))).expanduser().resolve()
            if not export.is_dir():
                raise ArchiveError(f"c38 raw export is missing: {export}")
            profile_path = _plain_or_gzip(
                export, "planner_p1_accepted_trajectory_risk_profile", str(export)
            )
            sources.append(profile_path)
            profile = _profile_for_summary(profile_path, run_id, _number(summary_run["mean"], "run mean"))
            for row in profile:
                item = _mechanism_row_base(pair_index, kind, role, run_id)
                item.update({
                    "record_type": "accepted_profile",
                    "planning_attempt_id": row["planning_attempt_id"],
                    "candidate_id": row["candidate_id"],
                    "snapshot_generation_id": row["snapshot_generation_id"],
                    "query_base_time_s": row["query_base_time_s"],
                    "sample_index": row["sample_index"],
                    "arc_fraction": row["arc_fraction"],
                    "x": row["x"], "y": row["y"], "z": row["z"],
                    "c_pi": row["c_pi"],
                    "gradient_dot_displacement": row.get("grad_dot_displacement", ""),
                    "delta_c_pi": row.get("delta_c_pi", ""),
                    "objective_applied": row.get("objective_applied", ""),
                })
                mechanism.append(item)
            if role != "enabled":
                continue
            optimization_path = _plain_or_gzip(
                export, "planner_p1_candidate_optimization", str(export)
            )
            sources.append(optimization_path)
            optimization = _read_csv(
                optimization_path, OPTIMIZATION_REQUIRED, "candidate optimization"
            )
            if {row["run_id"] for row in optimization} != {run_id}:
                raise ArchiveError(f"optimization run ID does not match {run_id}: {optimization_path}")
            eligible = [
                row for row in optimization
                if _truthy(row["support_full_valid"])
                and _truthy(row["objective_applied"])
                and _truthy(row["optimization_success"])
            ]
            if not eligible:
                raise ArchiveError(f"no complete objective-applied optimization rows: {optimization_path}")
            for row in eligible:
                pre_raw = _number(row["pre_raw_p1_cost"], "pre_raw_p1_cost")
                post_raw = _number(row["post_raw_p1_cost"], "post_raw_p1_cost")
                pre_mean = _number(row["pre_mean_c_pi"], "pre_mean_c_pi")
                post_mean = _number(row["post_mean_c_pi"], "post_mean_c_pi")
                pre_max = _number(row["pre_max_c_pi"], "pre_max_c_pi")
                post_max = _number(row["post_max_c_pi"], "post_max_c_pi")
                item = _mechanism_row_base(pair_index, kind, role, run_id)
                item.update({
                    "record_type": "same_snapshot_optimization",
                    "planning_attempt_id": row["planning_attempt_id"],
                    "candidate_id": row["candidate_id"],
                    "snapshot_generation_id": row["snapshot_generation_id"],
                    "query_base_time_s": row["query_base_time_s"],
                    "gradient_dot_displacement": row["grad_integrity_dot_displacement"],
                    "pre_raw_p1_cost": row["pre_raw_p1_cost"],
                    "post_raw_p1_cost": row["post_raw_p1_cost"],
                    "raw_p1_cost_delta": post_raw - pre_raw,
                    "pre_mean_c_pi": row["pre_mean_c_pi"],
                    "post_mean_c_pi": row["post_mean_c_pi"],
                    "mean_c_pi_delta": post_mean - pre_mean,
                    "pre_max_c_pi": row["pre_max_c_pi"],
                    "post_max_c_pi": row["post_max_c_pi"],
                    "max_c_pi_delta": post_max - pre_max,
                    "initial_control_points_hash": row["initial_control_points_hash"],
                    "final_control_points_hash": row["final_control_points_hash"],
                    "support_signature": row["support_signature"],
                    "objective_applied": row["objective_applied"],
                })
                mechanism.append(item)
    return mechanism, sources


def _plot_completeness(path: Path, campaigns: list[dict[str, Any]]) -> None:
    colors = ["#2563eb" if row["classification"].startswith("complete") else "#94a3b8" for row in campaigns]
    plt.figure(figsize=(9, 4.5))
    bars = plt.bar([row["campaign_id"] for row in campaigns], [row["hard_gate_passes"] for row in campaigns], color=colors)
    plt.axhline(10, color="#0f172a", linestyle="--", linewidth=1)
    plt.ylim(0, 10.8)
    plt.ylabel("runs passing all hard gates (of 10)")
    plt.title("c31–c38 retained campaign completeness")
    for bar, row in zip(bars, campaigns):
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + .12, str(row["hard_gate_passes"]), ha="center", fontsize=9)
    _savefig(path)


def _plot_primary_thresholds(path: Path, pairs: list[dict[str, Any]]) -> None:
    rows = [row for row in pairs if row["campaign_id"] in {"c31", "c32", "c38"} and row["kind"] == "primary"]
    labels = [f"{row['campaign_id']}-p{int(row['pair_index']) + 1}" for row in rows]
    x = list(range(len(rows)))
    width = .36
    plt.figure(figsize=(10, 4.8))
    plt.bar([value - width / 2 for value in x], [_number(row["mean_improvement"], "mean improvement") for row in rows], width, label="mean improvement")
    plt.bar([value + width / 2 for value in x], [_number(row["cvar_improvement"], "CVaR improvement") for row in rows], width, label="smooth-CVaR improvement")
    plt.axhline(OLD_MEAN_THRESHOLD, color="#2563eb", linestyle="--", label="old mean threshold 0.00836")
    plt.axhline(OLD_CVAR_THRESHOLD, color="#f97316", linestyle=":", label="old CVaR threshold 0.00677")
    plt.axhline(0, color="#0f172a", linewidth=.8)
    plt.xticks(x, labels)
    plt.ylabel("reference − enabled")
    plt.title("Complete campaigns: primary metrics versus historical v1 thresholds")
    plt.legend(fontsize=8, ncol=2)
    _savefig(path)


def _plot_c38_pairs(path: Path, pairs: list[dict[str, Any]]) -> None:
    rows = [row for row in pairs if row["campaign_id"] == "c38"]
    labels = [f"{row['kind']}-{int(row['pair_index']) + 1}" for row in rows]
    x = list(range(len(rows)))
    width = .25
    plt.figure(figsize=(10, 4.8))
    plt.bar([v - width for v in x], [_number(row["mean_improvement"], "mean") for row in rows], width, label="mean improvement")
    plt.bar(x, [_number(row["cvar_improvement"], "CVaR") for row in rows], width, label="CVaR improvement")
    plt.bar([v + width for v in x], [-_number(row["max_regression"], "max regression") for row in rows], width, label="max improvement")
    plt.axhline(0, color="#0f172a", linewidth=.8)
    plt.xticks(x, labels, rotation=15)
    plt.ylabel("positive means lower enabled risk")
    plt.title("c38 five retained pairs: mean / smooth-CVaR / exact max")
    plt.legend(fontsize=8)
    _savefig(path)


def _plot_mechanism(path: Path, mechanism: list[dict[str, Any]]) -> None:
    rows = [row for row in mechanism if row["record_type"] == "same_snapshot_optimization"]
    deltas = [_number(row["raw_p1_cost_delta"], "raw P1 delta") for row in rows]
    dots = [_number(row["gradient_dot_displacement"], "gradient dot displacement") for row in rows]
    means = [_number(row["mean_c_pi_delta"], "mean PL delta") for row in rows]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    axes[0].scatter(deltas, means, s=18, alpha=.65, color="#2563eb")
    axes[0].axhline(0, color="#0f172a", linewidth=.8)
    axes[0].axvline(0, color="#0f172a", linewidth=.8)
    axes[0].set_xlabel("post − pre raw P1 objective")
    axes[0].set_ylabel("post − pre mean PL")
    axes[0].set_title("same-snapshot local risk change")
    axes[1].hist(dots, bins=min(20, max(5, int(math.sqrt(len(dots))))), color="#0f766e", alpha=.8)
    axes[1].axvline(0, color="#0f172a", linewidth=.8)
    axes[1].set_xlabel("raw P1 gradient · displacement")
    axes[1].set_ylabel("candidate optimizations")
    axes[1].set_title("descent-direction evidence")
    fig.suptitle("c38 objective-applied, full-support candidate mechanism")
    _savefig(path)


def _plot_primary_profiles(path: Path, mechanism: list[dict[str, Any]]) -> None:
    profiles = [row for row in mechanism if row["record_type"] == "accepted_profile" and row["pair_kind"] == "primary"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8))
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for row in profiles:
        grouped[(row["pair_index"], row["role"])].append(row)
    for (pair_index, role), rows in sorted(grouped.items()):
        rows.sort(key=lambda row: int(_number(row["sample_index"], "sample_index")))
        label = f"primary-{int(pair_index) + 1} {role}"
        style = "--" if role == "reference" else "-"
        axes[0].plot([_number(row["x"], "x") for row in rows], [_number(row["y"], "y") for row in rows], style, label=label)
        axes[1].plot([_number(row["arc_fraction"], "arc") for row in rows], [_number(row["c_pi"], "c_pi") for row in rows], style, label=label)
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title("authoritative checkpoint trajectories")
    axes[0].axis("equal")
    axes[1].set_xlabel("normalized arc fraction")
    axes[1].set_ylabel("predicted PL / c_pi")
    axes[1].set_title("risk profile on retained trajectory")
    axes[1].legend(fontsize=8)
    fig.suptitle("c38 primary paired trajectories and risk profiles")
    _savefig(path)


def _readme() -> str:
    return """# P1-2 c31–c38 retrospective archive

This directory is a read-only retrospective generated from retained compact
c31–c38 evidence and the retained c38 raw campaign. It does not run the ROS
campaign, planner, legacy prequalification analyzer, or formal analyzer.

Historical authority remains unchanged: c31, c32, and c38 are three complete,
comparable prequalification failures; c33–c37 are incomplete diagnostics. The
legacy P1-2 verdict remains **BLOCKED**, with zero formal-analyzer invocations.
Phase 3 v2 is a protocol for a future fresh campaign and cannot retroactively
change this verdict or establish a product-level P1 PASS.

## Files and limits of inference

- `p1_2_campaign_completeness_overview.png` proves which retained campaigns had
  10/10 hard-gate runs. It cannot prove P1 effectiveness.
- `p1_2_primary_threshold_comparison.png` reproduces the complete primary-pair
  improvements against the old `0.00836/0.00677` cross-run thresholds. It
  proves the historical v1 stop condition, not that P1's local mechanism is
  ineffective and not that those thresholds are suitable for Phase 3 v2.
- `c38_pair_metric_dashboard.png` shows mean, smooth-CVaR, and exact-max changes
  for all five retained c38 pairs. Independent-run and route-choice noise mean
  it cannot identify a same-candidate causal effect or serve as P2 evidence.
- `c38_same_snapshot_mechanism.png` shows local before/after P1 objective and PL
  changes plus gradient/displacement direction for c38 objective-applied,
  full-support candidates on immutable snapshots. It supports a local descent
  mechanism. It cannot establish closed-loop benefit, a production weight, or
  Phase 3 v2 PASS.
- `c38_primary_trajectory_risk_profiles.png` reproduces the compact-summary-
  bound checkpoint trajectories and risk profiles. It shows the observed c38
  paired paths and risks, but cannot separate P1 from independent-run noise.

The three gzip CSVs are the normalized, recomputable run, pair, and mechanism
tables. `source_inventory.json` and `source_hashes.sha256` bind every input
read; `artifact_hashes.sha256` binds every generated output except itself.
"""


def build_archive(compact_root: Path, raw_root: Path, output_dir: Path) -> None:
    compact_root = compact_root.expanduser().resolve()
    raw_root = raw_root.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not compact_root.is_dir():
        raise ArchiveError(f"compact root is missing: {compact_root}")
    if not raw_root.is_dir():
        raise ArchiveError(f"c38 raw campaign is missing: {raw_root}")
    if output_dir.exists():
        raise ArchiveError(f"output directory already exists: {output_dir}")

    campaigns, runs, pairs, compact_sources = _load_compact(compact_root)
    mechanism, raw_sources = _load_c38_raw(raw_root, runs, pairs)
    sources = sorted({path.resolve() for path in (*compact_sources, *raw_sources)}, key=str)
    inventory = {
        "schema_version": "p1_2_retrospective_source_inventory_v1",
        "read_only": True,
        "formal_analyzer_invoked": False,
        "sources": [
            {"path": str(path), "size_bytes": path.stat().st_size, "sha256": _sha256(path)}
            for path in sources
        ],
    }
    optimizations = [row for row in mechanism if row["record_type"] == "same_snapshot_optimization"]
    summary = {
        "schema_version": "p1_2_retrospective_summary_v1",
        "archive_date": "2026-08-10",
        "historical_verdict": "BLOCKED",
        "product_p1_status_changed": False,
        "formal_analyzer_invocation_count": 0,
        "legacy_formal_analyzer_invoked": False,
        "future_protocol": "Phase 3 v2",
        "future_campaign_required": True,
        "old_cross_run_thresholds": {
            "mean_improvement": OLD_MEAN_THRESHOLD,
            "smooth_cvar_improvement": OLD_CVAR_THRESHOLD,
            "retained_for_historical_reproduction_only": True,
        },
        "campaigns": campaigns,
        "mechanism_observations": {
            "objective_applied_full_support_rows": len(optimizations),
            "raw_p1_objective_decreased": sum(_number(row["raw_p1_cost_delta"], "raw delta") < 0 for row in optimizations),
            "gradient_dot_displacement_negative": sum(_number(row["gradient_dot_displacement"], "dot") < 0 for row in optimizations),
            "mean_pl_decreased": sum(_number(row["mean_c_pi_delta"], "mean delta") < 0 for row in optimizations),
            "interpretation": "local same-snapshot descent evidence only; not a Phase 3 v2 PASS",
        },
        "figures": list(FIGURES),
        "derived_tables": [
            "p1_2_retrospective_runs.csv.gz",
            "p1_2_retrospective_pairs.csv.gz",
            "p1_2_retrospective_mechanism.csv.gz",
        ],
    }

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=f".{output_dir.name}-", dir=output_dir.parent))
    try:
        run_fields = ("campaign_id", "classification", "run_index", *[field for field in runs[0] if field not in {"campaign_id", "classification", "run_index"}])
        pair_fields = ("campaign_id", "classification", "pair_index", *[field for field in pairs[0] if field not in {"campaign_id", "classification", "pair_index"}])
        _write_json(temporary / "p1_2_retrospective_summary.json", summary)
        _write_json(temporary / "source_inventory.json", inventory)
        _write_gzip_csv(temporary / "p1_2_retrospective_runs.csv.gz", run_fields, runs)
        _write_gzip_csv(temporary / "p1_2_retrospective_pairs.csv.gz", pair_fields, pairs)
        _write_gzip_csv(temporary / "p1_2_retrospective_mechanism.csv.gz", MECHANISM_FIELDS, mechanism)
        (temporary / "source_hashes.sha256").write_text(
            "".join(f"{item['sha256']}  {item['path']}\n" for item in inventory["sources"])
        )
        (temporary / "README.md").write_text(_readme())
        _plot_completeness(temporary / FIGURES[0], campaigns)
        _plot_primary_thresholds(temporary / FIGURES[1], pairs)
        _plot_c38_pairs(temporary / FIGURES[2], pairs)
        _plot_mechanism(temporary / FIGURES[3], mechanism)
        _plot_primary_profiles(temporary / FIGURES[4], mechanism)
        generated = sorted(path for path in temporary.iterdir() if path.name != "artifact_hashes.sha256")
        (temporary / "artifact_hashes.sha256").write_text(
            "".join(f"{_sha256(path)}  {path.name}\n" for path in generated)
        )
        temporary.rename(output_dir)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compact-root", required=True, type=Path)
    parser.add_argument("--c38-raw-campaign", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        build_archive(args.compact_root, args.c38_raw_campaign, args.output_dir)
    except ArchiveError as exc:
        print(f"archive failed closed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
