#!/usr/bin/env python3
"""Run and fail-closed analyze the development-only ICRA interface ladder."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import os
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = (
    REPOSITORY / "results/icra27/dev_runs/interface_integration"
).resolve()
DEFAULT_INSTALL_ROOT = Path("/home/dev/ws_iap/install").resolve()
STAGE_ORDER = ("estimator", "p0", "p4", "p5-final", "full", "shutdown")
SEVEN_STAGE_ORDER = (
    "p0_snapshot", "closed_collision", "p4_selection_application",
    "ego_final_bspline", "p5_final_pass_before_publish",
    "normal_publication", "p5_runtime_committed",
)
FORBIDDEN_LAYER_ARGS = {
    "planner_enable_p1": "false",
    "planner_enable_p2": "false",
    "planner_enable_p3_local": "false",
    "planner_enable_p3_global": "false",
}
COMMON_ARGS = {
    "experiment": "icra_p0_p4_v2_p5_dev",
    "scenario": "icra072_p4_selection_trigger_v1",
    "odometry_acc_scale": "1.0",
    "odometry_initialization_mode": "NAIVE",
    "lidar_start_delay_s": "2.0",
    "planner_start_delay_s": "10.0",
    "record_bag": "false",
    "start_rviz": "false",
    "run_validator": "true",
    **FORBIDDEN_LAYER_ARGS,
}


class StageSpec:
    def __init__(self, duration_s: float, **launch_args: str) -> None:
        self.duration_s = duration_s
        self.launch_args = {**COMMON_ARGS, **launch_args}


STAGES = {
    "estimator": StageSpec(
        20.0,
        start_planner="false",
        planner_enable_p4="false",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
    ),
    "p0": StageSpec(
        35.0,
        start_planner="true",
        planner_enable_p4="false",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
    ),
    "p4": StageSpec(
        45.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="false",
        planner_enable_p5_runtime="false",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "p5-final": StageSpec(
        60.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="false",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "full": StageSpec(
        75.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="true",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
    "shutdown": StageSpec(
        20.0,
        start_planner="true",
        planner_enable_p4="true",
        planner_enable_p5_final="true",
        planner_enable_p5_runtime="true",
        **{"safety_viz.enable_p4_viz": "true"},
    ),
}


def _result(failures: list[str], **details) -> dict:
    unique = list(dict.fromkeys(failures))
    return {
        "schema_version": "icra_interface_stage_summary_v1",
        "development_only": True,
        "qualification_claim": False,
        "scientific_effect_claim": False,
        "result": "PASS" if not unique else "FAIL",
        "failures": unique,
        **details,
    }


def _finite_number(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def analyze_estimator(metrics: dict) -> dict:
    failures: list[str] = []
    required = (
        "acc_scale", "rotation_deg", "velocity_norm_mps", "bias_norm",
        "odom_count", "position_error_p95_m", "finite",
    )
    if any(key not in metrics for key in required):
        failures.append("estimator_metrics_incomplete")
    if not _finite_number(metrics.get("acc_scale")) or not math.isclose(
            float(metrics.get("acc_scale", math.nan)), 1.0,
            rel_tol=0.0, abs_tol=1.0e-9):
        failures.append("acc_scale_not_si")
    limits = (
        ("rotation_deg", 5.0, "initial_rotation_exceeded"),
        ("velocity_norm_mps", 0.1, "initial_velocity_exceeded"),
        ("bias_norm", 0.05, "initial_bias_exceeded"),
        ("position_error_p95_m", 0.25, "aligned_position_error_exceeded"),
    )
    for key, upper, failure in limits:
        if not _finite_number(metrics.get(key)) or float(metrics[key]) > upper:
            failures.append(failure)
    if int(metrics.get("odom_count", 0) or 0) < 10:
        failures.append("iap_odom_not_continuous")
    if metrics.get("finite") is not True:
        failures.append("estimator_non_finite")
    return _result(failures, metrics=metrics)


def _health_payload(row: dict) -> tuple[float, dict]:
    payload = row.get("payload", row)
    received = row.get(
        "receive_steady_s", payload.get("capture_receive_steady_s", math.nan))
    try:
        return float(received), payload
    except (TypeError, ValueError):
        return math.nan, payload


def _healthy(payload: dict) -> bool:
    return (
        payload.get("ready") is True
        and payload.get("stale") is False
        and payload.get("reason") == "ok"
        and int(payload.get("generation_id", 0) or 0) > 0
    )


def analyze_p0(rows: list[dict], capture_start_s: float | None = None) -> dict:
    observations = sorted(
        (_health_payload(row) for row in rows), key=lambda item: item[0])
    observations = [item for item in observations if math.isfinite(item[0])]
    failures: list[str] = []
    first_index = next(
        (index for index, (_, payload) in enumerate(observations)
         if _healthy(payload)), None)
    if first_index is None:
        return _result(["p0_healthy_generation_missing"], health_count=len(rows))
    first_time = observations[first_index][0]
    start_time = (
        float(capture_start_s) if capture_start_s is not None
        else observations[0][0]
    )
    if first_time - start_time > 15.0:
        failures.append("p0_first_healthy_generation_late")
    window: list[tuple[float, dict]] = []
    longest: list[tuple[float, dict]] = []
    current: list[tuple[float, dict]] = []
    continuity_broken = False
    for observation in observations[first_index:]:
        if not _healthy(observation[1]):
            continuity_broken = continuity_broken or bool(current)
            current = []
            continue
        current.append(observation)
        if (not longest or
                current[-1][0] - current[0][0] >
                longest[-1][0] - longest[0][0]):
            longest = list(current)
        if (len(current) >= 10 and
                current[-1][0] - current[0][0] >= 15.0):
            window = list(current)
            break
    if not window:
        window = longest
    span = window[-1][0] - window[0][0] if window else 0.0
    if len(window) < 10 or span < 15.0:
        failures.append("p0_healthy_window_too_short")
    if not window or any(not _healthy(payload) for _, payload in window):
        failures.append("p0_health_not_continuous")
    elif span < 15.0 and continuity_broken:
        failures.append("p0_health_not_continuous")
    generations = [int(payload.get("generation_id", 0) or 0)
                   for _, payload in window]
    if (not generations or any(right < left for left, right in
                               zip(generations, generations[1:]))):
        failures.append("p0_generation_not_monotonic")
    if not generations or max(generations) - min(generations) < 3:
        failures.append("p0_generation_did_not_advance")
    return _result(
        failures,
        health_count=len(observations),
        window_count=len(window),
        window_span_s=span,
        first_healthy_delay_s=first_time - start_time,
        generation_min=min(generations) if generations else None,
        generation_max=max(generations) if generations else None,
    )


DECISION_ID_FIELDS = (
    "planning_attempt_id", "collision_segment_id", "request_hash",
    "snapshot_generation_id", "snapshot_config_hash", "occupancy_epoch",
)


def _support_complete(row: dict, prefix: str) -> bool:
    try:
        samples = int(row[f"{prefix}_sample_count"])
        valid = int(row[f"{prefix}_valid_count"])
        unknown = int(row[f"{prefix}_unknown_count"])
        stale = int(row[f"{prefix}_stale_count"])
        non_finite = int(row[f"{prefix}_non_finite_count"])
    except (KeyError, TypeError, ValueError):
        return False
    return (
        samples > 0 and valid == samples and unknown == 0 and stale == 0
        and non_finite == 0
    )


def _selected_decisions(decisions: list[dict]) -> list[dict]:
    return [
        row for row in decisions
        if row.get("status") == "RISK_SELECTED"
        and str(row.get("selection_applied")) == "1"
        and all(row.get(field) for field in DECISION_ID_FIELDS)
        and all(row.get(field) for field in
                ("original_hash", "risk_hash", "selected_hash"))
        and _support_complete(row, "original")
        and _support_complete(row, "risk")
    ]


def _decision_lineage_key(row: dict, lineage: bool = False) -> tuple[str, ...]:
    values = [str(row.get(field, "")) for field in DECISION_ID_FIELDS]
    if lineage:
        values.extend(str(row.get(field, "")) for field in (
            "original_guide_hash", "risk_guide_hash", "selected_guide_hash"))
    else:
        values.extend(str(row.get(field, "")) for field in (
            "original_hash", "risk_hash", "selected_hash"))
    return tuple(values)


def _lineage_groups(decisions: list[dict], lineage: list[dict]) -> list[dict]:
    selected_keys = {_decision_lineage_key(row): row
                     for row in _selected_decisions(decisions)}
    groups: dict[tuple[str, ...], list[dict]] = {}
    for row in lineage:
        if str(row.get("selection_applied")) != "1":
            continue
        key = _decision_lineage_key(row, lineage=True)
        if key not in selected_keys:
            continue
        identity = (
            str(row.get("trajectory_id", "")),
            str(row.get("trajectory_start_ns", "")),
            str(row.get("control_points_hash", "")),
            str(row.get("final_bspline_identity", "")),
        )
        if not all(identity):
            continue
        groups.setdefault(key + identity, []).append(row)
    return [
        {
            "key": key,
            "rows": rows,
            "ordered_stages": [row.get("stage") for row in rows],
            "stages": {row.get("stage") for row in rows},
            "closed_collision_observed": any(
                str(row.get("closed_collision_observed")) == "1"
                for row in rows),
            "trajectory_id": int(key[-4]),
            "start_ns": int(key[-3]),
        }
        for key, rows in groups.items()
        if key[-4].isdigit() and key[-3].isdigit()
    ]


def _stable_bspline(bsplines: list[dict]) -> tuple[bool, float]:
    valid = [row for row in bsplines
             if int(row.get("payload", row).get("trajectory_id", 0) or 0) > 0
             and int(row.get("payload", row).get("start_time_ns", 0) or 0) > 0]
    times = sorted(float(row.get("receive_steady_s", math.nan)) for row in valid)
    times = [value for value in times if math.isfinite(value)]
    span = times[-1] - times[0] if len(times) >= 2 else 0.0
    return len(valid) >= 3 and span >= 5.0, span


def _poscmd_sustained(times: list[float]) -> tuple[bool, float]:
    finite = sorted(float(value) for value in times if _finite_number(value))
    if len(finite) < 2:
        return False, 0.0
    span = finite[-1] - finite[0]
    rate = (len(finite) - 1) / span if span > 0.0 else 0.0
    return span >= 5.0 and rate >= 80.0, rate


def _p5_safe(payload: dict) -> bool:
    return (
        payload.get("action") == "OK"
        and payload.get("raw_action") == "OK"
        and payload.get("reason") == "ok"
        and payload.get("raw_reason") == "ok"
        and payload.get("active_reasons") == []
        and payload.get("current_reason") == ""
        and payload.get("future_reason") == ""
        and payload.get("final_candidate_rejected") is False
        and payload.get("current_integrity_source") == "FUSED"
    )


def _message_identity(row: dict) -> tuple[int, int] | None:
    payload = row.get("payload", row)
    try:
        identity = (
            int(payload.get("trajectory_id", 0) or 0),
            int(payload.get("start_time_ns", 0) or 0),
        )
    except (TypeError, ValueError):
        return None
    return identity if identity[0] > 0 and identity[1] > 0 else None


def _runtime_identity(payload: dict) -> tuple[int, int] | None:
    try:
        identities = {
            (int(sample.get("trajectory_id", 0) or 0),
             int(sample.get("trajectory_start_time_ns", 0) or 0))
            for sample in payload.get("samples", [])
            if sample.get("trajectory_sample_source") == "runtime_committed"
        }
    except (TypeError, ValueError):
        return None
    if len(identities) != 1:
        return None
    identity = next(iter(identities))
    return identity if identity[0] > 0 and identity[1] > 0 else None


def analyze_stage_records(
        stage: str, health: list[dict], decisions: list[dict],
        lineage: list[dict], bsplines: list[dict], p5_status: list[dict],
        poscmd_times: list[float], capture_start_s: float | None = None) -> dict:
    if stage not in ("p4", "p5-final", "full"):
        raise ValueError(f"unsupported record stage: {stage}")
    p0 = analyze_p0(health, capture_start_s)
    failures = list(p0["failures"])
    selected = _selected_decisions(decisions)
    if not selected:
        failures.append("p4_risk_selected_missing")
    groups = _lineage_groups(decisions, lineage)
    published_groups = [group for group in groups if {
        "final_bspline_before_p5", "normal_publish_authorized",
    }.issubset(group["stages"])]
    if not published_groups:
        failures.append("p4_selected_lineage_missing")
    stable, bspline_span = _stable_bspline(bsplines)
    if not stable:
        failures.append("stable_bspline_missing")
    poscmd_ok, poscmd_rate = _poscmd_sustained(poscmd_times)
    if not poscmd_ok:
        failures.append("poscmd_not_sustained")

    if stage in ("p5-final", "full"):
        final_groups = [group for group in published_groups
                        if "p5_final_pass_before_publish" in group["stages"]]
        final_records = [row for row in p5_status
                         if row.get("payload", row).get("phase") == "final"
                         and _p5_safe(row.get("payload", row))]
        # The terminal lineage writer runs synchronously after evaluateFinal()
        # returns OK and before normal publication.  It is therefore the
        # authoritative causal record for the selected P4 trajectory, even if
        # DDS discovery makes the external status capture begin one trajectory
        # later.  Captured publications are checked independently below against
        # their exact final-status identities.
        matched = final_groups
        if not matched:
            failures.append("p5_final_identity_or_status_invalid")
        published_identities = {
            identity for row in bsplines
            if (identity := _message_identity(row)) is not None
        }
        final_status_identities = {
            (int(payload.get("final_candidate_traj_id", 0) or 0),
             int(payload.get("final_candidate_start_time_ns", 0) or 0))
            for row in final_records
            for payload in [row.get("payload", row)]
        }
        published_candidates_final_ok = (
            bool(published_identities)
            and published_identities.issubset(final_status_identities)
        )
        if not published_candidates_final_ok:
            failures.append("p5_published_candidate_without_prior_final_ok")
        runtime_committed = [
            row for row in p5_status
            if row.get("payload", row).get("phase") == "runtime"
            and any(sample.get("trajectory_sample_source") == "runtime_committed"
                    for sample in row.get("payload", row).get("samples", []))
        ]
        if stage == "p5-final" and runtime_committed:
            failures.append("p5_runtime_leaked_into_final_only_stage")
        if stage == "full":
            runtime_identities: set[tuple[int, int]] = set()
            runtime_ok = bool(runtime_committed)
            publication_times = [
                float(row.get("receive_steady_s")) for row in bsplines
                if _finite_number(row.get("receive_steady_s"))
                and _message_identity(row) is not None
            ]
            first_publication_s = min(publication_times, default=math.inf)
            in_window_runtime_count = 0
            for row in runtime_committed:
                payload = row.get("payload", row)
                identity = _runtime_identity(payload)
                if not _p5_safe(payload) or identity is None:
                    runtime_ok = False
                    continue
                receive_s = float(row.get("receive_steady_s", math.inf))
                if receive_s >= first_publication_s:
                    in_window_runtime_count += 1
                    if identity not in published_identities:
                        runtime_ok = False
                        continue
                runtime_identities.add(identity)
            published_runtime_match = bool(
                runtime_identities.intersection(published_identities))
            runtime_ok = (
                runtime_ok
                and in_window_runtime_count > 0
                and published_runtime_match
            )
            if not runtime_ok:
                failures.append("p5_runtime_identity_or_status_invalid")

            ordered_terminal = any(
                group["ordered_stages"].index("final_bspline_before_p5")
                < group["ordered_stages"].index("p5_final_pass_before_publish")
                < group["ordered_stages"].index("normal_publish_authorized")
                for group in matched
                if all(required in group["ordered_stages"] for required in (
                    "final_bspline_before_p5",
                    "p5_final_pass_before_publish",
                    "normal_publish_authorized",
                )))
            stage_status = {
                "p0_snapshot": p0["result"] == "PASS",
                "closed_collision": any(
                    group["closed_collision_observed"] for group in groups),
                "p4_selection_application": bool(groups),
                "ego_final_bspline": bool(published_groups) and stable,
                "p5_final_pass_before_publish": (
                    bool(matched) and published_candidates_final_ok),
                "normal_publication": ordered_terminal,
                "p5_runtime_committed": runtime_ok,
            }
            first_missing_stage = next(
                (name for name in SEVEN_STAGE_ORDER
                 if not stage_status[name]), None)
            seven_stage = {
                "stage_order": list(SEVEN_STAGE_ORDER),
                "stage_status": stage_status,
                "first_missing_stage": first_missing_stage,
                "result": "PASS" if first_missing_stage is None else "FAIL",
            }
        else:
            runtime_identities = set()
            runtime_ok = False
            seven_stage = None
    else:
        final_records = []
        runtime_committed = []
        runtime_identities = set()
        runtime_ok = False
        published_candidates_final_ok = False
        seven_stage = None

    return _result(
        failures,
        stage=stage,
        p0=p0,
        selected_count=len(selected),
        lineage_group_count=len(groups),
        published_group_count=len(published_groups),
        bspline_count=len(bsplines),
        bspline_span_s=bspline_span,
        poscmd_count=len(poscmd_times),
        poscmd_rate_hz=poscmd_rate,
        p5_final_ok_count=len(final_records),
        p5_published_candidates_final_ok=published_candidates_final_ok,
        p5_runtime_committed_count=len(runtime_committed),
        p5_runtime_ok_count=(len(runtime_committed) if runtime_ok else 0),
        p5_runtime_identities=[list(identity)
                               for identity in sorted(runtime_identities)],
        seven_stage=seven_stage,
    )


def analyze_shutdown(
        launch_exit_code: int | None, stdout: str,
        owned_group_cleared: bool, residual_nodes: list[str],
        runner_escalated: bool = False) -> dict:
    failures: list[str] = []
    if launch_exit_code != 0:
        failures.append("launch_exit_nonzero")
    if runner_escalated or re.search(
            r"escalating to '?(?:SIGTERM|SIGKILL)|sending signal 'SIGKILL'",
            stdout, flags=re.IGNORECASE):
        failures.append("shutdown_escalated")
    if re.search(r"process has died .*exit code (?:-[0-9]+|[1-9][0-9]*)",
                 stdout):
        failures.append("child_process_nonzero")
    if re.search(r"segmentation fault|RCLError|terminate called|system_error",
                 stdout, flags=re.IGNORECASE):
        failures.append("shutdown_exception_or_crash")
    if not owned_group_cleared:
        failures.append("owned_process_group_remaining")
    if residual_nodes:
        failures.append("task_ros_nodes_remaining")
    return _result(
        failures,
        launch_exit_code=launch_exit_code,
        owned_group_cleared=owned_group_cleared,
        residual_nodes=residual_nodes,
    )


def _read_jsonl(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            rows.append({"kind": "capture_parse_error", "raw": line})
    return rows


def _read_csv(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _vector(text: str, label: str) -> list[float] | None:
    match = re.search(rf"{re.escape(label)}=vec\(([^)]+)\)", text)
    if not match:
        return None
    try:
        return [float(value) for value in match.group(1).split(",")]
    except ValueError:
        return None


def _estimator_metrics(run_root: Path, records: list[dict]) -> dict:
    main_logs = list((run_root / "runtime/iap_logs").glob("**/runtime/iap_main.log"))
    odom_logs = list((run_root / "runtime/iap_logs").glob("**/runtime/iap_odom.log"))
    main_text = main_logs[-1].read_text() if main_logs else ""
    odom_text = odom_logs[-1].read_text() if odom_logs else ""
    scale_match = re.search(r"auto-detected acc_scale=([0-9.eE+-]+)", main_text)
    if not scale_match:
        scale_match = re.search(r"configured acc_scale=([0-9.eE+-]+)", main_text)
    configured_scale = math.nan
    config_paths = sorted((run_root / "runtime").glob("**/config_ros.json"))
    for config_path in config_paths:
        try:
            config = json.loads(config_path.read_text())
            candidate = float(config["glim_ros"]["acc_scale"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if math.isfinite(candidate):
            configured_scale = candidate
            break
    pose_match = re.search(r"T_world_imu=se3\(([^)]+)\)", odom_text)
    pose = []
    if pose_match:
        try:
            pose = [float(value) for value in pose_match.group(1).split(",")]
        except ValueError:
            pose = []
    velocity = _vector(odom_text, "v_world_imu") or []
    bias = _vector(odom_text, "imu_bias") or []
    rotation_deg = math.inf
    if len(pose) == 7 and all(math.isfinite(value) for value in pose):
        norm = math.sqrt(sum(value * value for value in pose[3:]))
        if norm > 0.0:
            qw = max(-1.0, min(1.0, abs(pose[6] / norm)))
            rotation_deg = math.degrees(2.0 * math.acos(qw))
    metrics_paths = list((run_root / "exports").glob("**/iap_sim_truth_vs_est.csv"))
    errors = []
    if metrics_paths:
        for row in _read_csv(metrics_paths[-1]):
            try:
                value = float(row["position_error_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(value):
                errors.append(value)
    p95 = math.inf
    if errors:
        ordered = sorted(errors)
        p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
    odom_count = sum(row.get("kind") == "iap_odom" for row in records)
    values = pose + velocity + bias + errors
    return {
        "acc_scale": float(scale_match.group(1)) if scale_match
        else configured_scale,
        "rotation_deg": rotation_deg,
        "velocity_norm_mps": math.sqrt(sum(value * value for value in velocity))
        if velocity else math.inf,
        "bias_norm": math.sqrt(sum(value * value for value in bias))
        if bias else math.inf,
        "odom_count": odom_count,
        "position_error_p95_m": p95,
        "finite": bool(values) and all(math.isfinite(value) for value in values),
    }


def _stage_start_steady_s(run_root: Path) -> float | None:
    """Return the functional-stage origin, falling back for old captures."""
    for path, key in (
            (run_root / "launch_started.json", "started_steady_s"),
            (run_root / "capture_ready.json", "ready_steady_s")):
        if not path.is_file():
            continue
        try:
            value = float(json.loads(path.read_text()).get(key))
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def analyze_run(stage: str, run_root: Path) -> dict:
    records = _read_jsonl(run_root / "capture.jsonl")
    health = [row for row in records if row.get("kind") == "p0_health"]
    bsplines = [row for row in records if row.get("kind") == "normal_bspline"]
    p5_status = [row for row in records if row.get("kind") == "p5_status"]
    poscmd_times = [row.get("receive_steady_s") for row in records
                    if row.get("kind") == "poscmd"]
    stage_start = _stage_start_steady_s(run_root)
    decisions = _read_csv(run_root / "exports/planner_p4_risk_astar_debug.csv")
    lineage = _read_csv(
        run_root / "exports/planner_p4_risk_astar_debug.csv.lineage.csv")
    if stage == "estimator":
        return analyze_estimator(_estimator_metrics(run_root, records))
    if stage == "p0":
        return analyze_p0(health, stage_start)
    if stage in ("p4", "p5-final", "full"):
        return analyze_stage_records(
            stage, health, decisions, lineage, bsplines, p5_status,
            poscmd_times, stage_start)
    raise ValueError(f"unsupported functional stage: {stage}")


def _capture_main(args: argparse.Namespace) -> int:
    import rclpy
    from iap.msg import IntegrityReport
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String
    from traj_utils.msg import Bspline

    output = args.capture_output.resolve()
    ready = args.capture_ready.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    class Capture(Node):
        def __init__(self) -> None:
            super().__init__("icra_interface_integration_capture")
            self.stream = output.open("x", buffering=1)
            reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                String, "/planning/risk_grid_health",
                lambda message: self.json_record("p0_health", message.data),
                reliable)
            self.create_subscription(
                String, "/planning/integrity_gate_status",
                lambda message: self.json_record("p5_status", message.data),
                reliable)
            self.create_subscription(
                Bspline, "/drone_0_planning/bspline", self.bspline, reliable)
            self.create_subscription(
                PositionCommand, "/drone_0_planning/pos_cmd",
                lambda _message: self.record("poscmd", {}), reliable)
            self.create_subscription(
                Odometry, "/drone_0_visual_slam/odom",
                lambda message: self.record("iap_odom", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                }), qos_profile_sensor_data)
            self.create_subscription(
                IntegrityReport, "/iap/integrity",
                lambda message: self.record("integrity", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "hpl": float(message.hpl),
                    "vpl": float(message.vpl),
                    "fusion_mode": message.fusion_mode,
                }), reliable)
            self.create_subscription(
                PointCloud2, "/sim/drone_0/lidar",
                lambda message: self.record("occupancy_input", {
                    "stamp_s": float(message.header.stamp.sec)
                    + 1.0e-9 * float(message.header.stamp.nanosec),
                    "point_count": int(message.width) * int(message.height),
                }), qos_profile_sensor_data)
            self.imu_count = 0
            self.create_subscription(
                Imu, "/sim/drone_0/imu_iap", self.imu,
                qos_profile_sensor_data)
            ready.write_text(json.dumps({
                "schema_version": "icra_interface_capture_ready_v1",
                "ready": True,
                "ready_steady_s": time.monotonic(),
                "pid": os.getpid(),
            }, indent=2, sort_keys=True) + "\n")
            self.create_timer(args.capture_duration, rclpy.shutdown)

        def record(self, kind: str, payload: dict) -> None:
            self.stream.write(json.dumps({
                "kind": kind,
                "receive_steady_s": time.monotonic(),
                "payload": payload,
            }, sort_keys=True) + "\n")

        def json_record(self, kind: str, raw: str) -> None:
            try:
                payload = json.loads(raw)
            except json.JSONDecodeError:
                payload = {"parse_error": True, "raw": raw}
            self.record(kind, payload)

        def bspline(self, message: Bspline) -> None:
            self.record("normal_bspline", {
                "trajectory_id": int(message.traj_id),
                "start_time_ns": int(message.start_time.sec) * 1_000_000_000
                + int(message.start_time.nanosec),
                "control_point_count": len(message.pos_pts),
                "knot_count": len(message.knots),
            })

        def imu(self, message: Imu) -> None:
            if self.imu_count >= 100:
                return
            self.imu_count += 1
            acc = message.linear_acceleration
            self.record("raw_imu", {
                "norm_mps2": math.sqrt(acc.x * acc.x + acc.y * acc.y
                                        + acc.z * acc.z),
            })

        def destroy_node(self):
            self.stream.close()
            return super().destroy_node()

    rclpy.init()
    node = Capture()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def _json_write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _node_names(environment: dict[str, str]) -> set[str]:
    completed = subprocess.run(
        ["ros2", "node", "list"], cwd=REPOSITORY, env=environment,
        capture_output=True, text=True, check=False, timeout=5.0)
    return {line.strip() for line in completed.stdout.splitlines()
            if line.strip()}


def _group_cleared(pid: int) -> bool:
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return True
    except PermissionError:
        return False
    return False


def _capture_group_diagnostics(pgid: int, path: Path) -> None:
    lines = [
        f"process_group={pgid}",
        f"captured_utc={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}",
    ]
    try:
        completed = subprocess.run(
            ["ps", "-e", "-o", "pid=,ppid=,pgid=,stat=,comm=,wchan:48="],
            capture_output=True, text=True, check=False, timeout=3.0)
        lines.extend(["", "processes:", completed.stdout.rstrip()])
        pids = []
        for row in completed.stdout.splitlines():
            fields = row.split(None, 5)
            if len(fields) >= 3 and fields[2] == str(pgid):
                pids.append(int(fields[0]))
        for pid in pids:
            task_root = Path(f"/proc/{pid}/task")
            for task in sorted(task_root.glob("*")):
                lines.extend(["", f"pid={pid} tid={task.name} kernel_stack:"])
                try:
                    lines.append((task / "stack").read_text().rstrip())
                except OSError as error:
                    lines.append(f"unavailable: {error}")
    except (OSError, subprocess.SubprocessError, ValueError) as error:
        lines.extend(["", f"diagnostic_error: {error}"])
    path.write_text("\n".join(lines) + "\n")


def _stop_group(
        process: subprocess.Popen, timeout_s: float,
        diagnostics_path: Path | None = None) -> tuple[int, bool, bool]:
    escalated = False
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        code = process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        escalated = True
        if diagnostics_path is not None:
            _capture_group_diagnostics(process.pid, diagnostics_path)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        code = process.wait(timeout=5.0)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not _group_cleared(process.pid):
        time.sleep(0.05)
    return code, _group_cleared(process.pid), escalated


def _wait_capture_ready(process: subprocess.Popen, path: Path) -> bool:
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            return False
        if path.is_file():
            try:
                if json.loads(path.read_text()).get("ready") is True:
                    return True
            except json.JSONDecodeError:
                pass
        time.sleep(0.05)
    return False


def _launch_shell(install_root: Path, launch_args: dict[str, str]) -> str:
    argv = ["ros2", "launch", "iap", "test_icra.launch.py"]
    argv.extend(f"{key}:={value}" for key, value in launch_args.items())
    return (
        "source /opt/ros/jazzy/setup.bash && "
        f"source {shlex.quote(str(install_root / 'setup.bash'))} && exec "
        + shlex.join(argv)
    )


def _run_one(
        stage: str, run_root: Path, install_root: Path,
        start_rviz: bool = False, shutdown_variant: str | None = None) -> dict:
    spec = STAGES[stage]
    run_root.mkdir(parents=True, exist_ok=False)
    for child in ("runtime/ros_logs", "exports", "bags"):
        (run_root / child).mkdir(parents=True, exist_ok=True)
    environment = {**os.environ, "ROS_LOG_DIR": str(run_root / "runtime/ros_logs")}
    baseline_nodes = _node_names(environment)
    capture_command = [
        sys.executable, str(Path(__file__).resolve()),
        "--capture-output", str(run_root / "capture.jsonl"),
        "--capture-ready", str(run_root / "capture_ready.json"),
        "--capture-duration", str(spec.duration_s + 20.0),
    ]
    capture_stream = (run_root / "capture_stdout.log").open("x")
    capture = subprocess.Popen(
        capture_command, cwd=REPOSITORY, env=environment,
        stdout=capture_stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    if not _wait_capture_ready(capture, run_root / "capture_ready.json"):
        capture_code, capture_cleared, _capture_escalated = _stop_group(
            capture, 2.0)
        capture_stream.close()
        summary = _result(
            ["capture_not_ready"], stage=stage,
            capture_exit_code=capture_code,
            capture_group_cleared=capture_cleared)
        _json_write(run_root / "summary.json", summary)
        return summary

    launch_args = dict(spec.launch_args)
    if shutdown_variant == "baseline":
        launch_args.update({
            "experiment": "baseline_fused_nominal_off",
            "scenario": "fused_nominal",
            "start_planner": "true",
            "planner_enable_p4": "false",
            "planner_enable_p5_final": "false",
            "planner_enable_p5_runtime": "false",
            "planner_start_delay_s": "3.0",
            "lidar_start_delay_s": "2.0",
            "odometry_initialization_mode": "NAIVE",
        })
    launch_args.update({
        "start_rviz": "true" if start_rviz else "false",
        "run_duration_s": "0" if stage == "shutdown" else str(spec.duration_s),
        "validation_duration_s": str(max(5.0, spec.duration_s - 5.0)),
        "runtime_root_dir": str(run_root / "runtime"),
        "export_root_dir": str(run_root / "exports"),
        "iap_log_root": str(run_root / "runtime/iap_logs"),
        "bag_output_dir": str(run_root / "bags"),
        "p4.debug_csv_path": str(
            run_root / "exports/planner_p4_risk_astar_debug.csv"),
    })
    shell_command = _launch_shell(install_root, launch_args)
    _json_write(run_root / "launch_command.json", {
        "stage": stage,
        "shutdown_variant": shutdown_variant,
        "argv": ["bash", "-lc", shell_command],
        "launch_args": launch_args,
    })
    stdout_path = run_root / "stdout.log"
    launch_stream = stdout_path.open("x")
    launch = subprocess.Popen(
        ["bash", "-lc", shell_command], cwd=REPOSITORY, env=environment,
        stdout=launch_stream, stderr=subprocess.STDOUT,
        start_new_session=True)
    started = time.monotonic()
    _json_write(run_root / "launch_started.json", {
        "schema_version": "icra_interface_launch_started_v1",
        "started_steady_s": started,
    })
    early_exit = False
    if stage == "shutdown":
        while time.monotonic() - started < spec.duration_s:
            if launch.poll() is not None:
                early_exit = True
                break
            time.sleep(0.1)
        launch_code, launch_cleared, launch_escalated = _stop_group(
            launch, 5.0, run_root / "shutdown_stacks.txt")
    else:
        deadline = started + spec.duration_s + 20.0
        while launch.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)
        if launch.poll() is None:
            launch_code, launch_cleared, launch_escalated = _stop_group(
                launch, 5.0, run_root / "timeout_stacks.txt")
        else:
            launch_code = launch.returncode
            launch_cleared = _group_cleared(launch.pid)
            launch_escalated = False
            early_exit = time.monotonic() - started < spec.duration_s - 1.0
    capture_code, capture_cleared, capture_escalated = _stop_group(
        capture, 3.0, run_root / "capture_stacks.txt")
    launch_stream.close()
    capture_stream.close()
    time.sleep(0.5)
    residual_nodes = sorted(_node_names(environment) - baseline_nodes)
    stdout = stdout_path.read_text(errors="replace")
    if stage == "shutdown":
        summary = analyze_shutdown(
            launch_code, stdout, launch_cleared, residual_nodes,
            runner_escalated=launch_escalated)
    else:
        summary = analyze_run(stage, run_root)
        extra = []
        if early_exit:
            extra.append("launch_exited_early")
        if launch_code != 0:
            extra.append("launch_exit_nonzero")
        if not launch_cleared or not capture_cleared:
            extra.append("owned_process_group_remaining")
        summary = _result(
            [*summary["failures"], *extra],
            **{key: value for key, value in summary.items()
               if key not in ("result", "failures")})
    summary.update({
        "stage": stage,
        "shutdown_variant": shutdown_variant,
        "launch_exit_code": launch_code,
        "capture_exit_code": capture_code,
        "launch_group_cleared": launch_cleared,
        "capture_group_cleared": capture_cleared,
        "launch_runner_escalated": launch_escalated,
        "capture_runner_escalated": capture_escalated,
        "residual_nodes": residual_nodes,
        "elapsed_s": time.monotonic() - started,
    })
    _json_write(run_root / "summary.json", summary)
    return summary


def _gpu_preflight(root: Path) -> dict:
    path = REPOSITORY / "scripts/dev_planner/run_gate0_qualification.py"
    spec = importlib.util.spec_from_file_location("icra_interface_gpu", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load GPU preflight")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root.mkdir(parents=True, exist_ok=False)
    return module.run_gpu_preflight(root)


def _session_root(results_root: Path) -> Path:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    candidate = results_root / f"run-{stamp}-{os.getpid()}"
    candidate.parent.mkdir(parents=True, exist_ok=True)
    return candidate


def _run_main(args: argparse.Namespace) -> int:
    install_root = args.install_root.resolve()
    if not (install_root / "setup.bash").is_file():
        raise SystemExit(f"install root is not ready: {install_root}")
    results_root = args.results_root.resolve()
    session = _session_root(results_root)
    session.mkdir(parents=True, exist_ok=False)
    preflight = _gpu_preflight(session / "preflight")
    if preflight.get("gpu_ready") is not True:
        print("GPU_NOT_READY")
        return 4
    requested = args.stage
    if args.through:
        requested = args.through
        stages = STAGE_ORDER[:STAGE_ORDER.index(requested) + 1]
    else:
        stages = (requested,)
    session_summary = {
        "schema_version": "icra_interface_integration_session_v1",
        "development_only": True,
        "qualification_claim": False,
        "scientific_effect_claim": False,
        "stage_order": list(stages),
        "repetitions": args.repetitions,
        "runs": [],
        # Do not leave a partial or interrupted --through session looking like
        # a completed acceptance.  PASS is written only after every requested
        # functional and shutdown repetition has finished successfully.
        "result": "RUNNING",
    }
    for stage in stages:
        variants = ("baseline", "full") if stage == "shutdown" else (None,)
        for repetition in range(1, args.repetitions + 1):
            for variant in variants:
                suffix = f"-{variant}" if variant else ""
                run_root = session / f"{stage}-r{repetition:02d}{suffix}"
                summary = _run_one(
                    stage, run_root, install_root,
                    start_rviz=args.rviz and stage == "full",
                    shutdown_variant=variant)
                session_summary["runs"].append({
                    "stage": stage,
                    "repetition": repetition,
                    "variant": variant,
                    "path": str(run_root),
                    "result": summary["result"],
                    "failures": summary["failures"],
                })
                _json_write(session / "session_summary.json", session_summary)
                if summary["result"] != "PASS":
                    session_summary["result"] = "FAIL"
                    session_summary["first_failed_stage"] = stage
                    _json_write(session / "session_summary.json", session_summary)
                    print(f"FAIL {stage} {run_root}")
                    return 1
    session_summary["result"] = "PASS"
    _json_write(session / "session_summary.json", session_summary)
    print(f"PASS {session}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--stage", choices=STAGE_ORDER, default="full")
    mode.add_argument("--through", choices=STAGE_ORDER)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--results-root", type=Path,
                        default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--install-root", type=Path,
                        default=DEFAULT_INSTALL_ROOT)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--capture-output", type=Path)
    parser.add_argument("--capture-ready", type=Path)
    parser.add_argument("--capture-duration", type=float, default=120.0)
    args = parser.parse_args()
    if args.capture_output or args.capture_ready:
        if not args.capture_output or not args.capture_ready:
            raise SystemExit("capture output and ready paths are both required")
        return _capture_main(args)
    if args.repetitions < 1:
        raise SystemExit("repetitions must be positive")
    if args.rviz and (args.stage != "full" or args.through):
        raise SystemExit("--rviz is valid only with --stage full")
    return _run_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
