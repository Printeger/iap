#!/usr/bin/env python3
"""Independent committed-final analyzer and power-input summarizer for ICRA-075."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
RESULTS_ROOT = (REPOSITORY / "results/icra27/icra075").resolve()
CORE_PATH = REPOSITORY / "scripts/dev_planner/icra075_exploratory.py"


def _load_core():
    spec = importlib.util.spec_from_file_location("icra075_core", CORE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load ICRA-075 analysis core")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CORE = _load_core()


def _contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _resolve(path: Path) -> Path:
    result = (path if path.is_absolute() else REPOSITORY / path).resolve()
    if not _contained(result, RESULTS_ROOT):
        raise SystemExit("ICRA-075 analyzer paths must be repository-local")
    return result


def _read_json(path: Path) -> dict:
    value = json.loads(path.read_text())
    if not isinstance(value, dict):
        raise ValueError(f"expected object: {path}")
    return value


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def _identity(payload: dict) -> tuple[int, int] | None:
    trajectory_id = payload.get("trajectory_id")
    start_ns = payload.get("trajectory_start_time_ns", payload.get("start_time_ns"))
    if type(trajectory_id) is int and trajectory_id > 0 and type(start_ns) is int and start_ns > 0:
        return trajectory_id, start_ns
    return None


def _truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def _positive_int(value: object) -> int | None:
    try:
        parsed = int(str(value))
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def analyze_row(root: Path) -> tuple[dict, int]:
    failures = []
    try:
        manifest = _read_json(root / "run_manifest.json")
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        manifest = {}
        failures.append(f"RUN_MANIFEST_MISSING_OR_INVALID:{type(exc).__name__}")
    row = manifest.get("matrix_row", {})
    try:
        descriptor = CORE.load_v2_descriptor(row.get("scene", ""))
    except Exception as exc:
        descriptor = None
        failures.append(f"DESCRIPTOR_INVALID:{type(exc).__name__}")
    if descriptor and descriptor.get("descriptor_sha256") != row.get("descriptor_sha256"):
        failures.append("DESCRIPTOR_HASH_MISMATCH")
    try:
        capture = [json.loads(line) for line in
                   (root / "lineage_capture.jsonl").read_text().splitlines() if line.strip()]
    except (OSError, json.JSONDecodeError) as exc:
        capture = []
        failures.append(f"CAPTURE_MISSING_OR_INVALID:{type(exc).__name__}")
    capture_binding_ok = bool(capture) and all(
        item.get("scene") == row.get("scene") and
        item.get("descriptor_sha256") == row.get("descriptor_sha256")
        for item in capture)
    if not capture_binding_ok:
        failures.append("CAPTURE_SCENE_DESCRIPTOR_BINDING_MISMATCH")
    p0_records = [item.get("payload", {}) for item in capture
                  if item.get("kind") == "p0_health" and
                  item.get("payload", {}).get("ready") is True and
                  item.get("payload", {}).get("stale") is False]
    p0_ready = bool(p0_records) and capture_binding_ok
    bsplines = [item["payload"] for item in capture
                if item.get("kind") == "normal_bspline" and
                _identity(item.get("payload", {})) is not None]
    final_payloads = [item["payload"] for item in capture
                      if item.get("kind") == "p5_status" and
                      item.get("payload", {}).get("phase") == "final" and
                      item.get("payload", {}).get("action") == "OK" and
                      not item.get("payload", {}).get("final_candidate_rejected", False)]
    runtime_payloads = [item["payload"] for item in capture
                        if item.get("kind") == "p5_status" and
                        item.get("payload", {}).get("phase") == "runtime" and
                        item.get("payload", {}).get("action") == "OK"]
    selected_final = selected_p5 = selected_runtime = None
    for final in reversed(bsplines):
        identity = _identity(final)
        matching_final = [payload for payload in final_payloads
                          if (payload.get("final_candidate_traj_id"),
                              payload.get("final_candidate_start_time_ns")) == identity]
        matching_runtime = [payload for payload in runtime_payloads if any(
            sample.get("trajectory_sample_source") == "runtime_committed" and
            (sample.get("trajectory_id"), sample.get("trajectory_start_time_ns")) == identity
            for sample in payload.get("samples", []))]
        if matching_final and matching_runtime:
            selected_final, selected_p5, selected_runtime = final, matching_final[-1], matching_runtime[-1]
            break
    p4_path = root / "exports/planner_p4_risk_astar_debug.csv"
    p4_rows = _read_csv(p4_path) if p4_path.is_file() else []
    lineage_path = Path(str(p4_path) + ".lineage.csv")
    lineage_rows = _read_csv(lineage_path) if lineage_path.is_file() else []
    configuration = row.get("configuration")
    if configuration == "P0_P5_CONTROL":
        p4_ok = manifest.get("launch_contract", {}).get("planner_enable_p4") is False
    elif configuration == "PROVIDER_BOTTLENECK_V2_METRICS_ONLY":
        p4_ok = bool(p4_rows) and manifest.get("launch_contract", {}).get("p4.metrics_only") is True
    else:
        p4_ok = any(item.get("selection_applied") == "1" for item in p4_rows)
    lineage_ok = True
    production_identity = None
    p4_stage_binding = None
    p4_enabled = manifest.get("launch_contract", {}).get("planner_enable_p4") is True
    expected_selection_applied = configuration != "PROVIDER_BOTTLENECK_V2_METRICS_ONLY"
    if p4_enabled and selected_final is not None:
        identity = _identity(selected_final)
        stages = {"final_bspline_before_p5", "p5_final_pass_before_publish",
                  "normal_publish_authorized"}
        matching_lineage = [item for item in lineage_rows
                            if item.get("schema_version") == "p4_v2_end_to_end_lineage_v2"
                            and _truthy(item.get("selection_applied")) ==
                            expected_selection_applied
                            and (_positive_int(item.get("trajectory_id")),
                                 _positive_int(item.get("trajectory_start_ns"))) == identity]
        identities = {item.get("final_bspline_identity") for item in matching_lineage
                      if item.get("final_bspline_identity")}
        generations = {item.get("snapshot_generation_id") for item in matching_lineage}
        ready_generations = {str(item.get("generation_id")) for item in p0_records}
        selected_generations = {item.get("snapshot_generation_id") for item in p4_rows
                                if _truthy(item.get("selection_applied")) ==
                                expected_selection_applied}
        lineage_ok = (
            stages.issubset({item.get("stage") for item in matching_lineage}) and
            len(identities) == 1 and len(generations) == 1 and
            generations <= ready_generations and generations <= selected_generations)
        if lineage_ok:
            production_identity = next(iter(identities))
            p4_stage_binding = {
                "stage": ("P4_SELECTION_APPLIED_BOUND" if expected_selection_applied
                          else "P4_METRICS_ONLY_NOT_APPLIED_BOUND"),
                "snapshot_generation_id": next(iter(generations)),
                "trajectory_id": identity[0],
                "trajectory_start_time_ns": identity[1],
                "production_final_bspline_identity": production_identity,
            }
        else:
            failures.append("P4_EGO_P5_PUBLICATION_LINEAGE_MISMATCH")
    elif not p4_enabled and selected_final is not None:
        identity = _identity(selected_final)
        ready_generations = sorted({str(item.get("generation_id")) for item in p0_records})
        lineage_ok = bool(ready_generations)
        p4_stage_binding = {
            "stage": "P4_DISABLED_BOUND",
            "snapshot_generation_ids": ready_generations,
            "trajectory_id": identity[0],
            "trajectory_start_time_ns": identity[1],
            "publication_identity": selected_final.get("publication_identity"),
        }
    stage_status = {
        "p0_snapshot": p0_ready,
        "p4_selection": p4_ok,
        "ego_final": selected_final is not None and lineage_ok,
        "p5_final": selected_p5 is not None,
        "publication": selected_final is not None,
        "p5_runtime": selected_runtime is not None,
    }
    missing = CORE.first_missing_stage(stage_status)
    if missing:
        failures.append(missing)
    if (manifest.get("result") != "PASS" or
            manifest.get("owned_process_groups_cleared") is not True):
        failures.append("RUNNER_OR_CLEANUP_NOT_PASS")
    analysis = {
        "schema_version": "p4_v2_inverse_corridor_analysis_v1",
        "result": "FAIL",
        "run_id": row.get("run_id", root.name),
        "scene": row.get("scene"),
        "seed": row.get("seed"),
        "configuration": configuration,
        "development_only": True,
        "effect_claim": False,
        "stage_status": stage_status,
        "first_missing_stage": missing,
        "failures": failures,
    }
    if descriptor and selected_final and selected_p5 and selected_runtime:
        margins = [float(sample["im_min"]) for sample in selected_p5.get("samples", [])
                   if isinstance(sample.get("im_min"), (int, float)) and
                   math.isfinite(float(sample["im_min"]))]
        selected_final = dict(selected_final)
        safety = CORE.evaluate_committed_final_safety(
            selected_final, CORE.load_map_asset(),
            float(manifest.get("launch_contract", {}).get("manager/max_vel", 0.0)),
            float(manifest.get("launch_contract", {}).get("manager/max_acc", 0.0)),
            float(manifest.get("launch_contract", {}).get(
                "manager/feasibility_tolerance", 0.0)))
        latency_values = []
        for item in p4_rows:
            try:
                value = float(item.get("risk_search_latency_ms", ""))
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and value >= 0.0:
                latency_values.append(value)
        p4_latency = 0.0 if configuration == "P0_P5_CONTROL" else (
            max(latency_values) if latency_values else None)
        selected_final.update({
            **safety,
            "p4_search_latency_ms": p4_latency,
            "p4_latency_applicability": (
                "NOT_APPLICABLE_P4_DISABLED" if configuration == "P0_P5_CONTROL"
                else "MEASURED_P4_DEBUG_CSV"),
        })
        runtime_identity = _identity(selected_final)
        oracle = CORE.analyze_committed_final(
            descriptor, selected_final,
            {"action": selected_p5.get("action"),
             "min_al_minus_pl_m": min(margins) if margins else None},
            {"action": selected_runtime.get("action"),
             "trajectory_id": runtime_identity[0],
             "trajectory_start_time_ns": runtime_identity[1]},
        )
        oracle.update(safety)
        oracle.update({
            "publication_identity": selected_final.get("publication_identity"),
            "production_final_bspline_identity": production_identity,
            "capture_scene_descriptor_binding": capture_binding_ok,
            "p4_latency_applicability": selected_final["p4_latency_applicability"],
            "p4_stage_binding": p4_stage_binding,
        })
        try:
            CORE.validate_complete_analysis(oracle)
        except ValueError as exc:
            failures.append(f"ANALYSIS_INCOMPLETE_OR_UNSAFE:{exc}")
        analysis.update(oracle)
        analysis.update({
            "run_id": row["run_id"], "seed": row["seed"],
            "configuration": configuration, "stage_status": stage_status,
            "result": "PASS" if not failures else "FAIL",
            "first_missing_stage": missing, "failures": failures,
        })
    return analysis, 0 if analysis["result"] == "PASS" else 1


def power_inputs(matrix_root: Path) -> tuple[dict, int]:
    expected = CORE.build_matrix(CORE.load_protocol())
    analyses = []
    attempts = []
    for row in expected:
        path = matrix_root / row["run_id"] / "analysis.json"
        try:
            analysis = _read_json(path)
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            analysis = {"run_id": row["run_id"], "result": "FAIL",
                        "first_missing_stage": "ANALYSIS_MISSING",
                        "failure": type(exc).__name__}
        analyses.append(analysis)
        attempts.append({"run_id": row["run_id"], "result": analysis.get("result"),
                         "first_missing_stage": analysis.get("first_missing_stage")})
    if any(item.get("result") != "PASS" for item in analyses):
        return {
            "schema_version": "icra075_exploratory_power_inputs_v1",
            "result": "FAIL",
            "development_only": True,
            "effect_claim": False,
            "freezes_sesoi": False,
            "freezes_sample_size": False,
            "first_missing_stage": "COMPLETE_40_ROW_MATRIX",
            "attempts": attempts,
        }, 1
    record = CORE.compute_power_inputs(analyses)
    record.update({"result": "PASS", "attempts": attempts,
                   "ablation_rows": [item for item in analyses
                                     if item["configuration"] in
                                     CORE.load_protocol()["primary_exploratory_configurations"]]})
    return record, 0


def main() -> int:
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--run-root", type=Path)
    mode.add_argument("--matrix-root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.run_root:
        root = _resolve(args.run_root)
        output = _resolve(args.output) if args.output else root / "analysis.json"
        if output.parent != root or output.exists():
            raise SystemExit("row analysis output must be new inside its row root")
        record, code = analyze_row(root)
    else:
        root = _resolve(args.matrix_root)
        output = (_resolve(args.output) if args.output else
                  root / "icra075_exploratory_power_inputs_v1.json")
        if output.parent != root or output.exists():
            raise SystemExit("power output must be new inside its matrix root")
        record, code = power_inputs(root)
    output.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(record["result"])
    return code


if __name__ == "__main__":
    raise SystemExit(main())
