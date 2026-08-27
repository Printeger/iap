#!/usr/bin/env python3
"""Classify the retained ICRA-075 matrix-002 P5 compatibility blocker."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
DEFAULT_RUN_ROOT = (
    REPOSITORY / "results/icra27/icra075/matrix-002/primary-75001-p0_p5_control")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative(path: Path) -> str:
    return str(path.resolve().relative_to(REPOSITORY))


def _single(root: Path, pattern: str) -> Path:
    matches = sorted({path.resolve() for path in root.glob(pattern)})
    if len(matches) != 1:
        raise ValueError(f"expected one {pattern}, observed {len(matches)}")
    return matches[0]


def _range(rows: list[dict], key: str) -> tuple[float, float]:
    values = [float(row[key]) for row in rows]
    return min(values), max(values)


def diagnose(run_root: Path) -> dict:
    root = run_root.resolve()
    capture_path = root / "lineage_capture.jsonl"
    launch_path = root / "launch_command.json"
    manifest_path = root / "run_manifest.json"
    validation_path = _single(root / "exports", "*/test_planner_integrity_validation.csv")
    araim_path = _single(root / "runtime/iap_logs", "*/export/iap_araim.csv")
    runtime_config_path = _single(
        root / "runtime/iap_logs", "*/metadata/config/config_gnss.json")
    production_manifest_path = _single(root / "exports", "*/test_planner_manifest.json")

    capture = [json.loads(line) for line in capture_path.read_text().splitlines()
               if line.strip()]
    p5 = [row["payload"] for row in capture if row.get("kind") == "p5_status"]
    if not p5 or any(row.get("phase") != "final" for row in p5):
        raise ValueError("retained capture must contain final P5 observations only")
    if any((row.get("action"), row.get("reason")) !=
           ("REQUEST_REPLAN", "current_low_margin") for row in p5):
        raise ValueError("retained P5 blocker is not uniformly current_low_margin")

    with validation_path.open(newline="") as stream:
        validation = list(csv.DictReader(stream))
    with araim_path.open(newline="") as stream:
        araim = [row for row in csv.DictReader(stream)
                 if row.get("row_type") == "epoch"]
    if not validation or not araim:
        raise ValueError("retained current-integrity evidence is empty")

    runtime_config = json.loads(runtime_config_path.read_text())["integrity"]
    production_manifest = json.loads(production_manifest_path.read_text())
    launch = json.loads(launch_path.read_text())
    row_manifest = json.loads(manifest_path.read_text())
    launch_arguments = dict(argument.split(":=", 1)
                            for argument in launch["launch_arguments"])
    gnss_scenario_argument = launch_arguments["gnss_scenario_file"]
    gnss_scenario_path = Path(gnss_scenario_argument).resolve()
    if not gnss_scenario_path.is_file():
        raise ValueError("retained GNSS scenario path is unavailable")
    hal = {float(row["HAL"]) for row in araim}
    val = {float(row["VAL"]) for row in araim}
    if hal != {10.0} or val != {20.0}:
        raise ValueError(f"unexpected retained alert limits: HAL={hal}, VAL={val}")
    if runtime_config.get("integrity_fusion_mode") != "max_pl":
        raise ValueError("retained runtime fusion mode is not max_pl")
    hpl_min, hpl_max = _range(validation, "hpl")
    vpl_min, vpl_max = _range(validation, "vpl")
    future_margin = [float(row["future_min_im"]) for row in p5]
    current_margin = [float(row["current_im_min"]) for row in p5]
    if min(hpl_min - 10.0, vpl_min - 20.0) <= 0.0:
        raise ValueError("retained PL does not prove alert-limit incompatibility")
    if min(future_margin) <= 0.3:
        raise ValueError("future gate is not isolated from the current blocker")

    source_files = [
        REPOSITORY / "src/iap/integrity/integrity_extension.cpp",
        REPOSITORY / "src/iap/integrity/integrity_monitor.cpp",
        REPOSITORY / "src/iap/planner/plan_manage/src/p5_runtime_integrity_gate.cpp",
        REPOSITORY / "launch/test_planner.launch.py",
        REPOSITORY / "launch/icra075_exploratory.launch.py",
    ]
    evidence_files = [capture_path, launch_path, manifest_path, validation_path,
                      araim_path, runtime_config_path, production_manifest_path]
    return {
        "schema_version": "icra075_p5_compatibility_diagnosis_v1",
        "task": "ICRA-075",
        "classification": "FROZEN_CONTRACT_INCOMPATIBLE",
        "exercised_source_head": row_manifest["source_binding"]["head_commit"],
        "development_only": True,
        "p5_final_status_count": len(p5),
        "unique_candidate_identity_count": len({
            (row["final_candidate_traj_id"], row["final_candidate_start_time_ns"])
            for row in p5}),
        "action_reason_counts": {"REQUEST_REPLAN/current_low_margin": len(p5)},
        "current_integrity_source_counts": dict(Counter(
            row["current_integrity_source"] for row in p5)),
        "fusion_mode_counts": dict(Counter(row["fusion_mode"] for row in validation)),
        "final_hpl_source_counts": dict(Counter(
            row["final_hpl_source"] for row in validation)),
        "final_vpl_source_counts": dict(Counter(
            row["final_vpl_source"] for row in validation)),
        "alert_limits_m": {"hal": 10.0, "val": 20.0},
        "observed_current_pl_m": {
            "hpl_min": hpl_min, "hpl_max": hpl_max,
            "vpl_min": vpl_min, "vpl_max": vpl_max,
        },
        "current_margin_m": {"minimum": min(current_margin),
                             "maximum": max(current_margin)},
        "future_margin_m": {"minimum": min(future_margin),
                            "maximum": max(future_margin)},
        "p5_current_replan_margin_m": 0.3,
        "units": "metres for AL, PL and integrity margins; seconds for stamps/ages",
        "frame_authority": "map",
        "stamp_authority": "IntegrityReport.header.stamp from monitor report stamp",
        "frame_authority_binding": {
            "value": "map",
            "source_path": "src/iap/integrity/integrity_extension.cpp",
            "source_symbol": "IntegrityExtension::publishIntegrity",
            "source_sha256": _sha256(
                REPOSITORY / "src/iap/integrity/integrity_extension.cpp"),
        },
        "stamp_authority_binding": {
            "value": "IntegrityReport.header.stamp from monitor report stamp",
            "report_source_path": "src/iap/integrity/integrity_monitor.cpp",
            "report_source_symbol": "IntegrityMonitor::processFrame",
            "report_source_sha256": _sha256(
                REPOSITORY / "src/iap/integrity/integrity_monitor.cpp"),
            "message_source_path": "src/iap/integrity/integrity_extension.cpp",
            "message_source_symbol": "IntegrityExtension::publishIntegrity",
            "message_source_sha256": _sha256(
                REPOSITORY / "src/iap/integrity/integrity_extension.cpp"),
        },
        "value_authority": {
            "current_hpl_vpl": {
                "message_fields": ["IntegrityReport.hpl", "IntegrityReport.vpl"],
                "fusion": "max_pl",
                "selected_source": "GNSS",
                "fusion_config_path": _relative(runtime_config_path),
                "fusion_config_key": "integrity.integrity_fusion_mode",
                "fusion_config_sha256": _sha256(runtime_config_path),
                "provider_launch_argument_key": "gnss_scenario_file",
                "provider_launch_argument_value": gnss_scenario_argument,
                "provider_config_resolved_path": str(gnss_scenario_path),
                "provider_config_sha256": _sha256(gnss_scenario_path),
                "retained_csv": _relative(validation_path),
                "retained_csv_sha256": _sha256(validation_path),
            },
            "current_hal_val": {
                "message_fields": ["IntegrityReport.hal", "IntegrityReport.val"],
                "runtime_config": _relative(runtime_config_path),
                "runtime_keys": ["integrity.HAL_trunk_default",
                                 "integrity.VAL_default"],
                "runtime_config_sha256": _sha256(runtime_config_path),
                "retained_csv": _relative(araim_path),
                "retained_csv_sha256": _sha256(araim_path),
            },
            "p5_current_gate": {
                "integrity_topic": "/iap/integrity",
                "source_fields": ["hpl", "vpl", "hal", "val", "im"],
                "threshold_source_path": "launch/test_planner.launch.py",
                "threshold_source_sha256": _sha256(
                    REPOSITORY / "launch/test_planner.launch.py"),
                "threshold_key": "p5.current_replan_margin_m",
                "launch_wrapper": "launch/icra075_exploratory.launch.py",
                "launch_wrapper_sha256": _sha256(
                    REPOSITORY / "launch/icra075_exploratory.launch.py"),
                "launch_command": _relative(launch_path),
                "launch_command_sha256": _sha256(launch_path),
                "production_manifest": _relative(production_manifest_path),
                "production_manifest_sha256": _sha256(production_manifest_path),
            },
        },
        "launch_arguments": launch["launch_arguments"],
        "requires_forbidden_contract_change_to_pass": True,
        "required_change_classes": [
            "GNSS/provider truth or PL definition/value",
            "max_pl fusion policy/source",
            "HAL/VAL definition/value",
            "P5 current margin/action threshold or hard-gate weakening",
        ],
        "repairable_miswiring_found": False,
        "reason": (
            "The current P5 gate consumes the intended fresh map-frame fused monitor report. "
            "Its frozen max_pl policy truthfully selects GNSS PL, whose minimum HPL/VPL "
            "already exceeds the frozen 10/20 m alert limits; all future samples remain "
            "above the replan margin. Passing therefore requires a forbidden contract change, "
            "not correction of a misrouted, misframed, mis-united, stale, or wrong-file value."),
        "evidence_sha256": {_relative(path): _sha256(path) for path in evidence_files},
        "authority_source_sha256": {_relative(path): _sha256(path) for path in source_files},
    }


def _write_new(output: Path, result: dict) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        stream.write(json.dumps(result, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = diagnose(args.run_root)
    output = args.output.resolve()
    _write_new(output, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
