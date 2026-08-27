#!/usr/bin/env python3
"""Capture 60 measured production flat-null snapshot replays for ICRA-076."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path

import icra076_preregistration as CONTRACT


BINARY = Path("/home/dev/ws_iap/build/bspline_opt/test_p4_collision_guide")
FILTER = (
    "--gtest_filter=P4CollisionGuideDecision."
    "Icra074FlatNullEqualCostsAndLengthUseStableHash")
FIXTURE = CONTRACT.REPOSITORY / (
    "src/iap/planner/bspline_opt/test/"
    "icra074_targeted_optimization_fixture.hpp")
DECISION_TEST = CONTRACT.REPOSITORY / (
    "src/iap/planner/bspline_opt/test/test_p4_collision_guide.cpp")


def _constant(source: str, name: str) -> float:
    match = re.search(
        rf"inline constexpr double {re.escape(name)}\s*=\s*([^;]+);", source)
    if match is None:
        raise RuntimeError(f"missing fixture constant {name}")
    return float(match.group(1))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = CONTRACT.validate_output_path(
        args.output, CONTRACT.REPOSITORY,
        CONTRACT.REPOSITORY / "results/icra27/icra076")
    fixture_source = FIXTURE.read_text()
    resolution = _constant(fixture_source, "kResolutionM")
    flat_cost = _constant(fixture_source, "kFlatCost")
    command = [str(BINARY), FILTER, "--gtest_repeat=1"]
    observations = []
    for replay_index in range(1, 61):
        completed = subprocess.run(
            command, cwd=CONTRACT.REPOSITORY, capture_output=True, text=True,
            check=False)
        transcript = completed.stdout + completed.stderr
        if completed.returncode != 0 or "[  PASSED  ] 1 test." not in transcript:
            raise RuntimeError(
                f"production replay {replay_index} failed: {completed.returncode}")
        snapshot = {
            "schema_version": "p4_risk_grid_snapshot_replay_v1",
            "provider_truth": "FLAT_NULL",
            "resolution_m": resolution,
            "original_interior_provider_c_pi_m": flat_cost,
            "risk_interior_provider_c_pi_m": flat_cost,
        }
        observations.append({
            "replay_index": replay_index,
            "command": command,
            "exit_code": completed.returncode,
            "transcript": transcript,
            "transcript_sha256": hashlib.sha256(
                transcript.encode("utf-8")).hexdigest(),
            "serialized_snapshot": snapshot,
            "serialized_snapshot_sha256": CONTRACT.canonical_sha256(snapshot),
        })
    record = {
        "schema_version": "icra076_measured_repeatability_replay_v1",
        "task": "ICRA-076",
        "outcome_blind": True,
        "held_out_accessed": False,
        "fixture": {
            "path": str(FIXTURE.relative_to(CONTRACT.REPOSITORY)),
            "sha256": CONTRACT.file_sha256(FIXTURE),
        },
        "decision_test": {
            "path": str(DECISION_TEST.relative_to(CONTRACT.REPOSITORY)),
            "sha256": CONTRACT.file_sha256(DECISION_TEST),
        },
        "observation_count": len(observations),
        "observations": observations,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        stream.write(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "schema_version": record["schema_version"],
        "result": "PASS",
        "output": str(output),
        "observation_count": len(observations),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
