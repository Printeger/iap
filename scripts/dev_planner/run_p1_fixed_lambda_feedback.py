#!/usr/bin/env python3
"""Run the deterministic fixed-lambda P1 feedback loop without ROS launch."""

from __future__ import annotations

import argparse
import json
import subprocess
import time
from pathlib import Path


TARGETS = (
    (
        "bspline_opt/test_p1_integrity_cost",
        "P1IntegrityCostTest.FixedLambdaConflictFixtureCharacterizesLegacyRegression:"
        "P1IntegrityCostTest.LegacyOneStageCollapsesFourDistinctProjectedRiskSeeds:"
        "P1IntegrityCostTest.FixedLambdaTwoStagePreferenceDescendsRisk",
    ),
    (
        "ego_planner/test_p1_candidate_selection",
        "P1CandidateSelectionTest.RejectsLegacyFixedLambdaConflictAndSelectsNormalizedWinner",
    ),
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-root", type=Path, required=True)
    args = parser.parse_args()

    started = time.monotonic()
    results: list[dict[str, object]] = []
    green = True
    for relative_binary, test_filter in TARGETS:
        binary = (args.build_root / relative_binary).resolve()
        if not binary.is_file():
            results.append({
                "binary": str(binary),
                "filter": test_filter,
                "returncode": None,
                "executed": 0,
                "status": "missing_binary",
            })
            green = False
            continue
        completed = subprocess.run(
            [str(binary), f"--gtest_filter={test_filter}"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        executed = completed.stdout.count("[ RUN      ]")
        expected = test_filter.count(":") + 1
        passed = completed.returncode == 0 and executed == expected
        green = green and passed
        results.append({
            "binary": str(binary),
            "filter": test_filter,
            "returncode": completed.returncode,
            "executed": executed,
            "expected": expected,
            "status": "green" if passed else "red",
            "output": completed.stdout,
        })

    payload = {
        "schema": "p1_fixed_lambda_feedback_v1",
        "status": "green" if green else "red",
        "elapsed_s": time.monotonic() - started,
        "results": results,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if green else 1


if __name__ == "__main__":
    raise SystemExit(main())
