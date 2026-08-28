#!/usr/bin/env python3
"""Write one repository-local, source-bound ICRA-076 verification record."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import icra076_preregistration as contract


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--focused-tests-exit", type=int, required=True)
    parser.add_argument("--validator-exit", type=int, required=True)
    parser.add_argument("--six-package-build-exit", type=int, required=True)
    parser.add_argument("--repeatability-replay-exit", type=int, required=True)
    args = parser.parse_args()
    allowed = contract.REPOSITORY / "results/icra27/icra076"
    output = contract.validate_output_path(
        args.output, contract.REPOSITORY, allowed)
    admission = contract.source_admission(contract.REPOSITORY)
    protocol = contract._json(
        contract.REPOSITORY / "config/icra27/icra076_preregistration_v1.json")
    environment = contract.current_freeze_environment_binding(
        protocol, contract.REPOSITORY)
    exits = {
        "FOCUSED_TESTS": args.focused_tests_exit,
        "VALIDATOR": args.validator_exit,
        "SIX_PACKAGE_BUILD": args.six_package_build_exit,
        "REPEATABILITY_REPLAY": args.repeatability_replay_exit,
    }
    record = {
        "schema_version": "icra077a_repository_local_verification_v3",
        "source_head": admission["head_commit"],
        "commands": [
            {"category": category, "argv": argv, "enabled": True,
             "skipped": False, "exit_code": exits[category]}
            for category, argv in contract.expected_verification_argv().items()
        ],
        **environment,
    }
    contract.validate_verification(record, admission["head_commit"])
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        stream.write(json.dumps(record, indent=2, sort_keys=True) + "\n")
    print(json.dumps(record, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
