#!/usr/bin/env python3
"""Validate the outcome-blind ICRA-076 protocol without creating results."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import icra076_preregistration as CONTRACT


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--protocol", type=Path,
        default=CONTRACT.REPOSITORY / "config/icra27/icra076_preregistration_v1.json")
    parser.add_argument(
        "--seed-registry", type=Path,
        default=CONTRACT.REPOSITORY / "config/icra27/icra076_seed_registry_v1.json")
    parser.add_argument("--freeze-record", type=Path)
    args = parser.parse_args()
    if args.freeze_record is not None:
        result = CONTRACT.validate_freeze_record(args.freeze_record)
    else:
        validated = CONTRACT.validate_preregistration(
            args.protocol, args.seed_registry)
        result = {
            "schema_version": "icra076_protocol_validation_v1",
            "result": "PASS",
            "protocol_canonical_sha256": validated["protocol_canonical_sha256"],
            "execution_order_sha256": validated["execution_order_sha256"],
            "execution_order_count": len(validated["execution_order"]),
            "delta_peak_m": validated["delta_peak_m"],
            "minimum_success_count": validated["minimum_success_count"],
        }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
