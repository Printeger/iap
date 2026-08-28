#!/usr/bin/env python3
"""Create the sole pushed-source ICRA-076 preregistration freeze record."""

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
    parser.add_argument("--verification", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    verification = CONTRACT.load_verification(args.verification)
    record = CONTRACT.create_freeze_record(
        args.protocol, args.seed_registry, args.output, verification,
        verification_path=args.verification)
    print(json.dumps({
        "schema_version": record["schema_version"],
        "result": record["result"],
        "output": str(args.output.resolve()),
        "source_head": record["source_admission"]["head_commit"],
        "source_inventory_count": len(record["source_inventory"]),
        "install_inventory_count": len(record["install_inventory"]),
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
