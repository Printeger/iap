#!/usr/bin/env python3
"""Fail-closed recorder/finalizer preflight for one explicit evidence bundle."""

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--export-dir", required=True, type=Path)
    parser.add_argument("--bag-dir", required=True, type=Path)
    args = parser.parse_args()
    export_dir, bag_dir = args.export_dir.resolve(), args.bag_dir.resolve()
    manifest_path = export_dir / "test_planner_manifest.json"
    errors = []
    if not manifest_path.is_file():
        errors.append(f"missing manifest: {manifest_path}")
        manifest = {}
    else:
        manifest = json.loads(manifest_path.read_text())
    provenance = manifest.get("artifact_provenance", {})
    metadata = bag_dir / "metadata.yaml"
    if not bag_dir.is_dir(): errors.append(f"missing bag directory: {bag_dir}")
    if not metadata.is_file() or not metadata.stat().st_size:
        errors.append(f"missing or empty metadata: {metadata}")
    if Path(str(provenance.get("bag_path", ""))).resolve() != bag_dir:
        errors.append("manifest bag path does not match actual bag directory")
    if not provenance.get("process_end_stamp_utc"):
        errors.append("manifest lacks process_end_stamp_utc")
    if not provenance.get("bag_metadata_complete"):
        errors.append("manifest does not mark bag metadata complete")
    if provenance.get("recorder_completed") is not True:
        errors.append(f"recorder did not complete normally: {provenance.get('recorder_exit_code')}")
    if not provenance.get("recorder_command"):
        errors.append("manifest lacks recorder command")
    print(json.dumps({"passed": not errors, "errors": errors,
                      "manifest": str(manifest_path), "bag": str(bag_dir)}, indent=2))
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
