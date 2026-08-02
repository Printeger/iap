#!/usr/bin/env python3
"""Close a launch manifest after its validator and rosbag recorder exit."""

import argparse
import json
import os
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--wait-timeout-s", type=float, default=15.0)
    parser.add_argument("--recorder-exit-code", type=int)
    parser.add_argument("--recorder-command", default="")
    args = parser.parse_args()
    path = Path(args.manifest).resolve()
    if not path.is_file():
        raise SystemExit(f"manifest does not exist: {path}")
    manifest = json.loads(path.read_text())
    provenance = manifest.get("artifact_provenance")
    if not isinstance(provenance, dict):
        raise SystemExit("manifest has no artifact_provenance")
    bag_path = Path(provenance.get("bag_path", ""))
    metadata_path = bag_path / "metadata.yaml"
    validator_path = path.parent / "test_planner_validation_summary.json"
    deadline = time.monotonic() + max(0.0, args.wait_timeout_s)
    while time.monotonic() < deadline and (not metadata_path.is_file() or not validator_path.is_file()):
        time.sleep(0.1)
    provenance.update({
        "process_end_stamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "process_end_epoch_s": time.time(),
        "bag_metadata_path": str(metadata_path.resolve()) if metadata_path.exists() else "",
        "bag_metadata_complete": metadata_path.is_file() and metadata_path.stat().st_size > 0,
        "validator_summary_path": str(validator_path.resolve()) if validator_path.exists() else "",
        "validator_summary_complete": validator_path.is_file() and validator_path.stat().st_size > 0,
        "recorder_exit_code": args.recorder_exit_code,
        "recorder_completed": args.recorder_exit_code == 0,
        "recorder_command": args.recorder_command,
    })
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    os.replace(temporary, path)
    print(json.dumps({"manifest": str(path), "run_id": provenance.get("run_id"),
                      "bag_metadata_complete": provenance["bag_metadata_complete"],
                      "validator_summary_complete": provenance["validator_summary_complete"],
                      "recorder_completed": provenance["recorder_completed"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
