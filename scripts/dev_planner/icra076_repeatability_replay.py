#!/usr/bin/env python3
"""Retain 60 production-emitted flat-null measurements for ICRA-076."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import icra076_preregistration as CONTRACT


PROBE = Path("/home/dev/ws_iap/build/bspline_opt/icra076_repeatability_probe")
DEFAULT_SNAPSHOT = (
    CONTRACT.REPOSITORY / "config/icra27/icra076_flat_null_snapshot_v1.json")


def regular_file_record(path: Path, rendered_path: str) -> dict:
    if not path.is_file() or path.is_symlink():
        raise CONTRACT.Icra076Error("REPLAY_FILE_TYPE_INVALID", str(path))
    return {
        "path": rendered_path,
        "type": "regular_file",
        "size_bytes": path.stat().st_size,
        "sha256": CONTRACT.file_sha256(path),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    repository = CONTRACT.REPOSITORY.resolve()
    snapshot = CONTRACT._repository_input_path(args.snapshot, repository)
    output_root = CONTRACT.validate_output_path(
        args.output_root, repository, repository / "results/icra27/icra076")
    if not PROBE.is_file() or PROBE.is_symlink():
        raise CONTRACT.Icra076Error("REPLAY_PROBE_INVALID", str(PROBE))
    executable = regular_file_record(PROBE, str(PROBE))
    serialized_input = regular_file_record(
        snapshot, str(snapshot.relative_to(repository)))
    source_head = CONTRACT._run_git(repository, ["rev-parse", "HEAD"])
    protocol = CONTRACT._json(
        repository / "config/icra27/icra076_preregistration_v1.json")
    environment = CONTRACT.current_freeze_environment_binding(
        protocol, repository)
    output_root.mkdir(parents=True, exist_ok=False)
    records = []
    measurements = []
    for invocation_index in range(1, 61):
        argv = [
            str(PROBE), "--snapshot", str(snapshot),
            "--invocation-index", str(invocation_index)]
        completed = subprocess.run(
            argv, cwd=repository, capture_output=True, check=False)
        if completed.returncode != 0 or completed.stderr:
            raise CONTRACT.Icra076Error(
                "REPLAY_PROBE_PROCESS_FAILED",
                f"invocation={invocation_index},exit={completed.returncode}")
        measurement = CONTRACT.parse_probe_output(
            completed.stdout, invocation_index)
        output_path = output_root / f"measurement-{invocation_index:03d}.json"
        with output_path.open("xb") as stream:
            stream.write(completed.stdout)
        output_record = regular_file_record(output_path, output_path.name)
        if output_record["sha256"] != hashlib.sha256(completed.stdout).hexdigest():
            raise CONTRACT.Icra076Error("REPLAY_EMISSION_WRITE_DRIFT")
        records.append({
            "invocation_index": invocation_index,
            "argv": argv,
            "exit_code": completed.returncode,
            "output": output_record,
        })
        measurements.append(measurement)
    calculation = CONTRACT.calculate_measured_repeatability(measurements)
    manifest = {
        "schema_version": "icra077a_production_measured_replay_manifest_v2",
        "outcome_blind": True,
        "held_out_accessed": False,
        "source_head": source_head,
        "executable": executable,
        "serialized_input": serialized_input,
        "measurement_count": len(records),
        "measurements": records,
        "calculation": calculation,
        **environment,
    }
    manifest_path = output_root / "manifest.json"
    with manifest_path.open("x") as stream:
        stream.write(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "schema_version": manifest["schema_version"],
        "validation_result":
            CONTRACT.EVIDENCE_IDENTITY["validation_result"],
        "output": str(manifest_path),
        "measurement_count": len(records),
        "u95_repeatability_m": calculation["u95_repeatability_m"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
