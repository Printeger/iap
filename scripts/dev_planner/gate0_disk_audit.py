#!/usr/bin/env python3
"""Create a read-only disk/archive candidate inventory for ICRA Gate 0."""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import time
from collections import defaultdict
from pathlib import Path


FILE_MIN_BYTES = 100 * 1024 * 1024
DIRECTORY_MIN_BYTES = 1024 * 1024 * 1024
FIELDS = (
    "record_type", "path", "bytes", "type", "mtime_utc", "age_days",
    "age_bucket", "item_count",
)


def age_bucket(age_days: float) -> str:
    if age_days < 7:
        return "<7d"
    if age_days <= 30:
        return "7-30d"
    if age_days <= 90:
        return "31-90d"
    return ">90d"


def row_for_path(path: Path, size: int, now: float, kind: str) -> dict[str, object]:
    stat = path.stat()
    age = max(0.0, (now - stat.st_mtime) / 86400.0)
    suffix = "directory" if path.is_dir() else (path.suffix.lower() or "no_extension")
    return {
        "record_type": kind,
        "path": str(path.resolve()),
        "bytes": int(size),
        "type": suffix,
        "mtime_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(stat.st_mtime)),
        "age_days": f"{age:.6f}",
        "age_bucket": age_bucket(age),
        "item_count": 1,
    }


def inventory(results: Path, scan_roots: list[Path]) -> list[dict[str, object]]:
    now = time.time()
    rows = []
    seen_files = set()
    for root in scan_roots:
        if not root.exists():
            continue
        for directory, _, filenames in os.walk(root):
            for filename in filenames:
                path = Path(directory) / filename
                try:
                    size = path.stat().st_size
                except OSError:
                    continue
                resolved = path.resolve()
                if size >= FILE_MIN_BYTES and resolved not in seen_files:
                    seen_files.add(resolved)
                    rows.append(row_for_path(path, size, now, "candidate_file"))
    if results.exists():
        completed = subprocess.run(
            [
                "du", "-B1", f"--threshold={DIRECTORY_MIN_BYTES}",
                "--max-depth=4", str(results.resolve()),
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        for line in completed.stdout.splitlines():
            size_text, path_text = line.split("\t", 1)
            path = Path(path_text)
            if path.resolve() == results.resolve():
                continue
            rows.append(row_for_path(path, int(size_text), now, "candidate_directory"))

    aggregates: dict[tuple[str, str], dict[str, int]] = defaultdict(
        lambda: {"bytes": 0, "count": 0}
    )
    for row in rows:
        key = (str(row["age_bucket"]), str(row["type"]))
        aggregates[key]["bytes"] += int(row["bytes"])
        aggregates[key]["count"] += 1
    for (bucket, kind), values in sorted(aggregates.items()):
        rows.append({
            "record_type": "aggregate",
            "path": f"<aggregate:{bucket}:{kind}>",
            "bytes": values["bytes"],
            "type": kind,
            "mtime_utc": "",
            "age_days": "",
            "age_bucket": bucket,
            "item_count": values["count"],
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    rows = inventory(
        workspace / "src/iap/results",
        [workspace / "src/iap/results", workspace / "log", workspace / "build"],
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {len(rows)} rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
