#!/usr/bin/env python3
"""Reject maintained IAP artifact writers that bypass the run contract."""

from __future__ import annotations

import argparse
import hashlib
import re
from collections import Counter
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ALLOWLIST = PACKAGE_ROOT / "test/fixtures/run_artifact_legacy_allowlist.txt"
ABSOLUTE_MACHINE_PATH = re.compile(r"/(?:tmp|home/[A-Za-z0-9_.-]+)(?:/|\b)")
ARTIFACT_WORD = re.compile(
    r"(?:csv|log|dump|bag|output|export|artifact|manifest|summary|profiling|metadata)",
    re.IGNORECASE,
)
LEGACY_ROOT_CHILD = re.compile(
    r"(?:run_root|output_dir)\s*/\s*[\"'](?:logs|dump|bags|estimator|runtime_config)[\"']"
)
ROOT_MANIFEST = re.compile(
    r"(?:run_root|output_dir)\s*/\s*[\"'][^\"']*manifest\.json[\"']"
)


def maintained_files() -> list[Path]:
    files: list[Path] = []
    for path in (PACKAGE_ROOT / "config").rglob("*"):
        if path.suffix in {".json", ".yaml", ".yml"} and "icra27" not in path.parts:
            files.append(path)
    for root in (
        PACKAGE_ROOT / "apps",
        PACKAGE_ROOT / "include/iap",
        PACKAGE_ROOT / "src/iap",
        PACKAGE_ROOT / "launch",
    ):
        for path in root.rglob("*"):
            if not path.is_file() or "bp" in path.parts or "test" in path.parts:
                continue
            if path.suffix in {".cpp", ".hpp", ".py"}:
                files.append(path)
    files.append(PACKAGE_ROOT / "tools/ana_log.py")
    return sorted(set(files))


def fingerprint(kind: str, path: Path, line: str) -> str:
    relative = path.relative_to(PACKAGE_ROOT).as_posix()
    digest = hashlib.sha256(line.strip().encode("utf-8")).hexdigest()
    return f"{kind}\t{relative}\t{digest}"


def violations() -> Counter[str]:
    found: Counter[str] = Counter()
    for path in maintained_files():
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        is_config = "config" in path.relative_to(PACKAGE_ROOT).parts
        for line in lines:
            if ABSOLUTE_MACHINE_PATH.search(line) and (is_config or ARTIFACT_WORD.search(line)):
                found[fingerprint("absolute-artifact-path", path, line)] += 1
            if path.suffix == ".py" and LEGACY_ROOT_CHILD.search(line):
                found[fingerprint("legacy-run-root-child", path, line)] += 1
            if path.suffix == ".py" and ROOT_MANIFEST.search(line):
                found[fingerprint("root-manifest", path, line)] += 1
    return found


def read_allowlist(path: Path) -> Counter[str]:
    if not path.exists():
        return Counter()
    entries: Counter[str] = Counter()
    for line in path.read_text(encoding="utf-8").splitlines():
        if line and not line.startswith("#"):
            entries[line] += 1
    return entries


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--allowlist", type=Path, default=DEFAULT_ALLOWLIST)
    parser.add_argument("--update-allowlist", action="store_true")
    args = parser.parse_args()
    current = violations()
    if args.update_allowlist:
        args.allowlist.parent.mkdir(parents=True, exist_ok=True)
        lines = ["# Temporary migration debt; remove entries as writers migrate."]
        for entry, count in sorted(current.items()):
            lines.extend([entry] * count)
        args.allowlist.write_text("\n".join(lines) + "\n", encoding="utf-8")
        return 0
    allowed = read_allowlist(args.allowlist)
    unexpected = current - allowed
    stale = allowed - current
    if unexpected:
        print("new run-artifact contract violations:")
        for entry, count in sorted(unexpected.items()):
            print(f"  {count}x {entry}")
        return 1
    if stale:
        print("stale run-artifact allowlist entries; shrink the allowlist:")
        for entry, count in sorted(stale.items()):
            print(f"  {count}x {entry}")
        return 1
    print(f"run-artifact contract check passed ({sum(current.values())} migration entries)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
