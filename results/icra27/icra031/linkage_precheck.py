#!/usr/bin/env python3
import json
import sys
from pathlib import Path

manager_log, demo_log, consumer_log, output = map(Path, sys.argv[1:5])
expected_iap = Path(sys.argv[5]).resolve()
expected_plan = Path(sys.argv[6]).resolve()
allowed = [Path(value).resolve() for value in sys.argv[7:]]

def resolutions(path):
    parsed = []
    for line in path.read_text().splitlines():
        fields = line.split()
        if "not found" in line:
            parsed.append((fields[0] if fields else "", None, line))
        elif len(fields) >= 3 and fields[1] == "=>":
            parsed.append((fields[0], Path(fields[2]).resolve(), line))
    return parsed

failures = []
records = {}
for label, path in (("manager", manager_log), ("demo11", demo_log), ("direct_consumers", consumer_log)):
    entries = resolutions(path)
    records[label] = [{"library": lib, "path": str(resolved) if resolved else "", "line": line} for lib, resolved, line in entries]
    for library, resolved, _ in entries:
        if resolved is None:
            failures.append(f"not_found:{label}:{library}")
            continue
        text = str(resolved)
        if "/results/icra27/" in text and "/build" in text:
            failures.append(f"build_tree_resolution:{label}:{text}")
        if "/results/icra27/" in text and not any(resolved == prefix or prefix in resolved.parents for prefix in allowed):
            failures.append(f"stale_task_resolution:{label}:{text}")
        if library == "libiap.so" and resolved != expected_iap:
            failures.append(f"unexpected_iap:{label}:{text}")
        if library == "libplan_env.so" and resolved != expected_plan:
            failures.append(f"unexpected_plan_env:{label}:{text}")
manager_iap = [x for x in records["manager"] if x["library"] == "libiap.so"]
demo_iap = [x for x in records["demo11"] if x["library"] == "libiap.so"]
all_iap = [x for entries in records.values() for x in entries if x["library"] == "libiap.so"]
all_plan = [x for entries in records.values() for x in entries if x["library"] == "libplan_env.so"]
if len(manager_iap) != 1 or Path(manager_iap[0]["path"]) != expected_iap:
    failures.append("manager_iap_not_exactly_one")
if len(demo_iap) > 1 or any(Path(x["path"]) != expected_iap for x in demo_iap):
    failures.append("demo_iap_not_zero_or_exact")
if not all_iap or not all_plan:
    failures.append("direct_consumer_library_class_missing")
result = {
    "ready": not failures, "failures": failures,
    "expected_iap": str(expected_iap), "expected_plan_env": str(expected_plan),
    "allowed_prefixes": [str(path) for path in allowed],
    "manager_iap_total": len(manager_iap), "demo_iap_total": len(demo_iap),
    "iap_total": len(all_iap), "plan_env_total": len(all_plan), "records": records,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
