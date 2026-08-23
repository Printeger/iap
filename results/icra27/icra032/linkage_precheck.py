#!/usr/bin/env python3
import json
import sys
from pathlib import Path

output = Path(sys.argv[1])
expected_iap = Path(sys.argv[2]).resolve()
expected_plan = Path(sys.argv[3]).resolve()
allowed = [Path(value).resolve() for value in sys.argv[4:9]]
logs = [Path(value) for value in sys.argv[9:]]

failures = []
records = {}
iap_total = 0
plan_total = 0
for path in logs:
    parsed = []
    for line in path.read_text().splitlines():
        fields = line.split()
        library = fields[0] if fields else ""
        resolved = None
        if "not found" in line:
            failures.append(f"not_found:{path.name}:{library}")
        elif len(fields) >= 3 and fields[1] == "=>" and fields[2].startswith("/"):
            resolved = Path(fields[2]).resolve()
            text = str(resolved)
            if "/results/icra27/" in text and "/build" in text:
                failures.append(f"build_tree_resolution:{path.name}:{text}")
            if "/results/icra27/" in text and not any(resolved == prefix or prefix in resolved.parents for prefix in allowed):
                failures.append(f"stale_task_resolution:{path.name}:{text}")
            if library == "libiap.so":
                iap_total += 1
                if resolved != expected_iap:
                    failures.append(f"unexpected_iap:{path.name}:{text}")
            if library == "libplan_env.so":
                plan_total += 1
                if resolved != expected_plan:
                    failures.append(f"unexpected_plan_env:{path.name}:{text}")
        parsed.append({"library": library, "path": str(resolved) if resolved else "", "line": line})
    records[path.name] = parsed
if iap_total == 0 or plan_total == 0:
    failures.append("direct_consumer_library_class_missing")
result = {
    "ready": not failures,
    "failures": failures,
    "expected_iap": str(expected_iap),
    "expected_plan_env": str(expected_plan),
    "allowed_prefixes": [str(path) for path in allowed],
    "iap_total": iap_total,
    "plan_env_total": plan_total,
    "records": records,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
