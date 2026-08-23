#!/usr/bin/env python3

import json
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix


output = Path(sys.argv[1])
required = (
    "iap",
    "ego_planner",
    "plan_env",
    "path_searching",
    "bspline_opt",
    "local_sensing",
    "odom_visualization",
    "poscmd_2_odom",
    "gnss_sim",
    "so3_quadrotor_simulator",
    "so3_control",
    "rclcpp_components",
)
expected = {
    "iap": Path("results/icra27/icra028/install").resolve(),
    "ego_planner": Path("results/icra27/icra026/install_ego").resolve(),
    "plan_env": Path("results/icra27/icra026/install_plan_env").resolve(),
    "path_searching": Path(
        "results/icra27/icra026/install_path_searching"
    ).resolve(),
    "bspline_opt": Path(
        "results/icra27/icra026/install_bspline_opt"
    ).resolve(),
}
raw_prefixes = os.environ.get("AMENT_PREFIX_PATH", "")
prefixes = [str(Path(value).resolve()) for value in raw_prefixes.split(":") if value]
failures = []
packages = []
for package in required:
    try:
        resolved = Path(get_package_prefix(package)).resolve()
        error = ""
    except Exception as exc:
        resolved = None
        error = f"{type(exc).__name__}: {exc}"
        failures.append(f"package_not_found:{package}")
    in_path = resolved is not None and str(resolved) in prefixes
    exact = package not in expected or resolved == expected[package]
    exists = resolved is not None and resolved.is_dir()
    if not in_path:
        failures.append(f"prefix_not_active:{package}")
    if not exact:
        failures.append(f"unexpected_prefix:{package}")
    if not exists:
        failures.append(f"prefix_not_directory:{package}")
    packages.append(
        {
            "package": package,
            "resolved_prefix": str(resolved) if resolved else "",
            "expected_prefix": str(expected.get(package, "")),
            "in_active_ament_prefix_path": in_path,
            "matches_exact_task_artifact": exact,
            "prefix_exists": exists,
            "error": error,
        }
    )

result = {
    "ament_prefix_path_raw": raw_prefixes,
    "ament_prefix_path": prefixes,
    "ld_library_path_raw": os.environ.get("LD_LIBRARY_PATH", ""),
    "required_packages": list(required),
    "packages": packages,
    "ready": not failures,
    "failures": failures,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
