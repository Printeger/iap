#!/usr/bin/env python3
"""Repository-local deterministic preflight for ICRA-033."""

import hashlib
import importlib.util
import json
import math
import os
import subprocess
from pathlib import Path


REPO = Path(__file__).resolve().parents[3]
TASK = REPO / "results/icra27/icra033"
IAP_INSTALL = TASK / "install"
EGO_INSTALL = TASK / "install_ego"
ICRA026 = REPO / "results/icra27/icra026"
OUTPUT = TASK / "preflight/static_preflight.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


runner_path = REPO / "scripts/dev_planner/run_gate0_qualification.py"
spec = importlib.util.spec_from_file_location("icra033_runner", runner_path)
if spec is None or spec.loader is None:
    raise RuntimeError("runner_import_failed")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

failures = []
config = runner.p0_effective_config(
    TASK / "runs/smoke", run_duration_s=20, validation_duration_s=15
)
expected = {
    "iap_mapping_backend": "cpu",
    "p0.predictor.worker_count": 4,
    "run_duration_s": 20,
    "validation_duration_s": 15,
    "p0.size_x_m": 30.0,
    "p0.size_y_m": 30.0,
    "p0.size_z_m": 6.0,
    "p0.resolution_m": 0.75,
    "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5",
    "p0.refresh_period_s": 0.5,
    "p0.skip_occupied_voxels": True,
    "record_bag": False,
    "start_rviz": False,
    "planner_safety_profile": "off",
    "planner_enable_all_safety": False,
    "p0.predictor.sigma_grow_m_sqrt_s": 0.01,
    "p0.predictor.sigma_growth_profile": "legacy_iap_rq320_baseline_v1",
}
disabled = (
    "planner_enable_p1", "planner_enable_p2", "planner_enable_p3_local",
    "planner_enable_p3_global", "planner_enable_p4", "planner_enable_p5_runtime",
    "planner_enable_p5_final", "p1.use_integrity_cost", "p1.metrics_only",
    "p2.enable_candidate_ranking", "p3.enable_local_reference_bias",
    "p3.enable_global_reference_bias", "p4.enable_risk_aware_astar",
    "p5.enable_runtime_gate", "p5.enable_final_gate",
)
failures.extend(
    f"config_mismatch:{key}" for key, value in expected.items()
    if config.get(key) != value
)
failures.extend(
    f"non_p0_enabled:{key}" for key in disabled if config.get(key) is not False
)
horizons = str(config["p0.horizons_s"]).split(",")
query_count = (
    int(float(config["p0.size_x_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_y_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_z_m"]) / float(config["p0.resolution_m"]))
    * len(horizons)
)
if query_count != 76800:
    failures.append("query_count_not_76800")
config_preflight = runner.run_p0_qualification_config_preflight(
    TASK / "preflight/config", config
)
if not config_preflight["qualification_config_ready"]:
    failures.append("qualification_config_preflight_failed")

from ament_index_python.packages import get_package_prefix

ament_expected = {
    "iap": IAP_INSTALL.resolve(),
    "ego_planner": EGO_INSTALL.resolve(),
    "plan_env": (ICRA026 / "install_plan_env").resolve(),
    "path_searching": (ICRA026 / "install_path_searching").resolve(),
    "bspline_opt": (ICRA026 / "install_bspline_opt").resolve(),
}
ament_packages = {}
active_prefixes = {
    Path(value).resolve()
    for value in os.environ.get("AMENT_PREFIX_PATH", "").split(":")
    if value
}
for package, expected_prefix in ament_expected.items():
    resolved_prefix = Path(get_package_prefix(package)).resolve()
    ament_packages[package] = str(resolved_prefix)
    if resolved_prefix != expected_prefix:
        failures.append(f"ament_prefix_mismatch:{package}:{resolved_prefix}")
    if resolved_prefix not in active_prefixes:
        failures.append(f"ament_prefix_inactive:{package}:{resolved_prefix}")

installed_analyzer = IAP_INSTALL / "lib/iap/gate0_analyzer.py"
if sha256(installed_analyzer) != sha256(REPO / "scripts/dev_planner/gate0_analyzer.py"):
    failures.append("installed_analyzer_not_current")

expected_iap = (IAP_INSTALL / "lib/libiap.so").resolve()
expected_plan = (ICRA026 / "install_plan_env/lib/libplan_env.so").resolve()
allowed_roots = tuple(
    path.resolve()
    for path in (
        IAP_INSTALL,
        EGO_INSTALL,
        ICRA026 / "install_plan_env",
        ICRA026 / "install_path_searching",
        ICRA026 / "install_bspline_opt",
    )
)
binaries = (
    TASK / "build_iap/test_risk_grid_map",
    TASK / "build_iap/test_rolling_spatial_advisory_window",
    TASK / "build_ego/test_p0_risk_grid_runtime",
    TASK / "build_ego/test_p0_occupancy_epoch_adapter",
    EGO_INSTALL / "lib/ego_planner/ego_planner_node",
)
linkage = {}
iap_links = 0
plan_links = 0
for binary in binaries:
    completed = subprocess.run(
        ["ldd", str(binary)], check=False, text=True, capture_output=True
    )
    if completed.returncode != 0:
        failures.append(f"ldd_failed:{binary.name}")
    resolved_rows = []
    for line in completed.stdout.splitlines():
        fields = line.split()
        if "not found" in line:
            failures.append(f"library_not_found:{binary.name}:{fields[0]}")
        if len(fields) >= 3 and fields[1] == "=>" and fields[2].startswith("/"):
            library = fields[0]
            # Preserve the loader's lexical install path.  Some retained
            # workspace installs are symlinked development installs; resolving
            # those symlinks would mislabel the loader-selected install entry.
            resolved = Path(fields[2]).absolute()
            if "/results/icra27/" in str(resolved) and not any(
                resolved == root or root in resolved.parents for root in allowed_roots
            ):
                failures.append(f"stale_task_link:{binary.name}:{resolved}")
            if "/build" in str(resolved) and "/results/icra27/" in str(resolved):
                failures.append(f"build_tree_link:{binary.name}:{resolved}")
            if library == "libiap.so":
                iap_links += 1
                if resolved != expected_iap:
                    failures.append(f"unexpected_iap_link:{binary.name}:{resolved}")
            if library == "libplan_env.so":
                plan_links += 1
                if resolved != expected_plan:
                    failures.append(f"unexpected_plan_link:{binary.name}:{resolved}")
            resolved_rows.append({"library": library, "path": str(resolved)})
    linkage[binary.name] = resolved_rows
if iap_links == 0 or plan_links == 0:
    failures.append("direct_consumer_library_class_missing")

result = {
    "schema_version": "icra033_static_preflight_v1",
    "ready": not failures,
    "failures": failures,
    "effective_config": config,
    "frozen_expected": expected,
    "disabled_expected": list(disabled),
    "query_count": query_count,
    "qualification_config_preflight": config_preflight,
    "expected_iap": str(expected_iap),
    "expected_plan_env": str(expected_plan),
    "ament_packages": ament_packages,
    "iap_link_count": iap_links,
    "plan_env_link_count": plan_links,
    "linkage": linkage,
    "artifact_sha256": {
        "installed_iap": sha256(expected_iap),
        "installed_ego": sha256(EGO_INSTALL / "lib/ego_planner/ego_planner_node"),
        "installed_analyzer": sha256(installed_analyzer),
    },
    "provisional_profile": True,
    "empirically_calibrated": False,
}
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
