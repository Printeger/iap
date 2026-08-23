#!/usr/bin/env python3

import importlib.util
import json
import sys
from pathlib import Path


output = Path(sys.argv[1])
repo_root = Path.cwd().resolve()
runner_path = repo_root / "scripts/dev_planner/run_gate0_qualification.py"
spec = importlib.util.spec_from_file_location("gate0_runner_icra030", runner_path)
if spec is None or spec.loader is None:
    raise RuntimeError("runner_import_spec_unavailable")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

run_dir = (repo_root / "results/icra27/icra030/runs/smoke").resolve()
config = runner.p0_effective_config(
    run_dir, run_duration_s=20, validation_duration_s=15
)
expected = {
    "experiment": "p0_open_sky",
    "scenario": "gnss_open_sky",
    "iap_mapping_backend": "cpu",
    "corridor_map_stamp_authority_topic": "/sim/drone_0/truth_odom",
    "planner_safety_profile": "off",
    "planner_enable_all_safety": False,
    "planner_enable_p1": False,
    "planner_enable_p2": False,
    "planner_enable_p3_local": False,
    "planner_enable_p3_global": False,
    "planner_enable_p4": False,
    "planner_enable_p5_runtime": False,
    "planner_enable_p5_final": False,
    "p0.enable_risk_grid": True,
    "p0.size_x_m": 30.0,
    "p0.size_y_m": 30.0,
    "p0.size_z_m": 6.0,
    "p0.resolution_m": 0.75,
    "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5",
    "p0.refresh_period_s": 0.5,
    "p0.predictor.worker_count": 4,
    "p0.skip_occupied_voxels": True,
    "p1.use_integrity_cost": False,
    "p1.metrics_only": False,
    "p2.enable_candidate_ranking": False,
    "p3.enable_local_reference_bias": False,
    "p3.enable_global_reference_bias": False,
    "p4.enable_risk_aware_astar": False,
    "p5.enable_runtime_gate": False,
    "p5.enable_final_gate": False,
    "record_bag": False,
    "start_rviz": False,
    "run_validator": False,
    "run_duration_s": 20,
    "validation_duration_s": 15,
    "runtime_root_dir": str(run_dir / "runtime"),
    "export_root_dir": str(run_dir / "exports"),
    "iap_log_root": str(run_dir / "runtime/iap_logs"),
}
failures = []
if config != expected:
    failures.append("effective_config_mismatch")

runtime_root = Path(str(config["runtime_root_dir"]))
iap_log_root = Path(str(config["iap_log_root"]))
timing_path = iap_log_root / "profiling/iap_timing.csv"
for label, path in (
    ("runtime_root", runtime_root),
    ("iap_log_root", iap_log_root),
    ("timing_path", timing_path),
):
    if not path.is_absolute():
        failures.append(f"not_absolute:{label}")
try:
    iap_log_root.relative_to(runtime_root)
    timing_path.relative_to(runtime_root)
except ValueError:
    failures.append("log_or_timing_not_below_runtime")
if iap_log_root == runtime_root or timing_path == runtime_root:
    failures.append("log_or_timing_not_strict_descendant")

horizons = str(config["p0.horizons_s"]).split(",")
query_count = (
    int(float(config["p0.size_x_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_y_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_z_m"]) / float(config["p0.resolution_m"]))
    * len(horizons)
)
if query_count != 76800:
    failures.append("query_count_not_76800")

result = {
    "ready": not failures,
    "failures": failures,
    "effective_config": config,
    "expected_config": expected,
    "derived_timing_path": str(timing_path),
    "query_count": query_count,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
