#!/usr/bin/env python3
import importlib.util
import json
import sys
from pathlib import Path

output = Path(sys.argv[1])
repo_root = Path.cwd().resolve()
runner_path = repo_root / "scripts/dev_planner/run_gate0_qualification.py"
spec = importlib.util.spec_from_file_location("gate0_runner_icra032", runner_path)
if spec is None or spec.loader is None:
    raise RuntimeError("runner_import_spec_unavailable")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
run_dir = (repo_root / "results/icra27/icra032/runs/smoke").resolve()
config = runner.p0_effective_config(run_dir, run_duration_s=20, validation_duration_s=15)

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
failures = [f"unexpected:{key}" for key, value in expected.items() if config.get(key) != value]
failures.extend(f"non_p0_enabled:{key}" for key in disabled if config.get(key) is not False)

runtime_root = Path(str(config["runtime_root_dir"]))
iap_log_root = Path(str(config["iap_log_root"]))
timing_path = iap_log_root / "profiling/iap_timing.csv"
for label, path in (("runtime_root", runtime_root), ("iap_log_root", iap_log_root), ("timing_path", timing_path)):
    if not path.is_absolute():
        failures.append(f"not_absolute:{label}")
try:
    iap_log_root.relative_to(runtime_root)
    timing_path.relative_to(runtime_root)
except ValueError:
    failures.append("log_or_timing_not_below_runtime")

horizons = str(config["p0.horizons_s"]).split(",")
query_count = (
    int(float(config["p0.size_x_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_y_m"]) / float(config["p0.resolution_m"]))
    * int(float(config["p0.size_z_m"]) / float(config["p0.resolution_m"]))
    * len(horizons)
)
if query_count != 76800:
    failures.append("query_count_not_76800")
preflight = runner.run_p0_qualification_config_preflight(output.parent / "config_preflight", config)
if not preflight["qualification_config_ready"]:
    failures.append("qualification_config_preflight_failed")
result = {
    "ready": not failures,
    "failures": failures,
    "effective_config": config,
    "frozen_expected": expected,
    "disabled_expected": list(disabled),
    "baseline_provenance": "IAP-RQ-320 provisional qualification baseline; not empirically calibrated",
    "qualification_config_preflight": preflight,
    "derived_timing_path": str(timing_path),
    "query_count": query_count,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["ready"] else 1)
