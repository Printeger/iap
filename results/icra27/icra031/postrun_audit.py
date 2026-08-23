#!/usr/bin/env python3
import json
import math
import sys
from collections import Counter
from pathlib import Path

output = Path(sys.argv[1])
repo_root = Path.cwd().resolve()
task_root = (repo_root / "results/icra27/icra031").resolve()
run_root = task_root / "runs"
smoke_root = run_root / "smoke"
runtime_root = smoke_root / "runtime"
failures = []

def load(path):
    return json.loads(path.read_text())

runner_exit = int((task_root / "smoke_runner_exit_code.txt").read_text().strip())
analyzer_exit = int((task_root / "smoke_analyzer_exit_code.txt").read_text().strip())
gpu = load(run_root / "gpu_preflight.json")
dependencies = load(run_root / "launch_dependency_preflight.json")
qualification = load(run_root / "p0_qualification_config_preflight.json")
manifest = load(smoke_root / "gate0_run_manifest.json")
log_preflight = load(smoke_root / "effective_log_path_preflight.json")
summary = load(smoke_root / "analyzer/p0_smoke_summary.json")
analysis = load(smoke_root / "analyzer/gate0_analysis.json")

planner_manifests = list((smoke_root / "exports").glob("*/test_planner_manifest.json"))
if len(planner_manifests) != 1:
    failures.append("test_planner_manifest_count_not_one")
planner_manifest = load(planner_manifests[0]) if len(planner_manifests) == 1 else {}

if runner_exit != 0 or manifest.get("exit_code") != 0 or manifest.get("capture_exit_code") != 0:
    failures.append("runner_or_capture_exit_nonzero")
if gpu.get("gpu_ready") is not True:
    failures.append("gpu_not_ready")
cuda = gpu.get("cuda", {})
if cuda.get("cuInit_result") != 0 or cuda.get("device_count", 0) < 1:
    failures.append("cuda_contract_failed")
if dependencies.get("launch_dependencies_ready") is not True or dependencies.get("failure_reasons") != []:
    failures.append("launch_dependencies_not_ready")
if qualification.get("qualification_config_ready") is not True:
    failures.append("qualification_config_not_ready")
for section in ("requested", "effective"):
    values = qualification.get(section, {})
    if values.get("p0.predictor.sigma_grow_m_sqrt_s") != 0.01:
        failures.append(f"sigma_not_exact:{section}")
    if values.get("p0.predictor.sigma_growth_profile") != "legacy_iap_rq320_baseline_v1":
        failures.append(f"profile_not_exact:{section}")
if qualification.get("baseline_provenance", {}).get("status") != "provisional_qualification_baseline_not_empirically_calibrated":
    failures.append("baseline_provenance_invalid")

if manifest.get("launch_started") is not True or manifest.get("planner_crash") is not False:
    failures.append("launch_or_planner_lifecycle_invalid")
if manifest.get("required_processes_ok") is not True:
    failures.append("required_processes_not_ok")
iap_process = manifest.get("required_processes", {}).get("iap_rosnode", {})
if iap_process.get("seen") is not True or iap_process.get("runtime_failure") is not False:
    failures.append("iap_rosnode_lifecycle_invalid")
for item in manifest.get("process_failures", []):
    if item.get("phase") != "controlled_shutdown":
        failures.append("runtime_process_failure")

config = manifest.get("effective_config", {})
frozen = {
    "iap_mapping_backend": "cpu", "p0.predictor.worker_count": 4,
    "p0.predictor.sigma_grow_m_sqrt_s": 0.01,
    "p0.predictor.sigma_growth_profile": "legacy_iap_rq320_baseline_v1",
    "run_duration_s": 20, "validation_duration_s": 15,
    "p0.size_x_m": 30.0, "p0.size_y_m": 30.0, "p0.size_z_m": 6.0,
    "p0.resolution_m": 0.75, "p0.horizons_s": "0.0,0.5,1.0,1.5,2.0,2.5",
    "p0.refresh_period_s": 0.5, "p0.skip_occupied_voxels": True,
    "record_bag": False, "start_rviz": False, "planner_safety_profile": "off",
    "planner_enable_p1": False, "planner_enable_p2": False,
    "planner_enable_p3_local": False, "planner_enable_p3_global": False,
    "planner_enable_p4": False, "planner_enable_p5_runtime": False,
    "planner_enable_p5_final": False,
}
for key, expected in frozen.items():
    if config.get(key) != expected:
        failures.append(f"frozen_config_mismatch:{key}")

def strict_descendant(path_text, root):
    path = Path(path_text)
    if not path.is_absolute():
        return False
    try:
        path.resolve().relative_to(root.resolve())
    except ValueError:
        return False
    return path.resolve() != root.resolve()

if log_preflight.get("effective_log_paths_ready") is not True:
    failures.append("effective_log_path_preflight_failed")
for key in ("requested_log_root", "derived_timing_path"):
    if not strict_descendant(str(log_preflight.get(key, "")), runtime_root):
        failures.append(f"preflight_path_not_below_runtime:{key}")
logging_config = planner_manifest.get("iap_logging_effective_config", {})
for key in ("log_root", "referenced_logging_config_path", "root_config_path", "timing_csv_path"):
    if not strict_descendant(str(logging_config.get(key, "")), runtime_root):
        failures.append(f"effective_logging_path_not_below_runtime:{key}")
iap_log_root = Path(str(logging_config.get("log_root", "")))
actual_logs = [path for path in iap_log_root.rglob("*") if path.is_file()]
timing_files = list(iap_log_root.rglob("iap_timing.csv"))
if not actual_logs or not timing_files:
    failures.append("iap_log_or_timing_output_missing")
for path in actual_logs:
    if not strict_descendant(str(path), runtime_root):
        failures.append(f"runtime_log_escaped:{path}")

health = [json.loads(line) for line in (smoke_root / "risk_grid_health.jsonl").read_text().splitlines() if line.strip()]
integrity = [json.loads(line) for line in (smoke_root / "integrity_report.jsonl").read_text().splitlines() if line.strip()]
reason_counts = Counter(str(row.get("reason", "")) for row in health)
generation_counts = Counter(str(row.get("generation_id")) for row in health)
finite_callback_count = sum(
    isinstance(row.get("refresh_callback_end_steady_s"), (int, float))
    and math.isfinite(float(row["refresh_callback_end_steady_s"])) for row in health
)
if summary.get("captured_observation_count") != len(health):
    failures.append("health_count_summary_mismatch")
if summary.get("integrity_report_count") != len(integrity):
    failures.append("integrity_count_summary_mismatch")
if summary.get("gate") != "P0_EVIDENCE_CONTRACT_FAIL":
    failures.append("unexpected_analyzer_classification")
if summary.get("successful_generation_count") != 0 or analyzer_exit != 1:
    failures.append("unexpected_live_outcome")
if analysis.get("gate0b") != summary or summary.get("manifest_failures") != []:
    failures.append("analyzer_evidence_inconsistent")
expected_failures = {
    "refresh_callback_end_steady_s_invalid", "zero_successful_generations",
    "fewer_than_required_successful_generations", "refresh_query_shape_mismatch",
}
if set(summary.get("failures", [])) != expected_failures:
    failures.append("analyzer_failures_changed")
bag_files = [str(path) for path in task_root.rglob("*") if path.is_file() and (path.suffix == ".db3" or path.name == "metadata.yaml")]
if bag_files:
    failures.append("unexpected_bag_output")

result = {
    "evidence_ready": not failures, "failures": failures,
    "runner_exit": runner_exit, "analyzer_exit": analyzer_exit,
    "gpu_ready": gpu.get("gpu_ready"), "cuda": cuda,
    "launch_dependencies_ready": dependencies.get("launch_dependencies_ready"),
    "qualification_config_ready": qualification.get("qualification_config_ready"),
    "required_processes_ok": manifest.get("required_processes_ok"),
    "iap_rosnode": iap_process, "controlled_shutdown_records": manifest.get("process_failures", []),
    "analyzer_gate": summary.get("gate"), "analyzer_failures": summary.get("failures"),
    "smoke_accepted": runner_exit == 0 and analyzer_exit == 0,
    "health_count": len(health), "finite_callback_identity_count": finite_callback_count,
    "integrity_count": len(integrity), "valid_integrity_count": summary.get("valid_integrity_report_count"),
    "successful_generation_count": summary.get("successful_generation_count"),
    "expected_refresh_query_count": summary.get("expected_refresh_query_count"),
    "reason_counts": dict(sorted(reason_counts.items())), "generation_counts": dict(sorted(generation_counts.items())),
    "effective_logging": logging_config, "actual_iap_log_file_count": len(actual_logs),
    "actual_timing_files": [str(path) for path in timing_files], "bag_files": bag_files,
}
output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result["evidence_ready"] else 1)
