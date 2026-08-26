#!/usr/bin/env python3
"""Fail-closed ICRA P0+P5 profile and synthetic qualification analyzer."""

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path


CONTRACT_SCHEMA = "icra_p0_p5_qualification_contract_v1"
EVIDENCE_SCHEMA = "icra_p0_p5_synthetic_evidence_v1"
RESULT_SCHEMA = "icra_p0_p5_validation_result_v1"
LIVE_EVIDENCE_SCHEMA = "icra_p0_p5_live_evidence_v1"
LIVE_RESULT_SCHEMA = "icra_p0_p5_live_result_v1"
CASE_IDS = ("SAFE_NORMAL", "FINAL_REJECT", "RUNTIME_FAIL")
FULL_SENSOR_SCENARIO = "icra_p0_p5_fused_degraded_corridor_v1"
FULL_SENSOR_TOPICS = (
    "/planning/risk_grid_health", "/planning/integrity_gate_status",
    "/drone_0_planning/bspline", "/sim/drone_0/imu",
    "/sim/drone_0/imu_iap", "/sim/drone_0/lidar",
    "/sim/drone_0/lidar_body", "/ublox_driver/range_meas",
    "/gnss_sim/diagnostics", "/iap/integrity",
)
FULL_SENSOR_QUALIFICATION_VALUES = {
    "use_gnss": True, "use_araim": True,
    "enable_gnss_integrity": True, "enable_gnss_araim": True,
    "enable_lidar_integrity": True, "integrity_fusion_mode": "max_pl",
    "validator_require_gnss_valid": True,
    "validator_require_lidar_valid": True,
    "gnss_time_source": "trigger_topic", "gnss_ephemeris_source": "rinex",
    "gnss_scenario_file": (
        "/home/dev/ws_iap/src/iap/results/icra27/icra070/install_v2/"
        "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml"
    ),
    "gnss_rinex_nav_file": (
        "/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx"
    ),
    "gnss_trigger_topic": "/sim/drone_0/lidar",
    "gnss_fallback_to_synthetic_on_rinex_error": False,
    "gnss_enabled_constellations": "GPS,BDS,GAL,GLO",
    "gnss_pr_noise_base": 5.0, "gnss_dop_noise_base": 0.5,
    "gnss_enable_map_occlusion": True, "gnss_enable_skymask": True,
    "gnss_enable_nlos": True, "gnss_enable_multipath": True,
    "gnss_enable_fault_injection": False,
}
FULL_SENSOR_PROFILE_VALUES = {
    "p0.predictor.source_mode": "fusion",
    "p0.predictor.gnss_epoch_policy": "auto",
    "p0.predictor.use_current_integrity_prior": True,
    "p0.predictor.worker_count": 4,
    "p0.predictor.sigma_grow_m_sqrt_s": 0.01,
    "p0.predictor.sigma_growth_profile": "legacy_iap_rq320_baseline_v1",
    "planner_enable_p1": False, "planner_enable_p2": False,
    "planner_enable_p3_local": False, "planner_enable_p3_global": False,
    "planner_enable_p4": False, "planner_enable_p5_runtime": True,
    "planner_enable_p5_final": True,
}
LIVE_RUN_IDENTITIES = {
    "SAFE_NORMAL": "icra-p0-p5-live-safe-normal-001",
    "FINAL_REJECT": "icra-p0-p5-live-final-reject-001",
    "RUNTIME_FAIL": "icra-p0-p5-live-runtime-fail-001",
}
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SOURCE_REPOSITORY = Path(__file__).resolve().parents[1]
ICRA068_LOCAL_PACKAGES = (
    "iap", "bspline_opt", "path_searching", "plan_env", "ego_planner",
    "traj_utils", "cmake_utils", "odom_visualization", "pose_utils",
    "quadrotor_msgs", "uav_utils", "poscmd_2_odom", "gnss_sim",
    "local_sensing", "so3_control", "so3_quadrotor_simulator", "gnss_comm",
)
ICRA068_EXECUTABLE_PATHS = (
    "lib/iap/demo11_corridor_map_publisher", "lib/iap/demo4_lidar_body_bridge",
    "lib/iap/iap_rosnode", "lib/iap/planner_evidence_provenance_publisher.py",
    "lib/iap/test_araim_validator.py",
    "lib/iap/planner_bag_recorder_with_finalizer.py",
    "lib/ego_planner/ego_planner_node", "lib/ego_planner/traj_server",
    "lib/gnss_sim/gnss_sim_node", "lib/local_sensing/pcl_render_node",
    "lib/odom_visualization/odom_visualization",
    "lib/poscmd_2_odom/poscmd_2_odom",
    "lib/so3_quadrotor_simulator/so3_quadrotor_simulator",
)
ICRA068_CONFIG_PATHS = (
    "share/iap/config/config_odometry_gpu.json",
    "share/iap/config/gnss_sim/demo7_skymask_nlos.yaml",
    "share/iap/config/sim_demo11/config.json",
    "share/iap/config/sim_demo11/config_gnss.json",
    "share/iap/config/sim_demo11/config_ros.json",
    "share/iap/config/sim_ego/config_global_mapping_gpu.json",
    "share/iap/config/sim_ego/config_logging.json",
    "share/iap/config/sim_ego/config_sensors.json",
    "share/iap/config/sim_ego/config_sub_mapping_gpu.json",
    "share/iap/config/sim_ego/config_viewer.json",
    "share/iap/config/sim_ego/fastdds_udp_only.xml",
    "share/local_sensing/config/camera.yaml",
    "share/so3_control/config/corrections_hummingbird.yaml",
    "share/so3_control/config/gains_hummingbird.yaml",
    "share/iap/config/icra27/icra_p0_p5_qualification_v1.json",
)
ICRA068_ALIAS_PATHS = {
    "share/iap/launch/test_planner.launch.py": "launch/test_planner.launch.py",
    "share/iap/launch/icra_p0_p5_qualification.py": "launch/icra_p0_p5_qualification.py",
    "share/iap/config/icra27/icra_p0_p5_qualification_v1.json": (
        "config/icra27/icra_p0_p5_qualification_v1.json"
    ),
}
ICRA068_LIBRARY_ROOTS = {
    "lib/libglobal_mapping.so", "lib/libgnss_extension.so",
    "lib/libintegrity_extension.so", "lib/libodometry_estimation_gpu.so",
    "lib/libsim_extension.so", "lib/libsub_mapping.so",
    "lib/libso3_control_component.so", "lib/libiap.so",
    "lib/libgnss_comm_lib.so",
}
ICRA068_MANIFEST_KEYS = {
    "schema_version", "git_commit", "install_root", "active_prefixes",
    "packages", "installed_aliases", "runtime_libraries", "file_hashes",
    "linkage_output_sha256", "build_profile", "closure_ready",
}


class ContractError(RuntimeError):
    """A profile, manifest, or evidence value violates the frozen contract."""


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def evidence_sha256(run):
    payload = json.dumps(
        run, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _evidence_sha256_or_invalid(value):
    try:
        return evidence_sha256(value)
    except (TypeError, ValueError):
        return "INVALID"


def p0_profile_binding(contract):
    values = contract["profile_values"]
    return {
        "worker_count": values["p0.predictor.worker_count"],
        "sigma_grow_m_sqrt_s": values["p0.predictor.sigma_grow_m_sqrt_s"],
        "sigma_growth_profile": values["p0.predictor.sigma_growth_profile"],
        "horizons_s": values["p0.horizons_s"],
        "resolution_m": values["p0.resolution_m"],
        "size_m": [
            values["p0.size_x_m"], values["p0.size_y_m"],
            values["p0.size_z_m"],
        ],
        "refresh_period_s": values["p0.refresh_period_s"],
        "stale_timeout_s": values["p0.stale_timeout_s"],
    }


def build_launch_binding(
    contract, contract_path, case_id, git_commit, run_id, effective_values
):
    expected = (
        _case_values(contract, case_id)
        if case_id is not None else _profile_values(contract)
    )
    if effective_values != expected:
        raise ContractError("launch binding effective values mismatch")
    fixture_alias = (
        contract["cases"][case_id]["fixture_alias"]
        if case_id is not None else "none_v1"
    )
    return {
        "schema_version": contract["schema_version"],
        "route_id": contract["route_id"],
        "profile_name": contract["profile_name"],
        "qualification_family": contract["qualification_family"],
        "case_id": case_id,
        "git_commit": git_commit,
        "run_id": run_id,
        "effective_values": effective_values,
        "p0_profile": p0_profile_binding(contract),
        "p5_thresholds": dict(contract["p5_thresholds"]),
        "fixture_alias": fixture_alias,
        "analyzer_version": contract["analyzer_version"],
        "contract_path": contract["contract_path"],
        "contract_sha256": _sha256(contract_path),
        "raw_artifact_hashes": "REQUIRED_AT_ANALYSIS",
        "qualification_status": "NOT_RUN",
    }


def _require_mapping(value, label):
    if not isinstance(value, dict):
        raise ContractError(f"{label} must be an object")
    return value


def load_contract(path):
    path = Path(path)
    try:
        contract = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"cannot load qualification contract {path}: {exc}") from exc
    _require_mapping(contract, "contract")
    if contract.get("schema_version") != CONTRACT_SCHEMA:
        raise ContractError("wrong qualification contract schema_version")
    if tuple(contract.get("cases", {}).keys()) != CASE_IDS:
        raise ContractError("contract must register SAFE_NORMAL, FINAL_REJECT, RUNTIME_FAIL in order")
    for key in (
        "route_id", "profile_name", "qualification_family", "analyzer_version",
        "contract_path",
        "profile_values", "qualification_values", "p5_thresholds", "fixture_switches",
        "required_processes", "required_topics",
    ):
        if key not in contract:
            raise ContractError(f"contract missing {key}")
    if contract["profile_name"] != "icra_p0_p5":
        raise ContractError("contract profile_name is not icra_p0_p5")
    if tuple(contract["required_topics"]) != FULL_SENSOR_TOPICS \
            or len(contract["required_processes"]) != 16 \
            or "test_planner_gnss_sim_node" not in contract["required_processes"] \
            or any(
                contract["cases"][case_id].get("scenario") != FULL_SENSOR_SCENARIO
                for case_id in CASE_IDS
            ) \
            or any(
                contract["qualification_values"].get(key) != expected
                for key, expected in FULL_SENSOR_QUALIFICATION_VALUES.items()
            ) \
            or any(
                contract["profile_values"].get(key) != expected
                for key, expected in FULL_SENSOR_PROFILE_VALUES.items()
            ):
        raise ContractError("contract full-sensor qualification mismatch")
    return contract


def _profile_values(contract):
    values = dict(contract["profile_values"])
    values.update(contract["p5_thresholds"])
    for switch in contract["fixture_switches"]:
        values[switch] = False
    return values


def _case_values(contract, case_id):
    cases = contract["cases"]
    if case_id not in cases:
        raise ContractError(f"unregistered qualification case {case_id!r}")
    values = _profile_values(contract)
    values.update(contract["qualification_values"])
    values.update(cases[case_id].get("fixture_values", {}))
    values["scenario"] = cases[case_id]["scenario"]
    values["experiment"] = cases[case_id]["experiment"]
    return values


def resolve_launch_values(contract, case_id, explicit_overrides):
    """Return the exact arm values, rejecting rather than coercing conflicts."""
    expected = _case_values(contract, case_id)
    overrides = _require_mapping(explicit_overrides, "explicit_overrides")
    protected_prefixes = ("p0_6.fixture.", "p5_3.fixture.", "p5_4.fixture.",
                          "p5_5.fixture.", "p5_6.fixture.", "p5_7.fixture.")
    protected = set(expected)
    for key, actual in overrides.items():
        if key in protected:
            if actual != expected[key]:
                raise ContractError(
                    f"conflicting explicit override for {key}: {actual!r} != {expected[key]!r}"
                )
        elif key.startswith(protected_prefixes):
            raise ContractError(f"unregistered fixture override for {key}")
    return expected


def resolve_profile_values(contract, explicit_overrides):
    """Resolve the named profile without arming any qualification fixture."""
    expected = _profile_values(contract)
    overrides = _require_mapping(explicit_overrides, "explicit_overrides")
    protected_prefixes = ("p0_6.fixture.", "p5_3.fixture.", "p5_4.fixture.",
                          "p5_5.fixture.", "p5_6.fixture.", "p5_7.fixture.")
    for key, actual in overrides.items():
        if key in expected and actual != expected[key]:
            raise ContractError(
                f"conflicting explicit override for {key}: {actual!r} != {expected[key]!r}"
            )
        if key not in expected and key.startswith(protected_prefixes):
            raise ContractError(f"unregistered fixture override for {key}")
    return expected


def _finite_tree(value, label, failures):
    if isinstance(value, float) and not math.isfinite(value):
        failures.append(f"{label}: non-finite number")
    elif isinstance(value, dict):
        for key, child in value.items():
            _finite_tree(child, f"{label}.{key}", failures)
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _finite_tree(child, f"{label}[{index}]", failures)


def _validate_common_run(contract, run, failures):
    case_id = run.get("case_id", "<missing>")
    prefix = f"run[{case_id}]"
    if not isinstance(run.get("run_id"), str) or not run["run_id"].strip():
        failures.append(f"{prefix}: missing run identity")
    case = contract["cases"].get(case_id)
    if case is None:
        failures.append(f"{prefix}: unregistered case")
        return
    if run.get("fixture_alias") != case["fixture_alias"]:
        failures.append(f"{prefix}: fixture leakage or wrong alias")
    processes = run.get("processes")
    if not isinstance(processes, list):
        failures.append(f"{prefix}: malformed process evidence")
    else:
        identities = [process.get("identity") for process in processes if isinstance(process, dict)]
        if len(identities) != len(set(identities)) or set(identities) != set(contract["required_processes"]):
            failures.append(f"{prefix}: missing or duplicate process identities")
        if any(
            not isinstance(process, dict)
            or process.get("alive_until_controlled_shutdown") is not True
            for process in processes
        ):
            failures.append(f"{prefix}: process death before controlled shutdown")
    if run.get("controlled_shutdown") is not True:
        failures.append(f"{prefix}: uncontrolled shutdown")
    topics = run.get("topic_counts")
    if not isinstance(topics, dict) or set(topics) != set(contract["required_topics"]):
        failures.append(f"{prefix}: topic identity gap")
    elif any(type(count) is not int or count <= 0 for count in topics.values()):
        failures.append(f"{prefix}: topic sample gap")
    samples = run.get("p0_samples")
    if not isinstance(samples, list) or len(samples) < 2:
        failures.append(f"{prefix}: missing P0 readiness samples")
    else:
        sequences = []
        for sample in samples:
            if not isinstance(sample, dict):
                failures.append(f"{prefix}: malformed P0 row")
                continue
            sequences.append(sample.get("sequence"))
            refresh_s = sample.get("refresh_s")
            if sample.get("ready") is not True or sample.get("stable") is not True:
                failures.append(f"{prefix}: unstable P0")
            if type(refresh_s) not in (int, float) or not math.isfinite(refresh_s) or refresh_s < 0:
                failures.append(f"{prefix}: malformed or non-finite P0 row")
            if not (
                sample.get("gnss_epoch_seen") is True
                and sample.get("gnss_epoch_valid") is True
                and sample.get("gnss_epoch_fresh") is True
                and type(sample.get("predictor_gnss_used_count")) is int
                and sample["predictor_gnss_used_count"] > 0
                and type(sample.get("predictor_lidar_used_count")) is int
                and sample["predictor_lidar_used_count"] > 0
                and type(sample.get("predictor_horizon_fusion_count")) is int
                and sample["predictor_horizon_fusion_count"] > 0
            ):
                failures.append(f"{prefix}: full-sensor P0 evidence mismatch")
        if any(type(value) is not int for value in sequences) or sequences != sorted(set(sequences)):
            failures.append(f"{prefix}: malformed or duplicate P0 sequence")
    integrity_samples = run.get("integrity_samples")
    if not isinstance(integrity_samples, list) or not integrity_samples:
        failures.append(f"{prefix}: missing full-sensor integrity evidence")
    else:
        integrity_sequences = []
        for sample in integrity_samples:
            if not isinstance(sample, dict):
                failures.append(f"{prefix}: malformed full-sensor integrity row")
                continue
            integrity_sequences.append(sample.get("sequence"))
            if sample.get("valid") is not True \
                    or type(sample.get("n_sv_used")) is not int \
                    or sample["n_sv_used"] <= 0:
                failures.append(f"{prefix}: full-sensor satellite evidence mismatch")
        if any(type(value) is not int for value in integrity_sequences) \
                or integrity_sequences != sorted(set(integrity_sequences)):
            failures.append(f"{prefix}: malformed full-sensor integrity sequence")
    events = run.get("events")
    if not isinstance(events, list):
        failures.append(f"{prefix}: malformed events")
        return
    sequences = []
    for event in events:
        if not isinstance(event, dict) or not isinstance(event.get("type"), str):
            failures.append(f"{prefix}: malformed event row")
            continue
        sequences.append(event.get("sequence"))
        if not isinstance(event.get("candidate_id"), str) or not event["candidate_id"]:
            failures.append(f"{prefix}: missing candidate identity")
    if any(type(value) is not int for value in sequences) or sequences != sorted(set(sequences)):
        failures.append(f"{prefix}: malformed or duplicate event sequence")


def _events(run, event_type):
    events = run.get("events")
    if not isinstance(events, list):
        return []
    return [event for event in events if isinstance(event, dict) and event.get("type") == event_type]


def _validate_case(contract, run, failures):
    case_id = run.get("case_id")
    prefix = f"run[{case_id}]"
    accepts = _events(run, "FINAL_ACCEPT")
    rejects = _events(run, "FINAL_REJECT")
    publishes = _events(run, "NORMAL_PUBLISH")
    runtime = _events(run, "RUNTIME_ACTION")
    if case_id == "SAFE_NORMAL":
        if len(accepts) != 1 or len(publishes) != 1:
            failures.append(f"{prefix}: safe case requires one accept and one normal publish")
        elif accepts[0].get("candidate_id") != publishes[0].get("candidate_id") \
                or accepts[0].get("sequence", 0) >= publishes[0].get("sequence", 0):
            failures.append(f"{prefix}: final accept must precede matching safe publish")
        if rejects or runtime:
            failures.append(f"{prefix}: false final/runtime trigger in safe case")
    elif case_id == "FINAL_REJECT":
        if len(rejects) != 1 or accepts or runtime:
            failures.append(f"{prefix}: final case requires one isolated rejection")
        if rejects:
            identity = rejects[0].get("candidate_id")
            if rejects[0].get("reason") != "p5_7_rejected_trajectory":
                failures.append(f"{prefix}: wrong registered final rejection reason")
            if any(event.get("candidate_id") == identity for event in publishes):
                failures.append(f"{prefix}: rejected identity was normally published")
    elif case_id == "RUNTIME_FAIL":
        if len(accepts) != 1 or len(publishes) != 1 or len(runtime) != 1 or rejects:
            failures.append(f"{prefix}: runtime case requires accept, publish, then one runtime action")
        elif not (
            accepts[0].get("candidate_id") == publishes[0].get("candidate_id")
            == runtime[0].get("candidate_id")
            and accepts[0].get("sequence", 0) < publishes[0].get("sequence", 0)
            < runtime[0].get("sequence", 0)
        ):
            failures.append(f"{prefix}: runtime action must follow matching accepted publication")
        if runtime:
            case = contract["cases"][case_id]
            if runtime[0].get("action") != case["expected_runtime_action"] \
                    or runtime[0].get("reason") != case["expected_runtime_reason"]:
                failures.append(f"{prefix}: absent or wrong frozen runtime action/reason")
        if _events(run, "RUNTIME_SAFE"):
            failures.append(f"{prefix}: fabricated runtime safe evidence")


def analyze_bundle(contract, bundle, contract_path, repository_root=None):
    failures = []
    repository_root = Path(repository_root or Path(contract_path).resolve().parents[2])
    if not isinstance(bundle, dict):
        bundle = {}
        failures.append("bundle must be an object")
    _finite_tree(bundle, "bundle", failures)
    if bundle.get("schema_version") != EVIDENCE_SCHEMA:
        failures.append("wrong evidence schema_version")
    if bundle.get("validation_only") is not True:
        failures.append("synthetic evidence must be validation_only")
    manifest = bundle.get("manifest")
    if not isinstance(manifest, dict):
        manifest = {}
        failures.append("missing manifest")
    expected_manifest = {
        "route_id": contract["route_id"],
        "contract_sha256": _sha256(contract_path),
        "analyzer_version": contract["analyzer_version"],
        "fixture_identities": {
            case_id: contract["cases"][case_id]["fixture_alias"]
            for case_id in CASE_IDS
        },
    }
    for key, expected in expected_manifest.items():
        if manifest.get(key) != expected:
            failures.append(f"manifest {key} mismatch")
    if not isinstance(manifest.get("git_commit"), str) \
            or re.fullmatch(r"[0-9a-f]{40}", manifest.get("git_commit", "")) is None:
        failures.append("manifest git_commit malformed")
    runs = bundle.get("runs")
    if not isinstance(runs, list):
        runs = []
        failures.append("runs must be a list")
    case_ids = [run.get("case_id") for run in runs if isinstance(run, dict)]
    run_ids = [run.get("run_id") for run in runs if isinstance(run, dict)]
    if sorted(case_ids) != sorted(CASE_IDS):
        failures.append("missing or duplicate case identities")
    if len(run_ids) != len(set(run_ids)) or any(not identity for identity in run_ids):
        failures.append("missing or duplicate run identities")
    expected_run_identities = {
        run.get("case_id"): run.get("run_id")
        for run in runs if isinstance(run, dict)
    }
    if manifest.get("run_identities") != expected_run_identities \
            or set(expected_run_identities) != set(CASE_IDS):
        failures.append("manifest run identity binding mismatch")
    hashes = manifest.get("raw_artifact_hashes")
    if not isinstance(hashes, dict) or set(hashes) != set(run_ids) \
            or any(not isinstance(value, dict) or not value for value in hashes.values()):
        failures.append("raw artifact hashes missing, malformed, or identity-mismatched")
    else:
        run_by_id = {
            run.get("run_id"): run for run in runs
            if isinstance(run, dict) and run.get("run_id")
        }
        for run_id, artifacts in hashes.items():
            raw_run_bound = False
            for relative, expected_hash in artifacts.items():
                path = repository_root / relative
                if not isinstance(relative, str) or Path(relative).is_absolute() \
                        or not _within(path, repository_root):
                    failures.append(f"run[{run_id}]: raw artifact path is not repository-local")
                elif not path.is_file():
                    failures.append(f"run[{run_id}]: raw artifact missing")
                elif not isinstance(expected_hash, str) or SHA256_RE.fullmatch(expected_hash) is None \
                        or _sha256(path) != expected_hash:
                    failures.append(f"run[{run_id}]: raw artifact hash mismatch")
                else:
                    try:
                        raw_payload = json.loads(path.read_text())
                    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                        raw_payload = None
                    if raw_payload == run_by_id.get(run_id):
                        raw_run_bound = True
            if not raw_run_bound:
                failures.append(f"run[{run_id}]: raw artifacts do not bind run evidence")
    normalized_hashes = manifest.get("normalized_evidence_sha256")
    if not isinstance(normalized_hashes, dict) or set(normalized_hashes) != set(run_ids):
        failures.append("normalized evidence hashes missing or identity-mismatched")
    elif len(run_ids) == len(set(run_ids)):
        for run in runs:
            if not isinstance(run, dict) or not run.get("run_id"):
                continue
            try:
                actual_hash = evidence_sha256(run)
            except (TypeError, ValueError):
                failures.append(f"run[{run.get('case_id')}]: raw evidence is not canonical JSON")
                continue
            if normalized_hashes[run["run_id"]] != actual_hash:
                failures.append(f"run[{run.get('case_id')}]: normalized evidence hash mismatch")
    case_results = {}
    for run in runs:
        if not isinstance(run, dict):
            failures.append("malformed run row")
            continue
        before = len(failures)
        case_id = run.get("case_id")
        if case_id in contract["cases"] and run.get("run_id"):
            expected_binding = build_launch_binding(
                contract, contract_path, case_id,
                manifest.get("git_commit"), run["run_id"],
                _case_values(contract, case_id),
            )
            if run.get("launch_binding") != expected_binding:
                failures.append(f"run[{case_id}]: launch binding mismatch")
        _validate_common_run(contract, run, failures)
        _validate_case(contract, run, failures)
        case_results[run.get("case_id", "<missing>")] = {
            "status": "PASS" if len(failures) == before else "FAIL"
        }
    return {
        "schema_version": RESULT_SCHEMA,
        "validation_only": True,
        "status": "VALIDATION_ONLY_PASS" if not failures else "VALIDATION_ONLY_FAIL",
        "qualification_claim": False,
        "route_id": contract["route_id"],
        "analyzer_version": contract["analyzer_version"],
        "contract_sha256": _sha256(contract_path),
        "validated_manifest": manifest,
        "launch_binding_sha256": {
            run.get("case_id", "<missing>"): _evidence_sha256_or_invalid(
                run.get("launch_binding", {})
            )
            for run in runs if isinstance(run, dict)
        },
        "case_results": case_results,
        "failures": failures,
    }


def build_synthetic_validation_bundle(contract, contract_path, git_commit):
    """Build deterministic typed inputs that can validate the analyzer, never flight."""
    common = {
        "processes": [
            {"identity": name, "alive_until_controlled_shutdown": True}
            for name in contract["required_processes"]
        ],
        "controlled_shutdown": True,
        "topic_counts": {name: 3 for name in contract["required_topics"]},
        "p0_samples": [
            {"sequence": 1, "ready": True, "stable": True, "refresh_s": 0.20,
             "gnss_epoch_seen": True, "gnss_epoch_valid": True,
             "gnss_epoch_fresh": True, "predictor_gnss_used_count": 4,
             "predictor_lidar_used_count": 4,
             "predictor_horizon_fusion_count": 6},
            {"sequence": 2, "ready": True, "stable": True, "refresh_s": 0.21,
             "gnss_epoch_seen": True, "gnss_epoch_valid": True,
             "gnss_epoch_fresh": True, "predictor_gnss_used_count": 5,
             "predictor_lidar_used_count": 5,
             "predictor_horizon_fusion_count": 6},
        ],
        "integrity_samples": [
            {"sequence": 1, "valid": True, "n_sv_used": 8},
            {"sequence": 2, "valid": True, "n_sv_used": 9},
        ],
    }
    runs = [
        {
            **json.loads(json.dumps(common)),
            "case_id": "SAFE_NORMAL", "run_id": "synthetic-safe-normal-v1",
            "fixture_alias": "none_v1",
            "events": [
                {"sequence": 1, "type": "FINAL_ACCEPT", "candidate_id": "safe-v1"},
                {"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "safe-v1"},
            ],
        },
        {
            **json.loads(json.dumps(common)),
            "case_id": "FINAL_REJECT", "run_id": "synthetic-final-reject-v1",
            "fixture_alias": "p5_7_rejected_trajectory_zone_v1",
            "events": [{
                "sequence": 1, "type": "FINAL_REJECT",
                "candidate_id": "reject-v1", "reason": "p5_7_rejected_trajectory",
            }],
        },
        {
            **json.loads(json.dumps(common)),
            "case_id": "RUNTIME_FAIL", "run_id": "synthetic-runtime-fail-v1",
            "fixture_alias": "p5_6_future_unknown_zone_v1",
            "events": [
                {"sequence": 1, "type": "FINAL_ACCEPT", "candidate_id": "runtime-v1"},
                {"sequence": 2, "type": "NORMAL_PUBLISH", "candidate_id": "runtime-v1"},
                {
                    "sequence": 3, "type": "RUNTIME_ACTION",
                    "candidate_id": "runtime-v1", "action": "EMERGENCY_STOP",
                    "reason": "future_unknown_timeout",
                },
            ],
        },
    ]
    for run in runs:
        run["launch_binding"] = build_launch_binding(
            contract, contract_path, run["case_id"], git_commit,
            run["run_id"], _case_values(contract, run["case_id"]),
        )
    return {
        "schema_version": EVIDENCE_SCHEMA,
        "validation_only": True,
        "manifest": {
            "route_id": contract["route_id"],
            "git_commit": git_commit,
            "contract_sha256": _sha256(contract_path),
            "analyzer_version": contract["analyzer_version"],
            "fixture_identities": {
                case_id: contract["cases"][case_id]["fixture_alias"]
                for case_id in CASE_IDS
            },
            "run_identities": {
                run["case_id"]: run["run_id"] for run in runs
            },
            "raw_artifact_hashes": {
                run["run_id"]: {} for run in runs
            },
            "normalized_evidence_sha256": {
                run["run_id"]: evidence_sha256(run) for run in runs
            },
        },
        "runs": runs,
    }


def bind_run_artifacts(bundle, repository_root, raw_directory):
    """Write one canonical raw evidence file per run and bind its real hash."""
    repository_root = Path(repository_root).resolve()
    raw_directory = Path(raw_directory).resolve()
    if not _within(raw_directory, repository_root):
        raise ContractError("raw artifact directory must be repository-local")
    raw_directory.mkdir(parents=True, exist_ok=True)
    hashes = {}
    normalized = {}
    for run in bundle["runs"]:
        run_id = run["run_id"]
        path = raw_directory / f"{run_id}.json"
        path.write_text(json.dumps(
            run, sort_keys=True, separators=(",", ":"), allow_nan=False
        ) + "\n")
        relative = str(path.relative_to(repository_root))
        hashes[run_id] = {relative: _sha256(path)}
        normalized[run_id] = evidence_sha256(run)
    bundle["manifest"]["raw_artifact_hashes"] = hashes
    bundle["manifest"]["normalized_evidence_sha256"] = normalized
    return bundle


def write_live_bundle(
    contract, contract_path, runs, git_commit, install_manifest_path, output_path
):
    """Bind completed real runs and their raw sources without analyzing them."""
    output_path = Path(output_path).resolve()
    repository_root = SOURCE_REPOSITORY
    if not _within(output_path, repository_root):
        raise ContractError("live output must be repository-local")
    install_manifest_path = Path(install_manifest_path).resolve()
    if not _within(install_manifest_path, repository_root):
        raise ContractError("install manifest must be repository-local")
    if {run.get("case_id"): run.get("run_id") for run in runs} != LIVE_RUN_IDENTITIES:
        raise ContractError("live run identities/order mismatch")
    raw_directory = output_path.parent / "normalized_raw"
    raw_directory.mkdir(parents=True, exist_ok=False)
    raw_hashes = {}
    normalized_hashes = {}
    for run in runs:
        if run.get("validation_only") is not False:
            raise ContractError("live run cannot be validation-only")
        run_id = run["run_id"]
        raw_path = raw_directory / f"{run_id}.json"
        raw_path.write_text(json.dumps(
            run, sort_keys=True, separators=(",", ":"), allow_nan=False
        ) + "\n")
        artifacts = {
            str(raw_path.relative_to(repository_root)): _sha256(raw_path),
        }
        sources = run.get("raw_sources")
        if not isinstance(sources, list) or len(sources) != len(set(sources)):
            raise ContractError("live raw source inventory is malformed")
        for relative in sources:
            path = repository_root / relative
            if not path.is_file() or path.is_symlink():
                raise ContractError(f"live raw source unavailable: {relative}")
            artifacts[str(relative)] = _sha256(path)
        raw_hashes[run_id] = artifacts
        normalized_hashes[run_id] = evidence_sha256(run)
    runner_state_path = output_path.parent / "runner_state.json"
    bundle = {
        "schema_version": LIVE_EVIDENCE_SCHEMA,
        "validation_only": False,
        "manifest": {
            "route_id": contract["route_id"],
            "git_commit": git_commit,
            "contract_sha256": _sha256(contract_path),
            "analyzer_version": contract["analyzer_version"],
            "fixture_identities": {
                case_id: contract["cases"][case_id]["fixture_alias"]
                for case_id in CASE_IDS
            },
            "run_identities": dict(LIVE_RUN_IDENTITIES),
            "raw_artifact_hashes": raw_hashes,
            "normalized_evidence_sha256": normalized_hashes,
            "install_manifest_path": str(
                install_manifest_path.relative_to(repository_root)
            ),
            "install_manifest_sha256": _sha256(install_manifest_path),
            "runner_state_path": str(runner_state_path.relative_to(repository_root)),
            "runner_state_sha256": _sha256(runner_state_path),
        },
        "runs": runs,
    }
    output_path.write_text(json.dumps(
        bundle, indent=2, sort_keys=True, allow_nan=False
    ) + "\n")
    return bundle


def live_process_lifecycle_exact(process, required_processes) -> bool:
    process_rows = process.get("required_processes", {}) \
        if isinstance(process, dict) else {}
    failures = process.get("process_failures", []) \
        if isinstance(process, dict) else None
    controlled_failures_only = isinstance(failures, list) and all(
        isinstance(item, dict)
        and item.get("phase") == "controlled_shutdown"
        and item.get("reason")
        == "required_process_stopped_during_controlled_shutdown"
        and item.get("process_name") in required_processes
        for item in failures
    )
    return bool(
        isinstance(process, dict)
        and process.get("required_processes_ok") is True
        and process.get("controlled_shutdown") is True
        and process.get("orphan_check_passed") is True
        and process.get("forced_orphan_cleanup") is False
        and process.get("remaining_process_group_pids") == []
        and set(process_rows) == set(required_processes)
        and all(
            isinstance(value, dict) and value.get("seen") is True
            and value.get("runtime_failure") is False
            for value in process_rows.values()
        )
        and controlled_failures_only
    )


def live_linkage_sha(path, environment) -> str | None:
    completed = subprocess.run(
        ["ldd", str(path)], capture_output=True, text=True,
        check=False, env=environment,
    )
    output = completed.stdout + completed.stderr
    if completed.returncode != 0 or "not found" in output \
            or any(forbidden in output for forbidden in (
                "/home/dev/ws_iap/build", "/home/dev/ws_iap/install",
                "/home/dev/ws_iap/src/iap/build",
                "/home/dev/ws_iap/src/iap/install",
            )):
        return None
    canonical_lines = sorted(
        re.sub(r"\s+\(0x[0-9a-fA-F]+\)$", "", line.strip())
        for line in output.splitlines() if line.strip()
    )
    canonical = "\n".join(canonical_lines) + "\n"
    return hashlib.sha256(canonical.encode()).hexdigest()


def live_install_manifest_exact(manifest, repository_root) -> bool:
    repository_root = Path(repository_root).resolve()
    install_root = repository_root / "results/icra27/icra068/install"
    expected_prefixes = [
        str(install_root), "/root/ros2_ws/install", "/opt/ros/jazzy",
    ]
    expected_packages = {
        package: str(install_root) for package in ICRA068_LOCAL_PACKAGES
    } | {"rclcpp_components": "/opt/ros/jazzy"}
    runtime_libraries = tuple(sorted(
        str(path.relative_to(install_root))
        for path in (install_root / "lib").glob("*.so")
        if path.is_file() and not path.is_symlink()
    ))
    expected_file_keys = set(ICRA068_EXECUTABLE_PATHS) \
        | set(ICRA068_CONFIG_PATHS) | set(runtime_libraries) \
        | {"/opt/ros/jazzy/lib/rclcpp_components/component_container"}
    file_hashes = manifest.get("file_hashes") if isinstance(manifest, dict) else None
    linkage = manifest.get("linkage_output_sha256") \
        if isinstance(manifest, dict) else None
    aliases = manifest.get("installed_aliases") \
        if isinstance(manifest, dict) else None
    build_profile = {
        "package_count": len(ICRA068_LOCAL_PACKAGES),
        "cmake_build_type": "Release", "build_testing": False,
        "build_with_cuda": True, "merge_install": True,
        "symlink_install": False,
    }
    if not (
        isinstance(manifest, dict)
        and set(manifest) == ICRA068_MANIFEST_KEYS
        and manifest.get("schema_version")
        == "icra068_qualification_install_manifest_v1"
        and manifest.get("closure_ready") is True
        and manifest.get("install_root") == str(install_root)
        and manifest.get("active_prefixes") == expected_prefixes
        and manifest.get("packages") == expected_packages
        and manifest.get("build_profile") == build_profile
        and manifest.get("runtime_libraries") == list(runtime_libraries)
        and set(runtime_libraries).issuperset(ICRA068_LIBRARY_ROOTS)
        and isinstance(file_hashes, dict)
        and set(file_hashes) == expected_file_keys
        and all(SHA256_RE.fullmatch(value or "") for value in file_hashes.values())
        and isinstance(linkage, dict)
        and set(linkage) == set(runtime_libraries)
        and all(SHA256_RE.fullmatch(value or "") for value in linkage.values())
        and isinstance(aliases, dict)
        and set(aliases) == set(ICRA068_ALIAS_PATHS)
    ):
        return False
    for relative, expected_hash in file_hashes.items():
        path = Path(relative) if Path(relative).is_absolute() else install_root / relative
        if not path.is_file() or path.is_symlink() or _sha256(path) != expected_hash:
            return False
    for relative, source_relative in ICRA068_ALIAS_PATHS.items():
        row = aliases.get(relative)
        installed = install_root / relative
        if not (
            isinstance(row, dict)
            and row.get("source_path") == source_relative
            and SHA256_RE.fullmatch(row.get("source_sha256", ""))
            and row.get("source_sha256") == row.get("installed_sha256")
            and installed.is_file() and not installed.is_symlink()
            and _sha256(installed) == row.get("installed_sha256")
        ):
            return False
    linkage_environment = dict(os.environ)
    linkage_environment["LD_LIBRARY_PATH"] = ":".join((
        str(install_root / "lib"), "/root/ros2_ws/install/glim_ros/lib",
        "/root/ros2_ws/install/glim/lib", "/opt/ros/jazzy/lib",
        "/opt/ros/jazzy/lib/x86_64-linux-gnu",
    ))
    for relative in runtime_libraries:
        if live_linkage_sha(
            install_root / relative, linkage_environment
        ) != linkage[relative]:
            return False
    return True


def analyze_live_bundle(
    contract, bundle, contract_path, repository_root=None
):
    """Authoritatively analyze real evidence; synthetic evidence is rejected."""
    repository_root = Path(repository_root or SOURCE_REPOSITORY).resolve()
    technical = []
    if not isinstance(bundle, dict):
        bundle = {}
        technical.append("live bundle must be an object")
    if bundle.get("schema_version") != LIVE_EVIDENCE_SCHEMA:
        technical.append("wrong live evidence schema_version")
    if bundle.get("validation_only") is not False:
        technical.append("live qualification rejects validation_only evidence")
    runs = bundle.get("runs") if isinstance(bundle.get("runs"), list) else []
    if any(not isinstance(run, dict) or run.get("validation_only") is not False for run in runs):
        technical.append("live run contains validation-only or malformed evidence")
    manifest = bundle.get("manifest") if isinstance(bundle.get("manifest"), dict) else {}
    if manifest.get("run_identities") != LIVE_RUN_IDENTITIES:
        technical.append("live fixed identity binding mismatch")
    raw_hashes = manifest.get("raw_artifact_hashes") \
        if isinstance(manifest.get("raw_artifact_hashes"), dict) else {}
    for run in runs:
        if not isinstance(run, dict):
            continue
        run_id = run.get("run_id")
        sources = run.get("raw_sources")
        artifacts = raw_hashes.get(run_id, {}) if isinstance(raw_hashes, dict) else {}
        if not isinstance(sources, list) or len(sources) != len(set(sources)) \
                or not set(sources).issubset(set(artifacts)):
            technical.append(f"run[{run_id}]: raw source inventory mismatch")
            continue
        names = {Path(relative).name for relative in sources}
        if not {
            "process_result.json", "capture_ready.json", "launch_command.json",
            "stdout.log", "metadata.yaml",
        }.issubset(names) or not any(
            Path(relative).suffix in {".db3", ".mcap"} for relative in sources
        ):
            technical.append(f"run[{run_id}]: complete live raw sources missing")
        process_paths = [
            repository_root / relative for relative in sources
            if Path(relative).name == "process_result.json"
        ]
        if len(process_paths) != 1:
            technical.append(f"run[{run_id}]: process result identity mismatch")
            continue
        try:
            process = json.loads(process_paths[0].read_text())
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            process = {}
        if not live_process_lifecycle_exact(
            process, contract["required_processes"]
        ):
            technical.append(f"run[{run_id}]: process lifecycle evidence mismatch")
    for key in ("install_manifest_path", "runner_state_path"):
        relative = manifest.get(key)
        path = repository_root / relative if isinstance(relative, str) else repository_root
        expected = manifest.get(key.replace("_path", "_sha256"))
        if not isinstance(relative, str) or Path(relative).is_absolute() \
                or not _within(path, repository_root) or not path.is_file() \
                or not isinstance(expected, str) or _sha256(path) != expected:
            technical.append(f"{key} binding mismatch")
    install_path = repository_root / str(manifest.get("install_manifest_path", ""))
    if install_path.is_file():
        try:
            install_manifest = json.loads(install_path.read_text())
        except (OSError, json.JSONDecodeError):
            install_manifest = {}
        if install_manifest.get("git_commit") != manifest.get("git_commit"):
            technical.append("install manifest commit mismatch")
        if not live_install_manifest_exact(install_manifest, repository_root):
            technical.append("install closure inventory is not exact")
    runner_path = repository_root / str(manifest.get("runner_state_path", ""))
    if runner_path.is_file():
        try:
            runner_state = json.loads(runner_path.read_text())
        except (OSError, json.JSONDecodeError):
            runner_state = {}
        expected_ids = list(LIVE_RUN_IDENTITIES.values())
        if not (
            runner_state.get("state") == "COMPLETE"
            and runner_state.get("registered") == expected_ids
            and runner_state.get("attempted") == expected_ids
            and runner_state.get("completed") == expected_ids
            and runner_state.get("retries") == 0
            and runner_state.get("gpu_preflight_invocations") == 1
            and runner_state.get("launch_invocations") == 3
            and runner_state.get("install_manifest_sha256")
            == manifest.get("install_manifest_sha256")
        ):
            technical.append("runner state is not exact complete one-shot evidence")
    validation_input = json.loads(json.dumps(bundle))
    validation_input["schema_version"] = EVIDENCE_SCHEMA
    validation_input["validation_only"] = True
    base = analyze_bundle(
        contract, validation_input, contract_path, repository_root
    )
    failures = list(base["failures"])
    technical_markers = (
        "manifest", "identity", "raw artifact", "hash", "process", "topic",
        "P0", "launch binding", "malformed", "missing", "uncontrolled",
    )
    behavioral = [
        failure for failure in failures
        if not any(marker.lower() in failure.lower() for marker in technical_markers)
    ]
    technical.extend(failure for failure in failures if failure not in behavioral)
    if technical:
        status = "P5_PROSPECTIVE_QUALIFICATION_TECHNICAL_BLOCKER"
    elif behavioral:
        status = "P5_PROSPECTIVE_QUALIFICATION_FAIL"
    else:
        status = "P5_PROSPECTIVE_QUALIFICATION_PASS"
    return {
        "schema_version": LIVE_RESULT_SCHEMA,
        "validation_only": False,
        "status": status,
        "qualification_claim": status == "P5_PROSPECTIVE_QUALIFICATION_PASS",
        "route_id": contract["route_id"],
        "contract_sha256": _sha256(contract_path),
        "case_results": base["case_results"],
        "technical_failures": technical,
        "behavioral_failures": behavioral,
        "validated_manifest": manifest,
    }


def _within(path, root):
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except ValueError:
        return False


def _claim_live_analyzer_once(output_path, input_path, contract_path):
    output_path = Path(output_path)
    marker = output_path.with_name(output_path.name + ".invocation.json")
    if output_path.exists() or output_path.is_symlink() \
            or marker.exists() or marker.is_symlink():
        raise ContractError("live analyzer invocation/output already exists")
    marker.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps({
        "schema_version": "icra_p0_p5_live_analyzer_invocation_v1",
        "claimed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "input_path": str(Path(input_path).resolve()),
        "input_sha256": _sha256(input_path),
        "contract_path": str(Path(contract_path).resolve()),
        "contract_sha256": _sha256(contract_path),
    }, indent=2, sort_keys=True) + "\n"
    descriptor = os.open(
        marker, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644
    )
    try:
        os.write(descriptor, payload.encode("utf-8"))
    finally:
        os.close(descriptor)
    return marker


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    analyze = subparsers.add_parser("analyze")
    analyze.add_argument("--contract", required=True)
    analyze.add_argument("--input", required=True)
    analyze.add_argument("--output", required=True)
    analyze.add_argument("--repository-root", required=True)
    analyze_live = subparsers.add_parser("analyze-live")
    analyze_live.add_argument("--contract", required=True)
    analyze_live.add_argument("--input", required=True)
    analyze_live.add_argument("--output", required=True)
    analyze_live.add_argument("--repository-root", required=True)
    emit = subparsers.add_parser("emit-synthetic-input")
    emit.add_argument("--contract", required=True)
    emit.add_argument("--git-commit", required=True)
    emit.add_argument("--output", required=True)
    emit.add_argument("--repository-root", required=True)
    args = parser.parse_args(argv)
    requested_repository = Path(args.repository_root).resolve()
    if requested_repository != SOURCE_REPOSITORY or not (SOURCE_REPOSITORY / ".git").exists():
        raise ContractError(
            f"repository-root must be the actual checkout {SOURCE_REPOSITORY}"
        )
    paths = [("contract", args.contract), ("output", args.output)]
    if args.command in {"analyze", "analyze-live"}:
        paths.append(("input", args.input))
    for label, path in paths:
        if not _within(path, args.repository_root):
            raise ContractError(f"{label} must be repository-local")
    contract = load_contract(args.contract)
    if args.command == "emit-synthetic-input":
        if re.fullmatch(r"[0-9a-f]{40}", args.git_commit) is None:
            raise ContractError("git commit must be a full lowercase SHA-1")
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        bundle = build_synthetic_validation_bundle(
            contract, args.contract, args.git_commit
        )
        bind_run_artifacts(bundle, SOURCE_REPOSITORY, output.parent / "raw")
        output.write_text(json.dumps(
            bundle,
            indent=2, sort_keys=True, allow_nan=False,
        ) + "\n")
        return 0
    if args.command == "analyze-live":
        _claim_live_analyzer_once(args.output, args.input, args.contract)
    try:
        bundle = json.loads(Path(args.input).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise ContractError(f"cannot load synthetic evidence: {exc}") from exc
    result = (
        analyze_live_bundle(contract, bundle, args.contract, SOURCE_REPOSITORY)
        if args.command == "analyze-live"
        else analyze_bundle(contract, bundle, args.contract, SOURCE_REPOSITORY)
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return 0 if result["status"] in {
        "VALIDATION_ONLY_PASS", "P5_PROSPECTIVE_QUALIFICATION_PASS"
    } else 2


if __name__ == "__main__":
    raise SystemExit(main())
