#!/usr/bin/env python3
"""Fail-closed ICRA P0+P5 profile and synthetic qualification analyzer."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path


CONTRACT_SCHEMA = "icra_p0_p5_qualification_contract_v1"
EVIDENCE_SCHEMA = "icra_p0_p5_synthetic_evidence_v1"
RESULT_SCHEMA = "icra_p0_p5_validation_result_v1"
CASE_IDS = ("SAFE_NORMAL", "FINAL_REJECT", "RUNTIME_FAIL")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
SOURCE_REPOSITORY = Path(__file__).resolve().parents[1]


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
        "profile_values", "p5_thresholds", "fixture_switches",
        "required_processes", "required_topics",
    ):
        if key not in contract:
            raise ContractError(f"contract missing {key}")
    if contract["profile_name"] != "icra_p0_p5":
        raise ContractError("contract profile_name is not icra_p0_p5")
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
        if any(type(value) is not int for value in sequences) or sequences != sorted(set(sequences)):
            failures.append(f"{prefix}: malformed or duplicate P0 sequence")
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
                    except (OSError, json.JSONDecodeError):
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
            {"sequence": 1, "ready": True, "stable": True, "refresh_s": 0.20},
            {"sequence": 2, "ready": True, "stable": True, "refresh_s": 0.21},
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


def _within(path, root):
    try:
        Path(path).resolve().relative_to(Path(root).resolve())
        return True
    except ValueError:
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    analyze = subparsers.add_parser("analyze")
    analyze.add_argument("--contract", required=True)
    analyze.add_argument("--input", required=True)
    analyze.add_argument("--output", required=True)
    analyze.add_argument("--repository-root", required=True)
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
    if args.command == "analyze":
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
    try:
        bundle = json.loads(Path(args.input).read_text(), parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        raise ContractError(f"cannot load synthetic evidence: {exc}") from exc
    result = analyze_bundle(contract, bundle, args.contract, SOURCE_REPOSITORY)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    return 0 if result["status"] == "VALIDATION_ONLY_PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
