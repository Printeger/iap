#!/usr/bin/env python3
"""Outcome-blind ICRA-076 preregistration and byte-freeze contracts."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
from decimal import Decimal, localcontext
from math import comb
from pathlib import Path
from typing import Any


REPOSITORY = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPOSITORY / "config/icra27/icra076_preregistration_schema_v1.json"
REPLAY_PATH = REPOSITORY / "config/icra27/icra076_repeatability_replay_v1.json"
ARMS = ("P0_P5_CONTROL", "P0_P4_V2_P5_TREATMENT")
SCENES = ("PRIMARY", "EXACT_MIRROR", "FLAT_NULL")
FORBIDDEN_INPUT_TOKENS = ("held_out", "held-out", "icra077", "formal_results")


class Icra076Error(ValueError):
    """Typed fail-closed preregistration error."""

    def __init__(self, code: str, detail: str = "") -> None:
        self.code = code
        self.detail = detail
        super().__init__(f"{code}: {detail}" if detail else code)


def _fail(code: str, detail: str = "") -> None:
    raise Icra076Error(code, detail)


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        _fail("JSON_INPUT_INVALID", f"{path}: {exc}")
    if not isinstance(value, dict):
        _fail("JSON_OBJECT_REQUIRED", str(path))
    return value


def _reject_forbidden_path_tokens(path: Path) -> None:
    rendered = str(path).lower()
    if any(token in rendered for token in FORBIDDEN_INPUT_TOKENS):
        _fail("HELD_OUT_PATH_FORBIDDEN", str(path))


def _admit_input_path(path: Path) -> Path:
    """Reject forbidden aliases and every symlink component before file read."""
    _reject_forbidden_path_tokens(path)
    absolute = path.absolute()
    resolved = absolute.resolve(strict=False)
    _reject_forbidden_path_tokens(resolved)
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            _fail("INPUT_SYMLINK_FORBIDDEN", str(current))
    return absolute


def _repository_input_path(path: Path, repository: Path) -> Path:
    """Admit a repository-local regular input before opening it."""
    repository = repository.resolve()
    candidate = path if path.is_absolute() else repository / path
    absolute = _admit_input_path(candidate)
    try:
        absolute.relative_to(repository)
    except ValueError:
        _fail("EXTERNAL_INPUT_PATH_FORBIDDEN", str(path))
    try:
        absolute.resolve().relative_to(repository)
    except ValueError:
        _fail("EXTERNAL_INPUT_PATH_FORBIDDEN", str(path))
    return absolute


def load_verification(path: Path, repository: Path = REPOSITORY) -> dict[str, Any]:
    """Load a repository-local verification manifest fail-closed."""
    return _json(_repository_input_path(path, repository.resolve()))


def _finite_number(value: Any, code: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _fail(code, "not numeric")
    converted = float(value)
    if not math.isfinite(converted):
        _fail(code, "not finite")
    return converted


def _repository_path(repository: Path, relative: str) -> Path:
    lowered = relative.lower()
    if any(token in lowered for token in FORBIDDEN_INPUT_TOKENS):
        _fail("HELD_OUT_PATH_FORBIDDEN", relative)
    candidate = Path(relative)
    if candidate.is_absolute() or ".." in candidate.parts:
        _fail("EXTERNAL_INPUT_PATH_FORBIDDEN", relative)
    return _repository_input_path(candidate, repository)


def validate_output_path(output: Path, repository: Path,
                         allowed_root: Path) -> Path:
    repository = repository.resolve()
    allowed_root_absolute = Path(allowed_root).absolute()
    output_absolute = Path(output).absolute()
    try:
        allowed_root_absolute.relative_to(repository)
        output_absolute.relative_to(allowed_root_absolute)
    except ValueError:
        _fail("OUTPUT_OUTSIDE_ALLOWED_ROOT", str(output))
    current = output_absolute.parent
    while True:
        if current.is_symlink():
            _fail("OUTPUT_SYMLINK_COMPONENT", str(current))
        if current == allowed_root_absolute:
            break
        if current == current.parent:
            _fail("OUTPUT_OUTSIDE_ALLOWED_ROOT", str(output))
        current = current.parent
    if output_absolute.exists() or output_absolute.is_symlink():
        _fail("OUTPUT_ALREADY_EXISTS", str(output))
    return output_absolute


def inventory_path(path: Path, base: Path) -> dict[str, Any]:
    path_absolute = path.absolute()
    base_absolute = base.absolute()
    try:
        relative = str(path_absolute.relative_to(base_absolute))
    except ValueError:
        _fail("INVENTORY_PATH_OUTSIDE_ROOT", str(path))
    if path_absolute.is_symlink():
        target = path_absolute.resolve()
        if not target.is_file():
            _fail("INVENTORY_TYPE_INVALID", relative)
        return {
            "path": relative,
            "type": "symlink_to_file",
            "size_bytes": target.stat().st_size,
            "sha256": file_sha256(target),
        }
    if not path_absolute.is_file():
        _fail("INVENTORY_TYPE_INVALID", relative)
    return {
        "path": relative,
        "type": "regular_file",
        "size_bytes": path_absolute.stat().st_size,
        "sha256": file_sha256(path_absolute),
    }


def verify_inventory(records: list[dict[str, Any]], base: Path,
                     category: str) -> None:
    expected_code = f"{category}_BYTE_DRIFT"
    if category not in {"SOURCE", "INSTALL"}:
        _fail("INVENTORY_CATEGORY_INVALID", category)
    if not isinstance(records, list) or not records:
        _fail(f"{category}_INVENTORY_EMPTY")
    observed_paths: set[str] = set()
    for expected in records:
        relative = expected.get("path")
        if not isinstance(relative, str) or relative in observed_paths:
            _fail(f"{category}_INVENTORY_IDENTITY_INVALID")
        observed_paths.add(relative)
        try:
            actual = inventory_path(base / relative, base)
        except Icra076Error:
            _fail(expected_code, relative)
        if actual != expected:
            _fail(expected_code, relative)


def expected_verification_argv() -> dict[str, list[str]]:
    return {
        "FOCUSED_TESTS": [
            "bash", "-lc",
            ("python3 test/test_icra073_inverse_corridor.py -v && "
             "python3 test/test_icra074_geometry.py -v && "
             "python3 test/test_icra075_exploratory.py -v && "
             "python3 test/test_icra076_preregistration.py -v")],
        "VALIDATOR": [
            "python3", "scripts/dev_planner/validate_icra076_preregistration.py"],
        "SIX_PACKAGE_BUILD": [
            "colcon", "--log-base", "/home/dev/ws_iap/log", "build",
            "--paths", "/home/dev/ws_iap/src/iap",
            "/home/dev/ws_iap/src/iap/src/iap/planner/plan_env",
            "/home/dev/ws_iap/src/iap/src/iap/planner/traj_utils",
            "/home/dev/ws_iap/src/iap/src/iap/planner/path_searching",
            "/home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt",
            "/home/dev/ws_iap/src/iap/src/iap/planner/plan_manage",
            "--packages-select", "iap", "plan_env", "traj_utils",
            "path_searching", "bspline_opt", "ego_planner",
            "--build-base", "/home/dev/ws_iap/build",
            "--install-base", "/home/dev/ws_iap/install",
            "--symlink-install", "--cmake-args", "-DBUILD_TESTING=ON"],
        "REPEATABILITY_REPLAY": [
            "python3", "scripts/dev_planner/icra076_repeatability_replay.py",
            "--snapshot", "config/icra27/icra076_flat_null_snapshot_v1.json",
            "--output-root",
            "results/icra27/icra076/repeatability-replay-003"],
    }


def validate_verification(verification: dict[str, Any],
                          expected_source_head: str | None = None) -> None:
    if set(verification) != {"schema_version", "source_head", "commands"} or \
            verification.get("schema_version") != \
            "icra076_repository_local_verification_v1":
        _fail("REQUIRED_VERIFICATION_NOT_PASS", "schema")
    source_head = verification.get("source_head")
    if (not isinstance(source_head, str) or len(source_head) != 40 or
            any(c not in "0123456789abcdef" for c in source_head)):
        _fail("REQUIRED_VERIFICATION_SOURCE_INVALID")
    if expected_source_head is not None and source_head != expected_source_head:
        _fail("REQUIRED_VERIFICATION_SOURCE_DRIFT")
    commands = verification.get("commands")
    if not isinstance(commands, list):
        _fail("REQUIRED_VERIFICATION_NOT_PASS", "commands missing")
    expected = expected_verification_argv()
    categories = set(expected)
    observed: set[str] = set()
    for command in commands:
        if not isinstance(command, dict):
            _fail("REQUIRED_VERIFICATION_NOT_PASS", "command not object")
        category = command.get("category")
        if category in observed or category not in categories:
            _fail("REQUIRED_VERIFICATION_NOT_PASS", str(category))
        observed.add(category)
        argv = command.get("argv")
        if (not isinstance(argv, list) or not argv or
                not all(isinstance(token, str) and token for token in argv) or
                command.get("enabled") is not True or
                command.get("skipped") is not False or
                command.get("exit_code") != 0):
            _fail("REQUIRED_VERIFICATION_NOT_PASS", str(category))
        if argv != expected[category]:
            _fail("REQUIRED_VERIFICATION_COMMAND_DRIFT", str(category))
    if observed != categories:
        _fail("REQUIRED_VERIFICATION_NOT_PASS", "category missing")


def _verify_bound_file(repository: Path, record: dict[str, Any]) -> None:
    if set(record) != {"path", "sha256"}:
        _fail("BOUND_FILE_IDENTITY_INVALID", repr(record))
    path = _repository_path(repository, record["path"])
    if not path.is_file() or path.is_symlink():
        _fail("BOUND_FILE_TYPE_INVALID", record["path"])
    observed = file_sha256(path)
    if observed != record["sha256"]:
        _fail("BOUND_FILE_DRIFT", record["path"])


def _binomial_upper_tail(n: int, successes: int, probability: Decimal) -> Decimal:
    return sum(
        Decimal(comb(n, count)) * probability ** count *
        (Decimal(1) - probability) ** (n - count)
        for count in range(successes, n + 1))


def exact_binomial_passing_rule(n: int, probability: float,
                                alpha: float) -> dict:
    """Return the smallest exact one-sided rejection count."""
    if n <= 0:
        raise ValueError("n must be positive")
    with localcontext() as context:
        context.prec = 60
        p = Decimal(str(probability))
        threshold = Decimal(str(alpha))
        if not Decimal(0) < p < Decimal(1) or not Decimal(0) < threshold < Decimal(1):
            raise ValueError("probability and alpha must be in (0,1)")
        minimum = next(
            count for count in range(n + 1)
            if _binomial_upper_tail(n, count, p) <= threshold)
        at_minimum = _binomial_upper_tail(n, minimum, p)
        below = _binomial_upper_tail(n, minimum - 1, p)
        return {
            "n": n,
            "null_probability": str(p),
            "alpha": str(threshold),
            "minimum_success_count": minimum,
            "failure_count_allowed": n - minimum,
            "tail_at_minimum": format(at_minimum, "f"),
            "tail_below_minimum": format(below, "f"),
        }


def calculate_measured_repeatability(
        measurements: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate 60 production measurements and return nearest-rank U95(|D|)."""
    if not isinstance(measurements, list) or len(measurements) != 60:
        _fail("REPEATABILITY_MEASUREMENT_COUNT_INVALID")
    absolute_d_peak: list[float] = []
    frozen_snapshot_identity: dict[str, Any] | None = None
    frozen_sample_identity: dict[str, Any] | None = None
    measurement_fields = {
        "schema_version", "status", "unit", "domain", "measurement_source",
        "invocation_index", "snapshot_identity", "sample_identity",
        "B_original_m", "B_risk_m", "D_peak_m"}
    for expected_index, measurement in enumerate(measurements, start=1):
        if (not isinstance(measurement, dict) or
                set(measurement) != measurement_fields):
            _fail("REPEATABILITY_MEASUREMENT_INVALID")
        if (measurement.get("schema_version") !=
                "icra076_production_replay_measurement_v1" or
                measurement.get("status") != "PASS" or
                measurement.get("unit") != "m" or
                measurement.get("domain") !=
                "controllable_interior_b_equals_2r" or
                measurement.get("measurement_source") !=
                "P4GuideDecision.risk_profile.max" or
                measurement.get("invocation_index") != expected_index):
            _fail("REPEATABILITY_MEASUREMENT_INVALID")
        snapshot_identity = measurement.get("snapshot_identity")
        if (not isinstance(snapshot_identity, dict) or
                set(snapshot_identity) != {
                    "serialized_input_sha256", "snapshot_config_hash",
                    "snapshot_generation"} or
                not isinstance(snapshot_identity.get("serialized_input_sha256"), str) or
                len(snapshot_identity["serialized_input_sha256"]) != 64 or
                not isinstance(snapshot_identity.get("snapshot_config_hash"), str) or
                not snapshot_identity["snapshot_config_hash"] or
                not isinstance(snapshot_identity.get("snapshot_generation"), int) or
                isinstance(snapshot_identity.get("snapshot_generation"), bool) or
                snapshot_identity["snapshot_generation"] <= 0):
            _fail("REPEATABILITY_SNAPSHOT_IDENTITY_INVALID")
        if frozen_snapshot_identity is None:
            frozen_snapshot_identity = snapshot_identity
        elif snapshot_identity != frozen_snapshot_identity:
            _fail("REPEATABILITY_SNAPSHOT_IDENTITY_DRIFT")
        sample_identity = measurement.get("sample_identity")
        if (not isinstance(sample_identity, dict) or
                set(sample_identity) != {
                    "equal_arc_lattice_count", "original_lattice_hash",
                    "risk_lattice_hash", "original_controllable_sample_count",
                    "risk_controllable_sample_count", "original_valid_count",
                    "risk_valid_count"} or
                sample_identity.get("equal_arc_lattice_count") != 200 or
                not isinstance(sample_identity.get("original_lattice_hash"), str) or
                not sample_identity["original_lattice_hash"] or
                not isinstance(sample_identity.get("risk_lattice_hash"), str) or
                not sample_identity["risk_lattice_hash"]):
            _fail("REPEATABILITY_SAMPLE_IDENTITY_INVALID")
        original_count = sample_identity.get(
            "original_controllable_sample_count")
        risk_count = sample_identity.get("risk_controllable_sample_count")
        if (not isinstance(original_count, int) or isinstance(original_count, bool) or
                not isinstance(risk_count, int) or isinstance(risk_count, bool) or
                not 0 < original_count <= 200 or not 0 < risk_count <= 200 or
                sample_identity.get("original_valid_count") != original_count or
                sample_identity.get("risk_valid_count") != risk_count):
            _fail("REPEATABILITY_SAMPLE_IDENTITY_INVALID")
        if frozen_sample_identity is None:
            frozen_sample_identity = sample_identity
        elif sample_identity != frozen_sample_identity:
            _fail("REPEATABILITY_SAMPLE_IDENTITY_DRIFT")
        b_original = _finite_number(
            measurement.get("B_original_m"),
            "REPEATABILITY_MEASUREMENT_INVALID")
        b_risk = _finite_number(
            measurement.get("B_risk_m"),
            "REPEATABILITY_MEASUREMENT_INVALID")
        d_peak = _finite_number(
            measurement.get("D_peak_m"),
            "REPEATABILITY_MEASUREMENT_INVALID")
        if not math.isclose(
                d_peak, b_original - b_risk, rel_tol=0.0, abs_tol=1.0e-12):
            _fail("REPEATABILITY_B_D_INCONSISTENT")
        absolute_d_peak.append(abs(d_peak))
    rank = math.ceil(0.95 * len(absolute_d_peak))
    u95 = sorted(absolute_d_peak)[rank - 1]
    return {
        "schema_version": "icra076_measured_repeatability_calculation_v1",
        "unit": "m",
        "replay_count": len(absolute_d_peak),
        "nearest_rank": rank,
        "absolute_D_peak_m": absolute_d_peak,
        "u95_repeatability_m": u95,
    }


def parse_probe_output(raw: bytes, expected_index: int) -> dict[str, Any]:
    """Parse exactly one machine-readable measurement emitted by the probe."""
    try:
        decoded = raw.decode("utf-8")
        measurement = json.loads(decoded)
    except (UnicodeDecodeError, json.JSONDecodeError):
        _fail("PROBE_OUTPUT_NOT_MEASUREMENT")
    if (not isinstance(measurement, dict) or
            measurement.get("schema_version") !=
            "icra076_production_replay_measurement_v1" or
            measurement.get("invocation_index") != expected_index or
            not all(field in measurement for field in (
                "B_original_m", "B_risk_m", "D_peak_m",
                "snapshot_identity", "sample_identity"))):
        _fail("PROBE_OUTPUT_NOT_MEASUREMENT")
    return measurement


def _verify_absolute_file_record(record: dict[str, Any]) -> None:
    if set(record) != {"path", "type", "size_bytes", "sha256"}:
        _fail("REPLAY_FILE_IDENTITY_INVALID")
    path = _admit_input_path(Path(record["path"]))
    if (record.get("type") != "regular_file" or not path.is_file() or
            path.is_symlink() or path.stat().st_size != record.get("size_bytes") or
            file_sha256(path) != record.get("sha256")):
        _fail("REPLAY_EXECUTABLE_OR_INPUT_DRIFT", str(path))


def load_measured_replay_manifest(
        manifest_path: Path, expected_source_head: str | None = None,
        repository: Path = REPOSITORY) -> dict[str, Any]:
    """Validate every retained probe emission and calculate U95(|D_peak|)."""
    repository = repository.resolve()
    manifest_path = _repository_input_path(manifest_path, repository)
    try:
        manifest_path.relative_to(repository / "results/icra27/icra076")
    except ValueError:
        _fail("REPLAY_MANIFEST_OUTSIDE_ALLOWED_ROOT")
    manifest = _json(manifest_path)
    required_fields = {
        "schema_version", "task", "outcome_blind", "held_out_accessed",
        "source_head", "executable", "serialized_input", "measurement_count",
        "measurements", "calculation"}
    if (set(manifest) != required_fields or
            manifest.get("schema_version") !=
            "icra076_production_measured_replay_manifest_v1" or
            manifest.get("task") != "ICRA-076" or
            manifest.get("outcome_blind") is not True or
            manifest.get("held_out_accessed") is not False or
            not isinstance(manifest.get("source_head"), str) or
            len(manifest["source_head"]) != 40 or
            (expected_source_head is not None and
             manifest["source_head"] != expected_source_head)):
        _fail("REPLAY_MANIFEST_CONTRACT_INVALID")
    _verify_absolute_file_record(manifest.get("executable", {}))
    serialized_input = manifest.get("serialized_input", {})
    if set(serialized_input) != {"path", "type", "size_bytes", "sha256"}:
        _fail("REPLAY_FILE_IDENTITY_INVALID")
    input_path = _repository_path(repository, serialized_input["path"])
    if (serialized_input.get("type") != "regular_file" or
            not input_path.is_file() or input_path.is_symlink() or
            input_path.stat().st_size != serialized_input.get("size_bytes") or
            file_sha256(input_path) != serialized_input.get("sha256")):
        _fail("REPLAY_EXECUTABLE_OR_INPUT_DRIFT", str(input_path))
    records = manifest.get("measurements")
    if (manifest.get("measurement_count") != 60 or
            not isinstance(records, list) or len(records) != 60):
        _fail("REPEATABILITY_MEASUREMENT_COUNT_INVALID")
    measurements: list[dict[str, Any]] = []
    manifest_root = manifest_path.parent
    executable_path = manifest["executable"]["path"]
    for index, record in enumerate(records, start=1):
        expected_argv = [
            executable_path, "--snapshot", str(input_path),
            "--invocation-index", str(index)]
        if (not isinstance(record, dict) or set(record) != {
                "invocation_index", "argv", "exit_code", "output"} or
                record.get("invocation_index") != index or
                record.get("argv") != expected_argv or
                record.get("exit_code") != 0):
            _fail("REPLAY_INVOCATION_IDENTITY_INVALID")
        output_record = record.get("output", {})
        if (not isinstance(output_record, dict) or
                set(output_record) != {"path", "type", "size_bytes", "sha256"} or
                output_record.get("path") != f"measurement-{index:03d}.json"):
            _fail("REPLAY_EMISSION_IDENTITY_INVALID")
        output_path = _repository_input_path(
            manifest_root / output_record["path"], repository)
        try:
            output_path.relative_to(manifest_root)
        except ValueError:
            _fail("REPLAY_EMISSION_IDENTITY_INVALID")
        raw = output_path.read_bytes()
        if (output_record.get("type") != "regular_file" or
                output_path.is_symlink() or len(raw) != output_record.get("size_bytes") or
                hashlib.sha256(raw).hexdigest() != output_record.get("sha256")):
            _fail("REPLAY_EMISSION_BYTE_DRIFT", output_record["path"])
        measurement = parse_probe_output(raw, index)
        if (measurement["snapshot_identity"]["serialized_input_sha256"] !=
                serialized_input["sha256"]):
            _fail("REPLAY_SERIALIZED_INPUT_IDENTITY_MISMATCH")
        measurements.append(measurement)
    calculation = calculate_measured_repeatability(measurements)
    if manifest.get("calculation") != calculation:
        _fail("REPEATABILITY_CALCULATION_DRIFT")
    return {
        **calculation,
        "measurement_count": len(measurements),
        "source_head": manifest["source_head"],
        "manifest_sha256": file_sha256(manifest_path),
        "executable": manifest["executable"],
        "serialized_input": serialized_input,
    }


def _repeatability_bound(replay: dict[str, Any], repository: Path) -> dict[str, Any]:
    expected_fields = {
        "schema_version", "outcome_blind", "held_out", "unit",
        "replay_count", "sample_count", "measured_replay_manifest_path",
        "probe_executable_path", "serialized_input_path",
        "required_replay_command", "calculation"}
    if set(replay) != expected_fields:
        _fail("REPEATABILITY_FIELDS_INVALID")
    if replay.get("schema_version") != "icra076_production_measured_replay_binding_v2":
        _fail("REPEATABILITY_SCHEMA_MISMATCH")
    if replay.get("outcome_blind") is not True or replay.get("held_out") is not False:
        _fail("REPEATABILITY_ADMISSION_INVALID")
    if replay.get("unit") != "m":
        _fail("STATISTICAL_UNIT_INVALID", "repeatability")
    count = replay.get("replay_count")
    if count != 60 or replay.get("sample_count") != 200:
        _fail("REPEATABILITY_CARDINALITY_INVALID")
    required_replay_command = [
        "python3", "scripts/dev_planner/icra076_repeatability_replay.py",
        "--snapshot", "config/icra27/icra076_flat_null_snapshot_v1.json",
        "--output-root", "results/icra27/icra076/repeatability-replay-003",
    ]
    if replay.get("required_replay_command") != required_replay_command:
        _fail("REPEATABILITY_COMMAND_DRIFT")
    if replay.get("calculation") != {
            "metric": "abs(D_peak)",
            "u95_method": "nearest_rank_ceil_0.95_n",
            "nearest_rank": 57}:
        _fail("REPEATABILITY_METHOD_INVALID")
    if replay.get("probe_executable_path") != \
            "/home/dev/ws_iap/build/bspline_opt/icra076_repeatability_probe":
        _fail("REPEATABILITY_PROBE_IDENTITY_INVALID")
    if replay.get("serialized_input_path") != \
            "config/icra27/icra076_flat_null_snapshot_v1.json":
        _fail("REPEATABILITY_SERIALIZED_INPUT_IDENTITY_INVALID")
    manifest_relative = replay.get("measured_replay_manifest_path")
    if isinstance(manifest_relative, str):
        _reject_forbidden_path_tokens(Path(manifest_relative))
    if manifest_relative != \
            "results/icra27/icra076/repeatability-replay-003/manifest.json":
        _fail("REPEATABILITY_MANIFEST_IDENTITY_INVALID")
    manifest_path = _repository_path(repository, manifest_relative)
    if not manifest_path.exists():
        return {
            "schema_version": "icra076_repeatability_calculation_pending_v2",
            "unit": "m", "replay_count": count, "nearest_rank": 57,
            "pending": True, "u95_repeatability_m": None,
            "input_canonical_sha256": canonical_sha256(replay),
            "manifest_path": manifest_relative,
        }
    measured = load_measured_replay_manifest(manifest_path, repository=repository)
    expected_executable = replay["probe_executable_path"]
    expected_input = replay["serialized_input_path"]
    if measured["executable"].get("path") != expected_executable:
        _fail("REPLAY_EXECUTABLE_PATH_DRIFT")
    if measured["serialized_input"].get("path") != expected_input:
        _fail("REPLAY_SERIALIZED_INPUT_PATH_DRIFT")
    return {
        **measured,
        "pending": False,
        "input_canonical_sha256": canonical_sha256(replay),
        "manifest_path": manifest_relative,
    }


def build_execution_order(registry: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    seeds = registry["confirmatory_seeds"]
    scenes = list(SCENES)
    for round_index in range(60):
        rotation = round_index % len(scenes)
        scene_order = scenes[rotation:] + scenes[:rotation]
        for scene in scene_order:
            scene_index = scenes.index(scene)
            arm_order = list(ARMS)
            if (round_index + scene_index) % 2:
                arm_order.reverse()
            seed = seeds[scene][round_index]
            for arm in arm_order:
                execution_index = len(rows) + 1
                rows.append({
                    "execution_index": execution_index,
                    "round": round_index + 1,
                    "scene": scene,
                    "seed": seed,
                    "arm": arm,
                    "run_id": (
                        f"icra077-{execution_index:03d}-{scene.lower()}-"
                        f"{seed}-{arm.lower()}"),
                })
    return rows


def _validate_seed_registry(registry: dict[str, Any], repository: Path) -> None:
    if set(registry) != {"schema_version", "outcome_blind",
                        "historical_seed_exclusion_policy", "confirmatory_seeds"}:
        _fail("SEED_REGISTRY_FIELDS_INVALID")
    if registry.get("schema_version") != "icra076_seed_registry_v1":
        _fail("SEED_REGISTRY_SCHEMA_MISMATCH")
    if registry.get("outcome_blind") is not True:
        _fail("SEED_REGISTRY_NOT_OUTCOME_BLIND")
    exclusion = registry.get("historical_seed_exclusion_policy", {})
    if set(exclusion) != {"reserved_inclusive_range", "reason", "bound_sources",
                          "known_seed_values"}:
        _fail("HISTORICAL_SEED_REGISTRY_INVALID")
    reserved = exclusion.get("reserved_inclusive_range")
    if reserved != [0, 99999999]:
        _fail("HISTORICAL_SEED_RANGE_DRIFT")
    bound_sources = exclusion.get("bound_sources", [])
    expected_sources = (
        "config/icra27/p4_g0c_protocol_v1.json",
        "config/icra27/p4_g0c_protocol_v2.json",
        "config/icra27/p4_g0c_protocol_v3.json",
        "config/icra27/p4_g0c_protocol_v4.json",
        "config/icra27/p4_g0c_protocol_v5.json",
        "config/icra27/p4_g0c_protocol_v6.json",
        "config/icra27/icra075_exploratory_protocol_v1.json",
        "docs/icra27/GATE0_QUALIFICATION_REPORT.md",
        "docs/icra27/ICRA_IMPLEMENTATION_PLAN.md",
    )
    if (not isinstance(bound_sources, list) or
            tuple(record.get("path") for record in bound_sources) !=
            expected_sources):
        _fail("HISTORICAL_SEED_SOURCES_INCOMPLETE")
    for record in bound_sources:
        _verify_bound_file(repository, record)
    known = exclusion.get("known_seed_values")
    expected_known = [
        1, 11, 23, 37, 53, 71, 89, 107, 127, 149, 173,
        211, 223, 237, 253, 271, 73001,
        75001, 75002, 75003, 75004, 75005, 11022, 20260011]
    if (known != expected_known or
            any(value < reserved[0] or value > reserved[1] for value in known)):
        _fail("HISTORICAL_SEED_REGISTRY_INVALID")
    seeds = registry.get("confirmatory_seeds", {})
    if tuple(seeds) != SCENES:
        _fail("SCENE_REGISTRY_MISMATCH")
    flattened: list[int] = []
    for scene in SCENES:
        values = seeds[scene]
        if (not isinstance(values, list) or len(values) != 60 or
                any(not isinstance(value, int) or isinstance(value, bool)
                    for value in values) or len(values) != len(set(values))):
            _fail("CONFIRMATORY_SEED_CARDINALITY_INVALID", scene)
        if any(value <= reserved[1] or value > 2147483647 for value in values):
            _fail("HISTORICAL_SEED_REUSE", scene)
        flattened.extend(values)
    if len(flattened) != len(set(flattened)):
        _fail("CONFIRMATORY_SEED_REUSED_ACROSS_SCENES")
    for scene in SCENES:
        expected_start = {
            "PRIMARY": 100000001,
            "EXACT_MIRROR": 100000101,
            "FLAT_NULL": 100000201,
        }[scene]
        if seeds[scene] != list(range(expected_start, expected_start + 60)):
            _fail("CONFIRMATORY_SEED_GENERATION_DRIFT", scene)


def validate_preregistration(
        protocol_path: Path, registry_path: Path,
        repository: Path = REPOSITORY, schema_path: Path | None = None,
        replay_path: Path | None = None) -> dict[str, Any]:
    repository = repository.resolve()
    protocol_path = _repository_input_path(protocol_path, repository)
    registry_path = _repository_input_path(registry_path, repository)
    schema_path = _repository_input_path(
        schema_path or (repository / SCHEMA_PATH.relative_to(REPOSITORY)),
        repository)
    replay_path = _repository_input_path(
        replay_path or (repository / REPLAY_PATH.relative_to(REPOSITORY)),
        repository)
    protocol = _json(protocol_path)
    registry = _json(registry_path)
    schema = _json(schema_path)
    replay = _json(replay_path)
    if schema.get("schema_version") != "icra076_preregistration_schema_v1":
        _fail("SCHEMA_DOWNGRADE_OR_MISMATCH")
    required = schema.get("required_top_level_fields")
    if not isinstance(required, list) or set(protocol) != set(required):
        _fail("PROTOCOL_FIELDS_INVALID")
    if protocol.get("schema_version") != schema.get("protocol_schema_version"):
        _fail("PROTOCOL_SCHEMA_DOWNGRADE_OR_MISMATCH")
    if (protocol.get("task") != "ICRA-076" or
            protocol.get("outcome_blind") is not True or
            protocol.get("held_out_access") is not False or
            protocol.get("empirical_power_claim") is not False):
        _fail("OUTCOME_BLIND_CONTRACT_INVALID")
    if protocol.get("formal_arms") != list(ARMS):
        _fail("FORMAL_ARMS_INVALID")
    if protocol.get("scenes") != list(SCENES):
        _fail("FORMAL_SCENES_INVALID")
    _validate_seed_registry(registry, repository)
    seed_binding = protocol.get("seed_registry", {})
    if (seed_binding.get("path") !=
            str(registry_path.resolve().relative_to(repository)) or
            seed_binding.get("canonical_sha256") != canonical_sha256(registry)):
        _fail("SEED_REGISTRY_IDENTITY_MISMATCH")
    repeatability = _repeatability_bound(replay, repository)
    repeatability_contract = protocol.get("repeatability", {})
    if (repeatability_contract.get("input_path") !=
            str(replay_path.relative_to(repository)) or
            repeatability_contract.get("input_canonical_sha256") !=
            repeatability["input_canonical_sha256"] or
            repeatability_contract.get("u95_repeatability") !=
            "FROZEN_FROM_MEASURED_REPLAY_MANIFEST" or
            repeatability_contract.get("unit") != "m" or
            repeatability_contract.get("outcome_derived") is not False):
        _fail("REPEATABILITY_CONTRACT_MISMATCH")
    estimand = protocol.get("estimand", {})
    resolution = _finite_number(estimand.get("risk_grid_resolution_m"),
                                "ESTIMAND_VALUE_INVALID")
    endpoint_buffer = _finite_number(estimand.get("endpoint_buffer_m"),
                                     "ESTIMAND_VALUE_INVALID")
    if (estimand.get("primary") != "D_peak=B_original-B_risk" or
            estimand.get("risk_quantity") != "provider_only_predicted_c_pi" or
            estimand.get("domain") != "controllable_interior" or
            estimand.get("endpoint_buffer_rule") != "b=2r" or
            endpoint_buffer != 2.0 * resolution or estimand.get("unit") != "m" or
            estimand.get("one_primary_event_per_independent_run_seed") is not True):
        _fail("ESTIMAND_CONTRACT_INVALID")
    _verify_bound_file(repository, {
        "path": estimand.get("resolution_authority", {}).get("path"),
        "sha256": estimand.get("resolution_authority", {}).get("sha256")})
    sesoi = protocol.get("domain_sesoi", {})
    sesoi_value = _finite_number(sesoi.get("value"), "DOMAIN_SESOI_INVALID")
    if (sesoi_value != 0.3 or sesoi.get("unit") != "m" or
            sesoi.get("outcome_derived") is not False or
            sesoi.get("authority", {}).get("key") != "p5.current_replan_margin_m" or
            sesoi.get("authority", {}).get("value") != sesoi_value):
        _fail("DOMAIN_SESOI_CONTRACT_INVALID")
    _verify_bound_file(repository, {
        "path": sesoi.get("authority", {}).get("path"),
        "sha256": sesoi.get("authority", {}).get("sha256")})
    delta = protocol.get("delta_peak", {})
    delta_value = (None if repeatability["pending"] else
                   max(sesoi_value, repeatability["u95_repeatability_m"]))
    if (delta.get("formula") != "max(domain_SESOI,U95_repeatability)" or
            delta.get("unit") != "m" or
            delta.get("value") != "FROZEN_FROM_MEASURED_REPLAY_MANIFEST"):
        _fail("DELTA_PEAK_CONTRACT_INVALID")
    sample = protocol.get("sample_size", {})
    if sample != {
            "independent_seed_runs_per_scene": 60,
            "paired_arms_per_seed": 2,
            "total_seed_runs": 180,
            "total_arm_runs": 360,
            "basis": "USER_DECISION_006_CONSERVATIVE_ROUTE_CEILING",
            "empirical_power_demonstrated": False}:
        _fail("SAMPLE_SIZE_CONTRACT_INVALID")
    binomial = protocol.get("exact_binomial_rule", {})
    computed_rule = exact_binomial_passing_rule(60, 0.9, 0.05)
    if (binomial.get("alternative") != "Pr(D_peak>delta_peak)>0.9" or
            binomial.get("null_probability") != 0.9 or
            binomial.get("alpha_one_sided") != 0.05 or
            binomial.get("n") != 60 or
            binomial.get("minimum_success_count") != computed_rule["minimum_success_count"] or
            binomial.get("failure_count_allowed") != computed_rule["failure_count_allowed"] or
            binomial.get("calculation") != "exact_binomial_upper_tail"):
        _fail("BINOMIAL_RULE_DRIFT")
    order = build_execution_order(registry)
    order_contract = protocol.get("execution_order", {})
    if order_contract != {
            "scene_cycle": list(SCENES),
            "rounds": 60,
            "round_scene_rotation": "left_rotate_by_round_mod_3",
            "paired_arm_order": (
                "CONTROL_FIRST_IF_(round+scene_index)_EVEN_ELSE_TREATMENT_FIRST"),
            "expanded_order_sha256": canonical_sha256(order),
            "no_retry": True}:
        _fail("EXECUTION_ORDER_INVALID")
    gates = protocol.get("formal_gates", {})
    expected_gates = {
        "PRIMARY": {
            "primary_success_count_min": 59,
            "mirror_direction": "NOT_APPLICABLE"},
        "EXACT_MIRROR": {
            "primary_success_count_min": 59,
            "mirror_direction": (
                "treatment_safe_homotopy_y_sign_negative_and_effect_direction_positive")},
        "FLAT_NULL": {
                "absolute_D_peak_max_m": 0.0,
                "provider_risk_selection_count_max": 0,
                "complete_pairs_required": 60},
        "secondary": {
            "D_mean_noninferiority_min_m": 0.0,
            "whole_path_D_peak_noninferiority_min_m": 0.0,
            "path_length_ratio_max": 1.3,
            "per_search_timeout_s": 0.2,
            "provider_support": "FINITE_COMPLETE",
            "coverage": "COMPLETE",
            "timeout_count_max": 0,
            "fallback_count_max": 0,
            "collision_count_max": 0,
            "dynamics_failure_count_max": 0,
            "p5_final": "PASS",
            "normal_publication_identity": "MATCH",
            "p5_runtime_identity": "MATCH"}}
    if gates != expected_gates:
        _fail("FORMAL_GATE_DRIFT")
    missing = protocol.get("missing_data_policy", {})
    if missing != {
            "missing_or_incomplete_run": "FAIL_AND_RETAIN_IN_DENOMINATOR",
            "exclusion_allowed": False,
            "retry_allowed": False,
            "extra_primary_events": "DESCRIPTIVE_ONLY_RUN_CLUSTERED_NO_N_INCREMENT",
            "disabled_or_skipped_required_test": "FAIL_BEFORE_RESULT_CREATION"}:
        _fail("MISSING_DATA_POLICY_INVALID")
    byte_freeze = protocol.get("byte_freeze", {})
    if (set(byte_freeze) != {"source_inventory_roots", "shared_install_root",
                             "rebuilt_packages", "runtime_install_packages",
                             "inventory_record_fields", "drift_policy"} or
            byte_freeze.get("shared_install_root") != "/home/dev/ws_iap/install" or
            byte_freeze.get("rebuilt_packages") != [
                "iap", "plan_env", "traj_utils", "path_searching",
                "bspline_opt", "ego_planner"] or
            byte_freeze.get("runtime_install_packages") != [
                "iap", "plan_env", "traj_utils", "path_searching",
                "bspline_opt", "ego_planner", "map_generator", "gnss_sim",
                "local_sensing", "odom_visualization", "poscmd_2_odom",
                "so3_quadrotor_simulator", "so3_control"] or
            byte_freeze.get("inventory_record_fields") != [
                "path", "type", "size_bytes", "sha256"] or
            byte_freeze.get("drift_policy") != "INVALIDATE_BEFORE_ICRA077"):
        _fail("BYTE_FREEZE_CONTRACT_INVALID")
    output = protocol.get("output_policy", {})
    if output != {
            "allowed_root": "results/icra27/icra076",
            "non_overwriting": True,
            "reject_symlink_components": True,
            "forbidden_input_tokens": list(FORBIDDEN_INPUT_TOKENS)}:
        _fail("OUTPUT_POLICY_INVALID")
    return {
        "schema_version": "icra076_validated_preregistration_v1",
        "protocol_canonical_sha256": canonical_sha256(protocol),
        "registry_canonical_sha256": canonical_sha256(registry),
        "repeatability": repeatability,
        "repeatability_pending": repeatability["pending"],
        "u95_repeatability_m": repeatability["u95_repeatability_m"],
        "delta_peak_m": delta_value,
        "endpoint_buffer_m": endpoint_buffer,
        "minimum_success_count": computed_rule["minimum_success_count"],
        "binomial_rule": computed_rule,
        "execution_order": order,
        "execution_order_sha256": canonical_sha256(order),
    }


def _run_git(repository: Path, argv: list[str]) -> str:
    completed = subprocess.run(
        ["git", *argv], cwd=repository, capture_output=True, text=True,
        check=False)
    if completed.returncode != 0:
        _fail("SOURCE_GIT_COMMAND_FAILED", " ".join(argv))
    return completed.stdout.strip()


def source_admission(repository: Path = REPOSITORY) -> dict[str, Any]:
    repository = repository.resolve()
    head = _run_git(repository, ["rev-parse", "HEAD"])
    remote = _run_git(repository, ["rev-parse", "origin/dev/icra"])
    divergence_text = _run_git(
        repository, ["rev-list", "--left-right", "--count",
                     "HEAD...origin/dev/icra"])
    try:
        divergence = [int(value) for value in divergence_text.split()]
    except ValueError:
        _fail("SOURCE_DIVERGENCE_INVALID", divergence_text)
    status = _run_git(
        repository, ["status", "--porcelain=v1", "--untracked-files=all"])
    expected_status = "?? docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf"
    if head != remote or divergence != [0, 0] or status != expected_status:
        _fail("SOURCE_NOT_PUSHED_CLEAN", json.dumps({
            "head": head, "remote": remote, "divergence": divergence,
            "status": status}, sort_keys=True))
    protected = (
        (".claude/settings.local.json", 72,
         "27aac0ccca0ad0ab573578864cf27b9560d3f819bdeeae62378f8c20e62a8f64"),
        ("src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~", 2962,
         "29a73228c1d58d5c4983dc217230d6cbc6ff6e295a77a004d8e6a207fd242028"),
        ("docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf", 243368,
         "1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6"),
    )
    artifacts = []
    for relative, size, digest in protected:
        path = repository / relative
        if (not path.is_file() or path.is_symlink() or
                path.stat().st_size != size or file_sha256(path) != digest):
            _fail("PROTECTED_ARTIFACT_DRIFT", relative)
        artifacts.append({
            "path": relative, "type": "regular_file",
            "size_bytes": size, "sha256": digest})
    return {
        "schema_version": "icra076_pushed_source_admission_v1",
        "head_commit": head,
        "origin_dev_icra_commit": remote,
        "left_right_divergence": divergence,
        "status_porcelain": status,
        "protected_artifacts": artifacts,
    }


def _tracked_paths(repository: Path, roots: list[str]) -> list[Path]:
    output = _run_git(repository, ["ls-files", "--", *roots])
    paths = [repository / line for line in output.splitlines() if line]
    if not paths:
        _fail("SOURCE_INVENTORY_EMPTY")
    return sorted(paths)


def collect_source_inventory(protocol: dict[str, Any],
                             repository: Path = REPOSITORY) -> list[dict[str, Any]]:
    roots = protocol.get("byte_freeze", {}).get("source_inventory_roots")
    if not isinstance(roots, list) or not roots or len(roots) != len(set(roots)):
        _fail("SOURCE_INVENTORY_ROOTS_INVALID")
    for relative in roots:
        path = _repository_path(repository, relative)
        if not path.exists() or path.is_symlink():
            _fail("SOURCE_INVENTORY_ROOT_INVALID", relative)
    records = [inventory_path(path, repository)
               for path in _tracked_paths(repository, roots)]
    if len({record["path"] for record in records}) != len(records):
        _fail("SOURCE_INVENTORY_DUPLICATE")
    return records


def collect_install_inventory(protocol: dict[str, Any],
                              install_root: Path) -> list[dict[str, Any]]:
    freeze = protocol.get("byte_freeze", {})
    packages = freeze.get("runtime_install_packages")
    if packages != [
            "iap", "plan_env", "traj_utils", "path_searching", "bspline_opt",
            "ego_planner", "map_generator", "gnss_sim", "local_sensing",
            "odom_visualization", "poscmd_2_odom", "so3_quadrotor_simulator",
            "so3_control"]:
        _fail("INSTALL_PACKAGE_SET_INVALID")
    install_root = install_root.absolute()
    records: list[dict[str, Any]] = []
    for package in packages:
        root = install_root / package
        if not root.is_dir() or root.is_symlink():
            _fail("INSTALL_PACKAGE_ROOT_INVALID", package)
        package_paths = sorted(
            path for path in root.rglob("*")
            if path.is_file() or path.is_symlink())
        if not package_paths:
            _fail("INSTALL_PACKAGE_EMPTY", package)
        records.extend(inventory_path(path, install_root)
                       for path in package_paths)
    if len({record["path"] for record in records}) != len(records):
        _fail("INSTALL_INVENTORY_DUPLICATE")
    return records


def create_freeze_record(
        protocol_path: Path, registry_path: Path, output_path: Path,
        verification: dict[str, Any], repository: Path = REPOSITORY,
        verification_path: Path | None = None,
        install_root: Path = Path("/home/dev/ws_iap/install")) -> dict[str, Any]:
    repository = repository.resolve()
    validated = validate_preregistration(
        protocol_path, registry_path, repository)
    if validated["repeatability_pending"]:
        _fail("MEASURED_REPLAY_REQUIRED")
    protocol_path = _repository_input_path(protocol_path, repository)
    registry_path = _repository_input_path(registry_path, repository)
    protocol = _json(protocol_path)
    validate_verification(verification)
    replay_command = next(
        command for command in verification["commands"]
        if command["category"] == "REPEATABILITY_REPLAY")
    replay = _json(repository / REPLAY_PATH.relative_to(REPOSITORY))
    if replay_command["argv"] != replay["required_replay_command"]:
        _fail("REPEATABILITY_VERIFICATION_MISMATCH")
    admission = source_admission(repository)
    if validated["repeatability"].get("source_head") != admission["head_commit"]:
        _fail("REPLAY_SOURCE_HEAD_DRIFT")
    validate_verification(verification, admission["head_commit"])
    if verification_path is None:
        _fail("VERIFICATION_BINDING_REQUIRED")
    verification_path = _repository_input_path(verification_path, repository)
    verification_binding = inventory_path(verification_path, repository)
    allowed_root = repository / protocol["output_policy"]["allowed_root"]
    output = validate_output_path(output_path, repository, allowed_root)
    source_inventory = collect_source_inventory(protocol, repository)
    install_inventory = collect_install_inventory(protocol, install_root)
    record = {
        "schema_version": "icra076_preregistration_freeze_v1",
        "task": "ICRA-076",
        "result": "PASS",
        "outcome_blind": True,
        "held_out_accessed": False,
        "empirical_power_claim": False,
        "source_admission": admission,
        "protocol": {
            "path": str(protocol_path.resolve().relative_to(repository)),
            "canonical_sha256": validated["protocol_canonical_sha256"],
        },
        "seed_registry": {
            "path": str(registry_path.resolve().relative_to(repository)),
            "canonical_sha256": validated["registry_canonical_sha256"],
        },
        "validated_contract": {
            "endpoint_buffer_m": validated["endpoint_buffer_m"],
            "domain_sesoi_m": protocol["domain_sesoi"]["value"],
            "u95_repeatability_m": validated["u95_repeatability_m"],
            "delta_peak_m": validated["delta_peak_m"],
            "sample_size": protocol["sample_size"],
            "binomial_rule": validated["binomial_rule"],
            "formal_gates": protocol["formal_gates"],
            "missing_data_policy": protocol["missing_data_policy"],
        },
        "repeatability_calculation": validated["repeatability"],
        "execution_order": validated["execution_order"],
        "execution_order_sha256": validated["execution_order_sha256"],
        "source_inventory": source_inventory,
        "source_inventory_sha256": canonical_sha256(source_inventory),
        "install_root": str(install_root.absolute()),
        "install_inventory": install_inventory,
        "install_inventory_sha256": canonical_sha256(install_inventory),
        "verification": verification,
        "verification_binding": verification_binding,
        "icra075_disposition": (
            "BLOCKED_USER_ACCEPTED_BYPASS_NOT_PASS_0_OF_40_NO_POWER_INPUTS"),
        "icra077_authorized": False,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as stream:
        stream.write(json.dumps(record, indent=2, sort_keys=True) + "\n")
    return record


def validate_freeze_record(
        record_path: Path, repository: Path = REPOSITORY,
        install_root: Path = Path("/home/dev/ws_iap/install")) -> dict[str, Any]:
    repository = repository.resolve()
    record_path = _repository_input_path(record_path, repository)
    record = _json(record_path)
    if (record.get("schema_version") != "icra076_preregistration_freeze_v1" or
            record.get("task") != "ICRA-076" or record.get("result") != "PASS" or
            record.get("outcome_blind") is not True or
            record.get("held_out_accessed") is not False or
            record.get("empirical_power_claim") is not False or
            record.get("icra077_authorized") is not False):
        _fail("FREEZE_RECORD_CONTRACT_INVALID")
    protocol_binding = record.get("protocol", {})
    registry_binding = record.get("seed_registry", {})
    protocol_path = _repository_path(repository, protocol_binding.get("path", ""))
    registry_path = _repository_path(repository, registry_binding.get("path", ""))
    validated = validate_preregistration(protocol_path, registry_path, repository)
    if (protocol_binding.get("canonical_sha256") !=
            validated["protocol_canonical_sha256"] or
            registry_binding.get("canonical_sha256") !=
            validated["registry_canonical_sha256"] or
            record.get("execution_order_sha256") !=
            validated["execution_order_sha256"] or
            record.get("execution_order") != validated["execution_order"]):
        _fail("FREEZE_IDENTITY_DRIFT")
    admission = record.get("source_admission", {})
    current_admission = source_admission(repository)
    head = current_admission["head_commit"]
    frozen_head = admission.get("head_commit")
    if (not isinstance(frozen_head, str) or
            admission.get("origin_dev_icra_commit") != frozen_head or
            admission.get("left_right_divergence") != [0, 0]):
        _fail("FREEZE_SOURCE_HEAD_DRIFT")
    ancestor = subprocess.run(
        ["git", "merge-base", "--is-ancestor", frozen_head, head],
        cwd=repository, capture_output=True, text=True, check=False)
    if ancestor.returncode != 0:
        _fail("FREEZE_SOURCE_HEAD_DRIFT")
    validate_verification(record.get("verification", {}), frozen_head)
    verification_binding = record.get("verification_binding", {})
    if not isinstance(verification_binding, dict):
        _fail("VERIFICATION_BINDING_INVALID")
    verification_path = _repository_path(
        repository, verification_binding.get("path", ""))
    if inventory_path(verification_path, repository) != verification_binding:
        _fail("VERIFICATION_BYTE_DRIFT")
    if load_verification(verification_path, repository) != record.get("verification"):
        _fail("VERIFICATION_BYTE_DRIFT")
    protocol = _json(protocol_path)
    source_inventory = collect_source_inventory(protocol, repository)
    install_inventory = collect_install_inventory(protocol, install_root)
    if (record.get("source_inventory") != source_inventory or
            record.get("source_inventory_sha256") !=
            canonical_sha256(source_inventory)):
        _fail("SOURCE_BYTE_DRIFT")
    if (record.get("install_root") != str(install_root.absolute()) or
            record.get("install_inventory") != install_inventory or
            record.get("install_inventory_sha256") !=
            canonical_sha256(install_inventory)):
        _fail("INSTALL_BYTE_DRIFT")
    expected_contract = {
        "endpoint_buffer_m": validated["endpoint_buffer_m"],
        "domain_sesoi_m": protocol["domain_sesoi"]["value"],
        "u95_repeatability_m": validated["u95_repeatability_m"],
        "delta_peak_m": validated["delta_peak_m"],
        "sample_size": protocol["sample_size"],
        "binomial_rule": validated["binomial_rule"],
        "formal_gates": protocol["formal_gates"],
        "missing_data_policy": protocol["missing_data_policy"],
    }
    if (record.get("validated_contract") != expected_contract or
            record.get("repeatability_calculation") != validated["repeatability"]):
        _fail("FREEZE_CALCULATION_DRIFT")
    return {
        "schema_version": "icra076_freeze_validation_v1",
        "result": "PASS",
        "source_inventory_count": len(source_inventory),
        "install_inventory_count": len(install_inventory),
        "execution_order_count": len(validated["execution_order"]),
        "source_head": head,
    }
