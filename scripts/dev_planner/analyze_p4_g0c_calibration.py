#!/usr/bin/env python3
"""Fail-closed analyzer for a complete registered P4-G0C calibration bundle."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from p4_g0c_protocol import (  # noqa: E402
    DecisionSchemaError,
    ProtocolBundle,
    decision_identity,
    effective_config_sha256,
    expand_run_plan,
    load_protocol_bundle,
    parse_decision_row,
    validate_decision_header,
)


ANALYSIS_SCHEMA = "p4_g0c_analysis_v1"
DRAFT_SCHEMA = "p4_g0c_threshold_draft_v1"
RUN_MANIFEST_SCHEMA = "p4_g0c_run_manifest_v1"
RUNNER_STATE_SCHEMA = "p4_g0c_runner_state_v2"
RUNNER_STATE_FILENAME = "p4_g0c_runner_state.json"
REQUIRED_PROCESSES = ["iap_rosnode", "ego_planner_node"]
ALLOWED_ROOT_METADATA = {
    RUNNER_STATE_FILENAME,
    "p4_g0c_analysis.json",
    "p4_g0c_threshold_draft.json",
}
ALLOWED_PREFLIGHT_METADATA = {"gpu_preflight.json"}


class AnalysisError(RuntimeError):
    """A typed input or deterministic computation is invalid."""


def load_bundle(
    protocol_path: Path, registry_path: Path, fixture_path: Path
) -> ProtocolBundle:
    bundle = load_protocol_bundle(protocol_path, registry_path, fixture_path)
    bundle.protocol_path = str(Path(protocol_path).resolve())
    bundle.registry_path = str(Path(registry_path).resolve())
    bundle.fixture_path = str(Path(fixture_path).resolve())
    return bundle


def milliseconds_to_seconds(value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise AnalysisError("millisecond value must be finite")
    return value / 1000.0


def quantile_type7(values: list[tuple[float, int]], probability: float) -> dict[str, Any]:
    if not values:
        raise AnalysisError("quantile requires at least one value")
    if not math.isfinite(probability) or not 0.0 <= probability <= 1.0:
        raise AnalysisError("quantile probability must be finite in [0,1]")
    normalized = []
    for value, source_index in values:
        value = float(value)
        if not math.isfinite(value):
            raise AnalysisError("quantile values must be finite")
        normalized.append((value, int(source_index)))
    ordered = sorted(normalized, key=lambda item: (item[0], item[1]))
    h = (len(ordered) - 1) * probability
    lower = math.floor(h)
    upper = math.ceil(h)
    fraction = h - lower
    lower_value, lower_source = ordered[lower]
    upper_value, upper_source = ordered[upper]
    value = lower_value + fraction * (upper_value - lower_value)
    return {
        "value": value,
        "probability": probability,
        "lower_sorted_index": lower,
        "upper_sorted_index": upper,
        "lower_source_row_index": lower_source,
        "upper_source_row_index": upper_source,
        "fraction": fraction,
    }


def _raw_bundle_hash(root: Path, paths: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in sorted(set(paths), key=lambda item: str(item.relative_to(root))):
        relative = str(path.relative_to(root)).encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
        digest.update(b"\0")
    return digest.hexdigest()


def _root_inventory_failures(
    root: Path, registered_run_ids: list[str]
) -> tuple[list[str], list[Path]]:
    failures = []
    bundle_paths: list[Path] = []
    allowed_names = set(registered_run_ids) | ALLOWED_ROOT_METADATA | {
        "preflight"
    }
    try:
        entries = list(root.iterdir())
    except OSError as exc:
        return [f"root_inventory_unreadable:{exc}"], bundle_paths
    for entry in entries:
        if entry.name not in allowed_names:
            failures.append(f"root_inventory_unregistered:{entry.name}")
        if entry.is_symlink():
            failures.append(f"root_inventory_symlink:{entry.name}")
    for run_id in registered_run_ids:
        path = root / run_id
        if not path.is_dir() or path.is_symlink():
            failures.append(f"root_inventory_registered_run:{run_id}")
    for name in ALLOWED_ROOT_METADATA:
        path = root / name
        if path.exists() and (not path.is_file() or path.is_symlink()):
            failures.append(f"root_inventory_metadata_type:{name}")
    preflight = root / "preflight"
    if preflight.exists():
        if not preflight.is_dir() or preflight.is_symlink():
            failures.append("root_inventory_preflight_type")
        else:
            for entry in preflight.iterdir():
                if (
                    entry.name not in ALLOWED_PREFLIGHT_METADATA
                    or not entry.is_file()
                    or entry.is_symlink()
                ):
                    failures.append(
                        f"root_inventory_preflight_metadata:{entry.name}"
                    )
                else:
                    bundle_paths.append(entry)

    expected_manifests = {
        (root / run_id / "p4_g0c_run_manifest.json").resolve()
        for run_id in registered_run_ids
    }
    expected_csvs = {
        (root / run_id / "p4_decisions.csv").resolve()
        for run_id in registered_run_ids
    }
    for path in root.rglob("*"):
        if path.is_dir() and (
            path.name.startswith("p4-g0c-")
            or "retry" in path.name.lower()
        ):
            if (
                path.parent != root
                or path.name not in registered_run_ids
            ):
                failures.append(
                    f"root_inventory_run_like_directory:{path.relative_to(root)}"
                )
        if "manifest" in path.name.lower() and not path.is_dir():
            if (
                path.name != "p4_g0c_run_manifest.json"
                or path.resolve() not in expected_manifests
                or path.is_symlink()
            ):
                failures.append(
                    f"root_inventory_manifest:{path.relative_to(root)}"
                )
        if path.suffix.lower() == ".csv" and not path.is_dir():
            if (
                path.name != "p4_decisions.csv"
                or path.resolve() not in expected_csvs
                or path.is_symlink()
            ):
                failures.append(
                    f"root_inventory_decision_csv:{path.relative_to(root)}"
                )
    return failures, bundle_paths


def _runner_state_failures(
    bundle: ProtocolBundle,
    root: Path,
    plan: list[dict[str, Any]],
) -> tuple[list[str], dict[str, Any], Path]:
    path = root / RUNNER_STATE_FILENAME
    try:
        state = json.loads(path.read_text())
        if not isinstance(state, dict):
            raise ValueError("runner state root is not an object")
    except OSError as exc:
        return [f"runner_state_missing:{exc}"], {}, path
    except (json.JSONDecodeError, ValueError) as exc:
        return [f"runner_state_malformed:{exc}"], {}, path

    expected_ids = [record["run_id"] for record in plan]
    expected_attempts = [
        {"attempt_index": index, "run_id": run_id, "state": "COMPLETE"}
        for index, run_id in enumerate(expected_ids, start=1)
    ]
    failures = []
    expected_scalars = {
        "schema_version": RUNNER_STATE_SCHEMA,
        "runner_state": "COMPLETE",
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "completed_run_count": 15,
        "launch_invocations": 15,
        "launch_started": True,
        "retries": 0,
        "failure_reason": "",
        "failed_run_id": "",
    }
    for key, value in expected_scalars.items():
        if state.get(key) != value:
            if key.endswith("sha256"):
                category = "runner_state_hash"
            elif key == "runner_state":
                category = "runner_state_state"
            else:
                category = f"runner_state_{key}"
            failures.append(category)
    if state.get("registered_run_ids") != expected_ids:
        failures.append("runner_state_registered_ids")
    attempted = state.get("attempted_run_ids")
    if attempted != expected_ids:
        typed_attempted = (
            isinstance(attempted, list)
            and all(isinstance(item, str) for item in attempted)
        )
        if typed_attempted and len(set(attempted)) != len(attempted):
            failures.append("runner_state_duplicate_attempt")
        elif typed_attempted and set(attempted) == set(expected_ids):
            failures.append("runner_state_attempt_order")
        else:
            failures.append("runner_state_attempted_ids")
    completed = state.get("completed_run_ids")
    if completed != expected_ids:
        failures.append("runner_state_completed_ids")
    if state.get("attempts") != expected_attempts:
        failures.append("runner_state_attempt_ledger")
    if state.get("runs") != plan:
        failures.append("runner_state_plan")
    return failures, state, path


def _manifest_failures(
    bundle: ProtocolBundle,
    record: dict[str, Any],
    manifest: dict[str, Any],
    csv_path: Path,
    expected_config_hash: str,
) -> list[str]:
    run_id = record["run_id"]
    failures = []
    expected = {
        "schema_version": RUN_MANIFEST_SCHEMA,
        "gate": "G0C",
        "run_id": run_id,
        "seed": record["seed"],
        "repetition": record["repetition"],
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "csv_path": str(csv_path.resolve()),
        "required_process_set": REQUIRED_PROCESSES,
        "required_processes_ok": True,
        "runner_state": "COMPLETE",
        "launch_exit_code": 0,
        "retry_count": 0,
        "record_bag": False,
        "start_rviz": False,
        "selection_applied": False,
        "immutable_run_id": True,
        "overwrite_allowed": False,
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            category = "hash_mismatch" if key.endswith("sha256") else "manifest_truth"
            failures.append(f"{category}:{run_id}:{key}")
    effective = manifest.get("effective_values")
    if effective != bundle.protocol["effective_values"]:
        failures.append(f"config_mismatch:{run_id}:effective_values")
        if isinstance(effective, dict):
            if effective.get("p4.metrics_only") is not True:
                failures.append(f"metrics_only_false:{run_id}")
            if effective.get("selection_applied") is not False:
                failures.append(
                    f"selection_applied:{run_id}:manifest_effective"
                )
    config_hash = str(manifest.get("effective_config_sha256", ""))
    if len(config_hash) != 64 or any(c not in "0123456789abcdef" for c in config_hash):
        failures.append(f"hash_mismatch:{run_id}:effective_config_sha256")
    elif config_hash != expected_config_hash:
        failures.append(f"hash_mismatch:{run_id}:effective_config_sha256")
    return failures


def _row_metrics(
    row: dict[str, str], source_index: int, noise_floor: float,
    path_ratio_tolerance: float,
) -> tuple[dict[str, float] | None, list[str], tuple[Any, ...] | None]:
    failures = []
    prefix = f"row_{source_index}"
    try:
        typed = parse_decision_row(row, path_ratio_tolerance)
    except DecisionSchemaError as exc:
        failures.append(f"{exc.code}:{prefix}:{exc}")
        return None, failures, None
    strings = typed["strings"]
    counts = typed["integers"]
    floats = typed["floats"]
    identity = decision_identity(typed)
    if (
        strings["status"] != "ORIGINAL_SELECTED"
        or strings["reason"] != "METRICS_ONLY"
    ):
        failures.append(f"incomplete_decision:{prefix}")
    if counts["selection_applied"] != 0:
        failures.append(f"selection_applied:{prefix}")
    if strings["selected_hash"] != strings["original_hash"]:
        failures.append(f"identity_mismatch:{prefix}")
    if (
        counts["original_sample_count"] != 200
        or counts["original_valid_count"] != 200
        or counts["risk_sample_count"] != 200
        or counts["risk_valid_count"] != 200
        or any(counts[key] != 0 for key in (
            "original_unknown_count", "original_stale_count",
            "original_non_finite_count", "risk_unknown_count",
            "risk_stale_count", "risk_non_finite_count",
        ))
    ):
        failures.append(f"coverage:{prefix}")
    mean_improvement = floats["original_mean"] - floats["risk_mean"]
    max_improvement = floats["original_max"] - floats["risk_max"]
    if mean_improvement <= noise_floor or max_improvement <= noise_floor:
        failures.append(f"noise_floor:{prefix}")
    if floats["path_length_ratio"] > 1.3:
        failures.append(f"path_ratio:{prefix}")
    if (
        floats["original_search_latency_ms"] >= 200.0
        or floats["risk_search_latency_ms"] >= 200.0
        or floats["total_search_latency_ms"] >= 400.0
    ):
        failures.append(f"timeout:{prefix}")
    if failures:
        return None, failures, identity
    return {
        "mean_improvement": mean_improvement,
        "max_improvement": max_improvement,
        "path_ratio": floats["path_length_ratio"],
        "total_search_s": milliseconds_to_seconds(
            floats["total_search_latency_ms"]
        ),
    }, [], identity


def _threshold_draft(
    bundle: ProtocolBundle,
    raw_bundle_sha256: str,
    metrics: list[tuple[int, dict[str, float]]],
) -> dict[str, Any]:
    def q(name: str, p: float) -> dict[str, Any]:
        return quantile_type7([(item[name], index) for index, item in metrics], p)

    mean_q = q("mean_improvement", 0.10)
    max_q = q("max_improvement", 0.10)
    ratio_q = q("path_ratio", 0.95)
    search_q = q("total_search_s", 0.95)
    return {
        "schema_version": DRAFT_SCHEMA,
        "state": "DRAFT_UNCALIBRATED",
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "calibration_bundle_sha256": raw_bundle_sha256,
        "complete_decision_count": len(metrics),
        "gates": {
            "mean_improvement_min": {
                "value": mean_q["value"], "unit": "risk_cost",
                "formula": "Q10(original_mean-risk_mean)",
                "quantile_source": mean_q,
            },
            "max_improvement_min": {
                "value": max_q["value"], "unit": "risk_cost",
                "formula": "Q10(original_max-risk_max)",
                "quantile_source": max_q,
            },
            "path_ratio_max": {
                "value": min(1.30, ratio_q["value"] + 0.02),
                "unit": "dimensionless",
                "formula": "min(1.30,Q95(path_ratio)+0.02)",
                "quantile_source": ratio_q,
            },
            "total_search_timeout_s": {
                "value": min(
                    0.40,
                    search_q["value"] + max(0.01, 0.20 * search_q["value"]),
                ),
                "unit": "s",
                "formula": "min(0.40 s,Q95(total_search_s)+max(0.01 s,0.20*Q95(total_search_s)))",
                "quantile_source": search_q,
            },
        },
        "registry_updated": False,
        "application_enabled": False,
    }


def analyze(bundle: ProtocolBundle, runs_root: Path) -> dict[str, Any]:
    root = Path(runs_root).expanduser().resolve()
    plan = expand_run_plan(bundle.protocol, root)
    registered_ids = [record["run_id"] for record in plan]
    failures: list[str] = []
    inventory_failures, bundle_paths = _root_inventory_failures(
        root, registered_ids
    )
    failures.extend(inventory_failures)
    runner_failures, runner_state, runner_state_path = _runner_state_failures(
        bundle, root, plan
    )
    failures.extend(runner_failures)
    if runner_state_path.is_file():
        bundle_paths.append(runner_state_path)
    metrics: list[tuple[int, dict[str, float]]] = []
    denominator = 0
    seen_run_ids: set[str] = set()
    expected_config_hash = effective_config_sha256(
        bundle.protocol["effective_values"]
    )
    noise_floor = float(bundle.protocol["numerical_noise_floor"]["value"])
    ratio_tolerance = float(
        bundle.protocol["path_ratio_consistency"]["absolute_tolerance"]
    )
    for record in plan:
        run_dir = Path(record["run_dir"])
        manifest_path = run_dir / "p4_g0c_run_manifest.json"
        csv_path = run_dir / "p4_decisions.csv"
        if not run_dir.is_dir():
            failures.append(f"missing_run:{record['run_id']}")
            continue
        if manifest_path.is_file():
            bundle_paths.append(manifest_path)
        if csv_path.is_file():
            bundle_paths.append(csv_path)
        try:
            manifest = json.loads(manifest_path.read_text())
            if not isinstance(manifest, dict):
                raise ValueError("manifest root is not an object")
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            failures.append(f"malformed_manifest:{record['run_id']}:{exc}")
            manifest = {}
        manifest_run_id = str(manifest.get("run_id", ""))
        if manifest_run_id in seen_run_ids:
            failures.append(f"duplicate_run:{manifest_run_id}")
        elif manifest_run_id:
            seen_run_ids.add(manifest_run_id)
        manifest_errors = _manifest_failures(
            bundle, record, manifest, csv_path, expected_config_hash
        )
        failures.extend(manifest_errors)
        try:
            with csv_path.open(newline="") as stream:
                reader = csv.DictReader(stream, strict=True)
                rows = list(reader)
        except (OSError, csv.Error) as exc:
            failures.append(f"malformed_csv:{record['run_id']}:{exc}")
            continue
        try:
            validate_decision_header(reader.fieldnames)
        except DecisionSchemaError as exc:
            failures.append(f"{exc.code}:{record['run_id']}:{exc}")
            denominator += len(rows)
            continue
        if not rows:
            failures.append(f"empty_run_csv:{record['run_id']}")
            continue
        run_identities: set[tuple[Any, ...]] = set()
        for row in rows:
            source_index = denominator
            denominator += 1
            row_metric, row_failures, identity = _row_metrics(
                row, source_index, noise_floor, ratio_tolerance
            )
            failures.extend(row_failures)
            if identity is not None:
                if identity in run_identities:
                    failures.append(
                        f"duplicate_decision_identity:"
                        f"{record['run_id']}:row_{source_index}"
                    )
                    row_metric = None
                else:
                    run_identities.add(identity)
            if row_metric is not None:
                metrics.append((source_index, row_metric))
    if len(metrics) < int(bundle.protocol["minimum_complete_decisions"]):
        failures.append(
            f"minimum_complete_decisions:{len(metrics)}<"
            f"{bundle.protocol['minimum_complete_decisions']}"
        )
    raw_hash = _raw_bundle_hash(root, bundle_paths) if bundle_paths else ""
    result = {
        "schema_version": ANALYSIS_SCHEMA,
        "analysis_status": "REJECTED" if failures else "DRAFT_ELIGIBLE",
        "protocol_sha256": bundle.protocol_sha256,
        "registry_sha256": bundle.registry_sha256,
        "fixture_sha256": bundle.fixture_sha256,
        "raw_bundle_sha256": raw_hash,
        "registered_run_count": len(plan),
        "registered_run_denominator_count": len(plan),
        "attempted_run_denominator_count": (
            len(runner_state.get("attempted_run_ids", []))
            if isinstance(runner_state.get("attempted_run_ids"), list) else 0
        ),
        "completed_run_denominator_count": (
            len(runner_state.get("completed_run_ids", []))
            if isinstance(runner_state.get("completed_run_ids"), list) else 0
        ),
        "complete_decision_count": len(metrics),
        "denominator_decision_count": denominator,
        "failed_rows_retained_in_denominator": True,
        "failures": failures,
        "registry_updated": False,
        "application_enabled": False,
    }
    if not failures:
        result["threshold_draft"] = _threshold_draft(
            bundle, raw_hash, metrics
        )
    return result


def _parser() -> argparse.ArgumentParser:
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=repo / "config/icra27/p4_g0c_protocol_v1.json")
    parser.add_argument("--registry", type=Path, default=repo / "config/icra27/p4_threshold_registry_v1.json")
    parser.add_argument("--fixture", type=Path, default=repo / "config/icra27/p4_g0c_live_fixture_v1.json")
    parser.add_argument("--runs-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--draft-output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        bundle = load_bundle(args.protocol, args.registry, args.fixture)
        result = analyze(bundle, args.runs_root)
        if args.output:
            if args.output.resolve() == args.registry.resolve():
                raise AnalysisError("analyzer output cannot overwrite registry")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        if args.draft_output and "threshold_draft" in result:
            if args.draft_output.resolve() == args.registry.resolve():
                raise AnalysisError("threshold draft cannot overwrite registry")
            args.draft_output.parent.mkdir(parents=True, exist_ok=True)
            args.draft_output.write_text(
                json.dumps(result["threshold_draft"], indent=2, sort_keys=True) + "\n"
            )
    except (AnalysisError, RuntimeError) as exc:
        print(f"P4_G0C_ANALYZER_FAILED: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["analysis_status"] == "DRAFT_ELIGIBLE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
