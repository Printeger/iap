#!/usr/bin/env python3
"""Fail-closed offline classification for diagnostic P4 equal-arc traces."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable

CATEGORIES = (
    "ZERO_WEIGHT_INVALID_CORNER",
    "POSITIVE_WEIGHT_OCCUPIED_SKIP",
    "OUT_OF_MAP",
    "TIME_SUPPORT",
    "STALE",
    "PROVIDER_INVALID",
    "OTHER",
)
ARMS = ("original", "risk")
EXPECTED_SAMPLES_PER_ARM = 200
RISK_GRID_SOURCE_OCCUPIED_SKIP = 1 << 31
OCCUPIED_SKIP_BINDING = (
    "occupied", "RAW_OCCUPIED", "raw_cloud",
    RISK_GRID_SOURCE_OCCUPIED_SKIP,
)
IDENTITY_FIELDS = (
    "planning_attempt_id", "collision_segment_id", "request_hash"
)
SAMPLE_FIELDS = (
    "point_x", "point_y", "point_z", "query_time_s", "query_tau_s",
    "sample_valid", "sample_stale", "sample_cost", "top_reason",
    "risk_generation_id", "frame_id",
)
CORNER_IDENTITY_FIELDS = (
    "corner_id", "temporal_layer", "horizon_id", "voxel_x", "voxel_y",
    "voxel_z",
)
TRACE_FIELDS = (
    "schema_version", *IDENTITY_FIELDS, "arm", "sample_index",
    *SAMPLE_FIELDS, "corner_id", "temporal_layer", "horizon_id",
    "horizon_s", "temporal_weight", "voxel_x", "voxel_y", "voxel_z",
    "voxel_position_x", "voxel_position_y", "voxel_position_z",
    "spatial_weight", "source_flags", "corner_cost", "corner_valid",
    "corner_stale", "corner_unknown", "corner_reason", "occupancy_class",
    "occupancy_source",
)


class TraceClassificationError(RuntimeError):
    pass


def _exact_float(row: dict[str, str], key: str) -> float:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise TraceClassificationError(f"invalid_{key}") from exc
    if not math.isfinite(value):
        raise TraceClassificationError(f"non_finite_{key}")
    return value


def _truth(value: object) -> bool:
    if value == "1":
        return True
    if value == "0":
        return False
    raise TraceClassificationError("invalid_boolean")


def _exact_int(row: dict[str, str], key: str) -> int:
    raw = row.get(key)
    try:
        value = int(raw)
    except (TypeError, ValueError) as exc:
        raise TraceClassificationError(f"invalid_{key}") from exc
    if str(value) != raw:
        raise TraceClassificationError(f"non_canonical_{key}")
    return value


def _category(rows: list[dict[str, str]]) -> str | None:
    if _truth(rows[0].get("sample_valid")):
        return None
    reasons = " ".join(
        (row.get("top_reason", "") + " " + row.get("corner_reason", ""))
        .lower() for row in rows
    )
    if "out_of_map" in reasons or "outside_map" in reasons:
        return "OUT_OF_MAP"
    if any(token in reasons for token in ("horizon", "time_support", "query_time")):
        return "TIME_SUPPORT"
    if _truth(rows[0].get("sample_stale")) or "stale" in reasons:
        return "STALE"
    invalid = [row for row in rows if row.get("corner_reason") not in ("", "none")]
    positive = [
        row for row in invalid
        if _exact_float(row, "spatial_weight") *
        _exact_float(row, "temporal_weight") > 0.0
    ]
    if invalid and not positive:
        return "ZERO_WEIGHT_INVALID_CORNER"
    if positive:
        occupied = [
            row for row in positive
            if (
                row.get("corner_reason"), row.get("occupancy_class"),
                row.get("occupancy_source"),
                _exact_int(row, "source_flags"),
            ) == OCCUPIED_SKIP_BINDING
        ]
        if len(occupied) == len(positive):
            return "POSITIVE_WEIGHT_OCCUPIED_SKIP"
        return "PROVIDER_INVALID"
    if any(token in reasons for token in ("provider", "unknown_voxel", "invalid_cost")):
        return "PROVIDER_INVALID"
    return "OTHER"


def _validate_sample_rows(rows: list[dict[str, str]]) -> None:
    reference = rows[0]
    for field in SAMPLE_FIELDS:
        if any(row.get(field) != reference.get(field) for row in rows[1:]):
            raise TraceClassificationError(
                f"sample_field_conflict:{field}"
            )
    corner_identities = [
        tuple(row.get(field, "") for field in CORNER_IDENTITY_FIELDS)
        for row in rows
    ]
    if len(corner_identities) != len(set(corner_identities)):
        raise TraceClassificationError("duplicate_corner_evidence")
    for row in rows:
        for key in (
            "point_x", "point_y", "point_z", "query_time_s",
            "query_tau_s", "sample_cost", "temporal_weight",
            "spatial_weight",
        ):
            _exact_float(row, key)
        for key in ("temporal_weight", "spatial_weight"):
            weight = _exact_float(row, key)
            if weight < 0.0 or weight > 1.0:
                raise TraceClassificationError(f"out_of_range_{key}")
        for key in (
            "risk_generation_id", "corner_id", "temporal_layer",
            "horizon_id", "voxel_x", "voxel_y", "voxel_z", "source_flags",
        ):
            _exact_int(row, key)
        for key in (
            "sample_valid", "sample_stale", "corner_valid", "corner_stale",
            "corner_unknown",
        ):
            _truth(row.get(key))
        for key in (
            "top_reason", "frame_id", "corner_reason", "occupancy_class",
            "occupancy_source",
        ):
            if not row.get(key):
                raise TraceClassificationError(f"missing_{key}")
        optional_corner_values = (
            "horizon_s", "voxel_position_x", "voxel_position_y",
            "voxel_position_z", "corner_cost",
        )
        if row["corner_id"] == "-1":
            if (
                row["temporal_layer"] != "-1"
                or row["horizon_id"] != "-1"
                or any(row[key] != "nan" for key in optional_corner_values)
                or row["corner_reason"] != "not_evaluated"
                or row["occupancy_class"] != "UNAVAILABLE"
                or row["occupancy_source"] != "unavailable"
            ):
                raise TraceClassificationError("invalid_corner_sentinel")
        else:
            for key in optional_corner_values:
                _exact_float(row, key)


def classify_trace_rows(rows: Iterable[dict[str, str]]) -> dict[str, Any]:
    grouped: dict[tuple[str, ...], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if set(row) != set(TRACE_FIELDS):
            raise TraceClassificationError("trace_header_mismatch")
        if row.get("schema_version") != "p4_equal_arc_profile_trace_v1":
            raise TraceClassificationError("schema_mismatch")
        arm = row.get("arm")
        if arm not in ARMS:
            raise TraceClassificationError("arm_mismatch")
        sample_index = _exact_int(row, "sample_index")
        if sample_index < 0 or sample_index >= EXPECTED_SAMPLES_PER_ARM:
            raise TraceClassificationError("sample_index_out_of_range")
        identity = tuple(row.get(field, "") for field in IDENTITY_FIELDS)
        if any(not value for value in identity):
            raise TraceClassificationError("identity_missing")
        grouped[identity + (arm, str(sample_index))].append(row)
    identities = sorted({key[:3] for key in grouped})
    result_rows = []
    totals = Counter()
    for identity in identities:
        arm_counts = {}
        categories = Counter()
        categorized_indices = {
            arm: {name: [] for name in CATEGORIES} for arm in ARMS
        }
        for arm in ARMS:
            samples = {
                int(key[4]): value for key, value in grouped.items()
                if key[:3] == identity and key[3] == arm
            }
            if set(samples) != set(range(EXPECTED_SAMPLES_PER_ARM)):
                raise TraceClassificationError(
                    f"sample_coverage_mismatch:{identity}:{arm}:{len(samples)}"
                )
            arm_counts[arm] = len(samples)
            for sample_index, sample_rows in sorted(samples.items()):
                _validate_sample_rows(sample_rows)
                category = _category(sample_rows)
                if category:
                    categories[category] += 1
                    totals[category] += 1
                    categorized_indices[arm][category].append(sample_index)
        result_rows.append({
            "planning_attempt_id": identity[0],
            "collision_segment_id": identity[1],
            "request_hash": identity[2],
            "sample_counts": arm_counts,
            "invalid_counts": {name: categories[name] for name in CATEGORIES},
            "invalid_sample_indices": categorized_indices,
        })
    if not identities:
        raise TraceClassificationError("trace_empty")
    return {
        "schema_version": "p4_profile_trace_classification_v1",
        "identity_count": len(identities),
        "expected_samples_per_arm": EXPECTED_SAMPLES_PER_ARM,
        "categories": list(CATEGORIES),
        "totals": {name: totals[name] for name in CATEGORIES},
        "rows": result_rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.trace.open(newline="", encoding="utf-8") as stream:
        result = classify_trace_rows(csv.DictReader(stream))
    args.output.write_text(
        json.dumps(result, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
