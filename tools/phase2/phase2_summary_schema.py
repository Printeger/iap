#!/usr/bin/env python3
"""Shared Phase 2 summary schema helpers.

The online evaluator owns the stable ``phase2_summary.json`` schema. Offline
analysis may add alignment/validation results, but it must not replace the
online schema with a smaller analysis-only document.
"""

from __future__ import annotations

import copy
import math
from typing import Any


PHASE2_SUMMARY_SCHEMA_VERSION = 1

REQUIRED_ONLINE_SUMMARY_FIELDS = (
    "fallback_count",
    "fallback_rate",
    "fallback_reason_histogram",
    "finite_gnss_prediction_count",
    "integrity_snapshot",
    "current_consistency_raw",
    "current_consistency_anchored",
    "current_consistency",
    "phase_h_lite",
    "stage1_capabilities",
    "pi_cost",
)

ADVISORY_FIM_FIELDS = (
    "enabled",
    "use_lidar_advisory_fim",
    "fusion_mode",
    "query_count",
    "regularized_count",
    "gnss_fim_valid_count",
    "lidar_fim_valid_count",
)

URG_FIELDS = (
    "urg_enabled",
    "urg_active",
    "urg_query_count",
    "urg_front_field_points",
    "urg_backend_field_points",
    "urg_unknown_count",
    "urg_stale_count",
    "urg_mean_update_ms",
    "urg_p95_update_ms",
)


def finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def derive_advisory_fusion_mode(
    summary: dict[str, Any] | None,
    online_rows: list[dict[str, Any]] | None = None,
) -> str:
    """Return the effective advisory fusion mode for a Phase 2 run."""
    summary = summary if isinstance(summary, dict) else {}
    advisory_fim = summary.get("advisory_fim")
    if isinstance(advisory_fim, dict):
        mode = advisory_fim.get("fusion_mode")
        if isinstance(mode, str) and mode.strip():
            return mode.strip()

    config = summary.get("stage1_predictor_config")
    if isinstance(config, dict) and as_bool(config.get("phase2_use_advisory_fim_add")):
        return "fim_add"

    if online_rows:
        row_modes = {
            str(row.get("advisory_fusion_mode", "")).strip()
            for row in online_rows
            if str(row.get("advisory_fusion_mode", "")).strip()
        }
        if row_modes == {"fim_add"}:
            return "fim_add"

    return "legacy"


def missing_required_online_fields(summary: dict[str, Any] | None) -> list[str]:
    summary = summary if isinstance(summary, dict) else {}
    predicted = summary.get("predicted_integrity")
    predicted = predicted if isinstance(predicted, dict) else {}
    missing: list[str] = []
    for field in REQUIRED_ONLINE_SUMMARY_FIELDS:
        if field in summary:
            continue
        if field in {
            "fallback_count",
            "fallback_rate",
            "fallback_reason_histogram",
            "finite_gnss_prediction_count",
        } and field in predicted:
            continue
        missing.append(field)
    return missing


def merge_online_and_offline_summary(
    online_summary: dict[str, Any] | None,
    offline_summary: dict[str, Any],
) -> dict[str, Any]:
    """Augment an online summary with offline analysis without dropping schema.

    Top-level offline scalar values such as ``aligned_sample_count`` replace the
    online placeholders. Nested dictionaries are merged so online-only schema
    fields survive while offline metrics are added.
    """
    merged: dict[str, Any] = (
        copy.deepcopy(online_summary) if isinstance(online_summary, dict) else {}
    )
    for key, value in offline_summary.items():
        if (
            isinstance(value, dict)
            and isinstance(merged.get(key), dict)
            and key not in {"validation"}
        ):
            nested = copy.deepcopy(merged[key])
            nested.update(copy.deepcopy(value))
            merged[key] = nested
        elif key == "warnings" and isinstance(value, list):
            existing = merged.get("warnings") if isinstance(merged.get("warnings"), list) else []
            merged["warnings"] = list(dict.fromkeys([str(item) for item in existing + value]))
        elif key == "errors" and isinstance(value, list):
            existing = merged.get("errors") if isinstance(merged.get("errors"), list) else []
            merged["errors"] = list(dict.fromkeys([str(item) for item in existing + value]))
        else:
            merged[key] = copy.deepcopy(value)
    merged.setdefault("phase2_summary_schema_version", PHASE2_SUMMARY_SCHEMA_VERSION)
    return merged
