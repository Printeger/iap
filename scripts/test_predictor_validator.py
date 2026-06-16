#!/usr/bin/env python3

import csv
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


class TestPredictorValidator(Node):
    def __init__(self):
        super().__init__("test_predictor_validator")
        self.csv_path = Path(
            self.declare_parameter(
                "csv_path", "/tmp/test_predictor_query_probe.csv"
            ).value
        )
        self.probe_summary_path = Path(
            self.declare_parameter(
                "probe_summary_path", "/tmp/test_predictor_query_probe_summary.json"
            ).value
        )
        self.validation_summary_path = Path(
            self.declare_parameter(
                "validation_summary_path",
                "/tmp/test_predictor_validation_summary.json",
            ).value
        )
        self.duration_s = float(self.declare_parameter("duration_s", 85.0).value)
        self.min_queries = int(self.declare_parameter("min_queries", 10).value)
        self.output_mode = str(
            self.declare_parameter("predictor_output_mode", "fusion").value
        )
        self.required_source = str(
            self.declare_parameter("required_selected_source", "").value
        ).strip()
        self.debug_enabled = _as_bool(
            self.declare_parameter("debug_enabled", False).value
        )
        self.require_debug_logs = _as_bool(
            self.declare_parameter("require_debug_logs", False).value
        )
        debug_file_paths = str(
            self.declare_parameter("debug_file_paths", "").value
        )
        self.debug_file_paths = [
            Path(item.strip())
            for item in debug_file_paths.split(",")
            if item.strip()
        ]
        self.require_selected_valid = _as_bool(
            self.declare_parameter("require_selected_valid", True).value
        )
        self.require_gnss_valid = _as_bool(
            self.declare_parameter("require_gnss_valid", False).value
        )
        self.require_lidar_valid = _as_bool(
            self.declare_parameter("require_lidar_valid", False).value
        )
        self.require_fusion_valid = _as_bool(
            self.declare_parameter("require_fusion_valid", False).value
        )
        self.max_lambda_sum_error = float(
            self.declare_parameter("max_lambda_sum_error", 1.0e-8).value
        )
        self.done = False
        self.result_code = 1
        self.timer = self.create_timer(self.duration_s, self.finalize)
        self.get_logger().info(
            f"validating predictor CSV={self.csv_path}; duration_s={self.duration_s:.1f}"
        )

    def finalize(self):
        if self.done:
            return
        self.done = True
        failures = []
        rows = []
        if not self.csv_path.is_file():
            failures.append(f"missing predictor CSV: {self.csv_path}")
        else:
            with self.csv_path.open(newline="") as f:
                rows = list(csv.DictReader(f))

        if len(rows) < self.min_queries:
            failures.append(
                f"received {len(rows)} predictor queries, expected >= {self.min_queries}"
            )

        selected_valid_seen = False
        gnss_valid_seen = False
        lidar_valid_seen = False
        fusion_valid_seen = False
        bad_sources = set()
        nonfinite_valid_fields = set()
        empty_fallback_reasons = 0
        lambda_sum_errors = []
        source_counts = {}
        fallback_hist = {}

        for row in rows:
            source = row.get("selected_source", "")
            source_counts[source] = source_counts.get(source, 0) + 1
            selected_valid = row.get("selected_valid") == "1"
            selected_fallback = row.get("selected_fallback") == "1"
            selected_valid_seen = selected_valid_seen or selected_valid
            gnss_valid_seen = gnss_valid_seen or row.get("gnss_valid") == "1"
            lidar_valid_seen = lidar_valid_seen or row.get("lidar_valid") == "1"
            fusion_valid_seen = fusion_valid_seen or row.get("fused_valid") == "1"

            reason = row.get("selected_fallback_reason", "")
            if selected_fallback and not reason:
                empty_fallback_reasons += 1
            if reason:
                fallback_hist[reason] = fallback_hist.get(reason, 0) + 1

            if self.required_source and selected_valid and source != self.required_source:
                bad_sources.add(source)

            if selected_valid:
                for field in (
                    "selected_hpl",
                    "selected_vpl",
                    "selected_pl",
                    "fused_lambda_pred_trace",
                    "fused_lambda_pred_min_eig",
                    "fused_lambda_pred_condition",
                ):
                    if not _finite(row.get(field, "nan")):
                        nonfinite_valid_fields.add(field)

            if _finite(row.get("lambda_sum_error", "nan")):
                lambda_sum_errors.append(abs(float(row["lambda_sum_error"])))

        if self.require_selected_valid and not selected_valid_seen:
            failures.append("selected_valid was never true")
        if self.require_gnss_valid and not gnss_valid_seen:
            failures.append("gnss_valid was never true")
        if self.require_lidar_valid and not lidar_valid_seen:
            failures.append("lidar_valid was never true")
        if self.require_fusion_valid and not fusion_valid_seen:
            failures.append("fused_valid was never true")
        if bad_sources:
            failures.append(
                f"selected source differed from required {self.required_source}: "
                + ",".join(sorted(bad_sources))
            )
        if nonfinite_valid_fields:
            failures.append(
                "non-finite fields in valid selected rows: "
                + ",".join(sorted(nonfinite_valid_fields))
            )
        if empty_fallback_reasons:
            failures.append(
                f"{empty_fallback_reasons} fallback rows had empty fallback reason"
            )
        if lambda_sum_errors and max(lambda_sum_errors) > self.max_lambda_sum_error:
            failures.append(
                f"lambda_sum_error max {max(lambda_sum_errors):.3g} exceeded "
                f"{self.max_lambda_sum_error:.3g}"
            )

        probe_summary = {}
        if self.probe_summary_path.is_file():
            try:
                probe_summary = json.loads(self.probe_summary_path.read_text())
            except json.JSONDecodeError as exc:
                failures.append(f"invalid probe summary JSON: {exc}")
        else:
            failures.append(f"missing probe summary: {self.probe_summary_path}")

        debug_file_status = {
            str(path): {
                "exists": path.is_file(),
                "size_bytes": path.stat().st_size if path.is_file() else 0,
            }
            for path in self.debug_file_paths
        }
        if self.require_debug_logs:
            missing_debug = [
                path for path, status in debug_file_status.items()
                if not status["exists"] or status["size_bytes"] <= 0
            ]
            if missing_debug:
                failures.append(
                    "missing or empty required debug files: "
                    + ",".join(sorted(missing_debug))
                )

        summary = {
            "passed": not failures,
            "failures": failures,
            "csv_path": str(self.csv_path),
            "probe_summary_path": str(self.probe_summary_path),
            "query_count": len(rows),
            "output_mode": self.output_mode,
            "required_selected_source": self.required_source,
            "debug_enabled": self.debug_enabled,
            "require_debug_logs": self.require_debug_logs,
            "debug_file_status": debug_file_status,
            "selected_valid_seen": selected_valid_seen,
            "gnss_valid_seen": gnss_valid_seen,
            "lidar_valid_seen": lidar_valid_seen,
            "fusion_valid_seen": fusion_valid_seen,
            "source_counts": source_counts,
            "fallback_reason_histogram": fallback_hist,
            "max_lambda_sum_error": max(lambda_sum_errors)
            if lambda_sum_errors
            else None,
            "probe_summary": probe_summary,
        }
        self.validation_summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.validation_summary_path.write_text(json.dumps(summary, indent=2) + "\n")

        if failures:
            self.get_logger().error("Predictor validation FAILED")
            for failure in failures:
                self.get_logger().error(f"  - {failure}")
            self.result_code = 2
        else:
            self.get_logger().info("Predictor validation PASSED")
            self.result_code = 0

        rclpy.shutdown()


def main():
    rclpy.init()
    node = TestPredictorValidator()
    try:
        rclpy.spin(node)
    finally:
        code = node.result_code
        node.destroy_node()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
