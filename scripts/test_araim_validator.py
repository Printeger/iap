#!/usr/bin/env python3

import csv
import json
import math
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

from iap.msg import IntegrityReport


def safe_publisher_count(node, topic, observed_counts):
    """Use the last live graph observation when shutdown invalidates context."""
    try:
        return int(node.count_publishers(topic))
    except Exception:  # rclpy raises RCLError after launch begins context shutdown.
        return int(observed_counts[-1]) if observed_counts else 0


class TestAraimValidator(Node):
    def __init__(self):
        super().__init__("test_araim_validator")

        self.integrity_topic = self.declare_parameter(
            "integrity_topic", "/iap/integrity"
        ).value
        self.duration_s = float(self.declare_parameter("duration_s", 85.0).value)
        self.min_messages = int(self.declare_parameter("min_messages", 10).value)
        self.csv_path = Path(
            self.declare_parameter(
                "csv_path", "/tmp/test_araim_integrity_validation.csv"
            ).value
        )
        self.summary_path = Path(
            self.declare_parameter(
                "summary_path", "/tmp/test_araim_validation_summary.json"
            ).value
        )
        self.schema_version = str(
            self.declare_parameter("schema_version", "").value
        ).strip()
        self.run_id = str(self.declare_parameter("run_id", "").value).strip()
        self.manifest_path = str(
            self.declare_parameter("manifest_path", "").value
        ).strip()
        self.required_fusion_mode = self.declare_parameter(
            "required_fusion_mode", "max_pl"
        ).value
        self.require_gnss_valid = bool(
            self.declare_parameter("require_gnss_valid", True).value
        )
        self.require_lidar_valid = bool(
            self.declare_parameter("require_lidar_valid", True).value
        )
        self.require_fallback_valid = bool(
            self.declare_parameter("require_fallback_valid", True).value
        )
        self.required_final_source = str(
            self.declare_parameter("required_final_source", "").value
        ).strip()
        allowed_sources_csv = str(
            self.declare_parameter(
                "allowed_final_sources_csv", "GNSS,LIDAR,FALLBACK,CONSERVATIVE"
            ).value
        )

        self.allowed_sources = {
            item.strip()
            for item in allowed_sources_csv.split(",")
            if item.strip()
        }
        self.count = 0
        self.gnss_valid_seen = False
        self.lidar_valid_seen = False
        self.fallback_valid_seen = False
        self.bad_fusion_modes = set()
        self.nonfinite_fields = set()
        self.bad_final_sources = set()
        self.bad_required_final_source = set()
        self.publisher_counts = []
        self.done = False
        self.result_code = 1

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "schema_version",
                "run_id",
                "manifest_path",
                "stamp",
                "integrity_state",
                "hpl",
                "vpl",
                "gnss_valid",
                "gnss_hpl",
                "gnss_vpl",
                "lidar_valid",
                "lidar_hpl",
                "lidar_vpl",
                "fallback_valid",
                "fallback_hpl",
                "fallback_vpl",
                "fusion_mode",
                "final_hpl_source",
                "final_vpl_source",
                "failure_reason",
            ]
        )

        self.sub = self.create_subscription(
            IntegrityReport, self.integrity_topic, self.on_integrity, 50
        )
        self.status_timer = self.create_timer(1.0, self.on_status_timer)
        self.final_timer = self.create_timer(self.duration_s, self.finalize)

        self.get_logger().info(
            f"validating {self.integrity_topic}; CSV={self.csv_path}; "
            f"summary={self.summary_path}; duration_s={self.duration_s:.1f}"
        )

    def on_integrity(self, msg: IntegrityReport):
        self.count += 1
        self.gnss_valid_seen = self.gnss_valid_seen or bool(msg.gnss_valid)
        self.lidar_valid_seen = self.lidar_valid_seen or bool(msg.lidar_valid)
        self.fallback_valid_seen = self.fallback_valid_seen or bool(msg.fallback_valid)

        if msg.fusion_mode != self.required_fusion_mode:
            self.bad_fusion_modes.add(msg.fusion_mode)

        finite_fields = {
            "hpl": msg.hpl,
            "vpl": msg.vpl,
            "gnss_hpl": msg.gnss_hpl,
            "lidar_hpl": msg.lidar_hpl,
            "fallback_hpl": msg.fallback_hpl,
        }
        for name, value in finite_fields.items():
            if not math.isfinite(float(value)):
                self.nonfinite_fields.add(name)

        for name, value in (
            ("final_hpl_source", msg.final_hpl_source),
            ("final_vpl_source", msg.final_vpl_source),
        ):
            if value not in self.allowed_sources:
                self.bad_final_sources.add(f"{name}={value}")
            if self.required_final_source and value != self.required_final_source:
                self.bad_required_final_source.add(f"{name}={value}")

        stamp = msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec
        self.csv_writer.writerow(
            [
                self.schema_version,
                self.run_id,
                self.manifest_path,
                f"{stamp:.9f}",
                int(msg.integrity_state),
                f"{msg.hpl:.9g}",
                f"{msg.vpl:.9g}",
                int(bool(msg.gnss_valid)),
                f"{msg.gnss_hpl:.9g}",
                f"{msg.gnss_vpl:.9g}",
                int(bool(msg.lidar_valid)),
                f"{msg.lidar_hpl:.9g}",
                f"{msg.lidar_vpl:.9g}",
                int(bool(msg.fallback_valid)),
                f"{msg.fallback_hpl:.9g}",
                f"{msg.fallback_vpl:.9g}",
                msg.fusion_mode,
                msg.final_hpl_source,
                msg.final_vpl_source,
                msg.failure_reason,
            ]
        )
        if self.count % 10 == 0:
            self.csv_file.flush()

    def on_status_timer(self):
        publishers = self.count_publishers(self.integrity_topic)
        self.publisher_counts.append(int(publishers))
        self.get_logger().info(
            "status: msgs=%d publishers=%d gnss=%s lidar=%s fallback=%s"
            % (
                self.count,
                publishers,
                self.gnss_valid_seen,
                self.lidar_valid_seen,
                self.fallback_valid_seen,
            )
        )

    def finalize(self):
        if self.done:
            return
        self.done = True
        publishers_now = safe_publisher_count(
            self, self.integrity_topic, self.publisher_counts)
        self.publisher_counts.append(int(publishers_now))

        failures = []
        if self.count < self.min_messages:
            failures.append(
                f"received {self.count} integrity messages, expected >= {self.min_messages}"
            )
        nonzero_publisher_counts = [
            count for count in self.publisher_counts if count > 0
        ]
        if not any(count == 1 for count in nonzero_publisher_counts):
            failures.append(
                f"{self.integrity_topic} publisher count was never observed as 1; counts={self.publisher_counts}"
            )
        if any(count > 1 for count in nonzero_publisher_counts):
            failures.append(
                f"{self.integrity_topic} publisher counts were {self.publisher_counts}, expected no count > 1"
            )
        if self.require_gnss_valid and not self.gnss_valid_seen:
            failures.append("gnss_valid was never true")
        if self.require_lidar_valid and not self.lidar_valid_seen:
            failures.append("lidar_valid was never true")
        if self.require_fallback_valid and not self.fallback_valid_seen:
            failures.append("fallback_valid was never true")
        if self.bad_fusion_modes:
            failures.append(
                "unexpected fusion_mode values: "
                + ",".join(sorted(str(v) for v in self.bad_fusion_modes))
            )
        if self.nonfinite_fields:
            failures.append(
                "non-finite integrity fields: "
                + ",".join(sorted(self.nonfinite_fields))
            )
        if self.bad_final_sources:
            failures.append(
                "unexpected final source values: "
                + ",".join(sorted(self.bad_final_sources))
            )
        if self.bad_required_final_source:
            failures.append(
                f"final source differed from required {self.required_final_source}: "
                + ",".join(sorted(self.bad_required_final_source))
            )

        summary = {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "manifest_path": self.manifest_path,
            "passed": not failures,
            "failures": failures,
            "message_count": self.count,
            "publisher_counts": self.publisher_counts,
            "gnss_valid_seen": self.gnss_valid_seen,
            "lidar_valid_seen": self.lidar_valid_seen,
            "fallback_valid_seen": self.fallback_valid_seen,
            "required_fusion_mode": self.required_fusion_mode,
            "require_gnss_valid": self.require_gnss_valid,
            "require_lidar_valid": self.require_lidar_valid,
            "require_fallback_valid": self.require_fallback_valid,
            "required_final_source": self.required_final_source,
            "csv_path": str(self.csv_path),
        }
        self.csv_file.flush()
        self.csv_file.close()
        self.summary_path.write_text(json.dumps(summary, indent=2) + "\n")

        if failures:
            self.get_logger().error("ARAIM validation FAILED")
            for failure in failures:
                self.get_logger().error(f"  - {failure}")
            self.result_code = 2
        else:
            self.get_logger().info("ARAIM validation PASSED")
            self.result_code = 0


def main():
    rclpy.init()
    node = TestAraimValidator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if not node.done:
            node.finalize()
    code = node.result_code
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
