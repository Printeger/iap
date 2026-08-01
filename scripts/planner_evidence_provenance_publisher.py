#!/usr/bin/env python3
"""Publish immutable launch provenance into the recorded evidence bag."""

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class EvidenceProvenancePublisher(Node):
    def __init__(self):
        super().__init__("planner_evidence_provenance")
        manifest_path = Path(self.declare_parameter("manifest_path", "").value)
        if not manifest_path.is_file():
            raise RuntimeError(f"manifest_path is unavailable: {manifest_path}")
        manifest = json.loads(manifest_path.read_text())
        payload = {
            "schema_version": manifest.get("artifact_provenance", {}).get("schema_version", ""),
            "run_id": manifest.get("artifact_provenance", {}).get("run_id", ""),
            "manifest_path": str(manifest_path.resolve()),
            "export_dir": manifest.get("export_dir", ""),
            "bag_path": manifest.get("artifact_provenance", {}).get("bag_path", ""),
        }
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(String, "/planning/evidence_provenance", qos)
        self.message = String(data=json.dumps(payload, sort_keys=True))
        self.timer = self.create_timer(0.5, self.publish)
        self.publish()

    def publish(self):
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = EvidenceProvenancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch may already have shut the shared ROS context down before the
        # executor unwinds.  Treat that normal shutdown ordering as success so
        # the provenance publisher cannot make an otherwise complete run look
        # like a failed evidence process.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
