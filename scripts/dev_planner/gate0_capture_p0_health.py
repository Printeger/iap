#!/usr/bin/env python3
"""Capture only the raw P0 risk-grid health JSON topic for Gate 0B."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import iap.msg
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Gate0P0HealthCapture(Node):
    def __init__(
        self,
        output_path: Path,
        duration_s: float,
        integrity_output_path: Path | None = None,
    ) -> None:
        super().__init__("gate0_p0_health_capture")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open("a", buffering=1)
        self._integrity_stream = (
            integrity_output_path.open("a", buffering=1)
            if integrity_output_path else None
        )
        if integrity_output_path:
            integrity_output_path.parent.mkdir(parents=True, exist_ok=True)
        self._subscription = self.create_subscription(
            String, "/planning/risk_grid_health", self._on_health, 100
        )
        self._integrity_subscription = self.create_subscription(
            iap.msg.IntegrityReport, "/iap/integrity", self._on_integrity, 100
        )
        self._timer = self.create_timer(duration_s, self._finish)

    def _on_health(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            payload = {"capture_parse_error": True, "raw": message.data}
        payload["capture_receive_steady_s"] = time.monotonic()
        self._stream.write(json.dumps(payload, sort_keys=True) + "\n")

    def _on_integrity(self, message: iap.msg.IntegrityReport) -> None:
        if not self._integrity_stream:
            return
        stamp_s = float(message.header.stamp.sec) + 1e-9 * float(
            message.header.stamp.nanosec
        )
        valid = all(
            map(math.isfinite, (message.hpl, message.vpl, message.hal,
                                message.val, message.im))
        )
        payload = {
            "capture_receive_steady_s": time.monotonic(),
            "stamp_s": stamp_s,
            "valid": valid,
            "hpl": message.hpl,
            "vpl": message.vpl,
            "hal": message.hal,
            "val": message.val,
            "im": message.im,
            "n_sv_used": message.n_sv_used,
            "integrity_state": message.integrity_state,
        }
        self._integrity_stream.write(json.dumps(payload, sort_keys=True) + "\n")

    def _finish(self) -> None:
        self._stream.flush()
        rclpy.shutdown()

    def destroy_node(self) -> bool:
        self._stream.close()
        if self._integrity_stream:
            self._integrity_stream.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--integrity-output", type=Path)
    parser.add_argument("--duration-s", type=float, default=65.0)
    args = parser.parse_args()
    rclpy.init()
    node = Gate0P0HealthCapture(
        args.output.resolve(),
        args.duration_s,
        args.integrity_output.resolve() if args.integrity_output else None,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
