#!/usr/bin/env python3
"""Capture committed-final/P5/publication identity for one ICRA-075 row."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from traj_utils.msg import Bspline


REPOSITORY = Path(__file__).resolve().parents[2]
RESULTS_ROOT = (REPOSITORY / "results/icra27/icra075").resolve()


def _contained(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def validate_paths(output: Path, ready: Path) -> tuple[Path, Path]:
    output = (output if output.is_absolute() else REPOSITORY / output).resolve()
    ready = (ready if ready.is_absolute() else REPOSITORY / ready).resolve()
    if not _contained(output, RESULTS_ROOT) or ready.parent != output.parent:
        raise SystemExit("ICRA-075 capture paths must share a repository-local row root")
    if not output.parent.is_dir() or output.exists() or ready.exists():
        raise SystemExit("ICRA-075 capture requires a new output in an existing row root")
    return output, ready


class Capture(Node):
    def __init__(self, output: Path, ready: Path, duration_s: float,
                 scene: str, descriptor_sha256: str):
        super().__init__("icra075_exploratory_capture")
        self._stream = output.open("x", buffering=1)
        self._binding = {"scene": scene, "descriptor_sha256": descriptor_sha256}
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/planning/risk_grid_health",
                                 lambda message: self._json("p0_health", message.data), qos)
        self.create_subscription(String, "/planning/integrity_gate_status",
                                 lambda message: self._json("p5_status", message.data), qos)
        self.create_subscription(Bspline, "/drone_0_planning/bspline",
                                 self._bspline, qos)
        ready.write_text(json.dumps({
            "schema_version": "icra075_capture_readiness_v1",
            "ready": True,
            "pid": os.getpid(),
            **self._binding,
            "topics": ["/planning/risk_grid_health",
                       "/planning/integrity_gate_status",
                       "/drone_0_planning/bspline"],
        }, indent=2, sort_keys=True) + "\n")
        self.create_timer(duration_s, rclpy.shutdown)

    def _json(self, kind: str, raw: str):
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            payload = {"parse_error": True, "raw": raw}
        self._write(kind, payload)

    def _bspline(self, message: Bspline):
        start_ns = int(message.start_time.sec) * 1_000_000_000 + int(message.start_time.nanosec)
        control = [[float(point.x), float(point.y), float(point.z)]
                   for point in message.pos_pts]
        knots = [float(value) for value in message.knots]
        identity_input = {
            "trajectory_id": int(message.traj_id),
            "start_time_ns": start_ns,
            "order": int(message.order),
            "position_control_points": control,
            "knots": knots,
        }
        digest = hashlib.sha256(json.dumps(
            identity_input, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self._write("normal_bspline", {
            **identity_input,
            "final_bspline_identity": digest,
            "publication_identity": digest,
            **self._binding,
        })

    def _write(self, kind: str, payload: dict):
        self._stream.write(json.dumps({
            "kind": kind, "receive_steady_s": time.monotonic(),
            **self._binding, "payload": payload,
        }, sort_keys=True) + "\n")

    def destroy_node(self):
        self._stream.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    parser.add_argument("--scene", choices=("PRIMARY", "EXACT_MIRROR", "FLAT_NULL"),
                        required=True)
    parser.add_argument("--descriptor-sha256", required=True)
    args = parser.parse_args()
    output, ready = validate_paths(args.output, args.ready_file)
    rclpy.init()
    node = Capture(output, ready, args.duration_s, args.scene, args.descriptor_sha256)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
