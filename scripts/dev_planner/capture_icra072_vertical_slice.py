#!/usr/bin/env python3
"""Capture the minimal live lineage required by the ICRA-072 dev smoke."""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from traj_utils.msg import Bspline


REPOSITORY = Path(__file__).resolve().parents[2]
DEV_RUNS_ROOT = (REPOSITORY / "results/icra27/dev_runs/layer1").resolve()
RUN_ID_PATTERN = re.compile(r"run-[0-9]{3,}")


def validate_capture_paths(output: Path, ready: Path) -> tuple[Path, Path]:
    resolved_output = (output if output.is_absolute()
                       else REPOSITORY / output).resolve()
    resolved_ready = (ready if ready.is_absolute()
                      else REPOSITORY / ready).resolve()
    run_root = resolved_output.parent
    if (run_root.parent != DEV_RUNS_ROOT or
            RUN_ID_PATTERN.fullmatch(run_root.name) is None or
            not run_root.is_dir()):
        raise SystemExit(
            f"capture output must be inside an existing {DEV_RUNS_ROOT}/"
            "run-[0-9]{3,}")
    if resolved_ready.parent != run_root:
        raise SystemExit("capture readiness file must share the run root")
    return resolved_output, resolved_ready


class VerticalSliceCapture(Node):
    def __init__(self, output: Path, ready: Path, duration_s: float) -> None:
        super().__init__("icra072_vertical_slice_capture")
        output.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output.open("x", buffering=1)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, "/planning/risk_grid_health",
            lambda message: self._record_json("p0_health", message.data), qos)
        self.create_subscription(
            String, "/planning/integrity_gate_status",
            lambda message: self._record_json("p5_status", message.data), qos)
        self.create_subscription(
            Bspline, "/drone_0_planning/bspline", self._record_bspline, qos)
        ready.parent.mkdir(parents=True, exist_ok=True)
        ready.write_text(json.dumps({
            "schema_version": "icra072_capture_readiness_v1",
            "ready": True,
            "pid": os.getpid(),
            "topics": [
                "/planning/risk_grid_health",
                "/planning/integrity_gate_status",
                "/drone_0_planning/bspline",
            ],
        }, indent=2, sort_keys=True) + "\n")
        self.create_timer(duration_s, rclpy.shutdown)

    def _record_json(self, kind: str, raw: str) -> None:
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            payload = {"parse_error": True, "raw": raw}
        self._write(kind, payload)

    def _record_bspline(self, message: Bspline) -> None:
        self._write("normal_bspline", {
            "trajectory_id": int(message.traj_id),
            "start_time_s": (
                float(message.start_time.sec)
                + 1e-9 * float(message.start_time.nanosec)
            ),
            "order": int(message.order),
            "position_control_point_count": len(message.pos_pts),
            "knot_count": len(message.knots),
        })

    def _write(self, kind: str, payload: dict) -> None:
        self._stream.write(json.dumps({
            "kind": kind,
            "receive_steady_s": time.monotonic(),
            "payload": payload,
        }, sort_keys=True) + "\n")

    def destroy_node(self) -> bool:
        self._stream.close()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    args = parser.parse_args()
    output, ready_file = validate_capture_paths(args.output, args.ready_file)
    if output.exists() or ready_file.exists():
        raise SystemExit("capture output already exists")
    rclpy.init()
    node = VerticalSliceCapture(output, ready_file, args.duration_s)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
