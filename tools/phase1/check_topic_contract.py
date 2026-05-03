#!/usr/bin/env python3
import argparse
import sys
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


@dataclass
class TopicSpec:
    name: str
    type_name: str
    min_hz: float
    required: bool = True
    expected_frame: str = ""
    expected_child_frame: str = ""
    allow_quiet_window: bool = False


DEFAULT_TOPICS = [
    TopicSpec("/sim/drone_0/truth_odom", "nav_msgs/msg/Odometry", 20.0, True, "map"),
    TopicSpec("/drone_0_visual_slam/odom", "nav_msgs/msg/Odometry", 5.0, True, "map", "imu"),
    TopicSpec("/sim/drone_0/imu_iap", "sensor_msgs/msg/Imu", 20.0, True, "imu"),
    TopicSpec("/sim/drone_0/lidar", "sensor_msgs/msg/PointCloud2", 2.0, True, "map"),
    TopicSpec("/sim/drone_0/lidar_body", "sensor_msgs/msg/PointCloud2", 2.0, True, "lidar"),
    TopicSpec("/map_generator/global_cloud", "sensor_msgs/msg/PointCloud2", 0.1, True, "map"),
    TopicSpec("/drone_0_planning/bspline", "traj_utils/msg/Bspline", 0.01, True, allow_quiet_window=True),
    TopicSpec("/drone_0_planning/pos_cmd", "quadrotor_msgs/msg/PositionCommand", 20.0, True, "map"),
    TopicSpec("/demo9/desired/odom", "nav_msgs/msg/Odometry", 20.0, True, "map"),
    TopicSpec("/demo9/drone/path", "nav_msgs/msg/Path", 0.1, True, "map"),
    TopicSpec("/demo9/truth/path", "nav_msgs/msg/Path", 0.1, True, "map"),
    TopicSpec("/demo9/desired/path", "nav_msgs/msg/Path", 0.1, True, "map"),
    TopicSpec("/ublox_driver/range_meas", "gnss_comm/msg/GnssMeasMsg", 0.1, False),
    TopicSpec("/ublox_driver/ephem", "gnss_comm/msg/GnssEphemMsg", 0.1, False),
    TopicSpec("/ublox_driver/glo_ephem", "gnss_comm/msg/GnssGloEphemMsg", 0.1, False),
    TopicSpec("/iap/integrity", "iap/msg/IntegrityReport", 0.1, False),
]


class ContractChecker(Node):
    def __init__(self, specs):
        super().__init__("phase1_topic_contract_checker")
        self.specs = specs
        self.samples = {
            spec.name: {"count": 0, "first": None, "last": None, "frame": "", "child_frame": ""}
            for spec in specs
        }
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        for spec in specs:
            msg_type = get_message(spec.type_name)
            self.create_subscription(
                msg_type,
                spec.name,
                lambda msg, topic=spec.name: self._on_msg(topic, msg),
                qos,
            )

    def _stamp(self, msg):
        header = getattr(msg, "header", None)
        if header is None:
            return self.get_clock().now().nanoseconds * 1.0e-9
        stamp = getattr(header, "stamp", None)
        if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
            return self.get_clock().now().nanoseconds * 1.0e-9
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _on_msg(self, topic, msg):
        sample = self.samples[topic]
        stamp = self._stamp(msg)
        sample["count"] += 1
        if sample["first"] is None:
            sample["first"] = stamp
        sample["last"] = stamp
        header = getattr(msg, "header", None)
        if header is not None:
            sample["frame"] = getattr(header, "frame_id", "") or sample["frame"]
        sample["child_frame"] = getattr(msg, "child_frame_id", "") or sample["child_frame"]

    def report(self, duration):
        failures = []
        rows = []
        graph_types = {
            name: set(type_names) for name, type_names in self.get_topic_names_and_types()
        }
        for spec in self.specs:
            sample = self.samples[spec.name]
            count = sample["count"]
            advertised = spec.type_name in graph_types.get(spec.name, set())
            hz = 0.0
            if count > 1 and sample["first"] is not None and sample["last"] and sample["last"] > sample["first"]:
                hz = float(count - 1) / float(sample["last"] - sample["first"])
            elif count > 0 and duration > 0:
                hz = float(count) / float(duration)

            status = "OK"
            if spec.name in graph_types and not advertised:
                status = "FAIL"
                observed = ", ".join(sorted(graph_types[spec.name])) or "-"
                failures.append(f"{spec.name}: type {observed} != {spec.type_name}")
            if spec.required and count == 0 and not (spec.allow_quiet_window and advertised):
                status = "FAIL"
                failures.append(f"{spec.name}: no messages")
            elif spec.required and count > 0 and hz < spec.min_hz:
                status = "FAIL"
                failures.append(f"{spec.name}: hz {hz:.2f} < {spec.min_hz:.2f}")
            if spec.expected_frame and sample["frame"] and sample["frame"] != spec.expected_frame:
                status = "FAIL"
                failures.append(f"{spec.name}: frame '{sample['frame']}' != '{spec.expected_frame}'")
            if spec.expected_child_frame and sample["child_frame"] and sample["child_frame"] != spec.expected_child_frame:
                status = "FAIL"
                failures.append(
                    f"{spec.name}: child_frame '{sample['child_frame']}' != '{spec.expected_child_frame}'"
                )
            rows.append((status, spec.name, spec.type_name, count, hz, sample["frame"], sample["child_frame"]))
        return rows, failures


def parse_args():
    parser = argparse.ArgumentParser(description="Check the Phase 1 ROS topic contract.")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--use-gnss", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--use-araim", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--expect-glo", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main():
    args = parse_args()
    specs = []
    for spec in DEFAULT_TOPICS:
        required = spec.required
        if spec.name.startswith("/ublox_driver/"):
            required = args.use_gnss
        if spec.name == "/ublox_driver/glo_ephem":
            required = args.use_gnss and args.expect_glo
        if spec.name == "/iap/integrity":
            required = args.use_araim
        specs.append(
            TopicSpec(
                spec.name,
                spec.type_name,
                spec.min_hz,
                required,
                spec.expected_frame,
                spec.expected_child_frame,
                spec.allow_quiet_window,
            )
        )

    rclpy.init()
    node = ContractChecker(specs)
    end_time = node.get_clock().now().nanoseconds * 1.0e-9 + args.duration
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds * 1.0e-9 < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        rows, failures = node.report(args.duration)
        node.destroy_node()
        rclpy.shutdown()

    print("status topic type count hz frame child_frame")
    for status, name, type_name, count, hz, frame, child in rows:
        print(f"{status} {name} {type_name} {count} {hz:.2f} {frame or '-'} {child or '-'}")
    if failures:
        print("\nFailures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
