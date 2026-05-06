#!/usr/bin/env python3
import argparse
import csv
import math
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def quat_to_rp(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


class Demo6PhaseRecorder(Node):
    def __init__(self, args):
        super().__init__("demo6_phase_z_recorder")
        self.args = args
        self.desired = None
        self.truth = None
        self.iap = None
        self.rows = []
        self.started = False
        self.unwrapped_prev = None
        self.phase_offset = None
        self.start_wall = time.monotonic()
        self.last_sample = 0.0

        self.create_subscription(Odometry, args.desired_topic, self._desired_cb, 20)
        self.create_subscription(Odometry, args.truth_topic, self._truth_cb, 20)
        self.create_subscription(Odometry, args.iap_topic, self._iap_cb, 20)
        self.timer = self.create_timer(1.0 / args.sample_rate, self._sample)

    def _desired_cb(self, msg):
        self.desired = msg

    def _truth_cb(self, msg):
        self.truth = msg

    def _iap_cb(self, msg):
        self.iap = msg

    def _phase(self):
        p = self.desired.pose.pose.position
        return math.atan2(p.y - self.args.center_y, p.x - self.args.center_x)

    def _sample(self):
        now = time.monotonic()
        if now - self.start_wall > self.args.timeout:
            self.get_logger().warning("timeout before collecting requested circle coverage")
            raise SystemExit
        if self.desired is None or self.truth is None or self.iap is None:
            return

        p = self.desired.pose.pose.position
        r = math.hypot(p.x - self.args.center_x, p.y - self.args.center_y)
        phase = self._phase()

        if not self.started:
            near_radius = abs(r - self.args.radius) < self.args.radius_tolerance
            near_height = abs(p.z - self.args.height) < self.args.height_tolerance
            if not near_radius or not near_height:
                return
            self.started = True
            self.phase_offset = phase
            self.unwrapped_prev = 0.0
            self.get_logger().info("circle sampling started")

        unwrapped = phase - self.phase_offset
        while unwrapped < self.unwrapped_prev - math.pi:
            unwrapped += 2.0 * math.pi
        while unwrapped > self.unwrapped_prev + math.pi:
            unwrapped -= 2.0 * math.pi
        self.unwrapped_prev = unwrapped

        t = self.get_clock().now().nanoseconds * 1e-9
        d = self.desired
        tr = self.truth
        ia = self.iap
        roll, pitch = quat_to_rp(ia.pose.pose.orientation)
        dz = ia.pose.pose.position.z - tr.pose.pose.position.z
        row = {
            "t": t,
            "phase_rad": unwrapped,
            "phase_deg": math.degrees(unwrapped),
            "desired_z": d.pose.pose.position.z,
            "truth_z": tr.pose.pose.position.z,
            "iap_z": ia.pose.pose.position.z,
            "iap_minus_truth_z": dz,
            "truth_vz": tr.twist.twist.linear.z,
            "iap_vz": ia.twist.twist.linear.z,
            "iap_roll_deg": math.degrees(roll),
            "iap_pitch_deg": math.degrees(pitch),
        }
        self.rows.append(row)

        if unwrapped >= 2.0 * math.pi:
            self._write_and_summarize()
            raise SystemExit

    def _write_and_summarize(self):
        with open(self.args.output, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

        values = [r["iap_minus_truth_z"] for r in self.rows]
        min_row = min(self.rows, key=lambda r: r["iap_minus_truth_z"])
        max_row = max(self.rows, key=lambda r: r["iap_minus_truth_z"])
        bins = [[] for _ in range(self.args.bins)]
        for row in self.rows:
            idx = min(self.args.bins - 1, max(0, int((row["phase_rad"] % (2.0 * math.pi)) / (2.0 * math.pi) * self.args.bins)))
            bins[idx].append(row["iap_minus_truth_z"])
        bin_means = [statistics.fmean(b) if b else float("nan") for b in bins]
        amp = max(bin_means) - min(bin_means)

        print(f"CSV: {self.args.output}")
        print(f"samples: {len(self.rows)}")
        print(f"iap_z-truth_z mean: {statistics.fmean(values):.4f} m")
        print(f"iap_z-truth_z min:  {min(values):.4f} m at {min_row['phase_deg']:.1f} deg")
        print(f"iap_z-truth_z max:  {max(values):.4f} m at {max_row['phase_deg']:.1f} deg")
        print(f"phase-bin mean amplitude: {amp:.4f} m")
        print("phase_bin_deg,mean_iap_minus_truth_z")
        for i, mean in enumerate(bin_means):
            center = (i + 0.5) * 360.0 / self.args.bins
            print(f"{center:.1f},{mean:.4f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--desired-topic", default="/demo6/desired/odom")
    parser.add_argument("--truth-topic", default="/sim/drone_0/truth_odom")
    parser.add_argument("--iap-topic", default="/drone_0_visual_slam/odom")
    parser.add_argument("--output", default="/tmp/demo6_phase_z.csv")
    parser.add_argument("--center-x", type=float, default=0.0)
    parser.add_argument("--center-y", type=float, default=0.0)
    parser.add_argument("--radius", type=float, default=1.0)
    parser.add_argument("--height", type=float, default=2.0)
    parser.add_argument("--radius-tolerance", type=float, default=0.05)
    parser.add_argument("--height-tolerance", type=float, default=0.15)
    parser.add_argument("--sample-rate", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--bins", type=int, default=12)
    args = parser.parse_args()

    rclpy.init()
    node = Demo6PhaseRecorder(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
