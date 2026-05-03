#!/usr/bin/env python3
import atexit
import csv
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from traj_utils.msg import Bspline


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def msg_stamp_or_now(msg, node):
    header = getattr(msg, "header", None)
    if header is not None:
        stamp = getattr(header, "stamp", None)
        if stamp is not None and (stamp.sec != 0 or stamp.nanosec != 0):
            return stamp_to_sec(stamp)
    return node.get_clock().now().nanoseconds * 1.0e-9


def odom_position(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), float(p.z))


def odom_velocity(msg):
    v = msg.twist.twist.linear
    return (float(v.x), float(v.y), float(v.z))


def vector_norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def finite_or_zero(value):
    return float(value) if math.isfinite(float(value)) else 0.0


class Phase1ClosedLoopLogger(Node):
    def __init__(self):
        super().__init__("phase1_closed_loop_logger")

        self.declare_parameter("log_root", "/home/dev/ws_iap/src/iap/log")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("run_duration_s", 120.0)
        self.declare_parameter("goal_x", 8.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_z", 2.0)
        self.declare_parameter("use_gnss", True)
        self.declare_parameter("use_araim", True)
        self.declare_parameter("allow_truth_alignment", True)
        self.declare_parameter("map_source", "local_sensing_cloud")
        self.declare_parameter("gnss_ephemeris_source", "rinex")
        self.declare_parameter("gnss_enabled_constellations", "GPS,BDS,GAL,GLO")
        self.declare_parameter("gnss_rinex_nav_file", "")
        self.declare_parameter("truth_odom_topic", "/sim/drone_0/truth_odom")
        self.declare_parameter("iap_odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("integrity_topic", "/iap/integrity")

        self.log_root = Path(self.get_parameter("log_root").value)
        self.explicit_run_dir = str(self.get_parameter("run_dir").value)
        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
            float(self.get_parameter("goal_z").value),
        )
        self.run_duration_config_s = float(self.get_parameter("run_duration_s").value)
        self.use_gnss = bool(self.get_parameter("use_gnss").value)
        self.use_araim = bool(self.get_parameter("use_araim").value)
        self.allow_truth_alignment = bool(self.get_parameter("allow_truth_alignment").value)
        self.map_source = str(self.get_parameter("map_source").value)
        self.gnss_ephemeris_source = str(self.get_parameter("gnss_ephemeris_source").value)
        self.gnss_enabled_constellations = str(self.get_parameter("gnss_enabled_constellations").value)
        self.gnss_rinex_nav_file = str(self.get_parameter("gnss_rinex_nav_file").value)

        self.topics = {
            "truth_odom": str(self.get_parameter("truth_odom_topic").value),
            "iap_odom": str(self.get_parameter("iap_odom_topic").value),
            "pos_cmd": str(self.get_parameter("pos_cmd_topic").value),
            "bspline": str(self.get_parameter("bspline_topic").value),
            "integrity": str(self.get_parameter("integrity_topic").value),
        }

        self.start_wall = time.time()
        self.initial_latest_target = self._current_latest_target()
        self.run_dir = None
        self.export_dir = None
        self.files = {}
        self.writers = {}
        self.outputs_open = False
        self.finalized = False

        self.truth_cache = deque(maxlen=6000)
        self.iap_cache = deque(maxlen=6000)
        self.first_truth_pos = None
        self.last_truth_pos = None
        self.first_truth_stamp = None
        self.last_truth_stamp = None

        self.topic_stats = {}
        self.counts = {
            "planner_trajectory_count": 0,
            "planner_command_count": 0,
            "truth_odom_count": 0,
            "iap_odom_count": 0,
            "integrity_count": 0,
        }
        self.tracking_errors = []
        self.estimation_errors = []
        self.horizontal_errors = []
        self.vertical_errors = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, self.topics["truth_odom"], self._on_truth_odom, qos)
        self.create_subscription(Odometry, self.topics["iap_odom"], self._on_iap_odom, qos)
        self.create_subscription(PositionCommand, self.topics["pos_cmd"], self._on_pos_cmd, qos)
        self.create_subscription(Bspline, self.topics["bspline"], self._on_bspline, qos)
        self._create_integrity_subscription(qos)

        self.create_timer(0.5, self._open_outputs_if_ready)
        self.create_timer(2.0, self._write_periodic_summary)
        atexit.register(self.finalize)

        self.get_logger().info(
            f"phase1 logger started; waiting for IAP run dir under {self.log_root}"
        )

    def _current_latest_target(self):
        latest = self.log_root / "latest"
        try:
            if latest.exists():
                return latest.resolve()
        except OSError:
            return None
        return None

    def _resolve_run_dir(self):
        if self.explicit_run_dir:
            path = Path(self.explicit_run_dir).expanduser()
            return path.resolve() if path.exists() else path

        latest = self.log_root / "latest"
        try:
            if not latest.exists():
                return None
            target = latest.resolve()
            if not target.is_dir():
                return None

            if self.initial_latest_target is None or target != self.initial_latest_target:
                return target

            if target.stat().st_mtime >= self.start_wall - 2.0:
                return target
        except OSError:
            return None
        return None

    def _open_outputs_if_ready(self):
        if self.outputs_open:
            return
        run_dir = self._resolve_run_dir()
        if run_dir is None:
            return

        self.run_dir = run_dir
        self.export_dir = self.run_dir / "export"
        self.export_dir.mkdir(parents=True, exist_ok=True)

        self._open_csv(
            "desired_vs_truth",
            "desired_vs_truth.csv",
            [
                "stamp",
                "desired_x",
                "desired_y",
                "desired_z",
                "desired_vx",
                "desired_vy",
                "desired_vz",
                "desired_ax",
                "desired_ay",
                "desired_az",
                "truth_x",
                "truth_y",
                "truth_z",
                "truth_vx",
                "truth_vy",
                "truth_vz",
                "iap_x",
                "iap_y",
                "iap_z",
                "err_truth_des_x",
                "err_truth_des_y",
                "err_truth_des_z",
                "err_iap_truth_x",
                "err_iap_truth_y",
                "err_iap_truth_z",
                "horizontal_tracking_error",
                "vertical_tracking_error",
                "position_tracking_error",
                "estimation_position_error",
            ],
        )
        self._open_csv(
            "tracking_error",
            "tracking_error.csv",
            [
                "stamp",
                "horizontal_tracking_error",
                "vertical_tracking_error",
                "position_tracking_error",
                "estimation_position_error",
                "err_iap_truth_x",
                "err_iap_truth_y",
                "err_iap_truth_z",
            ],
        )
        self._open_csv(
            "planner_traj",
            "planner_traj.csv",
            [
                "stamp",
                "start_time",
                "trajectory_id",
                "order",
                "knot_count",
                "control_point_count",
                "duration",
                "first_x",
                "first_y",
                "first_z",
                "last_x",
                "last_y",
                "last_z",
            ],
        )
        self._open_csv(
            "planner_cmd",
            "planner_cmd.csv",
            [
                "stamp",
                "trajectory_id",
                "trajectory_flag",
                "desired_x",
                "desired_y",
                "desired_z",
                "desired_vx",
                "desired_vy",
                "desired_vz",
                "desired_ax",
                "desired_ay",
                "desired_az",
                "yaw",
                "yaw_dot",
            ],
        )
        self.outputs_open = True
        self._write_topic_contract()
        self._write_summary()
        self.get_logger().info(f"phase1 logger writing export files under {self.export_dir}")

    def _open_csv(self, key, name, fieldnames):
        path = self.export_dir / name
        handle = path.open("w", newline="", buffering=1)
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        self.files[key] = handle
        self.writers[key] = writer

    def _create_integrity_subscription(self, qos):
        try:
            msg_type = get_message("iap/msg/IntegrityReport")
            self.create_subscription(msg_type, self.topics["integrity"], self._on_integrity, qos)
        except Exception as exc:
            self.get_logger().warn(
                f"integrity topic type unavailable for optional count: {exc}"
            )

    def _record_topic(self, topic, msg):
        stamp = msg_stamp_or_now(msg, self)
        stat = self.topic_stats.setdefault(
            topic, {"count": 0, "first_stamp": None, "last_stamp": None}
        )
        stat["count"] += 1
        if stat["first_stamp"] is None:
            stat["first_stamp"] = stamp
        stat["last_stamp"] = stamp

    def _on_truth_odom(self, msg):
        self._record_topic(self.topics["truth_odom"], msg)
        self.counts["truth_odom_count"] += 1
        self.truth_cache.append(msg)
        pos = odom_position(msg)
        stamp = stamp_to_sec(msg.header.stamp)
        if self.first_truth_pos is None:
            self.first_truth_pos = pos
            self.first_truth_stamp = stamp
        self.last_truth_pos = pos
        self.last_truth_stamp = stamp

    def _on_iap_odom(self, msg):
        self._record_topic(self.topics["iap_odom"], msg)
        self.counts["iap_odom_count"] += 1
        self.iap_cache.append(msg)

    def _on_integrity(self, msg):
        self._record_topic(self.topics["integrity"], msg)
        self.counts["integrity_count"] += 1

    def _nearest(self, cache, stamp):
        if not cache:
            return None
        return min(cache, key=lambda msg: abs(stamp_to_sec(msg.header.stamp) - stamp))

    def _on_bspline(self, msg):
        self._record_topic(self.topics["bspline"], msg)
        self.counts["planner_trajectory_count"] += 1
        if not self.outputs_open:
            return

        stamp = msg_stamp_or_now(msg, self)
        start_time = stamp_to_sec(msg.start_time)
        duration = 0.0
        if len(msg.knots) >= 2:
            duration = float(msg.knots[-1] - msg.knots[0])

        first = msg.pos_pts[0] if msg.pos_pts else None
        last = msg.pos_pts[-1] if msg.pos_pts else None
        self.writers["planner_traj"].writerow(
            {
                "stamp": f"{stamp:.9f}",
                "start_time": f"{start_time:.9f}",
                "trajectory_id": int(msg.traj_id),
                "order": int(msg.order),
                "knot_count": len(msg.knots),
                "control_point_count": len(msg.pos_pts),
                "duration": f"{duration:.6f}",
                "first_x": f"{first.x:.9f}" if first else "",
                "first_y": f"{first.y:.9f}" if first else "",
                "first_z": f"{first.z:.9f}" if first else "",
                "last_x": f"{last.x:.9f}" if last else "",
                "last_y": f"{last.y:.9f}" if last else "",
                "last_z": f"{last.z:.9f}" if last else "",
            }
        )

    def _on_pos_cmd(self, msg):
        self._record_topic(self.topics["pos_cmd"], msg)
        self.counts["planner_command_count"] += 1
        stamp = msg_stamp_or_now(msg, self)

        if self.outputs_open:
            self.writers["planner_cmd"].writerow(
                {
                    "stamp": f"{stamp:.9f}",
                    "trajectory_id": int(msg.trajectory_id),
                    "trajectory_flag": int(msg.trajectory_flag),
                    "desired_x": f"{msg.position.x:.9f}",
                    "desired_y": f"{msg.position.y:.9f}",
                    "desired_z": f"{msg.position.z:.9f}",
                    "desired_vx": f"{msg.velocity.x:.9f}",
                    "desired_vy": f"{msg.velocity.y:.9f}",
                    "desired_vz": f"{msg.velocity.z:.9f}",
                    "desired_ax": f"{msg.acceleration.x:.9f}",
                    "desired_ay": f"{msg.acceleration.y:.9f}",
                    "desired_az": f"{msg.acceleration.z:.9f}",
                    "yaw": f"{msg.yaw:.9f}",
                    "yaw_dot": f"{msg.yaw_dot:.9f}",
                }
            )

        truth = self._nearest(self.truth_cache, stamp)
        iap = self._nearest(self.iap_cache, stamp)
        if truth is None or iap is None or not self.outputs_open:
            return

        truth_p = odom_position(truth)
        truth_v = odom_velocity(truth)
        iap_p = odom_position(iap)
        des_p = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
        des_v = (float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z))
        des_a = (float(msg.acceleration.x), float(msg.acceleration.y), float(msg.acceleration.z))

        err_truth_des = (
            truth_p[0] - des_p[0],
            truth_p[1] - des_p[1],
            truth_p[2] - des_p[2],
        )
        err_iap_truth = (
            iap_p[0] - truth_p[0],
            iap_p[1] - truth_p[1],
            iap_p[2] - truth_p[2],
        )
        horizontal_tracking_error = math.hypot(err_truth_des[0], err_truth_des[1])
        vertical_tracking_error = abs(err_truth_des[2])
        position_tracking_error = vector_norm3(*err_truth_des)
        estimation_position_error = vector_norm3(*err_iap_truth)

        self.horizontal_errors.append(horizontal_tracking_error)
        self.vertical_errors.append(vertical_tracking_error)
        self.tracking_errors.append(position_tracking_error)
        self.estimation_errors.append(estimation_position_error)

        row = {
            "stamp": f"{stamp:.9f}",
            "desired_x": f"{des_p[0]:.9f}",
            "desired_y": f"{des_p[1]:.9f}",
            "desired_z": f"{des_p[2]:.9f}",
            "desired_vx": f"{des_v[0]:.9f}",
            "desired_vy": f"{des_v[1]:.9f}",
            "desired_vz": f"{des_v[2]:.9f}",
            "desired_ax": f"{des_a[0]:.9f}",
            "desired_ay": f"{des_a[1]:.9f}",
            "desired_az": f"{des_a[2]:.9f}",
            "truth_x": f"{truth_p[0]:.9f}",
            "truth_y": f"{truth_p[1]:.9f}",
            "truth_z": f"{truth_p[2]:.9f}",
            "truth_vx": f"{truth_v[0]:.9f}",
            "truth_vy": f"{truth_v[1]:.9f}",
            "truth_vz": f"{truth_v[2]:.9f}",
            "iap_x": f"{iap_p[0]:.9f}",
            "iap_y": f"{iap_p[1]:.9f}",
            "iap_z": f"{iap_p[2]:.9f}",
            "err_truth_des_x": f"{err_truth_des[0]:.9f}",
            "err_truth_des_y": f"{err_truth_des[1]:.9f}",
            "err_truth_des_z": f"{err_truth_des[2]:.9f}",
            "err_iap_truth_x": f"{err_iap_truth[0]:.9f}",
            "err_iap_truth_y": f"{err_iap_truth[1]:.9f}",
            "err_iap_truth_z": f"{err_iap_truth[2]:.9f}",
            "horizontal_tracking_error": f"{horizontal_tracking_error:.9f}",
            "vertical_tracking_error": f"{vertical_tracking_error:.9f}",
            "position_tracking_error": f"{position_tracking_error:.9f}",
            "estimation_position_error": f"{estimation_position_error:.9f}",
        }
        self.writers["desired_vs_truth"].writerow(row)
        self.writers["tracking_error"].writerow(
            {
                "stamp": row["stamp"],
                "horizontal_tracking_error": row["horizontal_tracking_error"],
                "vertical_tracking_error": row["vertical_tracking_error"],
                "position_tracking_error": row["position_tracking_error"],
                "estimation_position_error": row["estimation_position_error"],
                "err_iap_truth_x": row["err_iap_truth_x"],
                "err_iap_truth_y": row["err_iap_truth_y"],
                "err_iap_truth_z": row["err_iap_truth_z"],
            }
        )

    def _metric_summary(self, values):
        if not values:
            return {"rmse": None, "p95": None, "max": None}
        vals = [finite_or_zero(v) for v in values]
        rmse = math.sqrt(sum(v * v for v in vals) / len(vals))
        sorted_vals = sorted(vals)
        idx95 = min(len(sorted_vals) - 1, int(math.ceil(0.95 * len(sorted_vals))) - 1)
        return {"rmse": rmse, "p95": sorted_vals[idx95], "max": max(sorted_vals)}

    def _topic_hz(self):
        out = {}
        for topic, stat in self.topic_stats.items():
            count = int(stat["count"])
            first = stat["first_stamp"]
            last = stat["last_stamp"]
            hz = 0.0
            if count > 1 and first is not None and last is not None and last > first:
                hz = float(count - 1) / float(last - first)
            out[topic] = {
                "count": count,
                "first_stamp": first,
                "last_stamp": last,
                "hz": hz,
            }
        return out

    def _run_duration(self):
        if self.first_truth_stamp is not None and self.last_truth_stamp is not None:
            return max(0.0, self.last_truth_stamp - self.first_truth_stamp)
        return max(0.0, time.time() - self.start_wall)

    def _distance_to_goal(self, pos):
        if pos is None:
            return None
        return vector_norm3(pos[0] - self.goal[0], pos[1] - self.goal[1], pos[2] - self.goal[2])

    def _simulator_movement(self):
        if self.first_truth_pos is None or self.last_truth_pos is None:
            return 0.0
        return vector_norm3(
            self.last_truth_pos[0] - self.first_truth_pos[0],
            self.last_truth_pos[1] - self.first_truth_pos[1],
            self.last_truth_pos[2] - self.first_truth_pos[2],
        )

    def _summary_data(self):
        export_dir = self.export_dir if self.export_dir else None
        araim_path = export_dir / "iap_araim.csv" if export_dir else None
        return {
            "run_duration_s": self._run_duration(),
            "configured_run_duration_s": self.run_duration_config_s,
            "goal": {"x": self.goal[0], "y": self.goal[1], "z": self.goal[2]},
            "use_gnss": self.use_gnss,
            "use_araim": self.use_araim,
            "allow_truth_alignment": self.allow_truth_alignment,
            "map_source": self.map_source,
            "gnss_ephemeris_source": self.gnss_ephemeris_source,
            "gnss_enabled_constellations": self.gnss_enabled_constellations,
            "gnss_rinex_nav_file": self.gnss_rinex_nav_file,
            **self.counts,
            "desired_vs_truth_count": len(self.tracking_errors),
            "tracking": self._metric_summary(self.tracking_errors),
            "tracking_horizontal": self._metric_summary(self.horizontal_errors),
            "tracking_vertical": self._metric_summary(self.vertical_errors),
            "estimation": self._metric_summary(self.estimation_errors),
            "topic_hz": self._topic_hz(),
            "iap_araim_found": bool(araim_path and araim_path.exists() and araim_path.stat().st_size > 0),
            "simulator_movement_m": self._simulator_movement(),
            "initial_distance_to_goal_m": self._distance_to_goal(self.first_truth_pos),
            "final_distance_to_goal_m": self._distance_to_goal(self.last_truth_pos),
        }

    def _write_topic_contract(self):
        if not self.outputs_open:
            return
        data = {
            "phase": "phase1_ego_planner_closed_loop",
            "frames": {
                "global_frame": "map",
                "iap_planner_body_frame": "imu",
                "iap_lidar_body_cloud_frame": "lidar",
            },
            "topics": {
                "truth_odom": {
                    "name": self.topics["truth_odom"],
                    "type": "nav_msgs/msg/Odometry",
                    "role": "plant state only; sensor simulation, GNSS simulation, visualization, logging",
                },
                "iap_odom": {
                    "name": self.topics["iap_odom"],
                    "type": "nav_msgs/msg/Odometry",
                    "role": "planner and SO3 controller feedback",
                },
                "planner_bspline": {
                    "name": self.topics["bspline"],
                    "type": "traj_utils/msg/Bspline",
                    "role": "ego_planner output",
                },
                "position_command": {
                    "name": self.topics["pos_cmd"],
                    "type": "quadrotor_msgs/msg/PositionCommand",
                    "role": "traj_server output and SO3 controller input",
                },
                "integrity": {
                    "name": self.topics["integrity"],
                    "type": "iap/msg/IntegrityReport",
                    "role": "optional ARAIM/integrity report",
                },
            },
            "runtime": {
                "use_gnss": self.use_gnss,
                "use_araim": self.use_araim,
                "allow_truth_alignment": self.allow_truth_alignment,
                "map_source": self.map_source,
                "gnss_ephemeris_source": self.gnss_ephemeris_source,
                "gnss_enabled_constellations": self.gnss_enabled_constellations,
                "gnss_rinex_nav_file": self.gnss_rinex_nav_file,
            },
            "observed": self._topic_hz(),
        }
        (self.export_dir / "topic_contract.json").write_text(json.dumps(data, indent=2) + "\n")

    def _write_summary(self):
        if not self.outputs_open:
            return
        (self.export_dir / "phase1_summary.json").write_text(
            json.dumps(self._summary_data(), indent=2) + "\n"
        )

    def _write_periodic_summary(self):
        if not self.outputs_open:
            return
        self._write_topic_contract()
        self._write_summary()

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        try:
            if self.outputs_open:
                self._write_topic_contract()
                self._write_summary()
        finally:
            for handle in self.files.values():
                try:
                    handle.flush()
                    handle.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = Phase1ClosedLoopLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
