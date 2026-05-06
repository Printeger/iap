#!/usr/bin/env python3
import atexit
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from traj_utils.msg import Bspline
from visualization_msgs.msg import Marker, MarkerArray


CSV_FIELDS = [
    "stamp",
    "traj_id",
    "sample_index",
    "sample_t_from_now",
    "sample_abs_time",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "ax",
    "ay",
    "az",
    "yaw",
    "dist_to_obstacle",
    "dist_to_vertical_lower",
    "dist_to_vertical_upper",
    "AL_H_pred",
    "AL_V_pred",
    "AL_pred",
    "current_HPL",
    "current_VPL",
    "current_PL",
    "PL_H_pred",
    "PL_V_pred",
    "PL_pred",
    "IM_H_pred",
    "IM_V_pred",
    "IM_pred_axis_min",
    "IM_pred_scalar",
    "IM_pred",
    "risk_state_pred",
    "pl_model",
    "al_model",
    "odom_source",
    "map_source",
]


def stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def finite_float(value):
    return float(value) if finite(value) else math.nan


def fmt(value):
    if value is None:
        return "nan"
    try:
        f = float(value)
    except (TypeError, ValueError):
        return "nan"
    if math.isfinite(f):
        return f"{f:.9f}"
    return "nan"


def quantile(values, q):
    vals = sorted(float(v) for v in values if finite(v))
    if not vals:
        return None
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - pos) + vals[hi] * (pos - lo)


class BsplineTrajectory:
    def __init__(self, control_points, order, knots):
        self.control_points = np.asarray(control_points, dtype=float)
        self.order = int(order)
        self.knots = np.asarray(knots, dtype=float)

    @classmethod
    def from_msg(cls, msg):
        points = [[p.x, p.y, p.z] for p in msg.pos_pts]
        return cls(points, int(msg.order), list(msg.knots))

    def valid(self):
        ncp = len(self.control_points)
        return (
            ncp > self.order
            and self.order >= 0
            and len(self.knots) >= ncp + self.order + 1
        )

    def duration(self):
        if not self.valid():
            return 0.0
        ncp = len(self.control_points)
        p = self.order
        end_idx = ncp
        start = self.knots[p]
        end = self.knots[end_idx]
        if math.isfinite(start) and math.isfinite(end) and end >= start:
            return float(end - start)
        return 0.0

    def evaluate(self, u):
        if not self.valid():
            return np.array([math.nan, math.nan, math.nan], dtype=float)

        p = self.order
        ncp = len(self.control_points)
        m = ncp + p
        lo = self.knots[p]
        hi = self.knots[m - p]
        ub = min(max(float(u), float(lo)), float(hi))

        k = p
        while k + 1 < len(self.knots) and self.knots[k + 1] < ub:
            k += 1

        d = [np.array(self.control_points[k - p + i], dtype=float) for i in range(p + 1)]
        for r in range(1, p + 1):
            for i in range(p, r - 1, -1):
                denom = self.knots[i + 1 + k - r] - self.knots[i + k - p]
                alpha = 0.0 if abs(denom) < 1.0e-12 else (ub - self.knots[i + k - p]) / denom
                d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]
        return d[p]

    def evaluate_t(self, t):
        if not self.valid():
            return np.array([math.nan, math.nan, math.nan], dtype=float)
        return self.evaluate(float(t) + self.knots[self.order])

    def derivative(self):
        if not self.valid() or self.order <= 0 or len(self.control_points) < 2:
            return None
        p = self.order
        derived = []
        for i in range(len(self.control_points) - 1):
            denom = self.knots[i + p + 1] - self.knots[i + 1]
            if abs(denom) < 1.0e-12:
                derived.append([math.nan, math.nan, math.nan])
            else:
                derived.append((p * (self.control_points[i + 1] - self.control_points[i]) / denom).tolist())
        return BsplineTrajectory(derived, p - 1, self.knots[1:-1])


class Phase2PlannerIntegrityEvaluator(Node):
    def __init__(self):
        super().__init__("phase2_planner_integrity_evaluator")

        self.declare_parameter("log_root", "/home/dev/ws_iap/src/iap/log")
        self.declare_parameter("run_dir", "")
        self.declare_parameter("odom_topic", "/drone_0_visual_slam/odom")
        self.declare_parameter("bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("pos_cmd_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("map_topic", "/sim/drone_0/lidar")
        self.declare_parameter("integrity_topic", "/iap/integrity")
        self.declare_parameter("eval_horizon_s", 5.0)
        self.declare_parameter("eval_dt_s", 0.2)
        self.declare_parameter("max_samples_per_traj", 30)
        self.declare_parameter("pl_model", "constant_current")
        self.declare_parameter("al_model", "cloud_clearance")
        self.declare_parameter("drone_radius", 0.35)
        self.declare_parameter("safety_buffer", 0.20)
        self.declare_parameter("gamma_h", 0.8)
        self.declare_parameter("gamma_v", 0.8)
        self.declare_parameter("z_min", 0.5)
        self.declare_parameter("z_max", 5.0)
        self.declare_parameter("safe_margin", 0.0)
        self.declare_parameter("publish_markers", True)

        self.log_root = Path(str(self.get_parameter("log_root").value))
        self.explicit_run_dir = str(self.get_parameter("run_dir").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.bspline_topic = str(self.get_parameter("bspline_topic").value)
        self.pos_cmd_topic = str(self.get_parameter("pos_cmd_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.integrity_topic = str(self.get_parameter("integrity_topic").value)
        self.horizon_s = float(self.get_parameter("eval_horizon_s").value)
        self.dt_s = float(self.get_parameter("eval_dt_s").value)
        self.max_samples = int(self.get_parameter("max_samples_per_traj").value)
        self.pl_model = str(self.get_parameter("pl_model").value)
        self.al_model = str(self.get_parameter("al_model").value)
        self.drone_radius = float(self.get_parameter("drone_radius").value)
        self.safety_buffer = float(self.get_parameter("safety_buffer").value)
        self.gamma_h = float(self.get_parameter("gamma_h").value)
        self.gamma_v = float(self.get_parameter("gamma_v").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)
        self.safe_margin = float(self.get_parameter("safe_margin").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)

        self.start_wall = time.time()
        self.initial_latest_target = self._current_latest_target()
        self.run_dir = None
        self.export_dir = None
        self.csv_file = None
        self.csv_writer = None
        self.outputs_open = False
        self.finalized = False

        self.latest_odom_stamp = math.nan
        self.latest_cloud_points = None
        self.latest_cloud_stamp = math.nan
        self.current_hpl = math.nan
        self.current_vpl = math.nan
        self.current_integrity_stamp = math.nan
        self.seen_bspline = False
        self.last_bspline_wall = 0.0
        self.last_fallback_wall = 0.0
        self.fallback_traj_id = 0

        self.traj_count = 0
        self.sample_count = 0
        self.risk_counts = {
            "SAFE_PRED": 0,
            "MARGINAL_PRED": 0,
            "UNSAFE_PRED": 0,
            "UNKNOWN_PL": 0,
            "UNKNOWN_AL": 0,
        }
        self.im_values = []
        self.pl_values = []
        self.warnings = []
        self.errors = []

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)
        self.create_subscription(Bspline, self.bspline_topic, self._on_bspline, qos)
        self.create_subscription(PositionCommand, self.pos_cmd_topic, self._on_pos_cmd, qos)
        self.create_subscription(PointCloud2, self.map_topic, self._on_cloud, qos)
        self._create_integrity_subscription(qos)

        self.marker_pub = None
        if self.publish_markers:
            self.marker_pub = self.create_publisher(MarkerArray, "/iap/planner_integrity_markers", 10)

        self.create_timer(0.5, self._open_outputs_if_ready)
        self.create_timer(2.0, self._write_summary)
        atexit.register(self.finalize)

        self.get_logger().info(
            f"phase2 evaluator started; odom={self.odom_topic} bspline={self.bspline_topic} map={self.map_topic}"
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
        path = self.export_dir / "integrity_along_planner_traj.csv"
        self.csv_file = path.open("w", newline="", buffering=1)
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self.outputs_open = True
        self._write_summary()
        self.get_logger().info(f"phase2 evaluator writing export files under {self.export_dir}")

    def _create_integrity_subscription(self, qos):
        try:
            msg_type = get_message("iap/msg/IntegrityReport")
            self.create_subscription(msg_type, self.integrity_topic, self._on_integrity, qos)
        except Exception as exc:
            self._warn_once(f"integrity topic type unavailable; PL will be NaN online: {exc}")

    def _warn_once(self, text):
        if text not in self.warnings:
            self.warnings.append(text)
            self.get_logger().warn(text)

    def _on_odom(self, msg):
        self.latest_odom_stamp = stamp_to_sec(msg.header.stamp)

    def _on_integrity(self, msg):
        self.current_integrity_stamp = stamp_to_sec(msg.header.stamp)
        self.current_hpl = finite_float(getattr(msg, "hpl", math.nan))
        self.current_vpl = finite_float(getattr(msg, "vpl", math.nan))

    def _on_cloud(self, msg):
        self.latest_cloud_stamp = stamp_to_sec(msg.header.stamp)
        try:
            if hasattr(point_cloud2, "read_points_numpy"):
                arr = point_cloud2.read_points_numpy(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
                arr = np.asarray(arr, dtype=float).reshape(-1, 3)
            else:
                pts = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                arr = np.asarray(pts, dtype=float).reshape(-1, 3) if pts else np.empty((0, 3))
            if arr.size:
                arr = arr[np.isfinite(arr).all(axis=1)]
            self.latest_cloud_points = arr
        except Exception as exc:
            self.latest_cloud_points = None
            self._warn_once(f"failed to parse map cloud {self.map_topic}: {exc}")

    def _on_bspline(self, msg):
        self._open_outputs_if_ready()
        self.seen_bspline = True
        self.last_bspline_wall = time.time()
        if not self.outputs_open:
            return
        if not finite(self.latest_odom_stamp):
            self._warn_once("skipping bspline evaluation until IAP odom is available")
            return

        traj = BsplineTrajectory.from_msg(msg)
        if not traj.valid():
            self._warn_once("received invalid planner bspline; skipping trajectory")
            return

        planner_now = self.get_clock().now().nanoseconds * 1.0e-9
        start_time = stamp_to_sec(msg.start_time)
        t_cur = max(0.0, planner_now - start_time) if finite(start_time) else 0.0
        duration = traj.duration()
        if duration <= 0.0:
            self._warn_once("received zero-duration planner bspline; skipping trajectory")
            return
        if t_cur > duration:
            self._warn_once("received planner bspline that is already expired; skipping trajectory")
            return

        pos_traj = traj
        vel_traj = pos_traj.derivative()
        acc_traj = vel_traj.derivative() if vel_traj is not None else None
        sample_rows = []
        sample_limit = max(0.0, min(self.horizon_s, duration - t_cur))
        count = min(self.max_samples, int(math.floor(sample_limit / max(self.dt_s, 1.0e-6))) + 1)
        count = max(1, count)

        for idx in range(count):
            sample_t = min(idx * self.dt_s, sample_limit)
            eval_t = min(t_cur + sample_t, duration)
            pos = pos_traj.evaluate_t(eval_t)
            vel = vel_traj.evaluate_t(eval_t) if vel_traj is not None else np.array([math.nan] * 3)
            acc = acc_traj.evaluate_t(eval_t) if acc_traj is not None else np.array([math.nan] * 3)
            row = self._make_sample_row(
                stamp=planner_now,
                traj_id=int(msg.traj_id),
                sample_index=idx,
                sample_t_from_now=sample_t,
                sample_abs_time=self.latest_odom_stamp + sample_t,
                pos=pos,
                vel=vel,
                acc=acc,
                yaw=math.nan,
            )
            self._write_sample(row)
            sample_rows.append(row)

        self.traj_count += 1
        self._publish_markers(sample_rows)
        self._write_summary()

    def _on_pos_cmd(self, msg):
        if self.seen_bspline and time.time() - self.last_bspline_wall < 2.0:
            return
        now_wall = time.time()
        if now_wall - self.last_fallback_wall < max(self.dt_s, 0.2):
            return
        self.last_fallback_wall = now_wall
        self._open_outputs_if_ready()
        if not self.outputs_open:
            return
        if not finite(self.latest_odom_stamp):
            self._warn_once("skipping pos_cmd fallback until IAP odom is available")
            return
        self._warn_once("using pos_cmd fallback; future B-spline trajectory is unavailable")

        self.fallback_traj_id += 1
        planner_stamp = stamp_to_sec(msg.header.stamp)
        if not finite(planner_stamp):
            planner_stamp = self.get_clock().now().nanoseconds * 1.0e-9
        pos = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        vel = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        acc = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
        row = self._make_sample_row(
            stamp=planner_stamp,
            traj_id=-self.fallback_traj_id,
            sample_index=0,
            sample_t_from_now=0.0,
            sample_abs_time=self.latest_odom_stamp,
            pos=pos,
            vel=vel,
            acc=acc,
            yaw=float(msg.yaw),
        )
        self._write_sample(row)
        self.traj_count += 1
        self._publish_markers([row])
        self._write_summary()

    def _nearest_obstacle_distance(self, pos):
        if self.al_model != "cloud_clearance":
            return math.nan
        points = self.latest_cloud_points
        if points is None or len(points) == 0:
            self._warn_once("map/cloud not available yet; AL_H_pred is NaN")
            return math.nan
        p = np.asarray(pos, dtype=float).reshape(1, 3)
        if not np.isfinite(p).all():
            return math.nan
        diff = points - p
        dist2 = np.einsum("ij,ij->i", diff, diff)
        if len(dist2) == 0:
            return math.nan
        return float(math.sqrt(float(np.min(dist2))))

    def _al_values(self, pos):
        dist = self._nearest_obstacle_distance(pos)
        if finite(dist):
            clearance_h = dist - self.drone_radius - self.safety_buffer
            al_h = self.gamma_h * max(clearance_h, 0.0)
        else:
            al_h = math.nan
        z = finite_float(pos[2])
        lower = z - self.z_min if finite(z) else math.nan
        upper = self.z_max - z if finite(z) else math.nan
        if finite(lower) and finite(upper):
            al_v = self.gamma_v * max(min(lower, upper), 0.0)
        else:
            al_v = math.nan
        al = min(al_h, al_v) if finite(al_h) and finite(al_v) else math.nan
        return dist, lower, upper, al_h, al_v, al

    def _pl_values(self):
        if self.pl_model != "constant_current":
            return math.nan, math.nan, math.nan
        hpl = self.current_hpl
        vpl = self.current_vpl
        pl = max(hpl, vpl) if finite(hpl) and finite(vpl) else math.nan
        return hpl, vpl, pl

    def _im_values(self, al_h, al_v, al, pl_h, pl_v, pl):
        if not finite(al):
            return math.nan, math.nan, math.nan, math.nan, math.nan, "UNKNOWN_AL"
        if not finite(pl):
            return math.nan, math.nan, math.nan, math.nan, math.nan, "UNKNOWN_PL"
        im_h = al_h - pl_h if finite(al_h) and finite(pl_h) else math.nan
        im_v = al_v - pl_v if finite(al_v) and finite(pl_v) else math.nan
        axis_vals = [v for v in (im_h, im_v) if finite(v)]
        im_axis_min = min(axis_vals) if axis_vals else math.nan
        im_scalar = al - pl
        candidates = [v for v in (im_axis_min, im_scalar) if finite(v)]
        im = min(candidates) if candidates else math.nan
        if not finite(im):
            state = "UNKNOWN_PL"
        elif im > self.safe_margin:
            state = "SAFE_PRED"
        elif abs(im) <= self.safe_margin:
            state = "MARGINAL_PRED"
        else:
            state = "UNSAFE_PRED"
        return im_h, im_v, im_axis_min, im_scalar, im, state

    def _make_sample_row(self, stamp, traj_id, sample_index, sample_t_from_now, sample_abs_time, pos, vel, acc, yaw):
        dist, lower, upper, al_h, al_v, al = self._al_values(pos)
        current_hpl, current_vpl, current_pl = self._pl_values()
        pl_h, pl_v, pl = current_hpl, current_vpl, current_pl
        im_h, im_v, im_axis_min, im_scalar, im, state = self._im_values(al_h, al_v, al, pl_h, pl_v, pl)
        row = {
            "stamp": fmt(stamp),
            "traj_id": int(traj_id),
            "sample_index": int(sample_index),
            "sample_t_from_now": fmt(sample_t_from_now),
            "sample_abs_time": fmt(sample_abs_time),
            "x": fmt(pos[0]),
            "y": fmt(pos[1]),
            "z": fmt(pos[2]),
            "vx": fmt(vel[0]),
            "vy": fmt(vel[1]),
            "vz": fmt(vel[2]),
            "ax": fmt(acc[0]),
            "ay": fmt(acc[1]),
            "az": fmt(acc[2]),
            "yaw": fmt(yaw),
            "dist_to_obstacle": fmt(dist),
            "dist_to_vertical_lower": fmt(lower),
            "dist_to_vertical_upper": fmt(upper),
            "AL_H_pred": fmt(al_h),
            "AL_V_pred": fmt(al_v),
            "AL_pred": fmt(al),
            "current_HPL": fmt(current_hpl),
            "current_VPL": fmt(current_vpl),
            "current_PL": fmt(current_pl),
            "PL_H_pred": fmt(pl_h),
            "PL_V_pred": fmt(pl_v),
            "PL_pred": fmt(pl),
            "IM_H_pred": fmt(im_h),
            "IM_V_pred": fmt(im_v),
            "IM_pred_axis_min": fmt(im_axis_min),
            "IM_pred_scalar": fmt(im_scalar),
            "IM_pred": fmt(im),
            "risk_state_pred": state,
            "pl_model": self.pl_model,
            "al_model": self.al_model,
            "odom_source": self.odom_topic,
            "map_source": self.map_topic,
        }
        return row

    def _write_sample(self, row):
        if not self.csv_writer:
            return
        self.csv_writer.writerow(row)
        self.sample_count += 1
        state = row.get("risk_state_pred", "UNKNOWN_PL")
        self.risk_counts[state] = self.risk_counts.get(state, 0) + 1
        im = finite_float(row.get("IM_pred"))
        pl = finite_float(row.get("PL_pred"))
        if finite(im):
            self.im_values.append(im)
        if finite(pl):
            self.pl_values.append(pl)

    def _summary_data(self):
        finite_im = [v for v in self.im_values if finite(v)]
        finite_pl = [v for v in self.pl_values if finite(v)]
        return {
            "available": True,
            "run_dir": str(self.run_dir) if self.run_dir else "",
            "traj_count": int(self.traj_count),
            "sample_count": int(self.sample_count),
            "aligned_sample_count": 0,
            "online_truth_used": False,
            "odom_source": self.odom_topic,
            "map_source": self.map_topic,
            "pl_model": self.pl_model,
            "al_model": self.al_model,
            "sampling": {
                "horizon_s": self.horizon_s,
                "dt_s": self.dt_s,
                "max_samples_per_traj": self.max_samples,
            },
            "predicted_integrity": {
                "safe_count": int(self.risk_counts.get("SAFE_PRED", 0)),
                "marginal_count": int(self.risk_counts.get("MARGINAL_PRED", 0)),
                "unsafe_count": int(self.risk_counts.get("UNSAFE_PRED", 0)),
                "unknown_count": int(
                    self.risk_counts.get("UNKNOWN_PL", 0) + self.risk_counts.get("UNKNOWN_AL", 0)
                ),
                "min_IM": min(finite_im) if finite_im else None,
                "mean_IM": sum(finite_im) / len(finite_im) if finite_im else None,
                "p05_IM": quantile(finite_im, 0.05),
                "p50_IM": quantile(finite_im, 0.50),
                "p95_PL": quantile(finite_pl, 0.95),
                "max_PL": max(finite_pl) if finite_pl else None,
            },
            "actual_alignment": {
                "matched_count": 0,
                "match_ratio": 0.0,
                "mean_time_alignment_error_s": None,
                "mean_spatial_tracking_error": None,
                "mean_estimation_error": None,
                "mean_pred_actual_PL_error": None,
                "mean_pred_actual_IM_error": None,
                "safe_unsafe_label_agreement_ratio": None,
            },
            "warnings": self.warnings,
            "errors": self.errors,
        }

    def _write_summary(self):
        if not self.outputs_open or not self.export_dir:
            return
        (self.export_dir / "phase2_summary.json").write_text(
            json.dumps(self._summary_data(), indent=2) + "\n"
        )

    def _publish_markers(self, rows):
        if not self.marker_pub:
            return
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for idx, row in enumerate(rows):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "phase2_pi_lite"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(
                x=finite_float(row.get("x")),
                y=finite_float(row.get("y")),
                z=finite_float(row.get("z")),
            )
            state = row.get("risk_state_pred", "UNKNOWN_PL")
            scale = 0.12
            color = (0.5, 0.5, 0.5, 0.65)
            if state == "SAFE_PRED":
                color = (0.1, 0.8, 0.2, 0.75)
            elif state == "MARGINAL_PRED":
                color = (1.0, 0.75, 0.05, 0.8)
                scale = 0.18
            elif state == "UNSAFE_PRED":
                color = (1.0, 0.05, 0.05, 0.9)
                scale = 0.24
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        try:
            if self.outputs_open:
                self._write_summary()
        finally:
            if self.csv_file:
                try:
                    self.csv_file.flush()
                    self.csv_file.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = Phase2PlannerIntegrityEvaluator()
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
