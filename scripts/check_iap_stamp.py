#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from gnss_comm.msg import (
    GnssEphemMsg,
    GnssGloEphemMsg,
    GnssIonosphereParameter,
    GnssMeasMsg,
)
from iap.msg import IntegrityReport
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand, SO3Command
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = PACKAGE_ROOT / "results" / "topic_monitor"
SIM_2022_MIN_S = 1650000000.0
SIM_2022_MAX_S = 1665000000.0
SIM_DOMAIN = "sim-2022"
LIDAR_BODY_FRESHNESS_LIMIT_S = 1.0
PREDICTOR_SOURCE_COUNTER_FIELDS = [
    "predictor_gnss_used_count",
    "predictor_lidar_used_count",
    "predictor_prior_used_count",
    "predictor_stale_current_prior_count",
    "predictor_regularized_count",
    "predictor_conservative_max_count",
]
PREDICTOR_LIDAR_INPUT_FIELDS = [
    "predictor_lidar_map_point_count",
    "predictor_lidar_fim_primitive_count",
    "predictor_lidar_fim_valid_normal_count",
    "predictor_lidar_fim_fallback_reason",
]
P0_UNKNOWN_REASON_FIELDS = [
    "dominant_unknown_reason",
    "dominant_unknown_count",
]


def header_stamp(msg: Any) -> Any:
    header = getattr(msg, "header", None)
    return getattr(header, "stamp", None)


def marker_array_stamp(msg: MarkerArray) -> Any:
    if not msg.markers:
        return None
    return msg.markers[0].header.stamp


def no_stamp(_: Any) -> Any:
    return None


def health_refresh_stamp(msg: String) -> Optional[float]:
    try:
        data = json.loads(msg.data)
    except Exception:
        return None
    value = data.get("refresh_stamp_s")
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    value = data.get("last_grid_stamp_s")
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return None


def stamp_s(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    if isinstance(stamp, (int, float)):
        value = float(stamp)
        return value if math.isfinite(value) else None
    if hasattr(stamp, "sec") and hasattr(stamp, "nanosec"):
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return None


def stamp_domain(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value) or value <= 0.0:
        return "missing"
    if SIM_2022_MIN_S < value < SIM_2022_MAX_S:
        return "sim-2022"
    return "wall-or-other"


def fmt_float(value: Optional[float], precision: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def fmt_time(value: Optional[float]) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return datetime.fromtimestamp(value, tz=timezone.utc).strftime(
        "%Y-%m-%d %H:%M:%S.%f UTC"
    )[:-7] + " UTC"


def md_escape(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def safe_name(value: str) -> str:
    safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in value)
    return safe.strip("_") or "iap_topic_monitor"


@dataclass(frozen=True)
class TopicSpec:
    name: str
    msg_type: Any
    stamp_getter: Callable[[Any], Any]
    required: bool
    min_hz: float
    category: str
    notes: str
    qos: Any = 10
    stamp_required: bool = True
    parse_health_json: bool = False


@dataclass
class TopicStats:
    spec: TopicSpec
    count: int = 0
    receive_epoch_s: list[float] = field(default_factory=list)
    receive_mono_s: list[float] = field(default_factory=list)
    stamp_values_s: list[float] = field(default_factory=list)
    missing_stamp_count: int = 0
    invalid_stamp_count: int = 0
    domain_counts: Counter = field(default_factory=Counter)
    health_records: list[dict[str, Any]] = field(default_factory=list)
    raw_events: list[str] = field(default_factory=list)

    def record(self, msg: Any, stamp: Optional[float], now_epoch: float, now_mono: float) -> None:
        self.count += 1
        self.receive_epoch_s.append(now_epoch)
        self.receive_mono_s.append(now_mono)
        domain = stamp_domain(stamp)
        self.domain_counts[domain] += 1
        if stamp is None:
            self.missing_stamp_count += 1
        elif stamp <= 0.0 or not math.isfinite(stamp):
            self.invalid_stamp_count += 1
        else:
            self.stamp_values_s.append(stamp)
        if self.spec.parse_health_json:
            self._record_health(msg)

    def _record_health(self, msg: Any) -> None:
        data = getattr(msg, "data", "")
        try:
            parsed = json.loads(data)
        except Exception as exc:
            self.raw_events.append(f"health_json_parse_error: {exc}: {data[:160]}")
            return
        if isinstance(parsed, dict):
            self.health_records.append(parsed)

    def receive_intervals(self) -> list[float]:
        if len(self.receive_mono_s) < 2:
            return []
        return [
            b - a
            for a, b in zip(self.receive_mono_s[:-1], self.receive_mono_s[1:])
            if b >= a
        ]

    def stamp_intervals(self) -> list[float]:
        if len(self.stamp_values_s) < 2:
            return []
        return [
            b - a
            for a, b in zip(self.stamp_values_s[:-1], self.stamp_values_s[1:])
            if b >= a
        ]

    def mean_hz(self) -> Optional[float]:
        intervals = self.receive_intervals()
        if not intervals:
            return None
        total = sum(intervals)
        if total <= 0.0:
            return None
        return len(intervals) / total

    def min_hz(self) -> Optional[float]:
        intervals = self.receive_intervals()
        if not intervals:
            return None
        max_dt = max(intervals)
        return 1.0 / max_dt if max_dt > 0.0 else None

    def max_hz(self) -> Optional[float]:
        intervals = self.receive_intervals()
        if not intervals:
            return None
        min_dt = min(intervals)
        return 1.0 / min_dt if min_dt > 0.0 else None

    def max_gap_s(self) -> Optional[float]:
        intervals = self.receive_intervals()
        return max(intervals) if intervals else None

    def first_stamp(self) -> Optional[float]:
        return self.stamp_values_s[0] if self.stamp_values_s else None

    def last_stamp(self) -> Optional[float]:
        return self.stamp_values_s[-1] if self.stamp_values_s else None

    def latest_domain(self) -> str:
        return stamp_domain(self.last_stamp())

    def stuck(self) -> bool:
        if len(self.stamp_values_s) < 3:
            return False
        return max(self.stamp_values_s[-3:]) - min(self.stamp_values_s[-3:]) < 1.0e-9


def build_topic_specs() -> list[TopicSpec]:
    sensor_qos = qos_profile_sensor_data
    default_qos = QoSProfile(depth=50)
    return [
        TopicSpec("/sim/drone_0/truth_odom", Odometry, header_stamp, True, 50.0, "sim", "truth odom / sim time baseline", default_qos),
        TopicSpec("/sim/drone_0/imu", Imu, header_stamp, True, 50.0, "sim", "raw simulator IMU", sensor_qos),
        TopicSpec("/sim/drone_0/imu_iap", Imu, header_stamp, True, 50.0, "iap-input", "IMU consumed by IAP/GLIM", sensor_qos),
        TopicSpec("/sim/drone_0/lidar", PointCloud2, header_stamp, True, 1.0, "sim", "raw simulator lidar cloud", sensor_qos),
        TopicSpec("/sim/drone_0/lidar_body", PointCloud2, header_stamp, True, 1.0, "iap-input", "lidar cloud consumed by IAP/GLIM", sensor_qos),
        TopicSpec("/drone_0_visual_slam/odom", Odometry, header_stamp, True, 5.0, "iap-output", "IAP/GLIM odom output", default_qos),
        TopicSpec("/iap/integrity", IntegrityReport, header_stamp, True, 3.0, "integrity", "P0/P5 integrity input", default_qos),
        TopicSpec("/drone_0_planning/pos_cmd", PositionCommand, header_stamp, True, 5.0, "planner", "planner position command", default_qos),
        TopicSpec("/planning/risk_grid_health", String, health_refresh_stamp, True, 1.0, "p0", "P0 JSON health/debug", default_qos, parse_health_json=True),
        TopicSpec("/iap/rviz/risk_grid_health", MarkerArray, marker_array_stamp, True, 1.0, "p0-rviz", "P0 RViz health marker", default_qos),
        TopicSpec("/iap/rviz/predicted_pl_cloud", PointCloud2, header_stamp, True, 1.0, "p0-rviz", "P0 predicted PL cloud", sensor_qos),
        TopicSpec("/iap/rviz/risk_validity_cloud", PointCloud2, header_stamp, True, 1.0, "p0-rviz", "P0 validity cloud", sensor_qos),
        TopicSpec("/test_planner/desired/odom", Odometry, header_stamp, False, 1.0, "optional", "validator/desired path odom", default_qos),
        TopicSpec("/test_planner/so3_cmd", SO3Command, header_stamp, False, 5.0, "optional", "pos_cmd to simulator control chain", default_qos),
        TopicSpec("/map_generator/global_cloud", PointCloud2, header_stamp, False, 0.1, "optional", "planner global map input", sensor_qos),
        TopicSpec("/map_generator/local_cloud", PointCloud2, header_stamp, False, 0.1, "optional", "planner local map output", sensor_qos),
        TopicSpec("/ublox_driver/range_meas", GnssMeasMsg, no_stamp, False, 1.0, "gnss", "GNSS epoch source", default_qos, stamp_required=False),
        TopicSpec("/ublox_driver/ephem", GnssEphemMsg, no_stamp, False, 0.01, "gnss", "GPS/GAL/BDS ephemeris", default_qos, stamp_required=False),
        TopicSpec("/ublox_driver/glo_ephem", GnssGloEphemMsg, no_stamp, False, 0.01, "gnss", "GLONASS ephemeris", default_qos, stamp_required=False),
        TopicSpec("/ublox_driver/receiver_lla", NavSatFix, header_stamp, False, 0.1, "gnss", "P0 origin input", default_qos),
        TopicSpec("/ublox_driver/iono_params", GnssIonosphereParameter, header_stamp, False, 0.01, "gnss", "ionosphere parameters", default_qos),
    ]


def evaluate_topic(stats: TopicStats, latest_required_stamp: Optional[float], include_optional: bool) -> tuple[str, list[str]]:
    spec = stats.spec
    reasons: list[str] = []
    affects_result = spec.required or include_optional
    status = "PASS" if spec.required else "INFO"
    if stats.count == 0:
        if spec.required:
            return "FAIL", ["no messages received"]
        return ("FAIL" if affects_result else "INFO"), ["optional topic not observed"]

    mean_hz = stats.mean_hz()
    if mean_hz is None:
        reasons.append("not enough messages to estimate frequency")
        if spec.required:
            status = "WARN"
    elif mean_hz < spec.min_hz:
        reasons.append(f"mean frequency {mean_hz:.2f} Hz < expected {spec.min_hz:.2f} Hz")
        status = "FAIL" if affects_result else "INFO"

    if spec.stamp_required:
        if not stats.stamp_values_s:
            reasons.append("no valid header/derived stamp")
            status = "FAIL" if spec.required else ("WARN" if include_optional else "INFO")
        if stats.invalid_stamp_count > 0:
            reasons.append(f"{stats.invalid_stamp_count} invalid stamp samples")
            if spec.required:
                status = "FAIL"
        if stats.domain_counts["wall-or-other"] > 0:
            reasons.append(f"{stats.domain_counts['wall-or-other']} wall-or-other stamp samples")
            if spec.required or spec.name.startswith("/iap/rviz/"):
                status = "FAIL"
            elif include_optional:
                status = "WARN"
        if stats.stuck():
            reasons.append("last 3 valid stamps did not advance")
            if spec.required:
                status = "WARN" if status != "FAIL" else status
    else:
        reasons.append("message has no ROS header stamp; presence/frequency only")

    last_stamp = stats.last_stamp()
    latest_domain = stamp_domain(latest_required_stamp)
    if (
        latest_required_stamp is not None
        and last_stamp is not None
        and spec.required
        and spec.name in ("/iap/rviz/predicted_pl_cloud", "/iap/rviz/risk_validity_cloud")
        and stats.latest_domain() == latest_domain
    ):
        age = latest_required_stamp - last_stamp
        if age > 1.0:
            reasons.append(f"P0 cloud age_to_latest {age:.3f}s > 1.0s")
            if age > 2.0:
                status = "FAIL"
            elif status == "PASS":
                status = "WARN"

    if (
        latest_required_stamp is not None
        and last_stamp is not None
        and spec.name == "/sim/drone_0/lidar_body"
        and stats.latest_domain() == latest_domain
    ):
        age = latest_required_stamp - last_stamp
        if age > LIDAR_BODY_FRESHNESS_LIMIT_S:
            reasons.append(
                f"stale relative to latest sim required stamp {age:.3f}s > "
                f"{LIDAR_BODY_FRESHNESS_LIMIT_S:.1f}s"
            )
            status = "FAIL" if affects_result else "WARN"

    if not reasons:
        reasons.append("ok")
    return status, reasons


def summarize_p0_health(stats: TopicStats) -> dict[str, Any]:
    records = stats.health_records
    if not records:
        return {"count": 0}
    ready_count = sum(1 for r in records if r.get("ready") is True)
    stale_count = sum(1 for r in records if r.get("stale") is True)
    reasons = Counter(str(r.get("reason", "")) for r in records)
    valid_values = [float(r["valid_ratio"]) for r in records if isinstance(r.get("valid_ratio"), (int, float))]
    unknown_values = [float(r["unknown_ratio"]) for r in records if isinstance(r.get("unknown_ratio"), (int, float))]
    elapsed_values = [float(r["refresh_elapsed_ms"]) for r in records if isinstance(r.get("refresh_elapsed_ms"), (int, float))]
    refresh_stamps = [float(r["refresh_stamp_s"]) for r in records if isinstance(r.get("refresh_stamp_s"), (int, float))]
    grid_stamps = [float(r["last_grid_stamp_s"]) for r in records if isinstance(r.get("last_grid_stamp_s"), (int, float))]
    summary = {
        "count": len(records),
        "ready_ratio": ready_count / len(records),
        "stale_ratio": stale_count / len(records),
        "mean_valid_ratio": sum(valid_values) / len(valid_values) if valid_values else None,
        "mean_unknown_ratio": sum(unknown_values) / len(unknown_values) if unknown_values else None,
        "max_refresh_elapsed_ms": max(elapsed_values) if elapsed_values else None,
        "first_refresh_stamp_s": refresh_stamps[0] if refresh_stamps else None,
        "last_refresh_stamp_s": refresh_stamps[-1] if refresh_stamps else None,
        "last_grid_stamp_s": grid_stamps[-1] if grid_stamps else None,
        "reason_counts": reasons,
    }
    for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
        values = [int(r.get(field, 0) or 0) for r in records]
        summary[f"max_{field}"] = max(values) if values else None
        summary[f"last_{field}"] = values[-1] if values else None
    for field in PREDICTOR_LIDAR_INPUT_FIELDS:
        if field.endswith("_reason"):
            values = [str(r.get(field, "")) for r in records]
            summary[f"last_{field}"] = values[-1] if values else ""
            continue
        values = [int(r.get(field, 0) or 0) for r in records]
        summary[f"max_{field}"] = max(values) if values else None
        summary[f"last_{field}"] = values[-1] if values else None
    dominant_reasons = [
        str(r.get("dominant_unknown_reason", "")).strip()
        for r in records
        if str(r.get("dominant_unknown_reason", "")).strip()
    ]
    dominant_counts = [
        int(r.get("dominant_unknown_count", 0) or 0) for r in records
    ]
    summary["dominant_unknown_reason_counts"] = Counter(dominant_reasons)
    summary["last_dominant_unknown_reason"] = (
        dominant_reasons[-1] if dominant_reasons else ""
    )
    summary["max_dominant_unknown_count"] = (
        max(dominant_counts) if dominant_counts else None
    )
    summary["last_dominant_unknown_count"] = (
        dominant_counts[-1] if dominant_counts else None
    )
    return summary


class StampMonitor(Node):
    def __init__(self, args: argparse.Namespace, specs: list[TopicSpec]):
        super().__init__("iap_stamp_monitor")
        self.args = args
        self.specs = specs
        self.start_epoch_s = time.time()
        self.start_mono_s = time.monotonic()
        self.finished = False
        self.report_path: Optional[Path] = None
        self.overall_status = "UNKNOWN"
        self.fail_count = 0
        self.warn_count = 0
        self.raw_log: list[str] = []
        self.stats = {spec.name: TopicStats(spec) for spec in specs}
        for spec in specs:
            self.create_subscription(
                spec.msg_type,
                spec.name,
                lambda msg, topic=spec.name: self.on_msg(topic, msg),
                spec.qos,
            )
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f"monitoring {len(specs)} topics for {args.duration_s:.1f}s"
        )

    def on_msg(self, topic: str, msg: Any) -> None:
        stats = self.stats[topic]
        try:
            stamp = stamp_s(stats.spec.stamp_getter(msg))
        except Exception as exc:
            stamp = None
            stats.raw_events.append(f"stamp_getter_error: {exc}")
        stats.record(msg, stamp, time.time(), time.monotonic())

    def on_timer(self) -> None:
        if self.finished:
            return
        elapsed_s = time.monotonic() - self.start_mono_s
        if self.args.print_live:
            line = self.live_summary(elapsed_s)
            self.raw_log.append(line)
            print(line)
        if elapsed_s >= self.args.duration_s:
            self.finished = True
            self.report_path, self.overall_status, self.fail_count, self.warn_count = self.write_report()
            print(f"\nreport: {self.report_path}")
            print(
                f"overall: {self.overall_status} "
                f"fail={self.fail_count} warn={self.warn_count}"
            )
            rclpy.shutdown()

    def latest_required_stamp(self, domain: Optional[str] = SIM_DOMAIN) -> Optional[float]:
        values = [
            stats.last_stamp()
            for stats in self.stats.values()
            if stats.spec.required
            and stats.spec.stamp_required
            and stats.last_stamp() is not None
            and (domain is None or stats.latest_domain() == domain)
        ]
        return max(values) if values else None

    def required_wall_domain_topics(self) -> list[str]:
        return [
            name
            for name, stats in self.stats.items()
            if stats.spec.required
            and stats.spec.stamp_required
            and stats.latest_domain() == "wall-or-other"
        ]

    def live_summary(self, elapsed_s: float) -> str:
        latest = self.latest_required_stamp(SIM_DOMAIN)
        observed = sum(1 for stats in self.stats.values() if stats.count > 0)
        parts = [f"[{elapsed_s:6.1f}s] observed={observed}/{len(self.specs)}"]
        for name in (
            "/sim/drone_0/truth_odom",
            "/iap/integrity",
            "/iap/rviz/predicted_pl_cloud",
            "/iap/rviz/risk_grid_health",
        ):
            stats = self.stats[name]
            mean_hz = stats.mean_hz()
            age = None
            if latest is not None and stats.last_stamp() is not None and stats.latest_domain() == SIM_DOMAIN:
                age = latest - stats.last_stamp()
            parts.append(
                f"{name.split('/')[-1]} count={stats.count} "
                f"hz={fmt_float(mean_hz, 1)} age={fmt_float(age, 2)}"
            )
        return " | ".join(parts)

    def write_report(self) -> tuple[Path, str, int, int]:
        output_dir = Path(self.args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        prefix = "iap_topic_monitor" if self.args.name == "auto" else safe_name(self.args.name)
        report_path = output_dir / f"{prefix}_{stamp}.md"
        latest = self.latest_required_stamp(SIM_DOMAIN)
        evaluations = {
            name: evaluate_topic(stats, latest, self.args.include_optional)
            for name, stats in self.stats.items()
        }
        required_statuses = [
            evaluations[name][0]
            for name, stats in self.stats.items()
            if stats.spec.required
        ]
        fail_count = sum(1 for status in required_statuses if status == "FAIL")
        warn_count = sum(1 for status in required_statuses if status == "WARN")

        stamped_required = [
            stats.last_stamp()
            for stats in self.stats.values()
            if stats.spec.required
            and stats.spec.stamp_required
            and stats.last_stamp() is not None
            and stats.latest_domain() == SIM_DOMAIN
        ]
        latest_span = None
        if stamped_required:
            latest_span = max(stamped_required) - min(stamped_required)
            if latest_span > 2.0:
                fail_count += 1
            elif latest_span > 1.0:
                warn_count += 1
        else:
            warn_count += 1

        overall = "FAIL" if fail_count > 0 else ("WARN" if warn_count > 0 else "PASS")
        report_path.write_text(
            self.render_report(evaluations, overall, fail_count, warn_count, latest, latest_span),
            encoding="utf-8",
        )
        return report_path, overall, fail_count, warn_count

    def render_report(
        self,
        evaluations: dict[str, tuple[str, list[str]]],
        overall: str,
        fail_count: int,
        warn_count: int,
        latest_stamp: Optional[float],
        latest_span: Optional[float],
    ) -> str:
        lines: list[str] = []
        duration = time.monotonic() - self.start_mono_s
        lines.append("# IAP Topic Stamp/Frequency Report")
        lines.append("")
        lines.append("## Run Info")
        lines.append(f"- start_utc: `{fmt_time(self.start_epoch_s)}`")
        lines.append(f"- duration_s: `{duration:.3f}`")
        lines.append(f"- overall: `{overall}`")
        lines.append(f"- required_fail_count: `{fail_count}`")
        lines.append(f"- required_warn_count: `{warn_count}`")
        lines.append(f"- latest_required_stamp_domain: `{SIM_DOMAIN}`")
        lines.append(f"- latest_required_stamp_s: `{fmt_float(latest_stamp, 6)}`")
        lines.append(f"- required_latest_stamp_span_s: `{fmt_float(latest_span, 3)}`")
        wall_topics = self.required_wall_domain_topics()
        lines.append(
            "- required_wall_or_other_topics: `"
            + (", ".join(wall_topics) if wall_topics else "none")
            + "`"
        )
        lines.append("")

        lines.extend(self.render_summary(evaluations, latest_span))
        lines.extend(self.render_presence_table(evaluations))
        lines.extend(self.render_frequency_table(evaluations))
        lines.extend(self.render_domain_table(evaluations))
        lines.extend(self.render_wall_domain_table())
        lines.extend(self.render_lag_table(evaluations, latest_stamp))
        lines.extend(self.render_p0_health())
        lines.extend(self.render_raw_log())
        return "\n".join(lines) + "\n"

    def render_summary(
        self,
        evaluations: dict[str, tuple[str, list[str]]],
        latest_span: Optional[float],
    ) -> list[str]:
        lines = ["## Summary", "| Check | Result | Notes |", "|---|---:|---|"]
        for name, stats in self.stats.items():
            if not stats.spec.required:
                continue
            status, reasons = evaluations[name]
            lines.append(
                f"| `{md_escape(name)}` | `{status}` | {md_escape('; '.join(reasons))} |"
            )
        span_status = "PASS"
        span_note = "ok"
        if latest_span is None:
            span_status = "WARN"
            span_note = f"no {SIM_DOMAIN} required stamped topic observed"
        elif latest_span > 2.0:
            span_status = "FAIL"
            span_note = f"{SIM_DOMAIN} latest required stamp span {latest_span:.3f}s > 2.0s"
        elif latest_span > 1.0:
            span_status = "WARN"
            span_note = f"{SIM_DOMAIN} latest required stamp span {latest_span:.3f}s > 1.0s"
        lines.append(f"| `{SIM_DOMAIN} required stamp span` | `{span_status}` | {span_note} |")
        lines.append("")
        return lines

    def render_wall_domain_table(self) -> list[str]:
        wall_topics = self.required_wall_domain_topics()
        lines = ["## Wall/Other Required Topics"]
        if not wall_topics:
            lines.append("No required stamped topic ended in the `wall-or-other` domain.")
            lines.append("")
            return lines
        lines.extend([
            "| Topic | Last Stamp | Wall/Other Samples | Result |",
            "|---|---:|---:|---:|",
        ])
        for name in wall_topics:
            stats = self.stats[name]
            lines.append(
                f"| `{md_escape(name)}` | {fmt_float(stats.last_stamp(), 6)} | "
                f"{stats.domain_counts['wall-or-other']} | `FAIL` |"
            )
        lines.append("")
        return lines

    def render_presence_table(self, evaluations: dict[str, tuple[str, list[str]]]) -> list[str]:
        lines = [
            "## Topic Presence",
            "| Topic | Required | Category | Count | First Stamp | Last Stamp | First Receive | Last Receive | Result |",
            "|---|---:|---|---:|---:|---:|---|---|---:|",
        ]
        for name, stats in self.stats.items():
            status, _ = evaluations[name]
            lines.append(
                f"| `{md_escape(name)}` | `{stats.spec.required}` | {stats.spec.category} | "
                f"{stats.count} | {fmt_float(stats.first_stamp(), 6)} | "
                f"{fmt_float(stats.last_stamp(), 6)} | "
                f"{fmt_time(stats.receive_epoch_s[0] if stats.receive_epoch_s else None)} | "
                f"{fmt_time(stats.receive_epoch_s[-1] if stats.receive_epoch_s else None)} | `{status}` |"
            )
        lines.append("")
        return lines

    def render_frequency_table(self, evaluations: dict[str, tuple[str, list[str]]]) -> list[str]:
        lines = [
            "## Frequency",
            "| Topic | Mean Hz | Min Hz | Max Hz | Max Gap s | Expected Min Hz | Result | Notes |",
            "|---|---:|---:|---:|---:|---:|---:|---|",
        ]
        for name, stats in self.stats.items():
            status, reasons = evaluations[name]
            lines.append(
                f"| `{md_escape(name)}` | {fmt_float(stats.mean_hz(), 3)} | "
                f"{fmt_float(stats.min_hz(), 3)} | {fmt_float(stats.max_hz(), 3)} | "
                f"{fmt_float(stats.max_gap_s(), 3)} | {stats.spec.min_hz:.3f} | "
                f"`{status}` | {md_escape('; '.join(reasons))} |"
            )
        lines.append("")
        return lines

    def render_domain_table(self, evaluations: dict[str, tuple[str, list[str]]]) -> list[str]:
        lines = [
            "## Timestamp Domain",
            "| Topic | Stamp Required | Latest Domain | Sim Samples | Wall/Other Samples | Missing Samples | Invalid Samples | Stuck | Result |",
            "|---|---:|---|---:|---:|---:|---:|---:|---:|",
        ]
        for name, stats in self.stats.items():
            status, _ = evaluations[name]
            lines.append(
                f"| `{md_escape(name)}` | `{stats.spec.stamp_required}` | {stats.latest_domain()} | "
                f"{stats.domain_counts['sim-2022']} | {stats.domain_counts['wall-or-other']} | "
                f"{stats.domain_counts['missing']} | {stats.invalid_stamp_count} | "
                f"`{stats.stuck()}` | `{status}` |"
            )
        lines.append("")
        return lines

    def render_lag_table(
        self,
        evaluations: dict[str, tuple[str, list[str]]],
        latest_stamp: Optional[float],
    ) -> list[str]:
        lines = [
            "## Cross-topic Lag",
            "| Topic | Latest Stamp | Age To Latest s | Result |",
            "|---|---:|---:|---:|",
        ]
        for name, stats in self.stats.items():
            status, _ = evaluations[name]
            last = stats.last_stamp()
            age = (
                latest_stamp - last
                if latest_stamp is not None
                and last is not None
                and stats.latest_domain() == stamp_domain(latest_stamp)
                else None
            )
            lines.append(
                f"| `{md_escape(name)}` | {fmt_float(last, 6)} | "
                f"{fmt_float(age, 3)} | `{status}` |"
            )
        lines.append("")
        return lines

    def render_p0_health(self) -> list[str]:
        stats = self.stats.get("/planning/risk_grid_health")
        summary = summarize_p0_health(stats) if stats is not None else {"count": 0}
        lines = ["## P0 Health"]
        if summary.get("count", 0) == 0:
            lines.append("No parseable `/planning/risk_grid_health` JSON messages were observed.")
            lines.append("")
            return lines
        lines.append("| Metric | Value |")
        lines.append("|---|---:|")
        for key in (
            "count",
            "ready_ratio",
            "stale_ratio",
            "mean_valid_ratio",
            "mean_unknown_ratio",
            "max_refresh_elapsed_ms",
            "first_refresh_stamp_s",
            "last_refresh_stamp_s",
            "last_grid_stamp_s",
        ):
            lines.append(f"| `{key}` | `{fmt_float(summary.get(key), 6)}` |")
        lines.append("")
        lines.append("| Predictor Source Counter | Latest | Max |")
        lines.append("|---|---:|---:|")
        for field in PREDICTOR_SOURCE_COUNTER_FIELDS:
            lines.append(
                f"| `{field}` | `{fmt_float(summary.get(f'last_{field}'), 0)}` | "
                f"`{fmt_float(summary.get(f'max_{field}'), 0)}` |"
            )
        lines.append("")
        lines.append("| Predictor LiDAR Input | Latest | Max |")
        lines.append("|---|---:|---:|")
        for field in PREDICTOR_LIDAR_INPUT_FIELDS:
            if field.endswith("_reason"):
                lines.append(
                    f"| `{field}` | `{md_escape(summary.get(f'last_{field}', ''))}` |  |"
                )
            else:
                lines.append(
                    f"| `{field}` | `{fmt_float(summary.get(f'last_{field}'), 0)}` | "
                    f"`{fmt_float(summary.get(f'max_{field}'), 0)}` |"
                )
        lines.append("")
        lines.append("| Dominant Unknown | Latest | Max |")
        lines.append("|---|---:|---:|")
        for field in P0_UNKNOWN_REASON_FIELDS:
            if field.endswith("_reason"):
                lines.append(
                    f"| `{field}` | "
                    f"`{md_escape(summary.get(f'last_{field}', ''))}` |  |"
                )
            else:
                lines.append(
                    f"| `{field}` | "
                    f"`{fmt_float(summary.get(f'last_{field}'), 0)}` | "
                    f"`{fmt_float(summary.get(f'max_{field}'), 0)}` |"
                )
        reasons = summary.get("reason_counts", Counter())
        lines.append("")
        lines.append("| Reason | Count |")
        lines.append("|---|---:|")
        for reason, count in reasons.most_common():
            lines.append(f"| `{md_escape(reason)}` | {count} |")
        max_elapsed = summary.get("max_refresh_elapsed_ms")
        if isinstance(max_elapsed, (int, float)) and max_elapsed > 500.0:
            lines.append("")
            lines.append(
                f"WARNING: max `refresh_elapsed_ms` is {max_elapsed:.3f} ms, "
                "which is above the 500 ms P0 refresh-period budget."
            )
        lines.append("")
        return lines

    def render_raw_log(self) -> list[str]:
        lines = ["## Raw Log", "```text"]
        if self.raw_log:
            lines.extend(self.raw_log)
        else:
            lines.append("live logging disabled; no raw live summary captured")
        for stats in self.stats.values():
            for event in stats.raw_events[:20]:
                lines.append(f"{stats.spec.name}: {event}")
        lines.append("```")
        lines.append("")
        return lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor IAP topic stamps/frequencies and write a Markdown report."
    )
    parser.add_argument("--duration-s", type=float, default=60.0)
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR))
    parser.add_argument("--name", default="auto")
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--include-optional", action="store_true")
    parser.add_argument("--print-live", action="store_true")
    args = parser.parse_args()
    if args.duration_s <= 0.0:
        parser.error("--duration-s must be positive")
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = StampMonitor(args, build_topic_specs())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finished:
            node.report_path, node.overall_status, node.fail_count, node.warn_count = node.write_report()
            print(f"\nreport: {node.report_path}")
            print(
                f"overall: {node.overall_status} "
                f"fail={node.fail_count} warn={node.warn_count}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if args.strict and node.fail_count > 0:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
