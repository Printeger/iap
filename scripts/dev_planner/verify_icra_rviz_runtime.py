#!/usr/bin/env python3
"""Verify the final RViz ROS endpoints and the geometry they receive.

This probe is deliberately read-only.  It observes the live ROS graph and
messages after RViz has settled; startup log warnings are retained only as
diagnostics and never determine the verdict.
"""

import argparse
import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Any

import yaml


RVIZ_NODE = "/test_planner_rviz"
TARGET_TOPICS = {
    "/iap/local_map/current_hits_map": {
        "display": "GLIM Registered Current LiDAR",
        "kind": "cloud",
        "payload_required": True,
        "best_effort": True,
    },
    "/iap/rviz/predicted_pl_cloud": {
        "display": "Predicted PL Cloud",
        "kind": "cloud",
        "payload_required": True,
        "best_effort": True,
    },
    "/iap/rviz/planning_lattice_geometry": {
        "display": "Planning Lattice Geometry",
        "kind": "markers",
        "payload_required": True,
    },
    "/iap/rviz/p4_astar_guides": {
        "display": "P4 AStar Guides",
        "kind": "markers",
        "payload_required": True,
    },
    "/iap/rviz/p4_topology_channels": {
        "display": "P4 Topology Channels",
        "kind": "markers",
        "payload_required": True,
    },
    "/iap/rviz/current_traj_integrity_colored": {
        "display": "Current Trajectory Integrity",
        "kind": "markers",
        "payload_required": False,
    },
}


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def config_identity(
        source_path: Path, install_path: Path,
        loaded_path: Path | None) -> dict[str, Any]:
    # Preserve the lexical -d path.  A symlink-install intentionally points
    # the installed file at the source tree, but RViz must still be launched
    # with the installed package path.
    source = Path(os.path.abspath(source_path))
    installed = Path(os.path.abspath(install_path))
    loaded = (Path(os.path.abspath(loaded_path))
              if loaded_path is not None else None)
    source_hash = _sha256(source)
    install_hash = _sha256(installed)
    loaded_hash = _sha256(loaded) if loaded is not None else None
    return {
        "source_path": str(source),
        "source_sha256": source_hash,
        "install_path": str(installed),
        "install_sha256": install_hash,
        "rviz_config_path": str(loaded) if loaded is not None else None,
        "rviz_config_sha256": loaded_hash,
        "source_install_match": (
            source_hash is not None and source_hash == install_hash),
        "loaded_install_config": (
            loaded is not None and loaded == installed
            and loaded_hash is not None and loaded_hash == install_hash),
    }


def _topic_value(display: dict[str, Any]) -> str:
    topic = display.get("Topic", {})
    return str(topic.get("Value", "")) if isinstance(topic, dict) else ""


def validate_rviz_config(path: Path) -> dict[str, Any]:
    failures = []
    try:
        root = yaml.safe_load(path.read_text())
        manager = root["Visualization Manager"]
        displays = manager["Displays"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        return {
            "fixed_frame": None,
            "displays": {},
            "failures": [f"rviz_config_parse_failed:{error}"],
        }

    fixed_frame = manager.get("Global Options", {}).get("Fixed Frame")
    if fixed_frame != "map":
        failures.append("rviz_fixed_frame_not_map")
    by_topic: dict[str, list[dict[str, Any]]] = {}
    for display in displays:
        topic = _topic_value(display)
        if topic:
            by_topic.setdefault(topic, []).append(display)
    checked = {}
    for topic, specification in TARGET_TOPICS.items():
        matches = by_topic.get(topic, [])
        if len(matches) != 1:
            failures.append(f"rviz_display_count:{topic}:{len(matches)}")
            continue
        display = matches[0]
        topic_config = display.get("Topic", {})
        enabled = display.get("Enabled") is True and display.get("Value") is True
        if not enabled:
            failures.append(f"rviz_display_disabled:{topic}")
        if specification.get("best_effort"):
            if topic_config.get("Reliability Policy") != "Best Effort":
                failures.append(f"rviz_reliability_not_best_effort:{topic}")
            if topic_config.get("Durability Policy") != "Volatile":
                failures.append(f"rviz_durability_not_volatile:{topic}")
        if specification["kind"] == "cloud":
            try:
                if float(display.get("Alpha", 0.0)) <= 0.0:
                    failures.append(f"rviz_alpha_not_visible:{topic}")
            except (TypeError, ValueError):
                failures.append(f"rviz_alpha_invalid:{topic}")
        checked[topic] = {
            "name": display.get("Name"),
            "enabled": enabled,
            "class": display.get("Class"),
            "qos": {
                "reliability": topic_config.get("Reliability Policy"),
                "durability": topic_config.get("Durability Policy"),
                "depth": topic_config.get("Depth"),
            },
        }
    return {
        "fixed_frame": fixed_frame,
        "displays": checked,
        "failures": failures,
    }


def _policy_name(value: Any) -> str:
    name = getattr(value, "name", None)
    if name:
        return str(name).upper()
    text = str(value).upper()
    return text.rsplit(".", 1)[-1]


def qos_profile_dict(profile: Any) -> dict[str, Any]:
    return {
        "reliability": _policy_name(profile.reliability),
        "durability": _policy_name(profile.durability),
        "history": _policy_name(profile.history),
        "depth": int(profile.depth),
    }


def qos_compatible(publisher: dict[str, Any], subscriber: dict[str, Any]) -> bool:
    offered_reliability = str(publisher.get("reliability", "")).upper()
    requested_reliability = str(subscriber.get("reliability", "")).upper()
    if (requested_reliability == "RELIABLE"
            and offered_reliability != "RELIABLE"):
        return False
    offered_durability = str(publisher.get("durability", "")).upper()
    requested_durability = str(subscriber.get("durability", "")).upper()
    if (requested_durability == "TRANSIENT_LOCAL"
            and offered_durability != "TRANSIENT_LOCAL"):
        return False
    return True


def pointcloud_metrics(message: Any) -> dict[str, Any]:
    return {
        "frame_id": str(message.header.frame_id),
        "point_count": int(message.width) * int(message.height),
    }


def _finite_point(value: Any) -> bool:
    return all(math.isfinite(float(getattr(value, axis)))
               for axis in ("x", "y", "z"))


def _marker_has_geometry(marker: Any) -> bool:
    points = list(getattr(marker, "points", []))
    marker_type = int(getattr(marker, "type", -1))
    minimum_points = 2 if marker_type in (4, 5) else 1
    if len(points) >= minimum_points and all(_finite_point(p) for p in points):
        return True
    # Primitive, mesh and text markers carry their geometry in pose/scale.
    if marker_type in (0, 1, 2, 3, 9, 10, 11):
        position = getattr(getattr(marker, "pose", None), "position", None)
        scale = getattr(marker, "scale", None)
        return (position is not None and scale is not None
                and _finite_point(position) and _finite_point(scale)
                and any(float(getattr(scale, axis)) > 0.0
                        for axis in ("x", "y", "z")))
    return False


def marker_array_metrics(message: Any) -> dict[str, Any]:
    markers = list(message.markers)
    active = [marker for marker in markers
              if int(getattr(marker, "action", -1)) == 0]
    geometry = [marker for marker in active if _marker_has_geometry(marker)]
    paths = [marker for marker in geometry
             if int(getattr(marker, "type", -1)) in (4, 5)]
    frame_id = next((str(marker.header.frame_id) for marker in markers
                     if str(marker.header.frame_id)), "")
    return {
        "frame_id": frame_id,
        "marker_count": len(markers),
        "non_delete_marker_count": len(active),
        "geometry_marker_count": len(geometry),
        "path_marker_count": len(paths),
    }


def _full_node_name(endpoint: Any) -> str:
    namespace = str(endpoint.node_namespace or "/")
    name = str(endpoint.node_name)
    return f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"


def endpoint_dict(endpoint: Any) -> dict[str, Any]:
    return {
        "node": _full_node_name(endpoint),
        "qos": qos_profile_dict(endpoint.qos_profile),
    }


def rviz_config_from_processes() -> Path | None:
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / "cmdline").read_bytes().split(b"\0")
            args = [part.decode(errors="replace") for part in argv if part]
        except (OSError, PermissionError):
            continue
        if not any("rviz2" in arg for arg in args):
            continue
        if not any("test_planner_rviz" in arg for arg in args):
            continue
        for index, argument in enumerate(args[:-1]):
            if argument == "-d":
                return Path(args[index + 1])
    return None


def empty_report() -> dict[str, Any]:
    return {
        "schema_version": "icra_rviz_runtime_v1",
        "result": "RUNNING",
        "startup_qos_warning_count": 0,
        "p0_ready": False,
        "risk_generation_max": 0,
        "p4_candidate_ready": False,
        "topics": {},
        "config": {},
    }


def runtime_failures(report: dict[str, Any]) -> list[str]:
    failures = []
    config = report.get("config", {})
    if not config.get("source_install_match"):
        failures.append("rviz_source_install_config_mismatch")
    if not config.get("loaded_install_config"):
        failures.append("rviz_did_not_load_installed_test_config")
    failures.extend(config.get("validation_failures", []))
    for topic, specification in TARGET_TOPICS.items():
        evidence = report.get("topics", {}).get(topic, {})
        if not evidence.get("publisher_endpoints"):
            failures.append(f"publisher_missing:{topic}")
        if not evidence.get("rviz_subscriber_endpoints"):
            failures.append(f"rviz_subscriber_missing:{topic}")
        if not evidence.get("matched"):
            failures.append(f"qos_incompatible:{topic}")
        if (specification["payload_required"]
                and int(evidence.get("message_count", 0)) <= 0):
            failures.append(f"no_messages:{topic}")
        if (specification["payload_required"]
                and specification["kind"] == "cloud"
                and int(evidence.get("max_point_count", 0)) <= 0):
            failures.append(f"empty_pointcloud:{topic}")
        if (specification["payload_required"]
                and specification["kind"] == "markers"
                and int(evidence.get("max_geometry_markers", 0)) <= 0):
            failures.append(f"empty_marker_geometry:{topic}")
    if not report.get("p0_ready"):
        failures.append("p0_not_ready")
    if int(report.get("risk_generation_max", 0)) <= 0:
        failures.append("risk_generation_zero")
    if not report.get("p4_candidate_ready"):
        failures.append("p4_candidates_not_visible")
    required_settle = float(report.get("required_settle_s", 0.0))
    actual_settle = float(report.get("settled_after_last_first_data_s", 0.0))
    if actual_settle + 1.0e-6 < required_settle:
        failures.append("rviz_endpoint_not_settled")
    if not report.get("visual_proof", {}).get("captured"):
        failures.append("rviz_visual_proof_not_captured")
    return list(dict.fromkeys(failures))


def _warning_count(path: Path | None) -> int:
    if path is None or not path.is_file():
        return 0
    return sum("incompatible qos" in line.lower()
               for line in path.read_text(errors="replace").splitlines())


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def capture_visual_proof(path: Path) -> dict[str, Any]:
    try:
        from PIL import ImageGrab
        image = ImageGrab.grab(all_screens=True)
        path.parent.mkdir(parents=True, exist_ok=True)
        image.save(path, format="PNG")
    except Exception as error:  # pragma: no cover - depends on live X server
        return {
            "captured": False,
            "path": str(path.absolute()),
            "error": str(error),
        }
    return {
        "captured": path.is_file() and path.stat().st_size > 0,
        "path": str(path.absolute()),
        "sha256": _sha256(path),
        "width": int(image.width),
        "height": int(image.height),
        "bytes": int(path.stat().st_size),
    }


def run_probe(args: argparse.Namespace) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, QoSProfile, ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

    report = empty_report()
    report["required_settle_s"] = float(args.settle_seconds)
    report["started_unix_s"] = time.time()
    report["started_steady_s"] = time.monotonic()
    for topic, specification in TARGET_TOPICS.items():
        report["topics"][topic] = {
            "kind": specification["kind"],
            "payload_required": specification["payload_required"],
            "publisher_endpoints": [],
            "rviz_subscriber_endpoints": [],
            "matched": False,
            "first_message_unix_s": None,
            "first_message_steady_s": None,
            "message_count": 0,
            "frame_id": "",
            "max_point_count": 0,
            "max_marker_count": 0,
            "max_non_delete_markers": 0,
            "max_geometry_markers": 0,
            "max_path_markers": 0,
        }

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("verify_icra_rviz_runtime")
            marker_qos = QoSProfile(
                depth=50, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE)
            for topic, specification in TARGET_TOPICS.items():
                if specification["kind"] == "cloud":
                    self.create_subscription(
                        PointCloud2, topic,
                        lambda message, name=topic: self.cloud(name, message),
                        qos_profile_sensor_data)
                else:
                    self.create_subscription(
                        MarkerArray, topic,
                        lambda message, name=topic: self.markers(name, message),
                        marker_qos)
            health_qos = QoSProfile(
                depth=20, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(
                String, "/planning/risk_grid_health", self.health, health_qos)

        def _received(self, topic: str) -> dict[str, Any]:
            evidence = report["topics"][topic]
            evidence["message_count"] += 1
            if evidence["first_message_steady_s"] is None:
                evidence["first_message_steady_s"] = time.monotonic()
                evidence["first_message_unix_s"] = time.time()
            return evidence

        def cloud(self, topic: str, message: Any) -> None:
            evidence = self._received(topic)
            metrics = pointcloud_metrics(message)
            evidence["frame_id"] = metrics["frame_id"]
            evidence["max_point_count"] = max(
                evidence["max_point_count"], metrics["point_count"])

        def markers(self, topic: str, message: Any) -> None:
            evidence = self._received(topic)
            metrics = marker_array_metrics(message)
            evidence["frame_id"] = metrics["frame_id"]
            evidence["max_marker_count"] = max(
                evidence["max_marker_count"], metrics["marker_count"])
            evidence["max_non_delete_markers"] = max(
                evidence["max_non_delete_markers"],
                metrics["non_delete_marker_count"])
            evidence["max_geometry_markers"] = max(
                evidence["max_geometry_markers"],
                metrics["geometry_marker_count"])
            evidence["max_path_markers"] = max(
                evidence["max_path_markers"], metrics["path_marker_count"])

        def health(self, message: Any) -> None:
            try:
                payload = json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return
            report["p0_ready"] = report["p0_ready"] or bool(
                payload.get("ready", False))
            for key in ("generation_id", "result_generation_id"):
                try:
                    report["risk_generation_max"] = max(
                        report["risk_generation_max"], int(payload.get(key, 0)))
                except (TypeError, ValueError):
                    pass

        def sample_graph(self) -> None:
            for topic in TARGET_TOPICS:
                publishers = [endpoint_dict(endpoint) for endpoint in
                              self.get_publishers_info_by_topic(topic)]
                subscribers = [endpoint_dict(endpoint) for endpoint in
                               self.get_subscriptions_info_by_topic(topic)]
                rviz_subscribers = [endpoint for endpoint in subscribers
                                    if endpoint["node"] == args.rviz_node]
                evidence = report["topics"][topic]
                evidence["publisher_endpoints"] = publishers
                evidence["rviz_subscriber_endpoints"] = rviz_subscribers
                evidence["matched"] = any(
                    qos_compatible(publisher["qos"], subscriber["qos"])
                    for publisher in publishers
                    for subscriber in rviz_subscribers)

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + float(args.timeout)
    last_first_data = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.sample_graph()
            required = [report["topics"][topic]
                        for topic, specification in TARGET_TOPICS.items()
                        if specification["payload_required"]]
            first_times = [entry["first_message_steady_s"]
                           for entry in required]
            if all(value is not None for value in first_times):
                last_first_data = max(first_times)
                topology = report["topics"][
                    "/iap/rviz/p4_topology_channels"]
                guides = report["topics"]["/iap/rviz/p4_astar_guides"]
                live_candidate_ready = (
                    topology["max_path_markers"] >= 2
                    and guides["max_path_markers"] >= 2)
                if (report["p0_ready"]
                        and report["risk_generation_max"] > 0
                        and live_candidate_ready
                        and time.monotonic() - last_first_data
                        >= args.settle_seconds):
                    break
    finally:
        node.sample_graph()
        node.destroy_node()
        rclpy.shutdown()

    now = time.monotonic()
    report["settled_after_last_first_data_s"] = (
        max(0.0, now - last_first_data) if last_first_data is not None else 0.0)
    topology = report["topics"]["/iap/rviz/p4_topology_channels"]
    guides = report["topics"]["/iap/rviz/p4_astar_guides"]
    report["p4_candidate_ready"] = (
        topology["max_path_markers"] >= 2
        and guides["max_path_markers"] >= 2)
    loaded_path = rviz_config_from_processes()
    identity = config_identity(
        args.source_config, args.install_config, loaded_path)
    validation = validate_rviz_config(args.install_config)
    report["config"] = {
        **identity,
        "fixed_frame": validation["fixed_frame"],
        "displays": validation["displays"],
        "validation_failures": validation["failures"],
    }
    report["startup_qos_warning_count"] = _warning_count(args.startup_log)
    report["visual_proof"] = capture_visual_proof(args.visual_proof)
    report["visual_proof_path"] = report["visual_proof"]["path"]
    report["finished_unix_s"] = time.time()
    report["elapsed_s"] = now - report["started_steady_s"]
    report["failures"] = runtime_failures(report)
    report["result"] = "PASS" if not report["failures"] else "FAIL"
    _write_json(args.output, report)
    return 0 if report["result"] == "PASS" else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-config", type=Path, required=True)
    parser.add_argument("--install-config", type=Path, required=True)
    parser.add_argument("--startup-log", type=Path)
    parser.add_argument("--visual-proof", type=Path, required=True)
    parser.add_argument("--rviz-node", default=RVIZ_NODE)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--settle-seconds", type=float, default=2.0)
    args = parser.parse_args()
    if args.timeout <= args.settle_seconds or args.settle_seconds < 2.0:
        raise SystemExit("timeout must exceed a settle period of at least 2s")
    return run_probe(args)


if __name__ == "__main__":
    raise SystemExit(main())
