#!/usr/bin/env python3
"""Replay one captured planner GridMap failure into RViz.

The saved bytes are the planner's raw, inflated and observed voxel flags at a
single occupancy generation. RViz markers are diagnostics, never map input.
"""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


def load_snapshot(directory):
    directory = Path(directory).resolve()
    meta = json.loads((directory / "snapshot.json").read_text())
    dims = tuple(int(v) for v in meta["dimensions"])
    flags = np.fromfile(directory / meta["cell_flags_file"], dtype=np.uint8)
    if flags.size != np.prod(dims):
        raise ValueError("cells.bin length does not match GridMap dimensions")
    return directory, meta, flags.reshape(dims)


def roi(meta, dims):
    origin = np.array(meta["origin_m"], dtype=float)
    resolution = float(meta["resolution_m"])
    a = np.array(meta["failure_position_m"], dtype=float)
    b = np.array(meta["other_endpoint_m"], dtype=float)
    low = np.floor((np.minimum(a, b) - 5.0 - origin) / resolution).astype(int)
    high = np.ceil((np.maximum(a, b) + 5.0 - origin) / resolution).astype(int)
    return tuple(slice(max(0, low[i]), min(dims[i], high[i])) for i in range(3))


def centers(indices, meta):
    return (indices.astype(np.float32) + 0.5) * float(meta["resolution_m"]) + \
        np.asarray(meta["origin_m"], dtype=np.float32)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path, help="candidate or search snapshot directory")
    parser.add_argument("--slice-z", type=float, help="observed/unknown slice height in metres")
    args = parser.parse_args()
    directory, meta, flags = load_snapshot(args.snapshot)
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import Header
    from sensor_msgs_py import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Point

    rclpy.init()
    node = rclpy.create_node("iap_failure_map_replay")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    names = ("raw", "inflated", "observed", "unknown")
    publishers = {name: node.create_publisher(
        PointCloud2, "/iap/failure_map/" + name, qos) for name in names}
    marker_pub = node.create_publisher(MarkerArray, "/iap/failure_map/markers", qos)
    bounds = roi(meta, flags.shape)
    view = flags[bounds]
    offset = np.array([bounds[i].start for i in range(3)], dtype=int)
    z = args.slice_z if args.slice_z is not None else meta["failure_position_m"][2]
    z_index = int(np.clip(np.floor((z - meta["origin_m"][2]) /
                                     meta["resolution_m"]), 0, flags.shape[2] - 1))

    def cloud(mask, header, base_offset):
        indices = np.argwhere(mask) + base_offset
        return point_cloud2.create_cloud_xyz32(header, centers(indices, meta).tolist())

    def publish():
        header = Header()
        header.frame_id = meta["frame_id"]
        header.stamp = node.get_clock().now().to_msg()
        publishers["raw"].publish(cloud((view & 1) != 0, header, offset))
        publishers["inflated"].publish(cloud((view & 2) != 0, header, offset))
        slice_flags = flags[bounds[0], bounds[1], z_index]
        slice_offset = np.array([bounds[0].start, bounds[1].start, z_index])
        visible_free = ((slice_flags & 4) != 0) & ((slice_flags & 2) == 0)
        publishers["observed"].publish(cloud(
            visible_free[:, :, None], header, slice_offset))
        publishers["unknown"].publish(cloud(
            ((slice_flags & 7) == 0)[:, :, None], header, slice_offset))

        markers = MarkerArray()
        for index, (name, position, color, radius) in enumerate((
            ("failure", meta["failure_position_m"], (1.0, 0.1, 0.05), 0.35),
            ("other", meta["other_endpoint_m"], (0.0, 0.9, 0.9), 0.28),
            ("nearest_raw", meta["nearest_raw_center_m"], (1.0, 0.8, 0.0), 0.22),
        )):
            if any(value is None for value in position):
                continue
            marker = Marker()
            marker.header = header
            marker.ns = "failure_points"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = radius
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            markers.markers.append(marker)
        if meta["kind"] != "candidate":
            wire = Marker()
            wire.header = header
            wire.ns = "search_pool"
            wire.id = 20
            wire.type = Marker.LINE_LIST
            wire.action = Marker.ADD
            wire.pose.orientation.w = 1.0
            wire.scale.x = 0.035
            wire.color.r = wire.color.g = wire.color.b = 0.95
            wire.color.a = 0.7
            if meta.get("schema_version") in ("iap_gridmap_failure_v2", "iap_gridmap_failure_v3"):
                middle = np.asarray(meta["search_pool_center_m"], dtype=float)
                dimensions = np.asarray(meta["search_pool_dimensions"], dtype=float)
                step = float(meta["search_step_size_m"])
                lo = middle - np.floor(dimensions / 2.0) * step
                hi = middle + (dimensions - np.floor(dimensions / 2.0) - 1) * step
            else:
                middle = (np.asarray(meta["failure_position_m"]) +
                          np.asarray(meta["other_endpoint_m"])) / 2.0
                lo, hi = middle - 5.0, middle + 4.9
            corners = [np.array([x, y, z]) for x in (lo[0], hi[0])
                       for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
            for i in range(8):
                for j in range(i + 1, 8):
                    if np.count_nonzero(corners[i] != corners[j]) == 1:
                        for corner in (corners[i], corners[j]):
                            wire.points.append(Point(
                                x=float(corner[0]), y=float(corner[1]),
                                z=float(corner[2])))
            markers.markers.append(wire)
        caption = Marker()
        caption.header = header
        caption.ns = "failure_summary"
        caption.id = 21
        caption.type = Marker.TEXT_VIEW_FACING
        caption.action = Marker.ADD
        caption.pose.orientation.w = 1.0
        caption.pose.position.x = meta["failure_position_m"][0]
        caption.pose.position.y = meta["failure_position_m"][1]
        caption.pose.position.z = meta["failure_position_m"][2] + 0.8
        caption.scale.z = 0.25
        caption.color.r = caption.color.g = caption.color.b = 1.0
        caption.color.a = 1.0
        caption.text = (f"gen {meta['generation']} {meta['execution_reason']} "
                        f"required={meta['required_clearance_m']}m "
                        f"nearest={meta['nearest_raw_center_distance_m']}m")
        markers.markers.append(caption)
        with (directory / meta["risk_samples_file"]).open() as stream:
            samples = list(csv.DictReader(stream))
        if samples:
            risk = Marker()
            risk.header = header
            risk.ns = "queried_pl"
            risk.id = 10
            risk.type = Marker.CUBE_LIST
            risk.action = Marker.ADD
            risk.pose.orientation.w = 1.0
            risk.scale.x = risk.scale.y = meta["resolution_m"] * 1.4
            risk.scale.z = 0.035
            yz = flags.shape[1] * flags.shape[2]
            for sample in samples[::max(1, len(samples) // 5000)]:
                address = int(sample["address"])
                xyz = centers(np.array([[address // yz,
                    (address % yz) // flags.shape[2],
                    address % flags.shape[2]]]), meta)[0]
                if not all(bounds[i].start <= (xyz[i] - meta["origin_m"][i]) /
                           meta["resolution_m"] < bounds[i].stop for i in range(3)):
                    continue
                point = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
                risk.points.append(point)
                from std_msgs.msg import ColorRGBA
                hpl = float(sample["hpl_m"])
                if not math.isfinite(hpl) or int(sample["status"]) != 1:
                    risk.colors.append(ColorRGBA(r=0.65, g=0.15, b=0.8, a=0.85))
                else:
                    amount = float(np.clip((hpl - 0.25) / 0.40, 0.0, 1.0))
                    risk.colors.append(ColorRGBA(
                        r=amount, g=0.2, b=1.0 - amount, a=0.85))
            markers.markers.append(risk)
        marker_pub.publish(markers)

    node.get_logger().info(
        f"replaying {directory} generation={meta['generation']} "
        f"failure={meta['execution_reason']} at z={z_index}")
    publish()
    node.create_timer(2.0, publish)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
