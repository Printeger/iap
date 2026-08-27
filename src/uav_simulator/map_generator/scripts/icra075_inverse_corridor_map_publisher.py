#!/usr/bin/env python3
"""Publish exact ICRA-075 V2 ordinary occupancy at the public map seam."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


VARIANTS = {"PRIMARY", "EXACT_MIRROR", "FLAT_NULL"}


def _axis(low: float, high: float, resolution: float):
    count = int(round((high - low) / resolution))
    return [low + resolution * index for index in range(count + 1)]


def _box(points, bounds, resolution):
    for x in _axis(*bounds["x"], resolution):
        for y in _axis(*bounds["y"], resolution):
            for z in _axis(*bounds["z"], resolution):
                points.append((x, y, z))


def _cylinder(points, x, y, radius, height, resolution):
    for px in _axis(x - radius, x + radius, resolution):
        for py in _axis(y - radius, y + radius, resolution):
            if (px - x) ** 2 + (py - y) ** 2 <= radius ** 2 + 1e-12:
                for pz in _axis(0.0, height, resolution):
                    points.append((px, py, pz))


def build_points(asset: dict, variant: str):
    resolution = float(asset["resolution_m"])
    points = []
    _box(points, asset["central_cuboid_bounds_m"], resolution)
    for x, y, _ in asset["outer_tree_centres_m"]:
        _cylinder(points, x, y, float(asset["outer_tree_trunk_radius_m"]),
                  float(asset["outer_tree_height_m"]), resolution)
    for pair in asset["lidar_landmark_pairs_m"]:
        for x, y, _ in pair:
            _cylinder(points, x, y, float(asset["lidar_landmark_radius_m"]),
                      float(asset["lidar_landmark_height_m"]), resolution)
    mask = asset["gnss_only_mask"]
    if not mask["enabled"][variant]:
        return points
    amplitude = float(mask["risky_mask_amplitude_y_m"][variant])
    x_low, x_high = mask["x_interval_m"]
    half_width = float(mask["projection_half_width_m"])
    for x in _axis(float(x_low), float(x_high), resolution):
        u = (x + 12.0) / 24.0
        centre_y = amplitude * math.sin(math.pi * u)
        for y in _axis(centre_y - half_width, centre_y + half_width, resolution):
            for z in _axis(*mask["z_interval_m"], resolution):
                points.append((x, y, z))
    return points


class Icra075MapPublisher(Node):
    def __init__(self):
        super().__init__("icra075_inverse_corridor_map_publisher")
        asset_path = Path(self.declare_parameter("asset_path", "").value).resolve()
        variant = str(self.declare_parameter("scene_variant", "").value)
        expected_hash = str(self.declare_parameter("descriptor_sha256", "").value)
        stamp_topic = str(self.declare_parameter(
            "stamp_authority_topic", "/sim/drone_0/truth_odom").value)
        if variant not in VARIANTS or not asset_path.is_file():
            raise RuntimeError("ICRA-075 map asset/variant invalid")
        asset = json.loads(asset_path.read_text())
        if asset.get("schema_version") != "icra075_runtime_map_fixture_v1":
            raise RuntimeError("ICRA-075 map schema mismatch")
        if asset.get("descriptor_sha256", {}).get(variant) != expected_hash:
            raise RuntimeError("ICRA-075 descriptor/map binding mismatch")
        self._frame_id = str(asset["frame_id"])
        self._points = build_points(asset, variant)
        self._stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._global = self.create_publisher(PointCloud2, "/map_generator/global_cloud", qos)
        self._local = self.create_publisher(PointCloud2, "/map_generator/local_cloud", qos)
        self.create_subscription(Odometry, stamp_topic, self._on_odom, 10)
        self.create_timer(0.5, self._publish)
        self.get_logger().info(
            f"ICRA-075 V2 map ready variant={variant} descriptor={expected_hash} "
            f"points={len(self._points)}")

    def _on_odom(self, message: Odometry):
        self._stamp = message.header.stamp

    def _publish(self):
        if self._stamp is None:
            return
        header = Header(stamp=self._stamp, frame_id=self._frame_id)
        cloud = point_cloud2.create_cloud_xyz32(header, self._points)
        self._global.publish(cloud)
        self._local.publish(cloud)


def main():
    rclpy.init()
    node = Icra075MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
