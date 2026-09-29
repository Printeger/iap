#!/usr/bin/env python3
"""ICRA-075 wrapper that substitutes the exact V2 public map publisher."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_base_launch():
    path = Path(__file__).resolve().with_name("test_planner.launch.py")
    spec = importlib.util.spec_from_file_location("icra075_base_test_planner", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load installed test_planner launch")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = _load_base_launch()


def _setup(context):
    actions = BASE._launch_setup(context)
    retained = []
    replaced = 0
    for action in actions:
        if (isinstance(action, Node) and action.node_package == "iap" and
                action.node_executable == "demo11_corridor_map_publisher"):
            replaced += 1
            retained.append(Node(
                package="map_generator",
                executable="icra075_inverse_corridor_map_publisher.py",
                name="test_planner_corridor_map_publisher",
                output="screen",
                parameters=[{
                    "asset_path": str(Path(get_package_share_directory(
                        "map_generator")) / "config/icra075_inverse_corridor_v2.json"),
                    "scene_variant": LaunchConfiguration(
                        "icra075_scene_variant").perform(context),
                    "descriptor_sha256": LaunchConfiguration(
                        "icra075_descriptor_sha256").perform(context),
                    "stamp_authority_topic": LaunchConfiguration(
                        "corridor_map_stamp_authority_topic").perform(context),
                }],
            ))
        else:
            retained.append(action)
    if replaced != 1:
        raise RuntimeError(f"ICRA-075 expected one public map node, observed {replaced}")
    return retained


def generate_launch_description():
    iap_share = get_package_share_directory("iap")
    fastdds = os.path.join(iap_share, "config", "sim_ego", "fastdds_udp_only.xml")
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default)
          for name, default in BASE.ARG_DEFAULTS],
        DeclareLaunchArgument("icra075_scene_variant"),
        DeclareLaunchArgument("icra075_descriptor_sha256"),
        SetEnvironmentVariable("QT_X11_NO_MITSHM", "1"),
        SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds),
        OpaqueFunction(function=_setup),
    ])
