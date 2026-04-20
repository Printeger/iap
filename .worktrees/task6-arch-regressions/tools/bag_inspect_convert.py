#!/usr/bin/env python3
"""
tools/bag_inspect_convert.py
────────────────────────────────────────────────────────────────
Inspect and convert ROS1/ROS2 bags for the IAP pipeline.

Requires:
  pip3 install --break-system-packages rosbags

Usage
-----
  # 1. Print all topics, types, Hz and sample messages:
  python3 tools/bag_inspect_convert.py inspect \\
      data/realsense_bag_2025-12-19-15-29-43.bag

  # 2. Convert ROS1 .bag → ROS2 directory (sqlite3 + yaml):
  python3 tools/bag_inspect_convert.py convert \\
      data/realsense_bag_2025-12-19-15-29-43.bag \\
      data/realsense_ros2

  # 3. Check bag topics against config_ros.json and suggest remapping:
  python3 tools/bag_inspect_convert.py align \\
      data/realsense_bag_2025-12-19-15-29-43.bag \\
      config/config_ros.json
"""

from __future__ import annotations

import argparse
import collections
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _require_rosbags() -> None:
    try:
        import rosbags  # noqa: F401
    except ImportError:
        print(
            "[ERROR] 'rosbags' not found.\n"
            "  pip3 install --break-system-packages rosbags"
        )
        sys.exit(1)


def _dump(obj, indent: int = 0, max_array: int = 12) -> None:
    """Recursively pretty-print a rosbags dataclass message."""
    pad = "  " * indent
    if hasattr(obj, "__dataclass_fields__"):
        for field in obj.__dataclass_fields__:
            if field == "__msgtype__":
                continue
            val = getattr(obj, field)
            if hasattr(val, "__dataclass_fields__"):
                print(f"{pad}{field}:")
                _dump(val, indent + 1, max_array)
            elif isinstance(val, np.ndarray):
                if val.size <= max_array:
                    print(f"{pad}{field} = {val.tolist()}")
                else:
                    print(f"{pad}{field} = ndarray shape={val.shape} dtype={val.dtype}")
            elif hasattr(val, "__iter__") and not isinstance(val, (str, bytes)):
                lst = list(val)
                # List of dataclasses (e.g. PointField[])
                if lst and hasattr(lst[0], "__dataclass_fields__"):
                    print(f"{pad}{field} = [")
                    for item in lst[:4]:
                        sub = {
                            k: getattr(item, k)
                            for k in item.__dataclass_fields__
                            if k != "__msgtype__"
                        }
                        print(f"{pad}  {sub}")
                    if len(lst) > 4:
                        print(f"{pad}  ... ({len(lst)} total)")
                    print(f"{pad}]")
                elif len(lst) <= max_array:
                    print(f"{pad}{field} = {lst}")
                else:
                    print(f"{pad}{field} = [len={len(lst)}, first={lst[0]!r}, ...]")
            else:
                print(f"{pad}{field} = {val!r}")
    else:
        print(f"{pad}{obj!r}")


# ─────────────────────────────────────────────────────────────────────────────
# CMD: inspect
# ─────────────────────────────────────────────────────────────────────────────

def cmd_inspect(bag_path: Path) -> None:
    _require_rosbags()
    from rosbags.highlevel import AnyReader

    print(f"\n{'═'*65}")
    print(f"  Bag inspection: {bag_path.name}")
    print(f"{'═'*65}\n")

    # ── Pass 1: count messages ──────────────────────────────────────────────
    counts: dict[str, int] = collections.Counter()
    with AnyReader([bag_path]) as r:
        duration_ns: int = r.duration
        start_ns: int = r.start_time
        end_ns: int = r.end_time
        connections = list(r.connections)
        for conn, _ts, _raw in r.messages():
            counts[conn.topic] += 1

    duration_s = duration_ns / 1e9

    print(f"  Start time : {start_ns / 1e9:.6f} s (Unix)")
    print(f"  End time   : {end_ns / 1e9:.6f} s (Unix)")
    print(f"  Duration   : {duration_s:.3f} s")
    print(f"  Messages   : {sum(counts.values())}")
    print()

    # ── Topic table ──────────────────────────────────────────────────────────
    print(f"  {'Topic':<50} {'Type':<40} {'Count':>7}  {'Hz':>7}")
    print(f"  {'─'*50} {'─'*40} {'─'*7}  {'─'*7}")
    for conn in connections:
        cnt = counts.get(conn.topic, 0)
        hz = cnt / max(duration_s, 1e-6)
        print(f"  {conn.topic:<50} {conn.msgtype:<40} {cnt:>7}  {hz:>7.1f}")
    print()

    # ── Sample messages from interesting topics ───────────────────────────
    KEYWORDS = {
        "imu": 1,
        "lidar": 1,
        "points": 1,
        "fix": 1,
        "gps": 1,
        "navsatfix": 1,
        "image": 1,
        "camera": 1,
        "odometry": 1,
        "odom": 1,
    }
    want: dict[str, int] = {}
    for conn in connections:
        lc = conn.topic.lower()
        quota = max((v for k, v in KEYWORDS.items() if k in lc), default=0)
        if quota:
            want[conn.topic] = quota

    if want:
        print("── Sample messages ──────────────────────────────────────────────\n")
        seen: dict[str, int] = {}
        with AnyReader([bag_path]) as r:
            for conn, ts_ns, rawdata in r.messages():
                topic = conn.topic
                if want.get(topic, 0) == 0 or seen.get(topic, 0) >= want.get(topic, 0):
                    continue
                seen[topic] = seen.get(topic, 0) + 1
                try:
                    msg = r.deserialize(rawdata, conn.msgtype)
                    print(f"  ▶ {topic}  [{conn.msgtype}]  t={ts_ns/1e9:.6f}s")
                    _dump(msg, indent=2)
                    print()
                except Exception as exc:
                    print(f"  ▶ {topic}  [deserialization failed: {exc}]\n")
                if all(seen.get(t, 0) >= n for t, n in want.items()):
                    break


# ─────────────────────────────────────────────────────────────────────────────
# CMD: convert
# ─────────────────────────────────────────────────────────────────────────────

def cmd_convert(src: Path, dst: Path) -> None:
    _require_rosbags()

    # Prefer the rosbags-convert CLI (handles all edge cases including the
    # mixed ROS1-bag/ROS2-msgtype format produced by Livox/GLIM recorders).
    try:
        result = subprocess.run(
            ["rosbags-convert", "--src", str(src.resolve()), "--dst", str(dst.resolve())],
            capture_output=False,
            check=True,
        )
        print(f"\nConversion complete → {dst}")
        return
    except FileNotFoundError:
        pass  # fall through to Python API
    except subprocess.CalledProcessError as e:
        print(f"[WARN] rosbags-convert failed ({e}), trying Python API...")

    # Python API fallback
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    dst.mkdir(parents=True, exist_ok=True)

    written = 0
    dropped = 0
    with AnyReader([src]) as reader, Writer(dst, version=9) as writer:
        conn_map: dict[str, object] = {}
        for conn in reader.connections:
            try:
                out_conn = writer.add_connection(conn.topic, conn.msgtype, typestore=ts)
                conn_map[conn.topic] = out_conn
            except Exception as e:
                print(f"  [WARN] skip {conn.topic}: {e}")

        for conn, ts_ns, rawdata in reader.messages():
            out = conn_map.get(conn.topic)
            if out is None:
                dropped += 1
                continue
            try:
                writer.write(out, ts_ns, rawdata)
                written += 1
            except Exception as e:
                dropped += 1
                if dropped <= 5:
                    print(f"  [WARN] drop {conn.topic} @ {ts_ns}: {e}")

    print(f"\nConversion complete → {dst}")
    print(f"  Written: {written}  Dropped: {dropped}")


# ─────────────────────────────────────────────────────────────────────────────
# CMD: align
# ─────────────────────────────────────────────────────────────────────────────

def cmd_align(bag_path: Path, config_path: Path) -> None:
    _require_rosbags()
    from rosbags.highlevel import AnyReader

    # Load config (strip C-style comments)
    raw = config_path.read_text()
    raw = re.sub(r"//[^\n]*", "", raw)
    raw = re.sub(r"/\*.*?\*/", "", raw, flags=re.DOTALL)
    cfg = json.loads(raw)
    glim = cfg.get("glim_ros", {})

    conf_imu    = glim.get("imu_topic",    "(not set)")
    conf_points = glim.get("points_topic", "(not set)")
    conf_image  = glim.get("image_topic",  "(not set)")

    print(f"\n{'═'*65}")
    print("  Topic Alignment Report")
    print(f"{'═'*65}\n")
    print(f"  config_ros.json currently expects:")
    print(f"    imu_topic    : {conf_imu}")
    print(f"    points_topic : {conf_points}")
    print(f"    image_topic  : {conf_image}")
    print()

    with AnyReader([bag_path]) as r:
        bag_topics = {conn.topic: conn.msgtype for conn in r.connections}

    print("  Bag topics:")
    for t, mt in sorted(bag_topics.items()):
        match = "✓" if t in (conf_imu, conf_points, conf_image) else " "
        print(f"    {match} {t:<50}  [{mt}]")
    print()

    IMU_TYPES    = {"sensor_msgs/msg/Imu", "sensor_msgs/Imu"}
    POINTS_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"}
    IMAGE_TYPES  = {
        "sensor_msgs/msg/Image", "sensor_msgs/Image",
        "sensor_msgs/msg/CompressedImage", "sensor_msgs/CompressedImage",
    }

    cand_imu    = [t for t, mt in bag_topics.items() if mt in IMU_TYPES]
    cand_points = [t for t, mt in bag_topics.items() if mt in POINTS_TYPES]
    cand_image  = [t for t, mt in bag_topics.items() if mt in IMAGE_TYPES]

    def best(candidates: list[str], keywords: list[str]) -> str | None:
        for kw in keywords:
            for c in candidates:
                if kw in c.lower():
                    return c
        return candidates[0] if candidates else None

    best_imu    = best(cand_imu,    ["livox", "imu"])
    best_points = best(cand_points, ["livox", "lidar", "points"])
    best_image  = best(cand_image,  ["infra1", "color", "image"])

    print("  Suggested config_ros.json patch:")
    print(f'    "imu_topic":    "{best_imu    or conf_imu}",')
    print(f'    "points_topic": "{best_points or conf_points}",')
    print(f'    "image_topic":  "{best_image  or conf_image}",')
    print()

    if best_imu or best_points:
        imu_t    = best_imu    or conf_imu
        pts_t    = best_points or conf_points
        img_t    = best_image  or conf_image
        ros2_dst = bag_path.parent / (bag_path.stem + "_ros2")
        print("  ROS2 playback with topic remapping:")
        print(f"    ros2 bag play {ros2_dst} \\")
        print(f"        --remap {imu_t}:={conf_imu} \\")
        print(f"                {pts_t}:={conf_points} \\")
        print(f"                {img_t}:={conf_image}")
        print()
        print("  Or update config_ros.json topics (already done if you ran this")
        print(f"  script's convert + align and accepted the patch above).")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Inspect / convert ROS bags for the IAP pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("inspect", help="Print topic table and sample messages")
    s.add_argument("bag", type=Path)

    s = sub.add_parser("convert", help="Convert ROS1 .bag → ROS2 directory")
    s.add_argument("bag", type=Path, help="Source ROS1 .bag")
    s.add_argument(
        "output", type=Path, nargs="?",
        help="Destination dir (default: <bag_stem>_ros2 in same directory)",
    )

    s = sub.add_parser("align", help="Check bag topics vs config_ros.json")
    s.add_argument("bag", type=Path)
    s.add_argument(
        "config", type=Path, nargs="?",
        default=Path("config/config_ros.json"),
    )

    return p.parse_args()


def main() -> None:
    args = _parse_args()

    if args.cmd == "inspect":
        cmd_inspect(args.bag)

    elif args.cmd == "convert":
        out = args.output or (args.bag.parent / (args.bag.stem + "_ros2"))
        cmd_convert(args.bag, out)

    elif args.cmd == "align":
        cmd_align(args.bag, args.config)


if __name__ == "__main__":
    main()
