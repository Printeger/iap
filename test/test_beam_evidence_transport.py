"""Check full-size first-hit evidence through the real renderer's DDS ports."""

import argparse
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from iap.msg import LidarBeamEvidence

from nav_msgs.msg import Odometry

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2

from sensor_msgs_py import point_cloud2


class BeamTransport(unittest.TestCase):
    """Keep complete, fragmented beam scans bound to their exact cloud time."""

    def test_renderer_uses_reliable_complete_scan_delivery(self):
        """Observe actual publisher policy and paired full-size messages."""
        with tempfile.TemporaryDirectory(prefix='beam_') as temporary:
            env = {**os.environ, 'ROS_LOG_DIR': str(Path(temporary) / 'ros')}
            with open(Path(temporary) / 'renderer.log', 'w+') as log:
                process = subprocess.Popen(
                    [args.renderer, '--ros-args', '-p',
                     'renderer_mode:=spherical_first_hit_v1', '-p',
                     'sensing_rate:=10.0'],
                    env=env, stdout=log, stderr=subprocess.STDOUT)
                rclpy.init()
                node = rclpy.create_node('beam_transport_probe')
                odom_pub = node.create_publisher(Odometry, '/odometry', 10)
                world_pub = node.create_publisher(
                    PointCloud2, '/global_map', 10)
                beams, clouds = [], []
                node.create_subscription(
                    LidarBeamEvidence, '/iap/simulator/lidar_beam_evidence',
                    beams.append, QoSProfile(depth=8))
                node.create_subscription(
                    PointCloud2, '/pcl_render_node/cloud', clouds.append, 8)
                started = time.monotonic()
                try:
                    while len(beams) < 3 and time.monotonic() - started < 5:
                        self.assertIsNone(process.poll())
                        odom = Odometry()
                        odom.header.stamp = node.get_clock().now().to_msg()
                        odom.header.frame_id = 'map'
                        odom.pose.pose.position.z = 1.
                        odom.pose.pose.orientation.w = 1.
                        odom_pub.publish(odom)
                        world_pub.publish(point_cloud2.create_cloud_xyz32(
                            odom.header, [(2., 0., 1.)]))
                        rclpy.spin_once(node, timeout_sec=.02)
                    log.flush()
                    log.seek(0)
                    self.assertGreaterEqual(len(beams), 3, log.read())
                    info = node.get_publishers_info_by_topic(
                        '/iap/simulator/lidar_beam_evidence')
                    self.assertEqual(len(info), 1)
                    self.assertEqual(info[0].qos_profile.reliability,
                                     ReliabilityPolicy.RELIABLE)
                    paired = beams[:3]
                    expected = {(b.header.stamp.sec, b.header.stamp.nanosec)
                                for b in paired}
                    deadline = time.monotonic() + .5
                    while time.monotonic() < deadline:
                        cloud_stamps = {
                            (c.header.stamp.sec, c.header.stamp.nanosec)
                            for c in clouds}
                        if expected <= cloud_stamps:
                            break
                        rclpy.spin_once(node, timeout_sec=.02)
                    for scan in paired:
                        self.assertTrue(scan.complete)
                        self.assertEqual(len(scan.outcomes), 20480)
                        for rows in (scan.direction_x, scan.direction_y,
                                     scan.direction_z, scan.ranges_m):
                            self.assertEqual(len(rows), len(scan.outcomes))
                        self.assertTrue(scan.content_hash)
                        self.assertIn(
                            (scan.header.stamp.sec, scan.header.stamp.nanosec),
                            cloud_stamps)
                        stamp = (scan.header.stamp.sec
                                 + scan.header.stamp.nanosec * 1e-9)
                        self.assertAlmostEqual(scan.scan_end_stamp_s, stamp,
                                               delta=1e-6)
                    print('Reliable UDP renderer: 3 paired scans, '
                          '20480 beams/scan')
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--renderer', required=True)
    args, remaining = parser.parse_known_args()
    profile = (Path(get_package_share_directory('iap'))
               / 'config/sim_ego/fastdds_udp_only.xml')
    if not profile.is_file():
        parser.error(f'installed UDP profile missing: {profile}')
    # Both participants inherit this policy before initialization.
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    unittest.main(argv=['beam_transport', *remaining])
