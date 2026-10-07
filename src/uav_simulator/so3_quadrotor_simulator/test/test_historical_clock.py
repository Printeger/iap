"""CPU process regression of the production clock owner, not a forest trial."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Header
from gnss_comm.msg import GnssMeasMsg
from ament_index_python.packages import get_package_prefix

BINARY = sys.argv.pop(1)


class HistoricalClockTest(unittest.TestCase):
    def test_owner_sensors_consumer_pause_and_resume(self):
        with tempfile.TemporaryDirectory() as temporary:
            environment = {**os.environ, "ROS_LOG_DIR": str(Path(temporary)/"ros")}
            rclpy.init(args=[])
            node = rclpy.create_node("historical_clock_test", parameter_overrides=[
                Parameter("use_sim_time", value=True)])
            clocks, odometry, imu, gnss, lidar = [], [], [], [], []
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            subscriptions = [
                node.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock), qos),
                node.create_subscription(Odometry, "/odom", lambda m: odometry.append(m.header.stamp), qos),
                node.create_subscription(Imu, "/imu_iap", lambda m: imu.append(m.header.stamp), qos),
                node.create_subscription(GnssMeasMsg, "/ublox_driver/range_meas", lambda m: gnss.append(m.meas[0].time) if m.meas else None, qos),
                node.create_subscription(PointCloud2, "/pcl_render_node/cloud", lambda m: lidar.append(m.header.stamp), qos)]
            pause = node.create_publisher(Bool, "/sim/pause", 10)
            map_pub = node.create_publisher(PointCloud2, "/global_map", 10)
            cloud = point_cloud2.create_cloud_xyz32(Header(frame_id="map"), [[2.,0.,1.], [0.,2.,1.]])
            process = subprocess.Popen([BINARY, "--ros-args",
                "-p", "sim_time/enable:=true", "-p", "sim_time/start_utc:=2022-07-06T12:00:00Z",
                "-p", "simulator/hold_until_cmd:=true", "-p", "iap_imu/enable:=true",
                "-p", "rate/simulation:=200.0", "-p", "rate/odom:=100.0",
                "-p", "use_sim_time:=false"], env=environment,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            gnss_process = subprocess.Popen([str(Path(get_package_prefix("gnss_sim"))/"lib/gnss_sim/gnss_sim_node"),
                "--ros-args", "-p", "use_sim_time:=true", "-p", "truth_odom_topic:=/odom",
                "-p", "time_source:=odom_stamp", "-p", "enabled_constellations_csv:=GPS,BDS"],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            lidar_process = subprocess.Popen([str(Path(get_package_prefix("local_sensing"))/"lib/local_sensing/pcl_render_node"),
                "--ros-args", "-p", "use_sim_time:=true", "-r", "odometry:=/odom",
                "-p", "renderer_mode:=spherical_first_hit_v1", "-p", "sensing_horizon:=10.0",
                "-p", "sensing_rate:=10.0", "-p", "lidar.horizontal_samples:=8",
                "-p", "lidar.vertical_samples:=3", "-p", "map/x_size:=10.0",
                "-p", "map/y_size:=10.0", "-p", "map/z_size:=5.0"],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

            def spin(duration):
                until = time.monotonic()+duration
                while time.monotonic() < until:
                    map_pub.publish(cloud)
                    rclpy.spin_once(node, timeout_sec=.01)

            ns = lambda stamp: stamp.sec*10**9+stamp.nanosec
            try:
                until = time.monotonic()+8
                while (len(odometry)<10 or len(imu)<10 or len(gnss)<3 or len(lidar)<3) and time.monotonic()<until:
                    spin(.05)
                self.assertIsNone(process.poll())
                self.assertGreater(len(odometry), 9)
                self.assertGreater(len(imu), 9)
                self.assertGreater(len(gnss), 2)
                self.assertGreater(len(lidar), 2)
                self.assertTrue(all(ns(x) >= 1657108800*10**9 for x in lidar))
                self.assertTrue(all(315964800+x.week*604800+x.tow-18 >= 1657108800 for x in gnss))
                self.assertEqual(node.count_publishers("/clock"), 1)
                self.assertGreaterEqual(ns(clocks[0]), 1657108800*10**9)
                self.assertLess(ns(clocks[-1]), (1657108800+10)*10**9)
                self.assertEqual([ns(x) for x in clocks], sorted(ns(x) for x in clocks))
                self.assertTrue(set(ns(x) for x in odometry).issubset(set(ns(x) for x in clocks)))
                # Subscriptions can discover at different instants and spin
                # one queued callback at a time. Compare their common interval.
                lo = max(ns(odometry[0]), ns(imu[0]))
                hi = min(ns(odometry[-1]), ns(imu[-1]))
                self.assertEqual([ns(x) for x in odometry if lo<=ns(x)<=hi],
                                 [ns(x) for x in imu if lo<=ns(x)<=hi])
                self.assertLess(abs(node.get_clock().now().nanoseconds-ns(clocks[-1])), 30_000_000)
                pause.publish(Bool(data=True)); spin(.2)
                frozen = (len(clocks), len(odometry), ns(clocks[-1]), len(gnss), len(lidar))
                spin(.3)
                self.assertEqual((len(clocks), len(odometry), ns(clocks[-1]), len(gnss), len(lidar)), frozen)
                self.assertEqual(node.get_clock().now().nanoseconds, frozen[2])
                pause.publish(Bool(data=False)); spin(.3)
                self.assertGreater(ns(clocks[-1]), frozen[2])
                self.assertGreater(len(odometry), frozen[1])
                self.assertGreater(len(gnss), frozen[3])
                self.assertGreater(len(lidar), frozen[4])
                bad_source = subprocess.Popen([
                    str(Path(get_package_prefix("gnss_sim"))/"lib/gnss_sim/gnss_sim_node"),
                    "--ros-args", "-r", "__node:=unit_bad_historical_input",
                    "-p", "use_sim_time:=true", "-p", "truth_odom_topic:=/odom",
                    "-p", "time_source:=odom_stamp", "-p", "ephemeris_source:=rinex",
                    "-p", "rinex_nav_file:="+str(Path(temporary)/"missing.rnx"),
                    "-p", "fallback_to_synthetic_on_rinex_error:=false"],
                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                try:
                    until = time.monotonic()+3
                    while bad_source.poll() is None and time.monotonic()<until:
                        spin(.05)
                    self.assertEqual(bad_source.poll(), 2)
                finally:
                    bad_source.send_signal(signal.SIGINT)
                    output, _ = bad_source.communicate(timeout=5)
                self.assertIn("strict historical GNSS input rejected", output)
                # A competing producer can inject an abnormal epoch; the
                # production owner detects lost uniqueness and refuses it.
                competitor = node.create_publisher(Clock, "/clock", qos)
                competitor.publish(Clock(clock=clocks[0]))
                until = time.monotonic()+3
                while process.poll() is None and time.monotonic()<until:
                    spin(.05)
                self.assertEqual(process.poll(), 2)
            finally:
                failures = []
                for child in [process, gnss_process, lidar_process]:
                    child.send_signal(signal.SIGINT)
                    output, _ = child.communicate(timeout=5)
                    if child.returncode not in (0, 2 if child is process else 0):
                        failures.append(output)
                node.destroy_node(); rclpy.shutdown()
                if failures:
                    self.fail("\n".join(failures))

    def test_invalid_epoch_and_self_consumption_refused(self):
        with tempfile.TemporaryDirectory() as temporary:
            environment = {**os.environ, "ROS_LOG_DIR": str(Path(temporary)/"ros")}
            for epoch, consume in [("2022-02-31T12:00:00Z", "false"),
                                   ("2022-07-06T12:00:00garbage", "false"),
                                   ("2022-07-06T12:00:00Z", "true")]:
                result = subprocess.run([BINARY, "--ros-args", "-p", "sim_time/enable:=true",
                    "-p", "sim_time/start_utc:="+epoch, "-p", "use_sim_time:="+consume],
                    env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, timeout=5)
                self.assertEqual(result.returncode, 2, result.stdout)


if __name__ == "__main__":
    unittest.main()
