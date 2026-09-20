import argparse
import os
import struct
import subprocess
import time
import unittest


def fnv_curve_hash(start_ns, points, knots):
    value = 1469598103934665603

    def add(raw):
        nonlocal value
        for byte in raw:
            value ^= byte
            value = (value * 1099511628211) & 0xFFFFFFFFFFFFFFFF

    add(struct.pack("=q", start_ns))
    add(struct.pack("=q", 3))
    add(struct.pack("=q", len(points)))
    for point in points:
        for coordinate in point:
            add(struct.pack("=d", coordinate))
    add(struct.pack("=q", len(knots)))
    for knot in knots:
        add(struct.pack("=d", knot))
    return f"fnv1a64:{value:016x}"


class TrajectoryServerHandshakeTest(unittest.TestCase):
    def test_future_activation_and_identity_conflict(self):
        import rclpy
        from geometry_msgs.msg import Point
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from traj_utils.msg import Bspline, TrajectoryCommandStatus

        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(170 + os.getpid() % 50)
        server = subprocess.Popen(
            [ARGS.server], env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        previous_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
        rclpy.init()
        node = rclpy.create_node("trajectory_server_handshake_test")
        retained = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        command_pub = node.create_publisher(Bspline, "planning/bspline", retained)
        odom_pub = node.create_publisher(Odometry, "odometry", 10)
        statuses = []
        commands = []
        node.create_subscription(
            TrajectoryCommandStatus, "planning/pending_guard_status",
            statuses.append, retained)
        node.create_subscription(
            PositionCommand, "/position_cmd", commands.append, 50)

        points = [(0.0, 0.0, 1.0), (0.1, 0.0, 1.0),
                  (0.2, 0.0, 1.0), (0.3, 0.0, 1.0),
                  (0.4, 0.0, 1.0), (0.5, 0.0, 1.0)]
        knots = [-0.3, -0.2, -0.1, 0.0, 0.1,
                 0.2, 0.3, 0.4, 0.5, 0.6]
        start_ns = 100_200_000_000
        command = Bspline()
        command.order = 3
        command.traj_id = 7
        command.execution_instance_id = 9001
        command.start_time.sec = start_ns // 1_000_000_000
        command.start_time.nanosec = start_ns % 1_000_000_000
        command.pos_pts = [Point(x=x, y=y, z=z) for x, y, z in points]
        command.knots = knots
        command.curve_hash = fnv_curve_hash(start_ns, points, knots)

        def spin_at(stamp_ns, duration_s):
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                odom = Odometry()
                odom.header.stamp.sec = stamp_ns // 1_000_000_000
                odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
                odom.pose.pose.position.z = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            spin_at(100_000_000_000, 1.2)
            command_pub.publish(command)
            spin_at(100_050_000_000, 0.4)
            self.assertTrue(any(
                status.state == status.QUEUED and status.trajectory_id == 7
                for status in statuses))
            self.assertFalse(any(
                status.state == status.ACTIVATED and status.trajectory_id == 7
                for status in statuses))

            spin_at(100_250_000_000, 0.4)
            self.assertTrue(any(
                status.state == status.ACTIVATED and status.trajectory_id == 7
                for status in statuses))
            self.assertTrue(any(
                item.execution_instance_id == 9001
                and item.trajectory_id == 7
                and item.curve_hash == command.curve_hash
                for item in commands))

            conflict = Bspline()
            conflict.order = command.order
            conflict.traj_id = command.traj_id
            conflict.execution_instance_id = command.execution_instance_id
            conflict.start_time = command.start_time
            conflict.pos_pts = command.pos_pts
            conflict.knots = command.knots
            conflict.curve_hash = "fnv1a64:0000000000000000"
            command_pub.publish(conflict)
            spin_at(100_260_000_000, 0.3)
            self.assertTrue(any(
                status.state == status.REJECTED
                and status.rejection_reason ==
                    "invalid_curve_identity_or_payload"
                for status in statuses))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            server.terminate()
            try:
                server.wait(timeout=3)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=3)
            if server.stdout is not None:
                server.stdout.close()
            if previous_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain


PARSER = argparse.ArgumentParser()
PARSER.add_argument("--server", required=True)
ARGS, UNITTEST_ARGS = PARSER.parse_known_args()


if __name__ == "__main__":
    unittest.main(argv=[__file__, *UNITTEST_ARGS])
