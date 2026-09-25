import argparse
import os
import struct
import subprocess
import threading
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
    def test_planner_message_activates_unchanged_in_traj_server(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from traj_utils.msg import Bspline, TrajectoryCommandStatus

        self.assertIsNotNone(
            ARGS.planner_test,
            "the planner publication executable is required for this seam")
        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(190 + os.getpid() % 20)
        environment["IAP_PROCESS_HANDSHAKE"] = "1"
        server = subprocess.Popen(
            [ARGS.server], env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        previous_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
        rclpy.init()
        node = rclpy.create_node("planner_actual_curve_handoff_test")
        retained = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom_pub = node.create_publisher(Odometry, "odometry", 10)
        planner_messages = []
        statuses = []
        commands = []
        planner_subscription = node.create_subscription(
            Bspline, "planning/bspline", planner_messages.append, retained)
        status_subscription = node.create_subscription(
            TrajectoryCommandStatus, "planning/pending_guard_status",
            statuses.append, retained)
        command_subscription = node.create_subscription(
            PositionCommand, "/position_cmd", commands.append, 50)
        self.assertIsNotNone(planner_subscription)
        self.assertIsNotNone(status_subscription)
        self.assertIsNotNone(command_subscription)

        def spin_at(stamp_ns, duration_s):
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                odom = Odometry()
                odom.header.stamp.sec = stamp_ns // 1_000_000_000
                odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
                odom.pose.pose.position.z = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.01)

        planner = None
        planner_output = ""
        try:
            spin_at(10_000_000_000, 1.3)
            planner = subprocess.Popen(
                [ARGS.planner_test,
                 "--gtest_filter=P4LimitedPrefixPublication."
                 "IncompleteChannelGeometryPublishesTubeIntersectionPrefix"],
                env=environment, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True)
            deadline = time.monotonic() + 8.0
            while (not planner_messages or not any(
                    item.state == item.QUEUED for item in statuses)) and (
                    time.monotonic() < deadline):
                spin_at(10_000_000_000, 0.05)
            planner_output, _ = planner.communicate(timeout=5)
            self.assertEqual(planner.returncode, 0, planner_output)
            self.assertTrue(
                planner_messages,
                "the process test did not consume the planner's actual "
                f"message\nplanner output:\n{planner_output}")
            published = planner_messages[-1]
            self.assertGreater(published.traj_id, 0)
            self.assertGreater(published.execution_instance_id, 0)
            self.assertTrue(published.curve_hash)
            start_ns = (
                published.start_time.sec * 1_000_000_000
                + published.start_time.nanosec)
            queued = [item for item in statuses
                      if item.state == item.QUEUED
                      and item.trajectory_id == published.traj_id]
            self.assertTrue(queued)
            self.assertTrue(all(
                item.execution_instance_id == published.execution_instance_id
                and item.start_time == published.start_time
                and item.curve_hash == published.curve_hash
                for item in queued))

            spin_at(start_ns + 50_000_000, 0.5)
            activated = [item for item in statuses
                         if item.state == item.ACTIVATED
                         and item.trajectory_id == published.traj_id]
            self.assertTrue(activated)
            self.assertTrue(all(
                item.execution_instance_id == published.execution_instance_id
                and item.start_time == published.start_time
                and item.curve_hash == published.curve_hash
                for item in activated))

            motion_deadline = time.monotonic() + 4.5
            while time.monotonic() < motion_deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            matching_commands = [item for item in commands
                                 if item.trajectory_id == published.traj_id]
            self.assertTrue(matching_commands)
            self.assertTrue(all(
                item.execution_instance_id == published.execution_instance_id
                and item.trajectory_start_time == published.start_time
                and item.curve_hash == published.curve_hash
                for item in matching_commands))
            self.assertTrue(any(
                (item.velocity.x ** 2 + item.velocity.y ** 2
                 + item.velocity.z ** 2) ** 0.5 > 0.05
                for item in matching_commands))
            self.assertLessEqual(
                max(item.position.x for item in matching_commands), 5.0)
            terminal = matching_commands[-1]
            self.assertLess(
                (terminal.velocity.x ** 2 + terminal.velocity.y ** 2
                 + terminal.velocity.z ** 2) ** 0.5, 1.0e-6)
            self.assertLess(
                (terminal.acceleration.x ** 2 + terminal.acceleration.y ** 2
                 + terminal.acceleration.z ** 2) ** 0.5, 1.0e-6)
            self.assertAlmostEqual(
                terminal.position.x, published.pos_pts[-1].x, places=6)
            self.assertNotIn(
                "normal_channel_selected_candidate_missing", planner_output)
            self.assertNotIn(
                "P4-v2 P5-pass lineage write failed before publish",
                planner_output)
            self.assertNotIn("alternate-channel", planner_output)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            if planner is not None and planner.poll() is None:
                planner.terminate()
                planner.wait(timeout=3)
            server.terminate()
            try:
                server.wait(timeout=3)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=3)
            self.assertEqual(
                server.returncode, 0,
                "traj_server must tear down all ROS entities before shutdown")
            if server.stdout is not None:
                server.stdout.close()
            if previous_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain

    def test_parent_child_switch_activates_without_intermediate_stop(self):
        import rclpy
        from geometry_msgs.msg import Point
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import PositionCommand
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from traj_utils.msg import Bspline, TrajectoryCommandStatus

        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = str(210 + os.getpid() % 20)
        environment["IAP_PROCESS_HANDSHAKE"] = "1"
        server = subprocess.Popen(
            [ARGS.server], env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        previous_domain = os.environ.get("ROS_DOMAIN_ID")
        os.environ["ROS_DOMAIN_ID"] = environment["ROS_DOMAIN_ID"]
        rclpy.init()
        node = rclpy.create_node("continuous_successor_handoff_test")
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

        base_ns = 200_000_000_000

        def spin_ramp(start_ns, duration_s):
            started = time.monotonic()
            deadline = started + duration_s
            while time.monotonic() < deadline:
                stamp_ns = start_ns + int(
                    (time.monotonic() - started) * 1_000_000_000)
                odom = Odometry()
                odom.header.stamp.sec = stamp_ns // 1_000_000_000
                odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
                odom.pose.pose.position.z = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.01)

        def linear_command(trajectory_id, start_ns, point_offset):
            command = Bspline()
            command.order = 3
            command.traj_id = trajectory_id
            command.execution_instance_id = 9901
            command.start_time.sec = start_ns // 1_000_000_000
            command.start_time.nanosec = start_ns % 1_000_000_000
            points = [
                (0.05 * (index + point_offset), 0.0, 1.0)
                for index in range(14)]
            knots = [0.1 * (index - 3) for index in range(18)]
            command.pos_pts = [Point(x=x, y=y, z=z) for x, y, z in points]
            command.knots = knots
            command.curve_hash = fnv_curve_hash(start_ns, points, knots)
            return command

        try:
            spin_ramp(base_ns, 1.2)
            parent_start_ns = base_ns + 1_500_000_000
            parent = linear_command(21, parent_start_ns, 0)
            command_pub.publish(parent)
            spin_ramp(base_ns + 1_200_000_000, 0.55)
            self.assertTrue(any(
                item.state == item.ACTIVATED and item.trajectory_id == 21
                for item in statuses))

            switch_elapsed_s = 0.8
            child_start_ns = parent_start_ns + 800_000_000
            child = linear_command(22, child_start_ns, 8)
            child.parent_execution_instance_id = parent.execution_instance_id
            child.parent_traj_id = parent.traj_id
            child.parent_start_time = parent.start_time
            child.parent_curve_hash = parent.curve_hash
            child.parent_switch_elapsed_s = switch_elapsed_s
            command_pub.publish(child)
            spin_ramp(base_ns + 1_750_000_000, 1.0)

            queued = [item for item in statuses
                      if item.state == item.QUEUED
                      and item.trajectory_id == child.traj_id]
            activated = [item for item in statuses
                         if item.state == item.ACTIVATED
                         and item.trajectory_id == child.traj_id]
            self.assertTrue(queued)
            self.assertTrue(activated)
            self.assertTrue(all(
                item.execution_instance_id == child.execution_instance_id
                and item.start_time == child.start_time
                and item.curve_hash == child.curve_hash
                for item in queued + activated))

            handoff_commands = [
                item for item in commands
                if item.trajectory_id in (parent.traj_id, child.traj_id)]
            first_child = next(
                index for index, item in enumerate(handoff_commands)
                if item.trajectory_id == child.traj_id)
            self.assertGreater(first_child, 0)
            before = handoff_commands[first_child - 1]
            after = handoff_commands[first_child]
            before_speed = (
                before.velocity.x ** 2 + before.velocity.y ** 2
                + before.velocity.z ** 2) ** 0.5
            after_speed = (
                after.velocity.x ** 2 + after.velocity.y ** 2
                + after.velocity.z ** 2) ** 0.5
            self.assertGreater(before_speed, 0.05)
            self.assertGreater(after_speed, 0.05)
            position_jump = (
                (after.position.x - before.position.x) ** 2
                + (after.position.y - before.position.y) ** 2
                + (after.position.z - before.position.z) ** 2) ** 0.5
            self.assertLess(position_jump, 0.1)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            server.terminate()
            try:
                server.wait(timeout=3)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=3)
            self.assertEqual(
                server.returncode, 0,
                "traj_server must tear down all ROS entities before shutdown")
            if server.stdout is not None:
                server.stdout.close()
            if previous_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain

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
        status_receive_steady = {}
        commands = []
        def receive_status(message):
            statuses.append(message)
            status_receive_steady[(message.trajectory_id, message.state)] = (
                time.monotonic())
        node.create_subscription(
            TrajectoryCommandStatus, "planning/pending_guard_status",
            receive_status, retained)
        node.create_subscription(
            PositionCommand, "/position_cmd", commands.append, 50)

        # This is the process-level projection of the frozen observation
        # spline used by the planner publication regression. The final three
        # equal control points impose its certified terminal stop.
        points = [(-4.0, 0.0, 1.0), (-3.0, 0.0, 1.0),
                  (-2.0, 1.25, 1.0), (-1.0, 1.5, 1.0),
                  (0.0, 1.5, 1.0), (1.0, 1.5, 1.0),
                  (4.0, 0.0, 1.0), (4.0, 0.0, 1.0),
                  (4.0, 0.0, 1.0)]
        knots = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3,
                 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
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

        def spin_without_odom(duration_s):
            deadline = time.monotonic() + duration_s
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            # A transient-local planner command can arrive while traj_server
            # is starting, before its first odometry sample establishes the
            # execution clock. It must be retained without a wall-clock
            # rejection and admitted once odometry arrives.
            command_pub.publish(command)
            spin_without_odom(1.2)
            self.assertFalse(any(
                status.trajectory_id == 7 for status in statuses),
                "traj_server must not classify a command before its odom clock exists")
            spin_at(100_000_000_000, 0.4)
            spin_at(100_050_000_000, 0.4)
            self.assertTrue(any(
                status.state == status.QUEUED and status.trajectory_id == 7
                for status in statuses))
            self.assertFalse(any(
                status.state == status.ACTIVATED and status.trajectory_id == 7
                for status in statuses))

            replacement = Bspline()
            replacement.order = command.order
            replacement.traj_id = 9
            replacement.execution_instance_id = command.execution_instance_id
            replacement_start_ns = 100_300_000_000
            replacement.start_time.sec = replacement_start_ns // 1_000_000_000
            replacement.start_time.nanosec = (
                replacement_start_ns % 1_000_000_000)
            replacement.pos_pts = command.pos_pts
            replacement.knots = command.knots
            replacement.curve_hash = fnv_curve_hash(
                replacement_start_ns, points, knots)
            command_pub.publish(replacement)
            spin_at(100_100_000_000, 0.4)
            self.assertTrue(any(
                status.state == status.CANCELED and status.trajectory_id == 7
                for status in statuses))
            self.assertTrue(any(
                status.state == status.QUEUED and status.trajectory_id == 9
                for status in statuses))

            spin_at(100_350_000_000, 0.4)
            self.assertTrue(any(
                status.state == status.ACTIVATED and status.trajectory_id == 9
                for status in statuses))
            self.assertTrue(any(
                item.execution_instance_id == 9001
                and item.trajectory_id == 9
                and item.curve_hash == replacement.curve_hash
                for item in commands))
            activated_commands = [
                item for item in commands
                if item.execution_instance_id == 9001
                and item.trajectory_id == 9
                and item.curve_hash == replacement.curve_hash
            ]
            self.assertTrue(activated_commands)
            self.assertTrue(any(
                (item.velocity.x ** 2 + item.velocity.y ** 2
                 + item.velocity.z ** 2) ** 0.5 > 1.0e-3
                for item in activated_commands
            ), "the activated observation segment must command motion")
            self.assertTrue(all(
                abs(item.position.z - 1.0) < 1.0e-6
                for item in activated_commands
            ), "the first activated command must evaluate the curve origin")

            # The execution clock deliberately bounds ROS-time catch-up by
            # same-host steady time. Give the real process enough time to
            # traverse the short observation curve before checking its stop.
            spin_at(100_950_000_000, 0.05)
            spin_without_odom(0.8)
            completed_commands = [
                item for item in commands
                if item.execution_instance_id == 9001
                and item.trajectory_id == 9
                and item.curve_hash == replacement.curve_hash
            ]
            self.assertTrue(completed_commands)
            terminal = completed_commands[-1]
            self.assertAlmostEqual(terminal.position.x, 4.0, places=6)
            self.assertAlmostEqual(terminal.position.y, 0.0, places=6)
            self.assertLess(
                (terminal.velocity.x ** 2 + terminal.velocity.y ** 2
                 + terminal.velocity.z ** 2) ** 0.5,
                1.0e-6,
                "the observation command must stop at its endpoint")

            conflict = Bspline()
            conflict.order = command.order
            conflict.traj_id = replacement.traj_id
            conflict.execution_instance_id = replacement.execution_instance_id
            conflict.start_time = replacement.start_time
            conflict.pos_pts = replacement.pos_pts
            conflict.knots = replacement.knots
            conflict.curve_hash = "fnv1a64:0000000000000000"
            command_pub.publish(conflict)
            spin_at(100_260_000_000, 0.3)
            self.assertTrue(any(
                status.state == status.REJECTED
                and status.rejection_reason ==
                    "invalid_curve_identity_or_payload"
                for status in statuses))

            # The live stack publishes odometry much faster than the 100 Hz
            # command timer. A reliable odometry backlog used to leave the
            # server execution clock 10+ seconds behind the planner, turning
            # a 1.2 s future command into a long hover. Flood the production
            # process and require activation to remain bound to the newest
            # odometry epoch rather than to queued historical samples.
            loaded = Bspline()
            loaded.order = command.order
            loaded.traj_id = 11
            loaded.execution_instance_id = command.execution_instance_id
            load_base_ns = 101_000_000_000
            loaded_start_ns = load_base_ns + 500_000_000
            loaded.start_time.sec = loaded_start_ns // 1_000_000_000
            loaded.start_time.nanosec = loaded_start_ns % 1_000_000_000
            loaded.pos_pts = command.pos_pts
            loaded.knots = command.knots
            loaded.curve_hash = fnv_curve_hash(
                loaded_start_ns, points, knots)

            stop_flood = threading.Event()
            flood_started = time.monotonic()

            def flood_odometry():
                while not stop_flood.is_set():
                    elapsed_ns = int(
                        (time.monotonic() - flood_started) * 1_000_000_000)
                    odom = Odometry()
                    stamp_ns = load_base_ns + elapsed_ns
                    odom.header.stamp.sec = stamp_ns // 1_000_000_000
                    odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
                    odom.pose.pose.position.z = 1.0
                    odom_pub.publish(odom)

            flood = threading.Thread(target=flood_odometry, daemon=True)
            flood.start()
            command_pub.publish(loaded)
            activation_deadline = time.monotonic() + 1.5
            while (11, TrajectoryCommandStatus.ACTIVATED) not in (
                    status_receive_steady) and time.monotonic() < activation_deadline:
                rclpy.spin_once(node, timeout_sec=0.005)
            stop_flood.set()
            flood.join(timeout=1.0)
            activated = [
                status for status in statuses
                if status.trajectory_id == 11
                and status.state == status.ACTIVATED
            ]
            self.assertTrue(
                activated,
                "traj_server must activate under a high-rate odometry load")
            actual_ns = (
                activated[-1].actual_event_time.sec * 1_000_000_000
                + activated[-1].actual_event_time.nanosec)
            self.assertLessEqual(
                actual_ns - loaded_start_ns, 150_000_000,
                "activation clock may lag the newest odometry by at most 150 ms")

            # A child whose declared full parent identity is correct still
            # must not activate when its curve origin jumps away from the
            # parent's fixed switch p/v/a. The parent remains authoritative.
            discontinuous = Bspline()
            discontinuous.order = command.order
            discontinuous.traj_id = 12
            discontinuous.execution_instance_id = command.execution_instance_id
            discontinuous_start_ns = actual_ns + 300_000_000
            discontinuous.start_time.sec = (
                discontinuous_start_ns // 1_000_000_000)
            discontinuous.start_time.nanosec = (
                discontinuous_start_ns % 1_000_000_000)
            discontinuous_points = [
                (x + 4.0, y, z) for x, y, z in points]
            discontinuous.pos_pts = [
                Point(x=x, y=y, z=z)
                for x, y, z in discontinuous_points]
            discontinuous.knots = command.knots
            discontinuous.curve_hash = fnv_curve_hash(
                discontinuous_start_ns, discontinuous_points, knots)
            discontinuous.parent_execution_instance_id = (
                loaded.execution_instance_id)
            discontinuous.parent_traj_id = loaded.traj_id
            discontinuous.parent_start_time = loaded.start_time
            discontinuous.parent_curve_hash = loaded.curve_hash
            command_pub.publish(discontinuous)

            ramp_started = time.monotonic()
            ramp_end = ramp_started + 1.0
            while time.monotonic() < ramp_end:
                stamp_ns = actual_ns + int(
                    (time.monotonic() - ramp_started) * 1_000_000_000)
                odom = Odometry()
                odom.header.stamp.sec = stamp_ns // 1_000_000_000
                odom.header.stamp.nanosec = stamp_ns % 1_000_000_000
                odom.pose.pose.position.z = 1.0
                odom_pub.publish(odom)
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertTrue(any(
                status.state == status.REJECTED
                and status.trajectory_id == 12
                and status.rejection_reason ==
                    "parent_boundary_discontinuous_rebuild_required"
                for status in statuses))
            self.assertTrue(commands)
            self.assertEqual(commands[-1].trajectory_id, 11)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            server.terminate()
            try:
                server.wait(timeout=3)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=3)
            self.assertEqual(
                server.returncode, 0,
                "traj_server must tear down all ROS entities before shutdown")
            if server.stdout is not None:
                server.stdout.close()
            if previous_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = previous_domain


PARSER = argparse.ArgumentParser()
PARSER.add_argument("--server", required=True)
PARSER.add_argument("--planner-test")
ARGS, UNITTEST_ARGS = PARSER.parse_known_args()


if __name__ == "__main__":
    unittest.main(argv=[__file__, *UNITTEST_ARGS])
