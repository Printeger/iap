"""Exercise future Bspline activation through the real ROS command server."""

import argparse
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from geometry_msgs.msg import Point

from quadrotor_msgs.msg import PositionCommand

import rclpy

from traj_utils.msg import Bspline


class ScheduledExecution(unittest.TestCase):
    """Check the command stream at the trajectory server boundary."""

    def test_switch_and_immediate_cancellation(self):
        """Continue old commands, switch continuously, and cancel pending."""
        with tempfile.TemporaryDirectory(prefix='ego_scheduled_') as directory:
            env = {**os.environ, 'ROS_LOG_DIR': str(Path(directory) / 'ros')}
            with open(Path(directory) / 'server.log', 'w+') as log:
                process = subprocess.Popen(
                    [args.server], env=env, stdout=log,
                    stderr=subprocess.STDOUT)
                rclpy.init()
                node = rclpy.create_node('scheduled_execution_test')
                publisher = node.create_publisher(
                    Bspline, '/planning/bspline', 10)
                commands = []
                node.create_subscription(
                    PositionCommand, '/position_cmd', commands.append, 100)

                def spin(seconds):
                    until = time.monotonic() + seconds
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.01)
                        self.assertIsNone(process.poll())

                def message(identifier, start, position,
                            scheduled=False, moving=True):
                    curve = Bspline()
                    curve.order = 3
                    curve.traj_id = identifier
                    curve.start_time = rclpy.time.Time(seconds=start).to_msg()
                    curve.start_mode = (
                        Bspline.AT_TIME if scheduled else Bspline.IMMEDIATE)
                    curve.knots = [float(i - 3) for i in range(11)]
                    curve.pos_pts = [
                        Point(x=position + (i - 1 if moving else 0),
                              y=0., z=1.)
                        for i in range(7)]
                    return curve

                def stamp(command):
                    return (command.header.stamp.sec
                            + command.header.stamp.nanosec * 1e-9)

                try:
                    deadline = time.monotonic() + 5
                    while (not publisher.get_subscription_count()
                           or not node.count_publishers('/position_cmd')):
                        if time.monotonic() >= deadline:
                            break
                        spin(.02)
                    self.assertTrue(publisher.get_subscription_count())
                    start = node.get_clock().now().nanoseconds * 1e-9
                    publisher.publish(message(1, start, 0.))
                    until = time.monotonic() + 1.
                    while not commands and time.monotonic() < until:
                        publisher.publish(message(1, start, 0.))
                        spin(.05)
                    log.flush()
                    log.seek(0)
                    self.assertTrue(commands, log.read())
                    switch = node.get_clock().now().nanoseconds * 1e-9 + .5
                    # Linear cubics have velocity 1 and acceleration 0.
                    publisher.publish(message(
                        2, switch, switch - start, scheduled=True))
                    spin(.3)
                    recent = commands[-10:]
                    self.assertTrue(all(c.trajectory_id == 1 for c in recent))
                    self.assertGreater(
                        recent[-1].position.x, recent[0].position.x)
                    spin(.35)
                    new = [c for c in commands if c.trajectory_id == 2]
                    self.assertTrue(new)
                    self.assertTrue(all(stamp(c) >= switch for c in new))
                    for command in commands:
                        self.assertAlmostEqual(
                            command.position.x, stamp(command) - start,
                            delta=2e-5)
                        self.assertAlmostEqual(
                            command.velocity.x, 1., delta=1e-6)
                        self.assertAlmostEqual(
                            command.acceleration.x, 0., delta=1e-6)
                    old = [c for c in commands if c.trajectory_id == 1]
                    self.assertLess(stamp(new[0]) - stamp(old[-1]), .1)
                    # Late scheduled work cannot replace the active curve.
                    publisher.publish(message(
                        3, switch - .1, switch - .1 - start, scheduled=True))
                    spin(.1)
                    self.assertEqual(commands[-1].trajectory_id, 2)
                    future = node.get_clock().now().nanoseconds * 1e-9 + .4
                    publisher.publish(message(
                        4, future, future - start, scheduled=True))
                    spin(.05)
                    # Authorization can be withdrawn before a checked brake
                    # is constructed. Empty withdrawal is not a replacement
                    # curve and must keep the active predecessor commanding.
                    withdrawal = Bspline()
                    withdrawal.start_mode = Bspline.CANCEL_PENDING
                    withdrawal.traj_id = 99
                    publisher.publish(withdrawal)
                    spin(.03)
                    withdrawal.traj_id = 4
                    publisher.publish(withdrawal)
                    spin(.5)
                    self.assertFalse(
                        any(c.trajectory_id == 4 for c in commands),
                        'withdrawn pending trajectory activated before brake')
                    self.assertEqual(commands[-1].trajectory_id, 2)
                    for command in commands[-10:]:
                        self.assertAlmostEqual(
                            command.position.x, stamp(command) - start,
                            delta=2e-5)
                        self.assertAlmostEqual(command.velocity.x, 1.)
                        self.assertAlmostEqual(command.acceleration.x, 0.)
                    # A delayed copy of the withdrawn curve cannot resurrect
                    # the consumed identity, even with a new future time.
                    future = node.get_clock().now().nanoseconds * 1e-9 + .4
                    publisher.publish(message(
                        4, future, future - start, scheduled=True))
                    spin(.5)
                    self.assertFalse(any(c.trajectory_id == 4 for c in commands))
                    self.assertEqual(commands[-1].trajectory_id, 2)
                    # Unknown withdrawal did not consume 99 or cancel the
                    # legitimate queue; a new identity can still be queued.
                    future = node.get_clock().now().nanoseconds * 1e-9 + .4
                    publisher.publish(message(
                        5, future, future - start, scheduled=True))
                    spin(.05)
                    malformed = message(5, future, future - start)
                    malformed.start_mode = Bspline.CANCEL_PENDING
                    publisher.publish(malformed)
                    withdrawal.traj_id = 4
                    publisher.publish(withdrawal)
                    spin(.45)
                    self.assertEqual(commands[-1].trajectory_id, 5)
                    withdrawal.traj_id = 5  # already active: no effect
                    publisher.publish(withdrawal)
                    spin(.03)
                    self.assertEqual(commands[-1].trajectory_id, 5)
                    self.assertAlmostEqual(commands[-1].velocity.x, 1.)
                    # Immediate checked replacement still cancels a queue.
                    future = node.get_clock().now().nanoseconds * 1e-9 + .4
                    publisher.publish(message(
                        6, future, future - start, scheduled=True))
                    spin(.05)
                    now = node.get_clock().now().nanoseconds * 1e-9
                    publisher.publish(message(
                        7, now, commands[-1].position.x, moving=False))
                    spin(.55)
                    self.assertEqual(commands[-1].trajectory_id, 7)
                    self.assertFalse(
                        any(c.trajectory_id == 6 for c in commands))
                    self.assertAlmostEqual(
                        commands[-1].velocity.x, 0., delta=1e-6)
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', required=True)
    args, remaining = parser.parse_known_args()
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    unittest.main(argv=['scheduled_execution', *remaining])
