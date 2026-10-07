"""Test installed full-stack node declarations with synthetic CPU inputs."""

import argparse
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped

from iap.msg import IntegrityReport

from launch import LaunchContext, LaunchDescription, LaunchService

from nav_msgs.msg import Odometry

import numpy as np

from quadrotor_msgs.msg import PositionCommand

import rclpy

from sensor_msgs.msg import PointCloud2

from sensor_msgs_py import point_cloud2

from traj_utils.msg import Bspline


def load_runtime():
    """Load the installed canonical simulation declaration."""
    share = Path(get_package_share_directory('iap'))
    path = share / 'launch/_includes/full_stack_runtime.py'
    spec = importlib.util.spec_from_file_location('feedback_runtime', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_fixture():
    """Launch production planner/server declarations with fixture inputs."""
    runtime = load_runtime()
    import run_directory
    run = run_directory.resolve_run_directory(entrypoint='feedback_fixture')
    os.environ['IAP_RUN_DIR'] = str(run)
    context = LaunchContext()
    context.launch_configurations.update({
        'scenario': 'icra_dense_forest_four_fork_v2', 'start_rviz': 'false',
        'start_grid_map_visualizer': 'false', 'planner_start_delay_s': '0',
        'run_duration_s': '0'})
    parameters = runtime.planner_parameters({
        'map_size': [12, 12, 5], 'goal': [2, 0, 1],
        'max_velocity_mps': 1., 'integrity_profile': 'lidar_only',
        'manual_goal': True})
    parameters.update({'grid_map/registered_lidar_window_enabled': False,
                       'grid_map/resolution': .2, 'risk/source': 'gnss',
                       'planning/capture_failure_map': os.environ.get('IAP_FEEDBACK_CAPTURE') == '1'})
    nodes = []
    original_node = runtime.Node

    def capture_node(**kwargs):
        if kwargs['executable'] == 'ego_planner_node':
            kwargs['parameters'] = [parameters]
        action = original_node(**kwargs)
        if kwargs['executable'] in ('ego_planner_node', 'traj_server'):
            nodes.append(action)
        return action

    with mock.patch.object(runtime, 'Node', side_effect=capture_node):
        runtime._setup(context)
    service = LaunchService()
    service.include_launch_description(LaunchDescription(nodes))
    result = service.run()
    run_directory.finalize_run(run, lifecycle='completed',
                               safety_outcome='not_applicable')
    return result


class FullStackFeedback(unittest.TestCase):
    """Exercise the remaps and actual FSM confirmation across processes."""

    def test_server_switch_is_confirmed_without_false_braking(self):
        """Confirm the executing ID after a real scheduled continuation."""
        with tempfile.TemporaryDirectory(prefix='ego_feedback_') as temporary:
            root = Path(temporary)
            env = {**os.environ, 'IAP_RUN_ROOT': str(root / 'runs'),
                   'ROS_LOG_DIR': str(root / 'ros')}
            with open(root / 'launch.log', 'w+') as log:
                process = subprocess.Popen(
                    [sys.executable, __file__, '--launch-child'], env=env,
                    stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                rclpy.init()
                node = rclpy.create_node('full_stack_feedback_probe')
                odom_pub = node.create_publisher(
                    Odometry, '/drone_0_visual_slam/odom', 10)
                cloud_pub = node.create_publisher(
                    PointCloud2, '/grid_map/cloud', 10)
                report_pub = node.create_publisher(
                    IntegrityReport, '/iap/integrity', 10)
                goal_pub = node.create_publisher(
                    PoseStamped, '/move_base_simple/goal', 10)
                curves, commands = [], []
                node.create_subscription(
                    Bspline, '/drone_0_planning/bspline', curves.append, 10)
                node.create_subscription(
                    PositionCommand, '/drone_0_planning/pos_cmd',
                    commands.append, 100)
                position, velocity = [-2., 0., 1.], [0., 0., 0.]
                last_cloud, switched_at = 0., None
                confirmed_id, sent_goal = None, False
                # Explicit sensor-to-return rays in a static room support every
                # direction the terminal region may choose. No voxel is granted
                # observation by the test or inherited from an earlier frame.
                xy = np.arange(-4., 3.41, .1)
                ys = np.arange(-4., 4.01, .1)
                zs = np.arange(.1, 3.11, .1)
                room_returns = ([(x, y, z) for x in (-4., 3.4) for y in ys for z in zs]
                    + [(x, y, z) for y in (-4., 4.) for x in xy for z in zs]
                    + [(x, y, z) for z in (.1, 3.1) for x in xy for y in ys])
                started = time.monotonic()
                try:
                    while time.monotonic() - started < 15:
                        self.assertIsNone(process.poll())
                        # Model measurement/transport latency; a concurrent
                        # snapshot must not see a measurement from its future.
                        measured = (node.get_clock().now()
                                    - rclpy.duration.Duration(seconds=.03))
                        stamp = measured.to_msg()
                        if commands:
                            latest = commands[-1]
                            position = [latest.position.x, latest.position.y,
                                        latest.position.z]
                            velocity = [latest.velocity.x, latest.velocity.y,
                                        latest.velocity.z]
                        odom = Odometry()
                        odom.header.stamp, odom.header.frame_id = stamp, 'map'
                        odom.pose.pose.orientation.w = 1.
                        for index, axis in enumerate('xyz'):
                            setattr(odom.pose.pose.position, axis,
                                    position[index])
                            setattr(odom.twist.twist.linear, axis,
                                    velocity[index])
                        odom_pub.publish(odom)
                        report = IntegrityReport()
                        report.header = odom.header
                        report.current_motion_quality = (
                            IntegrityReport.CURRENT_MOTION_SUPPORTED)
                        report.current_motion_error_proxy_m = .05
                        report_pub.publish(report)
                        if time.monotonic() - last_cloud > .1:
                            cloud = point_cloud2.create_cloud_xyz32(
                                odom.header, room_returns)
                            cloud_pub.publish(cloud)
                            last_cloud = time.monotonic()
                        if (not sent_goal and time.monotonic() - started > 1.
                                and goal_pub.get_subscription_count()):
                            goal = PoseStamped()
                            goal.header = odom.header
                            goal.pose.position.x, goal.pose.position.z = 2., 1.
                            goal.pose.orientation.w = 1.
                            goal_pub.publish(goal)
                            sent_goal = True
                        rclpy.spin_once(node, timeout_sec=.01)
                        pending_ids = {c.traj_id for c in curves
                                       if c.start_mode == Bspline.AT_TIME}
                        if (commands and commands[-1].trajectory_id
                                in pending_ids):
                            confirmed_id = commands[-1].trajectory_id
                            if switched_at is None:
                                switched_at = time.monotonic()
                        if (switched_at is not None
                                and time.monotonic() - switched_at > .4):
                            break
                    log.flush()
                    log.seek(0)
                    output = log.read()
                    if confirmed_id is None:
                        ends = []
                        for c in curves:
                            q = np.array([[p.x, p.y, p.z] for p in c.pos_pts])
                            ends.append({'id': c.traj_id, 'mode': c.start_mode,
                                         'start': ((q[0] + 4*q[1] + q[2])/6).tolist(),
                                         'end': ((q[-3] + 4*q[-2] + q[-1])/6).tolist()})
                        evidence = []
                        for path in sorted(root.rglob('*.json')):
                            if path.name != 'snapshot.json': continue
                            data = json.loads(path.read_text())
                            evidence.append({key: data.get(key) for key in
                                ('kind', 'planning_attempt_id', 'generation',
                                 'search_requested_end_m', 'first_unobserved_position_m',
                                 'curve_execution_reason', 'search_path_cost_m',
                                 'search_terminal_cost_m')})
                        output += '\n' + json.dumps({'published_endpoints': ends,
                            'last_position': position, 'failures': evidence[-5:]})
                    self.assertIsNotNone(confirmed_id, output)
                    self.assertIn(
                        f'Trajectory {confirmed_id} executing at its '
                        'scheduled '
                        'connection', output)
                    self.assertNotIn('connection command missing', output)
                    self.assertNotIn('Checked continuous braking', output)
                    self.assertTrue(curves)
                    self.assertTrue(all(c.start_mode == Bspline.AT_TIME
                                        for c in curves[1:]), output)
                    self.assertGreater(position[0], -1.9)
                    receivers = node.get_subscriptions_info_by_topic(
                        '/drone_0_planning/pos_cmd')
                    self.assertIn('drone_0_ego_planner_node',
                                  {info.node_name for info in receivers})
                    print(f'Confirmed scheduled ID={confirmed_id}; '
                          f'curves={len(curves)} commands={len(commands)}; '
                          'missing-ID stop=0')
                finally:
                    node.destroy_node()
                    rclpy.shutdown()
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--launch-child', action='store_true')
    args, remaining = parser.parse_known_args()
    if args.launch_child:
        sys.exit(launch_fixture())
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    unittest.main(argv=['full_stack_feedback', *remaining])
