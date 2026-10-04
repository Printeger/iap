"""CPU process test: input -> original EGO FSM -> Bspline -> position command."""
import argparse
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import numpy as np
import yaml

PARSER = argparse.ArgumentParser()
PARSER.add_argument("--planner", required=True)
PARSER.add_argument("--server", required=True)
ARGS, EXTRA = PARSER.parse_known_args()
os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from quadrotor_msgs.msg import PositionCommand
from traj_utils.msg import Bspline


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def evaluate(knots, points, degree, t, derivative=0):
    # Independent Cox-de Boor basis evaluation checks both wire serialization
    # and server derivatives; it does not call the planner's C++ spline code.
    knots=np.array(knots,dtype=float)
    points=np.array(points,dtype=float)
    for _ in range(derivative):
        scale=degree / (knots[degree+1:degree+len(points)]-knots[1:len(points)])
        points=np.diff(points,axis=0)*scale[:,None]
        knots=knots[1:-1]
        degree-=1
    basis=((knots[:-1]<=t)&(t<knots[1:])).astype(float)
    for p in range(1,degree+1):
        next_basis=np.zeros(len(basis)-1)
        for j in range(len(next_basis)):
            left=knots[j+p]-knots[j]
            right=knots[j+p+1]-knots[j+1]
            if left>0: next_basis[j]+=(t-knots[j])/left*basis[j]
            if right>0: next_basis[j]+=(knots[j+p+1]-t)/right*basis[j+1]
        basis=next_basis
    return basis @ points


class EgoPipelineTest(unittest.TestCase):
    def test_goal_publishes_curve_and_matching_commands(self):
        repo = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("ego_launch", repo / "launch/_includes/full_stack_simulation.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        parameters = module.planner_parameters({"map_size": [12, 12, 5], "goal": [2, 0, 1],
            "max_velocity_mps": 1.0, "integrity_profile": "lidar_only", "manual_goal": True})
        parameters.update({"grid_map/registered_lidar_window_enabled": False,
                           "grid_map/resolution": 0.2})
        with tempfile.TemporaryDirectory(prefix="ego_pipeline_") as directory:
            root = Path(directory)
            config = root / "parameters.yaml"
            config.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
            env = {**os.environ, "ROS_LOG_DIR": str(root / "ros")}
            processes = []
            logs = []
            os.environ["ROS_LOG_DIR"] = str(root / "ros")
            rclpy.init(args=[])
            node = rclpy.create_node("ego_pipeline_probe")
            odom_pub = node.create_publisher(Odometry, "/odom_world", 10)
            map_odom_pub = node.create_publisher(Odometry, "/grid_map/odom", 10)
            cloud_pub = node.create_publisher(PointCloud2, "/grid_map/cloud", 10)
            goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            curves, commands, displayed_curves, risk_clouds, risk_statuses = {}, [], [], [], []
            subscriptions = [
                node.create_subscription(Bspline, "/planning/bspline", lambda m: curves.__setitem__(m.traj_id,m), 10),
                node.create_subscription(PositionCommand, "/position_cmd", commands.append, 100),
                node.create_subscription(Marker, "/planning/trajectory_curve", displayed_curves.append, 10),
                node.create_subscription(PointCloud2, "/grid_map/risk_slice", risk_clouds.append, 10),
                node.create_subscription(Marker, "/grid_map/risk_status", risk_statuses.append, 10),
            ]
            parameter_client = node.create_client(SetParameters, "/ego_planner_node/set_parameters")
            try:
                for label, command in [
                    ("planner", [ARGS.planner,"--ros-args","--params-file",str(config)]),
                    ("server", [ARGS.server,"--ros-args","-p","frame_id:=map","-p","traj_server/time_forward:=1.0"]),
                ]:
                    log = open(root / f"{label}.log", "w+")
                    logs.append(log)
                    processes.append(subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT))
                deadline=time.monotonic()+20
                sent_goal=False
                saw_risk_before_goal=False
                changed_metric=False
                metric_future=None
                start=time.monotonic()
                while time.monotonic()<deadline:
                    stamp=node.get_clock().now().to_msg()
                    odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id="map"
                    odom.pose.pose.position.x=-2.; odom.pose.pose.position.z=1.
                    odom.pose.pose.orientation.w=1.
                    odom_pub.publish(odom); map_odom_pub.publish(odom)
                    cloud=PointCloud2(); cloud.header=odom.header; cloud.height=1
                    cloud_pub.publish(cloud)
                    if risk_statuses and not sent_goal:
                        saw_risk_before_goal=True
                    if risk_statuses and not changed_metric and parameter_client.service_is_ready():
                        request=SetParameters.Request()
                        request.parameters=[Parameter("risk_viz/metric", value="vpl").to_parameter_msg()]
                        metric_future=parameter_client.call_async(request)
                        changed_metric=True
                    if not sent_goal and time.monotonic()-start>2.5 and goal_pub.get_subscription_count():
                        goal=PoseStamped(); goal.header=odom.header
                        goal.pose.position.x=2.; goal.pose.position.z=1.; goal.pose.orientation.w=1.
                        goal_pub.publish(goal); sent_goal=True
                    rclpy.spin_once(node,timeout_sec=0.02)
                    if (len(commands)>=20 and any(c.position.x>-1.85 for c in commands)
                            and displayed_curves and risk_clouds and saw_risk_before_goal
                            and any(" vpl " in m.text for m in risk_statuses)):
                        break
                    self.assertTrue(all(p.poll() is None for p in processes),"planner/server exited")
                self.assertTrue(curves,"FSM published no trajectory")
                self.assertGreaterEqual(len(commands),20)
                self.assertTrue(saw_risk_before_goal, "risk slice did not run while idle")
                self.assertTrue(metric_future.done())
                self.assertTrue(metric_future.result().results[0].successful)
                self.assertTrue(any(" vpl " in m.text for m in risk_statuses))
                self.assertTrue(risk_clouds)
                self.assertTrue(all(cloud.width == 0 for cloud in risk_clouds),
                                "missing monitor input must not be shown as valid PL")
                self.assertEqual({f.name for f in risk_clouds[-1].fields},
                                 {"x","y","z","rgb","hpl","vpl","status"})
                self.assertTrue(displayed_curves)
                self.assertEqual(displayed_curves[-1].header.frame_id,"map")
                self.assertGreater(len(displayed_curves[-1].points),2)
                checked=0
                for cmd in commands:
                    if cmd.trajectory_id not in curves: continue
                    curve=curves[cmd.trajectory_id]
                    points=np.array([[p.x,p.y,p.z] for p in curve.pos_pts])
                    elapsed=seconds(cmd.header.stamp)-seconds(curve.start_time)
                    t=curve.knots[curve.order]+elapsed
                    if not curve.knots[curve.order]<=t<curve.knots[-curve.order-1]: continue
                    self.assertEqual(cmd.header.frame_id,"map")
                    for value,order in [(cmd.position,0),(cmd.velocity,1),(cmd.acceleration,2)]:
                        actual=np.array([value.x,value.y,value.z])
                        self.assertTrue(np.isfinite(actual).all())
                        np.testing.assert_allclose(actual,evaluate(curve.knots,points,curve.order,t,order),atol=2e-5,rtol=1e-5)
                    checked+=1
                self.assertGreaterEqual(checked,10)
            except Exception:
                for log in logs:
                    log.flush(); log.seek(0); print(log.read()[-6000:])
                raise
            finally:
                for p in processes:
                    if p.poll() is None: p.send_signal(signal.SIGINT)
                for p in processes:
                    try: p.wait(timeout=5)
                    except subprocess.TimeoutExpired: p.kill(); p.wait()
                for log in logs: log.close()
                node.destroy_node(); rclpy.shutdown()
                self.assertTrue(all(p.returncode == 0 for p in processes),
                                f"planner/server shutdown codes: {[p.returncode for p in processes]}")


if __name__ == "__main__":
    unittest.main(argv=[__file__,*EXTRA])
