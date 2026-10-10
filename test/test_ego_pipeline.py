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
PARSER.add_argument("--visualizer", required=True)
PARSER.add_argument("--baseline", required=True)
ARGS, EXTRA = PARSER.parse_known_args()
os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from iap.msg import IntegrityReport
from iap.srv import GetGridMapPredictionInput
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
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
    def test_valid_history_reuses_predictions_and_clears_all_topics(self):
        with tempfile.TemporaryDirectory(prefix="ego_display_") as directory:
            root=Path(directory); payload=root/"input.bin"
            env={**os.environ,"ROS_LOG_DIR":str(root/"ros"),"IAP_TEST_PREDICTION_PAYLOAD":str(payload)}
            # Produce wire input with the real shared codec and Predictor fixture.
            subprocess.run([ARGS.baseline,"--gtest_filter=EgoBaseline.ReadOnlyExportUsesSamePredictorWithoutMutatingPlannerCache"],
                           env=env,check=True,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            wire=Path(str(payload)+"_on").read_bytes()
            rclpy.init(args=[]); node=rclpy.create_node("readonly_display_fixture")
            clouds, surfaces, statuses, paths=[],[],[],[]; exports=[]; export_requests=[]
            def export(request,response):
                exports.append(time.monotonic())
                export_requests.append((request.planning_input,request.planning_attempt_id))
                response.available=True; response.frame_id="map"; response.geometry_id="fixture"; response.generation=1
                response.payload=wire; return response
            service=node.create_service(GetGridMapPredictionInput,"/grid_map/prediction_input",export)
            subscriptions=[
                node.create_subscription(PointCloud2,"/grid_map/risk_slice",clouds.append,10),
                node.create_subscription(MarkerArray,"/grid_map/risk_surface",surfaces.append,10),
                node.create_subscription(Marker,"/grid_map/risk_status",statuses.append,10),
                node.create_subscription(NavPath,"/grid_map/glio_path",paths.append,10)]
            log=open(root/"visualizer.log","w+")
            process=subprocess.Popen([ARGS.visualizer],env=env,stdout=log,stderr=subprocess.STDOUT)
            try:
                end=time.monotonic()+5
                while time.monotonic()<end and not (any(c.width for c in clouds) and any(m.points for a in surfaces for m in a.markers if m.ns=="risk_surface") and any("ref=" in m.text for m in statuses)): rclpy.spin_once(node,timeout_sec=.02)
                self.assertTrue(any(c.width for c in clouds),"ongoing input starved current display task")
                self.assertTrue(any(m.points for a in surfaces for m in a.markers if m.ns=="risk_surface"))
                references={m.text.split("ref=")[-1].split(" age=")[0] for m in statuses if "ref=" in m.text}
                client=node.create_client(SetParameters,"/grid_map_visualizer/set_parameters")
                self.assertTrue(client.wait_for_service(timeout_sec=2))
                future=client.call_async(SetParameters.Request(parameters=[Parameter("risk_viz/metric",value="vpl").to_parameter_msg()]))
                end=time.monotonic()+2.2
                while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
                self.assertTrue(future.result().results[0].successful)
                # Identical input is queried once; metric refresh repaints exactly
                # one cloud without creating a new prediction reference.
                self.assertEqual(len([c for c in clouds if c.width]),2)
                self.assertEqual(len({seconds(c.header.stamp) for c in clouds if c.width}),1)
                self.assertGreaterEqual(len(exports),2)
                # A live display must read the retained immutable planning
                # input; fresh exports freeze the map again on the sensor path.
                self.assertTrue(all(retained and attempt==0 for retained,attempt in export_requests),
                                "display requested an extra live-map freeze")
                self.assertTrue(any(" vpl historical " in m.text for m in statuses))
                self.assertEqual(references,{m.text.split("ref=")[-1].split(" age=")[0] for m in statuses if "ref=" in m.text})
                on_cloud=next(c for c in clouds if c.width)
                def valid_hpl(cloud):
                    values=point_cloud2.read_points(cloud,field_names=("hpl","status"),skip_nans=True)
                    return np.asarray(values["hpl"][values["status"]==1],dtype=float)
                on_hpl=valid_hpl(on_cloud)
                self.assertGreater(len(on_hpl),0)
                # Same time/map/observations, only shared input prior changed.
                # The existing identity must invalidate historical PL reuse.
                wire=Path(str(payload)+"_off").read_bytes()
                end=time.monotonic()+3
                while time.monotonic()<end and len([c for c in clouds if c.width])<3:
                    rclpy.spin_once(node,timeout_sec=.02)
                off_cloud=[c for c in clouds if c.width][-1]
                self.assertEqual(seconds(off_cloud.header.stamp),seconds(on_cloud.header.stamp))
                off_hpl=valid_hpl(off_cloud)
                self.assertGreater(len(off_hpl),0)
                self.assertGreater(np.median(off_hpl),np.median(on_hpl)*2)
                clear=node.create_client(Trigger,"/grid_map/clear_risk_history")
                self.assertTrue(clear.wait_for_service(timeout_sec=2))
                future=clear.call_async(Trigger.Request()); end=time.monotonic()+1
                while time.monotonic()<end and not (future.done() and paths and not paths[-1].poses and clouds and not clouds[-1].width): rclpy.spin_once(node,timeout_sec=.02)
                self.assertTrue(future.result().success)
                self.assertTrue(any(m.action==Marker.DELETEALL for a in surfaces for m in a.markers))
                self.assertEqual(clouds[-1].width,0); self.assertFalse(paths[-1].poses)
            except Exception:
                log.flush(); log.seek(0); print(log.read()); raise
            finally:
                process.send_signal(signal.SIGINT); process.wait(timeout=5); log.close()
                node.destroy_node(); rclpy.shutdown()
                self.assertEqual(process.returncode,0)

    def test_goal_publishes_curve_and_matching_commands(self):
        repo = Path(__file__).resolve().parents[1]
        spec = importlib.util.spec_from_file_location("ego_launch", repo / "launch/_includes/full_stack_simulation.launch.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        parameters = module.planner_parameters({"map_size": [12, 12, 5], "goal": [2, 0, 1],
            "max_velocity_mps": 1.0, "integrity_profile": "lidar_only", "manual_goal": True})
        parameters.update({"grid_map/registered_lidar_window_enabled": False,
                           "grid_map/resolution": 0.2,
                           # GNSS measurements are deliberately absent here;
                           # the denser free-space LiDAR rays only establish
                           # physical observation, not valid advisory PL.
                           "risk/source": "gnss"})
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
            integrity_pub = node.create_publisher(IntegrityReport, "/risk/integrity", 10)
            goal_pub = node.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            curves, commands, displayed_curves, risk_clouds, risk_statuses = {}, [], [], [], []
            risk_surfaces, risk_legends, glio_paths = [], [], []
            subscriptions = [
                node.create_subscription(Bspline, "/planning/bspline", lambda m: curves.__setitem__(m.traj_id,m)
                                         if m.start_mode != Bspline.CANCEL_PENDING else None, 10),
                node.create_subscription(PositionCommand, "/position_cmd", commands.append, 100),
                node.create_subscription(Marker, "/planning/trajectory_curve", displayed_curves.append, 10),
                node.create_subscription(PointCloud2, "/grid_map/risk_slice", risk_clouds.append, 10),
                node.create_subscription(Marker, "/grid_map/risk_status", risk_statuses.append, 10),
                node.create_subscription(MarkerArray, "/grid_map/risk_surface", risk_surfaces.append, 10),
                node.create_subscription(MarkerArray, "/grid_map/risk_legend", risk_legends.append, 10),
                node.create_subscription(NavPath, "/grid_map/glio_path", glio_paths.append, 10),
            ]
            parameter_client = node.create_client(SetParameters, "/grid_map_visualizer/set_parameters")
            try:
                for label, command in [
                    ("planner", [ARGS.planner,"--ros-args","--params-file",str(config)]),
                    ("visualizer", [ARGS.visualizer,"--ros-args","-p","risk_viz/metric:=hpl"]),
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
                last_cloud_time=0.0
                while time.monotonic()<deadline:
                    stamp=node.get_clock().now().to_msg()
                    odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id="map"
                    odom.pose.pose.position.x=-2.; odom.pose.pose.position.z=1.
                    odom.pose.pose.orientation.w=1.
                    odom_pub.publish(odom); map_odom_pub.publish(odom)
                    # Cover the three-dimensional short trajectory, including
                    # its B-spline control polygon, with observed free rays.
                    # Sparse rays through the centre do not observe that
                    # volume and correctly fail the planner's unknown test.
                    if time.monotonic()-last_cloud_time>0.2:
                        cloud=point_cloud2.create_cloud_xyz32(
                            odom.header,
                            [(3.4, y, z)
                             for y in np.arange(-1.2, 1.21, 0.2)
                             for z in np.arange(0.4, 1.81, 0.2)])
                        cloud_pub.publish(cloud)
                        last_cloud_time=time.monotonic()
                    report=IntegrityReport(); report.header=odom.header
                    report.current_motion_quality=IntegrityReport.CURRENT_MOTION_SUPPORTED
                    report.current_motion_error_proxy_m=0.05
                    report.hpl=report.vpl=0.3
                    report.hal=0.55; report.val=0.60; report.im=0.25
                    integrity_pub.publish(report)
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
                self.assertTrue(all(
                    cloud.data[i * cloud.point_step + next(
                        field.offset for field in cloud.fields if field.name == "status")]
                    != 1
                    for cloud in risk_clouds
                    for i in range(cloud.width * cloud.height)),
                    "missing spatial advisory input must not be shown as valid PL")
                self.assertTrue(risk_surfaces)
                self.assertFalse(any(marker.action == Marker.DELETEALL for message in risk_surfaces for marker in message.markers),
                                 "ordinary missing PL must preserve historical surfaces")
                self.assertTrue(risk_legends)
                self.assertTrue(glio_paths)
                self.assertEqual(glio_paths[-1].header.frame_id, "map")
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
                # Clearing invalidates every retained display transport, and
                # loss of the independent display process leaves commands live.
                clear_client=node.create_client(Trigger, "/grid_map/clear_risk_history")
                self.assertTrue(clear_client.wait_for_service(timeout_sec=2))
                clear_future=clear_client.call_async(Trigger.Request())
                stop=time.monotonic()+2
                while time.monotonic()<stop and not clear_future.done():
                    rclpy.spin_once(node,timeout_sec=.02)
                self.assertTrue(clear_future.result().success)
                processes[1].send_signal(signal.SIGINT)
                processes[1].wait(timeout=5)
                count=len(commands)
                stop=time.monotonic()+.3
                while time.monotonic()<stop:
                    rclpy.spin_once(node,timeout_sec=.02)
                self.assertGreater(len(commands),count+5)
                self.assertIsNone(processes[0].poll())

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
