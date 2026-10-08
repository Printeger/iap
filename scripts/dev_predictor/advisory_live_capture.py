#!/usr/bin/env python3
"""Read-only ROS evidence capture inside the canonical launch's run directory."""
import argparse
import json
import math
import os
import time
from collections import Counter
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from advisory_validation import adopt_run_directory, artifact, json_write, manifest, sha, safe_label
from run_directory import canonical_run_uses_sim_time
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException


def plain(value):
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    if hasattr(value,'tolist'):return plain(value.tolist())
    return value


def main():
    p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=170);p.add_argument('--label',default='survey')
    args=p.parse_args();safe_label(args.label)
    run=adopt_run_directory(os.environ['IAP_RUN_DIR']);primary=json.loads((run/'metadata/run_manifest.json').read_text())
    if primary['entrypoint']!='iap_sim' or primary['scenario']!='icra_dense_forest_four_fork_v2':
        raise ValueError('canonical live run required')
    target=artifact(run,'export/planner/advisory_validation/'+args.label+'_events.jsonl')
    subscriptions={};counts=Counter();started=time.monotonic()
    rclpy.init();node=rclpy.create_node('advisory_live_capture', parameter_overrides=[
        Parameter('use_sim_time', value=canonical_run_uses_sim_time(run))])
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        with target.open('x',buffering=1) as out:
            def receive(topic,msg):
                if msg.__class__.__name__=='PointCloud2':
                    # Time/source evidence only. Full frozen predictor inputs
                    # remain owned by the existing input recorder.
                    payload={'identity':'POINTCLOUD_HEADER_ONLY',
                             'header':plain(message_to_ordereddict(msg.header)),
                             'height':msg.height,'width':msg.width,
                             'point_step':msg.point_step,'row_step':msg.row_step,
                             'data_bytes':len(msg.data)}
                else:
                    payload=plain(message_to_ordereddict(msg))
                if msg.__class__.__name__=='String':
                    try:payload=json.loads(msg.data)
                    except json.JSONDecodeError:pass
                out.write(json.dumps({'topic':topic,'receive_steady_s':time.monotonic(),
                           'receive_ros_s':node.get_clock().now().nanoseconds*1e-9,'payload':plain(payload)},allow_nan=False)+'\n')
                counts[topic]+=1
            next_discovery=0.
            while time.monotonic()-started<args.duration:
                if time.monotonic()>next_discovery:
                    for topic,types in node.get_topic_names_and_types():
                        if topic in subscriptions or len(types)!=1:continue
                        selected=(topic=='/iap/integrity' or topic in ('/clock','/drone_0_visual_slam/odom','/sim/drone_0/truth_odom',
                                  '/ublox_driver/range_meas','/ublox_driver/ephem','/ublox_driver/glo_ephem',
                                  '/sim/drone_0/lidar_body',
                                  '/drone_0_planning/bspline','/drone_0_planning/pos_cmd',
                                  '/sim/drone_0/imu_iap') or types[0]=='quadrotor_msgs/msg/SO3Command' or
                                  ('planning' in topic and types[0]=='std_msgs/msg/String'))
                        if selected:
                            subscriptions[topic]=node.create_subscription(get_message(types[0]),topic,
                                 lambda msg,t=topic:receive(t,msg),qos)
                    next_discovery=time.monotonic()+1.
                rclpy.spin_once(node,timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Owned driver stop; finalize complete evidence below.
    finally:
        node.destroy_node();rclpy.try_shutdown()
        manifest(run,'advisory_capture_'+args.label,{'identity':'LIVE_MEASUREMENT','source':primary['source'],
             'counts':dict(counts),'duration_s':time.monotonic()-started,'artifacts_sha256':{str(target.relative_to(run)):sha(target)}})
    print(json.dumps({'path':str(target),'counts':dict(counts)}))

if __name__=='__main__':main()
