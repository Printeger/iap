#!/usr/bin/env python3
"""Read-only freezes at physical fork entrances; no risk-labelled trigger."""
import argparse
import json
import os
import time
from pathlib import Path
from advisory_validation import (adopt_run_directory, artifact, binary_identity,
                                 json_write, manifest, save_record, sha, source_identity)
from run_directory import canonical_run_uses_sim_time


def entrance(position, geometry):
    for index in range(int(geometry['forked_forest.fork_count'])):
        x = geometry['forked_forest.fork_x_min_m'] + index * geometry['forked_forest.fork_length_m']
        if abs(position[0]-x) <= .5 and abs(position[1]) <= geometry['forked_forest.junction_clearance_radius_m']:
            return index
    return None


def observe_entrances(sample, geometry, entries, request_pending):
    """Retain reached/missed evidence on every odometry, even during a request."""
    if not geometry:
        return
    index = entrance(sample['position_m'], geometry)
    for entry in entries:
        if entry['status'] == 'NOT_REACHED' and sample['position_m'][0] > entry['entrance_x_m']+.5:
            entry.update(status='CAPTURE_FAILED', reason='entrance_passed_without_capture')
    if index is not None and entries[index]['status'] == 'NOT_REACHED':
        entries[index].update(status='REACHED', trigger=dict(sample))
        if request_pending:
            entries[index].update(status='CAPTURE_FAILED', reason='input_service_busy_at_entrance')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, required=True)
    args = parser.parse_args()
    run = adopt_run_directory(os.environ['IAP_RUN_DIR'])
    primary = json.loads((run/'metadata/run_manifest.json').read_text())
    identity = source_identity()
    if (identity['dirty'] or identity['revision'] != primary['source']['git_commit'] or
            primary['source'].get('git_worktree_clean') is not True or
            primary.get('entrypoint') != 'iap_sim' or
            primary['scenario'] != 'icra_dense_forest_four_fork_v2'):
        raise RuntimeError('clean canonical source identity required')
    import rclpy
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.executors import ExternalShutdownException
    from rcl_interfaces.srv import GetParameters
    from iap.srv import GetGridMapPredictionInput
    from nav_msgs.msg import Odometry
    from ament_index_python.packages import get_package_prefix
    identity['producer_binary'] = binary_identity((Path(get_package_prefix('ego_planner'))/'lib/ego_planner/ego_planner_node').resolve())
    rclpy.init()
    node = rclpy.create_node('fork_input_recorder', parameter_overrides=[
        Parameter('use_sim_time', value=canonical_run_uses_sim_time(run))])
    started = time.monotonic()
    geometry = {}
    entries = [{'fork_index': i, 'status': 'CAPTURE_FAILED',
                'reason': 'physical_geometry_unavailable'} for i in range(4)]
    latest = None
    pending = None
    failure = None
    names = ['fork_count', 'fork_x_min_m', 'fork_length_m', 'low_risk_amplitude_m',
             'high_risk_amplitude_m', 'corridor_width_m', 'junction_clearance_radius_m', 'risk_seed']
    names = ['forked_forest.'+name for name in names]
    def receive(msg):
        nonlocal latest
        p = msg.pose.pose.position
        latest = {'position_m': [p.x,p.y,p.z], 'stamp_s': msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                  'frame_id': msg.header.frame_id, 'receive_ros_s': node.get_clock().now().nanoseconds*1e-9}
        observe_entrances(latest, geometry, entries, pending is not None)
    subscription = node.create_subscription(Odometry, '/drone_0_visual_slam/odom', receive, 10)
    try:
        params = node.create_client(GetParameters, '/iap_sim_map_publisher/get_parameters')
        if not params.wait_for_service(timeout_sec=15):
            raise RuntimeError('physical scene parameter service unavailable')
        request = GetParameters.Request(); request.names = names
        future = params.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or len(future.result().values) != len(names):
            raise RuntimeError('physical scene parameter capture incomplete')
        geometry = dict(zip(names, map(parameter_value_to_python, future.result().values)))
        if any(v is None for v in geometry.values()) or geometry[names[0]] != 4:
            raise RuntimeError('unsupported physical fork geometry')
        entries = [{'fork_index': i, 'entrance_x_m': geometry[names[1]]+i*geometry[names[2]],
                    'status': 'NOT_REACHED'} for i in range(4)]
        client = node.create_client(GetGridMapPredictionInput, '/grid_map/prediction_input')
        while time.monotonic()-started < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
            if pending:
                index, future, issued = pending
                if future.done() or time.monotonic()-issued > 5:
                    response = future.result() if future.done() else None
                    entry = entries[index]
                    entry['reason'] = response.reason if response else 'input_service_timeout'
                    if response and response.available:
                        path = save_record(run, f'fork_entry_{index}', response.payload, identity,
                            {'frame_id': response.frame_id, 'geometry_id': response.geometry_id,
                             'generation': response.generation, 'planning_input': False,
                             'planning_attempt_id': response.planning_attempt_id, 'risk_version': response.risk_version,
                             'entry_trigger': entry['trigger'], 'physical_geometry': geometry})
                        entry.update(status='CAPTURED', payload=str(path.relative_to(run)), payload_sha256=sha(path))
                    else:
                        entry['status'] = 'CAPTURE_FAILED'
                    pending = None
            if pending is None:
                for index, entry in enumerate(entries):
                    if entry['status'] != 'REACHED':
                        continue
                    entry['status'] = 'CAPTURE_REQUESTED'
                    if client.service_is_ready():
                        pending = (index, client.call_async(GetGridMapPredictionInput.Request()), time.monotonic())
                    else:
                        entry.update(status='CAPTURE_FAILED', reason='input_service_unavailable')
                    break
            if entries and all(e['status'] in ('CAPTURED','CAPTURE_FAILED') for e in entries) and pending is None:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        failure = str(exc)
        raise
    finally:
        for entry in entries:
            if entry['status'] in ('REACHED', 'CAPTURE_REQUESTED'):
                entry.update(status='CAPTURE_FAILED', reason='owned_stop_during_capture')
        node.destroy_node(); rclpy.try_shutdown()
        root = artifact(run, 'export/advisory/forks/entrances.json')
        json_write(root, {'identity':'FORK_INPUT_CAPTURE', 'physical_geometry':geometry, 'entries':entries,
                          'error':failure, 'trigger_rule':'abs(x-entry)<=0.5m and abs(y)<=physical junction radius'})
        manifest(run, 'fork_inputs', {'source':identity, 'collector_sha256':sha(Path(__file__)), 'entries':entries, 'error':failure,
                 'artifacts_sha256':{str(root.relative_to(run)):sha(root)}})


if __name__ == '__main__':
    main()
