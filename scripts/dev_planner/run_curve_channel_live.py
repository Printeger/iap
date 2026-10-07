#!/usr/bin/env python3
"""Canonical forest reference run with owned processes and complete read-only capture."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts/dev_predictor'))
from advisory_validation import (source_identity, binary_identity, preflight,
                                manifest, sha)
from run_directory import resolve_run_directory, finalize_run


def stop(process):
    if process.poll() is not None:
        return
    # These groups were created by this invocation; never discover/kill other ROS jobs.
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--label', default='curve_capture')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 30:
        raise ValueError('duration must be at least 30 seconds')
    identity = source_identity()
    if identity['dirty']:
        raise RuntimeError('LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE: commit task changes first')
    run = resolve_run_directory(entrypoint='iap_sim', scenario='icra_dense_forest_four_fork_v2')
    os.environ['IAP_RUN_DIR'] = str(run)
    os.environ['ROS_LOG_DIR'] = str(run / 'runtime/ros')
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    from ament_index_python.packages import get_package_share_directory, get_package_prefix
    profile = Path(get_package_share_directory('iap')) / 'config/sim_ego/fastdds_udp_only.xml'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
    gpu = preflight(run)
    if gpu.get('gpu_status') != 'READY':
        finalize_run(run, lifecycle='failed')
        raise RuntimeError('GPU_NOT_READY: ' + json.dumps(gpu))
    executable = Path(get_package_prefix('ego_planner')) / 'lib/ego_planner/ego_planner_node'
    command = ['ros2', 'launch', 'iap', 'iap_sim.launch.py', f'run_dir:={run}',
               'scenario:=icra_dense_forest_four_fork_v2', 'start_rviz:=false',
               'start_grid_map_visualizer:=true', 'advisory_posterior_prior:=false',
               'advisory_guidance:=false', 'capture_failure_map:=true',
               f'run_duration_s:={args.duration}']
    manifest(run, 'forest_runtime_identity', {'schema': 'iap_forest_runtime_identity_v1',
             **identity, 'command': command, 'preflight': gpu,
             'runtime_binary': binary_identity(executable.resolve()),
             'environment': {k: os.environ[k] for k in ('IAP_RUN_DIR', 'ROS_LOG_DIR',
                             'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE')}})
    print('IAP_RUN_DIR=' + str(run), flush=True)
    jobs = {}
    outputs = []
    health = []
    started = time.monotonic()
    status = 'failed'
    try:
        def spawn(name, cmd):
            log = (run / 'runtime' / (name + '.log')).open('x')
            outputs.append(log)
            jobs[name] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
        spawn('capture', [sys.executable, str(REPO / 'scripts/dev_predictor/advisory_live_capture.py'),
                          '--duration', str(args.duration + 15), '--label', args.label])
        spawn('launch', command)
        spawn('record', [sys.executable, str(REPO / 'scripts/dev_predictor/advisory_validation.py'),
                         'record', '--label', args.label, '--count', str(math.ceil(args.duration / 10)),
                         '--interval', '10', '--timeout', '5'])
        import psutil
        while jobs['launch'].poll() is None:
            children = psutil.Process(jobs['launch'].pid).children(recursive=True)
            processes = []
            for process in children:
                try:
                    processes.append({'pid': process.pid, 'status': process.status(),
                                      'command': process.cmdline()})
                except psutil.Error:
                    continue
            health.append({'elapsed_s': time.monotonic() - started, 'processes': processes})
            if time.monotonic() - started > args.duration + 60:
                stop(jobs['launch'])
                break
            time.sleep(1)
        for name in ('capture', 'record'):
            try:
                jobs[name].wait(timeout=45)
            except subprocess.TimeoutExpired:
                stop(jobs[name])
        status = 'completed' if all(p.returncode == 0 for p in jobs.values()) else 'failed'
    finally:
        for process in jobs.values():
            stop(process)
        for log in outputs:
            log.close()
        health_path = run / 'export/analysis/process_health.json'
        health_path.write_text(json.dumps(health, indent=2) + '\n')
        manifest(run, 'forest_process_result', {'identity': 'LIVE_REFERENCE_MEASUREMENT',
                 'revision': identity['revision'], 'commands': command,
                 'elapsed_s': time.monotonic() - started,
                 'exit_codes': {n: p.returncode for n, p in jobs.items()},
                 'health_sha256': sha(health_path),
                 'logs_sha256': {str(p.relative_to(run)): sha(p)
                                for p in run.joinpath('runtime').glob('*.log')},
                 'mission_pass': False, 'note': 'process lifecycle is not mission acceptance'})
        primary = json.loads((run / 'metadata/run_manifest.json').read_text())
        if primary['lifecycle'] == 'active':
            finalize_run(run, lifecycle=status)
    print(json.dumps({'run': str(run), 'lifecycle': status}), flush=True)
    return 0 if status == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
