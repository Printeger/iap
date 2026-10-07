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
from advisory_validation import (source_identity, binary_identity, installed_build_identity, preflight,
                                manifest, sha, safe_label)
from run_directory import resolve_run_directory, finalize_run, register_subordinate_manifest


def stop(process):
    if process.poll() is not None:
        return
    # These groups were created by this invocation; never discover/kill other ROS jobs.
    def send(sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass  # The owned group already exited between poll and signal.
    send(signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        send(signal.SIGTERM)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            send(signal.SIGKILL)
            process.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--label', default='curve_capture')
    parser.add_argument('--rinex-nav-file', default='', help='Explicit strict historical GPS+BDS input')
    parser.add_argument('--capture-advisory-residuals',action='store_true',help='Opt-in bounded native GPU residual evidence')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 30:
        raise ValueError('duration must be at least 30 seconds')
    safe_label(args.label)
    identity = source_identity()
    if identity['dirty']:
        raise RuntimeError('LIVE_BLOCKED_BY_UNRELATED_DIRTY_WORKTREE: commit task changes first')
    run = resolve_run_directory(entrypoint='iap_sim', scenario='icra_dense_forest_four_fork_v2')
    jobs = {}
    outputs = []
    health = []
    started = time.monotonic()
    status = 'failed'
    command = []
    error = None
    try:
        os.environ['IAP_RUN_DIR'] = str(run)
        os.environ['ROS_LOG_DIR'] = str(run / 'runtime/ros')
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
        from ament_index_python.packages import get_package_share_directory, get_package_prefix
        profile = Path(get_package_share_directory('iap')) / 'config/sim_ego/fastdds_udp_only.xml'
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(profile)
        gpu = preflight(run)
        if gpu.get('gpu_status') != 'READY':
            raise RuntimeError('GPU_NOT_READY: ' + json.dumps(gpu))
        executable = Path(get_package_prefix('ego_planner')) / 'lib/ego_planner/ego_planner_node'
        build_root = REPO.parents[1] / 'build'
        def verified_binary(package, installed, build_name):
            built = build_root / package / build_name
            cache = build_root / package / 'CMakeCache.txt'
            if not cache.is_file() or 'CMAKE_BUILD_TYPE:STRING=Release' not in cache.read_text():
                raise RuntimeError(f'RELEASE_BUILD_REQUIRED: {cache}')
            if not built.is_file():
                raise RuntimeError(f'INSTALLED_BINARY_MISMATCH: {installed} != {built}; build and install before live')
            try:
                match = installed_build_identity(installed, built)
            except (ValueError, KeyError) as error:
                raise RuntimeError(f'INSTALLED_BINARY_MISMATCH: {installed} != {built}: {error}') from error
            result = binary_identity(installed.resolve())
            result.update(match)
            result['build_cache_sha256'] = sha(cache)
            return result
        runtime_binary = verified_binary('ego_planner', executable, 'ego_planner_node')
        input_binaries = {}
        for package, binary in [('gnss_sim', 'gnss_sim_node'),
                                ('so3_quadrotor_simulator', 'so3_quadrotor_simulator'),
                                ('local_sensing', 'pcl_render_node'), ('iap', 'iap_rosnode')]:
            installed = Path(get_package_prefix(package)) / 'lib' / package / binary
            input_binaries[package] = verified_binary(package, installed, binary)
        for plugin in ['gnss_extension', 'integrity_extension',
                       'planner_local_map_extension', 'odometry_estimation_gpu', 'sim_extension']:
            installed = Path(get_package_prefix('iap')) / 'lib' / ('lib' + plugin + '.so')
            input_binaries[plugin] = verified_binary('iap', installed, 'lib' + plugin + '.so')
        installed_core = Path(get_package_prefix('iap')) / 'lib/libiap.so'
        input_binaries['iap_core'] = verified_binary('iap', installed_core, 'libiap.so')
        installed_gpu = Path(get_package_prefix('iap')) / 'lib/libiap_gpu_match_evidence.so'
        input_binaries['gpu_match_evidence'] = verified_binary('iap',installed_gpu,'libiap_gpu_match_evidence.so')
        command = ['ros2', 'launch', 'iap', 'iap_sim.launch.py', f'run_dir:={run}',
                   'run_lifecycle_owner:=driver',
                   'scenario:=icra_dense_forest_four_fork_v2', 'start_rviz:=false',
                   'start_grid_map_visualizer:=true', 'advisory_posterior_prior:=false',
                   'advisory_guidance:=false', 'capture_failure_map:=true',
                   f'run_duration_s:={args.duration}']
        if args.rinex_nav_file:
            command.append('rinex_nav_file:=' + str(Path(args.rinex_nav_file).resolve()))
        if args.capture_advisory_residuals:
            command.append('capture_advisory_residuals:=true')
        manifest(run, 'forest_runtime_identity', {'schema': 'iap_forest_runtime_identity_v1',
                 **identity, 'command': command, 'preflight': gpu,
                 'runtime_binary': runtime_binary,
                 'input_binaries': input_binaries,
                 'environment': {k: os.environ[k] for k in ('IAP_RUN_DIR', 'ROS_LOG_DIR',
                                 'RMW_IMPLEMENTATION', 'FASTRTPS_DEFAULT_PROFILES_FILE')}}, owner=True)
        print('IAP_RUN_DIR=' + str(run), flush=True)
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
            try:
                children = psutil.Process(jobs['launch'].pid).children(recursive=True)
            except psutil.NoSuchProcess:
                children = []
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
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        for process in jobs.values():
            stop(process)
        for log in outputs:
            log.close()
        health_path = run / 'export/analysis/process_health.json'
        health_path.parent.mkdir(parents=True, exist_ok=True)
        health_path.write_text(json.dumps(health, indent=2) + '\n')
        manifest(run, 'forest_process_result', {'identity': 'LIVE_REFERENCE_MEASUREMENT',
                 'revision': identity['revision'], 'commands': command,
                 'elapsed_s': time.monotonic() - started,
                 'exit_codes': {n: p.returncode for n, p in jobs.items()},
                 'health_sha256': sha(health_path),
                 'logs_sha256': {str(p.relative_to(run)): sha(p)
                                for p in run.joinpath('runtime').glob('*.log')},
                 'mission_pass': False, 'error': error,
                 'required_input_failure': (run / 'metadata/manifests/historical_input_failure.json').exists(),
                 'note': 'process lifecycle is not mission acceptance'}, owner=True)
        # The driver is the only finalization owner. Children write subordinate
        # evidence; register every completed manifest after they have stopped.
        for path in sorted((run / 'metadata/manifests').glob('*.json')):
            register_subordinate_manifest(run, path)
        status = finalize_run(run, lifecycle=status)
    print(json.dumps({'run': str(run), 'lifecycle': status}), flush=True)
    return 0 if status == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
