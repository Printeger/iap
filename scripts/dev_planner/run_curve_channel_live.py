#!/usr/bin/env python3
"""Canonical forest reference run with owned processes and complete read-only capture."""
import argparse
import csv
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


def finalized_planning_inputs(run, require_search=False):
    """Read only atomically published, registered captures; keep invalid inputs."""
    inputs=[]
    for payload in sorted((run/'export/planner/failure_map').glob('*/planning_input.bin')):
        if payload.parent.name.endswith('.pending'): continue
        snapshot=payload.parent/'snapshot.json'
        manifest_path=run/'metadata/manifests'/('planner_failure_map_'+payload.parent.name+'.json')
        if not snapshot.is_file() or not manifest_path.is_file(): continue
        metadata=json.loads(snapshot.read_text())
        registration=json.loads(manifest_path.read_text())
        if registration.get('planning_input')!=str(payload.relative_to(run/'export')): continue
        if require_search and (not metadata.get('planning_goals_m') or not metadata.get('search_stage') or
            metadata.get('motion_quality')!=1 or not math.isfinite(metadata.get('motion_stamp_s') or float('nan'))):
            continue
        inputs.append((payload,snapshot,metadata))
    return inputs


def task_reached(run):
    for path in (run/'profiling').glob('planner_execution_*.csv'):
        with path.open() as stream:
            if any(row.get('event')=='task_reached' for row in csv.DictReader(stream)): return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=300)
    parser.add_argument('--label', default='curve_capture')
    parser.add_argument('--guidance',choices=('false','true'),default='false')
    parser.add_argument('--visualization',choices=('none','map','full'),default='none',
                        help='Display only: map enables PL display, full also starts RViz; planning inputs and budgets unchanged')
    parser.add_argument('--stop-after-planning-input',action='store_true',help='bounded first-input capture; stop owned launch when input.bin is saved')
    parser.add_argument('--rinex-nav-file', default='', help='Explicit strict historical GPS+BDS input')
    parser.add_argument('--capture-fork-entries',action='store_true',help='freeze physical fork entrance inputs with a read-only helper')
    parser.add_argument('--stop-after-task',action='store_true',help='stop owned jobs when the original FSM records task_reached')
    parser.add_argument('--capture-advisory-residuals',action='store_true',help='Opt-in bounded native GPU residual evidence')
    parser.add_argument('--capture-dense-lidar',action='store_true',help='Capture latest fixed level-0 target source slots, max 16384')
    parser.add_argument('--capture-source-observations',action='store_true',help='Retain actual source sampling geometry in selected inputs; no GPU trace required')
    parser.add_argument('--capture-native-response-frames', default='', help='M2 OFF: 1..31 predeclared native frame IDs, plus initial x0 elimination')
    parser.add_argument('--capture-native-source-freeze',action='store_true',help='M2 OFF: first nonzero source fixed-use boundary, one rolling packet and at most five extra snapshots')
    parser.add_argument('--capture-native-increments',action='store_true',help='M2 OFF: one-second input pairs and selected native postopt/IMU reports; no dense GPU trace')
    parser.add_argument('--omit-planner-map-dumps',action='store_true',help='M2 source-observation diagnostics only: omit repetitive planner maps and internal input dumps, retain failure ledgers')
    parser.add_argument('--record-count',type=int,default=None,help='Number of frozen inputs; default retains original 10s cadence')
    parser.add_argument('--record-initial-delay',type=float,default=0.,help='Predeclared first snapshot delay, seconds')
    parser.add_argument('--gnss-seed', type=int, default=None, help='Independent simulation observation seed; all noise/planner settings retained')
    parser.add_argument('--execution-evidence-ids', default='', help='Opt-in comma-separated owning IDs; retain existing immutable final/refusal corridor evidence')
    parser.add_argument('--protection-margin-replan',action='store_true',help='Experimental early physical-reserve scheduling; keeps original hard checks and total budget')
    args = parser.parse_args()
    if args.capture_dense_lidar and not args.capture_advisory_residuals:
        raise ValueError('--capture-dense-lidar requires --capture-advisory-residuals')
    if args.omit_planner_map_dumps and (not args.capture_source_observations or args.guidance != 'false'):
        raise ValueError('--omit-planner-map-dumps requires OFF source-observation diagnostics')
    if args.capture_native_increments and (not args.capture_source_observations or args.guidance != 'false' or
            (args.record_count is not None and args.record_count % 2)):
        raise ValueError('native increment diagnostics require OFF source observations and even total input count')
    if args.capture_native_response_frames and (not args.capture_native_increments or args.guidance != "false"):
        raise ValueError("native response capture requires OFF native increment diagnostics")
    if args.capture_native_source_freeze and not args.capture_native_response_frames:
        raise ValueError("source freeze capture requires native response selection")
    if args.stop_after_planning_input: args.duration=30.
    if not math.isfinite(args.duration) or args.duration < 30:
        raise ValueError('duration must be at least 30 seconds')
    if args.record_count is not None and args.record_count < 1:
        raise ValueError('record count must be positive')
    if not math.isfinite(args.record_initial_delay) or not 0 <= args.record_initial_delay < args.duration-5:
        raise ValueError('record delay must leave time for the source request')
    safe_label(args.label)
    from full_stack_runtime import gnss_observation_seed
    gnss_observation_seed(args.gnss_seed)
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
    intentional_stops=set()
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
        if args.visualization != 'none':
            installed = Path(get_package_prefix('ego_planner')) / 'lib/ego_planner/grid_map_visualizer'
            input_binaries['grid_map_visualizer'] = verified_binary('ego_planner', installed, 'grid_map_visualizer')
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
                   'scenario:=icra_dense_forest_four_fork_v2', 'start_rviz:='+str(args.visualization=='full').lower(),
                   'start_grid_map_visualizer:='+str(args.visualization!='none').lower(), 'advisory_posterior_prior:=false',
                   'advisory_guidance:='+args.guidance, 'capture_failure_map:='+str(not args.omit_planner_map_dumps).lower(),
                   f'run_duration_s:={args.duration}']
        if args.rinex_nav_file:
            command.append('rinex_nav_file:=' + str(Path(args.rinex_nav_file).resolve()))
        if args.capture_advisory_residuals:
            command.append('capture_advisory_residuals:=true')
        if args.capture_dense_lidar:
            command.append('capture_dense_lidar:=true')
        if args.capture_native_response_frames:
            command.append('capture_native_response_frames:=' + args.capture_native_response_frames)
        if args.capture_native_source_freeze:
            command.append('capture_native_source_freeze:=true')
        if args.capture_source_observations:
            command.append('capture_source_observations:=true')
        if args.gnss_seed is not None:
            command.append('gnss_observation_seed:=' + str(args.gnss_seed))
        if args.execution_evidence_ids:
            command.append('execution_evidence_ids:=' + args.execution_evidence_ids)
        if args.protection_margin_replan:
            command.append('protection_margin_replan:=true')
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
        capture_command = [sys.executable, str(REPO / 'scripts/dev_predictor/advisory_live_capture.py'),
                          '--duration', str(args.duration + 15), '--label', args.label]
        if args.capture_native_increments and args.omit_planner_map_dumps:
            capture_command += ['--compact-source-diagnostics']
        spawn('capture', capture_command)
        spawn('launch', command)
        launch_started=time.monotonic()
        record_command = [sys.executable, str(REPO / 'scripts/dev_predictor/advisory_validation.py'),
                         'record', '--label', args.label, '--count', str(args.record_count or (2 if args.capture_native_increments else 1)*math.ceil(args.duration / 10)),
                         '--initial-delay',str(args.record_initial_delay),
                         '--interval', '10', '--timeout', '5']
        if args.capture_native_increments:
            record_command += ['--capture-native-increments', '--pair-window', '1']
        spawn('record', record_command)
        if args.capture_fork_entries:
            spawn('forks', [sys.executable, str(REPO / 'scripts/dev_predictor/record_fork_inputs.py'),
                            '--duration', str(args.duration+15)])
        import psutil
        captured_input=False
        while jobs['launch'].poll() is None:
            captured_input=bool(args.stop_after_planning_input and finalized_planning_inputs(run,require_search=True))
            if (args.stop_after_task and task_reached(run)) or (args.stop_after_planning_input and (captured_input or time.monotonic()-launch_started>=30.)):
                for name in jobs:
                    if jobs[name].poll() is None:
                        intentional_stops.add(name); stop(jobs[name])
                break
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
        for name in (n for n in jobs if n!='launch'):
            try:
                jobs[name].wait(timeout=45)
            except subprocess.TimeoutExpired:
                stop(jobs[name])
        healthy=all(p.returncode==0 or (n in intentional_stops and p.returncode in (-signal.SIGINT,130))
                    for n,p in jobs.items())
        status = 'completed' if healthy and (not args.stop_after_planning_input or captured_input) else 'failed'
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
        planning_inputs=[]
        for payload,snapshot,metadata in finalized_planning_inputs(run):
            planning_inputs.append({'payload':str(payload.relative_to(run)),'payload_sha256':sha(payload),
                'snapshot':str(snapshot.relative_to(run)),'snapshot_sha256':sha(snapshot),
                'planning_attempt_id':metadata['planning_attempt_id'],
                'risk_version':metadata['planning_input_risk_version'],
                'generation':metadata['generation'],'reference_time_s':metadata['planning_time_s']})
        manifest(run,'planning_input_capture',{'source':identity,'inputs':planning_inputs,
            'first_input_only_requested':args.stop_after_planning_input,
            'status':'CAPTURED' if planning_inputs else 'NO_MATCHING_PLANNING_INPUT'},owner=True)
        if args.capture_advisory_residuals or args.capture_native_source_freeze:
            noise_path = run / 'export/simulation/gnss_injected_noise.csv'
            manifest(run, 'gnss_injected_noise_capture', {
                'path': str(noise_path.relative_to(run)),
                'sha256': sha(noise_path) if noise_path.is_file() else None,
                'status': 'CAPTURED' if noise_path.is_file() else 'MISSING',
                'scope': 'simulator injection draws; postfit residuals are separate evidence',
            }, owner=True)
        manifest(run, 'forest_process_result', {'identity': 'LIVE_REFERENCE_MEASUREMENT',
                 'revision': identity['revision'], 'commands': command,
                 'elapsed_s': time.monotonic() - started,
                 'exit_codes': {n: p.returncode for n, p in jobs.items()},
                 'intentional_stops':sorted(intentional_stops),
                 'task_reached_observed':task_reached(run),
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
