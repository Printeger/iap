#!/usr/bin/env python3
"""Pair original state-time truth and posterior evidence; never qualify by retiming."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
from advisory_validation import (sha, binary_identity, source_identity, safe_label,
                                 adopt_run_directory, manifest)
from advisory_coordinate_evidence import audit
from compare_advisory_error import interpolate_truth
from run_directory import write_subordinate_manifest


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def posterior_at_state(meta):
    e = meta['postopt_evidence']
    dimensions = e['tangent_dimensions']
    covariance = np.array(e['joint_covariance_row_major']).reshape(sum(dimensions), -1)
    linear = np.array(e['linearization_means'][:16]).reshape(4, 4)
    t_map_world = np.array(meta['coordinates']['T_map_world']).reshape(4, 4)
    j = np.zeros((3, sum(dimensions)))
    j[:, 3:6] = t_map_world[:3, :3] @ linear[:3, :3]
    position = j @ covariance @ j.T
    delta = meta['reference_time_s']-e['state_stamp']
    f = j.copy()
    f[:, 6:9] = delta*t_map_world[:3, :3]
    # Preserve the p-v cross terms. Q/acceleration/attitude propagation is
    # unavailable; this CV projection is diagnostic, never a propagated bound.
    cv_no_q = f @ covariance @ f.T
    shift = delta*t_map_world[:3, :3] @ np.array(e['optimized_means'][16:19])
    return position, cv_no_q, shift


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    safe_label(args.label)
    if not os.environ.get('IAP_RUN_DIR'):
        parser.error('adopt the current audit IAP_RUN_DIR')
    run = adopt_run_directory(os.environ['IAP_RUN_DIR'])
    source = args.source_run.resolve()
    primary = json.loads((source/'metadata/run_manifest.json').read_text())
    if (primary['scenario'] != 'icra_dense_forest_four_fork_v2' or
            not primary['source']['git_worktree_clean']):
        raise ValueError('real clean canonical source run required')
    recordings = source/'export/advisory/validation/recordings'
    odom = [r for p in recordings.glob('*_odometry.csv') for r in rows(p)]
    truth = [r for r in odom if r['source'] == 'truth' and r['frame'] == 'map' and r['body'] == 'drone_0']
    glio = [r for r in odom if r['source'] == 'glio']
    results = []
    commands = []
    inputs = {}
    for path in [source/'metadata/run_manifest.json', *recordings.glob('*_odometry.csv')]:
        inputs[str(path)] = sha(path)
    # Authoritative request lists include failures; no deletion or censoring.
    for request_manifest in sorted(recordings.glob('*_requests_manifest.json')):
        authority = json.loads(request_manifest.read_text())
        table = source/authority['requests_csv']
        if sha(table) != authority['requests_csv_sha256']:
            raise ValueError('request table checksum mismatch')
        values = rows(table)
        if [r['request_id'] for r in values] != authority['request_ids']:
            raise ValueError('complete request denominator mismatch')
        inputs[str(request_manifest)] = sha(request_manifest)
        inputs[str(table)] = sha(table)
        for request in values:
            item = {'request_id': request['request_id'], 'source_run': source.name,
                    'payload': request.get('payload', ''), 'qualified': False,
                    'raw_state_pair': False, 'reason': request.get('reason', ''),
                    'reference_time_qualified': False}
            if not request.get('payload'):
                results.append(item)
                continue
            payload = source/request['payload']
            info = json.loads(payload.with_name('record.json').read_text())
            if info['identity'] != 'REAL_FROZEN' or info['dirty'] or info['payload_sha256'] != sha(payload):
                raise ValueError('immutable real input sidecar mismatch')
            inputs[str(payload)] = sha(payload)
            inputs[str(payload.with_name('record.json'))] = sha(payload.with_name('record.json'))
            label = args.label+'_'+safe_label(payload.parent.name)
            command = [str(args.binary.resolve()), 'replay', label, str(payload), '120']
            commands.append(command)
            logfile = run/'runtime/ros'/('error_replay_'+label+'.log')
            with logfile.open('x') as stream:
                code = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT).returncode
            item['replay_exit_code'] = code
            if code:
                item['reason'] = 'native_replay_failed'
                results.append(item)
                continue
            directory = run/'export/advisory/validation'/(label+'_current')
            meta = json.loads((directory/'input.json').read_text())
            row = rows(directory/'points.csv')[0]
            try:
                coordinate = audit(meta)
                stamp = meta['pose_stamp']
                matching = [r for r in glio if abs(float(r['stamp'])-stamp) <= 1e-6]
                if not matching:
                    raise ValueError('raw_GLIO_state_identity_unmatched')
                world = np.array(meta['coordinates']['T_world_imu']).reshape(4, 4)
                transform = np.array(meta['coordinates']['T_map_world']).reshape(4, 4)
                # Recorder subscribes /drone_0_visual_slam/odom, whose producer
                # already applies T_map_world. Do not apply that transform twice.
                if not any(r['frame'] == 'map' and np.allclose(
                        [float(r[k]) for k in 'xyz'], (transform@world)[:3, 3],
                        atol=1e-8, rtol=0) for r in matching):
                    raise ValueError('recorded_GLIO_prediction_map_pose_mismatch')
                if not np.allclose((transform@world)[:3, 3], meta['position'], atol=1e-8, rtol=0):
                    raise ValueError('world_to_prediction_map_pose_mismatch')
                # These captures are the canonical known IMU/drone simulation
                # translation with identity body extrinsic. Verify the actual
                # frozen transform, not a fitted trajectory alignment.
                expected = np.eye(4);expected[:3, 3] = [-18., 0., 1.5]
                if not np.allclose(transform, expected, atol=1e-9, rtol=0):
                    raise ValueError('canonical_fixed_truth_map_transform_unproved')
                if not np.allclose(np.array(meta['coordinates']['T_lidar_imu']).reshape(4, 4), np.eye(4), atol=1e-9, rtol=0):
                    raise ValueError('canonical_body_extrinsic_unproved')
                position, _ = interpolate_truth(truth, stamp)
                difference = np.array(meta['position'])-position
                posterior, cv_no_q, shift = posterior_at_state(meta)
                item.update(raw_state_pair=True, pose_stamp_s=stamp,
                            reference_time_s=meta['reference_time_s'], gnss_stamp_s=meta['gnss_stamp'],
                            reference_pose_delta_s=meta['reference_time_s']-stamp,
                            state_epoch_delta_s=stamp-meta['gnss_stamp'],
                            reference_time_qualified=coordinate['calibration_time_qualified'],
                            error_h_m=float(np.linalg.norm(difference[:2])), error_v_m=float(abs(difference[2])),
                            error_xyz_m=difference.tolist(), predictor_valid=row['valid']=='1',
                            hpl_m=float(row['fused_hpl']) if row['fused_hpl'] else None,
                            vpl_m=float(row['fused_vpl']) if row['fused_vpl'] else None,
                            posterior_position_covariance_m2=posterior.tolist(),
                            cv_no_process_noise_covariance_diagnostic_m2=cv_no_q.tolist(),
                            cv_mean_shift_diagnostic_m=shift.tolist(),
                            map_up_enu_angle_deg=coordinate['map_up_enu_angle_deg'],
                            reason='REFERENCE_STATE_PROPAGATION_NOISE_MAP_AND_FUSED_FAULTS_UNQUALIFIED')
            except ValueError as error:
                item['reason'] = str(error)
            results.append(item)
    if len({r['request_id'] for r in results}) != len(results):
        raise ValueError('duplicate independent request identity')
    paired = [r for r in results if r['raw_state_pair']]
    summary = {'identity': 'REAL_STATE_TIME_ERROR_DIAGNOSTIC', 'source_run': source.name,
               'requested': len(results), 'raw_state_pairs': len(paired),
               'reference_time_qualified': sum(r['reference_time_qualified'] for r in results),
               'metre_qualified': 0, 'independent_runs': 1, 'formal_calibration_runs': 0,
               'formal_validation_runs': 0, 'posterior_used_as_predictor_prior': False,
               'position_error_quantiles': {axis: np.quantile([r['error_'+axis+'_m'] for r in paired], [.5, .95, 1.]).tolist()
                                            for axis in ('h', 'v')} if paired else {},
               'fused_fault_qualified': False, 'calibration_pass': False,
               'rows': results}
    out = run/'export/analysis'/args.label
    out.mkdir(parents=True, exist_ok=False)
    with (out/'actual_error.json').open('x') as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(14, 4))
    if paired:
        t = np.array([r['pose_stamp_s'] for r in paired]);t -= t.min()
        for axis, label in zip(axes[:2], ('h', 'v')):
            axis.plot(t, [r['error_'+label+'_m'] for r in paired], 'o-', label='actual state-time error')
            axis.plot(t, [r[label+'pl_m'] for r in paired], 'x--', label='different-reference conditional model')
            axis.set_ylabel(label.upper()+' (m)');axis.set_xlabel('original state time offset (s)');axis.legend()
        axes[2].plot(t, [r['reference_pose_delta_s'] for r in paired], 'o', label='reference - state')
        axes[2].plot(t, [r['state_epoch_delta_s'] for r in paired], 'x', label='state - GNSS epoch')
        axes[2].axhline(.05, color='red', ls=':');axes[2].set_ylabel('Original timestamp gap (s)');axes[2].legend()
    fig.suptitle('Real state-time pairs; different-reference curves are diagnostics, no coverage qualification')
    fig.tight_layout();fig.savefig(out/'actual_errors_and_time.png', dpi=150);plt.close(fig)
    write_subordinate_manifest(run, 'fusion_error_'+args.label, {
        'source_run': str(source), 'source_revision': primary['source']['git_commit'],
        'current_source': source_identity(), 'binary': binary_identity(args.binary.resolve()),
        'commands': commands, 'inputs_sha256': inputs,
        'artifacts_sha256': {str(p.relative_to(run)): sha(p) for p in [*out.iterdir(),
            *(run/'runtime/ros').glob('error_replay_'+args.label+'*.log'),
            *(run/'export/advisory/validation').glob(args.label+'*/*')] if p.is_file()},
        'qualification': {k: v for k, v in summary.items() if k != 'rows'}})
    print(json.dumps({k: v for k, v in summary.items() if k != 'rows'}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
