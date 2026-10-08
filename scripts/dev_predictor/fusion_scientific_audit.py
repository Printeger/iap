#!/usr/bin/env python3
"""Replay a native planning capture without granting error or motion qualification."""
import argparse
import csv
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts/dev_planner'))
from replay_on_search import verify_planning_capture
from advisory_validation import binary_identity, source_identity, sha, safe_label
from run_directory import (resolve_run_directory, adopt_run_directory,
                           write_subordinate_manifest, finalize_run)


def rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def read_matrix(entry, name):
    value = entry[name]
    return None if value is None else np.array(value['row_major']).reshape(3, 3)


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def lidar_reference(primitives, position, meta):
    """Direct measurement rows; no production normal matrix or Schur code."""
    groups = {}
    radius = meta['lidar_radius_m']
    for primitive in primitives:
        c = np.array([float(primitive[k]) for k in ('x', 'y', 'z')])
        n = np.array([float(primitive[k]) for k in ('nx', 'ny', 'nz')])
        if not np.isfinite(np.r_[c, n]).all() or np.linalg.norm(n) <= 1e-9:
            continue
        n /= np.linalg.norm(n)
        distance2 = np.sum((c-position)**2)
        if distance2 > radius**2:
            continue
        confidence = np.clip(float(primitive['confidence']), 0., 1.)
        if confidence <= 0:
            continue
        key = (*np.floor(c / meta['lidar_support_voxel_m'] + 1e-9).astype(int),
               int(np.argmax(np.abs(n))))
        weight = (meta['lidar_weight_scale'] * np.exp(-distance2/(2*radius**2)) *
                  confidence * float(primitive['weight']) / meta['lidar_sigma']**2)
        j = np.r_[n, np.cross(c-position, n)]
        groups.setdefault(key, []).append(np.sqrt(weight)*j)
    # Family averaging describes a conservative support proxy. It is not a
    # declaration that neighboring map patches are independent real samples.
    measurement = np.array([r/np.sqrt(len(group)) for group in groups.values() for r in group])
    return measurement.reshape(-1, 6), len(groups)


def gnss_reference(observations, antenna):
    systems = sorted({row['constellation'] for row in observations})
    measurement = np.zeros((len(observations), 6 + len(systems)))
    for i, row in enumerate(observations):
        u = np.asarray(row['los_map'])
        measurement[i, :3] = u
        measurement[i, 3:6] = u @ -skew(antenna)
        measurement[i, 6 + systems.index(row['constellation'])] = 1.
        measurement[i] /= row['final_sigma_m']
    return measurement


def position_reference(measurement):
    """QR/SVD residual projection of nuisance columns, independent of Schur."""
    p, nuisance = measurement[:, :3], measurement[:, 3:]
    if nuisance.size:
        u, s, _ = np.linalg.svd(nuisance, full_matrices=False)
        keep = s > (s[0]*1e-11 if len(s) else 0)
        p = p - u[:, keep] @ (u[:, keep].T @ p)
    return p.T @ p


def inspect(dataset, require_pose=False):
    checks = []
    sampling = dataset/'sampling_actual/matrices.jsonl'
    if sampling.is_file():
        values = [json.loads(line) for line in sampling.read_text().splitlines()]
        first = read_matrix(values[0], 'position_marginal')
        for value in values[1:]:
            error = float(np.max(np.abs(read_matrix(value, 'position_marginal')-first)))
            checks.append({'variant': 'actual_correlated_copies', 'copies': value['copies'],
                           'max_abs_error': error, 'pass': error <= 1e-12})
    for directory in sorted(p for p in dataset.iterdir() if p.is_dir()):
        if not (directory/'points.csv').is_file():
            continue
        meta = json.loads((directory/'input.json').read_text())
        table = rows(directory/'points.csv')
        matrices = [json.loads(line) for line in (directory/'matrices.jsonl').read_text().splitlines()]
        gnss = [json.loads(line) for line in (directory/'gnss_information_rows.jsonl').read_text().splitlines()]
        primitives = rows(directory/'lidar_primitives.csv') if (directory/'lidar_primitives.csv').is_file() else []
        for point, mat, obs in zip(table, matrices, gnss, strict=True):
            if point['module_called'] != '1' or directory.name == 'S2_weak_normal_support':
                continue
            position = np.array([float(point[k]) for k in ('x', 'y', 'z')])
            jl, count = lidar_reference(primitives, position, meta)
            rotation = np.array(meta['coordinates']['T_map_world']).reshape(4, 4)[:3, :3]
            body = np.array(meta['coordinates']['T_world_imu']).reshape(4, 4)[:3, :3]
            antenna = rotation @ body @ np.array(meta['coordinates']['lever_arm_imu'])
            jg = gnss_reference(obs['rows'], antenna)
            gl = read_matrix(mat, 'gnss')
            ll = read_matrix(mat, 'lidar')
            full = read_matrix(mat, 'fused_information')
            if full is None:
                continue
            # Old conditional 3D model: eliminate clocks, condition attitude.
            independent_gnss = position_reference(np.c_[jg[:, :3], jg[:, 6:]])
            conditional_lidar = jl[:, :3].T @ jl[:, :3]
            used_gnss = point['gnss_used'] == '1'
            used_lidar = point['lidar_used'] == '1'
            prior = read_matrix(mat, 'prior')
            old = prior + (independent_gnss if used_gnss else 0) + (conditional_lidar if used_lidar else 0)
            joint = np.vstack((jg if used_gnss else np.zeros((0, jg.shape[1])),
                               np.pad(jl if used_lidar else np.zeros((0, 6)), ((0, 0), (0, jg.shape[1]-6)))))
            corrected = position_reference(joint) / mat.get('cross_source_noise_inflation', 1.)
            if prior is not None:
                corrected += prior
            expected = corrected if require_pose else old
            error = float(np.max(np.abs(full-expected)))
            check = {'variant': directory.name, 'id': int(point['id']),
                     'support_groups': count, 'max_abs_error': error,
                     'conditional_eigenvalues': np.linalg.eigvalsh(old).tolist(),
                     'pose_marginal_eigenvalues': np.linalg.eigvalsh(corrected).tolist(),
                     'pass': bool(np.allclose(full, expected, rtol=2e-8, atol=1e-8))}
            if require_pose and 'joint_pose_information' not in mat:
                check['pass'] = False
                check['reason'] = 'POSITION_CONDITIONAL_INTERFACE_USED_AS_MARGINAL'
            checks.append(check)
    return {'scope': 'INDEPENDENT_NUMERICAL_REFERENCE', 'checks': checks,
            'pass': bool(checks) and all(c['pass'] for c in checks),
            'error_qualification': 'NOT_GRANTED', 'fused_integrity_qualification': 'NOT_GRANTED'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--require-pose', action='store_true')
    args = parser.parse_args()
    safe_label(args.label)
    inherited = os.environ.get('IAP_RUN_DIR')
    run = adopt_run_directory(inherited) if inherited else resolve_run_directory(
        entrypoint='fusion_scientific_audit', scenario='icra_dense_forest_four_fork_v2')
    snapshot = args.snapshot.resolve()
    payload = snapshot/'planning_input.bin'
    meta = json.loads((snapshot/'snapshot.json').read_text())
    input_identity = verify_planning_capture(snapshot, payload, meta)
    command = [str(args.binary.resolve()), 'replay_audit', args.label, str(payload), '120']
    env = {**os.environ, 'IAP_RUN_DIR': str(run), 'ROS_LOG_DIR': str(run/'runtime/ros')}
    log = run/'runtime/ros'/('fusion_audit_'+args.label+'.log')
    with log.open('x') as stream:
        result = subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT)
    summary = inspect(run/'export/advisory/validation'/args.label, args.require_pose) if result.returncode == 0 else {'pass': False}
    target = run/'export/analysis'/('fusion_audit_'+args.label+'.json')
    with target.open('x') as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
    write_subordinate_manifest(run, 'fusion_audit_'+args.label, {
        'identity': 'REAL_INPUT_DIAGNOSTIC', 'command': command, 'exit_code': result.returncode,
        'source': source_identity(), 'binary': binary_identity(args.binary.resolve()),
        'captured_identity': input_identity, 'payload_sha256': sha(payload),
        'snapshot_sha256': sha(snapshot/'snapshot.json'), 'require_pose': args.require_pose,
        'artifacts_sha256': {str(p.relative_to(run)): sha(p) for p in
            [log, target, *(run/'export/advisory/validation'/args.label).rglob('*')] if p.is_file()}})
    if not inherited:
        finalize_run(run, lifecycle='completed' if summary['pass'] else 'failed', safety_outcome='not_applicable')
    print(json.dumps({'run': str(run), 'pass': summary['pass'], 'checks': len(summary.get('checks', []))}))
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
