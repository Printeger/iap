"""Separate fitted-plane offset and sensor coverage on viewed fixed-map data.

This is a diagnostic, not a corrected estimator, noise calibration or validation.
Association identities, weights, GNSS and truth remain the original records.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

from heldout_fixed_map import grouped, position_metric, primitives, solve
from advisory_validation import artifact, safe_label, sha

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'launch/_includes'))
from run_directory import adopt_run_directory, write_subordinate_manifest


def fitted_plane_offsets(raw, centers, normals, support_counts, radius=1.5):
    offsets, spread = [], []
    for c, n, count in zip(centers, normals, support_counts):
        patch = raw[np.sum((raw-c)**2, axis=1) <= radius**2]
        assert len(patch) == count, 'ORIGINAL_PCA_NEIGHBORHOOD_MISMATCH'
        mean = np.mean(patch, axis=0)
        cov = (patch-mean).T @ (patch-mean) / len(patch)
        values, axes = np.linalg.eigh(cov)
        assert abs(n @ axes[:, 0]) > 1-1e-8, 'ORIGINAL_PCA_NORMAL_MISMATCH'
        offsets.append(float((mean-c) @ n))
        spread.append(float(values[0]))
    return np.array(offsets), np.array(spread)


def sensor_support(centers, T_map_lidar, sensor):
    local = (centers-T_map_lidar[:3, 3]) @ T_map_lidar[:3, :3]
    ranges = np.linalg.norm(local, axis=1)
    elevation = np.arctan2(local[:, 2], np.linalg.norm(local[:, :2], axis=1))
    azimuth = np.arctan2(local[:, 1], local[:, 0])
    return ((ranges >= sensor['min_range_m']) & (ranges <= sensor['max_range_m']) &
            (elevation >= sensor['vertical_min_rad']) &
            (elevation <= sensor['vertical_max_rad']) &
            (abs(azimuth) <= sensor['horizontal_fov_rad']/2))


def first_hit_support(origin, targets, occupied, lattice_origin, resolution):
    """Exact lattice crossing, excluding sensor and target cells.

    Only known raw occupied voxels occlude. An absent cell is not a proof of
    free space; the result is conditional geometric support, not authorization.
    """
    start = (origin-lattice_origin)/resolution
    result = []
    for target in targets:
        end = (target-lattice_origin)/resolution
        cell, last = np.floor(start).astype('int64'), np.floor(end).astype('int64')
        delta = end-start; step = np.sign(delta).astype('int64')
        boundary = cell+(step > 0)
        maximum = np.divide(boundary-start, delta, out=np.full(3, np.inf), where=delta != 0)
        increment = np.divide(1., abs(delta), out=np.full(3, np.inf), where=delta != 0)
        visible = True
        for _ in range(int(np.sum(abs(last-cell)))+3):
            if np.array_equal(cell, last): break
            axis = int(np.argmin(maximum)); cell[axis] += step[axis]; maximum[axis] += increment[axis]
            if not np.array_equal(cell, last) and tuple(cell) in occupied:
                visible = False; break
        else:
            raise AssertionError('LATTICE_TRAVERSAL_DID_NOT_CLOSE')
        result.append(visible)
    return np.array(result)


def evaluate(config_path, label):
    cfg = json.loads(config_path.read_text())
    run = adopt_run_directory(cfg['owner'])
    out = artifact(run, f'export/analysis/{safe_label(label)}/summary.json').parent
    assert not (out/'summary.json').exists(), 'EVIDENCE_ALREADY_EXISTS'
    inputs = {}
    def read(path):
        path = Path(path); inputs[str(path)] = sha(path)
        return json.loads(path.read_text())
    inputs[str(config_path)] = sha(config_path)
    inputs[str(Path(__file__))] = sha(Path(__file__))
    map_path = Path(cfg['map_metadata']); meta = read(map_path)
    primitive_path = map_path.with_name('lidar_primitives.csv')
    inputs[str(primitive_path)] = sha(primitive_path)
    c, n, weight = primitives(primitive_path)
    counts = [int(v['support_count']) for v in csv.DictReader(primitive_path.open())]
    raw_path = Path(cfg['raw_centers']); inputs[str(raw_path)] = sha(raw_path)
    raw = np.array([[float(v[k]) for k in ('x', 'y', 'z')] for v in csv.DictReader(raw_path.open())])
    lattice_origin = np.array(meta['origin']); resolution = meta['resolution_m']
    occupied = set(map(tuple, np.floor((raw-lattice_origin)/resolution+1e-8).astype('int64')))
    offsets, variance = fitted_plane_offsets(raw, c, n, counts)
    sensor = read(cfg['sensor_metadata'])['map_source_evidence']['current_observation_frame']
    rows = []
    for original in read(cfg['same_map_summary'])['rows']:
        if not original['qualified']:
            rows.append({'frame_id': original['frame_id'], 'qualified': False, 'reason': original['reason']})
            continue
        seed, frame = original['seed'], original['frame_id']
        original_cfg = read(Path(cfg['diagnostic_owner']) / f'metadata/config/dense_{seed}_increment_v1.json')
        candidates = list(Path(original_cfg['decoded_root']).glob(original_cfg['decoded_glob']))
        mp = next(p for p in candidates if json.loads(p.read_text())['estimation_frame_id'] == frame)
        current = read(mp)
        a = read(Path(cfg['diagnostic_owner']) / f'export/analysis/dense_{seed}_increment_v1/frame_{frame}.json')
        corr = read(Path(cfg['same_map_summary']).with_name(f'seed_{seed}_frame_{frame}_correspondence.json'))
        p = np.array(current['position']); radius = meta['lidar_radius_m']
        keys = np.column_stack((np.floor(c/meta['lidar_support_voxel_m']+1e-9).astype('int64'), np.argmax(abs(n), axis=1)))
        w = weight * meta['lidar_weight_scale'] * np.exp(-np.sum((c-p)**2, axis=1)/(2*radius**2)) / meta['lidar_sigma']**2
        T = np.array(current['coordinates']['T_map_world']).reshape(4, 4) @ np.array(current['coordinates']['T_world_imu']).reshape(4, 4)
        sensor_T = T @ np.linalg.inv(np.array(current['coordinates']['T_lidar_imu']).reshape(4, 4))
        support = sensor_support(c, sensor_T, sensor) & (np.sum((c-p)**2, axis=1) <= radius**2)
        ma, _ = grouped(np.column_stack((n[support], np.cross(c[support]-p, n[support]))), np.zeros(np.sum(support)), w[support], keys[support])
        ga, gb = np.array(a['admitted_gnss_design']), np.array(a['admitted_gnss_residual_whitened'])
        model = position_metric(np.vstack((np.pad(ma, ((0, 0), (0, ga.shape[1]-6))), ga)), meta['K_H_adv'])
        visible = support.copy()
        visible[support] = first_hit_support(sensor_T[:3, 3], c[support], occupied, lattice_origin, resolution)
        va, _ = grouped(np.column_stack((n[visible], np.cross(c[visible]-p, n[visible]))), np.zeros(np.sum(visible)), w[visible], keys[visible])
        first_hit_model = position_metric(np.vstack((np.pad(va, ((0, 0), (0, ga.shape[1]-6))), ga)), meta['K_H_adv'])
        points = np.array(a['reference_body_points']) @ T[:3, :3].T + T[:3, 3]
        used = np.array([v['used'] for v in corr['rows']]); j = np.array([v['primitive_index'] for v in corr['rows']])[used]
        points = points[used]
        A = np.column_stack((n[j], np.cross(points-p, n[j])))
        residual = np.einsum('ni,ni->n', n[j], c[j]-points)
        la, oldb = grouped(A, residual, w[j], keys[j])
        _, correctedb = grouped(A, residual+offsets[j], w[j], keys[j])
        both = np.vstack((np.pad(la, ((0, 0), (0, ga.shape[1]-6))), ga))
        expected = np.array(next(v for v in read(Path(cfg['diagnostic_owner']) / f'export/analysis/dense_{seed}_increment_v1/summary.json')['rows'] if v['frame_id'] == frame)['true_local_increment'])
        old = solve(both, np.r_[oldb, gb], expected, np.array(original['model']['weak_axis']))
        corrected = solve(both, np.r_[correctedb, gb], expected, np.array(original['model']['weak_axis']))
        assert abs(old['horizontal_error_m']-original['actual']['horizontal_error_m']) < 1e-9, 'ORIGINAL_SOLVE_MISMATCH'
        rows.append({'seed': seed, 'frame_id': frame, 'position': p.tolist(), 'qualified': True,
                     'original_model': original['model'], 'actual_geometry': original['actual_geometry'],
                     'fov_only_model': model, 'original_solve': old, 'plane_offset_only_solve': corrected,
                     'first_hit_model': first_hit_model, 'first_hit_eligible_primitives': int(np.sum(visible)),
                     'fov_eligible_primitives': int(np.sum(support)),
                     'matched_primitives_outside_sensor_support': int(np.sum(~support[j])),
                     'matched_plane_offset_rms_m': float(np.sqrt(np.mean(offsets[j]**2))),
                     'matched_plane_normal_spread_m': float(np.sqrt(np.mean(variance[j])))})
    result = {'rows': rows, 'claims': {'diagnostic_only': True, 'production_changed': False,
              'noise_model_qualified': False, 'independent_validation': False},
              'limits': ['FOV is the recorded unchanged simulator sensor model; first-hit probe uses raw lattice occlusion, without beam sampling or map uncertainty',
                         'centroid offset probe retains original seed association and weights, and changes only plane intercept',
                         'PCA normal spread is geometric dispersion, not a calibrated covariance of registered map error']}
    with (out/'summary.json').open('x') as stream: json.dump(result, stream, indent=2)
    write_subordinate_manifest(run, label, {'inputs_sha256': inputs, 'artifacts_sha256': {str((out/'summary.json').relative_to(run)): sha(out/'summary.json')}})
    print(json.dumps({'output': str(out), 'rows': len(rows), 'qualified': sum(v['qualified'] for v in rows)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path); parser.add_argument('label')
    args = parser.parse_args(); evaluate(args.config, args.label)
