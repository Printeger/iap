"""One-second relative-motion evaluation; truth never supplies a prediction.

Contract: docs/spec/local_increment_advisory_contract.md. The existing geometric
H score is only a diagnostic baseline, not a relative-error covariance or PL.
"""
import argparse
import csv
import itertools
import json
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'launch/_includes'))
from run_directory import adopt_run_directory, write_subordinate_manifest
from advisory_validation import sha, safe_label

WINDOW_S = 1.0
TRUTH_ENDPOINT_GATE_S = .05
ESTIMATE_BRACKET_GATE_S = .15
REGIONS = ((-18., -14.), (-14., -10.), (-10., -6.))
GATES = {'regional_concordance_min': .7, 'spearman_min': .5,
         'samples_per_run_min': 6, 'regions_per_run_min': 2,
         'independent_runs_required': 3}


def rigid(T):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all() or
            not np.allclose(T[3], [0, 0, 0, 1], atol=1e-10, rtol=0) or
            not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-8, rtol=0) or
            not np.isclose(np.linalg.det(T[:3, :3]), 1., atol=1e-8, rtol=0)):
        raise ValueError('POSE_NOT_RIGID')
    return T


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('INVALID_QUATERNION')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(a, b, fraction):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1., 1.))
    if dot > .9995:
        q = (1-fraction)*a + fraction*b
        return q / np.linalg.norm(q)
    angle = np.arccos(dot)
    return (np.sin((1-fraction)*angle)*a + np.sin(fraction*angle)*b)/np.sin(angle)


def relative_error(estimate_start, estimate_end, truth_start, truth_end):
    a, b, ta, tb = map(rigid, (estimate_start, estimate_end, truth_start, truth_end))
    estimated = np.linalg.solve(a, b)
    actual = np.linalg.solve(ta, tb)
    error = estimated[:3, 3] - actual[:3, 3]
    rotation = estimated[:3, :3] @ actual[:3, :3].T
    return {'translation_error_start_body_m': error.tolist(),
            'horizontal_error_m': float(np.linalg.norm(error[:2])),
            'translation_error_m': float(np.linalg.norm(error)),
            'rotation_error_rad': float(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))),
            'estimated_increment': estimated.tolist(), 'truth_increment': actual.tolist(),
            'truth_motion_distance_m': float(np.linalg.norm(actual[:3, 3])),
            'estimated_motion_distance_m': float(np.linalg.norm(estimated[:3, 3]))}


def corrected_pose(nominal, correction):
    """Original pose-row convention: translation, then map-left rotation."""
    T = rigid(nominal).copy()
    correction = np.asarray(correction, dtype=float)
    if correction.shape != (6,) or not np.isfinite(correction).all():
        raise ValueError('POSE_RESPONSE_UNAVAILABLE')
    v, w = correction[:3], correction[3:]
    angle = np.linalg.norm(w)
    K = np.array([[0., -w[2], w[1]], [w[2], 0., -w[0]], [-w[1], w[0], 0.]])
    R = np.eye(3)+K+.5*K@K if angle < 1e-8 else (
        np.eye(3)+np.sin(angle)/angle*K+(1-np.cos(angle))/angle**2*K@K)
    T[:3, 3] += v
    T[:3, :3] = R @ T[:3, :3]
    return rigid(T)


def paired_response_moments(nominal_start, nominal_end, joint_responses, K_H=5.):
    """Transport complete paired draws; never assemble independent marginals.

    Inputs describe a declared conditional observation experiment. This algebra
    does not supply its physical law or qualify its mean/covariance as real noise.
    Nominal poses are predictor query geometry, not evaluation truth.
    """
    joint = np.asarray(joint_responses, dtype=float)
    if joint.ndim != 3 or joint.shape[1:] != (2, 6) or len(joint) < 2 or not np.isfinite(joint).all():
        raise ValueError('JOINT_ENDPOINT_RESPONSE_UNAVAILABLE')
    errors = np.array([relative_error(corrected_pose(nominal_start, pair[0]),
                                     corrected_pose(nominal_end, pair[1]),
                                     nominal_start, nominal_end)['translation_error_start_body_m']
                       for pair in joint])
    mean = errors.mean(axis=0)
    covariance = np.cov(errors, rowvar=False, ddof=1)
    mse = covariance+np.outer(mean, mean)
    return {'mean_increment_response_m': mean.tolist(),
            'conditional_sampling_covariance_m2': covariance.tolist(),
            'score_m': float(K_H*np.sqrt(max(0., np.linalg.eigvalsh(mse[:2, :2])[-1]))),
            'joint_draw_count': len(joint), 'physical_error_moments_qualified': False}


class PoseSeries:
    def __init__(self, rows, source):
        expected = {'glio': ('map', 'imu'), 'truth': ('map', 'drone_0')}
        if source not in expected:
            raise ValueError('UNKNOWN_POSE_SOURCE')
        values = []
        self.ambiguous_stamps = set()
        for row in rows:
            if row['source'] != source:
                continue
            if (row['frame'], row['body']) != expected[source]:
                raise ValueError('POSE_FRAME_OR_EXTRINSIC_UNQUALIFIED')
            stamp = float(row['stamp'])
            p = np.array([float(row[k]) for k in ('x', 'y', 'z')])
            q = np.array([float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')])
            R = quaternion_rotation(q)
            if not np.isfinite(stamp) or not np.isfinite(p).all():
                raise ValueError('NONFINITE_POSE')
            if values and stamp < values[-1][0]:
                raise ValueError('NONMONOTONIC_POSE_TIME')
            if values and stamp == values[-1][0]:
                if not np.allclose(p, values[-1][1], rtol=0, atol=1e-12) or not np.allclose(R, quaternion_rotation(values[-1][2]), rtol=0, atol=1e-12):
                    self.ambiguous_stamps.add(stamp)
                continue
            values.append((stamp, p, q))
        if len(values) < 2:
            raise ValueError('POSE_SERIES_UNAVAILABLE')
        self.stamps = np.array([v[0] for v in values])
        self.positions = np.array([v[1] for v in values])
        self.quaternions = np.array([v[2] for v in values])
        self.source = source

    def at(self, stamp):
        i = int(np.searchsorted(self.stamps, stamp))
        if i < len(self.stamps) and self.stamps[i] == stamp:
            if stamp in self.ambiguous_stamps:
                raise ValueError('CONFLICTING_DUPLICATE_POSE_STAMP')
            p, q = self.positions[i], self.quaternions[i]
            evidence = {'stamp_s': stamp, 'exact_sample': True, 'bracket_s': [stamp, stamp]}
        else:
            if i == 0 or i == len(self.stamps):
                raise ValueError('POSE_WINDOW_NOT_BRACKETED')
            lo, hi = self.stamps[i-1:i+1]
            if lo in self.ambiguous_stamps or hi in self.ambiguous_stamps:
                raise ValueError('CONFLICTING_DUPLICATE_POSE_STAMP')
            if self.source == 'truth':
                if max(stamp-lo, hi-stamp) > TRUTH_ENDPOINT_GATE_S+1e-6:
                    raise ValueError('TRUTH_BRACKET_EXCEEDS_50MS')
            elif hi-lo > ESTIMATE_BRACKET_GATE_S+1e-6:
                raise ValueError('ESTIMATE_BRACKET_EXCEEDS_150MS')
            fraction = (stamp-lo)/(hi-lo)
            p = (1-fraction)*self.positions[i-1] + fraction*self.positions[i]
            q = slerp(self.quaternions[i-1], self.quaternions[i], fraction)
            evidence = {'stamp_s': stamp, 'exact_sample': False, 'bracket_s': [float(lo), float(hi)]}
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = quaternion_rotation(q), p
        return rigid(T), evidence


def ranks(values):
    values = np.asarray(values)
    return np.array([np.sum(values < v)+.5*(np.sum(values == v)-1) for v in values])


def regional_results(rows):
    results = []
    for seed in sorted({r['seed'] for r in rows}):
        valid = [r for r in rows if r['seed'] == seed and r.get('qualified')]
        regions = []
        for index, (lo, hi) in enumerate(REGIONS):
            selected = [r for r in valid if lo <= r['position'][0] and
                        (r['position'][0] < hi or (index == len(REGIONS)-1 and r['position'][0] == hi))]
            region = {'bin': [lo, hi], 'count': len(selected), 'frames': [r['frame_id'] for r in selected]}
            if selected:
                region.update(baseline_H_m=float(np.median([r['baseline_H_m'] for r in selected])),
                              error_m=float(np.median([r['relative']['horizontal_error_m'] for r in selected])))
            regions.append(region)
        represented = [r for r in regions if r['count']]
        pairs = [{'a': a['bin'], 'b': b['bin'], 'concordant': bool(
                    (a['baseline_H_m']-b['baseline_H_m'])*(a['error_m']-b['error_m']) > 0)}
                 for a, b in itertools.combinations(represented, 2)]
        concordance = float(np.mean([p['concordant'] for p in pairs])) if pairs else None
        rho = None
        if len(valid) >= 3:
            x = ranks([r['baseline_H_m'] for r in valid])
            y = ranks([r['relative']['horizontal_error_m'] for r in valid])
            if np.ptp(x) > 0 and np.ptp(y) > 0:
                rho = float(np.corrcoef(x, y)[0, 1])
        results.append({'seed': seed, 'requested': sum(r['seed'] == seed for r in rows),
                        'qualified_samples': len(valid), 'regions': regions, 'pairs': pairs,
                        'concordance': concordance, 'spearman': rho,
                        'passes_development_ordering_gates': bool(
                            len(valid) >= GATES['samples_per_run_min'] and
                            len(represented) >= GATES['regions_per_run_min'] and
                            concordance is not None and concordance >= GATES['regional_concordance_min'] and
                            rho is not None and rho >= GATES['spearman_min'])})
    return results


def evaluate(run, label, control_path):
    safe_label(label)
    output = run / 'export/analysis' / label
    output.mkdir(exist_ok=False)
    inputs = {}
    def read(path):
        path = Path(path)
        inputs[str(path)] = sha(path)
        return json.loads(path.read_text())
    control = read(control_path)
    inputs[str(Path(__file__))] = sha(Path(__file__))
    contract = REPO / 'docs/spec/local_increment_advisory_contract.md'
    inputs[str(contract)] = sha(contract)
    rows = []
    source_ambiguities = []
    for seed in (5101, 5102, 5103):
        config = read(REPO / f'log/20261011T044535Z_421/metadata/config/dense_{seed}_increment_v1.json')
        poses_path = Path(config['truth'])
        inputs[str(poses_path)] = sha(poses_path)
        with poses_path.open() as stream:
            original_poses = list(csv.DictReader(stream))
        estimated, truth = PoseSeries(original_poses, 'glio'), PoseSeries(original_poses, 'truth')
        source_ambiguities.append({'seed': seed,
                                  'estimate_stamps_s': sorted(estimated.ambiguous_stamps),
                                  'truth_stamps_s': sorted(truth.ambiguous_stamps)})
        source_run = Path(config['gpu']).parents[2]
        source_manifest = read(source_run / 'metadata/run_manifest.json')
        if not source_manifest['source']['git_worktree_clean']:
            raise ValueError('ORIGINAL_SOURCE_NOT_CLEAN')
        inputs[str(poses_path.with_name(f'dense_off{seed}_requests.csv'))] = sha(poses_path.with_name(f'dense_off{seed}_requests.csv'))
        metadata = {read(p)['estimation_frame_id']: p for p in
                    Path(config['decoded_root']).glob(config['decoded_glob'])}
        for missing in config['missing_requests']:
            rows.append(dict(seed=seed, frame_id=None, qualified=False,
                             reason=missing['reason'], original_missing_request=missing))
        selected = [r for r in control['rows'] if r['seed'] == seed]
        if [r['frame_id'] for r in selected] != config['qualified_frames']:
            raise ValueError('PREDECLARED_QUERY_SET_CHANGED')
        for original in selected:
            row = {'seed': seed, 'frame_id': original['frame_id'], 'qualified': False,
                   'position': original['position'], 'baseline_H_m': original['baseline_H_m'],
                   'old_absolute_primary_m': original['actual']['horizontal_error_m']}
            rows.append(row)
            try:
                if not original['qualified']:
                    raise ValueError('ORIGINAL_QUERY_UNQUALIFIED')
                meta = read(metadata[original['frame_id']])
                if meta['current_motion_quality'] != 1:
                    raise ValueError('ORIGINAL_MOTION_QUALITY_UNQUALIFIED')
                start = meta['reference_time_s']
                end = start+WINDOW_S
                row.update(start_s=start, end_s=end, window_s=WINDOW_S,
                           frozen_prediction_identity=meta['prediction_input_identity'])
                es, eb = estimated.at(start)
                ee, eeb = estimated.at(end)
                ts, tb = truth.at(start)
                te, teb = truth.at(end)
                row.update(qualified=True, relative=relative_error(es, ee, ts, te),
                           brackets={'estimate_start': eb, 'estimate_end': eeb,
                                     'truth_start': tb, 'truth_end': teb})
            except (KeyError, ValueError, np.linalg.LinAlgError) as error:
                row['reason'] = str(error)
    summary = {'schema': 'local_increment_baseline_discriminator_v1',
               'contract': str(contract.relative_to(REPO)), 'window_s': WINDOW_S,
               'prediction': 'unchanged V5 conditional geometric H baseline; not relative covariance or PL',
               'primary': 'one-second actual GLIO relative translation error, initial physical IMU XY, independent truth',
               'rows': rows, 'regional': regional_results(rows), 'gates': GATES,
               'source_ambiguities': source_ambiguities,
               'all_predeclared_regions_preserved': True, 'truth_in_prediction': False,
               'alignment_fitted': False, 'sample_means_subtracted': False,
               'production_changed': False, 'independent_validation': False,
               'qualified_relative_model': False, 'development_complete': False,
               'goal_status': 'active', 'inputs_sha256': inputs}
    path = output / 'summary.json'
    path.write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    write_subordinate_manifest(run, label, {'inputs_sha256': inputs,
        'artifacts_sha256': {str(path.relative_to(run)): sha(path)},
        'qualification': 'one viewed-data evaluation-object discriminator; no physical moments, M3 or route benefit'})
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--label', default='local_increment_baseline_v1')
    parser.add_argument('--control', type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(adopt_run_directory(args.run_dir), args.label, args.control)
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'inputs_sha256')}, indent=2))


if __name__ == '__main__':
    main()
