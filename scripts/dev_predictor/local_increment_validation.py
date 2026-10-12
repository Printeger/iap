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


def transport_retained_error_law(joint_mean, joint_covariance, retained_dim,
                                state_transition, innovation_response,
                                mean_offset, state_dimension_budget=55):
    """Bounded affine response reference, given a complete physical joint law.

    Input ordering is [retained sources/start endpoint, current state, new
    innovations]. Output keeps only [retained, next state]. The innovation
    cross blocks are REQUIRED, including when a measurement is reused. No
    posterior, loss model or independence assumption supplies missing blocks.
    Caller must qualify the physical law and native response separately; this
    algebra grants neither qualification nor a prediction from recorded means.
    """
    if joint_mean is None or joint_covariance is None:
        raise ValueError('PHYSICAL_JOINT_ERROR_LAW_UNAVAILABLE')
    F, G = np.asarray(state_transition, float), np.asarray(innovation_response, float)
    if (F.ndim != 2 or F.shape[0] != F.shape[1] or G.ndim != 2 or
            G.shape[0] != F.shape[0] or not isinstance(retained_dim, int) or retained_dim < 0):
        raise ValueError('ERROR_RESPONSE_LAYOUT_INVALID')
    state_dim, noise_dim = F.shape[0], G.shape[1]
    if retained_dim+state_dim > state_dimension_budget:
        raise ValueError('BOUNDED_ERROR_STATE_BUDGET_EXCEEDED')
    dim = retained_dim+state_dim+noise_dim
    mean, covariance = np.asarray(joint_mean, float), np.asarray(joint_covariance, float)
    offset = np.asarray(mean_offset, float)
    if (mean.shape != (dim,) or covariance.shape != (dim, dim) or
            offset.shape != (state_dim,) or not all(np.isfinite(v).all() for v in (mean, covariance, F, G, offset))):
        raise ValueError('JOINT_ERROR_BLOCKS_MISSING_OR_NONFINITE')
    if (not np.allclose(covariance, covariance.T, rtol=0, atol=1e-12) or
            np.linalg.eigvalsh(covariance).min() < -1e-10):
        raise ValueError('PHYSICAL_JOINT_ERROR_LAW_NOT_PSD')
    response = np.zeros((retained_dim+state_dim, dim))
    response[:retained_dim, :retained_dim] = np.eye(retained_dim)
    response[retained_dim:, retained_dim:retained_dim+state_dim] = F
    response[retained_dim:, retained_dim+state_dim:] = G
    output_mean = response @ mean
    output_mean[retained_dim:] += offset
    return output_mean, response @ covariance @ response.T


def condense_physical_score_law(loss_information, eliminated_dim, alias_dim,
                                joint_mean, joint_covariance, score_budget=1536,
                                alias_budget=72):
    """Offline native Schur response reference; never infer noise from H.

    Input is [frozen-source/start aliases, physical score forcing on D, on R].
    H is the verified native loss response in that D,R chart. Physical mean,
    covariance and every alias/forcing cross block must be supplied independently.
    Output keeps aliases and condensed forcing on R, rather than all old states
    or primitive history. No production budget or predictor is changed here.
    """
    if joint_mean is None or joint_covariance is None:
        raise ValueError('PHYSICAL_JOINT_SCORE_LAW_UNAVAILABLE')
    H = np.asarray(loss_information, float)
    if (H.ndim != 2 or H.shape[0] != H.shape[1] or
            not isinstance(eliminated_dim, int) or not 0 < eliminated_dim < len(H) or
            not isinstance(alias_dim, int) or alias_dim < 0):
        raise ValueError('NATIVE_SCHUR_LAYOUT_INVALID')
    n = len(H)
    if n > score_budget or alias_dim > alias_budget:
        raise ValueError('BOUNDED_SCORE_CARRIER_BUDGET_EXCEEDED')
    mean, covariance = np.asarray(joint_mean, float), np.asarray(joint_covariance, float)
    if (mean.shape != (alias_dim+n,) or covariance.shape != (alias_dim+n, alias_dim+n) or
            not all(np.isfinite(v).all() for v in (H, mean, covariance))):
        raise ValueError('PHYSICAL_JOINT_SCORE_BLOCKS_MISSING_OR_NONFINITE')
    if not np.allclose(H, H.T, rtol=1e-12, atol=1e-12):
        raise ValueError('NATIVE_LOSS_INFORMATION_NOT_SYMMETRIC')
    if (not np.allclose(covariance, covariance.T, rtol=1e-12, atol=1e-12) or
            np.linalg.eigvalsh(covariance / max(1., np.max(np.abs(covariance)))).min() < -1e-10):
        raise ValueError('PHYSICAL_JOINT_SCORE_LAW_NOT_PSD')
    D = eliminated_dim
    Hdd = H[:D, :D]
    scale = np.sqrt(np.diag(Hdd))
    if not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError('NATIVE_ELIMINATION_RESPONSE_SINGULAR')
    try:
        gain = np.linalg.solve(Hdd / np.outer(scale, scale), H[:D, D:] / scale[:, None]) / scale[:, None]
    except np.linalg.LinAlgError as error:
        raise ValueError('NATIVE_ELIMINATION_RESPONSE_SINGULAR') from error
    # No covariance inversion, H^-1 noise assumption, or independent marginals.
    response = np.zeros((alias_dim+n-D, alias_dim+n))
    response[:alias_dim, :alias_dim] = np.eye(alias_dim)
    response[alias_dim:, alias_dim:alias_dim+D] = -gain.T
    response[alias_dim:, alias_dim+D:] = np.eye(n-D)
    return (response @ mean, response @ covariance @ response.T,
            H[D:, D:] - H[D:, :D] @ gain, response)


def stationary_simulator_bootstrap_inputs(initial_pose, initial_velocity,
                                         initial_bias, initial_anchor_ecef,
                                         initial_rotation_ecef_map,
                                         first_fix_ecef, body_to_enu_calibration,
                                         enu_to_ecef_calibration,
                                         rest_specific_force_body,
                                         model_gravity_map, physical_recipe):
    """Conditional initial error law from independent physical constraints.

    Offline model input provider, not an estimator or complete forecast. Requires
    the original simulation's held startup, ideal IMU and noise-free initial fix,
    plus known initial body/ENU calibration. Read full initialization values once
    from the existing initialization callback, never later posterior covariance,
    rounded logs, actual matching residuals or a truth pose stream.

    Choose one global datum with physical pose(0) equal to the nominal pose(0).
    This leaves physical initial velocity/bias zero and changes the true gravity
    and ECEF mapping expressed in that datum. Their discrepancy with the original
    solver model is retained. Errors are deterministic conditional on the known
    initial physical state and observed initialization, not zero by default.
    Covariance zero here does not set subsequent map/innovation laws to zero.
    """
    required = {'held_before_first_estimator_feedback': True,
                'input_phase': 'initialization_before_fixed_lag_update',
                'imu_generation': 'ideal_specific_force_and_angular_velocity',
                'initial_fix_generation': 'noise_free_receiver_lla',
                'initial_body_enu_calibration_known': True,
                'calibration_uncertainty': 'none_in_declared_simulator_recipe'}
    if not isinstance(physical_recipe, dict) or any(
            physical_recipe.get(k) != value for k, value in required.items()):
        raise ValueError('STATIONARY_IDEAL_BOOTSTRAP_PHYSICAL_RECIPE_UNAVAILABLE')
    pose = rigid(initial_pose)
    velocity, bias, anchor, fix, acc, gravity = (
        np.asarray(v, dtype=float) for v in
        (initial_velocity, initial_bias, initial_anchor_ecef, first_fix_ecef,
         rest_specific_force_body, model_gravity_map))
    if (any(v.shape != (3,) for v in (velocity, anchor, fix, acc, gravity)) or
            bias.shape != (6,) or
            not all(np.isfinite(v).all() for v in (velocity, bias, anchor, fix, acc, gravity))):
        raise ValueError('BOOTSTRAP_PHYSICAL_INPUT_MISSING_OR_NONFINITE')
    rotations = []
    for value in (initial_rotation_ecef_map, body_to_enu_calibration, enu_to_ecef_calibration):
        value = np.asarray(value, dtype=float)
        if value.shape != (3, 3):
            raise ValueError('BOOTSTRAP_CALIBRATION_ROTATION_UNAVAILABLE')
        transform = np.eye(4); transform[:3, :3] = value
        rotations.append(rigid(transform)[:3, :3])
    nominal_rotation, body_enu, enu_ecef = rotations
    # The calibrated initial physical gravity must explain the independent
    # rest measurement; changing the solver's gravity cannot make this pass.
    if not np.allclose(body_enu @ acc, [0., 0., 9.81], atol=1e-10, rtol=0):
        raise ValueError('BOOTSTRAP_REST_FORCE_INCONSISTENT_WITH_IDEAL_RECIPE')
    physical_rotation = enu_ecef @ body_enu @ pose[:3, :3].T
    physical_anchor = fix - physical_rotation @ pose[:3, 3]
    R = physical_rotation.T @ nominal_rotation
    half_skew = .5*np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    sine = np.linalg.norm(half_skew)
    cosine = np.clip((np.trace(R)-1)/2, -1., 1.)
    angle = np.arctan2(sine, cosine)
    if sine < 1e-10 and cosine < 0:
        raise ValueError('BOOTSTRAP_ALIGNMENT_LOG_AMBIGUOUS')
    rotation_error = half_skew if angle < 1e-8 else half_skew*(angle/sine)
    physical_gravity = -pose[:3, :3] @ acc
    # Native Pose3 error is rotation then translation; vector states are direct
    # differences and the alignment Rot3 uses its original right tangent chart.
    mean = np.concatenate((np.zeros(6), velocity, bias,
                           (anchor-fix)+physical_rotation @ pose[:3, 3], rotation_error,
                           gravity-physical_gravity))
    return {'mean': mean, 'covariance': np.zeros((24, 24)),
            'layout': [('pose0_common_datum', 6), ('velocity0_map', 3),
                       ('imu_bias0_acc_then_gyro', 6), ('anchor_ecef', 3),
                       ('rotation_ecef_map_right', 3), ('gravity_model_minus_physical_map', 3)],
            'physical_rotation_ecef_map': physical_rotation,
            'physical_anchor_ecef': physical_anchor,
            'physical_gravity_map': physical_gravity,
            'complete_retained_or_innovation_law': False,
            'production_predictor_connected': False}


def native_pose_at(evidence, stamp, T_map_world):
    """Original optimized state plus recorded IMU mean transport, no covariance.

    This is an evaluation state representation. It neither supplies a noise law
    nor fits or corrects a production prediction using truth.
    """
    if (evidence.get('model') != 'postopt_X_V_B_R_E_active_clocks_joint_v1' or
            not evidence.get('optimized_valid') or not evidence.get('update_sequence')):
        raise ValueError('NATIVE_STATE_UNAVAILABLE')
    start, end = evidence['state_stamp'], evidence['gnss_stamp']
    if stamp < start-1e-6 or stamp > end+1e-6 or stamp-start > .100001:
        raise ValueError('TARGET_OUTSIDE_NATIVE_IMU_INTERVAL')
    means = np.asarray(evidence['optimized_means'], dtype=float)
    if (evidence['mean_dimensions'][:5] != [16, 3, 6, 9, 3] or
            len(means) != sum(evidence['mean_dimensions']) or len(means) < 37 or not np.isfinite(means).all()):
        raise ValueError('NATIVE_STATE_LAYOUT_UNAVAILABLE')
    frame = evidence['frame_id']
    expected = [(ord(symbol) << 56) | index for symbol, index in
                [('x', frame), ('v', frame), ('b', frame), ('r', 0), ('e', 0)]]
    if evidence['keys'][:5] != expected:
        raise ValueError('NATIVE_STATE_OWNER_MISMATCH')
    if abs(stamp-start) <= 1e-6:
        propagated = means
    else:
        from epoch_motion_reference import intervals, propagate
        rows = intervals(evidence['imu_measurements'], start, stamp)
        propagated = propagate(means, rows, stamp-start)
    return rigid(rigid(T_map_world) @ rigid(propagated[:16].reshape(4, 4)))


def native_endpoint(reports, stamp, T_map_world):
    """Earliest covering GNSS epoch, latest preceding state; never select by error."""
    candidates = {}
    for event in reports:
        report = event['report']
        evidence = report.get('advisory_postopt', {})
        start, end = evidence.get('state_stamp', 0.), evidence.get('gnss_stamp', 0.)
        if not isinstance(start, (int, float)) or not isinstance(end, (int, float)) or not np.isfinite([start, end]).all():
            continue
        if not start-1e-6 <= stamp <= end+1e-6 or stamp-start > .100001:
            continue
        if report.get('estimation_frame_id') != evidence.get('frame_id'):
            continue  # A later frame reusing an old bundle does not own its quality.
        if (not report.get('advisory_coordinates_valid') or
                report['advisory_coordinates_frame_id'] != evidence['frame_id'] or
                report['advisory_coordinates_epoch_source_identity'] != evidence['epoch_source_identity'] or
                # IntegrityReport uses Eigen::data() (column major), whereas
                # the recording codec's describe() exports row-major matrices.
                not np.allclose(np.asarray(report['advisory_t_map_world']).reshape(4, 4, order='F'), T_map_world, rtol=0, atol=1e-9)):
            raise ValueError('NATIVE_COORDINATE_OWNER_MISMATCH')
        key = (evidence['update_sequence'], evidence['frame_id'], start, end, evidence['epoch_source_identity'])
        serialized = json.dumps(evidence, sort_keys=True, allow_nan=False)
        if key in candidates and candidates[key][0] != serialized:
            raise ValueError('CONFLICTING_NATIVE_STATE_IDENTITY')
        if key not in candidates:
            candidates[key] = (serialized, report, evidence)
        elif candidates[key][1].get('current_motion_quality') != report.get('current_motion_quality'):
            raise ValueError('CONFLICTING_NATIVE_STATE_QUALIFICATION')
    if not candidates:
        raise ValueError('NATIVE_ENDPOINT_NOT_RECORDED')
    _, report, evidence = min(candidates.values(), key=lambda item: (item[2]['gnss_stamp'], -item[2]['state_stamp']))
    if report.get('current_motion_quality') != 1:
        raise ValueError('ORIGINAL_NATIVE_MOTION_QUALITY_UNQUALIFIED')
    T = native_pose_at(evidence, stamp, T_map_world)
    return T, {'frame_id': evidence['frame_id'], 'update_sequence': evidence['update_sequence'],
               'epoch_source_identity': evidence['epoch_source_identity'],
               'state_stamp_s': evidence['state_stamp'], 'gnss_stamp_s': evidence['gnss_stamp'],
               'evaluation_stamp_s': stamp, 'covariance_used': False}


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
