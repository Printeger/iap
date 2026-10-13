"""One fixed A0 development rejection check on already viewed evidence.

No truth or future endpoint contributes to any predictor input. Fixed-map
history uses the original forecast operator and checks its old position score.
The native current-map projection and G1 rows are reported separately.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import sys
import traceback

import numpy as np

from advisory_validation import sha, source_identity, json_write, safe_label
from fusion_scientific_audit import gnss_reference
from heldout_fixed_map import grouped, position_metric, primitives
from horizontal_probe_advisory import (project_information, clock_projected_rows,
                                       svd_probe_matrix, MODEL_VERSION,
                                       translation_gauge_information)
from local_increment_validation import regional_results, REGIONS, GATES
from surface_model_probe import sensor_support, sampled_first_hit_support

REPO = Path(__file__).resolve().parents[2]
A1 = 'pose6_lidar_translation_gauge_probe_a1'
sys.path.insert(0, str(REPO/'launch/_includes'))
from run_directory import (resolve_run_directory, adopt_run_directory,
                           write_subordinate_manifest, finalize_run)


class Evidence:
    def __init__(self):
        self.inputs = {}
        self.missing = []

    def bind(self, p):
        p = Path(p)
        self.inputs[str(p.resolve())] = sha(p)
        return p

    def read(self, p):
        return json.loads(self.bind(p).read_text())

    def table(self, p):
        with self.bind(p).open() as f:
            return list(csv.DictReader(f))

    def payload(self, metadata, *, allow_missing=False):
        p = metadata.with_name('input.bin')
        if allow_missing and not p.is_file():
            self.missing.append(str(p))
            return False
        self.bind(p)
        return True


def matrix(value, name):
    return np.array(value[name]['row_major']).reshape(6, 6)


def source_root(w, v):
    """Equivalent rows of native LiDAR information; no visibility revalidation."""
    return np.sqrt(np.maximum(0., w))[:, None]*v.T


def native_projection(point, matrices, metadata):
    out = project_information(matrix(matrices, 'joint_pose_information'),
                              metadata['K_H_adv'], metadata['K_V_adv'],
                              input_status=point['status'])
    if point['status'] == 'VALID':
        zero = project_information(matrix(matrices, 'joint_pose_information'),
                                   metadata['K_H_adv'], metadata['K_V_adv'], zero_probe=True)
        # These recordings use original zero b/s; check against native H/V,
        # rather than treating missing exported offsets as arbitrary zero.
        if not np.allclose([zero['Q_H'], zero['Q_V']],
                           [float(point['module_hpl']), float(point['module_vpl'])],
                           rtol=1e-8, atol=1e-9):
            raise ValueError('ORIGINAL_POSITION_OR_ZERO_OFFSET_BINDING_MISMATCH')
        out['zero_probe_native_HV_checked'] = True
    return out


def g1_checks(ev):
    path = REPO/'log/20261012T163704Z_119/export/analysis/g1/aligned.json'
    g1 = ev.read(path)
    checks, rows = [], []
    for value in g1['input_binding'].values():
        ev.bind(value['payload'])
        if sha(value['payload']) != value['payload_sha256']:
            raise ValueError('G1_PAYLOAD_HASH_MISMATCH')
    for r in g1['rows']:
        if 'native_matrices' not in r:
            continue
        meta = g1['effective_inputs'][r['request_id']]
        score = native_projection(r['native_point'], r['native_matrices'], meta)
        rows.append(dict(request_id=r['request_id'], query_id=r['query_id'], mode=r['mode'],
                         original_status=r['native_point']['status'], score=score))
        if r['mode'] != 'fusion' or r['native_point']['status'] != 'VALID':
            continue
        h = matrix(r['native_matrices'], 'joint_pose_information')
        l = matrix(r['native_matrices'], 'lidar_pose_information')
        rot = np.array(meta['coordinates']['T_map_world']).reshape(4, 4)[:3, :3]
        body = np.array(meta['coordinates']['T_world_imu']).reshape(4, 4)[:3, :3]
        lever = rot@body@np.array(meta['coordinates']['lever_arm_imu'])
        g = gnss_reference(r['admitted_gnss_rows']['rows'], lever)
        wl, vl = np.linalg.eigh(l)
        jl = source_root(wl, vl)
        if r['native_point']['gnss_used'] != '1':
            g = np.zeros((0, g.shape[1]))
        a = np.vstack((g, np.pad(jl, ((0, 0), (0, g.shape[1]-6)))))/np.sqrt(2)
        reference, observable = svd_probe_matrix(a)
        clock_rows = clock_projected_rows(a)
        checks.append({'request_id': r['request_id'], 'query_id': r['query_id'],
                       'check': 'measurement_SVD_probe_and_joint',
                       'max_abs_matrix_error': float(np.max(abs(reference-np.array(score['projected_matrix'])))),
                       'pass': bool(np.allclose(clock_rows.T@clock_rows, h, rtol=2e-8, atol=1e-8)
                                    and np.allclose(reference, score['projected_matrix'], rtol=2e-8, atol=1e-10)
                                    and observable.all()),
                       'scope': 'exported GNSS rows; equivalent native LiDAR square root'})
        # The existing G1 fixed admitted-row controls are a numerical
        # monotonicity check, not three fits or evidence of true global drift.
        control = []
        for sigma in (2., 1., .5):
            controlled = np.vstack((g/sigma, np.pad(jl, ((0, 0), (0, g.shape[1]-6)))))/np.sqrt(2)
            cp = clock_projected_rows(controlled)
            control.append(project_information(cp.T@cp, meta['K_H_adv'], meta['K_V_adv']))
        checks.append({'request_id': r['request_id'], 'query_id': r['query_id'],
                       'check': 'fixed_LiDAR_GNSS_monotone', 'sigma_scales': [2., 1., .5],
                       'scores': [s['S'] for s in control],
                       'pass': all(s['valid'] for s in control) and
                               control[0]['S']+1e-10 >= control[1]['S'] and
                               control[1]['S']+1e-10 >= control[2]['S'],
                       'admitted_GNSS_rows': len(g), 'true_global_anchor_qualification': False})
        if not len(g):
            checks[-1]['pass'] = None
            checks[-1]['status'] = 'UNAVAILABLE_EMPTY_GNSS_SET'
    return rows, checks


def inside_regions(position):
    return position is not None and REGIONS[0][0] <= position[0] <= REGIONS[-1][1]


def ordering(rows, field, label, score_key='score'):
    converted = []
    for row in rows:
        v = dict(row)
        v['qualified'] = bool(row.get('qualified') and
                              (row['seed'] != 7400 or inside_regions(row.get('position')))
                              and row.get(score_key, {}).get(field) is not None)
        if v['qualified']:
            v['baseline_H_m'] = row[score_key][field]
            relative = dict(row['relative'])
            if label == 'abs_z':
                relative['horizontal_error_m'] = abs(relative['translation_error_start_body_m'][2])
            v['relative'] = relative
        converted.append(v)
    out = regional_results(converted)
    for r in out:
        r['prediction_field'] = field
        r['label'] = label
        for region in r['regions']:
            if 'baseline_H_m' in region:
                region['dimensionless_score'] = region.pop('baseline_H_m')
    return out


def a1_projection(ga, la, K_H, K_V):
    g = clock_projected_rows(ga)
    l = translation_gauge_information(la.T@la)
    result = project_information((g.T@g+l)/2, K_H, K_V)
    result['model_version'] = A1
    result['semantic_id'] = 'dimensionless_translation_gauge_horizontal_probe_0p5m_v1'
    # Independent measurement projection removes the map translation rows.
    u, s, _ = np.linalg.svd(la[:, :3], full_matrices=False)
    keep = s > np.sqrt(64*np.finfo(float).eps)*(s[0] if len(s) else 0.)
    projected_l = la-u[:, keep]@(u[:, keep].T@la)
    ref, observed = svd_probe_matrix(np.vstack((g, projected_l))/np.sqrt(2))
    checked = bool(np.allclose(ref, result['projected_matrix'], rtol=2e-7, atol=1e-8)
                   and np.array_equal(observed, result['projected_observable']))
    return result, checked


def development(ev):
    map_path = REPO/'log/20261011T123441Z_520/export/advisory/validation/actual_measure_rows_v1/input.json'
    m = ev.read(map_path)
    ev.payload(map_path)
    primitive = ev.bind(map_path.with_name('lidar_primitives.csv'))
    c, n, w = primitives(primitive)
    raw_path = REPO/'log/20261011T123441Z_520/export/advisory/validation/actual_measure_map_v1/raw_centers.csv'
    raw = ev.table(raw_path)
    origin = np.array(m['origin'])
    occupied = {tuple(np.floor((np.array([float(v[k]) for k in ('x', 'y', 'z')])-origin)
                              /m['resolution_m']+1e-8).astype('int64')) for v in raw}
    keys = np.c_[np.floor(c/m['lidar_support_voxel_m']+1e-9).astype('int64'), np.argmax(abs(n), axis=1)]
    historical = ev.read(REPO/'log/20261011T160805Z_137/export/analysis/local_increment_baseline_v1/summary.json')
    pilot_root = REPO/'log/20261011T170422Z_954'
    pilot = ev.read(pilot_root/'export/analysis/native_pair_response_discriminator_scope_v2/summary.json')
    pilot_native = ev.read(pilot_root/'export/analysis/native_increment_representation_v1/summary.json')
    old = REPO/'log/20261011T044535Z_421'
    rows, checks = [], []
    for original in historical['rows']+pilot['rows']:
        r = {k: v for k, v in original.items() if k in
             ('seed', 'frame_id', 'pair_index', 'position', 'qualified', 'reason',
              'evaluation_reason', 'relative', 'baseline_H_m', 'start_s', 'reference_s')}
        r['label_representation'] = 'v1.2_native' if r['seed'] == 7400 else 'v1.1_publication_diagnostic'
        r['inside_predeclared_regions'] = inside_regions(r.get('position'))
        rows.append(r)
        # Preserve future endpoint failures in the denominator. Query-valid
        # scores can still be calculated and retained on those failed labels.
        if r['frame_id'] is None:
            continue
        if r['seed'] == 7400:
            index = 2*r['pair_index']
            mp = pilot_root/f'export/advisory/validation/native_pair_{index}/input.json'
            current = ev.read(mp)
            native_dir = pilot_root/f'export/advisory/validation/paired_response_rows_{index}_current'
            obs = ev.read(native_dir/'gnss_information_rows.jsonl')['rows']
            rot = np.array(current['coordinates']['T_map_world']).reshape(4, 4)[:3, :3]
            body = np.array(current['coordinates']['T_world_imu']).reshape(4, 4)[:3, :3]
            ga = gnss_reference(obs, rot@body@np.array(current['coordinates']['lever_arm_imu']))
            # Original baseline used the same rows with zero antenna columns.
            # The recorded antenna lever is zero; do not silently change it.
            if not np.allclose(ga[:, 3:6], 0., atol=1e-12):
                raise ValueError('HISTORICAL_ZERO_LEVER_ASSUMPTION_MISMATCH')
            # Sign conventions are clock equivalent with zero lever.
            ga[:, :3] *= -1
            ev.payload(native_dir/'input.json')
            native_mat = ev.read(native_dir/'matrices.jsonl')
            native_meta = ev.read(native_dir/'input.json')
            native_point = ev.table(native_dir/'points.csv')[0]
            r['current_map_native_score'] = native_projection(native_point, native_mat, native_meta)
            r['native_label'] = next(v for v in pilot_native['rows'] if v['pair_index'] == r['pair_index'])
        else:
            cfg = ev.read(old/f"metadata/config/dense_{r['seed']}_increment_v1.json")
            candidates = [p for p in Path(cfg['decoded_root']).glob(cfg['decoded_glob'])
                          if json.loads(p.read_text())['estimation_frame_id'] == r['frame_id']]
            if len(candidates) != 1:
                raise ValueError('HISTORICAL_QUERY_IDENTITY_NOT_UNIQUE')
            mp = candidates[0]
            current = ev.read(mp)
            r['original_payload_available'] = ev.payload(mp, allow_missing=True)
            prior = ev.read(old/f"export/analysis/dense_{r['seed']}_increment_v1/frame_{r['frame_id']}.json")
            ga = np.array(prior['admitted_gnss_design'])
        for k in ('lidar_radius_m', 'lidar_sigma', 'lidar_support_voxel_m', 'lidar_weight_scale', 'K_H_adv', 'K_V_adv'):
            if current[k] != m[k]:
                raise ValueError('FIXED_PARAMETERS_MISMATCH:'+k)
        if current['coordinates']['T_map_world'] != m['coordinates']['T_map_world']:
            raise ValueError('FIXED_MAP_FRAME_MISMATCH')
        p = np.array(current['position'])
        T = np.array(current['coordinates']['T_map_world']).reshape(4, 4)@np.array(current['coordinates']['T_world_imu']).reshape(4, 4)
        Ts = T@np.linalg.inv(np.array(current['coordinates']['T_lidar_imu']).reshape(4, 4))
        weights = w*m['lidar_weight_scale']*np.exp(-np.sum((c-p)**2, axis=1)/(2*m['lidar_radius_m']**2))/m['lidar_sigma']**2
        e = m['map_source_evidence']
        visible = sensor_support(c, Ts, e['current_observation_frame']) & (np.sum((c-p)**2, axis=1) <= m['lidar_radius_m']**2)
        visible[visible] = sampled_first_hit_support(Ts[:3, 3], c[visible], occupied, origin,
                                                    m['resolution_m'], Ts[:3, :3], e['forecast_sensor'])
        design = np.c_[n[visible], np.cross(c[visible]-p, n[visible])]
        la, _ = grouped(design, np.zeros(sum(visible)), weights[visible], keys[visible])
        a = np.vstack((np.pad(la, ((0, 0), (0, ga.shape[1]-6))), ga))
        base = position_metric(a, m['K_H_adv'])
        if not base['valid']:
            r.update(qualified=False, prediction_reason=base['reason'])
            continue
        if 'baseline_H_m' in original and not np.isclose(base['H_m'], original['baseline_H_m'], rtol=2e-7, atol=1e-9):
            raise ValueError('HISTORICAL_FORECAST_POSITION_SCORE_MISMATCH:'+str((r['seed'], r['frame_id'], base['H_m'], original['baseline_H_m'])))
        pose = clock_projected_rows(a)/np.sqrt(2)
        r['score'] = project_information(pose.T@pose, m['K_H_adv'], m['K_V_adv'])
        r['a1_score'], a1_checked = a1_projection(ga, la, m['K_H_adv'], m['K_V_adv'])
        zero = project_information(pose.T@pose, m['K_H_adv'], m['K_V_adv'], zero_probe=True)
        r['position_baseline_Q_H'] = zero['Q_H']
        r['position_baseline_Q_V'] = zero['Q_V']
        r['new_unavailable'] = base['valid'] and not r['score']['valid']
        r['source_support'] = {'visible_primitives': int(sum(visible)), 'GNSS_rows': len(ga)}
        ref, observed = svd_probe_matrix(a/np.sqrt(2))
        checks.append({'seed': r['seed'], 'frame_id': r['frame_id'], 'check': 'fixed_forecast_SVD',
                       'pass': bool(np.allclose(ref, r['score']['projected_matrix'], rtol=2e-8, atol=1e-10)
                                    and np.array_equal(observed, r['score']['projected_observable'])
                                    and np.isclose(zero['Q_H'], base['H_m'], rtol=2e-8))})
        checks.append({'seed': r['seed'], 'frame_id': r['frame_id'],
                       'check': 'translation_gauge_measurement_SVD', 'pass': a1_checked})
        print(r['seed'], r['frame_id'], r['score']['Q_H'], r['score']['Q_V'], r['score']['S'], flush=True)
    return rows, checks


def evaluate(run, label):
    out = run/'export/analysis'/label
    out.mkdir(exist_ok=False)
    ev = Evidence()
    for name in ('horizontal_probe_advisory.py', 'g2_a0_rejection_check.py', 'local_increment_validation.py',
                 'surface_model_probe.py', 'heldout_fixed_map.py'):
        path = ev.bind(Path(__file__).with_name(name))
        # Dirty offline work still needs immutable source bytes, not only SHA.
        snapshot = run/'metadata/config'/label/name
        snapshot.parent.mkdir(exist_ok=True)
        with snapshot.open('xb') as f:
            f.write(path.read_bytes())
    g1, checks = g1_checks(ev)
    rows, development_checks = development(ev)
    checks += development_checks
    result = {'schema': 'iap_g2_a0_rejection_v1', 'model_version': MODEL_VERSION,
              'identity': 'VIEWED_DEVELOPMENT_ONLY', 'gates': GATES, 'regions': REGIONS,
              'rows': rows, 'g1_single_source_and_controls': g1, 'mechanism_checks': checks,
              'mechanism_pass': bool(checks) and all(c['pass'] is True or
                                      c.get('status') == 'UNAVAILABLE_EMPTY_GNSS_SET' for c in checks),
              'ordering': {field: ordering(rows, field, 'abs_z' if field == 'Q_V' else 'XY')
                           for field in ('Q_H', 'Q_V', 'S')},
              'native_current_map_ordering': {
                  field: ordering([r for r in rows if r['seed'] == 7400], field,
                                  'abs_z' if field == 'Q_V' else 'XY', 'current_map_native_score')
                  for field in ('Q_H', 'Q_V', 'S')},
              'a1_ordering': {field: ordering(rows, field, 'abs_z' if field == 'Q_V' else 'XY', 'a1_score')
                             for field in ('Q_H', 'Q_V', 'S')},
              'a1_new_unavailable_count': sum(r.get('score', {}).get('valid', False) and
                                              not r.get('a1_score', {}).get('valid', False) for r in rows),
              'new_unavailable_count': sum(r.get('new_unavailable', False) for r in rows),
              'independent_qualification': False, 'true_global_anchor_qualification': False,
              'production_model_changed': False, 'truth_in_prediction': False,
              'inputs_sha256': ev.inputs, 'missing_original_payloads': ev.missing,
              'missing_payload_scope': 'historical decoded diagnostic only; no full native replay qualification'}
    target = out/'summary.json'
    json_write(target, result)
    write_subordinate_manifest(run, label, {'source': source_identity(), 'argv': sys.argv,
                                          'inputs_sha256': ev.inputs, 'result_sha256': sha(target),
                                          'qualification': 'DEVELOPMENT_REJECTION_CHECK_ONLY'})
    print(json.dumps({'run': str(run), 'mechanism_pass': result['mechanism_pass'],
                      'summary': str(target), 'a1_new_unavailable': result['a1_new_unavailable_count']}))
    return 0 if result['mechanism_pass'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    label = safe_label(args.label)
    inherited = os.environ.get('IAP_RUN_DIR')
    run = adopt_run_directory(inherited) if inherited else resolve_run_directory(
        entrypoint='g2_a0_rejection_check', scenario='icra_dense_forest_four_fork_v2')
    code = 1
    try:
        code = evaluate(run, label)
        return code
    except Exception:
        failure = run/'runtime'/f'{label}_failure.txt'
        # Existing evidence is never overwritten, including an earlier failure.
        if not failure.exists():
            failure.write_text(traceback.format_exc())
        raise
    finally:
        if not inherited:
            finalize_run(run, lifecycle='completed' if code == 0 else 'failed',
                         safety_outcome='not_applicable')


if __name__ == '__main__':
    raise SystemExit(main())
