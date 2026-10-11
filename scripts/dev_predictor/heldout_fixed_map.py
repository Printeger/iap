"""Same-map diagnostic using real scans from separately owned runs.

This conditions on the registered map; it never renews its age or authorizes
execution. GNSS/reference/state remain those of each original scan request.
All existing runs are diagnostic, even when scan/map source sets are disjoint.
"""
import argparse
import csv
import itertools
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'launch/_includes'))
from run_directory import adopt_run_directory, write_subordinate_manifest
from advisory_validation import sha, safe_label, artifact


def grouped(A, b, w, keys):
    _, groups = np.unique(keys, axis=0, return_inverse=True)
    weights = np.sqrt(w / np.bincount(groups)[groups])
    return A * weights[:, None], b * weights


def position_metric(A, K):
    # Full nuisance projection: unobservable attitude is allowed, unobservable
    # position is not. Use measurement SVD independently of production Schur.
    p, n = A[:, :3], A[:, 3:]
    if n.shape[1]:
        u, s, _ = np.linalg.svd(n, full_matrices=False)
        rank = np.sum(s > max(1e-10, s[0] * 1e-8)) if len(s) else 0
        p = p - u[:, :rank] @ (u[:, :rank].T @ p)
    h = p.T @ p / 2.  # Original single/two-source envelope, unchanged.
    ev, axes = np.linalg.eigh(h)
    if ev[0] <= max(1e-10, ev[-1] * 1e-8):
        return {'valid': False, 'reason': 'POSITION_RANK_DEFICIENT', 'eigenvalues': ev.tolist()}
    cov = np.linalg.inv(h)
    return {'valid': True, 'H_m': float(K * np.sqrt(np.linalg.eigvalsh(cov[:2, :2])[-1])),
            'weak_axis': axes[:, 0].tolist(), 'covariance_m2': cov.tolist()}


def solve(A, b, expected, weak):
    _, s, _ = np.linalg.svd(A, full_matrices=False)
    observable = position_metric(A, 1.)
    if not observable['valid']:
        return {'valid': False, 'reason': 'POSITION_UNOBSERVABLE_AFTER_NUISANCE'}
    x = np.linalg.lstsq(A, b, rcond=1e-8)[0]
    e = x[:6] - expected
    return {'valid': True, 'increment': x[:6].tolist(),
            'horizontal_error_m': float(np.linalg.norm(e[:2])),
            'position_error_m': float(np.linalg.norm(e[:3])),
            'weak_error_m': float(abs(e[:3] @ weak)),
            'unobservable_nuisance_directions': int(A.shape[1]-np.sum(s>max(1e-10,s[0]*1e-8))),
            'postfit_rms_mixed_whitened': float(np.sqrt(np.mean((A @ x - b)**2)))}


def primitives(path):
    rows = list(csv.DictReader(path.open()))
    return (np.array([[float(v[k]) for k in ('x', 'y', 'z')] for v in rows]),
            np.array([[float(v[k]) for k in ('nx', 'ny', 'nz')] for v in rows]),
            np.array([float(v['weight']) * float(v['confidence']) for v in rows]))


def source_map_identity(meta_path, cfg, inputs):
    """Bind decoded geometry to its exact original recording and producer run."""
    m = json.loads(meta_path.read_text())
    source_run = Path(cfg['gpu']).parents[2]
    request_csv = source_run / f"export/advisory/validation/recordings/dense_off{cfg['seed']}_requests.csv"
    records = list(csv.DictReader(request_csv.open()))
    payload = meta_path.with_name('input.bin')
    digest = sha(payload)
    matched = []
    for v in records:
        if v['payload']:
            original = Path(v['payload'])
            if not original.is_absolute():
                original = source_run / original
            if sha(original) == digest:
                assert v['run_id'] == source_run.name
                matched.append((v, original))
    assert len(matched) == 1, 'ORIGINAL_RECORDING_IDENTITY_NOT_UNIQUE'
    record, original = matched[0]
    manifest = source_run / 'metadata/run_manifest.json'
    primary = json.loads(manifest.read_text())
    assert primary['scenario'] == 'icra_dense_forest_four_fork_v2'
    assert primary['source']['git_worktree_clean'], 'SOURCE_RUN_NOT_CLEAN'
    for p in (meta_path, payload, request_csv, original, manifest):
        inputs[str(p)] = sha(p)
    return {'source_run': str(source_run), 'source_run_id': source_run.name,
            'request_id': record['request_id'], 'payload_sha256': digest,
            'frame_id': m['estimation_frame_id'], 'cloud_stamp_s': m['cloud_stamp'],
            'generation': m['generation'], 'geometry_id': m['geometry_id'],
            'heldout_proof': 'different resolver-owned producer processes and original request/run identities; same simulator scene is shared, not independent maps'}


def evaluate(cfg, run, label):
    old = Path(cfg['diagnostic_owner'])
    maps = {seed: json.loads((old / f'metadata/config/dense_{seed}_increment_v1.json').read_text())
            for seed in cfg['seeds']}
    for seed, c in maps.items():
        c['seed'] = seed
    map_meta = Path(cfg['map_metadata'])
    m = json.loads(map_meta.read_text())
    inputs = {str(Path(__file__)): sha(Path(__file__)), cfg['_config_path']: sha(Path(cfg['_config_path']))}
    map_id = source_map_identity(map_meta, maps[cfg['map_seed']], inputs)
    c_all, n_all, w_all = primitives(map_meta.with_name('lidar_primitives.csv'))
    inputs[str(map_meta.with_name('lidar_primitives.csv'))] = sha(map_meta.with_name('lidar_primitives.csv'))
    out = artifact(run, f'export/analysis/{label}/summary.json').parent
    assert not (out / 'summary.json').exists(), 'EVIDENCE_ALREADY_EXISTS'
    rows = []
    for seed, original_cfg in maps.items():
        sp = old / f'export/analysis/dense_{seed}_increment_v1/summary.json'
        summary = json.loads(sp.read_text())
        inputs[str(sp)] = sha(sp)
        metadata = {json.loads(p.read_text())['estimation_frame_id']: p
                    for p in Path(original_cfg['decoded_root']).glob(original_cfg['decoded_glob'])}
        for r in summary['rows']:
            row = {'seed': seed, 'frame_id': r['frame_id'], 'qualified': False,
                   'position': r.get('position'), 'diagnostic_only': True}
            rows.append(row)
            try:
                assert r.get('qualified') and r['full_original_inliers'], 'ORIGINAL_CAPTURE_NOT_COMPLETE'
                mp = metadata[r['frame_id']]
                current = json.loads(mp.read_text())
                source_id = source_map_identity(mp, original_cfg, inputs)
                row['source_identity'] = source_id
                row['source_sets_disjoint'] = seed != cfg['map_seed']
                # Same-run rows are retained as an explicitly self-inclusive control.
                assert current['geometry_id'] == m['geometry_id'] and current['frame_id'] == m['frame_id'], 'MAP_FRAME_MISMATCH'
                assert current['coordinates']['map_frame'] == m['coordinates']['map_frame'], 'COORDINATE_CONTRACT_MISMATCH'
                assert np.max(abs(np.array(current['coordinates']['T_map_world'])-
                                     np.array(m['coordinates']['T_map_world'])))<1e-9, 'CROSS_RUN_MAP_ALIGNMENT_MISMATCH'
                for key in ('lidar_radius_m', 'lidar_sigma', 'lidar_support_voxel_m', 'lidar_weight_scale', 'K_H_adv', 'K_V_adv'):
                    assert current[key] == m[key], 'FROZEN_PARAMETER_MISMATCH:'+key
                ap = sp.with_name(f"frame_{r['frame_id']}.json")
                a = json.loads(ap.read_text()); inputs[str(ap)] = sha(ap)
                p = np.array(current['position'])
                radius = m['lidar_radius_m']
                ids = np.flatnonzero(np.sum((c_all-p)**2, axis=1) <= radius**2)
                assert len(ids) >= 6, 'NO_FROZEN_MAP_SUPPORT'
                c, n = c_all[ids], n_all[ids]
                keys = np.column_stack((np.floor(c/m['lidar_support_voxel_m']+1e-9).astype('int64'), np.argmax(abs(n), axis=1)))
                A = np.column_stack((n, np.cross(c-p, n)))
                w = w_all[ids] * m['lidar_weight_scale'] * np.exp(-np.sum((c-p)**2, axis=1)/(2*radius**2)) / m['lidar_sigma']**2
                modelA, _ = grouped(A, np.zeros(len(A)), w, keys)
                ga = np.array(a['admitted_gnss_design']); gb = np.array(a['admitted_gnss_residual_whitened'])
                joint = np.vstack((np.pad(modelA, ((0, 0), (0, ga.shape[1]-6))), ga))
                model = position_metric(joint, m['K_H_adv'])
                row.update(model=model, production_groups=len(np.unique(keys, axis=0)))
                assert model['valid'], model.get('reason')
                T = np.array(current['coordinates']['T_map_world']).reshape(4, 4) @ np.array(current['coordinates']['T_world_imu']).reshape(4, 4)
                points = np.array(a['reference_body_points']) @ T[:3, :3].T + T[:3, 3]
                # Keep original primitive order for ties, exactly as the old
                # evaluator; chunk only memory, not sampling or associations.
                matches = np.concatenate([np.argmin(np.sum((part[:, None, :]-c[None, :, :])**2, axis=2), axis=1)
                                          for part in np.array_split(points, max(1, (len(points)+255)//256))])
                distances = np.linalg.norm(points-c[matches], axis=1)
                keep = (distances <= cfg['association_distance_m']) & (np.sum((points-p)**2, axis=1) <= radius**2)
                correspondence = [{'source_index': s['source_index'], 'gpu_target_index': s['target_index'],
                                   'primitive_index': int(ids[j]), 'distance_m': float(d), 'used': bool(ok),
                                   'family': keys[j].tolist()}
                                  for s, j, d, ok in zip(a['source_packet']['samples'], matches, distances, keep)]
                cp = out / f'seed_{seed}_frame_{r["frame_id"]}_correspondence.json'
                cp.write_text(json.dumps({'map_identity': map_id, 'scan_identity': source_id,
                                         'source_stamp_s': a['source_packet']['source_stamp'],
                                         'reference_s': current['reference_time_s'], 'rows': correspondence})+'\n')
                row.update(original_slots=r['original_source_count'], captured_matches=len(points),
                           association_count=int(np.sum(keep)), missing_association=int(np.sum(~keep)),
                           outside_radius=int(np.sum(np.sum((points-p)**2, axis=1)>radius**2)))
                assert np.sum(keep) >= 6, 'INSUFFICIENT_ACTUAL_ASSOCIATIONS'
                points, j = points[keep], matches[keep]
                normals = n[j]
                actualA = np.column_stack((normals, np.cross(points-p, normals)))
                residual = np.einsum('ni,ni->n', normals, c[j]-points)
                la, lb = grouped(actualA, residual, w[j], keys[j])
                both = np.vstack((np.pad(la, ((0, 0), (0, ga.shape[1]-6))), ga))
                actual = position_metric(both, m['K_H_adv'])
                solution = solve(both, np.r_[lb, gb], np.array(r['true_local_increment']), np.array(model['weak_axis']))
                # Evaluation-only decomposition. Truth is not inserted into the
                # estimate above or the predictor. It separates family-persistent
                # map/association mismatch from within-family variation; it is
                # not a calibrated source noise or a proposed production fix.
                expected = np.array(r['true_local_increment'])
                at_truth = residual - actualA @ expected
                family_keys, family = np.unique(keys[j], axis=0, return_inverse=True)
                family_mean = np.bincount(family, weights=at_truth) / np.bincount(family)
                bias = family_mean[family]
                _, biasb = grouped(actualA, bias, w[j], keys[j])
                _, centeredb = grouped(actualA, at_truth-bias, w[j], keys[j])
                bias_only = solve(la, biasb, np.zeros(6), np.array(model['weak_axis']))
                centered_only = solve(la, centeredb, np.zeros(6), np.array(model['weak_axis']))
                row['evaluation_only_residual_decomposition'] = {
                    'family_persistent_component': bias_only,
                    'within_family_component': centered_only,
                    'at_truth_rms_m': float(np.sqrt(np.mean(at_truth**2))),
                    'family_mean_rms_m': float(np.sqrt(np.mean(family_mean**2))),
                    'within_family_rms_m': float(np.sqrt(np.mean((at_truth-bias)**2))),
                    'family_mean_rows': [{'key': key.tolist(), 'mean_m': float(mean),
                                          'sample_count': int(count)}
                                         for key, mean, count in zip(family_keys, family_mean, np.bincount(family))],
                    'uses_truth_only_for_diagnosis': True,
                    'qualified_noise_model': False}
                row.update(actual_geometry=actual, actual=solution,
                           matched_groups=len(np.unique(keys[j], axis=0)),
                           information_difference=float(np.linalg.norm(la.T@la-modelA.T@modelA)))
                if seed == cfg['map_seed'] and r['frame_id'] == m['estimation_frame_id']:
                    assert abs(model['H_m']-r['model_h_m'])<1e-9, 'ORIGINAL_NATIVE_MODEL_CROSSCHECK_FAILED'
                    assert abs(solution['position_error_m']-r['fusion']['position_error_norm_m'])<1e-9, 'ORIGINAL_SOLVE_CROSSCHECK_FAILED'
                    row['original_native_model_and_solve_crosscheck'] = True
                if actual['valid']:
                    row['weak_alignment'] = float(abs(np.array(actual['weak_axis'])@np.array(model['weak_axis'])))
                row['qualified'] = solution['valid'] and actual['valid']
                if not row['qualified']: row['reason'] = 'ACTUAL_SOLVE_OR_GEOMETRY_UNOBSERVABLE'
            except (AssertionError, KeyError, ValueError, np.linalg.LinAlgError) as e:
                row['reason'] = str(e)
    regional = []
    for seed in cfg['seeds']:
        eligible = [r for r in rows if r['seed'] == seed and r['qualified']]
        regions = []
        for lo, hi in cfg['regions_x_m']:
            selected = [r for r in eligible if lo <= r['position'][0] < hi]
            if selected:
                regions.append({'x_interval': [lo, hi], 'frames': [r['frame_id'] for r in selected],
                                'median_model_H_m': float(np.median([r['model']['H_m'] for r in selected])),
                                'median_horizontal_error_m': float(np.median([r['actual']['horizontal_error_m'] for r in selected])),
                                'median_position_error_m': float(np.median([r['actual']['position_error_m'] for r in selected]))})
        pairs = [{'a': a['x_interval'], 'b': b['x_interval'],
                  'horizontal_concordant': bool((a['median_model_H_m']-b['median_model_H_m'])*
                                               (a['median_horizontal_error_m']-b['median_horizontal_error_m'])>0),
                  'position_concordant': bool((a['median_model_H_m']-b['median_model_H_m'])*
                                             (a['median_position_error_m']-b['median_position_error_m'])>0)}
                 for a, b in itertools.combinations(regions, 2)]
        regional.append({'seed': seed, 'source_disjoint_control': seed != cfg['map_seed'], 'regions': regions,
                         'pairs': pairs, 'horizontal_concordance': float(np.mean([p['horizontal_concordant'] for p in pairs])) if pairs else None,
                         'original_position_concordance': float(np.mean([p['position_concordant'] for p in pairs])) if pairs else None})
    lineage = None
    if cfg.get('native_map_metadata'):
        path = Path(cfg['native_map_metadata']); inputs[str(path)] = sha(path)
        native = json.loads(path.read_text())
        assert native['prediction_input_identity'] == m['prediction_input_identity'], 'NATIVE_LINEAGE_INPUT_MISMATCH'
        lineage = native['map_source_evidence']
    result = {'schema': 'iap_heldout_same_map_diagnostic_v1', 'map_identity': map_id,
              'map_source_evidence': lineage, 'regional_comparisons': regional,
              'rows': rows, 'missing_original_requests': {s: c['missing_requests'] for s, c in maps.items()},
              'claims': {'production_model_changed': False, 'new_independent_validation': False,
                         'map_bias_qualified': False, 'execution_authorized': False,
                         'freshness_renewed': False, 'truth_used_in_prediction': False},
              'limits': ['same physical scene and sensor generator; independent runs do not mean independent scenes',
                         'map conditions on original estimated alignment; common map bias retained',
                         'GNSS rows are original request rows, not recomputed frozen-map canopy visibility',
                         'association/coverage losses retained; newest fixed target level0 source only']}
    with (out/'summary.json').open('x') as stream: json.dump(result, stream, indent=2)
    write_subordinate_manifest(run, label, {'inputs_sha256': inputs,
                              'artifacts_sha256': {str(p.relative_to(run)): sha(p) for p in out.glob('*.json')},
                              'qualification': 'diagnostic fixed-map cross-run source-disjoint comparison; no heldout model validation'})
    print(json.dumps({'output': str(out), 'rows': len(rows), 'qualified': sum(r['qualified'] for r in rows),
                      'qualified_disjoint': sum(r['qualified'] and r.get('source_sets_disjoint', False) for r in rows)}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path); parser.add_argument('label')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    config['_config_path'] = str(args.config.resolve())
    evaluate(config, adopt_run_directory(config['owner']), safe_label(args.label))
