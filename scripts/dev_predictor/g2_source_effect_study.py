"""Bounded G2 source-effect protocol and fixed-reference offline means.

No truth enters an online predictor, no old probe is refitted, and preparation
or evaluator tests grant no source-effect/model qualification.
"""
import argparse
from collections import Counter
import csv
import gzip
import json
import os
import subprocess
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'launch/_includes'))
from advisory_validation import artifact, json_write, sha, source_identity, safe_label, validate_record, binary_identity, backend
from local_increment_validation import rigid, GATES, REGIONS, PoseSeries, native_endpoint, relative_error
from run_directory import resolve_run_directory, adopt_run_directory, register_subordinate_manifest, finalize_run


def pilot_protocol():
    return {
        'schema': 'iap_source_effect_protocol_v1',
        'conditions': {name: {'gnss_dropout': name.startswith('GB'),
                             'lidar_max_range_m': 10. if name.endswith('LG') else 3.}
                       for name in ('GG_LG', 'GG_LB', 'GB_LG', 'GB_LB')},
        'duration_s': 105., 'dropout_elapsed_s': [30., 60.],
        'record_interval_s': 2., 'record_initial_delay_s': 10., 'record_count': 92,
        'pilot_seed': 9100, 'formal_seeds_reserved': [9101, 9102, 9103],
        'formal_authorized_by_this_preparation': False,
        'scene': 'icra_dense_forest_four_fork_v2', 'map_seed': 41021,
        'fixed_origin_lla': [31.2304, 121.4737, 25.],
        'reference': 'analytic WGS84 producer ECEF/ENU; E0+Rew*p_world_imu; no fit',
        'phases_elapsed_s': {'pre': [15., 28.], 'absence': [32., 58.], 'post': [65., 95.]},
        'local': 'original native 1.000s start-body XY; original IMU and truth gates',
        'global': 'fixed ENU horizontal/3D error, phase change/growth and recovery correction',
        'global_phase_rule': '>=6 valid native states; median error; last-minus-first time-quartile median error growth',
        'detection_floors_m': {'local': .001, 'global': .02},
        'uncertainty': 'paired run/group unit; report every run; no window-as-N inference',
        'formal_rule': 'same paired effect direction in all three groups; near-floor/confounded is inconclusive',
        'local_ordering_gates': GATES, 'original_regions': REGIONS,
        'candidate': None, 'qualification': 'PREDECLARED_PILOT_ONLY',
        'confounders': ['actual trajectory', 'native FGO response', 'self-built map',
                        'missing GNSS-native local endpoints', 'ideal IMU/initial global datum',
                        'dropout skips producer random draws; recovery noise streams are not paired'],
    }


def fixed_reference(lla=(31.2304, 121.4737, 25.)):
    lat, lon = np.deg2rad(lla[:2]); alt = lla[2]
    a, e2 = 6378137., 6.6943799901413165e-3
    n = a / np.sqrt(1.-e2*np.sin(lat)**2)
    origin = np.array([(n+alt)*np.cos(lat)*np.cos(lon),
                       (n+alt)*np.cos(lat)*np.sin(lon),
                       (n*(1-e2)+alt)*np.sin(lat)])
    rotation = np.array([[-np.sin(lon), -np.sin(lat)*np.cos(lon), np.cos(lat)*np.cos(lon)],
                         [np.cos(lon), -np.sin(lat)*np.sin(lon), np.cos(lat)*np.sin(lon)],
                         [0., np.cos(lat), np.sin(lat)]])
    return origin, rotation


def fixed_global_error(row, truth_position):
    if any(int(row[key]) != 1 for key in ('pose_valid', 'datum_valid')) or int(row['fallback']):
        raise ValueError('NATIVE_GLOBAL_STATE_UNAVAILABLE_OR_FALLBACK')
    vector = lambda prefix: np.array([float(row[f'{prefix}_{axis}']) for axis in 'xyz'])
    p, anchor = vector('p_world_imu'), vector('anchor_ecef')
    rotation = np.array([float(row[f'R_ecef_world_{r}{c}']) for r in range(3) for c in range(3)]).reshape(3, 3)
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = anchor
    rigid(transform)
    truth_position = np.asarray(truth_position, float)
    if truth_position.shape != (3,) or not np.isfinite(np.r_[p, truth_position]).all():
        raise ValueError('GLOBAL_POSITION_NONFINITE')
    origin, enu = fixed_reference()
    predicted = anchor + rotation @ p
    error = enu.T @ (predicted - (origin + enu @ truth_position))
    return {'error_fixed_enu_m': error.tolist(), 'horizontal_error_m': float(np.linalg.norm(error[:2])),
            'error_3d_m': float(np.linalg.norm(error)), 'estimate_ecef_m': predicted.tolist(),
            'p_world_imu': p.tolist(), 'anchor_ecef': anchor.tolist(), 'R_ecef_world': rotation.tolist()}


def select_global_updates(rows):
    """Choose the latest completed sequence for an owner before reading truth."""
    selected = {}
    for row in rows:
        stamp, frame = float(row['state_stamp_s']), int(row['frame_id'])
        sequence = int(row['update_sequence'])
        if not np.isfinite(stamp) or frame < 0 or sequence <= 0:
            raise ValueError('NATIVE_GLOBAL_IDENTITY_INVALID')
        key = (stamp, frame)
        previous = selected.get(key)
        if previous and sequence == int(previous['update_sequence']) and row != previous:
            raise ValueError('CONFLICTING_GLOBAL_UPDATE_IDENTITY')
        if previous is None or sequence > int(previous['update_sequence']): selected[key] = row
    return [selected[key] for key in sorted(selected)]


def phase_statistics(rows, field, bounds):
    lo, hi = bounds
    selected = [r for r in rows if r.get('qualified') and lo <= r['elapsed_s'] <= hi]
    values = [r[field] for r in selected]
    first = [r[field] for r in selected if r['elapsed_s'] <= lo+(hi-lo)/4]
    last = [r[field] for r in selected if r['elapsed_s'] >= hi-(hi-lo)/4]
    return {'count': len(selected), 'median': float(np.median(values)) if values else None,
            'last_minus_first_quartile_median': float(np.median(last)-np.median(first)) if first and last else None,
            'phase_minimum_met': len(selected) >= 6,
            'independent_sample_count': 1 if selected else 0}


def recovery_decomposition(previous, current):
    """Exact two-update world-pose/datum split; not a causal factor attribution."""
    a, b = previous['global'], current['global']
    ra, rb = np.array(a['R_ecef_world']), np.array(b['R_ecef_world'])
    pa, pb = np.array(a['p_world_imu']), np.array(b['p_world_imu'])
    world = ra @ (pb-pa)
    datum = np.array(b['anchor_ecef'])-np.array(a['anchor_ecef'])+(rb-ra) @ pb
    _, enu = fixed_reference()
    return {'before_stamp_s': previous['stamp_s'], 'after_stamp_s': current['stamp_s'],
            'world_pose_change_fixed_enu_m': (enu.T @ world).tolist(),
            'datum_change_fixed_enu_m': (enu.T @ datum).tolist(),
            'truth_change_fixed_enu_m': (np.array(current['truth_position'])-previous['truth_position']).tolist(),
            'horizontal_error_change_m': b['horizontal_error_m']-a['horizontal_error_m'],
            'identity': 'FIRST_ACTUALLY_INJECTED_POST_DROPOUT_UPDATE; algebraic split, not causal isolation'}


def recorded_query_admission(metadata):
    # Export availability and a later qualified report cannot rehabilitate the
    # original frozen query. Retain both endpoint refusals in the pair ledger.
    if any(meta.get('current_motion_quality') != 1 for meta in metadata):
        raise ValueError('ORIGINAL_QUERY_MOTION_QUALITY_UNQUALIFIED')


def evaluate(run, source, label, binary, *, owner):
    """Original native local endpoints plus fixed global means; no score fit."""
    safe_label(label)
    output = artifact(run, 'export/analysis/' + label + '/summary.json')
    if output.exists(): raise ValueError('evaluation evidence already exists')
    result = {'schema': 'iap_g2_source_effect_evaluation_v1', 'local_rows': [], 'global_rows': [],
              'requests': [], 'inputs_sha256': {}, 'model_qualified': False,
              'source_effects_qualified': False, 'alignment_fitted': False, 'truth_in_prediction': False}
    inputs = result['inputs_sha256']
    def bind(path):
        path = Path(path); key = str(path.resolve()); digest = sha(path)
        if key in inputs and inputs[key] != digest: raise ValueError('INPUT_BYTES_CHANGED_DURING_EVALUATION')
        inputs[key] = digest; return path
    def read(path): return json.loads(bind(path).read_text())
    def table(path):
        with bind(path).open() as stream: return list(csv.DictReader(stream))
    try:
        for name in ('g2_source_effect_study.py', 'local_increment_validation.py',
                     'epoch_motion_reference.py', 'advisory_validation.py'):
            original = bind(Path(__file__).with_name(name))
            target = artifact(run, 'metadata/config/' + label + '/source/' + name)
            with target.open('xb') as stream: stream.write(original.read_bytes())
        primary = read(source/'metadata/run_manifest.json')
        if (primary['entrypoint'] != 'iap_sim' or primary['scenario'] != 'icra_dense_forest_four_fork_v2' or
                not primary['source']['git_worktree_clean']):
            raise ValueError('CLEAN_CANONICAL_SOURCE_REQUIRED')
        trial = primary['validation_trial']
        protocol = read(trial['protocol'])
        if inputs[str(Path(trial['protocol']).resolve())] != trial['protocol_sha256']:
            raise ValueError('PROTOCOL_HASH_MISMATCH')
        if protocol != json.loads(json.dumps(pilot_protocol())):
            raise ValueError('FROZEN_PILOT_EVALUATOR_PROTOCOL_MISMATCH')
        result.update(trial=trial, protocol=protocol, source_run=str(source), source_revision=primary['source']['git_commit'],
                      source_lifecycle=primary['lifecycle'])
        result['local_rows'] = [dict(pair_index=i//2, seed=trial['seed'], condition=trial['condition'],
                                    qualified=False, reason='NOT_EVALUATED')
                                for i in range(0, protocol['record_count'], 2)]
        recorder = source/'export/advisory/validation/recordings'
        authorities = list(recorder.glob('*_requests_manifest.json'))
        if len(authorities) != 1: raise ValueError('ONE_ORIGINAL_REQUEST_LEDGER_REQUIRED')
        authority = read(authorities[0]); requests = table(source/authority['requests_csv'])
        result['requests'] = requests
        if (inputs[str((source/authority['requests_csv']).resolve())] != authority['requests_csv_sha256'] or
                [r['request_id'] for r in requests] != authority['request_ids']):
            raise ValueError('REQUEST_LEDGER_HASH_OR_IDENTITY_MISMATCH')
        if ([int(r['request']) for r in requests] != list(range(len(requests))) or
                len(requests) > protocol['record_count']):
            raise ValueError('ORIGINAL_REQUEST_INDEX_LEDGER_INVALID')
        plan = authority['sampling_plan']
        if any(plan[key] != protocol[protocol_key] for key, protocol_key in (
                ('count', 'record_count'), ('interval_s', 'record_interval_s'), ('initial_delay_s', 'record_initial_delay_s'))):
            raise ValueError('ORIGINAL_SAMPLING_PLAN_MISMATCH')
        # Bind available half-pairs even when their partner failed. A vanished
        # artifact is an explicit request failure, never a removed denominator.
        result['request_bindings'] = {}
        for request in requests:
            binding = {'errors': []}
            result['request_bindings'][request['request_id']] = binding
            if request['available'] == 'True':
                payload = source/request['payload']
                for path in (payload, payload.with_name('record.json')):
                    try: bind(path)
                    except OSError as error: binding['errors'].append(f'{type(error).__name__}: {error}')
        result['source_process'] = read(source/'metadata/manifests/forest_process_result.json')
        capture_label = authorities[0].name.removesuffix('_requests_manifest.json')
        truth = PoseSeries(table(recorder/(capture_label+'_odometry.csv')), 'truth')
        native_path = bind(recorder/(capture_label+'_native_reports.jsonl.gz'))
        with gzip.open(native_path, 'rt') as stream: native = [json.loads(line) for line in stream]
        simulator = table(source/'export/simulation/gnss_injected_noise.csv')
        offsets = np.array([float(r['stamp'])-float(r['scenario_time_s']) for r in simulator])
        if not len(offsets) or not np.isfinite(offsets).all() or np.ptp(offsets) > 1e-5:
            raise ValueError('PRODUCER_ELAPSED_ORIGIN_UNAVAILABLE_OR_INCONSISTENT')
        origin_stamp = float(np.median(offsets)); result['producer_origin_stamp_s'] = origin_stamp
        result['global_native_original_count'] = 0
        original_global = table(source/'export/glio/gnss_global_state.csv')
        result['global_native_original_count'] = len(original_global)
        for item in select_global_updates(original_global):
            stamp = float(item['state_stamp_s'])
            row = dict(stamp_s=stamp, frame_id=int(item['frame_id']), update_sequence=int(item['update_sequence']),
                       elapsed_s=stamp-origin_stamp, qualified=False, native=item)
            result['global_rows'].append(row)
            try:
                true_pose, bracket = truth.at(stamp)
                value = fixed_global_error(item, true_pose[:3, 3])
                row.update(qualified=True, truth_position=true_pose[:3, 3].tolist(), truth_bracket=bracket)
                row['global'] = value
                row.update(horizontal_error_m=value['horizontal_error_m'], error_3d_m=value['error_3d_m'])
            except (ValueError, KeyError) as error: row['reason'] = str(error)
        global_rows = result['global_rows']
        recovery = next((i for i, r in enumerate(global_rows)
                         if float(r['native']['gnss_epoch_stamp_s'])-origin_stamp >= 60. and int(r['native']['pr_count']) > 0), None) if trial['condition'].startswith('GB') else None
        result['recovery'] = None
        if recovery is not None:
            result['recovery'] = dict(available=False, first_actual_update=global_rows[recovery],
                                      reason='FIRST_RECOVERY_OR_PREVIOUS_NATIVE_STATE_UNAVAILABLE')
            if recovery > 0 and all(r['qualified'] for r in global_rows[recovery-1:recovery+1]):
                result['recovery'] = {'available': True, **recovery_decomposition(global_rows[recovery-1], global_rows[recovery])}
        result['global_phases'] = {name: {field: phase_statistics(result['global_rows'], field, bounds)
                                for field in ('horizontal_error_m', 'error_3d_m')}
                                for name, bounds in protocol['phases_elapsed_s'].items()}
        result['actual_icp_support'] = table(source/'export/glio/iap_icp.csv')
        result['actual_gnss_factors'] = table(source/'export/glio/iap_gnss_factor_debug.csv')
        result['physical_gnss_observations'] = simulator
        replay_identity = binary_identity(binary); result['decoder_binary'] = replay_identity
        by_index = {int(r['request']): r for r in requests}
        for index in range(0, protocol['record_count'], 2):
            row = result['local_rows'][index//2]
            try:
                pair = [by_index[j] for j in (index, index+1)]
                row['request_ids'] = [r['request_id'] for r in pair]
                if any(r['available'] != 'True' for r in pair):
                    raise ValueError('ORIGINAL_PAIR_REQUEST_UNAVAILABLE')
                metadata = []
                for request in pair:
                    payload = bind(source/request['payload']); bind(payload.with_name('record.json'))
                    recorded = validate_record(payload, payload.with_name('record.json'))
                    for path, digest in recorded['producer_binary']['libraries_sha256'].items():
                        if path in replay_identity['libraries_sha256'] and replay_identity['libraries_sha256'][path] != digest:
                            raise ValueError('DECODER_PRODUCER_LIBRARY_MISMATCH')
                    decoded_label = label+'_'+str(request['request']).zfill(4)
                    backend(run, binary, ['replay_header', decoded_label, str(payload)], decoded_label)
                    decoded = read(run/'export/advisory/validation'/decoded_label/'input.json')
                    if any(decoded[k] != recorded[k] for k in ('frame_id', 'geometry_id', 'generation')):
                        raise ValueError('DECODED_SOURCE_IDENTITY_MISMATCH')
                    metadata.append(decoded)
                recorded_query_admission(metadata)
                meta = metadata[0]; start = meta['reference_time_s']; end = start+1.
                row.update(start_s=start, end_s=end, elapsed_s=start-origin_stamp, frame_id=meta['estimation_frame_id'],
                           position=meta['position'], original_input_metadata=metadata,
                           original_regions=REGIONS)
                transform = np.array(meta['coordinates']['T_map_world']).reshape(4, 4)
                reports = [r for r in native if r['request_id'] == pair[0]['request_id']]
                a, proof_a = native_endpoint(reports, start, transform)
                b, proof_b = native_endpoint(reports, end, transform)
                ta, truth_a = truth.at(start); tb, truth_b = truth.at(end)
                row.update(qualified=True, relative=relative_error(a, b, ta, tb),
                           native_endpoints=[proof_a, proof_b], truth_brackets=[truth_a, truth_b])
                row.pop('reason', None)
            except (ValueError, KeyError, OSError, subprocess.CalledProcessError, np.linalg.LinAlgError) as error:
                row['reason'] = f'{type(error).__name__}: {error}'
        for row in result['local_rows']:
            if row['qualified']: row['horizontal_error_m'] = row['relative']['horizontal_error_m']
        result['local_phases'] = {name: phase_statistics(result['local_rows'], 'horizontal_error_m', bounds)
                                 for name, bounds in protocol['phases_elapsed_s'].items()}
        result['status'] = 'PILOT_DIAGNOSTIC_COMPLETE; effects and fusion qualification not assigned'
    except Exception as error:
        result.update(status='EVALUATION_FAILED_PARTIAL_EVIDENCE_RETAINED', failure=f'{type(error).__name__}: {error}')
        raise
    finally:
        json_write(output, result)
        path = artifact(run, 'metadata/manifests/'+label+'.json')
        json_write(path, {'schema': 'iap_g2_source_effect_evaluation_manifest_v1',
                         'inputs_sha256': inputs, 'summary': str(output.relative_to(run)),
                         'summary_sha256': sha(output), 'status': result.get('status')})
        if owner: register_subordinate_manifest(run, path)
    return result


def prepare(run, *, owner):
    config = artifact(run, 'metadata/config/g2_source_effect_pilot/protocol.json')
    json_write(config, pilot_protocol())
    snapshots = {}
    for relative in ('scripts/dev_predictor/g2_source_effect_study.py',
                     'scripts/dev_predictor/local_increment_validation.py',
                     'docs/spec/source_effect_advisory_contract.md'):
        path = REPO / relative
        target = artifact(run, 'metadata/config/g2_source_effect_pilot/source/' + relative)
        with target.open('xb') as out: out.write(path.read_bytes())
        snapshots[relative] = {'sha256': sha(path), 'snapshot': str(target.relative_to(run))}
    trials = []
    for condition in pilot_protocol()['conditions']:
        path = config.with_name(condition + '.json')
        json_write(path, {'schema': 'iap_source_effect_trial_v1', 'scene': 'icra_dense_forest_four_fork_v2',
                         'map_seed': 41021, 'phase': 'pilot', 'seed': 9100, 'condition': condition,
                         'protocol': str(config), 'protocol_sha256': sha(config)})
        trials.append({'path': str(path), 'sha256': sha(path)})
    path = artifact(run, 'metadata/manifests/g2_source_effect_preregistration.json')
    json_write(path, {'schema': 'iap_g2_source_effect_preregistration_v1', 'source': source_identity(),
                     'protocol': str(config.relative_to(run)), 'protocol_sha256': sha(config),
                     'trials': trials, 'source_snapshots': snapshots, 'live_started': False,
                     'model_qualified': False})
    if owner: register_subordinate_manifest(run, path)
    return {'run': str(run), 'protocol': str(config), 'trials': trials}


def cell_diagnostics(result):
    """Descriptive run summaries; native updates/windows are never independent N."""
    origin = result['producer_origin_stamp_s']
    phases = {}
    for phase, bounds in result['protocol']['phases_elapsed_s'].items():
        lo, hi = bounds
        local = [r for r in result['local_rows'] if r.get('qualified') and lo <= r['elapsed_s'] <= hi]
        global_rows = [r for r in result['global_rows'] if lo <= r['elapsed_s'] <= hi]
        icp = [r for r in result['actual_icp_support'] if lo <= float(r['stamp'])-origin <= hi]
        physical = [r for r in result['physical_gnss_observations'] if lo <= float(r['scenario_time_s']) <= hi]
        factors = [r for r in result['actual_gnss_factors'] if lo <= float(r['stamp'])-origin <= hi]
        def median(rows, key):
            values = [float(r[key]) for r in rows if r.get(key) not in (None, '')]
            values = [v for v in values if np.isfinite(v)]
            return float(np.median(values)) if values else None
        phases[phase] = {
            'native_global_updates': len(global_rows),
            'native_pr_factors': sum(int(r['native']['pr_count']) for r in global_rows),
            'native_dop_factors': sum(int(r['native']['dop_count']) for r in global_rows),
            'native_updates_with_pr': sum(int(r['native']['pr_count']) > 0 for r in global_rows),
            'physical_observations': len(physical),
            'physical_emitted': sum(r['observation_emitted'] == '1' for r in physical),
            'physical_visibility': dict(Counter(r['visibility_state'] for r in physical)),
            'raw_pr_sigma_m': median(physical, 'raw_pr_sigma_m'),
            'effective_pr_sigma_m': median([r for r in factors if r['factor_type'] == 'PR'], 'sigma'),
            'factor_debug_note': 'postopt evaluations can repeat; totals come from native injection CSV',
            'icp_rows': len(icp),
            'icp_medians': {k: median(icp, k) for k in ('rmse', 'inlier_fraction', 'inlier_count',
                'vgicp_factor_count', 'condition_number', 'native_min_singular_value')},
            'weak_direction_note': 'native Pose3 rotation then translation; mixed units/sign; raw CSV retained',
            'qualified_local_windows': len(local),
            'rotation_error_rad': median([r['relative'] for r in local], 'rotation_error_rad'),
            'translation_error_m': median([r['relative'] for r in local], 'translation_error_m'),
            'truth_motion_distance_m': median([r['relative'] for r in local], 'truth_motion_distance_m'),
        }
    positions = [r['truth_position'] for r in result['global_rows'] if r.get('qualified')]
    represented = []
    for i, (lo, hi) in enumerate(REGIONS):
        rows = [r for r in result['local_rows'] if r.get('qualified') and lo <= r['position'][0] and
                (r['position'][0] < hi or (i == len(REGIONS)-1 and r['position'][0] == hi))]
        represented.append({'bin': [lo, hi], 'qualified_windows': len(rows)})
    return {'phases': phases, 'local_requested': len(result['local_rows']),
            'local_qualified': sum(bool(r.get('qualified')) for r in result['local_rows']),
            'local_reasons': dict(Counter(r.get('reason', 'QUALIFIED') for r in result['local_rows'])),
            'regions': represented, 'truth_position_min': np.min(positions, axis=0).tolist() if positions else None,
            'truth_position_max': np.max(positions, axis=0).tolist() if positions else None,
            'recovery': result['recovery'], 'independent_groups': 1}


def paired_contrasts(cells):
    """Bad-minus-good descriptive contrasts in both strata; no automatic PASS."""
    contrasts = []
    for source, pairs in (('L', [('GG_LG', 'GG_LB'), ('GB_LG', 'GB_LB')]),
                          ('G', [('GG_LG', 'GB_LG'), ('GG_LB', 'GB_LB')])):
        for good, bad in pairs:
            for phase in pilot_protocol()['phases_elapsed_s']:
                for field in ('local_XY', 'global_XY', 'global_3D'):
                    def stats(cell):
                        return (cell['local_phases'][phase] if field == 'local_XY' else
                                cell['global_phases'][phase]['horizontal_error_m' if field == 'global_XY' else 'error_3d_m'])
                    a, b = stats(cells[good]), stats(cells[bad])
                    floor = pilot_protocol()['detection_floors_m']['local' if field == 'local_XY' else 'global']
                    delta = b['median']-a['median'] if a['median'] is not None and b['median'] is not None else None
                    contrasts.append({'source': source, 'good': good, 'bad': bad, 'phase': phase, 'field': field,
                        'good_count': a['count'], 'bad_count': b['count'], 'bad_minus_good_m': delta,
                        'growth_bad_minus_good_m': b['last_minus_first_quartile_median']-a['last_minus_first_quartile_median']
                            if all(s['last_minus_first_quartile_median'] is not None for s in (a, b)) else None,
                        'phase_minimum_met': a['phase_minimum_met'] and b['phase_minimum_met'],
                        'above_detection_floor': abs(delta) > floor if delta is not None else False,
                        'detection_floor_m': floor, 'independent_groups': 1, 'qualified_effect': False})
    return contrasts


def original_a0_diagnostic(point, matrices, metadata):
    from horizontal_probe_advisory import project_information
    # Physical/stale/input refusals legitimately have no queried matrix. Their
    # status is evidence, not a zero-information finite score or a parser error.
    if point['status'] != 'VALID':
        return project_information(None, metadata['K_H_adv'], metadata['K_V_adv'], input_status=point['status'])
    if matrices.get('joint_pose_information') is None:
        raise ValueError('VALID_QUERY_WITHOUT_JOINT_INFORMATION')
    information = np.array(matrices['joint_pose_information']['row_major']).reshape(6, 6)
    zero = project_information(information, metadata['K_H_adv'], metadata['K_V_adv'], zero_probe=True)
    if not np.allclose([zero['Q_H'], zero['Q_V']],
                      [float(point['module_hpl']), float(point['module_vpl'])], rtol=1e-8, atol=1e-9):
        raise ValueError('ORIGINAL_POSITION_OR_ZERO_OFFSET_BINDING_MISMATCH')
    return project_information(information, metadata['K_H_adv'], metadata['K_V_adv'])


def compare(run, evaluations, selection_path, binary, label, *, owner):
    """Four-cell developer evidence and fixed receiver source responses only."""
    safe_label(label)
    output = artifact(run, 'export/analysis/'+label+'/summary.json')
    if output.exists(): raise ValueError('comparison evidence already exists')
    result = {'schema': 'iap_g2_four_cell_diagnostic_v1', 'inputs_sha256': {}, 'cells': {},
              'responses': [], 'model_qualified': False, 'source_effects_qualified': False,
              'formal_runs_started': False, 'independent_groups': 1, 'evaluation_source_snapshots': {}}
    def bind(path):
        path = Path(path).resolve(); digest = sha(path); key = str(path)
        if key in result['inputs_sha256'] and result['inputs_sha256'][key] != digest:
            raise ValueError('INPUT_BYTES_CHANGED_DURING_COMPARISON')
        result['inputs_sha256'][key] = digest
        return path
    def read(path): return json.loads(bind(path).read_text())
    try:
        for name in ('g2_source_effect_study.py', 'horizontal_probe_advisory.py', 'advisory_validation.py'):
            source = bind(Path(__file__).with_name(name))
            with artifact(run, 'metadata/config/'+label+'/source/'+name).open('xb') as stream:
                stream.write(source.read_bytes())
        selection = read(selection_path)
        if selection['request_indices'] != [6, 24, 56] or selection['conditions'] != list(pilot_protocol()['conditions']):
            raise ValueError('FIXED_RESPONSE_SELECTION_REQUIRED')
        result['response_selection'] = selection
        cells = {}
        for path in evaluations:
            cell = read(path); condition = cell['trial']['condition']
            if condition in cells or cell['trial']['seed'] != 9100 or cell['protocol'] != json.loads(json.dumps(pilot_protocol())):
                raise ValueError('ONE_FROZEN_PILOT_PER_CONDITION_REQUIRED')
            if not cell['status'].startswith('PILOT_DIAGNOSTIC_COMPLETE') or cell['source_lifecycle'] != 'completed':
                raise ValueError('COMPLETED_PILOT_REQUIRED; failed source/evaluation remains evidence')
            # Evaluator code is historical evidence: its owner saved these exact
            # bytes before evaluation. Workspace tools can advance independently.
            # Physical payloads/data still bind their original owner paths.
            evaluator_names = ('g2_source_effect_study.py', 'local_increment_validation.py',
                               'epoch_motion_reference.py', 'advisory_validation.py')
            for original, digest in cell['inputs_sha256'].items():
                original_path = Path(original).resolve()
                if original_path.parent == Path(__file__).resolve().parent and original_path.name in evaluator_names:
                    snapshot = Path(path).resolve().parents[3]/'metadata/config'/Path(path).parent.name/'source'/original_path.name
                    result['evaluation_source_snapshots'][condition+':'+str(original_path)] = str(snapshot)
                    if sha(bind(snapshot)) != digest: raise ValueError('EVALUATOR_SNAPSHOT_BYTES_CHANGED')
                elif sha(bind(original_path)) != digest: raise ValueError('EVALUATED_SOURCE_BYTES_CHANGED')
            process = cell['source_process']
            if process['required_input_failure'] or any(v != 0 for v in process['exit_codes'].values()):
                raise ValueError('PILOT_PROCESS_HEALTH_FAILED')
            cells[condition] = cell
            result['cells'][condition] = {'evaluation': str(path), 'source_run': cell['source_run'],
                'source_revision': cell['source_revision'], 'local_phases': cell['local_phases'],
                'global_phases': cell['global_phases'], **cell_diagnostics(cell)}
        if set(cells) != set(pilot_protocol()['conditions']): raise ValueError('ALL_FOUR_CONDITIONS_REQUIRED')
        result['contrasts'] = paired_contrasts(cells)
        result['decoder_binary'] = binary_identity(binary)
        for condition in selection['conditions']:
            cell = cells[condition]; requests = {int(r['request']): r for r in cell['requests']}
            for index in selection['request_indices']:
                response = {'condition': condition, 'request': index, 'available': False, 'modes': {}}
                result['responses'].append(response)
                try:
                    request = requests[index]
                    if request['available'] != 'True': raise ValueError('ORIGINAL_REQUEST_UNAVAILABLE')
                    payload = bind(Path(cell['source_run'])/request['payload'])
                    record = validate_record(payload, bind(payload.with_name('record.json')))
                    for library, digest in record['producer_binary']['libraries_sha256'].items():
                        if library in result['decoder_binary']['libraries_sha256'] and result['decoder_binary']['libraries_sha256'][library] != digest:
                            raise ValueError('DECODER_PRODUCER_LIBRARY_MISMATCH')
                    name = f'{label}_{condition}_{index:04}'
                    header_args = ['replay_header', name, str(payload)]
                    response['header_command'] = [str(binary), *header_args]
                    try:
                        backend(run, binary, header_args, name)
                        response['header_exit_code'] = 0
                    except subprocess.CalledProcessError as error:
                        response['header_exit_code'] = error.returncode
                        raise
                    finally:
                        for log in (run/'runtime/ros').glob('advisory_'+name+'*.log'): bind(log)
                        header_directory = run/'export/advisory/validation'/name
                        response['header_directory'] = str(header_directory.relative_to(run))
                        if header_directory.exists():
                            for item in header_directory.iterdir():
                                if item.is_file(): bind(item)
                    meta = read(run/'export/advisory/validation'/name/'input.json')
                    if any(meta[k] != record[k] for k in ('frame_id', 'geometry_id', 'generation')):
                        raise ValueError('DECODED_SOURCE_IDENTITY_MISMATCH')
                    response.update(available=True, original_motion_quality=meta['current_motion_quality'],
                        original_reference_elapsed_s=meta['reference_time_s']-cell['producer_origin_stamp_s'],
                        epoch_source_identity=meta['epoch_source_identity'], position=meta['position'])
                    points = artifact(run, 'metadata/config/'+label+'/'+name+'_receiver.txt')
                    with points.open('x') as stream: stream.write(' '.join(format(v, '.17g') for v in meta['position'])+'\n')
                    bind(points)
                    for mode in ('gnss', 'lidar', 'fusion'):
                        child = name+'_'+mode
                        arguments = ['replay_points', child, str(payload), '120', str(points), mode]
                        directory = run/'export/advisory/validation'/child
                        response['modes'][mode] = {'command': [str(binary), *arguments],
                            'retained_directory': str(directory.relative_to(run))}
                        try:
                            backend(run, binary, arguments, child)
                            response['modes'][mode]['exit_code'] = 0
                        except subprocess.CalledProcessError as error:
                            response['modes'][mode]['exit_code'] = error.returncode
                            raise
                        finally:
                            for log in (run/'runtime/ros').glob('advisory_'+child+'*.log'): bind(log)
                            if directory.exists():
                                for item in directory.iterdir():
                                    if item.is_file(): bind(item)
                        with (directory/'points.csv').open() as stream: point = next(csv.DictReader(stream))
                        matrices = json.loads((directory/'matrices.jsonl').read_text())
                        score = original_a0_diagnostic(point, matrices, meta)
                        response['modes'][mode].update({'point': point, 'original_A0_diagnostic': score,
                            'original_query_qualified': meta['current_motion_quality'] == 1,
                            'exit_code': 0, 'retained_directory': str(directory.relative_to(run))})
                except (ValueError, KeyError, OSError, subprocess.CalledProcessError) as error:
                    response['failure'] = f'{type(error).__name__}: {error}'
        result['status'] = 'FOUR_CELL_DEVELOPMENT_DIAGNOSTIC; no independent qualification'
    except Exception as error:
        result.update(status='COMPARISON_FAILED_PARTIAL_EVIDENCE_RETAINED', failure=f'{type(error).__name__}: {error}')
        raise
    finally:
        json_write(output, result)
        manifest = artifact(run, 'metadata/manifests/'+label+'.json')
        json_write(manifest, {'summary': str(output.relative_to(run)), 'summary_sha256': sha(output),
            'inputs_sha256': result['inputs_sha256'], 'status': result.get('status')})
        if owner: register_subordinate_manifest(run, manifest)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('prepare', 'evaluate', 'compare'))
    parser.add_argument('--evaluation', type=Path, action='append', default=[])
    parser.add_argument('--response-selection', type=Path)
    parser.add_argument('--source-run', type=Path)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--label', default='g2_source_effect_evaluation')
    args = parser.parse_args()
    if args.mode == 'evaluate' and (not args.source_run or not args.binary):
        parser.error('evaluate requires source-run and the committed decoder binary')
    if args.mode == 'compare' and (len(args.evaluation) != 4 or not args.response_selection or not args.binary):
        parser.error('compare requires four evaluations, response-selection and decoder binary')
    inherited = os.environ.get('IAP_RUN_DIR')
    owner = not inherited
    run = resolve_run_directory(entrypoint='g2_source_effect_preregistration' if args.mode == 'prepare' else 'g2_source_effect_evaluation', scenario='icra_dense_forest_four_fork_v2') if owner else adopt_run_directory(inherited)
    try:
        if args.mode == 'prepare': print(json.dumps(prepare(run, owner=owner), indent=2))
        else:
            result = (compare(run, args.evaluation, args.response_selection, args.binary.resolve(), args.label, owner=owner)
                      if args.mode == 'compare' else
                      evaluate(run, args.source_run.resolve(), args.label, args.binary.resolve(), owner=owner))
            print(json.dumps({'run': str(run), 'status': result['status']}))
    except Exception:
        if owner: finalize_run(run, lifecycle='failed')
        raise
    else:
        if owner: finalize_run(run, lifecycle='completed')


if __name__ == '__main__': main()
