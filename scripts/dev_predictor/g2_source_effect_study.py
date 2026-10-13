"""Bounded G2 source-effect protocol and fixed-reference offline means.

No truth enters an online predictor, no old probe is refitted, and preparation
or evaluator tests grant no source-effect/model qualification.
"""
import argparse
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('prepare', 'evaluate'))
    parser.add_argument('--source-run', type=Path)
    parser.add_argument('--binary', type=Path)
    parser.add_argument('--label', default='g2_source_effect_evaluation')
    args = parser.parse_args()
    if args.mode == 'evaluate' and (not args.source_run or not args.binary):
        parser.error('evaluate requires source-run and the committed decoder binary')
    inherited = os.environ.get('IAP_RUN_DIR')
    owner = not inherited
    run = resolve_run_directory(entrypoint='g2_source_effect_preregistration' if args.mode == 'prepare' else 'g2_source_effect_evaluation', scenario='icra_dense_forest_four_fork_v2') if owner else adopt_run_directory(inherited)
    try:
        if args.mode == 'prepare': print(json.dumps(prepare(run, owner=owner), indent=2))
        else:
            result = evaluate(run, args.source_run.resolve(), args.label, args.binary.resolve(), owner=owner)
            print(json.dumps({'run': str(run), 'status': result['status']}))
    except Exception:
        if owner: finalize_run(run, lifecycle='failed')
        raise
    else:
        if owner: finalize_run(run, lifecycle='completed')


if __name__ == '__main__': main()
