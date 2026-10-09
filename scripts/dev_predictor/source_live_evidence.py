#!/usr/bin/env python3
"""Bind simulator draws, final noise and actual match identities without calibration."""
import argparse
from collections import Counter, defaultdict
import csv
import json
import os
from pathlib import Path

import numpy as np
from advisory_validation import sha, source_identity
from gpu_match_evidence_audit import audit_sample
from run_directory import adopt_run_directory, write_subordinate_manifest


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def quantiles(values):
    return np.quantile(values, [0, .5, .95, 1]).tolist() if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--pairs-label', required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    from advisory_validation import safe_label
    safe_label(args.label); safe_label(args.pairs_label)
    run = adopt_run_directory(os.environ['IAP_RUN_DIR'])
    source = args.source_run.resolve()
    noise_path = source/'export/simulation/gnss_injected_noise.csv'
    factor_path = source/'export/glio/iap_gnss_factor_debug.csv'
    gpu_path = source/'export/glio/gpu_match_residuals.jsonl'
    noise = read_csv(noise_path)
    emitted = [r for r in noise if r['observation_emitted'] == '1']
    noise_index = {(round(float(r['stamp']), 6), int(r['sat_id'])): r for r in emitted}
    if len(noise_index) != len(emitted):
        raise ValueError('duplicate emitted GNSS acquisition identity')
    factors = read_csv(factor_path)
    factor_index = {(round(float(r['stamp']), 6), int(r['sat_id'])): r for r in factors
                    if r['factor_type'] == 'PR'}
    doppler_index = {(round(float(r['stamp']), 6), int(r['sat_id'])): r for r in factors
                     if r['factor_type'] == 'DOP'}
    directories = sorted((run/'export/advisory/validation').glob(args.pairs_label+'_*_current'))
    if not directories:
        raise ValueError('no complete frozen source chain inputs')
    epochs = {round(json.loads((p/'input.json').read_text())['gnss_stamp'], 6) for p in directories}
    raw_messages = {}
    event_paths = sorted((source/'export/planner/advisory_validation').glob('*_events.jsonl'))
    for event_path in event_paths:
        with event_path.open() as stream:
            for line in stream:
                event = json.loads(line)
                if event['topic'] != '/ublox_driver/range_meas':
                    continue
                for observation in event['payload']['meas']:
                    t = observation['time']
                    # Simulator 2022 epoch: GPST is UTC + 18 seconds. Preserve
                    # both clocks; this is the existing producer convention.
                    stamp = round(315964800 + t['week']*604800 + t['tow'] - 18., 6)
                    if stamp in epochs:
                        raw_messages[(stamp, observation['sat'])] = observation
    chain = []
    evidence_paths = []
    for directory in directories:
        meta_path = directory/'input.json'; row_path = directory/'gnss_information_rows.jsonl'
        meta = json.loads(meta_path.read_text())
        info = json.loads(row_path.read_text().splitlines()[0])
        effective = {r['sat_id']: r['final_sigma_m'] for r in info['rows']}
        evidence_paths.extend([meta_path, row_path])
        for sat in meta['gnss_satellites']:
            raw = noise_index.get((round(meta['gnss_stamp'], 6), sat['id']))
            message = raw_messages.get((round(meta['gnss_stamp'], 6), sat['id']))
            final = factor_index.get((round(meta['postopt_evidence']['state_stamp'], 6), sat['id']))
            doppler = doppler_index.get((round(meta['postopt_evidence']['state_stamp'], 6), sat['id']))
            if raw is None or final is None or message is None or doppler is None:
                raise ValueError('same-epoch raw/final GNSS chain unavailable')
            if (abs(float(raw['raw_pr_m'])-message['psr'][0]) > 1e-6 or
                    float(raw['raw_pr_sigma_m']) != message['psr_std'][0]):
                raise ValueError('simulator/receiver raw acquisition mismatch')
            fgo = sat['nominal_sigma_m']
            # GnssExtension applies satellite-clock pre-correction at the
            # transmission epoch. Raw and factor pseudoranges are different
            # objects; their difference is not receiver measurement error.
            if abs(float(final['sigma'])-fgo) > 1.1e-4:
                raise ValueError('frozen final sigma and factor debug mismatch')
            if abs(float(final['residual'])-sat['pr_residual_m']) > 1.1e-4:
                raise ValueError('frozen residual and factor debug mismatch')
            doppler_declared_mps = message['dopp_std'][0]*299792458./message['freqs'][0]
            if abs(doppler_declared_mps-float(raw['raw_dop_sigma_mps'])) > 1e-10:
                raise ValueError('Doppler Hz/mps sigma conversion mismatch')
            floor = 5./np.sin(max(sat['elevation'], np.deg2rad(10.)))**2
            chain.append(dict(request=directory.name, acquisition_epoch_s=meta['gnss_stamp'],
                original_state_s=meta['postopt_evidence']['state_stamp'], sat_id=sat['id'],
                constellation=sat['constellation'], simulator_visibility=raw['visibility_state'],
                raw_gps_week=message['time']['week'], raw_gps_tow_s=message['time']['tow'],
                raw_sigma_m=float(raw['raw_pr_sigma_m']), injected_error_m=float(raw['injected_pr_total_error_m']),
                raw_pseudorange_m=float(raw['raw_pr_m']), corrected_pseudorange_m=sat['pseudorange_m'],
                satellite_clock_precorrection_m=sat['pseudorange_m']-float(raw['raw_pr_m']),
                injected_white_error_m=float(raw['injected_pr_white_noise_m']),
                final_fgo_sigma_m=fgo, advisory_sigma_m=effective.get(sat['id']),
                raw_dop_sigma_mps=float(raw['raw_dop_sigma_mps']),
                receiver_dop_sigma_hz=message['dopp_std'][0], final_fgo_dop_sigma_mps=float(doppler['sigma']),
                injected_dop_white_error_mps=float(raw['injected_dop_white_noise_mps']),
                excluded=sat['excluded'], visible=sat['id'] in info['visible_sat_ids'],
                unknown=sat['id'] not in info['visible_sat_ids'] and sat['id'] not in info['excluded_sat_ids'],
                elevation_floor_reference_m=float(floor), postfit_residual_m=sat['pr_residual_m'],
                reason='max(raw, configured elevation/canopy floor) plus shared IMU-transfer envelope; Advisory max with candidate canopy floor'))
    draws = {}
    for sigma in sorted({float(r['raw_pr_sigma_m']) for r in emitted}):
        group = [r for r in emitted if float(r['raw_pr_sigma_m']) == sigma]
        white = np.array([float(r['injected_pr_white_noise_m']) for r in group])
        total = np.array([float(r['injected_pr_total_error_m']) for r in group])
        draws[str(sigma)] = dict(n=len(group), white_mean_m=float(white.mean()),
            white_std_m=float(white.std(ddof=1)), total_rmse_m=float(np.sqrt(np.mean(total**2))),
            total_error_quantiles_m=quantiles(total.tolist()))
    rejects = Counter(); first_reject = {}; total = good = requests = available = 0
    source_matches = Counter(); target_matches = Counter(); levels = defaultdict(dict)
    max_skew = 0.; frames = set()
    with gpu_path.open() as stream:
        for line in stream:
            p = json.loads(line); requests += 1
            if not p['available']:
                continue
            available += 1; frames.add(p['source_frame_id'])
            transform = np.array(p['linearization_transform_row_major']).reshape(4,4)
            target = np.array(p['T_world_target_row_major']).reshape(4,4)
            for s in p['samples']:
                total += 1
                covariance = np.array(s['covariance_m2_row_major']).reshape(3,3)
                skew = float(np.max(np.abs(covariance-covariance.T)))
                max_skew = max(max_skew, skew)
                source_matches[(p['source_frame_id'], s['source_index'])] += 1
                target_matches[(p['target_frame_id'], p['level_id'], s['target_index'])] += 1
                try:
                    e, _, _, _, _ = audit_sample(s, transform)
                except (ValueError, np.linalg.LinAlgError) as exc:
                    reason = str(exc); rejects[reason] += 1
                    first_reject.setdefault(reason, dict(request=p['request_id'], source_frame=p['source_frame_id'],
                        target_frame=p['target_frame_id'], level=p['level_id'], source_index=s['source_index'],
                        covariance=covariance.tolist(), covariance_skew_m2=skew))
                    continue
                good += 1
                levels[(p['source_frame_id'], p['target_frame_id'], s['source_index'])][p['level_id']] = target[:3,:3]@e
    paired = np.array([(v[0],v[1]) for v in levels.values() if 0 in v and 1 in v])
    correlation = [float(np.corrcoef(paired[:,0,i],paired[:,1,i])[0,1]) for i in range(3)] if len(paired)>2 else None
    report = dict(identity='REAL_SOURCE_CHAIN_DIAGNOSTIC', source_run=str(source),
        gnss_satellite_chain=chain, raw_injection_by_declared_sigma=draws,
        lidar=dict(requests=requests, available=available, source_frames=len(frames), samples=total,
            strict_algebra_pass_samples=good, rejected_samples=dict(rejects), first_reject=first_reject,
            covariance_max_skew_m2=max_skew, repeated_source_matches=quantiles(list(source_matches.values())),
            repeated_target_matches=quantiles(list(target_matches.values())),
            paired_levels=len(paired), cross_level_component_correlation=correlation),
        noise_calibrated=False, map_uncertainty_calibrated=False,
        scope='injection scale and deterministic identities only; postfit residuals and correlated samples do not establish unbiased independent measurement noise')
    out = run/'export/analysis'/args.label; out.mkdir(parents=True, exist_ok=False)
    path = out/'source_evidence.json'; path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    write_subordinate_manifest(run, args.label, dict(source=source_identity(),
        command=['source_live_evidence.py', '--source-run', str(source), '--pairs-label', args.pairs_label, '--label', args.label],
        input_sha256={str(p):sha(p) for p in [noise_path,factor_path,gpu_path,*event_paths,*evidence_paths]},
        artifacts_sha256={str(path.relative_to(run)):sha(path)}, qualified_noise=False))
    print(json.dumps({k:v for k,v in report.items() if k!='gnss_satellite_chain'}))


if __name__ == '__main__':
    main()
