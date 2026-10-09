#!/usr/bin/env python3
"""Stratify recorded simulator errors; temporal samples are not independent trials."""
import argparse
import csv
import json
import os
from collections import defaultdict
from pathlib import Path

import numpy as np
from advisory_validation import safe_label, sha, source_identity
from run_directory import adopt_run_directory, write_subordinate_manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args(); safe_label(args.label)
    run = adopt_run_directory(os.environ['IAP_RUN_DIR'])
    source = args.source_run.resolve()
    path = source/'export/simulation/gnss_injected_noise.csv'
    primary = source/'metadata/run_manifest.json'
    identity = json.loads(primary.read_text())
    if not identity['source']['git_worktree_clean']:
        raise ValueError('source run is not bound to a clean commit')
    with path.open() as stream:
        records = [r for r in csv.DictReader(stream) if r['observation_emitted'] == '1']
    groups = defaultdict(list); satellites = defaultdict(list)
    for r in records:
        el = float(r['el_deg'])
        band = '10–20°' if el < 20 else '20–40°' if el < 40 else '40–90°'
        groups[(r['visibility_state'], band, float(r['raw_pr_sigma_m']))].append(r)
        satellites[(r['sat_id'], r['visibility_state'])].append(r)
    def summarize(rows):
        white = np.array([float(r['injected_pr_white_noise_m']) for r in rows])
        bias = np.array([float(r['psr_extra_bias_m_nlos_multipath_fault']) for r in rows])
        total = np.array([float(r['injected_pr_total_error_m']) for r in rows])
        # Exact producer decomposition, not a fitted post-optimization residual.
        if not np.allclose(total, white+bias, atol=1e-9, rtol=0):
            raise ValueError('recorded injection decomposition mismatch')
        floor = 5/np.sin(np.maximum(np.deg2rad([float(r['el_deg']) for r in rows]), np.deg2rad(10)))**2
        return dict(samples=len(rows), white_mean_m=float(white.mean()),
                    white_std_m=float(white.std(ddof=1)) if len(rows)>1 else None,
                    bias_mean_m=float(bias.mean()), bias_std_m=float(bias.std()),
                    total_rmse_m=float(np.sqrt(np.mean(total**2))),
                    policy_floor_m=np.quantile(floor, [0, .5, 1]).tolist())
    report = dict(source_run=str(source), source_commit=identity['source'],
                  emitted_denominator=len(records), independent_runs=1, calibration_qualified=False,
                  groups=[dict(visibility=k[0], elevation_band=k[1], declared_sigma_m=k[2], **summarize(v))
                          for k, v in sorted(groups.items())],
                  satellites=[dict(sat_id=k[0], visibility=k[1], **summarize(v))
                              for k, v in sorted(satellites.items())],
                  conclusion='white-noise scale and injected persistent bias are separate; 5/sin²(elevation) is an uncalibrated policy floor, not measured sigma')
    out = run/'export/analysis'/args.label; out.mkdir(parents=True, exist_ok=False)
    data = out/'noise_strata.json'; data.write_text(json.dumps(report, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    for visibility in sorted({r['visibility_state'] for r in records}):
        rows = [r for r in records if r['visibility_state'] == visibility][::30]
        elevations = [float(r['el_deg']) for r in rows]
        ax[0].scatter(elevations, [float(r['injected_pr_total_error_m']) for r in rows], s=5, alpha=.35, label=visibility)
        ax[1].scatter(elevations, [float(r['psr_extra_bias_m_nlos_multipath_fault']) for r in rows], s=5, alpha=.35, label=visibility)
    e = np.linspace(10, 90, 100); ax[0].plot(e, 5/np.sin(np.deg2rad(e))**2, color='black', label='existing floor (policy)')
    for a in ax: a.set_xlabel('Elevation (deg)'); a.set_ylabel('Metres'); a.legend(fontsize=8)
    ax[0].set_title('Injected error and policy floor'); ax[1].set_title('Injected bias, separate from white noise')
    fig.suptitle('Single recorded run — temporal samples are correlated'); fig.tight_layout()
    plot = out/'noise_strata.png'; fig.savefig(plot, dpi=150); plt.close(fig)
    write_subordinate_manifest(run, args.label, dict(source=source_identity(),
        command=['diagnose_injected_noise.py', '--source-run', str(source), '--label', args.label],
        input_sha256={str(p): sha(p) for p in [path, primary]},
        artifacts_sha256={str(p.relative_to(run)): sha(p) for p in [data, plot]}, calibration_qualified=False))
    print(json.dumps(report['groups']))


if __name__ == '__main__':
    main()
