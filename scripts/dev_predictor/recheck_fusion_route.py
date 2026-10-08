#!/usr/bin/env python3
"""Cost archived ID5 guides/curve and source maps with unchanged physical policy."""
import argparse
import json
import os
from pathlib import Path
import subprocess

import numpy as np
from check_fusion_fixed_route import native_input, verify_planning_capture
from advisory_validation import sha, binary_identity, source_identity, safe_label
from run_directory import adopt_run_directory, write_subordinate_manifest


def sample_bspline(knots, controls, degree, times):
    """Independent de Boor evaluation of captured knots, no retiming/refitting."""
    knots, controls = np.asarray(knots), np.asarray(controls)
    if len(knots) != len(controls)+degree+1:
        raise ValueError('captured spline dimension mismatch')
    out = []
    for t in times:
        if not knots[degree] <= t <= knots[-degree-1]:
            raise ValueError('captured spline time outside domain')
        span = min(len(controls)-1, max(degree, np.searchsorted(knots, t, side='right')-1))
        values = controls[span-degree:span+1].copy()
        for r in range(1, degree+1):
            for i in range(degree, r-1, -1):
                lower = i+span-degree
                denominator = knots[i+span-r+1]-knots[lower]
                alpha = (t-knots[lower])/denominator if denominator else 0.
                values[i] = (1-alpha)*values[i-1]+alpha*values[i]
        out.append(values[degree])
    return np.array(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--left-guide', type=Path, required=True)
    parser.add_argument('--right-guide', type=Path, required=True)
    parser.add_argument('--search-binary', type=Path, required=True)
    parser.add_argument('--predictor-binary', type=Path, required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args();safe_label(args.label)
    run = adopt_run_directory(os.environ['IAP_RUN_DIR'])
    snapshot = args.snapshot.resolve();payload = snapshot/'planning_input.bin'
    meta = json.loads((snapshot/'snapshot.json').read_text())
    identity = verify_planning_capture(snapshot, payload, meta)
    params = json.loads(args.parameters.read_text())
    old = {name: json.loads(path.read_text()) for name, path in
           [('left', args.left_guide), ('right', args.right_guide)]}
    if not all(g['guide_found'] and g['scope'] == 'REAL_FROZEN' for g in old.values()):
        raise ValueError('complete original local guides required')
    stages = [c for c in meta['curve_stages'] if c['stage'] == 'release_corridor_checked']
    if len(stages) != 1 or stages[0]['physical_check_reason'] != 'OK':
        raise ValueError('unique captured released curve required')
    curve = meta['actual_curve']
    if not all(np.array_equal(curve[k], stages[0][k]) for k in ('control_points_m', 'knots_s')):
        raise ValueError('released stage and actual captured curve mismatch')
    degree = curve['degree'];knots = np.array(curve['knots_s'])
    t = np.linspace(knots[degree], knots[-degree-1],
                    int(np.ceil((knots[-degree-1]-knots[degree])/.01))+1)
    routes = {name: value['path_m'] for name, value in old.items()}
    routes['captured_curve'] = sample_bspline(knots, curve['control_points_m'], degree, t).tolist()
    for name, step in [('captured_curve_coarse', .02), ('captured_curve_fine', .005)]:
        times = np.linspace(knots[degree], knots[-degree-1],
                            int(np.ceil((knots[-degree-1]-knots[degree])/step))+1)
        routes[name] = sample_bspline(knots, curve['control_points_m'], degree, times).tolist()
    out = run/'export/analysis'/args.label;out.mkdir(parents=True, exist_ok=False)
    (out/'captured_routes.json').write_text(json.dumps({'guides': old, 'curve': curve,
        'sampled_curve': routes['captured_curve'], 'curve_time_s': t.tolist()}, indent=2))
    commands = [];results = {}
    for name, route in routes.items():
        request = out/(name+'.txt');request.write_text(native_input(meta, params, route))
        result = out/(name+'.json')
        command = [str(args.search_binary.resolve()), str(snapshot/'cells.bin'),
                   '--route-cost', str(payload), str(result)]
        commands.append(command)
        with request.open() as stream, (out/(name+'.log')).open('x') as log:
            code = subprocess.run(command, stdin=stream, stdout=log, stderr=subprocess.STDOUT).returncode
        if code:
            raise RuntimeError('route-cost replay failed: '+str(code))
        results[name] = json.loads(result.read_text())
    # A local slice of the same map, not a scene risk label or new risk layer.
    points = [[float(x), float(y), 1.65] for x in np.arange(-14.25, -8.95, .2)
              for y in np.arange(-4.05, 4.06, .2)]
    points += routes['left']+routes['right']+routes['captured_curve'][::10]
    point_file = out/'source_points.txt'
    np.savetxt(point_file, np.array(points), fmt='%.17g')
    for source in ('gnss', 'lidar', 'fusion'):
        command = [str(args.predictor_binary.resolve()), 'replay_points', args.label+'_'+source,
                   str(payload), '120', str(point_file), source]
        commands.append(command)
        with (out/(source+'_points.log')).open('x') as log:
            code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT).returncode
        if code:
            raise RuntimeError('explicit-point source replay failed: '+str(code))
    proof = {'identity': 'SAME_ID5_JOINT_MODEL_ROUTE_DIAGNOSTIC',
             'capture_authority': identity, 'original_weights_and_terminals': True,
             'cost_method': 'original AStar segment quadrature and explicit high-cost fallback',
             'curve_sampling_dt_s': float(t[1]-t[0]),
             'curve_risk_sampling_spread_m': max(results[k]['risk_cost_m'] for k in
                 ('captured_curve', 'captured_curve_coarse', 'captured_curve_fine'))-
                 min(results[k]['risk_cost_m'] for k in
                 ('captured_curve', 'captured_curve_coarse', 'captured_curve_fine')),
             'new_curve_generated': False, 'full_branch_ranking_qualified': False,
             'execution_authorized': False, 'route_costs': results}
    (out/'recheck.json').write_text(json.dumps(proof, indent=2))
    paths = [*out.iterdir(), *(run/'export/advisory/validation').glob(args.label+'_*/*')]
    write_subordinate_manifest(run, args.label, {'commands': commands, 'source': source_identity(),
        'binaries': [binary_identity(p.resolve()) for p in (args.search_binary, args.predictor_binary)],
        'input_sha256': {str(p): sha(p) for p in (payload, snapshot/'snapshot.json', snapshot/'cells.bin',
                                                 args.parameters, args.left_guide, args.right_guide)},
        'artifacts_sha256': {str(p.relative_to(run)): sha(p) for p in paths if p.is_file()}})
    print(json.dumps(proof))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
