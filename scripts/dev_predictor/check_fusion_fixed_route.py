#!/usr/bin/env python3
"""Check an experimental two-side route against native frozen physical authority."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO/'scripts/dev_planner'))
from replay_on_search import replay_input, verify_planning_capture
from advisory_validation import sha, binary_identity, source_identity, safe_label
from run_directory import (adopt_run_directory, resolve_run_directory,
                           write_subordinate_manifest, finalize_run)


def digest(value):
    import hashlib
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def native_input(meta, params, route):
    lines = replay_input(meta, params, 1.).splitlines()
    # Preserve map, motion, original terminals/reserve/ceiling and task endpoint;
    # only the offline polyline under assessment is replaced.
    lines[9:12] = [f'0 {len(route)-1} {len(route)}'] + [' '.join(map(str, p)) for p in route]
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--left-guide', type=Path, required=True)
    parser.add_argument('--right-guide', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    safe_label(args.label)
    inherited = os.environ.get('IAP_RUN_DIR')
    run = adopt_run_directory(inherited) if inherited else resolve_run_directory(
        entrypoint='fusion_fixed_route_prerequisite', scenario='icra_dense_forest_four_fork_v2')
    snapshot = args.snapshot.resolve()
    meta = json.loads((snapshot/'snapshot.json').read_text())
    payload = snapshot/'planning_input.bin'
    identity = verify_planning_capture(snapshot, payload, meta)
    params = json.loads(args.parameters.read_text())
    guides = [json.loads(p.read_text()) for p in (args.left_guide, args.right_guide)]
    if not all(g.get('guide_found') and g.get('scope') == 'REAL_FROZEN' for g in guides):
        raise ValueError('complete real frozen local guides required')
    left, right = [g['path_m'] for g in guides]
    if left[0] != right[0] or left[0] != meta['real_start_p_m']:
        raise ValueError('original guide/map start mismatch')
    task = [params['fsm/waypoint0_'+axis] for axis in 'xyz']
    local = [[-18., 0., 1.5], *left, *left[-2::-1], *right[1:], *right[-2::-1]]
    full = [*local, task]
    out = run/'export/analysis'/args.label
    out.mkdir(parents=True, exist_ok=False)
    verdicts = {}
    commands = []
    for name, route in (('two_side_local', local), ('retained_task_endpoint', full)):
        request = out/(name+'.txt')
        request.write_text(native_input(meta, params, route))
        result = out/(name+'.json')
        command = [str(args.binary.resolve()), str(snapshot/'cells.bin'), '--route-check', str(payload), str(result)]
        with request.open() as stream, (out/(name+'.log')).open('x') as log:
            process = subprocess.run(command, stdin=stream, stdout=log, stderr=subprocess.STDOUT)
        if process.returncode:
            raise RuntimeError('native fixed route check failed: '+str(process.returncode))
        value = json.loads(result.read_text())
        value['route_sha256'] = digest({'points': route})
        verdicts[name] = value
        commands.append(command)
    proof = {'identity': 'EXPERIMENTAL_ROUTE_PREREQUISITE', 'captured_identity': identity,
             'reference_route': {'points': full}, 'route_sha256': digest({'points': full}),
             'verdicts': verdicts, 'qualification': 'GEOMETRY_ONLY_NO_DYNAMIC_EXECUTION_AUTHORIZATION',
             'formal_9_plus_9_started': False}
    with (out/'prerequisite.json').open('x') as stream:
        json.dump(proof, stream, indent=2)
    write_subordinate_manifest(run, 'fusion_route_'+args.label, {
        'commands': commands, 'source': source_identity(), 'binary': binary_identity(args.binary.resolve()),
        'input_sha256': {str(p): sha(p) for p in (payload, snapshot/'cells.bin', snapshot/'snapshot.json',
                                                   args.parameters, args.left_guide, args.right_guide)},
        'artifacts_sha256': {str(p.relative_to(run)): sha(p) for p in out.iterdir() if p.is_file()}})
    if not inherited:
        finalize_run(run, lifecycle='completed', safety_outcome='not_applicable')
    print(json.dumps(verdicts))
    return 0 if all(v['physical_valid'] for v in verdicts.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
