#!/usr/bin/env python3
"""Same-map production OFF/ON search; sparse evidence never becomes full replay."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'scripts/dev_predictor'))
from advisory_validation import binary_identity, sha
from run_directory import resolve_run_directory, adopt_run_directory, finalize_run, write_subordinate_manifest
from analyze_failure_map import _number, _point


def replay_input(meta, params, budget):
    m = meta
    lines = [' '.join(map(str, m['dimensions'])), _point(m['origin_m']), _point(m['max_boundary_m']),
        f"{_number(m['resolution_m'])} {_number(m['cloud_stamp_s'])} {m['generation']} {json.dumps(m['frame_id'])} {_number(m['planning_time_s'])}",
        ' '.join([str(m['motion_quality']), str(int(m['motion_allow_bridged']))] + [_number(m[k]) for k in
            ('motion_stamp_s','motion_error_proxy_m','motion_body_radius_m','motion_tracking_reserve_m',
             'motion_budget_m','motion_max_age_s','environment_max_age_s')]),
        ' '.join(map(str,m['search_pool_dimensions']))+' '+_number(m['search_step_size_m']),
        _point(m['search_pool_center_m']), _point(m['real_start_p_m']), _point(m['planning_goals_m'][0]),
        '0 1 2', _point(m['real_start_p_m']), _point(m['planning_goals_m'][0]), f"{m['search_failure']} {_number(budget)}",
        f"{_number(m['guide_fitting_reserve_m'])} {_number(m['guide_reserve_taper_distance_m'])} {len(m['planning_goals_m'])}"]
    lines += [_point(p) for p in m['planning_goals_m']]
    lines += [_point([params['fsm/waypoint0_'+axis] for axis in 'xyz']),
        ' '.join(_number(params['planning/'+key]) for key in
            ('advisory_hpl_budget_m','advisory_vpl_budget_m','advisory_hpl_reserve_m',
             'advisory_vpl_reserve_m','advisory_unknown_multiplier','advisory_stale_soft_s')),
        f"{_number(m['risk_reference_time_s'])} {_number(m['risk_valid_until_s'])}"]
    return '\n'.join(lines)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot',type=Path)
    parser.add_argument('--parameters',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--payload',type=Path)
    parser.add_argument('--budget-s',type=float,default=1.)
    parser.add_argument('--label',default='original_search')
    args=parser.parse_args()
    if not args.label.replace('_','').isalnum(): raise ValueError('unsafe label')
    m=json.loads((args.snapshot/'snapshot.json').read_text())
    params=json.loads(args.parameters.read_text())
    data=replay_input(m,params,args.budget_s)
    inherited=os.environ.get('IAP_RUN_DIR')
    run=adopt_run_directory(inherited) if inherited else resolve_run_directory(
        entrypoint='on_search_replay',scenario='icra_dense_forest_four_fork_v2')
    out=run/'export/analysis'/args.label
    out.mkdir(parents=True,exist_ok=False)
    result={'source_run':m['run_id'],'planning_attempt_id':m['planning_attempt_id'],
        'budget_s':args.budget_s,'parameters_sha256':sha(args.parameters),
        'input_sha256':{f.name:sha(f) for f in args.snapshot.iterdir() if f.is_file()},
        'binary':binary_identity(args.binary.resolve()),'full_prediction_replay':bool(args.payload)}
    if args.payload: result['payload_sha256']=sha(args.payload)
    status='failed'
    try:
        (out/'replay_input.txt').write_text(data)
        for mode in ('off','full' if args.payload else 'sparse'):
            risk=args.payload if mode=='full' else args.snapshot/'queried_risk.csv'
            command=[str(args.binary.resolve()),str((args.snapshot/'cells.bin').resolve()),'--planning-search',mode,str(risk.resolve())]
            completed=subprocess.run(command,input=data,text=True,capture_output=True,timeout=args.budget_s+30)
            (out/(mode+'.stderr.log')).write_text(completed.stderr)
            if completed.returncode: raise RuntimeError(completed.stderr)
            verdict=json.loads(completed.stdout)
            verdict['scope']='REAL_FROZEN' if mode=='full' else 'PHYSICAL_OFF' if mode=='off' else 'SAVED_PL_PARTIAL_DIAGNOSTIC'
            (out/(mode+'.json')).write_text(json.dumps(verdict,indent=2)+'\n')
            result[mode]=verdict
        result['historical_full_on_status']='REPLAYED' if args.payload else 'INCONCLUSIVE_MISSING_COMPLETE_INPUT'
        if result['input_sha256']!={f.name:sha(f) for f in args.snapshot.iterdir() if f.is_file()}:
            raise RuntimeError('frozen input changed during replay')
        if args.payload and result['payload_sha256']!=sha(args.payload):
            raise RuntimeError('prediction payload changed during replay')
        status='completed'
    finally:
        write_subordinate_manifest(run,args.label,result)
        if not inherited: finalize_run(run,lifecycle=status)
    print(json.dumps({'run':str(run),**{k:{n:v.get(n) for n in ('guide_found','failure','search_s','predictor_calls','advisory_calls','expanded','missing_unique_voxels')} for k,v in result.items() if k in ('off','sparse','full')}}))

if __name__=='__main__':main()
