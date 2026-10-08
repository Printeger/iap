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
from advisory_validation import binary_identity, sha, manifest
from run_directory import resolve_run_directory, adopt_run_directory, finalize_run
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
        f"{_number(m['virtual_ceiling_height_m'])} {_number(m['inflation_radius_m'])}",
        f"{_number(m['risk_reference_time_s'])} {_number(m['risk_valid_until_s'])}"]
    return '\n'.join(lines)+'\n'


def verify_planning_capture(snapshot, payload, meta):
    run=(snapshot/ meta['run_manifest']).resolve().parent.parent
    primary=json.loads((run/'metadata/run_manifest.json').read_text())
    ledger=run/'metadata/manifests/planning_input_capture.json'
    if not ledger.exists():
        # Direct canonical launches have the writer's native artifact manifest,
        # without the driver collector ledger. Do not fabricate the latter or
        # claim a historically recorded binary/payload digest.
        native=run/'metadata/manifests'/('planner_failure_map_'+meta['artifact_label']+'.json')
        record=json.loads(native.read_text())
        if (not primary['source']['git_worktree_clean'] or
                (run/'export'/record['snapshot']).resolve()!=(snapshot/'snapshot.json').resolve() or
                (run/'export'/record['planning_input']).resolve()!=payload.resolve() or
                record['planning_attempt_id']!=meta['planning_attempt_id'] or
                record['risk_version']!=meta['risk_version'] or
                meta['planning_input_risk_version']!=meta['risk_version']):
            raise ValueError('native planning capture identity mismatch')
        return {'authority':'NATIVE_FAILURE_WRITER','recorded_revision':primary['source']['git_commit'],
                'observed_payload_sha256':sha(payload),'native_manifest_sha256':sha(native),
                'primary_registered':str(native.relative_to(run)) in primary.get('subordinate_manifests',[]),
                'limitation':'Driver ledger and historical producer binary/payload hashes unavailable; native snapshot/map/payload identity checked by C++ replay.'}
    capture=json.loads(ledger.read_text())
    if (not primary['source']['git_worktree_clean'] or capture['source']['dirty'] or
        capture['source']['revision']!=primary['source']['git_commit']):
        raise ValueError('unclean or mismatched captured revision')
    matching=[entry for entry in capture['inputs'] if (run/entry['payload']).resolve()==payload.resolve()]
    if len(matching)!=1: raise ValueError('payload not uniquely registered in planning capture')
    entry=matching[0]
    expected={'planning_attempt_id':meta['planning_attempt_id'],
        'risk_version':meta['planning_input_risk_version'],'generation':meta['generation'],
        'reference_time_s':meta['planning_time_s'],'payload_sha256':sha(payload),
        'snapshot_sha256':sha(snapshot/'snapshot.json')}
    if (any(entry.get(k)!=v for k,v in expected.items()) or
        (run/entry['snapshot']).resolve()!=(snapshot/'snapshot.json').resolve() or
        meta['planning_input_risk_version']!=meta['risk_version']):
        raise ValueError('captured planning attempt/risk/hash identity mismatch')
    return entry


def measurement_profile(meta, label):
    # The online gate must use the captured instrumentation configuration.
    # Instrumented controls are separate evidence, with their overhead retained.
    if label.startswith('profiled_'): return True
    if label=='extended_unprofiled': return False
    return bool(meta.get('search_performance_diagnostics',False))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('snapshot',type=Path)
    parser.add_argument('--parameters',type=Path,required=True)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--payload',type=Path)
    parser.add_argument('--budget-s',type=float,default=1.)
    parser.add_argument('--label',default='original_search')
    parser.add_argument('--require-guide',action='store_true',help='fail the regression gate if either production search has no guide')
    parser.add_argument('--diagnose',action='store_true',help='three cold measurements, same-cache warm control and offline extended search; never online authorization')
    parser.add_argument('--diagnostic-goal-indices', help='Offline subset of captured terminals; full endpoint clearance context retained')
    args=parser.parse_args()
    if args.diagnostic_goal_indices and (args.require_guide or args.diagnose):
        raise ValueError('goal subsets are offline witnesses and cannot grant online capability')
    if not 0 < args.budget_s <= 30:raise ValueError('replay budget must be positive and at most 30 s')
    if args.require_guide and args.budget_s!=1.:
        raise ValueError('--require-guide uses the original 1 s search budget; extended diagnostics cannot grant online capability')
    if not args.label.replace('_','').isalnum(): raise ValueError('unsafe label')
    m=json.loads((args.snapshot/'snapshot.json').read_text())
    params=json.loads(args.parameters.read_text())
    capture=verify_planning_capture(args.snapshot,args.payload,m) if args.payload else None
    if args.diagnose and not args.payload:raise ValueError('diagnostic controls require the actual complete payload')
    data=replay_input(m,params,args.budget_s)
    inherited=os.environ.get('IAP_RUN_DIR')
    run=adopt_run_directory(inherited) if inherited else resolve_run_directory(
        entrypoint='on_search_replay',scenario='icra_dense_forest_four_fork_v2')
    out=run/'export/analysis'/args.label
    out.mkdir(parents=True,exist_ok=False)
    result={'source_run':m['run_id'],'planning_attempt_id':m['planning_attempt_id'],
        'budget_s':args.budget_s,'parameters_sha256':sha(args.parameters),
        'input_sha256':{f.name:sha(f) for f in args.snapshot.iterdir() if f.is_file()},
        'binary':binary_identity(args.binary.resolve()),'full_prediction_replay':False,
        'diagnostic_goal_indices':args.diagnostic_goal_indices}
    if args.payload: result.update(payload_sha256=sha(args.payload),capture_authority=capture)
    result['measurements']=[]
    status='failed'
    try:
        (out/'replay_input.txt').write_text(data)
        groups=[(mode,mode,args.budget_s) for mode in ('off','full' if args.payload else 'sparse')]
        if args.diagnose:groups += [('warm','warm',args.budget_s),('extended','full',15.),('extended_unprofiled','full',15.)]
        if args.diagnose:groups += [('profiled_off','off',args.budget_s),('profiled_full','full',args.budget_s),('profiled_extended','full',15.)]
        for label,mode,budget in groups:
          for repetition in range(3 if args.diagnose and label in ('off','full','warm') else 1):
            risk=args.payload if mode=='full' else args.snapshot/'queried_risk.csv'
            if mode=='warm':risk=args.payload
            command=[str(args.binary.resolve()),str((args.snapshot/'cells.bin').resolve()),'--planning-search',mode,str(risk.resolve())]
            if args.diagnostic_goal_indices:command.append(args.diagnostic_goal_indices)
            completed=subprocess.run(command,input=replay_input(m,params,budget),text=True,capture_output=True,timeout=budget+35,
                env={**os.environ,'IAP_REPLAY_PROFILE':'1' if measurement_profile(m,label) else '0'})
            name=label+'_'+str(repetition) if args.diagnose else label
            (out/(name+'.stderr.log')).write_text(completed.stderr)
            if completed.returncode: raise RuntimeError(f"{mode} replay exit {completed.returncode}: {completed.stderr}")
            verdict=json.loads(completed.stdout)
            verdict['scope']='REAL_FROZEN' if mode=='full' else 'PHYSICAL_OFF' if mode=='off' else 'SAVED_PL_PARTIAL_DIAGNOSTIC'
            if mode=='warm':verdict['scope']='SAME_FROZEN_GRIDMAP_CACHE_DIAGNOSTIC'
            verdict.update(label=label,repetition=repetition,search_budget_s=budget,
                           captured_profiling_enabled=bool(m.get('search_performance_diagnostics',False)),
                           online_capability_evidence=not args.diagnostic_goal_indices and label in ('off','full') and budget==1.)
            (out/(name+'.json')).write_text(json.dumps(verdict,indent=2)+'\n')
            result['measurements'].append(verdict)
            if repetition==0:result[label]=verdict
        result['historical_full_on_status']='REPLAYED' if args.payload else 'INCONCLUSIVE_MISSING_COMPLETE_INPUT'
        result['full_prediction_replay']=bool(args.payload)
        if result['input_sha256']!={f.name:sha(f) for f in args.snapshot.iterdir() if f.is_file()}:
            raise RuntimeError('frozen input changed during replay')
        if args.payload and result['payload_sha256']!=sha(args.payload):
            raise RuntimeError('prediction payload changed during replay')
        if args.require_guide and not all(v['guide_found'] for v in result['measurements'] if v['label'] in ('off','full','sparse')):
            raise RuntimeError('production search did not deliver a guide within the original budget')
        status='completed'
    except Exception as exc:
        result['error']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        manifest(run,args.label,result,owner=not inherited)
        if not inherited: finalize_run(run,lifecycle=status)
    print(json.dumps({'run':str(run),**{k:{n:v.get(n) for n in ('guide_found','failure','search_s','predictor_calls','advisory_calls','expanded','missing_unique_voxels')} for k,v in result.items() if k in ('off','sparse','full')}}))

if __name__=='__main__':main()
