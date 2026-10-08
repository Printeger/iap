#!/usr/bin/env python3
"""Traceable offline fork sampling, retaining failed baseline/unknown coverage."""
import argparse
import csv
import json
import os
from pathlib import Path
import statistics
import subprocess
from advisory_validation import (binary_identity, sha, validate_record, manifest)
from run_directory import (adopt_run_directory, resolve_run_directory, finalize_run)


def replay_arguments(geometry, parameters):
    g = geometry
    return ' '.join(map(str, [g['forked_forest.'+k] for k in
        ('fork_x_min_m','fork_length_m','low_risk_amplitude_m','high_risk_amplitude_m',
         'corridor_width_m','junction_clearance_radius_m','risk_seed')] +
        [parameters['fsm/waypoint0_z']] + [parameters['planning/'+k] for k in
         ('body_radius_m','tracking_reserve_m','current_motion_budget_m','current_motion_max_age_s',
          'environment_max_age_s','advisory_hpl_budget_m','advisory_vpl_budget_m','advisory_hpl_reserve_m',
          'advisory_vpl_reserve_m','advisory_unknown_multiplier','advisory_stale_soft_s')] +
        [parameters['fsm/waypoint0_'+k] for k in 'xyz']))+'\n'


def summarize(rows):
    valid = [r for r in rows if r['queried']=='1' and r['valid']=='1']
    result = {'samples':len(rows), 'unobserved':sum(r['observed']=='0' for r in rows),
              'occupied':sum(r['raw_occupied']=='1' or r['inflated_occupied']=='1' for r in rows),
              'physical_ok':sum(r['physical_reason']=='OK' for r in rows), 'valid_predictions':len(valid),
              'gnss_admitted':sum(r['gnss_used']=='1' for r in valid),
              'lidar_admitted':sum(r['lidar_used']=='1' for r in valid)}
    for key in ('hpl','vpl','gnss_information_trace','lidar_information_trace'):
        values=[float(r[key]) for r in valid if r[key]]
        result[key+'_median']=statistics.median(values) if values else None
    return result


def verify_libraries(captured, replay):
    """Installation prefixes may differ; dependency identity must not disappear."""
    def by_name(libraries):
        result = {}
        for path, digest in libraries.items():
            name = Path(path).name
            if name in result and result[name] != digest:
                raise ValueError('ambiguous dependency identity: '+name)
            result[name] = digest
        return result
    old, current = by_name(captured), by_name(replay)
    for name in ('libiap.so', 'libplan_env.so', 'libpath_searching.so'):
        if name not in old or name not in current:
            raise ValueError('required dependency absent: '+name)
    for name in old.keys() & current.keys():
        # New inline diagnostic access changes DWARF; production source is
        # checked separately. Predictor/physical dependencies have no exception.
        if name != 'libpath_searching.so' and old[name] != current[name]:
            raise ValueError('captured Predictor/physical library mismatch: '+name)


def route_risk_covered(routes):
    return all(r['reference_executable'] and r['model_unknown_vertices']==0 and
               r['model_unknown_integral_samples']==0 for r in routes.values())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_run',type=Path)
    parser.add_argument('--binary',type=Path,required=True)
    parser.add_argument('--label',default='fork_channel_comparison')
    args=parser.parse_args()
    if not args.label.replace('_','').isalnum():raise ValueError('unsafe label')
    source=args.source_run.resolve()
    primary=json.loads((source/'metadata/run_manifest.json').read_text())
    process=json.loads((source/'metadata/manifests/forest_process_result.json').read_text())
    capture_path=source/'export/advisory/forks/entrances.json'
    capture=json.loads(capture_path.read_text())
    registration=json.loads((source/'metadata/manifests/fork_inputs.json').read_text())
    if (registration['artifacts_sha256'].get(str(capture_path.relative_to(source)))!=sha(capture_path) or
            registration['source']['revision']!=primary['source']['git_commit'] or
            'metadata/manifests/fork_inputs.json' not in primary['subordinate_manifests']):
        raise ValueError('fork capture manifest identity mismatch')
    parameters_path=source/'metadata/config/planner_parameters.json'
    parameters=json.loads(parameters_path.read_text())
    if (primary['scenario']!='icra_dense_forest_four_fork_v2' or not primary['source']['git_worktree_clean'] or
            parameters['planning/advisory_guidance_enabled'] is not False or parameters['risk/use_posterior_prior'] is not False):
        raise ValueError('clean explicitly OFF physical reference required')
    arrival_lifecycle_qualified=bool(primary['lifecycle']=='completed' and process['task_reached_observed'] and
                                    all(code==0 for code in process['exit_codes'].values()))
    # Arrival and process health do not constitute an execution identity audit.
    # No such qualified audit is attached to these reference captures.
    baseline_qualified=False
    replay_binary=binary_identity(args.binary.resolve())
    inputs=[]
    for entry in capture['entries']:
        if entry['status']!='CAPTURED':continue
        payload=(source/entry['payload']).resolve();payload.relative_to(source)
        if sha(payload)!=entry['payload_sha256']:raise ValueError('fork capture checksum mismatch')
        record=validate_record(payload,payload.with_name('record.json'))
        if record['revision']!=primary['source']['git_commit'] or record['physical_geometry']!=capture['physical_geometry']:
            raise ValueError('fork source/physical geometry identity mismatch')
        cost_path='src/iap/planner/path_searching/src/dyn_a_star.cpp'
        captured_source=subprocess.check_output(['git','-C',str(Path(__file__).resolve().parents[2]),
            'show',record['revision']+':'+cost_path])
        current_source=(Path(__file__).resolve().parents[2]/cost_path).read_bytes()
        if captured_source!=current_source:
            raise ValueError('captured production segment implementation changed')
        header='src/iap/planner/path_searching/include/path_searching/dyn_a_star.h'
        old_header=subprocess.check_output(['git','-C',str(Path(__file__).resolve().parents[2]),
            'show',record['revision']+':'+header],text=True)
        current_header=(Path(__file__).resolve().parents[2]/header).read_text()
        diagnostic_access = """    // Read-only offline attribution on a fresh checker. Uses the production
    // segment quadrature and physical queries; never authorizes execution.
    std::optional<double> diagnosticSegmentCost(const Eigen::Vector3d& from,
                                               const Eigen::Vector3d& to) {
        return segmentCost(from, to, true);
    }
"""
        if old_header!=current_header and old_header!=current_header.replace(diagnostic_access,'',1):
            raise ValueError('captured production cost header changed beyond diagnostic access')

        verify_libraries(record['producer_binary']['libraries_sha256'],replay_binary['libraries_sha256'])
        inputs.append((entry,payload,record))
    inherited=os.environ.get('IAP_RUN_DIR')
    run=adopt_run_directory(inherited) if inherited else resolve_run_directory(entrypoint='fork_channel_comparison',scenario=primary['scenario'])
    out=run/'export/analysis'/args.label;out.mkdir(parents=True,exist_ok=False)
    report={'identity':'FROZEN_FORK_DIAGNOSTIC','source_run':primary['run_id'],
            'baseline_qualified':baseline_qualified,'execution_authorized':False,'formal_risk_conclusion':False,
            'arrival_lifecycle_qualified':arrival_lifecycle_qualified,'execution_identity_verified':False,
            'baseline_limitation':'No qualified execution identity audit attached; arrival/lifecycle alone is insufficient.',
            'capture_sha256':sha(capture_path),'parameters_sha256':sha(parameters_path),'binary':replay_binary,
            'cost_source_sha256':sha(Path(__file__).resolve().parents[2]/'src/iap/planner/path_searching/src/dyn_a_star.cpp'),
            'cost_scope':'unchanged captured implementation; new diagnostic access seam may change cost-library debug identity',
            'entries':capture['entries'],'forks':[]}
    try:
        command_input=replay_arguments(capture['physical_geometry'],parameters)
        (out/'policy_input.txt').write_text(command_input)
        for entry,payload,record in inputs:
            label=args.label+'_fork'+str(entry['fork_index'])
            command=[str(args.binary.resolve()),str(payload),label,str(entry['fork_index'])]
            result=subprocess.run(command,input=command_input,text=True,capture_output=True,timeout=120,
                env={**os.environ,'IAP_RUN_DIR':str(run),'ROS_LOG_DIR':str(run/'runtime/ros')})
            (out/(label+'.log')).write_text(result.stdout+result.stderr)
            if result.returncode:raise RuntimeError('fork replay failed: '+result.stderr)
            root=run/'export/advisory/forks'/label
            replay=json.loads((root/'result.json').read_text())
            if any(replay[k]!=record[k] for k in ('generation','frame_id','geometry_id')):
                raise ValueError('frozen geometry/frame/generation mismatch')
            with (root/'samples.csv').open() as stream:rows=list(csv.DictReader(stream))
            replay.update(payload_sha256=sha(payload),capture_record_sha256=sha(payload.with_name('record.json')),
                          left=summarize([r for r in rows if float(r['y'])>=0]),
                          right=summarize([r for r in rows if float(r['y'])<=0]))
            replay['fair_route_comparison']=bool(baseline_qualified and replay['entrance_qualified'] and
                route_risk_covered(replay['routes']))
            left,right=replay['routes']['left'],replay['routes']['right']
            replay['risk_rescale_break_even']=None
            if route_risk_covered(replay['routes']) and all(r['risk_addition_m'] is not None for r in (left,right)):
                difference=right['risk_addition_m']-left['risk_addition_m']
                if difference:
                    replay['risk_rescale_break_even']=(left['length_m']+left['terminal_m']-right['length_m']-right['terminal_m'])/difference
            report['forks'].append(replay)
        (out/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        lines=['# 四分岔冻结输入对照', '', '执行基线资格：'+('通过' if baseline_qualified else '未通过；以下仅作离线诊断'),
               '', '| 分岔 | 侧 | 样本 | 未观测 | 占用 | 物理可检查 | 有效预测 | GNSS 准入 | LiDAR 准入 | HPL 中位数 | VPL 中位数 |',
               '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
        for fork in report['forks']:
            for side in ('left','right'):
                r=fork[side]
                values=[fork['fork_index']+1,side]+[r[k] for k in ('samples','unobserved','occupied','physical_ok','valid_predictions','gnss_admitted','lidar_admitted','hpl_median','vpl_median')]
                lines.append('| '+' | '.join('未知' if v is None else f'{v:.4f}' if isinstance(v,float) else str(v) for v in values)+' |')
        lines += ['', '取样覆盖两条通道的三条宽度带、入口到出口及实际状态连接段，原体素中心去重后每批 64 个查询。',
                  '表中 PL 只描述有预测覆盖的样本；未观测位置未查询、未填零。中位数不能替代全路线排序。',
                  '参考折线不可执行时，合法总成本留空；这不证明整个分支不可达。生产积分含物理检查及原成本倍率、拟合余量和终端距离。',
                  '当前输入不包含该入口的完整生产终端集合与实际后端候选，未据此声称完成 OFF/ON 搜索或曲线选路验证。']
        lines += ['', '长度／风险附加／终端／总成本（生产规则，参考通道存在物理未知时留空）：', '',
                  '| 分岔 | 侧 | 长度 | 风险附加 | 终端项 | 总成本 | 未观测参考顶点 |',
                  '|---|---|---:|---:|---:|---:|---:|']
        for fork in report['forks']:
            for side,route in fork['routes'].items():
                values=[fork['fork_index']+1,side]+[route[k] for k in ('length_m','risk_addition_m','terminal_m','total_cost_m','unknown_vertices')]
                lines.append('| '+' | '.join('未知' if v is None else f'{v:.4f}' if isinstance(v,float) else str(v) for v in values)+' |')
        (out/'report.md').write_text('\n'.join(lines)+'\n')
        hashes={str(p.relative_to(run)):sha(p) for p in out.rglob('*') if p.is_file()}
        for fork in report['forks']:
            label=args.label+'_fork'+str(fork['fork_index'])
            hashes.update({str(p.relative_to(run)):sha(p) for p in (run/'export/advisory/forks'/label).rglob('*') if p.is_file()})
        manifest(run,args.label,{'report':str(out.relative_to(run)),**report,'artifacts_sha256':hashes}, owner=not inherited)
        if not inherited:finalize_run(run,lifecycle='completed')
        print(out)
    except Exception as exc:
        manifest(run,args.label+'_failure',{'error':str(exc),'source_run':str(source)}, owner=not inherited)
        if not inherited:finalize_run(run,lifecycle='failed')
        raise


if __name__=='__main__':main()
