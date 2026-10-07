#!/usr/bin/env python3
"""Summarize committed canonical live trials without promoting diagnostic pairs."""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import numpy as np

# The archived copy is runnable with --repo; the installed source uses its root.
parser=argparse.ArgumentParser()
parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[2])
parser.add_argument('--trial',type=Path,action='append',required=True)
parser.add_argument('--label',default='forest_20261007')

def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def read(path):return json.loads(path.read_text())
def rows(path):return list(csv.DictReader(path.open()))
def stamp(msg):return msg['header']['stamp']['sec']+msg['header']['stamp']['nanosec']*1e-9
def point(msg):return [msg['pose']['pose']['position'][k] for k in ('x','y','z')]
def csv_write(path,values,fields):
    with path.open('x') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(values)

def information_diagnostics(value):
    if value is None:return {'lambda_min':None,'lambda_max':None,'weak_x':None,'weak_y':None,'weak_z':None}
    # Consume the eigensystem exported by the production predictor.
    return {'lambda_min':value['eigenvalues'][0],'lambda_max':value['eigenvalues'][-1],
            **dict(zip(('weak_x','weak_y','weak_z'),value['weak_direction']))}

def araim_rows(path):
    """Count only rows matching the recorded header; never shift old columns."""
    result={'epoch_rows':0,'worst_hyp_sat_id_minus_one':0,'one_constellation_epochs':0,
            'gnss_rejected_epochs':0,'invalid_rows_by_type':{}}
    invalid=Counter()
    with path.open() as f:
        reader=csv.reader(f);header=next(reader)
        for fields in reader:
            kind=fields[0] if fields else 'empty'
            if len(fields)!=len(header):
                invalid[kind]+=1;continue
            row=dict(zip(header,fields))
            if kind=='epoch':
                result['epoch_rows']+=1
                result['one_constellation_epochs']+=row['n_const']=='1'
                result['gnss_rejected_epochs']+=row['gnss_valid']=='0'
            if kind=='worst_hyp' and row['sat_id']=='-1':
                result['worst_hyp_sat_id_minus_one']+=1
    result['invalid_rows_by_type']=dict(invalid)
    if invalid['worst_hyp']:result['worst_hyp_sat_id_minus_one']=None
    return result

def unique_trials(paths):
    resolved=[p.resolve() for p in paths]
    if len(resolved)!=len(set(resolved)):raise ValueError('duplicate independent trial')
    ids=[read(p/'metadata/run_manifest.json')['run_id'] for p in resolved]
    if len(ids)!=len(set(ids)):raise ValueError('duplicate run identity')
    return resolved

def validated_replays(run):
    result={}
    for manifest_path in (run/'metadata/manifests').glob('advisory_replay_*.json'):
        m=read(manifest_path)
        if m['identity']!='REAL_REPLAY' or m['dirty'] or not m.get('source_sha256') or not m.get('binary',{}).get('libraries_sha256'):
            raise ValueError('replay provenance unavailable')
        label=manifest_path.stem.removeprefix('advisory_replay_')
        root=run/'export/advisory/validation'/label
        for filename in ('input.bin','input.json','points.csv','matrices.jsonl','variant.json'):
            path=root/filename;key=str(path.relative_to(run))
            if m['artifacts_sha256'].get(key)!=digest(path):raise ValueError('replay artifact checksum mismatch: '+key)
        current=root.with_name(label+'_current')
        if digest(current/'input.json')!=digest(root/'input.json') or digest(current/'input.bin')!=digest(root/'input.bin'):
            raise ValueError('receiver replay input mismatch')
        # Older manifests omitted sibling receiver rows. Their PL is displayed
        # only as an unpaired diagnostic, never used for qualified coverage.
        p=current/'points.csv'
        provenance='MANIFEST_BOUND' if m['artifacts_sha256'].get(str(p.relative_to(run)))==digest(p) else 'UNREGISTERED_RECEIVER_DIAGNOSTIC'
        result[digest(root/'input.bin')]=(p,read(root/'input.json'),rows(p)[0],m,provenance)
    return result

def truth_at(times,positions,t):
    i=bisect.bisect_left(times,t)
    if i<len(times) and times[i]==t:return np.asarray(positions[i])
    if not 0<i<len(times):raise ValueError('truth_time_unmatched')
    if max(t-times[i-1],times[i]-t)>.05:raise ValueError('truth_time_gap')
    u=(t-times[i-1])/(times[i]-times[i-1])
    return (1-u)*np.asarray(positions[i-1])+u*np.asarray(positions[i])

def capture(run):
    files=list((run/'metadata/manifests').glob('advisory_capture_*.json'))
    if len(files)!=1:raise ValueError('one authoritative live capture required')
    info=read(files[0]); name,expected=next(iter(info['artifacts_sha256'].items()))
    path=(run/name).resolve();path.relative_to(run)
    if info['identity']!='LIVE_MEASUREMENT' or digest(path)!=expected:raise ValueError('capture checksum mismatch')
    truth={};odom=[];cmd=Counter();motion=Counter();gnss=Counter();published=set();execution=set()
    for line in path.open():
        e=json.loads(line);m=e['payload'];topic=e['topic']
        if topic=='/sim/drone_0/truth_odom':
            if m['header']['frame_id']!='map' or m['child_frame_id']!='drone_0':raise ValueError('truth frame/body mismatch')
            truth[stamp(m)]=point(m)
        elif topic=='/drone_0_visual_slam/odom':
            if m['header']['frame_id']!='map' or m['child_frame_id']!='imu':raise ValueError('estimate frame/body mismatch')
            odom.append((stamp(m),point(m)))
        elif topic=='/iap/integrity':
            motion[str(m['current_motion_quality'])]+=1;gnss[str(m['gnss_valid'])]+=1
        elif topic=='/drone_0_planning/bspline':published.add(m['traj_id'])
        elif topic=='/drone_0_planning/pos_cmd':
            cmd[m['trajectory_id']]+=1
            if m.get('execution_instance_id',0)>0:execution.add(m['execution_instance_id'])
    ordered=sorted(truth);positions=[truth[t] for t in ordered]
    odom.sort()
    if not odom:raise ValueError('live odometry unavailable')
    return ordered,positions,{'start':odom[0][1],'capture_end':odom[-1][1],
        'forward_progress_m':odom[-1][1][0]-odom[0][1][0],
        'x_range':[min(p[0] for _,p in odom),max(p[0] for _,p in odom)],
        'published_trajectory_ids':sorted(published),'commanded_trajectory_ids':sorted(k for k in cmd if k>0),
        'execution_instance_ids':sorted(execution),'current_motion_quality_counts':dict(motion),
        'gnss_monitor_valid_counts':dict(gnss),'capture_manifest':str(files[0])}

def analyze(run,audit):
    primary=read(run/'metadata/run_manifest.json')
    if (primary['entrypoint']!='iap_sim' or primary['scenario']!='icra_dense_forest_four_fork_v2' or
        primary['lifecycle']!='completed' or not primary['source']['git_worktree_clean']):raise ValueError('completed clean canonical trial required')
    cfg=read(run/'metadata/config/config_ros.json')['glim_ros']['sim']
    effective=read(run/'metadata/manifests/full_stack.json')
    if effective['advisory_guidance_enabled'] or effective['advisory_posterior_prior_enabled']:
        raise ValueError('this report requires unguided observation-only exploration')
    if (cfg['align_planner_odom_to_truth'] or not cfg['static_planner_alignment_enabled'] or
        cfg['static_planner_translation_m']!=[-18.,0.,1.5]):raise ValueError('known canonical static alignment required')
    times,truth,behavior=capture(run)
    root=run/'export/advisory/validation';by_hash=validated_replays(run)
    values=[]
    for mpath in (root/'recordings').glob('*_requests_manifest.json'):
        index=read(mpath);table=run/index['requests_csv']
        if digest(table)!=index['requests_csv_sha256'] or index['run_id']!=run.name:raise ValueError('request manifest mismatch')
        requests=rows(table)
        if {r['request_id'] for r in requests}!=set(index['request_ids']):raise ValueError('lost recording request')
        for req in requests:
            v={'run_id':run.name,'request_id':req['request_id'],'recorded':req['available']=='True',
               'pair_qualified':False,'pair_reason':req['reason'],'hpl':None,'vpl':None,'pose_error_h_diagnostic':None,
               'pose_error_v_diagnostic':None,'reference_pose_delta_s':None,'map_up_enu_angle_deg':None,
               'position':None,'prior_used':None,'gnss_used':None,'lidar_used':None,'input_json':'','points_csv':'',
               'receiver_pl_provenance':''}
            if req['payload']:
                payload=run/req['payload'];side=read(payload.with_name('record.json'))
                if (side['identity']!='REAL_FROZEN' or side['dirty'] or side['revision']!=primary['source']['git_commit'] or
                    digest(payload)!=side['payload_sha256']):raise ValueError('real frozen provenance mismatch')
                replay=by_hash.get(side['payload_sha256'])
                if not replay:v['pair_reason']='recorded_input_not_replayed'
                else:
                    p,meta,row,replay_manifest,provenance=replay
                    if (side['source_sha256']!=replay_manifest['source_sha256'] or side['codec']!=replay_manifest['codec'] or
                        not side.get('producer_binary',{}).get('libraries_sha256')):raise ValueError('record/replay model identity mismatch')
                    for lib,expected in side['producer_binary']['libraries_sha256'].items():
                        actual=replay_manifest['binary']['libraries_sha256'].get(lib)
                        if actual is not None and actual!=expected:raise ValueError('record/replay library mismatch')
                    v.update(input_json=str(p.with_name('input.json')),points_csv=str(p),receiver_pl_provenance=provenance,
                        position=meta['position'],prior_used=row['prior_used']=='1',gnss_used=row['gnss_used']=='1',lidar_used=row['lidar_used']=='1',
                        hpl=float(row['fused_hpl']) if row['fused_hpl'] else None,vpl=float(row['fused_vpl']) if row['fused_vpl'] else None,
                        reference_pose_delta_s=meta['reference_time_s']-meta['pose_stamp'])
                    c=meta['coordinates'];R=np.asarray(c['R_map_enu']).reshape(3,3)
                    v['map_up_enu_angle_deg']=math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))
                    try:
                        # Canonical IMU and truth body origins coincide. The
                        # estimate is already in map via the fixed translation.
                        d=np.asarray(meta['position'])-truth_at(times,truth,meta['pose_stamp'])
                        v['pose_error_h_diagnostic']=math.hypot(d[0],d[1]);v['pose_error_v_diagnostic']=abs(float(d[2]))
                    except ValueError:pass
                    try:
                        audit(meta)
                        if row['valid']!='1':raise ValueError('prediction_unavailable:'+row['reason'])
                        if abs(v['reference_pose_delta_s'])>.05:raise ValueError('reference_pose_not_same_time')
                        if provenance!='MANIFEST_BOUND':raise ValueError('receiver_pl_not_manifest_bound')
                        # Up alignment drift and conditioned rotation uncertainty
                        # remain unresolved; no sample earns calibration here.
                        raise ValueError('physical_up_alignment_not_qualified')
                    except ValueError as e:v['pair_reason']=str(e)
            values.append(v)
    source_cfg=read(run/'metadata/config/config_gnss.json')['integrity']
    araim=run/'export/current_integrity/iap_araim.csv'
    source={'config':str(run/'metadata/config/config_gnss.json'),'araim_csv':str(araim),
            'araim_csv_sha256':digest(araim),
            'constellation_faults_enabled':source_cfg.get('enable_gnss_constellation_faults'),
            'degrade_on_degenerate_hypothesis':source_cfg.get('gnss_degrade_on_degenerate_hypothesis'),
            **araim_rows(araim)}
    reasons=Counter();astar=Counter();events=[]
    for p in (run/'runtime/ros').glob('ego_planner_node_*.log'):
        for line in p.open():
            failure=re.search(r'A\* endpoints failure=(\w+)',line)
            if failure:astar[failure.group(1)]+=1
            match=re.search(r'Planner rebound rejected: phase=(\w+) execution=(\w+) budget_expired=(\d) repair_denied=(\d)',line)
            if match:
                phase,reason,budget,repair=match.groups();reasons[reason]+=1
                events.append({'run_id':run.name,'phase':phase,'execution_reason':reason,'budget_expired':budget,
                               'repair_denied':repair,'source_log':str(p),'line':line.strip()})
    return values,events,{'run_id':run.name,'revision':primary['source']['git_commit'],
                         'independent_role':'exploratory_only','behavior':behavior,'source_audit':source,
                         'planning_rejection_counts':dict(reasons),'astar_endpoint_failure_counts':dict(astar),
                         'run_manifest':str(run/'metadata/run_manifest.json')}

def main(args):
    sys.path.insert(0,str(args.repo/'scripts/dev_predictor'))
    from advisory_validation import adopt_run_directory,artifact,json_write,manifest,safe_label
    from advisory_coordinate_evidence import audit
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    run=adopt_run_directory(os.environ['IAP_RUN_DIR']);safe_label(args.label)
    out=artifact(run,'export/analysis/advisory_validation/'+args.label+'/report.md').parent
    if (out/'summary.json').exists():raise FileExistsError('formal report already exists')
    values=[];events=[];trials=[]
    for trial in unique_trials(args.trial):
        v,e,t=analyze(trial.resolve(),audit);values+=v;events+=e;trials.append(t)
    csv_write(out/'error_pair_requests.csv',values,list(values[0]))
    csv_write(out/'planning_rejections.csv',events,list(events[0]) if events else ['run_id','phase','execution_reason','budget_expired','repair_denied','source_log','line'])
    summary={'identity':'LIVE_MEASUREMENT_AND_REAL_REPLAY','trials':trials,
             'independent_exploratory_runs':len(trials),'calibration_runs':0,'held_out_runs':0,'mission_comparison_runs':0,
             'requests':len(values),'recorded':sum(v['recorded'] for v in values),
             'qualified_error_pairs':sum(v['pair_qualified'] for v in values),
             'pair_reasons':dict(Counter(v['pair_reason'] for v in values)),
             'default_calibration_promoted':False,
             'results':{'admission_numerics':'INCONCLUSIVE_DUAL_SOURCE; REAL_LIDAR_REPLAY_OBSERVED; REGRESSION_EVIDENCE_SEPARATE',
                        'source_coordinates':'INCONCLUSIVE_UP_ALIGNMENT_AND_TIME; GNSS_MONITOR_REJECTED',
                        'real_spatial_sensitivity':'PARTIAL_LIDAR_OBSERVED_REGION; FORK_NOT_COVERED',
                        'empirical_error_conformity':'INCONCLUSIVE_NO_QUALIFIED_PAIRS',
                        'curve_handover_mission':'FAIL_CANONICAL_REFERENCE_GOAL_NOT_REACHED; FIXED_ROUTE_PENDING'}}
    def save(fig,name,title):
        fig.suptitle(title);fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/name,dpi=150);plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,10))
    for t in trials:
        group=[v for v in values if v['run_id']==t['run_id']];xs=range(len(group));label=t['run_id']
        axes[0].plot(xs,[v['recorded'] for v in group],'.',label=label)
        axes[1].plot(xs,[v['reference_pose_delta_s'] for v in group],'.',label=label)
        axes[2].plot(xs,[v['map_up_enu_angle_deg'] for v in group],'.',label=label)
    axes[0].set_ylabel('recorded (not zero PL)');axes[1].set_ylabel('reference - pose (s)');axes[1].axhline(.05,c='r',linestyle='--')
    axes[2].set_ylabel('map Up vs ENU Up (deg)')
    for ax in axes:ax.legend(fontsize=7);ax.set_xlabel('recording request; independent units are runs')
    save(fig,'availability_time_coordinates.png','LIVE / REAL REPLAY: input availability and coordinate/time evidence')
    selected=[];weak_rows=[]
    for t in trials:
        group=[v for v in values if v['run_id']==t['run_id'] and v['points_csv']]
        for tag,v in zip(('start_region','movement_mid_or_stall','last_record_near_stall'),(group[0],group[len(group)//2],group[-1])):
            p=Path(v['points_csv']).parent.parent/(Path(v['points_csv']).parent.name.removesuffix('_current'))/'points.csv'
            table=rows(p);selected.append({'run_id':t['run_id'],'tag':tag,'csv':str(p),'matrices':str(p.with_name('matrices.jsonl')),
                                         'valid':sum(r['valid']=='1' for r in table),'requested':len(table)})
            selected[-1]['failure_reasons']=dict(Counter(r['status']+':'+r['reason'] for r in table if r['valid']!='1'))
            for metric in ('hpl','vpl'):
                numbers=[float(r['fused_'+metric]) for r in table if r['fused_'+metric]]
                selected[-1][metric+'_range_m']=[min(numbers),max(numbers)] if numbers else None
            for line in p.with_name('matrices.jsonl').open():
                m=json.loads(line);entry={'run_id':t['run_id'],'tag':tag,'id':m['id'],
                    'numerical_status':m['numerical_status'],'regularization_fraction':m['regularization_fraction'],
                    'lambda_min':None,'lambda_max':None,'weak_x':None,'weak_y':None,'weak_z':None}
                entry.update(information_diagnostics(m['fused_information']))
                weak_rows.append(entry)
            fig,axes=plt.subplots(2,3,figsize=(13,7))
            for j,source in enumerate(('gnss_information','lidar','fused')):
                for i,metric in enumerate(('hpl','vpl')):
                    z=np.array([float(r[source+'_'+metric]) if r[source+'_'+metric] else np.nan for r in table])
                    x=np.array([float(r['x']) for r in table]);y=np.array([float(r['y']) for r in table]);valid=np.isfinite(z)
                    ax=axes[i,j];im=ax.scatter(x[valid],y[valid],c=z[valid],cmap='viridis',marker='s',norm=Normalize(.25 if i==0 else .20,.65 if i==0 else .55))
                    ax.scatter(x[~valid],y[~valid],c='gray',marker='x');ax.set_aspect('equal');ax.set_title(source+' '+metric.upper()+' m');fig.colorbar(im,ax=ax,extend='both')
            save(fig,'spatial_'+t['run_id']+'_'+tag+'.png','REAL REPLAY '+t['run_id']+' '+tag+'; missing = gray x; prior OFF')
    summary['selected_scans']=selected
    csv_write(out/'weak_directions.csv',weak_rows,list(weak_rows[0]))
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for t in trials:
        data=[v for v in weak_rows if v['run_id']==t['run_id'] and v['lambda_min'] is not None]
        axes[0].plot(range(len(data)),[v['lambda_min'] for v in data],'.',label=t['run_id'])
        axes[1].plot(range(len(data)),[v['regularization_fraction'] for v in data],'.',label=t['run_id'])
    axes[0].set_ylabel('unregularized weakest information');axes[1].set_ylabel('epsilon weak-direction fraction');axes[1].axhline(.01,c='r',linestyle='--')
    for ax in axes:ax.legend(fontsize=6)
    save(fig,'weak_information.png','REAL REPLAY: joint information and fixed 1% regularization criterion')
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    counts=Counter(v['pair_reason'] for v in values)
    axes[0].barh(list(counts),list(counts.values()));axes[0].set_xlabel('all recording requests')
    failures=Counter(key for s in selected for key,count in s['failure_reasons'].items() for _ in range(count))
    axes[1].barh(list(failures),list(failures.values()));axes[1].set_xlabel('selected scan points')
    save(fig,'failure_reasons.png','REAL REPLAY / LIVE: missing and rejected requests are retained')
    fig,axes=plt.subplots(2,1,figsize=(12,8))
    for t in trials:
        group=[v for v in values if v['run_id']==t['run_id'] and v['pose_error_h_diagnostic'] is not None]
        for i,metric in enumerate(('h','v')):
            axes[i].plot([v['position'][0] for v in group],[v['pose_error_'+metric+'_diagnostic'] for v in group],'.',label=t['run_id']+' pose-time error')
            axes[i].plot([v['position'][0] for v in group],[v['hpl' if metric=='h' else 'vpl'] for v in group],'x',label=t['run_id']+' saved-reference PL (unpaired)')
            axes[i].set_ylabel(metric.upper()+' m');axes[i].set_xlabel('position x (m)');axes[i].legend(fontsize=6)
    save(fig,'actual_error_vs_pl.png','LIVE DIAGNOSTIC: pose-time errors / unpaired reference PL; NOT calibration coverage')
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for t in trials:
        root=Path(t['run_manifest']).parents[1];data=[r for p in (root/'profiling').glob('advisory_validation_*.csv') if '_current' not in p.stem for r in rows(p)]
        for i,key in enumerate(('preparation_s','query_total_s')):
            axes[i].plot(range(len(data)),[float(r[key])*1000 for r in data],'.',label=t['run_id']);axes[i].set_ylabel(key+' ms');axes[i].legend(fontsize=6)
    save(fig,'timing.png','REAL REPLAY: preparation and 100-query timing; offline CPU load')
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for t in trials:
        root=Path(t['run_manifest']).parents[1]
        data=[r for p in (root/'profiling').glob('planner_flow_*.csv') if '_export' not in p.stem for r in rows(p)]
        for i,key in enumerate(('total_s','prediction_prepare_s')):
            axes[i].plot(range(len(data)),[float(r[key])*1000 for r in data],'.',label=t['run_id'])
            axes[i].set_ylabel(key+' ms');axes[i].legend(fontsize=6)
    save(fig,'online_timing.png','LIVE: original planner total and predictor preparation timing')
    json_write(out/'summary.json',summary)
    rel=lambda p:os.path.relpath(p,out)
    text=['# 坐标契约与真实森林验证（2026-10-07）','',
      f'本轮独立探索运行 {len(trials)} 次；校准 0/9、独立验证 0/9、任务 ON/OFF 对照 0/6。没有推广默认参数。',
      '后验先验与规划引导均关闭；同一 GridMap、原阈值／净空／动力学／预算及最终曲线闸门保留。',
      '这些探索运行不是配对 seed 协议，不能当作三次独立校准或引导 A/B。','',
      '| 验收项 | 结果 |','|---|---|']
    for k,v in summary['results'].items():text.append('| '+k+' | '+v+' |')
    text+=['',f'完整输入录制 {summary["recorded"]}/{summary["requests"]}；合格同参考时刻误差配对 {summary["qualified_error_pairs"]}。',
      '转换闭合不等于物理对齐已验证：优化后的 map Up/ENU Up 漂移与旋转不确定性仍需处理。',
      '当前 GNSS 监测拒绝、完整 epoch 录制缺失和物理停车分别归因；工作区阻塞已解除，不能解释停车。',
      'GNSS raw、anchored、information-derived PL 分别命名；被拒绝来源的官方 PL 为空。',
      'LiDAR 逐观测残差尚未导出；ICP RMSE 不可替代。GNSS 后验逐测距残差已在新输入与因子 CSV 保存。',
      '冻结空间预测与到达时误差关系、95%联合经验覆盖、趋势块分析尚无合格试验，不发布数值保证。','',
      '旧重放 manifest 未登记 sibling receiver CSV；其 PL 仅列为未配对诊断。空间 CSV、矩阵和输入均校验原 manifest hash；不追认接收点 PL 为校准证据。','',
      '历史 ARAIM CSV 的 worst_hyp 行有额外列，假设身份统计置空；不移动列或用错误的零计数下结论。正确格式的 epoch 行及冻结输入仍表明 GNSS 当前监测拒绝。原日志保留，生产导出格式修复待完成。','',
      '[全部原始请求及时间/配对原因](error_pair_requests.csv) · [当次规划拒绝日志索引](planning_rejections.csv) · [summary.json](summary.json)','',
      '[弱方向原始数表](weak_directions.csv)','',
      '![真实输入、时间及方向](availability_time_coordinates.png)','',
      '![失败原因](failure_reasons.png)','',
      '![信息与正则化](weak_information.png)','',
      '![实际误差与未配对PL诊断](actual_error_vs_pl.png)','',
      '![离线真实重放耗时](timing.png)','']
    text+=['![现场在线耗时](online_timing.png)','',
      '三次版本不同，分别报告；不将跨时刻、跨地图输入当成同输入 A/B 或修正前后改善幅度。','']
    for t in trials:
        b=t['behavior'];text += [f'## {t["run_id"]} / `{t["revision"][:7]}`','',
          f'位置 {b["start"]} → {b["capture_end"]}；前进 {b["forward_progress_m"]:.3f} m；X 范围 {b["x_range"]}。',
          f'发布轨迹 ID {b["published_trajectory_ids"]}；位置命令实际收到 ID {b["commanded_trajectory_ids"]}。',
          f'独立 execution_instance_id {b["execution_instance_ids"]}；不能把发布数量当成接续或任务完成。',
          f'运动质量计数 {b["current_motion_quality_counts"]}；GNSS 监测可用性 {b["gnss_monitor_valid_counts"]}。',
          f'当次 A* 起终点拒绝 {t["astar_endpoint_failure_counts"]}。',
          f'[版本／配置／生命周期]({rel(t["run_manifest"])}) · [GNSS最坏假设]({rel(t["source_audit"]["araim_csv"])})。',
          f'ARAIM 有效 epoch {t["source_audit"]["epoch_rows"]}，GNSS 拒绝 {t["source_audit"]["gnss_rejected_epochs"]}；格式无效行 {t["source_audit"]["invalid_rows_by_type"]}。',
          '所保存 failure map 常仅覆盖该种失败的首次发生；最后停车若无同代数地图，根因保持未知，不使用早期地图替代。','']
        for s in (s for s in selected if s['run_id']==t['run_id']):
            text += [f'{s["tag"]}：有效 {s["valid"]}/{s["requested"]}，没有覆盖实际分叉时不得改名为分叉扫描。',
              f'HPL {s["hpl_range_m"]} m；VPL {s["vpl_range_m"]} m。',
              f'[原始点CSV]({rel(s["csv"])}) · [全部矩阵／数值状态]({rel(s["matrices"])})',
              f'![真实冻结空间层](spatial_{t["run_id"]}_{s["tag"]}.png)','']
    text+=['## 下一步与未完成项','',
      '优先定位 FGO 世界→ECEF 旋转的物理竖直对齐和不确定性；核对 GNSS 整星座不可监测与 Advisory 来源拒绝的语义。不能关闭故障检查代替修复。',
      '缩短或明确生产冻结参考—位姿时序，保存原时刻；不能改旧时间戳满足 0.05 s。',
      '用最新失败身份／物理冻结图解决合法起点到 lattice／guide 的连接、观测时效与曲线接续，固定路线到终点后才开展 9+9；再做 6 次对照。',
      '没有校准组噪声尺度、冻结参数文件和独立验证通过证据，本轮默认参数保持原值。']
    text+=['','## 实际执行与复现','',
      '现场需提交、干净工作树和 GPU 预检；使用当前生产二进制，重放校验 source/producer hash。记录时版本与本报告各运行版本一一对应。',
      '```bash','source /opt/ros/jazzy/setup.bash','source /home/dev/ws_iap/install/setup.bash',
      'ros2 launch iap iap_sim.launch.py scenario:=icra_dense_forest_four_fork_v2 start_rviz:=false start_grid_map_visualizer:=true advisory_posterior_prior:=false advisory_guidance:=false capture_failure_map:=true run_duration_s:=180',
      '# 使用 launch 打印的 IAP_RUN_DIR；下列命令为第三次运行实际执行命令',
      'export IAP_RUN_DIR=/home/dev/ws_iap/src/iap/log/20261007T040151Z_586','export ROS_LOG_DIR=$IAP_RUN_DIR/runtime/ros',
      'python3 scripts/dev_predictor/advisory_live_capture.py --duration 160 --label lattice_survey',
      'python3 scripts/dev_predictor/advisory_validation.py record --label lattice_survey --count 26 --interval 5 --timeout 5',
      'python3 scripts/dev_predictor/advisory_validation.py replay --binary /home/dev/ws_iap/build/ego_planner/advisory_validation --payload "$IAP_RUN_DIR/export/advisory/validation/recordings/lattice_survey_0001/input.bin" --label lattice_0001',
      '```','',
      '重放／报告标签不可覆盖已有产物；重复执行时选择新的诊断标签或 resolver 分配的新分析目录。',
      '原报告、阶段快照、失败请求和版本哈希均保留；旧输入缺失 epoch 的历史读取不会授予校准资格。']
    (out/'report.md').write_text('\n'.join(text)+'\n')
    hashes={str(p.relative_to(run)):digest(p) for p in out.iterdir() if p.is_file()}
    manifest(run,'advisory_forest_report_'+args.label,{'identity':summary['identity'],'trials':trials,'artifacts_sha256':hashes,'report_generator_sha256':digest(Path(__file__))})
    print(out);return 0

if __name__=='__main__':raise SystemExit(main(parser.parse_args()))
