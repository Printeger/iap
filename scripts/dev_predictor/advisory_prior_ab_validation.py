#!/usr/bin/env python3
"""Paired posterior-prior A/B using full frozen inputs and production Predictor."""
import argparse
import collections
import csv
import json
import os
from pathlib import Path
import sys

import numpy as np
import advisory_validation as common


def identity():
    value = common.source_identity()
    for path in ("scripts/dev_predictor/advisory_prior_ab_validation.py",
                 "src/iap/planner/plan_manage/include/ego_planner/prediction_input.h",
                 "launch/iap_sim.launch.py", "launch/_includes/full_stack_runtime.py"):
        value["source_sha256"][path] = common.sha(common.REPO / path)
    return value


def fmt(value):
    return "missing" if value is None else f"{value:.12g}"


def load(run, label):
    root = run / "export/advisory/validation"
    tables, matrices, inputs = {}, {}, {}
    for mode in ("on", "off"):
        directory = root / (label + "_" + mode)
        tables[mode] = {p.parent.name: common.rows(p) for p in sorted(directory.glob("*/points.csv"))}
        matrices[mode] = {p.parent.name: [json.loads(line) for line in p.read_text().splitlines()]
                          for p in sorted(directory.glob("*/matrices.jsonl"))}
        inputs[mode] = {p.parent.name: json.loads(p.read_text()) for p in sorted(directory.glob("*/input.json"))}
    if not tables["on"] or tables["on"].keys() != tables["off"].keys():
        raise ValueError("complete matching A/B campaigns required")
    return root, tables, matrices, inputs


def analyze(run, label):
    root, tables, matrices, inputs = load(run, label)
    pairs = {}
    for case in tables["on"]:
        on, off = tables["on"][case], tables["off"][case]
        if len(on) != len(off): raise ValueError("unpaired candidates")
        observation = root / (label + "_on") / case / "observation_input.bin"
        payload = root / (label + "_off") / case / "input.bin"
        if observation.exists():
            if common.sha(observation) != common.sha(payload):
                raise ValueError("A/B observation codec differs: " + case)
            equality = True
        else:
            if case != "S5_missing_physical": raise ValueError("missing paired input: " + case)
            equality = None
        for a, b in zip(on, off):
            for field in ("x", "y", "z", "ix", "iy", "iz", "generation", "reference_time_s",
                          "cloud_stamp_s", "pose_stamp_s", "current_stamp_s", "gnss_stamp_s",
                          "observed", "raw_occupied", "inflated_occupied", "gnss_used", "lidar_used",
                          "gnss_hpl", "gnss_vpl", "lidar_hpl", "lidar_vpl"):
                if a[field] != b[field]: raise ValueError(f"unpaired {case}/{field}")
            if b["prior_used"] != "0": raise ValueError("disabled group used a prior")
            for row in (a, b):
                if row["valid"] != "1" and (row["fused_hpl"] or row["fused_vpl"]):
                    raise ValueError("invalid result has numeric official PL")
                if row["valid"] == "1" and any(float(row[f"fused_{m}"]) <= 0 for m in ("hpl", "vpl")):
                    raise ValueError("valid zero or negative PL")
                for check in ("codec_equal", "wrapper_equal", "repeat_equal", "batch_equal"):
                    if row[check] not in ("", "1"): raise ValueError("replay consistency failed")
        if inputs["off"][case]["has_lambda_base"]:
            raise ValueError("OFF frozen input retained prior participation")
        pairs[case] = {"points": len(on), "observation_codec_equal": equality,
                       "observation_sha256": common.sha(observation) if equality else None,
                       "on_identity": inputs["on"][case]["prediction_input_identity"],
                       "off_identity": inputs["off"][case]["prediction_input_identity"]}
    spatial, availability, mechanism = {}, {}, {}
    for mode in ("on", "off"):
        spatial[mode] = {}
        for field in ("gnss_hpl", "gnss_vpl", "lidar_hpl", "lidar_vpl", "fused_hpl", "fused_vpl"):
            values = [float(r[field]) for r in tables[mode]["S0"] if r[field]]
            spatial[mode][field] = {"min": min(values) if values else None,
                                    "max": max(values) if values else None,
                                    "range_m": max(values)-min(values) if values else None}
        rows = [r for table in tables[mode].values() for r in table]
        availability[mode] = {"requests": len(rows), "valid": sum(r["valid"] == "1" for r in rows),
                              "reasons": dict(collections.Counter(r["status"]+":"+r["reason"] for r in rows))}
        mechanism[mode] = {}
        for group in ("S1", "S2", "S3", "S4"):
            sequence = [tables[mode][f"{group}_{i}"][0] for i in range(3)]
            mechanism[mode][group] = {}
            for metric in ("hpl", "vpl"):
                values = [float(r["fused_"+metric]) if r["fused_"+metric] else None for r in sequence]
                status = ("PASS" if all(v is not None for v in values) and
                          all(b + 1e-12 >= a for a, b in zip(values, values[1:])) else "FAIL")
                mechanism[mode][group][metric] = {"values": values, "monotonic_status": status}
    gain = {metric: spatial["off"]["fused_"+metric]["range_m"] /
                    spatial["on"]["fused_"+metric]["range_m"]
            if spatial["on"]["fused_"+metric]["range_m"] else None for metric in ("hpl", "vpl")}
    gate = tables["off"]["S5_missing_gnss"][0]
    real = tables["off"]["S0"][0]["identity"] == "REAL_REPLAY"
    return {"schema": "iap_advisory_posterior_ab_v1", "identity": "REAL_REPLAY" if real else "SYNTHETIC_MECHANISM",
            "default_advisory_posterior_prior": False, "pairs": pairs, "spatial": spatial,
            "spatial_range_gain": gain, "availability": availability, "mechanism": mechanism,
            "weak_prior_fraction_on": matrices["on"]["S3_0"][0]["weak_direction_prior_fraction"],
            "admission_discrepancy": {k: gate[k] for k in ("reason", "wrapper_called", "module_valid", "lidar_used", "fused_hpl")},
            "input_availability": "INCONCLUSIVE_INPUT_REVIEW_REQUIRED" if real else "INCONCLUSIVE_INPUT_UNAVAILABLE",
            "spatial_sensitivity": "INCONCLUSIVE_SPATIAL_CONTRAST" if real else "INCONCLUSIVE_INPUT_UNAVAILABLE",
            "actual_error_conformity": "INCONCLUSIVE_LIVE_NOT_RUN", "independent_live_runs": 0,
            "real_forest_snapshots": int(real), "all_map_validated": False}


def report(run, label):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import Normalize
    root, tables, matrices, inputs = load(run, label)
    summary = analyze(run, label)
    out = common.artifact(run, f"export/analysis/advisory_validation/{label}/report.md").parent
    if (out / "report.md").exists(): raise FileExistsError("report evidence already exists")
    preflight = run / "metadata/manifests/advisory_preflight.json"
    live = json.loads(preflight.read_text()) if preflight.exists() else {"live_status": "NOT_RUN"}
    summary["live_status"] = live["live_status"]
    common.json_write(out / "summary.json", summary)
    images = []
    figure_identity = summary["identity"] + "; counterfactuals are offline diagnostics"
    def save(fig, name, caption):
        fig.suptitle(figure_identity + "\n" + caption, fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, .92)); fig.savefig(out / name, dpi=150); plt.close(fig)
        images.append((name, caption))
    for metric, lo, hi in (("hpl", .25, .65), ("vpl", .20, .55)):
        fig, axes = plt.subplots(2, 4, figsize=(14, 7), sharex=True, sharey=True)
        for i, mode in enumerate(("on", "off")):
            rows = tables[mode]["S0"]
            for j, source in enumerate(("gnss", "lidar", "prior", "fused")):
                ax=axes[i,j]; field=source+"_"+metric
                valid = [r for r in rows if r[field]]; missing = [r for r in rows if not r[field]]
                if valid:
                    scatter=ax.scatter([float(r["x"]) for r in valid],[float(r["y"]) for r in valid],
                                       c=[float(r[field]) for r in valid],norm=Normalize(lo,hi),cmap="turbo",s=28)
                    fig.colorbar(scatter,ax=ax,extend="both",label="m")
                if missing: ax.scatter([float(r["x"]) for r in missing],[float(r["y"]) for r in missing],marker="x",color="purple",label="missing")
                ax.set_title(mode+" "+source+" "+metric.upper()); ax.set_aspect("equal")
        save(fig, "spatial_ab_"+metric+".png", "Same 100 voxel centers; fixed production color limits; missing = purple x")
    fig, axes=plt.subplots(1,2,figsize=(11,4))
    for metric,ax in zip(("hpl","vpl"),axes):
        for mode in ("on","off"):
            values=[float(r["fused_"+metric]) if r["fused_"+metric] else np.nan for r in tables[mode]["S0"]]
            ax.plot(values,label=mode)
        ax.set_yscale("log");ax.set_xlabel("paired voxel ID");ax.set_ylabel(metric.upper()+" m");ax.legend()
    save(fig,"spatial_raw_values.png","Raw values reveal contrast even when fixed color limits saturate")
    fig,axes=plt.subplots(2,3,figsize=(13,7))
    for j,group in enumerate(("S1","S2","S3")):
        for i,metric in enumerate(("hpl","vpl")):
            ax=axes[i,j]
            for mode in ("on","off"):
                values=summary["mechanism"][mode][group][metric]["values"]
                ax.plot([1,10,100], [v if v is not None else np.nan for v in values],"o-",label=mode)
            ax.set_xscale("log");ax.set_yscale("log");ax.set_title(group+" "+metric.upper());ax.set_xlabel("source noise multiplier");ax.set_ylabel("m");ax.legend()
    save(fig,"degradation_ab.png","S1 GNSS, S2 LiDAR, S3 both; no color criterion")
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for metric,ax in zip(("hpl","vpl"),axes):
        for mode in ("on","off"):
            ax.plot([1,.1,0],summary["mechanism"][mode]["S4"][metric]["values"],"o-",label=mode)
        ax.set_xlabel("diagnostic prior alpha");ax.set_ylabel(metric.upper()+" m");ax.legend()
    save(fig,"prior_ablation.png","S4 alpha is diagnostic only; OFF never re-enables the prior")
    cases=("S3_0","S3_2","S2_weak_normal_support","weak_lidar_only","S3_regularization_limit")
    fig,axes=plt.subplots(1,2,figsize=(13,4))
    for mode in ("on","off"):
        minimum=[]; fractions=[]
        for case in cases:
            m=matrices[mode][case][0]; info=m["fused_information"]
            eig=info["eigenvalues"][0] if info else np.nan; minimum.append(eig)
            fractions.append(m["fusion_epsilon"]/(eig+m["fusion_epsilon"]) if info and eig>=0 else np.nan)
        axes[0].plot(cases,minimum,"o-",label=mode);axes[1].plot(cases,fractions,"o-",label=mode)
    axes[0].set_yscale("symlog",linthresh=1e-8); axes[0].set_ylabel("min eigenvalue of observation + participating prior")
    axes[1].set_ylabel("epsilon / (min eigenvalue + epsilon)")
    for ax in axes: ax.tick_params(axis="x",rotation=20);ax.legend()
    save(fig,"weak_direction_regularization.png","Full source eigenvectors/matrices are in JSONL; epsilon unchanged")
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    names=list(tables["off"])
    for mode,offset in (("on",-.18),("off",.18)):
        rates=[sum(r["valid"]=="1" for r in tables[mode][k])/len(tables[mode][k]) for k in names]
        axes[0].barh(np.arange(len(names))+offset,rates,height=.35,label=mode)
    axes[0].set_yticks(range(len(names)),names,fontsize=5);axes[0].set_xlabel("official valid / all requests");axes[0].legend()
    rejects=collections.Counter(r["status"]+":"+r["reason"] for rows in tables["off"].values() for r in rows if r["valid"]!="1")
    axes[1].barh(list(rejects),list(rejects.values()));axes[1].tick_params(axis="y",labelsize=6)
    for field in ("pose_stamp_s","current_stamp_s","cloud_stamp_s","gnss_stamp_s"):
        age=[float(r["reference_time_s"])-float(r[field]) if r[field] else np.nan
             for table in tables["off"].values() for r in table]
        axes[2].plot(age,label=field.removesuffix("_stamp_s"))
    axes[2].set_ylabel("reference minus source stamp (s)");axes[2].set_xlabel("paired request including rejected stale inputs");axes[2].legend()
    save(fig,"availability_reasons_time.png","Availability uses wrapper result; direct diagnostics never replace missing PL")
    timings=[]
    for path in sorted((run/"profiling").glob("advisory_validation_"+label+"_*.csv")): timings.extend(common.rows(path))
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for mode in ("on","off"):
        values=[r for r in timings if r["label"].startswith(label+"_"+mode+"/")]
        axes[0].plot([float(r["preparation_s"]) for r in values],".-",label=mode)
        axes[1].plot([float(r["total_s"]) for r in values],".-",label=mode)
    for ax,y in zip(axes,("preparation s","total s including codec / repeated diagnostics")):ax.set_ylabel(y);ax.legend()
    save(fig,"timing_ab.png","Offline measurement; includes repeat/batch and codec checks")
    fig,ax=plt.subplots(figsize=(9,3));ax.axis("off")
    ax.text(.5,.5,f"LIVE MEASUREMENT: NOT RUN\n{summary['real_forest_snapshots']} forest inputs; 0 independent paired runs\nNo coordinate/extrinsic/time calibration evidence",ha="center",va="center",fontsize=13)
    save(fig,"actual_error_vs_pl.png","LIVE MEASUREMENT missing; no synthetic errors plotted")
    with (out/"ab_values.csv").open("x") as stream:
        writer=csv.writer(stream);writer.writerow(["mode","case","id","status","reason","prior_used","hpl","vpl"])
        for mode in ("on","off"):
            for case,rows in tables[mode].items():
                writer.writerows([mode,case,r["id"],r["status"],r["reason"],r["prior_used"],r["fused_hpl"],r["fused_vpl"]] for r in rows)
    def link(path): return os.path.relpath(path,out)
    lines=["# 关闭 Advisory 后验先验：同输入 A/B 实验报告","",
           f"模型/工具提交：`{common.git('rev-parse','HEAD')}`；场景目标：`{common.SCENARIO}`；现场状态：`{summary['live_status']}`。",
           "默认配置为基于观测条件的融合 Advisory。开关只控制 FGO 后验代理是否再次作为空间先验，Current Monitor / GLIO/FGO / 执行检查保留。",
           "",f"证据身份：**{summary['identity']}**。真实冻结输入数量 {summary['real_forest_snapshots']}；尚未完成森林 start/middle/stop 覆盖及实际 GLIO/真值配对，不宣称未来定位误差保证。",
           "","| 独立结论 | 状态 |","|---|---|"]
    lines += [f"| {title} | `{summary[key]}` |" for title,key in (("真实输入可用性","input_availability"),("真实空间敏感性","spatial_sensitivity"),("实际误差符合性","actual_error_conformity"))]
    lines += ["", "## 同输入与数值证据", "",
              "每份 ON 完整输入另导出仅清除 has_lambda_base/矩阵的 observation_input.bin；其 SHA256 必须等于对应 OFF input.bin。候选坐标、源 PL 与来源标志逐点相等。缺物理地图的请求不可编码，配对检查为 N/A，仍保留拒绝。",
              f"配对检查见 [summary.json](summary.json) 的 pairs；汇总原值见 [ab_values.csv](ab_values.csv)。OFF 全部输入 has_lambda_base=false，全部 prior_used=0。弱方向基线 ON 先验比例 `{fmt(summary['weak_prior_fraction_on'])}`。",
              "", "| metric | ON spatial range m | OFF spatial range m | range ratio OFF/ON |","|---|---:|---:|---:|"]
    for metric in ("hpl","vpl"):
        lines.append(f"| {metric.upper()} | {fmt(summary['spatial']['on']['fused_'+metric]['range_m'])} | {fmt(summary['spatial']['off']['fused_'+metric]['range_m'])} | {fmt(summary['spatial_range_gain'][metric])} |")
    lines += ["", "| group | requests | official valid | fraction |", "|---|---:|---:|---:|"]
    for mode in ("on","off"):
        availability=summary["availability"][mode]
        lines.append(f"| {mode} | {availability['requests']} | {availability['valid']} | {availability['valid']/availability['requests']:.6%} |")
    lines += ["", "合成空间差异恢复及双源退化响应增强是模型机制证据；幅度不等于森林测量，也不证明 PL 数值尺度已经校准。单源补偿要看保留来源的信息和弱方向，不能要求每次退化都变红。", "",
              "| mode | S1 HPL (noise 1/10/100) | S2 HPL | S3 HPL | S3 VPL |","|---|---|---|---|---|"]
    for mode in ("on","off"):
        cells=[summary["mechanism"][mode][group][metric]["values"] for group,metric in (("S1","hpl"),("S2","hpl"),("S3","hpl"),("S3","vpl"))]
        lines.append("| "+mode+" | "+" | ".join(", ".join(f"{v:.10g}" if v is not None else "missing" for v in values) for values in cells)+" |")
    lines += ["", "## 仍存在的问题与下一步", "",
              f"缺 GNSS 的共享准入结果：[OFF 请求]({link(root/(label+'_off')/'S5_missing_gnss/points.csv')}) 为 `{summary['admission_discrepancy']}`。包装器与模块共同使用合法 LiDAR；来源时间与身份检查保留。",
              "Fusion 在求逆前检查未正则化联合矩阵，弱方向正则化占比超过 1% 时官方 PL 留空。弱法向过滤保留 PCA 法向的小分量，不能自动称为严格秩亏；以导出的最小特征值为准。极端双源噪声 ×1e6 的有限正则化解仅作为诊断，不是有效 PL。GNSS 存在 anchor/raw PL 与用于融合 FIM 的不同尺度，K_H/K_V=5、PCA 默认和固定噪声未经实际误差校准；PL 增大能反映观测变弱，但尚不能判定其绝对尺度正确。",
              "关闭此项后，FGO 后验作为 lambda_prior 再加入的这条重复信息路径已移除。GNSS 和 LiDAR 观测之间仍未建模相关性；具体因子重叠、姿态/平移耦合及实际误差需要真实输入核验。",
              "建议下一步先修融合模式的来源准入一致性，保留新鲜度与环境有效性；再明确弱方向秩判定和正则化语义；最后在固定坐标/外参/时间契约下完成至少三次配对运行以校准尺度。不得向预测器反馈真值。", "",
              "CPU 墙面规划夹具还暴露路径恢复问题：默认 OFF 时 Advisory 恢复将实际曲线推出地图，最终物理检查拒绝，未生成待发布轨迹。原成功曲线测试显式保留旧 ON 输入假设，另加 OFF 拒绝回归；没有放宽预算、代价或物理检查。该行为是规划覆盖风险，不能据机制实验宣称真实森林能正常完成路线。证据在 runtime/ros/prior_ab_tests.log 的首次失败及后续通过日志。", "",
              "## 图表", ""]
    for case in ("S2_weak_normal_support","weak_lidar_only","S3_regularization_limit"):
        row=tables["off"][case][0]; matrix=matrices["off"][case][0]; info=matrix["fused_information"]
        eigen=info["eigenvalues"][0] if info else None; epsilon=matrix["fusion_epsilon"]
        share=epsilon/(eigen+epsilon) if eigen is not None and eigen>=0 else None
        lines.insert(lines.index("## 图表"), f"- OFF {case}: 模块 HPL={row['module_hpl'] or 'missing'} m、VPL={row['module_vpl'] or 'missing'} m，min eigenvalue={fmt(eigen)}，epsilon 弱方向比例={fmt(share)}，GNSS/LiDAR used={row['gnss_used']}/{row['lidar_used']}；原因 `{row['reason']}`。")
    lines += [f"![{caption}]({name})\n" for name,caption in images]
    lines += ["## 原始输入、矩阵和失败原因", ""]
    for case in tables["on"]:
        lines.append(f"- {case}: "+"；".join(f"{mode} [CSV]({link(root/(label+'_'+mode)/case/'points.csv')}) [矩阵/弱方向]({link(root/(label+'_'+mode)/case/'matrices.jsonl')}) [输入身份]({link(root/(label+'_'+mode)/case/'input.json')})" for mode in ("on","off")))
    lines += ["", (f"预检：[JSON]({link(preflight)})。" if preflight.exists() else "现场预检未执行。") + "所有有效/无效/未调用结果均保留；耗时原表在 profiling/advisory_validation_*.csv；完整输入、源码、二进制及所有产物 hash 登记在本轮 manifest。",
              "", "## 执行与未完成项", "",
              "已运行同输入 S0–S5 A/B、完整 codec/包装器/批次一致性、来源与弱方向、先验消融和正则化诊断。真实三份扫描、固定物理合法路线、配对 seed 的至少三次探索运行、坐标/外参/时间现场核验及实际误差符合性待测。没有启动现场，没有修改或搬动用户文件来取得干净树。",
              "", "实际执行命令见本轮 metadata/manifests/advisory_prior_ab_*.json 和 runtime/ros 日志。重新复现时先 source ROS/workspace，取消 IAP_RUN_DIR 让 resolver 分配新目录；同名结果拒绝覆盖。", "",
              "```bash", "cd /home/dev/ws_iap", "source /opt/ros/jazzy/setup.bash", "source install/setup.bash",
              "python3 src/iap/scripts/dev_predictor/advisory_prior_ab_validation.py fixture --binary build/ego_planner/advisory_validation --label committed_ab", "```", ""]
    error_summary=run/"export/analysis/advisory_validation/error_summary.json"
    error_requests=run/"export/advisory/validation/error_requests.csv"
    if error_summary.exists() and error_requests.exists():
        lines += [f"实际误差请求：[CSV]({link(error_requests)})、[状态 JSON]({link(error_summary)})；无现场输入时仅有表头，不能解读为零误差。", ""]
    (out/"report.md").write_text("\n".join(lines),encoding="utf-8")
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode",choices=("fixture","replay","report"))
    parser.add_argument("--binary",type=Path)
    parser.add_argument("--payload",type=Path)
    parser.add_argument("--label",default="committed_ab")
    args=parser.parse_args();common.safe_label(args.label)
    owner=not os.environ.get("IAP_RUN_DIR")
    run=(common.resolve_run_directory(entrypoint="advisory_prior_ab_validation",scenario=common.SCENARIO)
         if owner else common.adopt_run_directory(os.environ["IAP_RUN_DIR"]))
    os.environ["IAP_RUN_DIR"]=str(run);os.environ["ROS_LOG_DIR"]=str(run/"runtime/ros")
    try:
        version=identity();binary=None
        if args.mode!="report":
            if args.binary is None: parser.error("--binary required")
            binary=common.binary_identity(args.binary.resolve())
            command=["fixture_ab",args.label]
            if args.mode=="replay":
                if args.payload is None: parser.error("--payload required")
                recorded=common.validate_record(args.payload,args.payload.with_name("record.json"))
                for path,h in recorded["producer_binary"]["libraries_sha256"].items():
                    if path in binary["libraries_sha256"] and binary["libraries_sha256"][path]!=h:
                        raise ValueError("producer/replay shared library mismatch")
                command=["replay_ab",args.label,str(args.payload)]
            common.backend(run,args.binary.resolve(),command,"prior_ab_"+args.label)
            if args.mode=="replay":
                meta=json.loads((run/"export/advisory/validation"/(args.label+"_off")/"S0/input.json").read_text())
                if any(meta[k]!=recorded[k] for k in ("frame_id","geometry_id","generation")):
                    raise ValueError("service/payload geometry identity mismatch")
        output=report(run,args.label)
        paths=[p for mode in ("on","off") for p in (run/"export/advisory/validation"/(args.label+"_"+mode)).rglob("*") if p.is_file()]
        paths += [p for p in output.rglob("*") if p.is_file()]
        paths += list((run/"profiling").glob("advisory_validation_"+args.label+"_*.csv"))
        common.manifest(run,"advisory_prior_ab_"+args.label,{**version,"binary":binary,"command":sys.argv,
                        "identity":json.loads((output/"summary.json").read_text())["identity"],"codec":common.CODEC,
                        "artifacts_sha256":{str(p.relative_to(run)):common.sha(p) for p in sorted(paths)}},owner)
        if owner:common.finalize_run(run,lifecycle="completed",safety_outcome="not_applicable")
        print(output)
    except Exception:
        if owner:common.finalize_run(run,lifecycle="failed",safety_outcome="unknown")
        raise


if __name__=="__main__": main()
