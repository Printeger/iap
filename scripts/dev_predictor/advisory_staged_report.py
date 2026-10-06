#!/usr/bin/env python3
"""Join staged evidence; missing forest/error data cannot become a mechanism PASS."""
import argparse
import csv
import json
import os
import math
from pathlib import Path
import numpy as np
import advisory_validation as common
import advisory_prior_ab_validation as ab


def regression_sources():
    roots=("src/iap/planner","src/iap/predictor","include/iap/predictor","launch","scripts/dev_predictor")
    return {str(p.relative_to(common.REPO)):common.sha(p)
            for root in roots for p in sorted((common.REPO/root).rglob("*"))
            if p.is_file() and p.suffix in (".cpp",".h",".hpp",".py")}


def numerical_quality(tables):
    answer = {}
    for mode in ("on", "off"):
        number=lambda x:float(x) if x else float("nan")
        values = {m: [number(tables[mode]["epsilon_"+str(i)][0]["fused_"+m]) for i in range(3)]
                  for m in ("hpl", "vpl")}
        usable=all(tables[mode]["epsilon_"+str(i)][0]["valid"]=="1" for i in range(3))
        usable=usable and all(math.isfinite(x) and x>0 for v in values.values() for x in v)
        changes = {m: (max(v)-min(v))/v[1] if usable else None for m,v in values.items()}
        safe_values={m:[x if math.isfinite(x) else None for x in v] for m,v in values.items()}
        answer[mode] = {"epsilon_factors": [.1,1.,10.], "values": safe_values,
                        "relative_change": changes, "status": "PASS" if usable and max(changes.values()) <= .05 else "FAIL"}
    return answer


def sampling(run):
    root = run/"export/advisory/validation"
    before = root/"density_before_no_prior/sampling.csv"
    after = root/"density_final/sampling.csv"
    if not before.exists() or not after.exists(): return {"status": "INCONCLUSIVE_MISSING_SAMPLING"}, []
    b = {r["case"]:r for r in common.rows(before)}; a={r["case"]:r for r in common.rows(after)}
    primitive_b=before.with_name("primitives.csv"); primitive_a=after.with_name("primitives.csv")
    if common.sha(primitive_b)!=common.sha(primitive_a): raise ValueError("sampling support differs")
    metrics=("hpl","vpl")
    available=all(r["valid"]=="1" and r["identity"]=="SYNTHETIC_MECHANISM" and
                  all(math.isfinite(float(r[m])) and float(r[m])>0 for m in metrics) for r in a.values())
    if not available: return {"status":"FAIL_INVALID_SAMPLING_RESULTS"},[]
    duplicate=max(abs(float(a["duplicates_"+str(i)][m])/float(a["duplicates_1"][m])-1) for i in (2,4) for m in metrics)
    density=max(abs(float(a["density_"+str(i)][m])/float(a["density_2"][m])-1) for i in (4,8) for m in metrics)
    coefficients=("xx","xy","xz","yx","yy","yz","zx","zy","zz")
    if not all(math.isfinite(float(r[m])) for r in a.values() for m in coefficients):
        return {"status":"FAIL_NONFINITE_SAMPLING_INFORMATION"},[]
    matrix=np.asarray([float(a["duplicates_1"][m]) for m in coefficients])
    matrix_change=max(float(np.max(np.abs(np.asarray([float(a["duplicates_"+str(i)][m]) for m in coefficients])-matrix)))
                      /max(1.,float(np.max(np.abs(matrix)))) for i in (2,4))
    return {"identity":"SYNTHETIC_MECHANISM", "duplicate_relative_change":duplicate,
            "surface_density_relative_change":density, "duplicate_tolerance":1e-12,"density_tolerance":.05,
            "duplicate_information_relative_change":matrix_change,
            "status":"PASS" if duplicate<=1e-12 and matrix_change<=1e-12 and density<=.05 else "FAIL",
            "support_sha256":common.sha(primitive_a),
            "before_provenance":"precommit diagnostic probe; original accumulating model",
            "raw": [str(p.relative_to(run)) for p in (before,after,primitive_b,primitive_a)]}, [(k,float(b[k]["hpl"]),float(a[k]["hpl"])) for k in a]


def source_audit():
    paths=["src/iap/planner/plan_manage/src/planner_risk.cpp",
           "src/iap/gnss/pseudorange_factor.cpp", "src/iap/predictor/gnss_advisory_predictor.cpp",
           "src/iap/predictor/lidar_observability_fim.cpp", "src/iap/predictor/fusion_advisory_predictor.cpp",
           "launch/_includes/profile_runtime.py"]
    return {"source_sha256":{p:common.sha(common.REPO/p) for p in paths},
            "position_information_units":"m^-2; covariance m^2; PL m",
            "common_conversion":"K_H sqrt(lambda_max(C_xy)), K_V sqrt(C_zz), named bias/reserve",
            "gnss_direction":"azimuth/elevation ENU; single scalar clock Schur eliminated without pseudo prior",
            "lidar_direction":"map-frame normal; orientation/map/surface confidence conditioned",
            "fgo_model":"Pose3 antenna lever arm, world/ECEF rotation and clock state retained internally",
            "legacy_pl":"GNSS raw hypothesis, monitor anchored, and information-derived PL are distinct",
            "shared_posterior_prior_default":False,
            "unresolved":["frozen input lacks measured world-to-ENU rotation/extrinsic proof",
                          "conditioned position information differs from FGO marginal state uncertainty",
                          "remaining within-surface and cross-source correlation not empirically calibrated"],
            "semantics_status":"FAIL_INCOMPLETE_FRAME_CONTRACT",
            "calibration_status":"INCONCLUSIVE_LIVE_NOT_RUN",
            "canonical_alignment":"static planner translation; align_planner_odom_to_truth=False"}


def report(run, campaign, label):
    common.safe_label(label); common.safe_label(campaign)
    summary=ab.analyze(run,campaign); _,tables,matrices,inputs=ab.load(run,campaign)
    quality=numerical_quality(tables); density,density_rows=sampling(run)
    out=common.artifact(run,"export/analysis/advisory_validation/"+label+"/report.md").parent
    if (out/"report.md").exists(): raise FileExistsError("staged report already exists")
    preflight=run/"metadata/manifests/advisory_preflight.json"
    flight=json.loads(preflight.read_text()) if preflight.exists() else {"live_status":"PENDING_PREFLIGHT"}
    audit=source_audit()
    checks_path=run/"metadata/manifests/advisory_stage_checks.json"
    checks=json.loads(checks_path.read_text()) if checks_path.exists() else {"checks":[]}
    passed=set()
    current_sources=regression_sources()
    required_binaries={"prediction_contracts":{"test_predictor_module","test_lidar_observability_fim",
                        "test_rolling_spatial_advisory_window","advisory_validation","libiap.so"},
                       # bspline_opt is statically linked into both executables.
                       "planning_contracts":{"test_ego_baseline","ego_planner_node","libplan_env.so"}}
    for check in checks["checks"]:
        log=run/check["log"]
        if common.sha(log)!=check["sha256"]: raise ValueError("check evidence hash mismatch")
        sources=check.get("source_sha256",{})
        binaries=check.get("binary_sha256",{})
        bound=required_binaries.get(check["name"],set())<={Path(p).name for p in binaries}
        if (check["exit_code"]==0 and bound and current_sources.keys()<=sources.keys() and binaries and
                all(common.sha(common.REPO/p)==h for p,h in sources.items()) and
                all(common.sha(Path(p))==h for p,h in binaries.items())):
            passed.add(check["name"])
    if checks.get("revision")!=common.source_identity()["revision"]: passed.clear()
    statuses={"admission_and_numerical_state":"PASS_CPU_REGRESSION" if "prediction_contracts" in passed else "INCONCLUSIVE_MISSING_REGRESSION",
              "source_definitions":audit["semantics_status"], "sampling_stability":density["status"],
              "real_spatial_sensitivity":"INCONCLUSIVE_INPUT_UNAVAILABLE",
              "actual_error_empirical_conformity":"INCONCLUSIVE_LIVE_NOT_RUN",
              "curve_publication_continuation":"PASS_CPU_FROZEN_CLOCK_REGRESSION" if "planning_contracts" in passed else "INCONCLUSIVE_MISSING_REGRESSION",
              "forest_mission_completion":"INCONCLUSIVE_LIVE_NOT_RUN"}
    joined={"schema":"iap_advisory_staged_validation_v1",**common.source_identity(),
            "campaign":campaign,"identity":summary["identity"],"statuses":statuses,
            "numerical_quality":quality,"sampling":density,"source_audit":audit,"ab":summary,
            "preflight":flight,"checks":checks,"default_calibration_promoted":False,"independent_live_runs":0,
            "stages_completed":"CPU code/contracts and synthetic diagnostics",
            "stages_pending":["three real forest recordings","frame/extrinsic/time proof",
                              "physically checked reference route and live observation seed/schedule evidence",
                              "9 calibration + 9 independent validation runs","6 paired full mission runs"]}
    common.json_write(out/"summary.json",joined); common.json_write(out/"source_audit.json",audit)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if density_rows:
        fig,axis=plt.subplots(figsize=(10,4)); x=np.arange(len(density_rows))
        axis.plot(x,[r[1] for r in density_rows],"o-",label="accumulating model (diagnostic before)")
        axis.plot(x,[r[2] for r in density_rows],"o-",label="grouped support (committed after)")
        axis.set_xticks(x,[r[0] for r in density_rows],rotation=15); axis.set_ylabel("HPL (m)")
        axis.set_title("SYNTHETIC MECHANISM: duplicate and same-surface density")
        axis.legend(); fig.tight_layout(); fig.savefig(out/"sampling_density.png",dpi=150);plt.close(fig)
    fig,axis=plt.subplots(); factors=[.1,1.,10.]
    for mode in ("on","off"):
        for metric in ("hpl","vpl"):
            axis.plot(factors,quality[mode]["values"][metric],"o-",label=mode+" "+metric)
    axis.set_xscale("log");axis.set_xlabel("fusion epsilon factor");axis.set_ylabel("PL (m)")
    axis.set_title("SYNTHETIC MECHANISM: numerical stability (5% criterion)");axis.legend()
    fig.tight_layout();fig.savefig(out/"epsilon_stability.png",dpi=150);plt.close(fig)
    relative=lambda p:os.path.relpath(run/p,out)
    link=lambda label,p:f"[{label}]({relative(p)})"
    base=Path("export/analysis/advisory_validation")/campaign
    lines=["# Advisory 分阶段验证正式报告", "", "本轮完成准入／数值修正、相关采样修正、实际曲线约束及离线校准与对照工具。",
           "真实森林、误差校准与完整任务没有执行，不能以机制结果替代验收。", "", "## 独立结论", "",
           "| 项目 | 状态 |", "|---|---|"]+[f"| {k} | {v} |" for k,v in statuses.items()]
    lines += ["", "工作区／GPU："+flight["live_status"]+"；真实独立运行 0。默认后验代理关闭，默认噪声／PL 换算未经实测推广。",
              "", "## 改善与仍存在的问题", "",
              "统一来源准入允许合法单源，并独立保留当前运动质量。联合 PSD／秩判定在 epsilon 前完成；弱方向正则化占比上限 1%，退化官方 PL 为空。",
              f"同输入 A/B 的合成 HPL 空间跨度：ON={summary['spatial']['on']['fused_hpl']['range_m']:.9g} m，OFF={summary['spatial']['off']['fused_hpl']['range_m']:.9g} m。",
              f"OFF 双源噪声 ×1/10/100 的 HPL={summary['mechanism']['off']['S3']['hpl']['values']}；单源退化允许另一来源补偿，不以变红为标准。",
              "GNSS raw／anchored／information-derived 数值分别命名。前两者使用原假设/当前监测语义；只有同定义的信息派生值适合与融合矩阵比较。",
              "GNSS 虚拟时钟 epsilon 已移除；共同秩亏／正则化主导不能产生正常有限官方 PL。LiDAR 支持分组修复重复点累加，但不能证明所有样本或两来源统计独立。",
              "GNSS ENU 与 LiDAR map 同帧证据仍缺：冻结输入没有估计器世界系到 ENU 旋转／外参证明。先完成该契约，再做来源残差噪声与独立误差校准；不能靠尺度拟合修正坐标错误。",
              "FGO 后验代理默认不再加入，避免这项共享强后验掩盖／重复计入；FGO 内部历史与先验保留，剩余条件化和相关性仍待实证。",
              "", "## 同输入实验与图表", "",
              "下列地图均为 SYNTHETIC_MECHANISM，完整物理图、候选点、参考时刻、来源规则和参数相同，A/B 只切换先验。不是森林真实扫描。",
              link("完整 A/B 报告",base/"report.md"), ""]
    for name in ("spatial_ab_hpl","spatial_ab_vpl","availability_reasons_time","degradation_ab",
                 "prior_ablation","weak_direction_regularization","spatial_raw_values","timing_ab","actual_error_vs_pl"):
        lines += [f"![{name}]({relative(base/(name+'.png'))})", ""]
    if density_rows:
        lines += ["![SYNTHETIC sampling](sampling_density.png)","",
                  f"精确重复 ×2/4 相对变化 {density['duplicate_relative_change']:.9g}；同覆盖表面密度相对变化 {density['surface_density_relative_change']*100:.6g}%（预定 5%）。",
                  "before 为带完整原始数值的预提交诊断，after 为提交模型；primitives 内容 hash 相同。该结果不证明真实残差米数已校准。", ""]
    lines += ["![SYNTHETIC epsilon](epsilon_stability.png)","",
              "epsilon ×0.1/1/10 正常有效 HPL/VPL 最大相对变化："+str({k:v["relative_change"] for k,v in quality.items()})+"；标准固定 ≤5%。",
              "", "## 曲线、校准和现场", "",
              "弱墙 OFF 的正向验收要求合法曲线、最新走廊发布闸门、模拟执行 ID 和一次固定 P/V/A 接续；执行状态见独立结论及绑定版本的测试日志。真实 ROS 接续未验收。",
              "软边界成本贯通初始化、rebound、refine；独立实际 cubic 极值检查拒绝采样间越界。物理／运动／发布规则及共享计算预算保留。",
              "规划引导开关默认开启；关闭仍计算、导出和显示预测，仅取消选路／修补作用。CPU 对照保持原始诊断，未知有限代价与仅 Advisory 不急停有回归。",
              "校准协议预声明三条件，每条件 3 校准 + 3 heldout；完整任务 guidance OFF/ON 各 3，map seed=41021。路线／物理合法性、观测 seed 注入与恒定单源退化已接入，但尚未获得真实证据。",
              "校准先以来源测量残差标定噪声，重放后冻结共同换算；工具核对完整请求清单、clean run、独立 seed/运行与 hash，拒绝重复 CSV 和删除失败请求。",
              "95% 为水平与垂直同时满足的经验比例，按独立运行报告；趋势按 5 s 块与 block bootstrap。冻结 tau=0 空间查询与后来到达误差分别报告，不授予未来误差保证。",
              "实际误差图明确显示 NOT RUN，没有插值成低风险。当前只定位为实验性路线风险指标。", "", "## 原始证据与复现", "",
              link("summary.json",Path("export/analysis/advisory_validation")/label/"summary.json"),
              link("来源审计 JSON",Path("export/analysis/advisory_validation")/label/"source_audit.json"),
              link("A/B 原始数值表",base/"ab_values.csv")]
    for p in density.get("raw",[]):lines.append(link(p,Path(p)))
    raw=Path("export/advisory/validation")
    for mode in ("on","off"):
        for case in ("S0","S0_current","S1_2","S2_2","S3_2","S3_regularization_limit","S5_missing_gnss","S5_both_missing"):
            for name in ("points.csv","matrices.jsonl","input.bin","input.json"):
                lines.append(link(mode+"/"+case+"/"+name,raw/(campaign+"_"+mode)/case/name))
    for p in ("metadata/manifests/advisory_preflight.json","export/analysis/advisory_validation/staged_final/protocol.json"):
        if (run/p).exists():lines.append(link(p,Path(p)))
    lines += ["", "已经执行的命令与退出状态见 runtime/ros；本轮产物禁止覆盖，复跑需由 resolver 分配新 run 或使用新 label。", "",
              "```bash", "source /opt/ros/jazzy/setup.bash", "source /home/dev/ws_iap/install/setup.bash",
              f"export IAP_RUN_DIR={run}", 'export ROS_LOG_DIR="$IAP_RUN_DIR/runtime/ros"',
              f"python3 src/iap/scripts/dev_predictor/advisory_prior_ab_validation.py fixture --binary build/ego_planner/advisory_validation --label {campaign}",
              "build/ego_planner/advisory_validation diagnostics density_final",
              "python3 src/iap/scripts/dev_predictor/advisory_calibration.py protocol --label staged_final",
              "python3 src/iap/scripts/dev_predictor/test_advisory_calibration.py",
              "python3 src/iap/scripts/dev_predictor/advisory_validation.py preflight",
              f"python3 src/iap/scripts/dev_predictor/advisory_staged_report.py --campaign {campaign} --label {label}", "```", "",
              "本轮未执行 canonical 森林启动、真实扫描、噪声 fit／heldout CLI 或任务对照。旧解析图和日志保持原身份，未计为本轮真实数据。"]
    if checks_path.exists():
        lines += ["",link("已执行检查、退出状态与日志 hash",Path("metadata/manifests/advisory_stage_checks.json"))]
        for check in checks["checks"]:
            lines += ["",link(check["name"],Path(check["log"])),"", "```bash", check["command"], "```"]
    (out/"report.md").write_text("\n".join(lines)+"\n")
    common.manifest(run,"advisory_staged_"+label,{"command":__import__("sys").argv,
        "source":common.source_identity(),"artifacts_sha256":{str(p.relative_to(run)):common.sha(p) for p in out.iterdir() if p.is_file()}})
    return out


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--campaign",required=True)
    parser.add_argument("--label",default="staged");args=parser.parse_args()
    print(report(common.adopt_run_directory(os.environ["IAP_RUN_DIR"]),args.campaign,args.label))
