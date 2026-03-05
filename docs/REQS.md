# IAP Requirements Checklist (docs/REQS.md)

> Purpose: 把 talk《Integrity-Aware Active Perception》的“优化版 pipeline”拆成可实现、可验收、可追溯的工程需求。
> Scope: 仅修改本仓库 src/iap；禁止修改 ../glim。

## 0. 术语与约定
- Estimator: 滑窗/因子图估计器（紧耦合 GNSS+IMU+LiDAR）
- Integrity: 完整性监测输出 PL/AL/IM，安全条件 PL < AL
- PL: Protection Level（对位置误差的保守上界）
- AL: Alert Limit（由障碍接近度动态给出）
- IM = AL - PL（安全裕度）
- Mode: NOMINAL / CAUTION / SEARCH（由 IM 触发）
- GNSS: pseudorange + doppler（含 receiver clock bias/drift）
- Trunk: 树干几何地标（圆柱模型）

## 1. Repo/Build/Dev（必须先做）
### IAP-RQ-000: Repo guardrails
- [ ] 本仓库内加入 AGENTS.md（禁止改 ../glim，强制维护 CHANGES/TRACEABILITY）
- [ ] 加入 docs/CHANGES.md、docs/TRACEABILITY.md、docs/REQS.md（本文件）
- [ ] 加入 doc-guard：改代码必须同步更新 CHANGES + TRACEABILITY（git hook + CI）

Acceptance:
- 能在本地提交前被 hook 拦截；PR 中被 CI 拦截。

### IAP-RQ-001: Rename ROS2 package name to "iap"
- [x] package.xml 的 `<name>` 改为 `iap`
- [x] CMakeLists.txt 的 `project(...)` 改为 `iap`
- [x] 所有依赖此包名的路径/launch/ament 导出同步更新
- [x] 编译命令 `colcon build --packages-select iap` 可用
- [x] 阅读代码，/src/iap/docs/spec/ 文件夹中的内容，并对比哪些是代码中已经实现的，哪些是等待实现的，列成表格，整理记录为一个readme文件，供更新后续REQ。→ 见 `docs/SPEC_VS_IMPL.md`

Acceptance:
- colcon 能成功构建 `iap`，且 `ros2 pkg list | grep iap` 可见。生成readme

### IAP-RQ-002: Build artifacts
- [ ] 生成 compile_commands.json（给 clangd）
- [ ] 提供最小 demo 运行方式（README 或 apps/ 下可执行）

Acceptance:
- VSCode clangd 无红线；demo 可运行并产生日志。

---

## 2. Estimator（紧耦合 FGO / 滑窗）
### IAP-RQ-010: State definition matches talk
- [ ] 状态包含：p,v,q, ba,bg, clk_bias, clk_drift
- [ ] 状态时序与滑窗边缘化可运行

Acceptance:
- 日志打印 state 的关键量（p,v,clk/clk_dot）。

### IAP-RQ-020: GNSS measurement model (pseudorange + doppler)
- [ ] 实现伪距因子：含 receiver clock bias
- [ ] 实现多普勒因子：含 receiver clock drift + 速度投影
- [ ] 每颗卫星观测作为独立观测通道（便于 per-sat integrity）

Acceptance:
- 日志输出每颗卫星 residual_pr/residual_dop；可启用/禁用 GNSS 因子对比。

### IAP-RQ-030: IMU preintegration + sliding window
- [ ] IMU 预积分因子可用
- [ ] bias 作为状态可估计
- [ ] 滑窗边缘化稳定（不数值炸）

Acceptance:
- 轨迹连续且残差统计合理；运行不崩。

### IAP-RQ-040: LiDAR odometry / mapping factor baseline
- [ ] 最小版本：scan-to-map ICP 相对位姿因子
- [ ] 输出 ICP quality report（inlier/rmse/cond 等）

Acceptance:
- ICP 因子可开关；质量指标能打点输出。

---

## 3. Trunk geometric landmark（树干地标 + TDOP）
### IAP-RQ-100: Trunk detection & parameterization
- [ ] LiDAR 分割树干并拟合圆柱/圆（中心、半径）
- [ ] 提供 detection confidence（用于 trunk fault prior）

Acceptance:
- 在日志/可视化中能看到 trunk 数量 K、半径分布、置信度。

### IAP-RQ-110: Trunk factor in FGO (optional baseline → full)
- [ ] Baseline-A: trunk 仅用于健康度/噪声膨胀（不入图）
- [ ] Full-B: trunk 作为几何观测因子入图（可用于降低 Σp）

Acceptance:
- Full-B 开启时，Σp/PL 明显优于仅 GNSS+IMU。

### IAP-RQ-120: TDOP metric (Tree DOP)
- [ ] 计算 TDOP 或其代理（角度多样性/几何强度）
- [ ] TDOP 与 trunk 分布可视化/日志输出

Acceptance:
- 场景变化（树更分散/更集中）时 TDOP 有明显变化。

---

## 4. Integrity monitoring（ARAIM-ish → full ARAIM）
### IAP-RQ-200: Integrity outputs (PL/AL/IM) + mode
- [ ] 输出 PL、AL、IM 与 mode
- [ ] 安全条件：PL < AL

Acceptance:
- PL/AL/IM 曲线可画；mode 切换可复现。

### IAP-RQ-210: Alert Limit AL from obstacle proximity
- [ ] AL 随障碍距离动态变化（至少实现 HAL 近似）
- [ ] 与无人机半径/树干半径相关

Acceptance:
- 越靠近障碍，AL 越小；日志可见 HAL/VAL/AL。

### IAP-RQ-220: GNSS per-satellite NIS gating (RAIM-ish baseline)
- [ ] per-sat NIS 统计（pr/dop/joint）
- [ ] downweight（gamma_R）与 exclude_sats 输出
- [ ] global_nis 超阈值触发 FDE（贪心即可）

Acceptance:
- 注入某颗卫星 bias 时能被降权/剔除，PL/IM 改善。

### IAP-RQ-230: Fault hypothesis set includes trunks (full ARAIM)
- [ ] fault hypotheses: H0 + satellite faults + constellation faults(可选) + trunk faults
- [ ] trunk fault prior 由 detection confidence 给出

Acceptance:
- 日志可见 N_f = N + C + K；PL 按 worst-case hypothesis 输出。

### IAP-RQ-240: ARAIM PL computation
- [ ] fault-free + faulted 模式下计算 σ_q,k 与 PL_q,k
- [ ] 输出 HPL/VPL（至少 HPL）

Acceptance:
- PL 的变化符合直觉：可见卫星少/几何差/树干退化 → PL 上升。

---

## 5. Prediction for planning（候选轨迹未来 PL_pred）
### IAP-RQ-300: Candidate trajectory generator
- [ ] Baseline: motion primitives（v/ω/altitude 离散）
- [ ] 可扩展: spline/MINCO（后续）

Acceptance:
- 每次规划生成 M 条候选轨迹（含时间戳点序列）。

### IAP-RQ-310: Predict visibility sets along τ
- [ ] 预测卫星可见集合 \hat{V}(τ)（简化：仰角阈值/遮挡模型）
- [ ] 预测 trunk 可观测集合 \hat{O}(τ)（FOV/距离/遮挡）

Acceptance:
- 同一时刻不同轨迹预测的 |V|、|O| 不同且合理。

### IAP-RQ-320: Information propagation → Σ_pred → PL_pred
- [ ] 传播信息矩阵/协方差得到 Σ_pred
- [ ] Baseline: PL proxy（K*sqrt(lambda_max(Σp)))
- [ ] Full: ARAIM PL_pred（跨 hypothesis worst-case）

Acceptance:
- 对“去几何更好区域”的轨迹，PL_pred 明显更低。

---

## 6. Planning / Optimization（完整性驱动代价）
### IAP-RQ-400: Integrity-aware objective
- [ ] 代价：Σ hinge(PL_pred - AL)^2 + λ_mission*dist_to_goal + λ_smooth*effort
- [ ] hinge 使用 max(0,·)

Acceptance:
- 在 IM 变小/为负时，planner 选择绕行策略以降低 hinge 项。

### IAP-RQ-410: Receding horizon loop
- [ ] horizon H（2–5s），执行 Δt（0.2–0.5s），重复
- [ ] mode=SEARCH 时优先恢复 IM；mode=NOMINAL 时优先推进任务

Acceptance:
- 可复现“退化→SEARCH→恢复→继续”的闭环行为。

---

## 7. Experiments & Metrics
### IAP-RQ-500: Baselines
- [ ] Passive（不主动，仅估计）
- [ ] Covariance-min（active SLAM 风格，最小 Σ）
- [ ] Integrity-aware（本方法）

Acceptance:
- 同一套场景下三者都能跑通并输出指标。

### IAP-RQ-510: Metrics
- [ ] Time(PL>AL) %
- [ ] Mission success rate
- [ ] Avg PL / Min IM
- [ ] Mission time, path length, control effort

Acceptance:
- 输出表格/曲线可直接用于汇报。

---

## 8. Documentation automation (IEEE Trans Methodology)
### IAP-RQ-900: Auto-generate IEEE Trans methodology chapter (with flowchart)
- [ ] 生成 `docs/methodology/methodology.tex`
- [ ] 文档首图为系统流程图（从 `docs/figures/system_flow.*` 引用）
- [ ] 从 TRACEABILITY 自动生成“模块-公式-接口-验证”骨架段落

Acceptance:
- 一条命令生成 .tex；能编译（至少结构无误），并包含流程图占位与小节骨架。