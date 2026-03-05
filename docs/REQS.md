# IAP Requirements Checklist (docs/REQS.md)

> Purpose: 把 talk《Integrity-Aware Active Perception》的“优化版 pipeline”拆成可实现、可验收、可追溯的工程需求。
> Scope: 仅修改本仓库 src/iap；禁止修改 ../glim（可参考其代码但不得提交改动）。

## 0. 术语与约定（强制一致）
- Estimator: 滑窗/因子图估计器（紧耦合 GNSS+IMU+LiDAR），基于现有 GLIM 框架演进
- Integrity: 完整性监测输出 PL/AL/IM，安全条件 **PL < AL**
- PL: Protection Level（对位置误差的保守上界）
- AL: Alert Limit（由障碍接近度动态给出）
- IM = AL - PL（安全裕度）
- Mode: NOMINAL / CAUTION / SEARCH（由 IM 触发）
- GNSS: pseudorange + doppler（含 receiver clock bias/drift）
- 伪距 residual 定义：**r_pr = meas - pred**
- 多普勒单位：**m/s**，residual 同样用 **meas - pred**
- 卫星位置/速度来源：广播星历（broadcast ephemeris）
- LiDAR: 使用 GLIM 的 scan-to-map ICP；需要 health(退化/错配) → 噪声膨胀（noise inflation）
- V̂(τ): 未来沿轨迹的可见卫星集合预测（必须考虑点云/地图遮挡）
- Ô(τ): 未来沿轨迹的“可观测性代理”（baseline 先用 LiDAR/ICP proxy，不强制 trunk）
- Trunk: 树干几何地标（圆柱模型）——**Upgrade 项**（非 baseline 必做）

---

## 1. Repo / Build / Dev（必须先做）
### IAP-RQ-000: Repo guardrails
- [x] 本仓库内加入 AGENTS.md（禁止改 ../glim，强制维护 CHANGES/TRACEABILITY）
- [x] 加入 docs/CHANGES.md、docs/TRACEABILITY.md、docs/REQS.md（本文件）
- [x] 加入 doc-guard：改代码必须同步更新 CHANGES + TRACEABILITY（git hook + CI）

Acceptance:
- 能在本地提交前被 hook 拦截；PR 中被 CI 拦截。

### IAP-RQ-001: Rename ROS2 package name to "iap"
- [x] package.xml 的 `<name>` 改为 `iap`
- [x] CMakeLists.txt 的 `project(...)` 改为 `iap`
- [x] 所有依赖此包名的路径/launch/ament 导出同步更新
- [x] 编译命令 `colcon build --packages-select iap` 可用
- [x] 生成 `docs/SPEC_VS_IMPL.md` 作为“现状基线盘点”

Acceptance:
- colcon 能成功构建 `iap`，且 `ros2 pkg list | grep iap` 可见。

### IAP-RQ-002: Build artifacts
- [x] 生成 compile_commands.json（给 clangd）
- [x] 提供最小 demo 运行方式（README 或 apps/ 下可执行）

Acceptance:
- VSCode clangd 无红线；demo 可运行并产生日志。

---

## 2. Estimator（紧耦合 FGO / 滑窗）
### IAP-RQ-010: State definition matches talk
- [x] 已有状态：p, v, q, b_a, b_g
- [x] 扩展状态：clk_bias, clk_drift（写入 EstimationFrame + factor graph key 管理）
- [x] 状态时序与滑窗边缘化可运行（扩展后不炸）

Acceptance:
- 日志打印 state 关键量（p, v, clk_bias, clk_drift），滑窗运行稳定。

### IAP-RQ-015: Expose position covariance Σ_p（插在 RQ-010 后）
- [x] 从 smoother/后端导出位置协方差块 Σ_p（至少 3x3 position block）
- [x] 将 Σ_p 封装进 EstimationFrame（或等价输出结构）
- [x] 日志输出 `lambda_max(Σ_p)` / `trace(Σ_p)`（供 PL proxy 与回归测试）
- [x] 提供接口占位：未来可替换为更精确的 `H Σ H^T` 等传播（先不实现真算）

Acceptance:
- replay 同一数据时 Σ_p 曲线稳定可视化；planner/integrity 可直接读 Σ_p。

### IAP-RQ-020: GNSS measurement model (pseudorange + doppler)
- [x] 新建 `src/iap/gnss/`：GNSS 因子、星历解算、ROS handler
- [x] 实现伪距因子：含 receiver clock bias，residual=meas-pred
- [x] 实现多普勒因子：含 receiver clock drift + 速度投影（m/s），residual=meas-pred
- [x] 每颗卫星观测作为独立观测通道（便于 per-sat integrity / gating）
- [x] 星历解算：由广播星历得到 sat pos/vel（可先接一个库/最小实现）

Acceptance:
- 日志输出每颗卫星 residual_pr / residual_dop；
- 可启用/禁用 GNSS 因子对比（回归用）。

### IAP-RQ-030: IMU preintegration + sliding window（GLIM baseline 已具备）
- [x] IMU 预积分因子可用
- [x] bias 作为状态可估计
- [x] fixed-lag smoother + 滑窗边缘化稳定

Acceptance:
- 轨迹连续且残差统计合理；运行不崩。

### IAP-RQ-040: LiDAR odometry / mapping factor baseline（GLIM baseline 已具备）
- [x] 最小版本：scan-to-map ICP 相对位姿因子（CPU/GPU/CT 任一路可跑）
- [x] 输出 ICP quality report（inlier/rmse/cond/degeneracy/相对位姿协方差等）
- [x] health→noise inflation：当退化/错配时，对 LiDAR factor 噪声膊胀（而非硬剖除）

Acceptance:
- ICP 因子可开关；质量指标能打点输出；
- 注入退化场景时噪声会膨胀，估计不至于发散。

---

## 3. Trunk geometric landmark（树干地标 + TDOP）【Upgrade】
### IAP-RQ-100: Trunk detection & parameterization
- [x] LiDAR 分割树干并拟合圆柱/圆（中心、半径）
- [x] 提供 detection confidence（用于 trunk fault prior）

Acceptance:
- 日志/可视化中能看到 trunk 数量 K、半径分布、置信度。

### IAP-RQ-110: Trunk factor in FGO (optional baseline → full)
- [x] Baseline-A: trunk 仅用于健康度/噪声膊胀（不入图）
- [ ] Full-B: trunk 作为几何观测因子入图（可用于降低 Σ_p）

Acceptance:
- Full-B 开启时，Σ_p/PL 明显优于仅 GNSS+IMU(+LiDAR)。

### IAP-RQ-120: TDOP metric (Tree DOP)
- [x] 计算 TDOP 或其代理（角度多样性/几何强度）
- [x] TDOP 与 trunk 分布可视化/日志输出

Acceptance:
- 场景变化（树更分散/更集中）时 TDOP 有明显变化。

---

## 4. Integrity monitoring（RAIM-ish baseline → full ARAIM）
### IAP-RQ-200: Integrity outputs (PL/AL/IM) + mode
- [x] 输出 PL、AL、IM 与 mode
- [x] 安全条件：PL < AL
- [x] 输出 integrity report：PL/AL/IM/mode + 关键中间量（Σ_p / |V| / gating 结果）

Acceptance:
- PL/AL/IM 曲线可画；mode 切换可复现。

### IAP-RQ-210: Alert Limit AL from obstacle proximity
- [x] AL 随障碍距离动态变化（至少实现 HAL 近似）
- [x] 与无人机半径/障碍半径相关（最小实现：几何安全裕度）

Acceptance:
- 越靠近障碍，AL 越小；日志可见 HAL/AL。

### IAP-RQ-220: GNSS per-satellite NIS gating (RAIM-ish baseline)
- [x] per-sat NIS 统计（pr/dop/joint）
- [x] downweight（gamma_R）与 exclude_sats 输出
- [x] global_nis 超阈值触发 FDE（贪心即可）

Acceptance:
- 注入某颗卫星 bias 时能被降权/剔除，PL/IM 改善。

### IAP-RQ-230: Fault hypothesis set includes trunks (full ARAIM)【Upgrade】
- [ ] fault hypotheses: H0 + satellite faults + constellation faults(可选) + trunk faults
- [ ] trunk fault prior 由 detection confidence 给出

Acceptance:
- 日志可见 N_f = N + C + K；PL 按 worst-case hypothesis 输出。

### IAP-RQ-240: ARAIM PL computation【Upgrade】
- [ ] fault-free + faulted 模式下计算 σ_q,k 与 PL_q,k
- [ ] 输出 HPL/VPL（至少 HPL）

Acceptance:
- PL 变化符合直觉：可见卫星少/几何差/退化 → PL 上升。

---

## 5. Prediction for planning（候选轨迹未来 PL_pred）
### IAP-RQ-300: Candidate trajectory generator
- [x] Baseline: motion primitives（v/ω/altitude 离散）
- [ ] 可扩展: spline/MINCO（后续）

Acceptance:
- 每次规划生成 M 条候选轨迹（含时间戳点序列）。

### IAP-RQ-310: Predict visibility/observability sets along τ
- [x] 预测卫星可见集合 V̂(τ)：点云/体素地图 ray-check 遮挡 +（可选）仰角掩膜
- [x] 预测可观测性代理 Ô(τ)：基于地图几何的 ICP 质量 proxy（可选加入遮挡统计）

Acceptance:
- 同一时刻不同轨迹预测的 |V̂|、Ô(τ) 有差异且合理。

### IAP-RQ-320: Information propagation → Σ_pred → PL_pred
- [x] 传播协方差得到 Σ_pred（baseline：经验增长模型）
- [x] Baseline: PL_pred proxy（K * sqrt(lambda_max(Σ_p_pred))）
- [ ] Full: ARAIM PL_pred（跨 hypothesis worst-case）【Upgrade】

Acceptance:
- 对“去几何更好区域/遮挡更少区域”的轨迹，PL_pred 明显更低。

---

## 6. Planning / Optimization（完整性驱动代价）
### IAP-RQ-400: Integrity-aware objective
- [x] 代价：Σ hinge(PL_pred - AL)^2 + λ_mission*dist_to_goal + λ_smooth*effort
- [x] hinge 使用 max(0,·)
- [x] mode=SEARCH 时提高 hinge 权重或加入"恢复可见卫星/可观测性"的项

Acceptance:
- 在 IM 变小/为负时，planner 选择绕行策略以降低 hinge 项（安全优先而非时间最短）。

### IAP-RQ-410: Receding horizon loop
- [x] horizon H（2–5s），执行 Δt（0.2–0.5s），重复
- [x] 规划输出轨迹/控制给执行器（或输出 nav_msgs/Path 先占位）
- [x] estimator <-> planner 初值互供：planner 用 estimator 当前状态；planner 的第一段作为 estimator 的 short-horizon 预测初值（可选）

Acceptance:
- 可复现“退化→SEARCH→恢复→继续”的闭环行为。

---

## 7. Experiments & Metrics
### IAP-RQ-500: Baselines
- [x] Passive（不主动，仅估计）
- [x] Covariance-min（active SLAM 风格，最小 Σ）
- [x] Integrity-aware（本方法）

Acceptance:
- 同一套场景下三者都能跑通并输出指标。

### IAP-RQ-510: Metrics
- [x] Time(PL>AL) %
- [x] Mission success rate
- [x] Avg PL / Min IM
- [x] Mission time, path length, control effort

Acceptance:
- 输出表格/曲线可直接用于汇报。

---

## 8. Documentation automation (IEEE Trans Methodology)
### IAP-RQ-900: Auto-generate IEEE Trans methodology chapter (with flowchart)
- [x] 生成 `docs/methodology/methodology.tex`
- [x] 文档首图为系统流程图（从 `docs/figures/system_flow.*` 引用）
- [x] 从 TRACEABILITY 自动生成"模块-公式-接口-验证"骨架段落

Acceptance:
- 一条命令生成 .tex；能编译（至少结构无误），并包含流程图占位与小节骨架。