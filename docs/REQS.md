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
- [ ] 完成 repository-local guard 验收：ICRA-071 handoff 时 relative `.githooks` 与 focused 33/33 已实现，
  但 Supervisor Review 发现 §8.6 handoff deadlock、active claim/RQ-ID 漏检及 full discovery 614/616；
  decision 002 后当前 verifier 仍 PASS，但写死 decision 001/task 071 的 focused suite 为 21/33。repair
  仍未验收；用户决策 `USER-ICRA-ROUTE-20260826-002` 将其保留为 non-blocking governance backlog，不再
  阻塞 ICRA-072 开发性全流程垂直切片

Acceptance:
- 能在本地 commit/push 前被相同语义的 hook 拦截；无 bypass 环境变量；明确本地 hook 不是同权限
  Agent 无法绕过的安全边界。

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

### IAP-RQ-003: Standalone ROS2 operation (no runtime dependency on glim_ros)
- [x] `iap_rosnode` 可独立启动，无需 source `/root/ros2_ws/install/setup.bash`
- [x] `librviz_viewer.so` 由 IAP 自身提供（从 glim_ros 移植，头文件改 `<iap/...>`）
- [x] `libstandard_viewer.so` 由 IAP 自身提供（从 glim 移植，头文件改 `<iap/...>`）
- [x] `config_ros.json` 的 `extension_modules` 只引用 IAP 自产的 `.so`

Acceptance:
- `source install/setup.bash` 后 `ros2 run iap iap_rosnode` 启动；
- RViz2 可见 `~/aligned_points`、`~/odom` 话题；
- 3D 桌面查看器窗口正常弹出。

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

### IAP-RQ-081: Ordinary EGO planner closed-loop baseline
- [x] 以 `/drone_0_visual_slam/odom` 作为 EGO planner、EGO grid map、SO3 controller 的默认反馈里程计。
- [x] 保留 truth odom 仅用于 plant state、sensor simulation、GNSS simulation、visualization 和 logging。
- [x] 使用 EGO 原生 `traj_server -> PositionCommand -> SO3ControlComponent -> so3_quadrotor_simulator` 命令链。
- [x] 默认 GNSS 仿真使用 RINEX 多星座 `GPS,BDS,GAL,GLO`，并保留 launch args 支持切换回 synthetic/GPS-only 或替换 RINEX 文件。
- [x] 默认启动 RViz，并像 demo8 一样可视化 IAP 估计、truth、desired 三条轨迹。
- [x] 暴露 `point_num` 和 `point1_*` 到 `point4_*` launch args；默认单目标仍由 `goal_x/y/z` 控制。
- [x] 输出 Phase 1 topic contract、闭环 tracking logs、topic summary 和 validation script。

Acceptance:
- `ros2 launch iap demo9_ego_planner_closed_loop.launch.py start_rviz:=false run_duration_s:=30`
  后 `tools/phase1/validate_phase1_closed_loop.py` 返回 0。

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

# IAP Requirements Checklist (Talk-aligned, Next Phase)

> Goal: 从“能跑的 baseline”升级到“符合 talk 公式/机制”的优化版（ARAIM + 预测可见性 + trunk factor）。

## Legend
- DONE: 已实现且机制对齐
- APPROX: 有实现，但与 talk 仅近似（需升级）
- GAP: talk 明确要求但缺失

---

## Phase-0: Baseline already present (do not rework unless needed)
### IAP-RQ-020 GNSS factors (pseudorange+doppler, meas-pred)
Status: DONE
- Pseudorange residual = meas - (||p - p_sat|| + clk_bias[m])
- Doppler residual = meas - (eᵀ(v_r - v_s) + clk_drift[m/s])
Acceptance: residual/jacobians correct; per-sat factor exists.

### IAP-RQ-100 Trunk detection + confidence
Status: DONE
- clustering + circle fit + confidence + TDOP proxy

### IAP-RQ-200 Integrity outputs PL/AL/IM/mode
Status: APPROX
- PL currently covariance proxy (not ARAIM)
- AL from obstacle distance ok

### IAP-RQ-400 Planner cost shape
Status: APPROX
- hinge² exists, but PL_pred lacks trajectory dependence and is not ARAIM-pred

---

## Phase-1: Make prediction physically meaningful (Talk §3, §7)  [TOP PRIORITY]

### IAP-RQ-311 Build/Expose local occupancy for ray checks
Talk: §7.2 needs predicted visible satellites/landmarks under canopy.
- [x] Provide a `LocalOccupancyGrid` (voxel hash or Octomap) from GLIM map/points
- [x] Query API: `ray_occluded(origin, dir, max_range)` and `occupancy_ratio(origin, dir, L)`
Acceptance:
- Unit test with synthetic occupied voxels (ray hits / misses correct)

### IAP-RQ-312 Predict satellite visibility set V^(τ) by ray casting
Talk: §7.2 predicted V̂ used for predicted geometry and PL_pred.
- [x] For each waypoint and each satellite direction, compute visibility (LOS not blocked within L_occ)
- [x] Output: |V_hat| and per-sat visibility flags
Acceptance:
- In a canopy map, moving under cover reduces |V_hat|; open sky increases.

### IAP-RQ-313 Estimate canopy density κ along LOS (prediction-time)
Talk: §3.2 σ_eff(κ, θ) canopy term.
- [x] Define κ = occupancy_ratio along LOS (0..1) or accumulated occupied length
- [x] Log κ per satellite per waypoint
Acceptance:
- κ increases under dense canopy; near 0 in open.

### IAP-RQ-314 Implement σ_eff(κ, θ) and weight matrix W
Talk: §3.2 σ²_eff = σ²_c * exp(α κ / sin θ)
- [x] Add params: α, σ_c, κ definition, elevation θ
- [x] Replace/extend elevation-only sigma with canopy-aware sigma (at least in prediction)
Acceptance:
- Same elevation but higher κ → larger σ_eff; matches monotonicity.

### IAP-RQ-321 Make PL_pred trajectory-dependent (replace sigma-only growth)
Talk: §7.2 predicted covariance from predicted geometry/information
- [x] At minimum: sigma_grow scaled by function f(|V_hat|, κ, lidar_obs_proxy)
- [x] Better: maintain 3×3 Σ_p_pred and update via info increments
Acceptance:
- Different candidates yield different PL_pred sequences.

### IAP-RQ-322 Coherent rolling P0 risk window
Status: PLANNED for the active ICRA P0 runtime; design frozen in
`docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`.
- [ ] Use a fixed world-aligned risk-voxel lattice and a UAV-centred local window.
- [ ] Recompute only newly exposed or explicitly invalidated spatial evidence while
      preserving all 76,800 logical risk voxels in each published generation.
- [ ] Bind reuse to source version/stamp, TTL and frame/config identity without restamping.
- [ ] Publish only coherent immutable generations; mixed-version or partial updates fail closed.

Acceptance:
- A one-cell window shift recomputes only the entering slab and evicts the leaving slab.
- Incremental and forced-full rebuilds are scientifically equivalent for identical inputs.
- Source changes, TTL expiry and frame reset invalidate the documented minimum safe region.
- P4/P5 continue to consume the existing immutable `RiskGridSnapshot` Interface.

### Active ICRA P0 conformance note — 2026-08-21

The checked items above describe capabilities present somewhere in the repository; they do
not by themselves qualify the active ICRA P0 runtime. ICRA-007 established that the current
production P0 provider does not bind the available `LocalOccupancyGrid` into GNSS visibility
and that its six frozen horizons do not apply empirical covariance growth. Consequently:

- IAP-RQ-311 is implemented as a reusable occupancy Module;
- IAP-RQ-312 and IAP-RQ-314 have tested predictor implementations but are not bound into the
  active production P0 GNSS path;
- IAP-RQ-320 and IAP-RQ-321 remain unqualified for the active P0 runtime because
  `Sigma_pred/PL_pred` lacks horizon growth;
- IAP-RQ-322 is planned and must not be reported as implemented before its staged tests and
  Gate-0B qualification pass.

---

## Phase-2: Turn trunks into real constraints (Talk §4.2, §5.1)

### IAP-RQ-131 Trunk data association & persistent IDs
Talk: trunk landmarks L={c_k} in optimization window
- [x] Maintain trunk map (local) with IDs
- [x] Associate detections to map IDs (nearest in xy + radius gate)
Acceptance:
- Same trunk observed across frames keeps consistent ID > 80% in replay.

### IAP-RQ-132 Trunk observation factor in factor graph (Full-B)
Talk: §4.2 trunk residual + Σ_trunk
- [x] Implement `TrunkFactor(x_t, c_k)` residual consistent with talk
- [x] Implement covariance Σ_trunk (range/bearing/z) and confidence→noise
Acceptance:
- Enable trunk factors reduces Σ_p and PL_proxy in forest scenes.

### IAP-RQ-133 TDOP weighted form (optional upgrade)
Talk: TDOP = sqrt(tr((Gᵀ W G)^-1))
- [x] Incorporate W from Σ_trunk/confidence
Acceptance:
- TDOP improves when trunks are angularly spread and confident.

---

## Phase-3: Implement ARAIM (Talk §6.4–6.6)  [CORE TALK COMPLIANCE]

### IAP-RQ-241 Hypothesis set enumeration
Talk: §6.2 fault hypotheses H0 + sat faults + trunk faults (+constellation optional)
- [x] Enumerate: H0 + each satellite single-fault + each trunk single-fault
- [x] Priors: P_sat from ISM config; P_trunk from confidence mapping
Acceptance:
- Logs: N_f = 1 + N_sat + K_trunk (optionally +C)

### IAP-RQ-242 Full & subset solutions (solution separation)
Talk: §6.4 compute full solution and subset solution by zeroing weights
- [x] Extract linearized WLS at epoch (G, r, W) from factor graph linearization
- [x] Compute p^(0) and p^(k) for each hypothesis k
Acceptance:
- Deterministic outputs; subset differs when faulted measurement removed.

### IAP-RQ-243 Separation statistics σ_ss,q,k
Talk: §6.4.3 / §6.5
- [x] Compute separation vector d_k = p^(0) - p^(k)
- [x] Compute σ_ss,q,k projected to directions (E/N or horizontal) as in talk
Acceptance:
- σ_ss increases when geometry weakens / fewer sats.

### IAP-RQ-244 Detection thresholds & multipliers (K_fa, K_md)
Talk: §6.5 / §6.6
- [x] Allocate P_FA budgets across hypotheses
- [x] Compute thresholds T_q,k and K_md from P_fault and P_HMI budget
Acceptance:
- Changing P_FA/P_HMI changes thresholds monotonically (more strict → larger PL)

### IAP-RQ-245 Faulted PL and overall PL
Talk: §6.6.2–6.6.3
- [x] Compute PL_faulted,q,k and PL_ff,q
- [x] Output PL_ARAIM = max(PL_ff, max_k PL_faulted,k)
Acceptance:
- Inject a biased satellite: PL rises; after exclusion/recompute PL drops.

### IAP-RQ-246 Close-loop FDE (exclude & recompute)
Talk: when detected, exclude measurement and recompute solution
- [x] If |d_k| > T_q,k: flag measurement and rebuild weights/factors
- [x] Re-run solve for current epoch
Acceptance:
- Logs show “detected→excluded→re-solved” loop.

---

## Phase-4: Integrity-aware planning must use predicted ARAIM PL (Talk §7.2–7.3)

### IAP-RQ-331 Predicted ARAIM PL along candidate trajectory
Talk: §7.2 predicted PL_ARAIM at future waypoints using predicted geometry
- [x] For each candidate waypoint: build predicted G/W using V_hat and (optional) predicted trunk visibility
- [x] Run Phase-3 ARAIM routine in "prediction mode" (no real residuals → use expected/noise model)
Acceptance:
- Under canopy candidates yield higher PL_pred_ARAIM; open path lower.

### IAP-RQ-421 Dynamic AL(τ) along trajectory
Talk: AL derived from proximity to obstacles (future waypoints)
- [x] Query ESDF/distance field for each waypoint → AL_i
Acceptance:
- Passing near obstacles yields lower AL_i.

### IAP-RQ-422 Planner uses (PL_pred_ARAIM_i - AL_i)
Talk: §7.3 hinge cost
- [x] Replace constant AL with per-waypoint AL_i
- [x] Use PL_pred_ARAIM_i (not proxy) in hinge
Acceptance:
- Planner chooses safer path even if longer when integrity violated.

### IAP-RQ-423 P4 collision-guide planning and P5 lineage
Status: **P4-v1 SCIENTIFIC_NO_GO RETAINED / ICRA-072A LAYER 1 PASS / ICRA-072B STABILIZATION TASK_READY /
INVERSE-CORRIDOR DESIGN FROZEN FOR LAYER 3, IMPLEMENTATION NOT STARTED**

Source: `docs/icra27/ICRA_SCOPE.md` and the 2026-08-20 Supervisor scope pivot. This requirement extends collision-guide planning evidence; it does not replace or verify the IAP-RQ-422 PL/AL admission rule.

- [ ] Collision scan returns `NO_COLLISION`, `CLOSED_SEGMENTS`, `OPEN_ENDED_COLLISION`, or `INVALID_INPUT`.
- [ ] The first 2/3 of the seed is the entry trigger window. After entry, scanning continues to the seed end to find a free exit.
- [ ] `OPEN_ENDED_COLLISION` never becomes `NO_COLLISION`, never invents an occupied endpoint, and cannot publish a new normal trajectory.
- [ ] Each closed segment produces original and risk-aware A* guides with one attempt/segment ID, free endpoints, occupancy epoch, immutable P0 snapshot, and query time model.
- [ ] Occupied nodes remain hard rejected. Unknown, stale, non-finite, timeout, or failed risk search falls back to the current-epoch original guide with an explicit reason.
- [ ] An occupancy-epoch or request-identity mismatch returns `DECISION_INVALID/REPLAN_REQUIRED`; neither guide is injected and that attempt cannot publish a new normal trajectory.
- [ ] P4 selects the risk guide only after fixed-200, same-snapshot mean/max risk dominance and the frozen path-length gate pass. Metrics-only mode retains the original guide.
- [ ] Before P4-G0C freezes thresholds, G0B and calibration run with `p4.metrics_only=true` and `selection_applied=false`; online risk-guide application begins only in G0D.
- [ ] The selected guide and hash remain traceable through control-point constraints, rebound optimization, refinement, feasibility, and the final B-spline.
- [ ] P5 final remains the hard integrity gate before normal publish; P5 runtime remains active after publish. P4 and P5 record snapshot generations separately.
- [ ] P4 remains advisory. EGO occupancy/dynamics keep motion-feasibility authority, and P5 remains the IAP hard integrity gate.
- [ ] After ICRA-072A integration and ICRA-072B stabilization close ICRA-072, implement the frozen
  `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1` PRIMARY/EXACT_MIRROR/FLAT_NULL geometry and evaluate only the
  committed final B-spline through an independent oracle. The design is frozen in
  `docs/icra27/dev/ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE.md`; implementation has not started and is deferred to
  ICRA-073.

Acceptance:

- Deterministic fixtures cover no collision, closed collision, open-ended collision, multiple obstacles, free endpoints, fallback, and occupancy-epoch change.
- One event yields auditable original/risk/selected guides with `200/200` valid samples, frozen risk/length/latency gates, and no A* timeout.
- The selected decision/hash reaches the final B-spline, and a P5 final rejection produces zero normal publication.

### IAP-RQ-424 User-owned ICRA research route and P4-v2 scientific recovery
Status: **DEVELOPMENT-FIRST ROUTE LOCKED / FOUR-LAYER WORKFLOW ACTIVE / ICRA-072A LAYER 1 PASS /
ICRA-072B STABILIZATION TASK_READY /
GUARD REPAIR NONBLOCKING / INVERSE-CORRIDOR DESIGN FROZEN, IMPLEMENTATION NOT STARTED**

Source: `docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md` and user decision
`USER-ICRA-ROUTE-20260826-002` bound to `b24a330d79d6e85e8080cf2a359bb1a18765e5a5`.

- [x] Freeze `route_owner=USER`, active `P0_P4_V2_P5`, required modules, research question, primary claim,
  formal arms, gate sequence, fallback policy and campaign-approval boundary in one machine-readable Markdown
  route lock.
- [x] Preserve P4-v1 G0C as immutable `SCIENTIFIC_NO_GO`; never relabel it as PASS or reuse it as P4-v2
  confirmatory evidence.
- [x] Define NO-GO transition as `BLOCKED_AWAITING_USER_RESEARCH_DECISION`, with no automatic contingency task.
- [ ] Complete the non-blocking ICRA-071 governance backlog: repair Supervisor handoff, exact active
  claim/module binding and nonexistent requirement-ID rejection. It is no longer a prerequisite for ICRA-072.
- [x] Install and verify repository-relative pre-commit, pre-push and commit-msg hooks with no bypass variable.
- [ ] Complete adversarial coverage and one zero-failure hermetic discovery; focused coverage is 33/33, but
  Review reproduced claim/RQ-ID gaps and the current full discovery is 614/616.
- [x] In ICRA-072, add P4-v2 provider/occupied/unknown support decomposition while preserving the v1 scalar
  query for replay and implement the minimal provider-only interior bottleneck/lexicographic time-aware search.
  Builder HEAD `6a6bdd3` proves natural live P4 selection but not the terminal EGO/P5/publication/runtime chain.
- [ ] Complete ICRA-072A Layer 1 using shared incremental build roots and unique repeatable development runs;
  then complete ICRA-072B production-shaped regression before closing ICRA-072. The controlling process is
  `docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md`.
- [x] Freeze `ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1` as a deferred ICRA-073 design with two feasible curved
  homotopies, PRIMARY/EXACT_MIRROR/FLAT_NULL causal controls, independent-oracle isolation and a 200-point
  final-B-spline analysis contract. This documentation checkbox does not mark fixture implementation or effect
  validation complete.
- [ ] Freeze SESOI, endpoint buffer `b=2r`, independent 30–60 seed-run sample size per scene and held-out protocol
  before confirmatory execution.

Acceptance:

- The local guard rejects any protected route change without a distinct user decision bound to the exact pushed
  anchor; documentation states that local hooks are procedural rather than a security boundary.
- P4-v2 confirmation uses provider-only `D_peak` as the sole primary, counts independent run/seed units, and
  passes the preregistered exact one-sided gate before G0D.
- G0D/P5 lineage passes before a separate user decision may authorize the 60-run formal campaign.

---
Initial ICRA-072 Review note (2026-08-26): the provider-only decomposition, time-aware
P4-v2 bottleneck search and static EGO/P5 seams are implemented with final
build `attempt_11`, 137/137 focused C++ tests, 22/22 launch tests and 3/3
runner/analyzer tool tests. The sole registered development smoke remains
immutable FAIL evidence: P0 generation was zero, and P4 selection, EGO
lineage, P5 final/runtime binding and normal publication were all zero. The
profile now statically binds the existing 0.01 legacy baseline, but that fix
was not exercised live. Review additionally found that a subsequent
no-collision refinement can clear the transient selected-guide vector required
by final lineage, and the launch manifest lost the explicit P4 evidence path.
ICRA-072 is reissued for the bounded lineage/path repair, fresh build and
exactly one `icra072-dev-smoke-002`. All mapped requirements remain
unqualified; no effect, optimization, qualification or campaign claim is made.

Latest ICRA-072 Review note (2026-08-26): the replacement fixed P0 startup and
the exact manifest path. The immutable `icra072-dev-smoke-002` passed GPU and
15/15 process health, produced 123 ready P0 rows and 1,464 P4-v2 decisions, but
all original guides had zero valid provider samples. Risk selection, final
B-spline, P5 and runtime lineage therefore remained zero. Final static
`attempt_15` passes focused tests but was built after the sole live run.
Review also finds that terminal lineage is not revalidated against live
occupancy after snapshot release and that the required manager/FSM/P5/runtime
regression is absent. ICRA-072 is reissued for one final bounded closure and
one non-overwriting `icra072-dev-smoke-003`; ICRA-073 and inverse-corridor
implementation remain unauthorized until full live Review PASS.

Final ICRA-072 Builder note (2026-08-26): terminal attempt/snapshot/epoch
revalidation, actual production FSM/P5/publish/runtime regression, analyzer
support accounting and the separately named development-only selection trigger
are implemented. Fresh `attempt_19` passes 199/199 focused C++ and 29/29
Python checks. The sole immutable `icra072-dev-smoke-003` passed one GPU
preflight and all 15 required processes, produced 124 ready P0 rows, 76 natural
risk selections and 339 decisions with complete provider support for both
guides. Its analyzer was invoked exactly once and failed closed: terminal
lineage, P5-final pass, committed runtime binding and normal B-spline
publication counts were all zero. No retry or tuning is permitted. ICRA-072 is
therefore BLOCKED pending Supervisor review; ICRA-073 and all effect,
qualification and campaign work remain unauthorized.

ICRA-072A Supervisor Review note (2026-08-27): Builder HEAD `cd56257` adds the shared six-package development
build, iterative tooling and a structurally complete `run-020`; those parts are retained. Layer 1 is not complete.
The development profile keeps `max_pl` fusion but changes P5 to `LIDAR_CERTIFIED`, so P5 reports final `OK` and
publishes while raw output labels the authoritative fused monitor `UNSAFE` (`HPL/VPL 28.904/75.079` versus
`HAL/VAL 10/20`). Runtime analysis also accepts a 20 ms start mismatch without a runtime trajectory ID, and not
every retained iteration has a machine first-missing-stage outcome. Verdict is `REQUEST_CHANGES`; continue the
same Gate at `run-021` with fused P5 authority, exact runtime identity and complete typed iteration records.
ICRA-072B and all Layer 3/4 work remain unauthorized.

Latest ICRA-072A Supervisor Review note (2026-08-27): Builder HEAD `b607b97` and fresh `run-023` prove strong
development integration: pushed-source checks, all seven P0/P4/EGO/fused-P5/publish/runtime stages, exact actual
selected samples, 15/15 process health and cleanup all pass. Layer 1 remains `REQUEST_CHANGES` because a mixed
runtime record can pass when only one committed sample matches, source admission suppresses arbitrary untracked
files, and the final changed-during-run source branch lacks required TDD. Builder also disclosed external `/tmp`
verification output and deletion contrary to repository-local retention. Continue ICRA-072A with the bounded
exact-admission repair and fresh `run-024` or later after a clean pushed implementation. ICRA-072B and all Layer
3/4 work remain unauthorized.

ICRA-072A exact-admission Builder checkpoint (2026-08-27): the bounded repair now rejects a counted fused runtime
row unless every committed sample carries the same explicit positive selected ID and integer-nanosecond start.
Source binding v2 inspects full status and permits only the protected PDF at exact path/hash while recording
allowlisted, observed and rejected paths; arbitrary untracked source and every tracked/staged state reject.
Focused initial/pre-ROS/final source-change coverage includes a complete mocked final-change lifecycle with typed
outcomes and owned-group cleanup. Shared build is 6/6, tools 17/17, retained C++ is 64/64 and hermetic launch is
24/24. Implementation `b7b5357` was then committed/pushed and verified at divergence `0 0` before fresh
`run-024`. That first attempt is runner/analyzer PASS with all seven stages, accepted selected ID `12`, start
`1657065616411275703 ns`, final identity `36cb40d791d9b347`, four exact fused-safe runtime rows, 15/15 required
process health and complete owned cleanup. The loop stopped. ICRA-072B and Layer 3/4 remain unauthorized pending
Supervisor review.

ICRA-072A Layer 1 Supervisor PASS (2026-08-27): reviewed Builder HEAD `ac7f923`; Standards and Spec axes pass.
Shared build is 6/6 and independent Supervisor reruns pass tools 17/17, hermetic launch 24/24 and focused C++
64/64. Fresh `run-024` binds pushed implementation `b7b5357` at initial/pre-ROS/final checks, passes GPU, 15/15
required-process health and owned cleanup, and reports all seven stages with no failures or first-missing stage.
Selected ID `12`, start `1657065616411275703`, final identity `36cb40d791d9b347` agree; all 44 samples in four
fused runtime rows are exact and safe. ICRA-072A is complete only as development integration. ICRA-072B is now the
sole active task; it must stabilize the happy path and epoch/attempt/lineage/P5 fail-closed boundaries before
ICRA-072 can close or ICRA-073 can be issued. No scientific, qualification or campaign claim is made.

Four-layer Supervisor disposition (2026-08-26): checkpoint `6a6bdd3` is
`ARCHIVED_AS_FOUND / BLOCKED_TERMINAL_CHAIN_MISSING`. User workflow decision
`USER-ICRA-WORKFLOW-20260826-001` authorizes ICRA-072A to iterate with shared
workspace build/install roots until one real identity reaches EGO final,
P5-final-before-publish, normal publication and P5 runtime. Failed development
runs may be fixed and repeated under new run IDs without intermediate Review.
ICRA-072B then stabilizes that chain; only its later Review PASS may issue
ICRA-073 inverse-corridor effect diagnostics. Formal hashes, one-shot rules,
held-out and qualification remain Layer 4.
