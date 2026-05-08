# IAP Method Chapter — Module Decomposition for IEEE Paper

## Recommended Method Section Structure (9–10 subsections)

按学术论文的信息流而非代码目录组织。每个 subsection 标注了对应的代码路径、关键公式、以及论文中应强调的创新点。

---

### §II-A. System Overview

**论文内容**: 一张系统框图 + 一段文字描述异步多模块流水线。

**代码**:
- `apps/iap_rosnode.cpp` — `IapRosNode`: load config → load core modules (dlopen) → load extension modules → subscribe IMU/PointCloud2 → queue bridge thread (odom → submap → global)
- `include/iap/util/extension_module.hpp` — `ExtensionModule::load_module()`

**写法要点**:
- 传感器数据定义: $\mathcal{D}_{imu} = \{(\tilde{\mathbf{a}}_t, \tilde{\boldsymbol{\omega}}_t, t)\}$, $\mathcal{D}_{lidar} = \{(\mathcal{P}_k, t_k)\}$, $\mathcal{D}_{gnss} = \{(\rho_t^j, \dot{\rho}_t^j, \mathbf{s}_t^j, t)\}$
- 强调 asynchronous multi-module pipeline,不深入 ROS2 实现细节

---

### §II-B. LiDAR-IMU-GNSS Factor Graph Odometry

**论文内容**: 滑动窗口 FGO 的状态向量、因子构成、优化问题。

**代码**:
```
src/iap/odometry/odometry_estimation_base.cpp   — 基类接口
src/iap/odometry/odometry_estimation_cpu.cpp    — CPU 后端
src/iap/odometry/odometry_estimation_gpu.cpp    — GPU 后端
src/iap/odometry/odometry_estimation_ct.cpp     — CT 后端
src/iap/common/imu_integration.cpp              — IMU 预积分
src/iap/common/cloud_deskewing.cpp              — 点云去畸变
src/iap/common/cloud_covariance_estimation.cpp  — 协方差估计
src/iap/preprocess/cloud_preprocessor.cpp       — 点云预处理
```

**状态向量**:
$$
\mathbf{x}_k = [\mathbf{T}_k, \mathbf{v}_k, \mathbf{b}_k^a, \mathbf{b}_k^g, \delta t_k^{clk}, \dot{\delta t}_k^{clk}]
$$

**优化问题**:
$$
\mathbf{X}^\star = \arg\min_{\mathbf{X}} \left(
\sum_i \|\mathbf{r}_i^{imu}\|^2_{\Sigma_{imu,i}} +
\sum_m \|\mathbf{r}_m^{lidar}\|^2_{\Sigma_{lidar,m}} +
\sum_j \|r_k^{gnss,j}\|^2_{\sigma^2_{gnss,j}} +
\|\mathbf{r}^{prior}\|^2_{\Sigma_{prior}}
\right)
$$

**LiDAR 残差** (点到平面 / GICP 统一形式):
$$
r_m^{lidar} = \mathbf{n}_m^\top (\mathbf{T}_k \mathbf{p}_m - \mathbf{q}_m)
$$
$$
\mathbf{r}_m^{gicp} = \mathbf{T}_k \mathbf{p}_m - \mathbf{q}_m, \quad
\Sigma_m^{gicp} = \Sigma_m^{map} + \mathbf{R}_k \Sigma_m^{scan} \mathbf{R}_k^\top
$$

**创新点**: 将 GNSS 伪距/多普勒因子和树干地标因子统一纳入 FGO 框架,而非后处理。

---

### §II-C. GNSS Measurement Model and Visibility Prediction

**论文内容**: 伪距/多普勒因子的解析雅可比、设计矩阵、树冠遮挡下的可见性预测。

**代码**:
```
src/iap/gnss/pseudorange_factor.cpp    — PseudorangeFactor::evaluateError()
src/iap/gnss/doppler_factor.cpp        — DopplerFactor::evaluateError()
src/iap/gnss/clock_between_factor.cpp  — ClockBetweenFactor
src/iap/gnss/gnss_handler.cpp          — GnssHandler::insert_epoch() / get_factors()
src/iap/gnss/visibility_predictor.cpp  — VisibilityPredictor::predict()
src/iap/map/local_occupancy.cpp        — LocalOccupancyGrid::ray_occluded() / occupancy_ratio()
```

**伪距残差** (corrected pseudorange):
$$
r_k^{\rho,j} = \rho_k^{j,corr} - \left( \|\mathbf{p}_k - \mathbf{s}_k^j\| + c \cdot \delta t_k^{clk} \right)
$$

**多普勒残差**:
$$
r_k^{\dot{\rho},j} = \tilde{\dot{\rho}}_k^j - \mathbf{e}_k^{j\top} (\mathbf{v}_k - \mathbf{v}_{sat,k}^j) - c \cdot \dot{\delta t}_k^{clk}
$$

**设计矩阵** (ENU + clock, $N \times 4$):
$$
\mathbf{G}_k = \begin{bmatrix}
-\mathbf{e}_k^{1\top} & 1 \\
\vdots & \vdots \\
-\mathbf{e}_k^{N\top} & 1
\end{bmatrix}, \quad
\mathbf{W}_k = \mathrm{diag}(\sigma_1^{-2}, \ldots, \sigma_N^{-2})
$$

**树冠噪声模型**: $\sigma_{eff}^2 = f(\sigma_{base}, \theta_{elev}, \kappa)$, 其中 $\kappa \in [0,1]$ 为 canopy density,由 `LocalOccupancyGrid::occupancy_ratio()` 沿卫星视线方向采样计算。

**可见性预测**: 对每颗卫星,从预测位置出发做 DDA 射线遍历 (`ray_occluded`),被遮挡的卫星不计入 $N_{vis}$。

**创新点**: Canopy-aware noise model + ray-cast visibility prediction,将环境几何引入 GNSS 权重。

---

### §II-D. Tree-Trunk Geometric Landmark and TDOP

**论文内容**: Kasa 圆柱拟合、EKF 地标跟踪、GTSAM Point2 因子、TDOP 几何多样性度量。

**代码**:
```
src/iap/trunk/trunk_detector.cpp   — TrunkDetector::detect() / kasa_fit() / compute_tdop()
src/iap/trunk/trunk_map.cpp        — TrunkMap::update() (EKF data association)
src/iap/trunk/trunk_factor.cpp     — TrunkFactor::evaluateError()
src/iap/trunk/trunk_extension.cpp  — TrunkExtensionModule (ROS2 集成)
```

**树干模型**: $\mathcal{T}_l = (\mathbf{c}_l, r_l)$, $\mathbf{c}_l = [c_{x,l}, c_{y,l}]^\top \in \mathbb{R}^2$

**Kasa 圆拟合**:
$$
(\hat{\mathbf{c}}_l, \hat{r}_l) = \arg\min_{\mathbf{c}_l, r_l} \sum_{i=1}^{M_l} \left( \|\mathbf{p}_i^{lidar} - \mathbf{c}_l\| - r_l \right)^2
$$

**EKF 地标跟踪** (TrunkMap):
- 状态: $\mathbf{c}_l$, 协方差 $\mathbf{P}_l \in \mathbb{R}^{2\times 2}$
- 预测: $\mathbf{P}_{l,k+1}^- = \mathbf{P}_{l,k} + \sigma_{proc}^2 \Delta t \cdot \mathbf{I}_2$
- 更新: $\mathbf{K} = \mathbf{P}^- \mathbf{H}^\top (\mathbf{H} \mathbf{P}^- \mathbf{H}^\top + \sigma_{obs}^2 \mathbf{I}_2)^{-1}$

**TrunkFactor 残差** (sensor XY frame, 2D):
$$
\mathbf{r} = \mathbf{z}_k - \mathbf{R}_{2\times 2}^\top (\mathbf{c}_l - \mathbf{p}_t^{xy})
$$
噪声: $\Sigma_{trunk} = (\sigma_{xy}^2 / confidence) \cdot \mathbf{I}_2$

**TDOP** (Tree Dilution of Precision):
$$
\mathbf{G}_{tree} = \begin{bmatrix} \mathbf{u}_1^\top \\ \vdots \\ \mathbf{u}_K^\top \end{bmatrix}, \quad
\mathbf{u}_l = \frac{\mathbf{c}_l - \hat{\mathbf{p}}_k^{xy}}{\|\mathbf{c}_l - \hat{\mathbf{p}}_k^{xy}\|}, \quad
\mathrm{TDOP} = \sqrt{\mathrm{tr}\left((\mathbf{G}_{tree}^\top \mathbf{W}_{tree} \mathbf{G}_{tree})^{-1}\right)}
$$

**创新点**: 将树干作为几何地标纳入 FGO 提供 LiDAR-only 约束;TDOP 量化森林环境中地标的几何多样性。

---

### §II-E. FGO-Aware Integrity Monitoring

**论文内容**: 从 iSAM2 边缘化提取位置协方差 → PL proxy → dynamic AL → 三状态机。这是论文的核心创新——将 FGO covariance、GNSS residual、LiDAR/trunk geometry 组织成统一的完好性监视管线。

**代码**:
```
src/iap/integrity/integrity_extension.cpp       — IntegrityExtensionModule (主调度)
src/iap/integrity/integrity_monitor.cpp         — IntegrityMonitor::compute()
src/iap/integrity/fgo_information_manager.cpp   — FGOInformationManager::extract()
```

**FGO covariance proxy PL**:
$$
\mathrm{PL}_k^{fgo} = K_{pl} \sqrt{\lambda_{\max}(\Sigma_{\mathbf{p},k})}
$$
其中 $\Sigma_{\mathbf{p},k}$ 由 `FGOInformationManager::extract()` 从 iSAM2 边缘化结果中提取 ($3\times 3$ 位置协方差子矩阵)。

**Dynamic Alert Limit**:

水平 AL (基于树干障碍):
$$
\mathrm{HAL}_k = \gamma_H \min_{l \in \mathcal{N}_k} \left( \|\hat{\mathbf{p}}_k^{xy} - \mathbf{c}_l\| - r_l - r_{drone} \right)
$$

垂直 AL (基于高度):
$$
\mathrm{VAL}_k = \gamma_V \cdot d_{z,k}
$$

总 AL:
$$
\mathrm{AL}_k = \min(\mathrm{HAL}_k, \mathrm{VAL}_k)
$$

**Integrity Margin**:
$$
\mathrm{IM}_k = \mathrm{AL}_k - \mathrm{PL}_k
$$

**三状态机**:
$$
s_k = \begin{cases}
\texttt{UNSAFE}, & \mathrm{PL}_k \geq \mathrm{AL}_k \\
\texttt{SAFE\_EXCLUDED}, & \mathrm{PL}_k < \mathrm{AL}_k \land N_{fault,k} > 0 \\
\texttt{SAFE}, & \mathrm{PL}_k < \mathrm{AL}_k \land N_{fault,k} = 0
\end{cases}
$$

**创新点**: PL 来源于 FGO 边缘化而非独立 WLS;AL 来源于实时树干几何而非固定值;PL/AL/IM 三者为无人机提供可操作的导航安全判据。

---

### §II-F. ARAIM Solution Separation (GNSS + LiDAR + Trunk Hypotheses)

**论文内容**: 单故障假设枚举、全解/子集解分离、FDE 闭环。

**代码**:
```
src/iap/integrity/araim.cpp       — Araim::run() / predict_geometry() / compute_core()
src/iap/integrity/lidar_araim.cpp — LidarAraim::run() / enumerate_hypotheses()
```

**故障假设空间**:
$$
\mathcal{H} = \{H_0\} \cup \{H_{sat,i}\}_{i=1}^{N_{sat}} \cup \{H_{const,c}\}_{c \in \mathcal{C}} \cup \{H_{trunk,l}\}_{l=1}^{N_{trunk}}
$$

**全解**:
$$
\hat{\mathbf{x}}^{(0)} = (\mathbf{G}^\top \mathbf{W}_0 \mathbf{G})^{-1} \mathbf{G}^\top \mathbf{W}_0 \mathbf{y}
$$

**子集解** (hypothesis $k$ 排除可疑测量):
$$
\hat{\mathbf{x}}^{(k)} = (\mathbf{G}^\top \mathbf{W}_k \mathbf{G})^{-1} \mathbf{G}^\top \mathbf{W}_k \mathbf{y}
$$

**分离向量与方差**:
$$
\mathbf{d}_k = \hat{\mathbf{p}}^{(0)} - \hat{\mathbf{p}}^{(k)}, \quad
\sigma_{ss,q,k} = \sqrt{\mathbf{e}_q^\top \Sigma_{ss,k} \mathbf{e}_q}
$$

**检测阈值**:
$$
T_{q,k} = K_{fa,k} \cdot \sigma_{ss,q,k}
$$

**故障保护级**:
$$
\mathrm{PL}_{q,k} = |d_{q,k}| + K_{fa,k} \cdot \sigma_{ss,q,k} + K_{md,k} \cdot \sigma_{q,k}
$$

**总 PL**:
$$
\mathrm{PL}_q = \max\left(\mathrm{PL}_{q,0}, \max_k \mathrm{PL}_{q,k}\right), \quad
\mathrm{HPL} = \max(\mathrm{PL}_E, \mathrm{PL}_N), \quad \mathrm{VPL} = \mathrm{PL}_U
$$

**LiDAR ARAIM**: 将 ICP 配准块 (`LidarAraimBlock`) 按 source_frame / target_frame / level_id 分组为 fault hypotheses,每个 block 包含 $\Lambda_B$ (6×6 信息矩阵) 和 $\eta_B$,通过 `LidarAraim::run()` 计算 LiDAR-only PL。

**创新点**: 将 ARAIM 的故障假设从传统的 GNSS satellite/constellation 扩展到 trunk landmark 和 LiDAR ICP block;结合 FGO 信息矩阵提供更严格的 $\Sigma^{(0)}$。

---

### §II-G. Future Protection-Level Prediction

**论文内容**: 沿候选轨迹预测 PL 演化——协方差传播 + 可见性依赖增长率 + GNSS ARAIM 几何预测 + LiDAR observability FIM + conservative fusion。

**代码**:
```
src/iap/planner/predicted_integrity.cpp       — PredictedIntegrityComputer::predict() / sigma_grow_at()
src/iap/planner/predicted_araim.cpp           — PredictedAraimComputer::predict_araim_pl()
src/iap/planner/future_pl_field_predictor.cpp — FuturePLFieldPredictor::query() / evaluate_point_direct()
src/iap/planner/pl_grid.cpp                   — PLGrid::interpolate()
src/iap/planner/lidar_observability_fim.cpp   — LidarObservabilityFIM
```

**协方差传播** (baseline):
$$
\sigma_{k+1}^2 = \sigma_k^2 + \sigma_{grow}^2(\mathbf{p}_k) \cdot \Delta t
$$

**可见性依赖增长率**:
$$
\sigma_{grow}(\mathbf{p}_k) = \sigma_0 \left[ 1 + \beta_v \frac{N_{nom} - N_{vis}(\mathbf{p}_k)}{N_{nom}} + \beta_\kappa \bar{\kappa}(\mathbf{p}_k) \right]
$$
其中 $N_{vis}(\mathbf{p}_k)$ 由 `VisibilityPredictor` 在位置 $\mathbf{p}_k$ 处射线检测得到,$\bar{\kappa}$ 为平均 canopy density。

**PL 预测**:
$$
\mathrm{PL}_k^{pred} = K_{pl} \cdot \sigma_k
$$

**ARAIM 几何预测** (r = 0 模式):
- 调用 `VisibilityPredictor::predict(pos, epoch)` → visible satellites
- 调用 `Araim::predict_geometry(visible_sats)` → pure geometry upper bound
- 得到 $\mathrm{HPL}^{gnss}$, $\mathrm{VPL}^{gnss}$

**LiDAR Observability FIM** (可选):
- 评估未来位置 $\mathbf{p}$ 周围 LiDAR 点云的 Fisher Information
- $\alpha_L(\mathbf{p})$ ∈ [0,1]: observability quality factor
- $\Delta \Lambda^{lidar}(\mathbf{p})$: 附加信息矩阵贡献

**Conservative Fusion**:
$$
\Lambda^{future}(\mathbf{p}) = \Lambda^{gnss}(\mathbf{p}) + \lambda_L \alpha_L(\mathbf{p}) \Delta \Lambda^{lidar}(\mathbf{p})
$$
$$
\mathrm{HPL}^{future} = K_{ff} \sqrt{\lambda_{\max}(\Sigma_{xy}^{future})} + b_H, \quad
\mathrm{VPL}^{future} = K_{ff} \sqrt{\Sigma_{zz}^{future}} + b_V
$$
$$
\mathrm{HPL} = \max(\mathrm{HPL}^{gnss}, \mathrm{HPL}^{fused}), \quad
\mathrm{VPL} = \max(\mathrm{VPL}^{gnss}, \mathrm{VPL}^{fused})
$$

**PL Grid 加速**: `PLGrid` 在当前位置周围构建 $N_x \times N_y \times N_z$ 的 3D 查找网格,`FuturePLFieldPredictor::rebuild_grid()` 预计算每个 cell 的 PL,`query()` 通过三线性插值实现 O(1) 查询。

**创新点**: Trajectory-dependent PL prediction——不同候选轨迹因穿越不同遮挡区域而得到不同的预测 PL;conservative fusion 保证不低估风险。

---

### §II-H. Planner-Integrity Cost and PI-lite Evaluation

**论文内容**: 将预测 PL 转化为规划代价,risk band 分类,PI-lite 评估器。

**代码**:
```
src/iap/planner/integrity_planner.cpp  — IntegrityPlanner::plan() / evaluate()
src/iap/planner/trajectory_generator.cpp — TrajectoryGenerator::generate()
src/iap/planner/pi_cost_adapter.cpp    — PICostAdapter
apps/phase2_planner_integrity_evaluator.cpp — PI-lite evaluator node
```

**候选轨迹生成** (motion primitives):
- 离散化网格: $\{v_{fwd}\} \times \{\omega_{yaw}\} \times \{v_{alt}\}$
- 积分 horizon $T$,步长 $dt$ → $\{\tau_1, \ldots, \tau_M\}$

**PI 代价函数**:
$$
J_{PI}(\tau) = w_H \left[\max(0, \mathrm{HPL} + m_H - \mathrm{HAL})\right]^2 +
w_V \left[\max(0, \mathrm{VPL} + m_V - \mathrm{VAL})\right]^2
$$

**总代价** ($\S5.2$):
$$
J(\tau) = w_{int} \cdot J_{PI}(\tau) + w_{turn} \cdot D_{turn}(\tau) + w_{mission} \cdot d_{goal}(\tau_{end}) + w_{smooth} \cdot E_{effort}(\tau)
$$

**Risk Band**:
$$
\mathrm{risk} = \begin{cases}
\texttt{UNSAFE}, & \mathrm{IM}_H < 0 \lor \mathrm{IM}_V < 0 \\
\texttt{MARGINAL}, & \mathrm{IM}_H < m_H \lor \mathrm{IM}_V < m_V \\
\texttt{SAFE}, & \text{otherwise}
\end{cases}
$$

**PI-lite 定位**: 当前 `phase2_planner_integrity_evaluator` 是只读评估器——它沿 EGO planner 的 B-spline 采样并报告预测 PL/AL/IM,但不修改 planner 的目标函数。Method 中应明确写为 "evaluates and reports predicted integrity risk along the planner trajectory, without closed-loop cost injection."

**创新点**: 将完整性从"事后监视"推进到"事前预测与规划",为 closed-loop integrity-aware planning 奠定框架。

---

### §II-I. Integrity-Aware A* Front-End Path Search (demo11)

**论文内容**: 将预测的 PL/AL 场注入 EGO planner 的 A* 前端搜索,使全局路径从"距离最短"变为"距离 + GNSS integrity 风险最小"。这是系统从 integrity monitoring 到 integrity-aware closed-loop navigation 的关键闭环。

**代码**:
```
src/iap/planner/future_pl_field_predictor.cpp  — FuturePLFieldPredictor (PL 场计算,复用 §II-G)
apps/phase2_planner_integrity_evaluator.cpp    — 发布 IntegrityFrontCostField (新增 publisher)
src/iap/sim/ego_planner_swarm_ws/src/bspline_opt/  — IntegrityCostMap (新增,订阅 cost field)
src/iap/sim/ego_planner_swarm_ws/src/path_searching/ — A* edge cost 修改
```
需求文档: `docs/dev_planner/req_astar.md`

**Architecture**: 跨进程解耦设计:
```
  IAP Process                              EGO Planner Process
  ┌────────────────────────┐              ┌────────────────────────────┐
  │ FuturePLFieldPredictor │              │                            │
  │   (复用 §II-G)         │              │  ┌──────────────────────┐  │
  │        │               │  PointCloud2 │  │  IntegrityCostMap    │  │
  │        ▼               │  /iap/       │  │  (订阅 + nearest     │  │
  │  IntegrityFrontCost─────┼──────────────┼──▶  neighbor 查询)     │  │
  │  Field (50m×50m, 1m)   │  2 Hz        │  └──────────┬───────────┘  │
  │  x,y,z,hpl,vpl,hal,val │              │             │              │
  │  cost,risk_band_code   │              │             ▼              │
  └────────────────────────┘              │  ┌──────────────────────┐  │
                                          │  │  A* Edge Cost        │  │
                                          │  │  edge = distance ×   │  │
                                          │  │  (1 + λ_front ×      │  │
                                          │  │   pi_cost(ratio))    │  │
                                          │  └──────────────────────┘  │
                                          └────────────────────────────┘
```

**Integrity Cost Field**: 在当前位置周围构建 $N_x \times N_y$ 的 2D cost field (高度固定为当前 odom z):
- 每个 cell: `{x, y, z, hpl, vpl, hal, val, cost, risk_band_code}`
- `FuturePLFieldPredictor` 对每个 cell 中心调用 `query(p_w, now)` → PL/AL → cost
- 发布频率 2 Hz,分辨率 1.0 m,覆盖 50 m × 50 m

**Front-End PI Cost** (ratio-based,区别于 back-end hinge-loss):

定义 risk ratio:
$$
r(\mathbf{p}) = \max\left( \frac{\mathrm{HPL}(\mathbf{p})}{\mathrm{HAL}(\mathbf{p})}, \frac{\mathrm{VPL}(\mathbf{p})}{\mathrm{VAL}(\mathbf{p})} \right)
$$

分段二次代价:
$$
c_{PI}(\mathbf{p}) = \begin{cases}
0, & r \leq 0.7 \\[4pt]
\left(\dfrac{r - 0.7}{0.3}\right)^2, & 0.7 < r \leq 1.0 \\[4pt]
1 + (r - 1.0)^2, & r > 1.0
\end{cases}
$$
并 clamp 到 $[0, 10]$。

**设计理由**: ratio-based cost 将 HPL/HAL 和 VPL/VAL 统一为无量纲量,不依赖后端 `phase2_pi_cost_weight_v`;三段式设计在 safe 区 ($r \leq 0.7$) 不惩罚,在 marginal 区 ($0.7 < r \leq 1.0$) 平滑增加,在 unsafe 区 ($r > 1.0$) 快速惩罚。

**A* 边代价修改**:

原始边代价:
$$
e_{orig}(\mathbf{p}_i, \mathbf{p}_j) = \|\mathbf{p}_i - \mathbf{p}_j\|
$$

Integrity-aware 边代价:
$$
e(\mathbf{p}_i, \mathbf{p}_j) = \|\mathbf{p}_i - \mathbf{p}_j\| \cdot \left(1 + \lambda_{front} \cdot c_{PI}(\mathbf{p}_j)\right)
$$

其中 $\lambda_{front}$ 控制 integrity 项的相对权重 (默认 2.0)。$c_{PI}(\mathbf{p}_j)$ 通过最近邻查询 `IntegrityCostMap` 获得;过期 (>1.0s)、无样本、或超查询半径 (1.5m) 的查询返回 0,退化为原始 EGO。

**两个搜索入口**:

1. **Rebound A\*** (绕障搜索): 在现有 `dyn_a_star` 每次重新规划时,边代价融入 integrity。覆盖短程避障场景。
2. **Global A\*** (全局路径): `planGlobalTraj()` 和 `planGlobalTrajWaypoints()` 在 50m × 50m × 50m 范围内以 0.5m step 做 integrity-aware A*,生成最多 80 个 waypoints;搜索失败时回退原始直线插点。

**Graceful Degradation**:
- Field 未订阅/未到达 → $c_{PI} = 0$,A* 退化为原始距离代价
- 查询点超出 field 范围 → $c_{PI} = 0$,field 覆盖区域外的路径不受影响
- Global A* 失败 → 回退原始 `planGlobalTraj()` 直线插点

**创新点**:

1. **Closed-loop integrity-aware front-end search**: 据我们所知,这是首次将预测的 ARAIM PL/AL 作为 A* 边代价注入无人机前端路径搜索,实现从 "monitoring" 到 "acting" 的闭环
2. **Two-stage integrity cost architecture**: 前端 A* 使用 ratio-based 分段代价 (计算廉价,引导全局拓扑),后端 B-spline 使用 hinge-loss (精确但计算昂贵,精化局部形状)
3. **Cross-process integrity field**: 通过 ROS2 PointCloud2 解耦 integrity prediction (IAP) 和 path search (EGO),不要求两个子系统共享内存或同步时钟
4. **Graceful degradation by design**: 任何环节失效都自动回退到标准 EGO,不引入新的 failure mode

---

### §II-J. Logging, Validation, and Experimental Metrics

**论文内容**: 实验配置、评估指标定义、数据记录与离线分析流程。

**代码**:
```
src/iap/util/run_log_manager.cpp  — RunLogManager (运行目录/产物管理)
src/iap/util/export_factors.cpp   — 因子导出
tools/ana_log.py                  — 综合分析入口
tools/phase1/validate_phase1_closed_loop.py  — Phase 1 验证
tools/phase2/validate_phase2_integrity_eval.py — Phase 2 验证
```

**关键指标**:
- 定位精度: ATE (Absolute Trajectory Error), RPE (Relative Pose Error)
- 完好性: $\mathrm{HPL}$, $\mathrm{VPL}$, $\mathrm{HAL}$, $\mathrm{VAL}$, $\mathrm{IM}_H$, $\mathrm{IM}_V$
- SAFE/UNSAFE epoch ratio, false-safe rate (error > PL), false-unsafe rate
- ICP 质量: RMSE, inlier fraction, condition number, $\gamma_{lidar}$
- GNSS 残差: per-satellite NIS, normalized residual
- 模块耗时: mean/p50/p95/p99 per module

---

## LaTeX 文件拆分建议 (给 Codex)

```
paper/method/
  00_system_overview.tex
  01_factor_graph_odometry.tex
  02_gnss_model_visibility.tex
  03_trunk_landmark_tdop.tex
  04_fgo_integrity_monitoring.tex
  05_araim_solution_separation.tex
  06_future_pl_prediction.tex
  07_planner_integrity_cost.tex          (PI-lite evaluation + back-end B-spline cost)
  08_integrity_aware_astar_search.tex    (demo11: front-end A* with integrity edge cost)
  09_experimental_setup.tex
  method_main.tex          (input 上述文件)
```

---

## 代码→论文章节快速映射

| 代码目录 | 对应 Method 小节 | 核心类 |
|----------|-----------------|--------|
| `apps/iap_rosnode.cpp` | §II-A System Overview | `IapRosNode` |
| `odometry/` + `common/` + `preprocess/` | §II-B FGO Odometry | `OdometryEstimationBase`, `IMUIntegration`, `CloudDeskewing` |
| `gnss/` + `map/local_occupancy.*` | §II-C GNSS Model | `GnssHandler`, `PseudorangeFactor`, `DopplerFactor`, `VisibilityPredictor` |
| `trunk/` | §II-D Trunk Landmark | `TrunkDetector`, `TrunkMap`, `TrunkFactor` |
| `integrity/integrity_monitor.*` + `integrity/fgo_information_manager.*` | §II-E FGO Integrity | `IntegrityMonitor`, `FGOInformationManager` |
| `integrity/araim.*` + `integrity/lidar_araim.*` | §II-F ARAIM | `Araim`, `LidarAraim` |
| `planner/predicted_integrity.*` + `planner/predicted_araim.*` + `planner/future_pl_field_predictor.*` + `planner/pl_grid.*` | §II-G Future PL | `PredictedIntegrityComputer`, `PredictedAraimComputer`, `FuturePLFieldPredictor` |
| `planner/integrity_planner.*` + `planner/pi_cost_adapter.*` + `planner/trajectory_generator.*` | §II-H PI Cost (back-end) | `IntegrityPlanner`, `PICostAdapter`, `TrajectoryGenerator` |
| `planner/future_pl_field_predictor.*` + `apps/phase2_planner_integrity_evaluator.cpp` + `sim/ego_planner_swarm_ws/src/bspline_opt/` + `sim/ego_planner_swarm_ws/src/path_searching/` | **§II-I A\* Search (front-end)** | `FuturePLFieldPredictor`, `IntegrityCostMap`, `AStar` |
| `util/run_log_manager.*` + `tools/ana_log.py` + `tools/phase*/` | §II-J Experiments | `RunLogManager`, validation scripts |

---

## 论文创新点层次 (Contribution Stack)

按创新程度从高到低排列,用于写 Introduction 的 contribution list:

1. **Closed-Loop Integrity-Aware Front-End Search** (§II-I): 首次将预测的 ARAIM PL/AL 作为 A* 边代价注入无人机前端路径搜索,实现从 integrity monitoring 到 integrity-aware closed-loop navigation 的完整闭环。Two-stage cost architecture (front-end ratio-based + back-end hinge-loss) 兼顾计算效率与精度
2. **FGO-Aware Integrity Monitoring** (§II-E): 从 iSAM2 边缘化直接提取 $\Sigma_{\mathbf{p}}$,替代传统 WLS-only $S_0$,PL 反映完整的 LiDAR-IMU-GNSS-trunk 融合不确定性
3. **Dynamic AL from Trunk Geometry** (§II-E): AL 来自实时树干检测而非固定保守值,使 PL/AL 比较具有操作意义
4. **Multi-Source ARAIM** (§II-F): 将故障假设从 GNSS 扩展到 trunk landmark + LiDAR ICP block,覆盖森林环境特有的故障模式
5. **Trajectory-Dependent PL Prediction** (§II-G): 不同候选轨迹因环境遮挡不同得到不同 PL,使 integrity 进入规划回路
6. **Canopy-Aware GNSS Model** (§II-C): $\sigma_{eff}$ 融入 canopy density $\kappa$,可见性预测使用 ray-cast
7. **PI Cost Formulation** (§II-H): hinge-loss 量化 back-end B-spline 的 PL-vs-AL margin,risk band 分类
