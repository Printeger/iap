# IAP: Spec vs. Implementation Status

> 目的：对照 `docs/spec/talk_spec.md` 与 `docs/spec/conventions.md`，梳理当前代码（继承自 GLIM）中**哪些已实现**、**哪些待实现**，作为后续 IAP-RQ 开发的基线参考。  
> 更新日期：2026-03-05  
> 作者：dev-agent (IAP-RQ-001)

---

## 1. 状态定义（Spec Section A）

| 状态分量 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **p** (position) | 世界系下位置 | ✅ 已实现 | `EstimationFrame::T_world_imu` (平移部分) |
| **q** (orientation) | 世界系下姿态四元数 | ✅ 已实现 | `EstimationFrame::T_world_imu` (旋转部分) |
| **v** (velocity) | 世界系下速度 | ✅ 已实现 | `EstimationFrame::v_world_imu` |
| **b_a** (accel bias) | 加速度计零偏 | ✅ 已实现 | `EstimationFrame::imu_bias[0:3]` |
| **b_g** (gyro bias) | 陀螺零偏 | ✅ 已实现 | `EstimationFrame::imu_bias[3:6]` |
| **δt** (clk_bias) | 接收机钟差（伪距用） | ❌ 未实现 | 待加入 `EstimationFrame` + GTSAM factor |
| **δṫ** (clk_drift) | 接收机钟速（多普勒用） | ❌ 未实现 | 待加入 `EstimationFrame` + GTSAM factor |

---

## 2. 估计器（Spec Section C）

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **IMU 预积分因子** | gtsam::ImuFactor，ba/bg 联合估计 | ✅ 已实现 | `src/iap/odometry/odometry_estimation_imu.cpp`, `src/iap/common/imu_integration.*` |
| **Fixed-lag smoother** | gtsam FixedLagSmootherExt，滑窗边缘化 | ✅ 已实现 | `odometry_estimation_imu.cpp`（`smoother_lag` 参数可配置） |
| **滑窗边缘化** | on_marginalized_keyframes 回调 | ✅ 已实现 | `odometry_estimation_gpu/cpu.cpp` |
| **LiDAR ICP 相对位姿因子** | scan-to-voxelmap GICP（CPU/GPU/CT 三路） | ✅ 已实现 | `odometry_estimation_cpu/gpu/ct.*` |
| **LiDAR ICP 质量报告** | inlier / rmse / cond 输出 | ⚠️ 部分 | CPU 中有 TODO 注释（`extract relative pose covariance`），quality 字段尚未输出 |
| **Σ_p 暴露接口** | 从 smoother 导出位置协方差块 | ❌ 未实现 | 需调用 `smoother.marginalCovariance(key)` 并封装到 EstimationFrame |
| **GNSS 伪距因子** | 含接收机 clk_bias，per-sat 独立 | ❌ 未实现 | 需新建 `src/iap/gnss/` |
| **GNSS 多普勒因子** | 含 clk_drift，速度投影（m/s） | ❌ 未实现 | 需新建 `src/iap/gnss/` |
| **卫星星历解算** | 播发星历解算卫星位置/速度 | ❌ 未实现 | 需集成 ephemeris 库或自行实现 |

---

## 3. 完整性监测（Spec Section B / D）

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **PL 计算（代理）** | `PL = K × √λ_max(Σ_p)` | ❌ 未实现 | 依赖 Σ_p 接口；需新建 `src/iap/integrity/` |
| **AL 计算** | 由障碍物距离动态给出（HAL/VAL） | ❌ 未实现 | 需新建 `src/iap/integrity/alert_limit.*` |
| **IM = AL − PL** | 完整性裕度 | ❌ 未实现 | — |
| **Mode machine** | NOMINAL / CAUTION / SEARCH | ❌ 未实现 | — |
| **Per-sat NIS gating** | 每颗卫星残差 NIS 统计，downweight γ_R | ❌ 未实现 | 依赖 GNSS 因子；需新建 `src/iap/integrity/gnss_integrity.*` |
| **FDE（贪心）** | global_NIS 超阈值触发卫星剔除 | ❌ 未实现 | — |
| **完整性融合报告** | PL / AL / IM / mode / per-sat {NIS, γ_R, exclude} | ❌ 未实现 | — |
| **全 ARAIM 假设集** | H0 + per-sat + constellation + trunk faults | ❌ 未实现 | Upgrade 项 |
| **HPL/VPL 计算** | 完整 ARAIM PL_q,k | ❌ 未实现 | Upgrade 项 |

---

## 4. 树干地标（Spec Section G — Upgrade）

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **LiDAR 树干分割** | 点云分割 + 圆柱拟合（中心、半径） | ❌ 未实现 | 需新建 `src/iap/trunk/` |
| **detection confidence** | 用于 trunk fault prior | ❌ 未实现 | — |
| **Trunk FGO 因子** | 几何观测因子入图（降低 Σ_p） | ❌ 未实现 | Upgrade 项 |
| **TDOP 指标** | 角度多样性 / 几何强度代理 | ❌ 未实现 | Upgrade 项 |

---

## 5. 预测层（Spec Section E）

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **GNSS 可见集预测 V̂(τ)** | 点云/地图遮挡 ray-check + 仰角掩膜 | ❌ 未实现 | 需新建 `src/iap/predictor/gnss_visibility.*` |
| **LiDAR 可观测性代理 Ô(τ)** | 地图 ICP 质量代理（含遮挡） | ❌ 未实现 | 需新建 `src/iap/predictor/lidar_observability.*` |
| **协方差传播 Σ → Σ_pred** | 经验增长模型（保留精确接口） | ❌ 未实现 | 需新建 `src/iap/predictor/covariance_propagator.*` |
| **PL_pred 计算** | Baseline: `K×√λ_max(Σ_pred)` | ❌ 未实现 | — |

---

## 6. 规划层（Spec Section F）

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **候选轨迹生成** | Motion primitives（v/ω/altitude 离散） | ❌ 未实现 | 需新建 `src/iap/planner/` |
| **完整性驱动代价** | `J = Σhinge(PL_pred−AL)² + λ_goal×dist + λ_u×effort` | ❌ 未实现 | — |
| **Receding horizon 执行** | Plan H 秒，执行 Δt 秒，循环 | ❌ 未实现 | — |

---

## 7. 基础设施 / 工具

| 功能 | 描述 | 实现状态 | 位置 |
|---|---|---|---|
| **点云预处理** | 去运动畸变、降采样、协方差估计 | ✅ 已实现 | `src/iap/preprocess/` |
| **ROS2 扩展模块接口** | 插件化，动态 so 加载 | ✅ 已实现 | `src/iap/util/extension_module*` |
| **配置系统（JSON）** | 参数解析与序列化 | ✅ 已实现 | `src/iap/util/config*` |
| **可视化 Viewer** | 标准/交互/地图编辑器 | ✅ 已实现 | `src/iap/viewer/` |
| **全局建图** | sub_mapping + global_mapping + pose graph | ✅ 已实现 | `src/iap/mapping/` |
| **IMU 健康验证** | 基础饱和检测 | ✅ 已实现 | `src/iap/common/imu_validation.*` |
| **GNSS ROS handler** | ROS 话题接入、IMU 健康扩展 | ❌ 未实现 | 需新建 `src/iap/gnss/ros_gnss_handler.*` |

---

## 8. 总览

```
已实现（继承自 GLIM）：
  ✅ IMU 预积分 + 滑窗 Fixed-lag smoother
  ✅ LiDAR ICP scan-to-map（CPU/GPU/CT）
  ✅ 基础状态：p, v, q, b_a, b_g
  ✅ 边缘化回调
  ✅ 点云预处理（去畸变、法向协方差）
  ✅ ROS2 插件体系 + 可视化

待实现（IAP 新增）：
  ❌ 状态扩展：clk_bias, clk_drift
  ❌ GNSS 紧耦合因子（伪距 + 多普勒）
  ❌ Σ_p 导出接口
  ❌ ICP 质量报告（inlier/rmse/cond）
  ❌ 完整性监测：PL / AL / IM / mode / NIS / FDE
  ❌ 预测层：V̂(τ) / Ô(τ) / Σ_pred / PL_pred
  ❌ 规划层：候选轨迹 / J(τ) / receding horizon
  [Upgrade] ❌ 树干地标 + TDOP + 全 ARAIM
```

---

## 9. 建议的新目录结构

```
src/iap/
  gnss/          # GNSS 因子、星历解算、ROS handler          → IAP-RQ-020
  integrity/     # PL/AL/IM、mode、NIS gating、ARAIM         → IAP-RQ-200~240
  predictor/     # 可见集预测、协方差传播、PL_pred            → IAP-RQ-310~320
  planner/       # 候选轨迹、J(τ)、receding horizon           → IAP-RQ-400~410
  trunk/         # 树干分割、圆柱拟合、TDOP                   → IAP-RQ-100~120 [Upgrade]
  health/        # IMU/LiDAR 健康度扩展                       → IAP-RQ-040~050 (partial)
  (已有)
  common/        # IMU 积分、去畸变、协方差估计
  odometry/      # 估计器核心（需 patch：Σ_p 导出 + GNSS factor hook）
  mapping/       # 全局建图
  preprocess/    # 点云预处理
  util/          # 工具（config, logging, …）
  viewer/        # 可视化
```

---

*参考文件：`docs/spec/talk_spec.md`、`docs/spec/conventions.md`*  
*追溯：IAP-RQ-001（生成本文档）*
