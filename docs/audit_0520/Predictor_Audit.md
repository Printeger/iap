# Predictor Audit

本文对照 `src/iap` 当前代码、`docs/audit_0520/IAP_Architecture_Audit.md` 以及设计文档 `docs/audit_0520/iap_reprot_v2_0.pdf`，梳理 Predictor 模块的设计目标、当前实现、输入输出、运行流程，以及两者的相同点和差异。

核心结论：

- 设计文档中的 Predictor 是 planner-side advisory 模块，用于预测候选位置或轨迹上的未来 PL/IM/PI cost；它不替代 Current ARAIM monitor。
- 当前代码已经具备 GNSS geometry advisory PL、LiDAR observability/FIM、`FuturePLFieldPredictor`、`PLGrid`、`PICostAdapter`、`UnifiedRiskGrid` 和 evaluator cost field 发布路径。
- 但默认运行路径仍偏保守和兼容：`phase2_planner_integrity_evaluator` 默认 `pl_model=constant_current`，未来预测、FIM-add、URG 都需要通过参数显式启用。
- 当前实现是“可用能力已存在，但默认系统集成还不是完整设计态”的状态。

## 1. Audit Scope

本次 audit 只覆盖 planner-side predictor 相关路径，不重新评估 Current ARAIM 的当前监控数学正确性。

主要代码范围：

- `include/iap/planner/integrity_snapshot.hpp`
- `src/iap/planner/integrity_snapshot.cpp`
- `include/iap/planner/predicted_araim.hpp`
- `src/iap/planner/predicted_araim.cpp`
- `include/iap/planner/gnss_geometry_pl_predictor.hpp`
- `src/iap/planner/gnss_geometry_pl_predictor.cpp`
- `include/iap/planner/lidar_observability_fim.hpp`
- `src/iap/planner/lidar_observability_fim.cpp`
- `include/iap/planner/future_pl_field_predictor.hpp`
- `src/iap/planner/future_pl_field_predictor.cpp`
- `include/iap/planner/pl_grid.hpp`
- `src/iap/planner/pl_grid.cpp`
- `include/iap/planner/pi_cost_adapter.hpp`
- `src/iap/planner/pi_cost_adapter.cpp`
- `include/iap/planner/unified_risk_grid.hpp`
- `src/iap/planner/unified_risk_grid.cpp`
- `apps/phase2_planner_integrity_evaluator.cpp`

参考文档中的关键章节：

- `IAP_Architecture_Audit.md` 的 `E.2 Future PL Prediction Chain (for Planner)`、`E.3 Risk / Cost Field Publication Chain`、`F.7 FuturePLFieldPredictor`、`F.8 PredictedAraimComputer`。
- `iap_reprot_v2_0.pdf` 的 3.0.5、3.3、3.4、3.5 相关内容：Certified Monitoring vs Advisory Prediction、Advisory PL predictor、Predicted PL Map、Unified Risk Grid、planner query。

## 2. Design-Side Predictor Model

设计文档中的 Predictor 模块定位如下：

| 项目 | 设计目标 |
|---|---|
| 模块性质 | Advisory planning signal，不是 certified monitor |
| 查询对象 | 候选未来位置 `p_i` 或轨迹采样点 `p(t_k)` |
| 输入 | 当前 localization snapshot、GNSS epoch、先验 covariance/information、LiDAR/map geometry、occupancy/ESDF、环境 AL 模型 |
| GNSS 预测 | 根据候选位置预测 satellite visibility 和 geometry information |
| LiDAR 预测 | 根据局部地图几何、surface primitives、FIM/TDOP/observability 估计 LiDAR information |
| 融合 | 在 position-domain information matrix 上融合 `Lambda_prior + Lambda_G + Lambda_L` |
| 输出 | `PL_pred`、`AL`、`IM = AL - PL_pred`、`J_PI`、风险 flags、age/staleness |
| 缓存 | 构建 rolling multi-layer PL/IM/cost map，供 A* 和 B-spline 共用 |
| fallback | stale/unavailable field 触发 unknown-risk penalty 或回退到标准 planner/current monitor supervision |

设计上有一个明确边界：

- Current ARAIM monitor 使用当前真实测量、残差和 fault hypotheses，输出当前 certified PL/alert 状态。
- Future Predictor 使用未来候选位置上的可见性、几何和地图信息，输出 advisory PL/IM/cost，不应被称为当前 certified ARAIM PL。

## 3. Current Code Predictor Inventory

当前代码中的 predictor 相关模块可以分成六层。

| 层级 | 当前代码 | 角色 |
|---|---|---|
| Snapshot | `IntegritySnapshot`、`IntegritySnapshotBuilder` | 保存当前 pose、current integrity、GNSS epoch、可选 prior information、LiDAR snapshot 摘要 |
| GNSS advisory | `PredictedAraimComputer`、`GnssGeometryPlPredictor` | 对 query position 做 visibility prediction，再做 geometry-only PL proxy |
| LiDAR advisory | `LidarObservabilityFim` | 支持 legacy point-cloud LOI/alpha 路径和 primitive-based advisory FIM 路径 |
| Future PL query | `FuturePLFieldPredictor`、`FuturePLQueryResult` | 统一 GNSS/LiDAR/FIM-add/direct/grid 查询输出 |
| PL cache | `PLGrid` | 缓存 `FuturePLQueryResult`，支持 trilinear interpolation 和 PL gradient |
| PI / risk map | `PICostAdapter`、`UnifiedRiskGrid` | 把 advisory HPL/VPL 与 HAL/VAL 转成 IM、PI cost、flags、staleness-aware query |
| Runtime bridge | `phase2_planner_integrity_evaluator` | ROS evaluator，订阅 odom/integrity/GNSS/cloud/bspline，生成 CSV 和 PointCloud2 front cost field |

### 3.1 Snapshot

`IntegritySnapshot` 当前字段包括：

- `p_wb`、`q_wb`
- `CurrentIntegrityState current`
- `GnssEpoch gnss_epoch`
- 可选 `lambda_base_pos`
- 可选 LiDAR snapshot / LiDAR ARAIM summary

`IntegritySnapshotBuilder` 能从 input 复制这些内容，但 evaluator 当前主要填充 pose、current integrity、GNSS epoch；没有在 evaluator 路径里接入完整的 `lambda_base_pos`、LiDAR ARAIM result、LiDAR block snapshot。

### 3.2 GNSS Advisory Predictor

`PredictedAraimComputer::predict_araim_result(pos)` 的当前流程：

1. 如果无 GNSS epoch，返回 `fallback_pl`，reason 为 `no_gnss_epoch`。
2. 调用 `VisibilityPredictor::predict(pos, epoch)`。
3. 过滤可见卫星和 excluded satellites。
4. 构造 `GnssGeometrySat` 列表。
5. 调用 `GnssGeometryPlPredictor::predict(geom)`。
6. 输出 advisory `hpl/vpl/pl_scalar/pl_e/pl_n/pl_u/pl_ff/sigma/pdop/n_vis/n_hypotheses`。

`GnssGeometryPlPredictor` 已经不 include current ARAIM solver header。它使用 geometry-only、`r=0` 的设计矩阵和 leave-one-satellite subset 近似，输出 advisory PL proxy。

### 3.3 LiDAR Advisory Predictor

`LidarObservabilityFim` 有两条路径：

- `evaluate(pos, map_points, current)`：legacy observability path，用点云近邻构造 `delta_lambda`，再结合 `lidar_alpha`、TDOP、condition、excluded trunk 数量等调制。
- `evaluate_advisory_fim(pos, primitives, current)`：primitive/FIM path，用 `LidarFimPrimitive` 的 normal 和 confidence 累加 LiDAR advisory information matrix。

`make_lidar_fim_primitives()` 支持从点云 normal 或 PCA 生成 primitive。evaluator 在 `on_cloud()` 里会维护 local occupancy、下采样 predictor point cloud，并生成 LiDAR FIM primitives。

### 3.4 FuturePLFieldPredictor

`FuturePLFieldPredictor::evaluate_point()` 当前支持三种主要模式：

| 模式 | 触发参数 | 行为 |
|---|---|---|
| GNSS-only advisory | 默认路径 | 调用 `PredictedAraimComputer`，输出 GNSS advisory HPL/VPL |
| Legacy fused FIM grid | `use_fused_fim_grid=true` 且 `use_lidar_observability=true` | GNSS base information + `lidar_alpha * delta_lambda`，再计算 fused PL；最终 `hpl/vpl = max(gnss, fused)`，保证不低于 GNSS |
| Formal advisory FIM-add | `use_advisory_fim_add=true` | 计算 prior/GNSS/LiDAR FIM，求 inverse covariance，使用 `K_H_adv/K_V_adv + bias/scale` 得到 `hpl_adv/vpl_adv` |

`FuturePLFieldPredictor::query()` 可先查 `PLGrid`；grid miss、grid stale 或未启用时回退到 direct query。

### 3.5 PLGrid

`PLGrid` 当前缓存的是 `FuturePLQueryResult`，包括：

- advisory PL fields
- GNSS/LiDAR/FIM diagnostics
- fallback reason
- grid generation/age/build time
- PL gradients

它不是完整 URG。它不直接保存 AL、IM、PI cost、occupancy、ESDF。

### 3.6 PICostAdapter / UnifiedRiskGrid / Evaluator

`PICostAdapter` 输入 HAL、VAL、advisory HPL、advisory VPL，输出：

- horizontal/vertical margin
- hinge cost / ratio cost
- total PI cost
- risk band
- gradient fields

`UnifiedRiskGrid` 是更接近设计文档中 rolling multi-layer map 的实现，保存：

- ESDF/occupancy
- HAL/VAL
- advisory HPL/VPL/PL
- advisory IM
- PI cost and gradient
- flags、age、staleness、unknown risk

`phase2_planner_integrity_evaluator` 是当前和 ego_planner 连接最实际的 runtime bridge：

- 订阅 odom、integrity report、GNSS measurements/ephemeris、map cloud、bspline。
- 维护 occupancy、GNSS epoch、LiDAR primitives。
- 通过 `pl_model` 决定使用 current PL、GNSS advisory direct、`FuturePLFieldPredictor`、或 FIM-related path。
- 发布 `/iap/integrity_front_cost_field`，供 ego_planner risk overlay 使用。

## 4. Input Comparison

| 输入类别 | 设计文档 | 当前代码支持 | 当前 evaluator 实际填充状态 |
|---|---|---|---|
| 当前 pose | `T_hat_t` / current pose | `IntegritySnapshot::p_wb/q_wb` | 已从 odom 填充 |
| Current monitor PL/AL/IM | 当前 certified monitor snapshot | `CurrentIntegrityState` | 已从 `/iap/integrity` 填充 |
| GNSS epoch | visible sats、sat positions、URA、elevation/azimuth | `GnssEpoch` / `SatObs` | 已由 range/ephem/receiver origin 构建 |
| Local occupancy | occupancy / raycast visibility | `LocalOccupancyGrid` | 已由 map point cloud 更新 |
| Local ESDF / clearance | 用于 AL、PI cost、planner query | 当前用 nearest cloud distance 近似 clearance；URG 保存 `esdf_m` 字段 | 有近似实现，不是完整 ESDF pipeline |
| Prior covariance / information | `Sigma_p`、`Lambda_prior` | `IntegritySnapshot::lambda_base_pos` | Builder 支持，但 evaluator 当前没有填入 |
| LiDAR block summary | VGICP blocks、quality、condition、age | `LidarAraimSnapshot` summary fields | Builder 支持，但 evaluator 当前没有填入 |
| LiDAR map geometry | surface primitives / local feature geometry | point cloud + `LidarFimPrimitive` | 已由 cloud callback 生成 primitives |
| Future horizon | `p_i, t + tau_i` | query 输入是 position；timestamp 主要用于 grid age/staleness | 显式 horizon 建模有限，轨迹采样有 sample time，但 GNSS/FIM 预测基本使用当前 epoch/snapshot |

## 5. Output Comparison

| 输出类别 | 设计文档 | 当前代码 |
|---|---|---|
| GNSS advisory PL | predicted GNSS geometry PL / information | `PredictedAraimResult`、`gnss_hpl/gnss_vpl` |
| LiDAR advisory info | LiDAR observability/FIM、TDOP、condition | `LidarObservabilityResult`、`LidarAdvisoryFimResult`、`lidar_alpha/lidar_tdop/lidar_condition` |
| Fused advisory PL | `PL_pred` from fused information | `FuturePLQueryResult::hpl/vpl/pl_scalar` and `hpl_adv/vpl_adv` |
| PL query result | Future query output with fallback flags | `FuturePLQueryResult` 包含 validity、fallback reason、diagnostics、grid metadata |
| PL grid | cached predicted PL map | `PLGrid` caches `FuturePLQueryResult` only |
| AL / IM / PI cost | multi-layer PL/AL/IM/cost map | evaluator rows、`PICostAdapter`、`UnifiedRiskGrid` |
| Planner topic | planner query / risk map | `/iap/integrity_front_cost_field` PointCloud2，包含 HPL/VPL/HAL/VAL/IM/cost/risk band 等字段 |
| Debug / audit output | runtime diagnostics | evaluator CSV：`future_integrity_snapshot.csv`、`integrity_along_planner_traj.csv`、`pl_grid_voxels.csv`、`urg_grid_voxels.csv` |

当前输出语义上已经区分了：

- `current_HPL/current_VPL/current_PL`：current certified monitor fields。
- `PL_H_pred/PL_V_pred/PL_pred`：planner advisory prediction fields。
- `gnss_hpl/gnss_vpl`：GNSS advisory proxy。
- `fused_hpl/fused_vpl`：legacy fused advisory fields。
- `hpl_adv/vpl_adv`：FIM-add advisory fields。
- `advisory_hpl_used/advisory_vpl_used`：PI cost 实际采用的 advisory PL。

## 6. Flow Comparison

### 6.1 Design Flow

设计文档中的 predictor flow 可以概括为：

```text
Localization Snapshot
  -> GNSS visibility / geometry information at candidate position
  -> LiDAR observability / FIM at candidate position
  -> Lambda_pred = Lambda_prior + Lambda_G + Lambda_L
  -> Sigma_pred = inverse(Lambda_pred)
  -> PL_pred = K * sigma + bias/overbound
  -> AL from environment
  -> IM = AL - PL_pred
  -> PI cost + risk flags
  -> rolling PL/IM/cost map
  -> A* and B-spline query the same map
```

### 6.2 Current Code Flow

当前 evaluator 的实际 flow 由 `pl_model` 和多个 feature flags 控制：

```text
ROS inputs:
  odom + /iap/integrity + GNSS range/ephem + point cloud + bspline

on_integrity:
  CurrentIntegrityState
  -> IntegritySnapshotBuilder
  -> FuturePLFieldPredictor::update_snapshot

on_cloud:
  point cloud
  -> LocalOccupancyGrid
  -> lidar map points
  -> make_lidar_fim_primitives
  -> FuturePLFieldPredictor::set_lidar_*()

pl_values(pos):
  if pl_model == constant_current:
    use current HPL/VPL
  else if field predictor enabled:
    FuturePLFieldPredictor::query(pos, now)
  else:
    PredictedAraimComputer::predict_araim_result(pos)
  if failed:
    fallback_pl or current fallback mode

PI:
  select_pi_advisory_pl(pl)
  -> PICostAdapter::evaluate(HAL, VAL, HPL_adv, VPL_adv)

Optional URG:
  rebuild_unified_risk_grid(center, stamp)
  -> make_unified_risk_voxel for each voxel
  -> stores AL, advisory PL, IM, PI cost, flags, age

Published field:
  build_integrity_front_cost_samples()
  -> /iap/integrity_front_cost_field
```

### 6.3 `pl_model` Behavior

| `pl_model` / flags | 当前行为 |
|---|---|
| `constant_current` | 默认兼容模式，直接使用 current monitor HPL/VPL，不是真正 future prediction |
| no field predictor flags | 调用 `PredictedAraimComputer` 做 GNSS geometry advisory direct query |
| `use_pl_grid=true` | `FuturePLFieldPredictor` 构建/查询 `PLGrid`，grid miss 后 direct query |
| `pl_model=fused_fim_grid` | 启用 legacy LiDAR observability fusion path |
| `use_advisory_fim_add=true` | 启用 formal FIM-add path，可选 `use_lidar_advisory_fim` |
| `use_unified_risk_grid=true` | 构建 URG，多层保存 AL/PL/IM/cost/flags；launch 默认通常为 false |

## 7. Same Points

当前代码和设计文档一致的点：

1. Predictor 被明确标注为 advisory/planner-side，不应作为 certified current monitor PL。
2. Current ARAIM 和 Future Predictor 在代码所有权上已经分离；`GnssGeometryPlPredictor` 不依赖 current ARAIM solver header。
3. GNSS path 已包含 visibility prediction、effective sigma、geometry-only PL proxy。
4. LiDAR advisory path 已存在，且包含点云 observability 和 primitive-based FIM 两类实现。
5. `FuturePLQueryResult` 已承载 GNSS、LiDAR、fused/FIM-add、fallback、grid metadata 和 diagnostics。
6. Grid cache、generation、age、staleness、fallback reason 等工程概念已实现。
7. `PICostAdapter` 已把 HPL/VPL 和 HAL/VAL 转成 margin、PI cost 和 risk band。
8. `UnifiedRiskGrid` 的字段结构与设计文档的 multi-layer risk map 思路高度一致。
9. evaluator 已能发布 planner-consumable PointCloud2 cost field，并记录较完整 CSV。

## 8. Differences and Gaps

当前代码和设计文档的主要差异：

| 差异 | 当前状态 | 影响 |
|---|---|---|
| 默认运行路径 | `phase2_planner_integrity_evaluator` 默认 `pl_model=constant_current` | 默认并不真正使用 future predictor，只是用 current PL 做兼容诊断 |
| Snapshot 完整度 | `IntegritySnapshot` 支持 prior/LiDAR snapshot，但 evaluator 主要填 pose/current/GNSS epoch | FIM-add 中 `prior_information(snapshot)` 经常可能走 missing prior fallback |
| Formal FIM-add | `use_advisory_fim_add` 后才启用 | 设计主路径存在，但不是默认路径 |
| Legacy fusion 仍存在 | `use_fused_fim_grid` 使用 LiDAR alpha + delta_lambda，并对最终 PL 做 `max(GNSS, fused)` | 更保守，但不是设计中纯 FIM-add 后直接输出 fused advisory PL 的形式 |
| PLGrid 层级 | `PLGrid` 只缓存 `FuturePLQueryResult` | AL/IM/PI cost 不在 PLGrid，而在 evaluator/URG 中 |
| URG 默认状态 | `use_unified_risk_grid=false` 是 launch/default 常见设置 | 完整 multi-layer map 能力存在，但不是默认集成路径 |
| Planner 集成方式 | `IntegrityPlanner` 是并行 C++ planner，active ego_planner integration 走 evaluator PointCloud2 field | 设计中的 planner query abstraction 尚未完全收敛到一个统一入口 |
| Future horizon | 设计强调 `p_i, t + tau_i`；当前 query 多用当前 epoch/snapshot 和 query position | 对卫星未来运动、地图未来变化、时间相关 uncertainty propagation 建模有限 |
| ESDF | 设计中 URG 应保存 ESDF；当前 evaluator 多用 point cloud nearest distance 近似 clearance | AL/gradient 与真实 ESDF map 的一致性仍需后续接入验证 |
| Unknown/stale policy | URG 支持 unknown/stale penalty；legacy front field path 较简单 | 只有启用 URG 时才更接近设计中的 stale field policy |

## 9. Practical Development Notes

后续继续开发 predictor 功能时，建议按以下优先级推进：

1. 先决定默认运行语义：如果目标是让系统真正使用 future predictor，应在 launch/config 中从 `constant_current` 切换到明确的 advisory predictor 模式，并记录 fallback policy。
2. 补齐 `IntegritySnapshot` runtime 输入：把 prior information `lambda_base_pos`、LiDAR ARAIM snapshot/result 或 source-wise FIM 接入 evaluator，否则 FIM-add 会缺少设计文档中的 `Lambda_prior`。
3. 收敛 LiDAR advisory 路径：明确 legacy `use_fused_fim_grid` 是否仅保留为兼容模式，主路径是否统一为 `use_advisory_fim_add + use_lidar_advisory_fim`。
4. 明确 PLGrid 和 URG 的职责边界：`PLGrid` 适合轻量 PL cache；完整 planner risk field 应以 `UnifiedRiskGrid` 为主。
5. 增强时间维度：把轨迹采样时间、GNSS epoch propagation、future covariance growth 或 `tau_i` 显式纳入 predictor query。
6. 将 active ego_planner 接口和 `IntegrityPlanner` 的关系写清楚：当前实际集成是 PointCloud2 risk overlay，不是 `IntegrityPlanner` 直接控制 ego_planner。
7. 对默认参数补测试或 launch audit：至少覆盖 GNSS-only advisory、FIM-add advisory、URG enabled、stale/missing field fallback 四类场景。

简短状态判断：

```text
设计目标：Future Predictor = advisory GNSS/LiDAR information fusion + PL/IM/cost map for planner.
当前代码：核心组件基本存在，但默认运行偏 compatibility/current monitor mirror。
主要缺口：runtime 输入完整度、默认启用策略、URG 默认接入、显式 future horizon。
```
