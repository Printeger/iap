# Safety Grid Map Builder Input Audit

本文对照 `docs/audit_0520/iap_reprot_v2_0.pdf`、`docs/audit_0520/IAP_Architecture_Audit.md`、`docs/audit_0520/Predictor_Audit.md` 以及当前 `src/iap` 代码，整理 Safety Grid map builder 需要的输入，以及 Predictor 输出应保持的形式。

核心结论以代码为准：

- Predictor 应继续保持单点 advisory query，只输出候选位置上的 GNSS/LiDAR/fused advisory integrity 结果。
- Safety Grid map builder 负责 rolling grid 采样、缓存、staleness、AL、IM、PI cost、flags、gradient 和 planner-facing schema。
- 当前 ego_planner 最实际的 planner 入口不是完整 URG 类型，而是 `GridMap` risk overlay 对 `/iap/integrity_front_cost_field` 的消费。
- 因此新 Safety Grid 模块需要同时满足设计文档中的 URG 语义，以及当前 ego_planner 的 PointCloud2 兼容字段。

## 1. 调研范围和结论

本次调研聚焦 planner 需要什么 map field，而不是重新评估 ARAIM 或 Predictor 数学模型。

主要代码范围：

- `apps/phase2_planner_integrity_evaluator.cpp`
- `include/iap/predictor/predictor_types.hpp`
- `include/iap/planner/future_pl_query_result.hpp`
- `include/iap/planner/unified_risk_grid.hpp`
- `include/iap/planner/pi_cost_adapter.hpp`
- `sim/ego_planner_swarm_ws/src/planner/plan_env/include/plan_env/grid_map.h`
- `sim/ego_planner_swarm_ws/src/planner/plan_env/src/grid_map.cpp`
- `sim/ego_planner_swarm_ws/src/planner/path_searching/src/dyn_a_star.cpp`
- `sim/ego_planner_swarm_ws/src/planner/bspline_opt/src/bspline_optimizer.cpp`

当前实现的关键事实：

| 结论 | 代码现状 |
|---|---|
| Predictor 不应直接承担建图职责 | 新增 `PredictorQueryResult` 已只包含 GNSS/LiDAR/fused advisory 结果，不包含 AL/IM/PI/grid metadata |
| Safety Grid 需要 rolling cache | 设计文档 3.4 要求 rolling multi-layer map；当前 `UnifiedRiskGrid` 已有接近实现 |
| ego_planner 当前主要吃 PointCloud2 | `GridMap::ingestRiskOverlayCloud()` 订阅 `/iap/integrity_front_cost_field` |
| risk overlay 最小字段很少 | 必需 `x,y,z,hpl_adv,vpl_adv,flags`，另需 `stamp_s` 或 `source_age_s` |
| AL/PI 在 GridMap 内会重算 | `riskOverlayHal()`、`riskOverlayVal()`、`riskOverlayPiCost()` 用 occupancy clearance / z 边界 / risk 参数计算 |
| B-spline 有 overlay 和 legacy 两条路径 | overlay 通过 `GridMap` 插值并本地差分梯度；legacy 读取 `/iap/integrity_cost_field` 的 cost/gradient |

## 2. 设计文档中的 PL Map Builder / URG

设计文档把 Predictor 与 map builder 分开：

- Predictor model：对单个候选位置和时间 `p, t + tau` 计算 advisory prediction。
- PL Map Builder / URG：在 rolling local grid 上采样 Predictor，缓存多层字段，给 A* 和 B-spline 共用。

设计侧核心公式和流程：

```text
Lambda_pred_i = Lambda_prior + Lambda_G(p_i, t + tau_i) + Lambda_L(p_i)
Sigma_pred_i = inverse(Lambda_pred_i)
PL_pred_i = K_i * sqrt(e_q^T Sigma_pred_i e_q) + b_i
IM_i = AL_i - PL_pred_i
J_PI_i = margin_to_cost(PL_pred_i, AL_i)
```

设计侧 URG layer：

| Layer | 设计含义 | 应归属模块 |
|---|---|---|
| `Occ` | 占据地图 | Safety Grid / map source |
| `ESDF` | clearance / obstacle distance | Safety Grid / map source |
| `AL` | 环境 alert limit | Safety Grid |
| `Lambda_G` | GNSS advisory information | Predictor 输出，可被 Safety Grid 记录为诊断层 |
| `Lambda_L` | LiDAR advisory information | Predictor 输出，可被 Safety Grid 记录为诊断层 |
| `PLpred` | fused advisory predicted PL | Predictor 输出的核心值 |
| `IM` | `AL - PLpred` | Safety Grid |
| `JPI` | planner PI cost | Safety Grid / planner adapter |
| `age` | voxel freshness | Safety Grid |
| `flags` | valid/stale/unknown/fallback/source | Safety Grid 聚合 Predictor 与 map 状态 |

设计文档中也强调 A* 与 B-spline 应查询同一个 PI cost field，避免前端和后端风险语义不一致。

## 3. 当前代码中的 Planner 消费路径

当前代码里，planner consumption 主要分为三条路径。

| 路径 | 输入源 | 消费方 | 实际行为 |
|---|---|---|---|
| `GridMap` risk overlay | `/iap/integrity_front_cost_field` | `GridMap`、`AStar`、`BsplineOptimizer` | 推荐路径；ingest advisory HPL/VPL，GridMap 本地算 cost 和 gradient |
| B-spline legacy field | `/iap/integrity_cost_field` | `BsplineOptimizer::onIntegrityCostField()` | 读取 cost、gradient、risk_band，按最近邻样本给控制点加 cost/gradient |
| front integrity legacy query | `/iap/integrity_front_cost_field` | `BsplineOptimizer::onFrontIntegrityCostField()` -> A* callback | 读取 hpl/vpl/hal/val/cost，按最近邻 query，overlay 启用时会被 overlay 路径替代 |

### 3.1 GridMap risk overlay

`GridMap::ingestRiskOverlayCloud()` 的实际最低输入要求：

| 字段 | 必需 | 用途 |
|---|---:|---|
| `x, y, z` | 是 | 样本位置，写入 GridMap voxel |
| `hpl_adv` | 是 | horizontal advisory PL |
| `vpl_adv` | 是 | vertical advisory PL |
| `flags` | 是 | 写入 risk overlay voxel flags |
| `stamp_s` | 二选一 | 样本时间戳，用于 age/staleness |
| `source_age_s` | 二选一 | 兼容 source age；可修正 stale stamp |

当前 `RiskOverlayVoxel` 只保存：

```text
hpl_adv, vpl_adv, stamp_s, age_s, flags, pi_cost
```

也就是说，虽然 front cost field 发布了 21 个字段，但 `GridMap` overlay 的最小计算闭环只依赖 advisory HPL/VPL、时间和 flags。

### 3.2 AStar overlay edge cost

`AStar` 在 overlay 模式下调用：

```text
GridMap::integrateRiskOnEdge(snapshot, current_pos, neighbor_pos, &integrity_cost)
```

`GridMap` 会沿 edge 采样，调用 `queryRiskInterpolated()`，再把平均 risk cost 加到 A* edge cost：

```text
edge_cost = static_cost + lambda_integrity_cost * metric_edge_length * integrity_cost
```

这意味着 Safety Grid 给 A* 的关键不是 dense diagnostics，而是可插值、可积分的 scalar PI cost field。

### 3.3 B-spline overlay cost / gradient

`BsplineOptimizer::calcIntegrityCost()` 在 overlay 模式下：

1. 对每段 B-spline 按 `risk_overlay_bspline_samples_per_segment` 采样。
2. 调 `GridMap::queryRiskInterpolated(snapshot, p)` 取 cost。
3. 调 `GridMap::queryRiskGradient(snapshot, p)` 做本地有限差分梯度。
4. 用 B-spline basis 把 sample gradient 分配回控制点。

因此 overlay 路径不要求 PointCloud2 直接提供 `grad_x/y/z`。当前 front field 里 `grad_x/y/z` 也是 0。

### 3.4 B-spline legacy cost field

legacy `/iap/integrity_cost_field` 仍存在，schema 为 16 字段：

```text
x, y, z,
hpl, vpl,
hal, val,
im_h, im_v, im_min,
cost,
grad_x, grad_y, grad_z,
risk_band, risk_band_code
```

`BsplineOptimizer::onIntegrityCostField()` 只实际读取：

```text
x, y, z, cost, grad_x, grad_y, grad_z, risk_band
```

该路径是兼容路径，不应作为新 Safety Grid 的唯一目标接口。

## 4. ego_planner 对 Safety Grid 的实际输入需求

从当前 planner 代码看，Safety Grid 对 ego_planner 的输出需要分两层理解。

### 4.1 当前最小可运行接口

若目标是喂给 `GridMap` risk overlay，Safety Grid publish 的 PointCloud2 最小兼容字段是：

```text
x, y, z,
hpl_adv, vpl_adv,
stamp_s 或 source_age_s,
flags
```

字段约束：

- `frame_id` 必须为空或等于 `GridMap` 的 `frame_id`，当前通常是 `map`。
- `hpl_adv/vpl_adv` 必须 finite，否则样本被跳过。
- `stamp_s` 必须 finite，或者由 finite `source_age_s` 推导。
- 同一个 voxel 多个样本写入时，`hpl_adv/vpl_adv` 取较大值，`stamp_s` 取较新值。

### 4.2 当前完整兼容接口

为了同时兼容 `GridMap`、front legacy query、可视化和 CSV，对外 topic 应继续支持 evaluator 当前 21 字段：

```text
x, y, z,
hpl, vpl,
hpl_adv, vpl_adv,
stamp_s, source_age_s,
flags,
hal, val,
im_h, im_v, im_min,
cost,
grad_x, grad_y, grad_z,
risk_band, risk_band_code
```

建议语义：

| 字段 | 建议来源 | 说明 |
|---|---|---|
| `hpl/vpl` | Safety Grid 中采用的 advisory PL | 为 legacy front query 保留；可等于 `hpl_adv/vpl_adv` |
| `hpl_adv/vpl_adv` | Predictor fused advisory result | `GridMap` overlay 实际消费 |
| `hal/val` | Safety Grid 从 occupancy/ESDF/z boundary 计算 | legacy query 和可视化需要 |
| `im_h/im_v/im_min` | Safety Grid 计算 | `AL - PL` |
| `cost` | Safety Grid 或 GridMap cost policy | 若由 GridMap 重算，该字段仍用于 legacy path |
| `grad_x/y/z` | Safety Grid 可选计算 | overlay 不依赖；legacy B-spline 依赖 |
| `risk_band/risk_band_code` | Safety Grid / `PICostAdapter` | flags 兼容和可视化 |
| `flags` | Safety Grid 聚合 | 应包含 valid/stale/unknown/source/fallback 信息 |

## 5. Safety Grid Builder 应接收的输入

Safety Grid Builder 的输入应分为四类。

| 输入类别 | 具体内容 | 来源 | 是否必须 |
|---|---|---|---|
| Query source | rolling grid center、resolution、extent、z slices、active voxel set | planner/evaluator config | 必须 |
| Predictor query context | `IntegritySnapshot`、query time、horizon、PredictorModule reference | Predictor upstream / ARAIM snapshot | 必须 |
| Environment map | occupancy、ESDF/clearance、z boundary、frame id、map origin/resolution | IAP local map 或 ego `GridMap` | 必须 |
| Cost policy | HAL/VAL policy、PI cost params、unknown/stale penalty、gradient mode | Safety Grid config | 必须 |

建议的 builder 输入 contract：

```text
SafetyGridBuildInput
  center_w
  stamp_s
  frame_id
  resolution_m
  half_extent_x_m
  half_extent_y_m
  z_slices / z_min / z_max
  active_voxel_policy
  IntegritySnapshot snapshot
  PredictorModule predictor
  Occupancy/ESDF query interface
  PICostAdapter params
  staleness/unknown policy
```

注意：`PredictorModule` 不应知道 grid extent、voxel generation、PointCloud2 schema 或 planner risk band。

## 6. Predictor 应输出给 Safety Grid 的形式

新独立 Predictor 的输出应作为 Safety Grid 的 advisory PL/FIM source。

推荐核心字段：

| Predictor 输出字段 | Safety Grid 用法 |
|---|---|
| `query_position` | 写入 voxel/sample 位置或用于一致性检查 |
| `query_time_s` | 生成 sample stamp、horizon diagnostics |
| `horizon_s` | 未来预测 horizon 诊断 |
| `fused.hpl` | 写入 `hpl_adv` / `hpl` |
| `fused.vpl` | 写入 `vpl_adv` / `vpl` |
| `fused.pl_scalar` | 写入 internal `pl_adv_m` |
| `gnss.hpl/vpl` | debug layer / fallback diagnostics |
| `gnss.lambda_gnss` | 可选诊断层，不直接给 planner |
| `lidar.lambda_lidar` | 可选诊断层，不直接给 planner |
| `fused.lambda_pred` | 可选诊断层，用于 audit / debug |
| `fused.sigma_pos` | 可选诊断层，用于 audit / debug |
| `valid/fallback/fallback_reason` | Safety Grid 聚合 flags 和 unknown risk |

Predictor 不应输出：

- `HAL/VAL`
- `IM`
- `PI cost`
- `risk_band`
- `grid_generation`
- `grid_age_s`
- `grid_build_time_ms`
- PointCloud2 planner fields

这与当前 `PredictorQueryResult` 的方向一致；相反，旧 `FuturePLQueryResult` 里包含 `grid_generation/grid_age_s/grid_build_time_ms`，更适合作为旧 `FuturePLFieldPredictor + PLGrid` 兼容类型，不应成为新 Predictor API 的长期边界。

## 7. 当前代码字段与目标接口对照表

### 7.1 设计 URG layer vs 当前代码

| 设计 layer | 当前已有实现 | 新 Safety Grid 建议 |
|---|---|---|
| `Occ` | `GridMap` occupancy、evaluator local occupancy、`UnifiedRiskVoxel::occ_prob` | Builder 输入，不由 Predictor 提供 |
| `ESDF` | evaluator clearance 近似、`UnifiedRiskVoxel::esdf_m`、`GridMap::rawOccupancyClearance()` | Builder 输入/内部计算 |
| `AL` | `AlertLimitSample`、`UnifiedRiskVoxel::hal_m/val_m`、`GridMap::riskOverlayHal/Val()` | Builder 计算 |
| `Lambda_G` | `PredictorQueryResult::gnss.lambda_gnss`、`FuturePLQueryResult::lambda_gnss_trace` | Predictor 输出，Builder 可记录 |
| `Lambda_L` | `PredictorQueryResult::lidar.lambda_lidar`、`FuturePLQueryResult::lambda_lidar_trace` | Predictor 输出，Builder 可记录 |
| `PLpred` | `PredictorQueryResult::fused.hpl/vpl/pl_scalar`、`FuturePLQueryResult::hpl_adv/vpl_adv` | Predictor 输出，Builder 写入 grid |
| `IM` | `UnifiedRiskVoxel::im_h_adv_m/im_v_adv_m/im_min_adv_m` | Builder 计算 |
| `JPI` | `PICostAdapter`、`UnifiedRiskVoxel::pi_cost`、`GridMap::riskOverlayPiCost()` | Builder 或 planner adapter 计算 |
| `age` | `UnifiedRiskVoxel::age_s`、`RiskOverlayVoxel::age_s` | Builder / GridMap 维护 |
| `flags` | `UnifiedRiskFlags`、`RiskOverlayVoxel::flags`、risk_band_code | Builder 聚合 |

### 7.2 Planner consumer table

| Consumer | 当前输入 | 实际读取字段 | 对 Safety Grid 的要求 |
|---|---|---|---|
| `GridMap` risk overlay | `/iap/integrity_front_cost_field` | `x,y,z,hpl_adv,vpl_adv,flags,stamp_s/source_age_s` | 必须发布 finite advisory PL 和时间 |
| `AStar` overlay edge cost | `GridMap::integrateRiskOnEdge()` | `RiskOverlayQuery::cost` | 需要可插值/可积分的 scalar cost |
| `BsplineOptimizer` overlay | `GridMap::queryRiskInterpolated()` + `queryRiskGradient()` | cost + 本地差分 gradient | Safety Grid 不必发布 gradient，但 field 要空间连续 |
| `BsplineOptimizer` legacy | `/iap/integrity_cost_field` | `x,y,z,cost,grad_x,grad_y,grad_z,risk_band` | 若保留 legacy，则需要发布 gradient |
| front legacy query | `/iap/integrity_front_cost_field` | `x,y,z,hpl,vpl,hal,val,cost` | 若启用 legacy front search，需要完整 21 字段中的核心 AL/PL/cost |

### 7.3 Predictor output vs Safety Grid computed fields

| 字段 | Predictor | Safety Grid | Planner Adapter / ROS topic |
|---|---:|---:|---:|
| query position | 是 | 是 | 是 |
| query time / horizon | 是 | 是 | stamp/age |
| GNSS advisory PL/FIM | 是 | 可记录 | debug optional |
| LiDAR advisory FIM | 是 | 可记录 | debug optional |
| fused advisory HPL/VPL | 是 | 是 | `hpl_adv/vpl_adv` |
| occupancy / ESDF | 否 | 是 | optional debug |
| HAL/VAL | 否 | 是 | `hal/val` |
| IM | 否 | 是 | `im_h/im_v/im_min` |
| PI cost | 否 | 是 | `cost` |
| gradient | 否 | 可选 | `grad_x/y/z` for legacy |
| staleness / unknown penalty | 否 | 是 | flags / cost |
| risk band | 否 | 是 | `risk_band/risk_band_code` |
| PointCloud2 schema | 否 | 否 | 是 |

## 8. 推荐模块边界

推荐后续模块拆分如下：

```text
Current ARAIM
  -> certified current monitor snapshot

PredictorModule
  -> single-point advisory query
  -> GNSS/LiDAR/fused PL and FIM diagnostics

SafetyGridMapBuilder
  -> sample active voxels
  -> combine Predictor result with occupancy/ESDF
  -> compute AL/IM/PI/staleness/unknown/flags
  -> maintain rolling grid cache

PlannerAdapter
  -> publish /iap/integrity_front_cost_field
  -> optionally publish /iap/integrity_cost_field
  -> preserve ego_planner compatibility schemas

ego_planner
  -> GridMap risk overlay
  -> AStar edge risk
  -> B-spline risk cost/gradient
```

职责边界建议：

| 模块 | 负责 | 不负责 |
|---|---|---|
| Predictor | 单点 advisory HPL/VPL/FIM/fallback diagnostics | 建图、AL、IM、PI cost、PointCloud2 |
| Safety Grid | rolling cache、AL/IM/PI、age、unknown/stale、flags、gradient | GNSS/LiDAR 数学预测细节、ROS planner schema |
| Planner Adapter | topic schema、frame/QoS、legacy compatibility | 重新计算 Predictor 数学 |
| GridMap / planner | 最终 A*/B-spline 查询和优化 | 生成 Predictor 结果 |

## 9. 后续开发注意事项

1. 新 Safety Grid 不要直接复用旧 `FuturePLQueryResult` 作为核心 Predictor API，因为其中混入了 grid generation、grid age、build time 等建图元数据。
2. 可以复用 `UnifiedRiskGrid` 的 voxel 层设计，但应从 evaluator 内部逻辑中抽出独立模块，避免 `phase2_planner_integrity_evaluator` 继续同时承担 snapshot、predictor、grid builder、publisher、CSV 的全部职责。
3. Planner 兼容优先级应以 `GridMap` risk overlay 为第一目标，legacy `/iap/integrity_cost_field` 只作为过渡兼容。
4. 若 Safety Grid 直接发布 `/iap/integrity_front_cost_field`，应至少满足 `GridMap::ingestRiskOverlayCloud()` 的最小字段要求；为了不破坏 front legacy 和可视化，建议继续发布 21 字段完整 schema。
5. `HAL/VAL` 当前在 `GridMap` 和 evaluator 中都能计算，后续需要选定单一权威来源。若 Safety Grid 输出完整 21 字段但 `GridMap` 又重算 HAL/VAL，必须在文档和命名中明确这是兼容字段，不是 overlay cost 的唯一输入。
6. 如果 B-spline 走 risk overlay 路径，front field 的 `grad_x/y/z` 可以继续为 0；如果保留 legacy B-spline path，则 Safety Grid 需要计算并发布有限梯度。
7. `flags` 不应只塞 `risk_band_code`。新模块应定义清晰 bitmask：valid advisory PL、GNSS valid、LiDAR valid、FIM-add used、unknown risk、stale、fallback、occupied/out-of-range 等。
8. 当前相关 enable flags 多数默认关闭；Safety Grid 后续接入时要同步检查 evaluator launch、ego `risk_overlay/enable`、`risk_overlay/use_for_astar`、`risk_overlay/use_for_bspline`、`optimization/use_integrity_cost` 等参数。
