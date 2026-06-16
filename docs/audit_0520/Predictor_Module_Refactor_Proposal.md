# Predictor Module Refactor Proposal

本文提出一个职责收缩方案：将 Predictor 调整为一个和 Current ARAIM 并列的独立 advisory 查询模块。它只回答“在某个查询位置，根据当前 snapshot 和环境信息，预计 GNSS/LiDAR/fused advisory PL 是多少”，不负责建 safety grid、不计算 planner cost、不发布 planner topic。

核心判断：

- 设计文档和当前代码都把 predictor 用于 planner：预测未来候选位置的 PL，并最终服务 safety/risk field。
- 但更合理的模块边界是把“prediction model”和“map builder / planner adapter”拆开。
- Predictor 应成为纯查询器；Safety Grid、PL Map、UnifiedRiskGrid、AL/IM/PI cost、PointCloud2 发布应放在后续模块中调用 Predictor。

## 1. 背景和判断

设计文档中已经有两个概念：

| 概念 | 设计含义 |
|---|---|
| Advisory PL Predictor | 对单个候选位置或轨迹点预测 advisory PL |
| Predicted PL Map / Unified Risk Grid | 对空间 voxel 批量采样 predictor，并缓存 PL/AL/IM/cost/flags |

因此，严格来说，设计文档不是要求 Predictor 本身一定建图；它是让 predictor 的输出进入 map builder 和 planner query pipeline。不过在部分流程图中，`PL_pred -> AL -> IM -> PI cost -> rolling map -> planner` 被写在同一条链路里，容易让实现把这些职责都放进 Predictor。

当前代码也有类似混合：

- `FuturePLFieldPredictor` 同时提供 direct query 和 `PLGrid` cache/rebuild。
- `FuturePLQueryResult` 同时包含 advisory PL、GNSS/LiDAR/FIM diagnostics，也带 grid generation、grid age、gradient 等 map/cache 字段。
- `phase2_planner_integrity_evaluator` 同时负责 snapshot 汇集、predictor 调用、AL、PI cost、URG、PointCloud2 field、CSV。
- `UnifiedRiskGrid` 已经接近独立 map builder，但默认不是 predictor 的唯一后续层。

所以，把 Predictor 缩减成独立查询模块是合理方向。这样模块边界会更接近 Current ARAIM：ARAIM 是 current monitor compute API，Predictor 是 future/advisory query API。

## 2. 目标职责边界

### Predictor 负责

- 接收 query position 和完整 `IntegritySnapshot`。
- 调用 GNSS Advisory Predictor 估计 GNSS advisory information / HPL / VPL。
- 调用 LiDAR Advisory Predictor 估计 LiDAR advisory information / observability / diagnostics。
- 调用 Fusion Advisory Predictor 融合 prior、GNSS、LiDAR advisory information。
- 输出单点 `PredictorQueryResult`。
- 提供明确 validity、fallback reason、diagnostics。

### Predictor 不负责

- 不创建 rolling grid。
- 不缓存 voxel。
- 不做 trilinear interpolation。
- 不计算 AL。
- 不计算 IM。
- 不计算 PI cost。
- 不发布 PointCloud2。
- 不决定 A* 或 B-spline 如何使用 risk。
- 不把 current certified monitor PL 当作 future advisory PL 的默认替代。

### 后续模块负责

| 后续模块 | 职责 |
|---|---|
| Safety Grid / Risk Grid Builder | 采样 active voxels、调用 Predictor、缓存结果、插值、age/staleness、unknown risk |
| Planner Adapter | 计算 AL、IM、PI cost、risk band、PointCloud2 field、A*/B-spline 接入 |
| Current ARAIM | certified current monitoring、fault detection、current PL/AL/IM/state |

## 3. 新 Predictor 模块结构

建议 Predictor 模块内部划分为三个主要子模块。

```text
PredictorModule
  |
  +-- GNSS Advisory Predictor
  |
  +-- LiDAR Advisory Predictor
  |
  +-- Fusion Advisory Predictor
```

### 3.1 GNSS Advisory Predictor

目标：预测 query position 上 GNSS geometry / visibility 对 advisory PL 的贡献。

输入：

- `query_position_map`
- `IntegritySnapshot::gnss_epoch`
- visible sats / sat positions / URA / elevation / azimuth
- local occupancy 或 IAP 局部地图中的 visibility/raycast 能力

输出：

- `gnss_valid`
- `gnss_hpl`
- `gnss_vpl`
- `gnss_pl_e/pl_n/pl_u`
- `gnss_sigma_h/sigma_v`
- `gnss_advisory_information` 或 `lambda_gnss`
- `n_visible`
- `pdop`
- `fallback_reason`

当前代码基础：

- `PredictedAraimComputer`
- `GnssGeometryPlPredictor`
- `VisibilityPredictor`

### 3.2 LiDAR Advisory Predictor

目标：预测 query position 上 LiDAR map geometry 对 localization observability / information 的贡献。

输入：

- `query_position_map`
- current certified monitor snapshot 中的 LiDAR quality / excluded trunk ids / TDOP 等辅助诊断
- IAP 输出的局部地图，尽量复用当前 `LocalOccupancyGrid` / map cloud / normals / primitives，不新增独立地图实体
- LiDAR block summary
- LiDAR map geometry / surface primitives

输出：

- `lidar_valid`
- `lambda_lidar` 或 `delta_lambda_lidar`
- `lidar_alpha`
- `lidar_tdop`
- `lidar_condition`
- `n_primitives`
- `fallback_reason`
- diagnostics for degenerate geometry / missing map / too few primitives

当前代码基础：

- `LidarObservabilityFim::evaluate()`
- `LidarObservabilityFim::evaluate_advisory_fim()`
- `make_lidar_fim_primitives()`

### 3.3 Fusion Advisory Predictor

目标：融合 prior、GNSS advisory result 和 LiDAR advisory result，输出最终 fused advisory PL。

输入：

- GNSS advisory result
- LiDAR advisory result
- `IntegritySnapshot::lambda_base_pos` 或 position prior covariance/information
- fusion params：`fim_epsilon`、`K_H_adv`、`K_V_adv`、bias/overbound terms

输出：

- `fused_valid`
- `fused_hpl`
- `fused_vpl`
- `fused_pl_scalar`
- `sigma_h`
- `sigma_v`
- `lambda_prior_trace`
- `lambda_gnss_trace`
- `lambda_lidar_trace`
- `lambda_fused_trace`
- `condition/min_eig`
- `fallback_reason`
- `fusion_mode`

设计原则：

- Fusion Predictor 只融合 advisory information，不处理 AL/IM/cost。
- 若 LiDAR 不可用，应能返回 GNSS-only advisory result，并显式标记 reason。
- 若 prior 缺失，应明确 fallback/regularization，而不是静默假设完整 prior。
- 是否采用 `max(GNSS, fused)` 作为保守输出，需要作为兼容策略单独标记，不应混在 formal FIM-add 主路径中。
- Predictor 不应为 unknown prediction 返回看似安全的 fallback PL。`no_gnss_epoch`、`too_few_sats`、`singular_geometry` 等不可用状态必须输出 `available=false`、`valid=false` 和明确 reason，PL 数值不应为有限默认值。

### 3.4 统一融合状态空间

Predictor 必须定义一个唯一 common fusion state，否则 `Lambda_prior + Lambda_G + Lambda_L` 形式上成立但物理含义可能不一致。

本方案固定为：

```text
Predictor fusion state = position-only 3D in map/ENU frame

Lambda_prior,p in R^{3x3}
Lambda_G,p     in R^{3x3}
Lambda_L,p     in R^{3x3}

Lambda_pred,p = Lambda_prior,p + Lambda_G,p + Lambda_L,p
```

GNSS 原始 normal matrix 通常包含 receiver clock：

```text
Lambda_G =
[ Lambda_pp  Lambda_pc
  Lambda_cp  Lambda_cc ]
```

进入 Predictor fusion 前必须通过 Schur complement 消去 clock：

```text
Lambda_G,p = Lambda_pp - Lambda_pc * Lambda_cc^{-1} * Lambda_cp
```

LiDAR 如果来自 6D pose FIM，也不能直接和 GNSS position information 相加。必须先取 position block，或对姿态变量边缘化后得到 position-only information：

```text
Lambda_L,p in R^{3x3}
```

因此代码层建议显式使用：

```cpp
enum class PredictorInformationState {
  Position3MapEnu = 0,
};
```

并要求 `lambda_prior`、`lambda_gnss`、`lambda_lidar`、`lambda_pred`、`sigma_pos` 都声明为 `Position3MapEnu` 语义。Fusion Advisory Predictor 只接受 finite、近似 PSD 的 3x3 position information；明显 indefinite 的 prior/GNSS/LiDAR information 不应参与 fusion，并需要写明 fallback reason，例如 `invalid_prior_position_information`。

## 4. 输入输出接口建议

### 4.1 Public Query Input

建议未来对外只提供一个主查询输入：

```cpp
struct PredictorQueryInput {
  PredictorQueryInput(Eigen::Vector3d query_position_map,
                      IntegritySnapshot snapshot,
                      double query_time_s,
                      double horizon_s = 0.0,
                      std::string frame_id = "map");

  Eigen::Vector3d query_position_map;
  IntegritySnapshot snapshot;
  double query_time_s;  // required finite time for f_pred(p, t + tau)
  double horizon_s;     // required finite and >= 0.0; 0.0 = quasi-static query
  std::string frame_id; // "map" or documented ENU frame
};
```

`query_time_s` 和 `horizon_s` 不应使用 NaN 表达“当前时间”。Predictor 查询语义是 `f_pred(p, t + tau)`，因此调用方必须显式给出有限时间。Safety Grid 如果暂时构建 quasi-static field，应显式传：

```cpp
PredictorQueryInput input(
    voxel_position_map,
    snapshot,
    snapshot.stamp,
    0.0,
    "map");
```

如果 `query_time_s` 非 finite、`horizon_s` 非 finite 或为负数、`frame_id` 不是 `"map"` / `"enu"`，`PredictorModule::query()` 应返回 fallback，例如 `invalid_query_time`、`invalid_horizon` 或 `unsupported_query_frame`。这样避免 NaN 污染 Safety Grid stamp、CSV、staleness 和 fallback 判断。

`IntegritySnapshot` 应包含或可引用以下信息：

- 当前 pose。
- 当前 certified monitor snapshot。
- visible sats、sat positions、URA、elevation/azimuth。
- IAP 局部地图引用或 lightweight map view。
- prior covariance / information。
- LiDAR block summary。
- LiDAR map geometry。

如果为了避免复制大地图，`IntegritySnapshot` 可以保留轻量字段，地图通过 Predictor 初始化时注入 read-only provider：

```cpp
predictor.set_local_map_provider(...);
predictor.set_lidar_geometry_provider(...);
```

这样仍满足“query input 包含 snapshot 和 query position”，同时避免把大地图实体复制进每次 query。

### 4.2 Public Query Result

建议输出只保留 prediction 语义：

```cpp
struct PredictorQueryResult {
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason;

  Eigen::Vector3d query_position_map;
  double query_time_s;
  double horizon_s;
  std::string frame_id;
  uint32_t source_flags;

  GnssAdvisoryResult gnss;
  LidarAdvisoryResult lidar;
  FusionAdvisoryResult fused;

  std::string query_source = "direct";
};
```

结果中不应包含：

- `hal`
- `val`
- `im_h/im_v/im_min`
- `pi_cost`
- `risk_band`
- `grid_generation`
- `grid_age_s`
- `grid_build_time_ms`
- `PointCloud2` 发布相关字段

这些字段属于 Safety Grid / Planner Adapter。

### 4.3 面向 Safety Grid 的 Predictor 输出接口改造计划

根据 `Safety_Grid_Map_Builder_Input_Audit.md` 的结论，Safety Grid Builder 需要 Predictor 成为单点 advisory 输入源，而不是 planner risk result。因此 `PredictorQueryResult` 应明确包含 Safety Grid 采样所需的最小 metadata 和 source diagnostics。

Safety Grid 从 Predictor 读取：

| 字段 | 用途 |
|---|---|
| `query_position_map` | Safety Grid voxel/sample 位置一致性检查，也可直接写入 advisory PL layer |
| `query_time_s` | Safety Grid 生成 sample stamp、age、staleness 诊断 |
| `horizon_s` | 未来预测 horizon 诊断；Stage 1 只透传，不改变 GNSS/LiDAR 数学 |
| `frame_id` | 查询坐标系；当前只接受 `"map"` 或 `"enu"` |
| `fused.hpl` | 写入 Safety Grid advisory HPL，后续 planner topic 中对应 `hpl_adv` |
| `fused.vpl` | 写入 Safety Grid advisory VPL，后续 planner topic 中对应 `vpl_adv` |
| `fused.pl_scalar` | 写入 Safety Grid scalar PL layer |
| `gnss` / `lidar` diagnostics | 可选写入 debug layer 或 audit CSV，不直接作为 planner cost |
| `fused.lambda_pred` / `fused.sigma_pos` | 可选诊断，帮助分析 FIM degeneration / regularization |
| `available` / `valid` / `fallback` / `fallback_reason` | Safety Grid 聚合 unknown risk 与 fallback flags |
| `source_flags` | bitmask，表达 valid/fallback/source/regularization/conservative max 状态 |

建议新增独立 `PredictorResultFlags`，不复用 `UnifiedRiskFlags`，避免 Predictor 依赖 planner risk 层：

```cpp
enum PredictorResultFlags : uint32_t {
  PREDICTOR_RESULT_VALID = 1u << 0,
  PREDICTOR_RESULT_FALLBACK = 1u << 1,
  PREDICTOR_RESULT_GNSS_VALID = 1u << 2,
  PREDICTOR_RESULT_LIDAR_VALID = 1u << 3,
  PREDICTOR_RESULT_FUSION_VALID = 1u << 4,
  PREDICTOR_RESULT_PRIOR_VALID = 1u << 5,
  PREDICTOR_RESULT_GNSS_USED = 1u << 6,
  PREDICTOR_RESULT_LIDAR_USED = 1u << 7,
  PREDICTOR_RESULT_REGULARIZED = 1u << 8,
  PREDICTOR_RESULT_CONSERVATIVE_MAX = 1u << 9,
  PREDICTOR_RESULT_AVAILABLE = 1u << 10,
};
```

### 4.4 不返回有限 fallback PL

旧 planner predictor 为了兼容某些路径，在 GNSS 不可用时会返回有限 `fallback_pl`。这个行为不适合作为新 Predictor 的语义，因为 Safety Grid 或 PlannerAdapter 可能把 unknown prediction 误读成 moderate risk。

新 Predictor 的原则是诚实表达 prediction availability：

```text
no_gnss_epoch / too_few_sats / singular_geometry
  -> available = false
  -> valid = false
  -> fallback = true
  -> fallback_reason = explicit reason
  -> no finite PL
```

因此 `fallback_pl` 在新 Predictor 中只作为 deprecated compatibility 参数保留，不应用于生成有限 advisory PL。Safety Grid 后续必须基于 `available=false`、`fallback_reason` 和 `source_flags` 决定 unknown-risk penalty、stale penalty、降速或 fallback planner，不能把 NaN PL 当 0，也不能自行把 unavailable prediction 解释成安全区域。

Safety Grid 自己计算和维护：

- `HAL/VAL`
- `IM`
- `PI cost`
- `risk_band`
- `staleness` / `unknown risk`
- `grid_generation`
- `grid_age_s`
- `grid_build_time_ms`
- PointCloud2 planner schema

这条边界保证 Predictor result 可以被 Safety Grid 直接采样，但不会把 planner-specific cost 或 grid state 拉回 Predictor。

### 4.5 Query API

建议模块公开最小接口：

```cpp
class PredictorModule {
 public:
  explicit PredictorModule(const PredictorParams& params);

  void set_params(const PredictorParams& params);
  void set_local_map(const LocalOccupancyGrid* map);
  void set_lidar_geometry(std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives);

  PredictorQueryResult query(const PredictorQueryInput& input) const;
};
```

接口要点：

- `query()` 是主要 public API。
- 不提供 `rebuild_grid()`。
- 不提供 `active_grid()`。
- 不暴露 grid stats。
- 可保留 internal diagnostics stats，但不作为 planner grid state。

## 5. 与当前代码的映射关系

| 目标模块 | 当前可复用代码 | 迁移建议 |
|---|---|---|
| `PredictorModule` | `FuturePLFieldPredictor::evaluate_point()` | 抽出 pure query 逻辑，移除 grid/cache 语义 |
| `GnssAdvisoryPredictor` | `PredictedAraimComputer`、`GnssGeometryPlPredictor` | 重命名并保留兼容 wrapper |
| `LidarAdvisoryPredictor` | `LidarObservabilityFim` | 保留 FIM primitive path 为主，legacy alpha path 标注兼容 |
| `FusionAdvisoryPredictor` | `FuturePLFieldPredictor` 中 `use_advisory_fim_add` 分支 | 抽成独立类，输入 GNSS/LiDAR/prior result |
| Safety Grid Builder | `PLGrid`、`UnifiedRiskGrid` | 移出 Predictor 模块，作为下游 map builder |
| Planner Adapter | `PICostAdapter`、`phase2_planner_integrity_evaluator` | 保持 planner cost 和 topic 发布职责 |

当前 `FuturePLQueryResult` 可作为过渡类型，但长期建议拆分：

- `PredictorQueryResult`：只含 prediction。
- `GridQueryResult`：含 grid age/generation/interpolation metadata。
- `PlannerRiskResult`：含 AL/IM/PI/risk band/PointCloud2 field mapping。

## 6. Safety Grid / Planner Adapter 的后移职责

重构后，完整 planner pipeline 应变成：

```text
PredictorModule::query(position, snapshot)
  -> PredictorQueryResult

SafetyGridBuilder:
  for active voxel:
    predictor.query(voxel_position, snapshot)
    store fused advisory PL and diagnostics
  compute interpolation / age / staleness / unknown risk

PlannerAdapter:
  query safety grid
  compute AL from environment
  compute IM = min(HAL - HPL, VAL - VPL)
  compute PI cost
  publish PointCloud2 or provide A*/B-spline query API
```

这样职责更清楚：

- Predictor 只关心“定位侧未来 advisory risk”。
- Safety Grid 关心“如何批量缓存和查询”。
- Planner Adapter 关心“如何把定位风险变成规划代价”。

这也避免 Predictor 依赖 planner topic、cost schema、grid stale policy。

## 7. 分阶段迁移计划

### Stage 1: 文档和命名收敛

- 保留现有代码行为。
- 在文档中明确 Predictor 不应拥有 grid/cost/publisher 职责。
- 将 `FuturePLFieldPredictor` 标注为 transitional class：当前含 query + grid，后续拆分。
- 在代码注释中区分 advisory query result、grid query result、planner risk result。

### Stage 2: 抽出 pure query module

- 新增 `PredictorModule` 或 `AdvisoryPredictor`。
- 从 `FuturePLFieldPredictor::evaluate_point()` 抽出 GNSS、LiDAR、fusion query 逻辑。
- 新增 `PredictorQueryInput` / `PredictorQueryResult`。
- 在 `PredictorQueryInput` 中强制显式传入 finite `query_time_s`、finite non-negative `horizon_s`、`frame_id`，不再用 NaN 表达当前时间。
- 在 `PredictorQueryResult` 中加入 `query_position_map`、`query_time_s`、`horizon_s`、`frame_id` 和 `source_flags`，用于 Safety Grid 采样和 fallback 聚合。
- 新增 `PredictorResultFlags`，只表达 Predictor source/diagnostic 状态，不复用 planner `UnifiedRiskFlags`。
- `FuturePLFieldPredictor::evaluate_point_direct()` 改为调用新模块。
- 保持旧 API 不变，降低 evaluator 和测试改动风险。

### Stage 3: 拆出 grid builder

- 将 `PLGrid` rebuild/query 逻辑移到 `SafetyGridBuilder` 或 `PredictionMapBuilder`。
- `FuturePLFieldPredictor` 逐步退化为 wrapper 或删除。
- Grid builder 通过 `PredictorModule::query()` 填充 advisory PL layer，读取 `fused.hpl`、`fused.vpl`、`fused.pl_scalar`、`source_flags` 和必要 diagnostics。
- `grid_generation/grid_age/gradients` 只存在于 grid result，不进入 predictor result。

### Stage 4: Planner adapter 收敛

- `PICostAdapter` 和 `UnifiedRiskGrid` 保持在 planner/risk 层。
- evaluator 只负责 ROS IO、snapshot assembly、调用 grid builder / planner adapter。
- `/iap/integrity_front_cost_field` 的字段来自 planner adapter，不来自 Predictor。

### Stage 5: 清理兼容路径

- 将 `PredictedAraimComputer` legacy name 替换为 `GnssAdvisoryPredictor` 或 `AdvisoryGnssPredictor`。
- 明确 legacy `fused_fim_grid` 是否废弃或仅保留 debug mode。
- 移除 `FuturePLQueryResult` 中 grid-specific 字段，或只在 compatibility alias 中保留。

## 8. 测试和验收标准

### Unit tests

- GNSS Advisory Predictor:
  - open sky epoch 输出 finite advisory HPL/VPL。
  - no epoch 输出 fallback reason。
  - too few sats 输出 fallback reason。
- LiDAR Advisory Predictor:
  - rich primitives 输出 valid FIM。
  - missing map/primitives 输出 explicit fallback。
  - degenerate geometry 输出 explicit fallback。
- Fusion Advisory Predictor:
  - GNSS-only 可输出 valid fused result。
  - GNSS + LiDAR FIM-add 输出 finite HPL/VPL。
  - missing prior 被诊断为 missing prior 或使用显式 configured fallback。

### Integration tests

- `FuturePLFieldPredictor` compatibility tests 继续通过。
- Grid builder 使用 new Predictor query 后，PLGrid/URG 结果与旧路径在固定输入下保持一致或差异被明确记录。
- evaluator 启用 advisory mode 后，CSV 中 current fields 和 advisory fields 不混淆。

### Acceptance criteria

- Predictor public API 没有 `rebuild_grid()` / `active_grid()`。
- Predictor result 没有 AL/IM/PI/risk band。
- Safety Grid / URG 仍能通过调用 Predictor 生成 planner field。
- Current ARAIM 和 Predictor 不共享主 solver，不互相作为主流程入口。

## 9. 结论

建议采用“Predictor = 独立 advisory 查询器”的方案。

回答关键问题：

- 是的，设计文档和当前代码都体现了 predictor 服务 planner 的目标。
- 但这不意味着 predictor 模块本身应负责建图和 planner cost。
- 更好的边界是：Predictor 输出单点 advisory GNSS/LiDAR/fused PL；Safety Grid / Risk Grid Builder 调用它批量建图；Planner Adapter 再计算 AL/IM/PI cost 并接入 A*/B-spline。

这样做的收益：

- Predictor 可以像 ARAIM 一样成为独立、可测试、可替换的模块。
- GNSS、LiDAR、Fusion 三个子模块边界清楚。
- Planner 相关 schema 和 topic 变化不会污染 prediction model。
- 后续如果替换 LiDAR FIM、GNSS visibility 或 fusion math，不需要改 grid/publisher/planner 层。

最终目标结构：

```text
Current ARAIM Module
  -> certified current monitor report

Predictor Module
  -> single-position advisory query result

Safety Grid / Risk Grid Builder
  -> cached PL/AL/IM/cost field

Planner Adapter
  -> A* / B-spline / PointCloud2 integration
```
