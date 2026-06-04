# EGO GridMap-Aligned URG Redesign Plan

## Summary
- URG 正式重构为 EGO `GridMap` 的 risk overlay，复用 EGO occupancy map，不新增独立 ESDF 地图。
- `phase2_planner_integrity_evaluator` 只发布 advisory PL field；EGO `GridMap` 本地计算 AL、IM、PI cost。
- Phase 1 严格只做 receive/query/logging，不改变 planner 行为。
- A* 和 B-spline 分阶段、独立开关接入；legacy phase2 cost/URG field 只用于诊断，不参与正式 planner cost。

## Key Changes
- Phase2 → EGO 正式接口：
  - Required PointCloud2 fields：`x, y, z, hpl_adv, vpl_adv, stamp_s, flags`。
  - 任一 required field 缺失时整帧拒绝更新并 warning，不做 partial update。
  - 可兼容 `source_age_s`，但进入 overlay 后统一换算为 `stamp_s`，`age_s = planner_now - stamp_s`。
  - FIM metadata 只作为 optional debug fields，planner runtime 不依赖。
  - `header.frame_id` 必须匹配 EGO world/map frame；不匹配时拒绝更新。
- EGO `GridMap::RiskOverlay`：
  - 与 `occupancy_buffer_` 使用同一 voxel index、resolution、bounds、rolling/local update window。
  - GridMap rolling window shift/reset 时，overlay buffer 同步 shift/reset；新进入 voxels 初始化为 `RISK_UNKNOWN`。
  - 存储：`hpl_adv`、`vpl_adv`、`stamp_s`、`age_s`、`flags`、`pi_cost`。
  - 同一 voxel 内多 advisory samples 保守聚合：`max(HPL)`、`max(VPL)`、latest stamp、OR-combined flags。
  - missing/uncovered voxels 标记 `RISK_UNKNOWN`，施加 `lambda_unknown` soft penalty；hard collision 仍只由 inflated occupancy 决定。
  - `pi_grad` 不缓存；B-spline gradient 第一版使用 finite/central difference。
- 地图层语义：
  - raw occupancy：clearance proxy / AL。
  - inflated occupancy：hard collision rejection。
  - risk overlay：soft integrity cost。
- EGO 本地 PI 定义：
  - `HAL(p)=gamma_h * max(d_clear_raw(p)-r_drone-m_safety, 0)`。
  - `VAL(p)` 来自 vertical bounds / vertical clearance。
  - `rH = HPL_adv / (HAL + eps)`，`rV = VPL_adv / (VAL + eps)`，`r = max(rH, rV)`。
  - `c_PI = 0` if `r < r_soft`。
  - `c_PI = w_soft * (r-r_soft)^2` if `r_soft <= r < 1`。
  - `c_PI = w_hard * (r-1)^2 + c_unsafe` if `r >= 1`。
  - `c_used(p)=c_PI(p)+lambda_stale*(1-exp(-age/tau))+lambda_unknown*I_unknown`。
- Threading/snapshot:
  - Overlay write/read 必须 thread-safe。
  - 每次 planning cycle 使用一致的 overlay snapshot，避免 A* 或 optimizer 在一次规划中读到混合版本。

## Planner Integration
- Phase 1: receive + query + logging only
  - Defaults:
    - `risk_overlay/enable=true`
    - `risk_overlay/use_for_astar=false`
    - `risk_overlay/use_for_bspline=false`
  - EGO `GridMap` 订阅 advisory PL field，写入 overlay。
  - 新增 `queryRiskInterpolated(p)`、`queryRiskGradient(p)`、`integrateRiskOnEdge(p0,p1)`。
  - 记录 overlay-vs-legacy difference/match ratio；只用于诊断，不自动调参。
- Phase 2: A* uses overlay
  - hard collision 仍用 `getInflateOccupancy()`。
  - edge risk 采样：`N=max(2, ceil(||p1-p0|| / (alpha * map_resolution)))`，`alpha=0.75` 默认。
  - edge cost：`edge_cost = ego_edge_cost + lambda_pi * length * mean_pi_cost`。
  - global A* 和 local rebound A* 共用同一个 GridMap risk snapshot。
- Phase 3: B-spline backend uses overlay
  - 独立开关：`risk_overlay/use_for_bspline`。
  - 替换 control-point nearest-sample PI cost。
  - 对 B-spline trajectory samples 查询 `queryRiskInterpolated(p)`。
  - cost 对 samples 积分。
  - gradient 由 central-difference `queryRiskGradient(p)` 得到，并按 B-spline basis 回传控制点。
- 保留实验组合：
  - A* off / B-spline off：baseline。
  - A* on / B-spline off：front-end only。
  - A* off / B-spline on：back-end only。
  - A* on / B-spline on：full formal flow。

## Replaced / Impacted Code
- 替换：
  - `BsplineOptimizer` 当前 `/iap/integrity_cost_field` nearest lookup backend cost。
  - `AStar` 当前 neighbor 单点 integrity callback cost。
  - phase2 URG full rebuild 作为 planner source 的路径。
- 保留：
  - `iap::UnifiedRiskGrid` 类和测试，作为 legacy validation/ablation。
  - phase2 legacy `/iap/integrity_cost_field`、`/iap/integrity_front_cost_field`，用于 RViz/CSV/debug。
- 需要修改：
  - `plan_env`：新增 overlay buffers、advisory subscriber、clearance/AL/PI query API、snapshot API、thread-safe access。
  - `path_searching`：A* edge risk integration。
  - `bspline_opt`：trajectory sample PI cost 和 central-difference gradient。
  - `ego_planner` launch：新增 overlay topic、stale/unknown/clearance/PI 参数和独立开关。
  - `demo11` launch：默认先启用 overlay logging；正式 flow 可通过独立开关启用 A* / B-spline overlay。

## Test Plan
- Unit tests:
  - overlay ingestion：samples received > 0，written voxels > 0。
  - required fields 缺失：整帧拒绝，旧 overlay 不被 partial update 污染。
  - same-voxel aggregation：max HPL/VPL、latest stamp、OR flags。
  - rolling/reset：overlay 跟随 GridMap shift/reset，新 voxels 为 `RISK_UNKNOWN`。
  - frame mismatch：拒绝更新并 warning。
  - timestamp：`stamp_s/source_age_s` 统一后 `age_s`、`clock_delta_s` 正确。
  - clearance：raw occupancy clearance 与 inflated hard collision 分离。
  - PI：H/V ratio risk、valid/stale/unknown penalty 正确。
  - thread/snapshot：一次 planning cycle 内 query generation 一致。
  - A*：人工 high-PI band 下，integrity off 走短路径，integrity on 绕开且 collision-free。
  - B-spline：穿越 high-PI 区域时 cost 非零，central-difference gradient finite。
- Integration tests:
  - phase2 advisory field 发布后 EGO overlay query hit ratio 超过阈值。
  - warm-up 后 unknown ratio、stale ratio 低于配置阈值。
  - legacy-vs-overlay difference/match ratio 可记录、可解释，但不触发自动调参。
- Demo11 acceptance:
  - Phase 1 默认不改变轨迹行为：overlay enabled，A* / B-spline usage disabled。
  - 正式 flow 测试时启用 `risk_overlay/use_for_astar=true` 和 `risk_overlay/use_for_bspline=true`。
  - `/drone_0_planning/bspline`、`/drone_0_planning/pos_cmd` 正常发布。
  - A* 和 optimizer risk query hit count > 0 when enabled。
  - overlay 不长期 unknown-only 或 stale-only。
  - 日志包含 `risk_source`、`sample_stamp`、`planner_now`、`age_s`、`clock_delta_s`。

## Assumptions
- phase2 继续是 advisory PL/FIM predictor，不迁入 EGO。
- planner runtime 第一版不依赖 FIM metadata。
- URG 正式定义为 `EGO GridMap-aligned risk overlay`。
- 第一版使用 raw occupancy clearance proxy，不宣称完整 ESDF。
