# Phase F 落地计划：LiDAR Observability + Conservative Fused FIM

## Summary
- 在 `stage1_iap_araim_predictor_handbook.md` 末尾追加新章节：`## 17. Phase F 落地计划：LiDAR Observability + Conservative Fused FIM`，不改写原 Phase A-H 和第 14-16 章。
- 本轮实现 **Phase F-lite + 保守融合**：新增 LiDAR observability/FIM proxy，并提供默认关闭的 `fused_fim_grid` PL mode。
- 采用已确认方案：LiDAR v1 基于 demo10 map cloud 的局部几何 proxy，并用 `IntegrityReport` 中的 `n_trunks_observed/tdop/excluded_trunk_ids` 做 current modulation；不扩展 ROS msg。
- 本轮不做 Phase G：不接 planner cost、不改 estimator、不把 future LiDAR 做成虚拟 VGICP block ARAIM。

## Key Changes
- 新增 `LidarObservabilityResult` 与 `LidarObservabilityFim`：
  - 输入：候选点 `p_w`、共享只读 map cloud、`IntegritySnapshot.current`。
  - 输出：`valid/delta_lambda/tdop_proxy/lidar_alpha/condition/n_primitives/bias_h/bias_v/fallback_reason`。
  - 点云 proxy：在半径内取邻近点，按方向单位向量累计 `DeltaLambda += w * u*u^T / sigma_lidar^2`；点数不足或信息矩阵退化时 `valid=false`、alpha 为 0。
- 扩展 `FuturePLFieldPredictor`：
  - 新增参数：`use_lidar_observability`、`lidar_search_radius_m`、`lidar_min_points`、`lidar_good_points`、`lidar_sigma_m`、`lidar_info_scale`、`lidar_alpha_min/max`、`lidar_condition_ref/max`、`lidar_tdop_ref/max`、`lidar_bias_h_m/v_m`。
  - 新增 `set_lidar_map_points(shared_ptr<const vector<Eigen::Vector3d>>)`；evaluator 在 cloud callback 中更新 shared immutable cloud。
  - `gnss_geometry_araim` 行为不变；新增 `fused_fim_grid` 使用 GNSS ARAIM + LiDAR FIM debug，并以 `max(gnss_araim_pl, fused_cov_pl + lidar_bias)` 作为官方 PL，保证不会比 GNSS-only 更乐观。
- demo10 evaluator 接入：
  - 新增 launch 参数：`phase2_use_lidar_observability` 及 LiDAR radius/min-points/info-scale/alpha/bias 参数；默认全部关闭或保守。
  - CSV 保留现有字段，新增 debug 列：`gnss_hpl,gnss_vpl,fused_hpl,fused_vpl,lidar_valid,lidar_alpha,lidar_tdop,lidar_condition,lidar_n_primitives,lidar_bias_h,lidar_bias_v,lidar_fallback_reason`。
  - Snapshot CSV 增加 `n_trunks_observed,current_tdop,lidar_modulation_alpha`。
  - `phase2_summary.json` 增加 `lidar_observability` 段：enabled、valid count/rate、alpha/tdop/condition stats、fallback histogram、conservative fusion check count。
- Analyzer/validator：
  - analyzer 生成 `future_lidar_alpha_tdop_timeline.svg`、`future_gnss_vs_fused_pl.svg`，并在 `phase_h_lite.lidar_observability` 标记为 available。
  - validator 在 LiDAR enabled 或 `fused_fim_grid` 模式下检查新增列、finite PL/IM、finite LiDAR debug 样本存在、`PL_H_pred >= gnss_hpl`、`PL_V_pred >= gnss_vpl`，grid miss/too-few-points 不产生 NaN。
  - LiDAR disabled 时要求 GNSS-only direct/grid regression 语义不变。

## Implementation Steps
1. Core:
   - 新增 `include/iap/planner/lidar_observability_fim.hpp` 与对应 `.cpp`。
   - 扩展 `FuturePLQueryResult` 承载 GNSS/fused/LiDAR debug 字段。
   - 扩展 `FuturePLFieldPredictor` 的 direct 和 grid rebuild 路径，使 grid cell 可缓存 fused result 与 LiDAR debug。
2. Evaluator:
   - map cloud callback 同时维护 AL 用点云和 predictor 用 shared point cloud。
   - `pl_model=fused_fim_grid` 时启用 conservative fused result；`phase2_use_lidar_observability=false` 时自动退化为 GNSS-only debug-safe 路径。
   - Summary、CSV、snapshot CSV 写入 LiDAR observability 统计。
3. Scripts/docs:
   - 更新 analyzer/validator 识别新字段和 conservative fusion invariant。
   - handbook 追加第 17 章，明确 Phase F 采用点云+TDOP proxy，不扩展 ROS msg，不做 Phase G。
4. Build integration:
   - CMake 加入新 source 和 `test_lidar_observability_fim`。
   - 不改 `IntegrityReport.msg`，不触发 ROS interface rebuild 风险。

## Test Plan
- Build/unit:
  - `bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh`
  - `colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim"`
- Unit tests:
  - rich local point cloud produces valid `DeltaLambda`、finite `tdop_proxy`、positive `lidar_alpha`。
  - too few points yields explicit fallback and alpha 0。
  - line/plane-degenerate geometry lowers alpha relative to isotropic geometry。
  - poor current `tdop` or excluded trunk ids downweights alpha。
  - LiDAR disabled returns byte-for-byte GNSS-only PL fields except new debug columns.
  - fused mode never outputs official HPL/VPL lower than GNSS ARAIM HPL/VPL.
- Regression:
  - `gnss_geometry_araim + phase2_use_lidar_observability:=false` 60s：与 Phase D GNSS-only semantics 一致。
  - `fused_fim_grid + phase2_use_lidar_observability:=true + phase2_use_pl_grid:=true` 60s：validator 通过，`lidar_alpha/tdop` finite 样本存在，fallback rate `< 5%`。
  - map cloud missing/empty：CSV finite，LiDAR fallback reason 明确，official PL 退回 GNSS-only conservative result。
- Scenario:
  - open-sky + normal cloud：LiDAR debug 平滑，official PL 不低于 GNSS PL。
  - sparse/degenerate cloud：`lidar_alpha` 下降或 fallback 增加。
  - SkyMask/NLOS：GNSS `n_vis/pdop/HPL` 趋势仍可见，LiDAR debug 不掩盖 GNSS 退化。
  - fault injection：current ARAIM diagnostics 保留；future fused mode 不声称 certified FDE、不 silent success、不 NaN。

## Assumptions
- Phase F 本轮以 demo10 read-only evaluator 为唯一 runtime 接入点。
- `fused_fim_grid` 默认关闭；必须显式设置 `phase2_pl_model:=fused_fim_grid` 和 `phase2_use_lidar_observability:=true` 才启用 LiDAR path。
- 第一版 LiDAR observability 只使用 map cloud + current TDOP/trunk diagnostics，不使用真实 future VGICP blocks。
- 没有 `Lambda_base_pos` 时，用 GNSS fault-free sigma 构造 diagonal base information；官方 PL 仍用 `max(GNSS ARAIM, fused covariance + bias)` 保守包络。
