# Stage 2 File-Level Plan: Advisory Future FIM Predictor

## Summary

Stage 2 will add advisory-only position-domain FIM prediction behind explicit disabled-by-default switches. It will not touch certified GNSS ARAIM, certified LiDAR ARAIM, `/iap/integrity`, monitor max fusion, PI cost math, planner behavior, URG, old topics, or old fields.

Current advisory path:
- `PredictedAraimComputer` in `include/iap/planner/predicted_araim.hpp`, `src/iap/planner/predicted_araim.cpp`
- `LidarObservabilityFim` in `include/iap/planner/lidar_observability_fim.hpp`, `src/iap/planner/lidar_observability_fim.cpp`
- `FuturePLFieldPredictor` in `include/iap/planner/future_pl_field_predictor.hpp`, `src/iap/planner/future_pl_field_predictor.cpp`
- `FuturePLQueryResult` and `PLGrid` in `include/iap/planner/future_pl_query_result.hpp`, `include/iap/planner/pl_grid.hpp`
- Phase 2 app in `apps/phase2_planner_integrity_evaluator.cpp`

Current data flow:
`phase2_planner_integrity_evaluator` subscribes odom, map cloud, `/iap/integrity`, and GNSS range/ephem; builds `IntegritySnapshot`; updates `FuturePLFieldPredictor`; trajectory/grid queries return `FuturePLQueryResult`; CSV rows consume `query.hpl/vpl/pl_scalar`.

## Current Implementation Findings

- GNSS advisory predictor:
  - Inputs: `GnssEpoch`, `VisibilityPredictor`, optional `LocalOccupancyGrid`, per-sat elevation/azimuth/sigma/exclusion.
  - It constructs a 4D geometry matrix internally via `Araim::predict_geometry`: row `[cos(el)sin(az), cos(el)cos(az), sin(el), 1]`.
  - State is 4D ENU position plus meter-equivalent clock bias.
  - It does not expose `H = G^T W G`; only `AraimResult::S0` covariance is exposed.
  - Schur complement should be implemented in `PredictedAraimComputer`, not in certified `Araim::compute_core`.

- LiDAR advisory predictor:
  - Current formula is not voxel-normal FIM. It uses raw map points and radial unit vectors:
    `delta_lambda += taper * inv_sigma2 * u*u^T`, then scales by scalar `lidar_alpha`.
  - `lidar_alpha` depends on point count, condition score, current TDOP, and excluded trunk IDs.
  - Available map representation is `LocalOccupancyGrid` plus `std::vector<Eigen::Vector3d>` parsed from `PointCloud2`.
  - No voxel normals are available in the Phase 2 path. Normals must be parsed from cloud fields if present or estimated locally; otherwise LiDAR FIM must return zero-with-flag.
  - Candidate query points are treated as world/local ENU frame. Map points are also assumed world frame, but `PointCloud2.header.frame_id` is currently ignored, so frame ambiguity is a real risk.

- Prior information:
  - `CurrentIntegrityState` has no covariance.
  - `IntegritySnapshot` has optional `lambda_base_pos`, but `phase2_planner_integrity_evaluator::write_snapshot` does not populate it.
  - `FGOPositionInfo` has `sigma_p`, `lambda_p`, and `pose_cov_6x6`, but this is internal to integrity/extension code and not published through `/iap/integrity`.
  - Stage 2 should use `snapshot.lambda_base_pos` when valid; otherwise use zero prior and set `prior_valid=false`. Do not invent a strong prior from monitor PL.

## Implementation Changes

- `predicted_araim.hpp/.cpp`
  - Add `GnssAdvisoryFimResult`.
  - Add `PredictedAraimComputer::predict_advisory_fim(const Eigen::Vector3d& pos_world)`.
  - Reuse current visibility filtering, build 4x4 `H_full = G^T W G`, then compute:
    `Lambda_G_pos = H_pp - H_pc * inv(H_cc + eps_clock) * H_cp`.
  - Symmetrize, PSD-check, and zero-with-flag on degeneracy.
  - Keep `predict_araim_result()` unchanged for old advisory PL behavior.

- `lidar_observability_fim.hpp/.cpp`
  - Add `LidarFimPrimitive { center_w, normal_w, weight, normal_confidence, support_count }`.
  - Add `LidarAdvisoryFimResult`.
  - Add `evaluate_advisory_fim(p_w, primitives, current)` implementing:
    `Lambda_L_pos = sum pi_range * weight_scale / sigma^2 * n*n^T`.
  - Preserve existing `evaluate()` scalar observability method for compatibility.
  - If no valid normals/primitives exist, return zero matrix, `valid=false`, reason `missing_lidar_normals`.

- `future_pl_field_predictor.hpp/.cpp`
  - Add config fields:
    `use_advisory_fim_add=false`, `use_lidar_advisory_fim=false`, `fim_epsilon=1e-6`,
    `lidar_fim_radius_m=8.0`, `lidar_fim_min_voxels=6`,
    `lidar_fim_range_sigma_base=0.5`, `lidar_fim_condition_max=1e6`,
    `lidar_fim_weight_scale=1.0`,
    `K_H_adv=5.0`, `K_V_adv=5.0`,
    `b_H_pred=0.0`, `b_V_pred=0.0`, `s_H_pred=0.0`, `s_V_pred=0.0`.
  - Add `set_lidar_fim_primitives(...)`, while retaining `set_lidar_map_points(...)`.
  - If `use_advisory_fim_add=false`, keep current behavior exactly.
  - If enabled, compute:
    `Lambda_adv = Lambda_prior + Lambda_G + Lambda_L`,
    `Sigma_adv = inv(Lambda_adv + fim_epsilon I)`,
    `HPL_adv = K_H_adv * sqrt(lambda_max(Sigma_xy)) + b_H_pred + s_H_pred`,
    `VPL_adv = K_V_adv * sqrt(Sigma_zz) + b_V_pred + s_V_pred`.
  - Store advisory FIM result in existing `hpl/vpl/pl_scalar`, and keep old fields populated for compatibility.

- `future_pl_query_result.hpp/.cpp` and `pl_grid.cpp`
  - Add diagnostic fields:
    `lambda_prior_trace`, `lambda_gnss_trace`, `lambda_lidar_trace`, `lambda_adv_trace`,
    `lambda_adv_min_eig`, `lambda_adv_condition`,
    `hpl_adv`, `vpl_adv`,
    `lidar_fim_valid`, `gnss_fim_valid`, `fim_regularized`,
    `advisory_fusion_mode`.
  - Update `PLGrid::interpolate()` to interpolate numeric diagnostics and combine boolean validity conservatively.

- `apps/phase2_planner_integrity_evaluator.cpp` and launch files
  - Declare/pass new params with `phase2_` launch names.
  - Build optional LiDAR primitives from map cloud:
    parse `normal_x/y/z` if present; if absent, use lightweight local PCA over the subsampled cloud; if insufficient support, mark primitive invalid.
  - Add CSV/debug columns requested above to trajectory rows and grid voxel exports.
  - Add summary JSON entries for Stage 2 config and FIM diagnostic counts.
  - Do not modify validator freshness/schema issues in Demo11.

Suggested result structs:

```cpp
struct FimDiagnostic {
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  bool valid = false;
  bool regularized = false;
  double trace = 0.0;
  double min_eig = 0.0;
  double max_eig = 0.0;
  double condition = 1.0e12;
  std::string fallback_reason = "not_evaluated";
};

struct GnssAdvisoryFimResult : FimDiagnostic {
  Eigen::Matrix4d h_full = Eigen::Matrix4d::Zero();
  int n_visible = 0;
  int n_used = 0;
};

struct LidarAdvisoryFimResult : FimDiagnostic {
  int n_primitives = 0;
  int n_valid_normals = 0;
};

struct FusedAdvisoryFimResult : FimDiagnostic {
  FimDiagnostic prior;
  GnssAdvisoryFimResult gnss;
  LidarAdvisoryFimResult lidar;
  Eigen::Matrix3d sigma_pos = Eigen::Matrix3d::Identity();
  double hpl_adv = 1e9;
  double vpl_adv = 1e9;
  std::string fusion_mode = "legacy";
};
```

## Implementation Order

1. Add advisory FIM structs and GNSS Schur complement in `PredictedAraimComputer`; unit test against direct Schur math.
2. Add LiDAR primitive/result types and normal-based LiDAR FIM path, leaving legacy scalar observability intact.
3. Add FIM-add branch in `FuturePLFieldPredictor` behind `use_advisory_fim_add=false`.
4. Extend `FuturePLQueryResult`, `PLGrid` interpolation, stats, CSV, and summary diagnostics.
5. Wire Phase 2 params and launch args without changing default behavior.
6. Add tests and run build/test/launch sanity checks.

## Tests And Commands

Add or extend:
- `test_predicted_araim.cpp`: GNSS Schur complement unit test.
- `test_lidar_observability_fim.cpp`: LiDAR normal anisotropy test.
- `test_future_pl_field_predictor.cpp`: FIM regularization, fused FIM monotonicity, legacy compatibility.
- `test_pl_grid.cpp`: interpolation preserves new diagnostics.

Commands:
```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter"
colcon test-result --verbose
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py start_rviz:=false run_duration_s:=30 phase2_pl_model:=fused_fim_grid phase2_use_advisory_fim_add:=true phase2_use_lidar_advisory_fim:=false phase2_use_pl_grid:=true
```

## Risks And Open Questions

- Coordinate frames: `PointCloud2.header.frame_id` is ignored; GNSS uses ENU az/el while map/planner points are assumed local world.
- Units/scaling: GNSS clock column is meter-equivalent; LiDAR FIM scale needs calibration against GNSS weights.
- Voxel normals: no existing Phase 2 voxel-normal map exists; local PCA or optional cloud normals are required for nonzero LiDAR FIM.
- GNSS 4x4 vs 3x3: current advisory GNSS is 4x4 internally; Stage 2 should Schur-complement clock in `PredictedAraimComputer`.
- Prior covariance: no covariance is available through `CurrentIntegrityState`; default should be zero prior unless `snapshot.lambda_base_pos` is populated.
- Inversion stability: all inversions need finite checks, epsilon regularization, condition diagnostics, and zero-with-flag fallbacks.
- Backward compatibility: old PL fields, topics, `fused_fim_grid` compatibility behavior, and `/iap/integrity` semantics must remain intact when Stage 2 switches are disabled.
