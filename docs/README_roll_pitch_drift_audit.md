# Roll/Pitch Signed Drift Audit

## Executive Summary

基于 `src/iap/log/latest` 当前对应 run（分析输出位于 `/tmp/iap_ana_roll_pitch_audit`）的 signed drift 审计，这轮没有看到“strict-local/final 链本身已经稳定单边向右滚”的强证据；`frontend->final` 的 signed roll 统计为 `mean=-0.003 rad`、`positive_ratio=0.453`、`negative_ratio=0.547`、`cumulative=-0.153 rad`，更像偏负但不够单边稳定。

当前最早、最强的单号 roll/pitch 段出现在 `postsolve_active_window->strict_local`，不是 `start->frontend`，也不是纯 `frontend->postsolve_strict_local`。同时，`frontend->postsolve_strict_local` 与 `postsolve_active_window->strict_local` 两段的 roll/pitch 幅值都与 `solver_update_ms` / `relin_shared` 保持中高相关，且 frame convention / extrinsic runtime 观测没有出现 surface-specific split。因此这轮更支持：

- 主因：`M9 solver-side orientation amplification`
- 次因：boundary amplification 在 `postsolve_active_window->strict_local` 段继续放大
- 次嫌疑已降级：`world/lidar/imu frame convention` 或 `T_lidar_imu` 应用方向不一致

下一刀最小修复点更应落在 `M9 solver-side orientation chain`，而不是先回头怀疑 `M8/M10/M11` 或 roll/pitch extrinsic 应用方向。

## Signed Drift Evidence

### Final strict-local / final signed residual

| Signal | Summary |
| --- | --- |
| `frontend->final roll` | `mean=-0.003 rad; p95_abs=0.720 rad; max_abs=1.222 rad; positive_ratio=0.453; negative_ratio=0.547; cumulative=-0.153 rad` |
| `frontend->final pitch` | `mean=0.016 rad; p95_abs=0.453 rad; max_abs=0.682 rad; positive_ratio=0.568; negative_ratio=0.432; cumulative=0.603 rad` |
| `roll_drift_primary_stage` | `postsolve_active_window->strict_local` |
| `pitch_drift_primary_stage` | `postsolve_active_window->strict_local` |
| `earliest_monotonic_drift_segment_candidate` | `roll postsolve_active_window->strict_local direction=positive frames=210-214 cumulative=4.429 rad (positive_ratio=1.000, negative_ratio=0.000)` |
| `signed_roll_drift_interpretation` | `frontend->final signed roll drift is not strongly one-sided; earliest monotonic roll segment appears on an intermediate stage` |

### What this means

- 用户感知的“起飞后逐渐向右 roll”目前**不能直接由 strict-local/final signed roll 序列单独支持**。
- 目前更像是：
  - strict-local 求解链已经存在 roll/pitch amplification
  - active-window surface 与 strict-local 的差异继续把这类 drift 放大成更明显的单边段
- 也就是说，当前证据更接近“solver-side orientation drift + boundary amplification”，而不是“final strict-local 自己一直单边滚”。

## M9 Solver-Side Correlation Summary

| Stage / Axis | Strongest correlation | Interpretation |
| --- | --- | --- |
| `frontend->postsolve_strict_local roll abs` | `relin_shared:0.536` |
| `postsolve_active_window->strict_local roll abs` | `relin_shared:0.732` |
| `frontend->postsolve_strict_local pitch abs` | `solver_update_ms:0.490` |
| `postsolve_active_window->strict_local pitch abs` | `relin_shared:0.537` |

补充观测：

- `frontend->postsolve_strict_local roll abs` 与 `solver_update_ms` 也有 `0.533` 的相关性
- `postsolve_active_window->strict_local roll abs` 与 `solver_update_ms` 也有 `0.594` 的相关性
- 当前整轮 report 仍显示：
  - `rotation_primary_corr = solver_update_ms`
  - `delta_rotation_vs_relin_shared` 偏高
  - `recalculated_imu_factor_vs_solver_update_corr`
  - `recalculated_prior_factor_vs_solver_update_corr`
  - `recalculated_lidar_cross_support_factor_vs_solver_update_corr`
    都仍然显著

这说明：

1. `frontend->postsolve_strict_local` 段已经有 solver-side orientation amplification，不是纯 active-window 独有问题。
2. `postsolve_active_window->strict_local` 段的 roll/pitch 幅值相关性更强，说明 boundary surface 差异在继续放大已存在的 orientation drift。
3. 因为 `frontend->postsolve_strict_local` 已经出现中高相关，所以剩余主嫌疑更像 `M9`，而不是把一切都归到 boundary 或 query semantics。

## Frame Convention / Extrinsic Audit Table

| Stage | Pose semantic | Source variable | Transform chain | Extrinsic application side | Resulting frame |
| --- | --- | --- | --- | --- | --- |
| `strict_local query pose` | `world->lidar` | `postsolve_strict_local_query.pose_lidar` | `query_pose=world_to_lidar; compare_pose=pose_lidar * T_lidar_imu` | `right_multiply_T_lidar_imu` | query 结果是 `world->lidar`，compare 结果是 `world->imu` |
| `active_window query pose` | `world->lidar` | `postsolve_publish_query.pose_lidar` | `query_pose=world_to_lidar; compare_pose=pose_lidar * T_lidar_imu` | `right_multiply_T_lidar_imu` | query 结果是 `world->lidar`，compare 结果是 `world->imu` |
| `final pose` | `world->lidar` | `selected_final_query.pose_lidar` / `new_frame->T_world_lidar` | `final lidar pose -> final imu pose via pose_lidar * T_lidar_imu` | `right_multiply_T_lidar_imu` | final truth 仍是 `world->lidar`，同时 materialize `world->imu` |
| `runtime summary` | `query_pose=world_to_lidar;compare_pose=world_to_imu_via_right_multiply_T_lidar_imu` | `runtime_postsolve_query_frame_convention_kind` | 与上面静态代码一致 | `query_pose_lidar_right_multiply_T_lidar_imu` | 未观测到 surface-specific split |

### Static code evidence

- `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp`
  - `build_postsolve_evaluation_context(...)`
    - 明确写入 `frame_convention_kind = query_pose=world_to_lidar;compare_pose=world_to_imu_via_right_multiply_T_lidar_imu`
    - 明确写入 `extrinsic_application_kind = query_pose_lidar_right_multiply_T_lidar_imu`
  - `new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu`
    - final materialization 链继续沿用同一右乘方向
  - `postsolve_publish_query.pose_lidar * T_lidar_imu`、`postsolve_strict_local_query.pose_lidar * T_lidar_imu`
    - active_window 与 strict_local 的 compare pose 构造一致
- `src/iap/src/iap/odometry/integrated_bspline_imu_factor.cpp`
  - `imu_model.T_sensor_imu = T_lidar_imu.inverse()`
  - 这里的 inverse 是因 factor 内部用的是 `sensor->imu` 语义，不是 postsolve query chain 里的 compare pose 方向冲突

### Runtime evidence

当前 `log/latest` 的 runtime 统计是单一值，没有出现 mixed：

- `postsolve_active_window_value_source_kind = postsolve_authoritative_values`
- `postsolve_strict_local_value_source_kind = postsolve_authoritative_values`
- `postsolve_query_frame_convention_kind = query_pose=world_to_lidar;compare_pose=world_to_imu_via_right_multiply_T_lidar_imu`
- `postsolve_extrinsic_application_kind = query_pose_lidar_right_multiply_T_lidar_imu`
- `final_materialization_source_kind = selected_final_postsolve_query_from_postsolve_authoritative_values`
- `new_frame_consumes_final_truth_only = True`

因此，本轮没有看到“active_window / strict_local / final 三条链用了不同 roll/pitch frame convention 或不同 extrinsic side”的直接证据。

## Verdict And Next Minimum Fix

### Verdict

1. 当前 signed 数据**不支持**“strict-local/final 链已经持续单边向右 roll”这个强说法。
2. 当前最早、最强的单边 signed roll/pitch 段出现在 `postsolve_active_window->strict_local`。
3. 但 `frontend->postsolve_strict_local` 自己已经带有明显的 solver-side orientation amplification 相关性，因此 boundary 更像 downstream amplifier，而不是唯一源头。
4. frame convention / extrinsic 链目前静态和 runtime 都是统一的，已从主嫌疑降到次嫌疑。

### Minimum next fix

下一刀最小修复点应优先落在：

- `M9 solver-side orientation chain`

更具体地说，下一轮应优先检查：

- `frontend->postsolve_strict_local` 段里 roll/pitch 与 `relin_shared`、`solver_update_ms`、`recalc_imu`、`recalc_prior` 的关系
- 哪类 factor / relinearization pressure 最先把 strict-local postsolve orientation 往 roll/pitch 方向拉坏
- boundary 是否只是在放大已经存在的 solver-side orientation drift，而不是自己制造新的符号偏置

如果后续再做 frame/extrinsic 修复验证，也应该是**在 M9 solver-side orientation chain 被进一步压缩之后**再做，而不是现在优先怀疑 `T_lidar_imu` 应用方向。
