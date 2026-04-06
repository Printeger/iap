# BSpline Incremental Unified Graph 下一步图结构减压计划

## Summary
下一步优化重点不是继续调 `smoother_lag`，而是收窄 **current/new segment 对旧 control / 旧 factor 的触达范围**，优先降低：

- `recalculated_factor_count`
- `relinearized_variable_count`
- `affected_variable_count`

当前信号最强的是 **旧 factor / 旧连接被当前新 segment 频繁重新牵动**，而不是 delta 注入量太大。因此路线应是：

- 先缩 current segment 的 key touch set 语义
- 再局部化 IMU / velocity 耦合
- 再单独收紧 prior 邻接规则
- 最后才处理 query / mirror / LiDAR side-effect audit

严格保持：
- 不改 runtime 壳 / `apps/iap_rosnode.cpp`
- 不改 CPU/GPU legacy odometry
- 不改 sub/global mapping
- 不改 viewer/topic/TF/extension loading
- 不把 `OdometryEstimationBSpline` 变成 god object
- local/navigation 逻辑不回流
- `frontend_only_mode` 继续只是 unified graph 的 layer mask
- batch unified / legacy two-stage 回退路径保留

## 最可疑的 3 个连接点

### 1. `current/new segment -> local_control_indices -> stamp refresh / query / mirror` 这条链过宽
**位置**
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)

**当前连接方式**
- 当前帧 local contribution 建好后，odometry owner 会扫描 factor keys，把所有 `s(*)` 汇成 `local_control_indices`
- 这些 key 随后被复用于：
  - timestamp refresh
  - query keys
  - mirror/update
  - 以及部分后续边界逻辑

**为什么会放大 solver pressure**
- 把“当前 segment 数学上真正需要的 support”与“实现上方便处理的一组 control keys”混在了一起
- 当前段因此会触达超出最小必要范围的旧 control，增加旧 factor 被 recalculated 的概率

**建议怎么改**
- 明确拆成独立 key 集：
  - `segment_support_controls`
  - `new_control_indices`
  - `frame_query_keys`
  - `mirror_sync_keys`
- Commit 1 只改这些 key set 的语义和用途边界，**不改 prior 插边规则本身**

**预期 telemetry 影响**
- `recalculated_factor_count` 下降
- `relinearized_variable_count` 下降
- `affected_variable_count` 下降
- `query_key_count` 下降
- `solver_update_ms` 最有希望先下降

**层级**
- 影响 `local + unified graph owner`
- 不触碰 solver abstraction

---

### 2. IMU sample / velocity consistency 通过 shared `j/k/g` 形成高敏感耦合
**位置**
- [ct_local_frontend.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/ct_local_frontend.cpp)
- [integrated_bspline_imu_factor.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/integrated_bspline_imu_factor.hpp)
- [integrated_bspline_velocity_factor.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/integrated_bspline_velocity_factor.hpp)

**当前连接方式**
- 每个 IMU factor 连接 4 个 pose controls + shared `gyro_bias / accel_bias / gravity`
- velocity consistency 因子连接当前 segment controls + 当前 velocity auxiliary

**为什么会放大 solver pressure**
- 因子数不算极大，但共享 singleton state 会放大旧图敏感性
- 当前 segment 对 shared `j/k/g` 的触达，容易把一批历史 IMU 因子一并重新牵动
- 这更符合现在“recalculated factors 比变量数更相关”的信号

**建议怎么改**
- 严格限定 local delta 只提交当前 newly appended segment 的 IMU / velocity 因子
- 审计 IMU sample 选取是否有兼容性扩边，若有则收回到 strictly current-segment support
- 收敛 shared state touch 语义，避免 query/mirror/stamp refresh 额外把 shared keys 扩成更大的 touched set

**预期 telemetry 影响**
- `recalculated_factor_count` 下降
- `relinearized_variable_count` 下降
- `solver_update_ms` 下降

**层级**
- 主要影响 `local layer`
- 少量影响 `unified graph owner` 的 delta key 选择
- 不碰 solver abstraction

---

### 3. active-state / registry mirror / estimate subset 过宽
**位置**
- [bspline_fixed_lag_registry.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp)
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- [bspline_graph_solver.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/bspline_graph_solver.cpp)

**当前连接方式**
- active mirror 维持整窗 active pose/aux/shared 视图
- estimate subset 目前会并入较宽的 active pose/aux key 集
- registry / mirror / frame extraction 的便利性语义和 solver 真正需要访问的 key 集尚未分开

**为什么会放大压力**
- 它未必直接扩大 solver update 的内部 touched set
- 但会扩大：
  - query/extract 开销
  - mirror/update 开销
  - 部分间接 bookkeeping
- 若当前 mirror 逻辑反向影响 solver touched-set，则还可能间接放大 `affected_variable_count`

**建议怎么改**
- 拆分：
  - `solver_required_query_keys`
  - `frame_extraction_keys`
  - `mirror_sync_keys`
- 默认不再每帧请求整窗 active pose/aux estimate
- active mirror 若确实需要整窗信息，改为 lazy/on-demand 同步

**预期 telemetry 影响**
- 主要目标：
  - `query_key_count` 下降
  - `estimate_query_ms` 下降
- 次级观察：
  - 若 mirror 逻辑确实反向放大 touched-set，则 `affected_variable_count` 可能间接受益

**层级**
- 影响 `registry mirror / frame extraction`
- 影响 `unified graph owner`
- 轻微触碰 solver abstraction 的 query 边界，但不改 solver 数学

---

## 四类连接点的明确判断

### A. LiDAR bucket factor 到 control support 的连接
- [spline_state_layout.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/spline_state_layout.hpp) 的 `support_at(...)` 已经是 cubic spline 的 **4-control 最小 support**
- 所以最可疑的不是 support 本身太宽
- 真正需要优先审计的是：
  - factor handle
  - query key 扩张
  - timestamp refresh
  - mirror side effects
- 结论：
  - **绝对必要**：bucket 对应的 4 个 support controls
  - **可以收窄**：support 之外附带引入的 stamp/query/mirror 连接
  - 这些附带连接更可能导致旧 factor 反复 recalculated

### B. IMU sample / velocity consistency 的跨段耦合
- strictly local 的部分：
  - 当前 segment 的 velocity consistency
  - 当前 segment 的 IMU sample support
- 高敏感放大器：
  - shared `j/k/g`
- 结论：
  - 这类因子虽然不一定多，但更可能引发旧状态更新范围扩大
  - 很适合在第二步收敛成更局部的邻接连接

### C. control prediction / smoothness priors
- 它们不是当前第一嫌疑
- 但它们确实可能通过邻接规则把新 control 继续挂进一条偏宽的控制链
- 结论：
  - 应作为 **单独 commit** 改
  - 不要和 Commit 1 混在一起，方便 A/B 对比

### D. query_keys / estimate subset / registry mirror
- 真正必要的应只包括：
  - 当前 frame 回填
  - 当前 trajectory 输出
  - 必要 shared keys
- 目前偏宽的部分主要是为了方便
- 结论：
  - 是中低风险、收益更偏 bookkeeping/query 的收窄点
  - 不是当前首要 solver update 根因，但值得作为后续减压步骤

---

## 推荐改造顺序

### Commit 1
**主题：只收窄 current segment 的 key set 语义，不动 prior 邻接规则**

**修改文件**
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- [ct_backend_summary.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/ct_backend_summary.hpp)

**改造目标**
- 拆分 key set 语义：
  - `segment_support_controls`
  - `new_control_indices`
  - `frame_query_keys`
  - `mirror_sync_keys`
- 不再让一个 `local_control_indices` 同时承担 stamp/query/mirror/boundary 多种职责
- **不改** prior 插边逻辑本身

**不该碰的边界**
- 不改 prediction/smoothness prior 的邻接规则
- 不改 local layer factor 数学
- 不改 solver abstraction

**验收标准**
- `query_key_count` 下降
- `recalculated_factor_count` 有下降趋势
- `affected_variable_count` 有下降趋势
- `new_factor_count` 不上升
- `solver_update_ms` 首先出现改善
- 外部行为不变

**为什么最优先**
- 这是最低风险、最直接减少 old-state touch set 的入口
- 最可能先降 `solver_update_ms`

---

### Commit 2
**主题：把 IMU / velocity delta 收敛成 strictly local contribution**

**修改文件**
- [ct_local_frontend.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/ct_local_frontend.cpp)
- [integrated_bspline_imu_factor.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/integrated_bspline_imu_factor.hpp)
- [integrated_bspline_velocity_factor.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/integrated_bspline_velocity_factor.hpp)
- 必要时少量改 [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)

**改造目标**
- 审计并收窄 IMU sample 的当前 segment 时间覆盖
- 确保 velocity consistency 只连接当前 segment 的最小必要 support
- 降低 shared `j/k/g` 被当前 delta 广泛牵动的机会

**不该碰的边界**
- 不改 IMU / velocity 数学目标
- 不改 navigation layer
- 不改 batch unified / legacy two-stage 回退路径

**验收标准**
- `recalculated_factor_count` 下降
- `relinearized_variable_count` 下降
- `solver_update_ms` 继续下降
- 不出现数值退化 / NaN / divergence

---

### Commit 3
**主题：单独收窄 prediction / smoothness prior 到 strictly neighbor-only**

**修改文件**
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- 必要时少量改 [bspline_fixed_lag_registry.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp)

**改造目标**
- 在 Commit 1 完成后，单独修改 prior 插边规则
- 保证 prediction / smoothness 只作用于：
  - newly inserted control
  - 直接邻接 control
- 不再通过 full active-state traversal 间接扩张控制链

**不该碰的边界**
- 不改 key set 语义分层
- 不改 local/navigation layer factor assembly
- 不改 solver abstraction

**验收标准**
- `new_factor_count` 变化很小
- `recalculated_factor_count` 继续下降
- `solver_update_ms` 进一步下降
- A/B 能清晰区分这是 prior 邻接规则带来的收益

---

### Commit 4
**主题：收窄 estimate subset 与 registry mirror 的默认范围**

**修改文件**
- [bspline_fixed_lag_registry.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp)
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- [bspline_graph_solver.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/bspline_graph_solver.cpp)

**改造目标**
- 拆开：
  - solver required query
  - frame extraction
  - mirror sync
- 默认不再每帧请求整窗 active pose/aux estimate
- active mirror 改为按需同步

**不该碰的边界**
- 不改 solver update 数学
- 不改 runtime callbacks / topic / TF 语义
- 不把 solver state ownership 交回 registry

**验收标准**
- `query_key_count` 明显下降
- `estimate_query_ms` 明显下降
- 次级观察：
  - 若当前 mirror 逻辑确实反向放大 touched-set，则 `affected_variable_count` 可能间接受益
- 外部输出行为不变

---

### Commit 5
**主题：LiDAR factor handle / query / stamp side-effect audit**

**修改文件**
- [ct_local_frontend.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/ct_local_frontend.cpp)
- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- 必要时少量配合 [spline_state_layout.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/spline_state_layout.hpp)

**改造目标**
- 明确审计 LiDAR factor handle 在 single-bucket 下是否带来：
  - 额外 query 扩张
  - 额外 stamp refresh
  - 额外 mirror side effects
- **不改 LiDAR factor 数学核**
- **不改 cubic 4-control 最小 support**
- 只清理 support 之外的附带连接副作用

**不该碰的边界**
- 不改 `support_at(...)` 的 spline 数学
- 不改 bucket policy 本身
- 不改 target map 语义

**验收标准**
- `points_in_bucket` 基本不变
- `factor_total_ms` 不必是主目标
- 重点看：
  - `recalculated_factor_count` 是否继续下降
  - `solver_update_ms` 是否继续下降
- LiDAR 匹配质量不显著退化

---

## Assumptions
- 当前主矛盾是 factor recalculation / graph churn，不是 delta 注入量，也不是 lag 本身。
- 当前 navigation layer 不是下一步优先优化对象，因为最近带 telemetry 的 run 中 GNSS/backend 因子贡献很弱。
- [bspline_graph_solver_incremental.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/bspline_graph_solver_incremental.cpp) 在当前分支不存在；incremental solver 逻辑实际在 [bspline_graph_solver.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/bspline_graph_solver.cpp)。
- 所有 commit 都以“不改变外部 full-system 行为”为前提，只优化 BSpline unified incremental graph 的内部连接结构。
