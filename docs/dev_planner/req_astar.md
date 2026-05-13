# ARAIM Integrity-Aware EGO Front Search

## Summary
新增 EGO 侧 `IntegrityCostMap`，复用现有 advisory PL proxy 预测结果，不重新实现 current certified ARAIM monitor。`phase2_planner_integrity_evaluator` 额外周期发布一个面向前端搜索的大范围 advisory integrity cost field；EGO 的 A* 在边代价中查询该 field，使前端路径从“只按距离/碰撞”变为“距离 + GNSS advisory integrity 风险”。同时保留开关，可分别启用/关闭 rebound A* 和 global A*。

## Key Changes
- **新增 front cost field**
  - 保留现有 `/iap/integrity_cost_field` 给 B-spline 后端使用；这是兼容 topic，字段为 advisory planner samples。
  - 新增 `/iap/integrity_front_cost_field`，默认同样使用 PointCloud2 字段：`x,y,z,hpl,vpl,hal,val,cost,risk_band_code`，梯度字段可置 0。这里的 `hpl/vpl` 是 advisory predicted PL proxy，不是 `/iap/integrity` 的 current certified monitor PL。
  - demo11 默认周期发布：`2Hz`，`50m x 50m`，`1.0m` 分辨率，中心为最新 odom，高度为最新 odom z。
  - Advisory PL proxy 仍由 `FuturePLFieldPredictor/PLGrid` 计算；front field 只是发布给 EGO 查询的 planner-side 缓存源。

- **EGO 侧 IntegrityCostMap**
  - 在 `bspline_opt` 内新增轻量缓存，订阅 `/iap/integrity_front_cost_field`。
  - 查询策略：最近邻，半径默认 `1.5m`；field 过期、无样本、超半径、非有限值时返回 cost 0。
  - 前端 cost 使用 ratio 归一化，而不是米制 raw cost：
    ```text
    risk_ratio = max(HPL / HAL, VPL / VAL)
    pi_cost = 0                              if risk_ratio <= 0.7
    pi_cost = ((risk_ratio - 0.7)/0.3)^2      if 0.7 < risk_ratio <= 1.0
    pi_cost = 1 + (risk_ratio - 1.0)^2        if risk_ratio > 1.0
    pi_cost = clamp(pi_cost, 0, 10)
    ```
  - 这样 `HPL > HAL` 或 `VPL > VAL` 都会影响 A*，不依赖后端 `phase2_pi_cost_weight_v`。

- **A* 边代价接入**
  - `path_searching::AStar` 不直接订阅 ROS topic；只新增查询回调和参数。
  - 原边代价：
    ```text
    edge = distance
    ```
    改为：
    ```text
    edge = distance * (1 + lambda_integrity_front * pi_cost)
    ```
  - 默认未知区域按 0 处理，保证 field 缺失时退化为原 EGO。

- **两个可切换入口**
  - Rebound A*：在现有 `dyn_a_star` 绕障搜索中启用 integrity edge cost。
  - Global A*：`planGlobalTraj()` 和 `planGlobalTrajWaypoints()` 在开关打开时先尝试 integrity-aware A* 生成 waypoints；失败则回退原直线插点。
  - Global A* 使用 `0.5m` step，100³ pool 覆盖约 `50m x 50m x 50m`，用于 demo11 起点到目标的长距离参考路径。

## Public Parameters
新增/转发这些参数：

```text
planner_use_integrity_front_search       default demo9=false, demo11=true
planner_use_integrity_global_search      default demo9=false, demo11=true
planner_lambda_integrity_front           default=2.0
planner_integrity_front_cost_topic       default=/iap/integrity_front_cost_field
planner_integrity_front_nearest_radius_m default=1.5
planner_integrity_front_stale_timeout_s  default=1.0
planner_integrity_front_cost_max         default=10.0
planner_integrity_global_astar_step_m    default=0.5
planner_integrity_global_max_waypoints   default=80

phase2_publish_integrity_front_cost_field      default demo11=true
phase2_integrity_front_cost_field_topic        default=/iap/integrity_front_cost_field
phase2_integrity_front_cost_field_publish_hz   default=2.0
phase2_integrity_front_cost_field_resolution_m default=1.0
phase2_integrity_front_cost_field_size_x_m     default=50.0
phase2_integrity_front_cost_field_size_y_m     default=50.0
```

`planner_lambda_integrity` 保持原语义，只控制 B-spline 后端 soft cost；新增 `planner_lambda_integrity_front` 专门控制 A* 前端。

## Test Plan
- Build:
  ```bash
  colcon build --base-paths src/iap src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm --packages-select iap path_searching bspline_opt ego_planner
  ```
- Unit/logic tests:
  - `IntegrityCostMap`：验证 ratio cost、nearest 查询、stale fallback、unknown=0。
  - A* edge cost：用假 integrity callback 验证高风险节点累计 gScore 更高，关闭开关时结果与原始距离代价一致。
- Demo11 scenario:
  - `planner_use_integrity_front_search:=false planner_use_integrity_global_search:=false` 作为 baseline。
  - 两个开关打开后，检查 `/iap/integrity_front_cost_field` 有周期消息，A* debug 中 `integrity_samples_used > 0`。
  - 对比 global/reference path 和最终 trajectory：开启后路径平均 `risk_ratio` 降低，`min IM` 提高，且规划仍成功。
- Regression:
  - field 缺失/过期时 planner 不崩溃，A* 自动退回原行为。
  - A* 搜索失败时 `planGlobalTraj()` 回退原直线插点。

## Assumptions
- 不直接让 EGO 访问 `PLGrid` 对象；跨进程只通过 PointCloud2 cost field。
- 未知 integrity 区域按 0 cost 处理，优先保证 planner 可用性。
- v1 不改变 current certified ARAIM monitor 算法、不改变 occupancy map 语义、不把 `PL > AL` 设为硬不可通行，只作为可调软代价进入前端搜索。
