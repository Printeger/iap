# P4 Gate-0 audit

> 文档状态（2026-08-20）：**静态审计（STATIC AUDIT）**，源码引用绑定审阅 HEAD `bd3858a`。
>
> 本文回答代码落点与链路缺口，不是 P4 qualification、运行 PASS 或实现完成声明。

## 1. P4 当前具体在哪个 A* edge/node cost 中调用 `RiskGridSnapshot`？

P4 调用发生在 **A* edge cost**，不是 node cost，也不在 heuristic 中。`AStar::astarSearchImpl()` 展开 26 邻域时，先对 neighbor 做占据栅格硬拒绝，再把 `current -> neighbor` 的几何代价交给 `edgeCostWithRisk()`（`src/iap/planner/path_searching/src/dyn_a_star.cpp:274-316`）。该函数在 edge midpoint

```text
query_pos = 0.5 * (current_pos + neighbor_pos)
```

调用 `risk_snapshot_->queryCost(query_pos, query_time_s, &sample)`（同文件 `:177-197`）。查询时间为

```text
query_base
+ (norm(quantized_current - search_start) + current_edge_length) / query_speed
```

（同文件 `:166-175`）。它是起点到当前量化节点的径向距离再加当前 edge 长度，不是沿 A* predecessor chain 累积的实际路径行程。有效 edge 的实际增量是

```text
geometric_cost
+ lambda_p4_risk * geometric_cost
  * clamp(sample.cost, 0.0, risk_cost_max)
```

无效、stale 或 non-finite 查询则是 `geometric_cost + unknown_edge_penalty`。`gScore` 使用该 edge cost；`fScore` 仍是 `gScore + getHeu()`，风险没有进入 `getHeu()`（同文件 `:309-324`）。现有 focused test `test_p4_risk_astar` 的 4/4 用例通过，但它只验证单 edge 公式、unknown penalty 和可记录的 ratio metrics，没有验证完整路径或 planner 链。

## 2. 在 P1/P2/P3 全关时，现有场景能否稳定产生 collision segment？

**不能。** 当前最强的实测证据是否定结果：`results/icra27/gate0/raw-20260816-v3/gate0a/` 中 primary、mirror、flat-null 各 3 次，共 9 个 run、378 个 planning attempt，全部为 `collision_segment_count=0`、`base_generated_count=1`。

这些 effective manifest 同时关闭 P1/P2/P3/P4/P5 并保留 `manager/use_distinctive_trajs=true`。因此 singleton 的直接原因是 early prepass 未产出闭合 segment，而不是开关抑制了 fanout。

这不代表 seed 没有碰撞。central-obstacle seed 确实进入障碍，但约前 2/3 的扫描窗在障碍内部结束，没有观察到出口；后续 optimizer recheck 仍可检测碰撞并触发 rebound。

现有 `p4_manual_collision_guide` 也不是稳定 fixture：它只是 `scenario=manual`、`planner_safety_profile=p4` 加 debug/viz（`launch/test_planner.launch.py:728-733`），`manual` preset 本身为空（同文件 `:453-455`）。现有测试计划同样把 P4-2/P4-3/P4-6 标为“需要 collision-guide / long-detour / occupied-low-risk fixture”，并规定“缺 fixture -> blocked”（`docs/dev_planner/safety_planner_p0_p5_test_plan.md:368-373`）。所以当前没有无人值守、固定 seed、可重复地产生 collision segment 的 P4 场景。

## 3. 能否用现有 fixture 或很小修改构造 baseline guide vs lower-risk guide？

**可以做小改动构造，但不能零修改复用现有 launch fixture 就得到可信的 guide 对照。** 最小可行组合是：复用现有 deterministic fork/central-obstacle 几何（`launch/test_planner.launch.py:343-376`；`include/iap/planner/p1_fixture_geometry.hpp:155-190`），在 focused P4 fixture 中用现有 `RiskGridMap + RiskPredictionProvider` 测试 seam 给障碍物两侧的 free corridor 注入确定性的高/低 `c_pi`，然后对相同 start/end、相同 occupancy、相同 snapshot 分别调用 `AstarSearchOriginal()` 与 `AstarSearchRiskAware()`，硬断言 risk guide 的 mean/max risk 更低且 path-length ratio 不超阈值。

现有 fixture 单独不够：P1 fork 在现有 Gate-0 的 378 次尝试中仍产生 0 个 segment。

P0-6 是 occupied-low-risk overlap，occupied neighbor 会在风险查询前被硬拒绝，不能充当 free lower-risk corridor。P0-5 synthetic affine field 是 analyzer-only。

P5-3/P5-4 fixture 会在 risk-grid refresh 时写入有限 PL，并改变由其计算的 `c_pi`，所以会影响 P4 `queryCost()`。

P5-6 会把对应 support 注入为 invalid/unknown 和 NaN PL；它不产生由有限 PL 派生的有效 `c_pi`。

voxel 的 `c_pi` 会置为 `unknown_cost`，但 `queryCost()` 因 invalid/unknown support 保守失败并进入 unknown-edge 处理。P5-7 只在 final-candidate 的 `queryPredictedPL()` 路径生效。

这些 fixture 的空间、时间和语义为 P5 验证设计，不能替代同端点双 guide 的 P4 资格场景。Gate-0 仍建议新增小型 test-only P4 spatial-risk fixture，并确保 seed 稳定穿过 central obstacle。

若要做真实 planner 级对照，还必须同时修复第 4 问中的 snapshot/guide 记录时序。

## 4. 从 collision detected → P4 guide → B-spline → P5 final 的真实调用链有没有断点？

**有断点；当前不能宣称这条真实链已闭环。** 源码中的核心候选数据路径是连通的：`reboundReplan()` 参数化初始 cubic B-spline（`planner_manager.cpp:1035-1038`），给 A* 设置 P4 snapshot 后调用 `initControlPoints()`（`:1092-1097`）；后者检测 collision segment 并用 A* path 生成 control-point base/direction（`bspline_optimizer.cpp:982-1079, 1171-1239`）；随后 `distinctiveTrajs()`、rebound optimization、EGO feasibility/refinement 形成最终 `UniformBspline`，写入 `local_data_`（`planner_manager.cpp:1121, 1402-1403, 2091-2139, 2404`）；FSM 成功返回后，P5 final 在 normal B-spline publish 之前评估该 `local_data_`（`ego_replan_fsm.cpp:1050-1052, 1065-1129, 1152-1154`）。

但可审计的 P4 guide 链至少在五处断开：

1. 首次 collision path 走 `initControlPoints()`，其中只调用一次由开关分派的 `AStar::AstarSearch()`，不生成 `original_path` vs `risk_path` 对照，也不写 `last_p4_guides_`；紧随其后的 `publishP4Guides()` 因此拿到空列表（`planner_manager.cpp:1097-1102`）。
2. 真正同时计算 original/risk guide、做 ratio gate、写 CSV/`last_p4_guides_` 的代码只在 `check_collision_and_rebound()`（`bspline_optimizer.cpp:1899-1975`），而 manager 已在进入 rebound optimizer 前执行 `clearP4RiskSnapshot()`（`planner_manager.cpp:1104`）。该后续检查即使触发，也只能走 `snapshot_unavailable` 的 original fallback；并且没有后置的 P4 RViz publish。
3. 现有 `path_mean_cost/path_max_cost` 累积的是 risk A* 搜索期间被展开 edge 的查询值，不是 original/risk 最终 guide 的同规则风险 profile；baseline path 也没有被同 snapshot 重采样。因此现有 metrics 不能证明 lower-risk guide。
4. `manager/use_distinctive_trajs=true` 时，P4 guide 生成的约束还会派生正/反方向候选；P2 关闭后 legacy final-cost 选择仍可能选择与 P4 preference 不一致的最终 B-spline。现有链没有 selected-guide-to-final lineage 保证。
5. `p4_manual_collision_guide` 的 profile 启用 P4 及其隐式 P0 依赖，但不启用 P5；profile resolver 只有 `p5`/`all` 才默认启用 P5 runtime/final（`launch/test_planner.launch.py:1414-1447`）。所以现有 P4 experiment 不执行 P5 final，除非显式额外开启。

结论是：**“risk-aware 初始 A* 影响 B-spline”这段有源码连接，“最终 B-spline 在显式启用时进入 P5 final”也有连接；但 collision → 可比较/可观测的 P4 guide → 同一 guide 衍生 B-spline → P5 final 的同一次真实执行证据链目前断裂。**
