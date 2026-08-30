# ICRA 森林在线 RiskMap、融合与 P4 修复

## 状态与边界

本文件记录 `icra_dense_forest_four_fork_v2` 的开发集成契约。它用于在线安全规划器的接口验证和可视化调试，不构成 P4 科学效果、正式资格或飞行安全声明。历史 `icra_dense_forest_four_fork_v1` 与 ICRA072 配置继续保留，用于复现和回归。

核心边界是：仿真障碍物/环境几何真值只能由传感器模拟器读取。`/map_generator/global_cloud` 可供 LiDAR renderer 和 GNSS 遮挡/NLOS/multipath 模拟使用，P0、EGO、P4 和 P5 不得直接或间接订阅 `/map_generator/*` 或 `/sim/world/*`。在线 profile 在 launch preflight 和 runner 的 ROS graph 审计中对此 fail closed。本轮按既定假设仍以 `/sim/drone_0/truth_odom` 提供规划 pose；它不提供障碍物地图或未来可见性，从 truth odometry 切换到估计 pose 是后续独立集成任务。

2026-08-30 用户后续决策覆盖了本计划中基础 EGO 的 unknown
fail-closed 要求：森林 v2 恢复原始 EGO 探索语义，设置
`grid_map/unknown_as_occupied=false`。该放宽仅适用于 P4 关闭时的基础
rebound/A*；P0 仍保留 UNKNOWN 风险状态，P4 开启后仍要求 frozen
observed-free support，P5 权威与阈值不变。

## 在线地图契约

森林 v2 使用已知任务 geofence，而不是已知障碍物地图：

- frame：`map`
- origin：`(-21, -11, 0) m`
- extent：`42 × 22 × 8 m`
- EGO occupancy resolution：`0.1 m`
- Risk overlay resolution：`0.5 m`，每轴严格覆盖 `5` 个 EGO voxel
- geometry identity：由 frame、origin、extent、voxel dimensions 和 resolution 生成

EGO occupancy 是三态的不可变 generation 快照：`OCCUPIED`、`OBSERVED_FREE`、`UNKNOWN`。机载点云的命中位置标为 occupied；传感器原点到命中的 ray traversal 标为 observed；没有回波本身不能证明 free。未观测空间保留 unknown。基础 EGO 在 P4 关闭时按原始行为允许 A* 穿越 unknown；P0 support、GNSS/LiDAR visibility 以及启用 P4 后的风险 guide 仍对 unknown fail closed。

P0 从 EGO 的 frozen occupancy epoch 获取共享 `PlanningLatticeGeometry`，不再根据全局点云 bbox 或 UAV 当前位置重算网格原点。这样 RiskMap 与 EGO 的坐标、边界及场景中心保持一致，且远处分叉在进入机载观测范围前显示为 unknown，而不是伪造的有效风险。

## 风险通道与融合

每个 risk voxel 保留以下可复算通道：

- GNSS-only：HPL、VPL、ratio、valid/stale/reason
- LiDAR-only：HPL、VPL、ratio、valid/stale/reason
- prior-only：等效 PL、信息来源和 provenance
- FIM fused：应用 conservative floor 前的结果
- safety fused：应用 conservative floor 后的权威结果
- horizontal/vertical floor source 与增量

森林 v2 的 alert-limit policy 固定为 `fixed_hal10_val20_v1`：

```text
HAL = 10 m
VAL = 20 m
risk_ratio = max(HPL / HAL, VPL / VAL)
```

本 profile 关闭尚未接通环境输入的 dynamic AL。HAL/VAL 是固定允许误差阈值，不是 PL 测量结果。

不对 PL 做时间低通滤波。current/P5 继续以 conservative safety-fused `FUSED/max_pl` 为权威。P4 候选必须先通过完整、fresh、observed 且 `safety_fused.risk_ratio < 1` 的安全门；只有通过安全门后，森林 v2 才以 pre-conservative FIM ratio 做 bottleneck 排序。若无候选通过，P4/P5 必须拒绝，不能关闭 GNSS、放宽阈值或用 RViz 归一化色值参与规划。

## RViz 契约

`/iap/rviz/predicted_pl_cloud` 是权威安全视图，使用固定 ratio 色阶：

- `< 0.5`：蓝到绿
- `0.5–0.8`：黄
- `0.8–1.0`：橙
- `>= 1.0`：红
- unknown：灰
- stale：黑
- occupied：紫，优先级最高

PointCloud2 同时携带 AL、risk ratio、GNSS/LiDAR/prior/FIM/safety 分源 PL/ratio、floor attribution 和 observed 状态。另有不参与规划的 P5/P95 相对对比云，以及 GNSS-only、LiDAR-only、FIM-fused 可切换诊断云。geofence wireframe、origin 和 geometry identity marker 用于直接检查网格范围与偏移。

## P4 在线决策与组合 identity

P4 保持局部 collision-segment guide 语义，不把 guide 伪装为从 UAV 起点开始的全局轨迹。每个局部 planning attempt 捕获一个不可变组合：

- risk generation/stamp/config hash
- geometry identity
- frozen occupancy generation/stamp
- GNSS、LiDAR、prior source generations/stamps
- alert-limit policy identity

启用 P4 时，original/risk A* 共用同一 frozen occupancy query 和 risk snapshot。端点以及最终稠密 guide samples 必须为 `map` 坐标、observed-free、非 raw/inflated occupied，并在同一 RiskMap support 内。搜索结束但发布前若 live occupancy generation 已变化，整个 attempt 重新规划。关闭 P4 时，original A* 使用原始 EGO 的二值 live-map 语义，不执行 frozen observed-free 完整性门。

P4 debug CSV 保存 `source_identity_hash`、`geometry_id`、occupancy stamp 和 map-frame collision-segment 首尾坐标，end-to-end lineage 保存同一决策 identity。P0 health 从对应 completed generation 的不可变 snapshot 原子发布 config/source identity，canonical config hash 同时绑定实际 HAL/VAL；runner 只有在同一 generation 的风险 profile、P0 source identity、空间上属于该分叉的 P4 decision 和已发布 lineage 全部一致时，才接受该分叉证据。不同分叉可使用不同 generation，但必须维持同一 geometry/config identity 和相同实际 HAL/VAL。RViz 以端点球、UAV 到 collision segment 的虚线和 `P4 local collision guide` 标签表达局部 guide 的真实含义。

## 森林 v2 与 runner 证据

v2 保持四分叉、固定 seed `41021/21` 和真实几何风险，并增加约 `5 m` 的自然冠层启动空地；不注入人工 risk。runner 保存：

- effective launch manifest、forest fingerprint 和展开后的四分叉定义
- planner truth-subscription ROS graph audit
- geometry、observed bbox、unknown/free/occupied 覆盖率
- GNSS/LiDAR/prior/FIM/safety 分源统计及 floor source
- 每个分叉的 immutable composite identity 与 P4/P5 lineage

分源报告必须说明开阔侧差异来自 GNSS、LiDAR、FIM 还是仅在 safety floor 后被覆盖。显示归一化不能制造规划差异；若真实 FIM profile 不满足四个分叉的低风险侧差异门，应停止场景调试。

## 验收顺序

1. 单元测试验证三态 occupancy、共享 geometry、色彩优先级、分源转换/floor attribution 和 immutable identity。
2. launch preflight 与 live graph audit 证明 planner 数据面无世界真值订阅。
3. P0 在局部已观测 planning ROI 内持续 `ready=true && stale=false`；全图 unknown 不影响局部 readiness。
4. 每个分叉的左右候选都 observed/fresh/complete 后，P4 才逐分叉决策；四次均选择真实 FIM 风险更低侧。
5. 至少一个真实 `RISK_SELECTED/selection_applied=1` lineage 到达 final B-spline，P5 final/runtime 为 `OK/ok`。
6. P4-off 与 P4-on 各连续三次、full 连续三次且每次 90 秒后，才进行 RViz 人工验收。

自动测试或开发运行未完成上述 live 重复门时，只能报告实现/离线回归结果，不得标记整体资格 `PASS`。

## 当前开发验证状态

截至本轮实现，森林 v2 的单次 live P0 已通过：共享 geometry 为
`origin=(-21,-11,0) m`、`extent=(42,22,8) m`、risk resolution `0.5 m`，固定
`HAL/VAL=10/20 m`，ROS graph 审计未发现 P0/EGO/P4/P5 对
`/map_generator/*` 或 `/sim/world/*` 的订阅。起点时约 `98.8%` 的全域栅格保持
unknown，符合局部在线观测语义，而不是用世界真值提前填满。

P4/full live 门当前仍未通过，不能标记整体 `PASS`。在线 GNSS 卫星射线进入未观测空间时按契约返回
unknown，因此前方分叉没有可用于排序的 GNSS-only profile；已有观测中实际主要由 LiDAR FIM
决定，而第一个同时覆盖两侧的分叉样本里，预定开阔侧的 pre-conservative FIM ratio
没有低于冠层侧。森林 v2 已关闭 `fallback_to_original_when_risk_not_ready`，所以 support 不完整或
真实风险差异不存在时 P4 会 fail closed，不会退回原始 guide 并把它误报为风险选择。

这不是坐标色图可修复的问题。下一步资格工作必须先增加能够证明自由空间/卫星射线的真实在线观测
（例如带 no-return/range 语义的深度 ray traversal），或重新设计并冻结能在原始 LiDAR/FIM
结果中产生目标差异的真实森林几何；不得用显示归一化、人工 risk、修改权重或放宽 P5 阈值制造通过。
