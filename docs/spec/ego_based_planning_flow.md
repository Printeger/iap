# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)。
本文件只描述当前接口、实施状态和验证边界，历史记录由 Git 保存。

## 当前状态

开发起点 `f994cc9c632e8bb534cd9f47d6b663ad32673a47`，分支 `dev/iap_refactor`。
起始差异仅用户提供的未跟踪 v1.1 文件。原版 ego-planner-swarm 保持只读。

| 阶段 | 当前真实状态 |
|---|---|
| D1 | 待迁移：执行查询同时用于搜索；Advisory strict/fallback 仍在线；目标候选仍使用执行资格 |
| D2 | 待迁移：正常局部目标允许非零末速，发布仍有直线停止空间授权；实际 spline、物理／motion／时效／接续检查已存在 |
| D3 | 待实现：一次执行连接恢复及必要观察、实际新证据收益、共享预算余量和一次质量修正 |
| D4 | 尚未运行本轮原完整 ON；不得沿用历史现场作为本轮到达 |

## 本轮已确定的三个边界

当前 guide 无足够执行前缀，只说明该 guide 受阻。恢复在原额度内复用同一 A*，
优先连接已观测推进目标，必要时才连接正观测收益目标。搜索未找到与超时分别报告。
所有连接均按执行语义，不把未知或 warning 改成执行许可。

观察最多八个廉价候选，使用已知障碍、执行观测和已有连接证据筛选。
一次多目标连接搜索输出一条路径，仅对所选目标求解一次 EGO。
推进连接与观察连接合计一个恢复机会，不逐候选完整搜索／优化。
实际收益只在执行反馈确认动作完成后，依合法相关新观测判断；自身移动不能重置尝试。

沿用唯一 PlanningBudget，1.5 s 总量、1.0 s 累计搜索、原三个修复额度。
正常搜索、恢复、可选质量计算须为后端／最终硬检查／提交留余量；
分配待一次冷计时决定，未完成检查默认拒绝，无预算恢复返回 BUDGET。

## 当前主线与待迁移接口

原 FSM → beginPlanningView 冻结地图／motion／risk 和预算 → LocalTarget 集合 →
BsplineOptimizer::searchRecoveryGuide（同一个 A*，strict/fallback）→ guide 拟合／优化 →
assessTrajectory → guide retention 硬门 → 最新实际曲线及相关 corridor → 原子提交 →
原 traj_server accepted/pending/active 反馈和 FSM 监督。

迁移顺序 D1 → D2 → D3 → D4；D1 单独提交不代表未知路线获得执行许可。
保留一张 GridMap、一个 A*、一条 guide、一个工作候选、原 EGO/FSM/traj_server。
不修改 GLIO、观测来源、预测模型、消息或任务到达规则。

## 有效配置与验证边界

原 scene `icra_dense_forest_four_fork_v2`、map seed 41021、GNSS seed 20260502、
原终点 `[18,0,1.5]`、原到达规则、300 s、Advisory ON、prior OFF，关闭可视化。
实际启动前再核对安装配置和身份，不用默认参数替代有效配置。
不改变 body/tracking/motion 预留、PL budget/reserve、FOV、年龄、动态限制。

复用 T1–T6 和现有 baseline/pipeline/scheduled/traj_server/full_stack_feedback 入口，
日常只测受影响项，最终一次相关集成。冻结输入初始定位一次、修复验证一次，
没有新证据不重复同实验。现场必须干净且提交／安装一致、GPU 预检通过。
本轮最多两次完整 ON：默认一次，明确相关修复和定向通过后才一次重跑。
不做完整 OFF、9+9、来源校准或连续机体包络证明。

交付仅原任务真实到达且必要验证通过可记 DEV_ACCEPTED／Goal complete；
IMPLEMENTED 或明确范围 BLOCKED 仍非目标完成。
