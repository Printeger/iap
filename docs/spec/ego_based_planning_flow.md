# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)。
本文件只描述当前接口、实施状态和验证边界，历史记录由 Git 保存。

## 当前状态

开发起点 `f994cc9c632e8bb534cd9f47d6b663ad32673a47`，分支 `dev/iap_refactor`。
起始差异仅用户提供的未跟踪 v1.1 文件。原版 ego-planner-swarm 保持只读。

| 阶段 | 当前真实状态 |
|---|---|
| D1 | 已迁移并定向验证：GridRouteCell/queryRouteCell 与 execute 分离；统一有限 Advisory 代价，A* 无 warning 拒绝／fallback 重搜；未知原目标保留路线身份，GuideIdentity 分开三个目标 |
| D2 | 已迁移并定向验证：执行点／体素边查询截取连续前缀；新正常曲线实际 PVA 起点及零末 V/A；停止空间／非零末速授权退出；发布和监督检查完整 active 尾段；空检查默认不可执行 |
| D3 | 已实现并定向验证：同一 A* 一次多目标执行连接恢复；推进优先，八个观察候选廉价过滤；实际完成后新证据收益；共享预算余量；质量报告与硬发布解耦 |
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
计时依据原森林 `20261009T083044Z_095`：首轮总量 0.2088 s、搜索 0.04964 s，
成功轮次搜索后工作最高 0.3869 s。搜索保留 0.5 s（正常单次最多 0.5 s），
可选质量保留 0.4 s。恢复只使用累计搜索与总量余量，无预算返回 BUDGET；
未完成硬检查默认拒绝。以上是本机测量留量，不是极低耗时承诺。

## 当前主线与接口

原 FSM → beginPlanningView 冻结地图／motion／risk 和预算 → 单一路线 LocalTarget →
BsplineOptimizer::searchRecoveryGuide（同一个 A*，route 语义及有限软代价）→ 连续执行前缀 → 静止末端 guide 拟合／优化 →
assessTrajectory（completed 必须为真）→ 可选 guide retention 报告及最多一次修正 → 最新实际曲线及相关 corridor → 原子提交 →
原 traj_server accepted/pending/active 反馈和 FSM 监督。

正常路线使用 A* 现有 CostProof 阶段在预算内比较有限软代价；超时有 incumbent
只说明已有 guide，不声称最优。无足够执行前缀时，tryObservationApproach 在同一
搜索中优先连接已观测推进目标，再连接正收益观察候选。恢复超时不解释成不可达；
推进未判定前不因 observation incumbent 而启动观察。只保留所选路径，随后一次 EGO。
观察记录绑定实际轨迹 ID：server feedback＋末端 odom 确认静止完成后，
相关 unknown→free/occupied 新证据才算 GAIN；NO_GAIN、DATA_UNAVAILABLE、
COMPLETION_UNCONFIRMED 不重置同事件。FSM 继续使用现有等待证据机制。
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

D1 定向证据：route 未知保留 execute 拒绝且检查 known obstacle，分段代价连续；
A* 唯一 warning 通路、较低风险路线选择和断连多目标共三项通过。
原目标未知身份和原目标侧向越过两项真实 FSM 回归通过。
有限软代价、物理执行和原任务到达分别判断。

D2 定向证据：非零起始 P/V/A 的未知尾部 guide 仅提交已观测前缀，
mission/route/committed endpoint 分开，最终实际末 V/A 小于 1e-5；冷运行总量
0.00402 s、优化 0.000713 s、候选检查 0.000122 s（合成 free-map，非现场性能保证）。
真实 captured guide 拟合、retime 导数、采样间越界和撤销后的旧尾段拒绝检查通过。
新 fixture 首次遗漏既有 swarm 初始化导致崩溃，补齐 fixture 初始化后定向通过。
默认空检查不能放行；正常末速和直线停止空间已无在线授权调用。

D3 定向证据：短未知前缀存在已观测旁路、最高收益观察点断连但较低收益点可达、
累计搜索耗尽不能重置、唯一 warning 通路实际曲线静止发布、完成后新 free/occupied
才有收益均通过。A* 与 GridMap 受影响测试通过；旧 strict、目标池及非零末速
断言迁移到 v1.1，相关 pipeline、scheduled、traj_server、full_stack_feedback、failure_map 以及
搜索／后端重放入口通过。baseline 集成 79 项通过，剩余前缀 fixture 原有窄未知带
被代价搜索成功绕开；改为确实存在未知尾部的同类合成输入后该项定向通过。
新重放识别 ends_at_rest，单次质量修正失败保留物理候选，最终硬检查耗尽仍拒绝。
本轮完整 ON 计数为 0/2，现场到达尚未验证。
