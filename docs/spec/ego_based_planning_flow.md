# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)。
本文件只描述当前真实流程、接口和验证边界，历史开发过程由 Git 保存。

## 状态与身份

开发起点 `f994cc9c632e8bb534cd9f47d6b663ad32673a47`，分支 `dev/iap_refactor`。
起始差异仅用户提供的未跟踪 v1.1 文件；原版 ego-planner-swarm 保持只读。

| 阶段 | 实际状态 |
|---|---|
| D1 | 实现及验证：route/execute 分离、统一有限 Advisory 代价、三个目标身份 |
| D2 | 实现及验证：连续执行前缀、真实零末 V/A、实际曲线检查、接续和撤销后的完整旧尾段检查 |
| D3 | 实现及验证：一次共享预算内执行连接恢复、观察后当前帧真实收益、最多一次质量／几何修正 |
| D4 | 相关集成通过；前两次原 ON 均未到达。末次超时恢复缺口已同输入修复验证，按新授权进入现场复验 |

交付状态 **IMPLEMENTED**，未达到 DEV_ACCEPTED，Goal 未标记完成。
2026-10-10（Asia/Shanghai）用户解除开发现场次数／墙钟限制，覆盖需求第 12.4 节；允许依据相关修复和新证据自主迭代。每次仍为原 300 s ON 任务，在线预算、安全条件、任务和到达规则不变。最终修复能否使原任务按原规则到达仍未验证。

## 当前主线与权限

原 FSM → beginPlanningView 冻结 GridMap／motion／risk 和唯一 PlanningBudget →
一个正常路线目标 → 同一 A* 在预算内 CostProof 比较有限软代价 →
selectExecutablePrefix 截取连续执行前缀 → 原 EGO 拟合／优化静止末端曲线 →
assessTrajectory 完成实际曲线硬检查 → 可选质量报告／一次修正 →
最新 corridor、当前 motion、PVA、时效及接续检查 → 原子提交 →
publicationStillTimely → 原 traj_server accepted/pending/active 反馈 → 原 FSM 监督。

GridRouteCell/queryRouteCell 允许物理未知但保留已知障碍、净空、motion、时效拒绝；
execute 查询始终要求合法观测。route 许可不会隐式转成 execute 许可。
Advisory 根据合法 HPL/VPL 与原 budget/reserve 得到统一有限代价；warning 不再
strict 拒绝或 fallback 重搜。未知／失效保持有限缺失代价，不伪装低风险或无障碍。
GuideIdentity 分开 mission_goal、route_target、committed_endpoint，保存冻结身份及区间。

正常和观察曲线都要求实际起点 P/V/A 与真实零末 V/A，retime 后重查。
原正常非零末速与直线 terminal_stopping_space 授权已退出在线主线。
assessTrajectory 的 completed 默认为假，空检查／硬检查耗尽不能发布。
已有实际采样、坐标极值、净空、观测、motion、时效、tracking 和发布检查保留；
未新增完整连续机体包络证明。pending 撤销前后都检查 active 至静止末端的完整尾段，
以实际 command feedback 消费身份，不靠生效时刻推断切换。

## 一次恢复与观察收益

当前 guide 无足够执行前缀，只说明该 guide 受阻。正常 lookahead 已知冲突、
正常搜索超时但尚有预算，也进入同一个 tryObservationApproach 连接恢复机会。
原任务目标本身的已知冲突仍拒绝，不修改任务终点。
恢复复用同一 A* 多目标搜索，优先已观测推进目标；确需改善观测时，再选观察目标。
只保留所选 guide，随后仅对所选目标求解 EGO，不引入新搜索器、常驻目标池或恢复 FSM。

观察最多八个廉价位置，依执行端点、邻接证据、已知遮挡和原 FOV 的正收益预估过滤。
端点／邻接可用仍不等于连接可用，最终由同一次执行语义连接搜索证明。
若推进搜索超时，不能以 observation incumbent 声称推进目标不可达。
SEARCH_TIMEOUT、NO_EXECUTABLE_CONNECTION_FOUND、BUDGET 分别报告。

观察记录绑定实际轨迹 ID；server feedback、合法末端 odom 和静止确认完成后，
相关 unknown→free/occupied 且新当前帧真实掩码覆盖该体素才记 GAIN。
NO_GAIN、DATA_UNAVAILABLE、COMPLETION_UNCONFIRMED 不重置同事件；自身移动不续发额度。
沿用 FSM 的等待证据机制；正常推进连接不作为失败观察事件锁住后续推进。
恢复请求目标集随既有 search evidence 保存，重放使用真实请求目标和冻结输入。

## 共享预算与质量边界

PlanningBudget 仍为总量 1.5 s、累计搜索 1.0 s、原三个修复额度，不提高或重置。
计时依据原森林 `20261009T083044Z_095`：成功轮次搜索后工作最高约 0.3869 s。
搜索保留 0.5 s 给后端／硬检查／提交，正常单次最多 0.5 s，恢复只用累计搜索
与总量剩余量；无余量返回 BUDGET。可选质量工作保留 0.4 s。
这些是本机实测留量，不是极低耗时指标承诺。

guide retention 是质量报告；质量退化可触发最多一次修正，失败则保留已受检候选。
NOT_COMPARED／PREFERENCE_DEGRADED 不单独否决提交；最终硬检查仍必须完成。
正常、连接恢复、几何修正、后端重启及发布重查共用同一预算与原额度。

## 配置、验证与剩余边界

两次现场均为 `icra_dense_forest_four_fork_v2`，map seed 41021、GNSS seed 20260502，
初始 `[-18,0,1.5]`、终点 `[18,0,1.5]`、原静止到达规则、300 s 窗口，
Advisory ON、prior OFF、可视化关闭。原历史 NAV hash
`42e87bd2edff3bb66e6d58b01c1710e9547270cfe795203ce947b8d88356bd32`。
有效上限 v=0.5 m/s、a=2.0 m/s²；PL budget H/V=0.55/0.60 m、reserve=0.10/0.10 m；
body=0.35 m、tracking reserve=0.10 m、tracking limit=0.30 m，motion/environment age=0.5 s。
均未放宽；FOV、地图来源、GLIO、执行消息、预测模型保持原实现。

复用现有 baseline／A*／GridMap 与 pipeline、scheduled、traj_server、full_stack_feedback、
failure_map 和后端重放入口。覆盖未知 guide 但有已观测旁路、高收益观察点断连但另点可达、
耗尽预算不能重发、唯一 warning 通路实际发布，以及撤销后旧尾段继续受检。
相关检查通过；未开展 OFF 对照、9+9、来源校准或正式统计实验。

现场 1：`20261009T163141Z_282`，SHA `e442c8ff`；目标筛选阻断，无轨迹提交，未到达。
现场 2：`20261009T164820Z_516`，SHA `2ab6164e`；guide→实际曲线→反馈贯通，
车辆从约 −18 m 推进至 `[-6.6,0.6,1.5]`，随后长路线搜索超时，未到达。
两次均干净提交、Release 安装 exact_bytes、GPU READY（cuInit=0，device_count=1）。
现场 2 冷首轮总量 0.19546 s、累计搜索 0.00585 s、后端 0.000251 s、候选检查 0.0000615 s；
成功轮次最高总量约 0.99543 s，均为现场已有计时，非性能重复实验。

末次冻结输入位于现场 2 `export/planner/failure_map/terminal_final`，generation=2551，
planning time=1657109100.062、risk anchor=1657109099.899，真实 map／Current motion／
Advisory payload／起始 PVA／路线目标／pool center 均保留。
`export/analysis/v11_timeout_input.json` 绑定原输入 hash；原搜索 0.5 s 超时后仍有
约 0.815 s 总余量。最后修复允许使用原余量进入一次执行连接；同输入验证中
正常搜索仍超时，恢复约 0.003 s，累计搜索约 0.503 s，实际曲线硬检查／静止末端通过。
该修复已完成定向及直接依赖集成，下一次现场验证原预算内连接恢复是否持续推进；尚无到达证据。
独立原到达规则审计 `20261009T171436Z_756` 确认两次均未到达。
