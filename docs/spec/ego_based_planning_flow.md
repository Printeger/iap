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
| D4 | 相关集成通过；原 ON 尚未到达，继续验证共享预算下的持续已观测连接 |

交付状态 **IMPLEMENTED**，未达到 DEV_ACCEPTED，Goal 未标记完成。
2026-10-10（Asia/Shanghai）用户解除开发现场次数／墙钟限制，覆盖需求第 12.4 节；允许依据相关修复和新证据自主迭代。每次仍为原 300 s ON 任务，在线预算、安全条件、任务和到达规则不变。原任务到达仍未验证；非终点短段完成可触发重规划，最终段保留原静止到达等待，全部动力学／硬检查不变。

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
A* 有独立 Advisory 回调时，该回调统一负责首次样本及缓存命中的有限代价；
manager 的 A* 物理回调不再重复读取 Advisory。queryRouteViewCell 的
include_advisory 参数支持这一分工，默认行为保留；实际曲线／EGO 查询不变。
细采样仍逐点执行物理查询，软代价不能覆盖拒绝。原采样、权重和缓存更新规则不变。
GridMap 原净空缓存只保存冻结障碍距离上下界，以 generation 失效；变化的 guide
拟合阈值不再清空所有体素。上下界不足以判定新阈值时仍用原精确扫描，不复用
旧接受结果；空有限扫描也只有有限下界。观测、净空参数和执行检查不变。

正常和观察曲线都要求实际起点 P/V/A 与真实零末 V/A，retime 后重查。
fitGuideCurve 的 nominal_interval 是 ctrl_pt_dist 的物理时间尺度；实际采样间距按
弧长同比例分配时间，最少控制点数量不能把短前缀拉长为反复被接续替换的慢动作。
voxel 细化保持该总时间。证据模型 guide_arc_voxel_diagonal_arc_time_v2；重放可读取
历史 v1 输入，但明确记录应用的当前模型，历史现场记录不改写。既有后端重放的
initialize 模式可读取 committed 捕获及其原 ON payload，在原剩余预算下验证重新拟合。
原正常非零末速与直线 terminal_stopping_space 授权已退出在线主线。
assessTrajectory 的 completed 默认为假，空检查／硬检查耗尽不能发布。
已有实际采样、坐标极值、净空、观测、motion、时效、tracking 和发布检查保留；
未新增完整连续机体包络证明。pending 撤销前后都检查 active 至静止末端的完整尾段，
以实际 command feedback 消费身份，不靠生效时刻推断切换。
EXEC_TRAJ 的非终点局部曲线完成也触发下一轮，不要求短静止段超过滚动重规划阈值。
最终任务曲线保留原静止等待／到达分支，不以参考曲线完成抢占车辆实际静止确认。
尾段不足 0.1 s 时不新建接续：实际同 ID 完成命令、新鲜同 frame 的末端 odom、
位置／静止及曲线零末 V/A 确认后，从实测 PVA 进入原全局参考规划入口；未确认
继续监督旧尾段。此触发不改变原任务静止到达条件，也不直接授权新曲线。

## 一次恢复与观察收益

当前 guide 无足够执行前缀，只说明该 guide 受阻。正常 lookahead 已知冲突、
正常搜索超时但尚有预算，也进入同一个 tryObservationApproach 连接恢复机会。
原任务目标本身的已知冲突仍拒绝，不修改任务终点。
恢复复用同一 A* 多目标搜索，优先已观测推进目标；确需改善观测时，再选观察目标。
只保留所选 guide，随后仅对所选目标求解 EGO，不引入新搜索器、常驻目标池或恢复 FSM。

观察最多八个廉价位置，依执行端点、邻接证据、已知遮挡和原 FOV 的正收益预估过滤。
预估雷达姿态／FOV 的轻量元数据随同一个物理 epoch 冻结；正常搜索期间实时地图
更新不能使恢复误读为空元数据而跳过观察候选。该元数据不授予观测／执行许可；
动作后的收益仍取合法新当前帧。定向断连候选反例加入搜索期间 generation 更新，
修复前恢复失败，修复后同一冻结证据下只搜索一次并选出可达观察点。
偏移集中在 observationOffsets：六个 ±0.5 m 轴向位置保留，两个任务方向组合位置
在 ±0.5 m 高度内按原 FOV 边界内侧调整，使固定关键体素进入视场；不改变姿态、
FOV、探针身份、候选数量或执行许可。现场 `20261009T181320Z_163` 的终点冻结输入
（generation 2613、原 ON payload／motion／PVA）中，原八个固定偏移没有正收益，
可连接的约 0.5 m 横移配合小幅高度调整被遗漏；同输入在现有 baseline 入口验证。
同一反例同时要求观察 committed_endpoint 等于所选 route_target，不能截取中途前缀
后标记观察执行。A* 的细采样执行拒绝在 Advisory 代价积分中保留，不再覆盖为 OK；
正常 route 的未知许可仍由 route 查询独立决定。
实际端点保持不变；同一 A* 为两端复用原 1 m 内、最近优先、共享截止时间的格点
附接检查，终点不再只沿起点方向找格点。附接使用与前缀一致的内部体素及采样检查；
端点附接可用仍须完整搜索连接，不能据此放行执行或宣称存在通路。
端点／邻接可用仍不等于连接可用，最终由同一次执行语义连接搜索证明。
若推进搜索超时，不能以 observation incumbent 声称推进目标不可达。
SEARCH_TIMEOUT、NO_EXECUTABLE_CONNECTION_FOUND、BUDGET 分别报告。

观察记录绑定实际轨迹 ID；server feedback、合法末端 odom 和静止确认完成后，
相关 unknown→free/occupied 且新当前帧真实掩码覆盖该体素才记 GAIN。
NO_GAIN、DATA_UNAVAILABLE、COMPLETION_UNCONFIRMED 不重置同事件；自身移动不续发额度。
沿用 FSM 的等待证据机制。连接超时／未找到结果且未选中观察目标，不消耗观察动作，
不能封住下一轮恢复。已用观察记录只移除观察目标，仍允许当轮一次已观测推进
连接；推进成功不抹掉原观察结果。观察未提交也保留记录，邻近未知 key 或已知
掩码丢失不续发额度，相关新已知证据才解除同事件约束。
恢复请求目标集随既有 search evidence 保存，重放使用真实请求目标和冻结输入。

## 共享预算与质量边界

PlanningBudget 仍为总量 1.5 s、累计搜索 1.0 s、原三个修复额度，不提高或重置。
计时依据原森林 `20261009T083044Z_095`：成功轮次搜索后工作最高约 0.3869 s。
搜索保留 0.5 s 给后端／硬检查／提交，正常单次使用当前可用搜索量的四分之一，
剩余量供同一次恢复使用；恢复仍受累计 1.0 s 和总剩余量限制，无余量返回 BUDGET。
冻结森林输入显示原正常 0.5 s 后恢复仅剩约 0.27–0.33 s；一次离线连接取证
首条已观测路径约 0.488 s 出现（离线 2 s 搜索不授予在线许可）。按上述分配
同输入正常约 0.206 s、累计搜索约 0.809 s，后端／静止曲线检查通过；恢复
有实际工作时间且没有新增额度。可选质量工作仍保留 0.4 s。
这些是本机实测留量，不是极低耗时指标承诺。

guide retention 是质量报告；质量退化可触发最多一次修正，失败则保留已受检候选。
NOT_COMPARED／PREFERENCE_DEGRADED 不单独否决提交；最终硬检查仍必须完成。
正常、连接恢复、几何修正、后端重启及发布重查共用同一预算与原额度。

## 配置、验证与剩余边界

现场均为 `icra_dense_forest_four_fork_v2`，map seed 41021、GNSS seed 20260502，
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

当前现场证据：`20261009T195356Z_457`（SHA `26fb4aaf`），干净提交、Release
安装 exact_bytes、GPU READY（cuInit=0，device_count=1），原有效配置与 NAV 不变。
guide→实际静止曲线→执行反馈贯通，随后在 x≈−5.37 停滞，300 s 内无 task_reached。
末轮正常约 0.208 s、恢复约 0.624 s 均 TIME_BUDGET；未增加在线额度。
`terminal_final` generation=2554 的原 map／Current motion／ON payload／PVA
及原正常请求目标由 `export/analysis/v11_clearance_input.json` 绑定。
发现正常 guide 的渐变拟合预留反复使净空缓存整批失效。已有同体素精确对照
夹具扩展交替阈值：修复前 miss 从 4 增至 19，修复后保持 4，所有物理结论
一致；更大阈值仍能通过原精确扫描发现旧有限扫描之外的障碍。
现场同输入冷重放修复前后均通过，不能伪装成已复现现场超时：正常 guide 的
endpoint≈[-0.493607,-0.002605,1.4982]，静止曲线约 24.87 s，实际硬检查通过。
净空 miss 从约 60248 降至 34451；搜索仍受约 0.208 s 的正常截止约束，搜索及
后端耗时约 0.25408→0.24077 s，加原前置约 0.16794 s 均在原 1.5 s 内。
该几何缓存修复是否能在实际负载下持续提交并到达，仍待原任务现场复验。
相关 GridMap 精确物理对照、受影响实际曲线反例和 pipeline／full_stack_feedback 复用。

已有同输入代价与前缀证据继续复用：`20261009T192949Z_307` generation=2587，
统一 Advisory 读取减少重复访问，正常静止前缀 endpoint≈[4.50595,1.89239,1.50617]、
约 11.739 s，通过实际硬检查；独立较长执行连接在约 0.571 s 内仍超时。
该 run 末轮最新提交地图出现未知时正确拒绝，未借旧冻结检查放行。

已有完整观察连接证据继续复用：`20261009T181320Z_163` generation=2613 同身份
原 ON 输入在现有 baseline 入口交付完整目标 [18.33843,0.43888,1.60902]，
静止曲线约 2.69394 s，实际硬检查通过，累计搜索约 0.02631 s、总规划约 0.56812 s。
动作完成后的相关新观测收益仍由现场及原反馈机制判断，不能以预估收益代替。

已有同输入证据继续复用：generation=2551 的正常超时后恢复，累计搜索约 0.503 s，
实际曲线／静止末端通过；`20261009T175012Z_684` 的原 ON 短前缀重新拟合约
0.72066 s，动力学、实际曲线硬检查及静止末端通过。短段完成／错误反馈／尚未静止、
最终段等待原到达的定向反例及 pipeline、full_stack_feedback 均通过。
这些证据分别支持代价／连接、实际曲线和执行时序；终点附近或局部成功不能代替到达。
