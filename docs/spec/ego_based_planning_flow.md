# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)，包含第 0 节连续飞行追加授权与修订后的 §9.1。
当前追加 Goal **DEV_ACCEPTED**：冻结反例的真实执行分支、身份与原硬检查验证通过，
`05224791` 原四分叉 ON 任务在266.403 s到达，连续性逐身份审计完成。不继承历史状态，
不等于全程不停、任意输入安全或实飞资格。开发基线
`b8b8bcba15b672ca902c93eec50de171d39df05b`，当前分支 `dev/iap_refactor`，开始时工作树干净。
原版 `src/ego-planner-swarm` 只读；历史运行与证据只读。

## 主线与实际执行范围

保留 D1 route/execute 分离、有限 Advisory 软代价、三个目标身份；D2 原 EGO、真实零末
V/A、离散实际检查；D3 同一个 A*、一次有界连接恢复和原共享预算；追加 D4 按本次授权验收完成。

原 FSM → beginPlanningView 冻结同一 GridMap／motion／risk → 一个路线目标 → 同一个 A*
交付一条长 guide → selectExecutablePrefix 顺序检查至首次拒绝 → 选择局部停止终点 →
原 EGO 拟合／优化完整曲线 → 实际硬检查 → 有余量的质量报告／原一次修正 → 最新 corridor、
motion／tracking／PVA／时效／发布检查 → 原 traj_server → 权威队列结果与实际命令 → 原 FSM。

未知可参与 route 意图，始终不授予 execute 许可。Advisory warning、缺失和历史 PL 显示年龄
不单独否决实际运动，也不改变物理事实。GridMap 净空缓存仍只保存按 generation 失效的
距离上下界，不缓存不同精确位置的接受结果；证据模型、来源噪声、年龄与净空不变。

mission_goal／route_target／committed_endpoint 分开；参考进度来自实测投影。选中原目标、
局部停车和 guide 到达不算任务到达。原到达守卫仍要求参考完成、估计位置距终点 <0.30 m、
实测速度 <0.1 m/s。

## Guide 与局部曲线

selectExecutablePrefix 保留原 ray traversal、半体素细采样与原执行查询，遇拒绝结束，不能
跨未知缺口。非终点截短前缀缺乏既有最小推进尺度时，正常规划仍可使用原一次连接恢复。
观察动作须保留已证明收益的原观察终点，不能截短后虚报观察完成。

正常前缀拟合范围由 `ctrl_pt_dist / nominal_guide_interval` 的实际初始化速度、两个原
1.5 s 规划＋0.1 s 发布窗口，以及已有实际末段减速尺度／原停止构造尺度推导。异常过长
旧减速段不能迫使下轮继续承诺长尾，使用原速度／加速度停止尺度限制其时域贡献。
长 guide 的路线目标保持，局部终点仍必须处于连续受检前缀内。只缩短 guide 拟合输入，
不裁剪 spline；原 EGO 独立生成同一未来 P/V/A 起点和真实零末 V/A 的完整曲线。

fitGuideCurve 保留 `guide_arc_half_voxel_arc_time_v3`：弧长按 nominal_interval 分配实际
时间，初始化采样不大于半体素；最少控制点不会把短动作拉长。最终实际 spline 的边界、
逐轴动力学、离散观测／净空、坐标极值、短尾端点和时效检查继续独立负责许可。
正常非零末速和直线 terminal_stopping_space 授权仍已退出主线。

continuationTiming 仍按实际 ID／start time 扫描 0.02 s 末段减速起点，保留 1.6 s 接续提前量、
1.5 s 预算、0.1 s 发布留量与原200 ms监督周期；实际接续须早于明显减速且旧速度 >0.1 m/s。窗口不足的短
停止动作可执行，但不计连续巡航成功。停车确认仍由 executingTrajectoryRestConfirmed
核对同 ID 末端命令、新鲜同 frame 里程计、位置／实测低速及 spline 真零末 V/A。

EXEC_TRAJ 达到原触发条件且当次还有 moving window 时，在同一回调启动已有
planFromCurrentTraj，避免转 REPLAN 后等待下一回调消耗接续窗口。事务重新读取当前
时刻并通过原完整检查；失败重试、短停止和停车后起步仍走原 REPLAN 路径。

## 队列权威与远端失效

Bspline 保留 IMMEDIATE／AT_TIME／CANCEL_PENDING，并扩展 REPLACE_PENDING、predecessor_id、
replace_pending_id。traj_server 仍是唯一队列／命令所有者。`planning/trajectory_feedback`
提供 request ID／mode、accepted／reason、实际 active／pending ID 和请求生效时刻；与
Bspline 同一 drone 主题映射。结果产生于服务端接收回调的同一原子决策之后。

scheduled 请求必须有匹配 active、未来且位于旧有效区间的 ts、精确 P/V/A；替换还必须有
匹配的 pending B、同一 ts 与 B/C 边界。迟到、重复、乱序、身份／边界错误保留原队列。
已接受 ID 单调保留；B 激活后本次 C 不再插入。替换不发送 CANCEL_PENDING。

远端 A 拒绝 → 原 EMERGENCY_STOP 入口检查真实当前至 ts 前段、tracking／motion 与完整
停止 B（含 peer separation）→ 提交 B → 等待服务端接受为 pending 的权威结果 → 同一预算
有余量且实际分支仍合格 → 一次 A* 长 guide／一次 EGO C 事务 → 最新检查 A 前段＋B＋C →
发送 REPLACE_PENDING → 匹配结果接受 C，或保留 B；求解、预算、迟到／拒绝均不清空 B。

manager 只保留一个 active、一个 pending B 和一个工作 C。C 提交前与等待结果期间不覆盖
B；结果不确定保留 B/C 两个可能身份及分支检查，实际同 ID 命令解决激活身份，不按时刻
推断切换。实际切换、求值与命令时间戳仍共用一次 server 时钟采样。

确认 B 且分支合格时，实际授权为 A 当前至 ts → B／C → 各自静止末端，替代保证不再执行
的旧远端。executing_tail_executable 仍单独表示完整 A 尾段当下合格。没有确认／分支证明
保留完整旧尾段规则；近期／motion／后备不合格不能靠远端 lead 延长运动。

取消任何 pending 必须仍有完整 A、tracking 与 swarm 合格依据。C 接受后再失效且 A 尾段
不合格，不能裸取消恢复 A；原受检替代／停止入口处理，失败明确无替代动作获准／保证
失效。完整 A 合格时保留原合法取消；权威取消结果或原后启动时刻的实发 A 命令解决身份。

## 共享预算与保护响应

B 构造、检查、接收确认、C 冻结／搜索／求解、最终硬检查／提交／确认共享同一个
PlanningBudget；总上限 1.5 s、累计搜索 1.0 s、原 3 修复额度不增加／重置。B 重构消耗
原 BackendRestart 额度。beginPlanningView 可绑定已有预算，不能重建余量。
已有 pending 不触发重复完整搜索；本任务唯一新增机会是确认停止 B 后该窗口的一次 C。
C 不作第二次恢复搜索；失败让 B 执行。

原搜索后 0.5 s 后端／检查留量、可选质量前 0.4 s 留量和最后修复额度保留。
PlanningBudget::workExpired 在原 solver 工作检查点复用原实际分支监督，按原 200 ms
尺度检查当前输入与 B／可能 C；输入／分支失效中止 C，并记录 execution proof lost。
expired 不运行重型监督，以免在 GridMap 原子提交持有地图锁时递归取锁。命令、odom 与
权威结果的独立 callback group 继续及时接收。等待 B 确认时按同一原 200 ms 尺度复用
inspectPendingStopProof 检查 A 近期＋B；未确认证明只用于中止等待，不授予执行许可。
等待反馈保留原 0.1 s 提交留量，缺失结果不耗尽返回原保护所需的时间。此机制不是另一套 FSM／许可框架。

assessRemainingTrajectory／assessTrajectory 接受同一个可选 PlanningBudget；B 确认检查、
C 求解 guard、A 近期／B／可能 C 的实际采样与 corridor 捕获传递同一对象。预算过期
返回 completed=false／budget_exhausted，不伪称物理拒绝；保留已确认 B，不重开 C 预算。
等待结果先消费已到的权威反馈，再做必要监督。

预算内 corridor 保留同 generation 的全部实际采样、raw 比较邻域、观测／净空／motion
硬事实；地图 mutex 按同一截止时间尝试取得。全地图 failure_evidence 是可选诊断副本，
此预算路径不复制；原普通监督仍保存完整冻结诊断，已有规划输入保留全图。
最新检查没有完整诊断副本时保持 unavailable，不借用新旧地图或伪造来源。

## 冻结证据与验证边界

`analyze_curve_observation.compare_commit_rejection` 对同 spline 拒绝时刻／位置读取提交最终
检查 mask 与首次拒绝 mask，计算同点观测／来源、raw 距离、所需净空、误差代理、年龄和
采样相位；缺失字节明确报告 unavailable，不使用历史 PL 缓存年龄归因物理过期。

100318Z_883 实际 ID6、ID64 的提交与拒绝 spline 相同；采样起点有变化，但提交及拒绝
地图字节均缺失，不能判定同点来源／障碍变化。100813Z_456 pending ID10 的同点 observed
保持，最近 raw 中心距 0.674→0.544 m，所需净空约 0.548 m，构成真实净空变化；不能把
pending ID10 描述成当时执行的 A（当时 active ID9）。同 run 实际 ID3 的未知点无当前／
active 来源，最近清除 producer=current_replace；producer 是历史标签，缺少同 ID 提交
地图，不能确定撤销时间。实际 ID16 在 23.24 s 曲线执行约 0.168 s 后拒绝远端净空。

定向 server 测试先红后绿，覆盖 B→C 原子替换及实际非零接缝、错误身份／ts／边界、
重复／乱序／迟到与队列保留。扩展原 CapturedDistantRevocation 测试，加载原失败 map、
实际 spline／PVA、原参数，以真实 FSM→manager→server receive／cmd 链验证 B 确认、C
生成／替换／激活及 C 失败后 B 激活。冻结时刻不刷新；这是离线机制证据，不能替代新鲜
现场观测、完整实时 ON 输入及闭环实测。证据汇入新 run 的 export/analysis 与 runtime。

原参数冻结调用链以当前 `05224791` 复核：A3→B4→C5，真实接续速度0.333 m/s，
C 完整时长3.70 s；处理1.091 s、单次搜索0.924 s。C 请求丢失／拒绝时实际执行 B4，
0.50 s 后真零 V/A；B4／C5 证明撤销分别进入受检保护5／6，不能恢复失效 A3。
C 接收反馈丢失时保留不确定身份，由实际 C5 命令确认；处理1.412 s。
B 确认反馈丢失时 C 搜索为0，服务端实际 B4 激活并停车；处理1.402 s。

冻结实际 ID16 的 C 只有0.344 m合格前缀，未达到原推进要求；真实 A16→B17，接续
速度0.329 m/s、B17 0.50 s真零停车，处理1.028 s、单次搜索0.888 s。该短动作不计
连续成功。ID3、ID16 的离线风险夹具在冻结时刻重构：原历史完整预测输入未保存，
不能称为完整历史 ON 重放；物理 map、实际 spline、motion／误差代理与原参数均绑定。

当前18项原保护／预算测试、6项地图／证据契约通过；真实 server 2项覆盖原子替换及
拒绝、迟到、重复、乱序、身份／ts／PVA 与队列保留。合法取消仍需完整旧尾段，失效
旧尾段的取消被拒绝。分析工具22项、入口契约47项通过。预算诊断复制、触发回调和
分支失败身份的定向验证先红后绿。capture 直接保存权威反馈；仅 context 已关闭且
message-take 错误匹配关闭竞争时正常收尾，活跃 context 的相同错误仍失败。

原冻结比较／验证见 `log/20261010T113748Z_630/export/analysis`；当前预算／保护与
五种原参数分支复核见 `log/20261010T132641Z_063/export/analysis`。参数环境误绑定
的默认参数测试和五个 SKIPPED run 有明确排除记录，不计入原 ON 通过证据。
最新反馈与 ID16 复核见 `log/20261010T135000Z_896/export/analysis`。

## 配置与现场门禁

唯一入口 iap_sim.launch.py；icra_dense_forest_four_fork_v2，map seed41021、GNSS seed20260502；
初始[-18,0,1.5]、目标[18,0,1.5]，Advisory ON／prior OFF，原300 s窗口，RViz＋风险图。
NAV SHA256 `42e87bd2edff3bb66e6d58b01c1710e9547270cfe795203ce947b8d88356bd32`。
v=0.5 m/s、a=2.0 m/s²；PL H/V budget=0.55/0.60 m，reserve=0.10/0.10 m；body=0.35 m、
tracking reserve=0.10 m、tracking limit=0.30 m，motion/environment age=0.5 s。
无安全参数、FOV、来源噪声、地图证据或任务到达规则改变。提交干净工作树、共享消息／
预算头的六包 Release 全链安装一致与 GPU 门禁已通过。运行代码 SHA
`052247919be8e2eeb082e4e62b931083819cb7ab`；安装 ego_planner_node 指向 build 同一文件，
SHA256 `a3af513153b1efaccc90b44b5a54c5e9da83f416ca2d179c757ed4936653a182`。
GPU nvidia-smi、cuInit(0)和设备数1均通过，READY；同 SHA／配置无新证据不重复现场。

## 当前任务验收与停顿边界

现场 `log/20261010T133956Z_514`：原全部 planner 参数与保存输入逐项相同，四入口
(-16、-8、0、8 m)均捕获；RViz、风险图及11个必要进程覆盖到达，四作业正常退出，
无必要输入失败。原 task_reached 为1657109066.403，历史时钟起算266.403 s，首次规划后
254.118 s；原规则独立审计 `log/20261010T134854Z_289/export/planner/task_arrival_audit.json`
确认非初始 WAIT_TARGET。普通 process manifest 的 mission_pass=false 表示生命周期
不等于任务资格，保留原值；本次 DEV_ACCEPTED 是本追加授权的开发验收，不是形式／实飞安全资格。

75个实际激活ID，47次非零 P/V/A 接续，全部早于旧曲线明显减速，接缝误差≤1e-5。
A30→B31→C32 与 A35→B36→C37 均有服务端先确认 B、后原子替换 C 的直接反馈，
同一 ts／PVA，实际 C 激活；接续速度0.337／0.333 m/s，处理1.036／1.112 s，
累计搜索0.818／0.828 s。A30 是远端净空失效；A35 的首次输入年龄0.502 s确实过期，
其后仅在新鲜输入、受检 B 和实际分支合格时换路，不把过期输入继续当作许可。

99条预算记录：总处理最大1.111723 s、累计搜索最大0.882899 s、修复最多2，未越原上限。
仍有21段非终点零速命令80.019 s；离散 GLIO 实测低速(<0.1)35段14.493 s。
两者含义不同，不能声称全程持续巡航；未完成连续机体包络、任意可达或预测模型校准。
逐ID证据在 `log/20261010T135000Z_896/export/analysis/per_identity_stop_review.json`：

| 实际静止身份 | 触发／处理与验证边界 |
|---|---|
| 7、24、70、74、75 | 原受检短停止动作，正常巡航窗口不足；24另有近期净空／实测动力学保护失败。 |
| 9、11、12、15、20、26 | 净空／分支不合格后的受检保护停止；15之前 B14 已确认但最新分支失效，未继续 C。 |
| 53 | 原物理输入过期，受检停止；非历史 PL 显示年龄。 |
| 10、38、44、56、65、69、73 | 接续已尝试但最新发布、目标、预算或曲线事务失败；ID10随后近期净空失效进入原保护，其余保留当下合格的静止末端。失败枚举不定义地图根因。 |
| 22、71 | 正常 pending 23／72 已交付，随后净空／观测拒绝；完整旧尾段及原 tracking／peer 合格时合法撤销，server确认后保留旧静止末端。 |

15个 moving_window_missed 身份全部可追溯：9个此前已有规划／pending撤销，6个
(ID7、24、70、74、75、80)原短停止无正常窗口；无此前“有窗口却未尝试”的同回调缺口。
ID80的短末段靠近任务终点，不纳入非终点命令停车统计，但保留窗口失败记录。
Release阶段细分提交条件缺少完整捕获时如实保留不确定，不据此断言地图实现错误。

现场9条“保证失效”日志涉及实际ID10、11、13、24：原停止构造在近期净空或实测
动力学检查失败，未授予替代；保留命令不是安全证明。首次拒绝同代 raw中心距／所需净空
分别约0.54655/0.54694、0.47947/0.54612、0.44956/0.54662、0.54528/0.54567 m，
观测均为true，物理年龄约0.20–0.315 s；不是远端尾段可以排除的单纯拒绝。
ID24还记录实测动力学比1.309。其后新鲜检查通过才发布保护11／12／15／25／26，
不追认此前失效区间。重复近端保护失败保留为本轮限制，不能把这些起停称为已消除或安全巡航。

最终审计包括完整实际接缝、B/C权威反馈、逐身份停顿／低速、原到达、配置／健康／GPU与
当前冻结调用链，汇总 `log/20261010T135000Z_896/export/analysis`。旧现场与失败收尾记录
不改写；原地图／传感器模型未更改。此验收关闭本追加目标，保留上述必要保护与证据限制。
