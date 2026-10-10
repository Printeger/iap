# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)，包含第 0 节连续飞行追加授权与修订后的 §9.1。
当前追加 Goal **IN_PROGRESS**；代码与离线 **IMPLEMENTED**，现场尚未验收，不继承历史 D1→D4 DEV_ACCEPTED。开发基线
`b8b8bcba15b672ca902c93eec50de171d39df05b`，当前分支 `dev/iap_refactor`，开始时工作树干净。
原版 `src/ego-planner-swarm` 只读；历史运行与证据只读。

## 主线与实际执行范围

保留 D1 route/execute 分离、有限 Advisory 软代价、三个目标身份；D2 原 EGO、真实零末
V/A、离散实际检查；D3 同一个 A*、一次有界连接恢复和原共享预算。追加 D4 验收仍待完成。

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
1.5 s 预算与 0.1 s 发布留量；实际接续须早于明显减速且旧速度 >0.1 m/s。窗口不足的短
停止动作可执行，但不计连续巡航成功。停车确认仍由 executingTrajectoryRestConfirmed
核对同 ID 末端命令、新鲜同 frame 里程计、位置／实测低速及 spline 真零末 V/A。

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

原参数冻结 ID3：A3→B4→C5，接续速度 0.333 m/s，C 完整时长 3.70 s；总处理
1.14 s、单次累计搜索 0.882 s。C 请求丢失／拒绝时实际 A3→B4，B 0.50 s 后真零
V/A；C 反馈缺失时不覆盖本地 B，实际 C5 命令才确定激活身份。C5 再失效时受检即时
停止 ID6 替代，未裸取消／恢复 A3。丢失反馈处理约 1.426 s，未重开预算。原参数
ID16 的唯一 C guide 只有 0.344 m 合格前缀，未达到原推进要求，应验证 A16→B17
真实停车，而非将不合格短 C 记为连续成功。机制默认参数的早期测试不代替原 ON 参数
复核，参数环境误绑定记录保存在新分析 run 的 unit_parameters_results.json。

受影响原执行／保护 9 项、追加时效／激活／合法取消 5 项、server 2 项、冻结分析工具
22 项及入口契约 46 项已通过；
B 反馈丢失时没有获得接续授权，单次搜索数为 0，服务端实际 B4 仍激活并真零停车；
ID16 后备检查通过：1.022 s 总处理、0.829 s 单次搜索，B17 0.50 s 真零停车。
冻结 C 实际末段减速起点 3.30 s，早于减速的 1.6 s 接续窗口具备。
证据汇总 `log/20261010T113748Z_630/export/analysis`，主原参数成功复核
`20261010T121121Z_520`，B 反馈缺失 `20261010T121331Z_226`，ID16 后备
`20261010T121512Z_172`；原参数 C 故障／撤销见 native_results.json 对应 run。
第一岔口现场与原任务到达待验收，
当前不标记 DEV_ACCEPTED。

## 配置与现场门禁

唯一入口 iap_sim.launch.py；icra_dense_forest_four_fork_v2，map seed41021、GNSS seed20260502；
初始[-18,0,1.5]、目标[18,0,1.5]，Advisory ON／prior OFF，每次原300 s窗口，RViz＋风险图。
NAV SHA256 `42e87bd2edff3bb66e6d58b01c1710e9547270cfe795203ce947b8d88356bd32`。
v=0.5 m/s、a=2.0 m/s²；PL H/V budget=0.55/0.60 m，reserve=0.10/0.10 m；body=0.35 m、
tracking reserve=0.10 m、tracking limit=0.30 m，motion/environment age=0.5 s。
无安全参数、FOV、来源噪声、地图证据或任务到达规则改变。提交干净工作树、共享消息／
预算头的依赖全链安装一致与 GPU 门禁必须通过；同 SHA／配置无新证据不重复现场。
