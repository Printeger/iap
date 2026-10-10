# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)。
本文件记录当前流程、接口与验证边界；开发历史保存在 Git 和各运行的原始证据中。

## 状态与身份

开发起点 `f994cc9c632e8bb534cd9f47d6b663ad32673a47`，分支 `dev/iap_refactor`。
原版 ego-planner-swarm 保持只读；仅修改 IAP 仓库任务文件。

| 阶段 | 实际状态 |
|---|---|
| D1 | 完成：route/execute 分离、统一有限 Advisory 软代价、三个目标身份 |
| D2 | 完成：连续执行前缀、真实零末 V/A、实际曲线硬检查、提前接续及取消后的完整旧尾段检查 |
| D3 | 完成：一次有界执行连接恢复、相关新观测收益、共享预算及最多一次质量修正 |
| D4 | 完成：相关执行集成、原 ON 任务真实到达与实际连续性验证 |

当前状态 **DEV_ACCEPTED**。现场代码 SHA `70e9fab9bd02e16e0fb4694a36030b11b9fdf300`，
原完整 ON `20261010T092736Z_175` 在仿真第 **244.421 s** 由实际 ID86 触发原到达守卫，
到达审计 `20261010T093416Z_931` 确认；11 个子进程正常退出。
开启 RViz／地图显示，保留原 300 s 窗口。历史单次到达不代替本次证据。

本轮用户已解除开发现场次数／墙钟限制；每次原任务仍为 300 s，在线预算、任务与硬边界不变。
本次到达和连续性证据、残余停顿身份及简短报告位于 `log/20261010T094011Z_065/export/analysis/`。
DEV_ACCEPTED 不代表 PL 已校准、完整连续机体包络证明、正式统计资格或实飞认证。

## 正常主线与查询权限

原 FSM → beginPlanningView 冻结同一 GridMap／motion／risk 和唯一 PlanningBudget →
一个正常路线目标 → 同一 A* 比较有限软代价 → selectExecutablePrefix →
原 EGO 拟合／优化真实静止末端曲线 → assessTrajectory 实际曲线硬检查 →
可选质量报告／最多一次修正 → 最新 corridor／当前 motion／PVA／时效／接续检查 →
原子提交 → publicationStillTimely → 原 traj_server pending／active 反馈 → 原 FSM 监督。

GridRouteCell/queryRouteCell 允许物理未知，但不放开已知障碍、净空、motion 与时效拒绝。
execute 查询始终要求合法观测；route 许可不转成 execute 许可。
Advisory 根据合法 HPL/VPL 和原 budget/reserve 得到统一有限代价；warning 无 strict 通行拒绝
或 fallback 通行授权，缺失／失效仍为有限缺失代价，不伪装低风险或无障碍。
A* 独立 Advisory 回调统一负责首次与缓存样本代价；物理回调不重复计算 Advisory。
细采样逐点物理拒绝保留，软代价不能覆盖拒绝。

GuideIdentity 分开 mission_goal、route_target、committed_endpoint，记录冻结身份及区间。
参考进度来自真实车辆投影，不来自选中目标；局部停车或 guide 到达目标不算任务到达。
原到达守卫要求参考曲线完成、实际估计位置距任务终点小于 0.30 m、实测速度小于 0.1 m/s。

GridMap 净空缓存只保留冻结障碍距离上下界，以 generation 失效；阈值变化不清空全部缓存。
上下界不足时仍精确扫描，不复用旧接受结果。空有限扫描也只提供有限下界。

## Guide、实际曲线与停止末端

按既有执行点／边查询取 guide 的顺序连续前缀，遇到拒绝即停止，不能跨物理未知间隙。
前缀只用于初始化，最终执行许可来自真实 B-spline 的原硬检查。
正常与观察曲线均保持真实起点 P/V/A、零末 V/A；拟合、时间调整或修正后重查。

fitGuideCurve 按实际采样弧长同比例分配 nominal_interval 的物理时间，避免最少控制点
把短前缀拉成长时间慢动作。初始化弧长间距不大于半体素，保留原更密采样并保持总时间，
避免抹掉已观测转角。证据模型 guide_arc_half_voxel_arc_time_v3；重放读取历史 v1/v2 时
明确记录所应用模型，不改写历史记录。
未知／越界实际样本的 guide 支撑使用原执行查询；额外拟合余量不能抹掉物理合格支撑。
质量／拟合偏好仍使用自己的查询，最终曲线独立硬检查。

正常非零末速与直线 terminal_stopping_space 授权已退出在线主线。
assessTrajectory 的 completed 默认为假，空检查／预算耗尽不能发布。
原实际采样、短尾端点、坐标极值、净空、观测、motion、时效、tracking 与发布检查保留。
不新增完整连续机体扫掠包络，也不新增周围任意未知即拒绝的强包络政策。

## 正常提前接续、提交与取消

FSM 的 continuationTiming 统一决定触发与未来接续时刻，按实际 active ID／start time
缓存旧实际 spline 的末段减速起点。复用原 0.02 s 尺度扫描一次，仅用于时序，不授予物理许可。
保留原 1.6 s 提前量、1.5 s 总预算与 0.1 s 发布留量；触发取原 1 s 阈值和减速前完整窗口的较早者。
未来时刻必须在旧有效区间内、明显减速之前，且旧速度大于 0.1 m/s；同刻 P/V/A 初始化新曲线。
不再 min(1.6, remaining) 截到零速终点。短段／失窗不搜索非法 scheduled，而按原停车后恢复。
已有 pending 不重复搜索／提交；原实际激活、撤销反馈决定身份，不能靠时刻推断切换。

正常 scheduled 发布的最新 corridor 检查完整旧尾段和候选实际曲线，覆盖切换及 CANCEL_PENDING。
assessRemainingTrajectory 的 executing_tail_executable 只表示同 epoch 内完整 active 合格。
pending 单独失效，且完整旧尾段、当前 tracking、swarm 仍合格，才能取消候选并继续旧曲线。
取消不恢复失效旧尾段的许可，旧尾段条件失效仍请求受检保护。

实际曲线完成后，由 executingTrajectoryRestConfirmed 统一确认普通短段与受检制动的静止：
同 ID 末端命令、新鲜同 frame 的末端 odom、位置／实测低速度，以及真实曲线零末 V/A。
旧里程计低速度不能提前完成刚发布的制动。最终任务分支仍使用原到达守卫。

## 受检保护停止的未来接续

保护停止没有路线可供排名，不绑定完整 Advisory 预测或逐点计算软代价；原动力学和实际曲线
硬检查全部保留。Advisory 预警／短暂缺失不单独急停。

在原 EMERGENCY_STOP 入口中，仅实际同 ID 新鲜命令和运动接续窗口有效时提出原未来 1.6 s
停止连接。manager 用同一个最新 corridor 核对当前实测 tracking／frame／motion、激活前旧段、
完整实际停止曲线与两段 peer separation；停止起点 P/V/A 等于旧曲线同刻值，末 V/A 真为零。
原子 commit、0.1 s 留量、原 AT_TIME／pending 协议和实际反馈保留；不先覆盖旧 active。

监督仍检查完整旧尾段；只有原停止状态中已提交受检停止的 pending，才可另用该实际停止连接
证明。executing_tail_executable 仍为假，不授权取消后的旧全尾或普通延伸。
任一证明失效立即撤销并重新请求原受检保护；缺激活反馈重新挂起保护请求，迟到未来请求拒绝。
无法证明未来停止时，仅剩余原试次可求解实测立即后备；拒绝不代表保留的旧命令已经安全。

未来与立即后备共享原三个构造试次；活动规划视图中复用其 PlanningBudget，视图外独立保护动作
同样限 1.5 s，不给原正常搜索／恢复补预算或额度。没有新候选管理器、调度器、状态机或协议。

## 一次有界连接恢复与观察

当前 guide 前缀受阻、没有找到可执行连接、搜索超时分别报告，不能以乐观 guide 的未知截断
或过短前缀推断已观测范围没有通路，也不能无限等待。
正常 lookahead 已知冲突、正常搜索超时但尚有预算、拟合余量 START_BLOCKED／NO_PATH，
均可进入同一个恢复机会；实际起点硬拒绝仍拒绝。端点合法不代表连接合法。
截断非终点乐观前缀沿任务方向推进不足原 max(0.20 m, 2 voxel) 时，也在拟合前使用同一机会。
完整已观测绕行与完整正收益观察路径不套用方向筛选；原任务终点本身已知冲突不修改终点。

复用同一 A* 多目标搜索，优先已观测推进目标；确需改善观测时才选择观察目标。
最多八个廉价观察位置，以执行观测、已知障碍、邻接／连接证据、原 FOV 和正收益预估过滤，
不对八点分别完整搜索／优化。一次恢复交付一条 guide，随后只对所选目标求解 EGO。
冻结原 sensor/FOV 元数据随同一个 epoch 保存，搜索期间地图更新不使恢复误读为空元数据。
候选偏移适配原 FOV 内侧，不扩大 FOV 或改变探针身份。
原 1 m 内端点格点附接两端均检查，不能以端点附接可用代替完整连接。

观察动作要求到达所选观察 endpoint，不截取中途前缀冒充观察完成。
只有实际同 ID 完成、合法末端 odom／静止反馈与相关新当前帧覆盖，才判断 unknown→已知的真实收益。
NO_GAIN、DATA_UNAVAILABLE、COMPLETION_UNCONFIRMED 不因自身移动续发额度。
连接超时／未找到结果不消耗未交付的观察动作；已用观察额度仍允许本轮一次已观测推进连接。
请求目标与同身份冻结输入沿用既有导出器，区分 SEARCH_TIMEOUT、NO_EXECUTABLE_CONNECTION_FOUND 与 BUDGET。

## 共享预算与质量

PlanningBudget 保持总 1.5 s、累计搜索 1.0 s、原三个修复额度，不提高或重置。
依据原森林计时，搜索保留 0.5 s 给后端／硬检查／提交；正常单次使用当前可用搜索量的四分之一，
同一恢复使用剩余量，仍受总剩余与累计搜索约束。无余量返回 BUDGET。
可选质量工作保留 0.4 s，并保留最后一次原 repair 给硬提交重查；可选内部 BackendRestart
额度不足时软拒绝，保留已受检工作值，不置硬失败 denied。

Guide retention 是质量报告；NOT_COMPARED／PREFERENCE_DEGRADED 不单独否决硬合格候选。
最多一次修正，失败回到已有受检候选；最终最新 corridor／当前 motion／接续检查仍必须完成。
A* 发现队列采用原几何强度3×有限代价上限3的距离优先系数；实际边／风险／终端代价不变。
正常 CostProof 在首条路径后以可采纳距离界重开已发现节点比较；执行连接 Guide 只交付一条路径。
缓存与执行细采样拒绝保留。
冻结重放的解码／准备不属于在线路径；恢复原已消费资源时只扣一次，不在线重置预算。

## 可视化、启动与取证

用户入口只有 `iap_sim.launch.py`。launch 持有同运行根／ROS domain 互斥至全部子进程退出，
第二个图在创建节点前报 SIM_RUN_IN_USE，原 clock 多生产者硬拒绝保留。
现场绑定干净提交、Release 安装一致性、nvidia-smi／cuInit(0)／设备数预检。
既有开发取证工具不作为用户启动入口，没有增加启动脚本。

地图显示通过原 GetGridMapPredictionInput 的 planning_input=true／attempt=0 读取最近规划保留输入，
不额外冻结实时地图或更新规划状态。无输入显式缺失；原 PL 时间身份与历史过期判断保留。
热力图及历史彩色面投影到 z=0；fixed_z_m 只选择采样切片，查询仍跟随实际高度。
图例在地面，文字保留小幅偏移。既有显示进程用 GLIO 里程计发布 /grid_map/vehicle 四旋翼图标，
复用 hummingbird.mesh，保留真实高度／姿态，不依赖 PL 可用性；无更新 0.5 s 过期。
长期没有新规划输入时历史 PL 面按原 60 s 寿命消失，不续期旧 PL、不填未知。

capture_failure_map 开启时，原 execution CSV v2 记录 P/V/A、rolling_window、规划／发布／反馈／拒绝，
并以 0.1 s 尺度记录实测 motion；取证关闭时不启用。
实际失败 ID 首次完整尾段拒绝保存同 epoch 地图／curve（最多128），沿用原有界写队列与 manifest。
初始提交不代表发布／激活，planner 首次收到反馈的延迟也不代表服务端激活迟到。
实际连续性使用 traj_server 激活时刻、同刻 spline／实发命令和 GLIO 实测运动分别核对。

## 配置、验证和残余停顿

原 scene icra_dense_forest_four_fork_v2，map seed41021、GNSS seed20260502，
初始[-18,0,1.5]、终点[18,0,1.5]，ON／prior OFF，原300 s与到达规则。
NAV hash `42e87bd2edff3bb66e6d58b01c1710e9547270cfe795203ce947b8d88356bd32`。
v=0.5 m/s、a=2.0 m/s²；PL H/V budget=0.55/0.60 m、reserve=0.10/0.10 m；
body=0.35 m、tracking reserve=0.10 m、tracking limit=0.30 m、motion/environment age=0.5 s。
FOV、GLIO、地图来源、执行消息、预测模型和到达规则未改变。

六组场景与原入口覆盖未知 guide 有已观测旁路、最高收益点断连但另点可达、预算不能重发、
唯一 warning 通路通过执行及取消后旧尾段继续受检。已有通过结果复用，不重跑全历史。
真实92→93冻结反例先红后绿，当前停止／连续性9项原生及原3项执行集成通过（17.74 s），
最终保护／预算烟测通过；证据 `20261010T090340Z_961`。

当前原现场25次正常非零接续均在旧明显减速前实际激活，同刻参考速度差最大约0.000706 m/s；
45→46→47→48→49→50链，其中前四个接缝两侧有0.25 s内非零GLIO实测样本。
11次未来受检停止中7次实际激活，初始实发速度约0.300–0.348 m/s；4次因证明失效撤销。
相对前次同配置 `085620Z_383`，非终点低速样本段13→4、累计采样跨度38.10→5.60 s，制动发布38→26。
采样有间隔，跨度不当作完整连续低速时长或正式统计。正常规划最大1.029 s、累计搜索0.744 s，
受检未来停止最大0.165 s，保持原预算。

四段残余低速分别绑定：ID25为未知／停止证明撤销后的受检保护；38→40为pending39未知撤销后
合格旧零末端停车；53→54为release／预算拒绝后失窗；73→74为release拒绝后的静止后备。
真实净空／未知／时效及tracking保护仍生效，不承诺任意环境不停。
当前D1→D4及追加连续性开发验收已完成，无范围阻塞；未开展PL校准、9+9、OFF对照、
六次任务统计或完整连续包络证明。
