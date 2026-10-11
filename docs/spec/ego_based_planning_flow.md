# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

本轮 pending／来源排序追加 Goal 正在实现，基线 `1d1b1a6b`，工作树与安装字节一致。
原 active33／pending34 在 gen637 的定向反例：A 完整尾段拒绝，但 A 至 ts 和完整 B 合格，
原普通 pending 监督却拒绝并进入急停（`log/20261011T044535Z_421/runtime/pending_red.log`）。
`assessRemainingTrajectory(now, include_advisory, budget)` 统一以 server 确认身份选择分支，
删除 caller 的 pending_checked_stop 参数与构造标签 pending_branch_proof；已确认且未撤销的
普通／保护 pending 同用原 assessCheckedStopConnection。完整 A 尾段仍单独检查并决定取消
资格；未确认、接缝／连接／pending／motion／tracking／peer 失败仍拒绝。原确认和实际
命令权威、原预算与许可阈值保持；超过 ts 缺少实际激活反馈不推定已执行。
等待 B 确认因分支失效退出时，原发布动作不能清除保护请求；同预算复用
inspectPendingStopProof 确认失效并返回 false，保留队列身份，走原受检保护入口。
当前状态 IMPLEMENTED，现场待验证。真实 A33/B34 反例红→绿、11项直接执行回归、2项
server 回归通过；原冻结 A3 的 success／rejected／lost／feedback_lost／backup_feedback_lost／
backup_revoked 六分支通过，失败替换实际执行 B4，撤销资格后实际激活受检保护5。
`log/20261011T044535Z_421/runtime/pending_atomic_*` 保留首轮等待失效 red 及修复 green。
这些是固定 epoch 原生调用链证据，不追认历史区间，也不授予未来地图持续保证。B 排序尚未取得
独立支持，C 前置未通过、未启动新收益对照。固定地图局部退化代理不含历史绝对漂移、
全故障或认证 PL。

短程 ON4101 `log/20261011T045538Z_253` 绑定干净 `65745197`，GPU READY、11个安装产物
与 Release 构建一致，进程正常收尾。新链身份／路线不同于历史反例，不按相同 seed 追认
历史 A33/B34；4条近期／年龄能力边界保护失败仍保留，不宣称整段连续保证。追加取证
`TrajectoryAssessment.executing_tail_refusal` 保存被分支替代的完整 A 原拒绝及其 immutable
epoch；FSM 通过原 captureRemainingFailure 保存 excluded_tail_ID 和原事件时刻／首拒绝点。
不重查或隐藏拒绝，不改变许可或预算；原 writer 最多保存32个 excluded_tail_ID。
原生 A33/B34 回归验证原 gen637／INSUFFICIENT_CLEARANCE／首拒绝时刻保留，实际 B34
命令与 manager 激活身份闭合（`runtime/pending_refusal_green.log`）；现场身份审计正在完成。

LiDAR 排序追加验证使用已冻结的 dense_lidar_ranking_protocol_v1，独立 OFF seeds
5101／5102／5103，每次90s，规则与区域在查看新结果前固定。诊断接口
`capture_dense_lidar:=true`／driver `--capture-dense-lidar` 要求原 residual capture，
仅 simulation full_stack 可用，默认关闭。原 postopt quality batch 中最新 fixed target
level-0 因子最多16384个 source slots，其他因子保持128；不加 matching、重试、
quality pass 或同步，不改 cost／Hessian／GNSS／规划预算。只有 stride=1 且采样数与
原 inliers 闭合才称完整原匹配；逐请求保留 source slots、有效数、唯一 source／target
与 queue drop，超限依然稀疏并显式标记。临时 pinned buffer 在原析构同步后释放。
fixed_map_increment 诊断按原 source 身份逐项映射生产 primitive CSV 的原始行号，
同时保留半径子集行号、GPU target 身份、同表面族和缺失原因；以相同冻结地图、
原 IMU 参考、0.5m 关联及真实扫描均值解局部增量，真值仅评价。修正原子集索引
冒充 primitive 原行号的取证错误；模型矩阵与参数未改。跨帧／共享地图／当前扫描
自相关仍是条件限制。CUDA 原代价与 Hessian 一致性、取证异常、16384点队列压力、4项 host audit
与48项启动接口检查通过（`log/20261011T044535Z_421/runtime/dense_*_v2.log`）；
历史8个样本408条关联修正原行号并重建模型矩阵闭合，原排序 rho=.857 未改变。
独立数据尚未采集，C 的排序前置未通过。

风险引导轻量实证闭环已完成，包含不通过结论；起始HEAD `2c8cf4bd`，工作树与安装身份已核对。
规划实验提交`040c5ed7`与离线资格修复`fb4980c4`分别验证，未修改生产融合模型或风险权重。
记录权威为`log/20261011T022904Z_324`；上一轮报告、输入及失效记录保持只读。
图文报告：[`empirical_report_v1/REPORT.md`](../../log/20261011T022904Z_324/export/analysis/empirical_report_v1/REPORT.md)。

实验启动参数`protection_margin_replan:=true`（driver `--protection-margin-replan`）
仅启用`planning/protection_margin_replan_enabled`，默认关闭，未推广生产默认。
同一FSM在完整实际尾段受检合格后，对已冻结corridor的当前实际曲线位置作一次原始
净空标量查询；同身份、新鲜相邻监督记录的余量下降且小于一体素，或按已测下降率
接近原1.6s接续＋200ms监督窗口时，请求原REPLAN。它只是调度，不是停止空间许可。
每身份只登记首次请求；原按需异步writer保存`reserve_declining_ID`的同epoch曲线、
motion/tracking/command时序，reason保留OK，不冒充拒绝。没有新图、距离场或执行器。

原合法前进事务优先；若失败且物理余量未恢复，剩余同一1.5s PlanningBudget内才可
调用原planCheckedBrake及原发布/反馈权威。预算不重建，pending分支不干预；普通失败、
恢复余量、Advisory预警/缺失均不独立触发保护。拒绝保护后保留并监督旧运动，不能称
已停或合格悬停。突然变化及起点已违规仍可能没有许可动作，不授予连续保证。
定向反例原来在余量下降时不处理，到起点净空违规才尝试并失败；实验实现提前请求，
原硬检查通过的保护曲线经原server receive/command回调真实激活。稳定/恢复余量不制动。
验证：14项相关原生检查与48项启动接口检查通过（`runtime/protection_native_v9.log`、
`runtime/protection_launch_v2.log`）；原反例red保留`runtime/protection_trigger_red.log`。
扩查发现既有撤回测试与完整旧尾段许可冲突：它在起点未观测时仍期待撤回，恢复段
未更新FSM实测位置。基线2c8cf4bd源码单独编译同样失败（`protection_pending_baseline.log`）；
修正测试前提，未放宽生产许可。实际配对OFF/ON均绑定干净040c5ed7、相同seed4101、
prior OFF、原预算/物理政策/任务目标。ON真实owner23在余量仍约.295m时请求保护，
原受检.5s制动24由server激活并于44ms后发送实际命令，该窗口无保证失效。
后续active33/pending34在新地图下撤销旧尾段资格，原制动两次动力学不合格；
直到新受检35命令激活仍有.195s保证缺口。34未先激活；保留33命令不等于受检悬停，
不由此推断碰撞。此实验不提供pending期间持续合法备份或突然变化下的连续保证，
不再追加ON收益运行，不推广默认。OFF本窗口11次余量请求通过前进处理、无提前制动；
仅支持所观察窗口及稳定/恢复余量原生反例，不承诺所有正常条件均无制动。

既有GPU数据的9/17局部固定target增量求解已与独立锚点真值相对位姿及独立白化求解
交叉核验；原stamp、50ms truth资格及缺失/拒绝保留。GPU子地图不等于Predictor注册图，
GNSS传播参考也不同，故`local_matched_increment_v1/summary.json`不授予融合空间排序。
同生产GridMap、真实采样与原IMU传播参考的局部增量诊断现已完成，冻结后新OFF4101
保留9请求/8完整输入/8求解。工具`scripts/dev_predictor/fixed_map_increment.py CONFIG LABEL`
只在已有run内登记离线结果；星座钟差＋位置的实际设计矩阵先核验列秩，秩不足保留
`GNSS_POSITION_UNOBSERVABLE_AFTER_CLOCK_NUISANCE`，不输出普通有限单源协方差。
本轮实测3行/5列反例红→绿；20个融合/LiDAR求解与H/V及排序结果逐项不变。
这是诊断器资格修复，未改变生产模型、噪声/K/风险权重，未用已查看留出拟合。
稀疏扫描关联与完整Predictor支持集仍不相同，共享地图/源内相关及IMU传播不确定性
未经经验校准；不能用solver covariance当真值或授予绝对误差/95%覆盖。
同对象实际求解的H代理/局部误差排序：诊断seed3101为12/17合格、Spearman .678；
冻结后独立OFF4101为8/9请求合格、.857；同一冻结估计器的配对ON为8/9、.190。
弱方向平均对齐.490/.603/.617，尚无方向一致性资格。稀疏支持、共享地图及后验GNSS
残差限制保留；不因部分正相关宣布普适排序，不拟合噪声/K，不授予米数覆盖。
同一真实入口committed_5的675非插值空间查询中，GNSS-only无合格有限值，
LiDAR/Fusion各477，198物理过滤；按共同边缘弱方向消元投影，GNSS信息份额至多
2.34e-5，LiDAR主导是条件模型结论。全局部目标两侧均END_UNOBSERVED；仅原guide
受检前缀同端点OFF/ON/充足离线参考取得同一路径与离散最优性，统一成本33.842192m。
实际首分岔x[-15.5,-9]全段在共享冻结图中均不可完整评估；共同受检x[-15.5,-13.8]
的实际终点y/z不同，不能由较低部分成本宣布ON胜出。ON首分岔实际12.168m/34.630s，
OFF9.547m/25.929s；下沉已在guide，owner7后端偏离至多.0381m、3.7s时长不变，
wire与实际命令身份一致。未证实后端丢失路线，也未取得整体ON任务收益或全支路排序。
同owner7原epoch统一stage成本：guide33.010839m→优化32.926291m，长度/风险附加
略降，原dynamics及release保持同成本，未通过新离线曲线冒充现场执行。

唯一需求基线：[IAP_Safety_Planner_Requirements_v1.1_20261009.md](IAP_Safety_Planner_Requirements_v1.1_20261009.md)，包含第 0 节连续飞行追加授权与修订后的 §9.1。
当前追加 Goal **DEV_ACCEPTED**：冻结反例的真实执行分支、身份与原硬检查验证通过，
`05224791` 原四分叉 ON 任务在266.403 s到达，连续性逐身份审计完成。不继承历史状态，
不等于全程不停、任意输入安全或实飞资格。开发基线
`b8b8bcba15b672ca902c93eec50de171d39df05b`，当前分支 `dev/iap_refactor`，开始时工作树干净。
原版 `src/ego-planner-swarm` 只读；历史运行与证据只读。

上一轮 Advisory 轻量验证闭环完成（包含不通过结论），连续飞行主线不变。仿真可显式指定
`gnss_observation_seed`（driver `--gnss-seed`），只改变 GNSS 随机抽样 seed；
与 `advisory_trial` 的 seed 权威互斥。默认留空维持原 seed，实际 seed 登记在
full_stack 与 scenario 清单；地图、噪声幅值、任务、风险权重、物理检查与在线预算均保持。
诊断 OFF seeds 1101/1102，留出 OFF seed2101，禁止使用留出结果调整本轮模型。
该接口的 launch 取参验证比较全部 GNSS 参数，要求只有 random_seed 改变。

Advisory 当前语义为 `pose6_fixed_two_source_envelope_v2`：GNSS 独占星座钟差先消元，
两源共同 map pose（位置＋左局部姿态）先联合、再边缘化位置；两源或单源均保留
相同 factor-two 零均值跨源包络。官方条件 H/V 为未正则化位置边缘协方差的
`K_H sqrt(lambda_max(C_xy)) + b_H + s_H` / `K_V sqrt(C_zz) + b_V + s_V`，
本配置 K_H=K_V=5。它是固定地图／对齐／外参、声明源内噪声条件下的未校准几何代理；
正常误差包络、经验误差界和故障完整性 PL 不互换。数值资格与 Current Monitor 执行许可分开。

当前针对性开发首先澄清预测对象：`FusionAdvisoryResult.prediction_target` 为
`LOCAL_LINEARIZED_POSE_INCREMENT_FIXED_MAP`，`bounds_estimator_absolute_error=false`，
验证输出显式序列化这两个字段。瞬时固定地图信息逆矩阵描述局部线性化位姿增量，
不含 GLIO 历史漂移/地图/对齐偏差的绝对定位误差模型。同参考时刻、同map轴、同单位
只能授予配对诊断，不能授予同一误差随机变量或覆盖资格。误差审计仍保留原绝对误差、
原条件H/V和两次超界；不改数学计算、K/sigma、prior OFF、风险权重或物理执行规则。
诊断1102原请求11/14在传播前已有z误差约-.4044/-.4536 m；传播与真值运动之差为
毫米量级，不能解释该持续误差。独立seed3101已在模型/参数冻结后采集，
`log/20261010T170315Z_470`绑定干净d43fe8af；18请求/17时间合格配对/15实际执行
运动点，H/V诊断超界0/0，平均z偏差-.094457 m。未拟合、未授予绝对误差覆盖资格。
接口验证：`log/20261010T163030Z_438/export/analysis/vertical_object_corrected/diagnosis.json`
的6份完整原输入重放通过对象声明检查；原H/V保持至1e-10、原2次超界保留。
来源去重/噪声尺度4项`AdvisoryCalibration.*`通过；该结果不授予绝对误差覆盖资格。

针对性搜索核查仅调整首次发现队列的几何优先级：9→4（有限warning倍率上限3加
单位余量）；同一A*仍在发现后用原单位距离下界重开全部已发现节点，只有该阶段
可授予离散最优性。未引入第二路线、风险权重、采样、缓存资格或预算变化。
完整committed_4输入的3次冷运行共同成本41.813822→40.927293，准备＋搜索
约0.604 s；仍未证明最优，离线参考40.673760不变。预热结果单独报告；committed_8/14
的共同成本34.429614/28.011888和可行性保持，44项A*测试通过。
`log/20261010T163030Z_438/export/analysis/search_minimal/summary.json`保存重放与预算。
原EGO初始化/优化/调时/硬检查对新guide的原1.233333 m弧长前缀重放通过，路线偏离
0.005824 m<0.136603 m；保守计入旧准备时间＋本轮准备/搜索仍使用原1.5 s预算。
`search_prefix_backend_v2/summary.json`为离线实际后端曲线，未发送命令，不能称ON实飞收益。

执行取证增加按真实trajectory ID启用的`execution_evidence_ids`启动参数（空默认关闭，
逗号分隔，最多32身份，要求`capture_failure_map=true`）。既有异步writer保存既有
不可变预算内物理epoch的`execution_corridor.csv`及代数/时间/运动元数据，manifest登记；
不在规划线程复制新地图、重采、放宽检查或增加预算。`committed_N`是attempt标签，
不是trajectory ID；新`committed_trajectory_id`与corridor的trajectory_id明确真实owner。
CSV保留producer raw/inflated/observed及comparison_mask：7为实际受检voxel，1为仅raw
净空邻域；不能把邻域observed位授予执行许可，也不能把voxel受检称精确点/时刻受检。
同点比较器支持稀疏提交证据，对缺地址和未受检位保留缺失；新采集不能补造历史提交。
验证：原预算内final-check缺mask反例保留；预算内原epoch在生产map更新后逐行导出一致，
原full-map_available仍false。相关4项原生执行/取证检查、22项map工具与48项启动接口
检查通过。按完整样条匹配，历史ID10/13/24分别对应committed_13/23/33，
final gen264/352/595缺mask；ID11没有匹配提交。此前按attempt文件名描述ID的说明
已更正。更早同曲线地图的同拒绝点raw距离减少可复核，但不是最终提交代，不能由此
确定配准/观测/采样中唯一根因。新OFF保存指定4个真实owner提交走廊，均与实际wire
完全一致；该窗口4身份未拒绝且保证失效日志0，不能补历史mask或恢复连续保证。

针对性A/B/C工作完成，保留模型资格与执行能力边界不通过。图文报告及机器结论：
`log/20261010T163030Z_438/export/analysis/targeted_report_v2/REPORT.md`和`summary.json`。
冻结hash为870a39db2fc2c7f637c452258b0ddd0d11e287b5fccfe7832c728f5c2ecc389f，
数学引擎/噪声/K/风险权重未变。新ON预算内成本仍高于原OFF及充足离散参考；
后端证据为离线实际样条，无新ON命令或任务收益，配对ON收益实验没有启动。

完整新冻结使用 v9，保留原 state/GNSS/cloud/map/reference/receipt；合格正向短区间
由真实 IMU 传播均值、F、Q 和交叉状态，原后验 envelope `2(FPFᵀ+Q)` 只作对照，
不作为新的独立观测先验。旧记录不重编码授予时间资格，不刷新 stamp 或扩大配对门槛。

本轮 GNSS 表示重复反例：16 条观测复制一次成为32条，名义 FIM trace 翻倍。
最早错误在 Advisory 的可见几何集合逐行累加。修复后，同 epoch 的
`(constellation,sat_id)` 完全相同记录只授予一次几何/故障假设；同身份不同观测
拒绝 GNSS 为 `conflicting_duplicate_observation`，不得赋予新独立信息。
原始冻结 payload、监测身份、可见性数组不改写；masked/receiver/Fusion 共用此权威集合，
receiver/visibility cache 身份覆盖用于重复一致性判断的字段。
该修复不改 GLIO、Current Monitor、风险权重或执行政策。
已采集 seed2101 保留修复前冻结模型身份；修复后独立留出改用未查看的 seed2102，
追加运行只为此确定的软件反例，不拟合经验参数。

固定路线离线评估器 `failure_map_replay --route-check/--route-cost` 与生产搜索
共用保存的两套时间：物理检查使用 `planning_time_s`，完整 payload 身份使用
`risk_reference_time_s`；原 `risk_valid_until_s` 也必须提供且有效。v9 正向传播后
两时刻不同属于合法输入，不能比较为同一时刻或修改 stamp。真实 committed_4 的
原工具拒绝为红，修复后同输入可评估；错 reference、缺 reference、错 map 仍拒绝。
此修复仅涉及离线工具，不授予执行许可，不改变现场规划预算或模型版本。

本轮可复核结果见[图文报告及复现命令](../../log/20261010T145917Z_405/export/analysis/closure_report_v1/REPORT.md)。
原9条保证失效逐身份核对为10/11/13/24，首次拒绝到新命令约2.752/1.408/1.717/1.518 s；
保留命令有实测运动，不能称受检悬停，不推断碰撞，不追认区间保证。原近期净空／制动
动力学能力边界仍保留；现有6项执行保护测试通过，不授予新的连续执行保证。
完整新入口同输入独立矩阵/消元/退化271项通过，生产GNSS重复反例定向修复及15项依赖
测试通过。原地图proxy的LiDAR组内／跨帧依赖、地图误差与对齐不确定性未校准。
2次OFF诊断＋原版留出＋修复后独立留出取得68个合格同刻truth配对、65个有效条件模型
配对；诊断1102有2个V超界，最大error/model约1.019。最终2102未超界不能授予95%或
经验米数/故障PL资格，也未用于调参。map-up与估计ENU-up存在变化，H/V仍明确为map轴。
第0分岔完整冻结697离散点，完整左右参考均有未知，未授予全支路排名。同输入OFF/ON
预算搜索与15 s离线参考使用原物理政策/风险权重；统一评估总成本约40.696/41.814/40.674，
在线ON为预算内incumbent，不能宣称最优或更安全。ID4局部guide→优化/时间→原wire→261条
实际命令身份一致，最大独立deBoor误差5.4e-8 m；另以原2601条仿真真值独立复核实际曲线，
只作离线评价，不补入地图或许可。完整历史长guide及反事实ON执行未证明。
模型用途及原执行保证边界未满足，因此实际配对ON收益实验未启动，无实际任务收益结论；
无经验参数推广、物理门槛放宽、在线预算或风险权重修改。

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
