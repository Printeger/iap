# 完整性感知 EGO 当前流程（v1.1 轻量闭环）

最小模型推进：已有代码／记录支持原仿真的严格静止启动与理想IMU／初始fix；
4766个启动真值样本不变，仅验证适用条件，不进入预测。
`stationary_simulator_bootstrap_inputs` 实现24维初始物理输入：只接受初始化阶段
的完整nominal值与独立静止／理想传感器／ENU标定条件，保留速度、偏置、重力
和ECEF对齐误差均值；一份全局初始datum不逐区域对齐。零条件协方差来自
确定的物理初始状态，非后验；不把后续创新或已消元来源物理律补零。
4项provider检查加原20项相对／score检查通过，两个零绝对容差的测试算术失败
保留。原native接口／预算不改，尚未接入IAP或授予完整预测资格。此条件下
bootstrap可由物理推导取得，下一关键provider收敛为LiDAR有符号表示／匹配
误差及其共享来源关系；已压缩的旧联合律仍不可恢复，M2与M3–M5均未通过。

观察器调查已收口：`log/20261012T054026Z_869` 的7包完整、零预算超限，
完整回调最高4.872503ms，冻结姿态逐元素一致；真实307维消元响应通过，
10052条缓存PR的原始sigma与生成端完全一致。此前5.156ms失败仍保留。
这些结果只资格化当前记录的接口和响应，不是提前预测、排序或选路通过。
不再扩展采集／缓存／格式／通用诊断。端到端最小候选及每项物理输入的
来源、假设、可取得／已丢失／不可识别关系见
[local_increment_model_feasibility.md](local_increment_model_feasibility.md)。
当前主要阻碍是物理模型：缺初始化／冻结来源／起点与创新的联合物理律，
以及前瞻LiDAR表示／匹配偏差均值依据；原生H不能补出它们。有限score
传递代数可实现，但现有输入尚不能实例化完整1秒预测。最小设计决策为
独立物理标定provider或自bootstrap起的物理推导与有限矩传递，均非新增
历史轨迹系统；不把缺失填零或当作共同刚体基准全部抵消。v1.2评价契约、
旧失败、原门槛与7301–7303未查看划分保持，M2模型、M3–M5均未完成。

`log/20261012T051028Z_320` 来源227的冻结姿态逐元素一致；277→278原生
消元的307维响应重建通过（H相对差4.4e-13、对角归一化差3.2e-12，梯度
9.5e-16）。因此预消元接口在此反例可提供响应结构，不需继续扩展依赖。
全部6必要包写入，但完整回调2次超5ms（最大5.156ms），仍保留为预算失败。
每帧第一次更新的滚动副本不是前帧最终解，删除这项冗余复制，仅保留ordinal1。
主缺口收敛为冻结来源／窗口起点／物理创新RHS的联合矩及跨消元传递，当前
物理律仍缺失，原排序失败、契约和通过线不变；不把响应通过当模型通过。
最小数学候选落实为离线 Schur score 载体：保留冻结来源／起点别名及有限
边界RHS的完整物理均值、协方差和交叉矩，经原生H响应消元；不保存完整历史。
`condense_physical_score_law` 缺任一物理律即失败，原H仅供响应；不取H逆为
噪声或补独立边缘。上限沿用采集1536维，别名72维，仅离线可实现性原型，
不接入生产、不扩在线预算。四项受控反馈检查及原契约回归20 PASS；真实
物理provider尚未取得，数学代数通过不授予主要排序或独立验收资格。
下一最小输入接入保留 PR 原始 epoch／卫星／频率及 receiver-reported sigma，
在 fallback、canopy floor 与postfit之前绑定因子。观察器导出不可变身份，
重复线性化不能当新噪声。reported sigma 不代表总偏差律、物理联合矩或
历史来源关系；旧已丢失记录不回填，原估计器量测、权重和算法不变。

`log/20261012T045503Z_714` 同帧身份已修复、缓存响应绑定正确，但274后置包
仍因写入队列满拒绝，保留为接口失败。v5在消元已消费前帧后复用**同一个**
滚动槽作为单包待写空间，保持两包队列／一个额外槽／5ms原上限；溢出仍失败。
不扩容或重复采集赌通过。冻结姿态检验必须使用实际 GTSAM retract 与原生
float32 转换，不能把分析器的 Expmap 近似差异归罪于物理来源模型。
原生等价性增加长连接／部分 clique 消元反例，逐更新核对全部保留状态，
检验 const getDelta 的惰性计算是否改变结果；该检查必须先于下一次取证。
85步／原5秒／原additional update 对照已通过，全部保留状态逐项一致至
1e-12（观察器套件5 PASS）；这仍不授予物理联合模型资格。

`log/20261012T044345Z_309` 保留为采集选择失败：原生同帧两次更新，273重复
输出造成274队列拒绝并误报前帧缺失；全部请求、旧失败均留存。v4记录帧内
原生更新序号，前帧取最后一次，边界只输出一次；后置快照改在**实际消元后**。
实际流程在 `lag-0.1` 移除帧对象，且早于 `update_frames`：地图来源冻结取
首次固定使用的**前一帧最终更新**。同号原生变量在消元前仍可变化，二者必须
作为不同版本的误差别名。一个滚动包止于首次固定使用，选中快照另带前帧
来源状态／更新序号；原生 float32 核对证明首次目标逐元素一致，而消元时
活动变量已偏离约0.2mm。不用消元当帧状态冒充被冻结来源。
最多100帧／每帧2次回调，保持5ms及存储上限。来源224在273固定、275消元，
后续压缩因子265维，不能预设55维载体足够；新取证仅核验这一真实响应关系。

原生响应定向采集 v3 仅在诊断 opt-in 时记录首个非零**关键帧**来源的固定化
边界：前一更新、首次固定使用、下一更新、真实消元与+10更新，重合合并。
普通连续帧因子不能触发；一个滚动包、最多100更新、最多32输出，原5ms
复制预算外还记录完整回调时间。缺失或预算失败不授予资格。固定姿态与当前
消元更新未必同轮，必须从原生所有者验证；不猜测或用后验补齐关系。
原1秒主评价、原门槛、旧失败和未查看独立种子均保留；本采集仅回答新边界
问题。物理噪声/RHS来源在压缩因子内仍不能恢复，最小联合载体尚未成立。
该诊断复用已有 GNSS 注入噪声 CSV 开关，保留 PR/Doppler 白噪声及偏差来源，
无需额外 GPU 残差轨迹；不新增随机抽样，不把 solver sigma 当物理误差。

第三轮 `log/20261012T034758Z_695` 9/9复制完整、2.16–2.74ms、无写入失败，
但4/9的目标响应与原生缓存 Gaussian 不一致（最大元素差31042.25）；原结果
保留为来源绑定失败，不能把复制完整授予模型资格。根因是 postopt quality
重线性化同一 nonlinear GPU factor，覆盖最新 reduction，而 iSAM 仍缓存旧
Gaussian。现将目标／来源响应绑定至每次返回的 Gaussian，clone 保留；worker
核对同一缓存块，差异明确失败。原 Hessian 系数、原求解及5秒窗口不改。
“缓存A→qualityB→仍读A”的原生GPU反馈检查及原生扰动检查通过；联合物理误差
律与精确窗口所有者仍未资格化，不使用后验或损失核补齐，M2继续。

第二轮补采 `log/20261012T033007Z_640` 的10请求全部保留：活动图实际约
3297–3517因子，8项复制期限失败、2项压缩因子维度失败。旧2048上限确实也
小于真实活动图，不把空槽说成唯一原因。观察器 v2 改为限量深复制缓存 CPU
Gaussian factor，worker 计算法方程并按上三角 gzip 编码；支持原生压缩因子，
1536维／行上限，8MiB投影数值、16MiB缓存副本、原5ms逐操作检查保持有界。
3301因子／300维压缩因子的原生反馈测试已通过原复制期限，所有5秒估计与
异常不fallback检查仍通过。不是模型通过，不增加历史漂移模型或修改估计器。

原生首轮补采 `log/20261012T031930Z_593` 全部9请求为容量失败，原失败保留，
不授予模型资格。观察器容量先前错误地包含消元后的空 factor slot；现通过
原生 VariableIndex 枚举活动关系，限定4096活动因子／32768索引关系，原8MiB
数值与5ms逐操作检查不变，原估计器槽位回收政策不改。VGICP 保留由2048个
活 weak-reference 限量，不再由空槽停止；显式记录槽位／活动数量和缺失。
此为取证实现修复，不改变区域通过标准、原失败、预算或任何生产预测。

当前 Goal 已按用户的窄范围依赖授权恢复 active，仍在 M2。gtsam_points 原工作区
干净，批准补丁的五个文件基线 hash 全部一致；独立依赖提交
`0a3c1b16a7e256721525f1fcb09588e033f85e63` 增加更新后、消元前的可选只读回调。
IAP GNSS 在原 smoother update 前接入，默认关闭；预声明最多31帧另加首次 x0
消元，限时／限量复制原 CPU 响应结构，worker gzip 保存。IAP VGICP 仅在 opt-in
时保留原生 reduction 的目标／来源块，修复 unary 路径丢弃目标响应的取证缺口，
不改变原匹配、求解、5秒 lag、物理检查、预算、PL门槛或风险权重。
接口、边界和未取得的物理关系见
[native_response_observer_contract.md](native_response_observer_contract.md)。
`log/20261012T024925Z_202` 保留补丁、版本、构建和接口核验：原5秒窗口85步
关闭回调与原基线逐字节一致，开启及异常每步估计一致，异常不进入 fallback。
接口通过不等于联合误差模型通过；来源／短窗口起止状态／创新的物理联合律及
原生扰动一致性仍待新记录验证。旧失败、v1.2评价对象和原通过标准保持；
独立排序、模型冻结、实际选路和完整OFF/ON收益尚未完成。

当前持续开发目标为“可信局部风险指标＋实际风险选路收益”，基线 `f90d42c9`。
工作入口及预定完成／数据划分规则见
[credible_local_risk_goal.md](../dev_predictor/credible_local_risk_goal.md)。
M1 已从原始数据重算三次反序（区域一致率 1/3、2/3、2/3），复现 owner
`log/20261011T064140Z_343`。固定 OFF5101/frame401 的同一地图，对 OFF5102/5103
共16个完整跨运行扫描求解后，水平区域一致率仍均为1/3；原同输入 anchor 的模型
H 与实际解均在1e-9内重建。该对照排除“地图变化／当前扫描混入是唯一解释”，
不把两个已查看运行重新授予独立验证身份。真实匹配覆盖、共同状态及原时刻保留。
`log/20261011T064237Z_231/export/analysis/heldout_fixed_map_v4` 保留来源、关联与诊断
分解；族均值主导误差，不能用统一零均值 sigma 或族数比例冒充修复。
原 v9 codec 丢失活动障碍来源的原生反例红→绿；v10 保存原 current/active metadata、
occupied contribution 与来源健康字段，旧记录仍明确 unavailable。仅新增取证能力，
不增加风险地图或执行 authority；replay_metadata 不续龄。来源补采已完成，M2
可见支持已做最小修复，噪声／地图偏差尚未资格化或冻结，M3–M5 未完成。
下一步核验地图／关联偏差与评价对象。历史报告是保留证据，不代表本目标完成。

来源补采 OFF6101 `log/20261011T070312Z_015` 已完成90s，干净 `f704b8e1`、GPU READY、
11项 Release 安装与原 NAV hash闭合，8份完整 v10 输入和1次启动缺失保留。记录证明
旧 frame401 评价对应 current400、cloud=.450／source=.544；新8份的评价帧均不在
raw current/active 来源中。旧报告“当前扫描入图”不是已证实事实：原始点纳入与
优化历史影响注册位姿须分别审查；旧缺失来源不补造。跨运行同一地图仍反序，故
当前证据也不支持把地图自相关作为唯一根因。来源 ICP RMSE约.22–.30m、既有 LiDAR
PL约2.5–5.9m，语义不同，不能直接套成残差 sigma 或新增独立信息。下一步核验
PCA表面、首回波可见支持与地图位姿不确定性。replay_metadata CSV只提供原几何，
不作再配准、续龄或执行授权；M2 模型修复及 M3–M5 仍未完成。

M2 覆盖修复：原生5ms反例显示完全遮挡的第二面墙使支持组81→162并增加信息。
makeFrozenPredictor 现在在 PCA 信息累加前绑定原传感器视场／量程和同一 raw
占用体素的首回波遮挡；不使用膨胀或虚拟天花板，未知体素不成为 free 证据，
缺失传感器几何明确拒绝 LiDAR，不由 legacy 点密度恢复。保存的参考姿态／外参
决定局部视场，姿态仍按原共同状态边缘化，原噪声、分组、权重与预算不变。
surface_model_probe 固定关联只改 PCA 截距的对照多数误差增大，因此未采用；
24份旧诊断中首回波后仅约30–60%视场内表面可见，同尺度H约为旧值1.3–2倍。
这修复可见支持接口，不授予局部误差排序资格；地图偏差与零均值噪声对象的
对应、独立空间排序和完整执行收益仍须继续验证。
原生反例与缺失／视场／legacy 检查已绿，25项LiDAR、99项模块、7项接口／pending、
9项工具通过。`log/20261011T074219Z_674` 同输入原生与独立 NumPy 信息最大差约
5e−12、H差2e−16；64点诊断query_total=.142s。此检查不授予现场预算或 M3 资格。
追加来源诊断 `log/20261011T075507Z_790` 保留8次初始datum真值未括定；其余 raw
来源位姿存在厘米级、约.002–.013rad误差，当前评价帧后验位置σ约2mm，且帧身份
不同，不能将其当作历史地图来源噪声。缓存身份反例显示改变传感器视场、量程、
姿态或可用性仍命中同一身份；现纳入记录的传感器／证据和 ray 格子几何，policy
该次为 `pose6_known_raw_first_hit_two_source_envelope_v3`。配置 envelope不再替代未记录
传感器输入，以保持原输入与只读重放的资格一致。完整payload SHA与数值cache
identity语义分别保存。原生红→绿，8项缓存／可见支持／只读／pending检查及9项
工具检查通过；M2偏差模型／评价对象仍在归因，M3–M5未执行。

曲面支持反例 `CurvedTrunkSupportsRetainLocalSurfaceNormals` 已红：半径1.5m覆盖
整根半径.2m圆柱后生成0个法向。支持生成现改为半径上限内最近10个不同几何点，
10来自原GLIO预处理 `k_correspondences`，不按路线／误差拟合。几何重复不占名额，
圆柱径向法向与重复输入原生检查已绿；26项LiDAR、99项模块检查通过。
模型cache policy推进为 `pose6_knn10_raw_first_hit_two_source_envelope_v4`，诊断metadata
明确导出邻域策略。相同固定raw地图的迭代配准对照中，仅将全半径邻域改为最近10点
使23/24个已查看样本水平误差降低，中位数.170→.0367m；20/24在预定10步未收敛，
全部保留，不据此授予噪声校准／排序资格。8项原生接口／pending、9项工具检查通过。
同输入新几何原生6维信息与独立NumPy最大差1.27e−11，receiver H=.74847m；
旧3区域排序仍仅1/3、2/3、1/3，原门槛／负结论不变，M3–M5未执行。

离散扫描支持反例已红→绿：已记录两条竖直射线均未穿过小墙面，旧模型仍计入
16组信息；加入中央第三条射线后确有命中。frozen binding现使用原
`spherical_first_hit_v1`网格的最近射线，并要求其首个known raw体素为目标体素。
这仅是最近射线可证实支持子集，不宣称遍历所有可能穿过体素边角的射线。
LocalEvidence布局缺失／非支持模型明确拒绝LiDAR且不走legacy；无local evidence
的历史／合成连续几何诊断不能进入M3扫描支持资格。forecast_sensor原值导出，
cache policy为 `pose6_knn10_sampled_raw_first_hit_two_source_envelope_v5`。
9项接口／pending（含缺失／非支持布局）、9项工具检查绿。原frame401同输入
原生与独立NumPy信息最大差1.36e−11，H=.869003m、prior OFF，304支持组。
64点诊断query_total=.116s、prepare=.155s；不是现场搜索预算资格。
来源一致性对照 `log/20261011T084121Z_261` 保留初版跨不同表面的较大差值；
共同原10点邻域和原来源归属控制后，仅22–87个重叠patch合格，均值差中位
约.009–.022m、p95 .023–.055m，无法校准共同地图偏差。原主要指标／门槛未改，
M3–M5仍未执行，Goal active。

来源端点取证补齐：v11 codec 在保留 v10 来源布局的尾部增加 current/active
生产者 `hits_lidar`，保留原点索引、LiDAR坐标和同一来源 `T_map_lidar`；端点在
来源更新时复制一次、冻结时共享不可变指针。生产者发送前按飞行体素保留首个
端点，坐标未改成中心；不能称为原始全部射线或按点数授予独立噪声资格。
`replay_metadata` 新增 `source_endpoints.csv`，明确导出
角色／帧／点索引、原坐标和注册坐标；旧 v1–v10 端点标记 unavailable，不由
体素中心恢复。此字段仅用于诊断，不改变 raw占用、预测支持、缓存或执行授权。
现有3份同源端点／体素中心对照显示最近10点 patch偏离量中位均约.09–.11m，
原端点也不能保证单平面；体素化不是唯一解释。取证owner
`log/20261011T094835Z_462`；来源datum对齐旧探针未收敛／缺共同覆盖，条件响应
旧探针区域排序未通过，均不授予噪声或地图偏差资格。原主要评价与门槛保持，
M2仍进行，M3–M5未完成。
端点保留原生反例已红→绿；23项来源窗口、26项占用快照、10项接口／pending、
10项validation工具和14项报告工具检查通过。旧v9/v10真实输入可读且端点明确
缺失；同一frame401的LiDAR／融合信息及协方差逐值保持，H=.869003m。
此修复不授予排序、性能或执行收益资格。

当前M2收敛门禁：固定地图与跨生产者扫描已可比较，不再以数据不可比较
概括持续排序失败。全原射线覆盖、原端点与体素中心、native算子法向投影
对照分别排除支持遗漏、量化、投影作为唯一解释；不继续重复这些几何取证。
`log/20261011T103953Z_882/export/analysis/convergence_v1` 保留原主要指标／门槛：
新来源完整固定地图上的六维来源共同位姿均值／散布追加候选仍为0、0、0，
原失败地图1/3、2/3、1/3结果不覆盖。原区域代表误差向量共同均值平方占比
93%–99.6%，仅为解释性矩分解，不能宣称地图来源位姿唯一因果。无真值的
原支持族残差均值候选24请求各缺6–69族双运行覆盖，0/24 qualified；缺失不补零。
剩余单一缺口是有符号残差均值及来源不确定性，最小模型需将损失权重、随机
噪声与持续均值分别表达，保留完整姿态／钟差响应和原GridMap来源身份。
已有v11端点与native源点／协方差的有限均值实验已完成，结果及下一项模型
接口见下文；original inliers不等于全部source slots，未记录源点不等于新地图
关联缺失。上述候选未采用，生产V5及所有执行规则保持；
独立排序、冻结、规划贯通与完整OFF/ON仍未获得资格，Goal active。

均值模型取证的最小接口补齐：实际native kernel原来仅导出原目标的inlier
几何，旧frame401共有6044个source slots、只存1775个匹配源点；不能将另4269个
说成新固定地图的关联失败。现有有界capture在同一CUDA stream上先读取原
source均值／Gaussian covariance，再按原样计算匹配与reduction；共用原pinned
buffer及128／16384上限，无新增同步。artifact `iap_gpu_match_residual_v2` 分别
保存原 `samples` 和 `source_samples`，后者含原source index及源坐标；匹配为0
仍保留原源点。`source_geometry_available`、slot闭合及ledger独立计数明确资格；
stride>1仍只是有界子集，旧v1不重构未记录源点。源covariance是原VGICP损失
kernel，qualified_noise/motion仍false，不能直接当作物理噪声或来源位姿covariance。
源记录缺失／CUDA取证失败不复用旧数据，原匹配残差、cost及Hessian保留；
原生0匹配反例已红→绿，同步／异步、单／双节点、surface validation、稀疏／
dense及失败恢复对照通过，writer压力与6项audit（含旧真实协方差反例）通过。
检查owner `log/20261011T105641Z_447`，红反例原日志绑定前一owner，不移动证据。
这仅修复模型选择输入，不授予残差均值、原排序、现场预算或执行收益资格。

均值候选有限筛查已完成，`log/20261011T105641Z_447/export/analysis/convergence_v2`：
固定跨生产者地图上8请求只改扫描集合，完整源点与原匹配子集均区域一致率0/3；
不再把匹配点筛选作为唯一解释。另一来源完整固定地图上原24请求全部保留，
真实端点均值／形状散布候选仍1/3、2/3、1/3，补入此前六维来源均值／散布后
0/3、1/3、1/3；原权重、完整姿态／钟差、评价及门槛未改，两候选均拒绝采用。
主要障碍仍是预测条件方差不能表达实际持续均值，数据已可比较。最小改造需
让预期观测生成／原关联的有符号残差矩进入原完整状态响应；v11 wire首点与
均匀patch池不足以代表实际0.5m random-grid、target10000观测分布。下一项只
冻结一份地图来源的wire前返回测度、实际native点及原坐标／索引／时刻／预处理
身份，随必要原输入保存，不再采多seed全帧大日志。均值模型尚未资格化；原V5
只保留条件几何资格，M2继续，M3–M5未进入，失败与未资格假设均保留。

来源采样接口 v12：`capture_source_observations` 默认 false，仅 simulation full_stack
可显式启用。原 RegisteredLidarFrame／来源窗口／FrozenInput 保存完整 wire 前 adapter
返回点（LiDAR float64）与同一来源实际 CPU→GPU 匹配均值（IMU float32）、原索引、
T_lidar_imu 与预处理身份；坐标转换仍使用该来源 T_map_lidar。点数上限分别65536／16384，
越界或无效只将诊断证据置 unavailable，不改变物理准入。当前／活动来源共享不可变
观测，位姿更新保留同一帧／时刻，禁止从上一帧借用。codec 保留 v1–v11 读取，历史
采样证据 unavailable；replay_metadata 导出 source_sampling_points.csv，不由中心或
wire 端点重构原点。记录真实已抽样 native 点，但不记录随机生成器状态，不能声称
从 raw 重现原随机抽样。Gaussian kernel 未被命名为物理噪声。
此接口只补最小模型选择证据，不改生产V5、主要评价或执行授权。定向原生
codec 丢失反例已红→绿，7项只读／并发冻结、24项来源窗口、26项占用快照、
48项入口和10项validation检查通过；原 ROS 来源更新与实际 CPU→GPU 均值闭合
通过，额外观测存在／缺失的原预测值和同版本cache身份相同。旧v11真实输入
采样证据明确缺失，8项原矩阵逐值差0。六包Release完整重建／安装；检查owner
`log/20261011T123441Z_520`。接口通过不等于均值模型或排序通过，M2继续。
诊断 recorder 的 `--initial-delay` 与 driver 的 `--record-count`／`--record-initial-delay`
允许预先声明只保存一份必要输入，默认保留原10s节奏；所有请求失败账本继续保存。
来源测度的单次现场 `log/20261011T131912Z_451` 已完成，GPU READY／原NAV／干净
`dc1d1a4f`／安装身份与必要健康闭合。current347、active0／254／293均保存完整
wire前与native源点，原索引／float32／变换闭合，坐标差0；唯一原payload 2.89MB，
逐字节相同重放input.bin与其共享inode。全部24原请求在同一新固定地图上，wire
矩候选区域1/3、1/3、1/3；仅换native测度后2/3、2/3、1/3，仍未达到原.7门槛；
全局Spearman的改善不能替代区域规则，不进入M3。sampling测度有影响但非充分
修复，下一处缺口为未来观测／实际关联算子的有符号残差响应及来源偏差，禁止
继续按区域修补patch、调参数或增加同目的采集。
旧driver仍逐规划保存map／内部输入，本次整run约1.05GB，已有证据不删。后续
`--omit-planner-map-dumps` 仅允许OFF来源观测诊断，传递原capture_failure_map=false，
省去重复mask与内部规划输入；标量失败／请求分母、原物理检查和指定单份service
输入保持。M4／完整OFF/ON不得用此取证简化替代原完整证据要求。

M2收敛 v3／v4：四个同地图控制分支逐值复现通过，native测度、理想来源姿态及
静态点池均未通过原区域门禁，不再重复已排除的FOV／PCA／体素／inlier核查。
实际sensor返回首个占据体素入口，非中心；复用已安装renderer与原生random-grid
的单候选也未通过。新生成器先前误用传播后参考时刻，失败分支保留；仅改实际
source时刻后为2/3、1/3、1/3，Spearman .690／.048／.476。评价侧理想查询
姿态仍2/3、1/3、2/3，不能据此给生产加入真值。全部24原请求／原三个区域、
主要评价／基线／权重／自由姿态与clock／prior OFF及.7/.5门槛保持。

最小剩余模型是已登记来源与真实物理表面之间的联合有符号残差响应：e=P b−δ，
需要观测、参考修正及交叉矩，或等价紧凑共享latent；分别保留均值和物理协方差。
当前x348联合后验不覆盖地图source0／254／293，不能拿当前后验、独立marginal、
loss sigma或VGICP kernel替代。原fixed-lag外keyframe为fixed-pose unary factor，
简单增加当前marginal字段不能恢复其随机性。下一步限于活动source/reference的
紧凑因果模型及物理占据条件关系，不建立完整历史漂移，不增加第二风险地图；
数据缺资格记unavailable，不补零或删除区域。合成响应契约已验证共享状态抵消、
交叉项必要、偏差／散布分别进入、重复不虚增与位置退化仍失败；不是物理校准。

`log/20261011T141528Z_749/export/analysis/convergence_v4` 保存三个新增分支、
原失败、图、机器状态及复现入口。新owner `20261011T143735Z_885` 选定值逐值
重放一致，容差1e−12；直接读取固定MC gzip数组，不重抽2线程随机序列。上一
四分支报告位于 `log/20261011T123441Z_520/export/analysis/convergence_v3`，
新owner `20261011T135343Z_122` 已复现。以上报告完成≠开发完成；生产V5不改，
M2继续，M3–M5未进入。无新同目的现场；未来取证只回答改变模型选择的缺口。

M2最小模型可实现性边界：原主要评价在`heldout_fixed_map.solve`中为
`e=P b−δ_reference`，δ由同一参考估计与独立真值的绝对差定义。因此它是局部
修正后的绝对误差，不是两时刻相对定位增量。若`b=Aδ_reference+ε`，参考误差
可在可观测方向抵消，不必建立完整历史漂移；但冻结来源的物理基准误差仍通过
`e=P_L J_source d_source+Pε`进入。原24个已保存的fixed-association地图x+0.05m
敏感性结果中，协方差不变，水平修正改变0.0499907–0.0500000m。此证据说明
评价对象包含来源绝对基准，不能宣称其为当前排序反序的唯一／主要因果，也
不能静默将主要指标改成相对增量。用户已明确本阶段排除共同绝对基准偏差，
局部形变、相对错位及影响运动增量的偏差仍纳入；新相对增量契约单独版本化，
旧评价、所有原失败区域及.7/.5门槛完整保留。契约改变不授予通过，Goal active，
未因交付可实现性判断而进入M3；短窗口／增量／坐标时间／资格须先预声明再实施。

地图完整来源是active0／254／293加current347；current_hit同样产生raw占用。
仅三个旧来源的理想场景反例曾得到全区域通过，但它遗漏current347，不能授予
资格。完整来源×查询姿态四个固定分支的区域一致率依次为2/3、1/3、1/3；
1/3、2/3、1/3；1/3、1/3、2/3；3/3、2/3、3/3，均失败。最后一个分支仅换
已保存GNSS残差仍为3/3、2/3、3/3；不重复同方案等待通过。原生筛选与完整源
对照已排除其作为唯一解释；未来观测算子仍必须说明实际VGICP目标和准入，
不能将全生成点与原后验匹配子集视作同一算子。上述旧失败保留。

可增加字段取得与已丢失的信息明确分开：在实际frame348，current347仍在5s
lag内，可前瞻导出二者真实联合状态；原GPU目标bucket／均值／kernel及准入
也可在owner stream冻结导出，但kernel不代表物理噪声。active0／254／293分别
已旧35.106／9.699／5.705s，状态及来源交叉条件已被smoother边缘化；fallback
同样删除旧values／factors／factor_map，当前后验或增加导出keys无法恢复。
旧24请求未保存的全部native源点与原目标快照也无法从残余inliers重构。

保存活动来源的紧凑消元条件在数学上可避免完整历史，但现有callback中
pre-update在新因子更新前，finish／marginalized均在消元后；准确保留需要
IAP拥有的消元前模型接口，不是新增消息字段即可获得。即便取得求解器联合
Gaussian，它仍不能替代有符号物理均值、固定姿态unary目标的因果噪声关系或
不可见表面的条件分布。最小响应`D=[P_pose,−I6]`可实现，当前缺合格的
`μ_joint,Σ_joint`，候选资格为`UNAVAILABLE_PHYSICAL_JOINT_LAW`；禁止用当前
后验、独立marginals、loss sigma、kernel、真值或缺失补零填充。生产V5仍仅有
条件几何资格。可实现性审计及固定失败引用保存在
`log/20261011T141528Z_749/export/analysis/minimum_model_feasibility_v1/audit.json`，
完整来源四分支逐值重放已由`log/20261011T153303Z_160`完成，不重新随机采样。

用户明确的局部相对增量评价已独立预声明为
[`local_increment_advisory_contract.md`](local_increment_advisory_contract.md) v1：
固定1.000s、原估计与独立真值各自`T(t)⁻¹T(t+1)`、起始IMU局部XY增量误差，
共同左SE(3)基准严格抵消，局部形变保留；禁止逐区域／窗口对齐或减均值。
旧绝对评价和失败保持。下一项先实现该评价与固定语义反例，再仅一次以原24
已查看请求区分旧绝对对象与局部反序；原V5分数仅作几何baseline，不冒充
相对增量物理协方差／PL。独立seed7301／7302／7303保留未查看，模型及资格
冻结前不采集；原数值排序门槛、完整区域与全部原执行规则保持。
相对增量评价实现位于`scripts/dev_predictor/local_increment_validation.py`，仅使用
原密集估计／truth pose和固定query身份；truth仅在评价端，既不拟合对齐也不
改变预测。GLIO两帧缺口及truth原50ms门禁明确失败，全部缺失请求保留分母。
共同旋转／平移消除、局部形变保留、起始姿态影响和静止窗口的定向检查通过
后，才执行一次已查看数据的baseline区分实验；其结果不授予相对物理协方差。
首次读取`20261011T160620Z_524`在计算误差前因旧GLIO一个冲突重复stamp失败。
v1.1明确所有使用歧义stamp的端点／插值窗口失败，记录原歧义，不选择其值；
未使用该stamp的窗口仍可评价。窗口与时间／排序门槛均不改。
固定1s比较`20261011T160805Z_137`仍为区域2/3、0/3、1/3，Spearman
.393／−.714／−.548；23/24原有效query可评价，缺两帧的frame803和三个原
service缺失继续保留。排除“仅改评价对象即足够”假设，不重复baseline。
最小新候选仅将两端有符号观测响应联合传入相对变换；完整paired draws保留
交叉关系，公共偏差抵消、差异偏差保留；不同端独立抽样使用不同seed。该
响应代数不填补物理来源或GNSS噪声关系，也不授予相对PL资格；物理关系未
解决前不进入独立验收。无需为此恢复旧source的绝对历史漂移。
两端已登记场景的有符号LiDAR响应component在`20261011T161653Z_300`仍为
2/3、1/3、1/3，Spearman 0／−.452／−.119，拒绝作为充分修复；它未将缺失
GNSS或来源噪声补为协方差，zero-GNSS-residual只是明确的反事实component。
旧同epoch发布插值与原生参考均值位置差中位2.086mm／最大5.588mm、姿态差
中位5.141mrad，与新局部误差信号同量级；旧future原生状态未存，不能恢复。
v1.2预声明一次seed7400／105s开发取证，18input／9个1s pair、首延迟10s、
pair-start间隔10s；原场景／NAV／任务／OFF及全部物理质量／预算／权重保持。
`--capture-native-increments`复用既有IntegrityReport原postopt／IMU，只保存
pair-start前0.5s及后2.3s原报告gzip，payload／失败请求全部保留。原recorder
默认不变；paired模式按首次issue绝对节拍，非等待服务完成后累计漂移。
`native_pose_at`／`native_endpoint`只重放原生均值至固定endpoint，不用covariance
补物理误差；原100ms／IMU20ms覆盖门禁保持，冲突身份、缺失或原质量不合格
明确失败，无发布插值fallback。`--omit-planner-map-dumps`配合此OFF诊断时仅
保留clock／来源／运动质量／标量执行事件，完整pose CSV及selected原生报告
由recorder唯一保存；compact scope不可作完整任务证据。独立seed仍未采集。
验证：原生选择／时间输运6项及局部增量11项定向测试通过；原recorder／backend
10项接口回归通过。仅Python与契约改变，不改变已核验的Release二进制。
7400原生补录首次读取暴露decoder矩阵布局错误：IntegrityReport的Eigen::data()
按列主序发布／消费，codec describe按行主序导出；只修正离线报告decoder，
不变更坐标／评价对象。新增非零平移和旋转的真实布局反例，7项原生测试通过；
首次坐标拒绝在任何真值误差／排序计算前发生，保留读取失败，不重复采集。
7400开发补录18请求中2个启动期epoch不可用；9个窗口中7个原生endpoint合格，
另1个原生endpoint覆盖缺失。三个原区域内只有3个合格窗口，不追加同配置采集。
`native_pair_response_discriminator_scope_v2`保留全部请求与区域：几何score区域
一致率0、Spearman−1；实际成对源点／原GNSS residual响应区域一致率1、Spearman1，
但3<6故两者均FAIL。未来端观测只用于诊断。早期v1将区域外窗口混入总体rho，
保留并明确其排序口径无资格；v2没有扩展区域或更改门槛。原生／发布增量差
约0.487–2.898mm，不能把发布表示视为充分根因。
最小模型缺口为有物理依据的短窗口联合误差响应（native时序增益、活动来源相对
形变与重用观测谱系），不是进一步PCA／FOV／半径修补。IAP内的有界仿射响应
参考`transport_retained_error_law`只消费完整joint均值／协方差和显式innovation
交叉块；无物理law即UNAVAILABLE，不代入posterior／loss或独立边缘；保留活动
source及起点，递推当前状态，不存完整历史。该代数可实现但物理输入尚未取得，
55维只是此固定开发对象的参考状态上界，未授予在线预算或生产模型资格。
有界响应5项、原生接口7项及增量11项共23项定向测试通过；共同datum取消、
相对bias保留、重用观测交叉关系、与完整潜变量递推一致及缺失／预算拒绝均受检。
实际更新后／消元前观察接口位于只读gtsam_points；5文件候选patch及隔离原生测试
保存在7400 owner，依赖未改。测试证明旧／新keys共存、原lag与估计保持，以及
observer异常不触发原fallback；需要用户明确授权此窄范围依赖例外才可应用。
接口只暴露solver响应结构，不把其Gaussian赋予物理误差资格。M2未通过，
M3–M5仍未进入，报告／接口证明完成不等于Goal完成。

持续诊断存储收紧：GPU大样本仍显式启用，现由原后台worker直接写gzip level 1
无损流，声明`gpu_match_residuals.jsonl.gz`及raw_compression；不保留展开副本，
原逐请求账本、失败／drop分母与packet字段保持。audit及实际来源重放读取声明
编码，缺失／截断必须失败；历史plain文件继续可读。writer同一2000请求／dense
压力反例红→绿，安装Release检查通过；Python历史／gzip相同审计及缺失／截断
反例通过。8份实际选定native packet共39.33MB，压缩11.62MB（29.5%），逐字节
解压相同；这是离线编码检查，不授予现场队列或实时预算资格。
只常规保留唯一原输入、必要选定观察、完整请求失败账本、配置／hash与前后结果；
record.json不能替代input.bin。用户清理已使部分旧原输入和OFF6103全帧raw缺失，
8份新原输入已按原SHA从重放副本恢复，提前提取的8份完整扫描仍可复算；原全帧
复算资格明确失去，不改写旧清单或宣称数据完整。所有失败区域／门槛保留，Goal active。

本轮 pending／来源排序追加 Goal 已交付有限执行修复及独立排序负结论，基线
`1d1b1a6b`，初始工作树／安装字节一致；现场均绑定干净提交及 Release 安装。
图文报告、机器验收、复现脚本及提交清单位于
[`goal_report_v1/REPORT.md`](../../log/20261011T044535Z_421/export/analysis/goal_report_v1/REPORT.md)。
原 active33／pending34 在 gen637 的定向反例：A 完整尾段拒绝，但 A 至 ts 和完整 B 合格，
原普通 pending 监督却拒绝并进入急停（`log/20261011T044535Z_421/runtime/pending_red.log`）。
`assessRemainingTrajectory(now, include_advisory, budget)` 统一以 server 确认身份选择分支，
删除 caller 的 pending_checked_stop 参数与构造标签 pending_branch_proof；已确认且未撤销的
普通／保护 pending 同用原 assessCheckedStopConnection。完整 A 尾段仍单独检查并决定取消
资格；未确认、接缝／连接／pending／motion／tracking／peer 失败仍拒绝。原确认和实际
命令权威、原预算与许可阈值保持；超过 ts 缺少实际激活反馈不推定已执行。
等待 B 确认因分支失效退出时，原发布动作不能清除保护请求；同预算复用
inspectPendingStopProof 确认失效并返回 false，保留队列身份，走原受检保护入口。
当前状态 IMPLEMENTED_LIMITED_VERIFIED。真实 A33/B34 反例红→绿、11项直接执行回归、2项
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
命令与 manager 激活身份闭合（`runtime/pending_refusal_green.log`）；两个现场身份审计共15,560条实际命令与自身wire spline数值闭合；OFF5101的
A9/B10独立拒绝gen195及stop11实际命令／反馈保留。初次原制动动力学不可行，
约.112s尚无合格完整接续分支仍是能力边界，不宣称已消除。

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
历史8个样本359条关联修正原行号并重建模型矩阵闭合，原排序 rho=.857 未改变。
3个独立 seed 的数据已采集，C 的排序前置未通过。

稠密 CUDA 取证新增原始审计反例：三个独立运行的输入 Ca／Cb 严格对称正定，
实际 float32 `Cb+R Ca Rᵀ` 因抵消留下约1.0–1.3e-7 m² 非对称，原按近零交叉项设
固定对称容差的 host 审计错误拒绝。保留原始数据和 red；host 审计改用两个3项
点积及一次加法的逐元素 float32 前向误差界（γ₃），检查构造误差及非对称界，
不对称化原协方差、不修复特征值、不修改 matcher／来源噪声。三个实际首拒绝
样本及超过算术界的注入错误回归通过；进一步核对发现原 target 输入也保留
7e-9–1.5e-8 m² 舍入偏差，host 不再错误要求逐位对称。按原输入的实际偏差加
构造算术界检查 tap，输出 source／target／tap skew 最大值，不修复原值或授予
统计噪声资格；6个真实拒绝样本及注入错误回归通过，三个原始全量取证共7,938,121条记录核验通过，queue drop为0／20／55均在
账本中，预定评价因子完整性闭合。独立24个局部
解、27次请求（3次启动缺失）均保留；3个 seed 是3个运行单位，时序点不是独立
实验。全路线 rho=.643／.786／.857，但冻结区域排序一致率1/3／2/3／2/3，不满足
预定≥.7的要求；形成区域排序尚未支持的负结论，C 未执行，不宣布 ON 收益。

以下为上一轮保留的历史实证记录，不替代本轮验收。

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
