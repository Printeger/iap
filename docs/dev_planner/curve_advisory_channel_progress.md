# Curve、真实Advisory、通道与森林对照实施进度

本轮起始HEAD：173f0e9；当前分支dev/iap_refactor。场景与唯一入口固定。
实施方案来自本轮用户确认；旧四阶段报告只作为历史证据。
分析／构建证据run：`log/20261007T091820Z_056`。

| 阶段 | 代码 | 自动化 | 当前现场证据 | 直接阻塞 |
|---|---|---|---|---|
| A Curve接续 | 取证／生产重放／精确PVA／同guide／重定时及发布proof所有权已修复 | 真实Curve红例保留；7组规划＋43入口；到达审计4项 | 2ff5c31d OFF300秒：54发布／48命令ID／31激活，原FSM到达；发布首违同代 | 已激活后的迟到撤销42／45未闭环；完整接续资格未通过 |
| B Advisory空间／米数 | 历史GPS＋北斗／活动钟差／raw整星座故障；FGO世界位置协方差；原子优化状态与jointcov v8、原生GPU残差及后台取证已接入 | 广播独立222074配对；核心9组／规划7组／入口43项／离线14项／残差审计4项／校准6项通过 | v7真实冻结／坐标代数读取通过，Monitor持续UNSAFE；v8 60秒549条packet／5份原时间重放通过 | 未传播原时刻差、异常跳变、联合故障、Up/外参与生产GPU残差资格不足；正式0/9＋0/9 |
| C 通道／后端 | ≤16前左右目标、米制连续多终端、同guide及独立实际路线检查已实施 | 米制／终端项／预算incumbent／风险与guide机制回归通过 | 同版本原始sample与guide／实际拒绝曲线图已导出；新现场末次ENVIRONMENT_UNOBSERVED拒绝 | 真实低风险选择及后端保留资格未通过，实际PL米数未验证 |
| D 完整任务收益 | 冻结对照／诊断驱动已有；正式协议保持 | 正式前置未满足 | 配对0/6；最近OFF诊断原FSM到达，末truth距目标.286m | A接续及B/C真实资格不足；默认不推广 |

此表是当前状态；以下按提交／run记载的阶段事实是历史证据，不授予新的资格。

## 事实、假设与策略

事实：历史末次guide有53点，Curve返回但actual_curve/final_check为空；取证
入口晚于优化／动力学早退。不能推断其具体动力学失败原因。
假设：初始化、优化、边界绑定、重定时或最终检查中的一处导致失败；新现场
候选及同输入重放用于区分，不先选定根因。
策略：prior OFF；阶段A guidance OFF；正式星座使用时刻匹配的历史GPS＋北斗
RINEX，2022-07-06T12:00:00Z统一/clock；synthetic只作机制回归。
连续偏好采用1+0.5r，未知1.5；现有双源9+9协议保持，不能以LiDAR-only替代。
默认不推广；在线预算、净空、原任务终点及全部执行授权保持。

## 当前交付边界

取证最多24份阶段曲线，保留前23和最新并记录丢弃；原唯一后台writer与队列
上限保持。未生成／未检查状态明确，不借用上一条执行轨迹。相关代码在
同一逻辑提交中更新EGO流程、README和本进度。代码提交不等于Curve根因已修。

审查修正：每次候选修订撤销旧final_check；执行监督不携带候选阶段。
首个Curve最终拒绝单列attempt_failure_curve，不增加近期缓存或writer队列。
启动40项契约及2项规划/导出CTest通过；证据在分析run/runtime/capture_*。
森林driver取得唯一结束归属，启动失败也登记并finalize为failed。

Curve事实：cc02469 的120秒OFF现场在12/gen260获得55点guide；优化正常
退出，重定时四次动态比率仍为2.469/1.374/1.258/1.195。取证绑定完整。
生产重放保留原图与原时刻；单纯重拟合仍失败，零末端速度改变边界，不能当修复。
已定位可复现接口问题：边界P/V/A重绑定后内部控制点未随之refine，邻接导数持续超限；真实完整根因尚未闭环。
已在原4次检查/总预算内恢复refine，重绑索引，保持所有最终物理门。
旧离线run 20261007T100131Z_836 / 20261007T100133Z_005 未绑定生产view，缺少净空平面，仅保留诊断资格。
修正后的真实平面／原剩余预算重放20261007T101503Z_947仍失败，消耗两次剩余修复并被第三次拒绝；隔离完整预算20261007T101513Z_849也失败。正式A未通过。
6项规划CTest及40项canonical启动契约通过；重放覆盖原症状、自由图refine/PVA、障碍修复额度拒绝、backend早退结构化证据。
新增动力学回归fixture仅平移捕获控制点到观测自由空间，物理现场资格单列。


ecc7af0干净GPU OFF300秒现场：9次发布、server命令ID1–9、实际前进4.444m；距原终点31.557m，未到达。所有进程正常结束，A仍未通过。
证据：log/20261007T102011Z_225；分析图/JSON在本轮分析run/export/analysis/curve_refine_live。

B输入审计直接事实：固定版本RTKLIB180043ee对GPS31/北斗44颗在12:00UTC起300秒窗口每60秒交叉检查。C59/C60位置差最大3,636,455.77m；其余位置差小于0.3微米；除C59/C60外生产速度最大差0.01124m/s；含它们时最大209.321m/s。北斗TTR少14秒，AODE未读入；C39最近TOE记录的TTR在查询时刻之后。结果尚无输入资格，不可用于正式试验。
审计源码、参考实现hash、两份CSV及比较JSON保存在本轮分析run/export/analysis/rinex_audit。GAL/GLO未参加本轮数值交叉检查。

C当前策略：≤16终端保持原100³池，固定任务终点在池内且端点合法时优先；粗空间网格覆盖前、左、右并分散端点，取消参考1m球与投影增量门。端点合法不授予路径可达。每轮OFF/ON都先做同一完整guide搜索，仍计入原共享Search额度；该guide决定初始化、端点和末端切向，制动空间按原规则限速。已有曲线不作未经版本验证的guide复用。米制全边／起终连接积分与剩余距离一次计入，A*保留完整incumbent并区分最优证明／原预算内未证明结果；实际曲线风险保留与现场资格后续实施。

C1/C2自动化事实：原米制红例2m路径随步长计为24/12；已修复并保存红绿输出。
31项搜索回归含低预警线双路线OFF/ON、任务剩余项一次、共享目标体素独立解析成本、
原预算内完整incumbent未证明返回。GridMap拒绝1e9哨兵并保留raw/status/version。
6项规划回归与40项canonical契约通过。全包CMake/style lint仍失败，输出channel_legacy_lint.log；未做全文件格式重排。
完整反馈窄墙夹具会因射线锥收窄正确撤销pending（unknown点[-.4003,-.4393,1.0538]），
宽前墙仍缺起点后方观测；切换确认夹具改为显式回波静态房间，保留全部原断言。
两种旧输入的拒绝证据均保留，不据新夹具宣称真实森林安全或接续通过。

C1/C2现场直接事实：干净25023ac，run20261007T111334Z_929，35次发布、32个命令ID，
实际运动[-18,0,1.5]→[10.19805,.71399,1.88004]；300秒未到固定终点。
任务前进不归因于PL。日志存在pending未知撤销与一次unverified hover，不能声明
无安全／接续失败。最新失败地图与完整事件保留；分析run/export/analysis/channel_metric_live。
C3策略：原拟合余量＋体素外接球作为路线偏离界，半／四分之一体素积分比较收敛差；
同版本模型乘数（含UNKNOWN1.5、既有预警回退3）比较归一化代价；VALID PL资格另列。OFF/ON同审计，ON独立风险门；不改变物理门。
该数值可比性不等于B的概率／米数资格。自动化和新现场待登记。

C3审查修正：模型乘数与搜索共享唯一映射；全部UNKNOWN仍保留模型版本。
状态有效空间比例与来源贡献覆盖分别报告，后者当前not_available。
半／四分之一体素不一致红例导致guide短越线；搜索／实际fine统一四分之一体素，
不扩大时间预算。原弱墙ON更细采样可先耗尽搜索，也可因路线丢失耗尽修复；
两种失败均不覆盖执行轨迹，OFF同物理输入保留PVA接续正例。

C3相关Release四包顺序构建通过；32 A*、46 baseline、GridMap行为回归通过。
其余5项生产重放／管线／定时／反馈／失败导出及40 canonical契约通过。
日志channel_retention_final_*、channel_retention_verified_baseline.log；原迭代红例完整保留。
全包CMake/uncrustify仍有既有及新增段落格式分歧，单列channel_legacy_lint.log，
不称所有lint通过。新的C3森林现场尚待干净提交；真实PL资格与正式对照仍阻塞。

最终原弱墙ON输入还观察到首次solver -1008 rounding-error早退，严格保留为Curve拒绝，
不放行、不增重试；该回归按权威Search超时、预算／修复耗尽或非正常solver退出
核对失败身份与旧执行不变。OFF仍要求成功及全部PVA接续断言。


B1事实：生产共享广播接口已修复C59–63 GEO及位置模型速度微分、
BDS TTR +14/AODE/AODC；所请求星座必须全部有健康已播发原年龄内记录。
唯一loader拥有格式及整数资格检查；无新鲜度重赋。仿真器与前端均接入。
B1自动化：8接口回归（真实C59红绿、时间/issue、任一星座缺失、
未使用mixed系统、畸形/截断、非整数健康、未选BDS问题）通过；两个Release生产目标构建通过。
同原NAV、独立RTKLIB的available选择在6历元444配对身份/metadata一致，
最大位置2.493e-7m、速度2.702e-4m/s。此前原实现及首版过严mixed拒绝红例保留。
B1策略：保留原年龄限制及strict no-fallback配置；不改变Advisory准入。
B1阻塞：canonical历史clock/严格输入配置、per-system clock/FIM/故障、
冻结合格时间/外参/残差模型未完成；正式0/9+0/9，D0/6。

C3现场事实：干净a301cb1、run20261007T115721Z_898、300秒OFF，
5发布/5命令/ID4定时切换，真实前进10.724m，终点距离25.290m，未到达。
本轮未记录pending撤销或unverified hover，不代表完整安全接续资格。
首个Curve45/gen209：guide合法、动力学与物理通过，路线偏离0.590m
大于策略corridor0.1366m；后续路线修正导致动态失败，原额度拒绝。
末次仍为Budget，路线偏离后重定时/refine耗尽3次修复。不是通过放宽路线门解决。
B仍未取得有效PL覆盖；来源贡献not_available，不能把UNKNOWN模型比较称为真实风险验证。
图文/JSON在本轮分析run/export/analysis/channel_retention_live；原现场不删失。


B2代码：唯一入口显式rinex_nav_file固定历史UTC12、冻结原文件/hash、
GPS+BDS且禁止synthetic回退；唯一SO3clock按物理模拟tick推进，消费者统一ROS时间。
steady节拍／预算、run真实UTC身份与原300秒任务window未改，sim elapsed另报。
暂停／恢复冻结clock/physics/传感器；历史GNSS与实际first-hit LiDAR使用ROS timer。
必要clock/GNSS退出及clock源竞争停止全图，首个及逐模块输入故障留存，
唯一finalize权威优先failed，driver使用其返回值。默认guidance OFF，ON仅显式实验。
B2自动化：43canonical；两个CPU过程回归覆盖clock/odom/IMU/GNSS/LiDAR暂停恢复、
非法日期/self-clock/竞争源、strict缺文件退出；8广播接口、原planner回跳拒绝通过。
这使用合成小型传感器输入，不计正式现场或真实预测资格；生产全图现场待登记。
B2阻塞：BDS状态/cov/ISB、活动列FIM及故障、冻结时间/外参、实际残差与经验校准尚未完成。
正式B0/9+0/9，D0/6；A/C真实修复与路线保留仍受最新Curve/Budget证据阻塞。

B3事实：131459Z_623历史诊断有24行分星座钟差、12份含交叉项的ISB协方差，
但实际优化器仅注册c，缺d/h/j；GTSAM类型阈值查询没有缺键保护。正常退出
不能证明优化正确；原报告已标为未合格。原状态／GNSS epoch最大相差0.057s。
B3修复：因子键与优化器注册共用映射；每个钟差沿用原GPS阈值。缺键断言
先红，修复后96项ARAIM与实际扩展注入回归通过；注入回归使用同生产类型表。
Release构建／安装通过，新的干净现场待登记。活动列FIM／故障与冻结时间、
外参、实际残差资格仍缺，正式B0/9+0/9、D0/6；未推广。

B4事实：eb1b10e干净历史run133003Z_100三项GPU／安装身份通过，所有必要
过程正常退出，c/d/h/j原阈值齐全；24行clock诊断、12份ISB协方差。
GPS/BDS观测4744/11860；Monitor88.542/211.622m，前进4.322m、未到达。
图文与原请求在analysis/historical_clock_registered_live，仍无Advisory资格。
B4红例：混合观测只有4列，未知／旧mixed输入被接受；全套旧fixture仅改
编号未改GPS声明，身份冲突保留红证据后补显式星座。
B4修复：共享实际星座活动列；单星／整星座故障重建剩余列并比较位置子块。
独立增广矩阵／偏差不变性、系统／最后一星移除、真实退化及Monitor身份计数
回归加入；100项ARAIM与共6项CTest、43canonical通过，相关Release库／组件／
node构建安装通过；新的Monitor现场待登记。Advisory raw／FIM未迁移，冻结
时间／外参／实际残差仍阻塞，正式B0/9+0/9、D0/6。

B5事实：d2d0b1c干净60秒history134006Z_572；551次Monitor计算中位
0.812ms、p95 0.954ms、max1.493ms；末HPL/VPL99.982/224.361m，
前进7.705m未到达，冻输入5/6、首个失败留存。图文／hash在
analysis/historical_monitor_clock_live。不能以链路正常授予B资格。
B5红例：mixed S0只有4列；FIM差0.1544；raw把subset−full写反得到
0m分离项。B5修复：raw／query／legacy FIM共用活动列与原信息消元，
缓存包含星座，raw单星最后系统移除重建列；新v6冻结模型身份，旧格式
只保留诊断资格。focused回归绿；最终4组core与codec／baseline复核通过，相关Release构建安装
通过、43canonical通过。首次全包测试的录制夹具失败已修复，其余7组行为
检查当次通过；flake8／CMake／uncrustify仍失败并单列。新的干净现场待登记。
B5阻塞：raw整星座及联合故障、状态epoch／cov同步、外参旋转不确定性、
实际GPU残差／噪声与9＋9均未合格；B0/9+0/9、D0/6，不推广。

审查P2复现：近退化GPS8＋BDS1，最后BDS移除时rank-one分母舍入为正，
没有重建列。原阈值真实红例捕获；按实际星座计数强制删除该列，保留原秩门。
原Advisory录制测试另缺owner clock契约，补SYNTHETIC_TEST_DOUBLE字段；
不改变生产拒绝。全包flake8／CMake／uncrustify失败单列，不能称所有检查通过。

v5真实冻结原输入复核：clock_v5_history_diag保持recording_version=5、
legacy_common_clock、原reference1657108856.628；历史诊断身份及生产wrapper
拒绝旧版本均保留。未改写原run记录，不借重编码变为v6资格。

B5现场：干净dcd7462历史60秒run140712Z_090，GPS／北斗观测4736／11840，
伪距因子4408／11020；v6冻结请求5/6、首个失败保留。末Monitor
HPL/VPL101.240／237.451m，前进4.340m未到达；3条发布／3个命令ID。
图文与hash在analysis/historical_advisory_clocks_live。链路可运行，原B资格缺口
仍在，正式9＋9与6保持0。

A/C取证补齐：生产`audit`重查真实attempt45/gen209每个候选阶段，保留原时刻、
物理代数和stage-owned guide。run141714Z_204确认guide_fit／guide_bound／首轮
optimized偏离0.149697／0.346292／0.590059m，界0.136603m；最早拟合已丢路，
边界及优化进一步恶化。无PL补造、无在线预算或执行授权；geometry fixture在
原坐标及平移坐标下捕捉同症状，2项重放测试通过。拟合／边界／求解语义尚待修复，
不把新的红例诊断写成A/C完成。

A/C拟合接口修复：真实45/gen209在生产helper红例中终端切向点积仅0.15136，
软拟合两端P/V/A不精确，硬绑定改变控制点0.36571m；仅精确边界修复后仍偏离
0.324051m，证明短末端连接与拟合采样切向不一致仍致丢路。
策略：共享同一guide弧长采样的末端切向与初始化，原计数／间隔／物理制动限值
保持；边界P/V/A从拟合开始就是等式，随后绑定不再改变形状。无额外重试／预算／
清空安全拒绝，独立动态／物理／路线／发布门不变。
生产冻结initialize重放run144136Z_739：原图gen209、原规划／cloud时间、原剩余
额度；动态与601点物理检查OK，路线偏离0.119900m小于0.136603m，新增repair0。
原与重算终端速度、原控制点和重拟合控制点分列，PL仍NOT_AVAILABLE。
新stage记录terminal_stop；initialize拒绝缺owned guide和无策略历史零速度，非零
旧速度仅能证明继续策略。48项baseline、2项重放方法、管线／定时执行／完整反馈3组与43入口契约通过；
相关Release库及全部生产planner构建安装通过，新现场接续待提交后验证。
旧attempt12缺owned guide，不能借顶层后来的guide宣称同输入初始化通过。

A/C最新现场事实：干净d2f0c7a，run145306Z_103，OFF／prior OFF300秒，
固定终点和原预算。39次曲线发布、38个命令ID，10次scheduled激活；
真值[-18,0,1.5]→[18.096356,-.001162,1.936614]，前进36.096m。
终点距离0.447m是0.5m邻域观察，不能作为原到达规则通过；FSM未完成转移。
3次pending撤销，未见unverified hover；首个Curve31/gen295与末次Target274/gen2195
冻结输入完整留存。图文在analysis/curve_exact_fit_live，不授予PL或正式收益资格。

A接续新红例：旧森林ID31在授权撤销后、检查刹车前激活；真实server小型回归
精确撤销ID4后仍激活，2.5秒可重复命令信号，原binary/source/test hash已登记。
事实：旧server没有独立撤销语义，只在IMMEDIATE替代时清队列。
策略：复用现有Bspline空载荷CANCEL_PENDING=2精确撤销，server保留当前执行；
接受ID单调保留防迟到副本。撤销先于取证／制动构造，不新增FSM／budget／运动授权。
错误／已激活ID及非空撤销拒绝／忽略；晚到撤销不能逆转已经激活的轨迹。
统一消息及五个下游包Release构建／安装通过；四组接续行为CTest（baseline48）、
43项入口和2项Curve重放方法通过，相关日志pending_withdrawal_*。审查修正重复
WARN限频；没有剩余高优先级发现。c426b81 OFF300秒现场run152217Z_653：ID10提前0.609s撤下、未激活／命令；
当前ID9继续并接入检查刹车。33曲线／32命令ID／9次定时激活；前进36.062m，
距离0.669m、未到达。另有一次后续未验证hover；末次266/gen1870 Target和首个
55/gen107 Curve保留。B正式0/9+0/9，D0/6继续阻塞。

A检查刹车授权红例：c426现场1791386794.095拒绝检查刹车，却发布未验证ID33。
真实FSM回归在物理未知／地图过期／运动质量不支持三种输入下同样被改为新ID与
新控制点，red_verified记录原binary及源码hash。
事实：调用方把false转换为unchecked EmergencyStop生成与true；不是物理拒绝需放宽。
策略：删除未检查生成入口；false直接返回，不写新轨迹、不发布、不占执行ID。
继续原FSM和检查制动次数／预算；保留旧命令不等于安全停止保证。新合法输入
可以通过原检查制动、保持确切起点P/V/A、零终端速度并发布。自动化及新现场待登记。

审查P2组合红例：撤销pending后刹车失败，旧ID继续但manager永远等待新ID，
阻塞之后的合格输入。真实FSM／manager在合成100→101.7s时钟下复现，
red_verified及source/binary hash保留。修复：现有pending只追加撤销请求时间；
同一命令的旧ID＋header时间在请求与预定生效之后确认撤销，释放local pending而
不改执行曲线／授予新运动。前置／过期／未来／其他ID不得确认；若pending已实际
激活仍承认该执行身份并保持恢复要求。独立ID/time atomics合并为原子消息锁存，
无新topic/FSM／轨迹缓存／重试额度。新回归与提交后现场待登记。


审查P1红例：server切换判断与求值／header两次clock读取跨越102s生效边界，
得到旧ID＋103s时间；可错误确认撤销。真实callback合成测试17ms复现，
原身份与日志server_clock_boundary_red保留。单次时间采样修复并处理首帧零dt
的yaw NaN；不改时间窗／动态限值。50项baseline、server时间回归、两项重放方法、
管线／定时／完整反馈共六组CTest与43项入口通过；probe-free三项八轮全绿。
迭代两次139失败仍留存，崩溃原因未证实，不能把重复成功写成根因修复。
Spec／Standards复查分别登记，无剩余硬问题；新现场待干净提交。


A恢复接口现场：干净941fe41、GPU三项／安装Release身份复核，唯一入口OFF／
prior OFF300秒run160125Z_356。27条曲线／26个命令ID、8次定时激活；
真值末[11.289083,.231779,2.419395]，前进29.289m、距终点6.778m，原规则未完成。
ID4提前0.714918s撤下，未激活／命令；检查制动拒绝后旧ID3继续，在预定生效后
确认并退役本地pending。2次检查制动拒绝、未见unchecked hover；不能据此声称
安全停止或整轮安全。末次362/gen2151 Target、首个Curve12/gen55与完整原图保留。
图文／独立wire→command核对在analysis/checked_recovery_live；物理时效仍阻塞。

B真实空间诊断：原历史v6 input SHA296d38f5...未经重编码，原时刻1657108846.095、
原图gen326同输入生产wrapper重放。100个物理样本（63可查询／37物理过滤）；
63中38仅LiDAR、15 GNSS raw有效＋LiDAR、10 GNSS information贡献＋LiDAR但
raw singular_geometry。63/63重复／batch／wrapper一致只证明确定性。
联合诊断HPL0.273–0.446m／VPL0.261–0.504m；raw与information值分别留存，
不能以联合小值消除raw大值或资格缺失。原状态与GNSS差49ms未传播；Up／旋转、
实际GPU残差、raw整星座与联合故障资格未完成，无可达路线覆盖资格。
图文physical_pl_sources.png与report在analysis/clock_v6_original_diag，原正式0/9+0/9、
D0/6继续阻塞；未调参、未推广。


A物理时效取证：c426现场限频warning的137样本中位189.811ms／max342.743ms；
accumulate包含mutex等待，不能直接归因为射线。单帧真实map beam几何进入生产
snapshot构造稳定69–73ms，形成可重复性能红例；身份与每项原binary hash已登记。
假设／测量：ASCII格式化单独改后31–37ms；按既有时间排序直写压缩数据、去掉
66.528MB状态／时间临时数组后稳定10.5–10.9ms，首轮14–15ms。原10ms指标仍
未满足，不授予性能整体资格或完整300ms根因。字节／来源／两个hash与原版本完全
一致；相同时间、乱序加入、旧命中不被新free覆盖的回归含旧生产hash oracle。
原缓存、物理时效、预算及运动门保持。23项registered、两组GridMap及六组
planner行为与43入口通过，Release plan_env构建／安装通过，新现场待提交。


A物理快照现场：干净61f6425、三项GPU／安装身份通过，OFF300秒run161537Z_154。
32曲线／29命令ID、11次定时激活；三条撤销7/15/27均未激活／命令，撤销余量
0.830/0.594/0.322s，未见unchecked hover。本轮未出现制动拒绝，不扩展拒绝证明。
真值末[12.464440,.360832,1.856333]，前进30.464m、距目标5.559m，任务未完成。
末次631/gen2759 Budget及首个Curve45/gen231留存；实际净空／未知覆盖和额度
仍阻塞。141条限频warning中位41.671ms／max136.977ms，非严格配对性能资格。
图文与独立wire核对analysis/physical_snapshot_live；命令P/V/A最大线缆误差
7.32e-8m／4.63e-8mps／1.64e-6mps²，不替代现场物理或真实误差资格。

同输入12/gen55 audit run161348Z_545：23阶段均检查，fit0.147885、优化0.232996m
超过原0.136603m corridor。initialize run162232Z_462在原时刻／图／剩余额度下
动态与601点物理通过，但路线仍丢失0.232996m；wrapper physical verdict=true不是
路线通过。尚无新增修复，原动态与路线红例保留。此前362/gen2151的枚举阶段
正确为Target，已校正本进度／流程误写的Search；原raw记录未更改。

C原始风险导出红例：同一真实attempt3/gen18已查询793次冻结风险，候选CSV0行、
context_matches=false。冻结query局部缓存与旧全局缓存导出分离，且物理snapshot
取得早于当前风险绑定；不借插值／色标补值。assert_frozen_risk_export.py一条命令
复现，原代码／input／binary hash在frozen_risk_export_red_before。修复待实施。


C取证接口修复：共享原始query cache与分类视图，按明确冻结物理代数绑定；
导出前核对几何并在规划线程复制值。事实是旧export与实际query不共享authority；
策略是修该接口，不补算／插值，不以live最新值替换。CSV新增原来源mask与GNSS
raw geometry状态，后者不授予联合资格；1e9保留INVALID。测试夹具按新接口
赋值，补live先更新再capture及错代数回归；首次planner编译失败日志原样留存。
自动化最终结果与新现场待补；A路线红例、B正式0/9+0/9、D0/6保持阻塞。

最终Release构建／安装、三组地图与七组planner行为检查、43项canonical通过；
首次编译失败保留。命令、source/binary/log hash登记于frozen_risk_export_implementation。
冻结查询取证修复的现场仍待干净提交，不替代B／D资格。

Spec／Standards最终均无硬问题；INVALID不授予当前VALID，既有合格历史的
STALE_REFERENCE有界回退保持，缺历史时UNKNOWN。


C冻结原始导出现场：干净49de7c4、GPU三项及安装Release身份通过，唯一入口OFF／
prior OFF300秒run163937Z_338。attempt1/gen16的819次实际风险查询导出106唯一
原始格点、risk18且context匹配，原红例checker通过。106项flags1433全部LiDAR
贡献、GNSS raw未计算；非双源资格。三维同版本物理／H/V／来源／guide／被拒绝
曲线见analysis/frozen_risk_export_live/raw_physical_risk_curve.png，无插值／重新查询。
37曲线／33命令ID、15次定时激活；撤销6/13/16/22均未激活／命令，余量
1.091／0.967／0.251／0.325s，无unchecked hover。本轮无制动拒绝，不扩展拒绝证明。
实际前进29.148m、末[11.147866,-1.414617,1.766779]、距目标7.002m，原规则未完成。
首个Curve55/gen409与末次Budget563/gen2548保持；全部299次进程健康采样、
命令／binary／配置／输入hash见frozen_risk_export_live_final。取证接口现场通过，
A路线／物理拒绝及B正式0/9+0/9、D0/6继续阻塞，无参数推广。


A/C路线纠正接口调查（未完成）：12/gen55生产backend重放164734Z_062保持原时刻、
物理图、P/V/A及剩余额度，物理／动态通过但route0.232996>0.136603m。
assert_route.py已实际exit1锁定路线症状；先前checker字段名错误单列，不算红例。
独立固定控制点／interval／PVA拟合采样1→16倍后仍0.145149m，原拟合复算
控制点差<2e-14；此假设不足以修复。fit_probe只诊断，不当生产绿证据。
真实接口回归PureRouteCorrectionPreservesLegalDeviationInsideItsCorridor红：
0.08m合法偏离仍被施加guide中心线约束。策略是纯几何只纠正超原corridor
样本，使用原half-voxel拟合余量；将低预警线风险丢失与几何丢失分别传入，
未知／物理与风险支持约束保持独立，最终corridor／物理／动态门不变。
当前代码修复构建中；完整生产原输入纠正与新现场尚待验证，A未授予通过。


路线接口最终Release bspline／planner构建安装、七组planner与43入口通过。
Spec P2曾复现：最近AVOID段被过滤后，远处VALID段距离错误替代几何距离；
现先计算全guide几何距离再独立选支撑，带预警回退回归通过。低预警风险
目标恢复原中心线且不增余量，迭代-1008日志保留，没有放行异常退出。
两轴最终无硬问题，命令／source／binary／失败日志hash见route_corridor_implementation。
12/gen55诊断扩展probe170108Z_172不是完整生产入口：同原剩余两次额度，
物理／动态通过但最终route0.264558m，额度拒绝；正式A仍未通过。首次单向
纠正0.496693m、重定时refine0.234067m，保留平面另一侧及重绑定调查线索，
不据接口绿例宣称整条根因修复。原始input/time/PVA未改；PL仍NOT_AVAILABLE。


A/C双侧路线约束调查：固定捕获12/gen55控制点及简化共线guide，平移+16m x到
已观测自由fixture；端点PVA绑定后，生产接口一次原额度内纠正偏离0.491611m，
其余断言通过，guide_tube_symptom_red实际exit1。此fixture物理近似明确，
不等同原森林数值／资格；原冻结生产红例仍在164734Z_062。
事实：双侧二次成本改善偏离但原剩余两次纠正后仍0.139751m；删控制点平面
仅改3.1e-5m，不足解释残余。可移动basis归一化／端点余量版本恶化，失败日志
与hash保留，没有采用。策略：原半体素余量在真实几何丢失时绑定整条实际曲线，
双侧四次excess成本重视峰值并消费现有guide_weight；无真实几何违反仍不添加。
原始time／图／PVA／额度下，诊断扩展175524Z_207在两次原纠正后动态与完整物理
通过、route0.135264<0.136603m。此结果尚不是正式生产重放或现场资格。
现在将纠正额度／原因／约束绑定收拢为生产manager与backend replay共用接口；
原例测试覆盖原剩余两次且逐次独立查动力学，未扩大任何预算。正式入口将分别
报告各门及physical_geometric_candidate_valid，禁止以仅动态绿值表示候选合法。
重定时后同guide约束及新森林接续仍待后续验证；B正式0/9+0/9、D0/6不变。


重放取证边界补充：真实生产重放在已dynamic pass后发生纠正backend早退时，
原feasible会错误沿用。测试只包装生产main的输出，在“纠正准备完成”处消耗
原steady预算，不改ROS采集时间、不加生产fault开关；旧源码单项实际红
curve_late_failure_red：dynamics_feasible=True。现新solver前清除结论，当前候选
未查即not_checked。另将旧refine-only“动态／物理绿但route丢失”的exit0改为
复合拒绝；不是放宽原拒绝。ON缺原冻结预测输入拒绝回归。最终重建待验证。


A/C双侧路线纠正正式生产验证：Release构建／安装、七组planner行为回归及43项
入口检查通过。backend run182744Z_678、initialize run182745Z_581均保持原
attempt12/gen55图、时刻、冻结参数与PVA，原剩余两次纠正后动态／601点物理／
独立路线门全部通过：max0.135264m < 原corridor0.136603m，无重定时。
assert_route.py两次exit0；独立NumPy均匀601点采样图production_red_green.png保留优化红
0.233010m、第一纠正0.137271m仍拒绝、第二纠正0.135320m绿；此均匀采样
与生产实际采样的最大值分别报告，不混作逐点一致性证明。不存在预测PL输入，
OFF_GEOMETRY_ONLY，execution_authorized=false；不授予现场发布或Advisory米数资格。
晚期纠正失败红例修复及额度耗尽／ON缺冻结输入拒绝回归通过，旧refine-only
动态／物理通过但route丢失仍复合拒绝。两项审查最终无硬问题。
命令与源码／binary／配置／input／日志hash登记guide_tube_production_implementation。
新森林接续与重定时同guide保留仍待验证；B正式0/9+0/9、D0/6保持阻塞。


A/C双侧纠正现场：干净aa83f77、GPU三项与安装Release／配置身份通过，
OFF/prior OFF、固定终点300秒run182910Z_832，进程／录制driver完成。84曲线／
70命令ID，25次定时激活；14个撤销ID均未激活／命令，生效前余量0.041–0.954s。
独立wire→command P/V/A最大误差1.33e-7m／3.68e-8mps／2.05e-6mps²，
定时接续边界P/V/A差最大9.01e-8／7.21e-8／8.34e-8；无unchecked hover，
本轮无制动拒绝，不扩展拒绝证明。300健康采样中间288条全部所需进程存活；
其余为启动0.81–10.86s和正常结束301.68s，不能声称300条均已启动。
实际前进34.810m，末[16.810230,.518368,1.968758]、距目标1.380m，原FSM未完成。
首个Curve50/gen662：初始unknown、优化后净空与route拒绝，再纠正solver-1008，
当前final_check=not_checked；末次387/gen2640 Curve保留unknown与route0.179288m。
此轮验证合法发布／server接续／命令／运动的链条，但其余Curve根因及完整任务仍未
通过；与旧现场路径不同，不当严格收益对照。候选原始风险139项同物理66/risk66
全部LiDAR-only flags1433，GNSS raw未计算，不授予双源资格。图文／三维／hash
索引analysis/guide_tube_live与guide_tube_live_final；B正式0/9+0/9、D0/6不变。


A/C重定时绑定：抽取原生产retime清除入口后，真实单项红例
uniform_retime_red显示实际约束成本278.525266→0、梯度消失，仅保持断言失败。
策略：同一冻结guide／图、同控制点索引的uniform缩放保留normalized t/dt basis及
全部实际样条约束，普通新fit仍清除；换控制点数在修改状态前拒绝。不增加额度／
重定时次数，不放宽动力学、独立物理／route及发布门。两轴无硬问题；补非零端点
P/V/A重绑定后的样本求值回归。Release构建／安装、七组planner通过；扩展非零PVA重绑定单项及43入口检查通过。
保留样本加权位置与新样条求值差<1e-12，成本／梯度完全一致；现场尚待验证。
新Curve50/gen662原参数生产重放183620Z_726复现纠正solver-1008；与重定时
清除是独立问题，保持拒绝。B官方B1I 3.0原PDF重新取得并hash登记bds_icd_retry，
规范逐项审计仍待完成，正式B0/9+0/9、D0/6保持。


A/C重定时绑定现场：干净5a2f03f、GPU三项／安装Release／配置通过，
300秒OFF run184545Z_141。89曲线／81命令ID／49定时激活，独立wire命令P/V/A
最大误差7.81e-8／7.54e-8／1.45e-6；定时边界差2.01e-7／9.21e-8／1.17e-7。
八个撤销ID生效前0.084–1.104s撤下且无命令；第九个ID37已激活后才撤销，
server忽略，planner报告ENVIRONMENT_STALE随后独立已检查制动，完整接续仍有缺口。
另一次已检查制动拒绝无替换授权；无unverified hover。299健康样本中288条完整，
缺项均为0.82–10.86s启动。实际前进36.322m，末[18.322028,.033121,1.551477]，
距固定目标0.327793m、原FSM未完成；不改原到达规则，不当严格配对收益。
首个Curve49/gen595纠正后动力学失败／refine早退；末次211/gen2766原额度重定时
后ratio1.145056仍失败，final_check=not_checked。全部冻结候选保留。
6/gen100/risk114原始144项全部LiDAR-only flags1433，GNSSraw未计算。
图文／3D／命令／hash索引analysis/uniform_retime_live及uniform_retime_live_final。
B正式0/9+0/9、D0/6、默认不推广；重定时接口回归绿不表示全部Curve／任务通过。


B坐标接口：FGO原6×6是Pose3右局部切空间，平移块不等于world。
实际Pose3::retract数值扰动单项红例Up方差0.000425、独立期望0.0401m²。
生产提取现用同一优化X(IMU)的translation Jacobian，原local6×6仍交给LiDAR；
world位置协方差逆与轴sigma由原提取派生。时间／frame／准入／prior OFF不变，
旋转后谱及最大特征值运动代理保持；ENU与天线/外参旋转不确定性仍未取得资格。
Release core／实际GNSS及Integrity插件构建安装，16项fusion/covariance单项、
6组完整性／预测回归与43入口通过；两轴无硬问题，fresh snapshot前提写入接口。
红／绿／源码／binary／日志hash索引fgo_covariance_red_verified、fgo_covariance_implementation。
此修复尚无独立新现场协方差／GNSS时间配对验证，B不授予通过。

B星历扩窗：原历史NAV在2022-07-06 12:00UTC起300秒、10Hz共3001历元，
每历元31GPS／43BDS健康且原播发时间可用的NAV记录；222074生产／独立RTKLIB配对，
TOC/TOE/TTR/AODE及健康metadata全部一致，最大ECEF差2.766e-7m、
速度差2.828e-4mps、钟差1.084e-19s。速度为生产中央10ms与参考前向1ms分别保存。
仅广播传播接口，NAV总数不等于实际可见/使用数。原6历元证据不改写。
官方B1I3.0时间／钟差/TGD／GEO旋转审计见analysis/bds_icd_audit；C59/C60轨道身份
独立依据、信号传播/接收时间误差与生产双源各层资格仍待验证。
analysis/rinex_full_window绑定命令、NAV/独立源码/实际binary/输入hash；初次分析
仅小数offset字符串不同拒绝，改按原数值配对，原CSV不改写，不算模型红例。
正式B0/9+0/9、D0/6、默认不推广。


B／A协方差版历史现场：干净3634d4e、GPU三项／安装Release／配置通过，
GPS＋北斗RINEX、12:00UTC、300秒run190444Z_824。2992wire历元都含两信号；
GPS23936／BDS59840观测；GLIO真实PR残差样本23560／58900、RMS9.158／7.835m。
2944 Monitor均两星座、GNSS/LiDAR valid但UNSAFE，末HPL107.637／VPL229.277m。
C-G联合钟差差值协方差59/59有效，G自身差值不请求；原state/epoch差42–59ms未传播。
冻结28/30、两失败保留；原131风险样本25raw+LiDAR／32info-only+LiDAR／74LiDAR-only，
联合故障／来源资格／生产GPU残差／米数仍不足。原/clock无负跳，完整暂停仍未取得资格。
74曲线／67命令ID／37定时激活，真实前进36.053m、距固定目标0.165853m但原FSM未完成。
命令P/V/A及定时边界独立核对见execution.json；七个server撤销无命令，两个ignored
ID4/15不计成功。server日志为墙钟，历史接收余量不可判定；初次跨域相减字段已撤回。
无unverified hover，一次checked brake拒绝无替换；健康300条中288条完整，启动／退出缺项见JSON。
首个Curve4/gen62重定时后ratio1.100470仍拒绝；末次290/gen2294 Budget，末动力学
ratio105.239，final_check未检查，候选与原图留存。图文analysis/world_covariance_rinex_live；
world_covariance_rinex_live_final绑定完整命令、身份／NAV／配置／binary／input hash。
Curve49/gen595生产重放190538Z_435复现纠正后动态1.311→1.501→3.004及refine异常；
私有成本梯度诊断190805Z_870不是生产绿证据，条件数／约束可实现性仍待定位。
B正式0/9+0/9、D0/6、不推广；更靠近目标不写成任务通过或严格配对收益。


A/B server时钟取证接口：历史run190444Z_824暴露logger墙钟与/clock历史时间
不同域；原撤销ID证据保留，但不能从前缀得到生效前余量。现权威queue callback
直接记录receipt_ros_time_s、原effective_ros_time_s，激活记录activation_ros_time_s；
ignored撤销保留active ID。使用原单次捕获now／clock_now，不修改排队、授权或控制。
Release server构建安装，server时间／定时执行／完整反馈3组通过；新历史现场字段
及完整GPU图暂停尚待验证，旧跨域分析值撤回而非重写现场。正式B0/9+0/9、D0/6。


A/B历史时钟字段与完整GPU暂停现场：干净fe939ac、run192221Z_654，
GPU三项／安装Release／配置／NAV身份通过，OFF/prior OFF300秒steady，显式暂停12.006秒，
历史时钟推进288.156秒；不计正常配对／正式校准。排空后8.720秒，唯一/clock、
真值／IMU／GPU LiDAR／GNSS／GLIO消息计数及header冻结；命令872次重复同一历史时间戳。
恢复后继续推进，无负跳；DDS启动发现0计数单列，异常时间跳变仍未验证。
77发布／64命令ID／24定时激活，13个撤销原生效前0.163–1.041秒完成且无激活／命令。
比较使用显式receipt/activation/effective ROS时间；不使用墙钟前缀。本轮ignored为0，
不能据无事件声明其他现场晚撤销问题已修复。两个已检查制动拒绝无替换，unverified hover为0。
独立wire命令P/V/A最大差7.64e-8／3.85e-8／1.71e-6，定时边界差7.86e-8／5.26e-8／1.50e-8。
300健康样本288条完整，缺项仅启动／正常退出。真实前进36.551m，末距原固定目标0.551403m，
原FSM未完成；首个Curve11/gen122动态拒绝final未检查，末356/gen2617 Budget，
最终实际检查ENVIRONMENT_UNOBSERVED；两类状态分别记录。
2872wire双信号历元／GPS22976＋BDS57440；2825 Monitor双星座均UNSAFE，两条有检测，
不等于定向故障资格。钟差C-G联合协方差57/57有效；42–59ms原时刻差仍未传播。
冻结28/30、所有失败保留；candidate13/physical136/risk154的142原始样本39raw＋LiDAR／
39info-only＋LiDAR／64LiDAR-only。图文、实际命令、来源／时刻／hash索引
analysis/server_ros_clock_pause/report.md与server_ros_clock_pause_live_final。
Curve49/gen595原输入组成探针192357Z_326：前两次retime无物理平面，
479条tube占全部物理目标；第三次316平面，tube仍主导。近固定PVA的小可动系数只是
部分贡献，不能推断必然无解；原拒绝及诊断额外steady开销保留，不授予生产绿例。
组成／梯度诊断见analysis/attempt49_constraint_components，Curve根因仍未闭环。
正式B0/9＋0/9、D0/6、默认不推广。


B raw整星座补齐已实现：两个生产算法接口红例（机制几何）明确旧GPS-only有限PL及双星座
遗漏2个假设。固定Kmd=1，旧HPL0.948783／VPL4.795923，独立SVD完整故障期望
1.262886／6.167788。原生产算法＋测试源码／binary／日志hash见raw_constellation_red_verified；
红测试源码按该hash精确重建保存analysis/raw_constellation_faults/red_test_source.cpp。
加入每个活动整星座子集与N+K统一分配；单星座raw失败不取消完整FIM诊断，
不放宽已有来源准入。v7冻结clock/fault模型及hash，旧v1–v6历史读取；状态／epoch、
旋转及真实残差资格仍不足。118项预测模块、7组完整性／预测、7组规划、43入口与12项坐标／报告契约通过，
Release构建安装完成，新现场未执行。旧正例改为明确双星座机制输入，原单星座缺资格及FIM数学回归仍保留；
所有迭代失败未删除，不以机制夹具代表历史RINEX资格。B正式0/9＋0/9、D0/6。

坐标取证同步认可v7的确切clock/fault模型，v6及未知模型仍拒绝新资格；原record、
binary、时间、外参、方向及来源一致性门保留。真实v6原文件363cd9f1…在
195102Z_653解码／重编码保持codec6与legacy_single_satellite_v1，100请求生产
wrapper绑定／调用均0，81物理可用点明确historical_codec_input；原文件SHA不变。
历史诊断使用当前算法，不宣称与旧模型数值等价或新鲜度。日志raw_constellation_*；
首次广义Python discovery遗漏工具必需--binary而报导入错误，原失败保留，随后按
各工具正式入口完成相关检查，不能将该错误记为通过。联合故障与真实米数资格不变。


d738289干净历史GPS＋北斗OFF300秒现场195232Z_628：49发布／43命令ID／24定时激活，
6撤销均原ROS生效前完成、无对应激活/命令；0检查制动拒绝、无unchecked hover。
实际前进36.076373m，末位置[18.076373,-0.073743,1.166094]，距目标0.350377m，原FSM未到达。
首个Curve61/gen558及末次Curve192/gen1662均最终未检查；candidate2/gen14则是已检查
ENVIRONMENT_UNOBSERVED，98原样本8raw＋LiDAR、81信息贡献＋LiDAR、9LiDAR-only。
2992双信号历元，GPS23936／BDS59840；2945实际两星座Monitor全UNSAFE，C−G协方差59/59有效，
状态／GNSS差49–58ms未传播。冻结28/30，全部启动／结束失败保留。
4份真实v7冻结原时间扫描与坐标代数通过，但参考／状态178–228ms及Up轴变化仍未取得
校准或物理H/V资格。原0029不可用及对应重放失败保留，另取已成功0028停滞诊断；
未声称左右入口／全图覆盖。图文analysis/raw_constellation_v7_live/report.md及
raw_constellation_v7_live_final登记完整hash；正式B0/9＋0/9、D0/6、默认不推广。

A初始化诊断：Curve49原冻结P/V/A、guide、图、时刻及剩余预算下，将原.4m采样
细化至不超过现有voxel对角线、保持原名义时长／终端速度，200002Z_948通过原动态及
完整物理／路线检查，偏离0.097226m<原0.136603m。独立Curve50原输入200003Z_401
动力学／路线通过而完整物理仍ENVIRONMENT_UNOBSERVED、原修复额度耗尽；正确拒绝保持。
两者均为替代初始化探针，不授予生产修复／发布／现场接续资格；根因修复仍待实现及回归。


A Curve49红回归已捕捉最早guide_fit违反：真实几何（平移至自由观测图，仅机制资格）
偏离0.258424m>原0.136603m；实际生产原冻结initialize重放200354Z_877也仍红，
动力学失败、末次refine舍入错误／最终未检查。先前测试const API编译错误保留，
纠正后才登记症状红例；源码、binary、日志见voxel_sampling_production_red。
现补齐初始化采样：名义末端切向与时长保持，控制网格细化至既有voxel对角线，
原预算、guide、修复与所有门保持。生产构建／绿回归／完整原输入重放／新60秒接口现场取证通过；完整预测资格待完成。

审查P2红例：重复短guide拟合把细化dt再细化，原7.2秒错误缩至2.4秒；
voxel_sampling_repeat_red保留源码／binary／日志。明确名义间隔输入与实际输出，
TargetShortening使用本轮名义输入，不复用细化/retime输出；重复拟合边界及时间回归保留。
全回归首次6/7组通过，旧障碍夹具因新拟合直接避障而未消耗修复额度，原日志保留。
改为合法结果门加明确违规实际候选调用生产有界纠正，证明不覆盖执行曲线并消耗原一次额度。

新捕获重放红例 voxel_sampling_new_capture_red：0.4000000000000001秒实际间隔
被再拟合成0.36281902376604586秒；原红日志与测试源码保持。补充guide_fit名义间隔／
采样模型，重放区分旧粗采样历史规则与新显式名义值，未知／缺失模型身份拒绝。
障碍纠正夹具使用生产冻结物理上下文，避免默认查询未生成权威违反列表。验证待完成。

A初始化生产绿例：203359Z_935原Curve49/gen595 initialize完整动态／物理／路线通过，
偏离0.097226480m<0.136602540m，原9.6秒时长及起终P/V/A保留，原剩余额度新增一次纠正，
耗时0.001705秒；物理时刻／冻结图及参数hash原样。203400Z_589旧粗控制点backend仍红，
表明修复权威在初始化，不宣称旧非法曲线变合法。Curve50 initialize203401Z_241
动态／路线通过而ENVIRONMENT_UNOBSERVED正确拒绝；旧backend203401Z_912红保留。
Curve12 initialize203402Z_572／backend203403Z_221均完整通过。
事实：原guide_fit采样丢失起点反向速度所需转向；不能推断所有粗表示必然无解。
策略：沿用名义末端切向与时长，将guide弧长采样细化至现有voxel对角线，预算／额度不增加。
代码完成、Release构建／安装完成；7组规划＋43入口全通过，两轴只读审查闭环。
全部红／迭代失败保留；首次绘图缺scipy依赖失败保留，随后用独立Cox-de Boor绘图，
起终P/V/A及9.6秒由独立导数计算核验。图文analysis/voxel_sampling_production/report.md，
生产重放／边界／图／commands/hash登记voxel_sampling_production_{replays,report,green}。
现场接续待执行；B0/9＋0/9、D0/6、默认不推广。

A新现场921d340／203631Z_733：canonical干净/GPU3/安装Release/配置导航身份全通过；
111发布／91命令ID／45定时激活／19生效前安全撤销、无对应激活/命令；ignored0。
命令与wire P/V/A误差6.83e-8／2.44e-8／1.53e-6，定时边界1.57e-7／6.48e-8／1.99e-7。
实际前进35.290398m，末truth[17.290398,.084167,2.832231]、原终点距1.511774m，原FSM未到达。
3次checked brake拒绝，无unchecked hover；不据本轮宣称全接续／安全事件通过。
Curve99/gen1204新采样身份204344Z_809重放：名义1.2000000000000002／实际.4000000000000001、
初始control完全相同。真实起点速度X=-.578234m/s超过原.525m/s容差上限；精确PVA下四次
重定时均正确动态拒绝，最终未检查。不能提高阈值或替换真实起点来造绿例。
B2992双信号wire、2945双星座Monitor均UNSAFE，C−G联合协方差59/59，47–59ms未传播；
冻结28/30全部失败保持，正式0/9＋0/9、D0/6、默认不推广。完整3D/raw/命令/时间/hash
见analysis/voxel_sampling_live/report.md和voxel_sampling_live_final。

取证修复：现场两次assessment46 vs live47/50拒绝保存；生产红例epoch4→5同症状，
epoch_override_capture_red保留test/binary/log hash。原物理检查通过后FSM追加tracking拒绝，
现优先复用原physical_epoch.failure_evidence并在同epoch/time/motion查询补充cell，
不重捕新图，不改一致性门或执行决定。capture/evaluation时间分列。Release构建、7规划
组及43入口通过，两轴审查通过；新现场导出闭环待验证。审查另记既有组合身份风险：
pending物理拒绝＋active跟踪异常同时出现时，原FSM覆盖reason/time却保留pending ID，
仍需单独纠正，不据当前代数修复声明该组合通过。

pending＋tracking所有权生产红例：原pending物理失败被active tracking覆写，生成tracking_error
而非原remaining_failure；原test/binary/log hash登记pending_tracking_owner_red。
现pending物理原因／ID／本地首违时间位置保持，active tracking参考单列，原撤销与
匹配命令确认前保留predecessor均保持。首个物理违反为INSUFFICIENT_CLEARANCE，
首次绿检错误期待PHYSICAL_OBSTACLE的失败保留；改为精确原assessment原因，无放宽。
缺物理epoch的scalar query明确nullopt，live推进仍查原代数；组合测试还核对原代数、
pending本地首违时间位置。仅重建并清理本进程已登记临时test路径，外部run不认领。
Release构建／7组规划／43入口通过，两轴审查通过，新现场验证待执行。

30秒导出诊断d8598cb／210335Z_198：前置全部通过、进程完成、代数错配0；
remaining_failure12/gen33保存原1657108814.088评价时刻、首违本地15.246秒与位置，
INSUFFICIENT_CLEARANCE原因保持。candidate17/gen128未知拒绝也保存新首违字段。
本轮未发生pending＋active tracking组合，明确未取得组合现场覆盖；正式任务0/6不变。
pending_tracking_owner_live_export登记原配置/binary/log/snapshot hash。继续B原子冻结接口。


## B 原子优化冻结接口（v8）

GNSS smoother finish按实际注入帧／epoch保存优化与线性化均值，以及包含
活动星座clock交叉项的联合协方差；一次bundle发布／Monitor读取。缺数据或
失败不借旧状态，原时间差标明NOT_PROPAGATED。ROS及v8冻结接口详见
[Advisory契约](../spec/advisory_prediction_contract.md#b-原子优化冻结接口v8)。
旧≤7维持历史读取；原来源准入／运动／预算不变。codec红例3项失败已保存，
核心7组、规划7组、入口43项、离线14项与校准6项通过；新60秒接口现场取证通过；完整预测资格待完成。正式B0/9＋0/9、D0/6。


B v8干净732f057现场212610Z_122（60秒接口诊断）：549条实际CG、25维joint包，
owner/layout/coordinate均值错配0；5/6冻结及5份同原时间重放／审计通过，失败请求保留。
取证median5.566ms／P95 7.289ms，原预算保持；原state−epoch −60至＋47ms未传播，
优化与线性化位置最大差17.392mm。11条可配对ISB CSV与同包差值完全一致；
末次550帧Monitor未覆盖。图文／hash索引：
`log/20261007T091820Z_056/export/analysis/optimized_bundle/report.md`。
该短运行不是正式300秒任务；原FSM未到达，B空间／米数、真实GPU残差／联合故障、
C真实通道保留、D0/6与B0/9＋0/9仍阻塞。继续可独立执行的生产取证工作。


B GPU取证：安装API无per-match residual/covariance，不能用原汇总值或CPU重建取代。
已找到同头hash的85d0f4c上游本地源；IAP窄原生适配正在CUDA构建，原生产路径保持。
代价／Hessian／inlier等价、后台导出／现场和噪声／相关性均待完成，正式分母不变。

GPU原生取证机制：Release CUDA／8组合原后端代价、Hessian、inlier等价通过；
旧样本红例8项失败保存后修复，allocation/download/sync失败明确不可用；
同步与异步接口、128上限、零匹配和恢复覆盖。GPU构建CMake≥3.24。
生产路径尚未接入；真实残差、噪声／相关性、联合故障及正式分母保持待验证。


B GPU生产取证已接入原postopt quality pass；默认capture_advisory_residuals=false。
显式true仅更改run-local取证开关，原stride／匹配／预算不变。16×128后台raw queue，
小请求账本保留全部身份与drop，原实际CUDA e/S与归约count/cost独立保存；
frame/target/level/原时刻／外参／GNSS owner绑定，未匹配不借旧epoch授予资格。
核心9组、规划7组、入口43项、审计4项通过；新干净历史现场样本审计通过；正式噪声／联合故障及B0/9＋0/9、D0/6保持阻塞。
接口：docs/spec/gpu_match_evidence_contract.md。

GPU取证评审修复：全请求ledger保留原target/voxel/keys和GNSS epoch/update身份；
quality批次异常逐factor取消／记录后原样重抛，保持原失败行为。直接canonical启动无
driver身份时明确unavailable，审计不允许修改原时刻或借旧包；身份红→绿4项已保存。


B GPU现场闭环（干净7634cb49／222259Z_458，60秒显式诊断）：GPU3／安装Release／配置NAV
身份前置通过，全部进程exit0。3,983账本、3,982请求／可用导出、545 source frame、
357,945实际样本，queue drop／写失败0。独立e/S最大误差1.339e-6m／2.207e-7m²，
owner全部匹配；抽样median2.181%与多层相关性明确不授予独立noise或米数资格。
544双星座Monitor均UNSAFE，25D包owner/layout/means错配0；原state−epoch −91至＋50ms
NOT_PROPAGATED，5/6冻结、500原时间空间请求357 VALID／133 PHYSICAL_FILTERED／10
COORDINATE_ERROR，4 raw GNSS有限请求仍是诊断。18发布／13命令ID／3定时激活，5条
生效前撤销无对应激活／命令；前进10.251743m、末距固定终点25.759394m，原FSM未到达。
末attempt31实际曲线同图重放通过，但生产发布前制动空间净空拒绝；原代码仅覆写旧
assessment reason/position，未绑定新release epoch/time/cell，取证owner缺口需下一项修复。
同输入backend通过不等于release授权。核心9组、规划7组、入口43项、审计4项通过；
B空间／米数与C真实收益未资格，B0/9＋0/9、D0/6、默认不推广。
图文与原hash索引：log/20261007T091820Z_056/export/analysis/gpu_match_live/report_v2.md，
原相对图链接报告及manifest均保留，correction manifest只修正新报告链接。


### A 发布前组合物理检查的取证所有权

原冻结 candidate assessment 继续拥有实际曲线的 guide/Advisory 指标，最新 release
assessment 单独拥有发布物理证明；两者不是同一时间或地图的授权。新几何或新 attempt
清除两者；只记录检查结果的 release stage 不清除已完成的路线指标。

发布前仍按原顺序／空间步长检查前驱接续区间、实际曲线、末端可检查制动空间，
不增预算、修复或放宽净空。每次组合检查从同一 captureExecutionView 绑定完整
physical_epoch、generation、原评价 time、motion、首违 cell/position。curve／predecessor
首违 coordinate 是各自局部秒；terminal_stopping_space 是末端起算的米数，不能写成
曲线本地时间。该否定检查不更换执行轨迹。

failure snapshot 的 base/search/guide/input time 仍为原 planning view；独立 final_check
保存实际 release scope、原 generation/time/motion 和首违 section/time/stopping_distance。
新地图使用已有 final_check_cells.bin；不把最新原时刻赋予早期 backend 输入。每个检查
stage 保存 physical_generation/evaluation_time/scope 与首违 section。缺 epoch 的 curve 或
publication 检查均为 not_checked，保留 final_check_precondition_reason；阶段输出
physical_precondition_reason，不能借旧实际曲线检查写 checked。未检查的 guide/risk
不能当作零成本；CSV仍消费同候选的原冻结 route/Advisory owner。

同输入 backend 重放只证明原曲线检查，不涵盖最新前驱／制动空间与提交／server授权。
旧数据缺实际 release epoch 时保持该重放限制，不能从所存前一代物理图追认发布。
现场7634cb49 attempt31 的 phase5暴露该问题；生产 seam红例1→2代数错配与原binary/log
在 campaign release_corridor_owner_red 保留。完整Release／7组规划及43入口通过，三项机制与缺epoch真实导出回归通过，
两轴只读审查无剩余阻塞；安装已同步，新干净现场待完成；正式B0/9＋0/9、D0/6、默认不推广。


### 原 FSM 到达证据及发布 owner 现场核验

干净 `2ff5c31d` 的300秒 `225155Z_510` 已完成；54发布／48命令ID／31定时激活，
实际前进36.056m。attempt4 base59→release63净空拒绝；末完整拒绝attempt141
base1581→actual1584 OK→组合1585前驱未知，最新原map／motion／首违cell同代，
后续142–146成功。Curve89原时间生产重放仍动力学失败／final not_checked；141原
backend通过不等于组合发布通过。报告与hash索引见
[300秒发布检查现场报告](../../log/20261007T091820Z_056/export/analysis/release_owner_live/report.md)。

原到达规则保持。PRESET_TARGET 到达分支先同步调用 planNextWaypoint 重置路点，
日志可为 `[FSM]: from REPLAN_TRAJ to WAIT_TARGET`；仅匹配EXEC_TRAJ会漏判。
只读 `scripts/dev_planner/audit_task_arrival.py` 对原run提交源码、非初始化FSM转换
及原log hash审计，独立输出新resolver run；启动INIT、状态打印或SAFETY转换不算到达。
4项回归红绿（含原log缺hash／篡改拒绝）与历史审计 `230957Z_186` 确认225155与195232到达；203631与192221未到达。
旧报告／JSON／manifest不改写，旧false结论由本段更正。末wire54独立端点与附近
odom佐证保持分开，附近采集不冒充FSM精确应用样本。lifecycle退出0不授予到达。

7次pending撤销中5次生效前成功，42／45迟到约.070／.138秒且已命令执行；
原FSM到达不授予全接续安全资格。当前A合法发布／切换／运动及到达已观察，迟到反馈
仍待闭环；B传播／方向／噪声与联合故障资格阻塞，C真实合格风险收益未验证，
B0/9＋0/9、D正式0/6、默认不推广。本轮未启用GPU逐匹配导出。
