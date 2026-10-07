# Curve、真实Advisory、通道与森林对照实施进度

本轮起始HEAD：173f0e9；当前分支dev/iap_refactor。场景与唯一入口固定。
实施方案来自本轮用户确认；旧四阶段报告只作为历史证据。
分析／构建证据run：`log/20261007T091820Z_056`。

| 阶段 | 代码 | 自动化 | 现场 | 直接阻塞 |
|---|---|---|---|---|
| A Curve接续 | 取证／逐阶段审计／生产初始化重放；一致切向与精确PVA拟合已实施 | 实际45/gen209红例→合法冻结曲线；50项baseline、server时间及2项重放回归通过 | d2f0c7a OFF300s：39曲线／38命令ID，前进36.096m；原规则未到达 | 精确pending撤销现场确认；旧hover越权与撤销确认接口已修待现场；物理时效与Curve／Target阻塞 |
| B Advisory空间／米数 | 历史RINEX／统一clock／前端、Monitor、Advisory活动钟差及v6冻结已实施 | 星历8项／43入口；100 ARAIM、clock／FIM／codec与baseline回归通过 | dcd7462历史60秒，5/6冻结，Monitor UNSAFE；正式0/9＋0/9 | raw整星座／联合故障、时间协方差／旋转与实际GPU残差资格不足 |
| C 通道／后端 | <=16前左右目标、米制连续多终端目标、同guide与实际曲线保留审计已实施；拟合接口修复 | 32 A*、48 baseline；真实冻结曲线路线偏离0.120m小于0.137m | d2f0c7a距终点0.447m；只进入0.5m邻域，任务未完成 | 真实原始PL导出／风险资格与现场后端保留不足 |
| D 完整任务收益 | 试验驱动／报告待完善 | 待做 | 配对0/6 | A/B/C真实前置未满足 |

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
