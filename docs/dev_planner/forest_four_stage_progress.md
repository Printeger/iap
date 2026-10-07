# 真实森林四阶段实施进度

开始：2026-10-07；目标仓库 `src/iap`，当前分支 `dev/iap_refactor`。
工作初始 HEAD `5de1ec0db8621696281809f42969b83fe178e6c7`，工作树干净。
统一场景与唯一入口按用户要求。GPU nvidia-smi / cuInit / device_count 均通过。
安装 ego_planner_node/traj_server/failure_map_replay 是本工作区 build 的链接，
本轮增量构建这些程序及 iap_current_integrity；不使用另一个工作树。

## 预期、观察与最早约束

预期：保留全部授权检查，关闭后验与引导，固定合法路线到达声明终点；然后
验证坐标/时间/真实双源，9+9，最后6次对照并有条件推广。
观察：三次历史运行均未到达 canonical `[18,0,1.5]`，只有每类首次地图。
最早证实违反：重复失败被首次导出去重挡住，最后停车缺同身份冻结输入。
这不是对停车的预设根因。历史 endpoint gen234 与末轮 gen1628 不一致。

## 阶段一

已实现：最近不可变失败保留；有界后台导出；尝试/轨迹/预算/格点身份；
合法真实起点到有限范围可行格点的完整物理连接恢复。
红/绿：`log/20261007T050106Z_396/runtime/{capture_red,capture_green2,start_red,astar_green}.log`。
历史 endpoint 原请求 START_BLOCKED，原离线工具可发现有限替代端点；
历史 timeout 离线2秒原范围找到路径，不提高在线预算。
最终Release构建、5项规划行为CTest、A*与ARAIM CTest均通过；
新固定路线尚未执行，canonical 到达尚未证明。接下来提交与现场180秒运行，
用 terminal_final 分别核对真实起点/接入/guide/实际曲线/失败阶段。

## 阶段二

已实现生产 worst_hyp CSV 格式修复，回归复现62列/表头60列并修复。
历史坏行保留。代码追踪：优化后 R(0) 对世界→ECEF 旋转自由估计，
Advisory 绑定该帧 pose 与时间；冻结参考为调用时 ROS now，存在处理延迟。
Current Monitor `gnss_src.valid` 权威决定 gnss_valid，Advisory 还检查 epoch 身份；
不能把未判定来源强行准入，来源缺失不触发独立急停。
尚缺：物理重力方向、旋转不确定性、有效时间同步/传播及精度实测、真实双源，
匹配 LiDAR 模型逐观测残差。本轮继续记录/分析不依赖规划通过的证据。

## 阶段三

真实校准0/9、独立验证0/9。核对协议 `advisory_calibration.py`：normal、
gnss_degraded、lidar_degraded；校准1101/1102/1103，验证2101/2102/2103；
map seed41021，整次运行固定退化，所有请求保留。未变更或事后挑选条件。
未满足前置：物理固定路线、阶段二契约、逐观测残差与合格误差配对。
禁止把未合格探索录制计入校准，未冻结或推广未经验证的噪声/PL参数。

## 阶段四

完整对照0/6，三对预声明seed2101/2102/2103，开关OFF/ON。
未满足前置：阶段一固定路线与基础任务、阶段三独立验证。
默认参数未推广，后验先验关闭。

## 原始证据与下一步

分析/构建resolver运行：`log/20261007T050106Z_396`。
历史正式报告原路径保持；原日志不覆盖。
本文件持续更新实测运行、提交/配置身份、尚缺条件和解锁动作。

## 现场续轮与依赖（提交0a8ff61之后）

180s 实跑 `log/20261007T051237Z_248`：guide/prior OFF；运行结束但未到
目标，末轮135/gen1648，terminal_final attempt133/gen1623，旧快照无法解释
最后停车。探索现场没有满足固定路线，26份原始预测录制不计校准。首轮
启动DDS profile路径错误导致默认传输，外部startup launch.log已登记并保留。
GLIO记录首/末约[-16.609,-.161,1.378]/[-16.618,-.159,1.393]，采集晚于
初始移动，不能据晚开始记录断言全程无发布。

修复完整freeze在预测准备后的混代，红/绿通过；OFF不用偏好刷新，保留raw
预测，红例847次无用查询；原OFF回归矛盾断言改为独立预测保持/搜索0刷新，
ON避让回归保留。4项GridMap行为测试、其余7项planner测试通过，最终构建
与37项baseline重复核验通过（fullfreeze_final_tests.log）。canonical预分配owner39项通过。

生产integrity_extension实际writer已重建（首轮旧库仍输出62列，明确失败）；
GNSS坐标动力学CSV记录旋转切空间边缘协方差、bias、速度、原状态与接收
时间，诊断不授权。canonical synthetic源实现只生成GPS，Current Monitor
整星座假设仍拒绝，GNSS不是有观测即可准入。历史陈旧2022RINEX不默认为
2026场景合法替代。阶段二仍缺物理重力核验、同步/传播实测与逐LiDAR残差。

下一步：最终提交干净版本，GPU预检，正确DDS profile和启动日志归属下
重新180s实跑；从启动前记录实际发布/命令/运动，核对literal最后失败身份。
新report须指出离线工具未重放guide拟合余量与多目标，不把基础路径当在线
同规则修复成功。9+9与6不启动于不合格前置，不推广配置。

## 第二轮与修复回归

7f1ebb4 / `log/20261007T053238Z_917`（正确DDS，启动日志归属run）180s
未到达，真实cmd/publishID1、2，184轮记录17个None/120搜索/33发布/14预算；None包括未选到目标，不能计为成功。
最终candidate attempt184随后成功，因此不代表最后失败；新增红例覆盖终态
记录被可修复中间失败覆盖，另红例复现motion连续微增导致足够走廊被丢弃。
`disposition_radius_red.log` 两项红；`disposition_radius_green.log`38项绿。
新增final_check独立epoch证明、完整attempt_idCSV、未分配候选IDnull。

同guide余量及16目标离线gen60，原起点合法，穷尽250402节点，当前已观测
池内未找到到原目标集合的通路，未知拒绝保留；旧scope不足的基础净空结果
另存，不能合并。生产writer全60列，sat_id=-1假设仍导致GNSS不可判定；
坐标CSV1739有效协方差，主轴sigma4.82–4.98°，PL当前没有传播该不确定性。
物理静止匹配310份，单源/原时间记录继续保留，9+9和6前置仍未满足。
下一步提交、GPU预检、第三次canonical实测，检查终态literal失败与发布行为。

## 第三轮 literal 最后失败与未知曲线修正

c80ccc2 / `20261007T054246Z_172`，最后attempt730/gen1575；与CSV末行同
身份，A* NONE成功，真实B-spline ENVIRONMENT_UNOBSERVED、修复配额拒绝。
固定路线与canonical仍未到达，实际前进约4.455m；不将None无目标行计成功。
未知样本缺实际guide几何约束：复用既有曲线平面目标处理未知/越界，支撑
点原物理查询检查。红例unknown_curve_red.log，green5通过；早期夹具漏接
A*query及一次缺少optimizer完整初始化的测试崩溃保留，未当生产失败。
只读提取c80原静态后端与最终夹具同输入复核：原方法false（红），新实现完整曲线合法（绿）。未切checkout/创建工作树。早期误用动态库替换不能替换静态后端，其PASS已标无效对照。归档link命令跨行误解析污染本轮build静态archive，已只删除该生成库并由CMake重建为3个正确成员，再重新链接生产节点和测试；保留错误命令及修复日志。
25项A*包括选中终点格点身份回归通过；五项规划行为、39项canonical启动回归通过；静态库修复后重链接/五项行为再核验通过。
修正无目标统计/最终取证，不使用旧搜索/guide。下一次300s森林实跑仍关闭
引导/后验，固定在线预算不变；较长任务观察窗口不授予更长单次搜索预算。
阶段二GPS整星座、时间/旋转传播/逐观测LiDAR残差前置仍缺，9+9与6保持0。

## 当前终态：第四轮与四阶段报告（3c84bcd之后）

`20261007T060542Z_204`：正确DDS、提交干净、GPU READY，300s canonical。
完整运行结束但未到18m终点，实际GLIO x前进16.422m；38个发布ID、34个
实际pos_cmd ID，最后server执行/反馈38。末CSV与terminal_final均为
attempt265/gen2398。起点/接入合法；原1s A*扩展99219后超时。
同完整16目标、100³原池、guide余量/原净空/同冻结物理代数离线穷尽226897
节点1.674s无路；最低原物理净空独立归因亦穷尽242253节点1.707s无路。
只证明此已观测离散池到保存目标无合法通路，不泛化整个森林/其他目标。
不增加在线预算、不假造未知观测，也未把仿真未验证hover算物理制动PASS。

四轮合计123请求、119完整录制，第三轮24/26、第四43/45，失败请求全保留。
第四轮四份同源/库hash重放前三份LiDAR-only有效、末份current不可用；
reference-pose差0.176–0.208s均超过0.05。旋转Up漂移最大37.508°，R(0)
边缘sigma约1.719–4.985°且未传播至PL。匹配428份原IMU/原pose的物理Up
原force角约0.768°；原样本/5s块及估计bias修正诊断另存，不授予校准。
GPU模型逐观测残差接口与合法GNSS多星座/子集准入仍缺。

校准0/9、独立验证0/9、配对任务0/6；前置未通过，未生成合格冻结配置，
未推广默认。原36m canonical路线候选带hash保存且明确未物理资格通过。
图文报告：
[forest_four_stage_final/report.md](../../log/20261007T050106Z_396/export/analysis/forest_four_stage_final/report.md)。
同目录status.json与inventory.json登记状态/原始输入/图表/配置hash及所有失败。
分析run生命周期completed仅指交付结束，不是四阶段PASS；现场safety_outcome unknown。
阶段一解锁需合法真实观测支持的完整参考通路、重复执行及物理制动证据；
阶段二解锁需经过精度实测的状态/协方差传播、物理方向与旋转不确定性、合法
GNSS来源及匹配模型逐观测残差。满足后沿原协议9+9，再6，条件推广。
