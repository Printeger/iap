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
未到达，真实cmd/publishID1、2，184轮内部结果17成功/120搜索/33发布/14预算。
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
