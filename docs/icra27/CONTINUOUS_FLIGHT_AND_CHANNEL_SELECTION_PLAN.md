# 正确通道选择与连续飞行修复计划

日期：2026-09-20

状态：实施中；代码完成、CPU/进程门禁和 live 验收分别记录，本文不是完成报告。

截至 2026-09-23，第一岔路纠偏代码已覆盖候选状态重置、known/unknown
证据拆分、完整最终 bundle 比较、不可比时公共前缀/保持，以及紧凑证据保留策略。
CPU/进程全量门禁、单岔路 primary/mirror 和 180 秒森林验收尚未在本状态行宣称通过；
下方交付清单只有在对应证据命令完成后才勾选。

调查基线：`605e96e7e4c17ef6591b51a0ad1abc19579289a6`。实施时重新记录 HEAD，先核对后续改动。

目标场景：`icra_dense_forest_four_fork_v2`，默认 BDS、`MISSION_BEST_EFFORT`。

## 1. 开发目标与完成边界

让无人机在存在局部安全、可跟踪、可停车的通道时，公平比较候选实际轨迹，选择符合任务风险排序的通道，并提前准备后继段、连续推进至任务终点。消除由迟到轨迹、曲线身份冲突、候选计算偏置、重复搜索和提交超时引起的非必要停车。

“连续运行、不再停止”指健康输入和可行通道条件下不再非必要地走停；不允许通过删除碰撞、跟踪、地图新鲜度或制动检查达成。任务终点正常停车、真实局部危险下减速或认证停车属于正确行为。

第一分叉右侧是场景设计的低遮挡侧，但不能把 `y<0`、场景标签或真值地图作为在线选择规则。正确选择必须来自相同输入条件下的实际曲线证据；若在线预测与场景预期不一致，必须解释差异并修复已证实的问题。

本阶段不修改 GNSS PL/AL 主公式、BDS 默认、P5 开关、1 秒地图新鲜度、现有局部净空数值，不恢复绝对 LiDAR PL 相加或任意线性 drift。不新增 RL、联合协方差体系或另一套授权器。

## 2. 已知证据与尚未证明的假设

### 2.1 证据位置

最新用户运行目录：

```text
results/icra27/dev_runs/interface_integration/run-20260920T034140Z-4145765/full-r01-risk/
  launch_command.json
  summary.json
  capture.jsonl
  stdout.log
  exports/planner_p4_risk_astar_debug.csv.forward_candidates.csv
  exports/planner_p4_risk_astar_debug.csv.forward_lineage.csv
  exports/planner_p4_risk_astar_debug.csv.execution_events.csv
  exports/planner_p4_risk_astar_debug.csv.runtime_window_batch.csv
```

对照运行：`run-20260919T183006Z-3674668`、`run-20260919T183220Z-3694377`、`run-20260919T183421Z-3713973`；最后审查后运行 `run-20260919T184944Z-3768888`。均位于上述 `interface_integration` 父目录。旧产物只读；缺失冻结输入时明确标为不可精确复放，不从零散 CSV 伪造原快照。

### 2.2 已确认

| 现象 | 证据 | 可得出的结论 |
| --- | --- | --- |
| 第一分叉预期右侧更好 | manifest 中 `low_risk_side=right`、`low_risk_y_sign=-1` | 应审计右侧为何不能稳定参与选择；不等于每个 epoch 的 PL 必然右低左高 |
| 部分决策右侧风险更低 | decision event 3：左 1.003 / 右 0.951；31：1.124 / 1.085；54：0.926 / 0.851 | 预测不是完全没有区分度，但这些是 guide 证据，不是最终曲线比较 |
| 右侧 refinement 失败 | 约 `(-12.66,-2.80,1.25)`，margin `-0.079 m`；另一搜索约在 `(-16.22,-1.32,1.27)` 超时 | 某个 guide/目标不可用或搜索未完成；没有证明整条右侧通道封闭 |
| 左侧候选反复当选 | 多轮仅左侧 refinement 完成；manager 存在 `latched_channel_candidate_ready` 覆盖选择 | 候选生成完成率与旧偏好会影响选择，未完成不能当作高风险 |
| 实际短暂运动后停止 | 最新运行位移约 `0.457 m`；trajectory 25 获得正式授权 | 不是全程从未发布运动轨迹 |
| 停车前全球风险正常 | runtime 最坏风险比约 `0.79–0.84` | 该次首次撤权不应归因于 GNSS 超预算 |
| 跟踪误差超界 | 约 0.904 s 时误差 0.164 m 排定制动；约 1.004 s 时 0.177 m 撤权 | 当前跟踪/制动包络无法覆盖实测偏差 |
| 曲线可能迟到 | 轨迹开始 `1657065619.444`，捕获曲线时附近 odometry stamp 已为 `1657065619.644` | 有约 0.20 s 时间偏移线索；捕获器接收时间不能代替控制器接收时间 |
| trajectory 26 身份冲突 | pending guard 26 是非恒定控制点曲线，收到 QUEUED/ACTIVATED；随后 normal spline 26 是六个相同控制点 | 同 ID 对应不同曲线；零速来自后续 EmergencyStop，不能说预计算制动本来就是零速 |
| 停车后难以恢复运动 | lineage 中 95 行 `geometry_commit_commit_baseline_budget_exceeded`，并有 pending/rate-limited | 还需解决发布前几何提交耗时；行数含重复阶段，不等于 95 次独立决策 |

### 2.3 必须先验证的假设

- 控制器加速度能力不足尚未证明。当前捕获的指令最大加速度约 `1.34 m/s²`、差分 jerk 约 `2.73 m/s³`，不是标定限制，也不是充分的失控证据。
- 需要区分计划开始时间过早、发布/接收队列延迟、控制器响应、里程计输出延迟及时间戳错配。
- 右侧“遮挡少”可能指天空遮挡；它不直接等价于树干净空大。需分别复放地面几何、GNSS LOS、未知 support 软衰减和时间增长。
- 旧通道覆盖选择存在代码风险，但不能把所有左侧选择都归因于它；逐轮记录覆盖前后排名。
- RViz 闪动是显示与候选生命周期问题，不能以消除闪动代替完成第二通道。

## 3. 实施规则与复用边界

1. 开始读取 `AGENTS.md`、`CONTEXT.md`、`docs/spec/conventions.md`、`docs/spec/talk_spec.md` 和相关 PDF；运行 `git status --short --branch`，保存本任务基准 HEAD。
2. 源码修改仅限本仓库。保留用户 RViz、`path_searching` 和未跟踪文档/CSV；如确需修改有用户改动的同一文件，先分析 diff，仅局部合并，不覆盖。不要改 `src/glim`。
3. 每个修复先建立实际生产 seam 的失败测试，再实现。日志归因测试不能代替控制/发布链路测试。
4. 优先扩展现有 `P4PreparedSuccessorBundle`、执行证书、固定窗口、单槽 worker、LocalClearanceEvaluator 和 traj_server 握手；避免增加职责重叠的状态机和线程。
5. 调查、标定和仿真真值仅用于离线评估。在线授权只能使用生产输入。
6. 每个完成的逻辑单元单独提交，显式 stage 文件；更新相应契约和 `docs/CHANGES.md`。本文所述未来行为不视为现有实现已满足。

## 4. 推荐实施顺序

```text
S0 固定复放与时间/身份诊断
 → S1 唯一曲线身份、未来开始时间和控制端握手
 → S2 控制响应标定、轨迹可跟踪性与恢复制动
 → S3 右侧净空/refinement、缓存及公平调度
 → S4 多通道实际 B-spline 认证与选择、去除偏置锁定
 → S5 后继完整预认证、几何提交性能与滚动衔接
 → S6 RViz/runner 证据、CPU 与 live 闭环验收
```

S3 的只读复放可提前进行。先稳定 S1/S2，再用 live 飞行评价选路，避免控制问题掩盖路线问题。每阶段均产出可独立运行的测试与简短结果表。

## 5. S0：闭合可重复证据链

### 修改

- 增加仓库内离线分析入口，消费上述 capture/CSV，按执行实例、轨迹 ID、曲线 hash 和时间戳关联，而非只按 ID。
- 导出唯一决策级的左右通道表：guide、refined、final-spline、选中与覆盖原因；同时记录未完成原因和阶段耗时。
- 时间链记录 ROS/仿真时间和 steady duration：状态测量 → 曲线准备完成 → 最终认证起止 → 发布 → traj_server 接收/排队 → 激活 → 首个 PositionCommand → 控制器接收 → 控制输出 → odometry。
- ROS 时间用于物理到达和时序一致性，steady 时间用于耗时；仿真暂停不被当作车辆滞后。跨进程 steady 差只在同一主机、同一时钟基准下使用。
- 检查控制器真正订阅的 odom 与规划器 odom 的 frame、参考点、stamp 和变换；核对 command 前馈加速度、质量、重力、增益、饱和和实际调用频率。发现具体实现错误才修，不先调增益掩盖。
- 时间对齐后分别报告沿轨迹滞后、横向误差、垂直误差，不能把“找到最近曲线点”后的横向小误差用来替代按时间的真实跟踪误差。

### 测试与产物

- 一个命令复现基线：已授权 25 → tracking 越界 → guard 26 激活 → 同 ID 的另一曲线覆盖；输出发生时间和 hash。
- 相同 ID 不同曲线、延迟到达、重复/乱序消息、仿真暂停均能被分析器区分。
- 无控制端接收证据时延迟报告为“不确定”，不把 capture 接收延迟全算成控制延迟。
- 新建冻结裁剪复放证据，包含精确 occupancy、来源、净空策略、GNSS epoch 和坐标契约；旧证据不足则重新捕获等价案例，单独标识。

## 6. S1：修复时间起点与曲线身份

### 6.1 唯一身份

- nominal、successor、pending guard、recovering brake、emergency 全部从同一分配入口预留唯一 trajectory ID；被取消的 ID 也不回收复用。
- 控制点、节点向量、开始时间或曲线内容变化，必须获得新曲线身份；snapshot 重认证只更新证据，不改变物理曲线身份。
- 控制端拒绝“相同执行实例/ID、不同 hash”的消息；重复同内容消息幂等。进程重启明确切换执行实例并废弃旧 pending。
- QUEUED/ACTIVATED/CANCELED/REJECTED 至少绑定 ID、curve hash、start time 和父身份，不能只凭 `ACTIVATED:26` 判定当前控制对象。
- 执行证据与控制端实际 active hash 不匹配时先记录真实控制对象及原因，避免错误地把另一个曲线当成原制动已完成。

### 6.2 未来开始与原子切换

- 初始曲线准备完成后选择未来开始时间，提前量覆盖最新认证、发送、接收及调度延迟的实测上界与明确余量；不要直接固定为这次捕获到的 0.20 s。
- 用该开始时间重新计算所有绝对到达时间、GNSS 风险和窗口证据；若错过开始条件则取消整个待提交对象，重新准备新身份，不能回填过去的时间。
- `PREPARED → QUEUED → ACTIVATED` 复用现有握手。开始前控制端保持旧有效轨迹/已有悬停，不提前执行 child；manager 只有收到匹配激活证据才切换 active certificate。
- 移动中的后继沿用父曲线已有固定锚点；全部准备和最终快速复核在切换前完成。迟到不跳过 child 的开头追赶，不延长父曲线终点或截止时间。
- 区分“执行已准时发生但确认消息晚到”和“控制命令真正晚执行”。确认晚到可根据已记录的实际激活时刻核验；不能由当前 callback 时间推测激活。
- 固定窗口只能在最终曲线/开始时间确定后建立一次。准备阶段若重新定时，旧证据全部失效；提交后 runtime 只推进现有窗口。

### 测试

- 注入 0/50/200/300 ms 发布延迟，验证只允许满足握手与认证期限的曲线激活，迟到曲线不追赶。
- pending 或消息重复时父 ID、开始时间、端点不变。
- guard 激活后 emergency 必须使用新 ID；旧 ACK 不能认证新曲线。
- 提交、P5、控制端和 PositionCommand 引用同一内容/时间身份。

## 7. S2：把控制能力用于生成与制动

### 7.1 有依据的控制标定

先完成 S1 并排除 frame/stamp/前馈错误，然后在开阔仿真环境进行速度、加速度、jerk 和转弯组合的有限扫描。包括从静止启动、非零初速切换、左右转弯、连续后继和减速停车。记录指令/反馈 p/v/a、有效采样率、延迟、饱和与误差。

选择满足现有跟踪包络且保有余量的能力配置，并用独立轨迹验证。不要用本次 1 秒片段推导部署级上界，不把观测分位数冒充严格概率保证。标定报告必须绑定控制器/仿真模型、质量/增益、更新率、算法版本和适用速度范围。

默认采用最简单的能力配置：分轴速度/加速度/jerk 限制、实测延迟上界、验证过的跟踪误差界。仅当固定界无法解释速度变化时再引入有依据的速度相关误差模型；禁止新增无标定的每秒 drift。

### 7.2 生成阶段约束

- nominal、successor、brake 共用一份能力配置。起点状态连续、批准终点、终端零速度/零加速度维持硬约束。
- 从真实 B-spline 导数检查速度、加速度和 jerk；结合控制点导数界或逐 span 极值检查，不能仅用 0.2 s 采样漏掉峰值。
- 三次 B-spline 保证 p/v/a 连续；jerk 在节点处可有跳变，要分别检查两侧界，不声称“三阶连续”。无必要不升级曲线阶数。
- 优先修改时间分配/生成器约束来满足能力，非零起始速度时重新求解边界条件，不能盲目整体拉伸破坏起点速度。
- 重新定时后重算实际到达风险、净空、所有制动和 P5。延长时间可能增加 GNSS 暴露，必须纳入实际曲线排序。
- 同一 LocalClearanceEvaluator 消费统一跟踪包络，避免既扣完整 tracking bound，又把同一延迟误差重复加一遍。

### 7.3 在包络耗尽前制动

- 增加由观测误差、误差增长、处理/切换延迟和已验证制动能力计算的剩余可控余量。触发点应早于制动证书有效域被耗尽；不得以任意降低/提高 0.15 m 门限代替分析。
- 正常预计算制动从名义锚点 p/v/a 出发，其证书还必须覆盖允许的实测位置与速度偏差；不能只检查位置接近。
- 若实际状态超出名义制动证书适用域但仍局部可控，按切换时刻的估计 p/v/a 生成有限时长恢复制动；必须包含估计至切换期间的运动、状态不确定性和命令过渡段，并在原批准通道/停车边界内验证。
- “从实际速度开始”不能制造另一种指令不连续：实测状态与原命令有差异时，要验证连接过渡和制动整段；使用可用 IMU/状态估计约束加速度，不把噪声差分当精确初始加速度。
- 新恢复制动只有在时限内、冻结碰撞/净空/动力学/本任务模式条件均通过后才能激活。数据断流时不能凭无效新地图重新认证；只能使用仍适用于当前状态的预认证方案。
- 无可认证动作或近迫真实碰撞时保留 emergency 并明确记录。禁止把恒定位置曲线称作物理连续的认证停车，也不得让普通重规划反复发布 emergency 重置轨迹。

### 测试

- 延迟修正前后分别量化跟踪误差，再比较加速度/jerk 约束贡献。
- 非零初速和弯曲路径中名义/实测偏差覆盖正确，制动起点与终点连续性及边界正确。
- 在有效域即将耗尽前启动制动；超出有效域时不能继续声称旧证书有效。
- 重现本次 guard 26 激活后被另一条 26 覆盖的场景，要求唯一身份及正确控制交接。

## 8. S3：修复右侧路径并缓存各通道结果

### 8.1 区分坏路径与坏通道

- 对 `(-12.66,-2.80,1.25)` 周围输出 raw occupancy、障碍 AABB、required envelope、planning buffer、最近障碍、escape direction、走廊边界和搜索池边界。
- 核对 margin 是否已经扣过 0.05 m planning buffer，避免把“最终安全余量”和“生成额外余量”混淆。
- 对右侧通道做确定性横向/局部走廊扫描，寻找同一通道内满足统一净空的中心路径。不要只沿坏 guide 回退，也不要把某个中间 waypoint 当成必须抵达的任务点。
- 中间点受阻时，先在同一通道的连通自由区域中选可行 waypoint；必要时重新运行局部 A* 并验证整段。终点后缀回退保持有意义的进展与停车条件。
- 保留 sentinel/搜索池边界回归；`BUDGET_EXHAUSTED` 是未完成，不得转译为 `CLOSED`。碰撞封闭只有充分的冻结证据才能宣称，有限搜索失败应保留不确定性。
- 左侧有天空遮挡不等于飞行高度存在实体碰撞；两类证据分别记录。右侧天空开阔也不能豁免局部净空。

### 8.2 缓存与公平调度

- 复用现有 worker，维护最多现有 `max_channels=4` 个通道槽位，稳定 ID 来自拓扑/走廊身份，不取决于本轮输出索引。
- 每个槽位保存 guide、已完成 refinement、最终曲线/bundle、失败原因、依赖的走廊区域和内容身份。计算进度与安全证据状态分开记录。
- 新地图仅使相关区域的几何缓存失效；GNSS 新 epoch 只使风险证据待更新。GNSS 变化不重启无关 A*，相同 snapshot ID 也不能绕过 freshness。
- 首轮给每个通道有限计算机会，后续对未完成通道续算或从已存路径 warm-start；不得让先成功的左侧无限占用其他通道的份额。实现不支持 A* 续搜时，至少复用已完成分段，不宣称恢复了未保存搜索状态。
- 每轮仍遵守现有 500 ms route 工作预算；单次 direct batch 不超过 150 ms；多曲线工作在截止时间前分批完成，运行时 watchdog 优先，不以扩大单次查询预算解决。
- 不能在每代重置未完成任务造成饥饿。若截至决策期限仍有通道未完成，记录 `PARTIAL_COMPARISON`；只允许由已认证证据支持的有限推进，不能宣称已完成所有通道的最优比较。

## 9. S4：所有可行通道比较实际曲线

### 9.1 统一候选认证入口

抽取并复用现有 prepare-only 流程：输入冻结起点状态、计划切换时间、channel guide、执行快照和任务策略；输出最终停止 B-spline、真实制动库、固定窗口、局部/动力学/GNSS 暴露结果及 typed failure。

该接口不得修改当前 `LocalTrajData`、父证书、任务 exposure episode、P5 debounce 或实际发布状态。候选模拟使用 episode 的只读副本；最后只安装被选中的 bundle。最多每通道保留一个准备好的最佳版本，避免无界 fanout。

### 9.2 可比条件

- 对同一分叉优先使用相同起点/切换锚点、冻结地图、GNSS epoch 和可比较的下游任务 station；每条曲线按自身真实到达时间预测。
- 路径更长导致更晚到达是实际代价，不应强制相同时间；但不能把被裁短的左侧曲线与完整右侧曲线当作同等任务完成度。
- 裁短候选只获得有限执行偏好，报告实际终点和未评价后缀。没有公共可观测终点时，使用共同的局部任务范围比较，报告远期未知。
- 缓存曲线可来自不同准备轮次，最终排名必须对同一最新可用冻结快照重认证；不能比较不同 epoch 的陈旧 PL。若仅来得及复核选中对象，声明比较不完整。
- 当前地图看不到远端时，未知天空只按已有 best-effort 策略计算；地图本身的局部运动 support 缺失仍是硬失败。

### 9.3 排序与旧偏好

- 先排除局部碰撞、净空、动力学、跟踪/制动、local freshness 的硬失败。
- 继续采用既有任务模式分组：预算内 → 预算外但 GNSS 可比 → 全球证据不完整；组内按本任务的峰值、0.5 s 最坏区段、连续暴露、积分、恢复时间、净空、净推进和稳定 hash 排序。未知组沿用 support/几何/恢复的明确排序，不将 NaN 当零。
- 保留完整分解及 winner/runner-up；相同物理曲线重复窗口不重复累计暴露。
- 删除 `latched_channel_candidate_ready` 无条件覆盖赢家的路径。当前轨迹的执行保护与下一段路线偏好是两件事。
- 第一版只在风险排序真正相同的 tie 中偏好旧通道；若以后引入显著改善迟滞，参数要有测试和明确量纲，不能掩盖组别变化、硬风险或明显更优的新通道。
- 选中另一通道也只能通过已认证连接在可切换区域切换；已经进入不可横穿的分叉时，不允许因为排序变化横跨障碍。

### 9.4 验收

- 冻结两侧实际 B-spline：右侧风险更好且局部可行时选择右侧；镜像/交换证据后相应选择另一侧，证明没有左右硬编码。
- guide 安全但最终曲线不安全时淘汰；另一条最终曲线安全时可以胜出。
- 右侧计算慢但更安全时仍获得完整评估；超时报告 pending，不记录成危险路线。
- 若森林右侧仍未胜出，必须给出两条最终曲线、相同输入身份及完整 PL/unknown/sigma/几何/时间分解；不能仅以“右侧理论上更安全”改 PL 或阈值。

## 10. S5：完整后继提前完成与发布瓶颈

- 无论当前执行是 LIMITED_PREFIX、正常正式曲线还是 mission-degraded 短段，都建立后继准备任务。不能仅修 LIMITED_PREFIX 分支，正式曲线依然到端点才规划。
- 沿用截止时间调度：准备上界、认证、最新重认证、传输与切换余量均在锚点之前预留；基于实测阶段耗时检查原 1.5 s 提前量是否足够，不能用平均值证明 WCET。
- 父轨迹执行期间完成最终控制点、节点、制动库、固定窗口和 P5 preview；发布前仅做同一缓存曲线的最新证据重认证和必要碰撞增量检查。
- 任一处理过程不得阻塞控制消息发送和 execution watchdog；保持单槽/latest-wins、同父 single-flight，并对父已失效/过截止时间协作取消。
- 对 `geometry_commit_commit_baseline_budget_exceeded` 分解：占用扫描、走廊构建、内容 hash、delta 合并、碰撞查询和重复认证。缓存同一几何内容/曲线的提交基线，使用空间索引；历史增量缺口时在预算内做最新冻结全量走廊复核，而不是假定无变化。
- 几何预处理缓存不等于授权缓存；新鲜度、内容变化和最终实际曲线身份仍验证。不能用旧基线跳过新障碍。
- 当前段通过验证但后继 pending 时继续原段；没有合格后继时可正常停车，但必须报告哪条通道在哪个阶段失败，不能把 pending 当作“无路”。
- 后继在固定时刻由控制端切换，p/v/a 连续；不得在切换 callback 中才开始优化或完整构建制动库。

## 11. S6：可视化、runner 与诊断

- 修改 `safety_rviz_publisher.cpp`，按稳定 channel ID 更新 Marker；避免每个 pending 通知 DELETEALL。保留几何缓存的路径，但必须标明证据 age 和状态，过期结果不能继续显示为已批准。
- 区分 `PENDING`、`GEOMETRY_READY`、`CURVE_READY`、`CERTIFIED`、`REJECTED`、`ACTIVE`；它们是诊断状态，复用现有实际状态，不新建另一套运动许可。
- 显示左右坐标方向、guide/实际 B-spline 类型、失败点/障碍、所需净空、计划/实际激活时刻和跟踪误差。只有匹配 active hash 的曲线标为正在执行。
- runner 保留现有 `limited-prefix` 严格语义；另增独立 `--stage continuous-flight`（待实现）验收混合正式/短段/降级的连续运行，不能再把正式路线成功误计为这个新阶段失败。
- 新阶段必须验证实际 PositionCommand、odom 位移、切换前后身份、无中间停车、最后到达目标或合法终态；不要仅凭 `RISK_SELECTED` 或 launch exit 0 通过。
- `--rviz` 当前仅支持 `--stage full`；新阶段若需要 RViz，显式扩展 parser、launch 和测试再提供命令。不要给用户一个当前不接受的组合。
- 分开显示闭环进展失败与性能告警。原 local-map current 的 10 ms 告警继续记录，不能为通过而删除；也不能把该告警直接解释为 GNSS 或执行失败。

## 12. 文件入口（实际修改前重新定位符号）

| 工作 | 主要入口 |
| --- | --- |
| 选择、latch、准备、提交、运行时 | `src/iap/planner/plan_manage/src/planner_manager.cpp` 与对应头文件 |
| FSM、发布、EmergencyStop 调用 | `src/iap/planner/plan_manage/src/ego_replan_fsm.cpp` |
| 控制端激活、pending guard | `src/iap/planner/plan_manage/src/traj_server.cpp` |
| 路线排序、refinement、worker | `src/iap/planner/bspline_opt/src/p4_forward_route.cpp`、`bspline_optimizer.cpp` |
| 硬终端及导数 | `src/iap/planner/bspline_opt` 下实际 UniformBspline 实现；用 `rg` 定位，勿另写一套求解器 |
| 净空与任务暴露 | `src/iap/planner/trajectory_assurance.cpp`、`include/iap/planner/trajectory_assurance.hpp` |
| 固定窗口、P5 | `plan_manage/src/p4_execution_risk_window.cpp`、`p5_runtime_integrity_gate.cpp` |
| 实际控制/仿真能力 | `src/uav_simulator/so3_control/src/so3_control_component.cpp`、`SO3Control.cpp`、`src/uav_simulator/so3_quadrotor_simulator/` |
| 可视化与验收 | `plan_manage/src/safety_rviz_publisher.cpp`、`scripts/dev_planner/run_icra_interface_integration.py` |
| 参数与契约 | `launch/test_planner.launch.py`、`plan_manage/launch/advanced_param.launch.py`、两份 spec、`docs/CHANGES.md` |

## 13. 测试矩阵与量化验收

### 13.1 CPU / 冻结复放

至少覆盖：唯一 ID/不同 hash 拒绝；按真实时刻激活；延迟 ACK 与迟到命令区分；实际非零状态制动；jerk 两侧界；右侧坏 waypoint 修复；受阻/超时原因；缓存局部失效；调度公平；实际曲线排名；P5 preview 无副作用；snapshot 更新不重置父时间；新障碍/数据断流/strict 超限仍正确停车。

相关测试：Predictor、`test_trajectory_assurance`、`test_p4_forward_route`、`test_p4_risk_astar`、`test_planning_risk_context`、`test_p4_execution_risk_window`、`test_p0_risk_grid_runtime`、`test_p5_runtime_integrity_gate`、runner/launch；新增 traj_server 握手生产路径测试。完成后运行 IAP、bspline_opt、plan_env、ego_planner CTest，既有无关 lint 失败单独报告。

冻结 16 秒 BDS 窗口负载继续统计 direct batch p50/p95/max；目标 p95 <75 ms，每个可授权批次 <150 ms，超预算结果不得授权。多候选总 wall time 独立记录，不能把四个 100 ms 的查询宣称整轮 100 ms。

### 13.2 Live 验证分层

GPU preflight 必须先通过 `nvidia-smi` 和 CUDA `cuInit(0)`、device count >=1；失败则完成 CPU 验证并明确缺失 live。

1. 开阔区：验证准时启动、跟踪和非零速度认证制动，再验证连续后继。比较修时间前后，再比较能力约束前后，避免同时改所有变量后无法归因。
2. 单分叉冻结/在线场景：两侧都能完成实际曲线比较，确认右侧更优时确实沿负 Y 通道前进，镜像测试能反向。
3. 森林 v2：默认 BDS/best-effort 运行 3 次健康开发 smoke；另做有明确注入的断流/障碍测试及一次 strict 超限对照，产物独立保存。

健康森林验收目标：

- 3 次均实际通过第一分叉，至少每次完成 2 次有证据的无停顿后继切换；持续任务的完成目标是从 `(-18,0,1.5)` 到 `(18,0,1.5)`，通过四个分叉后正常悬停。
- 若观察窗口不足以抵达终点，新 continuous-flight stage 使用明确有上限的任务时长（初始开发上限 180 s），记录超时为未完成；这是实验时长，不改变预测时长或地图有效期。
- 非终点、非故障注入阶段不存在速度 <0.05 m/s 持续 >0.5 s 的停顿；结合有效命令与定位噪声核验，不将初始准备、终点悬停计入。
- 零次“迟到曲线跳过起步”、零次不同曲线复用身份、零次因新 generation 单独重置轨迹；同轨迹 window layout hash 不变。
- 所有后继在锚点之前达到 `PREPARED_CERTIFIED`；切换处 p/v/a 连续并满足测定跟踪包络；不在切换时执行 A*/优化。
- 右侧理论更好但未选时，必须有完整同快照实际曲线对照；仅 guide 比较、仅一个候选完成或沿用 latch，不算正确选路验收通过。
- 本次出现的 `geometry_commit_commit_baseline_budget_exceeded`、guard 身份冲突和捕获到的起步追赶问题在健康验收中归零。
- 若仍停止，按命令时序、控制响应、状态估计、局部几何、制动、输入失效、计算未完成、任务模式列出首因，附证据；不能将新的阻塞描述为整套系统已经修好。

已有可用的观察命令（用于基线，不能替代上述新验收）：

```bash
cd /home/dev/ws_iap/src/iap
source /home/dev/ws_iap/install/setup.bash
python3 scripts/dev_planner/run_icra_interface_integration.py \
  --stage full \
  --scenario icra_dense_forest_four_fork_v2 \
  --repetitions 1 \
  --rviz
```

实现新 stage 并补 CLI 测试后，开发验收命令应为：

```bash
python3 scripts/dev_planner/run_icra_interface_integration.py \
  --stage continuous-flight \
  --scenario icra_dense_forest_four_fork_v2 \
  --repetitions 3
```

### 13.3 最终报告表

| 指标 | 基线 | 修改后每次运行 | 验收解释 |
| --- | --- | --- | --- |
| 轨迹实际接收相对开始时间 | capture 线索约 +0.20 s，非控制端确证 | 控制端真实分布 | 准时排队/激活；迟到不追赶 |
| 跟踪误差、沿向/横向/垂向 | 约 0.164/0.177 m 触发保护 | p50/p95/max、误差增长 | 绑定标定能力和制动可用域 |
| 左右通道完成率/排名 | 右侧反复受阻/超时 | 每通道各阶段耗时与状态 | 未完成不得伪装成危险 |
| 实际曲线任务风险 | 原来只有选中曲线最终认证 | 两侧完整分解 | 同条件下有证据选优 |
| 唯一身份冲突 | guard/emergency 同 ID 26 | 数量 | 必须为 0 |
| 发布/几何提交超时 | 多行 baseline budget exceeded | 唯一事件数与阶段耗时 | 不再造成健康场景永久 HOLD |
| 实际位移/分叉/目标 | 最新约 0.457 m | 路径长度、净推进、通过分叉数 | 不能用 Marker 或发布次数替代 |
| 后继切换与停顿 | 未持续通过第一分叉 | 切换数、端点等待、停顿时间 | 健康场景连续执行至目标 |
| direct/准备/重认证耗时 | 旧 runtime 约 30 ms 量级 | p50/p95/max、timeout | 分开单次预算与整轮截止时间 |
| 停车首因与模式 | 跟踪/制动保护链 | NORMAL/DEGRADED/STOP 原因 | 真实故障停止仍必须有效 |

## 14. 交付清单与 Codex 开工指令

- [ ] S0：基线复放命令、冻结失败证据及时间/身份诊断。
- [ ] S1：曲线 ID 分配、未来开始、ACK 和原子切换，生产链路回归。
- [ ] S2：标定报告、统一能力参数、生成约束和实际状态恢复制动。
- [ ] S3：右侧 refinement 修复、各通道缓存与公平计算。
- [ ] S4：多通道最终 B-spline 排名，移除无条件旧偏好覆盖。
- [ ] S5：正式/短段共用后继链路，准备在前、重认证在前、准时切换。
- [ ] S6：RViz、continuous-flight runner、CPU 与 live 数据报告。
- [ ] 契约同步、任务范围 diff 检查、Standards/Spec 两个维度审查及分逻辑提交。

建议给开发 Codex 的指令：

> 读取本文件并依 S0–S6 实施。先核对当前 HEAD 与工作区，复用已有实现，不重复搭建授权框架。先复放证据、补生产链路失败测试，再修复；按阶段完成构建、测试和提交。重点先解决实际执行时刻、曲线身份、右侧候选可行性及公平比较。不能通过放宽风险/净空/新鲜度/跟踪门限实现连续飞行。完成后用实际命令、里程计、控制端确认和终态证明正确选路及连续衔接；任何未实现项、未达标指标和新问题必须明确列出。
