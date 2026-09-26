# MISSION 连续飞行修复开发计划

日期：2026-09-26。状态：待实施；本文是开发任务，不是完成报告。

依据：[对话记录](../whatsnext.md)、当前代码 `f15cb65fa376c3d4823b7249d193bcf5d8fbc07f`、2026-09-26 05:00:49 运行的 execution events。开工时重新核对 HEAD，后续已有修复必须复用。

## 1. 要交付什么

在 `icra_dense_forest_four_fork_v2`、BDS、`mission_best_effort` 下，只要局部运动仍安全，就选择风险较低的可行路线，连续通过四个分叉，从 `(-18,0,1.5)` 到达 `(18,0,1.5)` 并正常悬停。不能再因为 GNSS 累计额度、重复认证或下一段准备不完整而反复走停。

**代码和 CPU 测试通过只是进入 live 的条件。最终是否解决，以实际控制命令、控制反馈、运动轨迹和任务终态为准。** 本次目标是闭合该仿真场景和下文故障测试，不宣称真机或任意环境已获安全保证。

用户已经确认三项方向：

1. MISSION 下，GNSS exposure 保留统计和选路价值，取消其独立否决运动的权力。
2. 所有失败必须保留真实类别和首因，外层不能覆盖成笼统失败。
3. P4 统一执行前认证；P5 缩成运行时监督，只根据新的事实撤销执行权限。

为限制改动，本次保留现有 `STRICT_GLOBAL` 参数和严格绝对定位语义，不开发新的模式。两种模式共用同一流程、同一策略解释；“保留双模式”是兼容性实施选择，不是新增用户需求。

## 2. 现状与已知阻塞

当前流程是：地图/定位 → 通道初排 → 实际 B-spline 和制动曲线 → 认证与候选比较 → 发布/激活 → 准备下一段 → 运行时监督。

已有能力应直接复用：`LocalMotionAssurance` / `LocalClearanceEvaluator`、actual certifier、完整候选 bundle 比较、后继缓存、曲线身份、QUEUED/ACTIVATED 握手、固定风险窗口和现有 worker。`continuous-flight` stage、180 秒时长及 `--rviz` 支持也已经存在。旧计划中的“待新增”描述不能当成代码现状。

| 已确认事实 | 本次处理 |
| --- | --- |
| `6043f28` 的独立局部图实现已被 `f15cb65` 回滚 | 不恢复第二套因子图，不重做估计系统 |
| 轨迹 5 的后继失败只留下 `successor_curve_preparation_failed:rebound_replan_started` | 补全优化器到 FSM 的真实失败原因，再针对原因修复；不能猜成碰撞或动力学失败 |
| 停车后另行生成的轨迹 9，预测积分约 `0.399975655`，运行时加上历史约 `0.003279691`，超过 `0.4` 后制动 | 这是另一条轨迹；删除 MISSION 预算否决，不再修补“余额计算” |
| exposure 同时参与生成时裁短、最终认证、发布验证和运行时制动 | 必须贯通清理，不能只改 P5 文件 |
| 后继失败函数将本次准备置为终态并清空候选；常规候选已有切换下一通道能力 | 审查并复用现有调度，避免一次候选失败就放弃本轮所有可行后继 |
| live analyzer 已有命令/反馈/odom 检查，但用不同激活数量估计后继次数，部分 full 检查仅作诊断 | 防止把 guard 激活算作成功接棒；补齐任务通过所必需的检查 |

原始证据：`results/icra27/dev_runs/interface_integration/run-20260926T050049Z-450628/full-r01-risk/exports/planner_p4_risk_astar_debug.csv.execution_events.csv`。这些是历史事实，不证明新运行必然同因失败；输入缺失时不能声称已精确复放。

## 3. 边界与禁止事项

- 遵守 `AGENTS.md`：源码只改本仓库，不改 `src/glim` 或其他仓库。保留用户改动和未跟踪文件，尤其是 `docs/whatsnext.md` 的原始对话。
- 不新增因子图、定位器、通用策略框架、第二套授权器、额外执行协议或无界重试队列。不引入 RL，不新增本任务不需要的地理围栏等功能。
- 不修改 PL/AL 主公式，不放宽碰撞、净空、跟踪、新鲜度、动力学或制动标准，不关闭 P5 来制造成功。不把旧额度改成极大值来伪装删除预算。
- GNSS 不完整与局部地图/定位不完整必须分开。unknown 不等于已知障碍，也不等于已知安全；不能把所有 `complete=false`、超时或身份错误统一豁免。
- 不把 ICP 健康、LiDAR FIM、绝对 LiDAR PL 或仿真真值直接冒充局部误差界；不加无标定的每秒漂移项。
- 不因更换轨迹 ID 清零风险统计，不为失败曲线换标签后继续授权。不用公共前缀掩盖选路一直未完成。
- 仅在 live 首因证明需要时，局部修复控制时序、候选搜索、缓存或计算瓶颈；不顺手重构整个 planner，不重做旧 S0–S6 全部内容。

旧规范目前仍要求 MISSION 硬 exposure 预算，与本次确认的方向冲突。实施时必须同步 `docs/spec/conventions.md`、`docs/spec/talk_spec.md` 和 `docs/CHANGES.md` 的相关条款。受保护 PDF 保留为来源，不改写它；明确记录本次政策变化，不能为了保持旧测试而恢复旧否决。

## 4. 修改后的唯一流程

```text
冻结本轮地图、定位、GNSS 和计划切换起点
→ 枚举通道，用风险和进展安排求解顺序
→ 在候选通道生成有限长度、可终端停车的实际曲线与制动曲线
→ 用同一个认证入口检查适用硬条件
→ 对需要比较的候选，用同一快照、相当任务范围比较实际曲线
→ 发布合格赢家，控制端按约定时刻激活
→ 飞行中提前准备下一段，完成后连续切换
```

guide 只是求解参考，不能代替实际曲线拿到执行许可。候选失败只否定这次求解；按已有排序尝试其他候选。比较未完成就如实报告，不能宣称全局最优。旧通道只在真正并列时作为偏好，不得跨障碍切换。

| 条件 | MISSION | STRICT |
| --- | --- | --- |
| 碰撞、净空、有效定位/地图、跟踪、动力学、制动、身份及时序 | 必须满足 | 必须满足 |
| GNSS 风险超出旧 peak/duration/integral 限值，但局部运动检查通过 | 可继续；影响选路及重规划，不单独制动 | 按原严格绝对定位要求处理 |
| 仅 GNSS 证据缺失或过期，局部证据仍有效 | 明确标记 GNSS unknown/degraded，按缺失证据的既有排序规则处理 | 按原严格要求处理 |
| 局部证据失效或执行偏离安全包络 | 使用仍适用的认证制动；无可认证动作时保留 emergency | 同左 |
| 下一段尚未准备好 | 原段仍安全就继续；到安全终点前仍无合格后继则停车 | 同左 |

P5 的人话职责：继续、请求重规划、请求制动。复用现有 action 类型即可，不为这三个词再建状态机。GNSS 风险变化可触发有节制的重规划，但不能导致每帧取消后继、重置父轨迹或长期占满求解器。

“只根据新事实”包括时间流逝造成数据过期、反馈断流、实测误差增长、新障碍及 STRICT 所需的新完整性事实；不是只比较 snapshot ID，也不是跳过持续安全检查。只复核受影响的剩余轨迹和仍可达制动段，避免反复重做全部认证。

## 5. 按这个顺序开发

每一步都先读实际调用路径，再补能复现问题的相关测试、实现最小修复、构建并提交。不要只在 mock 结果上测试，不能把开始状态字符串当成失败原因。

### 步骤 1：把失败看清，并准备可信的 live 验收

1. 记录 HEAD、工作区、有效参数及 build/install 来源。核对原始事件，将“后继失败”与“停车后重规划被制动”分开。
2. 补全优化器、refinement、manager、FSM 的失败传递。已有 failure enum 能表达就复用；确实缺少求解器失败类别时最小扩展，不把所有 `false` 归为 dynamics。
3. 失败记录包含阶段、类别、原始 detail、父/候选身份、快照和时间；有失败点、净空、导数或超时时长就带上。曲线尚未生成时明确无曲线身份，不伪造 hash。
4. 清掉 `rebound_replan_started` 等初始化占位值作为最终 cause 的路径。已有“保留 first typed failure”代码不等于底层已经提供首因，要覆盖真实 return-false 分支。
5. 扩展现有 `continuous-flight` analyzer：guard/emergency 不计入后继；后继必须有父子关系、匹配身份、实际非零速度切换和连续命令。增加数据断流、提前进程退出、终点持续悬停及实际通过分叉的判定。

交付：能用现有事件说明两次停车，新增生产路径测试能区分净空、求解失败、超时和边界条件失败；失败的 live 不会被 analyzer 判成通过。

### 步骤 2：核对现有局部安全依据，固定策略

1. 沿 `LocalMotionEvidence` 的真实生产路径，检查注册地图来源、源帧健康、odom/控制反馈的新鲜度和 nominal/braking 的覆盖。复用 `LocalMotionAssurance`，不要新增“local-navigation 授权系统”。
2. 核对实际 launch 的跟踪包络和 surface bound。当前 launch 存在 `0.15 m` 跟踪界、`0.02 m` surface 默认及 `uncalibrated_default_v1`；默认值和名字本身不构成标定证明。查找已有证据，缺失则用现有仿真记录做有界检查并报告适用范围，不能把默认 ID 改名当作完成标定。
3. 若局部输入接线、首帧或健康关联有具体错误，在现有生产者/消费者上局部修复；不得通过新求解器解决。必须覆盖真实首帧初始化，避免再次出现“测试结果有效，实际生产不出结果”。
4. 将第 4 节策略落实到现有 `TrajectoryAssurance` 的单一判断处；同步相关规范。STRICT 保持原严格要求，不扩展新用途。

交付：明确每个硬条件的现有数据来源、失败行为和适用范围。若必要局部安全依据确实缺失，不能通过删 gate 绕过，也不能宣称任务已完成；指出具体缺口，继续完成其余有界工作。

### 步骤 3：贯通删除 MISSION exposure 的授权权力

这是同一个逻辑改动，不能留下“生成允许、发布拒绝”或“P4 允许、激活后制动”的中间版本作为完成版本。

| 路径 | 必须修改的行为 |
| --- | --- |
| 生成器 | 移除为了剩余 exposure 时间而反复裁短终点的生产路径及仅为它服务的配置拒绝；保留地图可见范围、终端停车和动力学限制 |
| 实际曲线认证 | MISSION 的 peak/duration/integral/prior episode 不再参与 authorized 判定；局部条件、制动和完整身份仍独立检查 |
| 发布、后继缓存、最新证据复核 | 不再因 `within_budget`、`budget_exhausted` 或旧 episode 拒绝合格 MISSION 曲线 |
| 运行时 manager/P5 | 删除 MISSION 累计额度制动和同一份证据的重复 GNSS 否决；保留新局部事实的安全检查 |
| 风险统计和选路 | 保留峰值、超限时长、积分、unknown、恢复趋势及实际推进；统计不再叫可消费余额，不能冒充绝对定位合格 |

逐项审计 `within_budget`、`budget_exhausted`、`prior_global_episode`、`fitP4TerminalStopToExposureDuration`、`PEAK_RATIO`、`EXCESS_INTEGRAL`、`CONTINUOUS_DURATION`、`PRIOR_EPISODE_EXHAUSTED` 的调用者。按职责删除/保留，不做全局字符串替换。

GNSS 不完整保持 unknown 与缺失原因，不能为了通过 `complete` 检查伪造有限 PL。区分 GNSS 查询失败和局部碰撞/新鲜度/身份失败；后者仍是硬失败。不要继续要求 advisory 查询成功才能维持已证明安全的 MISSION 局部运动。

已有 P5 final/preview 路径如果重复授予或否定相同曲线权限，应收敛为使用统一认证结果的薄调用；不保留第二套策略。候选计算不修改父轨迹、active certificate 或实际运行统计。无调用者的旧预算裁短代码删除；仍供统计/兼容读取的字段标清用途，不另建兼容框架。

交付：回归复现 `0.399975655 + 0.003279691 > 0.4`，在局部安全的 MISSION 下从生成到激活再到运行均不因额度停车；GNSS 风险明显变差仍能改变候选偏好；STRICT 和真实局部危险仍拒绝/制动。随后做一次短 live，验证首段可执行且原 exposure 首因消失；这还不是最终验收。

### 步骤 4：修复下一段准备与连续切换

1. 根据步骤 1 和步骤 3 的生产日志修实际阻塞，不预设碰撞、算力或控制器是原因。优先复用 normal candidate 的下一通道机制和现有 prepared bundle。
2. 一条不可变候选失败后不能修改它再冒充同一曲线；在截止时间内尝试下一候选，必要的有界生成修正必须产生新曲线身份。复用现有 worker/候选槽位，禁止无限重试。
3. normal、LIMITED_PREFIX 和 degraded 当前段都应提前准备后继。准备时间必须覆盖生成、完整认证、最新必要复核、发布和控制端接收；使用实测耗时，不用平均值证明截止时间一定满足。
4. 在父轨迹进入终端减速之前，从同一固定切换时刻的真实计划 p/v/a 生成后继，保持 p/v/a 连续。控制端按约定时刻切换；迟到候选取消，不从中途追赶。
5. pending/候选失败时不撤销仍安全的父轨迹，不重置父开始时间和终点。无合格后继才执行原安全停车；以后有新有效证据或新规划机会时允许恢复，不永久锁在失败状态。
6. 同快照、同切换起点和可比任务进度比较 actual bundle，给每个候选有界计算机会。失败/未完成不等于整条通道封闭；不能为连续性硬编码左/右通道或绕过实际曲线比较。
7. 若日志证明存在阻塞 callback、重复几何提交或重新计算，局部消除重复并保留原预算。不要靠增加线程、扩大所有预算或延长地图有效期掩盖问题。

交付：先在短 live 中证明至少两次真正连续的后继切换，再进入完整森林；失败时保留具体首因并继续修复。不能用“successor_fast_path_ready”或发布次数代替实际切换证据。

### 步骤 5：完整 live 验收与清理

按第 7 节完成健康场景和故障场景。每个新失败按最早破坏条件定位，补相关回归、最小修复、提交、重建后重跑。清除废弃分支及误导日志，更新文档和复现命令。

不继承历史对话中的“只能一次 live、不能根据 live 修复”的限制：本轮是日常开发，可以反复验证。若某次修改造成回归，可以 revert 该逻辑提交保护基线，但回滚不等于完成；还要继续实现有效修复。最终三次健康验收必须来自同一源码和配置，修改后重新计数，不混用不同版本的成功片段。

## 6. 文件入口与必要回归

以下路径均相对仓库根目录。用符号重新定位，不依赖行号。

| 工作 | 优先入口 |
| --- | --- |
| 实际曲线、exposure 与局部安全策略 | `src/iap/planner/trajectory_assurance.cpp`、`include/iap/planner/trajectory_assurance.hpp` |
| 生成、候选、后继、发布检查及运行时 ledger | `src/iap/planner/plan_manage/src/planner_manager.cpp`、对应头文件 |
| 优化器首因、FSM 失败传递 | `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`、`plan_manage/src/ego_replan_fsm.cpp`、`p4_actual_curve_certifier.cpp` |
| P5 当前/未来检查与 final/preview | `src/iap/planner/plan_manage/src/p5_runtime_integrity_gate.cpp` 及 manager 中实际调用者 |
| 控制端切换 | `src/iap/planner/plan_manage/src/traj_server.cpp`；仅有证据时修改 |
| 通道比较 | `src/iap/planner/bspline_opt/src/p4_forward_route.cpp` 及 manager 已有 bundle 比较 |
| live runner/analyzer | `scripts/dev_planner/run_icra_interface_integration.py`、`test/test_icra_interface_integration.py` |
| 参数及持久契约 | `launch/test_planner.launch.py`、`plan_manage/launch/advanced_param.launch.py`、两份 Markdown spec、`docs/CHANGES.md` |

必要回归覆盖：

- MISSION：旧峰值、时长、积分、历史额度超限均不能单独否决；GNSS 缺失仍明确 unknown，不伪造安全值。
- 相同物理证据经过生成、认证、发布、激活和下一次 watchdog 后，结论不因重复算账翻转。
- STRICT 的不合格绝对定位，以及两模式的碰撞、净空失败、无可用制动、身份不匹配和局部过期仍不能继续授权。
- 优化/refinement 真实失败类别端到端保留；候选 A 失败后，截止时间内 B 可接棒；超时则安全停车且之后能恢复。
- 后继准备不修改父状态；非零速度切换连续；迟到、重复/乱序确认、不同 hash 同 ID 均不能错误接管。
- analyzer 拒绝“只有发布没有移动”“把 guard 当后继”“数据中断”“到终点一帧后退出”“未通过四叉”和 required process 提前死亡等伪通过。

优先扩展 `test_trajectory_assurance`、`test_p4_actual_curve_certifier`、`test_p4_forward_route`、`test_p5_runtime_integrity_gate`、FSM/发布生产路径测试和 runner 测试。按修改范围执行构建与测试；最终集成运行受影响的 IAP/bspline_opt/plan_env/ego_planner 测试。测试数量不是交付指标。

## 7. Live 才是效果验收

### 7.1 每次运行的前提

- 本次源码、配置和测试先提交，实际运行源码工作副本须干净，manifest 记录 commit 和 `git_worktree_clean=true`。不得 stage 用户历史记录来凑干净；若有无关改动，保留原处，在仓库允许范围内准备独立干净工作副本，并明确 build/install 实际来源。
- 重建受影响包、source 对应 install，核对实际节点与源码版本。禁止源码已回滚、二进制仍旧而不报告。
- GPU preflight 必须同时通过 `nvidia-smi` 和 CUDA `cuInit(0)`、device count >= 1；失败记 `GPU_NOT_READY`，不能用 CPU 测试冒充 live。
- 只终止本任务拥有的进程。检查 required process 在任务期间存活；runner 正常收尾发出的信号与意外提前退出要分开，不能只看 launch exit code。

现有入口如下；若使用独立工作副本，替换为其已核对的源码/install 路径，不盲用共享旧安装：

```bash
cd /home/dev/ws_iap/src/iap
source /home/dev/ws_iap/install/setup.bash
python3 scripts/dev_planner/run_icra_interface_integration.py \
  --stage continuous-flight \
  --scenario icra_dense_forest_four_fork_v2 \
  --gnss-arm bds \
  --task-mode mission_best_effort \
  --repetitions 3
```

可加现有 `--rviz` 观察，但可视化不作为通过依据。180 秒是现有任务观察上限，不改变预测 horizon 或数据有效期；超时未到终点就是未完成。
runner 在创建 session、执行 GPU preflight 或启动 ROS 前要求目标文件系统至少有
`20 GiB` 可用空间；不足时以 `DISK_SPACE_LOW` 终止。普通开发、最终三次健康验收和
选路/故障对照均使用默认 compact 模式。只有已经明确首因、确实需要逐点卫星分解的
单次诊断运行才可显式添加 `--retain-raw-risk-detail`；该模式还必须使用
`--raw-detail-reason` 记录已确认首因、固定 `--repetitions 1`，并使用
`--raw-satellite-detail-max-rows` 的正数硬上限（默认仅 `5000` 行）。这类运行强制标记
`diagnostic_only=true`、`acceptance_eligible=false`，即使过程通过也只能得到
`DIAGNOSTIC_PASS`，不得用于标准 live、连续飞行验证或最终效果证明。runner 会在
20 GiB 基线之上预留
估算空间，达到上限后文件只追加一次 `TRUNCATED` 标记并停止详细行。

### 7.2 必须全部满足的健康验收

在最终同一 commit、相同配置下连续运行三次，三次都满足：

1. 实际通过四个分叉并到达目标；目标距离不超过现有 `0.5 m` 验收范围，速度小于 `0.05 m/s` 持续至少 2 秒，且有持续命令、反馈和 odom。不能只凭最后一帧过关。
2. 每次至少两次匹配父子身份的非零速度后继切换；所需准备/认证完成早于切换，p/v/a 连续并满足现有跟踪包络。guard、emergency、停车后重启都不能计入。
3. 从首次实际出发到最终接近目标，非终点低速 `<0.05 m/s` 持续 `>0.5 s` 计为停顿；结合命令、反馈和噪声判定。不要沿用只检查 `x>-17.5` 而漏掉起步后短距离停车的漏洞。首次准备与最终悬停排除。
4. 零次因 MISSION 的 peak/duration/integral/prior episode 单独拒绝执行或制动；至少一个 live 覆盖旧额度本会超限的情况，证明相关路径实际被执行。
5. 零碰撞、零未认证执行、零身份冲突、零迟到追赶、零仅因 generation 变化重置轨迹。跟踪误差在实际配置包络内，制动始终有适用依据。
6. 分叉选择有同快照、相当任务范围的实际候选风险/净空/时间分解；不能只比较 guide。用 odom 证明实际通过，而非规划控制点投票。若在线证据与场景预期低风险侧不同，必须解释并修复已证明的预测/比较错误，不能按场景标签强选。
7. 再用现有单分叉 primary/mirror 场景验证证据交换后选择随之改变。没有完成必要比较的运行不能算“正确选路已验收”。

### 7.3 必须有 live 的故障对照

复用已有注入入口；缺少时只补最小、显式、默认关闭的测试注入，不建新平台。每类至少一次，单独保存，不能混入健康三次计数。

| 注入 | 预期结果 |
| --- | --- |
| MISSION 仅 GNSS 变差/缺失，局部输入正常 | 风险/unknown 如实变化；可请求重规划，不因旧额度单独停车 |
| 新障碍进入剩余轨迹 | 有提前量时重规划或认证制动，不继续穿过障碍 |
| 局部地图或定位反馈断流 | 在适用新鲜度/制动时限内采取安全动作，不能沿用过期授权 |
| STRICT 的 GNSS 不合格 | 保留严格模式的拒绝/安全停车行为 |

故障必须在已经实际运动、且注入到对应生产输入后发生。记录注入时间、首次检测、动作、控制端激活及最终状态；制动曲线须适用于当时状态。测试本身造成 unrelated 进程退出不算正确制动。

### 7.4 保留什么证据

每次保留 commit 和 clean-worktree 身份、有效参数、GPU preflight、运行命令、owned
process/退出状态、`execution_events.csv`、`runtime_window_batch.csv`、紧凑的
`forward_channel_decisions.csv` 与必要候选对照，以及按时间关联的实际曲线、父子身份、
PositionCommand、controller trace 和 odom 摘要。失败运行另保留首个真实首因前后的
有界 stdout、lineage 和 runtime-window 行。最终证明依赖这些紧凑但充分的身份、控制、
运动、候选比较和故障事件证据；不要求、也不得在每次验收中保存全部逐点/逐卫星原始
CSV。

证据放仓库内 `results/` 的本任务目录。默认 compact 模式在生产端不创建
`forward_risk_samples.csv`、`gnss_risk_detail.csv`、`runtime_window.csv` 或
`runtime_window_satellite.csv`，而不是等运行结束后再删除。调试阶段保留首个失败和
最终验收所需的关键 capture；可再生大文件只有在紧凑首因证据落盘、运行结束、非
symlink、非 tracked 且无进程占用后才能按精确路径删除。遵守 AGENTS 的删除规则，
不建立仓库外归档，不清理任务范围外数据。

## 8. 完成判定与交付报告

- [ ] 所有 MISSION exposure 否决路径已审计并移除；风险统计和排序仍有效。
- [ ] P5 职责收敛，局部安全检查与 STRICT 要求仍有效，无新重复授权系统。
- [ ] 后继真实失败可定位、可恢复，并能提前准备和连续接管。
- [ ] 相关 CPU/生产路径测试通过，runner 不会误判上述伪通过。
- [ ] 同一版本三次森林全程通过，单分叉对照和 live 故障测试通过。
- [ ] 契约、运行命令和旧计划状态已同步；改动限于本任务必要范围。

报告按“基线 → 最终每次运行”列出：位移、通过分叉数、终点误差、连续切换数、最长中途停顿、最大跟踪误差、GNSS-only 停车次数、首因、required process 状态及证据目录。说明实际修改、删掉的重复逻辑、测试和已知限制。

任一健康运行未完成、故障保护未通过、必要证据缺失或 GPU 不可用，都不得写“彻底解决”。存在可复现问题时继续本计划内的诊断与修复；只有确实需要外部条件或超出本边界时，具体说明阻塞、已完成内容和缺失的那一步。不能以“CPU 全过”或“已回滚”结束为成功。

## 9. 给开发 Codex 的开工指令

> 阅读 AGENTS.md、本计划、相关 spec 和当前源码，按步骤 1→5 实施。用户已确认 MISSION exposure 取消独立停车权、失败保留首因、P5 收敛为运行时监督；在这个范围内自主完成实现、测试、提交和 live 修复循环，不反复请求已授权事项。先核对现状并复用已有认证、局部安全、候选比较、后继和 continuous-flight runner。禁止新增定位系统、重复授权框架或放宽安全标准。每次 live 使用可追溯的干净源码版本；最终必须交付同一版本三次完整森林通过、选路对照和故障保护证据。发现未解决问题继续修复，不把阶段性测试或回滚描述成完成。

## 10. 本轮执行进度

- 提交：步骤 1 为 `83fddbc`、`9f27a05`、`f6761b8`、`784d3d3`、`a4a35e9`、`30426ac`、`ffc118b`；步骤 2 为 `a751d5f`、`641b3ad`；步骤 3 为 `93bd62c`；步骤 4 为 `36e8236`、`24e8e74`、`1f59a51`、`8a2a526`、`04b46f4`、`0775497`、`825b7f0`；磁盘与证据策略为 `64f6764`、`b772ed9`（均基于计划提交 `e675443`）。用户历史文件 `docs/whatsnext.md` 保持未跟踪、未修改。
- 完成项：步骤 1 已补全 optimizer/refinement/manager/FSM 首因，continuous-flight analyzer 已拒绝 guard 伪后继、身份或父子链不符、零速切换、命令断流、required process 提前退出、短程停车、终点悬停不足和未由 odom 穿越四叉。步骤 2 已确认生产路径统一复用 `LocalMotionAssurance`/`LocalClearanceEvaluator` 且必要输入缺失时 fail closed；森林配置仍为 tracking `0.15 m`、safety margin `0.20 m`、surface `0.02 m`、`uncalibrated_default_v1`，故后续证据只适用于仿真/开发，不能冒充部署标定。步骤 3 已移除 MISSION exposure 的独立拒绝/制动权并保留真实风险诊断和排序；STRICT、局部净空、support/freshness、身份、动力学与制动门不变。
- 失败首因：基线轨迹 5 为 `SUCCESSOR_CURVE_PREPARATION_FAILED` 且旧 detail 泄漏初始化占位；轨迹 9 为 P5 重复累计 prior episode 后 exposure braking。原始证据：`results/icra27/dev_runs/interface_integration/run-20260926T050049Z-450628/full-r01-risk/exports/planner_p4_risk_astar_debug.csv.execution_events.csv` 及同目录 `stdout.log`。步骤 3 干净 live 已证明旧 exposure 不再制动，但第一 actual B-spline 因 `local_clearance_margin_not_positive` 被硬局部门拒绝，下一冻结通道又继承失败候选的 `HOLD_REQUIRED`，故未首航；`bspline_count=0` 和零位移是结果，不是首因。
- live 证据：步骤 3 使用独立 worktree/build/install，在 `93bd62c78b6dc7af731b25c6e2a71794d02b14eb` 运行 180 s；GPU preflight 通过、required processes 未提前退出、launch exit 0，manifest 为 `git_worktree_clean=true`。目录：`/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T081255Z-633590`。该失败运行不计入最终健康三次。
- 步骤 4 修改与 CPU 证据：typed failure 仍绑定不可变失败候选，下一冻结 guide 的新事务显式恢复 `RETAIN_COMMITTED_TRAJECTORY`；actual optimizer 使用固定 guide 投影约束拓扑管道，并在不改变最终硬 `0.5 m` corridor 的前提下预留一个 `0.05 m` geometry-commit chord。生产回归已从候选 B 生成前失败变为两个 actual bundle 均生成并完成比较；`test_planning_risk_context` 163/163、`test_trajectory_assurance` 42/42、`test_p5_runtime_integrity_gate` 46/46、bspline optimizer 6/6、runner 94/94 通过。
- 步骤 4 干净 live：`36e82369e4028143b8d00b6a4797135bc6ab18d0`、`git_worktree_clean=true`、GPU preflight 通过并完整运行 180 s。trajectory 2 实际授权、匹配激活并运动，最大跟踪误差 `0.0243 m`，已越过旧 `bspline_count=0` 阻塞；随后错误排队 guard 并停在约 `(-15.327,-0.991,1.364)`，故不计入健康验收。首因是 watchdog 时刻 `1657065614.7482536` 与最新 snapshot `1657065614.7482538` 仅差约 `0.24 µs`，aggregate evaluation-time 使用零容差而同对象的 source stamps 已使用 `1 µs` 容差，导致 `runtime_execution_snapshot_missing`。证据：`/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T083316Z-698805`。
- causal-tolerance 修复与 CPU 证据：`24e8e74` 仅将上述既有 `1 µs` causal tolerance 同步到 aggregate snapshot 时间；超过 `1 µs` 的未来数据和全部 freshness/局部安全门仍 fail closed。新增回归先红后绿；`test_p0_risk_grid_runtime` 117/117、`test_planning_risk_context` 163/163、`test_p5_runtime_integrity_gate` 46/46 通过。
- `24e8e74` 干净 live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T084442Z-745536`；GPU preflight 通过并完整运行 180 s，未发布也未运动。首因发生在 decision event 252：通道 2 已形成局部安全的完整 actual bundle（全局 GNSS 仍如实为 `GNSS_GEOMETRY_DEGENERATE`，局部最小净空 `0.012194 m`），通道 1 actual curve 被硬局部门以 `trajectory_assurance_rejected:local_precheck:local_clearance_margin_not_positive` 拒绝；比较器正确恢复通道 2 并记录 `normal_channel_comparison_complete`，但 FSM 随后无条件 `reject_candidate()`，丢弃已恢复赢家。证据：同目录 `compact_evidence/cause_lineage_excerpt.csv`、`compact_evidence/compact_failure_evidence.json` 及保留的 channel-decision CSV。该失败运行不计入最终健康三次。
- 当前最小修复与 CPU 证据：末通道的类型化失败若已完成比较并恢复更早的完整 bundle，FSM 保留该 bundle、绑定当前 planning attempt，并继续既有 latest-snapshot/P5/身份/局部安全/制动发布链；失败通道仍保持不可变且不重试。新增生产路径回归先证明旧 FSM 返回 false，修复后证明实际发布通道 2 对应的既有赢家；`test_planning_risk_context` 164/164、`test_p0_risk_grid_runtime` 117/117、`test_p5_runtime_integrity_gate` 52/52、`test_trajectory_assurance` 42/42 通过。P0 初次复跑因测试二进制与更新后的共享库不一致在冻结快照用例段错误，原目标就地重建后 117/117 通过。
- `d8dbc95` 干净 live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T092238Z-832138`；GPU `cuInit(0)`、device count 1 和 required processes 均通过。轨迹 121 被授权并匹配激活，实际移动、最大跟踪误差 `0.030908 m`，证明末通道已有赢家发布阻塞已解除；随后停在首个分叉前，故不计入最终健康三次。首因是安全 timer 使用的最新 odom stamp 冻结在 `1657065678.5507429`，同身份 controller trace 已推进约 `3.9 s`；P0 继续发布较新快照并最终从 32 项因果历史淘汰旧快照，watchdog 因而先记录 `runtime_execution_snapshot_missing`、调度 guard，再因冻结时间不能接受后续反馈而制动。证据：同目录 `summary.json`、`planner_p4_risk_astar_debug.csv.execution_events.csv` 及 `runtime_window_batch.csv`。
- watchdog-clock 修复与 CPU 证据：`8a2a526` 仅让 runtime watchdog、同次 geometry check 与 P5 handoff 使用既有 `latest odom stamp + steady elapsed` 调度时钟；规划生成仍使用精确 sensor stamp，同身份 controller trace 提供实际状态，所有 snapshot/support/trace/碰撞/跟踪/制动 freshness 门不变。新增回归先红（20 ms 后仍为相同 odom stamp）后绿（odom callback 暂时饥饿期间 watchdog 仍继续推进）；`test_planning_risk_context` 165/165、`test_p0_risk_grid_runtime` 117/117、`test_p5_runtime_integrity_gate` 52/52、`test_trajectory_assurance` 42/42 通过。
- `a20b5fc` 干净 live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T093657Z-885783`；GPU preflight、required processes 和 180 s 完整运行均通过。trajectory 14、18 均被授权、匹配激活并运动，最大跟踪误差 `0.032100 m`，且不再出现 `runtime_execution_snapshot_missing`，证明 watchdog 时钟修复有效；但只有一次实际分叉、零次真实后继切换，guard 15、20 均不计入后继，故不计入最终健康三次。
- 当前失败首因：trajectory 14 激活后首个 `outside_controllable_braking_domain` 将 FSM 从 `EXEC_TRAJ` 推入 `REPLAN_TRAJ`，真正后继准备直到 guard 15 接管后才开始。该次撤权的 controller trace 显示实际位置误差约 `0.0134 m`、仍在认证控制域和 `0.15 m` 跟踪包络内；执行 seam 却把约 `0.023 m` latency-reachable deviation 再从已经扣除认证跟踪包络的 `0.0136 m` 本地证书余量中扣除，形成双重计入。后续 `local_clearance_margin_not_positive` 仍是硬安全事实，不作豁免。
- 认证制动域修复与 CPU 证据：`04b46f4` 只消除上述双重计入；反馈仍须同时满足认证 position/velocity error 和绝对 velocity/acceleration 动力学界，离开认证域后仍须让完整 latency-reachable excursion 落入剩余净空，否则原样撤权。回归先红后绿，并增加绝对速度超限仍失败的反例；`test_planning_risk_context` 165/165、`test_p0_risk_grid_runtime` 117/117、`test_p5_runtime_integrity_gate` 46/46、`test_trajectory_assurance` 42/42 通过，`ego_planner_node` 完整重建通过。
- `83e27a9` 干净 live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T095429Z-947929`；GPU preflight、required processes 和完整 180 s 运行通过，但零发布、零位移，故不计入健康验收。最早失败事务为 decision event 2：通道 2 的 route-level MISSION degraded 状态先形成类型化 support 失败，通道 1 actual 又因 corridor support 过期失败；这两个失败 bundle 随后在相同 stable channel ID 和相同 snapshot 下被事件 3--296 错当成已完成通道，后续每个事件只生成通道 1，`normal_channel_typed_failure_complete` 从未重新排队通道 2。278 条 actual 均在 P5 前硬拒绝（184 条 local clearance、94 条 support expired）；这些硬门保持不变，缓存跨事件复用才是未尝试 runner-up 的首因。
- 当前有界修复与 CPU 证据：`0775497` 将 normal prepared-bundle/typed-failure 事务按现有 `decision_event_id` 隔离；同事件冻结 siblings 仍共享比较身份，后续事件即使复用同一 snapshot 和 stable channel ID 也必须重新生成 actual curve。新增回归先红（新事件首通道失败直接得到 `normal_channel_all_preparations_failed`）后绿，并与同事件 runner-up 恢复、完整 actual bundle 比较用例共同通过；`test_planning_risk_context` 166/166 通过，`ego_planner_node` 完整重建通过。未改变 local clearance、support expiry、动力学、制动、P5 或发布标准。
- 磁盘中断与清理：`42b2020` 的 `run-20260926T101027Z-1006013` 已经完整退出且 runner 的 launch/capture owned process group 均清空；因无界逐卫星记录耗尽文件系统，显式标记为 `DISK_SPACE_ABORTED`、`acceptance_eligible=false`，不计入健康或失败验收。7 个已结束失败开发 run 均已生成 clean commit、GPU/命令/process、首因、候选、控制/odom/轨迹和有界日志的 `compact_evidence/`；只按验证后的精确路径删除逐卫星、GNSS detail 和完整大 lineage，共释放 `44,138,055,001` bytes。清单：`/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/evidence_cleanup_manifest.json`；清理后 `df` 可用 `43,773,550,592` bytes。完整 capture/stdout 保留供步骤 4 时序复查，仓库其他位置未清理。
- 防复发修改：默认 production writer 不再生成 raw point/window/satellite 文件；compact channel、candidate、execution event 和 runtime batch 保留。显式 raw 只允许一次有明确首因的诊断，默认 satellite 上限 `5000` 行，达到上限只写一次 `TRUNCATED`，并强制排除在标准/连续飞行/最终验收之外；runner 在 GPU/ROS 前执行 20 GiB 加 raw 估算的磁盘门禁。runner 98/98、launch contract 32/32 与 17/17、`test_planning_risk_context` 167/167、`test_p4_risk_astar` 19/19、`test_p4_forward_route` 125/125 通过，两个 hermetic launch 套件均确认外部 ROS 日志无变化。该清理与证据策略修改不是 Goal 完成，也不改变任何规划、安全或 live 验收门限。
- `9998f48` compact live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T105426Z-1098810`；干净 worktree、GPU 与 required process 均正常，180 s 内未创建 raw point/window/satellite CSV，磁盘保持约 49 GiB 可用。轨迹 27 实际运动并通过第一个分叉，但首个后继 worker 在 `1657065635.4480627` 返回 `CANDIDATE_READY`、`successor_fast_path_ready`、`successor_failure=NONE` 时仍携带结构默认 `HOLD_REQUIRED`，被同一调用错误记录为 `SUCCESSOR_CURVE_PREPARATION_FAILED`；随后 support 过期、重新起飞与 guard 制动均为次生结果。该运行不计入健康验收。
- 步骤 4 最小修复：成功完成的后继 route 只作为子曲线准备输入，显式保持已认证父轨迹 `RETAIN_COMMITTED_TRAJECTORY`，不授予子曲线权限、不改变任何认证或安全门。生产分支回归及完整 `test_planning_risk_context` 167/167 通过。
- `825b7f0` compact live：目录 `/home/dev/ws_iap/live_worktrees/mission-step3/results/icra27/dev_runs/interface_integration/run-20260926T110803Z-1155973`；干净 worktree、GPU、required process 和 180 s 运行正常，未创建 raw point/window/satellite CSV。上一运行的 `successor_fast_path_ready` 携带 `HOLD_REQUIRED` 首因已消失；本次轨迹 21 授权并匹配激活，但 `1657065622.426177` 的 watchdog 仍使用 activation ACK 写入的临时 `elapsed=0` 样本，而同身份 controller trace 已实际推进约 `0.6 s`。因此 committed-window 选择错误包含已经过去的起点行，首个失败为 `runtime_corridor_support_stale_or_invalid:EXPIRED`，随后 guard 22 停车；该运行不计入健康验收。
- 当前有界修复：activation ACK 临时样本仅用于等待首个真实控制反馈；一旦同身份 controller trace 到达，即用其实际 elapsed/p/v/a 推进 watchdog，不再让临时零进度遮蔽真实控制反馈。未改变 support freshness、局部净空、跟踪、动力学或制动门；新增生产路径回归并与完整 `test_planning_risk_context` 167/167 通过。
- 下一步：提交并在独立干净 worktree 重建上述 activation-placeholder 修复，继续 compact continuous-flight live。必须先证明后继在父轨迹仍运动时准备并至少两次按父子身份接管，再进入步骤 5 的完整森林、三次同版本健康验收、选路对照和故障保护；不重新实现步骤 1--3。
