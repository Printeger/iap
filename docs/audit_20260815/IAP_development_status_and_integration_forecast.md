# IAP 当前开发状态、联调工期与风险审计

> 审计日期：2026-08-15 UTC
>
> 审计对象：`/home/dev/ws_iap/src/iap`，HEAD `b6c8236bbbf9858c65c1c5120fd4a3c052399700`
>
> 证据截止：仓库中截至 2026-08-10 的代码、测试结果和实验记录
>
> 审计性质：静态复核与既有证据汇总；本次未执行 ROS campaign，未修改产品代码，也未把历史失败追认为通过。

## 1. 执行摘要

IAP 已经不是“功能尚未接起来”的原型。估计、GNSS/IMU/LiDAR 完整性、未来风险预测、P0 风险场、P1–P5 规划接口、日志分析和验收工具均已有实现，基础闭环、Predictor、P0 和 P5 已有较强的自动化或运行证据。当前 `iap` 包最近一次留存结果为 23/23 CTest、288/288 test cases 通过；P1–P4 对应的嵌套 planner 功能 GTest 也有全绿记录。

但是，按仓库自己冻结的最终验收定义，系统还不能宣布“联调成功”：

- baseline 4 项、P0 6 项和 P5 8 项已经正式通过，共 18 项；
- Phase 3 v2 的 P1-1 至 P1-6 尚未执行 fresh campaign；旧 Phase 3 v1 的 P1-2 仍为 `BLOCKED`；
- P2、P3、P4、Integrated Ablation 和 Robustness/Stress 尚无正式通过证据；
- 旧 Demo11 full v2.0 acceptance 仍为 `FAIL`，没有被后续的专项 safety-planner campaign 追认或替代；
- 默认 25 m URG 的历史运行性能仍超过 1000 ms 门限；
- 当前实验数据占用 97 GiB，文件系统使用率 95%，只剩约 32 GiB，这已经是继续跑 fresh campaign 的直接操作风险。

因此，本审计给出两个不同维度的完成度：

| 维度 | 审计判断 | 解释 |
|---|---:|---|
| 代码/模块实现成熟度 | **约 85%–90%** | 核心模块、开关、日志、fixture 和单测大多存在；这不是产品验收比例。 |
| 现行 P0–P5 正式实验矩阵 | **18/63，约 29%** | 4 个 baseline + 6 个 P0 + 8 个 P5 已通过；P1 v2 以后 45 项尚未正式完成。 |
| 完整实验室联调结论 | **未通过** | 旧 Demo11 full acceptance 仍失败，P1 v2、P2–P4、全栈与压力阶段缺证据。 |

机械地统计 `docs/REQS.md` 可得到 105/112 checkbox 已勾选（约 94%），但该文件同时保留了旧版与 next-phase 两套状态，且部分旧未勾选项在后文已标为完成，所以不能把 94% 当作可交付完成度。

## 2. 本审计采用的“联调成功”定义

为避免把“能启动”“某个模块通过”和“产品联调成功”混为一谈，本报告采用三个里程碑：

1. **M1：基础闭环可运行**：IAP odom、EGO planner、控制器、仿真 plant、完整性 topic 和日志链路能够共同运行并产生轨迹。这个层级已有历史通过证据。
2. **M2：实验室审计级联调成功**：现行 `safety_planner_p0_p5_test_plan.md` 中 baseline、P0、P5、P1 v2、P2、P3、P4、A-ALL 及关键 robustness gates 全部通过；P5 始终是唯一 hard safety authority；完整报告、manifest、bag、哈希和图表闭环。**当前未达到。**
3. **M3：真实设备/外场可交付**：在真实 GNSS/LiDAR/IMU、真实时钟与算力约束下重复 M2，并完成飞行安全、故障注入、长时稳定性和回归基线。仓库现有证据主要来自仿真/合成 fixture，M3 不在当前已完成证据范围内。

用户如果只要求“一次仿真中跑出完整轨迹”，M1 已经实现；如果要求可审计、可重复、能证明安全边界的“联调成功”，应以 M2 为准。

## 3. 当前开发状态

### 3.1 已完成或已有强证据的部分

| 子系统 | 当前状态 | 主要证据 |
|---|---|---|
| Repo、构建、独立 ROS2 包 | 已实现 | `docs/REQS.md:24-59`；当前 HEAD 的 `iap` package 结果为 288 tests、0 failure。 |
| 紧耦合估计器、时钟状态、GNSS PR/多普勒、IMU、LiDAR health | 已实现 | `docs/REQS.md:63-107`；GNSS epoch 与启动语义的后续修复见 `docs/TRACEABILITY.md:438-450`。 |
| GNSS/LiDAR ARAIM、H/V safety、fusion/fallback | 模块级完成且有运行/单测证据 | `docs/audit_0520/status.md:9-23`；`docs/dev_ARAIM/ARAIM _test.md:743-946`、`:1620-1692`。 |
| Predictor advisory query | isolated 与 12 个 system experiments 已通过 | `docs/dev_predictor/predictor_test_report.md:44-72`、`:3336-3363`。 |
| P0 风险场 | P0-1 至 P0-6 正式 PASS | `docs/dev_planner/safety_planner_test_report.md:1982-2075`、`:2077-2209`、`:2211-2399`。 |
| P5 runtime/final hard gate | P5-1 至 P5-8 正式 PASS | 逐级结论见 `docs/dev_planner/safety_planner_test_report.md:2672`、`:2990`、`:3581`、`:3701`、`:3807`、`:3992`、`:4335`、`:4425`。 |
| P1 local mechanism | 有局部因果迹象，但无产品 PASS | c38 同快照 632/632 行满足 objective/mean/梯度方向；边界见 `docs/dev_planner/safety_planner_test_report.md:9740-9774`。 |
| 证据工具链 | 已实现 | Analyzer、manifest、归档、SHA256 和图表均存在；P1 retrospective 映射见 `docs/TRACEABILITY.md:475-481`。 |

### 3.2 已实现代码但尚未完成正式运行验收的部分

P1、P2、P3 和 P4 都有源代码和功能测试目标：P1 B-spline cost、P2 candidate ranking、P3 reference bias、P4 risk A*。最近留存的功能 GTest 结果均为 0 failure：P1 39 cases、P2 6、P3 9、P4 4，另有 P0/P5/context 等 planner tests。

这只能证明局部接口和确定性逻辑，不证明闭环中候选、snapshot、时间基准、控制执行、fallback 和统计效果同时成立。现行计划明确要求：

- P1 v2：6 组协议，包含固定 snapshot 因果、lambda/clip sweep、fresh 配对、unknown/stale 和 P5 authority isolation（`docs/dev_planner/safety_planner_p0_p5_test_plan.md:313-327`）；
- P2：6 组正式实验（同文件 `:331-342`）；
- P3：7 组正式实验（`:346-358`）；
- P4：6 组正式实验（`:362-373`）；
- 集成消融：10 组（`:377-392`）；
- 鲁棒/压力：10 组（`:396-411`）。

这些阶段目前不能按 unit test 结果自动视为通过。

### 3.3 当前未完成/未关闭的验收项

| 项目 | 当前正式状态 | 为什么仍未关闭 |
|---|---|---|
| P1 Phase 3 v2 | **未运行** | 协议已冻结，但明确要求 future fresh campaign；不得用 c31–c38 改写旧 verdict。 |
| 历史 P1 v1 P1-2 | **BLOCKED** | c31、c32、c38 是三次完整可比失败；formal analyzer 调用 0；见 `docs/dev_planner/safety_planner_test_report.md:9710-9747`。 |
| P2 Candidate Ranking | **未正式验收** | 旧 c38 运行时 P2 为 off，只可迁移场景，不能计作 P2 证据；见 `docs/TRACEABILITY.md:480`。 |
| P3 Reference Bias | **未正式验收** | 有实现和 GTest，但所需 coverage/no-backtracking/two-corridor/fallback fixtures 未形成完整通过链。 |
| P4 Risk-aware A* | **未正式验收** | 有实现和 GTest，但 collision guide、long-detour、unknown、occupied hard reject 尚缺正式整套结果。 |
| Full Demo11 v2.0 | **FAIL / 未重验通过** | 历史 legacy/enabled 均 `planner_trajectory_count=0`，且 enabled URG 1877.573 ms > 1000 ms；见 `docs/stage1_v2.0/implementation_audit_v2.md:402-430`。 |
| Integrated Ablation / A-ALL | **未运行或无正式报告** | 这是证明组合收益、无 double-count、无 collision/emergency storm 的关键阶段。 |
| Robustness / Stress | **未运行或无正式报告** | GNSS/integrity delay、NaN、CPU stress、动态风险和连续 final-gate fail 等仍待验证。 |
| 真实设备/外场 | **无足够证据** | 当前证据主要是仿真、合成传感器和预定义 fixture。 |
| IMU health/noise inflation | **需求状态未关闭** | 顶层架构要求 IMU 饱和/模型失配触发 noise inflation，但 `docs/TRACEABILITY.md:231` 仍保留 TODO；正式收口前需实现，或书面降级为后续范围。 |

## 4. 当前阻塞与已知困难

### 4.1 P1 的科学验收，不再只是 plumbing 问题

c38 已证明 10/10 validator、provenance、P0、安全、定位、checkpoint 和两臂 200/200 支持稳定；同 snapshot 的局部优化也显示风险下降。但旧跨运行绝对阈值仍未达到。仓库已经正确把路线选择迁到 P2，并冻结 Phase 3 v2。现在需要实现/固化 v2 harness 后重新证明：固定候选上的 soft preference 有效，同时 max risk、碰撞、动力学和 P5 authority 不退化。

困难在于这是一项统计与实验设计工作，不是简单调整一个 lambda。若 fresh v2 仍无法达到 9/10 同向改善或 null 非劣界，可能需要产品/算法负责人决定 P1 的 objective 或验收声明，而不能继续按结果调 fixture。

### 4.2 Full Demo11 的历史 blocker 仍未被正式关闭

旧 full acceptance 中，legacy 与 enabled 都没有 planner trajectory，日志出现 `AstarSearch`、`Coord2Index` 和 `drone is in obstacle`；同时 enabled URG 超过性能门限。之后的 P1 c38 专项 campaign 能产生完整候选和轨迹，说明部分启动/定位/传感问题已修复，但它不是同一个 Demo11 acceptance 协议，不能据此追认 Demo11 PASS。

### 4.3 URG 性能仍有硬门风险

历史数据表明 25 m grid 从约 8881 ms 优化到约 1230 ms，但仍高于 1000 ms；90 s full run 记录为 1877.573 ms。主要时间花在 PL query 和 AL/ESDF，全量 rebuild 且无 incremental update（`docs/stage1_v2.0/implementation_audit_v2.md:226-252`、`:523-535`）。在 P1–P4 同时启用时，CPU 竞争可能让该问题更严重。

### 4.4 证据数据已逼近磁盘上限

审计时：

```text
results/planner_validation = 97 GiB
/dev/nvme0n1p4 = 648 GiB total, 583 GiB used, 32 GiB available, 95% used
```

历史 P1 单个 90 s bag 已超过 2 GiB，也发生过 occupancy evidence 约 1.09 GiB、MCAP 约 1.76 GiB 后文件系统耗尽、finalizer 被杀的情况（`docs/dev_planner/safety_planner_test_report.md:9318-9322`）。不先制定容量预算、保留策略和 campaign 前置空间门，新的 P1 sweep/fresh pairs 很可能再次产生不可用证据。

### 4.5 测试绿色并不等于质量门全绿

- `iap` package 的 Aug-10 留存结果为 288/288 通过，这是强的核心回归证据。
- `ego_planner` 的 P0/P1/P2/P3/P5/context 功能 GTest 留存结果均为 0 failure；但同一 build tree 的全量 `colcon test-result` 仍报告 526 failures、29 skipped，主要来自 `uncrustify` 等 lint 输出。P1/P4 所在其他 nested package 也有 lint/xmllint 留存失败。

这不代表 526 个运行时算法缺陷，但意味着“全工作区 CI 绿”尚未成立，审计时必须区分功能测试和样式/质量测试。

### 4.6 分支与协作风险

当前 `dev/iap` 相对 `origin/dev/iap` ahead 153。大量仅存在本地分支的提交会提高机器故障、协作者基线不一致、campaign 无法异机复现和后续合并冲突风险。正式继续长 campaign 前应先确认远端备份/集成策略；本审计不执行 push。

### 4.7 环境与默认配置不能被忽略

历史 B0-3 曾因 GPU 不可用导致 `iap_rosnode` 退出，恢复 GPU 后才通过；因此 GPU/driver 是 campaign 前置条件，而不是普通性能提示。与此同时，P1–P5 的安全/偏好功能采用渐进验证的默认关闭或 metrics-only 策略，这对防止未验收功能进入默认产品是正确的，但也意味着“代码存在”不能解释为默认发布配置已经启用完整 IAP。

## 5. 未来工作量

以下估算以“一名熟悉当前代码、可稳定使用现有 ROS/GPU 机器的工程师”为基准，单位为有效工程人日，不包含排队等待和真实外场审批。

| 工作包 | 主要内容 | 最可能人日 | 退出条件 |
|---|---|---:|---|
| W0 容量与基线治理 | 空间预算、可恢复归档、远端备份、全量功能/质量测试基线 | 3–5 | fresh campaign 不会因空间/版本不可复现失败。 |
| W1 Phase 3 v2 harness | 固定 candidate/snapshot/initial hash/support，四类冻结 field，v2 analyzer/manifest | 5–9 | P1-1/P1-2 可确定性复算。 |
| W2 P1 v2 全套 | raw/normalized sweep、null tolerance、fresh paired、unknown/stale、P5 isolation | 10–20 | P1-1 至 P1-6 全部 PASS，或形成需要设计决策的封闭失败。 |
| W3 P2 正式验收 | 同 attempt fork ranking、double-count、fallback/single candidate | 5–9 | P2-1 至 P2-6 PASS。 |
| W4 P3 正式验收 | local/global bias、coverage、no-backtracking、detour 和 plan fail fixture | 6–11 | P3-1 至 P3-7 PASS。 |
| W5 P4 正式验收 | collision guide、path ratio、snapshot/unknown fallback、occupied reject | 5–9 | P4-1 至 P4-6 PASS。 |
| W6 Full Demo11 与性能 | 重现/关闭 zero trajectory，URG <1000 ms，legacy/enabled official validation | 7–14 | full v2.0 acceptance 两臂通过。 |
| W7 Integrated Ablation | A-0 至 A-ALL、double-count/collision/storm 复核 | 6–12 | 组合收益与边界可解释。 |
| W8 Robustness/Stress | R-1 至 R-10、长时运行、异常输入和 CPU 压力 | 8–16 | 关键 fail-safe gates 通过。 |
| W9 审计收口 | 链接、hash、traceability、报告、lint/CI 和可复现命令 | 3–6 | 同一 commit 上证据完整、CI/审计无歧义。 |
| **合计** | 不含真实外场 | **58–111 人日** | M2 实验室审计级联调完成。 |

### 5.1 工期区间

先区分两个容易被混用的日期：在现有代码基础上，得到一次不作为正式结论的
`all` profile 端到端 smoke，最可能还需 **4–7 周**；达到本文 M2 所定义的、
所有关键门槛和证据均闭合的审计级联调成功，最可能需 **11–16 周**。前者不能
用于替代 P1 v2、消融和压力验收。

| 情景 | 日历时间（一名工程师） | 假设 |
|---|---:|---|
| 乐观 | **7–9 周** | v2 harness 复用现有 fixture；P1 首轮通过；Demo11 zero-trajectory 已被现有修复自然关闭；URG 通过缓存/缩小有效更新域迅速达标；很少重跑。 |
| 最可能 | **11–16 周** | P1/P2–P4 各 1–2 个 debug cycle；需实现缺失 fixtures/analyzer；Demo11 和 URG 需要专项修复；证据与磁盘治理同步进行。 |
| 保守 | **20–30 周** | P1 v2 仍发生科学失败并需设计决策；URG 需要结构性增量更新；全栈出现时基/定位/重规划耦合问题；多次 campaign 因证据或环境失效重跑。 |

若两名工程师能够明确拆分为“P1/P2 实验与分析”和“P3/P4/Demo11 性能与 runtime”，最可能可压缩到约 **8–12 周**，但同一 GPU/ROS campaign 环境、共享证据盘和串行 fresh-pair 协议限制了线性加速。

真实设备/外场 M3 不应包含在上述承诺中。在 M2 之后，建议另预留至少 **8–16 周** 进行传感器标定、时钟同步、真实星历/遮挡、算力、飞行安全和重复性验证；没有硬件、场地、数据集和安全流程信息时无法给出更精确承诺。

## 6. 可能面对的困难与风险登记

| 风险 | 概率 | 影响 | 预警信号 | 建议控制 |
|---|---|---|---|---|
| P1 的局部下降不能转化为 fresh 闭环稳定收益 | 高 | 高 | 9/10 sign gate 失败；效果不超过 null tolerance；max PL/路径退化 | 严格预注册 v2；失败时升级设计决策，不做 outcome-driven tuning。 |
| P1/P2/P3 同时启用产生 cost double-count 或相互抵消 | 中高 | 高 | P2 score 跟随含 P1 的 total cost；路径 detour 激增 | 优先完成 P2-3/P2-4，再跑 stack ablation。 |
| URG/预测计算拖慢规划周期 | 高 | 高 | mean/P95 >1000 ms；callback backlog；P5 使用 stale snapshot | PL/FIM cache、batch query、dirty-region incremental rebuild；把延迟纳入 hard gate。 |
| 启动定位、时间基准和传感语义回归 | 中 | 高 | checkpoint error、GNSS double epoch、SO3 acceleration spike、snapshot stale | 保留 c33–c38 修复的 provenance/contract tests；每个 fresh run 先过 startup gate。 |
| IMU health 需求未闭合 | 中 | 高 | 饱和/模型失配时无明确 inflation/fallback evidence | 在 M2 scope 中实现并验证，或由设计权威明确移出并记录残余风险。 |
| 磁盘满导致 bag/finalizer/manifest 不完整 | 高（当前） | 高 | available < campaign budget；MCAP/occupancy 快速增长 | 先做容量门、压缩与保留策略；估算 campaign 最坏空间并保留安全余量。 |
| fixture 通过但真实场景不泛化 | 中高 | 高 | synthetic/pass 与 Demo11/真实数据方向不一致 | 增加未参与 sweep 的 seed、独立场景和最终真实数据 replay。 |
| unknown/stale/fallback 组合产生危险的低风险解释 | 中 | 高 | unknown 被选中；P5 action 仍为 OK；reason 为空 | 完成 P1-5、P2-5、P4-5 与 R-2/R-5，保持 fail-closed。 |
| P5 final gate 与 planner FSM 形成 replan/emergency storm | 中 | 高 | 高频拒绝、轨迹饥饿、action 振荡 | 完成 A-ALL、R-9、R-10；验证 debounce、retry budget 和 safe fallback。 |
| RINEX/真实 GNSS 与 synthetic 行为差异 | 中 | 高 | 星历文件缺失、星座 fault hypothesis 导致 sentinel、时钟漂移异常 | 固定有效 RINEX 与时间源，分别报告 synthetic/RINEX，不混用阈值。 |
| 全工作区 lint/CI 失败掩盖真实回归 | 高 | 中 | 功能 GTest 绿但 `colcon test-result` 红 | 建立 functional 与 quality 两层 gate，逐包清理 lint，不把历史 lint 失败当算法失败。 |
| 本地 ahead 153 导致证据无法异机复现 | 中高 | 高 | 远端缺 commit；manifest hash 在别处无法 checkout | 经授权后备份/推送或建立受控镜像，并用 commit hash 绑定每次 campaign。 |
| shutdown helper 崩溃污染自动化 | 中 | 中 | SIGINT 后 RCLError/system_error/segfault | 区分 evidence-complete 后 teardown 与运行期崩溃；最终仍应修到 CI clean。 |

## 7. 建议执行顺序与审计门

1. **先治理 W0**：没有磁盘余量和可远端复现 commit，不启动新的大 campaign。
2. **实现 Phase 3 v2 harness，但不改阈值追结果**；先完成 P1-1/P1-2 的确定性同 snapshot 机制门。
3. **完成 P1-3 至 P1-6**；只有 P1 v2 全部通过，才进入 P2 正式验收。
4. **P2 优先关闭 double-count**，再做 P3/P4 fixtures；这些可以在不违反串行 fresh-pair 协议的前提下部分并行开发。
5. **单独重跑 official Demo11 legacy/enabled**，不要用 safety-planner 专项结果替代；同时关闭 URG 1000 ms 门。
6. **按 A-0 → A-ALL 做消融**，再做关键 robustness，最后才形成“联调成功”声明。
7. 每个阶段只接受同一 commit、完整 manifest/provenance、validator/analyzer 正常退出、产物哈希可验证的结果。

建议每周审计看板只保留以下硬指标：

- 当前 commit/dirty state/远端可复现性；
- 磁盘可用空间与下一 campaign 最坏预算；
- 正式通过实验数（分母固定为 63，不将诊断运行计入）；
- P1 v2 当前 gate 与 fresh pair 数；
- Demo11 planner trajectory count；
- URG mean/P95 update latency；
- collision、P5 final reject、emergency storm 和 unknown-as-low-risk 计数；
- functional tests 与 lint/quality tests 分开统计。

## 8. 证据完整性与局限

### 8.1 当前仓库状态

审计开始时：

```text
## dev/iap...origin/dev/iap [ahead 153]
HEAD b6c8236bbbf9858c65c1c5120fd4a3c052399700
```

最近提交冻结了 P1 Phase 3 v2 与历史 evidence archive；没有改变 planner runtime 或产品 P1 状态。对应 `docs/TRACEABILITY.md:475-481`。

### 8.2 主要一手证据

- 需求与验收定义：`docs/REQS.md:1-425`。
- P0–P5 测试矩阵与最终 hard pass：`docs/dev_planner/safety_planner_p0_p5_test_plan.md:246-447`。
- P0/P5/P1 历史运行事实：`docs/dev_planner/safety_planner_test_report.md:1-9786`，其中最新 P1 结论为 `:9710-9774`。
- P1 状态与协议追溯：`docs/TRACEABILITY.md:445-481`。
- Predictor 一手实验：`docs/dev_predictor/predictor_test_report.md:44-72`、`:3336-3363`。
- 旧 Demo11 full acceptance：`docs/stage1_v2.0/implementation_audit_v2.md:357-430`、`:514-582`。
- 运行入口与闭环链路：`README.md:488-704`。
- 测试产物：`/home/dev/ws_iap/build/iap/Testing/20260810-0840/Test.xml` 及各 package `test_results` XML。

### 8.3 局限

- 本次没有重跑 ROS campaign，因此 runtime 判断依赖仓库已经保存的正式报告和 artifacts。
- 旧 `SPEC_VS_IMPL.md` 与 `REQS.md` 的早期状态存在滞后/重复；本报告优先使用时间更晚的 TRACEABILITY、正式测试报告和实际源码/测试结果。
- 工期是条件估算，不是承诺。P1 科学结果、URG 性能和真实外场条件是主要不确定性。
- “85%–90% 代码成熟度”是基于模块存在性、测试和接口覆盖的工程判断；唯一可机械复核的正式验收比例是现行矩阵的 18/63。

## 9. 最终审计意见

当前 IAP 的准确表述应是：**核心完整性与预测模块已实现，P0 数据底座和 P5 hard safety gate 已完成正式联调；规划 preference stack 具备代码与单测基础，但 P1 v2 尚未 fresh 验证，P2–P4、全栈消融、压力与 official Demo11 尚未正式收口。**

在不降低现行审计标准的前提下，一名工程师完成 M2 的最可能工期为 **11–16 周**。若现在对外宣称“系统已经联调成功”，证据不足；更稳妥的项目状态是：**模块开发后期、正式系统验收前中期，当前关键路径为 P1 Phase 3 v2 → P2–P4 → Demo11/URG → A-ALL/Robustness。**

## 10. 关键数字复核命令

以下只读命令可复核本报告使用的仓库、测试、容量和矩阵数字；它们不会启动
ROS campaign：

```bash
cd /home/dev/ws_iap/src/iap

git rev-parse HEAD
git rev-list --count origin/dev/iap..HEAD
git status --short --branch

colcon test-result --test-result-base /home/dev/ws_iap/build/iap --all --verbose
colcon test-result --test-result-base /home/dev/ws_iap/build/ego_planner --all --verbose

du -sh results/planner_validation
df -h /home/dev/ws_iap

rg '^\| (B0|P0|P5|P1|P2|P3|P4|A-|R-)' \
  docs/dev_planner/safety_planner_p0_p5_test_plan.md
```

最后一条需要按唯一 ID 去重；冻结矩阵共有 63 个具名门槛，其中当前正式通过
18 个、剩余 45 个。`ego_planner` 结果中的 lint failure 与功能 GTest 必须分开
读取，不应把 lint 条目解释为同数量的运行时算法失败。
