# ICRA 2027 P0 + P2 + P5 改造计划批判性评审

> 评审日期：2026-08-16
>
> 评审基线：`dev/icra`，冻结代码 `21180f3`，会议文档提交 `e2ba505`
>
> 性质：代码约束下的工程与论文可行性意见，不是算法结果或实验结论。

## 1. 总体结论

**架构方向合适，但计划按当前定义和排期直接执行，只能给出“有条件可行”，不能给出无条件 Go。**

- 保留 P0、P2、P5，会议范围关闭 P1/P3/P4，是正确的收缩；它复用了当前最成熟的 P0/P5 路径，也避免把尚未形成稳定系统证据的模块塞进 8 页论文。
- 把 P0/Predictor 定义为 advisory、P2 定义为 preference、P5 定义为独立 hard gate，职责分工清楚，适合形成单一论文主线。
- 当前 P5 normal publish 前置顺序已经成立，P0 也已经有不可变 generation/snapshot 和性能诊断基础；真正需要改造的核心主要集中在 P2 的证据契约，而不是重写 P0/P5。
- 但是，当前代码尚不能严格证明“P2 比较的每个候选都已满足完整的原始动力学可行性约束”；P1 关闭后能否稳定产生两个以上 base candidates 也未经系统证明。这两点直接决定核心 claim 是否成立。
- 计划总量为 41 人日，而 2026-08-17 至 2026-09-02 只有 13 个工作日；三人理论上只有 39 人日，尚未计入集成、失败重跑和论文协作。因此**当前排期在数学上已无缓冲，按原样不可行**。
- 当前磁盘只有约 32 GiB 可用，而计划自己的 campaign gate 至少要求 40 GiB；正式 campaign 目前是明确 No-Go。

我的建议是：**保留目标架构，缩小实现面，把最不确定的“P1-off 多候选”和“候选是否真的可行”提前到第一技术门；只要这两个门在 8 月 18 日前通过，会议版仍然可做。**

## 2. 计划中值得保留的决定

1. **P2 不生成候选。** 当前 `rankP2Candidates()` 只接收 `const std::vector<P2CandidateInput>&`，候选生成仍在 `BsplineOptimizer::distinctiveTrajs()`：`src/iap/planner/plan_manage/include/ego_planner/p2_candidate_ranking.h:71`、`src/iap/planner/bspline_opt/src/bspline_optimizer.cpp:474`。这个 seam 应保持不变。
2. **P2 enabled 路径使用 original cost。** 当前计算明确读取 `cost_breakdown.original_cost`：`src/iap/planner/plan_manage/src/p2_candidate_ranking.cpp:233-256`。继续用它并对 `total_cost` 做反向陷阱测试是合适的。
3. **P5 final gate 已在正常轨迹发布前。** `P5RuntimeIntegrityGate::evaluateFinal()` 位于 `ego_replan_fsm.cpp:1094-1128`，正常 `bspline_pub_->publish()` 位于 `:1152-1153`；非 OK 会恢复旧 `local_data_` 并返回 false。这里应补证据，不应重构主流程。
4. **P5 使用最新 snapshot，而不是继承 P2 decision。** 这符合独立 integrity authority 的工程语义。P2 与 P5 应分别记录 generation ID，不应强行要求同代。
5. **P1/P3/P4 只从会议 profile 关闭，不删除源码。** 这既保护现有测试和期刊路线，也避免维护 `#ifdef ICRA` 分叉。
6. **失败、fallback 和缺失场景进入统计。** `BLOCKED_SCENARIO_MISSING`、失败 run 不覆盖、阈值预注册等规则是可信实验所必需的，应保留。

## 3. 必须先解决的关键问题

### 3.1 “成功候选”目前不等于“全部通过原始动力学可行性检查”

当前 manager 在 rebound optimizer 返回 success 后立即把候选加入 `p2_candidates`，随后在 `planner_manager.cpp:1381` 执行 P2 排序：

- 候选收集：`src/iap/planner/plan_manage/src/planner_manager.cpp:1273-1351`；
- P2 ranking：同文件 `:1374-1383`；
- 只对选中候选构造 `UniformBspline`：`:1866`；
- 真正的 `pos.checkFeasibility()` 在选中之后才发生：`:1875-1882`；必要时还会 reallocate/refine。

因此，当前输入集合准确的名字是 **rebound-optimizer-success candidate set**，不是“每个候选都已通过最终 dynamics feasibility 的 feasible set”。`BsplineOptimizer::combineCostRebound()` 中存在 feasibility cost，不等于每个候选都已通过后续 hard feasibility check。

建议二选一，并在写代码前冻结：

- **推荐会议方案：收窄论文措辞。** 把候选集合称为“同一次 attempt 的 rebound-optimizer-success candidates”，明确 EGO 的后续 feasibility/refinement 定义保持不变。这样无需把昂贵的 per-candidate refinement 前移。
- **若必须保留 feasible candidate claim：** 在 P2 前增加一个只读 eligibility step，用现有 collision/dynamics 判定检查每个候选，只把全部通过者形成集合；P2 仍不得生成、修改或 hard reject。若 eligibility 后只剩一个候选，则 P2 no-op。不得把“只检查最终 winner”写成“所有候选可行”。

在没有完成上述选择前，`ICRA_SCOPE.md` 中 “same feasible candidate set” 的英文 claim 偏强。

### 3.2 P1 完全关闭后，多候选集合可能根本不存在

`BsplineOptimizer::distinctiveTrajs()` 在 collision segments 为空时明确只返回一个候选：`bspline_optimizer.cpp:481-487`。当前 P1 collision fanout 能把 singleton 扩成多个候选，但会议 profile 正确地要求关闭该路径：`planner_manager.cpp:1093-1125`。

这意味着 P2 论文的最大风险不是 ranking 公式，而是 base EGO 在目标场景中是否稳定产生至少两个 optimizer-success candidates。现有 `p2_degraded_lidar_good` preset 只打开 `manager/use_distinctive_trajs`，没有正式 analyzer 证明多候选：`launch/test_planner.launch.py:677-683`。

建议把 W2-07/W5-01 提前为 **Gate 0**：

1. 在 P1/P3/P4 全部 effective=false 时，用 seed 11 跑 primary、mirror、null 三个资格场景；
2. 只检查 `segments`、generator candidate count、optimizer-success count 和 candidate control points，不先开发完整 campaign；
3. 若不能稳定得到两个候选，立即选择：补一个位于 P2 之前的 deterministic upstream seed fixture，或在 8 月 18 日提前切换 P0+P5 备用路线；
4. synthetic fixture 必须在论文中标为 controlled fixture，不能包装成自然出现的 planner diversity。

等到 8 月 22 日才发现这个问题太晚，会浪费 W0/W1/W6 的投入。

### 3.3 original winner 目前有两种语义

当前 P2 disabled 路径按 `final_cost` 选 winner：`p2_candidate_ranking.cpp:225-230`；metrics-only/fallback 则按 `cost_breakdown.original_cost`：`:260-284`。`final_cost` 是 L-BFGS 返回的 total objective：`src/iap/planner/bspline_opt/src/bspline_optimizer.cpp:2373-2399`。

P1 完全关闭时两者理论上应相等，但“metrics-only 绝不改变 winner”不能只依赖这一假设。建议：

- manager 在构造 candidate set 时只计算一次 `original_winner_id`，并把它作为 immutable batch 字段传给 P2；
- metrics-only、null、single、stale、unknown、NaN fallback 都直接返回该 ID，不重新实现 comparator；
- ICRA profile 增加断言：每个候选的 `final_cost` 与 `OptimizerCostBreakdown::original_cost` 在冻结容差内一致；不一致时 attempt evidence invalid；
- 计划中的实际类型名应从不存在的 `ReboundCostBreakdown` 改为 `BsplineOptimizer::OptimizerCostBreakdown`，见 `bspline_optimizer.h:179`。

### 3.4 fallback 规则存在内部矛盾

计划一方面规定“任一样本 unknown/stale 即 fallback”，另一方面又提出 `P2 all-candidate minimum valid ratio = 0.30`。若任一 unknown/stale 都触发 batch fallback，那么 enabled comparison 实际要求每个候选 `valid_ratio=1.0`，0.30 门槛不再产生作用。

会议版应只保留一个可解释规则：

- **最保守且最容易 defend：** 正式 P2 selection 只接受所有候选、所有冻结采样点均 valid 且非 stale；否则整个 batch 返回 original winner。删除 selection path 的 0.30 语义，只把 ratio 当诊断指标。
- **更实用但更复杂：** 所有候选必须达到同一 valid-ratio 门，并具有可比较的 support mask；任一 stale 或非有限值仍使整个 batch fallback。该方案需要定义 support correspondence，不适合临近 deadline 时临时引入。

建议会议版先采用第一种，并在 Gate 0 观察 fallback rate；如果资格场景几乎全部 fallback，就按预注册规则切备用论文，不能看完正式结果再放宽门槛。

### 3.5 当前 score 的归一化可能放大数值噪声

当前实现为：

```text
(original_cost - min_original) /
(max_original - min_original + 1e-9)
```

见 `p2_candidate_ranking.cpp:233-256`。这种 per-attempt min-max normalization 会把很小的 original-cost span 也映射到接近 `[0,1]`，使 `lambda_candidate_integrity` 的含义随 candidate set 改变；计划中的 null/tie epsilon `1e-9` 对以米为单位的 PL 和真实 predictor 重复性也过于接近纯浮点容差。

更容易写进论文的方案是 **original-cost budget + risk preference**：先用预注册的 absolute/relative cost budget 定义“原始代价可接受集合”，再在集合内选最低 integrity score。它更符合“P2 是 preference，不是新的 feasibility/safety authority”。

如果因时间原因保留当前 weighted sum，则必须：

- 报告每次 attempt 的 raw cost span、normalized term 和 selected-vs-original cost degradation；
- 用资格运行的数值重复性冻结 tie epsilon，而不是直接使用 `1e-9`；
- 将论文 claim 限定为“在冻结 scoring rule 下偏好较低预测风险”，不能暗示无代价地普遍降低风险。

### 3.6 P2 排名对象与最终发布轨迹不是同一个几何/时间对象

P2 在 refinement 前对 control points 和旧 `ts` 评分；选中后可能执行 `refineTrajAlgo()`、time reallocation，并最终在 `updateTrajInfo()` 写入 `LocalTrajData`：`planner_manager.cpp:1875-1886,2383-2404`。P0 query 又显式依赖 future time，因此只保存 pre/post hash 还不够。

建议 W3-01 除 identity binding 外，增加两组只读指标：

- `selection_time_metrics`：P2 实际用于比较的候选分数；
- `publication_time_metrics`：对最终 refined trajectory 重新查询相同 P2 decision snapshot，仅用于审计，不再改变 winner。

论文必须区分“P2 选择了哪个 seed/candidate”和“最终发布轨迹的预测 PL/IM”。若 refinement 后风险排序结论反转，应作为结果披露，不能只用 lineage hash 掩盖。

### 3.7 “P5 是唯一 hard safety authority”措辞过强

原始 EGO collision/dynamics checks 仍然是硬约束，P5 只是 IAP 层的 integrity admission/monitoring authority。P5 也依赖 P0 predicted-PL evidence，因此它独立的是**决策权威和状态机**，不是完全独立的预测数据源。

建议统一改为：

> P5 is the sole hard integrity gate in the IAP layer; the original EGO collision and dynamics checks remain authoritative for motion feasibility.

同时避免使用 “current certified monitor”。当前仓库实现了 current online integrity monitor，但本会议明确不主张 certification-level proof。更稳妥的名称是 “authoritative current-state monitor within the system”。P0 可以单向读取 current integrity 作为 prior（当前 `p0.predictor.use_current_integrity_prior=true`），所以应声称 logical one-way separation，而不是未经证明的 physical isolation。

## 4. 建议的 module 与 interface 形状

计划已经接近正确 seam，但不应把 attempt、set、candidate、snapshot、CSV 字段分别散落在 manager、P2、P5 和 analyzer。建议形成一个深 module：

```text
P2RankingBatch
  AttemptIdentity
    planning_attempt_id
    query_base_time_s
  SnapshotIdentity
    generation_id
    stamp_s
    frame_id
  SuccessfulCandidateSet
    candidate_set_hash
    candidates[]
    original_winner_id

P2RankDecision rank(const P2RankingBatch&, const P2RankingConfig&)
```

该 interface 应保证：

- batch 构造完成后 immutable；P2 只返回 decision，不写回 candidates；
- attempt/set/snapshot 一致性在构造时验证一次，调用者和 analyzer 不重复猜测；
- candidate ID 保持 attempt-local；candidate hash 用带 schema、frame、degree、interval、维度的 canonical serialization；
- 不建议用主机字节序的 raw IEEE-754 + FNV 作为跨平台证据 ID。优先用 canonical bytes/text 的 SHA256；若保留 FNV，必须记录 endian/schema，并用完整 control points 二次校验碰撞；
- mirror run 不要求 candidate-set hash 相同，应使用独立 `mirror_pair_id` 和反射后 RMS correspondence；
- P5 只接收 `P5DecisionBinding` 中的 identity/lineage，不接收 P2 score。

另一个 seam 建议是把 ICRA evidence writer 做成 adapter。`RiskGridSnapshot::Generation` 只增加查询语义真正需要的 frame/input provenance；full-grid latency、CSV schema、artifact hash 等实验信息继续放在 health/evidence adapter，不要把论文 writer 的全部字段塞进核心 snapshot interface。

## 5. P0 性能风险应提前测量

ICRA launch 当前配置是 `30×30×6 m`、`0.75 m` resolution、6 horizons、`0.5 s` refresh，worker count 为 1：`launch/test_planner.launch.py:894-914`。按 `ceil(size/resolution)` 计算，open space 最坏为：

```text
40 × 40 × 8 × 6 = 76,800 queries / full refresh
```

实际 full-grid path 在 `RiskGridMap::refreshFromProvider()` 中完整构造该批次：`src/iap/planner/risk_grid_map.cpp:911-971`；runtime 已记录 query count 和 refresh/provider elapsed：`p0_risk_grid_runtime.cpp:915-963`。

因此 W1 不应先扩展 provenance schema，再发现 400 ms 的 proposed latency gate 过不了。建议 Gate 0 同时跑一个 open-space worst-case benchmark，直接读取已有 `/planning/risk_grid_health`：

- 若 p95 已通过，不改 P0 架构，只补证据 adapter；
- 若不通过，优先缩小 ICRA ROI/horizon 或提高 worker count，并把覆盖范围写入论文；
- 不在会议周期实现跨-generation cache 或 incremental grid，这会扩大算法风险。

## 6. 排期与工作包应重新排序

当前 31 个任务合计 41 人日：W0=5、W1=4.5、W2=11、W3=5、W4=1.5、W5=8、W6=6。2026-08-17 至 09-02 只有 13 个工作日，三人满负荷上限为 39 人日。现计划没有任何 integration buffer，且 80 个正式 runs 的墙钟时间和失败诊断很可能被低估。

建议按以下顺序执行：

| Gate | 最晚时间 | 必须得到的事实 | 未通过时处理 |
|---|---|---|---|
| Gate 0A | 08-17 | P1/P3/P4 全关时，primary/mirror/null 的 base candidate count 与 success count | 无稳定多候选则立即补 upstream fixture 或转 P0+P5 |
| Gate 0B | 08-18 | 76,800-query P0 full refresh 的 p50/p95 和 stale rate | 缩 ROI/horizon/worker；仍不过则缩论文场景 |
| Gate 1 | 08-19 | 最小 ICRA profile、effective config validator、磁盘方案 | 未通过不开发 campaign runner |
| Gate 2 | 08-22 | immutable batch、original winner、metrics-only/fallback 全部 no-op | 不成立则 P2 不进入正式论文 |
| Gate 3 | 08-26 | enabled P2 effect、post-refinement audit、P5 publish order/authority | 不稳定则激活 P0+P5 备用路线 |
| Gate 4 | 09-02 | 冻结 campaign index、结果和 analyzer version | 之后只修分析错误，不改算法/阈值 |

为留出至少 20% 缓冲，建议削减或后移这些内容：

- W2-01 不在会议分支重命名 `p1_planning_attempt_seq_`；直接复用现有 `planning_attempt_id`，中性化重构留给期刊。
- W1-01 不把全部性能/文件 provenance 写入 `RiskGridSnapshot::Generation`；改为小型 evidence adapter。
- `/planning/p2_candidate_ranking` JSON topic 降为可选诊断；正式证据先以 candidate CSV + manifest 为准。
- W1-03 的 P0-only paired sanity 只保留 open/null 和一个 degraded case，不在所有场景扩大资格矩阵。
- W5-04 优先复用现有 recorder/finalizer，先做最小可恢复 runner；通用 campaign framework 后移。
- W6-03/W6-04 的完整 artifact packaging 和 caption-ready polish 可在结果冻结后继续，但 analyzer schema 与 raw artifact SHA 必须在正式 run 前冻结。

## 7. 实验和统计上的建议

1. **P2 因果证据只来自同一 attempt。** 同一 batch 同时保存 original winner、hypothetical score winner 和 actual selected winner；不要用 C0/C1 两次 ROS run 证明 P2 选择因果。
2. **C0/C1/C2 跨 run 只用于系统后果。** 它们比较 mission、latency、fallback、P5 actions；必须冻结 seed、scenario、software/config hash，并披露 candidate correspondence 失败率。
3. **mirror 是 correspondence，不是 hash equality。** 对 primary candidate control points 做 y 反射后与 mirror candidates 做 assignment/RMS；保存映射和 unmatched candidates。
4. **报告 effect size，不只报告 winner count。** 至少包括 selected-vs-original 的 predicted PL/IM 差、original-cost degradation、轨迹长度差、fallback rate 和置信区间。
5. **把 fallback rate 当主要结果。** 一个只在极少数全 valid attempts 生效的 P2 不能只展示成功案例；所有 attempts 应进入分母。
6. **P5 unsafe fixture 与 P2 场景分开。** P2 不承担 unsafe 判决；targeted unsafe 只证明 final no-publish 或 runtime action。
7. **P0+P5 备用论文需要单独评估贡献强度。** 它在工程上可行，但科研 novelty 可能弱于 P2 主线；应在 08-26 前准备清晰的备用 research question，而不是只删除 P2 图表。

## 8. 工程验收修正

- `ICRA_IMPLEMENTATION_PLAN.md` 中的 `BsplineOptimizer::ReboundCostBreakdown` 应改为实际类型 `BsplineOptimizer::OptimizerCostBreakdown`。
- 从仓库根运行 `colcon list` 当前只发现 `iap`，不会发现嵌套的 `ego_planner`。最终命令 `colcon test --packages-select iap ego_planner` 不可靠，应沿用冻结清单中的分离命令：顶层 `colcon test --packages-select iap`，以及 `/home/dev/ws_iap/build/ego_planner` 的独立 CTest/嵌套 colcon build。
- baseline full planner CTest 当前因 `flake8`、`lint_cmake`、`uncrustify` 返回 8，但 7/7 functional gtests 通过。会议 CI 应冻结 baseline failure signature 并执行 “no new failure”，不能把既有 lint 红灯误判为 P2/P5 regression，也不应在此时顺手格式化大量历史文件。
- 当前容量预算的粗上限已超过 32 GiB 可用空间；在外部归档、扩容或用户批准清理前，campaign runner 必须保持 fail-closed。

## 9. 推荐的论文主张措辞

若不把 per-candidate hard feasibility check 前移，建议使用：

> Given one planning attempt, the same rebound-optimizer-success candidate set, and one immutable predicted-PL snapshot, integrity-aware re-ranking prefers trajectories with lower predicted integrity-risk scores without altering candidate generation or EGO feasibility definitions. P5 remains the sole hard integrity gate in the IAP layer, while the original EGO collision and dynamics checks remain authoritative for motion feasibility.

中文对应：

> 在同一次 planning attempt、同一组 rebound optimizer 成功候选和同一个不可变 predicted-PL snapshot 下，完整性感知重排在不改变候选生成及 EGO 可行性定义的前提下，更倾向较低预测完整性风险的轨迹；P5 是 IAP 层唯一 hard integrity gate，原始 EGO collision/dynamics checks 继续负责运动可行性。

这比 “P5 is the sole hard-safety authority” 更准确，也与“不主张 certification-level proof”一致。

## 10. 最终 Go/No-Go 意见

**Go，但必须是带前置条件的 Go：**

- 若 08-18 前证明 P1-off base planner 能提供可比较的多候选，且 P0 full-grid latency 有预算，则 P0+P2+P5 主线可行；
- 若多候选只能靠 P1 fanout 才存在，不应为了保住 P2 claim 偷开 P1；只能补 P2 之前、定义清楚的 upstream fixture，或切换备用论文；
- 若候选集合仍只做 optimizer-success 而未逐一 hard-feasibility 检查，必须收窄 feasible-set claim；
- 若正式 campaign 前磁盘仍低于 gate，实验不可启动；
- 不建议在会议周期重写 P0、改轨迹表示或恢复 P1/P3/P4。

简言之：**“P0 advisory + P2 same-batch preference + P5 hard integrity gate”是合适的会议架构；当前计划最大的问题不是架构，而是 claim 对候选可行性的表述过强、P2 treatment domain 尚未真实存在、证据基础设施过宽且排期无缓冲。先解决这三个问题，计划才真正可执行。**
