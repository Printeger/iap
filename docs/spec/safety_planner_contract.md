# Safety Planner Far-Boundary Contract

**Status:** Draft — migration target

**Owner:** IAP Safety Planner

**Version:** 1.0.0-draft

This document defines the durable external contract of the Safety Planner. It
describes the result the module owns, the facts that must retain their meaning,
and the seams at which planning and execution decisions become authoritative.

While this document is `Draft`, it records the intended contract and does not
claim that the current implementation conforms. `MUST`, `SHALL`, and `MUST NOT`
describe requirements that become normative when the contract is activated.

Version-specific architecture choices belong in an ADR. Scenario values and
PASS/FAIL thresholds belong in a versioned acceptance contract. The current
companion documents are:

- [ADR 0004: Single-route Safety Planner v1](../adr/0004-single-route-safety-planner-v1.md)
- [Safety Planner v1 acceptance contract](safety_planner_acceptance_v1.md)

## 1. Owned result

The Safety Planner extends EGO Planner with navigation-integrity-aware route
reasoning. Its end-to-end responsibility is to obtain and maintain an
executable local trajectory that:

- makes progress toward the active mission objective;
- satisfies EGO's physical, dynamic, and continuation requirements;
- uses navigation-integrity evidence according to the selected task policy;
- has been checked in its final executable form before publication; and
- remains subject to one runtime authority for continue, replan, and emergency
  stop decisions.

EGO remains the motion-trajectory generation, publication, and execution
framework. Safety Planner may add evidence, route preference, and assurance,
but it MUST NOT create a parallel execution authority above EGO.

## 2. Responsibilities outside the module

Safety Planner does not own:

- state estimation or the certified current Integrity result;
- generation of GNSS or LiDAR advisory predictions;
- vehicle control or `traj_server` execution;
- the physical meaning of EGO occupancy;
- simulation oracle/reference data; or
- a second authoritative planning map or execution state machine.

Internal adapters may cache or transform inputs, but they MUST preserve the
source identity and MUST NOT become independent authorities for the same fact.

## 3. Domain semantics

### 3.1 Physical occupancy and local motion safety

Physical occupancy describes obstacles in the EGO planning map. Local motion
safety covers collision, clearance, vehicle envelope, tracking allowance,
dynamics, continuation, and reachable braking behavior.

Navigation-integrity risk MUST NOT be written into, or presented as, physical
occupancy. A navigation-risk failure MUST NOT be reported as a collision.

### 3.2 Current and advisory protection levels

**Current PL** is the HPL/VPL associated with the current localization
solution. It is runtime evidence about the vehicle's present navigation state.

**Advisory PL** is predicted HPL/VPL for a planning query. Its interface may be
spatial or spatiotemporal:

```text
advisory PL = advisory PL(position [, prediction_time])
```

The query model, reference epoch, evidence generation, and validity status are
part of its identity. Current PL and advisory PL MUST NOT substitute for one
another.

### 3.3 Navigation alert policy

HAL and VAL are positive navigation-policy limits:

```text
HAL > 0
VAL > 0
```

They may change only through an explicit, identifiable policy transition, such
as a mission-phase change. The selected policy is frozen within one planning
attempt.

Physical clearance, tree distance, vehicle radius, tracking allowance, and
braking distance MUST NOT silently modify or masquerade as HAL or VAL.

### 3.4 Navigation Integrity Margin and normalized risk

For valid HPL/VPL and positive HAL/VAL:

```text
IM_H = HAL - HPL
IM_V = VAL - VPL
IM   = min(IM_H, IM_V)

R = max(HPL / HAL, VPL / VAL)
```

Therefore:

```text
R < 1  <=> IM > 0
R = 1  <=> IM = 0
R > 1  <=> IM < 0
```

An implementation MUST retain both horizontal and vertical quantities. The
following scalar shortcut is diagnostic-only and MUST NOT authorize motion:

```text
AL = min(HAL, VAL)
PL = HPL
IM = AL - PL
```

Navigation Integrity Margin and local geometric clearance margin are different
facts even though both are measured in metres.

### 3.5 Guide, candidate, and active trajectory

- A **guide** is geometric route information. It has no execution authority.
  Its nominal arc sampling determines terminal approach and initial total
  duration. EGO subdivides the guide arc sampling for initialization to spacing
  no larger than the existing map voxel diagonal without changing that boundary P/V/A or nominal duration. The nominal
  interval is an explicit input; the finer output interval is never reused
  as that input after target replacement or repeated fitting.
  The denser mesh spends the original shared deadline and repair quota. All
  actual-curve, dynamics, physical, route and publication gates remain mandatory.
- A **candidate trajectory** is an actual time-parameterized EGO trajectory
  proposed for publication.
- An **active trajectory** is the trajectory currently owned by the execution
  framework.
- An **assurance result** binds a checked candidate, the evidence and policy
  used to check it, and a typed PASS or failure.

## 4. External interface

### 4.1 Planning context

One planning attempt consumes a logically immutable `PlanningContext`
containing, as applicable:

- planning start state and timestamp;
- mission goal or local objective;
- physical-map generation and spatial-frame contract;
- advisory provider generation, reference epoch, and query semantics;
- current Integrity evidence identity;
- HAL/VAL policy and navigation task mode;
- active-trajectory identity and continuation boundary;
- relevant planning and assurance policy versions; and
- planning deadline or compute budget.

Physical and advisory data may use different storage, resolution, and update
times. Their frames, coordinates, query interpretation, and validity MUST be
related explicitly. They MUST NOT silently disagree.

Search, trajectory generation, assurance, and publication MUST NOT combine
incompatible context generations under one planning-attempt identity. Newer
evidence may invalidate unpublished work or start another attempt; it MUST NOT
silently mutate an in-flight context.

### 4.2 Planning result

Each attempt returns one `PlanningResult` with these semantic fields:

```text
context_identity
candidate: optional
disposition: READY | REJECTED | NO_SOLUTION | TIMEOUT | CANCELLED
primary_reason
contributing_reasons[]
assurance_result: optional
```

The concrete type names may differ. The semantics shall not.

`READY` requires one candidate with a matching PASS assurance result. A result
without a candidate does not revoke an independently valid active trajectory.

The runtime authority separately produces:

```text
active_trajectory_identity
action: CONTINUE | REPLAN | EMERGENCY_STOP
reason
evidence_identity
```

A planning result may trigger evaluation by that authority, but it cannot
choose the runtime action on the authority's behalf.

`HOLD` is an observable system condition in which no new executable
continuation is available. It may be derived from a planning result and runtime
state; it is not required to be a new EGO FSM state.

### 4.3 Failure taxonomy

The interface MUST preserve at least these failure classes:

- physical collision or clearance;
- known navigation-integrity violation;
- advisory unknown, stale, invalid, or unavailable;
- route-search failure;
- trajectory generation or optimization failure;
- dynamics or feasibility failure;
- continuation-boundary failure;
- evidence identity, freshness, or clock failure;
- stale or late candidate;
- publication or trajectory-identity failure; and
- compute-budget exhaustion.

One primary failure identifies the authoritative seam that rejected the work.
Other observed failures may be retained as contributing reasons. Pipeline order
alone MUST NOT erase a more authoritative cause.

## 5. Authority seams

Each major conclusion has one authority. Callers consume the conclusion and
its identity; they MUST NOT independently reinterpret or weaken it.

| Decision | Sole authority | Explicit limit |
|---|---|---|
| Advisory evidence and validity | Predictor/Integrity evidence provider | Does not authorize execution |
| Route decision | Configured route-reasoning seam | Produces route intent, not execution permission |
| Candidate generation | EGO trajectory generator/optimizer | Does not publish |
| Final candidate assurance | Trajectory-assurance seam | Returns PASS or typed failure; does not reroute |
| Publication | Existing EGO planner/FSM publication seam | Publishes only identity-matching assured work |
| Runtime action | Existing EGO runtime supervision | Decides continue, replan, or emergency stop |

Search and optimization may explore alternatives internally. At most one route
decision and one candidate result cross their respective authority seams for a
single attempt. Internal alternatives MUST NOT acquire publication authority or
form a parallel execution lifecycle.

Whether v1 generates actual trajectories for more than one topological route
inside an attempt is an architecture decision governed by ADR 0004, rather
than a permanent safety invariant of this contract.

## 6. Navigation task policy

The task mode is an explicit, versioned input to the planning context. It MUST
be carried through search, assurance, publication evidence, and runtime
supervision without silent substitution.

### 6.1 `STRICT_GLOBAL`

For valid global-navigation evidence:

```text
R >= 1
```

is an integrity-limit violation and cannot receive global-navigation execution
authorization. Missing or invalid evidence fails according to the configured
strict unknown policy.

### 6.2 `MISSION_BEST_EFFORT`

Global-navigation exceedance remains typed evidence for route preference,
diagnostics, and explicit mission-policy actions. It MUST NOT be relabeled as
physical unsafety.

When the configured best-effort policy grants local authority, motion still
requires independently complete and SAFE local motion assurance. Global risk
alone cannot grant local safety, and favorable local clearance cannot rewrite
the global risk measurement.

Any exposure threshold, episode budget, recovery rule, or early-replan action
belongs to the named task policy and must remain observable. It is not an
implicit change to HAL/VAL or to the definition of IM/R.

### 6.3 Unknown advisory evidence

```text
UNKNOWN != SAFE
UNKNOWN != KNOWN_UNSAFE
```

Unknown, stale, or incomplete advisory evidence MUST retain its status. It
MUST NOT be replaced by PL = 0, a synthetic low-risk value, or physical
occupancy.

Its operational disposition—blocked, finite-cost, bounded degraded exposure,
or HOLD—is selected by a named, versioned policy frozen in the planning
context. Changing that policy is not a tuning-only change.

## 7. Candidate generation and assurance

Trajectory generation consumes route intent and the exact start or
continuation boundary. It may use bounded EGO-native repair, retiming, or
multiple numerical seeds as private implementation details.

Final assurance checks the actual candidate after all applicable optimization,
repair, and retiming. If the candidate changes in a way that can affect an
assured property, that property MUST be checked again.

Final assurance covers, as applicable:

- physical collision and clearance;
- dynamics and feasibility;
- local tracking and braking envelope;
- navigation task policy;
- continuation consistency;
- evidence identity and freshness;
- trajectory identity and timing; and
- the decision deadline.

An assurance failure cannot crop, reroute, retime, or reinterpret the failed
candidate into a passing candidate. Such changes create new work that requires
another final assurance result.

Publication validates the assurance result's identity and freshness. It MUST
NOT replay a different policy or add a second route-selection decision.

## 8. Runtime and fallback semantics

Runtime helpers such as guards, braking verifiers, or recovery planners may
exist, but they remain subordinate to one EGO runtime authority. They MUST NOT
publish independently or maintain a parallel execution state machine.

When new evidence predicts that the active trajectory may lose an applicable
hard condition, the runtime authority may request replanning. Replanning uses
the active trajectory's actual continuation state.

Failure to obtain a replacement trajectory does not by itself prove that the
active trajectory is unsafe. A rejected candidate MUST NOT overwrite or revoke
an independently valid active trajectory. Revocation requires runtime evidence
and policy applicable to that active trajectory.

If continued execution is no longer authorized and no assured replacement is
available, the existing EGO runtime authority invokes its configured emergency
behavior. This contract does not claim that an implementation is formally
certified unless a separate active contract establishes that claim.

Fallback MUST NOT:

- turn unknown advisory evidence into safe evidence;
- ignore physical collision or failed local motion assurance;
- alter HAL/VAL to make a trajectory pass;
- reuse an assurance result for a different trajectory or context;
- convert optimizer failure into permanent map occupancy;
- permit stale work to take control; or
- retry an unchanged deterministic failure indefinitely.

## 9. Safety and liveness invariants

Every conforming implementation preserves these invariants:

1. Physical occupancy, local motion safety, and navigation integrity remain
   separately identifiable.
2. HAL/VAL come from an explicit navigation policy and are frozen per attempt.
3. Authoritative integrity uses HPL, VPL, HAL, and VAL independently.
4. `IM = min(HAL-HPL, VAL-VPL)` and `R = max(HPL/HAL, VPL/VAL)`.
5. Current PL and advisory PL cannot substitute for one another.
6. Unknown evidence is never interpreted as safe evidence.
7. A guide has no execution authority.
8. The final executable trajectory is assured after relevant modification.
9. Only an identity-matching PASS result can authorize publication.
10. A rejected candidate cannot overwrite a valid active trajectory.
11. Late, stale, cancelled, or identity-inconsistent work cannot take control.
12. Route reasoning, assurance, publication, and runtime action do not usurp
    one another's authority.
13. Planning time, retries, concurrency, memory, and retained history are
    bounded by versioned configuration.
14. Stable inputs and a feasible solution lead to progress or a bounded typed
    outcome; they do not leave an attempt pending indefinitely.

No solution, timeout, or HOLD is a legitimate result. It MUST NOT be reported
as mission success.

## 10. Identity, time, and freshness

Externally meaningful decisions are attributable to at least:

- planning-attempt identity;
- candidate and active-trajectory identity;
- start time or continuation boundary;
- physical-map generation;
- advisory/evidence generation and reference epoch;
- frame/spatial contract;
- navigation policy and task mode; and
- assurance-policy version.

Physical-map freshness, advisory freshness, current-Integrity freshness, and
active-trajectory freshness are independent. Freshness of one MUST NOT imply
freshness of another.

All execution-relevant time comparisons use an explicit clock domain. A newer
generation may invalidate unpublished work, but an older asynchronous result
MUST NOT replace a newer authoritative result.

## 11. Configuration boundary

Policy-level configuration includes:

- HAL/VAL policy and transition identity;
- task mode;
- advisory validity and unknown-evidence policy;
- physical and local-motion assurance requirements;
- exposure/replan/emergency policy;
- planning and execution horizons; and
- compute and resource budgets.

Changing a policy-level value requires versioned configuration and appropriate
contract or acceptance evidence.

Internal tuning includes search heuristic details, soft-cost shape and weight,
optimizer weights, numerical tolerances, sampling density, cache layout, and
thread scheduling. These may change without changing this contract when the
external semantics, authority seams, and invariants remain intact.

## 12. Observability and resource bounds

Bounded logs, metrics, and run artifacts must make high-level outcomes
explainable. When applicable they identify:

- planning trigger and attempt identity;
- input evidence and policy identities;
- route result;
- candidate and assurance result;
- primary and contributing failure reasons;
- publication disposition;
- runtime decision and active-trajectory identity;
- runtime replan or emergency reason; and
- deadline or resource exhaustion.

Repeated diagnostics are rate-limited. Explainability does not require
unbounded point-by-point logs, candidate histories, or raw evidence retention.

A timeout while producing a replacement candidate does not by itself revoke an
otherwise valid active trajectory.

## 13. Standard acceptance relationship

The standard Safety Planner integration scenario is:

```text
icra_dense_forest_four_fork_v2
```

Its scene version, launch profile, task mode, HAL/VAL, Oracle definition,
sampling, repetitions, numerical thresholds, and PASS/FAIL rules are defined
in [the versioned acceptance contract](safety_planner_acceptance_v1.md).

Simulation Oracle/reference evidence may use information unavailable to the
online system. It is a judge of actual execution and MUST NEVER be consumed by
the online planner, predictor, assurance, publication, or runtime path.

End-to-end acceptance is based on actual commands, odometry, and executed
trajectory behavior. A plausible guide, a passing unit test, or an RViz image
is not sufficient evidence of end-to-end acceptance.

## 14. Compatibility and activation

These are far-boundary breaking changes:

- merging physical occupancy and navigation-integrity risk semantics;
- changing the authoritative IM/R definitions;
- allowing current PL and advisory PL to substitute for one another;
- allowing implicit physical-clearance changes to HAL/VAL;
- moving publication or runtime action to a second authority;
- allowing assurance or runtime checking to perform hidden route selection;
- allowing online code to consume simulation Oracle/reference data;
- weakening identity/freshness rules so stale work may execute; or
- changing the meaning of a task or unknown-evidence policy without a version.

Class layout, file layout, storage, algorithms, caches, logging implementation,
and performance optimizations are not contract changes when observable behavior
and the authority seams remain consistent.

This contract may become `Active` only after:

1. its terminology and task-policy semantics are reviewed;
2. implementation conformance gaps are recorded or removed;
3. interface and authority tests cover the normative invariants; and
4. the versioned acceptance contract is active and has a retained passing run.

Curve 初始化采样模型 `guide_arc_voxel_diagonal_v1` 的 `guide_fit` 权威阶段同时冻结
`nominal_interval_s`（本轮名义输入）与 `interval_s`（细化后的实际输出）。
初始化重放使用明确的名义输入，缺失名义值或未知模型时拒绝；旧无模型字段的粗采样
捕获仅按 `LEGACY_COARSE_STAGE_INTERVAL` 历史规则读取。输出分别登记捕获／应用模型，
不更新冻结时刻或授予执行资格。TargetShortening 重拟合仍使用本轮原名义输入。

Remaining-curve evidence retains the assessment-owned immutable physical epoch even when
the FSM adds tracking or swarm rejection after a successful physical scan. Supplemental
cells use that original epoch, evaluation time and motion. `state.json` distinguishes
`capture_ros_time_s` from `assessment_time_s` and `assessment_generation`; the former
is capture receipt, while the latter identifies the frozen check. Generation checks and
all execution decisions remain mandatory.

When pending physical rejection and active tracking error coexist, the pending failure
retains its cause, curve ID, local violation time and position. Active tracking is a
separate reference observation in stop-state fields; it must not rename a pending
curve failure. Withdrawal and checked recovery retain their existing authority.
`curve_first_execution_time_s` and `curve_first_execution_position_m` identify the
first violation in the saved owning curve; for pending evidence the time is local
to that pending curve.


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


### 安全监督消费真实执行反馈

FSM执行入口和安全监督入口均先消费同一份最新position command，经既有
observeExecutingTrajectory确认active/pending所有权，然后绑定被检查曲线、局部时间与
跟踪期望。安全定时器不得等待执行定时器才更新实际command ID；仅经过scheduled时间
不授予激活。原命令身份／撤销确认规则、物理拒绝、紧急窗口和预算保持。
此入口修复不保证检查过程中发生的server切换或反馈尚未送达时的竞态已消除。


安全入口消费机制已完成Release及7组规划／43入口／4到达回归，干净300秒现场
91发布／83命令ID／53激活。仍有已生效48的撤销晚约2ms和3次checked brake拒绝；
不得以机制修复授予全部接续资格。原时间2828请求按2800空间和28receiver分母分别
登记，GNSS＋LiDAR information标志不能替代raw故障界限／米数／联合故障资格。
当前首违owner与时序／输入hash证据见
[现场报告](../../log/20261007T091820Z_056/export/analysis/safety_feedback_live/report.md)。


独立历史轨道身份补证：IGS带时间区间SATELLITE/PRN＋IDENTIFIER与本地NAV全部
44个北斗PRN在2022-07-06T12:00UTC匹配，2022生产GEO类别差异0。C59→C217、
C60→C229均BDS-3G，历史有效区间覆盖300秒。2026已有重分配，不能把PRN-only
类别实现外推为2026或未来输入资格；健康／信号及Advisory资格仍分开。原HTTP bytes、
headers／UTC／metadata与NAV hash及全映射见
[历史轨道身份独立补证](../../log/20261007T091820Z_056/export/analysis/rinex_identity/report.md)。
只关闭独立身份依据缺口，旧报告与RTKLIB数值验证保持原身份。
