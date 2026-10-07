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
