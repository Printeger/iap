# ICRA P0 -> P4 -> P5 deviation audit and scientific recovery roadmap

Status: **USER-OWNED ROUTE LOCK / DEVELOPMENT-FIRST P4-v2 END-TO-END RECOVERY**
Decision date: 2026-08-26
Requirements: `IAP-RQ-000`, `IAP-RQ-423`, `IAP-RQ-424`

## Development-first acceleration decision — 2026-08-26

User decision `USER-ICRA-ROUTE-20260826-002`, bound to pushed anchor
`b24a330d79d6e85e8080cf2a359bb1a18765e5a5`, keeps the research question, required modules, primary claim,
formal arms, scenes, fallback and campaign authority unchanged, but replaces the review-heavy early gate sequence.
The immediate objective is a runnable `P0 -> P4-v2 -> EGO -> P5` vertical slice and then one development live
smoke. Effect-size diagnosis, targeted optimization, power inputs and confirmatory science follow only after the
flow works. ICRA-071 repair remains a non-blocking governance backlog item.

This acceleration does not authorize a campaign or a scientific/effect claim. GPU preflight, required-process
health, occupancy-before-risk, EGO motion-feasibility authority, P5 final/runtime gates, fail-closed evidence and
artifact retention remain mandatory.

## 1. Audit identity and immutable anchors

This audit compares the user-authorized P0 -> P4 -> P5 research route with the current pushed repository.
It does not relabel historical evidence, execute ROS/GPU work, or qualify any module.

| Identity | Commit / artifact |
|---|---|
| Original route approval | `73cbdddd0f44165f61138dcd74c61ab8dd96ebae` |
| Source-audit baseline named by that approval | `bd3858a72ba06b7eb1551006876c55362c979bab` |
| Authoritative P4-G0C result | `6e37b9ee37bf11661b2da70751c55685938540fe` |
| First active-route divergence | `564dd6ad8c864f496b63a1b09afd3febe31eef21` |
| Current audited pushed HEAD | `48caa9ddf24990accb65e2ad230d12821487793c` |
| Current branch synchronization | `HEAD...origin/dev/icra = 0 0` at audit start |
| Protected untracked PDF | SHA-256 `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6` |

The plan-to-current range contains 243 commits and touches 862 files, with approximately 185,078 insertions
and 1,065 deletions. Those line counts include large retained evidence products and are not treated as a
scientific progress metric.

## 2. Machine-readable user route lock

The JSON object between the sentinels is the canonical research-route lock. It is intentionally embedded in
this Markdown so the human-readable audit and the future repository-local verifier have one route authority.
Agents may validate and quote this object. Only an explicit new user research decision may authorize changing
its protected research fields.

<!-- ICRA_USER_ROUTE_LOCK_V1_BEGIN -->
```json
{
  "schema_version": "icra_user_route_lock_v1",
  "route_owner": "USER",
  "active_route": "P0_P4_V2_P5",
  "required_modules": [
    "P0_ADVISORY_RISK_FIELD",
    "P4_V2_COLLISION_GUIDE",
    "EGO_MOTION_FEASIBILITY_AUTHORITY",
    "P5_FINAL_AND_RUNTIME_HARD_GATES"
  ],
  "research_question": "Can an immutable future-risk field guide collision-triggered local search toward a lower provider-only predicted-integrity-risk bottleneck while EGO retains motion-feasibility authority and P5 independently blocks unsafe final or executing trajectories?",
  "primary_claim": "P4-v2 lowers the controllable-interior maximum provider-only predicted integrity risk beyond a preregistered scientifically meaningful and repeatability-derived threshold.",
  "secondary_claims": [
    "controllable-interior provider-only mean risk is non-inferior and its improvement is reported",
    "whole-path maximum risk is non-inferior",
    "path length, latency, coverage and fallback remain within preregistered limits",
    "the selected guide retains identity through EGO refinement and P5 evaluation"
  ],
  "formal_arms": [
    "P0_P5_CONTROL",
    "P0_P4_V2_P5_TREATMENT"
  ],
  "qualification_scenes": [
    "PRIMARY",
    "EXACT_MIRROR",
    "FLAT_NULL"
  ],
  "gate_sequence": [
    "ICRA-072_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE",
    "ICRA-073_EFFECT_DIAGNOSTICS",
    "ICRA-074_TARGETED_OPTIMIZATION",
    "ICRA-075_EXPLORATORY_AND_POWER_INPUTS",
    "ICRA-076_PREREGISTRATION_FREEZE",
    "ICRA-077_HELD_OUT_CONFIRMATION",
    "ICRA-078_G0D_LINEAGE",
    "ICRA-079_PROSPECTIVE_P5_QUALIFICATION",
    "USER_CAMPAIGN_APPROVAL",
    "ICRA-080_60_RUN_CAMPAIGN"
  ],
  "fallback_policy": "PROPOSAL_ONLY_USER_ACTIVATION_REQUIRED",
  "scientific_no_go_transition": "BLOCKED_AWAITING_USER_RESEARCH_DECISION",
  "campaign_activation": "USER_APPROVAL_AFTER_ICRA079_REVIEW_PASS",
  "approval_anchor": "b24a330d79d6e85e8080cf2a359bb1a18765e5a5",
  "user_decision_id": "USER-ICRA-ROUTE-20260826-002",
  "protected_transition": {
    "from_anchor": "b24a330d79d6e85e8080cf2a359bb1a18765e5a5",
    "changes": [
      {
        "field": "route_owner",
        "old": "USER",
        "new": "USER"
      },
      {
        "field": "active_route",
        "old": "P0_P4_V2_P5",
        "new": "P0_P4_V2_P5"
      },
      {
        "field": "required_modules",
        "old": [
          "P0_ADVISORY_RISK_FIELD",
          "P4_V2_COLLISION_GUIDE",
          "EGO_MOTION_FEASIBILITY_AUTHORITY",
          "P5_FINAL_AND_RUNTIME_HARD_GATES"
        ],
        "new": [
          "P0_ADVISORY_RISK_FIELD",
          "P4_V2_COLLISION_GUIDE",
          "EGO_MOTION_FEASIBILITY_AUTHORITY",
          "P5_FINAL_AND_RUNTIME_HARD_GATES"
        ]
      },
      {
        "field": "research_question",
        "old": "Can an immutable future-risk field guide collision-triggered local search toward a lower provider-only predicted-integrity-risk bottleneck while EGO retains motion-feasibility authority and P5 independently blocks unsafe final or executing trajectories?",
        "new": "Can an immutable future-risk field guide collision-triggered local search toward a lower provider-only predicted-integrity-risk bottleneck while EGO retains motion-feasibility authority and P5 independently blocks unsafe final or executing trajectories?"
      },
      {
        "field": "primary_claim",
        "old": "P4-v2 lowers the controllable-interior maximum provider-only predicted integrity risk beyond a preregistered scientifically meaningful and repeatability-derived threshold.",
        "new": "P4-v2 lowers the controllable-interior maximum provider-only predicted integrity risk beyond a preregistered scientifically meaningful and repeatability-derived threshold."
      },
      {
        "field": "secondary_claims",
        "old": [
          "controllable-interior provider-only mean risk is non-inferior and its improvement is reported",
          "whole-path maximum risk is non-inferior",
          "path length, latency, coverage and fallback remain within preregistered limits",
          "the selected guide retains identity through EGO refinement and P5 evaluation"
        ],
        "new": [
          "controllable-interior provider-only mean risk is non-inferior and its improvement is reported",
          "whole-path maximum risk is non-inferior",
          "path length, latency, coverage and fallback remain within preregistered limits",
          "the selected guide retains identity through EGO refinement and P5 evaluation"
        ]
      },
      {
        "field": "formal_arms",
        "old": [
          "P0_P5_CONTROL",
          "P0_P4_V2_P5_TREATMENT"
        ],
        "new": [
          "P0_P5_CONTROL",
          "P0_P4_V2_P5_TREATMENT"
        ]
      },
      {
        "field": "qualification_scenes",
        "old": [
          "PRIMARY",
          "EXACT_MIRROR",
          "FLAT_NULL"
        ],
        "new": [
          "PRIMARY",
          "EXACT_MIRROR",
          "FLAT_NULL"
        ]
      },
      {
        "field": "gate_sequence",
        "old": [
          "ICRA-071_ROUTE_GUARD",
          "ICRA-072_RISK_DECOMPOSITION_AND_REPLAY",
          "ICRA-073_CONTROLLABILITY_FIXTURES",
          "ICRA-074_P4_V2_SEARCH",
          "ICRA-075_EXPLORATORY_AND_POWER_INPUTS",
          "ICRA-076_PREREGISTRATION_FREEZE",
          "ICRA-077_HELD_OUT_CONFIRMATION",
          "ICRA-078_G0D_LINEAGE",
          "ICRA-079_PROSPECTIVE_P5_QUALIFICATION",
          "USER_CAMPAIGN_APPROVAL",
          "ICRA-080_60_RUN_CAMPAIGN"
        ],
        "new": [
          "ICRA-072_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE",
          "ICRA-073_EFFECT_DIAGNOSTICS",
          "ICRA-074_TARGETED_OPTIMIZATION",
          "ICRA-075_EXPLORATORY_AND_POWER_INPUTS",
          "ICRA-076_PREREGISTRATION_FREEZE",
          "ICRA-077_HELD_OUT_CONFIRMATION",
          "ICRA-078_G0D_LINEAGE",
          "ICRA-079_PROSPECTIVE_P5_QUALIFICATION",
          "USER_CAMPAIGN_APPROVAL",
          "ICRA-080_60_RUN_CAMPAIGN"
        ]
      },
      {
        "field": "fallback_policy",
        "old": "PROPOSAL_ONLY_USER_ACTIVATION_REQUIRED",
        "new": "PROPOSAL_ONLY_USER_ACTIVATION_REQUIRED"
      },
      {
        "field": "scientific_no_go_transition",
        "old": "BLOCKED_AWAITING_USER_RESEARCH_DECISION",
        "new": "BLOCKED_AWAITING_USER_RESEARCH_DECISION"
      },
      {
        "field": "campaign_activation",
        "old": "USER_APPROVAL_AFTER_ICRA079_REVIEW_PASS",
        "new": "USER_APPROVAL_AFTER_ICRA079_REVIEW_PASS"
      }
    ]
  },
  "user_decision": {
    "route_disposition": "DEVELOPMENT_FIRST_END_TO_END_VERTICAL_SLICE_BEFORE_EFFECT_OPTIMIZATION",
    "p0_p5_disposition": "RETAIN_AS_MATCHED_CONTROL_ASSET",
    "p4_primary_endpoint": "MAX_PROVIDER_ONLY_INTERIOR_RISK",
    "confirmatory_size": "PREREGISTERED_ADAPTIVE_30_TO_60_INDEPENDENT_SEED_RUNS_PER_SCENE",
    "enforcement": "LEAN_MANDATORY_SAFETY_GATES_WITH_DEFERRED_SCIENCE_REVIEWS"
  },
  "guard_strength": "ACCIDENT_PREVENTION_NOT_A_SECURITY_BOUNDARY"
}
```
<!-- ICRA_USER_ROUTE_LOCK_V1_END -->

Protected research fields are `active_route`, `required_modules`, `research_question`, `primary_claim`,
`secondary_claims`, `formal_arms`, `qualification_scenes`, `gate_sequence`, `fallback_policy`,
`scientific_no_go_transition`, and `campaign_activation`. The route owner, decision identity, approval anchor,
protected transition and explicit user selections are also immutable without a new user decision. A Supervisor
may propose changes but cannot approve them.

## 3. Executive deviation verdict

There is no scientifically defensible single percentage for route deviation. The dimensions answer different
questions and must remain separate.

| Dimension | Original target | Current state at `48caa9d` | Deviation |
|---|---|---|---|
| Seven-gate goal completion | Scope, P0, G0A, G0B, G0C, G0D, campaign all pass | First four pass; G0C executed but NO_GO; G0D/campaign absent | `4/7 = 57%` goal completion |
| Process traversal | Reach and pass all seven gates | Reached gate 5, where the registered science gate failed | `5/7 = 71%` traversed, not achieved |
| Active system stages | P0 -> P4 -> P5 | P0 -> P5 contingency | one of three active stages removed: `33%` structural deviation |
| Core treatment/novelty | P4 is the causal treatment | P4 disabled in the active route | `100%` loss of the original treatment claim |
| Formal evidence | 2 arms x 3 scenes x 10 seeds = 60 runs | 0 formal runs | `100%` formal-evidence shortfall |
| P4 source investment | G0A/G0B/G0C plus G0D | G0A/G0B implemented; G0C valid NO_GO; G0D absent | substantial reusable code, incomplete treatment |
| Current P0+P5 qualification | prospective three-case PASS | static 593/593; replacement/parser/GPU/live/analyzer `0/0/0/0/0` | unqualified |

The correct summary is therefore: engineering reached a valid P4 measurement system, but the original
scientific treatment and its formal comparison are not delivered. The current P0+P5 work is retained as the
future matched control and cannot substitute for P4 novelty.

## 4. Original roadmap, divergence and current location

| Roadmap point | Intended result | Actual result | Disposition |
|---|---|---|---|
| 2026-08-20 / `73cbdddd` | User-authorized conditional P0 -> P4 -> P5 | route and seven gates frozen | retained authority anchor |
| P0 Gate-0B / ICRA-035 | qualify immutable advisory grid | PASS | reusable, not reopened |
| P4-G0A / ICRA-036..038 | collision scan contract | PASS | reusable |
| P4-G0B / ICRA-039..041 | same-event dual-guide metrics-only seam | PASS | reusable deep-module boundary |
| P4-G0C / ICRA-042..066 | show robust mean/max improvement and freeze thresholds | technically valid `SCIENTIFIC_NO_GO` | immutable v1 result; never relabel PASS |
| `564dd6a` | old plan permitted a separate contingency decision | Supervisor activated P0+P5 | first active-route divergence |
| ICRA-067..070 | qualify P0+P5 contingency | profile/harness and static repair work; no current live PASS | freeze; retain as control asset |
| P4-G0D | apply selected guide and prove B-spline/P5 lineage | not implemented | recover only after P4-v2 confirmation |
| Formal campaign | 60 matched runs | not started | remains blocked |

The route change was not made by the Builder or by an automatic runner. Commit `564dd6a` is the Supervisor
decision, anticipated by `29960831`. It followed the legitimate `6e37b9e` P4-G0C NO_GO. The then-current scope
explicitly allowed P0+P5 to become the main route after a new Supervisor decision, so the transition conformed
to the old written process. It had no distinct user authorization record. The root cause is therefore an
authority-model defect: the old process delegated research-route ownership to the Supervisor.

The later `d335665` instruction that reduced the target to 15 processes/GNSS-disabled operation was a genuine
scope contraction. It was withdrawn and corrected by `3c8fffe`; that contraction is not present at current
HEAD.

## 5. P4-G0C scientific forensics

### 5.1 Reproducible authoritative result

| Artifact | SHA-256 / result |
|---|---|
| `p4_g0c_analysis.json` | `572e5d79fc5148cb5a4c33d30296186fdeceaa4cc9454c05f2b0986f9cda9c1e` |
| protocol v6 | `8fffeef30777043342be96213be30385226b1244607acd0dcf356f01009e80eb` |
| live fixture v2 | `2ba39a328b8ba9deff0e82524cd5c8474484a0f4d0ff9abf8efb0da6e1cf86e4` |
| frozen raw bundle | `0d6adce40b2a4b45ea0bb0b37a44c89828557290ce979b3bb417b3fef6323635` |
| completeness | 15/15/15 runs; 192/192 complete decisions; zero technical failures |
| mean improvement | 174 positive, 17 zero, 1 negative; Q10 `0.000020000000000131024` |
| max improvement | 136 positive, 56 zero, 0 negative; Q10 `0` |
| unique path pairs | 57; 17 identical-path rows and 39 different-path max ties |

P4-v1 was not universally ineffective: 70.8% of decision rows strictly reduced maximum risk and no row
increased it. It nevertheless failed the preregistered robustness gate because more than 10% of rows tied.
Repeated rows are not independent evidence: two runs each repeat the same main zero-improvement path pair ten
times.

### 5.2 Root-cause ranking

1. **Metric/source contamination — confirmed.** `CONSERVATIVE_OCCUPIED_COST_SUPPORT` substitutes
   `unknown_cost=10` for occupied-skip interpolation corners and mixes that value into the scalar called
   `c_pi`. In retained readiness traces, occupied support contributes approximately 78.8%–86.9% of the
   observed path maxima. The result cannot be attributed solely to provider-predicted integrity risk.
2. **Fixture and guide-domain identifiability — confirmed.** Reconstructed provider-only values along the
   readiness main guides are approximately 1.135–1.156 and nearly flat. The fixture describes corridor centres
   at `|y|=2.0`, while observed main guides cover only original `y in [0,0.8]` and risk `y in [0,1.1]`.
   The P4 request did not expose the intended homotopy contrast.
3. **Objective/acceptance mismatch — confirmed.** Current A* minimizes
   `sum length * (1 + lambda * risk)`, an integral-like objective. It cannot guarantee reduction of a bottleneck
   maximum, which was the G0C acceptance metric.
4. **Fixed-endpoint estimand mismatch — confirmed.** Retained traces contain different-path decisions whose
   maximum occurs at the common sample 0 or 199. P4 cannot strictly improve a maximum fixed by shared
   endpoints.
5. **Pseudo-replication — confirmed.** The 192 row-level decisions substantially exceed the 57 unique path
   pairs and the 15 independent run containers. Row-level Q10 overweights repeated states.
6. **Time-dependent search state — structural risk, not proven as the r6 cause.** Query time depends on arrival
   distance, but the search stores only one label per voxel. Two arrivals with different time/risk futures are
   not equivalent and require time/horizon-aware labels.

There is no evidence that the top-level P0 -> P4 -> EGO -> P5 authority architecture caused the failure. The
same-event request identity, immutable snapshot, occupied-before-risk rule, EGO feasibility authority and P5
hard-gate boundary remain the correct system design.

## 6. P4-v2 scientific and module design

### 6.1 Preserve the external seam; replace the internal objective

Keep `planCollisionGuide(const P4GuideRequest&) -> P4GuideDecision`, same endpoints, snapshot, query base,
occupancy epoch, metrics-only isolation and fail-closed fallback. Do not reinterpret v1 evidence.

Add a versioned decomposition equivalent to:

```cpp
struct RiskCostDecomposition {
  bool valid;
  double provider_c_pi;
  double provider_support_weight;
  double occupied_support_weight;
  double unknown_support_weight;
  uint64_t generation_id;
  std::string reason;
};
```

Expose the decomposition through one versioned `queryRiskCostDecomposition()` query. The legacy scalar
`queryCost()` remains available for immutable v1 replay. P4-v2 consumes only `provider_c_pi` as its scientific
risk. Occupancy remains a hard rejection before risk; insufficient provider support is incomplete/fallback.
Clearance or occupied/unknown-support penalties, if retained, use separately named fields and never
masquerade as provider `c_pi` or enter the primary endpoint.

P4-v2 search state is `(voxel, horizon/time bin)` and may retain multiple nondominated labels at one spatial
voxel. It must not collapse different arrival times into the existing single spatial `gScore`. Its
lexicographic cost is:

```text
(maximum interior provider c_pi, integrated interior provider c_pi, path length)
```

Dominance and heuristic rules must preserve this ordering. The existing 0.2 s search limit and 1.30 path
ratio remain hard gates unless a future explicit user route decision changes them.

The collision bypass domain must expose and verify two reachable homotopies in both primary and exact-mirror
fixtures. A fixture is not identifiable when its intended contrast is centred at `|y|=2` while every produced
guide stays within approximately `|y|<=1.1`; such a fixture fails prequalification and cannot enter
confirmatory evidence.

### 6.2 Preregistered estimand

For risk-grid resolution `r`, freeze endpoint buffer `b=2r` before confirmatory data. A segment whose length is
not greater than `2b` is preregistered ineligible and cannot be deleted after seeing its effect.

```text
B_a      = max provider c_pi over arc interval [b, L_a-b]
D_peak   = B_original - B_risk                         # sole primary
A_a      = arc-length mean provider c_pi on [b, L_a-b]
D_mean   = A_original - A_risk                         # secondary
U95_repeatability = max(U95 flat-null, U95 byte-identical serialized replay)
delta_peak = max(domain SESOI, U95_repeatability)
```

Whole-path maximum is a secondary non-inferiority check, because shared endpoints are outside P4 choice
authority. Mean, length, latency, provider support, timeout and fallback remain secondary/mechanism endpoints.
Each `U95` is the preregistered 95% upper repeatability bound for `|D_peak|` from independent flat-null runs or
byte-identical serialized-snapshot replays; the larger bound is frozen before held-out access. The IEEE-754
`1e-12` bound is retained only as a numerical diagnostic; it is not the scientific SESOI.

### 6.3 Evidence sequence

1. Freeze r6 NO_GO and hashes.
2. Exploratory-only source decomposition, whole-field P0 slices, serialized snapshot replay, objective/query
   ablations and primary/mirror/null controllability checks. Historical and development seeds may be used
   here, but never for a new claim.
3. Estimate repeatability, zero mass, variance and intra-run correlation; compute per-scene sample size between
   30 and 60 independent seed-runs with at least 90% power before revealing held-out outcomes.
4. Freeze protocol, SESOI, seeds, order, hashes and exact sample size. Confirmatory seeds are disjoint from
   historical/exploratory/formal seeds.
5. Count one preregistered primary collision event per run. Extra events are descriptive or analyzed with the
   run/seed as the cluster; repeated snapshot/path rows never increase `n`.
6. Test the primary one-sided claim `Pr(D_peak > delta_peak) > 0.9` at `alpha=0.05` using the frozen exact
   binomial/tolerance rule. At `n=30`, this requires 30/30 successes; for another frozen `n`, compute the
   smallest passing count before the run.
7. Mirror must reverse homotopy while retaining the effect direction. Flat-null uses a preregistered
   equivalence/false-selection gate. Coverage is complete, timeouts/fallbacks are zero, and the length cap
   passes. No retry, exclusion or threshold adjustment is permitted.

Only confirmatory PASS authorizes G0D. Only G0D lineage and prospective P5 integration PASS make a campaign
eligible for a new user decision.

### 6.4 Frozen implementation and statistical acceptance coverage

The later P4-v2 Builder tasks must use TDD and cover, at minimum:

- provider/occupied/unknown risk decomposition and provider-support insufficiency;
- occupied-before-risk hard precedence and v1 `queryCost()` replay compatibility;
- endpoint-buffer inclusion/exclusion at exactly `b=2r` and short-segment ineligibility;
- bottleneck/minimax ordering, integral and path-length tie-breaks;
- time-state dominance with two arrivals at one voxel whose future risks differ;
- same-occupancy primary, exact-mirror and flat-null controllability;
- timeout, fallback, stale/unknown/non-finite input and occupancy-epoch invalidation;
- selected-guide identity through control points, B-spline refinement and P5 final/runtime evaluation.

Statistical tests must reject decision/path/snapshot rows as independent units. One preregistered collision
event contributes at most one primary result per independent run/seed; extra events require descriptive or
run/seed-clustered analysis. Tests must also prove that r6 remains NO_GO and that historical confirmatory seeds
cannot enter P4-v2 tuning or confirmation.

## 7. Standards and maintainability audit

### 7.1 Hard documented findings

- Mandatory requirement ID absent from four commits: `363be826`, `5f6b6494`, `2bd5ba4f`, `79add9cb`.
- Mandatory code/config documentation synchronization is incomplete in nine commits:
  `20d3c5d`, `544451f`, `f2119f6`, `32bd497`, `79add9c`, `e59d090`, `b629a8a`, `0346fd2`, `005ce1a`.
- The existing `core.hooksPath` is stale and absolute; the tracked pre-commit matcher expects paths prefixed by
  `src/iap/` even though this repository root is already `src/iap`. There is no effective tracked enforcement.

These are historical audit findings. They are not repaired by rewriting history and do not invalidate the
immutable r6 evidence. ICRA-071 prevents recurrence.

### 7.2 Judgment-call debt

- P4 protocol/runner/analyzer evolved through six coordinated versions with repeated literal hashes and
  schema cascades. Future versions need an immutable version-descriptor/adapter registry.
- `P0RiskGridRuntime` combines sensor capture, advisory construction, cache lifecycle, fixtures and evidence.
- The P0+P5 qualification runner combines install construction, Git trust, process orchestration and analysis.
- P4 repeats request/occupancy guards at several stages instead of centralizing the invariant.

These items enter a maintenance backlog and cannot be mixed into a bounded P4-v2 science task without a
separate authorization.

## 8. User-owned development state machine

Research route ownership and evidence verdict ownership are distinct:

```text
USER owns research question / required modules / claim / arms / route / fallback activation
  -> SUPERVISOR freezes tasks and judges evidence against the user-owned contract
    -> BUILDER implements exactly one active task
```

On a scientific NO_GO:

```text
SCIENTIFIC_NO_GO
  -> active_role=SUPERVISOR
  -> status=BLOCKED_AWAITING_USER_RESEARCH_DECISION
  -> next_task=NONE
  -> Supervisor may write proposals only
```

No contingency becomes active and no alternative `TASK_READY` may exist until a distinct user decision is
recorded against an exact pushed anchor. Supervisor and Builder commits cannot self-authorize protected field
changes.

ICRA-071 implemented a repository-local verifier plus pre-commit, pre-push and commit-message hooks, but Review
found lifecycle, exact-claim/RQ and full-discovery defects. Decision 002 retains that repair as non-blocking
governance backlog. Because the user selected a local guard, this is accident prevention, not a security
boundary: a process with repository write permission can still edit hooks or use `--no-verify`. Truly
non-bypassable enforcement requires a protected remote branch and an independent user approval identity.

## 9. Corrective roadmap and gates

The active execution grouping is `docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md`: ICRA-072A iterative
integration, ICRA-072B stabilization, ICRA-073..075 effect diagnosis/improvement and ICRA-076..079 formal
verification. This grouping leaves the machine-readable route lock and gate sequence above byte-for-byte
unchanged. Development one-shot and per-task build/install controls do not apply in Layers 1–3.

The frozen inverse-corridor diagnostic design is
`docs/icra27/dev/ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE.md`
(`ICRA_P4_V2_INVERSE_CORRIDOR_FIXTURE_V1`). It is still implementation-deferred because ICRA-072A integration
and ICRA-072B stabilization have not closed ICRA-072. Its independent oracle is an evaluation-only bypass and
may never feed P0, P4, EGO or P5 decisions. A development-only selection trigger used to close ICRA-072 is a
distinct engineering fixture and cannot be relabelled as inverse-corridor effect evidence.

| Task | Authorized result | Stop line |
|---|---|---|
| Supervisor recovery changeset | publish this audit, restore `P0_P4_V2_P5`, supersede unqualified ICRA-070 | no product/runtime change |
| ICRA-071 backlog | repair user-route/state/doc/RQ local guards | non-blocking after user decision 002; no security claim |
| ICRA-072A | iterative P0 -> P4-v2 -> EGO -> P5 vertical slice against shared build/install | one complete live identity; no effect claim |
| ICRA-072B | stabilize the successful chain as production-shaped happy-path and fail-closed regression | closes ICRA-072; may issue only ICRA-073 |
| ICRA-073 | implement PRIMARY/MIRROR/NULL inverse corridors, paired control/treatment and independent-oracle effect diagnostics after flow closure | measure retained evidence only; no tuning, held-out access or claim |
| ICRA-074 | targeted optimization derived from retained ICRA-073 evidence | no tune-during-ICRA-073, held-out access or threshold tuning |
| ICRA-075 | exploratory objective/source/domain ablation and power inputs | no held-out access or claim |
| ICRA-076 | freeze protocol, SESOI, hashes, seeds and 30–60 sample size per scene | no confirmatory run before Review PASS |
| ICRA-077 | primary/mirror/null held-out confirmatory | no retry/exclusion; primary exact gate |
| ICRA-078 | formal G0D lineage qualification | held-out confirmation required |
| ICRA-079 | prospective P0+P4+P5 treatment and P0+P5 control P5 qualification | campaign remains blocked |
| ICRA-080 | original 2 arms x 3 scenes x 10 seeds campaign | separate user approval required |

ICRA-070 is `SUPERSEDED_UNQUALIFIED_BY_USER_ROUTE_DECISION`, not PASS or scientific FAIL. Its code and evidence
remain useful as the future P0+P5 control. The old P0+P5-specific ICRA-071 plan is superseded before activation;
the task identifier is retained for the higher-priority user-route guard.

## 10. Artifact lifecycle and claim limits

- Preserve the protected PDF and all raw/compact/registered-live/scientific evidence and ordinary logs.
- User workflow decision `USER-ICRA-WORKFLOW-20260826-001` authorizes permanent retirement of only the exact
  regenerable build/install roots inventoried in
  `docs/icra27/dev/ICRA_REGENERABLE_BUILD_RETIREMENT_20260826.md`. This does not relabel or delete evidence.
- Layers 1–3 reuse `/home/dev/ws_iap/{build,install,log}` and may not create per-task or per-run build/install.
- A later task may delete only its reproducible build/install after its own Review PASS, pushed code/docs and
  verified `0 0` divergence. Raw, compact, manifest, log and scientific evidence remain retained.
- P4-v1 remains `SCIENTIFIC_NO_GO`; P4-v2 is a new prospective method, not a reanalysis that makes v1 pass.
- Even a successful P4-v2 may claim lower provider-only predicted risk in the frozen controllable domain and
  preserved downstream lineage. It may not claim certification, universal risk reduction, or replacement of
  EGO/P5 authority.
