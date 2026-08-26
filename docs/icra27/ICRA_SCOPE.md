# ICRA 2027 Conference Scope — User-owned P0 -> P4-v2 -> P5 recovery

## User route restoration — 2026-08-26

The user has explicitly restored P4 as the indispensable conference treatment and frozen the active route as
`P0_P4_V2_P5`. The complete decision, deviation audit and machine-readable route lock are authoritative in
`docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md`, bound to pushed approval anchor
`48caa9ddf24990accb65e2ad230d12821487793c` and decision `USER-ICRA-ROUTE-20260826-001`.

The top-level authority architecture remains:

```text
P0 immutable provider-risk advisory
  -> P4-v2 collision-guide preference
    -> EGO optimization/refinement/feasibility authority
      -> P5 final -> normal publish -> P5 runtime
```

P4-v1 G0C remains an immutable, technically valid `SCIENTIFIC_NO_GO`; it is not retried, tuned or relabelled.
P4-v2 is a prospective method that separates provider-only integrity cost from occupied support, optimizes an
interior bottleneck objective and uses independent exploratory/preregistered/held-out evidence.

ICRA-070 is `SUPERSEDED_UNQUALIFIED_BY_USER_ROUTE_DECISION`. Its P0+P5 code/evidence remain retained as the
future matched control; replacement/parser/GPU/live/analyzer are still `0/0/0/0/0`, so it is neither
qualification PASS nor scientific FAIL. Existing build/install and all raw evidence remain retained.

The only next implementation task is ICRA-071 repository-local user-route guard hardening. It performs no P4
product change, ROS/GPU/live execution or campaign. P4-v2 implementation starts only after ICRA-071 Review
PASS. Campaign remains blocked through P4-v2 confirmatory, G0D lineage, prospective P5 integration and a new
explicit user campaign decision.

The former P0+P5 contingency section below is preserved as historical evidence. Where it conflicts with this
restoration or the route lock, it has no active-task authority.

## Superseded contingency activation — 2026-08-25

ICRA-066 authoritatively closes P4-G0C as `SCIENTIFIC_NO_GO`: all 15 runs and 192 decisions are technically
valid, but the registered Type-7 Q10 maximum-risk improvement is `0`, not above the `1e-12` floor. P4-G0D and
the P0+P4+P5 treatment are therefore closed for the conference route; P4 source and evidence remain retained.

Under the then-current governance, the explicitly preregistered `P0+P5` contingency was activated by a
Supervisor decision. P0 remained advisory, original EGO planning retained motion-feasibility authority, and
P5 final/runtime remained the only IAP hard gates. P1/P2/P3/P4 stayed present but disabled. That historical
work order was an isolated fail-closed `icra_p0_p5` profile and prospective P5 system qualification; historical
P5 artifacts were not relabelled as qualification.

ICRA-067 passed profile and validation-only harness review. ICRA-068 then passed historical-fixture decoupling,
543/543 tests, isolated install closure and GPU preflight, but its runner emitted 19 malformed empty ROS launch
arguments. SAFE_NORMAL stopped before any required child started; the complete `-001` registration is retired.

ICRA-069 was authorized to repair empty-argument serialization, prove all three commands with
the real non-executing ROS parser, adopt the unchanged isolated product install with dual provenance, then run
fresh SAFE_NORMAL, FINAL_REJECT and RUNTIME_FAIL `-002` identities exactly once. No product/scope change is
authorized and P5 remains unqualified until the authoritative analyzer passes.

ICRA-069 closes the serialization and parser/GPU blockers, then reveals a qualification sensor-binding defect:
every fixed case resolves `use_gnss=false`, so launch correctly omits the GNSS simulator even though the target
system and canonical contract require it. SAFE_NORMAL stops fail-closed at 15/16 after its sole `-002` attempt.
The result is not a Builder, GPU or node-start failure, and the complete `-002` set is retired.

The first ICRA-070 command at `d335665` incorrectly proposed changing the system contract to 15 processes. It is
withdrawn. ICRA-070 was then the only authorized live gate and corrected all three cases to one dedicated
full-sensor scenario: existing corridor geometry plus existing degraded GNSS, GNSS/ARAIM and LiDAR integrity,
IMU/LiDAR estimation and `max_pl` fusion. The canonical 16 processes remain mandatory. It adds source/topic
evidence, installs a no-compile isolated overlay with complete provenance, and executes fresh `-003` cases once.
P5 fixtures, route geometry, algorithms, thresholds and scientific acceptance remain unchanged.

Supervisor review at `d88d42b` accepts the static full-sensor binding and 567/567 hermetic tests but does not
pass the gate. The no-compile install recursively copied ignored `launch/__pycache__` bytes; at least two `.pyc`
files differ from ICRA-068, so overlay inventory correctly stopped before parser, GPU, live or analyzer. One
ICRA-070 continuation is authorized to exclude every generated Python cache, preserve the blocker evidence,
freeze non-overwriting v2 provenance and then use the still-unregistered `-003` identities exactly once.

Supervisor review of the resulting `1b3c661...24d3e16` changes accepts the permanent cache exclusion and
fail-closed static repair implementation but again does not pass the gate. The sole repair entrypoint stopped
before mutation because task-local Git did not trust the repository as a safe directory. Independently, the
complete file-set verifier proves the old overlay is structurally incomplete: 469 non-cache entries versus
2,079 in the ICRA-068 base, with 1,610 missing. The old repair is exhausted and the old overlay is retained.
One same-Gate ICRA-070 continuation was authorized to create a new complete non-overwriting overlay from the
retained ICRA-068 non-cache file set, apply only the three current aliases, and then run the unused `-003`
parser/GPU/live/analyzer sequence. The user route decision later superseded that unfinished qualification
without changing or relabelling its evidence.

The superseded plan would not have started a campaign even after an ICRA-070 qualification PASS: it first
required a pure-static cross-layer guard. The active ICRA-071 now implements the stronger user-route guard in
`docs/icra27/ICRA_CROSS_LAYER_GUARD_PLAN.md`; its PASS authorizes only a separately issued ICRA-072 task, not a
campaign.

The conditional P0 -> P4 -> P5 scope below is retained as the audited P4-v1 design record. Its authority
boundaries remain applicable, but its v1 objective, G0C estimand and Supervisor-owned fallback rule are
superseded by the user-owned P4-v2 route lock and recovery roadmap above.

> Scope pivot authorized 2026-08-20. Source audit is bound to `dev/icra` commit `bd3858a72ba06b7eb1551006876c55362c979bab`.

> State at the 2026-08-20 route approval: **P0 `BLOCKED/UNQUALIFIED` → P4 `NOT_QUALIFIED` → P5 `IMPLEMENTED-BUT-UNQUALIFIED`**.

> Historical Gate 0A remains `NO_GO_P2`. The scope pivot does not convert that result into a P4 result and does not qualify any stage of the new route.

## Research question

Can an immutable future-risk field guide collision-triggered local A* around lower predicted-integrity risk, while EGO retains motion-feasibility authority and P5 independently blocks unsafe final or executing trajectories?

## Conference route

The route is conditional, not an unconditional serial call of all three modules.

```text
P0 immutable advisory snapshot
  → collision scan
    → NO_COLLISION: original EGO planning; a later rebound collision re-enters the same scan/P4 seam
    → CLOSED_SEGMENTS: P4 guide decision → EGO optimization/refinement → P5 final → publish → P5 runtime
    → OPEN_ENDED_COLLISION or INVALID_INPUT: no new normal publish; existing FSM/P5 safety path
```

If original EGO optimization completes without a later closed rebound collision, the NO_COLLISION branch continues to P5 final, normal publish and P5 runtime.

P0 supplies advisory `c_pi` and predicted PL evidence. P0 cannot write back to the current integrity monitor, declare safety, or accept a trajectory.

P4 may prefer a collision-free guide with lower advisory risk. It cannot make occupied space traversable, override dynamics, issue a safety PASS, or replace P5.

EGO occupancy, rebound optimization, refinement, collision checks and dynamics checks remain authoritative for motion feasibility.

P5 final and runtime are the only IAP-layer hard integrity gates. P5 final runs before normal publication; P5 runtime monitors the committed trajectory.

## Collision contract

The initial seed is generated from a polynomial or the retained B-spline plus a polynomial tail. A* is called only after collision scanning; it does not generate the initial route.

A collision segment remains `free point → occupied interval → free point`. Its A* endpoints must both be free and shared by baseline and risk-aware searches.

The first two thirds of the seed are only the entry-trigger window. After an occupied entry is found there, scanning must continue to the seed tail until a stable free exit is found.

The planned scan result is:

```cpp
enum class CollisionScanStatus {
  NO_COLLISION,
  CLOSED_SEGMENTS,
  OPEN_ENDED_COLLISION,
  INVALID_INPUT
};
```

`OPEN_ENDED_COLLISION` must not be reported as `NO_COLLISION`. It must not synthesize an occupied endpoint or allow a new normal trajectory to be published.

## Included development

- Qualify live P0 generation, query shape, freshness, source provenance and latency before starting P4 production work.
- Add a deterministic collision-and-risk fixture before changing production collision or guide logic.
- Return explicit collision-scan status and preserve the complete closed-segment definition.
- Introduce one deep P4 module seam: `planCollisionGuide(request) -> decision`.
- Generate original and risk-aware A* guides for the same event, endpoints, occupancy epoch, snapshot and query-time model.
- Resample each final guide at 200 equal-arc-length points and compare mean/max risk, validity and path-length ratio.
- Preserve the selected-guide identity through control-point constraints, rebound optimization, refinement and final B-spline evidence.
- Qualify P5 final before normal publication and P5 runtime during execution.
- Add the planned composite profile `icra_p0_p4_p5` with fail-closed effective-value validation.
- Retain reproducible manifests, hashes, latency, fallback and generation lineage for each decision.

## P0 rolling-window design freeze

The authoritative P0 refactor design is
`docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`. It freezes a fixed world-aligned
risk lattice, a UAV-centred dense rolling window, version/TTL-bound spatial reuse and
atomic immutable generation publication.

The refactor does not change the ICRA ROI, `0.75 m` resolution, six horizons,
`0.5 s` refresh period or formal `400 ms` threshold. Every successful generation still
contains 76,800 logical risk voxels. Actual spatial recomputation, provider dispatch,
GNSS/LiDAR invocations, horizon fusion, window shift and full-rebuild reason must be
reported separately.

Development is ordered: correct production map LOS and horizon covariance semantics;
deduplicate spatial work within one refresh; add the fixed lattice/ring window; add
version/TTL/delta invalidation; then profile CPU workers and qualify. GPU/CUDA and an
iKD-tree risk-grid replacement are outside this sequence unless a later reviewed CPU
profile creates a new explicit task.

No P4 production task may start until the completed P0 sequence passes Gate-0B.

## Retained but disabled modules

P1, P2 and P3 remain in the repository. Their source, tests, CMake targets and legacy profiles must not be deleted.

The ICRA treatment profile must disable both their high-level switches and lower-level objective, metrics, debug, fanout and visualization paths. `planner_enable_all_safety` must remain false.

`manager/use_distinctive_trajs` is false in every P4 qualification, calibration and formal-comparison arm. This prevents legacy topology fanout or later candidate selection from obscuring P4 guide lineage.

The P0-only ICRA-004 prerequisite is not a P4 experiment arm and retains its frozen smoke configuration unchanged.

P2 remains historically frozen by `NO_GO_P2`. Re-enabling it requires a separate user research-route decision
and is not a fallback inside this route.

## Excluded scope

- P1 soft integrity objective, P2 candidate ranking and P3 local/global reference bias in ICRA runs.
- A-ALL or the full P1–P4 stack as the conference treatment.
- Candidate-generation changes introduced only to revive P2.
- Continuous-time localization/planning joint optimization or a new trajectory representation such as BLOM/MINCO.
- PX4 or real-flight claims without a separately authorized evidence program.
- Certification, formal PHMI, physical isolation or general safety guarantees.

## Gate sequence

1. **Scope pivot:** docs, requirements, state and ICRA-004 agree on the conditional route.
2. **P0 Gate-0B:** GPU preflight, valid integrity, real P0 generations, 76,800 logical risk voxels per generation, separately reported recompute/reuse/invocation counts, at least 20 generations and p95 `≤400 ms`.
3. **P4-G0A:** deterministic closed segment plus no-collision, open-ended, multi-obstacle and free-endpoint tests.
4. **P4-G0B:** metrics-only same-event original/risk guides, immutable identity, final-path resampling and truthful fallback evidence.
5. **P4-G0C:** metrics-only calibration, frozen thresholds, zero search timeout and complete 200/200 path coverage.
6. **P4-G0D:** enable application only after threshold freeze; prove selected-guide lineage through B-spline and P5 final/runtime behavior.
7. **Campaign (P4-v1 historical stop):** v1 stopped at G0C. The active P4-v2 route instead follows
   ICRA-071..079 in the recovery roadmap and requires a distinct user decision before ICRA-080, plus fresh GPU
   and `>=40 GiB` storage preflight.

ICRA-004 remains the P0-only prerequisite. Its smoke keeps P1/P2/P3/P4/P5 disabled and cannot authorize P4 work or the fixed 60-second Gate-0B run.

## Experiment arms

The primary comparison is `P0+P5` versus `P0+P4+P5` over primary, exact-mirror and flat-null scenes with ten frozen seeds: 60 formal runs.

EGO baseline, P4 metrics-only, and P0+P4 with P5 off are qualification or mechanism diagnostics. They do not replace the primary comparison.

The historical rule allowed P0+P5 to become the main route after a Supervisor decision. It is superseded.
Under the active route lock, a P4 scientific NO-GO becomes `BLOCKED_AWAITING_USER_RESEARCH_DECISION`; only the
user can activate a fallback.

## Historical evidence interpretation

Gate 0A observed 378/378 singleton optimizer-success attempts and no eligible P2 comparison set. That narrow result remains `NO_GO_P2`.

The recorded early `collision_segment_count=0` does not prove that the seed was collision-free. In the audited fork scene, the two-thirds scan could see obstacle entry but stop before the exit.

Later optimizer rebound evidence is compatible with a real collision. Historical rows are preserved; the new route fixes the observation and control contract rather than relabeling old runs.

## Claim limits

A successful P4 result may support a claim that advisory risk changed a collision guide and that the effect survived to a P5-evaluated B-spline under the frozen fixtures.

It may not support certification, universal obstacle avoidance, guaranteed lower execution risk, or a claim that P0/P4 replaced current-integrity, occupancy, dynamics or P5 authority.
