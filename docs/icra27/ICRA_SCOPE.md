# ICRA 2027 Conference Scope — IAP P0 + P5

> Activated 2026-08-18 by the Gate 0 Supervisor review. Gate 0A returned the narrow verdict `NO_GO_P2`; P2 is frozen. Gate 0B remains blocked by an upstream required-process failure and has not produced a P0 latency result.

## Research question

Can a future protection-level advisory field be generated reliably from live GNSS/LiDAR/odometry/current-integrity inputs and used by an independent IAP hard integrity gate to fail safely before and during trajectory execution, while leaving original EGO collision and dynamics feasibility authoritative?

## Core claim boundary

P0 provides only a future-PL advisory field. It may read the authoritative current-state monitor within the system as a one-way prior, but it cannot write back, override current PL/AL/IM, declare safety, or directly accept/reject a trajectory.

P5 final and runtime gates together are the only hard integrity-gate authority in the IAP layer. Original EGO collision and dynamics checks remain authoritative for motion feasibility. A trajectory must satisfy both authorities; P5 does not replace the EGO checks, and EGO feasibility does not imply integrity safety.

The claimed separation is logical and one-way. This scope does not claim physical isolation, certification-level proof, certified active perception, formal PHMI guarantees, or real-world generalization without evidence.

## Included scope

- Current GNSS/LiDAR integrity interface and authoritative current PL/AL/IM monitor.
- Predictor advisory queries and P0 future predicted-PL field/generations.
- Explicit, manifest-bound qualification backend selection for reproducible CPU/GPU execution.
- Truthful source readiness, freshness, validity and required-process health evidence.
- P5 final admission before normal trajectory publication.
- P5 runtime monitoring of a committed trajectory.
- Fail-safe handling of unknown, stale, missing-source, invalid and non-finite evidence.
- Original EGO candidate generation, refinement, collision and dynamics behavior unchanged.

## Frozen or excluded scope

- **P2 is frozen by Gate 0A `NO_GO_P2`.** Across nine fixed runs, all 378 optimizer-success attempts were singleton; there was no eligible same-attempt reranking set. No P2 scoring, winner selection, batch identity, candidate fixture or candidate-generation work is permitted on the active route.
- P1 soft integrity cost, P3 local/global reference bias, P4 risk-aware local A*, A-ALL and the full P1-P4 stack remain closed for the conference route. Their source may remain in the frozen baseline.
- Continuous-time localization/planning joint optimization, BLOM/MINCO or another trajectory representation, certification claims and PX4/real flight are excluded unless separately authorized by a future scope decision backed by evidence.

## Current gate interpretation

- **Gate 0A: `NO_GO_P2` (narrow).** The 378 singleton-candidate observations qualify the candidate-set question only. They are sufficient to freeze P2 but are not a complete-system PASS.
- **Gate 0B: `BLOCKED / P0_INPUT_AVAILABILITY_FAIL`.** `iap_rosnode` died with exit `-6` on a machine without a CUDA device; the top-level launch exit 0 did not expose that required-process failure. No real P0 generation or 76,800-query workload occurred, so P0 p50/p95/max are unmeasured.
- The active recovery task is `ICRA-002 / GATE_0B`: explicitly select the CPU mapping backend, restore valid integrity input and one P0 generation, then run the unchanged fixed benchmark once.

## Minimum experiment route

1. Qualification-only CPU smoke proving live `iap_rosnode`, at least one valid integrity report and at least one 76,800-query P0 generation.
2. One fixed Gate 0B run proving at least 20 distinct generations and p95 `<= 400 ms` without required-process failure.
3. Only after Gate 0B passes, independent P5 final/runtime safe, unsafe, stale and unknown qualification under the P0 + P5 route.

P0-only qualification is an evidence exercise, not a planner-winner claim. P5 decisions and EGO feasibility semantics must not be changed to make Gate 0B pass.
