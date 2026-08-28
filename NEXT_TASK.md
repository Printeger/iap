# ICRA-077 — debt-bearing held-out confirmatory execution

> Active gate: `ICRA-077_HELD_OUT_CONFIRMATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Milestone: `ICRA-077_LAYER4_DEBT_BEARING_HELD_OUT_CONFIRMATION`
> Review handoff / user approval anchor: `f654cdb1c8f4b6dc2ed2f3c086db4c64b2df7dbb`
> User decision: `USER-ICRA-ROUTE-20260828-008`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: execute and analyze the frozen 360-row PRIMARY/MIRROR/NULL held-out confirmation once; do not start ICRA-078

## Starting authority and disclosed debt

The user explicitly accepts and bypasses two ICRA-076 blockers: replay-003 used `r=0.5` rather than frozen
`r=0.75/b=1.5 m`, and freeze-005 omitted tracked `thirdparty/json/include` probe build inputs. ICRA-076 remains
BLOCKED/user-bypassed/NOT PASS; do not relabel freeze-005 as Review PASS. This decision nevertheless authorizes
ICRA-077 against the unchanged freeze-005 contract and its exact debt disclosure.

Freeze authority is the retained `results/icra27/icra076/preregistration-freeze-005.json` with SHA-256
`aee60ed05efb816254159ed51ab04fa4c5f2977ebf711e86384a53e91aeaf686`, replay-003, verification-002, frozen
`delta_peak=0.3 m`, exact 59/60 rule, three disjoint 60-seed scene ranges and 360-row paired order. No empirical
power claim exists.

## Mandatory pre-access barrier

Before opening, generating or analyzing any held-out outcome:

1. Fetch-confirm pushed source divergence `0 0`; require exact retained user artifacts and no tracked/staged or
   unexpected untracked source. Independently validate freeze-005 and its exact hash. Any source/install byte,
   protocol, seed/order, replay, verification or freeze drift is terminal.
2. Prove that already-frozen source/install bytes provide a complete command surface that can execute every
   frozen row, capture one preregistered primary collision event, retain terminal lineage and aggregate every
   frozen primary/secondary/null gate. Do not add or modify runner, analyzer, launch, config, test, product or
   build bytes after freeze. If the frozen execution surface is missing or cannot express the exact 360 rows,
   return `BLOCKED_ICRA077_FROZEN_EXECUTION_SURFACE_MISSING` before outcome access; do not improvise a script.
3. Validate repository-local, new, non-symlinked `results/icra27/icra077/confirmatory-001` output authority and
   sufficient writable capacity without deleting/compressing prior artifacts. Record pre-access source/freeze,
   disk, command and held-out-access state.
4. Run the AGENTS §8.5 GPU preflight: `nvidia-smi`, `cuInit(0)`, `device_count>=1`. On failure emit
   `GPU_NOT_READY`, retain evidence and stop before ROS. Verify all required processes and owned-process cleanup
   boundaries before the first formal row.

## One-shot frozen execution

- Execute rows exactly in freeze-005 `execution_order`, with the frozen scene, seed, arm and run ID. No preview,
  pilot, subset-as-result, reorder, retry, replacement, exclusion, threshold adjustment or outcome-conditioned
  stop/restart is allowed. A missing/failed row remains in its frozen denominator and evidence is retained.
- For each row bind pushed source, freeze identity, scene descriptor/provider truth, seed, arm, immutable P0
  snapshot, attempt/segment/occupancy epoch, original/risk/selected guide, committed EGO final, P5-final-before-
  publish, normal publication and P5 runtime identity. Required-process early death, GPU loss, writer/flush error,
  source change, stale/missing provider support or identity drift must fail closed with the first missing stage.
- Control is `P0_P5_CONTROL`; treatment is `P0_P4_V2_P5_TREATMENT`. Evaluation/oracle truth must never enter P0,
  P4, EGO or P5 decisions. Each independent scene/seed contributes at most one preregistered primary event;
  additional events are descriptive and do not increase `n`.
- Cleanup only processes proven launched by this task. Preserve all rows, partial outputs, ordinary logs and raw/
  compact/live/scientific evidence. Never overwrite `confirmatory-001` or any earlier artifact.

## Frozen analysis and result

Produce one repository-local, non-overwriting confirmatory manifest and analysis from the already-frozen analysis
surface. Bind every row and first failure. Compute `D_peak=B_original-B_risk` over frozen controllable interior and
success `D_peak>0.3 m` without changing the accepted replay-domain debt.

- PRIMARY: at least 59/60 successes.
- EXACT_MIRROR: at least 59/60 successes, treatment safe homotopy sign reversed and effect direction positive.
- FLAT_NULL: 60 complete pairs, `abs(D_peak)<=0.0 m`, provider-risk selection count `<=0`.
- Secondary: frozen `D_mean`/whole-path non-inferiority, path ratio `<=1.30`, search timeout `<=0.2 s`, complete
  finite provider support/coverage, zero timeout/fallback/collision/dynamics failure, and exact P5 final/
  publication/runtime identities.

If the frozen primary or mandatory mirror/null/secondary gate fails after outcome access, retain all data and
return the exact `SCIENTIFIC_NO_GO`/typed blocker to Supervisor. Do not tune, retry, activate fallback, issue a new
task or reinterpret the bypassed freeze debt.

## Forbidden scope and exit

- No source/config/install/build mutation, rebuild, threshold/SESOI/sample-size/seed/order change, new analysis
  method, ICRA-078, G0D qualification, ICRA-079, campaign or publication claim.
- Preserve `/home/dev/ws_iap/{build,install,log}`, all earlier evidence/logs, hidden user artifacts and untracked
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged except new authorized ICRA-077 evidence and ordinary logs.
- Return `ICRA077_HELD_OUT_CONFIRMATION_READY_FOR_REVIEW` only after the complete immutable result is pushed and
  fetch-confirmed `0 0`. Otherwise return the first typed blocker. Only Supervisor Review PASS may issue ICRA-078.
