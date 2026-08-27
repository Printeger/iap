# ICRA-072 — Layer 1 exact runtime/source admission closure (ICRA-072A continuation)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072A_LAYER1_ITERATIVE_INTEGRATION`
> Immediate Review base: `8ee1d7d443c8226dae383eec951293192abd79e7`
> Reviewed Builder HEAD: `b607b976d283a077855c590b9374da94880fb29e`
> User-mandated lineage base: `3b5199e0cf8efc904f124cdb73156a3209eb6d80`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: close the remaining runtime-identity, source-admission, TDD and repository-local evidence blockers

## Accepted starting evidence

Preserve `run-001` through `run-023` byte-for-byte. `run-023` is strong development module-integration evidence:
its initial/pre-ROS/final source records bind pushed commit `c59de16e864701f6da5291f51aebfbc48388d92a`;
runner and analyzer pass; all seven P0 -> P4 -> EGO -> P5-final -> publish -> P5-runtime stages are present; 15/15
required processes stay healthy; cleanup passes; selected ID `6`, start `1657065614522279439` and final identity
`4388eac04c2cc922` agree; and every actual selected-identity runtime sample is authoritative fused and safe.

That run does not close Layer 1 because the general acceptance boundary is still incomplete:

1. A runtime row is selected when **any** `runtime_committed` sample has the expected ID/start. The same row may
   contain missing, malformed, sentinel or different committed identities and still pass.
2. Source admission runs `git status --untracked-files=no`, so arbitrary untracked source is invisible. The one
   protected untracked PDF needs an exact path/hash allowlist; all other untracked paths must reject admission.
3. The implemented post-cleanup `source_binding_changed_during_run` rejection has no focused RED/GREEN test proving
   typed outcome and owned-process cleanup.
4. Builder disclosed temporary external verification output followed by deletion. The breach remains historical;
   replacement verification output must be repository-local and retained, never hidden or deleted.

## Required TDD and implementation

1. For each runtime record counted for the accepted terminal trajectory, collect every `runtime_committed` sample
   and require **all** of them to carry the same explicit positive trajectory ID and integer-nanosecond start.
   A mixed record must fail closed even when one sample matches. Add RED/GREEN cases for matching plus missing,
   malformed, sentinel, mismatched-ID and mismatched-start samples. Preserve effective/raw `OK`, fused authority,
   empty reason and no-rejection requirements across every accepted record.
2. Replace blanket untracked suppression with full status inspection. Permit only
   `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` when its exact SHA-256 remains
   `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`; reject every other untracked path,
   tracked change, staged change, rename or deletion. Record the allowlisted path/hash and rejected paths in the
   source-binding evidence. Add focused tests for the protected PDF, arbitrary untracked Python/config, dirty
   tracked state and source changes between initial, pre-ROS and final checks.
3. Exercise `source_binding_changed_during_run` without ROS/GPU by mocking the complete runner lifecycle. Prove
   the attempt produces manifest, analyzer result and typed orchestration outcome, returns nonzero, and clears only
   its owned process groups. Keep GPU-before-ROS and no-retry semantics.
4. Re-run the focused C++ verification whose old console output was deleted, but retain the complete replacement
   output under a new repository-local task result/log root. Do not recreate, modify or erase the historical
   `/tmp` path. Record the exact command, exit code and retained path in Builder-owned logs.
5. The duplicated lineage tuple construction is a non-blocking maintainability smell. A small canonical identity
   helper is allowed only if it directly reduces drift in the repaired checks; no general refactor is requested.

## Build, verification and fresh evidence

Use only the shared build command documented in README and exactly the six packages `iap`, `plan_env`, `traj_utils`,
`path_searching`, `bspline_opt`, `ego_planner` under `/home/dev/ws_iap/{build,install,log}`. Run focused analyzer/
runner tests, the hermetic launch wrapper and the repository-local retained C++ verification. Do not create task-local
build/install trees or external output files.

Commit and push the repaired implementation and Builder-owned documentation first; fetch and prove divergence
`0 0`. Then run the next fresh identity, `run-024` or later, using the canonical README command. Stop at the first
runner/analyzer PASS. The accepted chain remains:

```text
P0 valid snapshot
  -> truthful closed collision
  -> P4-v2 risk guide naturally selected and applied
  -> EGO final B-spline
  -> authoritative-fused P5 final PASS before normal publish
  -> normal trajectory publication
  -> authoritative-fused P5 runtime OK on one exact committed trajectory identity
```

Attempt, segment/request, snapshot generation/configuration, occupancy epoch, selected-guide hashes,
control-point/final-B-spline identity, positive trajectory ID and lossless start time must agree end to end. Any
mixed/missing identity, source mismatch, required-process failure, cleanup failure, P5 ordering failure or missing
stage fails closed.

## Allowed scope

- `scripts/dev_planner/analyze_icra072_vertical_slice.py`,
  `scripts/dev_planner/run_icra072_vertical_slice.py` and focused tests.
- The smallest source-binding schema/report update required by the exact allowlist.
- Repository-local retained focused-test output under `results/icra27/icra072/`.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`; README only if the
  canonical command genuinely changes.

## Forbidden and retention

- Do not overwrite, edit, relabel, reanalyze in place or delete `run-001` through `run-023`, any other raw/compact/
  live evidence, ordinary log, shared build/install/log root or the protected PDF.
- Do not create/chmod/delete output, backup, archive, evidence or temporary verification files outside this
  repository. No disk cleanup is authorized.
- Do not edit Supervisor files, route lock, Gate sequence, scope, roadmap, implementation plan, plan review,
  workflow authority, guard plan, system-flow Markdown or protected PDF.
- No P5 source/threshold/AL/VAL/fusion change; no LiDAR-only, GNSS-only, fallback-only or advisory-P0 replacement
  for authoritative fused P5. Do not weaken occupancy, EGO, GPU, process or cleanup authority.
- No inverse-corridor implementation, effect diagnosis, optimization, SESOI/power/held-out/formal work,
  prospective qualification or campaign.

## Handoff

Stage only authorized files, inspect staged diff and untracked paths, use mapped requirement IDs, commit and push
normally. Return `ICRA072A_LAYER1_FLOW_READY_FOR_REVIEW` with the exact pushed HEAD, shared six-package build,
focused test counts and retained paths, fresh run identity, source-binding allowlist/rejections, first-missing-stage
result, accepted terminal identity, fused final/runtime safety, process health and cleanup evidence.

Only a later Supervisor PASS may issue ICRA-072B stabilization. ICRA-072 closes only after ICRA-072B Review PASS.
