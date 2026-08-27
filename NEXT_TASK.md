# ICRA-072 — Layer 2 canonical fail-closed repair (ICRA-072B)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072B_LAYER2_STABILIZATION`
> Review base: `9212bfef7c78b61aa841a0b6a33169804d9c448b`
> Reviewed Builder HEAD: `a63d3cc1098ce13baf28326dce5bf044ee7bd466`
> Immutable failed result: `results/icra27/icra072b/final_summary.json`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair the canonical harness's hermetic Git admission and skipped-test fail-closed behavior, then close the same Layer 2 Gate

## Accepted starting point

The shared six-package build is 6/6. The production terminal, P4 decision, P4 integration and P5 runtime suites
pass 8/8 + 2/2 + 2/2 + 4/4. Supervisor diagnostic execution under an isolated repository-local `HOME` plus
exact command-local trust also passes the existing tools suite 17/17. Do not change or broaden the product C++
implementation or those production-shaped tests in this repair.

The retained `final_summary.json` is an immutable truthful `FAIL`, SHA-256
`2669167ad2cfaa95100b0da01602a8d92c6ca7f1512e5d9383428595f11dc624`. Its tools suite exits 1 because the
isolated `HOME` cannot read the differently owned repository without explicit Git `safe.directory`; this fails
`final_trajectory_identity`, `p5_runtime_authority` and `operational_closure`. In addition, the runner currently
counts a skipped verbose Python unittest as observed and can let a required skipped row pass. These are the only
authorized repair targets.

Layers 1–3 are repeatable under the four-layer workflow. The failed `final_*` identity remains unchanged; after
repair, one fresh non-overwriting `repair-001` result is required. This is a same-Gate continuation, not a new
Layer, scientific run or qualification retry.

## Required RED/GREEN repair

1. Add focused RED/GREEN runner tests proving that the canonical suite environment works with isolated
   repository-local `HOME` only when it carries exact command-local trust for
   `/home/dev/ws_iap/src/iap`. Do not read ambient `/root/.gitconfig`, set a wildcard safe directory, disable
   ownership checking, or write system/global/local Git configuration files. The retained suite evidence must
   record the exact non-secret command-local Git trust environment or arguments.
2. Add focused RED/GREEN cases for skipped required tests. Any Python `unittest` skip and any C++/gtest skip or
   disabled required assertion must make its suite and dependent rows fail, even when test-count and assertion
   name parsing otherwise match. Record typed skip observations in the summary.
3. Preserve the existing missing/duplicate suite and row, absent assertion, nonzero exit, count-mismatch,
   exact pushed-source and protected-PDF checks. Prove absent or wrong repository trust still fails closed and
   no Git configuration file is created or modified by the runner.
4. Run the focused runner tests and the existing tools suite offline. Commit and push the complete repair before
   producing the fresh canonical result; fetch and prove divergence `0 0`.
5. Run the repaired canonical entrypoint once with exact non-overwriting outputs:
   `results/icra27/icra072b/repair-001_summary.json` and
   `results/icra27/icra072b/repair-001_logs/`. It must bind the pushed repair HEAD and report all five suites and
   all eight matrix rows PASS with no missing, duplicate, skipped, disabled or nonzero condition.

## Allowed scope

- `scripts/dev_planner/run_icra072b_stabilization.py`.
- `test/test_icra072b_stabilization.py` and only directly necessary runner/tool fixture coverage.
- `README.md`, `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`.
- New immutable `repair-001_summary.json` and `repair-001_logs/` under `results/icra27/icra072b/`.

Do not edit planner/P0/P4/EGO/P5 product C++, production C++ tests, Supervisor files, route authority, scope,
roadmap, implementation plan, plan review, workflow authority or system-flow authority. If a new product defect
appears, stop and return it for Supervisor review instead of expanding this task.

## Forbidden and retention

- No ROS launch, GPU preflight, live run, new `run-025`, bag capture, shared rebuild, algorithm change, effect
  diagnostic, inverse-corridor implementation, optimization, qualification or campaign work.
- Do not overwrite, edit, relabel, reanalyze in place or delete `final_summary.json`, `final_logs`, `run-001`
  through `run-024`, any raw/compact/live/scientific evidence, Supervisor diagnostic evidence, ordinary log,
  `/home/dev/ws_iap/{build,install,log}` or the protected PDF.
- Do not create/chmod/delete output, backup, archive, evidence or temporary verification files outside this
  repository. No disk cleanup is authorized.
- Do not weaken source binding, occupancy, EGO, fused-P5, GPU, required-process or cleanup fail-closed behavior.

## Exit and handoff

Layer 2 exits only when `repair-001_summary.json` binds the pushed repair HEAD and reports suite counts exactly
8 + 2 + 2 + 4 + 17 with every suite and all eight required rows PASS, zero skip/disabled observations and no
missing/duplicate/nonzero/count mismatch. The focused runner tests must pass, the already retained shared
six-package build remains green, tracked state is clean, only the exact protected PDF is untracked, and pushed
divergence is `0 0`.

Stage only authorized files, inspect staged diff and untracked paths, bind all mapped requirement IDs in commit
bodies, and push normally. Return `ICRA072B_CANONICAL_REPAIR_READY_FOR_REVIEW` with exact pushed HEAD, focused
test results, `repair-001` path/hash, per-suite/per-row results, command-local trust provenance and proof that no
Git configuration was mutated.

Only a later Supervisor ICRA-072B PASS may close ICRA-072 and issue ICRA-073. It cannot authorize qualification,
campaign or a scientific-effect claim.
