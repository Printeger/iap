# ICRA-071 — User-route guard Review repair

> Active gate: `USER_RESEARCH_ROUTE_AUTHORITY_GUARD`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor Review base: `9b813b0a52f405d874ce324f99f618221b5b7b8c`
> Reviewed Builder HEAD: `96c5cd85e37892eb4f565ce1181d57e62b817e0a`
> User decision: `USER-ICRA-ROUTE-20260826-001`
> Requirement mapping: `IAP-RQ-000`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair the rejected ICRA-071 guard and obtain a zero-failure hermetic static discovery

## Review verdict and retained boundary

ICRA-071 is `REQUEST_CHANGES`; ICRA-072 is not authorized. The route lock and active route remain unchanged:
`P0_P4_V2_P5`, with P0+P5 only as the matched control. P4-v1 remains immutable `SCIENTIFIC_NO_GO`, P4-v2
remains blocked, and campaign activation remains user-owned and forbidden.

The Review independently reproduced the current verifier and hook-path PASS plus focused 33/33 PASS. It also
reproduced four blocking defects:

1. A legitimate Supervisor §8.6 Review changeset from a HEAD whose state records `active_role=DEEPSEEK` fails
   `BUILDER_SUPERVISOR_FILE_STAGED`; the current hook therefore makes its own required review closure impossible.
2. Replacing the active scope's provider-only interior bottleneck/max-risk statement with a mean-risk statement
   still returns repository-consistent PASS.
3. `IAP-RQ-999` passes commit-message validation even though no such requirement exists in `docs/REQS.md`.
4. The sole complete hermetic discovery exited 1 with 614/616: two ICRA-070 tests read retained live repository
   state instead of an isolated fixture. Retained build/install/evidence must not be changed to make them pass.

The current requirements and traceability text also contains contradictory current rows: implementation is
recorded as focused PASS while older active rows still say `NOT_IMPLEMENTED` and that the hook remains absolute.

## Required repair

### 1. Close the Supervisor transition without weakening Builder protection

- Add a narrowly validated, repository-local Supervisor Review transition that lets §8.6 stage/commit the exact
  Supervisor-owned review set after a Builder handoff even when HEAD still records `DEEPSEEK`.
- The transition must bind the reviewed HEAD, unchanged route decision/anchor, one review verdict, same-task
  repair or next-task metadata, and no product/route-lock/evidence mutation. Generic or partial staging of
  Supervisor-owned files must still fail.
- State explicitly in code/tests/docs that this is procedural validation, not actor authentication; do not add
  a bypass environment variable, hidden token, global Git config or security claim.
- Add a positive adversarial fixture for a complete Supervisor Review transition and negatives for partial,
  stale-head, route-changing and product-mixed variants. Future §8.6 must not require `--no-verify`.

### 2. Verify exact active contract and real requirement IDs

- Make active scope/plan verification reject primary max-risk claim drift and required-module drift with stable
  typed reasons derived from the canonical route lock; do not mirror route/module/claim/arm literals in Python
  or shell.
- If machine-readable active-document bindings are needed, this task explicitly permits the smallest synchronized
  binding-only edits to `ICRA_SCOPE.md`, `ICRA_IMPLEMENTATION_PLAN.md` and `ICRA_PLAN_REVIEW.md`. They may only
  restate/point to the unchanged lock and must not alter route, claim, modules, arms, gates or authority.
- Parse the repository requirement inventory and reject every commit-message ID absent from `docs/REQS.md`;
  retain support for one or more existing `IAP-RQ-NNN` IDs.
- Convert the existing claim-drift and root-coverage helpers into end-to-end verifier/hook assertions for exact
  typed failures, rather than testing only a helper predicate.

### 3. Restore hermetic full discovery without touching retained state

- Repair only the two failing ICRA-070 tests so they derive replacement roots and compact inventories from a
  temporary repository fixture. Do not edit ICRA-070 runner/product behavior, constants, retained install trees,
  compact/raw evidence or expected historical verdicts.
- Prove the tests fail/pass on fixture state, not on whether the real retained `install_v2` or later compact
  artifacts exist.
- Run focused guard tests, focused repaired ICRA-070 tests, then exactly one fresh complete hermetic Python
  discovery under a new ICRA-071 repair evidence root. It must exit 0 with zero failures/errors and retain an
  empty external delta. Do not reuse or overwrite the prior 614/616 record.

### 4. Synchronize documentation truthfully

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `docs/REQS.md` so one current disposition is
  visible: implementation exists, Review requested changes, and acceptance remains pending until the repaired
  zero-failure discovery and Supervisor Review.
- Add compact non-overwriting repair evidence binding exact HEAD, hashes, commands/exits, hook path, transition/
  claim/RQ adversaries, full discovery and the unchanged protected PDF hash.

## Allowed files

- `scripts/dev_planner/verify_icra_research_route.py`.
- `.githooks/pre-commit`, `.githooks/pre-push`, `.githooks/commit-msg` only if required by the repair.
- `test/test_verify_icra_research_route.py` and the smallest fixture-only repair in
  `test/test_run_icra_p0_p5_qualification.py`.
- Binding-only active-document edits described above.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md` and new compact
  `results/icra27/icra071/` repair evidence.
- Repository-local `.git/config` only to retain exact `core.hooksPath=.githooks`; no global config mutation.

## Forbidden

- No route-lock sentinel/user decision, `AGENT_STATE.md`, `NEXT_TASK.md` or `SUPERVISOR_LOG.md` edit.
- No P0/P4/P5 product, runtime interface, launch/config/scenario, algorithm, threshold, objective or campaign work.
- No ROS/launch, GPU preflight, live identity, analyzer, build/install creation, retry or cleanup.
- No mutation/deletion/relabel of ICRA-068/070 build/install, compact/raw evidence, logs or the protected PDF.
- No weakening/removal of exact P4-v1 NO-GO, P4-v2 block, user route/fallback/campaign ownership or local-hook
  limitation; no bypass variable, `--no-verify` acceptance run, force-push, reset, clean or stash.

## Handoff and acceptance

Explicitly stage only allowed files, review the staged diff, commit with existing applicable requirement IDs
and push normally. Return `ICRA071_ROUTE_GUARD_REPAIR_READY_FOR_REVIEW` only after all focused suites and the
single fresh complete hermetic discovery exit 0. Any failure returns one typed blocker without retry.

Supervisor Review PASS may issue only ICRA-072. It still cannot authorize P4-v2 application, ROS/GPU/live work,
G0D, prospective P5 qualification, cleanup or campaign.
