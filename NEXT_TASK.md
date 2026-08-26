# ICRA-071 — User-owned research-route repository guard

> Active gate: `USER_RESEARCH_ROUTE_AUTHORITY_GUARD`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor audit anchor: `48caa9ddf24990accb65e2ad230d12821487793c`
> Supervisor recovery changeset: `0db8faac27dda58ef31aa57ad7033f294e758ebc`
> User decision: `USER-ICRA-ROUTE-20260826-001`
> Requirement mapping: `IAP-RQ-000`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: route-lock parser -> state/plan verifier -> local hooks -> adversarial tests; no P4 product work

## User decision and Supervisor task issuance

The user restored `P0_P4_V2_P5` as the sole research route and retained P0+P5 only as the matched control.
The canonical machine-readable decision is the `ICRA_USER_ROUTE_LOCK_V1` block in
`docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md`.

P4-v1 G0C remains an immutable, technically valid `SCIENTIFIC_NO_GO`; it is not retried or relabelled.
ICRA-070 is `SUPERSEDED_UNQUALIFIED_BY_USER_ROUTE_DECISION`: its static implementation and evidence remain
retained, but replacement/parser/GPU/live/analyzer were not invoked and no P5 qualification claim exists.
Existing ICRA-068/070 build/install, raw evidence and the protected PDF remain untouched.

The old process incorrectly gave the Supervisor enough authority to activate a fallback without a distinct
user approval record. ICRA-071 closes that procedural gap before P4-v2 work. It is pure governance code and
tests. It does not implement risk decomposition, change P4 algorithms, run ROS/GPU/live qualification or
authorize a campaign.

## 1. Parse the canonical route lock once

- Add one strict parser for exactly one JSON object between the route-lock sentinels. Reject missing/duplicate/
  reordered sentinels, duplicate JSON keys, unknown/missing fields, invalid types, invalid decision/anchor,
  empty/duplicate modules/arms/scenes and any route owner other than `USER`.
- Resolve the approval anchor through repository history without network access. The current lock must bind
  exact anchor `48caa9ddf24990accb65e2ad230d12821487793c` and decision
  `USER-ICRA-ROUTE-20260826-001`.
- Expose an immutable typed value to the verifier and hook entrypoints. Do not mirror active route, required
  modules, claim, arms or fallback as Python/shell literals.

## 2. Verify route, state, task and protected transitions

- Add `scripts/dev_planner/verify_icra_research_route.py` as the single pure verifier described by
  `docs/icra27/ICRA_CROSS_LAYER_GUARD_PLAN.md`.
- Require current state/task/scope/plan to agree on `P0_P4_V2_P5`, ICRA-071 and the campaign barrier. P0+P5 may
  appear only as the control or immutable history.
- Reject removal of P4, max-risk claim drift, alternate arms, automatic fallback/campaign activation, stale or
  reused decision identity, and an unauthorized route-lock edit.
- Encode the NO-GO state machine: without a new canonical user decision, scientific NO_GO permits only
  `BLOCKED_AWAITING_USER_RESEARCH_DECISION`, `active_role=SUPERVISOR`, `next_task=NONE`.
- Emit stable typed reasons and nonzero exits. A PASS proves repository consistency, not cryptographic user
  identity.

## 3. Repair repository-local hooks

- Replace the tracked pre-commit implementation with one that uses repository-root paths and invokes the same
  Python verifier for route/scope/claim changes.
- Code/interface/config paths under `src/`, `include/`, `apps/`, `msg/`, `cmake/`, `launch/`, `config/`,
  `scripts/`, `test/`, `tests/`, `tools/`, `docker/`, `data/`, `thirdparty/` and `.githooks/`, plus root
  build/package/toolchain files, require staged `DEV_LOG.md`, `docs/CHANGES.md` and `docs/TRACEABILITY.md`.
  Add an extension/category fallback outside generated/evidence trees so a newly introduced source root cannot
  evade synchronization merely because its directory name is not yet enumerated.
- Add pre-push route/state verification and commit-msg validation requiring one or more `IAP-RQ-NNN` IDs.
- Remove `IAP_SKIP_DOCS`; add no bypass environment variable. Hooks never stage or rewrite files.
- Add an idempotent repository-local installer/checker that sets only
  `git config --local core.hooksPath .githooks`, rejects absolute/stale paths and never changes global config.
  This local-config mutation is explicitly authorized for ICRA-071.

## 4. Adversarial tests and acceptance

- Build one valid temporary repository fixture and derive all mutations from it. Cover every negative and
  positive case frozen in the guard plan, including Builder ownership, no-go transition, stale approval,
  every enumerated code/interface/config root, the new-root extension fallback, synchronized docs, commit
  messages and hook-path installation.
- Prove documentation-only historical text does not accidentally activate a route or campaign.
- Run focused parser/verifier/hook tests, then complete hermetic Python discovery. No ROS/GPU/build/install/live
  command is permitted.
- Produce compact static evidence binding route-lock/verifier/hook/test hashes, exact commands/exits and the
  verified relative hook path. Do not treat hook PASS as external security enforcement.

## Documentation and handoff

- Update Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and new compact ICRA-071 static
  evidence with applicable requirement IDs.
- Explicitly stage only authorized files, review the staged diff, commit with applicable `IAP-RQ-*` IDs and
  push normally. Never stage the PDF, existing build/install, raw evidence or Supervisor-owned documents.
- Return `ICRA071_ROUTE_GUARD_READY_FOR_REVIEW` or one typed blocker. Do not issue ICRA-072 yourself.

## Allowed files

- New route-lock parser/verifier and smallest hook installer/checker below `scripts/dev_planner/`.
- Focused tests below `test/`.
- `.githooks/pre-commit`, new `.githooks/pre-push` and `.githooks/commit-msg`.
- Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact ICRA-071 evidence.
- Repository-local `.git/config` only for exact `core.hooksPath=.githooks`; it is not staged.

## Forbidden

- No edit to `AGENT_STATE.md`, `NEXT_TASK.md`, `SUPERVISOR_LOG.md`, the route-lock/audit document, ICRA scope/
  plan/review/guard plan or user decision record.
- No P0/P4/P5 product, public runtime interface, launch/config/experiment, threshold, objective, fixture,
  scenario, evidence semantics or campaign change.
- No ROS/launch, GPU preflight, live identity, analyzer, build/install creation or cleanup.
- No retry/relabel of P4-v1 G0C or ICRA-070; no historical evidence, raw/bag/log/PDF/external-repository change.
- No global Git config, bypass variable, `--no-verify` use in acceptance evidence, force-push, reset, clean,
  stash or unrelated process termination.

ICRA-071 Review PASS authorizes only a separately issued ICRA-072 observability/decomposition task. It does not
authorize P4-v2 application, G0D, prospective P5 qualification or campaign.
