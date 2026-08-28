# ICRA-077 — ICRA-077A pre-access governance-freeze closure

> Active gate: `ICRA-077_HELD_OUT_CONFIRMATION`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Milestone: `ICRA-077A_LAYER4_PRE_ACCESS_FREEZE_CLOSURE`
> Review handoff / user approval anchor: `fd4c1ed0bd9c7eccd40eaf4bc06d500074d8c429`
> User decision: `USER-ICRA-ROUTE-20260828-009`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: repair the self-invalidating pre-access governance boundary and produce a fresh offline freeze; do not access held-out outcomes or execute ICRA-077B

## Starting authority and retained evidence

`results/icra27/icra077/pre-access-blocker-001.json` is immutable evidence that freeze-005 correctly stopped
before held-out access, GPU, ROS, live execution, 360 rows or analysis. Preserve it and all prior replay,
verification and freeze attempts. Freeze-005 is no longer usable as current execution authority because three
legitimate post-freeze Supervisor updates produce governance-only byte drift:

- `docs/icra27/ICRA_IMPLEMENTATION_PLAN.md`;
- `docs/icra27/ICRA_FOUR_LAYER_DEVELOPMENT_WORKFLOW.md`;
- `docs/icra27/ICRA_P0_P4_P5_DEVIATION_AUDIT_AND_RECOVERY_ROADMAP.md`.

ICRA-076 remains BLOCKED/user-bypassed/NOT PASS. Its accepted `r=0.5` versus frozen `r=0.75/b=1.5 m` replay-domain
debt and omitted `thirdparty/json/include` build-input debt remain disclosed; this task must not repair, hide or
relabel either debt. The protected route remains `P0_P4_V2_P5`, P4-v1 remains `SCIENTIFIC_NO_GO`, P4-v2 and
ICRA-071 remain unchanged, and campaign activation remains user-owned.

## Required bounded design

1. Define exactly the three paths above as mutable governance authorities. No wildcard, directory-wide rule,
   caller-provided exemption or fourth path is allowed.
2. Represent each as an exact snapshot from the frozen pushed source commit, containing its repository-relative
   path, frozen source commit, regular Git blob OID/type, byte size and SHA-256. Resolve content from Git object
   storage (`git cat-file`/equivalent), not from the current working-tree path. Require the commit to exist, be
   an ancestor of current pushed HEAD and belong to the fetch-confirmed pushed lineage. Reject missing objects,
   path/type/OID/size/hash mismatch, symlinks and working-tree aliases. Legitimate later prose updates to exactly
   these three paths may differ without changing the frozen blob identity.
3. Independently freeze and validate a protected route-lock fingerprint covering route owner, active route,
   required modules, research question, primary/secondary claims, formal arms, qualification scenes, gate
   sequence, fallback policy, scientific-NO-GO transition and campaign activation. Any protected-field change
   without a distinct authorized re-freeze transition fails closed; ordinary decision metadata and Supervisor
   review prose may advance.
4. Keep every execution-affecting tracked product source, config, launch, runner, analyzer, test, probe/build
   input and shared-install byte under current-worktree/current-install strict validation. Do not weaken source
   admission, installed-byte checks, output containment, held-out-token rejection, one-shot/no-retry controls,
   required-process health, first-missing-stage or owned-process cleanup.
5. Update the schema/config/validator only as narrowly as needed. Add adversarial coverage proving:
   - a legitimate post-freeze update to each of the exact three governance documents keeps its recorded Git blob
     snapshot valid when protected fields do not change;
   - a protected route-field mutation is rejected;
   - blob path/source-commit/OID/type/size/SHA tampering is rejected;
   - missing, non-ancestor or not-fetch-confirmed source commits are rejected;
   - ordinary product/config/runner/analyzer/test/build-input mutation remains rejected;
   - no fourth governance path can use the snapshot treatment;
   - the retained three-document drift reproduces the old failure and passes only through the new bounded model.

## Fresh pushed-source evidence

1. Before source work, fetch-confirm `HEAD...origin/dev/icra = 0 0`, preserve the protected PDF and record the
   exact dirty/untracked state. Never overwrite blocker-001 or an earlier evidence identity.
2. Commit and push the bounded source/test/config change before generating canonical evidence. Then fetch-confirm
   divergence `0 0` and run the exact shared six-package build required by the frozen verification contract;
   reuse `/home/dev/ws_iap/{build,install,log}` and retain all ordinary logs.
3. Run the focused adversarial suite, affected tests, route verifier and hook-path verifier. Produce new,
   non-overwriting repository-local identities no earlier than:
   - `repeatability-replay-004`;
   - `verification-003`;
   - `preregistration-freeze-006`.
   Each must bind the pushed source, exact shared install, prior disclosed debt, protected-route fingerprint and
   exact governance Git blobs. Prior replay-003, verification-002 and freeze-005 remain immutable history.
4. Validate the new freeze at the final evidence HEAD. Also validate a controlled test fixture representing a
   later Supervisor-only governance prose update; the test must not mutate the real authority documents after
   the Builder creates freeze-006.
5. Push all authorized source, test, docs and fresh evidence; fetch-confirm divergence `0 0`. Return
   `ICRA077A_PRE_ACCESS_FREEZE_CLOSURE_READY_FOR_REVIEW` only when the offline closure is complete. Otherwise
   retain the first typed blocker.

## Forbidden scope and exit

- No held-out outcome opening/generation/analysis, GPU preflight, ROS, live run, 360-row execution, result
  interpretation, threshold/SESOI/sample-size/seed/order change, retry/exclusion, ICRA-077B, ICRA-078,
  qualification, campaign or publication/effect claim.
- Do not broaden the governance snapshot set, weaken current-byte validation for executable inputs, or use a
  current dirty governance file as the frozen content source.
- Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence, ordinary logs,
  hidden user artifacts and untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
- Only a subsequent Supervisor Review PASS of ICRA-077A may issue ICRA-077B for the frozen 360-row one-shot
  confirmation. ICRA-078 remains blocked until ICRA-077B Review PASS.
