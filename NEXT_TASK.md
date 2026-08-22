# ICRA-023 — Repair review ownership and historical-validator provenance

> Active gate: `GATE_0B`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA022_PRODUCT_PASS_STANDARDS_REPAIR_REQUIRED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`
> Conference route: conditional P0 -> P4 -> P5
> This task: documentation/provenance review repair only; no product or live-flow work

## Supervisor decision

ICRA-022's product repair is technically accepted. Independent verification passes plan-env 6/6,
P0 76/76, Adapter 7/7, rolling 23/23, retained Ego 8/8, P4 4/4, P1 integrity 39/39,
analyzer 25/25, runner 16/16 and capture 1/1. Current binaries resolve the repository-local
ICRA-022 `libiap.so` and `libplan_env.so` at their recorded hashes. The occupancy source stamp,
atomic generation, fail-closed invalid/future/stale behavior and analyzer classifications match the
issued functional specification.

Overall review is `REQUEST_CHANGES` for two process/provenance issues:

1. Builder logs and verification summary called the Builder's self-check a “final two-axis review”.
   Under `AGENTS.md`, only Supervisor can issue the final Standards/Spec verdict. The pushed final
   handoff commit also omitted every `IAP-RQ-XXX`; pushed history must not be rewritten, but the
   breach must be acknowledged and not repeated.
2. ICRA-022 required both a new case in `test_p0_risk_grid_runtime.cpp` and a passing ICRA-020
   validator. That historical validator runs `git diff --quiet` between ICRA-020's implementation
   commit and the current tree for the same test file, making the issued requirements mutually
   impossible. This was a Supervisor specification conflict. It is not a product failure and not an
   external environment blocker.

Repair only these review/provenance contracts. Do not alter ICRA-022 product code or run any live
flow. After ICRA-023 review passes, Supervisor will separately freeze the formal-generation
distribution and decide whether to authorize one replacement smoke.

## 1. Synchronize and preserve

- Follow the `AGENTS.md` synchronization protocol. Stop as `REMOTE_DIVERGED` if both sides lead;
  never reset, clean, stash, rebase, amend pushed history or overwrite another role's work.
- Preserve the untracked PDF and historical ICRA-011/014/020/021 evidence exactly. Required hashes
  remain those frozen in ICRA-022. Do not modify the ICRA-020 canonical JSON.
- Preserve the accepted ICRA-022 product implementation commit `544451f19e879a944fbc3264415248d1e43aa03a`.
  No product header/source/test behavior may change in this task.
- Retain all existing ICRA-022 `build*`/`install*` during development and Supervisor review. Reuse
  them read-only for tests/linkage; do not delete or rebuild them unless a real binary mismatch is
  proven. Supervisor deletes them only after review PASS and management-document push.
- Record an ICRA-023 START entry with the exact allowlist and stop line. Do not edit Supervisor-owned
  state/task/log/scope/plan/design/Gate documents.

## 2. Correct role and handoff claims

- Append a clear `DEV_LOG.md` erratum: the ICRA-022 phrases “final two-axis review”, “Standards PASS”
  and “Spec PASS_WITH_EXTERNAL_BLOCKER” were Builder self-checks only, not Supervisor verdicts.
- Correct `results/icra27/icra022/verification_summary.txt` so it labels the result “Builder
  self-check” and calls the validator failure an issued-spec/historical-provenance conflict, not an
  external blocker. Preserve commands, exit codes, hashes and the fact that the task returned for
  Supervisor review.
- Do not edit, amend or replace existing commits. Record that `2bd5ba4` lacked the mandatory RQ ID;
  every new ICRA-023 commit, including the final DEV_LOG-only handoff, must contain at least one
  applicable `IAP-RQ-320` or `IAP-RQ-322`.
- Builder may state test results and a Builder self-check. It may not declare a final Standards/Spec
  verdict, Gate decision, Supervisor PASS, next task or “Supervisor handoff” commit subject.

## 3. Make the ICRA-020 read-only validator provenance-stable

The ICRA-020 artifact describes its immutable recorded implementation commit. Later current-tree
test additions do not change that historical commit and must not invalidate the artifact.

- In `test/test_icra020_p0_rolling_worker_profile.py`, remove the requirement that the current HEAD/
  worktree be byte-equal to the recorded implementation commit for `IMPLEMENTATION_FILES`.
- Continue requiring the exact 40-hex `implementation_sha`, and use Git object checks to prove that
  the commit exists and that each required implementation path exists as a blob in that exact
  commit. Do not substitute current-tree contents for recorded-commit contents.
- Preserve every canonical schema, workload, sample, scientific equality, counter, timing,
  percentile, command, compiler/build provenance and no-promotion assertion.
- Preserve the post-review ephemeral policy: a recorded build/library path may be absent; if it
  exists it must be a regular repository-local file with the exact recorded SHA-256.
- Fail closed on a malformed/nonexistent commit, a missing recorded source path, wrong existing
  binary/library hash, modified canonical JSON or any scientific/counter/timing contradiction.
- Add focused coverage/helper assertions for valid recorded-commit provenance and invalid commit/
  missing-path provenance. The current ICRA-022 tree, which legitimately contains a later P0 test,
  must pass without relaxing any canonical artifact value.

This is validator-code maintenance only. Do not regenerate, rewrite or “update” the ICRA-020 JSON,
its implementation SHA, recorded paths or hashes. Do not invoke the disabled ICRA-020 profiler.

## 4. Verification

Write any new logs below `results/icra27/icra023/`; do not create new build/install trees.

- Run the direct ICRA-020 validator and the selected-root CTest; both must pass, yielding root 8/8.
- Rerun analyzer 25/25, runner 16/16 and capture 1/1.
- Reuse retained ICRA-022 binaries to rerun plan-env 6/6, P0 76/76, Adapter 7/7, rolling 23/23,
  retained Ego 8/8, P4 4/4 and P1 integrity 39/39.
- Recheck direct consumer linkage to the retained ICRA-022 `libiap.so`/`libplan_env.so` and verify
  their hashes remain
  `d988f19ce7a4f08f145cd4643f7cd66e26f3f9849d03db836107cae23ebcbe31` and
  `cadd44115d026695547a53b4ac884d4c80a851882d9cd1c942103dfe43ae1ecf`.
- Run `git diff --check`, inspect the staged allowlist, verify protected hashes/PDF and check no task
  process remains. Keep package-wide historical lint debt separate; no formatting sweep.

No GPU preflight, ROS, simulator, capture of live topics, smoke, qualification, campaign or formal
analyzer run is authorized.

## 5. Documentation and handoff

- Update `docs/CHANGES.md`, `docs/TRACEABILITY.md` and `DEV_LOG.md` with the provenance distinction,
  exact verification and explicit `Gate-0B NOT_QUALIFIED` statement.
- Push the validator/documentation commit with applicable RQ IDs, then push one final
  `DEV_LOG.md`-only handoff commit that also contains an applicable RQ ID.
- Return control to Supervisor. Do not issue the next task or decide whether a replacement smoke may
  run.

## Allowed files

- `test/test_icra020_p0_rolling_worker_profile.py`;
- `results/icra27/icra022/verification_summary.txt`, only for role/blocker label correction;
- new verification logs below `results/icra27/icra023/`;
- `DEV_LOG.md`;
- `docs/CHANGES.md`;
- `docs/TRACEABILITY.md`.

## Forbidden

- No modification to ICRA-022 product headers/sources/tests, Gate runner/analyzer, launch/default/
  YAML, worker/scientific configuration, P0 runtime, Predictor/RiskGrid/rolling/Adapter/plan-env,
  P1/P2/P3/P4/P5 code or public Interface.
- No rewrite/amend/rebase/force-push of existing history and no deletion of retained build/install.
- No canonical artifact change, ICRA-014/020 disabled-profile invocation, threshold relaxation,
  evidence-value relaxation or acceptance of a wrong existing binary/library.
- No GPU preflight, ROS/main flow, replacement smoke, qualification, bag, RViz, campaign, formal-
  generation distribution selection or Gate promotion.
- No modification of the untracked PDF, `src/glim`, another repository or external user data.
