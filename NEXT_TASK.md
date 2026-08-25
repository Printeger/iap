# ICRA-066 — Emit the authoritative offline P4-G0C scientific NO-GO

> Active gate: `P4_G0C_R6_AUTHORITATIVE_OFFLINE_NO_GO_OUTPUT`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA065_ANALYZER_IMPLEMENTATION_PASS_SUPERVISOR_EXPECTATION_CORRECTED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: frozen-hash check -> one authoritative analyzer call -> NO-GO handoff

## Supervisor decision

ICRA-065 code and evidence pass Review. The analyzer now validates immutable recovery-time alias A against the
frozen inventory while independently validating final mutable alias B; retains all structurally complete rows;
applies the `1e-12 risk_cost` floor to aggregate Type-7 Q10 gates; and distinguishes technical `REJECTED`,
scientific `SCIENTIFIC_NO_GO` and eligible `DRAFT_ELIGIBLE` outcomes.

The ICRA-065 validation stop was correct under its literal instruction, but the frozen expected mean Q10 was a
Supervisor calculation error. `sort -n` did not order scientific-notation values numerically. Exact float parsing
over all 192 rows gives Type-7 `h=19.1`; both bounding values are `0.000020000000000131024`, so:

~~~text
mean_improvement_gate = 0.000020000000000131024  > 1e-12  PASS
max_improvement_gate  = 0.0                      <= 1e-12 FAIL
~~~

The corrected value does not change the scientific result: P4-G0C is a genuine NO-GO because the risk guide
does not reliably improve the path maximum-risk metric. ICRA-066 is a single offline output task, not another
development, validation, format or live-execution loop.

## 1. Bind the already-reviewed implementation and evidence

- Follow `AGENTS.md` Git synchronization. Preserve the protected PDF and all raw/recovery/build/install evidence.
  Start a minimal ICRA-066 ledger as early as practical; ledger timing/format is documentation only and must not
  block the scientific call unless a credential is persisted or a task/external file is actually modified.
- Require `HEAD == origin/dev/icra`; require reviewed Builder commit
  `49730bfde7cbc63818ce6833b583c2191ae81592` to be the immediate parent of this Supervisor handoff and require
  the intervening diff to contain only `AGENT_STATE.md`, `NEXT_TASK.md` and `SUPERVISOR_LOG.md`. Then bind:
  - analyzer SHA-256: `ae2517e8fa1f392db45434cb7d005872f2ee721fdd6d15c177b4c3c6c69e0b42`;
  - analyzer-test SHA-256: `8e867e2a58dceed09f1fa6b0a213392419b42afb11851f59ad659be0e9ffa219`;
  - runner-state SHA-256: `475004c649af993013e2a58736b7bba78b384d0d28c4c700a92c5f940eee1dd1`;
  - retained recovery inventory SHA-256:
    `b39ddec294a7df0680b204509418a410d1f209449e7ed4d8292b55091d6e784e`;
  - old analysis and preserved-copy SHA-256:
    `f584fc5152b9060606365f283b374991aa0f07be6f98ac816a7b9a31fa9d7391`;
  - ICRA-065 frozen-input verification SHA-256:
    `1a8fd866bb1e48139d9ab4e56abf286d6fbf6fddbdf29d5c32ac859e6fd3c95c`.
- Require the ICRA-065 `input_hashes_after.json` status `PASS`, file count 103, empty mismatches, and exact equality
  of its `files` array with `input_hashes_before.json`. Do not recreate either manifest or rerun validation.
- The ICRA-065 pre-ledger Git/read ordering deviation is explicitly waived as non-scientific. Do not repair,
  reconstruct or re-execute history. The small duplicated test-helper smell is nonblocking and out of scope.

## 2. Replace only the obsolete analysis output

- Confirm the byte-exact old output is still present both at
  `results/icra27/icra063/runs_final/p4_g0c_analysis.json` and at the preserved
  `results/icra27/icra065/rejected_analysis_icra064.json`, with the frozen SHA above. Confirm the threshold draft
  is absent and registry bytes are unchanged.
- After all checks pass, remove only the exact obsolete `runs_final/p4_g0c_analysis.json`. This replacement is
  explicitly authorized because its old bytes are already retained and hash-bound outside `runs_final`.
  Do not delete, move or modify any other raw, runtime, recovery, runner, config or evidence file.
- Do not run another validation preflight or test suite. ICRA-065 and Supervisor Review already passed focused
  41/41 and full hermetic 516/516; repeated verification would add no scientific information.

## 3. Invoke the authoritative analyzer exactly once

- Run the reviewed analyzer exactly once with frozen v6 protocol/registry/fixture, the unchanged
  `results/icra27/icra063/runs_final` root, the exact retained recovery inventory, and output exactly
  `results/icra27/icra063/runs_final/p4_g0c_analysis.json`.
- Analyzer process exit `2` is the expected typed scientific NO-GO, not a task failure. Accept it only if the
  newly created output proves all of the following:
  - `analysis_status == SCIENTIFIC_NO_GO`;
  - `failures == []` and there are zero technical/provenance failures;
  - registered/attempted/completed runs are `15/15/15`;
  - complete/denominator decisions are `192/192`;
  - mean Q10 is exactly `0.000020000000000131024` and passes the frozen floor;
  - max Q10 is exactly `0.0` and `failed_scientific_gates` contains only
    `max_improvement_gate_at_or_below_noise_floor`;
  - retained recovery inventory SHA matches; registry is not updated; application remains disabled;
  - no threshold draft exists.
- If the one call returns a technical rejection, different metrics, writes no output, or changes any frozen
  input, stop and hand off without a retry. Do not restore by rerunning, tune thresholds or modify analyzer code.

## 4. Document, push and return

- Update only Builder-owned `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, the ICRA-066 ledger and one
  compact redacted final-result file. State clearly that runner PASS plus analyzer technical PASS led to a
  scientific P4-G0C NO-GO.
- Do not change product/analyzer/test/config code. Explicitly stage only the authorized docs/compact files;
  raw `runs_final/p4_g0c_analysis.json` remains retained locally and must not be staged unless already governed
  by the repository ignore policy. Never stage the PDF or raw run products.
- Commit with applicable requirement IDs and push, then return to Supervisor. Do not choose or execute G0D/P5.
  Supervisor will close P4 and issue the P0+P5 contingency-route task after reviewing this authoritative output.
- Retain all build/install products through Supervisor Review. After ICRA-066 Review PASS and pushed docs,
  Supervisor will remove only reproducible completed-task build/install directories; scientific/recovery/
  analyzer evidence and the protected PDF remain.

## Allowed files

- The exact authoritative `results/icra27/icra063/runs_final/p4_g0c_analysis.json` replacement.
- Builder-owned docs plus minimal ICRA-066 ledger and compact result evidence.

## Forbidden

- No source/test/config/protocol/registry/fixture/dependency/lineage/runner-state/raw-run change; no validation
  rerun, unit/full test rerun, build/install mutation, GPU preflight, ROS/launch, runner or identity execution,
  r7, row/run exclusion, formula/floor/threshold/seed/order/repetition change, draft/application, G0C PASS,
  G0D/P5 execution, external-repository write, credential persistence, PDF/raw-product staging or cleanup.
