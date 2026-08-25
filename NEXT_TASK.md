# ICRA-065 — Correct the r6 analyzer offline and freeze the P4-G0C NO-GO

> Active gate: `P4_G0C_R6_OFFLINE_ANALYZER_CORRECTION_AND_NO_GO_FREEZE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA064_RUNNER_PASS_ANALYZER_REQUEST_CHANGES_EXPECTED_P4_G0C_NO_GO`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: analyzer semantics -> offline reanalysis once -> authoritative scientific verdict

## Supervisor decision

ICRA-064 made material progress and closes the live r6 execution phase. All 15 registered identities are
unique, attempted and complete; there are exactly 15 launches, zero retries/exclusions, two runner sessions
(`1 + 14`) and two passing GPU preflights. All 192 decision rows are metrics-only, structurally complete,
200/200 in both arms, process-valid and retained in the denominator. No identity may ever run again.

The current analyzer combines one false tooling rejection with a real scientific result:

1. The recovery record correctly freezes the shared ROS `latest` target at adoption time. Fourteen later ROS
   launches legitimately advanced the mutable convenience alias. Requiring the historical target to equal
   the final current alias is a provenance-model bug.
2. The preregistered plan defines the improvement gates as Q10 over all complete decision improvements and
   compares those aggregate gates with the frozen numerical-noise floor. The current analyzer instead rejects
   every row whose individual mean or max improvement is at/below the floor, reducing 192 complete rows to
   136. That is not the registered formula.

Supervisor read-only replay over the unchanged 192 rows obtains Type-7 Q10 mean improvement `0.000304` and
Type-7 Q10 max improvement `0`. The frozen floor is `1e-12 risk_cost`; therefore P4-G0C is expected to close
as a genuine scientific NO-GO on the max-improvement gate. This result must not be hidden by row exclusion,
threshold relaxation, another live run or a new r7 identity.

## 1. Freeze the exact input and old analyzer result

- Follow the `AGENTS.md` Git synchronization protocol. Preserve the protected PDF and all retained r6 raw,
  runtime, recovery, build and install artifacts. Start the ICRA-065 command ledger with the first task command.
- Before replacing any analyzer output, preserve a byte-exact offline copy of the current rejected analysis
  under `results/icra27/icra065/` and bind these Supervisor-frozen hashes:
  - current analysis: `f584fc5152b9060606365f283b374991aa0f07be6f98ac816a7b9a31fa9d7391`;
  - completed runner state: `475004c649af993013e2a58736b7bba78b384d0d28c4c700a92c5f940eee1dd1`;
  - retained pre-recovery lstat/content inventory:
    `b39ddec294a7df0680b204509418a410d1f209449e7ed4d8292b55091d6e784e`;
  - recovery transition: `2808ae7e99f368ce173127291e349bca9c456754d71218f2f861605103a018ea`;
  - canonical original terminal state:
    `36750a604399fcdbdbada943ec5f46ab9024906085b4b5f43a41d86e6298ad6b`.
- Verify before and after the task that every run CSV/manifest/stdout/inventory and all protocol/registry/
  fixture/dependency/lineage inputs are byte-identical. Do not modify the authoritative runner state.
- The ICRA-064 ledger's historical `schema_version=icra063_command_ledger_v1` label is a waived Low metadata
  defect. Do not rewrite evidence or rerun anything to repair it. Use an ICRA-065 schema label in the new ledger
  and record the waiver once in `DEV_LOG.md`; it is not a Gate blocker.

## 2. Correct recovery provenance without weakening alias safety

- In the r6 analyzer, treat `recovery.shared_ros_logs_latest_target` as immutable recovery-time provenance,
  not as the expected value of the final mutable `launch_environment/ros_logs/latest` link.
- Add a required, explicit r6 analyzer input for the retained pre-recovery lstat/content inventory. Validate its
  exact schema, expected roots, frozen SHA-256 above and the `ros_logs/latest` typed entry. The recovery record's
  historical target must exactly equal that frozen entry's target.
- Require the historical target to be canonical, inside the exact task-local `ros_logs` root, an ordinary
  direct-child directory with no symlink chain, and still represented by retained/raw inventory evidence.
- Independently validate the final current `ros_logs/latest` alias using the existing exact safe-alias contract.
  It may point to a different ordinary direct child after later launches. Every alternate-name, escape,
  dangling, nested, chained, replaced or non-directory case remains fail-closed.
- Add an end-to-end regression in which session 1 freezes target A, session 2 advances the final alias to safe
  target B, and the analyzer accepts provenance. Add adversarial tests for tampered historical target/inventory,
  hash mismatch, missing target, outside-root target, unsafe final alias and target replacement.

## 3. Implement the preregistered aggregate noise-floor gate

- A decision is complete when its schema, identity, metrics-only state, selected/original identity, 200/200
  coverage, zero invalid counts, path-ratio cap and timeout requirements pass. Individual improvement at or
  below the noise floor does not make that row incomplete and must not remove it from the metric sample.
- Compute the existing deterministic Type-7 quantiles over all structurally complete retained rows:

~~~text
mean_improvement_gate = Q10(original_mean - risk_mean)
max_improvement_gate  = Q10(original_max  - risk_max)
path_ratio_gate       = min(1.30, Q95(path_ratio) + 0.02)
dual_search_p95_gate  = min(0.40 s, Q95(total_search_s)
                            + max(0.01 s, 0.20 * Q95(total_search_s)))
~~~

- Compare the two resulting Q10 improvement gates, not individual rows, with the frozen `1e-12 risk_cost`
  floor. Preserve every row and run in the denominator. Do not change Type-7 ordering/tie behavior, formulas,
  units, floor, seeds, repetitions, hard caps or minimum-complete-decision requirement.
- Separate technical rejection from scientific NO-GO in typed output. A structurally/provenance-invalid bundle
  remains `REJECTED`; a technically valid bundle whose aggregate improvement gate is at/below the floor must
  report an explicit `SCIENTIFIC_NO_GO` with the computed calibration statistics and exact failed gate(s).
  It must not emit a threshold draft. Preserve `DRAFT_ELIGIBLE` behavior for a truly eligible synthetic bundle.
- Add tests proving: fewer than 10% individual floor-level rows can still yield an eligible Q10 when the Type-7
  result is above the floor; a zero Q10 max gate yields `SCIENTIFIC_NO_GO`; all valid rows remain complete and in
  the denominator; technical failures never masquerade as scientific NO-GO.

## 4. Verify offline, then analyze the unchanged bundle exactly once

- Run focused analyzer/protocol tests and full hermetic Python discovery. No C++ rebuild is needed; source and
  the retained 7/7 risk-A* result are frozen. Verify no external ROS-log delta and no task process remains.
- Before the authoritative call, run a validation-only/read-only analyzer preflight if needed. It must not
  create or replace the analysis output or draft. Once tests and all frozen hashes pass, invoke the corrected
  analyzer exactly once on the unchanged completed r6 bundle, supplying the exact retained recovery inventory.
- Expected authoritative result:
  - technical provenance/structure failures: zero;
  - registered/attempted/completed runs: `15/15/15`;
  - denominator and complete decisions: `192/192`;
  - Type-7 Q10 mean improvement: `0.000304 risk_cost`;
  - Type-7 Q10 max improvement: `0 risk_cost`;
  - status: `SCIENTIFIC_NO_GO` caused by the max-improvement gate;
  - threshold draft absent, registry unchanged, application disabled.
- If the unchanged bytes produce different quantiles or a technical rejection after the specified corrections,
  stop and return evidence to Supervisor. Do not rerun the analyzer, reinterpret rows, tune science or launch ROS.

## 5. Document and hand off

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-065 evidence. Clearly
  distinguish `runner PASS`, `analyzer tooling corrected`, and `P4-G0C scientific NO-GO`.
- Commit with applicable requirement IDs and push. Explicitly stage only authorized task files. Do not edit
  Supervisor-owned files or stage raw run products, credentials or the PDF.
- Return to Supervisor after the one offline analysis. Do not start G0D/P5. The next Supervisor decision will
  choose between the P0+P5 contingency route and a separately justified P4 algorithm revision; Builder must not
  choose that research direction inside ICRA-065.
- Retain all current task build/install products through Supervisor Review. If ICRA-065 Review passes and the
  code/docs are pushed, Supervisor may then remove only reproducible completed-task build/install directories.
  Raw scientific/recovery/analyzer evidence and the protected PDF remain retained.

## Allowed files

- `scripts/dev_planner/analyze_p4_g0c_calibration.py` and its focused tests; the smallest protocol helper/test
  change strictly required to expose typed aggregate-gate statistics.
- Builder-owned docs, ICRA-065 command ledger and compact/preserved offline analysis evidence.
- The single corrected authoritative `p4_g0c_analysis.json` output after its old bytes are preserved and bound.

## Forbidden

- No GPU preflight, ROS/launch, r6 identity execution/retry, runner invocation or runner-state mutation. No r7,
  product C++, launch/config/fixture/protocol/registry/dependency/lineage/build/install change, formula/floor/
  threshold/seed/order/repetition change, row/run exclusion, draft application, G0C PASS claim, G0D/P5 run,
  external-repository write, credential persistence, raw-product/PDF staging or cleanup before Review.
