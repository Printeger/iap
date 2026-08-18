# ICRA Supervisor Log

## 2026-08-18 — Reconciled bootstrap and ICRA-001 review

### Review identity and synchronization

- Branch: `dev/icra`
- Review base: `8d4ec35ac80445bfeb5998f37bef3efd7654e7ab`
- Reviewed HEAD: `54ba4a64088db28deae18424eb9bdb12a91e8a63`
- Commit reviewed: `54ba4a6 test(icra): add Gate-0 read-only qualification evidence IAP-RQ-320 IAP-RQ-400 IAP-RQ-410 IAP-RQ-422`
- Startup synchronization: `git fetch origin`; divergence `0 0`; `git pull --ff-only origin dev/icra` reported already up to date. The existing untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.md` was preserved and is included in this reconciliation.

### Verdict

- Overall verdict: `NO_GO_P2`.
- Gate 0A narrow verdict: `NO_GO_P2`. The fixed seed-11, three-scenario, three-repeat evidence contains 378 planning attempts, 378 base candidates, 378 optimizer inputs and 378 optimizer successes. Every attempt is singleton and no attempt satisfies `generated >= 2 && optimizer_success >= 2`. This is sufficient to freeze the P2 conference route.
- The Gate 0A verdict is not a complete-system qualification. It does not establish valid GNSS/LiDAR integrity input, a working P0 generation, P0 performance, or P5 system behavior.
- Gate 0B verdict: `BLOCKED / P0_INPUT_AVAILABILITY_FAIL`, not a valid performance result. The run produced zero real P0 generations and executed zero 76,800-query workloads, so p50, p95 and max latency are unmeasured.
- Active conference route: P0 + P5. P0 supplies only a future-PL advisory field; P5 remains the IAP layer's sole hard integrity gate; original EGO collision/dynamics checks retain motion-feasibility authority.

### Standards axis

Hard findings:

1. The Gate 0 work created and chmod'd an archive under `/home/dev/ws_iap/backups/...`, outside `src/iap`. This violates `AGENTS.md` section 0. Existing data is retained, but no future ICRA task may repeat the write or alter it.
2. ICRA-001 expanded into Gate 0B execution and assigned a subsequent research direction without a Supervisor handoff. The required collaboration state/log/task files were absent.
3. `docs/CHANGES.md` describes the campaign but does not preserve the exact reproducible commands and exit codes required by the repository Definition of Done.
4. The new `IAP-RQ-422` traceability rows map launch isolation, hashing and an external dependency archive to a requirement whose declared seam is per-waypoint `PL_pred_ARAIM_i - AL_i`; this mapping is inaccurate and must be corrected in ICRA-002 without rewriting history.
5. `launch/test_planner.launch.py` changed general mirror-resolution semantics so an explicit manager value overrides the fixture-derived value. Gate 0 was limited to default-off read-only instrumentation; this behavior change exceeded that boundary even though its regression preserves the legacy fallback when no override is provided.
6. The aggregate Gate 0 CSV rows omit parts of the preregistered row-level provenance contract, including commit/configuration hash, seed and scenario. The ignored run manifests are not a substitute for the declared per-row fields.

Non-blocking maintenance risks:

- `planner_manager.cpp` repeatedly constructs large `Gate0QualificationEvent` and `Gate0ControlPointEvidence` records at individual hooks. This is duplicated event-construction logic.
- Event kinds, reasons, sentinel integers and lifecycle data are represented as primitive strings/integers. This primitive event model makes invalid combinations easy; do not refactor it during ICRA-002 unless required for the explicitly authorized evidence contract.

### Spec axis

Accepted evidence:

- The fixed logical seed, nine runs and 378 optimizer-success singleton candidates support the narrow `NO_GO_P2` decision. P1 fanout/supplement did not create the observed singleton set, and the selected singleton lineage reached recorded downstream EGO/update/publish events.

Rejected or incomplete evidence:

1. The top-level launch and runner manifests report exit 0 and `planner_crash=false`, while the raw logs show `iap_rosnode` died with exit `-6` after repeated `cudaErrorNoDevice`. In all nine Gate 0A runs, the integrity validator later exited 2 with zero integrity messages. The P0 run also lost `iap_rosnode`; its no-validator configuration hid that prerequisite failure from the manifest.
2. Consequently, the captured `message_stamp_unavailable`/`snapshot_unavailable` callbacks are downstream symptoms after an upstream required process died. They cannot support a P0 performance conclusion or performance-tuning recommendation.
3. The runner records only the top-level launch/capture return codes. It has no structured required-process result and treats launch exit 0 as success even when required child processes die.
4. The analyzer does not fail closed on every non-finite original-cost/control-point evidence case and its current process check can only inspect the incomplete runner manifest. Downstream aggregation also couples `selected_reached_downstream` to `qualified`, causing singleton downstream evidence to disappear in run-level aggregates.
5. Instrumentation expanded beyond the smallest Gate 0A observation seam into launch behavior, disk/archive tooling, P0 capture/analysis and broad planner hooks. This scope is not accepted as precedent for further expansion.
6. Gate 0 does not implement or validate `IAP-RQ-422`'s per-waypoint ARAIM-PL/dynamic-AL hinge and safer-path acceptance criterion; no such product requirement may be marked verified from these diagnostics.

### Required next action

- Unique next task: `ICRA-002 / GATE_0B`, defined in `NEXT_TASK.md`.
- Active role: `DEEPSEEK`; state: `TASK_READY`.
- First restore a live CPU mapping/integrity input path and one real P0 generation. Do not develop P2, alter P5 decisions, tune the fixed Gate 0B workload, run a campaign, create backups, or clean disk.
