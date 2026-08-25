# Traceability Matrix (IAP)

## 2026-08-24 ICRA-040 P4-G0B review repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Request/occupancy invalidation after original search must outrank search and geometry interpretation | `planCollisionGuide()` revalidates canonical request identity and live occupancy epoch immediately after `searchOriginal()` returns. Focused epoch-mutation cases cover returned failure, timeout and duplicate geometry; all three produce invalid/replan, no original/risk/selected guide, no selection and no risk search. Stable failure, timeout and duplicate geometry retain the established typed results | **REPAIRED / fail closed** |
| IAP-RQ-423 | Effective `metrics_only` truth must survive attempt/snapshot setup | `setP4RiskSnapshot()` no longer rewrites the configured value. Every registered G0B integration fixture explicitly passes true. The focused risk-enabled false boundary remains false, measures a strictly lower mean and non-increasing max risk guide, records `SELECTION_NOT_AUTHORIZED`, selects original and keeps `selection_applied=false` | **REPAIRED / authorization stop preserved** |
| IAP-RQ-423 | Existing G0B and accepted planner baselines must remain green | Focused precedence passes 3/3 and boundary passes 1/1. Final independent regressions pass decision 15/15, integration 5/5, collision 17/17, P1 39/39, path-searching P4 5/5, occupancy epoch 6/6 and affected plan-manager CTest 9/9 with 186 active cases, one existing disabled and zero failures | **GREEN / zero failures** |
| IAP-RQ-423 | Repair evidence must be fresh, repository-local and bound to retained dependencies | Fresh ICRA-040 bspline and plan-manager configure/build/install products resolve ICRA-040 bspline plus retained ICRA-039 IAP/typesupport, plan-env and path-searching. Dynamic resolution, headers, missing-library, workspace-default, deleted-task, build-tree and RUNPATH audits pass; all ten retained ICRA-039 tree manifests remain exact | **LINKAGE / PRESERVATION PASS** |

Builder result is `P4_G0B_REPAIR_READY_FOR_REVIEW`, not G0B PASS. No
threshold, calibration, G0C/G0D, risk-guide application, GPU, ROS/live flow,
launch, runner, analyzer, capture, smoke, benchmark, qualification, campaign,
P5, cleanup or Gate promotion occurred.

## 2026-08-24 ICRA-039 P4-G0B metrics-only dual-guide decision

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | One same-event source of truth must own both complete guides, profile truth and selection truth | Immutable `P4GuideRequest` and schema-versioned `P4GuideDecision` bind attempt/segment/endpoints, snapshot owner/generation/stamp/frame, query base, cumulative-distance timing, occupancy epoch/config, complete path hashes, 200-point equal-arc profiles, lengths/ratio, latencies and typed outcome. CSV and RViz consume the decision rather than recomputing it | **IMPLEMENTED / deep seam** |
| IAP-RQ-423 | Original and risk search must preserve identity and fail/fallback exactly | `P4CollisionGuidePlanner` runs original first and risk second with the same request, rehashes its complete identity between searches, and reconstructs/rechecks that request before injection. The manager explicitly binds its real nonzero attempt ID; epoch/request mismatch is invalid/replan with the selected guide cleared. Missing/unknown/stale/non-finite/incomplete risk, risk failure/0.2 s timeout and ratio failure preserve current-epoch original | **VERIFIED / fail closed** |
| IAP-RQ-423 | G0B must measure but never apply the risk guide | Registered G0B contexts explicitly set `metrics_only=true` while the general parameter default remains false. Production A* on the central-obstacle `p4_collision_guide_v1` fixture yields repeat-stable request/original/risk hashes `1c8abe0fa4e4136a` / `2a3380ee05f43a1f` / `b3789ad7a8e50365`; both profiles are 200/200, risk mean/max `1/1.0000000000000002` is strictly lower than `2.0295422607088973/10.500000000000002`, ratio is `1.0`, selected equals original and `selection_applied=false`. Constraint hash equals original-only | **METRICS-ONLY / geometry no-op** |
| IAP-RQ-423 | Initial/rebound integration and the accepted baseline must remain green | Both paths call one collection/validation seam, retain the immutable snapshot through injection, bind attempt `73` without P1 context and preserve earlier open/invalid/interpolation-only stops. A focused post-decision epoch mutation proves the production wrapper records invalid/replan and clears selection. Final tests pass decision 11/11, integration 4/4, collision 17/17, P1 39/39, path P4 5/5, occupancy 6/6 and plan-manager 9/9 (186 active, one disabled) | **GREEN / zero failures** |
| IAP-RQ-423 | Fresh products and compact evidence must be repository-local and reviewable | Fresh ICRA-039 builds/installations and exact CMake/direct linkage pass with zero workspace-default IAP, deleted-task, build-tree, missing-library or non-toolchain RUNPATH matches. Compact JSON/XML/summary evidence is below `results/icra27/icra039/`; build/install trees remain unstaged for review | **LINKAGE / BOUNDARY PASS** |

This result is ready only for Supervisor review. It does not qualify G0B,
authorize thresholds/calibration/G0C, apply a risk guide or execute P5. No GPU,
ROS/live flow, launch, runner, analyzer, capture, smoke, benchmark,
qualification, campaign or cleanup occurred.

## 2026-08-24 ICRA-038 P4 rebound truth-preservation repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | A truthful closed scanner result must not be downgraded merely because its segment has no occupied interior integer control point | `check_collision_and_rebound()` verifies that each closed segment has truthful integer occupancy evidence before applying the legacy direction/base-point suppression. An adjacent `(2,3)` result preserves exact `CLOSED_SEGMENTS` status/endpoints, sets existing `STOP_FOR_ERROR` and returns before A*/guide work | **IMPLEMENTED / focused regression GREEN** |
| IAP-RQ-423 | An unclassifiable member must reject the whole multi-segment rebound attempt | The ordinary-then-adjacent `[(2,5),(6,7)]` regression proves that the first segment is not consumed as a partial result when the second cannot be classified; the full scanner result remains observable and A*/guide output remains absent | **VERIFIED / fail closed** |
| IAP-RQ-423 | Preserve the ICRA-037 production contract and deterministic baseline | Scanner source, initial collision path, planner-manager, frozen fixture and CMake are unchanged. Final collision passes 17/17, P1 39/39, retained path-searching P4 4/4, occupancy epoch 6/6 and affected plan-manager CTest 9/9 (186 active, one existing disabled) | **GREEN / zero functional failures** |
| IAP-RQ-423 | Bound artifacts and dependencies to the reviewed task chain | Fresh ICRA-038 bspline/plan-manager builds and installs use ICRA-037 IAP/typesupport and intended read-only ICRA-026 plan-env/path-searching. All six ICRA-037 build/install tree manifests and the frozen fixture/PDF hashes remain unchanged | **LINKAGE / PRESERVATION PASS** |

Exact RED/GREEN attempts, command ledger, identities, linkage and limitations
are retained below `results/icra27/icra038/`. No scanner redesign,
original/risk guide, G0B, P5, GPU, ROS/live flow, smoke, benchmark,
qualification, cleanup or Gate promotion occurred.

## 2026-08-23 ICRA-037 shared P4 collision-scan GREEN contract

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Represent scan outcomes truthfully through one production source of truth | `CollisionScanResult` carries the exact four statuses and ordered closed endpoints. `scanCollisionSegments()` preserves the legacy entry window, follows an active run through the complete tail, requires free endpoints with occupied interior samples, preserves scan order and clears every partial segment on open-ended or invalid input | **IMPLEMENTED / frozen contract 11/11 GREEN** |
| IAP-RQ-423 | Initial and rebound collision paths must share the scanner and fail closed before downstream guide work | `initControlPoints()` and `check_collision_and_rebound()` both consume the same result. Open-ended/invalid outcomes expose no segments; tests deliberately omit A* and prove both paths return without guide output. The planner-manager initial caller returns failure before candidate fanout/publication; only closed segments reach existing handling | **VERIFIED / focused integration 3/3** |
| IAP-RQ-423 | Preserve the frozen fixture and existing non-frozen behavior | Fixture SHA-256 remains `49a676a5…c788`. Final collision target passes 15/15, including a review-added same-control-interval overlap regression; existing P1 passes 39/39, retained path-searching P4 passes 4/4, occupancy epoch passes 6/6 and affected plan-manager CTest passes 9/9 | **GREEN / zero functional failures** |
| IAP-RQ-423 | Use only current task and intended retained dependency artifacts | Fresh ICRA-037 IAP/bspline/plan-manager build and install pass. Corrected CMake and direct linkage resolve ICRA-037 IAP/bspline plus read-only ICRA-026 path-searching/plan-env, with zero workspace-default IAP or missing product libraries. The protected PDF remains unchanged and unstaged | **LINKAGE / BOUNDARY PASS** |

Exact TDD attempts, seven former RED outcomes, test counts, hashes and the
corrected linkage disclosure are retained below `results/icra27/icra037/`.
Build/install trees remain for Supervisor review. No original/risk guide work,
profile/scoring/selection/fallback, P5, GPU, ROS/live flow, smoke, benchmark,
qualification, cleanup or Gate promotion occurred.

## 2026-08-23 ICRA-036 deterministic P4 collision-scan RED fixture

| Req ID | Requirement/evidence seam | Test and review evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Freeze deterministic scan inputs and the exact four-status contract without production behavior | Test-local fixture fixes 15 finite samples at integer `x=0..14`, a 0.25 m occupancy grid and cases for no collision, one closed, late exit, open ended, four invalid forms, multiple closed and closed then open. Expected closed endpoints are fixed and proven free with an occupied interior; status vocabulary is exactly `NO_COLLISION`, `CLOSED_SEGMENTS`, `OPEN_ENDED_COLLISION`, `INVALID_INPUT` | **FROZEN / test-only** |
| IAP-RQ-423 | Exercise only the narrowest truthful legacy surface and expose missing behavior as assertion-level RED | The observer calls `BsplineOptimizer::initControlPoints()` and translates only valid empty/nonempty results to no-collision/closed. It does not implement a reference scan, infer open/invalid states or synthesize endpoints. The compiled 11-case target passes four integrity/current-behavior tests and intentionally fails seven named assertions: late exit, open ended, empty, non-finite, structural invalid, unavailable occupancy and closed then open | **INTENTIONAL RED / `P4_G0A_RED_READY_FOR_REVIEW`** |
| IAP-RQ-423 | Preserve the existing functional baseline and exact dependency boundary | Fresh ICRA-036 IAP and bspline configure/build/install pass. Existing bspline integrity passes 39/39, retained path-searching P4 passes 4/4 and occupancy epoch passes 6/6. Ament/direct consumers resolve ICRA-036 IAP/bspline plus intended read-only ICRA-026 plan-env/path-searching only; no workspace-default, deleted ICRA-035, build-tree or missing product library is used | **GREEN BASELINE / linkage verified** |
| IAP-RQ-423 | Keep production and live scope untouched | Optimizer header/source hashes remain `6c52f424…52656` and `288d4cfb…45d3`; the protected PDF remains unstaged and unchanged. No production API/status/scan/guide change, GPU, ROS, runner/analyzer, smoke, benchmark, P4/P5 flow, cleanup or Gate promotion occurred | **BOUNDARY PASS / Supervisor review pending** |

Exact fixture data, attempt disclosure, final failure names/reasons, hashes,
commands, linkage and static-boundary notes are retained below
`results/icra27/icra036/`. The package-wide linter attempt separately exposed
pre-existing CMake whitespace, historical formatting divergence and the
existing xmllint timeout; the two new C++ files pass their focused format
check. These static observations are not counted as functional baseline
failures and no forbidden historical or production file was reformatted.

## 2026-08-23 ICRA-035 fixed 60-second P0 Gate-0B benchmark

| Req ID | Requirement/evidence seam | Qualification evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Current reviewed implementation and exact runtime path must pass all static gates before live use | Fresh ICRA-035 IAP/EGO build/install passes; affected IAP targets pass 6/6 and EGO targets pass 2/2; ament and five direct IAP plus one plan-env links resolve exact ICRA-035 / intended ICRA-026 prefixes with no workspace-default, ICRA-033, build-tree, missing or stale product library. Source/installed runner, analyzer, capture and launch hashes match | **STATIC PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Qualification must use the exact frozen workload and provisional growth profile | Effective config hash `97b4ccb…11e5` binds CPU, worker 4, 60/55 s, 30×30×6 m, 0.75 m, horizons 0.0–2.5 s by 0.5 s, 0.5 s refresh, occupied skip, no bag/RViz, safety off, P1–P5 disabled and exact provisional `0.01` / `legacy_iap_rq320_baseline_v1`; static/config/log/dependency/capture preflights all pass | **PASS / provisional, not empirical calibration** |
| IAP-RQ-320 / IAP-RQ-322 | Exactly one benchmark may start only after mandatory GPU preflight | Guard consumed one runner slot; RTX 4070 Ti SUPER discovery, both `nvidia-smi` calls, CUDA `cuInit(0)` and one-device query pass before ROS. Runner exits 0; `iap_rosnode` is observed alive with no runtime failure and stops only during controlled shutdown | **PASS / runner 1, retry 0** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Gate-0B requires at least 20 strict 76,800-query successes, valid integrity/counter/source evidence and refresh p95 ≤ 400 ms | Sole analyzer exits 0/PASS: 209 observations, 105 completed attempts, 103 strict successes, two typed failures, 18 in-progress, 86 equivalent duplicates, zero conflicts and 607/607 valid integrity. Refresh p50/p95/max `175.482122 / 184.1007665 / 199.520467 ms`; provider p50/p95 `146.82252 / 150.8886328 ms`; generation interval p50/p95 `500.135382 / 511.2421743 ms`; failed/stale ratio `0.019047619`; exact query shape true | **GATE-0B PASS / analyzer 1, retry 0 / SUPERVISOR REVIEW PENDING** |
| IAP-RQ-320 / IAP-RQ-322 | Evidence and terminal state remain repository-local and bounded | Compact evidence is below `results/icra27/icra035/`; 38 IAP and 17 ROS log files are task-local; external `log/` remains exactly `a07fbf79…4221f0`, 43,763 files and 15,834,674,845 bytes; no bag or task process remains; protected PDF remains unstaged at `1f07da56…44f6`; build/install trees are retained for review | **PASS / no cleanup or promotion** |

Exact build/test commands, disclosed pre-live corrections, one-shot commands,
guards, stdout/exits, hashes and post-live audit are in
`results/icra27/icra035/verification_summary.md`. No source, header, test,
analyzer, runner, capture, launch, config, CMake or product file changed. No
smoke, campaign, P1–P5 execution, tuning, retry, artifact cleanup or Gate
promotion occurred. Builder returns this PASS only for Supervisor review and
does not claim empirical calibration or full IAP-RQ-322 completion.

## 2026-08-23 ICRA-034 typed message-clock failure reanalysis

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | A completed startup failure may lack message time only when no message clock truthfully exists | `gate0_analyzer.py` recognizes only exact `COMPLETED_FAILURE` / `message_stamp_unavailable` records with all three timestamp keys explicitly null, finite ordered steady identity, finite nonnegative elapsed, zero result/work, active/previous equality and unavailable matching snapshot; success and other failure reasons retain finite message-stamp requirements | **IMPLEMENTED / analyzer direct suite 42/42** |
| IAP-RQ-320 / IAP-RQ-322 | Any partial or malformed typed failure must remain fail closed | Tests cover missing/partial/fabricated stamps; missing/nonfinite/reversed steady time; NaN/negative elapsed; nonzero work/provider/predictor counters; nonzero result; chain, snapshot and reason mismatch. Typed-only cumulative counters are included in completed duplicate equivalence, and a changed counter produces conflict; a separate regression preserves valid `IN_PROGRESS` active-map cumulative counters | **VERIFIED / compile and diff check PASS** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Reanalysis must consume immutable ICRA-033 evidence exactly once and preserve its identity | Guard records one allowed/one consumed invocation. Pre/post SHA-256 and byte counts exactly match health `d91a0af…61bc3` / 112,289, integrity `53a08cf…d869` / 39,237 and manifest `04e2e971…bf1a` / 6,404. Exact command/stdout/empty stderr/exit 0 and output hashes are below `results/icra27/icra034/` | **PASS / no retry** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Immutable smoke must retain strict successful workload, timing, lifecycle and integrity evidence | Formal result is PASS with 31 observations, 16 completed, 14 successful 76,800-query generations, two coherent typed failures, three in-progress, 12 equivalent duplicates, zero conflicts and 166/166 valid integrity; p95 refresh/provider/interval are `194.48499765` / `150.42874975` / `506.1757368 ms` | **ANALYZER PASS / SUPERVISOR REVIEW PENDING** |

No GPU, ROS, launch, runner, capture, smoke, qualification, build/install,
reanalysis retry, 60-second benchmark, tuning, P4/P5 execution, cleanup or Gate
promotion occurred. Exact requested/effective `0.01` and profile
`legacy_iap_rq320_baseline_v1` remain provisional and are not an empirical
calibration or a claim of full IAP-RQ-322 completion.

## 2026-08-23 ICRA-032 immutable captured-source publication

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | A coherent captured refresh transaction must survive normal newer live source publication without mixing versions | `p0_risk_grid_runtime.cpp` terminal validation retains captured nonzero/internal owner, generation and finite stamp requirements; same-generation mutation and regression fail, while newer current/GNSS/LiDAR/occupancy versions cannot revoke the captured transaction and are applied by the next refresh's invalidation/recompute. Production-shaped all-source in-flight and focused source-race/rollback tests cover the boundary | **IMPLEMENTED / deterministic evidence only; full IAP-RQ-322 not claimed** |
| IAP-RQ-320 | Exact provisional sigma `0.01 m/sqrt(s)` must execute the runtime prediction algebra | C++ regression uses exact `0.01`, reaches prediction, proves tau-zero equality to the reference covariance and strict monotonic growth at positive horizons | **VERIFIED / provisional baseline, not empirical calibration** |
| IAP-RQ-320 / IAP-RQ-322 | Startup observation classification must not weaken completed-callback evidence | Analyzer counts strict generation-zero/no-start/no-end `not_ready` rows as pre-refresh observations and its 40 tests plus immutable ICRA-031 replay pass. Final review found three omitted possible work claims (`generation_interval_ms`, LiDAR evaluations/cache hits), which can still be misclassified. The finding occurred after the one-shot stop and was not repaired | **PARTIAL / BLOCKED / no post-live correction** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Current build, tests, frozen config and linkage must pass before live use | Current IAP/EGO build/install pass under ICRA-032. Active counts pass: P0 runtime 78, adapter 7, Predictor 46, rolling 23 and RiskGrid 43; runner/launch pass. Ament/direct linkage resolve ICRA-032 IAP/EGO and intended ICRA-026 plan-env/path/bspline only. Frozen CPU, worker 4, 20/15 s, 30 x 30 x 6 m, 0.75 m, six horizons, 0.5 s, occupied skip, no bag/RViz, safety off and P1–P5 disabled contract passes | **PASS / all failed static attempts disclosed** |
| IAP-RQ-320 / IAP-RQ-322 | Run exactly one guarded replacement smoke and one strict analyzer after preflight | Config, GPU (`nvidia-smi`, `cuInit(0)=0`, one device), dependency, task-local log and capture readiness pass; sole runner exits 0. Sole analyzer exits 1: 166/166 integrity reports are valid and five final generations succeed, but 13 failed refreshes and timing/work/snapshot inconsistencies produce `P0_EVIDENCE_CONTRACT_FAIL` | **BLOCKED / no live retry / Gate-0B NOT_QUALIFIED** |
| IAP-RQ-320 / IAP-RQ-322 | Outputs and terminal state must remain repository-local and immutable | Runtime/analyzer evidence is below ICRA-032; no bag exists; external `log/` remains byte-identical at `a07fbf79…4221f0`, 43,763 files and 15,834,674,969 bytes; task-process matches are zero; protected PDF hash remains `1f07da56…44f6` | **PASS** |

Exact TDD/static/precheck attempts, replay, one-shot guards/exits, live evidence
and postrun audit are retained below `results/icra27/icra032/`. An initial
postrun canonical-hash operand used a nonexistent analyzer filename and was
corrected to the actual outputs without rerunning smoke or analyzer. No
post-live product correction, tuning, benchmark, P4/P5 execution, cleanup,
Gate promotion or next-task selection occurred. Gate-0B remains
`NOT_QUALIFIED` pending Supervisor review.

## 2026-08-23 ICRA-031 covariance-growth qualification bind

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-320 | Exact finite growth baseline must reach EGO only through the frozen qualification profile | Launch declares generic `NaN`, parses the supplied text directly to a float and passes exact `0.01` to `p0.predictor.sigma_grow_m_sqrt_s`; the manifest also records profile `legacy_iap_rq320_baseline_v1`. Launch tests prove exact ROS/manifest source, generic invalidity, float materialization and locale-like `0,01` rejection | **IMPLEMENTED / launch 16/16 PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Missing, non-finite, negative, non-exact or wrong-profile qualification config must stop before GPU/ROS | Runner config preflight persists exact requested/effective value/profile and provisional-not-calibrated provenance; focused tests cover missing, NaN, infinity, negative, `0.02` and profile mismatch with exit 6 and zero GPU/smoke calls | **PRE-GPU GUARD VERIFIED / runner 27/27 PASS / Gate-0B NOT_QUALIFIED** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Static science/config/linkage evidence must pass before live use | Current IAP configure/build/install pass below ICRA-031; corrected explicit-library affected suite passes 4/4; retained ICRA-026 P0 runtime passes 76 tests against ICRA-031 `libiap.so`; exact 12-package ament closure and direct linkage resolve ICRA-031 IAP plus ICRA-026 plan-env without missing/build/stale/workspace-default paths | **VERIFIED / disclosed command-environment attempt retained** |
| IAP-RQ-320 / IAP-RQ-322 | One frozen smoke must pass config, GPU, dependency, logging, capture and lifecycle guards | Sole runner exits 0; requested/effective sigma/profile are exact; `nvidia-smi`, `cuInit(0)=0`, one CUDA device, dependency and capture readiness pass; required `iap_rosnode` is healthy through runtime and only stops during controlled shutdown | **PASS / runner invoked once** |
| IAP-RQ-311 / IAP-RQ-320 / IAP-RQ-321 | At least one accepted final generation must contain exactly 76,800 queries | Sole analyzer exits 1 with `P0_EVIDENCE_CONTRACT_FAIL`; 166/166 integrity reports are valid, but 34 health observations have generation 0, zero successful generations, one malformed callback identity and raw reasons `prior_generation_changed=28`, `message_stamp_unavailable=5`, `not_ready=1` | **BLOCKED / no live retry** |
| IAP-RQ-320 / IAP-RQ-322 | Runtime output and post-run state must stay bounded | 30 actual IAP log files and one timing CSV are below ICRA-031 runtime; no bag exists; external `log/` identity remains exact at `a07fbf79…4221f0`, 43,763 files and 15,834,674,969 bytes; task-process matches are zero | **PASS** |

Exact commands, TDD/static attempt disclosure, one-shot guards/exits, canonical
hashes and post-run audit are in
`results/icra27/icra031/verification_summary.txt`. The `0.01 m/sqrt(s)` value
is the provisional original IAP-RQ-320 qualification baseline and is not an
empirical calibration. No C++ science/default, analyzer/capture, workload or
P1–P5 behavior changed. No live retry, 60-second benchmark, campaign, cleanup
or Gate promotion occurred; Gate-0B remains `NOT_QUALIFIED` pending Supervisor
review.

## 2026-08-23 ICRA-030 clock/log-repair replacement smoke

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-320 / IAP-RQ-322 | Reuse only accepted retained artifacts in one exact launch environment | Precheck attempt 01 verifies the frozen ICRA-028 `libiap.so` and ICRA-026 `libplan_env.so` hashes, exact ament identity for IAP and four planner packages, all required launch packages, and semantic direct-consumer linkage without missing/build/stale/workspace-default IAP or plan-env resolution | **PASS / no rebuild or relink** |
| IAP-RQ-320 / IAP-RQ-322 | GPU, dependency and capture readiness must pass before one live P0 smoke | RTX 4070 Ti SUPER, both `nvidia-smi` commands exit 0, `cuInit(0)=0`, one CUDA device; dependency closure and capture are ready; runner exit 0; `iap_rosnode` is seen with no runtime failure and only controlled-shutdown disposition | **PASS / runner invoked once** |
| IAP-RQ-311 / IAP-RQ-320 | The repaired clock path must produce at least one final valid 76,800-query generation | 208/208 integrity reports are valid, but all 27 final health representatives are `invalid_covariance_growth_parameter`, generation 0 and zero queries. The sole analyzer exits 1 with `P0_INPUT_AVAILABILITY_FAIL`, three exact availability/query-shape failures and no recommendation | **BLOCKED / no live retry** |
| IAP-RQ-320 / IAP-RQ-322 | Effective logging, actual runtime output and process lifecycle must remain task-local | Root/referenced logging configs, IAP log root, 34 actual log files and actual timing CSV are below ICRA-030 runtime; no bag exists; external `log/` before/after identity is exactly unchanged; post-run task-process count is zero | **PASS** |

Exact commands, exits, attempt disclosure, canonical hashes and bounded evidence
are recorded in `results/icra27/icra030/verification_summary.txt`. The
post-run audit was repeated only to correct its outer `tee` directory timing;
the runner and analyzer were each invoked exactly once. No product/test/config
change, configure/build/install/relink, retry, benchmark, qualification,
P4/P5 run, cleanup or Gate promotion occurred. Gate-0B remains
`NOT_QUALIFIED` pending Supervisor review.

## 2026-08-23 ICRA-028 production publication seam and verification repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-320 | Tests must exercise the same publication API as Demo11 production | Remove the unused array overload; `test_demo11_publication_stamp_authority.cpp` calls the sole variadic API with seven named production-shaped clouds and proves authority gating plus bit-identical fanout stamps | **IMPLEMENTED / focused root 5/5 PASS** |
| IAP-RQ-311 / IAP-RQ-320 | Invalid message stamps must not replace an accepted authority | After accepting `10 s + 20 ns`, test exact `kNonPositive`, `kMalformed`, `kMalformed` and `kRegressed` results for zero, negative-sec, nanosecond overflow and regression; each retains `10/20`, then `10/21` advances the next seven-cloud publication | **IMPLEMENTED / focused root 5/5 PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Linkage verification must express consumer semantics instead of a fixed aggregate count | Phase-1 logs require `test_run_log_manager` exactly `1/1` at ICRA-028 `libiap.so`, allow Demo11 `0/0`, and reject missing, build-tree and every non-ICRA-028 task resolution | **VERIFIED / semantic linkage exit 0** |
| IAP-RQ-320 / IAP-RQ-322 | Immutable phase 1 must verify whitespace and every repository-local audit before phase 2 | Script `33db6b9a…3027997e` ran once and reported all commands exit 0, but its whitespace command printed real trailing-space matches and then misclassified a `grep` self-output error as success | **BLOCKED / semantic false PASS / no phase 2** |

The ICRA-028 build/install and raw evidence remain below
`results/icra27/icra028/`. Protected hashes, historical aggregates, the
ICRA-026 leak identity, retained trees and zero task process matches were
reported intact before the semantic failure was identified, but the task does
not claim an overall phase-1 PASS. No forbidden product file or live-flow
boundary changed; Gate-0B remains `NOT_QUALIFIED` pending Supervisor review.

## 2026-08-23 ICRA-027 occupancy-clock, runtime-log and provenance repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-320 | All Demo11 occupancy producers share one authoritative simulator message clock | `Demo11PublicationStampAuthority`; `demo11_corridor_map_publisher.cpp` truth-odom subscription and one-shot multi-cloud stamp snapshot; `test_demo11_publication_stamp_authority.cpp` covers no-publish, regression retention, monotonic update and seven-cloud identity, but review found it exercises the array overload rather than the production variadic fanout and does not prove zero/malformed retention after acceptance | **IMPLEMENTED / focused root 5/5 PASS / REVIEW GAPS MEDIUM+LOW** |
| IAP-RQ-320 / IAP-RQ-322 | Future IAP runtime logs cannot fall back to repository `log/` | `_materialize_iap_logging_config()` rewrites root/referenced logging plus timing only after strict descendant validation; `test_test_planner_launch.py` proves paths and preserved logging semantics in a temporary runtime tree | **IMPLEMENTED / launch 14/14 PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Qualification must reject missing, relative or escaping effective log paths before capture/launch | `run_effective_log_path_preflight()` structured evidence, manifest binding and exit 5; `test_gate0_runner.py` covers valid evidence, invalid values and zero capture/launch calls | **IMPLEMENTED / runner 24/24 PASS** |
| IAP-RQ-311 / IAP-RQ-320 / IAP-RQ-322 | Verification commands are immutable and complete before execution | Pre-execution script SHA-256 `72234f09…ed414a3e`; configure/build/install, tests and `ldd` exit 0; immutable assertion exits 1 because only one of its two consumers has a dynamic `libiap.so` entry, so later script hashes/final audits do not execute. A subsequent reviewer mistakenly ran one out-of-script cached-diff check, which exited 1 on evidence whitespace and modified nothing | **BLOCKED / script not changed or retried / post-stop violation disclosed** |

No P0 consumer science, freshness threshold, workload, external
`local_sensing`, global checked-in logging default, scenario geometry,
planner/P4/P5 behavior or Gate state is changed. ICRA-026 retained trees,
ignored leaked log evidence and the PDF remain protected.

Exact commands and exits are in
`results/icra27/icra027/verification_commands.sh`,
`verification_results.tsv` and `verification_summary.txt`. Builder does not
claim the post-stop allowlist, hash, leak-identity, process or retained-tree
assertions passed.

The required two-axis review found no other scope or behavior defect. The
review-period cached-diff command is not the immutable script's unexecuted
final diff assertion and supplies no PASS evidence; its violation and nonzero
result remain part of this BLOCKED handoff.

## 2026-08-23 ICRA-026 dependency-guarded replacement smoke

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Rebuild the reviewed current tree without stale task artifacts | Five task-local build/install pairs below `results/icra27/icra026/`; Python suites pass 36/36, 21/21, 1/1 and 5/5; C++ suites pass 8/8, 6/6, 76/76, 7/7, 23/23, 8/8, 4/4 and 39/39; disabled profiles remain uninvoked | **VERIFIED** |
| IAP-RQ-320 / IAP-RQ-322 | Bind every direct consumer and future runner to the exact ICRA-026 environment | Seven `ldd` checks resolve only task-local `libiap.so` and `libplan_env.so`, hashes `144ecf56…de63c1c` / `360cf23a…f46447`, with no missing/old/external build-tree entry; literal ordered ament audit resolves all nine packages and exact task-local IAP/EGO | **VERIFIED** |
| IAP-RQ-320 / IAP-RQ-322 | Enforce mandatory GPU and dependency preflight before capture/launch | Sole runner records `gpu_ready=true`, `cuInit_result=0`, `device_count=1`; then records `launch_dependencies_ready=true` for all nine packages before capture/launch | **PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Preserve frozen smoke configuration and required-process lifecycle | Runner exit 0; capture ready; required `iap_rosnode` observed with no runtime failure and controlled shutdown; CPU mapping, worker four, 20/15 s, fixed grid/horizons/refresh, occupied skip, no bag/RViz, safety off and P1–P5 disabled | **PASS** |
| IAP-RQ-320 / IAP-RQ-322 | Require at least one valid 76,800-query P0 generation before Gate-0B can advance | Sole analyzer exit 1: 166/166 valid integrity rows but 19/19 health representatives are `occupancy_stale`, generation 0, zero queries; classification is `P0_INPUT_AVAILABILITY_FAIL` with zero successful generations | **BLOCKED / no retry** |
| IAP-RQ-320 / IAP-RQ-322 | Confine every task-created runtime/output path to ICRA-026 | Final Builder review binds ignored `log/20260823T034015Z_103` to this exact smoke through its `run_info.json` and retained task-local stdout; it is outside the allowlist and was neither staged nor modified because external cleanup is forbidden | **BLOCKED / Supervisor disposition required** |
| IAP-RQ-320 / IAP-RQ-322 | Retain exact build/test/linkage/environment commands | Build/test commands, seven `ldd` operands and the literal overlay are retained, but the original linkage aggregation/redirection wrapper, faulty assertion and executable static ament-audit command were not preserved verbatim and are not reconstructed | **BLOCKED / incomplete command provenance** |

Retained commands and explicit provenance gaps, complete hashes, preservation
checks, process audit and bounded one-shot evidence are recorded in
`results/icra27/icra026/verification_summary.txt`, `retained_command_record.txt`
and `process_audit.txt`.
No source/default/threshold changed; no retry, tuning, 60-second benchmark,
qualification, P4/P5 execution or Gate decision occurred. Gate-0B remains
`NOT_QUALIFIED` pending Supervisor review.

## 2026-08-22 ICRA-025 final-generation and launch-dependency repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Classify only the final captured representative for every positive integral generation | `gate0_analyzer.py::analyze_p0_messages()` de-duplicates all positive non-boolean integral generation IDs after callback-key selection and only then classifies/validates the final row; focused tests cover success→failure, failure→success, success→success and final invalid success, while `analyze_directory()` proves the final failure remains non-PASS with one CSV row | **IMPLEMENTED / analyzer 36/36** |
| IAP-RQ-320 / IAP-RQ-322 | Make generation de-duplication visible without success-only semantics | Summary field `duplicate_generation_observation_count` counts every overwritten positive-generation callback representative regardless of `ready`; the misleading success-only field is no longer emitted | **VERIFIED** |
| IAP-RQ-320 / IAP-RQ-322 | Refuse capture/launch unless the supplied ament package closure is complete and correctly shadowed | `run_launch_dependency_preflight()` records ordered prefixes, nine required package resolutions, path existence/index membership and expected task-local IAP/EGO identity; `main()` returns distinct exit 4 before capture/launch on failure | **IMPLEMENTED / runner 21/21** |
| IAP-RQ-320 / IAP-RQ-322 | Prove a reproducible environment without running live flow | Read-only resolution after sourcing ROS Jazzy/workspace setup and prepending retained ICRA-024 planner prefixes resolves IAP/EGO task-locally, `so3_control` from its isolated workspace prefix and `rclcpp_components` from ROS Jazzy; exact recipe/results are in `results/icra27/icra025/verification_summary.txt` | **VERIFIED / no ROS launch** |
| IAP-RQ-320 / IAP-RQ-322 | Preserve accepted binaries, evidence and disabled/live boundaries | Retained ICRA-024 regressions/linkage pass at exact `libiap.so` and `libplan_env.so` hashes; protected evidence and ICRA-024 committed run artifacts remain unchanged | **VERIFIED / Gate-0B NOT_QUALIFIED** |

No new build/install tree, GPU preflight, capture subscription, ROS daemon or
graph query, launch, simulator, smoke, formal live analyzer, benchmark,
qualification, P4/P5 execution or Gate decision occurred. The preflight
validates and reports the supplied environment; it never silently repairs it.

## 2026-08-22 ICRA-024 Gate-0B successful-generation sample freeze

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Callback identity must be steady-clock-only, fail closed when malformed and retain the final captured representative | `gate0_analyzer.py::analyze_p0_messages()` accepts only finite `refresh_callback_end_steady_s`, preserves it in P0 rows, retains the last captured duplicate callback, and reports captured/representative/duplicate/malformed counts; focused tests prove a valid `refresh_stamp_s` is never a fallback | **IMPLEMENTED / fail-closed** |
| IAP-RQ-320 / IAP-RQ-322 | Formal latency must contain exactly one final contract-complete representative per successful generation | Strict success requires boolean `ready`, positive non-boolean integral `generation_id`, `reason=ok`, clean available snapshot and every existing source/counter/timing/workload contract; generation duplicates use the last captured candidate and remain visible in summary counts; integration coverage proves missing integrity/manifest evidence cannot downgrade an already proven evidence-contract failure | **IMPLEMENTED / analyzer 31/31** |
| IAP-RQ-320 / IAP-RQ-322 | Distribution selection must be frozen independently of class and observed latency | Focused fixtures include cold, rolling-shift, retained/entered, full-rebuild, exact-reuse, TTL-reuse, warm and slow rows in one distribution; a 20-row fixture proves no tail/outlier trimming and exact complete-set type-7 p50/p95/max | **FROZEN before live output** |
| IAP-RQ-320 / IAP-RQ-322 | Preserve fixed Gate-0B protocol thresholds and failure attribution | Failed callback representatives remain in failed/stale ratios and outside latency percentiles; smoke minimum remains 1 with no performance threshold, benchmark minimum remains 20 with one p95 `<=400 ms` threshold, and fixed worker count remains four | **VERIFIED / Gate-0B NOT_QUALIFIED** |
| IAP-RQ-320 / IAP-RQ-322 | Execute the replacement protocol once after repository-local verification and mandatory preflight | Required build/test/linkage checks pass and preflight records `gpu_ready=true`, `cuInit_result=0`, `device_count=1`; the sole smoke then fails before IAP startup because its isolated prefix search cannot find workspace package `so3_control`, and the sole analyzer truthfully records zero observations and `P0_INPUT_AVAILABILITY_FAIL` | **BLOCKED / no retry** |

No P0 product, runner/capture, workload, threshold, P4/P5 or Gate decision
changed. The bounded command, manifest, preflight, empty raw captures, analyzer
output and verification summary are retained under `results/icra27/icra024/`.
This launch-environment failure occurred before `iap_rosnode` started and is
not a P0 product/scientific result. The one-shot rule prohibited correction or
retry; **Gate-0B remains NOT_QUALIFIED** pending Supervisor review.

## 2026-08-22 ICRA-023 review ownership and historical provenance repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Historical ICRA-020 evidence must bind to its immutable recorded implementation, not demand that a later tree remain unchanged | `validate_recorded_commit_provenance()` requires the exact recorded commit object and a blob at each required `implementation_sha:path`; the current ICRA-022 tree is intentionally allowed to evolve independently | **IMPLEMENTED / validator 5/5; selected root 8/8** |
| IAP-RQ-320 / IAP-RQ-322 | Provenance failure must remain fail-closed without weakening the canonical artifact | Focused tests reject a nonexistent 40-hex commit and a missing recorded path; the accepted JSON SHA-256 is frozen before parse; schema/workload/science/counters/timing/percentiles/commands/build provenance, existing-file hashes and no-promotion assertions are unchanged | **VERIFIED / canonical ICRA-020 SHA-256 unchanged** |
| IAP-RQ-320 / IAP-RQ-322 | Builder result reporting must not impersonate final Supervisor review or rewrite pushed history | ICRA-022 role wording is corrected to Builder self-check; the issued-spec conflict and RQ-less `2bd5ba4` are acknowledged without amend/rebase/force-push; every ICRA-023 commit is required to carry an applicable RQ ID | **DOCUMENTED / SUPERVISOR review required** |
| IAP-RQ-320 / IAP-RQ-322 | Provenance-only repair must preserve all accepted product binaries and regressions | Retained artifacts pass plan-env 6/6, P0 76/76, Adapter 7/7, rolling 23/23, Ego 8/8, P4 4/4 and P1 39/39; analyzer/runner/capture pass 25/25, 16/16, 1/1; direct linkage and recorded library hashes remain exact | **VERIFIED / Gate-0B NOT_QUALIFIED pending Supervisor decision** |

Exact commands, exits, counts, linkage and protected hashes are recorded in
`results/icra27/icra023/verification_summary.txt`. No product/build/live-flow
work or disabled diagnostic was performed.

## 2026-08-22 ICRA-022 occupancy-epoch timestamp authority repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-320 / IAP-RQ-322 | A depth-fused occupancy generation must use the exact scientific image-header time, never node receipt/watchdog time | Both real depth callbacks validate finite positive sec/nanosec and bind source time with pending image/pose under `occupancy_epoch_mutex_`; the shared private commit helper keeps receipt time only for the watchdog and atomically publishes buffers/generation/source stamp. Tests prove `100.25 s`/`101.75 s` epochs with host receipt around `10000 s`, immutable prior generations, and point-cloud input-header semantics | **IMPLEMENTED / FOCUSED TESTS PASS 6/6** |
| IAP-RQ-311 / IAP-RQ-320 | Invalid pending time must fail closed without publishing partial or restamped content; P0 must keep strict clock-domain freshness | Invalid pending time exits before sequence/buffer/stamp mutation; the test compares generation, stamp, centers, and all 64 frozen diagnostic cells. P0 message-domain test accepts equal current/occupancy time and rejects future `+0.001 s` and stale `-0.501 s` as `occupancy_stale` | **IMPLEMENTED / P0 76/76; Adapter 7/7** |
| IAP-RQ-320 / IAP-RQ-322 | Analyzer diagnostics must distinguish availability, evidence-contract, and complete-benchmark performance failures | Protocol-neutral insufficient-count name; zero success input failure; malformed/incoherent/insufficient evidence-contract failure; tuning and `P0_PERFORMANCE_GATE_FAIL` only for a complete benchmark over `400 ms`; smoke has neither threshold nor advice | **IMPLEMENTED / analyzer 25/25; runner 16/16; capture 1/1** |
| IAP-RQ-311 / IAP-RQ-320 / IAP-RQ-322 | Repository-local verification must retain historical evidence and stop before any live run | IAP/planner builds and all non-conflicting required suites pass; linkage resolves local `libiap.so`/`libplan_env.so`; protected PDF and ICRA-011/014/020/021 hashes remain exact. The historical ICRA-020 validator alone fails because its zero-diff pin includes the P0 test file that ICRA-022 requires changing; validator is not allowlisted | **BLOCKED / Gate-0B NOT_QUALIFIED; SUPERVISOR disposition required** |

Exact commands, exit codes, counts, linkage, binary hashes and separately
identified plan-env package lint debt are recorded in
`results/icra27/icra022/verification_summary.txt`. No GPU preflight, ROS/main
flow, live analyzer, smoke, qualification, campaign, disabled ICRA-014
diagnostic, or ICRA-020 opt-in profile was invoked.

## 2026-08-22 ICRA-021 four-worker Gate-0B smoke

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | The post-refactor Gate-0B pair must use one preselected worker count without changing product defaults | `p0_effective_config()` requests exactly four for both `20/15 s` smoke and future `60/55 s` benchmark; runner, run manifest, runtime manifest, and every successful health row require requested/effective `(4,4)`; tests reject one and every sampled non-four value | **IMPLEMENTED / VERIFIED: global launch/runtime default remains one** |
| IAP-RQ-320 / IAP-RQ-322 | Current rolling production evidence must not be dropped or accepted when malformed/incoherent | `gate0_analyzer.py` CSV retains all exact rolling work/provenance counters, invalidation reason, source readiness/failure fields and refresh/provider/generation timing; per-success strict integer/nonnegative/range checks, all seven production identities, exact `ok` health reason, source seen/valid/fresh booleans, positive finite source stamps and clean snapshot state fail closed | **IMPLEMENTED / VERIFIED: analyzer 22/22, runner 16/16, capture 1/1** |
| IAP-RQ-320 / IAP-RQ-322 | Smoke is availability/lifecycle evidence and must not apply the later formal latency threshold | smoke requires at least one successful generation and one finite integrity report but ignores `400 ms`; benchmark retains at least 20 generations and R-7 p95 `<=400 ms`; zero generation/integrity, process death and capture-readiness failures remain nonzero | **IMPLEMENTED / VERIFIED: focused contracts green** |
| IAP-RQ-320 / IAP-RQ-322 | The mandatory one-shot live evidence must stop on failure without tuning or qualification | GPU preflight PASS: RTX 4070 Ti SUPER, driver `580.126.09`, `cuInit=0`, `device_count=1`; exactly one 20-second smoke had capture ready first, required `iap_rosnode` observed/alive through runtime, controlled shutdown and runner exit 0, but 24/24 P0 rows were unsuccessful (`occupancy_stale` 22, `message_stamp_unavailable` 2) despite 210/210 finite integrity rows; analyzer exit 1 | **BLOCKED / Gate-0B NOT_QUALIFIED: no retry or 60-second run** |

Canonical bounded evidence hashes: GPU preflight
`4bfda37b2a4d917e37e8f7b22161a97333329c56c5ce904c19d670239bdf9b8d`;
run manifest
`429633aa4818832461cdd852f31a9b128894220663e7e73162a1d9954c180ac0`;
raw health
`59f88a7eb9cde2695aad20aef7e6f32c4f065e1caf0bf127907f2a814b40ee59`;
raw integrity
`b82089044a2088d02b0e44c9a3a2eebd2e43168d559578046afe989352052aca`;
runtime manifest
`30cd0c2fe7d1731ac15d06a46219a8d27573b93cb4bb735cf90e48d9c859df02`;
analyzer result
`ad4d489fada54978c089c75a8638ce096ea48367c954b4f635dcadf12c693dc3`,
summary `87a8a946e4c07b8f26a86315bf6d6381d20b15fc4c63569ee0e280325c9cf98a`,
and CSV `d763d22b0ae1e9eca6fd19ab30cbcad7bbc831f43886d9432037941cb3705446`.
ICRA-011, disabled ICRA-014, accepted ICRA-020 and the protected untracked PDF
remain exact and were not regenerated or modified.

## 2026-08-22 ICRA-020 Stage-5 rolling P0 worker-scaling diagnostic

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-310 / IAP-RQ-311 / IAP-RQ-312 / IAP-RQ-314 | Measure the accepted production P0 runtime path without adding a public test Interface or timing simplified science | Explicitly disabled `P0RiskGridRuntimeStampTest.DISABLED_ICRA020_ProductionRuntimeWorkerScalingProfile`; exact opt-in filter/output/provenance requirements; wall interval covers only the real synchronous `refreshOnceForTest()` call while fresh replay, hashing and serialization stay untimed | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: test/evidence-only** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Freeze the complete worker/scenario workload and prove exact semantic work | workers 1/2/4; `40 x 40 x 8 x 6`; Fusion/required GNSS; 31 satellites, 704 occupancy voxels/FIM primitives and 23,309 map points; cold `12800`, stationary `0`, `+1 x` `320` and nonempty-delta `12800` spatial recomputes with exact retained/entered/evicted/source/fusion/invalidation contracts | **IMPLEMENTED / PROFILE EVIDENCE PENDING: two warmups plus ten measured samples per matrix cell** |
| IAP-RQ-320 / IAP-RQ-322 | Every measured result must be scientifically equal to a fresh complete rebuild and fail closed before PASS serialization | fresh deterministic runtime and untimed accepted base per sample; generation/stamp-independent full snapshot hash; stable source/content versions and current occupancy diagnostic checks; atomic artifact rename only after the complete matrix, finite timing, exact counters and stable cross-worker hashes validate | **IMPLEMENTED / PROFILE EVIDENCE PENDING: failed or incomplete execution writes no PASS artifact** |
| IAP-RQ-310 / IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Canonical cost-ranking evidence must be reproducible without promoting a Gate or production decision | fail-closed `p0_rolling_stage5_profile_v1` validator checks implementation/source and executable/library SHA-256, raw sample-derived R-7 summaries, exact 3 x 4 matrix and prohibited promotion claims; artifact labels latency diagnostic-only, Gate not run, worker not selected, reverse-ray pending and GPU not evaluated | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: no 400 ms verdict, qualification or production selection** |

ICRA-020 is a synthetic Stage-5 cost-ranking diagnostic only. It does not
select a worker count, authorize reverse-ray or GPU work, qualify P0/Gate-0B,
or run main flow, ROS launch, smoke, analyzer, formal benchmark, bag, RViz or
campaign work.

## 2026-08-22 ICRA-019 Phase-4B1 immutable occupancy delta

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-311 / IAP-RQ-312 / IAP-RQ-314 | The complete frozen raw-occupancy capture must have one deterministic fixed-lattice identity and an exact net delta | `P0RawOccupancyIdentity` and `P0RawOccupancyDelta` behind `P0OccupancyEpochAdapter`; finite centre alignment, mathematical floor, integer range, one-key-per-centre, sorted uniqueness, exact source/frame/origin/resolution/generation comparison, linear added/removed set difference and changed bounds; Adapter 7/7 covers reorder, negative keys, added/removed/mixed, skipped generations and every unavailable-proof case | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: no second map, journal, callback or GridMap change** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-321 | Authoritative occupancy source version and raw LOS content identity must remain distinct | P0 retains the last successfully committed normalized base/source/canonical owner/content identity; same-producer newer-generation empty delta reuses only the canonical LOS owner while current diagnostic query, occupancy generation/stamp, horizon growth/fusion and materialization remain current; rolling provenance validates nonzero content identity and same-generation contradictions | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: empty delta yields 0 GNSS/LiDAR spatial recomputes, 27 retained positions and 54 current horizon fusions** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Nonempty or unprovable occupancy change must remain conservative and transaction rollback-safe | added-only, removed-only and mixed production scenarios each produce `occupancy_source_changed`, rebuild all 27 active-GNSS positions and match a fresh complete RiskGrid; changed producer cannot reuse; same-version contradiction/regression fail closed; occupancy/prior/GNSS/LiDAR races after delta abort publication and retry from the unchanged committed base | **VERIFIED / SUPERVISOR REVIEW PENDING: no reverse-ray dependency or partial dirty-ray recomputation** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Phase-4B1 must preserve inactive-source isolation and all retained consumers | LidarOnly/GNSS-disabled/Optional/Auto, TTL/watchdog, worker 1/2/4, boundary movement and fresh-equivalence regressions remain green; repository-local root 7/7, plan-env 1/1, Ego 8/8, P4 4/4 and P1 integrity 39/39 pass; 14 direct consumers resolve current ICRA-019 `libiap.so`; ICRA-011 profile remains read-only | **VERIFIED / SUPERVISOR REVIEW PENDING: P0/Gate-0B remain unqualified** |

ICRA-019 is only Phase-4B1 empty-delta LOS reuse. It does not implement
reverse-ray/dirty-ray propagation, profile or tune CPU workers, develop or
assess GPU code, select production policy values, qualify P0/Gate-0B, or run
main flow, ROS launch, smoke, analyzer, benchmark, bag/RViz/campaign work.

## 2026-08-22 ICRA-018 absent-GNSS generation race repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-322 | Active GNSS absence is a versioned transaction state, including explicit-absent nonzero and never-seen zero generations | `p0_risk_grid_runtime.cpp` derives validation solely from `predictorSpatialSourceUsage().gnss` and compares captured/live generation exactly at both existing RiskGrid source checks without requiring `snapshot.has_epoch`; stable `0 == 0` proceeds, while every callback-created mismatch returns `PREDICTOR_SPATIAL_SOURCE_CHANGED` | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: no new version, callback, lock, timer, cache or Interface** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Optional/Auto callback races must roll back immutable RiskGrid and transactional rolling/watchdog state | production P0 regressions cover Optional explicit-absent to valid callback, Auto explicit-absent to invalid callback, and zero-to-nonzero first callbacks; exact ordered snapshot comparison, zero committed rolling diagnostics, same-version retries proving 27 retained slots/54 horizon fusions, and boundary retries proving the aborted candidate did not advance the successful-full-refresh watchdog epoch | **VERIFIED / SUPERVISOR REVIEW PENDING: callback work cannot publish an obsolete absent candidate** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 | Inactive GNSS configurations must not acquire a false transaction dependency | LidarOnly/Auto valid-callback and Fusion/GNSS-disabled invalid-callback regressions publish normally with generation changes, 27 retained positions, 54 spatial reuses and 54 horizon fusions; Required missing typed failure and valid-to-invalid races remain green | **VERIFIED / SUPERVISOR REVIEW PENDING: authoritative active-source selection preserved** |
| IAP-RQ-320 / IAP-RQ-322 | The review repair must preserve retained Phase-4A science and downstream behavior | repository-local focused 7/7, P0 70/70, root 7/7, plan-env 1/1, Ego 8/8, P4 4/4 and P1 integrity 39/39; directly linked consumers resolve the current ICRA-018 library; retained ICRA-011 profile remains read-only | **VERIFIED / SUPERVISOR REVIEW PENDING: no Phase-4B, calibration, activation or qualification** |

ICRA-018 closes only the absent-GNSS generation race. It does not accept
ICRA-017/018, qualify Phase 4 or Gate-0B, implement Phase-4B, choose production
policy values, or authorize main flow, ROS launch, smoke, qualification,
analyzer, benchmark, bag/RViz/campaign or GPU work.

## 2026-08-22 ICRA-017 Phase-4A provenance review repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-322 | Every non-null GNSS range callback is one observable valid-or-absent source update | parse from a coherent dependency copy; one final `health_state_mutex_` publication advances the nonzero generation, updates seen/stamp/count and installs or clears `latest_epoch_`; no-origin, empty conversion, all-filtered, missing-ephemeris, null no-op, in-flight abort and recovery regressions | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: stale epochs cannot survive an invalid callback or evade the end validator** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 | Occupancy authority is a stable producer owner plus exact generation, while each candidate retains one immutable LOS materialization | `P0OccupancyEpoch` stable token/live-owner/live-generation seam; Adapter validation; `GridMap` manager wiring; P0 start/end owner-generation checks and token-bound canonical LOS reuse; factory/probe-count, token replacement/expiry and coincident-generation regressions | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: sampled recapture, observation equivalence and direct Visibility replay removed** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Pre-candidate active-source rejection must reach P0 as typed current-attempt evidence without accepted work | retained rolling attempt diagnostic; production provider detailed begin reason; P0 pre-batch failure path; missing/zero/nonfinite current/GNSS/LiDAR, Required epoch and invalid-satellite regressions; previous generation/slots/watchdog retained | **VERIFIED / SUPERVISOR REVIEW PENDING: count `1`, reason `source_provenance_invalid`, all accepted-work counters zero** |
| IAP-RQ-320 / IAP-RQ-322 | The narrow repair must preserve Phase-4A TTL/watchdog/science and every retained consumer | repository-local root, plan-env, P0/Adapter, P1/P2/P3/planning-context/P4/P5 suites; read-only ICRA-011 profile; exact current-`libiap.so` linkage | **VERIFIED / SUPERVISOR REVIEW PENDING: no Phase-4B, activation, calibration or qualification** |

ICRA-017 repairs only the three ICRA-016 review findings. It does not qualify
ICRA-016/017, Phase 4 or Gate-0B; implement occupancy delta/reverse-ray; select
TTL/watchdog values; or run main flow, ROS launch, smoke, qualification,
analyzer, benchmark, bag/RViz/campaign or GPU work.

## 2026-08-21 ICRA-016 phase-4A versioned provenance and bounded retention

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-322 | Every active spatial source must have coherent, monotonic provenance and publication-race validation | authoritative `predictorSpatialSourceUsage`; rolling provenance record; atomic GNSS/current capture; LiDAR generation/stamp/owner acceptance and clearing under one mutex; occupancy/GNSS/current/LiDAR end validation; rolling-owned canonical/start/live occupancy visibility equivalence over every touched slot and its original epoch; contradictory/regressed/source-race and same-generation canopy-ray owner tests | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: active races roll back RiskGrid and rolling candidates; stable missing LiDAR remains conservative and non-reusable** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-321 | Only defined continuous spatial fields may retain advice for a bounded per-slot age without restamping | default-disabled GNSS and legacy-current TTL policies; slot-local source snapshot/provenance; discrete satellite/trunk invalidation; original GNSS stamp substituted for retained-advisory freshness; deterministic retention, expiry, entering-position and fresh-equivalence tests | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: no complete-result or partial-component cache; per-horizon validation/growth/fusion/materialization preserved** |
| IAP-RQ-320 / IAP-RQ-322 | Periodic full rebuild must be based on the last successfully committed full refresh and remain rollback-safe | default-disabled successful-full-refresh watchdog; commit-only epoch advance; stationary threshold, aborted race, same-time retry and post-commit reuse tests in rolling and production P0 | **IMPLEMENTED / SUPERVISOR REVIEW PENDING: failed candidates do not postpone the next forced rebuild** |
| IAP-RQ-320 / IAP-RQ-322 | Health evidence must separate exact/TTL retention, expiry, watchdog and invalid provenance without redefining legacy fields | additive typed rolling diagnostics and deterministic P0 JSON fields; aborted-candidate diagnostic clearing; retained root/profile/P0/P1–P5 suites and current-library linkage evidence | **VERIFIED / SUPERVISOR REVIEW PENDING: policies remain `NaN`/disabled; no calibrated or qualification claim** |

ICRA-016 is Phase-4A only. It does not implement occupancy delta/reverse-ray,
partial component caching, calibration, or production activation, and it does
not qualify Gate-0B. No main flow, ROS launch, smoke, qualification, analyzer,
benchmark, bag/RViz/campaign or GPU preflight ran.

## 2026-08-21 ICRA-013 phase-3A fixed world lattice

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Window geometry must use one anchor-relative integer world lattice with frozen even-side and negative-floor rules | `RiskGridMapParams::lattice_anchor_w`; internal integer world/lower keys in `risk_grid_map.cpp`; default/even/negative, same-key, one-cell, multi-axis and non-zero-anchor regressions in `test_risk_grid_map.cpp` | **IMPLEMENTED / REVIEW PENDING: bitwise-stable same-key origins/query positions and exact resolution-multiple crossings** |
| IAP-RQ-320 / IAP-RQ-322 | Proposed geometry and complete voxel data must publish atomically, while every failed shifted refresh retains the previous coherent generation | local proposed origin; serialized refresh writers; configuration epoch recheck in the success lock; mutex-protected value-return `origin()`; shifted provider, occupancy/prior and deterministic configure/concurrent-refresh regressions compare generation, origin and every ordered voxel | **IMPLEMENTED / REVIEW PENDING: failed/stale refresh never exposes proposed geometry; concurrent refresh IDs remain unique** |
| IAP-RQ-320 | Fixed lattice must not silently become a delta/reuse optimization or change scientific query semantics | snapped-origin full-order/scalar-affine regression; complete root, Predictor, local occupancy, frozen epoch, adapters and P1–P5/P0 linked-consumer suites; runtime linkage log | **VERIFIED / REVIEW PENDING: 286/286 PASS against current repository-local ICRA-013 `libiap.so`; full non-occupied provider dispatch retained** |

ICRA-013 is fixed-lattice geometry and atomic publication only. It does not add
ring storage, entering-slab dispatch, cross-refresh cache/version/TTL logic,
partial publication, restamping or performance savings, and it does not qualify
Gate-0B. No main flow, ROS launch, smoke, qualification, analyzer, benchmark,
GPU preflight, calibration or P1/P2/P3/P4/P5 behavior change ran.

## 2026-08-21 ICRA-012 phase-2 legacy diagnostic review repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Preserve legacy LiDAR-cache diagnostics while retaining generalized phase-2 spatial reuse | `PredictorModule::queryBatch()` counts legacy positions/evaluations only on successful LiDAR-capable cache population and legacy hits on coherent lookup; source-mode, non-cacheable and early-invalid ordering regressions | **IMPLEMENTED / REVIEW PENDING: GNSS-only legacy `0/0/0`; Fusion/LidarOnly semantics retained; actual invocation and generalized reuse remain separate** |
| IAP-RQ-320 / IAP-RQ-322 | Production GNSS-only workers must expose zero legacy LiDAR counters without losing deterministic phase-2 evidence | strengthened `MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent`; health snapshot assertions; current-library linkage log | **VERIFIED / REVIEW PENDING: workers 1/2/4 require legacy `0/0/0`, nonzero spatial/GNSS/fusion counts and equivalent science; six retained suites pass 139/139** |
| IAP-RQ-320 | Phase-2 reproduction commands must appear in the repository Definition-of-Done location | existing ICRA-011 `docs/CHANGES.md` entry now contains exact Predictor, production-runtime, offline-profile and Python-contract commands | **DOCUMENTED / REVIEW PENDING; retained JSON is read-only and Gate qualification remains `NOT_RUN`** |

ICRA-012 is a bounded review repair. It does not qualify ICRA-011/012 or Gate-0B,
implement phase 3, select production calibration, or authorize main flow, smoke,
qualification, analyzer, benchmark, GPU preflight or P1/P2/P3/P4/P5 work.

## 2026-08-21 ICRA-011 P0 phase-2 within-refresh spatial advisory deduplication

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-321 | Deduplicate only coherent spatial GNSS/LiDAR evidence while preserving scalar-equivalent per-horizon growth and fusion | private call-local `PredictorModule::SpatialAdvisory`; exact source-identity key; `BatchReusesSpatialAdvisoryAndRebuildsEveryHorizonRisk`, source/early-failure isolation and effective-freshness regressions | **PASS: 2/10 recompute/reuse for 2 x 6; GNSS/LiDAR 2/2; fusion 12; full result equivalence including typed/nonfinite state** |
| IAP-RQ-320 / IAP-RQ-322 | Production health must expose actual layer work for the current refresh attempt without changing logical/result-used evidence | additive Predictor diagnostics; worker aggregation; five exact P0 health JSON fields; real production-provider count/reset regression and worker 1/2/4 equivalence | **PASS: deterministic recompute/reuse/invocation/fusion counts; logical GNSS-used remains distinct; next early failure resets new fields to zero** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Canonical phase-2 workload and science must be reproducible offline without claiming qualification or calibration | `p0_phase2_spatial_dedup_profile.json`; fail-closed Python contract; synthetic finite growth constant and immutable map-LOS input; R-7 diagnostic summaries | **PASS DIAGNOSTIC: 76,800 logical/provider/conversion/fusion, 12,800 spatial/GNSS/LiDAR recompute, 64,000 reuse, zero scalar mismatch, stable workers 1/2/4 checksums; Gate `NOT_RUN`** |
| IAP-RQ-320 / IAP-RQ-322 | Phase-2 change must retain existing P0 science, fail-closed publication and focused coverage | six repository-local suites against the current `libiap.so` | **PASS: 137/137; no cross-refresh/phase-3 behavior, runtime qualification or production calibration** |

ICRA-011's core phase-2 seam and scientific/profile evidence are accepted, but phase 2 remains
review-pending until the bounded ICRA-012 legacy-counter and reproduction-document repair is
verified by Supervisor. Neither task qualifies Gate-0B, selects production `sigma_grow`,
implements the phase-3 lattice/ring/cross-refresh window, or authorizes main flow, smoke,
qualification, analyzer, benchmark, GPU preflight or P1/P2/P3/P4/P5 work.

## 2026-08-21 ICRA-010 positive-horizon typed-status repair

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-321 | A positive-horizon result may report covariance growth `APPLIED` only after the helper actually applies growth | `CovarianceGrowthStatus::NOT_EVALUATED` is the result default; the pre-validation speculative assignment is removed; exact Predictor regression covers four early failures plus applied/tau-zero/invalid controls | **PASS: early failures are `NOT_EVALUATED`; helper-reached positive is `APPLIED`; helper-reached tau zero is `NOT_REQUIRED_TAU_ZERO`; invalid horizon remains typed** |
| IAP-RQ-320 / IAP-RQ-322 | Any required non-`APPLIED` positive-horizon provider result must reject the complete refresh without replacing active data | real `P0RiskGridRuntime` production-provider regression induces stale required GNSS only after one accepted refresh and compares active snapshot identity, generation and ordered voxel data | **PASS: failed refresh reports `provider_refresh_failed` and preserves the previous immutable generation/data** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | The narrow repair must preserve established P0 phase-1 behavior | six repository-local affected suites | **PASS: 134/134; reason strings, algebra, configuration, counters, source logic and scientific outputs unchanged** |

ICRA-010 is a typed-status correctness repair only. It does not qualify Gate-0B,
select production growth calibration, implement rolling/delta/reuse, or run main flow, smoke,
qualification, analyzer, profile, benchmark, GPU preflight or P1/P2/P3/P4/P5 work.

## 2026-08-21 ICRA-009 P0 phase-1 semantic implementation

| Req ID | Requirement/evidence seam | Implementation and focused evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 | Production GNSS LOS and occupied-skip diagnostics must consume the same complete immutable map epoch | `GridMap::captureFrozenOccupancyEpoch`; `P0OccupancyEpochAdapter`; manager/runtime binding; nonzero-lattice, raw/fused/inflated-only, exact-capacity and production LOS regressions | **IMPLEMENTED PHASE 1; complete raw-cloud + fused-depth centres, inflated-only excluded; no open-sky fallback on invalid production epoch** |
| IAP-RQ-320 / IAP-RQ-321 | Positive horizons must grow the current finite SPD prior while tau zero preserves the accepted path | `EmpiricalCovarianceGrowthParams`, `CovarianceGrowthStatus`, internal Predictor propagation before advisory fusion; tau-zero, six-horizon, PSD/monotonic and invalid-input regressions | **IMPLEMENTED PHASE 1; `sigma_grow_m_sqrt_s` default remains invalid NaN; production calibration and qualification pending** |
| IAP-RQ-320 / IAP-RQ-322 | Occupancy and integrity-prior publication races must not publish mixed generations | immutable epoch live probe, `prior_source_generation`, `RiskGridSourceValidation`, start/end validation and fail-closed active-snapshot retention tests | **IMPLEMENTED PHASE 1; exact reasons include occupancy snapshot/adapter/frame/stale/generation and missing/stale/invalid prior states** |
| IAP-RQ-320 / IAP-RQ-322 | Worker parallelism must preserve ordered scientific output and existing logical evidence semantics | `MapLosAndGrowthWorkersOneTwoFourAreScientificallyEquivalent`; complete P0 runtime and root focused suites | **PASS: workers 1/2/4 equivalent within absolute 1e-12; 132 affected tests pass; 76,800 logical-shape semantics unchanged** |

ICRA-009 implements only the frozen phase-1 semantic seams. Rolling/delta/reuse,
performance qualification, production growth calibration and Gate-0B remain pending. No main
flow, smoke, qualification, benchmark, GPU preflight, analyzer or P1/P2/P3/P4/P5 behavior was
run or changed.

## 2026-08-21 ICRA P0 rolling-window design freeze

| Req ID | Requirement/evidence seam | Frozen design / next evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 | Active production P0 must bind map-based GNSS visibility and canopy-aware effective information | `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md`; ICRA-008 concrete Seam audit; later focused production tests | **PREDICTOR CAPABILITY EXISTS; ACTIVE P0 BINDING MISSING** |
| IAP-RQ-320 / IAP-RQ-321 | Horizon risk must propagate covariance/PL rather than reuse an invariant whole result | frozen `SpatialAdvisory` versus `HorizonRisk` separation; later monotonic/equivalence/fail-closed tests | **ACTIVE P0 MISSING SIGMA GROWTH** |
| IAP-RQ-322 | Fixed lattice, rolling local window, version/TTL invalidation and coherent immutable publication | design sections 3–7; staged implementation and forced-full equivalence tests | **PLANNED / NOT_IMPLEMENTED** |
| IAP-RQ-320 / IAP-RQ-322 | Gate evidence must preserve 76,800 logical voxels while separating actual recompute/reuse/invocation counts | design section 10; active scope and implementation-plan addenda | **CONTRACT FROZEN; EVIDENCE SCHEMA NOT_IMPLEMENTED** |

This design freeze changes no product source, threshold, ROI, horizon, worker configuration,
runtime result or historical evidence. ICRA-008 is a bounded implementation-readiness audit;
it cannot mark any planned row implemented or start P4.

## 2026-08-21 ICRA-007 P0 profile fidelity repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | Current P0 runtime and standards-required map LOS must be measured as different modes | `iap_predictor_offline_profile.cpp`; `results/icra27/icra007/p0_provider_profile.json`; `test_icra007_provider_profile.py` | **PASS diagnostic contract: `frozen_runtime=CURRENT_PRODUCTION` without GNSS occupancy; `map_los_candidate=NOT_CURRENT_PRODUCTION` with the sole 704-point occupancy difference** |
| IAP-RQ-320 | Offline result materialization must match production and resist field-mapping drift | `predictor_risk_conversion.hpp`; production `p0_risk_grid_runtime.cpp`; profiler; `test_predictor_risk_conversion.cpp` | **SHARED PURE 7-FIELD MAPPING; 2/2 focused conversion tests and 40/40 production P0 tests PASS; runtime behavior unchanged** |
| IAP-RQ-320 | Budget timing must exclude component clocks and profiler-only science capture, then quantify timer perturbation | Per-cell counter-only/component-timed raw iterations, type-7 summaries, post-timer real science replay and perturbation in `p0_provider_profile.json` | **PASS: all timer/count contracts exact; worker-1 perturbation frozen `+0.902882 ms/+0.156365%`, map LOS `+4.585693 ms/+0.391823%`; both `COST_RANKING_DIAGNOSTIC`** |
| IAP-RQ-320 | Horizon invariance observation must not be reported as scientific conformance | 91-field checksum/equivalence contract; existing frozen-horizon and freshness tests; profile semantic fields | **`MISSING_SIGMA_GROWTH`; standards conformance BLOCKED; whole-result cross-horizon reuse prohibited; no covariance-growth implementation** |
| IAP-RQ-320 | Worker 1/2/4 rankings must remain reproducible per mode | Counter-only evidence and `test_icra007_provider_profile.py` | **Frozen p95 `577.930797/300.252013/155.991150 ms`; map LOS p95 `1172.415454/606.576794/311.890300 ms`; checksums/counts stable; root CTest 30/30 PASS** |

ICRA-007 is a repository-local offline diagnostic repair only. It does not qualify Gate-0B, select a CPU remediation, repair production map LOS or horizon propagation, change the 400 ms limit, or authorize any smoke/qualification/P4/P5 work. The retained ICRA-005 `P0_PERFORMANCE_GATE_FAIL` remains authoritative.

## 2026-08-21 ICRA-006 offline P0 provider diagnosis

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | Retained ICRA-005 performance failure must reproduce without ROS or retained-output mutation | Current `gate0_analyzer.py` over committed ICRA-005 benchmark input; new `results/icra27/icra006/red_replay/` | **REPRODUCED: exit 1, sole `refresh_p95_over_400_ms`, 72 generations, p95 657.21388795 ms** |
| IAP-RQ-320 | Offline diagnostic must execute the production-shaped 12,800-position x six-horizon Predictor workload with map-based GNSS LOS and expose disjoint/labelled cost evidence | `iap_predictor_offline_profile.cpp`; 704-point `LocalOccupancyGrid`; opt-in timings and additive counters in `predictor_module.hpp/.cpp`; `p0_provider_profile.json` | **PASS: 76,800 logical/actually dispatched queries; GNSS/fusion 76,800 calls, LiDAR 12,800 evaluations + 64,000 cache hits; all timings finite** |
| IAP-RQ-320 | Horizon reuse semantics must be decided by tests with freshness behavior and preserved scalar/batch equivalence | `FrozenSnapshotScientificFieldsAreInvariantAcrossSixHorizons`, `FreshnessReferenceNotFutureQueryTimeControlsSixHorizonValidity`, and the six-horizon batch/scalar regression in `test_predictor_module.cpp`; exact 91-name whitelist in profile JSON | **HORIZON-INVARIANT under fixed P0 snapshot reference; all named scientific fields equal, metadata whitelist explicit; 37/37 tests PASS** |
| IAP-RQ-320 | Worker 1/2/4 diagnostics must preserve scientific results and report stable checksums/counts plus p50/p95/speedup | `p0_provider_profile.json`; `test_icra006_provider_profile.py` | **PASS: checksum `bc296383f5cb17cf`; p50 speedup 1.0/1.8957987/3.4966878; provider p95 1193.7742/629.9757/341.2319 ms; zero failed/non-finite iterations** |

ICRA-006 is diagnostic-only. It does not change the formal worker count, Gate threshold, launch/profile, Predictor scientific results or caching behavior, and it does not select a production optimization. No ROS/main-flow smoke, qualification, bag, RViz, campaign or P4/P5 work ran.

## 2026-08-21 ICRA-005 retained evidence closure and fixed P0 benchmark

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | The reviewed ICRA-004 smoke must retain the exact runtime manifest and analyzer-resolved configuration used for its verdict | Tracked `results/icra27/icra004/runs/smoke/exports/.../test_planner_manifest.json` (`111d57f7...f818`) and `results/icra27/icra004/runs/smoke/analyzer/effective_config.json` (`f9997494...263f`) | **RETAINED BY EXACT HASH; ICRA-004 not rerun or reconstructed** |
| IAP-RQ-320 | Smoke and benchmark integrity evidence must fail closed on zero valid captured reports, including non-finite reports marked valid | `apply_integrity_evidence_gate`, finite HPL/VPL/HAL/VAL/IM validation and shared CLI exit helper in `gate0_analyzer.py`; zero-row and invalid/non-finite benchmark tests in `test_gate0_analyzer.py` | **IMPLEMENTED; focused tests PASS** |
| IAP-RQ-320 | Fixed P0 full-grid benchmark must preserve GPU/capture/process/config/query-shape evidence and meet type-7 p95 `<=400 ms` | `results/icra27/icra005/runs/{gpu_preflight.json,benchmark/}`; 60/55 seconds, CPU mapping, fixed 30x30x6 m / 0.75 m / six horizons / 0.5 s / one worker / occupied skip; 565 valid integrity, 72 successful generations, 76,800 queries each | **P0_PERFORMANCE_GATE_FAIL: p95 657.21388795 ms > 400 ms; runner 0, analyzer 1** |

The single authorized ICRA-005 benchmark was not retried or tuned. This evidence is returned to Supervisor without changing Gate-0B, P4 or P5 status.

## 2026-08-21 ICRA-004 GPU preflight and one-shot P0 smoke

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | Every ICRA main-flow run must prove NVML and CUDA Driver API device readiness before capture or ROS startup | `run_gate0_qualification.py` structured `iap_gpu_preflight_v1`, automatic mode guard and `--gpu-preflight-only`; `test_gate0_runner.py` covers missing command, NVML nonzero, `cuInit` failure, zero devices, PASS and no launch after failure; `results/icra27/icra004/runs/gpu_preflight.json` | **PASS: both `nvidia-smi` commands exit 0; `cuInit=0`; `cuDeviceGetCount=0`; one RTX 4070 Ti SUPER** |
| IAP-RQ-320 | Capture must subscribe to the real P0 health/integrity topics with compatible QoS and be ready before launch | `gate0_capture_p0_health.py`; `test_gate0_capture_p0_health.py`; `capture_ready.json`; reliable/volatile keep-last depth 100 for `/planning/risk_grid_health` and `/iap/integrity`; runner fails before launch on absent/malformed readiness | **PASS: readiness recorded before launch; 30 health and 165 valid integrity rows captured** |
| IAP-RQ-320 | Smoke and full benchmark must retain distinct fixed validation contracts without weakening zero-record/query-shape failure | `gate0_analyzer.py`; `test_gate0_analyzer.py`; smoke is 20/15 s with at least one successful generation, full remains 60/55 s with at least 20 and p95 `<=400 ms` | **PASS for ICRA-004 smoke only; 10 successful generations, every generation exactly 76,800 queries** |
| IAP-RQ-320 | The one authorized replacement smoke must retain CPU mapping, P1/P2/P3/P4/P5 isolation, required-process lifetime and nonzero runner/analyzer exits on failure | `results/icra27/icra004/runs/smoke/{command.txt,gate0_run_manifest.json,risk_grid_health.jsonl,integrity_report.jsonl,stdout.log,analyzer}` | **SMOKE PASS: runner 0, analyzer 0, `iap_rosnode` alive through runtime; no retry and no 60-second benchmark** |

This smoke PASS authorizes Supervisor review only. Gate 0B remains unqualified until a separate task authorizes and reviews the fixed 60-second benchmark; P4/P5 qualification is unchanged.

## 2026-08-20 ICRA P0→P4→P5 scope pivot

| Req ID | Requirement/evidence seam | Planned implementation and evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Closed/open-ended collision contract and same-event guide identity/dominance | Active ICRA docs; planned collision-state tests, P4 fixture, and dual-guide evidence | **PLANNED / NOT_IMPLEMENTED** |
| IAP-RQ-423 | Selected-guide B-spline lineage and P5 final/runtime authority | Planned lineage hashes, composite-profile checks, and publish-order tests | **PLANNED / NOT_IMPLEMENTED** |

These rows register future work only. Current state is `P0 BLOCKED/UNQUALIFIED`, `P4 NOT_QUALIFIED`, and `P5 IMPLEMENTED-BUT-UNQUALIFIED`.

They do not convert historical `NO-GO-P2`, failed P0 Gate-0B evidence, the static P4 audit, or an existing P5 source connection into PASS.

## 2026-08-18 ICRA-003 Gate 0B repair and one-shot smoke evidence

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | P0 input path must report truthful live/stale source state and reject stale/future odometry/current integrity | `p0_risk_grid_runtime.h/.cpp`; `test_p0_risk_grid_runtime` covers source validity matrix, stale snapshot rejection, and non-recursive range callback | **IMPLEMENTED; smoke evidence BLOCKED / P0_INPUT_AVAILABILITY_FAIL** |
| IAP-RQ-320 | Gate 0B runner must fail closed on launch/capture/finalize/required-process failure and monitor only launch descendants | `run_gate0_qualification.py`; `test_gate0_runner.py` lifecycle tests | **IMPLEMENTED; smoke runner exit 0, analyzer exit 1** |
| IAP-RQ-320 | Analyzer must fail closed on incomplete timing/process evidence and serialize failure results with nonzero CLI status | `gate0_analyzer.py`; `test_gate0_analyzer.py`; `results/icra27/icra003/runs/smoke/analyzer` | **IMPLEMENTED; smoke analyzer exit 1** |
| IAP-RQ-320 | Mandatory one-shot CPU smoke must be preserved as evidence and must stop the sequence on failure | `results/icra27/icra003/runs/smoke/{command.txt,gate0_run_manifest.json,stdout.log,analyzer}` | **BLOCKED: 0 health/integrity captures, 0 successful generations; no retry or benchmark** |

## 2026-08-18 ICRA-002 Gate 0B input-availability and required-process evidence

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | Qualification-only CPU mapping backend must be explicit, validated, and manifest-bound | `iap_mapping_backend` launch arg; `_runtime_config`; launch/analyzer tests | **SUPERSEDED by ICRA-003 final-hash-after-override repair** |
| IAP-RQ-320 | P0 health schema must expose source readiness and exact snapshot-failure reasons | `p0_risk_grid_runtime.h/.cpp`; focused C++ tests | **SUPERSEDED by ICRA-003 live/stale/validity-matrix repair** |
| IAP-RQ-320 | Required-process evidence must be structured and analyzer-visible | runner/monitor and analyzer fields | **SUPERSEDED by ICRA-003 descendant-only/controlled-shutdown repair** |
| IAP-RQ-320 | Analyzer must fail closed on non-finite original cost, control-point gaps, and zero generations | `gate0_analyzer.py`; focused tests | **RETAINED and extended by ICRA-003** |

## 2026-08-16 ICRA Gate 0 read-only qualification

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Observe the natural rebound-optimizer-success candidate set without changing generation, optimization, ranking, refinement, feasibility, or publication | `Gate0QualificationWriter` header/source; append-only hooks in `planner_manager.cpp` and normal-publish hook in `ego_replan_fsm.cpp`; disabled/no-op, schema/full-matrix, and concurrent append gtest | **IMPLEMENTED READ-ONLY** |
| IAP-RQ-400 / IAP-RQ-422 | Prove P1/P3/P4/P2/P5 isolation and retain legacy mirror behavior while allowing geometry mirror with fanout mirror false | `test_planner.launch.py` explicit-manager override resolver; fixed runner manifests; `test_test_planner_launch.py` and `test_gate0_runner.py` | **VERIFIED; legacy fallback preserved** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | All Gate 0 attempts and complete staged control points must be grouped and hashed reproducibly | `gate0_analyzer.py`; `test_gate0_analyzer.py`; `results/icra27/gate0/candidate_qualification.csv`; `candidate_control_points.csv`; `.17g` canonical SHA256 contract | **378/378 attempts retained; every attempt singleton; NO-GO-P2** |
| IAP-RQ-320 / IAP-RQ-422 | Fixed 76,800-query P0 workload must yield at least 20 successful generations before latency is interpreted | `gate0_capture_p0_health.py`; `p0_full_grid_benchmark.csv`; `p0_full_grid_summary.json`; report section 4 | **P0_PERFORMANCE_GATE_FAIL; 0 successful generations; snapshot unavailable** |
| Operational audit (not IAP-RQ-422) | External dependency and disk state must remain auditable without modifying sources or deleting prior evidence | read-only `gnss_comm-closure` tar/list/metadata/environment/doctor archive; `gate0_disk_audit.py`; `disk_archive_candidates.csv`; `GATE0_QUALIFICATION_REPORT.md` | **ARCHIVE VERIFIED; CAMPAIGN_DISK_NO_GO; does not implement per-waypoint hinge** |

## 2026-08-09 P1-2 c36 retained result and single-epoch GNSS binding

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-020 / IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c36 (`24b9b87`) completed 10/10 runs but passed 7/10 hard gates. Three startup divergences (`82.768/182.789/63.030 m`) uniquely coincide with first-update double-epoch injection (`164` factors, 82 satellite records, false `6.2..6.6 m/s` clock drift); stable runs inject 82 factors/41 satellites. | `GnssHandlerEpochBindingTest.ConsumesOnlyNearestEpochAndRetainsLaterEpoch` red/green proves one nearest epoch per state and preservation of the next epoch. Manifest/fingerprint bind `nearest_single_epoch`; all scientific parameters remain unchanged. | **IMPLEMENTED; c36 incomplete/non-comparable; only c31/c32 count; fresh campaign required; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 deterministic stationary initialization repair

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Delay-only smoke supplied the full IMU window but `LOOSE` converged to `124.659°` alignment rotation with nonphysical velocity/bias in the stationary repeated startup scene. | `p1_fork_formal` alone materializes `NAIVE` into the run-local GLIM config; defaults are unchanged and manifest/fingerprint bind the effective mode. Unit regression preserves JSON comments. Installed 8 s smoke initialized identity pose, zero velocity/bias, and `0.007°` alignment rotation. | **IMPLEMENTED; fresh c36 required; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 c35 retained result and LiDAR initialization ordering

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c35 (`62396ac`) was incomplete: primary/mirror reference localization reached `60.589/153.721 m`, and v15 blocks broke primary two-arm support. Logs show first LiDAR at `0.733..0.933 s` before GLIM's `1.0 s` loose-init window. | v14/v15 geometry is removed, restoring v13. Formal-only LiDAR delay `2.0 s` is bound in scenario fingerprint/manifest; default is `0`. Launch test requires delay `>1.0 s`; geometry test proves no added low startup obstruction. | **IMPLEMENTED; c35 incomplete/non-comparable; only c31/c32 count; fresh campaign required; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 c34 retained result and real-FOV startup repair

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c34 (`62a7b55`) passed 9/10 hard gates; primary reference diverged at stationary startup to `95.182 m`, while the other nine localization errors were `0.130..0.366 m`. Renderer source proves v14 lateral pylons fail its `direction dot yaw >=0.5` FOV test. | Geometry v15 uses symmetric `z<=0.55 m` startup blocks inside the exact 3D FOV, outside formal lane centres and behind checkpoint. Updated red/green `StartupLocalizationBeaconsAreSymmetricAndLaneExternal` verifies count/FOV/symmetry/clearance; all geometry tests pass. | **IMPLEMENTED; c34 incomplete/non-comparable; only c31/c32 count; fresh campaign required; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 c33 retained result and startup localization repair

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c33 (`7d4537b`) passed 9/10 per-run hard gates. Both primary enabled runs chose lower and improved all risk metrics, null and soft passed, but mirror reference diverged before planning and had `53.284 m` checkpoint localization error. | c33 is retained losslessly compressed and is incomplete/non-comparable. Geometry v14 adds exact-symmetric startup pylons at `x=-11.25/-10.75 m`, `|y|=4.5 m`; `StartupLocalizationBeaconsAreSymmetricAndLaneExternal` proves presence, symmetry, and `>=1.70 m` clearance. They are behind the vehicle at the fixed checkpoint. | **IMPLEMENTED; fresh campaign required; only c31/c32 count; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 GNSS/LiDAR mask separation

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c32 candidate evidence proves the low v12 mask inverted fused contrast by improving upper-arm LiDAR observability. | Geometry v13 moves the physical mask to `z=7.30..7.55 m`, beyond the formal LiDAR bound `1.5 + 10 tan(30 deg)`, while GNSS continues raycasting the global cloud. `FormalReferenceArmGnssMaskStaysOutsideLidarVerticalFov` plus exact mirror/null/clearance tests pass. | **IMPLEMENTED; fresh campaign required; formal analyzer zero; P1-3 prohibited** |

## 2026-08-09 P1-2 c32 retained result and stop-rule correction

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c32 (`da5b15a`) passed all ten per-run hard gates and exact two-arm proofs, but both primary enabled runs selected upper and regressed mean/CVaR/max; mirror direction failed and null CVaR exceeded tolerance. Spec review verified that c17 had only 3/10 passing runs and cannot count as complete comparable evidence, leaving c31 and c32 as two complete failures. | Independent c32 summary and compact hashes are frozen in `docs/dev_planner/p1_formal_test_report_artifacts/2026-08-09-da5b15a/`; raw CSV evidence is retained losslessly gzipped. No calibration, formal run/analyzer, or P1-3 occurred. | **RETAINED; stop rule not met; fresh compliant sensor-geometry repair/campaign required; P1-3 prohibited** |

## 2026-08-09 P1-2 continuous physical GNSS-mask repair

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Clean c31 (`1958af4`) passed every structural, safety, localization, null, and soft gate, but its two complete primary pairs improved mean by only `0.001412/0.001991` and CVaR by `0.000656/0.002457`; sparse canopy balls leave gaps under the simulator's 0.5 m voxel / 0.25 m LOS sampler. | Geometry v12 adds a continuous `z=2.85..3.15 m` physical GNSS mask above the canonical reference arm, exact-mirrors it with the scene, excludes null/soft, and binds its dimensions into the scenario fingerprint. `FormalReferenceArmHasContinuousCollisionNeutralGnssMask` plus existing pointwise mirror/null/clearance regressions pass. | **IMPLEMENTED; second comparable failure retained; fresh third campaign required; analyzer count zero; P1-3 prohibited** |

## 2026-08-09 P1-2 gradual collision-envelope binding

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Retained c15 (`f2f96a6`) 10/10 validator PASS; all enabled STEP1 candidates reached fixed-200 support, while a wrongly shared collision base point forced immediate 2.5 m displacement and made STEP3 durations 25–35 s. The repair translates each base point so the declared gradual fan-out column lies exactly on the same clearance plane. | `P1SoftFallbackPolicyTest.OccupiedSingletonGetsSymmetricGeometricFanout` proves exact clearance-plane distance, gradual base translation, endpoints, symmetry, and zero-miss no-op. A new clean campaign is required. | **IMPLEMENTED; c15 retained; calibration/formal/analyzer not started; P1-3 prohibited** |

## 2026-08-09 P1-2 collision-feasible replacement closure

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Retained c14 (`be1e8ea`) 10/10 validator PASS; fixed-200 fan-out candidates existed, but an occupancy-invalid incumbent was indistinguishable from missing comparison evidence and metrics-only fan-out lacked persistent collision constraints. The repair carries P0-derived symmetric base-point/direction constraints, filters both channels on fixed-200 support, and permits replacement only for fully classified occupancy-only incumbent failure. | `test_p1_candidate_selection` proves occupancy-only replacement, STEP3 closure, and missing-evidence rejection; `test_planning_risk_context` covers formal-only symmetric fan-out; calibration tests prove unique nearest-event selection and typed same-event reference resolution. A new clean campaign is required. | **IMPLEMENTED; c14 retained; calibration/formal/analyzer not started; P1-3 prohibited** |

## 2026-08-07 P1 pre-frozen independent-run formal tolerance

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Ten serial P1-1/P1-1 pairs; derived common-terminal-arc fixed-200 mean/smooth-CVaR/exact-max; corner and resampling error budget; formal manifest ID/SHA/pre-run binding. Production same-snapshot candidate/replacement exact-max gates and P5 authority are unchanged. | Clean `000aa07`: 20 valid unique 90-second runs, frozen calibration `p1-null-20260808-000aa07`; diagnostic `cf80084c…` PASS; formal pair `47b2db09…` / `07b9d9ef…`, sole analysis `12f418ec…`; evidence in `docs/dev_planner/p1_formal_test_report_artifacts/2026-08-08-000aa07/` | **FORMAL FAIL (conclusive): mean/CVaR improvement below frozen thresholds; exact max improved; P1-3 prohibited** |

## 2026-08-06 P1 soft-objective / hard-acceptance incompatibility

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Retained, non-accepted smooth-CVaR pair `d7f1a11470ab454398968eebc6288b60` / `a0a1d5ad774a4b6f91c27709abc6f5d4`; sole analysis `fb41d1aaf6934fdd8d6441fb39d35f2e`; common terminal arc `0.2701304166 m`; P1-2 mean/max `0.4184994876`/`0.4242653019` versus P1-1 `0.4196563658`/`0.4236384817` | 39/39 formal figures nonempty, but strict max, baseline P0 freshness, `/tmp` runtime provenance, and the obsolete baseline manifest identity make the pair FAIL. Independent focused test `ProductionFixed200ModesHaveNoGateFeasibleRiskDirection` uses one real cubic B-spline, captured snapshot, and production mean/LSE/CVaR aggregation. Prefix freezing plus an X/Z-invariant affine time/Y risk field leaves exactly one reachable risk dimension: positive active Y lowers every scalar but raises max, negative Y raises mean, and zero is not strict. | **BLOCKED BY SPEC DECISION; the same deterministic planner fixture is unsatisfiable for all three H4 modes under the hard mean/max gate; P1-3 prohibited** |

## 2026-08-06 P1 smooth-CVaR formal recorder capacity failure

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Clean pair `3b4b7c2ef18241f3b7e9e2a901d8e434` / `dc7cedeff7934d1d9ad1f71586ae7e2f`; P1-2 export `1786054422358`, bag `20260806T221342Z`; filesystem exhausted during finalization | P1-1 sole preflight PASS; P1-2 recorder SIGKILL; sole P1-2 preflight FAIL on missing process-end/validator/bag metadata plus truncated sidecars; formal analyzer not invoked | **FAIL (operational); retain pair, recover reproducible capacity, fresh pair required** |

## 2026-08-06 P1 smooth-CVaR peak alignment

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-400 / IAP-RQ-410 | `fixed_200_smooth_cvar`, `alpha=0.90`, `T=0.01`; deterministic eta solve; stable sigmoid/softplus; entropy normalization; envelope gradient; v4 aggregation tail evidence; fresh run `805116b6bce9441886e15d216a0d4332` at clean `0afb316` | 37 P1 integrity, eight verifier, and 64 formal-analyzer tests; sole v4 preflight PASS; sole ten-figure diagnostic PASS; 56/56 full-support eligible candidates, 14/14 unique published winners, strict descent/negative alignment, 56 distinct final hashes, peak contribution `0.00767–0.01603` | **PASS; fresh formal pair authorized** |

## 2026-08-06 P1 LSE formal peak counterexample

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh pair `2ae433176f7842638fa0cd0767e3d64e` / `39a598c00b1d4944ab7adc3a4875d0f3` at clean `559e134`; sole analysis `f77a2480fc304409a6637470a9d4b44d`; common terminal arc `0.2680614616 m`; P1-2 mean/max `0.4187024725`/`0.4245587735` versus P1-1 `0.4204437873`/`0.4244592145` | Both sole v4 preflights PASS; 39/39 nonempty formal figures; `warnings=[]`, `inconclusive=[]`; all independent hard gates PASS, while `risk_profile_reduced` and its dependent cause-exclusion gate FAIL solely because max increased by `9.9559e-05` | **FAIL; retain pair, implement smooth CVaR, P1-3 prohibited** |

## 2026-08-06 P1 recorded-profile formal-entry smoke

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh enabled run `c8281cb59d9c4df5990d177258d4488b` at clean `78db4ef`; 52 candidates/13 attempts; context history and bag each contain 27 trajectory bindings, with latest recorded sequence `27` selected | Sole v4 verifier PASS/errors=[]; sole ten-figure diagnostic PASS; all candidates optimizer-success/rank-eligible/`200/200`/P1-descent; 13/13 winners strict mean+max descent, negative gradient displacement, accepted | **PASS; fresh formal pair authorized** |

## 2026-08-06 P1 recorded-profile shutdown boundary

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 | Retained fresh pair `f560550d96614bcfa9208bb9f1de10fe` / `e81b98a26d864cb68dfd2b442157ea14`, sole formal analysis `7a677e5327914de2aca1566d92eae9f3`: strict mean/max reduction passed, but profile seq `89` started at `1657065690.4515567` after rosbag's final recorded seq `88` start `1657065689.439588` | Accepted context is now atomic history; `select_latest_recorded_p1_profile` chooses the greatest profile sequence with an exact bag B-spline start. `AcceptedProfileContextAtomicallyRetainsEarlierPublishBindings`, `test_profile_selection_ignores_shutdown_tail_not_recorded_in_bag`, full 32-test integrity suite, and full 64-test P1-2 analyzer suite pass | **IMPLEMENTED; fresh smoke/formal pair required** |

## 2026-08-06 P1 retained-incumbent formal-entry smoke

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh enabled run `46d542b10f0840b09c494b18b720ed3b` at clean `006d4d9`; 36 candidates/9 attempts; 36 distinct initial/final hashes; 54 nonzero initial/final pairwise control-point and profile comparisons | v4 verifier PASS; sole diagnostic PASS with ten nonempty PNGs; all rows optimizer-success/rank-eligible/`200/200`/P1-descent; exactly one accepted winner per attempt; all winners strictly lower mean/max with negative raw-gradient dot displacement | **PASS; fresh formal pair authorized** |

## 2026-08-06 P1 retained-incumbent presence closure

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-400 / IAP-RQ-410 | Failed entry run `99352acb7ad94aea923172569f6f6537`, attempt `20/1`: decision/profile retain trajectory `15`, but candidate row incorrectly recorded `incumbent_available=0` when its comparison had `197/200` support | `P1OptimizationTrace::markIncumbentAvailable()` records identity before comparison in multi/single candidate paths; red-then-green `IncumbentPresenceDoesNotDependOnComparisonSupport`; candidate-selection and bundle-verifier suites pass | **IMPLEMENTED; fresh enabled smoke pending** |

## 2026-08-06 P1 reference-observation dual timestamp smoke

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh metrics-only run `00ee130958c1429db48fce6feb7f14a9` at clean `7f749c7`; final sequence `16`, unchanged trajectory `14`, source start `1657065619.9290924`, later observation `1657065621.4310680`, `200/200` over `2.098024 s` | v4 verifier PASS/errors=[]; source-to-recorded-B-spline delta `2.38e-7 s`; observation-to-odom/truth deltas `3.02/1.97 ms`; exact cloud/context available | **PASS; enabled diagnostic smoke pending** |

## 2026-08-06 P1 reference-observation dual timestamp binding

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 | Metrics-only reference observations keep the original B-spline publish/start identity in `trajectory_start_stamp_s` and the later observation/snapshot scene epoch in `accepted_stamp_s`; retained pair `eb2093a86c694128bed41ce3f9077997` / `306f8baafa184fd2976be702174f9fd9`, sole analysis `297909f855ce464ab8421342621dbec7` | C++ writer regression asserts distinct `10.0`/`10.2` source/observation stamps; analyzer regression binds B-spline to `10.0` and odom/truth to `11.0`; focused suites pass. Retained formal pair already proved strict aligned mean/max reduction but remains FAIL and is never reanalyzed | **IMPLEMENTED; fresh smoke/formal pair pending** |

## 2026-08-06 P1 fixed-lambda formal-entry smoke

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh enabled run `011a3f77fe354724a1c5d07024676c2f` at clean `f81474a`; 36 candidates/9 attempts; 36 unique initial and final hashes; 54 nonzero initial and final pairwise control-point/profile comparisons | Sole v4 verifier PASS and sole ten-figure diagnostic PASS; every row `200/200`, optimizer-successful, rank-eligible, P1-descending; exactly one accepted winner per attempt; every winner strictly lowers mean/max with negative raw-gradient/displacement dot product | **PASS; fresh formal pair authorized** |

## 2026-08-06 P1 metrics-only future-observation smoke

| Req ID | Evidence | Verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fresh run `b2467bf0b8d64d00a46a5e58e25ad2f9` at clean `30f4146`; final context/profile sequence `15` binds unchanged trajectory `12` to snapshot generation `30`, query base `1657065616.9940934`, fallback `metrics_only_reference_observation`, and an exact forced risk cloud | Sole v4 verifier PASS; remaining duration `1.386992 s <= 2.5 s`; fixed support `200/200`; `temporal_in_horizon=1`; temporal/occupied misses `0/0`; one observation per trajectory ID | **PASS; enabled diagnostic smoke pending** |

> 目的：确保“需求（IAP-RQ）↔ 实现 ↔ 测试/实验 ↔ 日志/指标”可追溯，防止做错/做多/漏做。

## 0. 规则
- 每个代码改动必须引用至少一个 IAP-RQ
- 每个 IAP-RQ 必须在本表中至少有：
  - 实现文件路径（Implementation）
  - 验证方式（Test/Experiment）
  - 可观测日志字段（Logs/Metrics）

| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | P0 authoritative health and P1 fresh planning context | `planner/plan_manage/src/p0_risk_grid_runtime.cpp`, `planner_manager.cpp`, `ego_replan_fsm.cpp`, `launch/test_planner.launch.py`, `scripts/dev_planner/analyze_safety_planner_run.py` | `test_p0_risk_grid_runtime`, `test_planning_risk_context`, `test_analyze_safety_planner_run_p1_2.py` | raw-health timing/generation, context timeline, accepted profile tuple, 13 figures | **DONE** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | P1 formal artifact provenance and fixed-lambda evidence binding | `launch/test_planner.launch.py`, P1 optimizer/timeline writers, `test_araim_validator.py`, `verify_safety_planner_evidence_bundle.py`, `analyze_safety_planner_run.py` | `test_verify_safety_planner_evidence_bundle.py`, `test_analyze_safety_planner_run_p1_2.py`, P1 smoke then fresh pair | `schema_version`, `run_id`, `manifest_path`, clean commit, install/runtime paths, bag provenance, process stamps | **IMPLEMENTED; c1-c8 retained before formal, new observability campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Deterministic fork/mirror/null/soft-risk P1 scenes and checkpoint-bound next-formal contract | `apps/demo11_corridor_map_publisher.cpp`, `launch/test_planner.launch.py`, `scripts/dev_planner/{p1_formal_metrics.py,calibrate_p1_formal_tolerances.py,analyze_safety_planner_run.py}`, `src/iap/planner/bspline_opt/{include/bspline_opt/bspline_optimizer.h,src/bspline_optimizer.cpp}` | `test_test_planner_launch.py`, `test_p1_formal_metrics.py`, `test_calibrate_p1_formal_tolerances.py`, `test_analyze_safety_planner_run_p1_2.py`, `test_verify_safety_planner_evidence_bundle.py`, `test_p1_integrity_cost` | expanded `scenario_contract`, immutable `scenario_fingerprint`, fixed seed/geometry/risk sources, strictly unique truth `x=-9.5+/-0.4 m` decision profile, formal-only 10 m local-map observability, localization gates, upper/lower full-200 precheck JSON, normalization fraction `0.30` | **IMPLEMENTED; c6-c8 diagnostic failures retained, fresh observability campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Resumable one-shot P1-2 campaign, independent prequalification, portable evidence roots, generated-cloud geometry invariants, and fail-closed startup/shutdown recovery | `scripts/dev_planner/{run_p1_2_campaign.py,analyze_p1_prequalification.py}`, `scripts/test_araim_validator.py`, `launch/test_planner.launch.py`, `include/iap/planner/p1_fixture_geometry.hpp`, `apps/demo11_corridor_map_publisher.cpp`, `src/iap/sim/sim_extension.cpp` | `test_run_p1_2_campaign.py`, `test_analyze_p1_prequalification.py`, `test_test_araim_validator_shutdown.py`, `test_test_planner_launch.py`, `test_p1_fixture_geometry`, `test_calibrate_p1_formal_tolerances.py` | `campaign.json`, command logs/exit codes/run IDs/export+bag paths including nonzero finalized runs, prequalification JSON/CSV, truth-estimate localization, strictly unique checkpoint, full candidate/occupancy context provenance, point-level mirror/null/clearance/soft-risk evidence | **IMPLEMENTED; c1-c8 retained, fresh campaign pending** |
| IAP-RQ-400 / IAP-RQ-410 | P1 candidate fan-out and objective evidence must be deterministic, bounded, aligned to fixed-200 admission, and distinguish rejected candidate from retained incumbent | `src/iap/planner/bspline_opt/{include/bspline_opt/bspline_optimizer.h,src/bspline_optimizer.cpp}`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `launch/test_planner.launch.py`, `scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py` | `test_p1_integrity_cost`, `test_p1_candidate_selection`; explicit-run diagnostic and recorder smoke | candidate CSV v3 fan-out/aggregation/final-hash/direction fields, replacement decision, paired fixed-200 candidate/retained profiles, recorder completion, nine diagnostic figures | **IMPLEMENTED; fresh enabled smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | Strict P1 pre-admission failures must be inspectable before candidate optimization without misreporting base fallback as first-trajectory loss | `planner_manager.cpp`, `bspline_optimizer.cpp`, `launch/test_planner.launch.py`, `verify_p1_pre_admission_feedback.py`, `analyze_p1_pre_admission_smoke.py` | `test_verify_p1_pre_admission_feedback`, `test_analyze_p1_pre_admission_smoke`, explicit recorder/pre-admission smoke | schema-v3 initial/base duration and fixed-200 reason counts; immutable snapshot tuple; base collision predicate; occupancy cloud; eight diagnostic figures | **IMPLEMENTED; fresh enabled smoke pending** |

---

## 1. 需求追溯表

| Req ID | 需求描述 | Talk/Idea 对照点 | Implementation（文件/模块） | Test/Experiment（如何验证） | Logs/Metrics（必须输出） | 状态 |
|---|---|---|---|---|---|---|
| IAP-RQ-000 | Repo guardrails：AGENTS.md、doc-guard（pre-commit hook + tools/doc_guard.py）、docs 三件套 | — | `AGENTS.md`, `.githooks/pre-commit`, `tools/doc_guard.py`, `docs/` | 提交代码时 hook 拦截缺失文档；`git config core.hooksPath` = `.githooks` | hook exit code | **DONE** |
| IAP-RQ-001 | Rename ROS2 package to `iap` | — | `package.xml`, `CMakeLists.txt`, `src/iap/`, `include/iap/`, `cmake/iap-config.cmake.in` | `colcon build --packages-select iap` 成功；`ros2 pkg list \| grep iap` 可见 | build exit code 0 | **DONE** |
| IAP-RQ-002 | Build artifacts + 模块耗时测量 | 开发基础设施; 实时性验证 | `apps/iap_status.cpp`, `launch/iap_demo.launch.py`, `.clangd`; **timing**: `gnss_extension.cpp` `on_smoother_update_finish_`, `integrity_monitor.cpp` `compute()`, `araim.cpp` `run()`, `trunk_detector.cpp` `detect()` — `std::chrono` + fopen/fprintf → `/tmp/iap_timing.csv` | `python3 tools/plot_icp_timing.py ... /tmp/iap_timing.csv` → Fig C2: 各模块 p99 < 50 ms | `stamp,module,elapsed_ms` in `/tmp/iap_timing.csv`; 4 modules | **DONE** |
| IAP-RQ-002 | Demo4 SO3 dynamics smoke demo | 开发基础设施; 可运行仿真 demo | `launch/demo4.launch`, `apps/demo4_lidar_body_bridge.cpp`, `apps/iap_rosnode.cpp`, `config/sim_demo4/config.json`, `config/sim_demo4/config_ros.json`, `config/sim_ego/demo4.rviz`, `config/sim_ego/fastdds_udp_only.xml`, `sim/ego_planner_swarm_ws/src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp`, `sim/ego_planner_swarm_ws/src/uav_simulator/fake_drone/src/hover_cmd_publisher.cpp`, `fake_drone/CMakeLists.txt` | `colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src --packages-select quadrotor_msgs poscmd_2_odom so3_quadrotor_simulator`; `colcon build --base-paths src/iap src/gnss_comm --packages-select iap`; `ros2 launch iap demo4.launch` | `/demo4/hover_position_cmd`, `/demo4/so3_cmd`, `/sim/drone_0/truth_odom`, `/sim/drone_0/imu`, `/sim/drone_0/imu_iap`, `/sim/drone_0/lidar`, `/sim/drone_0/lidar_body`; `load libodometry_estimation_gpu.so`; `iap input first imu`, `iap input first points`; no `libtrunk_extension.so` load; no `RTPS_TRANSPORT_SHM Error` | **DONE** |
| IAP-RQ-003 | Standalone ROS2 operation — no runtime dep on `glim_ros` | 独立部署需求 | `include/iap/util/rviz_viewer.hpp`, `src/iap/util/rviz_viewer.cpp` (**fix**: skip imu→lidar TF when frames identical); `include/iap/util/standard_viewer.hpp`, `include/iap/util/standard_viewer_mem.hpp`, `src/iap/util/standard_viewer*.cpp` (4 files); `CMakeLists.txt` (rviz_viewer + standard_viewer targets + tf2_ros/nav_msgs/geometry_msgs deps); `config/config_ros.json` (removed libmemory_monitor.so; **fix**: add dump_path, complete QoS sub-objects) | `ros2 run iap iap_rosnode` 启动后 RViz2 可见 `~/aligned_points` + `~/odom`；3D viewer 窗口弹出；无 TF_SELF_TRANSFORM 报错 | `[rviz] published odom`, `[rviz] published aligned_points` log lines | **DONE** |
| IAP-RQ-003 | Demo4 standalone simulator chain | 独立部署需求 | `launch/demo4.launch` uses only installed ROS2 packages from this workspace: `map_generator`, `poscmd_2_odom`, `so3_control`, `so3_quadrotor_simulator`, `local_sensing`, `odom_visualization`, `rviz2` | `ros2 launch iap demo4.launch start_rviz:=false` starts without GLIM runtime packages | `hover command`, `/demo4/so3_cmd`, `/sim/drone_0/truth_odom` | **DONE** |
| IAP-RQ-081 | Phase 1 ordinary EGO planner closed-loop baseline on IAP odometry | 普通闭环基线，不做 integrity-aware planning | `launch/demo9_ego_planner_closed_loop.launch.py`, `config/sim_demo9`, `config/sim_demo9/demo9_gnss.rviz`, `docs/phase1_ego_planner_integration/topic_contract.md`, `sim/ego_planner_swarm_ws/src/iap_phase1_tools`, `tools/build_phase1_ego_planner_closed_loop.sh`, `tools/phase1/check_topic_contract.py`, `tools/phase1/validate_phase1_closed_loop.py` | `bash tools/build_phase1_ego_planner_closed_loop.sh`; `ros2 launch iap demo9_ego_planner_closed_loop.launch.py start_rviz:=false run_duration_s:=30`; `python3 tools/phase1/validate_phase1_closed_loop.py --run-dir /home/dev/ws_iap/src/iap/log/latest` | `/drone_0_visual_slam/odom` used by EGO planner/grid map/SO3 controller; default GNSS is RINEX `GPS,BDS,GAL,GLO` with `/ublox_driver/ephem` and `/ublox_driver/glo_ephem`; `/demo9/drone/path`, `/demo9/truth/path`, `/demo9/desired/path`; `export/desired_vs_truth.csv`, `export/planner_traj.csv`, `export/planner_cmd.csv`, `export/topic_contract.json`, `export/phase1_summary.json` | **DONE** |
| IAP-RQ-010 | 状态扩展：clk_bias δt [m], clk_drift δṫ [m/s] 加入 factor graph (`C(i)=gtsam::Vector2`) | 状态向量包含 clock bias/drift | `include/iap/odometry/estimation_frame.hpp`, `odometry_estimation_imu.hpp/.cpp` (**fix**: per-type `FastMap` relinearize threshold, `clk_bias_relin_thresh=500`/`clk_drift_relin_thresh=5`); `config/config_odometry_gpu.json` (add `clk_bias_noise`, `clk_drift_noise`, clock relin thresholds; bump `isam2_relinearize_skip` 1→5) | `colcon build` 通过；`trace` 日志含 `clk_bias/clk_drift`；无 sync-mode 警告洪水 | `clk_bias(m), clk_drift(m/s)` | **DONE** |
| IAP-RQ-015 | Expose Σ_p 位置协方差块（3×3）到 EstimationFrame；供 PL proxy 使用 | 保护级别需基于不确定性 | `estimation_frame.hpp` (+sigma_p), `odometry_estimation_imu.cpp` (marginalCovariance) | `trace` 日志含 `trace(Σ_p)` 和 `lambda_max(Σ_p)` | `trace_sigma_p, lambda_max_sigma_p, PL_proxy` | **DONE** |
| IAP-RQ-020 | GNSS 紧耦合观测：伪距 + 多普勒建因子（含 clock bias/drift，per-sat） | GNSS tightly-coupled | `include/iap/gnss/`, `src/iap/gnss/` (PseudorangeFactor NoiseModelFactor4<Pose3,Vector2,Vector3,Rot3>, DopplerFactor NoiseModelFactor4<Pose3,Vector3,Vector2,Rot3>, GnssHandler, **GnssExtensionModule** ROS2 bridge); **ECEF pipeline**: E(0)/R(0) free variables with priors σ_E=5m/σ_R=5°; **corrections**: Klobuchar iono + Hopfield trop + Sagnac + TGD; **svdt sign fix**: `pr_meas=pr+svdt*c` (ADD); **svddt Doppler fix**: `dop_meas=dop+svddt*c`; iono from `/ublox_driver/iono_params`; gps_sec/tgd/svddt stored in SatObs/GnssEpoch; epoch/factor logs at `info` level; **post-opt diagnostic**: `on_smoother_update_finish` logs `clk_bias / clk_drift / PR_rms / Dop_rms`; **timestamp fix**: `gpst2utc(obs[0]->time)`; **clock init fix**: `C(frame_id)` inserted if absent; **clock warm-start**: post-opt clk state propagated to next frame via `bias+drift×dt` | 关闭/开启 GNSS 对比；`PR rms` 收敛至 <10 m；`clk_bias` 稳定跟踪接收机时钟偏差 (~300–400 km) | `res_pr, res_dop, clk, clk_dot`; `[gnss_ext] injection #N / diag #N clk_bias/clk_drift/PR_rms/Dop_rms`; `[gnss_ext] E(0)/R(0) inserted` | **DONE** |
| IAP-RQ-025 | GNSS 参数外部化到 config_gnss.json；移除环境变量覆盖 | 参数管理/可维护性 | `config/config_gnss.json` (16 params); `gnss_extension.cpp` (Config 读取); `gnss_handler_` → `unique_ptr` | 修改 JSON 后重启 PR_rms 变化；`enable_debug_csv=true` 生成 CSV | `[gnss_ext] Config loaded: ...` 日志行 | **DONE** |
| IAP-RQ-030 | GNSS per-satellite NIS gating：downweight / exclude（RAIM-ish baseline） | 卫星级完整性、FDE 思路 | `src/integrity/gnss_integrity.*` | 注入某卫星 bias，触发剔星 | `sat_nis, exclude_sats, gamma_R, global_nis` | TODO |
| IAP-RQ-040 | LiDAR ICP 因子健康度：退化/错配检测 → noise inflation / drop factor; CSV 输出 | trunk/几何退化会影响可观测性 | `EstimationFrame::IcpQuality`, `odometry_estimation_gpu/cpu.cpp` (Hessian cond, gamma_lidar); `config_odometry_gpu.json` `enable_icp_csv`/`icp_csv_path`; ICP CSV write in `update_frames()` + `create_factors()` | `python3 tools/plot_icp_timing.py /tmp/iap_icp.csv /tmp/iap_timing.csv` → Fig C1/C2 | `icp_rmse, inliers, cond, gamma_lidar, drop`; `/tmp/iap_icp.csv` | **DONE** |
| IAP-RQ-045 | Global mapping multiscan_window：保留最近 N 帧用于 point-to-multiscan 匹配 | 多帧约束改善全局一致性 | `GlobalMappingParams::multiscan_window` (default 3); `global_mapping.cpp` (frame window pruning); `config_global_mapping_cpu/gpu.json` | 调 N=1/3/5 观察全局漂移变化 | `global_mapping frame_window_size` | **DONE** |
| IAP-RQ-100 | Trunk 检测与圆拟合（中心, 半径, confidence） | 树干几何地标 | `include/iap/trunk/trunk_types.hpp`, `trunk_detector.hpp/.cpp` (Kasa fit, grid BFS) | 对进树林场景启动，日志输出 trunk 数量/半径/置信度 | `trunk_count, radii, confidence[]` | **DONE** |
| IAP-RQ-110 | Trunk 健康度因子接口 (Baseline-A: 不入图) | 树干布局影响 LiDAR 可观测性 | `TrunkDetector::health_factor()` ([0,1]) | health~0 时硬件应填充更大噪声 | `trunk_health` | **DONE (Baseline-A)** |
| IAP-RQ-120 | TDOP 指标（角度多样性） | 树干几何与完整性联系 | `TrunkDetectionResult::tdop/tdop2/lambda_min_H` | 树更分散时 TDOP 下降 | `tdop, lambda_min_H` | **DONE** |
| IAP-RQ-200 | Integrity 输出：PL/AL/IM/mode + 关键中间量; ARAIM CSV; K_fa_used 修复 | PL < AL 安全条件 | `integrity_types.hpp` (+K_fa_used); `araim_debug.hpp` (config constructor, write+worst_hyp); `integrity_monitor.hpp/.cpp` (last_araim_result_, K_fa_used forwarding); `integrity_extension.cpp` (CSV write, k_fa_used bug fix, traj CSV); `config_gnss.json` `"integrity"` section; **fix**: `integrity_monitor.cpp` distinguish UNSAFE-due-to-PL>=AL (warn) vs UNSAFE-in-recovery (info) | `python3 tools/plot_araim_timeline.py /tmp/iap_araim.csv` → Fig B1/B2/B3; IM>0 帧占比>80%; HPL<HAL 始终成立 | `/tmp/iap_araim.csv` (`row_type,stamp,HPL/VPL/HAL/VAL/IM,worst_hyp data`); `PL, AL, IM, mode, K_fa_used` | **DONE** |
| IAP-RQ-210 | Alert Limit AL 由障碍距离动态给出 | 近障碍时 AL 缩小 | `IntegrityMonitor::compute_AL()`, `set_obstacle_distance()` | 越靠近障碍 AL 越小；日志可见 | `AL, obstacle_dist` | **DONE** |
| IAP-RQ-220 | GNSS per-satellite NIS gating（RAIM-ish） | 卫星级 FDE | `IntegrityMonitor::run_gnss_gating()` (chi2 test, gamma_R, FDE greedy) | 注入 bias 卫星被降权/剔除 | `sat_nis, gamma_R, excluded_sats` | **DONE** |
| IAP-RQ-230 | ARAIM 假设集包含树干与星座故障【Upgrade】 | Talk §6.2 / Req upgrade | `include/iap/integrity/araim.hpp`, `araim.cpp: enumerate_hypotheses()` — GNSS sat + constellation + trunk hypotheses; configured priors from `Araim::Params` | `test_araim`: `TrunkHypothesesAddedToCount`; runtime `araim_n_hyp` grows with trunk hypotheses | `araim_n_hyp, excluded_trunk_ids` | **DONE** |
| IAP-RQ-240 | ARAIM PL 计算升级【Upgrade】 | Talk §6.4–§6.6 | `araim.cpp: compute_core()` — per-axis 3-term PL, `LDLT` solve path, cached row contributions, optional OpenMP hypothesis loop | `test_araim`: `ThreeTermPerAxisPL`, `TotalPLIsMaxOverHypotheses`, `ParallelMatchesSerial`; ad hoc serial/parallel benchmark | `PL_E/PL_N/PL_U, HPL, VPL, K_ff_used, K_fa_used` | **DONE** |
| IAP-RQ-247 | LiDAR ARAIM：按 `H_source / H_target / H_level` 评估当前帧 VGICP block 风险 | Req2: current-frame LiDAR factor snapshot + local linear subset solve | `include/iap/integrity/lidar_araim.hpp`, `src/iap/integrity/lidar_araim.cpp`; `odometry_estimation_cpu.cpp` + `odometry_estimation_gpu.cpp` (CPU/GPU VGICP block snapshot to `EstimationFrame::custom_data`); `integrity_monitor.hpp/.cpp` (LiDAR branch merge); `integrity_extension.cpp` (use `on_update_new_frame` + FGO snapshot sync); `integrity_types.hpp` / `araim_debug.hpp` (LiDAR diagnostics) | `test_araim`: `HypothesesEnumerateSourceTargetAndLevel`, `SingleBlockSubsetMatchesManualSolve`, `BiasModelIncreasesProtectionLevel`, `LidarOnlyPLOverridesFallbackByMax`; CUDA 条件测试：`GpuBackendBlocksProduceValidGroupedResult`, `LidarOnlyGpuBlocksOverrideFallbackByMax` | `lidar_valid, lidar_n_hyp, lidar_n_det, lidar_HPL, lidar_worst_mode` | **DONE** |
| IAP-RQ-300 | 候选轨迹生成（motion primitives） | 运动原语离散化候选轨迹 | `include/iap/planner/trajectory_types.hpp`, `trajectory_generator.hpp/.cpp` | 生成 M 条候选轨迹并可视化时间戳点序列 | `trajectory_count, speeds, yaw_rates` | **DONE** |
| IAP-RQ-310 | 预测可见/可观测性集合（占位） | ray-check 遮挡预测 | `include/iap/planner/predicted_integrity.hpp/.cpp` (placeholder) | TODO: 接地图后做 ray-check；当前返回占位值 | `n_vis_placeholder` | **DONE (placeholder)** |
| IAP-RQ-320 | 协方差传播 → Σ_pred → PL_pred | PL 预测供规划使用 | `include/iap/planner/predicted_integrity.hpp/.cpp` (sigma growth, K_pl=3.0) | PL_pred 随时间增长且不同轨迹有差异；σ_grow 可配置 | `PL_pred(s), sigma_pred(s)` | **DONE (baseline)** |
| IAP-RQ-320 | P0 fallback/unknown risk-grid semantics acceptance | P0 RiskGridSnapshot validation | `launch/test_planner.launch.py`, `scripts/dev_planner/analyze_safety_planner_run.py`, `docs/dev_planner/safety_planner_test_report.md` | `ros2 launch iap test_planner.launch.py experiment:=p5_fallback_unknown planner_enable_p5_runtime:=false planner_enable_p5_final:=false ...`; `python3 scripts/dev_planner/analyze_safety_planner_run.py --experiment-id P0-4 --fail-on-threshold` | `p0_4_fallback_unknown_semantics`, `zero_risk_fallback_check`, `fallback_unknown_reason_ok`, `next_debug_branch=continue_to_P0-5` | **DONE** |
| IAP-RQ-320 | P5-1 open-sky normal no-false-trigger validation | P5 runtime/final gate validation | `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p5_1.py`, `docs/dev_planner/safety_planner_test_report.md` | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=gnss_open_sky run_duration_s:=60 validation_duration_s:=60 start_rviz:=false run_validator:=true record_bag:=true`; `python3 scripts/dev_planner/analyze_safety_planner_run.py --experiment-id P5-1 --fail-on-threshold` | `p5_summary.action_counts`, `p5_summary.ok_action_ratio`, `final_gate_fail_count_max`, `future_min_im`, `bad_ratio`, `unknown_ratio`, `next_debug_branch` | **FAIL: debug P5 thresholds/AL provider** |
| IAP-RQ-320 | P5-3 future-only high-risk-zone causal replan evidence and PL/AL query-alignment rerun | P5 runtime/final gate validation; future-only PL/AL risk-grid evidence from the actual `queryPredictedPL()` path | `src/iap/planner/plan_manage/include/ego_planner/p5_runtime_integrity_gate.h`, `src/iap/planner/plan_manage/src/p5_runtime_integrity_gate.cpp`, `include/iap/planner/risk_grid_map.hpp`, `src/iap/planner/risk_grid_map.cpp`, `launch/test_planner.launch.py`, `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p5_1.py`, `src/iap/planner/plan_manage/test/test_p5_runtime_integrity_gate.cpp`, `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp`, `test/test_risk_grid_map.cpp`, `docs/dev_planner/safety_planner_test_report.md` | `ros2 launch iap test_planner.launch.py experiment:=p5_corridor scenario:=manual run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true p5.pred_alert_limit_mode:=current_msg_constant p5_3.fixture.enabled:=true`; `python3 scripts/dev_planner/analyze_safety_planner_run.py --experiment-id P5-3 --fail-on-threshold`; focused Python and CTest suites | `samples`, `query_tau_s`, `fixture_match`, `fixture_expected_hpl`, `fixture_expected_vpl`, `fixture_expected_reason`, `actual_hpl`, `actual_vpl`, `query_alignment_ok`, `current_sample_outside_fixture`, `future_sample_inside_fixture`, `future_fixture_query_aligned`, `active_topic_gap`, `bad_ratio_max`, `first_bad_tau`, `next_debug_branch`, `p5_3_plal_*.png`, `p5_3_query_alignment_*.png` | **FAIL -> 继续 debug P5-3 query alignment / PL-AL margin** |
| IAP-RQ-320 | P1 accepted-trajectory risk profile evidence for P1-2 | P0 risk-grid evidence sampled on the final accepted bspline, bound to its planning snapshot/query base and an accepted-profile context sidecar instead of latest RViz cloud. The formal recorder owns post-flush manifest finalization and the fixed P0 lattice reaches `2.5 s`, covering the degraded-LiDAR initial trajectory. | `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `launch/test_planner.launch.py`, `scripts/planner_bag_recorder_with_finalizer.py`, `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p1_2.py` | smoke evidence preflight, then serial fresh pair and one `--fail-on-threshold` analyzer invocation | profile CSV plus atomic sidecar, finalized manifest, final `profile_seq`, unique `sample_index=0..199`, objective/lambda tuple, snapshot bounds/freshness, per-profile coverage, required figures | **Pending fresh evidence** |
| IAP-RQ-320 | P0 lifecycle / health evidence isolation | Input state is copied before refresh while health remains independently observable; scheduler/refresh timestamps and requested/effective worker counts are emitted for receipt/header-gap diagnosis. Multi-worker prediction uses worker-local modules and deterministic original-index merge. | `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`, `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`, `launch/test_planner.launch.py`, `src/iap/planner/plan_manage/test/test_p0_risk_grid_runtime.cpp` | `test_p0_risk_grid_runtime`; fresh bag requires both raw JSON and RViz health topics | `refresh_callback_start_stamp_s, refresh_callback_end_stamp_s, health_callback_stamp_s, predictor_*_worker_count` | **IMPLEMENTED; fresh P1 evidence pending** |
| IAP-RQ-400 | Integrity-aware planning objective | hinge(PL_pred−AL)²代价+goal+effort | `include/iap/planner/integrity_planner.hpp/.cpp`; `evaluate()` | IM<0时选绕行轨迹；J_integrity > J_goal场景可复现 | `J_total, J_integrity, J_goal, J_effort` | **DONE** |
| IAP-RQ-400 | P1 objective application and accepted-profile reduction gate | Optimizer-applied P1 soft cost must show objective application and strict accepted final-trajectory `c_pi` mean/max reduction versus metrics-only reference | `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p1_2.py` | Fresh pair above; exact lambda `0.00001`; objective/gradient and binding figures generated from the same four paths | `applied_to_objective`, `metrics_only`, `lambda_integrity`, `risk_profile_reduced`, `required_topics_passed` | **FAIL -> lambda/gradient debug**: applied objective and strict reduction passed, but required P0/topic and accepted-profile context/reference-metadata gates failed |
| IAP-RQ-400 | P1 authoritative boundary barrier | P1 applies an inward, capped soft barrier for spatial snapshot/interpolation boundary misses; invalid/stale/time misses remain conservatively non-low-risk without fabricating a direction. This does not change hard P5 gates or the accepted lambda. | `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `src/iap/planner/risk_grid_map.cpp`, `src/iap/planner/bspline_opt/test/test_p1_integrity_cost.cpp` | `test_p1_integrity_cost --gtest_filter='P1IntegrityCostTest.SpatialOutOfMapUsesCappedInwardBarrierWithSkipPolicy:P1IntegrityCostTest.InvalidMissIsConservativeWithoutInventedGradient'` | `f_integrity, weighted_grad_integrity_norm, position_out_of_map, invalid_*` | **IMPLEMENTED; fresh reduction evidence pending** |
| IAP-RQ-400 / IAP-RQ-410 | Fixed-lambda deterministic convergence counterexample | One immutable fixed-200 fixture must independently prove the raw P1 gradient descends, characterize the legacy combined optimizer's opposing displacement/risk regression, and keep the unique legacy winner rejected while the normalized candidate is admitted. | `scripts/dev_planner/run_p1_fixed_lambda_feedback.py`, `src/iap/planner/bspline_opt/test/test_p1_integrity_cost.cpp`, `src/iap/planner/plan_manage/test/test_p1_candidate_selection.cpp` | `python3 scripts/dev_planner/run_p1_fixed_lambda_feedback.py --build-root /home/dev/ws_iap/build`; explicit executed-test count makes a missing target red | JSON `status/elapsed_s/executed`, fixed-200 mean/max, raw-gradient dot displacement, selection/rank/replacement decisions | **COUNTEREXAMPLE CAPTURED; repaired loop GREEN** |
| IAP-RQ-400 / IAP-RQ-410 | Two-stage frozen P1 preference, fixed-200 smooth peak alignment, and post-prepass diversity | A base-only feasible prepass must establish a fixed improvement budget and full support before the P1 stage; normalized P1 and candidate-local anchor remain differentiable/frozen, preserve the exact manifest lambda, and yield at least one full-support total/P1 descent candidate. Fixed mean and normalized LSE each produced a retained formal max-regression counterexample, so the same 200 samples now use entropy-normalized smooth CVaR at `alpha=0.90`, `T=0.01 c_pi`; singleton fan-out uses per-control-point projected gradients of that objective. | `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `docs/adr/0002-normalize-p1-after-base-prepass.md` | `run_p1_fixed_lambda_feedback.py`; mean/LSE/CVaR finite-difference/tie/wide-range and full `test_p1_integrity_cost`; `test_p1_candidate_selection`; `test_planning_risk_context`; nested build through `ego_planner`; fresh CVaR smoke pending | base-prepass lifecycle/termination/budget, active/full norms, normalization scale/constants, normalized P1/anchor merit, `fixed_200_smooth_cvar`, `T=0.01`, `aggregation_tail_fraction=0.90`, bounded peak contribution, full-support candidates and final diversity | **IMPLEMENTED; fresh smoke required** |
| IAP-RQ-400 / IAP-RQ-410 | Future-aligned incumbent replacement | A new candidate and the currently executing trajectory must be compared from the same planning epoch: incumbent samples begin at the planning-start offset into its B-spline, exclude already-flown history, and use the candidate snapshot/query base with risk-grid `tau` restarted at zero. Among self-descending candidates, an incumbent-non-regressing candidate ranks before candidates that cannot be published. Metrics-only rows do not claim enabled replacement closure. | `src/iap/planner/bspline_opt/{include/bspline_opt/bspline_optimizer.h,src/bspline_optimizer.cpp}`, `src/iap/planner/plan_manage/{src/planner_manager.cpp,src/p1_candidate_selection.cpp,test/test_p1_candidate_selection.cpp}`, `scripts/dev_planner/verify_safety_planner_evidence_bundle.py` | `P1IntegrityCostTest.IncumbentRiskUsesOnlyFutureTrajectoryWindow`; `P1CandidateSelectionTest.PrefersReplaceableCandidateOverLowerMeritIncumbentRegression`; metrics-only verifier regression; fresh smoke/formal pair | fixed-200 future window, mean/max replacement gate, exactly one winner, retained-profile epoch alignment, metrics-only closure isolation | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Fixed-200 winner and exact formal binding | Eligible, replaceable candidates rank by fixed-200 mean/max before normalized merit, while the per-candidate scalar optimizer remains unchanged. A deferred prepass has a distinct pending lifecycle stage; every v4 row has a replacement disposition; P0 health preserves exact snapshot time and treats only one duration/fraction-bounded startup-unavailable prefix as startup. | `src/iap/planner/plan_manage/{src/p1_candidate_selection.cpp,src/planner_manager.cpp,src/p0_risk_grid_runtime.cpp,test/test_p1_candidate_selection.cpp,test/test_p0_risk_grid_runtime.cpp}`, `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p1_2.py`, ADR 0002 | `P1CandidateSelectionTest.PrefersLowerFixedLatticeRiskBeforeOptimizerMerit`; `P0RiskGridRuntimeStampTest.HealthJsonPreservesExactSnapshotStampForEvidenceBinding`; formal analyzer startup/singleflight regressions; fresh smoke/formal pair | selected fixed-200 mean/max, `replacement_reason`, `p1_admission_pending/final`, 17-digit `last_grid_stamp_s`, bounded startup duration/ratio | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | Selected-candidate lifecycle identity closure | After bounded multi-candidate optimization, the shared immutable planning context must be rebound from the last optimizer executed to the actual selected candidate before replacement, stale-rejection, retained-incumbent, accepted-profile, or publish evidence is emitted. | `src/iap/planner/plan_manage/src/planner_manager.cpp`, `scripts/dev_planner/analyze_safety_planner_run.py` | formal candidate reconciliation hard gate; failed pair 4 preserves the candidate-1 versus candidate-4 counterexample; fresh smoke/formal pair | candidate CSV, replacement decision, retained profile, replacement/stale timeline, and accepted profile share attempt/generation/query-base/candidate identity | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 | P0 occupied interpolation and occupancy-epoch attribution | An occupied cost-query result is attributed to the exact time-layer/trilinear corner support captured from one stable occupancy generation; point-wise occupancy is not treated as equivalent evidence. A completed occupied query is not mislabeled stale, and accepted fallbacks emit the same corner evidence even without an optimizer start. A generation change fails the refresh closed. | `include/iap/planner/risk_grid_map.hpp`, `src/iap/planner/risk_grid_map.cpp`, `src/iap/planner/plan_env/include/plan_env/grid_map.h`, `src/iap/planner/plan_env/src/grid_map.cpp`, `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp` | `RiskGridMapTest.QueryTraceAttributesOccupiedInterpolationCorner`; `test_p0_risk_grid_runtime`; `P1IntegrityCostTest.EvidenceV4WritesAllCandidateSidecars`; `P1IntegrityCostTest.AcceptedFallbackWritesOccupiedInterpolationCornerEvidence` | query/time layers/corner weights and indices, source flags, invalid reason, raw/inflated occupancy, map voxel/frame/stamp/generation, accepted/initial/final phase, `planner_p0_occupancy_query_evidence.csv` | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 | Immutable live-map occupancy epoch per P0 refresh | Every P0 generation uses one atomically captured raw/fused/inflated GridMap epoch. Live cloud callbacks may advance the map after capture without changing the frozen query results; incomplete/pre-cloud capture fails closed, and mixed generations inside one refresh remain rejected. | `src/iap/planner/plan_env/include/plan_env/grid_map.h`, `src/iap/planner/plan_env/src/grid_map.cpp`, `src/iap/planner/plan_manage/include/ego_planner/p0_risk_grid_runtime.h`, `src/iap/planner/plan_manage/src/p0_risk_grid_runtime.cpp`, `src/iap/planner/plan_manage/src/planner_manager.cpp` | `test_p0_risk_grid_runtime` frozen-factory and missing-epoch regressions; `RiskGridMapTest.RefreshRejectsChangingOccupancyGeneration`; fresh smoke | captured occupancy generation/cloud stamp and per-corner raw/inflated source, no `occupancy_generation_changed` starvation under continuous cloud input | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | P1 evidence provenance v4 and candidate-diversity acceptance | Every fresh P1 optimizer attempt is closed by one schema/run/manifest identity across candidate rows, initial/final control points and fixed-200 profiles, pairwise distances, optimizer checkpoints, exact P0 query corners, accepted/rejected/retained decisions, and the formal figure report. Missing/legacy sidecars fail closed. A metrics-only reference with no optimizer attempt may omit the whole optimizer-attempt artifact group; a partial group is invalid. | `launch/test_planner.launch.py`, `src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `scripts/dev_planner/verify_safety_planner_evidence_bundle.py`, `scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py`, `scripts/dev_planner/analyze_safety_planner_run.py` | fixed-lambda feedback runner; `test_p1_integrity_cost`; bundle-preflight/analyzer Python suites; fresh 30 s smoke before formal pair | v4 identity, mode/attempt-aware atomic sidecars, active/full gradient norms, normalization/base-prepass/merit decomposition, `200/200` profiles, initial/final pairwise matrices, accepted-step checkpoints, exact occupancy corner epoch, exactly one winner, negative raw-gradient alignment, final diversity | **IMPLEMENTED; fresh formal pair pending** |
| IAP-RQ-400 / IAP-RQ-410 | Two-stage prepass soft fallback | A successful collision-feasible base prepass that lacks full P1 temporal/spatial support advances the normal base-only receding horizon, including when an incumbent exists, without applying P1. Failed base/P1 optimization retains the incumbent. Only full-support prepasses may start the normalized P1 optimizer or emit strict candidate evidence. | `src/iap/planner/plan_manage/include/ego_planner/p1_soft_fallback_policy.h`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `src/iap/planner/plan_manage/test/test_planning_risk_context.cpp` | `test_planning_risk_context`; fresh smoke must progress from the unchanged 4.4-second startup/base fallback into later full-support receding-horizon candidates | `base_prepass_start/end`, `base_optimizer_start/end`, `base_prepass_no_full_support`, no fallback row in candidate sidecars | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | Late-subscriber-safe accepted trajectory delivery | The latest accepted B-spline command remains available when `traj_server` finishes discovery after the planner's first publication, preventing an in-memory incumbent from being retained while the vehicle never receives it. | `src/iap/planner/plan_manage/include/ego_planner/trajectory_command_qos.h`, `src/iap/planner/plan_manage/src/ego_replan_fsm.cpp`, `src/iap/planner/plan_manage/src/traj_server.cpp` | `PlanningRiskContextTest.TrajectoryCommandSurvivesLateSubscriber`; fresh bag must contain `/drone_0_planning/bspline` and later seeds must shrink into the fixed P0 horizon | reliable/transient-local/keep-last-one QoS; bag topic count and pre-admission duration | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-410 | Receding horizon loop | 执行Δt后重规划 | `IntegrityPlanner::execution_target()`, `plan()` | 调用`plan()`+`execution_target()`模拟多步闭环 | `chosen_traj_id, dt_execute` | **DONE** |
| IAP-RQ-410 | Accepted receding-horizon bspline profile sequence | Each accepted final bspline in the planning loop is assigned a profile sequence and sampled before trajectory publication, so analyzer can select the last accepted receding-horizon trajectory as authoritative evidence | `src/iap/planner/plan_manage/include/ego_planner/planner_manager.h`, `src/iap/planner/plan_manage/src/planner_manager.cpp`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `scripts/dev_planner/analyze_safety_planner_run.py` | Fresh P1-1/P1-2 reruns write multiple accepted profile sequences; analyzer selects max `profile_seq` and fails closed if the final accepted profile lacks coverage | `profile_seq`, `trajectory_id`, `stamp`, final-profile `matched_sample_count`, `match_ratio`, `selected_profile_seq`, `trajectory_stability` | **FAIL -> lambda/gradient debug** |
| IAP-RQ-410 | P1 attempt/candidate context binding | The selected accepted profile must share one `planning_attempt_id`/`candidate_id` tuple with the P1 debug rows and immutable P0 snapshot context. Soft base fallbacks are marked separately in the lifecycle; strict candidate evidence exists only for full-support P1/metrics-only optimizers. Lifecycle setup before the artifact registry is initialized returns safely without emitting a partial timeline. | `src/iap/planner/plan_manage/src/planner_manager.cpp`, `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp`, `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p1_2.py` | focused analyzer suite plus smoke preflight and serial fresh pair; `p1_2_snapshot_candidate_binding.png` is required | `planning_attempt_id, candidate_id, snapshot_generation_id, query_base_time_s`, sidecar uniqueness/context freshness, `base_optimizer_*` distinction | **Pending fresh evidence** |
| IAP-RQ-500 | 三种 baseline（Passive/CovMin/IntegAware） | 对比 integrity 驱动的优势 | `apps/iap_experiment.cpp` (run_baseline ×3) | 同场景三 baseline 均输出指标 CSV | `baseline, violation_frac, avg_PL, mission_success` | **DONE (stub)** |
| IAP-RQ-510 | 指标：Time(PL>AL)%, AvgPL, MinIM, path/time/effort | 量化对比表格 | `include/iap/experiments/metrics.hpp` (MetricsCollector, write_comparison_table) | `ros2 run iap iap_experiment` 输出 /tmp/*_summary.md | `violation%, avg_PL, min_IM, path_len, time, effort` | **DONE (stub)** |
| IAP-RQ-900 | 自动生成 IEEE Trans methodology.tex（流程图+模块小节+公式） | 论文写作辅助 | `tools/gen_methodology.py` → `docs/methodology/methodology.tex`, `docs/figures/system_flow.tex` | `python3 tools/gen_methodology.py` 生成 .tex；结构无误（12 env 平衡） | gen exit code 0; env mismatch=0 | **DONE** |
| IAP-RQ-311 | 局部占用栅格供光线检测 | Talk §7.2 预测可见卫星/地标 | `include/iap/map/local_occupancy.hpp`, `src/iap/map/local_occupancy.cpp`; VoxelKey + Morton hash; DDA ray traversal | 合成占用体素验证光线命中/未命中正确 | `ray_occluded bool, occupancy_ratio κ` | **DONE** |
| IAP-RQ-312 | 对候选路径点预测卫星可见性集合 V̂(τ) | Talk §7.2 predicted V̂, geometry | `include/iap/gnss/visibility_predictor.hpp`, `src/iap/gnss/visibility_predictor.cpp`; ENU dir from (el,az); ray_occluded per sat | 树冠地图下移动减少 n_vis；开阔增加 | `n_vis, vis_flags[], mean_kappa` | **DONE** |
| IAP-RQ-313 | 估计 LOS 上的冠层密度 κ（预测时刻） | Talk §3.2 σ_eff(κ,θ) | `VisibilityPredictor::predict()` 调用 `occupancy_ratio()` → κ per satellite; `SatObs.kappa` 字段 | κ 在密集冠层下增大；开阔时接近 0 | `kappa per sat, mean_kappa` | **DONE** |
| IAP-RQ-314 | 实现 σ_eff(κ,θ) 和权重矩阵 W | Talk §3.2 σ²_eff = σ²_c · exp(α κ / sin θ) | `include/iap/gnss/canopy_noise_model.hpp` (header-only); `sigma_eff_canopy()`, `info_weight_canopy()` | 相同仰角 κ 更高 → σ_eff 更大，单调性 | `sigma_eff per sat` | **DONE** |
| IAP-RQ-321 | 使 PL_pred 与轨迹相关（替换单一 sigma 增长） | Talk §7.2 predicted covariance from predicted geometry | `predicted_integrity.hpp/.cpp`: `set_occupancy/set_epoch` API; `sigma_grow_at(pos)` = sigma_grow × max(1, f(n_vis, κ)) | 不同候选轨迹产生不同的 PL_pred 序列 | `sigma_grow_eff(s), PL_pred(s)` | **DONE** |
| IAP-RQ-130 | Trunk FGO 扩展模块激活 + ROS2 可视化；EstimationFrame ABI 布局修复 | 树干建图入因子图，RViz 高亮 | `include/iap/trunk/trunk_extension.hpp`, `src/iap/trunk/trunk_extension.cpp` (ExtensionModuleROS2, MarkerArray pub ~/trunks, map_mutex_); `CMakeLists.txt` (独立 trunk_extension .so); `config_ros.json`; **ABI fix**: IAP fields (`clk_bias,clk_drift,sigma_p,icp_quality`) 移至 struct 末尾 → `raw_frame` 偏移恢复与 libglim.so 一致 | 回放时 RViz 看到黄绿色树干圆柱；无 SIGSEGV | `[trunk_ext] visualization publisher created`; trunk cylinders in ~/trunks topic | **DONE** |
| IAP-RQ-131 | 树干数据关联与持久地标 ID | Talk §4.2 landmark map L={c_k} | `include/iap/trunk/trunk_map.hpp`, `src/iap/trunk/trunk_map.cpp`; EMA 平滑; XY距离+半径门限关联; 陈旧剪枝 | 回放中同一树干跨帧保持一致 ID >80% | `landmark_id, seen_count, center_xy` | **DONE** |
| IAP-RQ-132 | 树干观测因子入因子图 (Full-B) | Talk §4.2 TrunkFactor + Σ_trunk | `include/iap/trunk/trunk_factor.hpp`, `src/iap/trunk/trunk_factor.cpp`; `NoiseModelFactor1<Pose3>`; 3D 残差 r = z_k − R^T(c_k−p); 解析 H(3×6); `make_noise(confidence)` | 开启树干因子减小 Σ_p 和 PL_proxy | `r_trunk (3D), H_trunk` | **DONE** |
| IAP-RQ-133 | 置信度加权 TDOP | Talk: TDOP = sqrt(tr((G^T W G)^{-1})) | `trunk_types.hpp`: `tdop_weighted` 字段; `trunk_detector.cpp: compute_tdop()` += W=diag(conf²) 加权分支 | 树干分散且置信度高时 TDOP_weighted 更低 | `tdop_weighted` | **DONE** |
| IAP-RQ-241 | ARAIM 故障假设集枚举 | Talk §6.2 H0 + GNSS单星 + 树干地标故障 | `include/iap/integrity/araim_types.hpp`: FaultHypothesis; `araim.cpp: enumerate_hypotheses()` — non-excluded GNSS sat + constellation-wide + trunk hypotheses with configured priors | `test_araim`: `TrunkHypothesesAddedToCount`; runtime `araim_n_hyp` / `hypotheses.size()` consistency | `araim_n_hyp, hypothesis type counts` | **DONE** |
| IAP-RQ-242 | 全解与子集解（解分离） | Talk §6.4 S0 + S_k via subset solves | `araim.cpp: compute_core()` — nominal `A0/rhs0` precompute, per-row contribution caching, subset `Ak/rhsk` reuse, `LDLT` solve for `S0`, `Sk`, `p0`, `pk` | `test_araim`: `ValidResultWithGoodGeometry`, `ParallelMatchesSerial`; serial/parallel benchmark equality check | `d_k (4D), S0, worst_hyp` | **DONE** |
| IAP-RQ-243 | 分离统计量 σ_ss,q,k | Talk §6.4.3/§6.5 | `compute_core()`: `dS = Sk - S0`, per-axis `sigma_ss_E/N/U`, horizontal aggregation | `test_araim`: `ThreeTermPerAxisPL`, `TotalPLIsMaxOverHypotheses` | `sigma_ss_E, sigma_ss_N, sigma_ss_U, sigma_ss_horiz` | **DONE** |
| IAP-RQ-244 | 检测门限与乘子 (K_fa, K_md) | Talk §6.5/§6.6 P_FA 分配 | `Araim::Params` + `compute_core()` dynamic budget path; per-hypothesis `K_fa/K_md`, thresholds `T_E/T_N/T_U` | `test_araim`: `QInvReasonableValues`, `FaultDetectionThreshold` | `K_ff_used, K_fa_used, threshold per hyp` | **DONE** |
| IAP-RQ-245 | 故障 PL 与总 ARAIM PL | Talk §6.6.2–6.6.3 pl_faulted + pl_ff | `compute_core()`: per-axis 3-term PL, fault-free PL, `PL_E/N/U`, `HPL=max(E,N)`, `VPL=U`; `IntegrityMonitor::run_araim()` forwards to report | `test_araim`: `HplIsMaxOfPerAxis`, `FaultFreePLComponents`, `TotalPLIsMaxOverHypotheses` | `PL_E, PL_N, PL_U, HPL, VPL, pl_ff` | **DONE** |
| IAP-RQ-246 | FDE 闭环（检测→排除→重算） | Talk: 检测到异常则排除并重解 | `compute_core()`: per-hypothesis `fault_detected_*`, deterministic serial aggregation of `detected_rows` / excluded GNSS / trunk ids after parallel evaluation | `test_araim`: `ParallelMatchesSerial`; runtime warning path in `IntegrityMonitor::run_araim()` | `araim_n_det, araim_detected_rows, excluded_prns, excluded_trunk_ids` | **DONE** |
| IAP-RQ-331 | 沿候选轨迹预测 ARAIM PL（规划用） | Talk §7.2 predicted PL_ARAIM at future waypoints | `include/iap/planner/predicted_araim.hpp/.cpp`: `PredictedAraimComputer`; VisibilityPredictor → SatGeometry 列表 → `Araim::predict_geometry()` (r=0) → pl_araim | 树冠路径候选 PL_pred 更高；开阔路径更低 | `pl_araim per waypoint` | **DONE** |
| IAP-RQ-421 | 沿轨迹的动态 AL(τ) | Talk: AL 来自障碍接近度（未来路径点） | `CandidateTrajectory::AL_pred` 字段; `IntegrityPlanner::set_al_fn()` 回调; `plan()` 为每个路径点计算 AL_pred[k] = al_fn_(wpt_pos) | 靠近障碍时 AL_i 更小 | `AL_pred[k]` | **DONE** |
| IAP-RQ-422 | 规划器使用 (PL_pred_ARAIM_i − AL_i) | Talk §7.3 hinge cost 使用预测 ARAIM PL | `IntegrityPlanner::evaluate()` 使用 traj.AL_pred[k] 替代标量 AL; `plan()` 当 use_araim_pl 时用 araim_predictor_.predict_araim_pl() 替换 PL_pred[k] | 完整性违约时规划器选择更安全（即使更长）的路径 | `J_integrity per waypoint` | **DONE** |
| IAP-RQ-050 | IMU 健康度：饱和/模型失配 → noise inflation / alarm | 传感器健康度影响可信度 | `src/health/imu_health.*` | 人为制造饱和或异常噪声 | `acc_norm, gyro_norm, sat_flag, gamma_imu` | TODO |
| IAP-RQ-060 | 规划目标：J(τ)=Σ hinge(PL_pred-AL)^2 + λ_goal*dist + λ_u*effort | 优化版 integrity-aware cost | `src/planner/cost.*` | 单步生成候选轨迹打分 | `J_total, J_integrity, J_goal, J_effort` | TODO |
| IAP-RQ-070 | 预测层：对候选轨迹预测 PL_pred（baseline：代理；升级：ARAIM） | 预测可见/可观测集合与 PL_pred | `src/predictor/*` | 对比不同轨迹的 PL_pred 差异 | `PL_pred(s), AL(s), IM_pred(s)` | TODO |
| IAP-RQ-080 | Receding horizon 闭环：执行第一段，重估计重规划 | active perception loop | `src/planner/mpc_loop.*` / `apps/*` | 跑闭环仿真/回放 | `chosen_traj_id, replanning_rate` | TODO |
| IAP-RQ-090 | 实验与消融：直飞 vs 协方差最小 vs 完整性驱动 | 证明减少违约 PL>AL | `apps/experiments/*` + `docs/*` | 批量跑场景并出表 | `violation_time, min_IM, path_len, time` | TODO |

---

## 2. 未映射改动（临时区）
> 如果你临时改了代码但还没决定它对应哪个需求，先把改动写在这里（提交前必须移入上表）。

- 2026-03-22: IAP-RQ-010 / IAP-RQ-200
  - GNSS clock single-owner contract收敛：`clock_owner_mode` 跨模块联动，默认切到 `gnss`。
  - 增加 ready 时序契约：`IapSharedState::{set,clear,is}_clock_ready`；GNSS 生产 ready，odometry 在 GNSS-owner 下仅 `current+ready` 才读 `C(i)`。
  - 观测与一致性：`KeyLifecycleMonitor` 记录 ownership/missing/conflict/violation；A/B 验收日志显示 `c missing/conflicts/violations = 0`，无 hard optimizer error。

- 2026-07-16: IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 P1-2 Health/Freshness Repair fresh-runtime validation is **FAIL -> preflight environment setup**. Clean `HEAD=e1e185d94aa98b78f2541f3d261cdc2386559aa8`; Python compile and focused analyzer suites passed, but the IAP build stopped while sourcing ROS Jazzy with `set -u` (`AMENT_TRACE_SETUP_FILES` unset). No fresh P1-1/P1-2 exports or bags, analyzer output, final profiles, figures, or P1-3 evidence exist for this attempt; no older artifacts were used.

- 2026-07-16: IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 fresh authoritative P1-2 validation is **FAIL -> lambda/gradient debug**. The analyzer now hard-gates twelve required figures and fails closed on unavailable recorded risk-scene alignment. Fresh serial paths: P1-1 export `1784193115093`, bag `20260716T091155Z`; P1-2 export `1784193251717`, bag `20260716T091411Z`. One analyzer invocation returned exit 2 with failures in raw P0 health, accepted spatial context, reference coverage (`25/200`), and strict c_pi reduction; objective application/binding, validators, trajectory stability, scene alignment, and P5 isolation passed. P1-3 was not run.

## 2026-07-16 P1-2 health/freshness and gradient repair implementation

| Req ID | Requirement/evidence seam | Implementation | Verification and observability | Status |
|---|---|---|---|---|
| IAP-RQ-320 | P0 must publish truthful freshness/latency evidence and accepted profiles must bind to one immutable snapshot with spatial, temporal, frame, generation, query-time, freshness, and coverage classification kept separate | `include/iap/planner/p1_accepted_context_validation.hpp`, `src/iap/planner/p1_accepted_context_validation.cpp`, `src/iap/planner/plan_manage/{include/ego_planner/p0_risk_grid_runtime.h,src/p0_risk_grid_runtime.cpp,src/planner_manager.cpp,src/safety_rviz_publisher.cpp}` | `test_p0_risk_grid_runtime` (31), `test_planning_risk_context` (7); health fields `refresh_*_steady_s`, `refresh_queue_delay_ms`, `provider_batch_duration_ms`, `generation_interval_ms`, `input_callback_*`, `health_callback_count`, `process_cpu_delta_ms`; exact health/cloud generation+stamp; accepted sidecar spatial/temporal/reason counts and trajectory-start stamp | **IMPLEMENTED; fresh authoritative pair pending** |
| IAP-RQ-400 | Fixed-lambda P1 analytic gradient must match finite differences and produce observable descent in the combined optimizer | `src/iap/planner/bspline_opt/{include/bspline_opt/bspline_optimizer.h,src/bspline_optimizer.cpp}`, `scripts/dev_planner/analyze_safety_planner_run.py` | `test_p1_integrity_cost` (20), including bounded actual L-BFGS displacement/termination; P1-2 analyzer (39); profile fields `grad_*`, `neg_grad_*`, `pre_*`, `disp_*`, `grad_dot_displacement`, `delta_c_pi`; structured gradient/descent gate and overlay | **IMPLEMENTED; fresh strict mean/max reduction pending** |
| IAP-RQ-410 | Stale P1 retry must be single-flight per generation while existing trajectory and P5 state-machine semantics remain isolated | `src/iap/planner/plan_manage/{include/ego_planner/p1_replan_admission.h,src/p1_replan_admission.cpp,src/ego_replan_fsm.cpp}` | `test_p1_replan_admission` (4), `test_planning_risk_context` (7); admission precedes acquisition; timeline `retry_deferred`, attempt/rejection/reacquisition counts, replan-load correlation PNG | **IMPLEMENTED; fresh no-storm evidence pending** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | One source must govern all P1-2 hard-gate renderings and final status | `scripts/dev_planner/analyze_safety_planner_run.py`, `test/test_analyze_safety_planner_run_p1_2.py` | 23 structured hard-gate records, CSV/JSON/dashboard, exact 16 required PNG names/nonempty gate, fail-closed exact snapshot cloud/frame/stamp scene binding; analyzer suites 15/39/111/2 pass | **IMPLEMENTED; one-shot analyzer pending** |

## 2026-07-16 P1-2 repair fresh terminal evidence

| Req ID | Requirement/evidence seam | Fresh evidence | Result |
|---|---|---|---|
| IAP-RQ-320 | Truthful P0 health and immutable accepted-context classification | P1-1 export `1784221534659`, bag `20260716T170534Z`; P1-2 export `1784221650125`, bag `20260716T170730Z`; P1-2 275 health rows, 162 stale, refresh mean/max `904.255/968.558 ms`, queue mean/max `666.890/941.971 ms`, age max `2.009 s`; 26 temporal-horizon rejects in each run and no accepted sidecar | **FAIL: freshness and temporal admission** |
| IAP-RQ-400 | Objective-applied fixed-lambda P1 must reduce accepted-profile mean/max | Exact lambda `1e-05`; P1-2 has 26,006 finite debug rows with objective applied, but both accepted profiles are absent (`0/200`) | **FAIL: runtime reduction unavailable** |
| IAP-RQ-410 | One expensive retry per generation and continued publication | P1-2 28 acquisitions over 25 generations (max 2); P1-1 generation 0 acquired 311 times; both bspline counts zero; P5 leakage zero | **FAIL: startup/unavailable admission and publication** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | One-shot structured analyzer and 16 required PNGs | Exact four paths above; one `--fail-on-threshold` invocation returned 137 with cgroup `oom_kill=1` because wall/bag and simulation timebases were combined into one-second replan bins; 13/16 PNGs nonempty, final summary/hard-gate CSV/JSON absent | **FAIL -> analyzer timebase normalization/OOM debug; P1-3 not run** |

## 2026-07-16 P1-2 mixed-timebase, soft-fallback, and single-flight repair

| Req ID | Requirement/evidence seam | Implementation and focused verification | Status |
|---|---|---|---|
| IAP-RQ-320 | Analyzer must never allocate from unrelated absolute epochs and must not claim causal alignment without a recorded mapping | `scripts/dev_planner/analyze_safety_planner_run.py`, `launch/test_planner.launch.py`, `test/test_analyze_safety_planner_run_p1_2.py`; exact `1784221650`/`1657065600` regression proves the legacy 127,156,141-bin failure, bounds the replacement to at most 96 bins/1 MiB, writes a structured summary, and renders unmapped sources on independent panels; invalid startup stamps retain source-row identity | **IMPLEMENTED; fresh one-shot analyzer pending** |
| IAP-RQ-400 | P1 temporal coverage is a soft preference, never a global trajectory hard gate; metrics-only and enabled-unavailable cases need explicit base fallback | `p1_soft_fallback_policy.h`, `planner_manager.cpp`, `bspline_optimizer.cpp`, `test_planning_risk_context`, `test_p1_integrity_cost`; profile/sidecar rows append objective-requested/applied and fallback reason fields, including a fail-closed 200-sample profile when no snapshot exists | **IMPLEMENTED; fresh accepted-profile reduction pending** |
| IAP-RQ-410 | Every P0 generation, including generation 0 and startup-unavailable, has at most one planning admission before context acquisition/optimizer start | `p1_replan_admission.{h,cpp}`, `ego_replan_fsm.cpp`, `test_p1_replan_admission`; typed decisions cover base-initial fallback, keep-existing, same-generation deferral, and retry on the next ready/non-stale generation | **IMPLEMENTED; fresh no-storm evidence pending** |
| IAP-RQ-320 | P0 refresh must fit the unchanged freshness budget by avoiding repeated predictor work | `predictor_module.{hpp,cpp}`, `p0_risk_grid_runtime.{h,cpp}`, `test_predictor_module`, `test_p0_risk_grid_runtime`; batch queries reuse one LiDAR advisory per unique position while preserving per-horizon GNSS/freshness/fusion results and output order; health exposes evaluation/cache counters | **IMPLEMENTED; fresh latency evidence pending** |

## 2026-07-16 P1-2 mixed-timebase repair fresh terminal evidence

| Req ID | Fresh evidence | Result |
|---|---|---|
| IAP-RQ-320 | Fresh P1-1/P1-2 exports `1784225624574`/`1784225745662`, bags `20260716T181344Z`/`20260716T181545Z`; both validators PASS; P1-2 P0 refresh mean/max/p95 `329.794/375.824/351.757 ms`, queue max `0.179 ms`, generation interval max `513.298 ms`, post-startup ready/non-stale | **PASS raw freshness; formal analyzer gate unavailable** |
| IAP-RQ-400 | P1-1 final profile `190/200`, mean/max `0.414578/0.424604`, metrics-only temporal fallback; P1-2 `200/200`, objective applied, mean/max `0.414881/0.432228` | **FAIL: strict mean/max reduction** |
| IAP-RQ-410 | Bags contain 18/20 bsplines; admission/acquisition max `1/1`, optimizer-start max P1-1 `8`, P1-2 `4` | **FAIL: strict optimizer-start singleflight** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Sole analyzer invocation raw exit 1, `csv_artifacts` initialization-order exception, no structured summary/hard-gate files, 2/19 required PNGs | **FAIL -> analyzer csv_artifacts initialization-order / optimizer-start singleflight debug; P1-3 not run** |

## 2026-07-18 P1-2 analyzer/singleflight/fixed-lambda repair

| Req ID | Requirement/evidence seam | Implementation | Verification | Status |
|---|---|---|---|---|
| IAP-RQ-320 | Analyzer must produce a fresh structured failure rather than reuse an old summary when extraction/plotting aborts | `scripts/dev_planner/analyze_safety_planner_run.py`: atomic provisional/final lifecycle and valid-artifact filtering | P1-2 analyzer exception-lifecycle regression | **IMPLEMENTED; fresh run pending** |
| IAP-RQ-400 | Fixed-lambda candidate optimization evidence must bind objective/gradient/displacement fields to an immutable snapshot context | `bspline_optimizer.{h,cpp}`, `planner_manager.cpp`, `test_planner.launch.py`, candidate CSV and 3 required figures | `test_p1_integrity_cost`, P1-2 analyzer suite | **IMPLEMENTED; fresh strict reduction pending** |
| IAP-RQ-410 | One admitted attempt and one context acquisition per P0 generation; multiple distinct candidates are valid inside that attempt | `p1_replan_admission.{h,cpp}`, `ego_replan_fsm.cpp`, `planner_manager.cpp`, candidate-aware analyzer summary | `test_p1_replan_admission`, `test_planning_risk_context` | **IMPLEMENTED; fresh no-storm evidence pending** |

## 2026-08-01 P1-2 authoritative fixed-lattice candidate evidence

| Req ID | Requirement/evidence seam | Implementation | Verification | Status |
|---|---|---|---|---|
| IAP-RQ-400 | Every fixed-lambda optimizer start must retain finite pre/post objective, gradient, displacement, and `c_pi` evidence on one immutable fixed support | `src/iap/planner/bspline_opt/{include/bspline_opt/bspline_optimizer.h,src/bspline_optimizer.cpp}` writes a 200-sample snapshot/query-base support signature, control-point/config hashes, pre/post candidate trace, and a persistent complete deterministic JSON/CSV diagnostic | `P1IntegrityCostTest.FixedLambdaFeedbackGateUsesOneSnapshotAndRecordsAuthoritativeDiagnostic`; `/tmp/iap_p1_fixed_lambda_feedback_diagnostic.{json,csv}` records both modes' objectives, gradients, support, solver/selection, and strict enabled pre/post risk decrease; the fresh formal analyzer correctly fails malformed/incomplete candidate rows | **IMPLEMENTED; formal runtime evidence FAIL** |
| IAP-RQ-410 | Candidate identity must reconcile across optimizer starts, candidate rows, selection, and accepted profile without forbidding normal multi-candidate optimization | `src/iap/planner/plan_manage/src/planner_manager.cpp`, `scripts/dev_planner/analyze_safety_planner_run.py` | P1-2 analyzer candidate-evidence regressions and hard gates require finite/nonempty solver termination, 1–8 candidates, exactly one selected success, timeline/profile reconciliation, and exactly one raw admission/acquisition record per generation; sole formal pair/analyzer failure is reported in `docs/dev_planner/safety_planner_test_report.md` | **IMPLEMENTED; formal runtime evidence FAIL** |

## 2026-08-01 P1-2 candidate selection/ranking repair

| Req ID | Requirement/evidence seam | Implementation and focused verification | Status |
|---|---|---|---|
| IAP-RQ-400 | A fixed-lambda P1 soft preference must not replace a published trajectory solely because the base-weighted total objective is lower | `p1_candidate_selection.{h,cpp}`, `planner_manager.cpp`, `test_p1_candidate_selection`; Attempt 19/20 fixture separates attempt-local rank from replan replacement and requires full-support mean/max non-regression with one strict decrease | **IMPLEMENTED; fresh formal pair pending** |
| IAP-RQ-410 | A risk-rejected P1 replan must keep the current trajectory and retry at most once on a later healthy generation | `planner_manager.cpp`, `p1_replan_admission.{h,cpp}`, `test_p1_replan_admission`; replacement rejection sets the existing singleflight path and never invokes P5 | **IMPLEMENTED; fresh formal pair pending** |
| IAP-RQ-320 | Candidate CSV provenance strings and startup/scene evidence must be typed and causally bound | `analyze_safety_planner_run.py`, `test_analyze_safety_planner_run_p1_2.py`; v2 schema rejects legacy rows, finite failures include row/attempt/candidate/raw/type, activation gates P0 startup, and scene requires native odom/truth within 100 ms | **IMPLEMENTED; fresh formal pair pending** |

## 2026-08-06 P1-2 final-refinement publication closure

| Req ID | Requirement/evidence seam | Implementation and focused verification | Status |
|---|---|---|---|
| IAP-RQ-400 | The authoritative published trajectory must preserve the fixed-200 P1 preference after STEP3 changes control points/timing | `p1_candidate_selection.{h,cpp}`, `planner_manager.cpp`, `test_p1_candidate_selection`; final mean/max must descend from the selected seed and, when present, strictly replace the incumbent on the immutable attempt snapshot | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-410 | A refinement regression must not overwrite the incumbent or create ambiguous candidate/publication identity | deferred candidate trace emission; replacement timeline/decision and candidate-retained profile closure, including truthful `no_publish_no_incumbent` startup disposition | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 | Preflight and formal analysis must detect post-optimizer risk regression | `verify_safety_planner_evidence_bundle.py`, `analyze_safety_planner_run.py`, focused Python regressions; accepted profile is matched by generation/attempt/candidate and compared against seed/incumbent mean/max | **IMPLEMENTED; fresh smoke pending** |

## 2026-08-06 P1-2 temporal prepass admission closure

| Req ID | Requirement/evidence seam | Implementation and focused verification | Status |
|---|---|---|---|
| IAP-RQ-400 / IAP-RQ-410 | A base prepass must not consume an optimizer start when the fixed knot interval/control-point count already proves the seed exceeds the immutable P0 time horizon | `canP1BasePrepassRecoverSupport` in `p1_soft_fallback_policy.h`, guarded admission in `planner_manager.cpp`, `P1SoftFallbackPolicyTest.FixedDurationOutsideSnapshotHorizonCannotEnterBasePrepass`; spatial/occupied misses remain recoverable | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-320 / IAP-RQ-400 | Temporal admission repair must preserve P0 geometry, fixed-200 support, fixed lambda, and P5 semantics | No P0/P5/config change; only the provably futile prepass is skipped before the existing base fallback | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | Formal metrics-only and enabled runs must both deterministically reach a final full-support observation before the test fixture stops replanning | P1-only `test_planner.launch.py` no-replan threshold `0.2 m`, identical to the manager's existing close-to-goal boundary; manifest field `fsm.thresh_no_replan_meter`; `test_test_planner_launch.py` | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | A post-admission support failure must retain the published P1 incumbent, while pre-admission base fallback may still advance the horizon | `P1BasePrepassFallbackInput.has_p1_preference_incumbent`, manager lifecycle reset/publish tracking, `P1SoftFallbackPolicyTest.UnsupportedBasePrepassCannotOverwriteP1Incumbent`; preflight caught smoke `55340378…` profile overwrite | **IMPLEMENTED; fresh smoke pending** |
| IAP-RQ-400 / IAP-RQ-410 | Formal P1-1/P1-2 reduction must compare the same remaining receding-horizon extent rather than including history present only in the longer accepted profile | `p1_terminal_arc_profile`, `compare_p1_2_risk_profiles`, structured CSV/hard-gate fields, and `test_terminal_common_arc_comparison_excludes_unshared_history`; raw accepted artifacts remain 200 samples | **IMPLEMENTED; smoke `086cf8f…` PASS; formal pending** |
| IAP-RQ-320 / IAP-RQ-400 | The bag must contain the exact immutable risk snapshot bound to every accepted trajectory, independent of periodic RViz phase | forced accepted-cloud publication in `planner_manager.cpp` / `SafetyRvizPublisher`; `AcceptedSnapshotCloudBypassesPeriodicThrottle` | **IMPLEMENTED; smoke `086cf8f…` PASS; formal pending** |
| IAP-RQ-320 / IAP-RQ-410 | Formal lifecycle/scene gates must preserve bounded startup, admission rejection, close-to-goal acquisition, and degraded-odom evidence without misclassifying them as retry/availability failures | analyzer regressions for mixed startup prefix, admission/acquisition singleflight, metrics-only trace, and spatially divergent but frame/time-bound odom | **IMPLEMENTED; smoke `086cf8f…` PASS; formal pending** |
| IAP-RQ-400 / IAP-RQ-410 | Candidate/incumbent replacement must compare identical predicted-future domains while retaining full candidate self-descent evidence | `evaluateP1FixedLatticeRisk(..., window_duration_s)`, shared-forward-window selection/refinement evidence, candidate/verifier/analyzer regressions; fresh run `9a16ed92…` | **IMPLEMENTED; one-shot preflight + diagnostic smoke PASS; formal pending** |
| IAP-RQ-320 / IAP-RQ-400 | A metrics-only temporal fallback with sufficient genuine samples must retain valid snapshot/profile provenance without inventing unsupported terminal risk | `validate_p1_profile_context` requires exact metrics-only/fallback identity, objective-off state, matching authoritative flags/counts, freshness/binding, and formal coverage; enabled mode remains full-horizon and `p1_terminal_arc_profile` remains fail closed | **IMPLEMENTED; retained formal pair `5a95b19d…` / `606dfad2…` correctly remains FAIL because its terminal common arc has zero matched P1-1 values; fresh smoke/pair pending** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | The metrics-only context correction must preserve enabled fixed-200 optimization, replacement identity, and candidate diversity before another formal pair | run `d00bac8e…`: sole verifier + sole diagnostic PASS; 56/56 full-support eligible candidates, 14/14 unique selected/accepted winners, strict descent and negative alignment, shared replacement windows, 56 final hashes, 84 nonzero final pairs | **PASS; fresh formal pair authorized, P1-3 not authorized** |
| IAP-RQ-320 / IAP-RQ-400 | Formal P1-1 must expose a real terminal reference when an accepted trajectory was initially longer than the immutable snapshot horizon | metrics-only-only `recordP1MetricsOnlyReferenceObservation`, offset/window accepted-profile writer, exact captured-cloud publication; `MetricsOnlyReferenceObservationSamplesOnlyRemainingTrajectoryWindow`, `MetricsOnlyReferenceObservationIsOncePerTrajectoryInsideHorizon`, analyzer context regression | **IMPLEMENTED; two retained pairs proved zero terminal samples; fresh metrics-only smoke and formal pair pending** |

## 2026-08-08 P1-2 one-shot campaign checkpoint sampling repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Prequalification must observe the immutable truth-progress checkpoint without relaxing or moving its `x=-9.5+/-0.4 m` definition | Campaign `p1-2-20260808-6b395b0-c2` retained all ten serial runs with 10/10 validator PASS and stopped before calibration/formal; 2 m/s accepted-profile starts jumped from about `x=-9.98` to `x=-8.79`. `p1_fork_formal` now fixes manager/optimizer/B-spline speed at `1.0 m/s`; launch records it in the expanded scenario contract and manifest, and prequalification/calibration/formal binding require exact equality. | **IMPLEMENTED; fresh campaign required; formal analyzer invocation count remains zero** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | The fixed checkpoint must have complete temporal P0 support rather than accepting a partial profile | Campaign `p1-2-20260808-6446eea-c3` retained ten runs with only 22–40/200 support; campaign `p1-2-20260808-d890853-c4` retained ten validator-PASS runs and measured checkpoint trajectories up to 13.2 s, while its 10 s horizon remained partial. The formal-only horizon vector keeps `0,0.5,…,2.5` and adds sparse layers through 16 s; launch fingerprints it and prequalification requires the exact vector, while calibration/formal P0 identity binding freezes it after qualification. | **IMPLEMENTED; fresh campaign required; calibration/formal not started** |
| IAP-RQ-400 / IAP-RQ-410 | Every fresh run must deterministically sample the unchanged checkpoint window | In campaign c4 one reference sequence jumped from `x=-9.903` to `x=-9.061`; the formal preset now uses a 0.5 s replan period at the unchanged 1 m/s limit. The period is manifest-recorded, fingerprinted, prequalification-checked, calibration-bound, and formal-bound. | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Full-support interpolation must preserve the 1 s stale contract and conservative occupied-corner semantics | Campaign `p1-2-20260808-dc5f134-c5` retained ten validator-PASS runs; temporal range passed, but 2 s horizon gaps caused deterministic stale interpolation and four exit samples touched occupied corners. Formal horizons now have `<=1 s` gaps through 16 s, P0 Y extent is fixture-bounded at 12 m to preserve refresh capacity, and the exact symmetric central obstacle extends to `x=-1` to force exit clearance. | **IMPLEMENTED; fresh campaign required; no threshold/stale/fallback relaxation** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Both real fork arms must enter the obstacle region with conservative occupied-corner clearance | Campaign `p1-2-20260808-fbe090c-c6` retained ten validator-PASS runs and eliminated stale interpolation, but paths were still converging laterally at the inflated `x=-7` entrance; measured paths reach full separation by `x=-5`. The symmetric box now spans `x=-5..-1`, preserving all lateral/mirror/null and risk-density contracts. | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Residual conservative entrance corners must be cleared without weakening occupancy semantics | Campaign `p1-2-20260809-c8f7341-c7` retained ten validator-PASS runs and localized the remaining 6/15 misses to the central entrance. The final symmetric box is `x=-4.5..-1`, `|y|<=0.35`; equal upper/lower clearance and every risk/threshold contract remain unchanged. | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Stop only after the same scientific gate survives three complete fresh campaigns, compliant repairs, and no remaining nonviolating repair | c11-c13 each completed 10/10 validator-PASS runs and retained the occupied-support gate across geometry repairs. Code inspection proves an untried compliant repair remains: physical collision segmentation returns a singleton, while symmetric P1 supplement was gated after full support. c9-c13 calibration/formal/analyzer remained unstarted. | **DIAGNOSTIC, NOT TERMINAL; add P0-occupied-triggered symmetric geometric seeds before unchanged base prepass; P1-3 unauthorized** |

## 2026-08-09 P1-2 c16 prequalification analyzer closure

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Independent prequalification analysis must remain bounded on full occupancy evidence while validating every row's run/manifest provenance | `analyze_p1_prequalification.py` streams the complete artifact once and indexes only the selected attempt; `test_occupancy_scan_indexes_only_the_selected_attempt`; retained c16 (`5e3a7ee`) diagnostic completed in 204 s at about 152 MB RSS after the legacy join exceeded about 19 GB RSS | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Prequalification must fail closed before calibration when route effect, candidate completeness, checkpoint validity, or localization is not proven | c16 retained 10/10 validator-PASS runs; primary reference/enabled both selected lower with negative mean/CVaR improvement, metrics-only candidate evidence was absent, one enabled checkpoint lacked the opposite route, mirror/null/soft-risk included invalid fallbacks, and localization exceeded 0.5 m in two runs | **FAIL; compliant repair investigation continues; calibration/formal/analyzer/P1-3 not started** |

## 2026-08-09 P1-2 formal homotopy preservation

| Req ID | Requirement/evidence seam | Implementation and focused verification | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | The immutable checkpoint must retain full temporal support for observed ordinary fork trajectories without relaxing freshness or occupied-corner semantics | Formal-only P0 lattice extends with 1 s layers through 24 s; launch/manifest/scenario/calibration/formal binding updated; c16 measured 18.6–22.8 s checkpoint trajectories | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Receding-horizon candidate generation must preserve both collision-feasible fork homotopies and exact mirror identity after the incumbent enters one arm | chord-centered formal fanout in `p1_soft_fallback_policy.h`, formal-bound preserve/mirror parameters in `planner_manager.cpp` and launch; `FormalFanoutPreservesBothChordHomotopiesAndMirrorsCandidateIdentity` proves signed lanes and pointwise Y reflection; defaults disabled | **IMPLEMENTED; fresh candidate evidence required** |
| IAP-RQ-320 / IAP-RQ-400 | Metrics-only must expose real base-optimizer candidate support to independent prequalification without applying P1 or changing formal evidence semantics | `writeP1PrequalificationCandidateProfile` writes a dedicated immutable-context fixed-200 profile; `analyze_p1_prequalification.py` consumes it only for metrics-only precheck, while enabled and formal analysis retain authoritative v4 optimizer/occupancy sidecars | **IMPLEMENTED; fresh reference evidence required** |

## 2026-08-09 P1-2 c17 metrics-only publication isolation

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Metrics-only risk tracing must never gate the real base-planner publication | `P1RefinementRiskEvidence.metrics_only`, `decideP1RefinementRisk`, manager propagation, and `MetricsOnlyRefinementCannotGateTheBasePlannerPublication`; enabled risk and safety decisions are unchanged | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Prequalification must stop before calibration when the preregistered primary effect is absent | c17 (`98d69f6`) retained ten fresh 90-second runs with 10/10 validator PASS. Both valid reference repetitions selected the lower arm; the valid second enabled pair had mean improvement `-0.0006756432` and CVaR improvement `0.0002380348`, versus required `>0.00836` and `>0.00677`. c17 occupancy evidence is losslessly compressed in place. | **FAIL; second consecutive primary-effect reproduction after c16; calibration/formal/analyzer/P1-3 not started** |

## 2026-08-09 P1-2 c18 checkpoint and observability repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Prequalification must measure the actual enabled trajectory at the immutable checkpoint even when strict replacement retains an incumbent | formal-only `recordP1FormalDecisionObservation`, `shouldRecordP1FormalCheckpointObservation`, typed `p1_enabled_retained_incumbent_observation`, fixed-200 accepted profile/context/corners; c18 proves all enabled runs previously lacked a checkpoint | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Metrics-only candidate evidence must fail closed and remain export/provenance bound | `_candidate_evidence_paths` requires `planner_p1_prequalification_candidate_profile.csv` through `_artifact`; no ordinary-candidate fallback; `test_metrics_only_candidate_profile_is_strictly_export_bound` | **IMPLEMENTED; fresh evidence required** |
| IAP-RQ-320 / IAP-RQ-422 | Null/soft localization must gain real LiDAR structure without changing density, canopy, route clearance, or symmetry | `append_p1_observability_landmarks` adds pointwise symmetric, lane-external survey pylons; geometry contract v2 fingerprints exact dimensions; `FormalFixturesProvideSymmetricCollisionNeutralLidarLandmarks` plus mirror/null/soft invariants | **IMPLEMENTED; fresh localization evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Three-campaign blocker requires three comparable scientific-gate results | c18 primary pair metrics are incomplete (`None`) because checkpoint hard gates failed; retained-window diagnostics are not substituted for preregistered pair metrics | **NOT TERMINAL; c18 does not count as a third primary-effect failure; formal analyzer count zero** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Independent analyzer code failure invalidates the campaign and must not reuse its evidence after repair | c19 (`24752f5`) retained 10/10 validator-PASS runs; analyzer exit 1 `NameError` before pair evaluation; stale refactor references removed and pyflakes/focused tests pass | **OPERATIONAL FAIL; c19 not reanalyzed; fresh campaign required** |

## 2026-08-09 P1-2 c20 checkpoint/candidate/localization repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | The formal decision checkpoint must be unique and complete | `shouldRecordP1FormalCheckpointObservation` rejects an already-recorded checkpoint and incomplete fixed-200 support; the process flag advances only after the multi-file write succeeds | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Both arms must prove real, same-attempt upper/lower candidates without substituting optimizer-selected evidence | both arms consume the canonical export-bound `planner_p1_prequalification_candidate_profile.csv`; the planner records admitted collision-fanout trajectories on the immutable attempt snapshot before P1 optimization; selection remains proven by the accepted profile | **IMPLEMENTED; fresh evidence required** |
| IAP-RQ-320 / IAP-RQ-422 | Localization observability must preserve mirror/null geometry and route clearance | geometry v3 places exact-symmetric `0.5 m` pylons at `x=-12,-10,...,0`, `|y|=3.75..4.25`, `z=0..3 m`; generated-point tests prove start visibility, pointwise mirror/null symmetry, and lane-external clearance | **IMPLEMENTED; fresh localization evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | An incomplete hard-gate campaign cannot count toward the three scientific-failure stop rule | c20 (`65fdaf3`) retained 10/10 validator-PASS runs, but primary metrics were incomplete and only null-reference/soft-reference hard gates passed | **NOT TERMINAL; formal analyzer count zero** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Candidate artifact lookup must not depend on an arm-local variable after both arms share one artifact | `_candidate_evidence_paths(export, manifest)` has no metrics-only parameter; focused red/green interface test; c21 (`ab0ffeb`) is retained after analyzer exit 1 and not reanalyzed | **OPERATIONAL FAIL; fresh campaign required; formal analyzer count zero** |

## 2026-08-09 P1-2 c22 deferred-checkpoint evidence repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Same-generation single-flight must not suppress the read-only checkpoint observation | c22's deferred-replan observation preserved `planning_attempt_id=0` and all optimization/publication isolation, but c23 proved the event-driven replan seam can be silent throughout the checkpoint | **SUPERSEDED by executing-state observer below** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Both collision-feasible channels must be independently observable even when the live planner topology is already one-sided | `makeP1PrequalificationEvidenceFanout` generates only an exact chord-centred lower/upper formal evidence pair, evaluated by the real immutable P0 fixed-200 occupancy path and never passed to optimization; analyzer accepts only phase `prequalification_evidence`; `FormalEvidenceFanoutDoesNotDependOnPlannerReturningEmptySegments` | **IMPLEMENTED; fresh evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | An incomplete campaign cannot become a scientific stop result | c22 (`ff862fc`) retained 10/10 validator-PASS runs and a normal independent analyzer exit 2; primary/mirror/soft metrics were incomplete, while the sole complete null pair remained inside null tolerances | **NOT TERMINAL; calibration/formal/analyzer/P1-3 unstarted; formal analyzer count zero** |

## 2026-08-09 P1-2 c23 executing-checkpoint and risk-direction repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | A continuously executing incumbent must be observed even when no replan event occurs in the checkpoint | `EGOReplanFSM::execFSMCallback` retains the latest admitted immutable snapshot/attempt and invokes only `recordP1FormalDecisionObservation` while `EXEC_TRAJ`; `shouldAttemptP1ExecutingFormalObservation` excludes P5 ownership, absent context, non-execution, attempt zero, and an already-recorded checkpoint | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Route availability must be proven before physical commitment while the accepted effect remains checkpoint-bound | `_select_prequalification_candidate_rows` selects the latest complete pre-checkpoint immutable attempt (`start_x<=-9.9`) and still requires real collision-free full-200 upper/lower profiles; c23 contains such pairs in all ten runs near `x=-10.8`; `test_candidate_availability_uses_latest_complete_precheckpoint_attempt` | **IMPLEMENTED; fresh evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Primary/mirror risk asymmetry must come from mirrored physical GNSS obstruction, not hard-coded risk | geometry v4 centers the unchanged risky canopy count/probability/radius/clip above the risky lane, mirrors pointwise, and leaves all crowns above `z=2.83 m`; `RiskyCanopiesOccludeGnssAboveButNotInsideFlightLane`, exact mirror/null, central clearance, and soft-risk collision-height tests | **IMPLEMENTED; fresh direction/effect evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Incomplete evidence and mixed hard/scientific failures do not count as the third stable scientific gate | c23 (`ea4d0f0`) retained 10/10 validator-success runs; primary pair 2 exceeded mean/CVaR thresholds but selected the wrong lane, soft risk regressed, and most other pairs lacked enabled checkpoints | **NOT TERMINAL; calibration/formal/analyzer/P1-3 unstarted; formal analyzer count zero** |

## 2026-08-09 P1-2 c24 fresh observer and safe-lane observability repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Executing-state checkpoint evidence must use a fresh P0 context even when the incumbent's originating attempt is older than one second | `execFSMCallback` retains the incumbent's nonzero originating attempt ID but reacquires the latest immutable snapshot for each read-only observation attempt; `planningRiskContextFresh` and fixed-200 support remain unchanged | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | The declared safe route must have physical localization observability without hard-coded risk | geometry v5 added seven `z=0..0.55 m`, lane-external facades on the safe arm, but c25 showed they did not reverse the fused risk direction | **SUPERSEDED by fused-geometry v6 below** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c24 failure evidence remains immutable and cannot advance the formal sequence | c24 (`a4b8123`) retained 10/10 validator-success runs and 10/10 full-200 two-arm candidate prechecks; most enabled checkpoints were absent because the retained snapshot was stale, while the sole complete primary pair chose the wrong lane despite strong effect | **NOT TERMINAL; calibration/formal/analyzer/P1-3 unstarted; formal analyzer count zero** |

## 2026-08-09 P1-2 c25 incumbent-time and fused-geometry repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | A published incumbent remains observable while same-generation replans transiently put the FSM in `REPLAN_TRAJ` | `isP1IncumbentTrajectoryExecuting` checks nonzero trajectory identity and the closed start/duration interval; the read-only observer uses that physical execution predicate rather than `exec_state==EXEC_TRAJ`; boundary test covers the state-independent contract | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Route direction must follow measured physical fused risk without hard-coded cost | c25 exact-mirror prechecks show the dense structured lane has lower fused risk on both reflected scenes; geometry v6 places unchanged dense-tree/canopy parameters on primary lower and mirror upper, centers unchanged-count soft crowns over lower, removes v5 facades, and retains exact mirror/null and flight-layer clearance; `DenseObservableCanopiesCoverDeclaredLowerRouteOnlyAboveFlight` | **IMPLEMENTED; fresh direction/effect evidence required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c25 cannot advance the formal sequence because most enabled checkpoint profiles are absent | c25 (`f99e72c`) retained 10/10 validator-success runs and 10/10 two-arm prechecks; only one primary pair was complete, exceeded mean/CVaR and improved max, but selected the wrong lane | **NOT TERMINAL; calibration/formal/analyzer/P1-3 unstarted; formal analyzer count zero** |

## 2026-08-09 P1-2 c26 checkpoint scheduling and lane-clearance repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | A synchronous formal replan must not suppress the physical decision-checkpoint observation | `shouldDeferP1PeriodicReplanForFormalCheckpoint` retains the collision-checked incumbent only on the 1.5 m formal approach until the read-only observer records; safety collision/emergency callbacks remain authoritative; boundary regression covers formal/off/recorded/outside/non-crossing cases | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Both formal arms require stable, equal real point-cloud clearance under conservative occupancy | geometry v7 aligns `+/-2.5 m` lane centers with the unchanged `2.5 m` chord fanout and moves unchanged-count/dimension trunks plus exact-symmetric survey pylons outward; `FormalLaneCentersHaveEqualLowAltitudeClearance` proves `>=1.70 m` low-altitude clearance and existing exact-mirror/null tests remain authoritative | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c26 cannot advance the formal sequence because hard-gate evidence is incomplete | c26 (`4242d0c`) retained 10/10 validator-success runs; both complete primary effects/directions pass, but intermittent two-arm occupancy and three missing enabled checkpoints fail closed | **NOT TERMINAL; calibration/formal/analyzer/P1-3 unstarted; formal analyzer count zero** |

## 2026-08-09 P1-2 c27 neutral-arm and closed-window repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | A transient failed checkpoint observation must not immediately start a blocking periodic replan | the formal-only defer interval now covers approach through the closed checkpoint exit; successful record releases immediately and physical exit bounds failure behavior; updated boundary regression proves inside-window defer and post-window release | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Prequalification arm availability must be independent of incumbent side commitment | `makeP1PrequalificationEvidenceFanout` neutralizes lateral control points to current start-Y before the unchanged equal-clearance pair; `FormalEvidenceFanoutNeutralizesCommittedEndpointSide` proves exact pair reflection and neutral terminal controls | **IMPLEMENTED; evidence-only; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Lane structure must not obstruct the central split or post-fork merge | geometry v8 moves unchanged-count/dimension trunks to each lane's external boundary; `FormalLaneBoundaryTrunksStayOnTheExternalSide`, low-altitude-clearance, exact-mirror, and exact-null regressions pass | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c27 cannot advance because checkpoint and two-arm hard gates remain incomplete | c27 (`a18ed52`) retained 10/10 validator-success runs; reference mirror/null/soft passed both gates, while enabled checkpoints and primary/opposite arms remained incomplete | **NOT TERMINAL; incomplete/non-comparable; formal analyzer count zero** |

## 2026-08-09 P1-2 c37 SO3 startup feedback repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | The estimator and flight controller must consume acceleration streams with their declared coordinate/force semantics | `test_planner.launch.py` retains body specific force for IAP and maps SO3 feedback to the simulator's world linear acceleration; the manifest binds `so3_feedback_imu_semantics=world_linear_acceleration`; launch regression proves the topic contract | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c37 cannot advance because one run failed the localization hard gate | c37 (`e52832a`) retained all ten runs and 10/10 complete two-arm candidate proofs, but `pre_primary_2_enabled` reached `91.4916 m` checkpoint error after an upstream `96..112 m/s²` startup acceleration; only 9/10 hard gates passed | **NOT TERMINAL; incomplete/non-comparable; formal analyzer count zero** |

## 2026-08-09 P1-2 terminal c38 result

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Prequalification must exceed both primary effectiveness thresholds in two fresh pairs before calibration | c38 (`c9782a5`) passed all ten hard-gate runs and both enabled arms selected lower, but mean gains were `0.002999/0.001657 < 0.00836` and CVaR gains were `0.006611/0.000653 < 0.00677`; compact summary and hashes are archived under `2026-08-09-c9782a5/` | **TERMINAL SCIENTIFIC BLOCKER; third complete comparable failure (c31/c32/c38)** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Stop after the same scientific gate fails in three complete fresh campaigns with compliant repairs exhausted | c31 established insufficient physical contrast; c32 exposed and motivated repair of the LiDAR/GNSS mask interaction; c38 retained correct primary route direction after the compliant repair but still missed the fixed effect thresholds. Further outcome-driven geometry or parameter changes would violate the no-tuning rule. All three compact bundles and hashes are tracked under `2026-08-09-{1958af4,da5b15a,c9782a5}/`. | **STOP RULE SATISFIED; calibration/formal analyzer/P1-3 not run; analyzer count zero** |

## 2026-08-09 P1-2 c28 overhead-observability repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Restore near-lane physical observability without reintroducing collision obstacles or hard-coded risk | geometry v9 adds deterministic `z=2.85..3.35 m` overhead rafters on the preferred lane; mirror reflection and null paired symmetry are generated by the existing pointwise transforms; `OverheadRaftersAddCollisionNeutralLaneObservability` and all mirror/null/clearance tests pass | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c28 cannot advance despite complete checkpoint/candidate evidence | all 10 runs had unique checkpoints and two collision-feasible 200/200 arms; primary/mirror directions passed, but primary effect, null CVaR, soft effect, and soft localization gates failed | **NOT TERMINAL; soft hard gate makes campaign incomplete/non-comparable; formal analyzer count zero** |

## 2026-08-09 P1-2 c29 bounded-inner-observability repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | Restore the ordinary-planner route contrast without obstructing either exact evidence arm | geometry v10 alternates the unchanged boundary-tree count between external and inner structure, confines every inner trunk to the central-box longitudinal interval, and retains `>=1.70 m` formal lane-centre clearance; exact mirror/null and full generated-point regressions pass | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Keep null localization and the soft-island contract physical and manifest-bound | primary/mirror/null remove v9 rafters after paired null localization exceeded `0.50 m`; soft retains collision-neutral rafters and its unchanged crowns use the declared `y=-2.0 m` island centre so the lower route passes below rather than through the island | **IMPLEMENTED; no risk constants or safety changes** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c29 cannot advance because the base/enabled contrast disappeared and null hard gates failed | c29 (`977de9e`) retained all ten runs and structural proofs; both primary effects were negative and both null localization errors exceeded the hard limit | **NOT TERMINAL; incomplete/non-comparable; formal analyzer count zero** |

## 2026-08-09 P1-2 c30 canonical-reference and LOS repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-422 | A metrics-only formal reference must provide a deterministic mirror-bound control arm without consuming risk | `selectP1FormalMetricsOnlyReferenceCandidate` chooses upper for primary and lower for exact mirror only after collision-feasible formal fanout optimization; it never reads risk and is inactive outside metrics-only preserve-homotopies runs; unit tests cover primary/mirror and inactive fallbacks | **IMPLEMENTED; fresh campaign required** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Physical primary arm separation must exceed run noise without changing tree/canopy parameters | geometry v11 keeps unchanged dense trunks on the enabled arm and moves the unchanged risky-crown count/dimensions above the canonical reference arm for real GNSS LOS obstruction; exact generated-cloud mirror and collision-height regressions pass | **IMPLEMENTED; no encoded risk values** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | c30 cannot advance despite null and soft passing | c30 (`69c7e20`) passed null and soft pairs, but primary reference still followed lower, one reference localization diverged, and primary effect thresholds failed | **NOT TERMINAL; primary hard gate incomplete; formal analyzer count zero** |

## 2026-08-10 P1 Phase 3 v2 protocol and retrospective archive

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 | Historical P1 evidence must remain reproducible without rerunning or invoking the formal analyzer | `scripts/dev_planner/archive_p1_2_retrospective.py`; CMake install/CTest registration; `test/test_archive_p1_2_retrospective.py`; deterministic archive `docs/dev_planner/p1_formal_test_report_artifacts/2026-08-10-c9782a5-retrospective/` with normalized run/pair/mechanism gzip CSV, summary, source inventory, source/output SHA256, README, and five nonempty PNGs | **ARCHIVED READ-ONLY; 80 runs / 40 pairs / 2,632 mechanism rows; formal analyzer count zero** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | A revised P1 protocol must isolate fixed-candidate soft-cost causality from P2 route ranking and retain collision/dynamics/P5 authority | Phase 3 v2 in `safety_planner_test_plan.md` and `safety_planner_p0_p5_test_plan.md` defines P1-1 through P1-6 same-snapshot, preregistered sweep/null/fresh-pair, unknown/stale, and P5-isolation gates; fork primary/mirror/null/soft moves to same-attempt/same-candidate-set/same-snapshot P2-2 | **PROTOCOL FROZEN; future fresh campaign required; no planner/runtime change** |
| IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 / IAP-RQ-422 | Revised protocol and retrospective observations must not retroactively change the stopped v1 verdict | report section `2026-08-10 P1-2 c31–c38 retrospective and Phase 3 v2 protocol freeze`; archive summary keeps c31/c32/c38 complete comparable failures, c33–c37 incomplete diagnostics, `historical_verdict=BLOCKED`, `product_p1_status_changed=false`, and `formal_analyzer_invocation_count=0` | **NO STATUS CHANGE; no ROS campaign, formal analysis, or P1-3** |

## 2026-08-21 ICRA-014 dense rolling spatial-advisory reuse

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 | Reuse only the private spatial GNSS/LiDAR advisory while preserving exact source identity, timestamp and validity provenance | `RollingSpatialAdvisoryWindow` stores dense world-keyed slots containing the existing private `PredictorModule::SpatialAdvisory`; full Predictor results, prior-derived state, horizon outputs and `RiskVoxel` materialization are excluded. Exact source comparisons and conservative identity validation are covered by `test_rolling_spatial_advisory_window`. | **IMPLEMENTED; unit evidence only** |
| IAP-RQ-320 / IAP-RQ-321 | A shifted fixed lattice must retain overlap, recompute only entering spatial positions, and rerun every horizon fusion | Repository-local `canonical_rolling_spatial_diagnostic.json` records first `12800/0/12800/0/76800`, stationary `0/12800/0/0/76800`, and `+1 x` `320/12480/320/320/76800` as recompute/retained/entered/evicted/fusion counts; fresh-full-rebuild outputs are scientifically equal. SHA-256: `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`. | **IMPLEMENTED; canonical diagnostic invoked once** |
| IAP-RQ-320 / IAP-RQ-322 | Rolling reuse and immutable RiskGrid publication must be atomic and failure-safe | P0 begins one candidate window per refresh, commits only after complete RiskGrid refresh succeeds, and aborts on every failure or unfinished provider destruction. The last successfully published occupancy generation retains its immutable LOS owner across fresh adapter captures. End validation rejects occupancy/current/atomically-captured-GNSS/LiDAR changes; abort, query-identity mismatch, and source-change regressions prove the active snapshot/ring remain reusable. | **IMPLEMENTED; fail-closed** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Preserve frozen geometry, worker equivalence and all downstream behavior | Production P0 tests cover stationary, sub-voxel and `+1 x` shifts at worker counts 1/2/4 and compare against a fresh full rebuild; retained Predictor/RiskGrid/P0-P5 suites pass against the current ICRA-014 library. No main flow, smoke, qualification, formal benchmark, analyzer, GPU/CUDA, calibration, or phase-4 work was performed. | **IMPLEMENTED; SUPERVISOR review required** |

## 2026-08-21 ICRA-015 source-identity and legacy-diagnostic repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-312 / IAP-RQ-314 | Rolling equality must contain every and only active spatial-source fields | `rolling_spatial_advisory_window.cpp` projects GNSS identity to original epoch stamp, ordered count and consumed `sat_id/excluded/elevation/azimuth/pr_sigma` plus immutable occupancy identity; LiDAR identity always tracks FIM-primitives ownership and conditionally tracks legacy map ownership plus `n_trunks_observed/tdop/excluded_trunk_ids`. Exact collision-safe equality, inactive-source isolation, non-finite rejection and consumed/unconsumed field regressions are in `test_rolling_spatial_advisory_window.cpp`. | **IMPLEMENTED; review repair** |
| IAP-RQ-312 / IAP-RQ-320 / IAP-RQ-321 | Current time and validity must remain per-horizon inputs without erasing or restamping spatial evidence | Fusion current-stamp/prior refresh tests retain the ring, rerun every growth/fusion/materialization path and compare the complete `PredictorQueryResult` contract with a fresh forced-full query. Existing freshness tests prove cached evidence cannot bypass stale/invalid query outcomes; active consumed-current changes still invalidate. | **IMPLEMENTED; unit evidence only** |
| IAP-RQ-320 / IAP-RQ-322 | Production source validation must match the active spatial projection and remain atomic | `p0_risk_grid_runtime.cpp` validates the LiDAR FIM owner always and the legacy map owner only while legacy fallback is enabled. Production tests cover active owner races, disabled map-owner races, Fusion/GnssOnly/LidarOnly stationary refreshes with updated snapshot/current time and prior generation, full RiskGrid voxel/occupancy equivalence to a fresh rebuild, and existing rollback/source-race behavior. | **IMPLEMENTED; fail-closed** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Preserve phase-2 legacy call-local LiDAR cache diagnostics while exposing truthful rolling work | Fresh LiDAR-capable positions report legacy `1/1/(H-1)`, stationary cross-refresh positions and `GnssOnly` report `0/0/0`; a rejected horizon after population does not fabricate a hit. Cross-refresh work is represented by generalized spatial recompute/reuse, GNSS/LiDAR invocation, retained/entered/evicted and fusion counters. Rolling and production count regressions preserve the compatibility boundary. | **IMPLEMENTED; compatibility restored** |
| IAP-RQ-312 / IAP-RQ-314 / IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Retain accepted ICRA-014 ring/science behavior and make focused verification reproducible | The ICRA-014 `docs/CHANGES.md` entry contains repository-local root/plan_env/ego build commands, rolling/P0/retained-ICRA011 CTest commands and exact ICRA-015 `libiap.so` linkage proof. The disabled canonical diagnostic is not rerun; its existing SHA-256 remains `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`. | **IMPLEMENTED; SUPERVISOR review required** |

## 2026-08-23 ICRA-033 atomic refresh evidence

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 / IAP-RQ-322 | Active safe-map state must not be confused with mutable refresh work | `P0RiskGridRuntime::RefreshEvidenceRecord` exposes nonzero monotonic attempt ID, explicit state, result and previous-success identities; `completeRefreshEvidence` atomically freezes qualification fields while a failed attempt may retain the previous active generation | **IMPLEMENTED; deterministic transaction tests pass** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | Completed evidence must bind the source snapshot and remain immutable under concurrent callbacks | `buildSnapshot` captures `InputReadiness` under the combined health/LiDAR lock; the blocked-provider test mutates live stamps from 100 to 101 during work and proves the completed evidence retains the transaction's 100 stamps across repeated publication, failure and recovery | **IMPLEMENTED; 79/79 runtime tests pass** |
| IAP-RQ-320 / IAP-RQ-321 | Analyzer must group by attempt identity and reject every partial or contradictory claim | `gate0_analyzer.py` schema v2 plus 38 tests cover equivalent/conflicting duplicates, unknown/zero/regressed IDs, active/previous/result chain mismatch, result reuse, complete PRE/IN_PROGRESS inventory, ICRA-032 interleaving, and cold/later intervals | **IMPLEMENTED; fail-closed** |
| IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322 | One authorized replacement smoke must meet the frozen Gate-0B evidence contract | `results/icra27/icra033/`: static preflight PASS; runner invocation 1/exit 0; analyzer invocation 1/exit 1; 14 successful generations, 76,800-query shape, 166/166 valid integrity reports, but two startup failures lack finite message-clock attempt stamps | **BLOCKED; Gate-0B NOT_QUALIFIED; no benchmark/P4/P5** |

## 2026-08-24 ICRA-041 clean-room P4-G0B requalification

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Qualify reviewed P4-G0B behavior without consuming affected historical products | Zero product changes; fresh task-local IAP → plan-env → path-searching → bspline-opt → plan-manager chain; sanitized environment and `preflight/{build_identity,linkage}.json` prove current cache/runtime resolution with zero historical/default/build-tree/missing matches | **REQUALIFICATION READY FOR REVIEW** |
| IAP-RQ-423 | Preserve authoritative identity precedence and truthful metrics boundary | ICRA-041 binaries pass focused identity 3/3 and complete decision 15/15; focused false boundary 1/1 and integration 5/5 prove `SELECTION_NOT_AUTHORIZED`, original selection and no application | **DETERMINISTIC PASS; not Gate PASS** |
| IAP-RQ-423 | Preserve complete G0B regression and production-A* evidence | `results/icra27/icra041/test/`: collision 17/17, P1 39/39, fresh path P4 5/5, fresh occupancy 6/6, plan-manager 9/9 with 186 active/one disabled, and repeat-stable fixture hashes/statistics; all process exits zero | **DETERMINISTIC PASS; SUPERVISOR review required** |
| IAP-RQ-423 | Prove ICRA-041 adds no write to retained ICRA-039/040 trees | `preflight/retained_manifest.json`: 3,123 byte-level file/symlink entries, equal before/after canonical SHA-256 `d18c1c89…e3162`, `cmp` exit zero; full manifests retained unstaged | **NO FURTHER WRITE PROVEN; historical incident unchanged** |

## 2026-08-24 ICRA-042 P4-G0C protocol registration

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Freeze the complete pre-data G0C matrix and proposed threshold state | Canonical `config/icra27/p4_g0c_protocol_v1.json` binds seeds `[211,223,237,253,271]`, repetitions `[1,2,3]`, 15 immutable IDs, exact effective values, minimum 100 decisions, Type-7 rules and `1e-12 risk_cost`; `p4_threshold_registry_v1.json` retains four null gates, null bundle hash and disabled application | **PROTOCOL REGISTERED; registry uncalibrated** |
| IAP-RQ-423 | Make the metrics-only launch contract explicit and immutable | `launch/test_planner.launch.py` keeps the general `p4.metrics_only=false` default, registers exactly one G0C experiment/scenario, binds protocol/registry/fixture/config hashes and immutable run identity, records CSV/process/no-bag/no-RViz truth, and rejects conflicting overrides | **IMPLEMENTED; no online selection change** |
| IAP-RQ-423 | Stop future collection and analysis on incomplete or mutable evidence | `run_p4_g0c_calibration.py` expands the exact matrix, places GPU preflight before ROS, refuses existing run directories and stops without retry; `analyze_p4_g0c_calibration.py` requires all hashes/runs and complete typed 200/200 rows, rejects every timeout/coverage/application/path/noise violation and emits only a hash-bound `DRAFT_UNCALIBRATED` | **FAIL-CLOSED; synthetic tests only** |
| IAP-RQ-423 | Reproduce against current task-local products without historical/default planner linkage | `results/icra27/icra042/` records 21/21 focused, 16/16 launch golden, 376/376 Python and P4 15/15 + 5/5 + 17/17 + 5/5 + occupancy 6/6 + plan-manager 9/9; build/linkage manifests show exact installed bytes and zero historical/default IAP/planner resolution | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-043 P4-G0C protocol repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Future collection must expose every ordered attempt before launch and retain the first failure without retry/filtering | `p4_g0c_runner_state_v2` persists exact registered/attempted/completed IDs plus 15 indexed attempt records; runner tests inspect the on-disk RUNNING record inside the executor and prove first-failure FAILED visibility with zero retry | **IMPLEMENTED; synthetic tests only** |
| IAP-RQ-423 | Analysis must bind the authoritative runner state and exact calibration-root inventory | analyzer requires COMPLETE/hash-bound 15/15/15 ordered ledgers and rejects missing/partial/failed/reordered/duplicate state, extra retry/run-like directories, alternate manifests/decision CSVs and every header-only registered run | **FAIL-CLOSED; no live bundle analyzed** |
| IAP-RQ-423 | Production decisions must use one exact typed schema and internally consistent immutable path identity | shared exact ordered 36-column schema validates canonical IDs/counts, finite nonempty context, positive path lengths, duplicate decision identity and risk/original ratio within the pre-data `2e-5` tolerance frozen in canonical protocol bytes | **IMPLEMENTED; no threshold-value change** |
| IAP-RQ-423 | Preserve prior retained products and prove the repair without live execution | focused 50/50 (including review-remediation red/green cases) and final full Python discovery 389/389 pass; static checks pass; all 3,829 files across 12 ICRA-042 build/install trees retain identical before/after manifest hash `6836841b…d784d34`; compact evidence is under `results/icra27/icra043/` | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-044 P4-G0C live-artifact protocol repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Reject any dirty live root before state/GPU/launch while keeping plan-only non-mutating | `run_p4_g0c_calibration.run()` accepts only absent/empty non-symlink roots for non-plan modes; synthetic tests cover arbitrary file, retry dir, old analyzer output, registered run/state, root symlink, preflight-only reuse and zero fake-boundary calls | **IMPLEMENTED; no real GPU/launch** |
| IAP-RQ-423 | Bind every completed run to its complete real production artifact tree | shared `p4_g0c_run_artifact_inventory_v1`; runner state v3 binds inventory and dynamic test-planner manifest path/hash; analyzer recomputes every path/type/size/hash and rejects all required drift/escape/symlink/secondary-artifact cases | **FAIL-CLOSED; synthetic 15-run tree only** |
| IAP-RQ-423 | Make analyzer outputs named, raw-hash neutral and non-overwriting | CLI prevalidates exact in-root names, swap/alias/symlink/existence and registry collision before analysis; exclusive writes prevent overwrite; rejected bundles emit no draft; read-only reanalysis preserves the raw hash | **IMPLEMENTED; no registry/application change** |
| IAP-RQ-423 | Preserve retained products and prove repair without live execution | focused 64/64 and post-review Python discovery 403/403 pass; 3,829 files across 12 ICRA-042 trees retain identical before/after hash `6836841b…d784d34`; compact evidence is under `results/icra27/icra044/` | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-045 G0C analyzer lexical-alias repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Reject lexical output aliases before analysis or filesystem mutation | analyzer compares the expanded absolute request to canonical resolution for both output roles; direct CLI regression covers `nonexistent/../` and `runs/../runs/`, exit 2, zero `analyze()` calls and zero target/intermediate/other-output creation | **FAIL-CLOSED; synthetic only** |
| IAP-RQ-423 | Preserve canonical output behavior and prior G0C protections | fresh production-shaped bundle accepts canonical relative analysis and absolute draft paths; all ICRA-044 output, inventory and raw-hash adversaries remain green | **IMPLEMENTED; no schema/runner/product change** |
| IAP-RQ-423 | Reproduce repair while preserving protected and retained artifacts | focused 66/66 and one final Python discovery 405/405 pass; 3,829 files across 12 ICRA-042 trees retain identical before/after hash `6836841b…d784d34`; compact evidence is under `results/icra27/icra045/` | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-046 G0C live calibration

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Build and qualify only fresh task-local products before live use | six configure/install pairs exit 0; source/install config and launch bytes match; linkage is task/external-only; focused 66/66, full Python 405/405 and prescribed fresh C++ regressions pass, but `--show-args` did not establish runtime `so3_control` resolution before runner entry | **BUILD/TEST PASS; DEPENDENCY GATE VIOLATED** |
| IAP-RQ-423 | Gate the one-shot 5×3 calibration on a real GPU and stop at first failure | sole runner GPU preflight passes `nvidia-smi`, `cuInit=0`, `device_count=1`; first launch exits 1 because `so3_control` is not found, both required processes never start, ledger retains 1 attempted / 0 complete / 0 retry | **BLOCKED; first launch only** |
| IAP-RQ-423 | Preserve raw failure truth and forbid downstream threshold action | four-file raw runs manifest hash `f307e61a…97079438`; analyzer invocations 0, no analysis/draft, registry still proposed/null/disabled, all task products and raw evidence retained below ICRA-046 | **FAIL-CLOSED; not G0C PASS** |

## 2026-08-24 ICRA-047 G0C replacement protocol

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Replace consumed v1 identities without changing calibration science | canonical `p4_g0c_protocol_v2.json` registers 15 disjoint r2 IDs with unchanged seeds/repetitions/effective values/duration/floor/tolerance/quantiles/formulas/minimum/no-retry rules; `p4_threshold_registry_v2.json` remains proposed/null/disabled | **IMPLEMENTED; synthetic only** |
| IAP-RQ-423 | Preserve the disqualified ICRA-046 execution as immutable lineage | `p4_g0c_replacement_lineage_v2.json` binds v1 protocol `9e89…906d`, failed ID, missing `so3_control`, 1/0/0 ledger, raw manifest `f307…9438`, runner state `a6db…ef1`, zero analyzer and no calibration/draft/application eligibility | **BOUND; no v1 reuse** |
| IAP-RQ-423 | Prove complete package/executable/plugin/config closure before GPU | `p4_g0c_runtime_dependencies_v2.json`; `validate_runtime_dependencies()` checks 18 packages, 13 loadable script/full-native-ELF executables, SO3 component/full-native-ELF library, 14 exact SHA-256-bound configs, six config-selected full-native-ELF IAP libraries and one hashed launch contract before GPU-running state; ELF dynamic links must resolve and every missing/content-drift/truncated/wrong-architecture/unresolved-linkage case plus duplicate, historical, undeclared, alias and symlink adversaries is covered | **FAIL-CLOSED; zero real GPU/ROS** |
| IAP-RQ-423 | Make standalone dependency proof non-bypassable | `--dependency-preflight-only` and full mode call the same validator; roots are fresh/one-use and full repeats the gate; after recording and correcting a pre-review external-temp policy miss, repository-local focused G0C tests pass 62/62, launch golden 16/16 and final full Python discovery passes 417/417 | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-048 G0C v2 runtime-contract repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Effective runtime values must equal the registered v2 protocol through the real launch path | every G0C version uses explicit frozen values; real `_launch_setup` regression proves ego-planner, test-planner manifest, run manifest and protocol agree on P1/P2 `metrics_only=false`; runner/analyzer recompute both manifest hashes and reject exact-type effective-contract disagreement before COMPLETE/draft | **REPAIRED; synthetic only** |
| IAP-RQ-423 | Protocol and proposed-registry identity must be immutable without a protocol/dependency/launch hash cycle | trusted caller mode plus shared loader pins full protocol `8b0b2c3e…59de79` and registry `99ccf38c…beb94f` hashes before dependency/output work; launch separately freezes exact-type scientific/effective values and requires declared actual hashes; schema downgrade, coordinated and isolated drift regressions reject | **FAIL-CLOSED; acyclic trust split** |
| IAP-RQ-423 | Ambiguous raw bundles must not become threshold-draft eligible | inventory rejects secondary v1 and v2 run manifests; analyzer adversaries emit no draft while a normal production-shaped v2 bundle remains accepted | **REPAIRED; no threshold action** |
| IAP-RQ-423 | Preserve v1, lineage and retained evidence while refreshing only the unavoidable hash cascade | v1 hashes, lineage `9268ec4d…c6d5`, ICRA-046 aggregate `823d41bf…96b1`, ICRA-047 aggregate `b411cfd9…f81` and protected PDF remain unchanged; final repository-local focused 74/74, launch golden 16/16 and Python 429/429 pass | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-049 G0C top-level evidence binding

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Production top-level runtime evidence must match the nested registered protocol binding | shared protocol module freezes the exact 28-entry top-level mapping and validates every required field with canonical exact JSON type equality while preserving complete nested `p4.g0c` validation | **FAIL-CLOSED; synthetic only** |
| IAP-RQ-423 | Provenance refresh must not conceal semantic top-level disagreement | production-shaped runner/analyzer fixtures cover 28×3 remove/change/wrong-type adversaries; runner rejects before COMPLETE/inventory and analyzer rejects after legitimate inventory/state refresh with no draft; P1/P2 top-level-only drift is included | **REPAIRED; no threshold action** |
| IAP-RQ-423 | Preserve accepted ICRA-048 anchors and all retained evidence | no launch/config/protocol/registry/dependency/lineage/fixture bytes changed; focused 77/77, launch golden 16/16 and full Python 432/432 pass under repository-local TMPDIR | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-050 G0C r2 live calibration

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Fresh-build the complete declared closure without historical/default products | One sanitized non-symlink merged build under `results/icra27/icra050/{build,install,log}` exited 0 with 17/17 packages; exact sources, environment, command and hashes are retained | **BUILD EXIT 0; CUDA runtime library absent** |
| IAP-RQ-423 | Prove the complete dependency closure before GPU or ROS | Sole standalone runner used only the ICRA-050 install plus `/opt/ros/jazzy` and exited 2 with `DEPENDENCY_RUNTIME_LIBRARY_MISSING:iap:lib/libodometry_estimation_gpu.so`; state SHA `701c37b8…bd` | **BLOCKED; dependency gate failed** |
| IAP-RQ-423 | Stop without retry or downstream evidence mutation on a typed dependency failure | Zero full runner, GPU, ROS/launch and analyzer calls; no live runs root, analysis or draft; zero task processes; all task products and prior evidence retained | **FAIL-CLOSED; not G0C PASS** |

## 2026-08-24 ICRA-051 G0C CUDA reissue

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Fresh-build and statically prove the complete CUDA closure before runner use | Sole sanitized CUDA-on build exited 0 with 17/17 packages; exact package index and six non-symlink ELF libraries pass hashes/linkage; `libodometry_estimation_gpu.so` is loadable at `c241e032…f894` | **CUDA CLOSURE PASS** |
| IAP-RQ-423 | Require the complete standalone dependency gate before GPU or ROS | Sole separate dependency invocation exits 0 with 18 packages, 13 executables, one component, 14 configs and six libraries from exact ICRA-051/Jazzy prefixes; state `fc6812e4…cf1` | **DEPENDENCY PREFLIGHT PASS** |
| IAP-RQ-423 | Run the registered matrix once behind built-in GPU proof and stop on first failure | GPU passes `nvidia-smi`, `cuInit=0`, one device; first ID launch exits 1 on unavailable ROS logging directory before either required process starts; runner `7c3cafc5…46a7`, 1 attempted / 0 complete / 0 retry | **BLOCKED_LAUNCH_EXIT_1** |
| IAP-RQ-423 | Preserve raw failure truth and forbid downstream threshold action | Analyzer invocations 0, no analysis/draft, no retry/alternate root/threshold action/G0C verdict; zero task processes; complete task products and protected prior evidence retained | **FAIL-CLOSED; not G0C PASS** |
| IAP-RQ-423 | Keep every new log and temporary product repository-local | Full invocation omitted `ROS_LOG_DIR`, failed ROS logging initialization and created `/root/.ros/log/.../launch.log`; independent Standards review 0/0, Spec review 1 blocking / 0 nonblocking | **SPEC BLOCKER; no retry permitted** |

## 2026-08-24 ICRA-052 P4-G0C r3 launch-environment protocol repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Replace both consumed live matrices without changing accepted calibration science | Canonical r3 protocol/registry/lineage registers 15 disjoint `p4-g0c-r3-*` IDs, preserves the complete v2 scientific contract and proposed/null/disabled state, and binds exact ICRA-046 plus ICRA-051 failed identities/states/1-0-0 ledgers/external-log classification | **REGISTERED; synthetic only** |
| IAP-RQ-423 | Make mutable launch paths a runner-owned pre-attempt invariant | Shared validation derives four child-environment directories and eight per-run outputs below the fresh runs root; production runner creates/verifies/propagates them before GPU, launch or attempted-ID mutation and fails with `LAUNCH_ENVIRONMENT_NOT_READY` on missing/outside/relative/`..`/symlink/conflict/unknown evidence | **FAIL-CLOSED; zero live invocation** |
| IAP-RQ-423 | Prevent provenance refresh from concealing environment/output drift | Runner state, run manifest and test-planner manifest carry exact bindings; analyzer covers 12 fields x remove/change/wrong-type = 36 refreshed-provenance adversaries and never emits a draft | **IMPLEMENTED; no threshold action** |
| IAP-RQ-423 | Preserve prior truth and correct ICRA-051 review classification | ICRA-051 state `7c3cafc5...46a7`, external log `f506e556...58e7`, v1/v2 contracts and protected PDF remain byte-identical; Builder docs now record one High Standards plus one High Spec blocker for the external ROS log | **PRESERVED AND CORRECTED** |
| IAP-RQ-423 | Preserve v1/v2 validation while extending r3 dependency evidence | Existing dependency-preflight tests retain v2 protocol/registry/manifest/schema semantics against immutable historical launch bytes; separate v3 complete-closure coverage requires `p4_g0c_dependency_preflight_result_v3`; four absent-caller-key cases prove canonical runner ownership | **SPEC REVIEW BLOCKER REMEDIATED** |
| IAP-RQ-423 | Verify the repair without build, GPU, ROS or qualification | Repository-local focused discovery passes 84/84, full Python discovery 439/439, syntax 9/9, canonical JSON 4/4, fatal-only flake8 and diff checks pass; compact evidence is `results/icra27/icra052/` | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |
## 2026-08-24 ICRA-053 P4-G0C r3 XDG runtime repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Make r3 XDG runtime runner-owned before any live boundary | Five-key shared contract derives `launch_environment/xdg_runtime`; runner uses dirfd/no-follow creation and proves owner, exact `0700`, W/X access and canonical descendant identity before GPU/launch/attempt, overriding absent or malicious caller values | **FAIL-CLOSED; synthetic only** |
| IAP-RQ-423 | Remove the r3 external override while preserving legacy behavior | Production launch conditionally consumes `p4.g0c.child_xdg_runtime_dir` for r3 and retains `/tmp/runtime-root` only for non-r3; runner state plus run/test-planner manifests and analyzer bind exact XDG value/mode | **REPAIRED; zero live invocation** |
| IAP-RQ-423 | Prove registered paths equal the production launch surface | Independent AST inspection enumerates path-valued production environment actions and the exact five-key/eight-output r3 binding; it fails on the ICRA-052 unconditional literal and any new unregistered action or binding sink | **STRUCTURAL PROOF GREEN** |
| IAP-RQ-423 | Keep the structural proof independent of the declared map | Post-review remediation parses runner child/launch arguments and direct writes plus launch runtime/export/log/bag/CSV/manifest sink chains, normalizes them to eight semantic outputs, and filters only path-valued environment actions before comparing with the literal binding | **SPEC BLOCKER REMEDIATED** |
| IAP-RQ-423 | Reject refreshed-provenance semantic drift without draft | Runner covers 10 XDG evidence attacks with zero GPU/launch/attempt; analyzer covers 13 fields x 3 mutations = 39 plus two mode cases, refreshes legitimate hashes and emits no threshold draft | **ADVERSARIAL PASS** |
| IAP-RQ-423 | Correct ICRA-052 review truth and preserve protected history | Builder docs append one High Standards blocker for external test temp and one High Spec blocker for unregistered external XDG; v1/v2, r3 science/lineage, ICRA-051 state/log and PDF bytes remain exact | **CORRECTED AND PRESERVED** |
| IAP-RQ-423 | Verify without external temp, build, GPU, ROS or qualification | Every Python command used ICRA-053 TMPDIR; focused 87/87, launch 16/16, full discovery 442/442, syntax 9/9, canonical JSON 4/4, fatal-only flake8 and diff checks pass; compact evidence is `results/icra27/icra053/` | **READY FOR SUPERVISOR REVIEW; not G0C PASS** |

## 2026-08-24 ICRA-054 hermetic test and mutation-surface closure

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Make launch-context Python verification hermetic by construction | One explicit-root bootstrap owns five canonical environment directories, enforces XDG `0700`, exports before ROS launch imports and supplies an early read-only guard; focused development coverage passes 5/5 and launch contracts pass 11/11 plus golden 16/16 | **IMPLEMENTED; synthetic only** |
| IAP-RQ-423 | Inventory the complete r3 environment and filesystem/process path surface | Fail-closed static classifier inventories four environment actions and 50 production path/mutation records, normalizes all eight outputs, covers 24 mutation primitives and rejects variable/join/list/unresolved/unknown adversaries | **IMPLEMENTED; formal suite incomplete** |
| IAP-RQ-423 | Correct ICRA-053 external-output claims without rewriting history | Builder docs now record eight empty external launch logs from ICRA-053 tests and four from Supervisor rerun; retained files and raw prior evidence are unchanged | **CORRECTED; historical bytes preserved** |
| IAP-RQ-423 | Keep every ICRA-054 output repository-local | `/root/.ros/log` metadata/content inventories compare exactly, but a diagnostic created and prematurely removed two `/tmp/icra054_*_names.txt` files; by the explicit immediate-blocker rule, later clean verification cannot cure this | **BLOCKED_EXTERNAL_TEMP_CREATION** |

## 2026-08-25 ICRA-055 hermetic classifier correction

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Make every authorized Python verification mode hermetic and externally observable | `run_p4_g0c_tests.py` allows only unittest/syntax/fatal-flake8/canonical-JSON below ICRA-055, owns five environment roots, preserves child exit, and compares complete pre/post external inventories; comparator/guard/child-result tests pass 8/8 | **IMPLEMENTED; zero external delta** |
| IAP-RQ-423 | Verify exact r3/legacy environment condition semantics | Classifier requires exact IfCondition + ordered Equals/NotEquals + `LaunchConfiguration("experiment")` + named v3 constant and an exact four-action multiset; nine malformed variants reject | **FAIL-CLOSED; production launch unchanged** |
| IAP-RQ-423 | Deny unclassified filesystem/process-output operations in every reachable scope | Module/sync/async/nested scopes, aliases, explicit os/shutil/pathlib/Path sets, five subprocess helpers and 32 mutation cases are covered; canonical siblings, positional streams, final/dynamic flags, recursive guards and unknown nested/dynamic namespaces reject; exact validation exposes `runs_root` beyond the five-environment/eight-output contract | **FAIL-CLOSED; production contract BLOCKED** |
| IAP-RQ-423 | Complete verification without live or retained-artifact mutation | Focused 111/111, launch 11/11 + 16/16, full Python 466/466, syntax 6/6, fatal-only flake8, canonical JSON 4/4 and diff checks pass; external 17,759-entry before/after hashes equal `82b029de...eee9`; compact evidence is ICRA-055 | **BLOCKED_PRODUCTION_SURFACE_EXCEEDS_EIGHT_OUTPUT_CONTRACT** |

## 2026-08-25 ICRA-056 two-layer container contract and r3 live task

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Model the corrected runner-owned container without adding a ninth launch output | `production_surface()` proves exactly one canonical `runs_root`, fresh-root guards, exact runner-state child and canonical preflight/environment/run descendants; five environment and eight output leaves remain separate and exact | **PHASE A IMPLEMENTED** |
| IAP-RQ-423 | Fail closed on every container/output contract drift | Focused regressions reject missing, duplicate, renamed and second containers, wrong state child, parent/sibling escape, changed ownership AST and an extra output semantic | **ADVERSARIAL PASS** |
| IAP-RQ-423 | Keep all synthetic verification repository-local before live work | ICRA-056 launcher owns five environment roots; bootstrap 8/8, classifier 18/18, focused 113/113, launch 11/11 + 16/16, full Python 468/468, syntax 6/6, flake8, canonical JSON 4/4 and diff checks pass with identical 17,759-entry external inventories | **PHASE A PASS; BUILD/LIVE PENDING** |

## 2026-08-25 ICRA-056 dependency provenance stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Build one complete fresh CUDA runtime before dependency/GPU/live | Sole merged non-symlink build exits 0 with 17/17 packages; static closure proves CUDA ON, 17 unique indexes, six ordinary ELF libraries, loadable GPU odometry library, zero unresolved/historical linkage and exact frozen hashes | **BUILD + STATIC CLOSURE PASS** |
| IAP-RQ-423 | Make dependency provenance exact before GPU or ROS | Sole standalone gate reports complete 18/13/1/14/6 validation and zero GPU/launch/attempt, but state `0d305191...32361` binds `manifest_path` to installed `libsub_mapping.so`; production reuses the manifest local variable during runtime-library validation | **BLOCKED_DEPENDENCY_MANIFEST_PATH_BINDING** |
| IAP-RQ-423 | Stop without consuming evidence after an output-binding failure | Full runner, GPU preflight, r3 attempts, retries and analyzer remain zero; runs/analysis/draft are absent, processes are zero, protected PDF and read-only `gnss_comm` identities remain exact | **FAIL-CLOSED; SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-057 dependency provenance repair

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Bind dependency success provenance to the selected manifest | `validate_runtime_dependencies()` owns immutable `resolved_manifest_path` and distinct artifact-path locals; success returns the same canonical manifest path plus protocol-bound SHA, prefixes and exact 18/13/1/14/6 counts | **PHASE A IMPLEMENTED** |
| IAP-RQ-423 | Preserve validation behavior while preventing last-artifact corruption | Focused 12/12 covers reordered config and alternate terminal ELF content plus wrong hash, missing artifact, prefix alias, artifact escape, manifest symlink loop and historical-prefix rejection | **REGRESSION PASS** |
| IAP-RQ-423 | Keep repair verification repository-local before adoption/live | Bootstrap 8/8, classifier 18/18, focused 116/116, launch 11/11 + 16/16, full discovery 471/471, syntax/flake8/canonical/diff all pass with empty 17,759-entry external delta | **PHASE A PASS; ADOPTION/LIVE PENDING** |

## 2026-08-25 ICRA-057 credential-output stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Prevent environment/credential values from entering ICRA-057 evidence | A read-only metadata search mistakenly included a retained event log whose tool output exposed serialized environment values; no value/name was copied into repository evidence and scoped tracked/compact scan is clean | **BLOCKED_CREDENTIAL_VALUE_OUTPUT_EXPOSURE** |
| IAP-RQ-423 | Stop before one-shot live boundaries after output violation | Adopted closure verdict is incomplete; dependency/GPU/full runner/r3/analyzer invocations are all zero, roots absent, processes zero, historical artifacts and protected identities retained | **FAIL-CLOSED; SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-058 direct r3 live continuation

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Reuse only the accepted frozen CUDA product before live | Read-only ICRA-056 closure has 17 exact package indexes, Release/CUDA ON/tests OFF, 14 bound configs, six ordinary ELF libraries, loadable GPU odometry SHA `0848175b...5c7cf`, zero unresolved/historical linkage and exact frozen input hashes | **ADOPTED CLOSURE PASS; no build** |
| IAP-RQ-423 | Require corrected provenance before GPU or ROS | Sole fresh ICRA-058 dependency preflight exits 0/PASS with canonical manifest/SHA, exact 18/13/1/14/6 counts and zero downstream activity; state SHA is `db8f0c1c...bdb46` | **DEPENDENCY PREFLIGHT PASS** |
| IAP-RQ-423 | Execute each r3 identity at most once behind mandatory GPU proof | Sole full runner passes `nvidia-smi`, `cuInit(0)` and one device, consumes only `p4-g0c-r3-seed211-rep01`, and records 1 attempted / 0 completed / 1 launch / 0 retry; required processes survive the 90-second interval and controlled shutdown | **ONE-SHOT BOUNDARY PRESERVED** |
| IAP-RQ-423 | Reject malformed scientific evidence without threshold action | First decision row has empty `snapshot_frame`, producing `malformed P4 decision CSV: ...:typed_identity`; runner exits 2/FAILED, analyzer/draft/action remain zero/absent, external logs and protected identities remain exact | **BLOCKED_MALFORMED_P4_DECISION_CSV_TYPED_IDENTITY** |

## 2026-08-25 ICRA-059 r4 P0 binding — Phase A

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Bind the already qualified P0 covariance-growth profile exactly | v4 freezes typed sigma `0.01`, profile `legacy_iap_rq320_baseline_v1` and exact ICRA-035 config/analyzer hashes; loader and runner reject missing, wrong-type, non-finite, non-exact or drifted evidence before GPU/ROS | **PHASE A PASS; no recalibration** |
| IAP-RQ-423 | Replace consumed r3 evidence without reuse or changed P4 science | v4 registers 15 disjoint ordered r4 IDs, binds v3 protocol/registry plus ICRA-058 state `9f6241a2…b62ddf` and compact `6a804b26…a32`, and excludes r3 namespace/artifacts | **R4 REGISTERED; no identity attempted** |
| IAP-RQ-423 | Materialize exact config and keep invalid snapshots fail-closed | v4 launch/dependency/runner/analyzer manifests carry exact P0 values; `snapshot_unavailable` with invalid generation/stamp/frame rejects as `p0_riskgrid_snapshot` with producer reason | **ADVERSARIAL PASS** |
| IAP-RQ-423 | Complete repository-local Phase-A verification before qualification | Hermetic focused 121/121 and full discovery pass; syntax, fatal-only flake8, canonical JSON and diff checks pass; all external inventory deltas are empty | **PHASE A READY FOR COMMIT/PUSH** |

## 2026-08-25 ICRA-059 readiness fail-closed result

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Prove the exact accepted P0 profile reaches runtime | Readiness manifests carry exact typed `0.01` and `legacy_iap_rq320_baseline_v1`; P0 reaches `ready=1`, generation 19 and finite stamps | **P0 PRODUCER READY AT BOUNDARY** |
| IAP-RQ-423 | Require a P4 request to receive a positive RiskGrid snapshot before any r4 identity | Nonregistered 20-second readiness produces 15 P4 rows, all generation zero / `snapshot_unavailable`; positive snapshot rows = 0 | **BLOCKED_R4_READINESS_NO_P4_POSITIVE_SNAPSHOT** |
| IAP-RQ-423 | Preserve one-shot calibration identities after readiness failure | Phase-C dependency/full runner/analyzer invocations = 0; r4 attempts/completions/retries = 0/0/0; no threshold action or G0C claim | **FAIL-CLOSED; SUPERVISOR REVIEW** |

## 2026-08-25 ICRA-060 deterministic RiskGrid admission and readiness

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Preserve the qualified P0 covariance-growth contract while gating P4 planning | v4/r4 alone enables the new barrier; exact sigma `0.01`, profile `legacy_iap_rq320_baseline_v1`, producer behavior and v1-v3 bytes remain unchanged | **PRESERVED** |
| IAP-RQ-423 | Admit planning only from one valid typed RiskGrid snapshot | default-false admission requires ownership, ready/non-stale health, positive generation, finite-positive stamp and nonempty frame; waiting is throttled and release stamp/generation/defer count are recorded once | **IMPLEMENTED; C++ 3/3 PASS** |
| IAP-RQ-423 | Prove repository-local CUDA/readiness closure before registered r4 work | fresh final attempt 04 builds 17/17 sequential merged non-symlink Release/CUDA packages; static closure, GPU preflight, required-process monitoring and hermetic 477/477 Python tests pass | **DEVELOPMENTAL CLOSURE PASS** |
| IAP-RQ-423 | Require at least one positive post-release P4 row before formal dependency/live | final disjoint readiness releases at generation 1, binds 9,600 positive available planning contexts and emits zero pre-release rows, but frozen collision scanning returns `OPEN_ENDED_COLLISION` before P4 guide collection and emits zero post-release rows | **BLOCKED_P4_OPEN_ENDED_COLLISION_BEFORE_GUIDE_REQUEST** |
| IAP-RQ-423 | Preserve the one-shot boundary and forbid unsupported remediation | formal dependency/full runner/analyzer = 0/0/0; registered r4 attempts/completions/retries = 0/0/0; no forbidden `bspline_opt`, scenario, science or threshold change was made | **FAIL-CLOSED; SUPERVISOR REVIEW** |

## 2026-08-25 ICRA-061 r5 closed fixture and pre-identity stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Preserve the qualified P0 profile while versioning only the collision fixture | v5 retains sigma `0.01`, `legacy_iap_rq320_baseline_v1`, seeds/repetitions/formulas/thresholds; fixture v2 changes only x to `[-9,-7]`; v1-v4 are byte-identical | **PRESERVED** |
| IAP-RQ-423 | Prove exact closed-segment eligibility before live identity use | production scanner returns r5 `CLOSED_SEGMENTS` with free endpoints/tail and r4 `OPEN_ENDED_COLLISION`; installed preflight materializes exact enabled/x/y/z, start, 7.5 m horizon and 0.4 m spacing | **DETERMINISTIC PREFLIGHT PASS** |
| IAP-RQ-423 | Close admission/evidence gaps and prove the final CUDA closure | ICRA-060 prose/ledger corrected; focused admission C++ 4/4 and live barrier evidence show 848 deferrals, release once, zero pre-release P4 rows; fresh build/static closure and Python 485/485 pass | **ENGINEERING CLOSURE PASS** |
| IAP-RQ-423 | Require exact standalone dependencies before registered execution | sole v5 dependency invocation passes 18 packages, 13 executables, one component, 14 configs and six libraries from ICRA-061/Jazzy; GPU/launch/attempt/retry are all zero | **DEPENDENCY PREFLIGHT PASS** |
| IAP-RQ-423 | Do not consume one-shot identities when readiness proves deterministic analyzer rejection | all 12 positive-identity closed-segment rows are `incomplete_profile`; original validity is 0-17/200 and risk validity 103-147/200 despite stable generation 17/global validity 0.984; immutable analyzer requires both 200/200 and retains failed rows | **BLOCKED BEFORE REGISTERED IDENTITY** |
| IAP-RQ-423 | Preserve scientific and downstream fail-closed boundaries | full runner/analyzer = 0/0; r5 attempts/completions/retries = 0/0/0; no row exclusion, science/profile/fixture/threshold change, draft, G0C claim, G0D or P5 execution | **SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-062 worker correction, trace and typed stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Run accepted predictor parallelism, not the legacy outer batch label | v5/r5 freezes typed worker 4; runner rejects absent/bool/string/non-4 values; live manifest is requested/effective `4/4`, outer batch remains 1 | **CORRECTED AND LIVE-PROVEN** |
| IAP-RQ-423 | Remove synthetic admission confidence without weakening runtime | Deleted production friend/callback and fake counter test; direct admission passes three focused unit cases and live release-once/positive-snapshot evidence | **IMPLEMENTED** |
| IAP-RQ-423 | Explain every incomplete equal-arc sample without changing decisions | default-off nonregistered trace records query/layer/corner/exact-weight/source/occupancy detail; classifier proves 12 identities x 200 samples x two arms; noninterference C++ passes | **DIAGNOSTIC CLOSURE PASS** |
| IAP-RQ-423 | Enter bounded support repair only under its exact predicate | totals are occupied skip 3,040, time support 10, every other category 0; time support remains explicitly fail-closed, so no Section-6 policy or r6 exists | **BLOCKED_R5_READINESS_TIME_SUPPORT** |
| IAP-RQ-423 | Preserve one-shot and downstream boundaries | registered r5/r6 attempts, completions and retries are zero; runner/analyzer/draft/action/G0C/G0D/P5 are absent | **SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-063 r6 support and typed stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Extend only the observed temporal tail and preserve integrity semantics | r6 fixes seven horizons through exactly 3.0 s; typed conservative policy supports only positive-weight occupied cost with finite `unknown_cost`; health/PL and other invalid sources remain false/fail-closed | **IMPLEMENTED; FOCUSED TESTS PASS** |
| IAP-RQ-423 | Keep P4 consumers and identities exact | original/risk guide and A* share one policy; v6 registers 15 disjoint r6 IDs while r5 is unconsumed and v1-v5 bytes remain unchanged | **OFFLINE CONTRACT PASS** |
| IAP-RQ-423 | Require fresh CUDA and preflight before readiness | 17/17 merged nonsymlink Release/CUDA build, six ELF libraries, zero historical linkage and mandatory GPU device count 1 pass | **BUILD + GPU PASS** |
| IAP-RQ-423 | Stop on the sole failed readiness without identity consumption | dependency 18/13/1/14/6 and P0 profile pass, but launch rejects effective config before ROS; required processes never start; readiness is not retried | **BLOCKED_R6_READINESS_PROTOCOL_EFFECTIVE_CONFIG_MISMATCH** |
| IAP-RQ-423 | Preserve registered one-shot and authority boundaries | r6 attempts/completions/retries, full runner and analyzer are 0/0/0/0/0; no draft, threshold action, G0C/G0D/P5 claim | **SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-063 corrected readiness and freeze

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Materialize the exact r6 temporal profile | v6 launch parses the protocol horizon CSV to `[0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]`; live manifest proves requested/effective worker `4/4` and preserves outer batch 1 | **LIVE-PROVEN** |
| IAP-RQ-423 | Repair correctable pre-identity orchestration without consuming readiness | Initial typed-list mismatch and missing evidence parent both reject before ROS; neither starts a required process or registered identity | **CORRECTED IN-TASK** |
| IAP-RQ-423 | Prove one true nonregistered r6 readiness | mandatory GPU proof remains PASS; admission releases once; 13 positive-snapshot closed-segment decisions are metrics-only with exact 200/200 arms and zero invalid categories; required processes are healthy with controlled shutdown | **READINESS PASS** |
| IAP-RQ-423 | Freeze before registered work | final fresh build passes 17/17 merged non-symlink Release/CUDA, six ELF, zero historical linkage and source/install equality; runner/analyzer/registered attempts remain zero at freeze | **FROZEN; MATRIX PENDING** |

## 2026-08-25 ICRA-063 registered one-shot terminal stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Require exact standalone runtime closure before identity use | final install plus Jazzy passes 18 packages, 13 executables, one component, 14 configs and six libraries with zero GPU/launch/attempt | **DEPENDENCY PREFLIGHT PASS** |
| IAP-RQ-320, IAP-RQ-322, IAP-RQ-423 | Consume registered identities only through the frozen r6 runner | sole runner invocation passes GPU and launches only `p4-g0c-r6-seed211-rep01`; raw decision evidence has 13 positive snapshots, metrics-only, 200/200 arms and zero invalid counts | **1 ATTEMPTED; SCIENTIFIC ROWS PRODUCED** |
| IAP-RQ-423 | Fail closed on non-ordinary raw artifact topology | finalization rejects producer-created `runtime/iap_logs/latest` symlink; artifact inventory is absent and the identity is not accepted complete | **BLOCKED_POST_IDENTITY_RUN_ARTIFACT_SYMLINK** |
| IAP-RQ-423 | Preserve terminal one-shot authority | completed/retry/remaining launches are 0/0/0; analyzer, draft, threshold action, G0C/G0D/P5 claim and P5 run remain absent | **TERMINAL; SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-064 exact recovery — pre-live freeze

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322 | Preserve the scientifically valid consumed first identity byte-for-byte | Prewrite lstat/content inventory plus exact runner/decision/run-manifest/test-manifest/stdout hashes; validation-only rechecks 13 positive-snapshot closed-segment producer `metrics_only` rows with 200/200 arms and zero invalid counts | **RETAINED SCIENCE PASS; NO WRITE** |
| IAP-RQ-423 | Admit only the two exact producer aliases without general symlink relaxation | r6 inventory v2 binds the literal run-local target; analyzer binds the exact task-local ROS alias and target contents; escape, alternate, dot, nested, dangling, chain/loop, type and replacement adversaries fail closed | **CONTRACT + ADVERSARIAL PASS** |
| IAP-RQ-423 | Recover without relaunching or rewriting consumed ID 1 | Typed recovery accepts only the exact terminal root/hash/reason and canonical ICRA-063 final-install/Jazzy dependency closure before any write; it preserves canonical original state, creates/validates only the missing inventory, records zero recovery launch/retry and resumes at ordered ID 2 | **VALIDATION-ONLY ADOPTION ELIGIBLE** |
| IAP-RQ-320, IAP-RQ-322, IAP-RQ-423 | Preserve hard occupancy independently of finite conservative cost support | Real risk-aware A* search routes around a finite occupied barrier, records occupied rejection and returns no occupied path point | **C++ SEARCH PROOF PASS** |
| IAP-RQ-423 | Freeze tested recovery code before the one live continuation | Focused suites plus 512/512 hermetic discovery pass with zero external ROS-log delta; retained state hash remains exact and recovery writes/launches/retries remain 0/0/0 | **PRE-LIVE COMMIT/PUSH READY** |
| IAP-RQ-423 | Prove validation-only is globally nonmutating over retained recovery input | Post-validation lstat/content recomputation matches all 113 first-run and 504 shared launch-environment entries from the prewrite inventory | **FULL-SCOPE EQUALITY PASS** |

## 2026-08-25 ICRA-064 continuation and terminal analysis

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320, IAP-RQ-322, IAP-RQ-423 | Adopt consumed ID 1 without changing science or retrying identity | Recovery inventory SHA `14fbc467...f34a`; decisions/run-manifest/test-manifest/stdout retain exact Supervisor hashes; recovery launch/retry = 0/0 | **OFFLINE ADOPTION PASS** |
| IAP-RQ-423 | Continue only ordered IDs 2--15 behind exact final-install and fresh GPU proof | Final runner SHA `475004c6...1dd1`; 15 unique attempted/completed, 15 launches, zero retries/exclusions, sessions `1 + 14`, GPU preflights `1 + 1`, required-process and artifact finalization complete | **RUNNER COMPLETE** |
| IAP-RQ-320, IAP-RQ-322 | Retain every scientific row and require registered calibration eligibility | Sole analyzer retains denominator 192 and complete count 136, but records 56 `noise_floor` failures | **ANALYZER REJECTED** |
| IAP-RQ-423 | Require exact typed recovery provenance through analysis | Recovery record preserves the pre-continuation shared ROS alias literal; producer advances `latest` during later runs, so sole analyzer reports `runner_state_recovery` rather than silently rewriting provenance | **FAIL-CLOSED PROVENANCE STOP** |
| IAP-RQ-423 | Preserve downstream authority after rejection | Analyzer invocations = 1; draft/action/G0C/G0D/P5 = 0/0/0/0/0; no identity or analyzer retry | **SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-065 analyzer correction validation stop

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Bind immutable recovery-time provenance without weakening final alias safety | Exact retained inventory schema/root/SHA and target subtree are validated; historical A may differ from independently safe final B; tamper, escape, missing, chain and replacement tests fail closed | **OFFLINE IMPLEMENTED; TESTS PASS** |
| IAP-RQ-320, IAP-RQ-322 | Apply the frozen floor to Type-7 aggregate improvements while retaining complete rows | Individual floor-level values no longer exclude structurally complete rows; typed statistics and exact failed gates distinguish `SCIENTIFIC_NO_GO` from technical `REJECTED` | **OFFLINE IMPLEMENTED; TESTS PASS** |
| IAP-RQ-423 | Validate unchanged r6 evidence before the single authoritative analysis | Frozen inputs 103/103 exact; read-only preflight has 0 technical failures, 15/15/15 runs, 192/192 decisions, max Q10 0, but mean Q10 `0.000020000000000131024` differs from frozen expected `0.000304` | **BLOCKED_ICRA065_VALIDATION_Q10_MEAN_MISMATCH** |
| IAP-RQ-423 | Preserve downstream authority on validation mismatch | Authoritative analyzer/output replacement = 0; old analysis SHA remains `f584fc51...d7391`; draft/registry/threshold/G0C/G0D/P5 actions remain absent | **FAIL-CLOSED; SUPERVISOR REVIEW REQUIRED** |

## 2026-08-25 ICRA-066 authoritative offline P4-G0C result

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-423 | Bind reviewed implementation and unchanged r6 evidence before replacement | Handoff parent/diff, analyzer/test SHA, runner state, recovery inventory, old/preserved analysis and ICRA-065 103-file arrays all match; draft absent | **FROZEN INPUT PASS** |
| IAP-RQ-320, IAP-RQ-322 | Emit aggregate Type-7 calibration statistics without row exclusion | Sole authoritative call retains 192/192 decisions; mean Q10 `0.000020000000000131024` passes floor and max Q10 `0` fails | **ANALYZER TECHNICAL PASS; SCIENTIFIC NO-GO** |
| IAP-RQ-423 | Preserve one-call and downstream authority | Analyzer calls = 1 with expected exit 2; output SHA `572e5d79...a9c1e`; technical failures 0, runs 15/15/15, sole max-improvement failed gate, no draft/registry/application/G0D/P5 action | **AUTHORITATIVE SCIENTIFIC_NO_GO; SUPERVISOR REVIEW** |

## 2026-08-25 ICRA-067 P0+P5 contingency activation

| Req ID | Requirement/evidence seam | Implementation and retained evidence | Status |
|---|---|---|---|
| IAP-RQ-320 | Preserve reviewed P0 and frozen P5 decisions | Contract binds P0 worker 4, sigma 0.01, baseline profile, horizons/ROI/resolution/refresh and every existing P5 threshold/query value | **PROFILE CONTRACT PASS; NO LIVE CLAIM** |
| IAP-RQ-421, IAP-RQ-422 | Isolate the conference route | `icra_p0_p5` enables P0 plus P5 final/runtime/evidence; high/lower P1/P2/P3/P4, distinctive/fanout/debug/metrics/trace/viz/application paths are exact false/empty and contradictory overrides reject | **FOCUSED TEST PASS** |
| IAP-RQ-423 | Pre-register three prospective cases and fail closed | SAFE_NORMAL, P5-7 FINAL_REJECT and P5-6 RUNTIME_FAIL bind case/run/fixture/process/topic/P0/event identities and canonical raw-row hashes; adversarial synthetic tests reject every required failure class | **VALIDATION-ONLY PASS; QUALIFICATION FALSE** |
| IAP-RQ-423 | Preserve P4 authority and repository-local execution | No P4 files or scientific evidence changed; no GPU/ROS/live/build ran. Full 525-test discovery is blocked only by four immutable P4-v6 checks that require the old launch SHA | **BLOCKED_ICRA067_FROZEN_P4_LAUNCH_HASH_CONFLICT** |
