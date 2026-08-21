# ICRA Supervisor Log

## 2026-08-21 — ICRA-015 review, phase-3B closure and ICRA-016 phase-4A authorization

### Review identity and independent verification

- Fixed review base: `eb66c078a97d00360e542bfd28bea897a66510e6`.
- Reviewed HEAD: `eb1cb67889960d995f7ca8dab318da649af82cb4`.
- Reviewed commits: `4d46187` and `eb1cb67`; both carry applicable requirement IDs.
  `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the seven ICRA-015 allowlisted files and passes
  `git diff --check`. The protected PDF remains the sole untracked file at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Supervisor independently rebuilt the current root rolling/Predictor/RiskGrid/occupancy/snapshot/
  conversion targets, the complete P0/Adapter/P1/P2/P3/P5 consumer set and P4 A*. All active suites
  pass: 271/271 GTests plus 2/2 registered retained-profile tests.
- Ten checked planner/test consumers resolve the current repository-local
  `results/icra27/icra015/build_iap/libiap.so`, SHA-256
  `7be09389420ca1b2a9e9653734cdb45e511cacfa64e0ca952d34105a7f4c2358`.
- The retained ICRA-011 JSON remains byte-identical at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`; the canonical ICRA-014
  diagnostic remains read-only at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`. No main flow, ROS launch,
  smoke, qualification, benchmark, analyzer or GPU preflight ran, and no task process remains.

### Standards axis

- **PASS, zero hard findings.** Exact allowlist, ownership, requirement IDs, synchronized
  `CHANGES`/`TRACEABILITY`/`DEV_LOG`, repository-local verification commands and two-commit handoff
  conform to `AGENTS.md` and ICRA-015.
- One non-blocking design judgement remains: rolling and production runtime independently spell the
  same three-boolean active-source projection. The duplication is small and did not justify widening
  the repair, but Phase-4A should centralize it so invalidation and publication validation cannot
  drift. Worst Standards issue: no hard issue; duplicated internal source projection is the sole
  judgement.

### Spec axis

- **PASS, zero findings.** GNSS, LiDAR and legacy-current identity now projects only active fields
  consumed by spatial science. Inactive-source changes do not erase reuse; active owner/consumed
  field changes still invalidate; non-finite active evidence remains conservative.
- `current.stamp/valid` remain per-horizon validation/freshness inputs. Stationary production refresh
  updates current time/prior generation, retains spatial advice without restamping it, performs all
  horizon work and matches forced-fresh science.
- Legacy LiDAR fields again report same-call populated-cache work (`1/1/(H-1)` for fresh work and
  `0/0/0` for cross-refresh-only retention); additive rolling diagnostics report retained work.
  ICRA-014/015 reproduction commands are present and all accepted ring, movement, rollback, worker
  and scientific-equivalence behavior remains green. Worst Spec issue: none.

### Disposition and next task

- Verdict: `ICRA015_PASS_PHASE3B_CLOSED`. The ICRA-014 findings are closed, and phase 3B/phase 3 are
  accepted as implementation stages. This does not qualify P0 or Gate-0B; P4 remains
  `NOT_QUALIFIED`, and P5 remains implemented but unqualified.
- Unique task: `ICRA-016 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-016 is Phase-4A development: centralize active-source projection, capture atomic monotonic
  source provenance, add per-slot original timestamps, bounded GNSS/legacy-current TTL retention and
  a successful-full-refresh watchdog. All policies default disabled; tests use synthetic values.
- Occupancy delta/reverse-ray work is deferred to Phase-4B. Production activation/calibration,
  worker/default tuning, main-flow smoke, qualification, GPU work and P1-P5 changes remain forbidden.

## 2026-08-21 — ICRA-014 review and ICRA-015 narrow repair authorization

### Review identity and independent verification

- Fixed review base: `597f3b79a098842589b340e1919234c4182cee9d`.
- Reviewed HEAD: `363be82694797c3a499c1e26dd08ed7100e76aa0`.
- Reviewed commits: `8b0c594` and `363be82`; the implementation commit carries all applicable
  requirement IDs. `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the fifteen ICRA-014 allowlisted files and passes
  `git diff --check`. The protected PDF remains the sole untracked file at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Supervisor independently rebuilt the current root rolling/Predictor/RiskGrid/occupancy/snapshot/
  conversion targets, the complete P0/Adapter/P1/P2/P3/P5 consumer set and P4 A*. The fifteen
  authorized GTest suites pass 263/263 executed tests; the retained ICRA-011 profile passes 2/2.
  An initial planning-context invocation loaded the workspace's stale `plan_env` and failed at the
  dynamic loader before tests; adding the prescribed ICRA-014 `build_plan_env` path made the full
  26/26 suite pass. This was an environment-path error, not a product assertion failure.
- Seven linked planner consumers resolve the current repository-local
  `results/icra27/icra014/build_iap/libiap.so`, SHA-256
  `bca1648834fffe32a6d88adcb8fd88890bfddeb54ef10dee9cc2b9c4f7663977`.
- The canonical ICRA-014 diagnostic remains read-only at SHA-256
  `44f47b23137d17f4b0cbc81af6827156865bdecb36089bf53f770960a2fb963d`; the retained ICRA-011
  JSON remains `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
  No main flow, ROS launch, smoke, qualification, benchmark, analyzer or GPU preflight ran.

### Standards axis

- **FAIL: one adjudicated hard finding; one design judgement retained.** The ICRA-014
  `docs/CHANGES.md` entry contains result prose but no executable rolling/P0/canonical-read-only
  reproduction command. `AGENTS.md` requires a command in CHANGES or README; `DEV_LOG.md` does not
  substitute for that location.
- The Standards reviewer also classified public `beginRefresh/commitRefresh/abortRefresh` as a hard
  Interface leak. Supervisor does not adopt that classification: `NEXT_TASK.md` expressly allowed
  the new rolling header and a PIMPL/friend/internal session Seam, the protocol was not added to
  `RiskPredictionProvider` or fake providers, and P4/P5 still see only `RiskGridMap`/immutable
  snapshot. The explicit transaction remains a non-blocking design judgement; ICRA-015 must not
  widen it further.
- Repeated shared-owner equality helpers are a minor duplication judgement, not justification for a
  repair-scope abstraction. Standards count after adjudication: one hard finding and two
  non-blocking judgements. Worst Standards issue: missing reproduction commands.

### Spec axis

- **FAIL: one P0 and one P1 finding.** The accepted core is substantial: fixed-capacity dense ring,
  signed world-key validation, transactional candidate rollback, exact first/stationary/`+1 x`
  `12800/0/320` spatial recomputes, `0/12800/12480` retained positions and 76,800 horizon fusion/
  materializations. Fresh-full scientific equivalence, worker determinism and retained suites pass.
- **P0:** spatial identity is compared unconditionally across disabled sources. GNSS epoch/occupancy
  affect `LidarOnly`; LiDAR owners/current affect `GnssOnly`; and `current.stamp/valid`, which belong
  to per-horizon freshness/validation, affect Fusion spatial identity. The current tests change only
  the prior while holding these values fixed, so they do not expose that normal production updates
  conservatively erase the intended ring reuse.
- **P1:** a slot retained from a previous refresh increments legacy `lidar_cache_hits` for all
  horizons while `unique_positions/lidar_evaluations` stay zero. That silently changes the frozen
  phase-2 call-local populated-cache contract from `1/1/(H-1)` or `0/0/0` to `0/0/H`; additive
  rolling counters, not legacy fields, must represent cross-refresh reuse.
- Spec count: two findings. Worst Spec issue: the P0 identity projection defect defeats the central
  production optimization under irrelevant or freshness-only source changes.

### Disposition and next task

- Verdict: `ICRA014_REQUEST_CHANGES`. Phase 3B, phase 3 and Gate-0B remain open. P4 remains
  `NOT_QUALIFIED`; P5 remains implemented but unqualified.
- Unique task: `ICRA-015 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-015 is a narrow review repair. It projects GNSS/LiDAR cache identity by active source mode and
  fields actually consumed by spatial science, keeps `current.stamp/valid` in per-horizon validation,
  restores truthful phase-2 legacy LiDAR diagnostic semantics and adds the missing executable
  reproduction commands. It must preserve the accepted ring, transaction, movement counts,
  full-horizon work and scientific equivalence.
- Phase-4 versions/TTL/occupancy delta/watchdog, CPU calibration/scaling, main-flow smoke,
  qualification, GPU work and P1-P5 changes remain forbidden. Phase 4 may be authorized only after a
  separate Supervisor review closes ICRA-015.

## 2026-08-21 — ICRA-013 review, phase-3A closure and ICRA-014 phase-3B authorization

### Review identity and independent verification

- Fixed review base: `61376de73544fbe9afb0a26103e19c0e5ace6ea1`.
- Reviewed HEAD: `ac5bda07cb61ba48aebd5e7e77845a67baa0d39b`.
- Reviewed commits: `86b926b` and `ac5bda0`; both carry applicable requirement IDs.
  `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the six ICRA-013 allowlisted files and passes
  `git diff --check`. The only untracked file remains
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Supervisor independently rebuilt current root, plan-env, P1, P4 and plan-manage targets. With the
  prescribed environment, all seven P1/P2/P3/planning-context/P4/P5/P0 consumers resolve
  `libiap.so` to the current ICRA-013 repository-local build.
- Complete root suites passed: risk grid 43/43, Predictor 45/45, local occupancy 6/6, PI adapter
  11/11 and unified risk grid 11/11.
- Retained/downstream suites passed: frozen occupancy epoch 2/2, P1 integrity cost 39/39, P2 ranking
  6/6, P3 bias 9/9, planning context 26/26, P4 risk A* 4/4, P5 runtime gate 33/33, P0 occupancy
  Adapter 3/3 and P0 runtime 48/48. Total: **286/286**.
- The retained ICRA-011 JSON remains byte-identical at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
  No main flow, ROS launch, smoke, qualification, benchmark, analyzer or GPU preflight ran.

### Standards axis

- **PASS, zero findings.** The six modified files match the exact allowlist; requirement IDs,
  synchronized `CHANGES`/`TRACEABILITY`/`DEV_LOG`, repository-local verification and two-commit
  handoff conform to `AGENTS.md`.
- The fixed lattice does not change Predictor/integrity/planning science, add ring/cache behavior or
  cross repository scope. All Fowler smell-baseline categories were checked; no reportable judgement
  smell was introduced. Worst Standards issue: none.

### Spec axis

- **PASS, zero findings.** Finite anchor, integer world/lower keys, mathematical negative floor,
  frozen even-side rule, stationary/sub-voxel stability and exact one/multi-cell crossing conform.
- Proposed geometry stays local until complete publication; provider and occupancy/prior failures
  retain generation, origin and every ordered voxel. Configure resets the generation, and
  configuration epoch plus serialized refresh writers prevent stale concurrent publication and
  duplicate generation IDs.
- Full provider dispatch and immutable snapshot consumer semantics remain intact. No ring, cache,
  TTL/delta, performance claim, runtime behavior or P1-P5 scope entered the task. Worst Spec issue:
  none.

### Disposition and next task

- Verdict: `ICRA013_PASS_PHASE3A_CLOSED`. The fixed-lattice and atomic-geometry foundation is
  accepted. This does not close phase 3 or qualify Gate-0B; P4 remains `NOT_QUALIFIED` and P5 remains
  implemented but unqualified.
- Unique task: `ICRA-014 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-014 adds a dense fixed-capacity ring for exact-identity GNSS/LiDAR `SpatialAdvisory` reuse.
  It hides ring state behind the production P0 provider Module, validates every slot by world key,
  stages ring changes transactionally and preserves the existing P4/P5 snapshot Interface.
- All 76,800 horizon results still execute freshness validation, covariance growth, fusion and
  materialization. Source identity changes conservatively force full spatial invalidation; TTL,
  occupancy delta, watchdog and finer invalidation remain phase 4.
- Calibration, main-flow smoke, qualification, GPU work and P4 remain forbidden.

## 2026-08-21 — ICRA-012 review, phase-2 closure and ICRA-013 phase-3A authorization

### Review identity and independent verification

- Fixed review base: `3fc24b98f8227dc4764a7daa8fb09ce9cb34876e`.
- Reviewed HEAD: `f9e5c68a1f01738c7c93d6e81b482783e5f8c5ec`.
- Reviewed commits: `4deb136` and `f9e5c68`; both carry applicable requirement IDs.
  `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the six ICRA-012 allowlisted files and passes
  `git diff --check`. The only untracked file remains
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Supervisor rebuilt the affected root, plan-env and plan-manage targets. Runtime linkage with the
  prescribed environment resolves product code to the current ICRA-012 `libiap.so`; only retained
  generated ROS typesupport comes from the repository-local ICRA-009 facade.
- The five exact Predictor regressions passed 5/5, the three exact production runtime regressions
  passed 3/3, and the Python profile contract passed 2/2.
- All six retained suites passed 6/6, 45/45, 35/35, 2/2, 3/3 and 48/48, for **139/139**.
  The retained ICRA-011 profile JSON remains byte-identical at SHA-256
  `778abd22158805c41150b4eeed9c37a3f660237a0bb0599e9a567e3533c7b32c`.
  No main flow, ROS launch, smoke, qualification, benchmark, analyzer or GPU preflight ran.

### Standards axis

- **PASS, zero findings.** File ownership/scope, exact allowlist, applicable requirement IDs,
  synchronized `CHANGES`/`TRACEABILITY`/`DEV_LOG`, repository-local verification and handoff conform.
- The ICRA-011 `docs/CHANGES.md` entry now contains executable Predictor, production runtime,
  offline-profile and Python evidence-contract commands in the required location.
- No reportable judgement smell was introduced by the bounded counter repair. Worst Standards
  issue: none.

### Spec axis

- **PASS, zero findings.** GNSS-only six-horizon work reports generalized spatial `1/5`, actual
  GNSS/LiDAR/fusion `1/0/6`, legacy LiDAR `0/0/0`, and scalar-equivalent ordered results.
- LidarOnly preserves generalized `1/5` and legacy `1/1/5`; Fusion retains the accepted two-position
  legacy shape. Non-cacheable LiDAR increments actual invocation without fabricating a populated
  legacy cache. Valid-then-early-invalid distinguishes lookup hit from actual reuse, and invalid-first
  does not poison the cache.
- Production workers 1/2/4 assert zero GNSS-only legacy counters while preserving nonzero and
  scientifically identical generalized spatial/GNSS/fusion counts.
- The private call-local `SpatialAdvisory`, coherent key, per-horizon covariance growth/fusion,
  failure retention, public Interfaces and canonical profile evidence remain unchanged. No phase-3,
  calibration, GPU or P1-P5 expansion entered the changeset. Worst Spec issue: none.

### Disposition and next task

- Verdict: `ICRA012_PASS_PHASE2_CLOSED`. The two ICRA-011 findings are closed and P0 phase 2 is
  accepted. This is an implementation-stage verdict, not Gate-0B qualification; P4 remains
  `NOT_QUALIFIED` and P5 remains implemented but unqualified.
- Unique task: `ICRA-013 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-013 is phase 3A: deepen the existing `RiskGridMap` Module with a fixed world-aligned lattice,
  deterministic integer world keys and atomic geometry-plus-generation publication. The existing
  `refreshFromProvider()`/immutable `RiskGridSnapshot` Interface remains the consumer Seam.
- This slice deliberately retains full provider evaluation. Dense ring storage, entering-slab-only
  spatial work and cross-refresh evidence reuse require a separately reviewed cache-validity Seam;
  they must not be approximated by caching complete time-dependent `HorizonRisk` results.
- Calibration, main-flow smoke, qualification, worker/default tuning, GPU and P4 remain forbidden.

## 2026-08-21 — ICRA-011 review and ICRA-012 narrow repair authorization

### Review identity and independent verification

- Fixed review base: `c865c74317e23b9cb5339174e662d1fc7e87a4ec`.
- Reviewed HEAD: `9faf12139d49b93c259af014249c3c1b447e179c`.
- Reviewed commits: `7be95f0` and `9faf121`; both carry applicable requirement IDs.
  `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the 13 ICRA-011 allowlisted files and passes
  `git diff --check`. The only untracked file remains
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Supervisor rebuilt the root Predictor/profile, plan-env and plan-manage test targets. Dynamic
  linkage with the prescribed environment resolves product code to the current ICRA-011
  `results/icra27/icra011/build_root/libiap.so`; only retained generated ROS typesupport comes
  from the repository-local ICRA-009 facade.
- The three exact Predictor regressions passed 3/3. The exact production count, worker 1/2/4
  equivalence and failure-after-success retention regressions passed 3/3. The Python profile
  evidence contract passed 2/2.
- All six retained suites passed 6/6, 43/43, 35/35, 2/2, 3/3 and 48/48, for **137/137**.
  No main flow, ROS launch, smoke, qualification, benchmark, analyzer or GPU preflight ran.

### Standards axis

- **FAIL, one hard finding:** the ICRA-011 entry in `docs/CHANGES.md` records results but no
  reproducible command, and README contains no ICRA-011 command. `AGENTS.md` Definition of Done
  requires the command in `docs/CHANGES.md` or README; `DEV_LOG.md` alone does not satisfy the
  prescribed location.
- One non-blocking Data-Clump/Shotgun-Surgery judgement: the five exact diagnostics repeat
  through Predictor, production state, reset/aggregation/copy and JSON serialization. This
  follows the existing flat schema and the task's exact keys, so ICRA-012 must not introduce a
  scope-expanding abstraction.
- Standards count: one hard finding and one non-blocking judgement. Worst Standards issue:
  missing reproduction command in the prescribed document.

### Spec axis

- **FAIL, one medium finding:** fixed-base `unique_positions` was the populated LiDAR-cache
  size, and therefore zero in `GnssOnly`. ICRA-011 now assigns it from the generalized
  `SpatialAdvisory` cache, causing `unique_positions` and production
  `predictor_unique_positions` to become nonzero in GNSS-only mode. This violates the explicit
  requirement that legacy `unique_positions`, `lidar_evaluations` and `lidar_cache_hits`
  retain their meanings. The existing GNSS-only worker regression checks the new counters but
  omits these legacy fields.
- All other phase-2 requirements conform: exact allowlist, private call-local internal Seam,
  coherent source key, early-failure non-poisoning, per-horizon growth/fusion/materialization,
  current-attempt health reset, worker aggregation, canonical `76800/12800/64000` profile
  counts, zero scalar mismatches and diagnostic-only latency.
- Spec count: one medium finding. Worst Spec issue: legacy LiDAR position-counter semantics
  changed in GNSS-only mode.

### Design disposition and next task

- Verdict: `ICRA011_REQUEST_CHANGES`. The core phase-2 implementation and performance evidence
  are accepted, but phase 2 is not closed while either review axis fails. Gate-0B remains
  `BLOCKED_PERFORMANCE_AND_CALIBRATION_PENDING`; P4 remains `NOT_QUALIFIED`.
- The Predictor remains a deep Module: callers retain only `query()`/`queryBatch()`, while
  `SpatialAdvisory` stays a private internal Seam. The repair must preserve that Depth and
  Locality; no public cache Interface or extra Adapter is justified.
- Unique next task: `ICRA-012 / GATE_0B` in `NEXT_TASK.md`. It restores legacy LiDAR diagnostics
  across source modes/non-cacheable/early-invalid cases, strengthens GNSS-only production
  worker evidence, and adds required ICRA-011 reproduction commands to `docs/CHANGES.md`.
- Phase 3 fixed-lattice/rolling-window work is the intended following stage only after this
  narrow repair passes a separate Supervisor review. It is not authorized now. Calibration,
  main-flow smoke, qualification, GPU work and P4 remain forbidden.

## 2026-08-21 — ICRA-010 review and ICRA-011 phase-2 authorization

### Review identity and verification

- Review base: `12c2396f9b9fe31038831547e57b08f57b87cd78`.
- Reviewed HEAD: `b0280367dae3cf61176cf80bc72f2b52e1452ce0`.
- Reviewed commits: `5c55c76` and `b028036`; both carry `IAP-RQ-320`,
  `IAP-RQ-321` and `IAP-RQ-322`. `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- The aggregate diff contains exactly the seven ICRA-010 allowlisted files and passes
  `git diff --check`. The preserved untracked PDF remains unchanged at SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- The Supervisor independently reran both exact regressions (1/1 each) and the six complete
  repository-local suites: 6/6, 41/41, 35/35, 2/2, 3/3 and 47/47, for 134/134 PASS. Runtime
  linkage resolves the ICRA-010 `libiap.so`. No main flow, ROS launch, smoke, qualification,
  profile, benchmark or GPU preflight ran during review.

### Standards axis

- PASS with zero findings: file ownership/scope, requirement IDs, synchronized
  `CHANGES`/`TRACEABILITY`/`DEV_LOG`, repository-local evidence and handoff all conform.
- No reportable baseline smell: `NOT_EVALUATED` is a proper domain state, the test lambda
  removes repetition, and Module-level vs production-publication regressions cover distinct
  Interfaces.

### Spec axis

- PASS with zero findings: `NOT_EVALUATED` is the default before the growth helper; only the
  helper returns `APPLIED` or `NOT_REQUIRED_TAU_ZERO`; invalid horizon remains explicitly typed.
- PASS: unsupported frame, stale odometry/snapshot and missing required GNSS remain non-applied;
  valid positive/tau-zero/invalid controls are covered.
- PASS: the real production provider rejects the positive-horizon early failure as
  `provider_refresh_failed` and retains identical active snapshot identity, generation and
  ordered data. No unrequested behavior or phase-2 work entered ICRA-010.

### Disposition and next task

- Verdict: `ICRA010_PASS_PHASE1_CLOSED`. Phase-1 P0 semantic implementation is accepted.
- Gate-0B remains blocked, now on the staged performance refactor, production
  calibration/activation and later qualification; this review does not qualify P0 or
  authorize P4.
- Unique task: `ICRA-011 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-011 enters frozen phase 2 directly: a private within-refresh SpatialAdvisory Seam inside
  the Predictor Module reuses GNSS/LiDAR spatial work while every horizon still performs
  covariance growth, fusion and result materialization. The public Predictor and
  `RiskGridSnapshot` Interfaces remain unchanged, preserving Depth and Locality.
- Canonical target counts are 76,800 logical/provider/fusion results, 12,800 spatial/GNSS/LiDAR
  recomputes and 64,000 within-refresh reuses. Focused scalar equivalence and a repository-local
  offline diagnostic must prove the boundary. Phase 3 rolling, cross-refresh reuse, worker
  tuning, calibration, smoke, qualification, GPU and P4 remain unauthorized.

## 2026-08-21 — ICRA-009 review and ICRA-010 typed-status repair authorization

### Review identity and verification

- Review base: `e67906df71444d0fb576c6dcaca02883108b4424`.
- Reviewed HEAD: `0069303008c719a708970f59732c44c2a05ad5b0`.
- Reviewed commits: `172556c` and `0069303`; both bind the applicable phase-1
  requirements, and `dev/icra` matched `origin/dev/icra` at divergence `0 0`.
- All 26 aggregate-diff paths are explicitly authorized by ICRA-009. `git diff --check`
  passed. The preserved untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remained unchanged at
  SHA-256 `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- The Supervisor independently reran the six repository-local focused suites: local occupancy
  6/6, Predictor 40/40, risk grid 35/35, frozen map epoch 2/2, Adapter 3/3 and P0 runtime 46/46,
  for 132/132 PASS. Two initial Supervisor invocations overwrote the system
  `LD_LIBRARY_PATH` and exited 127 before test execution; appending the existing environment
  reproduced 2/2 and 46/46. No ROS launch, main flow, smoke, qualification, profile or GPU
  preflight ran.

### Standards axis

- PASS: file scope, commit requirement IDs, synchronized `CHANGES`/`TRACEABILITY`/`DEV_LOG`,
  repository-local outputs and handoff procedure conform.
- Judgement-only smells: frozen and live occupancy diagnostics duplicate address/classification
  logic; the Adapter's private field forwarding is a Data Clump. Neither is a phase-1 hard
  finding or authorized refactor target.

### Spec axis

- PASS: the neutral frozen `GridMap` epoch preserves dependency direction and binds complete
  raw/fused LOS from the same immutable generation as diagnostics; inflated-only cells remain
  collision diagnostics.
- PASS: exact-capacity adaptation, provider ownership, occupancy/prior start-end validation,
  prior generation capture and atomic old-snapshot retention conform.
- PASS: valid tau-zero and positive-horizon covariance algebra, finite/SPD/monotonic tests and
  worker 1/2/4 equivalence conform.
- P1: `PredictorModule::queryWithLidar()` preassigns `APPLIED` before frame/freshness
  validation. A finite positive-horizon early return can therefore claim propagation happened
  when the helper never ran. Because the production provider rejects only required
  non-`APPLIED` results, this can publish an invalid/unknown replacement generation instead of
  failing the whole batch and retaining the active snapshot. Existing tests assert fallback
  reasons but not this status/publication contract.

### Disposition and next task

- Verdict: `ICRA009_REQUEST_CHANGES_TYPED_STATUS`. Gate-0B remains
  `BLOCKED_PERFORMANCE_AND_SEMANTICS`; P4 remains `NOT_QUALIFIED`.
- Unique task: `ICRA-010 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-010 is a narrow product repair: make covariance-growth status truthful on every early
  return and prove production whole-batch retention. It does not reopen map/covariance design.
- If ICRA-010 passes review, the following task enters frozen phase 2, within-refresh spatial
  advisory deduplication, without another broad audit. Phase 2, rolling, profile, smoke,
  qualification, calibration, GPU and P4 are not authorized by this task.

## 2026-08-21 — ICRA-008 review and ICRA-009 phase-1 development authorization

### Review identity and verification

- Review base: `6c122a318bbe0970eb6a45eab817a5bdc24ba43a`.
- Reviewed HEAD: `8b60d95d9ffa561f8e4408a68c47ff685747bcd5`.
- Reviewed commits: `a6d863e` and `8b60d95`; both bind `IAP-RQ-312`,
  `IAP-RQ-314`, `IAP-RQ-320`, `IAP-RQ-321` and `IAP-RQ-322`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0`. The only worktree item was the
  preserved untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, whose SHA-256 remained
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- `git diff --check` passed. The review used source inspection and the Builder's retained
  repository-local focused-test record; no ROS, main flow, smoke, qualification, profile or
  GPU preflight ran.

### Standards axis

- PASS: exactly `DEV_LOG.md` and
  `results/icra27/icra008/P0_SEMANTIC_SEAM_AUDIT.md` changed; both commits contain all mapped
  requirement IDs; no product/test/config/analyzer/Supervisor document changed.
- PASS: the report preserves the frozen design as authority, makes no Gate claim, records the
  initial stale-library test failure as well as the corrected test invocations, and contains
  no forbidden runtime/external-write evidence.
- Low judgement smell: proposed reasons were raw strings. ICRA-009 therefore requires domain
  enum/constants inside the Module and string serialization only at the health boundary.

### Spec axis

- Accepted: the audit correctly proves that current production occupied-skip diagnostics and
  the unbound GNSS `LocalOccupancyGrid` are separate map inputs; it selects one immutable
  same-generation binding and rejects `../glim`, mutable and different-source alternatives.
- Accepted: it inventories current/legacy covariance candidates, freezes the empirical
  `Sigma_base(tau) = Sigma_base(0) + sigma_grow^2 tau I3` Seam behind the existing Predictor
  Interface, preserves exact tau-zero behavior, and defines finite/PSD/monotonic/fail-closed
  rules without inventing a production value.
- Accepted: the exact test matrix, invariance-test replacement, 76,800 logical shape, current
  counter meanings and phase-1 no-schema-change conclusion are suitable for development.
- High correction: the report placed construction of `LocalOccupancyGrid` in `plan_env`, but
  that package has no IAP dependency and its proposed file set forbade adding one. The frozen
  resolution is a neutral `GridMap::FrozenOccupancyEpoch` Interface and an explicit testable
  `P0OccupancyEpochAdapter` in `ego_planner`, the package that already depends on both Modules;
  `planner_manager` only invokes it.
- High correction: occupancy received start/end version validation, but the required
  integrity-derived prior did not. ICRA-009 adds `prior_source_generation` and validates both
  source generations before provider work and immediately before atomic publication.
- Medium correction: default `LocalOccupancyGrid::max_voxels=200000` can silently truncate
  LOS. ICRA-009 requires exact capacity, insertion diagnostics and final unique-count equality;
  any mismatch retains the old snapshot.

### Disposition and next task

- Verdict: `ICRA008_AUDIT_ACCEPTED_WITH_SUPERVISOR_CORRECTIONS`. This is not an unqualified
  implementation-ready PASS, but all gaps are now concrete Supervisor decisions and do not
  justify another audit cycle.
- Gate-0B remains `BLOCKED_PERFORMANCE_AND_SEMANTICS`; P4 remains `NOT_QUALIFIED`; P5 remains
  implemented but unqualified.
- Unique task: `ICRA-009 / GATE_0B` in `NEXT_TASK.md`.
- ICRA-009 enters P0 phase-1 product development: neutral versioned map epoch, complete
  immutable production GNSS LOS Adapter, occupancy/prior source validator, and empirical
  horizon covariance growth with focused tests.
- The growth parameter is declared with an invalid fail-closed production default. Numerical
  calibration/activation, rolling window, within-refresh spatial dedup, performance work,
  smoke, qualification, GPU and P4 are separate future authority decisions.

## 2026-08-21 — ICRA-007 review, P0 design freeze and ICRA-008 authorization

### Review identity and verification

- Review base: `62646b4b5262a921b6895f7192d610e5b80100c6`.
- Reviewed HEAD: `bb3a87136361032b463985a002c844a430f99e07`.
- Reviewed commits: `3b6c5e2` and `bb3a871`; both bind `IAP-RQ-320`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0`; the pre-existing
  `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` remained untracked and untouched.
- `git diff --check` passed. Supervisor reran the repository-local, non-ROS
  `test_predictor_risk_conversion` (2/2), `test_predictor_module` (37/37) and
  ICRA-007 evidence contract (1/1); all passed. The ROS-aware P0 test was not
  rerun because the Builder already proved it writes outside the repository.

### Standards axis

- Hard procedural nonconformance: the ROS-aware focused P0 test created
  `/root/.ros/log/test_p0_risk_grid_runtime_484375_1787290745847.log` and the
  Builder then deleted it. This violates both the task's no-external-write/no-cleanup
  rule and `AGENTS.md` repository-boundary preservation. Recording the event is
  truthful but cannot make the execution a clean PASS.
- Judgement risks: the offline profiler duplicates production grouping/dispatch and
  its own replay loop; the 1,214-line diagnostic owns fixture, timing, hashing,
  validation and serialization; modes/statuses are raw strings. These do not block
  the accepted diagnostic but must not be copied into the product refactor.

### Spec axis

- PASS: `frozen_runtime=CURRENT_PRODUCTION` does not bind GNSS occupancy;
  `map_los_candidate=NOT_CURRENT_PRODUCTION` binds only the deterministic 704-point
  occupancy difference.
- PASS: both modes preserve 76,800 logical/dispatched/conversion queries, 76,800
  GNSS/fusion invocations, 12,800 LiDAR evaluations and 64,000 LiDAR hits per cell.
- PASS: provider timing stops after production-shaped grouping, dispatch, shared
  result conversion and worker join; real scientific replay stays outside the timer.
- PASS: component timer perturbation is below 0.4% at worker 1, checksums/counts are
  stable, and horizon invariance is truthfully classified as `MISSING_SIGMA_GROWTH`.
- PASS: no production science/cache/config/threshold, worker, GPU or P4/P5 change.

### P0 disposition and design freeze

- Verdict: `ICRA007_TECHNICAL_PASS_PROCEDURAL_NONCONFORMANCE`.
- Gate-0B remains `BLOCKED_PERFORMANCE_AND_SEMANTICS`; P4 remains `NOT_QUALIFIED`.
- Retained ICRA-005 provider/refresh p95 remains approximately `639.377/657.214 ms`.
  Faithful ICRA-007 worker-1 frozen provider p95 is `577.931 ms`; map-LOS candidate
  p95 is `1172.415 ms`. GNSS is the dominant ranked cost.
- `docs/icra27/P0_ROLLING_RISK_WINDOW_DESIGN.md` is frozen as the sole P0 refactor
  architecture source. Active scope, plan, requirements, code map, traceability and
  changes now distinguish the 76,800 logical field from actual spatial recompute,
  reuse, provider/advisory invocation and horizon-fusion work.
- The frozen sequence is semantic correctness, within-refresh spatial deduplication,
  fixed lattice/ring window, version/TTL/delta invalidation, CPU scaling, then an
  independently authorized smoke and Gate-0B qualification. P4 cannot start earlier.

### Required next action

- Unique task: `ICRA-008 / GATE_0B` in `NEXT_TASK.md`.
- Perform one repository-local implementation-readiness audit of the concrete
  production GNSS occupancy ownership/lifetime Seam, existing covariance-growth
  implementations, phase-1 tests, evidence counters and minimal ICRA-009 file scope.
- Do not change product/test/launch/analyzer code or run ROS. If ICRA-008 resolves the
  requested concrete decisions and passes review, the following task will enter P0
  phase-1 product development without another broad audit.

## 2026-08-21 — ICRA-006 review and ICRA-007 fidelity repair

### Review identity and verification

- Review base: `cf367231347e69cb3dec58016a94c2b48397af07`.
- Reviewed HEAD: `b4fc5746dc4de401dbf8ccf7c0f93706dbdabb88`.
- Reviewed commits: `f2ad7e3`, `b929821` and `b4fc574`; all bind `IAP-RQ-320`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0`; the preserved PDF remained the only untracked file.
- `git diff --check cf36723...b4fc574` passed. Supervisor reran `test_predictor_module` (37/37) and the profile evidence contract (1/1); both passed.
- A separate repository-local 1-warmup/5-iteration run reproduced status PASS, checksum `bc296383f5cb17cf`, worker 1/2/4 p95 `1182.208 / 628.348 / 341.097 ms`, and worker-1 GNSS/LiDAR/fusion cumulative p50 `1009.607 / 29.592 / 56.728 ms`. This is reproducibility evidence for the committed profiler, not current-runtime qualification.

### Standards axis

- Hard: the new test and profile make scientific equality over horizons `0.0..2.5 s` part of PASS. This truthfully observes current behavior but conflicts with `docs/spec/conventions.md` and `docs/spec/talk_spec.md`, which require empirical `Sigma -> Sigma_pred` future propagation and PL derived from it. The invariant result cannot authorize whole-result cross-horizon caching.
- Judgement-only duplicated-code smells: three nearly identical component timing blocks in `predictor_module.cpp`; the same 91 scientific fields are independently enumerated by hashing, whitelist output and test equality helpers.

### Spec axis

- High: the profiler does not exercise the frozen provider's GNSS path. It installs a 704-point `LocalOccupancyGrid` and performs map ray LOS, while `P0RiskGridRuntime::refreshTimerCallback()` currently sets only LiDAR map points/primitives on the production Predictor module. The offline worker-1 provider p95 `1193.774 ms` versus retained production provider p95 approximately `639.377 ms` corroborates the mismatch. Absolute component percentages and diagnostic-budget crossings cannot be attributed directly to current P0.
- Medium: the profiler's `result_materialization` moves `PredictorQueryResult` objects into another vector. Production materialization calls `makeRiskPredictionResult()` for every query. The reported region is not the production conversion cost requested by `NEXT_TASK.md`.
- No forbidden ROS/main-flow run, formal configuration change, threshold change, production optimization, P4/P5 work or ownership breach was found.

### Accepted diagnostic facts and verdict

- Accepted: exact logical shape; actual offline dispatch counts; stable worker checksums/counts; one LiDAR evaluation plus five cache hits per position; GNSS and fusion invoked once per dispatched horizon query; strong CPU scaling; current six-horizon scientific invariance; repository-local execution and documentation.
- Confirmed from ICRA-005: current production provider p95 is approximately `639.377 ms`, with total refresh p95 `657.214 ms`; the provider envelope is the runtime blocker.
- Code inspection confirms only LiDAR is cached per position. GNSS and fusion are recomputed for every horizon. The ICRA-006 map-LOS profile makes GNSS the largest cost in its intended-mode workload, but the exact current-runtime GNSS share remains unqualified until the path mismatch is repaired.
- Verdict: `ICRA006_REQUEST_CHANGES`. Gate 0B remains `BLOCKED_PERFORMANCE_AND_SEMANTICS`; P4 remains `NOT_QUALIFIED`.
- GPU acceleration is not authorized. Four CPU workers achieved about `3.47x` reproducibly on a 20-core CPU, while the larger algorithmic opportunity is to compute spatial GNSS/LiDAR advisory once per position and retain only a cheap horizon-dependent propagation/fusion stage.

### Required next action

- Unique task: `ICRA-007 / GATE_0B` in `NEXT_TASK.md`.
- Repair the profiler to distinguish exact frozen-runtime behavior from a separately labelled map-LOS candidate path, measure real production result conversion, quantify component-timer perturbation, and report missing horizon propagation as a blocker.
- Do not implement caching, covariance growth, worker/profile changes or a GPU path in ICRA-007. Supervisor will use faithful evidence to issue one bounded CPU remediation task.

## 2026-08-21 — ICRA-005 review and ICRA-006 diagnostic authorization

### Review identity and synchronization

- Review base: `a33beadffa51d4669501d194065bc20da51e36d9`.
- Reviewed HEAD: `381ea49ea197a3fbba992650831f93e44bd95b8c`.
- Reviewed commits: `fba4c18` and `381ea49`; both bind `IAP-RQ-320`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0` after `git fetch origin`.
- The only untracked item remained `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`; it was preserved and excluded.
- `git diff --check a33bead...381ea49` passed. The focused analyzer suite passed 15/15, and a read-only raw-trace replay reproduced the committed performance failure.

### Evidence verdict

- The retained-evidence hashes matched, and the benchmark integrity analyzer was correctly changed to fail closed on zero or invalid/non-finite captured integrity reports.
- GPU preflight passed before capture/ROS with both required `nvidia-smi` calls, `cuInit(0)=0`, `device_count=1` and one RTX 4070 Ti SUPER.
- The one authorized fixed benchmark preserved the 60/55-second contract, CPU mapping, one worker, six horizons and 76,800-query logical shape. P1/P2/P3/P4/P5 stayed disabled.
- `iap_rosnode` remained alive through runtime; 565/565 integrity reports were valid. There were 72 successful and 2 failed generations, and every successful generation recorded 76,800 logical queries.
- Refresh p50/p95/max were `649.6330975 / 657.21388795 / 661.487876 ms`; interval p50/p95 were `650.4311489999992 / 658.0863929999996 ms`; stale ratio was `0.5945945945945946` and failed ratio `0.02702702702702703`.
- Analyzer exit 1 had exactly one failure: `refresh_p95_over_400_ms`.
- Supervisor verdict: `ICRA005_P0_PERFORMANCE_GATE_FAIL`. Gate 0B is `BLOCKED_PERFORMANCE`; P4 remains `NOT_QUALIFIED` and P5 remains `IMPLEMENTED_BUT_UNQUALIFIED`.

### Performance diagnosis boundary

- Retained raw health rows place provider batch p50/p95 at approximately `633.259 / 639.377 ms`; median non-provider refresh overhead is approximately `16.235 ms`. The provider consumes about 97% of median refresh wall time.
- The frozen provider processes up to 12,800 spatial positions across six horizons with one worker. Existing diagnostics show one LiDAR evaluation plus five LiDAR cache hits per repeated position, while code inspection indicates GNSS and fusion remain invoked per horizon.
- Historical predictor microprofile evidence ranks GNSS above LiDAR and fusion per query, but it is not the same ICRA-005 workload. It is a hypothesis for measurement, not a formal component-level conclusion.
- The formal analyzer CSV drops finite provider-duration rows during health-row deduplication even though the raw JSONL retains them. ICRA-006 may use a separate diagnostic parser but may not change the formal analyzer or retained verdict.

### Standards and scope findings

- Spec disposition: PASS for the one-shot benchmark contract; the truthful performance result is a Gate failure, not an incomplete execution.
- Standards disposition: accepted with one recorded boundary violation. ROS created `/root/.ros/log/2026-08-21-03-51-32-690827-mint-X-365799/launch.log` outside the repository and DeepSeek then removed it. This did not change the retained performance evidence, but it violated the no-external-write/no-cleanup rule and must not recur.
- No benchmark retry, workload tuning, backend fallback or P4/P5 work is accepted or authorized.

### Required next action

- Unique task: `ICRA-006 / GATE_0B`, defined in `NEXT_TASK.md`.
- Active role: `DEEPSEEK`; state: `TASK_READY`.
- Build a non-main-flow, repository-local profiling loop; decompose provider cost, test horizon semantics, and measure worker 1/2/4 scaling with output equivalence.
- ICRA-006 does not implement the selected optimization. Supervisor will use its evidence to authorize one bounded remediation task, followed by a separate smoke and fixed benchmark sequence.

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

## 2026-08-18 — ICRA-002 review

### Review identity and synchronization

- Branch: `dev/icra`
- Review base: `eeb3be6d2de5e878be773522b357a1a634bb62b2`
- Reviewed HEAD: `b7022d792a3e104fd7e0b38021d0168cc1235cdf`
- Reviewed commits: `489e4ca` (ICRA-002 implementation) and `b7022d7` (handoff SHA record).
- Startup synchronization: the worktree was clean; `git fetch origin` produced divergence `0 0`; `git pull --ff-only origin dev/icra` reported already up to date. `HEAD` and `origin/dev/icra` both resolved to the reviewed HEAD.
- State recovery: the requested `docs/icra27/AGENT_STATE.md` does not exist. Per `AGENTS.md`, the root `AGENT_STATE.md` is the unique state source. Its handoff used invalid role `SOL`, status `BLOCKED`, and was written by DeepSeek despite Supervisor ownership; this review restores the Supervisor role from the protocol and treats the commit as the review handoff.

### Disposition

- Review disposition: `REQUEST_CHANGES`.
- Gate 0A verdict remains `NO_GO_P2`; P2 remains frozen.
- Gate 0B remains `BLOCKED / UNQUALIFIED`. No mandatory smoke or fixed benchmark was run, so there is still no valid P0 input-availability, generation-count or latency result.
- Accepted partial work: explicit CPU/GPU selection, the basic readiness/failure-reason schema, structured process fields, non-finite original-cost rejection, control-point validation, zero-generation classification, recommendation suppression below 20 generations, and downstream aggregation independent of P2 qualification are useful foundations. They do not satisfy the execution gate or the fail-closed contract as committed.
- Unique repair task: `ICRA-003 / GATE_0B` in `NEXT_TASK.md`. No later P5 task is authorized until this repair is reviewed.

### Standards axis

Hard findings:

1. `AGENT_STATE.md` is Supervisor-owned, but DeepSeek edited it and set `active_role: SOL`; the only protocol roles are `SUPERVISOR` and `DEEPSEEK`. This also directly violated the ICRA-002 BLOCKED-path instruction not to edit that file.
2. `run_gate0_qualification.py` still fails open. `run_gate0b()` returns only the top-level launch code even when `required_processes_ok` is false or capture fails. Shutdown phase is inferred from `run_duration_s - 1` rather than an actual controlled-shutdown transition, and host-wide command matching can credit an unrelated user process as this launch's child.
3. The mandatory 20-second no-bag smoke was skipped. `CAMPAIGN_DISK_NO_GO` governs the formal campaign, not this bounded smoke; the implementation has only a hard-coded 60-second P0 configuration. `DEV_LOG.md` therefore lacks the required smoke command, exit code and evidence.
4. The handoff claims no writes outside the repository while recording `colcon`/CTest outputs under `/home/dev/ws_iap/build` and related workspace roots. Those verification writes exceeded the repository boundary.
5. `docs/CHANGES.md` and `docs/TRACEABILITY.md` map backend/readiness/process plumbing indiscriminately to `IAP-RQ-320`, `IAP-RQ-400` and `IAP-RQ-410`. These changes do not implement the RQ-400 hinge objective or RQ-410 receding-horizon loop. The traceability statement also overclaims exact reason/readiness coverage: changed tests omit `message_stamp_unavailable`, `snapshot_builder_invalid` and GNSS readiness assertions.

Non-blocking maintenance risk:

- `run_gate0b()` reads `RequiredProcessMonitor._seen` directly. The monitor should expose one structured result rather than leaking private mutable state.

### Spec axis

Blocking findings:

1. The required fixed sequence is absent: no 20-second smoke was executed, the runner provides no smoke mode, and therefore the conditional 60-second Gate 0B also has no evidence. The 27 GiB free-space observation is not a valid blocker for a no-bag smoke.
2. Required-process evidence is not fail closed: a runtime child death or capture failure can still produce runner exit 0; an unrelated same-name process can satisfy discovery; and elapsed-time classification cannot prove runner-controlled shutdown.
3. `rangeCallback()` already holds `health_state_mutex_` and acquires it again when a valid nonempty epoch is produced. The non-recursive mutex deadlocks the live GNSS input path that Gate 0B is intended to restore; existing tests do not exercise this callback path.
4. Readiness does not meet the unseen/invalid/stale contract. Origin has no freshness or stamp and equates seen with valid; GNSS marks seen only after a valid nonempty epoch. `currentMessageStamp()` uses the last odometry/current message as “now”, so stopped input does not age, while `buildSnapshot()` does not reject stale odometry/current-integrity input.
5. Backend provenance is partial: the odometry SHA256 is computed before an optional initialization-mode override, so it may not describe the final effective file. The new test checks one generic file but not invalid backend selection or all three final configs.
6. Analyzer fixes are incomplete. Non-finite latency values are silently dropped before p95, allowing incomplete timing evidence to pass; refinement/update/publication reachability remains collapsed; and the CLI always exits 0 even when Gate 0B fails.
7. Required focused coverage is incomplete. The process test mutates monitor internals rather than exercising real subprocess runtime/control-shutdown behavior, and no launch test proves a live `iap_rosnode`, valid integrity evidence or a successful 76,800-query generation.

### Supervisor verification

- `git diff --check eeb3be6...b7022d7`: exit 0.
- `python3 -m unittest discover -s test -p 'test_gate0_runner.py' -v`: exit 0, 4 tests.
- `python3 -m unittest discover -s test -p 'test_gate0_analyzer.py' -v`: exit 0, 9 tests.
- `python3 -m unittest discover -s test -p 'test_test_planner_launch.py' -v`: exit 0, 11 tests.
- These passing focused tests confirm the asserted unit behavior but also expose the missing lifecycle, readiness and smoke coverage above. The Supervisor did not execute ROS or write evidence outside the repository during review.

### Required next action

- Active role: `DEEPSEEK`; state: `TASK_READY`.
- Execute only `ICRA-003`. Repair the evidence path first, then run one smoke; run one fixed Gate 0B only after smoke PASS. Record a real blocker in `DEV_LOG.md` and return control without editing Supervisor-owned state.

## 2026-08-18 — ICRA-003 environmental invalidation and retry authorization

### Handoff and evidence status

- Review base: `7950b47bd09f8bce6752b762466b50153651ebf9`
- Reviewed HEAD: `9eb3481ba9bd17c07f5fe34698ec2035eaa904a1`
- DeepSeek completed the ICRA-003 implementation and repository-local test suites, ran exactly one 20-second smoke, stopped after analyzer failure, did not retry, and did not run the 60-second benchmark.
- The smoke manifest reports `iap_rosnode` seen with no runtime failure and a controlled-shutdown stop. Topic capture files contain zero health/integrity rows, while stdout contains integrity reports and P0 generations with 76,800 refresh queries. These conflicting observations remain diagnostic only and cannot qualify Gate 0B.
- Gate 0B remains unqualified; Gate 0A remains `NO_GO_P2` and P2 remains frozen.

### Operator clarification and current preflight

- The operator confirmed that the Docker environment had lost its functional GPU attachment and requires a container restart. The IAP main flow still requires GPU access; selecting the CPU mapping backend does not remove that prerequisite.
- Supervisor preflight in the current container: `/dev/nvidiactl`, `/dev/nvidia0` and `/dev/nvidia-uvm` exist, and `libcuda.so.1` loads, but `nvidia-smi --query-gpu=index,name,uuid,driver_version --format=csv,noheader` fails with `Failed to initialize NVML: Unknown Error`.
- Verdict for the current container: `GPU_NOT_READY`. Per the operator's standing instruction, no ROS run may start in this state.
- ICRA-003 smoke disposition is `INVALID_ENVIRONMENT / GPU_NOT_READY`; its artifacts are retained and it has no Gate 0B performance meaning.

### Authorized next action

- Unique task: `ICRA-004 / GATE_0B` in `NEXT_TASK.md`.
- Active role: `DEEPSEEK`; state: `TASK_READY`, but execution must wait until the operator restarts Docker.
- Implement a persistent NVML plus CUDA Driver API preflight. Failure must stop before ROS and return `GPU_NOT_READY / BLOCKED` without retry.
- After preflight PASS, exactly one replacement 20-second smoke is authorized in a new evidence directory. The 60-second benchmark remains forbidden pending Supervisor review.

## 2026-08-20 — P0→P4→P5 scope pivot and ICRA-004 reissue

### Decision identity and worktree protection

- The operator explicitly authorized the conference target change from the P0+P5 contingency route to conditional `P0 -> P4 -> P5`.
- The read-only review used `HEAD=bd3858a72ba06b7eb1551006876c55362c979bab`; `origin/dev/icra` matched with divergence `0 0` after `git fetch origin`.
- ICRA-004 had no `DEV_LOG.md` start record and no `results/icra27/icra004/` directory. It is reissued, not cancelled or renumbered.
- Existing untracked `Change_Needed.md`, `P4_GATE0_AUDIT.md` and `dev/ICRA_SYSTEM_FLOW.pdf` were preserved. The two Markdown inputs enter this preparation; the PDF remains untouched and untracked.

### Scope verdict

- Route verdict: `CONDITIONAL_GO_P0_P4_P5_PREPARATION`.
- Current qualification state: `P0 BLOCKED/UNQUALIFIED -> P4 NOT_QUALIFIED -> P5 IMPLEMENTED-BUT-UNQUALIFIED`.
- Gate 0A remains the historical `NO_GO_P2`: all 378 optimizer-success attempts were singleton. The new target does not alter that evidence and does not imply `GO_P4`.
- P1/P2/P3 remain present in source, tests and legacy profiles. The future ICRA composite profile must disable their high- and low-level effective paths rather than delete them.
- P4 is conditional on a closed `free -> occupied -> free` collision segment. With no closed segment, original EGO planning continues to P5 without forcing P4.
- P4 remains advisory. Original EGO occupancy, collision, dynamics, refinement and feasibility checks retain motion authority. P5 final and runtime remain the IAP hard integrity gates.

### Static audit disposition

- The early Gate 0 collision counter observed no closed segments, but the seed crossed the central obstacle. The prepass stopped inside the obstacle before observing its exit; zero closed segments is not proof of no collision.
- Initial collision handling dispatches only one A* and does not create an original/risk guide pair. The later dual-guide path normally sees no snapshot because the manager clears it before rebound optimization.
- Existing P4 `path_mean_cost/path_max_cost` describe risk queries on expanded edges, not a risk profile of the returned guide. They cannot support a lower-risk claim.
- With `manager/use_distinctive_trajs=true`, later legacy candidate selection can replace the P4-derived direction. All ICRA comparison arms will therefore freeze it to `false`.
- P5-3/P5-4/P5-6 voxel fixtures can affect both `queryPredictedPL()` and P4 `queryCost()`. A separately named P4 fixture is still required to avoid coupling P4 and P5 evidence semantics.

### Next task and stop line

- Unique next task remains `ICRA-004 / GATE_0B`, reissued under conference route `P0_P4_P5` and handed off as `TASK_READY` in this changeset.
- ICRA-004 remains a P0-only GPU-preflight and one-shot smoke task. Its smoke keeps P1/P2/P3/P4/P5 disabled and does not authorize the 60-second benchmark.
- P4 code, fixtures, profiles and experiments remain prohibited until P0 Gate 0B passes and a later Supervisor task first authorizes deterministic red fixtures.
- This scope-pivot preparation changes documentation and coordination state only. It runs no ROS experiment and creates no product-code qualification evidence.
- The operator subsequently authorized the scope-pivot Markdown changeset to be committed and pushed, including the two preserved Markdown inputs but excluding the untracked PDF. This changeset therefore returns the active role to DeepSeek as `TASK_READY`; only ICRA-004 is authorized.

## 2026-08-21 — ICRA-004 review and ICRA-005 authorization

### Review identity and synchronization

- Review base: `73cbdddd0f44165f61138dcd74c61ab8dd96ebae`.
- Reviewed HEAD: `3de08928ec6fe57922e64bd892c7f55882e1b8a0`.
- Commits: `728d53d`, `20d3c5d`, `3de0892`; all bind `IAP-RQ-320`.
- `dev/icra` matched `origin/dev/icra` at divergence `0 0`. The only untracked item remained `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
- No task-started `iap_rosnode`, capture or test-planner launch process remained during review.

### Two-axis review

- Standards: PASS with no hard violation. Non-blocking smells were duplicated smoke/benchmark lifecycle code and the benchmark-specific failure name `fewer_than_20_successful_generations` remaining misleading in smoke mode.
- Spec: PASS with no blocking ICRA-004 deviation. Changes stayed within allowed files; no P4/P5 product work, benchmark run, Supervisor-owned edit or external-repository change occurred.
- Supervisor reran `test_gate0_runner.py` (15), `test_gate0_analyzer.py` (13) and `test_gate0_capture_p0_health.py` (1); all passed. One ResourceWarning in the controlled-shutdown test is non-blocking.

### Evidence verdict

- GPU evidence records both required `nvidia-smi` commands at exit 0, `cuInit(0)=0`, `cuDeviceGetCount=0`, and one RTX 4070 Ti SUPER.
- The one 20-second smoke retained 30 health rows, 165/165 valid integrity rows and 10 successful generations, each with exactly 76,800 queries.
- Runner and analyzer exited 0. `iap_rosnode` was observed as a launch descendant, had no runtime failure, and stopped only during controlled shutdown.
- Frozen configuration remained CPU mapping, `20/15`, no bag/RViz and P1/P2/P3/P4/P5 disabled. No smoke retry or 60-second benchmark occurred.
- Verdict: `ICRA004_SMOKE_PASS`. This is only the Gate-0B prerequisite; P0 remains unqualified pending the fixed benchmark.

### Evidence boundary and next task

- The analyzer consumed a runtime `test_planner_manifest.json` and produced `effective_config.json`, but both were ignored and absent from the ICRA-004 Git changeset. Their retained hashes are now frozen in `NEXT_TASK.md`; ICRA-005 must force-add the unchanged files before running anything.
- The existing analyzer only applies zero-valid-integrity fail-closed classification to `p0-smoke`. ICRA-005 must extend that same evidence rule to `p0-full-grid` and add focused tests before the benchmark.
- Unique next task: `ICRA-005 / GATE_0B`. After those two bounded closures pass, exactly one unchanged 60-second benchmark is authorized. Any failure stops without retry or tuning and returns to Supervisor.
