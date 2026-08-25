# ICRA-063 — Version r6 support semantics and complete P4 calibration

> Active gate: `P4_G0C_R6_TEMPORAL_AND_OCCUPIED_SUPPORT_LIVE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA062_ENGINEERING_PASS_R5_TEMPORAL_SUPPORT_REQUEST_CHANGES`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-322`, `IAP-RQ-423`
> Conference route: conditional P0 -> P4 -> P5
> This task: r6 time/support repair -> one readiness -> 15 registered runs -> analyzer

## Supervisor decision

ICRA-062 is accepted as a faithful diagnostic task. The final r5 readiness uses predictor worker `4/4`,
releases admission once, produces 12 positive-snapshot closed-segment decisions, and keeps required processes
healthy. Across 12 identities and two 200-sample arms, the classifier reports 3040
`POSITIVE_WEIGHT_OCCUPIED_SKIP` and 10 `TIME_SUPPORT`; every other invalid category is zero. The ten time
failures are risk-arm sample 199 in attempts 1-10 at `tau ~= 2.50208 s`, just beyond the current `2.5 s`
P0 horizon. No r5 registered identity was consumed.

This is one bounded scientific repair and live task. Do not return for an intermediate audit or Review.
Create r6 because both the temporal-support envelope and P4 cost-query semantics change. After one passing
r6 readiness, continue immediately through dependency preflight, all 15 registered identities and the
analyzer. Correctable evidence issues are repaired in this same task and never become a standalone loop.

## 1. Preserve accepted work and boundaries

- Follow `AGENTS.md` synchronization. Preserve the protected PDF, all historical evidence, and v1-v5
  protocol/fixture/registry/dependency/lineage bytes. Record r5 as unconsumed and superseded; do not rewrite
  it. Preserve the v2 obstacle geometry, start, goal, speed, grid resolution/extent, P0 sigma/profile,
  predictor worker `4/4`, seeds, repetitions, formulas, thresholds, selection authority, admission FSM,
  scanner behavior, process rules and all P5 behavior.
- Use only `results/icra27/icra063/` for new outputs. Use a minimal sanitized child environment before every
  build/test/ROS command. Initialize the structured command recorder before the first task command and record
  exact argv/cwd, safe environment-key allowlist, start/end/duration and exit code.
- Retain all current ICRA-056/059/060/061/062 build/install products. Build ICRA-063 into a new repository-local
  root; do not reuse a historical build as final evidence.

## 2. Fold evidence corrections into the technical task

- Stop tracking exactly
  `results/icra27/icra062/readiness_attempt_03/p4_profile_trace_classification.json` while preserving its
  ignored local raw copy. Compact evidence may retain only aggregate counts, hashes and the minimal ten-row
  time-support binding needed to reproduce the verdict. Do not stage other raw readiness products.
- Record the four honest ICRA-062 pre-recorder `UNRECOVERABLE_PRE_RECORDER_FIELD` entries as an immutable Low
  process deviation. Do not invent timestamps or rerun ICRA-062.
- If the touched classifier/trace code still uses the literal occupied source flag, replace it with a named
  domain constant or typed field. Do not perform an unrelated refactor of the launch profile-selection code.

## 3. Create the r6 temporal-support identity

- Create new v6 protocol, registry, dependency and lineage artifacts with exactly 15 new r6 identities in the
  same frozen order, seeds and repetitions as r5. Bind every mechanically affected source/config/build hash.
- Set the r6 effective P0 horizons to exactly `[0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]` seconds. This continues
  the existing fixed 0.5-second cadence and supports the measured `2.50208 s` endpoint without clamping or a
  tuned epsilon. Queries above `3.0 s` remain fail-closed.
- Preserve the accepted predictor worker requested/effective values `4/4`. Do not change the separate legacy
  outer `p0.batch_worker_count=1` field.

## 4. Add a typed P4 conservative occupied-cost-support policy

- Introduce an explicit query policy or enum whose default is the legacy strict behavior. Enable a distinct
  `CONSERVATIVE_OCCUPIED_COST_SUPPORT` policy only for the r6 P4 experiment.
- Under that policy, only a positive-weight corner whose source is `OCCUPIED_SKIP` may contribute the existing
  finite `unknown_cost` to cost interpolation. A zero-weight corner does not participate. Every other
  provider-invalid, stale, non-finite, out-of-map, time-support or unknown source remains fail-closed.
- Apply the identical policy to original-guide and risk-guide profile queries and to risk-aware A* edge-cost
  queries. Do not silently select different semantics by arm or call site.
- Preserve hard safety authority: occupied RiskGrid health and `queryPredictedPL()` remain invalid; original
  EGO occupancy/inflation/collision/dynamics checks remain unchanged; risk-aware A* may never traverse an
  occupied node; P5 remains unchanged. The policy supplies a conservative finite planning cost, not free
  space or valid integrity.

## 5. Focused verification

- Add tests proving default/legacy strict behavior is byte- and result-compatible; r6 exposes the exact seven
  horizons; `tau ~= 2.50208 s` is supported and `tau > 3.0 s` fails.
- Prove positive-weight occupied support returns the configured finite conservative cost, while all other
  positive-weight invalid categories still fail. Prove occupied health/PL validity remains false and A*
  never traverses occupied nodes.
- Prove original and risk arms use identical support semantics, diagnostic trace remains noninterfering and
  registered runs have trace disabled. Preserve existing worker-equivalence, admission and scanner tests.

## 6. Fresh build and one r6 readiness

- Produce one fresh 17-package merged non-symlink Release/CUDA closure using the sequential executor,
  `BUILD_TESTING=OFF`, CUDA/nvcc ON and OpenCV/viewer OFF. Validate indexes, six ELF libraries, zero historical
  linkage, installed/source equality and final hashes.
- Before ROS, run the mandatory GPU preflight. `nvidia-smi`, `cuInit(0)` and `device_count >= 1` must pass;
  otherwise emit `GPU_NOT_READY` and stop before launch without retry.
- Run exactly one nonregistered traced r6 readiness. Require predictor requested/effective `4/4`, max horizon
  `3.0 s`, admission release once, no P4 row before release, positive snapshot identities, closed segments,
  healthy required processes and controlled shutdown.
- Every decision must be `METRICS_ONLY`; both arms must be exactly 200/200; invalid counts for time,
  out-of-map, stale, provider-invalid and other unknown sources must be zero. Trace may classify occupied
  support used by the explicit policy, but it must not create an invalid sample. If readiness fails these
  scientific/process conditions, stop once with a typed genuine blocker. Do not tune again.

## 7. Freeze and execute the registered matrix

- After readiness PASS, freeze and commit/push the exact r6 code/config/build hashes, then continue without
  Supervisor Review. Run one standalone dependency preflight from the final ICRA-063 install plus
  `/opt/ros/jazzy`; require exactly 18 packages, 13 executables, one component, 14 configs and six libraries,
  with zero GPU/launch/identity consumption.
- Invoke the r6 full runner once. Its GPU preflight precedes ROS. Execute all 15 registered IDs in frozen order
  exactly once: 15 attempted, 15 completed, 15 launches, zero retry/exclusion. Every accepted row requires a
  positive snapshot identity, closed segment, `METRICS_ONLY`, 200/200 profiles and zero invalid samples.
- Invoke the r6 analyzer once after runner `COMPLETE`; require exact `DRAFT_ELIGIBLE`. Do not apply the draft,
  enable selection, claim G0C PASS, start G0D/P5 or tune the result.
- One-shot protection begins with the first registered identity. A real GPU/process/RiskGrid/CSV/inventory/
  scientific failure after that point is terminal; no source/config/build change or identity retry is allowed.
  A narrow analyzer-only defect may be fixed and the unchanged complete bundle reanalyzed once.

## 8. Handoff and artifact lifecycle

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-063 evidence. Commit
  with applicable `IAP-RQ-XXX` IDs and push. Do not edit Supervisor-owned files or stage raw products/PDF.
- Do not stop for formatting, path/mode, optional metadata or another correctable pre-identity orchestration
  issue. Repair it in-task. Stop only for genuine GPU/security/external-mutation, failed r6 readiness, or a
  post-identity failure.
- Retain all ICRA-056/059/060/061/062/063 build/install products through Supervisor Review. Only after
  ICRA-063 Review PASS and pushed code/docs may reproducible build/install directories for those tasks be
  deleted. Retain scientific/compact evidence, raw local evidence and the protected PDF.

## Allowed files

- P0 r6 horizon-profile wiring and focused tests; v6 protocol/registry/dependency/lineage and matching
  runner/analyzer/profile artifacts.
- RiskGrid cost-interpolation policy, the P4 profile and risk-A* call sites, classifier named constants and
  focused tests needed for the exact r6 semantics above.
- Builder-owned docs and compact redacted ICRA-063 evidence; removal from Git tracking of the one exact raw
  ICRA-062 classification artifact while preserving its local copy.

## Forbidden

- No fixture, obstacle, speed, grid geometry, P0 sigma, worker count, threshold, formula, seed or repetition
  tuning; no time clamp/tolerance; no v1-v5 or historical-evidence mutation; no alternate horizon beyond the
  exact `3.0 s` extension.
- No analyzer coverage weakening, failed-row exclusion, occupied traversal, health/PL-validity fabrication,
  scanner/admission weakening, CPU fallback, registered retry, threshold application, G0C PASS claim,
  G0D/P5 run, external-repository write, credential persistence, raw-product/PDF staging or cleanup before
  Review.
