# ICRA-067 — Activate the isolated P0+P5 profile and qualification harness

> Active gate: `P0_P5_CONTINGENCY_PROFILE_AND_QUALIFICATION_HARNESS`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Supervisor verdict: `ICRA066_PASS_P4_G0C_NO_GO_P0_P5_CONTINGENCY_ACTIVATED`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`
> Conference route: P0 + P5 contingency
> This task: isolated profile -> fail-closed manifest -> prospective P5 harness; no live run

## Supervisor decision

P4-G0C is authoritatively closed as scientific NO-GO: the risk guide did not achieve a positive Q10 maximum-
risk improvement. Do not revise thresholds, create r7, enter G0D or reuse P4 as the conference treatment.

The pre-registered contingency is now active. P0 remains the immutable future-risk advisory field; original
EGO planning retains occupancy/dynamics authority; P5 final is the hard gate before normal publication and P5
runtime monitors the committed trajectory. P1/P2/P3/P4 remain compiled and tested but disabled in ICRA runs.

Existing `p5` presets and historical P5-1..P5-8 artifacts are useful implementation history, not prospective
qualification of the new route. ICRA-067 adds the missing isolated profile and qualification contract without
changing P0/P5 scientific decisions or launching the main flow.

## 1. Add one deep, fail-closed conference profile

- Add the named profile `icra_p0_p5` to the existing launch/profile resolver rather than duplicating launch
  defaults. It must resolve P0 enabled, P5 final/runtime enabled, and P1/P2/P3/P4 disabled at both high and
  lower-level switches. Freeze `planner_enable_all_safety=false` and `manager/use_distinctive_trajs=false`.
- Disable P1/P2 metrics-only/debug/objective/fanout/viz paths, P3 local/global/debug/viz paths, and all P4
  decision/metrics/debug/trace/viz/application paths. Do not delete their code or legacy profiles.
- Preserve the reviewed P0 Gate-0B profile: worker 4, sigma `0.01`, profile
  `legacy_iap_rq320_baseline_v1`, frozen horizons/ROI/resolution/refresh semantics. Do not reopen P0 performance.
- Enable existing P5 final/runtime and machine-readable evidence without changing thresholds, actions, retry/
  emergency policy, PL/AL formulas or query semantics.
- Reject every explicit CLI/preset/lower-level override that contradicts the profile, including all-safety,
  P1-P4 enable, P5 disable, P0 disable, distinctive-on, metrics/debug/viz leakage and fixture combinations not
  registered by the qualification arm. Do not silently coerce a conflicting override.

## 2. Bind profile identity and prospective evidence

- Introduce the smallest versioned canonical profile/qualification manifest needed to bind route, git commit,
  effective switches/values, P0 profile, P5 thresholds, fixture identity, run identity, analyzer version and
  raw artifact hashes. One source must drive launch resolution and analyzer validation; do not create two sets
  of defaults.
- Add one non-live `icra_p0_p5_qualification` preset/arm family with three typed cases:
  1. `SAFE_NORMAL`: P0 becomes ready; P5 final accepts; exactly one normal B-spline identity is published;
     runtime does not falsely trigger.
  2. `FINAL_REJECT`: the registered final-only fixture is hit; P5 final rejects the candidate; that candidate
     identity has zero normal publication.
  3. `RUNTIME_FAIL`: a registered post-publication unsafe/stale/unknown condition produces the frozen P5 runtime
     action/reason and never fabricates safe evidence.
- Reuse existing P5 fixture and analyzer primitives where their semantics match. Version aliases/manifest
  identity for ICRA; do not treat historical artifacts as new evidence and do not add a new scenario or planner.
- The analyzer/harness must fail closed on missing/duplicate run identities, process death, malformed/non-finite
  rows, topic gaps, unstable P0, wrong switches/profile/hash, fixture leakage, reject-with-publish, missing safe
  publish, or absent runtime action. Controlled shutdown is not runtime failure.

## 3. Test-only acceptance

- Add focused launch/profile golden tests and analyzer synthetic tests for all three cases plus malicious
  overrides. Prove P1/P2/P3/P4 are off, P0/P5 are on, final precedes publish, rejected identity is not published,
  and runtime failure remains separate from final rejection.
- Run focused tests and the repository-local hermetic Python suite. Build only the smallest non-CUDA C++ P5
  unit target if a changed interface requires it; otherwise do not build product packages.
- Produce a validation-only manifest from synthetic inputs. It may not start GPU preflight, ROS, the full runner
  or any live identity. Do not claim P5 qualification from unit/synthetic evidence.

## 4. Document and hand off

- Update `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md` and compact redacted ICRA-067 evidence. Record
  the exact future live commands but do not execute them.
- Commit with applicable requirement IDs and push. Stage only allowed source/tests/config/docs/compact files;
  preserve the PDF and all scientific evidence.
- Return to Supervisor with either `PROFILE_AND_HARNESS_READY` or one typed technical blocker. Do not start the
  live P5 qualification, campaign, paper-result generation or another route pivot.
- Retain any ICRA-067 build/install through Supervisor Review. After PASS and push, Supervisor will delete only
  that task's reproducible build/install products.

## Allowed files

- `launch/test_planner.launch.py` and the smallest existing profile/manifest helper required by the deep profile.
- Existing P5 analyzer/runner helpers and their focused Python tests; existing P5 C++ unit tests only if needed.
- One versioned ICRA P0+P5 profile/qualification config, Builder docs and compact ICRA-067 evidence.

## Forbidden

- No P0/P5 product decision or threshold change; no P4 repair/r7/G0D; no P1/P2/P3 work; no new planner,
  trajectory representation, scenario or fixture semantics; no full build unless interface necessity is proven;
  no GPU preflight, ROS/launch, live runner/identity, rosbag, campaign, threshold tuning/application, external-
  repository write, credential persistence, PDF/raw staging or deletion of scientific evidence.
