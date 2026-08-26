# ICRA P0+P5 cross-layer contract and guard-hardening plan

Status: **Supervisor-frozen plan; ICRA-071 is not active until ICRA-070 reaches qualification PASS and passes review**
Requirements: `IAP-RQ-000`, `IAP-RQ-020`, `IAP-RQ-030`, `IAP-RQ-040`, `IAP-RQ-220`, `IAP-RQ-320`,
`IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`

## Purpose and campaign barrier

This plan prevents a launchable diagnostic mode from silently replacing the intended ICRA system target.
The frozen route is P0+P5 with GNSS pseudorange+doppler, IMU and LiDAR; GNSS/ARAIM and LiDAR integrity;
GPU mapping; P0 fusion; P5 final/runtime; and P1/P2/P3/P4 disabled. A reduced sensor/process/backend mode is a
diagnostic mode and cannot make a qualification or campaign claim.

The required order is:

```text
ICRA-070 full-sensor qualification PASS + Supervisor review
  -> ICRA-071 pure-static guard hardening PASS + Supervisor review
    -> separate campaign authorization
```

ICRA-071 may not start ROS, perform GPU preflight, create a build/install, rerun live qualification or launch a
campaign. It must statically re-evaluate the retained ICRA-070 raw evidence under the hardened evidence rules.
If that evidence does not satisfy the stronger rules, campaign stays blocked and a separate live requalification
requires explicit Supervisor authorization.

## One canonical design target

Create a versioned v2 machine-readable design contract. It is the sole normative truth for:

- route, claim scope and mandatory sensor modalities;
- GPU backend and required GNSS ephemeris/timing policy;
- P0/P5 enabled state and P1/P2/P3/P4 disabled state;
- qualification case geometry and exact P5-6/P5-7 fixtures;
- conditional process specifications and required topic identities;
- full-sensor evidence semantics and terminal verdict vocabulary.

Runtime paths, resolved package locations and file hashes belong in a generated effective contract, not the
enduring design target. The v1 contract remains read-only historical input or is migrated explicitly; it cannot
remain a second active truth source. Python literal mirrors of normative JSON values are forbidden.

Any proposed reduction of a mandatory sensor, process, topic, backend, evidence invariant, fixture or claim
scope must produce `SCOPE_CHANGE_REQUIRES_USER_APPROVAL`. It cannot be treated as an ordinary config change.

## Typed resolver and projection chain

Provide one pure resolver with a typed result equivalent to:

```text
resolveQualificationCase(target, case_id, runtime_inputs)
  -> EffectiveQualificationContract
```

The result includes typed effective values, resolved runtime artifact paths/hashes, enabled processes, required
topics, evidence rules, case identity and a deterministic contract hash. Launch, runner and analyzer consume
this result instead of independently reimplementing target values.

The static verifier must check the complete chain in one direction:

```text
system target
  -> effective case config
    -> launch process/topic projection
      -> process/topic monitor contract
        -> normalizer/analyzer evidence contract
```

The target process set is not inferred from whichever processes a launch happened to create. Conditional
processes are projected from final effective values such as `use_gnss`, `run_validator`, `record_bag` and
`start_planner`, then compared exactly with the target and monitor sets. A generic `icra_p0_p5` profile combined
with a LiDAR-only, fallback-only, open-sky or other diagnostic scenario must be ineligible for qualification.

## Static verifier and adversarial matrix

Add a deterministic repository-local command equivalent to:

```bash
python3 scripts/dev_planner/verify_icra_route_contract.py \
  --route icra_p0_p5 --all-cases
```

It must require no ROS graph, GPU, build/install or network and must finish fast enough for pre-commit and CI.
It emits a typed failure and nonzero exit for at least these mutations:

- GNSS, IMU, LiDAR, ARAIM or either integrity source disabled;
- GNSS process removed or made optional;
- `lidar_only`, `fallback_only`, open-sky or incompatible scenario/profile binding;
- CPU mapping backend or synthetic ephemeris fallback;
- required process/topic gap or unexpected conditional projection;
- stale/invalid GNSS, zero GNSS use, zero LiDAR use, zero fused horizons or `n_sv_used=0`;
- RINEX/scenario path or SHA drift;
- route geometry, wall/floor/resolution, P5-6/P5-7 fixture, threshold/action or case-order drift;
- target/effective/launch/monitor/analyzer contract-hash mismatch.

Tests must prove the current three full-sensor cases pass and the historical GNSS-disabled bindings fail for the
correct reason. Mutations must be generated from one valid fixture so changing literal copies together cannot
make a bad route test-green.

## Sustained runtime-evidence semantics

Retained live evidence must not be qualified by cherry-picking a small number of good samples. ICRA-071 must
define and freeze an explicit startup/warm-up boundary using captured monotonic timestamps. For the complete
post-warm-up observation window it must retain total, eligible, good, bad and omitted counts, time span and
coverage ratios for P0 health and IntegrityReport rows.

Every post-warm-up ready/non-stale P0 generation must show fresh valid GNSS, positive GNSS and LiDAR predictor
use, and positive fused horizons. Every aligned IntegrityReport row must be finite/valid with `n_sv_used>0`.
Startup transients may be excluded only by the frozen warm-up rule; arbitrary later rows may not be filtered
out. The analyzer independently checks the raw rows, hashes, denominators and coverage rather than trusting a
normalized subset. ICRA-071 applies these rules statically to ICRA-070 raw evidence before campaign.

## Repository enforcement

ICRA-071 must make the same verifier unavoidable at the repository control plane:

- set a repository-relative `core.hooksPath=.githooks` for this workspace and provide a read-only verification
  command that fails when the path is absent or absolute/stale;
- fix pre-commit matching for repository-root paths including `launch/`, `config/`, `scripts/`, `test/`,
  `src/`, `include/`, `CMakeLists.txt` and `package.xml`;
- retain the mandatory `docs/CHANGES.md` and `docs/TRACEABILITY.md` synchronization rule;
- add a commit-message guard for one or more applicable `IAP-RQ-XXX` IDs;
- add tracked CI invoking the same route verifier and focused tests, without a separate implementation;
- add tests proving relevant staged paths trigger the guard and documentation-only/history files do not
  incorrectly authorize a campaign.

The current absolute hooks path `/home/dev/code/ws_iap/src/iap/.git/hooks` and the current `src/iap/...`
pre-commit matcher are known invalid configurations and are not accepted as satisfying `IAP-RQ-000`.

## State, provenance and supersession

The generated effective contract hash, design-contract hash, claim scope and invariant set must be recorded in
qualification manifests and the Supervisor handoff. A Supervisor verdict records which target/evidence hashes
it reviewed. Later contract versions must declare what they supersede; historical evidence remains immutable
and cannot silently inherit a newer claim.

`AGENT_STATE.md` remains Supervisor-owned. Builder tooling may emit a proposed machine-readable handoff report,
but may not edit gate status or authorize campaign. A passing static verifier authorizes only Supervisor review,
not the next state transition.

## ICRA-071 acceptance

ICRA-071 passes only when:

1. the v2 target and typed resolver are the sole active truth;
2. all three target cases pass the cross-layer verifier and every registered mutation fails closed;
3. retained ICRA-070 raw evidence passes the frozen sustained-evidence audit, or campaign remains explicitly
   blocked without relabelling the ICRA-070 result;
4. repository-relative hooks and CI invoke the same verifier;
5. focused and complete hermetic static tests pass with no ROS/GPU/build/install/campaign invocation;
6. Supervisor review records the exact contract hashes and explicitly decides whether campaign may be issued.
