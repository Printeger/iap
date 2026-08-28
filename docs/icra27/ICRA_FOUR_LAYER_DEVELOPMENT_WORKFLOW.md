# ICRA four-layer development and validation workflow

Status: **BLOCKED — LAYER 4 ICRA-076 REPAIR REVIEW / USER DECISION REQUIRED**

Requirements: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`,
`IAP-RQ-423`, `IAP-RQ-424`

Current disposition: repaired measurement/U95/local-evidence paths pass offline, but replay measured `r=0.5`
rather than frozen `r=0.75/b=1.5 m`, and the freeze omits tracked third-party JSON probe inputs. ICRA-076 remains
NOT PASS; ICRA-077 is blocked pending user repair-or-bypass direction.

## 1. Purpose

This document is the single process authority for getting the existing IAP modules running before applying
scientific or qualification controls. It groups the unchanged route-lock gate sequence into four layers. It
does not change the `P0_P4_V2_P5` route, research question, claims, arms, qualification scenes, campaign barrier
or the `ICRA_USER_ROUTE_LOCK_V1` sentinel.

Development runs in Layers 1–3 are repeatable. A failed run may be diagnosed, followed by a code/config repair,
incremental rebuild and a new run identity without an intermediate Supervisor Review. Formal one-shot,
held-out, full-hash and qualification rules begin only in Layer 4.

## 2. Layer map

| Layer | Existing gates | Goal | Exit |
|---|---|---|---|
| 1 — integration | ICRA-072A milestone | Make one real trajectory traverse P0 -> P4 selection -> EGO final -> P5 final -> normal publish -> P5 runtime | One analyzer PASS with complete same-identity live lineage |
| 2 — stabilization | ICRA-072B milestone | Turn the Layer 1 success path and failure boundaries into automated production-shaped regression | Happy path plus epoch/attempt/lineage/P5 fail-closed tests pass; ICRA-072 may close |
| 3 — effect diagnosis | ICRA-073..075 | Implement PRIMARY/MIRROR/FLAT_NULL, independent oracle, targeted optimization and exploratory/power inputs | Retained diagnostic evidence; no held-out claim |
| 4 — formal verification | ICRA-076..079 | Freeze protocol/build/thresholds/seeds, run held-out confirmation, G0D lineage and prospective P5 qualification | ICRA-079 Review PASS; campaign still needs a distinct user decision |

ICRA-080 is outside the four development layers and remains a separately approved campaign.

## 3. Layer 1 contract

Layer 1 has one outcome: a real, normally published trajectory with this order and identity:

```text
valid immutable P0 snapshot
  -> truthful closed collision
    -> natural P4-v2 risk-guide selection and application
      -> EGO final B-spline
        -> P5 final PASS before normal publication
          -> normal publication
            -> the same committed trajectory bound to P5 runtime
```

Attempt, segment, request, P0 snapshot/configuration, occupancy epoch, selected-guide hashes and final trajectory
identity must agree. GPU preflight, required-process health, occupancy-before-risk, EGO feasibility, P5
final-before-publish, runtime fail-closed behavior and owned-process cleanup remain mandatory.

Layer 1 may use `icra072_p4_selection_trigger_v1`. The trigger is development infrastructure: it may expose only
ordinary occupancy and the production P0 snapshot to P4. It is not the inverse-corridor fixture, may not inject
an expected route/oracle label, and cannot support an effect, qualification or paper claim.

The Builder may run `run-001`, `run-002`, and later unique identities until one passes. A run directory is never
overwritten, but there is no one-shot limit and no new build/install per run. Each run automatically records its
command, working directory, source commit, shared install root, GPU/process state and first missing pipeline
stage. Every run remains retained throughout Layer 1; any later compaction or retirement needs explicit boundary
Review authority.

## 4. Shared build and command authority

Layers 1–3 incrementally reuse the existing workspace roots:

```text
/home/dev/ws_iap/build
/home/dev/ws_iap/install
/home/dev/ws_iap/log
```

The workspace install retains simulation and message dependencies. Each build updates only `iap`, `plan_env`,
`traj_utils`, `path_searching`, `bspline_opt` and `ego_planner` through the canonical build entrypoint. No task,
attempt or run may create another build/install tree. Exact copyable build/run/analyze commands live only in the
ICRA development section of `README.md`; task and change documents link there instead of duplicating them.

Before Layer 4, remove and rebuild only those six packages' subdirectories in the shared build/install roots,
then freeze the relevant installed bytes. Do not delete or rebuild unrelated workspace packages. Any post-freeze
source or installed-byte change invalidates the formal freeze and returns work to the appropriate earlier layer.

## 5. Evidence and Review policy

- Layers 1–3 retain compact manifests, accepted/latest-blocker runtime evidence and ordinary logs, but do not
  require full build-tree hashes, one-shot execution or a Supervisor Review between repair iterations.
- Builder code commits still bind applicable requirements and synchronize `DEV_LOG.md`, `docs/CHANGES.md` and
  `docs/TRACEABILITY.md` once per logical layer changeset. Per-run commands are machine-recorded, not copied into
  multiple authority documents.
- Supervisor Review normally occurs once at a layer boundary. User decision 003 explicitly bypasses blocked
  Layer 2 into Layer 3 without converting it to PASS. Layer 3 does not become a scientific claim without Layer 4.
- Layer 4 alone enables full hashes, frozen SESOI/thresholds/seeds, held-out separation, non-retry and
  qualification controls.

## 6. Regenerable artifact retirement

The pre-workflow Builder checkpoint is `6a6bdd3e674dd58fafae4153e5a2b5cb5225d730`, archived as
`ARCHIVED_AS_FOUND / BLOCKED_TERMINAL_CHAIN_MISSING`: P0 and natural P4 selection ran, but terminal lineage/P5/
publication did not. Its raw, compact and registered live evidence remains. User-authorized cleanup removes only
the exact regenerable build/install roots listed in
`docs/icra27/dev/ICRA_REGENERABLE_BUILD_RETIREMENT_20260826.md` after that inventory is pushed and revalidated.

## 7. Retention matrix

| Artifact class | Layers 1–3 | Layer 4 | Retirement rule |
|---|---|---|---|
| Shared `/home/dev/ws_iap/{build,install,log}` | Reuse and incrementally update | Selectively rebuild six packages, then freeze relevant installed bytes | Never delete as part of historical-task cleanup |
| Per-run raw, compact, manifest, analysis and normal logs | Retain without overwrite | Retain under frozen protocol | No deletion without later explicit Review authority |
| P4-v1 scientific and registered historical live evidence | Immutable retention | Immutable retention | Never relabel or reuse as P4-v2 evidence |
| Historical task-local build/install trees | Delete only by the pushed literal inventory | Prohibited after freeze | Regenerate from retained source and commands if needed |
| Protected `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` | Untracked, unstaged, unchanged | Same | Never clean, stage or overwrite |
| Full source/install hashes | Optional development provenance | Mandatory frozen evidence | Re-freeze after any relevant byte changes |
