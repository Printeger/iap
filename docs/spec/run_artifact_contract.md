# IAP Run Artifact Contract

Status: normative
Schema: `iap_run_artifact_v1`

This document is the sole authority for artifacts produced by an IAP run.
`AGENTS.md` contains only enforcement rules; user-facing READMEs contain only
commands and navigation.

## 1. Terms and ownership

- **run root**: the directory configured by `IAP_RUN_ROOT`.
- **run ID**: a collision-safe UTC identifier of the form
  `YYYYMMDDTHHMMSSZ_mmm`, optionally followed by `_NN`.
- **run directory**: exactly `<run root>/<run ID>`.
- **run owner**: the top-level launcher, or `RunLogManager` for a standalone
  process. Exactly one owner allocates and finalizes a run.
- **adopter**: a child process that receives `IAP_RUN_DIR` and writes only
  inside that run.
- **automatic artifact**: any output created as a side effect of running IAP.
- **explicit export**: a user-requested `save(path)` or offline conversion to
  a caller-selected path.

An automatic artifact MUST be contained by the current run directory. An
explicit export MAY be outside it, but the owner MUST record its absolute path,
kind, requester, and timestamp in the run manifest. Configuration defaults do
not count as explicit user intent.

## 2. Public configuration and allocation

`IAP_RUN_ROOT` is the only user-facing persistent run-location setting. In a
source or symlink-install workspace it defaults to `<iap source>/log`; in an
installed package it defaults to `${XDG_STATE_HOME:-~/.local/state}/iap/log`.

`IAP_RUN_DIR` is an internal, absolute leaf-directory contract passed by a run
owner to child processes. A process receiving it MUST adopt that exact run and
MUST NOT allocate another timestamp or update `latest`.

The owner creates the run with atomic `mkdir`. Collisions append `_01`, `_02`,
and so on. Allocation and `latest` updates are serialized by a lock in the run
root. `latest` is an atomically replaced relative symlink. A non-symlink named
`latest` is an error and MUST NOT be removed.

The former user-facing launch argument `output_dir` has been removed from all
four canonical entrypoints. New tools and documentation MUST use
`IAP_RUN_ROOT`; `run_dir` exists only for private owner-to-child adoption.

## 3. Required layout

Only the following four first-level directories are permitted:

```text
<run root>/<run ID>/
├── runtime/
├── profiling/
├── export/
└── metadata/
```

The canonical layout is:

```text
runtime/
├── iap_*.log
└── ros/

profiling/
├── iap_timing.csv
└── planner_timing.csv

export/
├── glio/
├── current_integrity/
├── advisory/
├── planner/
├── simulation/
├── capture/
└── analysis/

metadata/
├── run_manifest.json
├── config/
├── processes/
└── manifests/
```

GNSS factor, ICP, estimator trajectory, map, and factor dumps belong to
`export/glio/`. Current ARAIM and PL/AL/IM evidence belongs to
`export/current_integrity/`. Future-integrity and risk-grid evidence belongs to
`export/advisory/`. Candidate, P1-P5, lineage, certification, and HOLD evidence
belongs to `export/planner/`. Simulation truth and metrics belong to
`export/simulation/`; rosbags to `export/capture/`; offline reports and plots to
`export/analysis/`.

`ROS_LOG_DIR` MUST be `<run>/runtime/ros`. Effective and source configuration
snapshots MUST be under `metadata/config/`. No CSV or manifest may be written
directly in the run directory.

## 4. Manifest and lifecycle

`metadata/run_manifest.json` is the only run-level authority. The owner writes
it atomically using a temporary sibling and rename. It contains at least:

- `schema_version`, `run_id`, `run_root`, and `run_dir`;
- `entrypoint`, optional `scenario`, and enabled `modules`;
- `started_at_utc`, nullable `ended_at_utc`, and `lifecycle`;
- `safety_outcome`;
- source commit and dirty state, build identity, and host identity;
- configuration snapshot and subordinate-manifest references;
- `run_class`, `retention_class`, and `external_exports`.

Lifecycle values are `active`, `completed`, `failed`, and `interrupted`.
Safety outcomes are `executable`, `hold`, `not_applicable`, and `unknown`.
HOLD is a valid safety result and is not a process failure.

Only the owner updates the run manifest. Children write process records under
`metadata/processes/` or immutable subordinate manifests under
`metadata/manifests/`. The historical `metadata/run_info.json` is a read-only
input format for migration tools and MUST NOT be generated for new runs.

## 5. Path API rules

Artifact APIs accept a category/module and a relative leaf name. They MUST
reject absolute names, `..`, and any resolved path outside the run. Modules
MUST NOT create their own timestamp directory, infer a root from cwd, or honor
the directory component of a legacy `*_csv_path` setting. During migration a
legacy path may contribute only its basename and MUST produce one warning.

Machine-specific paths and `/tmp` output defaults are forbidden in maintained
configuration and production writers. Input resources are configured
independently and are not artifacts.

## 6. Analysis and compatibility

`tools/ana_log.py` defaults to `<source>/log/latest`. New analysis artifacts
are written only below `export/analysis/` and MUST NOT overwrite online module
outputs. It may read the historical flat `export/*.csv` layout and
`metadata/run_info.json`, but compatibility reads do not authorize new writers
to produce those layouts.

## 7. Retention

Before allocating a new ordinary development run, the owner may prune old
runs. The repository-local default root enables this automatically. A root
selected through `IAP_RUN_ROOT` outside the repository requires
`IAP_RETENTION_ENABLED=1`.

Retention keeps the newest three finalized runs and every run ended within the
last seven days. A run is eligible only when it is both older than seven days
and outside the newest three.

The collector examines only direct, non-symlink children whose names match the
run-ID grammar and whose manifest says `run_class=development`,
`retention_class=ordinary`, a terminal lifecycle, and a non-null end time. It
MUST skip active/locked, malformed, formal-evidence, protected, symlinked, and
unrecognized directories. It MUST never traverse into `results/` or delete an
explicit external export. Under the root lock it revalidates the candidate,
atomically renames it to a root-local quarantine name, and only then removes
the quarantined tree. A failure is reported and does not block allocation.

## 8. Migration

Migration proceeds in this order: freeze this contract; adopt the outer run in
C++; normalize GLIO/simulation/flight categories; contain ROS logs and
manifests; migrate analyzer and READMEs; remove temporary and machine paths;
then enable retention. A checked allowlist may temporarily describe existing
violations, but new violations are forbidden and the allowlist must be empty at
the end of migration. Formal or frozen evidence is never rewritten to look
like a new run.

### Curve capture and forest driver authority (2026-10-07)

A candidate geometry revision invalidates its previous final assessment before any
optimizer or retiming early return. Attempt-owned `candidate`, `attempt_failure`,
`attempt_failure_curve` and opt-in `committed_<attempt_id>` attach that attempt's
backend stages. With `capture_failure_map` enabled, the existing asynchronous
writer also captures up to 32 committed candidates, each binding the original
frozen map, prediction input, full production terminal set, selected guide,
curve stages and final assessment. Commit means manager candidate acceptance;
publication, server activation and executed command identity require separate
feedback evidence. These successful captures never replace `terminal_final`
while a PlanningView is active. Execution supervision has its own supplied assessment;
its candidate `final_check_state` is `not_applicable`.

The forest measurement driver preallocates one canonical run and passes the private
`run_lifecycle_owner:=driver` handoff. The canonical launch retains its ordinary
shutdown ownership for direct invocation. The driver finalizes after all owned jobs
stop, registers subordinate manifests, and marks startup failures as failed.
The 300-second OFF diagnostic at `log/20261007T092819Z_721` ended normally, with
terminal attempt 432/gen 2803 rejected by physical clearance after feasible dynamics.
Its older capture schema has the ownership limitations above; this is reference
evidence, not A acceptance or a task completion.

### Production Curve replay and boundary refinement

`curve_backend_replay` reads captured candidate stages, original planning/cloud
and motion times, frozen voxel flags and explicit parameter YAML. `retime` replays
the old four-check symptom; `refine` starts at the captured optimized boundary;
`backend` also repeats guide initialization and rebound optimization, then the
bounded candidate-correction loop through the same production
`EGOPlannerManager::correctCurveCandidate` authority. These are read-only
mechanism experiments. No ROS server publication or freshness renewal is available.
Exit zero requires `physical_geometric_candidate_valid`: normal backend termination,
dynamics, full physical checking and geometric retention. Each constituent verdict
remains separately retained. Replacing a candidate clears the preceding dynamics
conclusion before invoking its solver; a failed new backend leaves the current
final check `not_checked`, while earlier stage evidence remains historical. `execution_authorized` is always false. Missing frozen
predictor input limits this replay to `OFF_GEOMETRY_ONLY`; explicit guidance ON
is rejected for `backend`/`initialize`.

`audit` evaluates the production geometric guide-retention check independently
for every already captured stage and its owned guide. The result schema is
`iap_curve_stage_audit_v1`, identity `CAPTURED_STAGE_GEOMETRY_DIAGNOSTIC`.
Original attempt, physical generation and acquisition times remain unchanged.
No solver, online budget, freshness update or execution authorization is involved.
`risk_evidence=NOT_AVAILABLE` explicitly excludes reconstructed PL qualification.
Exit zero requires all stages checked and route preserved; rejection is a
completed diagnostic, not a failed artifact capture. The wrapper registers
command, source, parameter, binary and frozen input hashes before execution.

`initialize` starts at the captured `guide_fit` and invokes production fitting.
It requires a stage-owned guide and preserves original start P/V/A, target and
physical/time inputs. Stage `terminal_stop` records the fixed-task stop policy
when known. Historical nonzero terminal V proves continuation; historical zero
without an explicit policy, or a policy conflicting with nonzero V, is rejected.
Captured controls precede refitted controls; target velocities and policy source
are named separately. Captured remaining resources are retained even though the
fit is repeated. Full physical, dynamic and geometric-retention verdicts are
independent; absent PL stays `NOT_AVAILABLE`. This replay does not publish,
repeat target-search/shortening policy or authorize a server switch. Backend
correction, original remaining repairs, boundary binding, checked retiming and
refine share the production policy; raw PL qualification is excluded.

The observed attempt 12/gen 260 in run `20261007T094106Z_545` rejected four
boundary-only stretches with ratios 2.469/1.374/1.258/1.195. Rebinding endpoint
P/V/A without updating adjacent interior controls reproduces that dynamic failure.
Refine solves it in the translated free-map regression. The real frozen forest
replay with production clearance planes still fails dynamics and exhausts the
original repair allowance (`20261007T101503Z_947`); even an explicitly isolated
full-budget experiment fails (`20261007T101513Z_849`). A is not passed. Earlier
replays `20261007T100131Z_836` / `20261007T100133Z_005` omitted clearance planes
and are limited diagnostic evidence, not equivalent production replays.

The production seam rebinds constraints and refines the same guide after each
useful stretch, keeping four dynamics checks and the shared steady budget.
Adding clearance constraints consumes the existing CurveCorrection allowance.
Solver termination and its provisional physical verdict remain separate;
publication still requires the independent complete check. No unused stretch
follows the last failed check.

Default replay uses the captured stage's remaining 1.5-second budget and remaining
three repair slots. `--isolated-budget` is explicitly separate mechanism evidence.
Each new stage owns its guide; historical stages must match the supplied guide
endpoint. Budget-interrupted final checks are `incomplete`, with sampled count and
checked interval. Replay starts at the captured guide/optimized spline, so it does
not reproduce pre-guide polynomial initialization, commit-window timing, or the
previous server trajectory's connection check. Those require live evidence.
No replay result declares full online attempt equivalence or A acceptance.

Canonical simulation saves `metadata/config/planner_parameters.json`. Failure
snapshots additionally retain `virtual_ceiling_height_m` and `inflation_radius_m`;
old snapshots require explicit historical parameter supplementation and remain
historical evidence. The captured-boundary regression uses a translated free map
only to verify dynamics and exact endpoint P/V/A; it is not forest acceptance.

### Channel selection costs and proof evidence

Planning behavior is authoritative in [EGO flow](ego_based_planning_flow.md#local-terminal-region-and-mandatory-guide-c-implementation-in-progress).
Failure JSON and per-attempt planning CSV append `search_path_cost_m`,
`search_path_length_m`, `search_risk_cost_m`, `search_terminal_cost_m`,
`search_optimality_proven`, `search_budget_exhausted` and `search_advisory_changed`.
The objective includes both real connectors and the endpoint-to-fixed-task distance
exactly once. Unknown/sentinel values retain their status; raw HPL/VPL and versions
are unchanged. Cost fields without a search are JSON null. A successful guide can
be explicitly unproven; this field is not execution authorization. Historical cost
fields before this contract used lattice-step units and must not be compared as metres.


Curve stages and `final_check` append `guide_retention`: checked/budget status,
original risk version, guide/actual metric length and risk costs, valid spatial
fractions, maximum sampled 3D guide deviation, geometric corridor, quadrature uncertainty,
route/risk loss and query count. `comparable_model_cost` separately describes stable finite model multipliers, including
UNKNOWN 1.5; it can support preference repair without valid PL.
`source_contribution_coverage` is explicitly `not_available` until the production
source model supplies it. `comparable_valid_risk` describes numerical sample
coverage only; it does not grant independently validated PL or source contribution
qualification. `route_checked` belongs to the captured candidate revision. The
attempt CSV includes the same metrics for successful and rejected candidates;
a new geometry invalidates the preceding final assessment. Unchecked values have
`checked=false`, and must not be interpreted as a zero-risk curve. Export continues
through the existing bounded writer and shared resolver.

`guide_retention.risk_version` is the first frozen model sample's version, including
all-UNKNOWN input. Search and audit use `gridAdvisoryCostMultiplier` for the same
warning/degraded fallback multiplier 3; raw PL/classification remain intact.


### Historical simulation input failures

The canonical `rinex_nav_file` input is copied to `metadata/config/historical_nav.rnx`;
its hash, fixed UTC epoch, requested GPS+BDS and unqualified Advisory status are
recorded in `metadata/manifests/full_stack.json`. Sensor/consumer ROS time uses
that unique historical `/clock`; run IDs and lifecycle timestamps remain actual UTC.
Record/capture tools wait for this owner clock identity and reject missing/unknown
contracts. Point-cloud clock evidence is explicitly `POINTCLOUD_HEADER_ONLY`;
it cannot substitute for the complete frozen input recorder.

An unexpected historical clock-owner or GNSS exit writes a per-module
`historical_input_failure_<module>.json` and the immutable first
`historical_input_failure.json` through the shared resolver. Normal launch
shutdown does not create this failure. The shared `finalize_run` returns the
chosen lifecycle and gives saved input failure priority over apparent successful
launch exits or later interrupts; both launch and driver remain single-owner
finalizers. Individual child failures and source hashes remain reviewable.

GNSS clock evidence uses `export/glio/constellation_clock.csv`, registered by
`metadata/manifests/gnss_coordinate_dynamics.json` v2. It names actual system,
state/epoch/source identity, optimized bias/drift, system-minus-GPS difference
and joint marginal difference covariance. The original covariance time remains
the state time; unavailable reference/covariance fields stay empty. This sampled
diagnostic does not qualify a frozen state or grant Advisory validity.

The forest driver binds installed binaries, plugins and the core library to
their workspace Release build hashes before starting processes. An ordinary
installed copy is permitted when byte-identical or when its only byte changes
are CMake removing the build RPATH string and dynamic entries. All other bytes
must match; a common GNU build ID alone is insufficient. Both original hashes
and the match method are retained. Build-cache identities join command/config/input hashes in
the subordinate runtime identity manifest.

新预测冻结传输为`iap_prediction_input_v7`，显式`clock_model`登记活动星座
偏差模型、`gnss_fault_model`登记单星＋整星座故障集合，两者参与输入hash；
旧v1–v6保留历史读取身份，v6保留真实钟差模型及legacy_single_satellite_v1，
生产GridMap拒绝旧版本及不支持模型。原wire版本不能通过声明或重编码取得v7资格。
诊断矩阵导出分别记录单星与整星座最坏假设身份、退化星座与假设总数；
该身份没有授予优化状态时间／covariance／外参不确定性及联合米数资格。
元数据对未识别模型写unsupported，原字段完整保留在input.bin；不能重赋新鲜度。

Uniform retiming of the same frozen guide/control indexing uses
`BsplineOptimizer::rebindAfterUniformRetime`. Actual-sample physical planes,
guide preferences and bilateral corridor constraints retain their normalized
`t/dt` cubic weights; costs use the current rebound endpoint controls. The
manager and offline production replay share this binding. A changed control
count is rejected before mutation. An ordinary new target fit still clears
these constraints. This binding neither spends an additional repair nor grants
execution; independent physical, dynamic, route and publication checks remain.


Server lifecycle logs explicitly bind ROS clock values: successful pending
withdrawal records `receipt_ros_time_s` and `effective_ros_time_s`; ignored
withdrawal records receipt time and the current active ID; scheduled activation
records `activation_ros_time_s` and effective time. These values come from the
same authoritative callback clock used for its queue decision. The ROS logger
prefix is a wall UTC diagnostic and must never be subtracted from a historical
ROS effective time. Legacy logs without explicit ROS receipt time retain IDs,
commands and disposition evidence but no qualified historical receipt margin.
Logging fields do not authorize, acknowledge, or undo an active trajectory.


An explicit `/sim/pause` diagnostic records each requested Bool value, original
steady and ROS times, the unique clock publisher and pause subscriber counts,
and observations after draining in-flight messages. Record the actual historical
span separately from the steady task window. A deliberate pause diagnostic is
not a normal paired task or calibration run. Repeated commands with frozen ROS
stamps do not advance trajectory time; message counts alone do not establish
clock advancement. Startup discovery counts and abnormal jump qualification
remain separate evidence.
