# IAP logging contract audit

Audit date: 2026-09-29

Audited branch/commit: `dev/iap_refactor` / `ab74e0d2bbddeabebda913fdec0f49563c8a018f`

Scope: repository-owned source, configuration, launch files, tests, documentation,
Git history, and the ignored `log/` tree. No external documentation was used.

## Executive conclusion

IAP did have a deliberate logging contract. The current-branch authority is the
2026-04-21 change `67fde2ef1f3ce4702c4d075e1f28a52887884485`
(`Add run-scoped IAP logging`), backed by
`docs/dev_ARAIM/araim_log.md:12-30,69-88,93-125` and
`docs/dev_ARAIM/araim_log_inventory.md:5-20`. Its intended shape was:

```text
<log_root>/<timestamp>/
  runtime/       # human-readable logs
  profiling/     # timing/performance data
  export/        # algorithm/debug/analysis exports
  metadata/      # run identity and configuration snapshot
<log_root>/latest -> <timestamp>/
```

The C++ implementation still preserves most of that *inside its own directory*.
The main regression is architectural rather than the loss of the original
writer: the 2026-09-29 canonical launch layer now allocates an outer timestamped
run, while the unchanged C++ `RunLogManager` allocates a second inner
timestamped run. Direct launch, planner, simulator, ROS, and C++ outputs are
therefore contained below the outer run in most canonical flows, but split
across multiple roots and schemas.

The result is best described as **semantic survival with structural drift**:

- Core IAP text logs, timing, GNSS/ARAIM/ICP exports, config snapshot, and
  `run_info.json` still use the original four categories when
  `RunLogManager` is initialized.
- All four canonical entrypoints have two run-directory owners, and none of
  their user-visible outer run roots directly implements the original fixed
  four-directory contract.
- Simulation has at least two export authorities: C++ IAP outputs under the
  inner log run and planner/simulator outputs under a separate outer export
  subtree.
- The primary analyzer and 19 lines in the root README still point to
  `src/iap/log/latest`, while canonical runs now live below
  `src/iap/log/runs/<entrypoint>/...`.
- Run retention is stated in `AGENTS.md` but is not implemented by either the
  current C++ manager or the new launch allocator. File rotation is not run
  retention.

This is not a case where every later writer escaped to arbitrary locations.
The launch refactor substantially improved containment and collision safety.
However, the repository no longer has one run identity, one artifact resolver,
one manifest, or one `latest` that all producers and tools agree on.

## 1. Historical authority and evolution

### 1.1 Before run-scoped logging

Commit `b695e8b7eb11cf31553c4fbc1a5d1585c3bc98d9` introduced the first
`logging.log_dir`-based spdlog files. Commit
`e5e5f30608a28324f79c213e85b105508ff3f36a` added timing CSV and changed
the shared root from `/tmp` to a machine-specific repository path. These
versions had no per-run directory and allowed executions to share files.

### 1.2 The most complete historical contract

Commit `7e1257e01ea8efab315679ab107408c04deaf6b0` contains the most complete
contract in `docs/REQ_LOG.md`, including:

- one independent timestamped directory per execution and no cross-run
  overwrite (`docs/REQ_LOG.md` at that commit, lines 5-12 and 57-90);
- package-root-relative resolution rather than cwd/config-copy-relative
  resolution (lines 45-55);
- the four directory categories and their meanings (lines 62-85);
- a unified `log.*` configuration, including `keep_last_n_runs: 20`
  (lines 92-177);
- legacy-path compatibility by basename only, with the category directory
  remaining authoritative (lines 230-239);
- one centralized `LogPaths` owner, automatic-output routing, retention, and
  no overwrite (lines 241-305);
- a boundary between automatic run artifacts and caller-controlled
  `save(path)` exports (lines 22-43 and 337-369).

Its implementation includes collision suffixes and retention in
`src/iap/common/log_paths.cpp` at the same commit, lines 95-127 and 198-230.

This commit is valuable design evidence but is **not an ancestor of current
HEAD**: `git merge-base 7e1257e HEAD` is
`cb61502baae67ce313f4b0b6867006f30523761f`, and
`git merge-base --is-ancestor 7e1257e HEAD` returns 1. It must therefore be
treated as a parallel-branch specification and design precedent, not as code
that silently evolved into the current manager.

### 1.3 Current-branch authoritative migration

Commit `67fde2ef1f3ce4702c4d075e1f28a52887884485` is an ancestor of current
HEAD and explicitly calls the work a “logging-contract migration” in
`docs/dev_ARAIM/araim_log.md:3-10`. It introduced:

- the `RunLogManager` singleton and four canonical path methods
  (`include/iap/util/run_log_manager.hpp:9-34`);
- UTC timestamp names with millisecond precision
  (`src/iap/util/run_log_manager.cpp:29-59`);
- one run directory below `logging.log_dir`
  (`src/iap/util/run_log_manager.cpp:110-119,151-158`);
- `runtime`, `profiling`, `export`, and `metadata`
  (`src/iap/util/run_log_manager.cpp:129-169`);
- `latest` maintenance (`src/iap/util/run_log_manager.cpp:171-195`);
- `metadata/run_info.json` with timestamp, process, run path, log root, cwd,
  config, build/source identity, host/user, git commit, and selected config
  paths (`src/iap/util/run_log_manager.cpp:197-261`);
- startup/shutdown config snapshots and `export/dump`
  (`apps/iap_rosnode.cpp:46-60,98-100,154-168`);
- spdlog routing to `runtime/` (`src/iap/util/logging.cpp:45-85`);
- timing routing to `profiling/` (`include/iap/util/timing_csv.hpp:29-68`).

The original inventory classified every then-known writer in
`docs/dev_ARAIM/araim_log_inventory.md:5-20`.

`git diff 67fde2e..HEAD -- include/iap/util/run_log_manager.hpp
src/iap/util/run_log_manager.cpp test/test_run_log_manager.cpp` contains only
file-mode changes. The central manager has not gained collision allocation,
atomic/concurrent `latest`, run retention, or launch-owned run adoption since
its first version.

### 1.4 Later ownership moved upward

Later code increasingly made the runner/launch layer responsible for paths:

- `01c9b5c9d6a7c59520bd7b3436f4eaddfab75753` added a full-stack
  `iap_log_root` constrained below a run's
  runtime root; the current code is
  `launch/_includes/full_stack_runtime.py:2575-2586,2622-2671`.
- `0922890d5ad6950e8492783243ac7ea197fb3730` added canonical launches and
  an outer layout of `runtime_config`, `export`, `logs`, and `dump`
  (`launch/_includes/profile_runtime.py:79-108`).
- `ab74e0d2bbddeabebda913fdec0f49563c8a018f` added collision-safe automatic
  outer runs and locked `latest` updates
  (`launch/_includes/run_directory.py:34-75,78-114`).

Those were useful improvements, but they did not teach the C++ manager to
adopt the already-created outer run. This is the exact point at which one
logging contract became two nested contracts.

## 2. Reconstructed contracts

### 2.1 Common core that is authoritative today

The current-branch migration document, implementation, README, and analyzer
agree on the following stable semantics:

| Requirement | Primary repository evidence |
|---|---|
| Every execution gets an isolated timestamped run | `docs/dev_ARAIM/araim_log.md:12-20,93-95`; `src/iap/util/run_log_manager.cpp:110-119` |
| Runtime logs are human-readable lifecycle/module logs | `docs/dev_ARAIM/araim_log.md:22-24`; `src/iap/util/logging.cpp:45-85` |
| Profiling is timing/performance data | `docs/dev_ARAIM/araim_log.md:25-26`; `include/iap/util/timing_csv.hpp:29-68` |
| Export is algorithm/debug/analysis output | `docs/dev_ARAIM/araim_log.md:27-28`; `docs/dev_ARAIM/araim_log_inventory.md:8-17` |
| Metadata describes the run and snapshots config | `docs/dev_ARAIM/araim_log.md:29-30,103-113`; `apps/iap_rosnode.cpp:46-60,164-168` |
| `latest` points to the most recent run | `docs/dev_ARAIM/araim_log.md:19-20,94-95`; `src/iap/util/run_log_manager.cpp:171-195` |
| Existing feature toggles remain respected | `docs/dev_ARAIM/araim_log.md:113`; examples at `src/iap/gnss/gnss_extension.cpp:210-235` and `src/iap/integrity/integrity_extension.cpp:230-289` |
| Important artifacts must not silently disappear | `docs/dev_ARAIM/araim_log.md:117-125` |

### 2.2 Historical extensions that were intended but are not present now

The parallel `7e1257e` contract additionally defined:

- `log.root_dir` relative to package root;
- `run_name` and collision suffixing;
- optional `latest`;
- `keep_last_n_runs`;
- a unified `log.runtime`, `log.profiling`, `log.export`, and `log.metadata`
  config instead of distributed `*_csv_path` keys;
- deprecation of old absolute paths;
- explicit `save(path)` retaining caller-controlled semantics.

Current HEAD implements only parts of these ideas. In particular, current
configuration still uses `logging.*`, `global.timing_csv_path`, and several
module-local `*_csv_path` keys.

### 2.3 What retention does and does not mean

There are three different policies that should not be conflated:

1. `rotate_logs`/`max_files` rotates files *within one runtime logger*;
   see `src/iap/util/logging.cpp:75-85`.
2. Run retention deletes or compacts old *run directories*. Current C++ and
   launch allocators do not implement this.
3. Formal evidence retention is a scientific/governance policy and must not be
   handled by an ordinary development-run garbage collector;
   see `AGENTS.md:135-140,148-153`.

The current `AGENTS.md:148` says ordinary development runs should retain only
the most recent few runs or seven days, but there is no corresponding retention
implementation in `RunLogManager` or `run_directory.py`. The historical
`7e1257e` implementation had `keep_last_n_runs`, but that code is not on the
current branch.

## 3. Current write-path inventory

This inventory distinguishes production/canonical runs, offline analysis,
formal evidence, tests, and retained legacy code. Those classes need different
retention and path rules.

### 3.1 Canonical run allocation and metadata

| Owner/mechanism | Current output | Classification/status | Evidence |
|---|---|---|---|
| `run_directory.py` | `log/runs/<entrypoint>/[<scenario>/]<UTC-ms>/`, plus parent `latest` | Outer canonical run identity; safe allocator | `launch/_includes/run_directory.py:34-75,78-114` |
| GLIO profile materializer | `<outer>/{runtime_config,export,logs,dump}` and `<outer>/launch_profile_manifest.json` | Second layout, not original four-category layout | `launch/_includes/profile_runtime.py:79-108,191-244` |
| Simulation launch | `<outer>/full_stack_manifest.json`, `runtime/`, `export/`, `bags/` | Outer full-stack bundle | `launch/iap_sim.launch.py:48-96` |
| Simulation environment | supplied output directory plus `scenario_manifest.json` | Run-local environment metadata | `launch/_includes/simulation_environment.launch.py:210-240` |
| Flight launch | `<outer>/flight_launch_manifest.json`, `planner/`, `estimator/` | Outer flight bundle; estimator contains another profile layout | `launch/iap_flight.launch.py:97-200,236-267` |

### 3.2 C++ IAP runtime writers

| Writer family | Actual mechanism/path when `iap_rosnode` runs | Category | Evidence |
|---|---|---|---|
| spdlog main/module logs | `<logging.log_dir>/<inner timestamp>/runtime/iap_*.log` | runtime | `src/iap/util/logging.cpp:45-85` |
| pipeline timing | inner `profiling/iap_timing.csv` | profiling | `include/iap/util/timing_csv.hpp:29-68` |
| GNSS factor diagnostics | inner `export/iap_gnss_factor_debug.csv` | export | `src/iap/gnss/gnss_extension.cpp:210-235,970-993` |
| Current Integrity outputs | inner `export/{iap_araim.csv,iap_araim_pl_decomp.csv,iap_lidar_araim_stage0.csv,traj_with_gnss.csv}` | export | `src/iap/integrity/integrity_extension.cpp:230-289,539-560` |
| CPU/GPU ICP health | inner `export/iap_icp.csv` | export | `src/iap/odometry/odometry_estimation_cpu.cpp:74-82,337-362`; `src/iap/odometry/odometry_estimation_gpu.cpp:92-104,296-326` |
| Demo8 truth ARAIM | inner `export/<csv_name>` | export | `src/iap/sim/demo8_truth_araim_extension.cpp:408-425` |
| map/factor dump | inner `export/dump/**` | export | `apps/iap_rosnode.cpp:98-100,154-168`; `src/iap/mapping/global_mapping.cpp:567-653`; `src/iap/mapping/sub_map.cpp:25-49` |
| config snapshot/run info | inner `metadata/config/**`, `metadata/run_info.json` | metadata | `apps/iap_rosnode.cpp:46-60,164-168`; `src/iap/util/run_log_manager.cpp:197-261` |
| simulation truth-vs-est | configured `metrics_csv_path`, not overridden by `RunLogManager` | outer/full-stack export when materialized; `/tmp` fallback otherwise | `src/iap/sim/sim_extension.cpp:144-151,197-214`; `launch/_includes/full_stack_runtime.py:2835-2838` |
| standalone experiment | inner export if manager initialized; otherwise `/tmp` fallback before initialization | export | `apps/iap_experiment.cpp:127-198` |
| environment-controlled ARAIM constructor | `IAP_ARAIM_DEBUG_CSV_PATH` or `/tmp/iap_araim_debug.csv` | dormant legacy bypass in current main integration | `include/iap/integrity/araim_debug.hpp:183-205`; main integration uses the explicit constructor at `src/iap/integrity/integrity_extension.cpp:230-250` |

### 3.3 Full-stack planner/simulator outputs

The maintained simulation runtime creates another per-process token below the
outer roots:

- runtime config: `<outer>/runtime/iap_<config>_test_planner_<pid_ms>/...`;
- planner/simulator export:
  `<outer>/export/test_planner_<experiment>_<scenario>_<pid_ms>/...`;
- IAP log root: `<outer>/runtime/iap_logs`, below which `RunLogManager` adds
  its own timestamp;
- configured GNSS, ARAIM, trajectory, ICP, simulator, P1/P2/P3/P4, validation,
  and manifest paths are constructed under the full-stack export directory.

Evidence:

- root resolution: `launch/_includes/full_stack_runtime.py:2560-2586`;
- log materialization: `launch/_includes/full_stack_runtime.py:2622-2671`;
- config copy and ICP path: `launch/_includes/full_stack_runtime.py:2674-2722`;
- simulator and integrity paths:
  `launch/_includes/full_stack_runtime.py:2801-2839,2841-2884`;
- planner debug paths:
  `launch/_includes/full_stack_runtime.py:3034-3076,3339-3382`;
- launch manifest: `launch/_includes/full_stack_runtime.py:4109-4268,4580-4598`.

The planner itself writes several related CSV families through paths supplied
by launch/config rather than `RunLogManager`: P1 optimizer/profile/context,
P2 candidate ranking, P3 reference bias, P4 risk/search/lineage, Gate-0
candidate/control-point evidence, and manager execution/forward-channel
diagnostics. Representative writers are:

- `src/iap/planner/bspline_opt/src/bspline_optimizer.cpp:4668-4766,4830-5006,5219-5415`;
- `src/iap/planner/plan_manage/src/p2_candidate_ranking.cpp:181`;
- `src/iap/planner/plan_manage/src/p3_reference_bias.cpp:99-134`;
- `src/iap/planner/plan_manage/src/gate0_qualification_writer.cpp:18-108`;
- `src/iap/planner/plan_manage/src/planner_manager.cpp:5125-5196,6642-7397,17212-18110`.

These writers are not inherently wrong: they accept explicit paths and are
usually contained by canonical launch. The problem is that no single public
artifact-path abstraction or test guarantees their category and containment.

### 3.4 ROS, launch, and build logs

ROS/launch logs are a separate mechanism. The canonical launch files do not
set `ROS_LOG_DIR`; the maintained full-stack include sets only
`XDG_RUNTIME_DIR` and Fast DDS environment values
(`launch/_includes/full_stack_simulation.launch.py:69-82`). Explicit
`ROS_LOG_DIR` containment exists only for frozen P4-G0C child environments
(`launch/_includes/full_stack_runtime.py:527-563,670-718`). Consequently an
ordinary canonical run can still produce ROS logs under the user's ROS default
location outside the run bundle.

The workspace-level `/home/dev/ws_iap/log` passed to `colcon --log-base` is a
build/test log and must remain separate from repository-local IAP runtime
artifacts; see `README.md:18-24`.

### 3.5 Offline tools, development runners, and formal results

- `tools/ana_log.py` reads a completed run and writes derived output to
  `<run>/analysis` by default (`tools/ana_log.py:44,70-72,8003-8015`). It is a
  derived-artifact writer, not a live logger.
- `scripts/dev_planner/run_gate0_qualification.py` and related runner/analyzer
  tools own explicit run/evidence roots, manifests, stdout, capture, summaries,
  and analyzer products. Representative lifecycle output is at
  `scripts/dev_planner/run_gate0_qualification.py:766-837,968`.
- `results/` contains development/formal experiment bundles governed by their
  individual protocols. It is not an alternate default for ordinary runtime
  logs (`AGENTS.md:135-140,153`).
- `launch/bp/` and deprecated programs such as
  `phase2_planner_integrity_evaluator` are retained historical surfaces. They
  must not define new canonical behavior (`launch/AGENTS.md:14-19`).

### 3.6 Tests

Unit tests should use temporary directories and remove them after the test;
this is already stated in `AGENTS.md:147`. There are still older planner C++
tests with literal `/tmp/*.csv` paths, for example
`src/iap/planner/bspline_opt/test/test_p1_integrity_cost.cpp:1761-2121`.
Those are test-isolation debt, not production log destinations.

## 4. How far the repository has drifted

### 4.1 Canonical launch topology: high structural drift

All four canonical entrypoints allocate an outer run with
`resolve_run_directory()` and eventually start `iap_rosnode`, which initializes
`RunLogManager` (`apps/iap_rosnode.cpp:46-55`). Because the C++ manager always
appends its own timestamp (`src/iap/util/run_log_manager.cpp:110-119`), **4/4
canonical entrypoints have two run-directory owners**.

Expected versus actual examples:

```text
# Original contract
<run>/runtime
<run>/profiling
<run>/export
<run>/metadata

# GLIO / GLIO+Integrity today
<outer>/runtime_config
<outer>/export
<outer>/dump
<outer>/logs/<inner timestamp>/{runtime,profiling,export,metadata}

# Simulation today (simplified)
<outer>/full_stack_manifest.json
<outer>/runtime/iap_<config>_test_planner_<token>/...
<outer>/runtime/iap_logs/<inner timestamp>/{runtime,profiling,export,metadata}
<outer>/export/test_planner_<experiment>_<scenario>_<token>/...
<outer>/bags

# Flight today (simplified)
<outer>/flight_launch_manifest.json
<outer>/planner/*.csv
<outer>/estimator/{runtime_config,export,dump,logs/<inner timestamp>/...}
```

Therefore **0/4 outer canonical run roots directly contain all four required
directories with the original meanings**. Containment below the outer root is
mostly good, but discoverability and single-run identity are not.

### 4.2 Export routing: split authority

The GLIO materializer writes configured CSV paths into `<outer>/export`
(`launch/_includes/profile_runtime.py:180-197`), but the initialized C++
writers override those values with `RunLogManager::export_path()`; examples are
`src/iap/gnss/gnss_extension.cpp:215-219` and
`src/iap/integrity/integrity_extension.cpp:232-275`. Thus the created outer
`export/` and the effective C++ `export/` are not the same directory.

Simulation additionally keeps simulator/planner outputs in the full-stack
export directory while C++ IAP outputs go to the nested manager export. A user
or analyzer cannot treat `<outer>/export` as the complete run export.

### 4.3 Config-path debt: measurable but partly masked at runtime

The following reproducible static query was run over tracked non-ICRA
configuration files:

```bash
rg -n --glob '*.json' --glob '*.yaml' --glob '*.yml' \
  --glob '!config/icra27/**' \
  '^\s*["A-Za-z][^:]*\b(log_dir|[A-Za-z0-9_]*csv_path|dump_path|output_dir|bag_output)["A-Za-z0-9_.-]*\s*:' \
  config
```

It finds **96 output-path assignments**: **51 point to `/tmp` and 45 point to
the machine-specific `/home/dev/ws_iap/src/iap/log...` tree**. None is a
portable relative sink declaration. This includes historical demo profiles as
well as canonical source profiles.

Canonical launch materialization rewrites many of these values before runtime,
so this number is not equivalent to 96 active leaks. It does prove that the
configuration layer never completed the intended move from absolute path
defaults to category/file-name declarations. Any direct/noncanonical execution
can still revive those destinations.

### 4.4 Documentation and analyzer drift: directly user-visible

- The root README has **19 lines** referring to the old
  `src/iap/log/latest` path, including the documented layout and analyzer
  commands (`README.md:774,811,814,934,974-1018,1058-1096`).
- Only the newer launch documentation explains
  `src/iap/log/runs/<entrypoint>/...` (`launch/README.md:14-23`), and the root
  README mentions it only at `README.md:60-62`.
- `tools/ana_log.py` still defaults to `PACKAGE_ROOT / "log" / "latest"`
  (`tools/ana_log.py:44,70`), but the automatic canonical `latest` is now below
  each entrypoint/scenario parent (`launch/_includes/run_directory.py:98-113`).
- The canonical launch contract test verifies allocation and one
  `runtime_config/config.json`, but does not assert a unified final artifact
  topology (`test/test_canonical_launch_contracts.py:309-385`).
- The C++ test has one case. It verifies the four child directories and only
  conditionally checks that `latest` is a symlink; it does not validate
  metadata content, collision safety, atomicity, retention, or refusal to
  delete a non-symlink `latest` (`test/test_run_log_manager.cpp:11-53`).

### 4.5 Existing ignored `log/` snapshot: evidence of historical mixing

The ignored tree is not a normative source and may contain outputs from older
commits, but it illustrates the operational result. At audit time:

- 9 top-level directories existed;
- 6 were complete four-category timestamped runs;
- 14 loose top-level files existed;
- no top-level `latest` symlink existed;
- total size was approximately 143 MiB.

This snapshot must not be used to judge the correctness of the new allocator,
but it shows why rules and automated conformance checks are needed.

### 4.6 Contract scorecard

Rather than assigning an arbitrary single percentage, the table below scores
independently verifiable properties.

| Contract property | Current status | Assessment |
|---|---|---|
| One isolated outer run per canonical invocation | Pass | New launch allocator is collision-safe |
| One owner of run identity | Fail | Launch allocator plus C++ allocator in 4/4 entrypoints |
| Fixed four categories directly under user-visible run | Fail | 0/4 canonical roots satisfy it |
| Runtime logs categorized | Partial | Correct category, nested below inner run |
| Profiling categorized | Partial | Correct category, nested; full-stack tests also use another runtime convention |
| Algorithm exports categorized | Partial | C++ category correct, but split from planner/simulator exports |
| Unified run metadata/manifest | Fail | `run_info`, launch profile, sim/flight, scenario, and planner manifests are separate |
| One unambiguous `latest` | Fail | Outer per-entrypoint links plus inner manager links; analyzer still expects old global link |
| No default `/tmp`/machine-specific production paths | Partial | Canonical rewriting masks many; source config and fallbacks retain them |
| Run retention | Fail | Policy text exists; no current allocator implements it |
| Formal evidence separated from ordinary logs | Pass | `AGENTS.md:135-140,153` and dedicated runner roots |
| User docs/tools match runtime | Fail | README/analyzer still target old root |

The fair summary is: **2 of 12 properties fully pass, 4 partially pass, and 6
fail at the canonical-run level**. This does not mean only one-sixth of the log
code works; it means the system-level contract is no longer end-to-end. The
inner C++ logger is largely intact, while coordination around it has drifted
substantially.

## 5. Recommended contract for the refactor

The next refactor should choose one run directory at the launch/process
boundary and make every automatic writer adopt it. Do not create a third
manager.

### 5.1 Canonical v2 layout

```text
<run_dir>/
  metadata/
    run_manifest.json
    config/...
    launch/...
  runtime/
    iap_*.log
    ros/...
  profiling/
    iap_timing.csv
  export/
    estimator/...
    integrity/...
    advisory/...
    planner/...
    dump/...
  capture/                 # optional rosbag/raw topic capture
  analysis/                # optional derived reports; reproducible, not raw evidence
```

The four original categories remain mandatory. `capture/` and `analysis/` are
optional explicit extensions because bags and derived reports do not fit
cleanly into runtime/profiling/export/metadata. No producer may invent another
top-level directory without changing the contract.

The outer entrypoint/scenario grouping remains useful:

```text
<IAP_RUN_ROOT>/<entrypoint>/[<scenario>/]<run_id>/...
```

Only the leaf `<run_id>` is a run. Grouping directories may have `latest`; a
run itself must never contain another timestamped run or another `latest`.

### 5.2 Ownership rules

1. `resolve_run_directory()` (or its successor) is the only allocator for
   canonical launch runs.
2. C++ `RunLogManager` must gain an **adopt existing run** mode. In that mode it
   creates/checks categories directly below `<run_dir>` and must not append a
   timestamp or update a second `latest`.
3. Direct `ros2 run iap ...` may let `RunLogManager` allocate a run, because no
   launch boundary exists. Allocation and adoption are mutually exclusive.
4. All automatic writers receive category paths through one public resolver.
   Config may enable/disable and choose a basename, but must not choose an
   arbitrary production directory.
5. Explicit user export/save APIs may honor an explicit path. Their API and log
   message must identify them as external exports, not automatic run artifacts.

### 5.3 Category rules

- `runtime/`: human-readable process/module/ROS lifecycle logs only.
- `profiling/`: timing, latency, throughput, memory, and performance counters.
- `export/`: algorithm/debug tables, estimator/integrity/advisory/planner
  diagnostics, trajectories, map/factor dumps, and generated scenario inputs.
- `metadata/`: immutable identity/context—entrypoint, scenario, run ID,
  start/end UTC, status, command/arguments, PID/host/user, git commit and dirty
  state, build/install identity, config snapshots/hashes, enabled modules,
  and an artifact index.
- `capture/`: optional raw bags/topic capture.
- `analysis/`: reproducible post-run summaries/figures. It may be deleted and
  regenerated without changing raw-run identity.

### 5.4 Manifest and lifecycle

Use one authoritative `metadata/run_manifest.json` with a versioned schema.
Subsystem manifests may exist below `metadata/`, but the run manifest must
index them. It should start with status `STARTING`, move to `RUNNING` only after
ready conditions, and finish as `PASS`, `FAIL`, `HOLD`, `INTERRUPTED`, or
`INCOMPLETE`. Manifest writes should be atomic.

`metadata/run_info.json` can remain as a compatibility alias/schema during
migration, but there must not be two independent sources of run identity.

### 5.5 Retention

- Ordinary successful development runs: keep the newest 3 and anything newer
  than 7 days by default. The exact `count OR age` behavior must be stated and
  tested.
- Ordinary failed runs: compact only after finalization, retaining manifest,
  summary, relevant runtime error excerpt, and necessary plots, as already
  requested by `AGENTS.md:149-152`.
- Active, incomplete, symlinked, locked, or nonconforming directories: never
  delete automatically.
- `results/**`, formal/frozen/evidence runs, explicit user exports, and
  externally configured `IAP_RUN_ROOT` data: no automatic deletion unless
  their own protocol explicitly opts in.
- Retention must operate only on directories containing a recognized run
  manifest and must never follow symlinks.

### 5.6 Prohibitions

- No automatic production output may default to `/tmp`, cwd, repository root,
  `log/res`, or a machine-specific absolute path.
- No new `*_csv_path` may control an arbitrary directory. Prefer
  `*_csv_enable` plus a basename or artifact key.
- No component included by a canonical launch may allocate a nested run.
- No run may overwrite/reuse an existing output directory.
- No runtime writer may write directly to `results/`; only an explicitly named
  experiment/evidence runner may promote or copy a finalized bundle there.
- No test may use a shared fixed file; use a test-owned temporary directory.
- No documentation example may rely on a global `log/latest` when `latest` is
  scoped by entrypoint/scenario.

## 6. Where the rules should live

The rules should not live in README alone. Use three layers with one declared
authority:

1. **`docs/spec/run_artifact_contract.md` — normative contract.** Put the full
   schema, categories, lifecycle, retention, explicit-export exception, and
   migration policy here. This keeps detailed policy reviewable without making
   `AGENTS.md` enormous.
2. **`AGENTS.md` — mandatory agent guardrails.** Include the short MUST/MUST NOT
   version and link to the spec. Agents are required to read this file before
   editing, so this is where enforcement language belongs.
3. **`README.md` and `launch/README.md` — user operation.** Explain where runs
   appear, how to set `IAP_RUN_ROOT`, how to find `latest`, how to analyze a
   run, what is retained, and how formal evidence differs. Do not duplicate all
   implementation rules.

During migration, the spec should list the known grandfathered deviations from
this audit. The “no new deviation” rule can take effect immediately; claiming
full compliance should wait until the dual allocator and analyzer paths are
fixed.

## 7. Proposed `AGENTS.md` text

The following is intentionally compact and enforceable:

```markdown
## Run artifacts and logging (mandatory)

The normative contract is `docs/spec/run_artifact_contract.md`.

- Every canonical invocation has exactly one leaf `run_dir`, allocated by the
  top-level entrypoint. Included launches and processes adopt that directory;
  they must not append another timestamp or create another `latest`.
- Automatic outputs may write only below that run directory:
  human logs -> `runtime/`, performance data -> `profiling/`, algorithm/debug
  products -> `export/`, identity/config/manifests -> `metadata/`, raw capture
  -> `capture/`, derived reports -> `analysis/`.
- Production defaults must not write to `/tmp`, cwd, repository root,
  machine-specific absolute paths, `log/res`, or `results/`. Tests use owned
  temporary directories. Formal evidence enters `results/` only through an
  explicitly authorized evidence protocol.
- New writers must use the shared run-artifact resolver; do not add standalone
  timestamp generation, directory allocation, `latest` handling, or arbitrary
  `*_csv_path` defaults. Explicit caller-supplied `save(path)` APIs are the only
  exception and must be documented as external exports.
- A run must contain `metadata/run_manifest.json` and config provenance. It
  must never reuse or overwrite an existing run directory. Manifest/artifact
  writes that establish identity must be atomic.
- Ordinary-run retention must never touch active/incomplete runs, symlinks,
  formal/frozen evidence, or unrecognized directories. File rotation is not
  run retention.
- Any change to a writer, launch path, manifest, analyzer default, or retention
  policy must update the contract tests and the user-facing logging section.
  The tests must prove one run owner, containment, category placement,
  collision behavior, `latest`, manifest completeness, and forbidden-path
  absence.

Known pre-contract paths are migration debt, not precedent. Do not copy them
into new code.
```

## 8. Proposed README content

README should answer operational questions, not govern code review. Suggested
content:

```markdown
## Logs and run artifacts

Each canonical launch automatically creates one run directory:

`<IAP_RUN_ROOT>/<entrypoint>/[<scenario>/]<UTC-run-id>/`

In a source workspace `IAP_RUN_ROOT` defaults to `src/iap/log/runs`. Set it
once for deployment, for example `export IAP_RUN_ROOT=/data/iap_runs`.

Every run uses the same layout:

- `runtime/`: IAP, ROS, and module text logs
- `profiling/`: timing/performance tables
- `export/`: estimator, integrity, advisory, planner, trajectory, and dump data
- `metadata/`: run manifest, command/source/build identity, and config snapshots
- `capture/`: optional bag/raw capture
- `analysis/`: generated reports and figures

Use the entrypoint/scenario-scoped `latest` link, for example
`src/iap/log/runs/glio/latest` or
`src/iap/log/runs/iap_sim/fused_nominal/latest`. Analyze an explicit run with
`python3 tools/ana_log.py --run <run_dir>`.

Ordinary development logs are disposable and subject to the documented
retention policy. Formal/frozen evidence lives under an explicitly authorized
`results/` protocol and is never removed by ordinary log retention.
```

`launch/README.md` should additionally list the exact `latest` path for each of
the four entrypoints and the required/optional artifacts of each launch
contract.

## 9. Enforcement needed so future agents actually comply

Documentation is necessary but insufficient. Add automated gates:

1. A canonical layout test that materializes all four launch profiles and
   asserts exactly one run ID, mandatory categories, one manifest, and no nested
   timestamp directory.
2. A sink-containment integration test using a temporary `IAP_RUN_ROOT`, then
   checking every created regular file resolves below the allocated run.
3. A static test rejecting new production literals matching `/tmp/*.csv`,
   `/home/.../log`, root-level `*.csv`, direct timestamp generation, and new
   unmanaged `ofstream`/`write_text` sinks unless allowlisted with rationale.
4. Manifest schema tests for start/final states, git/config/build identity, and
   artifact indexing.
5. Collision and concurrent `latest` tests. Preserve the stronger outer-helper
   tests at `test/test_canonical_launch_contracts.py:309-361`.
6. Safe retention tests using only temporary directories: active/incomplete,
   symlink, unknown, formal, newest-N, and age-bound cases.
7. Analyzer tests that resolve entrypoint/scenario `latest` and read the exact
   canonical layout.

The migration should be incremental:

1. freeze the v2 contract and add “no new deviation” static checks;
2. let `RunLogManager` adopt the outer run and remove its inner timestamp;
3. unify profile/full-stack/flight paths below the fixed categories;
4. route ROS logs into `runtime/ros`;
5. consolidate manifests and update `ana_log.py`/README;
6. migrate source config from absolute paths to enable flags + basenames;
7. implement safe ordinary-run retention last.

This sequence preserves current logging content while removing duplicated path
ownership before adding deletion behavior.

## 10. Reproduction commands

```bash
# Establish audited state
git rev-parse HEAD
git branch --show-current

# Show current-branch contract introduction
git show --stat 67fde2ef1f3ce4702c4d075e1f28a52887884485
git show 67fde2e:docs/dev_ARAIM/araim_log.md | nl -ba
git show 67fde2e:docs/dev_ARAIM/araim_log_inventory.md | nl -ba

# Show parallel, more complete historical design and prove ancestry status
git show 7e1257e:docs/REQ_LOG.md | nl -ba
git show 7e1257e:src/iap/common/log_paths.cpp | nl -ba
git merge-base 7e1257e HEAD
git merge-base --is-ancestor 7e1257e HEAD

# Prove RunLogManager has not semantically changed since introduction
git diff 67fde2e..HEAD -- \
  include/iap/util/run_log_manager.hpp \
  src/iap/util/run_log_manager.cpp \
  test/test_run_log_manager.cpp

# Count stale README references
rg -n 'src/iap/log/latest|/home/dev/ws_iap/src/iap/log/latest' README.md

# Inventory configured sink paths
rg -n --glob '*.json' --glob '*.yaml' --glob '*.yml' \
  --glob '!config/icra27/**' \
  '^\s*["A-Za-z][^:]*\b(log_dir|[A-Za-z0-9_]*csv_path|dump_path|output_dir|bag_output)["A-Za-z0-9_.-]*\s*:' \
  config

# Inspect the ignored log-tree shape without modifying it
find log -mindepth 1 -maxdepth 1 -printf '%y %f\n' | sort
```
