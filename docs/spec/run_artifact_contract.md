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
optimizer or retiming early return. Only `candidate` and `attempt_failure` attach
that attempt's backend stages. Execution supervision has its own supplied assessment;
its candidate `final_check_state` is `not_applicable`.

The forest measurement driver preallocates one canonical run and passes the private
`run_lifecycle_owner:=driver` handoff. The canonical launch retains its ordinary
shutdown ownership for direct invocation. The driver finalizes after all owned jobs
stop, registers subordinate manifests, and marks startup failures as failed.
The 300-second OFF diagnostic at `log/20261007T092819Z_721` ended normally, with
terminal attempt 432/gen 2803 rejected by physical clearance after feasible dynamics.
Its older capture schema has the ownership limitations above; this is reference
evidence, not A acceptance or a task completion.
