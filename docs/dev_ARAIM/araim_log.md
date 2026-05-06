You are working in my current repository.

Task:
Migrate the current logging system so that it follows the same run-artifact organization style as the target repo `Printeger/iap` on branch `dev/ct-iap`.

This is not a superficial logger API rewrite.
This is a logging-contract migration.

Core requirement:
Before changing code, you must first inventory all existing logs and classify them into the target run-directory structure.

Target run-directory structure:
<log_root>/<timestamp>/
  runtime/
  profiling/
  export/
  metadata/

And maintain:
<log_root>/latest

Meaning of each category:
- runtime:
  Human-readable runtime logs such as INFO/WARN/ERROR, lifecycle logs, module state transitions, startup/shutdown messages, important events.
- profiling:
  Performance and timing outputs such as pipeline timing, callback timing, optimizer timing, latency, throughput, fps, memory/perf counters.
- export:
  Algorithm/debug/analysis outputs such as CSV dumps, factor debug outputs, residual exports, GNSS/LiDAR/ARAIM debug tables, trajectory evaluation data, quality metrics.
- metadata:
  Static run context such as run_info.json, config snapshot, git revision, build type, executable/node name, hostname, username, environment info.

Important rule:
Do NOT start implementation immediately.
First perform an audit and create a complete mapping:
existing log/output -> runtime / profiling / export / metadata / unresolved

Use `unresolved` only when classification is genuinely ambiguous.
Do not migrate blindly.

Phase 1 — Audit and classification
Inspect the current codebase and identify all of the following:
1. all text logger entrypoints, wrappers, macros, sinks, and log files
2. all CSV writers and other structured debug/export writers
3. all timing/profiling/performance writers
4. all places where run metadata can be collected
5. the process entrypoint(s) where one run starts
6. the config-loading path(s) whose content should be snapshotted

Then produce:
A. a log inventory table
B. a classification table with one row per existing output
C. a migration plan tied to actual files and functions in this repository

Use this table format:

| current file/function | current output name/path | purpose | proposed category | target path | migration action | confidence |
|---|---|---|---|---|---|---|

Rules for classification:
- If it is primarily for humans reading execution flow, it is runtime.
- If it is primarily for measuring speed/resource usage, it is profiling.
- If it is primarily for offline debugging/analysis of algorithm internals, it is export.
- If it describes the run itself rather than streaming events, it is metadata.
- When uncertain, mark unresolved and explain why.

Do not ask me to manually classify unless absolutely necessary.
Make the best possible classification from the code.

Phase 2 — Design before code
Based on the real code structure, design a minimal logging architecture migration.
Requirements:
1. Introduce one centralized run-log manager responsible for:
   - computing log root
   - creating timestamped run directory
   - creating runtime/profiling/export/metadata subdirectories
   - creating/updating `latest` symlink if supported
   - exposing canonical paths to the rest of the codebase
2. Minimize changes to existing business logic.
3. Reuse existing logger/config/util infrastructure where possible.
4. Avoid unrelated refactors.
5. Keep backward compatibility of log content as much as possible.
6. Path creation must be idempotent and robust.

Before implementation, show:
- the proposed manager/component name
- where it will be initialized
- which files/functions will be changed
- which existing outputs will move to which new paths

Phase 3 — Implementation
Implement the migration in small, reviewable commits.

Implementation requirements:
- Create one run-scoped directory per execution.
- Create/update `latest` as a symlink on supported platforms; otherwise degrade gracefully.
- Route the main text log to:
  runtime/<project>_main.log
- Route subsystem/module text logs to:
  runtime/<project>_<module>.log
  or the closest naming convention that fits this codebase.
- Move timing/performance/profiling outputs into profiling/.
- Move algorithm/debug/export CSVs and similar artifacts into export/.
- Write metadata/run_info.json containing at least:
  - start timestamp
  - executable or node or process name
  - git commit if available
  - build type if available
  - hostname if available
  - username if available
  - selected config path(s)
  - working directory if easy to obtain
- Save config snapshots into metadata/.
- Preserve existing config toggles for timing/debug/export where they already exist.
- For any unresolved outputs, do not guess silently:
  either keep them temporarily with a TODO note, or classify them with an explicit rationale.

Phase 4 — Verification
After coding:
1. build the project
2. run the smallest available smoke test or runtime entrypoint
3. verify that a new run directory is created correctly
4. verify that runtime/profiling/export/metadata files appear in the expected places
5. verify that existing important logs still exist and were not silently dropped
6. verify `latest` behavior
7. summarize exact changes by file

Expected final report:
1. Audit summary
2. Inventory + classification table
3. File-by-file implementation summary
4. Verification results
5. Remaining unresolved items
6. Follow-up improvements only if truly necessary

Quality bar:
- Do not invent abstractions unless justified by the real code.
- Base every change on actual files/functions in this repo.
- Keep naming/style consistent with the repository.
- Prefer small helpers over duplicated filesystem logic.
- Update README or developer docs to document the new log layout.
- Be explicit about anything that cannot be matched exactly.

Execution policy:
- First audit and classify.
- Then plan.
- Then implement.
- Then verify.
- Do not skip the classification phase.