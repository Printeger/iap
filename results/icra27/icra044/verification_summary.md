# ICRA-044 verification summary

IAP-RQ-423 closes the G0C live-artifact protocol boundary with synthetic
Python evidence only. A non-plan runner invocation now accepts only an absent
or empty ordinary root and rejects every existing child or root symlink before
state persistence and before the GPU/launch seam. Plan-only remains
non-mutating; a preflight-only root is non-reusable.

Runner state `p4_g0c_runner_state_v3` binds every COMPLETE attempt to canonical
`p4_g0c_run_artifact_inventory_v1` bytes and to the exact recorded
`test_planner_manifest_path` plus both SHA-256 values. Each inventory contains
every regular file and symlink-free directory below its run, excluding only
itself. Analyzer recomputes paths, types, sizes and hashes and rejects
missing/add/change/remove/duplicate/escape/symlink drift, nested retry/run
directories, second G0C manifests and second production-schema P4 decision
CSVs.

The launch-manifest binding permits the real dynamic
`exports/<run-token>/test_planner_manifest.json` path only when it is absolute,
normalized, symlink-free and inside that run's `exports/` tree. Production
timing, GNSS/ARAIM/validation and other inventoried files are not rejected by a
global basename blacklist.

In-root analyzer outputs are limited to `p4_g0c_analysis.json` and
`p4_g0c_threshold_draft.json`, are excluded from raw calibration hashing and
use exclusive no-overwrite writes. Arbitrary, swapped, aliased, symlinked or
existing destinations reject before analysis/write; rejected analysis emits
no draft.

Verification:

- dirty-root and artifact/output red suites each exited 1 against ICRA-043;
- focused protocol 6/6, runner 14/14, analyzer 22/22, launch contract 6/6 and
  launch golden 16/16 passed (64/64);
- the post-review repository Python discovery passed 403/403;
- Python syntax, fatal-only flake8 and `git diff --check` passed;
- all 3,829 files across 12 retained ICRA-042 build/install trees preserve
  exact before/after SHA-256
  `6836841bc7ee74594ff80926bfd67c8531ea2d26076b27406cb9aeea3d784d34`;
- protected fixture/PDF hashes remain exact.

No GPU preflight, ROS, launch, calibration, CTest/retained binary, bag/RViz,
smoke, benchmark, threshold draft/freeze/application, G0D or P5 ran. Result is
`P4_G0C_LIVE_ARTIFACT_PROTOCOL_READY_FOR_REVIEW`, not G0C PASS.
