# ICRA-059 final verification

- Phase A commit/push: `7ec1f94`, exit 0.
- Build attempt 01: colcon argv rejected before configuration because the
  global `--log-base` option followed the `build` verb; exit 2, retained.
- Build attempt 02: exact 17-package sequential merged Release/CUDA command,
  exit 0, 17 packages in 4m42s, retained.
- Pre-identity correction: v4-only nonregistered readiness identity and exact
  launch-time P0 effective mapping; protocol/registry/dependency hashes rolled
  forward canonically.
- Build attempt 03: same exact build selection/settings under a fresh root,
  exit 0, 17 packages in 4m44s. Static closure exit 0: 17 indexes, six ELF
  libraries, exact installed launch hash, no missing/historical linkage.
- Readiness GPU preflight: one invocation, exit 0, `gpu_ready=true`, both
  `nvidia-smi` calls 0, `cuInit(0)=0`, device count 1.
- Readiness launch: one invocation, nonregistered ID
  `p4-g0c-r4-readiness-attempt01`, 20 seconds, process exit 0. Exact P0 values
  appear in top-level and effective manifests; required processes exit cleanly.
- Readiness scientific verdict: FAIL. P0 reaches ready/generation 19, but the
  decision CSV has 15 rows, zero positive snapshot identities and only
  `snapshot_unavailable`. Typed result:
  `BLOCKED_R4_READINESS_NO_P4_POSITIVE_SNAPSHOT`.
- Final hermetic focused suite: 122/122 PASS; canonical JSON and diff checks
  PASS; external inventory 17,759 entries with empty delta.
- Phase C dependency/full runner/r4 identity/analyzer counts: all zero.
- Final process audit: zero matching task/required processes. Protected PDF
  remains untracked and SHA `1f07da56…44f6`; v1-v3 hashes remain exact.

The complete build argv and per-package stdout/stderr are retained by colcon
under `results/icra27/icra059/build_attempt_{02,03}/log/`; the exact readiness
argv and repository-local ROS logs/products remain under
`results/icra27/icra059/readiness_attempt_01/`. These raw trees are ignored and
were not staged or cleaned.
