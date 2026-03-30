# Continuous-Time Solver Refactor README

## Goal

Refactor the continuous-time odometry path toward one production route only:

- `frontend_mode = CT_LIDAR_GPU`
- `ct_lidar_gpu_backend = KERNEL`

The target architecture is:

- GLIM-style incremental fixed-lag solver organization
- C-LIUO-style local continuous-time solve domain
- shared target ownership instead of per-segment target ownership
- runtime-first KERNEL execution with diagnostics isolated from the hot path

## Public Route Policy

- `BUCKET` is no longer a public runtime backend.
- `KERNEL` is the only intended public GPU route.
- During the transition period, `BUCKET` may still exist internally for parity
  and one-time A/B work, but normal configs and launches must not use it.

## Phase Order

### P0. Public deprecation and spec

- Keep this file as the source-of-truth execution spec.
- Mark `BUCKET` deprecated in public docs and runtime.
- Default config stays on `KERNEL`.
- Runtime selection of `BUCKET` must fail loudly unless an internal-only escape
  hatch is used for parity work.

### P1. Incremental CT solver skeleton

- Introduce a local continuous-time solve domain abstraction.
- Stop treating the whole historical active window as the default LiDAR solve
  domain.
- Build current scan factors over:
  - current scan domain
  - a small recent overlap region
- Keep the current batch path only as a temporary compatibility shell while the
  fixed-lag solver lifetime is migrated.

### P2. Shared target handle and KERNEL-only runtime path

- Introduce a shared target handle carrying:
  - target identity
  - target revision
  - target mode
  - target-side lookup resources
- Move runtime factor ownership away from per-segment target state.
- Ensure KERNEL target refresh only refreshes target-side resources and never
  rebuilds source-side staging.
- Keep runtime and diagnostic result collection explicitly separate.

Current status:
- `ISharedTargetHandle / SharedTargetHandle` 已落下。
- `OdometryEstimationBSpline` 已开始通过 shared handle cache 复用 target identity/revision。
- `IntegratedBSplineGICPFactorGPUKernel` 已支持直接绑定 shared target GPU resources，并在 target revision 变化时通过 `refresh_target_handle(...)` 只切换 target-side resources。
- 真正长期存活的 incremental fixed-lag solver 仍未完成；当前仍处于 batch-compatible 过渡壳。

### P3. Shared-state and carried-prior stabilization

- Make `ecef_origin / ecef_rot / clock` ownership explicit.
- Keep shared GNSS state valid across carried-prior replay and lag retirement.
- Long-bag acceptance requires:
  - no missing-key carried-prior warnings
  - no GNSS factor collapse caused by replay/ownership bugs

### P4. Acceptance and BUCKET deletion

- Run internal parity checks against the frozen BUCKET implementation.
- After KERNEL passes parity and long-bag validation:
  - remove BUCKET from public config and code paths
  - remove BUCKET-specific tests and docs
  - keep only CPU CT fallback and KERNEL GPU production paths

## Implementation Rules

- Do not change the public `EstimationFrame` compatibility contract.
- Do not expand planner scope during this refactor.
- Preserve the continuous-time state model; change solver organization, target
  ownership, and runtime lifecycle.
- Diagnostics must be opt-in:
  - runtime mode only returns the current factor outputs required by odometry
  - diagnostic mode enables whole-window result aggregation, CSV, numeric audit,
    and degeneracy reporting

## Immediate Execution Targets

1. Add `ICTSolveDomain` / `BSplineSolveDomain`.
2. Add `ISharedTargetHandle` / `SharedTargetHandle`.
3. Make `BUCKET` internal-only at runtime.
4. Move the LiDAR solve loop to current-domain plus recent-overlap selection.
5. Keep KERNEL on the existing unified result/profile/baseline surface.
