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
- `BSplineIncrementalSolverSkeleton` 已落下，开始显式追踪 local solve-domain 的 active/new/retired segments、new/retired keys，以及未来增量 fixed-lag solver 所需的 `new_values / new_stamps` 生命周期载荷。
- `BSplineFixedLagStateRegistry::seed_clock_values(...)` 已落下；`OdometryEstimationBSpline` 现在会在 `BSplineIncrementalSolverSkeleton::prepare_update(...)` 之前，先把 shared `j / k / g / e / r` states 和 solve-domain 需要的 `c` keys 显式种入 authoritative `Values`。
- B-spline 路径现在会通过 `reset_bspline_incremental_smoother()` 重置一套 CT 专用 smoother shell，并显式注册 `s / u / j / k / g / c / e / r` 的 relinearization policy，未来长期存活 incremental fixed-lag solver 不再只能继承 legacy discrete-time 的 threshold 集。
- per-segment 的 velocity / IMU / GNSS factors 现已开始收口成持久化 cache；`OdometryEstimationBSpline` 不再每帧都为同一 active solve-domain segment 重新 `make_shared` 这些非 LiDAR 因子，而是开始复用 segment 生命周期内稳定的 factor inventory。
- control-point anchor / prediction / smoothness priors 现已开始按 local solve-domain 控制点集合收口；batch-compatible 壳不再默认把整条历史 active-window control span 全部通过先验重新拖进当前 solve。
- 真正 authoritative 的长期存活 incremental fixed-lag solver 仍未完成；当前仍处于 batch-compatible 过渡壳，但 solver lifetime 和 key/segment 生命周期接口已不再只是文档概念。

### P3. Shared-state and carried-prior stabilization

- Make `ecef_origin / ecef_rot / clock` ownership explicit.
- Keep shared GNSS state valid across carried-prior replay and lag retirement.
- Long-bag acceptance requires:
  - no missing-key carried-prior warnings
  - no GNSS factor collapse caused by replay/ownership bugs

Current status:
- shared `ecef_origin / ecef_rot` seeding 与 solve-domain clock seeding 已开始从 batch-compatible 建图后半段前移到增量生命周期入口。
- `BSplineIncrementalSolverSkeleton` 现在看到的 authoritative `Values` 已包含 shared GNSS/ECEF alignment states 和 solve-domain 需要的 segment clock keys，这为后续 carried-prior replay / key retirement 提供了正确的入口语义。
- shared-state 的长期回放验收、authoritative long-lived incremental fixed-lag solver、以及无 `e0` 缺键的长包验证仍未完成。

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
6. Introduce an explicit incremental solver skeleton so add/remove lifecycle and future smoother payloads are no longer embedded ad hoc inside `insert_frame_ct_lidar()`.
7. Move shared `ecef_origin / ecef_rot / clock` seeding to happen before incremental delta preparation, so local solve-domain lifecycle and future carried-prior replay observe the same authoritative key set.
8. Replace the remaining batch-compatible solver shell with an authoritative long-lived incremental fixed-lag solver owner, then finish long-bag shared GNSS state validation before deleting BUCKET.
