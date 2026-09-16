# IAP Conventions (强约定)

## 1) GNSS tight coupling conventions
- Measurement residual definition:
  - Pseudorange: r_rho = meas - pred
  - Doppler:     r_D   = meas - pred
- Doppler unit: m/s (not Hz)
- Satellite position/velocity source: broadcast ephemeris
- Receiver clock:
  - clock bias δt in seconds (or meters via c*δt, but must be consistent)
  - clock drift δt_dot in s/s (or m/s via c*δt_dot)

## 2) LiDAR backend conventions
- Use GLIM ICP pipeline as baseline for LiDAR constraints.
- Health monitoring:
  - Detect degeneracy / mismatch per scan factor
  - Convert to noise inflation gamma_lidar (>=1) and/or drop factor

## 3) Visibility/observability prediction conventions
- GNSS visibility set V^:
  - Must use point-cloud/map-based occlusion model (ray-based LOS check)
  - Elevation mask allowed (optional), but cannot be the only model
  - In registered online-LiDAR mode, a frozen `TrustedLocalMapSupport`
    envelope may make hit-only point clouds model-complete: inside its range,
    FOV, frame and validity interval, absence of a hit means “no obstacle was
    detected by this model”. Outside or after expiry remains `UNKNOWN`.
  - Model-complete support is not `OBSERVED_FREE`; ray-observed free space and
    model assumptions must remain separately identifiable.
  - P0, P4 and final trajectory checks share this admission rule. When
    `require_observed_support` is enabled, a voxel with no ray-derived
    observation may still be evaluated if the frozen trusted envelope reports
    `MODEL_COMPLETE`; occupied/inflated cells still follow the collision
    policy, and outside-envelope, incomplete, expired or frame-invalid support
    remains fail-closed. P0 cache identity includes the envelope identity.
  - Support identity includes pose, extent, timestamp/expiry and model
    version, and participates in P0/P4 snapshot and cache identity.
  - Envelope validity is evaluated once per immutable planning/evaluation
    snapshot using `evaluation_time_s` and the map acquisition stamp, on the
    same ROS/simulation clock. A candidate's future arrival `query_time_s`
    must not make a currently fresh map expire. `query_time_s` remains
    authoritative for trajectory arrival, temporal-layer selection and
    uncertainty growth. Publication and execution perform a new current-time
    freshness check; an earlier pass is never cached as permanent authority.
    A newer map generation does not invalidate computation already bound to a
    fresh immutable snapshot; the existing collision-delta commit remains the
    geometric change check.
    `MODEL_COMPLETE` describes a solved advisory over its explicit local
    satellite set `S_i`; excluded UNKNOWN satellites remain counted in
    diagnostics.
  - Environmental LiDAR hits required by the sky-risk kernel may be retained
    outside the vehicle flight/geofence lattice. The geofence still limits
    executable trajectories.
- Planner-map coordinates are the common risk-computation frame. Receiver,
  candidate, local-map sensor origin and GNSS LOS must be expressed in that
  frame and bound to one explicit `frame_contract_id`. GNSS azimuth is
  clockwise from ENU/map north (`azimuth=0 -> +Y`); if a future map contract
  rotates map from ENU, LOS vectors must be rotated by that same contract.
  Receiver and candidate raw advisory PL use the same frozen GNSS epoch,
  certified exclusion set, local-map snapshot and local satellite set. At the
  receiver position their spatial advisory increment is zero; crossing the
  measured-support radius must not introduce a discontinuity. The epoch's
  finite positive pseudorange sigma remains a common lower bound on the
  canopy-derived sigma on both sides of that radius; the radius changes only
  support admission.
- `RiskGrid` PL is a coarse search/cost field, not final execution authority.
  Trilinear/temporal PL interpolation is valid only when every positive-weight
  corner has normal GNSS geometry and the same local satellite set, trusted
  support authority/status and active prediction-source combination. A
  topology change returns `DIRECT_RECHECK_REQUIRED`; insufficient, singular
  leave-one-out or numerically failed geometry returns an explicit geometry
  status with non-finite PL. Such values are never replaced by a large finite
  sentinel and never participate in ordinary interpolation.
- Final publication and runtime execution checks sample the actual immutable
  B-spline (including its endpoint, at no more than 0.2 s spacing) and use one
  direct ForwardRisk batch on a coherent immutable `P0ExecutionRiskSnapshot`.
  This lightweight authority binds the frozen occupancy/support, GNSS epoch,
  certified current Integrity, LiDAR/FIM inputs and all frame/geometry/policy
  identities before the corresponding dense RiskGrid is built. The true map
  frame contract is carried separately from the lattice geometry identity.
  Repeated identical tuples retain the same execution snapshot ID. A dense grid
  that misses its 500 ms end-to-end budget is discarded without partial
  publication and cannot invalidate a still-fresh execution snapshot. The
  execution channel is an occupancy-commit-generation-driven single-slot
  latest-wins worker; a 50 ms timer only detects missed notifications and
  never performs a second eager map freeze. Map commit observers run after the
  occupancy writer lock is released and use lifetime-safe weak ownership. If a
  newer generation arrives while an older request is building, the older
  result is explicitly superseded before the atomic publication boundary.
  Replaceable source factories are synchronized and worker exceptions are
  reported as classified failures without terminating the worker. Every
  attempt emits a classified
  request/capture/build/publish record on
  `/planning/execution_snapshot_attempt`. Dense-grid workers cooperatively
  relinquish their scheduler timeslice at predictor batch boundaries while an
  execution request is pending, and reuse the
  worker's frozen occupancy whenever it is the current generation;
  a later-finishing RiskGrid transaction cannot overwrite its authority.
  RiskGrid health distinguishes the active immutable generation from the last
  build attempt: generation/source races and budget cancellation remain in
  attempt evidence, while a retained active generation remains usable only
  until its ordinary freshness deadline. The result identity binds trajectory
  timing, control points/knots, sampling
  lattice, execution-snapshot/occupancy identities, diagnostic RiskGrid
  generation and GNSS epoch. `SAFE` with complete
  GNSS/LiDAR/FIM support is required; unsafe, incomplete, degenerate, expired
  or over-budget results fail closed. Same-generation result reuse never
  bypasses current map/GNSS/certified-Integrity freshness, and any generation
  change forces recomputation. RiskGrid remains available as a planning
  heuristic and for diagnostics. Its absence, expiry or build timeout disables
  only grid-derived ordering and P1 cost preference: enumeration continues
  from frozen occupancy and authorization still comes from the execution
  snapshot direct batch. The legacy
  `require_risk_grid_ready_before_planning` setting is diagnostic-only.
- Execution publication retains a bounded four-result history, separate from
  the single pending work slot, solely to select the newest snapshot causal to
  an execution evaluation timestamp. Each selected result still undergoes the
  ordinary current-time freshness checks; history is not a persistent safety
  ticket. Registered sparse execution snapshots preserve the transaction's
  current-vehicle footprint as true `OBSERVED_FREE` using the captured pose
  and vehicle radius, without copying the dense observation grid or treating
  the surrounding model envelope as measured free space.
- Certified current-Integrity lookup is causal: concurrent callbacks may retain
  a bounded history, but an execution check selects the newest sample at or
  before its evaluation time. One microsecond is the maximum timestamp
  comparison tolerance for ROS floating-point conversion; it does not extend
  the configured freshness window. Unsafe or invalid newer causal samples must
  not be hidden by older valid samples.
- P5 consumes that same execution-snapshot-bound direct batch. It does not
  require a matching or newly published RiskGrid; acquiring a newer grid
  generation between P4 and P5 does not by itself invalidate a completed
  coherent result. Ordinary RiskGrid PL is never a P5 authority. The existing
  P5-4/P5-7 fixture overlay remains an explicit test policy, not a production
  interpolation fallback.
- Planner-side GNSS geometry uses all four constellations
  `GPS+BDS+GAL+GLO` in formal and forest launch defaults. This changes neither
  the PL equation nor AL. The full information matrix is factored once;
  leave-one-out protection levels use an algebraically equivalent rank-one
  downdate, with the original LDLT solve at the numerical boundary. Exact
  satellite-ID/LOS/sigma bit patterns may use a bounded 4096-entry cache;
  approximated or quantized cache keys are forbidden.
- A newly appearing advisory satellite must be present in three consecutive
  raw epochs before use. Disappearance, certified exclusion and hard LOS
  obstruction remove it immediately. This admission state is explicitly
  diagnostic and is not added to the certified epoch identity. Final and
  runtime checks use the intersection of usable satellites over the complete
  remaining short-trajectory batch and recompute candidate and receiver raw
  PL from that common core; an insufficient intersection is `UNKNOWN/HOLD`.
- The optional GNSS LOS clearance transition is part of the predictor and
  snapshot identity. For width `w>0`, an occupancy generation owns a truncated
  distance field to occupied voxel surfaces; the LOS proximity is
  `1-smoothstep(0,w,clearance)` and combines with the original discrete canopy
  coverage as `1-(1-kappa_discrete)(1-kappa_clearance)`, exactly once. The
  union is evaluated per LOS sample: proximity is exactly one for an occupied
  sample, so the result reduces to the continuous proximity kernel without
  retaining or double-counting the binary boundary jump. `sigma_eff` remains
  `max(epoch_sigma, canopy_sigma)`. Envelope/support/elevation gates are not
  smoothed and true ray intersections still obey hard-occlusion exclusion.
  Width `0` preserves legacy behavior; dense forest v2 uses `0.4 m`.
  Implementations cap width at `5 m` and `64` voxels and cap dense storage at
  `16M` float cells (64 MiB) and each axis at `2048` cells, bounding transform
  scratch space as well. The exact separable EDT is linear in those cells
  and independent of occupied-voxel count and transition radius. Pathological
  wider coordinate extents fail closed with maximum proximity in constant
  query time rather than using an unbounded sparse or radius-cubed fallback.
- LiDAR observability set O^:
  - Baseline: use map-based proxy for “ICP observability/quality” (may include occlusion)
  - Trunk landmarks: optional upgrade (not required for baseline)

## 4) Covariance/Integrity propagation conventions
- Baseline propagation: empirical covariance growth model (Σ growth)
- Keep an interface for exact propagation/update:
  - S = H Σ H^T + R
  - (optional) Σ update with Kalman-style measurement information
- Planning uses PL_pred derived from Σ_pred (proxy) in baseline

## 5) Receding-horizon execution conventions
- A published P4 B-spline is an immutable, finite execution grant. Its
  certificate binds trajectory id, start time, duration/knots/control points,
  approved endpoint/deadline, terminal velocity/acceleration, braking model,
  map/risk snapshot identity and selection authority.
- Every P4 curve, including common-prefix and observe-more curves, must satisfy
  fixed zero terminal velocity and acceleration before final acceptance. The
  production terminal fit treats start position/velocity/acceleration,
  approved endpoint and terminal velocity/acceleration as hard equalities.
  If it retimes a dynamically feasible curve, collision and risk are checked
  again at the final curve's actual arrival times before publication.
- If every complete topology route fails the risk gate, P4 may authorize only
  the continuous risk-safe portion of their geometry-common corridor. It
  stops at the first unsafe or unsupported sample, reserves tracking/braking
  margin, requires valid current Integrity below AL and enough distance to
  stop, then emits `DEFER_RISK_SELECTION/COMMON_PREFIX` with
  `LIMITED_PREFIX` authority. It is never labelled `RISK_SELECTED`; absent
  sufficient safe distance, the action remains HOLD. Executable progress is
  the continuous safe common-corridor distance minus current-speed stopping
  distance and vehicle/tracking reserve, capped by
  `max_limited_prefix_progress_m` (default/formal forest: 8 m). The legacy
  `max_creep_progress_m` is only a compatibility override. If the remaining
  distance cannot stop the current state, the action remains HOLD. The cropped curve is
  independently checked at its actual arrival times and may not pass the
  approved endpoint.
- A geometry-only prefix, a single route with incomplete risk support, or a
  prefix containing any invalid/stale/unsupported risk sample has no motion
  authority. The current certified monitor sample permits the prefix check to
  start, but cannot substitute for complete risk support along the executable
  segment.
- `PENDING` and `RATE_LIMITED` are typed worker results. If the committed
  certificate still passes identity, tracking, collision and runtime Integrity
  checks, the FSM continues it without retrying initialization or changing its
  start time, endpoint or deadline.
- A valid committed `LIMITED_PREFIX` is replaced only after at least 1.0 s of
  execution when the new common-corridor endpoint advances by at least 0.5 m
  and its direct-risk maximum is no worse than the old remaining curve. An
  invalid certificate or an already reached endpoint may be replaced
  immediately. Ordinary generations, pending/rate limiting and smaller
  numerical endpoint changes retain the exact trajectory id, start, endpoint
  and deadline.
- A replacement is a parent-bound prepared successor. Before atomic publish it
  rechecks the parent trajectory ID/start/control-point hash, switch window,
  bounded position/velocity/acceleration boundary mismatch, execution-snapshot
  freshness, direct risk and collision state. A late, discontinuous or
  identity-mismatched successor is discarded while the parent continues;
  accepted successors record their parent and switch without an intermediate
  endpoint hold.
- Native refinement reports structured status. A densely sampled coarse path
  that is collision-free in frozen occupancy is accepted directly; A* runs
  only for colliding segments. Budget, occupancy, collision, no-path, invalid
  result, corridor escape and short-output failures remain distinct, and every
  successful refinement is directly re-certified.
- RiskGrid providers accept a position-major spatial batch plus a horizon
  list. Generation health reports occupancy/support scan, query layout,
  provider, voxel materialization, commit and total build timing. Source and
  budget validity are checked at each stage boundary; an over-budget
  generation is discarded whole and never partially published. Rich
  provider/topology voxels are stored only for spatial cells admitted by
  occupancy/support; skipped-cell occupancy and support are stored once per
  spatial layer and reconstructed with the exact horizon stamp on query. This
  storage optimization may not change public interpolation, trace or UNKNOWN
  semantics.
- A committed limited prefix stores independently parameterized stopping
  curves at no more than 0.2 s anchor spacing. Every curve is start-state
  continuous, terminal-zero, dynamics/collision checked, and all curve samples
  are checked in one direct-risk batch. If source data expires, the executor
  schedules the nearest future anchor, continues the old approved curve for no
  more than 0.2 s, then publishes that curve with a new trajectory and braking
  certificate identity under `LIMITED_PREFIX_BRAKING`, without extending the
  original endpoint or deadline. Tracking loss, imminent collision, current certified Integrity
  violation, an unsafe direct batch or an unavailable suffix remains an
  emergency fail-closed condition. This terminal state is reported separately
  from normal arrival at the originally approved endpoint.
- Scheduling a stopping curve does not mutate the committed trajectory or its
  `LIMITED_PREFIX` certificate. Until the selected anchor (at most 0.2 s), a
  newer fresh execution snapshot triggers a complete direct-risk, corridor
  support-age, certified-Integrity/GNSS, collision and identity recheck of the
  original remaining B-spline. A wholly safe result cancels the schedule as
  `FAILSAFE_BRAKING_CANCELED_RECOVERED` without changing trajectory ID, start,
  endpoint or deadline. Unsafe/unknown/over-budget evidence, collision,
  Integrity failure, tracking loss, or an already activated braking curve
  cannot be canceled.
- Candidate mutation and final lineage/P5/publication form one execution-
  commitment transaction. If a final gate rejects the candidate, both the
  incumbent B-spline and its execution certificate/evidence are restored.
  While `LIMITED_PREFIX_BRAKING` is active, ordinary replanning cannot replace
  or relabel that curve; only endpoint completion or an explicit runtime /
  collision revocation ends its authority.
- Runtime timing evidence follows `LiDAR source -> ROS receive -> occupancy
  freeze -> support ready -> execution snapshot publish -> RiskGrid
  start/end -> execution query`. Freshness failures retain all observable contributors
  and choose the primary cause in this order: `SOURCE_DATA_GAP`,
  `OCCUPANCY_BUILD_LAG`, `SNAPSHOT_QUEUE_LAG`, then
  `RISK_GRID_BUILD_LAG`. Grid lag affects search only and cannot revoke a
  trajectory independently proven safe by a fresh execution snapshot.
- Trusted support keeps at most 64 original current-frame envelopes inside the
  unchanged 1.0 s hard window. Each trajectory sample selects the newest
  genuinely covering envelope and computes observation age from its original
  scan stamp at the current evaluation time. Future arrival time continues to
  drive prediction only. This corridor-local admission neither fabricates
  `OBSERVED_FREE` nor lets an unrelated fresh region cover an expired path.
- A sub-metre limited prefix prefers the continuous nominal public approach
  while it remains inside the geometry-common free corridor. This prevents a
  half-voxel A* centre offset from becoming an unintended initial climb,
  descent or branch commitment. The chosen prefix and its final stopped
  B-spline still require independent geometry, support and direct-risk checks.
- Reaching the approved endpoint is a normal hold state, distinct from safety
  revocation. Runtime collision and GNSS sky-risk kernels are checked
  independently; this remains required when P5 is disabled.
- Development smoke acceptance keeps limited authority distinct from formal
  route selection. `LIMITED_PREFIX_EXECUTED_TO_ENDPOINT` requires the same
  trajectory id/start/control-point certificate across publication and
  commands, non-trivial odometry displacement, no approved-endpoint overrun,
  and a sustained zero-velocity/zero-acceleration command plus stationary
  odometry at that endpoint. `LIMITED_PREFIX_EXECUTED_THEN_RISK_REVOKED`
  instead requires a newer risk generation, a concrete runtime PL/AL
  violation and cessation of the old trajectory. `FORMAL_ROUTE_SELECTED`
  never satisfies the limited-prefix smoke by itself.
- Runtime current-Integrity authority comes from the fresh certified monitor
  sample in the latest coherent P0 transaction and its own HAL/VAL. The raw
  map advisory evaluated at the receiver is diagnostic/predictive and must not
  replace that certified-current decision. Remaining trajectory samples still
  use their actual positions and arrival times for predicted risk. The
  configurable tracking-error limit must be finite, positive, and at most 5 m
  (production default: 0.75 m).
- `p4.debug_generation_probe_enable` is diagnostic-only and defaults false.
  For two adjacent snapshots that are both fresh at one evaluation time, it
  holds approved-trajectory positions and arrival times fixed and evaluates
  old-map/old-epoch, new-map/old-epoch, old-map/new-epoch and
  new-map/new-epoch. Direct ForwardRisk is also compared with each generation's
  RiskGrid interpolation. Probe results may classify map/support, GNSS epoch /
  satellite-set, interpolation or mixed changes, but never grant or revoke
  motion and never bypass map or GNSS freshness. Classification compares the
  typed boundary (index, safety/ranking/failure, satellite-set hash and grid
  interpolation state), not the index alone. Each counterfactual batch has a
  strict diagnostic budget; execution refreshes ROS time and repeats current
  map/GNSS/certified-Integrity freshness checks after the probe.
  When an interpolation anomaly is found, the probe records both temporal
  layers' eight spatial corners, their spatial/temporal/combined weights, PL,
  support and source identities, full satellite IDs/hash, geometry condition,
  worst exclusions and explicit failure state in a rate-limited corner CSV.
