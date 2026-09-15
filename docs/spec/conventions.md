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
  direct ForwardRisk batch on the corresponding coherent P0 snapshot. The
  result identity binds trajectory timing, control points/knots, sampling
  lattice, occupancy/risk generations and GNSS epoch. `SAFE` with complete
  GNSS/LiDAR/FIM support is required; unsafe, incomplete, degenerate, expired
  or over-budget results fail closed. Same-generation result reuse never
  bypasses current map/GNSS/certified-Integrity freshness, and any generation
  change forces recomputation. RiskGrid remains available as a planning
  heuristic and for diagnostics.
- P5 consumes that same direct batch and its exact immutable RiskGrid/P0
  identity; acquiring a newer generation between P4 and P5 does not by itself
  invalidate a completed coherent result. Ordinary RiskGrid PL is never a P5
  authority. The existing P5-4/P5-7 fixture overlay remains an explicit test
  policy, not a production interpolation fallback.
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
  sufficient safe distance, the action remains HOLD. The configured maximum
  prefix progress is a hard upper bound; if current-speed braking needs more,
  the action remains HOLD. The cropped curve is
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
