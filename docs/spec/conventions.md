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
- Runtime current-Integrity authority comes from the fresh certified monitor
  sample in the latest coherent P0 transaction and its own HAL/VAL. The raw
  map advisory evaluated at the receiver is diagnostic/predictive and must not
  replace that certified-current decision. Remaining trajectory samples still
  use their actual positions and arrival times for predicted risk. The
  configurable tracking-error limit must be finite, positive, and at most 5 m
  (production default: 0.75 m).
