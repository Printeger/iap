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
  - Support identity includes pose, extent, timestamp/expiry and model
    version, and participates in P0/P4 snapshot and cache identity.
  - Envelope validity is evaluated at each candidate's `query_time_s`, not at
    snapshot refresh time. `MODEL_COMPLETE` describes a solved advisory over
    its explicit local satellite set `S_i`; excluded UNKNOWN satellites remain
    counted in diagnostics.
  - Environmental LiDAR hits required by the sky-risk kernel may be retained
    outside the vehicle flight/geofence lattice. The geofence still limits
    executable trajectories.
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
  fixed zero terminal velocity and acceleration before final acceptance.
- `PENDING` and `RATE_LIMITED` are typed worker results. If the committed
  certificate still passes identity, tracking, collision and runtime Integrity
  checks, the FSM continues it without retrying initialization or changing its
  start time, endpoint or deadline.
- Reaching the approved endpoint is a normal hold state, distinct from safety
  revocation. Runtime collision and GNSS sky-risk kernels are checked
  independently; this remains required when P5 is disabled.
- Runtime current-Integrity checks use the actual receiver position. The
  configurable tracking-error limit must be finite, positive, and at most
  5 m (production default: 0.75 m).
