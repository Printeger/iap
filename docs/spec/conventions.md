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
  generation and GNSS epoch. In `STRICT_GLOBAL`, `SAFE` with complete
  GNSS/LiDAR/FIM support is required. In `MISSION_BEST_EFFORT`, only explicitly
  classified task-global GNSS failures (AL exceedance, missing/inconsistent
  GNSS anchor, too few usable satellites, unknown sky or GNSS geometry
  degeneracy) may become degraded navigation evidence after local motion and
  braking are independently proven. Occupancy/support, LiDAR/FIM, freshness,
  computation-budget and evidence-identity failures always fail closed.
  Same-generation result reuse never
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
- P4 is the sole pre-execution authority for the actual B-spline, its local
  envelope, braking library, GNSS evidence and exposure. Publication only
  validates the resulting `P4ExecutionCertificate` identity, freshness,
  deadline, task/execution mode and remaining exposure; it never replays the
  direct batch. P5 starts only after a full-identity `ACTIVATED` acknowledgement
  and evaluates newly reachable runtime evidence. Ordinary RiskGrid PL is
  never a publication or P5 authority.
- Runtime execution feedback is matched by the complete active execution
  identity. A fresh PositionCommand is the preferred execution clock. If its
  callback is temporarily stale while the same identity's controller trace is
  fresh, the trace supplies elapsed time plus commanded and feedback `p/v/a`;
  callback skew alone is not a controller outage. Old identities and elapsed
  rollback are rejected without refreshing either source. If neither source
  is fresh after the existing activation grace window, or if trace saturation,
  tracking or controllability fails, runtime retains its fail-closed brake.
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
  diagnostic and is not added to the certified epoch identity. Production
  final/runtime checks use `BRAKING_WINDOW_POINTWISE`: each deterministic window
  covers the nominal commands that may execute before the next handover, every
  real certified braking curve reachable from those commands, and a 0.4 s
  transition overlap. Every point uses its own known-visible, non-blocked LOS
  satellite set for candidate and receiver geometry, fault subsets and PL/AL;
  a window never intersects or overwrites those point-local sets. Transition
  evidence evaluates the same space-time samples once for each owning window,
  accepting only when every point is complete and below AL. Exact equal point
  sets may reuse receiver, candidate and geometry calculations, but every
  original logical window ID retains its own certificate and transition
  responsibility. A point with insufficient satellites, missing braking
  anchor, anchor gap over 0.2 s, degenerate geometry or budget expiry makes its
  window `UNKNOWN/HOLD`. The legacy whole-curve `COMMON_CORE` remains an
  explicit diagnostic/A-B policy, not the formal forest default. The former
  braking-window common-core production semantics are replaced. During the
  final 0.2 s after the last discrete guard anchor, the exact remaining
  hard-terminal spline is registered as the reachable suffix braking curve
  and can be activated without a state discontinuity; it may neither extend
  the deadline nor move the approved endpoint. Every admission certificate
  binds the window-layout hash and each request-ordered window sequence hash of
  `(evidence_point_id, local_satellite_set_hash)` pairs.
  P4 successor reauthorization and runtime P5 reject a policy/layout/set mismatch;
  a legacy common-core result cannot be relabelled as a window certificate.
  The window layout is constructed exactly once when the actual B-spline is
  committed. Its absolute trajectory times, evidence-point IDs, original
  window IDs, transition duplicates, braking curves and layout hash are
  immutable for that trajectory lifetime. Runtime projects every still-
  reachable nominal row and every braking row whose anchor has not passed,
  retaining their original window responsibilities. It must not move a
  boundary, create a watchdog-time sample or renumber a window. A new
  occupancy generation or GNSS epoch recomputes LOS, support, sigma, geometry
  and PL on that same physical lattice. A layout-hash change inside one
  trajectory is an internal certificate error, not GNSS evidence.
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
  map/risk snapshot identity, GNSS core policy, window/braking layout identity
  and selection authority.
- Every P4 curve, including `LIMITED_PREFIX` and rolling-successor curves,
  must satisfy fixed zero terminal velocity and acceleration before final acceptance. The
  production terminal fit treats start position/velocity/acceleration,
  approved endpoint and terminal velocity/acceleration as hard equalities.
  If it retimes a dynamically feasible curve, collision and risk are checked
  again at the final curve's actual arrival times before publication.
- If every complete topology route fails the risk gate, P4 may authorize only
  the continuous risk-safe portion of their geometry-common corridor. It
  stops at the first unsafe or unsupported sample, reserves tracking/braking
  margin, requires valid current Integrity below AL and enough distance to
  stop, then emits `DEFER_RISK_SELECTION/LIMITED_PREFIX` with
  `LIMITED_PREFIX` authority. It is never labelled `RISK_SELECTED`; absent
  sufficient safe distance, the action remains HOLD. Executable progress is
  the continuous safe common-corridor distance minus current-speed stopping
  distance and vehicle/tracking reserve, capped by
  `max_limited_prefix_progress_m` (default/formal forest: 8 m). The legacy
  `max_creep_progress_m` is only a compatibility override. If the remaining
  distance cannot stop the current state, the action remains HOLD. The cropped curve is
  independently checked at its actual arrival times and may not pass the
  approved endpoint.
- The common corridor is the nominal path contained by every candidate's safe
  tube. Candidate centerlines need not coincide. Decision schema v18 records
  its actual endpoint, tube boundary, and stopping reserve as
  `limited_prefix_endpoint`, `limited_prefix_boundary`, and
  `limited_prefix_stopping_reserve_m`. Predicted information gain is a
  non-authorizing diagnostic: it may choose an earlier feasible stop but may
  neither relax a gate nor turn a safe stoppable prefix into HOLD. Missing,
  zero, or non-positive gain therefore falls back to the farthest safe prefix.
- A geometry-only prefix or an actual executable segment containing any
  invalid/stale/unsupported local-motion or braking sample has no motion
  authority. The current certified monitor sample permits the prefix check to
  start, but cannot substitute for complete support along the executable
  segment. Whole-guide `route_evidence_complete` remains a ranking diagnostic:
  incomplete evidence on an uncommitted route suffix does not override an
  actual bundle already authorized by P4 `TrajectoryAssurance` under the
  MISSION local-integrity contract.
- `PENDING` and `RATE_LIMITED` are typed worker results. If the committed
  certificate still passes identity, tracking, collision and runtime Integrity
  checks, the FSM continues it without retrying initialization or changing its
  start time, endpoint or deadline.
- A valid committed `LIMITED_PREFIX` arms an independent successor route-guide
  task from an absolute deadline. Preparation is released immediately on commit; the
  latest preparation start is the approved endpoint
  time minus the configured generation WCET, direct authorization budget,
  latest-snapshot reauthorization budget, command-switch margin and scheduler
  guard (defaults total 1.5 s). It is a deadline, never a reason to wait.
  A completed route guide immediately enters a prepare-only planner pass. That
  pass freezes the immutable endpoint-minus-command-switch-margin anchor,
  binds the optimized curve start to the parent's position, velocity and
  acceleration at that absolute anchor after the asynchronous route result is
  delivered, and takes the selected guide endpoint as the curve terminal,
  builds the final terminal-stop B-spline, braking library and risk-window
  layout, and completes dynamics, collision, local-clearance and direct-GNSS
  certification while the parent keeps executing.
  The lane is single-flight/latest-wins and bypasses ordinary P4 rate limiting.
  It consumes one already-generated frozen guide. Any geometry, clearance,
  corridor, GNSS, support, freshness or budget failure terminates that
  child attempt; it does not launch topology/A*, switch channels or regenerate
  the immutable child. The parent retains authority and stops on its certified
  curve.
  While it runs, ordinary periodic planning cannot reset the parent. The
  prepare-only pass has no publication authority and may not mutate runtime
  P5 state, exposure or the parent's runtime certificate. Its complete child
  bundle is cached by exact control-point, knot, braking, window, snapshot and
  policy identities. `PREPARED_CERTIFIED` is emitted only after P4 actual-curve
  certification and atomic bundle cache succeed; a refined curve alone is not
  a certificate. Publication validates the cached P4 certificate once.
  Replacement still requires at least 1.0 s of execution. Its minimum endpoint
  advance is dynamic: the motion needed to cover the next switch + generation
  + authorization cycle plus 0.05 m stability margin, with a 0.10 m jitter
  floor. Risk dominance compares only the shared corridor; the new extension
  must independently pass the full actual-curve and braking checks. An
  invalid certificate or an already reached endpoint may be replaced
  immediately. Ordinary generations, pending/rate limiting and smaller
  numerical endpoint changes retain the exact trajectory id, start, endpoint
  and deadline.
- A replacement is a parent-bound prepared successor. Before atomic publish it
  rechecks the live parent trajectory ID/start/control-point hash, child
  trajectory identity, switch window,
  bounded position/velocity/acceleration boundary mismatch, execution-snapshot
  freshness, direct risk and collision state. A late, discontinuous or
  identity-mismatched successor is discarded while the parent continues;
  accepted successors record their parent and switch without an intermediate
  endpoint hold. Smoke accounting ends the parent's observation window at the
  authenticated child authorization stamp and reports
  `LIMITED_PREFIX_ROLLED_TO_SUCCESSOR`; child motion is never charged as a
  parent endpoint overrun.
- At the fixed switch anchor, a cached successor never reruns topology search,
  A*, B-spline optimization or braking-window construction. It loads the exact
  prepared curve, obtains one latest execution snapshot and performs only
  freshness, Integrity, incremental collision, local-clearance and direct-GNSS
  P4 reauthorization. A successful recheck atomically rebinds the
  certificate; a failure leaves the parent endpoint and deadline unchanged.
  It may not activate before the absolute anchor. A callback no more than
  0.2 s late compares `parent(anchor)` with `child(0)`, never parent/child at
  callback time. This handoff consumes execution-snapshot authority and
  bypasses the P1/RiskGrid planning-context publish gate; RiskGrid remains a
  search hint rather than a second execution veto.
  This exact-curve reauthorization also runs when the snapshot ID is unchanged,
  because evaluation time and the already-consumed global-exposure episode can
  advance without changing the sensor tuple. Runtime P5 begins only after the
  child is activated.
- A successor is computed against one immutable execution snapshot. At the
  serialization boundary, a newer snapshot ID by itself is not a rejection:
  the manager samples the exact child B-spline once, rechecks current
  Integrity/GNSS, corridor support, incremental collision and direct
  ForwardRisk, then atomically rebinds a SAFE result. A second snapshot arrival
  is left to the runtime watchdog. Rejections name the changed semantic source
  (`direct_risk`, `support`, `Integrity/GNSS`, collision, incomplete query or
  curve identity), not a generic authority-ID mismatch.
- Successor failures are typed as GNSS limit/exposure, support or individual
  input staleness, local clearance, braking, direct-query timeout,
  latest-snapshot semantic change, collision, dynamics, insufficient progress,
  compute budget, missed deadline or invalid corridor. A missed deadline never
  extends the parent curve; it reaches its approved stopped endpoint.
- P4 route search emits `CANDIDATE_READY`, never formal authority. The manager
  builds the real terminal-stop B-spline and its <=0.2 s braking-anchor
  library, constructs reaction/braking commitment windows, and directly
  checks every nominal, braking and dual-transition sample at its actual
  arrival time. Local motion and braking must be entirely safe. Exact GNSS
  evidence grants `NORMAL_EXECUTION`; in `MISSION_BEST_EFFORT`, a bounded
  actual may instead receive `MISSION_DEGRADED_EXECUTION` when fresh online
  GNSS-independent local-navigation integrity leaves positive adjusted
  clearance on every nominal and braking sample. GNSS exposure remains
  truthful ranking/replan evidence but is not by itself a motion veto. Only
  that complete actual bundle is promoted
  atomically to `RISK_SELECTED`. The first failure retains curve position, arc length,
  arrival time, PL/AL, satellite IDs, sigma/geometry and spatial/temporal
  growth. A braking-branch failure is projected to its anchor station on the
  nominal B-spline; that projection is diagnostic and must not sum braking
  rows or duplicated handover memberships as route progress. Certification
  never feeds a failed actual curve back into generation: it does not crop,
  shift, retime, switch channels or change execution intent. Multi-channel
  preparation continues with the next already-frozen guide and records the
  typed failure; an independently generated common corridor may become an
  ordinary `LIMITED_PREFIX` only after the complete certification chain.
- Best-effort route discovery first schedules refined, clearance-aware channel
  guides, then converts every locally feasible stable channel into an actual
  stopping B-spline and side-effect-free prepared bundle. Each bundle includes
  its real braking library, fixed execution window, local/dynamic/tracking
  checks and direct GNSS/exposure assurance. Before ranking, cached curves
  are re-certified against the same latest immutable snapshot. The comparison
  removes any candidate that fails local collision, clearance,
  dynamics, tracking, support freshness or braking proof. Refined channel
  preferences determine preparation order only. Bundle authorization group is
  derived solely from the actual trajectory's `TrajectoryExecutionMode`.
  Formal bundles precede degraded bundles; degraded bundles are ordered by
  conservative peak ratio, continuous exceedance, positive exposure integral,
  actual/braking-tube unknown exposure, minimum local-navigation-adjusted
  margin, raw clearance margin, actual progress and stable bundle hash.
  Overlapping GNSS intervals do not cause HOLD once both locally safe
  bundles are complete. The old stable channel is only a tie-break after the
  complete ordering key is equal. Winner/runner-up, actual endpoints,
  unevaluated suffixes and the full decomposition remain recorded. If the
  bounded preparation deadline expires, the state is `PARTIAL_COMPARISON` and
  only already certified finite progress is eligible. RiskGrid is a search
  hint and cannot authorize or veto any final curve.
- A queued braking guard has explicit `REQUESTED`, `QUEUED`, `ACTIVATED` and
  `ABSENT` controller states. Time reaching the switch stamp is not proof of
  execution: only a matching traj_server `ACTIVATED` acknowledgement permits
  the atomic trajectory-ID/certificate handover. Missing acknowledgement or
  `ABSENT` at the switch deadline fails closed without relabelling the parent
  trajectory as a braking trajectory. Once a non-recoverable guard is issued,
  its anchor, deadline, curve identity and server acknowledgement state are
  immutable; repeated risk/staleness observations may not slide the stop
  forward or reuse its trajectory ID for another curve.
- Native refinement reports structured status. A blocked requested suffix is
  first scanned back toward the start and replaced by the farthest point that
  satisfies frozen occupancy, the final clearance envelope, the current
  speed's reaction/braking progress, and at least 0.10 m meaningful progress.
  This suffix progress test contains only kinematic reaction and braking
  distance; vehicle radius and fixed safety margin are already represented by
  the shared clearance evaluator and must not be added a second time.
  If none exists it reports `TARGET_SUFFIX_BLOCKED` and
  never asks A* to push the endpoint out of the search pool. The A* pool keeps
  an outer sentinel plus protection cells; adjusted endpoints must remain in
  its searchable interior. Failures record original/adjusted endpoints,
  voxel indices, searchable world bounds, corridor bounds, nearest reachable
  frontier, boundary contact and separate raw/inflated/clearance reject counts.
  The first new failure classification/content identity also embeds a bounded
  frozen local replay crop (origin, dimensions, resolution and per-cell
  raw/inflated/clearance/corridor flags). A rejection family is named
  `*_CLOSED` only when it is the sole observed physical blocker; mixed evidence
  remains `NO_PATH_UNCLASSIFIED` rather than claiming a false root cause.
  The crop covers the A* searchable interior and has a CPU replay adapter that
  reconstructs occupancy and clearance queries without consulting the live
  map. Crop capture cooperates with the refinement deadline; a timeout returns
  its safety decision instead of spending the remaining budget on diagnostics.
  A densely sampled coarse path
  that is collision-free in frozen occupancy is accepted directly; A* runs
  only for colliding segments. Target suffix, pool bounds, raw occupancy,
  clearance envelope, corridor boundary, budget and unclassified no-path
  failures remain distinct, and every successful refinement is directly
  re-certified.
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
  continuous, terminal-zero, dynamics/collision checked, selects the first
  feasible earlier stop on the approved corridor without extending the old
  endpoint/deadline, and all curve samples are checked in one direct-risk
  batch. If source data expires, the executor
  schedules the nearest future anchor, continues the old approved curve for no
  more than 0.2 s, then publishes that curve with a new trajectory and braking
  certificate identity under `LIMITED_PREFIX_BRAKING`, without extending the
  original endpoint or deadline. A directly proven future UNSAFE result also
  schedules this transition as non-recoverable: a later SAFE generation may
  not cancel it or silently resume the parent curve. Tracking loss, imminent
  collision, current certified Integrity violation or an unavailable braking
  curve remains an emergency fail-closed condition. This terminal state is
  reported separately from normal arrival at the originally approved endpoint.
- A formal route carries the same braking-window library. Runtime revalidation
  checks only the current commitment window, its next handover overlap and the
  braking curves that can still be selected, using the latest immutable
  execution snapshot. A later-window failure cannot be skipped: authority may
  extend only through the last continuously safe certified stop. A direct
  formal-window failure therefore activates its existing certified brake
  immediately; the LIMITED_PREFIX marginal confirmation state machine is not
  widened to formal routes. RiskGrid never authorizes a window.
- Every braking-window runtime decision first materializes an immutable
  evidence record and only then changes execution state. The record binds the
  actual curve and fixed layout to execution-snapshot, occupancy/support,
  GNSS-epoch, Integrity and policy identities, and retains active windows,
  worst nominal/per-window samples, PL decomposition, exact satellite sets,
  per-satellite LOS/support/kappa/sigma/exclusion state, geometry condition and
  timing. SAFE, real UNSAFE, incomplete/UNKNOWN and support/provider failures
  all follow this order; an execution event references the evidence sequence.
  The commit-time P5 certificate is stored separately from later runtime
  evidence and cannot be overwritten by a watchdog recheck.
- The optional fixed-layout generation probe is diagnostic-only and runs on a
  bounded latest-wins background channel. At one evaluation time it evaluates
  old-map/old-epoch, new-map/old-epoch, old-map/new-epoch and
  new-map/new-epoch on the exact committed rows, with a separate previous-time
  replay for temporal growth. It reports `MAP_CONTENT_OR_SUPPORT`,
  `GNSS_EPOCH_OR_SET`, `TIME_GROWTH`, `INTERACTION/MIXED`, `STABLE` or
  `NOT_COMPARABLE_STALE_PREVIOUS`; it never authorizes motion or delays a
  revoke/brake.
- A complete LIMITED_PREFIX with a certified braking library distinguishes
  `SAFE`, `MARGINAL_UNSAFE_ARMED`, `CONFIRMED_UNSAFE_BRAKING` and
  `HARD_UNSAFE_BRAKING`. Only a future direct-risk ratio
  `1 < max(HPL/HAL,VPL/VAL) <= 1.005` may arm, and only when a freshly direct-
  certified guard brake stops before the first unsafe boundary. Three distinct
  semantic evidence tuples or 0.35 s without recovery confirms braking;
  the reserved marginal guard deadline must leave the full 0.35 s window.
  Confirmation replaces that reservation with a certified anchor no more than
  0.2 s ahead; without the full reserve the observation is HARD immediately.
  entering `MARGINAL_UNSAFE_ARMED` immediately schedules the certified guard
  anchor as a future-dated command in the trajectory server. Before each
  prequeue the exact guard curve consumes the latest collision delta. Only
  updated complete `SAFE` evidence may request cancellation of the same queued
  trajectory ID before activation; planner state is cleared only after a
  `CANCELED:<trajectory_id>` server acknowledgement. `ACTIVATED` wins a racing
  cancellation and is irreversible. Guard command and acknowledgement topics
  are isolated per drone. The trajectory server enforces the absolute deadline
  even when the planner watchdog skips over the anchor;
  repeated watchdog reads and snapshot-ID-only changes do not count. A newer
  complete SAFE tuple disarms without changing trajectory identity. Armed
  motion cannot pass the reserved guard anchor. Larger/current violations,
  insufficient stopping margin, UNKNOWN/over-budget data, stale inputs,
  collision, hard occlusion, tracking loss or current Integrity failure are
  HARD and schedule braking immediately. Once a brake is scheduled for a HARD
  condition or actually activated, it cannot be canceled.
- Candidate mutation, P4 certification and publication form one execution-
  commitment transaction. If certificate validation rejects the candidate, both the
  incumbent B-spline and its execution certificate/evidence are restored.
  While `LIMITED_PREFIX_BRAKING` is active, ordinary replanning cannot replace
  or relabel that curve; only endpoint completion or an explicit runtime /
  collision revocation ends its authority.
- `LIMITED_PREFIX` geometry for an unresolved fork must come from a corridor
  shared by at least two distinct topology channels. Collision-free cross-links
  or pairwise tube overlap alone are not enough: every executable nominal
  sample must lie in every candidate's vehicle/tracking/topology tube. It must
  retain the normal stopping/tracking reserve and pass the same immutable
  actual-curve, braking-library and P4-certificate publication seam as a
  final channel or rolling successor. Certification failure never crops,
  shifts, retimes or reinterprets that curve. Local-clearance recovery is only
  a generation diagnostic for an ordinary `LIMITED_PREFIX`; it receives no
  certification exception.
- Fixed-point and generation replay are diagnostic only. If enabled on an
  execution check, once any diagnostic predictor query is attempted the
  checker refreshes ROS time after the diagnostic work (regardless of result
  completeness or CSV write success)
  and repeats freshness, corridor support, current Integrity, GNSS, collision
  and direct-risk gates exactly once before granting continued motion.
- Runtime timing evidence follows `LiDAR source -> ROS receive -> occupancy
  freeze -> support ready -> execution snapshot publish -> RiskGrid
  start/end -> execution query`. Freshness failures retain all observable contributors
  and choose the primary cause in this order: `SOURCE_DATA_GAP`,
  `OCCUPANCY_BUILD_LAG`, `SNAPSHOT_QUEUE_LAG`, then
  `RISK_GRID_BUILD_LAG`. Grid lag affects search only and cannot revoke a
  trajectory independently proven safe by a fresh execution snapshot.
- Cross-generation fixed-layout replay is driven by execution-snapshot
  identity, not RiskGrid publication. Its prior production time must belong to
  the same saved successful execution snapshot. Classification compares the
  complete fixed-row PL decomposition, support, satellite sigma/kappa and
  geometry identity; a stable unsafe boundary alone is not `STABLE`. Detailed
  cells are emitted at the same union of physical focus rows.
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
- Execution authorization separates task-global navigation quality from local
  obstacle-relative motion safety. Direct GNSS ForwardRisk samples of the
  exact nominal B-spline define global exposure; fused PL and LiDAR FIM do not
  replace this channel. The default policy permits
  `CONTROLLED_DEGRADED_EXECUTION` only when peak ratio is at most `1.05`, one
  continuous exceedance is at most `2.3 s`, the positive exceedance integral
  is at most `0.115 ratio*s`, and either recovery is predicted within `2.0 s`
  or a currently certified braking curve remains available. The public task
  contract is `p4.assurance.task_mode`: `STRICT_GLOBAL` retains strict
  `GNSS PL < AL`, while `MISSION_BEST_EFFORT` permits degraded motion only
  under the fresh GNSS-independent local-navigation contract below.
  Whitelisted incomplete GNSS evidence never receives a fabricated finite PL;
  it charges the entire
  bounded segment at `maximum_ratio`, with duplicate semantic evidence
  identities charged once. Budget exhaustion remains truthful GNSS ranking,
  replan and diagnostic evidence in MISSION, but does not veto a bundle whose
  online local-navigation margins remain positive; it is never reported as
  global integrity satisfaction. An exposure episode survives trajectory ID changes
  and ends only after `0.5 s` continuously below `0.95*AL`.
- Every immediate actual trajectory is bounded before B-spline resampling.
  Route generation, channel comparison and topology freezing consume the full
  decision horizon; only the selected actual execution seed is cropped.
  Its frontier is the minimum of guide length, decision horizon and consecutive
  fresh local support; its target is the farthest point before that frontier
  that still reserves the complete current `p/v/a` stopping distance. The
  successor switch cadence (normally at most `2.5 s`) never caps that endpoint,
  and required successor progress is only an acceptance floor, not a target
  distance. Exposure never becomes a distance proxy.
  For a degraded MISSION candidate, the generator computes remaining exposure
  in seconds, subtracts the parent interval from the latest charged observation
  to the fixed switch, generates the child from the true switch `p/v/a`,
  applies the production terminal-stop time law, and uses a fixed 16-step
  monotone endpoint search until the real child duration fits that same time
  budget. The lower endpoint bound must still extend the certified parent by
  the required progress and retain the complete stop. Every regenerated curve
  receives new control points, knots and identity before collision, clearance,
  dynamics, braking and P4 certification. The full guide remains only a
  channel/successor reference and cannot obtain execution authority.
- The MISSION exposure values are an explicit risk-acceptance policy, not PL
  fabrication or permission to treat unknown as free. The zero-speed
  production fixture for `min_creep_progress_m=0.25` requires
  `T_min=2.280084270 s` and `0.114004213 ratio*s` at ratio `1.05`; defaults
  `2.3 s` and `0.115 ratio*s` add only numerical tolerance. Startup rejects an
  incompatible MISSION configuration with the typed detail
  `mission_exposure_policy_incompatible_with_minimum_terminal_stop`.
  `STRICT_GLOBAL` is unchanged.
- The `icra_dense_forest_four_fork_v2` task profile has a longer, internally
  consistent MISSION horizon while retaining `maximum_ratio=1.05`. Two
  `2.5 s` bounded parent executions, the `2.280084270 s` minimum terminal
  stop, a `0.2 s` switch margin, and a `0.2 s` scheduler guard total
  `7.680084270 s`; the task rounds this upward to a continuous limit of
  `8.0 s` and therefore sets the matching worst-case integral limit to
  `(1.05 - 1.0) * 8.0 = 0.4 ratio*s`. This is a task-specific profile, not a
  change to the general defaults. Actual exposure, parent-to-switch exposure,
  and child-after-switch exposure including its terminal stop all consume the
  same persistent mission ledger; trajectory replacement, guard handoff, ACK,
  and successor activation do not reset it.
- A received GNSS epoch certifies reception only at the exact receiver
  reference; no measured-support radius grants future candidate positions
  synthetic map support. Every candidate LOS records its sample count,
  covered count, unknown fraction and first missing support. Strict-global
  mode fails closed on any required unknown LOS support. Mission-best-effort
  mode retains a satellite unless it is hard-occluded, integrity-excluded,
  absent from the epoch or below the elevation mask, and applies
  `kappa_upper = 1-(1-kappa_known)(1-unknown_fraction)` before the unchanged
  canopy sigma model. This soft result is a degraded route-ranking estimate,
  not a certified integrity bound. The support policy and task mode are part
  of snapshot, cache and execution-certificate identity.
- Execution snapshots have separate freshness meanings. `localFreshAt`
  covers registered occupancy/support/SLAM authority and is the hard
  prerequisite for movement and certified braking. `globalFreshAt` covers
  GNSS epoch/current global evidence. Missing global evidence blocks
  `STRICT_GLOBAL`, but in `MISSION_BEST_EFFORT` it changes navigation state
  and ranking rather than suppressing a locally fresh execution snapshot.
- Route discovery/refinement is asynchronous and has its own bounded wall-time
  contract (`p4.forward.route_compute_budget_ms`, default `500 ms`). The
  worker preserves a direct-authorization reserve and may return a smaller
  set of fully refined candidates instead of a larger uncertified partial
  set. Before refinement, one shared batch ranks all enumerated coarse guides
  so a tight deadline does not silently select by enumeration order. This
  coarse result is never execution authority. Direct
  ForwardRisk authorization and reauthorization calls retain the independent
  `p4.forward.compute_budget_ms` limit of `150 ms`; a larger route-search
  budget must never be forwarded to the safety kernel. Mission-best-effort
  uses one coarse scheduling batch, refines the highest-ranked geometric
  channels that fit the route deadline, and re-certifies the resulting paths
  once. In `STRICT_GLOBAL`, incomplete evidence on an unselected alternative
  does not block another complete safe route; the selected route itself
  remains fail-closed.
- A runtime global-navigation exposure event must report the
  decision-time decomposition, not only the first point above AL. The
  execution event records the evaluated peak ratio, maximum continuous
  exceedance, positive exceedance integral, their exact policy limits, the
  carried episode state, and independent flags for hard-global, peak,
  duration, integral, and already-exhausted-episode causes. The first unsafe
  point remains spatial evidence, but it is not by itself the explanation for
  the aggregate risk observation. Repeated watchdog reads do not consume
  budget. In `STRICT_GLOBAL` any limit violation remains a hard rejection. In
  `MISSION_BEST_EFFORT`, peak, duration, integral and exhausted-episode flags
  are diagnostics/ranking/replan inputs while a fresh positive local-integrity
  certificate remains valid; they cannot independently command HOLD or
  braking.
- `LocalMotionAssurance` independently checks the exact nominal curve and all
  reachable braking curves at no more than `0.2 s` spacing. Its directional
  margin subtracts vehicle radius, measured tracking bound, a deployment-
  calibrated registered-surface bound, curve approximation and fixed safety margin
  from obstacle-surface clearance. The registered local map is authoritative
  SLAM geometry: current and active-window obstacles use the same local
  execution envelope. Planner code must not synthesize a current-to-source
  relative uncertainty by adding their absolute LiDAR PL values, and it must
  not charge a source frame's ICP residual a second time. Missing source
  identity/registration health, stale support, degenerate ICP, collision,
  excessive tracking error, or any
  unsafe braking curve is UNKNOWN/UNSAFE and cannot be overridden by GNSS
  exposure policy or good LiDAR FIM. Registered source-frame pose/voxel
  content and its exact, non-degenerate ICP registration health are immutable
  certificate identity. Absolute LiDAR PL fields remain compatibility
  diagnostics and do not affect local clearance or source admission. That
  source health travels with the registered frame when it enters the active
  window; planner-side Integrity history must not reconstruct or replace it.
  Matching requires the exact estimator frame ID and exact source stamp;
  temporal proximity or a matching ROS `frame_id` string is insufficient. A
  late exact health report replaces that registered-frame contribution as one
  atomic remove/add transaction. The first estimator frame explicitly marks
  the planner-map datum: it has zero inter-frame registration contribution by
  definition and therefore does not require a self-ICP report, while its scan,
  deskew, tracking, curve, and fixed margins are still charged normally.
  Current-scan surface evidence may supersede
  an older obstacle's provenance only when it reobserves the exact same
  occupancy voxel; an adjacent voxel has no surface identity authority. This does not create free-space
  evidence or remove occupancy. Invalid AABBs,
  missing/nonpositive frame identity, degenerate or incomplete source ICP
  health fail closed. In healthy registered-map operation the planner must not
  invent an elapsed-time-linear SLAM drift term: no producer supplies a
  certified bound for such a term, and fresh registered updates already own
  map-relative alignment. Loss of registration health, stale support or a
  source-data gap remains fail-closed and activates the existing certified
  braking path.
  Mission fallback additionally requires the monitor's online
  `LocalNavigationSourceEvidence`. Its 15-state covariance ordering is
  `[rotation, position, velocity, accelerometer bias, gyroscope bias]`. The
  result carries the source stamp, exact estimator frame, current H/V bounds,
  health flags, and source/model identities alongside every predicted bound.
  The
  producer rebuilds a local factor graph from an explicit whitelist of LiDAR,
  IMU and trunk factor types. Known pseudorange, Doppler, clock,
  `LinearContainerFactor`, generic pose/velocity/bias prior, generic pose
  between and damping factors are excluded because their C++ types cannot
  prove a local sensor origin. Any other unrecognized factor touching an
  X/V/B state invalidates the source, so current or marginalized GNSS
  information cannot contaminate it. A pose datum removes only the
  coordinate gauge; no zero odometry covariance, truth value or fixed
  localization bound is admitted. The source-specific LiDAR ARAIM run uses only fixed-map target
  blocks and lower-bounds every propagated result.
  `evaluateLocalNavigationIntegrity(source, model, offsets)` propagates that
  covariance over each actual nominal/braking time offset with the exact
  accelerometer, gyroscope, integration and bias-random-walk covariances bound
  into the estimator model identity. Its maximum propagation domain is the
  active odometry estimator's configured fixed-lag window, also bound into the
  model identity; it assumes no future LiDAR improvement.
  Horizontal and vertical bounds are the configured coverage multiplier times
  the propagated position covariance (never below current LiDAR HPL/VPL).
  For obstacle direction `d`, authorization uses
  `m_adjusted = m_clearance - H*||d_xy|| - V*|d_z|`; a certified-empty capped
  query conservatively subtracts `max(H,V)`. The same H/V bounds are charged
  against all six faces of the bound task-frame lattice, and the task frame
  and finite bounds are certificate inputs. Propagation time is source age at
  evaluation plus the sample offset from the active curve origin; a runtime
  suffix never resets covariance age to zero. Every nominal and reachable
  braking sample must have both adjusted margins greater than zero. Stale
  evidence, mismatched model
  identity, a non-positive/non-finite covariance, ICP degeneracy, horizon
  overflow or non-positive adjusted margin fails closed. The evidence and
  model identities are certificate inputs and persist across rolling-child
  replacement; ACK or trajectory-ID changes do not mint a new source.
  After activation, P5 records a locally authorized GNSS degradation through
  the existing debounced `REQUEST_REPLAN` action while the P4 certificate
  keeps the current trajectory executable; that GNSS condition alone never
  arms a braking guard.
  Active-window generation is a semantic state version, not a heartbeat:
  an unchanged complete window neither advances generation nor publishes an
  empty delta. Consumers distinguish the highest observed generation from the
  fully committed generation. A complete recovery response is eligible to
  become the committed baseline when its frame contract is valid and its
  generation is no older than both the request base and the local committed
  state; being behind an observed-but-uncommitted delta is not a rejection.
  Such a baseline remains recovery-pending and cannot make a frozen occupancy
  epoch healthy until later deltas or recovery reach the observed generation.
  Recovery serves the last non-empty, complete committed active-window state;
  a newer complete producer serial that has not yet been committed does not
  invalidate that baseline or mix its pending frames into the response. A
  producer-declared incomplete state still rejects recovery. Registered current
  frames run on a separate data-plane callback group, while active-window
  deltas, the recovery client response, and the recovery timer share the
  serialized control-plane group. Deadline handling removes and retries only a
  request with no ready response. A middleware arrival protects one existing
  control-plane scheduling period so a queued callback can run; an unmatched
  old arrival cannot suppress later finite timeout cleanup. An obsolete request
  never overwrites a newer committed generation.
  `p4.execution.max_tracking_error_m` is a loss-of-control rejection threshold,
  not a future-error bound. Local envelopes use the separate certified
  `p4.assurance.local_tracking_error_bound_m` (default `0.15 m`) and
  `p4.assurance.local_safety_margin_m` (default `0.20 m`); the P4 stopping
  boundary continues to use its independent `p4.forward.safety_margin_m`.
  ICP RMSE, condition, gamma and degeneracy are registration-health evidence,
  not a position covariance or a surface-error bound; they therefore cannot
  be added to every obstacle envelope. A deployment may provide a bound whose
  identity is tied to the actual LiDAR measurement/extrinsic/time-sync model.
  Simulation truth is evaluation-only and never enters that model. The
  repository's `0.02 m` / `uncalibrated_default_v1` values remain diagnostic
  compatibility fields and are excluded from the authorization envelope; they
  cannot grant MISSION fallback. Legacy `local_scan_error_min_m` and
  `local_lidar_error_multiplier` are accepted only for configuration
  compatibility and have no authorization effect.
  Route refinement and final assurance share the same immutable
  `LocalClearanceEvaluator` semantics and frozen evidence identity; a
  latest-snapshot reauthorization deliberately rebuilds it from the newer
  immutable evidence. Refinement samples guides at no more than `0.05 m`
  spacing, and every newly generated nominal/braking spline must retain the
  `0.05 m` planning reserve after smoothing. Runtime reauthorization keeps
  the unchanged strict condition `signed_margin > 0`. A final clearance
  failure records the nearest obstacle and escape direction as diagnostics;
  it does not retry, move or reinterpret the failed actual curve. Any bounded
  guide repair or alternate-channel enumeration must already have occurred
  before that immutable B-spline was generated. Soft guidance or planning
  buffer never grants execution authority.
  Runtime tracking above the certified local bound schedules the existing
  certified brake; the larger loss-of-control threshold remains the immediate
  emergency-revocation boundary.
- A complete long route is only `ROUTE_PREFERENCE_SELECTED` until its actual
  terminal-stop curve passes this unified assurance. RiskGrid, LiDAR FIM,
  GNSS exposure, recovery trend and mission progress may rank alternatives,
  but only the current reaction/braking envelope receives execution authority.
  P4 final admission and successor reauthorization produce the curve-bound
  assurance hash and execution mode. Publication verifies that immutable
  certificate rather than reinterpreting its GNSS result; runtime P5 may revoke
  only from post-activation facts. A newer-snapshot successor must rebuild that
  complete P4 certificate; its execution mode, assurance hash, and snapshot
  identity are atomically carried into publication. A GNSS-only recheck
  cannot reuse the old local-motion hash. New route
  authorization consumes the active episode's remaining duration/integral
  budget before publication, not only on the next runtime watchdog tick.
- `p4.debug_generation_probe_enable` is diagnostic-only and defaults false.
  For two adjacent snapshots that are both fresh at one evaluation time, it
  holds approved-trajectory positions and arrival times fixed and evaluates
  old-map/old-epoch, new-map/old-epoch, old-map/new-epoch and
  new-map/new-epoch. Direct ForwardRisk is also compared with each generation's
  RiskGrid interpolation. Probe results may classify map/support, GNSS epoch /
  satellite-set, sigma/canopy, geometry, time-growth, interpolation or mixed
  changes and never authorize motion. Marginal/confirmed/hard events also
  replay the fixed first-failure position across adjacent execution snapshots
  using both the production absolute arrival time and a fixed relative tau.
  The probe never bypasses map or GNSS freshness. Classification compares the
  typed boundary (index, safety/ranking/failure, satellite-set hash and grid
  interpolation state), not the index alone. Each counterfactual batch has a
  strict diagnostic budget; execution refreshes ROS time and repeats current
  map/GNSS/certified-Integrity freshness checks after the probe.
  When an interpolation anomaly is found, the probe records both temporal
  layers' eight spatial corners, their spatial/temporal/combined weights, PL,
  support and source identities, full satellite IDs/hash, geometry condition,
  worst exclusions and explicit failure state in a rate-limited corner CSV.
