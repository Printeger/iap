# IAP Talk Spec (摘要版)

> Goal: implement optimization-based Integrity-Aware Active Perception.
> Sensors: GNSS (pseudorange+doppler), IMU, LiDAR (GLIM ICP baseline).

## A) State
x(t) = { p, v, q, b_a, b_g, clk_bias δt, clk_drift δt_dot }

## B) Integrity quantities
- PL(t): protection level (baseline proxy from Σ_p)
- AL(t): alert limit (derived from obstacle clearance / safety margin)
- IM(t) = AL(t) - PL(t)
- Safe iff PL(t) < AL(t)
- Mode machine:
  - NOMINAL: IM large
  - CAUTION: IM small
  - SEARCH: IM <= 0 or GNSS fault alarm

## C) Estimator (fixed-lag factor graph)
- IMU preintegration factors
- LiDAR ICP relative pose factors (GLIM baseline)
- GNSS tightly coupled factors:
  - pseudorange residual: meas - pred
  - doppler residual:    meas - pred (m/s)
  - sat pos/vel from broadcast ephemeris
- Estimator must expose Σ_p (position covariance block) or equivalent info matrix

## D) Integrity monitoring (baseline)
- PL proxy: PL = K * sqrt(lambda_max(Σ_p))
- GNSS per-satellite NIS gating:
  - compute per-sat residuals and NIS
  - downweight (gamma_R) or exclude satellites
- fused integrity report:
  - PL, AL, IM, mode
  - per-sat {NIS, gamma_R, exclude}

## E) Prediction for planning (baseline)
For each candidate trajectory τ:
- predict GNSS visibility set V^(τ) using point-cloud occlusion model
- registered hit-only LiDAR may supply immutable trusted local-map support:
  inside the configured sensor envelope no-hit is model-complete; outside,
  expired, unhealthy or frame-invalid support is UNKNOWN. This inference is
  never relabelled as measured free space. Envelope freshness is evaluated at
  the current `evaluation_time` of the immutable planning round, not at a
  candidate's future arrival `query_time`. The future time still selects the
  prediction layer and covariance growth. Successful geometry is complete
  only over that sample's explicit local satellite set; publication/execution
  recheck freshness against their then-current ROS/simulation time. A newer
  generation does not itself revoke an in-flight immutable evaluation;
  collision deltas remain independently rechecked.
  P0 uses the same rule: a no-hit voxel inside a `MODEL_COMPLETE` envelope is
  eligible for prediction even when it is not ray-observed, without changing
  its occupancy evidence to `OBSERVED_FREE`. All other incomplete support and
  occupied/inflated geometry remains fail-closed.
  RiskGrid may guide coarse search only. Its PL is interpolated as a scalar
  only when all positive spatial/time corners share satellite-set, support and
  prediction-source topology and report normal geometry. Topology changes
  request a direct ForwardRisk recheck; degenerate geometry is an explicit
  invalid state with non-finite PL, never a huge finite interpolation value.
  P4 pre-execution authority comes from direct batched ForwardRisk samples of
  the actual B-spline and its actual arrival times. It consumes a
  lightweight immutable execution-risk snapshot published from a complete
  occupancy/support + GNSS epoch + certified-Integrity tuple before the dense
  RiskGrid. RiskGrid is a bounded, discard-on-timeout background search
  product; its delay or absence is not execution revocation evidence. A cached
  direct batch is reusable only for the same curve and execution snapshot.
  Publication validates the P4 certificate identity, freshness, deadline and
  task/execution mode without replaying that batch or adding another MISSION
  exposure gate. P5 begins after ACTIVATED
  and monitors current-time evidence. The execution snapshot path is
  occupancy-generation-driven, single-slot/latest-wins; its 50 ms timer is
  only a missed-notification watchdog. Commit callbacks execute outside the
  occupancy writer lock with lifetime-safe weak ownership; a newer request
  supersedes any older in-flight build before it can publish. Factory failures
  are classified and do not terminate the worker. Every attempt records classified
  capture/build/publication timing, and dense-grid predictor workers yield
  their current scheduler timeslice while execution work is pending. A
  dense-grid callback cannot overwrite it. A
  failed background build is reported separately from the retained grid: the
  old generation may guide search only while it is still fresh.
  Missing/stale RiskGrid also cannot block candidate enumeration: frozen
  occupancy supplies geometry and the execution snapshot direct batch supplies
  risk ranking and final authority. The position-major provider API evaluates
  each spatial point across all horizons without reconstructing a flat-query
  hash table. Admitted rich voxels remain horizon-specific, while skipped
  occupancy/support diagnostics are stored once per spatial cell and exposed
  with their exact queried horizon; stage timings and the 500 ms
  whole-generation deadline remain observable.
  Trusted support retains up to 64 original current-frame envelopes within the
  unchanged one-second hard validity window. Each real B-spline point uses the
  newest fresh envelope that spatially covers that point, so unrelated stale
  map regions do not reject a fresh execution corridor and unrelated fresh
  regions do not validate an expired corridor. This is model support only and
  never changes ray-observation occupancy labels.
  Formal planner defaults use GPS+BDS+GAL+GLO. New advisory satellites require
  three consecutive epochs before admission, while disappearance, certified
  exclusion and hard occlusion remove them immediately. Execution checks use
  deterministic braking-window common cores rather than one intersection over
  the whole future curve. A window contains the commands committed through
  reaction/handover delay plus every reachable certified stopping curve;
  adjacent windows overlap by 0.4 s and both satellite sets must independently
  pass on identical transition samples. Far-future support loss therefore
  cannot remove a satellite from the current stopping envelope. An identical
  set in adjacent windows reuses exact GNSS calculations but does
  not merge their logical window IDs or safety certificates. Every runtime P5 sample
  must still resolve to its original window and matching canonical set hash.
  An insufficient local core is UNKNOWN, never an interpolated PL, and RiskGrid
  remains search-only. The explicit legacy A/B policy retains the old
  whole-curve intersection. In the final 0.2 s after the last discrete guard
  anchor, the exact hard-terminal spline remainder is registered as the only
  reachable suffix braking curve and can be activated without extending its
  approved endpoint or deadline. P4 certificates bind both the deterministic
  window layout and exact per-point satellite sets; policy/layout/set mismatches
  fail closed during successor reauthorization and runtime monitoring.
  Window responsibility is frozen once, when that exact curve is committed:
  absolute sample times, braking anchors, evidence IDs, original window IDs
  and the layout hash do not move during execution. The watchdog projects all
  still-reachable nominal rows and braking curves whose anchors have not
  passed, preserving their original window memberships. New maps and GNSS
  epochs update the physical evidence at those same points; they do not redraw
  the experiment. The runtime watchdog uses the already-defined scheduling
  clock (last odometry stamp plus same-machine steady elapsed) so a long safety
  callback cannot freeze its evidence time and evict every causal snapshot.
  Fresh exact-identity controller feedback supplies executed `p/v/a`; planning
  and generated state remain on exact sensor time, and every freshness and
  local-safety gate remains unchanged.
  Before any runtime continue/reject/brake transition, P4 persists the full
  window evidence and makes the execution event reference it. A diagnostic
  background four-cell replay then separates map/support change, GNSS
  epoch/set change, normal time growth and mixed interaction without entering
  the authority path. The replay is triggered by execution-snapshot identity,
  not RiskGrid publication, and compares every cell at the same physical focus
  rows. Its classification includes exact PL/support/sigma/geometry evidence,
  so an unchanged unsafe index cannot hide a numerical jump as `STABLE`.
  A bounded history of four completed execution snapshots is retained only to
  choose the newest result causal to the current ROS evaluation stamp; the
  work channel itself remains single-slot/latest-wins and every selected
  result is freshness-checked again. The registered sparse snapshot also
  carries the captured current vehicle pose/radius so its physically occupied
  footprint remains measured free without copying or relabelling the whole
  dense observation layer.
  Optionally, canopy sigma may use a frozen occupancy-generation clearance
  field to turn distance from an occupied surface into a smooth bounded LOS
  proximity. This leaves hard intersections, support boundaries, elevation
  masks and the epoch-sigma floor unchanged; `0 m` disables the transition.
- predict LiDAR observability proxy O^(τ) (ICP quality proxy, map-based, may include occlusion)
- propagate Σ -> Σ_pred using empirical growth model (keep exact interface)
- compute PL_pred from Σ_pred

## F) Planning objective (optimization/selection)
- J(τ) = Σ hinge(PL_pred - AL)^2 + λ_goal * dist(goal) + λ_u * effort
- Receding horizon: plan H seconds, execute first Δt, replan
- The executed Δt/prefix is accepted only with a bound endpoint/deadline and
  zero terminal velocity/acceleration imposed as hard equalities together with
  the execution-continuous start state and approved endpoint. Retiming is
  allowed only before final dynamics, collision and time-aligned risk checks.
  While a new result is pending, an
  unchanged valid prefix continues; runtime collision or Integrity revocation
  triggers replan/braking/emergency handling.
  Certified current-Integrity samples are selected causally at the planner's
  evaluation timestamp; callback ordering may not turn a next-tick sample into
  a false missing/unsafe result. Candidate final gates are transactional with
  the incumbent execution certificate.
- When no full topology route is risk-safe, the planner may crop the
  continuous risk-safe geometry-common corridor before the first unsafe or
  unsupported sample, subtract current-speed stopping distance plus
  vehicle/tracking reserve (up to an 8 m lookahead), and publish it only
  as a terminal-stopped `LIMITED_PREFIX`. Too little safe distance remains a
  normal HOLD, including when current-speed braking exceeds the configured
  maximum prefix progress; a limited prefix is not a full-route risk selection. A
  geometry-only or risk-incomplete fallback cannot move. The sub-metre prefix
  prefers the continuous nominal public approach while that
  approach remains geometry-common/free, avoiding an unintended half-voxel
  climb/descent or early branch caused by A* cell centres. The actual stopped
  B-spline remains independently support/risk checked. During execution the
  latest coherent P0 transaction supplies a newly fresh certified-current
  Integrity check; raw map advisory at the receiver remains predictive, while
  the remaining curve is still checked at its future positions and times.
  A valid limited prefix is not replaced for ordinary worker/generation churn:
  after a one-second minimum commitment, a replacement must extend the safe
  common endpoint far enough to cover the next command-switch, successor
  generation and two direct-authorization intervals, plus a stability margin
  (with a small jitter floor). The successor has an independent, single-slot
  deadline lane: it starts preparing as soon as the parent is committed, while
  the default 1.5 s value remains the latest allowed start. An early route
  result immediately produces and fully certifies the final child B-spline,
  braking library and fixed risk windows against its immutable switch anchor.
  Because route delivery is asynchronous, the prepare-only pass—not the FSM
  request site—binds the child start to the parent's exact position, velocity
  and acceleration at that anchor. The selected safe guide endpoint owns the
  child terminal; a stale periodic local target cannot append an unchecked
  tail.
  It consumes one already-generated frozen guide. Any certification failure
  terminates that child attempt without a full-channel fallback, alternate
  guide, crop or retiming; the parent continues to its certified stop.
  Ordinary P4 rate limiting does not apply.
  Preparation is not motion authority and cannot alter the parent or runtime
  P5 state. At handoff the planner does not rerun route search, A* or
  optimization; it only reauthorizes the exact cached curve and braking
  evidence once against the latest execution snapshot. That reauthorization
  is mandatory even when the snapshot ID is unchanged because local freshness
  and GNSS diagnostics can advance. Publication then validates the rebound P4
  certificate once. In bounded normal-channel comparison, a typed failure of
  the last frozen guide does not discard an earlier complete bundle: the
  comparison restores that winner, rebinds it to the current planning attempt,
  and sends it through the same latest-snapshot, P5, identity, local-safety and
  braking gates before publication. The failed immutable guide is not retried
  or altered.
  Shared-corridor risk must not worsen and the extension is independently
  certified. The successor is published
  only if parent identity, switch window, position/velocity/acceleration
  boundary state and current direct authority still match; otherwise the old
  prefix continues to its endpoint. Activation never occurs before the fixed
  anchor; a late callback compares the parent at the anchor with child time
  zero. The cached handoff is not re-gated by a stale P1/RiskGrid planning
  context because the current execution snapshot is its authority.
  The committed terminal-stopped curve also carries independently generated,
  checked stopping curves at <=0.2 s state anchors. When input data expires,
  execution reaches the nearest future anchor within 0.2 s and publishes the
  selected curve with a new braking certificate/trajectory identity; it never
  extends the original deadline or passes the approved endpoint. Before that
  anchor the certificate remains the original `LIMITED_PREFIX`. A slight
  future exceedance up to 0.5% may enter a 0.35 s confirmation state only when
  a directly certified guard brake can still stop before the unsafe boundary;
  its reserved deadline must leave the full confirmation window. Confirmation
  replaces the reservation with a certified brake no more than 0.2 s ahead;
  otherwise the observation is immediately HARD.
  the guard transition is prequeued at the trajectory server as soon as the
  state becomes armed, after the exact guard curve passes the latest collision-
  delta check, so its absolute switch deadline does not depend on the next
  planner watchdog tick. Updated complete direct `SAFE` evidence requests a
  same-ID cancellation, but the planner keeps the guard pending until the
  trajectory server acknowledges `CANCELED`; an `ACTIVATED` acknowledgement
  makes a racing cancellation irreversible. These command/status topics are
  namespaced per drone;
  three distinct semantic tuples confirm braking, while an updated SAFE tuple
  disarms without changing the original trajectory. A larger/current
  exceedance in STRICT_GLOBAL, stale/unknown local input, inadequate stopping
  margin, current Integrity failure, collision or tracking loss is immediately HARD.
  HARD-scheduled and activated braking cannot be canceled. Ordinary replanning
  is suspended while that braking certificate is active, so worker/generation
  churn cannot relabel the stopping curve. Planner-side time alone cannot
  declare that switch: only a matching traj_server `ACTIVATED` acknowledgement
  changes the trajectory ID and braking authority; a missing acknowledgement
  or `ABSENT` response at the switch deadline fails closed. The first hard
  guard freezes its anchor, deadline and curve identity; later watchdog ticks
  cannot roll the stop forward or clear its server state.
- Coarse/refined routes are only `CANDIDATE_READY`. Formal `RISK_SELECTED`
  authority is created after the actual terminal-stop B-spline passes direct
  braking-window checks at its real arrival times, including all braking
  curves and both cores in every transition overlap. A braking-branch failure
  is first projected to its nominal anchor station; duplicated transition or
  brake rows are never summed into executable progress. The first actual-curve
  failure is recorded as a typed terminal result and never fed back into curve
  generation. The immutable failed B-spline is not cropped, shifted, retimed,
  switched to another channel or reinterpreted. Multi-channel preparation may
  continue only with the next already-frozen guide while the committed parent
  trajectory remains retained; the failed channel's HOLD disposition is not
  inherited by that new preparation transaction. The optimizer preserves the
  frozen guide tube and the unchanged final structural corridor check remains
  authoritative. A stoppable prefix of the
  corridor shared by at least two topology channels is generated independently
  and passes the same complete actual-curve certification chain. Every point
  of that nominal executable prefix must be inside every channel tube; tube
  overlap alone is insufficient. The selected branch itself cannot be
  relabelled as that public prefix. Diagnostic fixed-point replay is followed
  by a refreshed-time repetition of every execution safety gate.
  Publication on
  a newer execution snapshot performs at most one exact-curve reauthorization;
  an ID-only change is not a safety failure.
- Global GNSS degradation and local collision avoidance are separate
  contracts. GNSS advisory is evaluated on the actual curve as task-position
  exposure (peak, continuous duration, positive integral, 0.5 s rolling worst
  section, time-weighted CVaR90 and predicted recovery). The comparison
  thresholds are `r<=1.05`, `continuous<=2.3 s`,
  `integral<=0.115 ratio*s`; the dense-forest profile uses `8.0 s` and
  `0.4 ratio*s`. `STRICT_GLOBAL` still requires complete `PL < AL` evidence.
  `MISSION_BEST_EFFORT` uses exposure and recovery only to classify, record
  and order candidates. Locally safe motion with a usable certified brake is
  `MISSION_DEGRADED_EXECUTION` even when those diagnostic thresholds are
  exceeded or GNSS is incomplete. It does not claim global integrity.
  Replanning does not reset the diagnostic episode; `0.5 s` below `0.95*AL`
  ends it.
  Predicted recovery uses that same sustained `0.95*AL` condition; merely
  crossing back below AL for one sample is not recovery. Runtime diagnostics
  expose all four threshold explanations: a hard or
  peak limit, excessive continuous duration, excessive positive integral, or
  a previously exhausted persistent episode. Values and thresholds come from
  the same `TrajectoryAssurance` evaluation that made the decision. A logged
  first point just above AL locates the exposure; it must not be presented as
  proof that this one point triggered a MISSION brake.
- GNSS reception at the receiver is not extrapolated through a `0.45 m`
  measured-support bubble. Candidate LOS support is sampled explicitly. In
  strict mode, an unknown sample is fail-closed. In mission-best-effort mode,
  unknown sky support retains the satellite with the deterministic conservative
  canopy proxy `1-(1-kappa_known)(1-unknown_fraction)`; only true hard
  occlusion, certified exclusion, disappearance and the elevation mask remove
  it immediately. This prevents a LiDAR vertical-FOV boundary from deleting
  an otherwise received satellite while keeping the degraded estimate clearly
  separate from certified integrity.
- A fresh registered map can publish local execution authority without a fresh
  GNSS epoch. Global and local freshness are evaluated separately: local
  freshness remains mandatory for every nominal/braking curve, while global
  freshness determines `NORMAL`, `CONTROLLED_DEGRADED`, or
  `MISSION_DEGRADED` state under the selected task contract.
- The asynchronous topology/refinement worker has a separate `500 ms` route
  deadline. The direct GNSS/local authorization batch remains limited to
  `150 ms`; route computation cannot lend its remaining time to a safety
  query. Best-effort evaluates all coarse channel guides in one shared batch
  to schedule expensive refinement by mission risk, then directly evaluates
  the refined candidate set again. Coarse results are hints only and cannot
  authorize motion.
- Every locally feasible channel is converted before resampling into a
  bounded actual terminal-stop
  B-spline and a side-effect-free prepared bundle (dynamics, tracking,
  braking, fixed-window, local-motion and direct GNSS evidence).
  Prepared curves are rebound to one latest immutable snapshot before the
  winner is installed. Hard failures are eliminated first; incomplete work is
  reported as `PARTIAL_COMPARISON` and can authorize only the already
  certified finite prefix. `STRICT_GLOBAL` admits only formal GNSS evidence.
  `MISSION_BEST_EFFORT` may choose the least-risk degraded actual whenever
  its local-motion and braking certificate is valid. Exposure exhaustion is
  diagnostic and cannot independently cause braking. Ordering is formal
  before degraded, then conservative peak ratio, continuous exceedance,
  positive exposure integral, actual/braking-tube unknown exposure, actual
  progress and a stable bundle hash, with the old stable channel used only as
  a true-key tie-break. Complete locally safe bundles are therefore still
  orderable when their GNSS intervals overlap.
  The generator bounds the endpoint only by visible local support, required
  progress, terminal stopping and dynamics. It does not crop or regenerate a
  curve to fit an exposure duration. STRICT remains unchanged.
  Winner, runner-up, actual endpoints, unevaluated suffixes and the full
  geometry/time/risk decomposition are retained. Execution remains a rolling
  reaction-and-stop envelope and is re-evaluated on every new immutable
  snapshot.
- Globally incomplete candidates are compared first by the mean unknown
  LOS-sample fraction over eligible satellites, then usable satellite count,
  geometry condition, predicted support recovery, LiDAR observability, local
  clearance and progress. One missing ray sample is not treated like an
  entirely unsupported satellite ray.
- Local execution is admitted by `LocalMotionAssurance`, not by RiskGrid or
  LiDAR FIM. It evaluates the actual nominal and braking splines against
  obstacle surfaces using the vehicle/tracking/calibrated-surface/curve/safety
  envelope. The registered map is authoritative SLAM geometry, so current and
  older registered obstacles use the same local envelope. The planner neither
  adds current/source absolute LiDAR PL to invent a relative error nor charges
  an arbitrary elapsed-time-linear drift term. No certified producer bound
  exists for that term; registered-map relative alignment belongs to SLAM.
  Missing provenance, exact source registration
  health, fresh support, or any braking-curve proof is fail-closed. LiDAR
  observability remains a ranking/recovery cue and cannot shrink this
  certified envelope by itself.
  ICP RMSE, condition and gamma are health signals only: RMSE is not treated
  as centimetres of pose uncertainty. A fixed, offline-calibrated
  `local_surface_error_bound_m` supplies the surface term and its calibration
  ID is certificate identity. The P4 refiner and final gate use the same
  `LocalClearanceEvaluator` implementation and require matching frozen
  snapshot/evidence identity; deliberate latest-snapshot reauthorization
  rebuilds it from the new immutable evidence. Generation keeps an additional
  `0.05 m` buffer, while runtime authorization remains `margin > 0`. A failure
  records the nearest obstacle and escape direction for diagnostics only; it
  cannot move, crop, retime or regenerate the immutable actual curve.
  Exact ICP health is frozen into the registered-frame message before the
  frame becomes a long-lived active-window obstacle. Existing source PL fields
  are compatibility diagnostics only. Planner-local history cannot reconstruct
  source health; exact estimator-frame identity plus exact acquisition stamp
  is required. Late exact health replaces the frame atomically. The
  explicitly identified first estimator frame is the planner-map datum, so it
  has no self-ICP registration term; scan/deskew and all other local margins
  still apply. A
  registered active-window generation advances only for a semantic frame,
  pose, source-health, or completeness change; unchanged complete updates are
  not heartbeat deltas. GridMap keeps observed and committed generations
  distinct: a valid complete recovery baseline may commit while behind an
  observed delta, but it stays recovery-pending and cannot authorize a frozen
  occupancy epoch until the committed state catches up.
  A
  current scan can supersede old provenance only for the exact same occupied
  surface voxel and never fabricates observed free space. A successor rebound
  to a newer execution snapshot rebuilds local and global P4 assurance and
  publishes the rebound P4 mode/hash rather than the stale admission.
  The controller loss-of-control threshold (`0.75 m`) and stopping-distance
  margin (`0.5 m`) are not silently reused as per-point uncertainty. The local
  certificate has independent tracking (`0.15 m`) and fixed-clearance
  (`0.20 m`) bounds, each exposed as a policy parameter. Legacy ICP-RMSE scale
  parameters are parse-only diagnostics and never enter the clearance
  envelope; a surface residual is neither pose error nor GNSS measurement
  sigma.
  Exceeding the certified tracking bound activates a certified brake; the
  larger controller threshold is reserved for immediate loss-of-control
  revocation.
- The public execution state is `NORMAL_EXECUTION`,
  `CONTROLLED_DEGRADED_EXECUTION`, `MISSION_DEGRADED_EXECUTION`, or
  `RECOVERY_OR_EXIT`. A route preference
  has no motion authority. Only the reaction-and-stopping envelope certified
  by one immutable execution snapshot is published; P4, P5 and runtime bind
  the same actual-curve, braking, obstacle-source and assurance identities.
  Loss of local proof activates the existing certified brake before the
  approved boundary; MISSION exposure threshold exhaustion does not.

## Frozen beam evidence and interval channel decisions

- The first-hit renderer publishes `/iap/simulator/lidar_beam_evidence` with
  `HIT`, `NO_RETURN`, and `INVALID` outcomes plus the complete sampling,
  range, frame, timestamp, model-version, and content-hash contract. The local
  map associates it with the hit cloud only when both timestamps match.
- `RegisteredLidarWindow` accumulates explicit rays into immutable two-bit
  `UNKNOWN` / `OBSERVED_FREE` / `RAW_OCCUPIED` snapshots. Hit prefixes and
  no-return beams establish free space; hit endpoints remain raw occupied;
  missing, invalid, mismatched, out-of-range, or free evidence older than one
  second is fail-closed. Collision inflation is never GNSS raw obstruction.
- Each frozen occupancy epoch owns the accumulated snapshot. GNSS evaluates
  each satellite on one LOS sample set and reruns the real PL solver twice:
  unknown LOS is open only for the diagnostic lower bound, while satellites
  with incomplete support are excluded from the formal upper solve. Too few
  remaining satellites makes the upper PL unavailable, never a finite
  penalty. PL/AL authorization always uses the upper bound.
- Final curves store lower/upper peak, rolling, duration, integral, and
  recovery metrics. After unchanged hard local gates, authorization groups
  come only from actual-curve assurance. Conservative upper values provide a
  deterministic ordering when complete bundle intervals overlap; overlap by
  itself is not `PARTIAL_COMPARISON`. Whole-grid unknown fraction is
  diagnostic only. Final-curve, clearance-tube, every braking tube, and GNSS
  LOS support remain route-scoped evidence.
- Whitelisted incomplete GNSS evidence in `MISSION_BEST_EFFORT` remains
  incomplete/unknown with its typed failure; no finite PL or synthetic ratio
  is invented. The episode fields remain diagnostic observations and route
  comparison inputs, not a consumable execution balance.
- When two incomplete channels cannot yet be ordered, their executable common
  region is the nominal path contained by every candidate's safe tube, not a
  requirement that candidate centerlines coincide. A feasible result is an
  ordinary `LIMITED_PREFIX` with zero candidate/channel IDs and no route-winner
  authority. It reserves the complete stopping distance and uses the standard
  stopped-B-spline collision, clearance, dynamics, direct-risk, brake,
  assurance, P4-certificate publication, and post-activation P5 runtime chain.
- The observation planner may score earlier stop points by minimum normalized
  information gain across channels, using frozen raw occluders and the
  registered sensor model. That score is diagnostic only: it may shorten a
  feasible endpoint but cannot grant motion, relax a safety gate, or veto the
  farthest safe stoppable prefix. Zero gain or unavailable sensor geometry is
  therefore not a HOLD reason by itself.

Historical captures made before the beam-evidence topic contain only
decision-level data and are labeled `non_exact_replay_for_local_evidence`;
they must not be presented as exact frozen-evidence replay.

## G) Upgrade items (optional, after baseline closes the loop)
- trunk landmarks + TDOP
- full ARAIM hypothesis set beyond per-sat gating
- exact covariance propagation with HΣH^T + measurement information update
