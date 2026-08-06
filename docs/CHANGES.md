# Changes Log (IAP)

> 规则：任何代码改动必须在这里记录，并包含 IAP-RQ-XXX。

## Unreleased
- fix(planner-trajectory-command-qos): IAP-RQ-400 / IAP-RQ-410 — retain the latest accepted B-spline command for a late-starting trajectory server.
  - The fresh run `9e9103bd130040fcb905aca5a52e27af` had a healthy continuously advancing P0 grid but recorded zero `/drone_0_planning/bspline` messages: the startup base fallback was published before `traj_server` became ready, so the vehicle stayed still and every later seed remained 4.4 seconds long against the unchanged 2.5-second P0 horizon.
  - The planner publisher and trajectory-server subscriber now share reliable, transient-local, keep-last-one QoS. This changes command delivery only; P0 geometry/horizon/stale semantics, fixed P1 lambda/support, optimizer admission, and P5 gates are unchanged.
  - A focused regression fixes the QoS contract, and the planning-context/P1 admission/P0 runtime suites plus the fixed-lambda feedback loop pass.
- fix(planner-p0-frozen-occupancy-epoch): IAP-RQ-320 / IAP-RQ-400 — bind each P0 refresh to one immutable occupancy-map epoch while the live map continues updating.
  - GridMap now captures geometry, raw/fused/inflated buffers, cloud stamp, and generation under one short writer lock, then serves all refresh queries from that read-only copy. A 10 Hz cloud update after capture cannot mix generations inside the P0 snapshot; the next refresh captures the next epoch.
  - A configured epoch factory that cannot capture a complete post-cloud epoch fails closed as `occupancy_generation_changed`. The existing RiskGrid regression still rejects a query source that changes generation within one captured refresh.
  - The fresh run `5c19f4ac1976483fbc779af41c814bec` proved the prior whole-refresh terminal check was unsatisfiable in this fixture: provider construction took about 438 ms while occupancy advanced every 100 ms, leaving only a stale prior snapshot. Its failed preflight and ten diagnostic figures remain in the primary report.
  - Focused verification: all 33 `test_p0_risk_grid_runtime` tests and all 33 `test_risk_grid_map` tests pass. P0 geometry, horizons, occupied-skip semantics, 1.0-second stale timeout, fixed P1 lambda/support, and P5 semantics are unchanged.
- fix(planner-p1-base-prepass-fallback): IAP-RQ-400 / IAP-RQ-410 — keep P1 a soft preference when the two-stage base prepass succeeds but the fixed-200 risk lattice cannot cover the initial trajectory.
  - A full-support base prepass still enters only the normalized P1 stage. An unsupported but collision-feasible base result now advances the normal base-only receding horizon even when an incumbent exists; failed base optimization still keeps that incumbent and is never published.
  - Both distinctive and single-candidate planning paths update objective/timeline identity before choosing the next stage, so a base fallback cannot impersonate a P1 optimizer start or emit strict candidate evidence.
  - The fresh 30-second run `6b8193a91a744d639833a1b819d29cac` exposed the regression: the 4.4-second startup trajectory exceeded the unchanged 2.5-second P0 horizon and every successful prepass was discarded. The failed run and its ten diagnostic figures remain recorded in `docs/dev_planner/safety_planner_test_report.md`.
  - The follow-up run `2a1d0341223142c6a06626f2882ea3dc` proved late-subscriber delivery but published only one B-spline: 47 successful unsupported prepasses were retained in memory instead of advancing the vehicle into the fixed horizon. The policy regression now covers this existing-incumbent case explicitly.
  - The diagnostic lifecycle plot now renders large timelines as one vectorized collection and appends its complete report fragment atomically; a 30,000-row regression prevents the prior ten-minute per-row plotting path.
  - Focused verification: 14 `test_planning_risk_context` tests, the diagnostic/pre-admission Python tests, and the fixed-lambda feedback runner pass. The fixed lambda, 1.0-second stale timeout, P0 geometry/horizons, fixed support, P5 semantics, and P1-3 prohibition are unchanged.
- feat(planner-p1-evidence-v4): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make fixed-lambda convergence, diversity, and occupied-corner attribution independently replayable from one fail-closed evidence bundle.
  - The sole accepted schema is now `p1_evidence_provenance_v4`; launch manifests, validator/preflight, diagnostic smoke, and formal analyzer reject legacy or incomplete bundles.
  - Candidate evidence adds active/full base, raw-P1, normalized-weighted-P1, and total gradient norms; base/P1 cosine; frozen normalization and base-prepass fields; actual normalized-P1/anchor merit decomposition; aggregation parameters; and terminal/selection/replacement identities.
  - Five required sidecars record initial/final control points, each candidate's two fixed-200 profiles with invalid values separated from reasons, initial/final control-point and profile pairwise distances, base/P1 optimizer checkpoints, and the exact two-layer/trilinear P0 occupancy query support with captured raw/inflated map epoch diagnostics.
  - The explicit-bundle diagnostic generates the required ten views and marks candidate-dependent views `UNAVAILABLE` when source evidence is absent. The formal 23-figure contract embeds pairwise diversity in snapshot/candidate binding, actual normalized merit in objective decomposition, and inflated interpolation corners in the recorded scene overlay.
  - Focused verification covers the C++ sidecar writer, strict bundle preflight, analyzer candidate/hard-gate semantics, and Python schema parsing. The fixed lambda, fixed `200/200` support, P0/P5 hard semantics, and P1-3 prohibition remain unchanged.
- fix(planner-p0-occupied-attribution): IAP-RQ-320 / IAP-RQ-400 — bind occupied risk-grid failures to the actual spatiotemporal interpolation support and the occupancy-map epoch without weakening fail-closed semantics.
  - `RiskGridSnapshot::queryCost` has a read-only trace overload that records the query point/time, both participating horizon layers, every trilinear corner's weight/index/position/source flags/value/invalid reason, and the captured raw/inflated occupancy diagnostic.
  - GridMap exposes raw-cloud/fused versus inflated occupancy, voxel index/center, resolution, inflation, frame, cloud stamp, and a seqlock-style generation. P0 refresh rejects a changing generation before publishing a healthy snapshot and does not retry that generation.
  - Regression coverage proves that a free query point can still be conservatively rejected because an interpolation corner is occupied, and that a mid-refresh generation change is rejected. Existing occupied results remain invalid/unknown; fixed support and P0/P5 geometry are unchanged.
  - Focused verification: all 33 `test_risk_grid_map`, all 31 `test_p0_risk_grid_runtime`, and the fixed-lambda feedback runner pass after rebuilding IAP, `plan_env`, and `ego_planner`.
- fix(planner-p1-2-two-stage-normalization): IAP-RQ-400 / IAP-RQ-410 — replace enabled P1's one-stage low-weight optimization with a base-feasible prepass followed by a frozen, budget-normalized soft P1 stage.
  - The fixed constants are `lambda_ref=1e-5`, `beta=0.10`, and `reference_displacement=0.025 m`; the launch/manifest `p1.lambda_integrity` remains `0.00001`. The merit uses the raw P1 delta from the candidate seed plus a differentiable candidate-local anchor, and remains frozen through internal rebound restarts.
  - Production planning now records `base_prepass_start/end`, delays full P1 admission until the prepass result has `200/200` support, applies the existing candidate cap only to P1 optimizer starts, and preserves all collision, feasibility, swarm, terminal, replacement, snapshot, and P5 gates.
  - Singleton supplements are generated after the prepass from the true fixed-200 projected gradient of every active control point at maximum per-column displacements `0.025/0.05/0.10 m`; the base seed remains present and the fixed prefix remains unchanged.
  - Candidate evidence separates active and full base/raw-P1 norms, raw lambda-weighted and normalized-weighted P1 norms, base/P1 cosine, frozen normalization fields, base-prepass termination/duration, and actual normalized-P1/anchor merit components. The design rationale is recorded in ADR 0002.
  - Focused verification: the JSON feedback runner is green in about 0.18 s; all 26 `test_p1_integrity_cost`, 4 `test_p1_candidate_selection`, and 10 `test_planning_risk_context` tests pass; nested packages through `ego_planner` build successfully. No P1-3 or lambda sweep is included.
- test(planner-p1-2-fixed-lambda-counterexample): IAP-RQ-400 / IAP-RQ-410 — add a seconds-scale, ROS-free feedback command and a deterministic conflict fixture for the fixed `p1.lambda_integrity=0.00001` failure.
  - `run_p1_fixed_lambda_feedback.py` invokes only the named optimizer and selection GTests, emits structured JSON with elapsed time and red/green status, and fails when a filter executes fewer tests than expected.
  - The fixture keeps one immutable snapshot/query base and fixed 200-sample mean, verifies the analytic raw P1 descent probe, and characterizes the legacy one-stage optimizer counterexample: total/base descent moves along the positive raw-P1 gradient and increases both mean and max risk.
  - Candidate-selection coverage proves that the legacy optimizer winner remains selected for identity closure but is not rank-eligible or replacement-admitted, while a normalized descent candidate is selected and admitted. The feedback loop intentionally remains red until the two-stage optimizer test is implemented; no ROS launch, P1-3, or lambda sweep is involved.
- fix(planner-p1-2-selection-provenance): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — prevent a successful low-weight P1 optimizer result from silently replacing an existing trajectory when its fixed-lattice P1 risk regresses.
  - `P1CandidateSelection` separates same-attempt ranking from cross-attempt replacement. A full-support candidate must reduce or preserve both mean/max `c_pi` with one strict decrease before it is rank-eligible; a rejected replacement retains the existing trajectory and is deferred until a new healthy generation, without affecting P5.
  - The deterministic Attempt 19/20 regression records that these are separate replans: Attempt 19 descends P1 risk; Attempt 20 descends base/total objective but raises raw/weighted P1 and mean/max risk, so it is diagnostic-selected but not replacement-admitted.
  - Candidate evidence is now `p1_evidence_provenance_v2`; analyzer finite validation treats schema/run/manifest as provenance strings, reports typed numeric failures with full candidate identity, and requires new runtime bundles rather than accepting legacy rows.
  - Planning-context timeline emits an explicit `planner_activation` boundary; P1-2 startup analysis excludes only preceding P0 health rows. Exact scene alignment now also requires raw odom/truth messages within 100 ms of the accepted trajectory start while retaining exact P0 snapshot/cloud/b-spline binding.
  - Focused verification: P1-2 analyzer regression suite, `test_p1_candidate_selection`, `test_p1_replan_admission`, and `test_planning_risk_context`. No P1-3 or lambda sweep is included.
- fix(planner-p1-2-smoke-completeness): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make the formal recorder finalize its own manifest after rosbag flush (invoked through the runtime Python interpreter, so installed script mode cannot silently prevent recording), and extend the default P0 fixed lattice through `2.5 s` so the approximately `2.1 s` degraded-LiDAR initial P1 trajectory is not soft-fallbacked solely for temporal coverage. This keeps the fixed `0.00001` lambda, existing stale threshold, and all hard gates unchanged.
  - Base-fallback optimizer lifecycle remains explicit as `base_optimizer_start/end`; only an admitted, full-support P1 optimizer (or a full-support metrics-only measurement) writes a strict candidate CSV row. This prevents invalid fallback rows from impersonating P1 candidate evidence while preserving their diagnostic provenance.
  - The smoke preflight now rejects partial fixed-lattice support exactly like the analyzer, and P1 admission requires all 200 fixed-lattice samples before an enabled candidate can be selected.
  - Planner lifecycle tests can initialize a manager without an artifact writer; optional timeline output now returns safely until the optimizer registry exists.
- fix(planner-p1-2-artifact-provenance): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make every formal P1 bundle self-identifying and fail closed before the one authoritative analyzer run.
  - `test_planner.launch.py` now creates immutable `p1_evidence_provenance_v1` metadata (run ID, clean commit, workspace/install prefix, resolved launch/executable/library paths and hashes, export/bag paths, and start stamp). It passes the identity to P1 writers and the validator, and records a transient-local `/planning/evidence_provenance` message in each bag.
  - Debug, candidate, accepted-profile, context-sidecar, and planning-timeline CSV rows now include schema/run/manifest identity. The launch invokes `finalize_planner_evidence_manifest.py` after recorder exit to record recorder/validator completion and process end; `verify_safety_planner_evidence_bundle.py` is the red-capable smoke preflight and rejects stale, cross-export, legacy, mismatched, or unfinalized artifacts.
  - Analyzer and smoke preflight deserialize the bag’s provenance payload and compare schema/run/manifest/export/bag fields directly; they also verify current launch/executable/library file hashes against the manifest, rather than trusting paths or a topic name alone.
  - P1-2 analysis rejects invalid provenance for either formal bundle, emits the required artifact-provenance figure, and routes recording/install/provenance failures to `FAIL -> artifact provenance/runtime install/recording completeness debug`; only actual fixed-lambda gradient/contribution failures enter the lambda branch. The required P1-2 figure contract is now 23 images, including provenance.
  - Focused verification: `test_analyze_safety_planner_run_p1_2.py` and `test_verify_safety_planner_evidence_bundle.py`; no P1-3 or lambda sweep is part of this change.
- fix(planner-p1-2-health-gradient-admission): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — repair P0 freshness diagnostics, accepted-context classification, P1 gradient effectiveness, and same-generation retry storms without changing the fixed `0.00001` integrity weight, the 1.0 s stale timeout, or P5 semantics.
  - Added the shared `P1AcceptedContextValidation` contract. Runtime publication and offline analysis now distinguish strict trilinear spatial bounds, temporal horizon, frame/generation/query-time binding, freshness, coverage, and exclusive 200-sample miss reasons; an invalid final candidate cannot update `LocalTrajData` or publish.
  - P0 health JSON now exposes scheduled/start/end steady times, queue/provider/refresh duration, generation interval, input age/callback counts, health callback count, and process CPU delta. Refresh scheduling retains real completion timestamps and fixed deadlines.
  - P1 affine-field central finite-difference coverage now checks every free control point, and objective-applied optimization uses a bounded relative convergence precision while metrics-only retains the historical solver parameter. Accepted profiles append gradient, negative-gradient, pre-position, displacement, directional derivative, and cost-delta evidence.
  - Added generation-gated `P1ReplanAdmission`: admission runs before planning-context acquisition, one rejected stale/unavailable generation permits one acquisition/optimization, repeated ticks defer while the existing polynomial executes, and the next healthy generation unlocks one retry. P5 runtime/final/emergency paths bypass this controller.
  - The P1-2 analyzer now derives CSV/JSON/dashboard/final verdicts from structured hard-gate records, requires 16 nonempty PNGs, separates spatial/temporal/coverage evidence, and renders unavailable scene evidence fail-closed. Risk clouds carry their snapshot generation and header stamp; the sidecar appends the distinct trajectory-start stamp; scene evidence requires exact health/cloud/profile/trajectory frame, generation, and stamp binding.
  - The required diagnostic figures now distinguish temporal misses, plot positive/negative gradient, c_pi contour, displacement and directional evidence, classify refresh failures and planning rejections, and bin acquisitions/optimizers/rejections/deferred retries/reacquisitions/fallbacks, P0 latency/CPU, and callback progress on one time base.
  - Focused verification before runtime validation: analyzer suites P1-1/P1-2/P5-1/P0-6 = 15/39/111/2 passing; CTests `test_p1_integrity_cost` 20, `test_planning_risk_context` 7, `test_p0_risk_grid_runtime` 31, and `test_p1_replan_admission` 4 passing; aggregate IAP and planner builds pass. Historical workspace lint/XML failures remain separately recorded. Fresh P1-1/P1-2 evidence is pending the prescribed one-shot run.
- fix(planner-p1-2-freshness): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — repair authoritative P0 health and P1 fresh-context evidence.
  - `/planning/risk_grid_health` is unconditional absolute raw JSON, independent of debug/RViz, and records coherent refresh/input generation plus callback, steady-clock, queue, mutex, and worker timing.
  - The P1 degraded preset uses four deterministic worker-local P0 predictors without changing geometry or the 1.0 s stale timeout.
  - P1 records acquisition/optimizer/accept/pre-publish/publish on an immutable context timeline and fails stale contexts before trajectory update, accepted evidence, or bspline publish. The existing FSM failure/poly/random fallback budget remains the retry bound.
  - Manifests record raw-health, worker count, matched metrics-only reference identity, and context timeline; P1-2 requires thirteen nonempty figures including callback, freshness, and fallback timelines.
  - Reproduction (run serially, each for 90 s): `ros2 launch iap test_planner.launch.py experiment:=p1_degraded_lidar_good planner_safety_profile:=p1 p1.metrics_only:=true p1.lambda_integrity:=0.00001`; then repeat with `p1.metrics_only:=false p1.lambda_integrity:=0.00001`. Analyze only the four new absolute export/bag paths with `python3 scripts/dev_planner/analyze_safety_planner_run.py --experiment-id P1-2 --export-dir <p1-2-export> --bag-dir <p1-2-bag> --baseline-export-dir <p1-1-export> --baseline-bag-dir <p1-1-bag> --fail-on-threshold`.
- test(planner-p1-2-post-repair-fresh-pair): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make the P1-2 fresh-pair artifact contract authoritative and record its post-repair runtime result.
  - `analyze_safety_planner_run.py` replaces the P1-2 required-figure contract with exactly ten acceptance artifacts: scenario, topic activity, P0 health, snapshot/candidate binding, objective/gradient timeline, accepted-profile risk reduction, accepted-profile coverage, trajectory stability, artifact completeness, and cause exclusion. Legacy debug-summary, B-spline timeline, manifest-switch, and validation-summary figures remain diagnostic-only.
  - The new binding figure compares final profile, one-row sidecar, and debug attempt/candidate/snapshot/query-base tuples for the fresh P1-1/P1-2 pair. The objective figure compares metrics-only/applied state, lambda, integrity cost, weighted cost, and gradient ratio. The completeness figure reports manifest, validator, rosbag metadata, profile, atomic sidecar, debug CSV, and required-figure presence for both runs; required figures still fail closed when absent or empty.
  - Fresh runs on `2026-07-16` used P1-1 export/bag `1784181834979` / `20260716T060354Z` followed serially by P1-2 `1784181942139` / `20260716T060542Z`. Both raw runs produced complete nonempty artifacts and validator pass, but the sole P1-2 `--fail-on-threshold` analyzer returned `2`: P0 health/topic and accepted-profile context/reference-metadata gates failed. Acceptance remains **FAIL -> lambda/gradient debug**; P1-3 was not run.
  - Focused verification: Python P1-1 (15), P1-2 (30), P5-1 (111), and P0-6 (2) tests; `test_p1_integrity_cost` (15), `test_planning_risk_context` (3), and `test_p0_risk_grid_runtime` (28) CTests pass. Workspace-wide historical lint results remain reported separately by `colcon test-result --all`.
- fix(planner-p1-accepted-binding): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — bind P1 evidence and objective evaluation to one immutable planning context and make snapshot-boundary misses conservative.
  - P1 now carries `planning_attempt_id` and `candidate_id` through its objective metrics, debug CSV, accepted-profile rows, and the atomically replaced accepted-profile context sidecar. The analyzer rejects mixed candidate/attempt tuples alongside the existing generation/query-base/trajectory checks.
  - `position_out_of_map` and `position_out_of_interpolation_bounds` now receive a capped soft barrier whose gradient points toward the authoritative snapshot interior; stale, invalid, and time-horizon evidence remains non-low-risk without inventing a direction. Provider invalid/stale reasons survive snapshot querying when needed for that conservative classification. This only changes P1's soft objective and leaves P5 and the fixed P1-2 lambda unchanged.
  - P0 uses separate input, refresh, and health callback groups. Refresh copies a coherent input state before prediction; health records refresh/health callback stamps and requested/effective worker counts for bag/analyzer diagnosis. Predictor batches use deterministic original-index merging with worker-local `PredictorModule` copies when more than one worker is requested.
  - Both raw JSON and RViz P0 health topics are required by analyzer gates. An accepted P1-2 profile must also match an objective-applied P1 debug row on attempt/candidate/snapshot/query-base tuple.
  - Focused verification: `test_p1_integrity_cost` (15 tests), `test_risk_grid_map` (31 tests), `test_p0_risk_grid_runtime` (28 tests), and `test_analyze_safety_planner_run_p1_2.py` (26 tests). No fresh 90-second P1-1/P1-2 pair has been run, so acceptance remains `FAIL -> lambda/gradient debug` pending the prescribed fresh evidence.
- fix(planner-p1-accepted-context): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make accepted P1 profile evidence fail closed and decouple P0 health cadence from refresh latency.
  - Each accepted final B-spline now writes `planner_p1_accepted_trajectory_risk_profile_context.csv` beside the existing 200-sample profile. The sidecar binds profile sequence, trajectory, planning/acceptance timing, snapshot generation/query base, spatial/time bounds, expected/matched samples, and miss categories.
  - The analyzer selects only the greatest `profile_seq`; it requires exactly one complete `sample_index=0..199` set, one metadata tuple, an in-bounds/fresh matching context, P1-1 metrics-only metadata, P1-2 enabled metadata and exact lambda, and all required topic-health statuses before P1-2 can pass.
  - P0 adds an independent periodic health timer so the original JSON health topic continues reporting snapshot age while a heavy refresh is running. The launch manifest records P0 geometry, timing, and batch-worker configuration.
  - Focused P1-2 analyzer coverage now includes final-profile non-fallback, duplicate-index rejection, and required-topic failure. No fresh P1-1/P1-2 ROS rerun was performed in this change; acceptance remains `FAIL -> lambda/gradient debug` until those artifacts exist.
- fix(planner-p1-risk-profile-evidence): IAP-RQ-320 / IAP-RQ-400 / IAP-RQ-410 — make P1-2 risk-reduction evidence use accepted final-trajectory profiles instead of latest RViz cloud coverage.
  - `bspline_optimizer`: writes `planner_p1_accepted_trajectory_risk_profile.csv` beside the P1 debug CSV with 200 samples from each accepted final bspline, including objective application state, metrics-only state, lambda, snapshot generation, query base time, hit/valid/stale flags, `c_pi`, and miss reason.
  - `planner_manager`: preserves the planning P0 risk snapshot for the accepted bspline profile, aligns planner risk queries to the snapshot stamp, and logs the accepted profile path/sequence before publishing trajectory info.
  - `test_planner.launch.py`: records `p1.accepted_profile_path` in the manifest and widens the P1 degraded-LiDAR preset's P0 X extent so fresh P1 final accepted trajectories remain inside the reference risk grid without changing the global P0 map default.
  - `analyze_safety_planner_run.py`: makes P1-2 compare final `profile_seq` rows from P1-2 and P1-1 accepted-profile CSVs, keeps strict `>=20` and `>=0.5` coverage gates, requires strict mean/max `c_pi` reduction, treats missing/low-coverage profiles as hard failures, and keeps `/iap/rviz/predicted_pl_cloud` as diagnostic overlay evidence only.
  - Added required `p1_2_risk_sample_coverage_overlay.png`, expanded P1-2 cause exclusions for accepted-profile missing/coverage/reduction failure, and covered the analyzer with focused missing-profile, low-coverage, mean/max-not-reduced, cloud-miss happy-path, and required-figure tests.
  - Fresh rerun result is archived in `docs/dev_planner/safety_planner_test_report.md`: P1-1 accepted-profile reference passed coverage (`200/200`), but P1-2 final accepted profile had `0/200` finite `c_pi` samples, so P1-2 remains `FAIL -> lambda/gradient debug`.
- fix(planner-p5-3-query-alignment-proof): IAP-RQ-320 — make P5-3 query-alignment evidence authoritative and fail-closed.
  - `risk_grid_map.cpp`: `queryPredictedPL()` now emits P5-3 fixture diagnostics (`fixture_match`, expected PL, expected reason) from the same snapshot-relative query-time fixture decision that injects `10.2/10.2` PL; fixture interval checks use the existing boundary epsilon so tau-boundary queries cannot fall through to interpolation-only evidence.
  - `p5_runtime_integrity_gate`: carries the runtime fixture diagnostics into non-decision P5 status `samples` JSON alongside `query_tau_s`.
  - `analyze_safety_planner_run.py`: P5-3 query alignment now prefers runtime-emitted fixture fields over analyzer-reconstructed geometry, treats missing sample/marker/figure evidence as `FAIL`, keeps the active-window topic-gap gate, and routes all non-pass P5-3 outcomes to `FAIL -> 继续 debug P5-3 query alignment / PL-AL margin`.
  - Tests cover provider-backed tau-zero queries, future fixture queries with authoritative diagnostics, occupied-skip bypass, P5 status JSON fields, runtime `fixture_match=false` mismatches, and active-window odom gap failure.
- fix(planner-p5-3-query-alignment): IAP-RQ-320 — align P5-3 fixture evidence with the actual `queryPredictedPL()` decision path.
  - `risk_grid_map.cpp`: add a query-time P5-3 fixture override so future samples inside the configured spatial window and snapshot-relative `tau=[1.2,2.0]` return injected `10.2/10.2` PL with reason `p5_3_high_risk_zone`; tau-zero/current queries remain provider-backed.
  - `p5_runtime_integrity_gate`: expose `query_tau_s` in non-decision status sample diagnostics so analyzer geometry is aligned with the actual snapshot-relative query.
  - `analyze_safety_planner_run.py`: add query-aligned expected-vs-actual sample evidence, a dedicated query-alignment CSV, required `p5_3_query_alignment_*.png` figures, a P5 evidence-window topic-gap gate, and `BLOCKED_SCENARIO_MISSING` classification when query-alignment sample evidence is absent.
  - Tests cover query-time fixture PL at tau boundaries, current/tau-zero exclusion, analyzer failure on actual queried PL mismatch, active-window odom gap failure, and the exact query-alignment figure filename set.
- fix(planner-p5-3-plal-margin): IAP-RQ-320 — make P5-3 fixture evidence future-only without weakening P5 safety semantics.
  - `risk_grid_map.hpp`, `test_planner.launch.py`, and focused risk-grid/P0 runtime tests: narrow the default P5-3 fixture to the downstream corridor window `x=[-10.8,-8.7]`, `y=[-0.75,0.75]`, `z=[1.0,1.35]` while keeping `tau=[1.2,2.0]`, injected PL, `p5.max_bad_ratio`, and emergency thresholds unchanged.
  - `p5_runtime_integrity_gate`: add non-decision `samples` diagnostics to P5 status JSON with per-sample tau, position, PL/AL, IM, state flags, and source reason.
  - `analyze_safety_planner_run.py`: add future-only PL/AL gates, same-row sample-link attribution, flattened sample CSV export, and required `p5_3_plal_*.png` debug figures.
  - Tests cover tau-zero fixture exclusion, future sample inclusion, first-bad-tau `0.0` failure, sample-link attribution, emergency-storm rejection, and the exact PL/AL figure filename set.
- fix(planner-p5-3-debug-rerun): IAP-RQ-320 — preserve causal future-risk evidence for P5-3 without changing P5 safety-action semantics.
  - `p5_runtime_integrity_gate`: add non-decision diagnostic fields `current_reason`, `future_reason`, and `active_reasons` to status JSON so concurrent current and future gate reasons remain visible.
  - `risk_grid_map.hpp` and `test_planner.launch.py`: widen the disabled-by-default P5-3 high-risk-zone fixture bounds and include `p5.max_bad_ratio` in the manifest while keeping existing P5 thresholds unchanged.
  - `analyze_safety_planner_run.py`: classify P5-3 failures into scenario-isolation, reason-attribution, or PL/AL-margin branches; add debug figures with fixed `p5_3_debug_*.png` filenames; and retain `REQUEST_REPLAN` as a required acceptance gate.
  - Tests cover concurrent current/future P5 reasons, fixture defaults, analyzer recognition of `future_reason`/`active_reasons`, bad-ratio coverage, and visible future attribution that does not coincide with replan rows.
  - `safety_planner_test_report.md`: append the P5-3 Debug/Rerun section; the rerun remains `FAIL -> debug P5-3 PL/AL margin` and does not proceed to P5-4.
- test(planner-p5-1): IAP-RQ-320 — validate open-sky P5 no-false-trigger behavior.
  - `scripts/dev_planner/analyze_safety_planner_run.py`: add P5-1 topic/manifest gates, full P5 status JSON parsing, action/margin/final-gate CSV exports, RViz marker evidence extraction, P5 figures, and `PASS -> P5-2` / `debug P5 thresholds/AL provider` branching.
  - `test/test_analyze_safety_planner_run_p5_1.py`: add focused analyzer tests for OK status rows, emergency action, replan storm, final-gate fail, stale P0 health, and missing P5 status topic.
  - `docs/dev_planner/safety_planner_test_report.md`: archive the P5-1 launch/analyzer result, generated artifact paths, hard-gate failure metrics, and required figure conclusions.
- test(planner-p0-4): IAP-RQ-320 — accept P0 fallback/unknown semantics with P5 forced off.
  - `launch/test_planner.launch.py`: make the `p5_fallback_unknown` preset explicitly enable the P0 risk grid so the P0-4 command keeps P0 active even when P5 runtime/final are overridden off.
  - `scripts/dev_planner/analyze_safety_planner_run.py`: add P0-4 topic expectations, P5-off manifest allowance, fallback/unknown reason semantics, zero-risk fallback checks, and the `continue_to_P0-5` pass branch.
  - `docs/dev_planner/safety_planner_test_report.md`: archive the P0-4 launch, analyzer result, topic health, reason histogram, zero-risk fallback evidence, and figure conclusions.
- fix(phase1-demo9-hardening): IAP-RQ-081 — harden demo9 before Phase 2 validation.
  - `tools/build_phase1_ego_planner_closed_loop.sh`: required demo9 packages now fail fast when missing; `plan_env` remains optional; the script verifies `iap_phase1_tools phase1_closed_loop_logger` after build.
  - `launch/demo9_ego_planner_closed_loop.launch.py`: decoupled `planner_use_dynamic` from `use_so3_dynamics`, defaulted GNSS smoke tests to synthetic GPS ephemerides, added explicit RINEX file validation, completed the default `point0..point6` closed waypoint route, and dereferences installed config symlinks when creating runtime config copies.
  - `phase1_closed_loop_logger` and `validate_phase1_closed_loop.py`: record planner/controller odom feedback metadata and add `--official` validation checks for SO3 dynamics, IAP odom feedback, no truth alignment, and required planner command logs.
  - `README.md` and `docs/phase1_ego_planner_integration/topic_contract.md`: document demo8/demo9 usage, official validation commands, synthetic GNSS defaults, waypoint semantics, and debug-vs-official boundaries.
- feat(phase1-ego-closed-loop): IAP-RQ-081 — add the ordinary EGO planner closed-loop baseline on IAP odometry.
  - `launch/demo9_ego_planner_closed_loop.launch.py`: explicitly composes map generation, SO3 plant/controller, local_sensing, LiDAR body bridge, GNSS sim, IAP, EGO planner, EGO `traj_server`, optional RViz, and a run-duration shutdown.
  - `config/sim_demo9`: baseline IAP config for demo9; launch copies it to `/tmp` and applies `use_gnss`, `use_araim`, and `allow_truth_alignment` without mutating repo config.
  - `sim/ego_planner_swarm_ws/src/iap_phase1_tools`: new `ament_python` package containing `phase1_closed_loop_logger`, which writes `desired_vs_truth.csv`, `tracking_error.csv`, `planner_traj.csv`, `planner_cmd.csv`, `topic_contract.json`, and `phase1_summary.json`.
  - `docs/phase1_ego_planner_integration/topic_contract.md`: documents the Phase 1 topic/type/frame/remap contract and the truth-vs-IAP odometry boundary.
  - `tools/build_phase1_ego_planner_closed_loop.sh`, `tools/phase1/check_topic_contract.py`, `tools/phase1/validate_phase1_closed_loop.py`: build, topic smoke-check, and run validation helpers.
  - Default behavior keeps EGO planner `odom_world`, EGO `grid_map/odom`, and SO3 controller `odom` on `/drone_0_visual_slam/odom`; `/sim/drone_0/truth_odom` remains plant/sensor/logging only.
  - Demo9 GNSS defaults to RINEX multi-constellation `GPS,BDS,GAL,GLO`, including `/ublox_driver/glo_ephem` for GLONASS, while retaining launch args to override the ephemeris source, RINEX file, and constellation list.
  - Demo9 now starts RViz by default with `config/sim_demo9/demo9_gnss.rviz` and publishes demo8-style three-trajectory visualization topics: `/demo9/drone/path`, `/demo9/truth/path`, and `/demo9/desired/path`.
  - Demo9 exposes EGO multi-waypoint launch args `point_num` and `point1_*` through `point4_*`; the default single target remains `goal_x/y/z` mapped to EGO `point0`.
  - Demo9 GNSS satellite signal rays and NLOS path markers are thinner and semi-transparent by default; `gnss_sim_node` now exposes `signal_ray_width_m`, `signal_ray_alpha`, `nlos_path_width_m`, and `nlos_path_alpha`.
- feat(demo4-dynamics): IAP-RQ-002 / IAP-RQ-003 — add a complete real quadrotor dynamics control chain on top of `launch/demo1.launch`.
  - `launch/demo4.launch`: keeps demo1's map, local sensing, and RViz flow, but replaces fake truth odometry with `hover PositionCommand -> SO3ControlComponent -> SO3Command -> so3_quadrotor_simulator -> truth odom/IMU`.
  - `poscmd_2_odom`: adds `hover_cmd_publisher`, a steady `quadrotor_msgs/PositionCommand` source for controller input and visualization.
  - `config/sim_ego/demo4.rviz`: makes the global obstacle cloud and simulated LiDAR displays visibly prominent for demo inspection.
  - `config/sim_ego/fastdds_udp_only.xml` + `demo4.launch`: force FastDDS to use UDP transport for this demo, avoiding stale `/dev/shm/fastrtps_*` lock files that can make RViz discover point-cloud topics but receive no samples.
  - `apps/demo4_lidar_body_bridge.cpp`: converts demo4's map-frame simulated cloud into a lidar-frame scan topic for IAP odometry.
  - `so3_quadrotor_simulator`: adds optional `iap_imu/enable` + `iap_imu/topic` parameters that publish an extra ROS-standard IMU specific-force topic for IAP while preserving the original simulator `imu` output.
  - `apps/iap_rosnode.cpp`: allows launch-time `imu_topic` / `points_topic` overrides, so demo4 can wire simulated IMU and bridge cloud without cloning config files.
  - `config/sim_demo4/config.json`: demo4-specific IAP config that reuses `sim_ego` settings and selects the GPU odometry/sub-mapping/global-mapping configs for the simulated LiDAR-IMU pipeline.
  - `config/sim_demo4/config_ros.json`: demo4-specific ROS config that wires `/sim/drone_0/imu_iap` + `/sim/drone_0/lidar_body` and disables `libtrunk_extension.so`, so simulated random-forest cylinders do not inject trunk factors while validating LiDAR-IMU odometry.
  - `demo4.launch`: starts `demo4_lidar_body_bridge` and `iap_rosnode` by default (`start_iap:=true`) with `/sim/drone_0/imu_iap` and `/sim/drone_0/lidar_body`.
  - Run: `ros2 launch iap demo4.launch`
- perf(araim-fgo): IAP-RQ-241 / IAP-RQ-242 / IAP-RQ-243 / IAP-RQ-244 / IAP-RQ-245 / IAP-RQ-246 / IAP-RQ-300 — parallel hypothesis loop, inverse-free subset solves, and stronger FGO snapshot.
  - `araim.hpp`: added `parallel_hypotheses` and `hypothesis_threads` controls to make serial/parallel validation explicit.
  - `araim.cpp`: refactored `compute_core()` to precompute nominal normal-equation contributions, reuse per-row matrix/RHS terms across hypotheses, replace explicit `A.inverse()` with `LDLT` solves, and evaluate subset hypotheses into fixed-size result slots with optional OpenMP `parallel for`.
  - `enumerate_hypotheses()` now uses configured satellite / constellation / trunk priors instead of hard-coded fault probabilities.
  - `fgo_information_matrix.hpp` + `fgo_information_manager.cpp`: expanded `FGOPositionInfo` with `frame_id`, `p_world`, full 6x6 pose covariance, factor totals, key-count summary, GNSS sat/constellation lists, trunk landmark ids, and factor type tags derived from the smoother graph.
  - `integrity_extension.cpp`: switched proxy-frame FGO consumption to a single richer snapshot read, keeping `sigma_p` fallback behavior unchanged while exposing stronger FGO context to future integrity work.
  - `test/test_araim.cpp`: added serial-vs-parallel numerical equivalence coverage and default-value checks for the stronger FGO snapshot structure.
  - Validation:
    - `colcon build --packages-select iap --cmake-args -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF`
    - `colcon test --packages-select iap --return-code-on-test-failure`
    - Ad hoc benchmark (`24 sats + 8 trunk hyps`, 2000 iters): serial `0.0136 ms`, parallel `0.0193 ms`, `HPL delta = 0`
    - Ad hoc benchmark (`48 sats + 16 trunk hyps`, 1500 iters): serial `0.0207 ms`, parallel `0.0202 ms`, speedup `1.02x`, `HPL delta = 0`
- fix(clock-ownership): IAP-RQ-010 / IAP-RQ-200 — converge single-owner clock contract and suppress GNSS-owner read noise.
  - Added `clock_owner_mode` (`dual|odometry|gnss`) wiring across odometry/GNSS/trunk; defaults switched to `gnss` in odometry configs.
  - Added cross-module clock readiness marker in `IapSharedState` (`set_clock_ready/clear_clock_ready/is_clock_ready`).
  - GNSS extension now sets readiness when `C(frame_id)` is prepared and clears it on clock-chain reset/recovery.
  - Odometry in GNSS-owner mode now reads only `C(current)` and only when ready; non-ready frames are treated as warmup (no `c missing` warning storm).
  - Added lifecycle/ownership observability hardening (`KeyLifecycleMonitor`) and per-symbol relinearization policy registry with startup validation (including `l:Point2` threshold dimension).
  - Acceptance A/B replay (`dual` vs `gnss`) now shows `c missing=0`, `conflicts=0`, `violations=0`, and no hard optimizer errors.
- fix(clk-relin): IAP-RQ-010 — per-type iSAM2 relinearization threshold to eliminate sync-mode GPU linearization flood.
  - Root cause: `clk_bias` jumps ~`drift×dt`=61×0.1=6m/frame at cold-start, always exceeding global threshold 0.1, forcing iSAM2 to re-linearize clock state every frame. Each re-linearization calls `IntegratedVGICPFactorGPU::linearize()` without pre-computed result → sync-mode GPU stall → warning flood.
  - `odometry_estimation_imu.cpp`: replace single `setRelinearizeThreshold(double)` with `FastMap<char,Vector>` per key type: `x/v/b/e/r` keep 0.1; `c` (clock) uses `[clk_bias_relin_thresh, clk_drift_relin_thresh]`.
  - `odometry_estimation_imu.hpp`: add `clk_bias_relin_thresh` / `clk_drift_relin_thresh` fields to Params.
  - `config_odometry_gpu.json`: add `"clk_bias_relin_thresh": 500.0` / `"clk_drift_relin_thresh": 5.0`.
- fix(warnings): IAP-RQ-003 / IAP-RQ-010 / IAP-RQ-200 — silence 6 startup/runtime warning categories.
  - **W1** `config_ros.json`: add missing `dump_path` key.
  - **W2** `config_odometry_gpu.json`: add `clk_bias_noise=100` / `clk_drift_noise=1` (IAP-RQ-010 params missing from config); bump `isam2_relinearize_skip` 1→5 to batch relinearization and eliminate sync-mode flood during GNSS injection.
  - **W3** `config_ros.json`: complete `imu_qos` / `points_qos` / `image_qos` with `history`, `reliability`, `durability` fields.
  - **W4** `rviz_viewer.cpp`: skip `imu→lidar` TF when `imu_frame_id == lidar_frame_id` (Livox shares one frame — was spamming `TF_SELF_TRANSFORM`).
  - **W6** `integrity_monitor.cpp`: distinguish `UNSAFE`-due-to-`PL>=AL` (warn) from `UNSAFE`-in-recovery-phase (info) — message was misleadingly printing `"PL=1.719 >= AL=10.000"` when 1.719 < 10.000.
- feat(standalone): IAP-RQ-003 — self-contained ROS2 operation; no runtime dependency on `glim_ros` package.
  - **Root cause**: `config_ros.json` listed `librviz_viewer.so` and `libstandard_viewer.so` which only existed in GLIM's install. `iap_rosnode` silently skipped them via `continue`, leaving RViz blank.
  - **`librviz_viewer.so`**: ported from `glim_ros2/src/glim_ros/rviz_viewer.cpp` into `src/iap/util/rviz_viewer.{hpp,cpp}`. Publishes `~/aligned_points`, `~/odom`, `~/pose`, `~/map`; broadcasts TF `map→odom→base_frame` and `imu→lidar`.
  - **`libstandard_viewer.so`**: ported from `glim/src/glim/viewer/standard_viewer*.cpp` (4 files) into `src/iap/util/standard_viewer*.{hpp,cpp}`. Iridescence 3D desktop viewer unchanged.
  - **CMakeLists.txt**: added `find_package` for `tf2_ros`, `nav_msgs`, `geometry_msgs`; added `rviz_viewer` and `standard_viewer` shared library targets.
  - **`config_ros.json`**: removed `libmemory_monitor.so` (GLIM-only, no IAP port needed).
  - **CLAUDE.md**: updated primary run command to `ros2 run iap iap_rosnode`; legacy GLIM mode documented as secondary.
- refactor(config): IAP-RQ-025 / IAP-RQ-000 — consolidate config directory from 18 → 15 files.
  - **Merge 1 (logging)**: `config_logging.json` deleted; its `"logging"` section inlined into `config.json`.
    `util/logging.cpp`: use `GlobalConfig::instance()` directly instead of loading a separate file.
  - **Merge 2 (sensor + preprocess)**: `config_preprocess.json` deleted; its `"preprocess"` section
    appended to `config_sensors.json`. `config.json` global manifest: removed `config_preprocess` key.
    `preprocess/cloud_preprocessor.cpp`: now loads only `config_sensors` (one `Config` object) and
    reads both `"sensors"` and `"preprocess"` sections from it.
  - **Merge 3 (GNSS + integrity)**: `config_integrity.json` deleted; its `"integrity"` section appended
    to `config_gnss.json`. `config.json` global manifest: removed `config_integrity` key (the existing
    `config_gnss` key covers both). `integrity/integrity_extension.cpp:42`: changed
    `get_config_path("config_integrity")` → `get_config_path("config_gnss")`.
    `integrity/integrity_extension.hpp`: updated comments to reference `config_gnss.json`.
  - No algorithmic or parameter changes — pure file consolidation.
- feat(observability): IAP-RQ-200 / IAP-RQ-040 / IAP-RQ-002 — validation output & visualization suite.
  - **IAP-RQ-200 (ARAIM CSV)**: `config_integrity.json` new flags `enable_araim_csv`/`araim_csv_path`.
    `araim_debug.hpp`: added config constructor `AraimDebugCSV(bool, path)` and `write(report, AraimResult&)` overload that emits `row_type=epoch` + optional `row_type=worst_hyp` row with full per-hypothesis 3-term data.
    `integrity_types.hpp`: added `K_fa_used` field to `IntegrityReport`.
    `integrity_monitor.cpp`: `run_araim()` stores `last_araim_result_` and forwards `K_fa_used`.
    `integrity_monitor.hpp`: `last_araim_result_` member + getter.
    `integrity_extension.cpp`: reads config flags, instantiates `AraimDebugCSV`, calls `write()` each smoother update.
    Bug fix: `integrity_extension.cpp:220` — `msg.k_fa_used = 0.0` → `= report.K_fa_used`.
  - **IAP-RQ-040 (ICP CSV)**: `config_odometry_gpu.json` new flags `enable_icp_csv`/`icp_csv_path`.
    `odometry_estimation_gpu.hpp/.cpp` and `cpu.hpp/.cpp`: params read config; `update_frames()` appends per-frame row `stamp,frame_id,rmse,inlier_fraction,condition_number,gamma_lidar,drop_flag`.
  - **IAP-RQ-002 (timing)**: `std::chrono` instrumentation in `gnss_extension.cpp` (`on_smoother_update_finish_`), `integrity_monitor.cpp` (`compute()`), `araim.cpp` (`run()`), `trunk_detector.cpp` (`detect()`). Each writes `stamp,module,elapsed_ms` to `/tmp/iap_timing.csv`.
  - **Trajectory CSV**: `config_integrity.json` new flags `enable_traj_csv`/`traj_csv_path`. `integrity_extension.cpp`: appends `stamp,x,y,z` after each smoother update for trajectory comparison.
  - **Config GNSS CSV enabled**: `config_gnss.json` `enable_debug_csv` set to `true`.
  - **Python plotting**: `tools/plot_araim_timeline.py` (Fig B1/B2/B3), `tools/plot_icp_timing.py` (Fig C1/C2), `tools/plot_trajectory_comparison.py` (Fig D1).
- fix(odometry): IAP-RQ-130 — fix EstimationFrame ABI layout mismatch vs libglim.so.
  - IAP additions (`clk_bias`, `clk_drift`, `sigma_p`, `icp_quality`) were inserted before original GLIM fields, shifting `raw_frame` offset by ~136 bytes.
  - Moved all IAP-added fields to **after** `custom_data` (struct tail), preserving original GLIM field offsets.
  - SIGSEGV in `TrunkExtensionModule::on_new_frame_` resolved.
- feat(trunk): IAP-RQ-130 — activate trunk FGO extension module with ROS2 visualization.
  - `trunk_extension.hpp/cpp`: base class → `ExtensionModuleROS2`; added `create_subscriptions()`, `publish_markers_()`.
  - `publish_markers_()`: detections as yellow-green cylinders (1 s TTL, ns=`det`), landmarks as bright-green cylinders + white text IDs (ns=`lm`/`lm_label`), DELETEALL on each update.
  - `map_mutex_` guards all `map_.update()` / `map_.landmarks()` / `map_.confirmed_landmarks()` accesses.
  - `CMakeLists.txt`: `trunk_extension.cpp` moved out of `libiap` → separate `trunk_extension` shared library with `rclcpp` + `visualization_msgs` deps.
  - `config_ros.json`: `libtrunk_extension.so` added to `extension_modules`.
  - Entry point: `extern "C" create_extension_module()` → `TrunkExtensionModule`.
  - `on_new_frame_`: uses `frame->raw_frame->points` (CPU, always valid) instead of `*frame->frame` (may be GPU null).
- feat(gnss): IAP-RQ-025 — externalize GNSS parameters to `config_gnss.json`.
  - New `config/config_gnss.json` with 16 parameters (pr noise, canopy model, elevation cut, lever arm, clock Q, ECEF priors, debug CSV).
  - `gnss_extension.cpp`: load all params via `glim::Config`; removed `IAP_GNSS_DEBUG_CSV` env-var fallback (config is sole source of truth).
  - `gnss_handler_` changed to `std::unique_ptr<GnssHandler>` (fixes mutex move issue).
- feat(mapping): IAP-RQ-045 — add `multiscan_window` parameter to global mapping.
  - `GlobalMappingParams::multiscan_window` (default 3): keep last N frames for point-to-multiscan matching.
  - Config key `global_mapping/multiscan_window` in `config_global_mapping_cpu.json` and `config_global_mapping_gpu.json`.
- feat(gnss): IAP-RQ-020 — full ECEF pipeline with E(0)/R(0) free variables.
  - Replace local-ENU coordinate frame with ECEF throughout GNSS pipeline.
  - `PseudorangeFactor`: `NoiseModelFactor4<Pose3, Vector2, Vector3, Rot3>` — keys X(i), C(i), E(0), R(0).
    Corrections: Klobuchar iono, Hopfield trop, Sagnac, TGD. Analytical Jacobians for all 4 keys.
  - `DopplerFactor`: `NoiseModelFactor4<Pose3, Vector3, Vector2, Rot3>` — keys X(i), V(i), C(i), R(0).
    Sagnac velocity correction included.
  - `GnssExtension`: inserts `E(0)` + `R(0)` with loose priors (σ_E=5 m, σ_R≈5°) on first GNSS
    injection; stamps both on every injection to keep them alive in fixed-lag smoother.
    Subscribes to `/ublox_driver/iono_params` (Klobuchar GPS coefficients).
  - `GnssHandler::get_factors()` now takes `anc_ecef` parameter for Doppler Sagnac.
  - `gnss_types`: `SatObs` gains `tgd`, `svddt`; `GnssEpoch` gains `gps_sec`, `iono_params`.
  - `CMakeLists`: `ament_target_dependencies(iap gnss_comm)` so factor .cpp can include gnss_utility.
  - svdt sign fix: `sat.pr_meas = pr + svdt * c` (ADD per RTKLIB/LIGO; previous commit used subtract).
  - svddt correction applied to Doppler: `sat.dop_meas = dop_raw + svddt * c`.
  - Elevation filter: skip satellites below 5° elevation.
  - `ecef_to_local()` removed (no longer needed; factors work in ECEF directly).
- fix(gnss): IAP-RQ-020 — apply svdt satellite clock correction to pseudorange.
  - Root cause: `sat.pr_meas` was storing raw pseudorange `pr` without subtracting the
    satellite clock error `svdt * CLIGHT`. Each GPS satellite has a unique clock offset
    of ±1 µs ≈ ±300 m. Without correction, the shared receiver clock state `C(i)` received
    conflicting information from each satellite, preventing convergence (PR rms = 237 km).
  - Fix: `sat.pr_meas = pr - svdt * CLIGHT` in `on_range_meas_()`.
    `eph2pos`/`geph2pos` already compute `svdt`; it is now applied.
  - Expected outcome: `PR rms` drops from ~237 km to O(<20 m) after first convergence;
    `clk_bias` converges to true receiver clock offset (~300–400 km).
- fix(gnss): IAP-RQ-020 — clock warm-start to eliminate PR rms ~237 km divergence.
  - Root cause: `C(frame_id)` was always inserted as `[0,0]` (cold-start). iSAM2
    cannot converge receiver clock from 0 → ~350 km in one real-time linearization
    step, leaving large per-satellite pseudorange residuals that never collapsed.
  - Fix: `on_smoother_update_finish_` stores post-opt `clk_bias / clk_drift /
    frame_stamp` into atomics; next `on_smoother_update_` propagates them forward
    with clock-walk model (`bias_next = bias + drift × dt`, `drift_next = drift`).
  - Warm-start `call_once` log now includes initial bias/drift for observability.
  - Expected outcome: `PR rms` drops from ~237 km → O(<10 m) after first convergence;
    `clk_bias` rapidly tracks true receiver clock offset.
- fix(gnss): IAP-RQ-020 — inject `C(frame_id)` into `new_values`/`new_stamps` explicitly.
  - Root cause: glim's `OdometryEstimationIMU::update_smoother()` (no clock variable)
    takes precedence over IAP's override at runtime due to shared-library symbol resolution.
    `C(frame_id)` was never added to the smoother → iSAM2 fallback discarded all GNSS factors.
  - Fix: in `on_smoother_update_`, if `!new_values.exists(C(frame_id))`, insert
    `Vector2(0,0)` initial estimate + `frame_stamp` in `new_stamps`.  Also always writes
    `new_stamps[C(frame_id)] = frame_stamp` even when already present (odometry path).
  - One-time `call_once` log confirms which path was taken ('not in new_values').
- fix(gnss): IAP-RQ-020 — `epoch.stamp` now derived from GPS observation time (UTC)
  instead of `node_->get_clock()->now()` (wall clock).
  - Root cause: wall clock during bag replay differs from bag recording time by months,
    causing `GnssHandler::get_factors()` to drain all epochs as "too old" (delta >> `time_tolerance`).
  - Fix: `gnss_comm::gpst2utc(obs_list[0]->time)` → UTC Unix time → `epoch.stamp`;
    aligns with LiDAR `frame_stamp` within ±0.1 s.
  - Adds a `std::call_once` diagnostic log on the first epoch: prints
    `epoch UTC stamp`, `last_frame_stamp`, and `delta` for alignment verification.
- feat(gnss): post-optimization diagnostic via `on_smoother_update_finish`.
  - Registers `on_smoother_update_finish` callback; fires after iSAM2 optimization.
  - Stores last-injected PR/Doppler factors (by key-count heuristic: 2=PR, 3=Dop).
  - Queries `C(frame_id)` from smoother → `clk_bias [m]`, `clk_drift [m/s]`.
  - Evaluates `NoiseModelFactor::unwhitenedError()` per factor → PR RMS [m], Dop RMS [m/s].
  - Logs at `info` level (first call + every 50): `clk_bias / clk_drift / PR_rms / Dop_rms`.
  - Adds `factor_count_diag_` counter and `last_pr_factors_`, `last_dop_factors_` storage.
- IAP-RQ-020 (bridge): `GnssExtensionModule` — full ROS2 GNSS data pipeline.
  - `include/iap/gnss/gnss_extension.hpp` + `src/iap/gnss/gnss_extension.cpp`:
    `GnssExtensionModule : ExtensionModuleROS2`; subscribes `/ublox_driver/range_meas`,
    `/ublox_driver/ephem`, `/ublox_driver/glo_ephem`, `/ublox_driver/receiver_lla`.
  - NavSatFix → WGS-84 geodetic → ECEF origin + ENU rotation matrix; thread-safe origin.
  - `GnssMeasMsg` → L1 obs index, sat-clock-corrected pseudorange, Doppler m/s
    (`dop = -dopp_hz × c/f`), sat ECEF pos/vel from `eph2pos/geph2pos`/vel,
    transformed to local ENU; per-satellite elevation from ENU unit vector.
  - `OdometryEstimationCallbacks::on_new_frame` → track `last_frame_id/stamp`.
  - `on_smoother_update` → `GnssHandler::get_factors()` → inject into `new_factors`.
  - `create_extension_module()` C entry-point for GLIM dlopen.
  - CMakeLists.txt: `gnss_extension` SHARED lib; `find_package(glog, gnss_comm, sensor_msgs)`.
  - `config/config_ros.json`: `libgnss_extension.so` added to `extension_modules`.
  - `colcon build` passes; `libgnss_extension.so` exports all symbols.

## 2026-03-05 (Phase-4)
- IAP-RQ-331/421/422: Predicted ARAIM PL in planner + per-waypoint AL.
  - `include/iap/planner/predicted_araim.hpp` + `src/iap/planner/predicted_araim.cpp`: `PredictedAraimComputer`; calls `VisibilityPredictor::predict()` at each waypoint → builds `SatGeometry` list → `Araim::predict_geometry()` (r=0) → returns `pl_araim` (geometry-only upper bound).
  - `include/iap/planner/trajectory_types.hpp`: `CandidateTrajectory` += `AL_pred` (per-waypoint Alert Limit vector).
  - `include/iap/planner/integrity_planner.hpp` + `.cpp`: `Params::use_araim_pl`, `Params::araim_pred_params`; `set_occupancy()`, `set_epoch()`, `set_al_fn()` setters; `plan()` fills `AL_pred` via `al_fn_` callback and replaces `PL_pred[k]` with `araim_predictor_.predict_araim_pl(wpt)` when `use_araim_pl`; `evaluate()` uses `traj.AL_pred[k]` per step.
  - CMakeLists.txt: +predicted_araim.cpp.
  - `colcon build` passes.

## 2026-03-05 (Phase-3)
- IAP-RQ-241/242/243/244/245/246: Full ARAIM engine (single-fault, horizontal PL).
  - `include/iap/integrity/araim_types.hpp`: `FaultHypothesis{type, row, sat_id, p_fault}`, `SubsetSolution{d_horiz, sigma_ss_E/N/horiz, threshold, pl_faulted, fault_detected}`, `AraimResult{valid, pl_ff, pl_araim, S0, hypotheses, subsets, n_det, detected_rows}`.
  - `include/iap/integrity/araim.hpp`: `Araim` class; `Params{K_fa=4.5, K_md=5.5, K_ff=5.33}`; `run(epoch, n_trunk)` (real residuals + FDE); `predict_geometry(visible_sats)` (r=0, planning mode).
  - `src/iap/integrity/araim.cpp`: `build_G/W/r(epoch)`; `enumerate_hypotheses()`; `compute_core()` — S0=(G^T W G)^{-1}; subset solutions exclude row k; σ_ss_horiz, threshold, pl_faulted per hypothesis; pl_ff=K_ff·√(S0[0,0]+S0[1,1]); pl_araim=max(pl_ff, max_k pl_faulted_k); FDE: detected_rows.
  - `include/iap/gnss/gnss_types.hpp`: `SatObs` += `pr_residual` field (meas−pred [m]).
  - `include/iap/integrity/integrity_types.hpp`: `IntegrityReport` += `pl_araim`, `pl_ff`, `araim_valid`, `araim_n_hyp`, `araim_n_det`, `araim_detected_rows`.
  - `include/iap/integrity/integrity_monitor.hpp` + `.cpp`: `Params::araim_params`; private `Araim araim_`; `run_araim()` — calls `araim_.run()`, merges result into report (replaces `report.PL` with `pl_araim` when valid); trace log extended with `pl_araim/araim_n_hyp/araim_n_det`.
  - CMakeLists.txt: +araim.cpp.
  - `colcon build` passes.

## 2026-03-05 (Phase-2)
- IAP-RQ-131/132/133: Trunk data association + TrunkFactor + confidence-weighted TDOP.
  - `include/iap/trunk/trunk_map.hpp` + `src/iap/trunk/trunk_map.cpp`: `TrunkMap` with EMA-smoothed landmark table; nearest-XY + radius-gate association; stale pruning; `confirmed_landmarks()`; `TrunkLandmark{id, center_xy, radius, confidence, seen_count, last_stamp}`.
  - `include/iap/trunk/trunk_factor.hpp` + `src/iap/trunk/trunk_factor.cpp`: `TrunkFactor : NoiseModelFactor1<Pose3>`; 3D residual `r = z_k − R^T(c_k−p)`; analytical H (3×6); `make_noise(confidence)` diagonal noise deflated by `1/√conf`.
  - `trunk_types.hpp`: `TrunkDetectionResult` += `tdop_weighted` field (IAP-RQ-133).
  - `trunk_detector.cpp`: `compute_tdop()` extended — W = diag(conf²), TDOP_W = sqrt(trace((G^T W G)^{-1})).
  - CMakeLists.txt: +trunk_map.cpp, +trunk_factor.cpp.
  - `colcon build` passes.

## 2026-03-05 (Phase-1)
- IAP-RQ-311/312/313/314/321: Local occupancy + visibility predictor + canopy noise model + trajectory-dependent PL_pred.
  - `include/iap/map/local_occupancy.hpp` + `src/iap/map/local_occupancy.cpp`: `LocalOccupancyGrid` (voxel hash `unordered_map<VoxelKey, uint8_t>`, Morton hash); Amanatides & Woo DDA ray traversal; `occupancy_ratio(origin, dir, L)` → κ; `insert(cloud, T_world_sensor)`.
  - `include/iap/gnss/canopy_noise_model.hpp` (header-only): `σ_eff(κ, θ) = σ_c · exp(0.5·α·κ / sin θ)` and `info_weight_canopy()` (Talk §3.2).
  - `include/iap/gnss/visibility_predictor.hpp` + `src/iap/gnss/visibility_predictor.cpp`: `VisibilityPredictor`; ENU direction from (el, az); per-sat ray_occluded + occupancy_ratio → κ → σ_eff; `VisibilityResult{n_vis, vis_flags, kappas, sigma_effs, mean_kappa}`.
  - `gnss_types.hpp`: `SatObs` += `kappa` field.
  - `predicted_integrity.hpp/.cpp`: `set_occupancy(grid*)` + `set_epoch(epoch*)` API; `sigma_grow_at(pos)` = sigma_grow × max(1, 1 + β_vis·visibility_deficit + γ_κ·κ); called per waypoint in `predict()`.
  - CMakeLists.txt: +visibility_predictor.cpp, +local_occupancy.cpp.
  - `colcon build` passes.

## 2026-03-05
- IAP-RQ-900: Auto-generate IEEE Trans methodology chapter.
  - `tools/gen_methodology.py`: reads `docs/TRACEABILITY.md` → writes `docs/methodology/methodology.tex` (IEEEtran class, TikZ flowchart, per-module subsections with formulas, traceability table) and `docs/figures/system_flow.tex` (standalone TikZ).
  - Generated .tex has 12 balanced `\begin`/`\end` environments; structure verified.
  - Formula skeletons for RQ-015/020/040/100/200/220/320/400 embedded.
  - Run: `python3 tools/gen_methodology.py`
- IAP-RQ-500/510: Experiment runner + metrics (Passive/CovMin/IntegAware baselines).
  - `include/iap/experiments/metrics.hpp`: MetricSample (stamp,PL,AL,IM,violation,path_increment,control_effort,mode); MetricsCollector (add/reset/log_summary/write_csv); ExperimentResult; write_comparison_table() → Markdown.
    * Metrics: Time(PL>AL)%, AvgPL, MinIM, path length, mission time, control effort, success.
  - `apps/iap_experiment.cpp`: iap_experiment node; runs three baselines (Passive/CovMin/IntegAware) against a synthetic degraded-zone scenario; writes per-baseline CSV + Markdown summary table to /tmp/.
  - CMakeLists.txt: +iap_experiment executable.
  - `colcon build` passes.
- IAP-RQ-400/410: Integrity-aware planner + receding horizon loop.
  - `include/iap/planner/integrity_planner.hpp`: IntegrityPlanner with Params (w_integrity, w_mission, w_smooth, search_weight_multiplier, dt_execute, al_default); `plan(pos0,vel0,yaw0,goal,sigma0,report)` → best CandidateTrajectory; `execution_target(chosen)` → first point ≥ dt_execute.
  - `src/iap/planner/integrity_planner.cpp`:
    * Generates candidates via TrajectoryGenerator (IAP-RQ-300).
    * Predicts PL_pred via PredictedIntegrityComputer (IAP-RQ-320).
    * Evaluates J_total = w_int * Σ hinge(PL_pred−AL)² + w_miss * dist + w_smooth * effort.
    * mode==SEARCH boosts w_int by search_weight_multiplier (default ×5).
    * `execution_target()` returns first waypoint at stamp ≥ dt_execute (receding horizon IAP-RQ-410).
    * trace-log: n_candidates, best_id, J_total/J_int/J_goal/J_eff, AL, sigma0.
  - `colcon build` passes [33.5s].
- IAP-RQ-300/310/320: Planner modules — trajectory generator + predicted integrity.
  - `include/iap/planner/trajectory_types.hpp`: TrajectoryPoint (stamp, pos, vel, yaw), CandidateTrajectory (id, points[], PL_pred[], sigma_pred[], J_total/integrity/goal/effort).
  - `include/iap/planner/trajectory_generator.hpp/.cpp`: motion primitives (speed×yaw_rate×alt_rate grid); default speeds={0.5,1.0,1.5} m/s; yaw_rates={−0.3,0,0.3} rad/s; alt_rates={−0.2,0,0.2} m/s; horizon=3 s, dt=0.2 s; `generate(state)` → vector of CandidateTrajectory.
  - `include/iap/planner/predicted_integrity.hpp/.cpp`: sigma growth model σ(t+dt)=sqrt(σ²+σ_grow²·dt); PL_pred = K_pl·σ_pred (RQ-320 baseline); `predict(traj, sigma0)` and `predict_all(trajs, sigma0)`.
  - RQ-310: visibility/observability placeholder implemented; actual ray-cast deferred to map integration phase.
  - `colcon build` passes [2.89s].
- IAP-RQ-200/210/220: Integrity monitoring module.
  - `include/iap/integrity/integrity_types.hpp`: IntegrityMode (NOMINAL/CAUTION/ALERT/SEARCH), IntegrityReport (PL, AL, IM, mode, lambda_max_sigma_p, sat_nis, excluded_sats, gamma_R, icp_degenerate, gamma_lidar, tdop, safe()).
  - `include/iap/integrity/integrity_monitor.hpp`: IntegrityMonitor with Params; set_obstacle_distance(); compute(frame, epoch, trunk).
  - `src/iap/integrity/integrity_monitor.cpp`:
    * PL = K_pl * sqrt(lambda_max(Σ_p)) via SelfAdjointEigenSolver (RQ-200 baseline)
    * AL = al_scale * obstacle_dist − uav_radius, clamped to al_min (RQ-210)
    * IM = AL − PL; safe() when IM > 0
    * GNSS NIS gating: per-sat NIS_k = r_k² / σ_k²; exclude if > χ²(1,0.01); global NIS greedy FDE; gamma_R = sqrt(max_nis/thresh) (RQ-220)
    * Mode state machine: NOMINAL→CAUTION→ALERT→SEARCH→NOMINAL; recovery_counter
    * trace-log: PL/AL/IM/mode/lambda_max/icp_degenerate/gamma_lidar/tdop; warn on ALERT
  - `colcon build` passes [4.82s].
- IAP-RQ-100/110/120: Trunk detection + TDOP metric + health factor.
  - `include/iap/trunk/trunk_types.hpp`: TrunkObservation (center_xy, radius, confidence, bearing_xy, p_fault), TrunkDetectionResult (trunks, tdop, tdop2, lambda_min_H).
  - `include/iap/trunk/trunk_detector.hpp`: TrunkDetector with Params; height+range filter; 8-connected grid BFS clustering; Kasa circle fit; TDOP = sqrt(trace(H⁻¹)) via SelfAdjointEigen; `health_factor()` scalar [0,1] (Baseline-A).
  - `src/iap/trunk/trunk_detector.cpp`: implementation.
  - Full-B (trunk as FGO factor) deferred to upgrade phase.
  - `colcon build` passes.
- IAP-RQ-040: ICP quality report + noise inflation in LiDAR odometry.
  - `EstimationFrame::IcpQuality`: inlier_count, inlier_fraction, rmse, cond_number, degeneracy_flag, gamma_lidar.
  - `OdometryEstimationCPUParams`: +`icp_cond_threshold` (500), +`gamma_lidar_max` (10.0).
  - `odometry_estimation_cpu.cpp`: re-linearize at optimal; `hessianBlockDiagonal()` + JacobiSVD → cond_number; dynamic_cast GICP/VGICP factors → inlier metrics; `gamma_lidar = sqrt(cond/thresh)` clamped to max; BetweenFactor/PriorFactor precision divided by gamma².
  - Trace-level log per frame: inliers, rmse, cond, degenerate, gamma.
- IAP-RQ-020: GNSS measurement model — pseudorange + Doppler factors.
  - New `include/iap/gnss/`: `gnss_types.hpp` (SatObs, GnssEpoch), `pseudorange_factor.hpp`, `doppler_factor.hpp`, `gnss_handler.hpp`.
  - New `src/iap/gnss/`: `pseudorange_factor.cpp` (PseudorangeFactor: NoiseModelFactor2<Pose3,Vector2>, analytical Jacobians), `doppler_factor.cpp` (DopplerFactor: NoiseModelFactor3<Pose3,Vector3,Vector2>), `gnss_handler.cpp` (epoch queue, elevation-dependent noise, get_factors()).
  - Each satellite is an independent observation channel (per-sat gating ready).
  - Ephemeris (sat_pos/sat_vel) pre-computed outside factors; minimal stub accepted.
  - `colcon build` passes.
- IAP-RQ-015: Expose Σ_p from smoother marginal covariance.
  - `EstimationFrame`: +`sigma_p` (Eigen::Matrix3d, zero-init)
  - `odometry_estimation_imu.cpp`: `smoother->marginalCovariance(X(i))`; extract `pose_cov.block<3,3>(3,3)`; compute `trace`, `lambda_max` via SelfAdjointEigenSolver; `trace`-level log `sigma_p trace/lambda_max/PL_proxy`.
  - Interface placeholder: downstream can read `frame->sigma_p`; replace block with exact `HΣH^T` when needed (RQ-320).
- IAP-RQ-010: Extend state with `clk_bias`/`clk_drift` [δt m, δṫ m/s].
  - `EstimationFrame`: +`clk_bias`, +`clk_drift` (double)
  - `OdometryEstimationIMUParams`: +`clk_bias_noise`(100m), +`clk_drift_noise`(1m/s)
  - `odometry_estimation_imu.cpp`: `C(i)=Vector2[δt,δṫ]` key; loose PriorFactor; clock prediction δt_next=δt+δṫ*Δt; `trace`-level log: `clk_bias / clk_drift`.
  - `colcon build` passes; fixed-lag smoother runs with clock states. → `.githooks`; pre-commit doc-guard verified. AGENTS.md + CHANGES/TRACEABILITY/REQS confirmed present.
- IAP-RQ-002: Add `apps/iap_status.cpp` (smoke-test demo), `launch/iap_demo.launch.py`, `.clangd` (CompilationDatabase path). CMakeLists.txt: `IAP_VERSION` define, install launch/. `colcon build` 产生 compile_commands.json; workspace root symlink.
  - `ros2 launch iap iap_demo.launch.py` 可用，输出 `iap_status: OK`
- IAP-RQ-001: Generate `docs/SPEC_VS_IMPL.md`：对照 spec/ 与代码现状，汇总已实现/待实现条目，建议新目录结构。
- IAP-RQ-001: Renamed ROS2 package from `glim` to `iap`.
  - `src/glim/` → `src/iap/` (C++ source directory)
  - `include/glim/` → `include/iap/` (public header directory)
  - `cmake/glim-config.cmake.in` → `cmake/iap-config.cmake.in`
  - CMakeLists.txt: `add_library(glim)` → `add_library(iap)`, all install targets/exports renamed.
  - All `glim_LIBRARIES` → `iap_LIBRARIES`; install paths `share/glim` → `share/iap`, `bin/glim` → `bin/iap`.
  - `colcon build --packages-select iap` passes; `ros2 pkg list | grep iap` shows `iap`.
- Init: add traceability & agent rules. (IAP-RQ-000)

## 2026-07-16 (P1-2 Health/Freshness Repair runtime validation)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: Terminal validation preflight at clean `HEAD=e1e185d94aa98b78f2541f3d261cdc2386559aa8` ran the supplied Python compile and focused suites successfully (15/30/111/2 tests). The IAP build command then stopped before compilation because `/opt/ros/jazzy/setup.bash` was sourced under `set -u` and referenced unset `AMENT_TRACE_SETUP_FILES`.
- Per the one-shot validation contract, no retry, source change, planner build, CTest, workspace test-result scan, P1-1/P1-2 launch, analyzer run, artifact reuse, or P1-3 execution occurred. Outcome: **FAIL -> preflight environment setup**.

## 2026-07-16 (P1-2 fresh authoritative health/freshness validation)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: Expanded the P1-2 analyzer contract to twelve hard-gated figures, including recorded-only risk-scene alignment, authoritative raw P0 health/context freshness, complete gate dashboard, artifact completeness, and cause exclusion. Missing or unalignable source data now renders a labelled unavailable figure and fails closed.
- Updated the focused P1-2 analyzer suite for the twelve exact figure names, supplementary diagnostics, fail-closed scene behavior, dashboard gates, and nonempty plot output. Preflight passed Python compile, analyzer suites (15/33/111/2), the IAP and planner builds, and focused CTests (15/4/29). Historical workspace lint results remain separately non-clean.
- Ran one fresh serial P1-1/P1-2 pair with exact `p1.lambda_integrity=0.00001`, then invoked the analyzer exactly once with only those four paths. The analyzer returned exit 2, status FAIL, no warnings/inconclusive items, and `next_debug_branch=FAIL -> lambda/gradient debug`.
- Terminal evidence: both validators and manifests passed, objective application/binding/trajectory stability/recorded scene/P5 isolation passed, but raw P0 health, accepted-context validity, P1-1 coverage (`25/200`), strict c_pi reduction (P1-2 mean/max `0.383345/0.406930` versus `0.335276/0.338695`), and cause exclusion failed. P1-3 was not run.

## 2026-07-16 (P1-2 health/freshness repair terminal rerun)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: implemented truthful P0 refresh/queue/provider/generation/callback/CPU health evidence, shared accepted-context spatial/temporal/frame/generation/freshness/coverage validation, fixed-lambda gradient/FD/descent tracing, exact risk-cloud generation binding, and generation-gated P1 retry admission without changing the 1-second stale timeout, P0 map, P5 semantics, or lambda.
- Preflight passed Python analyzer suites `15/39/111/2`, builds, and focused GTests `20/7/31/4`. The serial fresh pair at commit `0861506` produced valid manifests, validators, debug CSVs, timelines, and bags, but no bspline or accepted profile: each run rejected 26 candidates as `temporal_out_of_horizon`; P1-2 raw health was stale in `162/275` rows.
- The one permitted analyzer invocation returned raw exit `137`. Health rows use wall/bag stamps around `1784221650`, while planning rows use simulation stamps around `1657065600`; the replan correlation plot attempted one-second bins over the approximately 127-million-second separation and the cgroup recorded `oom_kill=1`. Only 13/16 required PNGs were written and no structured final summary/hard-gate files were serialized. Terminal outcome: **FAIL -> analyzer timebase normalization/OOM debug**; P1-3 was not run.

## 2026-07-16 (P1-2 mixed-timebase, soft-fallback, and single-flight repair)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: bound analyzer time bins to the declared run duration, added an explicit clock-domain mapping contract, and fail closed with independent subplots when causal alignment cannot be proved. A provisional atomic summary is written before figure generation, including the estimated bin-memory ceiling and peak RSS, so plot failure cannot regress to an unexplained exit 137.
- Kept P1 as a soft preference: metrics-only and unavailable/low-coverage P1 contexts publish the base candidate, while an objective-applied invalid candidate keeps the existing trajectory or defers one base-initial fallback to a new generation. P5 remains the sole hard safety authority; P0 geometry, the 1-second stale timeout, and `p1.lambda_integrity=0.00001` are unchanged.
- Completed generation admission for startup, generation 0, unavailable snapshots, stale snapshots, and healthy retry. Each generation can enter acquisition/optimization at most once; repeated ticks defer while retaining the current polynomial, and no-existing-trajectory startup gets one base-planner fallback rather than a reacquisition loop.
- Reduced repeated P0 provider work by batching horizons per unique position and caching the LiDAR advisory within an immutable predictor input. Health JSON now reports unique positions, LiDAR evaluations, and cache hits without altering snapshot timestamps.
- Added deterministic mixed-timebase/OOM, temporal-horizon fallback, generation-singleflight, predictor-batch, and accepted-profile fallback regressions. Unmapped streams now render on independent clock-domain panels and invalid startup stamps retain exact source-row binding. Preflight passed analyzer suites `42/15/111/2`, `test_p1_integrity_cost`, `test_planning_risk_context`, `test_p0_risk_grid_runtime`, `test_p1_replan_admission`, and `test_predictor_module`; fresh authoritative evidence remains pending.

## 2026-07-16 (P1-2 mixed-timebase repair fresh terminal evidence)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: ran one serial 90-second fresh P1-1/P1-2 pair after green preflight, then invoked the analyzer exactly once with those four paths. Both validators passed and bags contain 18/20 bsplines; P1-2 P0 refresh mean/max/p95 is `329.794/375.824/351.757 ms` with max queue delay `0.179 ms`.
- Final profiles are available and bound: P1-1 is `190/200` with metrics-only temporal fallback; P1-2 is objective-applied and `200/200`. Strict effectiveness failed because P1-2 mean/max `0.414881/0.432228` exceeds P1-1 `0.414578/0.424604`. Admission/acquisition is one per generation, but optimizer-start maxima remain P1-1 8 and P1-2 4.
- The one permitted analyzer invocation returned raw exit 1 at `csv_artifacts` initialization order after writing 11 diagnostics but before provisional/final summary and hard-gate serialization; 2/19 required figures exist. No analyzer retry, source fix, artifact replacement, lambda sweep, or P1-3 run followed. Terminal outcome: **FAIL -> analyzer csv_artifacts initialization-order / optimizer-start singleflight debug**.

## 2026-07-18 (P1-2 analyzer, candidate evidence, and admission repair)

- IAP-RQ-320/IAP-RQ-400/IAP-RQ-410: analyzer artifact registries now exist before bag/exact-cloud work. P1-2 atomically writes a provisional summary with a fresh `analysis_run_id`; exceptions replace it with a structured fail-closed result containing phase, exception type, peak RSS, and only nonempty artifacts.
- IAP-RQ-410: admission decisions allocate a monotonic `planning_attempt_id` only for admitted work. The FSM passes it to its one immutable P1 context acquisition; deferred ticks retain ID zero. Candidate starts are bounded by `p1.max_candidates_per_attempt` clamped to 1–8; P5 remains outside this controller.
- IAP-RQ-400: added `P1OptimizationTrace`, append-only candidate optimization CSV evidence, and three required P1-2 figures for the candidate funnel, objective contribution, and gradient/displacement alignment. The required figure contract is 22 PNGs.
- Verification: P1-2 analyzer suite (43), `test_p1_integrity_cost`, `test_p1_replan_admission`, `test_planning_risk_context`, `test_p0_risk_grid_runtime`, and `test_predictor_module` pass. No fresh launch or one-shot analyzer evidence was produced.

## 2026-08-01 (P1-2 fixed-lambda authoritative candidate evidence repair)

- IAP-RQ-400: candidate optimization rows now measure pre/post `c_pi` on one fixed 200-sample lattice against the immutable P0 snapshot. Each row includes immutable-support, initial-control-point, and configuration signatures; pre/post base/total/raw/weighted-P1 objective and gradient values; displacement; raw integrity-gradient alignment; support coverage; solver status; and explicit selection evidence.
- IAP-RQ-410: each optimizer start writes one authoritative candidate row. The manager annotates selection score/reason after normal multi-candidate ranking, while unsuccessful candidates remain explicit unselected evidence rather than disappearing.
- IAP-RQ-400/IAP-RQ-410: the P1-2 analyzer now fails closed for missing/non-finite candidate schema, incomplete fixed-lattice support, invalid candidate limits, absent exactly-one selected successful candidate, or disagreement among optimizer timeline, candidate CSV, and accepted profile. Candidate figures now present funnel/risk, objective-and-gradient ratios, and gradient/displacement/`delta c_pi` evidence.
- Verification: rebuilt IAP and planner targets; `test_p1_integrity_cost` includes a fixed-`lambda=0.00001` metrics-only/enabled same-snapshot diagnostic gate and persists its complete JSON/CSV evidence in `/tmp/iap_p1_fixed_lambda_feedback_diagnostic.json` and `/tmp/iap_p1_fixed_lambda_feedback_diagnostic.csv`. The analyzer requires nonempty solver termination evidence and counts raw admission/acquisition records (including identical duplicate rows), while regressions cover candidate reconciliation and fail-closed support/selection/singleflight behavior. The sole fresh 90-second P1-1/P1-2 pair and sole analyzer invocation are recorded in `docs/dev_planner/safety_planner_test_report.md`; they fail closed on incomplete/malformed runtime evidence, so P1-3 was not run.

## 2026-08-01 P1 candidate generation / objective-alignment debug

- IAP-RQ-400/IAP-RQ-410: candidate fan-out now records topology input/survival, return count, cap/truncation, optimizer/full-support/eligibility counts, selection, replacement outcome, and singleton cause. Enabled P1 alone supplements a topology-caused singleton with deterministic immutable-snapshot risk-gradient displacements of 0.025/0.05/0.10 m; metrics-only is unchanged and the base candidate is retained.
- IAP-RQ-400: P1's selected aggregation mode is `fixed_200_mean`, aligned with the admission lattice; `lambda_integrity=0.00001`, P0 geometry/staleness, and P5 semantics are unchanged. Candidate provenance persists aggregation mode, temperature, adaptive/fixed sample counts, and peak contribution. Evidence schema is now `p1_evidence_provenance_v3`, rejecting older rows.
- IAP-RQ-400/IAP-RQ-410: rejected replacements emit a run-bound replacement decision plus paired fixed-200 candidate/retained-incumbent profiles. The accepted profile remains the actually published trajectory and is never reused for rejected-candidate evidence.
- IAP-RQ-400: `scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py` requires explicit export, bag, and run ID; it writes the six diagnostic figures and an Observation/Verdict/Conclusion fragment. It is diagnostic-only and cannot satisfy P1-2 effectiveness/progression.
- Reproduce the focused checks with `ctest --test-dir /home/dev/ws_iap/build/bspline_opt -R test_p1_integrity_cost --output-on-failure` and `ctest --test-dir /home/dev/ws_iap/build/ego_planner -R test_p1_candidate_selection --output-on-failure`. For one explicit fresh bundle: `python3 scripts/dev_planner/analyze_p1_candidate_diagnostic_smoke.py --export-dir <fresh-export> --bag-dir <fresh-bag> --run-id <manifest-run-id>`.
- IAP-RQ-400/IAP-RQ-410: fixed-lambda diagnostic evidence now records final control-point hashes plus raw/weighted-P1/total gradient-to-displacement products. `evaluateP1RawCostForTest` provides a snapshot-bound fixed-200 negative-gradient probe; it does not alter the production objective or lambda.
- IAP-RQ-410: recorder finalization now records the exact recorder command, exit code, and completion state. Use `python3 scripts/dev_planner/verify_planner_recorder_smoke.py --export-dir <fresh-export> --bag-dir <fresh-bag>` before any planner diagnostic smoke.
- IAP-RQ-400/IAP-RQ-410: the diagnostic-only analyzer now validates retained replacement/profile identity on attempt/candidate/generation/query-base, rejects an incumbent publish identity mismatch, emits nine named diagnostic figures, and can append each figure's run-bound Observation/Verdict/Conclusion to the primary report. The H1 fixed-200 probe now uses the optimizer's mutable control-point block and adds an H4 central finite-difference binding check; it leaves production lambda/objective unchanged.
- IAP-RQ-410: recorder preflight is fail-closed for malformed manifests and has focused success, malformed, nonzero-exit, and bag-mismatch tests. The fresh 30-second diagnostic smoke's recorder bag finalized correctly, but it emitted no candidate optimization rows after first-trajectory generation failed; the diagnostic analyzer exited `2`, so no formal pair or P1-3 was run.

## 2026-08-02 P1 pre-admission feedback instrumentation

- IAP-RQ-400/IAP-RQ-410: every enabled-P1 attempt now writes a schema-v3 pre-admission row binding immutable snapshot generation/stamp/query-base to initial duration/margin, all fixed-200 miss classes, admission verdict/reason, and a base-optimizer postpass result. This adds no P1 admission and preserves `lambda_integrity=0.00001`, P0 geometry, and P5 semantics.
- IAP-RQ-320/IAP-RQ-410: accepted profile samples now contain the base collision check's exact inflated-map predicate, while the recorder includes the planner's remapped inflated occupancy cloud. This is only evidence for occupied-start diagnosis; it never relaxes required 200/200 support.
- IAP-RQ-400/IAP-RQ-410: `verify_p1_pre_admission_feedback.py` is a seconds-scale explicit-bundle feedback loop. `analyze_p1_pre_admission_smoke.py` writes eight pre-admission figures, each with Observation/Verdict/Conclusion, and fails closed when P1 candidate evidence is absent. Focused Python regressions cover the feedback contract and figure completeness.
