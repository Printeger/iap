# IAP launch entrypoints

Only the four files below are canonical user entrypoints. New demos must not be
added at the launch root; validation and paper-specific wrappers belong in the
directories documented below.

| Entrypoint | Runtime composition | Environment |
|---|---|---|
| `glio.launch.py` | GLIO only | none; live and rosbag use the same topics |
| `glio_integrity.launch.py` | GLIO + Current Integrity Monitor | none |
| `iap_sim.launch.py` | all four IAP modules | selected simulation scenario |
| `iap_flight.launch.py` | all four IAP modules | real vehicle IO only |

Every entrypoint requires an absolute `output_dir`. Canonical launches write no
CSV to the repository root. The deprecated
`phase2_planner_integrity_evaluator` is forbidden from all four graphs.

## Commands

```bash
# GLIO with live input, or with a rosbag played separately on the same topics
ros2 launch iap glio.launch.py \
  output_dir:=/tmp/iap_runs/glio_001

# GLIO + current integrity
ros2 launch iap glio_integrity.launch.py \
  output_dir:=/tmp/iap_runs/integrity_001

# Full simulation
python3 src/iap/scripts/dev_planner/run_gate0_qualification.py \
  --output-root /tmp/iap_runs/sim_001/preflight \
  --gpu-preflight-only
ros2 launch iap iap_sim.launch.py \
  scenario:=fused_nominal \
  output_dir:=/tmp/iap_runs/sim_001

# Full flight is deliberately fail-closed and requires deployment calibration
python3 src/iap/scripts/dev_planner/run_gate0_qualification.py \
  --output-root /data/iap_runs/flight_001/preflight \
  --gpu-preflight-only
ros2 launch iap iap_flight.launch.py \
  flight_authorized:=true \
  controller_handshake_confirmed:=true \
  beam_evidence_topic:=/vehicle/lidar/beam_evidence \
  goal_x:=10.0 goal_y:=0.0 goal_z:=2.0 \
  local_surface_error_bound_m:=0.04 \
  local_surface_error_calibration_id:=vehicle_01_heldout_2026_09 \
  local_surface_error_calibration_manifest:=/data/iap/calibration/vehicle_01.json \
  output_dir:=/data/iap_runs/flight_001
```

The two full-stack profiles use GPU odometry by default. Do not run either ROS
launch unless the immediately preceding preflight reports `GPU_READY`; device
files or successful `libcuda.so` loading alone are not sufficient.

The scenario names accepted by `iap_sim.launch.py` are defined in
`config/scenarios/catalog.json`. Unknown names fail before any ROS process is
started.

The flight calibration manifest is an external retained deployment artifact;
the repository test fixture is never deployment authority. Its minimum schema
is:

```json
{
  "schema_version": "iap_local_surface_calibration_v1",
  "calibration_id": "vehicle_01_heldout_2026_09",
  "local_surface_error_bound_m": 0.04,
  "calibration_run_count": 3,
  "held_out_run_count": 1,
  "held_out_passed": true
}
```

## Launch contracts

### `glio.launch.py`

- Required process: `iap/iap_rosnode` named `glio`.
- Forbidden: integrity extension, planner, simulator, fake odometry, map
  generator, rosbag player, and truth adapter.
- Required input: configured IMU, LiDAR, GNSS range/ephemeris topics.
- Output: GLIO odometry/map/TF topics provided by `iap_rosnode`.
- Ready: estimator has initialized and publishes fresh odometry.
- Failure/HOLD: this profile has no planner; invalid or stale input must not be
  represented as navigation authorization.
- Config: `config/profiles/glio` by default.
- Results: `<output_dir>/{logs,export,dump,runtime_config}`.

### `glio_integrity.launch.py`

- Required process: `iap/iap_rosnode` named `glio_integrity`, loading GNSS and
  integrity extensions.
- Forbidden: planner, simulator, fake odometry, map generator, rosbag player.
- Required input: GLIO inputs plus integrity-required GNSS/LiDAR evidence.
- Output: GLIO outputs and `/iap/integrity`.
- Ready: fresh odometry and a fresh certified current-integrity report.
- HOLD: downstream motion must remain disabled while current integrity is
  missing, stale, invalid, or above its alert limit.
- Config: `config/profiles/glio_integrity` by default.
- Results: `<output_dir>/{logs,export,dump,runtime_config}`.

### `iap_sim.launch.py`

- Required processes: simulation environment, GLIO, Current Integrity Monitor,
  P0 Advisory Integrity, P4/P5 safety-aware planner, trajectory server.
- Forbidden: test validator, automatic bag recorder, and Phase-2 evaluator.
- Required input: generated IMU/LiDAR/GNSS plus selected scenario map.
- Output: integrity report, advisory risk grid/health, certified B-spline or a
  typed HOLD, position commands, and scenario/run manifests.
- Ready acceptance criterion: environment topics, estimator odometry, current
  integrity, P0 snapshot, registered local-map source health, and planner
  readiness are all fresh. The launch delay is only startup scheduling; it is
  not a topic-readiness gate. P0/P4/P5 remain fail-closed after that delay.
- HOLD: any applicable current-integrity, local-motion, support, braking,
  freshness, identity, P4, or P5 failure.
- Config: the maintained full-stack runtime materializes `config/sim_demo11`
  below the run directory. Scenario names come from
  `config/scenarios/catalog.json`; their exact established parameters are
  resolved by the private `_includes/full_stack_runtime.py` implementation.
- Results: everything is rooted below `output_dir`.

### `iap_flight.launch.py`

- Required processes: GLIO, Current Integrity Monitor, P0 Advisory Integrity,
  P4/P5 safety-aware planner, trajectory server.
- Forbidden: simulation packages, truth topics/adapters, fake odometry, random
  maps, rosbag playback/recording, auto-generated deployment calibration, and
  Phase-2 evaluator.
- Required input: explicitly configured vehicle IMU/LiDAR/GNSS, odometry,
  complete hardware ray/return evidence on `beam_evidence_topic`, controller
  feedback, calibration ID, alert-limit policy, and goal. The flight launch
  does not synthesize beam evidence; missing/incomplete evidence is HOLD.
- Output: certified executable trajectory or typed HOLD; no raw candidate may
  reach the controller.
- Ready acceptance criterion: explicit `flight_authorized:=true`, a retained
  calibration manifest with three calibration runs and a passing independent
  held-out run, fresh estimator/current-integrity/advisory/registered-map
  evidence, and a vehicle-side controller handshake. The launch verifies the
  manifest and requires `controller_handshake_confirmed:=true`; topic freshness
  remains a runtime P0/P4/P5 fail-closed responsibility.
- HOLD: launch refuses to start on a simulation extension or missing/default
  calibration; runtime failures follow the P4/P5 fail-closed contract.
- Config: `config/profiles/full_stack_flight` unless an equivalent deployment
  profile with GNSS, integrity, and planner-local-map extensions is supplied.
- Results: everything is rooted below `output_dir`.

## Internal and retained files

- `_includes/` contains private implementation included by canonical profiles.
  `simulation_environment.launch.py` must never start an IAP algorithm module.
  `full_stack_runtime.py` owns the maintained simulation graph.
- `tests/` is for focused launch tests and fixtures.
- `experiments/` is for paper/frozen experiment wrappers.
- `tools/` is for offline visualization and qualification helpers.
- `legacy/` is the eventual home for retained historical entrypoints.
- `bp/` is the temporary pre-refactor backup and the sole source location for
  those launch files; they must not also appear at the launch root. It remains
  installed during the transition so frozen scripts can resolve their old
  basenames, but none of them is canonical. Historical qualification profiles
  remain self-contained in `bp/test_planner.launch.py`; the maintained
  `_includes/full_stack_runtime.py` does not load code from `bp/`. Move or
  delete the backup only after every canonical profile has recorded
  integration/live evidence and all frozen references have been migrated.

Static or `--show-args` success is not real-flight acceptance. Removing `bp/`
requires recorded runtime evidence for all four canonical contracts.
