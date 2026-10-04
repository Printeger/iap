# IAP launch entrypoints

Only the four files below are canonical user entrypoints. New demos must not be
added at the launch root; validation and paper-specific wrappers belong in the
directories documented below.

| Entrypoint | Runtime composition | Environment |
|---|---|---|
| `glio.launch.py` | GLIO + visualization publisher; RViz by default | none; live and rosbag use the same topics |
| `glio_integrity.launch.py` | GLIO + Current Integrity Monitor + visualization publisher; RViz by default | none |
| `iap_sim.launch.py` | all four IAP modules | selected simulation scenario |
| `iap_flight.launch.py` | all four IAP modules | real vehicle IO only |

Every entrypoint automatically allocates one collision-safe
`<IAP_RUN_ROOT>/<UTC timestamp>/` shared by all modules. In a source or
`--symlink-install` workspace the default root is `src/iap/log`; entrypoint and
scenario identity live in `metadata/run_manifest.json`, not in extra directory
levels. The root maintains one concurrency-safe `latest` symlink. Set
`IAP_RUN_ROOT` once to change the base. The former `output_dir` launch argument
has been removed. The deprecated `phase2_planner_integrity_evaluator` is
forbidden from all four graphs.

## Commands

```bash
# GLIO with live input, or with a rosbag played separately on the same topics
ros2 launch iap glio.launch.py

# Headless/remote use
ros2 launch iap glio.launch.py start_rviz:=false

# GLIO + current integrity
ros2 launch iap glio_integrity.launch.py

# Headless/remote integrity monitoring
ros2 launch iap glio_integrity.launch.py start_rviz:=false

# Full simulation
python3 src/iap/scripts/dev_planner/run_gate0_qualification.py \
  --output-root /tmp/iap_runs/sim_001/preflight \
  --gpu-preflight-only
ros2 launch iap iap_sim.launch.py \
  scenario:=fused_nominal

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
  local_surface_error_calibration_manifest:=/data/iap/calibration/vehicle_01.json
```

For a deployed vehicle, configure its persistent root once in the service
environment rather than changing every command:

```bash
export IAP_RUN_ROOT=/data/iap_runs
```

Automatic retention is enabled only for the repository-local default root. To
enable it for an explicitly configured root, opt in after reviewing that root:

```bash
export IAP_RETENTION_ENABLED=1
```

Retention keeps the newest three finalized runs and all runs from the last
seven days. It ignores active, locked, protected, formal, malformed, and
symlinked directories; the complete policy is in
`docs/spec/run_artifact_contract.md`.

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
- Output: `/glio/odom`, `/glio/map`, `/glio/aligned_points`, and TF from the
  GLIO RViz extension. RViz starts by default with the maintained GLIO config;
  pass `start_rviz:=false` for headless use.
- Ready: estimator has initialized and publishes fresh odometry.
- Failure/HOLD: this profile has no planner; invalid or stale input must not be
  represented as navigation authorization.
- Config: `config/profiles/glio` by default.
- Results: the shared `<run_dir>/{runtime,profiling,export,metadata}` contract.

### `glio_integrity.launch.py`

- Required process: `iap/iap_rosnode` named `glio_integrity`, loading GNSS,
  integrity, and visualization-publisher extensions.
- Forbidden: planner, simulator, fake odometry, map generator, rosbag player.
- Required input: GLIO inputs plus integrity-required GNSS/LiDAR evidence.
- Output: `/glio_integrity/odom`, `/glio_integrity/map`,
  `/glio_integrity/aligned_points`, TF, `/iap/integrity`, and the current
  protection-level envelopes on `/iap/araim_envelopes`. RViz starts by default
  with the maintained integrity config; pass `start_rviz:=false` for headless
  use.
- Ready: fresh odometry and a fresh certified current-integrity report.
- HOLD: downstream motion must remain disabled while current integrity is
  missing, stale, invalid, or above its alert limit.
- Config: `config/profiles/glio_integrity` by default.
- Results: the shared `<run_dir>/{runtime,profiling,export,metadata}` contract.

### `iap_sim.launch.py`

- Starts the existing simulator and GLIO/current-integrity inputs, the restored EGO planner, and traj_server.
- The manager binds PredictorModule to the same GridMap. Spatial HPL/VPL is cached independently of physical occupancy.
- Stage 1 retains EGO physical route selection. Risk-guided search and full integrity trajectory checks are later stages.
- Scenarios are selected from `config/scenarios/catalog.json`; the maintained graph is `_includes/full_stack_runtime.py`. All outputs use one canonical run directory.
- The four-fork scene uses this same stage-1 graph. Old P0/P4/P5 and continuous-flight validator settings are retired.

### `iap_flight.launch.py`

This entrypoint refuses to start during the rebuild. Stages 4/5 must implement the actual trajectory check, execution handoff and stopping behavior before vehicle calibration/handshake can be revalidated. GLIO and current-integrity launches remain available independently.

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
