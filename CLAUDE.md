# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Repository Boundary (Hard Constraint)

- **Only modify files inside `src/iap/`**. Never touch `src/glim` or any other workspace package.
- Reading `src/glim` for reference is allowed. If you need GLIM functionality, copy/rewrite it into `src/iap` with a source attribution comment.

---

## Mandatory Traceability (Enforced by Git Hook)

Every code change **must**:

1. Reference at least one `IAP-RQ-XXX` requirement (see `docs/REQS.md` for the full list).
2. Update `docs/CHANGES.md` — what changed, why, which IAP-RQ-XXX.
3. Update `docs/TRACEABILITY.md` — requirement ↔ implementation file ↔ test/log mapping.

Commit messages must include one or more `IAP-RQ-XXX` identifiers. The pre-commit git hook will block commits that skip documentation updates.

---

## Build & Test

```bash
# Build (from workspace root /home/dev/code/ws_iap)
colcon build --symlink-install \
  --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Run all tests
colcon test --packages-select iap
colcon test-result --all

# Run a single GTest binary directly (after build)
./build/iap/test_araim
```

CPU-only build: edit `config/config.json` to replace `config_odometry_gpu.json` → `config_odometry_cpu.json`, `config_sub_mapping_gpu.json` → `config_sub_mapping_cpu.json`, `config_global_mapping_gpu.json` → `config_global_mapping_cpu.json`.

---

## Running the System

**Source order is critical** — IAP must precede GLIM so `dlopen()` picks IAP `.so` files first:

```bash
source /root/ros2_ws/install/setup.bash         # provides glim_rosbag / glim_rosnode
source /home/dev/code/ws_iap/install/setup.bash  # prepends IAP libs to LD_LIBRARY_PATH

# Offline mapping (recommended)
ros2 run glim_ros glim_rosbag \
  --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config \
  -- /home/dev/code/ws_iap/src/iap/data/realsense_ros2

# Real-time mapping (terminal 1)
ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config
# Real-time mapping (terminal 2)
ros2 bag play /home/dev/code/ws_iap/src/iap/data/realsense_ros2
```

Verify IAP loaded (not the GLIM defaults): `echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -E "iap|glim" | head -6` — IAP path must appear before `/root/ros2_ws`.

---

## Architecture

### Workspace Overlay Stack

```
/opt/ros/jazzy          ← base ROS2
       ↓
/root/ros2_ws           ← GLIM core library + glim_ros2 executables
       ↓
/home/dev/code/ws_iap   ← IAP (overrides and extends GLIM via LD_LIBRARY_PATH)
```

### Plugin Injection

GLIM is a plugin host. It calls `dlopen()` to load algorithm modules at startup. IAP provides replacement `.so` files (`libodometry_estimation_gpu.so`, `libsub_mapping.so`, `libglobal_mapping.so`, etc.) that shadow GLIM's defaults. Sourcing `ws_iap/install/setup.bash` prepends the IAP `lib/` directory so GLIM finds IAP versions first.

Extension modules (GNSS, trunk, integrity, rviz viewer) are listed in `config/config_ros.json` under `extension_modules` and are loaded separately at runtime.

### Factor Graph (GTSAM)

The estimator is a sliding-window factor graph using iSAM2. The state vector per keyframe includes: position `p`, velocity `v`, orientation `q`, IMU biases `b_a`/`b_g`, receiver clock bias `clk_bias`, and clock drift `clk_drift`. Sensor factors:

- **IMU**: pre-integration factor (GTSAM built-in)
- **LiDAR**: scan-to-map ICP residual; health monitoring inflates noise on degenerate scans
- **GNSS**: analytical-Jacobian pseudorange and Doppler factors in ECEF; satellite positions from broadcast ephemeris; per-satellite NIS gating
- **Trunk landmarks**: cylinder-model loop-closure factors

### Integrity (ARAIM)

`src/iap/integrity/` implements ARAIM (Autonomous Receiver Autonomous Integrity Monitoring):
- Hypothesis enumeration under single-fault assumptions
- Subset solutions for fault detection and exclusion
- Outputs Protection Level (PL), Alert Limit (AL), Integrity Margin (IM = AL − PL)
- Safety condition: PL < AL → NOMINAL mode; otherwise CAUTION/SEARCH

### Planner

Receding-horizon trajectory planner in `src/iap/planner/`. Cost function is dominated by `hinge(PL − AL)²`. The visibility predictor (`gnss/visibility_predictor.cpp`) forecasts satellite availability along candidate trajectories to predict future integrity.

### Key Module Locations

| Concern | Path |
|---------|------|
| Odometry estimation (GPU/CPU/CT) | `src/iap/odometry/` |
| GNSS factors & handler | `src/iap/gnss/` |
| ARAIM engine | `src/iap/integrity/` |
| Trunk detection & factor | `src/iap/trunk/` |
| Trajectory planner | `src/iap/planner/` |
| Sub-mapping / global mapping | `src/iap/mapping/` |
| Preprocessing | `src/iap/preprocess/` |
| Config/logging/module loading | `src/iap/util/` |
| 3D viewer | `src/iap/viewer/` |

### Configuration

Hierarchical JSON: `config/config.json` is the entry point specifying which per-module config files to load. Sensor calibration lives in `config_sensors.json`; GNSS parameters (PR noise, elevation cutoff, lever arm, ECEF reference) in `config_gnss.json`; integrity thresholds in `config_integrity.json`.

---

## Code Style

Google-style C++ with a **180-column limit** and **2-space indent** (`.clang-format` at repo root). Format with `clang-format -i <file>`. Clangd uses `build/iap/compile_commands.json` (configured in `.clangd`).

---

## Sensor Platform

Livox LiDAR (`/livox/lidar`, 10 Hz), Livox IMU (`/livox/imu`, 200 Hz), PX4/MAVROS IMU (`/mavros/imu/data`, 171 Hz), u-blox GNSS (`/ublox_driver/*`, 10 Hz). Dataset location: `src/iap/data/` (Shanghai area).
