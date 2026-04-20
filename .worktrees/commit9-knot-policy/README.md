# IAP — Integrity-Aware Positioning

A 3D LiDAR-IMU(-GNSS) localization/mapping system built on the GLIM architecture, with ARAIM integrity modules and a ROS2 runtime entry.

---

## 1) Recommended Runtime (aligned with current code)

Use `iap_rosnode` as the primary runtime entry. `glim_rosnode` is not required for the default workflow in this repository.

### 构建

```bash
cd /home/dev/code/ws_iap

colcon build --symlink-install \
  --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Run (two terminals)

Terminal 1: start IAP node

```bash
cd /home/dev/code/ws_iap
source install/setup.bash

ros2 run iap iap_rosnode \
  --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config
```

Terminal 2: play bag

```bash
cd /home/dev/code/ws_iap
source install/setup.bash

ros2 bag play /home/dev/code/ws_iap/src/iap/data/realsense_ros2
```

---

## 2) Runtime Mode Switches

### A. Default mode (GNSS + LiDAR + IMU)

By default, these modules are loaded from `extension_modules` in `config/config_ros.json`:

- `libgnss_extension.so`
- `libtrunk_extension.so`
- `libintegrity_extension.so`
- `libstandard_viewer.so`
- `librviz_viewer.so`

Also, `clock_owner_mode` defaults to `gnss` (see `config_odometry_{gpu,cpu,ct}.json`).

### B. LiDAR + IMU only (disable GNSS)

Edit `config/config_ros.json` and remove from `extension_modules`:

- `libgnss_extension.so`
- `libintegrity_extension.so` is also recommended to remove (it depends on GNSS epoch data)

You can keep `trunk/viewer` modules if needed.

### C. Odometry-only mode (disable local/global mapping)

Edit `config/config_ros.json`:

```json
"enable_local_mapping": false,
"enable_global_mapping": false
```

This runs the odometry path only.

---

## 3) Key Configuration Files

| File | Purpose |
|---|---|
| `config/config.json` | Top-level entry: points to sub-config files; can enable timing CSV |
| `config/config_ros.json` | ROS topics/QoS/extension loading and local/global mapping toggles |
| `config/config_odometry_gpu.json` | GPU odometry parameters (including `clock_owner_mode`) |
| `config/config_odometry_cpu.json` | CPU odometry parameters (including `clock_owner_mode`) |
| `config/config_odometry_ct.json` | Continuous-time odometry parameters (including `clock_owner_mode`) |
| `config/config_gnss.json` | GNSS and integrity-related parameters |
| `config/config_global_mapping_gpu.json` | Global mapping parameters |

---

## 4) Logs and Analysis

Default log directories:

- `src/iap/log/`
- Common result folder: `src/iap/log/res/`

If `global.enable_timing_csv=true` in `config/config.json`, it writes:

- `src/iap/log/res/iap_timing.csv`

Useful analysis scripts:

```bash
# ARAIM timeline
python3 tools/plot_araim_timeline.py src/iap/log/res/iap_araim.csv src/iap/log/res

# GNSS factor diagnostics
python3 tools/plot_gnss_factor_debug.py --csv src/iap/log/res/iap_gnss_factor_debug.csv --out src/iap/log/res

# ICP + module timing
python3 tools/plot_icp_timing.py src/iap/log/res/iap_icp.csv src/iap/log/res/iap_timing.csv src/iap/log/res
```

---

## 5) Troubleshooting

### `ros2 run iap iap_rosnode` fails to start

- Confirm `source /home/dev/code/ws_iap/install/setup.bash` is executed.
- Confirm `colcon build --packages-select iap` succeeded.

### `/rtcm` warning while playing bag

If `rtcm_msgs` is not installed, this warning may appear. It is usually non-fatal for the current replay workflow.

### No GUI window

If there is no DISPLAY environment, comment out `libstandard_viewer.so` in `config_ros.json`. Keep `librviz_viewer.so` or run with logs only.

---

## 6) Developer Notes

- After editing config files, restart `iap_rosnode` before replaying the bag.
- For A/B runs (for example, `dual` vs `gnss`), use copied config directories to avoid cross-run contamination.
- Before commit, update `docs/CHANGES.md` and `docs/TRACEABILITY.md` (enforced by repository hooks).
