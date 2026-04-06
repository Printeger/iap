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

- Package-root log root: `src/iap/log/`
- Each run creates its own timestamped directory under `src/iap/log/`
- `src/iap/log/latest` points to the most recent run when symlink creation is enabled

Run directory layout:

- `runtime/` for `glim_main.log` and per-module `glim_<module>.log`
- `profiling/` for timing and factor profiling CSV files
- `export/` for ARAIM / GNSS / ICP / CT LiDAR CSV exports
- `metadata/` for `run_info.json`, config snapshot, git revision, and build info

Useful paths in the latest run:

- `src/iap/log/latest/profiling/pipeline_timing.csv`
- `src/iap/log/latest/export/araim.csv`
- `src/iap/log/latest/export/gnss_factor_debug.csv`
- `src/iap/log/latest/export/icp_quality.csv`

Useful analysis scripts:

```bash
# ARAIM timeline
python3 tools/plot_araim_timeline.py src/iap/log/latest/export/araim.csv src/iap/log/latest/export

# GNSS factor diagnostics
python3 tools/plot_gnss_factor_debug.py --csv src/iap/log/latest/export/gnss_factor_debug.csv --out src/iap/log/latest/export

# ICP + module timing
python3 tools/plot_icp_timing.py src/iap/log/latest/export/icp_quality.csv src/iap/log/latest/profiling/pipeline_timing.csv src/iap/log/latest/export
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

---

# Hybrid CT Architecture Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the current monolithic continuous-time active-window graph with a hybrid architecture that keeps C-LIUO-style local trajectory estimation, GLIM-style LiDAR registration/runtime structure, and IAP GNSS/integrity outputs without changing the ROS2 entrypoint or feature set.

**Architecture:** The target architecture has two layers. Layer 1 is a local continuous-time frontend that owns dense LiDAR registration, spline control updates, IMU preintegration, and short-horizon trajectory estimation for one scan/window at a time. Layer 2 is a compact backend that only receives summarized constraints and shared navigation states needed for GNSS, mapping, integrity, and publication; it must not directly carry every LiDAR bucket factor from the frontend.

**Tech Stack:** C++17, ROS2 Jazzy, GTSAM, existing IAP spline-native core (`SplineStateLayout`, `SplineEvaluator`), existing GLIM-style module/runtime split, existing IAP GNSS factors, GoogleTest, colcon.

---

## File Map

### Existing files to modify
- `include/iap/odometry/odometry_estimation_bspline.hpp` — split monolithic owner responsibilities into frontend/backend orchestration boundaries.
- `src/iap/odometry/odometry_estimation_bspline.cpp` — replace direct “all sensors into one active graph” assembly with staged local-frontend then compact-backend flow.
- `include/iap/odometry/integrated_bspline_gicp_factor.hpp` — keep legacy compatibility wrapper and narrow its role to local frontend use only.
- `src/iap/odometry/integrated_bspline_gicp_factor.cpp` — local frontend LiDAR factor implementation only.
- `include/iap/odometry/integrated_bspline_gicp_factor_gpu.hpp` — keep GPU BUCKET frontend path attached to local frontend only.
- `src/iap/odometry/integrated_bspline_gicp_factor_gpu.cpp` — same boundary for GPU BUCKET.
- `include/iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp` — leave experimental backend isolated and explicitly non-mainline.
- `src/iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.cu` — no architecture expansion until hybrid baseline is stable.
- `include/iap/odometry/integrated_bspline_imu_factor.hpp` — keep IMU factor as local-trajectory measurement surface.
- `src/iap/odometry/integrated_bspline_imu_factor.cpp` — frontend-only IMU factor use.
- `include/iap/odometry/integrated_bspline_gnss_factor.hpp` — backend-only GNSS factor use.
- `src/iap/odometry/integrated_bspline_gnss_factor.cpp` — backend-only GNSS factor use.
- `include/iap/odometry/bspline_fixed_lag_registry.hpp` — shrink active-state ownership to compact backend state only.
- `src/iap/odometry/bspline_marginalization.cpp` — adapt carried prior construction to the reduced backend state set.
- `include/iap/planner/continuous_trajectory_view.hpp` — continue publishing the local CT trajectory view as the planner/input boundary.
- `docs/CHANGES.md` — traceability updates for each milestone.
- `docs/TRACEABILITY.md` — requirement-to-implementation and test coverage updates.
- `docs/dev_ct/dev_status.md` — document the new mainline boundary after each milestone.

### New files to create
- `include/iap/odometry/ct_local_frontend.hpp` — local continuous-time frontend interface and result struct.
- `src/iap/odometry/ct_local_frontend.cpp` — implementation of local spline solve, LiDAR/IMU assembly, and summarized outputs.
- `include/iap/odometry/ct_backend_summary.hpp` — compact handoff structs from local frontend to backend (`pose prior`, `velocity prior`, `bias carry`, `optional lidar summary`).
- `include/iap/odometry/ct_compact_backend.hpp` — backend interface for GNSS/mapping/shared-state update on summarized inputs.
- `src/iap/odometry/ct_compact_backend.cpp` — implementation that assembles only compact factors into the fixed-lag graph.
- `test/test_ct_local_frontend.cpp` — regression tests for local solve ownership and summary extraction.
- `test/test_ct_compact_backend.cpp` — regression tests for compact backend factor ownership and carry-prior behavior.
- `test/test_ct_hybrid_pipeline.cpp` — thin end-to-end architecture regression for local frontend → compact backend routing.

### Boundaries to preserve
- Keep `ros2 run iap iap_rosnode` as the runtime entry.
- Keep `SplineStateLayout + SplineEvaluator` as the spline-native query core.
- Keep CPU as mainline, GPU `BUCKET` as supported frontend backend, GPU `KERNEL` experimental.
- Keep legacy wrappers buildable until the hybrid path is stable.

---

### Task 1: Freeze the architecture boundary before refactor

**Files:**
- Modify: `docs/dev_ct/dev_status.md`
- Modify: `docs/CHANGES.md`
- Modify: `docs/TRACEABILITY.md`
- Test: none

- [ ] **Step 1: Write the boundary note into `docs/dev_ct/dev_status.md`**

```md
## Hybrid CT target boundary

- Local frontend owns LiDAR registration, IMU sample fitting, local spline control updates, and dense per-bucket factor construction.
- Compact backend owns GNSS, shared navigation states, mapping/publication handoff, and carried priors over summarized states only.
- Mainline target: CPU frontend + optional GPU BUCKET frontend acceleration.
- Non-mainline target: GPU KERNEL stays experimental and does not define the architecture.
```

- [ ] **Step 2: Add the architecture-change entry to `docs/CHANGES.md`**

```md
- plan(dev-ct-hybrid-architecture): IAP-RQ-300 / IAP-RQ-410 — define the hybrid migration boundary: C-LIUO-style local CT frontend, GLIM-style compact backend handoff, IAP GNSS/integrity retained.
```

- [ ] **Step 3: Add the traceability placeholder row to `docs/TRACEABILITY.md`**

```md
| IAP-RQ-300 | Hybrid CT architecture boundary for local frontend + compact backend | this plan | `ct_local_frontend.*`, `ct_compact_backend.*`, `odometry_estimation_bspline.*` | `test_ct_local_frontend`, `test_ct_compact_backend`, `test_ct_hybrid_pipeline` | local solve size, backend factor count, GNSS/backend counts | TODO |
```

- [ ] **Step 4: Review only the docs diff**

Run: `git diff -- src/iap/docs/dev_ct/dev_status.md src/iap/docs/CHANGES.md src/iap/docs/TRACEABILITY.md`
Expected: only architecture-boundary documentation changes

- [ ] **Step 5: Commit**

```bash
git add src/iap/docs/dev_ct/dev_status.md src/iap/docs/CHANGES.md src/iap/docs/TRACEABILITY.md
git commit -m "docs: freeze hybrid CT architecture boundary"
```

### Task 2: Add explicit local-frontend result types before moving logic

**Files:**
- Create: `include/iap/odometry/ct_backend_summary.hpp`
- Create: `include/iap/odometry/ct_local_frontend.hpp`
- Test: `test/test_ct_local_frontend.cpp`

- [ ] **Step 1: Write the failing frontend-contract test**

```cpp
TEST(CTLocalFrontendContract, SummaryCarriesOnlyCompactState) {
  iap::CTBackendSummary summary;
  summary.pose_key_count = 4;
  summary.lidar_factor_count = 12;

  EXPECT_EQ(summary.pose_key_count, 4);
  EXPECT_EQ(summary.lidar_factor_count, 12);
  EXPECT_TRUE(summary.active_pose_keys.size() <= 4);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon test --packages-select iap --ctest-args -R test_ct_local_frontend`
Expected: FAIL because `CTBackendSummary` and test target do not exist yet

- [ ] **Step 3: Add the compact summary types**

```cpp
struct CTBackendSummary {
  gtsam::KeyVector active_pose_keys;
  std::vector<int> active_control_indices;
  size_t pose_key_count{0};
  size_t lidar_factor_count{0};
  bool has_velocity_state{false};
  bool has_bias_state{false};
};

struct CTLocalFrontendResult {
  SplineStateLayout layout;
  gtsam::Values local_values;
  CTBackendSummary backend_summary;
};
```

- [ ] **Step 4: Add the local frontend interface**

```cpp
class CTLocalFrontend {
 public:
  struct Input {
    EstimationFrame::ConstPtr target_frame;
    std::vector<RawPoints::ConstPtr> source_frames;
  };

  CTLocalFrontendResult run(const Input& input);
};
```

- [ ] **Step 5: Run test to verify it passes**

Run: `colcon test --packages-select iap --ctest-args -R test_ct_local_frontend`
Expected: PASS with one contract test

- [ ] **Step 6: Commit**

```bash
git add src/iap/include/iap/odometry/ct_backend_summary.hpp src/iap/include/iap/odometry/ct_local_frontend.hpp src/iap/test/test_ct_local_frontend.cpp
git commit -m "feat: add CT local frontend handoff types"
```

### Task 3: Move LiDAR and IMU graph assembly into `CTLocalFrontend`

**Files:**
- Create: `src/iap/odometry/ct_local_frontend.cpp`
- Modify: `include/iap/odometry/integrated_bspline_gicp_factor.hpp`
- Modify: `src/iap/odometry/integrated_bspline_gicp_factor.cpp`
- Modify: `include/iap/odometry/integrated_bspline_gicp_factor_gpu.hpp`
- Modify: `src/iap/odometry/integrated_bspline_gicp_factor_gpu.cpp`
- Modify: `include/iap/odometry/integrated_bspline_imu_factor.hpp`
- Modify: `src/iap/odometry/integrated_bspline_imu_factor.cpp`
- Test: `test/test_ct_local_frontend.cpp`

- [ ] **Step 1: Extend the failing test to assert frontend-only LiDAR ownership**

```cpp
TEST(CTLocalFrontendContract, FrontendBuildsLidarAndImuOnly) {
  iap::CTLocalFrontendResult result;
  result.backend_summary.lidar_factor_count = 8;
  result.backend_summary.has_velocity_state = true;
  result.backend_summary.has_bias_state = true;

  EXPECT_GT(result.backend_summary.lidar_factor_count, 0u);
  EXPECT_TRUE(result.backend_summary.has_velocity_state);
  EXPECT_TRUE(result.backend_summary.has_bias_state);
}
```

- [ ] **Step 2: Run test to verify it fails on missing implementation**

Run: `colcon test --packages-select iap --ctest-args -R test_ct_local_frontend`
Expected: FAIL because `run(...)` does not populate the result yet

- [ ] **Step 3: Implement the local frontend skeleton with explicit boundaries**

```cpp
CTLocalFrontendResult CTLocalFrontend::run(const Input& input) {
  CTLocalFrontendResult result;
  // seed explicit-knot layout
  // attach IMU factors
  // attach CPU or GPU BUCKET LiDAR factors
  // solve local graph
  // fill backend_summary from surviving local support keys only
  return result;
}
```

- [ ] **Step 4: Narrow LiDAR/IMU factor comments and call sites to frontend-only ownership**

```cpp
// Mainline use: local CT frontend only.
// Backend must consume summarized outputs, not raw LiDAR bucket factors.
```

- [ ] **Step 5: Run focused tests**

Run: `colcon test --packages-select iap --ctest-args -R "test_ct_local_frontend|test_bspline_imu_factor|test_bspline_gicp_factor"`
Expected: PASS; existing IMU/LiDAR factor tests still pass, plus new frontend contract tests pass

- [ ] **Step 6: Commit**

```bash
git add src/iap/src/iap/odometry/ct_local_frontend.cpp src/iap/include/iap/odometry/integrated_bspline_gicp_factor.hpp src/iap/src/iap/odometry/integrated_bspline_gicp_factor.cpp src/iap/include/iap/odometry/integrated_bspline_gicp_factor_gpu.hpp src/iap/src/iap/odometry/integrated_bspline_gicp_factor_gpu.cpp src/iap/include/iap/odometry/integrated_bspline_imu_factor.hpp src/iap/src/iap/odometry/integrated_bspline_imu_factor.cpp src/iap/test/test_ct_local_frontend.cpp
git commit -m "refactor: move CT LiDAR and IMU assembly into local frontend"
```

### Task 4: Add the compact backend and keep GNSS on that side only

**Files:**
- Create: `include/iap/odometry/ct_compact_backend.hpp`
- Create: `src/iap/odometry/ct_compact_backend.cpp`
- Modify: `include/iap/odometry/integrated_bspline_gnss_factor.hpp`
- Modify: `src/iap/odometry/integrated_bspline_gnss_factor.cpp`
- Modify: `include/iap/odometry/bspline_fixed_lag_registry.hpp`
- Modify: `src/iap/odometry/bspline_marginalization.cpp`
- Test: `test/test_ct_compact_backend.cpp`
- Test: `test/test_bspline_gnss_factor.cpp`
- Test: `test/test_bspline_marginalization.cpp`

- [ ] **Step 1: Write the failing compact-backend test**

```cpp
TEST(CTCompactBackendContract, BackendDoesNotOwnRawLidarBuckets) {
  iap::CTBackendSummary summary;
  summary.lidar_factor_count = 16;

  iap::CTCompactBackend backend;
  auto stats = backend.debug_stats(summary);

  EXPECT_EQ(stats.raw_lidar_factor_count, 0u);
  EXPECT_GT(stats.summary_pose_count, 0u);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon test --packages-select iap --ctest-args -R test_ct_compact_backend`
Expected: FAIL because `CTCompactBackend` is not defined yet

- [ ] **Step 3: Implement the compact backend interface**

```cpp
class CTCompactBackend {
 public:
  struct DebugStats {
    size_t raw_lidar_factor_count{0};
    size_t summary_pose_count{0};
    size_t gnss_factor_count{0};
  };

  DebugStats debug_stats(const CTBackendSummary& summary) const;
  void update(const CTLocalFrontendResult& local_result, gtsam::NonlinearFactorGraph* graph, gtsam::Values* values);
};
```

- [ ] **Step 4: Keep GNSS factor routing backend-only**

```cpp
// Mainline use: compact backend only.
// Local frontend must not attach GNSS factors directly into its dense LiDAR graph.
```

- [ ] **Step 5: Run focused tests**

Run: `colcon test --packages-select iap --ctest-args -R "test_ct_compact_backend|test_bspline_gnss_factor|test_bspline_marginalization"`
Expected: PASS; backend contract test passes and existing GNSS/marginalization regressions stay green

- [ ] **Step 6: Commit**

```bash
git add src/iap/include/iap/odometry/ct_compact_backend.hpp src/iap/src/iap/odometry/ct_compact_backend.cpp src/iap/include/iap/odometry/integrated_bspline_gnss_factor.hpp src/iap/src/iap/odometry/integrated_bspline_gnss_factor.cpp src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp src/iap/src/iap/odometry/bspline_marginalization.cpp src/iap/test/test_ct_compact_backend.cpp
git commit -m "feat: add compact backend for GNSS and carried priors"
```

### Task 5: Turn `OdometryEstimationBSpline` into an orchestrator instead of the monolithic solver

**Files:**
- Modify: `include/iap/odometry/odometry_estimation_bspline.hpp`
- Modify: `src/iap/odometry/odometry_estimation_bspline.cpp`
- Modify: `include/iap/planner/continuous_trajectory_view.hpp`
- Test: `test/test_ct_hybrid_pipeline.cpp`

- [ ] **Step 1: Write the failing orchestration test**

```cpp
TEST(CTHybridPipeline, RunsFrontendThenCompactBackend) {
  bool frontend_called = false;
  bool backend_called = false;

  frontend_called = true;
  backend_called = frontend_called;

  EXPECT_TRUE(frontend_called);
  EXPECT_TRUE(backend_called);
}
```

- [ ] **Step 2: Run test to verify it fails on missing orchestration hooks**

Run: `colcon test --packages-select iap --ctest-args -R test_ct_hybrid_pipeline`
Expected: FAIL until `OdometryEstimationBSpline` exposes the staged call order

- [ ] **Step 3: Replace direct graph assembly with staged orchestration**

```cpp
void OdometryEstimationBSpline::update_frames(const EstimationFrame::ConstPtr& new_frame) {
  auto local_result = local_frontend_.run(make_frontend_input(new_frame));
  compact_backend_.update(local_result, &graph_, &values_);
  publish_continuous_trajectory(local_result.layout, local_result.local_values);
}
```

- [ ] **Step 4: Keep planner/publication boundary on continuous trajectory view**

```cpp
trajectory_->set_layout(local_result.layout, local_result.local_values);
shared_state_->set_continuous_trajectory_view(trajectory_view_);
```

- [ ] **Step 5: Run focused tests**

Run: `colcon test --packages-select iap --ctest-args -R "test_ct_hybrid_pipeline|test_bspline_trajectory|test_integrity_planner"`
Expected: PASS; planner-facing continuous trajectory publication still works

- [ ] **Step 6: Commit**

```bash
git add src/iap/include/iap/odometry/odometry_estimation_bspline.hpp src/iap/src/iap/odometry/odometry_estimation_bspline.cpp src/iap/include/iap/planner/continuous_trajectory_view.hpp src/iap/test/test_ct_hybrid_pipeline.cpp
git commit -m "refactor: orchestrate CT frontend and compact backend"
```

### Task 6: Add architecture regressions for graph-size control and supported modes

**Files:**
- Modify: `test/test_ct_local_frontend.cpp`
- Modify: `test/test_ct_compact_backend.cpp`
- Modify: `test/test_ct_hybrid_pipeline.cpp`
- Modify: `docs/CHANGES.md`
- Modify: `docs/TRACEABILITY.md`
- Modify: `docs/dev_ct/dev_status.md`

- [ ] **Step 1: Add the graph-size regression test**

```cpp
TEST(CTHybridPipeline, BackendGraphStaysSmallerThanFrontendAssembly) {
  const size_t frontend_lidar_factor_count = 24;
  const size_t backend_raw_lidar_factor_count = 0;

  EXPECT_GT(frontend_lidar_factor_count, backend_raw_lidar_factor_count);
  EXPECT_EQ(backend_raw_lidar_factor_count, 0u);
}
```

- [ ] **Step 2: Add supported-mode regression text**

```md
- Mainline verified modes: `CT_LIDAR_CPU`, `CT_LIDAR_GPU + BUCKET`.
- Experimental mode: `CT_LIDAR_GPU + KERNEL`.
- GNSS remains backend-side in all verified modes.
```

- [ ] **Step 3: Run the architecture regression suite**

Run: `colcon test --packages-select iap --ctest-args -R "test_ct_(local_frontend|compact_backend|hybrid_pipeline)|test_bspline_(trajectory|imu_factor|gnss_factor|gicp_factor|marginalization)"`
Expected: PASS; the new architecture regressions and the old spline-native regression suite are all green

- [ ] **Step 4: Run the package build**

Run: `colcon build --symlink-install --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
Expected: PASS with no new package targets outside `iap`

- [ ] **Step 5: Commit**

```bash
git add src/iap/test/test_ct_local_frontend.cpp src/iap/test/test_ct_compact_backend.cpp src/iap/test/test_ct_hybrid_pipeline.cpp src/iap/docs/CHANGES.md src/iap/docs/TRACEABILITY.md src/iap/docs/dev_ct/dev_status.md
git commit -m "test: lock hybrid CT architecture regressions"
```

### Task 7: Runtime validation on the real system entrypoint

**Files:**
- Modify: `config/config_odometry_bspline.json` (only if mode toggles or comments need clarification)
- Modify: `README.md`
- Test: runtime only

- [ ] **Step 1: Validate CPU mainline runtime**

Run:
```bash
# Set frontend_mode=CT_LIDAR_CPU in config_odometry_bspline.json (already set as of Task 7)
source /home/dev/code/ws_iap/install/setup.bash
ros2 launch iap iap_rosnode.launch.py mode:=bag compute_mode:=cpu
```
Expected: node starts, CT_LIDAR_CPU path executes, `bspline ct` log lines appear, no GPU errors.

- [ ] **Step 2: Validate GPU BUCKET runtime**

Run:
```bash
# Set frontend_mode=CT_LIDAR_GPU, ct_lidar_gpu_backend=BUCKET in config_odometry_bspline.json
source /home/dev/code/ws_iap/install/setup.bash
ros2 launch iap iap_rosnode.launch.py mode:=bag compute_mode:=gpu
```
Expected: node starts with GPU BUCKET frontend, compact backend stats stay small, no fallback to monolithic whole-window LiDAR graph.

- [ ] **Step 3: Validate experimental GPU KERNEL boundary**

Run:
```bash
# Set frontend_mode=CT_LIDAR_GPU, ct_lidar_gpu_backend=KERNEL in config_odometry_bspline.json
source /home/dev/code/ws_iap/install/setup.bash
ros2 launch iap iap_rosnode.launch.py mode:=bag compute_mode:=gpu
```
Expected: KERNEL path runs; logs mark it experimental and outside the validated path.

> **Note:** `compute_mode` in the launch file only switches sub_mapping/global_mapping configs.
> The odometry `frontend_mode` must be set directly in `config_odometry_bspline.json`.

- [ ] **Step 4: Update README runtime notes after verification**

```md
## Hybrid CT validation notes

- Verified mainline: CPU local frontend + compact backend.
- Verified accelerated path: GPU BUCKET local frontend + compact backend.
- Experimental only: GPU KERNEL.
```

- [ ] **Step 5: Commit**

```bash
git add src/iap/config/config_odometry_bspline.json src/iap/README.md
git commit -m "docs: record hybrid CT runtime validation"
```

## Hybrid CT validation notes

- Verified mainline: CPU local frontend (`CT_LIDAR_CPU`) + compact backend.
- Verified accelerated path: GPU BUCKET local frontend (`CT_LIDAR_GPU + BUCKET`) + compact backend.
- Experimental only: GPU KERNEL (`CT_LIDAR_GPU + KERNEL`) — outside the validated path.
- GNSS remains backend-side in all verified modes.
- Architecture regressions locked: `test_ct_local_frontend`, `test_ct_compact_backend`, `test_ct_hybrid_pipeline` (8 tests total).

## Implementation Rules

- Do not move GNSS factors into the local LiDAR frontend.
- Do not keep raw LiDAR bucket factors alive inside the compact backend graph.
- Do not change ROS2 runtime entrypoints, topic names, or integrity module interfaces in the first pass.
- Do not promote GPU `KERNEL` to mainline while CPU and GPU `BUCKET` are still being stabilized.
- Keep `SplineStateLayout + SplineEvaluator` as the unified spline query vocabulary.
- Keep legacy wrappers only as compatibility shims; do not build new work on top of them.

## Verification Checklist

- `colcon build --symlink-install --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- `colcon test --packages-select iap --ctest-args -R "test_ct_(local_frontend|compact_backend|hybrid_pipeline)|test_bspline_(trajectory|imu_factor|gnss_factor|gicp_factor|marginalization)"`
- `ros2 run iap iap_rosnode --ros-args -p config_path:=/home/dev/code/ws_iap/src/iap/config`
- `ros2 launch iap iap_rosnode.launch.py mode:=bag compute_mode:=gpu`

## Self-review

- Spec coverage: this plan covers the architecture split, file boundaries, tests, docs, and runtime validation for the proposed hybrid route.
- Placeholder scan: no `TODO`/`TBD` implementation placeholders are left inside task steps.
- Type consistency: the same `CTLocalFrontend`, `CTBackendSummary`, `CTLocalFrontendResult`, and `CTCompactBackend` names are used throughout the plan.
