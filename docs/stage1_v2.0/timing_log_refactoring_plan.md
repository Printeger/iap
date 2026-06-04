# 耗时分析Log系统重构计划

## 1. Context

当前 IAP 系统的耗时分析基于 `timing_csv` 基础设施（`include/iap/util/timing_csv.hpp`），通过 `ScopedTimer` (RAII) 和 `append()` 两种方式将耗时数据写入 `profiling/iap_timing.csv`。现有 16 个 timing module 使用扁平化命名（如 `araim`, `odom_deskew`），缺乏层次化的模块分类，导致分析报告难以按子系统聚合。

本次重构目标：将耗时 log 按五大模块（定位、ARAIM、Advisory PL Predict、IAP Planning、EGO Planner）进行层次化组织，直接替换旧名称为新命名规范，补充缺失的子模块耗时测量，为性能瓶颈定位提供结构化数据支撑。

## 2. 现有耗时Log系统现状

### 2.1 基础设施

| 组件 | 文件 | 说明 |
|------|------|------|
| `timing_csv::ScopedTimer` | `include/iap/util/timing_csv.hpp:67-95` | RAII 作用域计时器 |
| `timing_csv::append()` | `include/iap/util/timing_csv.hpp:56-65` | 手动写入 CSV 行 |
| `timing_csv::enabled()` | `include/iap/util/timing_csv.hpp:19-27` | 由 `enable_timing_csv` 配置控制 |
| CSV 输出 | `<run_dir>/profiling/iap_timing.csv` | 列: `stamp,module,elapsed_ms` |
| `RunLogManager` | `include/iap/util/run_log_manager.hpp` | 管理 run 目录和 profiling 路径 |

### 2.2 现有16个Timing Module（扁平命名 → 待替换）

| 旧名称 | 位置 | 测量内容 |
|--------|------|----------|
| `ros_pointcloud_callback_total` | `apps/iap_rosnode.cpp:342` | ROS 点云回调总耗时 |
| `pointcloud_extract_raw` | `apps/iap_rosnode.cpp:359` | 原始点云提取 |
| `cloud_preprocess_total` | `apps/iap_rosnode.cpp:375` | 点云预处理 |
| `odom_insert_frame_total` | `src/iap/odometry/odometry_estimation_imu.cpp:170` | 帧插入总耗时 |
| `odom_imu_integration` | `src/iap/odometry/odometry_estimation_imu.cpp:319` | IMU 预积分 |
| `odom_deskew` | `src/iap/odometry/odometry_estimation_imu.cpp:423` | 点云去畸变 |
| `odom_covariance_estimation` | `src/iap/odometry/odometry_estimation_imu.cpp:231,433` | 协方差估计 |
| `odom_create_factors` | `src/iap/odometry/odometry_estimation_imu.cpp:287,451` | 因子创建 |
| `odom_smoother_update` | `src/iap/odometry/odometry_estimation_imu.cpp:292,458` | iSAM2 优化更新 |
| `odom_update_frames` | `src/iap/odometry/odometry_estimation_imu.cpp:296,479` | 帧状态更新 |
| `async_odom_queue_wait` | `src/iap/odometry/async_odometry_estimation.cpp:119` | 异步队列等待 |
| `gnss_injection` | `src/iap/gnss/gnss_extension.cpp:1015` | GNSS 观测注入 |
| `araim` | `src/iap/integrity/araim.cpp:511` | GNSS ARAIM 计算 |
| `integrity` | `src/iap/integrity/integrity_monitor.cpp:493` | 完整性监控总耗时 |
| `trunk_detector` | `src/iap/trunk/trunk_detector.cpp:213` | 树干检测 |
| `pl_grid_build` | `src/iap/planner/future_pl_field_predictor.cpp:226` | PL 网格构建 |

### 2.3 分析工具链

| 工具 | 文件 | 功能 |
|------|------|------|
| `ana_log.py` | `tools/ana_log.py:545` | `RECOMMENDED_TIMING_MODULES` 列表 + 耗时统计分析 |
| `plot_icp_timing.py` | `tools/plot_icp_timing.py` | Fig C2 模块耗时时间线图 |
| `plot_araim_timeline.py` | `tools/plot_araim_timeline.py` | ARAIM 完整性时间线图 |

## 3. 用户提议分类的评估

### 3.1 用户原提议

```
1. 定位
   1.1 数据预处理耗时
   1.2 优化耗时
2. ARAIM
   2.1 GNSS ARAIM耗时
   2.2 LiDAR ARAIM耗时
   2.3 Fusion ARAIM耗时
3. Advisory PL Predict
   3.1 GNSS predict耗时
   3.2 LiDAR predict耗时
   3.3 Fusion predict耗时
4. Planning
   4.1 A star 耗时
   4.2 planning 优化求解耗时
```

### 3.2 调研发现与修正

**发现 1：A* 在 EGO planner 中，不在 IAP 核心** — IAP 自身的 `IntegrityPlanner` 使用运动基元轨迹生成，A* 代码位于 `sim/ego_planner_swarm_ws/src/planner/path_searching/src/dyn_a_star.cpp`（独立 ROS package），通过 `integrity_cost_query_` 回调与 IAP 的 PL grid 整合。B-spline 优化器在 `sim/ego_planner_swarm_ws/src/planner/bspline_opt/`。

→ **决定**：新增 Group 5: EGO Planner，含 5.1 A* 搜索 + 5.2 B-spline 优化。Group 4 保留为 IAP Planning（运动基元规划）。

**发现 2：Fusion ARAIM 计算量极小** — `IntegrityMonitor` 中的融合逻辑是 `max(PL_G, PL_L)`，耗时可忽略。

→ **修正**：2.3 改为「完整性监控总控」，内含 GNSS 门控、动态 AL、状态机子项。

**发现 3：缺少 Mapping 耗时** — Sub-mapping 和 Global mapping 零耗时测量。

→ **决定**：放在 Group 1.4（定位模块下）。

**发现 4：数据预处理范围不清晰** — ROS 回调层和 Odometry 层都有预处理步骤。

→ **修正**：1.1 为 ROS 层（回调+提取+预处理），1.2 为 Odometry 层（去畸变+协方差+积分等）。

## 4. 最终模块分类

### 命名规范

格式：`{major}.{minor}_{english_identifier}`

- `major`: 组号 (1-5)
- `minor`: 组内子模块序号
- `identifier`: 英文 snake_case 描述

### Group 1: 定位 (Localization)

| ID | 中文名 | Module Name | 状态 | 旧名称 |
|----|--------|-------------|------|--------|
| 1.1 | ROS回调总耗时 | `1.1_ros_callback_total` | 替换 | `ros_pointcloud_callback_total` |
| 1.1 | 点云提取 | `1.1_pointcloud_extract` | 替换 | `pointcloud_extract_raw` |
| 1.1 | 点云预处理 | `1.1_cloud_preprocess` | 替换 | `cloud_preprocess_total` |
| 1.2 | 帧插入总耗时 | `1.2_odom_insert_frame` | 替换 | `odom_insert_frame_total` |
| 1.2 | IMU积分 | `1.2_imu_integration` | 替换 | `odom_imu_integration` |
| 1.2 | 点云去畸变 | `1.2_pointcloud_deskew` | 替换 | `odom_deskew` |
| 1.2 | 协方差估计 | `1.2_covariance_estimation` | 替换 | `odom_covariance_estimation` |
| 1.2 | 因子创建 | `1.2_create_factors` | 替换 | `odom_create_factors` |
| 1.2 | iSAM2优化 | `1.2_smoother_update` | 替换 | `odom_smoother_update` |
| 1.2 | 帧状态更新 | `1.2_update_frames` | 替换 | `odom_update_frames` |
| 1.3 | GNSS注入 | `1.3_gnss_injection` | 替换 | `gnss_injection` |
| 1.3 | 树干检测 | `1.3_trunk_detector` | 替换 | `trunk_detector` |
| 1.4 | 子图插入 | `1.4_sub_map_insert` | **新增** | — |
| 1.4 | 全局图优化 | `1.4_global_map_optimize` | **新增** | — |
| 1.5 | 异步队列等待 | `1.5_odom_queue_wait` | 替换 | `async_odom_queue_wait` |

### Group 2: ARAIM (完整性监控)

| ID | 中文名 | Module Name | 状态 | 旧名称 |
|----|--------|-------------|------|--------|
| 2.1 | GNSS ARAIM | `2.1_gnss_araim` | 替换 | `araim` |
| 2.2 | LiDAR ARAIM | `2.2_lidar_araim` | **新增** | — |
| 2.3 | 完整性总控 | `2.3_integrity_total` | 替换 | `integrity` |
| 2.3 | GNSS门控 | `2.3_gnss_gating` | **新增** | — |
| 2.3 | 动态AL计算 | `2.3_dynamic_al` | **新增** | — |
| 2.3 | 状态机 | `2.3_state_machine` | **新增** | — |

### Group 3: Advisory PL Predict (咨询保护水平预测)

| ID | 中文名 | Module Name | 状态 | 旧名称 |
|----|--------|-------------|------|--------|
| 3.0 | 网格构建总耗时 | `3.0_pl_grid_build` | 替换 | `pl_grid_build` |
| 3.1 | GNSS咨询预测 | `3.1_gnss_predict` | **新增** | — |
| 3.2 | LiDAR咨询预测 | `3.2_lidar_predict` | **新增** | — |
| 3.3 | 融合PL计算 | `3.3_fusion_predict` | **新增** | — |

### Group 4: IAP Planning (IAP运动基元规划)

| ID | 中文名 | Module Name | 状态 | 旧名称 |
|----|--------|-------------|------|--------|
| 4.0 | 规划总耗时 | `4.0_plan_total` | **新增** | — |
| 4.1 | 候选轨迹生成 | `4.1_candidate_generation` | **新增** | — |
| 4.2 | PL预测量计算 | `4.2_predict_pl` | **新增** | — |
| 4.3 | 代价评估 | `4.3_cost_evaluation` | **新增** | — |

### Group 5: EGO Planner (外部A*+B-spline规划器)

| ID | 中文名 | Module Name | 状态 | 说明 |
|----|--------|-------------|------|------|
| 5.0 | EGO规划总耗时 | `5.0_ego_plan_total` | **新增** | `ego_replan_fsm.cpp` 中一次 replan 的总耗时 |
| 5.1 | A*搜索 | `5.1_astar_search` | **新增** | `dyn_a_star.cpp` A* 路径搜索 |
| 5.2 | B-spline优化 | `5.2_bspline_optimize` | **新增** | `bspline_optimizer.cpp` 轨迹优化 |

> **注意**：Group 5 代码在 `sim/ego_planner_swarm_ws/` 中，是独立的 ROS package（非 IAP 核心 `src/iap/`）。EGO planner 使用 `ament_cmake` 构建，不依赖 IAP 库。Group 5 的计时需要通过以下方式之一实现：
> - **方案A（推荐）**：在 EGO planner 代码中使用独立 `<chrono>` + 写入单独 CSV（如 `<run_dir>/profiling/ego_planner_timing.csv`），`ana_log.py` 在分析时合并两个 CSV。此方案零耦合。
> - **方案B**：让 EGO planner 的 CMakeLists.txt 依赖 IAP 的 `timing_csv.hpp`（需要确认无循环依赖）。优点是统一写入 `iap_timing.csv`。
>
> 建议采用方案A，保持 EGO planner 与 IAP 核心的解耦。

## 5. 模块总览：旧名→新名映射

| 旧名称 | 新名称 | 改动类型 |
|--------|--------|----------|
| `ros_pointcloud_callback_total` | `1.1_ros_callback_total` | 直接替换 |
| `pointcloud_extract_raw` | `1.1_pointcloud_extract` | 直接替换 |
| `cloud_preprocess_total` | `1.1_cloud_preprocess` | 直接替换 |
| `odom_insert_frame_total` | `1.2_odom_insert_frame` | 直接替换 |
| `odom_imu_integration` | `1.2_imu_integration` | 直接替换 |
| `odom_deskew` | `1.2_pointcloud_deskew` | 直接替换 |
| `odom_covariance_estimation` | `1.2_covariance_estimation` | 直接替换 |
| `odom_create_factors` | `1.2_create_factors` | 直接替换 |
| `odom_smoother_update` | `1.2_smoother_update` | 直接替换 |
| `odom_update_frames` | `1.2_update_frames` | 直接替换 |
| `async_odom_queue_wait` | `1.5_odom_queue_wait` | 直接替换 |
| `gnss_injection` | `1.3_gnss_injection` | 直接替换 |
| `trunk_detector` | `1.3_trunk_detector` | 直接替换 |
| `araim` | `2.1_gnss_araim` | 直接替换 |
| `integrity` | `2.3_integrity_total` | 直接替换 |
| `pl_grid_build` | `3.0_pl_grid_build` | 直接替换 |
| — | `1.4_sub_map_insert` | **新增** |
| — | `1.4_global_map_optimize` | **新增** |
| — | `2.2_lidar_araim` | **新增** |
| — | `2.3_gnss_gating` | **新增** |
| — | `2.3_dynamic_al` | **新增** |
| — | `2.3_state_machine` | **新增** |
| — | `3.1_gnss_predict` | **新增** |
| — | `3.2_lidar_predict` | **新增** |
| — | `3.3_fusion_predict` | **新增** |
| — | `4.0_plan_total` | **新增** |
| — | `4.1_candidate_generation` | **新增** |
| — | `4.2_predict_pl` | **新增** |
| — | `4.3_cost_evaluation` | **新增** |
| — | `5.0_ego_plan_total` | **新增** |
| — | `5.1_astar_search` | **新增** |
| — | `5.2_bspline_optimize` | **新增** |

总计：16 个旧名称直接替换，16 个全新模块（13 个 IAP 核心 + 3 个 EGO Planner）。

## 6. 实施计划

### Phase A: 基础设施增强

**A1. 新增 `CumulativeTimer` 辅助类** (`include/iap/util/timing_csv.hpp`)

Group 3 网格重建时每个 cell 调用 `evaluate_point()`，如逐 cell 写入 CSV 会产生数千行。`CumulativeTimer` 提供 start/stop 累积 + 一次性 flush：

```cpp
class CumulativeTimer {
public:
    void start();
    void stop();      // 累加本次 start-stop 间隔
    void reset();     // 清零累积值
    void flush(double stamp, const char* module);  // 写入 CSV 并清零
private:
    double accumulated_ms_ = 0.0;
    std::chrono::steady_clock::time_point t0_;
};
```

### Phase B: 定位模块 (Group 1) — 直接替换旧名称

所有现有 `ScopedTimer` / `append()` 调用中的旧名称直接替换为新层次化名称，不保留别名。

| 文件 | 行号 | 旧名称 | 新名称 |
|------|------|--------|--------|
| `apps/iap_rosnode.cpp` | 342 | `ros_pointcloud_callback_total` | `1.1_ros_callback_total` |
| `apps/iap_rosnode.cpp` | 359 | `pointcloud_extract_raw` | `1.1_pointcloud_extract` |
| `apps/iap_rosnode.cpp` | 375 | `cloud_preprocess_total` | `1.1_cloud_preprocess` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 170 | `odom_insert_frame_total` | `1.2_odom_insert_frame` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 231,433 | `odom_covariance_estimation` | `1.2_covariance_estimation` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 287,451 | `odom_create_factors` | `1.2_create_factors` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 292,458 | `odom_smoother_update` | `1.2_smoother_update` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 296,479 | `odom_update_frames` | `1.2_update_frames` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 319 | `odom_imu_integration` | `1.2_imu_integration` |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 423 | `odom_deskew` | `1.2_pointcloud_deskew` |
| `src/iap/odometry/async_odometry_estimation.cpp` | 119 | `async_odom_queue_wait` | `1.5_odom_queue_wait` |
| `src/iap/gnss/gnss_extension.cpp` | 1018 | `gnss_injection` | `1.3_gnss_injection` |
| `src/iap/trunk/trunk_detector.cpp` | 213 | `trunk_detector` | `1.3_trunk_detector` |

**新增 Mapping 耗时**（需新增 `#include <iap/util/timing_csv.hpp>`）：

| 文件 | 测量点 | 新名称 |
|------|--------|--------|
| `src/iap/mapping/async_sub_mapping.cpp` | 批量帧插入循环前后 | `1.4_sub_map_insert` |
| `src/iap/mapping/async_global_mapping.cpp` | `optimize()` 调用前后 | `1.4_global_map_optimize` |

### Phase C: ARAIM 模块 (Group 2)

**C1. GNSS ARAIM** (`src/iap/integrity/araim.cpp:511`) — 将 `"araim"` 替换为 `"2.1_gnss_araim"`

**C2. LiDAR ARAIM 新增计时** (`src/iap/integrity/lidar_araim.cpp`)：
- 新增 `#include <iap/util/timing_csv.hpp>` 和 `#include <chrono>`
- 在 `LidarAraim::run()` (line 298) 开头记录 `t0`，在 return 前写入 `"2.2_lidar_araim"`

**C3. 完整性总控及子模块** (`src/iap/integrity/integrity_monitor.cpp`)：
- `integrity` → `"2.3_integrity_total"` (line 493)
- 新增 `"2.3_gnss_gating"` — 包裹 `run_gnss_gating()` 调用 (line 432-434)
- 新增 `"2.3_dynamic_al"` — 包裹 `compute_dynamic_AL()` 调用 (line 411)
- 新增 `"2.3_state_machine"` — 包裹 `update_state()` 调用 (line 456)

### Phase D: Advisory PL Predict (Group 3)

**D1. 网格构建总耗时** (`src/iap/planner/future_pl_field_predictor.cpp:226`) — `"pl_grid_build"` → `"3.0_pl_grid_build"`

**D2. 子模块耗时拆分** — 在 `rebuild_grid()` 的三重循环和 `evaluate_point()` 中使用 `CumulativeTimer`：

```
// rebuild_grid() 中:
CumulativeTimer ct_gnss, ct_lidar, ct_fusion;

for each cell:
    value = evaluate_point(p, ..., &ct_gnss, &ct_lidar, &ct_fusion);

ct_gnss.flush(now_s, "3.1_gnss_predict");
ct_lidar.flush(now_s, "3.2_lidar_predict");
ct_fusion.flush(now_s, "3.3_fusion_predict");
```

`evaluate_point()` 签名变更：接受三个 `CumulativeTimer*`，在内部各阶段调用 start/stop。

### Phase E: IAP Planning (Group 4)

**E1. 规划总耗时及子模块** (`src/iap/planner/integrity_planner.cpp`)：

```cpp
CandidateTrajectory IntegrityPlanner::plan(..., double stamp) const {
  iap::timing_csv::ScopedTimer plan_timer(stamp, "4.0_plan_total");

  // Phase 1: candidate generation
  auto candidates = [&] {
    iap::timing_csv::ScopedTimer t(stamp, "4.1_candidate_generation");
    return generator_.generate(pos0, vel0, yaw0);
  }();

  // Phase 2: PL prediction
  {
    iap::timing_csv::ScopedTimer t(stamp, "4.2_predict_pl");
    predictor_.predict_all(candidates, sigma0);
    for (auto& traj : candidates) { /* ARAIM predict loop */ }
  }

  // Phase 3: cost evaluation
  {
    iap::timing_csv::ScopedTimer t(stamp, "4.3_cost_evaluation");
    for (auto& traj : candidates) { evaluate(traj, ...); }
  }

  return candidates[best_idx];
}
```

**接口变更**：`PlannerInterface::plan()` 新增 `double stamp = 0.0` 参数。需同步更新：
- `include/iap/planner/planner_interface.hpp` — 虚函数签名
- `include/iap/planner/integrity_planner.hpp` — 声明
- 所有 `plan()` 调用方（`apps/iap_experiment.cpp`、`apps/phase2_planner_integrity_evaluator.cpp`）

### Phase F: EGO Planner (Group 5)

> 采用方案A（独立 CSV，零耦合）

**F1. A* 搜索耗时** (`sim/ego_planner_swarm_ws/src/planner/path_searching/src/dyn_a_star.cpp`)：
- 在 `AstarSearchImpl()` 中使用 `<chrono>` 记录 start/end
- 写入独立 CSV 文件：`<run_dir>/profiling/ego_planner_timing.csv`（格式同 IAP：`stamp,module,elapsed_ms`）
- Module name: `5.1_astar_search`

**F2. B-spline 优化耗时** (`sim/ego_planner_swarm_ws/src/planner/bspline_opt/src/bspline_optimizer.cpp`)：
- 在 `BsplineOptimizer::optimize()` 中同上方式计时
- Module name: `5.2_bspline_optimize`

**F3. EGO 规划总耗时** (`sim/ego_planner_swarm_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp`)：
- 在一次完整的 replan 流程（A* + B-spline + 轨迹选择）外层包裹
- Module name: `5.0_ego_plan_total`

**F4. CSV 写入辅助**：在 EGO planner 中新增一个小型独立头文件（如 `ego_timing.hpp`），提供与 IAP `timing_csv` 相同的格式输出（复用相同的输出目录路径逻辑），但完全独立于 IAP 库。

**F5. 分析工具合并**：`ana_log.py` 在分析时自动检测 `ego_planner_timing.csv` 是否存在，存在则合并到同一份报告中。

### Phase G: 分析工具更新

**G1. `tools/ana_log.py`**:
- 替换 `RECOMMENDED_TIMING_MODULES` 为层次化版本：
```python
TIMING_MODULES_V2 = {
    "1": {  # 定位
        "label_cn": "定位 (Localization)",
        "modules": [
            "1.1_ros_callback_total", "1.1_pointcloud_extract",
            "1.1_cloud_preprocess", "1.2_odom_insert_frame",
            "1.2_imu_integration", "1.2_pointcloud_deskew",
            "1.2_covariance_estimation", "1.2_create_factors",
            "1.2_smoother_update", "1.2_update_frames",
            "1.3_gnss_injection", "1.3_trunk_detector",
            "1.4_sub_map_insert", "1.4_global_map_optimize",
            "1.5_odom_queue_wait",
        ]
    },
    "2": {  # ARAIM
        "label_cn": "ARAIM (完整性监控)",
        "modules": [
            "2.1_gnss_araim", "2.2_lidar_araim",
            "2.3_integrity_total", "2.3_gnss_gating",
            "2.3_dynamic_al", "2.3_state_machine",
        ]
    },
    "3": {  # Advisory PL Predict
        "label_cn": "Advisory PL Predict (咨询保护水平预测)",
        "modules": [
            "3.0_pl_grid_build", "3.1_gnss_predict",
            "3.2_lidar_predict", "3.3_fusion_predict",
        ]
    },
    "4": {  # IAP Planning
        "label_cn": "IAP Planning (运动基元规划)",
        "modules": [
            "4.0_plan_total", "4.1_candidate_generation",
            "4.2_predict_pl", "4.3_cost_evaluation",
        ]
    },
    "5": {  # EGO Planner (optional, from separate CSV)
        "label_cn": "EGO Planner (A*+B-spline规划)",
        "modules": [
            "5.0_ego_plan_total", "5.1_astar_search",
            "5.2_bspline_optimize",
        ]
    },
}
```
- 修改 `analyze_real_time_timing()` 按 Group 分组展示表格
- 新增 `module_group(name)` 辅助函数提取组号
- 新增 EGO planner CSV 自动合并逻辑

**G2. `tools/plot_icp_timing.py`**:
- 更新 `modules` 列表，使用新名称
- 新增 Group 5 对应颜色

### Phase H: 编译开关（可选）

在 `CMakeLists.txt` 中新增 `IAP_ENABLE_DETAILED_TIMING` 选项（默认 ON）。控制 Group 3 逐 cell 子计时器的 `CumulativeTimer`。在性能敏感部署中可关闭以减少 per-cell `steady_clock::now()` 开销。

## 7. 向后兼容策略

**策略：直接替换，不做双写。**

| 方面 | 处理方式 |
|------|----------|
| CSV 输出 | 只写新名称，不保留旧名称 |
| 分析工具 | `ana_log.py` 内置 `LEGACY_NAME_MAP` 映射表，用于解析历史日志 |
| 历史日志 | 旧日志通过映射表自动转换显示为层次化名称 |
| 新日志 | 直接使用新名称，无需转换 |

```python
# ana_log.py 中的旧名→新名映射（用于解析历史日志）
LEGACY_NAME_MAP = {
    "ros_pointcloud_callback_total": "1.1_ros_callback_total",
    "pointcloud_extract_raw": "1.1_pointcloud_extract",
    "cloud_preprocess_total": "1.1_cloud_preprocess",
    "odom_insert_frame_total": "1.2_odom_insert_frame",
    "odom_imu_integration": "1.2_imu_integration",
    "odom_deskew": "1.2_pointcloud_deskew",
    "odom_covariance_estimation": "1.2_covariance_estimation",
    "odom_create_factors": "1.2_create_factors",
    "odom_smoother_update": "1.2_smoother_update",
    "odom_update_frames": "1.2_update_frames",
    "async_odom_queue_wait": "1.5_odom_queue_wait",
    "gnss_injection": "1.3_gnss_injection",
    "trunk_detector": "1.3_trunk_detector",
    "araim": "2.1_gnss_araim",
    "integrity": "2.3_integrity_total",
    "pl_grid_build": "3.0_pl_grid_build",
}
```

## 8. 实施顺序与工作量估算

| 步骤 | 描述 | 依赖 | 预估 |
|------|------|------|------|
| 1 | `CumulativeTimer` 类 (`timing_csv.hpp`) | — | 30min |
| 2 | Group 1 替换 (13处) (`iap_rosnode.cpp`, `odometry_estimation_imu.cpp`, `async_odometry_estimation.cpp`, `gnss_extension.cpp`, `trunk_detector.cpp`) | 1 | 20min |
| 3 | Group 1 新增 Mapping (2处) (`async_sub_mapping.cpp`, `async_global_mapping.cpp`) | 1 | 20min |
| 4 | Group 2 替换+新增 (`araim.cpp`, `lidar_araim.cpp`, `integrity_monitor.cpp`) | 1 | 45min |
| 5 | Group 3 替换+子模块拆分 (`future_pl_field_predictor.cpp`) | 1 | 45min |
| 6 | Group 4 新增 + 接口变更 (`integrity_planner.cpp`, `planner_interface.hpp`, callers) | 1 | 45min |
| 7 | Group 5 EGO Planner 新增 (`dyn_a_star.cpp`, `bspline_optimizer.cpp`, `ego_replan_fsm.cpp`, `ego_timing.hpp`) | — | 45min |
| 8 | CMakeLists.txt 编译开关 | 5 | 10min |
| 9 | `ana_log.py` 更新 | 2-7 | 60min |
| 10 | `plot_icp_timing.py` 更新 | 2-7 | 15min |
| 11 | 编译验证 | 1-8 | 30min |
| 12 | 运行时验证（bag回放+CSV检查+报告生成） | 9-11 | 30min |

**总计：约 6.5 小时**

## 9. 验证计划

1. **编译**: `colcon build --packages-select iap` 无警告通过
2. **单元测试**: `colcon test --packages-select iap` 全部通过
3. **CSV 验证**: 运行 bag 回放，检查 `profiling/iap_timing.csv`:
   - 无旧名称残留
   - 所有新名称均有非零耗时数据
   - Group 3 子项耗时之和 ≈ `3.0_pl_grid_build` 总耗时（允许<5% 测量开销误差）
   - Group 2 子项耗时之和 ≤ `2.3_integrity_total`（子项只覆盖主要操作）
4. **EGO CSV 验证**: 检查 `profiling/ego_planner_timing.csv` 格式正确
5. **分析报告**: 运行 `ana_log.py`，验证:
   - 5 个 Group 层次化分组显示正确
   - 无 missing module 警告
   - 历史日志（旧名称）能正确映射显示
   - 瓶颈识别功能正常
6. **图表**: 运行 `plot_icp_timing.py`，验证 Fig C2 渲染正确

## 10. 风险与注意事项

| 风险 | 缓解措施 |
|------|----------|
| `PlannerInterface::plan()` 签名变更破坏编译 | 同步更新所有调用方（已在 grep 验证仅 2 个内部调用方） |
| Mapping 异步线程 timestamp 不对齐 | 使用 `0.0` sentinel 标记；分析时按 wall clock 对齐或单独统计 |
| Group 3 per-cell 计时引入可观测开销 | `IAP_ENABLE_DETAILED_TIMING` 编译开关 + CumulativeTimer 代替逐 cell 写 CSV |
| EGO planner CSV 路径与 IAP 不一致 | 通过环境变量或配置文件传递 `run_dir` 路径 |
| `odom_covariance_estimation` 两次出现共用同一名称 | 保持原有行为（两次出现用同一名称是正确的，归入同一统计） |

## 11. 涉及的关键文件

### IAP 核心 (src/iap/)

| 文件 | 改动类型 |
|------|----------|
| `include/iap/util/timing_csv.hpp` | 新增 `CumulativeTimer` 类 |
| `apps/iap_rosnode.cpp` | 替换旧名称 (3处) |
| `src/iap/odometry/odometry_estimation_imu.cpp` | 替换旧名称 (11处) |
| `src/iap/odometry/async_odometry_estimation.cpp` | 替换旧名称 (1处) |
| `src/iap/gnss/gnss_extension.cpp` | 替换旧名称 (1处) |
| `src/iap/trunk/trunk_detector.cpp` | 替换旧名称 (1处) |
| `src/iap/integrity/araim.cpp` | 替换旧名称 (1处) |
| `src/iap/integrity/lidar_araim.cpp` | **新增** chrono 计时 |
| `src/iap/integrity/integrity_monitor.cpp` | 替换旧名称 + 3个子计时器 |
| `src/iap/planner/future_pl_field_predictor.cpp` | 替换旧名称 + 3个子计时器 + `evaluate_point()` 签名变更 |
| `src/iap/planner/integrity_planner.cpp` | **新增** 4个计时器 |
| `include/iap/planner/planner_interface.hpp` | `plan()` 签名变更 (+stamp) |
| `include/iap/planner/integrity_planner.hpp` | `plan()` 签名变更 (+stamp) |
| `apps/iap_experiment.cpp` | `plan()` 调用方适配 |
| `apps/phase2_planner_integrity_evaluator.cpp` | `plan()` 调用方适配 |
| `src/iap/mapping/async_sub_mapping.cpp` | **新增** 计时器 |
| `src/iap/mapping/async_global_mapping.cpp` | **新增** 计时器 |
| `CMakeLists.txt` | 新增 `IAP_ENABLE_DETAILED_TIMING` 选项 |

### EGO Planner (sim/ego_planner_swarm_ws/)

| 文件 | 改动类型 |
|------|----------|
| `src/planner/path_searching/src/dyn_a_star.cpp` | **新增** A* 计时 |
| `src/planner/bspline_opt/src/bspline_optimizer.cpp` | **新增** B-spline 计时 |
| `src/planner/plan_manage/src/ego_replan_fsm.cpp` | **新增** 总耗时计时 |
| `src/planner/path_searching/include/ego_timing.hpp` | **新增** 独立计时辅助头文件 |

### 分析工具

| 文件 | 改动类型 |
|------|----------|
| `tools/ana_log.py` | 替换 `RECOMMENDED_TIMING_MODULES` + 层次化分组显示 + 旧名映射 + EGO CSV 合并 |
| `tools/plot_icp_timing.py` | 更新模块列表和颜色 |
