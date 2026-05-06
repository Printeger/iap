# Stage 1 IAP ARAIM 预测模块实现 Handbook

> 目标：在当前 `Printeger/iap` 的 `dev/iap` 分支上，把 demo10 中的 `phase2_pl_model:=constant_current` 升级为一个可运行、可验证、可逐步接入 planner cost 的 **Future ARAIM / PL Field Predictor**。
>
> 核心原则：Stage 1 只做 **解耦式、建议性、前向预测**。当前 ARAIM 仍然是“真监测器”；future predictor 只负责给未来候选轨迹提供 `HPL/VPL/IR/IM` 预测场，不替代当前 ARAIM，不污染 estimator，不直接声称 certified monitor。

---

## 0. 当前 IAP 状态与本 handbook 的落点

### 0.1 当前已有基础

当前 `dev/iap` README 描述的系统已经具备以下基础：

- IAP 是基于 GLIM 架构的 3D LiDAR-IMU(-GNSS) 定位与建图系统，并提供 UAV 仿真环境、随机森林地图、SO3 四旋翼动力学、仿真 LiDAR/IMU、GNSS 仿真、IAP 与仿真的桥接节点，以及 `demo1` 到 `demo10` 的分层示例。
- `demo7` 是 GNSS/ARAIM 集成主示例，支持 open-sky、SkyMask+NLOS、单星故障注入等场景。
- `demo8` 输出 GNSS/ARAIM 真值对照结果，关键产物包括 `/iap/integrity` 和 `export/iap_araim.csv`。
- `demo9` 是 EGO planner + IAP odom + SO3 controller 的 Phase 1 闭环验证。
- `demo10` 在 demo9 闭环栈上增加 `phase2_planner_integrity_evaluator`，沿 EGO planner 的未来 B-spline 采样并导出 `AL_pred`、`PL_pred`、`IM_pred`；当前官方命令仍使用 `phase2_pl_model:=constant_current`。

### 0.2 当前代码中已经存在的相关 C++ 模块

从当前 CMake 配置和头文件看，IAP 已经有几个可以复用的模块：

- `iap::Araim`：当前 GNSS ARAIM engine，支持真实 epoch 的 `run()` 和几何预测模式 `predict_geometry()`。
- `iap::PredictedAraimComputer`：当前已经是 “geometry-only ARAIM at candidate waypoint” 的雏形，内部使用 `VisibilityPredictor` + `Araim::predict_geometry()`，但接口只返回一个 scalar PL。
- `iap::PredictedIntegrityComputer`：当前是 `sigma_grow` random-walk 型 PL proxy，可作为 baseline，但不应作为最终 Stage 1 predictor。
- `iap::LidarAraim`：当前 LiDAR-ARAIM 结构已经有 `H_SOURCE / H_TARGET / H_LEVEL` 三类 block hypothesis，非常适合作为 **当前段监测与 snapshot 输出**，但不应被机械复制到未来段。
- ROS message 已有 `IntegrityReport.msg` 和 `DynamicALResult.msg`，前者包含 `hpl/vpl/pl_e/pl_n/pl_u/hal/val/im/n_sv_used/pdop/tdop/excluded_prns/excluded_trunk_ids` 等字段，后者包含动态 AL 相关字段。

### 0.3 本 handbook 的最终交付目标

完成后你应该得到：

```text
Current ARAIM + estimator snapshot
        |
        v
IntegritySnapshot
        |
        v
FuturePLFieldPredictor
  - GNSS visibility / SkyMask predictor
  - GNSS geometry ARAIM predictor
  - LiDAR observability FIM predictor
  - ARAIM-style PL computer
  - PL/IR/IM grid cache + trilinear interpolation
        |
        v
query(p) -> {HPL, VPL, IR, AL, IM, grad, valid, debug}
        |
        v
phase2_planner_integrity_evaluator / future planner cost
```

Stage 1 的第一完成标准不是“planner 自动绕开高 PL 区域”，而是：

1. demo10 能从 `constant_current` 切换到 `gnss_geometry_araim` 或 `fused_fim_grid`；
2. `integrity_along_planner_traj.csv` 中每个未来采样点都有 `HPL_pred/VPL_pred/AL_pred/IM_pred/n_vis/pdop/source/fallback_reason`；
3. `PL_pred(x_now)` 与当前 `/iap/integrity` 的 `hpl/vpl` 有可解释的一致性；
4. demo7 SkyMask/NLOS/fault 场景中，PL 场会随可见性和几何退化变化；
5. evaluator 保持 read-only，不修改 planner、estimator、当前 ARAIM、控制器。

---

## 1. 设计原则

### 1.1 不要把 future predictor 写成“未来再跑一次 ARAIM”

未来段没有真实未来 residual、没有真实未来 source scan、没有真实 future VGICP block，也没有真实 target-level block 线性化结果。因此 future predictor 不能假装自己是在做完整认证级 ARAIM。

正确定位：

```text
Current ARAIM = certified/current monitor
Future predictor = advisory/queryable forward integrity field
```

### 1.2 共享当前 ARAIM 的统计语义

未来预测必须和当前 ARAIM 共享：

- 同一套 `P_HMI_req / P_FA_req`；
- 同一套 `K_ff / K_fa / K_md` 语义；
- 同一套 fault prior 配置；
- 同一套 AL/IM 判据；
- 同一套 fallback/conservative 逻辑。

否则 planner 看到的是一个“漂亮但不可解释”的 cost field，而不是 ARAIM-compatible predictor。

### 1.3 在当前位置做 self-consistency check

每次 snapshot 更新后，必须检查：

```math
HPL_pred(p_now) \approx HPL_now
```

```math
VPL_pred(p_now) \approx VPL_now
```

第一阶段可以允许 5% 到 10% 偏差；模型稳定后目标收敛到 1% 到 3%。如果当前位置都对不齐，未来点预测没有可信度。

### 1.4 Stage 1 不做的事情

Stage 1 不做以下事情：

- 不把 planner cost 写回 estimator；
- 不修改 fixed-lag smoother 主拓扑；
- 不做连续时间 B-spline 联合 MAP；
- 不把 future LiDAR predictor 做成虚拟 `source × target × level` block ARAIM；
- 不在 planner 每次 query 时完整遍历所有 ARAIM hypothesis；
- 不声称 future PL 是认证级 protection level。

---

## 2. 推荐模块拆分

### 2.1 新增/改造文件建议

建议以 C++ 核心库 + demo10 evaluator 轻量调用的方式实现。

```text
include/iap/planner/
  integrity_snapshot.hpp              # 新增：统一 snapshot 结构
  future_pl_query_result.hpp           # 新增：query 返回值
  pl_grid.hpp                          # 新增：3D grid + interpolation
  future_pl_field_predictor.hpp        # 新增：主 predictor interface
  gnss_geometry_fim.hpp                # 新增或内嵌：GNSS FIM helper
  lidar_observability_fim.hpp          # 新增：LiDAR future observability helper
  pi_cost_adapter.hpp                  # 可选：planner cost wrapper

src/iap/planner/
  pl_grid.cpp
  future_pl_field_predictor.cpp
  gnss_geometry_fim.cpp
  lidar_observability_fim.cpp
  pi_cost_adapter.cpp

msg/
  FutureIntegrityPrediction.msg        # 可选：若要发布 debug topic
  FutureIntegrityGridInfo.msg          # 可选：若要发布 grid summary

launch/
  demo10_ego_planner_pi_lite_eval.launch.py  # 加新参数

config/
  sim_demo10_integrity_predictor.json   # 推荐新增专用配置

tools/phase2/
  analyze_phase2_integrity_eval.py      # 扩展字段分析
  validate_phase2_integrity_eval.py     # 扩展 validation
```

### 2.2 不建议第一步就做的文件改动

第一轮不要动：

```text
src/iap/odometry/*
src/iap/mapping/*
src/iap/integrity/araim.cpp 的核心 run() 逻辑
src/iap/integrity/lidar_araim.cpp 的当前段 hypothesis loop
sim/ego_planner_swarm_ws/src/* 的 planner optimizer
```

可以先在 `src/iap/planner/*` 中实现 predictor，等 demo10 read-only 通过后再接 planner cost。

---

## 3. 数据结构定义

### 3.1 `IntegritySnapshot`

新增 `include/iap/planner/integrity_snapshot.hpp`。

```cpp
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iap/gnss/gnss_types.hpp>
#include <iap/integrity/araim_types.hpp>
#include <iap/integrity/lidar_araim.hpp>
#include <vector>
#include <string>

namespace iap {

struct IntegritySnapshot {
  // ---------- time / pose ----------
  double stamp = 0.0;
  bool valid = false;
  Eigen::Vector3d p_wb = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_wb = Eigen::Quaterniond::Identity();

  // ---------- current covariance / information ----------
  Eigen::Matrix3d Sigma_pos_now = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Lambda_base_pos = Eigen::Matrix3d::Identity();
  bool has_lambda_base = false;

  // ---------- current ARAIM output ----------
  double hpl_now = 1e9;
  double vpl_now = 1e9;
  double hal_now = 0.0;
  double val_now = 0.0;
  double im_now = -1e9;
  double ir_now = 1.0;
  int integrity_state = 2;  // 0 SAFE, 1 SAFE_EXCLUDED, 2 UNSAFE

  // ---------- GNSS snapshot ----------
  bool has_gnss_epoch = false;
  GnssEpoch gnss_epoch;
  std::vector<int> excluded_prns;
  int n_sv_used = 0;
  double pdop_now = 1e9;
  double sigma_h_now = 1e9;

  // ---------- LiDAR ARAIM snapshot ----------
  bool has_lidar_snapshot = false;
  LidarAraimSnapshot lidar_snapshot;
  LidarAraimResult lidar_araim_result;
  double lidar_reliability_scale = 1.0;  // alpha_L
  double lidar_bias_overbound_h = 0.0;
  double lidar_bias_overbound_v = 0.0;

  // ---------- risk / multipliers ----------
  double P_HMI_req = 1e-7;
  double P_FA_req = 1e-5;
  double K_ff = 5.42;
  double K_fa = 4.50;
  double K_md = 5.50;

  // ---------- debug ----------
  std::string source = "unknown";
};

}  // namespace iap
```

### 3.2 `FuturePLQueryResult`

新增 `include/iap/planner/future_pl_query_result.hpp`。

```cpp
#pragma once

#include <Eigen/Core>
#include <string>

namespace iap {

struct FuturePLQueryResult {
  bool valid = false;
  bool fallback = false;
  std::string fallback_reason;

  double hpl = 1e9;
  double vpl = 1e9;
  double ir = 1.0;

  double hal = 0.0;
  double val = 0.0;
  double al = 0.0;

  double im_h = -1e9;
  double im_v = -1e9;
  double im_min = -1e9;

  double pl_ff_h = 1e9;
  double pl_ff_v = 1e9;
  double sigma_h = 1e9;
  double sigma_v = 1e9;

  int n_vis = 0;
  int n_hypotheses = 0;
  double pdop = 1e9;
  double tdop = 1e9;
  double lidar_alpha = 1.0;

  Eigen::Vector3d grad_hpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_vpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_im = Eigen::Vector3d::Zero();
};

}  // namespace iap
```

### 3.3 `FuturePLFieldPredictor` 主接口

新增 `include/iap/planner/future_pl_field_predictor.hpp`。

```cpp
#pragma once

#include <iap/planner/integrity_snapshot.hpp>
#include <iap/planner/future_pl_query_result.hpp>
#include <iap/map/local_occupancy.hpp>
#include <memory>
#include <mutex>

namespace iap {

class FuturePLFieldPredictor {
public:
  struct Params {
    enum class Mode {
      CONSTANT_CURRENT = 0,
      GNSS_GEOMETRY_ARAIM = 1,
      FUSED_FIM_GRID = 2
    };

    Mode mode = Mode::GNSS_GEOMETRY_ARAIM;

    double fallback_hpl = 20.0;
    double fallback_vpl = 20.0;
    double min_valid_hal = 0.1;
    double min_valid_val = 0.1;

    // Grid parameters
    bool use_grid = true;
    double grid_resolution = 1.0;
    double grid_size_x = 30.0;
    double grid_size_y = 30.0;
    double grid_size_z = 8.0;
    double grid_update_hz = 2.0;

    // GNSS prediction
    int min_sats = 4;
    bool use_map_occlusion = true;
    bool use_elevation_sigma = true;

    // LiDAR prediction
    bool use_lidar_observability = false;
    double lidar_info_scale = 1.0;
    double lidar_min_alpha = 0.1;
    double lidar_max_alpha = 1.0;

    // Consistency calibration
    bool calibrate_to_current_pl = true;
    double calibration_alpha_min = 0.5;
    double calibration_alpha_max = 5.0;
  };

  explicit FuturePLFieldPredictor(const Params& params);

  void set_occupancy(const LocalOccupancyGrid* grid);
  void update_snapshot(const IntegritySnapshot& snapshot);

  // Slow path: compute directly at p.
  FuturePLQueryResult evaluate_point(const Eigen::Vector3d& p_w) const;

  // Fast path: query active grid if available, otherwise fallback to evaluate_point().
  FuturePLQueryResult query(const Eigen::Vector3d& p_w) const;

  // Rebuild local grid around snapshot.p_wb. Should run in a prediction thread.
  void rebuild_grid();

  const Params& params() const { return params_; }

private:
  Params params_;
  const LocalOccupancyGrid* occupancy_ = nullptr;

  mutable std::mutex snapshot_mutex_;
  IntegritySnapshot snapshot_;
};

}  // namespace iap
```

---

## 4. 数学定义

### 4.1 统一预测信息矩阵

不要从零构造未来信息矩阵。每个候选位置 `p` 使用：

```math
\Lambda_{pred}^{(0)}(p)
=
\Lambda_{base}
+
\Delta\Lambda_{GNSS}(p)
+
\alpha_L(p)\Delta\Lambda_{LiDAR}(p)
```

其中：

- `Λ_base`：当前 estimator / smoother / FGO snapshot 提供的基线位置信息；没有完整 3×3 信息时，用当前 `Σ_pos_now` 逆阵加 damping；
- `ΔΛ_GNSS(p)`：候选位置处的预测 GNSS 几何信息；
- `ΔΛ_LiDAR(p)`：候选位置处的预测 LiDAR 几何可观性信息；
- `α_L(p)`：由当前 LiDAR ARAIM 结果调制的可靠性缩放。

### 4.2 GNSS 几何预测

第一版直接复用当前 `VisibilityPredictor` 和 `Araim::predict_geometry()`：

```cpp
PredictedAraimComputer pred_araim(params);
pred_araim.set_occupancy(local_grid);
pred_araim.set_epoch(&snapshot.gnss_epoch);
double hpl = pred_araim.predict_araim_pl(p_w);
```

但这个接口只返回 scalar `pl_araim`，建议立刻升级为：

```cpp
struct PredictedAraimResult {
  bool valid = false;
  double hpl = 1e9;
  double vpl = 1e9;
  double pl_ff_h = 1e9;
  double pl_ff_v = 1e9;
  double sigma_h = 1e9;
  double sigma_v = 1e9;
  double pdop = 1e9;
  int n_vis = 0;
  int n_hypotheses = 0;
  std::string fallback_reason;
};

PredictedAraimResult predict_araim(const Eigen::Vector3d& pos_world) const;
```

第一版 GNSS flow：

```text
p_world
  -> VisibilityPredictor::predict(p_world, epoch)
  -> visible SatGeometry list
  -> Araim::predict_geometry(visible_sats)
  -> PredictedAraimResult{hpl, vpl, pl_ff, sigma, n_vis, pdop}
```

### 4.3 LiDAR future predictor 不复制当前 block fault mode

当前 `LidarAraim` 的 `H_SOURCE / H_TARGET / H_LEVEL` 非常适合当前帧 VGICP block 完好性检测。未来段没有真实 future VGICP block，因此未来 LiDAR 应改为 observability-level model。

推荐第一版：从局部地图或 trunk map 中提取候选位置 `p` 附近可见 primitive，构造近似位置信息：

```math
\Delta\Lambda_{LiDAR}(p)
=
\sum_{k \in \hat{O}(p)}
J_k(p)^T R_k^{-1} J_k(p)
```

若使用 surfel/plane normal：

```math
J_k \approx n_k^T
```

```math
\Delta\Lambda_{LiDAR}(p) += \frac{1}{\sigma_k^2} n_k n_k^T
```

若使用 trunk / landmark bearing-range：

```math
u_k(p) = \frac{c_k - p_{xy}}{\|c_k - p_{xy}\|}
```

```math
\Delta\Lambda_{LiDAR,xy}(p) += \frac{1}{\sigma_{trunk,k}^2} u_k u_k^T
```

### 4.4 当前 LiDAR ARAIM 如何影响未来 LiDAR predictor

不要把当前 `H_SOURCE/H_TARGET/H_LEVEL` 原样搬到未来。它们应作为未来 LiDAR 信息的置信调制项。

| 当前 LiDAR ARAIM 输出 | 未来 predictor 处理 |
|---|---|
| `H_SOURCE` 可疑 | 降低全局 `alpha_L`，增加 bias overbound |
| 某 `H_LEVEL` 可疑 | 屏蔽或降权对应 resolution/level 的 map primitive |
| 某 `H_TARGET` 可疑 | 对对应 map region 降权，或增加该区域 bias |
| `gamma_lidar` 高 | 降低 `alpha_L`，提高 `PL_L` |
| `cond_proxy` 高 | 对退化方向提高 PL，TDOP 增大 |

第一版可以只做一个 scalar：

```cpp
alpha_L = clamp(
  1.0 / (1.0 + w_gamma * max_gamma_lidar + w_detect * n_detected),
  alpha_min,
  alpha_max);
```

### 4.5 ARAIM-style PL 结构

最终 PL 不应只是 `K * sqrt(cov)`，而应保留 ARAIM-style max structure：

```math
PL_q(p)
=
\max\left(
K_{ff,q}\sigma_{q,0}(p),
\max_{f \in H}
\left[
|d_{q,f}(p)|
+
K_{fa,f}\sigma_{ss,qf}(p)
+
K_{md,f}\sigma_{q,f}(p)
+
b_{q,f}(p)
\right]
\right)
```

Stage 1 可分三步实现：

1. `GNSS_GEOMETRY_ARAIM`：只用 `Araim::predict_geometry()`，此时 `d=0`，geometry-driven；
2. `FUSED_FIM_GRID`：加入 `Λ_base + ΔΛ_GNSS + α_L ΔΛ_LiDAR` 的 covariance PL；
3. `FUSED_ARAIM_STYLE`：GNSS 侧 subset hypothesis + LiDAR conservative bias overbound。

---

## 5. 可执行实施步骤

## Phase A：冻结 demo10 baseline，建立回归基准

### A1. 运行当前 demo10

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=constant_current \
  phase2_al_model:=cloud_clearance
```

### A2. 保存 baseline 产物

```bash
python3 src/iap/tools/phase2/analyze_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest

python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest

cp -r src/iap/log/latest src/iap/log/baseline_demo10_constant_current
```

### A3. 记录 baseline 指标

至少记录：

- `integrity_along_planner_traj.csv` 行数；
- 每条 planner trajectory 的 sample count；
- `AL_pred` 最小值、均值；
- `PL_pred` 是否等于 current PL；
- `IM_pred` 最小值；
- evaluator 是否有 NaN、空 trajectory、时间戳错位；
- analyzer/validator 是否通过。

### A4. Definition of Done

- demo10 能稳定运行 60 秒；
- `export/integrity_along_planner_traj.csv`、`phase2_integrity_eval_aligned.csv`、`phase2_summary.json` 都存在；
- 当前 `constant_current` baseline 的结果被保存，后续任何改动都能对比。

---

## Phase B：先接入已有 `PredictedAraimComputer`，替换 `constant_current`

目标：不做新数学，不做 grid，不做 LiDAR。先把当前已有 geometry-only ARAIM predictor 接到 demo10 evaluator。

### B1. 扩展 launch 参数

在 `demo10_ego_planner_pi_lite_eval.launch.py` 中允许：

```text
phase2_pl_model:=constant_current
phase2_pl_model:=gnss_geometry_araim
phase2_pl_model:=gnss_geometry_araim_fallback_current
```

建议 fallback 语义：

| 模式 | 行为 |
|---|---|
| `constant_current` | 完全保持旧行为 |
| `gnss_geometry_araim` | GNSS 预测失败时输出 conservative fallback |
| `gnss_geometry_araim_fallback_current` | GNSS 预测失败时回退 current HPL/VPL |

### B2. 扩展 `PredictedAraimComputer` 接口

当前接口：

```cpp
double predict_araim_pl(const Eigen::Vector3d& pos_world) const;
```

改为保留旧接口，同时新增：

```cpp
PredictedAraimResult predict_araim_result(const Eigen::Vector3d& pos_world) const;
```

旧接口调用新接口：

```cpp
double PredictedAraimComputer::predict_araim_pl(
    const Eigen::Vector3d& pos_world) const {
  return predict_araim_result(pos_world).hpl;
}
```

### B3. 输出字段扩展

`integrity_along_planner_traj.csv` 增加：

```text
pl_model,
hpl_pred,vpl_pred,pl_pred_scalar,
pl_ff_h,pl_ff_v,
sigma_h,sigma_v,
n_vis,pdop,n_hypotheses,
valid,fallback,fallback_reason
```

### B4. 第一版 HPL/VPL 处理

如果 `AraimResult` 当前主要是 horizontal PL，第一版可以这样处理：

```cpp
result.hpl = ar.pl_araim;
result.vpl = std::max(snapshot.vpl_now, vertical_scale * ar.pl_araim);
```

但这只是临时 bridge。真正应尽快让 `Araim::predict_geometry()` 返回 ENU/vertical 方向结果。

### B5. 测试命令

```bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=gnss_geometry_araim \
  phase2_al_model:=cloud_clearance
```

### B6. Definition of Done

- demo10 仍然 read-only；
- CSV 中 `n_vis/pdop/hpl_pred` 随轨迹点变化；
- open-sky 场景下 `hpl_pred` 平滑、无大范围 fallback；
- SkyMask 场景下遮挡区域 `n_vis` 降低、`hpl_pred` 增大；
- fallback rate < 5%（open-sky）或可解释（SkyMask）。

---

## Phase C：实现 `IntegritySnapshot`

目标：把散落的 current ARAIM / GNSS / LiDAR / AL 信息统一成 predictor 的输入。

### C1. Snapshot 数据来源

| Snapshot 字段 | 当前来源 |
|---|---|
| `p_wb/q_wb` | `/drone_0_visual_slam/odom` 或 estimator internal state |
| `hpl_now/vpl_now/hal_now/val_now/im_now` | `/iap/integrity` (`IntegrityReport.msg`) |
| `n_sv_used/pdop/excluded_prns` | `/iap/integrity` + GNSS debug |
| `gnss_epoch` | 当前 `gnss_extension` / `GnssHandler` 内部 latest epoch |
| `Lambda_base_pos` | `FGOInformationManager` 或 `Sigma_pos_now.inverse()` fallback |
| `lidar_snapshot` | `LidarAraimSnapshot` |
| `lidar_araim_result` | 当前 `LidarAraim::run()` result |
| `hal/val/al` | `DynamicALResult.msg` 或 evaluator 内 cloud clearance |

### C2. Snapshot builder

新增一个 helper：

```cpp
class IntegritySnapshotBuilder {
public:
  IntegritySnapshot build_from_latest(
      const nav_msgs::msg::Odometry& odom,
      const iap_msgs::msg::IntegrityReport& report,
      const GnssEpoch* epoch,
      const LidarAraimSnapshot* lidar_snapshot,
      const LidarAraimResult* lidar_result,
      const Eigen::Matrix3d* lambda_base_pos);
};
```

如果 demo10 evaluator 是 Python 节点，先做 Python 版 `SnapshotDict` 也可以，但 C++ 核心最终必须有一致结构。

### C3. Snapshot debug log

每次 snapshot 更新写一行：

```text
stamp,p_x,p_y,p_z,hpl_now,vpl_now,hal_now,val_now,im_now,
n_sv_used,pdop,has_epoch,has_lambda_base,has_lidar_snapshot,
lidar_alpha,n_lidar_hyp,n_lidar_detected
```

建议输出：

```text
export/future_integrity_snapshot.csv
```

### C4. Definition of Done

- 每个 `/iap/integrity` epoch 对应一个 snapshot；
- snapshot 中 `hpl_now/vpl_now/im_now` 与 `iap_araim.csv` 对齐；
- 没有 GNSS epoch 时 predictor 明确 fallback，不 silent success；
- 没有 LiDAR snapshot 时 `use_lidar_observability=false` 或 `alpha_L=0`，不影响 GNSS-only predictor。

---

## Phase D：实现 PL Grid Cache

目标：planner/evaluator 不在每个 trajectory sample 上完整重算 ARAIM，而是查询局部 PL field。

### D1. `PLGrid` 数据结构

```cpp
struct PLGridCell {
  FuturePLQueryResult value;
};

class PLGrid {
public:
  bool reset(const Eigen::Vector3d& center,
             double sx, double sy, double sz,
             double res);

  bool contains(const Eigen::Vector3d& p) const;
  PLGridCell& at(int ix, int iy, int iz);
  const PLGridCell& at(int ix, int iy, int iz) const;

  FuturePLQueryResult interpolate(const Eigen::Vector3d& p) const;
  void compute_gradients();
};
```

### D2. Grid 参数建议

```json
{
  "future_pl_grid": {
    "enabled": true,
    "resolution_m": 1.0,
    "size_x_m": 30.0,
    "size_y_m": 30.0,
    "size_z_m": 8.0,
    "update_hz": 2.0,
    "center_mode": "current_odom",
    "rebuild_distance_threshold_m": 2.0
  }
}
```

### D3. Double-buffer 机制

```cpp
std::shared_ptr<PLGrid> active_grid_;
std::shared_ptr<PLGrid> building_grid_;
std::mutex grid_mutex_;
```

更新：

```cpp
void FuturePLFieldPredictor::rebuild_grid() {
  IntegritySnapshot snap;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snap = snapshot_;
  }

  PLGrid grid;
  grid.reset(snap.p_wb, sx, sy, sz, res);

  parallel_for_each_cell([&](Eigen::Vector3d p) {
    grid.at(...).value = evaluate_point_direct(p, snap);
  });

  grid.compute_gradients();

  std::lock_guard<std::mutex> lock(grid_mutex_);
  active_grid_ = std::make_shared<PLGrid>(std::move(grid));
}
```

查询：

```cpp
FuturePLQueryResult FuturePLFieldPredictor::query(const Eigen::Vector3d& p) const {
  std::shared_ptr<PLGrid> grid;
  {
    std::lock_guard<std::mutex> lock(grid_mutex_);
    grid = active_grid_;
  }
  if (grid && grid->contains(p)) {
    return grid->interpolate(p);
  }
  return evaluate_point(p);
}
```

### D4. Definition of Done

- grid 更新线程 2 Hz 稳定运行；
- query 线程无锁或短锁，不阻塞 planner/evaluator；
- `query(p_now)` 与 direct evaluate 差异可控；
- grid 外点明确 fallback 或 direct evaluate；
- CSV 中记录 `query_source=grid/direct/fallback`。

---

## Phase E：GNSS ARAIM-style predictor 完整化

目标：让 GNSS future predictor 不只是 DOP proxy，而是沿用当前 `Araim` 的 solution separation 结构。

### E1. 扩展 `AraimResult`

如果当前 `AraimResult` 没有以下字段，建议补充：

```cpp
struct AraimResult {
  bool valid;
  double hpl;
  double vpl;
  double pl_e;
  double pl_n;
  double pl_u;
  double pl_ff_h;
  double pl_ff_v;
  double sigma_h;
  double sigma_v;
  double pdop;
  int n_sats;
  int n_hypotheses;
  std::vector<AraimSubsetDebug> subsets;
};
```

### E2. `predict_geometry()` 保持 r=0，但返回完整 debug

`predict_geometry()` 中 separation vector `d_k=0` 是合理的 geometry-only planning mode；但仍应返回每个 subset 的 `sigma_ss` 和 `sigma_k`，这样后续可以加 conservative bias。

### E3. 单星故障和星座故障

第一版：只保留单星 fault。

第二版：按 `constellation_id` 增加 constellation-level fault。

```text
H0: fault-free
H_sat_i: single satellite i removed/downweighted
H_const_c: all sats in constellation c removed/downweighted
```

### E4. Failure/fallback 规则

| 条件 | 行为 |
|---|---|
| `epoch == nullptr` | fallback，reason=`no_gnss_epoch` |
| `visible_sats < 4` | fallback，reason=`too_few_sats` |
| matrix singular | fallback，reason=`singular_geometry` |
| all sats excluded | fallback，reason=`all_excluded` |
| huge PL > cap | clamp + mark fallback/invalid depending config |

### E5. Definition of Done

- `predict_geometry()` 返回 HPL/VPL，不只 scalar；
- `n_hypotheses` 与 visible sats 数一致；
- SkyMask 场景下 `n_vis` 降低时 HPL/VPL 增大；
- 单星故障注入场景下 current ARAIM 有反应，future predictor 至少不会低估 current HPL。

---

## Phase F：LiDAR observability predictor

目标：给 future PL 一个 LiDAR 几何项，但不伪造 future VGICP ARAIM。

### F1. 第一版：基于局部点云/occupancy 的几何丰富度 proxy

如果暂时没有 trunk map / surfel map，先做 proxy：

```text
query p 附近 R 米内局部点云/voxel
  -> 统计点数、方向分布、局部 covariance eigenvalues
  -> 估计 degeneracy / condition number
  -> 得到 lidar_alpha 和 tdop_proxy
```

输出：

```cpp
struct LidarObservabilityResult {
  bool valid = false;
  Eigen::Matrix3d DeltaLambda = Eigen::Matrix3d::Zero();
  double tdop = 1e9;
  double alpha = 0.0;
  double bias_h = 0.0;
  double bias_v = 0.0;
  int n_primitives = 0;
};
```

### F2. 第二版：基于 trunk / landmark / surfel primitives

推荐优先级：

1. 已有 trunk map：用 trunk bearing/range 几何；
2. 有 surfel/submap：用 plane normal 信息；
3. 只有 occupancy：用近邻方向分布 proxy。

### F3. 当前 LiDAR ARAIM 调制

从 `LidarAraimResult` 中计算：

```cpp
double compute_lidar_alpha(const LidarAraimResult& r) {
  if (!r.valid) return 0.0;
  double penalty = 0.0;
  penalty += w_detect * r.n_detected;
  penalty += w_hyp * std::max(0, r.n_hypotheses - hyp_ref);
  // 可进一步读取 worst_mode、subsets、gamma_lidar 等
  return clamp(1.0 / (1.0 + penalty), alpha_min, alpha_max);
}
```

### F4. LiDAR 信息融合

```cpp
Lambda_pred = Lambda_base;
Lambda_pred += DeltaLambdaGNSS;
if (use_lidar_observability && lidar.valid) {
  Lambda_pred += lidar.alpha * params.lidar_info_scale * lidar.DeltaLambda;
}
Sigma_pred = inverse_with_damping(Lambda_pred);
```

### F5. PL 计算

第一版：

```cpp
sigma_h = sqrt(max_eigenvalue(Sigma_pred.block<2,2>(0,0)));
sigma_v = sqrt(Sigma_pred(2,2));
hpl = K_h * sigma_h + lidar.bias_h;
vpl = K_v * sigma_v + lidar.bias_v;
```

第二版：

- GNSS 用 ARAIM-style PL；
- LiDAR 用 conservative bias overbound；
- fused result 取 max 或 conservative fusion，不要简单加权平均。

推荐：

```cpp
hpl = std::max(hpl_gnss_araim, hpl_fused_cov + lidar.bias_h);
vpl = std::max(vpl_gnss_araim, vpl_fused_cov + lidar.bias_v);
```

### F6. Definition of Done

- `tdop`/`lidar_alpha` 随局部几何变化；
- LiDAR 退化时 `hpl/vpl` 不会变小；
- 当前 LiDAR ARAIM 检测到可疑时，未来 LiDAR 信息被降权；
- 关闭 `use_lidar_observability` 后 GNSS-only 结果保持一致。

---

## Phase G：AL / IM 统一

### G1. AL 计算

继续沿用 demo10 的 `phase2_al_model:=cloud_clearance`：

```math
AL_H(p) = \gamma_H \max(d_{obs}(p) - r_{drone} - b_{safe}, 0)
```

```math
AL_V(p) = \gamma_V \max(\min(z-z_{min}, z_{max}-z), 0)
```

```math
AL(p) = \min(AL_H(p), AL_V(p))
```

但建议 CSV 分开输出：

```text
hal_pred,val_pred,al_pred,hpl_pred,vpl_pred,im_h_pred,im_v_pred,im_min_pred
```

### G2. IM 计算

```math
IM_H(p) = HAL(p) - HPL(p)
```

```math
IM_V(p) = VAL(p) - VPL(p)
```

```math
IM_{min}(p) = \min(IM_H(p), IM_V(p))
```

### G3. PI cost wrapper

Stage 1 read-only evaluator 先只输出，不接 planner。等 predictor 通过后，planner cost 用：

```math
J_{integrity}(p)
=
w_H\max(0, HPL(p)-HAL(p))^2
+
w_V\max(0, VPL(p)-VAL(p))^2
```

或：

```math
J_{integrity}(p)
=
w\max(0, -IM_{min}(p))^2
```

### G4. Definition of Done

- `IM_pred` 明确区分 horizontal 和 vertical；
- 轨迹靠近障碍物时 `HAL` 下降，`IM_H` 下降；
- 高度接近上下界时 `VAL` 下降，`IM_V` 下降；
- PL 变差和 AL 变小都能让 `IM_min` 变差。

---

## Phase H：验证脚本扩展

### H1. 扩展 `validate_phase2_integrity_eval.py`

新增检查项：

```text
1. CSV required columns exist
2. hpl_pred/vpl_pred finite and non-negative
3. al_pred finite and non-negative
4. im_min_pred = min(hal-hpl, val-vpl) within tolerance
5. fallback_rate below threshold
6. n_vis/pdop available when pl_model contains gnss
7. PL_pred(now) vs /iap/integrity current PL consistency
8. no planner/control topic uses truth odom in official mode
```

### H2. 扩展 `analyze_phase2_integrity_eval.py`

新增 plots：

```text
figs/future_hpl_vpl_al_im_timeline.svg
figs/future_pl_vs_nvis_scatter.svg
figs/future_pl_vs_pdop_scatter.svg
figs/future_im_trajectory_xy.svg
figs/fallback_reason_histogram.svg
figs/grid_update_timing.svg
```

### H3. 场景测试矩阵

| 场景 | 命令/配置 | 预期 |
|---|---|---|
| open sky | `demo7_open_sky.yaml` | `n_vis` 高，PL 平滑，fallback 少 |
| SkyMask/NLOS | `demo7_skymask_nlos.yaml` | 遮挡区 `n_vis` 降，PDOP/PL 升 |
| single fault | `demo7_fault_injection.yaml` | current ARAIM 有检测/排除，future 不乐观 |
| obstacle close | dense forest / low clearance | AL 降，IM 降 |
| LiDAR degraded | 人为降低 inlier/提高 gamma | `alpha_L` 降，LiDAR 不再降低 PL |

### H4. Definition of Done

- 每个场景至少跑 60 秒；
- open sky fallback rate < 5%；
- SkyMask 场景 PL 与 `n_vis/pdop` 有明显相关性；
- LiDAR 关闭/退化时 fused predictor 不比 GNSS-only 更乐观；
- 报告中能看出 `constant_current`、`gnss_geometry_araim`、`fused_fim_grid` 的差异。

---

## 6. 配置建议

新增 `config/sim_demo10_integrity_predictor/config_integrity_predictor.json`：

```json
{
  "future_integrity_predictor": {
    "enabled": true,
    "mode": "gnss_geometry_araim",
    "fallback_hpl_m": 20.0,
    "fallback_vpl_m": 20.0,
    "calibrate_to_current_pl": true,

    "grid": {
      "enabled": false,
      "resolution_m": 1.0,
      "size_x_m": 30.0,
      "size_y_m": 30.0,
      "size_z_m": 8.0,
      "update_hz": 2.0
    },

    "gnss": {
      "enabled": true,
      "min_sats": 4,
      "use_map_occlusion": true,
      "use_elevation_sigma": true,
      "include_single_sat_faults": true,
      "include_constellation_faults": false
    },

    "lidar": {
      "enabled": false,
      "mode": "observability_proxy",
      "info_scale": 1.0,
      "alpha_min": 0.1,
      "alpha_max": 1.0,
      "use_current_lidar_araim_modulation": true
    },

    "debug": {
      "export_snapshot_csv": true,
      "export_grid_summary_csv": true,
      "export_query_debug": true
    }
  }
}
```

Launch 参数映射：

```python
DeclareLaunchArgument('phase2_pl_model', default_value='gnss_geometry_araim')
DeclareLaunchArgument('phase2_use_pl_grid', default_value='false')
DeclareLaunchArgument('phase2_use_lidar_observability', default_value='false')
DeclareLaunchArgument('phase2_pl_grid_resolution', default_value='1.0')
DeclareLaunchArgument('phase2_pl_grid_update_hz', default_value='2.0')
```

---

## 7. Commit 切分建议

### Commit 1 — Add future integrity data structures

改动：

```text
include/iap/planner/integrity_snapshot.hpp
include/iap/planner/future_pl_query_result.hpp
CMakeLists.txt
```

验收：能编译，无功能变化。

### Commit 2 — Extend PredictedAraimComputer result interface

改动：

```text
include/iap/planner/predicted_araim.hpp
src/iap/planner/predicted_araim.cpp
```

验收：旧 `predict_araim_pl()` 不破坏，新 `predict_araim_result()` 返回 debug fields。

### Commit 3 — Wire GNSS geometry ARAIM into demo10 evaluator

改动：

```text
launch/demo10_ego_planner_pi_lite_eval.launch.py
phase2_planner_integrity_evaluator 所在文件
tools/phase2/analyze_phase2_integrity_eval.py
tools/phase2/validate_phase2_integrity_eval.py
```

验收：`phase2_pl_model:=gnss_geometry_araim` 能运行并导出扩展 CSV。

### Commit 4 — Add PLGrid and query interpolation

改动：

```text
include/iap/planner/pl_grid.hpp
src/iap/planner/pl_grid.cpp
include/iap/planner/future_pl_field_predictor.hpp
src/iap/planner/future_pl_field_predictor.cpp
```

验收：direct query 与 grid query 一致；grid 外 fallback 明确。

### Commit 5 — Add IntegritySnapshot builder and self-consistency checks

改动：

```text
include/iap/planner/integrity_snapshot_builder.hpp
src/iap/planner/integrity_snapshot_builder.cpp
export/future_integrity_snapshot.csv writing path
```

验收：`PL_pred(p_now)` 与 current `/iap/integrity` 对齐并输出 consistency ratio。

### Commit 6 — Add LiDAR observability predictor

改动：

```text
include/iap/planner/lidar_observability_fim.hpp
src/iap/planner/lidar_observability_fim.cpp
future_pl_field_predictor.cpp
```

验收：`phase2_use_lidar_observability:=true` 时有 `tdop/lidar_alpha` 输出；关闭时 GNSS-only 不变。

### Commit 7 — Add fused FIM grid mode

改动：

```text
future_pl_field_predictor.cpp
config/sim_demo10_integrity_predictor/*.json
analysis/validation scripts
```

验收：`phase2_pl_model:=fused_fim_grid` 可运行，输出 `hpl/vpl/im/n_vis/pdop/tdop/lidar_alpha`。

---

## 8. 最小可运行版本路线图

### Week 1：GNSS geometry ARAIM 接入 demo10

目标：把 `constant_current` 换成 `gnss_geometry_araim`。

交付：

- `PredictedAraimResult`；
- demo10 新 `phase2_pl_model`；
- 扩展 CSV；
- open-sky + SkyMask 两组对比报告。

### Week 2：IntegritySnapshot + consistency check

目标：所有 predictor 输入统一来自 snapshot。

交付：

- `IntegritySnapshot`；
- `future_integrity_snapshot.csv`；
- `PL_pred(p_now)` consistency report。

### Week 3：PLGrid

目标：低频更新 PL field，高频 query。

交付：

- `PLGrid`；
- direct vs grid query 测试；
- grid update timing。

### Week 4：LiDAR observability proxy

目标：先实现 LiDAR 几何信息 proxy，不做 future block ARAIM。

交付：

- `LidarObservabilityResult`；
- `tdop/lidar_alpha` 输出；
- LiDAR enable/disable ablation。

### Week 5：fused FIM + validation

目标：形成 Stage 1 推荐版 predictor。

交付：

- `fused_fim_grid` 模式；
- validation scripts；
- open-sky / SkyMask / fault / dense obstacle / LiDAR degraded 五类测试。

### Week 6：准备 planner cost 接入

目标：保持 demo10 read-only 通过后，准备 demo11 或 planner branch。

交付：

- `pi_cost_adapter.hpp/cpp`；
- cost/gradient query 接口；
- 不改 planner optimizer 的 dry-run cost log。

---

## 9. 验收指标

### 9.1 正确性指标

| 指标 | 初始门槛 | 目标门槛 |
|---|---:|---:|
| open-sky fallback rate | < 10% | < 5% |
| `PL_pred(p_now)` vs current HPL 偏差 | < 10% | < 3% |
| NaN/Inf count | 0 | 0 |
| `IM_min = min(HAL-HPL, VAL-VPL)` 误差 | < 1e-6 | < 1e-9 |
| `n_vis` 与 SkyMask 场景响应 | 可见 | 明显 |
| LiDAR degraded 时 fused PL 是否不乐观 | 必须 | 必须 |

### 9.2 实时性指标

| 模块 | 初始门槛 | 目标门槛 |
|---|---:|---:|
| direct query | < 5 ms/query | < 1 ms/query |
| grid query | < 0.1 ms/query | < 0.05 ms/query |
| grid rebuild | < 500 ms | < 200 ms |
| grid update rate | 1 Hz | 2–5 Hz |
| demo10 60s run drop/crash | 0 | 0 |

### 9.3 科学性指标

- open-sky：PL field 平滑、近似各向同性；
- SkyMask/NLOS：遮挡方向或卫星少的位置 PL 增大；
- obstacle close：AL 降低导致 IM 降低；
- LiDAR degeneracy：TDOP/condition proxy 高时，LiDAR 不应降低 PL；
- fault injection：current ARAIM 检测/排除时，future predictor 不应比 normal 场景更乐观。

---

## 10. 常见错误与排查

### 10.1 `n_vis` 一直为 0

检查：

```bash
ros2 topic hz /ublox_driver/range_meas
ros2 topic hz /ublox_driver/ephem
ros2 topic echo /gnss_sim/diagnostics
```

可能原因：

- GNSS sim 未启动；
- `gnss_time_source:=trigger_topic` 的 trigger topic 没有发布；
- epoch 没有传到 predictor；
- 所有卫星都被 excluded；
- map occlusion 过于保守。

### 10.2 `PL_pred` 全部等于 fallback

检查 CSV 中 `fallback_reason`：

| reason | 处理 |
|---|---|
| `no_gnss_epoch` | 检查 GNSS epoch bridge |
| `too_few_sats` | 检查 SkyMask/occlusion/elevation mask |
| `singular_geometry` | 检查卫星分布或 matrix damping |
| `outside_grid` | 增大 grid 或允许 direct query |

### 10.3 `PL_pred` 比 current ARAIM 小很多

处理：

1. 启用 `calibrate_to_current_pl`；
2. 检查 `excluded_prns` 是否在 prediction 中生效；
3. 检查 GNSS `sigma_eff` 是否低估；
4. 检查 current ARAIM 是否包含 LiDAR/GNSS fusion，而 predictor 只用了 GNSS；
5. 临时用：

```cpp
hpl_pred = std::max(hpl_pred, safety_floor_ratio * snapshot.hpl_now);
vpl_pred = std::max(vpl_pred, safety_floor_ratio * snapshot.vpl_now);
```

`safety_floor_ratio` 可先设 `0.8`，逐步移除。

### 10.4 LiDAR 一打开 PL 反而异常变小

处理：

- 检查 `alpha_L` 是否被 clamp；
- 检查 `DeltaLambdaLiDAR` 是否过大；
- 给 LiDAR 信息加上 upper bound；
- 使用 conservative fusion：`max(gnss_araim_pl, fused_cov_pl + lidar_bias)`，不要直接让 LiDAR covariance 把 GNSS ARAIM PL 压得过低。

### 10.5 Planner/evaluator 卡顿

处理：

- 先关闭 grid，测 direct query；
- 再开启 grid，降低 `grid_size` 或增大 `resolution`；
- 用 OpenMP 并行 grid cells；
- 降低 `grid_update_hz`；
- planner/evaluator query 只读 active grid，禁止在 planner callback 中 rebuild grid。

---

## 11. 推荐 Codex 执行 prompt

```text
You are working in the Printeger/iap repository on branch dev/iap.

Goal: implement Stage-1 Future ARAIM / PL Field Predictor for demo10 without changing estimator, current ARAIM monitor, controller, or EGO planner behavior.

Requirements:
1. Keep demo10 read-only. Do not feed predicted PL back into planner cost yet.
2. Preserve existing phase2_pl_model:=constant_current behavior.
3. Add phase2_pl_model:=gnss_geometry_araim using the existing PredictedAraimComputer / VisibilityPredictor / Araim::predict_geometry pipeline.
4. Extend PredictedAraimComputer with a result struct that returns validity, HPL, VPL if available, PL scalar, n_vis, pdop, n_hypotheses, sigma fields, and fallback_reason. Keep the old predict_araim_pl() wrapper for compatibility.
5. Add IntegritySnapshot and FuturePLQueryResult structs under include/iap/planner/.
6. Extend demo10 output CSV with hpl_pred, vpl_pred, im_h_pred, im_v_pred, im_min_pred, n_vis, pdop, valid, fallback, fallback_reason, query_source.
7. Extend phase2 analysis and validation scripts to check the new fields.
8. Do not implement LiDAR future block-level ARAIM. Add TODO hooks only.
9. Add tests or smoke checks where possible.
10. Provide a concise implementation plan before editing, then implement in small commits.

Acceptance:
- constant_current still produces the old outputs.
- gnss_geometry_araim runs in demo10 for 60 seconds.
- open-sky has low fallback rate.
- SkyMask/NLOS scenario changes n_vis/pdop/HPL in the expected direction.
- No NaN/Inf in exported CSV.
```

---

## 12. 最终推荐版本定义

Stage 1 的 ARAIM 预测模块最终应定义为：

```text
A geometry-driven, ARAIM-structured, queryable future integrity field.
```

它的输入不是简单 `pose + current PL`，而是：

```text
IntegritySnapshot = current estimator information + current ARAIM result + GNSS epoch + LiDAR ARAIM reliability + map visibility support
```

它的输出不是一个 scalar `PL_pred`，而是：

```text
query(p) -> HPL, VPL, IR, HAL, VAL, IM_H, IM_V, IM_min, gradients, validity, debug
```

它的工程形态不是“planner 每次 query 都重算 ARAIM”，而是：

```text
low-rate field rebuild + high-rate trilinear query
```

它的 LiDAR 处理不是“虚拟 future source/target/level block ARAIM”，而是：

```text
current LiDAR block-level ARAIM -> future LiDAR observability confidence modulation
```

---

## 13. Source notes

This handbook was written against the current public `Printeger/iap` `dev/iap` repository as viewed on 2026-05-06, especially the README descriptions of demo7/demo8/demo9/demo10, current message definitions, and current C++ headers for `Araim`, `PredictedAraimComputer`, `PredictedIntegrityComputer`, and `LidarAraim`. It also follows the Stage 1 design constraints from the uploaded PI/PL design notes: keep Stage 1 decoupled, use PI as planner-side cost adapter, make future predictor advisory, use an integrity snapshot, prefer `update(snapshot)` / `query(p)`, use local PL grid interpolation, and treat future LiDAR as observability/FIM rather than virtual VGICP block ARAIM.

---

## 14. Phase A/B/E 落地计划：demo10 GNSS Geometry ARAIM

### 14.1 Summary

- 本轮不改写原 Phase A-H 描述，只追加本章节作为 demo10 第一轮落地计划。
- 第一轮范围固定为：Phase A baseline、Phase B 接入 GNSS geometry ARAIM、Phase E-lite 单星 solution-separation 输出完整化。
- 采用全 C++ demo10 evaluator：新增 `iap phase2_planner_integrity_evaluator`，不做 Python-to-C++ service，不重写 Python ARAIM 数学。
- 不做 Phase C/D/F：不引入 `IntegritySnapshot`、PL grid、LiDAR observability、planner cost 回写。

### 14.2 Phase A baseline 记录

代码改动前已运行 `constant_current` demo10 60s：

```bash
source install/setup.bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false run_duration_s:=60 allow_truth_alignment:=false \
  use_so3_dynamics:=true use_gnss:=true use_araim:=true \
  phase2_pl_model:=constant_current phase2_al_model:=cloud_clearance
```

Baseline run dir:

```text
src/iap/log/20260505T165429Z_061
src/iap/log/baseline_demo10_constant_current
```

Baseline metrics:

- CSV rows / sample count: `1102`
- trajectory count: `89`
- aligned sample count: `1102`
- AL min / mean: `0.000000 / 0.541966`
- PL min / mean / max: `47.279747 / 54.476647 / 79.952657`
- IM min / mean: `-78.752639 / -53.934681`
- analyzer: passed, with `pos_cmd fallback` warning
- validator: passed, with `pos_cmd fallback` and mostly unsafe predicted IM warnings

### 14.3 Key changes

`PredictedAraimComputer`:

- 新增 `PredictedAraimResult`，字段固定为 `valid/fallback/fallback_reason/hpl/vpl/pl_scalar/pl_e/pl_n/pl_u/pl_ff_h/pl_ff_v/sigma_h/sigma_v/pdop/n_vis/n_hypotheses`。
- 新主接口为 `predict_araim_result(pos)`；旧 `predict_araim_pl(pos)` 保留并返回 `result.hpl`。
- `epoch == nullptr` 才是 `no_gnss_epoch` fallback；`occupancy == nullptr` 按 open-sky 预测，不再直接 fallback。
- `pdop = sqrt(S0(0,0)+S0(1,1)+S0(2,2))`。
- `sigma_h = sqrt(sigma_ff_E^2 + sigma_ff_N^2)`，`sigma_v = sigma_ff_U`。

C++ evaluator:

- 新 executable：`iap phase2_planner_integrity_evaluator`；demo10 launch 调用这个 C++ 节点。
- 保留 Python evaluator 文件作为历史实现，不删除。
- C++ evaluator 复刻现有 Python evaluator 行为：订阅 odom、B-spline、pos_cmd fallback、map cloud、`/iap/integrity`，输出同名 CSV、summary JSON、可选 markers。
- 额外订阅 GNSS topics：`/ublox_driver/range_meas`、`/ublox_driver/ephem`、`/ublox_driver/glo_ephem`、`/ublox_driver/receiver_lla`、`/ublox_driver/iono_params`，内部构建 latest `GnssEpoch`。
- map cloud 同时用于原 `cloud_clearance` AL 计算，以及构建 `LocalOccupancyGrid` 给 `VisibilityPredictor` ray-cast 使用。
- 默认假设 map topic 坐标与 planner/world frame 一致，和现有 Python evaluator 保持一致。

demo10 PL modes:

- `constant_current`：保持原行为，`PL_H_pred=current_HPL`，`PL_V_pred=current_VPL`，`PL_pred=max(...)`。
- `gnss_geometry_araim`：调用 `PredictedAraimComputer::predict_araim_result()`；失败时输出 conservative fallback，默认 `phase2_fallback_pl_m=20.0`。
- `gnss_geometry_araim_fallback_current`：GNSS 预测失败时优先回退 current HPL/VPL；current 不可用时使用 `20.0m` fallback。
- `query_source` 固定为 `current/direct/fallback` 三类。

CSV / summary / validation:

- 现有 CSV 字段全部保留。
- 新增 debug/analysis 字段：`hpl_pred,vpl_pred,pl_pred_scalar,pl_ff_h,pl_ff_v,sigma_h,sigma_v,n_vis,pdop,n_hypotheses,valid,fallback,fallback_reason,query_source`。
- `PL_H_pred/PL_V_pred/PL_pred/IM_*` 继续作为官方字段；新增 lowercase 字段只做 debug/analysis。
- `phase2_summary.json` 增加 fallback count/rate、fallback reason histogram、finite GNSS prediction count。
- analyzer/validator 检查新增列、finite PL/IM、GNSS 模式下 `n_vis/pdop` 可用、fallback rate，并校验 `IM_pred_axis_min = min(AL_H-HPL, AL_V-VPL)`。

### 14.4 Implementation steps

1. Phase A baseline：
   - 在任何代码改动前运行 `constant_current` demo10 60s。
   - 跑 analyzer/validator。
   - 保存 `src/iap/log/latest` 到 `src/iap/log/baseline_demo10_constant_current`。
   - 记录 CSV 行数、sample count、AL/PL/IM 统计和 validator 结果。

2. C++ core：
   - 修改 `predicted_araim.hpp/cpp`，加入 result struct 与新接口。
   - 添加 gtest 覆盖 valid geometry、too-few-sats fallback、旧 wrapper 兼容、missing epoch fallback。
   - 不改变 `Araim::predict_geometry()` 的数学；Phase E-lite 只消费已有 single-sat `AraimResult` 字段。

3. C++ evaluator：
   - 新增 `apps/phase2_planner_integrity_evaluator.cpp`。
   - 在 `iap` CMake/package 增加 `traj_utils`、`quadrotor_msgs`、`gnss_comm` 等 ROS2 依赖，并安装 executable 到 `lib/iap`。
   - 更新 demo10 launch：Node package 从 `iap_phase1_tools` 切到 `iap`，参数名保持兼容，新增 GNSS topic/fallback 参数。
   - 更新 build script executable check，确认 `iap phase2_planner_integrity_evaluator` 存在。

4. Analysis/doc：
   - 更新 `tools/phase2/analyze_phase2_integrity_eval.py` 和 validator 以识别新字段。
   - 本章节明确本轮采用全 C++ evaluator 与 E-lite 单星范围。

### 14.5 Test plan

Build:

```bash
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim"
```

Regression:

- `phase2_pl_model:=constant_current` 跑 60s，CSV 旧字段语义与 Phase A baseline 一致。
- `phase2_pl_model:=gnss_geometry_araim` 跑 60s，`hpl_pred/vpl_pred/n_vis/pdop` 有 finite 值，open-sky fallback rate 目标 `< 5%`。
- `phase2_pl_model:=gnss_geometry_araim_fallback_current` 人为缺 GNSS epoch 时 CSV 仍 finite，fallback reason 为 `no_gnss_epoch`。

Scenario checks:

- open-sky：PL 平滑，`n_vis` 稳定，validator 通过。
- SkyMask/NLOS：`n_vis` 下降或 `pdop/hpl_pred` 上升趋势可见。
- single fault：current ARAIM 可检测/排除；future predictor 不声称 certified，只保证不会 silent success 或 NaN。

### 14.6 Assumptions

- Phase E 本轮只做 E-lite：复用已有 single-sat hypotheses，不加入 constellation-level hypotheses。
- C++ evaluator 允许复制现有 Python evaluator 的 B-spline De Boor、AL、summary 逻辑；Python evaluator 保留作为历史实现。
- `map_topic` 点云坐标沿用现有 demo10 假设：可直接与 planner/world positions 比较。
- 本轮不接 planner cost，不修改 estimator、current ARAIM monitor、controller 或 EGO planner optimizer。

---

## 15. Phase C/H 落地计划：IntegritySnapshot + H-lite Validation

### 15.1 Summary

- 本轮实现 Phase C core + demo10 evaluator 接入，并实现 Phase H 的 C-ready H-lite。
- 不改写原 Phase A-H 和第 14 章；本章节作为 Phase C/H 的落地记录。
- 不要求 PLGrid、LiDAR observability、fused FIM 或 planner cost。
- 保持上一轮 A/B/E-lite 行为：`constant_current`、`gnss_geometry_araim`、`gnss_geometry_araim_fallback_current` 语义不变。

### 15.2 Key changes

Core snapshot API:

- 新增 `IntegritySnapshot`、`CurrentIntegrityState`、`IntegritySnapshotBuilder`。
- Builder 使用纯 C++ 输入，不依赖 ROS message；demo10 evaluator 负责把 `IntegrityReport.msg`、odom、latest `GnssEpoch` 映射成 builder input。
- Snapshot 冻结当前 pose、current integrity、GNSS epoch 可用性、可选 `Lambda_base_pos`、可选 LiDAR snapshot/result 状态。

demo10 evaluator:

- odom callback 保存 latest pose，不只保存 stamp。
- `/iap/integrity` callback 每个 current integrity epoch 生成一条 snapshot。
- 新增 `export/future_integrity_snapshot.csv`。
- Snapshot CSV 包含 pose、current HPL/VPL/PL/HAL/VAL/IM、`n_sv_used/pdop/n_hypotheses/n_detected/excluded_prns`、`has_epoch/epoch_sat_count`、`has_lambda_base/has_lidar_snapshot/has_lidar_araim_result`、`pred_now_*` 和 consistency debug 字段。

H-lite analyzer / validator:

- Analyzer 读取 `future_integrity_snapshot.csv`，与 `iap_araim.csv` 按时间对齐，写入 `phase2_summary.json` 的 `integrity_snapshot` 和 `current_consistency` 段。
- `PL_pred(p_now)` consistency 采用 warn-first：要求 finite 样本存在；偏差超过 10% 只 warning，不阻断 validator。
- Validator 检查 snapshot CSV 存在、行数 > 0、current integrity 字段 finite、GNSS 模式下 `pred_now_n_vis/pred_now_pdop` finite、无 GNSS epoch 时 fallback reason 明确。
- 保留已有 official checks：finite PL/IM、`IM_pred_axis_min` 公式、fallback rate、official odom source、Phase1 validator。

Scenario and plots:

- demo9/demo10 launch 增加 `gnss_scenario_file` 参数，demo10 透传到 demo9 的 `gnss_sim_node`。
- 新增 Phase2 H-lite scenario runner，覆盖 `constant_current_open_sky`、`gnss_open_sky`、`gnss_skymask_nlos`、`gnss_fault_injection`。
- Analyzer 用 stdlib 生成轻量 SVG：future timeline、PL-vs-nvis、PL-vs-pdop、XY trajectory、fallback histogram、current consistency timeline。
- `grid_update_timing.svg` 本轮不生成，summary 中记录为 `skipped_not_applicable`。

### 15.3 Implementation steps

1. Core C++:
   - 新增 snapshot header/source。
   - 新增 `test_integrity_snapshot`，覆盖 full input、missing GNSS epoch、missing LiDAR、current field copy。
   - 将 snapshot source 和 test 接入 CMake。

2. Evaluator:
   - 在 demo10 C++ evaluator 中保存 latest odom pose。
   - 将 `/iap/integrity` 消息映射为 `CurrentIntegrityState`。
   - 每个 integrity epoch 写一行 `future_integrity_snapshot.csv`。
   - 在当前位置执行 `pl_model` 对应的 `pred_now` 查询并写 consistency 字段。

3. H-lite scripts:
   - 扩展 analyzer/validator 识别 snapshot CSV 和 current consistency。
   - 生成 lightweight SVG plots。
   - 新增 scenario runner 汇总场景结果，并把 D/F/G 项显式标记为 skipped。

4. Launch:
   - demo9 hardcoded open-sky scenario 改为 `gnss_scenario_file` 参数。
   - demo10 透传同名参数，默认仍为 `demo7_open_sky.yaml`。

### 15.4 Test plan

Build/unit:

```bash
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot"
```

Regression:

- `constant_current` 60s：旧 CSV 语义不变，新增 snapshot CSV 存在。
- `gnss_geometry_araim` 60s open-sky：snapshot `pred_now_*` finite，fallback rate `< 5%`。
- 人为缺 GNSS epoch：snapshot 与 trajectory CSV 都 finite，fallback reason 为 `no_gnss_epoch`。

Scenario H-lite:

- open-sky：`n_vis/pdop/HPL` 平滑，validator 通过。
- SkyMask/NLOS：相对 open-sky，`n_vis` 下降或 `pdop/HPL` 上升趋势可见。
- fault injection：current ARAIM diagnostics 被记录；future predictor 不 silent success、不 NaN、不声称 certified FDE。
- scenario runner 输出 aggregate JSON，列出每个场景 passed/warnings/skipped_not_applicable。

### 15.5 Assumptions

- 本轮不做 Phase D/F/G：不实现 PLGrid、不实现 LiDAR observability、不实现 fused FIM、不接 planner cost。
- `IntegritySnapshot` 先服务 demo10 read-only evaluator，同时保持后续 D/F/G 可复用。
- demo10 仍以 `/drone_0_visual_slam/odom` 为 official planner/evaluator odom source，truth 只允许 offline alignment。
- `PL_pred(p_now)` 与 current `/iap/integrity` 的一致性本轮是诊断门槛：finite 必须满足，10% 偏差先 warning。

## 16. Phase D 落地计划：PLGrid Cache

### 16.1 Summary

- 本轮实现 Core PLGrid + demo10 evaluator 接入：新增可复用 PL grid / field predictor，并让 demo10 在 GNSS 模式下可选走 grid 查询。
- 保持 A/B/C/E/H-lite 行为兼容：默认 `phase2_use_pl_grid:=false`，现有 `constant_current`、`gnss_geometry_araim`、`gnss_geometry_araim_fallback_current` 不变。
- 本轮不做 Phase F/G：不加入 LiDAR observability、不做 fused FIM、不接 planner cost。

### 16.2 Key changes

Core grid API:

- 新增 `FuturePLQueryResult`，承载 `PredictedAraimResult` 等价 PL/debug 字段、`query_source`、grid generation/age、`grad_hpl/grad_vpl/grad_pl_scalar`。
- 新增 `PLGrid`：固定中心、尺寸、分辨率，保存每个 cell 的 GNSS geometry ARAIM result，并支持 trilinear interpolation。
- 新增 `FuturePLFieldPredictor`：持有 latest `IntegritySnapshot`，支持 `evaluate_point_direct()`、`query()`、后台 `rebuild_grid()`。

Grid 查询规则:

- 只有 8 个插值角点都 valid 时返回 `query_source=grid`。
- grid 未就绪、过期、越界、角点 invalid 时直接走 `evaluate_point_direct()`，返回 `query_source=direct`。
- direct 仍失败时保持现有 fallback 语义；`gnss_geometry_araim_fallback_current` 仍优先回退 current HPL/VPL。

demo10 evaluator:

- 新增 launch 参数：`phase2_use_pl_grid`、`phase2_pl_grid_resolution_m`、`phase2_pl_grid_size_x_m/y_m/z_m`、`phase2_pl_grid_update_hz`。
- evaluator 维护 grid double-buffer，后台按 `update_hz` 基于 latest `IntegritySnapshot` rebuild，不阻塞 trajectory sampling。
- CSV 现有字段保留，`query_source` 允许新增 `grid`；新增 debug 字段记录 grid enabled、generation、age、build timing。
- `phase2_summary.json` 增加 `pl_grid` 段：enabled、active、resolution、dimensions、update_count、skip_count、query counts、build time stats、grid-vs-direct self-check。

Analyzer / validator:

- validator 允许 `query_source=grid`。
- 当 `phase2_use_pl_grid=true` 时，要求至少有 grid query、finite PL/IM、finite grid stats，并检查 grid miss 不会产生 NaN。
- analyzer 生成 `figs/grid_update_timing.svg`；grid enabled 时 summary 中不再标记 grid timing 为 skipped。

### 16.3 Implementation steps

1. Core:
   - 新增 `future_pl_query_result.hpp/cpp`、`pl_grid.hpp/cpp`、`future_pl_field_predictor.hpp/cpp`。
   - `PLGrid` 实现 reset/contains/indexing、trilinear interpolation、有限差分 gradient。
   - `FuturePLFieldPredictor` 用 snapshot 冻结 GNSS epoch，grid miss 自动 direct fallback。

2. Evaluator:
   - demo10 新增 grid launch/runtime 参数，默认关闭。
   - `/iap/integrity` snapshot 更新后喂给 field predictor。
   - grid enabled 时 future trajectory 查询优先走 `FuturePLFieldPredictor::query()`；grid disabled 时保持 direct predictor 路径。
   - CSV 和 summary 写入 grid debug 字段与 `pl_grid` 统计。

3. Scripts:
   - analyzer 保留原有 H-lite plots，并在 grid enabled 时输出 `grid_update_timing.svg`。
   - validator 增加 grid enabled 分支检查。

4. Tests:
   - 新增 `test_pl_grid` 覆盖 reset/contains/indexing/interpolation/gradient/invalid corner miss。
   - 新增 `test_future_pl_field_predictor` 覆盖 inside grid、outside direct、missing GNSS epoch fallback。

### 16.4 Test plan

Build/unit:

```bash
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor"
```

Regression:

- `phase2_use_pl_grid:=false` + `gnss_geometry_araim` 60s：结果与当前 direct mode 语义一致。
- `phase2_use_pl_grid:=true` + `gnss_geometry_araim` 60s：validator 通过，`query_source=grid` 出现，fallback rate `< 5%`。
- `gnss_geometry_araim_fallback_current` + grid miss：CSV finite，fallback behavior 与 direct mode 一致。

Scenario:

- open-sky：grid build 2Hz 稳定，grid-vs-direct self-check warning threshold 10%。
- SkyMask/NLOS：grid query 下仍能看到 `n_vis/pdop/HPL` 趋势变化。
- fault injection：future grid 不声称 certified FDE，只保证无 silent success/NaN。

### 16.5 Assumptions

- Grid 默认关闭；必须显式 `phase2_use_pl_grid:=true` 才启用 Phase D runtime path。
- 第一版 grid 只缓存 GNSS geometry ARAIM PL，不缓存 AL/LiDAR/FIM。
- Grid center 使用 latest `IntegritySnapshot.p_wb`，默认尺寸 `30 x 30 x 8 m`，分辨率 `1.0 m`，更新频率 `2 Hz`。
- Grid miss 使用 direct fallback，保证 demo10 read-only evaluator 不因 grid 覆盖边界产生非有限输出。

---

## 17. Phase F 落地计划：LiDAR Observability + Conservative Fused FIM

### 17.1 Summary

- 本轮实现 Phase F-lite：新增 planner-side LiDAR observability/FIM proxy，并提供默认关闭的 `fused_fim_grid` PL mode。
- Phase F 只接入 demo10 read-only evaluator，不接 planner cost、不改 estimator、不修改 current ARAIM monitor。
- LiDAR v1 使用 demo10 map cloud 的局部几何 proxy，并用 `IntegrityReport` 中已有的 `n_trunks_observed/tdop/excluded_trunk_ids` 做 current modulation。
- 不扩展 `IntegrityReport.msg`，不触发 ROS interface rebuild 风险。
- 官方 PL 始终采用 conservative envelope：`max(GNSS ARAIM PL, fused covariance PL + LiDAR bias)`，因此 `fused_fim_grid` 不会比 GNSS-only 更乐观。

### 17.2 Key Changes

Core LiDAR observability:

- 新增 `LidarObservabilityResult` 和 `LidarObservabilityFim`。
- 输入为候选点 `p_w`、共享只读 map cloud、`IntegritySnapshot.current`。
- 输出固定包含 `valid/delta_lambda/tdop_proxy/lidar_alpha/condition/n_primitives/bias_h/bias_v/fallback_reason`。
- 点云 proxy 在 `lidar_search_radius_m` 内取邻近点，按方向单位向量累计 `DeltaLambda += w * u * u^T / sigma_lidar^2`。
- 点数不足、信息矩阵退化或参数无效时返回 `valid=false`、`lidar_alpha=0`，并写明确 fallback reason。

Future PL field predictor:

- `FuturePLQueryResult` 扩展 GNSS/fused/LiDAR debug 字段：`gnss_hpl/gnss_vpl/fused_hpl/fused_vpl/lidar_valid/lidar_alpha/lidar_tdop/lidar_condition/lidar_n_primitives/lidar_bias_h/lidar_bias_v/lidar_fallback_reason`。
- `FuturePLFieldPredictor::Params` 新增 LiDAR 参数：`use_fused_fim_grid/use_lidar_observability/lidar_search_radius_m/lidar_min_points/lidar_good_points/lidar_sigma_m/lidar_info_scale/lidar_alpha_min/lidar_alpha_max/lidar_condition_ref/lidar_condition_max/lidar_tdop_ref/lidar_tdop_max/lidar_bias_h_m/lidar_bias_v_m`。
- 新增 `set_lidar_map_points(shared_ptr<const vector<Eigen::Vector3d>>)`，用于 evaluator 从 cloud callback 更新 immutable shared cloud。
- `gnss_geometry_araim` 行为保持 GNSS-only；只有 `pl_model=fused_fim_grid` 且 `phase2_use_lidar_observability:=true` 时启用 LiDAR FIM path。
- `phase2_use_lidar_observability:=false` 时，`fused_fim_grid` 自动退化为 GNSS-only debug-safe path。

demo10 evaluator:

- map cloud callback 同时维护 AL 点云、`LocalOccupancyGrid` 和 predictor 用 shared point cloud。
- 新增 launch/runtime 参数：`phase2_use_lidar_observability` 以及 LiDAR radius/min-points/info-scale/alpha/condition/tdop/bias 参数，默认关闭或保守。
- trajectory CSV 保留既有字段，并新增 Phase F debug 列：`gnss_hpl,gnss_vpl,fused_hpl,fused_vpl,lidar_valid,lidar_alpha,lidar_tdop,lidar_condition,lidar_n_primitives,lidar_bias_h,lidar_bias_v,lidar_fallback_reason`。
- Snapshot CSV 新增 `n_trunks_observed/current_tdop/lidar_modulation_alpha`。
- `phase2_summary.json` 新增 `lidar_observability` 段，记录 enabled、valid count/rate、alpha/tdop/condition stats、fallback histogram、conservative fusion check count。

Analyzer / validator:

- Analyzer 在 LiDAR enabled 时生成 `figs/future_lidar_alpha_tdop_timeline.svg` 和 `figs/future_gnss_vs_fused_pl.svg`。
- `phase_h_lite.lidar_observability` 和 `phase_h_lite.fused_fim_grid` 在可用时指向对应图表，否则保持 `skipped_not_applicable`。
- Validator 在 LiDAR enabled 或 `fused_fim_grid` 模式下检查新增列、finite official PL/IM、finite LiDAR debug 样本、`PL_H_pred >= gnss_hpl`、`PL_V_pred >= gnss_vpl`，并检查 summary 没有 conservative fusion violation。
- LiDAR disabled 时，GNSS-only direct/grid regression 语义保持不变，新增 debug 字段只作为附加列存在。

### 17.3 Implementation Steps

1. Core:
   - 新增 `include/iap/planner/lidar_observability_fim.hpp` 与 `src/iap/planner/lidar_observability_fim.cpp`。
   - 扩展 `FuturePLQueryResult` 承载 GNSS/fused/LiDAR debug 字段。
   - 扩展 `FuturePLFieldPredictor` 的 direct 和 grid rebuild 路径，使 grid cell 可缓存 fused result 与 LiDAR debug。

2. Evaluator:
   - cloud callback 同时维护 AL 用点云和 predictor 用 shared point cloud。
   - `pl_model=fused_fim_grid` 时启用 conservative fused result。
   - `phase2_use_lidar_observability=false` 时自动退化为 GNSS-only debug-safe result。
   - Summary、trajectory CSV、snapshot CSV 写入 LiDAR observability 统计。

3. Scripts/docs:
   - analyzer/validator 识别新字段和 conservative fusion invariant。
   - 本章节记录 Phase F 采用点云+TDOP proxy、不扩展 ROS msg、不做 Phase G。

4. Build integration:
   - CMake 加入 `lidar_observability_fim.cpp`。
   - 新增 `test_lidar_observability_fim` 并纳入 `iap` gtest。
   - 保持 `IntegrityReport.msg` 不变。

### 17.4 Test Plan

Build/unit:

```bash
bash src/iap/tools/build_phase1_ego_planner_closed_loop.sh
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim"
```

Unit coverage:

- rich local point cloud produces valid `DeltaLambda`、finite `tdop_proxy`、positive `lidar_alpha`。
- too few points yields explicit fallback and `lidar_alpha=0`。
- line/plane-degenerate geometry lowers alpha or falls back relative to isotropic geometry。
- poor current `tdop` or excluded trunk ids downweight alpha。
- LiDAR disabled keeps GNSS-only official PL fields.
- fused mode never outputs official HPL/VPL lower than GNSS ARAIM HPL/VPL.

Regression:

- `gnss_geometry_araim + phase2_use_lidar_observability:=false` 60s：与 Phase D GNSS-only semantics 一致。
- `fused_fim_grid + phase2_use_lidar_observability:=true + phase2_use_pl_grid:=true` 60s：validator 通过，`lidar_alpha/tdop` finite 样本存在，fallback rate 目标 `< 5%`。
- map cloud missing/empty：CSV official PL/IM finite，LiDAR fallback reason 明确，official PL 回退 GNSS-only conservative result。

Scenario checks:

- open-sky + normal cloud：LiDAR debug 平滑，official PL 不低于 GNSS PL。
- sparse/degenerate cloud：`lidar_alpha` 下降或 fallback 增加。
- SkyMask/NLOS：GNSS `n_vis/pdop/HPL` 趋势仍可见，LiDAR debug 不掩盖 GNSS 退化。
- fault injection：current ARAIM diagnostics 保留；future fused mode 不声称 certified FDE、不 silent success、不 NaN。

### 17.5 Assumptions

- Phase F 本轮以 demo10 read-only evaluator 为唯一 runtime 接入点。
- `fused_fim_grid` 默认关闭；必须显式设置 `phase2_pl_model:=fused_fim_grid` 和 `phase2_use_lidar_observability:=true` 才启用 LiDAR path。
- 第一版 LiDAR observability 只使用 map cloud + current TDOP/trunk diagnostics，不使用真实 future VGICP blocks。
- 没有 `Lambda_base_pos` 时，用 GNSS fault-free sigma 构造 diagonal base information；official PL 仍用 `max(GNSS ARAIM, fused covariance + bias)` 保守包络。
- 本轮不做 Phase G：不把 future LiDAR 做成虚拟 VGICP block ARAIM，也不把 fused FIM 接入 planner optimizer cost。

---

## 18. Stage 1 收尾状态：Phase F Hardening、G-lite、Planner-gated Integration

### 18.1 2026-05-06 baseline

Read-only baseline:

- 命令：`ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py start_rviz:=false run_duration_s:=60 allow_truth_alignment:=false use_so3_dynamics:=true use_gnss:=true use_araim:=true phase2_pl_model:=fused_fim_grid phase2_use_lidar_observability:=true phase2_use_pl_grid:=true phase2_al_model:=cloud_clearance phase2_publish_integrity_cost_field:=true`
- Run dir：`src/iap/log/20260506T060451Z_264`
- Analyzer + validator：通过。
- `sample_count=1047`，`traj_count=70`，fallback rate `1.81%`。
- `current_consistency.max_pl_ratio=0.0`。
- PL grid build time mean `492.55 ms`，max `599.18 ms`，last `459.58 ms`。
- LiDAR valid rate `85.09%`，fallback histogram `not_evaluated=133, too_few_points=1012`，无 `unknown` fallback。
- LiDAR debug non-finite count `0`；conservative fusion violation `0`。

Planner-gated baseline:

- 命令：`ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py start_rviz:=false run_duration_s:=60 allow_truth_alignment:=false use_so3_dynamics:=true use_gnss:=true use_araim:=true phase2_pl_model:=fused_fim_grid phase2_use_lidar_observability:=true phase2_use_pl_grid:=true phase2_al_model:=cloud_clearance planner_use_integrity_cost:=true`
- Run dir：`src/iap/log/20260506T061232Z_057`
- Analyzer + validator：通过。
- `sample_count=1079`，`snapshot_count=348`，`traj_count=82`，fallback rate `1.76%`。
- `current_consistency.max_pl_ratio=0.0`。
- PL grid build time mean `451.54 ms`，max `544.88 ms`，last `439.19 ms`。
- LiDAR valid rate `95.91%`，fallback histogram `not_evaluated=133, too_few_points=190`。
- `lidar_observability.nonfinite_debug_count=0`，`conservative_fusion_violation_count=0`。
- PI cost summary：`count=1079`，mean `531.30`，p50 `534.02`，p95 `567.68`，risk histogram `UNSAFE_PI=1079`，dominant axis histogram `vertical=1079`。
- Planner opened with `optimization/use_integrity_cost=true` and default soft weight `lambda_integrity=1e-5`; planner continued publishing successful plans. One `refine_optimize` line-search warning was observed while replanning still succeeded; no crash or NaN gradient was observed.

### 18.2 Implemented close-out changes

- Phase F hardening keeps LiDAR missing/sparse/degenerate/invalid cases as explicit fallback with finite sentinels and no official PL NaN.
- `FuturePLFieldPredictor` keeps invalid LiDAR as GNSS-only conservative official output; invalid LiDAR debug fields remain finite.
- Demo10 defaults for fused LiDAR observability are conservative: radius `8.0 m`, min points `12`, good points `80`, alpha `[0.02, 1.0]`, condition max `1e6`, TDOP max `20.0`, LiDAR bias `0.0 m`.
- `PICostAdapter` provides read-only cost, risk band, dominant axis, risk band numeric code, and finite-guarded gradient fields.
- Demo10 evaluator exports `pi_cost_h/pi_cost_v/pi_cost_total/pi_risk_band/pi_risk_band_code/pi_margin_h/pi_margin_v/pi_dominant_axis/pi_grad_x/pi_grad_y/pi_grad_z` and `phase2_summary.json.pi_cost`.
- Demo10 can publish read-only `sensor_msgs/msg/PointCloud2` integrity cost field on `/iap/integrity_cost_field` with `hpl,vpl,hal,val,im_h,im_v,im_min,cost,grad_x,grad_y,grad_z,risk_band,risk_band_code`.
- Planner integration is gated by `planner_use_integrity_cost:=false` by default and forwards to EGO `optimization/use_integrity_cost=false`.
- EGO `BsplineOptimizer` only subscribes to the field when enabled; missing/stale/too-small/non-finite/invalid-risk fields return zero cost with throttled warning.
- When enabled, `calcIntegrityCost()` uses nearest fresh samples, skips `UNKNOWN_PI`, clamps sample cost and gradient, and appends the soft cost to rebound/refine without changing existing collision/smoothness/feasibility terms.

### 18.3 Verification commands

```bash
python3 -m py_compile \
  src/iap/launch/demo10_ego_planner_pi_lite_eval.launch.py \
  src/iap/launch/demo9_ego_planner_closed_loop.launch.py \
  src/iap/sim/ego_planner_swarm_ws/src/planner/plan_manage/launch/advanced_param.launch.py \
  src/iap/tools/phase2/analyze_phase2_integrity_eval.py \
  src/iap/tools/phase2/validate_phase2_integrity_eval.py

colcon build \
  --base-paths src/iap src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm \
  --packages-select iap bspline_opt ego_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon test --packages-select iap \
  --ctest-args -R "test_pi_cost_adapter|test_lidar_observability_fim|test_future_pl_field_predictor" \
  --output-on-failure

colcon test-result --verbose --test-result-base build/iap
```

Result：build 通过；`iap` test result 为 `68 tests, 0 errors, 0 failures, 0 skipped`。EGO 侧仍有既有 warning，包括 deprecated ROS aliases、unused parameters 和 VLA warning。

### 18.4 Remaining runtime matrix

当前已完成 open-sky 60s read-only baseline 与 planner-gated 60s baseline。完整 handbook 场景矩阵仍建议继续逐项落盘：

- SkyMask/NLOS：确认 `n_vis/pdop/HPL` 趋势，LiDAR debug 不掩盖 GNSS 退化。
- fault injection：确认 future predictor 不声称 certified FDE，无 silent success/NaN。
- dense/near obstacle：确认 AL/PI cost 与 dominant axis/risk band 可解释。
- missing/empty/sparse/degenerate LiDAR cloud runtime：确认 official PL/IM finite、fallback reason 明确、LiDAR debug no-NaN。
- `planner_use_integrity_cost=false` vs `true` 同场景回归：关闭时 planner trajectory/bspline/optimizer summary 只允许 timestamp/run-id 差异；开启时 field missing/stale/non-finite 自动回原 cost。
