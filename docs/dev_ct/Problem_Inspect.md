# Q1: iap是想要在GLIM的框架下修改为B-spline的连续时间估计，但是现在修改后耗时却比两者都更高了，请你帮我分析，是哪里不对劲？我想要使用GLIM 的增量 fixed-lag 组织+C-LIUO的连续时间设计。

## A1: 
我会把问题拆成两层：先判断“为什么改成 B-spline 后反而比 GLIM 和 C-LIUO 更慢”，再给一个“怎样改回你想要的形态：GLIM 的增量 fixed-lag + C-LIUO 的连续时间设计”的落地路径。重点会放在求解组织和数据结构，而不是单个 kernel。

Thought for 2m 7s

核心问题不是“你把 B-spline 加进去了”，而是你把连续时间轨迹和因子加进了 GLIM 的代码库里，却没有同时把 GLIM 的增量 fixed-lag 求解组织保留下来。
你现在这条 feature/continuous-time-iap 路线，头文件自己就写得很清楚：这是一个 “Phase-1 B-spline odometry module”，目标是在当前 LiDAR-IMU pipeline 之上叠一层连续时间轨迹视图，同时不打破 legacy discrete outputs。这说明它本质上是“叠加式改造”，不是“按 GLIM 的增量求解思想重构过的连续时间后端”。

从你的 timing report 看，这个架构错位已经直接体现在耗时分布上了。KERNEL 已经把 LiDAR factor 本身压到了 18.0 ms，其中 correspondence 约 14.4 ms，但整窗 wall_ms 还是 848.1 ms；最大瓶颈是 lm_optimize_ms = 559.7 ms，第二大瓶颈是 graph_lidar_factor_target_refresh_ms = 237.2 ms。报告还明确指出：KERNEL 改善了 LiDAR runtime，却没有解决 solver cost 和 target refresh cost。

真正“不对劲”的地方有四个。

第一，你现在每帧都在重建整张活动窗图。
insert_frame_ct_lidar() 里先从 fixed_lag_registry_.control_buffer() 拿出当前活动状态和值，然后直接新建一个 gtsam::NonlinearFactorGraph graph;。后面又遍历 fixed_lag_registry_.segments()，把活动窗里所有 segment 的 LiDAR、velocity、IMU、GNSS 因子重新加进图里。也就是说，你不是像 GLIM 那样“向现有 fixed-lag smoother 增量添加当前帧相关因子，再让求解器自己维护活动窗”，而是在每次 scan 到来时重新组一次 full active-window batch problem。这和 timing report 里把 iap CT_LIDAR_GPU + KERNEL 定义为 “Full active-window batch LM every scan” 是一致的。

第二，你把 target/map 的拥有权放错了：target 现在是按 segment 绑定的，不是共享的求解域资源。
append_active_segment_constraint() 每来一帧就调一次 create_active_target_reference()，然后把 target_snapshot、target_tree、target_mode、target_build_ms 全部塞进当前 segment。这意味着 target 不是“当前局部求解域共享的一份 map”，而是“每个活动 segment 都背着一份 target 引用/快照”。而 timing report 里恰好显示：cache hit 大约 20.3、miss 只有 1，但 refresh 也有 20.3，说明虽然因子 cache 命中了，几乎每个活动段还是都要 refresh target；最终 graph_lidar_factor_target_refresh_ms 变成了图构建主热点。这个现象不是 kernel 慢，而是 target ownership 模型本身把刷新成本按活动窗放大了。

第三，你的连续时间 LiDAR GPU factor 内部还有明显的嵌套式重复线性化。
IntegratedBSplineGICPFactorGPU::collect_bucket_system() 里先把 bucket_graph_ 丢进 factor_set，再调用一次 factor_set.linearize(bucket_vals)；紧接着又对每个 bucket.factor 单独调用 linearize(bucket_vals)，提取 Hessian / linear term，并再次调用 bucket.factor->error(bucket_vals) 汇总误差。也就是一层 batch linearize 之后，又逐 bucket 再线性化、再算 error。这个设计会让 LM 里的每次 factor 评估远比“单个 GPU kernel correspondence”重得多。你在 timing report 里看到的 lidar_factor_ms 已经不高，但 lm_optimize_ms 仍然极重，和这里的实现方式是对得上的。

第四，你把局部连续时间因子做成了整窗 dense factor inventory。
在同一个 segment 循环里，你不止加 LiDAR 因子，还加一个 velocity factor；然后对 segment.imu_samples 里的每个 sample 都加一个 IntegratedBSplineIMUFactor；GNSS 开着时还会给 segment 再加 pseudorange 和 doppler 因子。与此同时，create_segment_imu_samples() 是从 scan 时间区间内抓一串 IMU 数据、按 stride 采样后全部塞进 segment.imu_samples。这意味着活动窗不是“少量连续时间状态 + 一组当前域约束”，而是“每个 segment 一套 LiDAR + velocity + 多个 IMU + 可选 GNSS 因子”的大图。对 B-spline 来说，这样的建模当然完整，但对实时求解来说非常重。

所以，问题不在于你选了 B-spline，也不在于 GPU kernel 没加速成功。
问题在于你目前的实现同时继承了三种最贵的东西：

GLIM 代码库里的 fixed-lag 壳子，
连续时间 B-spline 的高维状态与因子，
再加上每帧整窗 batch 重建与 per-segment target refresh。
这三者叠在一起，结果自然比 GLIM 和 C-LIUO 都重。GLIM 便宜，是因为它的组织是增量 fixed-lag smoother；C-LIUO 便宜，是因为它只解一个当前 optimization domain 的局部连续时间问题，并复用共享特征地图。你的 timing report 也明确把这三者的架构差异讲出来了。

你真正想要的是：GLIM 的增量 fixed-lag 组织 + C-LIUO 的连续时间设计。
按这个目标，我建议你不要继续在当前“整窗 batch graph”上小修小补，而是直接改求解组织。

第一步，把“活动窗所有 segment 都重新组图”改成“当前局部连续时间求解域增量更新”。
保留 BSplineControlWindow 和 BSplineFixedLagRegistry，但别再每帧从 control_buffer().values() 开始新建一张 graph。你已经有 carried prior 机制：BSplineCarriedPrior 保存 retained keys、linearization point 和 GaussianFactorGraph，还能 replay()。问题不是没有边缘化，而是你现在把边缘化结果当“重建 batch 图的材料”，而不是像 GLIM 那样把它当增量 fixed-lag 求解器的内部状态延续。你应该让求解器长期存活，只对“新 control point、新当前 scan LiDAR factor、当前域 IMU/GNSS 因子、移出窗口的旧因子/旧键”做增删。

第二步，把 target 从 segment 私有改成求解域共享。
C-LIUO 式设计的关键不是“用了 spline”，而是“当前域内用一份共享的 map / KD-tree / feature map”。你这里最该改的是 append_active_segment_constraint() 这一层：不要让每个 segment 都持有独立 target_snapshot/target_tree；改成 odometry 后端持有一个“当前求解域 target handle”，LiDAR factor 只引用 handle + version。这样 cache hit 时只需要确认 target version 是否变化，而不是把 refresh 成本按 20 个 active segments 乘一遍。graph_lidar_factor_target_refresh_ms 这么高，说明这一步是最高优先级。

第三步，把 LiDAR factor 从“一个 segment 一个大因子”改成“当前 scan / 当前域一个连续时间因子簇”。
你现在是每个活动段一个 IntegratedBSplineGICPFactorGPU，内部再拆 bucket。更接近 C-LIUO 的做法应该是：当前 scan 的所有点基于当前 spline domain 构 correspondence，一次性形成当前域的连续时间残差集合；优化变量只覆盖这个 domain 支撑到的 control points，再由 carried prior 把历史信息接上。这样一来，当前帧只新增一个局部 CT problem，而不是把整个活动窗的所有历史 LiDAR segments 又拉出来过一遍。

第四步，把 GPU factor 内部的重复线性化拆掉。
只要你还保留当前这套 bucket graph 结构，就至少要保证一件事：不要既 factor_set.linearize(bucket_vals)，又对每个 bucket.factor->linearize(bucket_vals) 再来一次，还顺手再 error() 一次。统计信息、误差、Hessian 提取最好并到同一条 pass 里。否则哪怕你把 outer architecture 改好了，LM 里的单因子代价还是偏高。

第五步，把 IMU 从“每个 segment 多 sample 多因子”收缩成更轻的连续时间约束。
你现在 create_segment_imu_samples() 会抓整个 scan 区间的一串 IMU 样本，随后逐样本加 IMU factor。连续时间不是必须意味着“一个样本一个因子”；它也可以是“当前域上若干固定 quadrature nodes”或“预积分/解析积分约束”。如果你想接近 GLIM 的运行代价，就必须压缩 IMU 因子数。

第六步，把 runtime 和 audit 彻底分离。
KERNEL timing 已经显示 postprocess 降了很多，但仍有 post_lidar_factor_error_ms 和 post_lidar_numeric_audit_ms。代码里 ensure_detailed_profile() 甚至会临时构造一个 CPU audit factor 来做详细 profile。这个对诊断有用，但绝不该留在运行态。它不是主因，但你要拿真实实时性能，就必须让 profiling/offline audit 和 runtime 分路。

如果把这件事压成一句最重要的话：

你现在的问题不是“B-spline 太慢”，而是“你把连续时间前端叠在了 GLIM 的代码框架上，却没有重构成 GLIM 风格的增量 fixed-lag solver”；结果就变成了一个 Phase-1 过渡架构：建模很强，但求解组织最重。

我建议你的重构路线是：

先把当前 odometry_estimation_bspline 分成两层。
上层是 Continuous-Time Domain Manager：维护当前 spline domain 的 control points、共享 target handle、当前帧 correspondence 和 carried prior。
下层是 Incremental Fixed-Lag Solver：长期存活，只接受“新增当前域因子 / 删除过期因子 / 更新 shared target version / 添加 carried prior”。
这样你既保留了 GLIM 的求解组织，也能用 C-LIUO 的连续时间建模思路。

# README_REFACTOR_CT_SOLVER.md

# Goal

Refactor `feature/continuous-time-iap` so that the continuous-time BSpline odometry path uses:

- **GLIM-style incremental fixed-lag solver organization**
- **C-LIUO-style local continuous-time solve domain**
- **shared target/map ownership**
- **runtime-oriented CT LiDAR factor evaluation**

The target state is **not** "full active-window batch LM every scan".
The target state is:

1. keep a long-lived fixed-lag solver alive,
2. update only the current local CT domain,
3. reuse a shared target handle across CT LiDAR factors,
4. carry history through a marginal prior,
5. expose the continuous trajectory through stable planner-facing interfaces.

---

# Problem Summary

Current BSpline odometry is too slow because the code is still organized like a
**phase-1 batch overlay**:

- every scan rebuilds a large active-window graph
- active segments keep their own target references
- cached CT LiDAR factors still pay target refresh cost
- CT LiDAR GPU factor internally performs nested bucket processing with repeated
  linearization/error collection
- the continuous-time frontend is strong, but solver organization is heavier
  than both GLIM and C-LIUO

This refactor must change the **solver/dataflow architecture**, not just tune kernels.

---

# Architectural Target

## Desired architecture

Use three layers:

### Layer A: Continuous-Time Domain Manager
Responsible for:
- current local spline domain
- active control points for the current solve domain
- shared target/map handle
- current scan correspondences / target version
- carried prior from the previous lagged solve

### Layer B: Incremental Fixed-Lag Solver
Responsible for:
- persistent nonlinear solver instance
- add/remove factors incrementally
- keep values alive across scans
- accept carried prior as history compression
- marginalize old control points and old local domain factors

### Layer C: Trajectory/Planner Bridge
Responsible for:
- exposing continuous-time trajectory view
- exposing control point access
- planner-safe snapshot interface
- no solver ownership here

---

# KEEP / DELETE / ADD

## KEEP THESE CLASSES

These classes are useful and should remain the backbone:

### State / trajectory / lag infrastructure
- `include/iap/odometry/bspline_control_window.hpp`
- `include/iap/odometry/bspline_fixed_lag_registry.hpp`
- `include/iap/odometry/bspline_trajectory.hpp`
- `include/iap/odometry/bspline_marginalization.hpp`

### Factor classes that should survive conceptually
- `include/iap/odometry/integrated_bspline_gicp_factor*.hpp`
- `include/iap/odometry/integrated_bspline_imu_factor.hpp`
- `include/iap/odometry/integrated_bspline_velocity_factor.hpp`
- `include/iap/odometry/integrated_bspline_pseudorange_factor.hpp`
- `include/iap/odometry/integrated_bspline_doppler_factor.hpp`

### Planner-facing interfaces
- `include/iap/planner/continuous_trajectory_view.hpp`
- existing control-point access / attachment abstractions if already wired

### Why keep them
They already encode the continuous-time state model, carried prior mechanism,
and planner-facing trajectory abstraction. We do not want to throw away the
continuous-time design.

---

## DELETE / STOP USING THESE PATTERNS

Do **not** necessarily delete files immediately.
But Codex should remove these code paths from the hot runtime path.

### 1. Full graph rebuild every scan
In `src/iap/odometry/odometry_estimation_bspline.cpp`, stop the pattern:

- fetch all active values
- construct a fresh `gtsam::NonlinearFactorGraph graph`
- re-append all active-segment LiDAR / IMU / GNSS / velocity factors
- solve the whole active window from scratch every scan

This is the primary architecture bug.

### 2. Per-segment target ownership
Stop storing target snapshots / target trees inside each active segment as the
main runtime ownership model.

Bad pattern:
- segment owns target snapshot
- segment owns target tree
- cache hit still refreshes target state per segment

Replace with shared target handle + versioning.

### 3. Nested repeated bucket linearization in the hot path
In `src/iap/odometry/integrated_bspline_gicp_factor_gpu.cpp`, the hot path must
not:
- batch-linearize
- then re-linearize per bucket
- then re-evaluate error again
inside one top-level factor evaluation.

Runtime path should do one consistent dataflow.

### 4. Runtime numeric audit in the default execution path
All detailed profiling / numeric audit / CPU comparison must be optional and
strictly separated from runtime mode.

---

# ADD THESE THREE CORE INTERFACES

These are the three most important new interfaces.

## Interface 1: `ICTSolveDomain`

Create a new interface, e.g.

- `include/iap/odometry/ct_solve_domain.hpp`

Purpose:
- represent the **current local continuous-time optimization domain**
- own the active control-point span for the current scan
- provide the set of factor keys touched by the current solve
- provide mapping from timestamps to supporting control points
- define which old keys/factors can be retired this scan

Suggested API:

```cpp
class ICTSolveDomain {
public:
  virtual ~ICTSolveDomain() = default;

  virtual bool empty() const = 0;
  virtual double start_time() const = 0;
  virtual double end_time() const = 0;

  virtual std::vector<gtsam::Key> active_control_keys() const = 0;
  virtual std::vector<gtsam::Key> retired_control_keys(double lag_time) const = 0;

  virtual std::vector<int> active_segment_ids() const = 0;
  virtual std::vector<int> retired_segment_ids(double lag_time) const = 0;

  virtual bool supports_time(double stamp) const = 0;
};
```

Concrete implementation:
- `BSplineSolveDomain`

Main rule:
- one scan only updates its **local spline domain**
- not the whole historical active window

---

## Interface 2: `ISharedTargetHandle`

Create a new interface, e.g.

- `include/iap/odometry/shared_target_handle.hpp`

Purpose:
- provide one **shared** target/map object for the current CT LiDAR solve
- target is versioned
- factors bind to a target handle, not to a per-segment private snapshot
- refresh must be handled once per target version, not once per segment

Suggested API:

```cpp
class ISharedTargetHandle {
public:
  virtual ~ISharedTargetHandle() = default;

  virtual uint64_t version() const = 0;
  virtual bool empty() const = 0;

  virtual void sync_from_window_snapshot(/* window snapshot or map source */) = 0;

  virtual const GaussianVoxelMapGPU* gpu_map() const = 0;
  virtual const gtsam_points::KdTree2<gtsam_points::PointCloud>::Ptr& cpu_tree() const = 0;
};
```

Concrete implementations:
- `SharedIvoxTargetHandle`
- `SharedCpuTreeTargetHandle`

Main rule:
- CT LiDAR factors never own the target map
- they only hold a non-owning/shared reference plus a last-seen version

---

## Interface 3: `IIncrementalCTSolver`

Create a new interface, e.g.

- `include/iap/odometry/incremental_ct_solver.hpp`

Purpose:
- keep the solver instance alive across scans
- accept incremental updates
- add current-domain factors only
- retire old-domain factors/keys only
- apply carried prior
- return updated values and covariance summaries

Suggested API:

```cpp
struct CTSolverUpdate {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  std::vector<gtsam::Key> remove_keys;
  std::vector<size_t> remove_factor_slots;   // or symbolic factor ids
  std::optional<iap::BSplineCarriedPrior> carried_prior;
};

struct CTSolverResult {
  gtsam::Values values;
  bool success = false;
  double optimize_ms = 0.0;
};

class IIncrementalCTSolver {
public:
  virtual ~IIncrementalCTSolver() = default;

  virtual void initialize(const gtsam::Values& initial_values) = 0;
  virtual bool initialized() const = 0;

  virtual CTSolverResult update(const CTSolverUpdate& update) = 0;
  virtual gtsam::Values current_values() const = 0;
};
```

Concrete implementation:
- `IncrementalBSplineFixedLagSolver`

Main rule:
- this object persists for the life of odometry
- do not reconstruct a whole batch graph every scan

---

# FILE-LEVEL REFACTOR PLAN

## 1. `odometry_estimation_bspline.cpp`

### Keep
- frame ingestion
- source cloud creation
- control window seeding
- marginal prior building
- trajectory view update
- output frame construction

### Remove from hot path
- full-window graph assembly from all active segments every scan
- per-segment target ownership
- full rescan of all active LiDAR factors as the default organization

### Replace with
- build/update current `BSplineSolveDomain`
- update one shared target handle for the current solve domain
- create only current-domain LiDAR/IMU/GNSS/velocity factors
- send incremental add/remove update into `IncrementalBSplineFixedLagSolver`

### Concrete substeps
1. Extract graph assembly code into:
   - `build_current_domain_lidar_factors(...)`
   - `build_current_domain_imu_factors(...)`
   - `build_current_domain_gnss_factors(...)`
2. Extract lifetime management into:
   - `compute_domain_retirements(...)`
3. Replace `append_active_segment_constraint(...)` with:
   - `update_current_domain_state(...)`
4. Remove segment-owned `target_snapshot` / `target_tree` from the default path.

---

## 2. `integrated_bspline_gicp_factor_gpu.cpp`

### Keep
- spline-support Jacobian mapping logic
- GPU correspondence implementation
- bucket-time interpolation logic if needed

### Remove / refactor
- repeated batch-plus-per-bucket linearization in runtime mode
- repeated error collection inside the same factor pass
- factor-owned target rebuild logic

### Replace with
- factor binds to `ISharedTargetHandle`
- factor checks target version
- factor refreshes only lightweight bindings if version changed
- one runtime evaluation pass computes:
  - correspondence
  - residual statistics
  - Hessian / gradient contributions
- optional detailed audit stays behind a debug flag

### Acceptance rule
A hot-path factor call should not do:
- global target rebuild
- CPU audit
- duplicate bucket graph linearization

---

## 3. `bspline_marginalization.hpp/.cpp`

### Keep
- `BSplineCarriedPrior`
- replay mechanism
- survivor/removable partition machinery

### Extend
Allow carried prior to be consumed directly by `IIncrementalCTSolver::update()`
instead of forcing a fresh batch graph reconstruction.

### Main change
Carried prior becomes the bridge from the previous fixed-lag domain to the next
incremental update, not just a helper used during whole-graph rebuild.

---

## 4. planner bridge

### Keep
- `ContinuousTrajectoryView`
- any existing trajectory attachment logic
- control access abstractions

### Change
Trajectory/planner interfaces must never depend on:
- active segment private target objects
- solver-internal factor layout
- batch graph ownership

They should only consume:
- `SplineWindowSnapshot`
- continuous trajectory sampling
- optional covariance/integrity summaries

---

# IMPLEMENTATION ORDER

Codex should modify the project in this order.

## Phase 0: safety prep
- add compile-time flags so the old path can still build
- keep `Phase-1` path behind a legacy option
- do not delete old code until the new path runs

## Phase 1: interfaces
Add:
- `ICTSolveDomain`
- `ISharedTargetHandle`
- `IIncrementalCTSolver`

Provide stub implementations first.

## Phase 2: shared target ownership
- implement `SharedIvoxTargetHandle`
- move target build/refresh logic out of active segments
- convert CT LiDAR factors to consume shared target handles

## Phase 3: local solve domain
- implement `BSplineSolveDomain`
- replace "iterate over all active segments" with "iterate over current domain"

## Phase 4: persistent solver
- implement `IncrementalBSplineFixedLagSolver`
- keep solver alive across scans
- support add/remove keys/factors and carried prior injection

## Phase 5: factor hot path cleanup
- simplify `IntegratedBSplineGICPFactorGPU` runtime path
- remove duplicate bucket linearization/error work
- gate profiling/audit behind strict debug flags

## Phase 6: odometry wiring
- rewire `OdometryEstimationBSpline` to use:
  - solve domain
  - shared target
  - persistent solver
- verify outputs to mapping/viewer remain compatible

---

# ACCEPTANCE CRITERIA

The refactor is successful only if all of the following become true.

## Architecture criteria
- no full active-window batch graph rebuild every scan
- no per-segment target ownership in the runtime path
- solver instance persists across scans
- carried prior is used as incremental history compression
- current scan updates only the local CT solve domain

## Runtime criteria
- `graph_lidar_factor_target_refresh_ms` drops sharply
- `lm_optimize_ms` drops because only local-domain factors are updated
- `lidar_factor_ms` stays low or improves
- runtime mode has no numeric audit overhead by default

## Compatibility criteria
- continuous trajectory view still works
- mapping/viewer still receive compatible outputs
- GNSS / IMU / velocity constraints still compile and attach to the current domain

---

# THINGS CODEX MUST NOT DO

- do not remove `BSplineCarriedPrior`
- do not delete `ContinuousTrajectoryView`
- do not collapse the system back to discrete-pose odometry
- do not keep per-segment target snapshots as the main runtime ownership model
- do not keep full-window batch LM as the default solver organization
- do not mix runtime path with detailed numeric audit

---

# SUGGESTED NEW FILES

Add at minimum:

- `include/iap/odometry/ct_solve_domain.hpp`
- `include/iap/odometry/shared_target_handle.hpp`
- `include/iap/odometry/incremental_ct_solver.hpp`

Likely implementations:

- `include/iap/odometry/bspline_solve_domain.hpp`
- `src/iap/odometry/bspline_solve_domain.cpp`
- `include/iap/odometry/shared_ivox_target_handle.hpp`
- `src/iap/odometry/shared_ivox_target_handle.cpp`
- `include/iap/odometry/incremental_bspline_fixed_lag_solver.hpp`
- `src/iap/odometry/incremental_bspline_fixed_lag_solver.cpp`

---

# FIRST PR SCOPE

The first PR should be narrow.

## PR-1 objective
Do **not** solve everything.
Only do this:

1. add the three interfaces
2. move target ownership out of active segments
3. make CT LiDAR factors consume `ISharedTargetHandle`
4. keep the old full-graph path compiling behind a legacy flag

If PR-1 succeeds, target refresh cost should already become easier to measure and reduce.

---

# SECOND PR SCOPE

## PR-2 objective
1. add `BSplineSolveDomain`
2. restrict current scan updates to the current local domain
3. stop iterating over all active segments in the hot path

---

# THIRD PR SCOPE

## PR-3 objective
1. add `IncrementalBSplineFixedLagSolver`
2. keep solver alive across scans
3. feed carried prior incrementally
4. remove full-window batch solve from the default path

---

# FOURTH PR SCOPE

## PR-4 objective
1. simplify `IntegratedBSplineGICPFactorGPU`
2. remove duplicate runtime linearization/error work
3. separate runtime mode from audit mode completely

---

# Short instruction for Codex

When editing this branch, optimize for:

- lower `lm_optimize_ms`
- lower `graph_lidar_factor_target_refresh_ms`
- preserving continuous-time BSpline estimation
- preserving planner-facing continuous trajectory interfaces
- preserving mapping/viewer compatibility

Do not optimize only the GPU kernel.
The main task is to fix the **solver organization and target ownership model**.