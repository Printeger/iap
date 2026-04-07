# IAP 当前系统流程

这份文档描述的是 `Printeger/iap` 的 `dev/ct-iap` 当前主线路径，而不是早期审计阶段的怀疑性草图。

它对应的实现重点是：

- 单一分层 factor-graph 主路径，而不是“frontend 一个优化器 + backend 另一个优化器”的双求解器系统
- `strict_local` 是当前默认 final pose surface，`active_window` 主要保留作 debug / regression 对照
- gravity / velocity / bias / frontend seed 都已经是显式 runtime mode，而不是隐含的固定语义

## 1. 当前主线运行模式

当前仓库自带的主配置和主路径语义可以概括为：

| 项 | 当前主线含义 |
| --- | --- |
| `frontend_mode` | `CT_LIDAR_CPU` |
| `frontend_only_mode` | `false`，默认走统一图主路径 |
| `use_legacy_bspline_two_stage_path` | `false` |
| `bspline_unified_solver_mode` | 支持 `BATCH_LM` 与 `INCREMENTAL_SMOOTHER`；当前 bundled config 为 `INCREMENTAL_SMOOTHER` |
| `final_pose_surface` | `strict_local` |
| `gravity_state_mode` | 支持 `shared_optimized` / `external_reference`；当前 bundled config 为 `external_reference` |
| `velocity_state_mode` | 支持 `optimize` / `keep_but_not_optimize`；当前 bundled config 为 `keep_but_not_optimize` |
| `velocity_mode_policy` | 支持 `always_optimize` / `auto_disable_without_gnss`；当前 bundled config 为 `auto_disable_without_gnss` |
| `bias_state_mode` | 支持 `shared_singleton` / `lagged_keyed`；当前 bundled config 为 `lagged_keyed` |
| `frontend_seed_mode` | 支持 `last_pose_copy` / `imu_forward_prediction`；当前 bundled config 当前设为 `imu_forward_prediction` |

这意味着：当前 IAP 不能再被简单描述成“shared gravity + shared bias + active velocity + 双阶段 frontend/backend”架构。

## 2. 系统主流程

### M0. Runtime Config / Mode Resolution

输入：
- config json
- runtime flags
- profiling / experiment 开关

输出：
- frontend mode
- solver mode
- final pose surface
- gravity / velocity / bias / frontend seed mode

authoritative state：
- 当前 run 真正生效的 runtime config snapshot

当前正确语义：
- `final_pose_surface` 是显式模式，当前主线默认是 `strict_local`
- gravity / velocity / bias / frontend seed 都是 mode-resolved runtime semantics
- report / run metadata 负责记录 config 与 runtime resolved 值，但不是运行真值本身

### M1. Sensor Ingress / Time Normalization

输入：
- LiDAR raw frame
- IMU samples
- 可选 GNSS / Doppler

输出：
- raw frame time range
- scan begin / end time
- bucket representative times
- factor build 使用的时域输入

authoritative state：
- 原始时间戳及其规范化后的当前帧时间语义

当前正确语义：
- 这一层决定后续 strict-local query、bucket support、IMU factor stamp、frontend seed window 的时间基准
- 时间语义错误会同时污染 frontend、solver、publish 三条链

### M2. State Containers / Registry

输入：
- 上一轮 graph result
- carried prior
- 历史 frame / segment / control buffer
- runtime mode

输出：
- control buffer
- auxiliary values
- active segment ownership
- active layouts / evaluators
- shared-state mirror / runtime references

authoritative state：
- 不是“registry 一处包打天下”的单一真值
- 真值依赖状态类型和当前 mode

当前正确语义：
- control / auxiliary / carried prior / frame materialization / registry cache 会并存多份副本
- 但 source-of-truth 是分类型的，不应再被描述成统一 shared-state 真值

### M3. Runtime State Resolution

输入：
- registry mirrors
- 当前 graph values
- runtime mode policy

输出：
- 当前 solve 真正生效的 gravity / bias / velocity / frontend seed semantics

authoritative state：
- gravity：
  - `external_reference` 模式下是 estimator-owned external reference
  - 不是 graph `g(0)` 的解
- bias：
  - `lagged_keyed` 模式下是当前 active lagged bias keys
  - frame / registry bias copy 只是 mirror / publish cache
- velocity：
  - 是否 actively optimized 取决于 `velocity_state_mode` 和 `velocity_mode_policy`
  - 无 GNSS 时主线可解析为 `keep_but_not_optimize`
- frontend seed：
  - 取决于 `frontend_seed_mode`
  - 允许 `last_pose_copy` 或 `imu_forward_prediction`

当前正确语义：
- 这一层不应该再被概括成“Shared-State Manager”
- 当前主线最准确的说法是“mode-resolved reference / auxiliary state resolution”

### M4. Control / Auxiliary State Manager

输入：
- control buffer
- segment / auxiliary ownership
- active lag window

输出：
- active controls
- active auxiliary states
- local support mapping

authoritative state：
- current active control set
- current active auxiliary set
- ownership map

当前正确语义：
- velocity / bias 都已不应被笼统写成 persistent shared singleton
- auxiliary states 的语义取决于 mode 和当前 solve 是否真的优化它们

### M5. Pre-solve Query Surface / Seed Builder

输入：
- pre-solve layout
- current control / auxiliary state
- runtime reference state
- raw frame timing

输出：
- start pose
- strict-local pre-solve query
- frontend seed inputs

authoritative state：
- strict-local pre-solve pose / layout query result

当前正确语义：
- 当前主线路径里，pre-solve pose 应与 `strict_local` surface 对齐
- frontend seed 是这层的一部分，不是独立的“第二套姿态真值”
- `frontend_seed_mode` 决定是 `last_pose_copy` 还是 `imu_forward_prediction`

### M6. CT Local Frontend / Local Layer Helper

输入：
- pre-solve seed
- LiDAR source frame
- IMU sample window
- target reference / snapshot
- runtime modes

输出：
- local layer solve seed values
- LiDAR / IMU local factors
- frontend shadow diagnostics
- 在 `frontend_only_mode` 下可输出独立 frontend solve result

authoritative state：
- 在当前主线统一图路径里，这一层主要提供 local factor contribution 与 diagnostics
- 不是默认的最终发布姿态 source-of-truth

当前正确语义：
- 不应再把它简单写成“frontend strict-local solve result 就是系统真值”
- 在 `frontend_only_mode=false` 的主线下，最终姿态仍由统一图 solver + postsolve query surface 决定

### M7. Unified Graph Assembly

输入：
- local frontend / local layer factors
- navigation layer states
- GNSS / clock / prior / carried prior
- active control / auxiliary ownership

输出：
- new values
- new factors
- active key sets
- 本轮进入 solver 的 graph delta

authoritative state：
- 当前进入统一 solver 的图问题

当前正确语义：
- frontend 与 backend 已不是两套优化器
- 它们是一个统一 fixed-lag graph 中的不同层 / 不同因子来源

### M8. Carry / Survivor / Marginal Prior Bridge

输入：
- active state set
- removable keys
- previous carried prior

输出：
- survivor keys
- carried Gaussian prior
- boundary bridge

authoritative state：
- 当前窗口边界保留的信息摘要

当前正确语义：
- 这层仍是误差放大器，但不等于系统主状态真值
- 在当前主线下，bias survivor anchor 已做过专门收敛，不能再用旧图把它写成“默认 shared bias anchor”

### M9. Unified Fixed-Lag Solver

输入：
- graph delta
- carried prior

输出：
- post-solve estimate

authoritative state：
- solver result

当前正确语义：
- 这里不是“只有 incremental solver”
- 正确说法是：统一 fixed-lag solver，当前实现支持 `BATCH_LM` 与 `INCREMENTAL_SMOOTHER`
- 当前 bundled config 选择的是 `INCREMENTAL_SMOOTHER`

### M10. Post-solve Write-back

输入：
- solver result

输出：
- control buffer update
- auxiliary update
- frame materialization inputs
- registry mirrors / caches 更新

authoritative state：
- 写回后的 mode-consistent source-of-truth 与镜像副本

当前正确语义：
- 这一层仍是最容易把 solve 结果误写成“下一轮 seed 真值”的地方
- 但现在必须区分：
  - authoritative graph state
  - publish/debug mirror
  - registry cache
- 不能再笼统写成“统一 shared-state update”

### M11. Postsolve Query Surface Selector

输入：
- post-solve state

输出：
- `postsolve_active_window_pose`
- `postsolve_strict_local_pose`
- final pose candidate

authoritative state：
- 当前选中的 `final_pose_surface`

当前正确语义：
- 当前主线默认 final surface 是 `strict_local`
- `active_window` 仍保留作 debug / regression / surface comparison
- 最终发布姿态由这一层统一选择，不应再被下游重新解释

### M12. Publish / Frame Materialization

输入：
- final pose candidate
- velocity output
- bias mirror

输出：
- `new_frame->T_world_lidar`
- `new_frame->T_world_imu`
- `new_frame->v_world_imu`
- `new_frame->imu_bias`
- odom / pose / TF / callbacks

authoritative state：
- published frame state

当前正确语义：
- 这里应该只消费上游最终选中的 final state
- frame 中的 velocity / bias 常常是 publish materialization，不应被倒推成“统一系统真值”

### M13. Diagnostics / Reports / Audit

输入：
- jump diagnostics
- solver update profile
- frontend / lidar / runtime logs

输出：
- report.md
- CSVs
- lifecycle / methodology docs

authoritative state：
- 只是观测，不是运行真值

当前正确语义：
- diagnostics 可以描述 mode、query surface、seed source、solver churn
- 但它们不应反过来定义系统 source-of-truth

## 3. 当前文档里最容易误导的旧说法

下面这些说法不再适合作为“当前 IAP 架构”的描述：

1. `gravity` 是主线默认的 persistent shared optimized state
- 这只适用于旧语义或兼容模式。
- 当前 bundled config 主线是 `external_reference`。

2. `bias` 仍是 persistent shared singleton
- 当前 bundled config 主线是 `lagged_keyed`。
- authoritative bias 已更接近 active lagged bias chain，而不是统一 `j0/k0` shared singleton。

3. `velocity` 总是 actively optimized odometry state
- 当前 bundled config 是 `keep_but_not_optimize + auto_disable_without_gnss`。
- 它仍可被发布 / bookkeeping 使用，但不应再被一概描述成主图主动优化状态。

4. `CT Local Frontend` 是默认 final-state solver
- 当前主线不是。
- 默认主线是统一图求解，frontend 更多承担 local factor / shadow diagnostics 角色。

5. 系统只有 incremental solver
- 当前实现支持 batch 和 incremental 两种统一 fixed-lag solver mode。

## 4. 一条更贴近当前实现的系统链

可以把当前 IAP 主线简化成：

`runtime mode resolution`
-> `sensor ingress / time normalization`
-> `control + auxiliary + reference state resolution`
-> `strict-local pre-solve query / seed build`
-> `CT local layer factor build`
-> `unified fixed-lag graph assembly`
-> `carry / survivor / marginal prior bridge`
-> `batch or incremental solver`
-> `postsolve strict_local / active_window query`
-> `final pose surface selection`
-> `frame materialization / publish`
-> `diagnostics / report`

这条链比旧版描述更贴合当前 `dev/ct-iap` 的系统结构。
