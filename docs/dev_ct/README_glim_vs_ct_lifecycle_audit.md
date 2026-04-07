# GLIM vs CT 变量生命周期对照审计

Last updated: 2026-04-07  
Branch context: `src/iap` `dev/ct-iap`

## 1. Purpose

这份审计文档的目标，不是证明 GLIM 或 CT/B-spline 的某条数学公式“对/错”，而是正式对照两条主路径中的关键变量生命周期与语义，回答下面这个问题：

> 为什么 GLIM 主路径能够稳定工作，而迁移到 CT/B-spline + shared-state + fixed-lag + survivor/carry 架构后，会出现剩余的 solver-side orientation drift。

当前运行证据已经把问题范围明显收敛：

- `active-window final pose surface` 已经不是主问题；`strict_local` 默认 final pose surface 已把主导性大跳压下。
- 剩余问题是 strict-local 模式下的 solver-side orientation drift，而不是 query/publish surface mismatch。
- 当前正式路线图已经继续收敛：
  - gravity 已可运行在 `external_reference`
  - velocity 已可运行在 no-GNSS `keep_but_not_optimize`
  - 本轮继续把 bias 收回到 lag-local keyed lifecycle，并补齐 CT frontend 的 IMU motion-prior seed
- yaw residual 审计与隔离实验给出的当前排序较稳定：
  - `C. IMU / bias / gravity`: strongest
  - `A. velocity / prior / shared-state`: second
  - `B. orientation semantics / frame-convention`: weakest

因此，这份文档重点不是“对象清单”，而是“失真路径追踪”：

- GLIM 中哪些对象只是局部状态、派生量、或一次性参考
- 到 CT 中后，哪些对象升级成共享状态、跨窗口桥接状态、或 postsolve query/publish 所依赖的持久对象
- 这些语义升级中，哪些最可能解释当前剩余 yaw-dominated drift

## 2. Scope

本次审计只覆盖：

- GLIM odometry smoother 主路径
  - `src/glim/src/glim/odometry/odometry_estimation_imu.cpp`
  - `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp`
- CT/B-spline odometry 主路径
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp`
  - `src/iap/src/iap/odometry/ct_local_frontend.cpp`
  - `src/iap/src/iap/odometry/bspline_marginalization.cpp`
  - `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp`
  - `src/iap/include/iap/odometry/bspline_graph_solver.hpp`

本次不覆盖：

- LiDAR factor 数学核正确性证明
- target map / correspondence 质量再定位
- active-window final pose surface 历史问题复盘
- B2/B3/B4 结构性改造

换句话说，本次只审计 `lifecycle / source-of-truth / semantic equivalence`。

## 2.1 Current Formal Runtime Modes

为把这份审计结论正式落到 `dev/ct-iap` 主路径，当前 CT/B-spline 已新增两组运行时语义模式：

- `gravity_state_mode = shared_optimized | external_reference`
- `bias_state_mode = shared_singleton | lagged_keyed`
- `frontend_seed_mode = last_pose_copy | imu_forward_prediction`
- `velocity_state_mode = optimize | keep_but_not_optimize`
- `velocity_mode_policy = always_optimize | auto_disable_without_gnss`

它们对应的正式含义如下。

### gravity external_reference

当 `gravity_state_mode=external_reference` 时：

- gravity 的 authoritative source of truth 不再是 odometry graph 里的 `g(0)`
- authoritative gravity 改为 estimator 持有的 external reference vector
- IMU / frontend / query path 读取这个 external reference
- registry 里的 gravity 仍可保留一份 mirror/debug copy
- 但这份 copy 不再是 graph-solved authoritative state
- solver result 也不再通过 `seed -> solve -> write-back -> reseed` 闭环改写 gravity

这更接近审计里的 GLIM 语义：gravity 是 reference-like quantity，而不是 persistent shared optimized state。

### velocity keep_but_not_optimize

当 `velocity_state_mode=optimize` 且 `velocity_mode_policy=auto_disable_without_gnss` 时：

- 有 GNSS/Doppler 约束的 solve 仍允许 velocity 进入图中并被约束
- 没有 GNSS/Doppler 约束的 solve 会把 runtime velocity mode 解析为 `keep_but_not_optimize`
- 此时 velocity 仍保留在 frame publish / bookkeeping / debug 路径中
- 但 odometry 不再主动用 velocity factor 或 current velocity prior 驱动当前 solve

这更接近审计里的 GLIM 语义：velocity 仍然是可发布、可记账的导航量，但在无 GNSS 场景下不再升级成 CT 主图里的主动优化主角。

### bias lagged_keyed

当 `bias_state_mode=lagged_keyed` 时：

- gyro bias / accel bias 的 authoritative source of truth 不再是 shared registry 上的 `j(0) / k(0)`
- authoritative bias 改为当前 fixed-lag 图里随 auxiliary/segment 演化的 lagged keyed bias states
- bias key 会随 lag/window 正常退休，不再以 persistent shared singleton 身份长期保活
- 新 bias key 只从唯一 authoritative previous lagged bias 初始化；如果 lagged 链尚未建立，才回退到 startup bootstrap bias
- registry 仍可保留 latest bias cache / publish mirror / debug copy
- 但这些副本只用于 publish、debug、以及空窗口 bootstrap，不再反向定义当前图中的 bias 真值
- lag-to-lag bias continuity 仍由现有 zero-delta transition prior 约束，prior 强度来自 `imu_ct_bias_inf_scale`
- boundary carry 不再把 lagged bias key 本体保留成 carried survivor anchor；bias 仍在当前图中优化，但 carried prior / oldest survivor 诊断会排除它作为边界锚点
- solve 后会把最新 active lagged bias 解镜像到 registry cache 和 publish copy，但这些 mirror 不再作为下一轮 new bias init 的 generic reseed 来源

这更接近审计里的 GLIM 语义：bias 是 lag-local keyed state，而不是跨窗口常驻的 shared singleton。

### frontend imu_forward_prediction

当 `frontend_seed_mode=imu_forward_prediction` 时：

- CT frontend 不再把当前 local support/control poses 直接 seed 成 last frame pose
- seed construction 改为使用上一时刻冻结的 pre-solve state `(pose, velocity, bias)` 和当前扫描覆盖区间的 IMU 积分做 forward prediction
- current frame / current segment 的初值因此更接近真实运动
- strict-local frontend solve、LiDAR factor 数学核、target/reference、以及 final pose surface 语义都保持不变
- runtime metadata 会显式记录：
  - `runtime_frontend_seed_mode`
  - `runtime_imu_forward_prediction_enabled`
  - `runtime_frontend_seed_fallback_used`
  - `runtime_frontend_seed_source`
  - `runtime_frontend_seed_imu_sample_count`
- 如果当前 frame 缺 target-frame 状态或 seed-only IMU 样本不足，seed 会显式 fallback 到 `last_pose_copy`
- jump/report 现在会把 seed mode/source/fallback 直接写进每帧残差审计，便于把：
  - frontend seed 不足
  - backend/boundary 放大
  - LiDAR target/correspondence 质量
  这三类 pitch/roll 残差来源拆开看

这更接近审计里的 GLIM 语义：frontend 初值来自 motion prior，而不是静态的 last-pose copy。

## 3. High-Level Mapping Table

| Variable | GLIM category | CT category | Current risk rank | One-line semantic mismatch summary |
| --- | --- | --- | --- | --- |
| pose | per-frame smoother state `X(i)` + frame materialization | control-point trajectory state + queried frame pose surfaces | medium | pose 从单一 frame-state 读取升级为“control points + layout/evaluator + publish query”的多副本体系 |
| velocity | per-frame smoother state `V(i)` | segment auxiliary optimization state | high | velocity 从 lag-local nav state 升级为跨 frontend / GNSS / prior / publish 的 auxiliary state |
| gyro bias | per-frame smoother state `B(i)` component | mode-dependent: legacy shared singleton `j(0)` or lagged keyed bias state | high | 正式收敛方向是把 gyro bias 从跨窗口 singleton 收回到 lagged keyed lifecycle |
| accel bias | per-frame smoother state `B(i)` component | mode-dependent: legacy shared singleton `k(0)` or lagged keyed bias state | high | accel bias 与 gyro bias 一样，正式收敛方向是去 shared-state 化 |
| gravity | implicit / reference-like IMU quantity, not a graph key in audited GLIM path | persistent shared singleton `g(0)` + registry cache + experiment hooks | highest | gravity 从外部参考量升级成持久共享优化状态，并有 pre/post solve 写回链 |
| carry / survivor / marginal prior / boundary bridge | implicit smoother lag retirement | explicit survivor set + carried Gaussian prior + boundary bridge | high | lag bookkeeping 从隐式 retire 变成显式 bridge object，可把 orientation-side 误差跨窗口保活 |

## 4. Detailed Lifecycle Audit

### 4.1 Pose

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | pose | pose |
| `GLIM_role` | per-frame smoother pose state `X(i)` and materialized `frame->T_world_imu / T_world_lidar` | control-point pose trajectory plus queried frame pose on explicit layouts |
| `CT_BSpline_role` | n/a | active control-point state, local solve seed/query surface, final publish pose |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | initial `X(0)` inserted from init state, later `X(current)` inserted from IMU prediction; `odometry_estimation_imu.cpp:202-208`, `255-257` | control points born when fixed-lag control window/registry extends; frame pose born by querying postsolve layout into `new_frame`; `odometry_estimation_bspline.hpp:605-610`, `odometry_estimation_bspline.cpp:6833-6854` |
| `touched_by` | IMU factor, LiDAR registration factors created by CPU/GPU subclass, smoother update, frame update; `odometry_estimation_imu.cpp:266-275`, `327-331`, `391-403`; `odometry_estimation_cpu.cpp:177-225` | LiDAR local layer, IMU factors, GNSS factors, marginal prior replay, carried prior, postsolve query; `ct_local_frontend.cpp:652-689`, `odometry_estimation_bspline.cpp:3881-4002`, `4050-4089`, `6833-6854` |
| `mutated_by` | smoother estimate update only; `odometry_estimation_imu.cpp:330-331`, `396-403` | solver updates control-point keys; registry/control window updated from solved values; final frame pose assigned from selected query result; `odometry_estimation_bspline.cpp:4217-4234`, `6716-6723`, `6850-6854` |
| `kept_alive_by` | fixed-lag smoother until `marginalized_cursor` retires frame; `odometry_estimation_imu.cpp:334-346` | active control buffer, survivor set, carried prior retained keys, active_window layout/evaluator mirrors; `odometry_estimation_bspline.hpp:606-609`, `bspline_marginalization.cpp:338-410` |
| `retired_by` | lag span check on `marginalized_cursor`; `odometry_estimation_imu.cpp:334-343` | partitioning into survivor/removable keys and carried prior rebuild; `bspline_marginalization.cpp:338-410` |
| `queried_by` | `smoother->calculateEstimate<X(i)>` during frame refresh; `odometry_estimation_imu.cpp:227-230`, `396-403` | start/frontend/final diagnostics and final publish query through layout/evaluator; `odometry_estimation_bspline.cpp:6833-6854`, `7067-7142` |

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | smoother state `X(i)` | graph control-point pose keys while solving; registry/control buffer mirror after update |
| `mirror_or_cache_copies` | `frames[i]->T_world_imu`, `frames[i]->T_world_lidar` | `fixed_lag_registry_.control_buffer()`, `control_window_`, `active_window_layout_`, `active_window_evaluator_`, `new_frame->T_world_lidar / T_world_imu` |
| `synchronization_path` | smoother estimate -> `update_frames()` -> frame copies; `odometry_estimation_imu.cpp:391-403` | solved values -> control buffer / control window update -> postsolve layout query -> `new_frame`; `odometry_estimation_bspline.cpp:4219-4234`, `6716-6723`, `6833-6854` |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | yes, last pose read from smoother for IMU prediction and LiDAR factor creation; `odometry_estimation_imu.cpp:227-242` | yes, control-point/support query during LiDAR/IMU/GNSS factor build; `ct_local_frontend.cpp:652-689`, `odometry_estimation_bspline.cpp:3881-4002` |
| `read_during_solver_update` | yes | yes |
| `read_during_postsolve_query` | no separate postsolve surface; frame refresh reads smoother directly | yes, explicit `active_window` / `strict_local` query surfaces; `odometry_estimation_bspline.cpp:6833-6848` |
| `read_during_publish` | frame materialization already equals smoother estimate | yes, selected final pose query assigned to `new_frame`; `odometry_estimation_bspline.cpp:6849-6854` |
| `read_during_marginal_prior_or_boundary_rebuild` | none explicit in audited path | yes, survivor keys and carried prior depend on active pose key set; `bspline_marginalization.cpp:338-410` |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | SE(3): meters + radians | SE(3): meters + radians |
| `frame_or_convention` | `T_world_imu`, published `T_world_lidar = T_world_imu * T_imu_lidar`; `odometry_estimation_imu.cpp:287-291`, `400-402` | control points parameterize world-to-sensor trajectory; final publish uses queried LiDAR pose then converts to IMU; `odometry_estimation_bspline.cpp:6853-6855` |
| `parameterization` | discrete keyed pose `X(i)` | explicit-knot control-point trajectory + evaluator query |
| `normalization_or_constraint` | pose damping / smooth lag only | pose priors, smoothness, marginal replay, query-surface selection |
| `gauge_or_reference_rule` | first pose anchored/damped in smoother | active-state anchor, marginal prior, carried information prior, final-pose surface selection |

#### F. Invariants

- pose query and publish should use the intended surface for the intended consumer
- pose retirement should not leave stale survivors that remain semantically active
- postsolve query should not silently change support surface relative to the residual being interpreted

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | lag-local keyed pose state with direct smoother readback |
| `CT_state_category` | control-point trajectory state plus multiple query/publish mirrors |
| `semantic_upgrade_or_shift` | pose 从“single keyed estimate per frame”升级成“trajectory state + evaluator surface + publish selection” |
| `semantic_mismatch_risk` | 这类升级已解释过 active-window final surface 问题，但在 strict-local 默认后已不是 highest-risk 当前主因 |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:206-217, 227-257, 391-403`; CT: `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:6833-6854`, `src/iap/src/iap/odometry/bspline_marginalization.cpp:338-410` |
| `current_runtime_evidence` | active-window final surface mismatch 已通过 strict-local 切默认显著缓解，说明 pose query surface 曾是主问题，但不再是当前残余首因 |
| `current_suspicion_level` | medium |
| `next_check_or_fix_target` | 暂不优先；只在 shared-state root cause 收敛后再回看 postsolve query/publish secondary effects |

### 4.2 Velocity

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | velocity | velocity |
| `GLIM_role` | per-frame IMU nav state `V(i)` | segment auxiliary optimization state |
| `CT_BSpline_role` | n/a | local frontend velocity factor, GNSS Doppler coupling, current velocity prior, publish velocity |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | `V(0)` from init state, `V(current)` from IMU prediction; `odometry_estimation_imu.cpp:202-212`, `251-257` | `bspline_velocity_key(auxiliary_index)` born with segment auxiliary state; `ct_local_frontend.cpp:652-661`, `odometry_estimation_bspline.cpp:3881-3893` |
| `touched_by` | IMU factor, fallback zero-velocity between factor, smoother update; `odometry_estimation_imu.cpp:266-275`, `396-403` | `IntegratedBSplineVelocityFactor`, GNSS Doppler, explicit current velocity prior, publish path; `ct_local_frontend.cpp:652-661`, `odometry_estimation_bspline.cpp:3881-3893`, `3990-4000`, `4080-4088`, `6598-6602`, `6855-6857` |
| `mutated_by` | smoother update on `V(i)` | solver update on velocity auxiliary keys |
| `kept_alive_by` | fixed-lag smoother window | active auxiliary indices, survivor auxiliary indices, carried prior retained keys |
| `retired_by` | frame retirement with lag | partition removable auxiliary indices and carried prior rebuild |
| `queried_by` | smoother estimate -> frame refresh; `odometry_estimation_imu.cpp:229`, `397` | publish `new_frame->v_world_imu`, diagnostics, GNSS Doppler | 

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | smoother key `V(i)` | graph velocity key per auxiliary segment |
| `mirror_or_cache_copies` | `frames[i]->v_world_imu` | `fixed_lag_registry_` auxiliary ownership, `new_frame->v_world_imu`, diagnostics rows |
| `synchronization_path` | smoother -> `update_frames()` -> frame copy | solved values -> estimate subset / registry active set -> `new_frame->v_world_imu`; `odometry_estimation_bspline.cpp:6716-6723`, `6855-6857` |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | yes, IMU factor uses `V(last)` and predicts `V(current)` | yes, velocity factor, GNSS Doppler, current velocity prior |
| `read_during_solver_update` | yes | yes |
| `read_during_postsolve_query` | none | not directly in pose query, but final publish reads solved velocity separately |
| `read_during_publish` | yes, copied into frame | yes, copied into `new_frame->v_world_imu`; `odometry_estimation_bspline.cpp:6855-6857` |
| `read_during_marginal_prior_or_boundary_rebuild` | none explicit | yes, current velocity prior / survivor auxiliary ownership |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | m/s | m/s |
| `frame_or_convention` | `v_world_imu` | `v_world_imu` auxiliary state |
| `parameterization` | keyed vector per frame | keyed vector per segment auxiliary index |
| `normalization_or_constraint` | IMU factor or zero-between fallback | velocity factor, GNSS Doppler, optional current velocity prior |
| `gauge_or_reference_rule` | only local lag state | auxiliary state can become quasi-shared through priors and carry |

#### F. Invariants

- velocity should remain auxiliary support for motion continuity, not silently become de facto shared global state
- disabling velocity factor or current velocity prior should not be the only thing preventing drift
- velocity coupling should not dominate yaw unless it is amplifying another shared-state error

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | lag-local optimized nav state |
| `CT_state_category` | reusable auxiliary optimization state shared by multiple subsystems |
| `semantic_upgrade_or_shift` | velocity 从 per-frame nav state 升级成 `auxiliary_index` state，并同时被 velocity factor、GNSS、prior、publish 消费 |
| `semantic_mismatch_risk` | 这会把 orientation-side drift 放大为更长尾的 solver coupling，但当前更像 amplifier 而不是 first cause |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:227-275, 396-403`; CT: `src/iap/src/iap/odometry/ct_local_frontend.cpp:652-661`, `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:3881-3893, 4080-4088, 6598-6602, 6855-6857` |
| `current_runtime_evidence` | yaw shootout 中 A 组 consistently second；`disable_velocity_factor` 会带来明显副作用，但没给出比 gravity 更强的正向证据 |
| `current_suspicion_level` | high |
| `next_check_or_fix_target` | 在 gravity handling 收敛后，优先检查 velocity factor / current velocity prior 与 shared-state orientation 的耦合顺序 |

### 4.3 Gyro Bias

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | gyro bias | gyro bias |
| `GLIM_role` | `B(i)` 的角速度 bias 分量 | persistent shared singleton `j(0)` |
| `CT_BSpline_role` | n/a | 所有 IMU factors 共享的 gyro bias key，同时写入 registry 和 published frame bias |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | `B(0)` from init frame bias, later `B(current)` from `last_imu_bias`; `odometry_estimation_imu.cpp:206-208`, `255-257` | seeded from `fixed_lag_registry_.shared_state().gyro_bias` into `j(0)`; `bspline_fixed_lag_registry.hpp:409-417`, `odometry_estimation_bspline.cpp:3601-3602` |
| `touched_by` | IMU prediction, bias between factor, optional fixed prior, smoother update; `odometry_estimation_imu.cpp:227-263` | IMU factor assembly on `j(0)`, shared-state seeding/write-back, experiments, publish copy; `ct_local_frontend.cpp:678-687`, `bspline_fixed_lag_registry.hpp:409-437`, `odometry_estimation_bspline.cpp:1818-1885`, `1918-1965`, `6858-6859` |
| `mutated_by` | smoother updates `B(i)` | solver updates `j(0)`; then registry write-back may overwrite runtime cache |
| `kept_alive_by` | lag-local state until frame retirement | registry singleton + graph key `j(0)` + published bias copy |
| `retired_by` | lag retirement of old `B(i)` keys | none; singleton is persistent across the run |
| `queried_by` | IMU integration and frame refresh; `odometry_estimation_imu.cpp:230`, `398` | IMU factors, diagnostics, final frame publish |

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | smoother key `B(i)` | graph key `j(0)` while solving, registry `shared_state_.gyro_bias` between solves |
| `mirror_or_cache_copies` | `frames[i]->imu_bias` | `fixed_lag_registry_.shared_state().gyro_bias`, `new_frame->imu_bias.tail<3>()`, experiment anchor/frozen state |
| `synchronization_path` | smoother -> `update_frames()` -> frame bias copy | registry seed -> IMU factor consumption -> solver result -> `update_shared_state_from_values()` -> `new_frame->imu_bias`; `bspline_fixed_lag_registry.hpp:409-437`, `odometry_estimation_bspline.cpp:4219-4234`, `6716-6723`, `6858-6859` |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | yes | yes |
| `read_during_solver_update` | yes | yes |
| `read_during_postsolve_query` | none | none direct |
| `read_during_publish` | yes, copied to frame | yes, copied to `new_frame->imu_bias` |
| `read_during_marginal_prior_or_boundary_rebuild` | none explicit | indirectly through shared singleton surviving all windows |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | angular-rate bias units consistent with IMU model | same |
| `frame_or_convention` | IMU bias vector in IMU model convention | same |
| `parameterization` | part of keyed `ConstantBias B(i)` | standalone persistent `Vector3 j(0)` |
| `normalization_or_constraint` | bias random-walk/between + optional fixed prior | shared singleton, optional freeze experiments |
| `gauge_or_reference_rule` | per-lag state | persistent global shared state for the whole lagged graph |

#### F. Invariants

- gyro bias should preserve one authoritative meaning through the whole solve/publish cycle
- bias should not silently diverge between registry cache, graph key, and published frame copy
- simple freeze should improve drift if gyro bias were the first cause; failure to do so is itself semantic evidence

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | lagged keyed bias state per frame |
| `CT_state_category` | persistent shared singleton outside lag retirement |
| `semantic_upgrade_or_shift` | bias 从 `B(i)` 升级成跨所有 IMU samples/segments 的 shared state key |
| `semantic_mismatch_risk` | 风险高，但当前运行证据说明“简单冻结 gyro bias 不工作”，所以它更像 shared-state package 中的 secondary object，而非独立首因 |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:255-263, 396-403`; CT: `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp:409-437`, `src/iap/src/iap/odometry/ct_local_frontend.cpp:678-687`, `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:1873-1878, 1922-1963, 6858-6859` |
| `current_runtime_evidence` | `freeze_gyro_bias` 没有明显改善 yaw residual，说明 gyro bias 不是单独 first cause；但它仍与 gravity 共享同一 persistent shared-state lifecycle |
| `current_suspicion_level` | high |
| `next_check_or_fix_target` | 继续把 gyro bias 当作 gravity handling 的邻接状态检查对象，而不是单独优先修复点 |

### 4.4 Accel Bias

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | accel bias | accel bias |
| `GLIM_role` | `B(i)` 的线加速度 bias 分量 | persistent shared singleton `k(0)` |
| `CT_BSpline_role` | n/a | 所有 IMU factors 共享的 accel bias key，同时写入 registry 和 published frame bias |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | with `B(i)`; `odometry_estimation_imu.cpp:206-208`, `255-257` | seeded from registry `shared_state_.accel_bias` into `k(0)`; `bspline_fixed_lag_registry.hpp:409-417`, `odometry_estimation_bspline.cpp:3601-3602` |
| `touched_by` | IMU prediction, bias between/fixed prior, smoother update | IMU factors, shared-state seeding/write-back, experiments, publish copy |
| `mutated_by` | smoother update | solver update on `k(0)` plus registry write-back |
| `kept_alive_by` | lag-local bias key | persistent singleton and registry cache |
| `retired_by` | lag retirement | none explicit; singleton persists |
| `queried_by` | IMU integration, frame refresh | IMU factors, diagnostics, publish |

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | keyed `B(i)` in smoother | `k(0)` while solving, registry `shared_state_.accel_bias` between solves |
| `mirror_or_cache_copies` | frame `imu_bias` vector | registry shared state, `new_frame->imu_bias.head<3>()`, experiment anchor |
| `synchronization_path` | smoother -> frame copy | registry seed -> solver -> registry write-back -> publish copy |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | yes | yes |
| `read_during_solver_update` | yes | yes |
| `read_during_postsolve_query` | none | none direct |
| `read_during_publish` | yes | yes |
| `read_during_marginal_prior_or_boundary_rebuild` | none explicit | indirectly via persistent shared-state reuse |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | linear-acceleration bias units consistent with IMU model | same |
| `frame_or_convention` | IMU bias vector convention | same |
| `parameterization` | part of keyed `ConstantBias B(i)` | standalone persistent `Vector3 k(0)` |
| `normalization_or_constraint` | bias between + optional fixed prior | shared singleton with optional freeze experiments |
| `gauge_or_reference_rule` | lag-local | persistent shared-state |

#### F. Invariants

- accel bias should remain single-source through seed/solve/publish
- accel bias freeze should only help if accel bias itself is dominant; failure to help is diagnostic

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | lagged keyed bias state |
| `CT_state_category` | shared singleton |
| `semantic_upgrade_or_shift` | accel bias follows the same upgrade path as gyro bias |
| `semantic_mismatch_risk` | 风险中高，但当前 runtime 证据比 gravity、gyro bias 还弱；更像同一 shared-state package 的附属对象 |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:255-263, 396-403`; CT: `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp:409-437`, `src/iap/src/iap/odometry/ct_local_frontend.cpp:678-687`, `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:1876-1878, 1922-1965, 6858-6859` |
| `current_runtime_evidence` | `freeze_accel_bias` 更差，说明 accel bias 不是当前首因；它更像 gravity/shared-state package 中的次级耦合对象 |
| `current_suspicion_level` | medium |
| `next_check_or_fix_target` | 仅在 gravity handling 收敛后作为次级检查项保留 |

### 4.5 Gravity

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | gravity | gravity |
| `GLIM_role` | IMU integration / initialization 所依赖的参考量；在本次审计的 GLIM odometry smoother 主路径里不是显式 graph key | persistent shared singleton `g(0)` |
| `CT_BSpline_role` | n/a | IMU factors 共享的 persistent gravity state，同时被 registry、soft/freeze modes、solver write-back 和 diagnostics 反复读写 |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | implicit/reference quantity inside IMU integration / init estimation;本次审计主路径未见 `gravity` graph key | born as `shared_state_.gravity` with default `[0,0,9.80665]`, then seeded to `g(0)`; `bspline_fixed_lag_registry.hpp:31-37, 409-417`, `odometry_estimation_bspline.cpp:3601-3602` |
| `touched_by` | indirect only via preintegration / init | IMU factors, shared-state seeding, shared-state write-back, freeze/soft gravity helpers, diagnostics; `ct_local_frontend.cpp:678-687`, `bspline_fixed_lag_registry.hpp:429-437`, `odometry_estimation_bspline.cpp:1791-1915, 1922-1965` |
| `mutated_by` | not a keyed solver variable in audited GLIM path | solver update on `g(0)` plus `update_shared_state_from_values()`, plus experiment helper overwrites via `apply_effective_shared_imu_state_to_registry()` / `enforce_frozen_shared_values()` |
| `kept_alive_by` | external IMU reference semantics; not lag-retained as a graph variable | persistent registry singleton and shared key `g(0)` that never retires with lag |
| `retired_by` | n/a | none; overwritten but not retired |
| `queried_by` | not directly queried for publish in audited GLIM path | IMU factor build, diagnostics/probes, soft-gravity modes; indirectly affects final pose and published frame |

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | external/reference IMU model quantity | graph key `g(0)` while solving, registry `shared_state_.gravity` between solves |
| `mirror_or_cache_copies` | none explicit in audited smoother code | registry `shared_state_.gravity`, experiment freeze anchor, `last_applied_gravity_`, diagnostics rows |
| `synchronization_path` | implicit through IMU integration configuration | registry seed -> `g(0)` -> solver result -> `update_shared_state_from_values()` -> `apply_effective_shared_imu_state_to_registry()` -> next graph seed; `odometry_estimation_bspline.cpp:3601-3602, 4229-4234, 6716-6723`, `bspline_fixed_lag_registry.hpp:409-437` |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | indirect only | yes, every IMU factor references `g(0)`; `ct_local_frontend.cpp:678-687`, `odometry_estimation_bspline.cpp:3912-3925` |
| `read_during_solver_update` | indirect | yes |
| `read_during_postsolve_query` | none direct | none direct; but query result is downstream of solved gravity-influenced trajectory |
| `read_during_publish` | none direct | none direct, but final pose and published bias are shaped by upstream gravity-influenced solve |
| `read_during_marginal_prior_or_boundary_rebuild` | none | indirectly through persistent shared-state surviving every boundary |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | m/s² reference gravity magnitude | m/s² vector |
| `frame_or_convention` | implicit world gravity in IMU integration | stored as world-frame vector with default `[0,0,9.80665]`; consuming factors interpret sign/convention consistently with their model |
| `parameterization` | implicit/reference quantity | explicit `Vector3 g(0)` shared key |
| `normalization_or_constraint` | external to audited smoother code | `normal`, `fixed_norm`, `limited_tilt`, `warmup_freeze_then_release`, legacy freeze; `odometry_estimation_bspline.cpp:1784-1915` |
| `gauge_or_reference_rule` | should act like stable reference aligned with IMU world convention | gravity norm should remain near `9.80665`, direction should evolve smoothly, yaw should not directly drive gravity direction |

#### F. Invariants

- gravity norm should remain near `9.81`
- gravity direction should not jump abruptly across startup / warmup release
- gravity should remain a stable world-frame reference, not a long-memory error reservoir
- gravity updates should not be repeatedly written back into the shared authoritative state in a way that re-seeds the next solve with drift

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | init/integration reference, not an explicit odometry graph state in the audited path |
| `CT_state_category` | persistent shared optimization state |
| `semantic_upgrade_or_shift` | gravity 从“外部参考/隐式量”升级成了 `shared_state_.gravity` + `g(0)` + experiment hook + write-back chain |
| `semantic_mismatch_risk` | 这是当前最危险的升级：错误一旦进入 gravity shared state，就可能被 pre-solve seeding、solver update、post-solve write-back 和下一轮 graph build 反复保活，最容易表现为 yaw-dominated orientation drift |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: audited main path未见 gravity graph key，`src/glim/src/glim/odometry/odometry_estimation_imu.cpp:206-275, 391-403`; CT: `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp:31-37, 409-437`, `src/iap/src/iap/odometry/ct_local_frontend.cpp:678-687`, `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:1791-1915, 1922-1965, 3601-3602, 4229-4234, 6716-6723` |
| `current_runtime_evidence` | strict-local residual审计表明 `C > A > B`；legacy `freeze_gravity` 是最强杠杆但不稳定；soft gravity mode 里 `warmup_freeze_then_release` 最稳定但仍未构成充分修复，说明 gravity handling 仍是 first-cause 候选 |
| `current_suspicion_level` | highest |
| `next_check_or_fix_target` | gravity handling 应优先收敛到“更接近 reference state、较少持久写回、受限 tilt / warmup 保护”的语义，而不是继续把它当 fully persistent shared free state |

### 4.6 Carry / Survivor / Marginal Prior / Boundary Bridge State

#### A. Identity / role

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `variable_name` | carry / survivor / marginal prior / boundary bridge state | carry / survivor / marginal prior / boundary bridge state |
| `GLIM_role` | lag retirement mostly implicit in smoother + `marginalized_cursor` | explicit survivor/removable partition and carried Gaussian prior replay |
| `CT_BSpline_role` | n/a | bridge state that keeps selected keys/factors alive across lag boundaries |

#### B. 生命周期字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `birth` | frame enters lag window | survivor partition and carried prior built from active state set and removable graph; `bspline_marginalization.cpp:338-410` |
| `touched_by` | lag span check only; `odometry_estimation_imu.cpp:334-343` | active-state set, factor ownership, carried prior replay/relinearize, marginal prior attach, diagnostics; `bspline_marginalization.cpp:338-410`, `odometry_estimation_bspline.cpp:4048-4089, 7230-7238` |
| `mutated_by` | smoother internals only | partition rebuild and carried prior rebuild/relinearization |
| `kept_alive_by` | fixed lag implicitly | `marginal_prior_.carried_prior`, `survivor_keys`, `retained_keys`, linearization point |
| `retired_by` | `marginalized_cursor++` and frame reset; `odometry_estimation_imu.cpp:334-343` | retained-keys change and carried prior replacement on next rebuild |
| `queried_by` | none explicit | graph build, boundary diagnostics, solver update diagnostics |

#### C. Source-of-truth 与副本关系

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `source_of_truth` | smoother lag ownership, mostly implicit | `ActiveSplineMarginalPrior` + `BSplineCarriedPrior` + active-state partition |
| `mirror_or_cache_copies` | frame container and lag cursor only | `marginal_prior_`, carried Gaussian graph, retained linearization point, diagnostics summaries |
| `synchronization_path` | smoother retires frames; no explicit replay object in audited path | active-state set -> partition -> removable linearization -> carried prior -> next graph replay; `bspline_marginalization.cpp:350-410`, `odometry_estimation_bspline.cpp:4050-4063` |

#### D. Read stages

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `read_during_factor_build` | none explicit | yes, carried prior replay / marginal prior attach |
| `read_during_solver_update` | implicit lag only | yes |
| `read_during_postsolve_query` | none | none direct |
| `read_during_publish` | none | diagnostics only |
| `read_during_marginal_prior_or_boundary_rebuild` | n/a | yes, this is the main read stage |

#### E. 数值语义字段

| Field | GLIM | CT/B-spline |
| --- | --- | --- |
| `unit` | n/a bookkeeping | inherited from retained pose/velocity/shared state blocks |
| `frame_or_convention` | n/a | inherited from retained key types |
| `parameterization` | implicit smoother lag ownership | explicit retained Gaussian factors + linearization point |
| `normalization_or_constraint` | smoother lag length | survivor set selection and carried prior replay |
| `gauge_or_reference_rule` | old frames disappear once outside lag | selected survivor keys continue to constrain new windows |

#### F. Invariants

- survivor keys must align with the actual active state set
- carried prior should bridge only intended retained states
- boundary bridge state must not preserve a stale orientation-side error longer than the active semantics justify

#### G. GLIM → CT 迁移语义

| Field | Audit |
| --- | --- |
| `GLIM_state_category` | implicit lag bookkeeping |
| `CT_state_category` | explicit cross-window bridge state |
| `semantic_upgrade_or_shift` | lag retirement 从“retire old frame and move on”升级成“retain selected survivor keys and replay a Gaussian prior into the next graph” |
| `semantic_mismatch_risk` | 这类升级不会单独制造 root cause，但会把 shared-state orientation drift 保活并放大成更长尾的 solver churn / boundary amplification |

#### H. 证据字段

| Field | Audit |
| --- | --- |
| `code_evidence` | GLIM: `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:334-343`; CT: `src/iap/src/iap/odometry/bspline_marginalization.cpp:338-410`, `src/iap/include/iap/odometry/odometry_estimation_bspline.hpp:606-607`, `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:4048-4089, 7230-7238` |
| `current_runtime_evidence` | carry/survivor 目前更像 amplifier：它与 boundary/cross-support churn 同步，但当前 strongest 证据仍然落在 shared IMU state / gravity，而不是 boundary bridge 自身 |
| `current_suspicion_level` | high |
| `next_check_or_fix_target` | 在 gravity handling 收敛后，检查 survivor retention 是否让 orientation-side shared-state error 过度跨窗口保活 |

## 5. Highest-Risk Semantic Mismatch Points

### 5.1 Gravity: init/reference quantity -> persistent shared state

- **为什么危险**
  - GLIM 主路径里没有同级别的 gravity graph key；gravity 更像 IMU integration / initialization 参考量。
  - CT 里 gravity 进入 `BSplineFixedLagSharedState`，被 seed 进 `g(0)`，被 solver 更新后再写回 registry，并在下一轮继续 seed。
  - 这让 gravity 从“外部参考”升级成“可被错误反复写回、反复再用”的持久 shared state。
- **代码证据**
  - `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp:31-37, 409-437`
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:1791-1915, 1922-1965, 3601-3602, 4229-4234, 6716-6723`
- **最可能的影响**
  - yaw-dominated solver-side orientation drift
  - warmup/release 期间的方向敏感性
  - 长尾 shared-state memory

### 5.2 Bias: lagged keyed state -> persistent shared singleton

- **为什么危险**
  - GLIM bias 是 `B(i)`，随 lag 自然退休。
  - CT bias 变成 `j(0)` / `k(0)`，和 gravity 一样成为 persistent shared singleton。
  - 一旦 registry cache、graph key、published bias copy 的同步顺序出问题，误差会比 GLIM 更容易长期保活。
- **代码证据**
  - GLIM `src/glim/src/glim/odometry/odometry_estimation_imu.cpp:255-263, 396-403`
  - CT `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp:409-437`
  - CT `src/iap/src/iap/odometry/ct_local_frontend.cpp:678-687`
  - CT `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:1922-1965, 6858-6859`
- **最可能的影响**
  - 与 gravity 同包的 shared-state drift
  - orientation-side solver memory
  - 但从当前 runtime 看，不像独立首因

### 5.3 Velocity: lag-local nav state -> auxiliary multi-consumer state

- **为什么危险**
  - GLIM velocity 是 frame-local `V(i)`。
  - CT velocity 变成 `auxiliary_index` state，同时被 velocity factor、GNSS Doppler、current velocity prior 和 publish 读取。
  - 这会把 orientation-side误差放大到更多 subsystem。
- **代码证据**
  - `src/iap/src/iap/odometry/ct_local_frontend.cpp:652-661`
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:3881-3893, 3990-4000, 4080-4088, 6598-6602, 6855-6857`
- **最可能的影响**
  - solver churn
  - long-tail coupling
  - amplifier rather than first cause

### 5.4 Carry / survivor / marginal prior: implicit lag bookkeeping -> explicit bridge state

- **为什么危险**
  - GLIM 主要靠 lag 窗口隐式退休旧状态。
  - CT 用 explicit survivor set 和 carried Gaussian prior 把一部分状态/信息桥到下一窗口。
  - 这使 orientation-side shared-state 漂移更容易跨窗口持续。
- **代码证据**
  - `src/iap/src/iap/odometry/bspline_marginalization.cpp:338-410`
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:4048-4089, 7230-7238`
- **最可能的影响**
  - boundary amplification
  - solver churn
  - residual long tail

### 5.5 Pose query / publish surface: important historically, weaker now

- **为什么危险**
  - pose 在 CT 中确实比 GLIM 多了 query surface 选择层。
  - 但 `strict_local` 默认后，这一层已不是 strongest runtime explanation。
- **代码证据**
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp:6833-6854`
- **最可能的影响**
  - query/publish inconsistency
  - 当前证据已弱于 gravity/shared-state 问题

## 6. Current Best-Fit Explanation

当前最符合代码和运行证据的解释是：

1. **gravity handling likely over-coupled as persistent shared state**
   - 在 GLIM 中，gravity 更像 IMU integration/init 的参考量。
   - 在 CT 中，它升级成 `shared_state_.gravity + g(0) + write-back chain`。
   - 这使 gravity 成为当前 `highest-risk` 对象。

2. **velocity / prior / shared-state 更像 amplifier，而不是 first cause**
   - velocity 在 CT 中被更多 subsystem 消费，并通过 explicit velocity factor / prior 被持续牵引。
   - 这能放大 orientation drift，但目前没有像 gravity 一样给出更强的 first-cause 杠杆。

3. **carry / survivor / marginal prior 形成长尾保活**
   - 这层最像“把 shared-state orientation drift 跨窗口带下去”的桥接机制。
   - 它解释的是为什么 drift 会持续、会放大，而不是最开始为什么出现。

4. **orientation semantics / frame-convention 当前证据弱**
   - strict-local query support clean
   - yaw chain inconsistency 证据弱
   - 当前不支持把它排到 first tier

一句话总结：

> 当前最像的不是某一个 IMU 因子公式错，而是 GLIM 中偏“参考/lag-local”的对象，在 CT 中升级成了“persistent shared state + auxiliary state + carried bridge”的持久链条；其中 gravity 是最可疑的 first-cause 对象，velocity/prior/shared-state 与 carry/survivor 更像 amplifier。

## 7. Next Checks / Fix Priorities

### First

**gravity handling**

- 优先检查 gravity 是否真的应该保持当前这种 `shared singleton + pre/post solve write-back` 语义
- 优先收敛：
  - gravity norm handling
  - gravity tilt update limiting
  - warmup-sensitive release strategy
- 当前不建议直接回到粗暴 freeze

### Second

**velocity/prior/shared-state amplification path**

- 在 gravity 语义更稳定后，再检查：
  - velocity factor
  - current velocity prior
  - shared-state 与 auxiliary velocity 的同步顺序
- 当前不建议继续优先做 A 组 disable 扩张

### Third

**carry / survivor / marginal prior bridge semantics**

- 检查 survivor selection 和 carried prior replay 是否让 orientation-side drift 保活过久
- 这应放在 gravity first-cause 更清楚之后

### Not Recommended First

- `active_window` final surface 问题：已基本压下
- broad frame-convention mismatch 追查：当前证据弱
- LiDAR factor math / target map / B2/B3/B4：与当前剩余问题不在同一优先层

## 8. Appendix

### 8.1 Runtime evidence snapshots used by this audit

本 README 参考了已有的运行证据结论：

- strict-local residual audit：`src/iap/log/2026-04-06_10-26-37/analysis/report.md`
- yaw residual A/B/C shootout：`src/iap/log/2026-04-06_11-37-56/analysis/report.md`
- gravity isolation matrix baseline / legacy / A-group reference：
  - `src/iap/log/2026-04-06_13-39-58_yaw_gravity_baseline/analysis/report.md`
  - `src/iap/log/2026-04-06_12-52-27_yaw_iso_freeze_gravity/analysis/report.md`
  - `src/iap/log/2026-04-06_12-57-14_yaw_iso_disable_velocity_factor/analysis/report.md`

这些文件是运行产物，不一定是长期 tracked source，但它们提供了当前 suspicion ranking 的直接运行证据。

### 8.2 Audited file set

- GLIM
  - `src/glim/include/glim/odometry/odometry_estimation_imu.hpp`
  - `src/glim/src/glim/odometry/odometry_estimation_imu.cpp`
  - `src/glim/src/glim/odometry/odometry_estimation_cpu.cpp`
- CT/B-spline
  - `src/iap/include/iap/odometry/odometry_estimation_bspline.hpp`
  - `src/iap/include/iap/odometry/bspline_fixed_lag_registry.hpp`
  - `src/iap/include/iap/odometry/bspline_graph_solver.hpp`
  - `src/iap/src/iap/odometry/odometry_estimation_bspline.cpp`
  - `src/iap/src/iap/odometry/ct_local_frontend.cpp`
  - `src/iap/src/iap/odometry/bspline_marginalization.cpp`

### 8.3 Terminology

- **authoritative state**: 当前数值语义上应被视为“真值”的那份 state
- **mirror/cache copy**: 为了下一轮 seed、publish、debug 或 query 临时保存的副本
- **shared state**: 跨 segment / frame / lag boundary 复用的持久状态
- **auxiliary state**: 不是 control-point pose，但和 segment 一起优化的辅助状态
- **carried prior**: 从上一轮 removable graph 线性化后保留到 survivor keys 的 Gaussian bridge
