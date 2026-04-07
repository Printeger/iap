# CT/B-spline Odometry Blocking Module Audit

本文件把当前 `dev/ct-iap` 主路径按真实调用顺序拆成模块，逐个记录：

- 验证方法
- checklist
- 当前结果
- 代码证据
- 运行证据
- 阻塞关系
- 若不通过，最小下一修复点

本次审计遵循阻塞式原则：

- 每个模块必须给出 `PASS` / `PASS_WITH_RISK` / `FAIL` / `BLOCKED_BY_UPSTREAM`
- 前序模块若 `FAIL`，后续模块不得再默认“应该没问题”
- 每个结论必须同时给代码证据和运行证据
- 本文优先做结构化验收，不顺手扩新 telemetry，不先做大改

## Scope And Sources

- 主运行证据：`log/latest`
- 主报告：`log/latest/analysis/report.md`
- metadata：`log/latest/metadata/run_info.json`、`log/latest/metadata/config_snapshot.json`
- 运行 CSV：`jump_diagnostics.csv`、`solver_update_profile.csv`、`frontend_frame_profile.csv`
- 模块契约来源：`docs/methodology/iap_sysytem.md`
- 生命周期语义来源：`docs/dev_ct/README_glim_vs_ct_lifecycle_audit.md`

## Overview

| Module | Status | Blocked By | Validation Method | Current Risk | Minimum Next Fix |
|--------|--------|------------|-------------------|--------------|------------------|
| M0 Runtime Config / Mode Policy | PASS | - | 对比 `config_snapshot` / `run_info` / `report` 三处模式字段，并回查 mode 解析与 runtime metadata 写入代码 | 低 | 无 |
| M1 Sensor Ingress / Time Normalization | PASS_WITH_RISK | - | 沿 `raw_frame stamp -> frontend_target_time -> query_time -> seed IMU window -> bucket query` 追唯一 target-time 契约，并对照新 target-time 字段与前端测试执行结果 | 中 | 跑一轮新的端到端 run，确认 runtime offsets 只剩 representative-time 描述性差值 |
| M2 State Containers / Registry | PASS_WITH_RISK | - | 追 `control_buffer` / `auxiliary_values` / `shared_state` / `frame copy` 的 ownership 与 write-back 路径 | 中 | 继续压缩 mirror fan-out，并把 authoritative/mirror 边界写死到代码注释或 helper |
| M5 Pre-solve Query Surface / Seed Builder | PASS_WITH_RISK | - | 追 `seed builder -> seeded_local_values -> query_time -> support_at(query_time)`，并对照 `frontend_target_time` / `seed_integration_end_time` / bucket query 一致性 | 中 | 用同一 target-time 契约各跑一轮 `last_pose_copy` 和 `imu_forward_prediction`，把时间契约问题与 seed 质量问题拆开 |
| M6 CT Local Frontend | PASS_WITH_RISK | - | 验证 seed 是否真实进入 solve，并检查 local target/correspondence 输入是否都消费同一个 `frontend_target_time` | 中 | 跑一轮新的端到端 run，独立评估 frontend correspondence / registration 质量 |
| M7 Graph Assembly | PASS_WITH_RISK | - | 检查图中状态身份是否与 runtime mode 一致，并确认 M1/M5 收敛后图装配没有被 seed 时间错位连坐 | 中 | 在新的端到端 run 上复核 residual 是否仍主要由 M8 链放大 |
| M8 Carry / Survivor / Marginal Prior Bridge | FAIL | - | 追 `active_state_set -> survivor_keys -> carried_prior -> postsolve support mismatch`，并对照 `boundary_shift` 与 survivor 分布 | 高 | 收窄 carried prior / survivor pose anchor，只保留 strict-local 真正必要的信息 |
| M9 Fixed-Lag Incremental Solver | PASS_WITH_RISK | M8 | 检查 solver 是否只消费 graph delta，并结合 `solver_update_profile` 看 residual 放大族群 | 中 | 先修 M8，再判断 solver 放大到底是“被动放大”还是“自身配置过宽” |
| M10 Post-solve Write-back | PASS_WITH_RISK | M8 | 追 solve result 如何分发到 `control_buffer` / `auxiliary_values` / `shared_state` / `evaluation_values` / `new_frame` | 中 | 收敛 postsolve evaluation/write-back 链，减少多份容器重组后的语义漂移 |
| M11 Postsolve Query Surface Selector | PASS_WITH_RISK | M8, M10 | 检查 `active_window` / `strict_local` query 与 final pose 选择逻辑，并对照两 surface 差异 | 中 | 保持 `strict_local` 默认，优先修 M8/M10 造成的 query surface 偏差 |
| M12 Publish / Frame Materialization | PASS_WITH_RISK | M10, M11 | 检查 publish 是否只消费 final truth，以及 `new_frame` 如何变成下一轮 target frame | 中 | 在 M10/M11 收敛后再次确认 frame copy 不再携带模糊语义 |
| M13 Diagnostics / Reports / Audit | PASS | - | 检查 report 是否只做观测、不改主路径，并验证 runtime mode 与结论一致 | 低 | 无 |

## Blocking Dependency Map

- `M0 FAIL -> block M1..M13`
- `M1 FAIL -> block M5/M6`，并使所有涉及 frontend seed / query-time 归因的下游判断降级
- `M2 FAIL -> block M5/M7/M10/M11/M12`
- `M5 FAIL -> block M6`，并弱化 M7 对 frontend 输入正确性的解释力
- `M8 FAIL -> block M9/M10/M11` 对 residual 放大机制的完整归因
- `M10 FAIL -> block M11/M12` 对 authoritative final state 的可信度
- `M11 FAIL -> block M12`
- `M12 FAIL -> block M13` 对 publish correctness 的判断

当前这轮真正落地的阻塞链有两条：

1. `M8 -> M9 -> M10 -> M11`
2. `M2 -> M10 -> M12`（仍是风险链，但不是当前第一个 FAIL）

其中：

- `M1 -> M5 -> M6` 在本轮 target-time contract fix 后已经不再是硬阻塞链
- 当前真正还在挡路的主链，是 residual/boundary 放大链 `M8 -> M9 -> M10 -> M11`

## Round-1 Target-Time Contract Fix

### 定义

- 唯一 authoritative `frontend_target_time` 定义为：
  - `current_source_frame.scan_start`
  - 在当前主路径里等价于 `raw_frame->stamp`

### 由谁生成

- `make_frontend_input()` 在 odometry 主路径里显式生成：
  - `input.frontend_target_time = raw_frame->stamp`
  - `input.frontend_target_time_kind = scan_start`
  - `input.frontend_target_time_source = current_source_frame.scan_start`

### 被谁消费

- `CTLocalFrontend::run()` 的 `query_time`
- `run_shadow_diagnostics(...)` 的 query time
- `make_frontend_motion_seed()` 的 IMU integration end time
- frontend pose diagnostics / jump diagnostics 里的 `bucket_query_time`

### 修前 / 修后变化

- 修前：
  - `start_pose query`
  - `frontend_pose query`
  - `bucket representative query`
  - `seed integration end`
  - 并不共享同一个 target-time contract
- 修后：
  - `start_pose query time = frontend_pose query time = seed_integration_end_time = bucket_query_time = frontend_target_time`
  - `bucket_representative_time` 保留，但只再表示 bucket 在 scan 内的描述性相对时间，不再充当 frontend 真值时间

### 为什么这能解除 `M1 -> M5 -> M6` 的阻塞

- `M1` 现在提供了唯一可追踪的 frontend 时间真值
- `M5` 不再允许 seed integration、query surface、bucket query 各自定义时间面
- `M6` 因而可以重新按“frontend 是否正确消费统一 pre-solve 时间面”来验收，而不再被上游直接阻塞

---

## M0. Runtime Config / Mode Policy

### 1. Intended contract

- 负责把 config 解析成唯一 runtime mode
- `config_snapshot`、`run_info`、`analysis/report` 三处必须一致
- 这个模块只做 mode 解析与记录，不应偷偷改变 solver 身份语义

### 2. Validation method

- 代码侧：
  - 查配置默认值与解析点
  - 查 `run_info` 初始化点
  - 查运行中 runtime metadata patch 点
- 运行侧：
  - 对比 `config_*`、`runtime_*`、report summary 三处字段
- 最低通过标准：
  - 模式值唯一
  - config/runtime/report 没有出现 `A / B / C` 三套口径

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [x] config/runtime/report values agree

### 4. Result summary

- `PASS`
- 当前 run 的 mode resolution 是一致的。
- 风险不在“到底跑了什么模式”，而在“当前 bundled config 选了一个实验性 seed 模式”。

### 5. Runtime evidence

- `final_pose_surface`: `strict_local`
- `gravity_state_mode`: `external_reference`
- `velocity_state_mode`: `keep_but_not_optimize`
- `bias_state_mode`: `lagged_keyed`
- `frontend_seed_mode`: `imu_forward_prediction`
- `config_*` 与 `runtime_*` 在 `report.md` 中一致，没有出现 config/runtime/report 三套口径

### 6. Code evidence

- `config/config_odometry_bspline.json:41-74`
- `src/iap/common/log_paths.cpp:440-503`
- `src/iap/odometry/odometry_estimation_bspline.cpp:2308-2363`

关键点：

- bundled config 定义 solver mode / final pose surface / lifecycle modes
- `log_paths.cpp` 先写 config/runtime 初值
- `sync_runtime_mode_metadata()` 再用真实运行状态补丁覆盖 runtime 字段

### 7. Verdict

- `PASS`

### 8. If fail: blocking reason and minimum next fix

- 不适用

---

## M1. Sensor Ingress / Time Normalization

### 1. Intended contract

- 把 `raw_frame stamp`、`scan_start/end`、bucket representative time、query time、IMU sample window 规范成一套唯一时间语义
- 下游不应再各自重新解释“当前帧时间”
- 这个模块不应允许 `bucket query`、`seed integration target`、`frontend query time` 各自跑在不同 target time 上

### 2. Validation method

- 代码侧：
  - 查 `make_frontend_input()` 如何生成 `frontend_target_time`
  - 查 `CTLocalFrontend::Input` / `FrontendPoseDiagnostics` 的 target-time 字段
  - 查 `resolve_frontend_target_time()` 与 `record_frontend_target_time_observation()`
  - 查 `create_frontend_seed_imu_samples(start_stamp, end_stamp)` 的 `end_stamp` 是否已经改成 `frontend_target_time`
- 运行侧：
  - 看 `test_ct_local_frontend` 的 target-time 合同测试是否通过
  - 看 `test_ct_hybrid_pipeline` 在改动后是否仍然通过
  - 看 report / metadata 是否已经具备：
    - `runtime_frontend_target_time_kind`
    - `runtime_frontend_target_time_source`
    - `runtime_frontend_target_time`
    - `runtime_frontend_target_time_offset_vs_*`
- 最低通过标准：
  - `frontend_target_time` 唯一
  - query / seed integration / bucket query 指向同一个 target-time 语义

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [x] raw_frame / bucket / query / imu windows use one time semantic

### 4. Result summary

- `PASS_WITH_RISK`
- 当前系统已经把 frontend pre-solve 的 authoritative target time 显式收敛为 `current_source_frame.scan_start`。
- 风险不再是“target time 不存在”，而是“还缺一轮新的端到端 run 去证明新字段在真实日志里也整体收敛”。

### 5. Runtime evidence

- `ctest --test-dir build/iap -R 'test_ct_local_frontend|test_ct_hybrid_pipeline'` 通过
- `CTLocalFrontendSolve.FrontendTargetTimeContractIsExplicitAndConsistent` 断言：
  - `frontend_target_time == start_pose_query_time == frontend_pose_query_time == bucket_query_time`
  - `seed_integration_end_time == frontend_target_time`
  - `frontend_target_time_consistent == true`
- `CTLocalFrontendSolve.ShadowDiagnosticsConsumeProvidedQueryTimeAsFrontendTargetTime` 断言：
  - shadow diagnostics 不再自推 query time，而是显式消费提供的 `frontend_target_time`
- 当前 `log/latest` 早于本轮修复，因此新的 run-level offset 统计仍待下一轮真实 run 验证；这也是本模块暂留 `WITH_RISK` 的原因

### 6. Code evidence

- `include/iap/odometry/ct_local_frontend.hpp`
- `src/iap/odometry/ct_local_frontend.cpp`
- `include/iap/odometry/odometry_estimation_bspline.hpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `src/iap/common/log_paths.cpp`
- `tools/ana_log.py`

关键点：

- `make_frontend_input()` 显式生成 `frontend_target_time = raw_frame->stamp`
- `resolve_frontend_target_time()` 负责 frontend 层唯一 target-time 解析
- `create_frontend_seed_imu_samples(..., end_stamp)` 的终点已改成 `frontend_target_time`
- `record_frontend_target_time_observation()` 把 start/frontend query、seed integration end、bucket query 是否对齐写进 runtime metadata
- `ana_log.py` 新增 target-time contract summary 与 findings

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 跑一轮新的端到端 odometry run
  - 确认 `runtime_frontend_target_time_offset_vs_scan_start = 0`
  - 并确认 `offset_vs_representative` 只剩描述性差值，不再被误解成契约错误

---

## M2. State Containers / Registry

### 1. Intended contract

- 明确谁是 authoritative state，谁只是 mirror/cache/debug copy
- solve 后写回路径必须可解释
- mirror/debug copy 不应反向污染 authoritative state

### 2. Validation method

- 代码侧：
  - 查 `control_buffer`
  - 查 `auxiliary_values`
  - 查 `shared_state`
  - 查 `seed_shared_values()` / `update_shared_state_from_values()`
  - 查 bias mirror/write-back helper
- 运行侧：
  - 看 runtime metadata：
    - `runtime_bias_source_of_truth`
    - `runtime_bias_writeback_mode`
    - `runtime_bias_can_be_survivor_anchor`
    - `runtime_gravity_state_mode`
    - `runtime_velocity_state_mode`
- 最低通过标准：
  - 真值归属唯一
  - mirror 不反向升级成主真值

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [ ] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [ ] graph state and registry mirror are not conflated

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 source-of-truth 已经比早期 shared-state 版本清楚得多。
- 但 registry 仍保留多份 mirror，postsolve 后仍有多段 fan-out，同步顺序需要继续谨慎审。

### 5. Runtime evidence

- `runtime_gravity_state_mode = external_reference`
- `runtime_velocity_state_mode = keep_but_not_optimize`
- `runtime_bias_state_mode = lagged_keyed`
- `runtime_bias_source_of_truth = active_lagged_bias_keys`
- `runtime_bias_can_be_survivor_anchor = False`
- `runtime_bias_writeback_mode = lagged_authoritative_with_mirror_cache`

### 6. Code evidence

- `include/iap/odometry/bspline_fixed_lag_registry.hpp:413-462`
- `src/iap/odometry/odometry_estimation_bspline.cpp:2020-2041`
- `src/iap/odometry/odometry_estimation_bspline.cpp:2193-2254`

关键点：

- `shared_state()` 只在 shared-singleton 模式下直接 seed/update bias/gravity
- `runtime_bias_source_of_truth_name()` 在 lagged mode 下明确返回 `active_lagged_bias_keys`
- `mirror_bias_cache_from_values()` 会把 solved lagged bias 同步到 `auxiliary_values()` 和 latest shared mirror
- `write_frame_bias_from_values()` 仍允许 frame copy 从 solved lagged bias 或 latest mirror 读值

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 继续减少 `auxiliary_values` / `shared_state` / `frame copy` 之间的隐式兜底
  - 尤其是让 postsolve evaluation 路径尽量少依赖重组容器

---

## M5. Pre-solve Query Surface / Seed Builder

### 1. Intended contract

- `start_pose` 来源必须唯一
- seed 必须被真实消费
- seed/query/support 必须落在同一个 strict-local pre-solve surface
- 该模块不应出现“seed 算在 A 时间面，solve 消费在 B 时间面”

### 2. Validation method

- 代码侧：
  - 查 `make_frontend_motion_seed()`
  - 查 `seed_frontend_local_values()`
  - 查 `query_time` 是否直接绑定 `input.frontend_target_time`
  - 查 `evaluate_pose_from_layout(..., query_time, Lidar, ...)`
- 运行侧：
  - 看 `start_pose_source_kind`
  - 看 `frontend_seed_mode/source/fallback/imu_sample_count`
  - 看 `frontend_target_time` / `seed_integration_end_time` / `bucket_query_time`
  - 看 target-time 一致性测试与 shadow diagnostics 测试
- 最低通过标准：
  - start pose 来源唯一
  - seed 真被 solve 消费
  - seed integration / query support / bucket query 共享同一 target-time 语义

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [x] start_pose source is unique
- [x] seed is actually consumed

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 seed 的真实消费链仍然成立，而且现在已经和 strict-local query/support 时间面绑定到同一个 `frontend_target_time`。
- 风险不再是“消费时间面不干净”，而是“在统一时间面下，`imu_forward_prediction` 的数值质量仍待单独评估”。

### 5. Runtime evidence

- `CTLocalFrontendSolve.FrontendTargetTimeContractIsExplicitAndConsistent` 同时覆盖 `last_pose_copy` 与 `imu_forward_prediction`
- 两种 seed mode 都断言：
  - `frontend_target_time == start_pose_query_time == frontend_pose_query_time == bucket_query_time`
  - `seed_integration_end_time == frontend_target_time`
  - `frontend_target_time_consistent == true`
- `CTLocalFrontendSolve.ImuForwardPredictionFallsBackWhenSeedSamplesMissing` 继续证明：
  - fallback 只改变 seed 来源，不改变 target-time contract
- `start_pose_frozen_before_factor_injection` / `start_pose_frozen_before_solver_update` 的既有前提没有被这轮改动破坏；本轮没有引入新的 postsolve 取巧路径

### 6. Code evidence

- `src/iap/odometry/ct_local_frontend.cpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `include/iap/odometry/ct_backend_summary.hpp`

关键点：

- `make_frontend_motion_seed()`：
  - IMU seed 样本按 `target_frame->stamp -> frontend_target_time` 归一化积分
  - `seed_integration_end_time` 显式记录为 `frontend_target_time`
- `CTLocalFrontend::run()`：
  - `seed_frontend_local_values()` 把 motion seed 写进 `result.local_values`
- `query_time`：
  - 不再本地重算，直接使用 `input.frontend_target_time`
  - 然后用 `evaluate_pose_from_layout(..., query_time, Lidar, ...)` 评估 `seed_pose`
- `make_frontend_input()`：
  - 无论 seed mode 是什么，都先显式生成 `frontend_target_time`
  - 当 IMU seed 启用时，`seed_imu_samples` 的终点改成 `frontend_target_time`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 在新的端到端 run 上分别跑 `last_pose_copy` 与 `imu_forward_prediction`
  - 如果仍有性能差异，就把问题归到 seed 数值/动力学质量，而不是 target-time contract

---

## M6. CT Local Frontend

### 1. Intended contract

- 消费 pre-solve seed、target snapshot、IMU/LiDAR 输入
- 输出 strict-local local solve 结果
- 不应把 debug seed 或 shadow path 当 authoritative result

### 2. Validation method

- 代码侧：
  - 查 `CTLocalFrontend::run()`
  - 查 `seeded_local_values`
  - 查 local LM optimize 前后的 pose diagnostics
- 运行侧：
  - 看 `start->frontend`
  - 看 `frontend_support_keys`
  - 看 `match_ratio` / `inlier_ratio`
- 最低通过标准：
  - 输入 seed 被真实消费
  - 输出确实是 strict-local frontend result

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [ ] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- frontend 现在已经不再被 `M1/M5` 的 target-time contract 直接阻塞。
- 当前剩余风险更像 correspondence / registration 质量问题，而不是“frontend 在错误时间面上消费 seed”。

### 5. Runtime evidence

- `ctest --test-dir build/iap -R 'test_ct_local_frontend|test_ct_hybrid_pipeline'` 通过
- `CTLocalFrontendSolve.FrontendTargetTimeContractIsExplicitAndConsistent` 与
  `CTLocalFrontendSolve.ShadowDiagnosticsConsumeProvidedQueryTimeAsFrontendTargetTime`
  共同证明：
  - frontend 真在消费提供的 seed
  - frontend 现在真在消费提供的 `frontend_target_time`
- `test_ct_hybrid_pipeline` 继续通过，说明把 target-time contract 显式化后，frontend 与系统主路径的基本耦合关系没有被破坏
- 目前还缺一轮新的 full-run 去给 `match_ratio` / `inlier_ratio` / `start->frontend residual` 提供新的实跑证据，所以本模块仍保留 `WITH_RISK`

### 6. Code evidence

- `src/iap/odometry/ct_local_frontend.cpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`

关键点：

- `motion_seed` 进入 `seed_frontend_local_values()`
- `seeded_local_values` 被用于 `seed_pose`
- `query_time` 现在显式取自 `input.frontend_target_time`
- `result.local_values` 被用于 `optimized_pose`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 跑一轮新的 full-run
  - 只针对 frontend 自己重新看 `match_ratio` / `inlier_ratio` / `target quality`，不要再把时间契约问题和 correspondence 质量混在一起

---

## M7. Graph Assembly

### 1. Intended contract

- 进入图的状态和 factor 身份必须和 runtime mode 一致
- reference 量不应被偷偷升级成 graph optimized state
- 已经收敛过的 gravity/velocity/bias 语义不应在这里被重新破坏

### 2. Validation method

- 代码侧：
  - 查 IMU factor 如何消费 gravity / bias
  - 查 velocity prior / factor 是否按 mode gating
  - 查 lagged bias transition prior 是否存在
- 运行侧：
  - 看 runtime mode summary
  - 看 `runtime_velocity_optimized`
  - 看 `runtime_bias_transition_prior_enabled`
- 最低通过标准：
  - 图里只优化被授权优化的状态

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- 图装配层目前基本遵守了已经收敛过的 gravity/velocity/bias 身份语义。
- 当前主要风险不再是“上游 seed 时间面不统一”，而是“下游 carry/boundary 放大”仍可能主导 residual。

### 5. Runtime evidence

- `ctest --test-dir build/iap -R 'test_ct_local_frontend|test_ct_hybrid_pipeline'` 通过
- `test_ct_hybrid_pipeline` 说明：
  - 在显式 `frontend_target_time` 接线之后，graph assembly 仍能完成主路径组装与基本集成
- `runtime_gravity_state_mode = external_reference`
- `runtime_velocity_state_mode = keep_but_not_optimize`
- `runtime_velocity_optimized = False`
- `runtime_bias_state_mode = lagged_keyed`
- `runtime_bias_transition_prior_enabled = True`
- `runtime_bias_transition_prior_strength = 1000`
- 但还没有新的 full-run 证明 target-time contract 修复后，graph residual 与 `M8` 链的关系是否发生结构变化

### 6. Code evidence

- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `src/iap/odometry/integrated_bspline_imu_factor.cpp`

关键点：

- IMU factor 在 `gravity_external_reference_enabled()` 时直接消费外部 gravity reference，不再用图中的 `g(0)`
- velocity prior 受 `velocity_explicit_optimization_for_solve` 和 `exp_disable_current_velocity_prior_` gating
- lagged bias 模式下，bias 用 first-key prior + lag-to-lag `BetweenFactor<Vector3>(Zero)` continuity prior
- incremental path 同样为 lagged bias 添加 continuity prior
- 本轮 `M1/M5` 修复没有改 graph state 身份，只是让 frontend/shadow query 的 target time 显式化

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 在修完 `M1/M5` 后重跑，确认图装配层当前的 residual 不是在被 seed 时间错位“连坐”

---

## M8. Carry / Survivor / Marginal Prior Bridge

### 1. Intended contract

- survivor 只保留必要信息
- carried prior 只桥接必须跨边界保留的约束
- 不应偷带过宽的 pose/control 误差
- bias 已经退出 boundary anchor 主线后，不应再有新的 boundary anchor 语义替代它

### 2. Validation method

- 代码侧：
  - 查 `build_bspline_marginalization_partition()`
  - 查 `build_bspline_carried_prior()`
  - 查 `bias_anchorable_survivor_keys()`
  - 查 jump diagnostics 如何生成 `carried_boundary_oldest` / `oldest_survivor`
- 运行侧：
  - 看 `postsolve_reason`
  - 看 `carried_boundary_oldest`
  - 看 `oldest_survivor`
  - 看 `frontend->postsolve_active_window`
  - 看 `active_window->strict_local`
- 最低通过标准：
  - survivor/carry 只保留必要状态
  - boundary shift 不应成为 top jump 的稳定主导解释

### 3. Checklist

- [x] input contract unique
- [ ] output contract unique
- [x] authoritative state unique
- [ ] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [ ] survivor keys keep only necessary information
- [x] bias is not the boundary anchor

### 4. Result summary

- `FAIL`
- bias 的 boundary-anchor 语义确实已经收敛了。
- 但 carried prior / survivor 仍保留了过宽的 pose/control 边界信息，当前 residual 主要还是在这里被放大。

### 5. Runtime evidence

- `top active-window reasons = boundary_shift:10`
- `frontend->postsolve_active_window p95 = 6.027 m`
- `frontend->postsolve_strict_local p95 = 0.459 m`
- `active_window->strict_local p95 = 5.800 m`
- `postsolve_active_window->strict_local rotation p95 = 2.598 rad`
- top jump frames 中：
  - `postsolve_reason = boundary_shift`
  - `oldest_survivor` 变成了 `s*`
  - `carried_boundary_oldest` 变成了 `c*`
- 说明：
  - bias 不再是 anchor
  - 但 boundary/carry 仍在主导 residual 放大

### 6. Code evidence

- `src/iap/odometry/bspline_marginalization.cpp:370-453`
- `src/iap/odometry/bspline_marginalization.cpp:456-470`
- `src/iap/odometry/odometry_estimation_bspline.cpp:2126-2128`
- `src/iap/odometry/odometry_estimation_bspline.cpp:3585-3595`
- `src/iap/odometry/odometry_estimation_bspline.cpp:6045-6048`
- `src/iap/odometry/odometry_estimation_bspline.cpp:7183-7186`

关键点：

- `build_bspline_marginalization_partition()` 直接把 `active_state_set.active_keys()` 作为 `survivor_keys`
- `build_bspline_carried_prior()` 对 `survivor_keys` 做线性化边缘化并整体保留
- `filter_bspline_survivor_anchor_keys()` 只负责移除 bias key，不会缩小 pose/control anchor 宽度
- jump diagnostics 的 `carried_boundary_oldest` / `oldest_survivor` 也是基于这条 retained-key 链

### 7. Verdict

- `FAIL`

### 8. If fail: blocking reason and minimum next fix

- 阻塞原因：
  - 当前 carried prior 仍然把过宽的 boundary support 带进 postsolve query
  - 这直接解释了 `boundary_shift` 为什么仍是 top jump 的统一特征
- 最小修复：
  - 收窄 survivor / carried prior 的 retained pose/control 集
  - 优先只保留 strict-local truly-needed support，而不是整段 active survivor keys

---

## M9. Fixed-Lag Incremental Solver

### 1. Intended contract

- solver 只消费 graph delta，不定义变量身份
- 只优化被 graph/模式授权优化的变量
- residual 放大若存在，也应能从 factor family / relinearization family 解释

### 2. Validation method

- 代码侧：
  - 查 `unified_graph_solver_->apply_delta(delta)`
  - 查 solver result 如何回传 `estimate_subset`
  - 查 solver profile 记录点
- 运行侧：
  - 看 `solver_update_profile`
  - 看：
    - `relinearized_variable_count`
    - `recalculated_imu_factor_count`
    - `recalculated_prior_factor_count`
    - `recalculated_shared_jkg_touching_factor_count`
- 最低通过标准：
  - solver 不偷偷改变状态身份
  - solver 放大机制可被 profile 解释

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 solver 看起来主要是在放大已经进入图/bridge 的问题，而不是重新定义状态身份。
- 但由于 `M8` 未通过，solver-side amplification 的根因解释仍然只能部分成立。

### 5. Runtime evidence

- `solver_update_ms_mean = 89.811`
- `dominant_recalculated_family = IMU`
- `relinearized_variable_vs_solver_update_corr = 0.8801`
- `recalculated_shared_jkg_touching_factor_vs_solver_update_corr = 0.6857`
- `strict_local residual more likely reflects solver-side orientation drift`

### 6. Code evidence

- `src/iap/odometry/odometry_estimation_bspline.cpp:7880-7959`
- `src/iap/odometry/odometry_estimation_bspline.cpp:8180-8245`

关键点：

- solver 入口是 `apply_delta(delta)`
- solver 返回 `estimate_subset`
- solver profile 行记录了 relinearization、recalculation、shared-jkg-touching 等统计

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 先修 `M8`
  - 再判断 solver 是否仍有独立于 carry/boundary 的 orientation amplification

---

## M10. Post-solve Write-back

### 1. Intended contract

- solve 结果写回路径必须唯一、清楚
- 临时 evaluation 值不能反向升级为下一轮 authoritative seed
- registry / control / aux / shared-state / frame copy 的更新顺序必须明确

### 2. Validation method

- 代码侧：
  - 查 `control_buffer.update_from_values`
  - 查 `update_shared_state_from_values`
  - 查 `mirror_bias_cache_from_values`
  - 查 `auxiliary_values` 重建
  - 查 `evaluation_values` 组装与 `write_frame_bias_from_values`
- 运行侧：
  - 看 runtime metadata：
    - `runtime_bias_writeback_mode`
    - `runtime_bias_source_of_truth`
  - 看 report 是否仍出现 bias anchor 回归
- 最低通过标准：
  - authoritative write-back 路径清晰
  - 临时值不会偷偷变成下一轮 seed 真值

### 3. Checklist

- [x] input contract unique
- [ ] output contract unique
- [x] authoritative state unique
- [ ] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [ ] write-back path is unique
- [ ] no temporary value becomes next authoritative seed

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 write-back 已经比旧 shared-state 时代可解释得多。
- 但 incremental path 里仍存在“solve subset -> registry mirrors -> evaluation_values -> frame copy”的多段重组链，语义复杂度偏高。

### 5. Runtime evidence

- `runtime_bias_writeback_mode = lagged_authoritative_with_mirror_cache`
- `runtime_bias_source_of_truth = active_lagged_bias_keys`
- `runtime_bias_can_be_survivor_anchor = False`
- 当前 report 没再显示 bias anchor 回归，但 residual 仍主要受 boundary/carry 放大影响

### 6. Code evidence

- `src/iap/odometry/odometry_estimation_bspline.cpp:2193-2254`
- `src/iap/odometry/odometry_estimation_bspline.cpp:7901-7959`
- `src/iap/odometry/odometry_estimation_bspline.cpp:8038-8062`

关键点：

- solver result 先写回 `control_buffer` / `control_window`
- 然后同步 shared mirror 与 bias mirror
- 清空并重建 `auxiliary_values`
- 再组 `evaluation_values`
- 再用 `evaluation_values` 生成 `new_frame`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 把 postsolve evaluation / publish 前使用的 value 容器进一步收敛
  - 尽量减少“先 mirror 再重组再 query”的语义跳转

---

## M11. Postsolve Query Surface Selector

### 1. Intended contract

- `final_pose_surface` 必须唯一
- `strict_local` / `active_window` 只做 surface 选择，不重新定义真值
- publish 前不应再次偷偷解释姿态

### 2. Validation method

- 代码侧：
  - 查 `evaluate_postsolve_layout_pose()`
  - 查 `select_final_pose_query()`
  - 查 final pose 写入 `new_frame`
- 运行侧：
  - 看 `runtime_final_pose_surface`
  - 看 `postsolve_active_window->strict_local`
  - 看 `postsolve_strict_local->final`
- 最低通过标准：
  - final pose surface 唯一
  - final pose 与 selected surface 一致

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 final pose 选择逻辑本身是清楚的：`strict_local` 已是唯一 final surface。
- 风险不在 selector 本身，而在 upstream 的 `active_window` / `strict_local` 差异仍然过大。

### 5. Runtime evidence

- `runtime_final_pose_surface = strict_local`
- `postsolve_active_window->strict_local translation p95 = 5.800 m`
- `postsolve_active_window->strict_local rotation p95 = 2.598 rad`
- `postsolve_strict_local->final = 0`

### 6. Code evidence

- `src/iap/odometry/odometry_estimation_bspline.cpp:825-970`
- `src/iap/odometry/odometry_estimation_bspline.cpp:8038-8062`
- `src/iap/odometry/odometry_estimation_bspline.cpp:6765-6787`

关键点：

- `evaluate_postsolve_layout_pose()` 对 layout+query_time 做显式查询
- `select_final_pose_query()` 只在 `active_window` 与 `strict_local` 之间二选一
- 当前 final pose 来自 selected strict-local query，不是 publish 前又做了一次新解释

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 继续保持 `strict_local` 默认 final surface
  - 优先修 `M8/M10`，而不是回退 selector

---

## M12. Publish / Frame Materialization

### 1. Intended contract

- publish 只消费 final truth
- `new_frame` / tf / publish copy 不应重新定义真值
- 输出可以作为下一轮 target frame 使用，但不应在过程中污染 registry authoritative state

### 2. Validation method

- 代码侧：
  - 查 `new_frame->T_world_lidar / T_world_imu / v_world_imu / imu_bias`
  - 查下一轮 frontend target frame 的来源
- 运行侧：
  - 看 `start_pose_source_kind`
  - 看 `new_frame` 派生字段是否与 final surface 对齐
- 最低通过标准：
  - publish/materialization 只复制 final truth，不再重解释

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [ ] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 publish/materialization 主要是在复制 selected final pose。
- 但 `new_frame` 会进入下一轮 `target_frame`，所以 materialized copy 仍是系统循环中的重要载体，不能完全当成“纯输出副本”。

### 5. Runtime evidence

- `start_pose_source_kind = pre_solve_strict_local_layout`
- 当前 final pose 使用 `strict_local`
- `postsolve_strict_local->final = 0`
- 说明 publish copy 与 selected final surface 是对齐的

### 6. Code evidence

- `src/iap/odometry/odometry_estimation_bspline.cpp:6773-6787`
- `src/iap/odometry/odometry_estimation_bspline.cpp:8038-8062`
- `src/iap/odometry/odometry_estimation_bspline.cpp:9042-9096`

关键点：

- `new_frame` 直接接收 selected final pose / velocity / bias
- 下一轮 frontend input 的 `target_frame` 来自 `frames.back()`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 在 `M10/M11` 收敛后，再确认 `new_frame` 作为 seed carrier 时没有携带多余的 surface 语义

---

## M13. Diagnostics / Reports / Audit

### 1. Intended contract

- diagnostics 只做观测
- report 结论必须与 runtime mode 一致
- 不能为了“看起来清楚”而改变主路径语义

### 2. Validation method

- 代码侧：
  - 查 `run_info` 初始化与 merge
  - 查 `ana_log.py` 如何读 `run_info` / `config_snapshot` / CSV
- 运行侧：
  - 看 report 中 runtime mode 与实际代码模式是否一致
  - 看 jump / solver / frontend summary 是否与 CSV 字段匹配
- 最低通过标准：
  - 诊断层只读，不反向改运行时

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS`
- 当前 diagnostics/report 层是被动观测层。
- 结论与 runtime mode 基本一致，没有看到“为报告方便而改主路径”的证据。

### 5. Runtime evidence

- report 中的：
  - `runtime_final_pose_surface`
  - `runtime_gravity_state_mode`
  - `runtime_bias_state_mode`
  - `runtime_frontend_seed_mode`
  - `runtime_velocity_state_mode`
  - 与 `run_info.json` 对齐
- report 还能复盘：
  - `boundary_shift`
  - `oldest_survivor`
  - `seed_source`
  - `pitch_vs_roll_dominance`

### 6. Code evidence

- `src/iap/common/log_paths.cpp:440-503`
- `src/iap/common/log_paths.cpp:671-681`
- `tools/ana_log.py:426-483`
- `tools/ana_log.py:4559-4597`

关键点：

- `run_info` 先写 config/runtime baseline
- `merge_run_info_metadata()` 再补 runtime resolved fields
- `ana_log.py` 只读 metadata 和 CSV，不回写主路径状态

### 7. Verdict

- `PASS`

### 8. If fail: blocking reason and minimum next fix

- 不适用

---

## Top 5 Answers

### 1. 当前第一个 FAIL 的模块是谁？

- `M8 Carry / Survivor / Marginal Prior Bridge`

原因：

- `M1/M5` 在本轮已经收敛到统一的 frontend target-time contract。
- 当前仍然稳定 `FAIL` 的最前一层，就是 `boundary_shift` 持续主导 residual 的 `M8`。

### 2. 当前最关键的阻塞链是哪个？

- `M8 -> M9 -> M10 -> M11`

解释：

- `M1 -> M5 -> M6` 在本轮之后已经不再是硬阻塞链
- 当前真正阻塞 residual / drift 继续往下分解的，是 `M8` 带头的 boundary/carry 放大链

### 3. 哪个模块一旦修好，最可能让后面一整串模块的判断重新有效？

- 现在最值的是 `M8 Carry / Survivor / Marginal Prior Bridge`。

原因：

- `M1/M5` 已经把 pre-solve target time 讲清楚了。
- 当前如果不先收窄 `M8` 的 survivor / carried prior anchor，`M9/M10/M11` 对 solver 放大、write-back、surface selector 的很多判断仍会继续被 boundary_shift 连坐。

### 4. 当前 residual / drift 最可能主要在哪一段被放大？

- 主要放大段在 `M8-M11`，尤其是：
  - `frontend -> postsolve_active_window`
  - `postsolve_active_window -> strict_local`

直接证据：

- `frontend->postsolve_active_window p95 = 6.027 m`
- `frontend->postsolve_strict_local p95 = 0.459 m`
- `active_window->strict_local p95 = 5.800 m`
- top jump frames 的主因统一是 `boundary_shift`

### 5. 为什么现在应该先修这个模块，而不是继续试新的局部想法？

- 因为现在 `M1/M5` 这条最早的契约失败已经被收敛掉了。
- 再继续试 seed 数值、局部 prior、或别的开关，只会重新把问题搅回“到底是 pre-solve 还是 boundary”这种混合归因。
- 当前路线更清楚了：
  1. 维持现在统一的 frontend target-time contract
  2. 下一刀直接修 `M8`
  3. 然后再重新判断 `M9/M10/M11` 到底还有多少独立问题
