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
| M8 Carry / Survivor / Marginal Prior Bridge | PASS_WITH_RISK | - | 追 `active_solve_keys -> carried_prior_retained_keys -> boundary_anchor_keys` 是否显式分离，并检查 carried prior 是否只保留 strict-local truly-needed support | 中 | 跑一轮新的端到端 run，确认 `retained_minus_strict_local_key_excess` 与 `boundary_shift` 同时下降 |
| M9 Fixed-Lag Incremental Solver | PASS_WITH_RISK | - | 检查 solver 是否只消费 graph delta，并结合 `solver_update_profile` 看 residual 放大族群 | 中 | 在新 run 上复核：若 `boundary_shift` 仍重，判断是否已转成 solver 被动放大而非 M8 本体 |
| M10 Post-solve Write-back | PASS_WITH_RISK | - | 追 `solver result -> postsolve_authoritative_values -> registry sinks / query / new_frame` 是否已经收敛为单一路径 | 中 | 跑一轮 fresh run，确认 `postsolve_value_source_consistent=true` 且 active-window/strict-local gap 明显下降 |
| M11 Postsolve Query Surface Selector | PASS_WITH_RISK | - | 检查 `active_window` / `strict_local` 是否共享同一 postsolve value source、同一 query time、同一 frame/extrinsic 语义，仅 layout/support 不同 | 中 | 若 fresh run 仍大幅分叉，优先转查 M12 或 orientation semantics，而不是回到 M8 |
| M12 Publish / Frame Materialization | PASS_WITH_RISK | - | 检查 `new_frame` 是否只消费 selected final truth，且不再成为 active-window surface 的隐式回流源 | 中 | 在 fresh run 上确认 `new_frame_consumes_final_truth_only=true` 且下一轮 start pose 不再受 active-window 污染 |
| M13 Diagnostics / Reports / Audit | PASS | - | 检查 report 是否已能直接显示 postsolve value source / frame convention / final materialization 语义 | 低 | 无 |

## Blocking Dependency Map

- `M0 FAIL -> block M1..M13`
- `M1 FAIL -> block M5/M6`，并使所有涉及 frontend seed / query-time 归因的下游判断降级
- `M2 FAIL -> block M5/M7/M10/M11/M12`
- `M5 FAIL -> block M6`，并弱化 M7 对 frontend 输入正确性的解释力
- `M8 FAIL -> block M9/M10/M11` 对 residual 放大机制的完整归因
- `M10 FAIL -> block M11/M12` 对 authoritative final state 的可信度
- `M11 FAIL -> block M12`
- `M12 FAIL -> block M13` 对 publish correctness 的判断

当前这一轮 Round-3 代码修复后，阻塞关系变成：

1. `M12 -> M13` 是唯一仍需 fresh-run 运行证据确认的后处理链
2. `M2 -> M10/M11/M12` 仍是架构层风险链，但不再是新的第一嫌疑

其中：

- `M1 -> M5 -> M6` 在本轮 target-time contract fix 后已经不再是硬阻塞链
- `M8` 代码契约已从“整段 active survivor set 整体保留”收敛到“strict-local support + current aux/nav retained keys”
- `M10/M11` 现在已经在代码上共享同一 `postsolve_authoritative_values` 和同一 `query_time`
- 但由于还没有 Round-3 fresh run，remaining residual 是否已经从 “value-source/query-materialization 语义问题” 进一步压缩到 `M12` 或 orientation semantics，仍需下一轮运行证据确认

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

## Round-2 Boundary Bridge Narrowing

### 修前 retained pose/control 集怎么定义

- `build_bspline_marginalization_partition()` 默认把 `active_state_set.active_keys()` 直接当成 `survivor_keys`
- `build_bspline_carried_prior()` 直接对这整组 survivor keys 线性化边缘化并整体保留
- `filter_bspline_survivor_anchor_keys()` 只会排除 bias anchor，不会缩小 pose/control retained 宽度

### 修后 retained pose/control 集如何收窄

- `active_solve_keys` 继续等于完整 active solve set，仍然用于 factor ownership / marginalization classification
- 新增 `BoundaryBridgeSelection` 与新的 `build_bspline_marginalization_partition(...)` overload
- `carried_prior_retained_keys` 现在只保留：
  - 当前 strict-local query 的 `query_support_keys`
  - 若 query support 不可用，则回退到最新 segment 的显式 `control_indices`
  - 当前 auxiliary index 上真正需要保留的 aux keys
  - navigation layer 明确要求保留的 retained keys
- `boundary_anchor_keys` 再从 `carried_prior_retained_keys` 派生，并继续排除 bias key

### strict-local truly-needed support 的定义

- 首选：`frontend_shadow_result.pose_diagnostics.query_support_keys`
- 回退：`current_segment.control_indices`
- 明确不再使用 `segment.active_control_indices` 作为 carried prior retained-set 的来源，因为它已经带有 active-window 宽支持面的语义污染

### survivor / carried prior / boundary anchor 现在如何区分

- `active_solve_keys`
  - 当前 fixed-lag solve 仍真正激活、仍用于 factor ownership 的完整 active state set
- `carried_prior_retained_keys`
  - carried prior 真正保留下来跨边界 replay 的窄集合
- `boundary_anchor_keys`
  - 用于 `oldest_survivor` 等边界 anchor 诊断的更窄集合

### 为什么这能直接缓解 `boundary_shift`

- 修前 `survivor_keys = active_state_set.active_keys()` 会把整段 active survivor window 直接塞进 carried prior
- 修后 carried prior 不再默认保留整段 active survivor set，而是围绕 strict-local truly-needed support 收窄
- 这一步先把 `M8` 的“整段保活”语义拿掉，让后续 residual 如果仍然很大，就能更干净地归到 `M10/M11`

## Round-3 Postsolve Materialization And Query-Surface Cleanup

### 修前路径

- incremental path 仍在走：
  - `solver_estimate_subset -> registry mirrors -> evaluation_values -> postsolve query -> new_frame`
- unified path 虽然更接近单一路径，但 `strict_local` 仍可能复用 pre-solve layout 语义
- `active_window` 与 `strict_local` 的主要差异已经不再来自 retained-set excess，而更像：
  - postsolve value source 不够显式
  - strict-local postsolve layout 没有在 solve 后重建

### 修后路径

- 现在两条主路径都收敛成：
  - `solver result -> postsolve_authoritative_values -> postsolve layouts/query -> final materialization -> mirror sinks`
- `control_buffer` / `control_window` 先接 solve 结果
- 再显式构造唯一的 `postsolve_authoritative_values`
- 再把 registry mirrors 当成只读 sink 同步，而不是 query source
- `new_frame` / deskew / bias / velocity / clock 全部只消费 `postsolve_authoritative_values`

### active_window / strict_local 现在共享什么

- 同一 `postsolve_authoritative_values`
- 同一 `query_time = raw_frame->stamp`
- 同一 frame convention：
  - `query_pose = world_to_lidar`
  - 比较姿态时统一右乘 `T_lidar_imu` 得到 `world_to_imu`
- 同一 extrinsic application 语义

### 两个 surface 现在还允许哪些差异

- 允许差异：
  - layout
  - support keys
- 不再允许差异：
  - value source
  - hidden write-back stage
  - frame convention
  - extrinsic application path

### 为什么这能直接缓解或解释 remaining boundary_shift

- 如果 Round-3 fresh run 后：
  - `postsolve_value_source_consistent = true`
  - `new_frame_consumes_final_truth_only = true`
  - 但 `active_window->strict_local` 仍然很大
- 那么剩余问题就更明确地不再属于 `M8` 或 “value source 混乱”，而更像：
  - `M12` materialization loop
  - 或 orientation semantics / extrinsic roll-pitch mismatch

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
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [x] survivor keys keep only necessary information
- [x] bias is not the boundary anchor

### 4. Result summary

- `PASS_WITH_RISK`
- 这轮代码已经把 `active_solve_keys`、`carried_prior_retained_keys`、`boundary_anchor_keys` 明确分开，而且 carried prior 不再默认保留整段 `active_state_set.active_keys()`。
- 当前剩余风险不在 M8 代码契约本身，而在还缺一轮新的 full-run 去证明 `boundary_shift` 与 active-window/strict_local 分叉已经随之下降。

### 5. Runtime evidence

- 代码级运行证据：
  - `ctest --test-dir build/iap -R 'test_bspline_marginalization|test_ct_hybrid_pipeline'` 通过
  - 新增 `PartitionCanNarrowCarriedPriorRetainedSetWithoutChangingSolveKeys`
    明确锁住：
    - `active_solve_keys` 仍等于完整 active solve set
    - `carried_prior_retained_keys` 可以显式窄于 active solve set
    - `boundary_anchor_keys` 继续排除 lagged bias keys
- report 兼容性证据：
  - `ana_log.py --run log/latest --no-plots --skip-external-tools` 通过
  - 新 report 已能输出：
    - `postsolve_reason_counts`
    - `retained_pose_control_key_count_*`
    - `strict_local_needed_support_key_count_*`
    - `retained_minus_strict_local_key_excess_*`
- 现有 `log/latest` 仍是修前 run，所以它依然显示旧现象：
  - `postsolve_reason_counts = boundary_shift:281, none:17`
  - `frontend->postsolve_active_window translation p95 = 17.459 m`
  - `postsolve_active_window->strict_local translation p95 = 17.928 m`
  - 这些旧数字说明修前问题真实存在，但不能再当作修后 verdict 的反证

### 6. Code evidence

- `include/iap/odometry/bspline_marginalization.hpp`
- `src/iap/odometry/bspline_marginalization.cpp`
- `include/iap/odometry/odometry_estimation_bspline.hpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `test/test_bspline_marginalization.cpp`

关键点：

- `BSplineMarginalizationPartition` 现在显式区分：
  - `active_solve_keys`
  - `strict_local_needed_support_keys`
  - `carried_prior_retained_keys`
  - `boundary_anchor_keys`
- `build_bspline_marginalization_partition(...)` 新 overload 接收窄 retained selection，但保留 `survivor_keys = active_solve_keys` 的 solve 语义
- `build_boundary_bridge_selection(...)` 负责把：
  - 当前 strict-local query support
  - fallback `current_segment.control_indices`
  - 当前 aux keys
  - nav retained keys
  收敛成 carried prior 真正保留的窄集合
- `update_marginal_prior_information(...)` 现在直接用 `carried_prior_retained_keys` 构建 carried prior
- jump diagnostics 的 `oldest_survivor` 已切到 `boundary_anchor_keys`，并新增 retained-vs-strict-local excess 指标

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 跑一轮新的 full-run
  - 重点看：
    - `retained_minus_strict_local_key_excess_mean/p95`
    - `postsolve_reason_counts`
    - `frontend->postsolve_active_window p95`
    - `postsolve_active_window->strict_local p95`
  - 如果 retained excess 已明显缩小而 divergence 仍大，就把第一嫌疑正式转给 `M10/M11`

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
- solver 仍然看起来主要是在放大已经进入图/bridge 的问题，而不是重新定义状态身份。
- 这轮 `M8` 代码契约已经收窄，所以 `M9` 不再被硬阻塞；但还需要新的 full-run 去判断 solver 放大是否已明显下降。

### 5. Runtime evidence

- 代码/测试证据：
  - `test_ct_hybrid_pipeline` 在 M8 收窄后继续通过，说明 solver 主路径没有被 carried prior retained-set 改动破坏
- 现有 full-run 证据仍来自修前日志：
  - `solver_update_ms_mean = 89.811`
  - `strict_local residual more likely reflects solver-side orientation drift`
  - 因为这批数字仍对应修前 run，所以它们现在只能作为“修前基线”，不能直接用于判定修后 solver 是否仍为主因

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
  - 在新的 run 上对比：
    - `boundary_shift_count/share`
    - `solver_update_ms_mean`
    - `reeliminated/relinearized/recalculated` 族群
  - 若 boundary 指标已降而 solver churn 仍高，再把下一刀落到 solver-side orientation cleanup

---

## M10. Post-solve Write-back

### 1. Intended contract

- solve 结果必须先收敛成唯一的 `postsolve_authoritative_values`
- registry mirrors / `auxiliary_values` / shared-state mirrors 只能是 write-back sinks
- query/materialization 不应再各自重组一份新的 postsolve surface

### 2. Validation method

- 代码侧：
  - 查 `build_postsolve_authoritative_values()`
  - 查 `sync_postsolve_registry_mirrors()`
  - 查 unified / incremental 两条路径是否都先构建 `postsolve_context`
  - 查 `new_frame` / deskew / bias / velocity / clock 是否都只读 `postsolve_context.authoritative_values`
- 运行侧：
  - 看 runtime metadata：
    - `runtime_postsolve_active_window_value_source_kind`
    - `runtime_postsolve_strict_local_value_source_kind`
    - `runtime_postsolve_value_source_consistent`
    - `runtime_final_materialization_source_kind`
    - `runtime_new_frame_consumes_final_truth_only`
  - 看 report 是否仍出现 bias anchor 回归
- 最低通过标准：
  - authoritative postsolve value source 唯一
  - query/materialization 不再吃到 mirror/fallback container

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient
- [x] write-back path is unique
- [x] no temporary value becomes next authoritative seed

### 4. Result summary

- `PASS_WITH_RISK`
- 当前两条 solver path 都已经显式收敛到：
  - `solver result -> postsolve_authoritative_values -> query/materialization -> mirror sinks`
- `M10` 的主要语义风险已经明显下降。
- 剩余风险在于：还缺一轮 fresh run 去证明新字段与 residual 下降一起出现，而不是只有代码契约干净。

### 5. Runtime evidence

- M8 fresh run 已显示：
  - `retained_pose_control_key_count = 4`
  - `strict_local_needed_support_key_count = 4`
  - `retained_minus_strict_local_key_excess = 0`
- 但修前 run 仍有：
  - `frontend->postsolve_active_window translation p95 = 8.004 m`
  - `frontend->postsolve_strict_local translation p95 = 0.549 m`
  - `postsolve_active_window->strict_local translation p95 = 7.823 m`
- 这正是本轮要收敛的前提证据；Round-3 新字段还需要 fresh run 才能变成运行侧确认

### 6. Code evidence

- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- [odometry_estimation_bspline.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/odometry_estimation_bspline.hpp)

关键点：

- `build_postsolve_authoritative_values()` 先把 solve result 覆盖到完整 active state 上
- `sync_postsolve_registry_mirrors()` 只把这份 authoritative values 同步到 registry sinks
- unified 和 incremental 两条路径都改成先构建 `postsolve_context`
- `new_frame`、`deskewed_source_points()`、`write_frame_bias_from_values()`、clock/velocity materialization 都只读 `postsolve_context.authoritative_values`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 跑一轮 fresh run
  - 若 `postsolve_value_source_consistent=true` 但 `active_window->strict_local` 仍高，则下一刀转查 `M11/M12` 而不是回到 M10

---

## M11. Postsolve Query Surface Selector

### 1. Intended contract

- `final_pose_surface` 必须唯一
- `strict_local` / `active_window` 只能共享同一 authoritative postsolve value source
- publish 前不应再次偷偷解释姿态

### 2. Validation method

- 代码侧：
  - 查 `evaluate_postsolve_layout_pose()`
  - 查 `build_postsolve_evaluation_context()`
  - 查 `select_final_pose_query()`
  - 查 `strict_local_layout` 是否在 postsolve 后重建，而不是复用 pre-solve layout
- 运行侧：
  - 看 `runtime_final_pose_surface`
  - 看：
    - `runtime_postsolve_active_window_value_source_kind`
    - `runtime_postsolve_strict_local_value_source_kind`
    - `runtime_postsolve_query_frame_convention_kind`
    - `runtime_postsolve_extrinsic_application_kind`
  - 看 `postsolve_active_window->strict_local`
  - 看 `postsolve_strict_local->final`
- 最低通过标准：
  - final pose surface 唯一
  - active-window / strict-local 只允许 layout/support 不同，不再允许 value source / frame convention / extrinsic path 不同

### 3. Checklist

- [x] input contract unique
- [x] output contract unique
- [x] authoritative state unique
- [x] key invariant holds
- [x] runtime evidence sufficient
- [x] code evidence sufficient

### 4. Result summary

- `PASS_WITH_RISK`
- 当前 final selector 仍然干净，`strict_local` 继续是唯一 final surface。
- 本轮真正的变化是：
  - active-window 与 strict-local 现在在代码上已经共享同一 `postsolve_authoritative_values`
  - 同一 `query_time = raw_frame->stamp`
  - 同一 frame convention / extrinsic application
- 因此 M11 剩余风险已经从“多条隐式 query 链”缩小到“layout/support 差异本身是否仍然过大”。

### 5. Runtime evidence

- `runtime_final_pose_surface = strict_local`
- M8 fresh run 仍显示修前基线：
  - `postsolve_active_window->strict_local translation p95 = 7.823 m`
  - `postsolve_active_window->strict_local rotation p95 = 2.181 rad`
- `postsolve_strict_local->final = 0`
- 这说明 selector 一直都没有把 final truth 搞混；Round-3 fresh run 需要回答的是：共享统一 postsolve value source 之后，这个 gap 是否明显缩小

### 6. Code evidence

- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)
- [odometry_estimation_bspline.hpp](/home/dev/code/ws_iap/src/iap/include/iap/odometry/odometry_estimation_bspline.hpp)

关键点：

- `evaluate_postsolve_layout_pose()` 对 layout+query_time 做显式查询
- `select_final_pose_query()` 只在 `active_window` 与 `strict_local` 之间二选一
- `build_postsolve_evaluation_context()` 统一了 value source / query time / frame convention / extrinsic application
- `strict_local_layout` 改成在 postsolve 后由 `create_segment_lidar_layout(current_postsolve_segment)` 重建，不再复用 pre-solve strict-local layout

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 继续保持 `strict_local` 默认 final surface
  - 若 fresh run 里 `postsolve_value_source_consistent=true` 但 gap 仍大，下一刀改查 `M12` 或 orientation semantics / extrinsic roll-pitch mismatch

---

## M12. Publish / Frame Materialization

### 1. Intended contract

- publish 只消费 final truth
- `new_frame` / tf / publish copy 不应重新定义真值
- 输出可以作为下一轮 target frame 使用，但不应在过程中污染 registry authoritative state

### 2. Validation method

- 代码侧：
  - 查 `new_frame->T_world_lidar / T_world_imu / v_world_imu / imu_bias`
  - 查 `final_materialization_source_kind`
  - 查 `new_frame_consumes_final_truth_only`
  - 查下一轮 frontend target frame 的来源
- 运行侧：
  - 看：
    - `runtime_final_materialization_source_kind`
    - `runtime_new_frame_consumes_final_truth_only`
    - `postsolve_strict_local->final`
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
- 当前 `new_frame` 在代码上已经被收敛成：
  - 只消费 selected final truth
  - 不再回读 active-window postsolve surface
- 但因为 `new_frame` 会进入下一轮 target frame，M12 仍需要 fresh run 来证明它没有把旧 surface 语义重新带回系统循环。

### 5. Runtime evidence

- 当前已有基线证据：
  - `runtime_final_pose_surface = strict_local`
  - `postsolve_strict_local->final = 0`
- Round-3 新增的：
  - `runtime_final_materialization_source_kind`
  - `runtime_new_frame_consumes_final_truth_only`
  还需要 fresh run 才能成为运行侧确认

### 6. Code evidence

- [odometry_estimation_bspline.cpp](/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp)

关键点：

- `new_frame` 的 pose / velocity / bias / clock 都从 `selected_final_pose_query` 与 `postsolve_context.authoritative_values` materialize
- 下一轮 frontend input 的 `target_frame` 来自 `frames.back()`

### 7. Verdict

- `PASS_WITH_RISK`

### 8. If fail: blocking reason and minimum next fix

- 当前未到 `FAIL`
- 最小下一修复建议：
  - 跑一轮 fresh run
  - 若 `new_frame_consumes_final_truth_only=true` 但下一轮仍出现 surface 语义回流，再单独把剩余问题落到 orientation semantics / extrinsic mismatch

---

## M13. Diagnostics / Reports / Audit

### 1. Intended contract

- diagnostics 只做观测
- report 结论必须与 runtime mode 一致
- 不能为了“看起来清楚”而改变主路径语义

### 2. Validation method

- 代码侧：
  - 查 `run_info` 初始化与 merge
  - 查 `ana_log.py` 是否已新增：
    - postsolve value source summary
    - postsolve frame convention / extrinsic application summary
    - final materialization source summary
- 运行侧：
  - 看 report 中 runtime mode 与实际代码模式是否一致
  - 看 report 是否能直接解释：
    - active_window 与 strict_local 的差异来自哪里
    - value source 是否一致
    - new_frame 是否只消费 final truth
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
- 当前 diagnostics/report 层继续保持只读。
- 本轮已经把 M10/M11 所需的新 runtime 语义字段接到：
  - `run_info`
  - `jump_diagnostics`
  - `analysis/report`
- 因此 M13 现在已经能更直接地区分：
  - value-source 混乱
  - query-surface/layout 差异
  - final materialization 语义

### 5. Runtime evidence

- report 中的：
  - `runtime_final_pose_surface`
  - `runtime_gravity_state_mode`
  - `runtime_bias_state_mode`
  - `runtime_frontend_seed_mode`
  - `runtime_velocity_state_mode`
  - 与 `run_info.json` 对齐
- 本轮又新增：
  - `runtime_postsolve_active_window_value_source_kind`
  - `runtime_postsolve_strict_local_value_source_kind`
  - `runtime_postsolve_value_source_consistent`
  - `runtime_postsolve_query_frame_convention_kind`
  - `runtime_postsolve_extrinsic_application_kind`
  - `runtime_final_materialization_source_kind`
  - `runtime_new_frame_consumes_final_truth_only`

### 6. Code evidence

- [log_paths.cpp](/home/dev/code/ws_iap/src/iap/src/iap/common/log_paths.cpp)
- [ana_log.py](/home/dev/code/ws_iap/src/iap/tools/ana_log.py)
- [README_blocking_module_audit.md](/home/dev/code/ws_iap/src/iap/docs/README_blocking_module_audit.md)

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

- 当前这轮只重审 `M10/M11/M12/M13` 后，没有新的硬 `FAIL`。
- 当前最先需要 fresh-run 运行证据确认的，不再是 `M8`，而是 `M12 Publish / Frame Materialization` 的闭环语义。

原因：

- `M10/M11` 在代码上已经统一到同一 `postsolve_authoritative_values`
- `M12` 才是这条链真正把 final truth 送进下一轮循环的最后一跳
- 所以如果 fresh run 之后 residual 仍然大，最值得优先核实的是：`new_frame` 是否真的只消费 final truth

### 2. 当前最关键的阻塞链是哪个？

- 当前最关键的高风险链已经从：
  - `M8 -> M9 -> M10 -> M11`
  收缩为：
  - `M12 -> M13`

解释：

- `M1 -> M5 -> M6` 之前已经解阻
- `M8` retained-set excess 已经到 0
- `M10/M11` 这轮已经在代码上把 value source / query time / frame convention / extrinsic path 统一掉
- 如果 fresh run 里 gap 还在，下一嫌疑就更像 `M12` 或 orientation semantics

### 3. 哪个模块一旦修好，最可能让后面一整串模块的判断重新有效？

- 现在最值的下一刀，已经从 `M8/M10/M11` 转到 `M12 Publish / Frame Materialization`。

原因：

- `M10/M11` 现在已经把 “solve result 到 postsolve query”的语义收敛清楚
- 真正会把姿态物化成下一轮 target frame 的，是 `new_frame`
- 因此如果 fresh run 之后 residual 仍不够好，修好 `M12` 最可能让后续判断重新变得干净

### 4. 当前 residual / drift 最可能主要在哪一段被放大？

- 现有 fresh run 里，主要放大段仍然在 postsolve surfaces 上，尤其是：
  - `frontend -> postsolve_active_window`
  - `postsolve_active_window -> strict_local`

直接证据：

- `frontend->postsolve_active_window p95 = 8.004 m`
- `frontend->postsolve_strict_local p95 = 0.549 m`
- `active_window->strict_local p95 = 7.823 m`
- `postsolve_reason_counts = boundary_shift:261, none:17`

补充说明：

- retained-set excess 已经是 0，所以这段放大更不像 M8
- Round-3 fresh run 将决定：这段放大是否已经从 “value/query 语义链” 真正进一步压缩到 `M12` 或 orientation semantics

### 5. 为什么现在应该先修这个模块，而不是继续试新的局部想法？

- 因为现在：
  - `M1/M5` 已经收敛
  - `M8` retained-set excess 已经到 0
  - `M10/M11` 也已经把 postsolve authoritative source 和 query tuple 统一掉
- 再去试新的 seed 数值、局部 prior、或新的实验开关，只会重新把问题搅回混合归因。
- 当前路线更清楚了：
  1. 跑一轮 Round-3 fresh run
  2. 用新增的 value-source / materialization 字段确认 `M10/M11` 是否已经真正干净
  3. 如果它们已经干净但 gap 仍大，下一刀就明确落到 `M12` 或 orientation semantics，而不再回头重猜 `M8/M10/M11`
