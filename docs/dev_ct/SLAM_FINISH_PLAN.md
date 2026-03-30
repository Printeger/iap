# IAP Continuous-Time SLAM Finish Plan

## 更新时间
- 2026-03-30

## 文档目的
- 将当前连续时间 `LiDAR + IMU + GNSS` SLAM 从“最小可用骨架”推进到“工程化完备版本”。
- 在 SLAM 主线完备前，冻结新的 planner 功能扩张，只保留现有连续时间规划接口的兼容维护。
- 为后续真正的 B-spline 规划器提供稳定、可信、可验证的定位底座。

## 当前基线
- 已具备 `OdometryEstimationBSpline`、active spline window、多 segment LiDAR factor、IMU factor、GNSS factor、velocity state、clock state、continuous trajectory publishing。
- 当前系统已经可以运行最小可用的 spline-native `LiDAR + IMU + GNSS` 联合优化骨架。
- `CT_LIDAR_GPU` 已形成双后端结构：
  - `BUCKET` 已退为内部过渡基线，不再是公开 runtime 路线
  - `KERNEL` 已完成第一版可运行 MVP，并接入 active-window odometry 主链与统一 result/profile/baseline surface
- 连续时间 odometry 求解器重构现额外受 [README_REFACTOR_CT_SOLVER.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/README_REFACTOR_CT_SOLVER.md) 约束：
  - 公开配置默认只保留 `KERNEL`
  - `BUCKET` 只允许内部 parity / 一次性 A/B 使用
  - 后续要继续从“每帧 full active-window batch graph”迁移到更接近 GLIM-style incremental fixed-lag + local CT solve domain 的组织方式
  - `BSplineIncrementalSolverSkeleton` 已开始显式追踪 active/new/retired CT solve-domain segments 与 key lifecycle，作为后续长期存活 fixed-lag solver owner 的兼容壳
- 当前主要缺口仍在：
  - 还不是最终的 fixed-lag 主状态组织。
  - LiDAR / IMU / GNSS 因子仍有“最小可用实现”成分。
  - marginal prior 仍是工程近似，不是成熟的 Schur 边缘化回灌。
  - 多类 Jacobian 仍是数值形式。
  - `gnss_extension` 仍保留部分长期 ownership。
  - mapping / 长时运行 / 退化恢复 / benchmark 还没有形成封板级验收。
  - GPU odometry 还没有完成 `cached BUCKET vs KERNEL`、`KERNEL runtime vs diagnostic` 的同配置 A/B，也还没有完成 `KERNEL` 路径下 shared GNSS state 生命周期的长包验收。

## 总体目标
1. 让 `OdometryEstimationBSpline` 成为连续时间 SLAM 的主 odometry，而不是局部 frontend 过渡层。
2. 让 LiDAR、IMU、GNSS 直接约束同一套 fixed-lag B-spline 控制点窗口，并具备稳定的边缘化与先验回灌。
3. 保持现有 `EstimationFrame` 兼容输出不变，让 `sub_mapping`、`global_mapping`、viewer 继续工作。
4. 形成可重复的验证闭环：单测、集成回放、长时稳定性、精度、耗时、内存都可量化。
5. 在此基础上再进入真正的 B-spline candidate planning。

## 完成定义
- `CT_LIDAR_CPU` 不再被视为“最小可用 local frontend”，而是默认连续时间 odometry 主线。
- LiDAR / IMU / GNSS 的关键因子都具备稳定 Jacobian、初始化、退化处理和测试覆盖。
- old control points 移出 lag window 时，已有稳定的边缘化信息回灌机制。
- `iap_rosnode` 使用连续时间配置可稳定运行，`sub_mapping` / `global_mapping` / viewer 兼容通过。
- 完成连续时间 SLAM 的 benchmark 与 A/B 对比，并形成结果文档。

## 执行原则
- 先完成 SLAM 主线，再做新的 planner 功能。
- 先按 [README_REFACTOR_CT_SOLVER.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/README_REFACTOR_CT_SOLVER.md) 收口连续时间 odometry 求解器，再继续做其余 GPU / benchmark 封板项。
- 每个工作包都必须包含：
  - 代码改动
  - 配置改动
  - 测试改动
  - 文档改动
  - 验收命令
- 优先保证状态组织和信息流正确，再追求 GPU 和性能优化。
- 所有新功能默认保持对 legacy discrete / legacy CT-GICP 路径的并存兼容。

## 里程碑

### M0：开发冻结线
目标：
- 冻结新的 planner feature 开发，只允许修复现有 continuous-time interface 的兼容问题。
- 明确后续开发全部围绕“SLAM 完备化”展开。

交付物：
- 本文档生效。
- `docs/dev_ct/dev_status.md` 的后续更新以本计划为准。

完成标准：
- 后续任务按本文的工作包执行，不再新增与 SLAM 完备无关的 planner 扩张项。

### M1：Fixed-Lag 主状态收口
目标：
- 将当前 active spline window 从“多段联合 LM frontend”推进为真正的 fixed-lag 主状态组织。

核心任务：
- 统一 control point、velocity、clock、bias、gravity、ECEF anchor 的生命周期与 key/stamp 管理。
- 明确窗口推进策略：
  - knot 插入
  - segment 退休
  - old state 边缘化
  - new state 初值生成
- 收敛 `RECONSTRUCT` 和 `CT_LIDAR_CPU` 的职责边界：
  - `RECONSTRUCT` 保留为 legacy/analysis path
  - `CT_LIDAR_CPU` 升级为正式主线
- 将当前工程化 boundary prior 逐步替换成更接近真实边缘化的信息先验。

建议涉及文件：
- `include/iap/odometry/odometry_estimation_bspline.hpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `include/iap/odometry/bspline_control_window.hpp`
- `src/iap/odometry/bspline_control_window.cpp`

完成标准：
- 文档中“这还是 local frontend，不是最终的 fixed-lag smoother 主链”可删除。
- old states 离窗后不会导致图结构不稳定或先验断裂。
- 有新的单测覆盖窗口推进和边缘状态处理。

### M2：LiDAR Continuous-Time 因子工程化
目标：
- 让 LiDAR 因子从最小可用版变成长期可维护版本。

核心任务：
- 明确并实现最终 target 策略：
  - 保留 snapshot 还是切换为更稳定的 local submap target
  - target 更新频率与 frozen target 的生命周期
- 用解析 Jacobian 替代当前数值 pose Jacobian，或至少完成严格的数值校验基线。
- 完善 correspondence / outlier / robust kernel 策略。
- 统一 deskew、source time query、target covariance 使用方式。
- 增加 profiling hook，为后续 GPU 版复用做接口准备。
- 冻结 `BUCKET` 为稳定基线，并把 `KERNEL` 推进成真正可验收的 GPU 主线后端。
- 保持 `runtime mode` 与 `diagnostic mode` 的显式边界：
  - runtime 默认只回收当前 factor
  - diagnostic 才执行整窗 result / CSV / numeric audit / degeneracy

建议涉及文件：
- `include/iap/odometry/integrated_bspline_gicp_factor.hpp`
- `src/iap/odometry/integrated_bspline_gicp_factor.cpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`

完成标准：
- LiDAR 因子不再依赖数值 pose Jacobian。
- 新增单测覆盖 Jacobian 正确性和 correspondence 稳定性。
- replay 下连续时间 LiDAR 因子的收敛质量不低于当前最小版本。
- `cached BUCKET` 与 `KERNEL` 可在同一配置下导出统一 baseline CSV 并完成直接 A/B。
- `KERNEL runtime` 的 `post_lidar_result_ms` 明显低于 `KERNEL diagnostic`。

### M3：IMU Continuous-Time 约束完成
目标：
- 将 IMU 从“sample-based 过渡实现”推进到完整的连续时间惯导约束。

核心任务：
- 确认最终状态布局：
  - velocity 是否继续保留为显式状态
  - 还是逐步收敛到由 spline 导数统一承载
- 完善 gyro / accel 残差定义和尺度归一化。
- 完成 bias / gravity / velocity 的解析 Jacobian 或高质量数值校验框架。
- 完成初始化与 observability 处理：
  - 静止初始化
  - 重力方向初始化
  - bias warm start
- 明确 IMU 与 LiDAR scan 时间域对齐策略。

建议涉及文件：
- `include/iap/odometry/integrated_bspline_imu_factor.hpp`
- `src/iap/odometry/integrated_bspline_imu_factor.cpp`
- `include/iap/odometry/integrated_bspline_velocity_factor.hpp`
- `src/iap/odometry/integrated_bspline_velocity_factor.cpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`

完成标准：
- IMU 因子从“最小可用 sample factor”升级为稳定主链约束。
- bias / gravity / velocity 的状态与先验组织明确且可重复运行。
- 新增单测覆盖 Jacobian、bias 估计、gravity 失配和初始化场景。

### M4：GNSS 连续时间主链完全下沉
目标：
- 将 GNSS 从“最小接线版”推进到连续时间 odometry 的真正内核组件。

核心任务：
- 明确 `gnss_extension` 的最终边界：
  - 只保留 ROS ingress
  - 不再承担 continuous-time 主链 ownership
- 将 raw batch、ephemeris、iono、anchor、epoch build、handler 统一纳入 `OdometryEstimationBSpline` 主链逻辑。
- 完善 GNSS 因子：
  - pseudorange Jacobian
  - doppler Jacobian
  - clock prior / between model
  - epoch-window 对齐策略
  - satellite visibility / elevation / validity gating
- 验证 GNSS 缺失、恢复、弱几何、多普勒稀疏等退化场景。

建议涉及文件：
- `include/iap/gnss/gnss_epoch_builder.hpp`
- `src/iap/gnss/gnss_epoch_builder.cpp`
- `include/iap/gnss/gnss_handler.hpp`
- `src/iap/gnss/gnss_handler.cpp`
- `include/iap/gnss/gnss_extension.hpp`
- `src/iap/gnss/gnss_extension.cpp`
- `include/iap/odometry/integrated_bspline_gnss_factor.hpp`
- `src/iap/odometry/integrated_bspline_gnss_factor.cpp`
- `src/iap/odometry/odometry_estimation_bspline.cpp`

完成标准：
- `gnss_extension` 被稳定收缩为 ROS ingress + legacy bridge。
- GNSS ownership 在 continuous-time 主链上单一清晰。
- GNSS 因子支持长时运行和恢复场景，不再只是最小接线。

### M5：系统集成与兼容封板
目标：
- 在不重写 mapping 的前提下，完成连续时间 odometry 与现有 SLAM 其余链路的系统级打通。

核心任务：
- 稳定输出兼容 `EstimationFrame`：
  - pose
  - sampled `imu_rate_trajectory`
  - deskewed frame
  - clock
  - sigma
  - ICP quality
- 跑通并验证：
  - `sub_mapping`
  - `global_mapping`
  - viewer
  - loop / pose graph 兼容链路
- 做典型数据集 replay：
  - LiDAR + IMU
  - LiDAR + IMU + GNSS
  - GNSS 中断 / 恢复
  - 低纹理 / 重复结构 / 林地场景

建议涉及文件：
- `src/iap/mapping/sub_mapping.cpp`
- `src/iap/mapping/global_mapping.cpp`
- `src/iap/viewer/...`
- `src/iap/odometry/odometry_estimation_bspline.cpp`

完成标准：
- `iap_rosnode` 可用连续时间配置稳定启动并长时运行。
- mapping / viewer 不需要为连续时间 odometry 做破坏性修改。
- 兼容输出与轨迹质量达到可交付状态。

### M6：鲁棒性、恢复与生命周期
目标：
- 完成连续时间 SLAM 在真实运行中的状态机与恢复逻辑。

核心任务：
- 冷启动策略：
  - 无 GNSS
  - GNSS 延迟到达
  - IMU 初始化不足
- 退化检测与 fallback：
  - LiDAR 退化
  - GNSS 几何恶化
  - IMU 异常
- 恢复策略：
  - GNSS 重新接入
  - bias/clock 重新稳定
  - target snapshot 失效刷新
- 增加诊断输出：
  - 当前窗口规模
  - 因子数量
  - 边缘化统计
  - 模块状态

完成标准：
- 能处理典型传感器中断和恢复场景。
- 状态变化有明确日志和可观测指标。

### M7：性能、GPU 与 Benchmark
目标：
- 在正确性稳定后完成性能基线、GPU odometry 封板，以及下一阶段更深层 kernel 优化准备。

核心任务：
- 基准对比：
  - legacy discrete
  - legacy CT-GICP
  - new B-spline CT
- 统计：
  - 前端耗时
  - 每类因子耗时
  - 边缘化耗时
  - 内存占用
- 明确 GPU 路线：
  - `BUCKET` 作为稳定基线的长期定位
  - `KERNEL` 的 kernel-stage baseline、numeric parity 和 runtime/diagnostic 双模式
  - 是否还需要继续推进 kernel 级 correspondence / reduction 优化

完成标准：
- 有 benchmark 表格和 replay 数据支撑。
- `cached BUCKET vs KERNEL`、`KERNEL runtime vs diagnostic` 的 A/B 数据完整。
- GPU 路线不再停留在方向性讨论，而是有已接入主线的 `KERNEL` backend 和后续优化清单。

### M8：封板文档与发布前验收
目标：
- 对连续时间 SLAM 做最终技术归档和发布前检查。

核心任务：
- 更新：
  - `docs/dev_ct/dev_status.md`
  - `docs/dev_ct/PLANS.md`
  - `docs/CHANGES.md`
  - `docs/TRACEABILITY.md`
  - `docs/methodology/methodology.tex`
  - 参数文档与运行文档
- 形成验收清单：
  - 单测
  - 集成测试
  - 回放脚本
  - benchmark
  - 已知风险

完成标准：
- 连续时间 SLAM 可作为“规划前冻结版本”进入下一阶段。

## 工作包拆分

### WP1：状态组织与边缘化
优先级：P0

任务：
- 明确主状态集合与 key namespace
- 统一 stamp / lag / prune / marginalize 流程
- 替换工程近似 prior

依赖：
- 无

验收：
- 新增窗口推进与边缘化测试
- 回放运行稳定

### WP2：LiDAR 因子工程化
优先级：P0

任务：
- 解析 Jacobian
- target 策略收敛
- correspondence 与 robust handling
- 冻结 `BUCKET` 为稳定工程基线
- 完成 `KERNEL` backend 的最小可运行实现并接入统一 result/profile/baseline surface
- 形成 `cached BUCKET vs KERNEL`、`runtime vs diagnostic` 的直接 A/B 能力

依赖：
- WP1

验收：
- Jacobian 测试
- A/B 精度与耗时对比
- CUDA smoke test、current-factor numeric parity、target-refresh 生命周期测试

### WP3：IMU 因子完成
优先级：P0

任务：
- 最终状态布局
- 解析 Jacobian
- 初始化与 observability

依赖：
- WP1

验收：
- IMU 残差与状态单测
- 长时 replay 不发散

### WP4：GNSS 主链收口
优先级：P0

任务：
- ownership 单一化
- Jacobian / clock / gating 完善
- 延迟、缺失、恢复场景验证

依赖：
- WP1

验收：
- GNSS 回放与中断恢复测试

### WP5：系统集成
优先级：P1

任务：
- mapping / viewer / rosnode 联调
- 兼容输出校验

依赖：
- WP1, WP2, WP3, WP4

验收：
- 端到端运行通过

### WP6：鲁棒性与恢复
优先级：P1

任务：
- 生命周期状态机
- fallback / recovery
- 诊断输出

依赖：
- WP2, WP3, WP4

验收：
- 退化与恢复场景通过

### WP7：性能与 GPU 路线
优先级：P2

任务：
- benchmark
- profile
- GPU 接口设计

依赖：
- WP5

验收：
- 形成可引用性能结果

### WP8：封板文档
优先级：P1

任务：
- 文档收口
- 方法学、追踪、变更、运行说明同步

依赖：
- 全部工作包

验收：
- 文档与代码一致

## 建议提交顺序
1. `feat(dev-ct-mainline): unify bspline fixed-lag state ownership`
2. `feat(dev-ct-lidar): replace numeric lidar jacobians and refine targets`
3. `feat(dev-ct-imu): finalize spline imu state layout and jacobians`
4. `feat(dev-ct-gnss): complete gnss ownership migration into bspline odometry`
5. `feat(dev-ct-integration): validate ct odometry with mapping and rosnode`
6. `feat(dev-ct-robustness): add lifecycle, fallback, and recovery logic`
7. `feat(dev-ct-benchmark): add replay benchmarks and gpu migration notes`
8. `docs(dev-ct): close slam finish documentation and acceptance`

## 验证矩阵

### 单元测试
- spline query / knot / window 推进
- LiDAR Jacobian / residual
- IMU Jacobian / bias / gravity / velocity
- GNSS epoch builder / factor / clock / queue / gating
- marginalization / prior continuity

### 集成测试
- `LiDAR + IMU`
- `LiDAR + IMU + GNSS`
- GNSS 中断 / 恢复
- 长时 replay
- viewer / mapping 兼容

### 性能测试
- 每帧总耗时
- LiDAR factor 耗时
- IMU factor 耗时
- GNSS factor 耗时
- marginalization 耗时
- 内存占用

## 暂不纳入本计划
- 真正的 B-spline candidate planning
- continuous-time local mapping / global mapping 重写
- GPU continuous-time LiDAR factor 的正式实现
- 非 `LiDAR + IMU + GNSS` 传感器组合扩张

## 风险清单
- 当前 velocity 是否长期保留为独立状态，仍需通过精度和稳定性验证决定。
- LiDAR target 策略如果切换过快，可能破坏当前 replay 稳定性。
- GNSS ownership 下沉过快，可能影响 legacy diagnostics 和完整性链路。
- 边缘化回灌如果实现不稳，会直接导致长窗运行发散。
- 若在主状态组织未稳定前提前做 GPU 化，调试成本会显著上升。

## 立即执行建议
1. 先做 WP1，把 `CT_LIDAR_CPU` 从 local frontend 收成 fixed-lag 主链。
2. 紧接着并行推进 WP2 和 WP3，先把 LiDAR/IMU 两条主约束做硬。
3. 然后完成 WP4，把 GNSS ownership 和 factor 质量补齐。
4. 最后做 WP5-WP8，完成系统联调、鲁棒性、benchmark 和封板文档。

## 计划使用方式
- 每完成一个工作包，就同步更新：
  - `docs/dev_ct/dev_status.md`
  - `docs/CHANGES.md`
  - `docs/TRACEABILITY.md`
- 每个工作包至少形成一次独立提交。
- 若某工作包引入新的架构分歧，先更新本文件，再继续实现。
