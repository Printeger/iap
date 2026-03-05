# Traceability Matrix (IAP)

> 目的：确保“需求（IAP-RQ）↔ 实现 ↔ 测试/实验 ↔ 日志/指标”可追溯，防止做错/做多/漏做。

## 0. 规则
- 每个代码改动必须引用至少一个 IAP-RQ
- 每个 IAP-RQ 必须在本表中至少有：
  - 实现文件路径（Implementation）
  - 验证方式（Test/Experiment）
  - 可观测日志字段（Logs/Metrics）

---

## 1. 需求追溯表

| Req ID | 需求描述 | Talk/Idea 对照点 | Implementation（文件/模块） | Test/Experiment（如何验证） | Logs/Metrics（必须输出） | 状态 |
|---|---|---|---|---|---|---|
| IAP-RQ-000 | Repo guardrails：AGENTS.md、doc-guard（pre-commit hook + tools/doc_guard.py）、docs 三件套 | — | `AGENTS.md`, `.githooks/pre-commit`, `tools/doc_guard.py`, `docs/` | 提交代码时 hook 拦截缺失文档；`git config core.hooksPath` = `.githooks` | hook exit code | **DONE** |
| IAP-RQ-001 | Rename ROS2 package to `iap` | — | `package.xml`, `CMakeLists.txt`, `src/iap/`, `include/iap/`, `cmake/iap-config.cmake.in` | `colcon build --packages-select iap` 成功；`ros2 pkg list \| grep iap` 可见 | build exit code 0 | **DONE** |
| IAP-RQ-002 | Build artifacts: compile_commands.json + 最小 demo | 开发基础设施 | `apps/iap_status.cpp`, `launch/iap_demo.launch.py`, `.clangd`, `CMakeLists.txt` (IAP_VERSION, install launch/) | `ros2 run iap iap_status config` 输出 OK; clangd 通过 .clangd 读取 build/iap/ | `iap_status: OK` | **DONE** |
| IAP-RQ-010 | 状态扩展：clk_bias δt [m], clk_drift δṫ [m/s] 加入 factor graph (`C(i)=gtsam::Vector2`) | 状态向量包含 clock bias/drift | `include/iap/odometry/estimation_frame.hpp`, `odometry_estimation_imu.hpp/.cpp` | `colcon build` 通过；`trace` 日志含 `clk_bias/clk_drift` | `clk_bias(m), clk_drift(m/s)` | **DONE** |
| IAP-RQ-015 | Expose Σ_p 位置协方差块（3×3）到 EstimationFrame；供 PL proxy 使用 | 保护级别需基于不确定性 | `estimation_frame.hpp` (+sigma_p), `odometry_estimation_imu.cpp` (marginalCovariance) | `trace` 日志含 `trace(Σ_p)` 和 `lambda_max(Σ_p)` | `trace_sigma_p, lambda_max_sigma_p, PL_proxy` | **DONE** |
| IAP-RQ-020 | GNSS 紧耦合观测：伪距 + 多普勒建因子（含 clock bias/drift，per-sat） | GNSS tightly-coupled | `include/iap/gnss/`, `src/iap/gnss/` (PseudorangeFactor, DopplerFactor, GnssHandler) | 关闭/开启 GNSS 对比 | `res_pr, res_dop, clk, clk_dot` | **DONE** |
| IAP-RQ-030 | GNSS per-satellite NIS gating：downweight / exclude（RAIM-ish baseline） | 卫星级完整性、FDE 思路 | `src/integrity/gnss_integrity.*` | 注入某卫星 bias，触发剔星 | `sat_nis, exclude_sats, gamma_R, global_nis` | TODO |
| IAP-RQ-040 | LiDAR ICP 因子健康度：退化/错配检测 → noise inflation / drop factor | trunk/几何退化会影响可观测性 | `EstimationFrame::IcpQuality`, `odometry_estimation_cpu.cpp` (Hessian cond, gamma_lidar) | 走廊/单面结构场景退化 | `icp_rmse, inliers, cond, gamma_lidar, drop` | **DONE** |
| IAP-RQ-100 | Trunk 检测与圆拟合（中心, 半径, confidence） | 树干几何地标 | `include/iap/trunk/trunk_types.hpp`, `trunk_detector.hpp/.cpp` (Kasa fit, grid BFS) | 对进树林场景启动，日志输出 trunk 数量/半径/置信度 | `trunk_count, radii, confidence[]` | **DONE** |
| IAP-RQ-110 | Trunk 健康度因子接口 (Baseline-A: 不入图) | 树干布局影响 LiDAR 可观测性 | `TrunkDetector::health_factor()` ([0,1]) | health~0 时硬件应填充更大噪声 | `trunk_health` | **DONE (Baseline-A)** |
| IAP-RQ-120 | TDOP 指标（角度多样性） | 树干几何与完整性联系 | `TrunkDetectionResult::tdop/tdop2/lambda_min_H` | 树更分散时 TDOP 下降 | `tdop, lambda_min_H` | **DONE** |
| IAP-RQ-200 | Integrity 输出：PL/AL/IM/mode + 关键中间量 | PL < AL 安全条件 | `include/iap/integrity/integrity_types.hpp`, `integrity_monitor.hpp/.cpp` | PL/AL/IM 曲线可画；mode 切换可复现 | `PL, AL, IM, mode` | **DONE** |
| IAP-RQ-210 | Alert Limit AL 由障碍距离动态给出 | 近障碍时 AL 缩小 | `IntegrityMonitor::compute_AL()`, `set_obstacle_distance()` | 越靠近障碍 AL 越小；日志可见 | `AL, obstacle_dist` | **DONE** |
| IAP-RQ-220 | GNSS per-satellite NIS gating（RAIM-ish） | 卫星级 FDE | `IntegrityMonitor::run_gnss_gating()` (chi2 test, gamma_R, FDE greedy) | 注入 bias 卫星被降权/剔除 | `sat_nis, gamma_R, excluded_sats` | **DONE** |
| IAP-RQ-300 | 候选轨迹生成（motion primitives） | 运动原语离散化候选轨迹 | `include/iap/planner/trajectory_types.hpp`, `trajectory_generator.hpp/.cpp` | 生成 M 条候选轨迹并可视化时间戳点序列 | `trajectory_count, speeds, yaw_rates` | **DONE** |
| IAP-RQ-310 | 预测可见/可观测性集合（占位） | ray-check 遮挡预测 | `include/iap/planner/predicted_integrity.hpp/.cpp` (placeholder) | TODO: 接地图后做 ray-check；当前返回占位值 | `n_vis_placeholder` | **DONE (placeholder)** |
| IAP-RQ-320 | 协方差传播 → Σ_pred → PL_pred | PL 预测供规划使用 | `include/iap/planner/predicted_integrity.hpp/.cpp` (sigma growth, K_pl=3.0) | PL_pred 随时间增长且不同轨迹有差异；σ_grow 可配置 | `PL_pred(s), sigma_pred(s)` | **DONE (baseline)** |
| IAP-RQ-050 | IMU 健康度：饱和/模型失配 → noise inflation / alarm | 传感器健康度影响可信度 | `src/health/imu_health.*` | 人为制造饱和或异常噪声 | `acc_norm, gyro_norm, sat_flag, gamma_imu` | TODO |
| IAP-RQ-060 | 规划目标：J(τ)=Σ hinge(PL_pred-AL)^2 + λ_goal*dist + λ_u*effort | 优化版 integrity-aware cost | `src/planner/cost.*` | 单步生成候选轨迹打分 | `J_total, J_integrity, J_goal, J_effort` | TODO |
| IAP-RQ-070 | 预测层：对候选轨迹预测 PL_pred（baseline：代理；升级：ARAIM） | 预测可见/可观测集合与 PL_pred | `src/predictor/*` | 对比不同轨迹的 PL_pred 差异 | `PL_pred(s), AL(s), IM_pred(s)` | TODO |
| IAP-RQ-080 | Receding horizon 闭环：执行第一段，重估计重规划 | active perception loop | `src/planner/mpc_loop.*` / `apps/*` | 跑闭环仿真/回放 | `chosen_traj_id, replanning_rate` | TODO |
| IAP-RQ-090 | 实验与消融：直飞 vs 协方差最小 vs 完整性驱动 | 证明减少违约 PL>AL | `apps/experiments/*` + `docs/*` | 批量跑场景并出表 | `violation_time, min_IM, path_len, time` | TODO |

---

## 2. 未映射改动（临时区）
> 如果你临时改了代码但还没决定它对应哪个需求，先把改动写在这里（提交前必须移入上表）。

- (none)