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
| IAP-RQ-010 | 从估计器导出位置协方差（或等价信息）用于 PL 计算 | 保护级别需基于不确定性 | `src/estimator/*` | 轨迹跑通并能取 Σp | `Sigma_p (or HPL/VPL)` | TODO |
| IAP-RQ-020 | GNSS 紧耦合观测：伪距 + 多普勒建因子（含 clock bias/drift） | GNSS tightly-coupled | `src/gnss/factors/*` | 关闭/开启 GNSS 对比 | `res_pr, res_dop, clk, clk_dot` | TODO |
| IAP-RQ-030 | GNSS per-satellite NIS gating：downweight / exclude（RAIM-ish baseline） | 卫星级完整性、FDE 思路 | `src/integrity/gnss_integrity.*` | 注入某卫星 bias，触发剔星 | `sat_nis, exclude_sats, gamma_R, global_nis` | TODO |
| IAP-RQ-040 | LiDAR ICP 因子健康度：退化/错配检测 → noise inflation / drop factor | trunk/几何退化会影响可观测性 | `src/health/lidar_health.*` | 走廊/单面结构场景退化 | `icp_rmse, inliers, cond, gamma_lidar, drop` | TODO |
| IAP-RQ-050 | IMU 健康度：饱和/模型失配 → noise inflation / alarm | 传感器健康度影响可信度 | `src/health/imu_health.*` | 人为制造饱和或异常噪声 | `acc_norm, gyro_norm, sat_flag, gamma_imu` | TODO |
| IAP-RQ-060 | 规划目标：J(τ)=Σ hinge(PL_pred-AL)^2 + λ_goal*dist + λ_u*effort | 优化版 integrity-aware cost | `src/planner/cost.*` | 单步生成候选轨迹打分 | `J_total, J_integrity, J_goal, J_effort` | TODO |
| IAP-RQ-070 | 预测层：对候选轨迹预测 PL_pred（baseline：代理；升级：ARAIM） | 预测可见/可观测集合与 PL_pred | `src/predictor/*` | 对比不同轨迹的 PL_pred 差异 | `PL_pred(s), AL(s), IM_pred(s)` | TODO |
| IAP-RQ-080 | Receding horizon 闭环：执行第一段，重估计重规划 | active perception loop | `src/planner/mpc_loop.*` / `apps/*` | 跑闭环仿真/回放 | `chosen_traj_id, replanning_rate` | TODO |
| IAP-RQ-090 | 实验与消融：直飞 vs 协方差最小 vs 完整性驱动 | 证明减少违约 PL>AL | `apps/experiments/*` + `docs/*` | 批量跑场景并出表 | `violation_time, min_IM, path_len, time` | TODO |

---

## 2. 未映射改动（临时区）
> 如果你临时改了代码但还没决定它对应哪个需求，先把改动写在这里（提交前必须移入上表）。

- (none)