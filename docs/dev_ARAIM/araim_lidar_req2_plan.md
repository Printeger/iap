# LiDAR ARAIM Req2 实施计划

## Summary
- 保存路径：`src/iap/docs/dev_ARAIM/araim_lidar_req2_plan.md`
- 目标：在保持当前 GNSS ARAIM / FGO / `IntegrityMonitor` 主语义不变的前提下，把 LiDAR ARAIM 扩展为 CPU 与 GPU 两条里程计链路都能产出同构 `LidarAraimSnapshot`，并按 `H_source / H_target(j) / H_level(l)` 三类模式接入统一完整性输出。
- 范围：本轮完成“共享接口 + CPU/GPU 端到端 block 快照 + 实时接入 `IntegrityMonitor`”；LiDAR ARAIM 求解器保持单一实现，不区分后端。

## Key Changes
- 新增 `LidarAraim`、`LidarAraimBlock`、`LidarAraimSnapshot`、`LidarAraimResult`、`LidarHypothesis`。
- 在 CPU odometry 的 `IntegratedVGICPFactor` 构建路径按 `target frame × voxelmap level` 生成 block snapshot，并将 `Lambda_B(6x6)` / `eta_B(6x1)`、`rmse_proxy`、`inlier_fraction`、`cond_proxy`、`age_sec`、`backend=CPU` 等元数据写入 `EstimationFrame::custom_data["lidar_araim_snapshot"]`。
- 在 GPU odometry 的 `IntegratedVGICPFactorGPU` 构建路径同步生成同构 block snapshot，并在 `update_frames()` 现有 GPU `icp_quality` 计算后回填 `current_icp_quality`、`gamma_lidar` 与 `valid`。
- GPU block 提取固定使用两阶段线性化：先对 GPU 因子图执行 `NonlinearFactorSetGPU::linearize(pred_values)` 填充对应关系，再逐个 `linearize(pred_values)` 转成 `HessianFactor` 抽取当前 source pose 的 `Lambda_B / eta_B`。
- CPU/GPU 的 `voxel_resolution` 统一从实际 target voxelmap 的 `voxel_resolution()` 读取，避免元数据因参数推导产生偏差。
- `IntegrityExtensionModule` 改为缓存 `on_update_new_frame` 的 IAP frame，并在同帧 FGO snapshot 到位后触发完整性计算。
- `IntegrityMonitor` 新增 LiDAR branch，最终 `PL_E/N/U`、`HPL/VPL`、`PL` 取 fallback / GNSS ARAIM / LiDAR ARAIM 的 worst-case。
- `IntegrityReport` 扩展 LiDAR 诊断字段，ROS message schema 暂不变更。
- `docs/TRACEABILITY.md` 同步更新为 CPU/GPU 两条 odometry 共同产出 LiDAR ARAIM snapshot 的实现与测试映射。

## Public / Interface Changes
- 新增内部接口：`LidarAraim::run(const LidarAraimSnapshot&, const FGOPositionInfo&)`。
- `IntegrityMonitor::compute()` 增加可选输入：`const FGOPositionInfo*`、`const LidarAraimSnapshot*`。
- `IntegrityReport` 增加 LiDAR 诊断字段：`lidar_valid`、`lidar_n_hyp`、`lidar_n_det`、`lidar_PL_E/N/U`、`lidar_HPL/VPL`、`lidar_worst_mode`。

## Test Plan
- 合成 snapshot 验证 `H_source / H_target / H_level` 枚举数量与分组逻辑。
- 单 block 场景验证 `Lambda_f / eta_f / delta_f / PL` 与手工计算一致。
- 恶化 `rmse / inlier_fraction / cond / age` 时，BiasModel 导致保护级单调上升。
- `IntegrityMonitor` 在 LiDAR-only 场景下按 max 规则将 LiDAR ARAIM 合入最终 `PL/HPL/VPL`。
- 在 CUDA 可用时增加 GPU 条件测试，验证 `backend=GPU` 的 block 能进入 LiDAR ARAIM hypothesis / monitor 合并链路。

## Assumptions / Defaults
- LiDAR ARAIM 求解器不区分 CPU/GPU；GPU 只负责按同一 snapshot 结构提供 block 线性化结果。
- LiDAR ARAIM 仅基于当前帧 source pose 的局部线性系统，不做每假设 fixed-lag smoother 重跑。
- target pose 在 LiDAR ARAIM 求解中冻结为当前线性化点；target 侧不确定性由 `pose_cov_6x6` 和 bias overbound 间接吸收。
- 若 GPU 构建关闭，系统自然退回现有 CPU-only LiDAR ARAIM，不新增额外配置分支。
