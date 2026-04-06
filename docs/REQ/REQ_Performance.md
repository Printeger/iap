请为 Printeger/iap 的 dev/ct-iap 分支实现“下一阶段性能诊断 telemetry”，目标是定位 BSpline unified/incremental smoother 路径为什么仍然很慢。

本次只做：
- 新增/补齐性能诊断 telemetry
- 对齐当前 log system 的目录、命名、config 风格
- 更新 `tools/ana_log.py` 以读取和分析这些新 telemetry

本次不做：
- 不修改 runtime 壳
- 不修改 CPU/GPU legacy odometry
- 不修改 sub_mapping/global_mapping
- 不修改 viewer/topic/TF
- 不修改外部模块加载逻辑
- 不直接优化算法逻辑（先补可诊断性）

--------------------------------------------------
一、背景与问题陈述
--------------------------------------------------

当前运行报告已经确认：

1. 前端每帧平均耗时约 735.7 ms，其中 `lm_solve_ms` 平均 729.2 ms，占 99.1%。
2. `target_map_prep_ms` 只有约 5 ms，bucket/build/publish 都几乎可以忽略。
3. 当前是：
   - `frontend_mode=CT_LIDAR_CPU`
   - `frontend_only_mode=false`
   - `bucket_mode=SINGLE_BUCKET`
   - `enable_local_mapping=false`
   - `enable_global_mapping=false`
4. 当前 graph/problem size telemetry 已经有：
   - `active_control_point_count`
   - `active_pose_key_count`
   - `imu_factor_count`
   - `lidar_factor_count`
   - `gnss_factor_count`
   - `local_residual_count`
5. 但关键诊断盲区仍然存在：
   - `frontend_lm_iteration.csv` 没有有效数据，报告判断为 `instrumentation_bug_suspected`
   - LiDAR bucket profiling 没开，`points_in_bucket` 为 n/a
   - 看不到 incremental smoother/update 内部到底慢在：
     - relinearization
     - linearization
     - elimination / linear solve
     - estimate extraction
     - fallback / reseed
   - 也看不到单个 LiDAR factor 内部到底有多少点、多少 correspondence、多少 Jacobian 评估

因此，本次要新增 telemetry，回答以下问题：
- incremental path 是否仍然在大规模重线性化旧因子？
- `new_factor_count / new_value_count` 是否真的只与 current segment 相关？
- 单个 LiDAR factor 是否内部太重？
- GNSS/navigation layer 是否在少数帧有“补账式” spikes？
- solver update 时间到底分布在哪些内部阶段？

--------------------------------------------------
二、总体要求
--------------------------------------------------

请新增两类 telemetry，并让它们与当前 log system 对齐：

A. Solver/Internal Update Telemetry
B. LiDAR Factor Internal Load Telemetry

并满足：
- 所有新输出都进入当前统一 log 目录结构
- 命名、配置风格与现有 `log.profiling.*` / `log.export.*` 结构一致
- 高频诊断输出必须受 config flag 控制，默认关闭
- `ana_log.py` 必须同步更新，能够读取新文件并生成新章节/新字段分析
- 缺失文件时要有健壮性，不崩溃

--------------------------------------------------
三、与当前 log system 对齐的输出设计
--------------------------------------------------

请沿用当前 log 根目录和分层目录习惯，新增文件放到：

- `profiling/solver_update_profile.csv`
- `profiling/lidar_factor_internal_profile.csv`

如果你认为还有必要新增：
- `profiling/relinearization_profile.csv`
也可以，但优先先把字段并入 `solver_update_profile.csv`，避免文件过碎。

命名要求：
- 使用蛇形命名
- 文件名要和当前 `frontend_frame_profile.csv`、`pipeline_timing.csv` 风格一致
- 不要写到 `/tmp`
- 不要绕开现有 log root/run dir 逻辑

--------------------------------------------------
四、配置要求
--------------------------------------------------

请在当前 log/profiling 配置体系中新增以下开关，默认全为 false：

```json
"log": {
  "profiling": {
    "solver_update_profile": false,
    "lidar_factor_internal_profile": false
  }
}

如果当前项目还没完全统一到 log.profiling.*，请兼容旧参数映射，但新主路径必须优先从 log.profiling.* 读取。

建议新增的最终配置键：

log.profiling.solver_update_profile
log.profiling.lidar_factor_internal_profile

可选保留旧兼容键（若你认为必要）：

bspline_solver_update_profile
ct_lidar_internal_profile

但若保留兼容键：

新键优先
旧键标记为 deprecated
run_info.json / config_snapshot.json 中保留最终生效值
五、A 类：Solver/Internal Update Telemetry

请新增 profiling/solver_update_profile.csv。

每帧至少输出以下字段：

基础识别字段：

frame_id
frame_stamp
solver_mode
frontend_only_mode
local_layer_enabled
navigation_layer_enabled
used_incremental_solver
fallback_used

本次 delta 规模：

new_factor_count
new_value_count
new_stamp_count
query_key_count
retired_key_count

当前活跃规模：

active_control_point_count
active_pose_key_count
active_aux_key_count
persistent_key_count
local_state_dimension
local_residual_count

solver update 内部耗时：

solver_update_ms
relinearization_ms
linearization_ms
elimination_ms
delta_solve_ms
estimate_query_ms
fallback_rebuild_ms

solver update 内部规模：

relinearized_variable_count
relinearized_factor_count
linearized_factor_count
bayes_tree_clique_count（若可用）
affected_variable_count（若可用）

优化/求解行为：

optimize_count
initial_error
final_error
error_drop_ratio
iteration_count（如果增量路径有等价概念就记录，没有则写 0 并解释）
solver_status
约束
若底层 smoother 无法提供某些字段，请填空值或 0，但不要伪造。
若当前还是 batch solver，也应能输出同一 schema，方便 A/B。
solver_update_ms 应尽量与当前 frontend_frame_profile.csv 中主求解耗时对齐或可映射。
目的

用这份文件判断：

增量路径是否仍在大规模重线性化旧因子
new_factor_count / new_value_count 是否真的是 delta-only
慢帧是否由 relinearization_ms / elimination_ms / estimate_query_ms 主导
fallback 是否被频繁触发
六、B 类：LiDAR Factor Internal Load Telemetry

请新增 profiling/lidar_factor_internal_profile.csv。

每帧/每因子至少输出以下字段：

基础字段：

frame_id
frame_stamp
bucket_mode
bucket_count
factor_index（若一帧只有一个 factor 也保留字段）
representative_time

输入规模：

points_in_bucket
source_point_count
target_candidate_count（若可用）
valid_correspondence_count
effective_residual_count

耗时字段：

factor_total_ms
correspondence_ms
covariance_lookup_ms
residual_eval_ms
jacobian_eval_ms

质量字段：

match_ratio
inlier_ratio
best_distance_mean（若可用）
best_second_gap_mean（若可用）

状态关联字段：

support_control_count
support_pose_key_count
active_control_point_count
约束
SINGLE_BUCKET 模式下也必须产出有效行，不能再是 points_in_bucket = n/a
如果某些字段当前实现拿不到，可以先保留列并写空值，但要尽量补齐 points_in_bucket、valid_correspondence_count、factor_total_ms、jacobian_eval_ms
与当前 bucket_stats.csv、lidar_factor_profile.csv 若有重叠，优先复用已有统计逻辑，避免重复代码
目的

用这份文件判断：

单 bucket 下单个 LiDAR factor 是否内部过重
correspondence / covariance / Jacobian 哪部分最贵
local_residual_count 的来源是否主要就是 LiDAR factor 内部点数过大
七、run_info / metadata 对齐要求

请在 run_info.json 或等价 metadata 中记录这些新 profiling 开关的 config 值和 runtime 生效值，至少包括：

config_log_profiling_solver_update_profile
runtime_log_profiling_solver_update_profile
config_log_profiling_lidar_factor_internal_profile
runtime_log_profiling_lidar_factor_internal_profile

如果当前 run_info 已经采用 config_* / runtime_* 风格，请保持一致。

八、ana_log.py 更新要求

请同步更新 tools/ana_log.py，读取并分析新 telemetry。

1. Artifact Coverage

新增：

solver_update_profile
lidar_factor_internal_profile

并给出：

found / missing / disabled / empty
reason
rows
size
path
2. 新增章节：Solver Update Analysis

请输出：

基础摘要：

solver_update_rows
used_incremental_solver_ratio
fallback_used_count
solver_update_mean_ms
solver_update_p95_ms
solver_update_max_ms

delta 规模摘要：

new_factor_count_mean / p95 / max
new_value_count_mean / p95 / max
retired_key_count_mean / p95 / max

重线性化摘要：

relinearized_variable_count_mean / p95 / max
relinearized_factor_count_mean / p95 / max
relinearization_ms_mean / p95 / max
elimination_ms_mean / p95 / max
estimate_query_ms_mean / p95 / max

请在 Findings 中明确判断：

是否存在“delta 小，但 relinearized_factor_count 仍很大”的迹象
是否存在“solver_update_ms 主要被 relinearization / elimination 主导”的迹象
3. 新增章节：LiDAR Factor Internal Load Analysis

请输出：

points_in_bucket_mean / p95 / max
valid_correspondence_count_mean / p95 / max
effective_residual_count_mean / p95 / max
factor_total_ms_mean / p95 / max
correspondence_ms_mean / p95 / max
jacobian_eval_ms_mean / p95 / max

请在 Findings 中明确判断：

单个 LiDAR factor 是否可能成为主瓶颈
correspondence / Jacobian 哪个更重
4. 扩展 Correlation Analysis

新增或优先尝试这些相关性：

solver_update_ms vs new_factor_count
solver_update_ms vs relinearized_factor_count
solver_update_ms vs relinearized_variable_count
solver_update_ms vs active_control_point_count
solver_update_ms vs local_residual_count
factor_total_ms vs points_in_bucket
jacobian_eval_ms vs points_in_bucket
5. Findings / Recommendations 要升级

如果新 telemetry 可用，请在 Findings 中优先给出如下结论模板：

“incremental path appears genuinely delta-scaled” 或
“incremental path appears pseudo-incremental; heavy relinearization remains”

以及：

“single LiDAR factor internal load appears dominant” 或
“LiDAR factor internal load appears secondary to solver update cost”

Recommendations 里要能输出更明确的下一步：

优先砍 relinearization
优先压 LiDAR factor 内部点数/对应数
优先限制 GNSS spike 注入
优先检查 fallback/reseed 频率
九、实现范围建议

请优先检查并修改：

src/iap/odometry/odometry_estimation_bspline.cpp
新增或修改 solver abstraction 实现文件
ct_local_frontend.*
ct_compact_backend.*
include/iap/common/log_paths.hpp
src/iap/common/log_paths.cpp
tools/ana_log.py
config/config_odometry_bspline.json
相关 profiling/test 文件
十、验收标准
打开 log.profiling.solver_update_profile=true 后，会生成：
profiling/solver_update_profile.csv
打开 log.profiling.lidar_factor_internal_profile=true 后，会生成：
profiling/lidar_factor_internal_profile.csv
新 telemetry 与当前 log system 的目录、命名、metadata 风格一致。
ana_log.py 能正确识别这两个新文件，并生成：
Solver Update Analysis
LiDAR Factor Internal Load Analysis
新的 correlation 结果
更明确的 findings/recommendations
缺失文件或配置关闭时，ana_log.py 不崩溃，并能解释原因。
不改变 full-system 外层行为。

输出要求：

给出修改文件清单
给出新增 CSV 的最终列名
给出 ana_log.py 新增/修改的章节摘要
给出一段示例 findings，说明它如何区分：
pseudo-incremental heavy relinearization
heavy single-factor LiDAR load