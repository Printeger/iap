先做一轮仓库内搜索，列出所有当前会写文件的地方，再开始改。

请对 `Printeger/iap` 的 `dev/ct-iap` 分支做一次“日志系统统一化（自动运行期产物）”重构。

目标
- 所有自动运行期产物统一放到一个稳定的 package-root 日志目录下。
- 每次运行创建一个独立的时间戳 run 目录，禁止不同运行共用同一个结果文件。
- 运行内可以追加写文件；跨运行绝不覆盖旧结果。
- 明确区分“自动运行期产物”和“用户显式导出 / save(path) 持久化结果”。
- 把 runtime logs / profiling / warnings / export / metadata 分层管理。
- 将当前分散在不同 config 文件、不同模块里的日志相关参数统一整理到一个 `log` 配置块下。
- 保持旧行为尽量兼容，但新路径和新配置应成为默认主路径。

背景约束
- 当前 README 已经说明默认日志目录是 `src/iap/log/`，公共结果目录是 `src/iap/log/res/`，并且 timing CSV 也会写到这个结果目录。
- 当前仓库里已经存在一批自动写盘点：`spdlog` runtime file sink、`timing_csv`、ICP CSV、CT LiDAR baseline/profile CSV、GNSS factor debug CSV、ARAIM CSV、integrity trajectory CSV、config dump 等。
- launch 会把 config 目录复制到 `/tmp/iap_launch_config_*` 后再运行，因此新的相对日志路径不能相对 `config_path` 或进程 cwd 解析，否则会错误落到 `/tmp/...`。
- 当前仓库里还存在一批由调用方显式指定 `path` 的保存/导出入口，例如 global mapping / submap / viewer save / rosnode dump；这些属于“用户触发的持久化导出”，默认不应被强行改造成 run-scoped 自动日志。
- 本次重构不改变核心 odometry / frontend / backend 算法逻辑，只整理日志系统、文件落盘、配置组织和调用接口。
- 每个修改完成后代码必须单独编译通过。

零、Scope 边界

本次重构纳入统一日志系统的内容：
- `spdlog` runtime file sink，含 `logging.*` 当前配置
- `timing_csv`
- ICP / CT LiDAR / numeric-reference / linearization 等自动诊断 CSV
- GNSS factor debug CSV
- ARAIM CSV
- integrity trajectory CSV
- config snapshot / run_info / git_rev / build_info 等 metadata

本次重构默认不强制纳入统一 run 目录的内容：
- `GlobalMapping::save(path)` / `GlobalMappingPoseGraph::save(path)` / `SubMap::save(path)` 等显式 `save(path)` 持久化导出
- viewer / rosnode 中用户触发的地图保存、dump 目录输出
- launch 过程中生成的临时 config 副本
- `tools/*` 离线脚本写出的转换产物
- `apps/iap_experiment.cpp` 这类独立实验程序的 `/tmp/...` 输出

边界要求：
- 对于“自动运行期产物”，必须统一落到当前 run 目录下。
- 对于显式 `save(path)` / 导出 API，本期默认保持调用方传入路径语义不变。
- 如果后续要把显式导出也接入统一目录，只能作为“未提供 path 时默认落到当前 run/export/”的增量能力，不能破坏现有显式路径语义。

一、目标目录规范

新默认日志根目录：
- `<package_root>/log`

其中 `package_root` 定义为当前仓库中的 `src/iap/` 目录，即包含 `README.md`、`config/`、`launch/`、`src/` 的包根目录。

路径解析规则：
- 若 `log.root_dir` 是绝对路径，则直接使用。
- 若 `log.root_dir` 是相对路径，则统一相对 `package_root` 解析，默认 `log` 解析为 `<package_root>/log`。
- 禁止把 `log.root_dir` 解析为相对 `config_path`、launch 复制出来的 `/tmp/iap_launch_config_*`、或进程当前工作目录。

每次运行创建一个新的 run 目录：
- `log/YYYY-MM-DD_HH-MM-SS/`
- 如果配置了 `run_name`，则：
  `log/YYYY-MM-DD_HH-MM-SS_<run_name>/`

每个 run 目录内部固定为：
- `runtime/`
  - `glim_main.log`
  - `glim_<module>.log`
  - `warnings.log`，可选 split sink
- `profiling/`
  - `pipeline_timing.csv`
  - `lidar_factor_profile.csv`
  - `numeric_reference.csv`
  - `linearization_check.csv`
- `export/`
  - `ct_lidar_baseline.csv`
  - `icp_quality.csv`
  - `gnss_factor_debug.csv`
  - `araim.csv`
  - `integrity_trajectory.csv`
  - `local_frontend_trajectory.csv`
  - `backend_summary.csv`
  - `bucket_stats.csv`
- `metadata/`
  - `config_snapshot.json`
  - `run_info.json`
  - `git_rev.txt`
  - `build_info.txt`

另外：
- 创建 `log/latest` 指向最近一次运行目录的软链接，配置可关闭。
- 同一次运行内部，单个文件可以 append。
- 不同运行之间绝不能向同一个文件继续写。

二、配置结构重构

请新增统一的顶层日志配置块：

```json
"log": {
  "root_dir": "log",
  "run_dir_mode": "TIMESTAMP",
  "run_name": "",
  "create_latest_symlink": true,
  "keep_last_n_runs": 20,

  "runtime": {
    "enable_console": true,
    "enable_file": true,
    "level": "INFO",
    "split_warnings_file": false,
    "main_file_name": "glim_main.log",
    "module_file_pattern": "glim_{module}.log",
    "warnings_file_name": "warnings.log",
    "rotate_files": true,
    "max_file_size_kb": 8192,
    "max_files": 10
  },

  "profiling": {
    "enable": false,
    "pipeline": false,
    "lidar_factor": false,
    "numeric_reference": false,
    "linearization_check": false,

    "pipeline_file": "pipeline_timing.csv",
    "lidar_factor_file": "lidar_factor_profile.csv",
    "numeric_reference_file": "numeric_reference.csv",
    "linearization_check_file": "linearization_check.csv"
  },

  "warnings": {
    "lidar_degeneracy": {
      "enable": false,
      "min_match_ratio": 0.5,
      "min_inlier_ratio": 0.35,
      "min_unique_target_ratio": 0.25,
      "max_target_reuse_ratio": 0.5,
      "max_ambiguity_rejection_ratio": 0.25,
      "min_mean_score_gap": 0.05
    }
  },

  "export": {
    "baseline_csv": false,
    "icp_quality_csv": false,
    "gnss_factor_debug_csv": false,
    "araim_csv": false,
    "integrity_trajectory_csv": false,
    "local_frontend_trajectory": false,
    "backend_summary": false,
    "bucket_stats": false,

    "baseline_csv_file": "ct_lidar_baseline.csv",
    "icp_quality_csv_file": "icp_quality.csv",
    "gnss_factor_debug_csv_file": "gnss_factor_debug.csv",
    "araim_csv_file": "araim.csv",
    "integrity_trajectory_csv_file": "integrity_trajectory.csv",
    "local_frontend_trajectory_file": "local_frontend_trajectory.csv",
    "backend_summary_file": "backend_summary.csv",
    "bucket_stats_file": "bucket_stats.csv"
  },

  "shared_output": {
    "publish_shared_trajectory": true,
    "attach_trajectory_to_frames": true,
    "attach_imu_rate_trajectory": true
  },

  "metadata": {
    "write_config_snapshot": true,
    "write_git_revision": true,
    "write_build_info": true,
    "config_snapshot_file": "config_snapshot.json",
    "run_info_file": "run_info.json",
    "git_rev_file": "git_rev.txt",
    "build_info_file": "build_info.txt"
  }
}
```

要求
- 允许保留旧参数兼容层，但新主路径必须优先从 `log.*` 读取。
- 若旧参数与新参数同时存在，以新参数为准。
- 旧参数保留 deprecation 注释。

说明：
- `log.root_dir` 使用上面的 package-root 相对解析规则。
- `*_file` 字段表示当前 run 子目录内的目标文件名。
- 兼容层读取旧 `*_path` 时，应只保留 basename 或显式给出 deprecation 提示，最终仍写入当前 run 对应子目录。
- `save_imu_rate_trajectory` 当前仓库语义更接近“是否把 IMU 率轨迹附着到 frame / shared output”，不是一个现成的自动文件导出开关，因此迁移到 `log.shared_output.attach_imu_rate_trajectory`，而不是 `log.export.*`。

三、旧参数迁移映射

请至少兼容并迁移这些旧参数：

运行日志与公共 timing：
- `logging.log_dir` -> `log.root_dir`
- `logging.save_logs` -> `log.runtime.enable_file`
- `logging.rotate_logs` -> `log.runtime.rotate_files`
- `logging.max_file_size_kb` -> `log.runtime.max_file_size_kb`
- `logging.max_files` -> `log.runtime.max_files`
- `global.enable_timing_csv` -> `log.profiling.pipeline`
- `global.timing_csv_path` -> `log.profiling.pipeline_file`

CT BSpline profiling / export：
- `ct_profile_pipeline` -> `log.profiling.pipeline`
- `ct_lidar_profile_factor` -> `log.profiling.lidar_factor`
- `ct_lidar_profile_numeric_reference` -> `log.profiling.numeric_reference`
- `ct_lidar_validate_linearization` -> `log.profiling.linearization_check`
- `ct_lidar_export_baseline_csv` -> `log.export.baseline_csv`
- `ct_lidar_baseline_csv_path` -> `log.export.baseline_csv_file`

CT degeneracy warning thresholds：
- `ct_lidar_warn_degeneracy` -> `log.warnings.lidar_degeneracy.enable`
- `ct_lidar_warn_min_match_ratio` -> `log.warnings.lidar_degeneracy.min_match_ratio`
- `ct_lidar_warn_min_inlier_ratio` -> `log.warnings.lidar_degeneracy.min_inlier_ratio`
- `ct_lidar_warn_min_unique_target_ratio` -> `log.warnings.lidar_degeneracy.min_unique_target_ratio`
- `ct_lidar_warn_max_target_reuse_ratio` -> `log.warnings.lidar_degeneracy.max_target_reuse_ratio`
- `ct_lidar_warn_max_ambiguity_rejection_ratio` -> `log.warnings.lidar_degeneracy.max_ambiguity_rejection_ratio`
- `ct_lidar_warn_min_mean_score_gap` -> `log.warnings.lidar_degeneracy.min_mean_score_gap`

Shared output：
- `publish_shared_trajectory` -> `log.shared_output.publish_shared_trajectory`
- `attach_trajectory_to_frames` -> `log.shared_output.attach_trajectory_to_frames`
- `save_imu_rate_trajectory` -> `log.shared_output.attach_imu_rate_trajectory`

ICP / GNSS / Integrity CSV：
- `enable_icp_csv` -> `log.export.icp_quality_csv`
- `icp_csv_path` -> `log.export.icp_quality_csv_file`
- `gnss.enable_debug_csv` -> `log.export.gnss_factor_debug_csv`
- `gnss.debug_csv_path` -> `log.export.gnss_factor_debug_csv_file`
- `integrity.enable_araim_csv` -> `log.export.araim_csv`
- `integrity.araim_csv_path` -> `log.export.araim_csv_file`
- `integrity.enable_traj_csv` -> `log.export.integrity_trajectory_csv`
- `integrity.traj_csv_path` -> `log.export.integrity_trajectory_csv_file`

兼容层统一规则：
- 若新 `log.*` 配置存在，则新配置优先。
- 旧 `*_path` 不再决定最终目录层级；兼容层只保留其文件名语义，最终仍写入当前 run 的 `runtime/`、`profiling/`、`export/`、`metadata/` 中。
- 对旧参数命中应打印一次 deprecation 提示，说明其对应的新 `log.*` 键名。

四、实现要求

新增统一的日志路径管理器

建议新增：
- `include/iap/common/log_paths.hpp`
- `src/iap/common/log_paths.cpp`

其职责：
- 创建 run 目录
- 创建 `runtime / profiling / export / metadata` 子目录
- 基于 package-root 规则生成各类文件绝对路径
- 创建/更新 `latest` 软链接
- 执行 `keep_last_n_runs` 清理策略

新增日志配置解析器

建议新增：
- `include/iap/common/log_config.hpp`
- `src/iap/common/log_config.cpp`

其职责：
- 解析 `log.*` 新配置
- 做旧参数兼容映射
- 提供统一访问接口

统一所有自动运行期落盘路径

请把当前散落在代码中的显式路径，尤其是 `/tmp/...csv` 和 `src/iap/log/res/...` 这类默认路径，改成：
- 统一通过 `LogPaths` 生成
- 落到本次 run 目录下对应子目录

补充边界：
- 自动运行期文件必须统一通过 `LogPaths` 生成。
- 显式 `save(path)` / 用户导出入口本期默认不强制改造为 `LogPaths`，但文档中必须明确标出其不在统一日志系统首批范围内。

区分五类输出

请把现有输出逻辑重新归类到以下五类：
- runtime logs
- profiling
- warnings
- export
- metadata

要求：
- runtime logger 只能写 `runtime/*`
- profiling CSV 只能写 `profiling/*`
- 自动结果导出只能写 `export/*`
- 运行元信息只能写 `metadata/*`
- `shared_output` 不属于文件日志，不应混进 `profiling/export` 目录
- 显式 `save(path)` 持久化导出不属于本期“自动运行期日志系统”范围

运行时行为
- 每次启动时自动创建新的时间戳 run 目录
- 写 `metadata/run_info.json`
- 如果开启 metadata 配置，则写 `config_snapshot.json`、`git_rev.txt`、`build_info.txt`
- 如果 `create_latest_symlink=true`，则更新 `log/latest`

写文件策略
- 同一运行内允许 append
- 跨运行不共用文件
- 禁止默认覆盖旧运行文件
- 如果 `run_dir_mode=FIXED_NAME`，默认也不要直接覆盖；至少要在 `overwrite_existing=false` 时拒绝覆盖并报错或自动追加后缀
- 但默认模式必须是 `TIMESTAMP`

五、建议修改位置

请优先检查并修改这些文件：

配置与公共基础设施：
- `config/config.json`
- `config/config_logging.json`
- `config/config_odometry_bspline.json`
- `config/config_odometry_gpu.json`
- `config/config_gnss.json`
- 其他使用 timing / profiling / export 参数的 config 文件
- `src/iap/util/logging.cpp`
- `include/iap/util/timing_csv.hpp`
- `src/iap/util/config.cpp`

自动运行期写盘入口：
- `src/iap/odometry/odometry_estimation_bspline.cpp`
- `src/iap/odometry/odometry_estimation_cpu.cpp`
- `src/iap/odometry/odometry_estimation_gpu.cpp`
- `src/iap/odometry/ct_local_frontend.cpp`
- `src/iap/odometry/ct_compact_backend.cpp`
- `src/iap/gnss/gnss_extension.cpp`
- `src/iap/integrity/integrity_extension.cpp`
- `include/iap/integrity/araim_debug.hpp`
- 任何写 `iap_timing.csv / iap_icp.csv / iap_araim.csv / iap_gnss_factor_debug.csv` 的代码路径

文档：
- `README.md`
- `docs/dev_ct/dev_status.md`，如有需要

本期默认不作为“统一 run 日志目录”首批强制改造对象，但需要在文档中明确边界：
- `src/iap/mapping/sub_map.cpp`
- `src/iap/mapping/global_mapping.cpp`
- `src/iap/mapping/global_mapping_pose_graph.cpp`
- `src/iap/viewer/map_editor.cpp`
- `src/iap/viewer/offline_viewer.cpp`
- `apps/iap_rosnode.cpp` 中用户指定 `dump_path_` 的保存链路
- `launch/iap_rosnode.launch.py` 的临时 config 复制逻辑
- `tools/*` 离线脚本与 `apps/iap_experiment.cpp`

六、README / 文档更新要求

请更新 README 中的 “Logs and Analysis” 部分：
- 说明默认根目录改为相对 package root 的 `log`
- 说明每次运行有独立时间戳目录
- 说明 `latest` 软链接
- 说明 `runtime / profiling / export / metadata` 目录含义
- 说明 timing CSV、GNSS factor debug、ARAIM、ICP CSV 等文件现在各自放到哪里
- 更新示例脚本路径，使其从新的目录结构读取
- 明确哪些写盘点属于“自动运行期日志系统”，哪些仍属于显式 `save(path)` / 用户导出

七、验收标准

- 每次运行都会创建一个新的时间戳 run 目录。
- 所有自动运行期日志 / CSV 都只写到这个 run 目录下。
- 不再默认写到 `/tmp` 或其他分散路径。
- `runtime / profiling / export / metadata` 目录结构固定可预期。
- 旧配置项仍可兼容，但新 `log.*` 配置优先。
- `latest` 软链接正确指向最近一次运行。
- README 中的日志说明与实际实现一致。
- 代码单独编译通过。
- 不改变 odometry / frontend / backend 主算法行为。
- 显式 `save(path)` / 用户导出链路保留调用方路径语义，不因本次重构被强行改成 run-scoped 自动日志。

八、输出要求

完成后请给出：
- 新增/修改文件清单
- 旧参数到新参数的完整映射表
- 每类自动运行期输出当前落盘的最终文件路径示例
- README 更新摘要
- 你如何保证“同次运行 append、跨运行不覆盖”的说明
- 哪些写盘链路被明确保留为显式导出、不纳入本期统一 run 日志目录
