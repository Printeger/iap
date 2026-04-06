#include <iap/common/log_config.hpp>

#include <filesystem>
#include <mutex>
#include <set>

#include <spdlog/spdlog.h>

#include <iap/util/config.hpp>

namespace iap {

namespace {

std::mutex& config_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::optional<LogConfig>& cached_config() {
  static std::optional<LogConfig> config;
  return config;
}

std::optional<glim::Config> load_named_config_if_exists(const std::string& config_name) {
  const auto path = glim::GlobalConfig::get_config_path(config_name);
  if (path.empty() || !std::filesystem::exists(path)) {
    return std::nullopt;
  }
  return glim::Config(path);
}

template <typename T>
std::optional<T> nested_param(const std::vector<std::string>& nested_module_names, const std::string& param_name) {
  auto* global = glim::GlobalConfig::instance();
  if (!global) {
    return std::nullopt;
  }
  return global->param_nested<T>(nested_module_names, param_name);
}

std::optional<bool> bool_param(const std::optional<glim::Config>& config,
                               const std::string& module_name,
                               const std::string& param_name) {
  if (!config) {
    return std::nullopt;
  }
  return config->param<bool>(module_name, param_name);
}

std::optional<int> int_param(const std::optional<glim::Config>& config,
                             const std::string& module_name,
                             const std::string& param_name) {
  if (!config) {
    return std::nullopt;
  }
  return config->param<int>(module_name, param_name);
}

std::optional<double> double_param(const std::optional<glim::Config>& config,
                                   const std::string& module_name,
                                   const std::string& param_name) {
  if (!config) {
    return std::nullopt;
  }
  return config->param<double>(module_name, param_name);
}

std::optional<std::string> string_param(const std::optional<glim::Config>& config,
                                        const std::string& module_name,
                                        const std::string& param_name) {
  if (!config) {
    return std::nullopt;
  }
  return config->param<std::string>(module_name, param_name);
}

std::string resolve_filename(const std::optional<std::string>& new_value,
                             const std::optional<std::string>& legacy_path,
                             const std::string& legacy_key,
                             const std::string& new_key,
                             const std::string& default_filename) {
  if (new_value && !new_value->empty()) {
    return *new_value;
  }
  if (legacy_path && !legacy_path->empty()) {
    warn_log_compat_once(legacy_key, new_key);
    return legacy_path_to_filename(legacy_path, default_filename);
  }
  return default_filename;
}

LogConfig load_log_config_impl() {
  LogConfig config;

  const auto odom_config = load_named_config_if_exists("config_odometry");
  const auto gnss_config = load_named_config_if_exists("config_gnss");

  config.root_dir = resolve_log_value<std::string>(
    nested_param<std::string>({"log"}, "root_dir"),
    glim::GlobalConfig::instance()->param<std::string>("logging", "log_dir"),
    "logging.log_dir",
    "log.root_dir",
    config.root_dir);
  config.run_dir_mode = resolve_log_value<std::string>(
    nested_param<std::string>({"log"}, "run_dir_mode"),
    std::nullopt,
    "",
    "log.run_dir_mode",
    config.run_dir_mode);
  config.run_name = resolve_log_value<std::string>(
    nested_param<std::string>({"log"}, "run_name"),
    std::nullopt,
    "",
    "log.run_name",
    config.run_name);
  config.create_latest_symlink = resolve_log_value<bool>(
    nested_param<bool>({"log"}, "create_latest_symlink"),
    std::nullopt,
    "",
    "log.create_latest_symlink",
    config.create_latest_symlink);
  config.keep_last_n_runs = resolve_log_value<int>(
    nested_param<int>({"log"}, "keep_last_n_runs"),
    std::nullopt,
    "",
    "log.keep_last_n_runs",
    config.keep_last_n_runs);

  config.runtime.enable_console = resolve_log_value<bool>(
    nested_param<bool>({"log", "runtime"}, "enable_console"),
    std::nullopt,
    "",
    "log.runtime.enable_console",
    config.runtime.enable_console);
  config.runtime.enable_file = resolve_log_value<bool>(
    nested_param<bool>({"log", "runtime"}, "enable_file"),
    glim::GlobalConfig::instance()->param<bool>("logging", "save_logs"),
    "logging.save_logs",
    "log.runtime.enable_file",
    config.runtime.enable_file);
  config.runtime.level = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "runtime"}, "level"),
    std::nullopt,
    "",
    "log.runtime.level",
    config.runtime.level);
  config.runtime.split_warnings_file = resolve_log_value<bool>(
    nested_param<bool>({"log", "runtime"}, "split_warnings_file"),
    std::nullopt,
    "",
    "log.runtime.split_warnings_file",
    config.runtime.split_warnings_file);
  config.runtime.main_file_name = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "runtime"}, "main_file_name"),
    std::nullopt,
    "",
    "log.runtime.main_file_name",
    config.runtime.main_file_name);
  config.runtime.module_file_pattern = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "runtime"}, "module_file_pattern"),
    std::nullopt,
    "",
    "log.runtime.module_file_pattern",
    config.runtime.module_file_pattern);
  config.runtime.warnings_file_name = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "runtime"}, "warnings_file_name"),
    std::nullopt,
    "",
    "log.runtime.warnings_file_name",
    config.runtime.warnings_file_name);
  config.runtime.rotate_files = resolve_log_value<bool>(
    nested_param<bool>({"log", "runtime"}, "rotate_files"),
    glim::GlobalConfig::instance()->param<bool>("logging", "rotate_logs"),
    "logging.rotate_logs",
    "log.runtime.rotate_files",
    config.runtime.rotate_files);
  config.runtime.max_file_size_kb = resolve_log_value<int>(
    nested_param<int>({"log", "runtime"}, "max_file_size_kb"),
    glim::GlobalConfig::instance()->param<int>("logging", "max_file_size_kb"),
    "logging.max_file_size_kb",
    "log.runtime.max_file_size_kb",
    config.runtime.max_file_size_kb);
  config.runtime.max_files = resolve_log_value<int>(
    nested_param<int>({"log", "runtime"}, "max_files"),
    glim::GlobalConfig::instance()->param<int>("logging", "max_files"),
    "logging.max_files",
    "log.runtime.max_files",
    config.runtime.max_files);

  config.profiling.pipeline = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "pipeline"),
    glim::GlobalConfig::instance()->param<bool>("global", "enable_timing_csv"),
    "global.enable_timing_csv",
    "log.profiling.pipeline",
    config.profiling.pipeline);
  config.profiling.pipeline_file = resolve_filename(
    nested_param<std::string>({"log", "profiling"}, "pipeline_file"),
    glim::GlobalConfig::instance()->param<std::string>("global", "timing_csv_path"),
    "global.timing_csv_path",
    "log.profiling.pipeline_file",
    config.profiling.pipeline_file);
  config.profiling.frontend_frame = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "frontend_frame"),
    std::nullopt,
    "",
    "log.profiling.frontend_frame",
    config.profiling.frontend_frame);
  config.profiling.frontend_frame_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "frontend_frame_file"),
    std::nullopt,
    "",
    "log.profiling.frontend_frame_file",
    config.profiling.frontend_frame_file);
  config.profiling.lidar_factor = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "lidar_factor"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_profile_factor"),
    "odometry_estimation.ct_lidar_profile_factor",
    "log.profiling.lidar_factor",
    config.profiling.lidar_factor);
  config.profiling.solver_update_profile = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "solver_update_profile"),
    bool_param(odom_config, "odometry_estimation", "bspline_solver_update_profile"),
    "odometry_estimation.bspline_solver_update_profile",
    "log.profiling.solver_update_profile",
    config.profiling.solver_update_profile);
  config.profiling.lidar_factor_internal_profile = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "lidar_factor_internal_profile"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_internal_profile"),
    "odometry_estimation.ct_lidar_internal_profile",
    "log.profiling.lidar_factor_internal_profile",
    config.profiling.lidar_factor_internal_profile);
  config.profiling.frontend_lm_iteration = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "frontend_lm_iteration"),
    std::nullopt,
    "",
    "log.profiling.frontend_lm_iteration",
    config.profiling.frontend_lm_iteration);
  config.profiling.frame_warning_profile = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "frame_warning_profile"),
    std::nullopt,
    "",
    "log.profiling.frame_warning_profile",
    config.profiling.frame_warning_profile);
  config.profiling.jump_diagnostics = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "jump_diagnostics"),
    std::nullopt,
    "",
    "log.profiling.jump_diagnostics",
    config.profiling.jump_diagnostics);
  config.profiling.target_map_prep_breakdown = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "target_map_prep_breakdown"),
    std::nullopt,
    "",
    "log.profiling.target_map_prep_breakdown",
    config.profiling.target_map_prep_breakdown);
  config.profiling.graph_problem_size = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "graph_problem_size"),
    std::nullopt,
    "",
    "log.profiling.graph_problem_size",
    config.profiling.graph_problem_size);
  config.profiling.numeric_reference = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "numeric_reference"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_profile_numeric_reference"),
    "odometry_estimation.ct_lidar_profile_numeric_reference",
    "log.profiling.numeric_reference",
    config.profiling.numeric_reference);
  config.profiling.linearization_check = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "linearization_check"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_validate_linearization"),
    "odometry_estimation.ct_lidar_validate_linearization",
    "log.profiling.linearization_check",
    config.profiling.linearization_check);
  config.profiling.lidar_factor_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "lidar_factor_file"),
    std::nullopt,
    "",
    "log.profiling.lidar_factor_file",
    config.profiling.lidar_factor_file);
  config.profiling.solver_update_profile_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "solver_update_profile_file"),
    std::nullopt,
    "",
    "log.profiling.solver_update_profile_file",
    config.profiling.solver_update_profile_file);
  config.profiling.lidar_factor_internal_profile_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "lidar_factor_internal_profile_file"),
    std::nullopt,
    "",
    "log.profiling.lidar_factor_internal_profile_file",
    config.profiling.lidar_factor_internal_profile_file);
  config.profiling.frontend_lm_iteration_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "frontend_lm_iteration_file"),
    std::nullopt,
    "",
    "log.profiling.frontend_lm_iteration_file",
    config.profiling.frontend_lm_iteration_file);
  config.profiling.frame_warning_profile_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "frame_warning_profile_file"),
    std::nullopt,
    "",
    "log.profiling.frame_warning_profile_file",
    config.profiling.frame_warning_profile_file);
  config.profiling.jump_diagnostics_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "jump_diagnostics_file"),
    std::nullopt,
    "",
    "log.profiling.jump_diagnostics_file",
    config.profiling.jump_diagnostics_file);
  config.profiling.numeric_reference_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "numeric_reference_file"),
    std::nullopt,
    "",
    "log.profiling.numeric_reference_file",
    config.profiling.numeric_reference_file);
  config.profiling.linearization_check_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "profiling"}, "linearization_check_file"),
    std::nullopt,
    "",
    "log.profiling.linearization_check_file",
    config.profiling.linearization_check_file);

  config.warnings.lidar_degeneracy.enable = resolve_log_value<bool>(
    nested_param<bool>({"log", "warnings", "lidar_degeneracy"}, "enable"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_warn_degeneracy"),
    "odometry_estimation.ct_lidar_warn_degeneracy",
    "log.warnings.lidar_degeneracy.enable",
    config.warnings.lidar_degeneracy.enable);
  config.warnings.lidar_degeneracy.min_match_ratio = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "min_match_ratio"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_min_match_ratio"),
    "odometry_estimation.ct_lidar_warn_min_match_ratio",
    "log.warnings.lidar_degeneracy.min_match_ratio",
    config.warnings.lidar_degeneracy.min_match_ratio);
  config.warnings.lidar_degeneracy.min_inlier_ratio = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "min_inlier_ratio"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_min_inlier_ratio"),
    "odometry_estimation.ct_lidar_warn_min_inlier_ratio",
    "log.warnings.lidar_degeneracy.min_inlier_ratio",
    config.warnings.lidar_degeneracy.min_inlier_ratio);
  config.warnings.lidar_degeneracy.min_unique_target_ratio = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "min_unique_target_ratio"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_min_unique_target_ratio"),
    "odometry_estimation.ct_lidar_warn_min_unique_target_ratio",
    "log.warnings.lidar_degeneracy.min_unique_target_ratio",
    config.warnings.lidar_degeneracy.min_unique_target_ratio);
  config.warnings.lidar_degeneracy.max_target_reuse_ratio = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "max_target_reuse_ratio"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_max_target_reuse_ratio"),
    "odometry_estimation.ct_lidar_warn_max_target_reuse_ratio",
    "log.warnings.lidar_degeneracy.max_target_reuse_ratio",
    config.warnings.lidar_degeneracy.max_target_reuse_ratio);
  config.warnings.lidar_degeneracy.max_ambiguity_rejection_ratio = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "max_ambiguity_rejection_ratio"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_max_ambiguity_rejection_ratio"),
    "odometry_estimation.ct_lidar_warn_max_ambiguity_rejection_ratio",
    "log.warnings.lidar_degeneracy.max_ambiguity_rejection_ratio",
    config.warnings.lidar_degeneracy.max_ambiguity_rejection_ratio);
  config.warnings.lidar_degeneracy.min_mean_score_gap = resolve_log_value<double>(
    nested_param<double>({"log", "warnings", "lidar_degeneracy"}, "min_mean_score_gap"),
    double_param(odom_config, "odometry_estimation", "ct_lidar_warn_min_mean_score_gap"),
    "odometry_estimation.ct_lidar_warn_min_mean_score_gap",
    "log.warnings.lidar_degeneracy.min_mean_score_gap",
    config.warnings.lidar_degeneracy.min_mean_score_gap);

  config.export_outputs.baseline_csv = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "baseline_csv"),
    bool_param(odom_config, "odometry_estimation", "ct_lidar_export_baseline_csv"),
    "odometry_estimation.ct_lidar_export_baseline_csv",
    "log.export.baseline_csv",
    config.export_outputs.baseline_csv);
  config.export_outputs.baseline_csv_file = resolve_filename(
    nested_param<std::string>({"log", "export"}, "baseline_csv_file"),
    string_param(odom_config, "odometry_estimation", "ct_lidar_baseline_csv_path"),
    "odometry_estimation.ct_lidar_baseline_csv_path",
    "log.export.baseline_csv_file",
    config.export_outputs.baseline_csv_file);
  config.export_outputs.icp_quality_csv = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "icp_quality_csv"),
    bool_param(odom_config, "odometry_estimation", "enable_icp_csv"),
    "odometry_estimation.enable_icp_csv",
    "log.export.icp_quality_csv",
    config.export_outputs.icp_quality_csv);
  config.export_outputs.icp_quality_csv_file = resolve_filename(
    nested_param<std::string>({"log", "export"}, "icp_quality_csv_file"),
    string_param(odom_config, "odometry_estimation", "icp_csv_path"),
    "odometry_estimation.icp_csv_path",
    "log.export.icp_quality_csv_file",
    config.export_outputs.icp_quality_csv_file);
  config.export_outputs.gnss_factor_debug_csv = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "gnss_factor_debug_csv"),
    bool_param(gnss_config, "gnss", "enable_debug_csv"),
    "gnss.enable_debug_csv",
    "log.export.gnss_factor_debug_csv",
    config.export_outputs.gnss_factor_debug_csv);
  config.export_outputs.gnss_factor_debug_csv_file = resolve_filename(
    nested_param<std::string>({"log", "export"}, "gnss_factor_debug_csv_file"),
    string_param(gnss_config, "gnss", "debug_csv_path"),
    "gnss.debug_csv_path",
    "log.export.gnss_factor_debug_csv_file",
    config.export_outputs.gnss_factor_debug_csv_file);
  config.export_outputs.araim_csv = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "araim_csv"),
    bool_param(gnss_config, "integrity", "enable_araim_csv"),
    "integrity.enable_araim_csv",
    "log.export.araim_csv",
    config.export_outputs.araim_csv);
  config.export_outputs.araim_csv_file = resolve_filename(
    nested_param<std::string>({"log", "export"}, "araim_csv_file"),
    string_param(gnss_config, "integrity", "araim_csv_path"),
    "integrity.araim_csv_path",
    "log.export.araim_csv_file",
    config.export_outputs.araim_csv_file);
  config.export_outputs.integrity_trajectory_csv = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "integrity_trajectory_csv"),
    bool_param(gnss_config, "integrity", "enable_traj_csv"),
    "integrity.enable_traj_csv",
    "log.export.integrity_trajectory_csv",
    config.export_outputs.integrity_trajectory_csv);
  config.export_outputs.integrity_trajectory_csv_file = resolve_filename(
    nested_param<std::string>({"log", "export"}, "integrity_trajectory_csv_file"),
    string_param(gnss_config, "integrity", "traj_csv_path"),
    "integrity.traj_csv_path",
    "log.export.integrity_trajectory_csv_file",
    config.export_outputs.integrity_trajectory_csv_file);
  config.export_outputs.local_frontend_trajectory = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "local_frontend_trajectory"),
    std::nullopt,
    "",
    "log.export.local_frontend_trajectory",
    config.export_outputs.local_frontend_trajectory);
  config.export_outputs.backend_summary = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "backend_summary"),
    std::nullopt,
    "",
    "log.export.backend_summary",
    config.export_outputs.backend_summary);
  config.export_outputs.bucket_stats = resolve_log_value<bool>(
    nested_param<bool>({"log", "export"}, "bucket_stats"),
    std::nullopt,
    "",
    "log.export.bucket_stats",
    config.export_outputs.bucket_stats);
  config.export_outputs.local_frontend_trajectory_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "export"}, "local_frontend_trajectory_file"),
    std::nullopt,
    "",
    "log.export.local_frontend_trajectory_file",
    config.export_outputs.local_frontend_trajectory_file);
  config.export_outputs.backend_summary_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "export"}, "backend_summary_file"),
    std::nullopt,
    "",
    "log.export.backend_summary_file",
    config.export_outputs.backend_summary_file);
  config.export_outputs.bucket_stats_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "export"}, "bucket_stats_file"),
    std::nullopt,
    "",
    "log.export.bucket_stats_file",
    config.export_outputs.bucket_stats_file);

  config.shared_output.publish_shared_trajectory = resolve_log_value<bool>(
    nested_param<bool>({"log", "shared_output"}, "publish_shared_trajectory"),
    bool_param(odom_config, "odometry_estimation", "publish_shared_trajectory"),
    "odometry_estimation.publish_shared_trajectory",
    "log.shared_output.publish_shared_trajectory",
    config.shared_output.publish_shared_trajectory);
  config.shared_output.attach_trajectory_to_frames = resolve_log_value<bool>(
    nested_param<bool>({"log", "shared_output"}, "attach_trajectory_to_frames"),
    bool_param(odom_config, "odometry_estimation", "attach_trajectory_to_frames"),
    "odometry_estimation.attach_trajectory_to_frames",
    "log.shared_output.attach_trajectory_to_frames",
    config.shared_output.attach_trajectory_to_frames);
  config.shared_output.attach_imu_rate_trajectory = resolve_log_value<bool>(
    nested_param<bool>({"log", "shared_output"}, "attach_imu_rate_trajectory"),
    bool_param(odom_config, "odometry_estimation", "save_imu_rate_trajectory"),
    "odometry_estimation.save_imu_rate_trajectory",
    "log.shared_output.attach_imu_rate_trajectory",
    config.shared_output.attach_imu_rate_trajectory);

  config.metadata.write_config_snapshot = resolve_log_value<bool>(
    nested_param<bool>({"log", "metadata"}, "write_config_snapshot"),
    std::nullopt,
    "",
    "log.metadata.write_config_snapshot",
    config.metadata.write_config_snapshot);
  config.metadata.write_git_revision = resolve_log_value<bool>(
    nested_param<bool>({"log", "metadata"}, "write_git_revision"),
    std::nullopt,
    "",
    "log.metadata.write_git_revision",
    config.metadata.write_git_revision);
  config.metadata.write_build_info = resolve_log_value<bool>(
    nested_param<bool>({"log", "metadata"}, "write_build_info"),
    std::nullopt,
    "",
    "log.metadata.write_build_info",
    config.metadata.write_build_info);
  config.metadata.write_mode_manifest = resolve_log_value<bool>(
    nested_param<bool>({"log", "metadata"}, "write_mode_manifest"),
    std::nullopt,
    "",
    "log.metadata.write_mode_manifest",
    config.metadata.write_mode_manifest);
  config.metadata.config_snapshot_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "metadata"}, "config_snapshot_file"),
    std::nullopt,
    "",
    "log.metadata.config_snapshot_file",
    config.metadata.config_snapshot_file);
  config.metadata.run_info_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "metadata"}, "run_info_file"),
    std::nullopt,
    "",
    "log.metadata.run_info_file",
    config.metadata.run_info_file);
  config.metadata.git_rev_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "metadata"}, "git_rev_file"),
    std::nullopt,
    "",
    "log.metadata.git_rev_file",
    config.metadata.git_rev_file);
  config.metadata.build_info_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "metadata"}, "build_info_file"),
    std::nullopt,
    "",
    "log.metadata.build_info_file",
    config.metadata.build_info_file);
  config.metadata.mode_manifest_file = resolve_log_value<std::string>(
    nested_param<std::string>({"log", "metadata"}, "mode_manifest_file"),
    std::nullopt,
    "",
    "log.metadata.mode_manifest_file",
    config.metadata.mode_manifest_file);

  config.profiling.enable = resolve_log_value<bool>(
    nested_param<bool>({"log", "profiling"}, "enable"),
    std::nullopt,
    "",
    "log.profiling.enable",
    config.profiling.pipeline || config.profiling.frontend_frame ||
      config.profiling.lidar_factor || config.profiling.solver_update_profile ||
      config.profiling.lidar_factor_internal_profile ||
      config.profiling.frontend_lm_iteration ||
      config.profiling.frame_warning_profile || config.profiling.jump_diagnostics ||
      config.profiling.target_map_prep_breakdown ||
      config.profiling.graph_problem_size || config.profiling.numeric_reference ||
      config.profiling.linearization_check);

  return config;
}

}  // namespace

const LogConfig& get_log_config() {
  std::lock_guard<std::mutex> lock(config_mutex());
  auto& config = cached_config();
  if (!config) {
    config = load_log_config_impl();
  }
  return *config;
}

void reset_log_config() {
  std::lock_guard<std::mutex> lock(config_mutex());
  cached_config().reset();
}

std::optional<bool> log_bool(const std::vector<std::string>& nested_module_names, const std::string& param_name) {
  return nested_param<bool>(nested_module_names, param_name);
}

std::optional<int> log_int(const std::vector<std::string>& nested_module_names, const std::string& param_name) {
  return nested_param<int>(nested_module_names, param_name);
}

std::optional<double> log_double(const std::vector<std::string>& nested_module_names, const std::string& param_name) {
  return nested_param<double>(nested_module_names, param_name);
}

std::optional<std::string> log_string(const std::vector<std::string>& nested_module_names, const std::string& param_name) {
  return nested_param<std::string>(nested_module_names, param_name);
}

void warn_log_compat_once(const std::string& legacy_key, const std::string& new_key) {
  if (legacy_key.empty() || new_key.empty()) {
    return;
  }

  static std::mutex warned_mutex;
  static std::set<std::string> warned_keys;
  const std::string combined = legacy_key + "->" + new_key;

  std::lock_guard<std::mutex> lock(warned_mutex);
  if (!warned_keys.insert(combined).second) {
    return;
  }

  spdlog::warn("legacy log config '{}' is deprecated; use '{}' instead", legacy_key, new_key);
}

std::string legacy_path_to_filename(const std::optional<std::string>& legacy_path, const std::string& default_filename) {
  if (!legacy_path || legacy_path->empty()) {
    return default_filename;
  }

  const std::filesystem::path path(*legacy_path);
  const auto filename = path.filename().string();
  return filename.empty() ? default_filename : filename;
}

}  // namespace iap
