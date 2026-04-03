#pragma once

#include <optional>
#include <string>
#include <vector>

namespace iap {

struct RuntimeLogConfig {
  bool enable_console = true;
  bool enable_file = true;
  std::string level = "INFO";
  bool split_warnings_file = false;
  std::string main_file_name = "glim_main.log";
  std::string module_file_pattern = "glim_{module}.log";
  std::string warnings_file_name = "warnings.log";
  bool rotate_files = true;
  int max_file_size_kb = 8192;
  int max_files = 10;
};

struct ProfilingLogConfig {
  bool enable = false;
  bool pipeline = false;
  bool frontend_frame = true;
  bool lidar_factor = false;
  bool frontend_lm_iteration = false;
  bool frame_warning_profile = false;
  bool target_map_prep_breakdown = false;
  bool graph_problem_size = false;
  bool numeric_reference = false;
  bool linearization_check = false;
  std::string pipeline_file = "pipeline_timing.csv";
  std::string frontend_frame_file = "frontend_frame_profile.csv";
  std::string lidar_factor_file = "lidar_factor_profile.csv";
  std::string frontend_lm_iteration_file = "frontend_lm_iteration.csv";
  std::string frame_warning_profile_file = "frame_warning_profile.csv";
  std::string numeric_reference_file = "numeric_reference.csv";
  std::string linearization_check_file = "linearization_check.csv";
};

struct LidarDegeneracyWarningsConfig {
  bool enable = false;
  double min_match_ratio = 0.5;
  double min_inlier_ratio = 0.35;
  double min_unique_target_ratio = 0.25;
  double max_target_reuse_ratio = 0.5;
  double max_ambiguity_rejection_ratio = 0.25;
  double min_mean_score_gap = 0.05;
};

struct WarningsLogConfig {
  LidarDegeneracyWarningsConfig lidar_degeneracy;
};

struct ExportLogConfig {
  bool baseline_csv = false;
  bool icp_quality_csv = false;
  bool gnss_factor_debug_csv = false;
  bool araim_csv = false;
  bool integrity_trajectory_csv = false;
  bool local_frontend_trajectory = false;
  bool backend_summary = false;
  bool bucket_stats = false;
  std::string baseline_csv_file = "ct_lidar_baseline.csv";
  std::string icp_quality_csv_file = "icp_quality.csv";
  std::string gnss_factor_debug_csv_file = "gnss_factor_debug.csv";
  std::string araim_csv_file = "araim.csv";
  std::string integrity_trajectory_csv_file = "integrity_trajectory.csv";
  std::string local_frontend_trajectory_file = "local_frontend_trajectory.csv";
  std::string backend_summary_file = "backend_summary.csv";
  std::string bucket_stats_file = "bucket_stats.csv";
};

struct SharedOutputLogConfig {
  bool publish_shared_trajectory = true;
  bool attach_trajectory_to_frames = true;
  bool attach_imu_rate_trajectory = true;
};

struct MetadataLogConfig {
  bool write_config_snapshot = true;
  bool write_git_revision = true;
  bool write_build_info = true;
  bool write_mode_manifest = false;
  std::string config_snapshot_file = "config_snapshot.json";
  std::string run_info_file = "run_info.json";
  std::string git_rev_file = "git_rev.txt";
  std::string build_info_file = "build_info.txt";
  std::string mode_manifest_file = "mode_manifest.json";
};

struct LogConfig {
  std::string root_dir = "log";
  std::string run_dir_mode = "TIMESTAMP";
  std::string run_name;
  bool create_latest_symlink = true;
  int keep_last_n_runs = 20;
  RuntimeLogConfig runtime;
  ProfilingLogConfig profiling;
  WarningsLogConfig warnings;
  ExportLogConfig export_outputs;
  SharedOutputLogConfig shared_output;
  MetadataLogConfig metadata;
};

const LogConfig& get_log_config();
void reset_log_config();

std::optional<bool> log_bool(const std::vector<std::string>& nested_module_names, const std::string& param_name);
std::optional<int> log_int(const std::vector<std::string>& nested_module_names, const std::string& param_name);
std::optional<double> log_double(const std::vector<std::string>& nested_module_names, const std::string& param_name);
std::optional<std::string> log_string(const std::vector<std::string>& nested_module_names, const std::string& param_name);

void warn_log_compat_once(const std::string& legacy_key, const std::string& new_key);
std::string legacy_path_to_filename(const std::optional<std::string>& legacy_path, const std::string& default_filename);

template <typename T>
T resolve_log_value(const std::optional<T>& new_value,
                    const std::optional<T>& legacy_value,
                    const std::string& legacy_key,
                    const std::string& new_key,
                    const T& default_value) {
  if (new_value) {
    return *new_value;
  }
  if (legacy_value) {
    warn_log_compat_once(legacy_key, new_key);
    return *legacy_value;
  }
  return default_value;
}

}  // namespace iap
