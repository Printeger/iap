#include <iap/common/log_paths.hpp>

#include <chrono>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <system_error>
#include <vector>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <Eigen/Core>

#include <iap/common/log_config.hpp>
#include <iap/util/config.hpp>

#include <unistd.h>

namespace iap {

namespace {

std::string make_timestamp_string() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf {};
#if defined(_WIN32)
  localtime_s(&tm_buf, &now_time);
#else
  localtime_r(&now_time, &tm_buf);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

std::string sanitize_run_name(const std::string& run_name) {
  std::string sanitized;
  sanitized.reserve(run_name.size());
  for (const char c : run_name) {
    if ((c >= 'a' && c <= 'z') ||
        (c >= 'A' && c <= 'Z') ||
        (c >= '0' && c <= '9') ||
        c == '-' || c == '_') {
      sanitized.push_back(c);
    } else if (!sanitized.empty() && sanitized.back() != '_') {
      sanitized.push_back('_');
    }
  }
  while (!sanitized.empty() && sanitized.back() == '_') {
    sanitized.pop_back();
  }
  return sanitized;
}

std::filesystem::path detect_package_root() {
  std::filesystem::path path(__FILE__);
  path = std::filesystem::absolute(path).lexically_normal();

  for (std::filesystem::path candidate = path.parent_path();
       !candidate.empty();
       candidate = candidate.parent_path()) {
    if (std::filesystem::exists(candidate / "README.md") &&
        std::filesystem::exists(candidate / "config" / "config.json") &&
        std::filesystem::exists(candidate / "src")) {
      return candidate;
    }
  }

  const auto cwd = std::filesystem::current_path();
  if (std::filesystem::exists(cwd / "src" / "iap" / "README.md")) {
    return cwd / "src" / "iap";
  }

  return cwd;
}

std::filesystem::path resolve_root_dir(const std::filesystem::path& package_root, const std::string& root_dir) {
  std::filesystem::path path(root_dir);
  if (path.empty()) {
    path = "log";
  }
  if (path.is_relative()) {
    path = package_root / path;
  }
  return std::filesystem::absolute(path).lexically_normal();
}

std::filesystem::path choose_run_dir(const std::filesystem::path& root_dir, const LogConfig& config) {
  const bool fixed_name =
    config.run_dir_mode == "FIXED_NAME" ||
    config.run_dir_mode == "fixed_name" ||
    config.run_dir_mode == "fixed";

  std::string base_name;
  if (fixed_name) {
    base_name = sanitize_run_name(config.run_name);
    if (base_name.empty()) {
      base_name = "run";
    }
  } else {
    base_name = make_timestamp_string();
    const std::string run_name = sanitize_run_name(config.run_name);
    if (!run_name.empty()) {
      base_name += "_" + run_name;
    }
  }

  std::filesystem::path candidate = root_dir / base_name;
  if (!std::filesystem::exists(candidate)) {
    return candidate;
  }

  for (int suffix = 1; suffix < 1000; ++suffix) {
    const auto with_suffix = root_dir / (base_name + "_" + std::to_string(suffix));
    if (!std::filesystem::exists(with_suffix)) {
      return with_suffix;
    }
  }

  return root_dir / (base_name + "_overflow");
}

std::optional<nlohmann::json> try_read_json(const std::filesystem::path& path) {
  if (path.empty() || !std::filesystem::exists(path)) {
    return std::nullopt;
  }

  std::ifstream ifs(path);
  if (!ifs) {
    return std::nullopt;
  }

  nlohmann::json json;
  try {
    ifs >> json;
  } catch (...) {
    return std::nullopt;
  }
  return json;
}

std::optional<glim::Config> load_named_config_if_exists(const std::string& config_name) {
  const auto path = glim::GlobalConfig::get_config_path(config_name);
  if (path.empty() || !std::filesystem::exists(path)) {
    return std::nullopt;
  }
  return glim::Config(path);
}

bool module_list_contains(
  const std::vector<std::string>& modules,
  std::initializer_list<const char*> patterns) {
  for (const auto& module : modules) {
    for (const char* pattern : patterns) {
      if (module.find(pattern) != std::string::npos) {
        return true;
      }
    }
  }
  return false;
}

std::string normalize_final_pose_surface(const std::string& surface) {
  if (surface == "strict_local" || surface == "STRICT_LOCAL" || surface == "strict") {
    return "strict_local";
  }
  return "active_window";
}

std::string normalize_gravity_mode(const std::string& mode) {
  if (mode == "fixed_norm" || mode == "FIXED_NORM" || mode == "fixed-norm") {
    return "fixed_norm";
  }
  if (mode == "limited_tilt" || mode == "LIMITED_TILT" || mode == "limited-tilt") {
    return "limited_tilt";
  }
  if (mode == "warmup_freeze_then_release" ||
      mode == "WARMUP_FREEZE_THEN_RELEASE" ||
      mode == "warmup-freeze-then-release") {
    return "warmup_freeze_then_release";
  }
  return "normal";
}

std::string normalize_gravity_state_mode(const std::string& mode) {
  if (mode == "external_reference" || mode == "EXTERNAL_REFERENCE" || mode == "external-reference") {
    return "external_reference";
  }
  return "shared_optimized";
}

std::string normalize_gravity_reference_source(const std::string& source) {
  if (source == "config_vector" || source == "CONFIG_VECTOR" || source == "config-vector") {
    return "config_vector";
  }
  return "startup_seed";
}

std::string normalize_velocity_state_mode(const std::string& mode) {
  if (mode == "keep_but_not_optimize" || mode == "KEEP_BUT_NOT_OPTIMIZE" || mode == "keep-but-not-optimize") {
    return "keep_but_not_optimize";
  }
  return "optimize";
}

std::string normalize_velocity_mode_policy(const std::string& policy) {
  if (policy == "auto_disable_without_gnss" ||
      policy == "AUTO_DISABLE_WITHOUT_GNSS" ||
      policy == "auto-disable-without-gnss") {
    return "auto_disable_without_gnss";
  }
  return "always_optimize";
}

std::string normalize_bias_state_mode(const std::string& mode) {
  if (mode == "lagged_keyed" || mode == "LAGGED_KEYED" || mode == "lagged-keyed") {
    return "lagged_keyed";
  }
  return "shared_singleton";
}

std::string normalize_frontend_seed_mode(const std::string& mode) {
  if (mode == "imu_forward_prediction" ||
      mode == "IMU_FORWARD_PREDICTION" ||
      mode == "imu-forward-prediction") {
    return "imu_forward_prediction";
  }
  return "last_pose_copy";
}

std::string yaw_isolation_experiment_name(
  const std::string& gravity_mode,
  bool freeze_gravity,
  bool freeze_gyro_bias,
  bool freeze_accel_bias,
  bool disable_velocity_factor,
  bool disable_current_velocity_prior) {
  std::vector<std::string> parts;
  if (freeze_gravity) {
    parts.emplace_back("legacy_freeze_gravity");
  } else if (gravity_mode != "normal") {
    parts.emplace_back("gravity_" + gravity_mode);
  }
  if (freeze_gyro_bias) {
    parts.emplace_back("freeze_gyro_bias");
  }
  if (freeze_accel_bias) {
    parts.emplace_back("freeze_accel_bias");
  }
  if (disable_velocity_factor) {
    parts.emplace_back("disable_velocity_factor");
  }
  if (disable_current_velocity_prior) {
    parts.emplace_back("disable_current_velocity_prior");
  }
  if (parts.empty()) {
    return "baseline";
  }

  std::ostringstream oss;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      oss << "+";
    }
    oss << parts[i];
  }
  return oss.str();
}

std::string run_command_capture(const std::string& command) {
  std::array<char, 256> buffer {};
  std::string output;

  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return output;
  }

  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe)) {
    output += buffer.data();
  }
  pclose(pipe);

  while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
    output.pop_back();
  }
  return output;
}

void write_text_file(const std::filesystem::path& path, const std::string& content) {
  std::ofstream ofs(path);
  if (!ofs) {
    spdlog::warn("failed to open {}", path.string());
    return;
  }
  ofs << content;
}

void write_json_file(const std::filesystem::path& path, const nlohmann::json& json) {
  std::ofstream ofs(path);
  if (!ofs) {
    spdlog::warn("failed to open {}", path.string());
    return;
  }
  ofs << std::setw(2) << json << std::endl;
}

nlohmann::json read_json_file_or_empty(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    return nlohmann::json::object();
  }

  std::ifstream ifs(path);
  if (!ifs) {
    spdlog::warn("failed to open {}", path.string());
    return nlohmann::json::object();
  }

  try {
    nlohmann::json json;
    ifs >> json;
    if (!json.is_object()) {
      return nlohmann::json::object();
    }
    return json;
  } catch (const std::exception& e) {
    spdlog::warn("failed to parse {}: {}", path.string(), e.what());
    return nlohmann::json::object();
  }
}

void update_latest_symlink(const std::filesystem::path& root_dir, const std::filesystem::path& run_dir) {
  const auto latest = root_dir / "latest";
  std::error_code ec;
  std::filesystem::remove(latest, ec);
  ec.clear();
  std::filesystem::create_directory_symlink(run_dir.filename(), latest, ec);
  if (ec) {
    spdlog::warn("failed to update latest symlink '{}': {}", latest.string(), ec.message());
  }
}

void cleanup_old_runs(const std::filesystem::path& root_dir,
                      const std::filesystem::path& current_run_dir,
                      int keep_last_n_runs) {
  if (keep_last_n_runs <= 0 || !std::filesystem::exists(root_dir)) {
    return;
  }

  std::vector<std::filesystem::path> run_dirs;
  for (const auto& entry : std::filesystem::directory_iterator(root_dir)) {
    if (!entry.is_directory()) {
      continue;
    }
    const auto filename = entry.path().filename().string();
    if (filename == "latest") {
      continue;
    }
    run_dirs.push_back(entry.path());
  }

  std::sort(run_dirs.begin(), run_dirs.end());
  while (static_cast<int>(run_dirs.size()) > keep_last_n_runs) {
    const auto oldest = run_dirs.front();
    run_dirs.erase(run_dirs.begin());
    if (oldest == current_run_dir) {
      continue;
    }

    std::error_code ec;
    std::filesystem::remove_all(oldest, ec);
    if (ec) {
      spdlog::warn("failed to remove old run directory '{}': {}", oldest.string(), ec.message());
    }
  }
}

void write_metadata_files(const LogPaths& paths, const LogConfig& config) {
  nlohmann::json run_info;
  run_info["package_root"] = paths.package_root().string();
  run_info["log_root"] = paths.root_dir().string();
  run_info["run_dir"] = paths.run_dir().string();
  run_info["run_dir_mode"] = config.run_dir_mode;
  run_info["run_name"] = config.run_name;
  run_info["pid"] = static_cast<long long>(::getpid());
  run_info["timestamp"] = make_timestamp_string();
  run_info["config_log_profiling_solver_update_profile"] = config.profiling.solver_update_profile;
  run_info["runtime_log_profiling_solver_update_profile"] = config.profiling.solver_update_profile;
  run_info["config_log_profiling_lidar_factor_internal_profile"] = config.profiling.lidar_factor_internal_profile;
  run_info["runtime_log_profiling_lidar_factor_internal_profile"] = config.profiling.lidar_factor_internal_profile;
  run_info["config_log_profiling_jump_diagnostics"] = config.profiling.jump_diagnostics;
  run_info["runtime_log_profiling_jump_diagnostics"] = config.profiling.jump_diagnostics;
  if (const auto odom_config = load_named_config_if_exists("config_odometry")) {
    const std::string configured_final_pose_surface =
      odom_config->param<std::string>("odometry_estimation", "final_pose_surface", "strict_local");
    const std::string configured_gravity_mode =
      odom_config->param<std::string>("odometry_estimation", "exp.gravity_mode", "normal");
    const double configured_gravity_fixed_norm_value =
      odom_config->param<double>("odometry_estimation", "exp.gravity_fixed_norm_value", 9.80665);
    const double configured_gravity_tilt_limit_rad =
      odom_config->param<double>("odometry_estimation", "exp.gravity_tilt_limit_rad", 0.02);
    const int configured_gravity_warmup_freeze_frames =
      odom_config->param<int>("odometry_estimation", "exp.gravity_warmup_freeze_frames", 20);
    const bool configured_exp_freeze_gravity =
      odom_config->param<bool>("odometry_estimation", "exp_freeze_gravity", false);
    const bool configured_exp_freeze_gyro_bias =
      odom_config->param<bool>("odometry_estimation", "exp_freeze_gyro_bias", false);
    const bool configured_exp_freeze_accel_bias =
      odom_config->param<bool>("odometry_estimation", "exp_freeze_accel_bias", false);
    const bool configured_exp_disable_velocity_factor =
      odom_config->param<bool>("odometry_estimation", "exp_disable_velocity_factor", false);
    const bool configured_exp_disable_current_velocity_prior =
      odom_config->param<bool>("odometry_estimation", "exp_disable_current_velocity_prior", false);
    const std::string configured_gravity_state_mode = normalize_gravity_state_mode(
      odom_config->param<std::string>("odometry_estimation", "gravity_state_mode", "shared_optimized"));
    const std::string configured_gravity_reference_source = normalize_gravity_reference_source(
      odom_config->param<std::string>("odometry_estimation", "gravity_reference_source", "startup_seed"));
    const Eigen::Vector3d configured_gravity_reference_vector =
      odom_config->param<Eigen::Vector3d>(
        "odometry_estimation",
        "gravity_reference_vector",
        Eigen::Vector3d(0.0, 0.0, 9.80665));
    const std::string configured_velocity_state_mode = normalize_velocity_state_mode(
      odom_config->param<std::string>("odometry_estimation", "velocity_state_mode", "optimize"));
    const std::string configured_velocity_mode_policy = normalize_velocity_mode_policy(
      odom_config->param<std::string>("odometry_estimation", "velocity_mode_policy", "always_optimize"));
    const std::string configured_bias_state_mode = normalize_bias_state_mode(
      odom_config->param<std::string>("odometry_estimation", "bias_state_mode", "shared_singleton"));
    const std::string configured_frontend_seed_mode = normalize_frontend_seed_mode(
      odom_config->param<std::string>("odometry_estimation", "frontend_seed_mode", "last_pose_copy"));
    run_info["config_final_pose_surface"] = configured_final_pose_surface;
    run_info["runtime_final_pose_surface"] = normalize_final_pose_surface(configured_final_pose_surface);
    run_info["config_gravity_state_mode"] = configured_gravity_state_mode;
    run_info["runtime_gravity_state_mode"] = configured_gravity_state_mode;
    run_info["config_gravity_reference_source"] = configured_gravity_reference_source;
    run_info["runtime_gravity_reference_source"] =
      configured_gravity_state_mode == "external_reference"
        ? configured_gravity_reference_source
        : "shared_optimized_graph_state";
    run_info["config_gravity_reference_vector"] = {
      configured_gravity_reference_vector.x(),
      configured_gravity_reference_vector.y(),
      configured_gravity_reference_vector.z(),
    };
    run_info["runtime_gravity_reference_vector"] = run_info["config_gravity_reference_vector"];
    run_info["config_gravity_mode"] = configured_gravity_mode;
    run_info["runtime_gravity_mode"] =
      configured_exp_freeze_gravity ? "legacy_freeze_gravity" : normalize_gravity_mode(configured_gravity_mode);
    run_info["config_gravity_fixed_norm_value"] = configured_gravity_fixed_norm_value;
    run_info["runtime_gravity_fixed_norm_value"] = configured_gravity_fixed_norm_value;
    run_info["config_gravity_tilt_limit_rad"] = configured_gravity_tilt_limit_rad;
    run_info["runtime_gravity_tilt_limit_rad"] = configured_gravity_tilt_limit_rad;
    run_info["config_gravity_warmup_freeze_frames"] = configured_gravity_warmup_freeze_frames;
    run_info["runtime_gravity_warmup_freeze_frames"] = configured_gravity_warmup_freeze_frames;
    run_info["config_exp_freeze_gravity"] = configured_exp_freeze_gravity;
    run_info["runtime_exp_freeze_gravity"] = configured_exp_freeze_gravity;
    run_info["config_exp_freeze_gyro_bias"] = configured_exp_freeze_gyro_bias;
    run_info["runtime_exp_freeze_gyro_bias"] = configured_exp_freeze_gyro_bias;
    run_info["config_exp_freeze_accel_bias"] = configured_exp_freeze_accel_bias;
    run_info["runtime_exp_freeze_accel_bias"] = configured_exp_freeze_accel_bias;
    run_info["config_exp_disable_velocity_factor"] = configured_exp_disable_velocity_factor;
    run_info["runtime_exp_disable_velocity_factor"] = configured_exp_disable_velocity_factor;
    run_info["config_exp_disable_current_velocity_prior"] = configured_exp_disable_current_velocity_prior;
    run_info["runtime_exp_disable_current_velocity_prior"] = configured_exp_disable_current_velocity_prior;
    run_info["config_velocity_state_mode"] = configured_velocity_state_mode;
    run_info["runtime_velocity_state_mode"] = configured_velocity_state_mode;
    run_info["config_velocity_mode_policy"] = configured_velocity_mode_policy;
    run_info["runtime_velocity_mode_policy"] = configured_velocity_mode_policy;
    run_info["config_bias_state_mode"] = configured_bias_state_mode;
    run_info["runtime_bias_state_mode"] = configured_bias_state_mode;
    run_info["runtime_bias_optimized"] = true;
    run_info["runtime_bias_source_of_truth"] =
      configured_bias_state_mode == "lagged_keyed"
        ? "active_lagged_bias_keys"
        : "shared_singleton_registry";
    run_info["runtime_bias_transition_prior_enabled"] = configured_bias_state_mode == "lagged_keyed";
    run_info["runtime_bias_transition_prior_strength"] =
      odom_config->param<double>("odometry_estimation", "imu_ct_bias_inf_scale", 1e3);
    run_info["runtime_bias_can_be_survivor_anchor"] = configured_bias_state_mode != "lagged_keyed";
    run_info["runtime_bias_writeback_mode"] =
      configured_bias_state_mode == "lagged_keyed"
        ? "lagged_authoritative_with_mirror_cache"
        : "shared_singleton_authoritative_writeback";
    run_info["config_frontend_seed_mode"] = configured_frontend_seed_mode;
    run_info["runtime_frontend_seed_mode"] = configured_frontend_seed_mode;
    run_info["runtime_imu_forward_prediction_enabled"] =
      configured_frontend_seed_mode == "imu_forward_prediction";
    run_info["runtime_frontend_seed_fallback_used"] = false;
    run_info["runtime_frontend_seed_source"] = "last_pose_copy";
    run_info["runtime_frontend_seed_imu_sample_count"] = 0;
    run_info["runtime_frontend_target_time_kind"] = "scan_start";
    run_info["runtime_frontend_target_time_source"] = "current_source_frame.scan_start";
    run_info["runtime_frontend_target_time"] = 0.0;
    run_info["runtime_start_pose_query_time"] = 0.0;
    run_info["runtime_frontend_pose_query_time"] = 0.0;
    run_info["runtime_seed_integration_end_time"] = 0.0;
    run_info["runtime_bucket_query_time"] = 0.0;
    run_info["runtime_frontend_target_time_consistent"] = false;
    run_info["runtime_frontend_target_time_offset_vs_representative"] = 0.0;
    run_info["runtime_frontend_target_time_offset_vs_scan_start"] = 0.0;
    run_info["runtime_frontend_target_time_offset_vs_scan_end"] = 0.0;
    run_info["runtime_has_gnss_constraints"] = false;
    run_info["runtime_velocity_optimized"] =
      configured_velocity_state_mode == "optimize" && configured_velocity_mode_policy == "always_optimize";
    run_info["runtime_experiment_name"] = yaw_isolation_experiment_name(
      normalize_gravity_mode(configured_gravity_mode),
      configured_exp_freeze_gravity,
      configured_exp_freeze_gyro_bias,
      configured_exp_freeze_accel_bias,
      configured_exp_disable_velocity_factor,
      configured_exp_disable_current_velocity_prior);
  }
  write_json_file(paths.metadata_path(config.metadata.run_info_file), run_info);

  if (config.metadata.write_config_snapshot) {
    nlohmann::json snapshot;

    const auto* global = glim::GlobalConfig::instance();
    if (global) {
      const auto config_path = global->param<std::string>("global", "config_path", std::string("."));
      snapshot["config_path"] = config_path;
      if (auto json = try_read_json(std::filesystem::path(config_path) / "config.json")) {
        snapshot["config.json"] = *json;
      }
    }

    for (const auto* config_name : {
           "config_logging",
           "config_ros",
           "config_viewer",
           "config_preprocess",
           "config_sensors",
           "config_odometry",
           "config_sub_mapping",
           "config_global_mapping",
           "config_gnss"}) {
      const auto path = glim::GlobalConfig::get_config_path(config_name);
      if (auto json = try_read_json(path)) {
        snapshot[config_name] = *json;
      }
    }

    write_json_file(paths.metadata_path(config.metadata.config_snapshot_file), snapshot);
  }

  if (config.metadata.write_git_revision) {
    const std::string git_rev = run_command_capture(
      "git -C \"" + paths.package_root().string() + "\" rev-parse HEAD 2>/dev/null");
    write_text_file(paths.metadata_path(config.metadata.git_rev_file), git_rev + "\n");
  }

  if (config.metadata.write_build_info) {
    std::ostringstream oss;
    oss << "iap_version=" << IAP_VERSION << "\n";
    oss << "compiled_at=" << __DATE__ << " " << __TIME__ << "\n";
    oss << "package_root=" << paths.package_root().string() << "\n";
    write_text_file(paths.metadata_path(config.metadata.build_info_file), oss.str());
  }

  if (config.metadata.write_mode_manifest) {
    nlohmann::json manifest;
    const auto odom_config = load_named_config_if_exists("config_odometry");
    const auto ros_config = load_named_config_if_exists("config_ros");

    const bool frontend_only_expected =
      odom_config ? odom_config->param<bool>("odometry_estimation", "frontend_only_mode", false) : false;
    const bool backend_expected_active = !frontend_only_expected;

    const bool enable_local_mapping =
      ros_config ? ros_config->param<bool>("glim_ros", "enable_local_mapping", true) : false;
    const bool enable_global_mapping =
      ros_config ? ros_config->param<bool>("glim_ros", "enable_global_mapping", true) : false;
    const bool mapping_expected_active = enable_local_mapping || enable_global_mapping;

    const std::vector<std::string> extension_modules =
      ros_config ? ros_config->param<std::vector<std::string>>("glim_ros", "extension_modules", {}) : std::vector<std::string>{};
    const bool gnss_expected_active = module_list_contains(extension_modules, {"gnss", "integrity"});

    manifest["frontend_only_expected"] = frontend_only_expected;
    manifest["frontend_only_observed"] = frontend_only_expected;
    manifest["backend_expected_active"] = backend_expected_active;
    manifest["backend_observed_active"] = backend_expected_active;
    manifest["mapping_expected_active"] = mapping_expected_active;
    manifest["mapping_observed_active"] = mapping_expected_active;
    manifest["gnss_expected_active"] = gnss_expected_active;
    manifest["gnss_observed_active"] = gnss_expected_active;
    manifest["observation_basis"] = "config";
    manifest["configured_extension_modules"] = extension_modules;

    write_json_file(paths.metadata_path(config.metadata.mode_manifest_file), manifest);
  }
}

std::unique_ptr<LogPaths>& cached_paths() {
  static std::unique_ptr<LogPaths> paths;
  return paths;
}

}  // namespace

LogPaths& LogPaths::instance() {
  auto& paths = cached_paths();
  if (!paths) {
    paths.reset(new LogPaths());
  }
  return *paths;
}

void LogPaths::reset() {
  reset_log_config();
  cached_paths().reset();
}

LogPaths::LogPaths() {
  initialize();
}

void LogPaths::initialize() {
  const auto& config = get_log_config();

  package_root_ = detect_package_root();
  root_dir_ = resolve_root_dir(package_root_, config.root_dir);
  run_dir_ = choose_run_dir(root_dir_, config);
  runtime_dir_ = run_dir_ / "runtime";
  profiling_dir_ = run_dir_ / "profiling";
  export_dir_ = run_dir_ / "export";
  metadata_dir_ = run_dir_ / "metadata";

  std::filesystem::create_directories(runtime_dir_);
  std::filesystem::create_directories(profiling_dir_);
  std::filesystem::create_directories(export_dir_);
  std::filesystem::create_directories(metadata_dir_);

  if (config.create_latest_symlink) {
    update_latest_symlink(root_dir_, run_dir_);
  }
  cleanup_old_runs(root_dir_, run_dir_, config.keep_last_n_runs);
  write_metadata_files(*this, config);
}

std::filesystem::path LogPaths::runtime_main_log_path() const {
  return runtime_dir_ / get_log_config().runtime.main_file_name;
}

std::filesystem::path LogPaths::runtime_module_log_path(const std::string& module_name) const {
  std::string file_name = get_log_config().runtime.module_file_pattern;
  const std::string placeholder = "{module}";
  const auto pos = file_name.find(placeholder);
  if (pos == std::string::npos) {
    file_name = "glim_" + module_name + ".log";
  } else {
    file_name.replace(pos, placeholder.size(), module_name);
  }
  return runtime_dir_ / file_name;
}

std::filesystem::path LogPaths::warnings_log_path() const {
  return runtime_dir_ / get_log_config().runtime.warnings_file_name;
}

std::filesystem::path LogPaths::profiling_path(const std::string& file_name) const {
  return profiling_dir_ / file_name;
}

std::filesystem::path LogPaths::export_path(const std::string& file_name) const {
  return export_dir_ / file_name;
}

std::filesystem::path LogPaths::metadata_path(const std::string& file_name) const {
  return metadata_dir_ / file_name;
}

void merge_run_info_metadata(const nlohmann::json& patch) {
  if (!patch.is_object() || patch.empty()) {
    return;
  }

  const auto path = LogPaths::instance().metadata_path(get_log_config().metadata.run_info_file);
  auto run_info = read_json_file_or_empty(path);
  for (const auto& [key, value] : patch.items()) {
    run_info[key] = value;
  }
  write_json_file(path, run_info);
}

}  // namespace iap
