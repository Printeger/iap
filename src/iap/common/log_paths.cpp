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

}  // namespace iap
