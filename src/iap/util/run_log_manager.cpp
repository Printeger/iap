#include <iap/util/run_log_manager.hpp>

#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#include <unistd.h>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <iap/util/config.hpp>

namespace glim {

namespace {

std::unique_ptr<RunLogManager> g_run_log_manager;

std::string iso_utc_timestamp(std::time_t t) {
  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

std::string run_directory_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
  const auto tt = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &tt);
#else
  gmtime_r(&tt, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%dT%H%M%SZ")
      << "_" << std::setw(3) << std::setfill('0') << millis;
  return oss.str();
}

std::optional<std::string> read_command_output(const std::string& command) {
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return std::nullopt;
  }

  std::string output;
  char buffer[256];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  const int rc = pclose(pipe);
  if (rc != 0 || output.empty()) {
    return std::nullopt;
  }

  while (!output.empty() && (output.back() == '\n' || output.back() == '\r' || output.back() == ' ')) {
    output.pop_back();
  }
  return output.empty() ? std::nullopt : std::optional<std::string>(output);
}

std::string getenv_or_empty(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

}  // namespace

RunLogManager& RunLogManager::initialize(const std::string& process_name,
                                         const std::string& config_dir_or_empty) {
  if (!g_run_log_manager) {
    g_run_log_manager.reset(new RunLogManager(process_name, config_dir_or_empty));
  }
  return *g_run_log_manager;
}

RunLogManager& RunLogManager::instance() {
  if (!g_run_log_manager) {
    throw std::runtime_error("RunLogManager is not initialized");
  }
  return *g_run_log_manager;
}

RunLogManager* RunLogManager::get_if_initialized() {
  return g_run_log_manager.get();
}

RunLogManager::RunLogManager(std::string process_name, std::string config_dir_or_empty)
    : process_name_(std::move(process_name)),
      config_dir_(std::move(config_dir_or_empty)),
      start_timestamp_(run_directory_timestamp()),
      start_timestamp_iso_(iso_utc_timestamp(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()))) {
  log_root_ = resolve_log_root();
  run_dir_ = log_root_ / start_timestamp_;
  create_layout();
  update_latest_symlink();
}

const std::filesystem::path& RunLogManager::log_root() const {
  return log_root_;
}

const std::filesystem::path& RunLogManager::run_dir() const {
  return run_dir_;
}

std::filesystem::path RunLogManager::runtime_path(const std::string& name) const {
  return category_path("runtime", name);
}

std::filesystem::path RunLogManager::profiling_path(const std::string& name) const {
  return category_path("profiling", name);
}

std::filesystem::path RunLogManager::export_path(const std::string& name) const {
  return category_path("export", name);
}

std::filesystem::path RunLogManager::metadata_path(const std::string& name) const {
  return category_path("metadata", name);
}

std::filesystem::path RunLogManager::category_path(const std::string& category,
                                                   const std::string& name) const {
  const std::filesystem::path base = run_dir_ / category;
  return name.empty() ? base : (base / name);
}

std::filesystem::path RunLogManager::resolve_log_root() const {
  const auto* config = GlobalConfig::get_if_initialized();
  const std::string default_root = (std::filesystem::current_path() / "log").string();
  const std::string configured_root = config
    ? config->param<std::string>("logging", "log_dir", default_root)
    : default_root;
  return std::filesystem::path(configured_root);
}

void RunLogManager::create_layout() {
  std::error_code ec;
  std::filesystem::create_directories(runtime_path(""), ec);
  ec.clear();
  std::filesystem::create_directories(profiling_path(""), ec);
  ec.clear();
  std::filesystem::create_directories(export_path(""), ec);
  ec.clear();
  std::filesystem::create_directories(metadata_path(""), ec);
}

void RunLogManager::update_latest_symlink() const {
  const auto latest = log_root_ / "latest";
  std::error_code ec;

  if (std::filesystem::exists(latest, ec) || std::filesystem::is_symlink(latest, ec)) {
    ec.clear();
    if (std::filesystem::is_symlink(latest, ec)) {
      std::filesystem::remove(latest, ec);
    } else {
      std::filesystem::remove_all(latest, ec);
    }
    if (ec) {
      spdlog::warn("[RunLogManager] failed to remove existing latest link '{}': {}",
                   latest.string(), ec.message());
      return;
    }
  }

  ec.clear();
  std::filesystem::create_directory_symlink(run_dir_.filename(), latest, ec);
  if (ec) {
    spdlog::warn("[RunLogManager] failed to create latest symlink '{}': {}",
                 latest.string(), ec.message());
  }
}

std::map<std::string, std::string> RunLogManager::collect_run_info_fields() const {
  std::map<std::string, std::string> fields;

  fields["start_timestamp"] = start_timestamp_iso_;
  fields["process_name"] = process_name_;
  fields["run_directory"] = run_dir_.string();
  fields["log_root"] = log_root_.string();
  fields["working_directory"] = std::filesystem::current_path().string();
  fields["config_dir"] = config_dir_;
  fields["build_type"] = IAP_BUILD_TYPE;
  fields["source_root"] = IAP_SOURCE_ROOT;

  char hostname[256] = {};
  if (gethostname(hostname, sizeof(hostname)) == 0) {
    fields["hostname"] = hostname;
  }

  std::string username = getenv_or_empty("USER");
  if (username.empty()) {
    username = getenv_or_empty("LOGNAME");
  }
  if (!username.empty()) {
    fields["username"] = username;
  }

  if (const auto git_commit = read_command_output("git -C \"" + std::string(IAP_SOURCE_ROOT) + "\" rev-parse HEAD 2>/dev/null")) {
    fields["git_commit"] = *git_commit;
  }

  if (const auto* config = GlobalConfig::get_if_initialized()) {
    fields["selected_config_root"] = config->param<std::string>("global", "config_path", std::string());
  }

  return fields;
}

void RunLogManager::write_run_info(const std::map<std::string, std::string>& extra_fields) const {
  nlohmann::json json;
  for (const auto& field : collect_run_info_fields()) {
    json[field.first] = field.second;
  }

  if (const auto* config = GlobalConfig::get_if_initialized()) {
    nlohmann::json config_paths = nlohmann::json::object();
    for (const auto& item : config->list_config_paths()) {
      config_paths[item.first] = item.second;
    }
    json["selected_config_paths"] = config_paths;
  }

  for (const auto& field : extra_fields) {
    json[field.first] = field.second;
  }

  std::error_code ec;
  std::filesystem::create_directories(metadata_path(""), ec);

  std::ofstream ofs(metadata_path("run_info.json"));
  if (!ofs.is_open()) {
    spdlog::warn("[RunLogManager] failed to open run_info.json for writing");
    return;
  }

  ofs << std::setw(2) << json << std::endl;
}

}  // namespace glim
