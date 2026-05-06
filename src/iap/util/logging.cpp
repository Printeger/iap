#include <filesystem>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/run_log_manager.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace glim {

std::shared_ptr<spdlog::logger> get_default_logger() {
  return spdlog::default_logger();
}

void set_default_logger(const std::shared_ptr<spdlog::logger>& logger) {
  spdlog::set_default_logger(logger);
}

namespace {
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> ringbuffer_sink;
}

std::string sanitize_log_name(const std::string& name) {
  std::string sanitized = name;
  for (char& ch : sanitized) {
    const bool keep = (ch >= 'a' && ch <= 'z') ||
                      (ch >= 'A' && ch <= 'Z') ||
                      (ch >= '0' && ch <= '9') ||
                      ch == '_' || ch == '-';
    if (!keep) {
      ch = '_';
    }
  }
  return sanitized.empty() ? "main" : sanitized;
}

std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> get_ringbuffer_sink(int buffer_size) {
  if (!ringbuffer_sink) {
    ringbuffer_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(buffer_size);
  }
  return ringbuffer_sink;
}

std::shared_ptr<spdlog::logger> create_module_logger(const std::string& module_name) {
  std::shared_ptr<spdlog::logger> logger = spdlog::get(module_name);
  if (logger) {
    return logger;
  }

  const auto* config = glim::GlobalConfig::get_if_initialized();
  const std::string log_filename = sanitize_log_name(module_name == "glim" ? "main" : module_name);

  std::filesystem::path log_path;
  if (const auto* run_logs = RunLogManager::get_if_initialized()) {
    log_path = run_logs->runtime_path("iap_" + log_filename + ".log");
  } else {
    const std::string log_dir = config
      ? config->param<std::string>("logging", "log_dir", std::string("/tmp"))
      : std::string("/tmp");
    log_path = std::filesystem::path(log_dir) / ("iap_" + log_filename + ".log");
  }

  if (log_path.has_parent_path() && !std::filesystem::exists(log_path.parent_path())) {
    std::filesystem::create_directories(log_path.parent_path());
  }

  logger = spdlog::stdout_color_mt(module_name);
  logger->sinks().push_back(get_ringbuffer_sink());

  if (config && !config->param<bool>("logging", "save_logs", true)) {
    return logger;
  }

  const bool rotate_logs = config ? config->param<bool>("logging", "rotate_logs", true) : true;
  if (rotate_logs) {
    const size_t max_file_size_kb = config ? config->param<int>("logging", "max_file_size_kb", 8192) : 8192;
    const size_t max_file_size_bytes = max_file_size_kb * 1024;
    const size_t max_files = config ? config->param<int>("logging", "max_files", 10) : 10;

    auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(log_path.string(), max_file_size_bytes, max_files);
    logger->sinks().push_back(rotating_sink);
  } else {
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path.string(), true);
    logger->sinks().push_back(file_sink);
  }

  logger->set_level(get_default_logger()->level());

  return logger;
}

}  // namespace glim
