#include <filesystem>
#include <algorithm>
#include <cctype>
#include <string>

#include <iap/common/log_config.hpp>
#include <iap/common/log_paths.hpp>
#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
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

spdlog::level::level_enum parse_level(const std::string& level) {
  const std::string normalized = [&level] {
    std::string lower = level;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return lower;
  }();

  if (normalized == "trace") return spdlog::level::trace;
  if (normalized == "debug") return spdlog::level::debug;
  if (normalized == "warn" || normalized == "warning") return spdlog::level::warn;
  if (normalized == "err" || normalized == "error") return spdlog::level::err;
  if (normalized == "critical") return spdlog::level::critical;
  if (normalized == "off") return spdlog::level::off;
  return spdlog::level::info;
}
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

  const auto& log_config = iap::get_log_config();
  const auto& log_paths = iap::LogPaths::instance();
  const auto level = parse_level(log_config.runtime.level);

  logger = spdlog::stdout_color_mt(module_name);
  logger->sinks().push_back(get_ringbuffer_sink());

  if (log_config.runtime.enable_console) {
    logger->set_level(level);
  } else {
    logger->sinks().clear();
  }

  if (log_config.runtime.enable_file) {
    const std::filesystem::path log_file =
      module_name == "glim" ? log_paths.runtime_main_log_path() : log_paths.runtime_module_log_path(module_name);

    if (log_config.runtime.rotate_files) {
      const size_t max_file_size_bytes =
        static_cast<size_t>(std::max(log_config.runtime.max_file_size_kb, 1)) * 1024ULL;
      const size_t max_files = static_cast<size_t>(std::max(log_config.runtime.max_files, 1));
      auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_file.string(), max_file_size_bytes, max_files);
      logger->sinks().push_back(rotating_sink);
    } else {
      auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file.string(), true);
      logger->sinks().push_back(file_sink);
    }

    if (log_config.runtime.split_warnings_file) {
      auto warnings_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_paths.warnings_log_path().string(), true);
      warnings_sink->set_level(spdlog::level::warn);
      logger->sinks().push_back(warnings_sink);
    }
  }

  logger->set_level(level);
  spdlog::set_level(level);

  return logger;
}

}  // namespace glim
