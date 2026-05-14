#pragma once

#include <iap/util/config.hpp>
#include <iap/util/run_log_manager.hpp>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <string>

namespace iap::timing_csv {

inline std::mutex& file_mutex() {
  static std::mutex m;
  return m;
}

inline bool enabled() {
  static bool initialized = false;
  static bool value = true;
  if (!initialized) {
    value = glim::GlobalConfig::instance()->param<bool>("global", "enable_timing_csv", true);
    initialized = true;
  }
  return value;
}

inline const std::string& path() {
  static std::string value;
  if (!value.empty()) return value;
  if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
    value = run_logs->profiling_path("iap_timing.csv").string();
    return value;
  }
  // Fallback: use IAP_SOURCE_ROOT to construct a sensible default.
  // The hardcoded default was previously a machine-specific path
  // (/home/dev/code/ws_iap/...) that broke when the workspace moved.
  value = glim::GlobalConfig::instance()->param<std::string>(
      "global", "timing_csv_path", std::string(IAP_SOURCE_ROOT) + "/log/res/iap_timing.csv");
  return value;
}

inline void ensure_header() {
  static std::once_flag once;
  std::call_once(once, [] {
    if (!enabled()) return;
    const std::filesystem::path csv_path(path());
    if (csv_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(csv_path.parent_path(), ec);
    }
    FILE* f = std::fopen(path().c_str(), "w");
    if (!f) return;
    std::fprintf(f, "stamp,module,elapsed_ms\n");
    std::fclose(f);
  });
}

inline void append(double stamp, const char* module, double elapsed_ms) {
  if (!enabled()) return;
  ensure_header();

  std::lock_guard<std::mutex> lk(file_mutex());
  FILE* f = std::fopen(path().c_str(), "a");
  if (!f) return;
  std::fprintf(f, "%.6f,%s,%.3f\n", stamp, module, elapsed_ms);
  std::fclose(f);
}

class ScopedTimer {
public:
  using Clock = std::chrono::steady_clock;

  ScopedTimer(double stamp, const char* module)
  : enabled_(enabled()),
    stamp_(stamp),
    module_(module) {
    if (enabled_) {
      start_ = Clock::now();
    }
  }

  ~ScopedTimer() {
    if (!enabled_) return;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    append(stamp_, module_, elapsed_ms);
  }

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
  bool enabled_ = false;
  double stamp_ = 0.0;
  const char* module_ = "";
  Clock::time_point start_{};
};

class CumulativeTimer {
public:
  using Clock = std::chrono::steady_clock;

  CumulativeTimer() : enabled_(enabled()) {}

  void start() {
    if (enabled_) t0_ = Clock::now();
  }

  void stop() {
    if (enabled_) accumulated_ms_ +=
        std::chrono::duration<double, std::milli>(Clock::now() - t0_).count();
  }

  void reset() { accumulated_ms_ = 0.0; }

  double accumulated_ms() const { return accumulated_ms_; }

  void flush(double stamp, const char* module) {
    if (enabled_ && accumulated_ms_ > 0.0) {
      append(stamp, module, accumulated_ms_);
      accumulated_ms_ = 0.0;
    }
  }

  CumulativeTimer(const CumulativeTimer&) = delete;
  CumulativeTimer& operator=(const CumulativeTimer&) = delete;

private:
  bool enabled_ = false;
  Clock::time_point t0_{};
  double accumulated_ms_ = 0.0;
};

}  // namespace iap::timing_csv
