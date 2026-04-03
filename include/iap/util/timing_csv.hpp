#pragma once

#include <iap/common/log_config.hpp>
#include <iap/common/log_paths.hpp>
#include <iap/util/config.hpp>

#include <cstdio>
#include <mutex>
#include <string>

namespace iap::timing_csv {

inline std::mutex& file_mutex() {
  static std::mutex m;
  return m;
}

inline bool enabled() {
  return iap::get_log_config().profiling.pipeline;
}

inline std::string path() {
  return iap::LogPaths::instance().profiling_path(iap::get_log_config().profiling.pipeline_file).string();
}

inline void ensure_header() {
  static std::once_flag once;
  std::call_once(once, [] {
    if (!enabled()) return;
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

}  // namespace iap::timing_csv
