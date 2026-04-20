#pragma once

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
  static bool initialized = false;
  static bool value = true;
  if (!initialized) {
    value = glim::GlobalConfig::instance()->param<bool>("global", "enable_timing_csv", true);
    initialized = true;
  }
  return value;
}

inline const std::string& path() {
  static bool initialized = false;
  static std::string value = "/home/dev/code/ws_iap/src/iap/log/res/iap_timing.csv";
  if (!initialized) {
    value = glim::GlobalConfig::instance()->param<std::string>(
        "global", "timing_csv_path", "/home/dev/code/ws_iap/src/iap/log/res/iap_timing.csv");
    initialized = true;
  }
  return value;
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
