// Standalone timing helper for EGO planner — writes to a separate CSV file
// with the same format as IAP's timing_csv so ana_log.py can merge them.
// No dependency on IAP libraries.
#pragma once

#include <chrono>
#include <cstdio>
#include <mutex>
#include <string>

namespace ego_timing {

inline std::mutex& file_mutex() {
  static std::mutex m;
  return m;
}

inline std::string& path() {
  static std::string p;
  if (p.empty()) {
    const char* env = std::getenv("EGO_TIMING_CSV_PATH");
    p = env ? std::string(env) : "/tmp/ego_planner_timing.csv";
  }
  return p;
}

inline void set_path(const std::string& p) { path() = p; }

inline void ensure_header() {
  static std::once_flag once;
  std::call_once(once, [] {
    FILE* f = std::fopen(path().c_str(), "w");
    if (!f) return;
    std::fprintf(f, "stamp,module,elapsed_ms\n");
    std::fclose(f);
  });
}

inline void append(double stamp, const char* module, double elapsed_ms) {
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
      : stamp_(stamp), module_(module) {
    start_ = Clock::now();
  }

  ~ScopedTimer() {
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - start_).count();
    append(stamp_, module_, elapsed_ms);
  }

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
  double stamp_ = 0.0;
  const char* module_ = "";
  Clock::time_point start_{};
};

}  // namespace ego_timing
