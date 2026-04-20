#pragma once
// IAP-RQ-510: Metrics collection and reporting

#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <spdlog/spdlog.h>
#include <iostream>

namespace iap {
namespace experiments {

/// @brief Per-frame metric sample (IAP-RQ-510)
struct MetricSample {
  double stamp          = 0.0;   ///< frame timestamp [s]
  double PL             = 0.0;   ///< protection level [m]
  double AL             = 0.0;   ///< alert limit [m]
  double IM             = 0.0;   ///< integrity margin = AL - PL [m]
  bool   violation      = false; ///< PL > AL
  double path_increment = 0.0;   ///< distance since last frame [m]
  double control_effort = 0.0;   ///< ‖u‖ (e.g. ‖Δvel‖) at this step
  int    mode           = 0;     ///< 0=NOMINAL,1=CAUTION,2=ALERT,3=SEARCH
};

/**
 * @brief Accumulates per-frame metric samples and computes aggregate stats.
 *
 * ### Metrics (IAP-RQ-510)
 *  - Time(PL>AL) %
 *  - Avg PL / Min IM
 *  - Total path length [m]
 *  - Total mission time [s]
 *  - Total control effort (Σ‖u‖)
 *  - Mission success (last frame IM > 0)
 */
class MetricsCollector {
 public:
  void reset() { samples_.clear(); mission_success_ = false; }

  void add(const MetricSample& s) { samples_.push_back(s); }

  void set_mission_success(bool ok) { mission_success_ = ok; }

  // ---- aggregates -------------------------------------------------------

  double time_violation_frac() const {
    if (samples_.empty()) return 0.0;
    const double n_viol = static_cast<double>(
        std::count_if(samples_.begin(), samples_.end(),
                      [](const MetricSample& s){ return s.violation; }));
    return n_viol / static_cast<double>(samples_.size());
  }

  double avg_PL() const {
    if (samples_.empty()) return 0.0;
    double s = 0.0;
    for (const auto& m : samples_) s += m.PL;
    return s / static_cast<double>(samples_.size());
  }

  double min_IM() const {
    if (samples_.empty()) return 0.0;
    double mn = std::numeric_limits<double>::infinity();
    for (const auto& m : samples_) mn = std::min(mn, m.IM);
    return mn;
  }

  double total_path_length() const {
    double s = 0.0;
    for (const auto& m : samples_) s += m.path_increment;
    return s;
  }

  double total_mission_time() const {
    if (samples_.empty()) return 0.0;
    return samples_.back().stamp - samples_.front().stamp;
  }

  double total_control_effort() const {
    double s = 0.0;
    for (const auto& m : samples_) s += m.control_effort;
    return s;
  }

  bool mission_success() const { return mission_success_; }

  const std::vector<MetricSample>& samples() const { return samples_; }

  /// Print summary to spdlog (info level)
  void log_summary(const std::string& label) const {
    spdlog::info(
        "[Metrics][{}] n={} viol={:.1f}% avgPL={:.3f}m minIM={:.3f}m "
        "pathLen={:.2f}m time={:.1f}s effort={:.3f} success={}",
        label, samples_.size(),
        time_violation_frac() * 100.0,
        avg_PL(), min_IM(),
        total_path_length(), total_mission_time(),
        total_control_effort(),
        mission_success_ ? "YES" : "NO");
  }

  /// Write per-frame CSV (stamp, PL, AL, IM, violation, path_incr, effort, mode)
  bool write_csv(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
      spdlog::error("[Metrics] Cannot open {} for writing.", path);
      return false;
    }
    f << "stamp,PL,AL,IM,violation,path_increment,control_effort,mode\n";
    for (const auto& m : samples_) {
      f << m.stamp << ","
        << m.PL << ","
        << m.AL << ","
        << m.IM << ","
        << (m.violation ? 1 : 0) << ","
        << m.path_increment << ","
        << m.control_effort << ","
        << m.mode << "\n";
    }
    spdlog::info("[Metrics] Wrote {} rows to {}", samples_.size(), path);
    return true;
  }

 private:
  std::vector<MetricSample> samples_;
  bool mission_success_ = false;
};

/**
 * @brief Summary row for cross-experiment comparison table (IAP-RQ-510).
 */
struct ExperimentResult {
  std::string label;
  double violation_frac   = 0.0;
  double avg_PL           = 0.0;
  double min_IM           = 0.0;
  double path_length      = 0.0;
  double mission_time     = 0.0;
  double control_effort   = 0.0;
  bool   mission_success  = false;
};

/// Write a Markdown comparison table to a file (or stdout if path is "")
inline bool write_comparison_table(
    const std::vector<ExperimentResult>& results,
    const std::string& path) {
  std::ostream* out_ptr = &std::cout;
  std::ofstream fout;
  if (!path.empty()) {
    fout.open(path);
    if (!fout.is_open()) {
      spdlog::error("[Metrics] Cannot open table file {}", path);
      return false;
    }
    out_ptr = &fout;
  }
  std::ostream& out = *out_ptr;

  out << "| Baseline | Viol%  | AvgPL(m) | MinIM(m) "
      << "| PathLen(m) | Time(s) | Effort | Success |\n";
  out << "|----------|--------|----------|----------"
      << "|------------|---------|--------|----------|\n";
  for (const auto& r : results) {
    out << "| " << r.label
        << " | " << std::to_string(r.violation_frac * 100.0).substr(0, 5)
        << " | " << std::to_string(r.avg_PL).substr(0, 6)
        << " | " << std::to_string(r.min_IM).substr(0, 7)
        << " | " << std::to_string(r.path_length).substr(0, 7)
        << " | " << std::to_string(r.mission_time).substr(0, 6)
        << " | " << std::to_string(r.control_effort).substr(0, 6)
        << " | " << (r.mission_success ? "YES" : "NO") << " |\n";
  }
  if (!path.empty()) {
    spdlog::info("[Metrics] Wrote comparison table to {}", path);
  }
  return true;
}

}  // namespace experiments
}  // namespace iap
