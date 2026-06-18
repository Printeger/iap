// IAP-RQ-500: Experiment runner (Passive / Covariance-min / Integrity-aware)
// IAP-RQ-510: Metrics collection and reporting
//
// Usage:
//   ros2 run iap iap_experiment --ros-args -p scenario:=forest_01
//
// Produces:
//   /tmp/iap_experiment_<scenario>_<baseline>.csv
//   /tmp/iap_experiment_<scenario>_summary.md
//
// NOTE: This is a stub runner. Actual sensor replay / ROS2 bag integration
//       is deferred to the integration phase. The runner demonstrates the
//       full metric pipeline and can be exercised without real data using
//       synthetic trajectories.

#include <iap/experiments/metrics.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/integrity_monitor.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <spdlog/spdlog.h>

#include <iap/util/config.hpp>
#include <iap/util/logging.hpp>
#include <iap/util/run_log_manager.hpp>

namespace iap {
namespace experiments {

// ---------------------------------------------------------------------------
/// @brief Synthetic scenario: straight-line flight through a "degraded zone"
///        where PL rises above AL between t=5s and t=10s.
///
/// Returns per-frame metric samples for the given baseline strategy.
///
/// @param baseline  0=Passive, 1=Covariance-min (min sigma path), 2=Integrity-aware
std::vector<MetricSample> run_synthetic_scenario(int baseline) {
  std::vector<MetricSample> frames;

  // Scenario parameters
  const double dt        = 0.2;   // [s]
  const double t_end     = 20.0;  // [s]
  const double AL_base   = 2.0;   // [m]
  const double sigma_grow = 0.04; // [m/sqrt(s)] — nominal

  // "Degraded zone" PL inflator: between t=5 and t=10, PL grows faster
  // unless the planner routes around it (int-aware baseline avoids it)
  const double t_degrade_start = 5.0;
  const double t_degrade_end   = 10.0;
  const double degrade_factor  = 3.5;  // sigma_grow multiplier in the zone

  double sigma = 0.3;  // initial [m]
  double prev_pos = 0.0;
  const double fwd_speed_nominal = 1.0;   // m/s
  const double fwd_speed_detour  = 0.75;  // m/s (cov-min / int-aware takes detour)

  IntegrityMonitor::Params im_p;
  bool mission_success = true;

  for (double t = 0.0; t <= t_end; t += dt) {
    const bool in_zone = (t >= t_degrade_start && t <= t_degrade_end);

    // --- Decide behaviour per baseline ---
    // baseline 0 (Passive): always fly through
    // baseline 1 (Cov-min): slower in zone to reduce sigma growth path-length
    // baseline 2 (Int-aware): routes around zone (avoids it entirely)
    bool avoid_zone = false;
    double speed = fwd_speed_nominal;
    if (baseline == 1 && in_zone) {
      speed = fwd_speed_detour;
    } else if (baseline == 2 && in_zone) {
      avoid_zone = true;  // int-aware avoids degraded zone
      speed = fwd_speed_detour;
    }

    // --- Sigma update ---
    double sg = sigma_grow;
    if (in_zone && !avoid_zone) {
      sg = sigma_grow * degrade_factor;
    }
    sigma = std::sqrt(sigma * sigma + sg * sg * dt);

    // --- PL / AL ---
    const double K_pl = 3.0;
    const double PL   = K_pl * sigma;
    const double AL   = AL_base;
    const double IM   = AL - PL;

    // --- Metrics ---
    MetricSample s;
    s.stamp          = t;
    s.PL             = PL;
    s.AL             = AL;
    s.IM             = IM;
    s.violation      = (PL > AL);
    s.path_increment = speed * dt;
    s.control_effort = std::abs(speed - fwd_speed_nominal) * dt;
    s.mode           = (IM < 0) ? 2 : (IM < 0.3 * AL ? 1 : 0);  // simple proxy
    frames.push_back(s);

    if (s.violation) mission_success = false;
  }

  // Last frame mode: if we recovered, success
  if (!frames.empty() && frames.back().IM > 0) mission_success = true;

  return frames;
}

// ---------------------------------------------------------------------------
ExperimentResult run_baseline(int baseline, const std::string& label,
                               const std::string& scenario) {
  spdlog::info("[Experiment] Running baseline='{}' scenario='{}'", label, scenario);

  const auto frames = run_synthetic_scenario(baseline);

  MetricsCollector mc;
  for (const auto& f : frames) mc.add(f);

  // Determine success: no violation in last 2s (5 % tail)
  bool success = true;
  const std::size_t tail = static_cast<std::size_t>(2.0 / 0.2);
  const std::size_t start = frames.size() > tail ? frames.size() - tail : 0;
  for (std::size_t i = start; i < frames.size(); ++i) {
    if (frames[i].violation) { success = false; break; }
  }
  mc.set_mission_success(success);
  mc.log_summary(label);

  std::string csv_path =
      "/tmp/iap_experiment_" + scenario + "_" + label + ".csv";
  if (const auto* run_logs = glim::RunLogManager::get_if_initialized()) {
    csv_path = run_logs->export_path(
      "iap_experiment_" + scenario + "_" + label + ".csv").string();
  }
  mc.write_csv(csv_path);

  ExperimentResult r;
  r.label           = label;
  r.violation_frac  = mc.time_violation_frac();
  r.avg_PL          = mc.avg_PL();
  r.min_IM          = mc.min_IM();
  r.path_length     = mc.total_path_length();
  r.mission_time    = mc.total_mission_time();
  r.control_effort  = mc.total_control_effort();
  r.mission_success = mc.mission_success();
  return r;
}

}  // namespace experiments
}  // namespace iap

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  std::string scenario = "synthetic_forest_01";
  std::string config_dir = "config";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.rfind("--scenario=", 0) == 0) {
      scenario = arg.substr(11);
    } else if (arg.rfind("--config_dir=", 0) == 0) {
      config_dir = arg.substr(13);
    }
  }

  glim::GlobalConfig::instance(config_dir, true);
  auto& run_logs = glim::RunLogManager::initialize("iap_experiment", config_dir);
  run_logs.write_run_info();
  glim::GlobalConfig::instance()->dump(run_logs.metadata_path("config").string());

  auto main_logger = glim::create_module_logger("glim");
  glim::set_default_logger(main_logger);
  spdlog::set_level(spdlog::level::info);

  spdlog::info("[iap_experiment] scenario='{}'", scenario);
  spdlog::info("[iap_experiment] run_dir='{}'", run_logs.run_dir().string());

  std::vector<iap::experiments::ExperimentResult> results;

  // Baseline 0: Passive (no active decisions)
  results.push_back(iap::experiments::run_baseline(0, "Passive", scenario));

  // Baseline 1: Covariance-min (reduce sigma growth)
  results.push_back(iap::experiments::run_baseline(1, "CovMin", scenario));

  // Baseline 2: Integrity-aware (this work)
  results.push_back(iap::experiments::run_baseline(2, "IntegAware", scenario));

  // Write comparison table
  std::string table_path = "/tmp/iap_experiment_" + scenario + "_summary.md";
  if (const auto* initialized_run_logs = glim::RunLogManager::get_if_initialized()) {
    table_path = initialized_run_logs->export_path(
      "iap_experiment_" + scenario + "_summary.md").string();
  }
  iap::experiments::write_comparison_table(results, table_path);
  iap::experiments::write_comparison_table(results, "");  // also to stdout

  return 0;
}
