#pragma once
// Stage 0 current LiDAR certified ARAIM component logging.

#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/lidar_araim.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

#include <spdlog/spdlog.h>

namespace iap {

class LidarAraimDebugCSV {
 public:
  LidarAraimDebugCSV() = default;
  LidarAraimDebugCSV(bool enabled, const std::string& path) {
    if (enabled) {
      enabled_ = true;
      open_file(path);
    }
  }

  void write(const IntegrityReport& report, const LidarAraimResult& result) {
    if (!enabled_ || !result.valid) return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (!header_written_) write_header();

    write_axis(report, result, "E");
    write_axis(report, result, "N");
    write_axis(report, result, "U");
    file_.flush();
  }

  bool enabled() const { return enabled_; }

 private:
  void open_file(const std::string& path) {
    path_ = path;
    file_.open(path_, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      spdlog::warn("[lidar_araim_debug] Failed to open CSV: {}", path_);
      enabled_ = false;
    } else {
      spdlog::info("[lidar_araim_debug] CSV logging to: {}", path_);
    }
  }

  void write_header() {
    // Keep legacy CSV column names for compatibility. hpl_lidar/vpl_lidar are
    // semantically lidar_certified_hpl/lidar_certified_vpl.
    file_ << "time_s,axis,hpl_lidar,vpl_lidar,"
          << "worst_hypothesis_type,worst_hypothesis_id,"
          << "sep_term_m,sigma_ss_term_m,sigma_subset_term_m,bias_term_m,"
          << "gamma_rmse,gamma_inlier,gamma_condition,gamma_age,"
          << "selected_target_count,target_window_size,"
          << "lambda_min_subset,condition_number_subset,"
          << "sigma_ss_raw_m2,sigma_ss_used_m,ss_variance_fallback_flag,"
          << "pl_e,pl_n,pl_u,"
          << "k_fa,k_md,"
          << "sigma_k_e,sigma_k_n,sigma_k_u,"
          << "t_e,t_n,t_u,"
          << "d_e,d_n,d_u,"
          << "bias_h,bias_v,gamma_total,"
          << "condition_number_full,"
          << "lambda_min_full,lambda_max_full,"
          << "lambda_max_subset,"
          << "sigma_ss_raw_e_m2,sigma_ss_raw_n_m2,sigma_ss_raw_u_m2,"
          << "sigma_ss_used_e_m,sigma_ss_used_n_m,sigma_ss_used_u_m,"
          << "ss_fallback_e,ss_fallback_n,ss_fallback_u\n";
    header_written_ = true;
  }

  static double axis_pl(const LidarSubsetSolution& ss,
                        const std::string& axis) {
    if (axis == "E") return ss.PL_E;
    if (axis == "N") return ss.PL_N;
    return ss.PL_U;
  }

  static int hypothesis_id(const LidarHypothesis& hyp) {
    if (hyp.type == LidarHypothesis::Type::H_TARGET) {
      return static_cast<int>(hyp.target_frame_id);
    }
    if (hyp.type == LidarHypothesis::Type::H_LEVEL) {
      return hyp.level_id;
    }
    return -1;
  }

  void write_axis(const IntegrityReport& report,
                  const LidarAraimResult& result,
                  const std::string& axis) {
    int best = -1;
    double best_pl = -1.0;
    for (int i = 0; i < static_cast<int>(result.subsets.size()); ++i) {
      const auto& ss = result.subsets[static_cast<std::size_t>(i)];
      if (!ss.valid) continue;
      const double pl = axis_pl(ss, axis);
      if (std::isfinite(pl) && pl > best_pl) {
        best = i;
        best_pl = pl;
      }
    }
    if (best < 0 || best >= static_cast<int>(result.hypotheses.size())) {
      return;
    }

    const auto& ss = result.subsets[static_cast<std::size_t>(best)];
    const auto& hyp = result.hypotheses[static_cast<std::size_t>(best)];

    double sep = 0.0;
    double sigma_ss_term = 0.0;
    double sigma_subset_term = 0.0;
    double bias = 0.0;
    double sigma_ss_raw_m2 = 0.0;
    double sigma_ss_used_m = 0.0;
    bool sigma_ss_fallback = false;
    if (axis == "E") {
      sep = std::abs(ss.d_E);
      sigma_ss_term = ss.T_E;
      sigma_subset_term = ss.K_md * ss.sigma_k_E;
      bias = ss.bias_H;
      sigma_ss_raw_m2 = ss.sigma_ss_raw_E_m2;
      sigma_ss_used_m = ss.sigma_ss_E;
      sigma_ss_fallback = ss.sigma_ss_fallback_E;
    } else if (axis == "N") {
      sep = std::abs(ss.d_N);
      sigma_ss_term = ss.T_N;
      sigma_subset_term = ss.K_md * ss.sigma_k_N;
      bias = ss.bias_H;
      sigma_ss_raw_m2 = ss.sigma_ss_raw_N_m2;
      sigma_ss_used_m = ss.sigma_ss_N;
      sigma_ss_fallback = ss.sigma_ss_fallback_N;
    } else {
      sep = std::abs(ss.d_U);
      sigma_ss_term = ss.T_U;
      sigma_subset_term = ss.K_md * ss.sigma_k_U;
      bias = ss.bias_V;
      sigma_ss_raw_m2 = ss.sigma_ss_raw_U_m2;
      sigma_ss_used_m = ss.sigma_ss_U;
      sigma_ss_fallback = ss.sigma_ss_fallback_U;
    }

    file_ << std::scientific << std::setprecision(12)
          << report.stamp << ","
          << axis << ","
          << result.HPL << ","
          << result.VPL << ","
          << LidarAraim::to_string(hyp.type) << ","
          << hypothesis_id(hyp) << ","
          << sep << ","
          << sigma_ss_term << ","
          << sigma_subset_term << ","
          << bias << ","
          << hyp.selected_risk.gamma_rmse << ","
          << hyp.selected_risk.gamma_inlier << ","
          << hyp.selected_risk.gamma_condition << ","
          << hyp.selected_risk.gamma_age << ","
          << result.selected_target_count << ","
          << result.target_window_K << ","
          << ss.lambda_min_subset << ","
          << ss.condition_number_subset << ","
          << sigma_ss_raw_m2 << ","
          << sigma_ss_used_m << ","
          << (sigma_ss_fallback ? 1 : 0) << ","
          << ss.PL_E << ","
          << ss.PL_N << ","
          << ss.PL_U << ","
          << ss.K_fa << ","
          << ss.K_md << ","
          << ss.sigma_k_E << ","
          << ss.sigma_k_N << ","
          << ss.sigma_k_U << ","
          << ss.T_E << ","
          << ss.T_N << ","
          << ss.T_U << ","
          << ss.d_E << ","
          << ss.d_N << ","
          << ss.d_U << ","
          << ss.bias_H << ","
          << ss.bias_V << ","
          << hyp.selected_risk.gamma_total << ","
          << result.condition_number_full << ","
          << result.lambda_min_full << ","
          << result.lambda_max_full << ","
          << ss.lambda_max_subset << ","
          << ss.sigma_ss_raw_E_m2 << ","
          << ss.sigma_ss_raw_N_m2 << ","
          << ss.sigma_ss_raw_U_m2 << ","
          << ss.sigma_ss_E << ","
          << ss.sigma_ss_N << ","
          << ss.sigma_ss_U << ","
          << (ss.sigma_ss_fallback_E ? 1 : 0) << ","
          << (ss.sigma_ss_fallback_N ? 1 : 0) << ","
          << (ss.sigma_ss_fallback_U ? 1 : 0) << "\n";
  }

  bool enabled_ = false;
  bool header_written_ = false;
  std::string path_;
  std::ofstream file_;
  std::mutex mutex_;
};

}  // namespace iap
