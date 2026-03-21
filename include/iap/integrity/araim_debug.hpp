#pragma once
// IAP-RQ-1100: ARAIM Debug Infrastructure (§11)
//
// Provides:
//   1. Per-epoch CSV logging of ARAIM results (controlled by IAP_ARAIM_DEBUG_CSV=1)
//   2. Formatted spdlog output for key integrity metrics
//   3. Helper to populate RViz2 marker arrays for spatial visualization

#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/araim_types.hpp>
#include <spdlog/spdlog.h>

#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>

namespace iap {

// ---------------------------------------------------------------------------
/// @brief Per-epoch CSV writer for ARAIM debug data.
///
/// File format (one row per epoch):
///   stamp, state, HPL, VPL, HAL, VAL, IM, PL_E, PL_N, PL_U,
///   pl_ff, K_ff, n_sv, n_const, PDOP, sigma_H, n_hyp, n_det,
///   n_trunks, tdop, worst_hyp
///
/// Enabled by environment variable IAP_ARAIM_DEBUG_CSV=1
/// Output path from IAP_ARAIM_DEBUG_CSV_PATH or default /tmp/iap_araim_debug.csv
class AraimDebugCSV {
 public:
  /// Env-var-controlled constructor (IAP_ARAIM_DEBUG_CSV=1).
  AraimDebugCSV();

  /// Config-controlled constructor (IAP-RQ-200 observability interface).
  AraimDebugCSV(bool enabled, const std::string& path);

  /// Write one epoch row for an integrity report.
  void write(const IntegrityReport& report);

  /// Write epoch row + optional worst_hyp row from ARAIM result (IAP-RQ-200).
  void write(const IntegrityReport& report, const AraimResult& ar);

  bool enabled() const { return enabled_; }

 private:
  void open_file(const std::string& path);
  void write_header();
  void write_epoch_row(const IntegrityReport& report);
  void write_worst_hyp_row(const IntegrityReport& report, const SubsetSolution& ss,
                            int sat_id, int hyp_index);

  bool enabled_           = false;
  std::string path_;
  std::ofstream file_;
  bool header_written_    = false;
  std::mutex mutex_;
};

// ---------------------------------------------------------------------------
/// @brief Formatted integrity log output at configurable verbosity.
class AraimDebugLogger {
 public:
  enum class Level { SILENT, SUMMARY, DETAILED, FULL };

  explicit AraimDebugLogger(Level level = Level::SUMMARY);

  /// Log an integrity report at the configured level.
  void log(const IntegrityReport& report) const;

  /// Log per-hypothesis ARAIM details (only at FULL level).
  void log_hypotheses(const AraimResult& result) const;

 private:
  Level level_;
  std::shared_ptr<spdlog::logger> logger_;
};

// ---------------------------------------------------------------------------
// Inline implementations
// ---------------------------------------------------------------------------

inline void AraimDebugCSV::open_file(const std::string& path) {
  path_ = path;
  file_.open(path_, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    spdlog::warn("[araim_debug] Failed to open CSV: {}", path_);
    enabled_ = false;
  } else {
    spdlog::info("[araim_debug] CSV logging to: {}", path_);
  }
}

inline AraimDebugCSV::AraimDebugCSV() {
  const char* env = std::getenv("IAP_ARAIM_DEBUG_CSV");
  if (env && std::string(env) == "1") {
    enabled_ = true;
    const char* path_env = std::getenv("IAP_ARAIM_DEBUG_CSV_PATH");
    open_file(path_env ? path_env : "/tmp/iap_araim_debug.csv");
  }
}

inline AraimDebugCSV::AraimDebugCSV(bool enabled, const std::string& path) {
  if (enabled) {
    enabled_ = true;
    open_file(path);
  }
}

inline void AraimDebugCSV::write_header() {
  file_ << "row_type,stamp,state,HPL,VPL,HAL,VAL,IM,"
        << "PL_E,PL_N,PL_U,pl_ff,K_ff,K_fa,"
        << "n_sv,n_const,PDOP,sigma_H,"
        << "n_hyp,n_det,n_trunks,tdop,"
        << "hyp_index,sat_id,d_E,d_N,d_U,"
        << "sigma_ss_E,sigma_ss_N,sigma_ss_U,"
        << "sigma_k_E,sigma_k_N,sigma_k_U,"
        << "hyp_PL_E,hyp_PL_N,hyp_PL_U,fault_detected\n";
  header_written_ = true;
}

inline void AraimDebugCSV::write_epoch_row(const IntegrityReport& report) {
  file_ << "epoch,"
        << std::fixed << std::setprecision(6)
        << report.stamp << ","
        << to_string(report.state) << ","
        << report.HPL << "," << report.VPL << ","
        << report.HAL << "," << report.VAL << ","
        << report.IM << ","
        << report.PL_E << "," << report.PL_N << "," << report.PL_U << ","
        << report.pl_ff << "," << report.K_ff_used << "," << report.K_fa_used << ","
        << report.n_sv_used << "," << report.n_constellations << ","
        << report.PDOP << "," << report.sigma_H << ","
        << report.araim_n_hyp << "," << report.araim_n_det << ","
        << report.n_trunks_observed << "," << report.tdop << ","
        << ",,,,,,,,,,,,\n";  // hyp columns empty for epoch row
}

inline void AraimDebugCSV::write_worst_hyp_row(const IntegrityReport& report,
                                                const SubsetSolution& ss,
                                                int sat_id, int hyp_index) {
  file_ << "worst_hyp,"
        << std::fixed << std::setprecision(6)
        << report.stamp << ","
        << to_string(report.state) << ","
        << ",,,,,"                        // HPL VPL HAL VAL IM
        << ",,,"                          // PL_E PL_N PL_U
        << ",,,,"                         // pl_ff K_ff K_fa
        << ",,,,,"                        // n_sv n_const PDOP sigma_H n_hyp
        << ",,,,"                         // n_det n_trunks tdop  (then hyp cols)
        << hyp_index << "," << sat_id << ","
        << ss.d_E << "," << ss.d_N << "," << ss.d_U << ","
        << ss.sigma_ss_E << "," << ss.sigma_ss_N << "," << ss.sigma_ss_U << ","
        << ss.sigma_k_E  << "," << ss.sigma_k_N  << "," << ss.sigma_k_U  << ","
        << ss.PL_E << "," << ss.PL_N << "," << ss.PL_U << ","
        << (ss.fault_detected ? 1 : 0) << "\n";
}

inline void AraimDebugCSV::write(const IntegrityReport& report) {
  if (!enabled_) return;
  std::lock_guard<std::mutex> lk(mutex_);
  if (!header_written_) write_header();
  write_epoch_row(report);
  file_.flush();
}

inline void AraimDebugCSV::write(const IntegrityReport& report, const AraimResult& ar) {
  if (!enabled_) return;
  std::lock_guard<std::mutex> lk(mutex_);
  if (!header_written_) write_header();
  write_epoch_row(report);
  if (ar.valid && ar.worst_hyp >= 0 && ar.worst_hyp < static_cast<int>(ar.subsets.size())) {
    const auto& ss = ar.subsets[ar.worst_hyp];
    int sat_id = -1;
    if (ar.worst_hyp < static_cast<int>(ar.hypotheses.size())) {
      sat_id = ar.hypotheses[ar.worst_hyp].sat_id;
    }
    write_worst_hyp_row(report, ss, sat_id, ar.worst_hyp);
  }
  file_.flush();
}

inline AraimDebugLogger::AraimDebugLogger(Level level) : level_(level) {
  logger_ = spdlog::get("araim_debug");
  if (!logger_) {
    logger_ = spdlog::default_logger();
  }
}

inline void AraimDebugLogger::log(const IntegrityReport& report) const {
  if (level_ == Level::SILENT) return;

  // SUMMARY: one-line key metrics
  logger_->info("[ARAIM] state={} HPL={:.3f} VPL={:.3f} HAL={:.3f} VAL={:.3f} "
                "IM={:.3f} n_sv={} n_det={}",
                to_string(report.state), report.HPL, report.VPL,
                report.HAL, report.VAL, report.IM,
                report.n_sv_used, report.araim_n_det);

  if (level_ < Level::DETAILED) return;

  // DETAILED: per-axis and quality
  logger_->info("[ARAIM]   PL_E={:.3f} PL_N={:.3f} PL_U={:.3f} pl_ff={:.3f} "
                "K_ff={:.3f} PDOP={:.2f} σ_H={:.3f}",
                report.PL_E, report.PL_N, report.PL_U,
                report.pl_ff, report.K_ff_used, report.PDOP, report.sigma_H);
}

inline void AraimDebugLogger::log_hypotheses(const AraimResult& result) const {
  if (level_ < Level::FULL || !result.valid) return;

  for (std::size_t i = 0; i < result.subsets.size(); ++i) {
    const auto& ss = result.subsets[i];
    const auto& hyp = result.hypotheses[i];
    logger_->debug("[ARAIM] hyp[{}] type={} row={} d=({:.3f},{:.3f},{:.3f}) "
                   "PL=({:.3f},{:.3f},{:.3f}) K_fa={:.2f} K_md={:.2f} det={}",
                   i,
                   hyp.type == FaultHypothesis::Type::GNSS_SAT ? "SAT" :
                   hyp.type == FaultHypothesis::Type::TRUNK ? "TRUNK" : "CONST",
                   hyp.row,
                   ss.d_E, ss.d_N, ss.d_U,
                   ss.PL_E, ss.PL_N, ss.PL_U,
                   ss.K_fa, ss.K_md,
                   ss.fault_detected ? "YES" : "no");
  }
}

}  // namespace iap
