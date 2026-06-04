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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
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
  void write(const IntegrityReport& report, const GnssAraimResult& ar);

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
/// @brief Dominant H/V GNSS ARAIM PL decomposition CSV.
///
/// This debug-only CSV writes two rows per valid GNSS ARAIM epoch: the dominant
/// horizontal contributor and the dominant vertical contributor. It is intended
/// to explain whether PL is dominated by separation, FA threshold, MD term, or
/// the fault-free geometry term.
class AraimPLDecompCSV {
 public:
  AraimPLDecompCSV() = default;
  AraimPLDecompCSV(bool enabled, const std::string& path);

  void write(const IntegrityReport& report, const GnssAraimResult& ar);

  bool enabled() const { return enabled_; }

 private:
  struct Row {
    std::string axis;
    int hyp_index = -1;
    std::string hyp_type = "UNKNOWN";
    int hyp_id = -1;
    int row_removed = -1;
    int sat_id = -1;
    int const_id = -1;
    double hyp_PL_axis = 0.0;
    double hyp_PL_E = 0.0;
    double hyp_PL_N = 0.0;
    double hyp_PL_U = 0.0;
    double abs_d_axis = 0.0;
    double d_E = 0.0;
    double d_N = 0.0;
    double d_U = 0.0;
    double sigma_ss_axis = 0.0;
    double sigma_ss_E = 0.0;
    double sigma_ss_N = 0.0;
    double sigma_ss_U = 0.0;
    double sigma_k_axis = 0.0;
    double sigma_k_E = 0.0;
    double sigma_k_N = 0.0;
    double sigma_k_U = 0.0;
    double term_d = 0.0;
    double term_fa = 0.0;
    double term_md = 0.0;
    double term_bias = 0.0;
    double total_reconstructed = 0.0;
    double K_md = 0.0;
    int dominant_const_id = -1;
    std::string dominant_const_name = "UNKNOWN";
    int n_used_total = 0;
    int n_removed_by_hyp = 0;
    int n_remaining_after_hyp = 0;
    double HDOP_full = 0.0;
    double VDOP_full = 0.0;
    double PDOP_full = 0.0;
    double HDOP_subset = 0.0;
    double VDOP_subset = 0.0;
    double PDOP_subset = 0.0;
    double sigma_H_full = 0.0;
    double sigma_V_full = 0.0;
    double sigma_H_subset = 0.0;
    double sigma_V_subset = 0.0;
    std::vector<int> removed_prn_list;
    std::vector<int> remaining_prn_list;
    bool fault_detected = false;
    bool degenerate = false;
    std::string failure_reason;
  };

  void open_file(const std::string& path);
  void write_header();
  void write_row(const IntegrityReport& report,
                 const GnssAraimResult& ar,
                 uint64_t frame_seq,
                 const Row& row);
  Row make_fault_free_row(const GnssAraimResult& ar, const std::string& axis) const;
  Row make_subset_row(const GnssAraimResult& ar,
                      int hyp_index,
                      const std::string& axis) const;
  Row dominant_horizontal(const GnssAraimResult& ar) const;
  Row dominant_vertical(const GnssAraimResult& ar) const;
  static std::string unsafe_reason(const IntegrityReport& report);
  static std::string hypothesis_type(const FaultHypothesis& hyp);
  static int hypothesis_id(const FaultHypothesis& hyp);
  static std::string constellation_name(int const_id);
  static std::string int_list_csv(const std::vector<int>& values);

  bool enabled_ = false;
  std::string path_;
  std::ofstream file_;
  bool header_written_ = false;
  uint64_t frame_seq_ = 0;
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
  void log_hypotheses(const GnssAraimResult& result) const;

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
        << "lidar_valid,lidar_n_hyp,lidar_n_det,lidar_HPL,lidar_mode,"
        << "hyp_index,sat_id,d_E,d_N,d_U,"
        << "sigma_ss_E,sigma_ss_N,sigma_ss_U,"
        << "sigma_k_E,sigma_k_N,sigma_k_U,"
        << "hyp_PL_E,hyp_PL_N,hyp_PL_U,fault_detected,"
        << "final_HPL_source,final_VPL_source,final_PL_source,"
        << "gnss_valid,gnss_HPL,gnss_VPL,gnss_PL_E,gnss_PL_N,gnss_PL_U,"
        << "gnss_pl_ff,gnss_K_ff,gnss_K_fa,gnss_n_hyp,gnss_n_det,"
        << "lidar_VPL,lidar_PL_E,lidar_PL_N,lidar_PL_U\n";
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
        << report.lidar_valid << "," << report.lidar_n_hyp << ","
        << report.lidar_n_det << "," << report.lidar_HPL << ","
        << report.lidar_worst_mode << ","
        << std::string(15, ',')
        << report.final_HPL_source << ","
        << report.final_VPL_source << ","
        << report.final_PL_source << ","
        << report.gnss_valid << ","
        << report.gnss_HPL << "," << report.gnss_VPL << ","
        << report.gnss_PL_E << "," << report.gnss_PL_N << "," << report.gnss_PL_U << ","
        << report.gnss_pl_ff << "," << report.gnss_K_ff_used << "," << report.gnss_K_fa_used << ","
        << report.gnss_n_hyp << "," << report.gnss_n_det << ","
        << report.lidar_VPL << ","
        << report.lidar_PL_E << "," << report.lidar_PL_N << "," << report.lidar_PL_U << "\n";
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
        << ",,,,,"                        // n_det n_trunks tdop lidar_valid
        << ",,,,"                         // lidar_n_hyp lidar_n_det lidar_HPL lidar_mode
        << hyp_index << "," << sat_id << ","
        << ss.d_E << "," << ss.d_N << "," << ss.d_U << ","
        << ss.sigma_ss_E << "," << ss.sigma_ss_N << "," << ss.sigma_ss_U << ","
        << ss.sigma_k_E  << "," << ss.sigma_k_N  << "," << ss.sigma_k_U  << ","
        << ss.PL_E << "," << ss.PL_N << "," << ss.PL_U << ","
        << (ss.fault_detected ? 1 : 0)
        << std::string(18, ',') << "\n";
}

inline void AraimDebugCSV::write(const IntegrityReport& report) {
  if (!enabled_) return;
  std::lock_guard<std::mutex> lk(mutex_);
  if (!header_written_) write_header();
  write_epoch_row(report);
  file_.flush();
}

inline void AraimDebugCSV::write(const IntegrityReport& report, const GnssAraimResult& ar) {
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

inline AraimPLDecompCSV::AraimPLDecompCSV(bool enabled, const std::string& path) {
  if (enabled) {
    enabled_ = true;
    open_file(path);
  }
}

inline void AraimPLDecompCSV::open_file(const std::string& path) {
  path_ = path;
  file_.open(path_, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    spdlog::warn("[araim_pl_decomp] Failed to open CSV: {}", path_);
    enabled_ = false;
  } else {
    spdlog::info("[araim_pl_decomp] CSV logging to: {}", path_);
  }
}

inline void AraimPLDecompCSV::write_header() {
  file_ << "stamp,frame_seq,axis,"
        << "HPL,VPL,PL_E,PL_N,PL_U,HAL,VAL,im_h,im_v,im_min,unsafe_reason,"
        << "final_hpl_source,final_vpl_source,gnss_valid,"
        << "n_used,n_hyp,n_detected,n_constellations,"
        << "pdop,sigma_H_0,sigma_E_0,sigma_N_0,sigma_V_0,"
        << "K_ff,K_fa,K_md,"
        << "hyp_index,hyp_type,hyp_id,row_removed,sat_id,const_id,"
        << "hyp_PL_axis,hyp_PL_E,hyp_PL_N,hyp_PL_U,"
        << "abs_d_axis,d_E,d_N,d_U,"
        << "sigma_ss_axis,sigma_ss_E,sigma_ss_N,sigma_ss_U,"
        << "sigma_k_axis,sigma_k_E,sigma_k_N,sigma_k_U,"
        << "term_d,term_fa,term_md,term_bias,total_reconstructed,"
        << "fault_detected,degenerate,failure_reason,"
        << "dominant_const_id,dominant_const_name,"
        << "n_used_total,n_removed_by_hyp,n_remaining_after_hyp,"
        << "HDOP_full,VDOP_full,PDOP_full,"
        << "HDOP_subset,VDOP_subset,PDOP_subset,"
        << "sigma_H_full,sigma_V_full,sigma_H_subset,sigma_V_subset,"
        << "removed_prn_list,remaining_prn_list\n";
  header_written_ = true;
}

inline std::string AraimPLDecompCSV::unsafe_reason(const IntegrityReport& report) {
  const bool h_unsafe = report.HPL >= report.HAL;
  const bool v_unsafe = report.VPL >= report.VAL;
  if (h_unsafe && v_unsafe) return "BOTH";
  if (h_unsafe) return "HORIZONTAL";
  if (v_unsafe) return "VERTICAL";
  return "NONE";
}

inline std::string AraimPLDecompCSV::hypothesis_type(const FaultHypothesis& hyp) {
  switch (hyp.type) {
    case FaultHypothesis::Type::GNSS_SAT:
      return "GNSS_SAT";
    case FaultHypothesis::Type::TRUNK:
      return "TRUNK";
    case FaultHypothesis::Type::CONSTELLATION:
      return "CONSTELLATION";
  }
  return "UNKNOWN";
}

inline int AraimPLDecompCSV::hypothesis_id(const FaultHypothesis& hyp) {
  switch (hyp.type) {
    case FaultHypothesis::Type::GNSS_SAT:
      return hyp.sat_id;
    case FaultHypothesis::Type::TRUNK:
      return hyp.trunk_id;
    case FaultHypothesis::Type::CONSTELLATION:
      return hyp.const_id;
  }
  return -1;
}

inline std::string AraimPLDecompCSV::constellation_name(int const_id) {
  switch (const_id) {
    case 0:
      return "GPS";
    case 1:
      return "GAL";
    case 2:
      return "BDS";
    case 3:
      return "GLO";
    default:
      return "UNKNOWN";
  }
}

inline std::string AraimPLDecompCSV::int_list_csv(const std::vector<int>& values) {
  std::ostringstream oss;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) oss << ';';
    oss << values[i];
  }
  return oss.str();
}

inline AraimPLDecompCSV::Row AraimPLDecompCSV::make_fault_free_row(
    const GnssAraimResult& ar, const std::string& axis) const {
  Row row;
  row.axis = axis;
  row.hyp_type = "FAULT_FREE";
  row.n_used_total = ar.n_used_total;
  row.n_remaining_after_hyp = ar.n_used_total;
  row.HDOP_full = ar.HDOP_full;
  row.VDOP_full = ar.VDOP_full;
  row.PDOP_full = ar.PDOP_full;
  row.HDOP_subset = ar.HDOP_full;
  row.VDOP_subset = ar.VDOP_full;
  row.PDOP_subset = ar.PDOP_full;
  row.sigma_H_full = ar.sigma_H_full;
  row.sigma_V_full = ar.sigma_V_full;
  row.sigma_H_subset = ar.sigma_H_full;
  row.sigma_V_subset = ar.sigma_V_full;
  row.remaining_prn_list = ar.used_prns;
  row.hyp_PL_E = ar.pl_ff_E;
  row.hyp_PL_N = ar.pl_ff_N;
  row.hyp_PL_U = ar.pl_ff_V;
  row.sigma_k_E = ar.sigma_ff_E;
  row.sigma_k_N = ar.sigma_ff_N;
  row.sigma_k_U = ar.sigma_ff_U;

  if (axis == "V") {
    row.hyp_PL_axis = ar.pl_ff_V;
    row.sigma_k_axis = ar.sigma_ff_U;
  } else if (ar.pl_ff_E >= ar.pl_ff_N) {
    row.hyp_PL_axis = ar.pl_ff_E;
    row.sigma_k_axis = ar.sigma_ff_E;
  } else {
    row.hyp_PL_axis = ar.pl_ff_N;
    row.sigma_k_axis = ar.sigma_ff_N;
  }

  row.term_md = ar.K_ff_used * row.sigma_k_axis;
  row.total_reconstructed = row.term_d + row.term_fa + row.term_md + row.term_bias;
  return row;
}

inline AraimPLDecompCSV::Row AraimPLDecompCSV::make_subset_row(
    const GnssAraimResult& ar, int hyp_index, const std::string& axis) const {
  Row row;
  row.axis = axis;
  row.hyp_index = hyp_index;

  if (hyp_index < 0 || hyp_index >= static_cast<int>(ar.subsets.size())) {
    row.failure_reason = "invalid_hypothesis_index";
    return row;
  }

  const auto& ss = ar.subsets[static_cast<std::size_t>(hyp_index)];
  row.row_removed = ss.row_removed;
  row.hyp_PL_E = ss.PL_E;
  row.hyp_PL_N = ss.PL_N;
  row.hyp_PL_U = ss.PL_U;
  row.d_E = ss.d_E;
  row.d_N = ss.d_N;
  row.d_U = ss.d_U;
  row.sigma_ss_E = ss.sigma_ss_E;
  row.sigma_ss_N = ss.sigma_ss_N;
  row.sigma_ss_U = ss.sigma_ss_U;
  row.sigma_k_E = ss.sigma_k_E;
  row.sigma_k_N = ss.sigma_k_N;
  row.sigma_k_U = ss.sigma_k_U;
  row.K_md = ss.K_md;
  row.fault_detected = ss.fault_detected;
  row.degenerate = ss.degenerate;
  row.failure_reason = ss.failure_reason;

  if (hyp_index < static_cast<int>(ar.hypotheses.size())) {
    const auto& hyp = ar.hypotheses[static_cast<std::size_t>(hyp_index)];
    row.hyp_type = hypothesis_type(hyp);
    row.hyp_id = hypothesis_id(hyp);
    row.sat_id = hyp.sat_id;
    row.const_id = hyp.const_id;
    row.dominant_const_id = hyp.const_id;
    row.dominant_const_name = constellation_name(hyp.const_id);
  }

  row.n_used_total = ar.n_used_total;
  row.n_removed_by_hyp = ss.n_removed_by_hyp;
  row.n_remaining_after_hyp = ss.n_remaining_after_hyp;
  row.HDOP_full = ss.HDOP_full;
  row.VDOP_full = ss.VDOP_full;
  row.PDOP_full = ss.PDOP_full;
  row.HDOP_subset = ss.HDOP_subset;
  row.VDOP_subset = ss.VDOP_subset;
  row.PDOP_subset = ss.PDOP_subset;
  row.sigma_H_full = ss.sigma_H_full;
  row.sigma_V_full = ss.sigma_V_full;
  row.sigma_H_subset = ss.sigma_H_subset;
  row.sigma_V_subset = ss.sigma_V_subset;
  row.removed_prn_list = ss.removed_prn_list;
  row.remaining_prn_list = ss.remaining_prn_list;

  if (axis == "V") {
    row.hyp_PL_axis = ss.PL_U;
    row.abs_d_axis = std::abs(ss.d_U);
    row.sigma_ss_axis = ss.sigma_ss_U;
    row.sigma_k_axis = ss.sigma_k_U;
  } else if (ss.PL_E >= ss.PL_N) {
    row.hyp_PL_axis = ss.PL_E;
    row.abs_d_axis = std::abs(ss.d_E);
    row.sigma_ss_axis = ss.sigma_ss_E;
    row.sigma_k_axis = ss.sigma_k_E;
  } else {
    row.hyp_PL_axis = ss.PL_N;
    row.abs_d_axis = std::abs(ss.d_N);
    row.sigma_ss_axis = ss.sigma_ss_N;
    row.sigma_k_axis = ss.sigma_k_N;
  }

  row.term_d = row.abs_d_axis;
  row.term_fa = ss.K_fa * row.sigma_ss_axis;
  row.term_md = ss.K_md * row.sigma_k_axis;
  row.total_reconstructed = row.term_d + row.term_fa + row.term_md + row.term_bias;
  return row;
}

inline AraimPLDecompCSV::Row AraimPLDecompCSV::dominant_horizontal(
    const GnssAraimResult& ar) const {
  Row best = make_fault_free_row(ar, "H");
  for (int i = 0; i < static_cast<int>(ar.subsets.size()); ++i) {
    Row candidate = make_subset_row(ar, i, "H");
    if (candidate.hyp_PL_axis > best.hyp_PL_axis) {
      best = candidate;
    }
  }
  return best;
}

inline AraimPLDecompCSV::Row AraimPLDecompCSV::dominant_vertical(
    const GnssAraimResult& ar) const {
  Row best = make_fault_free_row(ar, "V");
  for (int i = 0; i < static_cast<int>(ar.subsets.size()); ++i) {
    Row candidate = make_subset_row(ar, i, "V");
    if (candidate.hyp_PL_axis > best.hyp_PL_axis) {
      best = candidate;
    }
  }
  return best;
}

inline void AraimPLDecompCSV::write_row(const IntegrityReport& report,
                                        const GnssAraimResult& ar,
                                        uint64_t frame_seq,
                                        const Row& row) {
  file_ << std::fixed << std::setprecision(6)
        << report.stamp << ","
        << frame_seq << ","
        << row.axis << ","
        << report.HPL << "," << report.VPL << ","
        << report.PL_E << "," << report.PL_N << "," << report.PL_U << ","
        << report.HAL << "," << report.VAL << ","
        << report.im_h << "," << report.im_v << "," << report.IM << ","
        << unsafe_reason(report) << ","
        << report.final_HPL_source << "," << report.final_VPL_source << ","
        << report.gnss_valid << ","
        << report.n_sv_used << "," << report.araim_n_hyp << ","
        << report.araim_n_det << "," << report.n_constellations << ","
        << report.PDOP << "," << report.sigma_H << ","
        << ar.sigma_ff_E << "," << ar.sigma_ff_N << "," << ar.sigma_ff_U << ","
        << ar.K_ff_used << "," << ar.K_fa_used << "," << row.K_md << ","
        << row.hyp_index << "," << row.hyp_type << "," << row.hyp_id << ","
        << row.row_removed << "," << row.sat_id << "," << row.const_id << ","
        << row.hyp_PL_axis << "," << row.hyp_PL_E << ","
        << row.hyp_PL_N << "," << row.hyp_PL_U << ","
        << row.abs_d_axis << "," << row.d_E << "," << row.d_N << "," << row.d_U << ","
        << row.sigma_ss_axis << "," << row.sigma_ss_E << ","
        << row.sigma_ss_N << "," << row.sigma_ss_U << ","
        << row.sigma_k_axis << "," << row.sigma_k_E << ","
        << row.sigma_k_N << "," << row.sigma_k_U << ","
        << row.term_d << "," << row.term_fa << "," << row.term_md << ","
        << row.term_bias << "," << row.total_reconstructed << ","
        << (row.fault_detected ? 1 : 0) << ","
        << (row.degenerate ? 1 : 0) << ","
        << row.failure_reason << ","
        << row.dominant_const_id << "," << row.dominant_const_name << ","
        << row.n_used_total << "," << row.n_removed_by_hyp << ","
        << row.n_remaining_after_hyp << ","
        << row.HDOP_full << "," << row.VDOP_full << "," << row.PDOP_full << ","
        << row.HDOP_subset << "," << row.VDOP_subset << "," << row.PDOP_subset << ","
        << row.sigma_H_full << "," << row.sigma_V_full << ","
        << row.sigma_H_subset << "," << row.sigma_V_subset << ","
        << int_list_csv(row.removed_prn_list) << ","
        << int_list_csv(row.remaining_prn_list) << "\n";
}

inline void AraimPLDecompCSV::write(const IntegrityReport& report,
                                    const GnssAraimResult& ar) {
  if (!enabled_ || !ar.valid) return;
  std::lock_guard<std::mutex> lk(mutex_);
  if (!header_written_) write_header();
  const uint64_t seq = ++frame_seq_;
  write_row(report, ar, seq, dominant_horizontal(ar));
  write_row(report, ar, seq, dominant_vertical(ar));
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
  logger_->info("[ARAIM current_monitor] state={} monitor_fused_HPL={:.3f} "
                "monitor_fused_VPL={:.3f} HAL={:.3f} VAL={:.3f} "
                "monitor_IM={:.3f} n_sv={} n_det={}",
                to_string(report.state), report.HPL, report.VPL,
                report.HAL, report.VAL, report.IM,
                report.n_sv_used, report.araim_n_det);

  if (level_ < Level::DETAILED) return;

  // DETAILED: per-axis and quality
  logger_->info("[ARAIM current_monitor]   monitor_PL_E={:.3f} "
                "monitor_PL_N={:.3f} monitor_PL_U={:.3f} pl_ff={:.3f} "
                "K_ff={:.3f} PDOP={:.2f} σ_H={:.3f}",
                report.PL_E, report.PL_N, report.PL_U,
                report.pl_ff, report.K_ff_used, report.PDOP, report.sigma_H);
}

inline void AraimDebugLogger::log_hypotheses(const GnssAraimResult& result) const {
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
