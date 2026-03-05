// IAP-RQ-200 / RQ-210 / RQ-220: Integrity monitoring implementation

#include <iap/integrity/integrity_monitor.hpp>
#include <iap/util/logging.hpp>

#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>

namespace iap {

// ---------------------------------------------------------------------------
IntegrityMonitor::IntegrityMonitor() : params_(Params{}), araim_(Araim{}) {
  logger_ = glim::create_module_logger("integrity");
}

IntegrityMonitor::IntegrityMonitor(const Params& params)
: params_(params), araim_(params.araim_params) {
  logger_ = glim::create_module_logger("integrity");
}

// ---------------------------------------------------------------------------
void IntegrityMonitor::set_obstacle_distance(double dist_m) {
  obstacle_dist_ = dist_m;
}

// ---------------------------------------------------------------------------
double IntegrityMonitor::compute_PL(const glim::EstimationFrame& frame) const {
  // IAP-RQ-200 baseline: PL = K_pl * sqrt(lambda_max(Σ_p))
  // lambda_max computed from sigma_p (3×3) — see IAP-RQ-015
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
  const double lambda_max = eig.eigenvalues().maxCoeff();
  return (lambda_max > 0.0) ? params_.K_pl * std::sqrt(lambda_max) : 1e9;
}

// ---------------------------------------------------------------------------
double IntegrityMonitor::compute_AL() const {
  // IAP-RQ-210: AL from obstacle proximity
  if (obstacle_dist_ >= 1e8) {
    // No obstacle data → use default
    return params_.default_al;
  }
  const double al_raw = params_.al_scale * obstacle_dist_ - params_.uav_radius;
  return std::max(al_raw, params_.al_min);
}

// ---------------------------------------------------------------------------
void IntegrityMonitor::run_gnss_gating(const GnssEpoch& epoch,
                                        IntegrityReport& report) const {
  // IAP-RQ-220: per-satellite NIS gating
  // NIS_k = r_k^2 / sigma_k^2   (scalar, chi-squared 1 DOF)
  const double threshold = params_.chi2_1dof_thresh;

  report.sat_nis.clear();
  report.excluded_sats.clear();
  double global_nis = 0.0;
  int    n_used     = 0;

  for (const auto& sat : epoch.sats) {
    if (sat.excluded) {
      report.excluded_sats.push_back(sat.sat_id);
      continue;
    }

    // Use the pr NIS as primary gating (doppler nis can be added similarly)
    const double nis = sat.nis_pr;
    report.sat_nis.push_back(nis);

    if (nis > threshold) {
      logger_->warn("NIS gating: sat {} NIS_pr={:.2f} > {:.2f} -- downweighted",
                    sat.sat_id, nis, threshold);
      // In a full FDE implementation the caller would mark sat.excluded = true
      // and re-run. Here we just report.
      report.excluded_sats.push_back(sat.sat_id);
    } else {
      global_nis += nis;
      ++n_used;
    }
  }

  // Global NIS test — greedy FDE (report only, actual exclusion deferred)
  const double global_thresh = params_.chi2_global_mult * n_used * params_.chi2_1dof_thresh;
  if (n_used > 0 && global_nis > global_thresh) {
    logger_->warn("Global NIS {:.2f} > threshold {:.2f} -- FDE triggered (greedy)",
                  global_nis, global_thresh);
  }

  // gamma_R: simple proxy = max(1, sqrt(max_nis / chi2_thresh))
  double max_nis = 0.0;
  for (double n : report.sat_nis) max_nis = std::max(max_nis, n);
  report.gamma_R = std::min(std::max(1.0, std::sqrt(max_nis / threshold)),
                             params_.gamma_R_max);
}

// ---------------------------------------------------------------------------
void IntegrityMonitor::run_araim(const GnssEpoch& epoch,
                                  int n_trunk_obs,
                                  IntegrityReport& report) {
  // IAP-RQ-241–246: run ARAIM and merge results into the report
  const AraimResult ar = araim_.run(epoch, n_trunk_obs);

  report.araim_valid  = ar.valid ? 1 : 0;
  report.araim_n_hyp  = ar.n_hypotheses;
  report.araim_n_det  = ar.n_detected;
  report.araim_detected_rows = ar.detected_rows;

  if (!ar.valid) return;

  report.pl_araim = ar.pl_araim;
  report.pl_ff    = ar.pl_ff;

  // Replace the proxy PL with the ARAIM PL (ARAIM is more principled)
  report.PL = ar.pl_araim;
  report.IM = report.AL - report.PL;

  if (ar.n_detected > 0) {
    logger_->warn("ARAIM FDE: {} fault(s) detected; rows: {}",
                  ar.n_detected,
                  [&] {
                    std::string s;
                    for (int r : ar.detected_rows)
                      s += std::to_string(r) + " ";
                    return s;
                  }());
  }
}

// ---------------------------------------------------------------------------
IntegrityMode IntegrityMonitor::update_mode(const IntegrityReport& report) {
  const double al = report.AL;
  const double pl = report.PL;

  IntegrityMode next = current_mode_;

  switch (current_mode_) {
    case IntegrityMode::NOMINAL:
    case IntegrityMode::CAUTION:
      if (pl >= al) {
        next = IntegrityMode::ALERT;
        recovery_counter_ = 0;
      } else if (pl > params_.caution_fraction * al) {
        next = IntegrityMode::CAUTION;
      } else {
        next = IntegrityMode::NOMINAL;
      }
      break;

    case IntegrityMode::ALERT:
      // Transition to SEARCH (actively look for better geometry)
      next = IntegrityMode::SEARCH;
      recovery_counter_ = 0;
      break;

    case IntegrityMode::SEARCH:
      if (pl < params_.nominal_fraction * al) {
        ++recovery_counter_;
        if (recovery_counter_ >= params_.recovery_count) {
          next = IntegrityMode::NOMINAL;
          recovery_counter_ = 0;
        }
      } else {
        recovery_counter_ = 0;
      }
      break;
  }

  current_mode_ = next;
  return next;
}

// ---------------------------------------------------------------------------
IntegrityReport IntegrityMonitor::compute(const glim::EstimationFrame&       frame,
                                           const GnssEpoch*             epoch,
                                           const TrunkDetectionResult*  trunk) {
  IntegrityReport report;
  report.stamp = frame.stamp;

  // --- PL (IAP-RQ-200) ---
  report.PL = compute_PL(frame);
  report.lambda_max_sigma_p = [&] {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
    return eig.eigenvalues().maxCoeff();
  }();

  // --- AL (IAP-RQ-210) ---
  report.AL = compute_AL();

  // --- IM ---
  report.IM = report.AL - report.PL;

  // --- ICP health ---
  report.icp_degenerate = frame.icp_quality.degeneracy_flag;
  report.gamma_lidar    = frame.icp_quality.gamma_lidar;

  // --- GNSS NIS gating (IAP-RQ-220) ---
  if (epoch) {
    run_gnss_gating(*epoch, report);
  }

  // --- ARAIM (IAP-RQ-241–246) ---
  if (epoch) {
    const int n_trunk_obs = trunk ? static_cast<int>(trunk->trunks.size()) : 0;
    run_araim(*epoch, n_trunk_obs, report);
  }

  // --- TDOP (IAP-RQ-120) ---
  if (trunk) {
    report.tdop = trunk->tdop;
  }

  // --- Mode state machine  ---
  report.mode = update_mode(report);

  logger_->trace(
    "integrity: PL={:.3f}m AL={:.3f}m IM={:.3f}m mode={} lambda_max={:.4f}"
    " icp_degenerate={} gamma_lidar={:.2f} tdop={:.2f}"
    " pl_araim={:.3f} araim_n_hyp={} araim_n_det={}",
    report.PL, report.AL, report.IM, to_string(report.mode),
    report.lambda_max_sigma_p, report.icp_degenerate, report.gamma_lidar,
    report.tdop, report.pl_araim, report.araim_n_hyp, report.araim_n_det);

  if (report.mode == IntegrityMode::ALERT) {
    logger_->warn("INTEGRITY ALERT: PL={:.3f} >= AL={:.3f}  IM={:.3f}",
                  report.PL, report.AL, report.IM);
  }

  return report;
}

}  // namespace iap
