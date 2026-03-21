// IAP-RQ-200 / RQ-210 / RQ-220: Integrity monitoring implementation
// §1.12: Dynamic AL (HAL + VAL)
// §1.13: Three-state integrity state machine (SAFE / SAFE_EXCLUDED / UNSAFE)

#include <iap/integrity/integrity_monitor.hpp>
#include <iap/util/timing_csv.hpp>
#include <iap/util/logging.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

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

void IntegrityMonitor::set_altitude(double h_agl) {
  current_altitude_ = h_agl;
}

void IntegrityMonitor::set_canopy_height(double h_canopy) {
  canopy_height_ = h_canopy;
}

// ---------------------------------------------------------------------------
double IntegrityMonitor::compute_PL_proxy(const glim::EstimationFrame& frame) const {
  // Fallback PL when ARAIM is unavailable: PL = K_pl * sqrt(lambda_max(Σ_p))
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
  const double lambda_max = eig.eigenvalues().maxCoeff();
  return (lambda_max > 0.0) ? params_.K_pl * std::sqrt(lambda_max) : 1e9;
}

// ---------------------------------------------------------------------------
// §1.12: Dynamic AL — HAL from trunk geometry + VAL from altitude bounds
// ---------------------------------------------------------------------------
DynamicALResult IntegrityMonitor::compute_dynamic_AL(
    const glim::EstimationFrame& frame,
    const TrunkDetectionResult* trunk) const {

  DynamicALResult al;
  al.stamp = frame.stamp;
  al.current_altitude = current_altitude_;
  al.canopy_height_est = (canopy_height_ >= 0.0) ? canopy_height_
                                                   : params_.canopy_height_default;

  // --- HAL from trunk geometry (§1.12 horizontal) ---
  // HAL = γ_H · min_k( ‖p̂_xy − c_k‖ − r_k − r_drone )
  if (trunk && !trunk->trunks.empty()) {
    const Eigen::Vector2d p_xy = frame.T_world_imu.translation().head<2>();
    double min_clearance = 1e9;
    int    nearest_id    = -1;
    double nearest_dist  = 1e9;

    for (int ti = 0; ti < static_cast<int>(trunk->trunks.size()); ++ti) {
      const auto& t = trunk->trunks[ti];
      const double dist = (p_xy - t.center_xy).norm();
      const double clearance = dist - t.radius - params_.r_drone;
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest_id = ti;  // use index; TrunkObservation has no persistent id
      }
      min_clearance = std::min(min_clearance, clearance);
    }

    al.nearest_trunk_id   = nearest_id;
    al.nearest_trunk_dist = nearest_dist;

    if (min_clearance > 0.0) {
      al.HAL = params_.gamma_H * min_clearance;
    } else {
      al.HAL = 0.0;  // Inside trunk exclusion zone — immediate alert
    }
  } else {
    al.HAL = params_.HAL_trunk_default;
    al.nearest_trunk_id   = -1;
    al.nearest_trunk_dist = 1e9;
  }

  // --- VAL from altitude bounds (§1.12 vertical) ---
  // VAL = γ_V · (h_t − h_canopy − h_min)
  if (current_altitude_ > 0.0) {
    const double margin = current_altitude_ - al.canopy_height_est - params_.h_min;
    if (margin > 0.0) {
      al.VAL = std::min(params_.gamma_V * margin, params_.VAL_max);
    } else {
      al.VAL = 0.0;  // Below minimum safe height
    }
  } else {
    al.VAL = params_.VAL_default;
  }

  // --- Combined AL ---
  if (al.HAL < al.VAL) {
    al.AL = al.HAL;
    al.al_from_trunk = true;
  } else {
    al.AL = al.VAL;
    al.al_from_trunk = false;
  }

  // Apply minimum AL clamp
  al.AL = std::max(al.AL, params_.al_min);

  return al;
}

// ---------------------------------------------------------------------------
void IntegrityMonitor::run_gnss_gating(const GnssEpoch& epoch,
                                        IntegrityReport& report) const {
  // IAP-RQ-220: per-satellite NIS gating
  const double threshold = params_.chi2_1dof_thresh;

  report.sat_nis.clear();
  report.excluded_sats.clear();
  double global_nis = 0.0;
  int    n_used     = 0;
  int    n_constellations = 0;
  bool   has_gps = false, has_gal = false, has_bds = false, has_glo = false;

  for (const auto& sat : epoch.sats) {
    if (sat.excluded) {
      report.excluded_sats.push_back(sat.sat_id);
      continue;
    }

    const double nis = sat.nis_pr;
    report.sat_nis.push_back(nis);

    if (nis > threshold) {
      logger_->warn("NIS gating: sat {} NIS_pr={:.2f} > {:.2f} -- downweighted",
                    sat.sat_id, nis, threshold);
      report.excluded_sats.push_back(sat.sat_id);
    } else {
      global_nis += nis;
      ++n_used;
      // Track active constellations
      if (sat.sat_id >= 1  && sat.sat_id <= 32)  has_gps = true;
      else if (sat.sat_id >= 33 && sat.sat_id <= 56)  has_glo = true;
      else if (sat.sat_id >= 57 && sat.sat_id <= 88)  has_gal = true;
      else if (sat.sat_id >= 89 && sat.sat_id <= 152) has_bds = true;
    }
  }

  n_constellations = (has_gps ? 1 : 0) + (has_gal ? 1 : 0) +
                     (has_bds ? 1 : 0) + (has_glo ? 1 : 0);
  report.n_sv_used = n_used;
  report.n_constellations = n_constellations;

  // Global NIS test
  const double global_thresh = params_.chi2_global_mult * n_used * params_.chi2_1dof_thresh;
  if (n_used > 0 && global_nis > global_thresh) {
    logger_->warn("Global NIS {:.2f} > threshold {:.2f} -- FDE triggered (greedy)",
                  global_nis, global_thresh);
  }

  // gamma_R proxy
  double max_nis = 0.0;
  for (double n : report.sat_nis) max_nis = std::max(max_nis, n);
  report.gamma_R = std::min(std::max(1.0, std::sqrt(max_nis / threshold)),
                             params_.gamma_R_max);
}

// ---------------------------------------------------------------------------
void IntegrityMonitor::run_araim(const GnssEpoch& epoch,
                                  int n_trunk_obs,
                                  IntegrityReport& report) {
  // IAP-RQ-241–246: run ARAIM and merge per-axis results into report
  const AraimResult ar = araim_.run(epoch, n_trunk_obs);

  report.araim_valid  = ar.valid ? 1 : 0;
  report.araim_n_hyp  = ar.n_hypotheses;
  report.araim_n_det  = ar.n_detected;
  report.araim_detected_rows = ar.detected_rows;

  if (!ar.valid) return;

  // Forward per-axis ARAIM results (§1.11)
  report.HPL       = ar.HPL;
  report.VPL       = ar.VPL;
  report.PL_E      = ar.PL_E;
  report.PL_N      = ar.PL_N;
  report.PL_U      = ar.PL_U;
  report.pl_araim  = ar.pl_araim;
  report.vpl_araim = ar.vpl_araim;
  report.pl_ff     = ar.pl_ff;
  report.K_ff_used = ar.K_ff_used;
  report.K_fa_used = ar.K_fa_used;
  last_araim_result_ = ar;

  // Fault-free horizontal sigma for GNSS quality metric
  report.sigma_H   = std::sqrt(ar.sigma_ff_E * ar.sigma_ff_E +
                                ar.sigma_ff_N * ar.sigma_ff_N);

  // PDOP from S0 (4×4 full-solution covariance)
  if (ar.S0(0, 0) > 0 && ar.S0(1, 1) > 0 && ar.S0(2, 2) > 0) {
    report.PDOP = std::sqrt(ar.S0(0, 0) + ar.S0(1, 1) + ar.S0(2, 2));
  }

  // Replace the proxy PL with the ARAIM HPL (more principled)
  report.PL = ar.HPL;

  if (ar.n_detected > 0) {
    logger_->warn("ARAIM FDE: {} fault(s) detected; PRNs: {}",
                  ar.n_detected,
                  [&] {
                    std::string s;
                    for (int p : ar.excluded_prns)
                      s += std::to_string(p) + " ";
                    for (int t : ar.excluded_trunk_ids)
                      s += "T" + std::to_string(t) + " ";
                    return s;
                  }());
  }
}

// ---------------------------------------------------------------------------
// §1.13: Three-state integrity state machine
// ---------------------------------------------------------------------------
IntegrityState IntegrityMonitor::update_state(const IntegrityReport& report) {
  const double al = report.AL;
  const double pl = report.PL;

  // UNSAFE: PL ≥ AL (integrity not available)
  if (pl >= al) {
    current_state_ = IntegrityState::UNSAFE;
    recovery_counter_ = 0;
    return current_state_;
  }

  // SAFE_EXCLUDED: faults detected & excluded, but PL < AL
  if (report.araim_n_det > 0) {
    current_state_ = IntegrityState::SAFE_EXCLUDED;
    recovery_counter_ = 0;
    return current_state_;
  }

  // Transition from SAFE_EXCLUDED → SAFE requires recovery_count consecutive safe frames
  if (current_state_ == IntegrityState::SAFE_EXCLUDED) {
    if (pl < params_.nominal_fraction * al) {
      ++recovery_counter_;
      if (recovery_counter_ >= params_.recovery_count) {
        current_state_ = IntegrityState::SAFE;
        recovery_counter_ = 0;
      }
    } else {
      recovery_counter_ = 0;
    }
    return current_state_;
  }

  // Transition from UNSAFE → SAFE also requires recovery
  if (current_state_ == IntegrityState::UNSAFE) {
    if (pl < params_.nominal_fraction * al) {
      ++recovery_counter_;
      if (recovery_counter_ >= params_.recovery_count) {
        current_state_ = IntegrityState::SAFE;
        recovery_counter_ = 0;
      }
    } else {
      recovery_counter_ = 0;
    }
    return current_state_;
  }

  // SAFE
  current_state_ = IntegrityState::SAFE;
  return current_state_;
}

// ---------------------------------------------------------------------------
// Legacy 4-state mode (kept for backward compat)
// ---------------------------------------------------------------------------
IntegrityMode IntegrityMonitor::update_mode_legacy(const IntegrityReport& report) {
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
  const auto t0_integrity = std::chrono::high_resolution_clock::now();
  IntegrityReport report;
  report.stamp = frame.stamp;

  // --- PL fallback (IAP-RQ-200) ---
  report.PL = compute_PL_proxy(frame);
  report.lambda_max_sigma_p = [&] {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
    return eig.eigenvalues().maxCoeff();
  }();

  // --- Dynamic AL (§1.12: HAL + VAL) ---
  report.al_result = compute_dynamic_AL(frame, trunk);
  report.HAL       = report.al_result.HAL;
  report.VAL       = report.al_result.VAL;
  report.AL        = report.al_result.AL;
  report.HAL_trunk = report.al_result.HAL;   // legacy alias

  // --- Legacy obstacle-based AL integration ---
  if (obstacle_dist_ < 1e8) {
    const double al_obs = std::max(params_.al_scale * obstacle_dist_ - params_.uav_radius,
                                   params_.al_min);
    report.AL = std::min(report.AL, al_obs);
  }

  // --- IM (initial, before ARAIM may override PL) ---
  report.IM = report.AL - report.PL;

  // --- ICP health ---
  report.icp_degenerate = frame.icp_quality.degeneracy_flag;
  report.gamma_lidar    = frame.icp_quality.gamma_lidar;

  // --- GNSS NIS gating (IAP-RQ-220) ---
  if (epoch) {
    run_gnss_gating(*epoch, report);
  }

  // --- ARAIM (IAP-RQ-241–246) — replaces PL if valid ---
  if (epoch) {
    const int n_trunk_obs = trunk ? static_cast<int>(trunk->trunks.size()) : 0;
    report.n_trunks_observed = n_trunk_obs;
    run_araim(*epoch, n_trunk_obs, report);
  }

  // --- TDOP (IAP-RQ-120) ---
  if (trunk) {
    report.tdop = trunk->tdop;
  }

  // Final IM after all PL/AL adjustments
  report.IM = report.AL - report.PL;

  // --- Three-state machine (§1.13) ---
  report.state = update_state(report);
  report.planner_state = (report.state == IntegrityState::UNSAFE)
                           ? PlannerState::HOVER
                           : PlannerState::CRUISE;

  // --- Legacy 4-state mode (deprecated) ---
  report.mode = update_mode_legacy(report);

  logger_->trace(
    "integrity: PL={:.3f}m HPL={:.3f} VPL={:.3f} HAL={:.3f} VAL={:.3f} AL={:.3f}m IM={:.3f}m "
    "state={} lambda_max={:.4f} icp_degen={} gamma_lidar={:.2f} "
    "tdop={:.2f} araim_n_hyp={} araim_n_det={} n_sv={}",
    report.PL, report.HPL, report.VPL, report.HAL, report.VAL,
    report.AL, report.IM, to_string(report.state),
    report.lambda_max_sigma_p, report.icp_degenerate, report.gamma_lidar,
    report.tdop, report.araim_n_hyp, report.araim_n_det, report.n_sv_used);

  if (report.state == IntegrityState::UNSAFE) {
    logger_->warn("INTEGRITY UNSAFE: PL={:.3f} >= AL={:.3f}  IM={:.3f}",
                  report.PL, report.AL, report.IM);
  } else if (report.state == IntegrityState::SAFE_EXCLUDED) {
    logger_->info("INTEGRITY SAFE_EXCLUDED: {} faults excluded, PL={:.3f} < AL={:.3f}",
                  report.araim_n_det, report.PL, report.AL);
  }

  // IAP-RQ-002: timing measurement
  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_integrity).count();
    timing_csv::append(report.stamp, "integrity", elapsed_ms);
  }

  return report;
}

}  // namespace iap
