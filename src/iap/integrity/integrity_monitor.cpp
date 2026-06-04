// IAP-RQ-200 / RQ-210 / RQ-220: Integrity monitoring implementation
// §1.12: Dynamic AL (HAL + VAL)
// §1.13: Three-state integrity state machine (SAFE / SAFE_EXCLUDED / UNSAFE)

#include <iap/integrity/integrity_monitor.hpp>
#include <iap/integrity/numerical_guard.hpp>
#include <iap/util/timing_csv.hpp>
#include <iap/util/logging.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace iap {

// ---------------------------------------------------------------------------
IntegrityMonitor::IntegrityMonitor()
    : params_(Params{}), gnss_araim_(GnssAraimParams{}), lidar_araim_(LidarAraim{}),
      fusion_policy_(IntegrityFusionPolicyParams{}) {
  logger_ = glim::create_module_logger("integrity");
}

IntegrityMonitor::IntegrityMonitor(const Params& params)
: params_(params),
  gnss_araim_(params.gnss_araim_params),
  lidar_araim_(params.lidar_araim_params),
  fusion_policy_(IntegrityFusionPolicyParams{
      params.fusion_mode,
      params.require_valid_gnss,
      params.require_valid_lidar,
      params.conservative_hpl_m,
      params.conservative_vpl_m}) {
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
  // Step 2: numerical guards on covariance and eigenvalue
  if (!numerical_guard::is_valid_covariance(frame.sigma_p)) {
    return numerical_guard::kSentinel;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
  const double lambda_max = eig.eigenvalues().maxCoeff();

  if (!numerical_guard::is_valid_positive(lambda_max)) {
    return numerical_guard::kSentinel;
  }

  return params_.K_pl * std::sqrt(lambda_max);
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
  // Step 2: guard HAL and VAL against NaN/Inf
  if (!numerical_guard::is_valid(al.HAL)) {
    al.HAL = params_.HAL_trunk_default;
  }
  if (!numerical_guard::is_valid(al.VAL)) {
    al.VAL = params_.VAL_default;
  }

  if (al.HAL < al.VAL) {
    al.AL = al.HAL;
    al.al_from_trunk = true;
  } else {
    al.AL = al.VAL;
    al.al_from_trunk = false;
  }

  // Apply minimum AL clamp (Step 2: guard against NaN)
  al.AL = std::max(numerical_guard::sentinel_if_invalid(
      std::min(al.HAL, al.VAL)), params_.al_min);

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
IntegritySourceResult IntegrityMonitor::run_araim(const GnssEpoch& epoch,
                                                   int n_trunk_obs) {
  if (!params_.enable_gnss_araim) {
    return IntegritySourceResult::make_disabled("GNSS");
  }

  const GnssAraimResult ar = gnss_araim_.run(epoch, n_trunk_obs);

  if (!ar.valid) {
    return IntegritySourceResult::make_invalid("GNSS", "ARAIM result invalid");
  }

  if (!numerical_guard::is_valid(ar.HPL) || !numerical_guard::is_valid(ar.VPL)) {
    return IntegritySourceResult::make_invalid("GNSS", "GNSS ARAIM produced NaN/Inf PL");
  }

  last_gnss_araim_result_ = ar;

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

  return IntegritySourceResult::make_valid("GNSS",
      ar.HPL, ar.VPL, ar.PL_E, ar.PL_N, ar.PL_U);
}

// ---------------------------------------------------------------------------
IntegritySourceResult IntegrityMonitor::run_lidar_araim(
    const LidarAraimSnapshot& snapshot,
    const FGOPositionInfo* fgo_info) {
  if (!params_.enable_lidar_integrity) {
    return IntegritySourceResult::make_disabled("LIDAR");
  }

  const FGOPositionInfo empty_fgo;
  const auto& fgo = fgo_info ? *fgo_info : empty_fgo;
  const LidarAraimResult lr = lidar_araim_.run(snapshot, fgo);

  last_lidar_araim_result_ = lr;

  if (!lr.valid) {
    return IntegritySourceResult::make_invalid("LIDAR", "LiDAR ARAIM result invalid");
  }

  if (!numerical_guard::is_valid(lr.HPL) || !numerical_guard::is_valid(lr.VPL)) {
    return IntegritySourceResult::make_invalid("LIDAR", "LiDAR integrity produced NaN/Inf PL");
  }

  return IntegritySourceResult::make_valid("LIDAR",
      lr.HPL, lr.VPL, lr.PL_E, lr.PL_N, lr.PL_U);
}

// ---------------------------------------------------------------------------
// §1.13: Three-state integrity state machine (Step 1: H/V aware)
// ---------------------------------------------------------------------------
IntegrityState IntegrityMonitor::update_state(const IntegrityReport& report) {
  const bool h_safe = report.HPL < report.HAL;
  const bool v_safe = report.VPL < report.VAL;
  const bool safe = h_safe && v_safe;

  // UNSAFE: either dimension exceeds limit
  if (!safe) {
    if (!h_safe) {
      logger_->warn("INTEGRITY UNSAFE (horizontal): HPL={:.3f} >= HAL={:.3f}",
                    report.HPL, report.HAL);
    }
    if (!v_safe) {
      logger_->warn("INTEGRITY UNSAFE (vertical): VPL={:.3f} >= VAL={:.3f}",
                    report.VPL, report.VAL);
    }
    current_state_ = IntegrityState::UNSAFE;
    recovery_counter_ = 0;
    return current_state_;
  }

  // SAFE_EXCLUDED: faults detected & excluded, but both dimensions safe
  if (report.araim_n_det > 0) {
    current_state_ = IntegrityState::SAFE_EXCLUDED;
    recovery_counter_ = 0;
    return current_state_;
  }

  // Transition from SAFE_EXCLUDED → SAFE: requires both H and V below nominal
  if (current_state_ == IntegrityState::SAFE_EXCLUDED) {
    if (report.HPL < params_.nominal_fraction * report.HAL &&
        report.VPL < params_.nominal_fraction * report.VAL) {
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

  // Transition from UNSAFE → SAFE: requires both dimensions below nominal
  if (current_state_ == IntegrityState::UNSAFE) {
    if (report.HPL < params_.nominal_fraction * report.HAL &&
        report.VPL < params_.nominal_fraction * report.VAL) {
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
// Legacy 4-state mode (kept for backward compat; Step 1: H/V aware)
// ---------------------------------------------------------------------------
IntegrityMode IntegrityMonitor::update_mode_legacy(const IntegrityReport& report) {
  // Use worst-case ratio: max(HPL/HAL, VPL/VAL)
  const double worst_ratio = std::max(
      report.HAL > 0.0 ? report.HPL / report.HAL : 1e9,
      report.VAL > 0.0 ? report.VPL / report.VAL : 1e9);

  IntegrityMode next = current_mode_;

  switch (current_mode_) {
    case IntegrityMode::NOMINAL:
    case IntegrityMode::CAUTION:
      if (worst_ratio >= 1.0) {
        next = IntegrityMode::ALERT;
        recovery_counter_ = 0;
      } else if (worst_ratio > params_.caution_fraction) {
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
      if (worst_ratio < params_.nominal_fraction) {
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

// ===========================================================================
// Step 9: Decomposed compute() helpers
// ===========================================================================

IntegritySourceResult IntegrityMonitor::buildFallbackSource(
    const glim::EstimationFrame& frame,
    IntegrityReport& report) const {
  const double fallback_pl = compute_PL_proxy(frame);
  IntegritySourceResult fallback_src;
  if (fallback_pl >= numerical_guard::kSentinel * 0.99 || !std::isfinite(fallback_pl)) {
    report.numerical_failure.fallback_pl_invalid = true;
    if (!std::isfinite(fallback_pl)) {
      report.numerical_failure.any_nan_rejected = true;
    }
    fallback_src = IntegritySourceResult::make_invalid("FALLBACK", "covariance invalid");
  } else {
    fallback_src = IntegritySourceResult::make_valid("FALLBACK",
        fallback_pl, fallback_pl, fallback_pl, fallback_pl, fallback_pl);
  }

  report.fallback_HPL = numerical_guard::sentinel_if_invalid(fallback_src.HPL);
  report.fallback_VPL = numerical_guard::sentinel_if_invalid(fallback_src.VPL);
  report.lambda_max_sigma_p = [&] {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frame.sigma_p, Eigen::EigenvaluesOnly);
    return eig.eigenvalues().maxCoeff();
  }();

  return fallback_src;
}

IntegritySourceResult IntegrityMonitor::evaluateGnssSource(
    const GnssEpoch* epoch,
    const TrunkDetectionResult* trunk,
    IntegrityReport& report) {
  if (epoch) {
    run_gnss_gating(*epoch, report);
  }

  IntegritySourceResult gnss_src = IntegritySourceResult::make_disabled("GNSS");
  if (epoch && params_.enable_gnss_integrity) {
    const int n_trunk_obs = trunk ? static_cast<int>(trunk->trunks.size()) : 0;
    report.n_trunks_observed = n_trunk_obs;
    gnss_src = run_araim(*epoch, n_trunk_obs);

    const auto& ar = last_gnss_araim_result_;
    report.araim_valid  = ar.valid ? 1 : 0;
    report.araim_n_hyp  = ar.n_hypotheses;
    report.araim_n_det  = ar.n_detected;
    report.araim_detected_rows = ar.detected_rows;
    report.gnss_valid   = gnss_src.valid ? 1 : 0;
    report.gnss_n_hyp   = ar.n_hypotheses;
    report.gnss_n_det   = ar.n_detected;
    if (gnss_src.valid) {
      report.gnss_HPL       = ar.HPL;
      report.gnss_VPL       = ar.VPL;
      report.gnss_PL_E      = ar.PL_E;
      report.gnss_PL_N      = ar.PL_N;
      report.gnss_PL_U      = ar.PL_U;
      report.gnss_pl_ff     = ar.pl_ff;
      report.gnss_K_ff_used = ar.K_ff_used;
      report.gnss_K_fa_used = ar.K_fa_used;
      report.pl_araim  = ar.pl_araim;
      report.vpl_araim = ar.vpl_araim;
      report.pl_ff     = ar.pl_ff;
      report.K_ff_used = ar.K_ff_used;
      report.K_fa_used = ar.K_fa_used;
      report.sigma_H   = std::sqrt(ar.sigma_ff_E * ar.sigma_ff_E +
                                    ar.sigma_ff_N * ar.sigma_ff_N);
      if (ar.S0(0, 0) > 0 && ar.S0(1, 1) > 0 && ar.S0(2, 2) > 0) {
        report.PDOP = std::sqrt(ar.S0(0, 0) + ar.S0(1, 1) + ar.S0(2, 2));
      }
    } else {
      report.numerical_failure.gnss_araim_invalid = true;
    }
  }

  return gnss_src;
}

IntegritySourceResult IntegrityMonitor::evaluateLidarSource(
    const LidarAraimSnapshot* lidar_snapshot,
    const FGOPositionInfo* fgo_info,
    IntegrityReport& report) {
  IntegritySourceResult lidar_src = IntegritySourceResult::make_disabled("LIDAR");
  if (lidar_snapshot && params_.enable_lidar_integrity) {
    lidar_src = run_lidar_araim(*lidar_snapshot, fgo_info);
    const auto& lr = last_lidar_araim_result_;
    report.lidar_valid = lidar_src.valid ? 1 : 0;
    report.lidar_n_hyp = lr.n_hypotheses;
    report.lidar_n_det = lr.n_detected;
    report.lidar_PL_E  = lr.PL_E;
    report.lidar_PL_N  = lr.PL_N;
    report.lidar_PL_U  = lr.PL_U;
    report.lidar_HPL   = lr.HPL;
    report.lidar_VPL   = lr.VPL;
    report.lidar_worst_mode = lr.worst_mode;
    if (!lidar_src.valid) {
      report.numerical_failure.lidar_integrity_invalid = true;
    }
  }

  return lidar_src;
}

void IntegrityMonitor::fuseIntegritySources(
    const IntegritySourceResult& fallback_src,
    const IntegritySourceResult& gnss_src,
    const IntegritySourceResult& lidar_src,
    IntegrityReport& report) {
  const auto fused = fusion_policy_.fuse(fallback_src, gnss_src, lidar_src);

  report.HPL  = fused.HPL;
  report.VPL  = fused.VPL;
  report.PL_E = fused.PL_E;
  report.PL_N = fused.PL_N;
  report.PL_U = fused.PL_U;
  report.PL   = fused.HPL;
  report.final_HPL_source = fused.final_HPL_source;
  report.final_VPL_source = fused.final_VPL_source;
  report.final_PL_source  = fused.final_PL_source;
  report.fusion_mode_str  = to_string(params_.fusion_mode);

  if (!fused.any_source_valid && !fused.failure_reason.empty()) {
    report.numerical_failure.failure_reason = fused.failure_reason;
    report.numerical_failure.fallback_pl_invalid = true;
  }
}

void IntegrityMonitor::computeAlertLimits(
    const glim::EstimationFrame& frame,
    const TrunkDetectionResult* trunk,
    IntegrityReport& report) {
  const auto t0_al = std::chrono::high_resolution_clock::now();
  report.al_result = compute_dynamic_AL(frame, trunk);
  report.HAL       = report.al_result.HAL;
  report.VAL       = report.al_result.VAL;
  report.AL        = report.al_result.AL;
  report.HAL_trunk = report.al_result.HAL;
  {
    const double elapsed_al = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_al).count();
    timing_csv::append(report.stamp, "2.3_dynamic_al", elapsed_al);
  }

  if (obstacle_dist_ < 1e8) {
    const double al_obs = std::max(params_.al_scale * obstacle_dist_ - params_.uav_radius,
                                   params_.al_min);
    report.AL = std::min(report.AL, al_obs);
  }
}

void IntegrityMonitor::computeIntegrityMargins(IntegrityReport& report) const {
  report.im_h = numerical_guard::sentinel_if_invalid(report.HAL - report.HPL);
  report.im_v = numerical_guard::sentinel_if_invalid(report.VAL - report.VPL);
  report.IM   = numerical_guard::sentinel_if_invalid(std::min(report.im_h, report.im_v));
  if (!numerical_guard::is_valid(report.im_h) || !numerical_guard::is_valid(report.im_v)) {
    report.numerical_failure.im_invalid = true;
  }
}

void IntegrityMonitor::updateStateAndPlannerMode(IntegrityReport& report) {
  const auto t0_state = std::chrono::high_resolution_clock::now();
  report.state = update_state(report);
  report.planner_state = (report.state == IntegrityState::UNSAFE)
                           ? PlannerState::HOVER
                           : PlannerState::CRUISE;
  {
    const double elapsed_state = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_state).count();
    timing_csv::append(report.stamp, "2.3_state_machine", elapsed_state);
  }

  report.mode = update_mode_legacy(report);

  logger_->trace(
    "integrity: HPL={:.3f} VPL={:.3f} HAL={:.3f} VAL={:.3f} AL={:.3f}m "
    "im_h={:.3f} im_v={:.3f} IM={:.3f}m state={} lambda_max={:.4f} "
    "fusion={} hpl_src={} vpl_src={}",
    report.HPL, report.VPL, report.HAL, report.VAL,
    report.AL, report.im_h, report.im_v, report.IM, to_string(report.state),
    report.lambda_max_sigma_p, report.fusion_mode_str,
    report.final_HPL_source, report.final_VPL_source);

  if (report.state == IntegrityState::UNSAFE) {
    logger_->warn("INTEGRITY UNSAFE: HPL={:.3f} vs HAL={:.3f} (im_h={:.3f}), "
                  "VPL={:.3f} vs VAL={:.3f} (im_v={:.3f}), IM={:.3f}",
                  report.HPL, report.HAL, report.im_h,
                  report.VPL, report.VAL, report.im_v, report.IM);
  } else if (report.state == IntegrityState::SAFE_EXCLUDED) {
    logger_->info("INTEGRITY SAFE_EXCLUDED: {} faults excluded, HPL={:.3f}<HAL={:.3f} VPL={:.3f}<VAL={:.3f}",
                  report.araim_n_det, report.HPL, report.HAL, report.VPL, report.VAL);
  }
}

// ===========================================================================
// compute() — orchestration (Step 9: decomposed)
// ===========================================================================

IntegrityReport IntegrityMonitor::compute(const glim::EstimationFrame& frame,
                                           const GnssEpoch* epoch,
                                           const TrunkDetectionResult* trunk,
                                           const FGOPositionInfo* fgo_info,
                                           const LidarAraimSnapshot* lidar_snapshot) {
  const auto t0_integrity = std::chrono::high_resolution_clock::now();
  IntegrityReport report;
  report.stamp = frame.stamp;
  report.araim_n_det = 0;  // ensure deterministic state machine input

  // --- ICP health ---
  report.icp_degenerate = frame.icp_quality.degeneracy_flag;
  report.gamma_lidar    = frame.icp_quality.gamma_lidar;

  // --- Build source results ---
  const auto fallback_src = buildFallbackSource(frame, report);
  const auto gnss_src     = evaluateGnssSource(epoch, trunk, report);
  const auto lidar_src    = evaluateLidarSource(lidar_snapshot, fgo_info, report);

  // --- TDOP ---
  if (trunk) {
    report.tdop = trunk->tdop;
  }

  // --- Fuse ---
  fuseIntegritySources(fallback_src, gnss_src, lidar_src, report);

  // --- Alert limits, margins, state ---
  computeAlertLimits(frame, trunk, report);
  computeIntegrityMargins(report);
  updateStateAndPlannerMode(report);

  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_integrity).count();
    timing_csv::append(report.stamp, "2.3_integrity_total", elapsed_ms);
  }

  return report;
}

}  // namespace iap
