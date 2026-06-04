#pragma once
// IAP Step 3: Mapping helper from internal IntegrityReport to ROS msg.
// Extracted for unit-testability of the field mapping.

#include <iap/msg/integrity_report.hpp>  // generated from IntegrityReport.msg
#include <iap/integrity/integrity_types.hpp>

namespace iap {

/// Fill a ROS IntegrityReport message from the internal IntegrityReport struct.
/// All field mappings are explicit — no implicit conversion.
/// The caller is responsible for setting msg.header (stamp, frame_id).
inline void fill_integrity_report_msg(const IntegrityReport& report,
                                       iap::msg::IntegrityReport& msg) {
  // --- State ---
  msg.integrity_state = static_cast<uint8_t>(report.state);

  // --- Monitor-fused PL ---
  msg.hpl  = report.HPL;
  msg.vpl  = report.VPL;
  msg.pl_e = report.PL_E;
  msg.pl_n = report.PL_N;
  msg.pl_u = report.PL_U;

  // --- Alert Limits ---
  msg.hal = report.HAL;
  msg.val = report.VAL;

  // --- Integrity Margin (Step 1: H/V aware) ---
  msg.im     = report.IM;
  msg.im_h   = report.im_h;
  msg.im_v   = report.im_v;
  msg.im_min = report.IM;  // IM is already min(im_h, im_v)

  // --- Fault-free PL ---
  msg.pl_ff     = report.pl_ff;
  msg.pl_ff_v   = report.vpl_araim;
  msg.k_ff_used = report.K_ff_used;
  msg.k_fa_used = report.K_fa_used;

  // --- GNSS quality ---
  msg.n_sv_used        = static_cast<int32_t>(report.n_sv_used);
  msg.n_constellations = static_cast<int32_t>(report.n_constellations);
  msg.pdop             = report.PDOP;
  msg.sigma_h          = report.sigma_H;

  // --- ARAIM diagnostics ---
  msg.n_hypotheses = static_cast<int32_t>(report.araim_n_hyp);
  msg.n_detected   = static_cast<int32_t>(report.araim_n_det);
  msg.excluded_prns.clear();
  msg.excluded_trunk_ids.clear();
  for (int s : report.excluded_sats) {
    msg.excluded_prns.push_back(static_cast<int32_t>(s));
  }

  // --- Trunk geometry ---
  msg.n_trunks_observed = static_cast<int32_t>(report.n_trunks_observed);
  msg.tdop              = report.tdop;

  // =========================================================================
  // Step 3: Source breakdown fields
  // =========================================================================

  // --- GNSS source breakdown ---
  msg.gnss_valid      = (report.gnss_valid != 0);
  msg.gnss_hpl        = report.gnss_HPL;
  msg.gnss_vpl        = report.gnss_VPL;
  msg.gnss_pl_e       = report.gnss_PL_E;
  msg.gnss_pl_n       = report.gnss_PL_N;
  msg.gnss_pl_u       = report.gnss_PL_U;
  msg.gnss_pl_ff      = report.gnss_pl_ff;
  msg.gnss_k_ff_used  = report.gnss_K_ff_used;
  msg.gnss_k_fa_used  = report.gnss_K_fa_used;
  msg.gnss_n_hyp      = static_cast<int32_t>(report.gnss_n_hyp);
  msg.gnss_n_det      = static_cast<int32_t>(report.gnss_n_det);

  // --- LiDAR source breakdown ---
  msg.lidar_valid      = (report.lidar_valid != 0);
  msg.lidar_hpl        = report.lidar_HPL;
  msg.lidar_vpl        = report.lidar_VPL;
  msg.lidar_pl_e       = report.lidar_PL_E;
  msg.lidar_pl_n       = report.lidar_PL_N;
  msg.lidar_pl_u       = report.lidar_PL_U;
  msg.lidar_n_hyp      = static_cast<int32_t>(report.lidar_n_hyp);
  msg.lidar_n_det      = static_cast<int32_t>(report.lidar_n_det);
  msg.lidar_worst_mode = report.lidar_worst_mode;

  // --- Fallback source breakdown (Step 4: explicit source) ---
  msg.fallback_valid = !report.numerical_failure.fallback_pl_invalid;
  msg.fallback_hpl   = report.fallback_HPL;
  msg.fallback_vpl   = report.fallback_VPL;

  // --- Final fusion diagnostics (Step 4: from policy) ---
  msg.fusion_mode      = report.fusion_mode_str;
  msg.final_hpl_source = report.final_HPL_source;
  msg.final_vpl_source = report.final_VPL_source;
  msg.final_pl_source  = report.final_PL_source;

  // --- Numerical failure flags ---
  msg.fallback_pl_invalid        = report.numerical_failure.fallback_pl_invalid;
  msg.gnss_araim_invalid         = report.numerical_failure.gnss_araim_invalid;
  msg.lidar_integrity_invalid    = report.numerical_failure.lidar_integrity_invalid;
  msg.hal_invalid                = report.numerical_failure.hal_invalid;
  msg.val_invalid                = report.numerical_failure.val_invalid;
  msg.im_invalid                 = report.numerical_failure.im_invalid;
  msg.any_nan_rejected           = report.numerical_failure.any_nan_rejected;
  msg.any_inf_rejected           = report.numerical_failure.any_inf_rejected;
  msg.negative_variance_rejected = report.numerical_failure.negative_variance_rejected;
  msg.degenerate_geometry        = report.numerical_failure.degenerate_geometry;
  msg.failure_reason             = report.numerical_failure.failure_reason;
}

}  // namespace iap
