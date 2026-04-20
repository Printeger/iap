// IAP-RQ-300 / IAP-RQ-410:
// Compact backend implementation.
// Owns GNSS, shared navigation states, mapping/publication handoff, and
// carried priors over summarized states only. Never holds raw LiDAR bucket factors.

#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/ct_backend_summary.hpp>
#include <iap/odometry/integrated_bspline_gnss_factor.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>

namespace iap {

void CTCompactBackend::update(
  const CTLocalFrontendResult& local_result,
  const Input& input,
  gtsam::NonlinearFactorGraph* graph,
  gtsam::Values* values) {
  // IAP-RQ-300 / IAP-RQ-410: Guard against null outputs.
  if (!graph || !values) {
    return;
  }

  // IAP-RQ-300 / IAP-RQ-410: Skip GNSS assembly if anchor is not initialized.
  if (!input.gnss_anchor_initialized || input.gnss_epochs.empty()) {
    last_gnss_factor_count_ = 0;
    return;
  }

  // Copy layout into shared_ptr as required by factor constructors.
  const auto layout_ptr = std::make_shared<const SplineStateLayout>(local_result.layout);
  const gtsam::Key ecef_origin_key = bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = bspline_ecef_rot_key();

  // Seed ECEF anchor states once.
  if (!values->exists(ecef_origin_key)) {
    values->insert(ecef_origin_key, input.ecef_origin);
  }
  if (!values->exists(ecef_rot_key)) {
    values->insert(ecef_rot_key, input.ecef_rot);
  }

  std::size_t gnss_factor_count = 0;

  // IAP-RQ-300 / IAP-RQ-410: Derive stable auxiliary index from active control indices,
  // matching the pattern from odometry_estimation_bspline.cpp where segment.auxiliary_index
  // is the last control index of the segment.
  const gtsam::Key clock_key = bspline_clock_key(
    local_result.backend_summary.active_control_indices.empty()
      ? 0
      : static_cast<std::size_t>(local_result.backend_summary.active_control_indices.back()));
  const gtsam::Key velocity_key = bspline_velocity_key(
    local_result.backend_summary.active_control_indices.empty()
      ? 0
      : static_cast<std::size_t>(local_result.backend_summary.active_control_indices.back()));

  // Seed clock state if not present.
  if (!values->exists(clock_key)) {
    values->insert(clock_key, gtsam::Vector2(0.0, 0.0));
  }
  // Seed velocity state if not present (required by Doppler factor).
  if (!values->exists(velocity_key)) {
    values->insert(velocity_key, gtsam::Vector3(0.0, 0.0, 0.0));
  }

  for (std::size_t epoch_index = 0; epoch_index < input.gnss_epochs.size(); ++epoch_index) {
    const auto& epoch = input.gnss_epochs[epoch_index];

    const auto support = local_result.layout.support_at(epoch.stamp, SplineSensorId::Gnss);
    if (!support) {
      continue;
    }

    SplineStampContext ctx;
    ctx.support = *support;
    ctx.sensor_id = SplineSensorId::Gnss;

    for (const auto& sat : epoch.sats) {
      // IAP-RQ-300 / IAP-RQ-410: Skip excluded satellites and those below min elevation.
      if (sat.excluded || sat.elevation < input.gnss_min_elevation) {
        continue;
      }

      PseudorangeObservation pr_obs;
      pr_obs.pr_meas = sat.pr_meas;
      pr_obs.sat_pos = sat.sat_pos;
      pr_obs.tgd = sat.tgd;
      pr_obs.gps_sec = epoch.gps_sec;
      pr_obs.iono_params = epoch.iono_params;
      pr_obs.sigma = std::max({1e-3, sat.pr_sigma, input.gnss_pr_noise_base});
      pr_obs.sat_id = sat.sat_id;
      pr_obs.constellation = sat.constellation;
      pr_obs.elevation = sat.elevation;

      auto pr_factor = std::make_shared<IntegratedSplinePseudorangeFactor>(
        ctx, clock_key, ecef_origin_key, ecef_rot_key, pr_obs, layout_ptr);
      // Note: gnss_lever_arm from Input is not yet forwarded to IntegratedSplinePseudorangeFactor
      // because the new-style spline-native constructor does not expose a lever arm parameter.
      // TODO(Task 6): propagate lever arm through SplineSensorModel::T_sensor_imu offset.
      graph->add(pr_factor);
      gnss_factor_count++;

      DopplerObservation dop_obs;
      dop_obs.dop_meas = sat.dop_meas;
      dop_obs.sat_pos = sat.sat_pos;
      dop_obs.sat_vel = sat.sat_vel;
      dop_obs.anc_ecef_approx = input.ecef_origin;
      const double sin_el = std::sin(std::max(sat.elevation, input.gnss_min_elevation));
      const double modeled = input.gnss_dop_noise_base / std::pow(std::max(0.052, sin_el), input.gnss_elev_noise_exp);
      dop_obs.sigma = std::max({1e-3, sat.dop_sigma, modeled});
      dop_obs.sat_id = sat.sat_id;
      dop_obs.constellation = sat.constellation;
      dop_obs.elevation = sat.elevation;

      auto dop_factor = std::make_shared<IntegratedSplineDopplerFactor>(
        ctx, velocity_key, clock_key, ecef_rot_key, dop_obs, layout_ptr);
      graph->add(dop_factor);
      gnss_factor_count++;
    }
  }

  last_gnss_factor_count_ = gnss_factor_count;
}

CTCompactBackend::DebugStats CTCompactBackend::debug_stats(const CTBackendSummary& summary) const {
  DebugStats stats;
  stats.raw_lidar_factor_count = 0;  // backend never owns raw LiDAR factors
  stats.summary_pose_count = summary.pose_key_count;
  stats.gnss_factor_count = last_gnss_factor_count_;
  return stats;
}

}  // namespace iap
