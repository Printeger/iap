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

BSplineNavigationLayerContribution CTCompactBackend::assemble_navigation_layer(
  const LayerInput& input,
  gtsam::Values* values) {
  BSplineNavigationLayerContribution contribution;
  contribution.activation.enabled = input.graph_context.navigation_layer_enabled;
  contribution.activation.include_clock_states = input.graph_context.navigation_layer_enabled;
  contribution.activation.retain_shared_gnss_anchor = input.gnss_anchor_initialized;

  if (!values || !input.graph_context.navigation_layer_enabled || !input.graph_context.layout) {
    last_gnss_factor_count_ = 0;
    return contribution;
  }

  if (!input.gnss_anchor_initialized) {
    last_gnss_factor_count_ = 0;
    return contribution;
  }

  const gtsam::Key ecef_origin_key = bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = bspline_ecef_rot_key();

  if (!values->exists(ecef_origin_key)) {
    values->insert(ecef_origin_key, input.ecef_origin);
  }
  if (!values->exists(ecef_rot_key)) {
    values->insert(ecef_rot_key, input.ecef_rot);
  }
  contribution.activation.retained_keys = {ecef_origin_key, ecef_rot_key};

  std::size_t gnss_factor_count = 0;
  for (const auto& segment : input.segments) {
    if (segment.gnss_epochs.empty()) {
      continue;
    }

    const gtsam::Key clock_key = bspline_clock_key(segment.auxiliary_index);
    const gtsam::Key velocity_key = bspline_velocity_key(segment.auxiliary_index);

    if (!values->exists(clock_key)) {
      values->insert(clock_key, gtsam::Vector2(0.0, 0.0));
    }
    if (!values->exists(velocity_key)) {
      values->insert(velocity_key, gtsam::Vector3(0.0, 0.0, 0.0));
    }

    contribution.activation.active_auxiliary_indices.push_back(segment.auxiliary_index);

    for (const auto& epoch : segment.gnss_epochs) {
      const auto support = input.graph_context.layout->support_at(epoch.stamp, SplineSensorId::Gnss);
      if (!support) {
        continue;
      }

      SplineStampContext ctx;
      ctx.support = *support;
      ctx.sensor_id = SplineSensorId::Gnss;

      for (const auto& sat : epoch.sats) {
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

        contribution.graph.add(std::make_shared<IntegratedSplinePseudorangeFactor>(
          ctx,
          clock_key,
          ecef_origin_key,
          ecef_rot_key,
          pr_obs,
          input.graph_context.layout));
        ++contribution.gnss_pr_factor_count;
        ++gnss_factor_count;

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

        contribution.graph.add(std::make_shared<IntegratedSplineDopplerFactor>(
          ctx,
          velocity_key,
          clock_key,
          ecef_rot_key,
          dop_obs,
          input.graph_context.layout));
        ++contribution.gnss_dop_factor_count;
        ++gnss_factor_count;
      }
    }
  }

  std::sort(contribution.activation.active_auxiliary_indices.begin(), contribution.activation.active_auxiliary_indices.end());
  contribution.activation.active_auxiliary_indices.erase(
    std::unique(
      contribution.activation.active_auxiliary_indices.begin(),
      contribution.activation.active_auxiliary_indices.end()),
    contribution.activation.active_auxiliary_indices.end());
  contribution.clock_factor_count = contribution.activation.active_auxiliary_indices.size();
  last_gnss_factor_count_ = gnss_factor_count;
  return contribution;
}

void CTCompactBackend::update(
  const CTLocalFrontendResult& local_result,
  const Input& input,
  gtsam::NonlinearFactorGraph* graph,
  gtsam::Values* values) {
  if (!graph || !values) {
    return;
  }

  LayerInput layer_input;
  layer_input.graph_context.layout = std::make_shared<const SplineStateLayout>(local_result.layout);
  layer_input.graph_context.navigation_layer_enabled = true;
  layer_input.gnss_anchor_initialized = input.gnss_anchor_initialized;
  layer_input.ecef_origin = input.ecef_origin;
  layer_input.ecef_rot = input.ecef_rot;
  layer_input.gnss_lever_arm = input.gnss_lever_arm;
  layer_input.gnss_pr_noise_base = input.gnss_pr_noise_base;
  layer_input.gnss_dop_noise_base = input.gnss_dop_noise_base;
  layer_input.gnss_min_elevation = input.gnss_min_elevation;
  layer_input.gnss_elev_noise_exp = input.gnss_elev_noise_exp;

  LayerSegmentInput segment;
  segment.stamp = local_result.layout.controls().empty() ? 0.0 : local_result.layout.controls().back().stamp;
  segment.auxiliary_index = local_result.backend_summary.active_control_indices.empty()
    ? 0
    : static_cast<std::size_t>(local_result.backend_summary.active_control_indices.back());
  segment.gnss_epochs = input.gnss_epochs;
  layer_input.segments.push_back(std::move(segment));

  auto contribution = assemble_navigation_layer(layer_input, values);
  for (const auto& factor : contribution.graph) {
    graph->push_back(factor);
  }
}

CTCompactBackend::DebugStats CTCompactBackend::debug_stats(const CTBackendSummary& summary) const {
  DebugStats stats;
  stats.raw_lidar_factor_count = 0;  // backend never owns raw LiDAR factors
  stats.summary_pose_count = summary.pose_key_count;
  stats.gnss_factor_count = last_gnss_factor_count_;
  return stats;
}

}  // namespace iap
