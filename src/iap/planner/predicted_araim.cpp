// IAP-RQ-331: GNSS advisory PL proxy for planning (geometry-only mode).

#include <iap/planner/predicted_araim.hpp>
#include <iap/gnss/clock_geometry.hpp>
#include <spdlog/spdlog.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace iap {

// ---------------------------------------------------------------------------
PredictedAraimComputer::PredictedAraimComputer()
: params_{}, geom_predictor_(params_.geometry_params), vis_(params_.vis_params) {}

PredictedAraimComputer::PredictedAraimComputer(const Params& p)
: params_(p), geom_predictor_(p.geometry_params), vis_(p.vis_params) {}

// ---------------------------------------------------------------------------
void PredictedAraimComputer::set_occupancy(const LocalOccupancyGrid* grid) {
  grid_ = grid;
  vis_.set_occupancy(grid_);
}

void PredictedAraimComputer::set_epoch(const GnssEpoch* epoch) {
  epoch_ = epoch;
}

// ---------------------------------------------------------------------------
double PredictedAraimComputer::predict_araim_pl(
    const Eigen::Vector3d& pos_world) const {
  return predict_araim_result(pos_world).hpl;
}

PredictedAraimResult PredictedAraimComputer::predict_araim_result(
    const Eigen::Vector3d& pos_world) const {
  auto fallback = [&](const char* reason) {
    PredictedAraimResult out;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = reason;
    out.hpl = params_.fallback_pl;
    out.vpl = params_.fallback_pl;
    out.pl_scalar = params_.fallback_pl;
    out.pl_e = params_.fallback_pl;
    out.pl_n = params_.fallback_pl;
    out.pl_u = params_.fallback_pl;
    out.pl_ff_h = params_.fallback_pl;
    out.pl_ff_v = params_.fallback_pl;
    out.sigma_h = params_.fallback_pl;
    out.sigma_v = params_.fallback_pl;
    out.pdop = 1e9;
    return out;
  };

  // Fallback: no GNSS epoch available. A missing occupancy grid is treated as
  // open sky by VisibilityPredictor. This remains an advisory proxy.
  if (epoch_ == nullptr) {
    return fallback("no_gnss_epoch");
  }

  // 1. Predict visible satellites at this waypoint
  const VisibilityResult vis = vis_.predict(pos_world, *epoch_);

  if (vis.n_vis < 4) {
    // Too few sats for ARAIM — return fallback (conservative)
    auto out = fallback("too_few_sats");
    out.n_vis = vis.n_vis;
    return out;
  }

  // 2. Build SatGeometry for visible satellites
  std::vector<GnssGeometrySat> geom;
  geom.reserve(static_cast<std::size_t>(vis.n_vis));

  for (std::size_t i = 0; i < epoch_->sats.size(); ++i) {
    if (i >= vis.vis_flags.size() || !vis.vis_flags[i]) continue;
    if (epoch_->sats[i].excluded) continue;

    GnssGeometrySat sg;
    sg.elevation = epoch_->sats[i].elevation;
    sg.azimuth   = epoch_->sats[i].azimuth;
    sg.pr_sigma  = (i < vis.sigma_effs.size() && vis.sigma_effs[i] > 0.0)
                    ? vis.sigma_effs[i]
                    : epoch_->sats[i].pr_sigma;
    sg.sat_id    = epoch_->sats[i].sat_id;
    sg.constellation = epoch_->sats[i].constellation;
    geom.push_back(sg);
  }

  if (static_cast<int>(geom.size()) < 4) {
    auto out = fallback("too_few_sats");
    out.n_vis = static_cast<int>(geom.size());
    return out;
  }

  // 3. Run geometry-only ARAIM machinery (r = 0) to produce a non-certified
  // GNSS advisory PL proxy for planning.
  const GnssGeometryPlResult result = geom_predictor_.predict(geom);

  if (!result.valid) {
    auto out = fallback("singular_geometry");
    out.n_vis = static_cast<int>(geom.size());
    out.n_hypotheses = result.n_hypotheses;
    return out;
  }

  spdlog::trace("[GNSS Advisory PL Proxy] pos=({:.1f},{:.1f},{:.1f}) n_vis={} "
                "pl_ff_proxy={:.3f} gnss_advisory_hpl_proxy={:.3f}",
                pos_world.x(), pos_world.y(), pos_world.z(),
                vis.n_vis, result.pl_ff, result.HPL);

  PredictedAraimResult out;
  out.valid = true;
  out.fallback = false;
  out.fallback_reason.clear();
  out.hpl = result.HPL;
  out.vpl = result.VPL;
  out.pl_scalar = std::max(out.hpl, out.vpl);
  out.pl_e = result.PL_E;
  out.pl_n = result.PL_N;
  out.pl_u = result.PL_U;
  out.pl_ff_h = result.pl_ff;
  out.pl_ff_v = result.pl_ff_V;
  out.sigma_h = std::sqrt(std::max(
      0.0, result.sigma_ff_E * result.sigma_ff_E + result.sigma_ff_N * result.sigma_ff_N));
  out.sigma_v = result.sigma_ff_U;
  if (result.S0(0, 0) > 0.0 && result.S0(1, 1) > 0.0 && result.S0(2, 2) > 0.0) {
    out.pdop = std::sqrt(result.S0(0, 0) + result.S0(1, 1) + result.S0(2, 2));
  }
  out.n_vis = static_cast<int>(geom.size());
  out.n_hypotheses = result.n_hypotheses;
  return out;
}

GnssAdvisoryFimResult PredictedAraimComputer::predict_advisory_fim(
    const Eigen::Vector3d& pos_world) const {
  GnssAdvisoryFimResult out;

  auto fallback = [&](const char* reason) {
    out.valid = false;
    out.fallback_reason = reason;
    out.lambda.setZero();
    out.h_full.setZero();
    fill_fim_diagnostics(out);
    return out;
  };

  if (!pos_world.allFinite()) {
    return fallback("invalid_position");
  }
  if (epoch_ == nullptr) {
    return fallback("no_gnss_epoch");
  }

  const VisibilityResult vis = vis_.predict(pos_world, *epoch_);
  out.n_visible = vis.n_vis;

  std::vector<GnssGeometrySat> geom;
  geom.reserve(static_cast<std::size_t>(vis.n_vis));
  for (std::size_t i = 0; i < epoch_->sats.size(); ++i) {
    if (i >= vis.vis_flags.size() || !vis.vis_flags[i]) continue;
    if (epoch_->sats[i].excluded) continue;
    GnssGeometrySat sg;
    sg.elevation = epoch_->sats[i].elevation;
    sg.azimuth = epoch_->sats[i].azimuth;
    sg.pr_sigma = (i < vis.sigma_effs.size() && vis.sigma_effs[i] > 0.0)
                      ? vis.sigma_effs[i]
                      : epoch_->sats[i].pr_sigma;
    sg.sat_id = epoch_->sats[i].sat_id;
    sg.constellation = epoch_->sats[i].constellation;
    geom.push_back(sg);
  }
  out.n_used = static_cast<int>(geom.size());
  if (out.n_used < 4) {
    return fallback("too_few_sats");
  }

  Eigen::MatrixXd positions(geom.size(),3);
  Eigen::VectorXd weights(geom.size());
  std::vector<int> systems;
  for (std::size_t row = 0; row < geom.size(); ++row) {
    const auto& sat = geom[row];
    const Eigen::Vector3d direction(std::cos(sat.elevation)*std::sin(sat.azimuth),
        std::cos(sat.elevation)*std::cos(sat.azimuth), std::sin(sat.elevation));
    positions.row(row) = (epoch_->R_query_enu * direction).transpose();
    const double sigma = std::max(sat.pr_sigma,0.01);
    weights[row] = 1/(sigma*sigma);
    systems.push_back(gnss_constellation_id(sat.constellation));
  }
  const auto design = gnss_clock_design(positions, systems);
  if (design.rows() != static_cast<int>(geom.size())) return fallback("unsupported_constellation_clock");
  out.h_full = design.transpose() * weights.asDiagonal() * design;
  const double clock_eps =
      std::isfinite(params_.fim_clock_epsilon) && params_.fim_clock_epsilon > 0.0
          ? params_.fim_clock_epsilon
          : 1.0e-6;
  if (!gnss_eliminate_active_clocks(out.h_full, clock_eps, &out.lambda)) {
    return fallback("degenerate_clock_information");
  }

  if (!out.lambda.allFinite()) {
    return fallback("invalid_gnss_fim");
  }

  fill_fim_diagnostics(out);
  const double psd_eps =
      std::isfinite(params_.fim_psd_epsilon) && params_.fim_psd_epsilon > 0.0
          ? params_.fim_psd_epsilon
          : 1.0e-9;
  if (!std::isfinite(out.min_eig) || out.min_eig < -psd_eps ||
      out.max_eig <= 0.0) {
    return fallback("gnss_fim_not_psd");
  }
  if (out.min_eig < 0.0) {
    out.lambda += Eigen::Matrix3d::Identity() * (-out.min_eig + psd_eps);
    out.regularized = true;
    fill_fim_diagnostics(out);
  }

  out.valid = true;
  out.fallback_reason.clear();
  return out;
}

}  // namespace iap
