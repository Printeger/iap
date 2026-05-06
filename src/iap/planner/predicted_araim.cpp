// IAP-RQ-331: Predicted ARAIM PL for planning (geometry-only mode)

#include <iap/planner/predicted_araim.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>

namespace iap {

// ---------------------------------------------------------------------------
PredictedAraimComputer::PredictedAraimComputer()
: params_{}, araim_(params_.araim_params), vis_(params_.vis_params) {}

PredictedAraimComputer::PredictedAraimComputer(const Params& p)
: params_(p), araim_(p.araim_params), vis_(p.vis_params) {}

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
  // open sky by VisibilityPredictor.
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
  std::vector<Araim::SatGeometry> geom;
  geom.reserve(static_cast<std::size_t>(vis.n_vis));

  for (std::size_t i = 0; i < epoch_->sats.size(); ++i) {
    if (i >= vis.vis_flags.size() || !vis.vis_flags[i]) continue;
    if (epoch_->sats[i].excluded) continue;

    Araim::SatGeometry sg;
    sg.elevation = epoch_->sats[i].elevation;
    sg.azimuth   = epoch_->sats[i].azimuth;
    // Use canopy-aware sigma if available; otherwise default pr_sigma
    sg.pr_sigma  = (i < vis.sigma_effs.size() && vis.sigma_effs[i] > 0.0)
                    ? vis.sigma_effs[i]
                    : epoch_->sats[i].pr_sigma;
    sg.sat_id    = epoch_->sats[i].sat_id;
    geom.push_back(sg);
  }

  if (static_cast<int>(geom.size()) < 4) {
    auto out = fallback("too_few_sats");
    out.n_vis = static_cast<int>(geom.size());
    return out;
  }

  // 3. Run geometry-only ARAIM (r = 0)
  const AraimResult ar = araim_.predict_geometry(geom);

  if (!ar.valid) {
    auto out = fallback("singular_geometry");
    out.n_vis = static_cast<int>(geom.size());
    out.n_hypotheses = ar.n_hypotheses;
    return out;
  }

  spdlog::trace("[PredictedAraim] pos=({:.1f},{:.1f},{:.1f}) n_vis={} "
                "pl_ff={:.3f} pl_araim={:.3f}",
                pos_world.x(), pos_world.y(), pos_world.z(),
                vis.n_vis, ar.pl_ff, ar.pl_araim);

  PredictedAraimResult out;
  out.valid = true;
  out.fallback = false;
  out.fallback_reason.clear();
  out.hpl = ar.HPL;
  out.vpl = ar.VPL;
  out.pl_scalar = std::max(out.hpl, out.vpl);
  out.pl_e = ar.PL_E;
  out.pl_n = ar.PL_N;
  out.pl_u = ar.PL_U;
  out.pl_ff_h = ar.pl_ff;
  out.pl_ff_v = ar.pl_ff_V;
  out.sigma_h = std::sqrt(std::max(
      0.0, ar.sigma_ff_E * ar.sigma_ff_E + ar.sigma_ff_N * ar.sigma_ff_N));
  out.sigma_v = ar.sigma_ff_U;
  if (ar.S0(0, 0) > 0.0 && ar.S0(1, 1) > 0.0 && ar.S0(2, 2) > 0.0) {
    out.pdop = std::sqrt(ar.S0(0, 0) + ar.S0(1, 1) + ar.S0(2, 2));
  }
  out.n_vis = static_cast<int>(geom.size());
  out.n_hypotheses = ar.n_hypotheses;
  return out;
}

}  // namespace iap
