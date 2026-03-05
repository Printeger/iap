// IAP-RQ-331: Predicted ARAIM PL for planning (geometry-only mode)

#include <iap/planner/predicted_araim.hpp>
#include <spdlog/spdlog.h>
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

  // Fallback: no occupancy or epoch available
  if (grid_ == nullptr || epoch_ == nullptr) {
    return params_.fallback_pl;
  }

  // 1. Predict visible satellites at this waypoint
  const VisibilityResult vis = vis_.predict(pos_world, *epoch_);

  if (vis.n_vis < 4) {
    // Too few sats for ARAIM — return fallback (conservative)
    return params_.fallback_pl;
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
    return params_.fallback_pl;
  }

  // 3. Run geometry-only ARAIM (r = 0)
  const AraimResult ar = araim_.predict_geometry(geom);

  if (!ar.valid) {
    return params_.fallback_pl;
  }

  spdlog::trace("[PredictedAraim] pos=({:.1f},{:.1f},{:.1f}) n_vis={} "
                "pl_ff={:.3f} pl_araim={:.3f}",
                pos_world.x(), pos_world.y(), pos_world.z(),
                vis.n_vis, ar.pl_ff, ar.pl_araim);

  return ar.pl_araim;
}

}  // namespace iap
