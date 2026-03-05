// IAP-RQ-312: Satellite visibility prediction by ray casting
// IAP-RQ-313: Canopy density κ along LOS
// IAP-RQ-314: σ_eff(κ, θ) noise model

#include <iap/gnss/visibility_predictor.hpp>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace iap {

VisibilityPredictor::VisibilityPredictor() : params_(Params{}) {}
VisibilityPredictor::VisibilityPredictor(const Params& p) : params_(p) {}

void VisibilityPredictor::set_occupancy(const LocalOccupancyGrid* grid) {
  grid_ = grid;
}

// ---------------------------------------------------------------------------
Eigen::Vector3d VisibilityPredictor::enu_dir(double elevation, double azimuth) {
  // ENU: East=X, North=Y, Up=Z
  const double ce = std::cos(elevation);
  return Eigen::Vector3d(ce * std::cos(azimuth),
                         ce * std::sin(azimuth),
                         std::sin(elevation));
}

// ---------------------------------------------------------------------------
VisibilityResult VisibilityPredictor::predict(const Eigen::Vector3d& pos_world,
                                              const GnssEpoch& epoch) const {
  VisibilityResult res;
  const std::size_t N = epoch.sats.size();
  res.vis_flags.resize(N, false);
  res.kappas.resize(N, 0.0);
  res.sigma_effs.resize(N, params_.canopy.sigma_c);

  double kappa_sum  = 0.0;
  int    n_above_el = 0;

  for (std::size_t i = 0; i < N; ++i) {
    const SatObs& sat = epoch.sats[i];
    if (sat.excluded) continue;

    // Elevation mask
    if (sat.elevation < params_.min_elevation) {
      res.vis_flags[i] = false;
      continue;
    }
    ++n_above_el;

    const Eigen::Vector3d dir = enu_dir(sat.elevation, sat.azimuth);

    // κ and occlusion
    double kappa = 0.0;
    bool blocked = false;
    if (grid_ != nullptr) {
      kappa   = grid_->occupancy_ratio(pos_world, dir, params_.occ_L);
      blocked = grid_->ray_occluded(pos_world, dir, params_.occ_range);
    }

    res.kappas[i]    = kappa;
    res.vis_flags[i] = !blocked;
    if (!blocked) {
      ++res.n_vis;
      kappa_sum += kappa;
    }

    // σ_eff (RQ-314)
    res.sigma_effs[i] = sigma_eff_canopy(params_.canopy, kappa, sat.elevation);
  }

  res.mean_kappa = (res.n_vis > 0) ? (kappa_sum / res.n_vis) : 0.0;

  spdlog::trace("[VisibilityPredictor] pos=({:.1f},{:.1f},{:.1f}) "
                "n_sats={} n_above_el={} n_vis={} mean_kappa={:.3f}",
                pos_world.x(), pos_world.y(), pos_world.z(),
                N, n_above_el, res.n_vis, res.mean_kappa);

  return res;
}

}  // namespace iap
