// IAP-RQ-312: Satellite visibility prediction by ray casting
// IAP-RQ-313: Canopy density κ along LOS
// IAP-RQ-314: σ_eff(κ, θ) noise model

#include <iap/gnss/visibility_predictor.hpp>
#include <cmath>
#include <algorithm>
#include <utility>
#include <spdlog/spdlog.h>

namespace iap {

VisibilityPredictor::VisibilityPredictor() : params_(Params{}) {}
VisibilityPredictor::VisibilityPredictor(const Params& p) : params_(p) {}

void VisibilityPredictor::set_occupancy(const LocalOccupancyGrid* grid) {
  grid_ = grid;
}

void VisibilityPredictor::set_observation_predicate(
    ObservationPredicate predicate) {
  observation_predicate_ = std::move(predicate);
}

// ---------------------------------------------------------------------------
Eigen::Vector3d VisibilityPredictor::enu_dir(double elevation, double azimuth) {
  // ENU: East=X, North=Y, Up=Z
  const double ce = std::cos(elevation);
  return Eigen::Vector3d(ce * std::sin(azimuth),
                         ce * std::cos(azimuth),
                         std::sin(elevation));
}

// ---------------------------------------------------------------------------
VisibilityResult VisibilityPredictor::predict(const Eigen::Vector3d& pos_world,
                                              const GnssEpoch& epoch,
                                              const bool measured_epoch_support) const {
  VisibilityResult res;
  const std::size_t N = epoch.sats.size();
  res.vis_flags.resize(N, false);
  res.kappas.resize(N, 0.0);
  res.sigma_effs.resize(N, params_.canopy.sigma_c);
  res.unknown_flags.resize(N, false);

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

    if (measured_epoch_support &&
        (!std::isfinite(sat.pr_sigma) || sat.pr_sigma <= 0.0)) {
      res.unknown_flags[i] = true;
      ++res.n_unknown;
      continue;
    }

    // Away from the measured receiver voxel, the planning-side visibility
    // model is allowed to use only observed online space. A point-cloud miss
    // is not evidence of free space. Receiver-local measured support only
    // proves signal reception; NLOS quality remains represented by the
    // measurement sigma and integrity exclusions in the epoch.
    bool unknown_support = false;
    if (observation_predicate_ && !measured_epoch_support) {
      const double start_offset = std::max(0.0, params_.ray_start_offset);
      const double support_length = std::max(0.0, params_.occ_range);
      constexpr double kSupportStepM = 0.5;
      for (double distance = start_offset;
           distance <= support_length + 1.0e-9;
           distance += kSupportStepM) {
        if (!observation_predicate_(pos_world + distance * dir)) {
          unknown_support = true;
          break;
        }
      }
      res.unknown_flags[i] = unknown_support;
      if (unknown_support) {
        ++res.n_unknown;
        res.vis_flags[i] = false;
        continue;
      }
    }

    // κ and occlusion
    double kappa = 0.0;
    bool blocked = false;
    if (grid_ != nullptr) {
      const double start_offset = std::max(0.0, params_.ray_start_offset);
      const Eigen::Vector3d ray_origin = pos_world + start_offset * dir;
      const double occ_range = std::max(0.0, params_.occ_range - start_offset);
      kappa = grid_->occupancy_ratio(ray_origin, dir, params_.occ_L);
      blocked = params_.hard_occlusion &&
                grid_->ray_occluded(ray_origin, dir, occ_range);
    }

    res.kappas[i]    = kappa;
    res.vis_flags[i] = !blocked;
    if (!blocked) {
      ++res.n_vis;
      kappa_sum += kappa;
    }

    // σ_eff (RQ-314)
    const double canopy_sigma =
        sigma_eff_canopy(params_.canopy, kappa, sat.elevation);
    res.sigma_effs[i] = measured_epoch_support
        ? std::max(sat.pr_sigma, canopy_sigma)
        : canopy_sigma;
  }

  res.mean_kappa = (res.n_vis > 0) ? (kappa_sum / res.n_vis) : 0.0;

  spdlog::trace("[VisibilityPredictor] pos=({:.1f},{:.1f},{:.1f}) "
                "n_sats={} n_above_el={} n_vis={} n_unknown={} "
                "mean_kappa={:.3f}",
                pos_world.x(), pos_world.y(), pos_world.z(),
                N, n_above_el, res.n_vis, res.n_unknown, res.mean_kappa);

  return res;
}

}  // namespace iap
