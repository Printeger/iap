#pragma once
// IAP-RQ-312: Predict satellite visibility set V̂(τ) by ray casting
// IAP-RQ-313: Estimate canopy density κ along LOS

#include <iap/map/local_occupancy.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/canopy_noise_model.hpp>
#include <Eigen/Core>
#include <vector>
#include <cmath>

namespace iap {

/// @brief Per-epoch visibility prediction result (IAP-RQ-312/313).
struct VisibilityResult {
  int                  n_vis      = 0;   ///< number of visible (unblocked) satellites
  std::vector<bool>    vis_flags;        ///< visibility flag per satellite
  std::vector<double>  kappas;           ///< κ per satellite (occupancy_ratio along LOS)
  std::vector<double>  sigma_effs;       ///< predicted σ_eff per satellite (IAP-RQ-314)
  double               mean_kappa = 0.0; ///< mean κ over visible satellites
};

/**
 * @brief Predicts GNSS satellite visibility and canopy density along LOS.
 *
 * ### Visibility (IAP-RQ-312)
 * For each satellite, convert (elevation, azimuth) to a local ENU unit vector:
 * @code
 *   d = [cos(el)*cos(az), cos(el)*sin(az), sin(el)]   (ENU)
 * @endcode
 * Then check whether the ray from the waypoint in direction d is occluded
 * within `occ_range` metres using the LocalOccupancyGrid.
 *
 * ### Canopy density κ (IAP-RQ-313)
 * κ = occupancy_ratio(origin, d, occ_L) — fraction of `n_kappa_steps`
 * probe points along the first `occ_L` metres of the LOS that fall in
 * occupied voxels.  κ ∈ [0, 1].
 *
 * ### σ_eff (IAP-RQ-314)
 * @code
 *   σ_eff = σ_c · exp(0.5 · α · κ / sin(el))
 * @endcode
 *
 * If no `LocalOccupancyGrid` is set (nullptr), all satellites are treated
 * as visible with κ = 0 (open sky — conservative fallback).
 */
class VisibilityPredictor {
 public:
  struct Params {
    double min_elevation = 0.087;  ///< elevation mask [rad] (~5 deg)
    double occ_range     = 20.0;   ///< max ray length for occlusion check [m]
    double occ_L         = 5.0;    ///< survey length for κ computation [m]
    CanopyNoiseParams canopy;      ///< σ_eff model params (σ_c, α)
  };

  VisibilityPredictor();
  explicit VisibilityPredictor(const Params& p);

  /// @brief Provide a (possibly shared) occupancy grid for ray checks.
  ///        Pass nullptr to disable occlusion checks (open-sky assumption).
  void set_occupancy(const LocalOccupancyGrid* grid);

  /**
   * @brief Predict visibility + κ for all satellites at a given position.
   *
   * @param pos_world  Waypoint in world frame (ENU origin assumed = (0,0,0))
   * @param epoch      GNSS epoch with per-satellite elevation + azimuth
   * @return VisibilityResult with n_vis, vis_flags, kappas, sigma_effs, mean_kappa
   */
  VisibilityResult predict(const Eigen::Vector3d& pos_world,
                           const GnssEpoch& epoch) const;

  const Params& params() const { return params_; }

 private:
  /// Convert elevation+azimuth (ENU) to unit direction vector.
  static Eigen::Vector3d enu_dir(double elevation, double azimuth);

  Params                      params_;
  const LocalOccupancyGrid*   grid_ = nullptr;
};

}  // namespace iap
