#pragma once
// IAP-RQ-312: Predict satellite visibility set V̂(τ) by ray casting
// IAP-RQ-313: Estimate canopy density κ along LOS

#include <iap/map/local_occupancy.hpp>
#include <iap/map/trusted_local_map_support.hpp>
#include <iap/gnss/gnss_types.hpp>
#include <iap/gnss/canopy_noise_model.hpp>
#include <Eigen/Core>
#include <functional>
#include <limits>
#include <vector>
#include <cmath>

namespace iap {

/// @brief Per-epoch visibility prediction result (IAP-RQ-312/313).
struct VisibilityResult {
  int                  n_vis      = 0;   ///< number of visible (unblocked) satellites
  std::vector<bool>    vis_flags;        ///< visibility flag per satellite
  std::vector<double>  kappas;           ///< κ per satellite (occupancy_ratio along LOS)
  std::vector<double>  sigma_effs;       ///< predicted σ_eff per satellite (IAP-RQ-314)
  std::vector<bool>    unknown_flags;    ///< LOS support contains unobserved space
  std::vector<bool>    known_flags;      ///< LOS support required by the model is observed
  std::vector<bool>    blocked_flags;    ///< hard-occlusion evidence per satellite
  std::vector<std::size_t> support_sample_counts;
  std::vector<std::size_t> support_covered_sample_counts;
  std::vector<double> unknown_support_fractions;
  std::vector<double> first_missing_support_distances_m;
  std::vector<LocalMapSupportStatus> first_missing_support_statuses;
  int                  n_unknown  = 0;   ///< satellites rejected for unknown LOS support
  int                  n_known    = 0;   ///< satellites with complete online LOS support
  int                  n_blocked  = 0;   ///< satellites rejected by hard occlusion
  double               mean_kappa = 0.0; ///< mean κ over visible satellites
  LocalMapSupportAuthority support_authority =
      LocalMapSupportAuthority::STRICT_OBSERVATION;
  LocalMapSupportStatus support_status =
      LocalMapSupportStatus::MODEL_COMPLETE;
};

/**
 * @brief Predicts GNSS satellite visibility and canopy density along LOS.
 *
 * ### Visibility (IAP-RQ-312)
 * For each satellite, convert (elevation, azimuth) to a local ENU unit vector:
 * @code
 *   d = [cos(el)*sin(az), cos(el)*cos(az), sin(el)]   (ENU/map)
 * @endcode
 * where azimuth is clockwise from north (`azimuth=0 -> +Y`). The current
 * planner frame contract is translation-only relative to ENU.
 * Then check whether the ray from the waypoint in direction d is occluded
 * within `occ_range` metres when hard occlusion is enabled.  The soft canopy
 * model requires online observation only across the `occ_L` interval that
 * actually contributes to κ.
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
    double min_elevation = 0.1745;  ///< elevation mask [rad] (~10 deg)
    double occ_range     = 20.0;   ///< max ray length for occlusion check [m]
    double occ_L         = 5.0;    ///< survey length for κ computation [m]
    double ray_start_offset = 1.0; ///< ignore near-field voxels around query point [m]
    bool hard_occlusion = false;   ///< false keeps satellites usable and inflates σ via κ
    double clearance_transition_m = 0.0;
                                   ///< continuous LOS sigma transition width [m]
    CanopyNoiseParams canopy;      ///< σ_eff model params (σ_0, σ_mp, σ_c, α)
  };

  VisibilityPredictor();
  explicit VisibilityPredictor(const Params& p);

  /// @brief Provide a (possibly shared) occupancy grid for ray checks.
  ///        Pass nullptr to disable occlusion checks (open-sky assumption).
  void set_occupancy(const LocalOccupancyGrid* grid);

  /// @brief Provide the immutable online-map support test used by this query.
  /// A false result means UNKNOWN, not free. When configured, every sampled
  /// LOS point must be observed before a satellite may be treated as visible.
  using ObservationPredicate =
      std::function<bool(const Eigen::Vector3d& position_world)>;
  void set_observation_predicate(ObservationPredicate predicate);
  using SupportQuery =
      std::function<LocalMapSupportQuery(
          const Eigen::Vector3d& position_world, double evaluation_time_s,
          double query_time_s)>;
  void set_support_query(SupportQuery query);

  /**
   * @brief Predict visibility + κ for all satellites at a given position.
   *
   * @param pos_world  Waypoint in world frame (ENU origin assumed = (0,0,0))
   * @param epoch      GNSS epoch with per-satellite elevation + azimuth
   * @return VisibilityResult with n_vis, vis_flags, kappas, sigma_effs, mean_kappa
   */
  VisibilityResult predict(const Eigen::Vector3d& pos_world,
                           const GnssEpoch& epoch,
                           bool measured_epoch_support = false,
                           bool retain_unknown_support = false,
                           double query_time_s =
                               std::numeric_limits<double>::quiet_NaN(),
                           double evaluation_time_s =
                               std::numeric_limits<double>::quiet_NaN()) const;

  const Params& params() const { return params_; }

 private:
  /// Convert elevation+azimuth (ENU) to unit direction vector.
  static Eigen::Vector3d enu_dir(double elevation, double azimuth);

  Params                      params_;
  const LocalOccupancyGrid*   grid_ = nullptr;
  ObservationPredicate       observation_predicate_;
  SupportQuery               support_query_;
};

}  // namespace iap
