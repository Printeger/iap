#pragma once
// IAP-RQ-100: Trunk detection & parameterization
// IAP-RQ-110: Trunk health/noise inflation interface (Baseline-A)
// IAP-RQ-120: TDOP metric computation

#include <iap/trunk/trunk_types.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <memory>

namespace iap {

/**
 * @brief Detects tree trunks from a LiDAR scan and computes geometric metrics.
 *
 * Algorithm (IAP-RQ-100):
 *  1. Height-band filter: retain points in [z_min, z_max] relative to sensor.
 *  2. Range filter: discard points inside min_range or beyond max_range.
 *  3. 2-D grid clustering (XY plane) using a fixed resolution grid cell merging.
 *  4. Kasa closed-form circle fitting per cluster.
 *  5. Confidence = inlier_fraction × sigmoid(n_points) × radius_penalty.
 *
 * TDOP computation (IAP-RQ-120):
 *  - Build design matrix G (K×2): each row = unit bearing to trunk k.
 *  - H = G^T G  (2×2 information matrix).
 *  - TDOP = sqrt(trace(inv(H))).  Lower TDOP → better geometric diversity.
 *
 * Trunk health interface (IAP-RQ-110, Baseline-A):
 *  - `health_factor(result)`: scalar ∈ [0,1], where 1=healthy, <1=degraded.
 *    Users can multiply LiDAR noise by 1/health_factor.
 */
class TrunkDetector {
 public:
  struct Params {
    // Height filter
    double trunk_z_min      = 0.3;   ///< min height above sensor [m]
    double trunk_z_max      = 2.5;   ///< max height above sensor [m]
    double trunk_range_min  = 0.5;   ///< min horizontal range [m]
    double trunk_range_max  = 20.0;  ///< max horizontal range [m]

    // Clustering
    double grid_resolution  = 0.1;   ///< XY grid cell size for clustering [m]
    int    min_cluster_pts  = 5;     ///< minimum points per cluster
    int    max_cluster_pts  = 2000;  ///< maximum points per cluster (clutter guard)

    // Circle fitting / validation
    double radius_min        = 0.03;  ///< minimum plausible trunk radius [m]
    double radius_max        = 0.40;  ///< maximum plausible trunk radius [m]
    double fit_tolerance     = 0.05;  ///< inlier distance tolerance for Kasa fit [m]
    double min_confidence    = 0.3;   ///< discard detections below this confidence

    // TDOP
    double tdop_inf          = 1e9;   ///< TDOP value when < 2 trunks detected
  };

  TrunkDetector();
  explicit TrunkDetector(const Params& params);

  /**
   * @brief Run trunk detection on a single LiDAR frame.
   * @param frame   Input point cloud (sensor frame)
   * @param stamp   Frame timestamp [s]
   * @return TrunkDetectionResult with detected trunks and TDOP metrics
   */
  TrunkDetectionResult detect(const gtsam_points::PointCloud& frame, double stamp) const;

  /**
   * @brief Compute a scalar health factor (IAP-RQ-110, Baseline-A).
   *
   * health ∈ [0,1]: 1 = geometry excellent; <1 = degenerate/few trunks.
   * In Baseline-A the caller uses health to adjust LiDAR noise (not injected into FGO).
   */
  static double health_factor(const TrunkDetectionResult& result);

  const Params& params() const { return params_; }

 private:
  /// Kasa closed-form circle fitting. Returns {cx, cy, r, inlier_frac}.
  std::tuple<double, double, double, double> kasa_fit(
    const std::vector<Eigen::Vector2d>& pts) const;

  /// Compute TDOP from detected trunks.
  void compute_tdop(TrunkDetectionResult& result) const;

  Params params_;
};

}  // namespace iap
