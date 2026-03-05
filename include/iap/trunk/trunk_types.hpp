#pragma once
// IAP-RQ-100: Trunk landmark types
// IAP-RQ-120: TDOP metric type

#include <Eigen/Core>
#include <vector>

namespace iap {

/// @brief Single detected tree-trunk cylinder (2D circle in XY plane).
///
/// Populated by TrunkDetector after fitting a circle to clustered LiDAR returns.
struct TrunkObservation {
  // ---- Geometry -----------------------------------------------------------
  Eigen::Vector2d center_xy = Eigen::Vector2d::Zero();  ///< circle centre (receiver frame, XY) [m]
  double radius              = 0.0;   ///< fitted circle radius [m]
  double z_min               = 0.0;   ///< lowest point in cluster [m]
  double z_max               = 0.0;   ///< highest point in cluster [m]

  // ---- Confidence / quality -----------------------------------------------
  int    num_points          = 0;     ///< number of LiDAR returns in cluster
  double inlier_fraction     = 0.0;  ///< fraction of points within fit_tol of circle [0,1]
  double confidence          = 0.0;  ///< overall detection confidence [0,1] (IAP-RQ-100)
  ///< confidence ≈ inlier_fraction * f(aspect, n_points, radius)

  // ---- Bearing (for TDOP, IAP-RQ-120) ------------------------------------
  Eigen::Vector2d bearing_xy = Eigen::Vector2d::Zero();  ///< unit vector from receiver to trunk centre

  // ---- Fault prior placeholder (for ARAIM, IAP-RQ-230) ------------------
  double p_fault = 1e-4;  ///< prior probability that this trunk detection is faulty
};

/// @brief Full detection result for one LiDAR frame.
struct TrunkDetectionResult {
  double stamp               = 0.0;  ///< frame timestamp [s]
  std::vector<TrunkObservation> trunks;  ///< detected trunks

  // ---- TDOP metrics (IAP-RQ-120) ----------------------------------------
  double tdop  = 1e9;   ///< Tree DOP (= sqrt(trace(H^{-1})));  lower = better geometry
  double tdop2 = 1e9;   ///< TDOP considering only horizontal (2D: x,y)
  double lambda_min_H = 0.0;  ///< smallest eigenvalue of G^T G (degenerate when ≈ 0)
};

}  // namespace iap
