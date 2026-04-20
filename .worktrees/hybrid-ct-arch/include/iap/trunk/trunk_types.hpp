#pragma once
// IAP-RQ-100: Trunk landmark types
// IAP-RQ-120: TDOP metric type

#include <Eigen/Core>
#include <cmath>
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
  double tdop_weighted = 1e9;  ///< confidence-weighted TDOP (IAP-RQ-133): W = diag(conf²), TDOP_W = sqrt(trace((G^T W G)^{-1}))
  double lambda_min_H = 0.0;  ///< smallest eigenvalue of G^T G (degenerate when ≈ 0)

  // ---- Azimuth histogram (IAP-RQ-150: φ_t for RL state) -----------------
  std::vector<double> azimuth_histogram;  ///< φ_t[s]: min trunk distance per sector
};

// ---------------------------------------------------------------------------
// Azimuth histogram helpers (IAP-RQ-150)
// ---------------------------------------------------------------------------

/// @brief Parameters for azimuth histogram computation.
struct AzimuthHistogramParams {
  int    n_sectors  = 36;    ///< number of angular sectors (36 = 10° each)
  double max_range  = 20.0;  ///< fill value for empty sectors [m]
  bool   normalize  = false; ///< if true, normalize to [0,1] (divide by max_range)
};

/// @brief Compute azimuth histogram φ_t from trunk observations.
///
/// For each sector s ∈ [0, n_sectors), φ_t[s] = min distance to any trunk
/// whose bearing from receiver_xy falls in that sector.  Empty sectors are
/// filled with max_range.
///
/// @param trunks      Detected trunk observations (sensor-frame centres)
/// @param receiver_xy Receiver 2D position (typically zero if trunks are in sensor frame)
/// @param params      Histogram parameters
/// @return Vector of size n_sectors with per-sector minimum distances.
inline std::vector<double> compute_azimuth_histogram(
    const std::vector<TrunkObservation>& trunks,
    const Eigen::Vector2d& receiver_xy,
    const AzimuthHistogramParams& params = AzimuthHistogramParams{}) {
  const int N = params.n_sectors;
  const double sector_width = 2.0 * M_PI / N;  // radians per sector
  std::vector<double> phi(N, params.max_range);

  for (const auto& t : trunks) {
    const Eigen::Vector2d delta = t.center_xy - receiver_xy;
    const double dist = delta.norm();
    if (dist < 1e-6) continue;

    // Azimuth: atan2(y, x) → [−π, π], map to [0, 2π)
    double az = std::atan2(delta.y(), delta.x());
    if (az < 0.0) az += 2.0 * M_PI;

    int sector = static_cast<int>(std::floor(az / sector_width));
    if (sector < 0) sector = 0;
    if (sector >= N) sector = N - 1;

    phi[sector] = std::min(phi[sector], dist);
  }

  if (params.normalize) {
    for (auto& v : phi) v /= params.max_range;
  }

  return phi;
}

}  // namespace iap
