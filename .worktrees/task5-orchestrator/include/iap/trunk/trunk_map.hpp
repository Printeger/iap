#pragma once
// IAP-RQ-131: Trunk data association & persistent landmark IDs
// §2.6: Trunk EKF — track per-landmark 2D position covariance

#include <iap/trunk/trunk_types.hpp>
#include <Eigen/Core>
#include <vector>
#include <unordered_map>

namespace iap {

/// @brief One entry in the persistent trunk map.
/// §2.6: Now tracks a 2×2 covariance for EKF-style position update.
struct TrunkLandmark {
  int             id          = -1;  ///< unique landmark ID
  Eigen::Vector2d center_xy   = Eigen::Vector2d::Zero(); ///< best-estimate centre in world XY [m]
  double          radius      = 0.0; ///< best-estimate radius [m]
  double          confidence  = 0.0; ///< running-average confidence
  int             seen_count  = 0;   ///< number of times associated
  double          last_stamp  = 0.0; ///< timestamp of last observation [s]

  // ── §2.6: EKF state ──
  /// 2×2 position covariance (world frame) — initialized to σ²_init * I₂
  Eigen::Matrix2d P = Eigen::Matrix2d::Identity() * 1.0;
  /// True if this landmark has been confirmed (seen_count >= threshold)
  bool confirmed = false;
  /// True if this landmark is active in the current smoother window
  bool active    = true;
};

/**
 * @brief Maintains a local map of trunk landmarks and performs
 *        nearest-neighbour data association.
 *
 * ### Association rule (IAP-RQ-131)
 * A detection is associated to an existing landmark if both:
 *  - dist(detected_center, landmark_center) ≤ assoc_gate_m
 *  - |detected_radius - landmark_radius| ≤ assoc_radius_ratio * landmark_radius
 *
 * Unmatched detections spawn new landmarks.
 * Landmarks not seen for `stale_timeout_s` seconds are pruned on `update()`.
 *
 * ### Usage
 * @code
 *   TrunkMap map;
 *   auto assoc = map.update(detection_result, sensor_xy);
 *   // assoc[i] = {landmark_id, obs}
 * @endcode
 */
class TrunkMap {
 public:
  struct Params {
    double assoc_gate_m        = 0.30;  ///< max centroid distance for association [m]
    double assoc_radius_ratio  = 0.50;  ///< max relative radius difference for association
    int    min_confirm_count   = 2;     ///< minimum sightings to "confirm" a landmark
    double stale_timeout_s     = 5.0;   ///< seconds without sighting before pruning
    double ema_alpha           = 0.3;   ///< EMA smoothing weight for position update

    // ── §2.6: EKF parameters ──
    double sigma_init          = 1.0;   ///< initial position std σ₀ for new landmarks [m]
    double sigma_obs           = 0.15;  ///< observation noise σ_obs (range/bearing → XY) [m]
    double sigma_process       = 0.01;  ///< process noise per second (drift model) [m/s]
    bool   use_ekf             = true;  ///< if false, fall back to EMA update
  };

  TrunkMap();
  explicit TrunkMap(const Params& p);

  /**
   * @brief Update the trunk map with a new detection result.
   *
   * @param det        Detection result from TrunkDetector (current frame)
   * @param sensor_xy  Sensor 2D position in world frame [m] (for future transform;
   *                   detections are already expressed in sensor frame so sensor_xy=0
   *                   works for a static receiver).
   * @return List of (landmark_id, TrunkObservation) pairs — one per detection.
   *         landmark_id == -1 for newly spawned landmarks.
   */
  std::vector<std::pair<int, TrunkObservation>> update(
      const TrunkDetectionResult& det,
      const Eigen::Vector2d& sensor_xy = Eigen::Vector2d::Zero());

  /// Read-only access to all landmarks (confirmed + unconfirmed).
  const std::vector<TrunkLandmark>& landmarks() const { return landmarks_; }

  /// Only confirmed landmarks (seen_count >= min_confirm_count).
  std::vector<const TrunkLandmark*> confirmed_landmarks() const;

  /// Current number of tracked landmarks.
  std::size_t size() const { return landmarks_.size(); }

  void reset() { landmarks_.clear(); next_id_ = 0; }

  const Params& params() const { return params_; }

 private:
  /// Try to associate one observation to an existing landmark.
  /// Returns index into landmarks_ or -1 if no match.
  int find_association(const TrunkObservation& obs,
                       const Eigen::Vector2d& sensor_xy) const;

  Params                  params_;
  std::vector<TrunkLandmark> landmarks_;
  int                     next_id_ = 0;
};

}  // namespace iap
