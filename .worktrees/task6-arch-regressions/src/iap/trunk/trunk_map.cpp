// IAP-RQ-131: TrunkMap data association & persistent landmark IDs
// §2.6: EKF-style position covariance tracking for each landmark
#include <iap/trunk/trunk_map.hpp>
#include <spdlog/spdlog.h>
#include <Eigen/LU>
#include <cmath>
#include <limits>

namespace iap {

TrunkMap::TrunkMap() : params_{} {}
TrunkMap::TrunkMap(const Params& p) : params_{p} {}

// ---------------------------------------------------------------------------
// Private helper
// ---------------------------------------------------------------------------

int TrunkMap::find_association(const TrunkObservation& obs,
                               const Eigen::Vector2d& sensor_xy) const {
  const Eigen::Vector2d obs_xy = sensor_xy + obs.center_xy;

  int    best_idx  = -1;
  double best_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(landmarks_.size()); ++i) {
    const auto& lm = landmarks_[i];

    const double dist = (obs_xy - lm.center_xy).norm();
    if (dist > params_.assoc_gate_m) continue;

    if (lm.radius > 0.0) {
      const double r_diff = std::abs(obs.radius - lm.radius);
      if (r_diff > params_.assoc_radius_ratio * lm.radius) continue;
    }

    if (dist < best_dist) {
      best_dist = dist;
      best_idx  = i;
    }
  }
  return best_idx;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::vector<std::pair<int, TrunkObservation>>
TrunkMap::update(const TrunkDetectionResult& det,
                 const Eigen::Vector2d& sensor_xy) {
  // ---- Prune stale landmarks ----
  if (det.stamp > 0.0) {
    landmarks_.erase(
        std::remove_if(landmarks_.begin(), landmarks_.end(),
                       [&](const TrunkLandmark& lm) {
                         return (det.stamp - lm.last_stamp) > params_.stale_timeout_s;
                       }),
        landmarks_.end());
  }

  std::vector<std::pair<int, TrunkObservation>> result;
  result.reserve(det.trunks.size());

  const Eigen::Matrix2d R_obs = Eigen::Matrix2d::Identity() *
                                 (params_.sigma_obs * params_.sigma_obs);

  for (const auto& obs : det.trunks) {
    int idx = find_association(obs, sensor_xy);

    if (idx < 0) {
      // New landmark — initialize with prior covariance
      TrunkLandmark lm;
      lm.id          = next_id_++;
      lm.center_xy   = sensor_xy + obs.center_xy;
      lm.radius      = obs.radius;
      lm.confidence  = obs.confidence;
      lm.seen_count  = 1;
      lm.last_stamp  = det.stamp;
      lm.confirmed   = false;
      lm.active      = true;
      lm.P           = Eigen::Matrix2d::Identity() *
                        (params_.sigma_init * params_.sigma_init);
      landmarks_.push_back(lm);

      result.emplace_back(-1, obs);  // -1 → newly spawned
    } else {
      // Update existing landmark
      TrunkLandmark& lm = landmarks_[idx];
      const Eigen::Vector2d z = sensor_xy + obs.center_xy;  // observation in world

      if (params_.use_ekf) {
        // §2.6: EKF update
        // State: x = [cx, cy]  (landmark position)
        // H = I₂ (direct observation of position)
        // Innovation: y = z - x̂
        const Eigen::Vector2d y = z - lm.center_xy;

        // Process noise: Q = σ²_proc · dt · I₂
        const double dt = std::max(det.stamp - lm.last_stamp, 0.001);
        const Eigen::Matrix2d Q = Eigen::Matrix2d::Identity() *
                                   (params_.sigma_process * params_.sigma_process * dt);

        // Predict: P⁻ = P + Q
        const Eigen::Matrix2d P_pred = lm.P + Q;

        // Kalman gain: K = P⁻(P⁻ + R)^{-1}
        const Eigen::Matrix2d S = P_pred + R_obs;
        const Eigen::Matrix2d K = P_pred * S.inverse();

        // Update
        lm.center_xy = lm.center_xy + K * y;
        lm.P = (Eigen::Matrix2d::Identity() - K) * P_pred;
      } else {
        // Fallback: simple EMA
        const double a = params_.ema_alpha;
        lm.center_xy = a * z + (1.0 - a) * lm.center_xy;
      }

      // EMA update for radius / confidence (always)
      const double a = params_.ema_alpha;
      lm.radius     = a * obs.radius + (1.0 - a) * lm.radius;
      lm.confidence = a * obs.confidence + (1.0 - a) * lm.confidence;
      lm.seen_count += 1;
      lm.last_stamp  = det.stamp;
      lm.confirmed   = (lm.seen_count >= params_.min_confirm_count);
      lm.active      = true;

      result.emplace_back(lm.id, obs);
    }
  }

  spdlog::trace("[TrunkMap] n_det={} n_landmarks={} stamp={:.3f}",
                det.trunks.size(), landmarks_.size(), det.stamp);

  return result;
}

std::vector<const TrunkLandmark*> TrunkMap::confirmed_landmarks() const {
  std::vector<const TrunkLandmark*> out;
  for (const auto& lm : landmarks_) {
    if (lm.seen_count >= params_.min_confirm_count) {
      out.push_back(&lm);
    }
  }
  return out;
}

}  // namespace iap
