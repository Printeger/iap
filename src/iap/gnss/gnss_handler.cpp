// IAP-RQ-020: GnssHandler implementation

#include <iap/gnss/gnss_handler.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <algorithm>
#include <cmath>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::C;

namespace iap {

GnssHandler::GnssHandler() : params_(Params{}) {}
GnssHandler::GnssHandler(const Params& params) : params_(params) {}

void GnssHandler::insert_epoch(const GnssEpoch& epoch) {
  std::lock_guard<std::mutex> lk(mutex_);
  if (static_cast<int>(epoch_queue_.size()) >= params_.max_epoch_queue) {
    epoch_queue_.pop_front();  // back-pressure: drop oldest
  }
  epoch_queue_.push_back(epoch);
}

std::size_t GnssHandler::queue_size() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return epoch_queue_.size();
}

double GnssHandler::pr_sigma(double elevation) const {
  // Elevation-dependent noise: sigma = base / sin^exp(el)
  const double s = std::sin(std::max(elevation, params_.min_elevation));
  return params_.pr_noise_base / std::pow(s, params_.elev_noise_exp);
}

double GnssHandler::dop_sigma(double elevation) const {
  const double s = std::sin(std::max(elevation, params_.min_elevation));
  return params_.dop_noise_base / std::pow(s, params_.elev_noise_exp);
}

gtsam::NonlinearFactorGraph GnssHandler::get_factors(
    int    frame_idx,
    double frame_stamp,
    std::vector<GnssEpoch>* out_epochs) {

  // Drain matching epochs from the queue
  std::vector<GnssEpoch> matched;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    // Use a mutable reference trick via const_cast since we need to modify queue
    auto& q = epoch_queue_;
    auto it = q.begin();
    while (it != q.end()) {
      if (std::abs(it->stamp - frame_stamp) <= params_.time_tolerance) {
        matched.push_back(std::move(*it));
        it = q.erase(it);
      } else if (it->stamp < frame_stamp - params_.time_tolerance) {
        // Too old — discard silently
        it = q.erase(it);
      } else {
        ++it;
      }
    }
  }

  if (out_epochs) {
    *out_epochs = matched;
  }

  gtsam::NonlinearFactorGraph graph;

  for (const auto& epoch : matched) {
    for (const auto& sat : epoch.sats) {
      // Skip excluded or below elevation mask
      if (sat.excluded || sat.elevation < params_.min_elevation) {
        continue;
      }

      // IAP-RQ-020: each satellite is an independent observation channel
      // Pseudorange factor — keys: X(i), C(i)
      const double sigma_pr = pr_sigma(sat.elevation);
      graph.emplace_shared<PseudorangeFactor>(
        X(frame_idx), C(frame_idx),
        sat.pr_meas,
        sat.sat_pos,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_pr));

      // Doppler factor — keys: X(i), V(i), C(i)
      const double sigma_dop = dop_sigma(sat.elevation);
      graph.emplace_shared<DopplerFactor>(
        X(frame_idx), V(frame_idx), C(frame_idx),
        sat.dop_meas,
        sat.sat_pos,
        sat.sat_vel,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dop));
    }
  }

  return graph;
}

}  // namespace iap
