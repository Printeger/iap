// IAP-RQ-020: GnssHandler implementation

#include <iap/gnss/gnss_handler.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <algorithm>
#include <cmath>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::E;  // ECEF origin of world frame  E(0)
using gtsam::symbol_shorthand::R;  // world→ECEF rotation         R(0)

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
    int                     frame_idx,
    double                  frame_stamp,
    const Eigen::Vector3d&  anc_ecef,
    std::vector<GnssEpoch>* out_epochs) {

  // Drain matching epochs from the queue
  std::vector<GnssEpoch> matched;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto& q = epoch_queue_;
    auto it = q.begin();
    while (it != q.end()) {
      if (std::abs(it->stamp - frame_stamp) <= params_.time_tolerance) {
        matched.push_back(std::move(*it));
        it = q.erase(it);
      } else if (it->stamp < frame_stamp - params_.time_tolerance) {
        it = q.erase(it);  // too old — discard
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
      if (sat.excluded || sat.elevation < params_.min_elevation) continue;

      // ── PseudorangeFactor ────────────────────────────────────────────────
      // Keys: X(i), C(i), E(0), R(0)
      const double sigma_pr = pr_sigma(sat.elevation);
      graph.emplace_shared<PseudorangeFactor>(
        X(frame_idx), C(frame_idx), E(0), R(0),
        sat.pr_meas,
        sat.sat_pos,
        sat.tgd,
        epoch.gps_sec,
        epoch.iono_params,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_pr));

      // ── DopplerFactor ──────────────────────────────────────────────────
      // Keys: X(i), V(i), C(i), R(0)
      const double sigma_dop = dop_sigma(sat.elevation);
      graph.emplace_shared<DopplerFactor>(
        X(frame_idx), V(frame_idx), C(frame_idx), R(0),
        sat.dop_meas,
        sat.sat_pos,
        sat.sat_vel,
        anc_ecef,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dop));
    }
  }

  return graph;
}

}  // namespace iap
