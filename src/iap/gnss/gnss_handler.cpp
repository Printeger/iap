// IAP-RQ-020: GnssHandler implementation

#include <iap/gnss/gnss_handler.hpp>
#include <iap/gnss/constellation_clock.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <set>

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::E;  // ECEF origin of world frame  E(0)
using gtsam::symbol_shorthand::R;  // world→ECEF rotation         R(0)

namespace iap {

Eigen::MatrixXd gnss_postopt_joint_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    const gtsam::KeyVector& keys, const std::vector<std::uint32_t>& dimensions) {
  if (keys.empty() || keys.size()!=dimensions.size() || keys.size()>9 ||
      std::set<gtsam::Key>(keys.begin(),keys.end()).size()!=keys.size())
    throw std::invalid_argument("invalid postopt joint keys");
  std::uint32_t size=0;
  for (auto dimension:dimensions) {
    if (dimension==0 || dimension>6) throw std::invalid_argument("invalid postopt block dimension");
    size+=dimension;
  }
  if (size>29) throw std::invalid_argument("postopt joint dimension exceeds model");
  const auto marginal=linear_graph.marginal(keys);
  const auto actual_dimensions=marginal->getKeyDimMap();
  for(std::size_t i=0;i<keys.size();++i) {
    const auto found=actual_dimensions.find(keys[i]);
    if(found==actual_dimensions.end() || found->second!=dimensions[i])
      throw std::runtime_error("postopt joint block dimension mismatch");
  }
  const auto information=marginal->hessian(gtsam::Ordering(keys.begin(),keys.end())).first;
  if (information.rows()!=size || information.cols()!=size || !information.allFinite() ||
      (information-information.transpose()).norm()>1e-9*std::max(1.,information.norm()))
    throw std::runtime_error("postopt joint information unavailable");
  const Eigen::LDLT<Eigen::MatrixXd> factorization(information);
  if (factorization.info()!=Eigen::Success || factorization.vectorD().minCoeff()<=0.)
    throw std::runtime_error("postopt joint information not positive definite");
  const Eigen::MatrixXd covariance=factorization.solve(Eigen::MatrixXd::Identity(size,size));
  if (!covariance.allFinite()) throw std::runtime_error("postopt joint covariance nonfinite");
  return covariance;
}

Eigen::Matrix2d gnss_clock_difference_covariance(
    const gtsam::GaussianFactorGraph& linear_graph,
    gtsam::Key reference, gtsam::Key system) {
  if (reference == system) throw std::invalid_argument("clock difference requires distinct states");
  const auto marginal = linear_graph.marginal(gtsam::KeyVector{reference, system});
  const auto information = marginal->hessian(gtsam::Ordering{reference, system}).first;
  if (information.rows() != 4 || !information.allFinite()) {
    throw std::runtime_error("clock joint information unavailable");
  }
  const Eigen::LDLT<Eigen::Matrix4d> factorization(information);
  if (factorization.info() != Eigen::Success || factorization.vectorD().minCoeff() <= 0.0) {
    throw std::runtime_error("clock joint information not positive definite");
  }
  const Eigen::Matrix4d covariance = factorization.solve(Eigen::Matrix4d::Identity());
  Eigen::Matrix<double, 2, 4> difference;
  difference << -Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d result = difference * covariance * difference.transpose();
  if (!result.allFinite()) throw std::runtime_error("clock difference covariance nonfinite");
  return result;
}

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

double GnssHandler::pr_sigma(double elevation, double kappa) const {
  const double s = std::sin(std::max(elevation, params_.min_elevation));
  const double base_sigma = params_.pr_noise_base / std::pow(s, params_.elev_noise_exp);

  // Open-sky epochs should be controlled by the experiment's receiver noise
  // setting. The full canopy model intentionally inflates obstructed LOS, but
  // its canopy floor is too conservative for nominal open-sky validation.
  if (kappa <= 1.0e-6) {
    return std::max(base_sigma, 0.05);
  }

  return std::max(base_sigma, sigma_eff_canopy(params_.canopy, kappa, elevation));
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

  // Bind at most one receiver epoch to a state. Multiple observations from
  // distinct times constrain distinct clock/velocity states and must never be
  // collapsed onto one LiDAR frame during startup backlog recovery.
  std::vector<GnssEpoch> matched;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto& q = epoch_queue_;
    const double oldest_allowed = frame_stamp - params_.time_tolerance;
    auto it = q.begin();
    while (it != q.end()) {
      if (it->stamp < oldest_allowed) {
        it = q.erase(it);  // too old — discard
      } else {
        ++it;
      }
    }

    auto nearest = q.end();
    double nearest_dt = params_.time_tolerance;
    for (it = q.begin(); it != q.end(); ++it) {
      const double dt = std::abs(it->stamp - frame_stamp);
      if (dt <= nearest_dt) {
        nearest = it;
        nearest_dt = dt;
      }
    }
    if (nearest != q.end()) {
      const double selected_stamp = nearest->stamp;
      matched.push_back(std::move(*nearest));
      // The selected epoch supersedes every older buffered epoch. Preserve
      // later epochs, including those also inside this frame's tolerance, for
      // their own future state.
      q.erase(std::remove_if(q.begin(), q.end(), [selected_stamp](const GnssEpoch& epoch) {
        return epoch.stamp <= selected_stamp;
      }), q.end());
    }
  }

  if (out_epochs) {
    *out_epochs = matched;
  }

  gtsam::NonlinearFactorGraph graph;

  for (const auto& epoch : matched) {
    for (const auto& sat : epoch.sats) {
      if (sat.excluded || sat.elevation < params_.min_elevation) continue;
      const auto clock_key = gnss_clock_key(sat.constellation, frame_idx);

      // ── PseudorangeFactor ────────────────────────────────────────────────
      // Keys: X(i), constellation receiver clock(i), E(0), R(0)
      const double sigma_pr = pr_sigma(sat.elevation, sat.kappa);
      graph.emplace_shared<PseudorangeFactor>(
        X(frame_idx), clock_key, E(0), R(0),
        sat.pr_meas,
        sat.sat_pos,
        sat.tgd,
        epoch.gps_sec,
        epoch.iono_params,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_pr),
        params_.lever_arm,
        sat.sat_id,
        sat.constellation,
        sat.elevation);

      // ── DopplerFactor ──────────────────────────────────────────────────
      // Keys: X(i), V(i), constellation receiver clock(i), R(0)
      const double sigma_dop = dop_sigma(sat.elevation);
      graph.emplace_shared<DopplerFactor>(
        X(frame_idx), V(frame_idx), clock_key, R(0),
        sat.dop_meas,
        sat.sat_pos,
        sat.sat_vel,
        anc_ecef,
        gtsam::noiseModel::Isotropic::Sigma(1, sigma_dop),
        sat.sat_id,
        sat.constellation,
        sat.elevation);
    }
  }

  return graph;
}

}  // namespace iap
