// IAP-RQ-300: FGO Information Matrix extraction
// Hooks into on_smoother_update_finish to extract Σ^(0)_{p,p} from iSAM2.

#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/gnss/clock_between_factor.hpp>
#include <iap/gnss/doppler_factor.hpp>
#include <iap/gnss/pseudorange_factor.hpp>
#include <iap/trunk/trunk_factor.hpp>
#include <iap/util/logging.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam_points/factors/reintegrated_imu_factor.hpp>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <set>

using gtsam::symbol_shorthand::X;

namespace iap {

// ---------------------------------------------------------------------------
FGOInformationManager::FGOInformationManager() : params_{} {
  logger_ = glim::create_module_logger("fgo_info");
}

FGOInformationManager::FGOInformationManager(const Params& params)
: params_(params) {
  logger_ = glim::create_module_logger("fgo_info");
}

// ---------------------------------------------------------------------------
void FGOInformationManager::extract(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
    long frame_id,
    double stamp) {

  FGOPositionInfo info;
  info.stamp = stamp;
  info.frame_id = frame_id;

  try {
    const gtsam::Pose3 pose = smoother.calculateEstimate<gtsam::Pose3>(X(frame_id));
    info.p_world = pose.translation();

    // Extract 6×6 marginal covariance for Pose3: [rotation(3) | translation(3)]
    const gtsam::Matrix pose_cov = smoother.marginalCovariance(X(frame_id));

    info.pose_cov_6x6 = pose_cov;
    // Position block is the lower-right 3×3
    info.sigma_p = pose_cov.block<3, 3>(3, 3);
    info.pose_cov_valid = true;

    // Validate: check eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(info.sigma_p);
    if (eig.eigenvalues().minCoeff() < params_.min_eigenvalue) {
      logger_->warn("[fgo_info] Near-singular Σ_p at frame {} (min_eig={:.2e})",
                    frame_id, eig.eigenvalues().minCoeff());
      // Still use it but flag
    }

    info.eig_vals = eig.eigenvalues();
    info.sigma_E = std::sqrt(std::max(0.0, info.sigma_p(0, 0)));
    info.sigma_N = std::sqrt(std::max(0.0, info.sigma_p(1, 1)));
    info.sigma_U = std::sqrt(std::max(0.0, info.sigma_p(2, 2)));

    // Information matrix Λ = Σ^{-1}
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    info.lambda_p = info.sigma_p.llt().solve(I3);

    info.valid = true;

    logger_->trace("[fgo_info] frame={} σ_E={:.4f} σ_N={:.4f} σ_U={:.4f} "
                   "eig=[{:.4f},{:.4f},{:.4f}]",
                   frame_id, info.sigma_E, info.sigma_N, info.sigma_U,
                   info.eig_vals(0), info.eig_vals(1), info.eig_vals(2));

  } catch (const std::exception& e) {
    logger_->warn("[fgo_info] marginalCovariance(X({})) failed: {}",
                  frame_id, e.what());
    info.valid = false;
  }

  // Factor counting (optional, for diagnostics)
  if (params_.count_factors) {
    try {
      std::set<int> gnss_sat_ids;
      std::set<char> gnss_constellations;
      std::set<int> trunk_landmark_ids;
      std::set<std::string> factor_tags;
      std::set<gtsam::Key> window_keys;

      const auto& factors = smoother.getFactors();
      for (const auto& factor : factors) {
        if (!factor) continue;

        ++info.n_total_factors;
        for (gtsam::Key key : factor->keys()) {
          window_keys.insert(key);
        }

        if (const auto* pr = dynamic_cast<const PseudorangeFactor*>(factor.get())) {
          ++info.n_gnss_factors;
          gnss_sat_ids.insert(pr->sat_id());
          gnss_constellations.insert(pr->constellation());
          factor_tags.insert("PseudorangeFactor");
          continue;
        }

        if (const auto* dop = dynamic_cast<const DopplerFactor*>(factor.get())) {
          ++info.n_gnss_factors;
          gnss_sat_ids.insert(dop->sat_id());
          gnss_constellations.insert(dop->constellation());
          factor_tags.insert("DopplerFactor");
          continue;
        }

        if (dynamic_cast<const ClockBetweenFactor*>(factor.get())) {
          ++info.n_clock_factors;
          factor_tags.insert("ClockBetweenFactor");
          continue;
        }

        if (dynamic_cast<const gtsam::ImuFactor*>(factor.get()) ||
            dynamic_cast<const gtsam_points::ReintegratedImuFactor*>(factor.get())) {
          ++info.n_imu_factors;
          factor_tags.insert("ImuFactor");
          continue;
        }

        if (const auto* trunk = dynamic_cast<const TrunkFactor*>(factor.get())) {
          ++info.n_trunk_factors;
          factor_tags.insert("TrunkFactor");
          const auto& keys = trunk->keys();
          if (keys.size() >= 2) {
            const gtsam::Symbol lm_symbol(keys[1]);
            if (lm_symbol.chr() == 'l') {
              trunk_landmark_ids.insert(static_cast<int>(lm_symbol.index()));
            }
          }
          continue;
        }

        ++info.n_other_factors;
      }

      info.window_key_count = static_cast<int>(window_keys.size());
      info.gnss_sat_ids.assign(gnss_sat_ids.begin(), gnss_sat_ids.end());
      info.gnss_constellations.assign(gnss_constellations.begin(), gnss_constellations.end());
      info.trunk_landmark_ids.assign(trunk_landmark_ids.begin(), trunk_landmark_ids.end());
      info.factor_type_tags.assign(factor_tags.begin(), factor_tags.end());

      logger_->trace(
          "[fgo_info] frame={} factors total={} gnss={} trunk={} imu={} clock={} other={} keys={}",
          frame_id, info.n_total_factors, info.n_gnss_factors,
          info.n_trunk_factors, info.n_imu_factors, info.n_clock_factors,
          info.n_other_factors, info.window_key_count);
    } catch (...) {}
  }

  // Thread-safe store
  {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_info_ = info;
  }
}

// ---------------------------------------------------------------------------
FGOPositionInfo FGOInformationManager::latest() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return latest_info_;
}

// ---------------------------------------------------------------------------
bool FGOInformationManager::has_data() const {
  std::lock_guard<std::mutex> lk(mutex_);
  return latest_info_.valid;
}

}  // namespace iap
