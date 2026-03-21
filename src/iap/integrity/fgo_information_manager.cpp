// IAP-RQ-300: FGO Information Matrix extraction
// Hooks into on_smoother_update_finish to extract Σ^(0)_{p,p} from iSAM2.

#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/util/logging.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <Eigen/Eigenvalues>

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

  try {
    // Extract 6×6 marginal covariance for Pose3: [rotation(3) | translation(3)]
    const gtsam::Matrix pose_cov = smoother.marginalCovariance(X(frame_id));

    // Position block is the lower-right 3×3
    info.sigma_p = pose_cov.block<3, 3>(3, 3);

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
      // Count factors by type in the smoother's factor graph
      // Note: getFactors() returns the underlying factor graph
      // We do a simple heuristic based on factor key count
      info.n_gnss_factors  = 0;
      info.n_trunk_factors = 0;
      info.n_imu_factors   = 0;
      // Detailed factor counting would require iterating getFactors() and
      // checking dynamic_cast — keep as placeholder for now
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
