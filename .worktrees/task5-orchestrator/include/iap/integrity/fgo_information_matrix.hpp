#pragma once
// IAP-RQ-300: FGO Information Matrix extraction from iSAM2
// §1.9 Step C+: Σ^(0) from factor-graph marginals → feed ARAIM
//
// Extracts the position-block (3×3) of the information (Hessian) matrix
// from the incremental fixed-lag smoother after each optimization cycle.
// This gives us Σ^(0) = Λ^{-1}_{p,p} which is more rigorous than the
// WLS-only S0 = (G^T W G)^{-1} used by ARAIM's own geometry matrix.

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>

// Forward declarations to avoid heavy GTSAM headers in this header
namespace gtsam_points {
class IncrementalFixedLagSmootherExtWithFallback;
}

namespace iap {

// ---------------------------------------------------------------------------
/// @brief Per-epoch snapshot of FGO-derived position information.
struct FGOPositionInfo {
  double stamp          = 0.0;     ///< timestamp of the extraction
  bool   valid          = false;   ///< true if extraction succeeded

  /// Position covariance Σ^(0)_{p,p} (3×3, ENU or world frame)
  Eigen::Matrix3d sigma_p = Eigen::Matrix3d::Identity();

  /// Information matrix block Λ_{p,p} = Σ^{-1}_{p,p}
  Eigen::Matrix3d lambda_p = Eigen::Matrix3d::Zero();

  /// Per-axis position sigmas [m]
  double sigma_E = 1e9;
  double sigma_N = 1e9;
  double sigma_U = 1e9;

  /// Eigenvalues of Σ_p (ascending)
  Eigen::Vector3d eig_vals = Eigen::Vector3d::Constant(1e9);

  /// Contributing factor count (how many GNSS+trunk factors are in the window)
  int n_gnss_factors  = 0;
  int n_trunk_factors = 0;
  int n_imu_factors   = 0;
};

// ---------------------------------------------------------------------------
/// @brief Manages FGO information matrix extraction from the odometry smoother.
///
/// Usage:
///   1. Create instance in the integrity extension module.
///   2. Register the callback: `Callbacks::on_smoother_update_finish.add(...)`.
///   3. In the callback, call `extract(smoother)`.
///   4. In the integrity monitor, call `latest()` to get the most recent snapshot.
class FGOInformationManager {
 public:
  struct Params {
    /// GTSAM key index of the latest pose to extract covariance for.
    /// Set to -1 to auto-detect the most recent X(i) key.
    long target_key_index = -1;

    /// Whether to additionally count factor types in the smoother.
    bool count_factors = true;

    /// Minimum eigenvalue threshold for valid extraction.
    double min_eigenvalue = 1e-12;
  };

  FGOInformationManager();
  explicit FGOInformationManager(const Params& params);

  /**
   * @brief Extract position information from the smoother.
   *
   * Called from `on_smoother_update_finish` callback. Thread-safe.
   *
   * @param smoother  The incremental fixed-lag smoother after optimization
   * @param frame_id  Key index of the latest pose frame
   * @param stamp     Timestamp of the frame
   */
  void extract(gtsam_points::IncrementalFixedLagSmootherExtWithFallback& smoother,
               long frame_id,
               double stamp);

  /// Get the most recent valid extraction result. Thread-safe.
  FGOPositionInfo latest() const;

  /// Whether we have at least one valid extraction.
  bool has_data() const;

 private:
  Params params_;
  FGOPositionInfo latest_info_;
  mutable std::mutex mutex_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace iap
