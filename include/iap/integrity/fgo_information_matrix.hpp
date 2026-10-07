#pragma once
// IAP-RQ-300: FGO Information Matrix extraction from iSAM2
// §1.9 Step C+: Σ^(0) from factor-graph marginals → feed ARAIM
//
// Pushes the Pose3 marginal into world position using its translation
// Jacobian after each fixed-lag optimization cycle. Its inverse is position
// marginal information, not a block of the full pose Hessian. This differs from
// WLS-only S0 = (G^T W G)^{-1} used by ARAIM's own geometry matrix.

#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

// Forward declarations to avoid heavy GTSAM headers in this header
namespace gtsam {
class Pose3;
}
namespace gtsam_points {
class IncrementalFixedLagSmootherExtWithFallback;
}

namespace iap {

// ---------------------------------------------------------------------------
/// @brief Per-epoch snapshot of FGO-derived position information.
struct FGOPositionInfo {
  /// Bind one marginal to a fresh snapshot before extraction derives inverse,
  /// sigmas and validity. Does not grant validity or change the timestamp.
  void bindPoseCovariance(const gtsam::Pose3& pose,
                         const Eigen::Matrix<double, 6, 6>& covariance);

  double stamp          = 0.0;     ///< timestamp of the extraction
  long   frame_id       = -1;      ///< frame id of the extracted pose key
  bool   valid          = false;   ///< true if extraction succeeded
  bool   pose_cov_valid = false;   ///< true if pose_cov_6x6 and sigma_p are valid

  /// Position marginal covariance in estimator world axes (ENU only if aligned).
  Eigen::Matrix3d sigma_p = Eigen::Matrix3d::Identity();

  /// Inverse world position marginal covariance (not the full Hessian block).
  Eigen::Matrix3d lambda_p = Eigen::Matrix3d::Zero();

  /// Unchanged Pose3 right-local tangent marginal [rot(3) | trans(3)], for
  /// consumers with local pose Jacobians; never relabel it as world covariance.
  Eigen::Matrix<double, 6, 6> pose_cov_6x6 =
      Eigen::Matrix<double, 6, 6>::Zero();

  /// Current nominal pose translation in world frame.
  Eigen::Vector3d p_world = Eigen::Vector3d::Zero();

  /// Per-world-axis position sigmas [m]; E/N/U names require world/ENU alignment.
  double sigma_E = 1e9;
  double sigma_N = 1e9;
  double sigma_U = 1e9;

  /// Eigenvalues of Σ_p (ascending)
  Eigen::Vector3d eig_vals = Eigen::Vector3d::Constant(1e9);

  /// Contributing factor count (how many GNSS+trunk factors are in the window)
  int n_total_factors = 0;
  int n_gnss_factors  = 0;
  int n_trunk_factors = 0;
  int n_imu_factors   = 0;
  int n_clock_factors = 0;
  int n_other_factors = 0;
  int window_key_count = 0;

  /// Lightweight metadata snapshots for downstream integrity/debug use.
  std::vector<int> gnss_sat_ids;
  std::vector<char> gnss_constellations;
  std::vector<int> trunk_landmark_ids;
  std::vector<std::string> factor_type_tags;
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
