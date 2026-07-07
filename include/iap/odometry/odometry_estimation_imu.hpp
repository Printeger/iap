#pragma once

#include <map>
#include <memory>
#include <random>
#include <string>

#include <iap/odometry/odometry_estimation_base.hpp>
#include <gtsam_points/util/gtsam_migration.hpp>
#include <gtsam_points/util/indexed_sliding_window.hpp>

namespace gtsam {
class Pose3;
class Values;
class ImuFactor;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace gtsam_points {
class IncrementalFixedLagSmootherExt;
class IncrementalFixedLagSmootherExtWithFallback;
}  // namespace gtsam_points

namespace glim {

class IMUIntegration;
class IMUValidation;
class CloudDeskewing;
class CloudCovarianceEstimation;
class InitialStateEstimation;

/**
 * @brief Parameters for OdometryEstimationIMU
 */
struct OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMUParams();
  virtual ~OdometryEstimationIMUParams();

public:
  enum class SigmaPUpdateScope { CURRENT, ALL_ACTIVE };

  // Sensor params;
  bool fix_imu_bias;
  double imu_bias_noise;
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Matrix<double, 6, 1> imu_bias;

  // Init state
  std::string initialization_mode;
  bool estimate_init_state;
  Eigen::Isometry3d init_T_world_imu;
  Eigen::Vector3d init_v_world_imu;
  double init_pose_damping_scale;

  // Optimization params
  double smoother_lag;
  bool use_isam2_dogleg;
  double isam2_relinearize_skip;
  double isam2_relinearize_thresh;

  // IAP-RQ-010: clock noise parameters (loose priors until GNSS factors added)
  double clk_bias_noise;   ///< Sigma for clock bias random-walk prior [m]
  double clk_drift_noise;  ///< Sigma for clock drift random-walk prior [m/s]
  std::string clock_owner_mode;  ///< dual | odometry | gnss

  // Per-type iSAM2 relinearization thresholds for clock state (IAP-RQ-010)
  // Clock bias can legitimately move 100s of m/frame — keep loose to avoid sync-mode linearization
  double clk_bias_relin_thresh;   ///< iSAM2 relinearize threshold for clock bias [m]
  double clk_drift_relin_thresh;  ///< iSAM2 relinearize threshold for clock drift [m/s]

  // Logging params
  bool validate_imu;
  bool save_imu_rate_trajectory;
  SigmaPUpdateScope sigma_p_update_scope;

  int num_threads;                  // Number of threads for preprocessing and per-factor parallelism
  int num_smoother_update_threads;  // Number of threads for TBB parallelism in smoother update (should be kept 1)
};

/**
 * @brief Base class for LiDAR-IMU odometry estimation
 */
class OdometryEstimationIMU : public OdometryEstimationBase {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params);
  virtual ~OdometryEstimationIMU() override;

  virtual void insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) override;
  virtual EstimationFrame::ConstPtr insert_frame(const PreprocessedFrame::Ptr& frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;
  virtual std::vector<EstimationFrame::ConstPtr> get_remaining_frames() override;

protected:
  virtual void create_frame(EstimationFrame::Ptr& frame) {}
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) = 0;

  virtual void fallback_smoother() {}
  virtual void update_frames(const int current, const gtsam::NonlinearFactorGraph& new_factors);

  virtual void
  update_smoother(const gtsam::NonlinearFactorGraph& new_factors, const gtsam::Values& new_values, const std::map<std::uint64_t, double>& new_stamp, int update_count = 0);
  virtual void update_smoother(int update_count = 1);

protected:
  std::unique_ptr<OdometryEstimationIMUParams> params;

  // Sensor extrinsic params
  Eigen::Isometry3d T_lidar_imu;
  Eigen::Isometry3d T_imu_lidar;

  // Frames & keyframes
  int marginalized_cursor;
  gtsam_points::IndexedSlidingWindow<EstimationFrame::Ptr> frames;

  // Utility classes
  std::unique_ptr<InitialStateEstimation> init_estimation;
  std::unique_ptr<IMUIntegration> imu_integration;
  std::unique_ptr<IMUValidation> imu_validation;
  std::unique_ptr<CloudDeskewing> deskewing;
  std::unique_ptr<CloudCovarianceEstimation> covariance_estimation;

  // Optimizer
  using FixedLagSmootherExt = gtsam_points::IncrementalFixedLagSmootherExtWithFallback;
  std::unique_ptr<FixedLagSmootherExt> smoother;

  bool odometry_owns_clock_ = true;

  std::shared_ptr<void> tbb_task_arena;
};

}  // namespace glim
