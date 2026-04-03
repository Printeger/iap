#pragma once

#include <string>

#include <iap/odometry/odometry_estimation_imu.hpp>

namespace gtsam_points {

class GaussianVoxelMapCPU;
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace glim {

/**
 * @brief Parameters for OdometryEstimationCPU
 */
struct OdometryEstimationCPUParams : public OdometryEstimationIMUParams {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPUParams();
  virtual ~OdometryEstimationCPUParams();

public:
  // Registration params
  std::string registration_type;    ///< Registration type (GICP or VGICP)
  int max_iterations;               ///< Maximum number of iterations
  int lru_thresh;                   ///< LRU cache threshold
  double target_downsampling_rate;  ///< Downsampling rate for points to be inserted into the target

  double ivox_resolution;  ///< iVox resolution (for GICP)
  double ivox_min_dist;    ///< Minimum distance between points in an iVox cell (for GICP)

  double vgicp_resolution;               ///< Voxelmap resolution (for VGICP)
  int vgicp_voxelmap_levels;             ///< Multi-resolution voxelmap levesl (for VGICP)
  double vgicp_voxelmap_scaling_factor;  ///< Multi-resolution voxelmap scaling factor (for VGICP)

  // ---- IAP: ICP quality / health (IAP-RQ-040) ---------------------------
  double icp_cond_threshold;  ///< Hessian condition number threshold for degeneracy (default 500)
  double gamma_lidar_max;     ///< Maximum noise inflation factor for degenerate LiDAR (default 10.0)
  bool        enable_icp_csv  = false;           ///< Write per-frame ICP quality CSV
  std::string icp_csv_path    = "icp_quality.csv";
  // -----------------------------------------------------------------------

  // ---- Scan-to-multi-scan (GLIO2-style) -----------------------------------
  bool use_scan_to_map;               ///< false = scan-to-multi-scan (default), true = legacy accumulator
  int full_connection_window_size;    ///< Recent frames always get binary factors (default: 3)
  int max_num_keyframes;              ///< Max active keyframes in sliding window (default: 15)
  std::string keyframe_update_strategy; ///< "OVERLAP" | "DISPLACEMENT" (default: "OVERLAP")
  double keyframe_max_overlap;        ///< Add keyframe when overlap drops below this (default: 0.7)
  double keyframe_min_overlap;        ///< Drop keyframe when overlap below this (default: 0.01)
  double keyframe_delta_trans;        ///< [DISPLACEMENT] translation threshold m (default: 2.0)
  double keyframe_delta_rot;          ///< [DISPLACEMENT] rotation threshold rad (default: 0.5)
  // -------------------------------------------------------------------------
};

/**
 * @brief CPU-based semi-tightly coupled LiDAR-IMU odometry
 */
class OdometryEstimationCPU : public OdometryEstimationIMU {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationCPU(const OdometryEstimationCPUParams& params = OdometryEstimationCPUParams());
  virtual ~OdometryEstimationCPU() override;

private:
  virtual gtsam::NonlinearFactorGraph create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) override;

  virtual void fallback_smoother() override;

  void update_target(const int current, const Eigen::Isometry3d& T_target_imu);

  // Scan-to-multi-scan keyframe management
  void update_keyframes(int current);
  void update_keyframes_overlap(int current);
  void update_keyframes_displacement(int current);

private:
  // Registration params
  std::mt19937 mt;                                                                   ///< RNG
  Eigen::Isometry3d last_T_target_imu;                                               ///< Last IMU pose w.r.t. target model
  std::vector<std::shared_ptr<gtsam_points::GaussianVoxelMapCPU>> target_voxelmaps;  ///< VGICP target voxelmap
  std::shared_ptr<gtsam_points::iVox> target_ivox;                                   ///< GICP target iVox
  EstimationFrame::ConstPtr target_ivox_frame;                                       ///< Target points (just for visualization)

  // Scan-to-multi-scan state
  std::vector<EstimationFrame::ConstPtr> keyframes;  ///< Active keyframe list
};

}  // namespace glim
