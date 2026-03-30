#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal GPU continuous-time LiDAR factor over four B-spline pose control
// points. Uses GPU VGICP subfactors over time buckets and maps their unary
// Hessians back to the four control poses.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/bspline_pose_jacobian.hpp>
#include <gtsam_points/config.hpp>

#ifdef GTSAM_POINTS_USE_CUDA

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

struct CUstream_st;

namespace iap {

class IntegratedBSplineGICPFactorGPU : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineGICPFactorGPU>;

  enum class JacobianMode {
    NUMERIC_FULL,
    SEMI_ANALYTIC,
  };

  IntegratedBSplineGICPFactorGPU(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    const std::shared_ptr<const gtsam_points::iVox>& target,
    const std::shared_ptr<const gtsam_points::PointCloud>& source,
    CUstream_st* stream = nullptr,
    gtsam_points::TempBufferManager::Ptr temp_buffer = nullptr);

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

  void set_enable_profiling(bool enable) { enable_profiling_ = enable; }
  void set_jacobian_mode(JacobianMode mode) { jacobian_mode_ = mode; }
  JacobianMode jacobian_mode() const { return jacobian_mode_; }
  void set_numeric_eps(double eps);

  BSplineLidarFactorProfile profiling_report() const;
  BSplineLidarFactorResult make_result(double factor_error, int inlier_count, double inlier_fraction) const;

  int num_inliers() const { return last_inlier_count_; }
  double inlier_fraction() const;
  std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local = true) const;

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  struct BucketFactor {
    double stamp = 0.0;
    double u = 0.0;
    gtsam::Key key = 0;
    std::size_t point_count = 0;
    std::shared_ptr<gtsam_points::PointCloudGPU> source_gpu;
    std::shared_ptr<gtsam_points::IntegratedVGICPFactorGPU> factor;
  };

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  void build_bucket_factors();
  void update_bucket_poses(const gtsam::Values& values) const;
  gtsam::Values bucket_values() const;
  void update_profile(
    const char* stage,
    double pose_update_ms,
    double gpu_linearize_ms,
    double reduction_ms,
    double total_ms,
    double total_error) const;

  JacobianMode jacobian_mode_ = JacobianMode::SEMI_ANALYTIC;
  double numeric_eps_ = 1e-4;
  bool enable_profiling_ = false;

  CUstream_st* stream_ = nullptr;
  gtsam_points::TempBufferManager::Ptr temp_buffer_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;
  std::shared_ptr<gtsam_points::GaussianVoxelMapGPU> target_gpu_;
  std::shared_ptr<gtsam_points::PointCloudCPU> target_cpu_points_;
  std::vector<BucketFactor> bucket_factors_;
  gtsam::NonlinearFactorGraph bucket_graph_;
  std::vector<double> time_table_;
  std::vector<int> time_indices_;
  std::vector<std::size_t> time_bucket_populations_;
  std::size_t target_point_count_ = 0;

  mutable std::vector<gtsam::Pose3> bucket_poses_;
  mutable std::vector<PoseJacobianArray> bucket_pose_jacobians_;
  mutable BSplineLidarFactorProfile last_profile_;
  mutable int last_inlier_count_ = 0;
};

}  // namespace iap

#endif  // GTSAM_POINTS_USE_CUDA
