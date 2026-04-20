#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Frozen engineering GPU continuous-time LiDAR factor over four B-spline pose
// control points. Uses GPU VGICP subfactors over time buckets and maps their
// unary Hessians back to the four control poses. The future kernel-level
// spline-native backend is intended to live behind a separate runtime switch.
// Mainline use: local CT frontend only (GPU BUCKET path).
// Backend must consume summarized outputs, not raw GPU LiDAR bucket factors.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/bspline_pose_jacobian.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
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

class IntegratedSplineGICPFactorGPU : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedSplineGICPFactorGPU>;

  enum class JacobianMode {
    NUMERIC_FULL,
    SEMI_ANALYTIC,
  };

  IntegratedSplineGICPFactorGPU(
    const SplineBucketContext& ctx,
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
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq_ = dist * dist; }
  void set_correspondence_candidate_count(int count) { correspondence_candidate_count_ = std::max(1, count); }
  void set_correspondence_accept_ratio(double ratio) { correspondence_accept_ratio_ = ratio; }
  void set_correspondence_min_score_gap(double gap) { correspondence_min_score_gap_ = std::max(0.0, gap); }
  void set_outlier_mahalanobis_threshold(double threshold) { outlier_mahalanobis_threshold_ = std::max(0.0, threshold); }
  void set_robust_weight_floor(double floor) { robust_weight_floor_ = std::clamp(floor, 0.0, 1.0); }
  void set_robust_kernel(IntegratedBSplineGICPFactor::RobustKernel kernel, double width) {
    robust_kernel_ = kernel;
    robust_kernel_width_ = std::max(1e-6, width);
  }

  BSplineLidarFactorProfile profiling_report() const;
  BSplineLidarNumericAudit check_against_numeric_full(const gtsam::Values& values, double perturbation_scale = 1e-5) const;
  BSplineLidarDegeneracyReport diagnose_degeneracy(
    const IntegratedBSplineGICPFactor::DegeneracyThresholds& thresholds) const;
  void refresh_target(const std::shared_ptr<const gtsam_points::iVox>& target);
  BSplineLidarFactorResult make_result(
    double factor_error,
    int inlier_count,
    double inlier_fraction,
    const BSplineLidarNumericAudit* numeric_audit = nullptr,
    const BSplineLidarDegeneracyReport* degeneracy = nullptr) const;

  int num_inliers() const { return last_inlier_count_; }
  double inlier_fraction() const;
  std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local = true) const;
  std::shared_ptr<gtsam_points::PointCloudGPU> source_staging_identity() const { return source_gpu_; }

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  struct BucketSystem {
    std::vector<gtsam::Matrix6> info_mats;
    std::vector<gtsam::Vector6> linear_terms;
    double constant = 0.0;
    double total_error = 0.0;
    int inlier_count = 0;
  };

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  gtsam::Values control_pose_values() const;
  void rebuild_target_gpu();
  void build_bucket_factor();
  void update_bucket_pose(const gtsam::Values& values) const;
  PoseJacobianArray compute_bucket_pose_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
    JacobianMode mode) const;
  gtsam::Values bucket_values() const;
  BucketSystem collect_bucket_system(const gtsam::Values& bucket_vals) const;
  void map_bucket_system(
    const BucketSystem& system,
    const PoseJacobianArray& pose_jacobians,
    Eigen::Matrix<double, 24, 24>* H,
    Eigen::Matrix<double, 24, 1>* g) const;
  std::shared_ptr<IntegratedSplineGICPFactor> make_cpu_audit_factor() const;
  void ensure_detailed_profile() const;
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
  double max_correspondence_distance_sq_ = 1.0;
  int correspondence_candidate_count_ = 3;
  double correspondence_accept_ratio_ = 0.0;
  double correspondence_min_score_gap_ = 0.0;
  double outlier_mahalanobis_threshold_ = 0.0;
  double robust_weight_floor_ = 0.0;
  IntegratedBSplineGICPFactor::RobustKernel robust_kernel_ = IntegratedBSplineGICPFactor::RobustKernel::NONE;
  double robust_kernel_width_ = 1.0;

  CUstream_st* stream_ = nullptr;
  gtsam_points::TempBufferManager::Ptr temp_buffer_;
  SplineBucketContext ctx_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;
  std::shared_ptr<gtsam_points::GaussianVoxelMapGPU> target_gpu_;
  std::shared_ptr<gtsam_points::PointCloudCPU> target_cpu_points_;
  std::shared_ptr<gtsam_points::PointCloudGPU> source_gpu_;
  std::shared_ptr<gtsam_points::IntegratedVGICPFactorGPU> bucket_factor_;
  std::size_t target_point_count_ = 0;

  mutable gtsam::Pose3 bucket_pose_;
  mutable PoseJacobianArray bucket_pose_jacobians_;
  mutable std::array<gtsam::Pose3, kBSplineControlPointCount> last_control_poses_;
  mutable bool bucket_pose_valid_ = false;
  mutable bool bucket_pose_jacobians_valid_ = false;
  mutable std::shared_ptr<gtsam_points::PointCloudGPU> source_staging_identity_cache_;
  mutable bool last_control_poses_valid_ = false;
  mutable BSplineLidarFactorProfile last_profile_;
  mutable int last_inlier_count_ = 0;
};

using IntegratedBSplineGICPFactorGPU = IntegratedSplineGICPFactorGPU;

}  // namespace iap

#endif  // GTSAM_POINTS_USE_CUDA
