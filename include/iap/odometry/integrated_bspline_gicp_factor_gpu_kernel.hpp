#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Dedicated kernel-level GPU continuous-time LiDAR factor. Unlike the frozen
// BUCKET backend, this factor evaluates per-point spline poses directly on the
// GPU and accumulates a 24x24 system over the four control poses.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_lidar_factor_result.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <gtsam_points/config.hpp>

#ifdef GTSAM_POINTS_USE_CUDA

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/point_cloud_gpu.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

struct CUstream_st;

namespace gtsam_points {
class TempBufferManager;
}

namespace iap {

class IntegratedBSplineGICPFactorGPUKernel : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineGICPFactorGPUKernel>;

  struct DeviceControlPose;
  struct DeviceKernelStats;

  IntegratedBSplineGICPFactorGPUKernel(
    const SplineBucketContext& ctx,
    const std::shared_ptr<const gtsam_points::iVox>& target,
    const std::shared_ptr<const gtsam_points::PointCloud>& source,
    CUstream_st* stream = nullptr,
    std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer = nullptr);

  // Legacy compatibility wrapper for the pre-Commit-11 experimental KERNEL path.
  IntegratedBSplineGICPFactorGPUKernel(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    const std::shared_ptr<const gtsam_points::iVox>& target,
    const std::shared_ptr<const gtsam_points::PointCloud>& source,
    CUstream_st* stream = nullptr,
    std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer = nullptr);
  ~IntegratedBSplineGICPFactorGPUKernel() override;

  size_t dim() const override { return 24; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

  void set_enable_profiling(bool enable) { enable_profiling_ = enable; }
  void set_jacobian_mode(IntegratedBSplineGICPFactor::JacobianMode mode) { jacobian_mode_ = mode; }
  IntegratedBSplineGICPFactor::JacobianMode jacobian_mode() const { return jacobian_mode_; }
  void set_numeric_eps(double eps);
  void set_max_correspondence_distance(double dist);
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

  const void* source_staging_identity() const { return source_gpu_.get(); }

 private:
  struct EvaluationResult;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  void rebuild_target_gpu();
  void ensure_source_gpu() const;
  EvaluationResult evaluate(const gtsam::Values& values, bool compute_hessian) const;
  void update_profile(const EvaluationResult& eval, const char* stage) const;
  std::shared_ptr<gtsam::HessianFactor> make_hessian_factor(const EvaluationResult& eval) const;
  std::shared_ptr<IntegratedSplineGICPFactor> make_cpu_reference_factor(
    IntegratedBSplineGICPFactor::JacobianMode mode) const;

  IntegratedBSplineGICPFactor::JacobianMode jacobian_mode_ =
    IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC;
  bool enable_profiling_ = false;
  double numeric_eps_ = 1e-4;
  double max_correspondence_distance_ = 1.0;
  double max_correspondence_distance_sq_ = 1.0;
  int correspondence_candidate_count_ = 3;
  double correspondence_accept_ratio_ = 0.0;
  double correspondence_min_score_gap_ = 0.0;
  double outlier_mahalanobis_threshold_ = 0.0;
  double robust_weight_floor_ = 0.0;
  IntegratedBSplineGICPFactor::RobustKernel robust_kernel_ = IntegratedBSplineGICPFactor::RobustKernel::NONE;
  double robust_kernel_width_ = 1.0;

  CUstream_st* stream_ = nullptr;
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer_;
  SplineBucketContext ctx_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;
  mutable std::shared_ptr<gtsam_points::PointCloudGPU> source_gpu_;
  std::shared_ptr<gtsam_points::GaussianVoxelMapGPU> target_gpu_;
  std::shared_ptr<gtsam_points::PointCloudCPU> target_cpu_points_;
  std::vector<float> normalized_times_;
  std::vector<double> time_table_;
  std::vector<std::size_t> time_bucket_populations_;
  std::size_t target_point_count_ = 0;
  mutable float* normalized_times_gpu_ = nullptr;
  mutable float* linearized_hessian_gpu_ = nullptr;
  mutable float* linearized_gradient_gpu_ = nullptr;
  mutable DeviceKernelStats* kernel_stats_gpu_ = nullptr;
  mutable int* matched_target_indices_gpu_ = nullptr;

  mutable std::array<gtsam::Pose3, kBSplineControlPointCount> last_control_poses_;
  mutable bool last_control_poses_valid_ = false;
  mutable BSplineLidarFactorProfile last_profile_;
  mutable int last_inlier_count_ = 0;
};

}  // namespace iap

#endif  // GTSAM_POINTS_USE_CUDA
