#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal CPU continuous-time LiDAR factor over four B-spline pose control points.

#include <iap/odometry/bspline_control_window.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <array>
#include <algorithm>
#include <cstddef>
#include <vector>

namespace gtsam_points {
struct FlatContainer;
template <typename VoxelContents>
class IncrementalVoxelMap;
using iVox = IncrementalVoxelMap<FlatContainer>;
}  // namespace gtsam_points

namespace iap {

class IntegratedBSplineGICPFactor : public gtsam::NonlinearFactor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedBSplineGICPFactor>;

  enum class JacobianMode {
    NUMERIC_FULL,
    SEMI_ANALYTIC,
  };

  enum class RobustKernel {
    NONE,
    HUBER,
    CAUCHY,
  };

  struct ProfilingStats {
    bool valid = false;
    std::size_t source_point_count = 0;
    std::size_t target_point_count = 0;
    std::size_t matched_point_count = 0;
    std::size_t inlier_point_count = 0;
    std::size_t rejected_distance_count = 0;
    std::size_t rejected_ambiguity_count = 0;
    std::size_t rejected_outlier_count = 0;
    std::size_t rejected_robust_count = 0;
    double match_ratio = 0.0;
    double inlier_ratio = 0.0;
    double mean_robust_weight = 1.0;
    double pose_update_ms = 0.0;
    double correspondence_ms = 0.0;
    double accumulation_ms = 0.0;
    double total_ms = 0.0;
    double total_error = 0.0;
    const char* stage = "none";
  };

  struct LinearizationCheckResult {
    bool valid = false;
    double perturbation_scale = 0.0;
    double base_error = 0.0;
    double predicted_error = 0.0;
    double actual_error = 0.0;
    double abs_error = 0.0;
    double rel_error = 0.0;
  };

  IntegratedBSplineGICPFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    const std::shared_ptr<const gtsam_points::iVox>& target,
    const std::shared_ptr<const gtsam_points::PointCloud>& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree = nullptr);

  void set_num_threads(int num_threads) { num_threads_ = num_threads; }
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq_ = dist * dist; }
  void set_enable_profiling(bool enable) { enable_profiling_ = enable; }
  void set_jacobian_mode(JacobianMode mode) { jacobian_mode_ = mode; }
  void set_numeric_eps(double eps) { numeric_eps_ = std::max(1e-8, eps); }
  void set_correspondence_candidate_count(int count) { correspondence_candidate_count_ = std::max(1, count); }
  void set_correspondence_accept_ratio(double ratio) { correspondence_accept_ratio_ = ratio; }
  void set_correspondence_min_score_gap(double gap) { correspondence_min_score_gap_ = std::max(0.0, gap); }
  void set_outlier_mahalanobis_threshold(double threshold) { outlier_mahalanobis_threshold_ = std::max(0.0, threshold); }
  void set_robust_weight_floor(double floor) { robust_weight_floor_ = std::clamp(floor, 0.0, 1.0); }
  void set_robust_kernel(RobustKernel kernel, double width) {
    robust_kernel_ = kernel;
    robust_kernel_width_ = std::max(1e-6, width);
  }

  JacobianMode jacobian_mode() const { return jacobian_mode_; }
  RobustKernel robust_kernel() const { return robust_kernel_; }
  double robust_kernel_width() const { return robust_kernel_width_; }
  int correspondence_candidate_count() const { return correspondence_candidate_count_; }
  double correspondence_accept_ratio() const { return correspondence_accept_ratio_; }
  double correspondence_min_score_gap() const { return correspondence_min_score_gap_; }
  double outlier_mahalanobis_threshold() const { return outlier_mahalanobis_threshold_; }
  double robust_weight_floor() const { return robust_weight_floor_; }

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;
  LinearizationCheckResult check_linearization(const gtsam::Values& values, double perturbation_scale = 1e-4) const;

  const std::vector<int>& time_indices() const { return time_indices_; }
  const std::vector<gtsam::Pose3>& source_poses() const { return source_poses_; }
  const ProfilingStats& last_profiling_stats() const { return last_profile_; }
  int num_inliers() const { return static_cast<int>(accepted_inlier_count_); }
  double inlier_fraction() const;
  std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local = true) const;

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;
  using QuaternionJacobianArray = std::array<Eigen::Matrix<double, 4, 3>, kBSplineControlPointCount>;
  using PointJacobianArray = std::array<gtsam::Matrix36, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  void update_poses(const gtsam::Values& values) const;
  void update_correspondences() const;
  PointJacobianArray point_jacobians(
    const std::array<gtsam::Pose3, kBSplineControlPointCount>& control_poses,
    std::size_t time_index,
    const Eigen::Vector3d& source_point) const;
  double robust_cost(double mahalanobis_error) const;
  double robust_weight(double mahalanobis_error) const;

  int num_threads_ = 1;
  double max_correspondence_distance_sq_ = 1.0;
  double numeric_eps_ = 1e-4;
  bool enable_profiling_ = false;
  JacobianMode jacobian_mode_ = JacobianMode::SEMI_ANALYTIC;
  RobustKernel robust_kernel_ = RobustKernel::NONE;
  double robust_kernel_width_ = 1.0;
  int correspondence_candidate_count_ = 3;
  double correspondence_accept_ratio_ = 0.0;
  double correspondence_min_score_gap_ = 0.0;
  double outlier_mahalanobis_threshold_ = 0.0;
  double robust_weight_floor_ = 0.0;

  std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;

  std::vector<double> time_table_;
  std::vector<std::array<double, kBSplineControlPointCount>> basis_table_;
  std::vector<int> time_indices_;

  mutable std::vector<gtsam::Pose3> source_poses_;
  mutable std::vector<PoseJacobianArray> pose_jacobians_;
  mutable std::vector<Eigen::Vector4d> blended_quaternion_coeffs_;
  mutable std::vector<QuaternionJacobianArray> blend_quaternion_jacobians_;
  mutable std::vector<long> correspondences_;
  mutable std::vector<Eigen::Matrix3d> mahalanobis_;
  mutable std::size_t matched_correspondence_count_ = 0;
  mutable std::size_t rejected_distance_count_ = 0;
  mutable std::size_t rejected_ambiguity_count_ = 0;
  mutable std::size_t rejected_outlier_count_ = 0;
  mutable std::size_t rejected_robust_count_ = 0;
  mutable std::size_t accepted_inlier_count_ = 0;
  mutable double robust_weight_sum_ = 0.0;
  mutable ProfilingStats last_profile_;
};

}  // namespace iap
