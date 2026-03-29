#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal CPU continuous-time LiDAR factor over four B-spline pose control points.

#include <iap/odometry/bspline_control_window.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <array>
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

  struct ProfilingStats {
    bool valid = false;
    std::size_t source_point_count = 0;
    std::size_t target_point_count = 0;
    std::size_t matched_point_count = 0;
    double match_ratio = 0.0;
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

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;
  LinearizationCheckResult check_linearization(const gtsam::Values& values, double perturbation_scale = 1e-4) const;

  const std::vector<int>& time_indices() const { return time_indices_; }
  const std::vector<gtsam::Pose3>& source_poses() const { return source_poses_; }
  const ProfilingStats& last_profiling_stats() const { return last_profile_; }
  std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local = true) const;

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  void update_poses(const gtsam::Values& values) const;
  void update_correspondences() const;

  int num_threads_ = 1;
  double max_correspondence_distance_sq_ = 1.0;
  double numeric_eps_ = 1e-4;
  bool enable_profiling_ = false;

  std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;

  std::vector<double> time_table_;
  std::vector<int> time_indices_;

  mutable std::vector<gtsam::Pose3> source_poses_;
  mutable std::vector<PoseJacobianArray> pose_jacobians_;
  mutable std::vector<long> correspondences_;
  mutable std::vector<Eigen::Matrix3d> mahalanobis_;
  mutable std::size_t matched_correspondence_count_ = 0;
  mutable ProfilingStats last_profile_;
};

}  // namespace iap
