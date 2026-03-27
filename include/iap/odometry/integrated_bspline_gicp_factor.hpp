#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Minimal CPU continuous-time LiDAR factor over four B-spline pose control points.

#include <iap/odometry/bspline_control_window.hpp>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam_points/ann/nearest_neighbor_search.hpp>
#include <gtsam_points/types/point_cloud.hpp>

#include <array>
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

  IntegratedBSplineGICPFactor(
    const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
    const std::shared_ptr<const gtsam_points::iVox>& target,
    const std::shared_ptr<const gtsam_points::PointCloud>& source,
    const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree = nullptr);

  void set_num_threads(int num_threads) { num_threads_ = num_threads; }
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq_ = dist * dist; }

  size_t dim() const override { return 6; }
  double error(const gtsam::Values& values) const override;
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override;

  const std::vector<int>& time_indices() const { return time_indices_; }
  const std::vector<gtsam::Pose3>& source_poses() const { return source_poses_; }
  std::vector<Eigen::Vector4d> deskewed_source_points(const gtsam::Values& values, bool local = true) const;

 private:
  using PoseJacobianArray = std::array<gtsam::Matrix6, kBSplineControlPointCount>;

  std::array<gtsam::Pose3, kBSplineControlPointCount> control_poses(const gtsam::Values& values) const;
  void update_poses(const gtsam::Values& values) const;
  void update_correspondences() const;

  int num_threads_ = 1;
  double max_correspondence_distance_sq_ = 1.0;
  double numeric_eps_ = 1e-4;

  std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree_;
  std::shared_ptr<const gtsam_points::iVox> target_;
  std::shared_ptr<const gtsam_points::PointCloud> source_;

  std::vector<double> time_table_;
  std::vector<int> time_indices_;

  mutable std::vector<gtsam::Pose3> source_poses_;
  mutable std::vector<PoseJacobianArray> pose_jacobians_;
  mutable std::vector<long> correspondences_;
  mutable std::vector<Eigen::Matrix3d> mahalanobis_;
};

}  // namespace iap
