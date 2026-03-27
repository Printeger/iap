#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 B-spline odometry module. Builds a continuous-time trajectory view
// on top of the current LiDAR-IMU odometry pipeline without breaking legacy
// discrete outputs consumed by mapping / viewer modules.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_trajectory.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/odometry_estimation_cpu.hpp>

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace gtsam {
class NonlinearFactorGraph;
}

namespace glim {

struct OdometryEstimationBSplineParams : public OdometryEstimationCPUParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationBSplineParams();
  virtual ~OdometryEstimationBSplineParams();

 public:
  std::string spline_knot_mode;
  double spline_nominal_dt = 0.0;
  double spline_finite_difference_dt = 0.01;
  double compatibility_sample_dt = 0.01;
  bool publish_shared_trajectory = true;
  bool attach_trajectory_to_frames = true;
};

class OdometryEstimationBSpline : public OdometryEstimationCPU {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OdometryEstimationBSpline(
    const OdometryEstimationBSplineParams& params = OdometryEstimationBSplineParams());
  virtual ~OdometryEstimationBSpline() override;
  EstimationFrame::ConstPtr insert_frame(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames) override;

 protected:
  void update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) override;

 private:
  EstimationFrame::ConstPtr insert_frame_reconstruct(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
 EstimationFrame::ConstPtr insert_frame_ct_lidar(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  void initialize_control_window(const PreprocessedFrame::Ptr& raw_frame, const gtsam::Pose3& initial_pose);
  gtsam::Pose3 predict_scan_end_pose(double scan_duration) const;
  gtsam_points::PointCloud::ConstPtr create_lidar_source_cloud(const PreprocessedFrame::Ptr& raw_frame) const;
  std::shared_ptr<gtsam_points::iVox> create_active_target_snapshot() const;
  std::optional<gtsam::Pose3> create_segment_imu_measurement(const PreprocessedFrame::Ptr& raw_frame) const;
  void update_marginal_prior_from_active_window();
  void append_active_segment_constraint(
    const PreprocessedFrame::Ptr& raw_frame,
    const gtsam_points::PointCloud::ConstPtr& source);
  void prune_active_segment_constraints(double min_stamp);
  void insert_target_cloud(const EstimationFrame::Ptr& frame);
  void update_frame_history(
    const EstimationFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  void publish_continuous_trajectory(int current);
  void update_frame_attachment(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;
  void update_compatibility_trajectory(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;

  iap::BSplineTrajectory::Params trajectory_params_;
  double compatibility_sample_dt_ = 0.01;
  bool publish_shared_trajectory_ = true;
  bool attach_trajectory_to_frames_ = true;
  std::string frontend_mode_ = "RECONSTRUCT";
  double max_correspondence_distance_ = 1.0;
  double ctrl_point_anchor_inf_scale_ = 1e6;
  double ctrl_point_prediction_inf_scale_ = 1e3;
  double ctrl_point_smoothness_inf_scale_ = 1e2;
  double ctrl_point_marginal_inf_scale_ = 1e4;
  double imu_ct_trans_inf_scale_ = 10.0;
  double imu_ct_rot_inf_scale_ = 100.0;
  int lm_max_iterations_ = 8;

  struct ActiveSplineSegmentConstraint {
    double stamp = 0.0;
    double scan_end = 0.0;
    std::array<std::size_t, iap::kBSplineControlPointCount> control_indices{};
    gtsam_points::PointCloud::ConstPtr source;
    std::shared_ptr<const gtsam_points::iVox> target_snapshot;
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;
    std::optional<gtsam::Pose3> imu_delta;
  };

  struct ActiveSplineMarginalPrior {
    bool valid = false;
    std::array<std::size_t, 2> control_indices{};
    gtsam::Pose3 first_pose;
    gtsam::Pose3 relative_delta;
  };

  std::shared_ptr<gtsam_points::iVox> ct_target_ivox_;
  std::unique_ptr<iap::BSplineControlWindow> control_window_;
  std::unique_ptr<iap::BSplineControlWindowBuffer> control_buffer_;
  std::vector<ActiveSplineSegmentConstraint> active_segment_constraints_;
  ActiveSplineMarginalPrior marginal_prior_;
  std::shared_ptr<iap::BSplineTrajectory> latest_trajectory_;
};

}  // namespace glim
