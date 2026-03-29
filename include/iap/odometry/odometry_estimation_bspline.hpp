#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 B-spline odometry module. Builds a continuous-time trajectory view
// on top of the current LiDAR-IMU odometry pipeline without breaking legacy
// discrete outputs consumed by mapping / viewer modules.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_marginalization.hpp>
#include <iap/odometry/bspline_trajectory.hpp>
#include <iap/gnss/gnss_epoch_builder.hpp>
#include <iap/gnss/gnss_handler.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/odometry/integrated_bspline_gnss_factor.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/integrated_bspline_velocity_factor.hpp>
#include <iap/odometry/odometry_estimation_cpu.hpp>
#include <iap/gnss/canopy_noise_model.hpp>
#include <iap/gnss/clock_between_factor.hpp>
#include <iap/gnss/gnss_types.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

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
  struct ActiveSplineIMUSample {
    double stamp = 0.0;
    double u = 0.5;
    Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
  };

  struct ActiveSplineSegmentConstraint {
    double stamp = 0.0;
    double scan_end = 0.0;
    std::array<std::size_t, iap::kBSplineControlPointCount> control_indices{};
    std::size_t velocity_index = 0;
    gtsam_points::PointCloud::ConstPtr source;
    std::shared_ptr<const gtsam_points::iVox> target_snapshot;
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;
    std::vector<ActiveSplineIMUSample> imu_samples;
    std::vector<iap::GnssEpoch> gnss_epochs;
  };

  struct ActiveSplineMarginalPrior {
    bool valid = false;
    std::array<std::size_t, 2> control_indices{};
    gtsam::Pose3 first_pose;
    gtsam::Pose3 relative_delta;
    std::size_t auxiliary_index = 0;
    bool has_velocity = false;
    gtsam::Vector3 velocity = gtsam::Vector3::Zero();
    bool has_clock = false;
    gtsam::Vector2 clock = gtsam::Vector2::Zero();
    iap::BSplineCarriedPrior carried_prior;
  };

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
  std::vector<ActiveSplineIMUSample> create_segment_imu_samples(const PreprocessedFrame::Ptr& raw_frame) const;
  void sync_gnss_epochs_from_shared_state();
  std::vector<iap::GnssEpoch> consume_segment_gnss_epochs(double segment_start, double segment_end);
  void prune_active_ct_state(double min_active_stamp);
  void update_marginal_prior_from_active_window();
  void update_marginal_prior_information(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const std::vector<gtsam::Key>& survivor_keys,
    const iap::BSplineCarriedPrior* previous_prior);
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
  double imu_ct_bias_inf_scale_ = 1e3;
  double imu_ct_gravity_inf_scale_ = 1e3;
  double velocity_ct_inf_scale_ = 1e3;
  int imu_ct_sample_stride_ = 4;
  int lm_max_iterations_ = 8;
  double gnss_time_tolerance_ = 0.1;
  double gnss_min_elevation_ = 10.0 * M_PI / 180.0;
  double gnss_pr_noise_base_ = 5.0;
  double gnss_dop_noise_base_ = 0.5;
  double gnss_elev_noise_exp_ = 2.0;
  double gnss_sigma_ecef_origin_ = 5.0;
  double gnss_sigma_ecef_rot_ = 0.087;
  Eigen::Vector3d gnss_lever_arm_ = Eigen::Vector3d::Zero();
  iap::CanopyNoiseParams gnss_canopy_params_;
  iap::ClockBetweenFactor::Params gnss_clock_between_params_;
  bool gnss_anchor_initialized_ = false;
  Eigen::Vector3d gnss_origin_ecef_ = Eigen::Vector3d::Zero();
  gtsam::Rot3 gnss_ecef_rot_ = gtsam::Rot3::Identity();
  Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity_world_ = Eigen::Vector3d::UnitZ() * 9.80665;

  std::shared_ptr<gtsam_points::iVox> ct_target_ivox_;
  std::unique_ptr<iap::BSplineControlWindow> control_window_;
  std::unique_ptr<iap::BSplineControlWindowBuffer> control_buffer_;
  std::vector<ActiveSplineSegmentConstraint> active_segment_constraints_;
  ActiveSplineMarginalPrior marginal_prior_;
  std::shared_ptr<iap::BSplineTrajectory> latest_trajectory_;
  gtsam::Values latest_ct_aux_values_;
  std::unique_ptr<iap::GnssEpochBuilder> gnss_epoch_builder_;
  std::unique_ptr<iap::GnssHandler> gnss_handler_;
  std::deque<iap::GnssRawObservationBatch> pending_raw_gnss_batches_;
};

}  // namespace glim
