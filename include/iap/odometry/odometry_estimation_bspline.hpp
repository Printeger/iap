#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 B-spline odometry module. Builds a continuous-time trajectory view
// on top of the current LiDAR-IMU odometry pipeline without breaking legacy
// discrete outputs consumed by mapping / viewer modules.
//
// Commit 0 migration boundary:
// The current implementation still uses the fixed 4-control-point local spline
// window as its operational path. Follow-up commits will migrate toward an
// explicit knot vector plus unified spline evaluator shared by IMU, GNSS, and
// LiDAR queries. This commit intentionally does not change algorithms,
// residuals, public interfaces, ROS/plugin wiring, or logging keywords; it
// only freezes the migration boundary and the CPU-first rollout order.

#include <iap/odometry/bspline_control_window.hpp>
#include <iap/odometry/bspline_fixed_lag_registry.hpp>
#include <iap/odometry/bspline_marginalization.hpp>
#include <iap/odometry/bspline_trajectory.hpp>
#include <iap/odometry/ct_compact_backend.hpp>
#include <iap/odometry/ct_local_frontend.hpp>
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
#include <gtsam_points/config.hpp>
#include <memory>
#include <vector>

#ifdef GTSAM_POINTS_USE_CUDA
namespace gtsam_points {
class StreamTempBufferRoundRobin;
class TempBufferManager;
}

namespace iap {
class IntegratedSplineGICPFactorGPU;
using IntegratedBSplineGICPFactorGPU = IntegratedSplineGICPFactorGPU;
class IntegratedBSplineGICPFactorGPUKernel;
}

struct CUstream_st;
#endif

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

enum class BSplineLidarTargetMode {
  ACTIVE_WINDOW_SNAPSHOT,
  GLOBAL_IVOX_REFERENCE,
};

enum class BSplineGpuLidarBackend {
  BUCKET,
  KERNEL,
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

  struct ActiveSplineTargetReference {
    std::shared_ptr<const gtsam_points::iVox> target_snapshot;
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;
    BSplineLidarTargetMode mode = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
    std::size_t contributing_frames = 0;
    std::size_t point_count = 0;
    std::size_t snapshot_frame_count = 0;
    std::size_t snapshot_point_count = 0;
    double snapshot_span_sec = 0.0;
    bool snapshot_policy_accepted = false;
    double build_ms = 0.0;
    double target_snapshot_clone_ms = 0.0;
    double target_voxel_lookup_prep_ms = 0.0;
    double target_covariance_prep_ms = 0.0;
    double source_to_target_transform_ms = 0.0;
  };

  struct FrameWarningProfileRow {
    int frame_id = -1;
    double stamp = 0.0;
    std::size_t warning_count = 0;
    std::string warning_categories;
    std::string top_warning_message;
  };

  struct ActiveSplineLidarFactorCacheKey {
    bool valid = false;
    bool gpu = false;
    BSplineGpuLidarBackend gpu_backend = BSplineGpuLidarBackend::BUCKET;
    BSplineLidarTargetMode target_mode = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
    std::array<std::size_t, iap::kBSplineControlPointCount> control_indices{};
    std::size_t bucket_u_signature = 0;
    const void* source_identity = nullptr;
    const void* target_identity = nullptr;
    std::size_t target_revision = 0;
    std::size_t config_signature = 0;
  };

  struct ActiveSplineSegmentConstraint : public iap::BSplineFixedLagSegmentState {
    gtsam_points::PointCloud::ConstPtr source;
    std::shared_ptr<const gtsam_points::iVox> target_snapshot;
    std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree;
    BSplineLidarTargetMode target_mode = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
    std::size_t target_frame_count = 0;
    std::size_t target_point_count = 0;
    std::size_t snapshot_frame_count = 0;
    std::size_t snapshot_point_count = 0;
    double snapshot_span_sec = 0.0;
    bool snapshot_policy_accepted = false;
    double target_build_ms = 0.0;
    std::vector<ActiveSplineIMUSample> imu_samples;
    std::vector<iap::GnssEpoch> gnss_epochs;
    ActiveSplineLidarFactorCacheKey lidar_factor_cache;
    std::shared_ptr<iap::IntegratedBSplineGICPFactor> cached_cpu_factor;
#ifdef GTSAM_POINTS_USE_CUDA
    std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> cached_gpu_bucket_factor;
    std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> cached_gpu_kernel_factor;
#endif
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
  ActiveSplineTargetReference create_active_target_reference() const;
  std::vector<ActiveSplineIMUSample> create_segment_imu_samples(const PreprocessedFrame::Ptr& raw_frame) const;
  std::shared_ptr<const iap::SplineStateLayout> build_active_window_layout() const;
  void refresh_active_window_layout();
  std::shared_ptr<const iap::SplineStateLayout> create_segment_imu_layout(const ActiveSplineSegmentConstraint& segment) const;
  std::shared_ptr<const iap::SplineStateLayout> create_segment_lidar_layout(const ActiveSplineSegmentConstraint& segment) const;
  std::vector<iap::SplineBucketContext> create_segment_lidar_buckets(const ActiveSplineSegmentConstraint& segment) const;
  void sync_gnss_epochs_from_shared_state();
  std::vector<iap::GnssEpoch> consume_segment_gnss_epochs(double segment_start, double segment_end);
  void prune_active_ct_state(double min_active_stamp, const iap::SplineActiveStateSet& active_state_set);
  void update_marginal_prior_from_active_window();
  void update_marginal_prior_information(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const std::vector<gtsam::Key>& survivor_keys,
    const iap::BSplineCarriedPrior* previous_prior);
  void append_active_segment_constraint(
    const PreprocessedFrame::Ptr& raw_frame,
    const gtsam_points::PointCloud::ConstPtr& source);
  void insert_target_cloud(const EstimationFrame::Ptr& frame);
  void update_frame_history(
    const EstimationFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  void publish_continuous_trajectory(int current);
  void publish_continuous_trajectory_from_layout(
    std::shared_ptr<const iap::SplineStateLayout> layout,
    const gtsam::Values& values);
  void publish_fixed_lag_telemetry(int current) const;
  void update_frame_attachment(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;
  void update_compatibility_trajectory(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;
  bool lidar_collect_window_results() const;
  std::size_t lidar_factor_config_signature(bool use_gpu_lidar) const;
  std::size_t lidar_target_revision(const ActiveSplineSegmentConstraint& segment) const;
  ActiveSplineLidarFactorCacheKey make_lidar_factor_cache_key(
    const ActiveSplineSegmentConstraint& segment,
    bool use_gpu_lidar) const;
  bool same_lidar_factor_cache_base(
    const ActiveSplineLidarFactorCacheKey& lhs,
    const ActiveSplineLidarFactorCacheKey& rhs) const;
  std::shared_ptr<iap::IntegratedBSplineGICPFactor> get_or_create_cpu_lidar_factor(
    ActiveSplineSegmentConstraint& segment,
    bool* cache_hit);
#ifdef GTSAM_POINTS_USE_CUDA
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> get_or_create_gpu_bucket_lidar_factor(
    const iap::SplineBucketContext& bucket_ctx,
    ActiveSplineSegmentConstraint& segment,
    CUstream_st* stream,
    std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
    bool* cache_hit,
    bool* target_refreshed);
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> get_or_create_gpu_kernel_lidar_factor(
    const iap::SplineBucketContext& bucket_ctx,
    ActiveSplineSegmentConstraint& segment,
    CUstream_st* stream,
    std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
    bool* cache_hit,
    bool* target_refreshed);
#endif
  void maybe_export_lidar_baseline_csv(
    double stamp,
    const std::vector<iap::BSplineLidarFactorResult>& results,
    int current_factor_index);
  void maybe_write_frontend_frame_profile(const iap::FrontendFrameProfile& profile);
  void maybe_write_lidar_factor_profiles(
    int frame_id,
    double stamp,
    const std::vector<iap::FrontendBucketProfileRow>& profiles);
  void maybe_write_frontend_lm_iterations(
    int frame_id,
    double stamp,
    const std::vector<iap::FrontendLMIterationProfileRow>& iterations);
  void maybe_write_frame_warning_profile(const FrameWarningProfileRow& row);
  FrameWarningProfileRow build_frame_warning_profile(
    int frame_id,
    double stamp,
    const iap::CTLocalFrontend::Input& input,
    const iap::CTLocalFrontendResult& local_result,
    const iap::FrontendFrameProfile& profile) const;
  void log_frontend_only_stats(const iap::FrontendFrameProfile& profile) const;

  // IAP-RQ-300 / IAP-RQ-410: Hybrid orchestration helpers (Task 5).
  // Build the CTLocalFrontend::Input from the current raw frame and frame history.
  iap::CTLocalFrontend::Input make_frontend_input(
    const PreprocessedFrame::Ptr& raw_frame,
    const gtsam_points::PointCloud::ConstPtr& prepared_source_cloud) const;
  // Build the CTCompactBackend::Input from the local frontend result and shared GNSS state.
  iap::CTCompactBackend::Input make_backend_input(const iap::CTLocalFrontendResult& local_result) const;

  iap::BSplineTrajectory::Params trajectory_params_;
  // Planned hybrid split: orchestrator bridges CTLocalFrontend and CTCompactBackend.
  // Task 5 wires these as members; Task 6+ migrates real factor assembly behind them.
  iap::CTLocalFrontend ct_local_frontend_;
  iap::CTCompactBackend ct_compact_backend_;
  double compatibility_sample_dt_ = 0.01;
  bool publish_shared_trajectory_ = true;
  bool attach_trajectory_to_frames_ = true;
  // Commit 0 guardrail: keep CPU CT LiDAR as the default mainline whenever the
  // config is missing, while preserving all legacy frontends as explicit opt-ins.
  std::string frontend_mode_ = "CT_LIDAR_CPU";
  bool frontend_only_mode_ = false;
  double max_correspondence_distance_ = 1.0;
  iap::CTLocalFrontend::BucketConfig lidar_bucket_config_;
  BSplineLidarTargetMode lidar_target_mode_ = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
  BSplineGpuLidarBackend lidar_gpu_backend_ = BSplineGpuLidarBackend::BUCKET;
  iap::IntegratedBSplineGICPFactor::JacobianMode lidar_jacobian_mode_ =
    iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC;
  iap::IntegratedBSplineGICPFactor::RobustKernel lidar_robust_kernel_ =
    iap::IntegratedBSplineGICPFactor::RobustKernel::NONE;
  int lidar_correspondence_candidate_count_ = 3;
  double lidar_correspondence_accept_ratio_ = 0.0;
  double lidar_correspondence_min_score_gap_ = 0.0;
  int lidar_snapshot_frame_window_ = 0;
  int lidar_snapshot_min_frames_ = 2;
  int lidar_snapshot_min_points_ = 64;
  double lidar_snapshot_max_age_ = 0.0;
  double lidar_jacobian_numeric_eps_ = 1e-4;
  double lidar_outlier_mahalanobis_thresh_ = 0.0;
  double lidar_robust_kernel_width_ = 1.0;
  double lidar_robust_weight_floor_ = 0.0;
  bool frontend_frame_profile_enabled_ = true;
  bool lidar_factor_profile_ = false;
  bool frontend_lm_iteration_profile_enabled_ = false;
  bool frame_warning_profile_enabled_ = false;
  bool target_map_prep_breakdown_enabled_ = false;
  bool graph_problem_size_enabled_ = false;
  bool lidar_validate_linearization_ = false;
  bool lidar_profile_numeric_reference_ = false;
  bool lidar_warn_degeneracy_ = true;
  bool lidar_export_baseline_csv_ = false;
  bool pipeline_profile_ = false;
  double lidar_linearization_check_scale_ = 1e-4;
  double lidar_linearization_warn_ratio_ = 0.25;
  double lidar_numeric_reference_scale_ = 1e-5;
  std::string lidar_baseline_csv_path_ = "ct_lidar_baseline.csv";
  std::string frontend_frame_profile_csv_path_ = "frontend_frame_profile.csv";
  std::string frontend_lidar_factor_profile_csv_path_ = "lidar_factor_profile.csv";
  std::string frontend_lm_iteration_csv_path_ = "frontend_lm_iteration.csv";
  std::string frame_warning_profile_csv_path_ = "frame_warning_profile.csv";
  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds lidar_degeneracy_thresholds_;
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

  std::shared_ptr<gtsam_points::iVox> ct_target_ivox_;
  // Commit 7 note:
  // `control_window_` is now a legacy bootstrap/advance helper used to keep the
  // current fixed-4-control-point rollout stable while the scheduler, factor
  // assembly, and trajectory publication move to explicit-knot layout/evaluator
  // paths owned by `active_window_layout_`.
  std::unique_ptr<iap::BSplineControlWindow> control_window_;
  iap::BSplineFixedLagStateRegistryT<ActiveSplineSegmentConstraint> fixed_lag_registry_;
  ActiveSplineMarginalPrior marginal_prior_;
  std::shared_ptr<const iap::SplineStateLayout> active_window_layout_;
  std::shared_ptr<iap::SplineEvaluator> active_window_evaluator_;
  std::shared_ptr<iap::BSplineTrajectory> latest_trajectory_;
  std::unique_ptr<iap::GnssEpochBuilder> gnss_epoch_builder_;
  std::unique_ptr<iap::GnssHandler> gnss_handler_;
  std::deque<iap::GnssRawObservationBatch> pending_raw_gnss_batches_;
  std::size_t ct_target_revision_ = 0;
  bool lidar_baseline_csv_header_written_ = false;
  bool lidar_baseline_csv_first_row_logged_ = false;
  bool frontend_frame_profile_header_written_ = false;
  bool frontend_lidar_factor_profile_header_written_ = false;
  bool frontend_lm_iteration_header_written_ = false;
  bool frame_warning_profile_header_written_ = false;
  bool frontend_iteration_without_frame_warned_ = false;
  bool target_map_breakdown_without_frame_warned_ = false;
  bool graph_problem_size_without_frame_warned_ = false;
#ifdef GTSAM_POINTS_USE_CUDA
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> ct_lidar_gpu_stream_buffers_;
#endif
};

}  // namespace glim
