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
#include <iap/odometry/bspline_graph_solver.hpp>
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

enum class BSplineFinalPoseSurface {
  ACTIVE_WINDOW,
  STRICT_LOCAL,
};

enum class BSplineGravityMode {
  NORMAL,
  FIXED_NORM,
  LIMITED_TILT,
  WARMUP_FREEZE_THEN_RELEASE,
};

enum class BSplineGravityStateMode {
  SHARED_OPTIMIZED,
  EXTERNAL_REFERENCE,
};

enum class BSplineGravityReferenceSource {
  STARTUP_SEED,
  CONFIG_VECTOR,
};

enum class BSplineVelocityStateMode {
  OPTIMIZE,
  KEEP_BUT_NOT_OPTIMIZE,
};

enum class BSplineVelocityModePolicy {
  ALWAYS_OPTIMIZE,
  AUTO_DISABLE_WITHOUT_GNSS,
};

enum class BSplineBiasStateMode {
  SHARED_SINGLETON,
  LAGGED_KEYED,
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
    std::size_t voxel_count = 0;
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

 public:
  struct FrameWarningProfileRow {
    int frame_id = -1;
    double stamp = 0.0;
    std::size_t warning_count = 0;
    std::string warning_categories;
    std::string top_warning_message;
  };

  struct JumpDiagnosticsRow {
    int frame_id{-1};
    double frame_stamp{0.0};
    double raw_frame_stamp{0.0};
    double scan_begin_time{0.0};
    double scan_end_time{0.0};
    double representative_time{0.0};
    double bucket_representative_time{0.0};
    double start_pose_query_time{0.0};
    double frontend_pose_query_time{0.0};
    long long current_segment_id{-1};
    std::string start_pose_source_kind{"unknown"};
    bool start_pose_frozen_before_factor_injection{false};
    bool start_pose_frozen_before_solver_update{false};
    std::string frontend_seed_mode{"last_pose_copy"};
    std::string frontend_seed_source{"last_pose_copy"};
    bool frontend_seed_fallback_used{false};
    std::size_t frontend_seed_imu_sample_count{0};

    double start_pose_tx{0.0};
    double start_pose_ty{0.0};
    double start_pose_tz{0.0};
    double start_pose_qx{0.0};
    double start_pose_qy{0.0};
    double start_pose_qz{0.0};
    double start_pose_qw{1.0};

    double frontend_pose_tx{0.0};
    double frontend_pose_ty{0.0};
    double frontend_pose_tz{0.0};
    double frontend_pose_qx{0.0};
    double frontend_pose_qy{0.0};
    double frontend_pose_qz{0.0};
    double frontend_pose_qw{1.0};

    double post_solve_query_pose_tx{0.0};
    double post_solve_query_pose_ty{0.0};
    double post_solve_query_pose_tz{0.0};
    double post_solve_query_pose_qx{0.0};
    double post_solve_query_pose_qy{0.0};
    double post_solve_query_pose_qz{0.0};
    double post_solve_query_pose_qw{1.0};

    double postsolve_strict_local_pose_tx{0.0};
    double postsolve_strict_local_pose_ty{0.0};
    double postsolve_strict_local_pose_tz{0.0};
    double postsolve_strict_local_pose_qx{0.0};
    double postsolve_strict_local_pose_qy{0.0};
    double postsolve_strict_local_pose_qz{0.0};
    double postsolve_strict_local_pose_qw{1.0};

    double final_pose_tx{0.0};
    double final_pose_ty{0.0};
    double final_pose_tz{0.0};
    double final_pose_qx{0.0};
    double final_pose_qy{0.0};
    double final_pose_qz{0.0};
    double final_pose_qw{1.0};

    double delta_start_to_frontend_translation_norm{0.0};
    double delta_start_to_frontend_rotation_rad{0.0};
    double delta_start_to_frontend_pitch_rad{0.0};
    double delta_start_to_frontend_roll_rad{0.0};
    double delta_frontend_to_postsolve_query_translation_norm{0.0};
    double delta_frontend_to_postsolve_query_rotation_rad{0.0};
    double delta_frontend_to_postsolve_query_pitch_rad{0.0};
    double delta_frontend_to_postsolve_query_roll_rad{0.0};
    double delta_frontend_to_postsolve_strict_local_translation_norm{0.0};
    double delta_frontend_to_postsolve_strict_local_rotation_rad{0.0};
    double delta_frontend_to_postsolve_strict_local_pitch_rad{0.0};
    double delta_frontend_to_postsolve_strict_local_roll_rad{0.0};
    double delta_postsolve_query_to_final_translation_norm{0.0};
    double delta_postsolve_query_to_final_rotation_rad{0.0};
    double delta_postsolve_query_to_final_pitch_rad{0.0};
    double delta_postsolve_query_to_final_roll_rad{0.0};
    double delta_postsolve_strict_local_to_final_translation_norm{0.0};
    double delta_postsolve_strict_local_to_final_rotation_rad{0.0};
    double delta_postsolve_strict_local_to_final_pitch_rad{0.0};
    double delta_postsolve_strict_local_to_final_roll_rad{0.0};
    double delta_postsolve_active_window_to_postsolve_strict_local_translation_norm{0.0};
    double delta_postsolve_active_window_to_postsolve_strict_local_rotation_rad{0.0};
    double delta_postsolve_active_window_to_postsolve_strict_local_pitch_rad{0.0};
    double delta_postsolve_active_window_to_postsolve_strict_local_roll_rad{0.0};
    double delta_frontend_to_final_translation_norm{0.0};
    double delta_frontend_to_final_rotation_rad{0.0};
    double delta_frontend_to_final_yaw_rad{0.0};
    double delta_frontend_to_final_pitch_rad{0.0};
    double delta_frontend_to_final_roll_rad{0.0};
    double delta_frontend_to_final_dx{0.0};
    double delta_frontend_to_final_dy{0.0};
    double delta_frontend_to_final_dz{0.0};
    double current_velocity_norm{0.0};
    double current_velocity_heading_rad{0.0};
    bool current_velocity_heading_valid{false};
    std::size_t velocity_factor_count{0};
    std::size_t prior_factor_count{0};
    bool uses_shared_imu_state{false};
    double frontend_world_to_lidar_yaw{0.0};
    double frontend_world_to_imu_yaw{0.0};
    double final_world_to_lidar_yaw{0.0};
    double final_world_to_imu_yaw{0.0};
    double lidar_to_imu_extrinsic_yaw{0.0};
    std::string yaw_chain_consistency_flag{"none"};
    double gyro_bias_norm{0.0};
    double accel_bias_norm{0.0};
    double gravity_world_x{0.0};
    double gravity_world_y{0.0};
    double gravity_world_z{0.0};
    double gravity_dir_tilt_rad{0.0};

    double lidar_layout_domain_begin{0.0};
    double lidar_layout_domain_end{0.0};
    std::size_t start_pose_support_key_count{0};
    std::string start_pose_support_keys_summary;
    bool start_pose_support_mismatch_flag{false};
    std::string start_pose_support_mismatch_reason{"none"};
    std::size_t lidar_support_key_count{0};
    std::string lidar_support_keys_summary;
    std::size_t frontend_pose_support_key_count{0};
    std::string frontend_pose_support_keys_summary;
    std::size_t postsolve_query_support_key_count{0};
    std::string postsolve_query_support_keys_summary;
    std::string postsolve_query_layout_name;
    std::string postsolve_query_support_mismatch_reason{"none"};
    std::size_t postsolve_strict_local_support_key_count{0};
    std::string postsolve_strict_local_support_keys_summary;
    std::string postsolve_strict_local_layout_name;
    std::string postsolve_strict_local_support_mismatch_reason{"none"};
    std::size_t strict_local_query_support_key_count{0};
    std::string strict_local_query_support_keys_summary;
    std::string strict_local_query_reason{"none"};

    double match_ratio{0.0};
    double inlier_ratio{0.0};
    std::size_t points_in_bucket{0};
    std::size_t candidate_correspondence_count{0};
    std::size_t accepted_correspondence_count{0};
    double accept_ratio{0.0};
    double registration_delta_translation_norm{0.0};
    double registration_delta_rotation_rad{0.0};
    double factor_total_ms{0.0};
    double target_map_prep_ms{0.0};
    std::size_t target_point_count{0};
    std::size_t target_voxel_count{0};
    double target_snapshot_clone_ms{0.0};
    double target_voxel_lookup_prep_ms{0.0};
    double target_covariance_prep_ms{0.0};
    double source_to_target_transform_ms{0.0};

    double solver_update_ms{0.0};
    std::size_t reeliminated_variable_count{0};
    std::size_t relinearized_pose_variable_count{0};
    std::size_t relinearized_aux_variable_count{0};
    std::size_t relinearized_shared_variable_count{0};
    std::size_t recalculated_velocity_factor_count{0};
    std::size_t recalculated_prior_factor_count{0};
    std::size_t recalculated_imu_factor_count{0};
    std::size_t recalculated_lidar_factor_count{0};
    std::size_t recalculated_lidar_current_segment_factor_count{0};
    std::size_t recalculated_lidar_same_support_factor_count{0};
    std::size_t recalculated_lidar_cross_support_factor_count{0};

    double pose_guess_translation_norm{0.0};
    double pose_guess_rotation_rad{0.0};
    std::string carried_boundary_oldest_key_summary;
    std::string oldest_survivor_key_summary;
    bool uses_local_lidar_layout_override{false};
    std::string frontend_pose_query_support_keys_summary;
  };

 private:
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
    std::size_t target_voxel_count = 0;
    std::size_t snapshot_frame_count = 0;
    std::size_t snapshot_point_count = 0;
    double snapshot_span_sec = 0.0;
    bool snapshot_policy_accepted = false;
    double target_build_ms = 0.0;
    double target_snapshot_clone_ms = 0.0;
    double target_voxel_lookup_prep_ms = 0.0;
    double target_covariance_prep_ms = 0.0;
    double source_to_target_transform_ms = 0.0;
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
  EstimationFrame::ConstPtr insert_frame_ct_lidar_legacy_two_stage(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  EstimationFrame::ConstPtr insert_frame_ct_lidar_unified_graph(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  EstimationFrame::ConstPtr insert_frame_ct_lidar_incremental_graph(
    const PreprocessedFrame::Ptr& frame,
    std::vector<EstimationFrame::ConstPtr>& marginalized_frames);
  void initialize_control_window(const PreprocessedFrame::Ptr& raw_frame, const gtsam::Pose3& initial_pose);
  gtsam::Pose3 predict_scan_end_pose(double scan_duration) const;
  gtsam_points::PointCloud::ConstPtr create_lidar_source_cloud(const PreprocessedFrame::Ptr& raw_frame) const;
  ActiveSplineTargetReference create_active_target_reference() const;
  std::vector<ActiveSplineIMUSample> create_segment_imu_samples(const PreprocessedFrame::Ptr& raw_frame) const;
  std::shared_ptr<const iap::SplineStateLayout> build_layout_from_controls_and_knots(
    std::vector<iap::BSplineControlPointState> controls,
    std::vector<double> knots) const;
  std::shared_ptr<const iap::SplineStateLayout> build_active_window_layout() const;
  std::shared_ptr<const iap::SplineStateLayout> build_segment_local_layout(
    const ActiveSplineSegmentConstraint& segment) const;
  std::shared_ptr<const iap::SplineStateLayout> build_strict_segment_local_layout(
    const ActiveSplineSegmentConstraint& segment) const;
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
  void maybe_write_solver_update_profile(const iap::SolverUpdateProfileRow& row);
  void maybe_write_lidar_factor_profiles(
    int frame_id,
    double stamp,
    const std::vector<iap::FrontendBucketProfileRow>& profiles);
  void maybe_write_lidar_factor_internal_profiles(const std::vector<iap::LidarFactorInternalProfileRow>& rows);
  void maybe_write_frontend_lm_iterations(
    int frame_id,
    double stamp,
    const std::vector<iap::FrontendLMIterationProfileRow>& iterations);
  void maybe_write_frame_warning_profile(const FrameWarningProfileRow& row);
  void maybe_write_jump_diagnostics(const JumpDiagnosticsRow& row);
  void maybe_log_jump_event(const JumpDiagnosticsRow& row) const;
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
  iap::CTLocalFrontend::LayerInput make_local_layer_input(
    const gtsam::Values& values,
    std::shared_ptr<const iap::SplineStateLayout> factor_layout) const;
  iap::CTLocalFrontend::LayerInput make_local_layer_delta_input(
    const ActiveSplineSegmentConstraint& segment,
    std::shared_ptr<const iap::SplineStateLayout> factor_layout) const;
  // Build the CTCompactBackend::Input from the local frontend result and shared GNSS state.
  iap::CTCompactBackend::Input make_backend_input(const iap::CTLocalFrontendResult& local_result) const;
  iap::CTCompactBackend::LayerInput make_navigation_layer_input(
    std::shared_ptr<const iap::SplineStateLayout> factor_layout,
    bool navigation_layer_enabled) const;
  iap::CTCompactBackend::LayerInput make_navigation_layer_delta_input(
    const ActiveSplineSegmentConstraint& current_segment,
    const ActiveSplineSegmentConstraint* previous_segment,
    std::shared_ptr<const iap::SplineStateLayout> factor_layout,
    const gtsam::KeyVector& existing_keys,
    bool navigation_layer_enabled) const;
  bool gravity_graph_key_enabled() const;
  bool gravity_external_reference_enabled() const;
  bool bias_shared_singleton_mode() const;
  bool bias_lagged_keyed_mode() const;
  gtsam::Vector3 sanitized_gravity_vector(const gtsam::Vector3& gravity) const;
  void set_external_gravity_reference(
    const gtsam::Vector3& gravity_world,
    BSplineGravityReferenceSource source);
  gtsam::Vector3 authoritative_gravity_reference() const;
  const char* runtime_gravity_reference_source_name() const;
  const char* runtime_bias_source_of_truth_name() const;
  bool runtime_bias_transition_prior_enabled() const;
  double runtime_bias_transition_prior_strength() const;
  bool runtime_bias_can_be_survivor_anchor() const;
  const char* runtime_bias_writeback_mode_name() const;
  const char* runtime_frontend_seed_mode_name() const;
  const char* runtime_frontend_seed_source_name() const;
  bool imu_forward_prediction_enabled() const;
  struct FrontendSeedObservation {
    std::string mode{"last_pose_copy"};
    std::string source{"last_pose_copy"};
    bool fallback_used{false};
    std::size_t imu_sample_count{0};
  };
  FrontendSeedObservation inspect_frontend_seed_observation(double seed_end_stamp) const;
  void record_frontend_seed_observation(const FrontendSeedObservation& observation) const;
  gtsam::Key gyro_bias_key_for_auxiliary(std::size_t auxiliary_index) const;
  gtsam::Key accel_bias_key_for_auxiliary(std::size_t auxiliary_index) const;
  bool authoritative_previous_bias_auxiliary_index(
    std::size_t auxiliary_index,
    std::size_t* previous_auxiliary_index) const;
  gtsam::KeyVector bias_anchorable_survivor_keys(const gtsam::KeyVector& keys) const;
  void seed_bias_values_for_auxiliary(std::size_t auxiliary_index, gtsam::Values* values) const;
  void seed_bias_values_for_active_segments(gtsam::Values* values) const;
  void mirror_bias_cache_from_values(const gtsam::Values& values);
  void write_frame_bias_from_values(
    const gtsam::Values& values,
    std::size_t auxiliary_index,
    EstimationFrame* frame) const;
  bool solve_has_gnss_constraints(const ActiveSplineSegmentConstraint& segment) const;
  bool solve_has_gnss_constraints() const;
  bool velocity_explicit_optimization_enabled(bool solve_has_gnss_constraints) const;
  bool velocity_optimized_for_solve(bool solve_has_gnss_constraints) const;
  BSplineVelocityStateMode resolved_velocity_state_mode(bool solve_has_gnss_constraints) const;
  void record_runtime_mode_resolution(
    bool solve_has_gnss_constraints,
    bool velocity_explicit_optimization_enabled,
    bool velocity_optimized);
  void sync_runtime_mode_metadata(bool force = false) const;
  bool any_shared_imu_freeze_experiment_enabled() const;
  bool any_soft_gravity_experiment_enabled() const;
  bool any_gravity_isolation_experiment_enabled() const;
  bool any_yaw_isolation_experiment_enabled() const;
  bool gravity_warmup_freeze_active(int frame_id) const;
  void maybe_latch_isolation_freeze_anchor(const iap::BSplineFixedLagSharedState& candidate);
  iap::BSplineFixedLagSharedState effective_shared_imu_state(int frame_id) const;
  iap::BSplineFixedLagSharedState effective_shared_imu_state(
    const iap::BSplineFixedLagSharedState& candidate,
    int frame_id) const;
  void apply_effective_shared_imu_state_to_registry(int frame_id);
  void enforce_frozen_shared_values(gtsam::Values* values, int frame_id) const;
  std::string runtime_experiment_name() const;
  void reset_unified_graph_solver();
  std::vector<iap::CTLocalFrontend::IMUSample> create_frontend_seed_imu_samples(
    double start_stamp,
    double end_stamp) const;

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
  bool use_legacy_bspline_two_stage_path_ = false;
  iap::BSplineUnifiedSolverMode unified_solver_mode_ = iap::BSplineUnifiedSolverMode::BATCH_LM;
  BSplineFinalPoseSurface final_pose_surface_ = BSplineFinalPoseSurface::STRICT_LOCAL;
  BSplineGravityStateMode gravity_state_mode_ = BSplineGravityStateMode::SHARED_OPTIMIZED;
  BSplineGravityReferenceSource gravity_reference_source_ = BSplineGravityReferenceSource::STARTUP_SEED;
  gtsam::Vector3 gravity_reference_vector_config_ = gtsam::Vector3(0.0, 0.0, 9.80665);
  gtsam::Vector3 external_gravity_reference_ = gtsam::Vector3(0.0, 0.0, 9.80665);
  BSplineGravityMode gravity_mode_ = BSplineGravityMode::NORMAL;
  BSplineBiasStateMode bias_state_mode_ = BSplineBiasStateMode::SHARED_SINGLETON;
  gtsam::Vector3 bias_bootstrap_gyro_ = gtsam::Vector3::Zero();
  gtsam::Vector3 bias_bootstrap_accel_ = gtsam::Vector3::Zero();
  BSplineVelocityStateMode velocity_state_mode_ = BSplineVelocityStateMode::OPTIMIZE;
  BSplineVelocityModePolicy velocity_mode_policy_ = BSplineVelocityModePolicy::ALWAYS_OPTIMIZE;
  iap::CTLocalFrontend::FrontendSeedMode frontend_seed_mode_ =
    iap::CTLocalFrontend::FrontendSeedMode::LAST_POSE_COPY;
  mutable bool runtime_has_gnss_constraints_ = false;
  mutable bool runtime_bias_optimized_ = true;
  mutable bool runtime_velocity_optimized_ = true;
  mutable BSplineVelocityStateMode runtime_velocity_state_mode_ = BSplineVelocityStateMode::OPTIMIZE;
  mutable bool runtime_frontend_seed_fallback_used_ = false;
  mutable bool runtime_frontend_seed_observed_ = false;
  mutable bool runtime_frontend_seed_imu_success_observed_ = false;
  mutable std::size_t runtime_frontend_seed_imu_sample_count_ = 0;
  mutable std::string last_runtime_mode_metadata_signature_;
  double gravity_fixed_norm_value_ = 9.80665;
  double gravity_tilt_limit_rad_ = 0.02;
  int gravity_warmup_freeze_frames_ = 20;
  bool exp_freeze_gravity_ = false;
  bool exp_freeze_gyro_bias_ = false;
  bool exp_freeze_accel_bias_ = false;
  bool exp_disable_velocity_factor_ = false;
  bool exp_disable_current_velocity_prior_ = false;
  bool isolation_freeze_anchor_latched_ = false;
  iap::BSplineFixedLagSharedState isolation_freeze_anchor_;
  bool last_applied_gravity_initialized_ = false;
  gtsam::Vector3 last_applied_gravity_ = gtsam::Vector3(0.0, 0.0, 9.80665);
  double isolation_freeze_prior_precision_ = 1e12;
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
  bool solver_update_profile_enabled_ = false;
  bool lidar_factor_internal_profile_enabled_ = false;
  bool frontend_lm_iteration_profile_enabled_ = false;
  bool frame_warning_profile_enabled_ = false;
  bool jump_diagnostics_enabled_ = false;
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
  std::string solver_update_profile_csv_path_ = "solver_update_profile.csv";
  std::string lidar_factor_internal_profile_csv_path_ = "lidar_factor_internal_profile.csv";
  std::string frontend_lm_iteration_csv_path_ = "frontend_lm_iteration.csv";
  std::string frame_warning_profile_csv_path_ = "frame_warning_profile.csv";
  std::string jump_diagnostics_csv_path_ = "jump_diagnostics.csv";
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
  std::unique_ptr<iap::IBSplineGraphSolver> unified_graph_solver_;
  std::unique_ptr<iap::GnssEpochBuilder> gnss_epoch_builder_;
  std::unique_ptr<iap::GnssHandler> gnss_handler_;
  std::deque<iap::GnssRawObservationBatch> pending_raw_gnss_batches_;
  std::size_t ct_target_revision_ = 0;
  bool lidar_baseline_csv_header_written_ = false;
  bool lidar_baseline_csv_first_row_logged_ = false;
  bool frontend_frame_profile_header_written_ = false;
  bool frontend_lidar_factor_profile_header_written_ = false;
  bool solver_update_profile_header_written_ = false;
  bool lidar_factor_internal_profile_header_written_ = false;
  bool frontend_lm_iteration_header_written_ = false;
  bool frame_warning_profile_header_written_ = false;
  bool jump_diagnostics_header_written_ = false;
  bool frontend_iteration_without_frame_warned_ = false;
  bool target_map_breakdown_without_frame_warned_ = false;
  bool graph_problem_size_without_frame_warned_ = false;
#ifdef GTSAM_POINTS_USE_CUDA
  std::unique_ptr<gtsam_points::StreamTempBufferRoundRobin> ct_lidar_gpu_stream_buffers_;
#endif
};

}  // namespace glim
