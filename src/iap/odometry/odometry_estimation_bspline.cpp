#include <iap/odometry/odometry_estimation_bspline.hpp>
// Commit 0 migration boundary:
// This translation unit still executes the existing fixed 4-control-point local
// spline window path. Follow-up work will introduce an explicit knot vector and
// a unified spline evaluator shared by IMU/GNSS/LiDAR factor assembly. For this
// commit we only freeze the migration boundary and CPU-first frontend default;
// math behavior, residual models, public interfaces, plugin names, shared-state
// integration, ROS topics, and log keywords stay unchanged.

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <spdlog/spdlog.h>

#include <iap/common/cloud_covariance_estimation.hpp>
#include <iap/common/imu_integration.hpp>
#include <iap/odometry/initial_state_estimation.hpp>
#include <iap/util/config.hpp>
#include <iap/util/shared_state.hpp>
#include <iap/odometry/callbacks.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor_gpu.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp>
#endif

namespace glim {

namespace {

using Callbacks = OdometryEstimationCallbacks;

template <typename T>
void hash_combine(std::size_t* seed, const T& value) {
  const std::size_t hashed = std::hash<T>{}(value);
  *seed ^= hashed + 0x9e3779b97f4a7c15ULL + (*seed << 6U) + (*seed >> 2U);
}

iap::SplineKnotMode parse_knot_mode(const std::string& mode) {
  if (mode == "non_uniform" || mode == "non-uniform" || mode == "NON_UNIFORM") {
    return iap::SplineKnotMode::NonUniform;
  }
  return iap::SplineKnotMode::Uniform;
}

BSplineLidarTargetMode parse_lidar_target_mode(const std::string& mode) {
  if (mode == "GLOBAL_IVOX_REFERENCE" || mode == "global_ivox_reference" || mode == "global_ivox") {
    return BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
  }
  return BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
}

const char* to_string(BSplineLidarTargetMode mode) {
  switch (mode) {
    case BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT:
      return "active_window_snapshot";
    case BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE:
      return "global_ivox_reference";
  }
  return "unknown";
}

BSplineGpuLidarBackend parse_gpu_lidar_backend(const std::string& mode) {
  if (mode == "KERNEL" || mode == "kernel" || mode == "ct_kernel") {
    return BSplineGpuLidarBackend::KERNEL;
  }
  return BSplineGpuLidarBackend::BUCKET;
}

const char* to_string(BSplineGpuLidarBackend backend) {
  switch (backend) {
    case BSplineGpuLidarBackend::BUCKET:
      return "bucket";
    case BSplineGpuLidarBackend::KERNEL:
      return "kernel";
  }
  return "unknown";
}

iap::IntegratedBSplineGICPFactor::JacobianMode parse_lidar_jacobian_mode(const std::string& mode) {
  if (mode == "NUMERIC_FULL" || mode == "numeric_full" || mode == "numeric") {
    return iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL;
  }
  return iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC;
}

const char* to_string(iap::IntegratedBSplineGICPFactor::JacobianMode mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL:
      return "numeric_full";
    case iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC:
      return "semi_analytic";
  }
  return "unknown";
}

#ifdef GTSAM_POINTS_USE_CUDA
iap::IntegratedBSplineGICPFactorGPU::JacobianMode to_gpu_lidar_jacobian_mode(
  iap::IntegratedBSplineGICPFactor::JacobianMode mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL:
      return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::NUMERIC_FULL;
    case iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC:
      return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC;
  }
  return iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC;
}
#endif

iap::IntegratedBSplineGICPFactor::RobustKernel parse_lidar_robust_kernel(const std::string& mode) {
  if (mode == "HUBER" || mode == "huber") {
    return iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER;
  }
  if (mode == "CAUCHY" || mode == "cauchy") {
    return iap::IntegratedBSplineGICPFactor::RobustKernel::CAUCHY;
  }
  return iap::IntegratedBSplineGICPFactor::RobustKernel::NONE;
}

const char* to_string(iap::IntegratedBSplineGICPFactor::RobustKernel mode) {
  switch (mode) {
    case iap::IntegratedBSplineGICPFactor::RobustKernel::NONE:
      return "none";
    case iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER:
      return "huber";
    case iap::IntegratedBSplineGICPFactor::RobustKernel::CAUCHY:
      return "cauchy";
  }
  return "unknown";
}

iap::GnssHandler::Params make_gnss_handler_params(
  double min_elevation,
  double pr_noise_base,
  double dop_noise_base,
  double elev_noise_exp,
  double time_tolerance,
  const Eigen::Vector3d& lever_arm,
  const iap::CanopyNoiseParams& canopy_params) {
  iap::GnssHandler::Params params;
  params.pr_noise_base = pr_noise_base;
  params.dop_noise_base = dop_noise_base;
  params.elev_noise_exp = elev_noise_exp;
  params.time_tolerance = time_tolerance;
  params.min_elevation = min_elevation;
  params.lever_arm = lever_arm;
  params.canopy = canopy_params;
  return params;
}

iap::GnssEpochBuilder::Params make_gnss_epoch_builder_params(
  double min_elevation,
  double pr_noise_base,
  double dop_noise_base) {
  iap::GnssEpochBuilder::Params params;
  params.min_elevation = min_elevation;
  params.default_pr_sigma = pr_noise_base;
  params.default_dop_sigma = dop_noise_base;
  return params;
}

double sigma_from_covariance(const Eigen::Matrix3d& sigma_p) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sigma_p, Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success) {
    return 0.0;
  }
  return std::sqrt(std::max(0.0, eig.eigenvalues().maxCoeff()));
}

gtsam::Key bspline_gyro_bias_key() {
  return gtsam::symbol('j', 0);
}

gtsam::Key bspline_accel_bias_key() {
  return gtsam::symbol('k', 0);
}

gtsam::Key bspline_gravity_key() {
  return gtsam::symbol('g', 0);
}

gtsam::KeyVector make_key_vector(const std::array<gtsam::Key, iap::kBSplineControlPointCount>& keys) {
  return gtsam::KeyVector(keys.begin(), keys.end());
}

gtsam::KeyVector make_key_vector(
  const std::array<gtsam::Key, iap::kBSplineControlPointCount>& keys,
  std::initializer_list<gtsam::Key> extra_keys) {
  gtsam::KeyVector result(keys.begin(), keys.end());
  result.insert(result.end(), extra_keys.begin(), extra_keys.end());
  return result;
}

}  // namespace

OdometryEstimationBSplineParams::OdometryEstimationBSplineParams() : OdometryEstimationCPUParams() {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  spline_knot_mode = config.param<std::string>("odometry_estimation", "spline_knot_mode", "uniform");
  spline_nominal_dt = config.param<double>("odometry_estimation", "spline_nominal_dt", 0.0);
  spline_finite_difference_dt = config.param<double>("odometry_estimation", "spline_finite_difference_dt", 0.01);
  compatibility_sample_dt = config.param<double>("odometry_estimation", "compatibility_sample_dt", 0.01);
  publish_shared_trajectory = config.param<bool>("odometry_estimation", "publish_shared_trajectory", true);
  attach_trajectory_to_frames = config.param<bool>("odometry_estimation", "attach_trajectory_to_frames", true);
}

OdometryEstimationBSplineParams::~OdometryEstimationBSplineParams() {}

OdometryEstimationBSpline::OdometryEstimationBSpline(const OdometryEstimationBSplineParams& params)
: OdometryEstimationCPU(params),
  compatibility_sample_dt_(params.compatibility_sample_dt),
  publish_shared_trajectory_(params.publish_shared_trajectory),
  attach_trajectory_to_frames_(params.attach_trajectory_to_frames) {
  Config config(GlobalConfig::get_config_path("config_odometry"));
  trajectory_params_.knot_mode = parse_knot_mode(params.spline_knot_mode);
  trajectory_params_.nominal_dt = params.spline_nominal_dt;
  trajectory_params_.finite_difference_dt = params.spline_finite_difference_dt;
  trajectory_params_.order = 3;
  frontend_mode_ = config.param<std::string>("odometry_estimation", "frontend_mode", "CT_LIDAR_CPU");
  max_correspondence_distance_ = config.param<double>("odometry_estimation", "max_correspondence_distance", 1.5);
  lidar_target_mode_ = parse_lidar_target_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_target_mode", "ACTIVE_WINDOW_SNAPSHOT"));
  lidar_gpu_backend_ = parse_gpu_lidar_backend(
    config.param<std::string>("odometry_estimation", "ct_lidar_gpu_backend", "BUCKET"));
  lidar_jacobian_mode_ = parse_lidar_jacobian_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_jacobian_mode", "SEMI_ANALYTIC"));
  lidar_snapshot_frame_window_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_frame_window", 0);
  lidar_snapshot_min_frames_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_min_frames", 2);
  lidar_snapshot_min_points_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_min_points", 64);
  lidar_snapshot_max_age_ = config.param<double>("odometry_estimation", "ct_lidar_snapshot_max_age", 0.0);
  lidar_correspondence_candidate_count_ =
    config.param<int>("odometry_estimation", "ct_lidar_correspondence_candidates", 3);
  lidar_correspondence_accept_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_correspondence_accept_ratio", 0.0);
  lidar_correspondence_min_score_gap_ =
    config.param<double>("odometry_estimation", "ct_lidar_correspondence_min_score_gap", 0.0);
  lidar_jacobian_numeric_eps_ = config.param<double>("odometry_estimation", "ct_lidar_jacobian_numeric_eps", 1e-4);
  lidar_outlier_mahalanobis_thresh_ =
    config.param<double>("odometry_estimation", "ct_lidar_outlier_mahalanobis_thresh", 0.0);
  lidar_robust_kernel_ = parse_lidar_robust_kernel(
    config.param<std::string>("odometry_estimation", "ct_lidar_robust_kernel", "NONE"));
  lidar_robust_kernel_width_ = config.param<double>("odometry_estimation", "ct_lidar_robust_kernel_width", 1.0);
  lidar_robust_weight_floor_ = config.param<double>("odometry_estimation", "ct_lidar_robust_weight_floor", 0.0);
  lidar_factor_profile_ = config.param<bool>("odometry_estimation", "ct_lidar_profile_factor", false);
  lidar_validate_linearization_ = config.param<bool>("odometry_estimation", "ct_lidar_validate_linearization", false);
  lidar_profile_numeric_reference_ =
    config.param<bool>("odometry_estimation", "ct_lidar_profile_numeric_reference", false);
  pipeline_profile_ = config.param<bool>("odometry_estimation", "ct_profile_pipeline", false);
  lidar_warn_degeneracy_ = config.param<bool>("odometry_estimation", "ct_lidar_warn_degeneracy", true);
  lidar_export_baseline_csv_ = config.param<bool>("odometry_estimation", "ct_lidar_export_baseline_csv", false);
  lidar_linearization_check_scale_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_check_scale", 1e-4);
  lidar_linearization_warn_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_warn_ratio", 0.25);
  lidar_numeric_reference_scale_ =
    config.param<double>("odometry_estimation", "ct_lidar_numeric_reference_scale", 1e-5);
  lidar_baseline_csv_path_ = config.param<std::string>(
    "odometry_estimation",
    "ct_lidar_baseline_csv_path",
    "/tmp/iap_ct_lidar_baseline.csv");
  lidar_degeneracy_thresholds_.min_match_ratio =
    config.param<double>("odometry_estimation", "ct_lidar_warn_min_match_ratio", 0.0);
  lidar_degeneracy_thresholds_.min_inlier_ratio =
    config.param<double>("odometry_estimation", "ct_lidar_warn_min_inlier_ratio", 0.0);
  lidar_degeneracy_thresholds_.min_unique_target_ratio =
    config.param<double>("odometry_estimation", "ct_lidar_warn_min_unique_target_ratio", 0.0);
  lidar_degeneracy_thresholds_.max_target_reuse_ratio =
    config.param<double>("odometry_estimation", "ct_lidar_warn_max_target_reuse_ratio", 0.0);
  lidar_degeneracy_thresholds_.max_ambiguity_rejection_ratio =
    config.param<double>("odometry_estimation", "ct_lidar_warn_max_ambiguity_rejection_ratio", 0.0);
  lidar_degeneracy_thresholds_.min_mean_score_gap =
    config.param<double>("odometry_estimation", "ct_lidar_warn_min_mean_score_gap", 0.0);
  ctrl_point_anchor_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_anchor_inf_scale", 1e6);
  ctrl_point_prediction_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_prediction_inf_scale", 1e3);
  ctrl_point_smoothness_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_smoothness_inf_scale", 1e2);
  ctrl_point_marginal_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_marginal_inf_scale", 1e4);
  imu_ct_trans_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_trans_inf_scale", 10.0);
  imu_ct_rot_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_rot_inf_scale", 100.0);
  imu_ct_bias_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_bias_inf_scale", 1e3);
  imu_ct_gravity_inf_scale_ = config.param<double>("odometry_estimation", "imu_ct_gravity_inf_scale", 1e3);
  velocity_ct_inf_scale_ = config.param<double>("odometry_estimation", "velocity_ct_inf_scale", 1e3);
  imu_ct_sample_stride_ = config.param<int>("odometry_estimation", "imu_ct_sample_stride", 4);
  lm_max_iterations_ = config.param<int>("odometry_estimation", "lm_max_iterations", 8);

  Config gnss_config(GlobalConfig::get_config_path("config_gnss"));
  gnss_time_tolerance_ = gnss_config.param<double>("gnss", "time_tolerance", 0.1);
  gnss_min_elevation_ = gnss_config.param<double>("gnss", "min_elevation_deg", 10.0) * M_PI / 180.0;
  gnss_pr_noise_base_ = gnss_config.param<double>("gnss", "pr_noise_base", 5.0);
  gnss_dop_noise_base_ = gnss_config.param<double>("gnss", "dop_noise_base", 0.5);
  gnss_elev_noise_exp_ = gnss_config.param<double>("gnss", "elev_noise_exp", 2.0);
  gnss_sigma_ecef_origin_ = gnss_config.param<double>("gnss", "sigma_ecef_origin", 5.0);
  gnss_sigma_ecef_rot_ = gnss_config.param<double>("gnss", "sigma_ecef_rot", 0.087);
  gnss_lever_arm_ = gnss_config.param<Eigen::Vector3d>("gnss", "lever_arm", Eigen::Vector3d::Zero());
  gnss_canopy_params_.sigma_0 = gnss_config.param<double>("gnss", "canopy_sigma_0", 1.0);
  gnss_canopy_params_.sigma_mp = gnss_config.param<double>("gnss", "canopy_sigma_mp", 0.5);
  gnss_canopy_params_.sigma_c = gnss_config.param<double>("gnss", "canopy_sigma_c", 5.0);
  gnss_canopy_params_.alpha = gnss_config.param<double>("gnss", "canopy_alpha", 2.0);
  gnss_clock_between_params_.q_bias = gnss_config.param<double>("gnss", "clock_q_bias", 1.0);
  gnss_clock_between_params_.q_drift = gnss_config.param<double>("gnss", "clock_q_drift", 0.1);

  T_lidar_imu = params.T_lidar_imu;
  T_imu_lidar = T_lidar_imu.inverse();
  control_window_ = std::make_unique<iap::BSplineControlWindow>();
  fixed_lag_registry_.set_shared_imu_state(
    params.imu_bias.tail<3>(),
    params.imu_bias.head<3>(),
    Eigen::Vector3d::UnitZ() * 9.80665);
  gnss_epoch_builder_ = std::make_unique<iap::GnssEpochBuilder>(make_gnss_epoch_builder_params(
    gnss_min_elevation_,
    gnss_pr_noise_base_,
    gnss_dop_noise_base_));
  gnss_handler_ = std::make_unique<iap::GnssHandler>(make_gnss_handler_params(
    gnss_min_elevation_,
    gnss_pr_noise_base_,
    gnss_dop_noise_base_,
    gnss_elev_noise_exp_,
    gnss_time_tolerance_,
    gnss_lever_arm_,
    gnss_canopy_params_));
  ct_target_ivox_ = std::make_shared<gtsam_points::iVox>(params.ivox_resolution);
  ct_target_ivox_->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
  ct_target_ivox_->set_lru_horizon(params.lru_thresh);
  ct_target_ivox_->set_neighbor_voxel_mode(1);
#ifdef GTSAM_POINTS_USE_CUDA
  if (frontend_mode_ == "CT_LIDAR_GPU" && lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
    ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
  }
#endif
  logger->info("odometry_bspline initialized frontend_mode={} lidar_gpu_backend={} knot_mode={} nominal_dt={:.4f} compatibility_sample_dt={:.4f} lidar_target_mode={} lidar_jacobian_mode={} lidar_k_candidates={} lidar_accept_ratio={:.3f} lidar_score_gap={:.3f} lidar_snapshot_window={} lidar_snapshot_min_frames={} lidar_snapshot_min_points={} lidar_snapshot_max_age={:.3f} lidar_outlier_thresh={:.3f} lidar_robust_kernel={} lidar_robust_width={:.3f} lidar_robust_w_floor={:.3f} lidar_profile={} lidar_validate={} ct_pipeline_profile={} lidar_baseline_csv={} lidar_baseline_path={}",
    frontend_mode_,
    ::glim::to_string(lidar_gpu_backend_),
    iap::to_string(trajectory_params_.knot_mode),
    trajectory_params_.nominal_dt,
    compatibility_sample_dt_,
    ::glim::to_string(lidar_target_mode_),
    ::glim::to_string(lidar_jacobian_mode_),
    lidar_correspondence_candidate_count_,
    lidar_correspondence_accept_ratio_,
    lidar_correspondence_min_score_gap_,
    lidar_snapshot_frame_window_,
    lidar_snapshot_min_frames_,
    lidar_snapshot_min_points_,
    lidar_snapshot_max_age_,
    lidar_outlier_mahalanobis_thresh_,
    ::glim::to_string(lidar_robust_kernel_),
    lidar_robust_kernel_width_,
    lidar_robust_weight_floor_,
    lidar_factor_profile_,
    lidar_validate_linearization_,
    pipeline_profile_,
    lidar_export_baseline_csv_,
    lidar_baseline_csv_path_);
}

OdometryEstimationBSpline::~OdometryEstimationBSpline() {
  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(nullptr);
    iap::IapSharedState::instance().set_spline_control_access(nullptr);
    iap::IapSharedState::instance().clear_bspline_fixed_lag_telemetry();
  }
}

void OdometryEstimationBSpline::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  OdometryEstimationIMU::update_frames(current, new_factors);
  publish_continuous_trajectory(current);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame(
  const PreprocessedFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  if (frontend_mode_ == "CT_LIDAR_CPU" || frontend_mode_ == "CT_LIDAR_GPU") {
    return insert_frame_ct_lidar(frame, marginalized_frames);
  }
  return insert_frame_reconstruct(frame, marginalized_frames);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_reconstruct(
  const PreprocessedFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  return OdometryEstimationCPU::insert_frame(frame, marginalized_frames);
}

gtsam_points::PointCloud::ConstPtr OdometryEstimationBSpline::create_lidar_source_cloud(
  const PreprocessedFrame::Ptr& raw_frame) const {
  auto frame_cpu = std::make_shared<gtsam_points::PointCloudCPU>(raw_frame->points);
  frame_cpu->add_times(raw_frame->times);
  covariance_estimation->estimate(raw_frame->points, raw_frame->neighbors, frame_cpu->normals_storage, frame_cpu->covs_storage);
  frame_cpu->normals = frame_cpu->normals_storage.data();
  frame_cpu->covs = frame_cpu->covs_storage.data();
  return frame_cpu;
}

void OdometryEstimationBSpline::initialize_control_window(
  const PreprocessedFrame::Ptr& raw_frame,
  const gtsam::Pose3& initial_pose) {
  control_window_->initialize(raw_frame->stamp, raw_frame->scan_end_time, initial_pose);
  fixed_lag_registry_.reset_from_window(*control_window_);
  marginal_prior_ = ActiveSplineMarginalPrior();
  fixed_lag_registry_.clear_auxiliary_values();
  ct_target_revision_ = 0;
}

gtsam::Pose3 OdometryEstimationBSpline::predict_scan_end_pose(double scan_duration) const {
  if (!control_window_ || !control_window_->initialized()) {
    return gtsam::Pose3();
  }

  const gtsam::Pose3 last_start = control_window_->evaluate(0.0);
  const gtsam::Pose3 last_end = control_window_->evaluate(1.0);
  const double last_duration = std::max(1e-3, control_window_->segment_duration());
  const double scale = scan_duration / last_duration;
  const gtsam::Vector6 delta = gtsam::Pose3::Logmap(last_start.between(last_end));
  return last_end.compose(gtsam::Pose3::Expmap(scale * delta));
}

OdometryEstimationBSpline::ActiveSplineTargetReference OdometryEstimationBSpline::create_active_target_reference() const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  const auto* cpu_params = static_cast<const OdometryEstimationCPUParams*>(params.get());
  ActiveSplineTargetReference target_ref;

  if (lidar_target_mode_ == BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE && ct_target_ivox_ &&
      !ct_target_ivox_->voxel_points().empty()) {
    target_ref.target_snapshot = ct_target_ivox_;
    target_ref.target_tree = target_ref.target_snapshot;
    target_ref.mode = BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
    target_ref.point_count = target_ref.target_snapshot->voxel_points().size();
    target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
    return target_ref;
  }

  auto snapshot = std::make_shared<gtsam_points::iVox>(cpu_params->ivox_resolution);
  snapshot->voxel_insertion_setting().set_min_dist_in_cell(cpu_params->ivox_min_dist);
  snapshot->set_lru_horizon(cpu_params->lru_thresh);
  snapshot->set_neighbor_voxel_mode(1);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  std::size_t first_frame_index = 0;
  if (lidar_snapshot_frame_window_ > 0 && active_frames.size() > static_cast<std::size_t>(lidar_snapshot_frame_window_)) {
    first_frame_index = active_frames.size() - static_cast<std::size_t>(lidar_snapshot_frame_window_);
  }

  bool inserted = false;
  double snapshot_start_stamp = 0.0;
  double snapshot_end_stamp = 0.0;
  for (std::size_t i = first_frame_index; i < active_frames.size(); ++i) {
    if (!active_frames[i] || !active_frames[i]->frame) {
      continue;
    }
    if (lidar_snapshot_max_age_ > 0.0 && !active_frames.empty()) {
      const double latest_stamp = active_frames.back()->stamp;
      if (latest_stamp - active_frames[i]->stamp > lidar_snapshot_max_age_) {
        continue;
      }
    }

    auto transformed = gtsam_points::PointCloudCPU::clone(*active_frames[i]->frame);
    for (int j = 0; j < transformed->size(); ++j) {
      transformed->points[j] = active_frames[i]->T_world_lidar * active_frames[i]->frame->points[j];
      transformed->covs[j] =
        active_frames[i]->T_world_lidar.matrix() * active_frames[i]->frame->covs[j] * active_frames[i]->T_world_lidar.matrix().transpose();
    }
    snapshot->insert(*transformed);
    inserted = true;
    if (target_ref.snapshot_frame_count == 0) {
      snapshot_start_stamp = active_frames[i]->stamp;
    }
    snapshot_end_stamp = active_frames[i]->stamp;
    target_ref.snapshot_frame_count++;
  }

  target_ref.snapshot_point_count = inserted ? snapshot->voxel_points().size() : 0;
  target_ref.snapshot_span_sec =
    target_ref.snapshot_frame_count == 0 ? 0.0 : std::max(0.0, snapshot_end_stamp - snapshot_start_stamp);

  bool snapshot_policy_accepted = inserted;
  if (lidar_snapshot_min_frames_ > 0 &&
      target_ref.snapshot_frame_count < static_cast<std::size_t>(lidar_snapshot_min_frames_)) {
    snapshot_policy_accepted = false;
  }
  if (lidar_snapshot_min_points_ > 0 &&
      target_ref.snapshot_point_count < static_cast<std::size_t>(lidar_snapshot_min_points_)) {
    snapshot_policy_accepted = false;
  }
  if (lidar_snapshot_max_age_ > 0.0 && target_ref.snapshot_span_sec > lidar_snapshot_max_age_) {
    snapshot_policy_accepted = false;
  }
  target_ref.snapshot_policy_accepted = snapshot_policy_accepted;

  const bool global_reference_available = ct_target_ivox_ && !ct_target_ivox_->voxel_points().empty();
  if (snapshot_policy_accepted || (!global_reference_available && inserted)) {
    target_ref.target_snapshot = snapshot;
    target_ref.target_tree = target_ref.target_snapshot;
    target_ref.mode = BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT;
    target_ref.contributing_frames = target_ref.snapshot_frame_count;
    target_ref.point_count = target_ref.snapshot_point_count;
    target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
    return target_ref;
  }

  target_ref.target_snapshot = global_reference_available ? ct_target_ivox_ : snapshot;
  target_ref.contributing_frames = inserted ? target_ref.snapshot_frame_count : 0;
  target_ref.target_tree = target_ref.target_snapshot;
  target_ref.mode = BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
  target_ref.point_count = target_ref.target_snapshot ? target_ref.target_snapshot->voxel_points().size() : 0;
  target_ref.build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
  return target_ref;
}

std::vector<OdometryEstimationBSpline::ActiveSplineIMUSample> OdometryEstimationBSpline::create_segment_imu_samples(
  const PreprocessedFrame::Ptr& raw_frame) const {
  std::vector<ActiveSplineIMUSample> samples;
  if (!imu_integration) {
    return samples;
  }

  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);
  const double finite_difference_dt = std::min(trajectory_params_.finite_difference_dt, 0.25 * scan_duration);
  const double sample_start = raw_frame->stamp + finite_difference_dt;
  const double sample_end = raw_frame->scan_end_time - finite_difference_dt;
  if (sample_end <= sample_start) {
    return samples;
  }

  std::vector<double> delta_times;
  std::vector<Eigen::Matrix<double, 7, 1>> imu_data;
  imu_integration->find_imu_data(sample_start, sample_end, delta_times, imu_data);
  if (imu_data.empty()) {
    return samples;
  }

  const std::size_t stride = static_cast<std::size_t>(std::max(1, imu_ct_sample_stride_));
  auto append_sample = [&](std::size_t idx) {
    const auto& imu = imu_data[idx];
    ActiveSplineIMUSample sample;
    sample.stamp = imu[0];
    sample.u = std::clamp((sample.stamp - raw_frame->stamp) / scan_duration, 0.0, 1.0);
    sample.linear_acc = imu.block<3, 1>(1, 0);
    sample.angular_vel = imu.block<3, 1>(4, 0);
    samples.push_back(sample);
  };

  for (std::size_t i = 0; i < imu_data.size(); i += stride) {
    append_sample(i);
  }

  if ((imu_data.size() - 1) % stride != 0) {
    append_sample(imu_data.size() - 1);
  }

  return samples;
}

std::shared_ptr<const iap::SplineStateLayout> OdometryEstimationBSpline::create_segment_imu_layout(
  const ActiveSplineSegmentConstraint& segment) const {
  auto layout = std::make_shared<iap::SplineStateLayout>();

  std::vector<iap::BSplineControlPointState> controls;
  controls.reserve(iap::kBSplineControlPointCount);
  const auto& buffered_states = fixed_lag_registry_.control_buffer().states();

  for (const auto control_index : segment.control_indices) {
    const auto it = std::find_if(buffered_states.begin(), buffered_states.end(), [&](const auto& state) {
      return state.index == control_index;
    });
    if (it == buffered_states.end()) {
      return nullptr;
    }
    controls.push_back(*it);
  }

  const double span_start = segment.stamp;
  const double span_end = std::max(segment.scan_end, segment.stamp + 1e-3);
  layout->set_controls(std::move(controls));
  layout->set_knots({
    span_start,
    span_start,
    span_start,
    span_start,
    span_end,
    span_end,
    span_end,
    span_end,
  });

  // Commit 4 compatibility note:
  // The active spline control poses still live in the LiDAR frame, so the IMU
  // sensor slot stores the inverse extrinsic here to let the current evaluator
  // recover world->imu queries without rewriting sensor semantics yet.
  iap::SplineSensorModel imu_model;
  imu_model.id = iap::SplineSensorId::Imu;
  imu_model.T_sensor_imu = Eigen::Isometry3d(T_imu_lidar.matrix());
  layout->set_sensor_model(iap::SplineSensorId::Imu, imu_model);

  return layout;
}

std::vector<iap::GnssEpoch> OdometryEstimationBSpline::consume_segment_gnss_epochs(
  double segment_start,
  double segment_end) {
  sync_gnss_epochs_from_shared_state();
  if (!gnss_handler_) {
    return {};
  }
  return gnss_handler_->consume_epochs_in_range(segment_start, segment_end, gnss_time_tolerance_);
}

void OdometryEstimationBSpline::sync_gnss_epochs_from_shared_state() {
  if (!gnss_handler_ || !gnss_epoch_builder_) {
    return;
  }

  auto& shared = iap::IapSharedState::instance();
  if (const auto anchor = shared.get_gnss_anchor()) {
    gnss_epoch_builder_->set_anchor(*anchor);
    fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
  }

  const auto iono_params = shared.get_gnss_iono_params();
  if (!iono_params.empty()) {
    gnss_epoch_builder_->set_iono_params(iono_params);
  }

  const auto ephemeris_updates = shared.consume_pending_gnss_ephemeris_updates();
  for (const auto& update : ephemeris_updates) {
    gnss_epoch_builder_->update_ephemeris(update);
  }

  const auto raw_batches = shared.consume_pending_gnss_raw_batches();
  for (auto& batch : raw_batches) {
    pending_raw_gnss_batches_.push_back(std::move(batch));
  }

  while (!pending_raw_gnss_batches_.empty()) {
    const auto build_result = gnss_epoch_builder_->build_epoch(pending_raw_gnss_batches_.front());

    if (build_result.status == iap::GnssEpochBuilder::BuildStatus::MissingAnchor ||
        build_result.status == iap::GnssEpochBuilder::BuildStatus::MissingEphemeris) {
      break;
    }

    if (build_result.status == iap::GnssEpochBuilder::BuildStatus::Success && build_result.epoch.has_value()) {
      gnss_handler_->insert_epoch(*build_result.epoch);
    }

    pending_raw_gnss_batches_.pop_front();
  }
}

void OdometryEstimationBSpline::prune_active_ct_state(double min_active_stamp) {
  fixed_lag_registry_.prune_before(min_active_stamp);
  fixed_lag_registry_.retain_active_auxiliary_values(true);
}

void OdometryEstimationBSpline::update_marginal_prior_from_active_window() {
  marginal_prior_ = ActiveSplineMarginalPrior();

  const auto& control_buffer = fixed_lag_registry_.control_buffer();
  if (control_buffer.size() < 2) {
    return;
  }

  const auto& states = control_buffer.states();
  marginal_prior_.valid = true;
  marginal_prior_.control_indices = {states[0].index, states[1].index};
  marginal_prior_.first_pose = states[0].pose;
  marginal_prior_.relative_delta = states[0].pose.between(states[1].pose);
  marginal_prior_.auxiliary_index = states[1].index;

  const gtsam::Key velocity_key = iap::bspline_velocity_key(marginal_prior_.auxiliary_index);
  if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
    marginal_prior_.has_velocity = true;
    marginal_prior_.velocity = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key);
  }

  const gtsam::Key clock_key = iap::bspline_clock_key(marginal_prior_.auxiliary_index);
  if (fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
    marginal_prior_.has_clock = true;
    marginal_prior_.clock = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector2>(clock_key);
  }
}

void OdometryEstimationBSpline::update_marginal_prior_information(
  const gtsam::NonlinearFactorGraph& graph,
  const gtsam::Values& values,
  const std::vector<gtsam::Key>& survivor_keys,
  const iap::BSplineCarriedPrior* previous_prior) {
  try {
    marginal_prior_.carried_prior =
      iap::build_bspline_carried_prior(graph, values, survivor_keys, previous_prior);
  } catch (const std::exception& e) {
    marginal_prior_.carried_prior = iap::BSplineCarriedPrior();
    logger->warn("failed to build bspline marginal survivor prior: {}", e.what());
  }
}

void OdometryEstimationBSpline::append_active_segment_constraint(
  const PreprocessedFrame::Ptr& raw_frame,
  const gtsam_points::PointCloud::ConstPtr& source) {
  ActiveSplineSegmentConstraint segment;
  segment.stamp = raw_frame->stamp;
  segment.scan_end = raw_frame->scan_end_time;
  segment.source = source;
  const auto target_ref = create_active_target_reference();
  segment.target_snapshot = target_ref.target_snapshot;
  segment.target_tree = target_ref.target_tree;
  segment.target_mode = target_ref.mode;
  segment.target_frame_count = target_ref.contributing_frames;
  segment.target_point_count = target_ref.point_count;
  segment.snapshot_frame_count = target_ref.snapshot_frame_count;
  segment.snapshot_point_count = target_ref.snapshot_point_count;
  segment.snapshot_span_sec = target_ref.snapshot_span_sec;
  segment.snapshot_policy_accepted = target_ref.snapshot_policy_accepted;
  segment.target_build_ms = target_ref.build_ms;
  segment.imu_samples = create_segment_imu_samples(raw_frame);

  const auto states = control_window_->states();
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    segment.control_indices[i] = states[i].index;
  }
  segment.auxiliary_index = states[1].index;

  fixed_lag_registry_.append_segment(std::move(segment));
}

void OdometryEstimationBSpline::insert_target_cloud(const EstimationFrame::Ptr& frame) {
  auto transformed = gtsam_points::PointCloudCPU::clone(*frame->frame);
  for (int i = 0; i < transformed->size(); ++i) {
    transformed->points[i] = frame->T_world_lidar * frame->frame->points[i];
    transformed->covs[i] = frame->T_world_lidar.matrix() * frame->frame->covs[i] * frame->T_world_lidar.matrix().transpose();
  }
  ct_target_ivox_->insert(*transformed);
  ++ct_target_revision_;
}

void OdometryEstimationBSpline::update_frame_history(
  const EstimationFrame::Ptr& frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  frames.push_back(frame);

  while (marginalized_cursor < frames.size() - 1) {
    const double span = frame->stamp - frames[marginalized_cursor]->stamp;
    if (span < params->smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }

  Callbacks::on_marginalized_frames(marginalized_frames);
}

bool OdometryEstimationBSpline::lidar_collect_window_results() const {
  return lidar_factor_profile_ || lidar_profile_numeric_reference_ || lidar_export_baseline_csv_ || lidar_warn_degeneracy_;
}

std::size_t OdometryEstimationBSpline::lidar_factor_config_signature(bool use_gpu_lidar) const {
  std::size_t seed = 0;
  hash_combine(&seed, use_gpu_lidar);
  hash_combine(&seed, static_cast<int>(lidar_gpu_backend_));
  hash_combine(&seed, static_cast<int>(lidar_target_mode_));
  hash_combine(&seed, static_cast<int>(lidar_jacobian_mode_));
  hash_combine(&seed, static_cast<int>(lidar_robust_kernel_));
  hash_combine(&seed, max_correspondence_distance_);
  hash_combine(&seed, lidar_jacobian_numeric_eps_);
  hash_combine(&seed, lidar_outlier_mahalanobis_thresh_);
  hash_combine(&seed, lidar_robust_kernel_width_);
  hash_combine(&seed, lidar_robust_weight_floor_);
  hash_combine(&seed, lidar_correspondence_candidate_count_);
  hash_combine(&seed, lidar_correspondence_accept_ratio_);
  hash_combine(&seed, lidar_correspondence_min_score_gap_);
  hash_combine(&seed, lidar_factor_profile_);
  hash_combine(&seed, lidar_warn_degeneracy_);
  hash_combine(&seed, lidar_export_baseline_csv_);
  return seed;
}

std::size_t OdometryEstimationBSpline::lidar_target_revision(const ActiveSplineSegmentConstraint& segment) const {
  return segment.target_mode == BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE ? ct_target_revision_ : 0U;
}

OdometryEstimationBSpline::ActiveSplineLidarFactorCacheKey
OdometryEstimationBSpline::make_lidar_factor_cache_key(
  const ActiveSplineSegmentConstraint& segment,
  bool use_gpu_lidar) const {
  ActiveSplineLidarFactorCacheKey key;
  key.valid = true;
  key.gpu = use_gpu_lidar;
  key.gpu_backend = lidar_gpu_backend_;
  key.target_mode = segment.target_mode;
  key.control_indices = segment.control_indices;
  key.source_identity = segment.source.get();
  key.target_identity = segment.target_snapshot.get();
  key.target_revision = lidar_target_revision(segment);
  key.config_signature = lidar_factor_config_signature(use_gpu_lidar);
  return key;
}

bool OdometryEstimationBSpline::same_lidar_factor_cache_base(
  const ActiveSplineLidarFactorCacheKey& lhs,
  const ActiveSplineLidarFactorCacheKey& rhs) const {
  return lhs.valid && rhs.valid &&
         lhs.gpu == rhs.gpu &&
         lhs.gpu_backend == rhs.gpu_backend &&
         lhs.target_mode == rhs.target_mode &&
         lhs.control_indices == rhs.control_indices &&
         lhs.source_identity == rhs.source_identity &&
         lhs.config_signature == rhs.config_signature;
}

std::shared_ptr<iap::IntegratedBSplineGICPFactor> OdometryEstimationBSpline::get_or_create_cpu_lidar_factor(
  ActiveSplineSegmentConstraint& segment,
  bool* cache_hit) {
  const auto desired_key = make_lidar_factor_cache_key(segment, false);
  const bool key_match =
    segment.cached_cpu_factor &&
    same_lidar_factor_cache_base(segment.lidar_factor_cache, desired_key) &&
    segment.lidar_factor_cache.target_identity == desired_key.target_identity;

  if (!key_match) {
    auto factor = std::make_shared<iap::IntegratedBSplineGICPFactor>(
      std::array<gtsam::Key, iap::kBSplineControlPointCount>{
        iap::bspline_control_point_key(segment.control_indices[0]),
        iap::bspline_control_point_key(segment.control_indices[1]),
        iap::bspline_control_point_key(segment.control_indices[2]),
        iap::bspline_control_point_key(segment.control_indices[3])},
      segment.target_snapshot,
      segment.source,
      segment.target_tree);
    factor->set_num_threads(params->num_threads);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_jacobian_mode(lidar_jacobian_mode_);
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_robust_weight_floor(lidar_robust_weight_floor_);
    factor->set_enable_profiling(lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_);
    segment.cached_cpu_factor = factor;
    segment.lidar_factor_cache = desired_key;
    if (cache_hit) {
      *cache_hit = false;
    }
  } else if (cache_hit) {
    *cache_hit = true;
  }

  return segment.cached_cpu_factor;
}

#ifdef GTSAM_POINTS_USE_CUDA
std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> OdometryEstimationBSpline::get_or_create_gpu_bucket_lidar_factor(
  ActiveSplineSegmentConstraint& segment,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
  bool* cache_hit,
  bool* target_refreshed) {
  const auto desired_key = make_lidar_factor_cache_key(segment, true);
  const bool enable_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_profile_numeric_reference_;
  const bool cache_base_match =
    segment.cached_gpu_bucket_factor && same_lidar_factor_cache_base(segment.lidar_factor_cache, desired_key);
  const bool can_refresh_target =
    cache_base_match &&
    segment.lidar_factor_cache.target_mode == desired_key.target_mode &&
    segment.lidar_factor_cache.source_identity == desired_key.source_identity;

  if (!cache_base_match) {
    auto factor = std::make_shared<iap::IntegratedBSplineGICPFactorGPU>(
      std::array<gtsam::Key, iap::kBSplineControlPointCount>{
        iap::bspline_control_point_key(segment.control_indices[0]),
        iap::bspline_control_point_key(segment.control_indices[1]),
        iap::bspline_control_point_key(segment.control_indices[2]),
        iap::bspline_control_point_key(segment.control_indices[3])},
      segment.target_snapshot,
      segment.source,
      stream,
      temp_buffer);
    factor->set_jacobian_mode(to_gpu_lidar_jacobian_mode(lidar_jacobian_mode_));
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_robust_weight_floor(lidar_robust_weight_floor_);
    factor->set_enable_profiling(lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_);
    segment.cached_gpu_bucket_factor = factor;
    segment.lidar_factor_cache = desired_key;
    if (cache_hit) {
      *cache_hit = false;
    }
    if (target_refreshed) {
      *target_refreshed = false;
    }
  } else {
    if (segment.lidar_factor_cache.target_identity != desired_key.target_identity ||
        segment.lidar_factor_cache.target_revision != desired_key.target_revision) {
      segment.cached_gpu_bucket_factor->refresh_target(segment.target_snapshot);
      segment.lidar_factor_cache = desired_key;
      if (target_refreshed) {
        *target_refreshed = true;
      }
    } else if (target_refreshed) {
      *target_refreshed = false;
    }
    if (cache_hit) {
      *cache_hit = true;
    }
  }

  return segment.cached_gpu_bucket_factor;
}

std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> OdometryEstimationBSpline::get_or_create_gpu_kernel_lidar_factor(
  ActiveSplineSegmentConstraint& segment,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer,
  bool* cache_hit,
  bool* target_refreshed) {
  const auto desired_key = make_lidar_factor_cache_key(segment, true);
  const bool enable_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_ || lidar_profile_numeric_reference_;
  const bool cache_base_match =
    segment.cached_gpu_kernel_factor && same_lidar_factor_cache_base(segment.lidar_factor_cache, desired_key);
  const bool can_refresh_target =
    cache_base_match &&
    segment.lidar_factor_cache.target_mode == desired_key.target_mode &&
    segment.lidar_factor_cache.source_identity == desired_key.source_identity;

  if (!cache_base_match) {
    auto factor = std::make_shared<iap::IntegratedBSplineGICPFactorGPUKernel>(
      std::array<gtsam::Key, iap::kBSplineControlPointCount>{
        iap::bspline_control_point_key(segment.control_indices[0]),
        iap::bspline_control_point_key(segment.control_indices[1]),
        iap::bspline_control_point_key(segment.control_indices[2]),
        iap::bspline_control_point_key(segment.control_indices[3])},
      segment.target_snapshot,
      segment.source,
      stream,
      temp_buffer);
    factor->set_jacobian_mode(lidar_jacobian_mode_);
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_correspondence_min_score_gap(lidar_correspondence_min_score_gap_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_robust_weight_floor(lidar_robust_weight_floor_);
    factor->set_enable_profiling(enable_profile_surface);
    segment.cached_gpu_kernel_factor = factor;
    segment.lidar_factor_cache = desired_key;
    if (cache_hit) {
      *cache_hit = false;
    }
    if (target_refreshed) {
      *target_refreshed = false;
    }
  } else {
    segment.cached_gpu_kernel_factor->set_enable_profiling(enable_profile_surface);
    if (can_refresh_target &&
        (segment.lidar_factor_cache.target_identity != desired_key.target_identity ||
         segment.lidar_factor_cache.target_revision != desired_key.target_revision)) {
      segment.cached_gpu_kernel_factor->refresh_target(segment.target_snapshot);
      segment.lidar_factor_cache = desired_key;
      if (target_refreshed) {
        *target_refreshed = true;
      }
    } else if (target_refreshed) {
      *target_refreshed = false;
    }
    if (cache_hit) {
      *cache_hit = true;
    }
  }

  return segment.cached_gpu_kernel_factor;
}
#endif

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  using Clock = std::chrono::steady_clock;
  const auto t_window_start = Clock::now();
  const auto elapsed_ms = [](const Clock::time_point& start, const Clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
  };
  struct PipelineTiming {
    double gnss_mailbox_sync_ms = 0.0;
    double source_cloud_ms = 0.0;
    double segment_prepare_ms = 0.0;
    double target_build_ms = 0.0;
    double gnss_epoch_fetch_ms = 0.0;
    double marginalization_partition_ms = 0.0;
    double graph_build_ms = 0.0;
    double graph_lidar_factor_ms = 0.0;
    double graph_lidar_factor_new_build_ms = 0.0;
    double graph_lidar_factor_target_refresh_ms = 0.0;
    double graph_lidar_factor_reused_attach_ms = 0.0;
    double graph_velocity_factor_ms = 0.0;
    double imu_factor_assembly_ms = 0.0;
    double gnss_factor_assembly_ms = 0.0;
    double carried_prior_attach_ms = 0.0;
    double graph_prediction_prior_ms = 0.0;
    double graph_smoothness_ms = 0.0;
    double graph_shared_prior_ms = 0.0;
    double graph_clock_factor_ms = 0.0;
    double lm_optimize_ms = 0.0;
    double prune_active_ms = 0.0;
    double carried_prior_update_ms = 0.0;
    double marginalization_ms = 0.0;
    double postprocess_ms = 0.0;
    double post_lidar_result_ms = 0.0;
    double post_lidar_factor_error_ms = 0.0;
    double post_lidar_numeric_audit_ms = 0.0;
    double post_lidar_degeneracy_ms = 0.0;
    double post_lidar_result_pack_ms = 0.0;
    double post_lidar_window_aggregate_ms = 0.0;
    double post_lidar_csv_ms = 0.0;
    double post_lidar_log_ms = 0.0;
    double post_frame_state_ms = 0.0;
    double post_deskew_ms = 0.0;
    double post_covariance_ms = 0.0;
    double post_frame_store_ms = 0.0;
    double post_target_insert_ms = 0.0;
    double post_history_update_ms = 0.0;
    double post_publish_traj_ms = 0.0;
    double post_publish_telemetry_ms = 0.0;
    double post_callback_ms = 0.0;
    double window_wall_ms = 0.0;
    std::size_t graph_lidar_factor_cache_hit_count = 0;
    std::size_t graph_lidar_factor_cache_miss_count = 0;
    std::size_t graph_lidar_factor_refresh_count = 0;
  } pipeline_timing;

  Callbacks::on_insert_frame(raw_frame);
  {
    const auto t_sync_start = Clock::now();
    sync_gnss_epochs_from_shared_state();
    pipeline_timing.gnss_mailbox_sync_ms = elapsed_ms(t_sync_start, Clock::now());
  }

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);
  const bool use_gpu_lidar = frontend_mode_ == "CT_LIDAR_GPU";
  const bool collect_window_lidar_results = lidar_collect_window_results();

  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;

  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;
    if (init_estimation) {
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
    }

    if (!init_state && init_estimation) {
      logger->debug("waiting for initial IMU state estimation to be finished (bspline ct frontend)");
      return nullptr;
    }

    const gtsam::Pose3 initial_pose = init_state
      ? gtsam::Pose3(init_state->T_world_lidar.matrix())
      : gtsam::Pose3();

    initialize_control_window(raw_frame, initial_pose);

    new_frame->T_world_lidar = Eigen::Isometry3d(initial_pose.matrix());
    new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    new_frame->imu_bias = init_state ? init_state->imu_bias : params->imu_bias;
    new_frame->v_world_imu = init_state ? init_state->v_world_imu : Eigen::Vector3d::Zero();
    fixed_lag_registry_.set_shared_imu_state(
      new_frame->imu_bias.tail<3>(),
      new_frame->imu_bias.head<3>(),
      fixed_lag_registry_.shared_state().gravity);
    fixed_lag_registry_.clear_auxiliary_values();
    fixed_lag_registry_.auxiliary_values().insert(iap::bspline_velocity_key(control_window_->states()[1].index), new_frame->v_world_imu);

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    update_marginal_prior_from_active_window();
    publish_continuous_trajectory(current);
    publish_fixed_lag_telemetry(current);

    std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
    if (!active_frames.empty()) {
      Callbacks::on_update_new_frame(active_frames.back());
      Callbacks::on_update_frames(active_frames);
    }

    if (init_estimation) {
      init_estimation.reset();
    }
    return new_frame;
  }

  const gtsam::Pose3 predicted_end_pose = predict_scan_end_pose(scan_duration);
  control_window_->advance(raw_frame->stamp, raw_frame->scan_end_time, predicted_end_pose);
  fixed_lag_registry_.append_window(*control_window_);

  if (const auto anchor = iap::IapSharedState::instance().get_gnss_anchor()) {
    fixed_lag_registry_.set_shared_gnss_anchor(anchor->origin_ecef, gtsam::Rot3(anchor->R_ecef_world));
  }

  const double min_active_stamp = std::max(0.0, raw_frame->stamp - params->smoother_lag);

  {
    const auto t_source_start = Clock::now();
    new_frame->frame = create_lidar_source_cloud(raw_frame);
    pipeline_timing.source_cloud_ms = elapsed_ms(t_source_start, Clock::now());
  }
  const auto factor_source = gtsam_points::PointCloudCPU::clone(*new_frame->frame);
  {
    const auto t_segment_prepare_start = Clock::now();
    append_active_segment_constraint(raw_frame, factor_source);
    pipeline_timing.segment_prepare_ms = elapsed_ms(t_segment_prepare_start, Clock::now());
    if (!fixed_lag_registry_.segments().empty()) {
      pipeline_timing.target_build_ms = fixed_lag_registry_.segments().back().target_build_ms;
    }
  }
  if (!fixed_lag_registry_.segments().empty()) {
    const auto t_gnss_epoch_start = Clock::now();
    fixed_lag_registry_.segments().back().gnss_epochs = consume_segment_gnss_epochs(
      raw_frame->stamp,
      raw_frame->scan_end_time);
    pipeline_timing.gnss_epoch_fetch_ms = elapsed_ms(t_gnss_epoch_start, Clock::now());
  }

  gtsam::Values values = fixed_lag_registry_.control_buffer().values();
  const auto& active_states = fixed_lag_registry_.control_buffer().states();
  const iap::BSplineCarriedPrior previous_carried_prior = marginal_prior_.carried_prior;
  const gtsam::Key gyro_bias_key = bspline_gyro_bias_key();
  const gtsam::Key accel_bias_key = bspline_accel_bias_key();
  const gtsam::Key gravity_key = bspline_gravity_key();
  fixed_lag_registry_.seed_shared_values(values, fixed_lag_registry_.shared_state().gnss_anchor_initialized);

  gtsam::NonlinearFactorGraph graph;
  gtsam::NonlinearFactorGraph marginalization_graph;
  std::shared_ptr<iap::IntegratedBSplineGICPFactor> current_cpu_factor;
  std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactor>> active_lidar_cpu_factors;
#ifdef GTSAM_POINTS_USE_CUDA
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU> current_gpu_factor;
  std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU>> active_lidar_gpu_factors;
  std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel> current_gpu_kernel_factor;
  std::vector<std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel>> active_lidar_gpu_kernel_factors;
#endif
  std::size_t active_imu_factor_count = 0;
  std::size_t active_velocity_factor_count = 0;
  std::size_t active_gnss_pr_factor_count = 0;
  std::size_t active_gnss_dop_factor_count = 0;
  struct ActiveClockState {
    gtsam::Key key = 0;
    double stamp = 0.0;
    gtsam::Vector2 value = gtsam::Vector2::Zero();
  };
  std::vector<ActiveClockState> active_clock_states;
  const gtsam::Key ecef_origin_key = iap::bspline_ecef_origin_key();
  const gtsam::Key ecef_rot_key = iap::bspline_ecef_rot_key();
  const auto gnss_pr_sigma = [&](const iap::SatObs& sat) {
    const double canopy_sigma = iap::sigma_eff_canopy(gnss_canopy_params_, sat.kappa, sat.elevation);
    return std::max({1e-3, sat.pr_sigma, canopy_sigma, gnss_pr_noise_base_});
  };
  const auto gnss_dop_sigma = [&](const iap::SatObs& sat) {
    const double sin_el = std::sin(std::max(sat.elevation, gnss_min_elevation_));
    const double modeled = gnss_dop_noise_base_ / std::pow(std::max(0.052, sin_el), gnss_elev_noise_exp_);
    return std::max({1e-3, sat.dop_sigma, modeled});
  };
  auto segment_poses_from_values = [&](const ActiveSplineSegmentConstraint& segment) {
    std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses;
    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      poses[k] = values.at<gtsam::Pose3>(iap::bspline_control_point_key(segment.control_indices[k]));
    }
    return poses;
  };
  std::vector<ActiveClockState> seeded_clock_states;
  if (fixed_lag_registry_.shared_state().gnss_anchor_initialized) {
    for (const auto& segment : fixed_lag_registry_.segments()) {
      if (segment.gnss_epochs.empty()) {
        continue;
      }

      const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
      if (!values.exists(clock_key)) {
        gtsam::Vector2 init_clock = gtsam::Vector2::Zero();
        if (fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
          init_clock = fixed_lag_registry_.auxiliary_values().at<gtsam::Vector2>(clock_key);
        } else if (!seeded_clock_states.empty()) {
          init_clock = seeded_clock_states.back().value;
          const double dt = std::max(0.0, segment.stamp - seeded_clock_states.back().stamp);
          init_clock(0) += init_clock(1) * dt;
        }
        values.insert(clock_key, init_clock);
      }

      seeded_clock_states.push_back(ActiveClockState{clock_key, segment.stamp, values.at<gtsam::Vector2>(clock_key)});
    }

  }
  const iap::BSplineMarginalizationPartition marginalization_partition = [&] {
    const auto t_partition_start = Clock::now();
    auto partition = iap::build_bspline_marginalization_partition(
      fixed_lag_registry_.control_buffer().states(),
      fixed_lag_registry_.marginalization_segment_states(),
      values,
      min_active_stamp,
      !seeded_clock_states.empty());
    pipeline_timing.marginalization_partition_ms = elapsed_ms(t_partition_start, Clock::now());
    return partition;
  }();
  const auto t_graph_build_start = Clock::now();
  const bool enable_lidar_profile_surface =
    lidar_factor_profile_ || lidar_warn_degeneracy_ || lidar_export_baseline_csv_;
  for (std::size_t i = 0; i < fixed_lag_registry_.segments().size(); ++i) {
    auto& segment = fixed_lag_registry_.segments()[i];
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);

    if (!values.exists(velocity_key)) {
      if (fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
        values.insert(velocity_key, fixed_lag_registry_.auxiliary_values().at<gtsam::Vector3>(velocity_key));
      } else {
        const auto segment_poses = segment_poses_from_values(segment);
        const gtsam::Vector3 velocity_guess =
          iap::IntegratedBSplineVelocityFactor::predict_velocity(
            segment_poses,
            0.0,
            std::max(1e-3, segment.scan_end - segment.stamp),
            trajectory_params_.finite_difference_dt);
        values.insert(velocity_key, velocity_guess);
      }
    }

    std::array<gtsam::Key, iap::kBSplineControlPointCount> segment_keys{};
    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      segment_keys[k] = iap::bspline_control_point_key(segment.control_indices[k]);
    }

    if (!use_gpu_lidar) {
      const auto t_graph_lidar_start = Clock::now();
      bool cache_hit = false;
      const auto t_lidar_prepare_start = Clock::now();
      auto factor = get_or_create_cpu_lidar_factor(segment, &cache_hit);
      factor->set_enable_profiling(enable_lidar_profile_surface);
      const auto t_lidar_prepare_end = Clock::now();
      if (cache_hit) {
        pipeline_timing.graph_lidar_factor_cache_hit_count++;
      } else {
        pipeline_timing.graph_lidar_factor_cache_miss_count++;
        pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);
      }

      const auto t_lidar_attach_start = Clock::now();
      active_lidar_cpu_factors.push_back(factor);
      graph.add(factor);
      if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys))) {
        marginalization_graph.add(factor);
      }
      const auto t_lidar_attach_end = Clock::now();
      if (cache_hit) {
        pipeline_timing.graph_lidar_factor_reused_attach_ms += elapsed_ms(t_lidar_attach_start, t_lidar_attach_end);
      }

      if (i + 1 == fixed_lag_registry_.segments().size()) {
        current_cpu_factor = factor;
      }
      pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
    } else {
#ifdef GTSAM_POINTS_USE_CUDA
      switch (lidar_gpu_backend_) {
        case BSplineGpuLidarBackend::BUCKET: {
          const auto t_graph_lidar_start = Clock::now();
          if (!ct_lidar_gpu_stream_buffers_) {
            ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
          }
          auto stream_buffer = ct_lidar_gpu_stream_buffers_->get_stream_buffer();
          bool cache_hit = false;
          bool target_refreshed = false;
          const auto t_lidar_prepare_start = Clock::now();
          auto factor = get_or_create_gpu_bucket_lidar_factor(
            segment,
            stream_buffer.first,
            stream_buffer.second,
            &cache_hit,
            &target_refreshed);
          factor->set_enable_profiling(enable_lidar_profile_surface);
          const auto t_lidar_prepare_end = Clock::now();
          if (cache_hit) {
            pipeline_timing.graph_lidar_factor_cache_hit_count++;
            if (target_refreshed) {
              pipeline_timing.graph_lidar_factor_refresh_count++;
              pipeline_timing.graph_lidar_factor_target_refresh_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);
            }
          } else {
            pipeline_timing.graph_lidar_factor_cache_miss_count++;
            pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);
          }

          const auto t_lidar_attach_start = Clock::now();
          active_lidar_gpu_factors.push_back(factor);
          graph.add(factor);
          if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys))) {
            marginalization_graph.add(factor);
          }
          const auto t_lidar_attach_end = Clock::now();
          if (cache_hit && !target_refreshed) {
            pipeline_timing.graph_lidar_factor_reused_attach_ms += elapsed_ms(t_lidar_attach_start, t_lidar_attach_end);
          }

          if (i + 1 == fixed_lag_registry_.segments().size()) {
            current_gpu_factor = factor;
          }
          pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
          break;
        }
        case BSplineGpuLidarBackend::KERNEL:
        {
          const auto t_graph_lidar_start = Clock::now();
          if (!ct_lidar_gpu_stream_buffers_) {
            ct_lidar_gpu_stream_buffers_ = std::make_unique<gtsam_points::StreamTempBufferRoundRobin>();
          }
          auto stream_buffer = ct_lidar_gpu_stream_buffers_->get_stream_buffer();
          bool cache_hit = false;
          bool target_refreshed = false;
          const auto t_lidar_prepare_start = Clock::now();
          auto factor = get_or_create_gpu_kernel_lidar_factor(
            segment,
            stream_buffer.first,
            stream_buffer.second,
            &cache_hit,
            &target_refreshed);
          factor->set_enable_profiling(enable_lidar_profile_surface);
          const auto t_lidar_prepare_end = Clock::now();
          if (cache_hit) {
            pipeline_timing.graph_lidar_factor_cache_hit_count++;
            if (target_refreshed) {
              pipeline_timing.graph_lidar_factor_refresh_count++;
              pipeline_timing.graph_lidar_factor_target_refresh_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);
            }
          } else {
            pipeline_timing.graph_lidar_factor_cache_miss_count++;
            pipeline_timing.graph_lidar_factor_new_build_ms += elapsed_ms(t_lidar_prepare_start, t_lidar_prepare_end);
          }

          const auto t_lidar_attach_start = Clock::now();
          active_lidar_gpu_kernel_factors.push_back(factor);
          graph.add(factor);
          if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys))) {
            marginalization_graph.add(factor);
          }
          const auto t_lidar_attach_end = Clock::now();
          if (cache_hit && !target_refreshed) {
            pipeline_timing.graph_lidar_factor_reused_attach_ms += elapsed_ms(t_lidar_attach_start, t_lidar_attach_end);
          }
          if (i + 1 == fixed_lag_registry_.segments().size()) {
            current_gpu_kernel_factor = factor;
          }
          pipeline_timing.graph_lidar_factor_ms += elapsed_ms(t_graph_lidar_start, Clock::now());
          break;
        }
      }
#else
      logger->error("CT_LIDAR_GPU requested but CUDA support is unavailable");
      return nullptr;
#endif
    }

    const auto t_graph_velocity_start = Clock::now();
    auto velocity_factor = std::make_shared<iap::IntegratedBSplineVelocityFactor>(
      segment_keys,
      velocity_key,
      0.0,
      std::max(1e-3, segment.scan_end - segment.stamp),
      velocity_ct_inf_scale_,
      trajectory_params_.finite_difference_dt);
    graph.add(velocity_factor);
    if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys, {velocity_key}))) {
      marginalization_graph.add(velocity_factor);
    }
    active_velocity_factor_count++;
    pipeline_timing.graph_velocity_factor_ms += elapsed_ms(t_graph_velocity_start, Clock::now());

    const auto t_imu_factor_start = Clock::now();
    const auto segment_imu_layout = create_segment_imu_layout(segment);
    for (const auto& imu_sample : segment.imu_samples) {
      if (!segment_imu_layout) {
        continue;
      }

      const auto support = segment_imu_layout->support_at(imu_sample.stamp, iap::SplineSensorId::Imu);
      if (!support) {
        continue;
      }

      iap::SplineStampContext ctx;
      ctx.support = *support;
      ctx.sensor_id = iap::SplineSensorId::Imu;

      auto imu_factor = std::make_shared<iap::IntegratedSplineIMUFactor>(
        ctx,
        gyro_bias_key,
        accel_bias_key,
        gravity_key,
        imu_sample.angular_vel,
        imu_sample.linear_acc,
        imu_ct_trans_inf_scale_,
        imu_ct_rot_inf_scale_,
        segment_imu_layout);
      graph.add(imu_factor);
      if (marginalization_partition.should_marginalize_factor(
            make_key_vector(segment_keys, {gyro_bias_key, accel_bias_key, gravity_key}))) {
        marginalization_graph.add(imu_factor);
      }
      active_imu_factor_count++;
    }
    pipeline_timing.imu_factor_assembly_ms += elapsed_ms(t_imu_factor_start, Clock::now());

    if (fixed_lag_registry_.shared_state().gnss_anchor_initialized && !segment.gnss_epochs.empty()) {
      const auto t_gnss_factor_start = Clock::now();
      const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
      active_clock_states.push_back(ActiveClockState{clock_key, segment.stamp, values.at<gtsam::Vector2>(clock_key)});

      const double segment_duration = std::max(1e-3, segment.scan_end - segment.stamp);
      for (const auto& epoch : segment.gnss_epochs) {
        const double u = std::clamp((epoch.stamp - segment.stamp) / segment_duration, 0.0, 1.0);
        for (const auto& sat : epoch.sats) {
          if (sat.excluded || sat.elevation < gnss_min_elevation_) {
            continue;
          }

          auto pr_factor = std::make_shared<iap::IntegratedBSplinePseudorangeFactor>(
            segment_keys,
            clock_key,
            ecef_origin_key,
            ecef_rot_key,
            u,
            sat.pr_meas,
            sat.sat_pos,
            sat.tgd,
            epoch.gps_sec,
            epoch.iono_params,
            gnss_pr_sigma(sat),
            gnss_lever_arm_,
            sat.sat_id,
            sat.constellation,
            sat.elevation);
          graph.add(pr_factor);
          if (marginalization_partition.should_marginalize_factor(
                make_key_vector(segment_keys, {clock_key, ecef_origin_key, ecef_rot_key}))) {
            marginalization_graph.add(pr_factor);
          }
          active_gnss_pr_factor_count++;

          auto dop_factor = std::make_shared<iap::IntegratedBSplineDopplerFactor>(
            segment_keys,
            velocity_key,
            clock_key,
            ecef_rot_key,
            u,
            sat.dop_meas,
            sat.sat_pos,
            sat.sat_vel,
            fixed_lag_registry_.shared_state().ecef_origin,
            gnss_dop_sigma(sat),
            sat.sat_id,
            sat.constellation,
            sat.elevation);
          graph.add(dop_factor);
          if (marginalization_partition.should_marginalize_factor(
                make_key_vector(segment_keys, {velocity_key, clock_key, ecef_rot_key}))) {
            marginalization_graph.add(dop_factor);
          }
          active_gnss_dop_factor_count++;
        }
      }
      pipeline_timing.gnss_factor_assembly_ms += elapsed_ms(t_gnss_factor_start, Clock::now());
    }

  }

  if (!current_cpu_factor && 
#ifdef GTSAM_POINTS_USE_CUDA
      !current_gpu_factor &&
      !current_gpu_kernel_factor
#else
      true
#endif
  ) {
    logger->error("bspline ct frontend failed to create current segment factor");
    return nullptr;
  }

  const auto anchor_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_anchor_inf_scale_);
  const auto pred_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_prediction_inf_scale_);
  const auto smooth_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_smoothness_inf_scale_);
  const auto marginal_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_marginal_inf_scale_);
  const auto velocity_prior_noise = gtsam::noiseModel::Isotropic::Precision(3, velocity_ct_inf_scale_);
  const auto imu_bias_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_bias_inf_scale_);
  const auto gravity_noise = gtsam::noiseModel::Isotropic::Precision(3, imu_ct_gravity_inf_scale_);
  const auto gnss_ecef_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gnss_sigma_ecef_origin_));
  const auto gnss_ecef_rot_noise = gtsam::noiseModel::Isotropic::Sigma(3, gnss_sigma_ecef_rot_);
  const auto clock_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector2() << params->clk_bias_noise, params->clk_drift_noise).finished());
  const auto& shared_state = fixed_lag_registry_.shared_state();

  const bool use_marginal_prior =
    marginal_prior_.valid &&
    active_states.size() >= 2 &&
    active_states[0].index == marginal_prior_.control_indices[0] &&
    active_states[1].index == marginal_prior_.control_indices[1];
  const bool use_information_marginal_prior =
    !marginal_prior_.carried_prior.empty() &&
    marginalization_partition.can_replay_keys(marginal_prior_.carried_prior.retained_keys, values);

  bool information_prior_attached = false;
  const auto t_carried_prior_attach_start = Clock::now();
  if (use_information_marginal_prior) {
    try {
      const auto replayed_prior = marginal_prior_.carried_prior.replay();
      for (const auto& factor : replayed_prior) {
        if (!factor) {
          continue;
        }
        graph.add(factor->clone());
      }
      information_prior_attached = true;
    } catch (const std::exception& e) {
      logger->warn("failed to attach bspline marginal information prior, fallback to handcrafted prior: {}", e.what());
      marginal_prior_.carried_prior = iap::BSplineCarriedPrior();
    }
  }

  if (!information_prior_attached && use_marginal_prior) {
    auto pose_prior = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      marginal_prior_.first_pose,
      marginal_noise);
    auto delta_prior = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[0].index),
      iap::bspline_control_point_key(active_states[1].index),
      marginal_prior_.relative_delta,
      marginal_noise);
    graph.add(pose_prior);
    graph.add(delta_prior);
    marginalization_graph.add(pose_prior);
    marginalization_graph.add(delta_prior);
    if (marginal_prior_.has_velocity && values.exists(iap::bspline_velocity_key(marginal_prior_.auxiliary_index))) {
      auto velocity_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        iap::bspline_velocity_key(marginal_prior_.auxiliary_index),
        marginal_prior_.velocity,
        velocity_prior_noise);
      graph.add(velocity_prior);
      marginalization_graph.add(velocity_prior);
    }
  } else if (!active_states.empty()) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states.front().index),
      active_states.front().pose,
      anchor_noise);
    if (active_states.size() >= 2) {
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        iap::bspline_control_point_key(active_states[1].index),
        active_states[1].pose,
      anchor_noise);
    }
  }
  pipeline_timing.carried_prior_attach_ms = elapsed_ms(t_carried_prior_attach_start, Clock::now());

  const auto t_graph_prediction_prior_start = Clock::now();
  if (active_states.size() >= 2) {
    const auto& pred_a = active_states[active_states.size() - 2];
    const auto& pred_b = active_states.back();
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_a.index),
      pred_a.pose,
      pred_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(pred_b.index),
      pred_b.pose,
      pred_noise);
  }
  pipeline_timing.graph_prediction_prior_ms = elapsed_ms(t_graph_prediction_prior_start, Clock::now());

  const auto t_graph_smoothness_start = Clock::now();
  for (std::size_t i = 0; i + 1 < active_states.size(); ++i) {
    const gtsam::Key key_i = iap::bspline_control_point_key(active_states[i].index);
    const gtsam::Key key_j = iap::bspline_control_point_key(active_states[i + 1].index);
    auto smooth_factor = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      key_i,
      key_j,
      active_states[i].pose.between(active_states[i + 1].pose),
      smooth_noise);
    graph.add(smooth_factor);
    if (marginalization_partition.should_marginalize_factor(gtsam::KeyVector{key_i, key_j})) {
      marginalization_graph.add(smooth_factor);
    }
  }
  pipeline_timing.graph_smoothness_ms = elapsed_ms(t_graph_smoothness_start, Clock::now());

  const auto t_graph_shared_prior_start = Clock::now();
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gyro_bias_key, shared_state.gyro_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(accel_bias_key, shared_state.accel_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gravity_key, shared_state.gravity, gravity_noise);
  if (shared_state.gnss_anchor_initialized && values.exists(ecef_origin_key) && values.exists(ecef_rot_key)) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(ecef_origin_key, shared_state.ecef_origin, gnss_ecef_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(ecef_rot_key, shared_state.ecef_rot, gnss_ecef_rot_noise);
  }
  pipeline_timing.graph_shared_prior_ms = elapsed_ms(t_graph_shared_prior_start, Clock::now());

  const auto t_graph_clock_factor_start = Clock::now();
  if (!active_clock_states.empty()) {
    const bool clock_constrained_by_information_prior =
      information_prior_attached &&
      std::find(
        marginal_prior_.carried_prior.retained_keys.begin(),
        marginal_prior_.carried_prior.retained_keys.end(),
        active_clock_states.front().key) != marginal_prior_.carried_prior.retained_keys.end();
    if (!clock_constrained_by_information_prior) {
      const bool use_clock_boundary_prior =
        use_marginal_prior &&
        marginal_prior_.has_clock &&
        active_clock_states.front().key == iap::bspline_clock_key(marginal_prior_.auxiliary_index);
      const gtsam::Vector2 boundary_clock =
        use_clock_boundary_prior ? marginal_prior_.clock : active_clock_states.front().value;
      auto clock_prior = std::make_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        active_clock_states.front().key,
        boundary_clock,
        clock_prior_noise);
      graph.add(clock_prior);
      if (use_clock_boundary_prior) {
        marginalization_graph.add(clock_prior);
      }
    }
    for (std::size_t i = 1; i < active_clock_states.size(); ++i) {
      const double dt = std::max(1e-3, active_clock_states[i].stamp - active_clock_states[i - 1].stamp);
      auto clock_between = std::make_shared<iap::ClockBetweenFactor>(
        active_clock_states[i - 1].key,
        active_clock_states[i].key,
        dt,
        iap::ClockBetweenFactor::make_noise(dt, gnss_clock_between_params_));
      graph.add(clock_between);
      if (marginalization_partition.should_marginalize_factor(
            gtsam::KeyVector{active_clock_states[i - 1].key, active_clock_states[i].key})) {
        marginalization_graph.add(clock_between);
      }
    }
  }
  pipeline_timing.graph_clock_factor_ms = elapsed_ms(t_graph_clock_factor_start, Clock::now());
  pipeline_timing.graph_build_ms = elapsed_ms(t_graph_build_start, Clock::now());

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-4);
  lm_params.setAbsoluteErrorTol(1e-2);
  lm_params.setMaxIterations(lm_max_iterations_);

  const auto t_optimize_start = Clock::now();
  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
  } catch (const std::exception& e) {
    logger->error("bspline ct frontend optimization failed: {}", e.what());
  }
  pipeline_timing.lm_optimize_ms = elapsed_ms(t_optimize_start, Clock::now());

  const auto keys = control_window_->keys();
  logger->trace("bspline ct active_window={} active_segment_factors={} active_velocity_factors={} active_imu_factors={} active_gnss_pr_factors={} active_gnss_dop_factors={} current_segment_keys={} {} {} {}",
    active_states.size(),
    fixed_lag_registry_.segments().size(),
    active_velocity_factor_count,
    active_imu_factor_count,
    active_gnss_pr_factor_count,
    active_gnss_dop_factor_count,
    static_cast<std::uint64_t>(keys[0]),
    static_cast<std::uint64_t>(keys[1]),
    static_cast<std::uint64_t>(keys[2]),
    static_cast<std::uint64_t>(keys[3]));

  fixed_lag_registry_.control_buffer().update_from_values(values);
  control_window_->update_from_values(values);
  fixed_lag_registry_.update_shared_state_from_values(values);
  fixed_lag_registry_.clear_auxiliary_values();
  for (const auto& segment : fixed_lag_registry_.segments()) {
    const gtsam::Key velocity_key = iap::bspline_velocity_key(segment.auxiliary_index);
    if (values.exists(velocity_key) && !fixed_lag_registry_.auxiliary_values().exists(velocity_key)) {
      fixed_lag_registry_.auxiliary_values().insert(velocity_key, values.at<gtsam::Vector3>(velocity_key));
    }
    const gtsam::Key clock_key = iap::bspline_clock_key(segment.auxiliary_index);
    if (values.exists(clock_key) && !fixed_lag_registry_.auxiliary_values().exists(clock_key)) {
      fixed_lag_registry_.auxiliary_values().insert(clock_key, values.at<gtsam::Vector2>(clock_key));
    }
  }
  {
    const auto t_prune_start = Clock::now();
    prune_active_ct_state(min_active_stamp);
    pipeline_timing.prune_active_ms = elapsed_ms(t_prune_start, Clock::now());
  }
  {
    const auto t_carried_prior_update_start = Clock::now();
    update_marginal_prior_from_active_window();
    update_marginal_prior_information(
      marginalization_graph,
      values,
      marginalization_partition.survivor_keys,
      previous_carried_prior.empty() ? nullptr : &previous_carried_prior);
    pipeline_timing.carried_prior_update_ms = elapsed_ms(t_carried_prior_update_start, Clock::now());
  }
  pipeline_timing.marginalization_ms = pipeline_timing.prune_active_ms + pipeline_timing.carried_prior_update_ms;

  const auto t_postprocess_start = Clock::now();
  std::vector<iap::BSplineLidarFactorResult> lidar_results;
  lidar_results.reserve(std::max<std::size_t>(
    1,
    use_gpu_lidar
#ifdef GTSAM_POINTS_USE_CUDA
      ? (collect_window_lidar_results
          ? (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET
               ? active_lidar_gpu_factors.size()
               : active_lidar_gpu_kernel_factors.size())
          : 1U)
#else
      ? 0
#endif
      : (collect_window_lidar_results ? active_lidar_cpu_factors.size() : 1U)));
  iap::BSplineLidarFactorResult current_lidar_result;
  iap::IntegratedBSplineGICPFactor::NumericReferenceCheckResult current_numeric_check;
  iap::IntegratedBSplineGICPFactor::DegeneracyDiagnostics current_degeneracy;
  iap::BSplineLidarNumericAudit current_gpu_numeric_check;
  iap::BSplineLidarDegeneracyReport current_gpu_degeneracy;
  bool current_numeric_check_valid = false;
  bool current_degeneracy_valid = false;
  bool current_gpu_numeric_check_valid = false;
  bool current_gpu_degeneracy_valid = false;
  int current_lidar_result_index = -1;
  const auto t_post_lidar_result_start = Clock::now();

  auto process_cpu_factor = [&](const std::shared_ptr<iap::IntegratedBSplineGICPFactor>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::IntegratedBSplineGICPFactor::DegeneracyDiagnostics* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_degeneracy;
    }

    const iap::IntegratedBSplineGICPFactor::NumericReferenceCheckResult* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_numeric_check_valid = current_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };

#ifdef GTSAM_POINTS_USE_CUDA
  auto process_gpu_factor = [&](const std::shared_ptr<iap::IntegratedBSplineGICPFactorGPU>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::BSplineLidarDegeneracyReport* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_gpu_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      current_gpu_degeneracy_valid = current_gpu_degeneracy.valid;
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_gpu_degeneracy;
    }

    const iap::BSplineLidarNumericAudit* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_gpu_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_gpu_numeric_check_valid = current_gpu_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_gpu_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_gpu_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };

  auto process_gpu_kernel_factor = [&](const std::shared_ptr<iap::IntegratedBSplineGICPFactorGPUKernel>& factor, bool current_factor) {
    const auto t_factor_error_start = Clock::now();
    const double factor_error = factor->error(values);
    pipeline_timing.post_lidar_factor_error_ms += elapsed_ms(t_factor_error_start, Clock::now());

    const iap::BSplineLidarDegeneracyReport* degeneracy_ptr = nullptr;
    if (lidar_warn_degeneracy_) {
      const auto t_degeneracy_start = Clock::now();
      current_gpu_degeneracy = factor->diagnose_degeneracy(lidar_degeneracy_thresholds_);
      current_gpu_degeneracy_valid = current_gpu_degeneracy.valid;
      pipeline_timing.post_lidar_degeneracy_ms += elapsed_ms(t_degeneracy_start, Clock::now());
      degeneracy_ptr = &current_gpu_degeneracy;
    }

    const iap::BSplineLidarNumericAudit* numeric_ptr = nullptr;
    if (lidar_profile_numeric_reference_ && current_factor) {
      const auto t_numeric_start = Clock::now();
      current_gpu_numeric_check = factor->check_against_numeric_full(values, lidar_numeric_reference_scale_);
      current_gpu_numeric_check_valid = current_gpu_numeric_check.valid;
      pipeline_timing.post_lidar_numeric_audit_ms += elapsed_ms(t_numeric_start, Clock::now());
      numeric_ptr = &current_gpu_numeric_check;
    }

    const auto t_result_pack_start = Clock::now();
    auto result = factor->make_result(factor_error, factor->num_inliers(), factor->inlier_fraction(), numeric_ptr, degeneracy_ptr);
    pipeline_timing.post_lidar_result_pack_ms += elapsed_ms(t_result_pack_start, Clock::now());
    if (current_factor) {
      current_lidar_result = result;
      current_gpu_degeneracy_valid = degeneracy_ptr && degeneracy_ptr->valid;
      current_lidar_result_index = static_cast<int>(lidar_results.size());
    }
    lidar_results.push_back(std::move(result));
  };
#endif

  if (!use_gpu_lidar) {
    if (collect_window_lidar_results) {
      for (const auto& factor : active_lidar_cpu_factors) {
        process_cpu_factor(factor, factor == current_cpu_factor);
      }
    } else if (current_cpu_factor) {
      process_cpu_factor(current_cpu_factor, true);
    }
  } else {
#ifdef GTSAM_POINTS_USE_CUDA
    if (collect_window_lidar_results) {
      if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
        for (const auto& factor : active_lidar_gpu_factors) {
          process_gpu_factor(factor, factor == current_gpu_factor);
        }
      } else {
        for (const auto& factor : active_lidar_gpu_kernel_factors) {
          process_gpu_kernel_factor(factor, factor == current_gpu_kernel_factor);
        }
      }
    } else if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET && current_gpu_factor) {
      process_gpu_factor(current_gpu_factor, true);
    } else if (lidar_gpu_backend_ == BSplineGpuLidarBackend::KERNEL && current_gpu_kernel_factor) {
      process_gpu_kernel_factor(current_gpu_kernel_factor, true);
    }
#endif
  }

  const auto t_post_lidar_aggregate_start = Clock::now();
  const auto lidar_window_summary = iap::aggregate_bspline_lidar_factor_results(lidar_results);
  pipeline_timing.post_lidar_window_aggregate_ms = elapsed_ms(t_post_lidar_aggregate_start, Clock::now());
  pipeline_timing.post_lidar_result_ms = elapsed_ms(t_post_lidar_result_start, Clock::now());
  {
    const auto t_post_lidar_csv_start = Clock::now();
    maybe_export_lidar_baseline_csv(raw_frame->stamp, lidar_results, current_lidar_result_index);
    pipeline_timing.post_lidar_csv_ms = elapsed_ms(t_post_lidar_csv_start, Clock::now());
  }

  const auto t_post_lidar_log_start = Clock::now();
  if (lidar_factor_profile_ && lidar_window_summary.valid) {
    if (!use_gpu_lidar) {
      logger->trace(
        "bspline ct lidar cpu-summary backend={} segments={} profiled={} warnings={} total_src={} total_tgt={} total_matched={} total_inliers={} weighted_match={:.3f} weighted_inlier={:.3f} mean_unique_ratio={:.3f} min_unique_ratio={:.3f} max_reuse_ratio={:.3f} max_ambiguity_rej={:.3f} max_numeric_rel={:.6f} max_axis_rel={:.6f} total_pose_ms={:.3f} total_corr_ms={:.3f} total_accum_ms={:.3f} total_factor_ms={:.3f} cand_eval={} mean_cand_per_src={:.2f} mean_bucket={:.2f} peak_bucket={}",
        iap::to_string(iap::BSplineLidarFactorBackend::CPU_GICP),
        lidar_window_summary.result_count,
        lidar_window_summary.valid_profile_count,
        lidar_window_summary.warning_result_count,
        lidar_window_summary.total_source_point_count,
        lidar_window_summary.total_target_point_count,
        lidar_window_summary.total_matched_point_count,
        lidar_window_summary.total_inlier_point_count,
        lidar_window_summary.weighted_match_ratio,
        lidar_window_summary.weighted_inlier_ratio,
        lidar_window_summary.mean_unique_target_ratio,
        lidar_window_summary.min_unique_target_ratio,
        lidar_window_summary.max_target_reuse_ratio,
        lidar_window_summary.max_ambiguity_rejection_ratio,
        lidar_window_summary.max_numeric_rel_error,
        lidar_window_summary.max_rotation_axis_rel_error,
        lidar_window_summary.total_pose_update_ms,
        lidar_window_summary.total_correspondence_ms,
        lidar_window_summary.total_accumulation_ms,
        lidar_window_summary.total_factor_ms,
        lidar_window_summary.total_candidate_evaluation_count,
        lidar_window_summary.mean_candidates_per_source,
        lidar_window_summary.mean_time_bucket_population,
        lidar_window_summary.max_time_bucket_population);
    } else {
      logger->trace(
        "bspline ct lidar gpu-summary backend={} segments={} profiled={} warnings={} total_src={} total_tgt={} total_matched={} total_inliers={} weighted_match={:.3f} weighted_inlier={:.3f} mean_unique_ratio={:.3f} min_unique_ratio={:.3f} max_reuse_ratio={:.3f} max_ambiguity_rej={:.3f} max_numeric_rel={:.6f} max_axis_rel={:.6f} total_pose_ms={:.3f} total_corr_ms={:.3f} total_accum_ms={:.3f} total_factor_ms={:.3f} cand_eval={} mean_cand_per_src={:.2f} mean_bucket={:.2f} peak_bucket={}",
        iap::to_string(iap::BSplineLidarFactorBackend::GPU_GICP),
        lidar_window_summary.result_count,
        lidar_window_summary.valid_profile_count,
        lidar_window_summary.warning_result_count,
        lidar_window_summary.total_source_point_count,
        lidar_window_summary.total_target_point_count,
        lidar_window_summary.total_matched_point_count,
        lidar_window_summary.total_inlier_point_count,
        lidar_window_summary.weighted_match_ratio,
        lidar_window_summary.weighted_inlier_ratio,
        lidar_window_summary.mean_unique_target_ratio,
        lidar_window_summary.min_unique_target_ratio,
        lidar_window_summary.max_target_reuse_ratio,
        lidar_window_summary.max_ambiguity_rejection_ratio,
        lidar_window_summary.max_numeric_rel_error,
        lidar_window_summary.max_rotation_axis_rel_error,
        lidar_window_summary.total_pose_update_ms,
        lidar_window_summary.total_correspondence_ms,
        lidar_window_summary.total_accumulation_ms,
        lidar_window_summary.total_factor_ms,
        lidar_window_summary.total_candidate_evaluation_count,
        lidar_window_summary.mean_candidates_per_source,
        lidar_window_summary.mean_time_bucket_population,
        lidar_window_summary.max_time_bucket_population);
      logger->trace("bspline ct lidar gpu-backend backend={}", ::glim::to_string(lidar_gpu_backend_));
    }
  }

  if (lidar_factor_profile_) {
    const auto& current_segment = fixed_lag_registry_.segments().back();
    const auto& profile = current_lidar_result.profile;
    if (!use_gpu_lidar) {
      logger->trace(
        "bspline ct lidar factor target_mode={} jacobian_mode={} k_candidates={} accept_ratio={:.3f} score_gap={:.3f} robust_kernel={} robust_width={:.3f} robust_w_floor={:.3f} outlier_thresh={:.3f} target_frames={} target_points={} snapshot_frames={} snapshot_points={} snapshot_span_s={:.3f} snapshot_policy={} target_build_ms={:.3f} stage={} time_buckets={} bucket_mean={:.2f} bucket_peak={} cand_eval={} cand_per_src={:.2f} matched={}/{} inliers={} rej_dist={} rej_ambiguity={} rej_outlier={} rej_robust={} match_ratio={:.3f} inlier_ratio={:.3f} mean_w={:.3f} uniq_targets={} uniq_ratio={:.3f} reuse_peak={} reuse_ratio={:.3f} mean_dist={:.4f} max_dist={:.4f} mean_score={:.4f} mean_gap={:.4f} mean_ratio={:.4f} pose_ms={:.3f} corr_ms={:.3f} accum_ms={:.3f} total_ms={:.3f} error={:.6f}",
        ::glim::to_string(current_segment.target_mode),
        ::glim::to_string(lidar_jacobian_mode_),
        lidar_correspondence_candidate_count_,
        lidar_correspondence_accept_ratio_,
        lidar_correspondence_min_score_gap_,
        ::glim::to_string(lidar_robust_kernel_),
        lidar_robust_kernel_width_,
        lidar_robust_weight_floor_,
        lidar_outlier_mahalanobis_thresh_,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        current_segment.snapshot_frame_count,
        current_segment.snapshot_point_count,
        current_segment.snapshot_span_sec,
        current_segment.snapshot_policy_accepted,
        current_segment.target_build_ms,
        profile.stage,
        profile.time_bucket_count,
        profile.mean_time_bucket_population,
        profile.max_time_bucket_population,
        profile.candidate_evaluation_count,
        profile.mean_candidates_per_source,
        profile.matched_point_count,
        profile.source_point_count,
        profile.inlier_point_count,
        profile.rejected_distance_count,
        profile.rejected_ambiguity_count,
        profile.rejected_outlier_count,
        profile.rejected_robust_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.mean_robust_weight,
        profile.unique_target_count,
        profile.unique_target_ratio,
        profile.max_target_reuse,
        profile.max_target_reuse_ratio,
        profile.mean_match_distance,
        profile.max_match_distance,
        profile.mean_match_score,
        profile.mean_score_gap,
        profile.mean_score_ratio,
        profile.pose_update_ms,
        profile.correspondence_ms,
        profile.accumulation_ms,
        profile.total_ms,
        profile.total_error);
    } else {
      logger->trace(
        "bspline ct lidar gpu-factor target_mode={} jacobian_mode={} k_candidates={} accept_ratio={:.3f} score_gap={:.3f} robust_kernel={} robust_width={:.3f} robust_w_floor={:.3f} outlier_thresh={:.3f} target_frames={} target_points={} snapshot_frames={} snapshot_points={} snapshot_span_s={:.3f} snapshot_policy={} target_build_ms={:.3f} stage={} time_buckets={} bucket_mean={:.2f} bucket_peak={} cand_eval={} cand_per_src={:.2f} matched={}/{} inliers={} rej_dist={} rej_ambiguity={} rej_outlier={} rej_robust={} match_ratio={:.3f} inlier_ratio={:.3f} mean_w={:.3f} uniq_targets={} uniq_ratio={:.3f} reuse_peak={} reuse_ratio={:.3f} mean_dist={:.4f} max_dist={:.4f} mean_score={:.4f} mean_gap={:.4f} mean_ratio={:.4f} pose_ms={:.3f} corr_ms={:.3f} accum_ms={:.3f} total_ms={:.3f} error={:.6f}",
        ::glim::to_string(current_segment.target_mode),
        ::glim::to_string(lidar_jacobian_mode_),
        lidar_correspondence_candidate_count_,
        lidar_correspondence_accept_ratio_,
        lidar_correspondence_min_score_gap_,
        ::glim::to_string(lidar_robust_kernel_),
        lidar_robust_kernel_width_,
        lidar_robust_weight_floor_,
        lidar_outlier_mahalanobis_thresh_,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        current_segment.snapshot_frame_count,
        current_segment.snapshot_point_count,
        current_segment.snapshot_span_sec,
        current_segment.snapshot_policy_accepted,
        current_segment.target_build_ms,
        profile.stage,
        profile.time_bucket_count,
        profile.mean_time_bucket_population,
        profile.max_time_bucket_population,
        profile.candidate_evaluation_count,
        profile.mean_candidates_per_source,
        profile.matched_point_count,
        profile.source_point_count,
        profile.inlier_point_count,
        profile.rejected_distance_count,
        profile.rejected_ambiguity_count,
        profile.rejected_outlier_count,
        profile.rejected_robust_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.mean_robust_weight,
        profile.unique_target_count,
        profile.unique_target_ratio,
        profile.max_target_reuse,
        profile.max_target_reuse_ratio,
        profile.mean_match_distance,
        profile.max_match_distance,
        profile.mean_match_score,
        profile.mean_score_gap,
        profile.mean_score_ratio,
        profile.pose_update_ms,
        profile.correspondence_ms,
        profile.accumulation_ms,
        profile.total_ms,
        current_lidar_result.factor_error);
    }
  }

  if (!use_gpu_lidar && lidar_validate_linearization_) {
    const auto check = current_cpu_factor->check_linearization(values, lidar_linearization_check_scale_);
    if (check.valid) {
      const auto level =
        check.rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar linearization target_mode={} perturb={:.2e} base_error={:.6f} predicted={:.6f} actual={:.6f} abs={:.6e} rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.base_error,
        check.predicted_error,
        check.actual_error,
        check.abs_error,
        check.rel_error);
    }
  }

  if (!use_gpu_lidar && lidar_profile_numeric_reference_) {
    if (current_numeric_check_valid) {
      const auto& check = current_numeric_check;
      const double max_rel_error = std::max(check.rotation_rel_error, check.translation_rel_error);
      const auto level =
        max_rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar numeric-reference target_mode={} perturb={:.2e} rot_pred_num={:.6f} rot_pred_semi={:.6f} rot_actual={:.6f} rot_abs={:.6e} rot_rel={:.6f} rot_axis_max_rel={:.6f} rot_axis_mean_rel={:.6f} rot_axis_worst={} trans_pred_num={:.6f} trans_pred_semi={:.6f} trans_actual={:.6f} trans_abs={:.6e} trans_rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.numeric_rotation_predicted_error,
        check.semi_rotation_predicted_error,
        check.rotation_actual_error,
        check.rotation_abs_error,
        check.rotation_rel_error,
        check.max_rotation_axis_rel_error,
        check.mean_rotation_axis_rel_error,
        check.worst_rotation_axis,
        check.numeric_translation_predicted_error,
        check.semi_translation_predicted_error,
        check.translation_actual_error,
        check.translation_abs_error,
        check.translation_rel_error);
    }
  }

  if (use_gpu_lidar && lidar_profile_numeric_reference_) {
#ifdef GTSAM_POINTS_USE_CUDA
    if (current_gpu_numeric_check_valid) {
      const auto& check = current_gpu_numeric_check;
      const double max_rel_error = std::max(check.rotation_rel_error, check.translation_rel_error);
      const auto level =
        max_rel_error > lidar_linearization_warn_ratio_ ? spdlog::level::warn : spdlog::level::trace;
      logger->log(
        level,
        "bspline ct lidar gpu numeric-reference target_mode={} perturb={:.2e} rot_pred_num={:.6f} rot_pred_semi={:.6f} rot_actual={:.6f} rot_abs={:.6e} rot_rel={:.6f} rot_axis_max_rel={:.6f} rot_axis_mean_rel={:.6f} rot_axis_worst={} trans_pred_num={:.6f} trans_pred_semi={:.6f} trans_actual={:.6f} trans_abs={:.6e} trans_rel={:.6f}",
        ::glim::to_string(fixed_lag_registry_.segments().back().target_mode),
        check.perturbation_scale,
        check.numeric_rotation_predicted_error,
        check.semi_rotation_predicted_error,
        check.rotation_actual_error,
        check.rotation_abs_error,
        check.rotation_rel_error,
        check.max_rotation_axis_rel_error,
        check.mean_rotation_axis_rel_error,
        check.worst_rotation_axis,
        check.numeric_translation_predicted_error,
        check.semi_translation_predicted_error,
        check.translation_actual_error,
        check.translation_abs_error,
        check.translation_rel_error);
    }
#endif
  }

  if (!use_gpu_lidar && lidar_warn_degeneracy_) {
    const auto& diagnostics = current_degeneracy;
    const bool snapshot_fallback =
      lidar_target_mode_ == BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT && !fixed_lag_registry_.segments().back().snapshot_policy_accepted;
    if (snapshot_fallback || (current_degeneracy_valid && diagnostics.has_warning())) {
      std::string flags;
      auto append_flag = [&](const char* flag) {
        if (!flags.empty()) {
          flags += "|";
        }
        flags += flag;
      };

      if (snapshot_fallback) {
        append_flag("snapshot_fallback");
      }
      if (diagnostics.empty_target) {
        append_flag("empty_target");
      }
      if (diagnostics.low_match_ratio) {
        append_flag("low_match");
      }
      if (diagnostics.low_inlier_ratio) {
        append_flag("low_inlier");
      }
      if (diagnostics.low_target_diversity) {
        append_flag("low_target_diversity");
      }
      if (diagnostics.high_target_reuse) {
        append_flag("high_target_reuse");
      }
      if (diagnostics.high_ambiguity_rejection) {
        append_flag("high_ambiguity_rejection");
      }
      if (diagnostics.weak_score_separation) {
        append_flag("weak_score_separation");
      }
      if (flags.empty()) {
        flags = "none";
      }

      const auto& current_segment = fixed_lag_registry_.segments().back();
      const auto& profile = current_cpu_factor->last_profiling_stats();
      logger->warn(
        "bspline ct lidar degeneracy target_mode={} flags={} snapshot_policy={} target_frames={} target_points={} match_ratio={:.3f} inlier_ratio={:.3f} uniq_ratio={:.3f} reuse_ratio={:.3f} ambiguity_rej_ratio={:.3f} score_gap={:.4f} cand_eval={} bucket_peak={} bucket_mean={:.2f}",
        ::glim::to_string(current_segment.target_mode),
        flags,
        current_segment.snapshot_policy_accepted,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.unique_target_ratio,
        profile.max_target_reuse_ratio,
        diagnostics.ambiguity_rejection_ratio,
        profile.mean_score_gap,
        profile.candidate_evaluation_count,
        profile.max_time_bucket_population,
        profile.mean_time_bucket_population);
    }
  }

  if (use_gpu_lidar && lidar_warn_degeneracy_) {
#ifdef GTSAM_POINTS_USE_CUDA
    const auto& diagnostics = current_gpu_degeneracy;
    const bool snapshot_fallback =
      lidar_target_mode_ == BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT && !fixed_lag_registry_.segments().back().snapshot_policy_accepted;
    if (snapshot_fallback || (current_gpu_degeneracy_valid && diagnostics.has_warning())) {
      std::string flags;
      auto append_flag = [&](const char* flag) {
        if (!flags.empty()) {
          flags += "|";
        }
        flags += flag;
      };

      if (snapshot_fallback) {
        append_flag("snapshot_fallback");
      }
      if (diagnostics.empty_target) {
        append_flag("empty_target");
      }
      if (diagnostics.low_match_ratio) {
        append_flag("low_match");
      }
      if (diagnostics.low_inlier_ratio) {
        append_flag("low_inlier");
      }
      if (diagnostics.low_target_diversity) {
        append_flag("low_target_diversity");
      }
      if (diagnostics.high_target_reuse) {
        append_flag("high_target_reuse");
      }
      if (diagnostics.high_ambiguity_rejection) {
        append_flag("high_ambiguity_rejection");
      }
      if (diagnostics.weak_score_separation) {
        append_flag("weak_score_separation");
      }
      if (flags.empty()) {
        flags = "none";
      }

      const auto& current_segment = fixed_lag_registry_.segments().back();
      const auto& profile = current_lidar_result.profile;
      logger->warn(
        "bspline ct lidar gpu degeneracy target_mode={} flags={} snapshot_policy={} target_frames={} target_points={} match_ratio={:.3f} inlier_ratio={:.3f} uniq_ratio={:.3f} reuse_ratio={:.3f} ambiguity_rej_ratio={:.3f} score_gap={:.4f} cand_eval={} bucket_peak={} bucket_mean={:.2f}",
        ::glim::to_string(current_segment.target_mode),
        flags,
        current_segment.snapshot_policy_accepted,
        current_segment.target_frame_count,
        current_segment.target_point_count,
        profile.match_ratio,
        profile.inlier_ratio,
        profile.unique_target_ratio,
        profile.max_target_reuse_ratio,
        diagnostics.ambiguity_rejection_ratio,
        profile.mean_score_gap,
        profile.candidate_evaluation_count,
        profile.max_time_bucket_population,
        profile.mean_time_bucket_population);
    }
#endif
  }

  pipeline_timing.post_lidar_log_ms = elapsed_ms(t_post_lidar_log_start, Clock::now());

  const auto t_post_frame_state_start = Clock::now();
  const gtsam::Pose3 start_pose = control_window_->evaluate(0.0);
  const gtsam::Pose3 end_pose = control_window_->evaluate(1.0);
  new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
  new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
  new_frame->v_world_imu = values.at<gtsam::Vector3>(iap::bspline_velocity_key(control_window_->states()[1].index));
  new_frame->imu_bias.head<3>() = fixed_lag_registry_.shared_state().accel_bias;
  new_frame->imu_bias.tail<3>() = fixed_lag_registry_.shared_state().gyro_bias;
  const gtsam::Key current_clock_key = iap::bspline_clock_key(control_window_->states()[1].index);
  if (values.exists(current_clock_key)) {
    const auto clock = values.at<gtsam::Vector2>(current_clock_key);
    new_frame->clk_bias = clock(0);
    new_frame->clk_drift = clock(1);
  } else if (!frames.empty() && frames.back()) {
    new_frame->clk_bias = frames.back()->clk_bias;
    new_frame->clk_drift = frames.back()->clk_drift;
  }
  if (current_lidar_result.valid) {
    const double factor_error = current_lidar_result.factor_error;
    new_frame->icp_quality.inlier_count = current_lidar_result.inlier_count;
    new_frame->icp_quality.inlier_fraction = current_lidar_result.inlier_fraction;
    new_frame->icp_quality.rmse =
      std::sqrt(factor_error / std::max(new_frame->icp_quality.inlier_count, 1));
  }
  pipeline_timing.post_frame_state_ms = elapsed_ms(t_post_frame_state_start, Clock::now());

  std::vector<Eigen::Vector4d> deskewed_points;
  const auto t_post_deskew_start = Clock::now();
  if (!use_gpu_lidar) {
    deskewed_points = current_cpu_factor->deskewed_source_points(values, true);
  } else {
#ifdef GTSAM_POINTS_USE_CUDA
    if (lidar_gpu_backend_ == BSplineGpuLidarBackend::BUCKET) {
      deskewed_points = current_gpu_factor->deskewed_source_points(values, true);
    } else if (current_gpu_kernel_factor) {
      deskewed_points = current_gpu_kernel_factor->deskewed_source_points(values, true);
    }
#endif
  }
  pipeline_timing.post_deskew_ms = elapsed_ms(t_post_deskew_start, Clock::now());
  const auto t_post_covariance_start = Clock::now();
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  pipeline_timing.post_covariance_ms = elapsed_ms(t_post_covariance_start, Clock::now());
  const auto t_post_frame_store_start = Clock::now();
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }
  pipeline_timing.post_frame_store_ms = elapsed_ms(t_post_frame_store_start, Clock::now());

  {
    const auto t_post_callback_start = Clock::now();
    Callbacks::on_new_frame(new_frame);
    pipeline_timing.post_callback_ms += elapsed_ms(t_post_callback_start, Clock::now());
  }
  {
    const auto t_post_target_insert_start = Clock::now();
    insert_target_cloud(new_frame);
    pipeline_timing.post_target_insert_ms = elapsed_ms(t_post_target_insert_start, Clock::now());
  }
  {
    const auto t_post_history_update_start = Clock::now();
    update_frame_history(new_frame, marginalized_frames);
    pipeline_timing.post_history_update_ms = elapsed_ms(t_post_history_update_start, Clock::now());
  }
  {
    const auto t_post_publish_traj_start = Clock::now();
    publish_continuous_trajectory(current);
    pipeline_timing.post_publish_traj_ms = elapsed_ms(t_post_publish_traj_start, Clock::now());
  }
  {
    const auto t_post_publish_telemetry_start = Clock::now();
    publish_fixed_lag_telemetry(current);
    pipeline_timing.post_publish_telemetry_ms = elapsed_ms(t_post_publish_telemetry_start, Clock::now());
  }

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  if (!active_frames.empty()) {
    const auto t_post_callback_start = Clock::now();
    Callbacks::on_update_new_frame(active_frames.back());
    Callbacks::on_update_frames(active_frames);
    pipeline_timing.post_callback_ms += elapsed_ms(t_post_callback_start, Clock::now());
  }

  pipeline_timing.postprocess_ms = elapsed_ms(t_postprocess_start, Clock::now());
  pipeline_timing.window_wall_ms = elapsed_ms(t_window_start, Clock::now());

  if (pipeline_profile_) {
    const double profiled_stage_sum =
      pipeline_timing.gnss_mailbox_sync_ms +
      pipeline_timing.source_cloud_ms +
      pipeline_timing.segment_prepare_ms +
      pipeline_timing.gnss_epoch_fetch_ms +
      pipeline_timing.marginalization_partition_ms +
      pipeline_timing.graph_build_ms +
      pipeline_timing.lm_optimize_ms +
      pipeline_timing.marginalization_ms +
      pipeline_timing.postprocess_ms;
    const double unprofiled_ms = std::max(0.0, pipeline_timing.window_wall_ms - profiled_stage_sum);
    logger->info(
      "bspline ct pipeline-summary frontend={} gpu_backend={} lidar_result_scope={} frame={} scan_dt={:.3f} smoother_lag={:.2f} active_states={} active_segments={} gnss_mailbox_sync_ms={:.3f} source_cloud_ms={:.3f} segment_prepare_ms={:.3f} target_build_ms={:.3f} gnss_epoch_fetch_ms={:.3f} marginalization_partition_ms={:.3f} graph_build_ms={:.3f} graph_lidar_factor_ms={:.3f} graph_lidar_factor_new_build_ms={:.3f} graph_lidar_factor_target_refresh_ms={:.3f} graph_lidar_factor_reused_attach_ms={:.3f} graph_lidar_factor_cache_hits={} graph_lidar_factor_cache_misses={} graph_lidar_factor_refreshes={} graph_velocity_factor_ms={:.3f} imu_factor_assembly_ms={:.3f} gnss_factor_assembly_ms={:.3f} carried_prior_attach_ms={:.3f} graph_prediction_prior_ms={:.3f} graph_smoothness_ms={:.3f} graph_shared_prior_ms={:.3f} graph_clock_factor_ms={:.3f} lm_optimize_ms={:.3f} marginalization_ms={:.3f} prune_active_ms={:.3f} carried_prior_update_ms={:.3f} postprocess_ms={:.3f} post_lidar_result_ms={:.3f} post_lidar_factor_error_ms={:.3f} post_lidar_numeric_audit_ms={:.3f} post_lidar_degeneracy_ms={:.3f} post_lidar_result_pack_ms={:.3f} post_lidar_window_aggregate_ms={:.3f} post_lidar_csv_ms={:.3f} post_lidar_log_ms={:.3f} post_frame_state_ms={:.3f} post_deskew_ms={:.3f} post_covariance_ms={:.3f} post_frame_store_ms={:.3f} post_target_insert_ms={:.3f} post_history_update_ms={:.3f} post_publish_traj_ms={:.3f} post_publish_telemetry_ms={:.3f} post_callback_ms={:.3f} wall_ms={:.3f} unprofiled_ms={:.3f} lidar_factor_ms={:.3f} lidar_pose_ms={:.3f} lidar_corr_ms={:.3f} factor_results={} imu_factors={} gnss_pr_factors={} gnss_dop_factors={} velocity_factors={}",
      frontend_mode_,
      ::glim::to_string(lidar_gpu_backend_),
      collect_window_lidar_results ? "window" : "current_only",
      new_frame->id,
      scan_duration,
      params->smoother_lag,
      fixed_lag_registry_.control_buffer().states().size(),
      fixed_lag_registry_.segments().size(),
      pipeline_timing.gnss_mailbox_sync_ms,
      pipeline_timing.source_cloud_ms,
      pipeline_timing.segment_prepare_ms,
      pipeline_timing.target_build_ms,
      pipeline_timing.gnss_epoch_fetch_ms,
      pipeline_timing.marginalization_partition_ms,
      pipeline_timing.graph_build_ms,
      pipeline_timing.graph_lidar_factor_ms,
      pipeline_timing.graph_lidar_factor_new_build_ms,
      pipeline_timing.graph_lidar_factor_target_refresh_ms,
      pipeline_timing.graph_lidar_factor_reused_attach_ms,
      pipeline_timing.graph_lidar_factor_cache_hit_count,
      pipeline_timing.graph_lidar_factor_cache_miss_count,
      pipeline_timing.graph_lidar_factor_refresh_count,
      pipeline_timing.graph_velocity_factor_ms,
      pipeline_timing.imu_factor_assembly_ms,
      pipeline_timing.gnss_factor_assembly_ms,
      pipeline_timing.carried_prior_attach_ms,
      pipeline_timing.graph_prediction_prior_ms,
      pipeline_timing.graph_smoothness_ms,
      pipeline_timing.graph_shared_prior_ms,
      pipeline_timing.graph_clock_factor_ms,
      pipeline_timing.lm_optimize_ms,
      pipeline_timing.marginalization_ms,
      pipeline_timing.prune_active_ms,
      pipeline_timing.carried_prior_update_ms,
      pipeline_timing.postprocess_ms,
      pipeline_timing.post_lidar_result_ms,
      pipeline_timing.post_lidar_factor_error_ms,
      pipeline_timing.post_lidar_numeric_audit_ms,
      pipeline_timing.post_lidar_degeneracy_ms,
      pipeline_timing.post_lidar_result_pack_ms,
      pipeline_timing.post_lidar_window_aggregate_ms,
      pipeline_timing.post_lidar_csv_ms,
      pipeline_timing.post_lidar_log_ms,
      pipeline_timing.post_frame_state_ms,
      pipeline_timing.post_deskew_ms,
      pipeline_timing.post_covariance_ms,
      pipeline_timing.post_frame_store_ms,
      pipeline_timing.post_target_insert_ms,
      pipeline_timing.post_history_update_ms,
      pipeline_timing.post_publish_traj_ms,
      pipeline_timing.post_publish_telemetry_ms,
      pipeline_timing.post_callback_ms,
      pipeline_timing.window_wall_ms,
      unprofiled_ms,
      lidar_window_summary.total_factor_ms,
      lidar_window_summary.total_pose_update_ms,
      lidar_window_summary.total_correspondence_ms,
      lidar_window_summary.result_count,
      active_imu_factor_count,
      active_gnss_pr_factor_count,
      active_gnss_dop_factor_count,
      active_velocity_factor_count);
  }

  return new_frame;
}

void OdometryEstimationBSpline::publish_continuous_trajectory(int current) {
  (void)current;

  std::vector<iap::SplineControlPoint> control_points;
  if ((frontend_mode_ == "CT_LIDAR_CPU" || frontend_mode_ == "CT_LIDAR_GPU") &&
      !fixed_lag_registry_.control_buffer().empty()) {
    control_points = fixed_lag_registry_.control_buffer().spline_control_points(&fixed_lag_registry_.auxiliary_values());
  } else {
    control_points.reserve(frames.inner_size());

    for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
      if (!(*it)) {
        continue;
      }

      iap::SplineControlPoint cp;
      cp.stamp = (*it)->stamp;
      cp.pose = (*it)->T_world_imu;
      cp.vel = (*it)->v_world_imu;
      cp.sigma = sigma_from_covariance((*it)->sigma_p);
      control_points.push_back(cp);
    }
  }

  if (control_points.empty()) {
    return;
  }

  auto trajectory = std::make_shared<iap::BSplineTrajectory>(trajectory_params_);
  trajectory->set_control_points(control_points);
  latest_trajectory_ = trajectory;

  update_frame_attachment(trajectory);
  update_compatibility_trajectory(trajectory);

  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(trajectory);
    iap::IapSharedState::instance().set_spline_control_access(trajectory);
  }
}

void OdometryEstimationBSpline::publish_fixed_lag_telemetry(int current) const {
  (void)current;

  auto telemetry = fixed_lag_registry_.telemetry();

  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_bspline_fixed_lag_telemetry(telemetry);
  }

  logger->trace(
    "bspline fixed-lag telemetry state={} cps={} segs={} aux={} aux_values={} shared={} lag=[{:.3f}, {:.3f}] latest_segment=[{:.3f}, {:.3f}] anchor={}",
    iap::to_string(telemetry.lifecycle_state),
    telemetry.control_point_count,
    telemetry.segment_count,
    telemetry.active_auxiliary_count,
    telemetry.auxiliary_value_count,
    telemetry.active_shared_state_count,
    telemetry.lag_start_stamp,
    telemetry.lag_end_stamp,
    telemetry.latest_segment_stamp,
    telemetry.latest_segment_end,
    telemetry.gnss_anchor_initialized);
}

void OdometryEstimationBSpline::update_frame_attachment(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const {
  if (!attach_trajectory_to_frames_) {
    return;
  }

  const auto meta = trajectory->meta();
  for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
    if (!(*it)) {
      continue;
    }

    auto attachment = std::make_shared<iap::ContinuousTrajectoryAttachment>();
    attachment->trajectory_view = trajectory;
    attachment->control_access = trajectory;
    attachment->meta = meta;
    attachment->producer = "OdometryEstimationBSpline";
    (*it)->custom_data[iap::kContinuousTrajectoryAttachmentKey] = attachment;
  }
}

void OdometryEstimationBSpline::update_compatibility_trajectory(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const {
  if (compatibility_sample_dt_ <= 0.0) {
    return;
  }

  for (auto it = frames.inner_begin(); it != frames.inner_end(); ++it) {
    if (!(*it) || !(*it)->raw_frame) {
      continue;
    }

    const double scan_start = (*it)->stamp;
    const double scan_end = std::max(scan_start, (*it)->raw_frame->scan_end_time);
    if (scan_end < trajectory->start_time() || scan_start > trajectory->end_time()) {
      continue;
    }
    const auto samples = trajectory->sample_range(scan_start, scan_end, compatibility_sample_dt_);
    if (samples.empty()) {
      continue;
    }

    (*it)->imu_rate_trajectory.resize(8, samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto& sample = samples[i];
      const Eigen::Quaterniond q(sample.pose.linear());
      (*it)->imu_rate_trajectory.col(static_cast<Eigen::Index>(i))
        << sample.stamp,
           sample.pose.translation(),
           q.x(), q.y(), q.z(), q.w();
    }
  }
}

void OdometryEstimationBSpline::maybe_export_lidar_baseline_csv(
  double stamp,
  const std::vector<iap::BSplineLidarFactorResult>& results,
  int current_factor_index) {
  if (!lidar_export_baseline_csv_) {
    return;
  }

  const auto export_data =
    iap::make_bspline_lidar_baseline_export(stamp, frontend_mode_.c_str(), results, current_factor_index);
  if (!export_data.valid) {
    return;
  }

  const std::filesystem::path csv_path(lidar_baseline_csv_path_);
  if (csv_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(csv_path.parent_path(), ec);
    if (ec) {
      logger->warn(
        "failed to create CT LiDAR baseline CSV parent directory '{}': {}",
        csv_path.parent_path().string(),
        ec.message());
      return;
    }
  }

  std::FILE* file = std::fopen(lidar_baseline_csv_path_.c_str(), lidar_baseline_csv_header_written_ ? "a" : "w");
  if (!file) {
    logger->warn("failed to open CT LiDAR baseline CSV '{}': {}", lidar_baseline_csv_path_, std::strerror(errno));
    return;
  }

  if (!lidar_baseline_csv_header_written_) {
    iap::write_bspline_lidar_baseline_csv_header(file);
    lidar_baseline_csv_header_written_ = true;
  }

  iap::write_bspline_lidar_baseline_csv(file, export_data);
  std::fclose(file);

  if (!lidar_baseline_csv_first_row_logged_) {
    logger->info("bspline ct lidar baseline-csv first_rows_written path={}", lidar_baseline_csv_path_);
    lidar_baseline_csv_first_row_logged_ = true;
  }

  logger->trace(
    "bspline ct lidar baseline-csv path={} backend={} rows={} current_factor={} warnings={} total_ms={:.3f}",
    lidar_baseline_csv_path_,
    iap::to_string(export_data.backend),
    export_data.factor_results.size() + 1,
    export_data.current_factor_index,
    export_data.summary.warning_result_count,
    export_data.summary.total_factor_ms);
}

}  // namespace glim
