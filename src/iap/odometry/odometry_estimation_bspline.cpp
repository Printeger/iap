#include <iap/odometry/odometry_estimation_bspline.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
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

namespace glim {

namespace {

using Callbacks = OdometryEstimationCallbacks;

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
  frontend_mode_ = config.param<std::string>("odometry_estimation", "frontend_mode", "RECONSTRUCT");
  max_correspondence_distance_ = config.param<double>("odometry_estimation", "max_correspondence_distance", 1.5);
  lidar_target_mode_ = parse_lidar_target_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_target_mode", "ACTIVE_WINDOW_SNAPSHOT"));
  lidar_jacobian_mode_ = parse_lidar_jacobian_mode(
    config.param<std::string>("odometry_estimation", "ct_lidar_jacobian_mode", "SEMI_ANALYTIC"));
  lidar_snapshot_frame_window_ = config.param<int>("odometry_estimation", "ct_lidar_snapshot_frame_window", 0);
  lidar_correspondence_candidate_count_ =
    config.param<int>("odometry_estimation", "ct_lidar_correspondence_candidates", 3);
  lidar_correspondence_accept_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_correspondence_accept_ratio", 0.0);
  lidar_jacobian_numeric_eps_ = config.param<double>("odometry_estimation", "ct_lidar_jacobian_numeric_eps", 1e-4);
  lidar_outlier_mahalanobis_thresh_ =
    config.param<double>("odometry_estimation", "ct_lidar_outlier_mahalanobis_thresh", 0.0);
  lidar_robust_kernel_ = parse_lidar_robust_kernel(
    config.param<std::string>("odometry_estimation", "ct_lidar_robust_kernel", "NONE"));
  lidar_robust_kernel_width_ = config.param<double>("odometry_estimation", "ct_lidar_robust_kernel_width", 1.0);
  lidar_factor_profile_ = config.param<bool>("odometry_estimation", "ct_lidar_profile_factor", false);
  lidar_validate_linearization_ = config.param<bool>("odometry_estimation", "ct_lidar_validate_linearization", false);
  lidar_linearization_check_scale_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_check_scale", 1e-4);
  lidar_linearization_warn_ratio_ =
    config.param<double>("odometry_estimation", "ct_lidar_linearization_warn_ratio", 0.25);
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

  logger->info("odometry_bspline initialized frontend_mode={} knot_mode={} nominal_dt={:.4f} compatibility_sample_dt={:.4f} lidar_target_mode={} lidar_jacobian_mode={} lidar_k_candidates={} lidar_accept_ratio={:.3f} lidar_snapshot_window={} lidar_outlier_thresh={:.3f} lidar_robust_kernel={} lidar_robust_width={:.3f} lidar_profile={} lidar_validate={}",
    frontend_mode_,
    iap::to_string(trajectory_params_.knot_mode),
    trajectory_params_.nominal_dt,
    compatibility_sample_dt_,
    ::glim::to_string(lidar_target_mode_),
    ::glim::to_string(lidar_jacobian_mode_),
    lidar_correspondence_candidate_count_,
    lidar_correspondence_accept_ratio_,
    lidar_snapshot_frame_window_,
    lidar_outlier_mahalanobis_thresh_,
    ::glim::to_string(lidar_robust_kernel_),
    lidar_robust_kernel_width_,
    lidar_factor_profile_,
    lidar_validate_linearization_);
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
  if (frontend_mode_ == "CT_LIDAR_CPU") {
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
    std::shared_ptr<gtsam_points::PointCloud> target_points =
      std::make_shared<gtsam_points::PointCloudCPU>(target_ref.target_snapshot->voxel_points());
    target_ref.target_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(target_points);
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
  for (std::size_t i = first_frame_index; i < active_frames.size(); ++i) {
    if (!active_frames[i] || !active_frames[i]->frame) {
      continue;
    }

    auto transformed = gtsam_points::PointCloudCPU::clone(*active_frames[i]->frame);
    for (int j = 0; j < transformed->size(); ++j) {
      transformed->points[j] = active_frames[i]->T_world_lidar * active_frames[i]->frame->points[j];
      transformed->covs[j] =
        active_frames[i]->T_world_lidar.matrix() * active_frames[i]->frame->covs[j] * active_frames[i]->T_world_lidar.matrix().transpose();
    }
    snapshot->insert(*transformed);
    inserted = true;
    target_ref.contributing_frames++;
  }

  target_ref.target_snapshot = inserted ? snapshot : ct_target_ivox_;
  std::shared_ptr<gtsam_points::PointCloud> target_points =
    std::make_shared<gtsam_points::PointCloudCPU>(target_ref.target_snapshot->voxel_points());
  target_ref.target_tree = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(target_points);
  target_ref.mode = inserted ? BSplineLidarTargetMode::ACTIVE_WINDOW_SNAPSHOT : BSplineLidarTargetMode::GLOBAL_IVOX_REFERENCE;
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

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);
  sync_gnss_epochs_from_shared_state();

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);

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

  new_frame->frame = create_lidar_source_cloud(raw_frame);
  const auto factor_source = gtsam_points::PointCloudCPU::clone(*new_frame->frame);
  append_active_segment_constraint(raw_frame, factor_source);
  if (!fixed_lag_registry_.segments().empty()) {
    fixed_lag_registry_.segments().back().gnss_epochs = consume_segment_gnss_epochs(
      raw_frame->stamp,
      raw_frame->scan_end_time);
  }

  gtsam::Values values = fixed_lag_registry_.control_buffer().values();
  const auto& active_states = fixed_lag_registry_.control_buffer().states();
  const iap::BSplineCarriedPrior previous_carried_prior = marginal_prior_.carried_prior;
  const gtsam::Key gyro_bias_key = bspline_gyro_bias_key();
  const gtsam::Key accel_bias_key = bspline_accel_bias_key();
  const gtsam::Key gravity_key = bspline_gravity_key();
  fixed_lag_registry_.seed_shared_values(values, false);

  gtsam::NonlinearFactorGraph graph;
  gtsam::NonlinearFactorGraph marginalization_graph;
  std::shared_ptr<iap::IntegratedBSplineGICPFactor> current_factor;
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

    if (!seeded_clock_states.empty()) {
      fixed_lag_registry_.seed_shared_values(values, true);
    }
  }
  const auto marginalization_partition = iap::build_bspline_marginalization_partition(
    fixed_lag_registry_.control_buffer().states(),
    fixed_lag_registry_.marginalization_segment_states(),
    values,
    min_active_stamp,
    !seeded_clock_states.empty());
  for (std::size_t i = 0; i < fixed_lag_registry_.segments().size(); ++i) {
    const auto& segment = fixed_lag_registry_.segments()[i];
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

    auto factor =
      std::make_shared<iap::IntegratedBSplineGICPFactor>(segment_keys, segment.target_snapshot, segment.source, segment.target_tree);
    factor->set_num_threads(params->num_threads);
    factor->set_max_correspondence_distance(max_correspondence_distance_);
    factor->set_jacobian_mode(lidar_jacobian_mode_);
    factor->set_numeric_eps(lidar_jacobian_numeric_eps_);
    factor->set_correspondence_candidate_count(lidar_correspondence_candidate_count_);
    factor->set_correspondence_accept_ratio(lidar_correspondence_accept_ratio_);
    factor->set_outlier_mahalanobis_threshold(lidar_outlier_mahalanobis_thresh_);
    factor->set_robust_kernel(lidar_robust_kernel_, lidar_robust_kernel_width_);
    factor->set_enable_profiling(lidar_factor_profile_);
    graph.add(factor);
    if (marginalization_partition.should_marginalize_factor(make_key_vector(segment_keys))) {
      marginalization_graph.add(factor);
    }

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

    for (const auto& imu_sample : segment.imu_samples) {
      auto imu_factor = std::make_shared<iap::IntegratedBSplineIMUFactor>(
        segment_keys,
        gyro_bias_key,
        accel_bias_key,
        gravity_key,
        imu_sample.u,
        std::max(1e-3, segment.scan_end - segment.stamp),
        imu_sample.angular_vel,
        imu_sample.linear_acc,
        gtsam::Pose3(T_lidar_imu.matrix()),
        imu_ct_trans_inf_scale_,
        imu_ct_rot_inf_scale_,
        trajectory_params_.finite_difference_dt);
      graph.add(imu_factor);
      if (marginalization_partition.should_marginalize_factor(
            make_key_vector(segment_keys, {gyro_bias_key, accel_bias_key, gravity_key}))) {
        marginalization_graph.add(imu_factor);
      }
      active_imu_factor_count++;
    }

    if (fixed_lag_registry_.shared_state().gnss_anchor_initialized && !segment.gnss_epochs.empty()) {
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
    }

    if (i + 1 == fixed_lag_registry_.segments().size()) {
      current_factor = factor;
    }
  }

  if (!current_factor) {
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
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gyro_bias_key, shared_state.gyro_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(accel_bias_key, shared_state.accel_bias, imu_bias_noise);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(gravity_key, shared_state.gravity, gravity_noise);
  if (shared_state.gnss_anchor_initialized && values.exists(ecef_origin_key) && values.exists(ecef_rot_key)) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(ecef_origin_key, shared_state.ecef_origin, gnss_ecef_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Rot3>>(ecef_rot_key, shared_state.ecef_rot, gnss_ecef_rot_noise);
  }
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

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setlambdaInitial(1e-4);
  lm_params.setAbsoluteErrorTol(1e-2);
  lm_params.setMaxIterations(lm_max_iterations_);

  try {
    values = gtsam_points::LevenbergMarquardtOptimizerExt(graph, values, lm_params).optimize();
  } catch (const std::exception& e) {
    logger->error("bspline ct frontend optimization failed: {}", e.what());
  }

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
  prune_active_ct_state(min_active_stamp);
  update_marginal_prior_from_active_window();
  update_marginal_prior_information(
    marginalization_graph,
    values,
    marginalization_partition.survivor_keys,
    previous_carried_prior.empty() ? nullptr : &previous_carried_prior);

  if (lidar_factor_profile_) {
    const auto& current_segment = fixed_lag_registry_.segments().back();
    const auto& profile = current_factor->last_profiling_stats();
    logger->trace(
      "bspline ct lidar factor target_mode={} jacobian_mode={} k_candidates={} accept_ratio={:.3f} robust_kernel={} robust_width={:.3f} outlier_thresh={:.3f} target_frames={} target_points={} target_build_ms={:.3f} stage={} matched={}/{} inliers={} rej_dist={} rej_ambiguity={} rej_outlier={} match_ratio={:.3f} inlier_ratio={:.3f} mean_w={:.3f} pose_ms={:.3f} corr_ms={:.3f} accum_ms={:.3f} total_ms={:.3f} error={:.6f}",
      ::glim::to_string(current_segment.target_mode),
      ::glim::to_string(lidar_jacobian_mode_),
      lidar_correspondence_candidate_count_,
      lidar_correspondence_accept_ratio_,
      ::glim::to_string(lidar_robust_kernel_),
      lidar_robust_kernel_width_,
      lidar_outlier_mahalanobis_thresh_,
      current_segment.target_frame_count,
      current_segment.target_point_count,
      current_segment.target_build_ms,
      profile.stage,
      profile.matched_point_count,
      profile.source_point_count,
      profile.inlier_point_count,
      profile.rejected_distance_count,
      profile.rejected_ambiguity_count,
      profile.rejected_outlier_count,
      profile.match_ratio,
      profile.inlier_ratio,
      profile.mean_robust_weight,
      profile.pose_update_ms,
      profile.correspondence_ms,
      profile.accumulation_ms,
      profile.total_ms,
      profile.total_error);
  }

  if (lidar_validate_linearization_) {
    const auto check = current_factor->check_linearization(values, lidar_linearization_check_scale_);
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
  if (current_factor) {
    const double factor_error = current_factor->error(values);
    new_frame->icp_quality.inlier_count = current_factor->num_inliers();
    new_frame->icp_quality.inlier_fraction = current_factor->inlier_fraction();
    new_frame->icp_quality.rmse =
      std::sqrt(factor_error / std::max(new_frame->icp_quality.inlier_count, 1));
  }

  const auto deskewed_points = current_factor->deskewed_source_points(values, true);
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }

  Callbacks::on_new_frame(new_frame);
  insert_target_cloud(new_frame);
  update_frame_history(new_frame, marginalized_frames);
  publish_continuous_trajectory(current);
  publish_fixed_lag_telemetry(current);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  if (!active_frames.empty()) {
    Callbacks::on_update_new_frame(active_frames.back());
    Callbacks::on_update_frames(active_frames);
  }

  return new_frame;
}

void OdometryEstimationBSpline::publish_continuous_trajectory(int current) {
  (void)current;

  std::vector<iap::SplineControlPoint> control_points;
  if (frontend_mode_ == "CT_LIDAR_CPU" && !fixed_lag_registry_.control_buffer().empty()) {
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

}  // namespace glim
