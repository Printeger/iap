#include <iap/odometry/odometry_estimation_bspline.hpp>

#include <Eigen/Eigenvalues>

#include <cstdint>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <spdlog/spdlog.h>

#include <iap/common/cloud_covariance_estimation.hpp>
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

double sigma_from_covariance(const Eigen::Matrix3d& sigma_p) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sigma_p, Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success) {
    return 0.0;
  }
  return std::sqrt(std::max(0.0, eig.eigenvalues().maxCoeff()));
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
  ctrl_point_anchor_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_anchor_inf_scale", 1e6);
  ctrl_point_prediction_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_prediction_inf_scale", 1e3);
  ctrl_point_smoothness_inf_scale_ = config.param<double>("odometry_estimation", "ctrl_point_smoothness_inf_scale", 1e2);
  lm_max_iterations_ = config.param<int>("odometry_estimation", "lm_max_iterations", 8);

  T_lidar_imu = params.T_lidar_imu;
  T_imu_lidar = T_lidar_imu.inverse();
  control_window_ = std::make_unique<iap::BSplineControlWindow>();
  control_buffer_ = std::make_unique<iap::BSplineControlWindowBuffer>();
  ct_target_ivox_ = std::make_shared<gtsam_points::iVox>(params.ivox_resolution);
  ct_target_ivox_->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
  ct_target_ivox_->set_lru_horizon(params.lru_thresh);
  ct_target_ivox_->set_neighbor_voxel_mode(1);

  logger->info("odometry_bspline initialized frontend_mode={} knot_mode={} nominal_dt={:.4f} compatibility_sample_dt={:.4f}",
    frontend_mode_,
    iap::to_string(trajectory_params_.knot_mode),
    trajectory_params_.nominal_dt,
    compatibility_sample_dt_);
}

OdometryEstimationBSpline::~OdometryEstimationBSpline() {
  if (publish_shared_trajectory_) {
    iap::IapSharedState::instance().set_continuous_trajectory_view(nullptr);
    iap::IapSharedState::instance().set_spline_control_access(nullptr);
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
  control_buffer_->reset_from_window(*control_window_);
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

  if (control_buffer_ && !control_buffer_->empty()) {
    const double min_active_stamp = std::max(0.0, frame->stamp - params->smoother_lag);
    control_buffer_->prune_before(min_active_stamp);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);
}

EstimationFrame::ConstPtr OdometryEstimationBSpline::insert_frame_ct_lidar(
  const PreprocessedFrame::Ptr& raw_frame,
  std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();
  const double scan_duration = std::max(1e-3, raw_frame->scan_end_time - raw_frame->stamp);

  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;
  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->raw_frame = raw_frame;
  new_frame->frame_id = FrameID::LIDAR;
  new_frame->v_world_imu.setZero();
  new_frame->imu_bias.setZero();

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

    Callbacks::on_new_frame(new_frame);
    insert_target_cloud(new_frame);
    update_frame_history(new_frame, marginalized_frames);
    publish_continuous_trajectory(current);

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
  control_buffer_->append_window(*control_window_);

  new_frame->frame = create_lidar_source_cloud(raw_frame);

  gtsam::Values values = control_buffer_->values();
  const auto& active_states = control_buffer_->states();
  const auto keys = control_window_->keys();

  gtsam::NonlinearFactorGraph graph;
  auto factor = std::make_shared<iap::IntegratedBSplineGICPFactor>(keys, ct_target_ivox_, new_frame->frame, ct_target_ivox_);
  factor->set_num_threads(params->num_threads);
  factor->set_max_correspondence_distance(max_correspondence_distance_);
  graph.add(factor);

  const auto anchor_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_anchor_inf_scale_);
  const auto pred_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_prediction_inf_scale_);
  const auto smooth_noise = gtsam::noiseModel::Isotropic::Precision(6, ctrl_point_smoothness_inf_scale_);

  if (!active_states.empty()) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states.front().index),
      active_states.front().pose,
      anchor_noise);
  }
  if (active_states.size() >= 2) {
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[1].index),
      active_states[1].pose,
      anchor_noise);
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
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      iap::bspline_control_point_key(active_states[i].index),
      iap::bspline_control_point_key(active_states[i + 1].index),
      active_states[i].pose.between(active_states[i + 1].pose),
      smooth_noise);
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

  logger->trace("bspline ct active_window={} current_segment_keys={} {} {} {}",
    active_states.size(),
    static_cast<std::uint64_t>(keys[0]),
    static_cast<std::uint64_t>(keys[1]),
    static_cast<std::uint64_t>(keys[2]),
    static_cast<std::uint64_t>(keys[3]));

  control_buffer_->update_from_values(values);
  control_window_->update_from_values(values);

  const gtsam::Pose3 start_pose = control_window_->evaluate(0.0);
  const gtsam::Pose3 end_pose = control_window_->evaluate(1.0);
  new_frame->T_world_lidar = Eigen::Isometry3d(start_pose.matrix());
  new_frame->T_world_imu = new_frame->T_world_lidar * T_lidar_imu;
  new_frame->v_world_imu = (end_pose.translation() - start_pose.translation()) / scan_duration;

  const auto deskewed_points = factor->deskewed_source_points(values, true);
  const auto deskewed_covs = covariance_estimation->estimate(deskewed_points, raw_frame->neighbors);
  for (int i = 0; i < new_frame->frame->size(); ++i) {
    new_frame->frame->points[i] = deskewed_points[static_cast<std::size_t>(i)];
    new_frame->frame->covs[i] = deskewed_covs[static_cast<std::size_t>(i)];
  }

  Callbacks::on_new_frame(new_frame);
  insert_target_cloud(new_frame);
  update_frame_history(new_frame, marginalized_frames);
  publish_continuous_trajectory(current);

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
  if (frontend_mode_ == "CT_LIDAR_CPU" && control_buffer_ && !control_buffer_->empty()) {
    control_points = control_buffer_->spline_control_points();
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
