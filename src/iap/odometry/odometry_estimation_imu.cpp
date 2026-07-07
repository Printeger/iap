#include <iap/odometry/odometry_estimation_imu.hpp>
#include <Eigen/Eigenvalues>

#include <spdlog/spdlog.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/util/config.hpp>
#include <iap/util/convert_to_string.hpp>
#include <iap/util/key_lifecycle_monitor.hpp>
#include <iap/util/relinearization_policy.hpp>
#include <iap/util/shared_state.hpp>
#include <iap/util/timing_csv.hpp>
#include <iap/common/imu_integration.hpp>
#include <iap/common/imu_validation.hpp>
#include <iap/common/cloud_deskewing.hpp>
#include <iap/common/cloud_covariance_estimation.hpp>
#include <iap/odometry/initial_state_estimation.hpp>
#include <iap/odometry/loose_initial_state_estimation.hpp>
#include <iap/odometry/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::C;  // clock state [δt(m), δṫ(m/s)] (IAP-RQ-010)
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

namespace {

OdometryEstimationIMUParams::SigmaPUpdateScope parse_sigma_p_update_scope(const std::string& scope) {
  if (scope == "current" || scope == "CURRENT") {
    return OdometryEstimationIMUParams::SigmaPUpdateScope::CURRENT;
  }
  if (scope == "all_active" || scope == "ALL_ACTIVE") {
    return OdometryEstimationIMUParams::SigmaPUpdateScope::ALL_ACTIVE;
  }

  spdlog::warn("unknown odometry_estimation.sigma_p_update_scope='{}'; falling back to 'current'", scope);
  return OdometryEstimationIMUParams::SigmaPUpdateScope::CURRENT;
}

}  // namespace

OdometryEstimationIMUParams::OdometryEstimationIMUParams() {
  // sensor config
  Config sensor_config(GlobalConfig::get_config_path("config_sensors"));
  T_lidar_imu = sensor_config.param<Eigen::Isometry3d>("sensors", "T_lidar_imu", Eigen::Isometry3d::Identity());
  imu_bias_noise = sensor_config.param<double>("sensors", "imu_bias_noise", 1e-3);
  auto bias = sensor_config.param<std::vector<double>>("sensors", "imu_bias");
  if (bias && bias->size() == 6) {
    imu_bias = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(bias->data());
  } else {
    imu_bias.setZero();
  }

  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  fix_imu_bias = config.param<bool>("odometry_estimation", "fix_imu_bias", false);

  initialization_mode = config.param<std::string>("odometry_estimation", "initialization_mode", "LOOSE");
  const auto init_T_world_imu = config.param<Eigen::Isometry3d>("odometry_estimation", "init_T_world_imu");
  const auto init_v_world_imu = config.param<Eigen::Vector3d>("odometry_estimation", "init_v_world_imu");
  this->estimate_init_state = !init_T_world_imu && !init_v_world_imu;
  this->init_T_world_imu = init_T_world_imu.value_or(Eigen::Isometry3d::Identity());
  this->init_v_world_imu = init_v_world_imu.value_or(Eigen::Vector3d::Zero());
  this->init_pose_damping_scale = config.param<double>("odometry_estimation", "init_pose_damping_scale", 1e10);

  smoother_lag = config.param<double>("odometry_estimation", "smoother_lag", 5.0);
  use_isam2_dogleg = config.param<bool>("odometry_estimation", "use_isam2_dogleg", false);
  isam2_relinearize_skip = config.param<int>("odometry_estimation", "isam2_relinearize_skip", 1);
  isam2_relinearize_thresh = config.param<double>("odometry_estimation", "isam2_relinearize_thresh", 0.1);

  clk_bias_noise  = config.param<double>("odometry_estimation", "clk_bias_noise",  100.0);
  clk_drift_noise = config.param<double>("odometry_estimation", "clk_drift_noise", 1.0);
  clock_owner_mode = config.param<std::string>("odometry_estimation", "clock_owner_mode", "dual");

  clk_bias_relin_thresh  = config.param<double>("odometry_estimation", "clk_bias_relin_thresh",  500.0);
  clk_drift_relin_thresh = config.param<double>("odometry_estimation", "clk_drift_relin_thresh", 5.0);

  validate_imu = config.param<bool>("odometry_estimation", "validate_imu", true);
  save_imu_rate_trajectory = config.param<bool>("odometry_estimation", "save_imu_rate_trajectory", false);
  sigma_p_update_scope = parse_sigma_p_update_scope(
    config.param<std::string>("odometry_estimation", "sigma_p_update_scope", "current"));

  num_threads = config.param<int>("odometry_estimation", "num_threads", 4);
  num_smoother_update_threads = 1;
}

OdometryEstimationIMUParams::~OdometryEstimationIMUParams() {}

OdometryEstimationIMU::OdometryEstimationIMU(std::unique_ptr<OdometryEstimationIMUParams>&& params_) : params(std::move(params_)) {
  marginalized_cursor = 0;
  T_lidar_imu.setIdentity();
  T_imu_lidar.setIdentity();

  if (!params->estimate_init_state || params->initialization_mode == "NAIVE") {
    auto init_estimation = new NaiveInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    if (!params->estimate_init_state) {
      init_estimation->set_init_state(params->init_T_world_imu, params->init_v_world_imu);
    }
    this->init_estimation.reset(init_estimation);
  } else if (params->initialization_mode == "LOOSE") {
    auto init_estimation = new LooseInitialStateEstimation(params->T_lidar_imu, params->imu_bias);
    this->init_estimation.reset(init_estimation);
  } else {
    logger->error("unknown initialization mode {}", params->initialization_mode);
  }

  imu_integration.reset(new IMUIntegration);
  imu_validation.reset(new IMUValidation(logger, params->validate_imu));
  deskewing.reset(new CloudDeskewing);
  covariance_estimation.reset(new CloudCovarianceEstimation(params->num_threads));

  odometry_owns_clock_ = (params->clock_owner_mode != "gnss");
  auto& lifecycle = KeyLifecycleMonitor::instance();
  lifecycle.set_expected_owner('x', "odometry");
  lifecycle.set_expected_owner('v', "odometry");
  lifecycle.set_expected_owner('b', "odometry");
  if (params->clock_owner_mode == "odometry") {
    lifecycle.set_expected_owner('c', "odometry");
  } else if (params->clock_owner_mode == "gnss") {
    lifecycle.set_expected_owner('c', "gnss");
  }

  logger->info("clock_owner_mode={} odometry_owns_clock={}", params->clock_owner_mode, odometry_owns_clock_);

  gtsam::ISAM2Params isam2_params;
  if (params->use_isam2_dogleg) {
    isam2_params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  }
  isam2_params.findUnusedFactorSlots = true;
  isam2_params.relinearizeSkip = params->isam2_relinearize_skip;

  // Per-type relinearization thresholds (IAP-RQ-010): clock state uses loose thresholds
  // because clk_bias moves 100s of m/frame at cold-start, always triggering sync-mode
  // GPU linearization if the tight default (0.1) is used.
  // Key chars: x=Pose3(6), v=Vector3(3), b=imuBias(6), c=clk(2), e=ECEFpos(3), r=Rot3(3), l=trunk landmark Point2(2)
  {
    const double t = params->isam2_relinearize_thresh;
    RelinearizationPolicyRegistry policy_registry;
    policy_registry.register_policy('x', 6, gtsam::Vector6::Constant(t));
    policy_registry.register_policy('v', 3, gtsam::Vector3::Constant(t));
    policy_registry.register_policy('b', 6, gtsam::Vector6::Constant(t));
    policy_registry.register_policy('c', 2, (gtsam::Vector2() << params->clk_bias_relin_thresh,
                                                                params->clk_drift_relin_thresh).finished());
    policy_registry.register_policy('e', 3, gtsam::Vector3::Constant(t));  // ECEF origin (rarely moves)
    policy_registry.register_policy('r', 3, gtsam::Vector3::Constant(t));  // world→ECEF rotation (rarely moves)
    policy_registry.register_policy('l', 2, gtsam::Vector2::Constant(t));  // trunk landmark Point2

    policy_registry.validate_or_throw();
    logger->info("relinearization policy {}", policy_registry.summary());
    isam2_params.setRelinearizeThreshold(policy_registry.build_map());
  }

  smoother.reset(new FixedLagSmootherExt(params->smoother_lag, isam2_params));

#ifdef GTSAM_USE_TBB
  tbb_task_arena = std::make_shared<tbb::task_arena>(params->num_smoother_update_threads);
#endif
}

OdometryEstimationIMU::~OdometryEstimationIMU() {}

void OdometryEstimationIMU::insert_imu(const double stamp, const Eigen::Vector3d& linear_acc, const Eigen::Vector3d& angular_vel) {
  Callbacks::on_insert_imu(stamp, linear_acc, angular_vel);

  if (init_estimation) {
    init_estimation->insert_imu(stamp, linear_acc, angular_vel);
  }
  imu_integration->insert_imu(stamp, linear_acc, angular_vel);
}

EstimationFrame::ConstPtr OdometryEstimationIMU::insert_frame(const PreprocessedFrame::Ptr& raw_frame, std::vector<EstimationFrame::ConstPtr>& marginalized_frames) {
  iap::timing_csv::ScopedTimer frame_timer(raw_frame->stamp, "1.2_odom_insert_frame");
  if (raw_frame->size()) {
    logger->trace("insert_frame points={} times={} ~ {}", raw_frame->size(), raw_frame->times.front(), raw_frame->times.back());
  } else {
    logger->warn("insert_frame points={}", raw_frame->size());
  }
  Callbacks::on_insert_frame(raw_frame);

  const int current = frames.size();
  const int last = current - 1;

  // The very first frame
  if (frames.empty()) {
    EstimationFrame::ConstPtr init_state;

#ifdef GTSAM_USE_TBB
    auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
    arena->execute([&] {
#endif
      init_estimation->insert_frame(raw_frame);
      init_state = init_estimation->initial_pose();
#ifdef GTSAM_USE_TBB
    });
#endif

    if (init_state == nullptr) {
      logger->debug("waiting for initial IMU state estimation to be finished");
      return nullptr;
    }
    init_estimation.reset();

    logger->info("initial IMU state estimation result");
    logger->info("T_world_imu={}", convert_to_string(init_state->T_world_imu));
    logger->info("v_world_imu={}", convert_to_string(init_state->v_world_imu));
    logger->info("imu_bias={}", convert_to_string(init_state->imu_bias));

    // Initialize the first frame
    EstimationFrame::Ptr new_frame(new EstimationFrame);
    new_frame->id = current;
    new_frame->stamp = raw_frame->stamp;

    T_lidar_imu = init_state->T_lidar_imu;
    T_imu_lidar = T_lidar_imu.inverse();

    new_frame->T_lidar_imu = init_state->T_lidar_imu;
    new_frame->T_world_lidar = init_state->T_world_lidar;
    new_frame->T_world_imu = init_state->T_world_imu;

    new_frame->v_world_imu = init_state->v_world_imu;
    new_frame->imu_bias = init_state->imu_bias;
    new_frame->raw_frame = raw_frame;

    // Transform points into IMU frame
    std::vector<Eigen::Vector4d> points_imu(raw_frame->size());
    for (int i = 0; i < raw_frame->size(); i++) {
      points_imu[i] = T_imu_lidar * raw_frame->points[i];
    }

    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    {
      iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_covariance_estimation");
      covariance_estimation->estimate(points_imu, raw_frame->neighbors, normals, covs);
    }

    auto frame = std::make_shared<gtsam_points::PointCloudCPU>(points_imu);
    if (raw_frame->intensities.size()) {
      frame->add_intensities(raw_frame->intensities);
    }
    frame->add_covs(covs);
    frame->add_normals(normals);
    new_frame->frame = frame;
    new_frame->frame_id = FrameID::IMU;
    create_frame(new_frame);

    Callbacks::on_new_frame(new_frame);
    frames.push_back(new_frame);

    // Initialize the estimator
    gtsam::Values new_values;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

    new_stamps[X(0)] = raw_frame->stamp;
    new_stamps[V(0)] = raw_frame->stamp;
    new_stamps[B(0)] = raw_frame->stamp;
    if (odometry_owns_clock_) {
      new_stamps[C(0)] = raw_frame->stamp;  // IAP-RQ-010: clock state
    }

    new_values.insert(X(0), gtsam::Pose3(new_frame->T_world_imu.matrix()));
    new_values.insert(V(0), new_frame->v_world_imu);
    new_values.insert(B(0), gtsam::imuBias::ConstantBias(new_frame->imu_bias));
    // IAP-RQ-010: clock state [δt(m), δṫ(m/s)] initialised to zero
    if (odometry_owns_clock_) {
      new_values.insert(C(0), gtsam::Vector2(0.0, 0.0));
    }
    auto& lifecycle = KeyLifecycleMonitor::instance();
    lifecycle.record_write('x', "odometry");
    lifecycle.record_write('v', "odometry");
    lifecycle.record_write('b', "odometry");
    if (odometry_owns_clock_) {
      lifecycle.record_write('c', "odometry");
    }

    // Prior for initial IMU states
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(X(0), 6, params->init_pose_damping_scale);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), init_state->v_world_imu, gtsam::noiseModel::Isotropic::Precision(3, 1.0));
    new_factors.emplace_shared<gtsam_points::LinearDampingFactor>(B(0), 6, 1e6);
    // IAP-RQ-010: loose prior on clock (will be dominated by GNSS factors in RQ-020)
    if (odometry_owns_clock_) {
      const gtsam::Vector2 clk_noise_sigmas(params->clk_bias_noise, params->clk_drift_noise);
      new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(
        C(0), gtsam::Vector2(0.0, 0.0),
        gtsam::noiseModel::Diagonal::Sigmas(clk_noise_sigmas));
    }
    {
      iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_create_factors");
      new_factors.add(create_factors(current, nullptr, new_values));
    }

    {
      iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_smoother_update");
      update_smoother(new_factors, new_values, new_stamps);
    }
    {
      iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_update_frames");
      update_frames(current, new_factors);
    }
    KeyLifecycleMonitor::instance().maybe_log(logger, raw_frame->stamp, 400, 5.0);

    return frames.back();
  }

  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::FixedLagSmootherKeyTimestampMap new_stamps;

  const double last_stamp = frames[last]->stamp;
  const auto last_T_world_imu_ = smoother->calculateEstimate<gtsam::Pose3>(X(last));
  const auto last_T_world_imu = gtsam::Pose3(last_T_world_imu_.rotation().normalized(), last_T_world_imu_.translation());
  const auto last_v_world_imu = smoother->calculateEstimate<gtsam::Vector3>(V(last));
  const auto last_imu_bias = smoother->calculateEstimate<gtsam::imuBias::ConstantBias>(B(last));
  const gtsam::NavState last_nav_world_imu(last_T_world_imu, last_v_world_imu);

  // IMU integration between LiDAR scans (inter-scan)
  int num_imu_integrated = 0;
  int imu_read_cursor = 0;
  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_imu_integration");
    imu_read_cursor = imu_integration->integrate_imu(last_stamp, raw_frame->stamp, last_imu_bias, &num_imu_integrated);
  }
  imu_integration->erase_imu_data(imu_read_cursor);
  logger->trace("num_imu_integrated={}", num_imu_integrated);

  // IMU state prediction
  const gtsam::NavState predicted_nav_world_imu = imu_integration->integrated_measurements().predict(last_nav_world_imu, last_imu_bias);
  gtsam::Pose3 predicted_T_world_imu = predicted_nav_world_imu.pose();
  gtsam::Vector3 predicted_v_world_imu = predicted_nav_world_imu.velocity();

  // Overwrite the predicted state with the last states if no IMU data is available
  if (num_imu_integrated < 2 && last > 1) {
    const Eigen::Isometry3d T_delta = frames[last - 1]->T_lidar_imu.inverse() * frames[last]->T_lidar_imu;
    predicted_T_world_imu = gtsam::Pose3((frames[last]->T_world_imu * T_delta).matrix());
    predicted_v_world_imu = frames[last]->v_world_imu;
  }

  new_stamps[X(current)] = raw_frame->stamp;
  new_stamps[V(current)] = raw_frame->stamp;
  new_stamps[B(current)] = raw_frame->stamp;
  if (odometry_owns_clock_) {
    new_stamps[C(current)] = raw_frame->stamp;  // IAP-RQ-010: clock state
  }

  gtsam::Vector2 predicted_clk(0.0, 0.0);
  if (odometry_owns_clock_) {
    // Retrieve last clock estimate and predict forward
    const gtsam::Vector2 last_clk = smoother->calculateEstimate<gtsam::Vector2>(C(last));
    const double dt = raw_frame->stamp - last_stamp;
    // Clock model: δt_next = δt + δṫ * Δt
    predicted_clk = gtsam::Vector2(last_clk(0) + last_clk(1) * dt, last_clk(1));
  }

  new_values.insert(X(current), predicted_T_world_imu);
  new_values.insert(V(current), predicted_v_world_imu);
  new_values.insert(B(current), last_imu_bias);
  if (odometry_owns_clock_) {
    new_values.insert(C(current), predicted_clk);  // IAP-RQ-010
  }
  auto& lifecycle = KeyLifecycleMonitor::instance();
  lifecycle.record_write('x', "odometry");
  lifecycle.record_write('v', "odometry");
  lifecycle.record_write('b', "odometry");
  if (odometry_owns_clock_) {
    lifecycle.record_write('c', "odometry");
  }

  // IAP-RQ-010: loose clock prior (keep states in graph; GNSS factors will dominate in RQ-020)
  if (odometry_owns_clock_) {
    const gtsam::Vector2 clk_noise_sigmas(params->clk_bias_noise, params->clk_drift_noise);
    new_factors.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(
      C(current), predicted_clk,
      gtsam::noiseModel::Diagonal::Sigmas(clk_noise_sigmas));
  }

  // Constant IMU bias assumption
  new_factors.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(last), B(current), gtsam::imuBias::ConstantBias(), gtsam::noiseModel::Isotropic::Sigma(6, params->imu_bias_noise)));
  if (params->fix_imu_bias) {
    new_factors.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(current), gtsam::imuBias::ConstantBias(params->imu_bias), gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
  }

  // Create IMU factor
  gtsam::ImuFactor::shared_ptr imu_factor;
  if (num_imu_integrated >= 2) {
    imu_factor = gtsam::make_shared<gtsam::ImuFactor>(X(last), V(last), X(current), V(current), B(last), imu_integration->integrated_measurements());
    new_factors.add(imu_factor);
  } else {
    logger->warn("insufficient number of IMU data between LiDAR scans!! (odometry_estimation)");
    logger->warn("t_last={:.6f} t_current={:.6f} num_imu={}", last_stamp, raw_frame->stamp, num_imu_integrated);
    new_factors.add(gtsam::BetweenFactor<gtsam::Vector3>(V(last), V(current), gtsam::Vector3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1.0)));
  }

  // Motion prediction for deskewing (intra-scan)
  std::vector<double> pred_imu_times;
  std::vector<Eigen::Isometry3d> pred_imu_poses;
  imu_integration->integrate_imu(raw_frame->stamp, raw_frame->scan_end_time, predicted_nav_world_imu, last_imu_bias, pred_imu_times, pred_imu_poses);

  // Create EstimationFrame
  EstimationFrame::Ptr new_frame(new EstimationFrame);
  new_frame->id = current;
  new_frame->stamp = raw_frame->stamp;

  new_frame->T_lidar_imu = T_lidar_imu;
  new_frame->T_world_imu = Eigen::Isometry3d(predicted_T_world_imu.matrix());
  new_frame->T_world_lidar = Eigen::Isometry3d(predicted_T_world_imu.matrix()) * T_imu_lidar;
  new_frame->v_world_imu = predicted_v_world_imu;
  new_frame->imu_bias = last_imu_bias.vector();
  new_frame->raw_frame = raw_frame;

  if (params->save_imu_rate_trajectory) {
    new_frame->imu_rate_trajectory.resize(8, pred_imu_times.size());

    for (int i = 0; i < pred_imu_times.size(); i++) {
      const Eigen::Vector3d trans = pred_imu_poses[i].translation();
      const Eigen::Quaterniond quat(pred_imu_poses[i].linear());
      new_frame->imu_rate_trajectory.col(i) << pred_imu_times[i], trans, quat.x(), quat.y(), quat.z(), quat.w();
    }
  }

  // Deskew and tranform points into IMU frame
  std::vector<Eigen::Vector4d> deskewed;
  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_pointcloud_deskew");
    deskewed = deskewing->deskew(T_imu_lidar, pred_imu_times, pred_imu_poses, raw_frame->stamp, raw_frame->times, raw_frame->points);
    for (auto& pt : deskewed) {
      pt = T_imu_lidar * pt;
    }
  }

  std::vector<Eigen::Vector4d> deskewed_normals;
  std::vector<Eigen::Matrix4d> deskewed_covs;
  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_covariance_estimation");
    covariance_estimation->estimate(deskewed, raw_frame->neighbors, deskewed_normals, deskewed_covs);
  }

  auto frame = std::make_shared<gtsam_points::PointCloudCPU>(deskewed);
  if (raw_frame->intensities.size()) {
    frame->add_intensities(raw_frame->intensities);
  }
  frame->add_covs(deskewed_covs);
  frame->add_normals(deskewed_normals);
  new_frame->frame = frame;
  new_frame->frame_id = FrameID::IMU;
  create_frame(new_frame);

  Callbacks::on_new_frame(new_frame);
  frames.push_back(new_frame);

  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_create_factors");
    new_factors.add(create_factors(current, imu_factor, new_values));
  }

  // Update smoother
  Callbacks::on_smoother_update(*smoother, new_factors, new_values, new_stamps);
  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_smoother_update");
    update_smoother(new_factors, new_values, new_stamps, 1);
  }
  Callbacks::on_smoother_update_finish(*smoother);

  // Find out marginalized frames
  while (marginalized_cursor < current) {
    double span = frames[current]->stamp - frames[marginalized_cursor]->stamp;
    if (span < params->smoother_lag - 0.1) {
      break;
    }

    marginalized_frames.push_back(frames[marginalized_cursor]);
    frames[marginalized_cursor].reset();
    marginalized_cursor++;
  }
  logger->debug("|frames|={} |active|={} |marginalized|={}", frames.size(), frames.inner_size(), marginalized_frames.size());
  Callbacks::on_marginalized_frames(marginalized_frames);

  // Update frames
  {
    iap::timing_csv::ScopedTimer timer(raw_frame->stamp, "1.2_update_frames");
    update_frames(current, new_factors);
  }

  // Check if IMU prediction is good or not
  imu_validation->validate(
    Eigen::Isometry3d(last_T_world_imu.matrix()),
    last_v_world_imu,
    Eigen::Isometry3d(predicted_T_world_imu.matrix()),
    predicted_v_world_imu,
    new_frame->T_world_imu,
    new_frame->v_world_imu,
    new_frame->stamp - last_stamp);
  imu_validation->validate(new_frame->imu_bias);

  std::vector<EstimationFrame::ConstPtr> active_frames(frames.inner_begin(), frames.inner_end());
  Callbacks::on_update_new_frame(active_frames.back());
  Callbacks::on_update_frames(active_frames);
  logger->trace("frames updated");

  if (smoother->fallbackHappened()) {
    logger->warn("odometry estimation smoother fallback happened (time={})", raw_frame->stamp);
  }

  KeyLifecycleMonitor::instance().maybe_log(logger, raw_frame->stamp, 400, 5.0);

  return frames[current];
}

std::vector<EstimationFrame::ConstPtr> OdometryEstimationIMU::get_remaining_frames() {
  // Perform a few optimization iterations at the end
  // for(int i=0; i<5; i++) {
  //   smoother->update();
  // }
  // OdometryEstimationIMU::update_frames(frames.size() - 1, gtsam::NonlinearFactorGraph());

  std::vector<EstimationFrame::ConstPtr> marginalized_frames;
  for (int i = marginalized_cursor; i < frames.size(); i++) {
    marginalized_frames.push_back(frames[i]);
  }

  Callbacks::on_marginalized_frames(marginalized_frames);

  return marginalized_frames;
}

void OdometryEstimationIMU::update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) {
  logger->trace("update frames current={} marginalized_cursor={}", current, marginalized_cursor);

  static uint64_t missing_clock_count = 0;
  static uint64_t skipped_clock_read_count = 0;
  static uint64_t clock_not_ready_count = 0;

  gtsam::Values all_values;
  try {
    iap::timing_csv::ScopedTimer timer(frames[current]->stamp, "1.2_update_frames_calculate_estimate");
    all_values = smoother->calculateEstimate();
  } catch (const std::exception& e) {
    logger->error("update_frames: calculateEstimate() failed: {}", e.what());
    logger->error("current={}", current);
    logger->error("marginalized_cursor={}", marginalized_cursor);
    Callbacks::on_smoother_corruption(frames[current]->stamp);
    fallback_smoother();
    return;
  }

  {
    iap::timing_csv::ScopedTimer timer(frames[current]->stamp, "1.2_update_frames_state_copy");
    for (int i = marginalized_cursor; i < frames.size(); i++) {
      try {
        const bool has_x = all_values.exists(X(i));
        const bool has_v = all_values.exists(V(i));
        const bool has_b = all_values.exists(B(i));
        if (!has_x || !has_v || !has_b) {
          logger->warn("update_frames: missing core key at frame {} (X={} V={} B={}), skipping",
            i, has_x, has_v, has_b);
          continue;
        }

        Eigen::Isometry3d T_world_imu = Eigen::Isometry3d(all_values.at<gtsam::Pose3>(X(i)).matrix());
        Eigen::Vector3d v_world_imu = all_values.at<gtsam::Vector3>(V(i));
        Eigen::Matrix<double, 6, 1> imu_bias = all_values.at<gtsam::imuBias::ConstantBias>(B(i)).vector();

        frames[i]->T_world_imu = T_world_imu;
        frames[i]->T_world_lidar = T_world_imu * T_imu_lidar;
        frames[i]->v_world_imu = v_world_imu;
        frames[i]->imu_bias = imu_bias;

        // IAP-RQ-010: read back clock states [δt(m), δṫ(m/s)]
        // In GNSS clock-owner mode, only read the current frame's clock state to avoid
        // high-volume historical missing-key telemetry noise.
        bool should_read_clock = true;
        if (!odometry_owns_clock_ && i != current) {
          should_read_clock = false;
        }

        if (!odometry_owns_clock_ && i == current) {
          const bool clock_ready = iap::IapSharedState::instance().is_clock_ready(i);
          if (!clock_ready) {
            should_read_clock = false;
            ++clock_not_ready_count;
            if (clock_not_ready_count == 1 || clock_not_ready_count % 200 == 0) {
              logger->info("update_frames: clock not ready for current frame {}, skip read [count={}]", i, clock_not_ready_count);
            }
          }
        }

        if (should_read_clock) {
          if (all_values.exists(C(i))) {
            const auto clk = all_values.at<gtsam::Vector2>(C(i));
            frames[i]->clk_bias  = clk(0);
            frames[i]->clk_drift = clk(1);
          } else {
            if (odometry_owns_clock_) {
              KeyLifecycleMonitor::instance().record_missing('c', "odometry.update_frames.current_only");
              ++missing_clock_count;
              if (missing_clock_count == 1 || missing_clock_count % 200 == 0) {
                logger->warn("update_frames: missing current clock key C({}), keep previous clock state [count={}]", i, missing_clock_count);
              }
            } else {
              ++missing_clock_count;
              if (missing_clock_count == 1 || missing_clock_count % 1000 == 0) {
                logger->debug("update_frames: current clock key C({}) unavailable in gnss owner mode [count={}]", i, missing_clock_count);
              }
            }
          }
        } else {
          ++skipped_clock_read_count;
          if (skipped_clock_read_count == 1 || skipped_clock_read_count % 2000 == 0) {
            logger->debug("update_frames: skip historical clock read in gnss owner mode [count={}]", skipped_clock_read_count);
          }
        }

        logger->trace("state[{}]: p=({:.3f},{:.3f},{:.3f}) v=({:.3f},{:.3f},{:.3f}) clk_bias={:.4f}m clk_drift={:.4f}m/s",
          i,
          T_world_imu.translation().x(), T_world_imu.translation().y(), T_world_imu.translation().z(),
          v_world_imu.x(), v_world_imu.y(), v_world_imu.z(),
          frames[i]->clk_bias, frames[i]->clk_drift);
      } catch (const std::exception& e) {
        logger->error("update_frames: frame {} extraction failed: {}", i, e.what());
        continue;
      }
    }
  }

  const auto update_sigma_p = [&](int i) {
    if (i < marginalized_cursor || i >= frames.size() || !frames[i]) {
      return;
    }
    if (!all_values.exists(X(i))) {
      logger->warn("update_frames: missing X({}) for sigma_p update", i);
      return;
    }

    // IAP-RQ-015: extract position covariance Sigma_p from smoother marginal.
    try {
      const gtsam::Matrix pose_cov = smoother->marginalCovariance(X(i));  // 6x6 [rot|trans]
      frames[i]->sigma_p = pose_cov.block<3, 3>(3, 3);  // translation block
      const double trace_sigma_p = frames[i]->sigma_p.trace();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(frames[i]->sigma_p, Eigen::EigenvaluesOnly);
      const double lambda_max = eig.eigenvalues().maxCoeff();
      logger->trace("sigma_p [{}]: trace={:.6f} lambda_max={:.6f} (PL_proxy={:.4f}m)",
        i, trace_sigma_p, lambda_max, std::sqrt(std::max(0.0, lambda_max)));
    } catch (const std::exception& e) {
      logger->warn("marginalCovariance(X({})) failed: {}", i, e.what());
    }
  };

  {
    iap::timing_csv::ScopedTimer timer(frames[current]->stamp, "1.2_update_frames_sigma_p");
    if (params->sigma_p_update_scope == OdometryEstimationIMUParams::SigmaPUpdateScope::ALL_ACTIVE) {
      for (int i = marginalized_cursor; i < frames.size(); i++) {
        update_sigma_p(i);
      }
    } else {
      update_sigma_p(current);
    }
  }
}

void OdometryEstimationIMU::update_smoother(
  const gtsam::NonlinearFactorGraph& new_factors,
  const gtsam::Values& new_values,
  const std::map<std::uint64_t, double>& new_stamp,
  int update_count) {
#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(tbb_task_arena.get());
  arena->execute([&] {
#endif
    smoother->update(new_factors, new_values, new_stamp);
    for (int i = 0; i < update_count; i++) {
      smoother->update();
    }
#ifdef GTSAM_USE_TBB
  });
#endif
}

void OdometryEstimationIMU::update_smoother(int count) {
  if (count <= 0) {
    return;
  }

  update_smoother(gtsam::NonlinearFactorGraph(), gtsam::Values(), std::map<std::uint64_t, double>(), count - 1);
}

}  // namespace glim
