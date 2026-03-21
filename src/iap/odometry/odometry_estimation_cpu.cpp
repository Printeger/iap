#include <iap/odometry/odometry_estimation_cpu.hpp>

#include <Eigen/SVD>  // IAP-RQ-040: condition number for ICP degeneracy
#include <spdlog/spdlog.h>

#include <cerrno>
#include <cstring>
#include <filesystem>

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/factors/linear_damping_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <iap/util/config.hpp>
#include <iap/common/imu_integration.hpp>
#include <iap/common/cloud_deskewing.hpp>
#include <iap/common/cloud_covariance_estimation.hpp>

#include <iap/odometry/callbacks.hpp>

#ifdef GTSAM_USE_TBB
#include <tbb/task_arena.h>
#endif

namespace glim {

using Callbacks = OdometryEstimationCallbacks;

using gtsam::symbol_shorthand::B;  // IMU bias
using gtsam::symbol_shorthand::V;  // IMU velocity   (v_world_imu)
using gtsam::symbol_shorthand::X;  // IMU pose       (T_world_imu)

OdometryEstimationCPUParams::OdometryEstimationCPUParams() : OdometryEstimationIMUParams() {
  // odometry config
  Config config(GlobalConfig::get_config_path("config_odometry"));

  registration_type = config.param<std::string>("odometry_estimation", "registration_type", "VGICP");
  max_iterations = config.param<int>("odometry_estimation", "max_iterations", 5);
  lru_thresh = config.param<int>("odometry_estimation", "lru_thresh", 100);
  target_downsampling_rate = config.param<double>("odometry_estimation", "target_downsampling_rate", 0.1);

  ivox_resolution = config.param<double>("odometry_estimation", "ivox_resolution", 0.5);
  ivox_min_dist = config.param<double>("odometry_estimation", "ivox_min_dist", 0.1);

  vgicp_resolution = config.param<double>("odometry_estimation", "vgicp_resolution", 0.2);
  vgicp_voxelmap_levels = config.param<int>("odometry_estimation", "vgicp_voxelmap_levels", 2);
  vgicp_voxelmap_scaling_factor = config.param<double>("odometry_estimation", "vgicp_voxelmap_scaling_factor", 2.0);

  // IAP-RQ-040: ICP quality / health
  icp_cond_threshold = config.param<double>("odometry_estimation", "icp_cond_threshold", 500.0);
  gamma_lidar_max    = config.param<double>("odometry_estimation", "gamma_lidar_max",    10.0);
  enable_icp_csv     = config.param<bool>("odometry_estimation", "enable_icp_csv", false);
  icp_csv_path       = config.param<std::string>("odometry_estimation", "icp_csv_path", "/tmp/iap_icp.csv");
  spdlog::info("[odometry_cpu] enable_icp_csv={} icp_csv_path={}", enable_icp_csv, icp_csv_path);

  // Scan-to-multi-scan (GLIO2-style)
  use_scan_to_map             = config.param<bool>("odometry_estimation", "use_scan_to_map", false);
  full_connection_window_size = config.param<int>("odometry_estimation", "full_connection_window_size", 3);
  max_num_keyframes           = config.param<int>("odometry_estimation", "max_num_keyframes", 15);
  keyframe_update_strategy    = config.param<std::string>("odometry_estimation", "keyframe_update_strategy", "OVERLAP");
  keyframe_max_overlap        = config.param<double>("odometry_estimation", "keyframe_max_overlap", 0.7);
  keyframe_min_overlap        = config.param<double>("odometry_estimation", "keyframe_min_overlap", 0.01);
  keyframe_delta_trans        = config.param<double>("odometry_estimation", "keyframe_delta_trans", 2.0);
  keyframe_delta_rot          = config.param<double>("odometry_estimation", "keyframe_delta_rot", 0.5);
}

OdometryEstimationCPUParams::~OdometryEstimationCPUParams() {}

OdometryEstimationCPU::OdometryEstimationCPU(const OdometryEstimationCPUParams& params) : OdometryEstimationIMU(std::make_unique<OdometryEstimationCPUParams>(params)) {
  last_T_target_imu.setIdentity();
  if (!params.use_scan_to_map && params.registration_type != "VGICP") {
    spdlog::warn("scan-to-multi-scan mode requires registration_type=VGICP (got {}). Set use_scan_to_map=true or switch to VGICP.", params.registration_type);
  }
  if (params.registration_type == "GICP") {
    target_ivox.reset(new gtsam_points::iVox(params.ivox_resolution));
    target_ivox->voxel_insertion_setting().set_min_dist_in_cell(params.ivox_min_dist);
    target_ivox->set_lru_horizon(params.lru_thresh);
    target_ivox->set_neighbor_voxel_mode(1);
  } else if (params.registration_type == "VGICP") {
    target_voxelmaps.resize(params.vgicp_voxelmap_levels);
    for (int i = 0; i < params.vgicp_voxelmap_levels; i++) {
      const double resolution = params.vgicp_resolution * std::pow(params.vgicp_voxelmap_scaling_factor, i);
      target_voxelmaps[i] = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
      target_voxelmaps[i]->set_lru_horizon(params.lru_thresh);
    }
  } else {
    spdlog::error("unknown registration type for odometry_estimation_cpu ({})", params.registration_type);
    abort();
  }
}

OdometryEstimationCPU::~OdometryEstimationCPU() {}

gtsam::NonlinearFactorGraph OdometryEstimationCPU::create_factors(const int current, const gtsam_points::shared_ptr<gtsam::ImuFactor>& imu_factor, gtsam::Values& new_values) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  const int last = current - 1;

  if (current == 0) {
    last_T_target_imu = frames[current]->T_world_imu;
    if (params->use_scan_to_map) {
      update_target(current, frames[current]->T_world_imu);
    } else if (params->registration_type == "VGICP") {
      // Create per-frame voxelmaps for the first frame so it can become a keyframe
      frames[current]->voxelmaps.resize(params->vgicp_voxelmap_levels);
      for (int i = 0; i < params->vgicp_voxelmap_levels; i++) {
        const double resolution = params->vgicp_resolution * std::pow(params->vgicp_voxelmap_scaling_factor, i);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*frames[current]->frame);
        frames[current]->voxelmaps[i] = voxelmap;
      }
      update_keyframes(current);
    }
    return gtsam::NonlinearFactorGraph();
  }

  // ============================================================
  // Mode 1: Scan-to-multi-scan (GLIO2-style) — direct binary factors
  // ============================================================
  if (!params->use_scan_to_map) {
    // ① Create per-frame voxelmaps for the current frame (VGICP only)
    if (params->registration_type == "VGICP" && frames[current]->voxelmaps.empty()) {
      frames[current]->voxelmaps.resize(params->vgicp_voxelmap_levels);
      for (int i = 0; i < params->vgicp_voxelmap_levels; i++) {
        const double resolution = params->vgicp_resolution * std::pow(params->vgicp_voxelmap_scaling_factor, i);
        auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);
        voxelmap->insert(*frames[current]->frame);
        frames[current]->voxelmaps[i] = voxelmap;
      }
    }

    gtsam::NonlinearFactorGraph all_matching_factors;
    gtsam::Values pred_values;
    pred_values.insert(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

    // ② Lambda: binary factor — both poses variable in smoother
    const auto create_binary_factor = [&](gtsam::Key target_key, gtsam::Key source_key, const EstimationFrame::ConstPtr& target_frame) {
      for (const auto& voxelmap : target_frame->voxelmaps) {
        auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(target_key, source_key, voxelmap, frames[current]->frame);
        f->set_num_threads(params->num_threads);
        all_matching_factors.add(f);
      }
    };

    // ③ Lambda: unary factor — target pose is fixed (out of smoother window)
    const auto create_unary_factor = [&](const gtsam::Pose3& fixed_pose, gtsam::Key source_key, const EstimationFrame::ConstPtr& target_frame) {
      for (const auto& voxelmap : target_frame->voxelmaps) {
        auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(fixed_pose, source_key, voxelmap, frames[current]->frame);
        f->set_num_threads(params->num_threads);
        all_matching_factors.add(f);
      }
    };

    // ④ Full connection window: binary factors to recent frames
    for (int target = std::max(0, current - params->full_connection_window_size); target < current; target++) {
      create_binary_factor(X(target), X(current), frames[target]);
      if (!pred_values.exists(X(target))) {
        pred_values.insert(X(target), gtsam::Pose3(frames[target]->T_world_imu.matrix()));
      }
    }

    // ⑤ Keyframe connections
    for (const auto& keyframe : keyframes) {
      if (keyframe->id >= current - params->full_connection_window_size) {
        // Already covered by full connection window
        continue;
      }
      const double span = frames[current]->stamp - keyframe->stamp;
      if (span > params->smoother_lag - 0.1 || !frames[keyframe->id]) {
        // Keyframe is outside the smoother lag: use fixed pose (unary factor)
        const gtsam::Pose3 fixed_pose(keyframe->T_world_imu.matrix());
        create_unary_factor(fixed_pose, X(current), keyframe);
      } else {
        // Keyframe still in smoother window: binary factor
        create_binary_factor(X(keyframe->id), X(current), keyframe);
        if (!pred_values.exists(X(keyframe->id))) {
          pred_values.insert(X(keyframe->id), gtsam::Pose3(keyframe->T_world_imu.matrix()));
        }
      }
    }

    // ⑥ IAP-RQ-040: ICP quality assessment at predicted pose
    double cond_number = 1.0;
    int    inlier_count = 0;
    double inlier_fraction = 0.0;
    double rmse = 0.0;

    if (!all_matching_factors.empty()) {
      auto gfg = all_matching_factors.linearize(pred_values);

      // Condition number from Hessian diagonal block for X(current)
      {
        const auto H_blocks = gfg->hessianBlockDiagonal();
        const auto it = H_blocks.find(X(current));
        if (it != H_blocks.end()) {
          Eigen::JacobiSVD<Eigen::MatrixXd> svd(it->second, Eigen::ComputeThinU | Eigen::ComputeThinV);
          const auto& sv = svd.singularValues();
          const double sv_min = sv(sv.size() - 1);
          const double sv_max = sv(0);
          cond_number = (sv_min > 1e-10) ? sv_max / sv_min : 1e9;
        }
      }

      // Inlier count / fraction (populated after linearize)
      for (const auto& f : all_matching_factors) {
        if (auto* vgicp = dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(f.get())) {
          inlier_fraction += vgicp->inlier_fraction();
          inlier_count    += vgicp->num_inliers();
        }
      }
      if (!all_matching_factors.empty()) {
        inlier_fraction /= static_cast<double>(all_matching_factors.size());
      }

      const double total_error = all_matching_factors.error(pred_values);
      rmse = std::sqrt(total_error / std::max(inlier_count, 1));
    }

    const double cond_threshold = params->icp_cond_threshold;
    const bool   degeneracy_flag = (cond_number > cond_threshold);
    const double gamma_lidar = degeneracy_flag
      ? std::min(std::sqrt(cond_number / cond_threshold), params->gamma_lidar_max)
      : 1.0;

    auto& q          = frames[current]->icp_quality;
    q.inlier_count   = inlier_count;
    q.inlier_fraction= inlier_fraction;
    q.rmse           = rmse;
    q.cond_number    = cond_number;
    q.degeneracy_flag= degeneracy_flag;
    q.gamma_lidar    = gamma_lidar;

    logger->trace(
      "icp_quality[{}]: inliers={} ({:.1f}%) rmse={:.4f} cond={:.1f} degenerate={} gamma={:.2f}",
      current, inlier_count, inlier_fraction * 100.0, rmse,
      cond_number, degeneracy_flag, gamma_lidar);

    // IAP-RQ-040: write ICP quality CSV row
    if (params->enable_icp_csv) {
      const std::filesystem::path csv_path(params->icp_csv_path);
      if (csv_path.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(csv_path.parent_path(), ec);
        if (ec) {
          logger->warn("failed to create ICP CSV parent directory '{}': {}", csv_path.parent_path().string(), ec.message());
        }
      }

      static bool icp_csv_header_written = false;
      FILE* f = std::fopen(params->icp_csv_path.c_str(), icp_csv_header_written ? "a" : "w");
      if (f) {
        if (!icp_csv_header_written) {
          std::fprintf(f, "stamp,frame_id,rmse,inlier_fraction,condition_number,gamma_lidar,drop_flag\n");
          icp_csv_header_written = true;
        }
        std::fprintf(f, "%.6f,%ld,%.4f,%.4f,%.1f,%.4f,%d\n",
                     frames[current]->stamp, static_cast<long>(current),
                     rmse, inlier_fraction, cond_number, gamma_lidar,
                     degeneracy_flag ? 1 : 0);
        std::fclose(f);
      } else {
        logger->warn("failed to open ICP CSV '{}': {}", params->icp_csv_path, std::strerror(errno));
      }
    }

    // Soft prior at predicted pose: precision weakens with degeneracy (RQ-040)
    // When gamma=1 (healthy): strong prior; when gamma>1 (degenerate): prior relaxes,
    // letting GNSS factors dominate.
    const double lidar_precision = 1e3 / (gamma_lidar * gamma_lidar);
    all_matching_factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      X(current),
      gtsam::Pose3(frames[current]->T_world_imu.matrix()),
      gtsam::noiseModel::Isotropic::Precision(6, lidar_precision));

    // ⑦ Update keyframe list for future frames
    update_keyframes(current);

    return all_matching_factors;
  }

  // ============================================================
  // Mode 2: Scan-to-rolling-accumulator (legacy, use_scan_to_map=true)
  // ============================================================

  const Eigen::Isometry3d pred_T_last_current = frames[last]->T_world_imu.inverse() * frames[current]->T_world_imu;
  const Eigen::Isometry3d pred_T_target_imu = last_T_target_imu * pred_T_last_current;

  gtsam::Values values;
  values.insert(X(current), gtsam::Pose3(pred_T_target_imu.matrix()));

  // Create frame-to-model matching factor
  gtsam::NonlinearFactorGraph matching_cost_factors;
  if (params->registration_type == "GICP") {
    auto gicp_factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>>(
      gtsam::Pose3(),
      X(current),
      target_ivox,
      frames[current]->frame,
      target_ivox);
    gicp_factor->set_max_correspondence_distance(params->ivox_resolution * 2.0);
    gicp_factor->set_num_threads(params->num_threads);
    matching_cost_factors.add(gicp_factor);
  } else if (params->registration_type == "VGICP") {
    for (const auto& voxelmap : target_voxelmaps) {
      auto vgicp_factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(gtsam::Pose3(), X(current), voxelmap, frames[current]->frame);
      vgicp_factor->set_num_threads(params->num_threads);
      matching_cost_factors.add(vgicp_factor);
    }
  }

  gtsam::NonlinearFactorGraph graph;
  graph.add(matching_cost_factors);

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.setMaxIterations(params->max_iterations);
  lm_params.setAbsoluteErrorTol(0.1);

  gtsam::Pose3 last_estimate = values.at<gtsam::Pose3>(X(current));
  lm_params.termination_criteria = [&](const gtsam::Values& values) {
    const gtsam::Pose3 current_pose = values.at<gtsam::Pose3>(X(current));
    const gtsam::Pose3 delta = last_estimate.inverse() * current_pose;

    const double delta_t = delta.translation().norm();
    const double delta_r = Eigen::AngleAxisd(delta.rotation().matrix()).angle();
    last_estimate = current_pose;

    if (delta_t < 1e-10 && delta_r < 1e-10) {
      // Maybe failed to solve the linear system
      return false;
    }

    // Convergence check
    return delta_t < 1e-3 && delta_r < 1e-3 * M_PI / 180.0;
  };

  // Optimize
  // lm_params.setDiagonalDamping(true);
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, values, lm_params);

#ifdef GTSAM_USE_TBB
  auto arena = static_cast<tbb::task_arena*>(this->tbb_task_arena.get());
  arena->execute([&] {
#endif
    values = optimizer.optimize();
#ifdef GTSAM_USE_TBB
  });
#endif

  const Eigen::Isometry3d T_target_imu = Eigen::Isometry3d(values.at<gtsam::Pose3>(X(current)).matrix());
  Eigen::Isometry3d T_last_current = last_T_target_imu.inverse() * T_target_imu;
  T_last_current.linear() = Eigen::Quaterniond(T_last_current.linear()).normalized().toRotationMatrix();
  frames[current]->T_world_imu = frames[last]->T_world_imu * T_last_current;
  new_values.insert_or_assign(X(current), gtsam::Pose3(frames[current]->T_world_imu.matrix()));

  // --- IAP-RQ-040: ICP quality report + noise inflation --------------------
  // Re-linearize at optimal pose to populate correspondences & Hessian
  auto gfg = matching_cost_factors.linearize(values);

  // Condition number of the 6×6 Hessian diagonal block for pose X(current)
  double cond_number = 1.0;
  {
    const auto H_blocks = gfg->hessianBlockDiagonal();
    const auto it = H_blocks.find(X(current));
    if (it != H_blocks.end()) {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(it->second, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const auto& sv  = svd.singularValues();
      const double sv_min = sv(sv.size() - 1);
      const double sv_max = sv(0);
      cond_number = (sv_min > 1e-10) ? sv_max / sv_min : 1e9;
    }
  }

  // Inlier count / fraction — cast to concrete factor type
  int    inlier_count    = 0;
  double inlier_fraction = 0.0;
  for (const auto& f : matching_cost_factors) {
    if (auto* gicp = dynamic_cast<gtsam_points::IntegratedGICPFactor_<gtsam_points::iVox, gtsam_points::PointCloud>*>(f.get())) {
      const double frac = gicp->inlier_fraction();
      inlier_fraction += frac;
      inlier_count    += static_cast<int>(frac * frames[current]->frame->size());
    }
    if (auto* vgicp = dynamic_cast<gtsam_points::IntegratedVGICPFactor*>(f.get())) {
      inlier_fraction += vgicp->inlier_fraction();
      inlier_count    += vgicp->num_inliers();
    }
  }
  if (!matching_cost_factors.empty()) {
    inlier_fraction /= static_cast<double>(matching_cost_factors.size());
  }

  // RMSE proxy: sqrt(total_error / inlier_count)
  const double total_error = matching_cost_factors.error(values);
  const double rmse        = std::sqrt(total_error / std::max(inlier_count, 1));

  // Noise inflation: gamma_lidar = clamp(sqrt(cond/thresh), 1, max)
  const double cond_threshold = params->icp_cond_threshold;
  const bool   degeneracy_flag = (cond_number > cond_threshold);
  const double gamma_lidar = degeneracy_flag
    ? std::min(std::sqrt(cond_number / cond_threshold), params->gamma_lidar_max)
    : 1.0;

  // Populate quality report in the frame
  auto& q          = frames[current]->icp_quality;
  q.inlier_count   = inlier_count;
  q.inlier_fraction= inlier_fraction;
  q.rmse           = rmse;
  q.cond_number    = cond_number;
  q.degeneracy_flag= degeneracy_flag;
  q.gamma_lidar    = gamma_lidar;

  logger->trace(
    "icp_quality[{}]: inliers={} ({:.1f}%) rmse={:.4f} cond={:.1f} degenerate={} gamma={:.2f}",
    current, inlier_count, inlier_fraction * 100.0, rmse,
    cond_number, degeneracy_flag, gamma_lidar);

  // IAP-RQ-040: write ICP quality CSV row (scan-to-multi-scan path)
  if (params->enable_icp_csv) {
    const std::filesystem::path csv_path(params->icp_csv_path);
    if (csv_path.has_parent_path()) {
      std::error_code ec;
      std::filesystem::create_directories(csv_path.parent_path(), ec);
      if (ec) {
        logger->warn("failed to create ICP CSV parent directory '{}': {}", csv_path.parent_path().string(), ec.message());
      }
    }

    static bool icp_csv_header_written = false;
    FILE* f = std::fopen(params->icp_csv_path.c_str(), icp_csv_header_written ? "a" : "w");
    if (f) {
      if (!icp_csv_header_written) {
        std::fprintf(f, "stamp,frame_id,rmse,inlier_fraction,condition_number,gamma_lidar,drop_flag\n");
        icp_csv_header_written = true;
      }
      std::fprintf(f, "%.6f,%ld,%.4f,%.4f,%.1f,%.4f,%d\n",
                   frames[current]->stamp, static_cast<long>(current),
                   rmse, inlier_fraction, cond_number, gamma_lidar,
                   degeneracy_flag ? 1 : 0);
      std::fclose(f);
    } else {
      logger->warn("failed to open ICP CSV '{}': {}", params->icp_csv_path, std::strerror(errno));
    }
  }
  // -------------------------------------------------------------------------

  gtsam::NonlinearFactorGraph factors;

  // LiDAR factor noise — inflated by gamma_lidar when degenerate (RQ-040)
  // base precision = 1e3  →  base sigma = 1/sqrt(1e3) ≈ 0.032 m
  // inflated sigma = base_sigma * gamma_lidar  →  inflated precision = base_precision / gamma²
  const double lidar_precision = 1e3 / (gamma_lidar * gamma_lidar);
  factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(last), X(current), gtsam::Pose3(T_last_current.matrix()), gtsam::noiseModel::Isotropic::Precision(6, lidar_precision));
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(current), gtsam::Pose3(T_target_imu.matrix()), gtsam::noiseModel::Isotropic::Precision(6, lidar_precision));

  update_target(current, T_target_imu);
  last_T_target_imu = T_target_imu;

  return factors;
}

void OdometryEstimationCPU::fallback_smoother() {}

void OdometryEstimationCPU::update_target(const int current, const Eigen::Isometry3d& T_target_imu) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  auto frame = frames[current]->frame;
  if (current >= 5) {
    frame = gtsam_points::random_sampling(frames[current]->frame, params->target_downsampling_rate, mt);
  }

  auto transformed = gtsam_points::transform(frame, T_target_imu);
  if (params->registration_type == "GICP") {
    target_ivox->insert(*transformed);
  } else if (params->registration_type == "VGICP") {
    for (auto& target_voxelmap : target_voxelmaps) {
      target_voxelmap->insert(*transformed);
    }
  }

  // Update target point cloud (just for visualization).
  // This is not necessary for mapping and can safely be removed.
  if (frames.size() % 50 == 0) {
    EstimationFrame::Ptr frame(new EstimationFrame);
    frame->id = frames.size() - 1;
    frame->stamp = frames.back()->stamp;

    frame->T_lidar_imu = frames.back()->T_lidar_imu;
    frame->T_world_lidar = frame->T_lidar_imu.inverse();
    frame->T_world_imu.setIdentity();

    frame->v_world_imu.setZero();
    frame->imu_bias.setZero();

    frame->frame_id = FrameID::IMU;

    if (params->registration_type == "GICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_ivox->voxel_points());
    } else if (params->registration_type == "VGICP") {
      frame->frame = std::make_shared<gtsam_points::PointCloudCPU>(target_voxelmaps[0]->voxel_points());
    }

    std::vector<EstimationFrame::ConstPtr> keyframes = {frame};
    Callbacks::on_update_keyframes(keyframes);

    if (target_ivox_frame) {
      std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {target_ivox_frame};
      Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    }

    target_ivox_frame = frame;
  }
}

void OdometryEstimationCPU::update_keyframes(int current) {
  if (keyframes.empty()) {
    keyframes.push_back(frames[current]);
    return;
  }
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());
  if (params->keyframe_update_strategy == "OVERLAP") {
    update_keyframes_overlap(current);
  } else {
    update_keyframes_displacement(current);
  }
  Callbacks::on_update_keyframes(keyframes);
}

void OdometryEstimationCPU::update_keyframes_overlap(int current) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());

  if (!frames[current]->frame->size()) {
    return;
  }

  // Compute overlap of current frame against all keyframes
  std::vector<double> overlaps;
  std::vector<Eigen::Isometry3d> deltas;
  for (const auto& keyframe : keyframes) {
    deltas.push_back(keyframe->T_world_imu.inverse() * frames[current]->T_world_imu);
  }

  // Use real voxelmap overlap when available; fall back to geometric estimate
  if (!keyframes.empty() && !keyframes[0]->voxelmaps.empty()) {
    std::vector<gtsam_points::GaussianVoxelMap::ConstPtr> keyframe_voxelmaps;
    for (const auto& keyframe : keyframes) {
      if (!keyframe->voxelmaps.empty() && keyframe->voxelmaps.back()) {
        keyframe_voxelmaps.push_back(keyframe->voxelmaps.back());
      }
    }
    if (!keyframe_voxelmaps.empty()) {
      overlaps.push_back(gtsam_points::overlap(keyframe_voxelmaps, frames[current]->frame, deltas));
    }
  }
  if (overlaps.empty()) {
    for (const auto& delta : deltas) {
      const double dist  = delta.translation().norm();
      const double angle = Eigen::AngleAxisd(delta.linear()).angle();
      overlaps.push_back(std::exp(-dist / 5.0) * std::exp(-angle / 0.5));
    }
  }

  const double max_overlap = *std::max_element(overlaps.begin(), overlaps.end());
  if (max_overlap > params->keyframe_max_overlap) {
    return;  // Current frame overlaps sufficiently with existing keyframes — no new keyframe
  }

  // Add current frame as a new keyframe
  keyframes.push_back(frames[current]);

  if (static_cast<int>(keyframes.size()) <= params->max_num_keyframes) {
    return;
  }

  // Need to evict keyframes — first remove those with low overlap to the new keyframe
  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes;
  const auto& new_keyframe = frames[current];
  for (int i = 0; i < static_cast<int>(keyframes.size()) - 1; i++) {
    const Eigen::Isometry3d delta = keyframes[i]->T_world_imu.inverse() * new_keyframe->T_world_imu;
    double overlap_val = 0.0;
    if (!keyframes[i]->voxelmaps.empty() && keyframes[i]->voxelmaps.back()) {
      overlap_val = gtsam_points::overlap(keyframes[i]->voxelmaps.back(), new_keyframe->frame, delta);
    } else {
      const double dist  = delta.translation().norm();
      const double angle = Eigen::AngleAxisd(delta.linear()).angle();
      overlap_val = std::exp(-dist / 5.0) * std::exp(-angle / 0.5);
    }
    if (overlap_val < params->keyframe_min_overlap) {
      marginalized_keyframes.push_back(keyframes[i]);
      keyframes.erase(keyframes.begin() + i);
      i--;
    }
  }

  if (static_cast<int>(keyframes.size()) <= params->max_num_keyframes) {
    Callbacks::on_marginalized_keyframes(marginalized_keyframes);
    return;
  }

  // Still over limit — remove the oldest keyframe
  marginalized_keyframes.push_back(keyframes.front());
  keyframes.erase(keyframes.begin());
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

void OdometryEstimationCPU::update_keyframes_displacement(int current) {
  const auto params = static_cast<const OdometryEstimationCPUParams*>(this->params.get());

  const Eigen::Isometry3d delta = keyframes.back()->T_world_imu.inverse() * frames[current]->T_world_imu;
  const double delta_trans = delta.translation().norm();
  const double delta_rot   = Eigen::AngleAxisd(delta.linear()).angle();

  if (delta_trans < params->keyframe_delta_trans && delta_rot < params->keyframe_delta_rot) {
    return;
  }

  keyframes.push_back(frames[current]);

  if (static_cast<int>(keyframes.size()) <= params->max_num_keyframes) {
    return;
  }

  // Remove oldest keyframe
  std::vector<EstimationFrame::ConstPtr> marginalized_keyframes = {keyframes.front()};
  keyframes.erase(keyframes.begin());
  Callbacks::on_marginalized_keyframes(marginalized_keyframes);
}

}  // namespace glim
