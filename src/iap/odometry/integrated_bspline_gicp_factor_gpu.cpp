#include <iap/odometry/integrated_bspline_gicp_factor_gpu.hpp>

#ifdef GTSAM_POINTS_USE_CUDA

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>

namespace iap {

namespace frame = gtsam_points::frame;

namespace {

gtsam::Key bucket_pose_key(std::size_t index) {
  return gtsam::symbol('q', static_cast<uint64_t>(index));
}

std::vector<gtsam::Matrix> hessian_upper_blocks(const Eigen::MatrixXd& hessian) {
  constexpr int kBlockDim = 6;
  std::vector<gtsam::Matrix> blocks;
  blocks.reserve(kBSplineControlPointCount * (kBSplineControlPointCount + 1) / 2);

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    for (std::size_t j = i; j < kBSplineControlPointCount; ++j) {
      blocks.emplace_back(hessian.block<kBlockDim, kBlockDim>(kBlockDim * i, kBlockDim * j));
    }
  }

  return blocks;
}

std::vector<gtsam::Vector> linear_term_blocks(const Eigen::VectorXd& linear_term) {
  constexpr int kBlockDim = 6;
  std::vector<gtsam::Vector> blocks(kBSplineControlPointCount);
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    blocks[i] = linear_term.segment<kBlockDim>(kBlockDim * i);
  }
  return blocks;
}

}  // namespace

IntegratedBSplineGICPFactorGPU::IntegratedBSplineGICPFactorGPU(
  const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
  const std::shared_ptr<const gtsam_points::iVox>& target,
  const std::shared_ptr<const gtsam_points::PointCloud>& source,
  CUstream_st* stream,
  gtsam_points::TempBufferManager::Ptr temp_buffer)
: gtsam::NonlinearFactor(gtsam::KeyVector(keys.begin(), keys.end())),
  stream_(stream),
  temp_buffer_(std::move(temp_buffer)),
  target_(target),
  source_(source) {
  if (!target_) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPU requires a target iVox snapshot");
  }
  if (!source_ || !frame::has_points(*source_) || !frame::has_covs(*source_) || !frame::has_times(*source_)) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPU requires source points, covariances, and times");
  }

  target_cpu_points_ = target_->voxel_data();
  target_point_count_ = target_cpu_points_ ? static_cast<std::size_t>(target_cpu_points_->size()) : 0U;
  target_gpu_ = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(
    static_cast<float>(target_->leaf_size()),
    8192 * 2,
    10,
    1e-3,
    stream_);
  if (target_cpu_points_ && target_cpu_points_->size()) {
    auto target_points_gpu = gtsam_points::PointCloudGPU::clone(*target_cpu_points_, stream_);
    target_gpu_->insert(*target_points_gpu);
  }

  const double time_eps = 1e-3;
  time_table_.reserve(frame::size(*source_) / 10 + 1);
  time_indices_.reserve(frame::size(*source_));
  for (int i = 0; i < frame::size(*source_); ++i) {
    const double t = frame::time(*source_, i);
    if (time_table_.empty() || t - time_table_.back() > time_eps) {
      time_table_.push_back(t);
    }
    time_indices_.push_back(static_cast<int>(time_table_.size() - 1));
  }

  time_bucket_populations_.assign(time_table_.size(), 0U);
  for (const int time_index : time_indices_) {
    time_bucket_populations_[static_cast<std::size_t>(time_index)]++;
  }

  const double time_min = time_table_.empty() ? 0.0 : time_table_.front();
  const double time_max = time_table_.empty() ? 1.0 : time_table_.back();
  const double denom = std::max(1e-9, time_max - time_min);
  for (auto& t : time_table_) {
    t = (t - time_min) / denom;
  }

  build_bucket_factors();
}

void IntegratedBSplineGICPFactorGPU::set_numeric_eps(double eps) {
  numeric_eps_ = std::max(1e-8, eps);
}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineGICPFactorGPU::control_poses(
  const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

gtsam::Values IntegratedBSplineGICPFactorGPU::control_pose_values() const {
  gtsam::Values values;
  if (!last_control_poses_valid_) {
    return values;
  }

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    values.insert(keys_[i], last_control_poses_[i]);
  }
  return values;
}

void IntegratedBSplineGICPFactorGPU::build_bucket_factors() {
  bucket_factors_.clear();
  bucket_graph_.resize(0);

  if (time_table_.empty()) {
    return;
  }

  std::vector<std::vector<int>> bucket_point_indices(time_table_.size());
  for (int i = 0; i < frame::size(*source_); ++i) {
    bucket_point_indices[static_cast<std::size_t>(time_indices_[static_cast<std::size_t>(i)])].push_back(i);
  }

  for (std::size_t i = 0; i < bucket_point_indices.size(); ++i) {
    const auto& point_indices = bucket_point_indices[i];
    if (point_indices.empty()) {
      continue;
    }

    std::vector<Eigen::Vector4d> bucket_points;
    bucket_points.reserve(point_indices.size());
    std::vector<Eigen::Matrix4d> bucket_covs;
    bucket_covs.reserve(point_indices.size());
    std::vector<Eigen::Vector4d> bucket_normals;
    if (frame::has_normals(*source_)) {
      bucket_normals.reserve(point_indices.size());
    }

    for (const int point_index : point_indices) {
      bucket_points.emplace_back(frame::point(*source_, point_index));
      bucket_covs.emplace_back(frame::cov(*source_, point_index));
      if (frame::has_normals(*source_)) {
        bucket_normals.emplace_back(frame::normal(*source_, point_index));
      }
    }

    auto bucket_gpu = std::make_shared<gtsam_points::PointCloudGPU>();
    bucket_gpu->add_points(bucket_points, stream_);
    bucket_gpu->add_covs(bucket_covs, stream_);
    if (!bucket_normals.empty()) {
      bucket_gpu->add_normals(bucket_normals, stream_);
    }

    BucketFactor bucket;
    bucket.stamp = time_table_[i];
    bucket.u = time_table_[i];
    bucket.key = bucket_pose_key(i);
    bucket.point_count = point_indices.size();
    bucket.source_gpu = bucket_gpu;
    bucket.factor = std::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(
      gtsam::Pose3(),
      bucket.key,
      target_gpu_,
      bucket.source_gpu,
      stream_,
      temp_buffer_);
    bucket.factor->set_enable_surface_validation(frame::has_normals(*source_));
    bucket_graph_.add(bucket.factor);
    bucket_factors_.push_back(std::move(bucket));
  }
}

std::vector<IntegratedBSplineGICPFactorGPU::PoseJacobianArray> IntegratedBSplineGICPFactorGPU::compute_bucket_pose_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& poses,
  JacobianMode mode) const {
  std::vector<PoseJacobianArray> pose_jacobians(bucket_factors_.size());
  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    const double u = bucket_factors_[i].u;
    if (mode == JacobianMode::NUMERIC_FULL) {
      pose_jacobians[i] = bspline_pose_jacobians_numeric(poses, u, numeric_eps_);
    } else {
      pose_jacobians[i] = bspline_pose_jacobians_semi_analytic(poses, u);
    }
  }
  return pose_jacobians;
}

void IntegratedBSplineGICPFactorGPU::update_bucket_poses(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  last_control_poses_ = poses;
  last_control_poses_valid_ = true;
  bucket_poses_.resize(bucket_factors_.size());

  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    const double u = bucket_factors_[i].u;
    bucket_poses_[i] = BSplineControlWindow::interpolate(poses, u);
  }
  bucket_pose_jacobians_ = compute_bucket_pose_jacobians(poses, jacobian_mode_);
}

gtsam::Values IntegratedBSplineGICPFactorGPU::bucket_values() const {
  gtsam::Values values;
  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    values.insert(bucket_factors_[i].key, bucket_poses_[i]);
  }
  return values;
}

IntegratedBSplineGICPFactorGPU::BucketSystem IntegratedBSplineGICPFactorGPU::collect_bucket_system(
  const gtsam::Values& bucket_vals) const {
  BucketSystem system;
  system.info_mats.reserve(bucket_factors_.size());
  system.linear_terms.reserve(bucket_factors_.size());

  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(bucket_graph_);
  factor_set.linearize(bucket_vals);

  for (const auto& bucket : bucket_factors_) {
    const auto gaussian = bucket.factor->linearize(bucket_vals);
    auto hessian = std::dynamic_pointer_cast<gtsam::HessianFactor>(gaussian);
    if (!hessian) {
      system.info_mats.emplace_back(gtsam::Matrix6::Zero());
      system.linear_terms.emplace_back(gtsam::Vector6::Zero());
      continue;
    }

    system.info_mats.emplace_back(hessian->information());
    system.linear_terms.emplace_back(hessian->linearTerm());
    system.constant += hessian->constantTerm();
    system.total_error += bucket.factor->error(bucket_vals);
    system.inlier_count += bucket.factor->num_inliers();
  }

  return system;
}

void IntegratedBSplineGICPFactorGPU::map_bucket_system(
  const BucketSystem& system,
  const std::vector<PoseJacobianArray>& pose_jacobians,
  Eigen::Matrix<double, 24, 24>* H,
  Eigen::Matrix<double, 24, 1>* g) const {
  H->setZero();
  g->setZero();

  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    Eigen::Matrix<double, 6, 24> J = Eigen::Matrix<double, 6, 24>::Zero();
    for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
      J.block<6, 6>(0, 6 * k) = pose_jacobians[i][k];
    }

    *H += J.transpose() * system.info_mats[i] * J;
    *g += J.transpose() * system.linear_terms[i];
  }
}

void IntegratedBSplineGICPFactorGPU::ensure_detailed_profile() const {
  if (!last_profile_.valid || !last_profile_.minimal || !last_control_poses_valid_) {
    return;
  }

  std::array<gtsam::Key, kBSplineControlPointCount> control_keys{};
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    control_keys[i] = keys_[i];
  }

  IntegratedBSplineGICPFactor cpu_audit_factor(control_keys, target_, source_, target_);
  cpu_audit_factor.set_enable_profiling(true);
  cpu_audit_factor.set_jacobian_mode(IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC);
  cpu_audit_factor.set_numeric_eps(numeric_eps_);
  cpu_audit_factor.set_max_correspondence_distance(std::sqrt(max_correspondence_distance_sq_));
  cpu_audit_factor.set_correspondence_candidate_count(correspondence_candidate_count_);
  cpu_audit_factor.set_correspondence_accept_ratio(correspondence_accept_ratio_);
  cpu_audit_factor.set_correspondence_min_score_gap(correspondence_min_score_gap_);
  cpu_audit_factor.set_outlier_mahalanobis_threshold(outlier_mahalanobis_threshold_);
  cpu_audit_factor.set_robust_kernel(robust_kernel_, robust_kernel_width_);
  cpu_audit_factor.set_robust_weight_floor(robust_weight_floor_);

  const gtsam::Values cpu_values = control_pose_values();
  if (cpu_values.size() != kBSplineControlPointCount) {
    return;
  }

  cpu_audit_factor.error(cpu_values);
  const auto audit_profile = cpu_audit_factor.profiling_report();
  if (!audit_profile.valid) {
    return;
  }

  const std::size_t matched_point_count =
    std::max(audit_profile.matched_point_count, static_cast<std::size_t>(std::max(last_inlier_count_, 0)));
  last_profile_.minimal = false;
  last_profile_.candidate_evaluation_count = audit_profile.candidate_evaluation_count;
  last_profile_.matched_point_count = matched_point_count;
  last_profile_.inlier_point_count = static_cast<std::size_t>(std::max(last_inlier_count_, 0));
  last_profile_.unique_target_count = audit_profile.unique_target_count;
  last_profile_.max_target_reuse = audit_profile.max_target_reuse;
  last_profile_.comparative_score_count = audit_profile.comparative_score_count;
  last_profile_.rejected_distance_count = audit_profile.rejected_distance_count;
  last_profile_.rejected_ambiguity_count = audit_profile.rejected_ambiguity_count;
  last_profile_.rejected_outlier_count = audit_profile.rejected_outlier_count;
  last_profile_.rejected_robust_count = audit_profile.rejected_robust_count;
  last_profile_.match_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.matched_point_count) /
                                              static_cast<double>(last_profile_.source_point_count);
  last_profile_.inlier_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.inlier_point_count) /
                                              static_cast<double>(last_profile_.source_point_count);
  last_profile_.mean_candidates_per_source = audit_profile.mean_candidates_per_source;
  last_profile_.unique_target_ratio =
    matched_point_count == 0 ? 0.0 : static_cast<double>(last_profile_.unique_target_count) / matched_point_count;
  last_profile_.max_target_reuse_ratio =
    matched_point_count == 0 ? 0.0 : static_cast<double>(last_profile_.max_target_reuse) / matched_point_count;
  last_profile_.mean_match_distance = audit_profile.mean_match_distance;
  last_profile_.max_match_distance = audit_profile.max_match_distance;
  last_profile_.mean_match_score = audit_profile.mean_match_score;
  last_profile_.mean_score_gap = audit_profile.mean_score_gap;
  last_profile_.mean_score_ratio = audit_profile.mean_score_ratio;
  last_profile_.mean_robust_weight = audit_profile.mean_robust_weight;
}

void IntegratedBSplineGICPFactorGPU::update_profile(
  const char* stage,
  double pose_update_ms,
  double gpu_linearize_ms,
  double reduction_ms,
  double total_ms,
  double total_error) const {
  auto profile = make_bspline_lidar_minimal_profile(
    BSplineLidarFactorBackend::GPU_GICP,
    static_cast<std::size_t>(frame::size(*source_)),
    target_point_count_,
    static_cast<std::size_t>(std::max(last_inlier_count_, 0)),
    static_cast<std::size_t>(std::max(last_inlier_count_, 0)),
    stage);
  profile.time_bucket_count = time_table_.size();
  profile.max_time_bucket_population =
    time_bucket_populations_.empty() ? 0U : *std::max_element(time_bucket_populations_.begin(), time_bucket_populations_.end());
  profile.mean_time_bucket_population =
    time_bucket_populations_.empty()
      ? 0.0
      : static_cast<double>(frame::size(*source_)) / static_cast<double>(time_bucket_populations_.size());
  profile.pose_update_ms = pose_update_ms;
  profile.correspondence_ms = gpu_linearize_ms;
  profile.accumulation_ms = reduction_ms;
  profile.total_ms = total_ms;
  profile.total_error = total_error;
  last_profile_ = profile;
}

double IntegratedBSplineGICPFactorGPU::error(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  const auto t_pose_start = Clock::now();
  update_bucket_poses(values);
  const auto t_pose_end = Clock::now();

  const gtsam::Values bucket_vals = bucket_values();
  const auto t_gpu_start = Clock::now();
  const auto system = collect_bucket_system(bucket_vals);
  const auto t_gpu_end = Clock::now();
  last_inlier_count_ = system.inlier_count;
  const auto t_end = Clock::now();

  update_profile(
    "gpu_linearized",
    std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count(),
    std::chrono::duration<double, std::milli>(t_gpu_end - t_gpu_start).count(),
    0.0,
    std::chrono::duration<double, std::milli>(t_end - t_start).count(),
    system.total_error);
  return system.total_error;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineGICPFactorGPU::linearize(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  const auto t_pose_start = Clock::now();
  update_bucket_poses(values);
  const auto t_pose_end = Clock::now();

  const gtsam::Values bucket_vals = bucket_values();
  const auto t_gpu_start = Clock::now();
  const auto system = collect_bucket_system(bucket_vals);
  const auto t_gpu_end = Clock::now();

  Eigen::Matrix<double, 24, 24> H = Eigen::Matrix<double, 24, 24>::Zero();
  Eigen::Matrix<double, 24, 1> g = Eigen::Matrix<double, 24, 1>::Zero();

  const auto t_reduce_start = Clock::now();
  map_bucket_system(system, bucket_pose_jacobians_, &H, &g);
  const auto t_reduce_end = Clock::now();
  const auto t_end = Clock::now();
  last_inlier_count_ = system.inlier_count;

  update_profile(
    "gpu_linearized",
    std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count(),
    std::chrono::duration<double, std::milli>(t_gpu_end - t_gpu_start).count(),
    std::chrono::duration<double, std::milli>(t_reduce_end - t_reduce_start).count(),
    std::chrono::duration<double, std::milli>(t_end - t_start).count(),
    system.total_error);

  const auto Gs = hessian_upper_blocks(H);
  const auto gs = linear_term_blocks(g);
  return std::make_shared<gtsam::HessianFactor>(keys_, Gs, gs, system.constant);
}

double IntegratedBSplineGICPFactorGPU::inlier_fraction() const {
  const auto source_size = static_cast<std::size_t>(std::max(frame::size(*source_), 0));
  return source_size == 0 ? 0.0 : static_cast<double>(last_inlier_count_) / static_cast<double>(source_size);
}

BSplineLidarFactorProfile IntegratedBSplineGICPFactorGPU::profiling_report() const {
  if (enable_profiling_ && last_profile_.valid) {
    ensure_detailed_profile();
  }
  return last_profile_;
}

BSplineLidarNumericAudit IntegratedBSplineGICPFactorGPU::check_against_numeric_full(
  const gtsam::Values& values,
  double perturbation_scale) const {
  BSplineLidarNumericAudit result;
  result.perturbation_scale = perturbation_scale;

  if (perturbation_scale <= 0.0) {
    return result;
  }

  const auto saved_profile = last_profile_;
  const auto saved_inliers = last_inlier_count_;
  const auto saved_bucket_poses = bucket_poses_;
  const auto saved_bucket_pose_jacobians = bucket_pose_jacobians_;
  const auto saved_control_poses = last_control_poses_;
  const auto saved_control_valid = last_control_poses_valid_;

  const auto poses = control_poses(values);
  last_control_poses_ = poses;
  last_control_poses_valid_ = true;
  bucket_poses_.resize(bucket_factors_.size());
  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    bucket_poses_[i] = BSplineControlWindow::interpolate(poses, bucket_factors_[i].u);
  }
  const auto bucket_vals = bucket_values();
  const auto system = collect_bucket_system(bucket_vals);
  const auto numeric_jacobians = compute_bucket_pose_jacobians(poses, JacobianMode::NUMERIC_FULL);
  const auto semi_jacobians = compute_bucket_pose_jacobians(poses, JacobianMode::SEMI_ANALYTIC);

  auto make_factor = [&](const std::vector<PoseJacobianArray>& jacobians) {
    Eigen::Matrix<double, 24, 24> H = Eigen::Matrix<double, 24, 24>::Zero();
    Eigen::Matrix<double, 24, 1> g = Eigen::Matrix<double, 24, 1>::Zero();
    map_bucket_system(system, jacobians, &H, &g);
    return std::make_shared<gtsam::HessianFactor>(keys_, hessian_upper_blocks(H), linear_term_blocks(g), system.constant);
  };

  const auto numeric_linear = make_factor(numeric_jacobians);
  const auto semi_linear = make_factor(semi_jacobians);

  auto measure_actual_error = [&](const gtsam::VectorValues& delta) {
    const gtsam::Values perturbed = values.retract(delta);
    return error(perturbed);
  };

  auto make_delta = [&](bool rotation_only) {
    gtsam::VectorValues delta;
    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      gtsam::Vector6 d = gtsam::Vector6::Zero();
      const double scale = perturbation_scale * (1.0 + 0.1 * static_cast<double>(i));
      if (rotation_only) {
        d << 1.0, -0.7, 0.45, 0.0, 0.0, 0.0;
      } else {
        d << 0.0, 0.0, 0.0, 0.6, -0.35, 0.2;
      }
      d *= scale;
      delta.insert(keys_[i], d);
    }
    return delta;
  };

  const auto rotation_delta = make_delta(true);
  const auto translation_delta = make_delta(false);
  result.valid = true;
  result.numeric_rotation_predicted_error = numeric_linear->error(rotation_delta);
  result.semi_rotation_predicted_error = semi_linear->error(rotation_delta);
  result.rotation_actual_error = measure_actual_error(rotation_delta);
  result.rotation_abs_error = std::abs(result.semi_rotation_predicted_error - result.numeric_rotation_predicted_error);
  result.rotation_rel_error =
    result.rotation_abs_error /
    std::max(1e-9, std::max(std::abs(result.semi_rotation_predicted_error), std::abs(result.numeric_rotation_predicted_error)));

  result.numeric_translation_predicted_error = numeric_linear->error(translation_delta);
  result.semi_translation_predicted_error = semi_linear->error(translation_delta);
  result.translation_actual_error = measure_actual_error(translation_delta);
  result.translation_abs_error = std::abs(result.semi_translation_predicted_error - result.numeric_translation_predicted_error);
  result.translation_rel_error =
    result.translation_abs_error /
    std::max(1e-9, std::max(std::abs(result.semi_translation_predicted_error), std::abs(result.numeric_translation_predicted_error)));

  for (std::size_t axis = 0; axis < 3; ++axis) {
    gtsam::VectorValues axis_delta;
    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      gtsam::Vector6 d = gtsam::Vector6::Zero();
      d(static_cast<int>(axis)) = perturbation_scale * (1.0 + 0.1 * static_cast<double>(i));
      axis_delta.insert(keys_[i], d);
    }

    const double axis_numeric = numeric_linear->error(axis_delta);
    const double axis_semi = semi_linear->error(axis_delta);
    const double axis_actual = measure_actual_error(axis_delta);
    (void)axis_actual;
    result.axis_rotation_rel_error[axis] =
      std::abs(axis_semi - axis_numeric) / std::max(1e-9, std::max(std::abs(axis_semi), std::abs(axis_numeric)));
    result.mean_rotation_axis_rel_error += result.axis_rotation_rel_error[axis];
    if (result.axis_rotation_rel_error[axis] > result.max_rotation_axis_rel_error) {
      result.max_rotation_axis_rel_error = result.axis_rotation_rel_error[axis];
      result.worst_rotation_axis = axis;
    }
  }
  result.mean_rotation_axis_rel_error /= 3.0;

  last_profile_ = saved_profile;
  last_inlier_count_ = saved_inliers;
  bucket_poses_ = saved_bucket_poses;
  bucket_pose_jacobians_ = saved_bucket_pose_jacobians;
  last_control_poses_ = saved_control_poses;
  last_control_poses_valid_ = saved_control_valid;
  return result;
}

BSplineLidarDegeneracyReport IntegratedBSplineGICPFactorGPU::diagnose_degeneracy(
  const IntegratedBSplineGICPFactor::DegeneracyThresholds& thresholds) const {
  ensure_detailed_profile();

  BSplineLidarDegeneracyReport diagnostics;
  diagnostics.valid = last_profile_.valid;
  if (!diagnostics.valid) {
    return diagnostics;
  }

  diagnostics.empty_target = last_profile_.target_point_count == 0;
  diagnostics.ambiguity_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_ambiguity_count) /
                                              static_cast<double>(last_profile_.source_point_count);
  diagnostics.distance_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_distance_count) /
                                              static_cast<double>(last_profile_.source_point_count);
  diagnostics.outlier_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_outlier_count) /
                                              static_cast<double>(last_profile_.source_point_count);
  diagnostics.robust_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_robust_count) /
                                              static_cast<double>(last_profile_.source_point_count);

  diagnostics.low_match_ratio =
    thresholds.min_match_ratio > 0.0 && last_profile_.match_ratio < thresholds.min_match_ratio;
  diagnostics.low_inlier_ratio =
    thresholds.min_inlier_ratio > 0.0 && last_profile_.inlier_ratio < thresholds.min_inlier_ratio;
  diagnostics.low_target_diversity =
    thresholds.min_unique_target_ratio > 0.0 && last_profile_.unique_target_ratio < thresholds.min_unique_target_ratio;
  diagnostics.high_target_reuse =
    thresholds.max_target_reuse_ratio > 0.0 && last_profile_.max_target_reuse_ratio > thresholds.max_target_reuse_ratio;
  diagnostics.high_ambiguity_rejection =
    thresholds.max_ambiguity_rejection_ratio > 0.0 &&
    diagnostics.ambiguity_rejection_ratio > thresholds.max_ambiguity_rejection_ratio;
  diagnostics.weak_score_separation =
    thresholds.min_mean_score_gap > 0.0 &&
    last_profile_.comparative_score_count > 0 &&
    last_profile_.mean_score_gap < thresholds.min_mean_score_gap;

  diagnostics.warning_count += diagnostics.empty_target ? 1U : 0U;
  diagnostics.warning_count += diagnostics.low_match_ratio ? 1U : 0U;
  diagnostics.warning_count += diagnostics.low_inlier_ratio ? 1U : 0U;
  diagnostics.warning_count += diagnostics.low_target_diversity ? 1U : 0U;
  diagnostics.warning_count += diagnostics.high_target_reuse ? 1U : 0U;
  diagnostics.warning_count += diagnostics.high_ambiguity_rejection ? 1U : 0U;
  diagnostics.warning_count += diagnostics.weak_score_separation ? 1U : 0U;
  return diagnostics;
}

BSplineLidarFactorResult IntegratedBSplineGICPFactorGPU::make_result(
  double factor_error,
  int inlier_count,
  double inlier_fraction,
  const BSplineLidarNumericAudit* numeric_audit,
  const BSplineLidarDegeneracyReport* degeneracy) const {
  const auto profile = profiling_report();
  return make_bspline_lidar_factor_result(
    BSplineLidarFactorBackend::GPU_GICP,
    factor_error,
    inlier_count,
    inlier_fraction,
    &profile,
    numeric_audit,
    degeneracy);
}

std::vector<Eigen::Vector4d> IntegratedBSplineGICPFactorGPU::deskewed_source_points(
  const gtsam::Values& values,
  bool local) const {
  update_bucket_poses(values);

  std::vector<Eigen::Vector4d> points;
  points.reserve(frame::size(*source_));

  const gtsam::Pose3 reference = bucket_poses_.empty() ? gtsam::Pose3() : bucket_poses_.front();
  for (int i = 0; i < frame::size(*source_); ++i) {
    const int time_index = time_indices_[static_cast<std::size_t>(i)];
    const auto& pose = bucket_poses_[static_cast<std::size_t>(time_index)];
    const auto& source_pt = frame::point(*source_, i);

    const gtsam::Point3 world_pt = pose.transformFrom(source_pt.template head<3>().eval());
    Eigen::Vector4d pt = Eigen::Vector4d::Ones();
    if (local) {
      pt.template head<3>() = reference.transformTo(world_pt);
    } else {
      pt.template head<3>() = world_pt;
    }
    points.push_back(pt);
  }

  return points;
}

}  // namespace iap

#endif  // GTSAM_POINTS_USE_CUDA
