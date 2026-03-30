#include <iap/odometry/integrated_bspline_gicp_factor_gpu.hpp>

#ifdef GTSAM_POINTS_USE_CUDA

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

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
    target_gpu_->insert(*target_cpu_points_);
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

    auto bucket_cpu = std::make_shared<gtsam_points::PointCloudCPU>();
    bucket_cpu->points_storage.reserve(point_indices.size());
    bucket_cpu->covs_storage.reserve(point_indices.size());
    if (frame::has_normals(*source_)) {
      bucket_cpu->normals_storage.reserve(point_indices.size());
    }

    for (const int point_index : point_indices) {
      bucket_cpu->points_storage.emplace_back(frame::point(*source_, point_index));
      bucket_cpu->covs_storage.emplace_back(frame::cov(*source_, point_index));
      if (frame::has_normals(*source_)) {
        bucket_cpu->normals_storage.emplace_back(frame::normal(*source_, point_index));
      }
    }

    bucket_cpu->points = bucket_cpu->points_storage.data();
    bucket_cpu->covs = bucket_cpu->covs_storage.data();
    if (!bucket_cpu->normals_storage.empty()) {
      bucket_cpu->normals = bucket_cpu->normals_storage.data();
    }

    auto bucket_gpu = gtsam_points::PointCloudGPU::clone(*bucket_cpu, stream_);
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
    bucket.factor->set_enable_surface_validation(true);
    bucket_graph_.add(bucket.factor);
    bucket_factors_.push_back(std::move(bucket));
  }
}

void IntegratedBSplineGICPFactorGPU::update_bucket_poses(const gtsam::Values& values) const {
  const auto poses = control_poses(values);
  bucket_poses_.resize(bucket_factors_.size());
  bucket_pose_jacobians_.resize(bucket_factors_.size());

  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    const double u = bucket_factors_[i].u;
    const gtsam::Pose3 base_pose = BSplineControlWindow::interpolate(poses, u);
    bucket_poses_[i] = base_pose;

    for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
      gtsam::Matrix6 J = gtsam::Matrix6::Zero();
      for (int d = 0; d < 6; ++d) {
        gtsam::Vector6 delta = gtsam::Vector6::Zero();
        delta(d) = numeric_eps_;

        auto perturbed = poses;
        perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));
        const gtsam::Pose3 pose_plus = BSplineControlWindow::interpolate(perturbed, u);
        const gtsam::Vector6 xi = gtsam::Pose3::Logmap(base_pose.between(pose_plus));
        J.col(d) = xi / numeric_eps_;
      }
      bucket_pose_jacobians_[i][k] = J;
    }
  }
}

gtsam::Values IntegratedBSplineGICPFactorGPU::bucket_values() const {
  gtsam::Values values;
  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    values.insert(bucket_factors_[i].key, bucket_poses_[i]);
  }
  return values;
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
  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(bucket_graph_);
  factor_set.linearize(bucket_vals);
  const auto t_gpu_end = Clock::now();

  double total_error = 0.0;
  last_inlier_count_ = 0;
  for (const auto& bucket : bucket_factors_) {
    total_error += bucket.factor->error(bucket_vals);
    last_inlier_count_ += bucket.factor->num_inliers();
  }
  const auto t_end = Clock::now();

  update_profile(
    "gpu_linearized",
    std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count(),
    std::chrono::duration<double, std::milli>(t_gpu_end - t_gpu_start).count(),
    0.0,
    std::chrono::duration<double, std::milli>(t_end - t_start).count(),
    total_error);
  return total_error;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineGICPFactorGPU::linearize(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  const auto t_pose_start = Clock::now();
  update_bucket_poses(values);
  const auto t_pose_end = Clock::now();

  const gtsam::Values bucket_vals = bucket_values();
  const auto t_gpu_start = Clock::now();
  gtsam_points::NonlinearFactorSetGPU factor_set;
  factor_set.add(bucket_graph_);
  factor_set.linearize(bucket_vals);
  const auto t_gpu_end = Clock::now();

  Eigen::Matrix<double, 24, 24> H = Eigen::Matrix<double, 24, 24>::Zero();
  Eigen::Matrix<double, 24, 1> g = Eigen::Matrix<double, 24, 1>::Zero();
  double constant = 0.0;
  double total_error = 0.0;
  last_inlier_count_ = 0;

  const auto t_reduce_start = Clock::now();
  for (std::size_t i = 0; i < bucket_factors_.size(); ++i) {
    const auto gaussian = bucket_factors_[i].factor->linearize(bucket_vals);
    auto hessian = std::dynamic_pointer_cast<gtsam::HessianFactor>(gaussian);
    if (!hessian) {
      continue;
    }

    const gtsam::Matrix G_bucket = hessian->information();
    const gtsam::Vector g_bucket = hessian->linearTerm();
    constant += hessian->constantTerm();
    total_error += bucket_factors_[i].factor->error(bucket_vals);
    last_inlier_count_ += bucket_factors_[i].factor->num_inliers();

    Eigen::Matrix<double, 6, 24> J = Eigen::Matrix<double, 6, 24>::Zero();
    for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
      J.block<6, 6>(0, 6 * k) = bucket_pose_jacobians_[i][k];
    }

    H += J.transpose() * G_bucket * J;
    g += J.transpose() * g_bucket;
  }
  const auto t_reduce_end = Clock::now();
  const auto t_end = Clock::now();

  update_profile(
    "gpu_linearized",
    std::chrono::duration<double, std::milli>(t_pose_end - t_pose_start).count(),
    std::chrono::duration<double, std::milli>(t_gpu_end - t_gpu_start).count(),
    std::chrono::duration<double, std::milli>(t_reduce_end - t_reduce_start).count(),
    std::chrono::duration<double, std::milli>(t_end - t_start).count(),
    total_error);

  const auto Gs = hessian_upper_blocks(H);
  const auto gs = linear_term_blocks(g);
  return std::make_shared<gtsam::HessianFactor>(keys_, Gs, gs, constant);
}

double IntegratedBSplineGICPFactorGPU::inlier_fraction() const {
  const auto source_size = static_cast<std::size_t>(std::max(frame::size(*source_), 0));
  return source_size == 0 ? 0.0 : static_cast<double>(last_inlier_count_) / static_cast<double>(source_size);
}

BSplineLidarFactorProfile IntegratedBSplineGICPFactorGPU::profiling_report() const {
  return last_profile_;
}

BSplineLidarFactorResult IntegratedBSplineGICPFactorGPU::make_result(
  double factor_error,
  int inlier_count,
  double inlier_fraction) const {
  const auto profile = profiling_report();
  return make_bspline_lidar_factor_result(
    BSplineLidarFactorBackend::GPU_GICP,
    factor_error,
    inlier_count,
    inlier_fraction,
    &profile);
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
