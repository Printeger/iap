#include <iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp>

#ifdef GTSAM_POINTS_USE_CUDA

#include <cuda_runtime.h>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam_points/cuda/kernels/vector3_hash.cuh>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace iap {

namespace frame = gtsam_points::frame;

namespace {

constexpr int kStateDim = static_cast<int>(kBSplineControlPointCount * 6);
constexpr int kThreadsPerBlock = 128;

void throw_if_cuda_error(cudaError_t error, const char* context) {
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string(context) + ": " + cudaGetErrorString(error));
  }
}

__device__ inline int lookup_voxel_coord(
  const int max_bucket_scan_count,
  const int num_buckets,
  const gtsam_points::VoxelBucket* buckets_ptr,
  const Eigen::Vector3i& coord) {
  const uint64_t hash = gtsam_points::vector3i_hash(coord);
  for (int i = 0; i < max_bucket_scan_count; ++i) {
    const uint64_t bucket_index = (hash + static_cast<uint64_t>(i)) % static_cast<uint64_t>(num_buckets);
    const auto& bucket = buckets_ptr[bucket_index];
    if (bucket.second < 0) {
      return -1;
    }
    if (gtsam_points::equal(bucket.first, coord)) {
      return bucket.second;
    }
  }
  return -1;
}

__device__ inline float atomic_max_positive(float* address, float value) {
  int* address_as_i = reinterpret_cast<int*>(address);
  int old = *address_as_i;
  int assumed = 0;
  const int desired = __float_as_int(value);
  while (__int_as_float(old) < value) {
    assumed = old;
    old = atomicCAS(address_as_i, assumed, desired);
    if (old == assumed) {
      break;
    }
  }
  return __int_as_float(old);
}

__host__ __device__ inline float residual_norm_from_mahalanobis(float mahalanobis_error) {
  return sqrtf(fmaxf(0.0f, mahalanobis_error));
}

__host__ __device__ inline float robust_weight(
  IntegratedBSplineGICPFactor::RobustKernel kernel,
  float kernel_width,
  float mahalanobis_error) {
  const float residual_norm = residual_norm_from_mahalanobis(mahalanobis_error);
  switch (kernel) {
    case IntegratedBSplineGICPFactor::RobustKernel::NONE:
      return 1.0f;
    case IntegratedBSplineGICPFactor::RobustKernel::HUBER:
      if (residual_norm <= kernel_width) {
        return 1.0f;
      }
      return kernel_width / fmaxf(1e-9f, residual_norm);
    case IntegratedBSplineGICPFactor::RobustKernel::CAUCHY: {
      const float scaled = residual_norm / kernel_width;
      return 1.0f / (1.0f + scaled * scaled);
    }
  }
  return 1.0f;
}

__host__ __device__ inline float robust_cost(
  IntegratedBSplineGICPFactor::RobustKernel kernel,
  float kernel_width,
  float mahalanobis_error) {
  const float residual_norm = residual_norm_from_mahalanobis(mahalanobis_error);
  switch (kernel) {
    case IntegratedBSplineGICPFactor::RobustKernel::NONE:
      return mahalanobis_error;
    case IntegratedBSplineGICPFactor::RobustKernel::HUBER:
      if (residual_norm <= kernel_width) {
        return mahalanobis_error;
      }
      return 2.0f * kernel_width * residual_norm - kernel_width * kernel_width;
    case IntegratedBSplineGICPFactor::RobustKernel::CAUCHY: {
      const float scaled = residual_norm / kernel_width;
      return kernel_width * kernel_width * log1pf(scaled * scaled);
    }
  }
  return mahalanobis_error;
}

struct BSplineBasisWeights {
  float data[kBSplineControlPointCount];
  __host__ __device__ float operator()(int i) const { return data[i]; }
};

__host__ __device__ inline BSplineBasisWeights bspline_basis_gpu(float u) {
  const float clamped = fminf(1.0f, fmaxf(0.0f, u));
  const float u2 = clamped * clamped;
  const float u3 = u2 * clamped;
  BSplineBasisWeights weights{};
  weights.data[0] = (1.0f - 3.0f * clamped + 3.0f * u2 - u3) / 6.0f;
  weights.data[1] = (4.0f - 6.0f * u2 + 3.0f * u3) / 6.0f;
  weights.data[2] = (1.0f + 3.0f * clamped + 3.0f * u2 - 3.0f * u3) / 6.0f;
  weights.data[3] = u3 / 6.0f;
  return weights;
}

Eigen::Vector4f aligned_quaternion_coeffs(const gtsam::Pose3& pose, const Eigen::Quaternionf& reference) {
  Eigen::Quaternionf q(pose.rotation().toQuaternion().cast<float>());
  q.normalize();
  if (reference.dot(q) < 0.0f) {
    q.coeffs() *= -1.0f;
  }
  return q.coeffs();
}

}  // namespace

struct IntegratedBSplineGICPFactorGPUKernel::DeviceControlPose {
  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  float qw = 1.0f;
  float tx = 0.0f;
  float ty = 0.0f;
  float tz = 0.0f;
  float rotation[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
  };
};

struct IntegratedBSplineGICPFactorGPUKernel::DeviceKernelStats {
  int matched_count = 0;
  int inlier_count = 0;
  int rejected_distance_count = 0;
  int rejected_ambiguity_count = 0;
  int rejected_outlier_count = 0;
  int rejected_robust_count = 0;
  int candidate_evaluation_count = 0;
  int comparative_score_count = 0;
  float total_error = 0.0f;
  float sum_robust_weight = 0.0f;
  float sum_match_distance = 0.0f;
  float max_match_distance = 0.0f;
  float sum_match_score = 0.0f;
  float sum_score_gap = 0.0f;
  float sum_score_ratio = 0.0f;
};

struct IntegratedBSplineGICPFactorGPUKernel::EvaluationResult {
  bool valid = false;
  Eigen::Matrix<float, kStateDim, kStateDim, Eigen::RowMajor> H =
    Eigen::Matrix<float, kStateDim, kStateDim, Eigen::RowMajor>::Zero();
  Eigen::Matrix<float, kStateDim, 1> b = Eigen::Matrix<float, kStateDim, 1>::Zero();
  double total_error = 0.0;
  int matched_count = 0;
  int inlier_count = 0;
  int rejected_distance_count = 0;
  int rejected_ambiguity_count = 0;
  int rejected_outlier_count = 0;
  int rejected_robust_count = 0;
  int candidate_evaluation_count = 0;
  int comparative_score_count = 0;
  double mean_robust_weight = 1.0;
  double mean_match_distance = 0.0;
  double max_match_distance = 0.0;
  double mean_match_score = 0.0;
  double mean_score_gap = 0.0;
  double mean_score_ratio = 0.0;
  double kernel_pose_query_ms = 0.0;
  double kernel_correspondence_ms = 0.0;
  double kernel_residual_weight_ms = 0.0;
  double kernel_reduction_ms = 0.0;
  double host_sync_ms = 0.0;
  double host_result_pack_ms = 0.0;
  double total_ms = 0.0;
  std::vector<int> matched_target_indices;
};

namespace {

struct DeviceControlSet {
  IntegratedBSplineGICPFactorGPUKernel::DeviceControlPose poses[kBSplineControlPointCount];
};

__device__ Eigen::Matrix<float, 4, 3> quaternion_right_local_jacobian_device(const Eigen::Quaternionf& q) {
  Eigen::Matrix<float, 4, 3> J = Eigen::Matrix<float, 4, 3>::Zero();
  Eigen::Matrix3f hat = Eigen::Matrix3f::Zero();
  hat(0, 1) = -q.z();
  hat(0, 2) = q.y();
  hat(1, 0) = q.z();
  hat(1, 2) = -q.x();
  hat(2, 0) = -q.y();
  hat(2, 1) = q.x();
  J.block<3, 3>(0, 0) = 0.5f * (q.w() * Eigen::Matrix3f::Identity() + hat);
  J.row(3) = -0.5f * q.vec().transpose();
  return J;
}

__device__ Eigen::Matrix4f normalized_vector_jacobian_device(const Eigen::Vector4f& coeffs) {
  const float norm = coeffs.norm();
  if (norm < 1e-9f) {
    return Eigen::Matrix4f::Zero();
  }
  const Eigen::Vector4f normalized = coeffs / norm;
  return (Eigen::Matrix4f::Identity() - normalized * normalized.transpose()) / norm;
}

__device__ Eigen::Matrix<float, 3, 4> quaternion_local_coordinates_jacobian_device(const Eigen::Quaternionf& q) {
  const Eigen::Matrix<float, 4, 3> J = quaternion_right_local_jacobian_device(q);
  const Eigen::Matrix3f normal = J.transpose() * J;
  return normal.inverse() * J.transpose();
}

__device__ Eigen::Matrix<float, 3, 4> rotated_point_quaternion_jacobian_device(
  const Eigen::Vector4f& coeffs,
  const Eigen::Vector3f& point) {
  Eigen::Matrix<float, 3, 4> J = Eigen::Matrix<float, 3, 4>::Zero();
  const Eigen::Vector3f v = coeffs.head<3>();
  const float w = coeffs[3];
  const float vp = v.dot(point);

  for (int j = 0; j < 3; ++j) {
    const Eigen::Vector3f e = Eigen::Vector3f::Unit(j);
    J.col(j) = -2.0f * v[j] * point + 2.0f * e * vp + 2.0f * v * point[j] + 2.0f * w * e.cross(point);
  }
  J.col(3) = 2.0f * w * point + 2.0f * v.cross(point);
  return J;
}

__device__ Eigen::Matrix3f load_rotation_matrix(const IntegratedBSplineGICPFactorGPUKernel::DeviceControlPose& pose) {
  Eigen::Matrix3f R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = pose.rotation[r * 3 + c];
    }
  }
  return R;
}

__device__ Eigen::Vector3f load_translation(const IntegratedBSplineGICPFactorGPUKernel::DeviceControlPose& pose) {
  return Eigen::Vector3f(pose.tx, pose.ty, pose.tz);
}

__device__ Eigen::Quaternionf load_quaternion(const IntegratedBSplineGICPFactorGPUKernel::DeviceControlPose& pose) {
  Eigen::Quaternionf q(pose.qw, pose.qx, pose.qy, pose.qz);
  q.normalize();
  return q;
}

__global__ void evaluate_ct_lidar_kernel(
  DeviceControlSet control_set,
  const float* normalized_times,
  const Eigen::Vector3f* source_points,
  const Eigen::Matrix3f* source_covs,
  int num_points,
  const gtsam_points::VoxelMapInfo* voxelmap_info_ptr,
  const gtsam_points::VoxelBucket* buckets_ptr,
  const Eigen::Vector3f* voxel_means,
  const Eigen::Matrix3f* voxel_covs,
  float max_correspondence_distance,
  float max_correspondence_distance_sq,
  int correspondence_candidate_count,
  float correspondence_accept_ratio,
  float correspondence_min_score_gap,
  float outlier_mahalanobis_threshold,
  int robust_kernel,
  float robust_kernel_width,
  float robust_weight_floor,
  bool compute_hessian,
  float* hessian_upper,
  float* gradient,
  IntegratedBSplineGICPFactorGPUKernel::DeviceKernelStats* stats,
  int* matched_target_indices) {
  const int idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;
  }

  matched_target_indices[idx] = -1;

  const auto weights = bspline_basis_gpu(normalized_times[idx]);
  Eigen::Vector4f blended_coeffs = Eigen::Vector4f::Zero();
  Eigen::Vector3f translation = Eigen::Vector3f::Zero();
  Eigen::Quaternionf aligned_quats[kBSplineControlPointCount];
  Eigen::Matrix3f control_rotations[kBSplineControlPointCount];

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    const auto q = load_quaternion(control_set.poses[k]);
    aligned_quats[k] = q;
    control_rotations[k] = load_rotation_matrix(control_set.poses[k]);
    blended_coeffs += weights(static_cast<int>(k)) * q.coeffs();
    translation += weights(static_cast<int>(k)) * load_translation(control_set.poses[k]);
  }

  if (blended_coeffs.norm() < 1e-9f) {
    blended_coeffs = aligned_quats[0].coeffs();
  }
  blended_coeffs.normalize();
  const Eigen::Quaternionf blended_q(blended_coeffs[3], blended_coeffs[0], blended_coeffs[1], blended_coeffs[2]);
  const Eigen::Matrix3f blended_R = blended_q.toRotationMatrix();

  const Eigen::Vector3f source_point = source_points[idx];
  const Eigen::Vector3f transformed = blended_R * source_point + translation;

  const auto& info = *voxelmap_info_ptr;
  int search_radius = 1;
  if (max_correspondence_distance > 0.0f) {
    search_radius = static_cast<int>(ceilf(max_correspondence_distance / fmaxf(info.voxel_resolution, 1e-6f)));
    if (search_radius < 1) {
      search_radius = 1;
    }
  }
  const Eigen::Vector3i center_coord = gtsam_points::calc_voxel_coord(transformed, info.voxel_resolution);

  int best_voxel = -1;
  float best_sq_dist = max_correspondence_distance_sq;
  float best_score = 1.0e30f;
  float second_best_score = 1.0e30f;
  Eigen::Vector3f best_mean = Eigen::Vector3f::Zero();
  Eigen::Matrix3f best_voxel_cov = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f best_M = Eigen::Matrix3f::Zero();

  int valid_candidate_count = 0;
  for (int dx = -search_radius; dx <= search_radius; ++dx) {
    for (int dy = -search_radius; dy <= search_radius; ++dy) {
      for (int dz = -search_radius; dz <= search_radius; ++dz) {
        const Eigen::Vector3i voxel_coord = center_coord + Eigen::Vector3i(dx, dy, dz);
        const int voxel_id = lookup_voxel_coord(info.max_bucket_scan_count, info.num_buckets, buckets_ptr, voxel_coord);
        if (voxel_id < 0) {
          continue;
        }

        const Eigen::Vector3f mean = voxel_means[voxel_id];
        const Eigen::Vector3f residual = transformed - mean;
        const float sq_dist = residual.squaredNorm();
        if (sq_dist > max_correspondence_distance_sq) {
          continue;
        }

        const Eigen::Matrix3f voxel_cov = voxel_covs[voxel_id];
        Eigen::Matrix3f RCR = voxel_cov + blended_R * source_covs[idx] * blended_R.transpose();
        RCR.diagonal().array() += 1e-6f;
        const Eigen::Matrix3f M = RCR.inverse();
        const float score = residual.transpose() * M * residual;
        valid_candidate_count++;

        if (score < best_score) {
          second_best_score = best_score;
          best_score = score;
          best_sq_dist = sq_dist;
          best_voxel = voxel_id;
          best_mean = mean;
          best_voxel_cov = voxel_cov;
          best_M = M;
        } else if (score < second_best_score && valid_candidate_count > 1 && correspondence_candidate_count > 1) {
          second_best_score = score;
        }
      }
    }
  }

  atomicAdd(&stats->candidate_evaluation_count, valid_candidate_count);

  if (best_voxel < 0) {
    atomicAdd(&stats->rejected_distance_count, 1);
    return;
  }

  if (correspondence_candidate_count > 1 && second_best_score < 1.0e30f) {
    atomicAdd(&stats->comparative_score_count, 1);
    atomicAdd(&stats->sum_score_gap, second_best_score - best_score);
    atomicAdd(&stats->sum_score_ratio, best_score / fmaxf(1e-9f, second_best_score));

    if (correspondence_accept_ratio > 0.0f &&
        second_best_score > 1e-9f &&
        (best_score / second_best_score) >= correspondence_accept_ratio) {
      atomicAdd(&stats->rejected_ambiguity_count, 1);
      return;
    }

    if (correspondence_min_score_gap > 0.0f &&
        (second_best_score - best_score) <= correspondence_min_score_gap) {
      atomicAdd(&stats->rejected_ambiguity_count, 1);
      return;
    }
  }

  matched_target_indices[idx] = best_voxel;
  atomicAdd(&stats->matched_count, 1);
  const float match_distance = sqrtf(fmaxf(0.0f, best_sq_dist));
  atomicAdd(&stats->sum_match_distance, match_distance);
  atomic_max_positive(&stats->max_match_distance, match_distance);
  atomicAdd(&stats->sum_match_score, best_score);

  const Eigen::Vector3f residual = transformed - best_mean;
  const float mahalanobis_error = residual.transpose() * best_M * residual;
  if (outlier_mahalanobis_threshold > 0.0f &&
      residual_norm_from_mahalanobis(mahalanobis_error) > outlier_mahalanobis_threshold) {
    atomicAdd(&stats->rejected_outlier_count, 1);
    return;
  }

  const auto robust_kernel_mode = static_cast<IntegratedBSplineGICPFactor::RobustKernel>(robust_kernel);
  const float weight = robust_weight(robust_kernel_mode, robust_kernel_width, mahalanobis_error);
  if (robust_weight_floor > 0.0f && weight < robust_weight_floor) {
    atomicAdd(&stats->rejected_robust_count, 1);
    return;
  }

  atomicAdd(&stats->inlier_count, 1);
  atomicAdd(&stats->sum_robust_weight, weight);
  atomicAdd(&stats->total_error, robust_cost(robust_kernel_mode, robust_kernel_width, mahalanobis_error));

  if (!compute_hessian) {
    return;
  }

  const Eigen::Matrix4f normalization_jacobian = normalized_vector_jacobian_device(blended_coeffs);
  const Eigen::Matrix<float, 3, 4> quat_local = quaternion_local_coordinates_jacobian_device(blended_q);
  const Eigen::Matrix<float, 3, 4> H_transed_quat = rotated_point_quaternion_jacobian_device(blended_coeffs, source_point);

  Eigen::Matrix<float, 3, 6> point_jacobians[kBSplineControlPointCount];
  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    const auto quat_jac = quaternion_right_local_jacobian_device(aligned_quats[k]);
    const Eigen::Matrix<float, 4, 3> blended_quat_jac =
      normalization_jacobian * (weights(static_cast<int>(k)) * quat_jac);
    point_jacobians[k].setZero();
    point_jacobians[k].block<3, 3>(0, 0) = H_transed_quat * blended_quat_jac;
    point_jacobians[k].block<3, 3>(0, 3) = weights(static_cast<int>(k)) * control_rotations[k];
  }

  for (std::size_t a = 0; a < kBSplineControlPointCount; ++a) {
    const Eigen::Matrix<float, 6, 1> weighted_residual = weight * point_jacobians[a].transpose() * best_M * residual;
    for (int r = 0; r < 6; ++r) {
      atomicAdd(&gradient[a * 6 + r], weighted_residual[r]);
    }

    for (std::size_t c = a; c < kBSplineControlPointCount; ++c) {
      const Eigen::Matrix<float, 6, 6> block = weight * point_jacobians[a].transpose() * best_M * point_jacobians[c];
      for (int r = 0; r < 6; ++r) {
        const int row = static_cast<int>(a) * 6 + r;
        for (int cc = 0; cc < 6; ++cc) {
          const int col = static_cast<int>(c) * 6 + cc;
          atomicAdd(&hessian_upper[row * kStateDim + col], block(r, cc));
        }
      }
    }
  }
}

}  // namespace

IntegratedBSplineGICPFactorGPUKernel::IntegratedBSplineGICPFactorGPUKernel(
  const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
  std::shared_ptr<const iap::ISharedTargetHandle> target_handle,
  const std::shared_ptr<const gtsam_points::PointCloud>& source,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer)
: gtsam::NonlinearFactor(gtsam::KeyVector(keys.begin(), keys.end())),
  stream_(stream),
  temp_buffer_(std::move(temp_buffer)),
  source_(source) {
  if (!target_handle || !target_handle->target_snapshot()) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPUKernel requires a non-null shared target handle");
  }
  if (!source_ || !frame::has_points(*source_) || !frame::has_covs(*source_) || !frame::has_times(*source_)) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPUKernel requires source points, covariances, and times");
  }

  const double time_eps = 1e-3;
  time_table_.reserve(frame::size(*source_) / 10 + 1);
  time_indices_.reserve(frame::size(*source_));
  normalized_times_.reserve(frame::size(*source_));

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
  for (int i = 0; i < frame::size(*source_); ++i) {
    const double t = frame::time(*source_, i);
    normalized_times_.push_back(static_cast<float>((t - time_min) / denom));
  }
  for (auto& t : time_table_) {
    t = (t - time_min) / denom;
  }

  ensure_source_gpu();
  bind_target_handle(std::move(target_handle));
}

IntegratedBSplineGICPFactorGPUKernel::IntegratedBSplineGICPFactorGPUKernel(
  const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
  std::shared_ptr<const iap::SharedTargetGpuResources> target_gpu_resources,
  const std::shared_ptr<const gtsam_points::iVox>& target_snapshot,
  const std::shared_ptr<const gtsam_points::PointCloud>& source,
  CUstream_st* stream,
  std::shared_ptr<gtsam_points::TempBufferManager> temp_buffer)
: gtsam::NonlinearFactor(gtsam::KeyVector(keys.begin(), keys.end())),
  stream_(stream),
  temp_buffer_(std::move(temp_buffer)),
  source_(source) {
  if (!target_gpu_resources) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPUKernel requires non-null shared target GPU resources");
  }
  if (!source_ || !frame::has_points(*source_) || !frame::has_covs(*source_) || !frame::has_times(*source_)) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPUKernel requires source points, covariances, and times");
  }

  const double time_eps = 1e-3;
  time_table_.reserve(frame::size(*source_) / 10 + 1);
  time_indices_.reserve(frame::size(*source_));
  normalized_times_.reserve(frame::size(*source_));

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
  for (int i = 0; i < frame::size(*source_); ++i) {
    const double t = frame::time(*source_, i);
    normalized_times_.push_back(static_cast<float>((t - time_min) / denom));
  }
  for (auto& t : time_table_) {
    t = (t - time_min) / denom;
  }

  ensure_source_gpu();
  bind_target_gpu_resources(std::move(target_gpu_resources), target_snapshot);
}

IntegratedBSplineGICPFactorGPUKernel::~IntegratedBSplineGICPFactorGPUKernel() {
  if (normalized_times_gpu_) {
    cudaFree(normalized_times_gpu_);
  }
  if (linearized_hessian_gpu_) {
    cudaFree(linearized_hessian_gpu_);
  }
  if (linearized_gradient_gpu_) {
    cudaFree(linearized_gradient_gpu_);
  }
  if (kernel_stats_gpu_) {
    cudaFree(kernel_stats_gpu_);
  }
  if (matched_target_indices_gpu_) {
    cudaFree(matched_target_indices_gpu_);
  }
}

void IntegratedBSplineGICPFactorGPUKernel::set_numeric_eps(double eps) {
  numeric_eps_ = std::max(1e-8, eps);
}

void IntegratedBSplineGICPFactorGPUKernel::set_max_correspondence_distance(double dist) {
  max_correspondence_distance_ = std::max(1e-6, dist);
  max_correspondence_distance_sq_ = max_correspondence_distance_ * max_correspondence_distance_;
}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineGICPFactorGPUKernel::control_poses(
  const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

void IntegratedBSplineGICPFactorGPUKernel::ensure_source_gpu() const {
  if (!source_gpu_) {
    source_gpu_ = gtsam_points::PointCloudGPU::clone(*source_, stream_);
  }

  if (!normalized_times_gpu_ && !normalized_times_.empty()) {
    throw_if_cuda_error(cudaMalloc(reinterpret_cast<void**>(&normalized_times_gpu_), normalized_times_.size() * sizeof(float)), "cudaMalloc normalized_times_gpu");
    throw_if_cuda_error(
      cudaMemcpy(normalized_times_gpu_, normalized_times_.data(), normalized_times_.size() * sizeof(float), cudaMemcpyHostToDevice),
      "cudaMemcpy normalized_times_gpu");
  }

  if (!linearized_hessian_gpu_) {
    throw_if_cuda_error(cudaMalloc(reinterpret_cast<void**>(&linearized_hessian_gpu_), sizeof(float) * kStateDim * kStateDim), "cudaMalloc linearized_hessian_gpu");
  }
  if (!linearized_gradient_gpu_) {
    throw_if_cuda_error(cudaMalloc(reinterpret_cast<void**>(&linearized_gradient_gpu_), sizeof(float) * kStateDim), "cudaMalloc linearized_gradient_gpu");
  }
  if (!kernel_stats_gpu_) {
    throw_if_cuda_error(cudaMalloc(reinterpret_cast<void**>(&kernel_stats_gpu_), sizeof(DeviceKernelStats)), "cudaMalloc kernel_stats_gpu");
  }
  if (!matched_target_indices_gpu_) {
    throw_if_cuda_error(
      cudaMalloc(
        reinterpret_cast<void**>(&matched_target_indices_gpu_),
        sizeof(int) * std::max<std::size_t>(1, frame::size(*source_))),
      "cudaMalloc matched_target_indices_gpu");
  }
}

void IntegratedBSplineGICPFactorGPUKernel::bind_target_handle(std::shared_ptr<const iap::ISharedTargetHandle> target_handle) {
  if (!target_handle || !target_handle->target_snapshot()) {
    throw std::runtime_error("IntegratedBSplineGICPFactorGPUKernel cannot bind a null target handle");
  }

  target_handle_ = std::move(target_handle);
  target_ = target_handle_->target_snapshot();
  target_identity_ = target_handle_->identity();
  target_revision_ = target_handle_->revision();
  const auto& gpu_resources = target_handle_->gpu_resources();
  if (gpu_resources && gpu_resources->target_gpu) {
    target_cpu_points_ = gpu_resources->target_cpu_points;
    target_gpu_ = gpu_resources->target_gpu;
    target_point_count_ = gpu_resources->point_count;
  } else {
    rebuild_target_gpu();
  }
}

void IntegratedBSplineGICPFactorGPUKernel::bind_target_gpu_resources(
  std::shared_ptr<const iap::SharedTargetGpuResources> target_gpu_resources,
  std::shared_ptr<const gtsam_points::iVox> target_snapshot) {
  target_handle_.reset();
  target_ = std::move(target_snapshot);
  target_identity_ = target_gpu_resources->identity;
  target_revision_ = target_gpu_resources->revision;
  target_cpu_points_ = target_gpu_resources->target_cpu_points;
  target_gpu_ = target_gpu_resources->target_gpu;
  target_point_count_ = target_gpu_resources->point_count;

  if (!target_gpu_) {
    if (!target_) {
      throw std::runtime_error(
        "IntegratedBSplineGICPFactorGPUKernel requires a target snapshot when rebuilding GPU resources");
    }
    rebuild_target_gpu();
  }
}

void IntegratedBSplineGICPFactorGPUKernel::rebuild_target_gpu() {
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
}

bool IntegratedBSplineGICPFactorGPUKernel::target_matches(const iap::ISharedTargetHandle& target_handle) const {
  return target_identity_ == target_handle.identity() && target_revision_ == target_handle.revision();
}

void IntegratedBSplineGICPFactorGPUKernel::rebind_target(std::shared_ptr<const iap::ISharedTargetHandle> target_handle) {
  bind_target_handle(std::move(target_handle));
  last_profile_ = BSplineLidarFactorProfile();
  last_inlier_count_ = 0;
}

IntegratedBSplineGICPFactorGPUKernel::EvaluationResult IntegratedBSplineGICPFactorGPUKernel::evaluate(
  const gtsam::Values& values,
  bool compute_hessian) const {
  if (compute_hessian && jacobian_mode_ != IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC) {
    throw std::runtime_error(
      "IntegratedBSplineGICPFactorGPUKernel only supports SEMI_ANALYTIC runtime Jacobians; use check_against_numeric_full for numeric auditing");
  }

  EvaluationResult eval;
  const auto t_start = std::chrono::steady_clock::now();
  const auto poses = control_poses(values);
  last_control_poses_ = poses;
  last_control_poses_valid_ = true;

  DeviceControlSet control_set;
  Eigen::Quaternionf reference(poses[0].rotation().toQuaternion().cast<float>());
  reference.normalize();
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    const Eigen::Vector4f aligned_coeffs = aligned_quaternion_coeffs(poses[i], reference);
    control_set.poses[i].qx = aligned_coeffs[0];
    control_set.poses[i].qy = aligned_coeffs[1];
    control_set.poses[i].qz = aligned_coeffs[2];
    control_set.poses[i].qw = aligned_coeffs[3];
    const auto translation = poses[i].translation().cast<float>();
    control_set.poses[i].tx = translation.x();
    control_set.poses[i].ty = translation.y();
    control_set.poses[i].tz = translation.z();
    const Eigen::Matrix3f R = poses[i].rotation().matrix().cast<float>();
    std::memcpy(control_set.poses[i].rotation, R.data(), sizeof(float) * 9);
  }

  ensure_source_gpu();

  throw_if_cuda_error(cudaMemsetAsync(linearized_hessian_gpu_, 0, sizeof(float) * kStateDim * kStateDim, stream_), "cudaMemset linearized_hessian_gpu");
  throw_if_cuda_error(cudaMemsetAsync(linearized_gradient_gpu_, 0, sizeof(float) * kStateDim, stream_), "cudaMemset linearized_gradient_gpu");
  throw_if_cuda_error(cudaMemsetAsync(kernel_stats_gpu_, 0, sizeof(DeviceKernelStats), stream_), "cudaMemset kernel_stats_gpu");
  throw_if_cuda_error(
    cudaMemsetAsync(matched_target_indices_gpu_, 0xFF, sizeof(int) * std::max<std::size_t>(1, frame::size(*source_)), stream_),
    "cudaMemset matched_target_indices_gpu");

  cudaEvent_t kernel_start;
  cudaEvent_t kernel_end;
  throw_if_cuda_error(cudaEventCreate(&kernel_start), "cudaEventCreate kernel_start");
  throw_if_cuda_error(cudaEventCreate(&kernel_end), "cudaEventCreate kernel_end");
  throw_if_cuda_error(cudaEventRecord(kernel_start, stream_), "cudaEventRecord kernel_start");

  const int num_points = static_cast<int>(frame::size(*source_));
  const int num_blocks = std::max(1, (num_points + kThreadsPerBlock - 1) / kThreadsPerBlock);
  evaluate_ct_lidar_kernel<<<num_blocks, kThreadsPerBlock, 0, stream_>>>(
    control_set,
    normalized_times_gpu_,
    source_gpu_->points_gpu,
    source_gpu_->covs_gpu,
    num_points,
    target_gpu_->voxelmap_info_ptr,
    target_gpu_->buckets,
    target_gpu_->voxel_means,
    target_gpu_->voxel_covs,
    static_cast<float>(max_correspondence_distance_),
    static_cast<float>(max_correspondence_distance_sq_),
    correspondence_candidate_count_,
    static_cast<float>(correspondence_accept_ratio_),
    static_cast<float>(correspondence_min_score_gap_),
    static_cast<float>(outlier_mahalanobis_threshold_),
    static_cast<int>(robust_kernel_),
    static_cast<float>(robust_kernel_width_),
    static_cast<float>(robust_weight_floor_),
    compute_hessian,
    linearized_hessian_gpu_,
    linearized_gradient_gpu_,
    kernel_stats_gpu_,
    matched_target_indices_gpu_);
  throw_if_cuda_error(cudaGetLastError(), "evaluate_ct_lidar_kernel launch");
  throw_if_cuda_error(cudaEventRecord(kernel_end, stream_), "cudaEventRecord kernel_end");

  DeviceKernelStats host_stats;
  throw_if_cuda_error(cudaMemcpyAsync(&host_stats, kernel_stats_gpu_, sizeof(DeviceKernelStats), cudaMemcpyDeviceToHost, stream_), "cudaMemcpyAsync kernel_stats");
  if (enable_profiling_) {
    eval.matched_target_indices.resize(static_cast<std::size_t>(num_points), -1);
    throw_if_cuda_error(
      cudaMemcpyAsync(
        eval.matched_target_indices.data(),
        matched_target_indices_gpu_,
        sizeof(int) * std::max(1, num_points),
        cudaMemcpyDeviceToHost,
        stream_),
      "cudaMemcpyAsync matched_target_indices");
  }
  if (compute_hessian) {
    throw_if_cuda_error(
      cudaMemcpyAsync(eval.H.data(), linearized_hessian_gpu_, sizeof(float) * kStateDim * kStateDim, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync linearized_hessian");
    throw_if_cuda_error(
      cudaMemcpyAsync(eval.b.data(), linearized_gradient_gpu_, sizeof(float) * kStateDim, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync linearized_gradient");
  }

  throw_if_cuda_error(cudaStreamSynchronize(stream_), "cudaStreamSynchronize kernel evaluation");
  float kernel_ms = 0.0f;
  throw_if_cuda_error(cudaEventElapsedTime(&kernel_ms, kernel_start, kernel_end), "cudaEventElapsedTime");
  cudaEventDestroy(kernel_start);
  cudaEventDestroy(kernel_end);

  const auto t_after_sync = std::chrono::steady_clock::now();
  const auto t_pack_start = t_after_sync;

  eval.valid = true;
  eval.total_error = host_stats.total_error;
  eval.matched_count = host_stats.matched_count;
  eval.inlier_count = host_stats.inlier_count;
  eval.rejected_distance_count = host_stats.rejected_distance_count;
  eval.rejected_ambiguity_count = host_stats.rejected_ambiguity_count;
  eval.rejected_outlier_count = host_stats.rejected_outlier_count;
  eval.rejected_robust_count = host_stats.rejected_robust_count;
  eval.candidate_evaluation_count = host_stats.candidate_evaluation_count;
  eval.comparative_score_count = host_stats.comparative_score_count;
  eval.mean_robust_weight =
    host_stats.inlier_count == 0 ? 1.0 : static_cast<double>(host_stats.sum_robust_weight) / static_cast<double>(host_stats.inlier_count);
  eval.mean_match_distance =
    host_stats.matched_count == 0 ? 0.0 : static_cast<double>(host_stats.sum_match_distance) / static_cast<double>(host_stats.matched_count);
  eval.max_match_distance = host_stats.max_match_distance;
  eval.mean_match_score =
    host_stats.matched_count == 0 ? 0.0 : static_cast<double>(host_stats.sum_match_score) / static_cast<double>(host_stats.matched_count);
  eval.mean_score_gap =
    host_stats.comparative_score_count == 0
      ? 0.0
      : static_cast<double>(host_stats.sum_score_gap) / static_cast<double>(host_stats.comparative_score_count);
  eval.mean_score_ratio =
    host_stats.comparative_score_count == 0
      ? 0.0
      : static_cast<double>(host_stats.sum_score_ratio) / static_cast<double>(host_stats.comparative_score_count);

  const auto t_pack_end = std::chrono::steady_clock::now();
  eval.kernel_correspondence_ms = static_cast<double>(kernel_ms);
  eval.host_sync_ms = std::chrono::duration<double, std::milli>(t_after_sync - t_start).count() - eval.kernel_correspondence_ms;
  eval.host_result_pack_ms = std::chrono::duration<double, std::milli>(t_pack_end - t_pack_start).count();
  eval.total_ms = std::chrono::duration<double, std::milli>(t_pack_end - t_start).count();
  return eval;
}

void IntegratedBSplineGICPFactorGPUKernel::update_profile(const EvaluationResult& eval, const char* stage) const {
  auto profile = make_bspline_lidar_minimal_profile(
    BSplineLidarFactorBackend::GPU_GICP,
    static_cast<std::size_t>(frame::size(*source_)),
    target_point_count_,
    static_cast<std::size_t>(std::max(eval.matched_count, 0)),
    static_cast<std::size_t>(std::max(eval.inlier_count, 0)),
    stage);
  profile.minimal = !enable_profiling_;
  profile.backend = BSplineLidarFactorBackend::GPU_GICP;
  profile.time_bucket_count = time_table_.size();
  profile.max_time_bucket_population =
    time_bucket_populations_.empty() ? 0U : *std::max_element(time_bucket_populations_.begin(), time_bucket_populations_.end());
  profile.mean_time_bucket_population =
    time_bucket_populations_.empty()
      ? 0.0
      : static_cast<double>(frame::size(*source_)) / static_cast<double>(time_bucket_populations_.size());
  profile.candidate_evaluation_count = static_cast<std::size_t>(std::max(eval.candidate_evaluation_count, 0));
  profile.matched_point_count = static_cast<std::size_t>(std::max(eval.matched_count, 0));
  profile.inlier_point_count = static_cast<std::size_t>(std::max(eval.inlier_count, 0));
  profile.rejected_distance_count = static_cast<std::size_t>(std::max(eval.rejected_distance_count, 0));
  profile.rejected_ambiguity_count = static_cast<std::size_t>(std::max(eval.rejected_ambiguity_count, 0));
  profile.rejected_outlier_count = static_cast<std::size_t>(std::max(eval.rejected_outlier_count, 0));
  profile.rejected_robust_count = static_cast<std::size_t>(std::max(eval.rejected_robust_count, 0));
  profile.match_ratio =
    profile.source_point_count == 0 ? 0.0 : static_cast<double>(profile.matched_point_count) / static_cast<double>(profile.source_point_count);
  profile.inlier_ratio =
    profile.source_point_count == 0 ? 0.0 : static_cast<double>(profile.inlier_point_count) / static_cast<double>(profile.source_point_count);
  profile.mean_candidates_per_source =
    profile.source_point_count == 0
      ? 0.0
      : static_cast<double>(profile.candidate_evaluation_count) / static_cast<double>(profile.source_point_count);
  profile.mean_robust_weight = eval.mean_robust_weight;
  profile.mean_match_distance = eval.mean_match_distance;
  profile.max_match_distance = eval.max_match_distance;
  profile.mean_match_score = eval.mean_match_score;
  profile.mean_score_gap = eval.mean_score_gap;
  profile.mean_score_ratio = eval.mean_score_ratio;
  profile.comparative_score_count = static_cast<std::size_t>(std::max(eval.comparative_score_count, 0));
  profile.pose_update_ms = eval.kernel_pose_query_ms;
  profile.correspondence_ms = eval.kernel_correspondence_ms;
  profile.accumulation_ms =
    eval.kernel_residual_weight_ms + eval.kernel_reduction_ms + eval.host_sync_ms;
  profile.total_ms = eval.total_ms;
  profile.kernel_pose_query_ms = eval.kernel_pose_query_ms;
  profile.kernel_correspondence_ms = eval.kernel_correspondence_ms;
  profile.kernel_residual_weight_ms = eval.kernel_residual_weight_ms;
  profile.kernel_reduction_ms = eval.kernel_reduction_ms;
  profile.host_sync_ms = eval.host_sync_ms;
  profile.host_result_pack_ms = eval.host_result_pack_ms;
  profile.total_error = eval.total_error;

  if (enable_profiling_ && !eval.matched_target_indices.empty()) {
    std::unordered_map<int, std::size_t> reuse_counts;
    for (const int target_index : eval.matched_target_indices) {
      if (target_index < 0) {
        continue;
      }
      reuse_counts[target_index]++;
    }
    profile.unique_target_count = reuse_counts.size();
    for (const auto& [_, reuse] : reuse_counts) {
      profile.max_target_reuse = std::max(profile.max_target_reuse, reuse);
    }
    if (profile.matched_point_count > 0) {
      profile.unique_target_ratio =
        static_cast<double>(profile.unique_target_count) / static_cast<double>(profile.matched_point_count);
      profile.max_target_reuse_ratio =
        static_cast<double>(profile.max_target_reuse) / static_cast<double>(profile.matched_point_count);
    }
    profile.minimal = false;
  }

  last_profile_ = profile;
  last_inlier_count_ = eval.inlier_count;
}

std::shared_ptr<gtsam::HessianFactor> IntegratedBSplineGICPFactorGPUKernel::make_hessian_factor(
  const EvaluationResult& eval) const {
  std::vector<gtsam::DenseIndex> dims(kBSplineControlPointCount + 1, 6);
  dims.back() = 1;
  gtsam::SymmetricBlockMatrix augmented(dims);
  for (std::size_t a = 0; a < kBSplineControlPointCount; ++a) {
    const int row = static_cast<int>(a) * 6;
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(a), eval.H.block<6, 6>(row, row).cast<double>());
    for (std::size_t c = a + 1; c < kBSplineControlPointCount; ++c) {
      const int col = static_cast<int>(c) * 6;
      augmented.setOffDiagonalBlock(
        static_cast<gtsam::DenseIndex>(a),
        static_cast<gtsam::DenseIndex>(c),
        eval.H.block<6, 6>(row, col).cast<double>());
    }
    augmented.setOffDiagonalBlock(
      static_cast<gtsam::DenseIndex>(a),
      static_cast<gtsam::DenseIndex>(kBSplineControlPointCount),
      -eval.b.segment<6>(row).cast<double>());
  }
  augmented.setDiagonalBlock(
    static_cast<gtsam::DenseIndex>(kBSplineControlPointCount),
    Eigen::Matrix<double, 1, 1>::Constant(eval.total_error));
  return std::make_shared<gtsam::HessianFactor>(keys_, augmented);
}

std::shared_ptr<IntegratedBSplineGICPFactor> IntegratedBSplineGICPFactorGPUKernel::make_cpu_reference_factor(
  IntegratedBSplineGICPFactor::JacobianMode mode) const {
  std::array<gtsam::Key, kBSplineControlPointCount> keys{};
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    keys[i] = keys_[i];
  }
  auto factor = std::make_shared<IntegratedBSplineGICPFactor>(keys, target_, source_, target_);
  factor->set_enable_profiling(false);
  factor->set_jacobian_mode(mode);
  factor->set_numeric_eps(numeric_eps_);
  factor->set_max_correspondence_distance(max_correspondence_distance_);
  factor->set_correspondence_candidate_count(correspondence_candidate_count_);
  factor->set_correspondence_accept_ratio(correspondence_accept_ratio_);
  factor->set_correspondence_min_score_gap(correspondence_min_score_gap_);
  factor->set_outlier_mahalanobis_threshold(outlier_mahalanobis_threshold_);
  factor->set_robust_kernel(robust_kernel_, robust_kernel_width_);
  factor->set_robust_weight_floor(robust_weight_floor_);
  return factor;
}

double IntegratedBSplineGICPFactorGPUKernel::error(const gtsam::Values& values) const {
  const auto eval = evaluate(values, false);
  update_profile(eval, "gpu_kernel_error");
  return eval.total_error;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineGICPFactorGPUKernel::linearize(const gtsam::Values& values) const {
  const auto eval = evaluate(values, true);
  update_profile(eval, "gpu_kernel_linearized");
  return make_hessian_factor(eval);
}

double IntegratedBSplineGICPFactorGPUKernel::inlier_fraction() const {
  const auto source_size = static_cast<std::size_t>(std::max(frame::size(*source_), 0));
  return source_size == 0 ? 0.0 : static_cast<double>(last_inlier_count_) / static_cast<double>(source_size);
}

BSplineLidarFactorProfile IntegratedBSplineGICPFactorGPUKernel::profiling_report() const {
  return last_profile_;
}

BSplineLidarNumericAudit IntegratedBSplineGICPFactorGPUKernel::check_against_numeric_full(
  const gtsam::Values& values,
  double perturbation_scale) const {
  BSplineLidarNumericAudit result;
  result.perturbation_scale = perturbation_scale;
  if (perturbation_scale <= 0.0) {
    return result;
  }

  const auto numeric_factor = make_cpu_reference_factor(IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL);
  const auto numeric_linear = numeric_factor->linearize(values);
  const auto kernel_linear = linearize(values);
  if (!numeric_linear || !kernel_linear) {
    return result;
  }

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
  const auto measure_actual_error = [&](const gtsam::VectorValues& delta) {
    const gtsam::Values perturbed = values.retract(delta);
    return error(perturbed);
  };

  result.valid = true;
  result.numeric_rotation_predicted_error = numeric_linear->error(rotation_delta);
  result.semi_rotation_predicted_error = kernel_linear->error(rotation_delta);
  result.rotation_actual_error = measure_actual_error(rotation_delta);
  result.rotation_abs_error = std::abs(result.semi_rotation_predicted_error - result.numeric_rotation_predicted_error);
  result.rotation_rel_error =
    result.rotation_abs_error /
    std::max(1e-9, std::max(std::abs(result.semi_rotation_predicted_error), std::abs(result.numeric_rotation_predicted_error)));

  result.numeric_translation_predicted_error = numeric_linear->error(translation_delta);
  result.semi_translation_predicted_error = kernel_linear->error(translation_delta);
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
    const double axis_kernel = kernel_linear->error(axis_delta);
    result.axis_rotation_rel_error[axis] =
      std::abs(axis_kernel - axis_numeric) / std::max(1e-9, std::max(std::abs(axis_kernel), std::abs(axis_numeric)));
    result.mean_rotation_axis_rel_error += result.axis_rotation_rel_error[axis];
    if (result.axis_rotation_rel_error[axis] > result.max_rotation_axis_rel_error) {
      result.max_rotation_axis_rel_error = result.axis_rotation_rel_error[axis];
      result.worst_rotation_axis = axis;
    }
  }
  result.mean_rotation_axis_rel_error /= 3.0;
  return result;
}

BSplineLidarDegeneracyReport IntegratedBSplineGICPFactorGPUKernel::diagnose_degeneracy(
  const IntegratedBSplineGICPFactor::DegeneracyThresholds& thresholds) const {
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
    thresholds.min_unique_target_ratio > 0.0 &&
    last_profile_.unique_target_count > 0 &&
    last_profile_.unique_target_ratio < thresholds.min_unique_target_ratio;
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

BSplineLidarFactorResult IntegratedBSplineGICPFactorGPUKernel::make_result(
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

std::vector<Eigen::Vector4d> IntegratedBSplineGICPFactorGPUKernel::deskewed_source_points(
  const gtsam::Values& values,
  bool local) const {
  const auto poses = control_poses(values);
  std::vector<Eigen::Vector4d> points;
  points.reserve(frame::size(*source_));
  const gtsam::Pose3 reference = BSplineControlWindow::interpolate(poses, 0.0);

  const double time_min = time_table_.empty() ? 0.0 : time_table_.front();
  const double time_max = time_table_.empty() ? 1.0 : time_table_.back();
  (void)time_min;
  (void)time_max;

  for (int i = 0; i < frame::size(*source_); ++i) {
    const double u = normalized_times_[static_cast<std::size_t>(i)];
    const auto pose = BSplineControlWindow::interpolate(poses, u);
    const auto world_pt = pose.transformFrom(frame::point(*source_, i).template head<3>().eval());
    Eigen::Vector4d pt = Eigen::Vector4d::Ones();
    pt.head<3>() = local ? reference.transformTo(world_pt) : world_pt;
    points.push_back(pt);
  }
  return points;
}

}  // namespace iap

#endif  // GTSAM_POINTS_USE_CUDA
