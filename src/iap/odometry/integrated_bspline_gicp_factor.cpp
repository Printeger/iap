#include <iap/odometry/integrated_bspline_gicp_factor.hpp>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <unordered_map>

namespace iap {

namespace frame = gtsam_points::frame;

namespace {

double residual_norm_from_mahalanobis(const double mahalanobis_error) {
  return std::sqrt(std::max(0.0, mahalanobis_error));
}

Eigen::Matrix<double, 4, 3> quaternion_right_local_jacobian(const Eigen::Quaterniond& q) {
  Eigen::Matrix<double, 4, 3> J = Eigen::Matrix<double, 4, 3>::Zero();
  J.block<3, 3>(0, 0) = 0.5 * (q.w() * Eigen::Matrix3d::Identity() + gtsam::SO3::Hat(q.vec()));
  J.row(3) = -0.5 * q.vec().transpose();
  return J;
}

Eigen::Matrix4d normalized_vector_jacobian(const Eigen::Vector4d& coeffs) {
  const double norm = coeffs.norm();
  if (norm < 1e-9) {
    return Eigen::Matrix4d::Zero();
  }

  const Eigen::Vector4d normalized = coeffs / norm;
  return (Eigen::Matrix4d::Identity() - normalized * normalized.transpose()) / norm;
}

Eigen::Matrix<double, 3, 4> rotated_point_quaternion_jacobian(
  const Eigen::Vector4d& coeffs,
  const Eigen::Vector3d& point) {
  Eigen::Matrix<double, 3, 4> J = Eigen::Matrix<double, 3, 4>::Zero();

  const Eigen::Vector3d v = coeffs.head<3>();
  const double w = coeffs[3];
  const double vp = v.dot(point);

  for (int j = 0; j < 3; ++j) {
    const Eigen::Vector3d e = Eigen::Vector3d::Unit(j);
    J.col(j) = -2.0 * v[j] * point + 2.0 * e * vp + 2.0 * v * point[j] + 2.0 * w * e.cross(point);
  }
  J.col(3) = 2.0 * w * point + 2.0 * v.cross(point);

  return J;
}

}  // namespace

IntegratedBSplineGICPFactor::IntegratedBSplineGICPFactor(
  const std::array<gtsam::Key, kBSplineControlPointCount>& keys,
  const std::shared_ptr<const gtsam_points::iVox>& target,
  const std::shared_ptr<const gtsam_points::PointCloud>& source,
  const std::shared_ptr<const gtsam_points::NearestNeighborSearch>& target_tree)
: gtsam::NonlinearFactor(gtsam::KeyVector(keys.begin(), keys.end())),
  target_(target),
  source_(source) {
  if (!frame::has_points(*target_) || !frame::has_covs(*target_)) {
    throw std::runtime_error("IntegratedBSplineGICPFactor requires target points and covariances");
  }
  if (!frame::has_points(*source_) || !frame::has_covs(*source_) || !frame::has_times(*source_)) {
    throw std::runtime_error("IntegratedBSplineGICPFactor requires source points, covariances, and times");
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
  time_bucket_populations_.assign(time_table_.size(), 0);
  for (const int time_index : time_indices_) {
    time_bucket_populations_[static_cast<std::size_t>(time_index)]++;
  }

  const double time_min = time_table_.empty() ? 0.0 : time_table_.front();
  const double time_max = time_table_.empty() ? 1.0 : time_table_.back();
  const double denom = std::max(1e-9, time_max - time_min);
  for (auto& t : time_table_) {
    t = (t - time_min) / denom;
  }
  basis_table_.reserve(time_table_.size());
  for (const double t : time_table_) {
    basis_table_.push_back(BSplineControlWindow::basis(t));
  }

  if (target_tree) {
    target_tree_ = target_tree;
  } else {
    target_tree_ = target_;
  }
}

std::array<gtsam::Pose3, kBSplineControlPointCount> IntegratedBSplineGICPFactor::control_poses(const gtsam::Values& values) const {
  std::array<gtsam::Pose3, kBSplineControlPointCount> poses;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    poses[i] = values.at<gtsam::Pose3>(keys_[i]);
  }
  return poses;
}

double IntegratedBSplineGICPFactor::inlier_fraction() const {
  return frame::size(*source_) == 0 ? 0.0 : static_cast<double>(accepted_inlier_count_) / frame::size(*source_);
}

void IntegratedBSplineGICPFactor::update_poses(const gtsam::Values& values) const {
  const auto poses = control_poses(values);

  source_poses_.resize(time_table_.size());
  if (jacobian_mode_ == JacobianMode::NUMERIC_FULL) {
    pose_jacobians_.resize(time_table_.size());
    blended_quaternion_coeffs_.clear();
    blend_quaternion_jacobians_.clear();
  } else {
    pose_jacobians_.clear();
    blended_quaternion_coeffs_.resize(time_table_.size());
    blend_quaternion_jacobians_.resize(time_table_.size());
  }

  for (std::size_t i = 0; i < time_table_.size(); ++i) {
    const double u = time_table_[i];
    const gtsam::Pose3 base_pose = BSplineControlWindow::interpolate(poses, u);
    source_poses_[i] = base_pose;

    if (jacobian_mode_ != JacobianMode::NUMERIC_FULL) {
      const auto& weights = basis_table_[i];
      Eigen::Quaterniond reference(poses[0].rotation().toQuaternion());
      reference.normalize();

      std::array<Eigen::Quaterniond, kBSplineControlPointCount> aligned;
      Eigen::Vector4d blended_coeffs = Eigen::Vector4d::Zero();
      for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
        Eigen::Quaterniond q(poses[k].rotation().toQuaternion());
        q.normalize();
        if (reference.dot(q) < 0.0) {
          q.coeffs() *= -1.0;
        }
        aligned[k] = q;
        blended_coeffs += weights[k] * q.coeffs();
      }

      Eigen::Vector4d normalized_coeffs = blended_coeffs;
      Eigen::Matrix4d normalization_jacobian = Eigen::Matrix4d::Zero();
      if (blended_coeffs.norm() < 1e-9) {
        normalized_coeffs = aligned[0].coeffs();
      } else {
        normalized_coeffs.normalize();
        normalization_jacobian = normalized_vector_jacobian(blended_coeffs);
      }
      blended_quaternion_coeffs_[i] = normalized_coeffs;

      for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
        blend_quaternion_jacobians_[i][k] =
          normalization_jacobian * (weights[k] * quaternion_right_local_jacobian(aligned[k]));
      }
      continue;
    }

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
      pose_jacobians_[i][k] = J;
    }
  }
}

void IntegratedBSplineGICPFactor::update_correspondences() const {
  correspondences_.resize(frame::size(*source_));
  mahalanobis_.resize(frame::size(*source_));
  correspondence_sq_distances_.resize(frame::size(*source_));
  correspondence_best_scores_.resize(frame::size(*source_));
  correspondence_second_best_scores_.resize(frame::size(*source_));
  matched_correspondence_count_ = 0;
  rejected_distance_count_ = 0;
  rejected_ambiguity_count_ = 0;
  candidate_evaluation_count_ = 0;

  for (int i = 0; i < frame::size(*source_); ++i) {
    const int time_index = time_indices_[static_cast<std::size_t>(i)];
    const auto& pose = source_poses_[static_cast<std::size_t>(time_index)];
    const auto& source_pt = frame::point(*source_, i);

    const gtsam::Point3 transed_pt = pose * source_pt.template head<3>();
    const std::size_t candidate_count = static_cast<std::size_t>(std::max(1, correspondence_candidate_count_));
    std::vector<size_t> k_indices(candidate_count, static_cast<size_t>(-1));
    std::vector<double> k_sq_dists(candidate_count, std::numeric_limits<double>::max());
    const size_t num_found =
      target_tree_->knn_search(transed_pt.data(), candidate_count, k_indices.data(), k_sq_dists.data(), max_correspondence_distance_sq_);

    if (num_found == 0) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      correspondence_sq_distances_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      correspondence_best_scores_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      correspondence_second_best_scores_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      rejected_distance_count_++;
      continue;
    }

    const auto& source_cov = frame::cov(*source_, i);
    const Eigen::Matrix4d pose_matrix = pose.matrix();
    double best_score = std::numeric_limits<double>::max();
    double second_best_score = std::numeric_limits<double>::max();
    double best_sq_dist = std::numeric_limits<double>::max();
    long best_index = -1;
    Eigen::Matrix3d best_mahalanobis = Eigen::Matrix3d::Zero();

    for (std::size_t c = 0; c < num_found; ++c) {
      if (k_sq_dists[c] > max_correspondence_distance_sq_) {
        continue;
      }

      const long target_index = static_cast<long>(k_indices[c]);
      const auto& target_cov = frame::cov(*target_, target_index);
      Eigen::Matrix3d RCR = (target_cov + pose_matrix * source_cov * pose_matrix.transpose()).block<3, 3>(0, 0);
      RCR.diagonal().array() += 1e-6;
      const Eigen::Matrix3d M = RCR.inverse();
      const Eigen::Vector3d residual = transed_pt - frame::point(*target_, target_index).template head<3>();
      candidate_evaluation_count_++;
      const double score = residual.transpose() * M * residual;

      if (score < best_score) {
        second_best_score = best_score;
        best_score = score;
        best_sq_dist = k_sq_dists[c];
        best_index = target_index;
        best_mahalanobis = M;
      } else if (score < second_best_score) {
        second_best_score = score;
      }
    }

    if (best_index < 0) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      correspondence_sq_distances_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      correspondence_best_scores_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      correspondence_second_best_scores_[static_cast<std::size_t>(i)] = std::numeric_limits<double>::max();
      rejected_distance_count_++;
      continue;
    }

    if (correspondence_accept_ratio_ > 0.0 &&
        second_best_score < std::numeric_limits<double>::max() &&
        second_best_score > 1e-9 &&
        (best_score / second_best_score) >= correspondence_accept_ratio_) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      correspondence_sq_distances_[static_cast<std::size_t>(i)] = best_sq_dist;
      correspondence_best_scores_[static_cast<std::size_t>(i)] = best_score;
      correspondence_second_best_scores_[static_cast<std::size_t>(i)] = second_best_score;
      rejected_ambiguity_count_++;
      continue;
    }

    if (correspondence_min_score_gap_ > 0.0 &&
        second_best_score < std::numeric_limits<double>::max() &&
        (second_best_score - best_score) <= correspondence_min_score_gap_) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      correspondence_sq_distances_[static_cast<std::size_t>(i)] = best_sq_dist;
      correspondence_best_scores_[static_cast<std::size_t>(i)] = best_score;
      correspondence_second_best_scores_[static_cast<std::size_t>(i)] = second_best_score;
      rejected_ambiguity_count_++;
      continue;
    }

    correspondences_[static_cast<std::size_t>(i)] = best_index;
    mahalanobis_[static_cast<std::size_t>(i)] = best_mahalanobis;
    correspondence_sq_distances_[static_cast<std::size_t>(i)] = best_sq_dist;
    correspondence_best_scores_[static_cast<std::size_t>(i)] = best_score;
    correspondence_second_best_scores_[static_cast<std::size_t>(i)] = second_best_score;
    matched_correspondence_count_++;
  }
}

IntegratedBSplineGICPFactor::PointJacobianArray IntegratedBSplineGICPFactor::point_jacobians(
  const std::array<gtsam::Pose3, kBSplineControlPointCount>& control_poses,
  std::size_t time_index,
  const Eigen::Vector3d& source_point) const {
  PointJacobianArray jacobians;

  if (jacobian_mode_ == JacobianMode::NUMERIC_FULL) {
    gtsam::Matrix36 H_transed_pose;
    source_poses_[time_index].transformFrom(source_point, H_transed_pose);

    for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
      jacobians[k] = H_transed_pose * pose_jacobians_[time_index][k];
    }
    return jacobians;
  }

  const auto& weights = basis_table_[time_index];
  const Eigen::Matrix<double, 3, 4> H_transed_quat =
    rotated_point_quaternion_jacobian(blended_quaternion_coeffs_[time_index], source_point);

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    jacobians[k].setZero();
    jacobians[k].block<3, 3>(0, 0) = H_transed_quat * blend_quaternion_jacobians_[time_index][k];
    jacobians[k].block<3, 3>(0, 3) = weights[k] * control_poses[k].rotation().matrix();
  }

  return jacobians;
}

double IntegratedBSplineGICPFactor::robust_cost(const double mahalanobis_error) const {
  const double residual_norm = residual_norm_from_mahalanobis(mahalanobis_error);

  switch (robust_kernel_) {
    case RobustKernel::NONE:
      return mahalanobis_error;
    case RobustKernel::HUBER:
      if (residual_norm <= robust_kernel_width_) {
        return mahalanobis_error;
      }
      return 2.0 * robust_kernel_width_ * residual_norm - robust_kernel_width_ * robust_kernel_width_;
    case RobustKernel::CAUCHY: {
      const double scaled = residual_norm / robust_kernel_width_;
      return robust_kernel_width_ * robust_kernel_width_ * std::log1p(scaled * scaled);
    }
  }

  return mahalanobis_error;
}

double IntegratedBSplineGICPFactor::robust_weight(const double mahalanobis_error) const {
  const double residual_norm = residual_norm_from_mahalanobis(mahalanobis_error);

  switch (robust_kernel_) {
    case RobustKernel::NONE:
      return 1.0;
    case RobustKernel::HUBER:
      if (residual_norm <= robust_kernel_width_) {
        return 1.0;
      }
      return robust_kernel_width_ / std::max(1e-9, residual_norm);
    case RobustKernel::CAUCHY: {
      const double scaled = residual_norm / robust_kernel_width_;
      return 1.0 / (1.0 + scaled * scaled);
    }
  }

  return 1.0;
}

void IntegratedBSplineGICPFactor::update_profiling_stats(
  const char* stage,
  double pose_update_ms,
  double correspondence_ms,
  double accumulation_ms,
  double total_ms,
  double total_error) const {
  last_profile_.valid = true;
  last_profile_.stage = stage;
  last_profile_.source_point_count = frame::size(*source_);
  last_profile_.target_point_count = target_->voxel_points().size();
  last_profile_.time_bucket_count = time_bucket_populations_.size();
  last_profile_.max_time_bucket_population = 0;
  last_profile_.candidate_evaluation_count = candidate_evaluation_count_;
  last_profile_.matched_point_count = matched_correspondence_count_;
  last_profile_.inlier_point_count = accepted_inlier_count_;
  last_profile_.rejected_distance_count = rejected_distance_count_;
  last_profile_.rejected_ambiguity_count = rejected_ambiguity_count_;
  last_profile_.rejected_outlier_count = rejected_outlier_count_;
  last_profile_.rejected_robust_count = rejected_robust_count_;
  last_profile_.match_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(matched_correspondence_count_) / last_profile_.source_point_count;
  last_profile_.inlier_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(accepted_inlier_count_) / last_profile_.source_point_count;
  last_profile_.mean_time_bucket_population =
    last_profile_.time_bucket_count == 0 ? 0.0 : static_cast<double>(last_profile_.source_point_count) / last_profile_.time_bucket_count;
  last_profile_.mean_candidates_per_source =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(candidate_evaluation_count_) / last_profile_.source_point_count;
  last_profile_.mean_robust_weight =
    accepted_inlier_count_ == 0 ? 1.0 : robust_weight_sum_ / static_cast<double>(accepted_inlier_count_);
  last_profile_.pose_update_ms = pose_update_ms;
  last_profile_.correspondence_ms = correspondence_ms;
  last_profile_.accumulation_ms = accumulation_ms;
  last_profile_.total_ms = total_ms;
  last_profile_.total_error = total_error;

  last_profile_.unique_target_count = 0;
  last_profile_.max_target_reuse = 0;
  last_profile_.unique_target_ratio = 0.0;
  last_profile_.max_target_reuse_ratio = 0.0;
  last_profile_.mean_match_distance = 0.0;
  last_profile_.max_match_distance = 0.0;
  last_profile_.mean_match_score = 0.0;
  last_profile_.mean_score_gap = 0.0;
  last_profile_.mean_score_ratio = 0.0;
  last_profile_.comparative_score_count = 0;

  std::unordered_map<long, std::size_t> target_reuse_counts;
  double distance_sum = 0.0;
  double score_sum = 0.0;
  double gap_sum = 0.0;
  double ratio_sum = 0.0;
  std::size_t accepted_score_count = 0;
  std::size_t comparative_score_count = 0;

  for (std::size_t i = 0; i < correspondences_.size(); ++i) {
    const long target_index = correspondences_[i];
    if (target_index >= 0) {
      const double match_distance = std::sqrt(std::max(0.0, correspondence_sq_distances_[i]));
      const double match_score = correspondence_best_scores_[i];
      distance_sum += match_distance;
      score_sum += match_score;
      last_profile_.max_match_distance = std::max(last_profile_.max_match_distance, match_distance);
      target_reuse_counts[target_index]++;
      accepted_score_count++;
    }

    if (correspondence_best_scores_[i] < std::numeric_limits<double>::max() &&
        correspondence_second_best_scores_[i] < std::numeric_limits<double>::max()) {
      const double gap = correspondence_second_best_scores_[i] - correspondence_best_scores_[i];
      const double ratio =
        correspondence_best_scores_[i] / std::max(1e-9, correspondence_second_best_scores_[i]);
      gap_sum += gap;
      ratio_sum += ratio;
      comparative_score_count++;
    }
  }
  for (const std::size_t population : time_bucket_populations_) {
    last_profile_.max_time_bucket_population = std::max(last_profile_.max_time_bucket_population, population);
  }

  last_profile_.unique_target_count = target_reuse_counts.size();
  for (const auto& [_, reuse_count] : target_reuse_counts) {
    last_profile_.max_target_reuse = std::max(last_profile_.max_target_reuse, reuse_count);
  }

  if (matched_correspondence_count_ > 0) {
    last_profile_.unique_target_ratio =
      static_cast<double>(last_profile_.unique_target_count) / static_cast<double>(matched_correspondence_count_);
    last_profile_.max_target_reuse_ratio =
      static_cast<double>(last_profile_.max_target_reuse) / static_cast<double>(matched_correspondence_count_);
  }
  if (accepted_score_count > 0) {
    last_profile_.mean_match_distance = distance_sum / static_cast<double>(accepted_score_count);
    last_profile_.mean_match_score = score_sum / static_cast<double>(accepted_score_count);
  }
  if (comparative_score_count > 0) {
    last_profile_.comparative_score_count = comparative_score_count;
    last_profile_.mean_score_gap = gap_sum / static_cast<double>(comparative_score_count);
    last_profile_.mean_score_ratio = ratio_sum / static_cast<double>(comparative_score_count);
  }
}

double IntegratedBSplineGICPFactor::error(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  update_poses(values);
  const auto t_pose = Clock::now();
  update_correspondences();
  const auto t_corr = Clock::now();

  accepted_inlier_count_ = 0;
  rejected_outlier_count_ = 0;
  rejected_robust_count_ = 0;
  robust_weight_sum_ = 0.0;
  double total_error = 0.0;
  for (int i = 0; i < frame::size(*source_); ++i) {
    const long target_index = correspondences_[static_cast<std::size_t>(i)];
    if (target_index < 0) {
      continue;
    }

    const int time_index = time_indices_[static_cast<std::size_t>(i)];
    const auto& pose = source_poses_[static_cast<std::size_t>(time_index)];

    const auto& source_pt = frame::point(*source_, i);
    const auto& target_pt = frame::point(*target_, target_index);
    const gtsam::Point3 transed_source_pt = pose.transformFrom(source_pt.template head<3>().eval());
    const Eigen::Vector3d residual = transed_source_pt - target_pt.template head<3>();
    const double mahalanobis_error = residual.transpose() * mahalanobis_[static_cast<std::size_t>(i)] * residual;
    if (outlier_mahalanobis_threshold_ > 0.0 &&
        residual_norm_from_mahalanobis(mahalanobis_error) > outlier_mahalanobis_threshold_) {
      rejected_outlier_count_++;
      continue;
    }

    const double weight = robust_weight(mahalanobis_error);
    if (robust_weight_floor_ > 0.0 && weight < robust_weight_floor_) {
      rejected_robust_count_++;
      continue;
    }

    accepted_inlier_count_++;
    robust_weight_sum_ += weight;
    total_error += robust_cost(mahalanobis_error);
  }

  if (enable_profiling_) {
    const auto t_end = Clock::now();
    update_profiling_stats(
      "error",
      std::chrono::duration<double, std::milli>(t_pose - t_start).count(),
      std::chrono::duration<double, std::milli>(t_corr - t_pose).count(),
      std::chrono::duration<double, std::milli>(t_end - t_corr).count(),
      std::chrono::duration<double, std::milli>(t_end - t_start).count(),
      total_error);
  }

  return total_error;
}

gtsam::GaussianFactor::shared_ptr IntegratedBSplineGICPFactor::linearize(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  update_poses(values);
  const auto t_pose = Clock::now();
  update_correspondences();
  const auto t_corr = Clock::now();
  const auto poses = control_poses(values);

  std::array<std::array<gtsam::Matrix6, kBSplineControlPointCount>, kBSplineControlPointCount> H;
  std::array<gtsam::Vector6, kBSplineControlPointCount> b;
  for (auto& row : H) {
    for (auto& block : row) {
      block.setZero();
    }
  }
  for (auto& bi : b) {
    bi.setZero();
  }

  accepted_inlier_count_ = 0;
  rejected_outlier_count_ = 0;
  rejected_robust_count_ = 0;
  robust_weight_sum_ = 0.0;
  double total_error = 0.0;
  for (int i = 0; i < frame::size(*source_); ++i) {
    const long target_index = correspondences_[static_cast<std::size_t>(i)];
    if (target_index < 0) {
      continue;
    }

    const int time_index = time_indices_[static_cast<std::size_t>(i)];
    const auto& pose = source_poses_[static_cast<std::size_t>(time_index)];

    const auto& source_pt = frame::point(*source_, i);
    const auto& target_pt = frame::point(*target_, target_index);

    const gtsam::Point3 transed_source_pt = pose.transformFrom(source_pt.template head<3>().eval());
    const Eigen::Vector3d residual = transed_source_pt - target_pt.template head<3>();
    const Eigen::Matrix3d& M = mahalanobis_[static_cast<std::size_t>(i)];
    const double mahalanobis_error = residual.transpose() * M * residual;
    if (outlier_mahalanobis_threshold_ > 0.0 &&
        residual_norm_from_mahalanobis(mahalanobis_error) > outlier_mahalanobis_threshold_) {
      rejected_outlier_count_++;
      continue;
    }

    const double weight = robust_weight(mahalanobis_error);
    if (robust_weight_floor_ > 0.0 && weight < robust_weight_floor_) {
      rejected_robust_count_++;
      continue;
    }

    accepted_inlier_count_++;
    robust_weight_sum_ += weight;

    const auto point_jacs = point_jacobians(poses, static_cast<std::size_t>(time_index), source_pt.template head<3>().eval());
    for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
      b[k] += weight * point_jacs[k].transpose() * M * residual;
    }

    for (std::size_t a = 0; a < kBSplineControlPointCount; ++a) {
      for (std::size_t c = a; c < kBSplineControlPointCount; ++c) {
        H[a][c] += weight * point_jacs[a].transpose() * M * point_jacs[c];
      }
    }

    total_error += robust_cost(mahalanobis_error);
  }

  std::vector<gtsam::DenseIndex> dims(kBSplineControlPointCount + 1, 6);
  dims.back() = 1;
  gtsam::SymmetricBlockMatrix augmented(dims);

  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(i), H[i][i]);
    for (std::size_t j = i + 1; j < kBSplineControlPointCount; ++j) {
      augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(j), H[i][j]);
    }
    augmented.setOffDiagonalBlock(static_cast<gtsam::DenseIndex>(i), static_cast<gtsam::DenseIndex>(kBSplineControlPointCount), -b[i]);
  }
  augmented.setDiagonalBlock(static_cast<gtsam::DenseIndex>(kBSplineControlPointCount), Eigen::Matrix<double, 1, 1>::Constant(total_error));

  if (enable_profiling_) {
    const auto t_end = Clock::now();
    update_profiling_stats(
      "linearize",
      std::chrono::duration<double, std::milli>(t_pose - t_start).count(),
      std::chrono::duration<double, std::milli>(t_corr - t_pose).count(),
      std::chrono::duration<double, std::milli>(t_end - t_corr).count(),
      std::chrono::duration<double, std::milli>(t_end - t_start).count(),
      total_error);
  }

  return std::make_shared<gtsam::HessianFactor>(keys_, augmented);
}

IntegratedBSplineGICPFactor::LinearizationCheckResult IntegratedBSplineGICPFactor::check_linearization(
  const gtsam::Values& values,
  double perturbation_scale) const {
  LinearizationCheckResult result;
  result.perturbation_scale = perturbation_scale;

  if (perturbation_scale <= 0.0) {
    return result;
  }

  const auto gaussian = std::dynamic_pointer_cast<gtsam::HessianFactor>(linearize(values));
  if (!gaussian) {
    return result;
  }

  gtsam::VectorValues delta;
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    gtsam::Vector6 d;
    d << 1.0, -0.7, 0.5, -0.3, 0.2, -0.1;
    d *= perturbation_scale * (1.0 + 0.15 * static_cast<double>(i));
    delta.insert(keys_[i], d);
  }

  const gtsam::Values perturbed = values.retract(delta);
  result.valid = true;
  result.base_error = error(values);
  result.predicted_error = gaussian->error(delta);
  result.actual_error = error(perturbed);
  result.abs_error = std::abs(result.actual_error - result.predicted_error);
  result.rel_error =
    result.abs_error / std::max(1e-9, std::max(std::abs(result.actual_error), std::abs(result.predicted_error)));
  return result;
}

IntegratedBSplineGICPFactor::NumericReferenceCheckResult IntegratedBSplineGICPFactor::check_against_numeric_full(
  const gtsam::Values& values,
  double perturbation_scale) const {
  NumericReferenceCheckResult result;
  result.perturbation_scale = perturbation_scale;

  if (perturbation_scale <= 0.0) {
    return result;
  }

  std::array<gtsam::Key, kBSplineControlPointCount> keys{};
  for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
    keys[i] = keys_[i];
  }

  auto configure_factor = [&](JacobianMode mode) {
    IntegratedBSplineGICPFactor factor(keys, target_, source_, target_tree_);
    factor.set_max_correspondence_distance(std::sqrt(max_correspondence_distance_sq_));
    factor.set_enable_profiling(false);
    factor.set_jacobian_mode(mode);
    factor.set_numeric_eps(numeric_eps_);
    factor.set_correspondence_candidate_count(correspondence_candidate_count_);
    factor.set_correspondence_accept_ratio(correspondence_accept_ratio_);
    factor.set_correspondence_min_score_gap(correspondence_min_score_gap_);
    factor.set_outlier_mahalanobis_threshold(outlier_mahalanobis_threshold_);
    factor.set_robust_kernel(robust_kernel_, robust_kernel_width_);
    factor.set_robust_weight_floor(robust_weight_floor_);
    return factor;
  };

  auto numeric_factor = configure_factor(JacobianMode::NUMERIC_FULL);
  auto semi_factor = configure_factor(JacobianMode::SEMI_ANALYTIC);

  const auto numeric_linear = numeric_factor.linearize(values);
  const auto semi_linear = semi_factor.linearize(values);
  if (!numeric_linear || !semi_linear) {
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
  const auto rotation_perturbed = values.retract(rotation_delta);
  const auto translation_perturbed = values.retract(translation_delta);

  result.valid = true;
  result.numeric_rotation_predicted_error = numeric_linear->error(rotation_delta);
  result.semi_rotation_predicted_error = semi_linear->error(rotation_delta);
  result.rotation_actual_error = semi_factor.error(rotation_perturbed);
  result.rotation_abs_error = std::abs(result.semi_rotation_predicted_error - result.numeric_rotation_predicted_error);
  result.rotation_rel_error = result.rotation_abs_error /
                              std::max(1e-9,
                                       std::max(std::abs(result.semi_rotation_predicted_error),
                                                std::abs(result.numeric_rotation_predicted_error)));

  result.numeric_translation_predicted_error = numeric_linear->error(translation_delta);
  result.semi_translation_predicted_error = semi_linear->error(translation_delta);
  result.translation_actual_error = semi_factor.error(translation_perturbed);
  result.translation_abs_error =
    std::abs(result.semi_translation_predicted_error - result.numeric_translation_predicted_error);
  result.translation_rel_error = result.translation_abs_error /
                                 std::max(1e-9,
                                          std::max(std::abs(result.semi_translation_predicted_error),
                                                   std::abs(result.numeric_translation_predicted_error)));

  for (std::size_t axis = 0; axis < 3; ++axis) {
    gtsam::VectorValues axis_delta;
    for (std::size_t i = 0; i < kBSplineControlPointCount; ++i) {
      gtsam::Vector6 d = gtsam::Vector6::Zero();
      d(static_cast<int>(axis)) = perturbation_scale * (1.0 + 0.1 * static_cast<double>(i));
      axis_delta.insert(keys_[i], d);
    }

    const auto axis_perturbed = values.retract(axis_delta);
    result.numeric_axis_rotation_predicted_error[axis] = numeric_linear->error(axis_delta);
    result.semi_axis_rotation_predicted_error[axis] = semi_linear->error(axis_delta);
    result.axis_rotation_actual_error[axis] = semi_factor.error(axis_perturbed);
    result.axis_rotation_abs_error[axis] =
      std::abs(result.semi_axis_rotation_predicted_error[axis] - result.numeric_axis_rotation_predicted_error[axis]);
    result.axis_rotation_rel_error[axis] =
      result.axis_rotation_abs_error[axis] /
      std::max(1e-9,
               std::max(std::abs(result.semi_axis_rotation_predicted_error[axis]),
                        std::abs(result.numeric_axis_rotation_predicted_error[axis])));
    result.mean_rotation_axis_rel_error += result.axis_rotation_rel_error[axis];
    if (result.axis_rotation_rel_error[axis] > result.max_rotation_axis_rel_error) {
      result.max_rotation_axis_rel_error = result.axis_rotation_rel_error[axis];
      result.worst_rotation_axis = axis;
    }
  }
  result.mean_rotation_axis_rel_error /= 3.0;

  return result;
}

IntegratedBSplineGICPFactor::DegeneracyDiagnostics IntegratedBSplineGICPFactor::diagnose_degeneracy(
  const DegeneracyThresholds& thresholds) const {
  DegeneracyDiagnostics diagnostics;
  diagnostics.valid = last_profile_.valid;
  if (!diagnostics.valid) {
    return diagnostics;
  }

  diagnostics.empty_target = last_profile_.target_point_count == 0;
  diagnostics.ambiguity_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_ambiguity_count) /
                                              last_profile_.source_point_count;
  diagnostics.distance_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_distance_count) /
                                              last_profile_.source_point_count;
  diagnostics.outlier_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_outlier_count) /
                                              last_profile_.source_point_count;
  diagnostics.robust_rejection_ratio =
    last_profile_.source_point_count == 0 ? 0.0
                                          : static_cast<double>(last_profile_.rejected_robust_count) /
                                              last_profile_.source_point_count;

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

BSplineLidarFactorProfile IntegratedBSplineGICPFactor::profiling_report() const {
  BSplineLidarFactorProfile report;
  report.valid = last_profile_.valid;
  report.backend = BSplineLidarFactorBackend::CPU_GICP;
  report.source_point_count = last_profile_.source_point_count;
  report.target_point_count = last_profile_.target_point_count;
  report.time_bucket_count = last_profile_.time_bucket_count;
  report.max_time_bucket_population = last_profile_.max_time_bucket_population;
  report.candidate_evaluation_count = last_profile_.candidate_evaluation_count;
  report.matched_point_count = last_profile_.matched_point_count;
  report.inlier_point_count = last_profile_.inlier_point_count;
  report.unique_target_count = last_profile_.unique_target_count;
  report.max_target_reuse = last_profile_.max_target_reuse;
  report.comparative_score_count = last_profile_.comparative_score_count;
  report.rejected_distance_count = last_profile_.rejected_distance_count;
  report.rejected_ambiguity_count = last_profile_.rejected_ambiguity_count;
  report.rejected_outlier_count = last_profile_.rejected_outlier_count;
  report.rejected_robust_count = last_profile_.rejected_robust_count;
  report.match_ratio = last_profile_.match_ratio;
  report.inlier_ratio = last_profile_.inlier_ratio;
  report.mean_time_bucket_population = last_profile_.mean_time_bucket_population;
  report.mean_candidates_per_source = last_profile_.mean_candidates_per_source;
  report.unique_target_ratio = last_profile_.unique_target_ratio;
  report.max_target_reuse_ratio = last_profile_.max_target_reuse_ratio;
  report.mean_match_distance = last_profile_.mean_match_distance;
  report.max_match_distance = last_profile_.max_match_distance;
  report.mean_match_score = last_profile_.mean_match_score;
  report.mean_score_gap = last_profile_.mean_score_gap;
  report.mean_score_ratio = last_profile_.mean_score_ratio;
  report.mean_robust_weight = last_profile_.mean_robust_weight;
  report.pose_update_ms = last_profile_.pose_update_ms;
  report.correspondence_ms = last_profile_.correspondence_ms;
  report.accumulation_ms = last_profile_.accumulation_ms;
  report.total_ms = last_profile_.total_ms;
  report.total_error = last_profile_.total_error;
  report.stage = last_profile_.stage;
  return report;
}

BSplineLidarFactorResult IntegratedBSplineGICPFactor::make_result(
  double factor_error,
  int inlier_count,
  double inlier_fraction,
  const NumericReferenceCheckResult* numeric_reference,
  const DegeneracyDiagnostics* degeneracy) const {
  BSplineLidarFactorResult result;
  result.valid = last_profile_.valid;
  result.backend = BSplineLidarFactorBackend::CPU_GICP;
  result.factor_error = factor_error;
  result.inlier_count = inlier_count;
  result.inlier_fraction = inlier_fraction;
  result.rmse = std::sqrt(std::max(0.0, factor_error) / std::max(inlier_count, 1));
  result.profile = profiling_report();

  if (numeric_reference) {
    result.numeric_audit.valid = numeric_reference->valid;
    result.numeric_audit.perturbation_scale = numeric_reference->perturbation_scale;
    result.numeric_audit.numeric_rotation_predicted_error = numeric_reference->numeric_rotation_predicted_error;
    result.numeric_audit.semi_rotation_predicted_error = numeric_reference->semi_rotation_predicted_error;
    result.numeric_audit.rotation_actual_error = numeric_reference->rotation_actual_error;
    result.numeric_audit.rotation_abs_error = numeric_reference->rotation_abs_error;
    result.numeric_audit.rotation_rel_error = numeric_reference->rotation_rel_error;
    result.numeric_audit.numeric_translation_predicted_error = numeric_reference->numeric_translation_predicted_error;
    result.numeric_audit.semi_translation_predicted_error = numeric_reference->semi_translation_predicted_error;
    result.numeric_audit.translation_actual_error = numeric_reference->translation_actual_error;
    result.numeric_audit.translation_abs_error = numeric_reference->translation_abs_error;
    result.numeric_audit.translation_rel_error = numeric_reference->translation_rel_error;
    result.numeric_audit.axis_rotation_rel_error = numeric_reference->axis_rotation_rel_error;
    result.numeric_audit.worst_rotation_axis = numeric_reference->worst_rotation_axis;
    result.numeric_audit.max_rotation_axis_rel_error = numeric_reference->max_rotation_axis_rel_error;
    result.numeric_audit.mean_rotation_axis_rel_error = numeric_reference->mean_rotation_axis_rel_error;
  }

  if (degeneracy) {
    result.degeneracy.valid = degeneracy->valid;
    result.degeneracy.empty_target = degeneracy->empty_target;
    result.degeneracy.low_match_ratio = degeneracy->low_match_ratio;
    result.degeneracy.low_inlier_ratio = degeneracy->low_inlier_ratio;
    result.degeneracy.low_target_diversity = degeneracy->low_target_diversity;
    result.degeneracy.high_target_reuse = degeneracy->high_target_reuse;
    result.degeneracy.high_ambiguity_rejection = degeneracy->high_ambiguity_rejection;
    result.degeneracy.weak_score_separation = degeneracy->weak_score_separation;
    result.degeneracy.warning_count = degeneracy->warning_count;
    result.degeneracy.ambiguity_rejection_ratio = degeneracy->ambiguity_rejection_ratio;
    result.degeneracy.distance_rejection_ratio = degeneracy->distance_rejection_ratio;
    result.degeneracy.outlier_rejection_ratio = degeneracy->outlier_rejection_ratio;
    result.degeneracy.robust_rejection_ratio = degeneracy->robust_rejection_ratio;
  }

  return result;
}

std::vector<Eigen::Vector4d> IntegratedBSplineGICPFactor::deskewed_source_points(const gtsam::Values& values, bool local) const {
  update_poses(values);

  std::vector<Eigen::Vector4d> points;
  points.reserve(frame::size(*source_));

  const gtsam::Pose3 reference = source_poses_.empty() ? gtsam::Pose3() : source_poses_.front();
  for (int i = 0; i < frame::size(*source_); ++i) {
    const int time_index = time_indices_[static_cast<std::size_t>(i)];
    const auto& pose = source_poses_[static_cast<std::size_t>(time_index)];
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
