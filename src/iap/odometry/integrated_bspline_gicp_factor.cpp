#include <iap/odometry/integrated_bspline_gicp_factor.hpp>

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/ann/kdtree2.hpp>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <algorithm>
#include <chrono>
#include <limits>

namespace iap {

namespace frame = gtsam_points::frame;

namespace {

double residual_norm_from_mahalanobis(const double mahalanobis_error) {
  return std::sqrt(std::max(0.0, mahalanobis_error));
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
    std::shared_ptr<gtsam_points::PointCloud> target_points =
      std::make_shared<gtsam_points::PointCloudCPU>(target_->voxel_points());
    target_tree_ = std::make_shared<gtsam_points::KdTree2<gtsam_points::PointCloud>>(target_points);
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
    pose_rotation_jacobians_.clear();
  } else {
    pose_jacobians_.clear();
    pose_rotation_jacobians_.resize(time_table_.size());
  }

  for (std::size_t i = 0; i < time_table_.size(); ++i) {
    const double u = time_table_[i];
    const gtsam::Pose3 base_pose = BSplineControlWindow::interpolate(poses, u);
    source_poses_[i] = base_pose;

    if (jacobian_mode_ != JacobianMode::NUMERIC_FULL) {
      for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
        Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero();
        for (int d = 0; d < 3; ++d) {
          gtsam::Vector6 delta = gtsam::Vector6::Zero();
          delta(d) = numeric_eps_;

          auto perturbed = poses;
          perturbed[k] = perturbed[k].compose(gtsam::Pose3::Expmap(delta));

          const gtsam::Pose3 pose_plus = BSplineControlWindow::interpolate(perturbed, u);
          const gtsam::Vector6 xi = gtsam::Pose3::Logmap(base_pose.between(pose_plus));
          J.col(d) = xi / numeric_eps_;
        }
        pose_rotation_jacobians_[i][k] = J;
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
  matched_correspondence_count_ = 0;
  rejected_distance_count_ = 0;
  rejected_ambiguity_count_ = 0;

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
      rejected_distance_count_++;
      continue;
    }

    const auto& source_cov = frame::cov(*source_, i);
    const Eigen::Matrix4d pose_matrix = pose.matrix();
    double best_score = std::numeric_limits<double>::max();
    double second_best_score = std::numeric_limits<double>::max();
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
      const double score = residual.transpose() * M * residual;

      if (score < best_score) {
        second_best_score = best_score;
        best_score = score;
        best_index = target_index;
        best_mahalanobis = M;
      } else if (score < second_best_score) {
        second_best_score = score;
      }
    }

    if (best_index < 0) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      rejected_distance_count_++;
      continue;
    }

    if (correspondence_accept_ratio_ > 0.0 &&
        second_best_score < std::numeric_limits<double>::max() &&
        second_best_score > 1e-9 &&
        (best_score / second_best_score) >= correspondence_accept_ratio_) {
      correspondences_[static_cast<std::size_t>(i)] = -1;
      mahalanobis_[static_cast<std::size_t>(i)].setZero();
      rejected_ambiguity_count_++;
      continue;
    }

    correspondences_[static_cast<std::size_t>(i)] = best_index;
    mahalanobis_[static_cast<std::size_t>(i)] = best_mahalanobis;
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
  gtsam::Matrix36 H_transed_pose = gtsam::Matrix36::Zero();
  H_transed_pose.block<3, 3>(0, 0) = source_poses_[time_index].rotation().matrix() * -gtsam::SO3::Hat(source_point);
  H_transed_pose.block<3, 3>(0, 3) = source_poses_[time_index].rotation().matrix();

  for (std::size_t k = 0; k < kBSplineControlPointCount; ++k) {
    jacobians[k].setZero();
    jacobians[k].block<3, 3>(0, 0) = H_transed_pose * pose_rotation_jacobians_[time_index][k];
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

double IntegratedBSplineGICPFactor::error(const gtsam::Values& values) const {
  using Clock = std::chrono::steady_clock;
  const auto t_start = Clock::now();
  update_poses(values);
  const auto t_pose = Clock::now();
  update_correspondences();
  const auto t_corr = Clock::now();

  accepted_inlier_count_ = 0;
  rejected_outlier_count_ = 0;
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

    accepted_inlier_count_++;
    robust_weight_sum_ += robust_weight(mahalanobis_error);
    total_error += robust_cost(mahalanobis_error);
  }

  if (enable_profiling_) {
    const auto t_end = Clock::now();
    last_profile_.valid = true;
    last_profile_.stage = "error";
    last_profile_.source_point_count = frame::size(*source_);
    last_profile_.target_point_count = target_->voxel_points().size();
    last_profile_.matched_point_count = matched_correspondence_count_;
    last_profile_.inlier_point_count = accepted_inlier_count_;
    last_profile_.rejected_distance_count = rejected_distance_count_;
    last_profile_.rejected_ambiguity_count = rejected_ambiguity_count_;
    last_profile_.rejected_outlier_count = rejected_outlier_count_;
    last_profile_.match_ratio =
      last_profile_.source_point_count == 0 ? 0.0
                                            : static_cast<double>(matched_correspondence_count_) / last_profile_.source_point_count;
    last_profile_.inlier_ratio =
      last_profile_.source_point_count == 0 ? 0.0
                                            : static_cast<double>(accepted_inlier_count_) / last_profile_.source_point_count;
    last_profile_.mean_robust_weight =
      accepted_inlier_count_ == 0 ? 1.0 : robust_weight_sum_ / static_cast<double>(accepted_inlier_count_);
    last_profile_.pose_update_ms =
      std::chrono::duration<double, std::milli>(t_pose - t_start).count();
    last_profile_.correspondence_ms =
      std::chrono::duration<double, std::milli>(t_corr - t_pose).count();
    last_profile_.accumulation_ms =
      std::chrono::duration<double, std::milli>(t_end - t_corr).count();
    last_profile_.total_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
    last_profile_.total_error = total_error;
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

    accepted_inlier_count_++;
    const double weight = robust_weight(mahalanobis_error);
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
    last_profile_.valid = true;
    last_profile_.stage = "linearize";
    last_profile_.source_point_count = frame::size(*source_);
    last_profile_.target_point_count = target_->voxel_points().size();
    last_profile_.matched_point_count = matched_correspondence_count_;
    last_profile_.inlier_point_count = accepted_inlier_count_;
    last_profile_.rejected_distance_count = rejected_distance_count_;
    last_profile_.rejected_ambiguity_count = rejected_ambiguity_count_;
    last_profile_.rejected_outlier_count = rejected_outlier_count_;
    last_profile_.match_ratio =
      last_profile_.source_point_count == 0 ? 0.0
                                            : static_cast<double>(matched_correspondence_count_) / last_profile_.source_point_count;
    last_profile_.inlier_ratio =
      last_profile_.source_point_count == 0 ? 0.0
                                            : static_cast<double>(accepted_inlier_count_) / last_profile_.source_point_count;
    last_profile_.mean_robust_weight =
      accepted_inlier_count_ == 0 ? 1.0 : robust_weight_sum_ / static_cast<double>(accepted_inlier_count_);
    last_profile_.pose_update_ms =
      std::chrono::duration<double, std::milli>(t_pose - t_start).count();
    last_profile_.correspondence_ms =
      std::chrono::duration<double, std::milli>(t_corr - t_pose).count();
    last_profile_.accumulation_ms =
      std::chrono::duration<double, std::milli>(t_end - t_corr).count();
    last_profile_.total_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
    last_profile_.total_error = total_error;
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
