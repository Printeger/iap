#include <iap/odometry/bspline_factor_family.hpp>

#include <iap/gnss/clock_between_factor.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <iap/odometry/integrated_bspline_imu_factor.hpp>
#include <iap/odometry/integrated_bspline_velocity_factor.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace iap {

namespace {

bool all_keys_match_family(const gtsam::NonlinearFactor& factor, BSplineKeyFamily family) {
  for (const auto key : factor.keys()) {
    if (classify_bspline_key_family(key) != family) {
      return false;
    }
  }
  return true;
}

bool is_odometry_prior_factor(const gtsam::NonlinearFactor& factor) {
  if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(&factor) != nullptr) {
    return factor.keys().size() == 1 && classify_bspline_key_family(factor.keys().front()) == BSplineKeyFamily::POSE;
  }

  if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(&factor) != nullptr) {
    return !factor.keys().empty() && all_keys_match_family(factor, BSplineKeyFamily::POSE);
  }

  if (dynamic_cast<const gtsam::PriorFactor<gtsam::Vector3>*>(&factor) != nullptr) {
    if (factor.keys().size() != 1) {
      return false;
    }
    const char chr = gtsam::Symbol(factor.keys().front()).chr();
    return chr == 'u' || chr == 'j' || chr == 'k' || chr == 'g';
  }

  return false;
}

}  // namespace

BSplineFactorFamily classify_bspline_factor_family(const gtsam::NonlinearFactor& factor) {
  if (dynamic_cast<const IntegratedSplineIMUFactor*>(&factor) != nullptr ||
      dynamic_cast<const IntegratedBSplineIMUFactor*>(&factor) != nullptr) {
    return BSplineFactorFamily::IMU;
  }

  if (dynamic_cast<const IntegratedBSplineVelocityFactor*>(&factor) != nullptr) {
    return BSplineFactorFamily::VELOCITY;
  }

  if (dynamic_cast<const IntegratedSplineGICPFactor*>(&factor) != nullptr ||
      dynamic_cast<const IntegratedBSplineGICPFactor*>(&factor) != nullptr) {
    return BSplineFactorFamily::LIDAR;
  }

  if (dynamic_cast<const iap::ClockBetweenFactor*>(&factor) != nullptr) {
    return BSplineFactorFamily::OTHER;
  }

  if (is_odometry_prior_factor(factor)) {
    return BSplineFactorFamily::PRIOR;
  }

  return BSplineFactorFamily::OTHER;
}

BSplineKeyFamily classify_bspline_key_family(gtsam::Key key) {
  switch (gtsam::Symbol(key).chr()) {
    case 's':
      return BSplineKeyFamily::POSE;
    case 'u':
    case 'c':
      return BSplineKeyFamily::AUX;
    case 'j':
    case 'k':
    case 'g':
    case 'e':
    case 'r':
      return BSplineKeyFamily::SHARED;
    default:
      return BSplineKeyFamily::OTHER;
  }
}

bool factor_touches_shared_jkg(const gtsam::NonlinearFactor& factor) {
  for (const auto key : factor.keys()) {
    const char chr = gtsam::Symbol(key).chr();
    if (chr == 'j' || chr == 'k' || chr == 'g') {
      return true;
    }
  }
  return false;
}

void accumulate_bspline_factor_family_counts(
  BSplineFactorFamilyCounts& counts,
  const gtsam::NonlinearFactor::shared_ptr& factor) {
  if (!factor) {
    return;
  }

  switch (classify_bspline_factor_family(*factor)) {
    case BSplineFactorFamily::IMU:
      ++counts.imu_factor_count;
      break;
    case BSplineFactorFamily::VELOCITY:
      ++counts.velocity_factor_count;
      break;
    case BSplineFactorFamily::LIDAR:
      ++counts.lidar_factor_count;
      break;
    case BSplineFactorFamily::PRIOR:
      ++counts.prior_factor_count;
      break;
    case BSplineFactorFamily::OTHER:
      break;
  }

  if (factor_touches_shared_jkg(*factor)) {
    ++counts.shared_jkg_touching_factor_count;
  }
}

void accumulate_bspline_key_family_counts(BSplineKeyFamilyCounts& counts, gtsam::Key key) {
  switch (classify_bspline_key_family(key)) {
    case BSplineKeyFamily::POSE:
      ++counts.pose_key_count;
      break;
    case BSplineKeyFamily::AUX:
      ++counts.aux_key_count;
      break;
    case BSplineKeyFamily::SHARED:
      ++counts.shared_key_count;
      break;
    case BSplineKeyFamily::OTHER:
      break;
  }
}

}  // namespace iap
