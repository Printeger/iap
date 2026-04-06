#pragma once

#include <cstddef>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace iap {

enum class BSplineFactorFamily {
  IMU,
  VELOCITY,
  LIDAR,
  PRIOR,
  OTHER,
};

enum class BSplineKeyFamily {
  POSE,
  AUX,
  SHARED,
  OTHER,
};

struct BSplineFactorFamilyCounts {
  std::size_t imu_factor_count{0};
  std::size_t velocity_factor_count{0};
  std::size_t lidar_factor_count{0};
  std::size_t prior_factor_count{0};
  std::size_t shared_jkg_touching_factor_count{0};
};

struct BSplineKeyFamilyCounts {
  std::size_t pose_key_count{0};
  std::size_t aux_key_count{0};
  std::size_t shared_key_count{0};
};

BSplineFactorFamily classify_bspline_factor_family(const gtsam::NonlinearFactor& factor);
BSplineKeyFamily classify_bspline_key_family(gtsam::Key key);
bool factor_touches_shared_jkg(const gtsam::NonlinearFactor& factor);
void accumulate_bspline_factor_family_counts(
  BSplineFactorFamilyCounts& counts,
  const gtsam::NonlinearFactor::shared_ptr& factor);
void accumulate_bspline_key_family_counts(BSplineKeyFamilyCounts& counts, gtsam::Key key);

}  // namespace iap
