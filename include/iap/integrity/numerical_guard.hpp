#pragma once
// IAP Step 2: Numerical guard utilities for integrity monitoring.
// Prevents NaN/Inf/sentinel values from being silently published.

#include <cmath>
#include <Eigen/Core>
#include <limits>

namespace iap {
namespace numerical_guard {

/// Conservative PL value when source data is invalid.
constexpr double kSentinel = 1e9;

/// Check if a scalar is valid (finite, not NaN).
inline bool is_valid(double value) {
  return std::isfinite(value);
}

/// Check if a scalar is valid AND positive (for variances, sigmas).
inline bool is_valid_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

/// Check if a scalar is valid AND non-negative (for variances).
inline bool is_valid_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

/// Check if a 3D vector is finite.
inline bool is_valid(const Eigen::Vector3d& v) {
  return v.allFinite();
}

/// Check if a 3x3 matrix diagonal entries are valid and non-negative.
inline bool is_valid_covariance(const Eigen::Matrix3d& cov) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(cov(i, i)) || cov(i, i) < 0.0) {
      return false;
    }
  }
  return true;
}

/// Clamp to sentinel if invalid.
inline double sentinel_if_invalid(double value, double sentinel = kSentinel) {
  return std::isfinite(value) ? value : sentinel;
}

/// Return conservative sentinel and set flag if value is invalid.
inline double guard(double value, bool& failure_flag, double sentinel = kSentinel) {
  if (!std::isfinite(value)) {
    failure_flag = true;
    return sentinel;
  }
  return value;
}

/// Return conservative sentinel and set flag if value is not positive.
inline double guard_positive(double value, bool& failure_flag, double sentinel = kSentinel) {
  if (!std::isfinite(value) || value <= 0.0) {
    failure_flag = true;
    return sentinel;
  }
  return value;
}

}  // namespace numerical_guard
}  // namespace iap
