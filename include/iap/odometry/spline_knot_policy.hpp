#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Commit-9 spline knot placement policies for uniform and IMU-activity-driven spacing.

#include <Eigen/Core>

#include <vector>

namespace iap {

struct SplinePolicyImuSample {
  double stamp = 0.0;
  Eigen::Vector3d linear_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();
};

struct KnotPlacementDecision {
  std::vector<double> knots;
};

class SplineKnotPolicy {
 public:
  virtual ~SplineKnotPolicy() = default;
  virtual KnotPlacementDecision decide(double segment_start, double segment_end, const std::vector<SplinePolicyImuSample>& imu_samples) const = 0;
};

class UniformSplineKnotPolicy : public SplineKnotPolicy {
 public:
  explicit UniformSplineKnotPolicy(double nominal_dt) : nominal_dt_(nominal_dt) {}

  KnotPlacementDecision decide(
    double segment_start,
    double segment_end,
    const std::vector<SplinePolicyImuSample>& imu_samples) const override;

 private:
  double nominal_dt_ = 0.1;
};

class ImuActivitySplineKnotPolicy : public SplineKnotPolicy {
 public:
  struct Params {
    double min_dt = 0.03;
    double max_dt = 0.15;
    double activity_gyro_gain = 1.0;
    double activity_acc_gain = 1.0;
    int target_density_coarse = 1;
    int target_density_fine = 4;
  };

  explicit ImuActivitySplineKnotPolicy(const Params& params) : params_(params) {}

  KnotPlacementDecision decide(
    double segment_start,
    double segment_end,
    const std::vector<SplinePolicyImuSample>& imu_samples) const override;

 private:
  Params params_;
};

}  // namespace iap
