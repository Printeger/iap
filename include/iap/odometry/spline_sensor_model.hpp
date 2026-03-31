#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Foundational sensor-model types for the explicit-knot spline core.

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace iap {

enum class SplineSensorId {
  Imu,
  Lidar,
  Gnss,
};

struct SplineSensorModel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SplineSensorId id = SplineSensorId::Imu;
  Eigen::Isometry3d T_sensor_imu = Eigen::Isometry3d::Identity();
  double time_offset = 0.0;
};

}  // namespace iap
