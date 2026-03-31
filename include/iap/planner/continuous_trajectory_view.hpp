#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Continuous-time trajectory view reserved for future spline-based planning.

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace iap {

enum class SplineKnotMode {
  Uniform,
  NonUniform
};

inline const char* to_string(SplineKnotMode mode) {
  switch (mode) {
    case SplineKnotMode::Uniform:
      return "uniform";
    case SplineKnotMode::NonUniform:
      return "non_uniform";
  }
  return "unknown";
}

struct TrajectorySample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double sigma = 0.0;
};

struct SplineControlPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  double sigma = 0.0;
};

struct SplineMeta {
  double t_min = 0.0;
  double t_max = 0.0;
  double nominal_dt = 0.0;
  int order = 3;
  std::size_t knot_count = 0;
  std::size_t control_point_count = 0;
  SplineKnotMode knot_mode = SplineKnotMode::Uniform;
};

struct SplineWindowSnapshot {
  // Explicit knots are the authoritative spline-time layout whenever they are
  // populated; later adapters may keep legacy uniform reconstruction only as a
  // compatibility fallback.
  SplineMeta meta;
  std::vector<double> knots;
  std::vector<SplineControlPoint> control_points;
};

class ContinuousTrajectoryView {
 public:
  virtual ~ContinuousTrajectoryView() = default;

  virtual bool empty() const = 0;
  virtual double start_time() const = 0;
  virtual double end_time() const = 0;
  virtual SplineMeta meta() const = 0;

  virtual std::optional<TrajectorySample> sample(double stamp) const = 0;
  virtual std::optional<TrajectorySample> latest_sample() const = 0;
  virtual std::vector<TrajectorySample> sample_range(double start, double end, double step) const = 0;
};

class SplineControlAccess {
 public:
  virtual ~SplineControlAccess() = default;

  virtual SplineMeta meta() const = 0;
  virtual std::vector<double> knot_vector() const = 0;
  virtual std::vector<SplineControlPoint> control_points() const = 0;
  virtual SplineWindowSnapshot clone_window() const = 0;
};

struct ContinuousTrajectoryAttachment {
  std::shared_ptr<const ContinuousTrajectoryView> trajectory_view;
  std::shared_ptr<const SplineControlAccess> control_access;
  SplineMeta meta;
  std::string producer;
};

inline constexpr const char* kContinuousTrajectoryAttachmentKey = "iap.continuous_trajectory";

}  // namespace iap
