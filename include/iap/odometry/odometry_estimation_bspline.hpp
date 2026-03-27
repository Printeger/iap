#pragma once
// IAP-RQ-300 / IAP-RQ-410:
// Phase-1 B-spline odometry module. Builds a continuous-time trajectory view
// on top of the current LiDAR-IMU odometry pipeline without breaking legacy
// discrete outputs consumed by mapping / viewer modules.

#include <iap/odometry/bspline_trajectory.hpp>
#include <iap/odometry/odometry_estimation_cpu.hpp>

namespace gtsam {
class NonlinearFactorGraph;
}

namespace glim {

struct OdometryEstimationBSplineParams : public OdometryEstimationCPUParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OdometryEstimationBSplineParams();
  virtual ~OdometryEstimationBSplineParams();

 public:
  std::string spline_knot_mode;
  double spline_nominal_dt = 0.0;
  double spline_finite_difference_dt = 0.01;
  double compatibility_sample_dt = 0.01;
  bool publish_shared_trajectory = true;
  bool attach_trajectory_to_frames = true;
};

class OdometryEstimationBSpline : public OdometryEstimationCPU {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit OdometryEstimationBSpline(
    const OdometryEstimationBSplineParams& params = OdometryEstimationBSplineParams());
  virtual ~OdometryEstimationBSpline() override;

 protected:
  void update_frames(int current, const gtsam::NonlinearFactorGraph& new_factors) override;

 private:
  void publish_continuous_trajectory(int current);
  void update_frame_attachment(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;
  void update_compatibility_trajectory(const std::shared_ptr<iap::BSplineTrajectory>& trajectory) const;

  iap::BSplineTrajectory::Params trajectory_params_;
  double compatibility_sample_dt_ = 0.01;
  bool publish_shared_trajectory_ = true;
  bool attach_trajectory_to_frames_ = true;
  std::shared_ptr<iap::BSplineTrajectory> latest_trajectory_;
};

}  // namespace glim
