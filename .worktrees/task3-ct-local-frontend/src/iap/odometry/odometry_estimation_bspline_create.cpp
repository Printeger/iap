#include <iap/odometry/odometry_estimation_bspline.hpp>

extern "C" glim::OdometryEstimationBase* create_odometry_estimation_module() {
  glim::OdometryEstimationBSplineParams params;
  return new glim::OdometryEstimationBSpline(params);
}
