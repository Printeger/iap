#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <limits>

using namespace std;

namespace ego_planner
{
  struct P4ControlCapabilityProfile
  {
    std::string schema_version = "p4_control_capability_v1";
    Eigen::Vector3d maximum_velocity_mps = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum_acceleration_mps2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum_jerk_mps3 = Eigen::Vector3d::Zero();
    double measured_latency_bound_s = 0.0;
    Eigen::Vector3d position_tracking_bound_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_tracking_bound_mps = Eigen::Vector3d::Zero();
    std::string controller_identity;
    std::string simulator_identity;
    std::string code_version;

    bool valid() const;
  };

  struct BsplineDerivativeLimitResult
  {
    bool valid = false;
    bool velocity_ok = false;
    bool acceleration_ok = false;
    bool jerk_ok = false;
    Eigen::Vector3d maximum_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum_jerk = Eigen::Vector3d::Zero();
    std::size_t jerk_node_side_samples = 0;
    std::string first_violation_derivative;
    int first_violation_axis = -1;
    std::size_t first_violation_index = 0;
    double first_violation_value = std::numeric_limits<double>::quiet_NaN();
    double first_violation_limit = std::numeric_limits<double>::quiet_NaN();
    double required_time_scale = std::numeric_limits<double>::infinity();
    std::string reason;
  };

  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each row represents one single control point
    // The dimension is determined by column number
    // e.g. B-spline with N points in 3D space -> Nx3 matrix
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_;   // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints();

    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot();
    Eigen::MatrixXd getControlPoint();
    double getInterval();
    bool getTimeSpan(double &um, double &um_p);

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u);                                               // use u \in [up, u_mp]
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
    UniformBspline getDerivative();

    // Return the exact suffix on [t, duration] by knot insertion. The
    // resulting spline starts at local time zero without refitting or
    // changing the represented curve.
    bool sliceFrom(const double &t, UniformBspline &suffix);

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    // Least-squares path fit with exact cubic endpoint position, velocity and
    // acceleration constraints. Unlike parameterizeToBspline(), boundary
    // rows are eliminated before solving and therefore cannot be traded off
    // against interior fit error.
    static bool parameterizeToBsplineWithBoundaryConstraints(
        const double &ts, const vector<Eigen::Vector3d> &point_set,
        const vector<Eigen::Vector3d> &start_end_derivative,
        Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    bool checkFeasibility(double &ratio, bool show = false);
    BsplineDerivativeLimitResult checkDerivativeLimits(
        const P4ControlCapabilityProfile &profile,
        double tolerance = 0.0);
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum();
    double getLength(const double &res = 0.01);
    double getJerk();
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif
