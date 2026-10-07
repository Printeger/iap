#include "bspline_opt/uniform_bspline.h"
#include <stdexcept>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {

      if (i <= p_)
      {
        u_(i) = double(-p_ + i) * interval_;
      }
      else if (i > p_ && i <= m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
      else if (i > m_ - p_)
      {
        u_(i) = u_(i - 1) + interval_;
      }
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  std::vector<double> UniformBspline::coordinateExtremaTimes(double from,double to) {
    std::vector<double> times;
    if(p_!=3) throw std::invalid_argument("physical extrema require cubic spline");
    auto velocity=getDerivative();
    for(int span=p_;span<m_-p_;++span) {
      const double left=u_[span]-u_[p_],right=u_[span+1]-u_[p_],width=right-left;
      if(width<=0 || right<from || left>to) continue;
      times.push_back(std::clamp(left,from,to)); times.push_back(std::clamp(right,from,to));
      const auto v0=velocity.evaluateDeBoorT(left).eval();
      const auto vm=velocity.evaluateDeBoorT((left+right)/2).eval();
      const auto v1=velocity.evaluateDeBoorT(right).eval();
      for(int axis=0;axis<control_points_.rows();++axis) {
        const double a=2*(v1[axis]+v0[axis]-2*vm[axis]),b=v1[axis]-v0[axis]-a,c=v0[axis];
        const auto append=[&](double u) { const double t=left+width*u;
          if(u>0 && u<1 && t>=from && t<=to) times.push_back(t); };
        if(std::abs(a)<1e-13) { if(std::abs(b)>1e-13) append(-c/b); }
        else { const double d=b*b-4*a*c; if(d>=0) {
          append((-b+std::sqrt(d))/(2*a)); append((-b-std::sqrt(d))/(2*a));
        }}
      }
    }
    std::sort(times.begin(),times.end());
    times.erase(std::unique(times.begin(),times.end()),times.end()); return times;
  }

  double UniformBspline::getInterval() { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;

    Eigen::MatrixXd P = control_points_;
    int dimension = control_points_.rows();

    /* check vel feasibility and insert points */
    double max_vel = -1.0;
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
          fabs(vel(2)) > enlarged_vel_lim)
      {

        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_vel = max(max_vel, fabs(vel(j)));
        }
      }
    }

    /* acc feasibility */
    double max_acc = -1.0;
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i)
    {

      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
          fabs(acc(2)) > enlarged_acc_lim)
      {

        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;

        for (int j = 0; j < dimension; ++j)
        {
          max_acc = max(max_acc, fabs(acc(j)));
        }
      }
    }

    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    int num1 = 5;
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
    double t_inc = delta_t / double(num2 - num1);
    for (int i = num1 + 1; i <= num2; ++i)
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;
  }

  // void UniformBspline::recomputeInit() {}

  // 将一组点转换为控制点
  void UniformBspline::enforceBoundaryStates(Eigen::MatrixXd& points, double interval,
      const Eigen::Vector3d& start, const Eigen::Vector3d& start_velocity,
      const Eigen::Vector3d& start_acceleration, const Eigen::Vector3d& end,
      const Eigen::Vector3d& end_velocity, const Eigen::Vector3d& end_acceleration) {
    // Uniform cubic endpoint basis: position=(q0+4q1+q2)/6,
    // velocity=(q2-q0)/(2dt), acceleration=(q0-2q1+q2)/dt^2.
    if(points.rows()!=3 || points.cols()<7 || !(interval>0))
      throw std::invalid_argument("cubic boundary states need at least seven points and positive interval");
    const double squared=interval*interval;
    points.col(0)=start-start_velocity*interval+start_acceleration*squared/3;
    points.col(1)=start-start_acceleration*squared/6;
    points.col(2)=start+start_velocity*interval+start_acceleration*squared/3;
    const int n=points.cols();
    points.col(n-3)=end-end_velocity*interval+end_acceleration*squared/3;
    points.col(n-2)=end-end_acceleration*squared/6;
    points.col(n-1)=end+end_velocity*interval+end_acceleration*squared/3;
  }

  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    if(!std::isfinite(ts) || ts<=0 || point_set.size()<5 || start_end_derivative.size()!=4 ||
        !std::all_of(point_set.begin(),point_set.end(),[](const Eigen::Vector3d& p){return p.allFinite();}) ||
        !std::all_of(start_end_derivative.begin(),start_end_derivative.end(),[](const Eigen::Vector3d& p){return p.allFinite();}))
      throw std::invalid_argument("cubic fitting needs five finite samples, four derivatives and positive interval");
    const int count=point_set.size();
    // P/V/A is an equality constraint. Mixing derivative rows into a least
    // squares fit and overwriting its endpoint triplets afterwards changes the
    // fitted shape. Eliminate those fixed triplets before solving the interior.
    ctrl_pts=Eigen::MatrixXd::Zero(3,count+2);
    enforceBoundaryStates(ctrl_pts,ts,point_set.front(),start_end_derivative[0],start_end_derivative[2],
        point_set.back(),start_end_derivative[1],start_end_derivative[3]);
    Eigen::MatrixXd basis=Eigen::MatrixXd::Zero(count,count+2),samples(count,3);
    for(int i=0;i<count;++i) {
      basis.block<1,3>(i,i)<<1./6,4./6,1./6;
      samples.row(i)=point_set[i].transpose();
    }
    const Eigen::MatrixXd residual=samples-basis.leftCols(3)*ctrl_pts.leftCols(3).transpose()-
        basis.rightCols(3)*ctrl_pts.rightCols(3).transpose();
    ctrl_pts.middleCols(3,count-4)=basis.middleCols(3,count-4).colPivHouseholderQr().solve(residual).transpose();
  }

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
