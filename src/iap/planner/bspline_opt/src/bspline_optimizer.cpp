#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
#include <stdexcept>
// using namespace std;

namespace ego_planner
{
  namespace {
    double guideCorridorM(double resolution) {
      return (.5+std::sqrt(3.)/2)*resolution;
    }
    Eigen::Vector3d nearestGuidePoint(const std::vector<Eigen::Vector3d>& guide,
        const Eigen::Vector3d& position) {
      Eigen::Vector3d nearest=guide.front();
      double best=std::numeric_limits<double>::infinity();
      for(size_t j=1;j<guide.size();++j) {
        const Eigen::Vector3d edge=guide[j]-guide[j-1];
        const double fraction=edge.squaredNorm()>1e-12 ? std::clamp(
            (position-guide[j-1]).dot(edge)/edge.squaredNorm(),0.,1.) : 0.;
        const Eigen::Vector3d point=guide[j-1]+fraction*edge;
        const double distance=(point-position).squaredNorm();
        if(distance<best) {best=distance;nearest=point;}
      }
      return nearest;
    }
  }

  void BsplineOptimizer::setParam(rclcpp::Node::SharedPtr node)
  {

    clock_ = node->get_clock();
    node->declare_parameter("optimization/lambda_smooth", -1.0);
    node->declare_parameter("optimization/lambda_collision", -1.0);
    node->declare_parameter("optimization/lambda_feasibility", -1.0);
    node->declare_parameter("optimization/lambda_fitness", -1.0);

    node->declare_parameter("optimization/dist0", -1.0);
    node->declare_parameter("optimization/swarm_clearance", -1.0);
    node->declare_parameter("optimization/max_vel", -1.0);
    node->declare_parameter("optimization/max_acc", -1.0);

    node->declare_parameter("optimization/order", 3);

    node->get_parameter("optimization/lambda_smooth", lambda1_);
    node->get_parameter("optimization/lambda_collision", lambda2_);
    node->get_parameter("optimization/lambda_feasibility", lambda3_);
    node->get_parameter("optimization/lambda_fitness", lambda4_);

    node->get_parameter("optimization/dist0", dist0_);
    node->get_parameter("optimization/swarm_clearance", swarm_clearance_);
    node->get_parameter("optimization/max_vel", max_vel_);
    node->get_parameter("optimization/max_acc", max_acc_);

    node->get_parameter("optimization/order", order_);
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    this->grid_map_ = map;
  }

  void BsplineOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj)
  {
    this->grid_map_ = map;
    this->moving_objs_ = mov_obj;
  }

  void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    // Guide indexing and rebound storage must describe this same matrix.
    // Keep existing sample constraints when only its coordinates change.
    if(cps_.size!=points.cols()) {
      cps_.resize(points.cols());
      curve_clearance_constraints_.clear();
    }
    cps_.points = points;
  }

  bool BsplineOptimizer::curveViolates(const Eigen::MatrixXd& points, double interval) const {
    if (!planning_query_) return false;
    UniformBspline curve(points,3,interval);
    const auto derivative=curve.getDerivative().getControlPoint();
    double bound=0;
    for(int i=0;i<derivative.cols();++i) bound=std::max(bound,derivative.col(i).norm());
    const double spacing=std::min(.02,grid_map_->getResolution()/(2*std::max(.1,bound)));
    const double duration=curve.getTimeSum();
    const size_t count=static_cast<size_t>(std::ceil(duration/spacing));
    for(size_t i=0;i<=count;++i) {
      if (budget_ && budget_->expired()) return true;
      const auto cell=planning_query_(curve.evaluateDeBoorT(std::min(duration,i*spacing)));
      if(!cell.executable()) return true;
    }
    return false;
  }

  bool BsplineOptimizer::searchRecoveryGuide() {
    if (!planning_endpoints_ || planning_goals_.empty()) return false;
    if(budget_ && !budget_->tryRepair(PlanningBudget::Repair::Search)) return false;
    const auto start=planning_endpoints_->first;
    const Eigen::Vector3d center=planning_goal_center_.value_or((start+planning_goals_.front())/2);
    // One original 1 s search allowance, also bounded by the round's budget.
    // A retry consumes the remainder, never another fresh search allowance.
    const auto deadline=PlanningBudget::Clock::now()+std::chrono::duration_cast<PlanningBudget::Clock::duration>(
        std::chrono::duration<double>(std::min(1.,budget_ ? budget_->remaining() : 1.)));
    const auto remaining=[&]() { return std::max(0.,std::chrono::duration<double>(deadline-PlanningBudget::Clock::now()).count()); };
    recovery_search_evidence_.emplace_back();
    auto& evidence=recovery_search_evidence_.back();evidence.initial_remaining_s=remaining();
    const auto origin=guide_query_ ? guide_query_(start) : GridPlanningCell{};
    evidence.start_reason=origin.execution_reason;evidence.start_advisory=origin.advisory.classification;
    const auto reject=[&](AStar::Failure failure) {
      a_star_->recordPresearchFailure(failure,start,planning_goals_.front(),GridSearchCell(origin));
      evidence.final_failure=failure;initialization_failed_=true;
      reportSearchFailure(a_star_->lastResult(),cps_.points,0,cps_.size-1,"whole_curve_recovery");
      return false;
    };
    if(remaining()<=0 || (budget_ && budget_->expired()))return reject(AStar::Failure::TIME_BUDGET);
    if(!origin.executable())return reject(
        origin.execution_reason==GridExecutionReason::ENVIRONMENT_STALE ? AStar::Failure::MAP_STALE :
        origin.execution_reason==GridExecutionReason::CURRENT_MOTION_UNAVAILABLE ||
        origin.execution_reason==GridExecutionReason::CURRENT_MOTION_STALE ||
        origin.execution_reason==GridExecutionReason::CURRENT_MOTION_BUDGET ? AStar::Failure::CURRENT_MOTION : AStar::Failure::START_BLOCKED);
    evidence.normal_attempted=true;
    const bool found=a_star_->AstarSearchGoals(.1,start,planning_goals_,remaining(),center,
        AStar::GoalSearchPurpose::Guide);
    evidence.normal_failure=a_star_->lastResult().failure;
    evidence.final_failure=a_star_->lastResult().failure;evidence.guide_found=found;
    if(!found) {
      initialization_failed_=true;
      reportSearchFailure(a_star_->lastResult(),cps_.points,0,cps_.size-1,"whole_curve_recovery");
      return false;
    }
    guide_pts_=a_star_->getPath();
    guide_reinitialization_=guide_pts_.size()>=2;
    initialization_failed_=!guide_reinitialization_;
    return guide_reinitialization_;
  }

  void BsplineOptimizer::initializeFromGuide(const Eigen::MatrixXd& points) {
    // Constraint indices refer to a specific parameterization, never another target fit.
    curve_clearance_constraints_.clear();
    cps_.resize(points.cols()); cps_.points=points; cps_.clearance=dist0_;
    if(planning_query_ && planning_endpoints_) {
      const auto start=planning_query_(planning_endpoints_->first);
      if(std::isfinite(start.required_clearance_m)) cps_.clearance=std::max(dist0_,start.required_clearance_m);
    }
    guide_reinitialization_=false;
    const auto guide=guide_pts_;
    setGuidePath(guide);
    // Rebound constraints point toward the closest position on this same guide.
    // At that position the signed clearance is satisfied. Tracking retains the
    // route when the smoothing objective would otherwise cut the corner.
    for(int i=order_; i<cps_.size-order_ && guide.size()>=2; ++i) {
      Eigen::Vector3d nearest=guide.front(); double best=inf;
      for(size_t j=1;j<guide.size();++j) {
        const Eigen::Vector3d segment=guide[j]-guide[j-1];
        const double fraction=segment.squaredNorm()>1e-12 ? std::clamp(
            (points.col(i)-guide[j-1]).dot(segment)/segment.squaredNorm(),0.0,1.0) : 0;
        const Eigen::Vector3d candidate=guide[j-1]+fraction*segment;
        const double distance=(candidate-points.col(i)).squaredNorm();
        if(distance<best) { best=distance; nearest=candidate; }
      }
      Eigen::Vector3d direction=nearest-points.col(i);
      if(direction.norm()>1e-6) {
        direction.normalize(); cps_.direction[i].push_back(direction);
        cps_.base_point[i].push_back(nearest-direction*cps_.clearance);
      }
    }
  }

  void BsplineOptimizer::rebindAfterUniformRetime(const Eigen::MatrixXd& points) {
    if(points.rows()!=3 || points.cols()<7 || !points.allFinite() ||
        points.cols()!=cps_.points.cols() || guide_pts_.size()<2)
      throw std::invalid_argument("uniform retime requires the same guide and control indexing");
    // Each sample is indexed by t/dt. Uniformly scaling all knot intervals
    // leaves its four cubic weights and control indices unchanged, including
    // after physical endpoint P/V/A are rebound. The frozen map/guide and their
    // supporting planes remain the same input; independent checks still own
    // execution. A new target fit continues to use initializeFromGuide.
    auto constraints=std::move(curve_clearance_constraints_);
    initializeFromGuide(points);
    curve_clearance_constraints_=std::move(constraints);
  }

  bool BsplineOptimizer::addCurveClearanceConstraints(const Eigen::MatrixXd& points,
      double interval, const std::vector<std::pair<double,GridPlanningCell>>& violations) {
    if(points.cols()<7 || !(interval>0)) return false;
    UniformBspline curve(points,3,interval);
    const size_t previous=curve_clearance_constraints_.size();
    for(const auto& [time,cell]:violations) {
      if(budget_ && budget_->expired()) return false;
      if(!cell.nearest_raw_center.allFinite() || !std::isfinite(cell.required_clearance_m)) continue;
      const double parameter=std::clamp(time/interval,0.,double(points.cols()-3));
      const int first=std::min(int(std::floor(parameter)),int(points.cols()-4));
      const double u=parameter-first;
      const Eigen::Vector4d weights(std::pow(1-u,3)/6.,
          (3*u*u*u-6*u*u+4)/6.,(-3*u*u*u+3*u*u+3*u+1)/6.,u*u*u/6.);
      double movable=0;
      for(int j=0;j<4;++j) if(first+j>=order_ && first+j<points.cols()-order_) movable+=weights[j];
      if(movable<1e-8) continue; // fixed endpoint derivatives cannot be repaired here
      Eigen::Vector3d direction=curve.evaluateDeBoorT(time)-cell.nearest_raw_center;
      if(direction.norm()<1e-8) continue;
      direction.normalize();
      // A supporting plane outside the raw-center sphere. The extra half voxel
      // is fitting reserve, not a change to the final execution threshold.
      curve_clearance_constraints_.push_back({first,weights,cell.nearest_raw_center,
          direction,cell.required_clearance_m+.5*grid_map_->getResolution()});
    }
    return curve_clearance_constraints_.size()>previous;
  }

  BsplineOptimizer::GuideRetention BsplineOptimizer::assessGuideRetention(
      const Eigen::MatrixXd& points, double interval,
      const std::function<GridPlanningRisk(const Eigen::Vector3d&)>& advisory) const {
    GuideRetention result;
    if (!grid_map_ || guide_pts_.size()<2 || points.rows()!=3 || points.cols()<7 ||
        !points.allFinite() || !std::isfinite(interval) || interval<=0 || !advisory) return result;
    UniformBspline curve(points,3,interval);
    const double resolution=grid_map_->getResolution();
    // Existing half-voxel fitting reserve plus the lattice cell's circumsphere.
    // This bounds route displacement; it does not relax execution clearance.
    result.corridor_m=guideCorridorM(resolution);
    const double speed=curve.getDerivative().getControlPoint().colwise().norm().maxCoeff();
    struct Integral { double length=0, risk=0, valid=0; bool all_valid=true, stable_cost=true; };
    std::optional<uint64_t> model_version;
    const auto integrate=[&](bool actual,double step) {
      Integral integral;
      if(budget_ && budget_->expired()) {result.budget_exhausted=true;return integral;}
      const auto sample=[&](const Eigen::Vector3d& point) {
        ++result.samples;
        const auto value=advisory(point);
        if(!model_version) {model_version=value.version;result.risk_version=value.version;}
        if(value.version!=*model_version || !std::isfinite(value.cost_multiplier) ||
           value.cost_multiplier<1) integral.stable_cost=false;
        const bool valid=value.classification==GridAdvisoryClass::VALID &&
            value.query_status==GridRiskStatus::VALID && value.version!=0 &&
            std::isfinite(value.hpl) && std::isfinite(value.vpl) &&
            value.hpl>=0 && value.vpl>=0 && value.hpl<1e9 && value.vpl<1e9 &&
            std::isfinite(value.cost_multiplier) && value.cost_multiplier>=1;
        if (!valid || value.version!=result.risk_version) integral.all_valid=false;
        if(actual) {
          result.max_deviation_m=std::max(result.max_deviation_m,
              (point-nearestGuidePoint(guide_pts_,point)).norm());
        }
        return std::make_pair(gridAdvisoryCostMultiplier(value.classification,value.cost_multiplier)-1,
            valid ? 1. : 0.);
      };
      Eigen::Vector3d previous=actual ? Eigen::Vector3d(curve.evaluateDeBoorT(0)) : guide_pts_.front();
      auto previous_value=sample(previous);
      const size_t edges=actual ? 1 : guide_pts_.size()-1;
      for(size_t edge=0;edge<edges;++edge) {
        const double measure=actual ? curve.getTimeSum()*speed :
            (guide_pts_[edge+1]-guide_pts_[edge]).norm();
        const size_t count=std::max<size_t>(1,std::ceil(measure/step));
        for(size_t i=1;i<=count;++i) {
          if(budget_ && budget_->expired()) {result.budget_exhausted=true;return integral;}
          const double fraction=double(i)/count;
          const Eigen::Vector3d point=actual ? Eigen::Vector3d(curve.evaluateDeBoorT(curve.getTimeSum()*fraction)) :
              Eigen::Vector3d(guide_pts_[edge]+fraction*(guide_pts_[edge+1]-guide_pts_[edge]));
          const auto value=sample(point);const double length=(point-previous).norm();
          integral.length+=length;integral.risk+=length*.5*(previous_value.first+value.first);
          integral.valid+=length*.5*(previous_value.second+value.second);
          previous=point;previous_value=value;
        }
      }
      return integral;
    };
    const auto guide_coarse=integrate(false,.5*resolution);
    const auto actual_coarse=integrate(true,.5*resolution);
    const auto guide=integrate(false,.25*resolution);
    const auto actual=integrate(true,.25*resolution);
    if(result.budget_exhausted) return result;
    result.checked=true;result.guide_length_m=guide.length;result.curve_length_m=actual.length;
    result.guide_risk_cost_m=guide.risk;result.curve_risk_cost_m=actual.risk;
    result.guide_valid_fraction=guide.length>0 ? guide.valid/guide.length : 0;
    result.curve_valid_fraction=actual.length>0 ? actual.valid/actual.length : 0;
    const auto mean=[](const Integral& x) {return x.length>1e-9 ? x.risk/x.length : 0.;};
    result.quadrature_uncertainty=std::abs(mean(guide)-mean(guide_coarse))+
        std::abs(mean(actual)-mean(actual_coarse));
    result.comparable_valid_risk=guide.all_valid && actual.all_valid && guide_coarse.all_valid &&
        actual_coarse.all_valid && guide.length>1e-9 && actual.length>1e-9;
    result.comparable_model_cost=guide.stable_cost && actual.stable_cost && guide_coarse.stable_cost &&
        actual_coarse.stable_cost && guide.length>1e-9 && actual.length>1e-9;
    result.route_lost=result.max_deviation_m>result.corridor_m+1e-6;
    result.risk_preference_lost=result.comparable_model_cost &&
        mean(actual)>mean(guide)+result.quadrature_uncertainty+1e-6;
    return result;
  }

  bool BsplineOptimizer::addCurveGuideConstraints(const Eigen::MatrixXd& points, double interval,
      bool route_loss, bool risk_preference_loss) {
    if(!planning_query_ || guide_pts_.size()<2 || points.cols()<7 || !(interval>0)) return false;
    UniformBspline curve(points,3,interval);
    const size_t previous=curve_clearance_constraints_.size();
    const double reserve=.5*grid_map_->getResolution();
    const double corridor=guideCorridorM(grid_map_->getResolution());
    std::vector<CurveClearanceConstraint> corridor_constraints;
    bool corridor_violated=false;
    for(double time=0;time<=curve.getTimeSum();time+=.02) {
      if(budget_ && budget_->expired()) return false;
      const Eigen::Vector3d position=curve.evaluateDeBoorT(time);
      const auto cell=planning_query_(position);
      const bool physical_boundary=cell.execution_reason==GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
          cell.execution_reason==GridExecutionReason::OUT_OF_MAP;
      const bool warning_preference=false;
      const bool preference=risk_preference_loss;
      if(!physical_boundary && !preference && !route_loss) continue;
      Eigen::Vector3d nearest=position; double best=std::numeric_limits<double>::infinity();
      double geometric_best=std::numeric_limits<double>::infinity();
      for(size_t j=1;j<guide_pts_.size();++j) {
        const Eigen::Vector3d segment=guide_pts_[j]-guide_pts_[j-1];
        const double fraction=segment.squaredNorm()>1e-12 ? std::clamp(
            (position-guide_pts_[j-1]).dot(segment)/segment.squaredNorm(),0.,1.) : 0.;
        const Eigen::Vector3d candidate=guide_pts_[j-1]+fraction*segment;
        geometric_best=std::min(geometric_best,(candidate-position).squaredNorm());
        const auto support=guide_query_(candidate);
        if(!support.executable()) continue;
        const double distance=(candidate-position).squaredNorm();
        if(distance<best) {best=distance;nearest=candidate;}
      }
      const bool geometry_only=route_loss && !physical_boundary && !preference;
      if(geometry_only) {
        const double parameter=std::clamp(time/interval,0.,double(points.cols()-3));
        const int first=std::min(int(std::floor(parameter)),int(points.cols()-4));
        const double u=parameter-first;
        const Eigen::Vector4d weights(std::pow(1-u,3)/6.,(3*u*u*u-6*u*u+4)/6.,
            (-3*u*u*u+3*u*u+3*u+1)/6.,u*u*u/6.);
        double movable=0;
        for(int j=0;j<4;++j) if(first+j>=order_ && first+j<points.cols()-order_) movable+=weights[j];
        if(movable<1e-8) continue;
        const bool violated=std::sqrt(geometric_best)>corridor+1e-6;
        corridor_violated=corridor_violated || violated;
        // The distance is recomputed against the same whole guide during the
        // solve, so crossing to the other side never satisfies a one-sided
        // supporting plane. A real route loss activates the same fitting
        // corridor for all movable samples, without imposing a centerline.
        corridor_constraints.push_back({first,weights,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
            corridor-reserve,CurveConstraintKind::GuideCorridor});
        continue;
      }
      if(!std::isfinite(best) || best<1e-12) continue;
      Eigen::Vector3d direction=(nearest-position).normalized();
      const auto farther=guide_query_(nearest+reserve*direction);
      const bool room=farther.executable();
      const double parameter=std::clamp(time/interval,0.,double(points.cols()-3));
      const int first=std::min(int(std::floor(parameter)),int(points.cols()-4));
      const double u=parameter-first;
      const Eigen::Vector4d weights(std::pow(1-u,3)/6.,(3*u*u*u-6*u*u+4)/6.,
          (-3*u*u*u+3*u*u+3*u+1)/6.,u*u*u/6.);
      double movable=0;
      for(int j=0;j<4;++j) if(first+j>=order_ && first+j<points.cols()-order_) movable+=weights[j];
      if(movable<1e-8) continue;
      // Reuse the actual-sample plane objective for unknown/map boundaries and
      // advisory preferences. Each supporting guide point is independently
      // checked; this gradient grants no observation or execution authority.
      const double correction=std::sqrt(best)+(room && (physical_boundary || warning_preference) ? reserve : 0.);
      curve_clearance_constraints_.push_back({first,weights,position,direction,correction});
    }
    if(corridor_violated) curve_clearance_constraints_.insert(curve_clearance_constraints_.end(),
        corridor_constraints.begin(),corridor_constraints.end());
    return curve_clearance_constraints_.size()>previous;
  }

  void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d> &guide)
  {
    guide_pts_ = guide;
    guide_tracking_ = false;
    ref_pts_.clear();
    if (guide.size() < 2 || cps_.size < 7) return;
    vector<double> arc(guide.size(), 0.0);
    for (size_t i = 1; i < guide.size(); ++i)
      arc[i] = arc[i - 1] + (guide[i] - guide[i - 1]).norm();
    if (arc.back() < 1e-6) return;
    ref_pts_.reserve(cps_.size);
    size_t segment = 1;
    for (int i = 0; i < cps_.size; ++i) {
      // calcFitnessCost compares an actual cubic knot position at t=i*dt
      // with ref_pts_[i]. The spline has N-3 spans, not N-1 control-point
      // intervals. Using the latter pulls the curve backward along the guide
      // and can cut an otherwise legal advisory detour during smoothing.
      const double distance = arc.back() * std::min(i, cps_.size - 3) / (cps_.size - 3);
      while (segment + 1 < arc.size() && arc[segment] < distance) ++segment;
      const double length = arc[segment] - arc[segment - 1];
      const double alpha = length > 1e-8 ?
          (distance - arc[segment - 1]) / length : 0.0;
      ref_pts_.push_back(guide[segment - 1] * (1.0 - alpha) +
                         guide[segment] * alpha);
    }
    guide_tracking_ = true;
  }

  void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

  void BsplineOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void BsplineOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  // 返回多个安全的控制点集
  std::vector<ControlPoints> BsplineOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;                                                                            // 最多的轨迹数量
    constexpr int VARIS = 2;                                                                                // 允许的变化种类数
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS)))); // 允许变换的片段数量上限
    std::vector<ControlPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.size - 1)).norm() / (cps_.size - 1); // 计算控制点间的平均距离

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ControlPoints, ControlPoints>> RichInfoSegs;
    // 初始化两套控制点信息
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ControlPoints, ControlPoints> RichInfoOneSeg;
      ControlPoints RichInfoOneSeg_temp;
      // 获取指定片段的控制点信息
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);

      // cout << "RichInfoOneSeg_temp, out" << endl;
      // cout << "RichInfoSegs[" << i << "].first" << endl;
      // for ( int k=0; k<RichInfoOneSeg_temp.size; k++ )
      //   if ( RichInfoOneSeg_temp.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << RichInfoOneSeg_temp.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < RichInfoOneSeg_temp.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << RichInfoOneSeg_temp.base_point[k][k2].transpose() << " @ " << RichInfoOneSeg_temp.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.size - 1; j++)
        {
          // cout << "A *" << j << "*" << endl;
          //  遍历每个控制点及其后一个点，在两点间通过线性插值生成采样点
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            // cout << " " << planningOccupied(pt) << " pt=" << pt.transpose() << endl;
            //  如果检测到在障碍物内，则存储对应数据
            if (planningOccupied(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        // 查找片段与最后一个障碍物的交点
        for (int j = RichInfoSegs[i].first.size - 1; j >= 1; j--)
        {
          // cout << "j=" << j << endl;
          // cout << "B *" << j << "*" << endl;
          ;
          // 和上面同样的采样，然后检测是否在障碍物中
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            // cout << " " << planningOccupied(pt) << " pt=" << pt.transpose() << endl;
            ;
            if (planningOccupied(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        // 如果片段的起点或者终点在障碍物中，将会被移除
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;

          // cout << "RichInfoSegs[" << i << "].first" << endl;
          // for (int k = 0; k < RichInfoSegs[i].first.size; k++)
          // {
          //   if (RichInfoSegs[i].first.base_point.size() > 0)
          //   {
          //     cout << "###" << RichInfoSegs[i].first.points.col(k).transpose() << endl;
          //     for (int k2 = 0; k2 < RichInfoSegs[i].first.base_point[k].size(); k2++)
          //     {
          //       cout << "      " << RichInfoSegs[i].first.base_point[k][k2].transpose() << " @ " << RichInfoSegs[i].first.direction[k][k2].transpose() << endl;
          //     }
          //   }
          // }
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          // 检查控制点的base point是否为1
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << cps_.clearance << " cps.size=" << cps_.size << endl;
            // 输出错误信息
            for (int temp_i = 0; temp_i < cps_.size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"), "Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ControlPoints> blank;
            return blank;
          }

          // 通过取反获得相反方向的向量
          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          // 若当前控制点为片段的起始点 occ_start_id，则将障碍物交点 occ_start_pt 直接设为 base_pt_reverse
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          // 若当前控制点为片段的终止点 occ_end_id，则将终点交点 occ_end_pt 设为 base_pt_reverse
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          // 对于片段中的中间控制点，将基准点 base_pt_reverse 设置为当前控制点 points.col(j) 沿反向向量 base_vec_reverse 方向延伸的某一距离位置
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          // 检查base_pt_reverse是否在障碍物中
          if (planningOccupied(base_pt_reverse)) // Search outward.
          {
            // 最大搜索范围
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              // 不断将控制点向外移动，寻找不在障碍物中的控制点
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              // cout << base_pt_temp.transpose() << endl;
              if (!planningOccupied(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            // 如果找不到则删除这一段
            if (l > l_upbound)
            {
              RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          // 如果距离控制点足够远且不再障碍物中则无需继续搜索
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          // 基点和控制点太近则删除这一段
          else
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"), "base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.size)
        {
          // 为片段起点之前和终点之后的控制点设置统一的基准点和方向，使得这些控制点在障碍物影响范围外时能够保持一致的路径属性
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      // 片段只有一个控制点的情况
      else if (RichInfoSegs[i].first.size == 1)
      {
        cout << "i=" << i << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        cout << "RichInfoSegs[i].first.size=" << RichInfoSegs[i].first.size << endl;
        cout << "RichInfoSegs[i].first.direction.size()=" << RichInfoSegs[i].first.direction.size() << endl;
        cout << "RichInfoSegs[i].first.direction[0].size()=" << RichInfoSegs[i].first.direction[0].size() << endl;
        cout << "RichInfoSegs[i].first.points.cols()=" << RichInfoSegs[i].first.points.cols() << endl;
        cout << "RichInfoSegs[i].first.base_point.size()=" << RichInfoSegs[i].first.base_point.size() << endl;
        cout << "RichInfoSegs[i].first.base_point[0].size()=" << RichInfoSegs[i].first.base_point[0].size() << endl;
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (planningOccupied(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            // cout << base_pt_temp.transpose() << endl;
            if (!planningOccupied(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"),
                        "Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("distinctiveTrajs"),
                        "base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
      else
      {
        segments.erase(segments.begin() + i);
        RichInfoSegs.erase(RichInfoSegs.begin() + i);
        seg_upbound--;
        i--;
      }
    }
    // cout << "A3" << endl;

    // Step 2. Assemble each segment to make up the new control point sequence.
    // 将每个分段组合起来，组成新的控制点序列
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ControlPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // 初始化选择向量
    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    // 计算最大组合数
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      // 生成一个选择表
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"),
                        "Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);

        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ControlPoints cpsOneSample;
      cpsOneSample.resize(cps_.size);
      cpsOneSample.clearance = cps_.clearance;
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      // 遍历所有控制点
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.size)
      {
        // cout << "A ";
        //  if ( seg_id >= RichInfoSegs.size() )
        //  {
        //    cout << "seg_id=" << seg_id << " RichInfoSegs.size()=" << RichInfoSegs.size() << endl;
        //  }
        //  if ( cp_id >= cps_.base_point.size() )
        //  {
        //    cout << "cp_id=" << cp_id << " cps_.base_point.size()=" << cps_.base_point.size() << endl;
        //  }
        //  if ( cp_of_seg_id >= RichInfoSegs[seg_id].first.base_point.size() )
        //  {
        //    cout << "cp_of_seg_id=" << cp_of_seg_id << " RichInfoSegs[seg_id].first.base_point.size()=" << RichInfoSegs[seg_id].first.base_point.size() << endl;
        //  }
        //  判断控制点是否在当前控制范围内
        //  如果不在则直接从原始控制点集中复制数据
        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        // 如果 cp_id 位于当前片段范围内，根据 selection[seg_id] 的值选择片段的第一套或第二套基准点和方向
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          // 当遍历到片段的最后一个控制点时，将 cp_of_seg_id 重置为 0，并将 seg_id 指向下一个片段
          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("distinctiveTrajs"),
                    "Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  } // namespace ego_planner

  /* This function is very similar to check_collision_and_rebound().
   * It was written separately, just because I did it once and it has been running stably since March 2020.
   * But I will merge then someday.*/
  // 初始化控制点
  std::optional<BsplineOptimizer::RepairEndpoints>
  BsplineOptimizer::chooseRepairEndpoints(const Eigen::MatrixXd& points,
      const int segment_start, const int segment_end,
      AStar::Failure& failure) const
  {
    const auto expired = [&]() {
      if (budget_ && budget_->expired()) { failure = AStar::Failure::TIME_BUDGET; return true; }
      return false;
    };
    failure = AStar::Failure::NONE;
    if (!planning_query_ || points.cols() < 2 || segment_start < 0 ||
        segment_end >= points.cols() || segment_start >= segment_end)
      return std::nullopt;
    const double spacing = grid_map_->getResolution() * 0.5;
    std::vector<Eigen::Vector3d> samples;
    std::vector<int> control_sample(points.cols());
    samples.emplace_back(points.col(0));
    control_sample[0] = 0;
    for (int j = 1; j < points.cols(); ++j) {
      if (expired()) return std::nullopt;
      const Eigen::Vector3d a = points.col(j - 1), b = points.col(j);
      const int count = std::max(1, static_cast<int>(std::ceil(
          (b - a).norm() / spacing)));
      for (int k = 1; k <= count; ++k) {
        if (expired()) return std::nullopt;
        samples.push_back(a + (b - a) * (static_cast<double>(k) / count));
      }
      control_sample[j] = static_cast<int>(samples.size()) - 1;
    }
    const auto original_center =
        (points.col(segment_start) + points.col(segment_end)) / 2.0;
    const auto in_pool = [&](const Eigen::Vector3d& p) {
      return ((p - original_center).array().abs() < 4.9).all();
    };
    const auto lattice_connector_valid = [&](const Eigen::Vector3d& p) {
      Eigen::Vector3i index;
      for (int axis = 0; axis < 3; ++axis) {
        index[axis] = static_cast<int>((p[axis] - original_center[axis]) /
            0.1 + 0.5) + 50;
        if (index[axis] < 1 || index[axis] >= 99) return false;
      }
      const Eigen::Vector3d lattice = original_center +
          (index - Eigen::Vector3i::Constant(50)).cast<double>() * 0.1;
      const int count = std::max(1, static_cast<int>(std::ceil(
          (lattice - p).norm() / spacing)));
      for (int k = 0; k <= count; ++k) {
        if (expired()) return false;
        if (!planning_query_(p + (lattice - p) *
            (static_cast<double>(k) / count)).executable()) return false;
      }
      return true;
    };
    std::vector<bool> clear(samples.size()), prefix(samples.size()),
        suffix(samples.size());
    bool unknown_exit = false;
    for (size_t i = 0; i < samples.size(); ++i) {
      if (expired()) return std::nullopt;
      const auto cell = planning_query_(samples[i]);
      clear[i] = in_pool(samples[i]) && cell.executable();
      if (i + 1 == samples.size() &&
          (cell.execution_reason == GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
           cell.execution_reason == GridExecutionReason::OUT_OF_MAP))
        unknown_exit = true;
      prefix[i] = clear[i] && (i == 0 || prefix[i - 1]);
    }
    for (size_t i = samples.size(); i-- > 0;)
      suffix[i] = clear[i] && (i + 1 == samples.size() || suffix[i + 1]);
    int entry = -1, exit = -1;
    for (int i = control_sample[segment_start]; i >= 0; --i)
      if (prefix[i] && lattice_connector_valid(samples[i])) {
        entry = i; break;
      }
    for (size_t i = control_sample[segment_end]; i < samples.size(); ++i)
      if (suffix[i] && lattice_connector_valid(samples[i])) {
        exit = static_cast<int>(i); break;
      }
    if (expired()) return std::nullopt;
    if (entry < 0) failure = AStar::Failure::NO_VALID_REPAIR_ENTRY;
    else if (exit < 0) failure = unknown_exit
        ? AStar::Failure::END_UNOBSERVED
        : AStar::Failure::NO_VALID_REPAIR_EXIT;
    if (failure != AStar::Failure::NONE) return std::nullopt;
    RepairEndpoints result;
    result.entry = samples[entry];
    result.exit = samples[exit];
    for (int j = 0; j < points.cols(); ++j) {
      if (control_sample[j] < entry)
        result.prefix.emplace_back(points.col(j));
      if (control_sample[j] > exit)
        result.suffix.emplace_back(points.col(j));
    }
    return result;
  }

  std::vector<std::pair<int, int>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
  {

    if(planning_query_ && planning_endpoints_) {
      initializeFromGuide(init_points);
      if(curveViolates(init_points,bspline_interval_) && guide_pts_.empty()) {
        if(!searchRecoveryGuide()) initialization_failed_=true;
        return {{0,static_cast<int>(init_points.cols())-1}};
      }
      return {};
    }

    bool unknown_guess = false;
    if (planning_query_) {
      for (int i = 0; i < init_points.cols(); ++i) {
        if (budget_ && budget_->expired()) { initialization_failed_ = true; return {}; }
        const auto cell = planning_query_(init_points.col(i));
        unknown_guess |= cell.execution_reason == GridExecutionReason::ENVIRONMENT_UNOBSERVED ||
                         cell.execution_reason == GridExecutionReason::OUT_OF_MAP;
        if (cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE ||
            (cell.execution_reason != GridExecutionReason::OK &&
             cell.execution_reason != GridExecutionReason::ENVIRONMENT_UNOBSERVED &&
             cell.execution_reason != GridExecutionReason::OUT_OF_MAP &&
             cell.execution_reason != GridExecutionReason::PHYSICAL_OBSTACLE &&
             cell.execution_reason != GridExecutionReason::INSUFFICIENT_CLEARANCE)) {
          a_star_->recordPresearchFailure(
              cell.execution_reason == GridExecutionReason::ENVIRONMENT_STALE
                  ? AStar::Failure::END_STALE : AStar::Failure::CURRENT_MOTION,
              init_points.col(i), init_points.col(init_points.cols() - 1));
          reportSearchFailure(a_star_->lastResult(), init_points, i,
                              init_points.cols() - 1, "initial_control_points_input");
          initialization_failed_ = true; return {};
        }
      }
    }

    if (flag_first_init)
    {
      cps_.clearance = dist0_;
      if (planning_query_) {
        const auto start_cell = planning_query_(init_points.col(0));
        if (std::isfinite(start_cell.required_clearance_m))
          cps_.clearance = std::max(cps_.clearance,
                                    start_cell.required_clearance_m);
      }
      cps_.resize(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    // 进入或离开障碍物稳定的时间间隔
    constexpr int ENOUGH_INTERVAL = 2;
    // 障碍物检测的步长
    double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
    int in_id = -1, out_id = -1;
    vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    // 标识片段的起点和终点是否找到
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = planning_query_ ? (int)init_points.cols() - order_ - 1 :
        (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3;
    // 遍历所有点
    for (int i = order_; i <= i_end; ++i)
    {
      if (budget_ && budget_->expired()) { initialization_failed_ = true; return {}; }
      // cout << " *" << i-1 << "*" ;
      //  相邻两个点之间进行线性插值并检测障碍物
      for (double a = 1.0; a > 0.0; a -= step_size)
      {
        // TODO:没搞懂这是干嘛的
        if (budget_ && budget_->expired()) { initialization_failed_ = true; return {}; }
        occ = planningOccupied(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        // cout << " " << occ;
        //  cout << setprecision(5);
        //  cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

        // 进入障碍物
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        // 离开障碍物
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        // 如果状态没发生变化
        else
        {
          ++same_occ_state_times;
        }

        // 如果已经离开障碍物，则结束
        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        // 重置标志位并存储信息
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }
    // A pure advisory band may continue to the local target and have no
    // physical-style exit. Search the bounded local endpoint in that case.
    if (flag_got_start && !flag_got_end)
      segment_ids.emplace_back(in_id, i_end);
    // cout << endl;

    // for (size_t i = 0; i < segment_ids.size(); i++)
    // {
    //   cout << "segment_ids=" << segment_ids[i].first << " ~ " << segment_ids[i].second << endl;
    // }

    // return in advance
    if (segment_ids.size() == 0)
    {
      vector<std::pair<int, int>> blank_ret;
      return blank_ret;
    }

    /*** a star search ***/
    // 在每个无障碍片段 segment_ids 的起点和终点之间寻找一条路径
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    const Eigen::Vector3d original_in(init_points.col(segment_ids.front().first));
    const Eigen::Vector3d original_out(init_points.col(segment_ids.back().second));
    AStar::Failure endpoint_failure;
    auto endpoints = chooseRepairEndpoints(init_points,
        segment_ids.front().first, segment_ids.back().second,
        endpoint_failure);
    if (!endpoints && endpoint_failure != AStar::Failure::TIME_BUDGET &&
        unknown_guess && planning_endpoints_) {
      const auto& [start, target] = *planning_endpoints_;
      if (planning_query_(start).executable() && planning_query_(target).executable()) {
        RepairEndpoints whole;
        whole.entry = start; whole.exit = target;
        endpoints = whole; guide_reinitialization_ = true;
      }
    }
    if (planning_query_ && !endpoints) {
      a_star_->recordPresearchFailure(endpoint_failure, original_in, original_out);
      reportSearchFailure(a_star_->lastResult(), init_points,
          segment_ids.front().first, segment_ids.back().second,
          "initial_control_points");
      initialization_failed_ = true;
      return {};
    }
    const Eigen::Vector3d in = endpoints ? endpoints->entry : original_in;
    const Eigen::Vector3d out = endpoints ? endpoints->exit : original_out;
    const Eigen::Vector3d pool_center = guide_reinitialization_
        ? (in + out) / 2.0 : (original_in + original_out) / 2.0;
    bool found = a_star_->AstarSearch(0.1, in, out, -1.0, pool_center);

    if (!found) {
      reportSearchFailure(a_star_->lastResult(), init_points,
                          segment_ids.front().first, segment_ids.back().second,
                          "initial_control_points");
      static rclcpp::Clock failure_clock(RCL_SYSTEM_TIME);
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("initControlPoints"),
                  failure_clock, 1000,
                  "One-guide A* %s from (%.2f %.2f %.2f) to (%.2f %.2f %.2f), segments=%zu, advisory_rejected=%d fallback=%d",
                  AStar::failureName(a_star_->lastResult().failure),
                  in.x(), in.y(), in.z(), out.x(), out.y(), out.z(),
                  segment_ids.size(), a_star_->rejectedAdvisory(),
                  planning_advisory_fallback_);
      initialization_failed_ = true;
      return {};
    }
    const auto one_guide = a_star_->getPath();
    if (one_guide.size() < 2) {
      initialization_failed_ = true;
      return {};
    }
    vector<Eigen::Vector3d> full_guide;
    if (endpoints) full_guide = endpoints->prefix;
    else for (int j = 0; j < segment_ids.front().first; ++j)
      full_guide.push_back(init_points.col(j));
    full_guide.insert(full_guide.end(), one_guide.begin(), one_guide.end());
    if (endpoints)
      full_guide.insert(full_guide.end(), endpoints->suffix.begin(),
                        endpoints->suffix.end());
    else for (int j = segment_ids.back().second + 1; j < init_points.cols(); ++j)
      full_guide.push_back(init_points.col(j));
    setGuidePath(full_guide);
    if (guide_reinitialization_) return segment_ids;
    a_star_pathes.assign(segment_ids.size(), one_guide);

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    // 遍历每一段，设置边界
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          // 取当前片段结束点 segment_ids[0].second 和下一个片段起始点 segment_ids[1].first 的中间位置，向下取整后作为高边界
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          // 距末尾 order_ + 1 个点的位置
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      // 末尾的边界
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        // 尾段的低边界为当前片段起点和上一个片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 距末尾点 order_ + 1 个点
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        // 低边界：为当前片段起点和前一片段终点的中间位置，向上取整
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        // 高边界：为当前片段终点和下一片段起点的中间位置，向下取整
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    // 控制点的最小比例
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    // 控制点的最小数量
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      // 获取当前片段的点数
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        // 如果点数不够则在两侧扩展点，确保不能超过边界
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }

      // cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
    }
    // 避免重叠
    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      // 遍历该段所有控制点的id，并将标志位置为false
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      // 初始化交点标记
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        // 计算控制点的方向向量
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        // A*路径的中点
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        // 路径点和控制点之间的点积，判断当前路径点在控制点方向 ctrl_pts_law 的哪一侧
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          // 根据 val 的正负决定 Astar_id 的移动方向
          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= (int)a_star_pathes[i].size()) break;
          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 寻找可能的交点，TODO:还没搞懂
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          // 计算交点到控制点之间的距离
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            // 逐步进行采样
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              // 通过线性插值计算采样点位置
              occ = planningOccupied((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                // 记录基点并计算到控制点的方向
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                // cout << "A " << j << endl;
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      // 当轨迹片段只有两个控制点（即长度过短）时，A* 路径的方向可能与控制点的方向不一致，导致梯度方向相反
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        // 计算控制点方向向量和中点
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        // 计算A*中点并判断方向，同上
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
        {
          last_Astar_id = Astar_id;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          // 和上面不一样，这里减去的是中点
          if (Astar_id < 0 || Astar_id >= (int)a_star_pathes[i].size()) break;
          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
          {
            // 计算交点
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            // 满足距离要求则存储相关信息
            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      // step 3
      if (got_intersection_id >= 0)
      {
        // 遍历交点之后的控制点
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          // 如果没被标记则将前一个控制点的信息给该点
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
            // cout << "AAA " << j << endl;
          }

        // 遍历交点前的控制点，如果没被标记用后一个控制点的数据赋值
        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
            // cout << "AAAA " << j << endl;
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    return final_segment_ids;
  }

  // 急停情况下提前退出
  int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
    // cout << "k=" << k << endl;
    // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
    if (opt->budget_ && opt->budget_->expired()) return 1;
    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  // 利用combineCostRebound计算损失
  double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRebound(x, grad, cost, n);
    // Retain the best admissible point of this solve under its original
    // objective. Smoothness alone can cut across unobserved voxel gaps even
    // when the initial fit is physical; the solver output must retain that
    // physical constraint. This is candidate selection, never authorization.
    if(opt->planning_query_ && opt->planning_endpoints_ && std::isfinite(cost) &&
        cost<opt->physical_incumbent_cost_ && !opt->curveViolates(opt->cps_.points,opt->bspline_interval_)) {
      opt->physical_incumbent_cost_=cost;opt->physical_incumbent_=opt->cps_.points;
    }
    opt->iter_num_ += 1;
    return cost;
  }

  // 利用combineCostRefine计算优化后的损失
  double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
  {
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

    double cost;
    opt->combineCostRefine(x, grad, cost, n);

    opt->iter_num_ += 1;
    return cost;
  }

  // 几个计算损失的函数
  void BsplineOptimizer::calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_ - (double)(q.cols() - 2 * order_) * 1.0 / 3.0; // Only check the first 2/3 points
    const double CLEARANCE = swarm_clearance_ * 2;
    double t_now = clock_->now().seconds();
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    for (int i = order_; i < end_idx; i++)
    {
      double glb_time = t_now + ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (size_t id = 0; id < swarm_trajs_->size(); id++)
      {
        if ((swarm_trajs_->at(id).drone_id != (int)id) || swarm_trajs_->at(id).drone_id == drone_id_)
        {
          continue;
        }

        double traj_i_satrt_time = swarm_trajs_->at(id).start_time_.seconds();
        if (glb_time < traj_i_satrt_time + swarm_trajs_->at(id).duration_ - 0.1)
        {
          /* def cost=(c-sqrt([Q-O]'D[Q-O]))^2, D=[1/b^2,0,0;0,1/b^2,0;0,0,1/a^2] */
          Eigen::Vector3d swarm_prid = swarm_trajs_->at(id).position_traj_.evaluateDeBoorT(glb_time - traj_i_satrt_time);
          Eigen::Vector3d dist_vec = cps_.points.col(i) - swarm_prid;
          double ellip_dist = sqrt(dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2);
          double dist_err = CLEARANCE - ellip_dist;

          Eigen::Vector3d dist_grad = cps_.points.col(i) - swarm_prid;
          Eigen::Vector3d Coeff;
          Coeff(0) = -2 * (CLEARANCE / ellip_dist - 1) * inv_b2;
          Coeff(1) = Coeff(0);
          Coeff(2) = -2 * (CLEARANCE / ellip_dist - 1) * inv_a2;

          if (dist_err < 0)
          {
            /* do nothing */
          }
          else
          {
            cost += pow(dist_err, 2);
            gradient.col(i) += (Coeff.array() * dist_grad.array()).matrix();
          }

          if (min_ellip_dist_ > dist_err)
          {
            min_ellip_dist_ = dist_err;
          }
        }
      }
    }
  }

  void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    constexpr double CLEARANCE = 1.5;
    double t_now = clock_->now().seconds();

    for (int i = order_; i < end_idx; i++)
    {
      double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

      for (int id = 0; id < moving_objs_->getObjNums(); id++)
      {
        Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
        double dist = (cps_.points.col(i) - obj_prid).norm();
        // cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
        double dist_err = CLEARANCE - dist;
        Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else
        {
          cost += pow(dist_err, 2);
          gradient.col(i) += -2.0 * dist_err * dist_grad;
        }
      }
      // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
      // cout << "--" << endl;
    }
    // cout << "---------------" << endl;
  }

  void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
  {
    cost = 0.0;
    int end_idx = q.cols() - order_;
    double demarcation = cps_.clearance;
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    calcCurvePhysicalCost(q,cost,gradient);
    for (auto i = order_; i < end_idx; ++i)
    {
      for (size_t j = 0; j < cps_.direction[i].size(); ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector3d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += pow(dist_err, 3);
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
  }

  void BsplineOptimizer::calcCurvePhysicalCost(const Eigen::MatrixXd& q,
      double& cost, Eigen::MatrixXd& gradient) {
    // Constrain the actual spline sample, not its control polygon. Quadratic
    // penalties retain a useful gradient for sub-millimetre violations.
    const double weight=100.*std::max(1.,lambda1_/std::max(1e-6,lambda2_));
    for(const auto& constraint:curve_clearance_constraints_) {
      Eigen::Vector3d position=Eigen::Vector3d::Zero();
      for(int j=0;j<4;++j) position+=constraint.weights[j]*q.col(constraint.first_control+j);
      if(constraint.kind==CurveConstraintKind::GuideCorridor) {
        const Eigen::Vector3d offset=position-nearestGuidePoint(guide_pts_,position);
        const double distance=offset.norm(),deficit=distance-constraint.clearance;
        if(deficit<=0) continue;
        // This is guide tracking, so it consumes the same existing budgeted
        // guide weight as knot tracking. Physical supporting planes below
        // retain their original independent weight.
        const double sample_weight=weight*guide_weight_;
        // Independent retention limits the largest deviation. Emphasize a
        // concentrated peak instead of trading it for many smaller residuals;
        // the existing half-voxel reserve supplies the dimensional scale.
        const double reserve=.5*grid_map_->getResolution();
        const double squared_ratio=deficit*deficit/(reserve*reserve);
        cost+=sample_weight*squared_ratio*deficit*deficit;
        for(int j=0;j<4;++j) gradient.col(constraint.first_control+j)+=
            4*sample_weight*squared_ratio*deficit*constraint.weights[j]*offset/distance;
        continue;
      }
      const double deficit=constraint.clearance-(position-constraint.center).dot(constraint.direction);
      if(deficit<=0) continue;
      cost+=weight*deficit*deficit;
      for(int j=0;j<4;++j) gradient.col(constraint.first_control+j)-=
          2*weight*deficit*constraint.weights[j]*constraint.direction;
    }
    if(!curve_bounds_) return;
    for(int first=0;first+3<q.cols();++first) {
      if(budget_ && budget_->expired()) return;
      for(int axis=0;axis<3;++axis) {
        const double p0=q(axis,first),p1=q(axis,first+1),p2=q(axis,first+2),p3=q(axis,first+3);
        const double a=(-p0+3*p1-3*p2+p3)/6.,b=(p0-2*p1+p2)/2.,c=(p2-p0)/2.;
        std::vector<double> extrema{0.,1.};
        if(std::abs(a)<1e-14) {
          if(std::abs(b)>1e-14) extrema.push_back(-c/(2*b));
        } else {
          const double discriminant=4*b*b-12*a*c;
          if(discriminant>=0) {
            extrema.push_back((-2*b+std::sqrt(discriminant))/(6*a));
            extrema.push_back((-2*b-std::sqrt(discriminant))/(6*a));
          }
        }
        for(double u:extrema) {
          if(u<0 || u>1) continue;
          const Eigen::Vector4d w(std::pow(1-u,3)/6.,(3*u*u*u-6*u*u+4)/6.,
              (-3*u*u*u+3*u*u+3*u+1)/6.,u*u*u/6.);
          double value=0;
          for(int j=0;j<4;++j) {
            value+=w[j]*q(axis,first+j);
          }
          // Fitting reserve tapers to fixed legal endpoint P/V/A. It does not
          // change physical clearance or clip the submitted trajectory.
          const double reserve=.5*grid_map_->getResolution()*
              std::min({1.,double(first),double(q.cols()-4-first)});
          const double lower=curve_bounds_->first[axis]+reserve;
          const double upper=curve_bounds_->second[axis]-reserve;
          const double delta=value<lower ? value-lower : value>upper ? value-upper : 0.;
          cost+=weight*delta*delta;
          for(int j=0;j<4;++j) gradient(axis,first+j)+=2*weight*delta*w[j];
        }
      }
    }
  }

  void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {

    cost = 0.0;

    int end_idx = q.cols() - order_;

    // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
    double a2 = 25, b2 = 1;
    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
      Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
      Eigen::Vector3d tangent = ref_pts_[i] - ref_pts_[i - 2];
      if (tangent.norm() < 1e-8) continue;
      Eigen::Vector3d v = tangent.normalized();

      double xdotv = x.dot(v);
      Eigen::Vector3d xcrossv = x.cross(v);

      double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
      cost += f;

      Eigen::Matrix3d m;
      m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
      Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

      gradient.col(i - 1) += df_dx / 6;
      gradient.col(i) += 4 * df_dx / 6;
      gradient.col(i + 1) += df_dx / 6;
    }
  }

  void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
  {

    cost = 0.0;

    if (falg_use_jerk)
    {
      Eigen::Vector3d jerk, temp_j;

      for (int i = 0; i < q.cols() - 3; i++)
      {
        /* evaluate jerk */
        jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
        cost += jerk.squaredNorm();
        temp_j = 2.0 * jerk;
        /* jerk gradient */
        gradient.col(i + 0) += -temp_j;
        gradient.col(i + 1) += 3.0 * temp_j;
        gradient.col(i + 2) += -3.0 * temp_j;
        gradient.col(i + 3) += temp_j;
      }
    }
    else
    {
      Eigen::Vector3d acc, temp_acc;

      for (int i = 0; i < q.cols() - 2; i++)
      {
        /* evaluate acc */
        acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
        cost += acc.squaredNorm();
        temp_acc = 2.0 * acc;
        /* acc gradient */
        gradient.col(i + 0) += temp_acc;
        gradient.col(i + 1) += -2.0 * temp_acc;
        gradient.col(i + 2) += temp_acc;
      }
    }
  }

  void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
  {
    cost = 0.0;

    // zero cost and gradient in hard constraints
    Eigen::Vector3d q_3, q_2, q_1, dq;
    q_3 = q.col(q.cols() - 3);
    q_2 = q.col(q.cols() - 2);
    q_1 = q.col(q.cols() - 1);

    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
  }

  void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
  {

    // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

    cost = 0.0;
    double demarcation = 1.0; // 1m/s, 1m/s/s
    double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
    double al = ar, bl = -br, cl = cr;

    /* abbreviation */
    double ts, ts_inv2, ts_inv3;
    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    ts_inv3 = 1 / ts / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_ + demarcation)
        {
          double diff = vi(j) - max_vel_;
          cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

          double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) > max_vel_)
        {
          double diff = vi(j) - max_vel_;
          cost += pow(diff, 3) * ts_inv3;
          ;

          double grad = 3 * diff * diff / ts * ts_inv3;
          ;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -(max_vel_ + demarcation))
        {
          double diff = vi(j) + max_vel_;
          cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

          double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else if (vi(j) < -max_vel_)
        {
          double diff = vi(j) + max_vel_;
          cost += -pow(diff, 3) * ts_inv3;

          double grad = -3 * diff * diff / ts * ts_inv3;
          gradient(j, i + 0) += -grad;
          gradient(j, i + 1) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_ + demarcation)
        {
          double diff = ai(j) - max_acc_;
          cost += ar * diff * diff + br * diff + cr;

          double grad = (2.0 * ar * diff + br) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) > max_acc_)
        {
          double diff = ai(j) - max_acc_;
          cost += pow(diff, 3);

          double grad = 3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -(max_acc_ + demarcation))
        {
          double diff = ai(j) + max_acc_;
          cost += al * diff * diff + bl * diff + cl;

          double grad = (2.0 * al * diff + bl) * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else if (ai(j) < -max_acc_)
        {
          double diff = ai(j) + max_acc_;
          cost += -pow(diff, 3);

          double grad = -3 * diff * diff * ts_inv2;
          gradient(j, i + 0) += grad;
          gradient(j, i + 1) += -2 * grad;
          gradient(j, i + 2) += grad;
        }
        else
        {
          /* nothing happened */
        }
      }
    }

#else

    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;

    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

      // cout << "temp_v * vi=" ;
      for (int j = 0; j < 3; j++)
      {
        if (vi(j) > max_vel_)
        {
          // cout << "zx-todo VEL" << endl;
          // cout << vi(j) << endl;
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }

    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      // cout << "temp_a * ai=" ;
      for (int j = 0; j < 3; j++)
      {
        if (ai(j) > max_acc_)
        {
          // cout << "zx-todo ACC" << endl;
          // cout << ai(j) << endl;
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
      // cout << endl;
    }

#endif
  }
  /*                    上面七个都是计算损失的函数           */

  // 检查是否有障碍物？
  bool BsplineOptimizer::check_collision_and_rebound(void)
  {

    // Normal planning checks the actual curve at the manager boundary. Control
    // polygon probes cannot authorize/reject recovery or replace a whole guide.
    if (planning_query_ && planning_endpoints_) return false;

    int end_idx = cps_.size - order_;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = planning_query_ ? end_idx - 1 :
        end_idx - (end_idx - order_) / 3;
    for (int i = order_ - 1; i <= i_end; ++i)
    {

      bool occ = planningOccupied(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          cout.precision(2);
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false; // Not really takes effect, just for better hunman understanding.
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = planningOccupied(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          RCLCPP_ERROR(rclcpp::get_logger("check_collision_and_rebound"), "ERROR! the drone is in obstacle. This should not happen.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.size; ++j)
        {
          occ = planningOccupied(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.size) // Advisory warnings can extend to the endpoint.
        {
          if (!planning_query_) {
            force_stop_type_ = STOP_FOR_ERROR;
            return false;
          }
          out_id = end_idx - 1;
          j = out_id;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      const Eigen::Vector3d original_in(cps_.points.col(segment_ids.front().first));
      const Eigen::Vector3d original_out(cps_.points.col(segment_ids.back().second));
      AStar::Failure endpoint_failure;
      const auto endpoints = chooseRepairEndpoints(cps_.points,
          segment_ids.front().first, segment_ids.back().second,
          endpoint_failure);
      if (planning_query_ && !endpoints) {
        a_star_->recordPresearchFailure(endpoint_failure, original_in,
                                        original_out);
        reportSearchFailure(a_star_->lastResult(), cps_.points,
                            segment_ids.front().first,
                            segment_ids.back().second,
                            "rebound_collision_check");
        force_stop_type_ = STOP_FOR_ERROR;
        return false;
      }
      const Eigen::Vector3d in = endpoints ? endpoints->entry : original_in;
      const Eigen::Vector3d out = endpoints ? endpoints->exit : original_out;
      const Eigen::Vector3d pool_center = (original_in + original_out) / 2.0;
      bool found = a_star_->AstarSearch(0.1, in, out, -1.0, pool_center);

      if (!found) {
        reportSearchFailure(a_star_->lastResult(), cps_.points,
                            segment_ids.front().first, segment_ids.back().second,
                            "rebound_collision_check");
        static rclcpp::Clock recheck_failure_clock(RCL_SYSTEM_TIME);
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("reboundCollisionCheck"),
            recheck_failure_clock, 1000,
            "Rebound A* %s from (%.2f %.2f %.2f) to (%.2f %.2f %.2f)",
            AStar::failureName(a_star_->lastResult().failure),
            in.x(), in.y(), in.z(), out.x(), out.y(), out.z());
        force_stop_type_ = STOP_FOR_ERROR;
        return false;
      }
      const auto one_guide = a_star_->getPath();
      if (one_guide.size() < 2) {
        force_stop_type_ = STOP_FOR_ERROR;
        return false;
      }
      vector<Eigen::Vector3d> full_guide;
      if (endpoints) full_guide = endpoints->prefix;
      else for (int j = 0; j < segment_ids.front().first; ++j)
        full_guide.push_back(cps_.points.col(j));
      full_guide.insert(full_guide.end(), one_guide.begin(), one_guide.end());
      if (endpoints)
        full_guide.insert(full_guide.end(), endpoints->suffix.begin(),
                          endpoints->suffix.end());
      else for (int j = segment_ids.back().second + 1; j < cps_.size; ++j)
        full_guide.push_back(cps_.points.col(j));
      setGuidePath(full_guide);
      a_star_pathes.assign(segment_ids.size(), one_guide);

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
          while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
          {
            last_Astar_id = Astar_id;

            if (val >= 0)
              --Astar_id;
            else
              ++Astar_id;

            if (Astar_id < 0 || Astar_id >= (int)a_star_pathes[i].size()) break;
            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            // cout << val << endl;

            if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = planningOccupied((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        // step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          RCLCPP_WARN(rclcpp::get_logger("check_collision_and_rebound"), "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  // 设置时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
  {
    setBsplineInterval(ts);

    double final_cost;
    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点control_points、时间间隔ts，调用rebound_optimize(final_cost)将轨迹推出障碍物，
  // 得到最优的无碰撞轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)
  {
    // 将时间间隔存储到成员变量
    setBsplineInterval(ts);

    cps_ = control_points;

    bool flag_success = rebound_optimize(final_cost);

    optimal_points = cps_.points;

    return flag_success;
  }

  // 设置初始控制点init_points、时间间隔ts，调用refine_optimize()重新分配时间，得到最优的动力学可行轨迹，并将其控制点赋值给optimal_points
  bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
  {

    // 将控制点数据存储到成员变量
    setControlPoints(init_points);
    setBsplineInterval(ts);

    bool flag_success = refine_optimize();

    optimal_points = cps_.points;

    return flag_success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到光滑、无碰撞、动力学可行、与其他无人机碰撞、结束项的轨迹。
  bool BsplineOptimizer::rebound_optimize(double &final_cost)
  {
    optimization_result_.reset(); optimization_reason_="not_started";
    iter_num_ = 0;
    int start_id = order_;
    // int end_id = this->cps_.size - order_; //Fixed end
    int end_id = planning_endpoints_ ? this->cps_.size-order_ : this->cps_.size;
    // 变量个数
    variable_num_ = 3 * (end_id - start_id);

    rclcpp::Time t0 = clock_->now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    ;
    bool flag_force_return, flag_occ, success;
    new_lambda2_ = lambda2_;
    constexpr int MAX_RESART_NUMS_SET = 3;
    do
    {
      if (budget_ && (budget_->expired() || initialization_failed_)) {
        optimization_reason_=budget_->expired() ? "budget_expired" : "initialization_failed";
        return false;
      }
      if ((restart_nums || rebound_times) && budget_ &&
          !budget_->tryRepair(PlanningBudget::Repair::BackendRestart)) {
        optimization_reason_="backend_restart_denied"; return false;
      }
      /* ---------- prepare ---------- */
      min_cost_ = std::numeric_limits<double>::max();
      min_ellip_dist_ = INIT_min_ellip_dist_;
      iter_num_ = 0;
      flag_force_return = false;
      flag_occ = false;
      success = false;

      physical_incumbent_.reset();physical_incumbent_cost_=std::numeric_limits<double>::infinity();
      // 控制点数组初始化
      double q[variable_num_];
      memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

      // 初始化L-BFGS算法的参数
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16; // 算法保存的历史优化步数
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.01;

      /* ---------- optimize ---------- */
      t1 = clock_->now();
      // 执行优化
      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      optimization_result_=result; optimization_reason_=lbfgs::lbfgs_strerror(result);
      t2 = clock_->now();
      double time_ms = (t2 - t1).seconds() * 1000;
      double total_time_ms = (t2 - t0).seconds() * 1000;

      /* ---------- success temporary, check collision again ---------- */
      // 收敛、达到最大迭代次数、已达到最小值或被停止，则进入碰撞检测阶段
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // The line-search callback owns the last evaluated trial, which need
        // not be physically admissible. A normal solve selects its lowest
        // original-cost admissible incumbent before the manager's full checks.
        if(physical_incumbent_) {
          cps_.points=*physical_incumbent_;final_cost=physical_incumbent_cost_;
        }
        // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
        flag_force_return = false;

        /*** collision check, phase 1 ***/
        if ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))
        {
          success = false;
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2; // 提高规避权重

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, swarm too close, keep optimizing\n\033[0m", iter_num_, time_ms);

          continue;
        }

        /*** collision check, phase 2 ***/
        // 创建均匀的B样条曲线
        UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
        double tm,tmp; traj.getTimeSpan(tm,tmp);
        if(planning_query_ && planning_endpoints_) {
          // The manager classifies the complete post-optimization curve and
          // performs a budgeted guide correction, including the final tail.
          flag_occ=false;
        } else {
          const double step=std::min(.02,grid_map_->getResolution()/(2*std::max(.1,max_vel_)));
          for(double t=0;t<=traj.getTimeSum()+step;t+=step) {
            if(budget_ && budget_->expired()) return false;
            flag_occ=planningOccupied(traj.evaluateDeBoorT(std::min(t,traj.getTimeSum())));
            if(flag_occ) break;
          }
        }

        // cout << "XXXXXX" << ((cps_.points.col(cps_.points.cols()-1) + 4*cps_.points.col(cps_.points.cols()-2) + cps_.points.col(cps_.points.cols()-3))/6 - local_target_pt_).norm() << endl;

        /*** collision check, phase 3 ***/
// #define USE_SECOND_CLEARENCE_CHECK
#ifdef USE_SECOND_CLEARENCE_CHECK
        bool flag_cls_xyp, flag_cls_xyn, flag_cls_zp, flag_cls_zn;
        Eigen::Vector3d start_end_vec = traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm);
        Eigen::Vector3d offset_xy(-start_end_vec(0), start_end_vec(1), 0);
        offset_xy.normalize();
        Eigen::Vector3d offset_z = start_end_vec.cross(offset_xy);
        offset_z.normalize();
        offset_xy *= cps_.clearance / 2;
        offset_z *= cps_.clearance / 2;

        Eigen::MatrixXd check_pts(cps_.points.rows(), cps_.points.cols());
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts.col(i) = cps_.points.col(i);
          check_pts(0, i) += offset_xy(0);
          check_pts(1, i) += offset_xy(1);
          check_pts(2, i) += offset_xy(2);
        }
        flag_cls_xyp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_xy(0);
          check_pts(1, i) -= 2 * offset_xy(1);
          check_pts(2, i) -= 2 * offset_xy(2);
        }
        flag_cls_xyn = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) += offset_xy(0) + offset_z(0);
          check_pts(1, i) += offset_xy(1) + offset_z(1);
          check_pts(2, i) += offset_xy(2) + offset_z(2);
        }
        flag_cls_zp = initControlPoints(check_pts, false).size() > 0;
        for (Eigen::Index i = 0; i < cps_.points.cols(); i++)
        {
          check_pts(0, i) -= 2 * offset_z(0);
          check_pts(1, i) -= 2 * offset_z(1);
          check_pts(2, i) -= 2 * offset_z(2);
        }
        flag_cls_zn = initControlPoints(check_pts, false).size() > 0;
        if ((flag_cls_xyp ^ flag_cls_xyn) || (flag_cls_zp ^ flag_cls_zn))
          flag_occ = true;
#endif

        // 如果没有检测到碰撞视为优化成功
        if (!flag_occ)
        {
          printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
          success = true;
        }
        // 如果有碰撞则重新初始化
        else // restart
        {
          restart_nums++;
          initControlPoints(cps_.points, false);
          new_lambda2_ *= 2;

          printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      // 如果优化被强制取消
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("rebound_optimize"),
                                        "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
        // while (rclcpp::ok());
      }

    } while (
        ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    optimization_reason_+=success ? ";accepted" : ";postcheck_or_rebound_rejected";
    return success;
  }

  // 使用L-BFGS方法对目标函数进行优化，得到重新分配时间后，光滑、拟合较好、动力学可行的轨迹。
  bool BsplineOptimizer::refine_optimize()
  {
    optimization_result_.reset(); optimization_reason_="not_started";
    iter_num_ = 0;
    int start_id = order_;
    int end_id = this->cps_.points.cols() - order_;
    variable_num_ = 3 * (end_id - start_id);

    double q[variable_num_];
    double final_cost;

    memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

    double origin_lambda4 = lambda4_;
    force_stop_type_ = DONT_STOP;
    bool flag_safe = true;
    int iter_count = 0;
    do
    {
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.max_iterations = 200;
      lbfgs_params.g_epsilon = 0.001;

      int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
      optimization_result_=result; optimization_reason_=lbfgs::lbfgs_strerror(result);
      if(!lastOptimizationTerminatedNormally()) return false;
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        // pass
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("refine_optimize"),
                                        "Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
      }

      if (budget_ && budget_->expired()) return false;
      // 使用优化后的控制点创建新的轨迹
      UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
      double tm, tmp;
      traj.getTimeSpan(tm, tmp);
      double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
      for (double t = tm; t < tmp * 2 / 3; t += t_step)
      {
        if (planningOccupied(traj.evaluateDeBoorT(t)))
        {
          // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

          // 将ref_pts存储为矩阵形式
          Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
          for (size_t i = 0; i < ref_pts_.size(); i++)
          {
            ref_pts.row(i) = ref_pts_[i].transpose();
          }

          flag_safe = false;
          break;
        }
      }

      // 如果存在碰撞则调整参数并重新迭代
      if (!flag_safe)
        lambda4_ *= 2;

      iter_count++;
    } while (!flag_safe && iter_count <= 0);

    lambda4_ = origin_lambda4;

    // cout << "iter_num_=" << iter_num_ << endl;

    optimization_reason_+=flag_safe ? ";provisional_physical_pass" : ";provisional_physical_rejected";
    return flag_safe;
  }

  // 计算损失
  void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
  {
    // cout << "drone_id_=" << drone_id_ << endl;
    // cout << "cps_.points.size()=" << cps_.points.size() << endl;
    // cout << "n=" << n << endl;
    // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_distance, f_feasibility /*, f_mov_objs*/, f_swarm, f_terminal;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    // Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal = Eigen::MatrixXd::Zero(3, cps_.size);

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    // calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);
    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);

    double f_guide = 0.0;
    Eigen::MatrixXd g_guide = Eigen::MatrixXd::Zero(3, cps_.size);
    if (guide_tracking_ && ref_pts_.size() >= static_cast<size_t>(cps_.size))
      calcFitnessCost(cps_.points, f_guide, g_guide);

    f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_swarm + lambda2_ * f_terminal + lambda4_ * guide_weight_ * f_guide;
    // f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;
    // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_swarm + lambda2_ * g_terminal + lambda4_ * guide_weight_ * g_guide;
    // Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

  // 计算优化后的损失
  void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
  {

    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* ---------- evaluate cost and gradient ---------- */
    double f_smoothness, f_fitness, f_feasibility;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    // time_satrt = clock_->now();

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    double f_physical=0;
    Eigen::MatrixXd g_physical=Eigen::MatrixXd::Zero(3,cps_.points.cols());
    calcCurvePhysicalCost(cps_.points,f_physical,g_physical);

    /* ---------- convert to solver format...---------- */
    f_combine = lambda1_ * f_smoothness + lambda4_ * guide_weight_ * f_fitness + lambda3_ * f_feasibility + lambda2_ * f_physical;
    // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * guide_weight_ * g_fitness + lambda3_ * g_feasibility + lambda2_ * g_physical;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
  }

} // namespace ego_planner
