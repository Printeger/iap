#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <rclcpp/rclcpp.hpp>
#include "bspline_opt/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{
  struct LocalTarget {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double progress_m = 0.0;
  };

  class ControlPoints
  {
  public:
    double clearance;
    int size;
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }

    void segment(ControlPoints &buf, const int start, const int end)
    {

      if (start < 0 || end >= size || points.rows() != 3)
      {
        RCLCPP_ERROR(rclcpp::get_logger("segment"), "Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.clearance = clearance;
      buf.size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];

        // if ( buf.base_point[i - start].size() > 1 )
        // {
        //   ROS_ERROR("buf.base_point[i - start].size()=%d, base_point[i].size()=%d", buf.base_point[i - start].size(), base_point[i].size());
        // }
      }

      // cout << "RichInfoOneSeg_temp, insede" << endl;
      // for ( int k=0; k<buf.size; k++ )
      //   if ( buf.base_point[k].size() > 0 )
      //   {
      //     cout << "###" << buf.points.col(k).transpose() << endl;
      //     for (int k2 = 0; k2 < buf.base_point[k].size(); k2++)
      //     {
      //       cout << "      " << buf.base_point[k][k2].transpose() << " @ " << buf.direction[k][k2].transpose() << endl;
      //     }
      //   }
    }
  };

  class BsplineOptimizer
  {
    friend struct BsplineOptimizerTestAccess;

  public:
    struct SearchFailureContext {
      std::string stage;
      std::vector<Eigen::Vector3d> control_points;
      int segment_start = -1;
      int segment_end = -1;
    };
    struct RepairEndpoints {
      Eigen::Vector3d entry, exit;
      std::vector<Eigen::Vector3d> prefix, suffix;
    };
    std::optional<RepairEndpoints> chooseRepairEndpoints(
        const Eigen::MatrixXd& points, int segment_start, int segment_end,
        AStar::Failure& failure) const;
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}
    std::optional<int> lastOptimizationResult() const { return optimization_result_; }
    const std::string& lastOptimizationReason() const { return optimization_reason_; }
    bool lastOptimizationTerminatedNormally() const {
      if(!optimization_result_) return false;
      const int result=*optimization_result_;
      return result==lbfgs::LBFGS_CONVERGENCE || result==lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result==lbfgs::LBFGS_ALREADY_MINIMIZED || result==lbfgs::LBFGS_STOP;
    }

    /* main API */
    void setEnvironment(const GridMap::Ptr &map);
    void setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj);
    void setParam(rclcpp::Node::SharedPtr node);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    rclcpp::Clock::SharedPtr clock_;

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);

    // optional inputs
    void setPlanningBudget(PlanningBudget::Ptr budget) {
      budget_ = std::move(budget);
      a_star_->setPlanningBudget(budget_);
    }
    void setPlanningEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
      planning_endpoints_ = std::make_pair(start, end);
      planning_goals_ = {end}; planning_goal_center_.reset();
    }
    void setPlanningGoals(const std::vector<Eigen::Vector3d>& goals,
        std::optional<Eigen::Vector3d> center = std::nullopt) { planning_goals_ = goals; planning_goal_center_ = center; }
    // Frozen physical volume; actual cubic extrema are constrained in both solvers.
    void setCurvePhysicalBounds(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper) {
      curve_bounds_ = std::make_pair(lower, upper);
    }
    bool searchRecoveryGuide();
    struct RecoverySearchEvidence {
      GridExecutionReason start_reason = GridExecutionReason::OK;
      GridAdvisoryClass start_advisory = GridAdvisoryClass::UNKNOWN;
      bool normal_attempted = false, fallback_eligible = false, fallback_entered = false;
      AStar::Failure normal_failure = AStar::Failure::NONE, final_failure = AStar::Failure::NONE;
      double initial_remaining_s = 0., fallback_remaining_s = 0.;
      bool guide_found = false;
    };
    const std::vector<RecoverySearchEvidence>& recoverySearchEvidence() const { return recovery_search_evidence_; }
    void strengthenGuideTracking() { guide_weight_ *= 2.0; }
    // Constrain violating actual samples toward the one existing legal guide.
    // This is a route preference; final physical/motion checks remain independent.
    bool addCurveGuideConstraints(const Eigen::MatrixXd& points, double interval,
                                 bool route_loss = false, bool risk_preference_loss = false);
    struct GuideRetention {
      bool checked = false, budget_exhausted = false;
      bool comparable_valid_risk = false, route_lost = false, risk_preference_lost = false;
      bool comparable_model_cost = false;
      uint64_t risk_version = 0;
      double guide_length_m = 0, curve_length_m = 0;
      double guide_risk_cost_m = 0, curve_risk_cost_m = 0;
      double guide_valid_fraction = 0, curve_valid_fraction = 0;
      double max_deviation_m = 0, corridor_m = 0, quadrature_uncertainty = 0;
      size_t samples = 0;
    };
    // Diagnostic frozen raw Advisory in OFF and ON; this grants no physical authority.
    GuideRetention assessGuideRetention(const Eigen::MatrixXd& points, double interval,
        const std::function<GridPlanningRisk(const Eigen::Vector3d&)>& advisory) const;
    bool addCurveClearanceConstraints(const Eigen::MatrixXd& points, double interval,
        const std::vector<std::pair<double,GridPlanningCell>>& violations);
    bool curveViolates(const Eigen::MatrixXd& points, double interval) const;
    bool needsGuideReinitialization() const { return guide_reinitialization_; }
    const vector<Eigen::Vector3d>& recoveryGuide() const { return guide_pts_; }
    void initializeFromGuide(const Eigen::MatrixXd& points);
    // Same uniform knot/control indexing and frozen planning input only.
    void rebindAfterUniformRetime(const Eigen::MatrixXd& points);
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setPlanningQuery(std::function<GridPlanningCell(const Eigen::Vector3d&)> query,
                          bool advisory_fallback = false,
                          std::function<GridPlanningCell(const Eigen::Vector3d&)> guide_query = {}) {
      planning_query_ = std::move(query);
      guide_query_=guide_query ? std::move(guide_query) : planning_query_;
      planning_advisory_fallback_ = advisory_fallback;
      initialization_failed_ = false;
      guide_tracking_ = false;
      guide_reinitialization_ = false;
      planning_endpoints_.reset();
      planning_goals_.clear();
      recovery_search_evidence_.clear();
      guide_pts_.clear();
      guide_weight_ = 1.0;
      curve_clearance_constraints_.clear();
      curve_bounds_.reset();
      if (a_star_) a_star_->setPlanningQuery(guide_query_, advisory_fallback);
    }
    void setSearchFailureObserver(std::function<void(
        const AStar::Result&, const SearchFailureContext&)> observer) {
      search_failure_observer_ = std::move(observer);
    }
    bool initializationFailed() const { return initialization_failed_; }
    bool advisoryFallbackUsed() const { return planning_advisory_fallback_; }
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most
    void setLocalTargetPt(const Eigen::Vector3d local_target_pt) { local_target_pt_ = local_target_pt; };

    void optimize();

    ControlPoints getControlPoints() { return cps_; };

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<ControlPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);
    std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts);
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }
    inline double getSwarmClearance(void) { return swarm_clearance_; }

  private:
    std::function<void(const AStar::Result&, const SearchFailureContext&)>
        search_failure_observer_;
    void reportSearchFailure(const AStar::Result& result,
                             const Eigen::MatrixXd& points,
                             int segment_start, int segment_end,
                             const char* stage) const {
      if (!search_failure_observer_) return;
      SearchFailureContext context;
      context.stage = stage;
      context.segment_start = segment_start;
      context.segment_end = segment_end;
      for (int i = 0; i < points.cols(); ++i)
        context.control_points.emplace_back(points.col(i));
      search_failure_observer_(result, context);
    }
    GridMap::Ptr grid_map_;
    PlanningBudget::Ptr budget_;
    std::vector<Eigen::Vector3d> planning_goals_;
    double guide_weight_ = 1.0;
    enum class CurveConstraintKind { ClearancePlane, GuideCorridor };
    struct CurveClearanceConstraint {
      int first_control;
      Eigen::Vector4d weights;
      Eigen::Vector3d center, direction;
      double clearance;
      CurveConstraintKind kind = CurveConstraintKind::ClearancePlane;
    };
    std::vector<CurveClearanceConstraint> curve_clearance_constraints_;
    std::optional<std::pair<Eigen::Vector3d,Eigen::Vector3d>> curve_bounds_;
    std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>> planning_endpoints_;
    bool guide_reinitialization_ = false;
    std::function<GridPlanningCell(const Eigen::Vector3d&)> planning_query_;
    std::function<GridPlanningCell(const Eigen::Vector3d&)> guide_query_;
    bool planning_advisory_fallback_ = false;
    std::vector<RecoverySearchEvidence> recovery_search_evidence_;
    bool initialization_failed_ = false;
    std::optional<Eigen::Vector3d> planning_goal_center_;
    bool guide_tracking_ = false;
    bool planningOccupied(const Eigen::Vector3d& position) const {
      if (!planning_query_) return grid_map_->getInflateOccupancy(position) != 0;
      const auto cell = planning_query_(position);
      if (!cell.executable()) return true;
      const auto cls = cell.advisory.classification;
      return !planning_advisory_fallback_ &&
          (cls == GridAdvisoryClass::AVOID ||
           cls == GridAdvisoryClass::PREDICTED_DEGRADED);
    }
    fast_planner::ObjPredictor::Ptr moving_objs_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    int drone_id_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_ = .1; // B-spline knot span
    Eigen::Vector3d end_pt_;  // end of the trajectory
    // int             dim_;                // dimension of the B-spline
    //
    vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
    vector<Eigen::Vector3d> waypoints_; // waypts constraints
    vector<int> waypt_idx_;             // waypts constraints index
                                        //
    int max_num_id_, max_time_id_;      // stopping criteria
    int cost_function_;                 // used to determine objective function
    double start_time_;                 // global time for moving obstacles

    /* optimization parameters */
    int order_;                    // bspline degree
    double lambda1_;               // jerk smoothness weight
    double lambda2_, new_lambda2_; // distance weight
    double lambda3_;               // feasibility weight
    double lambda4_;               // curve fitting

    int a;
    //
    double dist0_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_;       // dynamic limits

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    Eigen::Vector3d local_target_pt_;

#define INIT_min_ellip_dist_ 123456789.0123456789
    double min_ellip_dist_;

    ControlPoints cps_;
    std::optional<int> optimization_result_;
    std::string optimization_reason_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcCurvePhysicalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize(double &final_cost);
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benckmark evaluation only */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif
